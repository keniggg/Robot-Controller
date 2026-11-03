#!/usr/bin/env python3
"""Write a bounded, evidence-backed gate policy audit; no runtime mutations."""
import hashlib
import json
from pathlib import Path

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
EXT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N = EXT/'ros_log/full_latest_contact_admission_20261013T011103Z'
R = EXT/'grasp_records/20261013T094326.672604Z_0b39e4a7c8514b43b66ae320b8aaeea8'
O = N/'verification/gen16_gate_rationality_audit'
M = 'src/alicia_grasp_modes/src/alicia_grasp_modes/'
B = 'src/alicia_flexible_grasp_supervisor/scripts/'
P = 'src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py'
CFG = 'src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml'

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def row(gid, stage, name, value, refs, purpose, consequence, judgement, recommendation, retain, evidence):
    return dict(id=gid,stage=stage,name=name,actual_or_configured_value=value,
        source_locations=refs,purpose=purpose,current_consequence=consequence,
        classification=judgement,recommendation=recommendation,
        execution_requirements_to_retain=retain,r4_evidence=evidence)

rows = [
 row('initial_ack','initial','初始预检 ACK 等待',
     '源码固定 min(token 剩余寿命, 0.8 s)；此次提取未包含 initial ACK 原文，不能宣称其实际耗时。',
     [M+'initial_preflight_task.py:76-97'],
     '协调两个节点是否承认同一初始观测 token；0.8 s 是通信/调度等候上限。',
     '等不到 ACK 抛 INITIAL_PREFLIGHT_ACK_TIMEOUT；pending commit 是 _StillPending。',
     '可改为等待/警告；ACK 的存在与绑定仍是提交前提。',
     '将通信超时与源有效期分离：在 owner/token 仍有效时继续等待；接近源过期则获取新观测并以新 token 重验。不要把 ACK 缺失变成 accepted。',
     '真实相同 token、request、控制 epoch、当前反馈与几何；过期 token 不可直接复用。',
     'R4 越过初始预检并实际执行唯一一次观测位移动，本轮未在此失败。'),
 row('session_ack','observation/supplement','补视 session ACK 等待',
     'task 固定 5 s；R4 第二 view 请求至任务失败 5.041967630 s，remote drain 约 5.965 s。',
     [M+'observe_return_task.py:720-758',M+'observe_return_remote.py:1080-1181'],
     '在 observation session 切换前排空旧 worker，确认同一视角、目标与 owner。',
     '等待本身直接终止任务 OBSERVE_RETURN_REMOTE_ACK_TIMEOUT。R4 迟到 ACK 为 TARGET_CHANGED。',
     '这是等待策略，不是碰撞或姿态安全量；应优先变为显式 WAITING/警告。',
     '先修 pause/worker 状态交接，使旧工作可靠取消。5 s 改进度提示；在 session/phase/control owner 仍有效且无故障时允许继续等待，并用有上限的进度 watchdog 防永久挂起。若仍使用数值阈值，应按实测 drain/RPC 上界设定，不能仅凭本轮 5.965 s 拍定新秒数。',
     'ACK accepted 且逐字段绑定相同请求后才允许后继动作；等待中用户切换模式或撤销立即退出；提交前重新验证源、SDK、目标与路径。',
     'wire1001 请求、1047 超时失败、1050 阶段退出、1086 迟到拒绝。原相位期限还剩约33.29 s，总session还剩506.28 s。'),
 row('measurement_ack','observation','运动前测量 lease 双 ACK',
     '非 qualified min(5 s, session 剩余)；qualified 使用 capture 剩余。',
     [M+'observe_return_task.py:130-169',M+'observe_return_measurement.py:15-26'],
     'producer 与 remote 在已绑定观测运动前共同承认冻结/测量租约。',
     '缺少/错误 ACK 分别为 ACK_TIMEOUT、绑定错误；不会把一方 ACK 当成三方成功。',
     '5 s 可等待优化；双参与者实际接受不可降为警告后运动。',
     '区分明确拒绝和暂未返回；未返回时等待并保持运动未提交，超过 owner 寿命则重新准备新 lease。',
     '双方独立校验同 request digest、source 与 owner，不用成功 stub 或单方响应替代。',
     '同一 lease 的 executing/requalifying/qualified 共8条 ACK；R4本阶段成功。'),
 row('capture','qualification','到位后三帧资格采集窗口',
     '默认 3 s，代码允许 >0 且 <=10 s；R4 原请求窗口恰3 s，0.831854343 s发出 qualified。',
     [M+'observe_return_task.py:261-289',M+'observe_return_measurement.py:135-145'],
     '在实测停稳后得到相同目标的独立新测量，不把运动中或旧图像当到位测量。',
     '窗口结束仍未三源匹配时 QUALIFICATION_TIMEOUT。',
     '总采集等待可延长/有限重采；每张源与控制稳定性仍须真实满足。',
     '负载导致未凑齐时优先延长有界等待或重新开一次新 capture，丢弃旧 partial；不能把3 s失败解释为目标已丢失，也不能靠重写时间戳续命。当前R4不需要调大此值。',
     '每张RGBD新鲜、精确TF、同目标/控制epoch、真实accepted停稳；三方 source 列表一致。',
     'wire563,575,578,580,582,583；三源 4655.902715921/4656.168704509/4656.370764493。'),
 row('fresh_source','qualification/contact/commit','源帧/固定站位几何新鲜度',
     '当前 source <=0.8 s，多处精确800000000 ns；R4有效191帧最大0.460143566 s，失败帧仅0.076583624 s。',
     [P+':3499-3562',M+'observe_return_task.py:374-384',M+'station_compute_reacquisition.py:60-117'],
     '限定用于当前几何、反馈关联的真实测量年龄；不是总计算工作量上限。',
     '不新鲜时不得继续作为当前metric geometry；已有带绑定的身份等待协议可进入PENDING并请求新数据。',
     '不能直接把过期几何降警告授权动作；可把等待旧计算结束与当前提交证据分离。',
     '允许较长离线/后台计算，结果提交前用当前RGBD/SDK重新验证注册、目标、支撑与轨迹；若0.8 s以后需调整，应以运动/深度不确定度与实测稳定性证明，不能只为通过测试放宽。过期应触发重采而非一律任务永久失败。',
     '运动提交必须有可证明当前的目标/位置/碰撞模型，不沿用旧几何冒充新帧。',
     '本轮没有0.8 s超时；时间线直接排除此门为首次失败根因。'),
 row('identity_pending','contact waiting','身份等待/PENDING与空输入',
     'station身份等待=min(source+5 s, phase, session)，几何寿命另为<=0.8 s；EMPTY不等于PENDING。',
     [M+'station_compute_reacquisition.py:1-15',M+'station_compute_reacquisition.py:60-117',M+'station_compute_wait_task.py:25',M+'station_compute_wait_remote.py:29',P+':3367-3397'],
     '没有新几何时保留经过登记的身份以等待重采，而不制造几何或执行权限。',
     '有效PENDING撤销可执行几何后等待；没有合同的空mask/object/错目标不能自动当可恢复pending。',
     '数据暂未就绪应WAITING，不能假通过，也不应无条件永久first_loss。',
     '将missing/processing/已知撤销分类；对同owner、实测静止且反馈在更新的短缺口走既有PENDING/重新测量，超期重建带证据的合同。真正目标切换/没有可验证身份则停止当前动作授权。',
     '空输入不生成物体或路径；没有真实新测量不得把身份合同升级几何权限。',
     'wire618/619曾发布TARGET_PROCESSING_PENDING，随后191 READY持续；撤销前不存在视觉失效。'),
 row('phase_budget','contact search','近场phase总预算与总session预算',
     'YAML near_field_replan_timeout_sec=90 s；多处缺配置fallback60 s，不是当前60 s。R4 wire相位89.996246099 s，总session600 s。',
     [CFG+':200-207',M+'observe_return_task.py:363-404',B+'remote_grasp6d_node.py:3458-3468',M+'observe_return_session.py:74-85'],
     '为一段owner/候选计算指定有界期限，防旧任务无限运行；90/600本身是作业调度策略。',
     '相位过期撤销ticket/selection；各方会拒绝旧header/deadline。',
     '可以统一配置延长预算，但不能各节点私自改已发布合同期限。',
     '成功优先时可预先设更长phase或设计明确续期事务：三方一致、仍静止/同目标、当前测量和SDK重新确认、旧结果不得跨新owner偷渡。R4在原90 s内仅57 s左右即失败，先修5 s交接而非先增phase。',
     '提交时实际owner、期限和目标仍匹配；已过期的计划不得仅改本地deadline复活。',
     'wire587 phase start1791884656624420881/deadline1791884746620666980；wire1001 supplement时还有38.335457086 s。'),
 row('two_family','contact search','两原始候选family提前让路',
     'hard-coded maximum_original_families=2；R4 shortlist43/check4，两原始family完成失败，14.348057508 s让路。',
     [M+'observe_return_remote.py:182-205',M+'observe_return_remote.py:267-286',M+'observe_return_remote.py:319-343'],
     '在有实测补视收益机会时提早转换视角，减少接触搜索耗时；两例失败不能证明剩余39条未查候选不可行（43条shortlist减4条checked，候选条目不等于family）。',
     '证据满足即CONTACT_SEARCH_YIELDED_FOR_MEASURED_VIEW，有效yield直接返回，未检查尾部不进入fallback。',
     '效率启发式，应改调度建议，优先成功率。',
     '两例失败仅记录告警/搜索里程碑；优先尝试同站位合法高度调整及剩余候选，再根据剩余预算与可测补视收益决定是否换视角。必须保留unchecked状态，不能把未查候选写成碰撞失败。',
     '每个最终候选仍完整通过几何、IK/关节限制、连续碰撞、真实控制反馈与执行检查；失败的那两条不能放行。',
     'R4 audit4ee7… failed_search：family(2,0)/(4,0)，无incumbent，phase仍余54.812618394 s。'),
 row('height_defer','contact search','高度替代被让路策略推迟',
     '在当前yield state无incumbent/nonfallback且有补视机会时，所有 height retry 返回None，标记deferred。',
     [M+'observe_return_remote.py:297-318',M+'observe_return_remote.py:319-368'],
     '先给不同原始方向首次检查机会，避免同一族消耗预算；属于搜索次序。',
     '若两族yield成立，deferred高度与tail都不会进入fallback；否则fallback在原phase内尝试。',
     '效率次序可改，应取消无条件优先补视。',
     '先对有明确可行高度区间的当前站位候选做有界重居中/高度微调并完整重验。用实际剩余预算比较“本地修正成本”和“补视移动+停稳+重采成本”；保留独立方向探索，避免完全耗尽补视窗口。',
     '高度区间真实非空、调整后需重新检查指尖/目标/桌面/其他部件与完整路径；空区间、真实碰撞不得转warning后执行。',
     '原R4 audit保存未查替代与最终高度拒绝；独立reviewer负责具体最终注册高度修正。'),
 row('thirty_sec','contact search','接触搜索30 s子预算',
     'deadline=min(start+30 s, original_phase-2 s)；R430 s未耗尽即两族提前yield。',
     [M+'observe_return_remote.py:192-201',M+'observe_return_remote.py:276-286',M+'observe_return_remote.py:287-295',M+'observe_return_remote.py:338-365'],
     '为补视留出时间的局部调度检查点，不是物理碰撞界限。',
     '无incumbent可停继续候选；无合法yield时已有一次fallback回原phase。',
     '可转检查点/警告，而不是把30 s当全任务失败证据。',
     '30 s触发策略评估：还有合法高度修正/未查候选则可在原90 s owner预算内继续。旧phase预留生成补视计划、发布/ACK和交接时间；总session另预留移动+停稳+capture+新contact搜索时间。不要将补视全程错误塞进旧phase，也不能默默延长旧合同。',
     '期间持续确认控制/目标/反馈；最终以当前几何重新验证；计算超时不得记成碰撞结论。',
     'R4 yielded1791884691808048486，子deadline1791884707459990978，原phase deadline1791884746620666880（审计float回转，实际wire末尾6980）。'),
 row('incumbent','contact search/publication','已合格候选后的继续比较',
     '已有8 s附加比较上限、2 s发布预留；单resolver4 s；nearfield exhaustive=True。',
     [B+'remote_grasp6d_node.py:699-704',B+'remote_grasp6d_node.py:10995-11043',B+'remote_grasp6d_node.py:17070-17155'],
     '寻找更优开口/路径/时长的候选；这是质量优化，而非拥有候选后的必需物理条件。',
     '当前可保留提前结束时的selected，但若后处理/RPC越过绝对phase，最终会清selected。',
     '成功优先应尽早完成提交重验；不要为比较牺牲可用候选。',
     '首个完整严格合格candidate即停止附加搜索并完成当前重验/发布；若保留比较，则要求下一次RPC最坏时长+全部后处理余量仍足。不要简单设置first_reachable_by_rank=True，因为它与exhaustive API互斥且原ranking不等同执行最终rank。',
     'owner已失效或最新重验失败时仍不得用旧incumbent运动；应重采重验。',
     'R4 request16明确incumbent_found=false，不能宣称本轮丢掉了已可执行方案；这是同链潜在风险。'),
 row('view_budget','supplement','观测总次数',
     '代码允许1..3，YAML默认3；R4原session实际max_observations=2，正在申请view2即失败。',
     [M+'active_observation_policy.py:11-21',M+'observe_return_session.py:58-65',M+'observe_return_session.py:81-99',CFG+':203',M+'observe_return_remote.py:945-961'],
     '限制重复观测与整体运行时间；次数本身不保证或否定几何可执行。',
     '达到上限后不再补视。次数是session immutable field，不能中途单边改。',
     '可配置的成功率/时间取舍，不是固定安全常数。',
     '下一新session可用已支持的3次上限，但应以实测信息增益、视角不同、可达路径与剩余总预算决定；不要将2次结束等价“所有抓取方法不可行”。需要>3必须协调协议/owner与验证，不能只改一个参数。',
     '每次观测移动仍需真实路径/控制确认，新增视角不得替代碰撞证明；现有session绑定不被热改。',
     'wire176/1001与audit4ee7均证明effective=2；本轮不是耗尽次数失败。未读取live参数，2与YAML3的来源未归因。'),
 row('candidate_unknown','contact search','候选RPC超时/EMPTY/PENDING',
     '不同字段分别表示没有候选、当前计算未完成或明确否决；R4 checked4/43不能解读全不可达。',
     [M+'observe_return_remote.py:235-258',M+'observe_return_remote.py:275-285','src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/grasp6d_pipeline.py:991-1020'],
     '保存“已证明失败”和“尚未得出结论”的区别。',
     '只有绑定完整失败receipt计completed family；RPC超时不会成为碰撞证明，但会耗预算。',
     '计算未知可警告、继续/重试其他候选；实际不合格候选仍禁止运动。',
     '独立记录 timeout/unknown/unchecked；允许有界重试并选择别的完整可行候选。空列表由测量暂缺导致时等待新源；只有完整搜索已检查全部候选才可声明全不可行。',
     '不能把pending或timeout转成planning_success；服务回来的旧owner结果作废。',
     'R4 两碰撞receipt是真实否决，但其余尾部和高度替代未充分探索。'),
 row('abort_cleanup','cancellation','任务结束时旧帧失效升级 first_loss',
     'R4 FAILED后54.62 ms producer first_loss；帧仍新鲜，station_compute_retirement_context_changed。',
     [M+'station_compute_wait_producer.py:245-279',M+'station_compute_wait_producer.py:360-368',P+':1964-1982',P+':3575-3581',P+':3367-3397'],
     '不让被撤销owner的旧帧继续产生有效几何；撤销本身是合理的。',
     '旧帧token不匹配被升级为perception_failure/target_lost，造成后续TARGET_CHANGED掩盖原ACKtimeout。',
     '已知取消应中性retired/discard，错误级别可降；不应制造新的视觉根因。',
     '单独区分明确任务终止/阶段撤销与真正目标/姿态改变：前者丢弃在途结果、保留原失败归因和审计；不发布geometry、不恢复执行权限、不假装目标仍被测量确认。',
     '真实控制权/目标/运动变化仍撤销权限；可恢复必须按新任务/测量合同。',
     'causal_analysis SHA d37aec24…；wire1047/1050/1052先于producerfailure1056，最后READY1045。'),
 row('geometry_action','execution','真实碰撞、错误目标、控制权、反馈缺失',
     '不是统一时间阈值；动作承诺需当前plan digest/owner/target/control epoch、实际SDK/accepted与连续路径几何。',
     [B+'grasp_task_node.py:7137-7220',M+'contact_station_task.py:45-54',M+'observe_return_task.py:975-1009',M+'observe_return_measurement.py:83-120'],
     '防错误目标/错误姿态/未知控制状态下提交动作，以及实际碰撞。',
     '当前候选或动作必须拒绝；允许安全重规划，不能用warning换取执行。',
     '真实运动边界必须保留；启发式阈值与真实碰撞要逐项区分。',
     '把名义高度/中心偏差等可修正问题转为重居中候选并完整重验；真实空可行域、支撑/手眼无效、连续碰撞、错目标、无新反馈仍不提交。硬门不应把某一个候选失败变成停止整个剩余候选搜索。',
     '全动作前保持原子owner检查、路径与反馈校验；不得温度查询或关节失能。',
     'R4唯一goal为已完成观测移动，无handgoal；contact候选未获可执行计划。')
]

DESIGN = r'''# 最小成功率优先策略设计（未实现）

先完成正在隔离验证的 pause/worker 并发修复与最终注册高度重居中，再决定合入这组独立策略。修改搜索次序不会把碰撞候选变为可执行方案；R4 没有已合格 incumbent，以下第1项是预防后续同链损失。

1. **首个完整合格候选尽快发布。** 在 `RemoteGrasp6DNode._direct_near_field_comparison_search` 中，保留当前“reachable/collision_free/within_joint_limits/ik_valid/planning_success 全真且有限代价”的精确判定。第一次满足后记录该 candidate 身份及严格证据，令继续比较函数立即返回 False（仍先校验 ticket/phase）。`bounded_moveit_select` 保留已核 candidate 和显式 unchecked tail；无需把互斥的 `first_reachable_by_rank` 与 `exhaustive=True` 强凑在一起。选择结果通过当前重建、注册/几何/SDK和发布检查后发布；审计写“首个完整合格”，不声称全局最优。超时/撤销后不得靠缓存 incumbent 直接运动。

2. **两族完成改成里程碑，取消无条件 height defer。** 修改 `ObserveReturnRemoteMixin._contact_height_window_retry`，当前站位存在合法高度可行域时允许进入原 bounded super retry；仍保留原真实碰撞/IK/反馈重验。`continuing` 不再因为 `len(original_families)>=2` 就返回 False；两族完成只产生 progress/warning。原方向探索和高度替代均保留身份与“未检查/已否决”区分，随后继续同站位剩余候选。若所有可行高度为空仍可转别的方向/有证据的补视，不能把margin失败改成warning后执行。

3. **30秒是决策检查点，使用两个不同时间池。** 原 contact phase 的精确整数期限 `D_phase` 不变；session 的 `D_session` 也不变。在开始搜索及每个重检查完成后维护一个静态不增长的最晚让路时刻 `D_yield = D_phase - R_handoff`。其中 `R_handoff` 至少覆盖补视规划/路径预检的实际有界上限、发布、ACK排空与原owner交接、误差余量；同时还需满足 `D_session-now > R_handoff + R_move_settle_capture + R_new_contact`。补视成功到位后源码会建立新的 qualified contact phase（task:1043-1065），所以不应把全部第二次移动和新contact的90秒都错误扣在旧phase里。

   `R_move_settle_capture` 使用实际已验证 timed path 时长及其执行超时上限、真实停稳窗口/5秒停稳上限、capture预算与注册成本；执行时长 lower-bound 或上一轮均值不能冒充上界。补视计划未形成时用明确配置的有界规划预检上限估算，不能捏造零成本；没有可信上限则不要宣称“已保证预留足够”。R4初次观测移动约34.6秒而第二视角计划约14.5秒计算，这证明只留固定2秒不足以代表补视全过程；实际预留值应由已有服务上限/阶段取证给出。

   到30秒：若有完整incumbent走第1项；否则若 `now + 下一候选最坏成本 < D_yield` 且有合法同站位height/tail，继续。若到了 `D_yield` 且有真实缺失表面、已注册信息增益、不同可达新视角和剩余session预算，发新typed调度让路证据并补视。若补视证据不成立，明确记录补视不可用，剩余原phase可用于同站位搜索至原发布预留线；不能将所有候选未查标成已失败。已无足够当前期限时停当前owner并按明确的新测量/新phase事务恢复，不修改已发布期限。不能简单关闭yield然后任fallback耗尽90秒。

4. **typed proof一并版本化，旧历史严格兼容。** `contact_search_yield.py` 当前 validator 对 `maximum_duration_sec==30`、`maximum_original_families==2`、`len(original_families)==2` 和原因值有固定wire语义。保留缺版本的旧v1分支原封不动用于历史校验。新调度使用显式 `proof_version=2`/新policy，包含原request/generation/target/source/phase绑定、`selected=false`、`reachable_count=0`、checked/unchecked/shortlist与height尝试/完成计数、真实失败receipt、原phase/session期限、固定 `latest_yield_ns`、预留分项及依据digest、30秒checkpoint和真实决策原因。`maximum_original_families=2`不可继续伪称新策略总上限；实际完成族可多于2，不能把数组截成2掩盖其他尝试。

   producer端writer、`_contact_yield_proof`、task `_observe_return_supplement_report`、`runner_contract.py` 两处调用共享按版本校验；未知版本拒绝，不用“更宽容”的fallback解释。调度证据只表示这次搜索未穷尽而主动让路，绝不表示所有方向碰撞或新动作已授权。真实表面缺口/注册增益/同目标/新视角路径仍是独立准入条件。应明确将调度证据与每条真实collision receipt分开；新策略即使没有那条碰撞证据也不能伪造碰撞，若要支持此类让路需独立的版本化理由和实测缺口验证。

5. **最小必要验证。** 用R4实际43候选/checked4/两条collision receipt重放，证明第2族后还能进入合法height/tail，而非靠放宽碰撞拿到成功。加首个完整合格后不开始新RPC、在途迟到/owner改变仍不发布、时间不足时按预算及时补视、不能无限更新预留/旧deadline、补视机会消失则继续明确未查尾部的反例。旧v1原R4 proof仍严格通过；篡改family计数、混代请求、缺新proof预算依据必须失败。只有软件通过，不能声称已真实夹持。
'''

def main():
    O.mkdir(exist_ok=False)
    wire_path=N/'verification/attempt4_producer_timeline_r1/wire.json'
    wire=json.loads(wire_path.read_text())
    audit=R/'keyframes_audits/4ee7be07d72f30b5d182127ad5b401d86ad56a432181a54e75f9048239746065.json'
    assert sha(audit)==audit.stem
    search=json.loads(audit.read_text())['observe_return']['failed_search']
    source_names=sorted({location.rsplit(':',1)[0] for r in rows for location in r['source_locations']} |
                        {M+'contact_search_yield.py',M+'runner_contract.py'})
    sources={name:sha(W/name) for name in source_names}
    evidence=dict(wire=str(wire_path),wire_sha256=sha(wire_path),
        causal_analysis=str(N/'verification/attempt4_producer_timeline_r1/causal_analysis.json'),
        causal_analysis_sha256=sha(N/'verification/attempt4_producer_timeline_r1/causal_analysis.json'),
        r4_contact_search_audit=str(audit),r4_contact_search_audit_sha256=sha(audit),
        r4_contact_search_summary=search,
        effective_configuration_from_wire=dict(max_observations=wire[176]['message']['max_observations'],
          capture_ns=wire[563]['message']['capture_deadline_ns']-wire[563]['message']['arrival_stamp_ns'],
          phase_wire_ns=wire[587]['message']['deadline']-wire[587]['message']['header']['stamp'],
          session_ns=int(wire[176]['message']['deadline_ns'])-int(wire[176]['message']['stamp_ns'])))
    design=O/'minimal_policy_design.md';design.write_text(DESIGN)
    report=dict(schema=1,kind='Read-only grasp-chain deadline and hard-gate rationality audit',
        user_priority='先保证抓取成功率，再提高效率；排查时间限制与可转警告的门限',
        result='audit_complete_no_runtime_changes',scope='R4 initial -> observation -> qualification -> contact search -> supplemental handoff -> execution boundaries',
        findings=rows,evidence=evidence,production_sha256=sources,
        minimal_policy_design=str(design),minimal_policy_design_sha256=sha(design),
        recommendations_in_priority_order=[
          '修复session/worker交接状态竞态；5 s从无条件失败改有owner约束的等待/进度告警，ACK有效前不执行。',
          '取消两family与height-defer导致的过早换视角；先尽力尝试同站位合法高度调整和尾候选，保留真实失败证据。',
          '30 s改搜索策略检查点；用剩余原phase分配本地搜索、发布与补视时间，必要时明确新phase事务。',
          '首个完整可执行候选出现后停止非必需比较、尽快当前重验和发布；不要让优化比较耗尽候选寿命。',
          '采集/PENDING/已知取消用等待、重采、retired分类；不把已取消旧帧升级为新视觉丢失。',
          '下一新session可采用已支持max3；任何新增动作仍完整审核目标、控制权、真实反馈与连续碰撞。'],
        limitations=['No live ROS parameter queries or hardware actions. Actual values are differentiated from source defaults; YAML max3 is not misreported as R4 effective max2.',
          'This is a bounded audit of the R4 grasp path, not a claim to enumerate every gate in all ROS nodes.',
          'No blanket threshold relaxation is implemented; retaining actual collision/owner/SDK execution checks does not prevent waiting or replanning.',
          'R4 has no executable contact incumbent; loss of an incumbent after phase expiry is a source-verified potential path, not an observed R4 event.',
          'R4 remote internal generation7/9 is distinct from selection generation5; numeric difference alone is not an owner mismatch.'],
        production_modified=False,metadata_modified=False,ros_contacted=False,temperature_queried=False,
        script_sha256=sha(__file__))
    result=O/'result.json';result.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    text=['# R4 抓取链时间限制与硬门审计','',
      '本轮首次失败是第二视角的 5 秒交接等待超时；随后旧在途帧遇到任务撤销，才被记成感知 first_loss。之前到位确认耗时 0.832 秒，连续 191 条 READY，最大源年龄 0.461 秒。','',
      '成功率优先应首先调整等待与搜索策略：两族失败、30 秒子预算、已有可用候选后的附加比较、最多两次观测都不是实际碰撞证明。真实碰撞、控制权改变、错目标和缺失实测反馈仍约束动作提交；这些约束不要求直接结束所有等待或重规划。','',
      '实际值以 R4 wire 为准：近场约 90 秒、session 600 秒、capture 3 秒、max_observations=2。YAML 中 max_observations=3，未查询live参数，其差异来源未归因。','',
      '| 阶段 / 门限 | 当前值与结果 | 判断与建议 |','| --- | --- | --- |']
    for r in rows:
        text.append('| '+r['stage']+' / '+r['name']+' | '+r['actual_or_configured_value'].replace('|','/').replace('\n',' ')+' | '+r['classification']+' '+r['recommendation']+' |')
    for r in rows:
        text+=['','## '+r['name'],'',r['purpose'], '', '当前后果：'+r['current_consequence'],'',
               '证据：'+r['r4_evidence'],'','建议：'+r['recommendation'],'',
               '动作提交仍须满足：'+r['execution_requirements_to_retain'],'',
               '源码：'+', '.join('`'+x+'`' for x in r['source_locations'])]
    text+=['','## 覆盖边界','',*['- '+x for x in report['limitations']], '',
           '原消息、每条 raw ROS 字节 SHA、源码 SHA 与精确审计绑定见同目录 result.json。此次仅新增审计，未更改生产源码、元数据、配置或现场状态。','']
    md=O/'audit.md';md.write_text('\n'.join(text))
    receipt=O/'receipt.json';receipt.write_text(json.dumps(dict(result=str(result),result_sha256=sha(result),markdown=str(md),markdown_sha256=sha(md),source_files_unchanged=all(sha(W/k)==v for k,v in sources.items()),row_count=len(rows)),indent=2)+'\n')
    print(json.dumps(dict(result=str(result),sha256=sha(result),receipt_sha256=sha(receipt),rows=len(rows)),ensure_ascii=False))

if __name__=='__main__':
    main()
