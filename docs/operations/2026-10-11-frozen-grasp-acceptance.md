# 2026-10-11 冻结功能后的实机验收

用户要求重启完整 ROS 与上位机，电子皮肤关闭，优先验收已有两项恢复修复并打通可重复端到端正例。本文按洛杉矶日期记录；会话目录使用 UTC 时间。

- 会话：`ros_log/full_latest_frozen_no_skin_20261012T021050Z`。
- 运行源：10.11 GitHub 备份 `6cc32253d05c80669ff03ef1a0faba9c3eac103c` 对应的已验证修复；此前361项部署源逐项一致。本次增加两份启动包装文件，不改生产算法。
- 启动：`.runtime/start_full_latest_frozen_two_observations_20261011.sh`，包含原无皮肤启动，覆盖 `/grasp/unknown_observe_return_max_observations=2`。线上确认不返回初始位、提升关闭、温度监测关闭/轮询0，GUI/11个主要节点和新鲜SDK反馈已运行。
- 优先验收 GEN19 支撑缺测三端有界等待：撤销旧几何/计划，在原截止内独立完整新证据恢复；不提前判丢失、不续旧0.8秒。
- 优先验收 GEN18 当前配准超限重新采集三张全新帧：不改10°配准界限。真实未触发时记录“未触发”，不能写为实机通过。
- 保留开口、实际接触范围、关节限位、桌面净空、身份/时序及完整命令曲线碰撞审核。模型分数不单独否决安全接触。观察位曝光/新增体素仍用于是否补视，不用于否决当前完整安全抓取。
- 当前与建议仍有差异：近场首个完整安全候选后有硬编码8秒比较窗。本轮优先冻结已验证算法，未冒充已实现首个候选立即结束搜索；如该等待实际阻塞，再针对该现有策略作最小改动。
- 闭爪、持握、提升分别验收。无触觉的 `HOLDING / GRASP_HOLD_UNVERIFIED / success=false` 为持握未确认，不能直接记实物抓取失败；视频/人工观察单列，受控提升另行验证。

当前本次重启首轮 selection generation 2：用户确认 `call_d1cd0fd756c148fab843daa9de588fee:0`，新位置凭证 `realigned_20261012T021309Z`；录制 `20261012T021428.519576Z_6e47045b48754a3da85f16cd53ae0cce`。该轮已结束：完成两次补视与到位审核，未接近/闭爪。最终 `NEAR_FIELD_DIRECT_TIMEOUT`，是在原期限内未选中完整合格候选，不证明目标不可抓。两项恢复修复在本轮均未触发，不能记作实机验收通过。GUI 已恢复手动。

进一步核对原始审计和仿真请求：原高度的部分候选被完整预测命令曲线与目标包围盒相交拒绝；已有两条提高接触高度的重试通过标称及预测命令曲线，随后才在 MuJoCo 闭爪/预载阶段失败。因此，不能把本轮全部归为路径不可达，也不能把原请求的后缀“缺少提升诊断”当作唯一拒绝原因。

- `0ec46c52c6eaa86ebae4d872`：高度提高约 2.581 mm；首次模拟双侧接触宽约 38.6 mm，随后按请求目标 29.729 mm 闭爪并预载，失去对向接触。离线原请求重放显示右指接触由侧面转向顶面。
- `ffad12d49afa8f64c04b9a4d`：高度提高约 2.779 mm；模拟闭爪没有形成合格的同时对向接触。两次重试 plan_id 与真正提交仿真的请求一致，结果未在交接中丢失。
- 本轮 req36 共 24 个严格检查短名单候选，仅 5 个被严格检查，19 个因共享时限未完成，不能记为全部不可抓。
- 闭爪余量计算发现不一致：未知物体测量门使用每侧 0.5 mm 开口余量，原固定闭爪计算却扣除每侧 2 mm，再扣除 2 mm 预载；本轮由此额外闭小 3 mm。现已修复并部署，真实接近和闭爪尚未验证。

证据在会话 `gen2_analysis/`：`compensation_geometry_summary.json`、`static_contact_patch_summary.json`、`frozen_policy_mujoco_gate_audit.json` 和 `mujoco_contact_trace/`。原失败的完整 MoveIt 曲线未保存，离线重建曲线不能冒充原曲线；原 MuJoCo 请求、实际加载模型及原响应可逐值复现。最新对准凭证为 `realigned_20261012T021937Z/alignment_receipt.json`，对应 `call_4d99e3a5cc5c4b49bbc9e28760709743:0`，随后已用于第3轮。

## 已部署修复及验证范围

1. 复用现有计划内接触证据字符串，schema2 冻结本次实测宽度、实际开口余量及所需开口，并绑定计划哈希和最终位姿。仿真与实机固定闭爪共用此数据；未知接触执行缺失新证据时拒绝，历史 schema1 仍可读，其他模型和手动控制保留原行为。
2. 在原接触高度连续可行区间中，先试正向 50%，再保留原正向 75% 和负向 75%。最多三个高度提案，共用原期限，首次完整路径通过即退出。补视上限仍为两次。不能全局用 50% 替换 75%：实录 GEN30 的 50% 路径仍碰撞，而原 75% 通过；这个失败回归已保留。
3. 当前 GEN2 第一方向的正向 50% 对应上移 1.720835534 mm，修正后闭爪目标 32.72894 mm。离线重建接触路径与原客户端“不要求提升”的对向预载审核通过；真实 MoveIt 全三阶段和实机尚未验证，仿真提升仍失败，保持自动提升关闭。第二方向不能据此视为通过。

隔离回归分别 74 项、77 项通过。实际工作树组合回归最终 **585 项通过、0 失败、0 跳过**，源哈希前后不变：`verification/gen2_contact_integration_r3/result.json`。此前组合失败分别来自旧测试缺显式开口元数据，以及 SanStand 文件系统不支持一项审计路径单测所需硬链接；保留原失败日志，未恢复生产默认余量回退。可丢弃 POSIX 单测夹具使用 `/tmp`，正式回归输出和实验记录仍直接写外盘。

部署回执：`gen2_contact_gateway_deployment/result.json`、`gen2_contact_compute_deployment/result.json`。已重载网关及三个计算节点，驱动、相机、GUI 和仿真服务未重启。部署前后使能未撤销，未查询温度。

## 第3轮首次启动被外盘空间不足阻挡（已解除）

部署后的只读反馈与用户确认关节位置最大差为 0，目标位于画面中央。随后录制器在创建记录与选择新代次之前报告 `RECORDING_BLOCKED_LOW_DISK`：可用 4,479,516,672 字节，最低要求 4,831,838,208 字节。第3轮没有创建新实验或代次，没有下发运动，GUI 保持手动，对准确认未消耗。两个只读观察器已关闭。

阻塞回执 `gen3_recording_blocked.json`；按 `AGENTS.md` 的 local_only 及外盘原件保留约定，未删除任何原始记录，未降低录制空间要求，未向 GitHub 上传或转存系统盘。需用户腾出外盘空间，或对明确列出的旧失败原始录制给予新的单次清理授权，再继续。


## 两份旧失败原始录制的单次清理已完成

用户实际回复 `call_aaa53eb3ec4c42c0b6425dd35736be4b:0`：允许仅清理两份指定失败录制后继续抓取。只删除 `20261006T085709.051090Z_bc8e72c457f542a18a60e9e94fb6b3e1/keyframes.bag` 与 `20261006T093000.069892Z_2b67317c7fb54de5a1fd94d6b4f7bb49/keyframes.bag`，共 3,202,546,861 字节（2.983 GiB），清理后可用 7,677,018,112 字节。删除前完整 SHA256 与授权清单一致，失败、关闭及保留摘要/结果/日志/审计逐项有据；新旧其他外盘原件未删除。授权、执行及保留证据核验见外盘 `maintenance/user_failed_cleanup_proposed_20261012T0310Z/execution_receipt.json`。自动上传、自动删除仍关闭，local_only 不变；这不是持续清理授权。

## 第3轮：首补视后的接触候选通过，执行交接失败

录制 `20261012T032110.570728Z_a1a1c737f44d453aa25a5585504da760`，selection generation 3，stamp 1791775272097881317。本轮完成第一次补视及到位审核，尚未执行预抓取、接近或闭爪，不能记成实物夹持成功。结束原因 `CONTACT_PREGRASP_PUBLISHED_PLAN_REQUIRED`，GUI 已经在核实任务 inactive 后恢复手动。

- 接触计划 `9e4f0ab73da260be18b162a8` 使用新高度重试第一提案，上移 2.064774526 mm。预抓取、接近、抓取三阶段路径预检及六条名义/补偿预测连续几何检查通过。
- 新 schema2 冻结实测宽 32.700438698 mm、每侧余量 0.5 mm，ROS 序列化后的 opening 33.700440079 mm。实际提交模拟的固定闭爪目标 30.700440079 mm，已消除旧配置多扣的 3 mm。
- 模拟闭爪预载时双侧法向与接触力通过；后续模拟提起仍失败。当前自动提升关闭，因此仅按闭爪分阶段审核接受，不能宣称含提升的完整仿真或实物抓取通过。
- 录制证据确认该计划只发布于 `/grasp_6d/preview_plan_enriched`，未发布于 `/grasp_6d/plan_enriched`；任务从预览冻结计划后调用执行网关，而网关要求对应正式发布凭证。不是正式计划发布后被撤销，也不是这一候选又被碰撞检查拒绝。
- 详细原审计、仿真请求/响应与 ROS 计划保存在 `gen3_analysis/accepted_contact_plan_audit.json`，SHA256 `b61b3e0fef21e9b0eff210c1a97b0fb470ffdbabbf1ca4f9d731c7e806ef335e`。

下一步修复显式选定接触计划的跨节点交接，保留预览与执行权限的区别、当前身份/代次/期限和完整路径审核。最新新位置确认 `call_nTxCStnuRyQvN1pJY3rit5L7:0` 已记录于 `realigned_20261012T033155Z`，尚未启动下一轮。

GEN3 指定恢复分支复核：546 条本代次原始感知状态中支撑缺测、支撑恢复和 pending_current_support 标记均为零；无配准超限重采事件。两项修复在本轮仍属未触发。监视器的 253 条通用 recovery 是正常站位契约延续，不能计为这两项实机通过。精确审计见 `gen3_analysis/recovery_trigger_audit.json`。

## 用户暂停：交接修复已部署，下一轮未开始

新增任务冻结接触通知，经原任务审核和冻结后发送，网关核对原计划哈希、来源、控制epoch、代次与观察会话才接纳。普通预览不获得执行权限；原prepare/retire双端确认、0.8秒新测量、路径碰撞与原执行期限保持。实际工作树组合999项通过，源哈希前后不变；`verification/gen3_contact_handoff_integration_r1/result.json`。部署回执为 `gen3_contact_gateway_deployment/result.json` 和 `gen3_contact_compute_deployment/result.json`，两者通过；原驱动/相机/GUI保持。

用户随后明确“先暂停”。下一轮 generation 4 尚未选择，也未创建录制或启动自动动作。当前GUI仍为手动，未发布失能或温度命令。该交接修复尚未实机验证，不能宣称已完成实物抓取；待用户唤醒后再继续。最新对准凭证尚未用于运动，恢复时须结合实际位置判断是否仍有效。


## 用户唤醒后的第4轮：交接通过，预抓取计算耗尽原60秒预算

用户继续处理后，20条实测关节反馈与暂停前对准凭证最大差为0、控制epoch相同。画面中看到前臂后，用户实际确认 `call_d79f3f9bfee3459cbc19b6d83c67d846:0`“手已移开，可以执行”，才启动第4轮。录制 `20261012T043724.808174Z_95a301a30b9a4db8872a9c4ac673cc7c`，selection4 / stamp1791779846496010541。

本轮完成两次补视及到位审核。首站接触提案在本地双侧共同支持检查阶段被拒，严格接触路径checked_count=0；没有首站补偿路径或MuJoCo失败证据。独立两侧点数与高度范围不能单独代表共同有效夹持区间，详见 `gen4_analysis/first_station_contact_rejection_summary.json`。

第二站接触计划 `be836d57c1ba9b345e7a2925` 在1791779995.733后成功冻结，新的专用通知和网关接纳走通，未再出现GEN3的PUBLISHED_PLAN_REQUIRED。网关实际进入预抓取plan-only求解，尚未提交接触运动，未接近或闭爪。1791780055.803以 `PREGRASP_CONTEXT_EXPIRED_OR_UNCONFIRMED: deadline_expired` 结束，GUI经inactive核实后恢复手动。

已定位耗时：MoveIt seed路径约14ms；固定轴接近兼容的预抓取端点选择耗41.655952秒（138/151迭代，第二解经几何复核通过）；最终joint路径约13.5ms；剩余约18.22秒在 `validate_selected_pregrasp_path` 内耗尽期限，尚未返回完整审核结果。不能把这一结论写成没有任何可达路径，也不能声称碰撞审核已通过。检查发现数值优化和逐点审核内层反复调用完整上下文校验，正在离线量化和修复重复开销，尚未放宽60秒期限、碰撞采样或姿态要求。

已固化原审计和完整网关失败响应：`gen4_analysis/gate_audit_be836d57c1ba9b345e7a2925.json`、`contact_pregrasp_terminal_be836d57c1ba9b345e7a2925.json`。用户新对准确认 `call_ca921c0cbecd46f38bbe47159dfdcc14:0` 已保存为 `realigned_20261012T044400Z`，尚未用于下一轮。

## 2026-10-12 继续：计算开销修复部署及第5轮

恢复核验发现，上次中断前已将计算开销修复的两份交付文件合入实际工作树，并完成 **1032 项通过、0 失败、0 跳过** 的组合回归，耗时 33.67 秒；`verification/gen4_contact_compute_integration_r1/result.json` 与 JUnit 一致，372 份受测源码及夹具恢复时逐项 SHA256 一致，未重复执行同组测试。独立补充的实际双曲线/任务侧 64 项、端点选择 31 项全部通过，证据为 `verification/gen4_compute_extra_review/independent_review_receipt.json`。

计算回调每次仍检查本地撤销、任务阶段、控制 epoch、观察会话、执行确认和绝对期限；耗时的完整上下文检查在首个计算回调及距上次检查开始至少 0.1 秒时执行，原阶段边界、返回与提交前完整检查保留。0.1 秒是计算回调的轮询周期，不代表能异步抢占单次 RPC 或几何计算。原 60 秒预算、优化迭代、碰撞采样与两条完整曲线均未放宽；新增阶段计时与检查次数审计。

网关已在任务及两轨迹 action 空闲时单独重载，回执 `gen4_contact_gateway_deployment/result.json` 与组合基线 `gen4_contact_runtime_deployment.json` 均通过。驱动、相机、GUI 和三个计算节点原进程保持，重载前后实测关节、控制 epoch 和 selection 一致。运行生产模块 SHA256 为 `41d5d03a81ee23cee5ab9cbbe732bb6228f8191bfcc6cd3c2fe2233fd3d3f67a`；历史暂存 manifest 的 `staged_only_not_deployed` 不作为现状。启动包装脚本仅新增本轮用户真实回复的精确白名单文本，该单行修改的前后 SHA256 与语法核验另记于 `verification/gen4_contact_compute_release_manifest.json`，未冒称它属于先前1032项受测版本。

用户本轮实际确认 `call_wihfuDzB1kMojlpsvX2TR4yp:0`：“已对准且手已移开，允许执行一轮”。新凭证为 `realigned_20261012T073729Z/alignment_receipt.json`。部署后20条实测反馈相对该凭证最大差为0，控制 epoch 相同，任务 inactive，见 `gen5_same_pose_after_deployment/same_pose_retry_context.json`。授权只启动本轮一次，不自动再次选择或重试。

第5轮已通过原录制空间门禁并启动：`grasp_records/20261012T074007.677004Z_e22cd708f46945dca9c6fcbbcac9e6f8`，selection generation 5 / stamp `1791790809404708623`。两份只读观察记录位于 `gen5_watch/`。自动提升关闭、最多两次补视、温度查询关闭；本段为启动记录，尚不能据此认定实机抓取成功。最终结果在下节追加。

### 第5轮已结束：第二补视接纳时旧参考过期

该轮完成首次补视运动与三帧到位审核，尚未提交第二补视运动、接触预抓取、接近或闭爪。最终状态为 `OBSERVE_RETURN_SUPPLEMENTAL_FAILED: TARGET_LOST: perception_failure: source_exceeds_existing_frame_limit`，录制已封口为 failure；原 bag 315,241,507 字节，SHA256 与原 manifest 一致。`gen5_finished/manual_restored.json` 证实任务和模式 inactive 后已恢复 GUI 手动，两份只读观察器已关闭。此次一轮授权已消耗，未自动再选择新代次。

原始时间证据区分当前图像与上一参考：

- 失败当前帧 `1791790874996427297`，故障时刻 `1791790875400450706`，图像年龄 **0.404023409 秒**，仍小于原 0.8 秒限制，不能写成当前相机帧已过期。
- 上一成功参考 `1791790874595781326`，原几何截止 `1791790875395781326`，故障时已超过 **4.669380 毫秒**。
- 当前帧进入计算时上一参考年龄约0.4183秒；已持有到位计算完成凭证，当前帧自身完成截止 `1791790875796427297` 尚余约0.396秒。
- 第二观察位提案发布于 `1791790875355154037`，任务在 `1791790875388593435` 切为 `MOVE_PREGRASP / admitting supplemental observation view`，位于同一帧计算末段。源码中的站位退休检查再次要求旧参考仍在0.8秒内；此处与既有当前帧完成窗口的交接是下一项定位重点。录制接收顺序不等于回调精确进入时刻，不将推断冒充已录制调用栈。

该轮未进入接触预抓取求解，本次计算开销修复仅已完成部署及离线验证，不能宣称解决了真实60秒超时或完成实物抓取。当前图像、原参考的新鲜度、身份/epoch、完整路径审核与原期限均未放宽。

原 bag 的恢复分支专项审计 `gen5_analysis/recovery_trigger_audit.json`：本代次358条感知状态、7条抓取计算状态、921条ROS日志中，支撑缺测/恢复、`pending_current_support`、`STATION_REGISTRATION_REACQUIRE` 和 `YAW_BOUND_EXCEEDED` 均为0；两项恢复修复仍为**未触发**。30条普通 `station_compute_recovery` 仅表示站位父契约延续，不记成修复验收成功。

离线边界复现已固化为 `.runtime/test_gen5_retirement_reference_boundary_20261012.py`：真实站位生产代码配合合成RGB-D，在旧参考0.795秒时正常弃帧、不丢目标；跨至0.804669380秒时，当前源仍为0.404023410秒，却触发相同底层新鲜度错误并永久标记本代次丢失。两项复现断言通过仅证明问题可复现，**不代表问题已修复**。注入点为分割结束，故上层错误包装与实录较晚跟踪阶段不同；原始失败输出和修正断言后的输出均保留于 `gen5_analysis/retirement_reference_boundary_reproduction*.stdout.txt`。

下一修复方向已独立审阅并仅记录于 `.runtime/gen5_supplement_handoff_design_20261012/independent_design.md`：旧站位仍能产生新测量时完成下一观察lease的双端确认，再切换MOVE；gateway接纳阶段须同步调整，过期旧参考不得作为ACK依据，执行前仍复核同一lease并在失败时撤销。单独忽略退休帧错误不能解决后续新帧与measurement接纳的旧参考检查。此方向尚未实现、测试或部署；当前运行仍为已通过1032项组合回归的计算开销修复版本。

### 2026-10-12 继续处理：补视交接修复已部署，待新一轮实机验收

上述暂存设计现已实现并部署。任务与网关在精确 `PLAN_PREGRASP / admitting supplemental observation view` 状态接纳下一观察位；保持旧站位有效，取得既有 measurement executing 请求的两端确认，再进入 MOVE。严格执行器只消费同一请求一次，重新核验 plan digest、session、selection、epoch、两份确认和原执行门禁；规划失败、确认失效或未提交均撤销并清理准备状态。

感知端新增的等待仅适用于已经在原规则下及时开始处理的那一帧：旧参考刚过期时不确认旧参考，允许该精确帧在既有 completion 截止内完成正常 READY，再走原新鲜参考冻结检查。当前源必须不晚于请求，原 ROS 与 monotonic 截止均不续期；缺失 READY、状态/身份/姿态/epoch/配置变化或超时均拒绝。READY 到冻结及确认仍处于现有 input_lock 内，独立线程探针验证 epoch 回调不能插入最后检查与提交之间。原 0.8 秒源新鲜度、60 秒接触计算预算、求解迭代、碰撞采样和到位三帧确认保持。

实际工作树的相关组合 **1661 项通过、0 失败、0 跳过**，838 份源码和夹具前后哈希一致；结果为 `verification/gen5_handoff_integrated_required_r2/result.json`。另做更广泛的3331项回归，3298通过、33失败；在逐文件恢复的修复前代码树按相同顺序运行3260项，3227通过、同33项失败。失败标识及错误消息（仅忽略进程内存地址）完全一致，没有新增失败。原有失败包括旧缓存测试缺失ticket、观察棱柱历史断言、station测试同步/恢复假设、旧碎片拒绝码，以及测试收集重载canonical模块造成的类引用污染；本次未修改这些生产行为或删改失败测试。完整结果及对照分别保存在 `gen5_handoff_integrated/`、`gen5_handoff_baseline_comparison/`，不能将广泛回归写成全部通过。相关1661项与广泛回归重叠，不相加为独立测试数。

安装仅涉及4个生产文件及5个测试文件，旧文件已逐项备份到 `verification/gen5_handoff_integration_install/before/`。发布清单 `verification/gen5_handoff_release_manifest.json` 绑定独立审查、严格相关组合、完整基线对照及844份源码/运行脚本哈希；其验收依据明确为相关范围通过且广泛范围无新增失败。历史各轮测试夹具错误、路径错误日志均保留。

网关与专用三个计算节点已依次空闲重载，最终基线 `gen5_handoff_runtime_deployment.json` 的 `whole_patch_deployed=true`。驱动、硬件接口、MoveIt、相机、夹爪控制器、路由和GUI进程保持；部署前后20条真实关节反馈最大差0，epoch `1791789357299933598` 与selection5均未变，任务IDLE且GUI手动，证据 `gen5_handoff_idle_before/result.json`、`gen5_handoff_idle_after/result.json`。未执行新抓取、未发布关节失能或查询温度。此前GEN4计算开销修复仍已部署，GEN5未到达其实际计算阶段的结论不变。

下一轮必须创建新selection并录制，不能沿用已消费的第5轮单次授权。当前补视修复为离线验证和部署完成，跨节点实际调度、完整抓取及两项历史恢复分支仍未实机验收；等待用户当前对准、手已移开及再执行一轮的明确确认。

用户随后实际回复 `call_59928ad1e6904dafa525e0a9d5016503:0`：“已对准且手已移开，允许执行一轮”。仅完成新凭证 `realigned_20261012T082728Z/alignment_receipt.json` 与同步RGB-D采集，未创建第6轮录制、未选择generation6、未发送运动。接着用户要求按旧失败记录规则清理空间，并新增“低于10 GiB时清理至可用约70 GiB”的持续规则，见存储操作文档。

用户最新明确要求“先把空间清理出来，然后暂停等我唤醒你进行后面的抓取任务”。因此本次仅完成空间维护；上述新单轮凭证尚未用于抓取，后续抓取保持暂停，恢复时先按实际位置和当前画面复核。最终清理结果及保留核验以外盘 `maintenance/user_failed_cleanup_space_20261012T085620Z_111214/receipt.json`、`verification.json` 为准，不能把已部署修复写成第6轮实机验收。


### 第6轮：选代后的执行器冷启动使初始身份模板过期

用户唤醒后原对准凭证已不匹配当前位置，旧姿态复用检查拒绝。用户实际确认 `call_J2vZknEK25IzWtSCjoWEUiIF:0`“当前已对准且手已移开，允许执行一轮”，新凭证 `realigned_20261012T101619Z`；检查画面左侧双手后，用户又确认 `call_75e4b1bfd96348cf86cfa0b0f63aa003:0`“双手已移到机械臂活动范围之外，可以启动”。新姿态20条实测最大差0，控制epoch为1791800107306397756。844份发布来源除包装器增加该真实回复白名单的一行外一致，11个运行节点PID和启动时刻一致，录制空间门禁通过；证据 `gen6_resume_readiness/result.json`。原1661项回归不包含该新增白名单，单行操作修改另有前后SHA与语法检查。

录制 `20261012T101953.207970Z_4e3aa6eb22544ecc91e46d6de1d2cf0c`，selection6 / stamp1791800395071051597。本轮在原300秒等待内未取得新预览，结束为 `NO_FRESH_MODE_PREVIEW selection_changed=False terminal=`。任务未进入active，未补视、预抓取、接近或闭爪；只读观察器5775条实测关节相对起点最大差0。GUI经任务及两轨迹action空闲检查、GUI自身发布确认、独立20条反馈复查恢复手动，见 `gen6_finished_gui_ack/result.json` 与 `gen6_finished_idle_verified/result.json`。

600秒录制自然结束并封口，3350组关键帧；原bag 3,824,608,857字节，SHA256 `e25d6357cf18f64469195de87d0b41c3e47abd9e05ad9a40b0c47121461dfc8f` 已与manifest核对，三个启动/观察会话均已退出。失败记录和文本日志完整保留在外盘，本轮单次授权已消耗，没有自动再试。

定位证据区分了生产端与消费者：生产端以及 `/perception/object` 路由均有新鲜detected=true，remote订阅图也正确；首个不可逆拒绝为 `initial template receipt rejected: initial template does not match acquired source`。首次 acquired 源1791800395830269098，在拒绝时刻1791800396726848840年龄为0.896579742秒，已超原0.8秒；同时最新tracked源1791800396363036632年龄仅0.363812208秒。消费者因初始模板未在期限内绑定而退休本代次，之后把该代次消息按已有规则拒绝，因此不能将后续通用TARGET_LOST误写成相机没有看到目标，也不是订阅到纸盒话题。启动包装器在选代完成后才启动新的rosrun/Python执行器，其冷启动时延是可移出该窗口的开销。当前正在隔离验证先预热执行器再选代的编排修复，消费者原身份和0.8秒门禁不变；尚未作为新一轮实机通过。

恢复分支审计 `gen6_analysis/recovery_trigger_audit.json` 覆盖3347条本代次感知状态、3条计算状态与2028条ROS日志，两项历史支撑缺测/配准重采分支均未触发。GEN5补视交接修复与GEN4接触计算优化本轮均未到达相关阶段，不能宣称已完成实机验收。

### 第6轮后：启动预热修复已安装，等待新一轮授权

封口bag逐条件核验原始模板绑定判据：除历史acquired源年龄外，其余11项状态、source/selection/epoch、不可变模板及实收mask帧/哈希/shape均通过；完整证据为 `gen6_analysis/initial_template_admission_cause.json`。合成像素结合原实测时间差的离线用例重现相同不可逆退休，不能冒称原像素执行重放。

最小修复只调整操作包装器：选代前创建预热子进程，导入未修改的原执行器。READY仅表示导入完成，不初始化ROS、不构造或运行runner；原路由两个reset ACK及latched selection提交完成后，父进程用私有pipe只交付一次精确selection，随后调用原runner.run()。原CLI、300秒预览等待、0.8秒源门禁、60秒接触预算以及全部计划/碰撞/执行检查保持。EOF、导入失败、超时、selection错误或源变化均不启动；释放后不终止可能正在执行的子进程。预热、选代请求、commit和释放请求时刻保存在各轮authorization，便于下一轮核对实际时延。

候选99项通过，独立评审36项通过；实际安装后的相关组合 **296项通过、0失败、0跳过**，845份来源前后哈希一致。测试范围覆盖真实subprocess/pipe、真实router双ACK→实际launcher及原runner/身份模板门禁；这几组有重叠，不相加。新文件 `.runtime/prewarm_mode_grasp_runner_20261012.py`、现有单轮包装器及其集成测试已安装，旧版已备份。未修改在线抓取算法、未重启任何机器人节点。

真实环境只做imports-only探针：导入约0.260秒，未释放selection，子进程因预期EOF以75退出；原ROS图、selection、epoch、关节和控制模式在探针前中后一致。第一次探针在导入前因GUI已为非手动而停止；第二次只读探针验证实际模式前后一致，没有把它改写为手动。结束后另用GUI自身ACK及独立20反馈检查恢复手动，见 `gen6_fix_ready_gui_ack/result.json` 和 `gen6_fix_ready_idle/result.json`。

发布证据 `verification/gen6_prewarm_release_manifest.json` 绑定安装、组合回归、独立审阅、现场不执行探针和原算法部署。此修复只是把可避免的导入开销移到选代前，原runner的ROS初始化仍在提交后，极慢调度仍会被原期限拒绝；尚未证明下一轮模板交接或实物抓取成功。GEN6单次授权已消耗，下一次generation7须取得新单轮确认，不能自动重试。GEN6原始录制已加入空间维护保护清单。

### 第7轮：预热通过，首站补视完成，近场搜索耗尽原期限

用户实际新确认 `call_03290515df7c401eb13692e4ba98a01f:0`“当前已对准且手已移开，允许执行一轮”；凭证 `realigned_20261012T104645Z`。847份当前发布来源与原运行节点复核一致，新姿态20反馈最大差0，录制空间约48.3GiB。记录为 `20261012T104800.873840Z_462e869d618f484285a6ca2fcd4f3303`，selection7 / stamp1791802082869364261。

预热实际耗时约0.303秒，在选代前完成：READY1791802082852104187，选代提交1791802082932137012，精确selection释放请求1791802082932430028，runner minimum1791802083076727390；首次acquired源1791802083413736820。执行器入口比首次acquired源早约337毫秒，未再发生GEN6初始模板过期拒绝，并产生有效观察计划 `74e525c18a928e55728b5dc2`、通过原动作入口确认。因此仅将启动修复记录为“本轮已越过原故障”，不声称从此不会出现超时。

第一次补视运动及endpoint纠偏完成，view1取得unknown/remote双方executing、requalifying、qualified确认。到位三帧为1791802150029829025、1791802150294190645、1791802150558480739。随后进入首站近场候选搜索，1791802240.066307544发布 `NEAR_FIELD_DIRECT_TIMEOUT: shared near-field phase deadline consumed before a strictly reachable candidate was selected`，任务1791802240.329823255释放执行槽。失败后的station_compute_retirement_context_changed/TARGET_LOST是退出后的现象，不当作主因。本轮只有view_index1，未进入第二补视、接触预抓取、接近或闭爪；GEN5第二补视交接与GEN4接触预抓取60秒优化仍未得到本轮覆盖。

原录制任务active结束后三秒自然封口：548组关键帧，141对观察RGB-D，bag 762,397,733字节，SHA256 `264574063949d3dfee12b7570b9423f107f65960017ee1fe90c2e79710bb5f4f` 已核验。runner、录制和两份只读观察器均已结束；GUI收到自身true确认后，经独立20反馈核验恢复手动，证据 `gen7_finished_gui_ack/result.json`、`gen7_finished_idle_verified/result.json`。该轮单次授权已消耗，无自动再试；原始录制加入当前问题清理保护名单。

完整恢复审计 `gen7_analysis/recovery_trigger_audit.json`：768条本代次感知状态、15条计算状态、1428条ROS日志，支撑缺测与配准重采均未触发；364条通用station_compute_recovery及1条processing_deadline_pending不是上述两项修复验收。初步交接范围与完整请求/双ACK证据见 `gen7_analysis/handoff_scope_observer_snapshot.json`；该文件明确绑定观察器当时已读前缀，封口bag核验另记，不将前缀快照冒充完整bag。

第7轮封口审计进一步定位（`gen7_analysis/near_field_rejection_summary.json`）：request27耗时25.155秒后为TARGET_PROCESSING_PENDING，未保留完整逐候选统计，不能写成零候选拒绝。request28耗时61.450秒，输入44项（GraspNet12、tabletop32）；GraspNet12项局部输入均拒绝，其中9项双侧证据不足、3项阶段姿态范围不支持。首轮2项及fallback6项共留下8项最终拒绝：4项接触序列/碰撞预检、3项接触高度配准裕量、1项原期限内MoveIt超时；完整四阶段预检通过0，MuJoCo最终选中0。fallback短名单44项仅检查6项，其余38项尚未检查，不能计作拒绝或证明无解。上述分阶段计数并非不重复候选总数，不相加。

第二补视未生成的直接条件：首轮 `attempted_families=2` 中，track24/v1在高度余量门就失败；只有track4/v0完成所需原始接触序列碰撞审核，`original_families=1`、collision_receipts=1，未达到原证书所需两个完整方向证据，进入fallback。随后耗尽到达位原90秒phase期限；此90秒是近场搜索phase预算，不是接触预抓取网关原60秒预算。request28准备约2.072秒、传输约0.404秒（服务端0.397秒包含其中，不相加）；最终严格比较阶段至任务失败约43.689秒。track26与track16最终拒绝日志间相差28.018秒，仅是相邻日志墙钟间隔，尚不能写成track16独占耗时。

原request28约4.2MB完整gate审计与按请求划分的原metrics已封存。剩余工作应先离线复核“尝试方向计数”与“完整证据方向计数”的让出条件，以及严格预检耗时；不得通过减少证书要求、放宽接触余量/碰撞审核或延长期限来冒充修复。当前未对这一新定位修改算法。最终证据完整性回执 `gen7_analysis/finish_verification.json` 证实847份发布来源、11个原节点进程保持，bag与观察器封口核验通过，后续实机轮次未授权。

### 第7轮后：完整预检方向计数修复与耗时归因

离线反例使用第7轮实录的首两次拒绝及时间间隔：高度余量提前失败没有完整回执，随后真正的接触序列碰撞拒绝才提供一个完整方向；原调度以两次尝试截止，反例按预期失败。最小生产修改仅把该截止计数从 `attempted_families` 改为已验证回执的 `original_families`。局部高度/IK拒绝、超时、错绑回执仍不能成为碰撞证据；继续有限短名单以获得两个完整且不同的原始方向，原30秒ROS/monotonic子预算、90秒原phase、2秒发布保留、恰好两个方向的证书要求、真实grasp曲线碰撞回执、已有安全候选优先和单次fallback均保持。原高度重试延后规则保持，不增加求解迭代或减少采样。

隔离候选302项通过，独立新进程124项通过，均确认实际导入候选文件。覆盖多次局部早拒、重复方向、有限候选耗尽、ROS和monotonic时限、恰30秒完整第二回执、父阶段发布保留、缺少grasp碰撞回执拒绝、真实安全候选，以及完整审计落盘后第二补视交接与提交后过期拒绝。新反例中实录前缀之后的运输与回执是明确标注的合成输入，不能写成第7轮实机重放成功。原版反例失败及首次新增测试收集错误都保留在外盘；后者仅修测试参数组织，不修改生产门禁。

耗时独立报告 `gen7_analysis/precheck_timing_independent_review.json`（SHA256 `2b383dce7cb6e26436486165c1283bd865ab2c188928e3655fe100863407543d`）按plan ID去重，恢复五次预检并逐字段核对消费者原始响应。track16的28.017817秒相邻拒绝日志间隔包含原始方案及三个高度方案，共四次预检；四次endpoint求解合计9.029948秒，已完成连续几何1.168513秒、6838个状态有效性采样。全部五次endpoint合计11.262326秒、已完成连续几何1.428232秒、8444个状态采样。这些几何统计不含最后失败的grasp曲线，样本到响应窗口亦非服务独占计时。没有发现旧GEN4同类的逐采样上下文重建；RPC、规划、失败曲线及锁等待仍缺独占计时，本次没有据此做未证实的性能修改。

计数修复恢复原预算内继续取得完整证据的机会，不会令单次预检本身变快，也不能保证实机一定完成第二方向、生成补视或成功夹持。第7轮原记录仍为失败，其单轮授权已消费。新的单轮必须在部署核验后取得当前对准及手已移开的实际确认。

实际工作树已安装1个生产文件和4个测试/夹具文件，旧文件逐项备份至 `verification/gen7_yield_install/before/`。实际组合 **1094项通过、0失败、0错误、0跳过**（36个文件），854份来源前后哈希一致；隔离302、独立124与组合范围重叠，不相加。统一发布清单 `verification/gen7_yield_release_manifest.json`，SHA256 `39eadf7d09cd16ff2a01653103694d785322bfbe57cc9b01f418d58a7ba47e8c`，绑定旧启动预热修复、当前安装/组合回归、独立评审和耗时分析。

专用感知/remote/任务三个节点已在任务及两动作空闲时重载，运行单元 `alicia-observation-measurement-1791803970974212671`；驱动、硬件接口、MoveIt、网关、相机、夹爪、路由和GUI八个进程身份保持。统一部署索引 `gen7_yield_runtime_deployment.json` 绑定本发布，部署前后20条真实反馈最大关节差0，epoch1791800107306397756及selection7均不变；任务IDLE，GUI手动，源854份匹配、11节点匹配，外盘可用47.51GiB，录制门禁通过，证据 `gen7_yield_pre_reload_idle/result.json`、`gen7_yield_after_gui_ack/result.json`、`gen7_yield_after_readiness/result.json`。没有发送新抓取、运动、失能或温度查询。

两次操作核验的失败也如实保留：首个GUI核验要求重复true后新回传，但GUI已手动时同值callback不重发，故超时；修正只读判据以接受新订阅收到的GUI自身当前锁存true，并持续核实参数及无false，不切至自动制造ACK。另一次在新统一检查器上使用旧GEN5部署加GEN6操作发布，因manifest不一致被拒绝；保持该严格断言，当前统一GEN7部署已通过。失败回执分别为 `gen7_yield_before_gui_ack/result.json`、`gen7_yield_preinstall_readiness/result.json`。操作准备完成，等待新的第8轮单次确认；目前仍不能记录本修复实机通过。

### 第8轮：有效让出证书已生成，第二观察位规划搜索不完整

用户实际新确认 `call_a19be83bcc7f4f58a8834ed7c838ebca:0`“当前已对准且手已移开，允许执行一轮”。新对准记录 `realigned_20261012T112306Z`，20反馈与对准最大差0.0015339808rad，854来源及11节点匹配，录制条件通过。执行selection8/stamp1791804303402431488，记录 `20261012T112501.434384Z_ec184365e63145858bb80bea45a03b94`，预热及原选代双ACK保持。

第一观察计划 `4e7fbea1ed0357303fa2dc8b` 完成运动、端点纠偏以及两端相同三帧到位确认。request24在11.517024279秒取得有效 `CONTACT_SEARCH_YIELDED_FOR_MEASURED_VIEW`：checked2/64、两个不同的完整原始方向、一个真实grasp连续碰撞回执，fallback=false，高度重试延后。该轮attempted=completed=2，证明真实让出及后续补视规划链可达，没有实测覆盖第7轮“局部早拒后继续第三族”的计数分歧分支。

request25进入view2补视规划，但内部106个观察位只检查4个；四次MoveIt均记录 `Unable to sample any valid states for goal tree` / `TIMED_OUT`，每次约0.83秒，累计3.313293秒，耗尽原3秒“开始新严格检查”预算，余102个未检查。没有安全incumbent，原8秒后续比较窗未启动。最终 `UNKNOWN_OBSERVATION_NO_SAFE_VIEW: OBSERVATION_SEARCH_INCOMPLETE`；未产生有效第二观察位预览，未进入admission、第二站executing双ACK或requalification，也未进入接触预抓取、接近或闭爪。不能把规划有限时间内未找到解写成106个全部不可达，也不能把外层一个封装候选的1/1当内部视角搜索完成。

350组关键帧，bag589,060,544字节，SHA256 `22ae9c9347a433253c77cd03ac2078094a268d6ccf45bc31976fae70575db0c0`。任务结束后三秒自然封口，13份manifest文件已逐一核SHA和大小；观察器精确PID在任务及两动作空闲后封口，854来源和11节点保持。GUI自身锁存true及独立20反馈手动核验通过，见 `gen8_finished_gui_ack/result.json`、`gen8_finished_idle_verified/result.json` 和 `gen8_analysis/finish_verification.json`。原始记录加入当前问题清理保护名单。

完整封口恢复审计 `gen8_analysis/recovery_trigger_audit.json`（SHA256 `e316d3013b91d6b6073097cf8ce9890d6067d0c6e73243aa6e7fd47c9ccd009d`）与 `handoff_acceptance_scope.json` 确认两个历史恢复分支均未触发；181条通用recovery不能作为其验收。补视交接修复仍未到达真正第二站admission执行。耗时/候选报告为 `trial_yield_and_view_review.json`，整链审查为 `end_to_end_review.json`；后者的候选补丁评审仍标pending，不是已修复实机成功。

用户随后要求“为什么老是出现重复的问题，就不能一次性解决好吗”，继而明确要求“你一直进行处理，直到成功抓取”。后续按这次持续任务授权推进修复、离线联合验证及逐轮实机试验；每轮仍须新的现场图像/实际反馈核验、同一目标、无手清冲突、动作空闲、完整录制和原全部执行门禁。持续授权不伪装为用户每轮重新对准确认，不自动抬升、查询温度或失能。当前先集中修复补视候选生成与验收使用中心不一致的问题：实录当前中心与retained缺失侧面中心约2.8mm偏差，原两组约25.36°候选共112个在原3mm新侧面证据门下全部拒绝，只剩45°候选。不得移动历史证据中心或放宽原门来修复。第8轮精确RGB-D/TF输入已重建，六项输入哈希及arrival审计匹配，见 `gen8_analysis/fk_replay_inputs/`；离线可生成端点不等于完整轨迹或实物夹持成功。


### 第8轮后：完整候选复现、IK 对照和持续授权

持续授权原文“你一直进行处理，直到成功抓取”已保存至本会话 `continuous_authorization_20261012/authorization.json`，控制文件为 active。平台未提供该普通消息的 ID 或时间，回执明确使用本地记录 ID／记录时间、platform_message_id=null；原 `call_a19be83...:0` 手清确认保持历史属性。每轮必须重新采集并人工复核同目标画面、SDK 反馈、完整来源和空闲状态，包装器一次只执行一轮，记录独占锁、不可返还的轮次账本及暂停／撤销提交边界。没有将第8轮旧确认改写为新的对准确认。

完整端点回放 `gen8_analysis/full_generator_baseline/result.json` 精确恢复原106个通过姿态及244个拒绝；冻结参考变换SHA与实录相同。修复候选结果 `full_generator_candidate/result.json` 在原350槽位中保留全部106个姿态，新增112个25.62–25.90度端点，原侧面信息、夹爪包络、视野、arrival投影、增益及15mm基线门全部保持。没有改历史缺测中心、3mm门、角度族上界、期限、迭代或采样。回放的ROS字典与来源lookup为离线替代，contact-only偏移为占位；真实端点构造和门禁、实录侧信息不确定度保持，不能当作动作许可。

进一步只读IK查询说明仅修端点几何不足以证明可达：新112个端点在当前碰撞场景、历史首站seed、每次0.02秒查询均为NO_IK_SOLUTION；96次／3秒子集也未求解。关闭查询自身碰撞筛选的12个分散对照仍未求解（未修改线上设置），不据此断言数学不可达。证据 `gen8_analysis/added_endpoint_current_scene_ik/result.json`。

相反，原关节空间生成器离线得到的两个端点均通过当前场景state validity、无碰撞接触；MoveIt FK和本地FK矩阵最大差6.11e-16。两个候选使用自身seed及历史首站seed，在碰撞检查开启时均成功求解，历史seed约0.28毫秒；见 `gen8_analysis/known_fk_current_scene_diagnostic/result.json`。因此继续定位该生成器在原2秒预算中的重复约束计算，准备请求内不变量预计算；尚未验证完整路径或实物夹持。当前生产工作树已安装几何及授权修复，在线三个消费者尚未重载，状态明确区分。

实际工作树第一轮59文件组合回归为1710通过／1失败（1711总项）。唯一失败为旧站点单测monkeypatch类与节点MRO不一致而误入真实发布器的测试隔离问题，保留 `verification/gen8_combined_integrated/pytest.xml`，未放宽生产stamp绑定；正在独立定位并将与FK性能修复一起复验。不能把当前组合写成全通过或部署完成。


两个关节空间候选进一步在当前MoveIt场景中，各用原0.25秒／1次attempt、精确关节目标规划成功；历史起点和目标q误差0，报告规划时间12.4／15.2ms。仅调用 `GetMotionPlan`，无execute、action、选择或发布。完整请求／响应见 `gen8_analysis/known_fk_joint_plans_current_scene/`。

按原路径算法离线筛查原始7／8点曲线，Link6/7/8名义CAD／目标OBB／支撑检查及每关节0.06601699039494283rad误差盒的支撑证明均通过，支撑下界4.067／5.513mm高于原3mm；视野198／231个有界样本通过，最小图像边距54.479／53.981px，原实测支撑与arrival prism投影通过。跟随误差证明仅覆盖支撑，不覆盖目标OBB跟踪误差；视野有界采样不等于连续可见性或遮挡证明。候选0原始时序导数通过，候选1峰速0.08044177rad/s超过原0.08，必须继续原strict-retime，未放宽限速。`gen8_analysis/known_fk_paths_offline_original_guards/`明确尚缺正式retime契约、实时hold／epoch／source／执行准入，不提供动作权限。


### 第9轮：补视执行链已通过，补视后接触阶段到期失败（2026-10-12）

上述1710通过／1失败是首次组合回归的历史结果，原XML及失败说明继续保留。测试隔离问题修正、补视中心一致性修复、请求内FK约束预计算及持续授权包装器现已合入实际工作树，最终实际组合为 **1873通过、0失败、0错误、0跳过**，871份来源前后SHA256一致。结果为 `verification/gen8_fk_actual_integrated/result.json`（SHA256 `39348066a5ef1b48d13c199138b675ff4f3f35f163e50209e551803498595e9a`），最终发布清单 `verification/gen8_ready_for_gen9_release_manifest.json` 的SHA256为 `6c8d924ebee71ea3b405b9be5f6d07ebbbffb9047fcb41921e51e017503bb15d`。各隔离／独立范围与本组合重叠，不累加测试数；旧GEN20测试的两项历史基线失败另行留证，没有写成本组合通过项。

FK优化只把同一请求内不变的CAD角点、归一化支撑法向及误差盒Taylor项预计算；逐关节状态的原式及最终原始端点门仍执行，预计算计入原2秒预算。独立数值等价、边界及请求隔离检查通过；离线实录3组对照在原预算中从0／0／1个候选变为1／1／1个候选，不保证所有线上负载下必有解，也不证明实物夹持。原期限、迭代上限、碰撞采样、最多两次观察位、双ACK、限速及不自动抬升规则保持。

三计算节点 `/unknown_tabletop_perception`、`/remote_grasp6d_node`、`/grasp_task_node` 已完成重载，统一部署 `gen8_ready_for_gen9_runtime_deployment.json`（SHA256 `094097e05a085bbdabadb1bcdf2bfd4222872250e89dc7acbff3e12e79edeb5d`）记录 `whole_patch_deployed=true`、`compute_reloaded=true`，运行单元 `alicia-observation-measurement-1791808026847264978`。驱动、硬件接口、MoveIt、运动网关、相机、夹爪、路由和GUI八个进程的PID／启动标识前后保持；重载本身未发送运动、失能或温度查询。此前“在线消费者尚未重载”仅为该阶段的历史状态。

第9轮按已有 `continuous_authorization_20261012/authorization.json` 的真实持续授权启动，未伪造新的用户手清确认。selection9/stamp1791808162970761537，记录 `20261012T122920.925646Z_9a08734d03f247ac895e5b762b04c615`；新场景核验、准备回执和实时观察器分别为 `gen9_continuation_scene/`、`gen9_continuation_readiness/`、`gen9_watch/`。本轮第二补视已通过严格准入、双ACK、运动及到位采样，已经越过第8轮补视路径未产生的阻断；这不等于整个抓取完成。

补视后接触阶段的phase_started为1791808264957453250，原90秒期限为1791808354957432985；到期后记录 `TARGET_LOST: station_compute_wait_expired_or_future`，未进入接触或闭爪。`command_result.json`、`result_summary.json`及manifest均记录failure，manifest为complete且recorder_closed=true；录制自然停止，553组关键帧。request40报告end_to_end约91.152秒，目前继续核查接触计算预算／重试及其与父阶段期限的关系，不把到期报错推断为现场目标实际消失，也不把补视通过写为实物夹持成功。

任务结束后GUI自身手动回传及空闲核验通过，见 `gen9_finished_manual/result.json`、`gen9_finished_idle_verified/result.json`：任务及两动作空闲，20条实际关节反馈稳定，871份来源／1873项组合回归及11个进程身份保持。两观察器按精确PID／启动标识停止，见 `gen9_watch/stop_receipt.json`。持续授权控制文件仍为active；本次失败没有暂停或撤回该授权。继续定位、验证后按原门禁推进，当前尚无实物抓取成功证据。

封口后的因果核验见 `gen9_analysis/termination_and_fk_second_view_review.json`（SHA256 `c8da624547c0af89e515c672daf3029d37d8f2e21c1209918ffeafe584d24882`）。13份manifest文件的大小与SHA256均通过，原bag未再打开写入。request40已在1791808341.288757得到完整接触序列成功候选：原族 `f0f4ad2595773b3b93d68c10` 沿支撑法向整体移动1.7375059906mm后形成 `856d418df04c64491bb1c8cd`。仿真前再次构建却从原族开始，重复原族碰撞失败和同一高度重试，1791808349.074856才再次返回相同成功方案，额外经过7.786099秒；第二次高度预检命中原15秒精确缓存，仍重读反馈／配置／场景。

真实仿真请求及响应保存在同次服务目录 `candidate_simulation_full_start_20261012T021050Z_jc7jctxv/service/requests/1791808349408050879_0d9de49a1af54edfaaadfd9a772d28ba/`。请求耗时7.605773秒，响应约晚于原90秒phase期限2.119435秒，不能再接回已经退休的选择。原响应记录动态抬升丢接触，保留 `simulation_ok=false`／55分；按此前既有 `unknown_lift_gate_enabled=false`、实机不自动抬升的闭合保持策略，原函数核验其IK、碰撞及承载双侧预紧证据为通过，见 `gen9_analysis/current_original_simulation_gate/result.json`。这只是历史仿真证据，不是实物成功，也没有改阈值或新增放行例外。

重复来源为 `_build_selected_preview_bundle` 每次从原始 `runtime.sequence`／`grasp_pose` 重建，已通过的高度修改只进入本次局部plan和 `final_execution_sequence`；严格检查入口又清除最终结果。因此修复方向是同请求、同原族绑定的高度提案优先重建，重新进行完整几何／高度／连续性及新反馈严格预检；不能直接把已注册姿态写回原族造成二次注册，也不能用旧成功结果取代新准入。早期日志的0.000075mm高度余量属于req1局部候选；request40最终候选原余量4.414274mm已超过原2.676768mm证据门，不能将早期拒绝误判成本轮所有候选都不可行。分析见 `gen9_analysis/height_margin_rebuild/result.json`。

本轮仍未进入接触预抓取执行网关，故原60秒执行计算优化尚未得到本轮覆盖；本次到期属于90秒近场规划阶段。原录制已加入空间维护保护表，回执 `gen9_analysis/protect_record/receipt.json`；没有删除文件。上述修复此时仅在隔离目录验证，尚未安装或用于下一轮。


### 第9轮后：保留已选高度几何，完整重验后再交给下一阶段

封口证据澄清第9轮并非所有接触候选都失败。request40的原方案 `f0f4ad2595773b3b93d68c10` 经同源支撑法向+1.7375059905585365mm提案，得到 `856d418df04c64491bb1c8cd`，在1791808341.288757通过完整pregrasp／approach／grasp预检；同一请求重建时又先检查原失败方案，再于1791808349.074856通过同一高度方案，相隔7.786099秒。第二次成功已有网关prediction cache命中，但仍执行原新反馈、场景及上下文核验；重复耗时不能全部算为成功提案的独占求解时间。

随后的真实仿真耗时7.605773秒，返回时已晚于原90秒父阶段期限约2.119435秒。其初始双侧闭合及预载证据通过；当前原配置为close／hold-only，原validator返回 `MUJOCO_LIFT_DIAGNOSTIC_ONLY` 通过，动态lift失败仅作诊断。没有修改仿真阈值、力、质量或lift策略，也没有将过期结果接回历史请求。证据为 `gen9_analysis/current_original_simulation_gate/result.json`、`height_margin_rebuild/result.json` 及 `termination_and_fk_second_view_review.json`。第9轮仍为失败，且没有实物夹持成功证据。

最小修复只在同一请求、完全相同原始plan内容、source、ticket／epoch／phase、CAD／opening／高度证据和registration policy绑定下保留已选高度的几何recipe。下一次仿真或发布重建从原family应用该位移一次，重新执行原几何、接触证据、3mm高度门及完整fresh contact precheck；不缓存严格成功结论，不累加高度。优先提案失败时，继续原方案及剩余有限提案，跳过本次已检查的同index，仍最多原方案加三个高度提案；原期限、迭代、碰撞采样、所有源／反馈／场景检查和执行门禁保持。GEN4的60秒contact执行guard在第9轮尚未进入，不能据此声称该guard已实测通过或失效。

真实ModeAware继承链测试在安装前查出模式层override缺少新参数的 `TypeError`，已最小补齐转发，原补视yield／defer条件完整保留。新增8项覆盖参数转发以及仍应defer的情形；三次真实调用依次贯穿选择、MuJoCo筛选、最终发布。后者在原版为4失败／6通过，在候选为10通过：原版需6次strict调用，候选需4次，每次构建仍有新的完整预检。受控时钟使用实录重复区间及仿真时长、另设0.1／1／2秒新预检耗时，只证明这一反例及截止门行为，不保证将来实机一定赶上期限。

隔离最终真实MRO／yield／补视交接／网关组合208项通过，独立最终组合356项通过，独立三构建10项通过；此前95项及独立238项范围重叠，不相加。依赖版本冲突、独立测试安装路径遗漏及ROS包路径误加载原版造成的环境失败分别保留；最终回归显式断言ModeAware实际加载隔离base与override两份来源。交付清单 `gen9_contact_selected_height_20261012/delivery_manifest.json` SHA256为 `c6defbc1656421dbfd1c82f9301136c53cea5a9521226fbe824adfa52008fce2`，含两个生产文件、一个旧测试修订和两个新增测试。独立评审 `verification/gen9_selected_height_independent_review.json` SHA256为 `301b8a87898a60636631f9e6a15c5b85de29a4cf4825bae987385290221a3fd9`，同时绑定交付及三构建独立评审。

实际工作树已安装这5份交付文件并保存原文件备份，74个文件的实际组合 **2030项通过、0失败、0错误、0跳过**，873份来源前后SHA256一致。`verification/gen9_selected_height_actual_integrated/result.json` 的SHA256为 `8c5304e374d215bedfdaa3c2a30670b6f5383b415c2edf6c6a7b57b8ec108035`；统一发布 `verification/gen9_ready_for_gen10_release_manifest.json` 的SHA256为 `c8676926c0de6816f4fb3b3ab867caf0ea6a2753d88042f1369c47ffd64a0548`。历史1710通过／1失败、GEN9实际失败及本次所有隔离失败证据均保留，不改写为成功。

仅感知、remote、任务三个计算节点已重载，运行单元 `alicia-observation-measurement-1791810339050654828`；八个硬件相关进程身份保持。重载回执 `gen9_fix_compute_deployment/result.json`，统一部署 `gen9_ready_for_gen10_runtime_deployment.json`（SHA256 `c1aaded78be35f6aeb1351f04aea15f1c342f3445808eb69a14b330da2c5ca35`）记录 `whole_patch_deployed=true`、`compute_reloaded=true`。重载后的空闲及来源核验为 `gen9_fix_post_reload_idle_verified/result.json`。修复已部署，但尚未有本修复后的实物抓取验收。

准备第10轮时，重载前的只读反馈与GEN9结束姿态对比不一致，故未沿用旧末端对准记录；新现场图桌边可见手指。原失败对比、新反馈空闲核验与场景图分别保存在 `gen9_fix_before_reload_idle/`、`gen9_fix_before_reload_current_idle/`、`gen9_fix_changed_pose_scene/`。主代理已请求现场清空确认，当前 `awaiting_scene_clear_confirmation=true`，第10轮尚未启动；该确认核实现场清空，既有持续授权保持active。当前没有发起新的自动抓取，也没有实物夹持成功断言。



### 第10轮：补视准入时执行权限被撤销

用户实际回复 `call_8e456d3fb70548a2a82141f0100a4059:0`“双手已移到活动范围之外”，回执为 `gen10_continuation_scene/scene_clear_receipt.json`；这是当前现场清空确认，既有持续抓取授权仍为active。主代理重新核验现场图及20条实际关节反馈后启动第10轮，selection10/stamp1791810616887247323，记录 `20261012T131015.007765Z_ca31b008d8ab4f9aa749a6c9ef9032a5`，初始plan `1000e0a8cf268b26c6dd6fda`。启动时任务为MOVE_PREGRASP，原现场清空等待已解除。

第一观察位之后，本轮在补视准入时记录 `OBSERVE_RETURN_SUPPLEMENTAL_FAILED: EXECUTION_AUTHORITY_REVOKED: frozen execution authority was revoked by a hard safety event`，未进入闭爪；最终manifest于2026-10-12T13:11:24.714083Z封口为complete／failure，recorder_closed=true。这条通用撤销信息本身不能定位具体触发原因，正在核查交接证据与回调次序；没有写成本修复实物验收或抓取成功。

录制自然停止，实际165组关键帧。按manifest实际清单逐项实读核验 **12份文件** 的大小及SHA256全部匹配，包含328,577,143字节bag，SHA256 `4bb04f17f5741c9e1d9df9fe380e25c0bf72575e4bcf9742c625c3b76e69dd2c`；核验清单为 `verification/gen10_finished_progress_update/manifest_verification.json`。GUI自身手动回传、任务及两动作空闲核验通过，见 `gen10_finished_manual/result.json`、`gen10_finished_idle_verified/result.json`；873份来源及进程身份保持。观察器由主代理封口，记录 `gen10_watch/stop_receipt.json`。

本轮记录已加入 `maintenance/failed_record_space_guard_protected.json`；第9轮及全部既有保护条目保留，没有删除录制或更改归档策略。当前进度为failure sealed、active_record=null、automatic_grasp_running=false，工作状态为调查GEN10权限撤销；第9轮失败历史保持，持续授权仍为active。


### 第10轮后：补视交接异步生命周期修复已安装，实际组合回归中

封口事件已定位最先触发的硬事件为 `TARGET_GEOMETRY_INVALID`：任务发布新观察session前先清除了旧station记录，旧生产者在异步排空期间发送的完整 `TARGET_PROCESSING_PENDING` 几何于1791810680.147左右到达，因归属记录已失而落入普通无效几何撤销分支。旧pending产生时刻为1791810680.144881，remote session ACK约在1791810680.39635到达，任务随后于1791810680.499606失败；首个TARGET_LOST在失败后才出现。该轮控制epoch、使能、硬件保护及GUI模式无相应先行变化，不能将此原因写成现场目标先丢失或真实硬件保护触发。证据为 `gen10_analysis/authority_revocation_independent/closed_events.json`。

本次按发布前、清旧记录、排空、session ACK、metadata rebind、双executing ACK、运动、新采样及终态逐段复核。unknown不订阅session，remote session ACK并不代表旧生产者停止；只有原measurement prepare收到双方精确executing ACK后，才固定旧通知来源截止。任务保留有界的旧owner回执及最多两个先前交接历史，完整绑定selection、epoch、target、phase、camera与原contract／recovery chain。false object先到时必须等同源typed status配对，未配对时禁止所有提交；等待释放回调锁，并在提交锁内重查，处理“等待返回后、提交前又来消息”的竞态。新接触计划冻结也通过原绑定动作入口，不能清除真实撤销。旧回执只过滤已证实的历史通知，不恢复几何、计划、station权限或续期。

原0.8秒源新鲜度、5秒station身份、phase/session截止、反馈容差、碰撞及measurement双ACK保持。普通无效rich plan、真实lost／stop／manual／epoch变化、缺失或伪造回执仍拒绝。独立真实ModeAware回调测试67项通过，旧代码12个异步排列反例全部失败；另40项连续补视历史及生产者完成路径通过。曾发现的wait到commit空窗已在最终候选复测；此前32项producer测试没有被当作整链验收。两项旧feedback fixture问题已在原工作树重现，修正为显式用完已有一次重资格机会后检查第二故障永久拒绝，并持原锁调用；remote生产代码未变，16项入口及48项原重资格测试全过。这些scope存在重叠，不相加。

最终交付 `gen10_supplement_handoff_authority_20261012/delivery_manifest.json` SHA256 `c03b53086f089974be7c3672df56dfffec7ca3ae42f5ab99c5ed66cebcfe13ae`，独立生命周期评审 `verification/gen10_handoff_lifecycle_independent_review.json` SHA256 `6c13a12c4b46c50eb4a5d55f778a3f502a0f358c85922be389563f624b6c3d7a`。实际工作树已安装8份文件并备份原文件，安装回执 `verification/gen10_handoff_integrated_install/result.json` 绑定877份来源；114个文件的实际组合回归正在 `verification/gen10_handoff_actual_integrated/` 运行，尚不能称通过。本节写入时尚未重载在线节点，本轮新修复尚未部署；在线仍是第9轮后发布。第10轮实录保持failure，持续授权active，第11轮尚未启动，仍无实物夹持成功证据。


### 第10轮后：实际组合复跑及三个计算节点部署完成

首次实际114文件组合共3085项，3082通过、3失败，未作为发布验收。三项均为 `test_initial_preflight_task::test_motion_lease_handoff_failure_never_retries_or_submits` 的模块重载身份问题：测试修改旧class，真实ModeAware MRO已使用新class，故原本应由fixture模拟的measurement begin反而执行真实方法，并因fixture没有session而失败。修正范围仅为测试选取实际MRO中对应class，不改变在线生产逻辑、lease门禁或“失败不重试、不提交动作”断言。首次失败日志、测试专用追加安装及修正后完整实际复跑均保留，历史GEN10／GEN9失败、1710通过／1失败亦未改写。

修正后的实际工作树完整组合 **3085项通过、0失败、0错误、0跳过**，877份来源前后SHA256一致；测试结果 `verification/gen10_handoff_actual_integrated_r2/result.json` SHA256 `0ef5903b9b4331ae63aed4ddeeadca272c402b1c7f891f96927b92df5a01a3b9`。统一发布 `verification/gen10_ready_for_gen11_release_manifest.json` SHA256 `1d387cd58ab015344f1520fd8668fe8c10507419192542aaa759d8ac3f1b9d81` 绑定实际来源、两次安装、独立评审及本次回归。

仅感知、remote及任务三个计算节点已重载，八个硬件相关进程身份保持。统一运行部署 `gen10_ready_for_gen11_runtime_deployment.json` SHA256 `88b004466d00cf02ddf4f0aadeedda19df81bd0bb15af6705b42ea7caec3d6aa` 记录whole_patch_deployed=true、compute_reloaded=true；重载后的空闲及来源核验为 `gen10_fix_post_reload_idle_verified/result.json`。本次补视异步交接修复已部署，原新鲜度、station身份期限、phase/session截止、测量双ACK、碰撞与控制门禁不变。离线与部署检查通过不等于实物抓取成功，GEN10实录仍为failure，持续授权仍active；下一轮由主代理核验当时现场和原准入条件后管理。


### 第11轮：第二观察位交接完成，接触预览晋升受阻后到期失败

在既有持续授权及当前现场／20条反馈核验后，第11轮以selection11、stamp1791814147004121541启动，记录 `20261012T140905.069509Z_022d4230fcb4456d8808a52b6c1db9ca`，初始观察计划 `1edcfedd6875d970a6cbfa6a`。实际完成初始和第二观察位运动；第二观察计划 `fbcd33c225975c681283e990` 取得unknown／remote双方executing、requalifying和qualified确认，到位三帧为1791814232197519063、1791814232397658348、1791814232598614931，1791814233.508662记录 `Supplemental observation completed`。GEN10的交接权限撤销本轮未再现；这说明本轮经过了所修路径，不代表其他阶段已通过或从此不存在时序失败。

第二观察位之后，1791814278.310283422的观察器收到有效 `CONTACT_EXECUTION_PLAN` 预览 `2b1ab71f0919c17e5d6ce5bb`。request43约43.032秒的metrics记录preview通过1项，但promoted通过0／拒绝1，结果为 `TARGET_PROCESSING_PENDING`。最终1791814323.578077554释放执行槽，失败 `TARGET_LOST: station_compute_wait_expired_or_future`。本轮没有接近或闭爪；当前定位方向为接触预览到晋升的pending与station期限关系，不能将通用TARGET_LOST当作已证实现场物体消失，也不能把有效预览或观察运动完成写成接触授权／抓取成功。生产者与任务侧进一步因果分析另行封存，本节不提前断定其根因。

runner以6退出、录制自然封口，manifest于2026-10-12T14:12:08.108846Z记录complete／failure、recorder_closed=true；542组关键帧及156对观察RGB-D。逐项实读核验manifest实际 **14份文件** 的大小与SHA256全部匹配，其中bag 777,088,952字节，SHA256 `cdafe52e43e127cf7030e91a7a59e688cee77cb91e9079ee40b8363de21bf3d9`。完整清单和关联文本证据为 `verification/gen11_finished_progress_update/manifest_verification.json` 及 `lifecycle_evidence.json`。

GUI自身手动回传及独立空闲核验通过，见 `gen11_finished_manual/result.json`、`gen11_finished_idle_verified/result.json`：任务及动作空闲、20条反馈、right_finger为0.04975m，877份来源与11个进程身份保持。两只读观察器按精确归属停止，见 `gen11_watch/stop_receipt.json`。本轮录制加入空间维护保护表，条目103→104，全部原有保护（含GEN9／GEN10）保留，没有删除录制。当前active_record=null、automatic_grasp_running=false，last/current记录均指向本轮sealed failure；持续授权保持active，继续定位，仍无实物抓取成功证据。


### 用户暂停（GEN11 结束后，2026-10-12）

用户原话：`先暂停`。已停止排查与后续自动尝试，持续授权控制 `continuous_authorization_20261012/control.json` 置为 `paused`。最新只读核验 `gen11_user_pause_idle/result.json` 确认任务及动作空闲、GUI 手动模式、877 份已部署源码一致；没有发运动、温度或关节失能命令。

GEN11 录制已封口并受保护，实物抓取尚未成功。补视交接已实测完成；接触预览未被任务绑定执行的根因调查未完成，不把正常 `EXECUTION_FROZEN` 预览模式当作已确认故障。初步排查指向观测站更新/隔离与旧快照准入之间的时序，需要恢复后完成原消息重放及完整调用链验证。暂停时实测姿态已不同于 GEN11 结束时，恢复时应重新读取现场和关节，不能沿用旧启动快照。等待用户明确唤醒后继续。


### 第11轮后恢复：已复现接触候选接收与绑定入口的协议冲突

用户最新要求“接着没处理完的问题继续处理，直到抓取任务成功”，已解除此前暂停；持续授权控制恢复active／revision3。`gen11_resume_idle_20261012/result.json` 记录任务FAILED且空闲、GUI手动、20条实际反馈稳定、877份在线来源及原进程身份保持。当前姿态沿用本次实时采样，不沿用第11轮结束姿态；本段写入时未启动第12轮，也未发送运动、温度查询或失能命令。SanStand约44.7GiB可用，未触发低于10GiB的失败实验清理。

真实ModeAware继承链用封口第11轮原始消息回放，候选 `2b1ab71f0919c17e5d6ce5bb` 经原回调被接收，原base完整几何、配准、偏移和视野检查18次全部通过；station外层均返回非终态 `TARGET_PROCESSING_PENDING: contact source predates measured recovery`。首次影响该候选的cutoff来自1791814267.427062511的反馈回调：当时当前station source1791814266.612100839超过原0.8秒新鲜度，恢复后cutoff变为1791814267.144820689，而仍在同站计算的计划保留原source1791814235.465517759。它没有获得跨组件可核验的在算证据，故任务不断等待到原90秒期限。不能把普通 `EXECUTION_FROZEN: robot execution is active` 晋升诊断当作此阻断原因；task-owned preview流程本来由任务绑定。

原audit回调读取并校验报告SHA后，原 `observed_contact` 报告检查与旧bound-plan检查也通过；普通before-freeze仍要求source晚于恢复cutoff。独立证据 `gen11_analysis/task_admission_independent/result.json` 的SHA256为 `6e4ed88ec8fb0732f356558021e3fc878efeb9347409cee3b2f526cef346f455`。此回放保留原validator，模拟ROS传输和时钟；bag接收顺序不能证明实际线程锁次序，不能将回放当成实物验收。继续沿原调用链还发现metadata contact rebind会被“old observation is historical while waiting for contact”分支挡住；本次修复需同时覆盖接收、冻结前、原绑定动作入口、冻结后与接触station双ACK交接。

修复正在隔离目录 `gen11_contact_compute_admission_20261012` 编写及独立验证，尚未安装或重载。拟由计算侧在原audit中绑定原始来源／实际三路准入证据／当前station合同和精确plan摘要，由任务侧与自己收到的同源历史独立交叉核验。原计划时间戳不改写，真实失效、身份变化、手动、取消、运动、截止和严格执行检查保持；仅单元成功不作为整链通过。第11轮仍是封口失败，没有接触、闭爪或实物抓取成功证据。


### 第11轮后修复暂存，用户要求立即暂停

用户先要求“修复完就暂停先不执行”，自动抓取授权当即设为paused；随后最新指令“先暂停”覆盖此前继续修复的要求，主代理已立即中断两个仍在工作的子代理。当前所有修复、安装、部署及抓取工作暂停，等待用户唤醒。隔离修复及验证文件保留，尚未安装到实际源码或重载在线节点，没有启动第12轮。

暂停收尾核对实际877份来源仍匹配 `verification/gen10_ready_for_gen11_release_manifest.json`；在线仍为第10轮后部署版本。隔离任务侧393项及独立整链40项结果已产生，远端范围144项已保留；范围重叠，不累加，也不代表实际工作树回归或实物验收。最终交付签核／实际安装／组合回归／部署均留待恢复后完成。暂停回执为 `gen11_user_pause_before_repair_install/receipt.json`。


### 2026-10-12 再次恢复：接触交接修复通过回归并全节点重启，首轮在观察后来源新鲜度检查失败

用户最新明确要求“你重新启动各节点和上位机然后继续处理没处理完的，然后进行抓取任务”，覆盖此前暂停要求。本次恢复包括各 ROS 节点与上位机的完整启动，而不再仅重载三个计算节点。恢复检查确认旧进程已经退出，不能将上一次部署的进程身份、控制 epoch、现场图或对准快照直接当作本次启动证据。新记录目录为 `ros_log/full_latest_contact_admission_20261013T011103Z`；本节日期按 America/Los_Angeles 为2026-10-12，目录名按 UTC 为2026-10-13。`startup.json` 保留本次用户原话及上一会话索引。

第11轮失败根因已用原始消息和实际 ModeAware 继承链复现：原始接触预览 `2b1ab71f0919c17e5d6ce5bb` 的几何、配准、目标偏移和相机视野检查通过，但观测站外层要求候选来源晚于测量恢复 cutoff。计算侧仍在同一观测站上处理原始快照，任务侧却没有可独立核验的计算来源凭证，因而一直返回 `contact source predates measured recovery`，最后耗尽原90秒阶段期限。正常的 `EXECUTION_FROZEN` 预览模式不是此故障原因；第11轮未执行接近或闭爪，仍为已封口失败。

修复在原审计报告中加入 `registered_station_computation_provenance`，绑定完整最终方案摘要、原始三组实际准入的 RGB-D 来源、请求和代次、目标 epoch，以及当前观测站证据。任务侧用自己的来源记录独立交叉核验，不以远端报告直接授予运动权限。原始图像、几何和方案时间戳保持原值；记录只包含有界的合同和摘要。候选复制、冻结前、元数据重新绑定及原子冻结使用同一核验，冻结成功后保存精确方案摘要，后续动作仍经过原冻结权限和上下文检查。真正的目标失效、来源矛盾、身份或 epoch 改变、手动、停止及截止均继续拒绝。

远端只有实际发布且通过原子审计提交的方案，才可作为接触观测站退出交接时的精确方案依据。这一凭证不代表当前几何有效；原生产者的新帧、任务真实关节反馈、请求有效期、阶段上下文以及双方精确 ACK 仍分别生效。审计计算及文件同步放在输入流锁之外，发布与原子提交保持短临界区，避免把整个报告写盘时间阻塞在输入接收路径内。原0.8秒新鲜度、5秒注册身份期限、90秒近场期限、会话上限、求解迭代、碰撞采样、最大两次观察及不自动抬升约定均未放宽。

隔离最终任务侧393项、远端144项、独立整链40项均通过，测试范围重叠，不能相加。独立整链通过实际生产者、remote 和任务的原回调与真实继承链，覆盖测量三帧握手、审计和预览到达次序、任务候选准入、冻结、接触观测站双 ACK 与动作入口；缺失或伪造凭证、来源和上下文变化、手动、截止及锁等待期间的切换继续拒绝。动作提交使用无硬件作用的哨兵；录制几何被平移到合成 RGB-D 场景，不构成新的实机碰撞路径验证或抓取验收。另以最终任务源码回放第11轮没有新凭证的旧消息，18次原外层检查继续拒绝、动作哨兵调用为0，未把旧失效证据改判为可执行。

交付清单 `.runtime/gen11_contact_compute_admission_20261012/delivery_manifest.json` 的 SHA256 为 `cc4a202555e3baeb06a4c4ce3ac8ab83f17dfd39cbaf2f80f17730509a23cd02`，包含5份生产源码、3份测试和1份压缩录制夹具。独立评审 `verification/gen11_contact_admission_independent_review.json` 的 SHA256 为 `2115fad112d8c3ffa4ef516f62c0d4670a8dad55b0af7f270e6fd51474ca75d7`，位于上一会话并精确绑定该交付。夹具两条消息与密封 wire 完整逐字节一致，13项报告字段与原审计一致。历史独立测试结果只记录了5份生产源码哈希，未记录测试和夹具当时的哈希；当前交付与原独立复核目录逐字节相同，本次实际工作树组合回归已同时绑定测试和夹具；历史结果未绑定两项哈希这一事实仍保留。

实际工作树已安装9份交付文件，并在本次会话 `verification/gen11_contact_integrated_install/` 保存原文件备份和安装回执；来源清单由877增至882份，实际组合回归范围由114增至117个测试文件。首次实际组合为3214通过、1失败，原始结果 `verification/gen11_contact_actual_integrated/result.json` 保留。唯一失败来自旧 `test_remote_modes.py` 的 prepared 测试夹具缺少生产类型本来必需的 ticket，真实生产构造一直提供该字段。仅向夹具补入请求 ticket，同时增加 direct 模式不产生 station-contact receipt 的断言，原正例结构、所有来源比较和反例结构变更断言保留；未为此改生产代码。该单文件80项通过并经独立复核，追加安装及备份见 `verification/gen11_contact_fixture_integrated_install/`。

修正后的实际117文件完整组合 **3215项通过、0失败、0错误、0跳过**，882份来源前后SHA256一致。结果为 `verification/gen11_contact_actual_integrated_r2/result.json`，SHA256 `bc4c71454e0e00231940b34f06655881ec29b7ae59fea345899276fd9ad883b6`；统一发布 `verification/gen11_contact_release_manifest.json`，SHA256 `5d7363f1b46764a9d209bc09d0b175e179ff37a21683ebaf5894ddefba965137`，绑定两次安装、独立审核、夹具修订与最终组合。首次失败及此前历史均未改写。

各节点与上位机已完整重启，非仅计算节点重载。`startup.json` 记录启动前无旧节点及监听者，记录本次 core/simulation 新运行单元，并补充顶层清单以外的启动依赖哈希；`node_registration.json` 记录11个必需节点均已注册。独立只读验收 `full_restart_acceptance/result.json` 的SHA256为 `146fa78db080061bfcc0d6841ab0fe5c603af5a9a7612a54c168a84e6dddde63`，确认11个节点均属新 core 进程组、进程身份全部替换、GUI窗口可见、两个轨迹控制器running、MuJoCo 8002监听归属当前仿真单元且模型身份匹配、URDF与手眼配对一致、882份来源匹配统一发布。验收工具只订阅非温度反馈并调用只读控制器查询，未发控制、温度或失能命令。

重新取得本次现场图和20条实际SDK反馈后，`attempt_1_readiness/result.json` 通过，与当前场景采样姿态最大关节差为0；这不是沿用暂停前的对准姿态。`attempt_1_scene/operator_observation.json` 绑定当前图像及就绪回执，记录同一目标可见且当前图中没有相冲突的手部；没有声称用户提供了新的确认。新控制epoch为1791854771428479964。既有持续授权加上本次明确恢复要求生效后，录制先就绪再启动自动抓取，selection generation2/stamp1791854923890747785，初始方案 `06d4e542f38ada725d5f23c1` 已进入MOVE_PREGRASP；本次重启后首轮记录为 `grasp_records/20261013T012841.933029Z_8537e8805ca34fa9be2d153cdbc37b25`。第一观察位已到达，1791854971.937测量重新取得qualified，1791854972.502进入near-field等待；随后1791854972.908约0.466秒的来源年龄被原新鲜度检查拒绝。最终 `GRASP_RESULT success=False`，原错误为 `TARGET_LOST: perception_failure: source_exceeds_existing_frame_limit (generation=2 observed_stamp_ns=1791854972904923677)`。本轮未到接触预览、接近或闭爪，因此不能据此声称GEN11新凭证路径已实机验收通过，也没有实物抓取成功。

录制于UTC2026-10-13T01:29:37.217797封口，`manifest.json` 为complete、result=failure、recorder_closed=true，`recorder_outcome.json` 记录bag_closed=true；manifest的SHA256为 `c8d5461f752b6d9ea70e2e89bb83b896318ba2b6901ee7fd8aa169066e02f3ce`。失败分类同时依据实际任务终态与StartGrasp响应，不依据runner单一非零退出码。主代理正在恢复手动模式及封口只读观察器，保留新启动节点继续诊断来源时间窗及回调时序；本节不提前记录尚未产生的收尾核验结果。第11轮及此前失败记录保持原分类并受保护。

新实验及运行日志直接写 SanStand，继续 `local_only`，不上传、不启用通用归档删除。机械臂温度查询和软件温度反应保持关闭，不发送关节失能命令。本次抓取启动使用重启后的实际来源、进程、控制epoch、任务与动作空闲、当前现场及反馈核验，未沿用暂停前的现场确认快照。



### 2026-10-12 观察后失败复核：GEN12 已安装待联合验证，固定站位外观修复继续暂存

本段沿用 `ros_log/full_latest_contact_admission_20261013T011103Z` 会话与重启后首轮封口失败记录。原 `source_exceeds_existing_frame_limit` 不能解释为当前图像已经过期或现场目标消失：失败时当前 RGB-D source 为1791854972438854694，年龄约0.466秒；被检查的旧测量参考为1791854972104432344，到1791854972904886245时为0.800454秒，超过原0.8秒门约0.454毫秒。首个phase在该帧开始处理之后到达；原在途帧没有首阶段交接归属，随后多分量身份回退又继续使用旧参考时间窗。这是阶段交接和外观参考两个相邻问题，不能只修阶段时序便声称整类失败已解决。

GEN12 将真实到位、endpoint／settle、原三帧身份检查和 qualified 双ACK完成后的时点作为 PLAN_PREGRASP 与规划phase起点；后续原 envelope、range、碰撞、冻结和动作校验继续执行。同一 lease／plan 精确复用同一phase，不重复递增或延长截止。receipt、ACK与选择、session、epoch、phase状态在原锁下交叉校验。生产者仅在原0.8秒内、同站位与上下文仍有效时，于明确计算检查点取消被首个合法phase接替的旧帧；不捕获真实计算异常来冒充取消，不更新旧参考。仅允许原第三qualified帧或当前已测参考两种确切phase来源，任意夹在其间的时间戳不能获得该取消资格。实际线程覆盖回调在最终提交锁之前／之后的次序，原first／supplement envelope和range拒绝仍没有接触动作。

最终暂存组合214项通过、0失败／错误／跳过，其中包括独立52项，不能再叠加计数；证据为 `verification/gen12_phase_owner_focused_r5/result.json`。候选绑定的原BAD图像完整process反例仍保留：提前phase但原计算时间表仍拒绝旧参考过期；将身份检查提前的诊断时间表仍拒绝 `metric_ambiguity_no_unique_complete_rgb_identity`；两者随后原NEXT帧均不复活已失效目标。见 `verification/gen12_registered_bad_negative/validation_receipt.json`；这是保留拒绝证据，不是实物成功。交付 `.runtime/gen12_phase_entry_frame_owner_20261012/delivery_manifest.json` SHA256为 `ce409f20b3e58a066a5b10845632304a7a124bc98d60fba1954a82786aba846b`，独立审核 `verification/gen12_phase_delivery_seal/independent_review.json` SHA256为 `3044c283888bcf072fe27ee2eb4771f4a9311da0257e60ec0d3de425ebdc77ea`。4份生产源码与3份新增测试已实际安装，备份和回执为 `verification/gen12_phase_integrated_install/result.json`；885份来源、121个测试文件等待后续最终实际组合验证。**GEN12 尚未重载到运行节点，本段不将磁盘安装写成在线部署。**

原图像、深度和TF独立诊断进一步说明BAD仍有目标：两个原生深度分量细化出的完整RGB轮廓属于同一物体；旧逻辑要求每个分量对运动前视角外观逐个通过，第二分量colour distance约0.404757超过原0.4门，之后把同一完整物体的两份细化结果当作两名身份竞争者。采用到位时已经独立完成原三帧确认的第三qualified来源1791854971572257757作为固定同站外观，BAD与真实下一帧均可通过原fragment几何方法；BAD各次完整mask与该固定参考的IoU约0.994565／0.940653／0.940495，均高于原0.85门。见 `verification/attempt1_qualified_and_origin_identity_candidates/result.json` 及 `attempt1_first_phase_with_qualified_source/`。这仍是离线诊断，不是完整运行链或计算期限验收。只替换native分量外观而让最终consensus继续使用运动前外观，BAD依然失败，不能把这种部分修改当成根因已修复。

GEN13 正在另一个隔离目录准备固定第三qualified帧外观方案：原到位三帧确认仍用原运动前证据，确认完成后固定同站外观随该站位继承，不从本帧候选自举，不随后续READY滚动更新；原几何、支持平面、SDK位姿、0.8秒来源期限、session／phase合同和外观阈值继续保留。仍须完成原BAD→NEXT连续process正例、真歧义及上下文变化反例，并计入新成功分支的真实计算耗时。**GEN13 未完成、未部署，实物抓取仍未成功。**

为补足后续定位的完整原始帧证据，主代理已验证独立只读录制sidecar：`full_frame_sidecar_readonly_check/` 用 `--duration=3`录得41对同原stamp的完整RGB-D，包含TF、非温度SDK及任务状态；录制自然关闭、任务未活跃、未发送运动或温度查询。`validation.json` 为passed，bag共71,303,386字节，SHA256为 `4ff41ec1e3cc6adbce3d48a882358c99fafa96e984e4ad3cfb5d35bdc1e1ec91`。这是录制链检查，不是新抓取实验，也不以新静止画面替换旧失败图像。新旧失败、审计和验证均继续保存在SanStand，local_only与温度约定不变。


### 2026-10-12 GEN13 固定同站身份与超时传播修复：已安装，实际组合回归进行中

本段继续使用 `ros_log/full_latest_contact_admission_20261013T011103Z`；日期按本地2026-10-12，目录按UTC2026-10-13。重启后首轮仍是封口失败，没有新增实物成功结论。对原始BAD的最终定位是：当前RGB-D年龄约0.466秒，被查的旧测量参考已经0.800454秒；两个原生深度分量经各自真实细化后属于同一完整物体，但运动前外观与到位后的成像差异，使第二分量colour distance约0.404757超过原0.4门。可选同物体合并未通过，后续多候选身份路径也未得到唯一完整身份。GEN12提前建立合格站位的规划phase并修补旧在途帧归属；GEN13处理固定同站外观与超时结果的传播，不通过延长几何寿命或接受任意一个候选规避拒绝。

GEN13只在真实最终qualified回调中，使用原三帧确认已经通过的第三帧1791854971572257757构造不可变外观模板。原到位前三帧继续对运动前证据进行确认；模板完成复制后，在真实input_lock内重新核对请求、原pending与hold对象、三帧来源、selection／epoch／model／相机／手眼上下文、从settle起的全部SDK及原插值括号、第三帧原0.8秒期限，随后才提交marker和qualified ACK。模板绑定完整measurement ownership、RGB与mask摘要及三帧来源，后续READY继承同一对象，不滚动更新，不从当前候选自举，也不跨站或跨lease沿用。普通非站位路径保持原外观行为。

同站正常fragment及最后station recovery均使用该固定第三帧，保留原颜色／形状0.4门、dense identity、原生分量core归属、几何关联、当前支持平面和深度检查；另外要求当前完整mask对固定模板IoU至少0.85，避免连续微小变化累积漂移。模板只证明身份，不提供旧几何或动作权限。manual、stop、epoch、位姿／配置变化、缺qualified回调、来源或模板绑定篡改及真实不同对象继续拒绝。

完整原图回放还暴露了一个独立传播缺陷：相同的当前帧0.8秒到期，若出现在arrival检查入口可按既有注册合同丢弃并等待，若恰在真实SDK括号返回后到期，却曾被普通ValueError路径摘除marker并变成首个target loss。现arrival入口、SDK等待循环和尾部统一使用原注册站位寿命检查，站位核验本身消耗的时间也计入当前源期限。`StationComputeDeadline`、`StationComputeRetired`、`StationFeedbackBracketGap`及真实站位变化异常保持原类型和对象穿过fragment包装层与process的可选提案catch；只有普通几何／身份ValueError保留原包装及回退。超时等待仍要求先前已注册且当前独立有效的站位，原5秒身份合同不续期，旧参考不刷新、不发布过期几何；同时出现真实manual／epoch／SDK往返移动／model变化时，原最终router仍拒绝等待。

隔离最终22个测试文件 **487项通过、0失败、0错误、0跳过**，4份生产源码和22份测试前后SHA256一致，见 `verification/gen13_station_appearance_focused_final_r5/result.json`，XML SHA256为 `b4859ed4f859867f9d4490cb859fdca857d15f0ce11155610a18ac2fc66ca99d`。其中已包含独立98项及root最小控制流6项，不能重复累加。独立范围覆盖不可变模板、复制期间及等待最终input_lock时的真实线程竞态、ACK之前不可消费、各类owner／来源篡改；4类typed异常分别经过两个真实native refinement、两个native core、两个member appearance和最终consensus共7处实际计算边界，保持同一异常实例直达外层router。路由观察使用无动作哨兵，不把注入异常本身当作真实注册等待授权。普通原始拒绝和真实分离RGB对象仍保持原行为。另21项精确到期边界覆盖入口scope、context、SDK之后的0.8秒前1ns／恰好／后1ns，以及同时发生真实站位变化的反例。

此前17文件组合为350通过／3失败，原始 `verification/gen13_station_appearance_focused_r2/` 保留。两项旧integration期待qualified之后继续使用运动前缓存，与本次修复的固定第三帧契约不再一致；修订后分别断言前三帧仍使用premotion、qualified后fragment才使用固定第三帧，两个缓存各只生成一次，原颜色强制失败／回退与身份检查继续执行。另一项support guard原先期待较晚的唯一身份错误，但在实际GEN12基线上已先由 `matching_fragment_independent_geometry_required` 拒绝；root基线复现1失败／4通过，仅更新该分支的精确错误文本后5项通过，未修改生产资格或接受断言。证据为 `gen13_support_guard_baseline/`、`gen13_support_guard_expectation/`。typed包装问题的最小基线为4失败／2通过，候选6项全过，见 `gen13_control_flow_baseline/` 和 `gen13_control_flow_candidate/`。独立过程中发现的复制窗口、真实超时传播缺陷，以及spy可选参数／线程夹具错误均各自保留原失败报告，不写成历史通过。

独立回放使用封存的全部5张qualified后RGB-D，原stamp／TF／SDK不变，真实qualified callback之后串行执行完整producer process；所有新增分支的实际耗时均计入诊断时钟，并保留原共享阶段时间下界。最终前4帧READY，完成时source age约0.38703／0.52596／0.46906／0.76437秒；第5帧0.822747秒正确进入`processing_deadline_pending`，没有first_loss、不刷新last_good或固定模板。9项绑定篡改全部拒绝；没有真实qualified callback的两种BAD→NEXT完整反例仍分别保留原source-age及no-unique-RGB拒绝。另在真实SDK括号返回后将诊断时钟明确推进到source+0.801秒，修前复现terminal，修后保持原已注册pending。统一回执 `verification/gen13_full_qualified_station_process_v5/validation_receipt.json` SHA256为 `92005ccc732b3a64b4bf5146d36b224a2ad1ec4c260b4be0aa0c926f2250f997`，绑定最终4份生产源码、全部原图／深度／配置／时钟脚本及历史失败；原keyframes bag另直接重算SHA256匹配 `50b20e8822eaceb07fbb2fc38dd8b2f525566c6c9c40d64959d718ac84e6433c`。

该回放有明确范围：前两张qualified原图未保存，原成功三帧状态依照封存receipt显式设定，第三帧实际原图与最终qualified回调真实执行；提前phase发生在原双方ACK可用且新callback完成之后，是注明的反事实调度。5张保存帧强制FIFO并不模拟生产latest-only队列，缺失相机帧不能重建，SDK后定点推进只验证到期分支，不是性能保证。不能将4帧READY或离线pending恢复写成新一轮实物抓取成功。

最终交付 `.runtime/gen13_station_fragment_identity_20261012/delivery_manifest.json` SHA256为 `265ed297254cb6eecbfe0aceccbb38a41ad8987bd031566eef045849afdc4e4f`；独立评审 `verification/gen13_station_identity_delivery_seal/independent_review.json` SHA256为 `ba91d0d41fae91de47a63a0507f440110d021d76c3b90a65e49e8a6c2721e0be`。交付共11文件：4份生产源码、5份新增测试及2份旧测试修订，基线为实际GEN12的885份来源；克隆时带入的GEN12旧manifest／patch已精确核验并另存历史副本，未当作GEN13交付使用。全部原测试、录制、失败与修复证据保留。

主代理已把11份文件安装到实际工作树，原文件备份及回执为 `verification/gen13_identity_integrated_install/result.json`，SHA256 `848a1e8f85564a637d11237a3c9f74a806d8d5f9201b7fb5bf6db6c3987e86f8`；来源清单增至890份，最终实际联合范围为137个测试文件。**本段写入时 `verification/gen13_actual_integrated/` 联合回归仍在运行，尚未产生最终通过回执；GEN12／GEN13尚未重载到在线节点，磁盘安装不等于运行部署，实物抓取仍未成功。** 后续实际回归、运行部署和新的抓取结果由主代理另行追加。当前仅修改操作文档及SanStand镜像，不查询温度、不发送运动／失能命令，不改变local_only、保留和空间维护规则。


### 2026-10-12 GEN12／GEN13 实际联合通过，三个计算节点部署完成

上述运行中的实际137文件联合现已完成：**3621项通过、0失败、0错误、0跳过**，890份来源前后SHA256一致。结果 `verification/gen13_actual_integrated/result.json` SHA256为 `a2c2c23653ef2643f8f4dc3ab430994f5b11812166094a16f875b908ddec6f8c`。统一发布 `verification/gen13_station_identity_release_manifest.json` SHA256为 `eb3b44a65fd997896097a173688af29a69df012f3097de8d7e0329b4ac3730d6`，绑定GEN12／GEN13实际安装、完整组合回归与此前交付证据；隔离487项及其中独立范围不与这3621项重复相加。此前运行中记录与全部失败历史保持，不回写为当时已经通过。

部署前重新通过GUI自身回传确认手动模式及任务／动作空闲，证据为 `gen13_before_deployment_manual/result.json`；它是部署前的历史状态，不充当后续抓取时的现场或空闲核验。主代理仅重载感知 `/unknown_tabletop_perception`、远端 `/remote_grasp6d_node` 和任务 `/grasp_task_node` 三个计算节点，运行单元 `alicia-observation-measurement-1791859419588515677`，新PID分别为28859／28860／28861，均与原PID及start_ticks不同。驱动、硬件接口、MoveIt、运动网关、相机、柔顺夹爪、模式路由和上位机GUI八个进程的PID／start_ticks／parent_pid前后完全保持；本次不重复冷启动硬件和GUI。

重载回执 `gen13_compute_reload/result.json` SHA256为 `43a580350d9c7b6ba87ceb630d8b8569d6c58fd977d5dd08b8ea6c3bcd9e3a19`。统一部署 `gen13_deployment.json` SHA256为 `c5b5d66eb0e3c439fa1ea491baf227f4c99e6daa68b20e9352ce0a0a09178f9a`，记录deployment_passed、whole_patch_deployed、compute_reloaded及hardware_processes_preserved均为true，890份来源与统一发布一致。因此此前“已安装尚未重载”状态已完成转换；阶段归属、固定第三合格帧外观和typed超时传播的组合修复现在已加载到在线计算节点。部署没有发送运动、温度查询或关节失能命令。

**软件回归与运行部署完成不等于实物抓取成功。** 重启后首轮仍是封口失败；本段未将新的录制、规划或运动过程判定为成功。主代理负责随后按当前selection、现场及真实反馈管理下一轮，独立文档任务只同步已封部署证据和元数据。原0.8秒几何新鲜度、固定站位身份期限、碰撞／执行门禁、不自动抬升、local_only及失败记录保留规则继续生效。


### 2026-10-12 部署后第2轮：观测测量ACK等待超时，失败已封口并继续定位

GEN13部署后，主代理以selection generation3／stamp1791860166743916511发起本次会话第2轮，记录为 `grasp_records/20261013T025604.344486Z_af3ecb9373264cc9b19a98143f7b5df5`。任务终态与StartGrasp响应均报告 `OBSERVATION_MEASUREMENT_FAILED: OBSERVATION_MEASUREMENT_ACK_TIMEOUT`，`GRASP_RESULT success=False`；runner按普通失败以6退出，没有launcher错误。本段只记录已确认的ACK等待超时，不据通用错误推断目标丢失、硬件保护或具体哪一端的根因；后续GEN14继续调查原请求／确认与时序证据。此前GEN13离线回归和部署通过的结论保持，但本轮没有实物抓取成功证据。

`manifest.json` 于UTC2026-10-13T02:56:59.260501封口，state=complete、result=failure、recorder_closed=true；`recorder_outcome.json` 记录bag_closed=true、81组关键帧、149对观察RGB-D，stop_reason为`observed_task_became_inactive`。逐项实读校验manifest全部 **10份文件** 的大小与SHA256均匹配；keyframes.bag共241,679,070字节，SHA256 `6fe9df1bc047f86b57faad2ff97822ff1f93955f41401f819f7698070580274e`。核验回执为 `attempt_2_seal/manifest_verification.json`，同时保留manifest摘要及封口结果依据，不以单一退出码推断失败。

独立完整原始帧sidecar亦已由主代理停止并封口：`attempt_2_full_frames/result.json` 记录returncode=0、recording_closed=true；`full_frames.bag` 共4,216,897,925字节，实读SHA256 `46086ebd97453de78052ee41e35541c70069209f8687663d894ef278609559b9` 与回执一致。两份bag均无`.active`文件，封口审计时lsof未发现打开的文件描述符。此sidecar包含原RGB-D、TF、非温度SDK及任务／请求／ACK等证据，不用抽样图替代完整录制。

GUI自身手动回传及任务／动作空闲核验通过，证据 `attempt_2_after_manual/result.json` 绑定generation3；这是收尾时点的核验，不是后续自动启动授权或现场快照。890份已部署来源与GEN13统一部署逐项一致。`maintenance/failed_record_space_guard_protected.json` 由105项增至107项，新增本轮完整实验及配套sidecar两条保护，全部105条历史逐项保留；更新前文件、文档及进度元数据备份均保存在 `attempt_2_seal/`。没有删除录制、上传文件或更改local_only和温度约定。当前记录为sealed failure、active_record=null、automatic_grasp_running=false，继续定位GEN14；根因尚未确认，实物抓取仍未成功。


### 2026-10-12 第2轮确认失败阶段，GEN14 错误分类修复已安装但尚未加载

继续复核第2轮原始请求、ACK及完整图像后，已能区分传输确认和测量资格：requalifying请求的远端ACK在3,030,777纳秒、感知ACK在4,281,997纳秒内到达，均accepted=true；随后九张完成计算的到位帧均被 `arrival_fragment_group_original_rgb_identity_required` 拒绝，qualified来源列表一直为空，第十张到达原3秒capture deadline。独立结果为 `verification/gen14_arrival_independent/result.json`，SHA256 `eb8f0aa0e75a5fb348f2e6963104325b5034fc387caefc42ff589e956db76e92`；原wire依据为 `verification/gen14_arrival_diagnosis/wire_diagnosis.json`。因此原 `OBSERVATION_MEASUREMENT_ACK_TIMEOUT` 文案混淆了“缺少传输确认”和“已确认请求但尚未取得合格测量”两种状态。

九帧对运动前外观的颜色距离约0.402–0.430，超过原0.4门；原ORB单应提案的完整轮廓双向IoU约0.56–0.73，未通过原完整mask门限。这些证据确认失败阶段及具体拒绝门，不足以单独认定光照、分割或几何提案的最终算法原因。原像素与匹配几何仍在独立调查，原颜色、完整mask、稠密独立验证、当前3D和三帧资格要求均未放宽。本轮未到qualified callback，GEN13固定第三合格帧模板没有建立或参与该失败，不能据此写成该模板修复回退。

原动作证据仅有一次已成功结束的初始机械臂轨迹；没有夹爪动作goal、没有接触phase，未执行接近或闭爪。第2轮仍保持原sealed failure和原错误全文，不以新分类回写历史实验。收尾手动及空闲核验、两份已关闭bag的精确SHA和107条保护清单仍见 `attempt_2_seal/`；没有新实物成功证据。

独立交付的GEN14 reporting-only修复只修改等待循环的终止错误：在同一请求的双方原ACK校验均已通过、但qualified条件仍未满足时，到原截止报告 `OBSERVATION_MEASUREMENT_QUALIFICATION_TIMEOUT` 并列出仍待qualified的一端或双方；没有完整合法双ACK时继续原 `OBSERVATION_MEASUREMENT_ACK_TIMEOUT`。它记录最后一对已完整校验的ACK，不重新授予权限。所有原capture／lease／monotonic预算、校验次序、等待间隔、返回条件及真实过期／拒绝异常保持；普通executing路径保持原行为，不续期或重试动作。

隔离验证包含作者13项、独立19项及原49项，共81项通过、0失败／错误／跳过，见 `verification/gen14_reporting_independent_candidate_r1/result.json`。同组在实际GEN13基线为77通过、4个预期的新分类断言失败，见 `gen14_reporting_independent_baseline_r1/`；原拒绝、manual／stop、owner变化、错请求、错三帧和真实ROS时钟越过capture／lease截止仍按原规则拒绝。初次作者11通过／2失败报告保留：浮点测试时钟与ROS纳秒舍入不一致碰到了原截止拒绝，后只将夹具步长按9位小数对齐，生产逻辑未因此改变。

交付共3文件（一份生产源码、两份新增测试），delivery SHA256为 `617785775106e726153a8fb674341ee5ad3749e929cf5bc1078361db65eab627`；独立评审 `verification/gen14_timeout_reporting_delivery_seal/independent_review.json` SHA256为 `b2db356bc26c2ca78b9d5ecfd2e5447d5b61ce836dd89877f502ca9defcc5ca1`。主代理已安装到实际工作树，原备份与回执为 `verification/gen14_reporting_integrated_install/result.json`。磁盘来源增至892份，139个测试文件的最终实际联合在 `verification/gen14_reporting_actual_integrated/` 运行，本段没有提前声明其通过。

**GEN14错误分类修复仅已安装到磁盘，尚未加载；在线仍是已部署的GEN13、890份运行来源。** `gen13_deployment.json` 与其source manifest继续指向那次已验证运行版本，不被磁盘安装清单替换。本次未部署、未发起新抓取；算法修复仍待调查及验证，实物抓取仍未成功。本文档及两处SanStand镜像、进度元数据的更新前副本和核验保存在 `verification/gen14_reporting_installed_document_update/`，local_only、失败录制保护及温度约定保持。


### 2026-10-12 GEN14 错误分类实际联合验证完成，仍未在线加载

上述139文件实际联合现已完成：**3653项通过、0失败、0错误、0跳过**，用时341.05秒，892份来源前后SHA256一致。结果 `verification/gen14_reporting_actual_integrated/result.json` SHA256为 `bb6785251cbfe5e88f0afb8fe0595d781296f0cbf603cc0fcbc5744a21b19a70`；统一发布 `verification/gen14_timeout_reporting_release_manifest.json` SHA256为 `cb13b6cf22ca73d3f67d80aaabb092b0654d64e844583d67a27921f4ae1290f2`。隔离81项已包含于该实际组合，不重复相加。此前“运行中”的记录按当时状态保留。

该完成状态只覆盖错误分类修复及原范围联合验证。**GEN14仍未加载，在线保持GEN13的890份运行来源；磁盘为验证后的892份来源，不能写成二者已经一致。** 完整图像的外观／几何提案原因仍在调查。另行的小支持区域ORB提案诊断 `verification/gen14_small_support_features_r2/result.json` 未产生可接受方案，不作为资格通过、算法修复完成或性能保证；它使用明确标注的宽裕诊断时钟，并非原source期限回放。

本次没有部署或新抓取，当前第2轮仍为封口失败，实物抓取仍未成功。更新前文档与进度元数据、892份来源再次只读核验及三文档一致性回执保存在 `verification/gen14_reporting_verified_document_update/`。后续实际算法候选须继续通过原身份、完整mask、独立稠密验证、当前3D、来源期限和三帧资格检查；温度与local_only约定保持。


### 2026-10-12 再次恢复：共同实测面候选通过组合回归，三方完整交接回放仍待完成

用户“先暂停”已执行并写入持续任务control的paused revision7；后续“继续处理”恢复为active revision8，原持续抓取授权不重新索取。恢复时当前SDK epoch为1791870077037791822，已不同于第2轮；当前现场及反馈见 `resume_gen14_scene_revision8/`。任务、路由、机械臂与夹爪动作均空闲后，GUI自身回传确认手动等待，见 `resume_gen14_manual_revision8/result.json`。这些都是恢复时点证据，不能代替后续启动前的实时检查。本次恢复尚未部署候选、尚未新增抓取。

原完整像素诊断表明，补视必须暴露原来缺失的侧面，原约束下视线至少变化37.43266度；继续原目标优化只改善约0.02度，不能修复原整体外观对应失败。新增候选明确采用“固定共同实测面的稠密身份核对”，不声称原整个非平面物体的单一H检查已通过。源面只由运动前原已接受RGB-D、同帧精确TF、支持面和配置确定；保留原深度编码、比例、数组与绑定SHA，缺深度仍未知。当前完整物体仍须通过原几何、支持面、所有原生core归属、形状／chroma和唯一性。共同面必须实际通过原完整面IoU／retained与独立稠密纹理门，不用局部直方图短路，也不按当前匹配分数选面或缩减失败分母。新露侧面由当前真实几何检验，不要求其已经出现在旧RGB中。

实接发现相同到位帧的fragment与最终确认会重复执行同一完整mask的身份计算。候选只在一次observe调用内、全部RGB／depth／mask／valid／K／TF／支持面／配置／参考及绝对截止逐字节一致时复用纯计算结果，命中前后仍重查owner与原期限，不跨帧、lease或站位缓存授权。独立反例还发现原aggregate85%面core规则可能忽略同一whole mask内的小独立平面；现每个达到原最少实测点数的独立面都必须被计入唯一性判定，不能由大面的总占比掩盖。

统一候选27文件回归 **852项通过、0失败／错误／跳过**，5份生产源码及27份测试前后SHA一致，实际工作树892份来源尚未改变。证据 `verification/gen14_measured_face_author_r2/result.json` 已含root22项和独立89项，不重复累加。此前root初次71项失败属于新增freeze默认clock绑定与测试时钟不同轴，调用方现显式传入创建deadline的同一monotonic函数；后续5项为新夹具误用已被process消费的latest，改用实际feed返回的同源消息后22项通过。原失败报告均保留。另有最终input_lock等待期间的真实线程竞态验证正在独立补充，852项不能冒称已覆盖该新增范围。

原57帧完整producer回放首版在第三帧age0.806309秒被原0.8秒门拒绝并清空三帧计数，见 `verification/gen14_arrival_independent/candidate_full_process_v3_r1/`。去除重复计算后r2在一次真实身份拒绝清零之后，以3349994182／3879824161／4478829145（均前缀179186021）的三张实际来源完成确认，三帧提交早于原capture截止约113毫秒；真实qualified callback完成仍早于capture约88.7毫秒，后续真实新帧READY。**r2只完成producer链，remote汇合及task双ACK尚未在该回放实际执行，不能称为三方交接通过。** 完整三方回放正在接入原plan、session、PRE消息和实际producer输出，保留真实任务等待循环及其原20毫秒等待；未注入合格ACK或到位成功。

时限口径纠正：本轮封存的 `perception_execution_configuration.json` 实际 `/grasp/near_field_replan_timeout_sec` 为90.0，capture参数未配置时使用原生产默认3.0秒。此前摘要中把phase默认60秒当作实配不准确；v3诊断手写60秒属于更严的诊断phase，并非在线实配。三方回放使用原配置90秒及原capture默认3秒，未改在线参数；其他原计算、来源、求解和碰撞预算继续分别保持。任务初态仅来自原历史计划、selection、session、executing及实测settle，见 `verification/gen14_arrival_independent/task_fixture_initialization/result.json`；没有预种到位三帧、qualified ACK或phase。

GEN14错误分类修复仍为已验证但未加载的磁盘版本；在线仍是GEN13。共同面算法仍在隔离目录 `.runtime/gen14_arrival_repair_20261013`，尚未安装或部署。第2轮仍为封口失败，实物抓取没有成功证据。新旧记录继续local_only，温度查询／软件反应和关节失能命令均未启用。


### 2026-10-12 GEN14 三方实接发现剩余身份失败，固定训练像素配准仍处于候选验证

上述最终提交锁验证已补齐：真实worker/main线程竞争中，在完整源面freeze后等待最终input_lock，SDK epoch变化、SDK越界再返回、原source期限及原lease期限过期均拒绝；无变更正例只返回accepted，未伪造qualified。5项通过，证据 `verification/gen14_measured_face_freeze_lock_r8/result.json` SHA256 `5a0bceff610e47126ea7b2a7e7b44dcdc00c72a528a6b8841c61f05013b89321`，源代码前后不变。它尚未计入前述852项组合，不能把重叠测试数量再次相加。

完整三方v4首次真实执行producer计算、remote回执汇合及task原20毫秒等待后，r2仍失败：`OBSERVATION_MEASUREMENT_QUALIFICATION_TIMEOUT: pending=unknown,remote`。原3秒capture未延长，executing和requalifying的双方accepted回执已到达；实际调度选中的1791860212813457250、1791860213479100227在共同面分布纹理门拒绝，连续三帧计数清零，随后到原截止。证据 `verification/gen14_arrival_independent/three_owner_v4_r2/result.json` SHA256 `df8c9b127c5fb861bcaa28faae909351a78e8f9d08480853d7360839ed1d20dc`，候选源码未变。这是候选尚未完整修好的反例，不能拿此前producer局部成功替代，也没有重试实物抓取。

为排除只选幸运帧，已按原算法复建全部57帧的完整当前几何和mask，三方实际处理的五帧mask SHA全部逐字节相同。固定原affine提案在57帧中38帧通过原稠密门；原实测面与TF/K给定透视、再用同一稀疏点拟合affine残差的独立诊断为46帧通过，仍有11帧拒绝，因此没有将它安装为修复。坏帧整体NCC和完整面重叠已够，但稀疏匹配大多集中在面的一侧，外推偏差导致分布纹理好格不足。

主代理另做固定train-only配准诊断：运动前8像素棋盘的训练格中心4×4像素预先选定，固定六自由度affine小修正及亮度gain/intercept，固定12次函数评估上限。优化不读取验收分数来选面、模型、步数或停止条件；每次残差与数值梯度实际读取的当前双线性四像素支持都累计记录，最终验收排除所有重叠支持，原参考29个验收格分母和所有原门限不变。全部57帧的当前支持交集实际为0，56帧通过；1791860213680062055仍只有13个好格、原要求15，继续拒绝。证据 `verification/gen14_arrival_diagnosis/train_only_refinement_all57_r1/result.json` SHA256 `46ce2905179c8b57ab135ea06444eff1f49f28b907f3e4e25dbf59a3b13de3c2`。这是宽裕离线诊断，尚未证明原source期限或三方交接成功，不是可部署验收。

后续候选必须检查优化过程的每次owner/原期限、当前有效面支持以及全部训练历史与验收像素的隔离，再做真实三方和实际联合回归。remote交接后的原融合注册也在补齐真实历史初始化和当前帧缓存，不用成功stub。共同面修复仍未安装或部署，在线仍是GEN13；第2轮保留原失败结果，实物抓取仍未成功。全部诊断记录只写SanStand，未发动作、温度查询或关节失能命令。


### 2026-10-13 GEN14 固定三相位预算仍失败，重复计算热点已定位，恢复隔离修复

train-only候选已补齐每次残差／解析Jacobian前后的owner与原期限检查、固定±3参数域内全部双线性四像素有效支持、全部训练历史与独立验收像素隔离，以及拟合期间RGB被修改的拒绝。解析Jacobian与原数值差分在全部57帧判定相同，56帧通过；原1791860213680062055仍因13个合格格不足15被拒绝，没有放宽判定。此前“12次函数评估”准确指求解器max_nfev=12；数值差分版本实际可能调用108次残差，不能把12写成所有计算回调的总数。缓存优化之前的统一候选31文件929项通过，证据 `verification/gen14_measured_face_author_r3/result.json`，包含此前线程锁、完整几何和train-only独立测试；新缓存修改后不能沿用这份旧源码回执宣称最终联合通过。

v5已执行真实producer三帧确认、remote原图汇合、task原等待／双ACK／phase和原始融合注册代码。历史初始三帧由原始RGB-D、SDK／TF及已录配置重新融合，中心和相机原点与历史逐值一致；只初始化已有历史表面，没有预种到位qualified或新的融合成功。第一轮三相位发现回放每次读时钟重复构造531条历史SDK；改为按原收到时刻逐条送入，独立核验1062个事件边界和1693个历史调用时点、共5510次对照，反馈stamp／q／128条deque完全一致。修正后的第二轮固定phase0／1／2仍全部未通过，完整六次失败、SDK等价证据及诊断保存在 `verification/gen14_arrival_independent/three_owner_r2_budget_receipt.json`，SHA256 `4350ee04d26b5e08624c9d7e2e601412a52ae291d35b7a4d89fd1ebc07f0862f`。不挑选一次幸运启动作为通过，不延长原0.8秒source、3秒capture或实际90秒phase期限。

第二轮phase0遇到真实身份拒绝并清空计数；phase1在第三帧完成前capture到期；phase2取得前两帧后在第三帧新reference准备期间source过期。冻结该版本的单次cProfile表明，正常帧约73次绑定检查反复深复制／解析相机与上下文，是主要重复耗时；SDK括号本身不是主因。cProfile含自身额外开销，只用于调用归因，不作为实时预算通过证据，原文件保存在 `verification/gen14_arrival_independent/frozen_r2_cprofile_phase0/`。

用户暂停后，任务控制已写paused revision9；本次明确“继续处理”后恢复active revision10，未执行新物理动作。当前新增PRE特征／训练池复用在57帧上初始H、最终H和优化参数全部逐值相同，独立缓存边界25项通过，见 `verification/gen14_precompute_independent_r2/result.json`。纹理格提取改为一次稳定分组，保留原每格样本顺序和同一np.std阈值；12项新增等价测试及原2项采样测试通过，见 `verification/gen14_reference_texture_grouping_r1/result.json`。主要live binding优化尚在修改，必须每次读取动态状态并精确检查原值，不能按对象id或时间间隔跳过检查。

共同面／性能候选仍仅在隔离目录，未安装、未部署，最终组合回归和三个固定时序验收尚未通过。磁盘上已验证的GEN14错误分类修复仍未加载，在线仍为GEN13；实物抓取未成功。后续三方通过亦只证明确认、融合和READY链，信息增益、可执行规划及实物夹持必须各自有实际证据，不相互替代。记录继续直接写SanStand，local_only及温度／失能约定不变。


### 2026-10-13 GEN14 重复计算优化联合通过，补齐真实订阅与并发后的三相位交接验收通过

共同实测面／train-only候选及重复计算优化已完成35个测试文件的联合验证：**1069项通过、0失败、0错误、0跳过**，用时110.53秒。候选来源前后SHA一致；该次测试同时核验实际工作树892份来源没有改变。结果 `verification/gen14_measured_face_combined_memo_r1/result.json` SHA256为 `3b547d21487ad643d437132febd577e4b724894f7ad1315eee31a61ad01d2dab`。该组合覆盖原31文件及新增缓存／绑定／采样等价边界；852、929及各独立子范围与1069有重叠，不相加。原身份门、每次动态owner／SDK／来源检查、0.8秒source及3秒capture期限保持。

优化后的v5第三轮仍按预定phase0／1／2各运行一次，三次均实际取得连续三帧资格、任务qualified receipt与contact phase，真实remote融合注册和新增覆盖亦通过；整体验收仍失败，随后producer丢失到位参考并报告 `target_lost_requires_new_generation`，双方station contract为空。原失败保存在 `verification/gen14_arrival_independent/three_owner_v5_phase0_r3/`、`three_owner_v5_phase1_r3/`、`three_owner_v5_phase2_r3/`，来源冻结为 `three_owner_v5_r3_source_freeze.json`，不改写这些结果。

逐回调时序查明此处回放额外串行化：v5在同一个dispatch线程执行remote的真实phase融合，分别耗时0.472／0.311／0.337秒，期间producer没有执行机会；恢复后原第三帧参考已超过0.8秒，`frame_timing.arrival_station_invalidated_reason=source_exceeds_existing_frame_limit`。真实部署的节点独立运行，remote的phase回调还明确允许相机订阅继续进入。另查明v5仅向remote普通joint_cb送数字重建消息，遗漏了独立accepted SDK ingress与50毫秒验证定时器，也未接齐task的后续station订阅。这些是回放缺陷；没有为使回放通过而修改生产门限，也没有将这三次离线失败冒称新一轮硬件故障。

v6保留v5不变，补齐封口原bag的完整ROS消息：531条 `/alicia_d/accepted_joint_states`、3199条 `/joint_states`、1419条 `/alicia_d/sdk_command`，原header、全部字段、原接收时刻及序列化字节均保留并逐项SHA核验。清单分别为 `accepted_feedback_full_r1/manifest.json`（SHA256 `c76bb01de47351639af5642d3d81e35811d2c2dae30a9cb403f68f579cef66de`）、`joint_states_full_r1/manifest.json`（`0d982e01678440d22893061c90ce9d9aaa0fe2ad5e1d5f3745173cdbfbb5de1e`）、`sdk_command_full_r1/manifest.json`（`196ac3f0707f52b7f4f3246f4e7dfebc7574202608a5f0f808c3b24aca8a6153`），均位于 `verification/gen14_arrival_independent/`。三个topic按各自实际订阅回调分流，不互相替代；只注入截至当时原receive已到的消息，清单中的未来消息不预填。已有运动期间的输入在其原历史时刻推进并计实际处理耗时，不把几十秒的已发生回调集中收费到capture开始。

回放使用按原订阅通道保持顺序的独立worker、真实生产锁、真实释放锁的task Condition、producer原6Hz周期以及remote／task各自50毫秒station验证定时器。RGB-D、mask、object、status、SDK与phase均走实际回调；producer发布时即按固定传输延迟投递。一个持续前进的perf_counter墙钟覆盖所有实际CPU、锁等待、回调和调度耗时，不能为融合暂停时钟或为参考续期。task仅补入经真实constructor AST核验的三个None初值，没有种入到位资格、station合同或成功返回。基础调度验证及早期异常清理／后台错误保留反例通过，见 `v6_scheduler_infrastructure_r3/result.json`（SHA256 `2d65d154bee559c2b8d0ac0b913dda69dd8237d9d7f567e7c3b62468512f4c98`）；静态独立评审 `v6_independent_design_review_r1/result.json` SHA256为 `3ba7cb272b7a56e44ae34c2973ca10363e514a5b682e44bc8b453e3d35ee1a70`。

最终v6在同一冻结版本按预定 **phase0→phase1→phase2各一次，3／3通过**，没有重跑某个相位或按结果挑起点。每次完整接收原57对RGB-D，remote实际color／depth回调各57次；原latest-only处理可以跳过中间图像，不声称57帧都进行了producer计算。三次均在原3秒capture内取得实际task qualified receipt，完成真实融合 `REGISTERED` 和新增实测覆盖（69／67／62个新占用voxel），随后各有5帧 `tracked READY`，最终producer、remote、task三方station合同逐字一致。所有实际处理帧的最大source age分别为0.590／0.596／0.660秒，READY最大为0.570／0.571／0.576秒，均未越过原0.8秒；没有first_loss、task正故障或worker错误。完整原时间窗结束后停止新timer，保留原输入和传输调度排空在途回调，实际耗时仍计同一时钟，最终所有线程均已join。

统一矩阵 `verification/gen14_arrival_independent/three_owner_v6_r1_matrix_result.json` SHA256为 `95cf6f5567360a874fbd6d1fec1d0c214e89984960ab1641893fba759ff29d0c`；来源冻结 `three_owner_v6_r1_source_freeze.json` SHA256为 `447093437cb88010e23a7744c4ef3b75e1dbe894e616b9724ad1fcf36b127d00`，204项来源在三次回放前后全部保持。三相位原结果SHA依次为 `1f22f2862cd1d112dfc264c324c8d87475acbd9fd5344f9d49bae1b15ae368be`、`912cc25e78c65c2adf3469eda1b306a56aa0fcb4f82bbdb23076ea5fb4cbd421`、`394055dbc9f813d87576d7dfb947d9043b1359cea1dc7c3c7fb9443297700a8a`。该模型保留共享Python GIL和偏保守的timer忙时调度，原定时器创建相位未录下，明确使用固定模拟相位；结果不是网络端到端或在线硬件性能保证。

**本段状态以本次离线矩阵完成时为准：共同面／性能候选尚未安装到实际工作树、尚未部署；主代理下一步负责交付独审、实际161文件联合回归和运行部署，并另行追加结果。** 已验证的GEN14错误分类仍只在磁盘，在线保持GEN13。本次通过只证明确认、原融合／信息增益、三方站位接纳和后续READY链，没有调用规划RPC，没有验证可执行抓取方案，没有执行新物理动作，更没有实物抓取成功证据。原第2轮和所有离线失败完整保留；local_only、SanStand主存储及温度／失能约定保持。三份文档的更新前备份、追加内容和一致性核验回执保存在 `verification/gen14_v6_pass_document_update/`。


### 2026-10-13 GEN14 已安装到实际工作树，161文件联合4470项通过，运行重载另行验收

上述隔离候选现已完成正式交付独立评审并安装到实际工作树：共16份变更，统一来源增至904份。安装回执与原文件备份见 `verification/gen14_common_face_integrated_install/result.json`，SHA256为 `84d10564638b8b59ab4682504ca8ba428dfd55e190afb69dcab703f8d2e87be9`；交付独审 `verification/gen14_common_face_delivery_review/result.json` SHA256为 `43cc39a1149350a30808e15038fc2928a53011d2d77f3e32cc6e57284c79d3de`。独审已复核冻结来源、原始消息、真实ACK／三方合同、完整57对窗口及实际融合与producer并发，原失败历史继续保留。

实际工作树的最终161个测试文件联合已完成：**4470项通过、0失败、0错误、0跳过**，用时355.60秒，904份来源在测试前后SHA一致。结果 `verification/gen14_common_face_actual_integrated/result.json` SHA256为 `d96803ce99f459963e3a1eb445f17dd1db281310450374914c4a3848647cd135`，明确 `production_worktree_tested=true`；安装、实际测试和发布清单的904份来源完全一致。此前1069项隔离组合及各独立子范围与本次实际组合有重叠，不重复累加；固定三相位回放另保留其完整证据，不用测试数量替代回放。

统一软件发布已封存为 `verification/gen14_common_face_release_manifest.json`，SHA256 `83341ac7afafa746fbc61d50640a7045f404db43161b929cd4b3157fecb41fb1`，记录reviewed、installed和passed均为true，physical_grasp_verified=false。共同面／train-only配准、重复计算优化及错误分类现已在实际磁盘完成组合验证；原0.8秒source、3秒capture、已录90秒phase和原几何／身份／执行门保持，不通过延长期限或省略动态校验取得通过。

**软件安装与回归完成仍不等于运行部署完成。** 主代理已在 `gen14_reload/` 发起计算节点重载，本段尚未纳入其最终部署回执，不提前声明在线节点已加载GEN14；运行版本、硬件进程保持及部署验收由主代理随后追加。本次软件验证未执行规划RPC或新物理抓取，实物抓取仍无成功证据。三份文档更新前备份、追加文本与SHA一致性回执位于 `verification/gen14_actual_4470_document_update/`；本次只更新操作文档和镜像，未改变源码、进度元数据、local_only或温度／失能约定。


### 2026-10-13 GEN14已部署；第3轮通过补视资格，但真实任务重复复用站位时失败，继续GEN15

4470项实际联合通过后，已重载感知、remote及任务三个计算节点；八个硬件／GUI进程身份不变，904份来源逐项一致。统一部署 `gen14_deployment.json` SHA256 `6769c7cfef5723b3c16ee7291dcdada92d33ebc2a815dcc4c5e78670fe29591b` 绑定发布 `verification/gen14_common_face_release_manifest.json`（SHA256 `83341ac7afafa746fbc61d50640a7045f404db43161b929cd4b3157fecb41fb1`）。新画面、20个SDK样本与epoch、动作空闲和录制检查通过后，沿用用户持续授权启动第3轮，generation4／stamp1791880718524907350，实验目录 `grasp_records/20261013T083836.150489Z_f4adfdd7ede44edeb725b4bfd7ac4f32`。未索取或冒称新增单轮确认。

本輪原requalifying请求为1791880762780848741；三个合格来源为1791880762822273731、1791880763089883804、1791880763359292745，约0.85秒完成双qualified确认，后续qualified通知亦确认。实际到位CAD包络检查通过，随后任务在重复复用刚发布的近场phase时返回 `QUALIFIED_STATION_PHASE_OWNER_CHANGED_OR_EXPIRED`。原任务FAILED消息及StartGrasp响应一致，runner退出6；录制中只有一次机械臂观察goal及其result，没有夹爪goal。没有实物抓取成功。

原phase1的header纳秒为1791880763644008874。setter使用float(ns)*1e-9得到1791880763.644009，而复用检查使用ROS to_sec()得到1791880763.6440089；相差一个ULP（约238.418579纳秒），严格相等条件确定失败。当时原deadline1791880853639396667仍余约89.6秒。诊断 `verification/gen15_qualified_owner_diagnosis/result.json` SHA256 `701e9ac39fc305b23e3a00cba2a7ebe0f4c125f6e9a840397a0c1119840a12b9` 绑定原wire及生产源码；无当时内存转储，未对其他内存谓词冒称已观测。

v6三相位的资格、融合、完整57帧和三方站位合同通过事实保持；它没有进入base任务的到位包络／距离检查后的第二次ensure调用，也没有完成后续真实规划。此次实测暴露了该覆盖缺口。GEN15将用原失败Unix纳秒时间戳补回真实后继调用，并同时修复接触接纳路径中将纳秒deadline转float再转回纳秒进行身份比较的同类缺陷；原精确期限和真实owner变更拒绝保持，不加epsilon。当前修复仅在隔离候选中，未安装或加载。

第3轮10份manifest文件和keyframes.bag已核验封口；原bag SHA256 `7fd3e8223a766c599904c0e0a7afb735b8f543c60f2b3c70a0da1559234270f1`。sidecar的包装父进程退出后，原即时finalize断言先于子写入者最终改名；原失败result保留，稍后正常出现的7,208,670,120字节bag已独立读取索引、核SHA256 `4357adcdb9cf011ea5adc3a07d2e36c8cc09788dc196c744e6cd2a0219d90ee4`、4199张RGB及无打开写入者，证据 `attempt_3_full_frames/close_verification_after_wrapper_assertion.json`，不伪写原wrapper退出成功。辅助封口等待修复与6项实文件竞争测试另在隔离候选中。

`attempt_3_seal/` 保留完整封口核验及旧元数据，当前问题保护清单107增至109条；没有删除录制或上传。任务和动作空闲后GUI手动回传已核实，见 `attempt_3_after_manual/result.json`。该状态仅为收尾时点，下一次动作仍需新现场检查；持续授权保持active，继续软件修复。


### 2026-10-13 GEN15 候选370项联合通过，补齐真实任务后继的v7三相位3／3通过，待实际安装回归

GEN15隔离候选已完成9个测试文件的联合验证：**370项通过、0失败、0错误、0跳过**。其中已包含独立121项Unix时间／owner边界、作者20项精度用例及主代理6项录制封口用例，不能再把这些子集与370相加。结果 `verification/gen15_owner_focused_r1/result.json` SHA256为 `fb2a6e16a43e0be203b6eba6997ac63687a78301b7c2f05a33635640737155c7`；独立121项结果 `verification/gen15_owner_independent_r5/result.json` SHA256为 `8253636c5d5a37c3e98a5edb8334d0ddee8ae729b18bece7b51b848c86825907`。该联合确认两个生产模块实际从GEN15候选加载，原已安装904份来源在验证期间未改变。

第3轮已证实的缺陷是phase起点缓存与原消息的转换不一致：原header `1791880763644008874` 经 `float(ns)*1e-9` 与 `Time.to_sec()` 得到相邻浮点数，严格相等检查误拒绝。候选从最终发布的ROS phase统一生成秒缓存，并保留起点与deadline的精确整数纳秒缓存；重复复用继续核对原request、ACK、selection、lease、session、plan、活动状态、phase-id、完整phase消息、ROS原截止和monotonic原截止，同时精确核验两个整数缓存。没有加入epsilon、延长原期限或用新的观测刷新旧owner。

**精度范围更正：原第3轮deadline `1791880853639396667` 恰好能从秒数往返还原；不能说接触审计的deadline比较在本轮也必然失败。** 接触接纳读取者的同类问题是合法相邻值 `1791880853639396668` 转成秒再转回纳秒会落到 `1791880853639396667`。这是独立反例证明的潜在缺陷，不是本轮第二个已发生故障。候选改为核对原消息的整数起点／deadline及统一秒缓存；缺缓存、真实1纳秒篡改和真实到期仍拒绝。审计 `.runtime/gen15_qualified_phase_timestamp_20261013/precision_audit.md` SHA256为 `2b5e5f65e52bc351032ed74c2dcae2a61a1f107585653091c6ab48cf4f6d9aad`。

原R3后继回放从原已完成观察运动和实测settle的历史边界开始，逐项核验原计划、请求、ACK、ObjectPose、TF与accepted SDK字节及接收时刻。真实task measurement finish首次创建原phase，再执行base `_execute_grasp6d_plan` 中未改动的到位分支：实际CAD包络、实际source-time RGB-D工作体积、第二次 `_ensure_qualified_station_phase`。同一原始Unix时间戳下，基线在CAD和范围均通过后精确重现 `QUALIFIED_STATION_PHASE_OWNER_CHANGED_OR_EXPIRED`，候选两次ensure成功且原phase、header与deadline不变。基线 `verification/gen15_task_continuation_baseline_r2/result.json` SHA256为 `a960b46a687120ec22887ee448e9a76efe1e29895f389ae16209dac0ab95f53f`；候选 `verification/gen15_task_continuation_candidate_r1/result.json` SHA256为 `66a5bef94a0ff709c7134093c4c0a942888ecbb326bd56ab6f07648d9f622c51`。这组是确定性原事件时间验证，不作为墙钟性能证明。首次r1因回放遗漏真实 `_strategy_config` 而走旧径向范围检查的失败亦保留；r2补回原工作体积配置适配，没有改生产门限或伪造范围成功。

封存v6原3／3通过事实不变，但范围仅到资格／融合／三方合同，未走真实任务后继的二次ensure。新v7补回实际 `_freeze_execution_plan` 的原wire digest、真实 `_strategy_config`、TF及上述原到位分支；原每订阅worker、真实锁、单一持续前进墙钟、固定6Hz和排空逻辑均保留，相关基础函数AST逐项与v6一致。固定 **phase0→phase1→phase2各一次，3／3通过**：每次恰有两次实际ensure，CAD与RGB-D范围通过；原phase所有者、header、deadline和纳秒缓存在完整窗口结束后仍一致。各完整接收57对RGB-D，三方最终station合同相同，后续各有6帧tracked READY；最大READY source age分别0.629／0.594／0.528秒，新增实测覆盖102／102／67个voxel，均无first_loss、task正故障、worker错误或遗留线程。所有CPU与等待仍计同一实际墙钟，原0.8秒source、3秒capture和已录90秒phase保持。

矩阵 `verification/gen14_arrival_independent/three_owner_v7_r1_matrix_result.json` SHA256为 `c3e08d1dfa0a0284b6f2e2f246c29c4b0702a8293c43ee4ac4d0bca88d741dc5`；来源冻结 `three_owner_v7_r1_source_freeze.json` SHA256为 `5e296ed87d2e5431a561619c29d1ea3611b86078fff183e1f146ca64c8e27542`，6088份候选／依赖／原消息与图像文件在矩阵前后全部未变，实际task base和observe_return_task均从GEN15候选加载。三相位原结果SHA依次为 `070b06366958a300e25d7427d19288f236fbe5929ff614cb5c707433eacc5f8a`、`942f0de2383ac4b5704fb3a865efb6e189af3c4914a55f25068c98777dbfbf01`、`ab6d85bac0e455b96ad84cc0d6e88efe17f1ffa38e7985b77dca832259368284`。回放只提取真实到位分支，不声称完整运动前序、remote候选规划RPC、接触方案审计／执行或实物成功均已验收。

正式候选清单 `.runtime/gen15_qualified_phase_timestamp_20261013/delivery_manifest.json` SHA256为 `c922c29f54ec32e3219e793805ca0185b803a119297494113918cb46cc82e4cd`，共7份变更：两份阶段生产源码、三份新增测试、一份既有测试fixture及录制sidecar脚本。sidecar修复只在包装父进程退出后等待实际bag封口完成，保留原非零退出／失败事实；原第3轮记录不改写成功。

**本段状态以GEN15候选验收封口时点为准：尚未安装到实际工作树、尚未在线加载；在线仍为已部署GEN14，实际165文件联合回归尚待主代理执行。** 独立交付复核、实际安装／联合验证、发布和部署由主代理随后追加，不用候选370项或回放替代。第3轮及所有旧失败保留，实物抓取仍未成功，本次未发送任何新动作。三文档更新前备份、追加内容、绑定证据与SHA一致性回执在 `verification/gen15_candidate_verified_document_update/`；未修改源码、交付清单或进度元数据，local_only、SanStand主存储及温度／失能约定保持。


### 2026-10-13 GEN15 实际第二轮4663项通过；首轮25项失败保留，计算节点重载尚未完成

候选370项和v7固定三相位通过后，安装到实际工作树的165文件首次联合**未通过**：4663项中4638通过、25失败、0错误、0跳过，用时360.20秒。原 `verification/gen15_phase_timestamp_actual_integrated/result.json`、pytest.xml及pytest.log完整保留，result SHA256为 `847d9b6edb34a6a2822547f4c0d4d75614a57b3e35091f1eecf87591afba8c87`。25项均来自 `test_gen11_contact_compute_admission_independent.py`；旧夹具以__new__手工复制active phase，仅写秒缓存，没有真实setter新增的起点／deadline整数纳秒缓存，严格接触owner门因此拒绝。部分负例在其变异之前的正向准备阶段即被拒绝，不能把这些失败解释成负例已充分执行，也不能以候选验收替代实际完整联合。

补充修复只在该旧夹具中从已有phase的 `header.stamp.to_nsec()` 和 `deadline.to_nsec()` 增加两项赋值，与真实setter对同一已发布phase产生的缓存一致。删除这两项赋值后整模块AST与原件完全相同，原40个正／负案例及断言未改；没有更改生产门、返回成功stub或放松owner检查。原40项加7项真实setter等价、缺失缓存及±1纳秒拒绝验证共 **47项通过、0失败／错误／跳过**，证据 `verification/gen15_gen11_fixture_proof_r1/result.json` SHA256为 `bf903b83e643099a07cb28c543617f3ab4193c2c6d1ef8a045b99a954f6108f2`。补充交付 `.runtime/gen15_gen11_fixture_completion_20261013/delivery_manifest.json` SHA256为 `28bead53ed15fd8485d1d13e04b32cf803e8ce5fe949886f3a6a76cda5e5457f`；原七文件交付清单及首轮失败结果保持不变。47项、此前370项及完整实际组合有重叠，不把它们直接相加。

该单测试文件补充已独审并安装，安装回执 `verification/gen15_phase_timestamp_integrated_install_r2/result.json` SHA256为 `6c912406bb335470f4d50ce2777f84fe56e340894d27a160c9def98978bafc0c`。在相同165文件范围再次运行实际联合：**4663项通过、0失败、0错误、0跳过**，用时357.38486秒，908份实际来源前后SHA一致；result明确 `production_worktree_tested=true`。第二轮结果 `verification/gen15_phase_timestamp_actual_integrated_r2/result.json` SHA256为 `2b83f9f6960f54d3e88648fa9375d9526457d44f3baa9cd460476ebdc7d4ee35`。安装、实际回归和最终发布的908份来源逐项相同，原生产修复、原范围门限及v7回放证据均保持。

最终软件发布 `verification/gen15_phase_timestamp_release_manifest.json` SHA256为 `16fe70b0bc0699df1ea0faac978dafa2bc4b45d7b0be483e1a679ab9eaefb11b`，reviewed／installed／passed均为true，physical_grasp_verified=false。它另绑定原封存来源清单 `verification/gen15_phase_timestamp_sealed_source_manifest.json`（SHA256 `8cefa97079b2923e0cd12f3a4732c1b0d7d819c97bf1f49707cf5e98e02474bc`）；历史sealer标签保存在legacy_sealer_kind，最终发布标签明确为GEN15，不改原封存证据。908是整套软件来源清单数量，包含测试／资源，不表示908份文件都被进程加载。

**上述软件验收完成，不等于GEN15已在线部署。** 首次 `gen15_reload/` 因GUI手动状态为false，在计算进程变更前停止，原result记录deployment_passed=false（SHA256 `2744cd992105e9391fc4e40f4dfd905e4a1070c39ac6030427757a8f61aa50e6`）。主代理随后只读确认任务FAILED且inactive、机械臂action为SUCCEEDED、夹爪活动列表为空，再恢复GUI手动等待；`gen15_before_reload_manual/result.json` SHA256为 `8eac4ef8eb113c073465c7f60561d5dff324659f58249f395b856811d5848fb9`，GUI自身publisher回传true已核验。

第二次 `gen15_reload_r2/` 在关闭RPC应答期间出现 `ConnectionRefusedError`，原result继续是deployment_passed=false（SHA256 `6439c2daddbf2334086d9fc0c9f6c0c2602ced1c293ab12f45fcab507deec4f1`）。该异常不等于没有发生关闭：旧task和remote已clean exit，日志末尾有各自正常退出记录；感知节点仍为原进程，八个硬件／GUI等保留进程身份未动。本段记录此部分重载时点，恢复脚本正在独审／待执行，不能把当前写成旧GEN14整套运行完整，也不能写成GEN15整套已加载。后续完整部署须用新的重载验收回执另行追加。

本次未发起新抓取、机械臂运动或夹爪动作；第3轮仍为已封口失败，实物抓取仍未成功。三份文档的更新前备份、追加文本及SHA一致性核验在 `verification/gen15_actual4663_partial_reload_document_update/`。本次仅同步文档和审计，未修改运行进度元数据、生产源码、交付清单、local_only或温度／失能约定。


### 2026-10-13 GEN15 计算节点恢复部署通过，908份来源与4663项实际验收绑定

前述首次25项实际回归失败、两次原reload失败及其部分退出事实均保留，未覆盖旧result。后续首次恢复 `gen15_reload_recovery/` 又因GUI手动状态false在变更前退出，shutdown_requests_sent为空、new_launch_started=false；该失败不是一次新的节点关闭或启动。随后手动恢复辅助脚本初版沿用“发布连接数至少5”的旧假设，在发布前失败，原日志保留在 `gen15_before_recovery_manual/`。当时task／remote已退出，master显示实际模式消费者只有driver、motion_gateway和GUI三个，加本次回执订阅共4个；不足5不能单独证明缺失当前必要消费者。修正后的操作脚本逐名核对三个必要消费者，并等待GUI自身publisher回传true后才继续。`gen15_before_recovery_manual_r2/result.json`（SHA256 `3c878e56b6cab70e222c3d4a72c27d6f7f21f2c7b364bf154fe3769c5c63507b`）记录四个连接、任务已退出和GUI回传，恢复成功。

GUI此前false状态的最初来源仍未知。只读观测 `gen15_manual_mode_observation/result.json` 只能证明当时GUI的latched消息和参数为false，不能据此归因用户操作、定时器缺陷或某个未记录事件。此前手动恢复成功仅证明各自回执时点，不将旧回执当作后续持续有效状态。

最终恢复 `gen15_reload_recovery_r2/result.json` SHA256为 `aa881d80e22e2c3c4dc33393b89ec8f0f6e0664c50076b472c38bf88016ecf65`：先按原PID／start_ticks确认旧task与remote已退出，再关闭仍在运行的原perception；三个旧计算进程均已退出，随后启动三个新计算进程，task／remote的reset ACK均为ready且selection一致。运行unit为 `alicia-observation-measurement-1791884445801392227`。八个保留进程的PID、start_ticks与parent_pid前后完全一致，硬件、控制、相机、路由和GUI没有为本次计算节点恢复而重启。该恢复回执未发送运动、力矩或温度查询命令。

统一 `gen15_deployment.json` SHA256为 `9158a4d1425b71e5dc416d37c515e5e8f11dc5101cd80c6dff82998336ff2ee1`，明确deployment_passed、whole_patch_deployed、compute_reloaded及hardware_processes_preserved均为true。其908份来源与最终发布 `verification/gen15_phase_timestamp_release_manifest.json`（SHA256 `16fe70b0bc0699df1ea0faac978dafa2bc4b45d7b0be483e1a679ab9eaefb11b`）逐项一致，并绑定实际165文件、4663项全部通过的验收。**GEN15现已完成软件安装、实际联合与计算节点加载验收。**

主代理随后将两处运行进度元数据统一到该发布／部署证据，同时保留第3轮失败、首轮软件回归失败及用户授权历史。核验与更新前备份位于 `verification/gen15_runtime_progress_1791884471700293524/`，result SHA256为 `8ea71767b28baf4d145ff3cfa54ec8c2a3fbf42e9f713101447a6cb9591426b1`，status=committed、passed=true。进程身份与selection是该核验时点的快照，不能替代后续动作前的新现场／录制检查。本段以部署完成、随后第4轮现场preflight与抓取准备开始时点为准，**没有新增实物抓取成功结论**；后续每轮仍由独立录制和实际结果判定。

三文档本次追加前备份、文本及SHA一致性回执在 `verification/gen15_deployed_document_update/`。本次文档同步没有修改运行元数据、生产源码或交付证据；local_only、SanStand主存储和温度／失能约定保持。

### 2026-10-13 GEN16：R4 补视交接失败分析与成功优先修复（实际组合回归与部署通过，待实物验收）

本段证据根目录为 `ros_log/full_latest_contact_admission_20261013T011103Z/`，下称 N。原第4轮 `grasp_records/20261013T094326.672604Z_0b39e4a7c8514b43b66ae320b8aaeea8` 已封口，结果仍为 **FAILED，实物抓取未成功**；只有一组机械臂 goal/result，没有夹爪动作。原录制、失败结果和后续诊断均保留。

反复出现“超时／交接失败”的外层表现，不等于每次都是同一处缺陷重现。此前回放没有覆盖真实任务的第二次 phase 复用，GEN15 已据原 R3 纳秒时间戳补齐该分支验证；R4 又实际经过资格确认、CAD／范围检查后的 phase 复用，并在失败前产生191帧 READY，进入了接触候选和下一次补视交接。本轮暴露的是后继并发与调度问题，不能继续把所有失败归为视觉资格不稳，也不能用局部回放或大量单测通过代替完整任务验收。

原 wire 的先后顺序显示：补视请求 header 为 `1791884708285209894`；任务 FAILED header 为 `1791884713327177524`，之后才撤销 phase；producer 首次失效为 `1791884713381796836`，原因是 `station_compute_retirement_context_changed`。这帧在原目标有效时已开始处理，跨过任务撤销后才被作废，source age 约0.077秒。故该 perception 失败是取消后的结果，不是本轮 ACK 超时的先因。remote 的工作排空约5.965秒，迟到的原 ACK 为拒绝／TARGET_CHANGED，不能改记成成功 ACK。证据见 `N/verification/attempt4_producer_timeline_r1/causal_analysis.json`。

生产参数刷新还存在可确定重现的并发覆盖：旧代码对实例浅拷贝，完成参数处理后拿旧副本与当时 live 字典作差，会把期间回调更新的 pause、session、反馈或控制权状态误当作“配置变化”写回。该缺陷与 R4 暂停后仍续发请求的时间线吻合；原 bag 没有记录全部进程内写入，不能据此声称已经逐条重现所有历史并发失败。

本轮合并五项修复，目标是保住可行方案和合法补视机会：

1. **参数更新只提交本次实际写入的配置字段**，保留并发回调的最新运行状态，原参数校验和锁继续生效。
2. **最终配准后重新调整接触高度**：在同一连续可行区间内平移四阶段姿态，计入原配准修正限额，再做几何、实测接触和原严格路径检查；没有把真实碰撞或高度余量不足改成警告。R4 两个被拒候选可由原区间推得约0.244／0.259毫米的居中调整；本次解析复现未使用完整融合点云，只证明区间重建与候选调整可能有效，不证明 R4 原轨迹已可执行。
3. **第一个完整严格合格的接触候选出现后停止额外8秒比较**，随后仍重新构建并检查发布条件。审计明确只在已检查候选内选择，不宣称全局最优；过期、撤销或来源失效仍不能发布执行方案。
4. **取消“两家族即提前结束”和无条件延后高度重验**，继续同站位候选及原高度替代。检查点保持 `min(开始+30秒, 原 phase 截止−2秒发布余量)`；原阶段余量短时可早于30秒。新版 v2 补视证据至少需要一个当前请求的真实绑定失败 family、真实 grasp 碰撞回执、独立实测缺口、注册增益，以及确实存在的未检查尾项；两家族既不是上限，也不是补视硬下限。所有真实 family／计数保留，不能把未查高度称为失败；v1 历史验证不变，未知版本拒绝。证据不足时仍使用原 phase 后备搜索，因此不承诺每个请求均在30秒结束。
5. **session ACK 的5秒改为一次等待告警**，在原 session／适用 phase 和一次确定的单调时钟上界内继续等真正匹配的 ACK；拒绝、控制权改变或合同到期仍终止，不能循环续期。等待 ACK 本身不授予动作权限，提交时的来源与执行检查保留。

门限审计据此区分调度启发式和执行依据：8秒附加比较、两家族停止及本次 session ACK 的5秒适合上述调整；原0.8秒来源鲜活度、3秒到位采集窗口、目标／控制权／真实 SDK／碰撞检查不能仅因耗时就变为警告。原 R4 的90秒 phase、其他路径的60秒合同和最多2次观察本轮未统一放宽；增加观察次数须同步 session 与消费者合同。另一类双端 measurement ACK 的5秒以及读取绑定审计的3秒等待仍列为待优化项，应按协议状态评估可恢复等待与重新取鲜活方案，不能与 session ACK 混用。完整审计见 `N/verification/gen16_gate_rationality_audit/`。

五项候选独审和针对性验证通过后，实际176个测试文件首轮联合 **5636项中5631通过、5失败、0错误、0跳过**，用时412.57秒，结果仍为未通过。原 `N/verification/gen16_success_first_actual_integrated/result.json` SHA256 为 `b445d6b7cc7a444f624c66a78e7a1ce37be9b4c63ba36a4b5ba26a785daf270d`，日志/XML不覆盖。其中4项仍断言“存在补视机会就必须延后高度搜索”，与此次明确改变的调度策略冲突；只更新该测试预期，保留参数／原期限透传，新增真实 mode＋base 高度流程的检查。另1项的测试辅助函数未持 stream 锁读取 busy／metrics，在真实 worker 已设 busy=false、但同一锁事务尚未释放 direct slot 的间隙提前返回；修订后观察整个完成事务，不等待或伪造 slot=None，故意移除真实清理仍会被原断言抓住。这两项都是测试修订，没有为通过回归再改生产门。

高度测试补充60项通过，worker同步补充21项通过；与候选和全量范围重叠，不相加。高度用例使用真实调度／高度 proposal 代码，但其几何门、控制权和严格服务边界含测试替身，不能称完整物理验证。补充包分别为 `.runtime/gen16_height_recipe_policy_test_20261013/` 与 `.runtime/gen16_worker_completion_fixture_20261013/`；失败历史与独审限制继续保留。

最终实际工作树回归为 **5640项全部通过，0失败／0错误／0跳过**，176文件，用时394.42秒，915份来源前后一致。回执 `N/verification/gen16_success_first_actual_integrated_r3/result.json` SHA256 `92c66651fb064351130badeeb823ae11123d74dda7f1f3fc37c2313ae520a4ed`。首轮失败未覆盖。

最终发布清单为 `N/verification/gen16_success_first_release_manifest.json`（SHA256 `97f9805ab90d1e6e6524583c26e52b7de9d1f7ae2defe4141d1195f6b9f13e3d`）。计算节点重载与双端 reset ACK 核验通过，保留8个硬件／相机／上位机相关进程，部署选择代次仍为5；`N/gen16_deployment.json` SHA256 `3ad18767f245e362a2c9be375712c2e4484659e474177f6fc9532522b17d0a99`。旧代次的实物失败结论保留，GEN15历史回放字段已转存历史，不冒称GEN16当前回放结果。部署前最新手动确认见 `N/gen16_before_reload_manual/result.json`。

**GEN16 已部署，实物抓取仍待下一轮实际验收。** 用户已确认停止模式操作并移开双手；下一步采集新现场、关节反馈并建立录制，不沿用旧姿态证据。新实验及夹持结果之后按真实回执追加。

实验记录继续直接保存在 SanStand，`local_only`、不主动查询温度及不发布关节失能命令的约定不变。

### 2026-10-13 GEN16 第5轮实测失败封口与后续注册边界修复

第5轮记录为 `grasp_records/20261013T112006.120233Z_d2b04edd648643b4b57d8e9febc4d88c`，代次6。补视交接已通过、两次观测动作完成，到位资格通过；request27 在50项候选中找到首个严格合格候选 `e45a537b1140dea7b17d8315`，停止附加比较。该结果仍是 preview，任务尚未接纳或发出夹爪动作，不能写成可直接执行或实物成功。

实际失败为 `STATION_CONTACT_PROVENANCE_INVALID: station_compute_wait_registration_too_late`。任务的已登记当前源为 `1791890537728708505`，几何期限为 `1791890538528708505`；对应日志头 `1791890538537096500`，晚约8.4毫秒，并非精确验证入口时间。首次有效检查与后继读取时钟跨越期限，被错误升为来源违规；原身份和阶段尚有余量。录包者在失败前已收到下一条 READY，不能等同任务回调已经完成。远端出具的历史 receipt 在当时有效，后续本地当前源已滚动，二者不能混用。

主录制14个manifest文件已核验，动作计数2组arm goal/result、0 hand goal；全帧录像6,535,449,414字节，SHA256 `56c95b9d57ec1fbcdbf4952ba5654e678b893b87cd64b292be674157e9c9107c`，已关闭且无可访问的打开写入者。封口 `ros_log/full_latest_contact_admission_20261013T011103Z/attempt_5_seal/`，原消息提取 `verification/attempt5_registration_timeline_r1/`；仍列为当前定位保护资料，不清理。上位机手动恢复回执为 `attempt_5_after_manual/result.json`。实物结果保持失败。

正在修复任务复验、最后冻结提交与远端receipt出具的同类跨窗行为：等待新测量再检查，保留原候选及期限，不用过期几何执行，不隐藏真实身份/控制/来源错误。任务侧165项针对性通过，完整组合回归、部署和下一轮实物验收待补。详见 `2026-10-13-grasp-gate-rationality.md`。

## GEN17 组合修复已部署，等待新的实物验收

已登记测量复验、最终冻结前提交、远端回执出具的超窗改为在原合同内等待新测量；补齐登记时钟配对、copy／状态通知原子检查，以及冻结后精确站位退役的历史审计交接。两个退役消息顺序的真实三参与方反例均保留；历史证明不恢复几何或动作权限。详见同目录 `2026-10-13-grasp-gate-rationality.md`。

实际工作树组合回归为 5704 项全部通过，181 个测试文件，920 份来源在测试和部署核验中一致。三个计算节点已重新载入，八个硬件／相机／上位机相关进程保持原身份。回执分别见 `verification/gen17_station_admission_actual_integrated/result.json`、`verification/gen17_station_admission_release_manifest.json` 和 `gen17_deployment.json`。此处确认的是软件验证与部署；第5轮仍为已封口失败，尚未新增实物成功结论。
