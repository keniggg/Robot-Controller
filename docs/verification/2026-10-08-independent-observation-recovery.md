# 2026-10-08 独立补观察与支撑可见性恢复记录

## 状态与记录范围

本轮目标是让已测得缺失侧面、需要补视角的普通 `unknown + observe_return` 任务及时规划观察动作，随后使用新观测重新规划完整接触抓取。此前一次性抓取已有实物通过记录；本轮不能据此声称修改后的所有一次性抓取场景均已完成实物回归。

当前记录状态：A+B 生产代码和契约已实现，组合回归为 **2,854 passed、1 skipped、8 warnings**，用时 60.92 s；功能部署已通过最终回执核对。gen32 在 B 分流前失败；随后两轮 15 fps 完整负载实验 gen33、gen34 也未成功。gen34 已进入普通独立补观察并获得六次 MoveIt 规划成功日志，但没有完整合格计划或运动。当前维持 15 fps，尚未完成补观察运动、回程或最终实物夹持。新失败审计补丁另有 **145 passed**，本文更新时待部署，不能混作前述组合回归的已部署版本。

记录根目录：`/media/zhuyupei/SanStand/实验记录/Robot-Controller`。

下文 `SESSION` 表示该根目录下的 `ros_log/supplement_grasp_20261008T103117Z`。源码和本文留在项目，证据及本文镜像在外盘，采用 `local_only`；本文不授权上传、自动删除、温度查询或任何机械臂动作。

## 已定位的规划依赖与 A+B 修改

原普通补视角链路存在两段可以分开处理的依赖：初始 `initial_contact` 可能先经过完整接触候选生成/学习推理/硬门复核才触发补观察；进入 `observation_prefix` 后，普通观察候选仍可能依赖接触候选及其假设接触序列。已经有真实缺失侧证据时，等待这些工作既延长规划窗口，也使“先去看清楚”依赖当前视角下的接触候选。代码检查证实了该耦合；它不证明所有历史失败都由该耦合造成。

### A：普通补观察使用独立观察输入

`mode_aware_remote_grasp6d.py` 对普通 `BILATERAL_SURFACE_EVIDENCE_MISSING` 前缀启用独立观察准备、接受和审计。`silhouette_observation_planning.py` 共享已有独立单姿态规划流程，新增合法来源和角色 `measured_observation`，保持普通会话的 `diagnostic_observation_only=false`。

- 发布 `FAR_FIELD_OBSERVATION_PLAN`，仅有一个观察姿态；不提供 approach/grasp/lift 接触姿态，不伪造接触几何通过，不赋予夹持宽度或接触执行权限。
- 独立输入绑定新请求的真实图像源、当前目标、几何、相机内参、TF、手眼关系、支撑点和 OBB。该准备路径不调用 WSL 学习推理、接触候选准备或历史抓取跟踪器。
- 新可达 FK 观察候选允许在接触种子和原观察格点均为空时产生。候选仍逐一经过原观察方向、包络、工作体积/FOV、真实支撑可见性、缺失侧增益、基线和已访问视角检查，再经过原严格 IK、碰撞、整条路径和跟随误差等执行验证。
- 原普通 `observe_return` 的起点、会话、回程及后续接触生命周期保留。只有取得新证据并重新规划、接受完整四姿态 `CONTACT_EXECUTION_PLAN` 后才能进入接触动作。原 silhouette 诊断分支仍使用窄语义，不借此获得回程或接触权限。
- 观察候选比较沿用原每请求预算和严格验证预算；不能继承旧请求的比较截止时间，也不能通过新增候选族绕过限额。

对应消费端同步支持独立单姿态契约：`rich_plan_integrity.py`、`observation_path_guard.py`、`grasp_task_node.py`、`runner_contract.py`、`mode_aware_grasp_runner.py` 和 `mode_aware_grasp_task.py`。执行器继续使用原动作执行链；单姿态阶段不解包假设接触姿态，后续接触必须重新绑定完整计划。

### B：确切缺失侧且零几何候选时，在学习推理前转补观察

基础 `remote_grasp6d_node.py::_prepare_and_predict` 在冻结几何候选生成之后、GraspNet 输入构建/推理之前加入默认无作用的模式 hook。普通模式在 `_observe_return_measured_view_preflight` 同时满足以下条件时才转换阶段：

1. 当前为 `unknown + observe_return`、`initial_contact`、有效近场阶段，未暂停、非诊断会话，尚无观察会话。
2. 几何候选为空，当前诊断中的 `materialized_count` 严格为整数 0，阶段为 `CONTACT_EXECUTION_PLAN` 且接触门没有延后。
3. 没有已有有效预览/接触计划、正在执行的动作或未决执行授权。
4. 请求和目标身份仍当前；冻结快照属于当前实测多视角表面；当前检测和几何有效；原 `measured_bilateral_view_gaps` 以真实侧面掩码、点数和完整 OBB/夹爪开口条件确认证据缺失。
5. 原初始接触阶段预算在缺失侧计算前后均有效；阶段已过期不能借转换重新获得预算。

转换只冻结缺失侧与来源证据、取消旧请求并开启观察前缀。它不是接触成功，也不是“穷尽接触搜索后失败”。审计明确记录：

```text
policy = current_request_measured_view_deficit_preflight
contact_search_completed = false
learned_candidates_requested = false
tabletop_geometry_preflight_completed = true
materialized_count = 0
contact_or_motion_authorized = false
```

触发证据携带请求、generation、target epoch/track、源时间戳、selection、真实缺失侧和来源视角列表，并记录内容摘要。新观察计划审计再绑定自身请求的五项来源标识；旧触发证据不能被当作新图像或运动授权。原来确实完成接触搜索的 fallback 继续保留真实完成证明，不能用 B 的非穷尽证明替换。

| 初始情形 | 本轮行为边界 |
| --- | --- |
| 直接策略或非 unknown 模式 | hook 不分流，沿原规划流程 |
| 已有有效接触计划/预览或执行授权 | 不抢占现成计划，不提前转观察 |
| 已生成至少一个几何接触候选 | 沿原接触流程 |
| 没有真实实测缺失侧证据，包括视角足够的情形 | 沿原接触流程 |
| 零几何候选且有确切实测缺失侧，其他条件全部成立 | 允许先观察，再用新证据规划接触 |
| 仅学习候选可能成功、但尚未请求学习推理 | B 可能先观察；这是明确的策略变化，不能宣称完全保持旧的一次性抓取行为 |
| 源过期、pending、身份变化、原预算到期或严格门失败 | 保持拒绝/取消；A+B 不放宽感知或执行条件 |

## 支撑可见性恢复：已复现的窄分支缺陷

`tabletop.py::segment_with_support_fallback` 先对当前帧进行独立支撑审计。旧冻结平面的原 2 mm 可见带可能已无足够点，而当前帧新拟合平面仍满足原质量、空间覆盖和相对不可变参考的一致性条件。此前 `locked_support_plane_no_longer_visible` 直接抛出，未进入已有同帧重拟合恢复流程。

本轮只在该异常且 `support_current_audit_available is True` 时允许进入原同帧支撑重拟合与重分割。当前审计缺失、不合格或存在正向矛盾时仍拒绝；参考平面和目标身份保持不可变。原拟合阈值、内点比例、残差、覆盖、参考一致性、目标关联及源新鲜度门均不改变。此处不把历史同文件中的拟合/环选择修改算成本轮新增修复。

合成复现修复前为 **2 failed、2 passed**；修复后相关四个测试文件为 **46 passed、1 skipped**。原结果显式标注 `synthetic_reproduction=true` 和 `hardware_failure_causality_claimed=false`。这证明该分支可以错误拒绝满足原条件的同帧恢复，尚未证明它就是 gen26/29/30/31 的实机首失原因。

证据：`SESSION/verification/support_visibility_recovery/{before.log,after.log,after.xml,result.json}`。修复时 `tabletop.py` SHA256 为 `63d11e107c7041484d73d96893bc9eeaa71acfcf50f9a84fb92bcd7a7be1eff6`。

## 不同实验的超时证据必须分开

| 实验 | 实测终止条件与时间 | 已证明的范围 |
| --- | --- | --- |
| gen26 | 首失时当前帧约 **33.234835 ms**，旧合格参考约 **851.190806 ms**；有 **3 次 pending** | 已核对封存首失报告，支持“当前输入新鲜但旧合格参考过期”；不能由此解释以后每次 stale |
| gen29 | `source_exceeds_existing_frame_limit`；首失时旧参考 **818.877458 ms**，当前帧 **214.202881 ms** | 12 次当前支撑 pending 均为低内点比例，10 次恢复；5 次 initial_contact 准备被取消。最终当前帧支撑已恢复，随后恢复/RGB 流程的第二次资格检查因旧参考超过原 **0.8 s** 而拒绝 |
| gen30 | `source_frame_stale_or_future`；入槽 **7.644 ms**，排队 **187.542 ms**，处理后到 post_segment 已 **903.716 ms** | 当前帧确实处理超龄；decode 墙钟 **557.185 ms**、当前线程 CPU **3.564 ms**，分割墙钟 **150.682 ms**，TF **0.164 ms**。定位到耗时区间，未定位具体等待机制 |
| gen31 | 首个终止事件为无源帧的 `exact_rgbd_stream_stale`；首失时 color 到达间隔 **942.917 ms**，完整配对到达间隔 **909.370 ms** | 首失是感知入口缺少完整 RGB-D 对；末个 ready 帧以 **741.970 ms** 源龄成功发布，decode 仅 **1.243 ms**。首失后出现的配对延迟和 decode 超时是后续事件，不能倒推成首失原因 |
| gen32 | `support_gap_not_qualified: support_plane_inlier_ratio_low`；当前帧 **183.231354 ms**，last-good 帧 **491.354466 ms** | 尚未进入旧参考龄检查。上一 ready 帧的真实关节反馈跨度 **223.400414 ms**，原 200 ms 函数回放拒绝；缺少恢复参考导致下一次支撑缺失直接失败。详见本轮追加分析，不能归类为 0.8 s 超时 |
| gen33，15 fps | `exact_rgbd_stream_stale`；color/depth 原始到达间隔约 **812.879/810.063 ms** | 149 帧 ready 后整对输入中断；跨首失的 SDK `wait_for_frames` 墙钟为 **1,298.909 ms**。SDK 等待边界有证据，设备/USB/主机根因仍未分清 |
| gen34，15 fps | 先发生 `UNKNOWN_OBSERVATION_NO_SAFE_VIEW / OBSERVATION_SEARCH_INCOMPLETE`，只比较 **6/227** 视角；约 **13.6 s** 后才发生 `exact_rgbd_stream_stale` | 本轮首先停止在观察计划资格检查，后续断流不能倒推为先前规划失败的原因；6 次 MoveIt 成功不等于完整路径或执行成功 |

gen31 末个 ready 帧的 RGB refinement 墙钟为 619.074 ms，当前线程 CPU 为 72.447 ms；首失后另一个帧的 rectified source validation 墙钟为 893.698 ms、线程 CPU 为 4.344 ms。墙钟明显高于线程 CPU 能证明该区间不是全由当前线程计算占用，不能单凭它区分调度、GIL、原生锁等待或其他线程工作，也不能据此指定相机设备/传输故障。

gen29 同一原始采样点回放保持原 2 mm、0.70、170/140/110 环和采样规则。最后两次 pending 的原拟合最大内点比例分别约 0.672 和 0.665；8192 次、多 seed 的独立加强搜索仍未达到 0.70。最终恢复帧则通过，r170 内点比例为 0.711667。没有证据支持“只需提高 RANSAC 次数就能使这些失败帧合格”；有限搜索也不是全局不存在合格平面的数学证明。

同像素深度存在毫米级帧间变化，记录姿态基本一致。可以确认实测分布变化，尚不能把各部分归因到桌面起伏、传感器噪声、场景变化或滤波历史。录制的 `depth/image_raw` 是实际发布的最终深度，可能已含 SDK/对齐/滤波处理；诊断 NPZ 中的 raw 指未改动的发布 uint16 数组，不代表 SDK 滤波前数据。相机参数快照标注 `image_time_association=none`，不能当作逐帧因果证据。

证据索引：

- gen26：记录根目录下 `ros_log/unknown_receive_buffer_20261008T073618Z/fresh_confirmed_grasp_20261008T084434Z/analysis/first_loss_timeline/first_loss_report.json`。
- gen29：`SESSION/gen29_failure_analysis.json`、`SESSION/gen29_support_diagnosis/diagnosis.json` 及同目录的原环输入、`fit_counterfactual_*.json`。
- gen30：`SESSION/gen30_failure_analysis.json`。
- gen31：`SESSION/gen31_failure_analysis.json`。三个 generation 的报告均附对应封存实验路径及完整性信息。
- gen32：`SESSION/gen32_failure_analysis.json`，含七个 manifest 文件的实际/预期哈希、逐 ready 帧反馈检查、图像元信息和执行记录核对。
- gen33：`SESSION/gen33_failure_analysis.json`，含完整负载、SDK 等待与感知入口时序证据。
- gen34：`SESSION/gen34_failure_analysis.json`，含同 generation 审计、六次只规划日志、先规划失败后感知首失的时间顺序和十个 manifest 文件核验；排除一份录制时带入的 gen33 旧审计。

## 对分析与建议的证据结论

| 判断或建议 | 当前结论 |
| --- | --- |
| “新图像很新，但合格参考已旧，因此在补观察前取消” | 对 gen26、gen29 已由时间线证实；不能推广到 gen30/31 |
| “补观察不应该以当前视角下可执行接触候选为先决条件” | 代码依赖已核对；A 已按独立观察契约实现，真实执行仍必须完整路径验证 |
| “真实缺失侧时应减少无收益的初始接触等待” | B 已按上表窄条件实现；减少实际延迟的幅度和抓取成功率仍需实机测量 |
| “旧参考面不可见但当前面合格时，应走同帧恢复” | 已独立合成复现并修复；不声称已经证实历史实机因果 |
| “末两次低内点比例只是随机 RANSAC 次数不足” | 相同点的加强搜索未支持该解释；没有增加生产拟合预算或降低质量门 |
| “把一次 get_param 改 cached 就能解决 557 ms” | 不支持。预热后各 20 次只读探针的中位数约 0.393/0.317 ms，原缓存路径也已存在；未作该修改 |
| “30→15→30 fps 能可靠降低总负载并消除失败” | 已完成两轮 15 fps 完整负载实验，支撑与反馈连续性比 gen32 所记录区间更好，但两轮均失败，未建立帧率变化的独立因果证明。当前仍为 15 fps，不能声称已完成恢复到 30 fps 的闭环对照或问题已消失 |
| “本次失败说明新 FK 仍不可达或一定是五自由度导致” | gen28 冻结数据的新候选通过原完整路径离线检查；gen29–31 均未运行到该新规划链，不能据这些实验作此归因 |
| “一次性抓取已不受任何影响” | 不作绝对保证。保留路径有代码和离线测试边界，B 确有上述策略变化；修改后全部一次性场景的实物回归尚未完成 |

## 版本与验证交接

四个 producer 文件冻结清单：`SESSION/verification/independent_observation/source_manifest.json`，该清单初版 SHA256 为 `81fc7d0c475cd4b0d46d91fe1b99ac42114c4713cf65922398367034015e90b5`。它记录源文件完整哈希、接口、保留条件和消费端关联文件；消费端及本轮 `tabletop.py` 应以最终组合部署清单另行绑定，不能仅凭这四个文件宣称全链部署一致。

已有离线证据：

- 原 silhouette/observe-return/身份生命周期三文件回归 **56 passed**，`SESSION/verification/independent_observation/first.log`。该轮早于最终阶段预算和显式 false 修订，不替代最终组合回归。
- 新普通独立观察、契约和阶段预算最终专项 **110 passed**，用时 3.08 s，`SESSION/verification/independent_observation_tests/{final.log,final.xml,result.json}`。覆盖初始过期、缺失侧计算中耗尽预算、来源/角色拒绝、真实缺失侧与观察增益以及单姿态到完整接触的边界；ROS 严格服务传输等仍有测试隔离，专项通过不代表实物执行。中间 fixture 失败记录保留，不能删除或冒充生产失败。
- 支撑可见性恢复 **46 passed、1 skipped**，见上述独立复现记录。
- gen28 真实冻结几何的新 FK 原全路径检查：`SESSION/robust_margin0_full_check/result.json`。它是特定冻结输入的离线路径证据，不是新实时源授权或实物抓取结果。

后续结果应明确记录最终源码哈希、测试命令与数量、部署对象及重载校验、实际 generation/实验目录、是否进入独立 observation_prefix、是否完成观察动作/回程、新证据下的完整接触规划和最终实物夹持结果。失败时保留真实停止原因，不把已有 HOLDING 状态作为本次动作或成功证据。

### 最终组合回归

最终组合结果为 **2,854 passed、1 skipped、8 warnings**，用时 **60.92 s**。源码绑定清单为 `SESSION/verification/independent_observation_combined_final/validated_sources.json`，组合日志及 XML 位于同目录。此前 `independent_observation_combined` 首次运行发生跨包测试收集路径错误，其记录保留；该收集错误不是生产功能失败，也不计入上述最终通过数量。

### 部署结果

最终回执 `SESSION/independent_observation_deployment_completion_retry2/result.json` 的 `deployment_passed=true`，记录 remote、task、unknown perception 和 gateway 的实际进程身份，以及 task/remote reset ready 应答。最终补全部署未再次重启 gateway，回执记录未发送运动命令、未查询温度、未发送关节失能命令，驱动和相机等保留进程另有核对。

此前几次部署工具检查因节点所有权、已注销观察者或旧节点注册等条件中断，原失败和部分完成回执保留在 `independent_observation_deployment*` 目录；这些失败不能改写成一次部署即成功。最终成功回执与组合源码清单共同作为本次交接依据，部署通过本身不证明抓取成功。

### gen32 实机失败轮追加

实验目录：记录根目录下 `grasp_records/20261008T122515.003686Z_ffbc823b23544d479e051ca3923e2df2`。录制已封口，manifest 为 `complete + failure`、`recorder_closed=true`，七个列出文件的大小及 SHA256 均核对一致。分析报告为 `SESSION/gen32_failure_analysis.json`，报告初版 SHA256 为 `f39e3a0f878dd4f520ce0034e4337c76de18ecd234402e5c2199cc4fd4ef6efb`。

首失前先经历 14 次未锁定目标时的支撑低内点比例、初始三帧确认重试，以及一次未锁定时的当前帧超龄。最终源 `1791462324133942842` 获得 acquired，随后五帧 tracked，总计 **6 帧 ready**。下一源 `1791462325668514728` 在 tracking 阶段首失，没有经过 `pending_current_support`；不能把此前未锁定阶段的超龄当作这次已锁定目标首失原因。

首失的直接入口为 `_hold_missing_current_support`。低内点比例理由明确在允许列表内，五帧 tracked 均有实际 RGB refinement 计时和度量，因此该入口的排他条件指向 `_locked_rgb_hold_reference is None`，不是关闭了 RGB refinement 或理由不受支持。

对封存 `/alicia_d/accepted_joint_states` 使用未修改的 `bracket_feedback` 逐帧检查：前五个 ready 源的前后跨度分别约 99.907、100.029、110.010、100.009、99.807 ms，均合格；最后 ready 源 `1791462325360391616` 的前后反馈为 `1791462325309931344` 和 `1791462325533331758`，跨度 **223.400414 ms**，原函数抛出 `accepted_feedback_does_not_bracket_source`。参考准备是 best effort，会捕获该异常返回 None；正常几何仍可发布 ready，随后更新 last-good 并将恢复缓存赋为 None。

上述代码路径与原函数回放构成足以解释缺失缓存的证据，**不是当时内部异常的直接日志**。封存记录没有序列化该消费者的私有回调缓冲区或被捕获异常，不能宣称精确读到了该内部状态。

首失时当前源龄为 **183.231354 ms**、last-good 源龄为 **491.354466 ms**，都小于原 0.8 s。简单保留前一个反馈合格参考也无法使该事件通过：该源 `1791462324928574800` 此时已 **923.171282 ms**。增加等待不能缩短已经发生的真实反馈样本空洞；本轮没有放宽 200 ms、0.8 s 或支撑质量条件。

唯一规划请求使用 snapshot `1791462324533272266`，总耗时 **1,130.858421 ms**，在基础 `_prepare_and_predict` 的几何候选生成后、B hook 前的请求有效性检查被取消。该值是**请求总耗时**；错误栈位置不能证明全部时间花在 CAD、几何候选生成或任何单独步骤。各 WSL 计时为 0，无 B 转换、`measured_observation`、新 FK 或严格规划记录。

封存计划全部无效，审计目录为空，抓取状态为 IDLE；动作状态只有本次 selection 之前的历史 succeeded goal。选择之后记录到的 **671 条**真实六轴关节向量完全一致，未观察到本次新轨迹或机械臂运动；锚定前 0.5 s 至首失的 22 条反馈亦一致。这不证明物体或照明静止，但不支持把故障归因于用户未保持机械臂静止。相应七组图像元信息均为 653×489、`camera_link`、同源 stamp/seq 的 bgr8 + 16UC1。

当前尚无保持全部原门即可由“保留旧缓存”一行修复挽救该封存序列的证据。gen32 分析完成时拟进行 15 fps 完整负载对照，后续实际结果追加如下；不能将部署或离线测试通过替代最终实物结果。

### 15 fps 完整负载第一轮：gen33

实验目录：`grasp_records/20261008T123552.595043Z_704863368cdf4c2abd14255ee3daab9e`。这是正常感知、规划和录制并存的完整负载尝试，不是只有相机的空载测试。报告为 `SESSION/gen33_failure_analysis.json`，实验已封存为失败，未发布有效计划，未观察到活动执行。

初始接触请求 2 完成原候选搜索，原审计 checked 44、reachable 0；随后发布 `OBSERVE_RETURN_VIEW_REQUIRED BILATERAL_SURFACE_EVIDENCE_MISSING`，进入补观察阶段。补观察请求的 WSL 总耗时为 0。最终目标因 `exact_rgbd_stream_stale` 失效，相关请求作废，未得到有效观察计划。

首失前 **149 帧 ready** 覆盖 **29.804 s**；148 帧 tracked 的独立当前支撑审计均可用，内点比例最低 0.72278、中位数 0.86417，无支撑 pending。所有 ready 源在封存反馈中的夹持跨度均不超过 110.621 ms，六轴位置一致，没有重现 gen32 的 223.4 ms 间隙。处理墙钟中位数为 148.129 ms、最大 210.362 ms；末个 ready 帧源龄 182.149 ms 即发布，并非处理超龄。

首失 watchdog 没有源帧，color/depth 最后原始到达已分别间隔 812.879/810.063 ms。独立录制图像的源序号 1657→1658 连续，但源时间间隔 **1,328.951 ms**。相机后台证据进一步将跨越首失的长间隔定位到 `wait_for_frames`：墙钟 **1,298.909 ms**，该线程 CPU **0.087131 ms**；两路 SDK 帧号各只增加 1，SDK 时间间隔约 1,355.839 ms，等待之后的区域滤波只耗时 13.668 ms。

因此可以说停顿发生在 SDK 等待/返回边界，早于该次逐帧区域滤波和发布；不能据此认定相机硬件、USB、驱动、主机调度或 GIL 中某一项是根因。周期参数元数据并非完整逐帧历史，且不同 generation 的时间窗口没有隔离帧率因果。15 fps 在该区间改善了已观察到的支撑与反馈连续性，但未消除整对输入停顿。

### 15 fps 完整负载第二轮：gen34

实验目录：`grasp_records/20261008T124106.426913Z_8690434647cf4b4e838df3f34e6b250c`，已封存为失败；十个 manifest 文件均核验通过。分析报告为 `SESSION/gen34_failure_analysis.json`，初版 SHA256 为 `462e6be54ee6f7b45aef3e193ef60021ab0bd5df0c94c2105441f8453811fd51`。录制中的 `eaea0244...` 审计属于 gen33，不计入 gen34 的规划次数或来源证据。

本轮首次停止位置已与 gen32 不同：

1. 初始接触请求 16 实际生成 **32 个几何候选**，因此按 B 的零候选边界保留原接触流程；不是 B 没有正确触发。完成检查的 38 个候选均未被接受，随后通过原有真实缺失侧 fallback 转入观察前缀。
2. 请求 17 使用新源 `1791463283670048475`，审计明确为普通 `measured_observation`，`diagnostic_observation_only=false`、`contact_candidates_requested=false`、`contact_geometry_checked=false`、`remote_grasp_inference_required=false`，未调用抓取跟踪器，WSL 总耗时为 0。该观察请求已解除未来接触候选依赖。
3. 网关记录 **6 次 `execute=False` 的 `check_pose_strict result success=True`**。但最终审计 `selected=null`，停止于 `UNKNOWN_OBSERVATION_NO_SAFE_VIEW`，具体汇总为 `OBSERVATION_SEARCH_INCOMPLETE: bounded comparison checked 6/227 views; unchecked views are not proven unreachable`。这说明有限预算内没有视角被完整接受，不证明 227 个视角全部不可达。

上述六次 success 日志来自 `handle_pose_strict` 内部的 MoveIt 规划返回。外层 `_with_observation_planner` 随后仍要构造 `observation_path_evidence`；若失败，可将响应改为 `OBSERVATION_PREVIEW_PATH_UNAVAILABLE`。远端还要执行后续路径、跟随和来源资格检查。因此这六条日志既不能证明最终网关响应成功，也不能证明原整条路径门已通过，更不能证明执行成功。

旧失败审计只保留汇总，观察请求的 `rows=[]`，没有逐视角 `orientation_search`、端点拒绝和调度详情。现有封存数据尚不能分清到底是哪一视角的**网关凭证生成**或**后续资格检查**导致拒绝；不能选择其中一个作为已经证实的根因。

该观察请求约在 `1791463291.322069` 完成并停止；感知首次 `exact_rgbd_stream_stale` 为 `1791463304.930408477`，晚约 **13.6 s**。此前共有 **172 帧 ready**，无支撑 pending。后来的断流不能倒推为先前的观察规划拒绝原因，也不能未经独立证据就认定和 gen33 是同一次 SDK 机制。

本轮没有有效计划或新动作 goal；选择之后 **1,036 条**已接收六轴关节向量一致，无实测机械臂运动。观察、回程、重新规划接触与最终夹持均未执行成功。当前帧率保持 **15 fps**；本文记录不包含恢复到 30 fps 的动作或成功结论。

### 待部署的失败审计补丁

针对 gen34 丢失逐视角拒绝原因的问题，`silhouette_observation_planning.py::_build_diagnostic_observation_gate_audit` 在未选出候选时增加 `observation_failure_diagnostics`，保留同一次规划的 orientation search、端点拒绝、调度，以及绑定相同 prepared/source 的 FK 缓存报告和来源标识。它不增加规划服务调用、不重复采集图像，不改变候选顺序、阈值、预算、原路径门或执行权限。

专项 **145 passed、0 failed、0 skipped**；证据为 `SESSION/verification/failed_observation_audit/validated_sources.json`，生产文件 SHA256 为 `8c4275705dae72208fe8687730c0046bb3ea04704e7171c285249b714bf11c42`。本文更新时补丁待部署；它只能使下一次失败可定位，不会补写 gen34 缺失的事实，也不能声称已经修复观察路径拒绝。
