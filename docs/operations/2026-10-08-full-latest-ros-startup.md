# 2026-10-08 最新完整 ROS 启动记录

本轮按用户要求直接启动最新 ROS 节点并请求关节正向使能，跳过此前完整入口中的前置交接流程。WSL 由用户启动。启动阶段使能回执为 `PENDING:POSITIVE_ENABLE_REQUESTED`，后续新对准回执已确认 `CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE`。**最新状态：101 源组合已部署，generation 31 已使用用户真实 131549Z 新对准确认启动，尚无本轮实物成功证据。此次实质改变为首个端点合格基线后优先尝试同方向 2× 暴露；另两项只增加分割子阶段计时和接触高度重试分支审计，未修复或证明解决 gen30 的偶发处理尖峰、预测命令碰撞及完整接触可行性。gen30 已封存失败，未执行接触或闭爪；所有原碰撞、身份、当前源 0.8 秒时效和路径门保持。** 不能把使能确认、候选通过、离线回放或观察运动完成写成抓取成功。

外盘会话：`/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_ros_20261009T003407Z`。目录名使用 UTC；启动时美国洛杉矶当地日期仍为 **2026-10-08**。本记录依据 `startup.json`、`enable_receipt.json`、启动日志及执行代理的补起回报编写，文档编写过程没有调用 ROS 或修改生产代码。

## 本轮入口与运行范围

工作树为 `/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade`。实际根服务为 `alicia-full-latest-20261009t003407z.service`，入口为该工作树的 `.runtime/start_full_latest_ros_20261008.sh`；脚本加载当前 `devel/setup.bash` 后直接执行：

```bash
roslaunch --skip-log-check /home/zhuyupei/alicia_wa_full/.ros_log/ros_live_20260922_session/resume_modes.launch
```

准确 `systemd-run` 参数和外盘输出路径保存在 `startup.json`；此次没有经旧 `start_ros.sh` 的交接和仿真等待流程。根 roslaunch PID 为 **8606**。启动范围包括驱动、硬件接口、控制器、MoveIt、网关、相机、手眼、感知、规划、任务、模式路由、夹爪控制、日志和 GUI。主要启动 PID：驱动 **8718**、相机 **8738**、GUI **8779**。这些是本轮日志记录的进程身份，不是以后重启可复用的标识。

默认模式为 `unknown + observe_return`，启动回执的 GUI 状态为 `manual_control=true`。这是启动阶段的时间点证据，不代表自动交接后的实时 GUI 状态。节点就绪本身不表示目标已对准或已有可执行计划；后续用户确认与 runner 启动另列如下。

## 补起项与设备连接

- 旧固定仿真服务通过 `BindsTo` 绑定旧 root，不适用于本轮新 root。已独立启动 `alicia-latest-simulation-20261009t003407z.service`，运行原 `candidate_simulation_startup.py --serve`；执行代理回报 active，`startup.json` 保存启动成功返回。没有用旧仿真记录代替本次启动。
- 只读监督 `alicia-latest-watch-20261009t003407z.service` 已启动，日志为 `watch.log`。已打开 GNOME 实时终端跟踪本轮日志。
- 原夹爪控制进程 **8769** 因启动时等待网关超过 10 秒而退出；网关就绪后以 `alicia-startup-gripper-20261009t003407z.service` 补起，当前补起进程 PID **11203**。`gripper_start.log` 保留补起证据，原失败日志不删除。
- 用户接入 USB 后，真实相机自动连接，当前启动配置为 **30 fps**；串口 `/dev/alicia_arm` 指向 `ttyACM0`。前期“No device connected”属于接入前的重试记录，不应当作接入后的最终状态。此次 30 fps 来自新完整启动配置，不能把它写成此前 15 fps 完整负载问题已经解决或完成了抓取对照。

## 使能与抓取边界

`enable_receipt.json` 记录已通过 `/demonstration` 发布 `false` 请求正向使能，`positive_enable_requested=true`，未发送失能命令；当时状态为 **`PENDING:POSITIVE_ENABLE_REQUESTED`**。该历史回执保留原样，不用后来的确认覆盖它；后续 `alignment_receipt.json` 已记录真实运动反馈确认。

启动时的 `awaiting_user_alignment=true`、`automatic_grasp_started=false` 同样保留为历史证据。后续自动抓取须使用本轮新图像、目标选择、generation、关节反馈和路径资格，不能沿用旧 q、gen34 的目标/审计或旧轨迹。

软件温度监控和温度轮询继续关闭，不查询温度或据此采取软件反应，不发送关节失能命令。实验和日志继续 `local_only` 存入 SanStand，不上传、自动删除或静默回退系统盘。

## 证据与待更新项

会话内 `startup.json` 保存启动命令与仿真/监督返回；`enable_receipt.json` 保存使能请求、手动状态和当时节点列表；`ros.log`、`ros/`、`simulation.log`、`watch.log`、`gripper_start.log` 保存运行记录。它们仍可继续追加，不能把正在运行的日志称为封存实验。

后续只追加本轮真实结果。本记录不声称补充视角或一次性抓取实物回归已经成功。

## 用户对准确认与自动抓取交接

已读取会话 `alignment_receipt.json` 和本轮实验的 `authorization.json`，两者绑定用户消息 **`call_4e5931c5785e4c94b1e5fd6c2eb58618:0`**：“已对准，可以继续自动抓取”。回执时间为 `1791506638384960174 ns`，图像源为 `1791506638260601282 ns`，真实 SDK 关节反馈源为 `1791506638342770677 ns`，控制 epoch 为 `1791506524580754980 ns`；`actuation_status=CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE`，`previous_alignment_reused=false`。

本轮实验目录：`/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20261009T004422.530234Z_2ca7b5431ccf4c3f93c20132672c5172`。授权记录 `operator_authorized_autonomous_grasp=true`、`user_alignment_claimed=true`、`old_plan_reused=false`，新选择为 `unknown + observe_return`、**generation 2**、源时间 `1791506664577421426 ns`。此前 generation 1 仅为交接前选择，不继承其计划。

`.runtime/run_current_aligned_grasp_20261008.py` 已在完整实验录制下运行，`runner.log` 记录 `CANDIDATE_COMPUTATION_START success=True`。首次交接记录写入时 manifest 仍为 `recording`；这只是当时状态，最终封存结果见下节。runner 开始计算不等于已发生观察运动、接触、夹持或抓取成功。

## 首次自动抓取封存结果与当前卡点

上述实验 manifest 已为 `complete`、`recorder_closed=true`、`result=failure`，结束时间 **2026-10-09 00:45:32 UTC**。具体终止原因为：

```text
6D pregrasp failed: strict cached execute blocked:
OBSERVATION_PATH_INVALID: OBSERVATION_VIEW_CONFIGURATION_CHANGED
```

本次与旧 gen34 的失败阶段不同，已经产生可提交的独立补视角计划：

- 初始 request **1** 的 62 条稳定评估中，57 条带 MoveIt 结果，全部是同一接触必要碰撞预检拒绝：`CONTACT_SEQUENCE_PRECHECK_FAILED: nominal contact goal has unavoidable target OBB collision within unchanged pose/opening tolerances: left_finger,right_finger`。不能把错误类别 `MOVEIT_CHECK_ERROR` 直接解释成服务离线或控制器未启动。
- 后续 request **22** 进入普通 `measured_observation` 独立观察分支，`contact_geometry_checked=false`；其 5 个已检查候选具有 `following_support.ok=true`，最终选择计划 **`69da48b037e320f0ca123959`**，审计为 `PLAN_READY`、`STRICT_SERVICE_SUCCESS`，附真实规划路径凭证。源绑定为 stream generation **3**、request **22**、snapshot **`1791506704393857240 ns`**、target epoch **14**、track **`g1-t14`**；不要把 stream generation 与用户选择 generation 2 混用。
- 任务完成严格预检并进入执行入口；日志明确 `alicia_controller, hand_controller` 已运行，保持同步通过，`max_error=0.007670 rad <= 0.035000 rad`。最终在实际观察轨迹提交前的配置重检失败。候选的完整路径资格成功不表示所有执行时重检、到达新视角后的测量、后续接触或夹持已经成功。

因此，**当前已知直接阻断是执行前配置一致性检查**；本轮证据不支持继续把 FK 无解、候选排程不足或控制器未加载作为当前必修阻断。

封存 bag 含 **69** 次 `/camera` 参数快照。所有快照仅 `runtime_profile.stationary_temporal` 子树变化，其他参数精确一致，包括内参、图像尺寸、rectification、geometry activation 和其他相机配置。执行窗口相邻三份快照的 `regions[0].visual.frame_number` 从 **13802 → 13831 → 13861**；旧实现将整个 `/camera`（包括该持续更新的状态）存入 `camera_configuration_snapshot` 并直接比较，单独这类动态诊断变化就足以复现拒绝。

证据边界：网关当时用于比较的两个完整 configuration 字典没有写入三份 gate audit，`/experiment/runtime_execution_error_snapshot` 为空。因此这是由实际采样参数与原比较代码形成的充分复现证据，不能声称已直接记录到失败瞬间的唯一字段差异。修复仍需保留真实几何、源激活、安装标定和限制变化的拒绝规则。

本轮没有提交补视角多点运动轨迹，所有已录 `FollowJointTrajectory` 状态列表为空；但不是完全没有控制命令或物理变化。执行入口发出 **1 个 0.02 s 的控制器单点 hold** 并转发为一次 `/joint_commands`。681 帧真实关节反馈中 Joint4 的变化范围为 **0.00766990394 rad**，其余 5 个机械臂关节不变；614 帧 control reference 全部一致。不能把这次记录写成“完全未移动”，也不能把这段 hold 写成补视角到达。

离线分析位于本会话 `aligned_grasp_20261009T004422Z_analysis/`：

- `analysis.json`，SHA256 `d102cf789ebfadf673de3ebed8c4d0715a57fb52c5d19a35db5e412025b5dbba`：57 个候选的逐项错误、选中计划/源/路径凭证、相关日志、相机差异、hold 与关节反馈统计。
- `camera_parameter_execution_window.json`，SHA256 `1993f01430a921c4b3705210e6eaaebf97a7be9b3591539165e82ed86a2a7c81`：执行窗口 3 份原始参数快照及读取时间，供配置比较修复离线复现。

本次文档更新时没有再启动新抓取，配置比较补丁正等待独立验证和冻结；不宣称已部署、已复试或已完成实物抓取。原实验与历史授权回执保持不变。

## 后续配置修复与第二轮补视角实机结果

上一节末尾是首次封存时的历史状态。随后 `observation_view_gateway.py` 区分固定相机配置和逐帧诊断元数据，保留投影、标定、安装与运动限制变化的拒绝规则。离线验证 **65 passed**；部署源 SHA256 为 `a48364c41c8965d5896622930351f0910163787bc33511c68cfe490d893ad667`。仅通过原 roslaunch 的 respawn 重载空闲网关，PID 从 8733 变为 21166，其余驱动、相机和任务节点未重启。部署回执为本会话 `observation_view_gateway_deployment/result.json`。

第二轮封存实验为 `grasp_records/20261009T005610.945830Z_db47451a69e647de884e9af9aaeb38a2`，用户选择 generation **3**，补视角计划 `1fa1132f4b16bdbd9a539284`。轨迹 action `/motion_gateway-2-1791507410.504` 于 `1791507410.505713` 提交，运行 **31.917323144 秒**；`1791507442.457812` 返回状态 3、error 0。机械臂确实完成了补视角移动，不能将本轮归为执行前失败。

任务随后报告 `MEASURED_OBSERVATION_GEOMETRY_INVALID: TARGET_LOST: perception_failure: support_plane_local_distance_inconsistent: 0.004410 > 0.004000 m`。首次失效源为 `1791507424405864715 ns`，发生在运动中，图像年龄 **59.380 ms**。其后 110 帧保持 `target_lost_requires_new_generation`；到位任务消费的是已锁存的失效，而非终点重新拟合结果。

为区分瞬时测量异常与持续终点偏差，进行了 **12.055 秒**被动采样，记录 37 对 exact RGB-D、TF、121 条 SDK 实测位置和 6 份相机配置。未订阅温度、设备诊断或关节 effort。采样期间 6 个关节位置范围均为零。`arrival_static_capture/support_plane_analysis.json` 离线复核结果：

- 运动中首失帧重放偏差 **4.409540 mm**，精确复现原 4 mm 拒绝。
- 静止到位后的 10 帧全部通过原支撑拟合及平面一致性检查；最大局部偏差 **0.269–1.212 mm**，中位 **0.562 mm**；内点比例 **96.61%–98.67%**，均覆盖四个象限。
- 相机投影、缩放、rectification 和激活配置与原实验一致。未放宽 2 mm、70%、4 mm 等原门限，也未恢复 generation 3 的执行权限。

上述结果支持修复“绑定观察运动期间保留身份、到位后重新测量”的流程；尚不能单独确定运动偏差由深度、拟合、TF 或标定中的哪项造成。支撑复核通过不等于完整身份关联、接触计划和实物夹持已通过。完整报告 SHA256 为 `40a0f991238a11f0f2e20e642a884d9de9b62bdf4f3fca6b5f725ce05de1e6b4`。

用户后续消息“你继续处理，我先移动机械臂”撤回当前自动运动窗口。`manual_reposition_receipt.json` 记录 GUI 手动直控已经为 true，任务空闲，没有发送运动目标、力矩或失能命令。上面的静止图像与姿态只用于离线分析；用户新位置不能继承旧对准回执。下一轮须在新的“已对准”消息之后记录新实测来源。

## 再次对准、到位重测修复与 generation 4 结果

用户随后通过消息 **`call_05769a4033f5419891863af69872dfce:0`** 再次明确“已对准，可以继续自动抓取”。新回执为本会话 `realigned_20261009T012205Z/alignment_receipt.json`，记录时间 `1791508926077514410 ns`、图像源 `1791508925990562915 ns`、SDK 实测关节源 `1791508926044385769 ns`，`previous_alignment_reused=false`。控制 epoch 为 `1791506524580754980 ns`，使能状态为 `CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE`。该回执取代上节的等待对准状态，不覆盖历史回执。

到位重测修复使用绑定 session、计划、generation、目标 epoch、控制 epoch 和固定截止时间的观察生命周期。在原严格执行前验证之后，任务等待 unknown 与 remote 双端确认；非接触观察运动期间保留冻结目标身份及原支撑参考，暂停发布具有测量权限的目标/掩膜。原执行和静止确认成功后，必须获得三帧新的真实测量并通过原支撑、唯一目标关联、RGB 与新鲜度门，双方确认和任务收到同源实际对象后才继续原流程。它不恢复已经锁存丢失的目标、不以旧几何授权接触，也没有放宽原 4 mm 支撑一致性门。

最终综合验证为 **3037 passed、1 skipped**，验证前后源 SHA 一致，清单位于 `verification/observation_measurement_final/validated_sources.json`。部署成功回执为 `observation_measurement_deployment_retry1/result.json`：仅重载 unknown、remote、task，分别为 PID **35671、35672、35673**；驱动、相机、网关及其他保留进程未重启。此次部署本身未发送运动或力矩命令，原失败部署记录保留。

为记录暂停目标/掩膜发布期间的真实图像，recorder 在合法 `executing` / `requalifying` 窗口约每秒保存 5 对 exact RGB-D，另写 bag 内 `/experiment/observation_rgbd/color`、`depth`、`metadata`。图像保持原消息、原 header 和原源时间，不补造目标或掩膜，不成为执行数据源；原四项完整关键帧判定与计数保持。窗口结束或固定 ROS/单调时钟截止即停止，旧请求不能重开。离线回归 **64 passed**，清单为 `verification/observation_rgbd_recorder/validated_sources.json`，recorder 源 SHA256 为 `f887e9e4dc1c8b71403db42f5e612c4759a3ba8c6f04f12e37ed18a10be888ca`。

新位置实验为 `grasp_records/20261009T013718.389935Z_3f02b61db15c47708671ab455fa45907`，新选择 generation **4**、stamp `1791509839874920129 ns`。录制于 **2026-10-09 01:38:14 UTC** 封存，manifest 为 `complete`、`recorder_closed=true`、`result=failure`。runner 未记录 `GRASP_START`，终止于：

```text
UNKNOWN_OBSERVATION_NO_SAFE_VIEW: OBSERVATION_SEARCH_INCOMPLETE:
bounded comparison checked 4/166 views; unchecked views are not proven unreachable
```

request **2** 的审计 `d2d8a90bcfe5d49bb31c767a47fb5b78798f9fc39b51654deacebda07306826b.json` 给出明确数据边界缺陷：`joint_space_fk_search.audit.status=invalid_input`，原因 `cannot pickle 'mappingproxy' object`，`directions=[]`、`candidate_count=0`，耗时 **0.000560 s**。源证据经递归冻结后，调用处浅拷贝仍携带冻结 binding，而 helper 对该 binding 使用 `deepcopy`，在进入方向优化前就失败。旧测试从 JSON 读取普通字典，且 adapter 测试替换了 helper，未覆盖真实冻结容器穿过两层接口的场景。

本轮实际检查的四个候选均属旧 `camera_orientation_lattice`：前三个路径视野最小图像边距为 **35.214、35.150、30.888 px**，均未达到原 **36 px**；第四个在最紧跟随误差盒下出现支撑净距 **−10.715 mm** 的反例，未达到原 **3 mm**。另 162 个候选未检查，不能据此断言全空间无解。冻结容器复制修复正在单独验证，保留源绑定、缓存身份、共享预算和全部原门；本节不声称它已部署或实机复试成功。本轮尚未运动，因此也不能把尚未进入的 request/ACK/到位重测链写成实机通过或失败。

## 冻结映射与候选排程修复，以及 generation 5/6

随后冻结映射窄修完成 **41 passed**：`reachable_observation_views.py` 仅用递归容器复制替代 binding/gaps 的直接 deepcopy，原冻结输入不变。新增测试实际经过生产冻结函数、adapter 和真实生成器，验证同值绑定、缓存隔离及原端点门。源 SHA 为 `4386e41837379006711ae1c754d4f5e002c0c9b718b30c030996eed6a55b8dd4`；`fk_mapping_deployment/result.json` 记录部署成功，硬件节点身份保持。

新选择 generation **5** 的实验 `grasp_records/20261009T014405.965804Z_6ffa3ee69c35414abb29d86c19a84a1b` 已封存为失败，仍未进入运动。真实源成功产生 **2 个** FK 候选，耗时 **0.920025 s**，原五项端点门通过，证明冻结映射缺陷已在本轮消除。但 212 个候选中，FK 排在零基索引 **140、141**，未取得严格路径检查机会。前四个旧视角候选均因 `OBSERVATION_POSE_PLAN_FAILED` 被拒，原三秒检查预算耗尽，其余 208 项未检查。审计 SHA 为 `25113e950719ebc45f10837670177f392545e7b4f75a169a5c0dc59278c52b5b`。端点候选生成不证明完整路径合格。

为修正这一实测排序缺口，仅普通未知目标缺面观察分支在同等实测支撑资格内交错 FK 与旧视角候选；保留各族原序、全部候选、原三秒/六次检查预算、首个完整合格路径后的八秒比较窗口，以及全部路径/源/执行门。真实 212 候选回放将前四项调整为原索引 **140、0、141、1**。验证 **188 项既有回归 + 17 项专项**通过，证据在 `verification/observation_family_schedule/`。生产脚本 SHA 为 `d0dae17e4d16ef530af51c224022d5aa263a8fb9e841cb3632c9359bbab5d2db`。

`family_schedule_deployment/result.json` 记录加载成功，随后启动新选择 generation **6**，stamp `1791510692472267150 ns`，实验目录 `grasp_records/20261009T015130.939636Z_053c5f63c0a5405da2a53aa19e654e4c`。本节写入时仍在规划；不把排序回放或自动实验启动写成运动、到位重测或夹持成功。用户最新对准授权持续有效，记录继续 local_only，不查询温度、不发送失能命令。


Generation 6 后续封存为失败：计划 `eab2b32c8d1d5ed6e84882dc` 的 FK 候选通过完整路径检查，执行前双端 ACK 成功，机械臂完成观察运动；9 条独立实测样本证明到位静止。随后 requalifying 请求也得到双端 accepted ACK，但三个秒窗口内 13 条完成的本帧审核均因 `rgb_instance_reaches_search_boundary` 失败，另第 14 帧完成时已超过截止时间，没有取得三帧 qualified ACK。任务于 `1791510746.800631` 报 `OBSERVATION_MEASUREMENT_ACK_TIMEOUT`，目标首次失效发生在撤销之后。当时问题是到位 RGB 分割边界；不是初始握手丢失、运动期目标失效或缺少当前图像。录制额外保存了 113 对运动/到位原始同步 RGB-D，供封存回放。该轮结束时机械臂位于自动观察终点；后续用户再次手动调整，不能用这一终点或更早的对准 q 直接重试。

## 到位 RGB 搜索及固定站位延续修复

封存回放定位到的是算法裁剪边界，不是真实图像边界，也不是沿用旧视角 ROI。代表源 `1791510744309704780 ns` 的当前深度种子只覆盖盒子上半部，bbox 为 `[249,333,124,57]`；16 px 搜索窗底端为 406，32 px 搜索窗底端为 422，完整 RGB 盒子轮廓到约 419，与第二次搜索窗的强制背景边界冲突。原图为 653×489，盒子距真实图像底部仍约 70 px。右侧 NFL 白色标志没有被第三次搜索并入目标。证据图、原始帧及逐次掩膜位于 `gen6_arrival_rgb_boundary/`。

修复仅允许绑定观察到位的测量在原 16→32 px 两次搜索均因 `rgb_instance_reaches_search_boundary` 失败后，再尝试同一当前帧的 48 px 搜索窗。三次尝试均保留首次可信核心；其他异常不触发额外重试，第三次仍触边、有效 RGB、保留比例、面积、身份及全部原支撑/质量门失败时继续拒绝。普通初始获取、一次性抓取和无到位资格的跟踪仍使用原 16→32 px 默认路径。

为使到位三帧确认后能够继续真实新帧融合，确认完成的同一站位可携带这项有限搜索资格。资格固定绑定原到位关节位置、selection、控制 epoch、图像源与相机配置、原 anchor/支撑平面及 session 截止时间；固定关节位置不会随逐帧参考更新而滚动。正常帧在目标选择前验证资格，发布前在输入锁内再次验证，只能由新测量通过原审核后继承。手动接管、离站或离开后返回、失效、新选择、epoch/配置变化或固定截止时间届满均使该资格失效。它只决定是否多一次 RGB 搜索，不刷新旧几何时间，不新增运动权限，也不恢复已丢失 generation 的执行资格。原 0.8 秒新鲜度、3 秒到位采样截止和三帧确认要求均未改变。

实录验证范围及限制如下：

- 到位窗口内保存了 13 对可完整回放的 exact RGB-D，其中 5 对与已发布失败 status 的源时间精确重合，另外 8 对是同一窗口的采样；最早失败源未被 5 Hz 录制捕获，不能写成 14 个失败源全部重放。13 对在第三次有界搜索后均通过原门，可组成 4 组三帧确认；同帧改用当前支撑拟合、但仍保持原两次搜索的对照仍为 13/13 触边。
- `gen6_arrival_rgb_boundary/production_replay_result.json` 使用实际修改后的到位 helper，恢复完整盒子轮廓；面积比约 2.039–2.148，仍低于原 4 倍限制。此结果没有放宽支撑、边界或外观门。
- `verification/observation_arrival_rgb_search/gen6_actual_station_process.json` 进一步调用真实 unknown producer 的请求回调及 `process()`：前两帧不发布对象，第三帧形成原三帧证明；处理 qualified 回执后，随后十帧正常跟踪均发布各自新源的对象/掩膜，参考时间严格递增，固定站位标记与原 anchor/平面保持。回放使用封存 RGB-D、TF 和 SDK 实测关节；参数传输为离线替身，时钟设为各帧源时间后 100 ms，不能据此证明未来实机调度必定满足新鲜度限制。

最终验证 **448 passed、1 skipped、0 failed、0 errors**，由节点/站位 365 项、RGB/桌面 67 项及新增低层边界 16 项组成，不重复累计早期回归。跳过项因旧本地 RGB-D fixture 未分发，与本次 generation 6 实录回放无关。结果、三批日志和 30 个源文件 SHA 清单位于 `verification/observation_arrival_rgb_search/deployment_sources.json`；前一部署清单中的其余 22 项 SHA 保持一致。

`arrival_rgb_station_deployment/result.json` 已记录 `deployment_passed=true`。仅重载 unknown、remote、task，PID 分别由 42891/42892/42893 变为 **50707/50708/50709**；驱动 8718、硬件接口 8721、MoveIt 8731、网关 21166、相机 8738、夹爪 11203、模式路由 8758、GUI 8779 的 PID、启动标记和父进程均保持。部署本身没有发送运动或力矩命令，没有查询温度。部署后的 generation 6 reset ACK 只是加载就绪屏障，不是恢复旧目标或启动下一轮。

## 最新手动调整与 generation 7 启动

`realigned_20261009T020127Z/` 曾记录用户对准确认，但约 02:16 UTC 的只读监督又发现用户改变关节位置，因此执行代理没有沿用该回执启动自动抓取，而是等待新的停稳确认。用户随后通过 **`call_c0a324790b1d44a4ae4a837592d91240:0`** 明确“已对准，可以继续自动抓取”。新 `realigned_20261009T022007Z/alignment_receipt.json` 记录 `previous_alignment_reused=false`、`CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE`；回执时间为 `1791512408182465076 ns`，图像源 `1791512408149846315 ns`，SDK 实测源 `1791512408141535807 ns`。六轴 q 约为 `[-1.906738,0.533825,-0.230097,-0.092039,-0.377359,-0.050621] rad`，作为此次新回执的测量记录，不可供后续实验直接复用。

已读取新实验 `grasp_records/20261009T022026.944578Z_cca56e3000b84b5289c1adfa0e09a9bb/authorization.json`：新选择为 **generation 7**、`unknown + observe_return`、stamp `1791512428462812662 ns`，`old_plan_reused=false`，录制继续 `local_only`。执行代理已启动本轮 runner；本次文档更新仅记录启动交接，不声称新一轮到位重测、后续接触或实物抓取成功。新修复尚不能替代一次性抓取的全部实物回归结论。

## Generation 7 到位确认通过及后续配准失败

上一节的启动状态随后已有封存结果：本轮于 **2026-10-09 02:21:45 UTC** 封存为失败。计划 `4e6c76325b0477ad558e381c` 实际完成观察运动，9 条独立实测样本证明静止；到位请求起点为 `1791512499989999055 ns`。unknown 与 remote 在约 **0.803 秒**后均确认三帧合格源：`1791512500048933982`、`1791512500386725664`、`1791512500540977478 ns`，随后 qualified 释放回执也得到双方 accepted 确认。正常跟踪继续，原到位 CAD 检查返回 `MEASURED_OBSERVATION_ENVELOPE_OK`，任务进入 near_field。这些是本轮真实日志证据，不再只是前述离线回放。

`arrival_rgb_station_deployment/consumers.log` 先于任务错误记录 `Observe-return reached view registration rejected: YAW_BOUND_EXCEEDED`；`runner.log` 也先记录 `OBSERVE_RETURN_REGISTRATION_FAILED: YAW_BOUND_EXCEEDED`，随后才返回 `NEAR_FIELD_PLAN_PHASE_INVALID: Preview is not a contact execution plan`。因此后者是没有合格近场接触计划的后续症状，本轮首因已转移到跨视角配准，不能继续归为到位 RGB 触边、三帧不足或双端 ACK 超时。

`gen7_registration_analysis/reconstruction.json` 使用封存的原参考点与精确近场源 `1791512500836532592 ns` 离线重建，同一参考点数组逐项相等，重现配准偏航 **13.533°** 超过原 **10°** 门限。该数值是封存输入的离线重建，失败瞬间的完整 RegistrationResult 没有直接序列化；它没有证明目标实际转动，也未构成配准修复。原门限保持，尚无有效近场接触执行或实物夹持成功证据。

为评估从观察终点独立重新获取的可行性，`gen7_current_arrival_capture/initial_acquisition_replay.json` 只读回放随后静止采样中的三对真实 RGB-D 与精确 TF：不带旧 anchor 或第三次搜索资格，原分割、中心唯一选择、默认 16→32 px RGB 及 `InitialAcquisition` 均通过，第三帧完成初始确认。三个目标中心距离为 **13.36、13.51、12.95 px**，小于原 **61.125 px** 上限。该结果仅说明这三帧可重新获取，不能证明实机时效、后续接触规划或原跨视角融合问题已解决。

既有 `.runtime/run_user_requested_grasp_20261008.py` 可记录持续抓取授权和本轮新的当前姿态 preflight，创建新 generation 后调用原 `mode_aware_grasp_runner.py`；它明确 `user_alignment_claimed=false`，无需伪造一次新的人工对准。原 runner 仍要求新 selection、新源计划、完整审计和全部执行门，不能复用失败 generation 或旧计划。此处仅完成代码只读核查，没有通过该入口启动实验。

不过用户随后再次手动移动，旧终点恢复方案没有执行，其图像、q 和离线结果只保留为诊断。最新用户确认 **`call_8ac81768f0634493bd1c9c5a278f0e50:0`** 对应 `realigned_20261009T022535Z/alignment_receipt.json`，回执时间 `1791512735876723289 ns`、图像源 `1791512735857094287 ns`，`previous_alignment_reused=false`、`CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE`。后续实验应绑定该新位置的实际测量，不能继承本节旧自动终点的获取或执行资格。

## 配准第三初值窄修、验证与 generation 8 启动

进一步对同一份冻结点云逐步回放，定位到可复现的算法缺口：原配准只比较恒等变换与 PCA 旋转加中点平移两个初值，缺少“零旋转、仅中点平移”的候选。原初始全点匹配代价分别为 **2.104437×10⁻⁵、1.486608×10⁻⁵ m²**，因此选择 PCA 并在有限 ICP 后停在 13.533° 的局部解。零旋转中点平移的初始代价 **1.336289×10⁻⁵ m²** 已严格更小；它随后收敛到 0.085801°，最终相同代价为 **1.265804×10⁻⁵ m²**，也低于旧结果的 **1.495257×10⁻⁵ m²**。这说明原候选集合遗漏了更好的合法初值；不需要以“能通过偏航门限”为由改选结果，也不能由点云回放认定物体真实转角或证明全局最优。

`multiview_surface.py` 仅在原重叠比例与 footprint 跨度准入条件内，加入第三个固定初值。其平移沿用原实测 footprint 中点及支撑高度计算，仅关闭 PCA 初始旋转；三者使用相同一对一对应、未匹配点惩罚和原全点代价，第三者必须严格低于原两者才替换，原恒等/PCA 平局规则保持。仍只优化唯一胜者，最多 **12 次**，没有三支各跑一次或失败后按门限换解。身份、时间顺序、支撑、偏航、实测点最大位移、内点、重叠和 RMSE 门限均未改变。源码 SHA256 为 `bc9cdab92bba792d1d82d780304cfc4debd0f93705189ef0ff8443dfce6df834`。

同一封存输入的实际原版/新版结果见 `verification/registration_translation_seed/result.json`：

| 指标 | 原版 | 新版 |
| --- | --- | --- |
| 结果 | `YAW_BOUND_EXCEEDED` | `REGISTERED` |
| 偏航 | 13.533173° | 0.085801° |
| 内点数 / 重叠比例 | 647 / 78.05% | 679 / 81.91% |
| 内点 RMSE | 1.075 mm | 1.147 mm |
| 实测点最大位移 | 10.855 mm | 5.790 mm |
| 原始支撑面角差 / 局部分离 | 0.333° / 1.649 mm | 相同 |

新版没有让每个指标都变小：内点 RMSE 略增，而覆盖更多实测对应点后的原总代价降低。五次交替离线耗时中位数为旧版 **202.25 ms**、新版 **119.46 ms**；本例新版只需五次更新收敛。此对照仅衡量这份点云在当前离线环境的配准函数，不是完整 ROS 负载时效或普遍加速保证。

独立纹理对照仍留下限制。`gen7_registration_analysis/texture_frozen_transform_comparison.json` 在相同冻结特征对应上比较变换；`texture_residual_decomposition.json` 将新版与严格 ORB 对应的约 **5.14 mm** 残差分解为约 **5.14 mm** 切向和 **0.17 mm** 法向 RMS，平均偏差主要沿盒子短轴，约 **5.08 mm**。已核对匹配点位于同一盒子顶面，但原图文字小、模糊且重复，少量近似共线特征不能排除相邻字符/文字行的系统偏配。因此它既不能证明剩余误差全部来自几何配准，也不能证明新版物理位置已精确；该纹理对照没有成为执行门或用于选择通过门限的分支。

最终 **233 passed、0 skipped、0 failed、0 errors，4.39 秒**，包含 multiview、新增初值专项、refinement continuity、final refinement stage/reference。旧历史记录的两项 PCA 分支测试显式固定该分支，保留原超角和位移拒绝断言；另有不替换任何初值的旧实录测试验证新默认选取。真实 10.5° 转动和 26 mm 位移负例保持，专项收紧四类最终门时要求输出变换逐项不变，以验证没有按门限通过与否择优。本次共享模块也用于直接/近场/最终精修，相关离线回归通过不等于所有一次性抓取实物回归完成。

冻结清单为 `verification/registration_translation_seed/deployment_sources.json`：上一版 30 项源 SHA 全部一致，加入本次生产文件与两份测试，共 33 项；外盘实录 fixture 单独绑定 SHA。`registration_translation_deployment_retry1/result.json` 记录部署成功，仅重载 unknown、remote、task，PID **50707/50708/50709 → 58101/58102/58103**。驱动 8718、硬件接口 8721、MoveIt 8731、网关 21166、相机 8738、夹爪 11203、模式路由 8758、GUI 8779 的 PID、启动标记和父进程均保持；部署本身未发送运动或力矩命令，未查询温度。较早部署尝试证据保留，最终成功以该 retry1 回执为准。

新实验为 `grasp_records/20261009T023854.657870Z_44fae4d9c54c40c8a8ecf0969e9ca5d1`。已读取其授权记录：用户确认仍为 `call_8ac81768f0634493bd1c9c5a278f0e50:0`，新选择 **generation 8**、`unknown + observe_return`、stamp `1791513536362171173 ns`，`old_plan_reused=false`、`physical_success=unverified`。执行代理已启动本轮实机任务；本节只记录启动和部署，不能把 generation 7 的反事实回放写成 generation 8 已到位、已接触或已夹持成功。

## Generation 8 接触缺面与下一次补观察的两处修复

Generation 8 随后于 **2026-10-09 02:40:56 UTC** 封存为失败，终止结果为 `NEAR_FIELD_SURFACE_VIEW_REQUIRED: current fused snapshot has no hard-safe candidate`。本轮已经完成观察运动、10 条独立实测样本的到位静止确认、三帧原门审核及双端 ACK，并进入真实 near-field 配准和三源实测表面融合。request **50** 的精确源 `1791513650897139310 ns` 返回 `REGISTERED`，偏航 **0.005775°**、重叠 **79.22%**、内点 RMSE **0.991 mm**；因此上一节配准修复已在该次实机输入上通过。融合保留 1672 个实测点及三个原源时间，不等于接触已安全。首因、审计 SHA 与原始分层结果见 `gen8_contact_analysis/analysis.json`。

本次 WSL 返回的 **12 个**候选全部未通过本地硬复核，主要拒绝为 `BILATERAL_SURFACE_EVIDENCE_MISSING`；没有进入接触 MoveIt 检查或执行接触/闭爪。拒绝标签可重叠，不能把标签计数当成候选总数。几何候选的 20 个方向中，15 个超过原单侧 25 mm 物理可达范围，其余 5 个不能同时满足实测共同支撑高度及原 **2 mm** 接触面重叠，因而 materialized 为 0。这不是应直接放宽接触门的证据。离线同源重建前次观察增益得到 48 个超过原深度噪声范围的新占据体素，通过原最少 3 个要求；该增益未直接序列化于失败瞬间，证据为 `gen8_contact_analysis/gain_gap_reconstruction.json`，不应表述为日志直接报告。

在仍有真实缺面的情况下，后续流程还存在两个可独立复现的结构缺陷：

1. base 将本次已完成、零合格接触搜索报告为 `NEAR_FIELD_SURFACE_VIEW_REQUIRED`，而 remote、task 和 runner 的补观察合同只接纳旧 `NEAR_FIELD_NO_HARD_SAFE_CANDIDATE` / `NEAR_FIELD_NO_REACHABLE_CANDIDATE` 形态，提前返回，未进入下一次补观察。修复保留新错误码本身，仅在同源注册已合格、审计 outcome 一致、搜索已完整结束、没有选中候选且原计数证明零本地合格/零路径检查时，贯通这项失败及其完整来源证明；不把未完成或存在有效接触候选的搜索当成此分支，也没有扩展诊断观察的枚举。
2. 原补观察准备将 **phase reference** 坐标中的融合点与 **current snapshot** 中的几何/相机配对，误得 0 个缺面。`gen8_contact_analysis/gap_frame_comparison.json` 在同一冻结输入上使用已有注册逆变换恢复当前快照点，得到 **2 个**原门定义的缺面：一个方向只有 5 个正侧实测点，另一个方向缺少可共同使用的支撑高度。修复只让缺面计算使用既有 `_snapshot_multiview_surface()`；原 reference 表面、观察增益、身份、源时间及 provenance 保留。注册、同源证据锁、ticket、session 截止和最多观察次数均未改变。

最终相关回归 **401 passed、0 skipped、0 failed、0 errors，3.78 秒**，包括新错误码全链路、未完成/非零计数/旧源/错误身份拒绝，以及非恒等旋转和平移下的真实快照转换与原 reference 不变测试。证据在 `verification/surface_view_supplement/{result.json,final.log,final.xml,production.patch}`。部署清单 `deployment_sources.json` 共 **36 项**：前版 33 项中 31 项 SHA 不变，remote/task 为本次预期修改，另加入 runner 合同及两份测试。runner 合同不在前版清单，不能声称它经过前版 SHA 对比。全部原接触、视角增益、路径、跟随误差、新鲜度和执行门仍需在新实验中真实通过；这些测试不代表全部一次性抓取实物回归完成。

## 两处修复部署及 generation 9 启动

用户再次明确“已对准，可以继续自动抓取”，消息 **`call_f0ad39fad2c54bd09509eaf46da4351a:0`** 对应 `realigned_20261009T024903Z/alignment_receipt.json`。新回执时间 `1791514143673924922 ns`、图像源 `1791514143643605470 ns`、SDK 实测源 `1791514143643041552 ns`，记录 `previous_alignment_reused=false` 和 `CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE`。它取代此前对准位置，不能复用 generation 8 的旧计划。

`surface_view_supplement_deployment_retry1/result.json` 记录部署成功。首次 ROS shutdown RPC 丢失回执后，三个消费者中仅旧 unknown 留存；执行代理只停止这三个消费者所属的旧 unit，再重新启动消费者，先前尝试证据保留。最终 unknown/remote/task PID 为 **63701/63702/63703**，替代 **58101/58102/58103**。驱动 8718、硬件接口 8721、MoveIt 8731、网关 21166、相机 8738、夹爪 11203、模式路由 8758、GUI 8779 的 PID、启动标记和父进程均保持。部署本身没有发送运动或力矩命令，没有查询温度；加载时旧 generation 的 reset ACK 仅表示消费者就绪，不赋予旧目标执行资格。

新实验为 `grasp_records/20261009T025544.067303Z_a3bedd2adc674427b0b193378c16fd52`。其 `authorization.json` 绑定上述新用户确认与新选择 **generation 9**、`unknown + observe_return`、stamp `1791514545763998985 ns`，`old_plan_reused=false`、`physical_success=unverified`。本次更新仅记录部署完成及新实机实验启动；尚不能声称第二次补观察、接触或夹持成功。实验继续保存于 SanStand，采用 `local_only`。

## Generation 9 到位 RGB 确认失败及最新重新对准

上一节启动的 generation 9 已封存为失败，准确状态链见 `gen9_arrival_analysis/analysis.json`。计划 `215b913b44ed482e85389842` 的观察运动实际完成，9 条独立 SDK 实测样本证明静止，关节峰峰值为 0。执行前和到位后的 requalifying 请求均正常获得 unknown、remote 双端 accepted ACK；原三秒采样窗为 `1791514596262291669` 至 `1791514599262291669 ns`。

窗口内完成的 **10 帧**审核中，**9 帧**返回 `rgb_instance_reaches_search_boundary`，只有源 `1791514596847848892 ns` 达到 `initial_instance_confirming_1_of_3`，下一帧触边后确认计数清零。没有形成三帧 qualified 证明，最终为 `OBSERVATION_MEASUREMENT_ACK_TIMEOUT`。另有一帧在截止后才完成，不能计作窗口内可用样本。此次不是 ACK 传输未到：真实新鲜 exact RGB-D 持续到达，首次永久目标失效发生在超时撤销之后。该轮没有进入配准、下一次补观察或接触/闭爪，因此既不能把 generation 8 的注册成功移算到本轮，也不能声称新 enum/frame 两处修复已被本轮端到端验证。到位 RGB 的具体边界机制继续用本轮封存数据分析；仅凭这个汇总不支持扩大截止时间或继续扩大搜索窗。

用户随后再次手动改变姿态，并通过 **`call_e20a192ae19a4d6c8556894f27f5b7b3:0`** 明确“已对准，可以继续自动抓取”。`realigned_20261009T030157Z/alignment_receipt.json` 记录新回执时间 `1791514917574584245 ns`、图像源 `1791514917552989959 ns`、SDK 实测源 `1791514917543085070 ns`，`previous_alignment_reused=false`，使能状态为 `CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE`。本次新位置不沿用 generation 9 的自动终点；修复验证完成后应创建新选择、新源计划再继续，不恢复失败计划。

本轮文档更新同时核对并修正会话 `current_progress.json` 与工作树 `.runtime/latest_full_ros_start.json`：两处 `latest_record` / `latest_grasp_record` 统一指向 generation 9 已失败记录，去除“generation 9 正在运行”和 generation 6 最新结果的过时描述，最新对准绑定 030157 回执。原自动终点 q 保留为 generation 9 的历史测量并明确不是当前姿态；新对准位置尚未启动自动实验。此次仅更新文档、镜像及进度元数据，没有调用 ROS 或修改生产代码。

## 定向第三次 RGB 搜索修复及 generation 10 启动

对 generation 9 保存的 12 对到位 RGB-D，原生产到位 helper 重放为 11 帧触边、1 帧通过；这是录制采样集合，与上一节窗口内 10 条已发布审核状态不是同一集合。代表帧深度种子左缘为 x=162，原第三次 48 px 搜索窗左缘为 x=114，三像素硬背景使可用内界成为 x=117，而真实白盒左缘约 x=115，仍截到盒子。支撑独立审核、唯一原目标关联已通过，触碰的是算法裁剪边界。对照中，四侧一律扩至 64 px 会把底部无效 RGB padding 纳入训练，11 帧被原有效 RGB 门拒绝，不能用这种无差别扩窗修复。

此次仅改变已有到位资格下的第三次尝试：读取第二次搜索的实际触边方向，触边侧使用 **64 px**、其余侧保持 **48 px**（按原图像尺度换算）。默认初始/一次性抓取仍为 **16→32 px**，总次数最多三次；只有原 `rgb_instance_reaches_search_boundary` 允许重试，第三次仍触边或任何其他原门失败均拒绝。首次可信核心、有效 RGB 整窗检查、唯一连通分量及 85% 核心保留、4 倍面积上限、真实深度/身份审核均未改变。到位之后仍必须通过原固定站位 marker 的来源、selection、epoch、固定 q 和截止时间验证才能沿用第三次资格；不保存新几何权限或增加运动权限。生产 `rgb_instance.py` SHA 为 `24983c94e2aaaa5fe2005f944a3ce9bbef1fd6f91db14e2191cf35681ee009c9`。

最终回归为 **132 passed、1 skipped、0 failed、0 errors，9.80 秒**；跳过项仍为未分发的旧本地 RGB-D fixture。专项覆盖四个方向及组合、同一绝对像素核心、未扩侧不引入 padding、实际扩展侧无效 RGB 必须拒绝、第三次换边仍不得再重试。真实修改后的 helper 对 generation 9 的 12 对及 generation 6 的 13 对均通过，分别形成四组三帧确认。进一步调用真实 unknown 请求回调和 `process()`，generation 9 完成三帧确认、qualified 回执及随后 **9 帧**正常跟踪，旧 generation 6 完成三帧及随后 **10 帧**；原 anchor、平面和固定站位 marker 保持。证据为 `gen9_arrival_analysis/production_directional_station/gen9_actual_station_process.json` 和 `production_gen6_station/gen6_actual_station_process.json`。回放使用封存 RGB-D/TF/实测关节，但参数传输和源年龄时钟为离线替身，设为 100 ms；不证明未来实机调度一定满足原新鲜度或三秒截止。

独立审查通过的冻结清单为 `verification/arrival_directional_rgb_search/deployment_sources.json`，SHA **`159057b2a4ae3564889f47008e7dcedfa0830105e3d23a79e00b5e22b40406d6`**，共 **37 项**：上一版 34 项源 SHA 不变，修改 RGB 文件及其专项测试，新增专属消费者重载脚本。该脚本核对旧部署的 PID、启动标记、父进程和固定三消费者 launch 精确 argv，只停止对应专属 systemd unit；替代此前逐节点 shutdown 的竞态窗口，不改变硬件进程或任务执行器。

`arrival_directional_rgb_deployment/result.json` 已记录 `deployment_passed=true`，新 unit 为 `alicia-observation-measurement-1791515401461283485`。unknown/remote/task PID **63701/63702/63703 → 69156/69157/69158**，新父进程为 69106；驱动、硬件接口、MoveIt、网关、相机、夹爪、模式路由和 GUI 八个核心进程身份均与上一部署一致。重载仅切换 GUI 控制归属并调用逻辑 reset，没有发送运动、力矩或温度查询。随后执行代理按新选择启动自动任务，不能把部署期间的 GUI 接管状态当成实验仍在等待人工操作。

新实验 `grasp_records/20261009T031019.431802Z_cf2238ea63164fd2a86f610925c32457/authorization.json` 绑定用户确认 **`call_e20a192ae19a4d6c8556894f27f5b7b3:0`** 及 030157 对准回执，选择为 **generation 10**、`unknown + observe_return`、stamp `1791515420966685295 ns`，`old_plan_reused=false`、`physical_success=unverified`。当前正在自动运行；文档及两处进度指针统一指向本轮，旧 generation 9 失败与终点测量保留为历史。此处不声称新一轮观察确认、后续接触或夹持成功，也不能将离线通过等同于全部一次性抓取实物回归。


## Generation 10 封存失败：接触预检与后续补观察

Generation 10 于 **2026-10-09 03:11:42 UTC** 封存为失败，任务最终返回 `NEAR_FIELD_PLAN_PHASE_INVALID: Preview is not a contact execution plan`。此前真实观察运动、到位三帧确认和注册已通过；request **24** 审计 `88ea33aa28b5406fa05c8bec8d0b1d8c3e6173ddada0c34144853578f1dacdd2.json` 中，16 个本地合格接触候选全部完成检查且没有可达候选，拒绝原因为 `CONTACT_SEQUENCE_PRECHECK_FAILED`：给定原位姿/开口容差时，名义接触终点与目标 OBB 存在不可避免的指体重叠。此后 request 25 确实开始了下一次观察搜索，但检查 4/251 项后预算结束。不能将未检查的其余候选认定无解，也不能将最终阶段错误直接归为到位 RGB 失败。

当前独立复核正在检查接触预检使用的注册坐标系、指体与开口模型及接触阶段允许范围；此节只记录封存事实，不预判碰撞门错误或放宽门限。此前本轮“运行中”的记录是启动时间点状态，两处进度指针已改为本轮失败；失败计划和旧测量没有重新获得执行资格。

## Generation 10 两项确定缺陷的修复与独立验证

接触坐标系复核确认了一处合同不一致：原 `_apply_registration_evidence_to_plan()` 在注册通过后，把四个阶段位姿变换到固定参考坐标系，却没有对该计划持有的 `object_geometry.pose_base` 施加同一变换。此前局部几何检查已经让候选与物体使用同一注册参考，而最终预检拿到的是参考系位姿配当前帧物体 OBB。修复仅将计划自有的物体 OBB 位姿加入原刚体变换；`build_rich_plan()` 已复制几何消息，因此不修改冻结 snapshot 或原 anchor。物体尺寸、身份、源时间和当前实测支撑平面保持，注册接受条件及碰撞门不变。无注册、无效证据与恒等变换路径分别有回归覆盖。

这处修复不能被写成 generation 10 的 16 个接触候选已经安全。`gen10_failure_analysis/contact_obb_quantification_v2.json` 独立调用原必要碰撞函数与容差核心，确认在修正到同一坐标系后，16 个候选的左右指体仍有 **32/32** OBB 碰撞；原容差核心最小重叠约 **8.7912 mm**。开口 50 mm 下原三体支撑距离的重建误差小于 1 nm，没有漏掉支撑抬升；候选声明的夹持轴也与姿态轴一致，没有 90° 回写错误。点云与指体包围盒的重叠是几何诊断，不能等同于完整 STL 网格或实物碰撞证明。

注册几何验证位于 `verification/contact_registered_geometry/`：原函数在新回归上预期失败，补丁后 **495 passed、0 failed、0 errors、0 skipped，13.774 秒**。新增九项覆盖四阶段/物体同变换、源与平面不变、碰撞判断的刚体不变性、计划摘要改变以及失效证据拒绝。主代理复核计划几何的复制所有权，独立代理复核实际补丁及同坐标系碰撞量化。生产 `remote_grasp6d_node.py` SHA 为 `0bf7453f91814d35d3d262b4dc5b83d58611fe3bf79f8eb5f826e7909ceaf857`；前两次测试中的新 fixture 字段缺失及修正日志也保留，没有把它们写成生产回归失败。

观察搜索的确定缺陷来自阶段语义混用。`AdaptiveStageProfile.depth_uncertainty_m` 为接触距离设计，实际等于视觉测量项加执行端点残差。本轮近场 profile 为 **13.551953729 mm = 3 mm + 10.551953729 mm**。原补观察入口冻结这个总量用于缺面方向和 FK 提议，切回 `observation_prefix` 后端点侧面筛选却因远场不读取执行残差而使用 3 mm；于是同一视觉任务的生成和审核采用不同含义。新增纯函数只从同源 profile 的两个显式字段拆出视觉项，缺字段、布尔值、非有限数或非正差值均拒绝，不修改输入，不把缺失执行误差默认当作零。

普通独立观察的初始预检、接触失败转观察与后续补观察统一冻结该视觉项；独立 lattice/FK 序列的侧面信息、缺面增益、重复视角判断和注册后的新增体素增益沿用这一含义。实际增益函数原本只在注册通过且来源链追加后，统计最新视角保留的点在旧体素之外、距离旧点超过测量噪声的新增体素，不以命令位姿误差作为新增信息。本轮第一观察的真实 `previous_gain` 本就使用 **3 mm**，以 1.5 mm 体素和至少 3 个新增体素的原规则计得 19 个新增体素；后续没有因深度噪声实测变化而升至 13.55 mm 的依据。原接触 profile 总量与全部距离、注册接受函数、体素大小、至少三体素、身份/来源、CAD、关节/FOV/跟随/停止/完整路径门和搜索预算均保持；诊断 silhouette 路径也保持原语义。

`gen10_offline_ik/information_split_result.json` 使用封存 request 25 几何与原模型作离线对照：总量 13.55195 mm 时得到零个合格 FK 提案，视觉 3 mm 时得到两个通过原端点门的提案，生成约 0.177 秒；其中一项原跟随误差盒支撑下界为 3.100 mm、图像边距约 99 px、预计缺面增益约 3.20 mm。这仅是提案与端点证据。原 depth/mask、安装变换和目标角点可按 SHA 核对，重新计算的光学/支撑浮点数组 hash 与原审计不同，不能宣称所有中间点云逐 bit 复原；同数量点云、实测相机原点及绑定信息已核对。附加的 `information_straight_path_review.json` 是对合成关节直线调用原路径 guard 的有条件复核，不是现场规划器生成的完整带时轨迹，也不替代新一轮执行资格或新鲜到位测量。

信息语义修复验证为 **344 passed、0 failed、0 errors、0 skipped，6.005 秒**，位于 `verification/observation_information_uncertainty/first.xml`。测试使用真实接触 profile 验证全部字段不变、冻结映射兼容、补观察及实际注册回调都取得视觉项；小于 3 mm 噪声、少于三体素、旧 provenance、未注册、异身份和断来源链仍拒绝。旧 fixture 仅补明确的 `execution_position_error_m=0`，没有删除或放松原断言。新 helper SHA 为 `cdb6775fc7e4c951943af6aa0626b812f196a99bcf8b4b0c83d6f24767bbc6f5`。这两批测试分别列出，不冒称去重后的全库总数或一次性抓取的全部实物回归。

合并部署清单位于 `verification/gen10_contact_and_information_deployment/deployment_sources.json`，含 **44 个**源/测试/运行脚本：上版 37 项中 34 项 SHA 不变、3 项按本轮预期更新、7 项首次纳入跟踪；“首次纳入”不代表文件均为新建。独立复核已将完整清单与现有文件逐项核对，并读过上述生产差异和反例测试，未发现剩余部署阻断。部署由主代理执行，后续以 `contact_and_information_deployment/result.json` 回执追加结果；本节写入时不声称已加载或新实机通过。

## 本轮最新人工对准：03:17:54

用户在 generation 10 失败后又手动调整，并通过 **`call_21eaaaeca6d242a0a39288a7cea33f74:0`** 明确“已对准，可以继续自动抓取”。新 `realigned_20261009T031754Z/alignment_receipt.json` 记录回执 `1791515874877351760 ns`、图像源 `1791515874857744455 ns`、SDK 实测关节源 `1791515874843086050 ns`，`previous_alignment_reused=false`、`CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE`。六轴 q 约为 `[-2.132233,0.348214,-0.039884,-0.276117,-0.909651,0.113515] rad`，只是此次对准时刻的实测记录。后续新选择必须绑定本次确认与当前新测量，不复用 030157 回执、generation 10 自动终点或失败计划。本次文档工作没有编辑运行状态指针，没有调用服务或发送运动命令。


## 两项修复部署完成及 generation 11 启动

`contact_and_information_deployment/result.json` 已记录 `deployment_passed=true`，新专属 unit 为 `alicia-observation-measurement-1791517264347118543`。unknown/remote/task PID 从 **69156/69157/69158** 变为 **78957/78958/78959**，新父进程 78906；驱动、硬件接口、MoveIt、网关、相机、夹爪、路由和 GUI 八个核心进程的 PID、启动标记及父进程全部保持。重载本身没有发送运动或力矩命令。最终已复核清单 SHA 为 **`768bcd0aa3b2b1a3ad1fadfd28459f2d8f6c587ec7e698cb514ed0763e6e3fc6`**，`reviewed=true`；部署后仍须创建新选择，不能执行旧 generation 的计划。

首次包装记录 `grasp_records/20261009T034119.472117Z_505fb21f69ee4cc49f8a2d0326189b76` 在创建新选择前被当前位置与 031754 对准回执的差异检查拒绝，runner 返回 1；执行代理复核 Joint5 差约 0.4617 rad，并恢复 GUI 手动控制，没有发起抓取运动。该记录已关闭，manifest 为 `state=complete`、`recorder_closed=true`、`result=unknown`，原因是没有观测到真实任务结果。因此它是启动前置条件拒绝，不是第 11 轮抓取失败，也不冒充旧位置仍有效。

用户随后通过 **`call_50b02d2e4ba64d2ba8e0c98d6f5cb3be:0`** 再次明确“已对准，可以继续自动抓取”。`realigned_20261009T034221Z/alignment_receipt.json` 记录回执 `1791517341778239727 ns`、图像源 `1791517341736778497 ns`、SDK 实测源 `1791517341743077911 ns`，`previous_alignment_reused=false`，使能仍为 `CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE`。本次六轴 q 约为 `[-1.969631,0.431049,-0.107379,-0.090505,-0.447922,0.098175] rad`，取代上节 031754 作为本轮启动依据，历史回执不覆盖。

新实验为 `grasp_records/20261009T034234.183696Z_9e74e994513542879c9e556c6ccd2180`。已核读 `authorization.json`，新选择为 **generation 11**、stamp `1791517355664368391 ns`、`unknown + observe_return`，绑定 034221 用户确认，`old_plan_reused=false`、`physical_success=unverified`，仍采用 `local_only`。真实消费者日志已在 `1791517381.767439` 进入 `MOVE_PREGRASP moving 6D pregrasp`，计划 `c4bd273190ef431cb3114d5d` 的执行期测量请求获得 unknown、remote 双端 accepted ACK。本节仅记录已经发生的启动与首个观察运动交接，不声称到位确认、后续接触或实物夹持完成。


## Generation 11 封存结果：到位确认超时

Generation 11 记录于 **2026-10-09 03:43:34.707933 UTC** 封存，manifest 为 `state=complete`、`recorder_closed=true`、`result=failure`，runner 返回 6。真实计划 `c4bd273190ef431cb3114d5d` 已获得绑定审计并进入首个观察移动，执行前双端 accepted ACK 见上一节。消费者在 `1791517407.568461` 发出同一 lease 的 `requalifying` 请求，随后于 `1791517410.586276` 记录 `OBSERVATION_MEASUREMENT_FAILED: OBSERVATION_MEASUREMENT_ACK_TIMEOUT`，执行槽释放；`runner.log` 中 `GRASP_RESULT success=False` 与封存任务状态相符。

本轮失败发生在观察移动后的到位确认阶段，尚未进入本轮注册后接触规划及第二次补观察。因此不能把这轮当成注册 OBB 修复已经救出接触候选，或新视觉信息项已经完成第二视角的端到端实机验证。超时本身不能证明 RGB 再次触边、支撑平面不符或 ACK 传输丢失；具体首个拒绝原因须由本轮封存 detector status 和请求/回执逐项判定，不能挪用 generation 9 或 generation 10 的首因。没有实物夹持成功证据，失败计划未获重新执行权限。


## Generation 11 年龄检查首因、窄修及恢复处理

封存 `gen11_arrival_analysis/analysis.json` 与 `causal_summary.json` 进一步确认：三秒窗口内九次完成审核按三组重复 `confirming_1_of_3 → confirming_2_of_3 → source_exceeds_existing_frame_limit`，没有记录到原 RGB 触边或支撑审核失败；到位请求已经得到双方 accepted ACK。三组第三帧在状态被 recorder 收到时的年龄分别为 **303.738、383.181、372.360 ms**，对应第一帧历史年龄为 **877.618、1049.548、974.365 ms**。这是状态接收时刻的精确差值，不是未记录的内部检查时间；结合生产代码顺序，可确定当前第三帧仍新鲜，而此前已逐帧完成审核的历史确认样本被末尾的全三帧年龄循环再次拒绝。remote 也存在同样的历史样本复查，单改 unknown 不能完成交接。

修复只涉及 `unknown_tabletop_perception.py` 和 `observe_return_measurement_remote.py`：每帧进入、退出原完整测量审核时仍检查原 **0.8 秒**新鲜度，任何失败仍清空确认；前两帧只作为固定到位窗口内已审核的身份历史，只有第三帧真实对象和 mask 发布当前几何。unknown 取消末尾全部历史年龄复查，并在 hold 构造之后、`input_lock` 内提交前重新检查第三帧；remote 取消相同历史年龄循环，保留最终帧在数据拼接、原 callback 后及 task qualified 释放时的新鲜度验证。三帧、固定三秒窗口、严格递增且到位后/截止前的来源链、原 anchor/支撑/RGB/唯一关联、连续实测静止及离站锁存、selection/epoch/config、失效退休和原后续几何/注册/接触/路径门均不变，不延长任何当前几何有效期。

最终五文件回归 **294 passed、0 failed、0 errors、0 skipped，26.035 秒**，证据在 `verification/arrival_confirmation_freshness/first.xml`。新增测试包括历史首帧超过 0.8 秒但当前第三帧新鲜的 producer 正例、按 generation 11 第一组原始时间间隔构造的 remote 交接正例；审核、hold 构造、锁等待导致当前帧过期，以及到位前历史、无序来源、最后释放时过期仍拒绝。独立审查确认只有两份生产文件和两份对应测试改变，其余 **40 项**部署源 SHA 保持。待部署 44 源清单 `verification/arrival_confirmation_freshness/deployment_sources.json` SHA 为 **`47c9d225b0980ff09b57d91d8ad1169843cffc3e781a8dbb59ecde9e2e231351`**，`reviewed=true`；unknown/remote 源 SHA 分别为 `cf8372e4c59f14097bd41ec11a959a69f8cf5ae4c973792d0df5cd6d8879b674`、`f0b2c3bb36ab14a03ef3e55d01b7b627cb7381501cd0abbf12c829d5357fd4e6`。

另一个独立对照实际调用旧、新 unknown producer，使用本轮保存的三对到位 RGB-D 和原 TF、原 anchor/运动前参考，并把完成时钟设置为第一组在线状态的接收时间。旧版复现第三帧历史过期拒绝，新版前两帧不发布、第三帧完成确认并只发布该帧真实几何，见 `gen11_arrival_analysis/actual_producer_completion_clock_{baseline,fixed}.json`。在线第一组前两帧 raw 未被 recorder 保存，因此这三个实际可回放源并非在线首组的逐帧等同重放；固定完成时钟也是离线反事实，不证明真实部署后的调度、到位成功或实物抓取。

用户曾要求完成修复后暂停、醒来再实机验证，冻结 `result.json` 和清单据此明确标记 **`ONDISK_NOT_DEPLOYED`**：当时运行节点仍对应上一版清单 `768bcd0a…`，保留旧历史年龄规则。之后用户明确恢复“继续处理”，并通过 **`call_d24557f7c7cc48dbbedf376c9fb6dcb2:0`** 再次确认已对准。新 `realigned_20261009T042656Z/alignment_receipt.json` 记录回执 `1791520016644124984 ns`、图像源 `1791520016612159490 ns`、`previous_alignment_reused=false`、`CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE`。旧暂停和冻结状态保留为时间点证据；是否已经加载、是否新一轮执行成功，须另读新的部署/实验回执，不用本节待部署清单推断。本次审查和文档工作没有调用 ROS、修改运行指针或发送运动命令。


## 新鲜度修复实际部署及 generation 12 启动

上节冻结时的 `ONDISK_NOT_DEPLOYED` 随后已有真实部署回执：`arrival_freshness_deployment/result.json` 为 `deployment_passed=true`，新 unit 为 `alicia-observation-measurement-1791520078637595828`。unknown/remote/task PID **78957/78958/78959 → 87122/87123/87124**，新父进程为 87071；八个保留核心进程身份完全一致，重载本身没有发送运动或力矩命令。该回执记录的是实际加载，早前冻结文件仍保留生成时状态，不改写历史。

已核读实验 `grasp_records/20261009T042814.711798Z_02d7a2117e114e0ea476744c19fa64c4/authorization.json`：新选择 **generation 12**、stamp `1791520096125842094 ns`、`unknown + observe_return`，绑定 042656 的真实用户确认 `call_d24557f7c7cc48dbbedf376c9fb6dcb2:0`，`old_plan_reused=false`，录制继续 `local_only`。runner 已提交新计划 `7c7fb2cd9c00d86e52210283`（源 `1791520108099282026 ns`）并调用 `GRASP_START_CALL`。本节只记录新轮启动，尚不声称三帧确认、近场接触或实物夹持完成。


## Generation 12 封存结果

本轮于 **2026-10-09 04:29:12.214922 UTC** 封存，manifest 为 `state=complete`、`recorder_closed=true`、`result=failure`。计划 `7c7fb2cd9c00d86e52210283` 进入实际首个观察移动，执行期双端 accepted ACK 正常；`1791520145.135844` 发出到位 `requalifying` 请求，`1791520148.140370` 报 `OBSERVATION_MEASUREMENT_FAILED: OBSERVATION_MEASUREMENT_ACK_TIMEOUT`，runner 返回同一失败结果。本轮尚未进入接触执行或形成实物抓取成功证据。

这与 generation 11 的表层超时代码相同，但本轮加载的两端源码已经移除历史全三帧年龄复查，不能凭终端字符串把首因也写成相同。具体测量状态、三帧进展与 ACK 接收链须按本轮封存来源继续核对；本节不把离线 294 项通过写成实机到位确认已通过，也不据此放宽当前几何新鲜度或固定确认窗口。

## Generation 12 到位掩膜缺块首因及窄修验证

`gen12_arrival_analysis/analysis.json` 后续核明：到位原三秒窗口内有 11 次完成测量审核，8 次为 `arrival_original_rgb_identity_not_matched`，2 次仅到 `initial_instance_confirming_1_of_3`，另 1 次要求唯一原目标关联；额外 1 条 waiting 状态不是测量结果。执行前和到位请求的双方 accepted ACK 正常，但没有形成 qualified 三帧凭据。因此本轮首因不是上一轮全历史年龄复查，也不能仅凭轮廓距离推断物体换了身份或原外观阈值过严。

封存同源 RGB-D 显示白盒深度分成四片，只有其中一片满足原 anchor 的唯一关联。原 16 px 裁剪中有 **565 个真实白盒像素**落入裁剪硬背景边；选中深度片 7 px 膨胀区以外，其他真实测得的白盒片仍被初始化为 `PR_BGD`。代表帧即使扩大至 48 px，缺失三角区的 **1703 个像素**仍为软背景，其原始实测高度为 **13.37–17.80 mm**，不是已测得的桌面。原分割因收缩在裁剪内部而没有触边，不能触发既有扩窗重试。证据为 `gen12_arrival_analysis/label_diagnostic/result.json`。

同一硬核心与原门的消融回放中，只加当前测得软前景、仍从 16 px 起搜为 **5/12** 通过；只从 48 px 搜索为 **2/12**；两者合用为 **12/12**。独立 `colour_diagnostics/result.json` 核对前后 RGB、depth、raw seed 完全相同及原 Lab 直方图逐项一致：代表帧掩膜从 10593 增至 13083 像素，原颜色距离从 **0.407221** 降至 **0.364285**，原 0.4 门未改。新增区域主要为明亮中性色，说明本例缺块改变了外观统计；封存相机参数没有与图像绑定的实际曝光、增益或白平衡读数，不能把本轮归为曝光变化，也不需要加入颜色归一化分支。

本次只修改 `rgb_instance.py`，SHA **`964ce5162a627618aaa44e183cdcf576db6a3eccca922461036753e3e52c2c44`**。仅 `observation_arrival_retry=True` 且有当前测量几何时，用原 16 px 裁剪生成并冻结同一硬前景核心，实际 GrabCut 从 48 px 开始；当前裁剪内有限、深度和支撑相对高度均在原范围的测得前景，只追加为 `PR_FGD`。原硬核心、实测支撑背景及裁剪边缘硬背景随后覆盖这些软标签。只有触边允许再试一次，触边侧 64 px、其他侧 48 px，最多两次；默认初始/一次性抓取以及无几何路径保持原搜索顺序和标签规则。原有效 RGB、唯一核心组件、85% 核心保留、4 倍面积、真实深度/中心/尺度/质量、唯一目标关联、颜色/轮廓 0.4、三帧 IoU、固定站位及当前新鲜度/截止门都保留。软前景不赋予新身份或运动权限。

最终四文件回归为 **96 passed、0 failed、0 errors、0 skipped，6.276 秒**，含真实 OpenCV 同色/异色相邻物体分离、实测桌面切断 RGB 连接桥、桥下缺深度时合并结果触发原 4 倍面积拒绝，以及无效 RGB、无效深度/高度、固定核心和有界重试负例。新生产 helper 对本轮保存的 12 对 RGB-D 全部通过原身份审核，第三个源 `1791520145744392156 ns` 首次形成三帧确认。它们不是线上全部审核帧；末个保存源在线处理完成时已超过原截止，离线图像通过不能倒推当时可接受。旧 generation 6、9 的真实 unknown 回调和 `process()` 回放分别完成三帧确认后 10 帧、9 帧正常跟踪，anchor、平面、固定站位 marker 不变；这两组使用明确的 **100 ms 模拟源年龄和替代传输**，不证明实机完整负载时效。结果及限制见 `verification/arrival_measured_probable_foreground/result.json` 与其中的实录引用。

## 到位软前景修复实际部署及 generation 13 启动

最终已独审的清单 `verification/arrival_measured_probable_foreground/deployment_sources.json` SHA 为 **`0a777de037a352bf9d896387110b6b078734a3a14e56de373ac2b7506d807c72`**，共 46 项：44 项前版源中只有上述 RGB 文件改变，43 项逐项不变，另加入专项测试和本轮回放脚本。其冻结时 `ONDISK_NOT_DEPLOYED` 保留为历史状态；新 `arrival_measured_foreground_deployment/result.json` 已记录 `deployment_passed=true`，unit 为 `alicia-observation-measurement-1791521280144361989`。unknown/remote/task PID **87122/87123/87124 → 94328/94329/94330**，新父进程 94277；驱动、硬件接口、MoveIt、网关、相机、夹爪、路由和 GUI 八个核心进程的 PID、启动标记和父进程均保持。部署回执明确未发运动或力矩命令，旧 generation reset 只表示新消费者就绪。

Generation 13 启动所用人工对准来自 **`call_64668017aa6546dcbebdb4b3269ae7e2:0`**，`realigned_20261009T043455Z/alignment_receipt.json` 记录回执 `1791520495633526563 ns`、图像源 `1791520495586889743 ns`、SDK 实测源 `1791520495605263291 ns`，`previous_alignment_reused=false`、`CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE`。已核读新实验 `grasp_records/20261009T044819.001213Z_4af08222d69a4f308e3e174629881c1c/authorization.json`：新选择 **generation 13**、stamp `1791521300428654193 ns`、`unknown + observe_return`，绑定本次确认，`old_plan_reused=false`、`physical_success=unverified`，保存策略为 `local_only`。本节记录部署和新轮启动，后续实际结果另补；离线通过不等于本轮观察、接触或夹持成功。

## Generation 13 实机到位通过、近场阶段交接超时

本轮计划 `01577fa94864542a70ba3a98` 完成首个实际观察运动，10 条独立 SDK 实测样本证明到位静止，六轴峰峰值均为 0。`arrival_measured_foreground_deployment/consumers.log` 记录到位请求起点 `1791521353422146081 ns`，unknown、remote 在 `1791521354.560319` 均返回 qualified，耗时约 **1.138 秒**，认可同一组三个真实源：`1791521353617894172`、`1791521353950798511`、`1791521354216399431 ns`。随后 task 的 qualified 释放请求也获得双方 accepted ACK。这是新生产版本本轮真实确认，不能再把本轮停点归为 generation 12 的 RGB 身份确认不足。

之后近场等待没有得到可执行的接触计划。真实日志出现后续观察方案 `PREVIEW_READY`，但 task 仍以接触阶段要求校验，最后记录 `NEAR_FIELD_PLAN_PHASE_INVALID: Preview is not a contact execution plan`；没有开始第二次观察运动。在 `1791521445.020`，任务以 `NEAR_FIELD_DIRECT_TIMEOUT: no fresh near-field 6D preview after 90.0s` 失败，runner 的 `GRASP_RESULT success=False` 与封存 manifest 一致。记录于 **2026-10-09 04:50:49.591682 UTC** 关闭，`state=complete`、`recorder_closed=true`、`result=failure`。

本轮已跨过到位 RGB 与测量交接，下一待处理项为近场与后续观察计划之间的阶段交接。超时和最后的计划类型拒绝是确切状态，不能据此声称所有接触候选碰撞已解决、后续观察已经执行或实物抓取已经成功。043455 对准 q 只保留为此次任务的历史起点；当前没有新自动任务，运行硬件保持，本次文档和状态更新没有发送 ROS 命令、运动或力矩命令。


## Generation 13 之后的新人工对准

用户再次手动调整后，通过 **`call_1a4fbf85d60b4525a61680ad54546a5d:0`** 明确已对准。`realigned_20261009T045306Z/alignment_receipt.json` 已记录回执 `1791521587333266735 ns`、图像源 `1791521587302617788 ns`、SDK 实测源 `1791521587300728503 ns`，`previous_alignment_reused=false`、`CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE`。这是一份尚待修复验证后用于新任务的确认，取代 043455 作为后续启动依据；不得因补记旧 generation 13 的失败而将新确认标为已消费或旧起点。当前继续修复补观察审计与阶段交接，本节没有启动新自动任务，也没有修改运行状态指针。

## 补观察内部审计修复、部署及 generation 14 启动

`gen13_handoff_analysis/arrival_and_terminal.json` 保存 generation 13 的实际证据：到位通过、已发布补观察需求及 5 份后续观察预览，但只有 view 1 的会话，没有可由 task 消费的 supplemental audit。代码复现定位到 `_build_diagnostic_observation_gate_audit()` 无条件调用仅允许初次入口的 `observe_return_audit_error()`；合法 `supplemental_observation` 因此被拒绝，原非晋升分支没有写出成功审计。线上没有持久化完整私有 pending/gain 对象，不能声称重建了当时全部内部状态；该结构缺口另由实际 builder、原审计落盘及 task callback 的组合测试复现。

本次仅增加内部 `supplemental_observation_audit_error(report, session, pending_request, now_ns)` 并让 builder 在 supplemental phase 调用它。校验逐项绑定真实 v2 observing session、待处理请求及摘要、原计划/目标/来源、下一 view index、最多观察数、固定截止、已注册增益及完成的接触失败证明，然后继续原 measured source/缺面增益及 strict observation 审计门。外部 runner 的默认初次入口校验仍拒绝 supplemental；task 现有会话、gateway admission、原计划冻结事务和严格执行流程，以及通用 promotion 均未改。此修复只让合法内部预览获得可消费审计，不授予外部直接启动补观察或接触执行的权限，也不解决此前接触几何碰撞本身。

最终两组互不重复回归为 **181 + 173 = 354 passed、0 failed、0 errors、0 skipped**，各耗时 5.589 和 1.001 秒。正链实际调用 builder 与非晋升 finalizer，将审计写盘后由原 task callback 加载并匹配会话；默认外部入口、错误会话/待处理摘要/源/增益/路径门仍拒绝。两份生产文件 SHA 为 runner contract **`34e939ce53de82cf42e1bde9d200fa82a83052bfe7b472a88306e86db36337f4`**、silhouette planning **`8fdcd929d16b3964878301b88d960b02efe8e20d7035bbca3a4e5a856d4dd477`**。证据在 `verification/supplement_preview_audit_handoff/{final.xml,existing_task_runner.xml,independent_review.json}`；47 源清单 `deployment_sources.json` SHA **`4184229e6cf90bfa7ac44397b525161e293640602ebfde13d48c44ec41031fe1`**，46 项旧源中 44 项不变、2 项预期修改，另新增专项测试。

`supplement_audit_handoff_deployment/result.json` 已记录 `deployment_passed=true`，unit 为 `alicia-observation-measurement-1791522206694235302`。unknown/remote/task PID **94328/94329/94330 → 102039/102040/102041**，新父进程 101988；八个保留核心进程的 PID、启动标记和父进程与前次完全相同，部署本身没有发送运动或力矩命令。

新实验 `grasp_records/20261009T050350.036425Z_ce203c7b9d4c4ba29e4303eb51b69d62` 的授权记录绑定上节 **045306** 用户确认 `call_1a4fbf85d60b4525a61680ad54546a5d:0`，新选择 **generation 14**、stamp `1791522231536506414 ns`、`unknown + observe_return`，`old_plan_reused=false`、`physical_success=unverified`、`local_only`。上节“尚未启动”是新确认刚收到时的历史状态，本节只记录后来实际部署及新轮启动；到位、第二次观察与接触/夹持结果须待本轮真实记录补充。本次仅更新文档和外盘镜像，没有修改运行状态指针或调用 ROS。

## Generation 14 封存结果：到位通过，后续视角搜索未完成

真实计划 `3e1ac52e2a05531c12565f2d` 执行首个观察运动后，于 `1791522291829627275 ns` 发起到位确认；unknown、remote 在 `1791522292.555731` 均返回 qualified，约 **0.726 秒**，绑定三源 `1791522291963380813`、`1791522292163151502`、`1791522292337444305 ns`，随后 qualified 释放也得到双方 accepted ACK。原到位确认在第二轮新输入上真实通过。

近场计算仍出现 `CONTACT_SEQUENCE_PRECHECK_FAILED`，原因明确为名义接触终点在原位置/开度容差内存在左右指体与目标 OBB 的不可避免碰撞，不能写成接触碰撞已修好。随后 runner 报告 `UNKNOWN_OBSERVATION_NO_SAFE_VIEW: OBSERVATION_SEARCH_INCOMPLETE: bounded comparison checked 4/210 views; unchecked views are not proven unreachable`。这是本轮有界视角搜索的实际终止，不能把其余 206 个未经检查的候选称为已证明不可达；四个已检查视角的具体拒绝首因仍由本轮审计另行分析。

任务最后返回 `NEAR_FIELD_PLAN_PHASE_INVALID: Preview is not a contact execution plan`，执行槽释放，记录在 **2026-10-09 05:05:27.823672 UTC** 封存为 `complete/failure`、`recorder_closed=true`。本轮没有第二次观察执行或接触/闭爪成功记录。因没有获得新的合格补观察方案，本轮也不能替代上一节内部审计修复的完整实机接续验证；最终计划类型错误不应掩盖此前搜索未完成的终止证据。本次只补文档及外盘镜像，运行状态指针由执行代理维护。

## Generation 14 FK 光轴证据缺项修复与实际部署

封存审计进一步确认，两个 FK 提案通过了缺面射线增益、视野、夹爪包络、跟随误差盒支撑下界和测得支撑投影，但随后被原 `_observation_side_evidence()` 拒绝：光轴相对支撑面的侧面投影证据比原 **3 mm** 门少 **0.6 mm 以上**。相机中心到目标的缺面射线增益与相机光轴的侧面投影是两个独立条件；原 FK 优化只包含前者，因而产生了在进入严格路径检查前就必然被拒的提案。这不是已经证明其余视角不可达，也不能用这两个端点的缺项解释全部四次严格路径拒绝。

本次只修改 `reachable_observation_views.py` 和必要的 `mode_aware_remote_grasp6d.py` 调用接线。helper 使用当前 prepared 的 OBB 轴、尺寸和归一化支撑法向计算原目标高度，使用 FK 与原手眼变换得到相机光轴，按原 `height × sin(incidence) ≥ uncertainty` 公式增加优化约束，并在端点重新审核。优化加入 **0.1 mm** 提案余量，接受条件仍是原门；求解器自报成功也不能绕过端点复核。当前 prepared 视觉 profile 的光轴门限显式传入并纳入缓存键，冻结缺面证据的射线增益噪声保留独立参数。原 adapter 光轴门、所有后续完整路径与执行门，以及两秒累计搜索、方向数、迭代数和原严格检查预算均不变。

最终三文件组为 **53 passed、0 failed、0 errors、0 skipped，2.495 秒**，见 `verification/fk_optical_side_evidence/final.xml`。测试覆盖倾斜支撑/非恒等手眼与原 base 函数数值等价、真实冻结源、无效当前门限拒绝、门限变化使缓存失效却不续两秒预算，以及伪造 solver success 返回原 gen14 坏端点时其余五门真通过但光轴门仍拒绝。封存数据回放中，gen14 新版得到两个原端点合格提案，耗时 **0.240 秒**、12/10 次迭代，光轴投影均 **3.100 mm**，预计缺面增益约 **3.200/3.300 mm**；gen10 仍得到两个提案，候选 q 与旧版逐值相同。原 RGB-D、目标角点、手眼及模型散列可核对；TF 重建存在浮点中间数组 hash 差异，故不声称逐 bit 复原所有原支撑数组。此回放只证明固定输入下的端点提案，不是完整轨迹或实物抓取通过。

48 源清单 `verification/fk_optical_side_evidence/deployment_sources.json` SHA 为 **`7c53daccf67abd26fe569d6fe46c1b9889cd12aabab6108ae4b33a6cb26aacb4`**：原 47 项中 45 项保持、2 项预期修改，新增一个专项测试。独立审查通过，helper/caller SHA 分别为 `c34897a4515f330f10b7a9457e7e1b2bec0656d1f92956e07b7d84e39bdbbe4b`、`f71b7b6017aa107e274a2e506418f05043de3a98bff93f77e773b1ceeff9fc19`。`fk_optical_side_deployment/result.json` 已记录 `deployment_passed=true`，unit 为 `alicia-observation-measurement-1791523186306167775`。unknown/remote/task PID **102039/102040/102041 → 108020/108022/108023**，新父进程 107970；八个核心进程的 PID、启动标记和父进程全部保持。部署本身没有发送运动或力矩命令。

## 05:21:15 新对准与 generation 15 实际进展

首次包装记录 `grasp_records/20261009T051958.573094Z_5eb816159eb1439eaa7809bdf751a109` 因 Joint3/Joint5 实测位置相对上次对准发生变化，在创建新选择前停止，没有 `authorization.json`。其 manifest 已在 **05:21:16.934864 UTC** 封存为 `complete/unknown`、`recorder_closed=true`，原因是没有真实任务结果；这不是一次已执行的抓取失败。

用户随后通过 **`call_1ef407083da84b23be6010b672fc3d75:0`** 再次明确已对准。`realigned_20261009T052115Z/alignment_receipt.json` 记录回执 `1791523276141511678 ns`、图像源 `1791523276120566368 ns`、SDK 实测源 `1791523276109318773 ns`，`previous_alignment_reused=false`，使能为 `CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE`。新实验 `grasp_records/20261009T052127.805554Z_6b8770587c5c4cd8abc679178eaa1c70/authorization.json` 绑定本次确认，选择 **generation 15**、stamp `1791523289233554124 ns`、`unknown + observe_return`，`old_plan_reused=false`、`physical_success=unverified`，记录继续 `local_only`。

真实计划 `01932b2605a2586ce5f87bdc` 在 `1791523317.926370` 进入首个观察运动，执行期双端 accepted ACK 正常。到位请求为 `1791523355634818315 ns`；unknown、remote 于 `1791523356.292150` 同时确认三源 `1791523355637379169`、`1791523355938484191`、`1791523356105984210 ns` 合格，约 **0.657 秒**，随后 qualified 释放得到双方 accepted ACK。本节写入时实验仍在进行，尚无第二次观察、接触或实物夹持成功结论；后续须依据本轮封存结果追加。本次只写操作文档和外盘镜像，没有修改运行状态指针、控制源码或调用 ROS。

## Generation 15 封存结果：第二、第三视角已执行，随后 RGB 身份歧义失败

本轮继续完成了两次真实补观察。第二视角计划 `0f46f6eae6cbd8cbc43ceacc` 在 `1791523368.683990` 进入运动，到位请求 `1791523373152940750 ns`，双端于 `1791523374.068073` 返回 qualified，约 **0.915 秒**，确认来源为 `1791523373441584110`、`1791523373642383337`、`1791523373841673374 ns`。第三视角计划 `3c980163b43d370fb33cace3` 在 `1791523418.379749` 进入运动，到位请求 `1791523441064351320 ns`，双端于 `1791523443.534610` 返回 qualified，约 **2.470 秒**，三源为 `1791523442892243862`、`1791523443057922601`、`1791523443295139074 ns`。两次 qualified 释放均得到双方 accepted ACK。因此，本轮已有补观察计划交接、实际路径执行及到位确认通过的证据，不能再描述为只生成预览而从未执行第二视角；外部 runner 对 supplement 的初始启动拒绝日志仍是预期边界，不是这两次内部执行失败。

第三次到位释放后，正常感知在源 **`1791523444257713317 ns`** 报首失，观测时间 **`1791523444493005037 ns`**，原因是 `perception_failure: metric_ambiguity_no_unique_complete_rgb_identity`。任务与 runner 随后明确返回 `TARGET_LOST`，执行槽释放；本记录于 **2026-10-09 05:24:09.245038 UTC** 封存为 `complete/failure`、`recorder_closed=true`。本次失败发生在三次观察均到位合格后的 RGB 身份歧义处理，不能归为到位三秒窗口失败或 FK 光轴门再次拒绝；具体掩膜/分支原因仍需同源回放核实，也没有证据说明接触碰撞或实物夹持已解决。

`manual_after_gen15_receipt.json` 记录 `1791523502245796203 ns` 时任务已不活动、GUI 手动关节控制已恢复，恢复操作没有发送运动或力矩命令。失败 generation 15 与旧计划不恢复执行资格；新的手动对准和后续自动任务由执行代理单独记录。本次只更新文档及外盘镜像，不修改运行状态指针。

## Generation 15 同源诊断与两项感知窄修（尚未部署）

实录状态进一步将首失定位为 **`support_gap_measured_revalidation`**，而不是多候选几何歧义入口。上一源 `1791523444091633558 ns` 因 `support_plane_inlier_ratio_low` 保持待恢复；首失源 `1791523444257713317 ns` 已重新取得通过原门的当前支撑面，原 tracker 也只匹配一个候选。其源年龄约 **235 ms**，没有证据表明此次因 0.8 秒时限失败。异常字符串来自两分支共用的 RGB 身份函数，不能据函数名称误判实际处理阶段。

第一项接线缺口确实存在：正常同站精修会传递已验证的到位搜索资格，支撑恢复与几何歧义恢复的两处精修回调却沿用默认参数。修复仅传递同一站位 reference，并在 RGB 处理前后、几何歧义保持状态提前返回的锁内再次核验；外层发布前的 marker 验证与继承不变。原 0.8 秒、固定到位 q/会话截止、身份/唯一关联、支撑、RGB IoU **0.85**、颜色/轮廓及替代对象非归属条件均保留。该中间版本分别通过 143 项恢复测试和 118 项原路径测试，见 `verification/arrival_recovery_refinement/`；但真实首失帧默认搜索 IoU 为 **0.711168**、传到位资格后为 **0.719114**，均未达 0.85，因此接线修复本身不是实录首失的充分解决方案，未单独部署。

第二项同源对照确定了旧参考掩膜的污染来源。上一合格源 `1791523443858786582 ns` 的在线 mask 可逐 bit 重建：它从已测得的白盒延伸到桌布，包含 8156 像素。该帧锁定参考平面与当前独立测得平面在目标足迹处最大差约 **3.281 mm**，仍在原 4 mm 一致性门内；原算法只要旧平面有一个可匹配候选便直接使用它分割，把接近前景高度边界的桌布区域也带入深度种子和硬前景。保持这帧原 RGB-D、固定 anchor/平面/足迹及全部门，仅调用已有 `segment(fit_current_support=True)` 后，参考 mask 变为 **5958 像素**；首失帧 mask 仍为 5884 像素，原身份函数得到 **IoU 0.957032、颜色距离 0.025458、轮廓距离 0.020433**，全部通过。证据为 `gen15_handoff_analysis/current_support_rgb_counterfactual/result.json`，没有降低身份门限、擦改原深度或手工修旧 mask。

相应窄修在 `tabletop.segment_with_support_fallback()` 增加默认 `False` 的 `prefer_current_support` 参数。仅到位确认，或正常处理已经核验的固定到位 marker，启用该偏好；只有旧平面恰有一个匹配且当前独立支撑审核通过，才使用原当前拟合路径重新取得本帧测量。旧平面已有两个可匹配身份时不会靠重新拟合丢掉一个；已有几何歧义保持期也不切换分割口径。新测量保持 `current_ring_fit` 真实来源，固定 tracker anchor/参考平面/足迹不改。normal 路径将 marker 验证移到分割前，分割后重核同一对象；若资格在分割中才建立，则暂扣这帧、等待下一帧。原发布锁内审核、实际几何、RGB 精修和接触门全部保留，默认初始及普通直接路径不启用此偏好。

最终去重联合回归为 **259 passed、1 skipped、0 failures、0 errors，27.140 秒**，见 `verification/arrival_current_support/final.xml`；唯一跳过项是项目未分发的旧本地 RGB-D fixture，未用伪造数据补齐，也不与前述中间测试简单相加。当前 unknown/tabletop SHA 分别为 `096b3d79ec114317a51e5109955b2473690f78c7845a934bdae4b5641a13dc7d`、`31bb3f085d24db9585762dcc6e2962556de966a61e34822266a3627cac781f05`。独立审查确认默认路径、原多候选拒绝及分割/RGB 期间站位失效的生产链约束仍在。

`gen15_handoff_analysis/patched_current_support_wrapper/result.json`（SHA **`02493e27c73368b866d8865b5c4cbb7bd09463d3ce9d1e77ab8f82f6c39d1979`**）实际调用修改后的分割 wrapper、原唯一匹配、原 RGB 精修及原身份函数，处理封存 **22 帧**：20 帧普通测量、1 帧缺支撑暂扣、1 帧首失源恢复，未出现该回放链的新拒绝；anchor/参考平面/足迹保持。首失后保存的 16 帧也通过原普通测量门，但相邻 mask 的最小 IoU 仍约 **0.719**，普通路径原本不要求逐帧 0.85，故不能声称全程轮廓完全稳定。回放从第三个真实到位合格源开始，前两个到位 RGB-D 未保存；它未模拟 unknown 节点的传输、source/lease/marker 和发布，也不能证明完整三帧在线确认、当前新鲜度或实物成功。首失后的图像是归档反事实，不会复活已经退休的 generation 15。本节写入时两项感知修复只在磁盘验证，尚未加载。

## 原接触高度窗口内的有限提案与合并冻结（尚未部署）

Generation 15 的接触分析与上述感知首失分开处理。封存源 19 的两种对称方向中，原名义接触几何及高度余量通过，但基于实测偏差的 predicted-command 连续指体检查拒绝目标 OBB 碰撞；原算法只检查窗口内的单一高度，尚未利用同一实测接触带中其他合法高度。`gen15_contact_prediction/production_height_full_geometry_and_curve.json` 使用原接触函数复算，正向 **2.788342 mm** 提案在两个方向上均保留原 **±2.676768 mm** 实测误差半径，并通过原四阶段解析几何及 nominal/predicted-command 连续夹爪几何；负向 **−3.062288 mm** 提案仍被原 command 碰撞门拒绝。曲线来自封存目标的离线 IK 重建，原失败 MoveIt 轨迹未保存，因此这不是完整机械臂场景、可执行轨迹或实际接触成功证明。

窄修仅在已注册的 unknown tabletop 接触计划出现确切 `predicted_command ... grasp ... target OBB collision` 后，使用原包含零点的连通高度窗口，扣除完整原误差半径，在正、负内部余量的 75% 处各提一个候选，最多两个。每项重新计算原注册几何、接触证据及高度余量，保持原累计位移上限、stream ticket 和请求截止，并执行原必要终点和完整 contact sequence precheck。只有全部通过才一起更新四个位姿、开度、接触证据、计划摘要和最终执行序列；原成功、未注册、其他候选来源及其他失败类型不进入新分支。第一项必要几何拒绝可继续第二项，但不会获得执行权限。选择与发布重新构建时仍从原冻结来源重算并重新检查，不累计高度移动，也不延长预算。

接触相关 11 文件最终 **245 passed、0 skipped、0 failures、0 errors，11.565 秒**，含 28 个新专项；与感知最终 259 passed / 1 skipped 为互不重复文件组，合计 **504 passed / 1 skipped**。最终 base remote/helper SHA 分别为 `28bec48fdb95ecb0e5cb83a6741c503b9834f07dcc2f699b381212e9c324bc2a`、`cd241f6ae6633f97dd5aa1776ac1d69c8c5ae41f267d351d0b8d09a20b070806`。`verification/gen15_perception_and_contact_height/deployment_sources.json`（SHA **`8e2ed9f58353d1b8290d656ec835984a5a2ed36ad4401affaf3e2a44248a4138`**）冻结 52 项：原 48 项中 45 项完全不变、3 项预期修改，另纳入已有接触高度 helper 和 3 个新测试。独立审查通过，清单状态仍为 `REVIEWED_ONDISK_NOT_DEPLOYED`；这些回归不能替代一次性抓取的全部实物回归。

用户通过 **`call_a423bf0efe9342dab769ea28ae6753a9:0`** 再次确认“已对准，可以继续自动抓取”。`realigned_20261009T054722Z/alignment_receipt.json` 记录回执 `1791524843041975259 ns`、图像源 `1791524843009641408 ns`、SDK 实测源 `1791524843009338485 ns`，`previous_alignment_reused=false`、`CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE`。该回执采集时仍显示旧 generation 15，不能视为复活旧目标或已有新任务授权；新部署和新 generation 应以随后实际回执补记。本节只更新文档与镜像，没有修改运行指针或发送 ROS、运动、力矩命令。

## 合并修复已部署，generation 16 在首次运动前失败

`current_support_contact_height_deployment/result.json` 已记录 `deployment_passed=true`，新 unit 为 `alicia-observation-measurement-1791524893918001175`。unknown/remote/task PID **108020/108022/108023 → 119957/119958/119959**，新父进程 119906；八个保留核心进程的 PID、启动标记和父进程全部与部署前一致。加载的是上述 52 项冻结清单，部署本身没有发送运动或力矩命令；上节“尚未部署”是冻结时的历史状态。

新实验 `grasp_records/20261009T054827.257779Z_e15d068b793c43ada6713a78f421f2bb/authorization.json` 绑定 **054722** 最新人工确认，新选择为 **generation 16**、stamp `1791524908935017585 ns`、`unknown + observe_return`，`old_plan_reused=false`、`physical_success=unverified`。本轮在首次运动前终止，runner 返回 `NO_FRESH_MODE_PREVIEW`，终端原因明确为 `TARGET_LOST: perception_failure: source_exceeds_existing_frame_limit`，首次丢失观测时间 `1791524921194578409 ns`。记录在 **2026-10-09 05:49:42.584316 UTC** 封存为 `complete/failure`、`recorder_closed=true`。本轮没有观察执行或接触验证，不能据此称上述感知恢复和高度提案已通过实机，也不能将本轮失败归为新的接触碰撞。

代码复核表明，初始静止站位的支撑缺测恢复同时受旧 last-good 参考与当前帧各自的原 0.8 秒限制；producer 过期会退休目标，remote 也有独立的 `last-good + 0.8 s` 固定 quarantine 截止。它与先前到位三帧历史列表的时效修复不是同一合同，不能只去掉一处检查便声称可恢复。确切当前帧/旧参考年龄、缺测原因和处理耗时仍以本轮封存分析为准。本轮未发生手动或自动位姿移动，执行代理保留最新用户授权，GUI 自动控制所有权保持；本次文档操作没有改运行状态指针或调用 ROS。

随后 `gen16_handoff_analysis/first_loss_analysis.json` 核定：旧参考为 `1791524920393871784 ns`，其后两次 `support_plane_inlier_ratio_low` 源为 `1791524920594934463`、`1791524920799158334 ns`；恢复帧 `1791524920993564844 ns` 在首失时仅 **201.014 ms**，旧参考却达到 **800.706625 ms**。该帧 `support_gap_measured_revalidation` 耗时约 98.388 ms 并正常返回，完整支撑与 RGB 身份已在旧参考约 **797.06 ms** 时通过；最终 `post_refine` 阶段约 3.613 ms 后，提交前重复旧参考检查越过原截止。整个处理为 150.179 ms / 线程 CPU 143.901 ms，没有本轮 CPU 长停顿或当前图像过期的证据。初始站位尚无 arrival marker，此轮没有启用新增的同站当前支撑偏好或到位 RGB 搜索资格。后续修复应处理这项跨提交时刻的合同边界，不能声称本轮已检验或否定了尚未触达的到位/接触分支。

## 保留原截止的 6 Hz 配置负载试验

为先验证更小范围的时序改善，专用三节点 launch 仅增加 `/unknown_perception/detect_hz`，类型 `double`、值 **6.0**，将原默认 5 Hz 请求周期从 200 ms 缩短为约 166.7 ms；原 0.8 秒及所有身份、支撑和执行门不变。节点本来使用单个串行 `rospy.Timer` 和 latest-only 精确 RGB-D 槽，处理超时不会并发重入，也不处理旧图像 FIFO。该参数只在初始化读取，须实际重载节点并创建新选择；它不是相机帧率或其他 detector 的 8 Hz 参数。6 Hz 能否在完整负载中减少两次缺测后的恢复等待仍待真实验证，不能从周期算式保证成功。

`verification/gen16_detect_hz6/deployment_sources.json` SHA **`0834d59a225364ddff30a502dc5ec14c5120bc4f412a7dd4806e0667274e54f7`**，52 项中只有 launch SHA 改为 `928bbc41514922255505034f19025a478918f3355c59714f8aec2455fcd1d2ba`，其他 51 项精确不变。XML 解析、原三消费者及 task remap、去除新增两行后还原前版散列的独审均通过。原 **504 passed / 1 skipped** 是未变生产源码与测试的继承证据，不是新增 6 Hz 测试；此配置没有新算法测试需求。本节记录清单冻结，部署及下一轮实际结果以随后回执补记，没有修改运行状态指针。

## Generation 17：6 Hz 已部署，首次运动前 RGB 身份恢复失败

`detect_hz6_deployment/result.json` 记录部署通过，unit 为 `alicia-observation-measurement-1791525306356424347`；unknown/remote/task PID **119957/119958/119959 → 122999/123000/123001**，新父进程 122948。八个核心硬件及网关/GUI 进程身份全部保持，部署没有发送运动或力矩命令。

新记录 `grasp_records/20261009T055520.443822Z_b937f5b63a8b4e3cb90f741b2eb36871` 使用同一份真实 **054722** 用户确认，期间没有手动或自动位姿移动；`authorization.json` 记录新选择 **generation 17**、stamp `1791525322144184112 ns`、`old_plan_reused=false`、`local_only`。实际用户原话仍是 `call_a423bf0efe9342dab769ea28ae6753a9:0` 的“已对准，可以继续自动抓取”；关于本轮 6 Hz 验证的执行说明不是另一条用户原话。

本轮确已在 6 Hz 请求配置下进行完整规划负载，但没有开始任何观察或接触运动。首失观测时间为 **`1791525329658025026 ns`**，终端原因是 `TARGET_LOST: perception_failure: metric_ambiguity_no_unique_complete_rgb_identity`；runner 最后返回 `NO_FRESH_MODE_PREVIEW`。记录于 **2026-10-09 05:56:26.781234 UTC** 封存为 `complete/failure`、`recorder_closed=true`。本轮没有重现 generation 16 的 `source_exceeds_existing_frame_limit` 终止，但该单轮结果不能证明 6 Hz 吞吐改善已稳健解决原期限问题。此次实际处理分支、掩膜与唯一身份失败原因仍待同源回放确认，不能只凭共享异常名称认定发生多候选身份歧义。

接触高度提案仍未在新实机流程中触发，因此其在线完整轨迹和实物夹持效果仍未验证。生产源码继续冻结；本节只同步客观结果及外盘文档镜像，不更改运行指针或发起新的自动任务。


## 初始固定站位当前支撑与 generation 18 的真实结果

Generation 17 的同源回放确认：旧 locked-plane 分割附带桌布纹理，而同一帧独立 current-support 分割及默认 RGB 搜索使首失身份 IoU 从约 0.848 提升到 **0.917656**；不是放宽 0.85 门限。随后仅对精确 `unknown + observe_return` 初始三帧确认后的站位保存不可滚动的实测 q/source/epoch/config/anchor/plane 标记，在新鲜 RGB-D、真实关节前后括号、无手动接管及原固定位置范围内优先测量当前支撑。前后及提交前均重核；移动再返回不能续用标记，多候选不能借重拟合选目标，RGB 默认搜索不变。`direct`、`two_stage` 保持原分割路径；`observe_return` 内初始视角足够时的后续测量也会使用新支撑偏好，不能称该内部路径逐位不变。

该版本 **390 passed、0 skipped、0 failed**，`verification/initial_station_current_support/deployment_sources.json` 冻结 54 项。`initial_station_current_support_deployment/result.json` 已记录部署通过。封存实录 `grasp_records/20261009T061104.522156Z_f598aaa2d91444a99bdf1a87af163419` 的 generation 18 仍在首次运动前失败；新站位支撑资格实际已生效，不能把本轮写成“补丁未运行”。约 12.14 秒的 acquired 至首失期间多次支撑缺测引发 `TARGET_MEASUREMENT_PENDING`，取消观察准备；最终首失为支撑缺测恢复时的 `metric_ambiguity_no_unique_complete_rgb_identity`，不是多候选选择、0.8 秒过期或新运动失败。当前源仅约 0.254 秒，旧几何参考约 0.486 秒。

`gen18_handoff_analysis/canonical_rgb_replay/result.json` 逐位复原已发布掩膜：固定 acquired 源 **1791526266737004041 ns** 为 5382 像素，滚动 last-good **1791526278569562435 ns** 为 6264 像素，当前首失 **1791526278801508188 ns** 为 5720 像素。滚动参考对当前的原完整轮廓 IoU 为 **0.8499536894**，颜色/形状分别 **0.069873 / 0.063355**；固定 acquired 对同一当前测量的原 IoU 为 **0.9194329184**，颜色/形状 **0.073797 / 0.017799**，全部原门通过。6264 像素的参考向右下桌布纹理延伸；它与前一帧的 IoU 仍为 0.857300，因此简单增加滚动相邻 0.85 检查不能充分防止此次参考污染。该反事实指定 acquired 模板，没有按哪个模板能过门择优。

另一个独立问题是恢复后原 cutoff 要求五帧都重新采集。真实稳定窗口多次不足等待五个新样本，导致准备反复取消。`gen18_handoff_analysis/ready_fusion_replay/result.json` 使用封存的真实合格 ready 历史及当前恢复后源，经原五帧融合函数通过稳定性检查；其中选定窗口最新源年龄 0.195270 秒、跨度 0.999668 秒、最小相邻 IoU 0.873142、最大质心位移 3.386855 像素、关节变化为零。旧记录没有新增的固定站位凭证，这只是实际数据可满足原融合门的离线反事实，不能给旧 generation、旧计划或旧图像恢复执行权。

## 固定初始身份模板与观察稳定历史准入（磁盘修复，尚未部署）

固定模板修复仅在上述初始站位创建时保存真实 acquired RGB 与归一化掩膜的只读副本及独立 source/hash。模板用于支撑缺测恢复的原完整 RGB 身份比较；当前候选、深度、支撑、mask 精修、几何和 last-good 的原 **0.8 秒** 截止继续来自当前实测。普通 status 公布固定 provenance，remote 在 acquired 的真实 ObjectPose、mask、ready 三消息任意顺序收齐且仍新鲜时，核对收到的规范化 mask 像素 SHA 后固定模板来源；RGB SHA 是 producer 的同源声明，不声称 remote 重新读取 RGB。后续缺失、替换或错误来源拒绝。恢复凭证把历史 `identity_template_source` 与新鲜几何 `reference_source_stamp_ns` 分开，不用历史模板刷新时间。

独立观察历史准入只针对已实际进入普通补观察、同固定站位且完成原 typed support recovery 的窗口：历史样本必须在当时真实 ready 且新鲜时入集，固定选择、epoch、标定/模型/配置、anchor、q 和模板来源一致；最新样本必须是 cutoff 之后的新实测。原五帧融合、跨度、稳定性及新鲜当前源检查保留，重新建立 ticket、prepared 和几何审计。接触、direct、旧 ticket、失效几何和已退休目标不获此资格。新增每个实际融合输入的 mask 散列用于把历史 ready 凭证与真正融合像素逐项对应，不改变融合数值算法。

上述固定模板生产/消费链及原支撑恢复五文件 **373 passed**；历史准入与原 remote/fusion 回归去重 **170 passed**，合计 **543 个不重复测试通过、0 skipped、0 failed**。最终模板与历史准入同进程交互 **61 passed / 14.36 秒**，已包含在 543 中，不重复计数；8 项相关生产/测试 SHA 在最终交互前后全部一致。3/5 帧、有/无 mask 四种 before/after 融合对照中，原数值字段和四个数组逐位相同，新增的每帧散列只携带来源。`verification/gen18_template_and_observation_history/deployment_sources.json` SHA **`d58045693c3aa2a6ac8ae4b0258ce08227c60d8e197c4f8990170c27f4289503`** 冻结 58 项：原 54 项中 50 项完全不变、4 项预期修改，另纳入 4 项新来源/测试。双方独立审查及主代理复核通过。

本节冻结时修复尚未部署，实物抓取成功仍未验证；原接触高度提案也尚未在这些首动前失败轮次触达。运行指针由主执行代理维护，本次仅修改项目操作文档与外盘镜像，不调用 ROS、不查询温度、不发送运动命令。

## Generation 19 启动消息顺序缺口与 generation 20 的实际验证

`template_and_observation_history_deployment/result.json` 已记录前述 58 源版本部署通过，unit 为 `alicia-observation-measurement-1791527918703753453`，八个核心进程身份保持。Generation 19 记录 `grasp_records/20261009T063901.432766Z_887d01a579944723b92f7000c6446937` 最终仍在首次运动前失败。`gen19_handoff_analysis/diagnosis_summary.json` 定位了更早的消费顺序缺口：真实 acquired 的 status、object 和 mask 已到达，但 streaming 在其后约 166 ms 才启动；此前 object/mask 回调未缓存原 acquired，后续固定模板一直无法完成三消息绑定，使 typed gap 落到普通目标丢失路径。另一个独立的最终问题是旧参考从处理开始的约 0.697 秒经过 188.789 ms 分割后达到 0.887 秒；599 条真实关节反馈完全静止，不能把泛化异常名称当成实际手动或机械臂运动。

启动修复只在精确当前 unknown/observe_return 选择下缓存实际收到的 acquired object/mask；未 streaming 时不建立身份 reference 或模板权限。原 start 建立新身份后，锁内按真实 source、原年龄和实际 mask hash 合并三消息；不重放旧 RGB-D 到五帧缓存、不刷新时间，也不吞掉真实 False。最终 **217 passed / 23.44 秒**，含实际 MRO 的 24 种消息/启动排列和过期拒绝。59 源清单 `verification/prestream_acquisition_identity/deployment_sources.json` SHA 为 `57bebc0f6a7db53d137e978c3efcaf4e1b0b67cb9ec9c372b7cf0df4ca8c1e6c`。`prestream_acquisition_identity_deployment/result.json` 确认部署通过，unit 为 `alicia-observation-measurement-1791535485996429588`，八个核心进程保持，部署未发运动或力矩命令。

Generation 20 封存记录 `grasp_records/20261009T084503.785647Z_98a11257317947ada4721dad54fdb028` 实际保持同一 remote 身份，出现 21 次 typed pending 和 **16 次原身份恢复**，固定 acquired 模板不变；启动丢缓存引发的 epoch 反复退休未重现。该轮仍未发生观察或接触运动，于 **08:46:34.053170 UTC** 封存 failure。最终源 `1791535532141202449 ns` 距旧合格源 `1791535531442678928 ns` 仅 **0.698523521 秒**，到首失观测 `1791535532277718305 ns` 时当前源年龄 **0.136515856 秒**，但旧参考年龄 **0.835039377 秒**。三次支撑缺测后，本帧分割的约 117.662 ms 跨越了旧参考原 0.8 秒截止。874 条实测关节反馈各轴范围为零，没有本轮移动证据。详细事实见 `gen20_handoff_analysis/diagnosis_summary.json`。

## 及时采集的初始静止身份恢复：双时钟合同已冻结，尚未部署

本次将旧几何授权截止与及时采集的新测量完成时间明确分开，仅用于已存在 typed support gap、固定初始站位与 acquired 模板、精确 unknown/observe_return 的恢复。首个凭证仍必须在旧参考原截止前建立；恢复源严格满足 `旧参考 < 当前源 < 旧参考 + maximum`，提交时当前源仍严格小于原 maximum，且不能晚于固定 `旧参考 + 2 × maximum`。maximum 使用原配置并至多 0.8 秒，不因新缺测帧滚动续期。整个待恢复期间几何保持撤权，旧计划取消；原当前支撑、唯一关联、完整 RGB 身份、固定 q/epoch/config/selection 和发布锁内复核全部保留。缺凭证、迟到首凭证、普通无 pending、direct/two_stage 和其他恢复分支仍走原严格路径，已经退休的身份不能补凭证复活。

producer 与 remote 共同校验版本化完成凭证，所有 age 保留真实值。只有完成全部原恢复门后，B 历史准入的临时 reference 才按这张新恢复源的原新鲜期建立；旧 pending 截止和历史样本证据不改。`verification/initial_static_identity_completion/final.xml` 为 **251 passed、0 skipped、0 failures，32.80 秒**，其中新专项 32 项：六种消息排列、真实 gen20 整数时间、三个独立截止边界、晚首凭证两种回调序列与不可复活、手动/配置/选择/epoch/out-and-back 失效，以及完成于旧参考 0.835 秒后实际进入原五帧融合、提交新 ticket、旧 ticket 继续取消的 B 集成正例。

`gen20_handoff_analysis/timely_completion_replay/result.json` 对原首失帧的内容回放仍通过原门：唯一候选 1、支撑 ratio 0.76、局部平面差约 0.488 mm、固定 acquired RGB IoU **0.952319379**。这仅证明原像素内容；新增生命周期测试使用受控时钟和真实 producer/consumer 方法，不是实机时延或执行通过证明。双时钟结果 `verification/initial_static_identity_completion/result.json` SHA 为 `8a63b14cbc385a02b6d0a1cd146490aff6355b4e5695700a76aa146ba06628b8`。本节写入时只在磁盘冻结，未加载；补视后当前位置接触策略仍由并行任务单独修改与验证，不能把本节写成抓取成功。

## FK 首次严格检查与补视后当前位置接触：三项修复合并冻结

观察排序窄修仅在普通独立补观察且最高支撑优先级内存在 FK 提案时，让首个 FK 先进入原完整 strict、路径可见性、跟随误差与当前 ticket 提交链，再决定是否进行批量提示计算。只有同一比较状态在真实完整准入后建立合格候选，才停止无谓提示；失败则恢复原有有界提示及族间排序，剔除已经检查的首项。原提示 3 秒/96 项、strict 3 秒/6 项、首合格后 8 秒窗口、所有几何与执行门不变；其他 mode、没有同层合格 FK 或已有 incumbent 保持原路径。`verification/fk_first_strict/final.xml` 为 **67 passed / 4.103 秒**，真实 base 循环被执行，严格服务和部分硬件结果使用离线夹具；没有把排序入口到 strict 的计时称为整请求或实机速度证明。

按用户新要求，`unknown_observe_return_return_to_origin` 默认 **false**：观察到位、三帧确认和同源注册完成后，默认从当前机械臂位置规划并继续原四阶段接触。显式严格布尔 `true` 保留原测得返回流程。新增内部 contact 审核绑定任务实际发布的 `NearFieldPlanningPhase`、当前仍活动的 phase/deadline、remote 注册 phase/source、相同 session/view/target/selection，以及双方 qualified 测量凭证的 digest 和真实三源；再审核原完整接触几何、MoveIt 与四阶段序列。它不允许外部 runner 将 `observation_contact` 当作新任务入口。初始接触入口不变，其他缺面仍须原补观察；此修改没有替代新鲜源、实际关节起点、冻结和接触碰撞门。

当前位置策略相关回归去重 **374 passed**：新专项 46、原 remote 59、原 runner/task 262、独立 base 执行 7。包括真实第二补观察 session 继续当前接触的正例、全部必要绑定反例、原返回路径显式 `true` 回归，以及实际 base 的预览/冻结/提交/实测端点/pregrasp→approach→grasp→close 顺序；后者使用无硬件运输夹具，不能当作实物动作。任务 phase 失效、旧源、目标变化、内容被改、规划中替换或未到达端点均拒绝。报告为 `verification/observed_contact_current_pose/result.json` 与 `verification/observed_contact_base_execution/result.json`。

三项修复共 **692 个不重复测试通过、0 skipped、0 failures、0 errors**；最终同进程新专项交互 **87 passed / 6.29 秒**已包含其中。交互后仅新增第二视角测试并单独通过，生产源码未变。最终清单 `verification/gen20_completion_fk_current_contact/deployment_sources.json` SHA **`08363035543b38108d406d804dfef508b24dc3f2dd48743d5439415359d28f0d`** 冻结 **66 项**：已部署 59 项中 47 项逐位不变、12 项预期修改，新增纳入 7 项（包括配置和测试）。各补丁独立审查与主代理复核通过；清单生成结束再次核对全部 SHA。此处仍记录 `REVIEWED_ONDISK_NOT_DEPLOYED`，实际部署与新一轮结果应以后续回执追加，实物夹持成功尚未验证。本次没有修改运行状态指针或发出 ROS、运动、力矩命令。

`completion_fk_current_contact_deployment/result.json` 随后已实际记录 `deployment_passed=true`，unit 为 `alicia-observation-measurement-1791537695784827656`；部署回执的 66 源 SHA 与上述冻结清单逐项完全一致。unknown/remote/task PID **148571/148572/148573 → 160318/160319/160320**，新父进程 160267；八个核心进程的 PID、启动标记和父进程均保持。部署本身没有发送运动或力矩命令。上段“尚未部署”是冻结时状态；本次部署通过不代表新抓取已开始或成功，实际新 generation 与结果另记。

## 09:21:58 启动包装提前退出：尚未创建 generation 21

记录 `grasp_records/20261009T092158.545067Z_c9514f768ec947afa5d546bb2be861e9` 的启动包装在新 selection 之前发现实际关节相对旧 **054722** 对准 preflight 超过原 **0.02 rad** 限制，`runner.log` 明确为该比较断言失败，返回码 1。记录没有 `authorization.json`，未提交新抓取任务或运动，也没有创建 generation 21；当前选择仍为 generation 20。recorder 于 **09:23:19.230681 UTC** 封存 `complete/unknown`、`recorder_closed=true`，原因为没有实际任务结果。它不是已经执行的新抓取失败，更不能作为三项修复的实机通过证据。

`prestart_alignment_changed_20261009T092158Z.json` 保存随后真实 `sdk_measured` 源 `1791537797372522875 ns`：相对旧对准 Joint1 **+0.184078**、Joint2 **−0.110447**、Joint5 **+0.101243**、Joint6 **−0.299126 rad**；因此不能继续沿用旧姿态确认。回执状态为 `IDLE`、`active=false`、`automatic_start_submitted=false`，GUI 手动控制已恢复。执行代理已询问用户当前新位置是否对准，本节记录时尚待回复；没有读取温度或发送关节失能命令。后续须依据新的实际确认与新记录追加，本次只更新文档和外盘镜像，不修改运行指针。

## Generation 21：首个补观察实际到位，RGB 碎片支持导致确认失败

用户随后实际确认“已对准”，回执为 `realigned_20261009T092534Z/alignment_receipt.json`。新记录 `grasp_records/20261009T092610.296727Z_b426d48a777045fba4e8e57580a46c67/authorization.json` 绑定该确认，创建 generation **21**、selection stamp `1791537972005057096 ns`，没有复用旧计划。请求 2 使用五张真实来源融合；原排序第 130 位的 FK 成为第一个 strict 检查项，并通过完整路径、支撑、可见性及跟随误差门。计划 `7375bbc87731610ca472c96c` 在 `1791537982.212314` 实际开始观察运动，随后测得静止到位。排序入口到原 strict 调用为 **0.635456 秒**，不是整请求耗时；完整请求约 **6.553 秒**。证据为 `gen21_handoff_analysis/planning_analysis.json`。

到位重核窗口从 `1791538008336143016` 到 `1791538011336143016 ns`，仍为原三秒。双端请求 accepted 正常，但 11 次测量中出现 **7 次搜索边界拒绝、3 次单深度种子面积支持拒绝、1 次只确认到 1/3**，未形成 qualified。当前源到状态记录年龄约 **0.259–0.358 秒**，不是旧参考 0.8 秒过期。本轮于 **09:26:55.458990 UTC** 封存 `complete/failure`，实际结果为 `OBSERVATION_MEASUREMENT_FAILED: OBSERVATION_MEASUREMENT_ACK_TIMEOUT`。没有进入后续注册、当前位置接触交接或闭爪；不能把观察运动通过写成实物抓取成功。状态与原始 ACK 见 `gen21_handoff_analysis/arrival_and_terminal.json`。

对确实在线审核的源 **`1791538008672485828 ns`** 做原算法同源跟踪：唯一匹配的深度种子只有 **2548 像素**，bbox 为 `[162,331,231,417]`，原可信硬核 214 像素。48 像素搜索后再向左扩到 64，最终 ROI 仍从 x=98 开始，选中 12671 像素的轮廓仍触左边界；真实白盒左缘约 x=90，算法裁剪切到了盒子，并非相机画面边界。即使忽略该边界，完整轮廓除以单个深度碎片也达到 **4.972920**，超过原 4 倍门。选中轮廓内其实有 **9158 个当前有效前景深度像素**，分散在多个连通片；左半内区有 4079 个有效前景点，却没有进入所选单片种子。原四连通分量大小约为 3183、2007、1310、1002 等，8 mm 深度边划分前后大块未变，证据指向缺深度孔洞导致的碎片化。手工图内区域只用于诊断计数，没有送入生产算法。证据为 `rgb_attempt_trace/result.json`、`depth_fragment_provenance.json` 及叠加图。

旧修复确已加载：部署回执、66 源清单与磁盘 RGB 源都为 `964ce516…`。gen6 的 48 像素搜索曾覆盖完整盒子，其面积比约 2.097；gen12 的当前深度 probable-foreground 修复处理了分片被训练成背景的问题，面积比约 2.735。gen21 的单片种子更小且偏向盒子一侧，触发了此前修复未覆盖的空间范围和支持分母问题。四轮相机内参、653×489 分辨率与深度比例相同，实测到位 q 不同；不能据此泛称相机参数改变或旧补丁回退。独立历史对照见 `gen21_handoff_analysis/historical_rgb_scope_review.json`。

## 到位空间提案与当前多碎片支持：127 项测试、实录回放及部署

本次只在到位确认及已经合格的同站 marker 路径传入固定实测 anchor 的足迹、实测高度与参考支撑面。它们投影到当前相机后只提出 RGB 搜索范围，并包含原第一张同帧 16 像素搜索的可信硬核；不提供历史像素或虚构深度。面积支持分母改为选中当前轮廓内，满足原深度/高度条件、且真实 XYZ 落在固定足迹零膨胀平面域中的当前测量点。旧 `rgb_to_depth_seed_area_ratio` 仍如实保存，另记当前支持点数和新比例；**4 倍数值门不变**。无效深度、支撑平面点和域外邻物不能增加分母。原硬前景、实测支撑硬背景、全部 RGB 可见、连通分量、0.85 核保留、位置/尺寸、颜色/轮廓、三帧确认与时效/反馈门保留。

到位后三条 normal、support-gap 和唯一 metric-recovery 接线共享同一冻结空间参考，原站位前后及提交前验证保持。多候选 metric ambiguity 只沿用原非授权 RGB 身份保持流程，不用目标空间域代替逐候选检查；只有原 3D 关联恢复唯一后才启用空间参考。默认初始、直接抓取和没有到位资格的路径仍传 `None`，保留原搜索与单种子面积判断。

最终 **127 passed、0 skipped、0 failures、0 errors，12.90 秒**，见 `verification/arrival_spatial_measured_support/final.xml`；唯一 warning 来自专门 NaN 拒绝夹具的 NumPy 比较。新负例验证域外同高邻物不增分母、缺深度不能由历史足迹补齐、0/2 个原匹配在 RGB 前拒绝、默认路径不变，以及多候选保持后唯一恢复与固定 marker 连续传递。面积隔离负例控制 GrabCut 输出，不声称覆盖所有邻物图像。首轮五个失败是多候选保持分支不应应用单目标空间域的真实接线问题，已按上述分层修复，原断言未删除。

实际生产分割和 `ObservationArrivalConfirmation` 回放中，gen21 保存的 **13/13** RGB-D 内容门通过，第三源 `1791538008904990196 ns` 完成原三帧确认；其中只有四个源与线上实际审核源严格重合。旧 gen12 的 **12/12** 可用源也通过，第三源仍为 `1791520145744392156 ns`，原 anchor/plane 与最后 ready mask 逐 bit 复原。两份报告为 `gen21_handoff_analysis/spatial_production_replay/result.json`、`gen12_arrival_analysis/spatial_measured_support_replay/result.json`。它们使用真实原始像素、当前深度和 TF，但没有重建缺失的线上帧，也没有证明实时 callback、截止、关节反馈或执行授权；不能用离线通过代替实物验证。

独审通过的 **71 源**清单为 `verification/arrival_spatial_measured_support/deployment_sources.json`，SHA **`13e61a649253c8c14c33b84a4420273acb23b449d63e8a4ece1716dd1fc792f4`**。原 66 项中仅 RGB、arrival helper、unknown 三个生产文件预期修改，另外 63 项逐 SHA 保持；新增两个专项测试和三个必要回放/跟踪源码。最终生产 SHA 分别为 `6fcec665…`、`49120330…`、`88bf1e44…`。

`arrival_spatial_measured_support_deployment/result.json` 已实际记录 `deployment_passed=true`，unit 为 `alicia-observation-measurement-1791539619805061560`；71 源与冻结清单一致。unknown/remote/task PID **160318/160319/160320 → 170117/170118/170119**，新父进程 170067；八个保留核心进程的 PID、启动标记与父进程均保持，部署本身未发运动或力矩命令。本节只记录部署完成，尚不声明下一轮抓取或实物夹持成功；新的任务与结果以实际授权和记录另记。运行指针继续由执行代理维护。

## Generation 22：到位后在 RGB 前被原单片几何身份门拒绝

新记录 `grasp_records/20261009T095400.350634Z_f3717a52509b4eb4a1e6354512f1b17d` 实际完成首个观察运动并测得静止。原三秒到位窗口为 `1791539674605140447–1791539677605140447 ns`，双端 accepted ACK 约在 3.0/4.3 ms 后到达。18 次在线审核均返回 `arrival_unique_original_identity_required`，当前源到状态记录约 **0.123–0.204 秒**，到位关节峰峰值为零；它们在 RGB 精修前被拒，不能据缺少新 RGB 指标判断上一补丁未加载。记录于 **09:54:41.779870 UTC** 封存 `complete/failure`，最终为 `OBSERVATION_MEASUREMENT_FAILED: OBSERVATION_MEASUREMENT_ACK_TIMEOUT`；未进入注册、接触或闭爪。

保存的 12 对 RGB-D 中，5 个源与线上审核严格相同。原算法逐帧重建后，这 12 帧的匹配数全部为 **0**；每个深度碎片的短轴比最大仅 **0.4066–0.4237**，达不到固定 anchor 原 **0.5** 下限。不能从错误名称把这轮误判成存在多个合格目标，也不能假称未录到的另外 13 个在线源已经逐帧回放。冻结 last-good 应取实际发布的 **`1791539649611108303 ns`**：其采集早于 executing 请求，但 ready 回执晚于请求、早于 unknown ACK；单用回执早于请求过滤会错选更旧参考。原 mask 已逐 bit 核验。事实和来源清单见 `gen22_handoff_analysis/diagnosis_summary.json`、`raw_matching/` 与 `source_images/manifest.json`。

## 仅零匹配时的当前碎片联合提案与最终所有权审核（磁盘冻结）

新增 `arrival_fragment_group.py` 只在已经绑定的到位确认，或合法固定到位站位的当前帧中使用；原匹配数必须为零，且至少有两个原生候选。原生候选 0/1、已有一个或多个合格匹配、没有空间参考时原对象返回，继续旧等待或拒绝流程。联合提案只包含本帧有效深度、原高度范围、固定实测足迹零膨胀域内的点；按原中位数、SVD 尺寸、支撑质量与原身份门重新计算，不将历史足迹填成几何，不降低 0.5 尺寸门。

提案必须再通过实际当前 RGB 精修。每个含原可信硬核的成员，分别至少 **0.85** 的硬核必须归属于同一个最终 RGB 连通区域，不能用大成员掩盖小旁物。无可信核的边缘片只作软证据；RGB 剔除的点不能留在最终几何中。最终原身份匹配、位置、尺寸、足迹、中位高度和质量均针对最终 RGB 内真实有效深度；tracker 的 raw mask 保留实际缺深度孔洞，没有人为连通。原颜色/轮廓及三帧一致性继续审核。固定域不是目标身份或运动授权，相关负例也不声称能区分任意不可观测的贴靠同色物体。

到位后三条正常/支撑恢复/几何歧义恢复路径使用同一固定空间与到位前外观参考，不随普通帧滚动。新正常分支暂存 tracker，只有发布锁内再次通过当前源新鲜度、固定 q、epoch、来源与 session 检查后才提交；处理或等锁时失效，原 last-good 不更新。两恢复分支只对本次调用返回的同一 `pre_refined` 对、唯一候选对象和有效到位 marker 使用适配：原完整轮廓身份函数读取已审核的最终 RGB，明确记录 `native_role=final_current_rgb_instance`；tracker 几何仍用真实测量 raw。`current_ring_fit` 在该受限分支还要通过原支撑平面一致性函数、原质量/足迹/时效/关节门。仅复制诊断字段、换候选对象或没有 marker 均不能取得例外。

最终测试按 `(classname, name)` 合并为 **177 个不重复通过、0 skipped、0 failures、0 errors**：`verification/arrival_fragment_group/final.xml` 中其余 167 项通过，加 `final2.xml` 最终专项 10 项通过（3.639 秒）。首轮四个新负例错误要求失效后旧 hold 引用仍保留；生产已按原规则撤销引用且没有提交新测量，只修正这些断言，保留具体拒因、last/last-good 不更新及所有新发布均 False 的检查，未改生产来保留失效权限。`effective_test_results.json` 保存逐例覆盖关系，不能把 173 与 10 简单相加。新正链实际运行三帧到位确认、qualified 释放、后续三帧 normal、两种恢复及原默认初始路径；节点传输和时钟仍为离线夹具。

实际生产 helper 回放 `gen22_handoff_analysis/fragment_group_frozen_replay/result.json` 为 **12/12** 内容门通过，第三源 **`1791539675217155694 ns`** 完成原确认，记录 helper、arrival、RGB 前后 SHA，最后参考 mask 逐 bit 一致。旧 gen21 **13/13**、旧 gen12 **12/12** 也在各自新的 `fragment_group_regression_replay/result.json` 中通过，没有覆盖前一版历史证据。两个旧回放脚本只增加可选 `--output`；移除输出参数改动后逐 bit 等于 71 源基线。回放未重建缺失在线帧，不验证实时截止、消息汇合、关节或硬件执行；尚无实物抓取成功结论。

最终 **75 源**清单 `verification/arrival_fragment_group/deployment_sources.json` SHA 为 **`0f0f5a991464d14571f21dfdb1ad1ffbe9370a7fd57422560eb97db862b1ded3`**：原 71 项中 67 项逐 SHA 保持，4 项预期修改（unknown、arrival helper、两个输出 CLI），新增联合提案 helper、两个专项测试和 gen22 回放源码。三项生产 SHA 为 helper **`4b3918dc…`**、arrival **`193c4496…`**、unknown **`d98763c4…`**。独审与主代理复核通过；本段仅记录磁盘冻结，部署与下一轮结果以后续实际回执追加。最新用户对准已单独保存在 `realigned_20261009T103042Z/alignment_receipt.json`，运行指针由执行代理维护，本次未改指针或调用 ROS。


## 当前碎片联合提案已部署；Generation 23 首到位确认通过、第二到位 RGB 有效域拒绝

`arrival_fragment_group_deployment/result.json` 已实际记录 `deployment_passed=true`，加载的 75 源与上述 `0f0f5a…` 清单逐 SHA 一致。新 unit 为 `alicia-observation-measurement-1791542722074667084`；unknown/remote/task PID **170117/170118/170119 → 179741/179742/179743**，新父进程 179691。八个保留核心进程的 PID、启动标记和父进程完全保持。部署本身未发运动或力矩命令，未读取温度；本次只重载三个计算节点。

第 23 轮记录 `grasp_records/20261009T104542.972841Z_73cf790625184b9e83c9c15be18db8f9/authorization.json` 绑定用户实际确认 `call_2f60f90d414742f487f8666d2ca83b3a:0` 及 `realigned_20261009T103042Z`，创建 generation **23**、selection stamp **1791542744395435333 ns**。第一个观察运动完成后，原三帧源 `1791542779489284753 / 1791542779822555303 / 1791542780089572429 ns` 实际完成 qualified，双端均确认同一组三源，约 **0.910 秒**内取得 remote qualified ACK；随后普通跟踪持续，并进入融合与近场规划。该事实证明本轮首次到位确认及后续规划交接实际通过，不等于已经执行接触或抓住物体。

本轮又实际执行了第二个观察运动。第二到位窗口为 `1791542793123099088–1791542796123099088 ns`，**9/9** 在线审核返回 `rgb_search_contains_unobserved_rgb`，未取得三帧 qualified，于 **10:46:40.297034 UTC** 封存 `complete/failure`，最终 `OBSERVATION_MEASUREMENT_FAILED: OBSERVATION_MEASUREMENT_ACK_TIMEOUT`。原错误指向 RGB 搜索范围包含未观测像素，具体是目标本身还是搜索背景边距须按同源图像与有效域继续核实，不能直接改写成相机画面缺失或旧补丁回退。原时间线、每次状态和 ACK 见 `gen23_handoff_analysis/arrival_and_terminal.json`。本轮没有实物抓取成功证明；运行指针由执行代理维护，本次只追加文档与镜像。


## Generation 23 同源诊断与仅背景有效域裁剪（79 源冻结）

对实际线上审核源 `1791542793794719457 ns` 跟踪原函数，第一张可信核搜索 `[40,300,226,452]` 全部 RGB 有效、可信核 980 像素；冻结实测足迹与高度的完整投影为 `[13,310,218,448]`。后续搜索加入背景后成为 `[0,294,234,464]`，其中 **359 个无效像素只在最左 0–2 列**，当前 seed 不含无效像素。另一实际线上源 `1791542794992742300 ns` 同样只在额外背景触域。12 张保存帧均在原 attempt 的全 RGB 有效检查处被拒，其中仅两帧与线上审核严格重合；最早线上失败未保存，不能声称已逐帧复现全部九次。真实第二观察前 last-good mask 逐 bit 一致。证据为 `gen23_handoff_analysis/rgb_valid_crop_trace/result.json`。

现有观察 FOV/路径检查按完整图像矩形与原 36 像素余量投影目标 OBB，没有认证分割背景 padding 的逐像素有效性。此次只在已经有到位空间参考、且期望搜索框包含无效背景时裁减额外背景：必须保留完整第一搜索框和完整实测投影，再在投影外保留原 **4 像素**硬背景/触边检测空间；这些必需区域出现无效像素则拒绝。算法在原期望范围内选取最大面积、坐标顺序确定的全有效矩形，每个像素再检查真实 rectification valid mask。本轮得到 `[3,294,234,464]`，没有将未观测黑像素训练成背景，没有改 first16 的有效检查或可信核，没有修改 RGB、深度或目标范围。裁剪后的轮廓若仍触边，原边界门继续拒绝；原颜色、面积、逐成员归属、最终真实几何、三帧、源/关节/站位及执行门全部保持。默认初始与 direct 路径不进入新裁剪。

最终 `verification/arrival_observed_background/final.xml` 为 **125 个不重复通过、0 skipped、0 failures、0 errors，9.00 秒**，已包含独立七项负例，不重复加数。覆盖多边/角落裁剪、最大矩形穷举对照、目标/原核/投影外 4 像素内无效必须拒绝、输入不改写、裁后触边仍拒及普通路径不变；另有独立 100 个小图与穷举结果逐值一致的纯算法核验，不能当作额外节点或实机测试。

最终生产实际回放 `gen23_handoff_analysis/observed_background_production_replay/result.json` 为 **12/12** 内容审核通过，第三源 **`1791542793794719457 ns`** 完成原三帧确认。旧 gen21 的 13 帧、gen22 的 12 帧、gen12 的 12 帧也分别通过其新 `observed_background_regression_replay/result.json`，各第三帧确认；所有报告绑定最终 RGB SHA **`872964d1…`**。这些回放包含原支撑、真实 RGB、逐成员所有权、最终几何、外观与确认函数，不证明实时 callback、截止或硬件执行，也没有重建缺失帧。

**79 源**冻结清单为 `verification/arrival_observed_background/deployment_sources.json`，SHA **`b58c63704265c4992c61ec21089df93567b1ba37f09440387ed14bd7c5b35a60`**。原 75 项中 74 项逐 SHA 保持，仅 RGB 一个生产文件修改；新增两个测试与真实回放脚本及其必需 trace 初始化依赖。独审与主代理复核通过，最新实际对准回执为 `realigned_20261009T105030Z`。本节记录磁盘冻结与离线验证，部署及下一实机结果待实际回执另记；运行指针保持由执行代理维护。


## 背景有效域裁剪已部署；Generation 24 已实际启动

`arrival_observed_background_deployment/result.json` 已记录 **`deployment_passed=true`**，实际加载 79 源与 `b58c6370…` 冻结清单逐 SHA 相同。新 unit 为 `alicia-observation-measurement-1791543629982592975`；unknown/remote/task PID **179741/179742/179743 → 184759/184760/184761**，新父进程 184709。八个核心进程的 PID、启动标记和父进程全部保持；部署本身未发送运动或力矩命令，也未读取温度。

第 24 轮记录 `grasp_records/20261009T110057.910067Z_c87184057f77415d9eac42bc852e8442/authorization.json` 已实际建立，绑定用户确认 **`call_e3f29964ef4f4fce952415037cd16f8e:0`** 与 `realigned_20261009T105030Z`，新 generation **24**、selection stamp **`1791543659504092931 ns`**，没有复用旧计划。本节核验时 recorder 状态为 `recording`、结果 `unknown`，只能记已启动；实物抓取、夹持及整任务成功均未确定。后续结果以本轮封存记录追加；本次没有编辑运行指针或执行 ROS 操作。

## Generation 24 封存：到位确认通过，首张正常帧处理跨过旧参考截止

第 24 轮只执行了一个观察运动，到位后三个真实源 `1791543686495713472 / 1791543686865828275 / 1791543687167207002 ns` 在 **1.040066004 秒**内完成 qualified，任务随后发出释放凭证。首张正常跟踪源 `1791543687661621093 ns` 在旧合格源之后 **0.494414091 秒**采集；开始处理时旧参考年龄 **0.530324221 秒**，约 0.300 秒处理后旧参考年龄成为 **0.828624487 秒**，但当前帧仍只有 **0.334210396 秒**。15 条到位后实测反馈各轴范围为零，epoch、selection 与语义相机配置保持，没有手动接管证据。原线上只记录泛化的 `observation_arrival_station_invalidated_during_recovery`；保存像素和原函数加受控相对时钟回放才复现内部 `source_exceeds_existing_frame_limit`，不能把该内部原因冒称原日志直接输出。

任务最终于 **11:01:39.757205 UTC** 封存 `complete/failure`，报 `OBSERVATION_CAMERA_RANGE_TIMEOUT`，八秒等待内没有再取得可用目标。未进入近场注册、第二观察或接触。`gen24_handoff_analysis/diagnosis_summary.json` 保存线上事实；`verification/arrival_frame_completion/baseline_clock_replay/result.json` 保存旧函数在旧参考年龄 0.79 秒通过、0.828624487 秒拒绝的对照。原实际当前像素通过原支撑、碎片归属、RGB 与最终几何门，第三帧参考 mask 逐 bit 一致；该内容回放与使用合成传输的真实 producer 时钟测试分别记录，没有冒称重新运行了未保存的在线三帧或完整硬件链。

## 仅到位站位正常帧的固定完成凭证（83 源冻结）

本次只修改 unknown producer：正常处理开始时已经通过原新鲜参考、固定到位 marker、来源和关节检查，且没有测量 lease、支撑 pending 或几何歧义 pending，才为这一张确切源建立内部完成凭证。凭证绑定同一个 reference、marker、lease、当前源和处理入口；固定完成截止为当前源原新鲜截止、旧参考两倍原 maximum 截止及 session 截止中的最早者，同时保留不可滚动的单调时钟截止。当前源在处理后和发布锁内仍须满足原至多 **0.8 秒**新鲜度，原 q、epoch、配置、anchor、平面、RGB、唯一几何与 session 检查全部保留。未及时进入、普通路径、初始站位、缺测/歧义恢复或已失效身份不能取得该凭证；缺测分支仍有原独立旧参考截止检查。

所有使用凭证的正常候选，包括原生单候选和碎片联合候选，均先暂存 tracker，发布锁内最后复核通过才更新 last、last-good 和当前几何。处理结束、新 lease 或进入原 pending 路径清理凭证；不重写源时间，不把旧几何续成新测量。frame timing 增加真实入口年龄、固定截止、是否跨旧参考截止及具体内部失效原因。remote/consumer 生产源码完全不变：其既有合同在未收到 False 时不会仅因普通旧 ready 超龄而退休 session；收到真正 False 仍撤销旧身份、session 与 surface，后续 True 不能复活旧权限。

最终去重 **150 passed、0 skipped、0 failures、0 errors**：producer 147 项（`final2.xml` 覆盖 `final.xml` 相同十项，其中一次旧诊断文字断言仅随新增具体原因修正），另有独立实际 consumer 回调三项。覆盖原生与碎片两种正常路径、处理/等锁超期、固定 q 往返、source/reference/lease/session 变更、旧初始双时钟回归，以及到位 qualified 后跨旧参考截止的实际 MRO 回调、原过期当前源不得进入同步样本、真实 False 后旧身份不可复活。独立 consumer 的硬件运输与 surface 为夹具；它保留原 consumer 源门，并不宣称 consumer 新增了 0.8 秒门。结果见 `verification/arrival_frame_completion/independent_review.json` 与 `verification/arrival_normal_completion_consumer/result.json`。

最终 **83 源**清单 `verification/arrival_frame_completion/deployment_sources.json` SHA 为 **`07fc7bbfff473e7e58556b59faed4ea6c03ea66c064b34e107831ef7bc24404a`**：原 79 项中 77 项完全不变，仅 unknown 生产源码与一项旧测试断言更新，新增三个测试和一个回放脚本。生产 SHA 为 **`2d5cf7eca3d8b187e27e0baec6a7ee4705a07a8dc2711a8c813aaa14b625e678`**。本节写入时为磁盘冻结、独审通过，实际部署须以后续回执为准，实物成功仍未验证。最新用户实际确认是 **`call_2c5400a1db1043da83d5d400345fcd23:0`**，保存在 `realigned_20261009T111320Z/alignment_receipt.json`，没有沿用 105030 姿态；运行指针由执行代理维护。

## 正常帧完成凭证已部署；Generation 25 已实际启动

`arrival_frame_completion_deployment/result.json` 已实际记录 **`deployment_passed=true`**，加载的 83 源与上述清单逐 SHA 相同。新 unit 为 `alicia-observation-measurement-1791544590714290230`；unknown/remote/task PID **184759/184760/184761 → 190308/190309/190310**，新父进程 190256。八个核心进程的 PID、启动标记和父进程完全保持；部署本身没有发送运动或力矩命令，也没有查询温度。

第 25 轮 `grasp_records/20261009T111644.245219Z_4761f38faa0749de93c9994d1dd07faf/authorization.json` 已实际建立，绑定新的 **111320Z** 对准回执与 `call_2c5400a1db1043da83d5d400345fcd23:0`，创建 generation **25**、selection stamp **`1791544605804177045 ns`**，未复用旧计划。本节仅确认已部署并启动新轮，物理夹持与任务成功仍未验证；封存结果另行追加。本次只维护文档及外盘镜像，没有修改运行指针或调用 ROS。

## Generation 25 封存：前两站持续跟踪，第三站完整必需区域超出 RGB 有效域

第 25 轮于 **11:18:00.195892 UTC** 封存 `complete/failure`，最终为 `OBSERVATION_MEASUREMENT_FAILED: OBSERVATION_MEASUREMENT_ACK_TIMEOUT`，实际执行了三个观察运动，尚未执行接触。第一、第二站分别约 **1.014 秒、1.569 秒**完成原三帧 qualified，随后有 **24、23** 次正常 ready。新正常帧完成凭证在两站分别 **2、21 次**实际跨旧参考截止完成，共 23 次；当前源和原测量门仍通过。因此上一修复已在线发挥作用，本轮首因不是相同的旧参考截止故障。第三站 **8/8** 在线审核报 `rgb_required_spatial_region_unobserved`。保存的 13 帧只有第七次在线源 **`1791544674989701986 ns`** 精确重合，不将其余未保存审核声称为逐帧重现。证据见 `gen25_handoff_analysis/diagnosis_summary.json` 与 `rgb_required_region_trace/result.json`。

该精确源的 first16 搜索 `[40,298,201,458]` 全部有效，但固定 anchor 足迹加实测高度的完整投影为 **`[-11,297,203,445]`**，必需范围 `[-15,293,207,458]` 已含 **343** 个无效像素。这次不是可裁掉的额外背景，不能沿上轮背景裁剪办法移除必需区域。名义规划只检查当前 prepared OBB：第三计划的 OBB 左边距为 **40.000 像素**，同一名义相机下原固定 anchor prism 左边距却只有 **6.535116 像素**，两种几何范围相差 **33.464884 像素**；实际到位 TF 又令该 prism 左移 **17.116172 像素**，成为 **−10.581056 像素**。到位时同一个 prepared OBB 自身也只剩 **25.777722 像素**。这些值来自原源、原角点摘要一致性和记录 TF，不是推测膨胀系数。名义与实际相机位置差约 **14.962 mm**、旋转差 **2.378°**；原观察分支记录实际误差，并不应用接触阶段的 6 mm 拒绝门。原跟随支撑盒也不认证相机视野。完整计算见 `gen25_handoff_analysis/planning_contract_review.json`。

连续补观察不是零信息重复：前两次注册后新增有效体素分别为 **19、4**，原最少 3 个体素门均通过；但随后两次接触搜索各有 12 个 learned 候选、**0 个 locally valid**，tabletop 25/26 个提案也均未形成接触计划。双侧点数充足不代表两侧在原支撑法向及插入轴上具有可用的共同接触高度，第二次 tabletop 实测最大 CAD 重合仅约 **0.534 mm**，低于原 **2 mm**。现有 FK 提议最小化关节位移、首个完整合格后可停止继续比较，能取得刚过门的增量，并不保证最大信息收益或一次补看就能夹持。当前证据不支持降低接触门，也不能写成碰撞或接触问题已解决。

## 规划与到位使用同一冻结实测 prism 的补充检查（验证中，未部署）

新补充合同从同一张真实 ready 状态复制固定 anchor 的足迹、平面与原 `median_height_m`，并绑定 exact RGB-D 源、selection、现有 reference 和相机 rectification 有效域。生产只新增只读高度字段；没有使用历史像素填充当前几何。普通补观察需要这些字段，缺字段或来源不符拒绝。新 prism 与原 prepared OBB 并行检查，沿原 **36 像素**边距、原深度范围及实际 RGB valid bitmap；FK 优化增加同一 prism 约束，其他候选提前筛除，最终原完整路径审核之后还按严格 IK 终点再检查 prism。原 OBB 全路径、支撑、碰撞与跟随门不改；新 prism 审核明确仅为终点预测，不要求旧起点也通过新终点条件。

独立 **45 项**回归已通过，覆盖真实 producer 的高度来源、source/selection 绑定、缺字段、矩形内部无效孔洞、派生积分图摘要、不可放宽的原边距/深度和两类缓存。首轮积分图错零反例实际失败，随后入口增加不可变数组摘要验证，该原负例复测通过；不是更改断言掩盖漏洞。原 endpoint 缓存的写入端也补齐新的合同参数，缓存签名变化仍不重置 FK 原共享两秒预算。主组合与最终源码冻结另以实际报告追加，本段不表示已经部署。

同源 9 个投影回放中，第一、第二名义提议及严格 IK 终点均通过，第三两者均拒绝；前两名义 prism 最小边距分别约 **40.869、36.334 像素**。第二实际到位位姿低于新 36 像素，进一步表明该门不能认证任意跟随偏差。旧记录没有新高度 metadata，回放明确从同一原 acquired 几何补入标量，仅验证数值合同，不冒称旧状态具有新权限。原到位当前 RGB、支撑、唯一身份及三帧门继续必须执行。证据见 `verification/observation_arrival_projection/gen25_gate_replay/result.json` 与 `independent_final.xml`。

最新用户真实确认 **`call_3ec7b5f944a94d9e83db4337bdddaf0e:0`** 已保存为 `realigned_20261009T112422Z/alignment_receipt.json`，未复用旧姿态；本节写入时尚未消费该确认开启新轮。本次只更新源码验证说明和文档镜像，未改变运行指针、执行 ROS 或声称实物成功。

上述补丁已完成最终冻结：`verification/observation_arrival_projection/deployment_sources.json` 共 **89 源**，SHA **`eb2ca702f1a5a9fd2d8c7f4c81b4edf43cb4b725113c044de50afba217e1e83a`**。83 源基线中 79 项逐 SHA 保持，仅 unknown 状态 metadata、普通观察接线、冻结来源/严格终点审核和 FK helper 四项生产更新；新增跟踪 pure projection helper、三个测试文件（其中一个是原先未纳清单的既有 source fixture）及两个回放脚本。`passed/reviewed/frozen=true`，此处仍只表示可部署清单，实际加载以部署回执为准。

最终 **237 项去重通过，0 failed、0 error、0 skipped**：主组 192 项由最终 producer 67 项覆盖同名病例，再加独立 45 项，不能相加成 304。主组首次两个失败是非 ready 的诊断 anchor 夹具没有 `metrics`；只读状态发布改为缺失时给 `None`，原缺高度的观察 capture 仍拒绝，原断言未修改。最终 producer 67 项通过并保留前次失败 XML。另一个 source fixture 暴露冻结 MappingProxy 不能直接 deepcopy，绑定处改用项目既有 thaw 副本函数，保留数值和隔离。逐例来源与覆盖关系保存在 `effective_test_results.json`，精确差异见 `changes.patch`。

最终 req19 同源 FK 重算恢复了 **2 个**端点提案，约 **0.241 秒**，实测支撑点 **100664** 个逐 bit 对应原源。新 prism 求解约束复用原 **4 像素 proposal reserve**，两提案边距约 40 像素，最终硬验收仍为原 36 像素；原两秒、两个方向、96 次迭代预算不变。第一次不加 reserve 的 36 像素边界提案报告保留，不作为最终证据。最终 helper 的九个旧端点也另存重放为 `gen25_gate_final_replay/result.json`，没有覆盖早版。两项真实源回放仍仅证明原内容及端点门，不证明新 MoveIt 完整全臂路径、实际到位或实物抓取成功。

## 同一冻结 prism 检查已部署；Generation 26 前三站已实际通过到位确认

`arrival_projection_deployment/result.json` 已实际记录 **`deployment_passed=true`**，回执 SHA 为 **`e87516e66138e438be830a35d2e211fa01082b010dc2dfb656f47cf6bd7a0dd6`**，89 个已加载文件与上述 `eb2ca702…` 清单完全一致。新 unit 为 `alicia-observation-measurement-1791546361041077087`；unknown/remote/task PID **190308/190309/190310 → 199503/199504/199505**，新父进程 199452。八个核心进程的 PID、启动标记和父进程全部保持，部署未发运动或力矩命令，也未查询温度。冻结清单中的 `deployed=false` 是部署前工件状态，保留原件；实际是否加载以本回执为准，不能由历史字段反推部署失败。

第 26 轮 `grasp_records/20261009T114637.653111Z_26d52c38592844ad8ff668163cae0f90/authorization.json` 已消费真实 **112422Z** 对准确认 `call_3ec7b5f944a94d9e83db4337bdddaf0e:0`，创建 generation **26**、selection stamp **`1791546399401629209 ns`**，没有复用旧计划。前三个观察运动均实际完成，unknown 与 remote 均确认各自同一组三个真实源，任务随后完成 qualified 释放并取得正常跟踪/近场阶段交接。

第三站的确认源为 **`1791546471263749837 / 1791546471567427396 / 1791546471801668167 ns`**；两端在 **`1791546472.185002 s`** 回报同源 qualified，随后完成释放 ACK。任务在 **`1791546472.699610 s`** 记录 `PLAN_PREGRASP supplemental observation sampled; requiring measured registration and fresh contact evidence`，因此已实际跨过上一轮第三站必需 RGB 域失败的位置。此处证明三站到位与后续交接，不等于第三站接触候选或完整接触轨迹已经通过。

本节读取时 recorder 仍为 `recording/result=unknown`，尚无闭爪或实物夹持成功证据；后续接触和封存结果由本轮实际日志追加。阶段事实来自 `arrival_projection_deployment/consumers.log`，没有把离线 237 项测试、部署成功或到位 qualified 写成整任务成功。本次只维护文档与外盘镜像，运行指针仍由执行代理管理。

## Generation 26 封存：三站到位通过，接触搜索期间当前处理帧超龄

上述录制已于 **2026-10-09 11:48:56.328219 UTC** 封存，manifest 为 `complete`、`recorder_closed=true`、`result=failure`。第一、第二、第三站到位至 remote qualified ACK 的录包接收间隔分别为 **1.489833、1.402829、1.049978 秒**；这些是消息接收间隔，不冒充精确计算完成耗时。三站均取得相同三源的双端确认，任务完成正常跟踪与近场交接。第一站严格 IK 终点的新 prism 最小边距为 **40.161209 像素**，要求仍为 36 像素，必需背景无效 RGB 像素为零，证明新增合同确已进入本轮被执行计划的审计。

第三站后 request 28 继续搜索接触计划，日志记录 **26** 次 `CONTACT_SEQUENCE_PRECHECK_FAILED`：原连续几何检查在路径 `10-end` 或 `11-end` 判定目标 OBB 与左/右指体碰撞，包含 `nominal` 与 `predicted_command` 分支。请求随后被目标失效取消，没有完整终局候选审计或保存的失败轨迹；不能据这些逐项拒绝声称已穷尽全部接触候选，也不能把后续取消当成碰撞的原因。没有发生接触执行或闭爪。

首次永久失效发生于 **`1791546529862875223 ns`**，原因为 `perception_failure: source_frame_stale_or_future`，处理源为 **`1791546527959462165 ns`**，上一合格源为 **`1791546527447860002 ns`**。实际记录明确区分输入与处理时间：

| 阶段 | 原始记录 |
| --- | --- |
| 当前源到 exact RGB-D 回调 | 21.564 ms |
| 当前源开始处理时年龄 | 42.367 ms |
| 完成凭证入口当前源 / 旧参考年龄 | 51.282 / 562.884 ms |
| 主 exact TF 查询 / decode | 0.089 / 2.304 ms |
| `rgb_refinement` 复合段 wall / 当前线程 CPU | 1.709048 / 0.421339 s |
| 整帧处理 wall / 当前线程 CPU | 1.865925 / 0.562702 s |
| `post_refine` 首失时当前源年龄 | 1.903382 s |
| 同时已在等待槽中的最新源年龄 | 68.158 ms |

当前源在进入处理和建立固定完成凭证时均及时；凭证固定截止为 **`1791546528759462165 ns`**，没有续期，处理结束后原当前源 **0.8 秒**门正确拒绝。旧参考到当前采集的间隔为约 0.512 秒，这不是上一轮仅旧参考过期的故障，也没有证据支持继续增加完成期限。更新的 exact RGB-D 在处理期间持续到达，失败时并非只有陈旧相机输入。末段八秒内 **80** 条 SDK 实测反馈，六关节和夹爪位置范围均为零。

该线上 `rgb_refinement` 标签包括实际 `refine_candidate` **以及**随后 `_prepare_locked_rgb_hold_reference` 的来源缓存构建；后者还有 cached 参数上下文、静态相机安装 TF 核对、关节括号、外观特征和摘要计算。复合段 wall 与当前线程 CPU 相差约 **1.288 秒**，仅能定位大量时间不在当前线程执行，不能直接断定是 GrabCut、GIL、锁、缺页、IO 或调度中的哪一种。前 20 张 ready 帧总处理 wall 中位为 **0.338577 秒**、最大 **0.702340 秒**。相机参数是周期快照，保留原读取区间，不能把某份 temporal 状态冒充失败像素的逐帧 SDK 凭证。

## 精确两帧离线内容核验与当前证据边界

上一合格源与首失源的原 RGB-D、TF、安装变换、Object、mask、相机参数和真实 accepted 关节前后括号均已导出。上一合格帧发布 mask 为 **15816 像素**，原算法重放逐 bit 一致；失败帧在线发布 mask 为空，这是失效通知，不能把空 mask 当成算法精修结果。离线原门重放仍取得唯一候选，原 raw mask **2893 像素**精修为 **15783 像素**。五次失败帧原 `refine_candidate` 用时 **0.129834–0.145094 秒**，其中 GrabCut 为 **0.046392–0.054526 秒**。这证明保存像素可通过原内容门，未复现线上复合段 1.709 秒，也没有重放 ROS 缓存、安装 TF 等待或当时系统竞争。新的嵌套计时仅作为后续定位手段，本节没有其部署回执，不将其写成已加载。

接触诊断另从 15 份原始源重建七个 SurfaceView，保留原融合、注册及外包络函数。request 9 的点数、各视角分配、注册变换及线上融合 OBB 一致；request 19 各视角点数一致，注册变换最大数值差约 `9.76e-7`。request 28 的重建 OBB 与线上尺寸最大差 **0.215 微米**，中心和轴向一致；但精确坐标去重对浮点误差敏感，重建联合点数 **2943**，线上为 **2838**。该输入可用于明确标注的毫米级几何诊断，不能据此声称候选 ID、排序、完整点集或失败轨迹逐 bit 复现；没有手动修改点云来匹配结果。

本会话新增证据索引：

- `gen26_handoff_analysis/arrival_and_terminal.json`：三站协议与终止原字段，SHA `937b5c261520b440e118f968e6211938d5e37cc0f8706c758ebc3e92aa1d447c`。
- `gen26_handoff_analysis/diagnosis_summary.json`：首失复合计时、前 20 张正常帧、80 条静止关节统计及 26 次接触拒绝，SHA `09d29bbd0f5020c1afd92023804bbcddeaf2d526ea48bd75241cdc97bb1f68c8`。
- `gen26_handoff_analysis/stale_source_images/manifest.json`：两张确切源及各自 NPZ、相机和关节括号，SHA `4abe3dcf51de869e89e761270a0f92648f17def1f6e76de2b6b98bb925ad7d22`。
- `gen26_handoff_analysis/offline_refine_timing/result.json`：同像素五次原函数分段计时及前后源码摘要，SHA `fe069beac1c46b872d2a1797b1234311c3f34e686bf234ffdae5a743174e705f`。
- `gen26_contact_analysis/source_inputs/manifest.json` 与 `fused_reconstruction_final/result.json`：原视角输入及接触几何重建，后者 SHA `858adf75dfe384074e7bee6ce1d3ab27670a83fe479582b86ce7005f497b93ba`；初稿单帧几何与最终融合外包络分开保留。

## 115240Z 新真实对准：未消费

用户在本轮终止并恢复手动调整后，实际消息 **`call_f42cc82938ff47eeb43cc7e740567f17:0`** 确认“已对准，可以继续自动抓取”。新回执 `realigned_20261009T115240Z/alignment_receipt.json` 的采样时间为 **`1791546761402091741 ns`**，图像源 **`1791546761338797569 ns`**，SDK 实测源 **`1791546761370050545 ns`**，控制 epoch 保持 `1791506524580754980 ns`，`previous_alignment_reused=false`；回执 SHA 为 `43f4256f7f88baf7cd94af10822efd6bb98bcc57ba84a214d6a239e1e2ff08a4`。

截至本节写入，该新确认尚未用于创建下一自动抓取轮次；回执所见 selection 仍为上一轮 generation 26，不代表自动任务仍活动，也不能把新手动姿态用于解释 gen26 的原轨迹。后续须记录真实新部署、选择与实验授权。此文档更新和外盘镜像未修改运行指针、未执行 ROS 服务或运动，也未读取温度；实验继续 `local_only` 完整保留。

## Gen26 接触高度与当前偏置：有限端点反事实完成

在上述重建误差范围内，使用原工厂及完整视角来源 `view_sources` 生成 **32** 个候选，其中 **9** 个通过原接触高度放置。对这 9 个候选分别检查名义位置和原有两个高度提案，共 **27** 个端点对照。原四阶段显式几何函数、双侧高度证据及未改变的 **2.676768 mm** 注册高度余量均保留；27 项显式几何检查通过，但名义与偏置预测两个端点同时无指体/支撑碰撞的为 **0/27**。其中 12 项名义 OBB 碰撞、24 项预测命令 OBB 碰撞，19 项预测指体内包含重建的实际测量点；27 项预测关节值均在原关节限位内。这些计数不是完整全臂或 MoveIt 路径验证，也不表示所有可能高度、姿态和 IK 分支无解。

代表性重建候选 `source_index=31, variant_index=1` 的目标沿 jaw 投影为 **40.365 mm**，当前开口 **49.75 mm**；名义两指各有 **4.093 mm** SAT 分离间隙，因此不能把本例归为开口不足。采用记录时间 `1791546486.7 s` 之前最新 SDK 命令与 accepted 反馈之差，六关节偏置为编码量子 **`[+1,+13,+12,+5,-1,+3]`**；按原 `SDK−accepted` 正号预测 `FK(q+bias)` 后，左指与目标 OBB 的最大 SAT 分离值为 **−4.668 mm**，预测指体内有 **55** 个测量点。27 项预测工具位移沿支撑法向为 **+10.177–+10.463 mm**、沿各自 jaw 轴为 **−8.418–+8.423 mm**。这描述命令模型中的偏移及相交，不是实际未来末端位移、物理碰撞或已经发生的侵入。代表候选原两个高度改变量仅为 **+0.044/−0.341 mm**，原余量和提案不能消除此处预测碰撞。

独立静态核对确认：当前视角与 reference 点/法向/offset 的逆变换符号正确；目标点与指体 FK 在同一 base 坐标中，物理支撑面沿用当前请求平面；偏置符号与原 `compensate_cartesian_path` 的 `positions + SDK − accepted` 一致。原预发布检查将观察站位偏置外推到虚拟接触位姿，而实际执行到预抓位稳定后还会重新测量偏置，二者属于不同阶段。此有限回放不能证明观察站位偏置会在后续预抓位保持；现有合同也不允许因此跳过失败的预检。后续仅增加及时保存的失败审计和分段计时，接触算法、碰撞门与高度门不据本回放放宽。

结果与脚本均保持离线、非执行权限：`gen26_contact_analysis/contact_endpoint_summary.json` SHA **`145fbc4be5df7f8c0a5ad9db28300ac26fc4d935ba48ac4dffa91b4bba50706f`**，原 27 项数据 `contact_endpoint_counterfactual.json` SHA **`3a4131863c55a512f9f5e3fd23a055d2c50178ec75fc7c2d8d5a5fa229687682`**。SDK/accepted 样本不是原失败服务精确采用的那一对，独立有界 IK 不是被取消请求的原 MoveIt 轨迹；候选 ID/顺序及点云去重差异继续适用前节限制。本次只追加既有文档与镜像，不新增真实实验或改变运行指针。

## Generation 27 封存：无动作，初始正常帧在 TF 等待后跨旧参考截止

细分 RGB 复合段计时与紧凑接触拒绝审计已由 `rgb_contact_diagnostic_deployment/result.json` 确认加载成功，unit 为 `alicia-observation-measurement-1791547838421627428`；这更新前文“尚未部署”的历史状态。随后录制 `grasp_records/20261009T121108.702839Z_3a4d30c05d58450cbe43f4b3802a4b1d` 实际消费 115240Z 真实对准授权，建立 generation **27**，于 **12:12:53.095770 UTC** 封存 `complete/failure/recorder_closed=true`。runner 退出码 3，只有候选计算开始/停止，无 `GRASP_START`；录制内 controller command、轨迹 action goal、观察 measurement request 均为零。

首失前全部 **59** 条 `sdk_measured` 的六关节及夹爪值完全相同；anchor 后到首失 **45** 条六轴峰峰值均为零，没有超过固定站位 **0.004 rad** 门。首失前 SDK 命令 **52** 条和 control reference **53** 条各只有一组位置值。GUI 在 selection 前已经切为 false，控制 epoch 保持 **`1791506524580754980 ns`**。这些记录排除了本轮已录时段内真实漂移/用户手动移动的解释，不能因外层错误包含 station invalidated 就重新要求用户对准。

首失观测时间 **`1791547875165506839 ns`**，当前源 **`1791547874795775413 ns`**，旧合格源 **`1791547874349609136 ns`**，原始 anchor **`1791547870646894931 ns`**。当前采集距旧参考 **0.446166277 秒**；开始处理时当前源仅 **0.046657801 秒**、旧参考 **0.492824078 秒**。主 exact TF 查询耗时 **0.321372756 秒 wall / 0.001697981 秒线程 CPU**，随后分割前校验时旧参考达到 **0.815897703 秒**、当前源仍仅 **0.369731426 秒**。`failed_stage=segmentation` 指该阶段入口，实际分割与 RGB 精修尚未进行，此前没有 typed support pending 或 metric ambiguity pending。

原 `_initial_station_support_reference` 在 TF 后通过 `_qualify_support_gap_source` 再检查旧参考 **0.8 秒**寿命，缺少正常单帧完成凭证；异常被统一包装为 `initial_station_current_support_invalidated`。这是源码与原时序共同确定的失败链，旧状态没有直接保存内部 `invalidated_reason`，不能冒称取得了当时内部异常文本。TF 的 wall/线程 CPU 差异也不能独自区分 TF 尚未到达、锁等待或线程调度。证据为 `gen27_handoff_analysis/initial_station_analysis.json`，SHA **`99bce15f1a0055c818573d1d7d56c5ede25d928c80e5b42b019036adfd70781f`**；限定 topic 原字段另存 `whitelisted_metadata.json`。

## 七阶段时效规则与共用正常帧完成修复

独审同时发现原 arrival normal 完成凭证也在主 TF 之后建立，decode 内相机参数/CvBridge 等待也可能位于凭证之前。修复不再只针对某一站：在原当前源年龄、frame-id 和 RGB-D 尺寸检查之后、decode/TF 之前记录同一张帧的及时入口。此记录没有身份、几何或动作权限；各阶段原 source、selection、epoch、固定 q、相机/手眼配置、anchor/平面、支撑、RGB 和唯一几何门仍须完成，失败不得回退成使用旧平面的可发布帧。初始站位不伪造观察 lease。

以下 `M=min(configured max_age, 0.8 秒)`，`r` 为旧合格源、`s` 为当前源。所有 deadline 固定，真实失效不得被后来同代次的 True 复活。

| 阶段 | 入场与固定时间界限 | 可提交的测量/权限 |
| --- | --- | --- |
| 初始 acquire | 无旧参考；各当前源保持原新鲜，原三帧确认 | 三帧原门通过后首次建立 anchor、平面、站位与模板 |
| 初始 normal | 开始处理及入口时 r、s 都新鲜；完成截止 `min(s+M,r+2M)` | 本帧全部原门及发布锁内复核通过后才提交新参考 |
| 初始 typed support gap | 首 gap 在 `r+M` 前注册原绑定凭证；采集 `r<s<r+M`；完成不晚于 `r+2M` 且 s 仍新鲜 | 旧几何超龄即无权限；原支撑/唯一 metric/RGB 恢复后仅提交真当前源 |
| arrival measurement | 同 lease、真实到位静止窗、固定 capture/session 截止；连续三张实测合格 | 只发布第三张当前源；qualified 等 receipt 期间也不得超其原新鲜度 |
| arrival normal | 与 initial normal 同一规则，另取原 session 截止中的最早者 | 仍绑定原 lease/marker/站位及当前源，全部原门后才提交 |
| metric ambiguity | 保持原旧参考 M 寿命；不能借 normal 凭证 | 必须原唯一 metric 与独立 RGB 身份恢复，不滚动旧参考 |
| observation executing | 同 session/lease 冻结身份，受原固定 session deadline 限制 | 运动中不发布几何权限；到位后重新走 measurement 或撤销 |

支撑 gap、缺独立当前支撑、metric ambiguity 和 RGB-only hold 均清理 normal 凭证；已有 typed pending、acquire、普通/direct 或运动阶段不能借用它。每帧完成后清理，下一帧必须重新及时入场；当前源原 M 新鲜度和配置更严格时的较短上限不变。相同规则覆盖 decode、TF、上下文和 RGB 等待，但没有让这些处理更快，也不能处理已经过期的当前图像。原完整矩阵见 `gen27_handoff_analysis/cross_phase_lifetime_matrix.json`，SHA **`81c4deea482a35f92e3e65ace65e43f4c861123018d5137fa5f82f2f8f712479`**，其中明确区分修前缺口与目标规则。

最终 **233 passed、0 failed、0 skipped**：原相关组 202 项，加独立实际 producer 跨阶段 31 项。后者使用原 synthetic RGB-D/运输夹具与 gen24/gen27 的真实相对时钟，覆盖两个 normal 阶段、decode/TF/context/RGB 等待、当前源过期、相机/手动/关节往返/引用变更、开始处理已经太晚不得倒推资格，以及失效后不可复活；不冒称原像素实机重放。首轮保留 4 个旧负例失败：原 `+0.6 秒` 只令旧参考过期、当前源仍新鲜，在新明确合同下应允许完成；仅把这四个“expired”负例改为 `+0.81 秒` 使当前源确实过期，原手动/相机/关节断言不改。

最终生产 unknown SHA **`ad0c210501c6d360c0aaaff4c553092276ba2d6e44c9f06a2e5bd1586c49d027`**。合并结果 `verification/station_frame_completion/result.json` SHA **`ba03545ea682472ebff021d396a962c6a17eab29e61ee04eceaa2ff77283d53b`**；独立 31 项结果 `verification/cross_phase_frame_lifetime/result.json` SHA **`967c1eb04520d64b1dc771d4b76909977bff35ea75a7b6a1533511c47b2fec9d`**。两项结果均在运行前后核对源码不变。

## 共用修复 93 源已部署，Generation 28 沿无动作失败后的原授权启动

合并清单 `verification/station_frame_completion/deployment_sources.json` 共 **93 源**，SHA **`f7f35cddbc39290d5b49143fea5c0625480d8c7ec5c8c039dc875351896e7e81`**；92 源基线中 90 项保持，更新 unknown 与必要旧测试，新增独立跨阶段测试。`station_frame_completion_deployment/result.json` 实际确认 **`deployment_passed=true`**，SHA **`a44d817246160d62064e1548e8b273f3ec4fbc5be3f00b7d74ae91104a357aec`**，新 unit **`alicia-observation-measurement-1791548754629431667`**。八个核心进程的 PID、启动标记及父进程保持，部署没有发送运动、力矩命令或查询温度。清单中部署前保留的 `deployed=false` 不覆盖实际部署回执。

由于 gen27 未提交运动且 59 条真实反馈不变，执行代理保留用户实际消息 **`call_f42cc82938ff47eeb43cc7e740567f17:0`** 的既有授权，没有制造新确认。`realigned_20261009T115240Z/continuation_after_gen27_no_motion.json` SHA **`0f0a809e82f45a7a181f5a8c9b3f49a2d6d2c22849e315f04fcd619fe6ffa330`** 明确 `new_user_confirmation_received=false`、`old_plan_reused=false`，原 runner 仍须检查当前姿态匹配并重新获取目标和完整路径。

新录制 **`grasp_records/20261009T122642.289598Z_97e4fc00eabf4de2bbac5e2ba35ba119`** 已建立 generation **28**，selection stamp **`1791548803800344467 ns`**。本节仅依据授权、部署回执及已有封存证据记录启动，未读取正在写入的大 bag；尚不声明本轮观察、接触、闭爪或实物抓取成功。运行指针继续由执行代理维护，本文与现有外盘镜像只同步事实。

## Generation 28 封存：共用完成凭证实际生效，当前帧处理超龄仍未解决

上述录制已于 **2026-10-09 12:27:32.150061 UTC** 封存，manifest 为 **`complete/failure/recorder_closed=true`**，SHA **`22a28e6275d4bca4084f236538d897bed31f2bc80a023aa647f448d1f21d577b`**；runner 退出码 6。一次真实观察运动完成，到位时间为 **`1791548838887871980 ns`**，unknown/remote 确认的三个实际源为 **`1791548838966604709 / 1791548839267357349 / 1791548839501886129 ns`**。remote qualified ACK 的录包接收时间为 **`1791548839904399871 ns`**，到位至该 ACK 接收间隔 **1.016527891 秒**；该数值是录包消息间隔，不替代内部计算完成时间。任务随后进入正常近场交接，未执行接触、闭爪或实物夹持。

到位释放后有 **19** 张 `ready` 正常帧，均使用当前测量 fragment-group 路径；其中 **10** 张成功帧明确记录 `arrival_frame_completion.used_past_geometry_deadline=true`。这些帧在旧参考超过原寿命后，仍按同帧固定完成期限、当前源新鲜度及全部原身份/几何门提交新测量。它们构成共用修复真实生效的证据；不表示当前源门被取消，也不意味着所有阶段已无失效。本轮初始正常帧没有需要跨旧期限的成功案例，初始分支的边界覆盖仍以此前 31 项实际 producer 测试为据。

首失源为 **`1791548847236813306 ns`**，上一合格源为 **`1791548846835940837 ns`**，first-loss 观测时间为 **`1791548848037258148 ns`**。错误为 `perception_failure: source_frame_stale_or_future`，`failed_stage=post_refine`。当前源采集距旧参考 **0.400872469 秒**；进入凭证时旧参考仅 **0.448049546 秒**，当前源 **0.047177077 秒**。本帧凭证固定当前完成截止为 **`1791548848036813306 ns`**，旧几何截止为 **`1791548847635940837 ns`**，没有随处理续期。first-loss 时旧参考为 **1.201317311 秒**，但本次直接拒绝的是当前源 **0.800444842 秒**，不是再次执行 gen27 的旧参考入场拒绝。

| 本帧阶段/时钟 | 原始记录 |
| --- | --- |
| 当前源到 exact RGB-D 回调 | 6.131411 ms |
| 输入槽至开始处理 | 40.765968 ms |
| 开始处理时当前源年龄 | 46.941996 ms |
| decode / 主 exact TF wall | 1.046240 / 0.070561 ms |
| segmentation wall / 当前线程 CPU | 98.235995 / 91.701329 ms |
| tracking wall / 当前线程 CPU | 650.266082 / 413.082124 ms |
| 整帧处理 wall / 当前线程 CPU | 756.711025 / 511.122110 ms |
| `hold.total` wall | 1.097456 ms |
| `hold.feedback` 开始时当前源年龄 | 约 800.328 ms |
| `post_refine` 失败计时点 / first-loss 点当前源年龄 | 800.419808 / 800.444842 ms |
| 失败时等待槽中最新 RGB-D 年龄 | 38.806439 ms |

当前源进入时及时，处理期间新 RGB-D 继续到达；主 TF、参数上下文和锁等待均不是本帧的大头。虽然最终只比原 0.8 秒上限多 **0.444842 ms**，这仍是明确超龄，不能据接近边界放宽当前图像有效期。上一合格帧总处理 wall 为 **0.414686 秒**，tracking 为 **0.303797 秒**，其发布 mask 为 **13880 像素**；本帧约 **0.650266 秒**的 tracking 热点需要进一步量化和缩短。

本轮新增 `rgb_components` **没有普通 `refine` 计时项**。最近正常帧记录原 metric 匹配为零、八个碎片进入恢复；失败帧处于同一跟踪复合路径，恢复前后上下文检查之间约 **0.645 秒**，其中包含 fragment-group 的空间测量、内部 RGB 精修、成员所有权和最终几何复核。`hold.feedback` 开始时当前源已经超龄，该小段失败后，外层 `post_refine` 再明确报告当前源过期；`hold.total` 本身只有约 1.1 ms，不能把它当主要耗时。tracking 的 wall 与当前线程 CPU 相差约 **0.237 秒**，目前不能仅据此归因于 GIL、操作系统调度、native 函数竞争或 IO。嵌套计时相互包含，process CPU 也可能包含其他线程，不能相加冒充整帧耗时。

静态审计已发现同帧重复的点投影、footprint 处理和局部法向计算；该路径的 GrabCut 调用只有一次。这是待测优化方向，不是“反复 GrabCut”的已证结论。**首失定位阶段尚未完成重复计算优化；后续冻结的离线修复、定量结果和部署回执见末节；部署不替代新轮实机验证。** gen26 的 `rgb_refinement` 1.709 秒是普通 refine 与 hold 的另一复合段，gen28 的 tracking 0.650 秒不能借用 gen26 离线 0.13–0.15 秒精修结果来证明已定位同一函数瓶颈。

## 当前解决范围、证据来源与手动交接状态

| 问题 | 已证范围 | 尚未解决/不可推出 |
| --- | --- | --- |
| gen27 正常帧及时入场、TF 后旧参考超龄 | 无运动、关节不变；两种 normal 共用固定单帧完成规则，233 项通过、93 源实际部署 | 不归因为手动移动；凭证不让已经过期的当前源重新有效 |
| gen28 到位后旧参考跨界 | 19 张正常 ready 中 10 张真实使用凭证跨旧截止后提交 | 不代表可以延长当前源 0.8 秒，也不替代新阶段原门 |
| gen28 当前源处理耗时 | exact 输入及时；tracking 650 ms 为热点，TF/hold 非主因 | 同帧复用的离线收益及等价性已验证；实机最坏时延与系统竞争归因仍待验证 |
| 接触与实物抓取 | 本轮仅真实观察到位与正常跟踪 | 未执行接触或闭爪，`physical_success=false`；gen26 有限偏置端点诊断不等于可执行接触计划 |

新增证据全部位于同一外盘会话，不修改原实验或运行指针：

- `gen28_handoff_analysis/arrival_and_terminal.json`：原 request/ACK、阶段及封存终止，SHA **`3cd089e91ff0d72a151bcd43995e4eedae31b8e7771f561282afc0b0ae2b80fa`**。
- `gen28_handoff_analysis/timing_summary.json`：19 张正常帧、10 次成功跨旧截止、首失与上一合格帧完整计时，SHA **`4012126d5844877ded42360b858a7f86cc36b32023e36653f73a733b9155dfb2`**；原 detector 状态另存 `detector_status.json`。
- `gen28_handoff_analysis/exact_source_images/manifest.json`：anchor、运动前来源以及首失/上一合格源共四份 exact RGB-D、TF、Object/mask、相机参数读取区间和 accepted 括号，SHA **`f752a24f6dc2ec22b55fc735d1a936754687c795c5e04e1ff085d7c25b44cb9c`**。失败源发布的零 mask 是失效通知，不是实际 RGB 精修输出；导出本身没有执行分割。运动前内存 histogram/contour 没有单独记录，不能把按原 RGB/mask 推导的值冒充直接录制值，也不能只靠跨 topic 接收次序认定冻结参考。

`manual_ready_after_gen28.json` 在 **`1791549250893002607 ns`** 记录 `task_active=false`、`gui_manual=true`，未发送运动或力矩命令、未查询温度，SHA **`585066e89d292604f9e9a2c77d94c1612bb71827834bade95094ebad3643ea19`**。该手动交接之后收到的真实新确认见下一节；本次没有把旧 115240Z 回执、GUI 手动可用或文档同步当成新的用户确认。后续新实验仍须关联该真实确认、当前源和新计划。实验继续外盘 `local_only` 保留；本文与唯一既有同名外盘镜像同步，不变更父任务的状态指针。


## 同帧重复计算复用：118 项离线通过，123703Z 新确认尚未消费

`verification/fragment_same_frame_computation/result.json` 已冻结 **118 passed、0 failed、0 skipped，9.49 秒**，状态明确为 `ONDISK_NOT_DEPLOYED`、`physical_grasp_verified=false`，SHA **`a4af3fba1bea15e614c2aa0647c6505e085567a17b8cc10e38166b13bcceb072`**。修改仅在同一次 fragment-group 调用内复用原先重复的投影/固定 footprint 空间域，以及严格相同点、深度、有效域上的只读局部拟合；原独立高度与标签运算、每成员归属、当前真实深度、RGB 有效域、面积、唯一关联、支撑及外观门保留。不存在跨帧缓存或公共 direct 路径新增缓存权限，当前源 0.8 秒和所有提交时钟不改。

对本轮上一合格源 **`1791548846835940837 ns`** 与首失源 **`1791548847236813306 ns`**，修改前后 mask、raw mask、位置、尺寸、footprint **逐 bit 相同，全部 metrics 相同**；前一源还与原线上发布 mask 逐 bit 相同。首失源线上只有失效空 mask，因此其内部结果只作修前/修后等价，不能称与线上未记录的精修结果相同。每源各五次前后离线调用，前一源 wall 中位 **0.214634 → 0.163793 秒**，首失源 **0.197696 → 0.134794 秒**，分别减少约 **23.69% / 31.82%**。这些是有限离线调用的诊断计时，不是完整 producer 处理时延、实机全负载最坏界限，也未复现原 0.650 秒在线状态。精确对照 `exact_comparison.json` SHA **`bfab623b764e148bf891aabf7825e423dd8360c5ee274e7ec444ea154f2b551e`**；原始前后重放保留在 `gen28_handoff_analysis/fragment_timing_before/` 与 `fragment_timing_after/`。修复仍待执行代理部署及新轮原门实机验证，不能写作抓取已成功。

用户随后以实际消息 **`call_753bd992577041c2b1a57b3a0af9c1b8:0`** 确认“已对准，可以继续自动抓取”。新 `realigned_20261009T123703Z/alignment_receipt.json` 的采样时间为 **`1791549423991301298 ns`**，RGB 源 **`1791549423966398239 ns`**，accepted 反馈源 **`1791549423954989193 ns`**；`previous_alignment_reused=false`，控制 epoch 保持 **`1791506524580754980 ns`**，回执 SHA **`57c0cca6b3d4af55eabc2117f77a644a7b062d666f58005a07af38769715208a`**。截至本节同步，该确认 **UNUSED，尚未消费到下一轮实验**；回执中的 generation 28 是当时保留的上一轮 selection，不代表新轮已经创建。历史 115240Z“未消费”描述仅对应其当时写入时点，不覆盖后来 gen27/gen28 的真实使用。本次文档及镜像更新无 ROS 操作、无温度读取，也不修改执行代理的运行指针。


## 同帧复用最终 96 源已部署

上述最终验证将新增 8 项测试改为自包含后另跑 **8 passed / 1.08 秒**，覆盖并替代首组内同 8 项；去重仍为 **118 项**，不加成 126 项。最终 `result.json` SHA 为 **`a4af3fba1bea15e614c2aa0647c6505e085567a17b8cc10e38166b13bcceb072`**；生产三源未因测试夹具自包含调整而改变。合并清单 `verification/fragment_same_frame_computation/deployment_sources.json` 共 **96 源**，SHA **`2d94524159f25eaffaceec9345dfc472161e286a74405952b852ddaad28393d0`**。

`fragment_same_frame_deployment/result.json` 已确认 **`deployment_passed=true`**，SHA **`93c572bcd6eff07edcd5895c6feba6bc837d562f5b2f136bc56a44ff352fc316`**；新 unit 为 **`alicia-observation-measurement-1791549844207305993`**。unknown/remote/task PID **214272/214273/214274 → 219903/219904/219905**，八个核心进程 PID、启动标记与父进程全部保持，部署未发运动或力矩命令，未查询温度。验证工件中的 `deployed=false` 保留为其冻结时状态，实际加载以此后部署回执为准。部署要求自动抓取前新建 selection；当前 reset ACK 所带 generation 28 不是新实验。下一轮需使用真实 123703Z 确认并重新获取源与完整路径，此处没有新轮执行或实物抓取成功证据。


## Generation 29 实际启动

`grasp_records/20261009T124424.224503Z_ee2856eb8dab4529a6d029babf69f7a5/authorization.json` 已将真实 **123703Z** 消息 `call_753bd992577041c2b1a57b3a0af9c1b8:0` 关联到新 generation **29**、selection stamp **`1791549865975115299 ns`**，明确 `old_plan_reused=false`。这更新上节确认尚未消费的历史时点。record 于 **2026-10-09 12:44:24.225129 UTC** 开始，当前为 `recording/result=unknown/recorder_closed=false`。

日志在 **`1791549874.352791 s`** 记录首观察候选原严格审核通过，`checked=1/170`；request 2 发布新的 `PREVIEW_READY`。任务随后发出与 generation 29、target epoch 4、view 1 同绑定的 executing request，unknown/remote 在 **`1791549875.341602 s`** 均 accepted。此处仅记录首次观察计划及协议推进，不把 executing ACK 当成实际到位或接触成功。监控仅读取本轮文本日志与小型元数据，不打开活动大 bag；后续到位、接触拒绝及终局另据实追加。


## Generation 29 封存：三站测量保持有效，终局为接触候选不足

本轮 manifest 已于 **2026-10-09 12:45:34.593927 UTC** 封存为 **`complete/failure/recorder_closed=true`**，SHA **`89fefcf94829a1f25d6d4d9306cf730b465a38db33af603713427e61e1cd0895`**。三站均获得 unknown 与 remote 对同一组三张实际源的 qualified ACK，到位至 remote ACK 的录包接收间隔依次为 **1.040461064 / 0.675583601 / 0.789664269 秒**。第三站源为 **`1791549926147498130 / 1791549926348215341 / 1791549926648993968 ns`**，随后正常进入 phase 3 的实测注册及接触重规划。

346 条 detector 状态中 **120 ready** 均对应唯一源；3 条 unavailable 仅为初始化等待 exact RGB-D、确认第 1/2 帧，均 `target_lost=false`；223 条 pending 全为预期的 `observation_measurement_pending`。**本轮没有 first_loss，也没有 typed support-gap 终止。** 到位后全录制共有 73 张正常 ready，但其中 12 张发生在任务 FAILED 之后的空闲感知；首次任务失败前实际为 **61 张，三个站依次 21 / 30 / 10 张**。不能把 73 张全部写成失败前的任务进展。

首站 21 张正常帧均走 fragment-group；tracking wall 中位/最大 **0.194379 / 0.265306 秒**，整帧 wall 中位/最大 **0.313036 / 0.390623 秒**，发布时当前源年龄最大 **0.424189 秒**。全部 73 张正常帧的当前源年龄最大 **0.581111 秒**，其中一次按共用完成凭证跨过旧参考截止后成功发布。第 2/3 站走原非 group 路径，因此其 tracking 小段不能与首站 group 的整个计算量直接比较。本轮事实说明同帧复用部署后首次完整负载实验未复现 gen28 当前帧超龄；不同视角及场景不是受控性能基准，不能推断任何负载下均满足 0.8 秒。

终局原任务状态源为 **`1791549930297894954 ns`**：`NEAR_FIELD_SURFACE_VIEW_REQUIRED: current fused snapshot has no hard-safe candidate`；随后 **`1791549930301838636 ns`** 释放执行槽。request 24 审计/日志摘要为 `locally_valid 12→0`、`tabletop_materialized=0`，原 rejection 汇总含 6 项双侧实测不足、6 项 learned 姿态 profile 不可用，另有 sweep collision 的跨阶段计数。它们不是 13 个通过相同阶段的候选。本轮尚未进入严格接触路径预检，因此没有可供解读的新增名义/偏置端点碰撞诊断；request 完成时 `GENERATION_STALE` 是任务终止后的发布状态，不替代原规划失败原因。第二及第三观察已执行，并不自动保证双侧接触信息充分；候选几何和补视终止条件由实际冻结审计继续独立检查。

本节仅提取封存的 detector、measurement request/ACK、task 和 accepted joint 非温度元数据，没有解码 RGB-D 或重跑分割。新增索引：`gen29_handoff_analysis/arrival_and_terminal.json` SHA **`9dab308ce06589dcd1ebc15ddcf00e60226c397378b2583ab5db05a12429adbb`**；`detector_status.json` SHA **`3d0648b017799851941375e357fdc65ce9e4620d3a54909feceb3afafe8ada3a`**；`timing_summary.json` SHA **`a2a497420154d1674ad224c32dce21c1440b8251590d1409fd7f7e29203c24d7`**。原始时间、去重统计、失败前/后边界与计时限制均保留，不以观察成功替代实物抓取成功。


## 124851Z 新真实对准待用

用户实际消息 **`call_976d28f0e4e34012b3bac2e9ea54481d:0`** 确认“已对准，可以继续自动抓取”，新 `realigned_20261009T124851Z/alignment_receipt.json` SHA **`0bce104fe41869d54668504c762fc3b6fc77d0caf5aca7a829c098725f93e42c`**。采样时间 **`1791550131785866737 ns`**、图像源 **`1791550131760946273 ns`**、accepted 反馈源 **`1791550131751020817 ns`**，`previous_alignment_reused=false`；旧 selection generation 29 只是采样时保留状态。此新回执当前 **UNUSED**，可供后续新一轮创建，尚无 generation 30 实验或动作事实。当前仍在分析融合后的接触候选和补视信息，不能把感知时效修复已通过本轮写成全部抓取问题已解决。


## Gen29 接触角度复算：加密角采样不能补足实测共同高度

`gen29_contact_analysis/angle_window/result.json` SHA **`399213c6124eaa0c97f376e223517263cbdf6b01fd67c9df719db967bbe9937e`** 已完成只读封存源对照。16 份 exact 输入重建七个原 SurfaceView：初始仍是原三帧融合，每个 reached-view 仍使用对应真实单帧，direct contact snapshot 使用各自原三帧。最终 FusedTargetSurface 为 **2058 点**，各视角代表点计数与线上审计一致；最终外包络尺寸为 **63.943 × 47.985 × 23.076 mm**，与原审计一致。它是保留当前中心后的注册点外包络，不能当成某一个当前视角直接测得的盒宽。

原 5° 角度网格复算得到 **26** 个提案，完整工厂 materialized **0**，26 项 reach 值与原线上精确一致。另在 40–70° 内执行 **601** 个轻量角对照，边界区间步长 0.01°、其余 0.25°；其中 258 个采样角同时通过 reach 与双侧点数的局部门，范围 **42.17–44.78°**，但这不是连续角域可行证明，也不是完整接触候选。对该窄窗内 **27** 个角运行原完整工厂，仍 **0** 个 materialized；原接触共同高度/patch overlap 最大仅 **0.316142 mm**，低于未改变的 **2 mm** 要求。由此不调整生产角度采样，也不以局部 reach/点数通过冒充接触通过；本回放没有进入 MoveIt 或执行。

在短轴方向约 **53.843872°**，原两侧当前平面高度的 5–95% 区间分别为：负侧 **19.995–21.464 mm**，正侧 **6.720–19.976 mm**。负侧主要只有顶边，双侧区间没有共同高度；原负/正侧即使有 **117/311** 个点，也不能替代实际足够的可接触高度。窄角窗仅产生很薄的共同区间，原完整 patch 门继续拒绝。这支持继续获取弱侧更充分的真实测量，而非降低双侧或碰撞条件。

重建边界仍明确保留：request 12 早期注册矩阵最大数值差 **3.473942e-6**；后两次注册矩阵与审计相同。最终联合点数均为 **2897**，双向点集 Hausdorff 差 **64.874 微米**、点顺序不同，因此不是整份运行时融合数据逐 bit 复现。反事实角扫描使用未替换的原重建表面，没有把审计 XYZ 匹配回填成输入。外包络短轴极值来自保留的第三站 reached-view 注册点，注册位移和保守包络变宽本身不能证明配准错误，也不能仅凭点云范围排除深度边缘噪声。以上均不授权删除点或放宽门。

原 req19 同源补视源 **`1791549916246385812 ns`** 已另用于有界 FK 反事实：更强的 **2× 视觉噪声尺度**预期侧面暴露提案可满足原端点审核，仍只是生成目标，不改变原最终支撑、可视域、光轴、following、碰撞或测量门。拟实现仅在 `reachable_observation_views.py` 内将 baseline 与 2× 提案共用原 **2 秒**预算；默认接触/角采样不变。该方案随后的最终冻结与排序验证见下一节，仍未部署。端点反事实不证明完整 MoveIt 路径、实际能看到足够侧面或实物抓取成功。最新 124851Z 实际对准确认仍 **UNUSED**。


## 预测侧面暴露的分层提案已冻结，尚未部署

最终 `verification/observation_exposure_tiers/result.json` SHA **`2b4315ad93bc4d3b337e3dc08c4a0bf2a9657419c190331aa72a7ddbf97d3bd1`**，共 **84 passed、0 failed、0 skipped / 6.07 秒**；生产仅改变 `reachable_observation_views.py`，SHA **`cec802754905e1e97e622a8c5e17886e1260d251542ea33e4375b90b9e272ee4`**。生成顺序先完成原 baseline 各方向，再尝试额外 2× 暴露目标，两方向最多四次尝试共用原 prepared **2 秒**预算；额外层失败或来不及完成不丢弃已合格 baseline。它不是把最终接受噪声翻倍，不改变原角采样、观察次数上限 3、strict/执行/到位或接触代码。

同源生产回放 `gen29_contact_analysis/exposure_tiers_production/result.json` SHA **`6d20eff901ba9238ae15fc40d48f3db7c05bfe5ae5aa5e4373c16902e5f77718`**，req19 的 **136974** 个真实支撑点摘要与原审计精确一致，arrival prism 审计也一致。四个提案在共享预算内总用时 **0.430059 秒**，保留各自按原噪声和所有原端点门计算的真实预测收益。原 mode 信息/运动排序不改：同一缺失负侧的首 FK 候选由 baseline 的预测增益 **3.599707 mm**、首排序代价 **18.536002**，变为 2× 提案的 **6.469819 mm**、代价 **4.407215**；两者 support 均为 sufficient。新提案最大单关节变化约 **0.476703 rad**，高于 baseline **0.239831 rad**，因此更充分的预期测量也可能带来更多运动和后续路径拒绝。

以上只是实际生成器与原排序的离线反事实，运输与 current-ticket 所有权使用夹具；**没有调用严格 MoveIt、全臂完整路径或实际运动**。最终侧面门仍按原 **3 mm**，更高提案目标不证明一定达到其请求值，更不能保证新增测量具备原 **2 mm** 共同接触高度。后续仍须原完整路径、到位三帧身份/支撑/注册和接触审核。最终状态为 `FROZEN_VERIFIED_NOT_DEPLOYED_BY_THIS_AGENT`，本文尚无该方案部署回执；最新 124851Z 对准仍未用于新一轮，不声称实物抓取已成功。


## 98 源分层提案已实际部署，Generation 30 使用新确认启动

最终独审 `verification/observation_exposure_tiers/independent_review.json` SHA **`6c1f330e686f8c614d5a5b580f7d592f7c2dd528ecf912315624fae05451c2e0`** 确认生成、原 mode 排序及预算/失败回退边界，无新实机权限。合并清单 `deployment_sources.json` 为 **98 源**，SHA **`1e4cc2e16204839dac0aad7d0a99b087f27dcec6015e1c9cf8d76112084bac64`**；96 源基线中 94 项未变，更新单个生产 helper 与原测试，新增专项测试及离线回放脚本。

`observation_exposure_tiers_deployment/result.json` 已记录 **`deployment_passed=true`**，SHA **`0ec4902d497a53f43ba2b0bb12cde0bd9c57a87c96f2bfead3afbe6fa068ae9d`**。新 unit **`alicia-observation-measurement-1791551116370397825`**；unknown/remote/task PID **219903/219904/219905 → 226839/226840/226841**，八个核心进程身份保持，部署无运动、力矩命令或温度查询。前述冻结结果的“未部署”是其当时状态，实际加载由此回执确认。

新录制 **`grasp_records/20261009T130538.916778Z_c2652d76024c434c928011052680fad0`** 的授权已实际消费 124851Z 消息 **`call_976d28f0e4e34012b3bac2e9ea54481d:0`**，创建 generation **30**、selection stamp **`1791551140568510532 ns`**，`old_plan_reused=false`。首观察 request 2 在 **`1791551149.105445 s`** 通过原严格路径选择，`checked=1/169`，executing 于 **`1791551150.140105 s`** 完成双端 ACK。

实际首计划审计 `eeba80e69413ee376e8acf7e452c142627aa460f234921bfa556dbbab6c2894a.json` 保存了新层级配置 `[1,2]`，选择仍为 **baseline tier 1**、预测增益 **3.200 mm**。第一个 baseline 方向约 **0.248 秒**通过，第二个 baseline 方向又用约 **1.697 秒**至共享总预算 **2.000319 秒**耗尽，故本次未尝试 2×，没有移除已合格 baseline。这是实际预算约束的预期行为，不能写成 2× 未加载或已被执行；后续站位仍须按新的实际审计区分。当前只监控文本与原子提交审计，不读取活动大 bag；观察到位和整任务结果待后续记录。


## Generation 30 封存：第二站实际 2× 提案通过，接触预检随后被原门拒绝

录制 `grasp_records/20261009T130538.916778Z_c2652d76024c434c928011052680fad0` 于 **2026-10-09 13:07:02.500457 UTC** 封存，manifest 为 **`complete/failure/recorder_closed=true`**，SHA **`b6c6ddbb25f13fb394175bb66ec81d61e8a7f2ace831540254af6667504c7123`**，runner 退出码 6。两站均获得 unknown/remote 对同组三个实际源的 qualified ACK，到位至 remote ACK 录包接收间隔分别为 **2.516128063 / 1.077168941 秒**。第一站曾有一张当前源超原期限而未通过，随后另三张真实合格源完成确认，没有放宽时效。

与首计划 baseline 不同，第二站 request **14** 实际选中 **tier 2.0**，预测缺侧增益 **6.510315775 mm**；两方向、两个层级共四次求解均 accepted。原严格选择 `checked=1/186` 通过，第二观察实际完成并静止确认，三源为 **`1791551192636539220 / 1791551192937194824 / 1791551193205377578 ns`**，双端 qualified 日志时间 **`1791551193.560326 s`**，之后进入 near-field phase 2。实际审计 `22ca21546e72ad1e71df29d624c79f0ab35f0e8e33f5bbf58fcf00ba272fab28.json` 保留该提案与原门结果；这不是只用旧源反事实推出的实际成功，但到位也不替代最终接触审核。

本轮确实进入接触严格预检。**`1791551207.068996 / 1791551208.331038 s`** 的两条新诊断均为 request **23**、同一 plan **`3a5e43936885310a2765ceac`**、同一源 **`1791551195107482671 ns`**，不同实时静止采样，不能算成两个独立已穷尽候选。`checked_stage_names=[pregrasp,approach,grasp]`，失败阶段为 `grasp`，原因为 **`predicted_command continuous geometry: target OBB collision at 11-end/left_finger`**。原要求开口 **38.333906 mm**，真实 accepted 开口 **49.75 mm**；不能将该预测碰撞简称为“开口不足”。

诊断保存了原四个接触 pose、同 frame 目标 OBB、接触高度证据、当前 SDK/accepted 静止样本和 bias。两次观察站位 `SDK−accepted` 均约 **`[+2,+13,+12,+3,−2,−4]`** 编码量子；诊断 `joint_error_sdk_counts` 字段采用相反的误差符号，应按其定义解读。`scope=read_only_candidate_constant_bias_prediction`、`execution_authorized=false`、`hardware_response_verified=false`，且没有保存原失败轨迹。因此这是原门真实拒绝及其输入，不是未来接触位姿实际碰撞的测量。后续离线高度偏移端点即使通过必要条件，也不能写成已通过完整路径、真实跟随或实物抓取；当前仍在独立复核。

## Gen30 感知首失：及时当前帧在 segmentation 内耗尽原完成期限

首永久失效的当前源 **`1791551214349752426 ns`**，上一合格源 **`1791551214008858919 ns`**，first-loss 观测 **`1791551216091520071 ns`**，录包收到状态 **`1791551216271806716 ns`**。外层原因是 `observation_arrival_station_invalidated_during_recovery`，原新增细节为 **`arrival_frame_completion_binding_changed_or_expired`**，`failed_stage=post_segment`。

| 本帧时序 | 原字段 |
| --- | --- |
| 当前采集与上一合格源间隔 | 0.340893507 s |
| exact RGB-D 回调时当前源年龄 | 14.103651 ms |
| 开始处理时当前源年龄 | 34.266233 ms |
| 完成凭证入口当前源 / 旧参考年龄 | 34.314394 / 375.207901 ms |
| 原固定当前源完成截止 | `1791551215149752426 ns` |
| decode wall / 主 TF wall | 1.116399 / 70.576415 ms |
| segmentation wall / 当前线程 CPU | 1.634106307 / 1.029160025 s |
| 整帧处理 wall / 当前线程 CPU | 1.710201710 / 1.035995039 s |
| post_segment 当前源年龄 | 1.741712808 s |
| first-loss 当前源 / 旧参考年龄 | 1.741767645 / 2.082661152 s |
| 失败时等待槽最新 exact RGB-D 年龄 | 50.385236 ms |

正常单帧完成凭证已在及时入口建立并绑定同一源/参考；到分割返回时，当前图像本身已超过原 **0.8 秒**约 **0.942 秒**，不能通过放宽旧参考期限救回。尚未进入 tracking、fragment-group 恢复或普通 RGB 精修，故本次不能归为上一轮优化前的 fragment-group tracking 热点。segmentation 是复合标签，现有计时不能直接把全部 1.634 秒归到 RANSAC、连通分量、锁或调度；其线程 CPU 约 1.029 秒，仅显示其中相当部分实际执行及约 0.605 秒其他耗时。输入继续到达，没有由这些字段证明相机停流。

任务失败前两站分别 **32 / 49** 张正常 ready，共 **81** 个唯一当前源；第二站 49 张均通过 fragment-group，其中 32 张成功使用原完成凭证跨旧参考截止。此前没有 typed support/metric pending 记录；192 条 pending 都是 observation measurement 生命周期等待。原完整检测状态共 344 条，随后 `target_lost_requires_new_generation` 是首失锁存的结果，不能当成新的独立首因。原门拒绝后未发布新的有效几何，没有借历史模板复活目标。

最小像素工件已一次导出四个 exact 源：初始 anchor **`1791551141090538501`**（mask 7703 像素）、第二次运动前最后 ready **`1791551182303511142`**（13356）、上一合格源 **`1791551214008858919`**（16124）、首失源 **`1791551214349752426`**（发布失效 mask 为 0）。四份均含原 RGB/depth、TF、手眼安装变换、Object/mask、camera 参数原读取区间及 accepted 前后括号，没有邻帧替换或本次导出中的分割。运动前最后 ready 同时早于 executing request 和 unknown ACK，期间没有另一 ready；内存 histogram/contour 没有直接记录，后续推导必须保留该限制。首失零 mask 仍只是失效通知，不是精修结果。

本会话新增索引：

- `gen30_handoff_analysis/arrival_and_terminal.json` SHA **`05f8f6ec07c5254a8dfae757c769a159838919a97c04bd40cc699d9033f8a4dc`**，`detector_status.json` SHA **`82477f879e5897870416336f69098dd17b960c1185505785bdf905e259bd1ce8`**。
- `gen30_handoff_analysis/timing_summary.json` SHA **`27a476bd35ca07f1501af9d0872d58fcc977ad7d8dfc98587b8620ceacd3bf0f`**；`failure_timing.json` SHA **`cebc2487b9e795eaef19183f979c72ec888498b1905b2af70fc6392fa375ddad`**，保留首失及上一合格帧完整计时。
- `gen30_handoff_analysis/contact_rejections.json` SHA **`ac6f64d00a043cb9566921e4f7e794dab3d471faf18740558f54596c5b35cc52`**；`selected_views.json` SHA **`b0dc359fa9bef154a7a5ea571ca88e166413d68a796ce4074ba7167597f191f0`**。
- `gen30_handoff_analysis/exact_source_images/manifest.json` SHA **`22e578d4d6e4ddba6452ec576fafcea3e0f0076c77e8166c5b420afa21511325`**。后续原分割性能分析由这些像素工件进行，本文不预先写作修复完成。

## Gen30 后手动交接与待确认

`manual_ready_after_gen30.json` 于 **`1791551442177813768 ns`** 记录 `task_active=false`、`gui_manual=true`、`new_user_confirmation_received=false`，SHA **`7f45c6a12f3729464443f010897d3be7f506c9fd204648f1f0d047b02a7b8cf6`**。恢复 GUI 和只读现场采样未发运动或力矩命令、未查询温度；该回执明确不是用户对准确认。执行代理已请求重新对准，截至本节尚未收到新回复。最后 **124851Z** 真实回执已经由 gen30 消费，不再次标为 UNUSED 或据此虚构下一轮确认。最终实际部署仍为上述 **98 源**，分割/接触后续分析尚不构成新部署或全路径成功；运行指针只由执行代理维护。

## Gen30 后诊断结论与修复边界

同源原分割回放 `gen30_handoff_analysis/segmentation_timing_before/result.json` SHA **`fe144cb7647b2ccfa93393691176e9cfce32c6f1c7a5f651f244fe95a56ba59f`**：首失帧三次纯分割 wall 为 **0.109884–0.116931 秒**，未复现在线 1.634106 秒；上一合格帧在线分割约 **0.11537 秒 / 0.11083 秒 thread CPU**，与离线接近。原在线 context 与其反馈锁在整帧约 0.07483 秒前已结束，而 segmentation 复合段在约 1.706 秒才结束。现有证据不能把余下耗时定为 RANSAC、GC、缺页、线程调度或 GrabCut；该帧根本未进入 RGB/fragment tracking。离线使用原源与冻结参考，但保留 TF 浮点重建、Config 默认值与前一帧路由参考限制，不能声称完整在线进程复现。

失败后仅做 12.374 秒只读 host `/proc` 采样：detector 的 29 个线程数量稳定，线程 CPU 合计 **6.038 秒**，runqueue wait 合计 **0.603 秒**；最忙线程分别 **3.963 / 0.092 秒**。没有 major fault，记录 **395655** 次 minor fault；这既不是磁盘 I/O 因果证明，也不能投射到此前失败瞬间。实际环境 `OPENBLAS_NUM_THREADS=1`、`OMP_NUM_THREADS=1`，系统 NumPy **1.17.4**、OpenCV **4.2.0**，启动日志确认 OpenCV worker threads **1**；加载的是系统 BLAS/LAPACK，没有 MKL/OpenBLAS。旧 gen14 的 12/1 线程邻帧对照使用用户 OpenCV **4.13.0**，不能当成本轮等环境重放，未重复该实验。完整结果 `gen30_handoff_analysis/runtime_load_diagnostic/interpretation.json` SHA **`cb61457f016b8bfbda2ca4c9ee00af72eeda282a353075828b77d00856902eb6`**。此调查没有附加图像消费者、ROS 操作或生产修改。

接触诊断 `gen30_contact_analysis/diagnosis_summary.json` SHA **`b477d3c3711d265365bc8b5f150557b54215b587ebd5a640de4e7874f1d1c2a1`** 保留原记录的 nominal 端点可行、固定 SDK−accepted 预测左指碰撞事实。原高度窗口内 **+2.551426 mm** 的独立 IK 对照可通过原解析高度及连续夹爪曲线审核；它没有使用原 req23 完整融合点云或当时失败 MoveIt 轨迹，也没有证明全臂碰撞、原融合面/注册审核、时效、实际跟踪和最终夹持。因此不将其写成“找到可执行抓取路径”；原高度重试为何提前返回仍需要新的实际分支记录。

## 101 源组合验证与实际部署

本次合并三项范围明确分开：

- **调度实修**：`reachable_observation_views.py` SHA **`2080bbd19d2cfd5881fe42658e71d4e9c989c3f404bcdbb5ee77aba75755c1fd`**。首个基线必须通过全部原端点门才获得同方向 2× 优先级，随后继续剩余 baseline 和其他 2×；仍共享原 **2 秒**、每方向/每 tier 的原实测 q 单 seed、默认 **96** 迭代。失败/超时保留此前已合格候选，原下游 strict/following/完整路径和到位测量不变。优先加强可能占用其他方向的预算是显式取舍，不保证共同接触高度。**86 passed**；结果 SHA **`c51b8e916e16ad2af02686074198adc16fb736dd378a35cc219b070f340fde1e`**，独审 SHA **`e424290e64a490ec0a977ddb06a8ef36de99f2cb91dea4d5720ad15f74443ee7`**。同源生产 FK/原 mode 排序回放 `exact_replay/result.json` SHA **`ad7854e052ad00b04e5b8a1f3728233e0184cb7363b278b7b1045d1980b26110`** 未调用 strict 服务，不提供运动权限。
- **分割诊断**：保留原 segmentation stage，新增状态锁获取、站位资格、纯数值分割 wall/thread/process CPU，以及进程级 GC collection 计数差。没有改变算法、GC 配置、门限或截止时间；GC 计数不是耗时或线程因果归属。相关 **86 passed**，`verification/segmentation_stage_component_timing/result.json` SHA **`780e2b24bcd974fa39860b47e56992f5b7b62a85774d9947f059efa49f488941`**。这项改动用于定位下次尖峰，不是尖峰已修复。
- **接触分支诊断**：持久记录原接触高度重试的准入短路条件、有限提案与原 gate 结果，不改变高度窗口、候选、碰撞、路径、ticket 或截止判断。相关 **76 passed**，`verification/contact_height_retry_diagnostic/result.json` SHA **`63514055d1879c513b747e719b676fda1983b66b9ea8f4d7e849056500d09fba`**。它保留下一轮首拒绝证据，不让离线 +2.551426 mm 对照获得执行资格。

最终 `verification/gen30_combined_repairs/deployment_sources.json` SHA **`8df288e0e55d5aa4ecaa46fd4bdd50c1a745aa997e2c59105f92b456a47ff547`** 含 **101 源**：相对前 98 源清单，**94 项未变、4 项更新、3 项新增**，去重 **248 项测试通过**，`physical_success_verified=false`。各子报告中的未部署状态是其冻结时事实，实际加载以此后的部署回执为准。

`gen30_combined_repairs_deployment/result.json` SHA **`cdcc3bc0d147d65f5ef92bd5a3b84bbcc7422ee6f6c287aabf2666551c8b553c`** 已确认 `deployment_passed=true`；unit **`alicia-observation-measurement-1791552339094869980`**，unknown/remote/task PID **234213 / 234214 / 234215**，父进程 **234163**。八个保留核心进程的 PID、start ticks 与父进程逐项一致。此部署没有运动或力矩命令、没有温度查询，也未替换底层驱动、相机、网关或 MoveIt；旧 generation 30 reset ACK 不是新抓取选择。

## 131549Z 真实确认与 Generation 31 启动

用户实际消息 **`call_98dea3d0d3134c05ad1fd11cb1a6f0e1:0`**：“已对准，可以继续自动抓取”，已保存 `realigned_20261009T131549Z/alignment_receipt.json`，SHA **`0e6d025908e78735ccb092b2664dbe2d826e72add0337010ddba66860747d048`**。采样时间 **`1791551750483961105 ns`**、图像源 **`1791551750463110685 ns`**、accepted 反馈源 **`1791551750446076359 ns`**，控制 epoch **`1791506524580754980 ns`**，`previous_alignment_reused=false`。这是更新上一节“尚无回复”的真实新确认，不是自动合成确认。

该回执已由新记录 **`grasp_records/20261009T132559.988617Z_2163a7a76bce42c78b2c17c952d2eb0e`** 的 `authorization.json` 消费，selection **generation 31**、stamp **`1791552361682603597 ns`**，`old_plan_reused=false`、`physical_success=unverified`。记录于 **2026-10-09 13:25:59.989099 UTC** 启动；本节核查时小型 manifest 为 `recording/result=unknown/recorder_closed=false`，未打开活动大 bag。后续观察到位、接触及终局需另依本轮实际证据追加。本文与唯一外盘镜像只记录已核实部署和启动事实，不改执行代理运行指针，不提前声称实机问题已全部解决。


## 2026-10-09：第 31 轮根因、站位计算暂失恢复与第 32 轮

第 31 轮记录 `20261009T132559.988617Z_2163a7a76bce42c78b2c17c952d2eb0e` 已封存为失败。两次补视均完成三源到位复核，均选到 2 倍增益观察候选；没有执行 CONTACT 或闭爪。首失效帧输入年龄约 28 ms，处理完成时约 3.298 s，其中 tracking 墙钟 2.639 s、线程 CPU 0.173 s，纯分割墙钟约 0.333 s、GC 次数为零。它不是第 30 轮纯分割耗时尖峰的同一证据，不能直接归因于接触信息不足。第 31 轮的晚到接触预检发生在任务结束后：原阶段有 90 秒期限，撤销后旧 prepared 保留 near_field，旧截止辅助函数返回 0；0 对服务表示原隐式 15 秒预算，不能描述为预算从未赋值或无限期。

本轮恢复修改仅适用于已独立确认到位的补视站位：当前几何仍受原最长 0.8 秒限制，过期立即清掉可执行几何、缓存和旧规划票据。目标身份最多等待原源时间起 5 秒，并同时受原任务和阶段截止约束；本地单调时钟也不续期。恢复必须有新的合格 RGB-D 测量、真实 accepted 关节反馈、固定站位/身份以及同源 Object/Mask/status 完整交接。正常及时 READY 不增加额外恢复硬门限。迟到旧 pending 不得推翻已恢复的新源，同源目标、掩码和合同不得被改写。补视切换和 CONTACT 冻结撤销该等待；旧视角计划只保留会话历史用途，不能重新执行。

另修复接触预检的提交前、服务返回后阶段/票据/期限复核；已取消任务不能借零截止继续计算或提交结果。legacy_gated 原合法零截止语义保留，但仍受原活动阶段期限和票据约束。没有改接触、碰撞、支撑阈值，也没有启用温度查询、关节失能或强制返回初始位。

最终 976 个去重测试通过：共享合同 59、感知 175、远端 369、任务与三端联测 277、接触取消 96。三端联测使用同一真实算法生成的发布消息，模拟处理延迟后丢弃旧结果，再接收新的完整测量；时钟、传输及物理计划构造为离线测试条件，不代表实机成功。统一清单为 `verification/station_compute_reacquisition_combined/deployment_sources.json`，111 个文件，其中基于前 101 源修改 4 个已有文件、新增 10 个；清单 SHA256 `149b35b4f66c79d21cf1b7797f4f252096d11349519ca5d1720efe1c65e3a6f5`。

部署 `station_compute_reacquisition_deployment/result.json` 已通过，单元 `alicia-observation-measurement-1791555456072996126`，只重载三计算节点，八硬件/GUI 节点 PID 与启动标识保持。第 32 轮使用此前真实确认 `call_d9f90b3c65f94d4fa1be9de3022226b1:0` 的 13:30:48 UTC 对准回执，启动前原执行器核实当前真实关节位置一致；该回执现已消费，位置变为历史起点。记录目录 `20261009T141807.628466Z_3d22b7ddd3734f50a5a707e2ffe5aacb`，selection stamp `1791555489253252267`，只读监督先于启动运行。此处记录时第一补视运动已开始，实物抓取尚未验证。所有实验留在 SanStand，local_only，不上传或删除。


第 32 轮补充结果：记录已封存 failure。第一观察位运动完成，10 条真实 accepted 反馈给出静止窗口；requalifying 请求后 remote 约 3.278 ms、unknown 约 5.541 ms 均返回 accepted，但未提供 qualified 三帧证明。3 秒窗口内 17 帧及另 1 条晚完成帧均拒于 `arrival_unique_original_identity_required`，处理接收年龄约 89–126 ms。全轮没有 `station_compute_wait` 合同，因此不能把此次归于新 5 秒恢复机制或计算超时；候选为零还是多匹配须原始像素复现。证据 `gen32_handoff_analysis/arrival_details.json`，SHA256 `4bdeead98b1239d0e3f4e61aa3849450f93daa241261cf0f5e28bf5f77519dd9`。没有执行 CONTACT/闭爪，GUI 已恢复手动控制，未失能关节。

用户随后实际确认 `call_6b3f5439eb7346ab985fac30071ed99d:0`，新回执 `realigned_20261009T142210Z/alignment_receipt.json`，receipt_ns `1791555731558829545`，epoch `1791552579261248640`；当前索引已指向这次新对准，不复用上轮已消费回执。该新确认尚未用于新一轮运动，等待到位身份匹配复现和修复。


## 第 32 轮多匹配碎片修复与第 33 轮初始计算超时

第 32 轮精确实录源重放确认 4 个深度块中 2 块各自通过原目标几何匹配，旧分支只处理零匹配，因此未进入 RGB 碎片恢复。新分支让每个完整原生匹配候选独立精修，均须证明同一当前 RGB 实例、互相拥有各候选可信核心，再取共同 RGB 交集并仅用当前真实深度重算几何。未裁掉竞争候选，未填历史点，也未改变原支撑、接触、碰撞或源帧时效门限。到位确认与之后正常跟踪均接入；零/单候选、初始一次性路径保持。真实双物体反例仍拒绝。

最终 279 项去重测试通过。精确 3 帧重放 `gen32_handoff_analysis/matching_fragments_final_replay/result.json` SHA256 `3424c5d26a6f979206f081c1e4614f1992256931a048dd001ab46f3fd4880f1d` 使用最终源码，第三帧完成确认；这仅为离线证明。新统一清单 `verification/arrival_matching_combined/deployment_sources.json` 共 116 文件（原 111 中 3 修改、108 未变，另加 3 测试和 2 重放脚本），SHA256 `8f957e0091e6935433c72e5930b244df7bb1b5373691e6a85cb038b9d432cf30`。`arrival_matching_deployment/result.json` 已通过，单元 `alicia-observation-measurement-1791557103353525295`，只重载三个计算节点，八个保留节点身份保持一致。

真实 142210Z 对准回执已用于第 33 轮 `20261009T144532.807783Z_f42fde5ce926439082043f73f1a4715e`，没有补视、CONTACT 或闭爪运动。该轮初始源 `1791557139297346353` 进入处理时 22.865 ms，纯分割正常返回耗时 1.188927 s（本线程 CPU 0.801110 s、全进程 CPU 2.299995 s、GC 0），完成后当前源年龄 1.244983 s，超出原 0.8 s，触发 `initial_station_current_support_invalidated`，内因 `arrival_frame_completion_binding_changed_or_expired`。不能将其归为新多碎片分支失败或接触条件太严；也不能仅由函数正常返回推断内部独立支撑审核合格。最新待处理帧当时仍很新鲜。

执行器已以 3 退出，本轮没有任务终局状态来自动结束 recorder，故仅向该已结束尝试的记录包装 PID 263687 发送 SIGINT，正常关闭只读录制并封存 failure；未向机械臂节点发任何信号、失能或温度指令。现转向精确源分割分项性能与初始站身份等待边界定位；不把此次写成补视实机通过，也未虚构新用户确认。


## 第 33 轮精确源性能定位与进程局部分配器修复

同一精确帧、同一输出的离线分割耗时出现 0.117–1.685 秒波动；单次慢 locked 分割 0.449 秒中用户 CPU 0.037 秒、系统 CPU 0.412 秒，6347 次 minor faults、零 major faults，固定投影本身亦出现 0.240 秒。宿主短窗没有内存不足、swap/reclaim/compaction/THP 或 CPU 饱和证据，不能归因于图像复杂度、RANSAC 增多或温度。原始诊断索引 `gen33_handoff_analysis/segmentation_diagnosis_summary.json` SHA256 `f476a8d6ae4a9bfd4d06c7e50261a37cdd5ae8a3a80db91bb7f9aaf05fc63d38`。

单进程 A/B 仅加 glibc 环境 `MALLOC_TRIM_THRESHOLD_=-1`、`MALLOC_MMAP_THRESHOLD_=33554432`。10 次稳态测量 minor faults 17754→0、系统 CPU 0.043986→0.000076 秒、平均 wall 0.078154→0.066293 秒；两帧全部掩码、几何、平面、足迹、质量与指标逐字节一致。该短窗 baseline 未复现秒级尾延迟，不能声称实机尖峰已根治。保留页可能保留 RSS 高水位，不构成目标历史像素缓存或固定内存上限。

修改只落在 unknown 节点私有 env：当前热部署 launch 与正式 `src/alicia_grasp_modes/launch/perception_modes.launch` 同步；所有几何/身份算法和门限不变。XML 节点结构对照除这两个 env 外完全一致。新清单 `verification/allocator_page_reuse/deployment_sources.json` 共 121 文件，SHA256 `78328775a35d0a310ec04c28724c334a2515f5767f8b49d0302c571395630d2e`；原 116 清单仅运行 launch 改变，加入正式 launch 与四个研究脚本。前次 279 测试是算法未改的历史证据，不称本次重跑。

`allocator_page_reuse_deployment/result.json` 已通过，unit `alicia-observation-measurement-1791557539481197719`，保留八节点身份；unknown PID 268265 的实际环境已核验。第 34 轮记录 `20261009T145258.586806Z_51a72aa111a74c4d95d2d58e75a1dc07` 使用原 142210Z 持续授权同姿态重试，gen33 从未运动，启动器仍检查新鲜 accepted q 与原位置。原确认时间与消息 ID 未改，另存 `same_pose_retry_context.json`，不虚构新确认。此节时尚无实物抓取成功。


## 2026-10-09（洛杉矶）用户要求完整重启

第 34 轮在首个补视位完成三帧确认后失败于 `TARGET_LOST: station_task_owner_changed`，随后感知端失效是任务结束后绑定撤销的结果，不是处理超时。first-loss 之前 168 READY（167 带时序）源龄中位 0.23355 秒、最高 0.42149 秒；171 次纯分割中位 0.08402 秒、最高 0.10448 秒。详见 `full_latest_ros_20261009T003407Z/gen34_handoff_analysis/allocator_live_performance_result.json`；实物未抓取。后续已通过真实回调复现：remote 的 `TARGET_PROCESSING_PENDING` 无效几何通知进入 task 的基类回调，误将等待状态升级为执行权永久撤销。原三端测试漏接这条几何消息回调；修复和补回归正在隔离副本进行，尚未部署。

用户要求“继续处理，但是你要重新启动各节点和上位机”。当前 Ubuntu 已重新启动、旧进程全无，初次 USB 未接入。用户明确回复接好后，SanStand 挂载 `/dev/sdb2`（UUID 4899-D74E），机械臂 `/dev/alicia_arm -> ttyACM0`，RealSense D405 已识别。新完整启动会话 `/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z`（目录 UTC 2026-10-10，洛杉矶当地 2026-10-09）。新入口 `.runtime/full_latest_restart_20261009.launch` 保留旧完整配置并明确 detect_hz=6、software_temperature_monitoring_enabled=false、temperature_poll_rate_hz=0。正式 unknown 节点仍带已验证 MALLOC_* 私有环境。

驱动、硬件接口、MoveIt、网关、相机、感知、任务、模式路由、夹爪控制、GUI、只读监督及独立本地仿真已启动；启动单元与外盘日志见新 `startup.json`。`runtime_receipt.json` 记录 driver PID7146、camera7157、GUI7201、gateway7156、unknown7179、remote7181、task7194、gripper7196。真实 sdk_measured 反馈约 4.9 ms，彩色图像约 6 ms；GUI 为手动。正向使能状态 `PENDING:POSITIVE_ENABLE_REQUESTED`，不得写作实测动作已确认。未查询温度、未发失能，也未恢复旧 generation/计划/对准姿态。新实测关节接近零位，与旧补视位不同；已请求用户重新对准。触觉节点运行但 `/dev/alicia_skin` 缺失，已询问接入情况；不能宣称闭爪触觉确认可用。


## 新启动后真实对准、几何等待回调修复与新 generation 2

用户真实回复 `call_h9lzSgD0r2zvf319zsVlyUNt:0` 已对准，回执位于新会话 `realigned_20261010T002626Z/alignment_receipt.json`，receipt1791591988281432151，epoch1791591831641651789，actuation 为 CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE。此为本次新位置；此前旧起点不再使用。用户同时说明触觉设备暂时无法连接。随后询问为何启动触觉，已解释完整历史入口 start_tactile=true 带入可选节点；没有把设备缺失作为当前 RGB-D 规划调试的前置条件，也没有改夹持确认规则。

几何回调修复已通过152项唯一测试：remote 无效几何通知附带原已登记契约与受限恢复链；task 仅在原qualified站位身份、实测关节、epoch、源上下文和固定期限成立时将其作为等待，不给旧几何或旧路径执行权限。已验证的旧契约晚到、未经历本地pending但已收到更新READY、漏收中间READY等情形不再误撤销当前状态；普通无效/CONTACT/真实目标丢失仍原样撤销，已撤销的权限绝不清回。真实 remote.geometry_pub→task.target_geometry_cb 已永久接入三端联测，原故障在修复前稳定复现。旧manifest121中更新3生产+1联测文件，新加1测试及3启动/回执脚本，125源统一清单SHA `7d06c8ac3584f1637ec7bccb6f31ba33fd74b869eb282c76160a53e8fd98e73b`。

`station_pending_geometry_deployment/result.json` 已通过，unit `alicia-observation-measurement-1791592282725350656`，只重载三计算节点，保留本次完整启动的八硬件/GUI节点。新 generation 2（重启后编号重新开始）于记录 `20261010T003150.142267Z_d03ab5d4d39f42f9ae4853a0de6bd652` 启动，selection_stamp1791592311921588182，使用本次002626真实确认。此处仍运行，不能写作已完成补视或抓取。


## 2026-10-09 洛杉矶：重启后 generation 2 交接修复与 VS Code 卡死定位

新 generation 2 记录 `20261010T003150.142267Z_d03ab5d4d39f42f9ae4853a0de6bd652` 已封存 failure，没有 CONTACT 或闭爪。首次补视完成三帧审核；第一个 pending 后仍运行约 16.55 秒。首次致命通知来自 remote 的 `station_compute_wait_registration_too_late`：源 1791592404502324819 在原 mask/object 回调期间越过 1791592405302324819 截止，1791592405311663150 发布 TARGET_LOST；此时更新源 1791592404840430736 已齐且尚未过期。task 约 70 ms 后的 station_task_owner_changed 与 producer 后续 station_changed 都是结果。193 个到位后 accepted 反馈无关节变化，记录未见 GUI 接管或 epoch 变化，不能归咎用户手动操作。

修复仅更新 `station_compute_wait_remote.py`：缓存每个候选重新读取当前时钟；原门控完成后候选变旧则立即隔离、丢弃，原已注册身份只能在原固定期限内等新的完整测量。保留原门控、实测关节、身份、epoch、0.8 秒几何及 5 秒身份期限；真实失效仍终止。新12项在原125源中复现6项失败，修复后相关164项全通过；两名代理分别审阅，root核对隔离副本与正式源逐字节一致。统一126源清单 `verification/station_registration_combined/deployment_sources.json` SHA256 `1c88b9ea6b1236449b8e85b8c613b347af64d597b309e484e3d85c6eb0434c4d`。`station_registration_deployment/result.json` 确认部署通过，unit `alicia-observation-measurement-1791593505171141008`；只重载3计算节点，8个硬件/GUI节点身份一致，无使能关闭或温度查询。当前尚未新实机验证，上一确认是已经消费的历史起点，GUI手动且已请求用户新对准。

近场参数只读审计每次75次ROS参数读取，12组中位35.75ms、最大44.98ms；历史494–882ms是含队列/锁等待的端到端指标，不能归因为参数读取。四命名空间新快照离线对照节约约30ms但本次未混入生产修改。证据 `gen2_handoff_analysis/runtime_param_latency/complete/audit_and_review.json`。

VS Code 两次主扩展宿主无响应；当前PID14832的10秒采样有97.77%样本在 TestItem pushDiff，调用栈进入 `ranch-hand-robotics.rde-ros-2-1.2.1` 的 parsePythonTestFile。该ROS2扩展每次测试文件变动无节流全量发现，显式只排除build；工作区931个Python测试文件、11809个测试方法（含调试基线）。已隔离的 SunYingkai 用量扩展没有占用CPU，不是本次根因。系统无内存压力，Codex后台继续运行。证据 `codex_ui_analysis/extension_host_profile_analysis.json` SHA256 `acd2119d37b4de92c416fb72fc8768e53ec5d0b23468af8202f43ae6d36e1b15`。

root与protocol-v3两个工作区 `.vscode/settings.json` 已给 ROS2 扩展配置 affinity=2，原使用量隔离配置保持，修改前备份在外盘 `codex_ui_analysis/`。本机VSCode源码确认需要 Developer: Reload Window 重建布局，仅 Restart Extension Host 不足。此节仅记录配置与刷新准备，尚未宣称已生效或卡死根治；隔离也不会消除ROS2自身扫描计算。所有记录local_only，无云端上传或删除。


## 2026-10-09 洛杉矶：插件隔离实测与重启后 generation 3

已执行 VS Code 的 Developer: Reload Window；`codex_ui_analysis/vscode_reload_20261010T005630.584588Z.json` 记录实际确认，未强制关闭进程。刷新后 Codex 主宿主 PID28041（后台28313）、用量宿主28068、ROS2及依赖宿主28073，实际进程与日志文件归属证明隔离生效。60秒观测主宿主平均CPU0.817%；随后在 generation 3 实验中独立120秒观测平均0.70%、最高2.1%，未新增扩展宿主无响应。后者 `generation3_editor_load_120s.json` SHA256 `5f99cb6a175ab5962c70207db030ab3c97a70c7bddc1421d373be799156c94ca`。该改动隔离 ROS2 测试全量扫描的影响，不是修改 ROS2 扫描算法；有限观测不能证明永不再卡。八个硬件/GUI进程身份在窗口刷新前后完全一致。

实际用户回复 `call_98f40ce7292749ee8d75a846d5df29b4:0` 已对准，回执 `realigned_20261010T010111Z/alignment_receipt.json`，receipt_ns1791594072120602130、epoch1791591831641651789。新 generation 3 selection_stamp1791594095215439558，记录 `20261010T010133.592558Z_0897bf6257e840319544f71499dac236` 已封存 failure；没有补视运动、CONTACT或夹爪命令，1047条六关节实际反馈无变化。

本轮539条故障前感知状态中447 READY、80短暂支撑缺测；538条处理时序最高源龄0.418466806秒，没有超0.8秒或producer TARGET_LOST。约100秒初始规划反复被支撑缺测取消，最后已发布观察计划在执行预检期间遇到源1791594201710199594缺测。remote旧逻辑在计划发布后退出初始缺测保持作用域，把missing object提升为TARGET_LOST并推进track g1-t11→g1-t12，task随后按原规则永久撤销权限；失败在首个物理动作之前。真实bag→task callback复现 `gen3_handoff_analysis/initial_pending_callback_replay.json` SHA256 `4e95d7da0280f445d0a270477198085b8b9e9bf61fd4ec35e50ba95f7df5a081`，后到READY不会清除已撤销权限。

9帧离线像素回放复现3次支撑缺测和6个相邻成功帧。末次缺测三环内点比65.22%/63.83%/68.94%，均有万级点数，残差P90约3.85–4.11mm；8个RANSAC种子仍不通过。全部支撑环样本属于center滤波区，107条1Hz元数据中center仅24条实际时间滤波、71条warming；整体局部滤波active不足以说明桌面支撑区被滤波。此周期采样不证明逐帧重置因果。`gen3_handoff_analysis/support_residuals/final/result.json` SHA256 `3f9a46cc6c2a8e66519c62ed8e75e3c65d747b3955d0abd16308285eb48b48f8`。未放宽接触、支撑或碰撞门限。

当前生产仍为已核对126源版本。正在隔离开发初始预检阶段的非执行缺测等待；不清除旧hard revoke，不以收信时间续0.8/1.6秒期限。跨selection自动重选不部署：它缺少旧目标身份绑定且首次失败后的录制会自动封包。没有实物抓取成功，没有新的对准回复被合成，没有关节失能/温度查询或云端上传。


### 2026-10-10 UTC：初始补视预检短暂缺测交接已部署

- 新增有界、不可执行的初始预检等待凭据，绑定原代次/目标/平面/实测关节/控制epoch及原截止时间；缺测期间撤下执行几何，恢复后重做预检，再经原测量双ACK及完整执行检查提交。仅适用于初始 unknown + observe_return FAR_FIELD 且夹爪已张开、尚未提交动作。直接接触与后续动作保留原检查。
- 第一次合并538项有6失败：4项是暂存目录遗漏原测试fixture，2项是缺字段时提前退回/诊断收尾健壮性缺口；均修复后最终539项全部通过。独立增量审阅通过。真实gen3消息六种交付顺序回放通过，回放动作为惰性计数，不能视为实机抓取成功。
- `verification/initial_preflight_combined_final/deployment_sources.json` 固定135个源文件；`initial_preflight_deployment/result.json` 验证仅重载三个计算节点，八个硬件/GUI节点进程身份保持。未发布失能命令、未查询温度。
- 修复不覆盖注册之前或原始测量双ACK接纳过程中出现的所有缺测；不扩展0.8秒几何时效，不清硬撤销，不改平面/接触门限。待gen4实机证据。


### 用户暂停：gen4 规划阶段失败，未运动

- 用户原对准确认为 `call_98f40ce7292749ee8d75a846d5df29b4:0`，续接前20份实测反馈与原位置最大差0、控制epoch不变；未伪造新的确认时间。
- 实验 `20261010T014859.062441Z_bcfc39d681c8414798c8270fb3bd4c72`：尚未获得可执行计划，无初始preflight token/measurement lease和实际关节位移。连续4个 support_plane_inlier_ratio_low 缺测后，来源1791596972173598766晚于旧参考0.8秒捕获截止67.215ms，被 identity_completion_source_not_captured_in_geometry_window 拒绝。当前帧约13.56ms、处理约4.4ms，不是本轮计算超时；新交接代码尚未触达。
- 首因证据：`gen4_handoff_analysis/first_loss_support_capture_cutoff.json`。原始日志未提供这4帧具体内点比例，不猜测数值。
- 收到“先暂停”，已停止新分析/验证，runner此前已返回3，随后仅SIGINT指定录制包装进程使bag正常封存，结果failure。节点和使能未改变，没有温度查询。相机15fps完整负载试验尚未执行，必须等用户恢复。


### 2026-10-10 UTC：恢复、15fps完整负载试验与gen5

- 用户“继续处理”后，仅相机切换到15fps；实际SDK双流15fps已确认，标定/投影/滤波配置和所有平面/抓取门限不变。驱动、GUI及计算节点身份保持，相机新PID49913。`fps15_full_load_trial/camera_transition/result.json`记录原进程、原配置、替换进程和实际profile；`runtime_baseline.json`是切换后的身份基线，不覆盖先前部署原件。YAML尚未持久化15fps。
- 以真实旧确认继续，20份新实测反馈仅Joint1差约0.001534rad，其余不变、控制epoch相同；当前图像盒子居中。新gen5重新获取目标，约13秒获得初始计划。初始preflight arm/commit与原measurement双ACK通过，起动前163帧未见support pending，最大source年龄0.2721s。此轮未触发新的初始缺测恢复，不声称实机验证该分支。
- 实物机械臂完成一次补视，view_index1到位并重测qualified，然后从补视结束位置规划抓取，无强制返回初始位。
- 最终4个候选均通过hard_recheck；首个最终路径审核缺独立sdk_transmitted样本，其余3个在grasp段出现名义几何夹指/目标OBB碰撞。任务首因 NEAR_FIELD_NO_REACHABLE_CANDIDATE；释放任务之后的arrival invalidation为后果，不能反写成首因。未进入接近/闭爪，无实物夹持成功。
- 实验`20261010T020909.515250Z_ec46b035f0d74e2688bb217beb3da64a`正常封存failure。GUI已恢复manual，已询问用户重新对准；修复期间继续离线分析。本轮15fps改善了初始观察链路，但单轮不是根治证明。


### 2026-10-09 LA：用户实测窄边40mm与gen5接触路径修复

- 用户明确给出实物窄边40mm，已作为独立外部尺寸参考保存 `gen5_contact_analysis/user_measured_width.json`；不把该数值强写进感知几何。当前夹爪实测开口49.75mm，物体真实宽度与代码某一姿态下的保守投影宽度必须区分。
- 由原始观测精确复建1848点融合面和2705点碰撞包络。保守OBB短边45.477561mm，同轴实际点跨度41.453764mm；保留旧中心且对称扩展的包络增加4.023797mm。原50°方向相对OBB54.591°短轴产生50.410368mm投影，不能解释为实物宽50mm。不删除测量点、不改旧身份中心或碰撞门限。
- 新增最多4个物体轴邻近方向，仅用于 unknown + observe_return 已qualified补视后的 observing_contact/near_field 分支。实录原生成、融合注册、双侧接触、高度居中、merge、stable、hard recheck、去重及原排序得到4个新52.295652°候选；倾角14.601640°/15.329570°由原算法得到，±2.676768mm高度余量与49.75mm开口端点碰撞均通过，原50°仍复现碰撞。端点通过不代表IK、全路径或物理抓取成功。
- SDK预检仅在精确缺少独立sdk_transmitted样本原因下，允许原候选截止内最多0.5s等待；两路真实来源都必须推进，原0.5s新鲜度和0.3s静止窗不变。健康手动只读preview保留，等待期间控制归属/epoch改变失败。同步ROS参数RPC不能强制中断，超时返回后失败，不能宣称绝对0.5s墙钟响应保证。
- 用户最新对准回执 `realigned_20261010T021256Z/alignment_receipt.json`，后续20份实测位置与回执最大差0，epoch1791591831641651789相同。修复尚未据此宣称抓取成功。

- 最终组合316个唯一用例全部通过；首轮24失败（23个隔离目录缺tools依赖、1个false分支接口兼容）完整保留，补齐依赖并让false分支沿用原调用后重跑全过。源清单 `verification/gen5_contact_combined/deployment_sources.json` 固定140文件，旧源码tar留在项目。两项独立审阅通过。
- 空闲网关原PID7156关闭请求已执行，但RPC连接先于回包关闭；原失败结果保留，未重复发送关闭请求。只读复核新PID61834、原父进程7001、其余10节点身份/关节位置/epoch/手动模式不变，`contact_sdk_gateway_verification/result.json`为通过证据。随后仅三个计算节点重载，`contact_yaw_consumers_deployment/result.json`通过；驱动、相机、上位机继续运行。
- gen6实机记录 `20261010T024239.212466Z_bd212804e3d2427d8b64e164bf10cec0`，以本次真实用户对准回执启动，新代次获取，不复用旧计划。当前已进入首次补视运动，尚不能标记接触或夹持成功。

- gen6已封存failure：首次补视实测最大关节位移1.23178657rad，到位三帧与双ACK均合格；qualified后约6.19ms任务报告冻结权限已撤销，尚未进入CONTACT，故新接触角度/SDK补样本分支未实机触达。日志在qualified前1.68s已有 `Grasp ignored stale object source age 38.377s outside [0, 8.000]s`；旧ObjectPose通过通用clear路径提前置撤销。源迟到的准确callback/锁复现待修复验证，不能写成当前新图像超时或物体4cm不可抓。
- 本轮GUI恢复manual后用户实际重新对准 `call_d0b59087e9e3454e9cb45d7a878d4a8e:0`，新回执 `realigned_20261010T024559Z/alignment_receipt.json`。修复完成后以该位置继续，未伪造或复用旧确认。


### 2026-10-09 LA：gen6初始预检锁生命周期回归修复与gen7

- 旧ObjectPose源1791600172893596172在独立录包器收到时仅0.235343s旧；原任务初始预检把plan锁放在base时效判断前，恰好等待同步补视运动后判38.377s过龄并通用清权。录包器收到时间不是任务真实入口时间；用实录消息与显式双线程时钟调度复现该竞争。
- 已committed后的obj/refresh恢复原有锁顺序；armed→committed竞争仅对当前同目标/同selection/同epoch/同有效执行lease、入口新鲜、因本地等待变旧的positive源作丢弃。canonical measurement digest与ROS序列化digest各自独立核对。没有更新几何/续时/清revoked；false、invalid、新源、入口已旧、fatal collector、owner变化保留原拒绝。
- 22项针对测试和322项组合全部通过（不相加）；独审报告 `verification/gen6_callback_handoff_independent_review.json`。变更只有initial_preflight_task.py与新测试，源码141清单已固定。`callback_handoff_deployment/result.json`证明只重载3计算节点、8硬件/GUI身份保持。旧源码tar在项目，实验外盘local_only，未查询温度或失能。
- 最新实际用户确认对应024559回执，20条当前实测与该位置最大差0、epoch不变。gen7新录制 `20261010T030141.809371Z_7724ffb5910c4565bf751bd9c281954c` 开始，结果待实机验证。
- Codex并行25秒负载复核未见新unresponsive：Codex宿主CPU平均2.80%、ROS宿主2.24%，仍隔离；仅为本次观察，不宣称所有界面消息已渲染或永不再卡。报告 `codex_ui_analysis/generation6_editor_load_25s_20261010T024718Z.json`。


### 2026-10-09 LA：gen7第二补视准入的计算合同退休竞争

- gen7 已封存 failure。首补视实测最大关节位移1.168893rad，三帧到位、双ACK、CAD审核通过，进入接触规划；source141旧回调超龄问题在本轮未复现。
- 第二补视准入时，在途post_segment仍持有被MOVE_PREGRASP回调退休的station-compute合同，被误判station_changed并提升为目标丢失。当前源龄0.176193s；到位至失败208条实测反馈六轴范围均0，无第二运动、接近或闭爪。原始88消息fixture与观察器封存于gen7_handoff_analysis。
- 第一补视的新增轴邻近角度确实参与搜索。精确四源1762点融合复建显示，窄边两侧共同高度约0.286mm，交集只剩7/4点；总点多不代表双侧接触带充分。继续核查观察面及滤波影响，未放宽接触门限。
- 用户真实新确认call_a0682891287d4429ac21a5cfbe10eb96:0已捕获realigned_20261010T030640Z/alignment_receipt.json，尚未用于新实验。待修复组合验证与独审后部署，仅重载计算节点。仍无实物夹持成功。

- GEN7退休修复最终570个唯一用例全部通过，独审通过。仅2个生产文件及3个测试/fixture变化；144源清单与旧源tar已保存。station_handoff_deployment/result.json证明仅三个计算节点重载，八个硬件/GUI身份保持。
- 030640确认后实测关节与epoch再次变化；用户真实新确认call_8d784c686b3b44ae80d24983d1157c76:0已捕获031346回执，原回执未用于动作。新epoch1791601627465467664；以新selection规划，不复用GEN7旧计划。


### 2026-10-09 LA：gen8到位后新测量提交受阻

- 实录20261010T031939.390394Z_5136b6b5d69d49a3bd1ba57514758965已封存failure；首次补视实测最大位移0.898913rad，三帧到位与双ACK通过，随后原90秒接触规划窗口没有新候选进入规划。持续SUBMISSION_REJECTED，日志current stamp是last_submitted_stamp_ns，不能据此认定当前相机/协调器测量陈旧。
- task首报TARGET_LOST: station_compute_wait_expired_or_future，producer失效在其后约8ms，不能倒置首因。没有第二观察运动、接近或闭爪。GUI已恢复手动，结束图中盒子偏左下，已询问真实新对准。当前source144仍部署，未放宽任何超时。
- GEN7侧面像素追查补充：7张原记录supervisor深度图目标ROI中4–16mm支撑高度点为0；RGB窄侧条区域大多无深度。取消global MAD的诊断重放只增顶面点，不增加侧面高度；不能通过扩大mask或取消MAD恢复不存在的测量。该证据不代表未滤波原生sensor。


### 2026-10-09 LA：并行修复负载下插件隔离复测

- 03:31 UTC 25秒观测，原Codex宿主28041保持同身份，平均CPU0.520%，ROS/Python/C++宿主28073约100.078%，新日志无unresponsive；低CPU不代表逐条webview渲染已被验证。原始报告generation8_repair_editor_load_25s_20261010T033142Z.json。
- 本地ROS2扩展1.2.1源确认：每个测试文件创建/变化都无去抖地全工作区discoverTests，findFiles仅显式排除build。已有日志223条发现事件、单轮32263项，生成的.runtime隔离副本会放大扫描。两工作区新增files.watcherExclude/search.exclude的**/.runtime/**，保留真实src与.worktrees源码可搜索；设置备份及哈希在codex_ui_analysis/runtime_watcher_exclusion。
- 临时生成文件探针已移除，25秒未新增discover日志，Codex平均0.559%，无新unresponsive；ROS宿主仍99.676%，不能据此宣称已有扫描/CPU占用被清除。未改扩展二进制、重启编辑器或ROS节点。probe_result.json留存边界。

- gen8原消息取证进一步确认：226组三源均齐且源龄<0.8s，906个实测站位样本六轴范围0；source间隔中位399.754ms是被选中处理帧间隔，不能说相机只有2.5fps。处理wall中位394.07ms，CPU中位342.71ms；tracking阶段中位212.87ms，segment107.50ms，原记录segment调用每帧仅1次。细分见gen8_handoff_analysis/processing_timing_distribution.json。
- 首个新三源到齐时，旧pin已过原0.8s约17.529ms。原回调先tick旧pin、quarantine并撤计算，再join新测量、推cutoff到新源−1；三帧融合因此反复含cutoff前的帧。真实Mode callback+submit隔离复现此饥饿，原完整planning活性修复仍开发中，尚未部署。修复设计将纯时效的未发布计算等待与执行几何/真实失效撤销分开；不改原当前源/最终接触路径门限，不使已撤执行计划复活。
- 实际新对准call_Avpgo5ml6jpBJFRH19GpAIZg:0已保存realigned_20261010T032445Z，epoch1791602660508766273。没有新的实机动作。
- VSCode本地API复核：findFiles使用files.exclude而忽略search.exclude，即便调用方指定build排除；另已加files.exclude **/.runtime/**供真正发现阶段排除。外盘file_discovery_exclusion.json保存代码证据与设置前后哈希，未据此宣称当前ROS扩展积压已消退。


### 用户暂停：gen8修复定向通过，尚未组合验证或部署

- 用户明确“先暂停”，三个协作智能体已中断；不启动新实机验证、不重载节点。gen8此前已failure并恢复GUI手动，自动runner已退出6。
- 实机仍为已核对source144。隔离stage gen8_station_compute_staging_20261010的两生产文件冻结：station_compute_wait_remote.py SHA38a7686b4beb8e1e9f62bc6f7be74afb3210733d87f8b5b07601ca32ea2ad9c4，mode_aware_remote_grasp6d.py SHA24f9453c7bc85b89267d9ed3ebbfb81eb29af8744e0e3f8c7189ca486c32cb0f。86定向测试全过，报告verification/gen8_station_compute_work/result.json SHA2cac8dd78c799f0240727ebcff00215aa333e511a3730bdb2881075a30fc96ac。
- 暂停时实录8项测试仍在修正fixture装配：需用原parse_session将wire字符串deadline规范化，并保留原3帧single_snapshot_direct设置；原字节/时间戳不变。最终组合、5文件独审报告、源码集成和计算节点部署均未完成，不能宣称已修复实机抓取。
- 已通过的worker调度回归使用真实Mode/coordinator/activation/publication/metrics，CPU预测与候选求解为受控桩；不代表IK或实物夹持成功。实际032445新对准未被用于动作，恢复后需按当时实测位置/epoch核对后使用，不伪造新确认。


### 2026-10-09 LA：恢复并部署 gen8 新测量提交修复

- 用户恢复并确认手臂移开、可以自动抓取；原032445对准续接20份新反馈最大差0.001534rad，epoch不变，盒子居中。清场确认 call_4a8169002900451cb87b62779fbedfb3:0 保存于gen9_resume_pose。
- 原实录8项回放通过。首轮组合1200/1203，3项旧registration_commit指出过期候选source未推进hard cutoff；恢复 `_station_compute_quarantine(ns, clock_only=True)` 的ns，旧测试断言未改。首轮失败证据保留，最终1203项全部通过，报告SHA23f68c353a610435335db0a5ba17b4af6d608edd3975eedaedbd3c1659510c36。
- 只对原三帧single_snapshot_direct补视站位的纯时效间隙保留未发布计算，并逐源审核历史；真实缺测/失效仍撤销。发布可执行计划前必须重新具有完整当前证据并通过原门限，不复活已撤执行计划、不续原绝对任务期限。
- 五文件独审SHA df52a1bb8157b3641ccda99d22c9c8864ebb56f262a2fb8199d145cded5ad5e6；旧源tar留项目，部署源清单147。station_admission_deployment/result.json证实三计算节点重载成功，八硬件/GUI进程保持。实机抓取成功仍未验证。
- Codex独立宿主25秒平均CPU0.519%，无新增unresponsive；ROS2宿主99.984%仍在处理自动发现测试。新报告 generation8_repair_editor_load_25s_20261010T041836Z.json。本地ROS2扩展没有关闭测试发现的配置，后续处理仅限该工作区扩展，不更改ROS1运行节点。


### 2026-10-09 LA：gen9初始观察候选覆盖不足，尚未触达交接修复

- gen9记录20261010T042204.923624Z_7543ac3513a6431496aae778c0561e24已正常封存failure，runner3；2850条SDK六轴反馈无位移，没有任务运动token/measurement lease，接触及闭爪均未发生。第一个失败是初始silhouette观察搜索4/47，后77.152秒支撑失效不是首因。
- 四个严格路径失败均MOVEIT_UNREACHABLE/OBSERVATION_POSE_PLAN_FAILED：roll0/d462.624mm、roll0/d373.812mm、roll±15/d462.624mm。其余43因预算未检；47个20ms IK提示均unknown，不能写成47条路径不可达。d285/269.822/196.188mm有合格端点但未获路径检查。报告observation_coverage/recorded_candidate_assessment.json保存原47位姿与失败取证。修复在隔离目录开发，live仍147。
- 原任务未进入active，录包器将等满600秒；确认runner已退出且task/modeinactive后，仅对已核实record路径的只读录包器发SIGINT，其wrapper自行封存为原失败。两只读observer正常退出0；没有发机器人停止/失能命令。
- UI自动键入因焦点跳转两次落入编辑区，均立即Undo恢复原未保存内容，未保存磁盘。停止该自动化方案后，用户call_iRUQYqhLdFSBB2l0YrAPQyXi:0确认已禁用工作区ROS2；只读VSCode工作区状态证实disabled列表仅ranch-hand-robotics.rde-ros-2。正在验证运行负载，未宣称卡死永久消除。

- 用户工作区禁用ROS2后旧宿主仍99.76%，已通过目视确认的 Developer: Reload Window 正常刷新（043338回执command_submitted），原未保存标签保留。刷新后旧主机退出，新主机103093承载Codex/ROS1/Python/C++，25秒平均5.316%（峰14.39%），Codex appserver3.677%，无新增unresponsive；报告post_ros2_disabled_editor_load_25s_20261010T043742Z.json SHAa4be770131b259bb6f56aa3819e0aee1ae6dec04b17dfac9a6b4f0baf68732f1。只读核对11个ROS/GUI进程身份全保持，未重启节点。此为窗口采样结果，不声称永不再卡或所有webview消息逐条已验。


### 2026-10-09 LA：gen9轮廓诊断观察覆盖修复部署

- 仅diagnostic且没有positive IK提示、候选数>1的观察搜索，在原预算内先轮流检查不同距离/光轴；首个严格路径+following+当前ticket commit全部通过后恢复原排序比较。原47候选无删除、原3秒/6检查及碰撞/视野/身份门限不变，普通一次性抓取路径不进入此分支。
- 实录47pose与bag精确tool0→camera_link TF逐字段/指纹核验；新前5旧索引[17,24,0,10,1]，距离269.822/196.188/462.624/285/373.812mm。被调度不代表实际可达。新增失效ticket不能产出incumbent回归。
- 原36定向与最终404项组合均通过（不相加）；首组合402pass2fail为单候选额外mount依赖及隔离缺URDF，分别恢复原单候选路径、原样补资源，旧断言未改。全部证据保留，最终combined_r2报告SHA4fba9eec8aa4b381657f5db924a4d217f3fa7f921253ec5faf5a0d11b8995c05，独审SHAa827fa331f5373d1886094ba6f6a854031d42b95f681324d92300216a2905476。
- 五文件已集成、旧源tar在项目，新清单150。silhouette_coverage_deployment/result.json证实仅三计算节点重载成功，八硬件/GUI进程不变。没有实机夹持成功声明，正准备gen10验证。


### 2026-10-09 LA：gen10共同执行参考失效与一次正使能恢复

- source150实机记录20261010T044441.309012Z_081c1da83cbe47cc9da10a94a4578168已封存failure、runner3；1055条SDK反馈六轴无运动。本轮实际走ordinary观察，6/168终止；6个内部原因全部为OBSERVATION_PREVIEW_PATH_UNAVAILABLE: driver reference has no fresh actuation epoch: UNCONFIRMED:CONFIRMED_FEEDBACK_STALE，外层被归为MOVEIT_UNREACHABLE。following审核未触达，不能据此判位姿不可达。14/96端点IK提示positive，FK搜索2s零候选，无成功FK证据被丢弃。
- 首次stale在LA20:31:01，driver连续约2s拒绝Joint3由-12.4°到-25.0°的突跳反馈后撤销执行确认。此后新测量不自动恢复sticky UNCONFIRMED。本轮任务内221反馈最大间隔0.1114s，不能说本轮feedback中断。GEN9四个原cause仍为OBSERVATION_POSE_PLAN_FAILED，不倒写成GEN10首因。证据strict_actuation_failure_assessment.json SHA8aafcb3ef6601e874ad7fabae5fda9a204906f1c4c2eb84ffc5716acfd156019。
- GUI恢复manual，observer/录包器已正常关闭。按用户既有正使能授权，唯一发布/demonstration Bool(false)，该驱动分支调用request_positive_enable；未发布失能或joint target。新epoch1791607808551091347（旧1791602660508766273），真实q变化0，状态PENDING:POSITIVE_ENABLE_REQUESTED。receipt gen10_handoff_analysis/positive_enable_recovery/result.json；尚非CONFIRMED，必须真实方向响应确认。
- 当前计划修复仅类型化共同执行参考错误并停止同源候选重试，不改变driver确认、超时或碰撞规则。普通观察原有受检路径prefix可确认PENDING；silhouette仍按原规则要求已CONFIRMED，不偷开bootstrap。新epoch后只允许新selection/new plan，旧对准的人工确认事实保留，续接须验证此次已记录授权epoch过渡及新q，不伪造新用户确认。


### 2026-10-09 LA：GEN10执行参考失效分类修复部署，GEN11准备

- 普通和轮廓观察的严格服务适配器仅识别实录gateway明确失败前缀，将共同执行参考失效保留为OBSERVATION_REFERENCE_UNAVAILABLE；同candidate和跨family均首错停止，之前找到的selected/reachable结果同时退休。成功response、其他IK失败、普通接触不改；pipeline只追加拒绝码白名单，不创造成功证据。
- 定向39项通过；首次扩大组合565通过27失败，后者均为旧variant_local_reuse fixture缺prepared.ticket。用原部署150逐项对照，相同27项异常全文一致；该29项文件明确排除且不计通过。最终限定组合563/563，16旧测试字节不改。最终报告SHA53689bfc28a15469bd0a6b6337da57726ae342a69ac1fd9fb206fc0266d080ec，独审SHA719bcbdb8a717ba41682480cbafcf81acdd61832f25e552ab9ec07fbfc0c433e。旧fixture问题未宣称修复。
- 五文件集成、旧源tar留项目，清单153。shared_reference_deployment/result.json证实仅三计算节点重载，八硬件/GUI身份保持。驱动、实际方向确认、原碰撞/身份/时效门限均保持；正使能PENDING不等于CONFIRMED。后续GEN11按实测位置及真实已记录enable epoch过渡验证，不伪造新人工对准确认。
- Codex复测同宿主25秒平均2.718%（前次5.316%），前次精确日志offset至今无新增unresponsive或被禁用ROS2激活；报告post_ros2_disabled_editor_load_25s_20261010T045614Z.json SHA4931ea0ba564115ff18a02cac73fc54cbddafc625d1bb24b2840ec5e84298ebd。只覆盖采样与日志范围，不保证永久无卡顿或每条消息已渲染。


### 2026-10-09 LA：GEN11真实响应缺失、GEN12手动交接与重启后的GEN13

- GEN11记录20261010T045823.108983Z_afb71d5ad1424dad9de6275837bc5f78封存failure。已有普通补视全路径计划；3秒受检前缀SDK最大写入0.0245437rad，199条录包/1176条观察器accepted反馈无位移。1.0004秒响应窗口10次反馈均无方向变化，驱动先ENCODER_RESPONSE_TIMEOUT并退休epoch，约6–8ms后感知initial_station_current_support_invalidated，再约0.73秒任务失败。图像age0.212秒；不能把二次感知失效写成测量超时首因。
- 原trajectory goal容差0.035rad大于0.025rad前缀，action success不代表实际运动；上层真实方向确认正确拒绝。原C++离线重放，正向2count可确认，1count及反向均不确认。非零速度word150和正使能包与官方SDK一致；GEN8三秒对照同协议曾实际运动。写入成功不是设备执行ACK，没有依据扩大幅度/超时。
- 用户真实确认本体供电正常、急停释放且无故障指示，随后说明原来仍手动控制、现在解除并授权。050405新执行授权记录不伪称新对准；一次正使能清理失败，GEN12新selection。该轮无preview或命令，GUI在1791608724048318386变manual，约89ms后initial_station参考失效；不是epoch先变。失败40.917秒后反馈近零，处在用户宣布重启背景，不能当作自动抓取造成的物理位移。
- GEN12记录20261010T050518.801299Z_cdd5a365fa5b41ac9aa1da12c4c3b45d封存failure，GUImanual恢复，仅只读录包器和observer关闭。用户重启后明确“已经对准了目标物，你可以执行了”，050736新实测q和新epoch1791608767718356620已记录，actuation为CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE。新画面左上前臂清场由真实call_52725b3590934ca3a93a22453d68d7ef:0确认。GEN13记录20261010T050849.339487Z_fa4a10a8358a4fa2b80214f1f626b5c7开始，仍未宣称抓取成功。所有门限保持，未发失能或温度查询。


### GEN13第二站支撑偏移的独立诊断，尚未归因机械松动

- 首补视后req33为接触复核（约1.25秒），12 learned候选为5双侧不足、6姿态profile不可用、1单侧开合行程不足；tabletop18提议中16先因单侧行程失败，剩2共高区间1.4544/1.2271mm，记录的最大CAD重叠1.4544/1.1772mm低于原2mm。窄边41.05mm加1mm余量仍小于50mm开口，不能说4cm太宽。req34的15秒是第二观察位搜索。原报告first_view_contact_rejections.json保留局部日志只记录前8项的边界。
- 第二站双方及时ACK requalifying，3秒18帧中1等待到位后源、17稳定支撑局部差5.543–5.904mm>4mm，qualified源0。精确重放3帧逐值复现；支撑ratio88–89%、RMS约1mm、4象限，主要是锚点整体法向偏移约−5.3至−5.5mm，边缘倾斜约±0.25mm，不能再归因点少或处理超时。
- 最初用默认URDF诊断出现不一致，已确认是诊断模型选择错误并保留原报告；正确expanded metric URDF哈希7812d6b69e854ed9f37b4e3cbd813ecd53dab3504065b2d04d287abc6fd852e0与6份审计176处模型引用一致，精确SDK插值+bag手眼重建TF到浮点误差。没有由此发现运行坐标链错误。
- 审核发生在原始base点的任何ICP之前；现ICP也先要求raw plane局部差≤4mm，25mm限的是通过配准的目标点修正幅度，不能用它替代支撑审核。源码消费者未见本轮raw/registered混比。不得删除审核后声称ICP已解决偏移。
- 三站角度和距离同步变化，单一全局深度offset/scale不能单独解释两个站位。6.193mm固定相机平移可拟合两约束但3未知欠约束；同站留出重复帧不是独立标定验证，未部署任何修正。正在用静态桌面RGB-D特征独立核验相对位姿。
- 用户确认相机安装未调整、未见松动，后提出腕部松动猜测；进一步确认仅根据偏移推测，尚未看到明显晃动。该假设未证实，不能写成机械故障结论。当前GUI手动、无自动任务、无新运动；继续离线诊断，不放宽4mm或抓取门限。


### 2026-10-09 LA：用户关机，转纯离线排查

- 用户随后补充“我手轻推机械臂腕部发现存在松动，各个关节并不是锁定得很死的状态，是能有一点幅度摆动的”。这更新了先前“仅推测”的事实：已有用户报告的外力下摆动，但尚未独立确认机械故障，也未证明它造成 GEN13 平面偏移。不得把先前假设字段当作用户仍未观察到摆动。
- 在本次关机指令之前完成的 10 秒只读观测，83 份 control_reference 均相同，84 份 accepted_joint_states 中 Joint3 跨度 0.0260776734 rad（约 1.49°）。没有外力时间/力值记录，不能据此认定自行漂移；CONFIRMED 表示此前实测方向响应，不是电机保持力矩测量。该观察器未收到 SDK command，不能把 control_reference 等同本次原始线缆命令。
- 用户最新明确“我先关闭机械臂，你先离线处理看看是不是其他地方导致的这个问题”。从此只读源码和已封存记录，允许离线复算/修复验证；不访问 ROS 或设备，不发使能/失能、运动或重载命令。实际断电状态未主动查询。原对准、epoch 和 GUI 状态全部视为历史，下一次在线工作须有新的明确准备授权；离线工作继续。
- 正确运行模型与实录关节/TF 的一致性已核验；独立桌面 RGB-D 配准因重复文字纹理，跨站训练/留出匹配未满足预设门限，不能据此确认标定错误或机械故障。第二站同站留出帧稳定也不能排除受力后静态偏移。第二站角度超出历史独立标定验证覆盖是适用范围限制，不是要求重标定的结论。
- 当前并行离线排查相机投影/整流、深度滤波历史及支撑平面比较数学；生产源码清单仍为 153，原 4 mm/4° 门限不改。本次用户指令、前后进度索引及只读观测局限保存于 gen13_handoff_analysis/offline_boundary。


### GEN13 纯离线软件原因排查结果

本轮只使用封存记录与源码，未访问 ROS、机械臂或温度，未重载节点；已部署清单 153 个文件逐项 SHA256 均保持。结论不是“抓取已修好”，而是将第二站持续支撑偏差与已修复的时效交接问题分开，并限定哪些软件原因有证据排除。

| 检查项 | 结果及边界 |
|---|---|
| 相机投影、整流、单位与帧绑定 | 59 份几何配置一致，独立重建两映射的 SHA 与录制相符；独立拟合仍 5.790 mm，绕开像素取整的源 SDK 射线仍 5.806 mm。不是扩画布内参或像素取整造成的约 5.7 mm 偏移。相机四源文件另与该次 camera_transition.json 绑定。 |
| 时间滤波跨站历史 | 两次运动的周期元数据均显示区域清空，到位后重新预热；82 项逻辑回归通过。不能据此证明真实深度经过 spatial 后无偏差。 |
| 平面比较和冻结参考 | 514 条状态的原参考逐位不变；初始唯一候选可重建。符号/缩放、换基、边界与凸域检验通过；第二站的字面法向/竖直高度差也大于 4 mm。 |
| 不同桌面采样区域 | 在记录 TF 定义的共同 XY 栅格区域拟合，第二站仍 5.348–5.555 mm。区域变化单独不足以解释偏移；共同栅格不是独立物理同点证据，鲁棒重加权和外推仍有限制。 |
| 反馈时间对应 | 初始和第二站源帧前后各 1 秒的实录关节替换，对同一相机平面的差值最大影响约 0.106 mm，不能解释这次全部偏移。 |
| 编码器量化 | 假定两站各轴独立 ±半个计数，4096 个角点组合最多改变约 1.877 mm；最有利组合将最大局部差降至 3.971 mm。不能解释全部 5.767 mm，但不能排除对 4 mm 边界判定有贡献。半计数居中与极端组合是分析假设，不是实测误差或完整硬件精度上限。 |

尚缺对齐前、spatial 前的原生深度，因此现有记录不能把传感器随视角变化的误差、处理偏差、未被编码器反映的结构形变和物理外参误差唯一分开。用户报告的外力下关节摆动已记录，但不是本次偏移的已证实原因。当前没有依据改标定或放宽 4 mm，更不以本次离线通过代表实物抓取成功。

下一步缺失证据是同源原生深度与最终处理深度的成对记录及对应相机几何/时间戳、实测关节。该诊断尚未启动；保持用户要求的关机/离线边界，收到新的在线准备授权后才处理。原对准和 epoch 不得续用。

完整证据清单、来源 SHA、条件与未解决项见外盘 `gen13_handoff_analysis/offline_audit_summary.json`；各专题原报告与共同区域/量化独立复核完整保留，local_only 不上传、不删除。


### 2026-10-09 LA：上电后 GUI 反馈恢复与用户授权的 4/7 mm 支撑策略

- 用户重新上电后报告滑条无响应，点击“同步当前关节”仍无效。实录启动反馈的 Joint2 约为 15.6°，随后原始反馈稳定在约 −0.7°，被旧基准的突跳审核持续拒收；状态为 `UNCONFIRMED:ENCODER_FEEDBACK_NOT_READY`，缺少新的 accepted feedback 和 `/joint_states`。GUI 的显示同步屏障阻止滑条进入原正使能恢复流程，而旧同步按钮只等待新反馈，形成互相等待。没有据此放宽运行中的突跳检查。
- 根据用户已有正使能授权及本次恢复手动控制请求，现场仅发送一次 `/demonstration Bool(false)`，清除旧反馈基准；未发 joint target、失能或温度查询。epoch 从 `1791611233356494303` 更新为 `1791611444239070487`，收到 10 份真实反馈，Joint2 为 −0.0122718463 rad，各轴采样跨度为零。此时状态是 `PENDING:POSITIVE_ENABLE_REQUESTED`，只证明反馈已恢复，不等于已验证真实运动。用户随后确认滑条恢复，并重新确认已对准目标。回执 `gui_powercycle_recovery/positive_enable_result.json` SHA `4cdeb439c52698e8ba6490bb41e595d24c726fdcbb16f34b83b35a08bba1be57`；恢复前后只读记录完整保留。
- GUI 缺口已离线修复：仅在手动直控、松开滑条、缺新鲜完整反馈且精确状态为 `UNCONFIRMED:ENCODER_FEEDBACK_NOT_READY` 时，用户显式点击同步才请求一次正使能。旧排队目标和恢复目标全部丢弃，必须收到晚于点击及旧 epoch 的新 `/alicia_d/control_reference_epoch`，再收到晚于该 epoch 的完整新鲜实测角，才仅同步显示。重复点击不重复使能；新鲜反馈的普通同步、自动模式、其他状态均不触发该恢复；不重放旧滑条目标。
- GUI 相关 86 项离线测试通过，报告 `verification/gui_powercycle_feedback_recovery/result.json` SHA `647d8845c83f85dd11d3657533cb84598bc62f2d3890d5fc85540125dc496358`。备份在项目 `.runtime/gui_powercycle_feedback_recovery_20261010/source_before.tar`。为保留用户刚完成的对准，当前 GUI 未重启，修复尚未加载到该进程，留待下次 GUI 启动生效；现场恢复来自前述单次正使能，不能写成新 GUI 补丁已实机验证。
- 用户另行明确授权将支撑平面偏移硬上限从 4 mm 调整到 7 mm，超过 4 mm 时记录诊断告警但不单凭该告警终止任务。此新授权覆盖前文“保持 4 mm”的历史状态。感知、观察交接、多视角注册和最终接触前复核的相应偏移策略已同步；显式更严的旧 4 mm 配置仍受支持。RMSE 4 mm、法向 4°、接触重叠 2 mm、身份、时效与碰撞条件未随之放宽，也未改标定。
- 4/7 mm 策略最终去重验证为 **1088 项通过、1 项跳过**，不把重复执行或被替换的旧负例计入通过数。旧“5 mm 配置超过硬界”的负例已改为 7.01 mm 并保留拒绝断言；首轮失败和修正记录完整保留。`verification/support_offset_warning_4mm_hard_7mm/integration_result.json` SHA `8ff449f92fd5dd2bb98ee9499ea75f26b1efddc798bb32f55df21135d918a7f4`，部署清单已准备。本节记录的是离线通过、准备部署状态；实际节点部署和随后抓取结果由后续回执更新，不宣称抓取成功。


### 2026-10-09 LA：4/7 mm 策略部署与 GEN14 首次接触运动

- `support_offset_policy_deployment/result.json` 确认仅重载感知、remote、task 三个计算节点，8 个硬件/GUI/路由节点 PID 与启动 tick 全部保留。新 unit 为 `alicia-observation-measurement-1791612050465387114`，172 源文件 SHA 与准备清单一致。运行 launch 同时覆写感知 hard=0.007、warning=0.004 和 `/grasp_6d/remote/multiview/maximum_support_offset_delta_m=0.007`，避免仅改 YAML 而残留旧 ROS 参数；`live_policy.json` 已只读核实这三项。GUI 离线恢复补丁仍未加载。
- 本轮使用用户“已经对准了目标物”的新姿态与实际“已移开，可以执行”回执，没有复用 GEN13 位置。GEN14 记录为 `grasp_records/20261010T060144.544180Z_94750fef3cae477792d13cdcb9d4ade0`。只完成一次补视，即取得 `CONTACT_EXECUTION_PLAN`：`212cd5c13c08337a1a24a79b`；随后真正移动到预抓取位，未强制返回最初观察位。
- 实际最终失败为 `CONTACT_ENDPOINT_COMPENSATION_FAILED: PREGRASP_HELD_AXES_GUIDE_DOES_NOT_REACH_FIXED_GOAL`：进入 pose index 1 的 `held_axes_approach` 前置计算后拒绝，尚未提交该接近运动、未闭爪。不能把预抓取通过记为实物夹持成功。`command_result.json` 已封存 failure，runner exit 6；GUI 手动已恢复，未失能。
- 另有独立交接缺口：合法接触运动开始后，旧补视 `observation_arrival_retry` 静止标记因关节位移失效，感知将其报成 `station_frame_completion_qualification_failed`，即使该帧仅约 7 ms、参考约 263 ms，均未超龄。该误报与最终接近路径拒绝分别排查，不将两者混称同一个故障。
- 用户随后于 `call_TiCjA6eWi87P8jQbX4BLpIiU:0` 再次回复“已对准，可以继续自动抓取”，新回执在 `realigned_20261010T060701Z/alignment_receipt.json`；当时仅记录新位置，等待交接及接近路径修复，不把此回执记为下一轮已经执行。


### 2026-10-09 LA：GEN14 接近策略预检缺口修复与放近后的新对准

- 用失败日志指定的两条真实源时间戳重放：SDK `1791612246072420004`、accepted `1791612246056488557`，模型 SHA 仍为 `7812d6b69e854ed9f37b4e3cbd813ecd53dab3504065b2d04d287abc6fd852e0`。接近开始时固定目标残差为 **13.8225 mm / 2.78°**，原 guide `[-2, 0, -20, 10, 0, 32]` 预测残差 **7.6643 mm / 2.09°**；拒绝项是位置仍大于原 6 mm，角度、跟随偏差和改善条件均通过。该检查发生在接近轨迹提交之前；日志的 `execution_requested=true` 不是已经执行接近运动的证据。
- 原候选预检验证名义 pregrasp/approach/grasp 路径，但只有 grasp 阶段运行常量跟随偏差预测；实际 approach 采用固定 Joint2/Joint5、其余轴最多 ±32 counts 的另一策略，前置预检没有审核这项可达条件。固定目标 z 分别为 pregrasp 72.285 mm、approach 54.630 mm、grasp 18.168 mm；前两点空间距离 18.103 mm，不能把相机照片中的“已在盒子上方”当作受限接近动作已经可达。
- 14 个固定多初值约束求解及相邻整数 counts 搜索保持锁轴、32 counts、6 mm/5°，未找到合格解；最佳整数结果仍为 7.3858 mm。该有限搜索不证明全局不可达，也没有据此扩大运动预算或解锁弱响应轴。实际大预抓取期间 Joint2/Joint5 分别移动约 −13.27°/+9.14°，只证明该段大运动有响应；随后两轴在零增量命令下各变化 −1 count，不能代替小步高精度响应验证。
- 修复提取共享的纯几何预测函数，保留原 8 轮、2-count 步长的 guide 及原准入。启用 held approach 时，候选预检从名义预抓取终点的线缆量化值和当前实测 SDK−encoder 偏差出发，传播可选的 pose0 补偿，再按实际 pose1 的 6 mm 条件审核；关闭该配置仍沿用原预检分支。预测不创建虚假的未来反馈或源时间戳，缓存必须包含该策略的成功证据，执行时仍检查真实新反馈和完整路径。失败回执增加初始及 guide 残差。用本轮实际执行的名义预抓取终点加移动前偏差作条件回放，pose0 预测 4.799 mm，通过；pose1 预测 7.855 mm，应在运动前拒绝。这不是另行录制的原预检轨迹。
- **284 项受影响离线回归全部通过**，包含新增 14 项和既有 270 项；原执行故障注入、旧成功 held approach、普通预检及任务交接测试保留。源码和测试共 7 个文件冻结，备份位于项目 `.runtime/gen14_held_precheck_20261010/source_before.tar`。交付 `gen14_analysis/held_precheck_delivery.json` SHA `c955a82d04eb4119e6448a428aec5baf121fd55825ac0dadf27f9b601c93d000`；根因与修复报告 `held_approach_failure_and_repair.json` SHA `8bb95aaa7337e25c876e7f3f75914331e25070c9e15edff1269192357756feec`。根智能体已独立核对源码、差异和测试证据，未发现阻断项，未重复运行测试；`held_precheck_independent_review.json` SHA `a358017a7bff2a8dbfc173c788cd3980652fd3e8e87b301e1b22b78e02df54c9`。
- 对该实录条件预测仅计时一次，两次 guide 合计 wall **90.384 ms**、thread CPU **90.374 ms**，约为 15 秒候选预算的 0.60%。计时排除导入、模型初始化、ROS 和 MoveIt 服务调度，不代表完整负载下的最坏耗时；原 deadline 未延长。`held_precheck_timing.json` SHA `c56a36e920dd86b9cba5a63a955bc4055d20a9fe2b4fdc2c3adba4f8a8bf36d1`。其他候选缺少完整预抓取轨迹，不能给出可信的全部候选通过率；46 份稳定评估使用同一距离 profile 也不等于全部受限路径已经证明不可达。
- 用户随后明确“已放近并对准，可以继续抓取”，新回执 `call_LdghCusyPaom3wZD9U6BfDQj:0` 存在 `realigned_20261010T061800Z/alignment_receipt.json`，SHA `1b030df1ad723cc11711b81f96c00990fdc33113f968276bcb5b054249884736`。回执绑定新实测关节和新图像，`previous_alignment_reused=false`，epoch 为 `1791611444239070487`；这取代先前 060701 对准位置，但不代表新一轮已执行。
- 本节截至源码离线通过与独审完成，gateway 尚未由本次记录宣称加载新补丁。合法接触运动触发旧补视静止标记失效的另一条 handoff 修复正在单独收尾，最终测试数量及整合清单以其正式报告为准，本节不预填数字。两路部署、实际节点身份及随后抓取结果由根智能体追加实录；当前仍未证明实物夹持成功。


### 2026-10-09 LA：GEN14 接触交接与空间裁剪参考的离线验证封存

- 本次修复针对合法接触运动仍被旧补视静止标记审核的问题。任务在原严格执行调用之前，按 selection、session、track、epoch、已发布接触计划 ID/摘要/源时间及 contact phase 精确绑定，完成感知与 remote 双方 prepare/retire 确认；随后再次审核真实反馈、控制权和原计划门禁。确认只表示退休旧站位权限的交接，不是控制器已提交或已运动的回执。旧在途计算不得再发布；初始一次性抓取没有合格补视链时沿用原路径。
- 退休时另保留冻结 base footprint、平面和高度作为当前帧 RGB 搜索区域的提议；每帧用当前精确 TF、真实深度重新投影。此上下文不保存历史图像、历史深度、可执行几何或静止时效特权，不设置 `observation_arrival_retry=True`。只在同一既有有限 contact phase/session、MOVE_PREGRASP/APPROACH_TARGET 内使用；阶段、选择、epoch、配置、控制权变化及计算后的取消均撤销。原 0.8 秒是握手期限，未用它续期运动感知；身份、新鲜度、真实支撑、RGB 有效域与碰撞门禁保留。producer 并不直接观察实时 task bound_plan：精确计划绑定加当前 unknown+observe_return 的调用链保证此运动区间不替换计划；重入 PLAN_PREGRASP 会永久清除该参考。
- 最终 23 文件、834 个用例中 **833 通过、1 个已证实的既有失败**，其中 **74 项接触交接相关测试全部通过**，不与旧 737 项结果相加。唯一失败是旧 `test_arrival_spatial_proposal_cannot_relax_original_raw_uniqueness[two_original_matches]` 期待 `arrival_unique_original_identity_required`，实际更早以 `matching_fragment_independent_geometry_required` 拒绝。独立加载修改前的精确 RGB 源码得到完全相同失败，旧测试与断言未改，也未把该失败计为通过。JUnit `spatial_combined_r1.xml` SHA `8d230db470e05b32aa9df26ed0334558c37f90e418bc68cb960eab05045a84fd`；测试/源文件清单 `spatial_combined_r1_manifest.json` SHA `ac103ad6acc1e7a70a14251b1437da7e83abc98a299e6e700fb11f0db9486c03`；原版本对照 `baseline_spatial_old_assertion/result.json` SHA `1ede3f4f559779a4f11cba92a09b74415144dcf62e9348a535745ded339a3db5`。
- 接触运动窗口以最后合格测量初始化，顺序回放其后 **301 对真实 RGB-D/精确 TF/accepted feedback**，最大相邻源间隔 **267.933 ms**；没有插入虚拟新帧。最终为 **60 帧 READY、241 帧 unavailable**：原运动开始后约 +5.65 秒的人工搜索边界失败消失，首个后续拒绝移至 +11.65 秒，源 `1791612206315101623`，为 `metric_ambiguity_reference_changed`。该帧 3 个深度候选中 2 个满足原 metric 匹配，均不贴图像边界；当前独立支撑 ratio 97.28%、4 象限、局部差 3.322 mm，前一合格源仅早 200.241 ms。拒绝发生在旧静站消歧恢复的 pose_invalidated 检查，不能写成超龄或已确认的真实遮挡。此后未自动复活失效目标；没有宣称完整运动阶段视觉跟踪恢复。现有冻结接触计划对感知 unavailable 的执行规则未改，实际 GEN14 最终接近失败仍是前节单独处理的 held-axes 预检问题。
- 最终回放 `sealed_continuous_spatial_roi_replay.json` SHA `4aac418de106465ed4b443e2f5682d7d8d50ce07528ef0089393ce1b5f3c06e5` 绑定全部 9 个生产文件；私有 importlib 感知入口的实际路径另由 `sealed_replay_loader_binding.json` SHA `8c92fa2b4697dec16603c69b29c99b15f6cab5fe6f7b42089373128ebf1a3b48` 补充。原 24 帧 READY 后失败的连续回放、中间首歧义分析及只读碎片探针全部保留；碎片探针没有接入生产，也不代表完整时效审核或实际执行通过。所有报告位于外盘 `ros_log/full_latest_restart_20261010T002347Z/gen14_analysis/contact_station_retirement/`。
- 项目交付 `.runtime/gen14_contact_station_retirement_20261010/delivery_v2.json` SHA `37fa24789a76b45011650fdd0d4f2f2679fe61d7ff7de9fd377a74eae953cba2`，9 个生产文件与 1 个测试文件冻结；旧交付 `delivery_v1.json` 和两次源备份保留。关键源 SHA：`unknown_tabletop_perception.py` 为 `817323e1725854ac3198878ab9878159a0668a6014e622f174282ff001473955`，`contact_station_producer.py` 为 `448680752541d2d149e6fd25d49f6c24dd5a72a5ce68715b91f351908e192736`，`rgb_instance.py` 为 `8abcbbd4cba2b9f4e600d40f952b153d522e6e88c0bcccdce8b26023010cad8c`。本节仅记录离线修复和验证封存，尚未以本节声明任何节点已加载该补丁、实机验证或夹持成功；没有硬件操作、失能或温度查询。实际部署、节点身份和下一轮结果由后续回执追加。


### 2026-10-10 UTC — GEN14 修复实际部署及 GEN15 启动

- 最终整合 `verification/gen14_contact_station_and_held_precheck/integration_result.json` SHA256 `fbdcf95eaf1603e8d15e784f9bada1edfa8e4472590ff100e900d6fd1b574be5`；185 源部署清单 SHA256 `56f476e2bfc3990d0c10d7ede19264b779e99f05d65934654e07fadcf61b842a`。1117 项通过、1 项原源码同因失败的旧断言；不宣称全部测试通过。
- 三计算节点实际重载成功，unit `alicia-observation-measurement-1791615161987832010`；随后空闲网关由原 roslaunch 从 PID 61834 恢复为 151532。驱动 PID 7146、上位机及其余硬件节点保持，控制 epoch 与关节位置核验通过；未发送运动、失能或温度查询。两次部署回执分别为 `gen14_contact_compute_deployment/result.json`、`gen14_contact_gateway_deployment/result.json`，组合运行基线为 `gen14_contact_combined_deployment/result.json`。
- 在线参数仍为支撑局部差硬界 0.007 m、诊断警戒 0.004 m、多视角支撑偏差 0.007 m；原件 `gen14_contact_combined_deployment/live_policy.json`。GUI 掉电同步补丁不在此次部署范围，GUI 未重启。
- 用户实际确认 `call_LdghCusyPaom3wZD9U6BfDQj:0`：“已放近并对准，可以继续抓取”，对应 `realigned_20261010T061800Z/alignment_receipt.json`。以新一轮实测目标启动 generation 15，selection stamp 1791615300095034599，记录 `/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20261010T065458.401325Z_1cbfec9472d144049738a77835f31d16`。新记录包含实际计算参数快照及新增的接触站位退休 request/ack；录制期间未开启触觉、抬升、上传或删除。此段记录时已发出第一补视运动，尚无抓取成功结论。


### 2026-10-10 UTC — GEN15 首补视后候选拒绝，第二补视机会缺口

- GEN15 `20261010T065458.401325Z_1cbfec9472d144049738a77835f31d16` 已以 failure 封存，runner exit 6。首次补视真实执行，到位三帧通过；request 17 接触候选审核全拒，MoveIt/held-precheck `checked_count=0`，未生成 CONTACT 计划、未接近、未闭爪。task 首次 FAILED 源 `1791615350177288532`；随后感知 `station_compute_retirement_context_changed` target_lost 的接收时间 `1791615350191134452`，不能倒置为感知先导致任务失败。GUI 已恢复手动，未失能。首因报告 `gen15_analysis/first_planning_blocker.json` SHA256 `772168b3348251d8d96e9d116215bbaaf391c0aac0848de9830af008797d3da7`。
- 9 个网络返回候选均未通过本地审核；几何生成器有 26 个 proposal、0 个 materialized。内部 54 次缺双侧证据、23 次 `GRIPPER_SWEEP_COLLISION` 不是 77 个独立候选，后 23 次实际 failed_gate 均为 `finger_reach`，不能说发生了实机碰撞。融合已有初始视角与补视来源，不能称未融合。
- 13.008754 mm 执行误差是这次 FAR_FIELD 到位的新鲜 FK 残差（TF/accepted 约 23.385 ms），不是上一轮遗留；对应总 depth_uncertainty 16.008754 mm 用于分段距离和倾斜 sweep，实际 contact overlap 仍为 2 mm。GEN14 同类到位误差为 11.949 mm；2.676768 mm 是独立 handeye 平移误差字段，不能混为一谈。报告 `gen15_analysis/runtime_execution_error_provenance.json` SHA256 `0536d4add2077cc48a02823ffef22db54197cea74a46b840e67e2bb7c9da141a`。
- req17 实测 OBB 为约 70.206×46.640×22.258 mm，精确窄轴 68.01385048° 已采样，含 1 mm 开口余量后约 47.640 mm；66.50692524°约49.470 mm也可容纳，70°约50.045 mm仍不可容纳。不是遗漏精确角度。三个未倾斜共同高度区间仅约0.220–0.267 mm；当前补视机会只消费完全缺侧的拒绝行，遗漏这类仍不够的正高度证据。六个旧拒绝方向全盒开口超过50 mm，故原机会列表为空，没有第二观察请求。正在修复这一机会分类，不改开口、接触2 mm或碰撞门。
- 不能把薄未倾斜高度直接判为所有姿态不可抓：GEN14 合格候选原共同高度也只有0.474 mm，约8.103°倾斜重测后CAD接触重叠5.087 mm、注册后3.018 mm并通过。新补视机会必须位于完整失败的接触搜索之后，不能跳过 learned/tilt 搜索。对比报告 `gen15_analysis/bilateral_evidence_vs_gen14.json` SHA256 `66b5bb49a796c6b11156422e95e8b8b223a791e6e63e30632d1389b3152c6388`。GEN15 旧摘要未保存精确左右点数及profile角度映射，近似点云复核未精确匹配，故这些字段保留 unknown，不冒称完整实录重放。
- 用户新确认 `call_IC3qKmXtVcrk5lAAez4J4RAK:0`：已对准，可以继续自动抓取；实际保存 `realigned_20261010T065926Z/alignment_receipt.json`，epoch `1791611444239070487`。该回执供修复后下一轮使用，不复用已消费的061800回执。


### 2026-10-10 UTC — GEN15 薄接触证据补视修复部署及 GEN16 初始站反馈间隔失败

- 新分支只在完整接触搜索无合格候选后，把实测正但不足的共同高度纳入补视机会；初始直抓及倾斜接触仍先完整搜索。200ms反馈包围、2mm接触重叠、夹爪开口、碰撞、时效、身份和补视预算未放宽。
- 本轮351个唯一测试通过（350通过+1依赖环境setup error，系统NumPy/SciPy对原nodeid复测1通过；32新增包含其中，不累计旧1117）。旧GEN15摘要缺乏profile-angle映射，fixture明确为实录摘要加构造映射，非完整融合点云重放。最终独审无阻断。
- 部署清单192项，SHA eba56221d097128065b2d202d3c972d7a858c772d9ca33e9dc274573f7c7583f。实际3节点及空闲gateway重载回执：`gen15_thin_gap_combined_deployment/result.json`。驱动及其他节点保持；epoch1791611444239070487，4mm警告/7mm硬界保持；不查询温度、不失能。
- 使用用户真实065926对准授权执行GEN16，记录`grasp_records/20261010T071816.396801Z_d424471a68e04c86a96270c4eea0686f`，selection1791616697987946271。已封存失败，未补视运动、未接近或闭爪，实测最大关节位移0。不能声称本轮已验证薄证据分支。
- 首失效source1791616705922231435；前后反馈1791616705856017128/1791616706056025958相距200008830ns，超原200ms限8830ns。图像失败年龄约141.6ms；最近合格参考进入处理时218.4ms、实际失败时341.7ms，非0.8s超龄；两侧实测q一致。initial station getter将包围失败永久写入marker，再由completion检查升级为整代目标丢失。
- GUI已恢复手动；只结束本轮recorder及两只读watcher，不向机器人节点发停止/失能命令。继续修复这处有界缺测恢复；下一轮执行前核实当前实测位置仍与实际用户授权一致，无需将未发生的移动归因给用户。


### 2026-10-10 UTC — GEN16 初始反馈缺测修复部署，GEN17 补视后反馈门禁先撤销

- 初始反馈缺测修复300唯一测试通过；最后task晚到消息断言单例替换原node结果，不记301。4生产文件，独立typed pending不发布几何、不插值、不续原0.8s期限；原200ms/.004rad不变，arrival/contact不增加权限。感知→remote→task真实三层恢复及消息顺序均验证。
- 本人独审补充`verification/gen16_initial_feedback_gap/independent_review.json` SHA4ce0e35e750f99242f05281e68c88587e18701bf16b3f7713438688e3e44815c。报告保存此前遇JUnit路径断言，根代理已据独审明确消息如实代存，未虚构新增审核，已封manifest不改。
- `verification/gen16_initial_feedback_span_pending/deployment_sources.json`196源，SHAf2acdec4b8f6a168d04301ce97be9681a873d26c0df0abb736553b607c677c69；实际三计算节点重载回执`gen16_feedback_gap_compute_deployment/result.json`，unit alicia-observation-measurement-1791618065390414843。保留driver/GUI/gateway，无温度读取/失能。
- 真实20包确认与065926原授权位置相差仅1编码器刻度，使用原授权同位置继续GEN17，没有伪造新用户确认。记录`grasp_records/20261010T074124.732449Z_1d17fd13ff5543a98a2f83fc0fca56f0`，selection1791618086331182003。首补视plan5668b8fdc7d09c6a5d133709执行并三帧qualified；随后尚未接触候选审核、接近或闭爪即结束失败。初始反馈缺测本轮未触发，不能宣称实际恢复分支已验证。
- `gen17_analysis/owner_invalidation_timeline.json`：首remote几何失效source1791618132766993761，station_compute_wait_station_moved_or_feedback_stale；46.749ms后task owner_changed，再157.300ms后producer context_changed。后两者为连锁结果。10份contract binding相同，qualified后37个实测q差至多1.11e-16rad；最新包年龄64.855523ms。无先发生的selection/session/epoch/manual改变。私有缓存未录制，锁饥饿尚需确定性复现，不将其写为已经证明的实机根因。
- recorder已自然封存失败，首次拟仅结束recorder的脚本因PID已不存在而在发送任何信号前退出；随后只结束两个readonly watcher。GUI已恢复手动。
- 用户首次074343确认后的图像仍左下，因此再次明确居中要求；最终实际确认`call_jEN1OLumJ0eOo72HMuALR50V:0`原文“盒子已在画面中央，可以继续”，保存`realigned_20261010T074653Z/alignment_receipt.json`，图像已居中。下一次GEN18仅用此新确认；runner白名单追加此真实原文，未改其他动作规则。


### 2026-10-10 UTC — GEN17 反馈入口独立接收修复部署，GEN18开始

- 本机rospy实际serial receive_callback + 注册MRO callback + 真实stream/geometry锁的确定性旧码反例已复现：持规划锁时新反馈不能入cache，门禁以相同station_moved_or_feedback_stale拒绝。它证明软件阻塞机制，不是完整TCP仿真，也不声称读取了GEN17私有缓存。反例与首次失败保留在`verification/gen17_feedback_ingress/`。
- 三生产文件修复：accepted ingress只用独立短锁保存实测反馈，50ms timer承接原回调重状态审核；同步context/join/ticket/commit仍保留原freshness/200ms/.8s/.004rad和epoch/身份规则。缓存保留往返位移、异常消息与溢出拒绝；先快照后取now避免并发新样本误判future。
- targeted首次115通过/1旧到位恢复失败，原因是新增timer提前对历史回填批次做freshness。恢复原callback检查时机后，旧断言不改，单例及最终10文件547项全部通过，不累加早期定向计数。本人独审`verification/gen17_feedback_ingress/independent_review.json` SHAe2e82445637c79cc8fbedac68f1a6731f29b672b6a3b9d78ff363c4c9e555d5d，无阻断。
- 198源最终manifest `verification/gen17_feedback_ingress_lock/deployment_sources.json` SHAd88bba93a25fa57b622c2e2a1f754556d4e3017920fa0f5465c2d5c880b8a07e，包含runner实际授权字符串追加。部署`gen17_feedback_ingress_compute_deployment/result.json`已通过，unit alicia-observation-measurement-1791619414320889142，仅三计算节点；GUI/driver/gateway保持，无失能/温度命令。
- 执行前20个新实测包与074653真实用户居中授权相差至多1编码器刻度，图像居中；仅核当前位置，不伪造新的用户确认。GEN18记录`grasp_records/20261010T080409.341157Z_434841dea0b347dda4b659c53f63bf33`，selection1791619450953784465，FAR_FIELD plan128792ac62de78fd86794397；已提交首补视运动，结果待监视，不预先记成功。


## 2026-10-10 UTC — GEN18 到位反馈时间包围缺测，已封存失败

- 记录：`grasp_records/20261010T080409.341157Z_434841dea0b347dda4b659c53f63bf33`；首补视三帧审核通过，尚未接近或闭爪。任务在 producer 到位站位审核失败后退出，录制自动封口（不是成功）。GUI 已恢复手动，两个只读观察器已结束，驱动和其他硬件节点未重启。
- 首因：source1791619497059015274 两侧真实接受反馈为1791619497028975282/1791619497255907571，间隔226.932289ms>200ms，两个位置完全相同；当前帧age208.904ms、旧参考age537.657ms，TF等待148.301ms。不是当前帧处理超时，也未见GEN17规划端反馈缓存陈旧告警。
- 下一真实source1791619497456113815有99.916930ms合格包围，但状态到达时旧参考age854.342ms。不能将异常帧作为有效测量，也不能放宽200ms或续旧几何；正在接入已有到位站位的有界身份等待及完整新测量恢复。初始站位GEN16修复本来不覆盖此作用域。
- 原始私有缓存回调顺序未录制，以上双侧反馈来自独立只读观察与录制证据；首因报告：`gen18_analysis/arrival_feedback_bracket_first_failure.json`。
- 最新真实用户确认 `call_ed987d8e51464a9e883a8fd676c6cfbf:0`：已对准，可以继续自动抓取；新位置 `realigned_20261010T081031Z/alignment_receipt.json`，箱体已回到画面中央，此确认尚未执行。旧074653确认已由GEN18消耗。


## 2026-10-10 UTC — GEN18 到位缺测恢复及迟到通知修复已部署，GEN19开始

- 五生产文件：感知节点、feedback_bracket_gap、station_compute_wait_producer/remote/task。已审核到位站位的双侧真实反馈跨度超200ms时拒绝该帧、发布false对象/空mask和明确typed pending，利用此前注册的不可变5秒identity契约等新测量；未通过该帧，未续旧几何或扩大200ms/.8s/.004rad。初始站位、未通过到位审核及CONTACT阶段不借用此权限。
- 新帧须完整支撑/RGB/唯一目标/真实反馈和三topic join，缺测之后即使旧几何仍新鲜也强制完整恢复证据。每个丢弃source均携带独立跨度证据，新完整恢复可以精确封存其迟到通知；真实lost、改变旧READY或把丢弃帧变成非空/detected仍撤销。临提交缺测不部分更新tracker/reference；发布复核时源超龄进入原deadline等待。
- 验证：producer48、consumer26、integration12为中间定向结果；最终11文件组合334个唯一用例全过、0跳过，不与历史547或中间次数累加。首轮producer5失败、consumer首次collection环境失败及旧源码12逻辑失败均保留，不计成功。交叉独审无阻断。
- 冻结202源清单 `verification/gen18_arrival_feedback_gap_combined/deployment_sources.json` SHA98ee96a6d27801c190c03fe8d2fda84574e39b48490f5cbef1b38145c4b6ec37。部署 `gen18_arrival_feedback_gap_compute_deployment/result.json` 成功，unit `alicia-observation-measurement-1791620934063423603`；仅3计算节点，8个保留节点PID和start ticks不变。
- GEN19：`grasp_records/20261010T082926.174495Z_ed2e482a1dbe4e5ab46962660f016697`，selection1791620967862544298，首补视plan51724df1e22cc1f06b5cfbed。沿用真实081031新对准确认，20条最新关节样本差一个编码器计数(.001534rad)、epoch不变、目标居中。首次运动1791620977924167394；抓取尚未成功，待实机结果。


## 2026-10-10 UTC — GEN19近场重提交缺口，已封失败

- 首补视通过三帧双端审核，1791621014.419进入近场重规划；request14在1017.910被TARGET_PROCESSING_PENDING取消。之后至原90秒期限内有323条READY且均带station_compute_wait，但未再出现新请求。新到位feedback gap触发0，不能宣称本轮已实机验证gap恢复。
- 任务在1791621104.424以NEAR_FIELD_DIRECT_TIMEOUT结束；producer first_loss=1791621104.518875，晚于任务终止95.522ms，属于阶段失效后果而非最初故障。源码与日志指向direct单次提交锁存仅对RESULT_EXPIRED释放；本次PREDICT_FAILED/TARGET_PROCESSING_PENDING加soft recovery均未释放，导致poll持续return。正在复现和最小修复，不延长phase或使用旧测量执行。
- 记录082926...ed2e482a已自动封口，GUI恢复手动，两个只读观察器退出；无接近、无闭爪、无实物成功。用户已收到新对准问题，旧081031确认已消耗。首因证据 `gen19_analysis/perception_first_failure_timeline.json`。


## 2026-10-10 UTC — GEN19近场取消重提交修复部署，GEN20开始

- 最小两生产修复：原remote worker将prepare阶段typed TARGET_PROCESSING_PENDING与真正预测故障区分；线程完成后由station专用hook核对完整站位、代次/目标/模式、原phase期限和无发布/验收事实，释放direct submission slot。基类默认hook不放行，acceptance阶段同名取消和几何拒绝不触发重试，不续时、不改source水位、不重放旧snapshot。
- 实录323条READY、322套同源消息均完整；实际私有work certificate未录制，未假称已证明缺证书原因。基线明确模拟允许的proof=None条件并用真实worker复现PREDICT_FAILED及提交占用；补丁新19项和相关旧142项全部通过，共161个唯一用例。早期夹具提交/采集窗口错误及旧码反例保留，不混算成功。
- 独审无阻断。203源清单 `verification/gen19_direct_slot_retry/deployment_sources.json` SHA66fb88b6926874660ba390608bfba82570592d4003b3dcfa18ea4f1f7eaabfaa。部署 `gen19_direct_slot_compute_deployment/result.json` 成功，unit alicia-observation-measurement-1791621937094250632；仅3计算节点，其他8节点身份保持。
- 最新真实对准确认 call_cd3a43d5cfa7462ab7e6645bdf697230:0，回执realigned_20261010T083446Z；执行前20条q与确认值完全相同、epoch不变，目标居中。GEN20记录 `grasp_records/20261010T084609.133186Z_f832fa3e55eb45a59d41fecf2246af73`；selection1791621970669396162，首补视plan b96367b55f89b2ef4b96e4b6，正在验证，未宣称成功。


## 2026-10-10 UTC — GEN20推进到接触候选，补视2 RGB区域未满足

- 实机证实上一修复：1791622018.134 request12以TARGET_PROCESSING_PENDING正确结束，2018.140释放新source采集，随后request13真实接触候选搜索。44硬复核均过，最终43因PREGRASP_HELD_AXES_GUIDE_DOES_NOT_REACH_FIXED_GOAL拒绝，1缺独立SDK采样。不是40mm窄边一律超夹爪。尚未进入预抓取接近/闭爪。
- request14产生新补视计划，2063.927进入补视2运动，2071.992实测settled；2072开始到位重测，2075.015因3秒ACK超时撤销。producer初丢失2075.176是后果。后续独读逐帧首因是rgb_required_spatial_region_unobserved及一帧rgb_search_contains_unobserved_rgb；支撑audit正常(.82–.85、四象限、RMS<1mm)、4–5mm偏移只告警，不扩大3秒或支撑门限。正在核规划/实测RGB覆盖一致性和held-axis接近路径。
- 记录084609...f832fa3e已完整自动封口，GUI手动恢复、只读观察器退出；无抓取成功。结束图像目标右上近边；已请求新居中对准。旧083446确认已使用，不重复执行。

## 2026-10-10 UTC — GEN20补视2参考区域越界与实物可见性区分

- 最新真实确认 `call_2878288377c24d84817cfb9c32b0893f:0`：“已对准，手已移开，可以继续自动抓取”；回执 `realigned_20261010T085151Z/alignment_receipt.json`。此位置尚未执行，下一轮使用该确认，不重复请求已提供的授权。runner仅添加此真实回复字符串，未改变执行条件。
- 用户指出第二补视时盒子仍在RGB-D画面内。根代理复看 `gen20_watch/ended_scene.png`，盒子可见且贴近右上边缘；不能把 `rgb_required_spatial_region_unobserved` 直接表述为“实物目标出了画面”。该检查针对冻结实测足迹和高度组成的参考棱柱投影、首轮可信种子范围及必要背景，校正后的画布也并非每个像素都有真实RGB。
- `gen20_analysis/arrival_projection/planned_actual_projection.json`（SHA afa52c3c15e437e58c760334b61db01638ddfa095d5d51e82db2ed6526e35afe）：相同参考棱柱和RGB有效掩码，规划投影bbox[409,40,613,232]通过既有36px边距及背景检查；13个实际到位源的bbox为[431,1,661,221]，653×489画布右侧越界，+4px必要边界为[427,-3,665,225]，裁内仍含817个无RGB像素。相机端运动学预测与实测关节TF相差13.108mm/2.160°，它不是独立测得的物理标定误差。
- 因此不是GEN25的未接入同一投影检查；本次规划端检查确实执行，但实际到位的参考投影失去边距。同期原图叠框与可见实物轮廓的差异仍在核实，不能将参考范围偏差全归因于机械松动或手眼标定。另一条接触接近路径修复仍在离线求解，未部署或宣称抓取成功。


## 2026-10-10 UTC — 用户明确暂停，GEN21未启动

- 用户指令“先暂停”，三名工作智能体已中断；本阶段未部署新代码、未启动GEN21、未发送机械臂运动或失能命令。现运行版本仍为gen19_direct_slot_compute_deployment。
- 补视构图最小修改：reachable_observation_views增加冻结参考棱柱的图像居中软目标，不改变原准入门限。最终92项通过、0跳过；delivery位于.runtime/gen20_centered_observation_20261010/delivery.json，SHA6699217b71e9c3d9429cf5a26bcae39769707b58b0ddad6de743f7b2909e156c；根最终独审未封口，未部署。实录相对误差扰动仅为回归探针，不是未来可见性保证。
- held pregrasp已找到原门限内可行数值端点；共享选择器、预检与task/gateway入口正在联动，尚未完成全部验证。当前工作树含未交付改动，不能直接重启到此版本后执行。GEN20封存记录及085151用户确认保留；暂停期间不使用该确认启动任务，等待用户唤醒。


## 2026-10-10 UTC — 恢复 GEN20 修复，最终执行时限复核

- 用户已恢复工作；GEN21仍未启动。补视居中最终92项、共享held预抓取选择160项、任务与网关交接326项通过，三个报告无重复，共578个唯一用例。补视与共享选择器已独立复核；网关最终审计发现外层session截止未限制内部60秒规划到提交，正在补齐此边界。578是该追加修补前的通过数，不能当作新修补已验证。
- 补视居中保留原36px、真实RGB有效域及到位审核；实录扰动回放改善不是未来视野保证。held选择及预检保持10mm/5°预抓取、6mm/5°接近、Joint2/5与32count限制，不把只读预检缓存为执行轨迹。实机提交仍从新鲜关节状态规划并审核完整路径。
- 113801确认后，1238读取发现最大约0.10431rad变化和新的epoch1791635727396883411，因此未沿用旧位置启动动作。用户真实确认call_1df45d2d581b45f6abec2e82f405b275:0“已对准，可以继续自动抓取”，保存realigned_20261010T123953Z/alignment_receipt.json；当前盒子居中。该确认尚未执行。
- 五份恢复指针已同步到123953确认；现运行版本仍gen19_direct_slot_compute_deployment，未重启驱动、未查温度、未发送失能命令。本次新增部署辅助脚本仅合并两次成功重载回执，不操作ROS。


## 2026-10-10 UTC — GEN20修复部署，GEN21首到位RGB身份失败

- 最终新补丁三套589唯一用例全过（92+160+337），独审无阻断；221源部署清单verification/gen20_grasp_repairs_combined/deployment_sources.json，SHA9d2a9b0246fc0576670c3eeb8cd9af8c68989f6a6c5c8ad6bb1fc0c5922fb4cc。外层session截止已绑定网关内部规划/提交，False/True不能续期。
- 三计算节点和空闲gateway实际重载通过，统一回执gen20_grasp_repairs_deployment/result.json，SHA270068003c46de3982703d0be81a367bf0f9b46c04557be3496ec546d9c5f833；unit alicia-observation-measurement-1791636414376536272。保留driver、GUI、相机及其他节点；无失能、温度查询或触觉启动。
- GEN21实录grasp_records/20261010T124848.744156Z_718c7a8591494d4bb396505dd64f279e；123953真实授权执行前20包q完全一致、epoch不变。首补视plan9a371cb4b81fe8739f3c8a52，实测运动约34.4秒，1791636583.394进入到位审核，6586.416以ACK_TIMEOUT失败，未预抓取/接近/闭爪。
- 实录14次到位评估中12次arrival_original_rgb_identity_not_matched，2次initial_instance_confirming_1_of_3，未形成连续三帧。首帧支撑通过（ratio97.89%、四象限、平面偏移0.849mm）；本轮不是RGB有效域越界、支撑或新held接近门限故障。具体颜色/轮廓拒绝指标尚在离线复现，不能把timeout或随后target_lost当首因。
- 录制已自然封存失败，GUI恢复手动，只读watcher退出。用户重新居中真实确认call_460644b5a67647568807ee59b7e8af46:0，回执realigned_20261010T125207Z；尚未执行GEN22。


## 2026-10-10 13:25 UTC — GEN21 RGB 种子修复部署与 GEN22 验证启动

GEN21 首补视到位后 14 次审核中 12 次原始 RGB 身份不匹配、2 次仅达到三帧确认的第一帧；支撑面合格，未进入接触预检。实录复现发现最后一张冻结参考中，符合原有颜色/法向/深度证据的可疑侧面硬前景只有 211 像素，低于随整体核心面积变化的 10% 下限，因此未被降为 probable foreground。该条件导致相邻参考帧的硬前景掩膜不一致。

最终改动仅 measured_mask_seeds.py：保留原有足量占比处理；在原规则因占比不足跳过时，仅处理已经满足全部原逐像素证据且 8 连通面积至少 25 像素的区域。本轮对应 197 像素。没有改变身份阈值、裁剪范围、深度数据、接触或碰撞审核。两次中间方案被旧实录回归拒绝，保留记录；最终 294 项唯一测试通过、零跳过。最终实录重放 12 个已保存到位帧从 3 个通过改善为 9 个通过，其中 3 个与线上审核源时间精确对应的已录帧全部通过；其余 3 个仍失败，不将离线结果写成完整实机成功。

部署：gen21_connected_side_seed_compute_deployment/result.json；226 个源码文件清单 SHA256 18b5afe040acccb8ff72e54e3b031d385b3c352781bdfaa3e90de432815d848b。仅重载三个感知/规划/任务消费者，八个硬件/相机/控制/GUI 节点身份保持。证据见 gen21_analysis/verification/combined_r3_manifest.json、side_seed_independent_review.json、gen21_analysis/side_seed_production_replay_r3.json。

以用户 12:52:07 UTC 实际“已对准，可以继续自动抓取”确认继续，部署后 20 个实测反馈样本与确认位置偏差为 0，控制 epoch 相同。GEN22 录制目录 20261010T132545.078718Z_69bac5676950480b80e6cae5cdc4f0ce，selection stamp 1791638746764785051。已实际完成第一补视，到位三帧/双参与者审核约 0.9 秒通过，当前近场重新规划中；截至记录时尚未接近或闭爪，不能认定抓取成功。全程 local_only、未查温度、未失能、未启动触觉节点。


## 2026-10-10 GEN22 结束及第二补视颜色误拒绝的离线证据

本轮第一补视到位三帧及双参与者审核真实通过，随后初次近场候选缺少双侧实测接触证据；重投递推进到第二补视。第二站 12 次线上审核全部报 arrival_original_rgb_identity_not_matched，支撑审核通过，偏移 3.638–4.458 mm；4 mm 警告没有被错误当作 7 mm 硬拒绝。捕获窗口 1791638841662415266 至 1791638844662415266，随后 ACK 超时、任务失败。未接近、未闭爪、未抓取成功。录制已关闭，GUI 已恢复，用户 13:30:29 UTC 重新确认的位置另存 realigned_20261010T133029Z，尚未用于新实验。

已录下的 11 张第二站相邻 RGB-D 真帧均复现颜色距离 0.431–0.435 超过原 0.40、轮廓距离 0.061–0.070 通过；它们不是 12 个线上审核源时间戳的精确帧，不能混称逐帧精确复现。第一站刚 qualified、前一 READY、最终 READY 的真实参考均失败，模板亮度稳定。两视角按实测深度映射的共同桌面 XY 格也呈 L 亮度约 -54、a/b 变化较小，支持实际明暗差异，并不能证明自动曝光或灯光的具体因果。没有发现错误固定使用最初 RGB 或明显串代。

尝试的连续直方图分箱仍无法使 GEN22 通过，未改生产。原 ORB 对应看似足够，但把整个描述子支持域限制在目标内后仅剩 0–3 对，因此拒绝将这份纹理证据用于身份放行。未接线候选归档 .runtime/gen22_arrival_texture_20261010/；生产颜色门限仍为原值。详见 gen22_analysis/arrival_and_planning_timeline.json、arrival_identity_replay_r1.json、appearance_template_and_lab_diagnostic.json、common_top_region_lab_diagnostic.json、orb_complete_descriptor_support_diagnostic.json。

同一轮另发现消费者在旧参考过期时先撤销再接纳已齐新 READY/object/mask 的排序缺口。真实三回调六种顺序可复现；修复只将原完整审核移到旧参考时钟检查之前，缺数据/真实失效/旧帧/控制状态变化仍拒绝，0.8 秒未放宽。相关 222 项回归通过且独立审核无阻断；截至本节记录尚未部署。交付 .runtime/gen22_station_admission_20261010/delivery.json，外盘 consumer_geometry/combined_r1_manifest.json 及 independent_review.json。独立录制器看到新消息齐全不等于规划消费者已收到，报告保留此证据边界，不将四次实机取消都断言为同一排序漏洞。


## 2026-10-10 14:16 UTC — GEN22 消费者排序修复已部署，颜色修复仍在集成

消费者 READY/object/mask 接纳顺序修复已完成实际部署：gen22_station_ready_admission_compute_deployment/result.json，unit alicia-observation-measurement-1791641167276158155。227 源清单 verification/gen22_station_ready_admission/deployment_sources.json，SHA256 07df812c741bde6a40cf0997e98cca104454ea9b0cc42b018222bce1af95708e；最终相关 222 个唯一用例通过，零跳过。仅重载三个计算节点，八个硬件、相机、控制及 GUI 节点身份保持。尚无新一轮实机结果，不把排序缺口的离线验证当作所有现场缺测取消均已解决。

颜色距离为目标区域颜色直方图的差异分数，包含 Lab 明暗，不是毫米或错误概率。保持原 0.40 颜色和轮廓门限；候选只在原颜色失败、形状/色度等约束通过时，以固定 ORB 提议配准后，用双侧真实有效目标掩膜内的独立像素审核纹理及受限明暗变化。ORB 对应本身无身份放行权，不把桌布纹理作为目标证据。纯算法最终 148 项通过，11 张实录邻帧 9 张通过、2 张保留拒绝，77 个挑战负例全部拒绝；这不是 12 张线上源帧的精确重放。正在接入到位三帧确认、每个原始碎片、合并结果及站位重新采集，完整集成未部署。

GEN23 尚未开始，13:30:29 用户真实对准回执仍未消耗。实录第二视角的可见侧壁有大量无深度像素，RGB 身份改善不等于双侧接触证据已经足够，仍需后续实际验证。未改支撑、碰撞、当前测量期限，未查询温度、失能或启动触觉节点。


## 2026-10-10 14:35 UTC — GEN22 颜色集成已部署，GEN23 在运动前会话确认超时

最终补视身份集成 7 个生产文件及 4 个新测试已冻结，17 文件组合 384/384 唯一测试通过、零跳过，运行前后源码一致；首轮缺少跨包测试路径导致 collection error，记录保留且不计通过。最终真实入口重放未注入配准或成功结果，GEN21 保持 9/12、GEN22 为 9/11，连续三帧规则不变。最终独审 appearance_independent_review.json SHA ae99ce0ea2e0a1ec0530b3a5401cd2ce30c9404d78f3924ab0d002e9b94c2bf0 无阻断。236 源清单 verification/gen22_bound_observation_appearance/deployment_sources.json SHA 63a0aab6c228113ff18495778f791b4678dc960c9ab023eb84329e2b4b57087a。实际部署 gen22_bound_appearance_compute_deployment/result.json 成功，unit alicia-observation-measurement-1791642594571339250，仅 3 计算节点重载，8 个保留节点身份不变。

13:30:29 实际用户确认在新部署后 20 个关节样本差异为 0、epoch 不变，目标居中。GEN23 实录 grasp_records/20261010T143205.248405Z_88760530fdb64a3b95102fe0abdc9b68，selection1791642727127860307。1791642751.054 接受观察计划057f4652cdd3e7faa3f94f74；2751.15开始初始观察会话；2756.221以OBSERVE_RETURN_REMOTE_ACK_TIMEOUT失败。无 executing/arrival，实测关节整个观察器期间最大变化0，未接近、闭爪或抓取成功。因此这轮没有实机覆盖新RGB分支。2756.511的OBSERVE_RETURN_TARGET_CHANGED晚于任务撤销，不能写作最初失败原因。同期后台新规划仍在运行，正在核实会话确认与worker锁的交接顺序；记录不足以直接断言具体锁等待。

录制已封失败、runner exit6；GUI手动恢复回执gen23_watch/manual_restored.json，两个精确匹配只读观察器已退出。尚未启动GEN24。源姿态无运动；后续是否沿用既有授权以新的实际关节和控制状态为准，不把历史确认伪写成新用户回复。local_only、未失能、未查温度、未启动触觉。


## 2026-10-10 UTC — 用户重启后的新授权与手动状态核实

用户询问 GEN23 是否仍在手动。只读 keyframes.bag 实证：/gui/joint_direct_mode 在14:32:05.915为true、14:32:06.592767切false，14:32:31.151进入观察交接，14:32:36.221失败，录制内未再切true；根在任务结束后14:34:10.098恢复手动。控制epoch未变、全轮q最大变化0。本轮失败不应归因为GUI持续手动，具体锁等待仍需并发复现与新增阶段诊断。

用户随后自行重启并重新对准，真实回复call_NvXHyOXsd87EJWxDwBewdgyq:0“已重启并对准，可以继续自动抓取”。新回执realigned_20261010T144419Z/alignment_receipt.json，receipt1791643459629570722，epoch1791643371453043998，六轴q=[-1.880660445947796,0.41570879351700873,-0.34054373491061235,-0.0015339807878856412,-0.010737865515199488,0.0030679615757712823]，夹爪0.0498m；新鲜图像目标居中，尚未用于GEN24。旧133029位置不再沿用。runner仅追加上述真实回复的允许字串，仍要求与receipt精确一致，原执行检查不变；该单行改动将纳入GEN23交接修复最终源清单。


## 2026-10-10 UTC — GEN23 会话修复部署、GEN24 失败后用户暂停

GEN23 会话交接修复经 164 个唯一回归用例通过、独审无阻断后部署。回执 gen23_session_handoff_compute_deployment/result.json，unit alicia-observation-measurement-1791644046694369680；240 源清单 verification/gen23_session_handoff_lock/deployment_sources.json，SHA256 8af952556f378b48bcb1c404483f5c1c8d0dfde7c2272d0064577e8e5545936f。只重载三个计算节点，八个硬件、相机、控制及 GUI 节点身份保持。旧锁顺序在离线并发测试中复现阻塞；GEN23 未记录私有锁等待，不能断言实机唯一原因。

GEN24 记录 20261010T145523.713830Z_9463dd99db1449279fafc4fa41b57be4 已封失败。两次补视运动真实执行；首次到位三帧和双端审核约 0.673 秒通过。接触 request29 的 36 个候选均报 nominal contact goal unavoidable target OBB collision(left_finger,right_finger)。只读初步量化：实录目标 OBB 约61.019×41.124×22.601 mm，而被审工具闭合轴上的投影宽度约62.556或65.694 mm，超实际夹爪开口；这不等于用户40 mm窄边不可抓，轴方向与候选生成的代码原因仍待复现、未修复。

第二站到位捕获窗口为1791644246435057163至1791644249435057163。12帧均报 matching_fragment_competitor_not_owned；24次原RGB直方图检查全部通过，颜色距离0.2047–0.2789、轮廓距离0.0697–0.1027，支撑平面差0.787–1.548 mm。本轮明暗纹理恢复分支未触发，不能把这次失败称为颜色门限再次误拒。完整status证据 gen24_analysis/arrival_measurement_reasons.json，SHA256 dcc410265cc90851de131ebe6bf48fc04fff486b681dae715e3eb808bbd5f2ed。任务最终ACK超时为持续审核失败的结果；未接近、未闭爪、无抓取成功。GUI已恢复手动，两个只读观察器退出，runner已退出。

用户真实新确认 call_eddNQOzfZ3LhkvKaCHYR9ahD:0“已对准，可以继续自动抓取”，已保存 realigned_20261010T150336Z/alignment_receipt.json，目标在画面中部，无新试验使用该确认。随后用户明确“先暂停”，三名分析智能体均中断；未部署GEN24新修改、未启动GEN25、未发送运动或失能命令，未查询温度。恢复时先续办碎片归属和接触轴诊断，并以实时位置/控制状态核对这份尚未使用的确认，不在暂停期间执行。所有证据 local_only 保存在SanStand。


## 2026-10-10 洛杉矶 — 按用户要求完整重启，排除电子皮肤

用户要求“重新启动各节点和上位机除了电子皮肤，然后继续处理任务”。现场原ROS/GUI进程已退出，本次使用项目新入口 `.runtime/full_latest_no_skin_20261010.launch` 和对应shell；明确 start_tactile=false，auto_torque_on_startup=true、GUI初始手动、相机15fps、感知6Hz，支撑4mm诊断/7mm硬界、关闭抬升与软件温度轮询。新目录 ros_log/full_latest_no_skin_20261011T000625Z（UTC命名）；根服务 alicia-noskin-core-20261011t000625z、配套本地仿真服务已启动，新运行节点没有 tactile_skin。未发布失能或抓取运动命令。

源码与此前已审核240源完全一致。runtime_receipt.json证实驱动8855、相机8872、GUI8899及全部主节点已运行，真实sdk_measured反馈与RGB帧新鲜，本地仿真health通过；actuation=CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE。全栈重启的计算节点现隶属core，第一次计算热部署须使用 load_observation_measurement_consumers_20261008.py，不能将core误当专属consumer unit停止。新基线 full_stack_deployment.json；旧GEN24记录和分析继续原目录保留。旧carton YOLO节点报告缺torch/ultralytics，本次unknown感知使用独立路径，不将该警告当作unknown已失效。

用户新的真实对准回复 call_17251774a4df46b7930ce7be68652a7c:0 已存 realigned_20261011T000755Z/alignment_receipt.json；图中手位于左侧，随后真实call_2fde366d522d4ff3aac75c59966c9bdf:0确认“手已移开，可以执行”，另存clearance_confirmation.json。这些授权尚未用于试验。继续并行离线修复前轮碎片归属和开爪路径条件不一致，尚无新抓取成功，电子皮肤未启动。所有新日志直写SanStand、local_only。


### 2026-10-10 洛杉矶：GEN24 窄范围碎片恢复部署，新重启 GEN2 验证

- 外盘运行根为 `ros_log/full_latest_no_skin_20261011T000625Z`；电子皮肤未启动，关节失能命令和温度查询均未发送。
- 上一会话 GEN24 第二补视的首个精确失败帧，两个匹配深度片共有 13 处对角邻接，原独立 RGB 分割只有一个完整保留两片可信核心；另一个漏掉暗侧面，触发 `matching_fragment_competitor_not_owned`。该帧原颜色和支撑审核已通过。
- 最终只修改 `tabletop.py` 的可选连接方式与 `arrival_fragment_group.py` 的受限恢复。默认仍为四邻域；只有原独立几何、身份、轮廓一致性审核通过且唯一已有分割完整覆盖各原生核心时，才检查原匹配像素是否形成一个完整的八邻域深度连续组。无新增深度、无非匹配片搭桥、无合并硬前景、无阈值放宽。原本全核心归属成功的路径仍使用原交集。
- 放弃的全局八邻域方案曾导致 GEN22 回放退化，未合入/部署。最终 GEN21 9/12、GEN22 9/11 的每帧结果、确认顺序和参考掩膜与旧版本一致。GEN24 精确首败帧离线进入 1/3 确认，不能算实机三帧到位或抓取成功。
- 最终定向测试 22/22；实际工作区 13 文件组合回归 271 通过、1 跳过（历史本地夹具未分发）。正式报告为新运行根 `verification/gen24_fragment_final_suite_manifest.json`。未叠加历史测试数。独审无阻断，冻结文件和回归绑定于 `verification/gen24_fragment_followup/deployment_sources.json`，SHA256 `d7350760190681228da89d57b3f9b983e3b8f1dbeabc57d751e3f6a3d9aea572`。
- 使用 `load_observation_measurement_consumers_20261008.py` 首次从完整启动中迁出并重载三个计算节点，保持八个驱动/相机/控制/GUI 进程身份不变。部署回执：`gen24_fragment_compute_deployment/result.json`，通过；后续重载应使用该独立单元。
- 沿用本次真实对准和移手确认，20 次新关节反馈相对已确认位置最大差 0.001534 rad，控制代次不变。GEN2 录制记录为 `grasp_records/20261011T003714.176499Z_8cd314e96edb4eba8d04c92b93166af5`。该确认已消费；实机结果待本轮结束后补记，不能把部署或离线回归记为实物抓取成功。


#### 新重启 GEN2 实机结果与接触补偿诊断

- GEN2 实机失败，未接近或闭爪。首补视到位后约 0.626 秒完成三帧与双端确认；三帧均原始颜色直方图通过，未触发 GEN24 新碎片恢复，不能声称新分支已实机覆盖。记录完整关闭，GUI 手动恢复。
- 第一处严格接触失败为 `predicted_command continuous geometry: target OBB collision at 11-end/right_finger`；12 个候选的原 pregrasp/approach 已通过，最后共同耗尽近场阶段时限，终态 `NEAR_FIELD_DIRECT_TIMEOUT`。目标退休发生在终态后 86.2 ms，不能当首因。详细时线 `gen2_analysis/readonly_first_failure_timeline.json`。
- 本轮对象 OBB 窄边投影 39.478648 mm、实际开口 49.8 mm；不能沿用上一 GEN24 倾斜方向投影超宽的解释。当前静止 SDK 减编码器偏差为 [-4,11,12,-5,-1,3] counts，仅施加一次，没有重复补偿证据。原预抓取、接近、接触补偿开关均启用。
- 固定同一目标/起点/模型/偏差的独立局部 IK 曲线重建，通过原连续 CAD 审核复现同一右指拒因；预测实际路径通过，补偿命令路径工具夹轴偏移 8.715 mm，整体位移 13.297 mm。这个数是补偿命令与理想实际路径差，不是已执行后的实测残差。原 MoveIt 失败轨迹没有保存，不能把独立重建称为原曲线逐点重放。
- 原合法高度窗口的 9 个端点样本仍碰右指。横向偏置约 -4.43 mm 的反事实只有 0.127 mm 最小模型间隙，不满足现成候选的中心合同，也未完成接触、完整路径和闭爪审核；未部署。没有把未采样连续空间宣称为绝对无解。
- 实录和只读独审报告在 `gen2_analysis/contact_collision/`、`gen2_analysis/independent_following_semantics.json`。任务仅执行过一次补视，尚未验证第二站的新碎片分支。后续需现成合法候选或新鲜跟随证据，不能删除补偿或碰撞门来获得通过。
- 用户消息 `call_726ac4d26f284100927e998819e85b2d:0` 原文“已经重新调整完毕并且对准了”，新起点保存在 `realigned_20261011T004306Z`，尚未用于下一轮动作。新站静止跟随为 SDK 减编码器 [-3,4,12,1,8,-2] counts，只证明本站，不能冒充未来预抓取实测。


#### 2026-10-11 01:06 UTC — 补视后接触方向轮转部署并启动 GEN3

- GEN2 实录 38 个去重候选中三个方向均有合法候选，但严格检查集中于同一方向及高度重试。现只在 observing_contact/returned_contact 的 direct near-field 路径中，保留原端点 IK 优先层级，在层内按 proposal_source_index 轮转不同夹持方向。同方向内部顺序、全部候选、初始一次性抓取排序、原碰撞门与截止时间保持；高度重试仍原地最多两次，不能声称每个方向必定得到检查。
- 最终 90 项相关回归通过、无跳过；独审 direction_schedule/independent_review.json 无阻断。247 源清单 verification/gen2_contact_direction_followup/deployment_sources.json SHA256 9b13e8b9d0ec21e5900ada8e5bb8965ef7730efe0e0fa1365c4732292032c755。部署 gen2_direction_compute_deployment/result.json 通过，单元 alicia-observation-measurement-1791680473477785148，仅三个计算节点重载，八个保留进程身份不变。
- 用户 00:43:06 的真实重新对准确认在部署后 20 个新反馈样本中关节最大差 0、epoch 不变，画面目标居中。GEN3 记录 grasp_records/20261011T010558.427673Z_e0beb4e762d74e7895fa100cec6eaaba，selection1791680760121573686 已启动；该确认现已消费。首补视会话交接及 executing 双端确认通过，后续结果未定，尚不能记录抓取成功。
- 补偿未关闭，不把命令曲线偏移等同实际执行残差。此前有限端点/IK 搜索没有找到合格解，不等于证明全部抓取路径无解；继续按新鲜实测和完整路径检查验证。电子皮肤未启动，未失能、未查温度，所有记录 local_only。


#### GEN3 结果：方向轮转实证与再次补视快照契约缺口

- GEN3 已封失败、runner exit6，GUI恢复手动，两个精确匹配的只读观察器退出；未接近或闭爪。首补视三帧与双端审核通过。req12 的两个实际方向50°/55°共8个候选全部完成严格审核，方向轮转成功获得跨方向检查，不能记作抓取成功。时线与顺序见 gen3_analysis/readonly_timeline_and_direction_coverage.json（SHA256 6051ee11e89c8194e263c404ec2d02403a159fce5576d0714a99b26fe665fc4a）。
- 本轮对象OBB约63.376×42.617×22.857mm，track1目标在夹轴投影51.6203mm大于49–50mm开口，双腕名义目标均撞指，尚未进入GEN2那种补偿command路径拒绝。另6个被注册后接触高度/支撑余量拒绝；原±2.676768mm平移证据要求未改，不应称标定再次失效。精确对照见 gen3_analysis/request12_contact_geometry_comparison.json（SHA256 5f1ed0a95544a371711214c9e0fba7f6cb517562c1cae3c107f70d522f265a00）。
- req13–20 已实际转再次补视规划，反复TARGET_PROCESSING_PENDING取消；producer在90秒原phase截止前持续READY/tracked，252条合格输出，远端89次暂缺发布。不是任务slot没有重投。终态在1791680895.578088为 TARGET_LOST: station_compute_wait_expired_or_future，首loss也在原phase截止1791680895.522341966之后。
- 源码定位 _station_compute_snapshot_work 固定按near_field_planning_snapshot_frames=3验证，而这些再次补视请求按planning_snapshot_frames=5构造。实际snapshot_evidence均5帧，导致计算凭据缺失、在几何暂时过期时反复取消。正在按真实阶段修复与交叉验证，尚未部署，不提前宣称后续观察成功。
- 用户新真实回复call_vpIpgC2e1FMJIMK4FEaNRWVW:0“已对准，可以继续自动抓取”，保存 realigned_20261011T011019Z；新实测q=[-1.9972429858271046,0.5307573526084318,-0.1303883669702795,-0.07363107781851078,-0.4632621979414636,-0.026077673394055898]，epoch1791677188993077035。目标居中，尚未用于新试验。


#### GEN3 五帧补视交接修复部署，继续核对历史高度修复

- 单生产文件 station_compute_wait_remote.py 按真实收集策略区分 direct_contact 与 supplementary_observation 的配置帧数，凭据绑定目的、阶段、配置及精确源帧集合。未改任何碰撞、支撑、接触证据要求和原绝对时限；暂缺只保留未发布计算，发布前仍需当前完整实测。
- 旧码回归 baseline_r2 确认真实五帧已成功提交而 work_certificate=False。新暂存4文件组合131通过，集成后互不重复6文件361通过，共492唯一通过、零跳过。暂存结果按逐字节一致的生产文件映射，69个同包导入源码一致，原始日志/XML保留；未伪称重复运行。独审 independent_review.json SHA d19f225a77b6fd5738d9f731a057e6d3d62cd079e6143531edf4c748aad8aaf4 无阻断。
- 248源清单 verification/gen3_observation_snapshot_followup/deployment_sources.json SHA107cac2f5b442da53cd4cc93694e07e97348f36995cdf64acfcc73113cb4aaf8；实际部署 gen3_observation_snapshot_compute_deployment/result.json通过，unit alicia-observation-measurement-1791681658836091327，保留硬件/相机/GUI节点。尚无GEN4实机结果。
- 用户强调应找正向窄边抓取路径，并指出旧高度测量问题曾修复。2mm要求是双侧实测共同区间与CAD指腹的最长连续交集，最终沿实际工具+Z接近轴；桌面法向共同实测范围是此前另一项约束，不应混称。GEN3窄方向分支最大1.606mm是某倾角分支的共同重叠，不是左侧高度；对向分支0.0356mm同理。垂直0°也参加过候选检查，但未保存其单独精确重叠，不能捏造数值或据有限证据判物理不可竖直抓。
- 当前正核实历史约10mm高度漂移的模型/手眼配套修复与当前补视融合路径；已找到实际用户夹持确认原件 ros_log/grasp_close_hold_20261006T061242Z/operator_confirmed_result.json。新011019对准确认仍未使用，未开启新一轮动作。


#### 2026-10-11 01:25 UTC — 历史高度核对与 GEN4 开始

- 历史证据索引 gen3_analysis/historical_one_shot_height_index.json SHA c7284165f350498629b251da5d928ddb47a1531cf659af70547fd88a4ccf6589，直接绑定9/27支撑/择高修复、9/29与10/6的initial_contact用户实物夹持确认，未把几何通过当实物成功。
- 当前object_geometry.py相对b2fdac56无变化；原双侧测量/连续指腹交集算法仍在，113.2mm最小支撑范围及整bbox排除由共用入口执行。补视融合C/C逆坐标转换与只扩不缩OBB链未发现确定性回归。GEN3物体高22.857mm，0.874mm是指定夹轴的共同测量带；两者不可混称。未保存全部1567个fused点及私有成员关系，不能据现有证据精确排除视角或融合采样影响。
- 新011019实际用户确认在部署后20个反馈样本最大差0.001534rad，epoch不变、目标居中。GEN4实际录制 grasp_records/20261011T012545.114130Z_eeaa272415b840acbb66eb2c6bd76625，selection1791681946724086761，已开始首次补视；该对准确认已消费，结果尚未得出。


#### GEN4 结果与视野跟随误差缺口（2026-10-11 UTC）

- GEN4已封失败并恢复GUI手动、两个只读观察器退出。未接近、未闭爪。req13五帧补视计算耗时9.912s完成PREVIEW_READY，第二补视实际执行；两站均三帧与双端确认通过。这是GEN3帧数修复实际跨阶段验证，不是抓取成功。
- 首终因1791682023.806032：OBSERVE_RETURN_SUPPLEMENTAL_FAILED: OBSERVATION_CAMERA_WORKING_VOLUME_INVALID: target OBB leaves required camera image margin。第二站qualified在2023.221713，晚4.727s的感知身份loss为后续事件。时线报告 gen4_analysis/readonly_first_failure_timeline.json SHA5d8cc524fb630fe1821922d1d4313f324006c68cb6adab1de761b729e20e89e1。
- 首站req12十二个stable候选高度余量均过原±2.676768mm，三档±6.233411/±2.800184/±2.768887mm；7个去重候选严格拒因均为名义夹指-OBB终点碰撞。不能将本轮继续表述为高度测量失败。
- 同一冻结OBB、653×489内参和手眼，用计划相机投影margin59.540px，strict命令FK59.518px，到位前后精确TF31.140px，低于原36px约4.860px；实际图中目标完整可见。真实执行轨迹终点与预检strict_goal完全一致，未换IK。实测末端与计划差9.333mm/1.853°，J2最大差0.025796rad仍在原0.035rad到位范围。报告 gen4_analysis/planned_vs_measured_observation_projection.json SHA37119519d253bb3c4c83e8362983607fd229264ef623f725395c280ed06493da。
- 规划、gateway和task使用同投影公式和有效门限，但规划只验命令FK；跟随包络用于碰撞/支撑，没有纳入相机视野。三帧审核使用目标prism/有效RGB与身份/支撑，与全OBB36px边距不是同一证据，不能凭qualified删除后验门。源码报告 gen4_analysis/observation_working_volume_contract_review.json SHA80a0cbe9744e880437e0f69aecb68f1b34c87e99771c570ee1d1616bd6ae68dc。
- 下一步只在计算侧观察候选和strict端点中按原关节到位容差+slack构造误差盒、保守投影视野，保持36px及其他门；尚在暂存设计，未部署，也未找到已审完整新抓取路径。
- 用户新call_9935c387b1f449629cc5e7ce0f45a198:0已对准确认，实际回执realigned_20261011T013141Z；epoch未变，新q=[-1.9941750242513336,0.48013598660820567,-0.02454369260617026,-0.11811652066719437,-0.6043884304269426,0.039883500485026674]，尚未用于GEN5。图像目标居中、左侧桌边外可见手；下一次运动前仍以当时现场图像和现有授权上下文判断，不把历史照片作为当前无手证明。


#### GEN4 后续：补观察到位纠偏、观察边距与预抓取精度偏好

- 用户指出观察位也应像预抓取位补偿误差，并要求放松硬门限1.5倍。本次明确实施范围为补观察图像边距36→24px；初始/接触通用相机边距仍36px，全部碰撞、真实关节限位、身份/支撑/新鲜度审核保持。规划、网关、任务后验及RGB获取使用同一独立观察参数，避免配置读法再次分叉。GEN4相同几何31.140px在新观察边距下可通过，但不等于后续抓取路径通过。
- 原观察流程没有主动端点补偿，GEN4驱动也没有执行trim；不能以预抓取补偿开关推断观察运动已经纠偏。新增独立观察纠偏服务，在原观察动作成功并静止后、三帧到位审核开始前，按当前实测FK检查冻结目标视野。合格则不运动；仅边距不足可在同一观察会话/截止/目标下尝试一次SDK−实测偏差补偿，需完整命令路径、预测实际路径、碰撞与视野审核。执行后以新鲜真实反馈和新的静止窗口开启原三帧审核。没有部署全关节误差盒的保守投影草稿。
- 用户要求未知预抓取即使允许约10mm也尽量补准。现在保留10mm硬验收，增加6mm优化偏好；6–10mm内尝试原有限搜索，改善到7mm也可接受。若提交前找不到合格改善，须复核当前真实反馈、归属、硬验收后才能保持原位；提交后失败、原位碰撞、归属变化不得转换为成功。未修改接近/闭爪碰撞要求。
- 三组件集成前已逐字节备份到项目.runtime/gen4_integration_backup_1791684288692696627；本次只更新protocol-v3-upgrade工作树，不覆盖用户根工作树驱动。首轮32文件联合回归1044通过、1项首次晚到回执夹具失败：夹具已收到回执却只清空内容，保留水位而再投同时间戳，被生产防复活规则正确拒绝。仅修测试为真正尚未投递的首次晚到，生产水位规则保持；原失败记录保留，最终验证待后文。

- 最终32文件联合回归1045/1045通过、零跳过，combined_r2保留完整XML/日志和278源运行前后哈希。另对新增观察补偿的44项重复用例绑定既有模型及两份历史实测夹具，dependency_binding通过、281源运行前后哈希一致；唯一通过数仍1045，不累计重复用例。

- 281源最终部署清单 verification/gen4_compensation_followup/deployment_sources.json，SHA256 24259ad1a1e4fe8d393d353554e5e693f6ba03a87af186727da3a3803e3ca017。gen4_observation_compensation_deployment/result.json部署通过：仅空闲motion_gateway及三计算节点重载，驱动/相机/GUI等保留进程身份及控制epoch保持，未发运动、失能或温度命令。新服务与24px观察门限、6mm预抓取偏好参数已加载；尚无GEN5实机结果。


#### GEN5 实机结果：新增纠偏入口错误重用静止目标准入时效

- GEN5记录 grasp_records/20261011T021311.497945Z_cd0a3abaa490426ca7844719e18c92b8；selection1791684793017119169。首次补视动作实际执行，编码器最大变化约0.964874rad；进入correcting observation endpoint的7ms内结束，尚未接近或闭爪。
- 首终因是 OBSERVATION_MEASUREMENT_FAILED: TARGET_STALE: live target observation age31.914s is outside[0,8.000]s。新补偿服务之前的task原调用链重新检查了运动前目标的普通静止准入时效，约32秒动作后必然不满足。这是新增交接遗漏；并非本轮相机流停顿、RGB颜色或支撑面再失败，也不能说预抓取6mm优化已经实机改善。
- 原1045回归没有覆盖完整真实invoke/validate/drift链的32秒交接，早前新增handoff测试模拟了该调用边界，漏掉此问题。现补真实链复现与负例，拟只在同一执行中观察纠偏的精确上下文里区分历史来源年龄，保留默认8秒及完整原身份/漂移检查、最终新鲜三帧。尚未部署后续修复。
- 本轮runner exit6、记录关闭；GUI手动恢复，两个精确匹配GEN5只读观察器已结束，未发失能和温度命令。用户新call_2bb6af032bb94cb69104d299f45960df:0“已对准，可以继续自动抓取”保存 realigned_20261011T021652Z，当前目标居中，epoch1791677188993077035。新确认未消费，待修复后使用；不复用GEN5旧起点。


#### GEN5 交接时效修复正式部署

- 真实ModeAware调用链离线复现31.914秒旧测量被原8秒准入检查拒绝。修复只给原executing lease内的专用补观察纠偏建立临时上下文；默认年龄审核保持，来源须在该视角提交前已有效，保留原冻结计划、身份、几何与中心漂移检查。等待服务期间换lease、手动接管、session/epoch变更、显式矛盾均拒绝，调用后撤销临时范围。
- 2生产文件、2测试文件已逐字节备份并集成；最终实际工作区194/194通过，零跳过，完整真实调用链新回归37项。独审gen5_analysis/independent_review.json无阻断。283源清单 verification/gen5_compensation_handoff_followup/deployment_sources.json SHA256 fd735d286a023b475617c4995a10e7c656d24f05d7371659bbbb3acdab6376e0。
- gen5_handoff_compute_deployment/result.json部署通过，unit alicia-observation-measurement-1791686041368473426；只重载3计算节点，网关、驱动、相机、GUI等保持。未发运动、失能或温度命令。此前中断发生在下一轮位置只读确认时，GEN6尚未启动，021652真实对准确认仍未消费。


#### GEN6 结果与普通补视搜索覆盖遗漏（2026-10-11 UTC）

- GEN6记录 grasp_records/20261011T023740.243842Z_0e7a6e5935a847a5ae5fcb14772f78a7 已封失败，GUI恢复手动，观察器退出；未接近或闭爪。首补视及纠偏入口通过，实际视野40.161px，纠偏服务返回no_motion，随后新三帧和双端qualified通过。GEN5交接时效修复获得实机验证；24px放宽、真实纠偏运动及预抓取6mm偏好尚未实机触发。
- req12接触搜索39角度中16缺双侧证据、23产生提案，22被夹爪扫掠/单侧25mm范围拒绝；剩余35°共同实测接触带8.610–9.045mm仅0.435mm，小于2mm。严格接触路径尚未开始，不能说本轮又发生预抓取路径碰撞或证明40mm窄边物体不可抓。
- req13共有220观察候选，3.006秒初始严格预算只检查4个tilt45°/roll−15°、188–190mm距离的相近候选，均完整路径失败；其余216未证不可达。旧距离/方向coverage只在not information_driven诊断分支启用，普通信息驱动补视被漏掉；且任一正向端点IK提示会关闭旧coverage，提示后完整路径失败仍无覆盖恢复。正在同时修这两种入口，保留原预算、全部候选和严格门。
- producer于1791686326.537发布UNKNOWN_OBSERVATION_NO_SAFE_VIEW/OBSERVATION_SEARCH_INCOMPLETE，task于6326.688误报NEAR_FIELD_PLAN_PHASE_INVALID。另修任务已存在的终止码白名单使首因原样传递；不把原因显示修复称为路径成功。时线 gen6_analysis/observation_handoff_and_terminal_timeline.json，SHA256 db1183979a84b02c33a9e2f978227c50fea6d42bad43a9594f4e9463f93fbd6a。
- 用户新真实确认call_e2aced1c023d4fb48e3b8fdd9b5d6e48:0已保存realigned_20261011T024307Z，epoch1791677188993077035，目标居中、当前图像未见手；尚未消费。所有新修复仍离线暂存，当前部署仍gen5_handoff_compute_deployment，未失能、未查温度、未启动电子皮肤。


#### GEN6 搜索与原因透传修复部署

- 普通信息驱动补视在获得完整strict/following/current-ticket合格路径前按距离/方向覆盖；正向IK提示最多先试一个，完整路径失败仍继续覆盖。保留支撑证据分级、零信息尾部、全部候选、原预算及碰撞门。GEN6真实220候选经生产dispatcher回放前4为285/186.978/383.022/190.046mm；200充分支撑先于20未知。
- 同时将6个实际producer终止码纳入原无效终止预览解析，保留原最小来源时间窗/模式/无效标识检查；未增执行权限。实际WORK联合15文件474/474通过、零跳过，源前后不变，两份独审无阻断。测试调整4.1mm旧负例至7.1mm只核既有7mm政策；视野缓存测试改用已部署的独立观察参数。
- 部署清单verification/gen6_search_terminal_followup/deployment_sources.json，293源，SHA256 cee4096723f917e46c8e3e31a6ee07ded75bd4f584050f5fa0f26721a7aed6e1。gen6_search_terminal_compute_deployment/result.json通过，unit alicia-observation-measurement-1791687291423940083；仅3计算节点重载，8个保留进程身份一致。024307用户确认经部署后20个样本最大关节差0、epoch不变、目标居中，准备GEN7；尚无实机成功结果。


#### GEN7 实际初始规划失败：动态调度与外层重复消费回归

- GEN7记录grasp_records/20261011T025609.070289Z_3fef5d33e78d4f78a762599c62ab76e3，selection1791687370626879453，runner exit3，已封failure。未开始任务运动、未接近或闭爪，观察器最大关节变化0；GUI已恢复手动。runner结束后精确SIGINT只读recorder，避免尚无GraspState终态的录制空等600秒，动作进程未发信号；全记录保留。
- 终止是UNKNOWN_OBSERVATION_NO_SAFE_VIEW: observation roll lattice has no MoveIt result。116候选、2个正向IK提示、dispatch记录116，但strict检查0且variant_results为空；并非预算不足或证明不可达。新普通补视返回动态generator，现有mode_aware_remote_grasp6d的FK-first先min遍历后next重遍历，将其提前耗尽；fallback族平衡同样先统计再重遍历。此次改动引入真实接口回归，作者、root和独审之前均漏检该外层组合。474通过只覆盖直接dispatcher与无FK的strict入口，不能据此宣称完整集成正确。
- 根因报告gen7_analysis/generator_wrapper_consumption_audit.json，SHA256 e1d5fdc83abd55b01d6aaf90f28f2ad69ea62d641e6cb61b832d22768a92efc8。正在给动态调度增加明确的静态候选检查接口，保留真实迭代时的完整incumbent切换；修所有FK-first/家族组合消费者及实际外层strict回归，不简单tuple提前消耗动态状态。
- 024307真实对准回执已用于GEN7，但机械臂无运动且姿态未变；后续经新鲜实测延续同一确认即可，不再次要求用户重对准。当前新修复尚未部署。


#### GEN7 外层候选消费修复正式部署

- 新ObservationCheckSchedule以initial_order提供静态检查快照，仅真实next更新dispatch并按完整qualified incumbent切回原评分；FK-first及族平衡不再消费活迭代器。修四个调用面，保留支撑分层、原门限/预算。空循环明确记录0/N未完成，不再使用all([])误报完成。
- 旧FK测试缺camera_target_distance_m，未真正触发动态调度；已增强原测试，在未改源上确切复现失败，并补真实外层回放GEN7全部116候选（4FK、2正向IK提示）。新受控路径回放实际4checked/112unchecked，不把其当实机路径可达。实际WORK最终12文件194/194通过、无跳过，源前后一致，独审gen7_analysis/schedule_handoff_independent_review.json无阻断。
- 295源部署清单verification/gen7_schedule_followup/deployment_sources.json，SHA256 edc5392b0195220213941f6a2b45929aa018e39870666994bd86d76e5f16030d。gen7_schedule_compute_deployment/result.json通过，unit alicia-observation-measurement-1791688036910749205，只3计算节点重载。部署后20反馈样本相对024307对准最大差0、epoch不变，当前图像居中且未见手，可延续现有授权进行GEN8，尚无新实机结果。


#### GEN8 实机结果与第二站小幅漂移（2026-10-11 UTC）

- GEN8记录grasp_records/20261011T030838.360708Z_7ed6dc99da194052acdd3b32de45532a，selection1791688120029028892，已封failure/runner exit6，GUI手动恢复，两只读观察器退出。两次实际补视和新三帧审核通过，未接近或闭爪。初始request2从228候选首FK即完整通过（203.20mm/23.52°/roll0°，路径14.107s），第二补视request6从222候选首FK即通过（201.68mm/15.25°/roll0°，路径5.394s）。两次外层路径均不再零检查；但首FK就成功，fallback分散coverage未实机触发，不能宣称其已实机验证。
- 首补视后request5的24个tabletop提案均因超单侧25mm可达范围拒绝；另9个窄向角度单侧23.53–24.99mm，但缺双侧接触证据。9个GraspNet提案6个stage_profile不可用、3个缺双侧证据。没有进入严格接触路径，不能将本轮描述为所有接触路径碰撞或物体不可抓。
- 第二站参考确为本次q=[−2.11075756,−.61819426,1.27013609,−.17027187,−1.43427204,.01687379]，两次观察纠偏均no_motion。第二站source8188.412/8188.678/8188.877（UTC epoch179168前缀）三帧通过；约9秒后J6由.01687379缓慢变至.01073787，1791688198501070048偏差.006135923rad超过原.004静止站位边界，其余5轴未变。不是旧站参考复用。
- 同刻8198.503感知报perception_failure:'observation_arrival_retry'，8198.541 remote以station_compute_wait_feedback_moved撤销几何，8198.562task以station_task_pose_changed失败。感知本帧处理237.7ms/帧龄291.2ms，并非超时。已离线用真实关节回调复现：超过静止范围后marker被正确删除，但scope资格检查后再次硬索引造成KeyError，掩盖规范站位撤销原因；实机无栈，精确哪条硬索引不能据日志独断。报告gen8_analysis/search_contact_and_arrival_marker_race.json SHA9cf9ccdf51b855f65a4a1875b4d9837376f9c6dd9417cb29c46639743c12f3b5。
- 正隔离修复marker撤销竞态，同时只读评估同一会话原deadline内重新静止/重新测量恢复路径，未放大.004或跳过原身份/碰撞审核。用户新call_cd2c2ca150a945888d4569b887f93eeb:0确认保存realigned_20261011T031157Z，receipt1791688318528699159，epoch不变，目标居中未见手；新确认未消费。


#### GEN8 漂移后恢复方案（离线实现中，尚未部署）

- rosbag证明第二站到失败期间GUI模式始终false、epoch不变、无新joint_commands/actiongoal，驱动92条相同目标J6=.0122718463rad；实测从arrival .0168737887变化。首次越界 .006135923rad对应tool0原点平移约.0022mm、相机.5022mm、转动.3515625°；失败后139ms累计.010737866rad对应相机.8789mm/.6152344°。不能仅凭这些判断硬件松动，亦不能以工具原点平移小免除重测。
- 采用同view最多一次纯测量恢复的隔离方案，保留.004站位原合同。触发必须是同selection/session/view/epoch、idle PLAN_PREGRASP、原qualified观测lease且仅实测pose-drift。立即废弃旧几何/待处理计划，三端各自验证本地parent与真实drift witness，然后新measurement lease、全新静止窗口、原三新帧/身份/registration后才允许新计划。绝不发送运动/补偿、绝不复活manual/epoch/目标身份hard撤销、原identity/session/phase绝对deadline不续，恢复次数不能因新phase而重置。
- 此方案尚在stage .runtime/gen8_station_requalification_20261011实现和跨节点反例审查；marker并发撤销独立修复也未部署。当前实机仍gen7_schedule_compute_deployment，最新031157用户确认未消费。


#### GEN8 站位漂移重测修复部署（2026-10-11 UTC）

- 9生产文件、12测试文件已冻结、备份并集成；实际工作树20文件911/911通过、零跳过，306源运行前后一致。独审station_requalification_final_review.json SHA e9b9a0e60e4d6ef47cfa53d5b8bc4ff0a3968a19d0d341553c4532dfa10c834d无阻断，最后75独立回归通过；不累计重复通过数。
- 同站最多一次重新静止与三新帧恢复，旧计划立即撤销、三端须各自持有已登记parent和真实SDK反馈；原固定恢复截止覆盖首次新phase完整配准，不延长原phase/session。阶段截止改为整数纳秒，保留同站既有实测信息增益；普通近场/抓取路径回归覆盖。修正arrival marker并发撤销KeyError。
- 独审还复现真实producer process(None)发无图像源pending而remote提前return、旧票仍可用的缺口。最终修复把只撤权入口放在图像源检查之前：不造RGB-D/TF来源、不认通知为本地SDK、不发ACK或新lease；真实target_lost/first_loss仍硬撤销。真实三端与无帧跨节点回归覆盖。
- 部署清单verification/gen8_station_requalification_followup/deployment_sources.json SHA 98bcf29831320c054b63b024d1ea7cca7243ef6aac6279efae2dbf191e6de9c9；gen8_station_requalification_compute_deployment/result.json通过，unit alicia-observation-measurement-1791695118410115643，仅三计算节点重载，硬件/相机/上位机进程保持，未发运动/失能/温度命令。
- 暂停后当前关节与031157旧对准相差最大0.243903rad，epoch已更新；不沿用旧确认。用户新call_c51d34d84a894602bb3b65f5ecbd1fda:0确认已对准，回执realigned_20261011T050540Z，epoch1791694455560621129。GEN9尚未开始，实机成功待验证。


#### GEN9 纠偏提交 XML-RPC 类型错误与修复部署（2026-10-11 UTC）

- GEN9记录grasp_records/20261011T050744.862188Z_7a4c1ea32d4348e095927672be500a64，selection1791695267403706550，封failure/exit6。已完成两次补视运动；第一站三新帧通过。第二站名义视野不足触发一次纠偏，预测误差由9.935mm改善至0.256mm，预测视野边距27.994px超过24px，完整命令/预测路径审核通过。实际补偿提交前终止于execute cached plan exception: int exceeds XML-RPC limits; bounded exact joint probe；补偿本身未提交，未接近/闭爪。
- 根因为observation_compensation_gateway.py before_execute将epoch_ns=1791694455560621129以int写参数服务器，超XML-RPC 32位范围。旧测试epoch1e9且set_param直接更新dict，漏了实际传输边界；同类接触/预抓取入口已有str。最小修复只把参数存储值转为十进制字符串，内部合同/回执仍为int。真实服务与XMLRPC编码回归覆盖32位最大/超界/实机值、最多一次消费、序列化/写入/写成但回执丢失时零额外动作；旧源复现保留。
- 1生产、2测试备份集成，实际WORK4文件61/61通过，307源前后不变；独审gen9_analysis/observation_xmlrpc_final_review.json SHA35e3d7db11a5c61f26bf5f298f59066628bbe8b501e219996b2fa9e95237ebed无阻断。清单verification/gen9_observation_xmlrpc_followup/deployment_sources.json SHA5e8a464738a6d4645266df3966dc6936715546e07219da9ed8c5427b95431c99。
- gen9_xmlrpc_gateway_reload/result.json通过，仅idle motion_gateway重载；其他10节点、epoch、位置保持。合成实际基线gen9_xmlrpc_deployment/result.json SHAb8f01c5f32a9c775429673af1cae26fa1722a6350bc3b070d8db3d91ebf9088e，三计算unit仍1791695118410115643。未发部署运动/失能/温度命令。
- 接触req23尚未进strict；最窄48.97度所需开口45.77mm<50mm，但融合实测共同高度0.422mm<2mm；9个窄向范围0.207–0.666mm，整体OBB高22.86mm，不能说物体不可抓或整体高度算错。req24正确承接第二补视，1/162首FK通过。
- 用户新call_db38c728ea014f2583a8104300126e2a:0对准保存realigned_20261011T051153Z；20新关节样本相对确认差0，epoch保持；新call_3c7d2fe525c14ed3b3183e44ce5d6ca2:0确认手臂已移开。GEN10录制grasp_records/20261011T051847.578728Z_a67795f9372b4a95a6fc41021eb8fa0d已启动，结果尚未得出。


#### GEN10 动作前代次重置失败（离线修复中）

- GEN10记录grasp_records/20261011T051847.578728Z_a67795f9372b4a95a6fc41021eb8fa0d，selection1791695929288217306，runner返回2；未调用GRASP_START_CALL、未执行任何观察/纠偏/接近/闭爪。录制器原等600秒，确认runner已退出后仅SIGINT精确只读recorder pid115753，保留early_recording_close.json，完整bag关闭并封failure；两个只读观察器已退出。
- 参数服务器selection已gen10，但latched selection仍gen9；router/status为error:remote reset acknowledgement mismatch，切换到报错约12ms。实机原ACK全文未记录，不声称已直接读到旧ACK字段。真实本机rospy.MasterProxy+ParamServerCache离线证实：get_param_cached订阅旧值后，普通get_param也会返回旧缓存，直接master getParam才读新值。缓存调用在GEN7基线已存在，并非GEN8重测补丁新增；本次只重载gateway，复用compute时暴露交接竞态。
- 正隔离修复remote/task reset事务的权威读取及同代快返/最终commit复核；保留热路径缓存、精确ACK、执行中拒绝，未部署此新修复。当前实机仍gen9_xmlrpc_deployment，GEN9整数修复尚未在自动纠偏实机触发。
- GUI手动已恢复，原051153确认及移手授权可在当前位置保持后延续，尚无需用户重新对准。下一轮需新选择代次及完整新目标测量，不复用GEN9失败目标。


#### GEN10 启动交接修复集成与补视方向缺陷（尚未重载）

- 重置事务以Master直接读取绕过真实rospy缓存，覆盖同代ready快返、worker清空后及最终提交；低频request_plan(True)/start_streaming及回包前也核对权威代次，热路径保留缓存。修正base将未启动误报already-running的响应窗口，保留执行中拒绝、精确ACK、消息水位。
- launcher保留原8秒等待，要求精确参数、双端ACK后latched selection和同代waiting_for_target/ready三者一致后才调用一次runner。错误直接退出，不增加目标检测等待或自动动作重试。真实router延迟remote/task ACK的旧入口均可复现抢跑；新交接测试通过。
- 4生产/5测试逐字节备份集成，实际WORK7文件264/264通过、零跳过，运行前后源哈希不变；两份独审gen10_analysis/{selection_reset_cache_final_review,launcher_selection_commit_final_review}.json均无阻断。尚未重载，实机仍GEN9 XMLRPC部署。
- 并行发现补视方向判断先取共同高度裁剪后的两侧mask，可能把完整侧截薄后选成薄弱侧。真实helper合成对照：负侧原始0.540mm、正侧20.800mm，裁后0.492/0.400mm，旧missing_side_signs误选+1。共同接触高度不足的拒绝本身正确；正在隔离只修观察方向判断，保留原接触共同裁剪与2mm门。GEN9保存当前帧支持一侧主要顶边、另一侧跨度较大，但完整融合原点未保存，不能声称该实机融合已重放证实。
- 初始已有合格接触候选的直抓路径在该补视helper之前返回。当前无自动任务、GUI手动、GEN10未运动，051153用户确认可经新鲜实测保持后延续。


#### 启动交接与补视薄弱侧修复正式部署；GEN11开始

- 6生产、8测试/夹具共14文件已备份集成；实际WORK12文件666/666通过、零跳过，315源前后哈希一致。三份独审无阻断，部署清单 verification/gen10_selection_and_direction_followup/deployment_sources.json SHA ae103b1612181ca64980812c23cf8cce9cad74bb598c2ecfc0db2546836b133e。
- gripper_geometry共享测量增加独立侧面mask副本，旧接触4元返回保持；observe_return_visual仅用独立侧范围判断补视方向。120组独立输入counts/bounds.hex/mask bytes全部等价，原27项直接抓取preflight护栏通过；2mm及其他接触/碰撞门不变。
- 仅3计算节点和idle gateway重载，联合gen10_selection_direction_deployment/result.json SHA 6013cfa505a4cd1cd2cb697e07d4b6596f84d8200fef8999c426a82d751da449，compute unit1791697699892757023。硬件/相机/GUI进程保持；部署没有运动、失能或温度命令。
- 部署后20新鲜反馈相对051153对准最大差0，epoch不变，当前目标居中、手已移至桌外。沿用已授权真实确认，未伪造新用户回复。GEN11记录20261011T054952.100454Z_2ef11eeda11e441391d0577b78903b19，selection1791697793905648708；已收到完整selection commit，CANDIDATE_COMPUTATION_START success=True，旧动作前错误本轮未复发。尚未得出物理抓取结果。


#### GEN11 结果：双侧信息补足，剩余卡点为接触路径

- GEN11 record20261011T054952.100454Z_2ef11eeda11e441391d0577b78903b19已封failure，runner exit6；两次补视实际运动及新三帧/双端到位审核均通过，未进入抓取预抓取位、接近或闭爪。观察端点纠偏为no_motion，GEN9大整数提交修复尚未实际触发补偿运动。
- 新side_measurement_basis=before_common_support_intersection已进真实审计。初始最窄64.518度所需开口39.249mm，负侧84点/1.829mm、正侧152点/8.926mm，首补视针对负侧。首补视后63.486度负侧105点/1.801mm、正侧203点/10.988mm，第二补视仍针对负侧，新预测增益3.2mm。两次首FK完整路径通过。
- 首补视后req22共同高度0.216/1.805mm仍低于2mm；第二补视后req30多个方向已达约5.8–10.7mm，32个tabletop候选物化，进入接触strict。检查34/64次：17次PREGRASP_HELD_AXES_GUIDE_DOES_NOT_REACH_FIXED_GOAL、16次nominal contact goal左右夹指与目标OBB碰撞、1次原deadline到期；剩30未检查不视为已拒绝。首终因NEAR_FIELD_DIRECT_TIMEOUT，不能再称为支撑/颜色/统一接触高度不足。
- 旧GEN20 held-pregrasp endpoint重选仍在调用，4个held/lookahead开关均True，补偿未删除；但selector抛错被外层转MOVEIT_CHECK_ERROR丢失精确残差。另发现异常证据自引用成环，正在离线保留原失败快照并修结构化诊断，同时重放实参以定位搜索不足，未放宽门限。
- GUI恢复手动，两个只读观察器精确退出；后续支撑丢失发生在任务退出/手动恢复之后，不作为GEN11首因。用户call_fae4e393ef1c415287f59e16611c9b12:0答“正在调整，先继续修复”；旧起点不再沿用，下一轮须用户调整完成的新确认。驱动/使能保持，无温度读取/反应、无失能命令。

#### GEN11 离线路径诊断与搜索反例（尚未部署）

- 诊断补丁 `.runtime/gen11_held_pregrasp_diagnostics_20261011/delivery.json` 已冻结并集成到工作树，交付 SHA256 `6ac75b2aa0dffdb4a85cd484c8825247bd0e32f06800794f1ace632c4d077a88`。保留异常原快照以消除循环引用，在实际 pregrasp callback 保存原名义关节终点、固定目标、每次求解和量化失败残差；外层 `MOVEIT_CHECK_ERROR`、失败阶段和重试分类不变。作者相关 174 项通过，独审 70 项通过，独审报告 SHA256 `91f8734eb60630b887049ab44f668ab453949bccf77f1d73dcf03fb19127eec6`。仅诊断完整性修复，不代表路径已可执行，运行节点仍为 GEN10 selection/direction 部署。
- GEN11 req30 保存的真实场景、固定四姿态、SDK/编码器 sample 和 17 份拒绝已提取至 `gen11_analysis/contact_strict_replay_inputs/`；原 MoveIt 名义预抓取终点未记录，后续 IK 重建必须明确是代理输入，不能声称精确重放原路径。
- 当前约 20.694 mm 的轴向预抓取—接近间距已是既有最小 5 mm 加实测观察端点残差 15.694 mm。该残差来自第二补视实际到位，不是未来预抓取补偿后的 10 mm 验收值，不能预先扣除尚未发生的补偿。两个阶段共同向外移 5/10/15 mm 符合原距离规则；三个代理候选共 12 次保持原 selector/predictor 的比较均未通过 6 mm 接近门，不能作为已验证解决方案。
- 联合小邻域发现搜索漏解反例：拒绝回执数组 index13（plan `a844fac7ec9c7821728dd5a9`，req30 stable evaluation32，倾角 7.217°）代理输入、内部求解目标 99%/99% 时，量化预抓取残差 9.9042 mm/4.9275°，原逐轴 guide 接近残差 6.0819 mm；在同一起点附近两轴共同调整，原 32 counts/固定 Joint2、Joint5/10 mm、6 mm、5°硬门内存在通过样例（如 guide `[6,0,-20,32,0,20]` 得 5.7846 mm/4.9970°）。这仅证明有界逐轴启发式会漏解；尚未完成完整路径与碰撞审核，也不能据此声称实物抓取成功。
- 新搜索实现仍在独立 staging；同时检查实际预抓取 6 mm 优先补偿是否改变后续接近起点，避免仅修纯预测器而遗漏执行交接。用户仍在手动调整，未启动任何新实机运动。

- 后续离线补充：30 个合格邻域 guide 的原夹爪/目标/支撑连续几何审核均通过（构造线性 SDK 曲线及恒定跟随偏差预测曲线）；报告 `gen11_analysis/qualified_guide_linear_geometry_review.json` SHA256 `b10296f9b1c4815d242b49e37f1eb4ec27600c36aa07b1236654057f9f22d47f`。范围仅 Link6/7/8 与冻结目标/支撑，未验证全臂 MoveIt、自碰、其他环境物体或接近到抓取段。新 selector 自行求得 `[6,0,-19,32,0,20]`，无需植入关节目标；预抓取 9.904172 mm/4.927481°，接近 5.913361 mm/4.936428°。实际服务回调离线证实优先补偿无有效改善后通过第二份新鲜反馈保持起点，再以相同 guide 接近；传输/硬件响应为模拟，不能当真实到位。


#### GEN11 接触搜索及诊断修复正式部署

- 三个生产文件与五个测试/夹具已备份集成。实际工作树18文件最终495/495通过、零跳过，321源运行前后不变；初轮12项仅因缺少测试模块路径导入失败，保留原日志，补充测试环境路径后完整重跑，不删除/跳过用例。组合独审60/60通过、无阻塞，报告 `gen11_analysis/held_guide_combined_final_review.json` SHA `f6976549dae137751504c0194344a9d9c75caa0e5e6936a12d09bf01b2f57e80`。
- 原两次局部优化保持优先，仅全失败后增加一次单位归一化后备；量化后仍通过相同10/6mm、5°及32counts原门。共用guide仅在原坐标搜索不足时检查最多96个两关节联合邻域，不改变已合格guide及接触merit策略。有限搜索失败不作为全局不可达结论。原绝对deadline、完整路径/碰撞与真实到位检查保持。
- 清单 `verification/gen11_contact_path_followup/deployment_sources.json` SHA `40e275af1086cd32840a5bf00b4bc2be4f8ed83f61628e3964a07eafe7bca3b6`；三个计算节点及空闲运动网关已重载。联合 `gen11_contact_path_deployment/result.json` SHA `19681f2caa406f1c2c256987c0d40e99c8f0fffac54b2c70fdaab92df9822236`，compute unit `alicia-observation-measurement-1791700007586760289`。驱动/相机/GUI进程保持，部署没有运动、失能或温度命令。
- 用户真实新确认 `call_eaf21df1c2f34adfa4423802f4d14f6f:0`：已对准，手已移开，可以继续自动抓取；保存 `realigned_20261011T062527Z/alignment_receipt.json`，receipt1791699927703931331，epoch1791694455560621129。原GEN11起点不复用；下一轮使用此次新位置，尚无新实机抓取结果。


#### GEN12 首补视后候选尺寸拒绝（继续离线定位）

- 记录 `grasp_records/20261011T063121.405208Z_782732915abe4e989167ce437e3355aa`，selection12/stamp1791700283094620704，已封failure/runner exit6。首次补视实际运动、新三帧与双端审核通过，纠偏no_motion；未接近、未闭爪，尚未调用新held-pregrasp搜索。首终因 `NEAR_FIELD_NO_HARD_SAFE_CANDIDATE`，不是本轮已测得接近残差超限。
- req19的10个tabletop提案均先在finger_reach拒绝，最小单侧需26.356mm超过现有物理25mm；learned另有双侧证据不足、倾角profile及支撑净空拒绝。记录物高22.716mm，请求最终OBB66.34×49.11×22.72mm，初始56.48×38.46×20.93mm。最初将请求2379点误称纯当前帧，已由下述原始RGB-D重建纠正：它是当前raw与已注册历史的精确并集；完整fusion来源成员仍不能只从并集反推。
- 任务1791700330.909释放槽；GUI于1791700347727094173恢复手动。只读观察器PID135585/135843精确SIGINT退出，最大关节变化0.863631rad；手动恢复后的支撑/目标失效不作为首因。
- 用户新确认 `call_cc843d8aaf134d3aaf10f6784fa958f7:0` 保存 `realigned_20261011T063359Z/alignment_receipt.json`，receipt1791700440597983121，已对准、手已移开；未用于新试验。继续离线处理，实际部署仍GEN11 contact_path 321源/495回归版本。


#### GEN12 原始数据复放确认横向对称扩框（修复 staging，尚未部署）

- 从关闭的 keyframes.bag 重建 req19 原3帧snapshot，mask SHA、8867有效target点及原几何中心/axes逐值相同；voxel按实际1.5mm。raw860点与审计2379点前缀逐点字节一致，原raw OBB为50.032815×34.938588×20.791982mm。最终2379=raw860+fusion1606−重复87，不能把2379称单帧点。原输入/脚本/源码清单 `gen12_analysis/raw_geometry_and_reach_manifest.json` SHA `f0a024c39d931ab6163799210eefb531f1c97bd80167f199ed199db4871bb895`。
- 旧enclose固定raw中心，按每轴最大绝对距离双侧对称外扩。保留原raw框8角点和全部2379点，在支撑平面内仅横向重定中心，框可由66.342700×49.105020×22.716437收紧为58.305512×43.980926×22.716437mm；局部中心移(-4.018594,-2.562047,0)mm，约4.766mm；axes/支撑平面/Z中心及Z外包络、点集字节均不改。不裁点、不按百分位缩框、不放宽夹爪物理开口。
- 原10方向reach逐项复算与原审计误差≤6.94e-18m。在新框中心重新生成提案后，其中40°、45°两方向单侧由拒绝变为24.438/23.658mm，满足原25mm；其余8方向仍拒绝。`gen12_analysis/recorded_enclosure_reach_replay.json` SHA `ec59dd6726ef959a22c0ecbb04ac6eebe0b101fd5da2225ff0fcfc8b1c6c85da`。必须重建接触证据、plan ID及整条路径；不能把旧冻结contact pose换框后直接执行。
- 独立真实包含性复核 `gen12_analysis/independent_real_enclosure_replay.json` SHA `b9742c21cd9abbd03c7dfbc1cf8c2f9a4ceec253d4b17f8e7f7e82521abba37c`。完整union的单view生成代理仍提示共同接触高度约1.0–1.51mm，未证明直接可抓；它不能冒充原fusion1606完整来源/votes重放。精确证据只主张恢复2/10原方向的几何reach检查。
- 同时定位审计交接丢失：_activate_prepared_geometry的_clear_geometry_cache会清空_latest_fused_scene_audit，旧PreparedPrediction未冻结该字段，使最终记录fused_scene_geometry=null。staging补source/identity/center/axes/size/point摘要一致的request-bound深冻结，RPC前捕获，成功activation后恢复，两报告入口显式取同prepared。诊断缺失不授权运动或阻断原有效几何；11个实际prepare→remote覆写→activation→audit离线回归通过。
- 原始直接/initial_contact每次seed单视角，不进入多视角enclosure；真实ModeAware入口回归2项通过。新helper仍在组合验证，尚未重载计算节点或开始GEN13，GUI手动保持，063359用户新对准确认仍未消费。


#### GEN12 多视角包围框及请求审计修复部署

- 2生产+8测试/fixture共10文件已备份集成。实际WORK8测试文件665/665通过、无跳过，330源运行前后一致；665为本次唯一测试数，不叠加作者或独审重复运行。交付SHA39dcc30e7b7072a6967531d03de286890379d6c94cdcf47831682d1bf72f55bf。两份独审分别84/84与11/11、均无阻塞，报告gen12_analysis/fused_enclosure_final_review.json与request_fused_scene_audit_independent_review_r2.json。
- 清单verification/gen12_enclosure_followup/deployment_sources.json SHA e2a87b52a03a1c0d054dda15478dd51a6d196ea59347f543ae21851fe852e05f；gen12_enclosure_compute_deployment/result.json通过，unit alicia-observation-measurement-1791701796889247292。仅三计算节点重载，驱动/网关/相机/上位机保持，无部署运动、失能、温度命令。
- 20份新鲜关节样本相对063359用户确认最大偏差0，epoch不变，目标居中。画面左侧可见手后用户真实call_yGn8lFwpw2TFWb2F69aakv5H:0回复“手已移开，可以执行”，原姿态授权延续，下一轮GEN13将重新生成所有目标/候选/路径。


#### GEN13 审计接口真实子类回归（必须修复，尚未抓取）

- 记录grasp_records/20261011T065841.384502Z_1cf5e42efa8d4fd99d1f6d72f95bd771，selection13/stamp1791701923196300745，failure/exit6。一次补视运动、三新帧1791701961946966886/1791701962212332725/1791701962479270935与双端审核通过；endpoint纠偏仍no_motion。
- 首因1791701965.546267 req13 ACCEPT_FAILED：base _run_candidate_gate_audit调用self._multiview_surface_audit(prepared)，但真实ModeAware override仍无参，TypeError takes1 positionalargument but2given。root新增诊断接口漏了该子类；作者、root和独立665/84/11回归均没有走真实ModeAware报告调用链，不能把那些测试通过当完整集成保证。此次失败由新增改动引入，未进入接触预抓取/接近/闭爪，也不能据此评价新包络的实机候选效果。
- 任务1965.645失败，1965.651释放；感知1965.680 station_invalidated发生在任务终止之后，是后续撤权不是首因。GUI于1791701992850415945恢复手动，两只读watcher PID144268/144353精确SIGINT退出，最大q变化0.8344855486rad；机械臂驱动/使能保持。
- 接口最小修复将保留旧无参虚方法，新增request audit包装入口，先调用真实派生方法保留mode字段，再合入冻结请求证据。补真实ModeAware selected/rejected报告入口回归，不仅测base helper；实现/独审进行中。
- 用户真实新call_m3dSIkCkFzFJK4Go8Jjuyv8a:0确认“已对准，手已移开，可以继续自动抓取”；回执realigned_20261011T070159Z/alignment_receipt.json，receipt1791702119992466926，未消费。下一轮用新起点，不复用063359位置。


#### GEN13 实际ModeAware审计接口兼容修复部署 / GEN14开始

- 最小修复1生产+2测试，保持原无参动态hook，新request helper先调用真实ModeAware hook保留mode审计字段，再覆盖同prepared冻结融合信息。两实际selected/rejected报告入口均走新helper，不改包围框/接触/运动门。
- 旧源8个实际ModeAware组合全复现同TypeError，新源通过；实际WORK相关540/540、零跳过、331源前后一致，独审21/21无阻塞。交付f060ed537a73c10bd77d483230132f7248457c94a1137b57d7b5fa2c8c15737c，独审gen13_analysis/request_multiview_audit_final_review.json SHA d944c51c0943fb5571e481b8b7abce6f8e4ed86e073e28b747ff69be0f93e2a1。
- 清单verification/gen13_audit_api_followup/deployment_sources.json SHA e7fd24efec353005b4dbcaa58f15ca7916d514ea4c964e0faeff328003fd2d06。gen13_audit_api_compute_deployment/result.json通过，unit alicia-observation-measurement-1791702525978299513，仅三计算重载，8硬件/相机/网关/GUI保持。
- 070159用户新确认后20反馈样本最大差0，epoch不变，目标居中，最新图像手已移开；同姿态延续授权开始GEN14 record20261011T070957.732968Z_f55879ef6b7140c3a5b3ebe7ec00e0d7。尚未确认实物抓取。


#### GEN14 包围框修复实机生效，第二补视未找到首条完整路径

- 记录20261011T070957.732968Z_f55879ef6b7140c3a5b3ebe7ec00e0d7，selection14/stamp1791702599603058338，failure/exit6。完成一次补视/到位三帧，未接触预抓取/接近/闭爪。GEN13 TypeError不再出现。
- req15审计baf025f…真实保存schema2 tangent_enclosure：875raw+1578fused−34duplicates=2419点，raw框53.671×37.774×20.419→保全部点新框66.275×43.415×21.359mm，支撑面XY中心移(+.132,−1.980)mm，法向中心不改。req16接收到4个missing-side方向61.618/60/63.309/65°；窄向所需开口44.415mm<50，negative有效高度.764mm、positive10.475mm仍不足双侧2mm，所以进入第二补视。不是全体候选因宽度被拒。
- req16当前4/220 strict距离/tilt/roll为285mm/45°/0°、187.462mm/45°/−15°、382.455mm/28.090°/0°、189.162mm/45°/−15°；旧分散覆盖排序确实运行，未回退到同距重复转腕。FK2.0004s仅1seed无候选，IKhint3.0041s查72无positive。4次MoveIt check_pose_strict execute=False均Unable to sample any valid states for goal tree/TIMED_OUT，约.838822/.840566/.862992/.917785s合3.460165s，触发局部初始3s/6次预算，未开启有incumbent后的8s比较。216未查不代表不可达；此错误尚不能区分IK、碰撞或取样约束原因。
- 1791702666.817任务首终因UNKNOWN_OBSERVATION_NO_SAFE_VIEW / OBSERVATION_SEARCH_INCOMPLETE，.830释放；之后station/targetlost均为撤销后果。GUI1791702718135353803恢复手动，只读watcher147897/147969退出，最大q变化.9449321653rad。当前离线分析真实候选与IK目标状态，暂不盲目扩大预算或重复运动。
- 用户真实新call_7ae60436545a4deaad69adaab3c36ebe:0确认已对准、手已移开；回执realigned_20261011T071421Z/alignment_receipt.json，receipt1791702861791254043，epoch不变，未用于新试验。运行部署仍gen13_audit_api_compute_deployment331源/540最终相关回归。


### 2026-10-11 10:31–10:50 UTC：GEN14 第二补视只读定位

- 未开始 GEN15，未发布运动、失能或温度查询。用户新的对准与清场回复已记录在 `realigned_20261011T103628Z/alignment_receipt.json`，旧 071421 姿态已因 0.1534 rad 位移和 epoch 变化失效。
- 220 个原补视目标以原记录关节 seed 在当前 MoveIt 上做有界只读 IK 对照（25 ms、碰撞筛选开/关），440 次均返回 -31；精选 12 个用 150 ms 复核仍未找到解。原记录 q 的 FK→IK 基线在两种模式均成功。结果只反映限时求解，不证明全分支不可达；碰撞场景为当前场景而非重建历史场景。证据 `gen14_analysis/live_ik_all_1042/result.json`、`live_ik_selected_1040/result.json`。
- 离线局部 IK 中 220 个候选的 210 个触及 Joint5 下限 -1.57 rad。未修改模型或关节边界。
- 已整合（尚未重载）hint 查询覆盖修复：同 72 次查询由全 45° 改为覆盖 27.606° / 28.090° / 45°。1 生产文件、2 测试、1 实录 fixture；实际 WORK 94/94 通过，source unchanged。该修复不改变候选位姿，也未解决原 220 个姿态无 IK 的问题。
- 原 FK 辅助分支已用录制深度、mask、TF、141094 个支撑点及到位 prism/RGB 有效区精确重建。49.8 mm 开口的首次复放在原 2 s 预算下得到一个通过全部端点门的候选；该候选只读 IK 及当前终点碰撞审核成功。生产 FK 使用 50 mm 开口，正在统一该参数复核，尚不以首次复放冒充完整生产参数验证；尚无完整路径或抓取成功结论。
- 当前在线模型 SHA `7812d6b69e854ed9f37b4e3cbd813ecd53dab3504065b2d04d287abc6fd852e0`。Joint1–6 rad 限位分别为 [-2.16,2.16]、[-1.57,1.57]、[-0.5,2.35619]、[-3.14,3.14]、[-1.57,1.5]、[-3.14,3.14]。


#### GEN14 搜索修复部署与 GEN15 第二站交接首因（2026-10-11 UTC）

- 两项修复已正式部署：端点提示在既定预算内覆盖不同距离/方向；FK 搜索保留在原两秒预算内提前通过全部端点条件的候选。实际工作树组合回归 216/216，通过独立复核；清单 `verification/gen14_observation_search_sealed/deployment_sources.json` SHA256 `88b20d39f23aa45ea4251b0c09ca5e063fada1c9c49899ba047d1d7e253ba715`，337 源。`gen14_observation_search_compute_deployment/result.json` 已确认三个计算节点重载，八个硬件/GUI 进程保持。未调整关节限位、接触证据或完整路径门限。
- GEN15 记录 `grasp_records/20261011T110808.411096Z_6fb1906474c24e59b6be584da92c6f43`，selection15/stamp1791716890012068271，最终 failure/exit6。首站 FK 在 0.6631 秒生成四个候选，第一个严格路径即通过并实际完成补视；到位、双端测量审核和融合通过，增加 41 个超出 3mm 噪声范围的新体素。狭窄方向缺侧独立接触高度约 1.284–1.309mm，未达到原有双侧 2mm 要求，所以需要第二站。未进行接触预抓取、接近或闭爪。
- 第二站 req10 内部生成一个端点合格 FK 候选，但生成种子时间 1791716949.198060573 之后约 0.281 秒，J1 反馈从 -2.095417756 变为 -2.096951737rad，恰为一个编码器计数约 0.00153398rad。外层精确相等比较因此满足 `inputs_changed` 拒绝条件，候选未进入严格路径检查；原审计缓存未保存此交接拒因。实际检查的四条均是普通姿态格点，共 3.350276 秒，均无可用目标关节状态。其余 218 条未查，不能推断所有路径不可达，也不能称 FK 候选路径失败。首终因是 `OBSERVATION_SEARCH_INCOMPLETE`，目标撤销发生在结束之后。
- 证据 `gen15_analysis/root_cause.json` SHA256 `b3faab933c35edd952f20831041e295d8d037c8ec3b3eda2f66e23280076489b`；交叉时序 `gen15_analysis/req10_strict_and_fk_cache_timeline.json` SHA256 `20181994dc5562f908cf9d17a36f34ab7b6ba932b3cf34221fa4c62ab6d901a2`。因原日志未持久化 inputs_changed 字符串，该首因是源码条件与真实反馈时序共同支持，不冒充原始日志直接记录。
- GUI 已恢复手动，两个只读观察器已精确关闭。用户新确认 `call_140e6cc97bca4382bd79adce17e2f8aa:0` 为“已对准，手已移开，可以继续自动抓取”，保存 `realigned_20261011T111704Z/alignment_receipt.json`；此次确认尚未用于自动运动。下一步修复种子量化交接与拒因持久化，完整路径仍从新鲜实测状态审核，修复通过后继续自动抓取。


#### GEN15 单计数交接修复部署

- 单生产方法修复已集成：在原始种子每轴一个 SDK 计数（2π/4096rad）加机器舍入量内保留无执行权限的 FK 提案；缓存不滚动基准、不续两秒预算。较大位移不能复用旧候选，控制权/epoch/session/来源失效在同一 prepared 上保持撤销。原冻结观察基线、端点、最新实测起点的完整路径审核保留，没有新增 latest-q 位移硬门。拒因同步保存至缓存审计。
- 实际 GEN15 SDK 时间序列重放：旧方法1失败/2通过，修复3/3；旧代码唯一失败为实录单计数交接被丢弃，strict 碰撞负例仍拒绝。原被丢弃终点未录下，测试用明确标注的独立场景端点检查传输，不能称原完整轨迹重放。最终工作树组合257/257、零跳过，全部源前后不变；两份独审无阻塞。
- delivery SHA `2179cca985b7f14832a16b3f95e9eab2a801722b387619d19c9c45bf7f332a7e`；341源清单 `verification/gen15_seed_handoff_sealed/deployment_sources.json` SHA `2d42712b642348bd9156ec5b8b0749026fd9373d31e423eeac488a738f39b0e2`。三个计算节点已重载，unit `alicia-observation-measurement-1791718518265219619`，记录 `gen15_seed_handoff_compute_deployment/result.json`；八个驱动/硬件/GUI节点保持。源码回归通过不代表实机抓取成功。


#### GEN16 实机结果：已到接触完整路径，补偿命令曲线碰撞

- 记录 `grasp_records/20261011T113712.131215Z_6bfa27d92a2944fbb1b445f2c8ebdd6f`，selection16/stamp1791718633818063497，已封 failure/exit6。首站生成四个 FK 候选，实际选 gap1/tier2（预测新增侧面6.2mm）、distance205.524mm、tilt28.7034°、roll0，第一条strict通过后完成运动和到位审核。该候选来自正常求解器收敛而非提前退出；handoff六轴delta为0，本轮未实机触发onecount容错，不扩大验证结论。
- 首补视后req13/source1791718687000939369进入接触规划。8份含高度重试的完整路径失败回执中，一部分共同接触高度约2.00005/2.12899/2.65624mm，已经满足2mm要求；pregrasp/approach被审核，grasp阶段的预测补偿命令曲线均在left_finger/目标OBB相交，不能概括为所有候选接触面不足。另一些前置提案不能容纳±2.676768mm支撑法向平移不确定性，这是不同拒因。
- 1791718774.755085 首终因 `NEAR_FIELD_DIRECT_TIMEOUT`，近场原阶段deadline耗尽；未进行第二补视、接触预抓取、接近或闭爪。补偿仍存在：从预测实际approach起点规划grasp，再逐点应用当前SDK−encoder偏差，两条曲线分别审核；当前首因发生在补偿命令曲线，不删除其碰撞检查。下一步离线核终端开口净空及已有候选覆盖。
- `gen16_analysis/fk_provenance_and_contact_terminal.json` SHA `22ee9894dc4fc7abe551acd8b814c30449b7f19c9255b8639e76582913aa73db`。GUI手动已恢复，两个只读观察器PID183427/183502精确退出，root所有运行sessions已关闭。用户重新对准确认 `call_0d494f8fd39e4e9f8901d3bbdc073600:0`，已保存 `realigned_20261011T114201Z/alignment_receipt.json`，尚未用于下一轮运动。


#### GEN16 离线候选覆盖与恢复设计（未部署）

- 8份strict失败实测SDK/encoder回执均与closed bag逐字段相同；无被拒绝的contact轨迹原件，当前proxy IK探索明确不冒充原轨迹重放。`gen16_analysis/contact_path_replay_inputs/req13_rejected_contact_inputs.json` SHA `715d57c05dba69eb5bc8714540034dd83fd3e286dafbb6df231b40ef61fe40c2`，closed bag核对SHA `148e9ed78a1ed8dbfd1b798dbc38a1e08546efa79c69e71f34f8cae1c132e5c7`。
- 34个既有姿态在相同registration下的代理精确IK端点：nominal清、补偿command均碰撞；原jaw方向±5mm/0.5mm离散平移均无双曲线同时清的正例。10代表姿态×12确定种子的分支对照，每姿态2个精确解，另一分支偏移反而更大。保持原碰撞检查，不将横移猜测加入生产。进一步有限27姿态族仅一组jaw轴+5°出现端点几何正例，尚未能重验新实测接触带，不能称抓取可行。
- 只读恢复审计确认req13 checked3/shortlist30、terminated_early=True；原补视入口要求完整失败证明，近场90秒时限用尽后不能补视，尽管仍有7个开口可容纳方向缺少测量，其中5个共同接触高度约0.699–0.912mm。首失败时原phase约余63.9秒，结束时总session仍约余473秒。
- 已选择隔离实现“有证据的接触搜索提前让出给补视”：仅observing_contact阶段，同request共享最多30秒/优先最多2个不同原候选，height重试不刷新预算；存在真实同请求补偿路径碰撞、独立当前缺面证据、前站实际观测增益、剩余session/views、无可执行incumbent且原phase尚未超时，才可产生新的typed early-yield proof。保留incomplete/未查候选信息，不把原timeout伪装成yield。producer/remote/task/runner共同验证，初始直接抓分支不改；原完整视角路径与到位/采样交接保持。此处为设计及staging，尚未集成或部署。
- 预抓取后重取真实bias已有执行端机制，但目前入口需要完整有效contact计划；为失败计划先授权仅pose0需要新阶段合同，范围更大，当前未实施。最新新对准 `realigned_20261011T114201Z` 尚未使用。


## GEN16 接触搜索让出至测量补视：已部署，等待 GEN17 实机

第16轮记录为失败：一次补视后8次补偿预测接触路径碰撞，只完成30个候选中的3个，随后原近场阶段超时。无接近或闭爪。当前测量仍有7个独立缺面/薄接触带方向。

本次5个生产文件的修改仅在 observing_contact 启用：以30秒共享子预算优先检查两个不同原始接触家族；只有同请求真实补偿碰撞回执、当前实测缺面、已注册实际信息增益及原阶段/会话仍有效时，才以 CONTACT_SEARCH_YIELDED_FOR_MEASURED_VIEW 准备下一补视。保留未检查状态，不将其伪装成全部候选失败；存在可行抓取时仍优先保留，证据不足时回到原剩余阶段预算并保留高度重试，不续开子预算。原阶段时限约束交接发生时刻，后续观察使用原会话及观察阶段时限。碰撞、实测起点、身份和观察完整路径检查不变。

实际工作树最终392项通过、0跳过，独立复核精确覆盖13个修改文件；不累计历史测试数量。交付SHA256 ee8603c649de7d9b17be349d3e5ed29232323e41778825df160b9a1cc801e2c8。部署清单350份源文件，SHA256 8e8f4bd5eb7f030194b019610aae597d84f88bf4c9e64d473ecf853e220c9137。仅重载三个计算节点，驱动/相机/网关/上位机8个进程身份保留。部署记录：/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_no_skin_20261011T000625Z/gen16_contact_search_yield_compute_deployment/result.json。

实录未保存原拒绝轨迹，也未直接保存第16轮私有step_gain；记录测试明确区分真实碰撞和反事实调度，未声称复现原整条轨迹或实物抓取成功。继续使用 realigned_20261011T114201Z 未消耗的用户对准确认；启动前读取最新实测起点。未发失能或温度查询，电子皮肤保持关闭，实验数据仅存SanStand。


## GEN17 回执时间戳类型衔接修复与 GEN18 准备

GEN17 完成首次补视及到位审核，两个原始接触候选均被真实预测路径碰撞拒绝。测量补视搜索分支已启动，但服务回包 selection.stamp_ns 为字符串、当前选择为整数，原字典比较拒绝了相同身份，随后回退旧搜索并近场超时；未接近或闭爪，结果为失败。修复复用严格 parse_selection 后比较，仍精确检查代次和整数时间戳，非法浮点及布尔值拒绝。新增完整服务回包链测试；实际集成 100 项通过，零跳过。

部署清单 verification/gen17_wire_selection_yield_sealed/deployment_sources.json，共 353 项，SHA256 5d732f6f9a7590f9fe87d6f4c6f506e889bf3bedc5fe9bc5907df534b9899e7c；compute unit alicia-observation-measurement-1791723833365075810。仅重载计算节点，硬件、网关、上位机 PID 保持。用户已重新对准、移手，12:45 对准凭证经新鲜实测最大变化 0.001534 rad，控制代次不变，准备 GEN18。此处仍未证明实物抓取成功。


## GEN18 实机：已跨过回执衔接，第二站配准失败被误归为候选耗尽

实机记录 grasp_records/20261011T131409.553041Z_fbd9eb44a4bb4493bdc065e0622ed215，selection18 / 1791724451111255407。首次启动记录 20261011T131153.187231Z_5ce1e4594a4842d0bf0a052d1f5ffbcb 为调用方将额外移手确认拼入原始参数导致凭证断言失败；发生于控制发布和selection前，未运动，录制已明确关闭，保留 startup_argument_error.json。纠正参数后才开始实际GEN18。

实际GEN18完成两次补视及到位审核：首站两个真实预测命令夹指碰撞回执成功转入第二补视，证明GEN17类型衔接修复跨过旧卡点。第二站req42/source1791724571988842248配准 YAW_BOUND_EXCEEDED，修正10.75795度超过10度；RMSE1.50038mm、支撑平面局部偏移2.7798mm。当前请求无bound fused surface，未生成抓取候选，0 strict检查，随后NEAR_FIELD_NO_HARD_SAFE_CANDIDATE结束。不能据此认定所有路径不可达、接触面不足2mm或桌面偏移超7mm。尚未接近或闭爪，实物抓取未成功。

GUI已恢复手动，两个只读监控已关闭；用户重新对准且移手，新凭证 realigned_20261011T131953Z 尚未消费。下一项修复是在同站、同身份及原期限内让当前配准缺测走重新采集，不放宽配准/碰撞门，不复用过期几何。


### GEN18 实录配准离线反例（不放宽门限）

独立证据 gen18_analysis/evidence_registration_reacquire_review.json，SHA256 893edb0c8f1e116621452a66b33b8e0c846c1d21264070c0442d91ccc9e3d742。原请求3帧RGB-D/mask/离线TF重建874点，与审计点数组完全相等、变换最大差0；原10度/7毫米等门限精确复现10.75794984度拒绝。仅后移一帧的滚动窗口恢复REGISTERED、0.03278964度；失败之后完全不重叠三帧1791724572260291576、1791724572592914581、1791724572991546630也恢复REGISTERED，0.04087407度、614内点、RMSE1.297596mm、最大修正3.477436mm。后一个窗口相对失败批次末帧晚1.002704382秒。

这是有限离线测量恢复证据。未来窗口在原实机任务已提前结束，未实际进入接触规划；不代表完整融合/碰撞路径通过，也不是实物成功。其身份、当前性、控制及站位检查必须仍由实际运行时重新满足。两个可移植点云/元数据夹具用于新修复回归。


### GEN18 配准重采修复已部署

两处生产文件限定于已到位 observing_contact 站的当前源 YAW_BOUND_EXCEEDED：真实worker准备期间撤销无效公共几何，丢弃该请求，原有释放入口要求三个全新来源；原owner、控制、身份、phase/session截止及10度门保留。其他硬几何失败与初始直接抓取不受新分支影响。完整实际工作树100项通过、零跳过，前后源码一致；独立46项通过，不叠加计数。交付SHA256 972626338565595f1f2303003197b35965add8eaf8f1bbcc66c2efc399c368d8；部署清单357项 SHA256 5da8b9307c429684778bcd40935d1db6fe123c5cd8bbfc82ffaaa3c91a87c4a5，compute unit alicia-observation-measurement-1791726133126558120。仅三个计算节点重载，硬件/GUI进程保持；准备GEN19实机。


## GEN19 实机：支撑暂缺通知未携带站位等待证明

记录 grasp_records/20261011T134408.435603Z_e3f0b55b6f894b74b2e66bf6f730e52e，selection19 / 1791726250119005203。首次补视运动、到位审核通过，尚未触发GEN18配准重采分支。source1791726305430401086发support_measurement_pending_revalidation，target_lost=false但station合同为空；remote据未绑定false object撤销，task在1791726305.5999因unqualified producer failure during station wait结束。约268ms后source1791726305495664596恢复tracked，object到达时原参考截止1791726305898196983仍余约96ms，但任务已结束。没有接近、闭爪、实物成功。

GUI已恢复手动，用户重新对准并移手；新凭证 realigned_20261011T135026Z 未消费。当前修复将支撑缺测与已登记站位父合同明确绑定，限期不超过原支撑参考+0.8秒及其它原截止，恢复必须重新审核当前帧。用户最新明确授权：修复后上传当前完整源码和问题/解决文档到GitHub；只此源码与文档上传，不改变原始实验local_only策略。入口文档2026-10-11-code-snapshot-and-grasp-issues.md记录真实状态。


## 2026-10-11 GEN19 支撑缺测站位交接修复与源码备份

GEN19 的首次失败是 producer 将短暂支撑缺测按普通 unavailable 通知，下游未收到已登记观察站的等待证明而提前退役。原参考尚有约 365.7 ms，新测量约 268.4 ms 后恢复，READY/object 到达时原截止仍余约 96.1 ms。

修复限于已独立合格并登记的观察站：真实当前 RGB-D、冻结身份、当前关节/控制归属及原支撑缺测审核通过后，发送带 `station_support_gap` 的 typed pending。两个消费者撤销旧几何和计划，保留原固定时钟。重复缺测不能续期，完整当前支撑/RGB/唯一身份审核及真实 object/mask/status 三话题 join 才能恢复新几何；最终提交锁内再次校验 ROS 与 monotonic 截止。初始/未登记站原策略不变。

5 生产文件、3 测试文件及 1 原始状态夹具已独立复核。实际 WORK 一次组合回归 376/376 通过，0 skipped，源码测试前后不变；不能与此前计数相加。封存清单 `verification/gen19_support_station_handoff_sealed/deployment_sources.json` SHA256 `821c7af9e0e0eda125f09c725616f653609930d067f0e5874ef661d45b1efa23`，361 项。集成结果 SHA256 `63ce5dc294dc6a349e6ee8d9fdcfeff2499da08b95fd53ea9b2d4146072b5a09`。

已仅重载三个计算节点，部署单元 `alicia-observation-measurement-1791727867934971321`，`gen19_support_station_handoff_compute_deployment/result.json` 成功，八个硬件/GUI/网关进程保留，未发布运动、温度查询或失能命令。修复尚待下一轮实机验证，补视抓取仍未宣称成功。

用户新增明确要求修复后上传当前完整代码和问题/已解决问题文档；将创建独立 10.11 备份分支，不覆盖 10.8 备份和当前工作分支。原始实验与日志继续 SanStand `local_only`，不重启自动上传/删除。总览见 `2026-10-11-code-snapshot-and-grasp-issues.md`。
