# 保留故障身份的局部观察与近场抓取恢复接口设计

只读设计，未改参数、selection、原 tracker、故障锁存或执行节点，未发送动作。目标是不移动电机，在同一标定板/电机/邻近桌面证据下，先分段到达更合适的局部观察姿态，随后只执行由该姿态附近新建的接触计划；不通过重选目标或更新 plan 时间戳消除旧故障。

## 当前 epoch 的身份衔接（13:35 补充，覆盖上文旧同代次假设）

本轮节点重启后，当前 selection 是 generation 1、stamp `1790597313246093034`、strategy `observe_return`；感知尚未锁定目标（`target_locked=false`、`target_lost=false`、`no_center_instance`）。旧 9 时抓取失败属于前 epoch，当前没有可直接继承的 production track。不能强制沿用不存在的旧 track/generation；也不能把手工 ROI 填成旧 track。

恢复上下文应始终保留历史失败记录 digest、旧 plan digest，同时独立记录当前 driver epoch、当前 selection、初次唯一电机识别链。新 tracker 经过真实三帧唯一 acquisition 后，允许新 ID，并通过明确的 `historical_failure_to_current_unique_motor` 桥接记录与旧失败关联；这不宣称跨 epoch track 数字相同，也不恢复旧计划许可。用户保持电机原位作为实验连续性事实保留。

初次锁定前的自动中心取景属于有限、非接触诊断提案：使用新鲜实测场景和 40 mm 路径证明，可沿用独立有界诊断执行层；不能要求先有 production track 才允许一切诊断观察。下面纯证据模块的生产 target gate 仅用于锁定后的局部观察/接触计划审查，不替代该前缀。

标定板可观测性按现有检测角点数、二维覆盖、同源与 RMS 等质量合同；如该 observer 要求检测角点离边 20 px 则保留。整张 18×13.5 cm 板都在 20 px 内是之前辨识实验的自定条件，不是永久抓取前提。板的物理占据/碰撞与视觉角点覆盖单独证明。

## 现有接口能做什么

| 可复用入口 | 实际行为与约束 |
|---|---|
| `observation_policy.observation_config()` | 仅为 unknown 的 `two_stage/observe_return` 设置相机距离偏好 0.26 m、范围 0.22–0.30 m。不是局部视觉控制器，也不是指尖净距。 |
| `centered_observation_pose_at_camera_distance()` | 保持输入 tool 姿态，将相机光轴指向目标；可复用其当前姿态优先的几何提案，不能把一次算出的远目标直接当闭环结果。 |
| `/supervisor/check_observation_pose_strict`、`/supervisor/plan_and_execute_observation_pose_strict` | 严格 MoveIt、实际 SDK 起点、连续路径/跟踪域及场景绑定，执行要求任务拥有 committed observation context。可以复用内部实现，现有接口不能任意换成新局部场景。 |
| `two_stage` | 到达观察位后，以新 near-field 快照重建并冻结接触计划；适合作为调度骨架。它的注册和支撑连续性仍在 base/FK 坐标下，不能处理已知 11 mm 正矛盾。 |
| `observe_return` | 初始视角足够时直接接触，否则补充视图并回到初始位重新规划。回初始位会恢复本次想减少的姿态跨度，不能直接作为“留在近处抓”的恢复分支。可复用 session/ACK、身份和时限合同。 |
| `_maybe_rebind_near_field_grasp6d_plan()` 与 `/grasp/near_field_phase` | 固定 phase/deadline、fresh Preview、完整接触候选冻结；已有一个 clear-view 重观察机会。它只针对 `NEAR_FIELD_SURFACE_VIEW_REQUIRED`，不是支撑矛盾恢复，且只允许一次。 |
| `_execute_clear_view_reacquisition()` | 有限回撤/侧移、完整路径排序、到位后新观测。当前配置最小接触净距 80 mm，属于退开取景，不能直接冒充 40 mm 净距的近目标接近。 |
| `/supervisor/check_pose_sequence_strict` | 为全新候选验证 pregrasp/approach/grasp/lift 序列，返回最大关节变化与路径代价；可用于局部域筛选，但还须检查每个轨迹点与连续包络的绝对域。 |
| `/supervisor/move_contact_approach`、`move_contact_grasp_pose`、`compensate_*` | 接收冻结 `Grasp6DPlan`，只处理其固定阶段位姿；不能用作任意视觉 ΔZ 或任意新 pose 通道。新局部接触计划必须先正式绑定。 |

代码位置：`alicia_grasp_modes/.../observation_policy.py:8`，`remote_grasp6d_node.py:2631,7103,7177`，`motion_gateway_node.py:161–201,229`，`grasp_task_node.py:3210,3913,4331`，`observe_return_task.py:182,211`。

## 真正缺失的证据入口

新增纯几何数据对象（建议 `LocalRecoveryObservation`，字段名明确 camera/board/base，不把 board 数据塞进名为 points_base 的现有字段）：

- `recovery_id`、原 `plan_id/digest`、原 fault 的 source/code/value、selection generation/stamp、原 target track/epoch、固定板布置 ID、driver epoch。
- exact RGB/Z16/板 TF/quality 的同一 source stamp；原接收时间、独立实际反馈停稳窗口；URDF/X SHA、相机模型指纹、SDK rays 模型、深度单位、RGB/深度配准来源。
- 电机的原始 RGB mask、实际有深度像素、`motor_points_camera`，同帧测得的邻近**桌面**点集/平面及其 footprint；板的 PnP `T_camera_board`、独立板内深度平面、板物理轮廓。所有原始 ROI、点数、空间覆盖、残差和像素源保留。
- 显式三条表达：`points_board = inv(T_camera_board) * points_camera`，`points_base_fk = FK(accepted)*X*points_camera`，以及原 camera 点。桌面法向/偏置按刚体平面规则变换；在电机 footprint 上比较距离，不能只比坐标原点处 d。

`.runtime/board_conditioned_geometry_audit.py:validate_observation/audit_pair` 的同源、身份、法向、平面变换规则可直接复用为底层函数。但它现有回放仅 `plane_label=board_depth_surface`，没有电机 mask、邻近桌面平面和接触几何；给板面结果换名字不算补齐证据。

`unknown_tabletop_perception.py:208` 在 tracker 输出前调用 `segment_with_support_fallback(...anchor,last,reference_plane_base)`，新支撑正矛盾会先退出。恢复入口必须独立从 exact 原始图像生成局部证据，不修改原 tracker 的 anchor/plane/loss latch。使用历史 mask/外观/尺度与唯一空间关联确认仍是同一电机，然后在固定板坐标建立新的局部关联；无唯一匹配就继续取景而不能改选板或其他物件。不能重新调用“图像中心最近物体”并称为原 track。

附加模型问题必须公开：现 `tabletop.py:_project_pixels` 是 fx/fy/cx/cy 针孔投影，板 tracker 用 RealSense SDK inverse-Brown rays（当前 coeff 非零）。局部证据应统一用已归档 SDK rays，且另输出 legacy 针孔结果的数值差以明确新旧证据的来源；不能混用两套点云，也不能把差异直接归因为 11 mm 根因。当前手眼 X 保持不变。

## 闭袋图像证明和仍缺什么

已读取 J6 闭袋 baseline 的 exact 原图：电机大约在像素 (268,156)，位于标定板上方的红色桌面；图像中心落在板内。标定板平面不是电机实际支撑桌面。新 resume 的 `local_visual_recovery_design/j6_baseline_source_rgb.png` 与 `source_image.json` 保存该来源，不能作为当前 live 画面使用。

J4/J6 原袋已经保存多帧 exact RGB/Z16/板姿态及停稳 actual，电机和附近桌面可见。因此首先可离线提取同帧电机与桌面证据，检查空间覆盖和跨姿态 board 稳定性；无需先做更多机械臂动作。板轮廓、笔记本、桌沿、线缆应从桌面拟合样本中分开；分别保留板面与桌面，不能选一个残差小的平面替代另一个。当前环形支撑采样可能与板重叠，这必须通过实际 ROI/残差审计确认，尚不能宣称通过。

至少保留多个新鲜帧、两个真实不同观察姿态以及一个未用于建立局部模型的返回/保留姿态。重复静止图像只能估计图像噪声，不能证明整个接触姿态范围。电机可见点云的质心会随遮挡变化，需比较可关联的表面、轮廓/footprint、宽高及桌面上接触目标，不能只要求完整点云质心一致。board 局部支撑比较沿用现有 4 mm/4° 限制；base 下旧矛盾继续显示。3D 非唯一对应、换板、物体被碰动或模型/epoch 变化会使该局部上下文失效。

## 显式恢复上下文与 40 mm 观察阶段

最小新增会话 `LocalRecoveryContext`，由 task 持有，绑定原 target、原故障、原计划和 board 布置，保留旧故障永久审计；不改 selection/generation。建议状态为 `EVIDENCE_ONLY → LOCAL_OBSERVE → POST_MOVE_EVIDENCE → LOCAL_CONTACT_READY`。每次观察动作之后必须停稳并收新图，按新测量重算下一步；限定最大步数/绝对截止时间，失败不做无界探测。

观察提案优先保持当前 tool 方向，以电机 ROI 和板可见范围同时约束视点；目标位于画面上部时先自动调整相机，使电机和足够板角点、桌面区域共视，而不是要求移动电机。可复用原 centered-view 数学与按真实时长/关节运动排序，去除大翻转候选的方式是局部上下文限定姿态域，而非修改全局未知物体策略的 `candidate_max_joint_delta_rad=0` 或恢复旧 90° veto。

**40 mm 为观察专用的最小物理净距下界**：至少对开爪指尖/掌部/相机/相关末端 CAD、桌面和板障碍，检查完整连续路径，计入 fresh actual→SDK/controller 起点桥接、跟踪误差包络和证据不确定性后的下界。不能用 tool0 的 z、相机距离、端点名义距离或只看对角线代替。若已有更严的距离要求则保留更严者。原 `validate_observation_trajectory` 与 `observation_endpoint_support_bound` 可复用；恢复 profile 的 40 mm 距离与场景需显式入 hash，不改全局 3 mm 接触标准或已有 80 mm clear-view profile。

当前相机工作带仍是 22–30 cm；40 mm CAD 净距与该视距能否同时满足必须由实际 X/CAD/新图验证，不能承诺一定可达。最后从 40 mm 观察下界进入接触范围时必须切换为**新的完整 contact authority**，不能继续借用“观察”名义下降。

网关有真实实现缺口：`CommittedObservationContexts` 一旦捕获 active_scene，就拒绝不同 source/scene；失效计划会撤销上下文。因此不能简单循环调用现 observation service 携带不同新图。应新增只作用于 recovery_id 的上下文入口和单调 `advance` 操作，校验上一证据 hash、动作完成证明、新 source、同 epoch/track/fault、模型未变，再使下一段新场景成为该恢复步骤的唯一 authority。底层 MoveIt、continuous path、SDK bridge、速度与碰撞模块复用；旧计划/旧缓存从不授权下一步。

建议独立新服务语义为 `prepare_local_observation(recovery_id,evidence_digest,proposal,execute=false)` 和 `execute_local_observation(recovery_id,prepared_digest)`；实际 ROS 类型可先沿用经过完整 schema 校验的 String 会话 + 私有 task/gateway 方法，再决定固定 .srv，不能裸发布 plan/state 来冒充活跃任务。

## 从更近姿态生成新接触计划

局部接触起点必须是最新停稳姿态，不回初始远位。板用于验证“电机/桌面未动”和局部有效域；当前 camera 点经当前 `FK*X` 一次映射成新 base geometry 供既有规划器使用，而不是把旧 base 目标减去 11 mm。重新生成 mask/表面、接触候选及完整 approach/grasp/lift 序列，并重新做全部 CAD、碰撞、MoveIt 和仿真；不继承旧碰撞结果。

在最终 snapshot 的局部有效域内，记录所有轨迹点、连续段和允许跟踪偏差相对当前 q 的最大绝对跨度；范围来自实际覆盖并留出验证的姿态域。若完整接触动作超域，先在 40 mm 观察 authority 下缩短姿态跨度并重新观察，而不是放宽域或让近场候选继续大幅翻转。不能凭空宣称“每步 2° 就必然覆盖最后几十毫米”：单步和累计跨度都要有证据；小步后重新观察可用于推进观察位，但不能把所有小步视为同一个已验证固定域。

原 2.6767676 mm 接触高度 stress、3 mm 支撑净距、centering override、现有未知物体 20 g 上限和提升仿真仅诊断策略原样保留。没有增加必须人工测量指尖的前置要求。X/CAD 的固定误差仍是本分支局限；局部观察只减少大姿态链误差，不提供外部绝对标尺。若新的局部证据误差已超过候选能承受的现有高度/包络，则该候选不能取得局部 authority，不能把 stress 值当作新误差已经被覆盖的证明。

旧 fault 的处理必须明确：`contradiction_after_plan()` 目前只拒绝 fault source 晚于 plan 的情形。新设计不能靠新 plan 时间较晚自动绕过；另持有 `recovery_id → original_fault_digest → new_plan_digest → local_evidence_digest → q_domain/deadline`，仅对这一全新局部计划给出限定授权，旧计划永远不恢复。任务 checkpoint、remote candidate audit、gateway 均须验证同一证据引用。离开局部域、目标换代或再出现 board 内真实几何矛盾时，授权失效且 fault 审计保留。

## 最小落地顺序

1. 先只写独立读取/证据适配器，回放现 J4/J6 原袋的 motor+table+board，而非继续只拟合板内深度；输出影像 ROI、源 hash、局部差、base 差和拒绝原因。
2. 实现纯 `LocalRecoveryContext` / `LocalRecoveryEvidence` 与判定函数；合成测试必须覆盖：11 mm 仅 base 漂移、真实桌面移动、错误电机身份、板更换、模型/epoch变化、非同源/旧帧、40 mm 包络不足、旧 fault 被新 stamp 隐藏、最后 contact joint 越域。
3. 接入有限观察路径上下文，先 dry-run 生成可审阅的整段路径及 40 mm 下界。保持现策略为父选择，只开启明确 recovery 子阶段，不切换全局 selection。
4. 用恢复终点新的近场证据生成局部 contact plan，完整重跑原检查并验证 q 域后才执行；记录真实夹住/仍在桌面的结果。当前本设计阶段未宣称这些实现或实抓已经通过。

## 已实现的独立纯模块（尚未接生产）

- `.runtime/local_recovery_evidence.py`：`FaultIdentity` 保留历史故障；`ActiveTargetIdentity` 与 `bind_active_target()` 接收真实新 epoch acquisition 的显式桥接；`LocalEvidence.from_motor_diagnostic()` 始终只输出手工 ROI 诊断级；`from_bound_observation()` 是未来生产 adapter 的严格 schema 入口；`RecoveryContext.append()` 不可变地保留证据及原故障；`review_proposal()` 只输出结构审查结果，`motion_authorized` 永远为 false，必须另行验证引用的真实源和路径证明。
- `from_bound_observation()` 要求独立桌面、电机 footprint、同源生产 mask/track 关联、SDK rays 及原始 actual 停稳样本；不接受一个“verified=true”替代所有来源。不将板内平面冒充桌面。源接收年龄 <=350 ms、accepted 关联 <=100 ms；离线居中 ±0.4 s 与在线因果前置 0.8 s 停稳证据明确区分，避免要求未来反馈却仍假称帧新鲜。
- observation 提案只审查 40 mm 连续净距和新路径/起点证明引用；contact 提案另要求独立训练/保留观测、全新 contact plan 与完整 gate 证明引用、连续 q 包络在局部域内、既有标定策略保留。新 plan 时间更晚不能消除原 fault 绑定。模块不能验证纸面 SHA 对应的真实物理证明，外层必须验证，当前未接执行服务。
- `.runtime/adapt_motor_local_recovery_diagnostics.py` 已读取根代理 `motor_adjacent_table_normal_oriented/*/report_with_provenance.json`：40 帧接入、0 拒绝，全部保留 `manual_roi_diagnostic`，0 个 production target/运行 context 被伪造。原四源 stamp、actual 原始窗口、图像 hash、板/桌/电机数据随证据保留。仍缺生产目标绑定、电机实际支撑 footprint 和局部响应域，因此不会晋级。没有重复读取拟合原袋。
- `.runtime/test_local_recovery_evidence.py` 17 项合成测试通过：新 epoch 新 ID 显式桥接、手工 ROI 无升权、exact source/新鲜度、原始停稳样本、换模型/epoch/板、11 mm 仅 base 漂移与真实板内移动区分、40 mm 下界、原 fault 不被新 stamp 隐藏、旧 plan 不复用、接触路径越域和训练冒充保留集拒绝等。

外盘结果 `local_recovery_evidence/adapted_motor_diagnostics.json`、三份源码快照及 verification.json。实现仍为纯合同骨架与诊断数据适配，不是已部署抓取修复或接触授权。
