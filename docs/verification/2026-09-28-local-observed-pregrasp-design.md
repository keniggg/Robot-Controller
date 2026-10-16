# 在最终预抓取姿态重新观测，再冻结接触计划

本报告只读审查现有节点与闭袋 RGB；没有修改参数、selection、目标、生产代码或发送动作。目标是把主要姿态变化放到接触几何冻结之前，减少姿态相关坐标链漂移。它不证明所有约 10–11 mm 误差都来自手眼，也不把历史 SUPPORT_GEOMETRY_CONTRADICTION 清除后当作新成功。

## 结论

比重标定全部参数更直接的路线存在：显式观察实验先到达一个**可作为最终预抓取位**的姿态，在同一姿态用新鲜相机量重建目标/桌面，最终接触计划继承这个已经实际到达的预抓取位，剩余接近/闭合只允许落在实测有效的局部范围内。现有 two_stage 提供大半流程，但目前没有保证观察位与最终预抓取位一致；现有 direct 也不是视觉闭环。这条路线需要补齐少量证据绑定和规划限制，不能仅换策略或重新点击识别。

最近闭袋图 `/ros_log/grasp_repair_resume_20260928T120606Z/local_visual_recovery_design/j6_baseline_source_rgb.png` 中板居中，电机约在 (266,155)。`tabletop.Config.center_radius_fraction=0.125`，640×480 初始获取圈半径是 60 px，电机距图像中心约 100 px。获取电机不需要精确把它移到正中心：图像约右移 25 px、下移 40 px 已可能使其进入获取圈，同时白板下缘仍有约 30 px 余量。这里只是图像空间可行性提示，不是关节命令或已通过几何的姿态。后续必须使用同模型 RGB-D/板观测重新证明，不能移动电机。

## 可直接复用的代码和接口

| 位置/接口 | 已有能力 | 不能据此假定的能力 |
|---|---|---|
| `remote_grasp6d_node.py:7103` `_make_observation_sequence`、`:7177` `_frozen_observation_reference_pose` | 观察目标以当前冻结 tool0 姿态构造，不被目标平面 yaw 牵着翻腕；相机到目标有界距离搜索 | 观察姿态最终一定能作为接触 pregrasp；独立固定板/电机一致性 |
| `mode_aware_remote_grasp6d.py:385` `_visible_unknown_pregrasp` | 对近场 pregrasp 搜索最多 40 mm 桌面平行偏移，保持姿态、接触点和支撑高度，之后重做扫掠/MoveIt | 任意缩短路径已经安全；direct 分支会跳过此功能 |
| `/supervisor/check_observation_pose_strict`，`SetTargetPose(execute=false)` | 只读规划和轨迹证据，现有观察速度/支撑/跟踪合同 | 独立于任务上下文的运动许可 |
| `/supervisor/execute_observation_pose_strict` 或 `/supervisor/plan_and_execute_observation_pose_strict` | 在已提交 FAR_FIELD_OBSERVATION_PLAN、当前活动任务、当前所有权下执行冻结观察路径 | 当前 GUI 直控中能直接调用；允许伪造 `/grasp/state`/`near_field_phase` 绕过上下文 |
| `.runtime/bounded_*` 当前单轴实验客户端 | GUI 模式下实际 SDK/epoch/accepted/同源板约束、40 mm 支撑/20 px 整板/120 mm 深度证明、所有 RGB/depth 归档 | 通用任意 Cartesian 观察路径或自动抓取；不能未经新审查扩成无限动作循环 |
| `/grasp_6d/request_plan` | 开启/关闭推理；`true` 已开启时可只是返回 already running | 强制重新取得到达后快照、替换所有缓存 |
| `/grasp_6d/replan_execution` | 请求提升允许的 preview，可能立即使用已有缓存 | 从当前位置重新识别；旧 preview 的观测时刻可能早于新姿态 |
| `grasp_task_node.py:3500` `_set_near_field_active` 与 `NearFieldPlanningPhase` | 明确到达后的源时刻、phase、deadline；remote `:4520` 按 exact source 重建到达观测 | 现消息仅有 base center/target，没有固定板/独立局部证书 |
| `grasp_task_node.py:3585` 附近 rebound，`:5767` `_validate_reused_contact_pregrasp` | 若新 pregrasp 与已到达位一致，可复用真实到达位并验证实际跟踪/补偿 | 当前生成器保证会复用；补偿只对 frozen base goal 收敛，不修复相机到手指绝对误差 |
| `/supervisor/check_pose_sequence_strict`、最终接触几何/夹爪扫掠/仿真 | 对最终固定四阶段计划、当前关节起点做严格验证 | 大幅 pregrasp 没有发生；仅有排名偏好不能证明小运动 |

当前未知物观察工作距离是 0.22–0.30 m、名义 0.26 m，来自 `alicia_grasp_modes/observation_policy.py`。它是相机工作距离而非指尖离桌面的 40 mm。观察阶段的 40 mm 要用 CAD 和现场板/桌面证明；接触执行仍遵守其自身未放宽的指尖/桌面几何规则，不能把观察净空误当成闭爪时净空要求。

## 为什么现有流程还会重复大调姿

`grasp_task_node.py` 到达 far-field observation 后确实进入新 near-field phase；但 near-field 生成的 `pregrasp` 若不同，它又执行一次 `6D near-field pregrasp`。随后 final_refine 还可能生成另一条 pregrasp。`mode_aware_remote_grasp6d.py:538` 只要求最终 joint_max_delta_rad 有限且非负，未知物大转动是成本排序而非硬约束。故“观察后再识别”本身无法阻止主要姿态变化发生在最后一次测量之后。

`mode_aware_grasp_task.execution_config` 对 direct 关闭 near-field replan 和 final_visual_refine；direct 保留物理门，但属于当前快照固定接触计划，之后仍可能大幅移动。`_register_near_field_surface` 在 direct 中仅重建单视图，明确不是跨姿态独立一致性证明。`observe_return` 会回到出发位再执行接触，因此不是本实验所需的“留在最近的最终观察位”。

## 最小新增内容

1. **独立的观察实验身份与旧失败引用。** 记录旧 selection/track/plan/support-fault/source-stamp，固定板布置 ID、模型 fingerprint、X hash、driver epoch、新实验 ID。旧 fault 保留。电机身份用 RGB 外观/轮廓与板坐标位置共同关联。`TargetTracker.lost` 当前只能通过新 selection 清除；不能调用 reset 来伪装恢复。需要一个明确、证据限定的局部重新关联入口，或独立诊断观察器先完成实验，之后以其证书授权一次新的识别边界。入口应能拒绝错物、板移动和模型变化，记录继承关系，而不是默认信任“这是新代”。
2. **先完成主要调姿，后冻结接触计划。** 现有候选可用于提出最终 pregrasp 的位置和抓取方向，但此时只授予观察运动权限，禁止闭爪。把相机视野、板/电机共视和 40 mm CAD 净空一起纳入观察姿态选择；路径优先最小实际关节运动，不围绕目标重新翻腕。每次停稳后保存同源 RGB/Z16/板/accepted/SDK，并重证下一段；不是把多次小动作当成无界累计许可。
3. **到达证书 `ObservedPregraspWitness`。** 至少含已实际到达的 q/tool pose、原 SDK、epoch、独立 stationary 窗口、exact 新鲜源、同电机/板布置身份、相机模型和 X hash、板坐标下的目标与桌面、原始图/深度引用和允许的局部关节区域。比较板坐标与 FK/base 两条链：板局部稳定而 base 漂移时不能把当前 ICP 拉回旧 base 后称为消除物理误差。
4. **最终生成器与执行器绑定同一实际预抓取位。** final contact plan 的 pregrasp 只能来自 witness 的实际到达位，或在其经验证的局部范围内产生明确的小修正；不能事后强行把任意计划的首点覆盖成 current pose。必须重新检查完整开爪 CAD、approach/grasp 几何和整个严格 MoveIt 序列。自由空间 orientation resolver 如要改 pregrasp，必须退回“观察提案”，先调姿后再次采样，不能在已冻结计划中悄悄保留大调姿。
5. **真正约束最后一段的路径。** 在 `_strict_check_final_near_field_plan` 的最终序列和执行提交处检查当前 q 是否仍在 witness 的实际起点/epoch，有效域覆盖全部轨迹点和连续几何/跟踪包络。局部范围应来自实测观测一致性和误差预算，不任意拍一个 5°/10 mm 上限。统计最大关节变化、累计关节路程、tool/camera 旋转和位移；成本排名不足以替代该检查。现 `_validate_reused_contact_pregrasp` 可作为真实到达验证的接续点。

## 仍须实证的边界

局部观察能减少“从旧姿态的目标坐标运动到很不同的姿态”的差分误差，但不能自动证明相机到真实手指接触点的固定偏置。最终应比较相机坐标中的目标/桌面与已验证相机到手指几何，或取得能独立看到真实指尖的证据；否则，即使 FK 到位误差很小也仍可能在物体上方闭合。

板姿态检测已使用 `RuntimeColorModel`/SDK rays，而 `tabletop.py` 的 `_project`/`_project_pixels`/`project_base_to_pixel` 和 `unknown_tabletop_perception.py` 的 intrinsics 入口是必须核对的相机射线接口。本报告不重复另一代理的 pinhole 与 inverse-Brown 数值审计，不据此擅自修改内参或 X。进入局部接触证据前两条路径必须确认使用同一真实相机模型，不能用板局部重复性替错模型背书。

验证需包含：旧 fault 保留、错 target/移动板/epoch/model 改变拒绝；同一实际姿态多帧测量重复性；逐次观察的桌面/目标在板坐标稳定；最终 witness 到 pregrasp 不发生新增大调姿；最终严格路径始终处于实测局部有效域；接触前同源支撑证据无矛盾；闭爪后真实夹持证据，不把 SDK/仿真闭合当成抓取成功。保留未知物 20 g 上限与提升仿真非硬门策略，不改既有速度修复。
