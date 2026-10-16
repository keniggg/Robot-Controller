# 提升整段偏置：默认关闭的生产接线与真实 MoveIt 适配器

生产代码已有可调用接口，尚未加载到 ROS，也没有执行新提升。`/grasp/unknown_lift_cartesian_compensation_enabled` 在 YAML 中明确为 **false**。开启后，unknown CONTACT plan 的 `LIFT_OBJECT / linear 6D lift` 使用 `/supervisor/move_contact_lift`，计划或执行失败不退回原普通提升路径。

## 实现

- `robot/lift_path_continuity.py`：已复审的纯完整路径推导与 finite-pad/双占据/支撑面验证，源 SHA `101b0df2e144185b858800c70b3a23ca216bb853e61440189d99f9563dd22606` 保持不变。历史 13 点提升仅改首点时 Noetic 首段不变；完整偏置在“偏差恒定”反事实中消除 13.495 mm 模型终点差，不是实机结果。
- `robot/lift_linear_gateway.py`：独立 `LIFT_OBJECT` 上下文，复用现有 active task、epoch、revocation、actuation、stationary following feedback 与 planner lock。冻结同源 accepted 开度、plan/model/selection/config、SDK/accepted，60 s 既有有限 episode。仅 execute=false 推导时不启动控制器、连接 action 或创建 hold publisher。缓存先消费，任何失效/执行失败无重发、补偿或自动返回。
- `robot/lift_moveit_adapter.py`：可直接连接 `/check_state_validity` 和 `/get_planning_scene`，并非一个永久缺失的 provider。保留原世界场景、已有附着物和 ACM，对 nominal、shifted、各自 Noetic 首个正时间点桥接及全部后续样条做两种 request-local 占据检查。按样条导数界选择样本，使相邻参数样本每轴位置变化不超过 0.002 rad；这是一项**全臂采样碰撞检查**，没有声称连续全臂证明。纯模块另提供命令路径的连续 gripper/object/support 几何界。
- `motion_gateway_node.py`：注册 `/supervisor/move_contact_lift`，复用 `CompensatePregrasp` 消息，不扩大旧 pregrasp 的 pose index 0/1/2 权限。
- `grasp_task_node.py`：只有显式 bool true 才在提升阶段选择新服务，仍调用原 measured endpoint 验证；不把提升执行失败交给 pregrasp trim/recovery。
- `moveit_planner.py::_execute_observation_contract`：只增加可选 action-goal builder，默认仍使用原 `bound_action_goal`。新 lift 临时装入 `bound_lift_action_goal`，执行仍经过同一 `ContractedTrajectoryExecutor` 的 live authority、before_send、失败 hold 和无 fallback 合同；调用结束恢复原 builder/guard/authority。

## 原有硬门与本次增强的边界

原 lift 已有：当前 accepted 起点、严格 Cartesian 规划、MoveIt 碰撞与关节限位、SDK controller-reference 桥接速度验证、controller 返回状态、停止后 measured endpoint 检查。未知物体 20 g 上限、现有速度修复未改；MuJoCo 的提升/总分仍只是诊断。

本次补齐整段偏置后，显式检查被改变的完整 nominal/shifted/SDK bridge，而不只检查首点。实际 MoveIt 请求是只读、请求局部的，使用 rich plan 原始 OBB，camera/mount 缺模作为明确的覆盖不足诊断，不以新增强要求制造永久硬门。若提供这些盒模型，必须给有效 tool0 frame、正尺寸、刚体变换与来源 SHA；纯几何会检查它们对支撑和目标的命令包络，但不会声称已覆盖相机/支架与整条机械臂之间的所有碰撞。

初始贴桌物体不能凭一个新增加的任意正负 following-error box 被强制判为不可能提升。因此此 route 使用**原配置**的 controller path/goal tolerance，并明确报告 `hardware_following_swept_volume_certified=false`、`hardware_stopping_swept_volume_certified=false`。它没有冒用 free-space observation 的跟随误差盒/停止包络证书。控制器的停止 hold 行为仍保留，未声称该 hold 的实际扫掠体积已经得到新证明。

命令模型中明确进入支撑或非许可接触区域仍会拒绝；有限指垫证据无法证明也不默认为通过。原旧 bag 在初始左指的 CAD 包络/OBB 交集仍 `unresolved finite pad coverage`，这可能来自保守包络和旧约 10–11 mm 场景链漂移，不能宣称实物碰撞非法或 lift 算法失败。

## 两种 MoveIt 请求的具体语义

每个请求使用独立 `RobotState` 副本，`is_diff=true`，保留已有附件，增加一个唯一 diagnostic ID，不调用 apply-planning-scene、publish-planning-scene、REMOVE 或任何真实 attachment 命令。

世界假设的物体 pose/frame 始终是 `base_link` 下固定的 rich-plan pose。临时 attached body 的 link 为经过 URDF 验证没有碰撞几何的 `tool0`，从而避免把物体挂到 base_link 时隐式豁免实际底座碰撞。携带假设使用同一个 tool0 下冻结的相对 OBB pose。请求对整个机器人检查，`touch_links` 只含 Link7/Link8；这一粗粒度权限必须与纯模块中的连续有限指垫交集证明同时通过，不能作为整根手指的独立碰撞豁免。

Noetic 的 `RobotState` 转换按 object header frame 转换附着物，并在 `is_diff=true` 下保留既有附件，见 [MoveIt 官方 conversions.cpp](https://docs.ros.org/en/noetic/api/moveit_core/html/conversions_8cpp_source.html)。本机安装的 CollisionObject.msg 同样将 object pose 与 primitive pose 分开，本实现两者均正确赋值。

环境/ACM/已有附件/padding/transforms/octomap 的摘要在检查前后及 action send 前比较。提交时在新停稳反馈与 hold 读取之后再次检查场景和来源年龄，防止等待新反馈期间场景变化被漏过。摘要只排除 live joint feedback 字段，不排除已有附着物或环境几何。请求构造前拒绝 JointState 名称/位置维度不一致、重复名称、非有限值和不支持的 MultiDOF 内容，避免消息转换部分忽略非法状态。

## 验证与尚缺实证

最终候选联合回归：**512 passed, 7 warnings in 9.68 s**。覆盖新 gateway 34 项、adapter 23 项、已复审 pure lift 35 项，以及现有 contact、gateway、task sequence、pregrasp task、contracted executor 和 MoveIt planner 的相关回归。warning 为现有 rospy 弃用提示。新增用例包含等待新反馈期间场景变化、最终场景查询导致反馈过期、非法 JointState 转换前拒绝，以及已提供但 frame 非法的 camera/mount 模型不能被当成缺模诊断。

历史 13 点曲线通过真实 adapter 算法、fake read-only service transport需要 312 个状态请求，离线 0.255 s。该数字不是在线 MoveIt 服务耗时或实机动作结果。测试验证：默认关闭无控制副作用；完整路径一次缓存消费；来源/epoch/开度/模型/场景失效拒绝；时间变化重证全部点；两种占据保留；SDK bridge 与样条内部 overshoot 被采样；旧 path/goal tolerance 原值绑定；没有伪造硬件跟随/停止/attachment 结论。

仍需父执行者在审查后加载源代码，先用新的近场、同源 motor/table 场景与闭爪反馈做 execute=false 的真实只读检查，再决定启用及实际 lift。当前未调用生产 lift service prepare/execute、未改变参数、未重启或加载节点、未发布动作。下述真实 MoveIt 只读接口联调不经过 task/gateway，也不产生当前执行许可。真实抓取与实际提升成功仍未验证。

归档：SanStand `ros_log/grasp_repair_resume_20260928T120606Z/lift_path_continuity/`，最终日志 `real_adapter_reviewed_regression.log`。纯模块历史回放在 `reviewed_20260928T140441Z/`；接线源快照与摘要另存独立 wiring 目录，不覆盖早期记录。

## 历史 fixture 的真实 MoveIt 只读接口联调

独立审查通过后，使用未改动的历史 13 点 fixture 连接当前 `/check_state_validity`、`/get_planning_scene`。当前 `/robot_description` 与 fixture 的 SHA 完全相同。服务请求沿用当前完整 RobotState 变量表，并仅在请求副本填入历史各点的关节与闭爪开度；没有伪造生产 task phase、改变 enable 或调用任何运动接口。

正式一轮 adapter 检查完整 nominal/shifted/SDK bridge，共 **312 次状态请求，4.460 s**（包括逐请求写外盘）；另 4 条两态起终点诊断请求。合计 **316 次状态 RPC 均 valid=true**，RPC 净耗时 **1.483 s**，最大单次 **12.2 ms**；4 次 scene RPC 合计 **15.0 ms**。固定世界与假设携带 frame 各 158 条，前后环境摘要相同。全流程约 4.740 s。该通过只说明当前 MoveIt 采样路径/服务消息可用；历史场景的 pure finite-pad 仍 unresolved，因此整体不是提升许可，更不是当前现场或实际提升验证。

为了验证服务确实使用了新增请求局部物体，又做了 **2 条碰撞负控**：保持 rich-plan 物体尺寸，仅把测试 OBB 中心置于已知 base STL 三角面的重心；两种 frame 下表示同一世界 pose。两条均原生返回 `valid=false`，接触双方明确为 `base_link` 和各自 diagnostic attached body（body type 2）；接触位置和穿透深度仅约 1e-16 m 浮点差，scene 摘要不变。这支持实际 Noetic frame 转换和无底座豁免的语义；它们只是请求副本，没有全局场景修改。

报告在 `native_readonly_20260928T143048Z/native_moveit_report.json`，SHA `7d48f57a139e3ab17e16d2683b07783b0556f0e0b7a3b5c72a01037b4eb41efd`；负控在其 `negative_controls_20260928T143217Z/`。首轮 `native_readonly_20260928T143010Z` 的 320 个原始 RPC 收发已保存，但最后汇总脚本遇到相对 `__file__` 路径错误；修复汇总器后才完成上述正式一轮，首轮保留且不作为新的许可。
