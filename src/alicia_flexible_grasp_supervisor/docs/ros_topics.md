# ROS 话题总表 v2

本文件是设计文档，不是 ROS 自动生成文件。真正的话题由各节点运行时创建。

## real-arm 当前版本已存在话题

| 话题 | 类型 | 发布节点 | 订阅节点 | 说明 |
|---|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | `alicia_d_driver_node` | GUI / MoveIt / robot_state_publisher | Alicia-D 当前关节状态，包含 `Joint1`~`Joint6` 和 `right_finger` |
| `/joint_commands` | `sensor_msgs/JointState` | `motion_gateway_node.py` / GUI | `alicia_d_driver_node` | Alicia-D 关节和夹爪命令 |
| `/zero_calibrate` | `std_msgs/Bool` | GUI / motion_gateway | `alicia_d_driver_node` | 零点校准 |
| `/demonstration` | `std_msgs/Bool` | GUI / motion_gateway | `alicia_d_driver_node` | 拖动示教/0 力矩模式 |

## 本包新增电子皮肤话题

| 话题 | 类型 | 发布节点 | 订阅节点 | 说明 |
|---|---|---|---|---|
| `/tactile/state` | `TactileState` | `tactile_skin_node.py` | GUI / 抓取节点 / 安全节点 | 左右电子皮肤综合状态 |
| `/tactile/skin1/frame` | `TactileFrame` | `tactile_skin_node.py` | GUI | 左指/皮肤 1 压力帧 |
| `/tactile/skin2/frame` | `TactileFrame` | `tactile_skin_node.py` | GUI | 右指/皮肤 2 压力帧 |

## 本包新增摄像头话题

| 话题 | 类型 | 发布节点 | 订阅节点 | 说明 |
|---|---|---|---|---|
| `/supervisor/camera/color/image_raw` | `sensor_msgs/Image` | `camera_node.py` | GUI / perception_node.py | RealSense 彩色图，GUI 实时显示 |
| `/supervisor/camera/depth/image_raw` | `sensor_msgs/Image` | `camera_node.py` | perception_node.py | RealSense 深度图 |

## 本包新增感知话题

| 话题 | 类型 | 发布节点 | 订阅节点 | 说明 |
|---|---|---|---|---|
| `/perception/object` | `ObjectPose` | `perception_node.py` | `grasp_task_node.py` / GUI | 目标检测与三维定位结果 |
| `/perception/object_pose_camera` | `geometry_msgs/PoseStamped` | `perception_node.py` | 调试/RViz | 相机坐标系下目标位姿 |
| `/perception/object_pose_base` | `geometry_msgs/PoseStamped` | `perception_node.py` | 抓取节点 | 机械臂 base 坐标系下目标位姿 |
| `/perception/object_detected` | `std_msgs/Bool` | `perception_node.py` | GUI | 是否检测到目标 |

## 本包新增抓取与安全话题

| 话题 | 类型 | 发布节点 | 订阅节点 | 说明 |
|---|---|---|---|---|
| `/grasp/state` | `GraspState` | `grasp_task_node.py` | GUI / logger | 自主抓取状态机状态 |
| `/safety/status` | `SafetyState` | `safety_monitor_node.py` | GUI / 抓取节点 | 安全状态 |

## 2026.9.20 预抓取补偿服务

以下是代码接口及历史配置说明。七轮实验已结束，用户已关闭硬件与WSL并转为离线处理；不表示话题或服务当前在线。反馈采样与最终新帧交接修复曾加载，但对应实机分支尚未验收；七轮均未闭爪或提起，详见[实验记录](../../../docs/verification/2026-09-20-pregrasp-budget-guide.md)。

| 服务 | 类型 | 调用者 | 约束 |
| --- | --- | --- | --- |
| `/supervisor/compensate_pregrasp` | `CompensatePregrasp` | `grasp_task_node` | 完整冻结 CONTACT 计划、活动 MOVE_PREGRASP、同 epoch；只补偿 poses[0]，保留物理目标。execute=false 只规划首步，不能当作轨迹已获执行许可。 |

补偿入口失败时，`ENDPOINT_CORRECTION_FEEDBACK_UNAVAILABLE` 追加的 `snapshot` 包含实际采样时刻、参考epoch、采样耗时、各源缓存条数及最新源戳。该诊断描述网关内部冻结窗口，不能由另一订阅端的录包新鲜度替代；原0.5秒新鲜度、0.3秒停稳、两秒等待和源/epoch检查保持。

JSON 返回与 ROS 日志新增 `conditional_budget_guide`，仅表示原始32 count及剩余步数内的条件搜索终点；每步仍独立检查路径和真实响应，不能视作全局最优或物理收敛。

最终视觉phase绑定到位后采集、接收后不超过0.2秒且源年龄不超过 `/grasp_6d/remote/planning_snapshot_max_inference_latency_sec`（默认1.2秒，本机既有配置5秒）的同目标对象快照，重复源帧不续期；等待和推理共享原20秒绝对期限。无新参考报 `FINAL_REFINE_REFERENCE_UNAVAILABLE`，remote继续验证精确RGB-D及全部几何条件。

remote候选生成对完全相同的有序倾角集合复用已验证CAD计算，并在同一次生成内复用捕获的配准表面；不缓存到后续请求。`ros_prepare_stages_ms.tabletop_candidates`仍计入原准备/推理源年龄门，新角度完整重算。

JSON 返回与 ROS 日志含 plan_id、固定目标、模型哈希、SDK/实测计数、源戳、每步响应及残差；`PREGRASP_MEASURED_CONVERGED` 只表示模型内到位，不是持物成功。[边界和验证](../../../docs/verification/2026-09-20-pregrasp-compensation.md)。

2026.9.20离线补充：推理metrics的 `snapshot_evidence` 在请求过期、准备失败及pending替换/取消时保留提交输入的 `snapshot_stamp_ns`、`source_stamp_ns`、`unique_source_frames`、`source_span_ms`；源戳保留整数纳秒。不从拒绝prediction或当前phase补历史；失败时未保存的phase字段省略。原失败性能指标仍不信任远端输出。最终序列检查传递原phase截止期，调用返回逾期不授予后续动作。上述修改未实机部署，详见[离线记录](../../../docs/verification/2026-09-20-offline-followup.md)。
