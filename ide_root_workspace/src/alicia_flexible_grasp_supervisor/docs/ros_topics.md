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
