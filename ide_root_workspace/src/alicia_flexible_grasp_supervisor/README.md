# alicia_flexible_grasp_supervisor v2

面向玄雅 Alicia-D 操作臂的 ROS1 上位机与自主柔顺抓取工作包。

本版本已根据当前 GitHub `real-arm` 仓库更新：

- 机械臂驱动包名从旧版 `alicia_duo_*` 更新为 `alicia_d_*`。
- 机械臂状态使用标准 `sensor_msgs/JointState` 的 `/joint_states`。
- 机械臂控制命令使用标准 `sensor_msgs/JointState` 的 `/joint_commands`。
- 关节名使用 `Joint1` ~ `Joint6` 与 `right_finger`。
- MoveIt 默认 group 使用 `alicia`，夹爪 group 使用 `hand`。
- 上位机新增实时摄像头画面显示功能，默认订阅 `/supervisor/camera/color/image_raw`。

## 推荐工作空间结构

```bash
catkin_ws/src/
├── real-arm/                         # Alicia-D ROS1 真机驱动、MoveIt、模型
├── arm-mujoco/                       # 玄雅 URDF/MJCF 模型
├── Electronic-Skin-ML/               # 电子皮肤 Python SDK
└── alicia_flexible_grasp_supervisor/ # 本包：上位机 + 传感器封装 + 自主柔顺抓取
```

## 主要功能

- 机械臂状态实时显示：订阅 `/joint_states`
- 机械臂关节控制：发布 `/joint_commands`
- 夹爪开合控制：通过 `right_finger` 关节发布到 `/joint_commands`
- MoveIt 路径规划：调用 `move_group`，默认 group 为 `alicia` / `hand`
- 笛卡尔空间点动：通过 `/supervisor/cartesian_jog` 服务实现小步位姿控制
- RealSense 摄像头实时显示：`camera_node.py` 采集并发布图像，GUI 显示
- 电子皮肤力反馈：封装 Electronic-Skin-ML SDK，发布压力数组和总力
- 自主柔顺抓取状态机：视觉定位 → 预抓取 → 力反馈闭合 → 抬升验证
- ROS 话题总表：`docs/ros_topics.md`

## 安装依赖

```bash
sudo apt update
sudo apt install -y \
  ros-noetic-moveit \
  ros-noetic-cv-bridge \
  ros-noetic-image-transport \
  ros-noetic-tf2-ros \
  python3-pyqt5 python3-opencv python3-numpy python3-yaml python3-serial

# RealSense Python，二选一
pip3 install pyrealsense2

# YOLOv8 目标检测。首次只验证通路可先装 CPU 版本；如果使用显卡，需要按 PyTorch 官网选择 CUDA 版本。
pip3 install ultralytics torch torchvision

# 电子皮肤 SDK
cd ~/catkin_ws/src/Electronic-Skin-ML
pip3 install -r requirements.txt
```

## 编译

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 先修改配置

1. 修改电子皮肤串口：`config/tactile.yaml`
2. 修改相机参数与是否启用 RealSense：`config/camera.yaml`
3. 修改手眼标定：`config/handeye.yaml`
4. 修改柔顺抓取力阈值：`config/grasp_params.yaml`
5. 修改是否启动真机：`launch/full_system.launch` 或命令行传参

## 启动方式

### 仅启动传感器和上位机

```bash
roslaunch alicia_flexible_grasp_supervisor full_system.launch start_real_arm:=false start_moveit:=false
```

### 启动真机 + MoveIt + 传感器 + 上位机

```bash
roslaunch alicia_flexible_grasp_supervisor full_system.launch start_real_arm:=true start_moveit:=false driver_port:=/dev/ttyACM1 driver_baudrate:=1000000
```

说明：当前 `real-arm` 的 `alicia_d_bringup.launch` 已经会启动驱动和 MoveIt；因此 `start_real_arm:=true` 时一般不要另外打开 `start_moveit`。

## 电子皮肤注意事项

同一个串口通常只能被一个程序占用。正式运行本包时，请关闭厂家电子皮肤上位机，避免它占用 COM/tty 端口。本包通过 Electronic-Skin-ML 的 `TactilePressureSDK` 直接读取数据。

## 摄像头显示说明

- `scripts/camera_node.py` 优先使用 `pyrealsense2` 采集 RealSense 彩色图和深度图。
- 如果未安装 RealSense SDK 或未接相机，可将 `config/camera.yaml` 中 `simulate: true`，GUI 会显示模拟图像。
- 默认不会在 RealSense 读帧失败时自动切到模拟图像；相机节点会尝试重启真实相机流，避免 GUI 显示误导性的假画面。
- GUI 的摄像头显示控件默认订阅 `/supervisor/camera/color/image_raw`。

## YOLOv8 目标识别说明

- 默认目标识别算法为 `config/camera.yaml` 里的 `perception.detector: yolov8`。
- `yolo_model: yolov8n.pt` 会使用 Ultralytics 官方 COCO 预训练模型；首次运行可能会下载权重，也可以提前放到本地后把路径改成例如 `models/yolov8n.pt`。
- `yolo_target_class` 留空时检测所有 YOLO 类别；GUI 输入 `瓶子`、`杯子`、`球`、`手机` 等常见词会自动映射到 `bottle`、`cup`、`sports ball`、`cell phone`。
- YOLO 只能稳定识别模型训练过的类别。若要识别自己的特定抓取物体，需要采集图片、标注 bbox、训练自定义 `.pt` 权重，然后把 `yolo_model` 指向该权重。
- 如果暂时没有安装 `ultralytics/torch`，感知节点不会崩溃，但会发布未检测状态并在 ROS 日志中提示安装命令。
- GUI“目标识别”页的“检测模型”下拉框可选择“YOLOv8 原模型”或“Carton 模型”；选择后点击“确定模型”，感知节点会在不重启 ROS 的情况下重新加载权重。
- 每次点击“确定模型”都会发起新的加载请求；修复权重或依赖问题后可直接再次确认同一模型重试。加载或失败期间会清除旧目标与预抓取/抓取计划，禁止复用旧位姿发起新动作。
- “Carton 模型”读取工作空间根目录的 `carton_model/best.pt`，类别固定为 `carton`；“YOLOv8 原模型”恢复根据目标描述解析 COCO 类别。
- 模型选择只对当前 ROS 运行有效。重新启动整套系统后恢复 `config/camera.yaml` 中的 `yolo_model_choice: original`。
- GUI 显示“加载失败”时先确认 `carton_model/best.pt` 存在，再查看 `/perception/detector_status` 和感知节点日志中的 Ultralytics/Torch 错误。

## 重要调试顺序

1. `rostopic echo /joint_states` 确认机械臂反馈。
2. `rostopic pub /joint_commands sensor_msgs/JointState ...` 小角度验证控制。
3. `roslaunch ... sensors.launch` 确认 `/tactile/state` 和 `/supervisor/camera/color/image_raw`。
4. 打开 GUI，确认摄像头画面与力反馈曲线。
5. 做手眼标定，填写 `config/handeye.yaml`。
6. 先做普通抓取，再打开柔顺抓取。
