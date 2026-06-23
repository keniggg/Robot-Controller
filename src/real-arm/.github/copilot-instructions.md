# Alicia Duo ROS — AI Agent Instructions

## 一句概览
这是一个 ROS1 (Noetic) catkin 工作区，用于 Alicia Duo 机械臂。AI 代理的目标是快速定位关键组件、构建/运行流程、调试点与跨包接口，便能在本仓库内安全地修补、实现或自动化任务。

## 关键组件与位置（快速索引）
- 驱动层: [alicia_duo_driver](alicia_duo_driver) — 串口/低层节点（C++ 在 `src/`，Python 工具在 `scripts/`）。关键文件：[alicia_duo_driver/src/serial_server_node_main.cpp](alicia_duo_driver/src/serial_server_node_main.cpp) 和 `scripts/servo_control_node.py`。
- 描描述与可视化: [alicia_duo_descriptions](alicia_duo_descriptions) — URDF、meshes、RViz launch（例：[alicia_duo_descriptions/launch/display.launch](alicia_duo_descriptions/launch/display.launch)）。
- MoveIt 接口: [alicia_duo_moveit](alicia_duo_moveit) — 运动规划集成，常见类/方法在 `scripts/`，例如子类化 `MoveItRobotController`（参见 [alicia_duo_moveit/scripts/moveit_control.py](alicia_duo_moveit/scripts/moveit_control.py)）。
- 校准: [alicia_duo_calibration / easy_handeye](alicia_duo_calibration/easy_handeye) — 手眼标定工具与 launch 文件。
- 视觉抓取: [alicia_duo_grasp_2d](alicia_duo_grasp_2d)（OpenCV）和 [alicia_duo_grasp_6d](alicia_duo_grasp_6d)（PyTorch/GraspNet）。关键：`alicia_duo_grasp_2d/scripts/grasp_2d.py` 和 `alicia_duo_grasp_6d/requirements.txt`。

## 数据流与集成点（必须知道）
- 低层串口/驱动 → 发布 `/joint_states` 与自定义消息 `ArmJointState`（见 [alicia_duo_driver/msg/ArmJointState.msg](alicia_duo_driver/msg/ArmJointState.msg)）。
- 感知模块发布物体位姿到 `/detected_object_position`，抓取/执行模块订阅该话题。
- 机械臂运动通过 MoveIt 接口（`alicia_duo_moveit`）下发轨迹；夹爪通过 `/gripper_control` 话题单独控制（不是 MoveIt gripper group）。

## 构建 / 运行 / 常用命令（示例）
- 在 Linux（或 WSL/容器）下工作：
```bash
source /opt/ros/noetic/setup.bash
cd <workspace_root>
catkin_make
source devel/setup.bash
```
- 启动可视化（例）：
```bash
roslaunch alicia_duo_descriptions display.launch
```
- 启动串口服务（例）：
```bash
roslaunch alicia_duo_driver serial_server.launch
```
- 运行 2D 抓取脚本（需先 source 工作区并确保 MoveIt/必要节点运行）：
```bash
python3 alicia_duo_grasp_2d/scripts/grasp_2d.py
```
- 安装 PyTorch / Python 依赖（6D 抓取）：
```bash
pip3 install -r alicia_duo_grasp_6d/requirements.txt
```

注：ROS Noetic 在 Windows 原生支持有限，建议使用 Ubuntu / WSL2 / Docker / VM。

## 项目约定与常见模式（搜索即见）
- 包结构：C++ 在 `src/`、Python 在 `scripts/`、launch 在 `launch/`、URDF 在 `urdf/`。遵循 ROS 包常规结构。
- Python 脚本经常使用 `sys.path.append(...)` 在运行时跨包导入（见 `alicia_duo_grasp_2d/scripts/grasp_2d.py`），修改或重构时注意保持运行时路径兼容。
- 自定义消息：使用 `message_generation` 与 `message_runtime`；查看 `alicia_duo_driver/msg/`。
- 日志与调试：节点使用 ROS 日志，调试常用命令 `rostopic echo`, `rosnode info`, `rqt_graph`, `rosbag`。

## 重要文件 / 例子（用于实现任务或补丁时引用）
- 驱动入口（串口）：[alicia_duo_driver/src/serial_server_node_main.cpp](alicia_duo_driver/src/serial_server_node_main.cpp)
- 驱动消息定义：[alicia_duo_driver/msg/ArmJointState.msg](alicia_duo_driver/msg/ArmJointState.msg)
- MoveIt 控制封装：[alicia_duo_moveit/scripts/moveit_control.py](alicia_duo_moveit/scripts/moveit_control.py)
- 2D 抓取脚本：[alicia_duo_grasp_2d/scripts/grasp_2d.py](alicia_duo_grasp_2d/scripts/grasp_2d.py)
- 6D 抓取依赖：[alicia_duo_grasp_6d/requirements.txt](alicia_duo_grasp_6d/requirements.txt)

## 安全与假设（AI 代理必须遵守）
- 不要在没有仿真或硬件安全审查的情况下直接发布到真实机器人。对发布到 `/gripper_control` 或关节话题的改动必须可回滚并经人工确认。
- 对串口交互或低层命令的修改要小步验证（先在仿真或通过干预开关验证）。

## 快速检查清单（补丁/功能实现前）
1. 能在本地启动关键节点（serial server、MoveIt、RViz）并复现基础话题。 
2. 明确要修改的消息/话题（如 `/joint_states`, `/gripper_control`）。
3. 在提交 PR 前，提供复现步骤与最小示例脚本（或回滚开关）。

## 需要我补充的内容？
如果有特定任务（例如：实现新消息、修复串口协议、把某个 Python 脚本改为包式导入），请告诉我目标和你希望我优先检查的文件，我会基于本文件做针对性补丁并运行基本验证。

---
原有 README 与安装脚本仍有价值：查看 [README.md](README.md) 与 [install/alicia_amd64_install.sh](install/alicia_amd64_install.sh)。
# Alicia Duo ROS Project - AI Agent Instructions

## Architecture Overview
This is a ROS1 (Noetic) catkin workspace for the Alicia Duo robotic arm. Core components include:
- **alicia_duo_driver**: Serial communication and low-level control (C++ nodes in `src/`, Python scripts in `scripts/`)
- **alicia_duo_moveit**: Motion planning with MoveIt (configs in `config/`, scripts in `scripts/`)
- **alicia_duo_descriptions**: URDF models and meshes for visualization (URDF in `urdf/`, meshes in `meshes/`)
- **alicia_duo_calibration**: Hand-eye calibration using easy_handeye
- **alicia_duo_grasp_2d/6d**: Vision-based grasping (Python scripts with OpenCV/PyTorch)
- **alicia_duo_drag_teaching**: Pose recording/replication for teaching

Data flows from serial driver → MoveIt planning → ROS control → actuators. Vision modules publish detected poses to `/detected_object_position` topics, consumed by grasp controllers.

## Developer Workflows
- **Build**: Run `catkin_make` in workspace root after sourcing ROS (`source /opt/ros/noetic/setup.bash`)
- **Environment**: Always `source devel/setup.bash` after build; add to `~/.bashrc` for persistence
- **Launch**: Use `roslaunch` for multi-node setups (e.g., `roslaunch alicia_duo_descriptions display.launch` for RViz visualization)
- **Debugging**: Check ROS logs with `rostopic echo /topic_name`; use `rqt_graph` to visualize node connections
- **Testing**: Run Python scripts directly (e.g., `python3 scripts/grasp_2d.py`) after sourcing workspace; ensure MoveIt nodes are running for arm control

## Project Conventions
- **File Structure**: ROS packages with `package.xml`, `CMakeLists.txt`; Python in `scripts/`, C++ in `src/`, configs in `config/`, launches in `launch/`
- **Imports**: Python scripts import across packages by adding paths (e.g., `sys.path.append('~/alicia_ws/src/alicia_duo_moveit/scripts')` in `alicia_duo_grasp_2d/scripts/grasp_2d.py`)
- **Messages**: Custom msgs in `msg/` (e.g., `ArmJointState.msg` in driver); use `message_generation` depend
- **Transforms**: Use `tf` for coordinate frame conversions; vision poses transformed to arm base frame
- **Gripper Control**: Uses ROS topic `/gripper_control` for gripper commands, not MoveIt gripper group
- **Calibration Data**: Hand-eye calibration matrices stored in `~/.ros/easy_handeye/*.yaml` files
- **Dependencies**: External libs like PyTorch for 6D grasp (install via `pip install -r requirements.txt` in `alicia_duo_grasp_6d/`); ROS deps via `rosdepc install`

## Integration Points
- **ROS Topics**: `/joint_states`, `/gripper_control`, `/detected_object_position` for inter-module comm
- **MoveIt Interface**: Scripts subclass `MoveItRobotController` from `alicia_duo_moveit/scripts/moveit_control.py` for arm motion
- **Vision Integration**: OpenCV for 2D detection, GraspNet models for 6D grasp prediction
- **Calibration**: Run `roslaunch easy_handeye calibrate.launch` for hand-eye setup

Reference: [README.md](README.md) for package descriptions; [install/alicia_amd64_install.sh](install/alicia_amd64_install.sh) for setup steps.</content>
<parameter name="filePath">d:\成信研究生院\机械臂力反馈项目\Alicia_duo_ros1-main\Alicia_duo_ros1-main\.github\copilot-instructions.md