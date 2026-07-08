# WSL MuJoCo 6D 抓取数字孪生第一版

本文对应 `mujoco_wsl_digital_twin_plan.docx` 中推荐的第一版目标：

- 路线 A：WSL 自己进行 GraspNet baseline 6D 推理，自己进行 MuJoCo 预演。
- 完整状态同步：WSL 直接订阅 Ubuntu ROS `/joint_states`。
- 物体模型：第一版默认使用鼠标复合几何体，可替换为低面数鼠标 mesh。
- 执行门控：真实机械臂执行 6D 抓取前，ROS 端先请求 WSL `/simulate_grasp`，低分或失败则阻止真机执行。

## 1. 新增代码

WSL 端：

- `tools/mujoco_digital_twin_server.py`
  - `GET /health`
  - `POST /predict`
  - `POST /sync_joint_state`
  - `POST /simulate_grasp`
- `tools/start_mujoco_digital_twin_wsl.sh`

ROS 端：

- `alicia_flexible_grasp/vision/mujoco_digital_twin_client.py`
- `grasp_task_node.py` 中加入 6D 执行前 MuJoCo 门控。
- `config/grasp_params.yaml` 中加入 `/mujoco_digital_twin` 配置。

## 2. WSL 端依赖

已有 GraspNet baseline 环境可继续使用：

```bash
conda activate grasp6d118
python - <<'PY'
import torch
print(torch.__version__)
print(torch.version.cuda)
print(torch.cuda.is_available())
PY
```

建议版本：

- Python 3.8
- PyTorch `2.4.1+cu118`
- CUDA Toolkit `11.8`
- MuJoCo Python `mujoco>=3.2`
- NumPy `>=1.23`
- ROS Noetic Python 包，仅在使用 `--ros-sync-joint-states` 时需要。

安装 MuJoCo 相关依赖：

```bash
conda activate grasp6d118
pip install "mujoco>=3.2" numpy scipy pyyaml
```

如果 WSL 要直接订阅 Ubuntu ROS `/joint_states`，WSL 内还需要能 import `rospy` 和 `sensor_msgs`。如果暂时不想在 WSL 安装 ROS，可以把 ROS 端参数 `/mujoco_digital_twin/send_joint_state_in_request` 改成 `true`，由 ROS 请求主动携带关节状态。

使用 ROS 订阅前建议先验证：

```bash
source /opt/ros/noetic/setup.bash
conda activate grasp6d118
python - <<'PY'
import rospy
from sensor_msgs.msg import JointState
print('rospy and sensor_msgs import ok')
PY
```

## 3. WSL 启动组合服务

在 WSL 中进入仓库：

```bash
cd ~/grasp6d_ws/Robot-Controller
source /opt/ros/noetic/setup.bash
conda activate grasp6d118
sed -i 's/\r$//' tools/start_mujoco_digital_twin_wsl.sh tools/mujoco_digital_twin_server.py
```

完整方式：WSL 订阅 Ubuntu ROS。

先设置 ROS 网络变量，示例：

```bash
export ROS_MASTER_URI=http://<Ubuntu_VM_IP>:11311
export ROS_IP=<WSL_IP>
```

启动服务：

```bash
./tools/start_mujoco_digital_twin_wsl.sh \
  --ros-sync-joint-states \
  --warmup
```

服务默认监听：

```text
http://0.0.0.0:9000
```

健康检查：

```bash
curl http://127.0.0.1:9000/health
```

## 4. ROS 端启动

把 `<WSL_IP>` 换成 Ubuntu 虚拟机能访问到的 WSL/Windows IP，不要带尖括号：

```bash
cd ~/alicia_wa_full
source devel/setup.bash
export GRASP6D_URL=http://<WSL_IP>:9000
roslaunch alicia_flexible_grasp_supervisor full_system.launch \
  start_real_arm:=true \
  driver_port:=/dev/alicia_arm \
  driver_baudrate:=1000000 \
  start_camera:=true \
  start_tactile:=true \
  start_gui:=true \
  use_remote_grasp6d:=true
```

当前默认配置中：

```yaml
grasp_6d.remote.server_url: "http://127.0.0.1:9000"
mujoco_digital_twin.enabled: true
mujoco_digital_twin.execution_gate_enabled: true
mujoco_digital_twin.send_joint_state_in_request: false
```

也就是说，默认按“WSL 订阅 ROS joint_states”的完整方式运行。

## 5. GUI 操作流程

1. 启动 WSL 组合服务。
2. 启动 ROS 端 full system。
3. GUI 目标识别页识别鼠标。
4. 点击“生成 6D 候选”。
5. remote_grasp6d_node 调用 WSL `/predict`，生成 `/grasp_6d/plan`。
6. 点击“执行 6D 抓取”。
7. grasp_task_node 先调用 WSL `/simulate_grasp`。
8. MuJoCo 返回 `simulation_ok=true` 且 `score>=80` 后，才继续真机 pregrasp/approach/grasp/lift。

如果仿真失败，GUI/日志会显示类似：

```text
MuJoCo simulation blocked execution: collision at approach: Link7/floor
```

## 6. 复杂鼠标模型

第一版默认用复合几何体近似鼠标：

```yaml
mujoco_digital_twin:
  object_model:
    type: "mouse_compound"
    label: "mouse"
    size_xyz_m: [0.10, 0.06, 0.035]
    mass_kg: 0.08
```

如果你有低面数鼠标碰撞模型，例如：

```text
/home/lv/grasp6d_ws/assets/mouse_collision.obj
```

改成：

```yaml
mujoco_digital_twin:
  object_model:
    type: "mouse_mesh"
    label: "mouse"
    mesh_path: "/home/lv/grasp6d_ws/assets/mouse_collision.obj"
    mesh_scale: [1.0, 1.0, 1.0]
    size_xyz_m: [0.10, 0.06, 0.035]
    mass_kg: 0.08
```

建议使用低面数、闭合、尺度正确的 mesh；不要直接用高面数视觉 mesh 做碰撞体。

## 7. 调试开关

临时跳过仿真门控：

```bash
rosparam set /mujoco_digital_twin/execution_gate_enabled false
```

WSL 不安装 ROS，由 ROS 请求携带 joint_states：

```bash
rosparam set /mujoco_digital_twin/send_joint_state_in_request true
```

只测试网络协议，不加载模型和 checkpoint：

```bash
python tools/mujoco_digital_twin_server.py --mock --host 0.0.0.0 --port 9000
```

真机执行前建议保持：

```bash
rosparam set /mujoco_digital_twin/execution_gate_enabled true
rosparam set /mujoco_digital_twin/allow_execution_on_error false
```

## 8. 第一版验收标准

- WSL `/health` 返回 `ok=true`。
- WSL 能通过 `/predict` 返回 GraspNet 6D 候选。
- WSL 能通过 ROS 订阅到 `/joint_states`，`joint_state_age_sec` 持续刷新。
- ROS 点击“执行 6D 抓取”前会先调用 `/simulate_grasp`。
- 仿真低分、IK 失败、碰撞或接触失败时，真机不会执行。
- 仿真通过时，真机按照 pregrasp、approach、grasp、lift 执行。
