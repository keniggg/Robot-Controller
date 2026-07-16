# WSL2 GraspNet + MuJoCo 纸箱抓取运行手册

这条链路把 Ubuntu/ROS 侧的一次 RGB-D 规划快照送到 WSL2：同一个服务进程先在
`/predict` 生成 GraspNet 候选，再由 `/simulate_grasp` 对同一个富计划执行 MuJoCo
门控。真实机械臂只能在富计划仍有效、响应 `plan_id` 完全一致、五个安全布尔量均为
`true` 且分数达到阈值时继续。

> 本阶段不使用力、触觉或滑移反馈。MuJoCo 通过只说明几何和运动学前置条件满足，
> 不保证实物抓取成功。

## 1. 唯一允许的 WSL 主流程

当前已探测到的 WSL `:8000` 仍是旧 GraspNet-only 代码，`/simulate_grasp` 返回
`unknown path`。因此必须先把 WSL checkout 同步到本功能分支的批准版本；在完成同步、
记录 WSL commit 并通过组合健康检查前，不得把旧进程当作可降级运行模式。同步前先运行
`git status --short` 并保留 WSL 中已有的本地修改，不要直接覆盖未知改动。同步完成后至少
确认以下文件来自同一批准版本：

```bash
cd ~/grasp6d_ws/Robot-Controller
git rev-parse HEAD
git status --short
test -x tools/start_mujoco_digital_twin_wsl.sh
test -f tools/mujoco_digital_twin_server.py
test -f src/arm-mujoco/synriard/mjcf/Alicia_D_v5_6/Alicia_D_v5_6_gripper_50mm.xml
rg -n "'/simulate_grasp'|'/predict'" tools/mujoco_digital_twin_server.py
```

如果 WSL checkout 没有可获取该分支的 Git remote，应使用项目既定的受控传输方式同步
完整 checkout；不要只复制一个 server 文件，因为协议、mesh、测试和启动脚本必须匹配。

同步后先在旧服务终端按 `Ctrl-C` 停止 GraspNet-only 进程，再确认 WSL 本机 8000 端口
已经释放：

```bash
ss -ltn | rg ':8000\b'
```

期望无输出；如果仍有监听项，先识别并正常停止旧进程，不要同时启动两个服务。

必须启动同时提供 `/predict`、`/health` 和 `/simulate_grasp` 的组合服务：

```bash
conda activate grasp6d118
cd ~/grasp6d_ws/Robot-Controller

export GRASPNET_BASELINE_ROOT=/home/lv/grasp6d_ws/graspnet-baseline
export GRASPNET_CHECKPOINT=/home/lv/grasp6d_ws/checkpoints/checkpoint-rs.tar
export GRASPNET_DEVICE=cuda:0
export MUJOCO_ALICIA_MODEL_XML="$PWD/src/arm-mujoco/synriard/mjcf/Alicia_D_v5_6/Alicia_D_v5_6_gripper_50mm.xml"

./tools/start_mujoco_digital_twin_wsl.sh \
  --pass-score 80 \
  --min-lift-success-m 0.015 \
  --warmup
```

WSL 环境需能导入 `mujoco>=3.2`、NumPy、SciPy，并具备既有 GraspNet 依赖、模型
checkpoint 和 Alicia-D mesh。ROS 主机与 WSL 主机必须使用同步的墙上时钟；
`snapshot_stamp_sec` 为零、位于未来或超过 2 秒都会被拒绝为 `PLAN_STALE`。

以下服务不能授权真实机械臂：

- `start_graspnet_baseline_server_wsl.sh` 启动的旧 GraspNet-only 服务没有
  `/simulate_grasp`；
- `--mock`、`--mock-graspnet` 或 `--mock-mujoco` 只用于协议测试；
- 任何仅返回 `/predict` 成功、HTTP 200 或高分但没有完整 MuJoCo 分项结果的服务。

组合服务启动后先做机器可判定的健康检查：

```bash
curl -fsS http://127.0.0.1:8000/health | python3 -m json.tool

curl -fsS http://127.0.0.1:8000/health | \
python3 -c 'import json,sys; h=json.load(sys.stdin); assert h.get("ok") is True, h; assert "grasp_backend" in h, h; assert "digital_twin" in h, h; assert h["digital_twin"].get("backend") == "mujoco", h; assert h["digital_twin"].get("ok") is True, h; print("combined GraspNet+MuJoCo health OK")'
```

`check_remote_grasp6d_server.py` 只能检查通用健康状态，不能单独证明服务器包含
MuJoCo；启用机械臂前必须额外验证上面的 `digital_twin` 字段。

服务是 headless HTTP server，没有内建 MuJoCo viewer。默认只能依据服务日志、ROS
诊断主题和响应字段检查。若需要画面，必须另行启动并记录一个独立 viewer；不能把
“日志正常”写成“viewer 已验证”。

## 2. WSL2 到 Ubuntu 的网络

在 WSL2 查询地址：

```bash
hostname -I
```

在 Ubuntu ROS 主机检查组合健康响应：

```bash
export GRASP6D_URL=http://REPLACE_WITH_WSL2_IP:8000
curl -fsS "$GRASP6D_URL/health" | python3 -m json.tool
```

若虚拟机不能直接访问 WSL2，可在管理员 PowerShell 建立 Windows 转发：

```powershell
netsh interface portproxy add v4tov4 listenaddress=0.0.0.0 listenport=8000 connectaddress=<WSL2_IP> connectport=8000
netsh advfirewall firewall add rule name="Alicia Grasp6D 8000" dir=in action=allow protocol=TCP localport=8000
```

然后把 `GRASP6D_URL` 改为 Windows 主机地址。`<WSL2_IP>` 是说明文字中的占位符，
不要把尖括号原样输入 Bash。

## 3. Ubuntu ROS 启动与安全参数

第一次部署包含新消息/服务后必须重新生成接口，重新 source，并重启所有客户端和
服务端。当前 `alicia_flexible_grasp_supervisor/StartGrasp` 请求包含 `execute` 和
`plan_id`，MD5 必须为 `5d246499be275f0453d1db3b1be742a6`。

先以机械臂禁用模式做相机和 WSL 验收：

```bash
cd /home/zhuyupei/alicia_wa_full
catkin_make --force-cmake --pkg alicia_flexible_grasp_supervisor
source devel/setup.bash

rossrv md5 alicia_flexible_grasp_supervisor/StartGrasp

export GRASP6D_URL=http://REPLACE_WITH_WINDOWS_HOST_OR_WSL2_IP:8000
rosrun alicia_flexible_grasp_supervisor check_remote_grasp6d_server.py "$GRASP6D_URL"

roslaunch alicia_flexible_grasp_supervisor full_system.launch \
  start_real_arm:=false \
  start_camera:=true \
  start_tactile:=false \
  start_gui:=true \
  use_remote_grasp6d:=true \
  remote_grasp6d_url:="$GRASP6D_URL"
```

`grasp_system.launch` 将同一个 `remote_grasp6d_url` 同时写入预测和仿真 URL。启动后
必须从 live parameter server 验证两者相同：

```bash
PREDICT_URL="$(rosparam get /grasp_6d/remote/server_url)"
TWIN_URL="$(rosparam get /mujoco_digital_twin/server_url)"
printf 'predict=%s\ntwin=%s\n' "$PREDICT_URL" "$TWIN_URL"
test "$PREDICT_URL" = "$TWIN_URL"
```

再核对 fail-closed 参数：

```bash
rosparam get /mujoco_digital_twin/enabled
rosparam get /mujoco_digital_twin/execution_gate_enabled
rosparam get /mujoco_digital_twin/allow_execution_on_error
rosparam get /mujoco_digital_twin/send_joint_state_in_request
rosparam get /mujoco_digital_twin/min_score
rosparam get /gripper/open_position_m
```

期望依次为 `true`、`true`、`false`、`true`、`80` 和 `0.05`。其中任何一项不符，
或两个 URL 不同，都不得开启机械臂。`allow_execution_on_error` 永远不得改成
`true` 作为恢复手段；网络、超时、协议或服务异常均应阻断执行。

## 4. 三模型互斥切换

目标识别页提供三个 profile，同一时刻只允许一个
`/perception/yolo_model_choice` 生效：

| choice | 界面名称 | task | 规划输入 |
| --- | --- | --- | --- |
| `original` | YOLOv8 原模型 | detect | bbox + 深度前景 |
| `carton` | Carton 模型 | detect | carton bbox + 深度前景 |
| `carton_segment` | Carton 分割模型 | segment | carton 实例 mask + 深度 |

在 GUI 选择 profile 后点击确认，等待 `/perception/detector_status` 变为
`ready:<choice>:...`。切换会递增 reload generation，并清除旧检测、旧 mask、旧几何和
旧富计划；切换完成前不能执行上一条计划。

`carton_segment` 是严格 mask 模式：checkpoint task 不匹配、没有实例 mask、mask
为空、尺寸不匹配，或无法取得三帧新鲜且时间戳匹配的 mask/RGB/depth/object 样本时，
规划失败并清除执行授权。如果节点从未收到 mask，失败码为 `MASK_MISSING`；最新收到
的 mask 为全零时是 `MASK_EMPTY`；曾收到非空 mask、但当前无法组成三份新鲜、精确
同时间戳的 RGB-D-mask-object 样本时是 `MASK_STALE`。三者都会经
`/grasp_6d/status`、失效 `object_geometry` 和富计划 tombstone 对外发布，不能沿用旧
mask 或旧计划。

分割轮廓只显示在目标识别页，并且只有 mask 与当前目标 header 完全匹配时才绘制；
普通相机页保持原始画面。detect profile 不伪造实例 mask。

## 5. 一次点击与三帧稳定窗口

默认 `/grasp_6d/remote/auto_request=false`。GUI 的“生成 6D 候选”一次点击只触发
一次 `/grasp_6d/request_plan` 请求；该请求在最多 1 秒内尝试收集三份新鲜、同时间戳
RGB/depth/object 样本，segment 模式还要求对应 mask。三帧还需通过以下稳定性检查：

- mask IoU、质心漂移和关节变化；
- mask 腐蚀、内部小孔处理、有效深度比例和 MAD 去异常值；
- 支撑面拟合、目标点云和 base-frame OBB；
- GraspNet 候选、真实夹爪几何、可达性及视野门控。

因此“一次点击”表示触发一个三帧采集/规划窗口，不表示一定生成成功。失败后上一条
富计划已经失效；排除原因后需要重新点击，以新快照生成新 `plan_id`。

相机验收时使用：

```bash
rosservice call /grasp_6d/request_plan "trigger: true"
```

相机-only 阶段不要调用 `/grasp/start`。

## 6. 诊断主题与命令

```bash
rostopic echo -n 1 /perception/detector_status
rostopic echo -n 1 /perception/object_mask/header
rostopic echo -n 1 /supervisor/camera/color/image_raw/header
rostopic echo -n 1 /supervisor/camera/depth/image_raw/header

rostopic echo -n 1 /grasp_6d/status
rostopic echo -n 1 /grasp_6d/object_geometry
rostopic echo -n 1 /grasp_6d/gate_audit
rostopic echo -n 1 /grasp_6d/plan_enriched
rostopic echo -n 1 /grasp_6d/plan
rostopic echo -n 1 /grasp/state
```

应记录的证据包括：

- color/depth/mask/object 的源时间戳；
- `valid_depth_points`、`valid_depth_ratio`、`depth_mad_m`、`fused_frames`；
- `support_inlier_ratio`、目标点数和 OBB 三维尺寸；
- WSL raw/NMS/collision/returned 候选数及本地逐级 gate counts；
- `candidate_width_m`、`required_open_width_m`、四阶段位姿和 `plan_id`；
- MuJoCo 的五个布尔分项、分数、`failure_code` 和 `failure_reason`。

## 7. 50 mm 夹爪硬限制

执行合同固定为 `Alicia_D_v5_6_gripper_50mm`。解析门控先把 OBB 沿候选 jaw axis
投影，再给两侧各加 2 mm 余量：

```text
required_open_width_m = OBB 在 jaw axis 上的投影宽度 + 2 × 0.002 m
```

必须满足 `0 < required_open_width_m <= 0.050`。候选网络给出的 width 只用于诊断，
不能越过这个 OBB 投影硬门。门控还使用完整 Link7/Link8 指爪与 Link6 掌部包络检查
支撑面和物体碰撞。

MuJoCo 编译模型会验证实际碰撞 mesh 满开内间隙约为 `49.9375 mm`，且与 50 mm
合同误差不超过 0.5 mm。仿真从满开逐步闭合，保留首次双侧接触的完整状态和宽度，
再沿检测到的支撑法向抬升；不会把指爪强制闭合到零间隙后才判断抓取。

## 8. 富计划与 `plan_id` 生命周期

`/grasp_6d/plan_enriched` 是唯一执行授权，包含：

- 与快照绑定的 header、模型选择和非空 `plan_id`；
- 固定顺序的四个位姿：pregrasp、approach、grasp、lift；
- 候选宽度、50 mm 门控后的 required width；
- 同一快照生成的 OBB 和支撑面。

纸箱质量与摩擦不在 `Grasp6DPlan` 消息或 `plan_id` 规范字节中；构建 schema-2 请求时，
它们由 live `/mujoco_digital_twin/object_model` 受控配置追加。现场记录必须同时保存该
配置，不能把运行时质量/摩擦误称为富计划自身字段。

`plan_id` 由快照纳秒、模型选择、四个位姿、宽度、OBB 和支撑面等规范化内容生成。
模型切换、目标/mask 丢失、几何失败、新请求、计划替换、超时、停止或执行中 authority
变化都会发布失效 tombstone 并撤销旧 ID。默认计划有效期为 2 秒。

`/grasp_6d/plan` 是派生的 `geometry_msgs/PoseArray`，只供 RViz/界面显示。旧
`grasp6d_node.py`（`use_remote_grasp6d=false`）只能发布这个 legacy 可视化结果，不能
产生执行授权。legacy topic 即使包含四个位姿，也绝不能启用“执行 6D 抓取”。

执行请求必须携带当前 `/plan_enriched` 的精确 `plan_id`。ROS 在发送 WSL 请求前后都
会重新验证 authority；网络请求进行中发生 stop、计划替换或过期时，即使 WSL 返回
成功也不得运动。`StartGrasp` MD5 不匹配的旧 GUI/节点必须重建并重启。

## 9. MuJoCo 响应的 fail-closed 判据

服务响应必须回显相同 `plan_id`，分数必须有限，并显式包含以下五个 Python/JSON
布尔量：

```text
simulation_ok=true
ik_success=true
collision_free=true
contact_success=true
lift_success=true
score >= /mujoco_digital_twin/min_score
```

字段缺失、类型错误、ID 不一致、非有限分数、HTTP/网络异常或任一分项为假都阻断。
主要诊断包括 `MODEL_TASK_MISMATCH`、`MASK_MISSING`、`MASK_STALE`、`MASK_EMPTY`、
`MASK_SIZE_MISMATCH`、`DEPTH_UNSTABLE`、`DEPTH_INSUFFICIENT`、
`SUPPORT_PLANE_INVALID`、`OBB_INVALID`、`WSL_UNAVAILABLE`、`WSL_PREDICT_FAILED`、
`NO_RAW_CANDIDATE`、`NO_GEOMETRIC_CANDIDATE`、`GRIPPER_MODEL_MISMATCH`、
`GRIPPER_TOO_NARROW`、`GRIPPER_SWEEP_COLLISION`、`PLAN_STALE`、
`PLAN_ID_MISMATCH`、`MUJOCO_IK_FAILED`、`MUJOCO_COLLISION`、
`MUJOCO_CONTACT_FAILED`、`MUJOCO_LIFT_FAILED`、`MUJOCO_SCORE_BELOW_THRESHOLD` 和
`MUJOCO_INTERNAL_ERROR`。

## 10. 安全恢复

WSL 服务异常时：

1. 保持机械臂禁用，不修改 ROS fail-closed 参数；
2. 记录 WSL `git status` 和 commit，按第 1 节同步完整批准版本并保留未知本地修改；
3. 停止旧终端中的 GraspNet-only 进程；
4. 用第 1 节的 `start_mujoco_digital_twin_wsl.sh` 启动组合服务；
5. 再次验证 `health.digital_twin.backend=mujoco` 且整体 `ok=true`；
6. 在 ROS 侧验证预测/仿真 URL 相同；
7. 重新点击生成候选，取得新的 `plan_id`，绝不复用重启前计划。

ROS 接口或节点更新后，重新构建、source 并重启 GUI、`grasp_task_node`、远程 6D 节点
及其服务客户端。不要通过以下方式“恢复”：

- 把 `allow_execution_on_error` 改成 `true`；
- 关闭 `enabled` 或 `execution_gate_enabled` 后继续真实运动；
- 改用旧 GraspNet-only、local legacy 或任何 mock；
- 手工复用旧 `plan_id`、旧 OBB 或旧仿真响应。

只有现场 camera-only 记录和真实 WSL MuJoCo 记录均完成，且最新计划获得完整匹配的
通过响应后，才可进入单独的真实机械臂授权评审。
