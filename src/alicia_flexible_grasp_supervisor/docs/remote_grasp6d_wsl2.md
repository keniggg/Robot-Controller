# WSL2 GraspNet + MuJoCo 纸箱抓取运行手册

这条链路把 Ubuntu/ROS 侧的一次 RGB-D 规划快照送到 WSL2：同一个服务进程先在
`/predict` 生成 GraspNet 候选，再由 `/simulate_grasp` 对同一个富计划执行 MuJoCo
门控。真实机械臂只能在富计划仍有效、响应 `plan_id` 完全一致、五个安全布尔量均为
`true` 且分数达到阈值时继续。

> 本阶段不使用力、触觉或滑移反馈。MuJoCo 通过只说明几何和运动学前置条件满足，
> 不保证实物抓取成功。

## 1. 唯一允许的 WSL 主流程

每次验收都必须重新检查 WSL `:8000` 的组合健康状态。若 `/health` 没有健康的
`digital_twin` 分项，或 `/simulate_grasp` 返回 `unknown path`，说明当前仍是旧
GraspNet-only 代码；此时必须先把 WSL checkout 同步到本功能分支的批准版本。在记录
WSL commit 并通过组合健康检查前，不得把旧进程当作可降级运行模式。同步前先运行
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
checkpoint 和 Alicia-D mesh。`--warmup` 会在对外接受请求前加载一次官方 GraspNet
baseline 及现有权重。同一 server 进程内模型、CUDA context 和 PyTorch allocator
缓存应常驻 GPU；不得为每个 `/predict` 重新加载模型，也不得在正常请求后
调用 `torch.cuda.empty_cache()`。当前实现只允许在确认 CUDA OOM 恢复路径中清理
缓存；正常性应通过 `gpu_allocated_mb`、`gpu_reserved_mb` 和
`gpu_peak_allocated_mb` 观测。

ROS 主机与 WSL 主机必须使用同步的墙上时钟，并只接受
`/predict` protocol version `3`。启动前在两端分别运行 `date +%s.%N`，并检查
`timedatectl status`（如使用 chrony 再检查 `chronyc tracking`）。不应通过放宽
过期阈值掩盖时钟偏差；`snapshot_stamp_sec` 为零、位于未来，或结果年龄超过
`result_max_age_sec` 都会失效。

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

富计划执行端会强制要求 `enabled` 和 `execution_gate_enabled` 两项都存在且为精确布尔
值 `true`；缺失、`false`、数字 `1` 或字符串 `"true"` 都会在网络请求和物理动作前以
`MUJOCO_GATE_CONFIG_INVALID` 闭锁，不能用“关闭仿真”绕过安全门。

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

## 5. 持续 latest-only 候选流水线

默认 `/grasp_6d/remote/auto_request=false`。GUI 点击“生成 6D 候选”后向现有
`/grasp_6d/request_plan` 发送 `trigger: true`，按钮改为“停止生成候选”，节点进入
持续推理。再点击会向同一 service 发送 `trigger: false`，只停止新的候选推理。
命令行等价操作为：

```bash
rosservice call /grasp_6d/request_plan "trigger: true"   # 启动持续候选
rosservice call /grasp_6d/request_plan "trigger: false"  # 停止持续候选
```

GUI 现有“停止抓取”只调用 `/grasp/stop`，是机械臂执行急停，绝不用于推理
状态控制。“停止生成候选”也不调用 `/grasp/stop`。启停推理不会清除已有的
有效 Execution readiness；是否仍允许执行只由 Execution 富计划的完整性、新鲜度和
后续安全门决定。

调度器任何时刻最多允许一个 WSL 请求正在执行，以及一个待处理的最新
快照。新快照覆盖旧的待处理快照；旧 generation、旧 request ID、目标 epoch 不匹配或
返回时已超过 `result_max_age_sec=1.2` 秒的结果只记录 metrics，不得进入候选池。
这个上界不能通过只提高 timer 频率来绕过。

每个请求仍从滚动 RGB-D-mask-object 数据中组成稳定快照：segment 模式要求
精确同时间戳的 RGB/depth/object/mask，detect 模式要求 RGB/depth/object。源帧到完整
样本的推理流水线延迟不得超过
`planning_snapshot_max_inference_latency_sec=1.2` 秒；样本完整后又必须在
`planning_snapshot_max_age_sec=0.35` 秒内采纳。三帧窗口还需通过：

- mask IoU、质心漂移和关节变化；
- mask 腐蚀、内部小孔处理、有效深度比例和 MAD 去异常值；
- 支撑面拟合、目标点云和 base-frame OBB；
- GraspNet 候选、真实夹爪几何、可达性及视野门控。

每个已采纳快照只按其精确时间戳查询一次 `T_base_optical`。ROS 端用固定
OpenCV-optical → ROS-`camera_link` 轴映射派生请求内只读的
`T_base_camera_link`；普通候选选择与 `full_scene`/普通审计共用同一个请求局部
冻结变换。WSL 推理期间即使机械臂继续运动、latest TF 已变化，也不会重新查询或改变
该批候选的 center、tool0 和姿态。生产候选坐标固定为 `opencv_optical`，配置或运行时
改成 `ros_camera_link` 会以 `CANDIDATE_FRAME_CONVENTION_INVALID` 在 WSL、MoveIt 前闭锁；
配置项是合同断言，不是可切换的生产坐标模式。

连续返回的本地安全候选按目标实例、中心距离、姿态角距离、接近方向和开度进行
跨帧匹配，平行夹爪的 tool-`Rz(180°)` 按物理等价姿态处理。初始合同固定为最近
5 个不同请求至少命中 3 次，然后稳健融合位置、姿态和开度。只有稳定候选的
前 `moveit_top_n=5` 个变体进入严格 MoveIt；生产参数只允许 3～10。

### 5.1 初始频率与分阶段提速

首次实机验收使用 `request_hz=1.5`、`performance_window_size=100`。在相同场景、相同模型和
相同缩放设置下收集至少 100 个 terminal metrics，同时记录端到端 P50/P95、过期率、
替换率、稳定候选率和 GPU allocated/reserved/peak。只有这些指标稳定且没有隐藏
积压时，才依次调为 `2.0`、`3.0`、`4.0`、`5.0` Hz；每一档都重新收集独立窗口。
若 P95 接近/超过结果年龄上限、`expired`/`replaced` 比例上升、稳定率下降或
`gpu_reserved_mb` 持续增长，立即回退上一档；不得只把定时触发频率调高后宣称
吞吐已提升。

初始生产值为：

```yaml
grasp_6d:
  plan_validity_sec: 5.0
  remote:
    request_hz: 1.5
    result_max_age_sec: 1.2
    stability_window_size: 5
    stability_min_hits: 3
    tracking_position_threshold_m: 0.025
    tracking_orientation_threshold_deg: 25.0
    tracking_approach_threshold_deg: 20.0
    tracking_width_threshold_m: 0.008
    target_instance_association_threshold_m: 0.08
    target_absolute_sanity_distance_m: 0.15
    moveit_top_n: 5
    replan_position_delta_m: 0.012
    replan_orientation_delta_deg: 12.0
    replan_target_drift_m: 0.025
    replan_cooldown_sec: 1.0
    selection_hysteresis_ratio: 0.12
    candidate_consecutive_invalidations: 2
    performance_window_size: 100
```

这些参数不改变 `/gripper/open_position_m=0.05` 等物理夹爪硬合同；不得为增加候选数
放宽非法深度、TF、碰撞、关节限位或最大开度门。

### 5.2 纯 GraspNet 输入模式 A/B

`/grasp_6d/remote/graspnet_input_mode` 只改变送入同一个 GraspNet 的 RGB-D 点集，
不会从目标 OBB、轮廓或支撑面人工合成俯抓/侧抓候选。远端返回的每个候选都保留原始
`candidate_index`，ROS 端只恢复平行夹爪严格等价的 identity / `Rz(180°)` 两个
`variant_index`；同分时按远端候选顺序、再按变体顺序稳定决胜。三个模式为：

- `masked_target`：默认生产模式，只发送当前目标的分割深度。detect/bbox 路径使用原有
  深度前景，不把同一个 bbox 派生的伪 mask 再当作独立证据做 bbox-mask IoU；真正的
  instance-mask 路径仍执行该一致性检查。
- `context_roi`：发送目标深度，并只追加 mask 周围 ROI 内、与同一快照支撑面相距不超过
  `graspnet_input_support_band_m` 的局部支撑点。它强制要求 instance mask、开启候选目标
  门、有效且位于 `ros_camera_link` 的支撑面；目标点云/OBB 仍只来自目标 mask，绝不被
  上下文点污染。
- `full_scene`：发送同一快照的完整深度，仅供诊断。它仍强制 instance mask，但不强制
  候选目标门或支撑面；WSL 返回后即以
  `GRASPNET_FULL_SCENE_DIAGNOSTIC_ONLY` 闭锁，不能进入候选选择、可达性、MoveIt、
  MuJoCo 或发布有效计划。

模式与所有阈值在一次请求开始时冻结；任一前置条件或构建门失败都原样闭锁，不会自动
回退到 `masked_target` 或 `full_scene`。审计记录输入模式、ROI、目标/支撑/总点数、目标
占比、内部孔填补、bbox-mask IoU，以及实际发送 depth 与有效 mask 的 SHA-256。
所有最小点数必须至少为 1，目标占比和 bbox-mask IoU 必须位于 `(0, 1]`；把门限设为
零、负数、非有限值或错误类型会以 `CONFIG_INVALID` 在 WSL 调用前闭锁。位置回退、姿态
回退和 identity / `Rz(180°)` 以外的变体列表同样是不可放宽的生产合同，启动时或运行时
参数刷新发现变化都会拒绝规划。
当前只完成离线单元/协议验收，`context_roi` 的真实相机 + WSL 现场 A/B 尚未验收；在取得
同一场景重复样本前继续保持 `masked_target` 默认值。

相机验收时使用：

```bash
rosservice call /grasp_6d/request_plan "trigger: true"
```

相机-only 阶段不要调用 `/grasp/start`，取得证据后运行
`rosservice call /grasp_6d/request_plan "trigger: false"` 停止持续候选。

## 6. 诊断主题与命令

```bash
rostopic echo -n 1 /perception/detector_status
rostopic echo -n 1 /perception/object_mask/header
rostopic echo -n 1 /supervisor/camera/color/image_raw/header
rostopic echo -n 1 /supervisor/camera/depth/image_raw/header

rostopic echo -n 1 /grasp_6d/status
rostopic echo -n 1 /grasp_6d/object_geometry
rostopic echo -n 1 /grasp_6d/gate_audit
rostopic echo -n 1 /grasp_6d/pipeline_metrics
rostopic echo -n 1 /grasp_6d/preview_plan_enriched
rostopic echo -n 1 /grasp_6d/preview_plan
rostopic echo -n 1 /grasp_6d/plan_enriched
rostopic echo -n 1 /grasp_6d/plan
rostopic echo -n 1 /grasp/state
```

应记录的证据包括：

- color/depth/mask/object 的源时间戳；
- `valid_depth_points`、`valid_depth_ratio`、`depth_mad_m`、`fused_frames`；
- `support_inlier_ratio`、目标点数和 OBB 三维尺寸；
- WSL raw/NMS/collision/returned 候选数及本地逐级 gate counts；
- 每个 request ID/generation/target epoch、snapshot stamp、WSL 分段耗时和端到端延迟；
- 候选在目标匹配、几何、可见性、支撑面、稳定性、MoveIt 各阶段的剩余数；
- 稳定拒绝码的计数/比例，以及全部失败时的 `primary_failure`；
- GraspNet `center` 与 Alicia `tool0` 的 camera/base 坐标、depth 偏移及契约残差；
- snapshot stamp/source frame、原始/规范候选坐标约定和冻结变换 SHA-256；
- `candidate_width_m`、`required_open_width_m`、四阶段位姿和 `plan_id`；
- MuJoCo 的五个布尔分项、分数、`failure_code` 和 `failure_reason`。

`translation_m` 始终表示 GraspNet 夹持中心，不是机械臂 TCP。执行位姿按
`tool0 = center + depth_m * model(+X) = center + depth_m * tool(+Z)` 只换算一次；
生产模式只接受 `depth_m ∈ {0.01, 0.02, 0.03, 0.04}`、固定 `Ry(+90°)`
模型到工具映射、工具 `+Z` 接近轴，以及 identity / `Rz(180°)` 两个平行夹爪变体。
缺失、非有限、越界 depth 或不一致的 tool0 一律在 IK、MoveIt 和 MuJoCo 之前闭锁。

`/grasp_6d/gate_audit` 是有界摘要，只包含完整报告路径、SHA-256、行数、冻结变换
hash/frames/stamp 及选中候选的 center/tool0 证据；完整 `T_base_optical`、
`T_base_camera_link` 和逐候选 rows 以原子替换方式写入 `gate_audit_output_path`，避免把
数百至上千行 JSON 塞入 ROS `String` 话题。

规划审计是强制安全门，默认完整文件为 `~/.ros/grasp6d_gate_audit_latest.json`。
`gate_audit_enabled` 不是现场可关闭开关：它必须保持布尔值 `true`，且输出路径不能为空；
初始化、运行时刷新、严格 JSON 写入或摘要发布失败均阻断有效计划。每个远端原始候选的
identity / `Rz(180°)` 两行都必须包含候选合同、六阶段解析几何、目标门、严格可达性、
排序和最终选择结果，不能只记录通过项；上述任一阶段抛异常时也必须保留完整失败行并
继续记录其余血缘。有效计划只接受 rows 与 selector evaluation 一一对应、恰好一个
`selected=true` 且该血缘与最终 selected candidate 完全相同的完整报告。

规划审计文件与 MuJoCo 执行审计文件必须是两个不同的实际文件。路径会经过 `~`、绝对
路径和真实路径规范化，并检查已存在文件身份；使用 `..`、符号链接或硬链接把两者指向
同一文件会以 `AUDIT_PATH_CONFLICT` / `MUJOCO_AUDIT_PATH_CONFLICT` 在推理或仿真网络调用
前闭锁。

组合 WSL `/predict` 响应会同时给出 `protocol_version`、`candidate_fields`、候选数组和与
数组分离的 `diagnostics`。ROS 客户端强制要求精确整数版本 `3`，以及有序字段
`score,width_m,height_m,depth_m,translation_m,rotation_matrix`；缺失、旧版、增删或乱序
均按远端协议失败。现场应保存 raw、NMS 后、碰撞过滤后和 returned 数量；这些诊断只用于
审计，不得成为推理输入或改变候选。每次请求在网络前清空上一批诊断，传输失败不能沿用
旧 evidence；整个响应（包括未知额外字段）必须是严格 JSON，隐藏的 NaN/Infinity 也会
闭锁。畸形 JSON 或非法 Content-Length 必须返回结构完整的失败包络。

`/grasp_6d/pipeline_metrics` 是有界 `std_msgs/String`，`data` 必须是
`allow_nan=false`、紧凑且 key 排序的严格 JSON。下面只是字段/格式示例，数值不是
RTX 4070 或实机实测结果：

```json
{"accepted":98,"busy":12,"completed":107,"decode_ms":2.1,"drop_reason":"","encode_ms":4.8,"end_to_end_ms":438.2,"event":"request_completed","expired":2,"failed":0,"generation":4,"gpu_allocated_mb":2350.0,"gpu_peak_allocated_mb":2610.0,"gpu_reserved_mb":2780.0,"latency_p50_ms":401.5,"latency_p95_ms":612.7,"pending_replacements":7,"primary_failure":null,"rejection_counts":{"TARGET_TOO_FAR":18},"rejection_ratios":{"TARGET_TOO_FAR":0.06},"replaced":7,"request_id":105,"result_age_ms":472.0,"ros_prepare_ms":19.6,"snapshot_stamp_sec":1784420000.125,"stage_counts":{"moveit_checked":{"entered":5,"passed":3,"rejected":2},"stable":{"entered":12,"passed":5,"rejected":7}},"stale":7,"started":100,"status":"PREVIEW_READY","submitted":107,"target_epoch":3,"transport_ms":31.4,"wsl_inference_ms":326.7,"wsl_postprocess_ms":42.0,"wsl_preprocess_ms":11.6,"wsl_total_ms":380.3}
```

`latency_p95_ms` 是最近 `performance_window_size=100` 个有限端到端样本的 95 分位；
它不是 WSL 纯 inference 时间。性能日志应按运行批次保存原始 topic 和环境信息：

```bash
mkdir -p ~/.ros/grasp6d_perf
rostopic echo /grasp_6d/pipeline_metrics | \
  tee ~/.ros/grasp6d_perf/pipeline_metrics_$(date +%Y%m%d_%H%M%S).log
nvidia-smi --query-gpu=timestamp,memory.used,memory.total,utilization.gpu \
  --format=csv -l 1 | tee ~/.ros/grasp6d_perf/nvidia_smi_$(date +%Y%m%d_%H%M%S).csv
```

用下面的读取脚本连续检查“已提交但未 terminal”不超过 2，并确认忙时新快照体现为
`replaced` 增长、过期/旧 generation 体现为 `expired`/`stale`，而不是 FIFO 队列不断增长：

```bash
python3 - <<'PY'
import json
import rospy
from std_msgs.msg import String

rospy.init_node('check_grasp6d_latest_only', anonymous=True)
for _ in range(100):
    metric = json.loads(rospy.wait_for_message(
        '/grasp_6d/pipeline_metrics', String, timeout=10.0).data)
    outstanding = metric['submitted'] - metric['completed']
    assert 0 <= outstanding <= 2, metric
    print(metric['request_id'], outstanding, metric['replaced'],
          metric['expired'], metric['stale'], metric['latency_p95_ms'])
PY
```

该检查需与 request ID/generation 日志、WSL 同期日志和 GPU 采样一起存档；单看 GUI
刷新频率或定时器周期不能证明无积压。

候选漏斗中，碰撞、关节限位、物理夹爪开度、非法深度/TF和目标完全丢失仍是硬
拒绝；接近方向、中心距离、画面居中、支撑面余量、关节运动量和模型分数通过软成本排序。
`stage_counts` 用于定位目标匹配、几何、可见性、支撑面、稳定性和 MoveIt 的逐级损失；
`rejection_counts`/`rejection_ratios` 用于根据实际分布调阈值。所有候选失败时必须有非空
`primary_failure`，应先处理占主导的真实硬门或数据质量问题，不能把“无可用候选”当作
可调试的最终原因。

## 7. 50 mm 夹爪硬限制

执行合同固定为 `Alicia_D_v5_6_gripper_50mm`。解析门控先把 OBB 沿候选 jaw axis
投影，再给两侧各加 2 mm 余量：

```text
required_open_width_m = OBB 在 jaw axis 上的投影宽度 + 2 × 0.002 m
```

必须满足 `0 < required_open_width_m <= 0.050`。候选网络给出的 width 只用于诊断，
不能越过这个 OBB 投影硬门。门控还使用完整 Link7/Link8 指爪与 Link6 掌部包络检查
支撑面和物体碰撞。

50 mm CAD 合同中的单指基准盒尺寸为 `[0.0434, 0.0286, 0.0600] m`，两指公共
中心在 tool0 下为 `[0.0004, 0.0003, -0.0302] m`。解析碰撞盒在每个轴的全尺寸上
增加 `0.0012 m`，使全开和全闭两端的 Link7/Link8 CAD 六个面都保留至少
`0.0005 m` 余量。Link6 掌部盒尺寸为 `[0.1175, 0.1550, 0.0774] m`，中心为
`[-0.0393, 0.0003, -0.09344] m`；这些偏移均以 Alicia tool0 为基准，不能改用
GraspNet center 或 tool0 原点近似。

MuJoCo 编译模型会验证实际碰撞 mesh 满开内间隙约为 `49.9375 mm`，且与 50 mm
合同误差不超过 0.5 mm。仿真从满开逐步闭合，保留首次双侧接触的完整状态和宽度，
再沿检测到的支撑法向抬升；不会把指爪强制闭合到零间隙后才判断抓取。

## 8. 富计划与 `plan_id` 生命周期

两阶段候选流水线的 topic 权限严格分离：

- `/grasp_6d/preview_plan_enriched` 和 `/grasp_6d/preview_plan` 是持续更新的 Preview，
  只用于界面/RViz、稳定分析和重规划判断，从不授权物理运动；
- `/grasp_6d/plan_enriched` 是唯一 Execution 执行授权；
- `/grasp_6d/plan` 是 Execution 的派生 `PoseArray`，只供可视化，自身也不是授权。

每个通过稳定、硬安全和有界 MoveIt 检查的结果可更新 Preview。只有机械臂未执行且
提升控制器通过冷却、迟滞、目标漂移和候选连续失效判定时，Preview 才能以新的
固定 `plan_id` 提升为 Execution。提升审计写入
`~/.ros/grasp6d_gate_audit_latest.json.execution`，与持续 Preview 审计分离；审计写入、
内容绑定或发布失败都不得留下部分授权。

Execution 富计划包含：

- 与快照绑定的 header、模型选择和非空 `plan_id`；
- 固定顺序的四个位姿：pregrasp、approach、grasp、lift；
- 候选宽度、50 mm 门控后的 required width；
- 同一快照生成的 OBB 和支撑面。

纸箱质量与摩擦不在 `Grasp6DPlan` 消息或 `plan_id` 规范字节中；构建 schema-2 请求时，
它们由 live `/mujoco_digital_twin/object_model` 受控配置追加。现场记录必须同时保存该
配置，不能把运行时质量/摩擦误称为富计划自身字段。

`plan_id` 由快照纳秒、模型选择、四个位姿、宽度、OBB 和支撑面等规范化内容生成。
模型切换、目标/mask 完全丢失、非法 TF/深度、几何硬门失败或执行急停会以失效
tombstone 撤销授权。新的有效 Preview 不撤销或替换 Execution。默认 Execution
富计划有效期为 5 秒。

`grasp6d_node.py`（`use_remote_grasp6d=false`）只能发布这个 legacy 可视化结果，不能
产生执行授权。legacy topic 即使包含四个位姿，也绝不能启用“执行 6D 抓取”。

执行请求必须携带当前 `/plan_enriched` 的精确 `plan_id`。成功 start 会在设置 active
的同一临界区深拷贝并摘要绑定这条 Execution；后续 MoveIt、MuJoCo 和动作检查只验证
该不可变副本。执行期间后来的有效 Preview 或有效 Execution 更新不会改写已绑定计划；
无效 Execution tombstone、目标安全事件或“停止抓取”会撤权。`StartGrasp` MD5 不匹配的
旧 GUI/节点必须重建并重启。

显式重规划只接受 `trigger: true`，且机械臂执行中会被拒绝：

```bash
rosservice call /grasp_6d/replan_execution "trigger: true"
```

机械臂空闲时，只在当前候选连续失效 2 次、位置变化超过 `0.012 m`、姿态变化
超过 `12°`、目标漂移超过 `0.025 m`，或受理的显式 replan 时考虑新 Execution；
`replan_cooldown_sec=1.0` 限制重规划频率，有效挑战者还需满足
`selection_hysteresis_ratio=0.12`，避免计划拖拉、抖动或来回切换。

富计划的 pregrasp 只能先调用 `/supervisor/check_pose_strict` 生成严格位姿缓存，再由
`/supervisor/execute_pose_strict` 执行同一个完全匹配且类型为 `strict pose` 的缓存。
缓存缺失、被覆盖或位姿不匹配时不得重新规划、调用 `go()`、替换姿态/位置或退回普通
`/supervisor/move_to_pose`。approach、grasp、lift 继续走对应的 Cartesian cached-only
执行；严格执行服务缺失会在 MuJoCo、开夹爪和任何物理动作前闭锁。

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
每次仿真尝试还必须把请求/负载/响应哈希、请求与回显 `plan_id`、五个布尔量、分数、
网络返回后的 authority 复核和最终判定原子写入
`~/.ros/grasp6d_mujoco_audit_latest.json`。审计采用严格 JSON；路径无效或写入失败时，
即使仿真响应通过也不能进行物理运动。响应对象的任何字段（包括未参与评分的额外诊断）
包含 NaN、Infinity 或不可严格 JSON 序列化的值，同样按 `WSL_UNAVAILABLE` 闭锁并留下
严格 JSON 审计。
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

本阶段的离线单元/集成测试、协议检查、camera-only 检查、WSL/RTX 4070 性能采样、
MuJoCo 检查乃至机械臂空载干跑，都不保证真实抓取成功。真实相机噪声、手眼标定、
目标材料/摩擦、夹爪尺寸与回差、驱动器动力学和紧急停车路径仍必须逐项实机验证。
