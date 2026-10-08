# Expanded metric URDF + X：部署依赖只读审查

范围：仅为根代理在 prospective 验证通过后决定部署提供依据。本审查没有修改生产文件、ROS 参数、模型或使能，没有重启节点或执行运动。`.runtime/` 本文是唯一新增文件。验证是否通过及是否部署由根代理另行决定。

## 结论

本候选可以在 **driver、hardware interface、ros_control 控制器继续运行且保持原 SDK 字/epoch** 的条件下更新，但不是只写两个 ROS 参数即可完成。必须成对替换 URDF 与 X，并重建 TF/FK/IK/碰撞模型以及依赖旧坐标的目标、平面、计划缓存。关节 origin 是 RSP/MoveIt 的初始化模型内容，没有可用的参数热替换接口。

ROS 参数服务器、静态 TF 和多个进程之间没有跨进程原子提交。本项目现有入口也没有一个完成这项事务的服务。实际一致性应通过短暂关闭新的感知计划/执行接收，在没有活动轨迹的既有任务状态下，成对安装、定向重启模型消费者、设置新的输入时间截点，最后恢复接收实现。不能把驱动重启或关节下使能当作部署步骤。

**另一个必须处理的模型源是 WSL MuJoCo MJCF。** 它不会读取 `/robot_description`，现有 HTTP 请求也不传候选 URDF；只更新 ROS 会留下旧仿真运动学。须同步产生并验证对应的 MJCF，再清空该服务模型缓存/重启该仿真进程，保持已有夹持策略和“提升仿真仅诊断”策略。不能把旧 MJCF 返回的通过结果称为候选模型一致性验证。

## 冻结输入身份

候选目录：`/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/grasp_repair_resume_20260928T120606Z/expanded_metric_training_once/candidate_model`。

| 输入 | SHA256 |
|---|---|
| `alicia_expanded_metric_candidate.urdf` | `7812d6b69e854ed9f37b4e3cbd813ecd53dab3504065b2d04d287abc6fd852e0` |
| `d405_expanded_metric_candidate_eye_on_hand.yaml` | `637701accca5eb5042075c213c5e00e9b189d3cd7788d916c8c06bf89f7b1a0c` |
| `manifest.json` | `135e3272646ca07347ce1c305af60bfc66428e96fde6db7f77a255e9987f1182` |
| 成对摘要（manifest） | `841ca1491fb2e05bced1a1f962749a496a9c4b9944d2833eefa2c3f1b02c3789` |
| 原 URDF | `890592a24b8ad818d995e69beb670ea4254bf84f589615bfac3c8562f62449b7` |
| 原 easy_handeye 标定 YAML | `6ab0e8c902fe87bad1935a69b72044930b79ce1b239728cfce6dda8e94533468` |

注意：`6ab0e8…` 是原 easy_handeye 标定文件，不是仓库 `config/handeye.yaml`；后者 manifest 记录摘要为 `c25eedd4fd51f9ba865e558ba1ff1465745f2f431773f1b23d8e502ac2704865`。本轮只读查询中，运行 `/robot_description` 仍是原 `890592…`，`/handeye/calibration_file` 仍指向 `/home/zhuyupei/.ros/easy_handeye/d405_cc200_auto_20260927_validated_eye_on_hand.yaml`；此观测不是后续部署状态保证。

候选仅修改 Joint2–5 的 `origin.rpy`；joint name/type/axis/limit、origin.xyz 和其他 XML 内容不变。等效零偏（弧度）为 `[0, -0.10884477008762602, -0.04481243724566793, -0.01459230964441808, -0.005775713091347232, 0]`。关系是 `FK_new(q) = FK_old(q + delta)`，**这里 q 始终是原始编码器/SDK 坐标，不再在命令、反馈或关节限位里加 delta**。

## 驱动和控制器为何保留

这不是凭控制器类型作猜测，而是核对本机实现后的结论，且只适用于本次 joint type/name/limits 均不变的候选。

- `src/real-arm/alicia_d_driver/src/alicia_d_hw_interface.cpp:9,65,92,111`：初始化读取 `/joints` 并注册 `PositionJointInterface`；回调复制原始位置，`read()` 复制到 joint handles；`write()` 把控制器给出的数值直接发 `/joint_commands`，frame 为 `ros_control`。文件没有解析 URDF 或 origin/传动变换。
- `.../alicia_d_hw_main.cpp`：50 Hz `read -> ControllerManager.update -> write`。继续运行可保留控制器插补和 SDK 持有引用。
- 本机 `/opt/ros/noetic/include/joint_trajectory_controller/joint_trajectory_controller_impl.h:243-270`：初始化解析 URDF，按 joint name 找 joint，并仅以 `type == CONTINUOUS` 缓存 `angle_wraparound_`；没有由 origin 求 FK 或修正位置值。
- 本机 `/opt/ros/noetic/include/joint_trajectory_controller/hardware_interface_adapter.h:95-137`：Position adapter 的 `updateCommand` 直接 `setCommand(desired_state.position[i])`；`starting()` 会把命令设为测量位置，所以没有必要为了 origin 变化重启它。
- `src/real-arm/alicia_d_driver/config/controllers.yaml`：arm/hand 都是 `position_controllers/JointTrajectoryController`。
- `src/real-arm/alicia_d_driver/include/alicia_d_driver/sdk_position_codec.hpp` 与 `alicia_d_driver_node.cpp:2186,2235-2255,3344`：弧度与 0–4095 SDK 字直接编码/解码；成功写串口后的 wire frame 构成 retained SDK/control reference。不存在 URDF 零偏叠加。

因此不调用 controller unload/reload/switch，不重启 driver/HW，不改变 enable，也不执行 bootstrap/sync/归零。持有命令、原始 accepted q、driver epoch 和 SDK 字应跨模型切换连续；模型中的 tool/base 位姿数值则预期发生变化。

## 生产消费者与必须清掉的缓存

| 消费者 | 当前来源与缓存 | 本候选最小刷新方式 |
|---|---|---|
| robot_state_publisher | 初始化 KDL tree，存 `segments_`/`segments_fixed_`/`model_`；本机只暴露 logger 服务，没有 model reload | 新 `/robot_description` 后重建该进程；保留原 `/joint_states` 和硬件 |
| MoveGroup | 初始化 RobotModelLoader/RDFLoader、IK solver、collision robot、PlanningSceneMonitor；`/apply_planning_scene` 不是模型替换 | 重建 MoveGroup；随后恢复有据的静态环境，重新生成旧相机/目标派生场景，不能盲目复制旧 base 坐标物体或删除全局目标规避碰撞 |
| handeye_transform | `handeye_transform_node.py:9-45,65` 启动时解析一次，发送一次静态 TF 后 spin；存在 calibration_file 时文件 transformation 优先于 xyz/q 参数 | 同时更新标定文件指针和对应 `/handeye` xyz/q 后重建唯一 publisher；不能留两个 publisher 竞争同一 child frame |
| motion_gateway | `robot/moveit_planner.py:131-134` 启动时构造 RobotCommander、两个 MoveGroupCommander；另有 `_observation_planner`，branch hints、last plan、SDK 补偿/闭爪 lift adapter、observation context/scene | 新 MoveGroup 后重建 gateway；仅 description hash 改变会清部分 hints，不会重建 Commander 内部 RobotModel |
| remote_grasp6d | 构造时读取 handeye 配置（3473/3893）；TF buffer、冻结 X、RGB-D、near-field reference/fusion、候选/preview/commit、observe_return session、MuJoCo evidence cache | 重建进程并建立一个显式新 model/selection generation；旧历史 fault 归档保留，不复用旧 plan/scene/registration |
| unknown_tabletop_perception | 不直接读 URDF，但 exact RGB stamp 查询 base←camera；10 s TF buffer、base support anchor、track/acquisition/first_loss | 重建 TF buffer 和 tracker；新截点后的图像才用于新模型。只调用同 generation 的 selection 回调不会重置 |
| carton perception | 构造时读取 X fallback/TF 设置，另有 temporal tracker 与帧缓存 | 同批刷新；即使当前 unknown 活跃也不能让另一个活跃 publisher 继续输出旧坐标 |
| grasp_task | TF buffer、committed execution context、contact/lift/pregrasp 路径、following history。following FK cache 虽按 description 文本重建，旧计划身份不自动失效 | 重建 task 并绑定新 model generation；禁止旧计划/旧收据继续执行 |
| grasp_mode_router | 读取参数旧 generation 后递增；`begin_switch` 清 plans 并建立 stamp cutoff。相同 mode/strategy 也可显式生成下一 generation | 可保留 router，在上述消费者一致后只作一次显式模型切换。重启会继承旧 generation 而非必然回到 1，但初始化也会进行一次选择，不应重复重启/选择来洗掉错误 |
| WSL MuJoCo | `tools/start_mujoco_digital_twin_wsl.sh:9` 独立 `--model-xml`；server:415 缓存 key 只有 object 参数，1961-1980 从 MJCF 载入并缓存。HTTP `build_mujoco_payload` 不携带 URDF/X | 候选对应 MJCF + 成对模型身份验证；重建该仿真进程或显式清空模型缓存。仅修改磁盘 MJCF 无法让已有 object key 重新载入 |
| 原始 RGB-D、driver、ros_control | 本候选不改变 intrinsics/raw image、SDK 坐标或 joint types/limits | 保留运行 |

RSP 证据为本机 `/opt/ros/noetic/include/robot_state_publisher/{robot_state_publisher,joint_state_listener}.h` 和只读服务列表。MoveIt 构造路径还可核对[官方 Noetic RobotModelLoader 源码](https://docs.ros.org/en/noetic/api/moveit_ros_planning/html/robot__model__loader_8cpp_source.html)。MoveGroup 的动态重配置服务仅配置 scene monitor 行为，并不能把已持有的 RobotModel 换掉。

额外诊断进程（固定板 invariance、离线/在线 SerialFK 工具）可能自行持有旧 X/FK，必须结束当前取证窗口后重新建立模型身份；不得把切换前后的样本混进同一 frozen 几何实验。GUI 旧预览/计划和 RViz RobotModel 也应刷新，不能当成动作许可。

## 启动源与实际重载边界

- 当前 `.ros_log/ros_live_20260922_session/resume_modes.launch` 引入完整 `full_system_modes.launch`。**不要为更新模型重新执行全 bringup**：它包含 driver/HW/controller 初始化，而且会从默认 URDF 磁盘源重新加载参数。
- `src/real-arm/alicia_d_moveit/launch/planning_context.launch:9` 从 `$(find alicia_d_descriptions)/urdf/alicia_duo_with_gripper.urdf` 加载 `/robot_description`；MoveGroup 的 include 使用 `load_robot_description=false`。未来持久化需要修改明确的 startup 模型源/覆盖入口，否则下一次启动会恢复旧模型。本文没有修改该文件。
- `src/alicia_flexible_grasp_supervisor/launch/sensors.launch:8,14` 加载 `config/handeye.yaml` 并启动 handeye_transform。因此 persistent 文件指针/镜像 xyz/q 和当前参数要一起保持成对身份。
- RSP 和 motion_gateway 在现有 launch 中 `respawn=true`；MoveGroup/handeye publisher 没有同样的自动复活保证。不能简单逐个 rosnode kill 并假定所有进程会按顺序刷新。需要由根代理定向管理模型消费者及其 respawn，不能影响硬件进程。
- `src/alicia_grasp_modes/launch/perception_modes.launch` 是已有不含 driver/controller/camera 的感知/remote/task/router 重载入口；其 initial_mode/initial_strategy 必须保留当前真实模式，不能用默认 carton/two_stage 覆盖 unknown/observe_return。
- 核对到 remote 的 shutdown hook 只停止 streaming worker；没有找到这些 Python 感知/任务节点的 shutdown 关节 disable hook。此结论不授权活动轨迹中重启：已有任务/动作的结束和 hold 仍按原合同处理。

## 最短一致更新顺序（仅依赖设计，不是已执行步骤）

1. prospective 验证完成后，根确认候选 pair 和归档原模型身份；保留旧 support contradiction、丢锁与失败计划的证据。完成独立 MJCF 同坐标派生及只读 FK 对照，不能在 SDK 或 joint_state 加偏置代替。
2. 使用既有任务 inactive/动作无在途状态作为部署窗口，冻结新的识别计划/执行接收；不改变 torque/SDK hold，不伪造 task phase。
3. 在规划/感知消费者无法接收任务的窗口成对安装 persistent URDF、handeye 文件指针/配置以及相应 runtime 参数。处理唯一 static publisher 和 RSP 的重建；不要给旧帧时间戳混配新 X。
4. 重建 MoveGroup，再重建 gateway/remote/perception/task 的模型与 TF 缓存；对应 WSL MJCF 服务清缓存。所有旧 plan/context 无效。保留驱动、HW、controllers、RGB-D 进程。
5. 只读对照相同 accepted q 下 candidate SerialFK、`/compute_fk`、RSP TF；核对 tool←camera 恰为候选 X、SDK 字/driver epoch 原样；再建立一次新的 selection generation 和晚于切换完成的 source cutoff，收新 RGB-D/目标/桌面证据后重新规划。不能复用旧 preview，更不能靠新 selection 抹去旧失败原因。

同帧名的 tf2 buffer 可能留有切换前最多 10 s 的 dynamic TF，静态 X 却立即是新版；因此“查 latest TF 正确”不足以证明旧 RGB 时间戳正确。重建上述 buffers 并拒绝 cutoff 前帧是必要的缓存一致性措施，不是新的抓取几何阈值。

## 保持不变与尚未完成的接口

现有 20 g 未知物体上限、速度修复、提升仿真非硬门、根已启用的整条 lift SDK−actual 连贯修复、原有碰撞/追踪合同都保持原策略。缺失的是 **实际 paired deployment 的进程编排与 MJCF 成对派生/版本确认**，不是 driver 角度修正，也不是新抓取硬门。本审查只确定可复用入口和不能省略的缓存；没有声称已经部署、验证了候选当前抓取精度或完成实际提升验证。
