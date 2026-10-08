# 完整提升路径偏置与双占据离线验证

本次仅新增纯推导/验证模块，没有注册 ROS 服务、修改生产提升入口、加载节点或执行硬件。用户保留的实际提升功能、提升仿真非硬门、未知物体 20 g 上限和现有提速修复均未改变。

## 已确认的软件问题

`grasp_task_node.py:3715` 的提升仍调用普通 `move_to_pose_linear`。`moveit_planner.py` 从当前 accepted 状态规划，controller-reference timing 只调整时间，不为所有关节路径点保留已有 SDK−accepted 偏置。控制器从当前 SDK hold 桥接到第一个正时间点，因此进入普通提升时会撤回之前保持的偏置。

历史精确记录为 `grasp_records/20260928T084603.735476Z_0270fad562cf46a9ae14b8c203da5bc1/keyframes.bag` 中 `/move_group-3-1790585355.952945109`，接收时间 1790585355.953595。其 13 个路径点、闭爪 accepted 反馈、SDK、epoch、最后 0.8 s 反馈窗口、原始 rich plan、URDF 和运行配置已提取到 `tests/fixtures/lift_following_20260928.json`，未用当前模型改写历史。

- accepted 开度：10.000 mm。
- SDK−accepted：[0.087890625, 1.494140625, 0.439453125, 0.087890625, 0.263671875, 0.3515625]°。
- 只将请求 `t=0` 点替换成 SDK 时，Noetic 实际首段系数与原轨迹逐项相同；该点本来会被跳过。
- 保留全部 13 个位置点的同一偏置，保持原速度、加速度、时间不变，能使“恒定 following bias”反事实模型下的终点误差从 13.494755 mm 降至浮点零；整首段假设 encoder 与 nominal 最大差为 3.55e-15 rad。

以上为模型内算术与指令连续性结论，不是机械臂已成功提升、实际偏置恒定或抓取成功的证据。

## 新纯模块

`robot/lift_path_continuity.py` 提供：

1. `FrozenLiftContext.from_plan`：绑定 rich plan 完整摘要、原 OBB/支撑面、canonical bilateral contact evidence、URDF SHA、SDK/accepted 关节、epoch、两个时间戳及同一 accepted 开度。不会重新识别或替换旧失败计划。
2. `derive_lift_path`：复用 `compensate_cartesian_path` 的既有 0.035 rad 最大 following 偏置合同，对**全部**位置点添加同一 SDK−accepted；保留导数、时间和 nominal 视觉目标。
3. `occupancy_at`：同时保留“目标仍在桌上”与“目标刚性随 tool”的两种互斥假设。两者不是同时存在的两个实物，故不相互碰撞检测；从未从 world 移除目标，也未确认 attachment。
4. `validate_lift_trajectory`：对给定 stationary hold→首个正时间点的 Noetic 样条和后续全部样条递归区间证明，覆盖 dropped t=0、关节限位、闭爪 CAD、支撑面和上述两种目标占据。低层函数明确是 conditional supplied hold；应由 `validate_lift_pair` 从冻结 SDK/accepted 派生合法的两个 hold。
5. `validate_lift_pair`：重验冻结输入，逐字节确认派生路径完全一致，分别检查完整 nominal 路径和完整 shifted 路径；首点、末点单独修改，导数或时间改变都不能复用证书。

### 接触与连续包络

每个样条区间的关节变化由精确 Noetic 多项式导数界给出。全姿态关节运动半径给出 gripper/body 位移界及 carried OBB 位移界；Minkowski 盒扩张覆盖中间状态，不能仅以关键点采样通过代替连续证明。支撑净空保持现有 gripper 3 mm，假设携带物不能进入支撑面。恰好贴面且无法得到非负区间证明时会报 unresolved，不添加向上或向下偏置来取得通过。

允许碰撞关系仅来自初始姿态已证明的左右指接触。对每个指使用保守 CAD 指盒与目标 OBB 的凸交集，检查交集投影，而不要求整件长物体投影落入指垫。连续区间将固定世界目标转换至 tool 并按逆运动位移界扩张，交集覆盖全部可能接触。所有交集投影顶点及其连线必须处于有凹槽的真实 STL 内侧指垫多边形内，Y 向只允许既有内半指厚度范围；palm、camera、mount 没有接触豁免。指盒是 CAD 包络，拒绝可能来自包络保守性，不能等同实际 STL 碰撞。

三面求交的近奇异非零行列式不会被当作相关面静默跳过：仅明确重复/相反法向或精确零列的结构相关情况可跳过，其余无法可靠求解即 unresolved。求解器计算顶点误差包络，超过 1e-9 m 不发接触证书；投影与 Y 上许可边界使用 1e-8 m 向内余量。左右指 0.5/5/20 nm 的定向边界测试分别覆盖拒绝、拒绝、通过。

### 证书边界

提供 `ToolBodyBox` 可以加入有来源 SHA 且已映射到 runtime tool0 的 camera/mount 保守盒。未提供时报告明确列出 `missing_rigid_tool_body_models`，不会声称整机完整覆盖。现有历史 URDF 无 camera collision，手眼外参不是机身碰撞模型；相机官方尺寸、安装 frame 与支架几何尚需独立对齐。

这些纯函数不认证实时新鲜度、stationarity、实际 following/stopping、手眼误差、整个机械臂 MoveIt 场景碰撞或真实抓取。所有输出的 `motion_authorized`、`attachment_confirmed`、`hardware_response_verified` 均为 false。

## 离线结果与未通过项

最终联合回归：155 passed in 6.21 s，其中新 lift suite 为 35 项。其余为既有 contact_linear_compensation 和 observation_path_guard，覆盖完整偏置、t=0 语义、上下文失效、双占据不删除、carried 支撑、额外相机体碰撞、样条内部碰撞、有限指垫和近奇异面。

旧 lift 的 nominal、旧 SDK bridge、完整 shifted 三个几何回放均在初始左指 CAD 包络/目标 OBB 交集处得到 `unresolved finite pad coverage`。这不能说明实物手指碰撞非法，也不能归因于 lift 算法：旧抓取存在约 10–11 mm 坐标链/场景偏差，且 CAD 指盒本来保守。没有放宽几何、重选目标、清空正矛盾或确认 attachment 来制造通过。

最终回放目录：
`/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/grasp_repair_resume_20260928T120606Z/lift_path_continuity/reviewed_20260928T140441Z`

保存完整 JSON、原/派生 ROS wire 轨迹、源码摘要及反事实首段逐点数据。联合回归日志在上级 `regression_after_numeric_review.log`。早期回放和测试日志仍保留。

## 生产接入前仍需完成

当前生产完全未调用本模块。明确接入点为：

- `grasp_task_node.py:3715` 的 `LIFT_OBJECT` 调用需要独立 lift 服务/上下文；不能放宽仅允许 approach/grasp 的 `_pregrasp_context` 来偷渡 pose index 3。
- `motion_gateway_node.py:173` 附近当前只注册 `move_contact_approach` 与 `move_contact_grasp_pose`。新的 lift handler 应单独校验 task stage、已提交 rich plan、现 epoch、最新静止 SDK/accepted、accepted 开度及单次缓存消费，执行前重绑所有证据。
- 从 accepted 含闭爪开度的 RobotState 规划完整 nominal path 后，调用纯推导和双路径验证；任何重定时须重新绑定并验证最终 Noetic 曲线。不能只复用旧开爪 observation guard，也不能只改第一点。
- 对完整 nominal、shifted 和真实 SDK bridge 做当前 MoveIt 全臂场景验证，保留桌上目标假设与携带目标假设；仅允许经过证明的指垫接触。相机/支架体来源与 runtime frame 需要补齐或明确证书覆盖界限。
- 使用抓取附近新鲜、同源 motor/table 证据解决旧坐标链矛盾后再决定实机实验。现有失败保持归档，仍需真实闭爪/提升反馈来验证偏置是否保持以及物体是否抓住。
