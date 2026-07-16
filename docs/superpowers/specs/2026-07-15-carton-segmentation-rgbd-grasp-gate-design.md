# Carton 实例分割、RGB-D 定位与 MuJoCo 抓取门控设计

## 1. 目标

在现有 Alicia-D 抓取系统中加入可切换的 Carton YOLOv8-seg 模型，并将实例掩膜真正用于三维定位、6D 候选生成、夹爪几何筛选和真机执行前的 WSL MuJoCo 门控。

本设计解决两个直接问题：

- 目标检测框包含桌面和背景深度，导致物体中心、姿态和 GraspNet 输入不稳定。
- “生成 6D 候选”经常需要重复点击，且当前筛选和仿真没有完整使用物体边界、纸箱尺寸及真实夹爪体积。

成功标准：

- 目标识别页面可在原始 YOLOv8、Carton detect 和 Carton segment 三种模型之间切换。
- Carton segment 模式在目标识别页面绘制实例轮廓，并发布与 RGB/深度帧同步的完整分辨率二值掩膜。
- 分割模式只使用实例掩膜内的有效深度计算中心、点云、OBB 和 6D 抓取候选；掩膜无效时拒绝规划，不退回检测框。
- 稳定场景中的一次“生成 6D 候选”请求，要么生成经过完整几何门控的计划，要么返回唯一、可操作的失败原因，不依赖反复手动点击刷新状态。
- 候选筛选考虑物体沿夹持轴的截面宽度、两根手指、掌部、支撑面和接近路径，而不只检查抓取中心。
- 真机执行前必须通过 WSL MuJoCo 的 IK、轨迹碰撞、双侧接触和抬升检查；WSL 错误或数据过期时禁止真机运动。
- 两个旧 detect 模型保持可用，并继续使用现有检测框深度路径。

## 2. 范围与非目标

本次范围包括：

- 第三个 YOLO 模型配置和运行期切换。
- 实例掩膜发布、GUI 轮廓显示、掩膜深度定位和物体点云提取。
- 深度相机的保边空间滤波、稳定帧融合和物体点云去噪。
- 纸箱三维 OBB、中心、主轴和支撑面估计。
- 基于物体几何和真实夹爪尺寸的候选过滤与排序。
- 富 6D 计划数据、动态纸箱模型、动态支撑面及 MuJoCo 失效关闭门控。
- 相应的单元、集成、协议、GUI 和模型冒烟测试。

本次不包括：

- 训练、微调或转换 `carton_segment_model/best.pt`。
- 同时运行或融合多个 YOLO 模型；任一时刻只运行用户确认的一个模型。
- 触觉、力矩、滑移、力控或测得接触力对候选排序和门控结果的影响。
- 根据仿真接触力自动调整真机夹爪电流或闭合力。
- 修改 TCP、URDF 或手眼标定结果。
- 自动执行真机抓取；操作员仍需显式点击执行。

MuJoCo 仍保留接触和刚体动力学，用于验证几何包络、双侧接触和物体抬升；“不考虑力反馈”表示本阶段不读取真实触觉/力矩数据，也不建立闭环力控判据。

## 3. 已确认基础与约束

- 现有模型切换设计位于 `2026-07-15-yolov8-carton-model-selection-design.md`。本设计在其基础上增加第三个 segment 配置；涉及模型数量和分割能力时，以本设计为准。
- `carton_segment_model/best.pt` 已验证为 Ultralytics `segment` 权重，唯一类别为 `carton`。
- 三种模型必须保留并允许运行期切换：
  - `yolov8n.pt`：通用 detect，类别由目标描述决定。
  - `carton_model/best.pt`：Carton detect，固定类别 `carton`。
  - `carton_segment_model/best.pt`：Carton segment，固定类别 `carton`。
- 分割模式采用失效关闭策略：没有新鲜、可靠、尺寸匹配的实例掩膜时，不生成 6D 计划，也不使用 bbox 作为替代。
- D405 当前深度比例由设备读取，实测配置为 `0.0001 m/unit`；所有滤波必须保留原始整数深度和该比例之间的一致关系。
- 现有 MuJoCo MJCF 已包含 Link7、Link8 和机械臂各连杆的碰撞网格，指爪总有效开口按当前配置的 50 mm 管理。
- 当前 MuJoCo 门控骨架存在，但 `execution_gate_enabled` 默认关闭，仿真物体还是固定鼠标复合几何体，需在本设计中改为动态纸箱几何。

## 4. 总体架构与数据流

```text
模型选择
   |
   v
YOLOv8ObjectDetector
   | bbox + 同一实例的完整分辨率 mask
   +---------------------------> 目标识别 GUI 绘制轮廓
   |
   v
PerceptionNode + 对齐深度
   | /perception/object_mask (mono8, 同时间戳)
   | /perception/object
   v
RemoteGrasp6DNode
   | 稳定帧融合 -> mask 点云 -> 支撑面 -> OBB
   | 只把 mask 内深度送往 WSL GraspNet
   v
候选几何过滤与排序
   | OBB / 夹爪扫掠体 / 支撑面 / IK
   v
富 6D 计划（四阶段位姿 + 候选宽度 + 物体几何）
   |
   v
GraspTaskNode -- /simulate_grasp --> WSL MuJoCo
   |                                动态 carton_box + 动态支撑面
   |<------ 明确的四项门控结果 -----|
   v
全部通过后才允许真机执行
```

数据必须按一次规划快照绑定。掩膜、深度、相机内参、目标消息、OBB、所选候选和富计划使用同一 ROS 时间戳；执行端不从多个“最新值”主题拼接计划，避免使用彼此不属于同一帧的数据。

## 5. 模型配置与运行期切换

在现有 `perception.yolo_models` 中加入第三个配置：

```yaml
perception:
  yolo_model_choice: "original"
  yolo_models:
    original:
      display_name: "YOLOv8 原模型"
      model_path: "yolov8n.pt"
      task: "detect"
      target_class_mode: "description"
    carton:
      display_name: "Carton 检测模型"
      model_path: "carton_model/best.pt"
      task: "detect"
      target_class_mode: "fixed"
      target_class: "carton"
    carton_segment:
      display_name: "Carton 分割模型"
      model_path: "carton_segment_model/best.pt"
      task: "segment"
      target_class_mode: "fixed"
      target_class: "carton"
      require_instance_mask: true
```

模型路径继续使用已有工作空间根目录解析规则。加载后必须校验 checkpoint 的实际任务与配置 `task` 一致；例如 segment 配置加载到 detect 权重时直接进入错误状态，不能让后续代码把不存在的 mask 当作正常结果。

模型切换沿用现有 `yolo_reload_generation` 和检测状态失效机制。切换、加载中或加载失败时，清除旧 bbox、mask、目标几何及待执行计划。两个旧 detect 模式不要求 mask，继续走现有 bbox 路径。

## 6. 检测结果与实例掩膜

### 6.1 检测器输出

`YOLOv8ObjectDetector` 的一次推理返回一个结构化结果，其中 bbox、类别、置信度、bbox 中心、实例 mask 和 mask 质心都来自同一个 Ultralytics 结果索引。

处理顺序为：

1. 按目标类别和置信度过滤实例。
2. 按现有目标选择/稳定策略选中一个实例。
3. 从该实例索引获取 bbox 与 mask，不能在 bbox 和 mask 列表中分别重新排序。
4. 把模型输入尺寸上的 mask 恢复到原始 RGB 分辨率，使用最近邻插值并阈值化为 `uint8` 的 0/255。
5. 计算 mask 面积、质心和与 bbox 的一致性；空 mask、非有限坐标或明显越界均视为无效。

detect 权重返回 `mask=None`。segment 配置若返回 `mask=None`，本帧检测不可用于定位和规划。

### 6.2 ROS 输出

新增 `/perception/object_mask`，类型为 `sensor_msgs/Image`，编码 `mono8`。其 `header.stamp` 和 `frame_id` 与产生它的 RGB/深度帧完全一致。

分割模式没有有效实例时，发布带当前时间戳的全零 mask，并同时发布 `ObjectPose.detected=False`。旧 detect 模式不把全零 mask 当作错误；消费者通过当前模型任务判断是否要求实例 mask。

### 6.3 GUI

目标识别页面在原有 bbox、标签和置信度之上绘制约 2 px 的实例轮廓。轮廓颜色与 bbox 保持一致，填充保持透明，避免遮挡深度或物体纹理。普通相机页面不增加轮廓。

GUI 状态区显示当前模型任务以及以下分割状态之一：`mask ready`、`mask stale`、`mask empty`、`mask size mismatch`。模型切换时立即清除旧轮廓。

## 7. 深度滤波与噪声抑制

采用三层处理，避免单纯增强时间滤波造成眼在手上相机运动拖影。

### 7.1 连续相机层

`RealSenseManager` 在深度对齐到彩色图后，对深度帧执行：

1. 使用现有 `depth_min_m` 和 `depth_max_m` 清除量程外值。
2. 启用保边空间滤波，参数由 `camera.yaml` 配置。
3. 保持输出尺寸与彩色图一致，不启用 decimation。
4. 默认不启用全局 hole filling，防止在纸箱/桌面边界制造假深度。
5. 默认不对连续视频启用持久时间滤波；该滤波可作为诊断开关，但不作为分割抓取的默认输入。

建议初始配置为：

```yaml
camera:
  depth_filter:
    spatial_enabled: true
    spatial_magnitude: 2
    spatial_smooth_alpha: 0.5
    spatial_smooth_delta: 20
    temporal_enabled: false
    hole_filling_enabled: false
```

如果当前 `pyrealsense2` 不支持某个配置项，启动时报告具体选项并退回未使用该项的深度流；segment 规划仍会由后续质量门控决定是否允许继续，不能静默声称滤波已启用。

### 7.2 规划快照层

收到一次“生成 6D 候选”请求后，`RemoteGrasp6DNode` 在最长 1.0 s 内主动收集稳定快照，而不是要求操作员再次点击。节点缓存最近 3 个同步的 RGB、深度、mask、目标和关节状态样本，仅在以下条件同时满足时做逐像素中值融合：

- 每帧年龄不超过 0.35 s。
- 相邻 mask IoU 不低于 0.85。
- mask 质心移动不超过 5 px。
- 六个机械臂关节的最大变化不超过 0.01 rad。
- 图像尺寸和相机内参一致。

任一条件失败时清空融合窗口，并在同一次请求的剩余时间内继续等待，不把运动前后的深度混合。1.0 s 内仍不能得到 3 帧时，本次请求以 `DEPTH_UNSTABLE` 结束并显示具体指标；按钮恢复可用，不存在静默超时或必须通过下一次点击刷新缓存的行为。旧 detect 模式使用 bbox/目标稳定性执行同类检查，但不要求 mask IoU。

### 7.3 实例和点云层

对融合深度执行：

- mask 向内腐蚀默认 2 px，减少物体边缘与桌面混合像素。
- 在 mask 内以中位数和 MAD 删除离群深度，默认保留中位数附近 `3.5 * MAD` 范围；MAD 接近零时改用毫米级绝对阈值。
- 只允许填补完全被有效 mask 深度包围的小型内部孔洞，不跨越 mask 边界。
- 使用扩展 bbox 的非 mask 上下文估计支撑面，避免把纸箱顶面当作桌面。
- 移除支撑面及其容差带内点后，对物体点云执行约 2–3 mm 体素降采样和统计离群点剔除。

每个快照产生深度质量指标：mask 面积、有效深度像素数/比例、深度中位数、MAD、支撑面内点率、物体点数和融合帧数。低于配置阈值时只发布失败原因，不发布可执行计划。

## 8. 三维物体几何

新增 `ObjectGeometry.msg`，作为感知/规划与执行门控之间的明确接口，至少包含：

- `Header header`
- `bool valid`
- `string label`
- `string source_mode`，取值固定为 `instance_mask` 或 `bbox_depth`
- `geometry_msgs/Pose pose_base`：OBB 中心和方向
- `geometry_msgs/Vector3 size_xyz_m`
- `geometry_msgs/Vector3 support_normal_base`
- `float32 support_offset_m`，与单位法向组成平面方程 `normal · point + offset = 0`
- `uint32 valid_depth_points`
- `float32 valid_depth_ratio`
- `float32 depth_mad_m`
- `string failure_reason`

OBB 计算规则：

1. 把实例点云投影到已估计的支撑平面。
2. 使用稳健 PCA 与最小面积矩形得到纸箱平面内长轴、短轴和中心。
3. 以支撑平面作为底面，以去除离群值后的高度百分位作为顶面。
4. 使用矩形中心和高度中点作为三维中心，长、宽、高作为 `size_xyz_m`。
5. OBB 姿态的 x 轴固定为平面内长轴、y 轴为短轴、z 轴为从支撑面指向物体一侧的单位法向，并保证右手系。
6. 平面内主轴按无符号轴处理；相邻快照选择与上一稳定结果夹角最小的等价方向，避免纸箱对称性导致 180 度翻转。

默认纸箱尺寸有效范围为每一轴不小于 5 mm、不大于 600 mm，高度不大于 500 mm；这些界限可配置。点数不足、OBB 退化、尺寸非有限或超出范围时，`valid=False` 并给出唯一失败原因。detect 模式使用现有 bbox 前景点云计算同一种 `ObjectGeometry`；它可以通过 MuJoCo 门控，但不会被标记为实例 mask 几何。

## 9. GraspNet 输入与候选选择

### 9.1 WSL 推理输入

segment 模式发送原始尺寸 RGB，但深度图中只有腐蚀后实例 mask 内的物体深度有效，其余像素为零。相机内参和深度比例保持不变。支撑面从本地扩展 bbox 上下文独立计算，不通过放回桌面深度来污染 GraspNet 目标云。

mask 必须与深度同尺寸、时间差不超过 0.35 s、非空且达到最小有效深度点数。任一条件不满足时不调用 WSL `/predict`。

detect 模式继续使用现有 bbox 前景深度路径，不伪造实例 mask。

### 9.2 几何硬门控

夹爪几何的单一真值来源是当前 50 mm MJCF/URDF 对应的 Link7、Link8 和掌部碰撞几何。配置中的 `max_inner_gap_m=0.05` 表示两侧夹持面的最大净间距，不等同于任意单个滑动关节的数值；启动检查必须验证它与 MJCF 两指关节范围和端点几何一致。候选宽度默认再增加每侧 2 mm、合计 4 mm 的深度噪声安全余量。

每个 WSL 候选转换到 `base_link` 后，依次检查：

- 抓取中心位于物体 OBB 内或允许的小容差带内，且不落在支撑面上。
- 候选的夹持轴穿过物体有效体积，而不是只与 bbox 相交。
- 沿夹持轴计算的纸箱截面宽度加安全余量不超过 `max_inner_gap_m`。
- 两根手指的可达中心和完整指爪扫掠体均高于支撑面安全间隙。
- 掌部和指爪在 pregrasp、approach、grasp、lift 四段插值路径中不穿过支撑面或物体非夹持区域。
- 接近方向不穿过支撑面，且满足现有 IK、关节范围和关节跳变限制。

候选报告的 `width_m` 仅作为模型建议；物理可行性使用 OBB 截面和真实夹爪几何复核。宽度过滤不再允许通过 `max_gripper_width_m: 0.0` 被整体关闭；诊断模式可以记录被拒候选，但不能让超宽候选进入可执行计划。

### 9.3 排序

通过硬门控后，排序优先级为：

1. OBB 中心和有效点云距离。
2. 夹持轴与纸箱平面主轴/短轴的一致性及截面余量。
3. 指爪和支撑面安全间隙、接近方向。
4. 当前关节状态到四阶段轨迹的运动代价。
5. GraspNet 模型分数。

模型分数不能补偿几何硬门控失败。没有候选通过时返回各门控的拒绝计数和首要失败原因。

## 10. 富 6D 计划接口

现有 `/grasp_6d/plan` 使用 `PoseArray`，无法携带所选候选宽度、OBB、支撑面和快照标识。新增 `Grasp6DPlan.msg` 并作为执行端的权威输入，至少包含：

- `Header header`
- `geometry_msgs/Pose[] poses`，固定顺序为 pregrasp、approach、grasp、lift
- `float32 score`
- `float32 candidate_width_m`
- `float32 required_open_width_m`
- `ObjectGeometry object_geometry`
- `string model_choice`
- `string plan_id`
- `string diagnostic`

`RemoteGrasp6DNode` 同时保留旧 `/grasp_6d/plan` `PoseArray` 作为可视化和兼容输出，但 `GraspTaskNode` 的真机执行只接受新的富计划主题。两者共享时间戳；旧主题不能单独触发执行。

计划在模型切换、目标丢失、mask/深度失效、超过现有计划有效期或新计划覆盖时立即失效。执行请求复制整条富计划后再开始门控，执行过程中不重新拼接最新 OBB 或宽度。

## 11. MuJoCo 数字孪生门控

### 11.1 ROS 端策略

配置改为：

```yaml
mujoco_digital_twin:
  enabled: true
  execution_gate_enabled: true
  allow_execution_on_error: false
  require_object_pose: true
  send_joint_state_in_request: true
  open_width_m: 0.05
  min_score: 80
```

ROS 请求包含：

- 当前且未过期的六轴关节和夹爪状态。
- 富计划中的四阶段位姿、候选宽度和计划标识。
- OBB 中心、方向、长宽高。
- 支撑面法向和偏移。
- 使用的夹爪模型标识和 50 mm 机械行程。

ROS 端必须显式检查 `simulation_ok`、`ik_success`、`collision_free`、`contact_success`、`lift_success` 全部为真，且 `score >= min_score`。缺字段按失败处理，不能只信任总分。

### 11.2 WSL 端动态场景

`/simulate_grasp` 将纸箱尺寸按 1 mm 量化后缓存 MuJoCo 模型，但每次请求独立设置物体和支撑面位姿：

- 新增 `carton_box`，用 OBB 的半尺寸生成 box 碰撞几何。
- 纸箱姿态使用 OBB 在 `base_link` 下的中心和四元数。
- MuJoCo 支撑平面使用检测到的平面法向/偏移，不再假定世界地面永远是 `z=0`。
- 机械臂、Link7、Link8 和掌部继续使用当前 MJCF 的真实碰撞网格。
- 尺寸、四元数、支撑面或关节状态非有限，计划标识为空，或数据年龄超限时立即拒绝。

### 11.3 夹爪闭合与门控判据

仿真从 `max_inner_gap_m=0.05` 的最大净开口开始，按经过端点几何验证的关节映射逐步闭合；所选候选的 `required_open_width_m` 必须位于机械范围内。闭合过程中记录首次双侧指爪/纸箱接触宽度，不再把关节位置直接强制到零后才判断接触。

门控依次验证：

1. 四阶段 IK 均收敛并满足位置/姿态误差阈值。
2. 四段插值轨迹中机械臂、掌部和指爪不与支撑面或不允许的物体区域碰撞。
3. 闭合过程中形成左右双侧接触，且没有先发生掌部撞击或单指把物体推离支撑面。
4. 按 lift 轨迹运行后，物体高度增加达到 `min_lift_success_m`。

任一项失败都返回结构化 `failure_code`、人类可读 `failure_reason` 和相关数值。门控结果只对应请求中的 `plan_id`；ROS 收到不匹配的响应时拒绝执行。

本阶段使用配置的纸箱质量和摩擦系数作为保守默认值，不使用真实触觉或力矩测量修正这些参数。

## 12. 错误处理与状态可见性

规划链路遵循单一首要失败原因，同时保留完整诊断计数。主要失败码包括：

- `MODEL_TASK_MISMATCH`
- `MASK_MISSING`、`MASK_STALE`、`MASK_EMPTY`、`MASK_SIZE_MISMATCH`
- `DEPTH_UNSTABLE`、`DEPTH_INSUFFICIENT`
- `SUPPORT_PLANE_INVALID`、`OBB_INVALID`
- `WSL_UNAVAILABLE`、`WSL_PREDICT_FAILED`
- `NO_RAW_CANDIDATE`、`NO_GEOMETRIC_CANDIDATE`
- `GRIPPER_MODEL_MISMATCH`、`GRIPPER_TOO_NARROW`、`GRIPPER_SWEEP_COLLISION`
- `MUJOCO_IK_FAILED`、`MUJOCO_COLLISION`、`MUJOCO_CONTACT_FAILED`、`MUJOCO_LIFT_FAILED`
- `PLAN_STALE`、`PLAN_ID_MISMATCH`

GUI 的生成状态显示当前阶段、失败码和简短中文解释。失败不会保留上一条可执行计划；再次点击会采集新的稳定快照，但不会复用失效 mask、OBB 或仿真结果。

日志记录模型选择、mask/深度时间戳、融合质量、OBB 尺寸、原始/各门控后候选数、所选宽度、MuJoCo 分项结果和 plan ID，便于区分“没有生成候选”和“候选被安全门控拒绝”。

## 13. 测试策略

实现遵循测试先行；测试不自动发送真实机械臂运动命令。

### 13.1 模型与掩膜

- 三个模型配置均可解析、切换和重复加载。
- detect/segment checkpoint 任务与配置不匹配时拒绝加载。
- bbox、类别和 mask 始终来自同一实例索引。
- letterbox/缩放后的 mask 能恢复到原图尺寸和正确位置。
- 空 mask、尺寸错误和模型切换会发布失效状态并清除旧轮廓。
- 目标识别页面显示 2 px 轮廓，普通相机页面不显示。
- 两个旧 detect 模式不因没有 mask 发生回归。

### 13.2 深度与 OBB

- 合成椒盐噪声和孔洞下，空间滤波保持物体/桌面边缘且降低 mask 内深度离散度。
- 稳定的 3 帧可融合；mask 跳变、质心移动或关节运动会清空窗口。
- mask 腐蚀能删除边缘混合深度，MAD 能删除飞点而不偏移主体平面。
- 合成旋转纸箱点云可恢复中心、长宽高和无符号主轴。
- 支撑面点不会进入目标云；点数不足和退化 OBB 明确失败。
- D405 `0.0001 m/unit` 在相机、ROS 消息和 WSL payload 中保持一致。

### 13.3 候选与夹爪几何

- 抓取中心在桌面或 OBB 外的候选被拒绝。
- 夹持轴未穿过纸箱体积的候选被拒绝。
- 沿窄边可放入 50 mm 夹爪的候选可通过，沿宽边超出行程的候选被拒绝。
- 手指中心虽安全但完整指爪/掌部扫掠体碰撞的候选被拒绝。
- 排序以 OBB/点云几何优先，模型高分不能越过硬门控。
- 没有候选时返回准确的分阶段拒绝计数。

### 13.4 富计划与 MuJoCo

- 富计划序列化包含四阶段位姿、宽度、OBB、支撑面和 plan ID。
- 旧 `PoseArray` 仍发布，但不能独立触发真机执行。
- WSL 可从 OBB 生成 `carton_box` 并设置倾斜/平移支撑面。
- MuJoCo 使用 Link7/Link8 碰撞网格和 50 mm 行程。
- IK、碰撞、单侧接触、超宽、抬升不足分别产生对应失败码。
- ROS 对 WSL 超时、缺字段、低分或任一分项失败均失效关闭。
- plan ID 不一致或计划过期时不调用运动服务。

### 13.5 实际权重与系统验收

- 使用 `carton_segment_model/best.pt` 做任务、类别和单帧 mask 冒烟测试。
- WSL `/health`、`/predict` 和 `/simulate_grasp` 协议测试通过。
- 先在 MuJoCo 中运行稳定纸箱场景，确认动态尺寸、支撑面和指爪碰撞可视结果。
- 真机联调前保持夹爪上方安全高度，先验证“仿真拒绝时无运动服务调用”。
- 在固定纸箱、稳定光照和 WSL 健康条件下连续请求 10 次；每次单击都必须得到计划或明确失败码，不允许静默、沿用旧计划或要求重复点击才能刷新内部状态。

## 14. 实施边界与顺序

后续实施计划按以下依赖顺序拆分：

1. 消息与纯数据结构、三个模型配置。
2. 检测器 mask 输出、ROS mask 发布和 GUI 轮廓。
3. 深度滤波、稳定快照和质量指标。
4. mask 点云、支撑面和 OBB。
5. 夹爪几何硬门控、排序和富计划。
6. MuJoCo 动态纸箱/支撑面、闭合逻辑和 ROS 失效关闭门控。
7. 全链路回归、实际 checkpoint 冒烟和安全验收。

每一阶段都先完成自动化测试再接入下一阶段。实现期间不修改 TCP、URDF、手眼标定，也不启用触觉或力反馈闭环。
