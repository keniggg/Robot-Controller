# YOLOv8 与 Carton 检测模型切换设计

## 目标

在 ROS 上位机的“目标识别”页面提供检测模型下拉框。操作员可在运行期间选择 Ultralytics 官方 YOLOv8 模型或工作空间中的自训练 Carton 模型，点击“确定模型”后让现有感知节点热加载所选权重，无需重启 ROS。

成功标准：

- 下拉框包含“YOLOv8 原模型”和“Carton 模型”。
- 原模型使用 `yolov8n.pt`，检测类别继续由现有目标描述解析逻辑决定。
- Carton 模型使用工作空间根目录下的 `carton_model/best.pt`，检测类别始终固定为 `carton`。
- 在两个模型之间往返切换时，当前模型、类别过滤器和 GUI 状态保持一致。
- 缺失或无法加载权重时，感知节点不崩溃、不继续输出旧模型的有效检测，并向 GUI 提供明确错误。

## 不在本次范围内

- 不同时运行两个模型，也不合并两个模型的检测结果。
- 不改动抓取、6D 位姿估计、相机采集或 TCP 标定流程。
- 不把本地模型上传、重新训练或转换成其他格式。
- 不持久化本次运行中的选择；重新启动完整 ROS 系统后使用 `camera.yaml` 的默认选择。
- 不在切换失败时静默回退到另一个模型。

## 现有基础

`perception_node.py` 每次处理最新帧前都会读取 `/perception` 参数。检测器签名包含 `yolo_model`、`yolo_target_class` 和运行时加载代次；任一值变化都会重建 `YOLOv8ObjectDetector`。因此模型切换和同选择重试都可以沿用 ROS 参数热更新机制，无需管理或重启节点。

当前 `carton_model/best.pt` 已验证为 Ultralytics detect 权重，唯一类别为 `{0: "carton"}`。

## 配置模型

在 `config/camera.yaml` 的 `perception` 段增加当前选择和两个模型配置：

```yaml
yolo_model_choice: "original"
yolo_models:
  original:
    display_name: "YOLOv8 原模型"
    model_path: "yolov8n.pt"
    target_class_mode: "description"
  carton:
    display_name: "Carton 模型"
    model_path: "carton_model/best.pt"
    target_class_mode: "fixed"
    target_class: "carton"
```

保留现有扁平参数 `yolo_model` 和 `yolo_target_class`，作为感知节点真正使用的运行时参数，兼容已有启动配置和测试。`yolo_model_choice` 用于 GUI 显示和规则选择；`yolo_models` 是下拉选项的单一配置来源。

GUI 每次成功完成模型路径预检后，还会在完整 `/perception` 字典内递增仅运行时使用的 `yolo_reload_generation`。该值不写入 `camera.yaml`，只用于区分操作员的每一次显式加载请求；因此即使模型、类别和路径都未改变，再次点击“确定模型”也会重新构造检测器，并可在修复依赖或损坏权重后重试同一选择。

默认仍为 `original`、`yolov8n.pt` 和现有 `mouse` 目标类别，因此未操作新控件时启动行为不变。

## 组件与职责

### 模型选择辅助模块

在 `alicia_flexible_grasp.vision` 下增加一个小型辅助模块，集中处理以下纯逻辑：

- 读取并校验模型配置。
- 根据选择返回模型路径和类别策略。
- Carton 模式返回固定类别 `carton`。
- 原模型模式接受目标描述解析得到的类别。
- 解析自定义相对权重路径。

GUI 和感知节点共用该模块，避免两处分别实现路径或类别规则。

### GUI

`PerceptionWidget` 在目标描述行上方或同一顶部控制区增加：

- 标签“检测模型”。
- `QComboBox`，显示配置中的两个 `display_name`。
- 独立的“确定模型”按钮。

GUI 初始化时读取 `/perception/yolo_model_choice` 并选中相应项目；未知值回退为界面上的原模型选项，但不会立即修改 ROS 参数。

点击“确定模型”时：

1. 读取当前目标描述并调用已有解析逻辑。
2. 根据模型配置计算 `yolo_model` 和 `yolo_target_class`。
3. 对 Carton 自定义路径做存在性预检；不存在则保持当前 ROS 配置不变并显示错误。
4. 递增 `yolo_reload_generation`，并一次性读取、更新和写回完整 `/perception` 字典，使选择、路径、类别和本次加载请求同步生效。
5. 清除 GUI 中所有可执行的旧状态，包括当前目标、预抓取位姿、待完成规划、已规划轨迹和锁定目标，并显示“正在加载”状态。如果预抓取规划或执行工作线程已经发出服务请求，失效其结果但继续持有“工作线程忙”状态；只有该线程返回或超时后才释放按钮，避免模型切换时启动重叠动作。

Carton 被选中后，“YOLO 类别”输入框显示 `carton` 并禁止编辑。切回原模型后重新运行当前目标描述解析，恢复类别输入框并允许编辑。

现有“开始识别”按钮仍更新描述、HSV 和抓取参数，但必须先根据当前 `yolo_model_choice` 应用类别策略：Carton 始终写入 `carton`；原模型继续使用已有描述解析结果。这样后续点击“开始识别”不会破坏模型选择。

### 感知节点

`PerceptionNode.refresh_detector()` 在构造 `YOLOv8ObjectDetector` 前解析模型路径，并把解析后的路径和 `yolo_reload_generation` 纳入检测器签名。模型、类别或加载请求代次变化时：

1. 先将当前检测器设为不可用并重置 `DetectionStabilizer`，避免发布旧模型的保持结果。
2. 立即在对象、稳定检测和原始检测三个输出上发布不可用状态（`ObjectPose.detected=False`、`detected=False`、`raw_detected=False`）。
3. 发布加载中状态并同步加载新模型。
4. 成功后设置新检测器并发布就绪状态。
5. 失败后保持检测器不可用、再次发布完整不可用状态、记录 `detector_error`、发布错误状态，并继续运行节点。检测器不可用期间，每次处理帧都会重复发布完整不可用状态。

检测循环在读取最新相机帧之前，每次迭代都独立调用 `refresh_detector()`。因此相机停止出帧时仍能观察新的 `yolo_reload_generation`、发布加载状态和完整不可用消息；有帧时后续推理路径保留原有刷新调用，检测器签名保证同一代次不会重复构造。

`GraspTaskNode` 收到 `ObjectPose.detected=False` 时清除最新目标和基于该目标缓存的待执行 6D 计划，新的规划或执行不能复用失效前的位姿。已经进入执行流程的传统抓取仍使用流程内复制并锁定的目标，不因共享缓存清除而突然改用其他目标。

感知节点通过一个 latched `std_msgs/String` 主题 `/perception/detector_status` 发布状态。消息使用稳定前缀，便于 GUI 判断：

- `loading:<choice>`
- `ready:<choice>:<resolved-model-path>`
- `error:<choice>:<message>`

GUI 订阅该主题，把状态转为中文显示。GUI 关闭时与现有对象订阅者一起注销该订阅，避免 Qt 对象销毁后收到回调。

## 模型路径解析

路径解析规则按以下顺序执行：

1. 绝对路径直接使用。
2. `yolov8n.pt` 这类无目录的 Ultralytics 标准权重名保持原样，允许 Ultralytics 使用缓存或首次下载机制。
3. 对含目录分隔符的自定义相对路径，先查找 ROS 包；若包位于 `<workspace>/src/<package>`，则先查找 catkin 工作空间根目录，再查找 ROS 包目录。
4. 已找到 ROS 包但无法推导 catkin 工作空间时，只查找 ROS 包目录。
5. 只有无法发现 ROS 包和工作空间时，才相对于当前工作目录查找。
6. 含目录分隔符的自定义路径仍不存在时，报告文件缺失。

在当前源码布局中，ROS 包位于 `<workspace>/src/alicia_flexible_grasp_supervisor`，因此即使 ROS 包目录或进程当前工作目录中存在同名文件，`carton_model/best.pt` 也优先解析为 `<workspace>/carton_model/best.pt`，不依赖 `roslaunch` 的进程工作目录。

## 数据流

```text
模型下拉框 + 确定模型
        |
        v
PerceptionWidget 计算模型路径/类别策略
        |
        v
/perception 参数字典原子更新（含递增加载代次）
        |
        v
PerceptionNode.refresh_detector()
        |
        +--> 重置旧检测状态并发布三路不可用消息
        +--> 解析模型路径
        +--> 加载 YOLOv8ObjectDetector
        |
        v
/perception/detector_status --> GUI 状态栏
        |
        v
所选模型输出 /perception/object
```

## 错误处理

- Carton 文件预检失败：GUI 不写入模型参数，保留当前有效模型并显示缺失路径。
- 配置缺少模型项或类别策略非法：拒绝切换并显示配置错误。
- Ultralytics、Torch 或模型反序列化失败：感知节点捕获异常，发布 `error` 状态并继续 ROS 循环。
- 模型加载过程中：旧检测器和稳定器已清空，不发布来自旧模型的“保持检测”。
- 模型加载和失败状态会使 GUI 取消当前目标、未完成/已完成的预抓取规划及锁定目标，并使抓取节点清除可用于新动作的目标缓存。
- 已经发出的预抓取规划/执行请求不会被当作不存在；其结果被标记为无效且按钮保持禁用，直到该请求返回或超时，再安全释放控件而不应用旧轨迹。
- 加载失败后：不自动回退；GUI 选择仍表示用户请求的模型，保留包含冒号在内的完整错误详情，并可通过再次确认同一选择重试。
- 切回原模型：重新解析当前目标描述，不复用 Carton 的固定类别。

## 测试策略

实现遵循测试先行，每个行为先加入会因功能缺失而失败的测试，再编写最小实现。

### 纯逻辑测试

- 原模型返回 `yolov8n.pt` 和描述解析类别。
- Carton 模型无论描述内容为何都返回 `carton`。
- 从 ROS 包目录正确推导工作空间，并解析 `carton_model/best.pt`。
- 标准权重名 `yolov8n.pt` 不被错误改写。
- 缺失自定义路径产生可操作的错误。

### GUI 测试

- 下拉框根据 `/perception/yolo_model_choice` 初始化。
- Carton 模式禁用类别编辑并写入固定类别。
- 切回原模型恢复描述解析和类别编辑。
- 点击“开始识别”不会覆盖 Carton 类别。
- Carton 文件缺失时不改写当前模型参数。
- GUI 能显示 loading、ready 和 error 状态，并正确注销新增订阅者。
- 下拉框尚未确认时，“开始识别”仍遵循 ROS 中已生效的模型选择。
- malformed/未知状态可见，包含冒号的 error 详情不会被截断。

### 感知节点测试

- 模型选择变化会以解析后的路径重建 YOLO 检测器。
- 切换时旧检测器与稳定检测状态被清空。
- 成功加载发布 ready 状态。
- 加载异常发布 error 状态、保留节点运行并使 detector 为 `None`。
- 同一选择的连续确认递增加载代次；首次构造失败后再次确认会真正重试。
- 加载或失败时对象、稳定检测和原始检测三路都发布否定状态，且检测器不可用期间持续发布。
- 相机无新帧时，检测循环仍观察加载代次并完成模型重载及三路失效发布，不调用推理。
- 已有正目标收到失效消息后，新的抓取规划/执行入口无可用目标；回归测试不调用运动服务。
- 模型切换发生在预抓取规划或执行工作线程期间时，禁止启动重叠请求；原线程完成或超时后释放控件，但忽略失效结果。
- 现有 HSV 检测选择行为保持不变。

### 验证

- 运行模型选择、YOLO 检测器、GUI 生命周期和相关感知测试。
- 运行功能包完整测试集，确认没有回归。
- 使用 `carton_model/best.pt` 做一次只加载元数据的冒烟验证，确认任务为 detect、类别为 `carton`；不启动相机推理，也不发送机械臂命令。

## 文档更新

更新功能包 README 的 YOLOv8 说明，记录下拉选择方法、Carton 权重位置、固定类别规则、运行期选择不会跨 ROS 重启持久化，以及错误状态的排查方式。
