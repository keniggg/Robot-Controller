# 2026-10-03 RGB-only 初始获取设计审查

## 当前范围和状态

用户当前目标约 5 mm 高、外观较暗。已有记录显示 RGB 区域可见，但深度不足以建立真实三维目标。约 5 mm 是现场诊断信息，不是允许写入目标专用动作、物体高度或抓取补偿的参数。历史抓取偏高问题已由用户确认解决；补观察后的跨视角一致性问题仍是另一项未完成任务。

本轮首先新增可独立验证的 RGB 稳定候选及支撑面射线观察兴趣体积，并进行只读现场验证。**这不等于完整 RGB-only bootstrap 已接入，不产生运动或抓取授权。** 本文为源码只读设计审查；本文编写者没有控制机器人。本轮运行结果、测试数量和代码版本应以对应验证记录为准，本文不预先宣称通过。

## 现有入口的循环依赖

1. `src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py` 的 `process` 先运行实测深度分割，再由临时 `TargetTracker.choose` 选择中心实例。返回 `no_center_instance` 时直接发布 unavailable 并退出，尚未进入 RGB 实例细化和 `InitialAcquisition`。
2. `src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_remote.py` 的 `_observe_return_silhouette_fallback` 需要已通过初始接触搜索的请求、已保存的实测表面、已有目标身份及有效 `GeometryEstimate`。它解决的是已建立目标的轮廓归属歧义，不是完全没有三维目标时的初始化。
3. `silhouette_observation_planning.py` 的 `_require_unknown_observation_prepared` 和 `_prepare_diagnostic_observation_prediction` 明确要求 `ObjectPose.detected=True`、有效 snapshot 和三维几何。
4. `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/observation_path_guard.py` 的 `FrozenObservationScene.from_plan` 校验已提交的 `Grasp6DPlan` 及完整 `ObjectGeometry`。`motion_gateway_node.py` 的 `_with_observation_planner` 在实际执行前还要求活动抓取任务持有该场景。
5. `mode_aware_grasp_runner.py`、`observe_return_task.py` 和 `observe_return_session.py` 继续绑定真实 `target_track_id`、源帧、已有表面及任务状态。因此仅删除 detected 条件，或给 RGB 区域填一个旧式三维几何和目标 ID，会破坏跨层语义和执行绑定。

## 本轮两种 observation-only 数据

### RGB 稳定候选

候选只说明在当前选择代次下，一个图像区域在若干不同源帧中持续出现。它可以为后续观察排序提供依据，不能证明物体类别、可抓取性、三维中心、完整轮廓或接触几何。

- `depth_valid` 比例只能作为诊断字段，不能决定 RGB 候选是否存在，也不能把缺深度像素补成实测点。
- 候选必须绑定图像像素坐标、图像尺寸、原始源时间戳和选择代次；稳定性不能由重复处理同一帧形成。
- 过期、乱序、选择代次改变、图像尺寸改变、候选歧义、明显位移或来源变化必须使历史稳定状态失效或按明确规则重新积累。
- 应保留 `ObjectPose.detected=false` 及原有三维获取门限；不能通过候选检测结果伪造 ready 状态。

### 支撑面射线观察兴趣体积

从完整 RGB 像素框边界发出相机射线，使用同源、独立通过质量要求的实测支撑面，以及通用的场景高度上界和相机工作深度范围，构造取景兴趣区域。

这些顶点是几何构造结果，不是深度测量，不得写入目标融合点云。高度上界是通用场景假设，不是该目标的实测高度。RGB 框可能没有包含遮挡部分或全部物体，故该体积 **不是完整目标体积，也不是完整碰撞包络**。即使所有顶点位于图像内，也仅预测针孔投影和深度范围，不证明无遮挡、可到达、碰撞自由、相机安装精度或最终能获得深度。

支撑证据必须记录精确源时间戳、平面系数、质量指标和可追溯的测量记录。该类型只能携带调用者已有的支撑验收结论，不能自行把 RGB、平面交点或假定高度解释为支撑测量。当前两种纯数据若未包含联合绑定，调用者必须核对同源像素框、图像尺寸、代次和相机/TF 配置，不能随意组合来自不同帧的候选与体积。

## 后续需要完成的独立执行路径

这些缺口是后续实施项，不是永久拒绝自主补观察。

1. 新增独立的 RGB 获取计划角色和有限 acquisition session。它以候选 ID 表示观察兴趣，不能冒充真实 `target_track_id` 或旧 `Grasp6DPlan`。
2. 为该角色提供独立的 execution owner、提交准入和撤销上下文。绑定选择代次、候选及场景摘要、RGB-D 源时间、TF/相机/手眼配置、控制器参考代次、计划摘要和共同截止时间。预览结果本身不授予执行权。
3. 在独立有效场景中规划非接触路径。兴趣体积不能独自承担避碰职责；必须另行取得实际场景和未知空间处理证据。保留原严格 MoveIt、计时曲线、控制器桥接、跟踪契约和夹爪/桌面间隔验证。
4. 优先选择一次即可补足深度或视角的信息收益较高、关节和工具运动代价较小的观察位。若信息仍不足，允许在同一个有限次数/时间预算内追加观察，不通过每次新请求重置预算。
5. 到位后仅接收停稳之后的新 RGB-D，用原 `InitialAcquisition` 和原几何门限重新建立真实目标。需核对该三维实例与原 RGB 候选的关联，不能只因新画面中心出现任何对象就自动交接。
6. 成功交接后销毁 observation-only 准入，创建新的真实目标身份/请求代次，再进入现有“初始信息充分直接抓取，不足再 observe-return”的流程。RGB 射线和兴趣体积不参与目标表面融合，不继续作为接触或闭爪依据。

## 可复用组件与不可绕过的边界

可复用 `observation_preview.py` 的只读路径证据、`observation_path_guard.validate_observation_trajectory` 的严格计时路径验证、`observation_view_guard.py` 的投影与实测桌面覆盖预测，以及 `observation_view_utility.information_motion_rank` 的信息/运动排序。复用的是算法和验证，不是伪造旧计划来借用执行权限。

手动接管、选择代次变化、源帧过期、候选歧义、TF/内参/手眼变化、SDK 参考代次变化、场景矛盾、轨迹或包络摘要改变、会话耗尽都必须撤销原授权。执行提交前应重新验证这些绑定。未取得真实三维目标时不闭爪、不提升；既有前景高度、预抓取到位、支撑一致性和融合门限均保持各自原有含义，不能互相替代或放宽。

## 后续验证重点

- 全部目标区域深度无效时，RGB 候选仍可被诊断，但三维检测和抓取资格仍为 false。
- 同一源帧重复、乱序帧、跨代次帧、尺寸变化与候选歧义不能制造稳定候选。
- 像素框边缘、ROS `camera_link` 与光学轴约定、倾斜桌面、平行/反向射线、工作范围边界都有明确测试。
- 支撑系数和证据源时间不匹配时拒绝生成体积；构造顶点不得被标记为实测点、完整碰撞几何或运动授权。
- 独立执行路径完成前，现场验证只读取图像、反馈和日志，不能将纯模块测试通过叙述成已完成自主 bootstrap 或抓取验证。
