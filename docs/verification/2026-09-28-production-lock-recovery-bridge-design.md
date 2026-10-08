# 居中后的生产目标与历史故障桥接：最小接入方案

这是只读代码审查与接口设计，不表示当前已自然锁定目标，也不授予任何旧计划许可。必须等有界居中任务结束后的实际原始帧再评价支撑面。已有失败记录、手眼参数、4 mm / 4° 一致性门及接触高度 stress 配置保持原义。

## 现成入口与身份差别

- `/perception/unknown/{object,object_mask,detector_status}` 是实际生产结果。mask/object 的 header 使用原 RGB 源 stamp；status 提供 exact RGB-D、source/color/depth/TF stamp、selection generation/stamp、锁定/丢失状态、anchor 源、初始支撑平面/测得 footprint，以及当前 `geometry_metrics.support_plane_base`。
- `ObjectPose` 本身没有 target track ID。`remote_grasp6d_node._current_stream_target_identity()` 用 remote 自身的 identity generation 与 target epoch 生成 ID，二者不能用 grasp mode 的 selection generation 替代。应从同一实际请求的 `/grasp_6d/object_geometry.target_track_id`、`/grasp_6d/plan_enriched.target_track_id` 和 `/grasp_6d/pipeline_metrics.target_epoch` 读取、交叉核对，不自行拼接。
- `InitialAcquisition.observe()` 本身要求三个递增源帧、至少 120 测量点、原支撑面一致性和 RGB 精修 mask IoU ≥0.85。前两次尚未锁定时发布的 mask 是不可用零 mask，不能谎称三个初始生产实例 mask 都已发布。若需重建首次确认，使用袋内原 RGB-D 重放同一原始 acquisition 配置，并核对 confirming→ready 状态；也可另外收集锁定后的三个独立原始 ready 帧来证明持续唯一关联。二者在报告中区分。
- `.runtime/local_recovery_evidence.py` 已提供纯数据结构 `FaultIdentity`、`ActiveTargetIdentity`、`RecoveryContext.bind_active_target()` 和 `LocalEvidence.from_bound_observation()`。它允许新 driver epoch / selection / track 显式关联历史故障，不要求延续已经消失的旧 tracker ID；任何结果始终不是运动授权。目前它仍是原型，不能只填写布尔值就称证据成立。生产合法 epoch 允许 0，适配时若实际遇到 0，须按真实 schema 对齐原型整数校验，不能伪造 epoch=1。

`/grasp_6d/request_plan` 实际是开启连续推理，不保证调用即新帧；`/grasp_6d/replan_execution` 甚至可能提升缓存 preview。恢复流程必须用最终停稳后的新 source cutoff、原始请求证据和当前 SDK/实际起点判定新计划，不能把服务 success 当新观测证明。

## 电机真实支撑面与高出的标定板

`tabletop.segment()` 初次识别拟合图像中心 70–170 px 环，必要时外径退到 140/110 px；保持相同深度残差/比例/三象限门。其输入不含标定板边界，因此它无法凭类型区分高出红垫约 5.4 mm 的板面和真实电机邻桌面。进入中心 60 px 只满足初选条件，不证明选对了支撑面。锁定后 `_audit_current_support()` 仍在相同大环上拟合，它可能先把板面作为 anchor，随后把真实红垫识别为矛盾；也可能反过来。需要实际源帧证据，不能预先认定是哪一种。

对同一个最终停稳后的源帧，仅做以下离线比对：

1. 使用原 RGB、Z16、SDK rays、原相机参数与 exact `T_camera_board`，验证唯一电机深度连通域与实际 production mask 的测量像素所有权。明确比较域/分母，不能拿 2 px 稀疏诊断点集冒充完整实例 mask，也不把 visual seed 当 tracker。
2. 分别拟合板内部的独立深度平面和电机 22–55 mm 邻桌环的独立平面。后者完整排除板及其外侧 6 mm，保留既有点数、RMS、inlier 和覆盖限制。板坐标仅用于区域选择和同场景比较，板顶高度不能替代邻桌高度。
3. 把 status 的初始平面、当前 production 平面、SDK 邻桌平面与 SDK 板顶平面都变换到同源 camera 或 board 坐标。使用电机真实 mask 的测量点在邻桌面的正交投影构成 footprint；在全部 footprint 顶点比较有符号距离和法向。不能比较两个全局 plane offset 常数就声称高度差。
4. production 平面与邻桌在 footprint 上超过 4 mm / 4° 时，保持失败；若更贴合板顶，记录“生产支撑面与板顶一致、与邻桌不一致”的证据。若两者均相容或拟合分不清，报告不可辨识，不能强制选表面或放宽门。还须独立报告 production pinhole 投影与 SDK rays 在当前实例和支撑环上的差异；不能把坐标模型差异吞进高度补偿。

## 最小恢复上下文

历史档案保持不可变：旧故障 code、source、plan ID/digest 与完整记录 SHA。新 bridge 附件记录当前 driver epoch、mode selection、真实 remote ID/epoch、最终停稳 cutoff、三帧源与 raw bag/图像/mask 哈希，以及同电机关联和独立邻桌结果。历史失败→当前唯一电机是明确的事实桥接；不重新签发旧计划，也不把旧绝对板位置与本轮板布局相减。

最小生产接入只需一个验证附件，不需新大框架：将已验证的 recovery context digest、bridge digest、最新局部证据 digest、fresh 计划 digest 与 actual pregrasp witness 固定在现有 gate audit 旁的不可变 sidecar，以同请求 ID / target / source cutoff 查询。`mode_aware_grasp_task._validate_bound_plan_locked()` 在当前选择和已有故障检查之外，核对该 sidecar 与即将执行的完整新计划；它不能清 `_unknown_support_fault` 或 reset selection。现有全路径几何、MoveIt、SDK 起点、反馈和接触限制仍重新验证。观察后大幅重新调姿仍不合格，后续 contact plan 必须从实际到达的观察/pregrasp姿态生成，并在预先声明的局部关节范围内。

目前真正缺少的是：实际锁定后的 raw production mask 与 SDK 电机/邻桌的绑定适配器；同一新请求的 remote identity 读证；sidecar 在计划审核/执行处的强制关联；以及 observed-pregrasp witness 对后续路径跨度的约束。纯 `RecoveryContext` 校验和一张居中图都不能替代这些。
