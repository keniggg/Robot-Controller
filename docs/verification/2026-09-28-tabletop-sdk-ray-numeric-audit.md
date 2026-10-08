# 2026-09-28 tabletop 与 SDK 光线的同像素数值审计

结论：当前电机 ROI (267,155) 的同帧同深度 3D 点，SDK inverse-Brown 与 tabletop 针孔结果相差 **0.431 mm**，在该姿态的 base-Z 分量为 **+0.261 mm**。它是需要统一的模型差异，但远小于已经记录的约 10–11 mm 跨姿态支撑矛盾。没有改动任何模型、参数或控制节点。

## 实際电机 ROI

来源：外盘 `ros_log/bounded_joint23_counterrotation_20260928T125945Z_146409/baseline_geometry.json` 与闭合的 `evidence.bag`。图像 / 板姿态 / 深度 source stamp 精确匹配 `1790600386906598567`；没有用邻帧深度替代。模型为归档的 SDK 2.56.5、640×480 inverse_brown_conrady，fx=395.768280、fy=395.602753、cx=317.996124、cy=244.833984。RGB 修正关闭，深度已由 SDK 配准到 color，深度比例为 0.00009999999747378752 m/unit。

像素 (267,155) 的原始深度为 317.599992 mm。下表统一用 **SDK − tabletop**，因此符号不能反向解释成实际机械臂应移动量。

| 表达 | 差值 mm |
|---|---|
| optical [右,下,前] | [-0.205420, -0.379115, 约0] |
| camera_link [前,左,上] | [约0, +0.205420, +0.379115] |
| base [X,Y,Z] | [+0.084733, -0.332217, +0.261492] |
| 欧氏距离 | 0.431191 |

中心周围 5×5 像素均有深度，差的 min / median / max 为 0.402009 / 0.431500 / 0.461572 mm。该邻域是图像窗口，不是已完成分割的电机 mask，不能用其统计冒充目标整体接触误差。SDK float32 深度舍入造成 optical-Z 约 -0.000010 mm，不能当作真实深度模型变化。

## 传递链

`tabletop.py:_project_pixels` 接收 fx/fy/cx/cy 与已配准深度，使用 `P_camera_link=[z, -(u-cx)z/fx, -(v-cy)z/fy]`。SDK 对相同原始像素和 z 使用归档 inverse-Brown 系数反投影得到 optical 点，再用固定轴变换 `[x,y,z]→[z,-x,-y]`。

`unknown_tabletop_perception.py:195–209` 传入同帧 TF；点云经 `T_base_camera = FK(accepted) * X` 转至 base，进入桌面拟合、支撑一致性、候选位置/footprint，候选在该脚本 268–277 行发布 camera/base pose。板 tracker 则从 `RuntimeColorModel.normalized_points` 使用 SDK rays 做 PnP。两条路径目前确实有不同的射线模型。

对于固定的同一帧变换，`ΔP_base=R_base_camera ΔP_camera`：平移项抵消，刚体旋转保留误差长度。因此 0.431 mm 不会仅经过正确的刚体 TF 就放大成 11 mm。平面拟合、目标选择和姿态规划是后续非线性运算，不能把单像素差当作整个计划修正；必须对相同源像素重放才能量化。

## 已有近场记录

同样核对了 `grasp_diagnostic_20260928T084245Z/support_visibility_replay/{initial,pregrasp_settled,before_close}.npz`，保留原点云选择、深度、accepted-FK、X 和冻结支撑比较位置。既有 `grasp_repair_resume_20260928T104059Z/support_ray_model_sensitivity.json` 正是两种 ray 的同像素支撑重放，而不是变更内参后的新实验：

| 阶段 | 1800 同源支撑点差中位/最大 mm | 针孔支撑差 mm | SDK 支撑差 mm | 单由 ray 切换导致的差 mm |
|---|---:|---:|---:|---:|
| initial | 0.441 / 1.672 | 0.0714 | 0.0573 | -0.0141 |
| pregrasp_settled | 0.378 / 1.215 | 10.1800 | 10.1698 | -0.0102 |
| before_close | 0.537 / 1.042 | 10.9242 | 10.9454 | +0.0213 |

这些支撑点不全在电机轮廓内，表中的 1.672 mm 最大值不能套到 motor ROI。initial 中冻结目标的最近投影像素 (313,257)，深度 275.300 mm，两种模型仅差 0.001085 mm；后两个阶段冻结目标投影分别约 (348.24,491.93) / (356.77,558.81)，已经超出画面，不能伪造“近场仍看见电机”的单点比较。这也解释为何近场需保存实际共视桌面证据，并区分外推。

上述支撑重放固定了原环带选点，闭爪附近覆盖可能不足，统计只能诊断 ray 敏感性，不能授予运动或解除原支撑矛盾。

## 底座信息与可辨识性边界

用户已明确底座固定牢靠；按此记录。软垫照片不是底座运动证据，不以其归因，不要求改变现场。

现有 J2/J3/J5 与 J4/J6 的 accepted 编码器及腕上 RGB-D 都没有独立测量桌面相对于底座的变换。形式上，观测 `Y_i=A_i X C_i` 的变化既可以由模型中未观测的 base 刚体变化表示，也可以由姿态相关内部 FK/X 误差表示；增加轴方向可排除某些简单固定参数模型，却不能一般性地唯一定位到哪一物理部件。相同姿态返回可重复只说明误差有可重复性，不能单凭此指定来源。若将来确需区分，需要独立同时看到固定桌面参考和底座参考的外部观测；本轮未增加此要求或动作。

## 归档

完整结果与 source stamp / SHA 在 `grasp_repair_resume_20260928T120606Z/tabletop_sdk_ray_audit/report.json`，纯 SDK 数学脚本为 `.runtime/audit_tabletop_sdk_point_difference.py`，没有创建相机 context/pipeline、ROS node、publisher 或 service。SanStand 保留源码副本 `analysis_source.py`。脚本读取现有闭袋和小数组，采用 nice 10 / 单线程，未改生产源码。
