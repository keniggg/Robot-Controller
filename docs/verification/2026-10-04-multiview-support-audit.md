# 2026-10-04 多视角配准支撑一致性审计

本次仅增加跨视角配准的诊断信息。历史抓取偏高问题已由用户确认解决；当前补观察后的坐标一致性、现场约 19–20 mm 支撑差及其物理根因仍未解决。固定板约 10.6 mm 是坐标一致性误差，不是抓取高度测量。

## 修改范围

成功的 `RegistrationResult` 新增不可变 `support_consistency_audit`，并由规划节点的 `latest_registration` 审计输出显式序列化。失败结果和未携带此诊断的旧式结果不生成该字段。

诊断同时计算修正前后的法向夹角、局部中心处平面差及局部足迹内最大平面差。两组距离使用完全相同的足迹：最终一对一对应的 reference 目标点投影到 reference 支撑平面。记录对应数量、足迹质心和变换语义 `moving_snapshot_base_estimate_to_phase_reference`。这些投影点不冒充实测桌面点，也不证明整个支撑面可靠。

平面按 `x' = R x + t`、`n' = R n`、`d' = d − n'·t` 变换；法向和 offset 同步归一化。新审计的局部锚点与旧门的 median 锚点不同，因此原始 `support_plane_separation_m`、`support_normal_angle_deg` 继续保留原有验收含义，不能用新字段替代旧门。

原始支撑差 4 mm、法向差 4°、内点/重叠/RMSE、位移与 yaw 等配准门限均未修改；ICP 优化、接受/拒绝和融合处理未修改。本次没有修改 TF 或用固定补偿拟合现场误差。

## 离线验证

最终联合测试 **197 通过，0 失败，0 跳过，用时 6.52 秒**；修改文件的 `git diff --check` 通过。准确命令、选测路径和 node IDs、全部 197 个展开后的 node IDs、文件 SHA256 与结果保存在项目工件 [fusion_support_audit_validation_20261004.json](../../.runtime/fusion_support_audit_validation_20261004.json)。工件保存时重新收集并核对选测清单，同时确认实现和修改测试的哈希与成功运行后记录一致；没有冒充第二次测试执行。

有辨识力的验证包括：

- 仅 moving 目标点额外增加 6 mm Z、支撑面保持不变：旧配准仍接受、RMSE 近零；新审计报告修正后的支撑差 5 mm。禁用新审计与启用审计的配准和融合输出逐值相同。
- 目标与支撑共同作正常刚体变换：修正后的支撑差回到近零。
- 固定对应足迹在任意旋转、倾斜和不同坐标原点下协变；新增未匹配目标点不会扩大验证范围。
- 倾斜平面在足迹中心相交时，中心差为零但足迹最大差非零；法向与 offset 同比例缩放保持距离不变。
- 原 4 mm 门及其相邻浮点边界不变；成功诊断可严格 JSON 序列化，失败/旧式结果不伪造诊断，输出修改不会反向改变内部证据。
- 现有接触表面坐标及 observe-return / supplemental-view 消费测试保持通过。

## 已验证文件哈希

| 文件 | SHA256 |
| --- | --- |
| `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/multiview_surface.py` | `bbf3e3a863e9a5c104e682ebe33820a0e84f42c27425ac1f8d34a4ee41327e2c` |
| `src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py` | `51dfb4287cc5e78b11d2ece0840e7d2cace38417fb792e663906ead65e6e38f4` |
| `src/alicia_flexible_grasp_supervisor/tests/test_multiview_surface.py` | `faa09f8d87c91b5750ef5b20aa77b50c12cf2290e785b62249d98ed9712654e3` |
| `src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py` | `43bc640ac4f526ca3fcfd2c399b874796ad122f35907c5bc34c3d826e95fe53e` |

本报告仅确认软件离线验证。编写与验证过程没有调用 ROS 节点、硬件或部署动作；规划节点后续是否加载这些哈希及现场结果应另行记录。本项目 JSON 是代码测试工件，不是写入系统盘的实验或机器人运行日志。
