# 2026-09-28 当前支撑面矛盾漏报修复

本验证全程离线，无 ROS 发布、服务调用或机械臂运动。原实验用户已确认小电机仍在桌面。

## 原因与修改

`segment_with_support_fallback` 原来先按冻结平面 ±2 mm 过滤支撑点。只要冻结分割还能给出身份匹配的候选，就不拟合实际当前桌面；若旧平面附近无足够点，则只报 `locked_support_plane_no_longer_visible`。因此真实桌面在坐标链中的漂移可能漏报，也可能被当作普通遮挡。

现在在冻结过滤前，独立从未经冻结平面筛选的有效目标环带拟合当前支撑。仍要求至少 70% 内点、RMS ≤2 mm、至少三个象限各 20 点，再按原始 immutable anchor 和完整 footprint 比较原来的 4° / 4 mm 限制。缺少深度、低内点比例、RMS 或象限不足只标记审计不可用，不生成正矛盾。原候选匹配、歧义判断及 target lost latch 保持不变。当前平面重分割路径也先检验 RMS/象限，再比较冻结平面。

审计只投影原始 ring 像素，顺序与完整图像相同；保留原来的 1,800 点上限与 128 次确定性 RANSAC，避免重复构造完整 640×480 点云。

## 实录对照

从 `scene_diagnostic.bag` 提取原 generation 2 detector status 的冻结 anchor、extent、plane、footprint；使用 `geometry_forensics/geometry_report.json` 中对应精确图像戳与接受关节 FK/手眼变换。生产 pinhole tabletop 投影保持原样。

- 中途接近帧 `1790585261041049003`：旧路径仍给出一个身份匹配候选，冻结环带内点比例仅 **0.00511734**。
- 同一帧独立当前拟合：**93.6667%** 内点，RMS **0.839875 mm**，**4** 个象限；冻结目标/footprint 上最大偏差 **6.015866 mm**，明确超过 **4 mm**，新路径正确报 `support_plane_local_distance_inconsistent`。
- 初始帧仍返回原身份匹配候选，当前审计偏差 **0.138127 mm**，通过。
- 闭爪附近帧当前桌面拟合虽可得约 11 mm 差异，但只有 **2** 个象限，不作为正矛盾；旧的 missing support 结果保持。不能利用这些弱覆盖帧直接决定向下补偿。
- 9 帧缓存深度回放证明投影优化前后全部判定及指标逐项一致。

这些证据修复了已证实的软件漏报；仍不能单独区分误差来自手眼、机械臂运动学或其他坐标链环节，也未证明物理抓取已成功。

## 验证与部署范围

离线相关测试 **95 passed, 1 skipped**（缺未分发的历史本地 RGB-D fixture），6 个既有 `cv_bridge.tostring` 弃用警告。命令：

```bash
source devel/setup.bash
/usr/bin/python3 -m pytest -q src/alicia_grasp_modes/tests/test_tabletop.py src/alicia_grasp_modes/tests/test_support_plane_contract.py src/alicia_grasp_modes/tests/test_support_observation.py src/alicia_grasp_modes/tests/test_component_budget.py src/alicia_grasp_modes/tests/test_target_identity_contract.py src/alicia_grasp_modes/tests/test_rgb_instance.py src/alicia_grasp_modes/tests/test_unknown_stream.py
```

变化只需加载 `unknown_tabletop_perception` 节点；`remote_grasp6d` 不导入本分割包装器，无需为此修改重启。

原记录及回放证据：`/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/grasp_diagnostic_20260928T084245Z/support_visibility_replay/`，包括原帧 `.npz`、`report.json`、`final_validation.json`、`timing.json`、`tests.log`。回放脚本留项目 `.runtime/replay_support_visibility_20260928.py`、`.runtime/verify_support_visibility_cache.py`，并在外盘保留副本。未上传或删除历史。

独立审计三个真实帧各三次测得 **30.8–100.4 ms**，中位 **48.8 ms**。这只是离线审计耗时，运行时完整 RGB 精化/队列时效仍需节点监督。

最终 `tabletop.py` SHA256：`39d46cf49206f90caa96de026104dd5ae7947d27e11e96ec1b8cf1d629aed19a`。
