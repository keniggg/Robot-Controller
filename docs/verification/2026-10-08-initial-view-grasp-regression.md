# 2026-10-08 一次性抓取代码保存前回归核验

本说明对应用户要求保存并标注“10.8日，一次性抓取通过的完整代码”的当前源码。**10.8 是本次保存日期；实物夹取成功证据来自 10.6 UTC 的 gen8，当前完整源码没有新增实机抓取成功验证。** 以下检查均为离线测试、封存数据重放或源码 SHA256 对比，没有调用机械臂。

## 历史成功与源码可比范围

历史录制为 `grasp_records/20261006T061242.642572Z_f8b97892d0a84d5599287c74677f32d2`，plan 为 `ee718df75606887be44ea8ff`。同名操作目录 `ros_log/grasp_close_hold_20261006T061242Z/operator_confirmed_result.json` 保存用户“已夹住新目标”的实际确认。原程序 `unknown/HOLDING/GRASP_HOLD_UNVERIFIED` 保持原样；实物已夹住、按要求保持不提升，不能改写为触觉自动验证成功。

该计划的原始 audit 为 `observe_return / initial_contact / current_view_contact_snapshot`，即初始视角直接接触抓取路径，未先执行补观察。

成功之前的 `ros_log/contact_tracking_final_validation_20261006T055016Z/validation.json` 冻结了 65 项文件 SHA256，`ros_log/contact_tracking_gateway_reconcile_20261006T055605Z/result.json` 重复清单并记录部署通过。当前逐项比对为：

| 范围 | 相同 | 变化 | 缺失 |
| --- | ---: | ---: | ---: |
| 历史清单全部文件 | 64 | 1 | 0 |
| 历史标注的生产文件 | 10 | 0 | 0 |

唯一变化是测试文件 `.runtime/full_startup_candidate/test_full_startup_candidate.py`。以下 10 项生产源码与历史清单逐字节相同：

1. `src/alicia_flexible_grasp_supervisor/scripts/motion_gateway_node.py`
2. `src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py`
3. `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/contact_execution.py`
4. `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/contact_height_margin.py`
5. `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/pregrasp_gateway.py`
6. `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/pregrasp_compensation.py`
7. `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/contact_tracking_contract.py`
8. `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/contracted_trajectory.py`
9. `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/moveit_planner.py`
10. `tools/mujoco_digital_twin_server.py`

这是成功前的**部分文件基线**，不是成功时刻完整源码树快照。封存 gen8 目录中没有完整源码树或 git commit 的对应证明，因此不能声称当前整个工程与历史成功版本完全一致，也不能由上述 10 项推定相机、感知和规划源码全部未变。

## 当前直接抓取与公共执行回归

2026-10-08 对以下 8 个现有测试文件执行离线回归，结果 **652 passed、0 failed、0 skipped，5.00 秒**：

- `src/alicia_grasp_modes/tests/test_remote_modes.py`
- `src/alicia_grasp_modes/tests/test_observe_return_remote.py`
- `src/alicia_grasp_modes/tests/test_observe_return_visual.py`
- `src/alicia_grasp_modes/tests/test_mode_runner.py`
- `src/alicia_grasp_modes/tests/test_runner_observe_return.py`
- `src/alicia_grasp_modes/tests/test_observe_return_task.py`
- `src/alicia_flexible_grasp_supervisor/tests/test_grasp_task_sequence.py`
- `src/alicia_flexible_grasp_supervisor/tests/test_task_contact_admission.py`

覆盖初始接触优先、已选中或已可达接触候选不会被缺视角机会转去补观察、来源/目标/姿态/几何绑定、接触准入与有界修正，以及公共任务执行顺序。单元测试中服务、动作和时间由测试替身隔离；通过不代表实际机械臂再次抓取成功。

另直接读取 gen8 原始封存的 3 份同 plan audit，使用当前 `selection_audit_error` 与 `observe_return_audit_error` 检查，全部仍符合来源与 `initial_contact` 合同结构，未修改原 JSON。检查使用历史自身 selection 的时间边界，不验证当前新鲜度、IK 或实际执行。三份中包含后续 `TARGET_LOST` 状态；结构兼容不代表该状态可执行，**不能据此复活或执行旧计划**。

## 本轮支撑平面修复的影响范围

最终 `tabletop.py` SHA256 为 `0188c515ca9d9af98bc36d301ba337bda9e56287dbad98e920b3300f299e181a`。锁定支撑面分支只让 `Segmentation.plane_base` 保留传入参考系数的精确副本，避免再次归一化造成约 `1.11e-16` 的身份误差。计算及候选 `metrics['support_plane_base']` 仍使用原归一化值；首次拟合、支撑质量阈值和严格身份比较未放宽。

相关 9 文件回归 **425 passed、1 skipped**；跳过项为既有本地 RGB-D fixture 缺失。缩放平面测试覆盖实际 `segment → rgb_instance.refine_candidate` 的米制高度语义，节点测试覆盖缺测保持及实测恢复。

gen27 封存 RGB-D 与精确 TF 的离线重放在 10 个有精确深度的来源帧上对比修复前后：mask、位置、尺寸、质量不变，参考平面返回值从约 `1.11e-16` 差异变为完全相等；原支撑内点不足仍保持不可用。另 2 帧缺精确深度，未冒称已重放。此项只证明该数值错误修复及测量输出兼容，不证明异步运行、路径规划或新实机抓取成功。

部署回执记录仅重载 perception 与 remote planner，任务、网关、驱动等进程身份和实测姿态保持。后续 gen28 已出现实测支撑恢复，但未提交抓取运动；不能作为当前源码新的实物成功验收。

## 结论与证据位置

现有证据支持：历史列出的核心接触执行源码保持不变，当前直接抓取/初始接触合同回归通过，本轮平面修复没有改变已重放帧的候选几何或降低质量要求。感知输入和身份生命周期属于直接抓取的公共入口，相关后续改动仍可能影响是否及时形成可执行计划；不能保证所有后续修改均不影响一次性抓取。完整物理效果尚未重跑验证。

所有原始实验与本次测试日志保留在 SanStand：`/media/zhuyupei/SanStand/实验记录/Robot-Controller/`。本说明仅保存核验结论及索引，没有复制原始录制或改写实验结果。

- 本次审查：`ros_log/user_grasp_20261008T090751Z/source_snapshot_review/`，包含 `historical_source_hash_comparison.json`（65 项完整哈希比对）、`gen8_current_audit_contract_replay.json`、`direct_initial_contact_pytest.log/.xml`、`review_summary.json`（当前源码及证据 SHA256）。
- 本轮平面证据：`ros_log/user_grasp_20261008T090751Z/support_plane_repair/`，包含 `source_before.json`、`real_frame_replay.json`、`support_plane_final_pytest.log/.xml`、`deployment/result.json`。
- 历史 65 项清单 SHA256：`4c82234bf49846f255c101ad4c89097d393a8338df19e22a8db8ea35bbecea6d`。
- 本次 `review_summary.json` SHA256：`e3a1afd239d6dda005e76713ba759dd1a2aa7e1b6e9f8dbc394fc8c558760637`。
