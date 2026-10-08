# 2026-09-28 近场预抓取时长修复

实录：`grasp_records/20260928T093250.556161Z_85aa129d3f024ebf92ea8a579d82a503`。所有回放均离线，未发布 ROS 控制；本报告不证明实机跟随或抓取成功。

## 具体原因

任务节点实际发布 `MOVE_PREGRASP` 与 `planning 6D near-field pregrasp` / `moving 6D near-field pregrasp`。网关既有 `_strict_pregrasp_timing_scope` 只识别不含 `near-field` 的两种标签，漏掉二阶段抓取的近场预抓取。因此候选评估用已配置的预抓取 J3/J4/J6 = 0.04 rad/s，而实际执行沿用接触阶段的 0.02 rad/s，并关闭了既有共线 C2 时间参数化。

最小修复只把上述两个精确标签加入该作用域；未知物模式、MOVE_PREGRASP、非 Cartesian 路径、既有限速、规划器锁与异常恢复均保持原合同。相似后缀、观察动作、误差补偿、接近、抓取、提升、carton 模式均不因此进入该配置。

## 各时间因素

实际 action goal 有 29 点，时长 **89.408441885 s**。J3 总行程 **1.261784251858 rad**。

- MoveIt retiming 基准约 **25.235685054 s**（按最终精确时长及两级因子反算）。
- 保持局部轨迹速度不超过 0.02 rad/s 的 `velocity_time_scale=2.5`，得到 **63.0892126 s**。
- 连续控制器 spline 证明又得到 **1.41717479346**，最终 **89.408441885 s**；限制段是最后的 `segment 27`、J3。该步骤覆盖插值内峰值及真实 SDK 初始保持位置，和仅检查轨迹点/相邻段速度的前一步不同，不能直接删除。
- 候选日志的 **41.466 / 41.488 s** 是 `pregrasp, approach, grasp, lift` 四段总路程/限速下界，不是最终 action 时长承诺。加减速、连续插值及 SDK 衔接还需证明。实际重新规划的预抓取单段下界：0.02 配置 **63.089212593 s**；0.04 配置 **31.544606296 s**。

同一录制关节路径、同一真实 SDK hold 离线对照：

| 配置 | 最终时长 | 连续速度证明 | 最大加速度 |
| --- | ---: | --- | ---: |
| 实录，未进入 C2 / 快速预抓取作用域 | 89.408442 s | 实录原合同通过 | 本报告未另行推算 |
| 只用既有 C2，仍沿用 0.02 rad/s | 71.153247 s | J3 ≤ 0.0190001 rad/s | 0.007181 rad/s² |
| 修复作用域，既有 C2 + 已配置 0.04 rad/s | 35.576624 s | J3 ≤ 0.0380001 rad/s | 0.028723 rad/s² |

修复对照保留原始 29 点的位置和顺序，连续单调共线证明通过；包含真实 SDK hold 到首个正时间点的速度及加速度证明，没有额外伸长。该对照约减少 60.2% 的指令时长，不应写成实机提速验证完成。实际运行仍需保留最终轨迹 CAD、跟随、控制权及视觉几何合同。

## 验证和证据

- `test_unknown_pregrasp_speed.py`、`test_strict_collinear_timing.py`、`test_controller_reference_timing.py`：111 passed，33.64 s。
- `test_motion_gateway_controller_start.py`：33 passed，1.28 s；本次共 144 项离线回归通过。
- 近场实录回归夹具：`tests/fixtures/near_field_pregrasp_timing_20260928.json`，源轨迹 wire SHA256 `ce4479e6cade0c7ebb4ee224d4ba5463bc2f2417c6fd2e3b451ed564993ddf7e`。
- 精确回放数值与完整证明：外盘实验目录 `timing_review/recorded_path_timing_replay.json`。
- 原 action 轨迹及 SDK hold：同目录 `recorded_goal.json`。
- 测试日志与 JUnit：同目录 `regression.log` / `regression.xml`、`gateway_regression.log` / `gateway_regression.xml`。

补偿未执行的原因独立于提速 bug：同次任务以 `PREGRASP_COMPENSATION_FAILED: SUPPORT_GEOMETRY_CONTRADICTION` 结束，新观测支撑面与冻结面相差 11.213 mm，超出现有 4 mm 几何一致性阈值。这个时长补丁不移除或放宽该拒绝，也没有修复手眼/机械臂跨姿态几何误差。
