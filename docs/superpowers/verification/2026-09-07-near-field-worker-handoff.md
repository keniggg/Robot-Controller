# 2026-09-07 近场推理线程交接修复

## 现场证据

调整输入 ROI 后，计划 `97f8591b1d5b7e1eac28bc95` 于 18:53:36
启动，复用当前观察位，相机距目标 0.1932 m。目标输入占比 0.173341，
通过未变的 0.15 门限；近场参考表面包含 321 个实测点。
18:54:38 仍以 `NEAR_FIELD_DIRECT_TIMEOUT` 结束，未执行闭爪或抬升。

推理指标揭示了另外一个独立故障：旧 generation 7 的请求 112 正在
候选选择时，请求 116 已由 coordinator 提升为 active，但还没交给
唯一的 worker。阶段切换后，新 generation 9 的请求 119 排队等待。
worker 收尾时取消 116；coordinator 随即提升 119，但原代码忽略了
这次 `complete()` 返回的 `next_ticket`。因此 119 没有开始推理，
直到任务超时才收到清理记录。单次近场策略此时已消耗提交机会。

原始指标保存在工作区根目录
`.ros_log/grasp_attempt_20260907_1842/stalled_handoff_metrics.json`。

## 修复和验证

worker 收尾阶段在同一锁内退休过期 ticket，并把由此提升的新 ticket
继续交接给 worker。每个 ticket 仍重新核对 generation、target epoch
及关闭状态；每个被取消的请求仅产生一次终态记录。不改变相机数据、
近场提交次数、候选条件或运动控制。

新增两条确定性并发回归覆盖近场切换与目标实例切换。修复前均失败：
请求 3 永远没有终态。修复后确认只推理请求 1 和 3，请求 2 被丢弃，
三个请求各有一条终态。相关关闭/取消回归共 5 passed。

完整相关回归 `test_remote_grasp6d_streaming.py`、
`test_latest_only_inference.py`、`test_fresh_grasp_runner.py`：
**304 passed，36.04 s，exit 0**。Python 语法检查及
`git diff --check` 通过。实物抓取结果仍需运行时验证。
