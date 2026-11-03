# R27 单规划节点接续准备

本目录仅是待审入口和离线契约测试；写入文件不表示已经重载、验收或抓取。实机操作只由主代理执行。

生产范围固定为 `src/alicia_grasp_modes/src/alicia_grasp_modes/silhouette_observation_planning.py`，新增业务测试固定为 `src/alicia_grasp_modes/tests/test_r27_observation_hint_preparation.py`。本轮该改动目前用于分阶段耗时诊断，不能称为已解决规划失败。唯一直接生产 import 是 `mode_aware_remote_grasp6d.py:51`。新进程仅替换 `/remote_grasp6d_node`，保留另 16 个节点、ROS master、原 core、模拟服务、驱动及硬件接口。

当前 R26 后段来源未变：`physical_reference_client.py` 为 `b0c75d0477538f66e9d39d3c8a10e169715981b75063d4a91624fe7cd6be3e01`，`mapped_contact_execution.py` 为 `8fec455a1eafb209d820f95e6129784cca922e75ce784e23adee7f5409257f08`。修复把真实来源采集移到发现服务/完整校验后，并扣除完整 callback 耗时；C++ 500ms 来源门及原 ROS/monotonic 截止仍保留。R26 重连回归为 701 文件、8 worker、19496 通过、原 33 条件跳过、0 失败、0 收集跳过，708.802 秒；严格零跳过 `full_regression_passed` 保持 false。新完整回归须单独绑定本次实际源码。

`remote_only_revision.py` 不改旧 release、contract、acceptance 或生产入口。它新增独立 `r27_remote_only_revision` 和 `r27_remote_only_acceptance`，不将重载假称全栈重启。原 helper 中没有任何函数被替换、猴补或关闭。

需明确不同于原全栈契约的地方只有：当前源为旧完整基线加逐项审阅的新 delta；remote 转入新独立 systemd unit，使用它的 MainPID、父 PID、start ticks、命令摘要、ControlGroup 和 InvocationID；remote 另用新 SSD 日志目录和从未使用的 pycache 前缀。其余 16 个节点仍须逐项等于原实际验收的进程身份，并保留原环境和归属。没有 cold_scan 或冷启动检查调用。

直接复用的原检查包括：`current_release_evidence` 的完整用例/阶段/跳过语义、`driver_build_evidence`、原存储身份/空间/local_only检查、原 Monitor 的 SDK20/同步RGB-D/手动/无活动动作检查、typed `verify_assets`/`runtime_binaries`/`graph_bindings`/`observed_ready`/`bounded_query`/`wait_for_fixed_query_sources`、原远端协议健康检查，以及原 `Session`/`attempt_lifecycle`/预热与选择/失败返手动流程。模拟服务 helper 仅把其 `UNIT` 配置指向原实际启动单元，再调用原 readiness 方法；未替换函数。返回手动沿用原有限控制 owner 检查，不因规划进程不可用而额外阻断返手动。

四步交接：

1. 主代理先完成代码审阅和带精确 source review 的冻结回归。本 helper 与 `tests/test_remote_only_revision.py` 必须列入新源码清单和实际测试范围。新增 `.runtime` 测试应同时出现在结果 `explicit_test_files` 中；不得只加 scope 而漏掉结果来源声明。
2. `prepare` 为离线操作：输入原 paired native/typed验收、新完整回归、source review 和当前用户继续指令；创建新 `release.json` 与 `revision.json`。需要 `source=actual_user_message`、实际原话及 `repair_and_continue_authorized=true`。它仍逐字节校验所有原输入，不改变旧文件。
3. 主代理独审后新建 `r27_remote_only_execution_review`，精确绑定 revision/entry，`reviewed=true`、`allowed_node_changes=["/remote_grasp6d_node"]`、`cold_check_authorized=false`。`reload` 在新鲜手动/IDLE/无活动动作、未变来源/驱动 epoch/真实反馈下停止原 remote，等待原进程和 graph owner消失，仅启动一个新 unit。停止失败不会强杀或重复启动。`accept` 则重新核验当前节点、相机、控制器、模拟服务、远端 `/health` 和原始 typed QUERY；不发 typed PREPARE/ARM/运动请求。
4. 新 `r27_remote_only_acceptance` 通过后，使用原 `tools/run_recorded_grasp.py --storage-backend ssd --root <SSD>/grasp_records --duration 600 -- /usr/bin/python3 -B <本helper> attempt ...`。`attempt` 必须绑定本次实际对准/继续授权、`maximum_attempts=1`、`automatic_retry=false`，且处于原600秒录制父进程中。同一 instruction SHA 只能消费一次；每次独立尝试均生成新 claim、录制和实际结果，不自动循环。

CLI 参数由 `python3 -B remote_only_revision.py <prepare|reload|accept|attempt> --help` 列出。`reload/accept/attempt` 均要求 `--revision/--revision-sha256` 与 `--execution-review/--execution-review-sha256`。主代理使用已 source 的 ROS/workspace 环境、`PYTHONNOUSERSITE=1`、`PYTHONDONTWRITEBYTECODE=1` 和未优化 Python；不用本 helper 的存在推断运行授权或参数。

真实成功须以本次录制中的真实闭合/夹持反馈及物体实际被夹住的证据判定。候选/预览通过、运动 action成功、PREPARE/ARM通过、runner退出0或 `HOLDING/GRASP_HOLD_UNVERIFIED` 均各保留原语义；没有物体被夹住的证据时保持未确认，不能写 `physical_grasp_verified=true`。本 helper从不自动把该字段改为true。若未抬升则明确记录未抬升；历史2026-10-06用户确认夹住不代表当前版本成功。

最小回归关注本次规划诊断正反分支、真实计时与取消/预算边界、不同候选数量/姿态/开口，以及原碰撞/无效来源/无支持模型继续拒绝。已验证且未变的R26后段结果复用；若需新的完整源码release，本次完整回归按主代理实测资源预算最多8 worker，不能把小范围试跑写成完整通过。
