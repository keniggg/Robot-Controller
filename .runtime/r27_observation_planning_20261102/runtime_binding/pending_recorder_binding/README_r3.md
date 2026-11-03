# R27 remote-only r3 接续

本文件说明 `remote_only_revision_r3.py`。`remote_only_revision.py` 和旧测试仍按 r2 原件保留；`compute_pair_revision.py` 及其测试是废弃准备稿，r3 拒绝将它们列入部署来源。文件存在不表示已经重载、通过验收或抓取成功。所有实机命令由主代理执行。

r3 的生产 delta 精确限制为以下五份源码：

- `src/alicia_grasp_modes/src/alicia_grasp_modes/silhouette_observation_planning.py`
- `src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_remote.py`
- `src/alicia_grasp_modes/src/alicia_grasp_modes/observation_source_refresh.py`
- `src/alicia_grasp_modes/src/alicia_grasp_modes/reachable_observation_views.py`
- `src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py`

对应业务测试为 `test_r27_observation_hint_preparation.py`、`test_r27_observation_source_refresh.py`、`test_r27_observation_refresh_contract.py`、`test_reachable_observation_integration.py`、`test_reachable_observation_optical_side.py`、`test_reachable_observation_early_qualified.py`、`test_reachable_observation_views.py`、`test_r27_fk_five_second_budget.py`、`test_observation_frozen_prism_guards.py` 和 `test_observation_seed_handoff.py`，全部位于 `src/alicia_grasp_modes/tests/`。后两份只修正原本依赖默认 2 秒的用例，使其验证同一 5 秒累计预算，保留其他明确使用 2 秒的语义。当前远端保留原 task/runner 协议与 `current_request_measured_view_deficit_preflight`。没有扩大到 compute pair 或研究目录。

新完整回归必须含以上十份业务测试和 `runtime_binding/tests/test_remote_only_revision_r3.py`；额外 runtime 测试须同时记录在实际 `test_files` 和 `explicit_test_files`。其中历史 2 秒 deadline 测试显式传入 `max_seconds=2`，并按原生产所需 36 px margin 检查有效性；生产几何余量没有放宽。原 R26 release SHA 为 `9e483c892acfa1b9361a74873e90c972e7be0bfdb634f105b2b25fffaa92449a`，其结果是 19496 通过、原 33 跳过；旧结果不覆盖新源码，不得写成零跳过完整通过。

新进程只替换 `/remote_grasp6d_node`，另 16 个节点、ROS master、core、模拟服务、driver epoch、相机和 GUI 保持原精确身份。原启动验收后，MoveIt 可能在同一 gateway 进程内延迟注册一个 `/move_group_commander_wrappers_<数字>` 辅助名称。r3 沿用原 `node_snapshot` 的名称、数量、PID/start ticks/父 PID 校验，并绑定相同 cmdline；只在重载前原 remote 仍在时采集一次当前 alias 快照，写入本次不可覆盖的 `construction.json`，同时原样记录旧验收中的 aliases。后续停旧进程、起新 remote、验收与尝试均严格保持这份当前快照的名称、URI 与进程身份，拒绝新增、消失、改名和 foreign alias。旧 acceptance 不改写。construction 路径和 SHA 随 reload→acceptance→attempt 传递；安全返手动仍沿用原控制 owner 检查范围。

remote 使用单独 systemd unit 的 MainPID、父 PID、start ticks、cgroup、InvocationID，以及新 SSD 日志目录和从未使用的 pycache prefix。先确认 manual/IDLE/无活动 action 和真实反馈，再请求旧 remote 退出；不强杀，不启动重复 owner。

同一新 remote 进程正常预载四个模块并核验路径/SHA，随后仍 `runpy` 原 catkin relay。有限 audit hook 仅核验该 relay 和原 mode entry 的实际 `exec` code object 与冻结源编译结果一致。临时 trace 仅观察原 mode 顶层；在最终 `if __name__ == '__main__'` 行、`rospy.init_node` 之前移除 trace，然后核验实际 `ModeAwareRemoteGrasp6DNode` 的两个 mixin 对象、刷新函数引用及 FK 方法的正式延迟导入，写入 `source_start.json`。未使用生产函数替换、猴补或另一份 mode main。审计不阻断其他路径的合法加载；源码证据无法完成时不进入原 main。该证据明确 `memory_attestation=false`；FK 和刷新计算没有由启动观察器执行，绑定证据不能当成算法验证或动作结果。

`prepare` 仍为离线命令，新增 `r27_remote_only_revision_r3` 和 release，旧 contract/release/acceptance 均不改写。独立执行审查的 kind 为 `r27_remote_only_execution_review_r3`，须精确绑定 revision/entry、`allowed_node_changes=["/remote_grasp6d_node"]`、`cold_check_authorized=false`。`reload` 产生 `r27_remote_only_reload_r3`；`accept` 产生 `r27_remote_only_acceptance_r3`。验收开始前即要求本次精确 reload 路径、SHA、kind、execution review 和 source receipt；再次检查原 SDK/同步 RGB-D/相机/GUI、远端实际协议依赖和 typed QUERY，不发送 typed PREPARE/ARM 或运动命令。没有 cold_scan 或冷启动检查。

接续尝试继续由原 `tools/run_recorded_grasp.py --storage-backend ssd --root <SSD>/grasp_records --duration 600 -- /usr/bin/python3 -B <r3helper> attempt ...` 承载。`attempt` 使用原 `Session`、`attempt_lifecycle`、选择事务、执行反馈与失败返手动，不修改旧 JSON 或原校验函数。必须绑定本次对准/继续指令、`maximum_attempts=1`、`automatic_retry=false`；同一指令 SHA 仅可 claim 一次。执行参数由 r3 的 `prepare/reload/accept/attempt --help` 给出。运行时必须具备实际 ROS/workspace 环境、`PYTHONNOUSERSITE=1`、`PYTHONDONTWRITEBYTECODE=1`、未优化 Python。

录制 wrapper 须按 `/usr/bin/python3 -B <绝对 tools/run_recorded_grasp.py> --storage-backend ssd --root <SSD>/grasp_records --duration 600 -- <原 attempt 命令>` 调用；可额外使用 Python `-u`，三个 wrapper 参数可换顺序，duration 也可写 `600.0`。不传 `-s`、自定义 config、ready-timeout 或 min-free-bytes。原 wrapper 使用当前 SSD `archive_config.json` 和其原空间保留量，先等 recorder READY，再直接启动 attempt。600 秒是原录制时限；任务结束后仍由原录制器执行 3 秒尾录，不能把配置时限写成已录满 600 秒。

只在 attempt 中，r3 为该次 READY recorder 建立精确绑定：READY/manifest/config 路径和 SHA、manifest 中 wrapper 的 PID/start ticks/boot、wrapper 与 recorder/attempt 的父子关系、实际 argv、record/bag 路径、600 秒和当前 SSD 环境。每次 preflight、进入自动前及选择前均重新检查三进程与不可改证据；PID 重用、父进程变更、记录结束、证据变化均拒绝。首次从实际 XMLRPC PID 和 `/proc` 确认唯一匿名 recorder 的名称、URI、启动时刻和 cmdline，写入单次 `recorder_binding_<instruction_sha>.json`，之后保持精确身份与端点快照。它只能发布 `/rosout` 日志，服务仅自身 `get_loggers`、`set_logger_level`；当前冻结 recorder 没有 TF Buffer，`tf2_frames` 不放行。不能用相似名称、另一个 record 或另一个只读观察器取得例外。全图其余 owner、原相机/SDK/控制 publisher 检查仍直接使用未过滤的真实 state。

reload 和 accept 不接受 recorder 例外；安全返手动也不依赖 recorder 仍在。prewarm 在 release 前只导入、不创建 ROS 节点，故不增加 runner 节点白名单。正式 runner release 后继续由原生命周期持有执行权，关闭前不强制切换模式。新的录制绑定证明进程/文件/图的来源，仍不声称内存证明或实物结果。

默认且本次唯一实验盘为 SSD。核验其配置、挂载身份、空间与原会话绑定后才记录；无跨盘回退、双写、清理、上传或归档 worker 启动。禁止温度查询和关节失能。

物体是否被夹住须由本次相机、SDK、夹爪真实反馈以及必要的用户夹住确认判断。无皮肤传感时 `HOLDING/GRASP_HOLD_UNVERIFIED` 或 runner code 6 保留“夹持未确认”语义，不能自动归类为实物失败并重试；runner code 0 也不能单独证明实物成功。本工具不擅自抬升/张开、不自动循环、不将 `physical_grasp_verified` 改为 true。
