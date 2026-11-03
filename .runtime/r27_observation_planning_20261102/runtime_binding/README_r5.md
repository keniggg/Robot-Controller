# H5：已验收 post-trajectory H4 的单节点接续

本入口 `remote_only_revision_r5.py` 只接续当前 H4 remote：`r27_minimum_placement_20261103_r2`，PID 112610 / start 2827157；其实际 post-trajectory acceptance SHA 为 `bc468af151eeab7fc21ac0176c2e1caca4659d2a485342da1b180b1ec9a80b65`，H4 revision `f814ff5c7ddf2728fa863cceb5f35fc4f9fe8eae5e24611ae27d430a1a2f535c`，continuation `4ca79b856c3b8ccd44653bc245a56fc1c63d0553f832517e8f1fd43e79fb5f4e`。87941 仅是历史祖先，不是本次停止或重载对象。固定运行身份是溯源约束，不指定物体、位姿、关节或候选。新文件不表示已部署或实物成功。

部署基准是已验收 composite 4286 sources：原 H4 full `736791f21fbb3de687eff6db7ff4e3f90d74031fe2675cfecaa629435b22dfd1` 的 4282 源，加原 post-trajectory 两模块/两测试的实际 151-case target。旧 full、target、review、真实 wrapper receipt、发现清单和 case/skip 分别按原封口字节核验；不构造旧 H4 `Revision`，不把旧 release 当当前树，也不改写旧验收或测试结果。R1 的唯一真实 action goal.header.seq 为 1（不是动作次数）；原 goal/result、两端 typed transition、后续新 epoch 上下文仍通过原纯 validator 验证。R2 的 `be242467…` 独立封口审计、原 manifest/authorization/claim、完整录制身份、两端 seq=1 和四类 arm/hand goal/result 零计数共同证明其未新执行动作。历史不代替当前 SDK/owner/typed/QUERY 新鲜度。

相对 composite 必需变化精确为：

- 生产 `src/alicia_grasp_modes/src/alicia_grasp_modes/reachable_observation_views.py`。
- 新 `tests/test_reachable_observation_support_proof_budget.py`、`tests/test_reachable_observation_support_box_independent.py` 及后者 `fixtures/supplement_support_box_20261103.json/.npz`（以上均在 `src/alicia_grasp_modes/`）。
- 本 H5 helper/test。

另外只允许四份既有测试维护：H3 test、post `tests/test_adapter.py` 的精确历史文件视图，以及 `test_gen8_endpoint_precompute_independent.py`、`test_reachable_observation_direction_coverage.py` 的两条拒绝路径 stub。preimage receipts 为 `attempt2_observation_failure/pre_support_box_revision_r1/receipt.json`（`9c4dc76…`）和 `pre_support_box_test_maintenance_r1/receipt.json`（`a0e0178…`）。不得删除/重命名原 case。原 Prepared/precompute、path_guard、本机 gateway、task/runner、typed/C++、模型和物理误差/余量均不变。新增逻辑只在最终 coarse support 证明不足时，在原共享 5 秒/每向 96 预算内用现成完整误差盒证明；证明方法和数值下界有变化，不能称原数值界逐字未变。晚到 proof 丢弃，其余原门仍独立必过。

两份 review 的 source map 必须一致。原 wrapper 的 `r27_offline_source_review` 仍以 R26 为基准，额外 source 精确 11 项：wrapper、H3/Test3、H4/Test4、post adapter/readiness/两 test、H5/Test5；额外 test 精确为这五份 runtime tests。H5 部署 review 为 `r27_support_box_source_review`，绑定 `baseline_release=PARENT_RELEASE`、`parent_acceptance=PARENT_ACCEPTANCE`、`parent_continuation=PARENT_CONTINUATION`、相对 composite 的 `changed_sources`、完整 `source_sha256` 和 `regression_source_review` 引用；`additional_source_files` 只含新 H5 两文件及需要显式列出的 H3/PostAdapter test。新完整回归必须保留父 full 所有 case、原 33 个 skip 的身份和原因以及原 post target 全部 case。不能把 pilot 标成 full。

所有 live 操作由主代理独占。先冻结来源、完成新定向和必要原 wrapper full，随后真实无 fixture `prepare`，再写 exact execution review。以下是参数模板，尖括号必须替换成实际文件及其 SHA；不得照抄为已完成的记录。`W` 指生产 worktree，`M` 指 SSD `maintenance/r27_observation_planning_20261102`，`S` 指该会话 `ros_log/full_latest_r26_manual_20261102_r1`。

```text
PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -B <W>/.runtime/r27_observation_planning_20261102/runtime_binding/remote_only_revision_r5.py prepare
  --baseline <S>/r27_minimum_placement_20261103_r2/post_trajectory_acceptance_r1/result.json
  --baseline-sha256 bc468af151eeab7fc21ac0176c2e1caca4659d2a485342da1b180b1ec9a80b65
  --typed <PARENT_REVISION.baseline_typed.path> --typed-sha256 <同引用SHA>
  --regression <新full/result.json> --regression-sha256 <实际SHA>
  --source-review <H5部署review.json> --source-review-sha256 <实际SHA>
  --instruction <当前有效继续授权.json> --instruction-sha256 <实际SHA>
  --output <M>/<全新prepare目录> --runtime-child <会话内全新单层名称>
```

prepare 仍实际调用原当前完整来源/发现范围/full/driver build 核验、原 R26 preserved typed 关系、43 typed sources/18 assets 和 55 startup inputs 校验。生成 `release.json/revision.json/preserved_typed_validation.json` 后，execution review 必须 `kind=r27_remote_only_execution_review_r5`、`reviewed=true`、`revision=<实际ref>`、`entry=<H5实际ref>`、`allowed_node_changes=["/remote_grasp6d_node"]`、`cold_check_authorized=false`。它不能预先冒充未来 reload 或 acceptance。

```text
/usr/bin/python3 -B <H5绝对路径> reload --revision <revision.json> --revision-sha256 <SHA> --execution-review <review.json> --execution-review-sha256 <SHA>
/usr/bin/python3 -B <H5绝对路径> accept --revision <revision.json> --revision-sha256 <SHA> --execution-review <review.json> --execution-review-sha256 <SHA> --reload <新runtime/reload_result.json> --reload-sha256 <SHA>
```

两条命令继承 `PYTHONNOUSERSITE=1/PYTHONDONTWRITEBYTECODE=1` 和原环境。reload 首先核当前 112610 wrapper argv、systemd/unit/MainPID/cgroup/InvocationID、真实 cwd path/device/inode、bytecode/log/revision 环境、原 16 节点/core/master/模拟进程/相机/GUI、原 gateway alias、epoch 和 manual/IDLE/静止反馈；停旧前再核。只请求正常退出，不 force kill；保存本次 `parent_stopped.json` 后才启动唯一新 unit。新 child 保留原真实 cwd；`PWD` 不是 cwd。旧 collect unit 回收不阻断新 child。

child 预载五模块，观察实际原 relay/mode 编译 code object 和原 main 前真实类/MRO/refresh/placement 接线；额外核实际 FK 模块的 `observation_following_support_bound` global 正是未变 path_guard 导出的同函数对象、globals、co_filename 和固定 SHA。记录依赖已绑定，但不声称 FK/full-box 已执行；`memory_attestation=false`。不增加永久 trace 或替换原函数。

accept 的新 kind 是 `r27_remote_only_post_trajectory_acceptance_r5`，`typed_initial_contract_ready=false`。原 `typed_readiness.py` 校验当前全部空态字段，只有历史 action transport seq 必须精确等于已证实的 1；不清零、不投影、不放行任意非零。两端原 QUERY 各一次，原固定回执和到期时间保持；服务先回/消息后到仅在原期限内等对应 source，超时拒绝，不重发。前后 exact IDLE/stage0/activeFalse/successFalse、SDK/encoder/JTC/controller/owner、实例、模拟参数及远端预测健康仍独立核验。

成功验收后才材料化新单次票。原自动授权字段仍由原 `validate_instruction` 验证，另外必须 `maximum_attempts=1`、`automatic_retry=false`，并精确绑定 `runtime_revision=<新revision ref>`、`current_acceptance=<新acceptance ref>`、`latest_resume=<新revision.instruction>`、`parent_continuation=<固定PARENT_CONTINUATION>`。两份已消耗旧票的 path 或 SHA 均拒绝。新 runtime 固定 `attempt_claim.json` 以 `xb` 独占创建；同票跨新 revision/acceptance 不匹配，同 runtime 换票也不能重试。

```text
PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -B <W>/tools/run_recorded_grasp.py --storage-backend ssd --root /home/zhuyupei/ssd_data/实验记录/Robot-Controller/grasp_records --duration 600 -- /usr/bin/python3 -B <H5绝对路径> attempt --revision <revision.json> --revision-sha256 <SHA> --execution-review <review.json> --execution-review-sha256 <SHA> --acceptance <新runtime/acceptance/result.json> --acceptance-sha256 <SHA> --instruction <新单次票.json> --instruction-sha256 <SHA>
```

wrapper 必须绝对原路径、`/usr/bin/python3 -B`，恰好 backend/root/duration 三选项，不带 `-s`、`--config` 或其他变体。原 600 秒 READY/manifest/parent/start ticks/boot/cmdline/source/录制路径链绑定唯一只读 recorder；reload/accept 不接受 recorder 例外。原 Session 在 preflight 和 prewarm 后、自动请求前复核当前 typed 历史；自动选择阶段不要求 sdk_manual，不把历史 seq 约束延续到新执行结束。原手动 cleanup 在独立分支先返回，不依赖 remote health/模拟政策/recorder/历史 seq；控制 owner/source/SDK/关闭执行所有权规则仍保持。

失败恢复按实际边界处理：若停旧前失败，检查报告和实际旧实例后另行审查，不删除失败产物。若已停旧而尚未启动新 unit，不重复运行一次性 reload，也不再要求旧 unit 活跃；保留 stop/construction/launch 证据，由主代理基于实际状态准备有限恢复。若新 unit 已运行但验收失败，保持 manual，核失败原因后仅对该实际新实例安排独立验收接续，不向已停父重载、不清零 typed。没有通过 acceptance 不进入 attempt。

测试读取真实完整祖先/continuation/R2 evidence，无历史 current-tree 替身。reload 与 child 只隔离 OS/RPC/current-source admission；child 用最小无 ROS entry 实际 runpy/observer/receipt，函数代码来自原 AST，未执行控制。Session 测试执行原 preflight/prewarm/select/return_manual，保留明确的传感/源准入 I/O fixture。accept 测试执行实际 H5 orchestration 和原 pure typed/fixed-query validator；生成类型、binary/graph/health/current-release 边界是标注的 I/O fixture。它们不代表真实部署或实物成功；最终无替身 prepare/reload/accept 仍各自必须成功。

无图像人工确认新增要求、不执行 cold scan、温度查询、失能、上传或清理。code 0、闭合 ACK、HOLDING、GRASP_HOLD_UNVERIFIED 或 runner 6 都不能单独证明实物成功；持物/结果未确认不是自动重试依据。保留相机/SDK/夹爪证据与必要夹住确认，不擅自 lift/open。
