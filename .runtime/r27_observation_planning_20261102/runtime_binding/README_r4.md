# R27 已验收 remote 的有限 r4 接续

`remote_only_revision_r4.py` 只接续已验收的 r5 remote：parent acceptance `aa912771b4dabb979272e174d1e3eb5bc5e0f972646393873f207cea96512850`、revision `b18f5f04a17b3d7bf8f3ecbe897fa67dddfe52638595ca9db14b51b5a2c620ec`、release `87bdf16a65d8bb289bdad314ddb9a1b381456fcd0af92137e17c1f918ce7b34d`。这组固定身份用于部署溯源，不放行特定物体、候选、位姿或抓取结果。旧 r3 helper、README、原验收和全部旧 JSON 保持原件；新文件存在不表示已启动、已验收或实物成功。

相对 parent release 必须且仅允许三份业务变化及三份 runtime 变化：`contact_joint_placement.py`、新 `test_contact_minimum_change_ray.py`、新 `fixtures/contact_minimum_ray_terminal_20261103.json`，以及本 r4 helper、本 r4 test、旧 r3 test 的有限历史视图维护。两份原 joint placement 测试仍在完整 scope；允许其必要覆盖维护，但本轮未改。原 R27 五份生产源码、原 task/runner、碰撞与连续曲线检查、原 placement 2 秒预算及各控制/时效门不由此工具修改。

保留旧测试的原 SHA `87396f192a7c9a71dcf4b45f94ef33e43ee151386bc2c9cbe28f4dc9110f4224` 的副本位于 SSD 的 `new_near_field_failure_20261102/pre_minimum_placement_revision_r1/`；其 receipt SHA 为 `e4aefa7d166a1f93e19979073ba58a5fa178fcbad6899506f67e074eda09799b`。旧真实 archive 构造用例只把已封口历史 release 对应的模块、旧 runtime 测试字节及测试发现范围呈现为只读历史文件视图；每份来源须匹配原 manifest 和 preimage SHA。旧 case 保留，不把新增测试塞入旧结果，不声称旧树仍安装，也不将原历史拒绝改成成功。

原完整回归 wrapper 不变，使用两份 source map 完全相同、含义不同的 review。回归 review 仍为 `r27_offline_source_review`，baseline 为原 R26 release `9e483c89…`，额外 source 精确含 wrapper、H3、Test3、H4、Test4，额外 test 精确含 Test3、Test4。部署 review 为 `r27_minimum_placement_source_review`，baseline 为上述 parent release，绑定 parent acceptance、有限 delta 和 `regression_source_review` 引用。r4 同时复核原 wrapper 的实际 `regression_entry_receipts/<runname>.json`、完整结果、两个 review、scope/source SHA、SSD 身份和未变来源；它不创建 overlay 或把定向结果冒充完整回归。原 19,933 个 parent case 必须全部保留，原 33 个 skip 的 case 与原因必须相同。

`prepare` 的 `--baseline` 是上述 parent acceptance；`--typed` 仍是 parent 保留的原 R26 paired typed acceptance。其余参数由本入口 `prepare --help` 提供。准备只读实际来源和历史证据，新增独立 release/revision，不连接设备。当前完整 source/case/discovery 和正式 driver build 仍调用原校验；43 份 typed source、18 份 typed/generated/build 资产、55 份 startup 输入及原 R26 三份 typed/六份完整 delta 的历史关系继续逐项核验。历史 parent full r4 只按封口 bytes 校验，不以新工作区反证旧验收无效。新完整回归后必须真实、无 fixture 的 `prepare`/`Revision` 构造成功，才可由主代理进行执行审查。

停旧前核验当前 parent 的 PID/start ticks/父 PID、原 wrapper cmdline、unit/MainPID/cgroup/InvocationID、实际 cwd path/device/inode、bytecode/log/revision 环境，以及其余 16 个节点、core/master、模拟服务、driver epoch、gateway alias 和真实 manual/IDLE/反馈。parent 的真实 relay 参数取自其封口 `construction.new_argv`，不把实际 `python3 -B <H3> child …` 错认成 relay argv，也不要求 child 已删除的 private 文件重新存在。新进程继续使用同一 cwd；systemd 的共同父进程可以相同，PID/start 与新 unit 仍严格不同。停旧只能请求正常退出，超时拒绝；不强杀、不启动重复 owner。

确认旧进程退出后保存本次 `parent_stopped.json`。旧 `--collect` unit 随后被回收是正常生命周期；新 child、accept 和 attempt 核验 sealed parent 链、本次 stop/construction/source receipt 以及当前新 unit，不再要求旧 unit 仍 Active 或仍能查询。`WorkingDirectory`、private payload、child 实际 cwd 和后续 graph 均保持同一已冻结目录身份。

新 child 正常预载五个源模块：原四个 observation 模块加 `alicia_flexible_grasp.grasp.contact_joint_placement`。实际原 relay/mode code object 及 before-main observer 保留；在移除 trace 后、原 main 前检查实际继承的 original remote 两个方法、其延迟 import 符号和预载 placement 函数 globals/co_filename，再写 source receipt。该检查不调用 planner、不替换生产函数、不执行另一份 main；`memory_attestation=false`。实际导入及完整 ROS 运行仍由真实 reload/accept 证明。

录制、`Session.preflight → observe → AttemptContext.recheck → graph`、唯一 recorder PID/父子关系/READY/manifest/source/端点绑定、owner 拒绝和失败返手动沿用 r3。accept 继续原 typed QUERY、SDK/同步 RGB-D、相机/GUI 与远端预测协议健康检查；无 typed PREPARE/ARM 或运动。每次 live policy 及非 manual-return 的 Context recheck 都按原配置逐值核验 `/mujoco_digital_twin/unknown_lift_gate_enabled`、`min_score`、`server_url`。这三个参数的独立复核适用于 GUI 已自动的阶段；安全返手动不依赖它们、remote health 或 recorder 存活。本地仿真地址与远端预测服务的含义独立，不改变原政策。当前读取的值不能证明历史每个候选执行时的值。

离线测试实际读取 parent 的 18 个封口引用与原 full r4 ledger，逐项损坏必须拒绝。reload 生命周期仅在 OS/RPC 边界隔离；child 用最小无 ROS entry 实际运行 runpy/observer/source receipt，两个 original remote 方法与 placement 导出定义来自真实生产 AST。双 review 测试是元数据绑定测试，不是伪造回归证明；recorder graph fixture 沿用明确隔离的来源准入/传感 I/O。完整当前 Revision 与真实设备部署仍须后续无替身验证。

所有实机动作只由主代理执行。默认 SSD 必须先核配置、UUID/device、可写空间并绑定原会话；不回退、双写、清理、上传。禁止温度查询、关节失能及冷启动检查。尝试仍单次指令、单次 claim、无自动重试；是否实物夹持成功须按原实际反馈与确认，不由回归、source receipt、runner code 或此 README 推断。
