# 下次完整启动准备稿

本目录状态为 `installed_for_next_cold_start`，运行验证状态为 `not_runtime_verified`：两个启动入口和固定 user unit 已按准备稿原子安装，原文件与权限信息保存于外盘部署记录；已执行一次 `systemctl --user daemon-reload`。未启动、停止或重启服务，未执行 ROS 控制。8 个指定现有进程的 PID、startticks、argv 前后完全一致，固定仿真 unit 保持 inactive/MainPID=0。默认执行两个 Python helper 只输出审阅信息；`--prepare`、`--serve`、`--wait-ready` 仍仅供下一次冷启动入口显式调用。

| 文件 | 作用 / 未来目标 |
|---|---|
| `cold_start_handoff.py` | 仅从未来 root 冷启动脚本进程调用时，校验并撤出列出的 9 个临时软件 unit，避免旧 `PartOf` 重启任务与完整 ROS 树重复拥有节点。默认只打印清单。 |
| `legacy_owner_commands.json` | 冻结允许交接的软件 unit 名称和完整 `ExecStart` argv；不含 root、硬件 driver/controller。 |
| `candidate_simulation_startup.py` | 默认只打印审阅信息；`--serve` 分配新的 SanStand 目录后调用冻结仿真入口；`--wait-ready` 校验固定 unit 的 invocation、实际监听进程归属及 health 中的模型绑定。 |
| `alicia-candidate-simulation.service` | 固定 user unit 模板，已安装到 `~/.config/systemd/user/alicia-candidate-simulation.service`；当前未启动。 |
| `candidate_pair_and_simulation.launch` | 最后加载已经冻结的候选 URDF、手眼配置和本机 8002 地址；无 node。 |
| `resume_modes.staged.launch` | 已原子安装到主项目 `.ros_log/ros_live_20260922_session/resume_modes.launch` 的完整副本。 |
| `start_ros.staged.sh` | 对应同目录 `start_ros.sh`；按顺序交接临时软件归属、启动仿真、等待匹配服务就绪，再执行原完整 roslaunch。不要直接运行此副本。 |
| `original_start_ros.sh`、`original_resume_modes.launch` | 原入口逐字备份，其 SHA 与 manifest 的 `source_entry_sha256` 对应。 |
| `startup_entries.patch`、`staging_manifest.json` | 两个入口的精确差异、原 SHA、artifact SHA、离线验证结果。不是实机冷启动成功凭据。 |

## 冷启动归属和服务顺序

本机 root unit 当前为 `Type=simple`，真实入口是 `/bin/bash /home/zhuyupei/alicia_wa_full/.ros_log/ros_live_20260922_session/start_ros.sh`。helper 同时核对 root `MainPID`、其直接父进程 argv 和 `Type=simple/exec`，只允许在该脚本尚未 `exec roslaunch` 时交接。当前正在运行的 root 主进程是 roslaunch，因此不能从旁路运行 `--prepare` 触发交接。类型检查也避免未来改成 `notify/oneshot/forking` 后，入口等待一个排在自身启动之后的服务。

根据本机 `systemd.unit(5)`，`PartOf` 将 root 的停止或重启动作单向传播给旧临时 unit；停止这些子 unit 不会沿该依赖反向停止 root。helper 先校验 `ConsistsOf` 没有未知子项，并核对 9 个 owner 的完整 argv，全部通过后才以一次 `systemctl --user stop` 事务交接。已加载但 inactive 的旧 unit 也纳入事务，取消可能排队的旧 `PartOf` start job。随后复核 root 仍在相同冷启动入口、旧 unit 均已 inactive/failed 且无待处理 Job。未知 owner、命令变化或未完成的交接会让入口退出，不启动后续 ROS 树。

固定仿真 unit 用 `BindsTo` 加 `After` 绑定 root，`Restart=no`，不使用 `PartOf` 和 `WantedBy`。root 冷启动脚本显式启动它；root 结束时它随之结束。原有时间戳仿真 unit 属于上述交接清单，不能原样重启：其输出目录固定且已经存在。新 helper 每次分配唯一目录，不复用旧输出。

`--wait-ready` 除原有 health 模型字段外，还核对 unit 的 `MainPID`、`InvocationID`、cgroup 以及 127.0.0.1:8002 监听 socket 的进程归属，并在读取 health 前后复核。新 unit 启动失败时，即使旧 8002 仍然健康也不能通过；调用期间 invocation 改变同样拒绝。等待有超时，不退回旧模型或远端 8001。

## 配置、存储和边界

URDF `7812d6b6…`、手眼 `637701ac…`、pair `841ca149…` 和仿真 XML `dc9e8bbc…` 保持冻结。原生产 URDF、旧手眼 YAML、冻结仿真入口和 pins 未修改。`--wait-ready` 按原 manifest 的绑定核对 `digital_twin.ok` 和编译模型；专用 backend 顶层 `ok=false` 不是其仿真 readiness 判据。

每次仿真启动以 `tempfile.mkdtemp` 在 SanStand `ros_log/` 下分配唯一父目录，写 `startup_request.json`（包括 invocation ID）和 `bootstrap.log`；冻结入口收到尚不存在的 `service/` 子目录。未挂载或路径逃出 SanStand 时显式失败，不回退系统盘。默认审阅和纯测试不监听端口，不访问硬件；测试使用的临时假文件不是实验记录。

相机和 unknown 在原完整树中各有一个 node，catkin relay 执行当前源码，相机仍经 `with_local_realsense_sdk.sh`。不重复 include 相机或 unknown 热加载树。实际反馈 TF remap、相机慢周期诊断、unknown 精确时间 TF 有界等待、RGB 同帧搜索等源码修复随冷启动加载；`.008` 补偿速度沿用 `grasp_params.yaml`。原使能配置、driver/SDK、lift 策略、20g 上限等参数保留。helper 不发布关节使能、失能或运动命令。

文件已在保持现有进程运行时安装为未来启动配置。首次真实完整启动仍需独立验证服务交接及 ROS 树单一归属；当前 44 项离线测试和静态 unit 验证不能替代该验证。仅重启机械臂电源不等于完整 ROS 栈冷启动，不调用这些交接操作。本轮用户要求修复后暂停，因此没有发起完整启动或自动抓取。

回退持久化时必须整体恢复匹配的 URDF、手眼和仿真端点，不能只移除最后 include 而保留新仿真地址。准备稿没有清除历史失败、保存目标、复用轨迹或改写 source cutoff。

## 已完成的离线验证

2026-09-29：安装前 41 项纯测试、bash 语法、结合真实 root unit 的 `systemd-analyze --user verify` 通过，两个原入口 SHA 和 8 个临时 unit argv 均吻合。安装后按 `deployed_entry_sha256` 读回的三个 SHA 和权限全部一致；41 项纯测试（0.09 秒）、实际入口 bash 语法、实际安装 unit 的静态验证再次通过，8 个现有进程身份保持不变。

```bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m pytest -q -p no:cacheprovider .runtime/full_startup_candidate/test_full_startup_candidate.py .runtime/full_startup_candidate/test_cold_start_handoff.py
bash -n .runtime/full_startup_candidate/start_ros.staged.sh
systemd-analyze --user verify /run/user/1000/systemd/transient/alicia-ros-live-20260928.service .runtime/full_startup_candidate/alicia-candidate-simulation.service
```

依据为本机 systemd 官方手册 `systemd.unit(5)` 的 `PartOf`、`BindsTo`、`After` 定义，及实际只读 unit 属性。未在本轮做完整栈运行验证。

部署证据：`/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/startup_persistence_20260929T095210Z_sgvtmgth`。其中 `originals/` 保存原入口，`deployment.json` 保存原权限与 SHA、各次原子替换结果、前后进程身份和新 unit 未运行的属性；`.runtime/latest_full_startup_persistence.json` 指向该记录。原入口 0664 权限保持，新增 unit 为 0644。

## 后续归属清单扩展

2026-09-29 为独立 carton 感知热加载补入 `alicia-idle-detector-cache-20260929.service`，完整 argv 指向 `carton_only.launch` 和本次 `idle_detector_repair_20260929T142135Z/ros` 记录。旧 `alicia-model-pair-perception-reload-resume-20260929T045300Z.service` 仍保留：下一次冷启动应同时撤出新 carton owner 与旧复合软件 owner，再由完整树接管。清单现在为 9 个软件 unit。此扩展只修改未来交接清单、说明和纯测试，不改已安装入口，不执行 daemon-reload、服务或 ROS 操作，也不改写前次部署证据。新增 service 的实时加载由根任务单独处理。

本次扩展纯测试为 44 项：覆盖新 owner 活跃、inactive 但有排队任务、完整 argv 改变时整体拒绝交接；原 composite owner 同时保留。扩展时只读快照显示新 unit 尚为 `not-found`，因此没有宣称它的实时 argv 已验证；当前 root 已列出的子服务均在清单内。三个已安装入口 SHA 与首次部署一致。独立证据：`/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/startup_owner_extension_20260929T142404Z_4d_mfj9f`。首次部署 pointer 中的 manifest SHA 属于当时快照，本次不修改该 pointer；最新清单与 hash 见本目录 manifest 及此次扩展记录。

随后仅补齐持久化指针的增量索引：`.runtime/latest_full_startup_persistence.json` 的 `initial_deployment` 保留首次部署报告、当时完整指针和初始 manifest SHA；`configuration_updates` 单独记录此次 owner 清单扩展及 44 项纯测试报告，`current_owner_registry` 指向当前 9 项清单。顶层 manifest SHA 更新为当前配置文档版本，仍明确 `runtime_verified=false`。此次索引同步没有启动、重载或停止服务，也不改写任何历史外盘报告。
