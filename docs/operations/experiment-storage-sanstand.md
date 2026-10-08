# SanStand 实验记录存储

实验记录和运行日志保存在 `/media/zhuyupei/SanStand/实验记录/Robot-Controller`。除下述用户明确授权的一次性旧失败实验清理外，其余记录完整保留。用户于本次会话明确取消继续上传全部实验到 GitHub；当前策略为 `local_only`，不再按“只保留最新 5 份”自动删除外盘旧实验。该指令优先于旧文档中的全量上传和五份保留规则。

## 2026-10-04 用户授权的一次性空间清理

用户明确要求从最早失败实验开始删除约 50G 原始记录，保留发现的问题、解决思路和方法、验证记录，达到约 50G 空间后停止。本次已按时间顺序处理 2026-09-07 至 2026-09-28 的 112 份明确失败实验，仅删除清单中的旧 bag、图像序列和数据输入，共释放 49.149 GiB；停止时剩余约 50.314 GiB。实际夹持成功（包括自动程序在提升阶段报失败的重叠录制）、结果未知、标定和当前运行会话均保留。

每个已清理目录新增 `实验简要记录.md` 和 `user_cleanup_receipt.json`，保留原始结果、授权、文本日志、审计及恢复元数据。原始 manifest 作为历史证据未改写；其登记的部分录制文件现已按用户指令删除，查询本地可用性时必须同时看清理回执，不能把历史清单理解为当前仍有完整 bag。摘要分别标明单次失败与关联问题族的后续修复/回归，不能据此将原失败实验记为成功。

清理索引、精确计划、删除回执、分类证据及核验结果保存在：
`/media/zhuyupei/SanStand/实验记录/Robot-Controller/maintenance/user_failed_cleanup_20261004T065532Z/`。
112 份摘要、1437 个保留源文件的 SHA256 和受保护录制均已核验；清理脚本的 12 项隔离测试通过。

该授权仅适用于本次限定清单，不启用后续自动删除或上传：`upload_enabled=false`、`delete_enabled=false`、`archive_policy=local_only` 及维护控制文件保持原样。本次清理不发送机械臂命令，也不停止 ROS 节点。

查询先在外盘 `ros_log/`、`grasp_records/` 和 `文档/` 查找。之前已经上传的 GitHub 历史内容及恢复索引保持原样；只有明确需要历史复核时才使用查询/恢复入口，不自动补传、下载或删除云端内容。

## 2026-10-07 用户授权的约 60G 单次清理

用户原话：“你先进行60个G的删除任务，这个任务就是和之前要求你删除50个G空间占用的要求一样”；随后要求“这个删除要求要记录到文档中”，并要求“清理完成后，继续处理没有完成的问题”。

沿用前次实际执行口径：以 SanStand **可用空间达到约 60 GiB** 为停止条件，按失败实验发生时间从早到晚清理。当前可用约 2.2 GiB，预计释放约 58 GiB；实际释放量、最后剩余量和停止条件均写入回执。单位口径沿用前次 GiB，不把目标理解成长期自动维持的最低空间。

清理仅限有结果/日志证据明确失败且已经关闭的实验中的大体积原始 bag、图像序列、视频和数据输入。每个目录先保存问题发现、解决思路和方法、验证结果的简要记录，再删除精确清单内的大文件；原结果、授权、文本日志、审计、恢复元数据及历史清单保留。历史 manifest 记载的已删原始文件须结合本次清理回执解释。

实物成功（包括闭爪成功后提升报错的实验）、成功、未知结果、标定、打开/仍写入的文件，以及本轮尚需用于定位断帧/支撑缺测的证据排除。不会仅因目录名含“失败”或自动程序非零退出就删除。达到目标后立即停止，并核对保留摘要/验证文件和自动存储策略。

本次是用户明确授权的一次性例外；`local_only`、`upload_enabled=false`、`delete_enabled=false` 保持。执行计划、分类证据、删除回执、摘要索引及核验落在外盘 `maintenance/user_failed_cleanup_*`。清理结束后恢复未完成的卡顿对照与抓取问题处理，不自动复用失效目标或旧轨迹。

本次已完成：按时间顺序清理 2026-09-28 03:20 至 2026-10-06 08:18 UTC 的 50 份明确失败且封口的录制，仅删除 50 个 bag，分配空间合计 **58.778 GiB**；开始可用 2.187 GiB，达到 **60.953 GiB** 后停止。并发日志继续写入，故可用空间净增量与删除文件分配量略有差异。本次无需删除任何图像或分析输入。

精确计划、独立分类/成功冲突复核、删除回执、摘要索引和核验保存在：
`/media/zhuyupei/SanStand/实验记录/Robot-Controller/maintenance/user_failed_cleanup_20261007T134426Z/`。
50 份摘要和 896 个保留文件的 SHA256、16 个受保护原始文件的身份及 3 份存储策略 SHA256 已核验一致，清理执行器 19 项 `/tmp` 隔离测试通过。用户确认实际夹住但提升失败的录制、结果未知、标定和本轮定位证据未删除。此任务已结束，不自动再次补足 60 GiB。

| 内容 | 外盘位置 |
|---|---|
| 正式 canonical 抓取记录 | `grasp_records/` |
| 当前未知物体会话及正式记录 | `ros_log/unknown_grasp_20260923/` |
| 未指定 ROS_LOG_DIR 的默认日志 | `ros_default_log/`（`~/.ros/log` 已指向此处） |
| ROS 会话日志 | `ros_log/ros_live_20260922_session/` |
| 连续终端监督 | `ros_log/unknown_grasp_20260923/supervision_continuous.log` |
| 归档清单与下载位置 | 对应记录目录中的 `manifest.json`、`result_summary.json`、`archive_manifest.json` |
| 全量盘点索引 | `ros_log/unknown_grasp_20260923/record_inventory_20260927.json`（盘点时快照，当前状态以各记录 manifest 为准） |
| 文档快照 | 外盘 `文档/`；源码文档原件仍在项目中 |

项目源码、构建产物及 operational `.sh`、`.py`、`.launch` 原件继续保留在 `/home/zhuyupei/alicia_wa_full`，尤其两个会话目录内的启动入口。`ROS_LOG_DIR` 只表示日志落点，不再用它查找 launch 或 Python 源码。不要把整个项目根替换成外盘路径，也不要将运行中的源码目录整体移动。

## 后续启动和正式录制

项目内 `.ros_log/unknown_grasp_20260923/sanstand_storage.sh` 是当前可重复使用启动入口的共同存储配置。它先确认 SanStand 为已挂载目录，再建立并检查外盘输出目录。未挂载或不可写时输出 `EXPERIMENT_STORAGE_UNAVAILABLE` 并退出，不改写到系统盘。这只是存储路径可用性检查，不发送任何机械臂命令。

`start_ros.sh` 的 ROS 日志和 stdout 写入外盘 ROS 会话目录，launch 原件仍取项目内 `resume_modes.launch`。`start_watch_current.sh` 的监督日志直接写外盘。`start_close_hold_attempt.sh` 将原有 420 秒默认录制及同一抓取命令输出到外盘 `ros_log/unknown_grasp_20260923`；策略、质量门限、运动参数和命令保持原值。`start_recorded_aligned.sh` 使用外盘 canonical `grasp_records`，原 600 秒设置不变。授权证据依旧通过录制器传入的 `GRASP_RECORD_DIR/authorization.json` 写入同一外盘实验目录。查授权时应遍历这两个外盘记录根，不能继续只查旧系统盘目录。

审计入口 `start_remote_with_audit.sh` 继续执行项目内 Python 原件，仿真 request/response 和候选路径审计使用外盘 unknown 会话目录。本轮已经运行的节点不会因编辑脚本而重载；修改在下次正常启动相应入口时生效。历史单次实验脚本与旧离线回放脚本作为实验资料保留，不应当作后续启动入口。

用户于 2026-10-04 明确要求“以后不要管机械臂温度的情况了，不要查询也不要对其反应”。后续启动持续采用 `/alicia_d_driver_node/software_temperature_monitoring_enabled=false`、`temperature_poll_rate_hz=0.0`；驱动默认及 `alicia_d_driver.launch`、`alicia_d_bringup.launch` 均已固定关闭，现有 `resume_modes.launch` 经完整启动链继承。维护、诊断及实验流程不主动查询温度，也不以温度告警、暂停、降级或使能/失能。温度专用解析、发布与软件门控关闭；SDK 原始通信、硬件固有保护、通用故障及真实编码器运动确认保持原规则。

该策略由驱动在启动时读取，给旧实例修改参数不能当作已生效。2026-10-04 的策略维护仅替换驱动进程，退出路径不发送失能，正向使能请求不等于编码器已确认运动。实际进程、二进制、前后 control epoch 和反馈证据保存在外盘 `ros_log/full_ros_resume_20261004T002930Z/driver_temperature_off_20261004T084125Z/`；后续冷启动保留关闭策略，不因本条说明再次重启节点。

项目工作树的 `.runtime/experiment_storage.json` 保存本机映射（运行配置由部署步骤生成，不提交凭据）：

```json
{
  "mount_point": "/media/zhuyupei/SanStand",
  "archive_root": "/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records",
  "ros_log_root": "/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log"
}
```

`tools/grasp_archive.py` 和 `tools/run_recorded_grasp.py` 默认使用配置的 canonical 根；`GRASP_RECORD_ROOT` 优先，显式的其他根（例如独立 `/tmp` 测试目录）继续有效。旧 canonical 根和旧 `unknown_grasp_20260923` 根这两个明确别名自动映射到上述外盘落点，其他任意路径不做替换。涉及外盘的根均要求配置挂载点在线；不存在该本机配置时，原默认行为保持兼容。两个外盘 `archive_config.json` 的 `retention_roots` 必须同时列出两个外盘实际根。

## 本轮正在运行的日志

本轮活跃 ROS stdout 的文件描述符仍指向系统盘原文件，暂留内部并持续镜像到外盘，直到写入者自然关闭。不能删除、移动或截断仍打开的源文件，也不为了更换日志落点重启机械臂节点。镜像是当前日志快照，最终关闭后核对字节数和 SHA256，再按部署流程完成最终迁移。新启动的监督进程和未来会话直接使用外盘。

## 查找和恢复更早实验

先查看外盘完整记录和索引。仅对以前已经上传、且需要独立查询或恢复的历史记录，使用 manifest 中的仓库、Release、附件顺序和 SHA256；历史 GitHub 目标为 `keniggg/Robot-Controller` 的 Releases。需要恢复时应取该实验完整的 `archive_manifest.json` 和登记附件，不能只取运行日志而遗漏 bag、关键帧审计或操作员观察。以下为按需恢复说明，不会自动执行下载。

在项目工作树执行恢复工具，目标选择外盘中一个尚不存在的目录：

```bash
python3 tools/grasp_archive.py restore "/外盘/对应实验/archive_manifest.json" "/media/zhuyupei/SanStand/实验记录/Robot-Controller/restored/RECORD_ID"
```

工具按附件顺序恢复并核对逐块、完整文件 SHA256。恢复副本放入独立目录，避免覆盖原始记录。当前不自动删除 SanStand 中的新旧实验，是否已经上传不改变本地保留要求。

历史 `legacy-grasp-*` Release 使用独立的历史恢复清单格式，应使用对应恢复入口：

```bash
python3 tools/archive_legacy_grasp_records.py \
  --restore-manifest "/外盘/历史实验/archive_manifest.json" \
  --restore-directory "/media/zhuyupei/SanStand/实验记录/Robot-Controller/restored/LEGACY_RECORD_ID"
```

该入口恢复归档的精确字节；已无损压缩的历史文件恢复为 `.gz`，其原文件 SHA256 和压缩审计仍保存在恢复清单中。它不删除 GitHub 或源文件，也不启动机械臂。

补充材料的 `supplemental-grasp-*` Release 内是 `historical_supplement.tar.gz` 与 `members.json`，需要两步恢复。先在对应 `recovery_index.json` 所在目录找到 `RECORD_ID.remote_manifest.json`，使用历史恢复入口从 GitHub 下载并校验这两个附件：

```bash
python3 tools/archive_legacy_grasp_records.py \
  --restore-manifest "/外盘/补充归档/RECORD_ID.remote_manifest.json" \
  --restore-directory "/media/zhuyupei/SanStand/实验记录/Robot-Controller/restored/SUPPLEMENTAL_RECORD_ID-assets"
```

再从这两个已验证附件离线恢复原始文件；工具逐文件验证大小和 SHA256，并按 `members.json` 中的相对路径创建文件：

```bash
python3 tools/prepare_supplemental_log_archive.py --restore \
  --bundle "/media/zhuyupei/SanStand/实验记录/Robot-Controller/restored/SUPPLEMENTAL_RECORD_ID-assets/historical_supplement.tar.gz" \
  --members "/media/zhuyupei/SanStand/实验记录/Robot-Controller/restored/SUPPLEMENTAL_RECORD_ID-assets/members.json" \
  --restore-directory "/media/zhuyupei/SanStand/实验记录/Robot-Controller/restored/SUPPLEMENTAL_RECORD_ID-files"
```

替换清单路径和示例记录 ID，两个目标目录均须尚不存在，SanStand 须已挂载。第二步不访问网络，也不上传或清理任何记录；恢复参数不能与准备、上传或清理参数混用。恢复出的 `.gz` 保留归档时的字节，不自动解压原日志。

## 持续维护服务

当前 `alicia-grasp-archive.service`、`alicia-grasp-archive-unknown.service` 和 `alicia-archive-finish-20260927.service` 已停止并禁用自动启动。两个外盘根及两个旧源根的 `archive_config.json` 均设置 `upload_enabled=false`、`delete_enabled=false`、`archive_policy=local_only`。`repo` 仅保留为已上传历史的查询信息，不代表上传授权。上传工具默认关闭上传；恢复/查询现有历史仍可用。

`alicia-active-log-mirror.service` 继续每 5 秒增量镜像本轮仍使用旧文件描述符的 ROS 日志，状态保存在外盘 `ros_log/.active_ros_log_mirror.json`。监督日志已经直接写外盘，不再从旧内部文件覆盖。`alicia-log-spool-cleanup.timer` 仅处理关闭后的系统盘重复副本：两端 SHA256 一致后移除内部副本，外盘原件保持完整，代码、配置及打开的源文件保留。

`system_disk_migration_cleanup.json` 保留逐文件原始身份、目的身份、大小和 SHA256，解释 exFAT 时间戳精度变化。已生成的历史 `legacy/`、`incomplete/` 索引和未完成补充打包文件继续保留作追溯材料；取消上传不删除这些外盘数据，也不把原上传队列显示为已经上传完成。当前状态见 `storage_status_20260927.json` 和 `maintenance/archive_finish_20260927/status.json`。

## 外盘晚挂载与后台暂停

`alicia-sanstand-recovery.timer` 每分钟检查外盘。`local_only` 策略下只恢复日志镜像，禁止启动两个归档 worker 和历史 finisher；已经运行的镜像进程保持原样。它不重启 ROS，不修改机械臂使能。挂载不可用时不写入空挂载目录，也不回退系统盘。

持久控制文件为外盘 `maintenance/storage_control.json`，当前内容含 `state=resumed`、`archive_policy=local_only`。恢复入口及存储服务的 `ExecCondition` 读取同一标记，区分本地镜像/系统盘重复副本迁移与 GitHub 上传。`paused`、标记缺失或损坏时不启动任务；普通暂停/恢复保留现有 `local_only` 策略。旧 `/tmp` 临时标记不作为持续上传授权。

收到明确的后台暂停指令时，在项目工作树执行：

```bash
python3 tools/recover_sanstand_archive_services.py --set-state paused --reason '用户明确暂停本地存储维护' &&
systemctl --user stop alicia-sanstand-recovery.timer alicia-log-spool-cleanup.timer alicia-active-log-mirror.service alicia-log-spool-cleanup.service
```

收到明确的继续本地维护指令后：

```bash
python3 tools/recover_sanstand_archive_services.py --set-state resumed --reason '用户明确继续本地存储维护' &&
systemctl --user start alicia-sanstand-recovery.timer alicia-log-spool-cleanup.timer &&
python3 tools/recover_sanstand_archive_services.py
```

这些命令不会启用已禁用的 GitHub 服务。设置标记要求 SanStand 已挂载；缺盘时应停止维护定时器和服务，挂载后再保存暂停状态，不把写入失败当作状态已持久保存。后续新增材料直接完整保留在外盘，不追加 GitHub 队列，不修改以前的不可变恢复索引。
