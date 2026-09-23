# 抓取记录与 Releases 归档

本功能只管理录制数据，不发布 ROS 控制命令。入口为 `tools/run_recorded_grasp.py`，包装操作者已经授权的现有抓取命令。录制仍使用 `record_grasp_keyframes_readonly.py` 原有精确同戳关键帧规则、所有原话题、分辨率、队列、LZ4 格式、默认 600 秒／原上限 1800 秒，以及任务终态后的原 3 秒余量。现有调用若传 240 秒，继续传 240；不得为了归档改变原参数。runner 正常退出不会提前结束录制，包装器等 `.bag` 和日志关闭后入队。

## 运行

```bash
cd ~/alicia_wa_full/.worktrees/protocol-v3-upgrade
source /opt/ros/noetic/setup.bash
source devel/setup.bash
# -- 后原样传入已授权的手动对准交接／抓取入口。
python3 tools/run_recorded_grasp.py --duration 600 -- /path/to/existing_authorized_grasp_command
```

每次产生 `.grasp_records/<UTC时间戳>_<UUID>/`。子命令收到 `GRASP_RECORD_DIR`，可将 `authorization.json` 写入其中。记录包含原始 `keyframes.bag`、原审计 JSON、录制／执行日志、关闭回执、结果证据、`manifest.json` 和持久摘要。不复制完整 bag 作上传缓存。

独立调用录制器仍支持 `--output /root/name.bag --duration 原值`，但 `--output` 的父目录现在作为存储根，在其下新建时间戳＋UUID 子目录；实际位置打印并写入 `recorder_ready.json`。建议统一使用包装器及默认根，保证保留数量按全部抓取合计。不要另起按 success/failure 区分的根目录。

## 结果的含义

- `success`：当前任务 `/grasp/state` 的新鲜 active→SUCCESS 且 success=true，或实际同步 `StartGrasp` 服务成功响应，或操作者明确确认。
- `failure`：实际任务 FAILED、服务失败响应或明确规划失败；记录其原始原因。
- `unknown`：没有实际结果、服务响应丢失或结果冲突。录制时长结束、命令退出码 0、attempt/retry 名称均不是成功证据。
- `interrupted`：中断而没有确定任务结果；异常退出且无法证实正常关闭的 bag 保留本地，不进入删除流程。

人工确认可用：

```bash
python3 tools/grasp_archive.py mark RECORD_ID success --reason '操作者观察物体已夹持并抬升'
```

这会记录人工证据并更新远端结果清单，不改写 bag。只有 `.bag` 已正常关闭、全部登记日志关闭才可入队。重启发现关闭回执完整时恢复入队；若不能证明关闭，则标记 interrupted/incomplete，保留所有本地文件，不猜测成功，也不自动修复或截断原 bag。

## 远程归档与恢复

配置位于 `.grasp_records/archive_config.json`，当前目标是操作者已明确允许公开上传的 `keniggg/Robot-Controller`。凭据从 `GH_TOKEN`／`GITHUB_TOKEN` 或 `gh auth token` 读取，不写入清单和日志。

每条记录使用独立 Release，文件按原始字节流约 1 GiB 分块；每块记录顺序、偏移、大小、SHA-256、附件 ID 和下载位置，另保留原文件 SHA-256。采用确定性名称，进程重启或响应丢失时复用已校验附件。每个 Release 达到 1000 附件自动分卷 `-v002` 等。上传失败按持久清单退避重试，不阻断机械臂。

GitHub 附件 SHA-256 不可用时流式回读核对；删除前重新核对远端。`archive_manifest.json` 同样上传校验，持久本地 manifest、摘要及下载位置不会被清理。未向 Git 提交任何录制数据；`.gitignore` 排除 bags、记录根和验证目录。

```bash
python3 tools/grasp_archive.py preview
python3 tools/grasp_archive.py worker --once
python3 tools/grasp_archive.py restore /path/to/manifest.json /new/restore/directory
```

恢复使用清单中的有序附件直接写入一份目标文件，逐块和完整文件 SHA-256 全部一致后才完成。目标目录必须不存在。下载的 `archive_manifest.json` 也可作为恢复输入；使用同仓库配置。

## 本地保留、容量与启用顺序

**所有结果合计，按完成时间保留最新 5 次完整记录。** 这 5 次也上传备份；正在录制／不完整记录额外保护，不占完整记录的 5 个名额。超过 5 次时，较旧记录只有全部文件和远端清单上传校验通过，删除时远端仍有效，且本地文件仍与登记 SHA-256 一致，才会逐个删除清单登记的数据文件。其他文件、启动脚本、标定、源码、`.git`、其他 worktree 和未登记历史目录均不删除，不递归删目录。

初始 `delete_enabled=false`。先执行纯预览：

```bash
python3 tools/grasp_archive.py preview
python3 tools/grasp_archive.py cleanup       # 仍只预览
```

真实端到端验证使用明确标记为“非抓取”的独立合成 bag，不连接 ROS master，也不改正式采集配置。为了在小样本验证分块，其验证专用块为 64 KiB；正式块始终 1 GiB。

```bash
source /opt/ros/noetic/setup.bash
python3 tools/validate_grasp_archive.py
# 上传、强制流式回读、恢复原文件及混合结果保留测试通过后，才允许启用。
python3 tools/grasp_archive.py enable-deletion --validation-report .grasp_records/validation_report.json
```

默认开始录制前至少需 **2 GiB** 可用空间，`min_free_bytes` 可配置。不足时先尝试归档和已经获准的清理；仍不足明确输出 `RECORDING_BLOCKED_LOW_DISK`，不会启动新的录制／包装器抓取命令，也不会终止已运行机械臂控制。后台和录制器持续监测磁盘余量，录制中报警但不擅自中止机械臂。**最新 5 次可能占用全部容量，因此保留数量不能代替容量检查。** 如果最新 5 次占满磁盘，需要增容或由操作者另行处理，不会为腾空间突破规则。

持久后台服务：

```bash
systemctl --user enable --now "$PWD/tools/systemd/alicia-grasp-archive.service"
journalctl --user -u alicia-grasp-archive.service -f
```

服务异常重启后继续处理持久清单；用户会话重新启动后恢复。是否在未登录时运行取决于主机现有用户 linger 设置，本功能不修改该系统策略。`.grasp_records/worker_status.json` 保存队列和容量状态。旧目录不会根据名称自动接管或删除。
