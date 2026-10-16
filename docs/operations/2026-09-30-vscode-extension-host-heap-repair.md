# 2026-09-30 VS Code 扩展宿主内存分配失败

## 已确认事实

00:49:47（本机 PDT）崩溃的是 VS Code 扩展宿主 PID 99731，随后自动启动 PID 114516。主窗口进程 PID 99361 没有随之重启。ROS 驱动、相机及托管节点没有因此退出；对话与依赖扩展的界面会中断。

本次 Crashpad 转储 652e3f3c-4195-4eb9-a8a2-916c08ab5e64 明确记录 `electron.v8-oom.is_heap_oom=1` 和 `CALL_AND_RETRY_LAST`。堆已用 1,076,494,736 字节，上限 4,294,967,296 字节。00:06:12 的另一小转储也有同样失败位置。不能据此说整机物理内存耗尽，或 4 GiB 堆已经用满。脱符号的原生栈不足以指认唯一责任扩展。

发现并修复的具体高风险路径是 `SunYingkai.codex-usage-monitor` 0.1.1：每 30 秒对近 14 天会话使用 `fs.readFile(..., 'utf8')`，再整串 `split`，且没有并发刷新保护。当前单份会话约 492 MB；这会建立巨大的 JavaScript 字符串及分割结果。崩溃前该插件日志最后停在一次刷新开始。此路径是有证据的嫌疑点，尚不构成唯一因果证明。

## 已保存的修复

- 插件读取改为 64 KiB 分块、1 MiB 单行上限，逐行投影为最新统计摘要；不改写任何会话。
- 同参数的并发刷新合并为一次；没有增加复杂历史缓存，也没有提高堆上限。
- 本工作区 `/home/zhuyupei/alicia_wa_full/.vscode/settings.json` 为 `SunYingkai.codex-usage-monitor` 设置 `extensions.experimental.affinity: 1`，让其在独立扩展宿主运行。未调整其他扩展或用户全局配置。
- 未主动重新加载 VS Code 或扩展宿主。当前进程仍使用已加载代码。读取器补丁在下次扩展宿主启动时加载；独立宿主 affinity 需要重新加载窗口，届时检查宿主分组日志。插件自动更新可能覆盖本地补丁，重新应用前必须核对版本和源码。

补丁及验证脚本在 `tools/vscode_usage_monitor/`；补丁程序绑定 0.1.1 版本，应用前备份原始读取器、扩展入口、package.json 并记录 SHA256。

## 验证与边界

13 项回归通过：Unicode 分块、CRLF、无末尾换行、超大行、重复读取、截断/替换、缺失文件、并发刷新、异常恢复，以及与原插件统计结果对比。192 MiB 合成文件在 `--max-old-space-size=64` 下成功扫描。

对实际近 14 天的 313 个文件做只读验证：读取 1,905,108,449 字节，22,564 个统计事件，最大统计行 907 字节；206 条大于 1 MiB 的行全部属于消息、工具输出或压缩历史，未发现超大统计行或未知超大行类型。耗时约 4.4 秒，进程峰值 RSS 94,756 KiB（约 92.5 MiB）。`--max-old-space-size=64` 限制的是 old-space；该 Node 进程报告的完整 JS 堆上限为 117,440,512 字节。

这些检查验证了修复后的内存分配路径，没有进行当前宿主热替换，也没有证明其他扩展绝不会再次崩溃。将来如果出现大于 1 MiB 的异常统计记录，此统计插件会跳过它；原始会话不受影响。

## 记录与恢复

全部证据位于 SanStand：
`/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/resumed_observe_return_grasp_20260930T040740Z/`

主要文件：

- `vscode_extension_host_crash_20260930.json`
- `vscode_usage_real_history_validation.json`
- `vscode_usage_final_regression.log`
- `vscode_usage_monitor_original_20260930/manifest.json`
- `vscode_workspace_affinity_change.json`
- `vscode_workspace_settings_before_affinity.json`

恢复工具先校验当前补丁、原始备份及扩展 manifest 的 SHA256；默认仅检查，添加 `--apply` 才恢复。恢复工作区设置只撤销本次对应的 affinity 项，保留其他后续设置。任何操作均不会主动重新加载窗口。

```bash
python3 tools/vscode_usage_monitor/restore_usage_monitor.py \
  --extension-dir /home/zhuyupei/.vscode/extensions/sunyingkai.codex-usage-monitor-0.1.1 \
  --backup-dir '/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/resumed_observe_return_grasp_20260930T040740Z/vscode_usage_monitor_original_20260930' \
  --affinity-report '/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/resumed_observe_return_grasp_20260930T040740Z/vscode_workspace_affinity_change.json'
```

官方依据：[V8 分配重试失败路径](https://chromium.googlesource.com/v8/v8/+/refs/heads/chromium/7132/src/heap/heap-allocator-inl.h)、[VS Code 独立扩展宿主配置](https://github.com/microsoft/vscode/issues/164515)。本机 1.129.1 的配置定义和宿主分配实现也已核对；配置修改不会迁移已经运行的宿主。

## 当前旧宿主的立即缓解（08:23 UTC 后）

为避免等待重载期间旧实现再次整读大文件，已利用该统计扩展的配置热响应实施临时隔离：只在本工作区设置 `codexUsageMonitor.codexHome` 为 `/home/zhuyupei/alicia_wa_full/.runtime/vscode_usage_monitor_empty_statistics`，刷新周期临时设为 86400 秒。该目录的 `sessions/` 为空且只读，并附用途说明；没有改 `CODEX_HOME`、真实 `~/.codex`、认证配置或任何会话文件，也不向界面提供旧的假统计。

已在旧宿主的实际日志看到：`Usage loaded: primary=no data secondary=no data files=0 events=0`。PID 114516（start ticks 2317891）和窗口 PID 99361 启动时间不变，所以这一步无需重启即可阻断该周期性危险读取。暂时显示无数据的是 Sun 的统计项；其他扩展不在此设置命名空间内。

一次性 user service `alicia-vscode-usage-quarantine-restore-20260930.service` 独立等待旧宿主退出，不发送任何进程信号。随后要求当前 extensions.json 中恰有一个 Sun 注册项，版本、目录、package.json SHA256 以及磁盘补丁 SHA256 全部一致，再以比较当前临时值的方式仅恢复上述两个工作区设置的原值；用户随后改过的项予以保留。affinity 不撤销。若补丁被更新覆盖，恢复器保持临时空统计源，写明 `blocked_patch_changed`，不冒险恢复旧的大文件扫描。

恢复器代码在 `tools/vscode_usage_monitor/restore_after_host_exit.py`，14 项测试覆盖 PID 复用身份、原值恢复、保留后续修改、SHA 不符拒绝恢复、无关配置保留，以及新版本目录注册但旧目录仍保留、注册重复/缺失/损坏等情形。外盘附加记录为 `vscode_live_quarantine_manifest.json`、`vscode_live_quarantine_restore_state.json`、`vscode_quarantine_restore_tests.log`、`vscode_quarantine_restore_service.log`。此一次性恢复器尚未等到旧宿主退出；不能将其等待状态写成已经恢复。

08:41 UTC 已仅重启上述等待恢复器（新 PID 130784）以加载注册目录防护，VS Code 与 ROS 未重启。此防护补上了“新版本目录已注册、旧目录 SHA 仍有效”的误放行路径。恢复成功后该一次性服务会退出，之后若扩展再次更新，本地补丁仍可能被替换，需要重新核验；没有停用全局自动更新。复核证据为 `vscode_restore_registry_guard_verification_20260930.json`。
