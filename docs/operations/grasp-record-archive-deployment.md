# 抓取记录归档部署结果

已在指定 protocol-v3-upgrade 工作树实现。108 项针对性测试通过；正式录制参数、话题和机械臂控制逻辑保持不变。ROS、GUI、相机及只读监控已恢复，当前保留手动对准，等待用户本次再次回复“已对准”后执行。

## 数据流程

每次时间戳+UUID独立目录；原始bag、日志、实际结果证据、文件大小/SHA-256及上传状态写入manifest。bag和日志正常关闭后由独立持久后台服务上传GitHub Releases，约1GiB原字节分块，支持断点复用与分卷。全部结果合计保留按完成时间最新10次；其他记录仅全部上传和再次验证后逐文件清理，摘要/清单/下载位置永久保留。异常、不完整、未上传或校验失败数据保留。录制开始余量默认2GiB，空间不足先尝试归档清理，仍不足阻止新录制；不终止控制。

## 验证

108项测试包括真实结果判定、混合结果最新5次、并发改写保护、显式登记文件范围、上传重试、正常关闭后崩溃恢复、低空间阻断、分块恢复和分卷。真实合成bag已上传、逐块强制流式回读及恢复原文件SHA-256校验通过。合成验证不连接机械臂、不是实际抓取，其64KiB测试块不改变正式1GiB配置。

远端验证：https://github.com/keniggg/Robot-Controller/releases/tag/grasp-record-20260923T033500.449193Z_838fe1c01c00484a97418bb8c2abc261

实际试运行没有待清理的正式记录，未删除任何实机记录。旧日志目录没有按名称自动接管或删除。后台服务 `alicia-grasp-archive.service` 已启用，可跨进程及用户会话重启继续队列。

## 授权和当前状态

用户明确允许公开上传到keniggg/Robot-Controller。自动删除首次启用被自动审批拒绝，随后用户已明确回复“允许启用上述自动删除规则”；远端重新核验后启用结果为 True。

后续已对准使用 `/home/zhuyupei/alicia_wa_full/.ros_log/ros_live_20260922_session/start_recorded_aligned.sh`，仅在用户本次明确对准后启动。它保留原手动→自动交接和模式runner，外包新的录制生命周期。历史抓取的候选耗时、接触证据及移动支撑面关联问题尚不能报告已解决。

操作文档：`docs/operations/grasp-record-archive.md`。


## 2026-09-23 interrupted upload recovery

Two real 1 GiB assets remained in GitHub `starter` state with `size=1073741824` and no digest. The previous zero-size-only recovery condition retried indefinitely. Recovery now refreshes the matching deterministic asset metadata, removes it only if still `starter`, and resumes the same source ranges. A listing that becomes `uploaded` before the fresh lookup is reused and verified, never removed. This follows GitHub's documented recovery for failed starter assets: https://docs.github.com/en/rest/releases/assets#upload-a-release-asset .

Capacity recovery now attempts already eligible, remotely verified old-record cleanup before waiting for long uploads; it still checks capacity afterwards. In production the first of six completed records was reverified and its registered data cleaned, preserving the latest five, manifests, summaries and remote references. Neither low space nor archiving sends robot control commands.

Validation after these changes: 89 archive and transport tests passed; background archive service restarted.


### Completion-ordered bounded queue, 2026-09-23

The live queue exposed a capacity starvation problem: filesystem iteration could upload newer multi-GiB records before older eligible records, and a whole queue pass held the cleanup lock. `process_queue` now orders by completion time and supports a positive `max_records` bound. The persistent worker processes one eligible record per cycle, releases the lock, and runs the unchanged verified-data cleanup before the next record. Failures retain normal backoff, allowing other records to advance. No deletion predicate, latest-five rule, recording settings, data format or robot control changed. Regression: 95 archive/transport tests passed, including deliberately scrambled creation/completion order, mixed outcomes, cleanup between records and retry backoff. Deployed by restarting only `alicia-grasp-archive.service`.

## Local retention expansion

The current aggregate retention is ten complete records, ordered by completion time across all outcomes. Existing local recordings remain protected while the queue is paused for migration and a new preview is validated. Prior five-record deployment notes above are historical. Upload, SHA-256 verification, restart recovery, file-scoped deletion and recording settings are unchanged. A five-record validation proof cannot enable cleanup under the ten-record policy; capacity admission remains independent.
