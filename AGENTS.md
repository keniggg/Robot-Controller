# 实验记录存储约定

用户指定 SanStand 为实验记录和运行日志的主存储：
`/media/zhuyupei/SanStand/实验记录/Robot-Controller`。

- 本次最新指令已取消继续将全部实验上传 GitHub。采用 `local_only`：外盘完整保留所有新旧实验，不再按最新 5 份自动删除；此规则覆盖旧的全量上传及五份保留约定。
- 查询先读外盘 `ros_log/`、`grasp_records/`、`文档/`。只有以前已经上传的历史可到 `keniggg/Robot-Controller` Releases 查询，按保留的 manifest/恢复索引验证；不自动补传、下载或删除云端历史。
- 新实验和日志直接写外盘。挂载不可用时明确报告，不静默回退到系统盘。
- 上传配置保持 `upload_enabled=false`、`delete_enabled=false`。`maintenance/storage_control.json` 为 `resumed` + `archive_policy=local_only`；日志镜像/系统盘重复副本迁移可以继续，归档 worker 和上传 finisher 不得由恢复定时器重新启动。
- 当前 ROS 节点可能仍持有系统盘日志文件描述符；持续镜像至外盘，等写入者关闭并核对两端 SHA256 后才移除系统盘重复副本。不能删除外盘原件，也不要为换日志路径重启机械臂节点。
- 源码、构建产物、启动脚本及版本管理中的操作文档留在项目。运行说明见 `.worktrees/protocol-v3-upgrade/docs/operations/experiment-storage-sanstand.md`。

## 用户明确授权的单次失败实验清理例外

- 2026-10-07 用户要求先进行约 60G 清理，规则与此前约 50G 清理相同，并明确要求记录到文档、清理后继续未完成问题。
- 沿用此前实际执行口径：从最早且有证据确认失败、已关闭的实验开始，删除大体积原始录制，SanStand 可用空间达到约 60 GiB 后停止；不持续维持该空间，也不自动再次清理。
- 每个处理目录先保留/生成问题发现、解决思路与方法、验证结果摘要；保留原结果、授权、文本日志、审计与恢复元数据，以及精确删除回执。不能把失败摘要写成成功。
- 成功/实物夹持成功、结果未知、标定、当前正在写入及当前问题定位仍需要的记录排除；不删除打开的文件，不按文件夹名称猜测失败。
- 此例外仅覆盖本次用户授权的限定清单。`upload_enabled=false`、`delete_enabled=false`、`archive_policy=local_only` 保持；不启用归档自动删除或上传，不改云端历史，不为清理重启或失能机械臂。
- 执行计划、回执与核验保存在外盘 `maintenance/user_failed_cleanup_*`；具体记录见项目存储操作文档及外盘文档镜像。

# 机械臂温度处理约定

用户明确要求以后不查询机械臂温度，也不对温度作软件反应。

- 保持 `software_temperature_monitoring_enabled=false`、`temperature_poll_rate_hz=0.0`；启动、重连和诊断均不主动查询温度。
- 不读取温度数值用于诊断，不根据温度发布告警、阻断动作或调整控制；不发布任何关节失能命令。
- 保留原始协议报文和非温度的一般故障、真实关节反馈与执行确认规则；不修改硬件固件保护。
