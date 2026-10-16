# 2026-10-13 GEN18 节点与上位机恢复，抓取保持暂停

用户最新明确范围为“先恢复节点与上位机，暂不开始抓取”（回复 `call_c09441fef4754b438699de661f40578e:0`）。随后“继续处理”用于完成恢复复核及记录，不扩大为开始抓取。没有启动新抓取轮次、选择新目标、切换自动模式或调整机械臂姿态。

本次外盘会话为 `/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen18_restart_20261014T012216Z`，以下证据路径均相对此目录。目录时间使用 UTC，本文日期使用用户所在的 America/Los_Angeles。

## 修复与验证

GEN18 修复了“高度重验阶段全部拒绝、MoveIt 实际检查为零，却报告全部路径不可达”的分类和证明缺口。完整明确拒绝、未完成重验、没有路径输入分别记录；remote、task、runner 共同验证后才继续判断补视机会。有真实合格候选时保留原成功路径。未放宽接触高度、碰撞、期限、次数和真实执行反馈要求。

- `verification/gen18_focused/result.json`：590 项通过，失败、错误、跳过均为零。补齐此前缺盘时未执行的文件交接和外盘 fixture 验证。
- `verification/gen18_combined/result.json`：184 个测试文件、5815 项通过，失败、错误、跳过均为零；运行前后 924 项源码摘要一致。
- `verification/gen18_independent_source_review.json`：独立源码复核通过，无阻塞项。该复核原回执覆盖 focused；随后另行只读核对组合 XML、日志及当前源码，也全部一致。
- `verification/gen18_release_manifest.json`：摘要 `59eeb809d81e107156e0629b1295255faf6f9d6754d87086567464570c7bbe18`。封存源码、测试、复核和启动依赖；其中 `installed` 指实际工作树源码已安装，运行加载由后续冷启验收证明。

第 6 轮原始记录的复读及证据边界见 [R6 原始证据核对](2026-10-13-r6-recorded-recheck-evidence.md)。该轮仍是失败，不能把源码回归或本次启动验收写成抓取成功。

## 恢复与运行验收

实际主机侧确认 SanStand 已挂载可写、机械臂串口 `/dev/alicia_arm` 指向已核实 USB 序列号的设备、D405 已连接、桌面可用。未改设备身份映射。新启动 core 与 simulation 两个独立用户服务，使用原已审查的手动模式启动链。

`cold_restart_acceptance/result.json` 的 `deployment_passed`、`whole_patch_deployed`、`all_processes_replaced` 均为 true：

- 11 个核心节点均为本次新进程，来源、进程组和实际源码一致；另有 TF、状态发布、日志及监控等配套节点。
- 驱动接受了 20 帧 SDK 实测关节反馈；验收时最新帧年龄约 41 ms，属于本次启动的新连接 epoch。
- 机械臂与夹爪控制器均为 running；抓取任务 IDLE，router 没有活动或预留任务。
- 上位机窗口实际映射到桌面；GUI 自身发布了 latched 手动模式确认，不仅是参数为 true。
- 相机连续输出新的 RGB 与深度帧；`idle_gui_camera/result.json` 的两路帧年龄约 43 ms，均收到至少 5 个不同时间戳，存在有效深度。
- 本地仿真服务的模型及手眼摘要匹配；`restored_status.json` 记录远端推理服务 HTTP 200、ok=true。

中断后再次核对两个服务仍为 active/running，并以 `final_idle_gui_camera/result.json` 保存最后的只读手动、任务空闲及相机检查。所有状态均为对应验收时刻的快照，不是对未来运行的保证。

## 待机状态与尚未验证的部分

恢复时相机画面朝向地面和背包，并非上一轮抓取桌面。`restored_status.json` 的 5 秒只读采样收到 31 条 `perception_failure: support_plane_not_found`，该次采样未取得可抓目标。router 先前保留的 `exact_rgbd_stream_stale` 是启动阶段状态；直接读取该时刻感知状态后，确认未找到支撑平面，不能据旧文字认定相机停止出帧。

用户后续“继续处理”后，`final_idle_gui_camera/current_scene.png` 已是桌面和小纸盒，左侧仍可见人的前臂；不能沿用早先朝向地面的描述，也不能据此认定现场已清空。最终相机帧约 39 ms，新帧检查通过；GUI 保持手动、任务 IDLE。router 的 generation 仍为 1，状态为 `waiting_for_target`、`unavailable:target_lost_requires_new_generation`，因此后续执行需要重新选择目标。本次没有为解除该状态自动重新选目标或启动任务。

相机日志仍记录个别 RGB-D 原始包时间差超过配对门限而被丢弃；连续新帧检查通过，但未进行长时配对稳定性验收。旧 carton/YOLO 分支另有依赖未安装告警，本次所选 unknown/observe_return 分支独立运行；未宣称 carton 检测可用，也未为本次恢复安装无关依赖。exFAT 不支持 ROS `latest` 符号链接，实际日志仍写入外盘 UUID 目录。

自动抓取控制保持原 `paused`、revision 11；`operator_restart_only_instruction.json` 记录本次仅恢复节点的新指令。没有消耗新一轮抓取授权，没有宣称现场已清空或姿态已对准，也没有生成新的实物抓取结果。后续实际抓取需要用户恢复执行后重新确认当前姿态和现场，并使用新会话目标。

## 存储与机械臂约定

测试、启动日志、验收 JSON 和画面均直接保存 SanStand。冷启验收时可用约 13.49 GiB，高于用户授权的 10 GiB 清理触发值，未执行失败记录清理。保持 local_only、上传关闭、通用删除关闭；未启动上传或归档 worker。

温度监控仍为 false、轮询频率 0.0；没有查询温度、基于温度作控制或发送关节失能命令。驱动按原启动配置进行正向使能；这属于用户授权的节点重启，不等于启动运动任务。
