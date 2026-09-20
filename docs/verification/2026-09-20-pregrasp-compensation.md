# 2026.9.20 第二阶段预抓取软件补偿

**后续实机状态：** 用户随后授权加载并运行，已完成一次实机尝试：补偿一小步使 FK 残差 18.33 → 16.55 mm，但 Joint5 +4 count、实测 0，按响应门停止，未闭爪。详见[首次实机记录](2026-09-20-pregrasp-compensation-live.md)。下文未部署状态描述的是此前代码开发阶段。

**最新修复：** 针对该失败已实现[按逐轴实测响应继续补偿](2026-09-20-pregrasp-adaptive-response.md)：保持弱响应轴，以有响应轴继续，所有步骤共享原预算；以下为初版设计与历史回放。

## 修改前备份

已上传并远端核验：`9a1ee9790d4e0b9875a8a864d03a99b44cad106f`，提交备注为 **backup: 2026.9.20 上传最新完整代码及抓取实验记录**。

- [完整修改前快照](https://github.com/keniggg/Robot-Controller/tree/backup/2026-09-20-before-pregrasp-compensation)
- 原实验分支 `codex/protocol-v3-upgrade` 同步到该提交。
- 后续开发分支 [`codex/pregrasp-compensation-20260920`](https://github.com/keniggg/Robot-Controller/tree/codex/pregrasp-compensation-20260920)，仍位于本地 `.worktrees/protocol-v3-upgrade` 目录；目录名不等于当前分支名。
- [问题、实验与原始 JSON 归档](2026-09-20-pregrasp-baseline.md)。该快照新增/更新 126 个文件，包含此前未提交的源码依赖、配置、测试、工具和记录。

## 本次实现

用户要求保持视觉自动定位，不依赖固定摆位或示教。此实现只用于 `CONTACT_EXECUTION_PLAN` 的 `MOVE_PREGRASP`：正常严格预抓取运动结束、真实反馈稳定后，按冻结计划的 `poses[0]` 计算位置和姿态残差。若复用同一名义预抓取位，也必须验证实测到位。

新增 `/supervisor/compensate_pregrasp`（`CompensatePregrasp.srv`），由任务节点携带完整冻结计划调用。视觉目标、计划 ID、物体几何和验收目标不随补偿命令移动。普通 GUI、远场观测、最终接近、闭爪和提起不调用此补偿入口。

本策略保持 **Joint2 的 SDK 目标**不变，利用其余五轴的小步命令降低 Cartesian 残差。保持目标不是锁死机械轴；Joint2 实际位置继续监测，若出现超过允许静止量的非指令运动，同样失败。该策略利用现有 5° 姿态容差，不能保证任意姿态或严格六维零误差都能达到；搜索无可行改善即退出。

每步在编码器计数网格上搜索，增量只取 −4、−2、0、2、4。预测从当前实际关节角出发，命令则叠加到成功下发的 SDK 参考上，避免把跟随偏差误当成新的起点命令。新的 SDK 命令和原视觉物理目标分别保留。

| 约束 | 实现 |
| --- | --- |
| 到位门 | 保留配置中原有位置 ≤6 mm、姿态 ≤5°；允许更严格配置，不接受放宽 |
| 初始补偿范围 | 位置残差 ≤25 mm，姿态已在原容差内，SDK/反馈逐轴差不超过既有 0.035 rad |
| 单步 | 每轴最多 4 count；规划曲线和控制器接续段也检查局部范围 |
| 总量 | 每轴相对本轮初始 SDK 指令最多 32 count，包含完整规划曲线 |
| 预算 | 最多 12 步、60 s；下一条轨迹及反馈/控制器完成时间必须能放入剩余预算 |
| 反馈 | SDK 与 accepted 编码器共享至少 0.3 s 静止窗口，源戳新鲜、同一控制 epoch；每步反馈窗口完全晚于动作完成 |
| 响应 | 每个实际被命令轴至少同向 2 count；过冲、未命令轴异常运动、无响应或误差代价不改善均退出 |
| 轨迹 | 使用现有严格 MoveIt 精确关节规划；最终定时曲线、夹爪 CAD、支撑/目标、关节限位、控制器参考及跟随合同重新验证 |
| 控制权 | 仅活动预抓取阶段；手动接管、断开、epoch/模型/配置变化、运动确认失效均撤销；不重置旧诊断额度 |
| 重复调用 | 每个 plan_id + epoch 的运动额度在提交前写入参数，网关重载不能重置；失败不自动回退、累加或关闭扭矩 |

此预抓取阶段选择任务级补偿，**不同时获取驱动层端点 trim 租约**，避免两个补偿回路同时修改目标。其他接触阶段保留原精度租约。服务返回收敛后，任务还会再次通过原连续到位检查与实测端点记录才允许后续接近。

## 小步路径停止预留

第一次离线几何复核发现，通用 0.08 rad/s、0.30 rad/s² 的最坏停止预留使原记录起点的第一小步不能通过。保留该失败报告，不将数学收敛当作路径已通过。

现只对此补偿路径采用最终定时曲线及控制器接续段的**已验证速度/加速度上界**计算同一 Noetic 停止模型的预留：`0.625*v_bound*T + 0.07*a_bound*T² + epsilon`。仍保留最大 0.08 rad/s、0.30 rad/s²、0.5 s 的限制以及原 SDK 半计数和路径跟随误差。发送器从同一最终轨迹及绑定控制参考独立重算这些导数，并严格匹配审计合同；缺参考、改曲线或改预留拒绝发送。远场观测仍使用原保守停止预留。

这不降低 3 mm 几何净空，也不是机械超调或真实停止距离的认证。物理执行仍受实时实际 MoveIt 路径、碰撞场景和控制器反馈约束。

## 离线结果及证据边界

使用 2026-09-19 记录中的真实 URDF、原预抓取视觉目标、**校正前** SDK/反馈计数和匹配的夹爪开口 49.75 mm。该起点误差是 **20.439075 mm**；不能与原运行完成部分校正后的 **15.744665 mm** 混作同一时刻。

假设其余被命令关节每步均按指令响应且静态偏差保持，Joint2 指令和反馈均保持不变：8 步后模型内残差为 **5.661700 mm / 2.074579°**，达到原 6 mm / 5° 门。Joint2 指令始终 1525 count，反馈始终 1506 count。原始关节跟随差没有被抹去；末端目标由其他关节在姿态容差内补偿。

每步构造的三秒假设曲线均通过当前冻结场景的条件几何/跟随检查。第一步选到每轴 0.06 rad 的显式路径合同，按曲线计算的停止预留约 0.001268 rad。**这些是假设曲线，尚非实时 MoveIt 的轨迹，也不是已执行证明。**

无任何轴响应、或 Joint3 不响应的两种反例，都在第一步反馈后退出，没有第二次补偿。

- [原通用停止预留的回放（第一步拒绝，历史对照）](evidence/2026-09-20/pregrasp_compensation_replay.json)
- [最终轨迹停止预留回放（8 条条件路径通过）](evidence/2026-09-20/pregrasp_compensation_checked_stop_replay.json)
- 数据源与 URDF/冻结计划：`tests/fixtures/pregrasp_following_20260919.json`，模型哈希与原始记录一致。

可重复的只读回放：

```bash
cd /home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade
source /opt/ros/noetic/setup.bash
source devel/setup.bash
PYTHONPATH=src/alicia_flexible_grasp_supervisor/src:$PYTHONPATH python3 tools/replay_pregrasp_compensation_readonly.py \
  --check-paths --output /tmp/pregrasp_replay_review.json
```

输出文件必须尚不存在，避免覆盖历史证据。工具不连接 ROS master，不发布运动命令；模拟时钟与响应假设均写入输出。

## 验证与上线状态

- `catkin_make --pkg alicia_flexible_grasp_supervisor -j2 -l2` 完成，新服务已生成。
- 230 项补偿、控制参考、停止包络和轨迹执行专项测试通过。
- supervisor 完整测试目录首轮为 **3080 passed / 16 failed / 3 skipped**；16 项失败均发生在本地模拟 HTTP 服务创建套接字时，原因是沙箱 `PermissionError`。仅将这 16 项以允许本地套接字的环境复跑，结果 **16 passed / 120 deselected**，未连接 ROS master 或实机。
- 随后补上复用预抓取位的验收分支和两个用例；最终新任务补偿模块与既有 `test_grasp_task_sequence.py` 合跑 **243 passed**。以上是完整回归加最终变更的定向复核，不宣称最后一次单命令全量结果；详细命令、日志摘要和源码哈希见[验证清单](evidence/2026-09-20/validation_results.json)。
- `git diff --check` 通过。
- 当前配置 `pregrasp_cartesian_compensation_enabled: true`；这是源码配置，未在本轮重启驱动、重载任务节点或触发新的物理抓取。
- 新版服务使用前需要从本工作树 `devel/setup.bash` 加载消息环境，确保 task 与 motion_gateway 同时使用新代码；仅改 IDE 根目录旧副本不会生效。
- 首轮实机必须记录逐步 SDK 指令、accepted/原始反馈、位置/姿态残差、路径审计及终止原因；只有后续接近、闭合、提起并保持物体才能报告抓取成功。

**尚未证明实机收敛或成功抓取。** 本次完成的是可执行代码、离线数学/几何验证和回归；固件目标接收、真实负载响应与绝对位置精度仍保留为未决项。
