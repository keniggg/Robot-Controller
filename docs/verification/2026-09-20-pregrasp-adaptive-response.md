# 2026.9.20 预抓取部分响应补偿修复

后续实机实际执行一步后发生J5振荡，用户现场确认；最新证据、从开始保持J5的修复及未实机验证边界见[腕部振荡记录](2026-09-20-wrist-oscillation.md)。本页保留此前阶段的设计与试验结果。

针对[首次实机补偿失败](2026-09-20-pregrasp-compensation-live.md)，修复了“预测每个轴都按 SDK 指令增量运动”和“只要一个轴响应不足就停止整个补偿”的问题。原视觉目标、6 mm / 5° 门、每步4计数、每轴累计32计数、12步/60秒预算、0.035 rad 跟随限制及所有路径检查保持。

## 实测原因与修复范围

首次实机 Joint5 SDK 1137→1141，actual 1140→1140。新命令只比实际高1计数，不能按4计数实际位移预测；但也不能据此确认死区或硬件损坏。Joint1/3/4/6 本步分别响应 −2/+3/−4/+4，位置误差实际已下降1.783 mm，旧实现却因 Joint5 单轴零响应终止全部补偿。

新策略：

1. 在编码器网格上选小步时，不给“落在实际位置周围不足2计数范围内”的新目标计算有效移动收益。SDK与实际的偏差可正可负，偏差符号本身不被当作物理响应方向证明。
2. 实测响应至少同向2计数的轴可继续；方向正确但不足2计数的轴保持其**最后 SDK 指令**至本轮结束，不重新累加，也不回写成实测角度。只有其他轴存在有效响应、末端位置误差确实改善，才允许重新规划下一步。
3. 按刚完成一步的实际/指令增量比更新预测增益，最多取1，不将模型预测当成执行成功。候选还须检查“仅响应2计数时”的跟随余量，提前选小步，避免已部分响应的轴耗尽0.035 rad差值限额。
4. 以位置误差下降为优化目标，姿态保持原5°硬约束；不为继续降低已合格的姿态误差而牺牲位置到位。
5. 全部轴无有效响应、反向运动、过冲、未命令轴额外移动超过2计数、位置不改善、跟随越界或控制权变化仍停止。保持轴在整条最终曲线及控制器接续段中均不得离开同一 SDK 量化格；不仅检查终点。
6. 所有步骤共享原 plan_id + epoch 的同一额度。部分响应后不会新建额度或复位已经失败的实机任务。失败日志也不再混入上一步遗留的预测字段。

这修复了任务级补偿对局部无响应的处理，不等于修复固件、电机、减速器或标定。

## 验证

- 精确存档本次运行时 URDF、原视觉目标、初始 SDK/实际计数、实际第一步命令/响应和匹配的近场计划：`tests/fixtures/pregrasp_following_20260920.json`。
- 新测试重放实机第一步部分响应，确认保持 Joint5=1141 而继续规划其他轴；覆盖首次搜索避开1137→1141的低有效距离、0/1计数弱响应保持、保持轴后续异常运动、完整曲线偏离保持量化格，以及同一额度不可重复调用。
- **96项补偿专项通过**；包含既有任务序列、网关、端点校正、连续轨迹、控制参考与停止包络的受影响回归 **574 passed / 7条既有弃用警告，60.19 s**。未将本次定向回归称为全仓全量测试。
- 以真实起点加明确假设“Joint5始终不响应、Joint3的4计数命令仅响应3计数、其他被命令轴正常响应”，10步得到 **5.744195 mm**，仍在原姿态容差内，10条三秒假设曲线均通过条件几何/跟随检查。全部轴不响应反例仍在第一步停止。
- [假设回放结果](evidence/2026-09-20/pregrasp_adaptive_response_replay.json)。这不是第二次实机结果，也不证明任意姿态都能收敛。

复核命令（仅离线，不连接 ROS master）：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
PYTHONPATH=src/alicia_flexible_grasp_supervisor/src:$PYTHONPATH python3 tools/replay_pregrasp_compensation_readonly.py \
  --fixture src/alicia_flexible_grasp_supervisor/tests/fixtures/pregrasp_following_20260920.json \
  --response joint3_partial_joint5_stalled --response all_axes_stalled --check-paths \
  --output /tmp/pregrasp_adaptive_review.json
```

## 实机验证状态

用户明确回复“已结束手动操作并关闭直控，继续实机验证”后，重新采样确认直控关闭。已加载提交 `ae4b5c81a999089112f463d891a0d232f52fa1d8`：task PID `173083`、gateway PID `173275`，driver 仍为 `7197`。重载前后已采样参数一致、六轴实测位置一致；未修改标定、限额或到位门。

修复版首轮实机目录：`.ros_log/grasp_adaptive_compensation_20260920_004916/`。远场计划 `569cf53dd62349b9f131b7fa` 已执行，观察及一次双侧表面补拍动作均得到控制器成功；补拍后的实测 CAD 支撑面净空约79.805 mm。但 **尚未进入接触预抓取与补偿**，近场规划因融合表面配准失败而停止，最终 `NEAR_FIELD_NO_HARD_SAFE_CANDIDATE`。没有接近、闭合、提起，夹爪保持49.75 mm。

录包定位到具体原因：旧参考帧 `1789891037440298318` 与补拍参考帧 `1789891085128008127` 的配准返回 `YAW_BOUND_EXCEEDED`。原始同时间戳 RGB-D、检测及 TF 离线复现的最终支撑面 yaw 修正为 `−14.3310946°`，超过原10°上限；重叠率约69.30%、RMSE约0.909 mm、最大测量点位移约12.959 mm。`multiview_surface.active=false` 后，接触候选因缺少当前绑定的融合表面无法产生。这是补偿之前的观测阻塞，不能记为新版补偿失败或收敛成功。

仅离线比较了不采用PCA初始化及增加迭代次数：前者给出另一局部解，但原实现选择的解具有更低的含未匹配惩罚的几何代价；后者仍超10°。没有因另一个解能过门就修改在线选择、放宽角度或换掉失败阶段的参考。随后从补拍后的清晰视角开始新的完整视觉任务，另存 `.ros_log/grasp_adaptive_clear_view_20260920_010632/`，不复用旧计划或补偿额度。

首轮[结果与配准摘要](evidence/2026-09-20/pregrasp_adaptive_first_live_result.json)、[本机证据校验清单](evidence/2026-09-20/pregrasp_adaptive_first_live_manifest.json)。图像与录包保留本机。

第二轮远场计划 `1d01faf79f9a7880b2a669af` 有效，任务按既有规则复用当前观察位，没有新关节轨迹。近场 phase 3 以 `1789891762820106744` 建立新参考，request 259 / generation 41 在准备14.810 s（其中几何2.571 s、桌面候选12.029 s）后，结果年龄16.393 s超过原15 s输入有效期，被 `RESULT_EXPIRED` 丢弃。此后本阶段没有第二个请求，90 s截止时任务终止 `NEAR_FIELD_DIRECT_TIMEOUT`。仍未进入补偿或闭爪。[第二轮结果](evidence/2026-09-20/pregrasp_adaptive_clear_view_result.json)、[校验清单](evidence/2026-09-20/pregrasp_adaptive_clear_view_manifest.json)。

## 近场过期请求占位修复

上述第二轮揭示独立的软件状态问题：`single_snapshot_direct` 提交后保留 generation 占位，worker 因源时间过期在候选接受之前丢弃结果，却未释放占位，因此采样循环永远返回，直到任务超时。修复仅允许 `RESULT_EXPIRED` 在同一目标/同一generation、仍活动且原绝对截止时间内释放占位，重新采集比原请求更新的帧；参考表面、目标身份、源时间水位及截止时间均保持。不接受过期结果，不增加图像有效期，不对几何淘汰或候选接受异常重试。

占位写入移入提交锁内，在唤醒worker之前完成，避免快速完成后被poller写回旧占位；采样与提交均重查原deadline。新增真实worker/采样集成回归覆盖过期后新帧成功、重复源拒绝、到期停止、参考与期限保持；负例覆盖几何淘汰、接受异常、停止、换代、换目标与旧阶段帧。两个既有时序测试的模拟时钟同步推进到其新阶段时间，避免测试在“当前时刻10.0 s却已有10.5 s阶段及11.0 s帧”的非物理条件下运行。

`test_remote_grasp6d_streaming.py`、`test_grasp_task_sequence.py`、`test_multiview_surface.py` 合跑 **742 passed / 7条既有弃用警告，111.64 s**。diff检查通过。此状态修复不改变预抓取补偿算法；加载后实机结果另行补记。

已提交并核验 GitHub 开发分支为 `6cce389183f2a0a56fd1c44e401f488c605ebf2f`。再次确认直控关闭后，只重载remote：PID `45418 → 184698`；task/gateway/driver均保持。已采样的 `/grasp`、`/robot`、完整 `/grasp_6d` 参数及重载前后关节位置一致。新实机记录目录 `.ros_log/grasp_expiry_recovery_20260920_011633/`，`deployment.json` 保存PID、提交及源文件哈希。
