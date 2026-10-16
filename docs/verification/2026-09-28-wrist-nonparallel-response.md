# J4/J6 非平行轴往返诊断

新增独立 `.runtime/bounded_wrist_response_diagnostic.py`。用途是采集相对 J2/J3 非平行的手腕轴运动，比较固定板、深度和实际反馈坐标链。不会改手眼、相机内参或机器人模型，也不会自动执行抓取。

命令入口（默认仅准备，无 `/joint_commands` 发布器）：

```bash
source devel/setup.bash
tools/with_local_realsense_sdk.sh /usr/bin/python3 -u \
  .runtime/bounded_wrist_response_diagnostic.py --joint 4 --step-deg 2 --direction 1
```

CLI 仅允许 J4 或 J6、2° 请求量、正/负方向。执行需独立进程显式 `--execute`，该进程重新采集并重新证明，不消费历史 plan。实际请求是 22 个编码器字，约 1.933594°；严格最多两条命令：出向、恢复初始 SDK 原字。仅接受 CONFIRMED、手动直控、任务空闲、新鲜反馈和同一正使能 epoch，无 bootstrap、同步、使能、补偿、停止、去使能、自动恢复路径。

物理响应界始终固定在本轮初始实际姿态：编辑轴 ±4.8°，其余五轴 ±0.4°，不随返程中心漂移；每段反向响应另限 0.5°。夹爪反馈必须留在本轮初始实际值与持有 SDK 值构成的冻结区间，所有命令保持其 SDK 字不变。SDK/actual 的现存偏差单独建模，不当成实际响应余量：当前 J3 持有 SDK 高约 0.615°，在 J4/J6 实验中仍是未编辑轴，实际漂移界保持 0.4°；其名义 SDK 桥接须单独证明，所有名义端点仍受初始 ±4.8° 限制。

完整实际六维响应盒和完整名义 SDK 六维盒分别分格。编辑轴 cell 宽最多 0.6°，其它轴最多 0.4°，每个 cell 全部角点和内点都在其中心的 L1 球内。每球使用纯解析证明，而非把网格点测试宣称连续证明。MoveIt 检查所有 cell 中心和实际/SDK/目标连线采样；明确这不是全臂连续碰撞证明。原 40 mm 夹爪平面净空、20 px 整板边缘、120 mm 板深度门限保留。

每个 cell、每个板角的相机坐标 enclosure 保存在内存及外盘 `continuous_cells_*.npz`，最终逐 cell/逐角做正深度归一化，再合并 normalized 域并计算 SDK 畸变区间，避免跨姿态或跨板角混合 XYZ 分母。发布前的新板观测会按各角实际世界位移扩大对应区间，平面变化按覆盖完整 cell 的 CAD 距离界扣款，重新检验全部物理门。消息构造后再次验证板源龄 ≤350 ms、实际/SDK 源龄 ≤300 ms、所有源时间不早于冻结 epoch。

85 秒整体期限，单段最多等待 12 秒独立停稳；每段完成后保留 3 秒持有记录并保存新板观测。失败不发下一条命令，包含不自动返程。`/bounded_wrist/paired_rgb` 和 `/bounded_wrist/pair_provenance` 保留精确 RGB/板姿态来源配对，原深度/TF/SDK/实际反馈话题同时录制。所有新运行数据直接写 SanStand，挂载不可用即退出。

50 项腕部离线测试通过：单轴字增量、精确原字返回、第三条拒绝、顺序/状态锁存、SDK 与实际偏差隔离、初始固定界、五个未编辑轴的完整 cell 覆盖、冻结夹爪、epoch/源龄、默认不创建关节发布器，以及真实执行分支的内存发布 stub。最后一组验证当消息构造令板源龄从 349 ms 越过 350 ms 时实际发布数为零，出向失败也不会调用返程。未通过真实硬件动作宣称抓取或标定已成功。

交付 SHA256：

- wrist 脚本：`3131042c4e7004f4ec9384bf7716b02379057398c59679f30e80ab60f9ed6a5c`
- helper：`d06204916f0f8e854a734aa74886157405bc209056a24841535f9bc6e7ba2116`

## 实机闭袋后的独立审计

根代理执行的 J4 `bounded_wrist_response_diagnostic_20260928T125306Z_126028` 与 J6 `bounded_wrist_response_diagnostic_20260928T125421Z_130257` 各发布两条，并准确恢复原 SDK words。只读审计保存在各 session 的 `fixed_target_hold_review.json` 与 `publication_start_audit.json`；审计未发布任何命令。

四个停稳后的 3 s 保持窗，raw-before-filter、accepted、sdk_command 和 control_reference 六轴 peak-to-peak 均为 0。每窗约 27 个 raw / accepted 样本，覆盖 2.8–2.9 s，最大 raw 间隔约 0.2001 s。反馈约 10 Hz / 0.087890625° 一个编码字，该结果不能排除更高频或低于一个字的机械振动。J4 出向出现约一个字的小回摆后稳定，J6 未见相似持续回摆。

相同 SDK 返回后，J4 实际相对初始为 -0.17578125°，J6 为 +0.263671875°，其余轴不变。它说明写入目标相同不保证实际角度完全返回，尚不足以归因为约 11 mm 的抓取偏差。

四条 command 的 starting_measured_rad 与发布前最后 accepted 六轴逐值相同，发布后首帧也相同；发布前帧龄依次为 57.894 / 90.705 / 30.412 / 27.474 ms，没有触及规划到发布间的起点漂移边界。

新增 4 个 J4/J6 × 正负方向离线边界测试：将真实发布起点沿命令方向改变 0.1°，再构造位于旧固定几何 box 内、相对新起点反向 0.6° 的反馈。现有独立 signed reverse guard 均拒绝，4/4 通过；此前 50 项回归未重新运行，执行源码保持原 SHA 不变。
