# J2/J3 反向等量SDK实验包装器

本文件描述实验包装器及各版证据。最新客户端已改为每个逻辑段 J2→J3 两次 one-hot 发布，86 项离线测试通过，尚未由本代理执行；旧双轴消息版本已废止。最新只读准备只证明了 positive 方向；negative 的连续视野下界 19.095 px 未达到原 20 px，因此本版本当前姿态不能作为四步实际执行的通过依据。所有本代理准备过程均为 0 条关节命令，没有同步、使能、失能、停止或自动恢复动作。

## 控制合同

`.runtime/bounded_joint23_counterrotation.py` 默认只读准备，不创建 `/joint_commands` publisher。仅 `--execute` 才允许预声明逻辑段：默认 positive → 原 SDK → negative → 原 SDK 四段，每段 J2→J3 两条 one-hot 消息，最多八条发布；显式 `--positive-only` 仅 positive → 原 SDK 两段，最多四条发布。默认 2°，可显式选择 3° 或 4°；分别量化为 22、34、45 个 SDK 字。J2/J3 字变化严格等量反向，其他轴和夹爪命令保持原值。每段均从新鲜反馈、SDK、模型、固定板观测和当前 MoveIt 场景重建；没有旧 plan 输入或 bootstrap/sync/enable/trim 路径。

所有 accepted 回调锁存初始实际各轴 ±4.8° 绝对范围、未编辑轴初始 ±0.4°、每段编辑轴反向 ≤0.5°/正向 ≤4.8° 与初始范围的交集，以及固定实际夹爪开度区间。每条实际 SDK 回调检查同 epoch、当前 one-hot 编辑轴的字区间、其余五轴和夹爪不变；SDK 端点源戳必须严格晚于本条发布时间，端点首次确认后不接受退回旧字。J2 的已写 SDK 与独立 0.8 s 停稳均确认后才取得 J3 的发布前新图并重新证明场景余量。任何故障都不再发送下一条命令，包括自动返回。

已有 SDK 跟随偏置可能在反向 0.5° 物理范围之外。本次 J3 的既有 SDK 比 actual 高约 0.615°。名义 actual/SDK/target 完整独立六维矩形单独连续证明，仍受初始绝对与 held 轴范围约束；它不能扩大 accepted 反馈的物理范围。两种空间的证据与 guard 分开保存。

## 连续几何与最新场景

物理范围分为完整六维 cell：J2/J3 边长不超过 0.6°，四个未编辑轴各二分，典型 cell 的 L1 半宽和 1.388889°，不超过纯 helper 的 2° 接口上限。每个 cell 中心均调用当前 MoveIt，完整 cell 另外使用 CAD 全局运动半径、逆链 Taylor 界和 SDK 2.56.5 外向区间证明 40 mm 平面净空、整块板 20 px 视野边距与至少 120 mm 深度。MoveIt 中心/桥接采样不宣称连续全臂碰撞证明。

整板投影先对每个 cell 的每个物理板角独立做正深度归一化，之后才合并归一化区间并计算 Brown 模型。不得先合并不同板角或不同 cell 的 XYZ，再把无关联的分子/深度组合相除。全部 cell/角的包围保存在外盘 npz；进程中的数组只读。最终新的板观测逐角加入世界位移上界，并对全 cell 的 CAD 半径扣除平面旋转/平移影响，再调用批投影接口证明原门限；不是仅接受“场景变化小于 3 mm”。

消息构造完成后、publish 紧前重新检查 accepted/SDK 源年龄 ≤300 ms、板源年龄 ≤350 ms，以及三者在当前冻结 epoch 之后；更新真实消息和合同的 command stamp。每段停稳后保持 3 s，所有 pose_ok 且同源 RGB/quality/board 精确配对的 RGB 保存到 `/bounded_joint23/paired_rgb`，原始 depth/TF/accepted/SDK 由 base 全程记录。实际 FK 转角残差必须报告，反向 SDK 字不等于真实纯平移；本实验不自动 trim 消旋。

总墙钟期限 110 s，任意超时停止追加命令。期限不因为重新采样或分段更新而延长。

## 只读准备与成本

全部数据在 SanStand `ros_log/` 下。

- `bounded_joint23_counterrotation_20260928T123223Z_69579`：旧准备将 SDK 桥接硬塞进物理矩形而拒绝；已以独立名义桥接证明修复，未扩大反馈范围。
- `bounded_joint23_counterrotation_20260928T123521Z_77346`：旧整板 XYZ 区间的连续 FOV 下界 −15.735 px；同一第 0 cell 的真实中点投影边距 84.75 px、64 角点最小 74.28 px。不是实际出界证据。严格逐板角归一化已修复这种不必要的保守。
- `bounded_joint23_counterrotation_20260928T124632Z_106001`：cell 均通过，但最后复核再次跨 cell 合并 XYZ，已用完整批投影修复。
- `bounded_joint23_counterrotation_20260928T125148Z_120542`：最终批投影版本。positive 通过 1296 个物理 cell、64 个名义 SDK cell、1408 个 MoveIt 点；连续平面 51.056 mm、最终整范围视野 27.594 px、深度 216.060 mm。耗时 23.144 s，其中纯 cell 几何 12.319 s、MoveIt 9.894 s、最终批投影 7.18 ms。negative 初段因 FOV 19.095 px <20 px 被拒绝（平面 80.092 mm），没有完成 negative 证明，更没有发运动命令。

约 23 s 的单段准备成本乘四，再加停稳保持、实际运动及观测，会接近或超过 110 s。不能仅凭 positive 准备通过就直接执行当前四步。需要进一步解决 negative 当前视野证书及准备预算，或选择其他已完整准备通过的独立诊断实验；不能提高 20 px/40 mm 门限、延长期限或删除未编辑轴覆盖来制造通过。

## 验证与归档

54 项离线合同测试通过：四命令上限/顺序、同 SDK 精确返回、真实旋转不等于字相消、逐回调绝对/方向/未编辑轴/夹爪/epoch/SDK 锁存、无停稳不返回、失败不自动返回、完整独立 cell 覆盖、真实 0.615° SDK 偏置隔离、正好 350 ms 与过界 1 ns、场景变更导致原剩余净空或投影失效的反例、逐角/逐 cell 深度绑定、默认只读和无旧 plan/恢复选项。未发起实机执行测试。

源码 SHA 与全部数据摘要记录在本次恢复目录 `counterrotation_implementation/manifest.json`。当前运行监控和任何实机执行由父代理管理，本代理不持有运行中的准备进程。


## 历史：首次显式 positive-only 续接（双轴协议已废止）

父代理随后将下一实验预先限定为 `--positive-only`：最多两条 positive → 原 SDK 命令，默认的四步模式保持不变。这是运行前明确选择的实验范围，不是失败后切换、忽略 negative 拒绝或自动恢复。实际返回仍在出向后重新采集当前姿态、SDK、板及场景，完整重证物理/名义矩形；只读准备不能替代返程的这次证明。

新增合同还将发布时的最新起点响应范围与规划时已证明的范围取交集，防止允许的 0.15° 规划间隙把反向 0.5° 变成更大范围。60 项合同测试通过。

只读 positive-only 准备目录为 `bounded_joint23_counterrotation_20260928T125636Z_137008`，已闭袋，terminal 为 `prepared_no_motion`，关节命令数 0。1296 物理 cell、128 名义 SDK cell、1472 MoveIt 点通过；连续平面净空 51.279 mm，批场景整板边距 28.026 px，最小深度 216.320 mm。准备耗时 25.162 s，其中 cell 几何 13.710 s，MoveIt 10.471 s，最终新场景批投影 3.25 ms。

该只读准备的源码是 ad7a0605...；随后收紧发布起点范围交集的最终执行待审源码为 `e7430e580762c50adb88959abb1d5ece91549c852fae426fb6ae9bd24e791252`。收紧仅改变发布合同，不改变已只读验证的几何；新执行进程仍必须重新生成全部证据。本代理没有发起实机执行。独立新归档目录为恢复记录下 `counterrotation_positive_only_implementation/`，保留原四步版本归档。


## one-hot 协议修正（当前版本）

父代理执行的 `bounded_joint23_counterrotation_20260928T125945Z_146409` 发布了一条旧双轴 effort=[0,1,1,0,0,0,0] 消息。driver 明确返回 `Rejected gui_direct command with invalid one-hot joint intent metadata`，SDK 与 accepted 反馈峰峰值均为 0。该次属于客户端协议错误，不能作为机械臂不响应或低旋转实验结论；故障记录保留，不重复归类成硬件异常。

真实驱动契约在 `src/real-arm/alicia_d_driver/src/alicia_d_driver_node.cpp:1227`：effort 和 name 长度相同且恰好一个 effort>0.5；1375 行附近仅修改该轴，其余轴保持已经准入的 manual goal，成功写入 SDK 后发布 control_reference。修正仅在实验客户端，不改 driver、参数、所有权或使能。

每个逻辑段现在严格执行：

1. 基于当前 actual/SDK/板源及完整独立二维响应盒，生成并连续证明全段几何及名义 SDK 独立盒。
2. 发布只标记 Joint2 的消息，所有其余位置来自当次新鲜 SDK；发布前扣除次数，立刻复核 epoch、accepted/SDK≤300 ms、同源板≤350 ms。
3. 等待 Joint2 精确 SDK 端点，其源戳必须严格晚于本条 publication；检查实际方向响应并独立 0.8 s 停稳。未确认或超时不发布 Joint3，不补发，不自动返回。
4. 获取新的板/图像/TF/accepted 证据，将原完整连续 cell 包围对新场景重新证明。Joint3 当前起点的反向 0.5°范围只与旧 physical 盒相交，不能扩大。再次检查停稳、SDK 前轴保持以及紧前源龄，再发布仅标记 Joint3 的消息。
5. Joint3 同样等待新 SDK 与实际停稳；整个逻辑段共享 12 s 响应预算，总 110 s 期限不延长。整段末保持 3 s 并记录实际 FK 残余旋转及 exact RGB/depth。

这是先 J2 后 J3 的端点对向实验，不是同时双轴运动，不证明真实纯平移。返回仍必须出向后重新采集、计算、证明，不使用起始状态假想返程。任一中间失效均没有后续 publication。

最新源码 SHA256 为 `3c42d876e3340e64d675173ef395b475f3eddbb262ef6ebb56cb7b30ffa703a4`；纯 helper 仍为 `d06204916f0f8e854a734aa74886157405bc209056a24841535f9bc6e7ba2116`。86 项离线测试包含完整 run 的模拟 publisher/7 个关节名称与 one-hot 元数据、真实 driver 单轴保留语义、J2→J3→J2→J3 四条上限、每阶段当前 SDK/源戳确认、未发 J3 却出现 J3 SDK 更新、第二条前图/SDK失效、同时间戳/旧 epoch 假确认、实际无响应、第二轴反向界交集及失败无返回。本轮没有 live prepare 或实机发布。本地新增 `counterrotation_onehot_implementation/` 归档，旧源码和闭袋实验保持原样。
