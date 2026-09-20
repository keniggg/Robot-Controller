# 2026-09-13 对准交权后的实机抓取：近场异常、跟随残差与修复

## 结论与边界

本轮真实执行了第一观察运动，**未成功抓取，未进入接触、闭爪、抬升**。
本轮直接中断原因是 ROS 近场结果接收代码写入不可变 `mappingproxy`，
随后单次请求阶段没有新结果，任务等满90秒才报告近场超时。不是证明所有
近场路径不可达，也不是接触门限过严。这一软件缺陷与持续11.83mm跟随
误差、原始E1/E2/温度异常是三个不同问题，不能互相替代解释。

操作者本轮已对准交权，执行前GUI直控false。沿用原路线和硬门限；无
`/grasp/stop`、`/demonstration=true`、torque_off、控制器停止/卸载、归零命令。
失败后未重放旧计划、未追加机械臂运动。保留驱动使能和后台纯订阅遥测。

## 可复核材料

工作树：`/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade`。
以下路径相对于根工作区 `/home/zhuyupei/alicia_wa_full/.ros_log/`：

- `grasp_attempt_20260913_aligned_xYlUCg/runner.log`、`parameters_before.yaml`。
- `runtime_resume_20260913_dSFcRQ/alignment_and_attempt.bag`：已正常关闭，
  包含同源RGB-D、SDK/accepted、动作、原始诊断、TF和任务状态。
- 同目录 `alignment_and_attempt_audits/` 保存规划审计副本；近场在完成审计
  之前抛异常，不能把目录里的远场报告误认成已通过的近场报告。
- ROS run `f0e99d88-af48-11f1-8feb-3db53bbffd36` 中的
  `grasp_task_node-13.log`、`motion_gateway-7.log`、`remote_grasp6d_node-11.log`。
- 持续遥测 `telemetry_*.bag`；`.active`卷不当作正常关闭的bag分析。

## 时间链（本机 PDT，原始ROS秒保留用于回放）

| 时刻 | 实际事件 |
| --- | --- |
| 01:08:38 | 启动新源窗口，minimum_stamp_ns=1789286918464268445，候选窗口300秒 |
| 01:10:31 | 新远场plan `bf5197e90f016b773f521982` 严格审计绑定并启动；其源戳1789286981646981954，执行前约49秒，在原120秒有效期内 |
| 01:10:33.254 | 唯一一次 FollowJointTrajectory goal，六轴同时允许运动 |
| 01:10:44.455 | 控制器返回SUCCEEDED/error_code=0，不等于实际TCP精确到位 |
| 01:10:45.428 | 实测请求端点差约11.8mm、2.20°；实际观察CAD净距16.546mm，通过无接触观察判据 |
| 01:10:49.239 | 新相机目标距离0.2100m，在原18–22cm观察范围内 |
| 01:10:49.250 | 进入近场90秒预算，等待新近场结果 |
| 01:11:00.065 | remote完成阶段切换，取消旧阶段推理并重置稳定窗口；不是取消机械臂使能 |
| 01:11:29.494 | request47/generation3：ACCEPT_FAILED，`'mappingproxy' object does not support item assignment` |
| 01:12:19.250 | NEAR_FIELD_DIRECT_TIMEOUT；最后看到的是源戳早于本轮近场窗口的旧Preview |
| 01:12:22 | 关键帧bag正常关闭；抓取服务已返回失败 |

观察选路在53个已检查姿态中52个可达，选择roll0°/tilt0°，最大关节变化
0.525rad、硬件速度下界6.567秒；含控制参考桥接的最终轨迹11.140秒。
不能称全局时间最优，也不能把约50秒远场计算耗时算成机械臂运动速度。
连续名义指令路径净距认证8.795mm，作用范围是commanded Link6/7/8，
不是对未知伺服跟随误差的物理全包络保证。

## 软件直接原因及本轮源码修复

`PreparedPrediction.__post_init__`递归冻结原始远程诊断与输入证据，是正确的
请求隔离机制。两个新增接收分支却直接执行：

1. `_dedupe_exact_contact_sequences_for_moveit` 写
   `prepared.remote_diagnostics['contact_sequence_deduplication']`。direct近场
   进入这一分支即异常，甚至空去重列表也会执行此写入。
2. `_near_field_unreachable_surface_recovery` 在符合补拍条件时写
   `prepared.remote_diagnostics['near_field_recovery']`；修复第一处后该处也
   会成为潜在的同类故障。

旧测试用可变 `SimpleNamespace(remote_diagnostics={})`，没有覆盖实机真正
使用的冻结对象。日志未保存异常栈，但异常类型、近场分支必经写入、进入
顺序与无近场MoveIt调用相互吻合；本轮不能据此声称具体近场接近路径已验证。

修复保留不可变原始证据，把这两类本地工作记录写入独立的
`acceptance_diagnostics`，经本请求funnel深拷贝进入审计。未放松精确四阶段
去重键、目标身份、纳秒、开口、几何门或补拍次数/预算。

接收异常现在保存异常类型和有长度上限的栈，direct近场发布绑定当前ticket
的**无有效位姿** `NEAR_FIELD_ACCEPT_FAILED` terminal；task显式识别该码，
不再空等90秒。旧generation不能向新阶段发布terminal。没有自动重试、
没有补造有效Preview，没有停止/失能调用。

回归覆盖真实冻结请求的精确去重、完整direct接收调用、补拍诊断隔离、
funnel深拷贝、异常即时终结与过期generation不发布、task准确返回故障码。
完整测试和部署状态见本文末尾，不以源码通过宣称实机抓取已成功。

## 持续跟随残差：不是主机卡顿后的短暂延迟

采用冻结 `robot_description`（SHA256
`890592a24b8ad818d995e69beb670ea4254bf84f589615bfac3c8562f62449b7`），
使用 `tools/analyze_following_gap_bag_readonly.py`，参数为上述已关闭bag、
`parameters_before.yaml`、`--at 1789287138`，无ROS控制调用。

accepted最后变化在1789287044.744之前，随后至1789287142.445保留同一最终
位置，885个样本；SDK最后变化在1789287044.334之前，同一目标881个样本。
约97秒不收敛。末端平移差 **11.833428mm**，旋转差 **0.038496rad/2.206°**。
误差向量accepted−SDK为 `[+0.681396,+7.156155,-9.399743] mm`。

| 关节 | accepted−SDK，4096码/圈 | 对总位移误差的Shapley投影，mm |
| --- | ---: | ---: |
| Joint1 | -3 | +0.194 |
| Joint2 | -7 | +3.244 |
| Joint3 | -10 | +6.151 |
| Joint4 | -1 | -0.003 |
| Joint5 | -8 | +2.250 |
| Joint6 | +4 | -0.002 |

这是固定模型下的诊断分解，不是应该发送的补偿，也不证明模型已物理标定。
Joint2/3/5占主要贡献，支持继续查伺服跟随/死区/负载/驱动到固件的执行契约。
同模型复算本轮IK目标到编码后的SDK目标仅0.195897mm，六轴量化差均小于
0.5 count；实测到IK目标为11.791366mm。此轮不能将11.8mm归为SDK量化或
IK目标组帧错误，关键差异仍在已发目标与独立实测之间。
SDK已写目标不等于MCU执行ACK；现有位置型控制器0.035rad关节容差比该残差
宽，因而action成功不能代替接触前6mm/5°实际TCP门。

观察是无接触获取新图像，可在实际CAD净距及相机范围通过后继续重建；接触
仍须严格到位。这是为何本轮11.8mm没有终结观察，但也绝不代表该误差解决。
近场回传的中心修正约[-0.2,+3.5,-8.5]mm，不单独等于标定误差或支撑面
注册偏差；历史跨姿态约15mm问题仍须独立真值与配准验证，不更改TF去凑齐。

## 原始硬件证据：健康仍有未知项

从1789287031起到bag结束，原始CRC有效位置回包中26次E1、6次E2，
94组温度样本。第8通道在1789287118.2409039的原始字节：
`24 24 21 21 23 24 25 47 23 23`，raw最大71°C；该通道因突变被标记
`unknown_rejected`，filtered NaN。不能用其他通道最大37°C排除异常，
也不能在映射未确认时称“Joint5真实过热71°C”。

01:16:30–35纯订阅新温度样本10通道均接受、raw最高38°C，但仍出现E1。
短窗口恢复正常不等于异常已解决，状态字语义/10舵机映射须厂家确认。
本轮驱动在线版本查询已证实firmware_raw613/hardware_raw110，与截图6.1.3/
1.1.0一致，不再把未知固件版本当作泛化借口；仍没有这版本舵机PID、死区、
状态字或10通道映射的完整官方契约。

## 下一步，不偏离既定路线

1. 部署本次纯规划/任务异常修复后只保留待命，不把流程不再崩溃当作抓取验收。
2. 在下一次物理实验前，确认E1/E2与第8通道突变含义、对应舵机和实际温度，
   区分热/负载保护、遥测异常与正常状态。不能在未知告警下提高速度或加大补偿。
3. 健康状态有证据后，沿原显式有界跟随探测验证J2/3/5双向、多姿态响应。
   手册中的外环PID只说明一种实现方向；当前PositionJointInterface适配器
   不使用配置中的PID gains，单改YAML不会解决。实现如有必要必须有新鲜
   accepted/epoch、限幅限速、反积分饱和、无响应退出和连续参考；不移植
   手册示例中period无效时可能写零位置的分支，不全局开启无界积分。
4. 把实测跟随误差纳入生产观察路径净距余量，而不只检查名义指令曲线；
   再用同源图像、冻结TF和独立尺度/位姿真值分离相机标定与机械模型误差。
5. 以上通过后重新生成新目标/新计划，验证近场重建、完整接近、6mm/5°到位、
   Cartesian覆盖、双侧触觉闭合及抬升持物证明。没有这些实测结果不报成功。

## 验证与部署追记

专项5项通过；完整Python **2722 passed / 3 skipped / 7原有warnings，169.00s**。
报告：`grasp_attempt_20260913_aligned_xYlUCg/pytest_repair_full.xml`。
`git diff --check`通过。

01:25仅重载无执行中的remote/task：原PID6823/6831退出，新用户服务
`alicia-remote-repair-20260913.service` PID18063、
`alicia-task-repair-20260913.service` PID18069。
未重启driver6794/gateway6813/GUI6842及控制器；没有发停止/失能服务。
remote `auto_request=false`，新task IDLE/inactive，未再次调用/grasp/start。

源码SHA256：remote
`2fa9dcc9e8ca5757e603fa917ea7e2777faaa5edd5583fd0e48f61c761bb5c4f`；task
`84ecb659f3267643850f599b3e6ba7d4139a2012cf833d94715e04d9befc9c77`。
重载30秒纯订阅监测：542个motion_enabled均true；272个accepted与269个SDK
各只有一个固定值；未收到新joint_commands；epoch保持1789286419274460851。
这是无位置目标变化的部署证据，不是11.83mm跟随误差消失的证据。

保持ROS和纯订阅遥测服务在线。由于跟随误差与E1/E2/原始温度异常未查清，
不追加物理试抓或补偿。需要厂家/现场补齐告警语义、通道映射及温度证据后
再进入受限物理实验；后台录制不冒充助手回合结束后的持续人工判断。

## 01:49 优先级后续更新

操作者随后要求先解决前两项，不优先排温度。本轮继续完成固定目标有界
反馈校正实现和离线验证，不让硬件未知项阻塞软件工作；没有修改温度保护
或发新物理命令。近场修复仍已加载，校正实验源码未加载，物理收敛未验证。
01:48实时GUI直控true、姿态已改变，需再次交权后再进行物理实验。详情见
[固定目标校正专项](2026-09-13-fixed-goal-endpoint-correction.md)。
