# 2026-09-29 相机与真实反馈停顿

## 范围与当前状态

用户最后明确表示自行移动机械臂，此后“继续处理”未明确交回自动运动。当前只做软件修复和被动观测，无新的规划执行、夹爪或关节使能/失能命令。所有逻辑适用于可抓取未知物体，没有当前盒子或电机专用补偿。实验与日志直接写SanStand，local_only，不上传或删除历史。

先前最终精修连续性修复完成1094项回归并加载。generation17未运动，generation18到达预抓取并完成补偿，但均因数据停顿结束，尚未独立验证该修复的真实最终接触结果。gen18补偿位置残差12.8702→4.48016mm，补偿约7.1秒；没有闭爪或提升，不是成功抓取。

## 停顿证据

- gen17 accepted反馈缺口645ms与SDK等待2.5205s同窗；raw SDK诊断是在SDK队列取出后加时间戳，不能等同串口线上到达时间。raw→accepted仅约毫秒，没有拒绝过滤或CRC导致的丢帧证据。
- gen18相机等待2.2251s及1.5775s，accepted缺口约842.6ms；另有消息发布到订阅回调约650ms的积压。把额外图像诊断录制从5Hz降为1Hz仍复现，不能据此把移动硬盘认定为根因。
- 14:27:15–14:30:15 UTC的180秒修复前基线：RGB最大host stamp缺口958.735ms、SDK wait929.738ms，accepted292.907ms。相机候选原生读线程在最大缺口采样均为S/do_select，新增runqueue等待0；不支持这些线程持续处于可运行队列却抢不到CPU。
- 慢等待括区有换出4237页、direct扫描570页、内存PSI full367.101ms；主缺页仅3、换入0、直接压缩停顿0。稍后1098次主缺页和902页换入时未出现≥250ms源缺口。因此仅能确认回收/换出压力同窗，不能认定“主缺页风暴导致相机断帧”。
- USB4-1所有358次采样active，suspended计数增量0；未支持runtime autosuspend。宿主USB/虚拟化延迟、video/metadata交付尚待区分。
- 用户新截图显示Windows主机29.5/31.7GB(93%)占用；这是后续当前快照，不是上述缺口的同步Windows采样。随后用户通过WSL调用Windows PowerShell取得vmware-vmx工作集15.63GB、vmmemWSL仅0.69GB；其余前8进程各≤0.53GB。没有.wslconfig内容输出，但SilentlyContinue不能区分文件缺失、空文件与读取失败。WSL实际配置上限尚未确认，不能把微软文档50%默认上限当作已占用。前8工作集不是全部物理内存账目，不能解释截图的全部93%。

基线与详细时间相关记录：`ros_log/passive_camera_feedback_20260929T142715Z/{summary.json,baseline_findings.md,baseline_findings.json,correlation_detail.json}`。SDK审计：`ros_log/camera_sdk_audit_20260929T143000Z`。记录目录时间标签不是实际事件时间，以文件内容时间戳为准。

## 已加载的两项修复

1. 感知配置缓存：保持参数变化、嵌套profile、force与reload generation失效逻辑；配置不变时不再重复查ROS包及模型路径。即使/perception/enabled=false，旧逻辑仍反复扫描路径。57项测试通过；离线24轮路径查找由24降为0，不将离线耗时称作真实系统加速。新PID645632；5秒单核CPU占用55.94%→18.38%，仅为短窗口证据。旧节点收到正常退出请求后超过首轮10秒等待，随后自然退出；未强杀或重启复合服务，原失败记录保留load_attempt1.json。
2. RGB-D配对：align前读取原始色/深timestamp、域与实际profile FPS；要求同一hardware_clock/global_time域，时间差不超过较慢流半周期（30fps时16.67ms）。拒绝缺失、混合/不可信时钟或过大差值，不发布、不伪造共同stamp、不将坏pair计为相机读异常/重启。147项测试通过，含真实436.794ms错配回放。ROS时间戳仍是host发布时间，不宣称校准了曝光时钟，也不能排除两帧共同陈旧。

只重载单独感知和camera软件节点。相机人为重载区间1790692518.9526315–1790692527.740253，不能统计为自然断帧；新相机PID645959。驱动PID8750、硬件接口8751、网关、任务、规划、TF与根ROS进程身份/控制epoch保持，全部/camera参数读回无差异。真实33.03ms错配已被拒绝，随后合法RGB-D继续发布。加载本身不发机械臂运动。

证据：`ros_log/idle_detector_repair_20260929T142135Z/{validation.json,load_result.json}`，`ros_log/rgbd_pair_time_repair_20260929T142631Z/{review.md,source_manifest.json,run2.log,load_result.json}`。

## 尚未完成

修复后180秒被动对照已结束。根因尚未最终确认，不能宣称秒级断帧已解决或当前目标已抓取。当前证据不支持WSL是主要常驻内存占用者；虚拟机占用是主要已知项，但不能据此归因全部断帧。恢复自动任务前需要用户明确结束手动调整，重新识别并选择当前目标，禁止复用gen18旧计划。

## 修复后被动对照结果与订阅覆盖限制

观测为14:37:19.692–14:40:19.692 UTC，目录`ros_log/passive_camera_feedback_20260929T143719Z`。相机/driver的PID及start_ticks全程不变，无运动命令。感知节点稍后5秒单核CPU为5.60%（初载18.38%），原轮55.94%；都是有限窗口，不能归因全部系统性能。

记录的3544对已发布RGB-D最大host stamp间隔102.244ms，首张有效图像却到14:37:21.429才收到，不能用这个最大值覆盖订阅启动前的缺帧。现有相机日志明确显示：14:37:15.460→20.462等待5.0018766秒并报“Frame didn't arrive within5000”；随后20.466→21.418又等待0.9513075秒，返回RGB/depth时间差6.204032秒的坏pair，已被新策略拒绝。后约178秒的平稳发布不构成秒级停顿已解决的证据。

accepted最大源间隔589.537ms（raw joint589.254ms）发生20.777→21.367；同窗尚无已收到相机样本。不能称它为“相机正常时仅驱动出问题”。raw20.7772326→accepted20.7772534约20.7微秒，但accepted到达观察者迟161ms；这是accepted时间戳后的发布/运输/订阅延迟，不能解释成raw到accepted计算耗时160ms。新观测仍不排除参数RPC/驱动锁等软件问题，但没有将该次缺口归因某一调用。

修复前最大RGB缺口的两个native读线程timeslices分别+5/+3，说明50ms采样间确实唤醒过；即使采样均为do_select也不是连续同一次select。SDK select超时固定5秒，原0.96秒缺口不能叫select timeout。生产库编译关闭详细V4L日志，当前没有热开精确DQBUF/metadata序号诊断的入口；当前账号也无kernel trace/USBmon读取权限。现阶段无法从已有数据唯一判定宿主USB延迟、V4L2 metadata/video配对等待或资源回收影响。

用户随后提供WSL `free -h`：total15Gi、used2.7Gi、available12Gi、swap4.0Gi已用0B。WSL当前可见内存约16GiB、余量充足；该Linux口径与Windows工作集0.69GB不相同，也不是同步采样。无需因这份数据提高WSL内存上限；未擅自修改WSL/VMware配置、关闭Windows进程、重启虚拟机或驱动。

结论：配置扫描开销与跨时刻RGB-D错误配对两项软件修复已验证并加载；相机/真实反馈底层停顿仍未解决。下一步需针对已观测时窗的设备交付和发布链路收集更直接证据或做受控资源对照，不能放宽freshness门限、使用旧图像、强行继续抓取来隐藏问题。

最终对照审计：`ros_log/passive_camera_feedback_20260929T143719Z/post_observation_boundary_audit.md`及JSON；相机journal另存`camera_journal_1790692637_2643.jsonl`。1619对raw/accepted源时间差最大1.750ms；首个有效图像后178.205秒内RGB最大102.244ms、accepted最大210.056ms，两者均无≥250ms缺口。这只描述该段，不证明启动边界5秒SDK停顿消失。全部被动记录结束，无后台抓取执行。

## 驱动热路径的待修复项（仅完成只读评审）

三处`gripper_stroke_m`读取使用`pnh_.param<double>`，分别处于命令模式锁、SDK发送/模式锁、反馈topic/data锁范围。已安装roscpp1.17.4的NodeHandle模板在参数存在时执行hasParam+getParam两次同步RPC；50Hz heartbeat单项即可调用100次RPC/秒。该结构会延长关键锁占用，但尚无逐次RPC耗时证据把实际589ms缺口归因到它。

不能简单在持锁路径替换getParamCached并称无阻塞：首次订阅失败、cache invalidation或本节点删除仍可能同步访问master。独立低频参数刷新线程/queue + 原子数值快照可隔离热路径；须保留参数动态修改、缺失/删除/错误类型回退0.05的语义，并验证阻塞RPC时的线程退出/对象销毁。此项尚未实现、构建或加载，本轮driver源码及进程未因该评审改变。未把推测性设计当作已完成修复。

后续离线驱动修复及有界追踪工具已保存、测试通过，按用户要求未做真实RGB-D/实机验证。见[离线收尾](2026-09-29-stall-offline-closeout.md)。底层卡顿仍未标记解决。
