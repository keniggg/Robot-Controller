# 2026-09-11 自动命令参考连续性修复（离线完成，未加载）

> 2026-09-12部署补记：操作者明确恢复并要求启动完整ROS、正向使能后自行对准。
> 本文“未加载/暂停”为9月11日收尾状态；9月12日已从同一worktree完整启动新
> driver/gateway等节点，运行driver SHA与本文最终构建一致。尚未执行新抓取，
> 等待本轮明确对准及交权；不把节点上线记为实机修复验收。
> 具体启动/使能/监督记录见[主运行日志](../../../src/alicia_flexible_grasp_supervisor/logs/2026-07-23-ros-latest-node-launch.md)。

## 当前授权与状态

操作者确认双手离开，并提出“并退回原安全位”，随后两次明确要求：
“修复完不启动6D抓取，直接暂停”。以此为最新指令：本轮仅修改源码、离线
构建/测试和更新文档，不启动抓取、不重载实机节点，不发送停止或失能命令。
修改位于 `.worktrees/protocol-v3-upgrade`，没有编辑IDE中的根工作树旧副本。

此前实机失败的源证据与完整时序见
[Joint2/跨姿态专项记录](2026-09-11-joint2-cross-pose-repair.md)。
到位静置约14.9秒136个accepted帧均不漂移；补拍前重发actual保持目标，
Joint2/3/5再次位移，tool0下降8.585mm，模型净距5.964→−1.309mm。
本轮修复这一已证实的参考重基准问题，不声称已解决实体跟随残差或历史
跨姿态约15mm的全部来源。

## 本轮修改

### 驱动：有成功命令时交权保持原位置计数

- 有有效且新鲜的成功SDK冻结帧时，手动→自动首命令须与该帧六臂及夹爪
  所有位置word一致；任一word改变均拒绝。新鲜完整实测反馈仍必需，但不能
  用actual替代命令基线来制造第二次实体运动。
- 非空但损坏/过期/未来的冻结帧直接拒绝，不回退actual来复活旧参考。真正
  清空的初始正向enable周期保留原fresh actual初始化，不改原0.003rad门限。
- 新增非latched只读 `/alicia_d/control_reference`，仅在成功SDK write后发布，
  stamp与原 `/alicia_d/sdk_command` 相同。包含六臂和right_finger（米），frame
  区分 sdk_manual / sdk_handoff_required / sdk_tracking / sdk_reference_unconfirmed。
  tracking仅证明软件已接纳命令参考，不是电机到位或硬件响应ACK。
- 不修改SDK串口协议、关节编号、速度参数或多关节直控；夹爪米换算来自同一
  成功帧raw/1000×既有stroke参数。
- 专用 `motion_gateway_reference_sync` 仅接受完整六臂且不带夹爪字段。接收
  时在mode锁内再验真实reset/ownership周期、fresh反馈及成功帧全部位置字；
  已tracking则不改在途目标/trim，旧排队请求不能变成另一条运动目标。
- 新增latched只读 `/alicia_d/control_reference_epoch`（Header，frame为
  sdk_reference_epoch），只在真实clear后发布。纳秒递增屏障区分重置与重复
  PENDING心跳；它不是硬件采样时间。时钟倒退产生未来屏障时撤销旧参考且
  拒绝新准入，不能用时钟异常复活旧命令。

### 网关：连续自动阶段零保持重发

- 同戳配对两个成功发送遥测topic，拒绝旧包倒序、错误frame、坏关节名、
  不同word、过期/未来帧；跨topic回调短有界等候不选旧匹配包代替新命令。
- 普通自动阶段要求新鲜SDK、accepted反馈和控制器desired一致（SDK同word），
  且存在本gateway已完成命令的证据；直接保留真正controller desired，不
  发布actual hold、不以“只有一次运动许可”的闩锁破坏连续任务。
- 完成回调可能先于下一controller帧；仅当其他证据全部相符、唯一缺口为
  帧戳早于完成时刻时，短有界只读等新帧，再完整重验。q/v/a及header来自
  同一深拷贝，禁止旧desired借用较新live header；完成时刻不会改写。
- 真正手动交权时，仅driver明确处于sdk_handoff_required才允许将ros_control
  同步到已成功发送的冻结SDK基线；driver会拒绝中间不同word，直到完全一致。
  保留arm-only硬件流不覆盖夹爪；不支持的gripper随流模式明确拒绝。
- 同步成功须收到本次publish之后的tracking同word回执，并满足原稳定窗口。
  新控制器帧/ACK未到时只等原deadline，不在20ms桥期结束立即误拒旧帧。
  初始/交权同值命令被HW的publish_only_on_change省略时，最多一次专用六轴
  同参考准入，仍等它之后的成功写入回执；不微动、不补夹爪、不重采actual。
- 每个新driver reset epoch撤销SDK/完成证据；重复PENDING只是状态心跳，不
  重新计时。等待前后都复核manual/epoch/SDK字/反馈，变化即撤销。普通自动
  接续不走这条一次准入发送路径。
- 闭爪/开爪携带的臂参考同样改为成功SDK基线，不再在0.5秒超时后重采actual
  引起臂二次阶跃。夹爪目标本身仍按原任务控制。

### 规划器：真实接续段速度与完成证明

- strict、Cartesian和独立prefix均在最终执行前检查Noetic实际采用的
  stationary desired→首个正时间点接续段及后续全部linear/cubic/quintic段。
- Bernstein包络证明连续每轴速度，保留既有0.08/0.02等上限，不只查waypoint
  速度。只有完整结构化超速证据允许复用原统一t/v/a伸时，q及几何路径不改；
  随后重新速度证明，最终观察CAD和轨迹hash也使用真正提交的伸时结果。
- 缺证据、非有限值、计算超时等不能借重定时绕过。CAD后执行前再核fresh
  reference、limits与trajectory hash；提交前拒绝不发额外hold、enable或stop。
- 仅真实execute(wait=True)返回成功，才通过callback记录最终轨迹深拷贝的
  完成证据；单帧v=a=0不证明轨迹完成。控制器失败但旧hardware-tolerance
  兜底成功不产生新完成证明。callback失败明确属于“已执行但证明记录失败”。

## 验证与构建

所有测试使用隔离 `ROS_MASTER_URI=http://127.0.0.1:11319`。
本轮证据目录（根工作树）为
`.ros_log/reference_continuity_repair_20260911_W1V558/`。

- 驱动限定源码合同与遥测RED复现转GREEN；新增C++基线helper先编译RED后
  GREEN，含4096位置码循环、单轴±1word拒绝、夹爪码及无效/过期/清空周期。
- 网关最终101项定向回归通过，包括本轮真实残差不再触发额外publish、多自动
  阶段/多轴允许、旧包、缺完成证据、手动交权、初始化及夹爪参考。
- 最终冻结版全量 **2571 passed /3 skipped /7 warnings，113.40秒**，见
  `pytest_full_verified.xml/log`。前两轮2521与2560通过报告保留，但不替代
  最终源码验证；warnings均为既有rospy弃用warn提示。
- `catkin_make --pkg alicia_d_driver -j2` 构建driver；C++测试必须显式执行
  `cmake --build build --target actuation_confirmation_test -- -j1`。早期
  `driver_cpp_build.log`误把测试target当package，未更新test binary，原
  `driver_cpp.log`仅81例旧结果；该记录保留，但不能作为最终驱动验证。
  最终重新构建/执行结果见 `driver_cpp_verified.xml/log`。
- 冻结版C++ **91 passed，195ms**，确认执行的是本worktree `devel/lib/`
  下显式重新构建的测试程序，非先前81例旧binary。
- 各独立审查检查SDK量化映射、接续段插点、无动作拒绝、交权ACK与完成证据。

源文件SHA256（非运行版本声明）：

- driver cpp：`1e5549f684a7d66eae7c523aa905b32c92c0e6b4628a4650a3d32221294a5c2b`
- gui_direct_hold.hpp：`5f0e9d02b0920deb2fecb1e008c7ff340181cffc803c40521a38ec5bbf368811`
- gateway：`7c45399ca67f8d3e64156017564f8fb73d1991e4b3122c2f9a279d5796c57999`
- planner：`06abf9b74f9609662afa9b54b2a6bdecc05cf9bfc84e1c376ef2c12de389dadd`
- guard：`eb96bf07d0353eacd0c339a0f1886464a59b3ff1ef8036434e0c8d1fccf6758a`
- 新driver二进制：`30edad32f5c2384cfc2928550eca8aaef5c2d6cf87b2ee93848ecb73b934c940`

## 未部署及后续边界

当前driver PID89835仍映射旧二进制SHA256
`d03d82c27a9ff6908783abc3e65bef2b96993a7294ef601849151cdb4f9dd5a4`，
不是磁盘上新构建文件。gateway106028、task102630、GUI89144、remote71319
均保持原启动时间；本轮没有重载、没有新鲜抓取runner、没有抓取运动。
原运行节点仍不包含本次同步修复，后续应成套加载匹配driver/gateway/planner，
不能把仅编译磁盘文件记为实机修好。

本轮源码、构建、最终回归及文档已收尾，现在按操作者指令暂停。不自动
发起后续动作，不把“暂停工作”实现为对机械臂发送停止/失能命令。

待操作者明确恢复工作后，才安排相应加载和实机验证。应先做只读规划预热
避免MoveIt冷启动消耗候选检查预算，再绑定新鲜场景；不重放旧计划或改TF。
仍须分别验证Joint2正向微调、真实跟随、到位CAD、远近4°/4mm、双侧接触、
闭合/抬升持物终态。新连续证明是控制器名义速度/夹爪几何，不是SDK固件
瞬时微步速度、实体误差界或新增全臂环境完备证明。
