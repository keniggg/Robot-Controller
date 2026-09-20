# 2026-09-11 Joint2 小步响应与跨姿态几何失配

## 授权、部署与验收边界

用户已结束手动操作、交回控制，明确要求继续处理两个瓶颈直到真实抓取成功。
运行/修改工作树仍为 `.worktrees/protocol-v3-upgrade`。不发布停止/失能、
不更改 TF/TCP 或固定目标、不放宽支撑面 4°/4 mm、接触实测 6 mm/5°、
Cartesian .98、IK 重复性 1e-6 rad 或真实 CAD 门限。已有 centering override
保留但不冒充消除了标定不确定度。离线通过不等于部署，更不等于持物成功。

## Joint2：原始无响应与独立数值缺陷要分开

18:53 失败窗口，Joint2 上游参考约 -.884079509 rad，按当时 codec 重建 SDK
目标 1471→1475（+4 count / +.3515625°）；编码器保持 1452，至少 6 s 未动。
同次修正中 J1/J3/J4/J6 分别响应 -2/+4/+6/-3 count。旧驱动只有限流串口
日志，这些是上游、codec、日志联合重建，**不是新增诊断话题的原始逐帧记录**。
这排除“微调没有形成可量化目标”；不能据此唯一断言某个机械部件损坏。

Joint2 前一运动方向为负、微调为正，换向死区/回差、负载下稳态伺服偏差是
主要待验证假设。没有 1 s 后迟到响应，故暂不支持盲目延长超时。对应窗口状态
为 0x00，E1 在超时后约 2.29 s 才出现；温度样本已旧，不能声称故障时温度
正常。现有 CONFIRMED 可能来自先前大运动，不能证明本次小步有响应。
本地官方 SDK 的通道顺序、CRC、符号和计数单位与驱动一致，未找到 Joint2
独有的索引/方向错误。

当前 URDF 的独立 FK 进一步闭环到原 bag：actual 的 tool0 为
[-.138495042826,-.411206618679,.111481697656] m，与记录 TF 差约 2.03e-8 mm；
真实绑定 pregrasp 为 [-.140534449153,-.415464887683,.129135046065] m，
残差 **18.273827 mm /2.679946°**，与日志 18.3 mm/2.68° 相符。
仅把 Joint2 实测换成 desired，末端位移向量为
(-.563002,-1.536253,+13.083205) mm，模 **13.185116 mm**；仅 Joint3 为
(-1.109108,-3.026402,+4.443232) mm，模 **5.489221 mm**。同时纠正两轴，
相对 desired 的 FK 只剩 **.503827 mm**。这是运动学反事实，不是实机已纠正，
且矢量模长不能直接相加。desired FK 与实际请求本身有 .085216 mm/.076262°
IK 残差，解释完全按关节目标计算的 18.226994 mm 与任务日志细小差别。
这使 Joint2/3 真实跟随成为预抓取失败的定量主要因素，不能靠视觉门限掩盖。

独立发现：从双精度弧度相减得到的恰好 2 count，可能比独立计算的 2 count
门限低几个浮点舍入单位，误留在 WAITING_RESPONSE。已仅为该比较加入按
运算数尺度计算的 4×double epsilon（硬件范围 <3e-15 rad）；方向仍严格 >0，
实际 2 count 最小响应、4 count 最大步长、稳定时间、1 s 超时均不变。
新增四项回归先 RED（1 失败/3 通过），修复后全量 C++ **81/81 通过**：
8188 个正反 2 count 接受、8190 个正反 1 count 拒绝，零/反向响应拒绝，
低于门限 1e-8 count 的可分辨差异也拒绝。报告：
`build/test_results/alicia_d_driver/endpoint_response_roundoff.xml`。
这项修复**不能解释或掩盖原窗口真实零响应**。

### 速度含义与尚未完成的实机验证

配置 .02 rad/s 是 J3/J4/J6 历史实测持续跟随上界，用于 strict 轨迹逐段
重定时；Joint2 继承 .08 rad/s。它不是已验证的固件瞬时微步速度上限。
驱动 SDK 15°/s 字段量化成 150 count/s（.230097 rad/s），官方协议最低
合法值 50 count/s（.076699 rad/s）；端点微调发送有界阶跃，绕过普通插值。
不能声称 4 count 幅度天然保证瞬时速度≤.02，也不能写未经支持的更低协议
值。当前未更改这些速度参数。后续响应必须结合实际成功 SDK 帧和编码器，
保持未编辑轴的最后 SDK 目标，不把实测值当成原伺服目标反复回填。

## 约 15 mm：独立背景纹理也发生跨姿态漂移

证据源为旧尝试 `grasp_attempt_20260911_path_compared_J74Eov/keyframes.bag`，
远场 stamp 1789179260566533088，到达 stamp 1789179345381506919。
两源戳各 ±1 s 内均有 118 条 joint_states、六轴跨度均 0。原反馈+当前 URDF
独立 FK 与记录 TF 平移差约 1.8e-13 mm，不是使用控制目标或关节顺序错误。
光学轴转换只有一次，标定 YAML/静态 TF 一致，平面比较使用目标局部共同
锚点而非直接相减 d。tool0 定义修改早于标定，未见之后 TCP 变更。

背景 RGB 特征 189 组匹配、131 个平面单应内点，其中 90 组有效 RGBD 对应，
现 TF 下残差中位 **18.072 mm**、P90 **22.849 mm**，base 中位偏移
**(+10.318,+2.388,-12.489) mm**。纯 RGB 单应可行旋转与 TF 差约 **3.694°**。
因此不能仅归因于盒子可见面/OBB 中心偏移或仅深度尺度；跨姿态外参/运动学
相容性仍需实机量测，尚未唯一定位到手眼、支架还是关节机械误差。

现场只读查询 D405（260522277974）出厂 color BGR8 640×480@30 profile：
fx/fy=438.60040283203125/438.4169616699219，cx/cy=
317.7792663574219/245.35714721679688，与运行参数**完全一致**。深度已对齐
到 color；不能误用原生 depth 的 389.4534 焦距。未开启第二条相机 pipeline。
畸变修正的离线 RGB 旋转诊断仍约 3.77°，非简单中心像素畸变解释。

27 组手眼样本最大两两旋转跨度 62.55°。远场距最近样本 7.88°，但旧 75°roll
观察后距任一样本至少 **56.42°**、位置至少 78.20 mm，确有明显外推；原交叉
验证通过不能证明这个任务姿态受覆盖。不能用人工 15 mm 偏移抵消此差异。

## 为什么旧观察规划选择 75°，以及有界候选扩展

冻结 request16 的 70 个 roll×距离端点，22 个通过原 CAD 门限，与 audit
一致。roll0 在 18–22 cm 都失败，22 cm 仍为 -4.258 mm 支撑面净距；
roll15°/22 cm 只有 2.114 mm <3 mm。负 roll 的严格 IK/路径请求全部不可达。
75°/21 cm 的硬件时长下界 19.479 s 确实最短；75°/22 cm 为 20.298 s，
90°/20 cm 为 22.633 s，180°至少 37.491 s。因此已有时长排序并未选错，
问题在于候选族固定光轴、只允许 roll，漏掉小幅倾转的观察端点。

拟在原族之外增加有界的支撑法向倾转：
`axis = normalize(cross(current_optical_axis, -measured_support_normal))`，
绕此 base 轴最短倾转，重新求目标居中及 tool0 位置。不使用物体 yaw，
不改目标、TF、18–22 cm、CAD/侧面信息或严格 IK/路径要求。

旧几何的离线端点结果（**尚无 IK/轨迹授权，不能直接执行**）：

| 倾转 / 距离 / roll | 最低 CAD 净距 | 侧面信息 | 距最近标定姿态 |
| --- | --- | --- | --- |
| 5° /22 cm /0° | 8.442 mm |14.295 mm |7.147° |
| 10° /20 cm /0° |4.878 mm |12.782 mm |6.571° |
| 15° /20 cm /0° |16.570 mm |11.171 mm |6.046° |

侧面信息不确定度仍 3 mm。候选扩展及回归正在实现；后续仍由已有严格检查
和硬件时长排序选择，不宣称全局最优，近场共享 90 s 与补拍预留不变。

## 本轮现场记录与未完成项

纯订阅关键帧记录 `grasp_joint_geometry_20260911_2UuqFC/keyframes.bag` 已
正常关闭，727 个同源关键帧；本段仅记录静止对准状态，无新运动或抓取。
输入/实测/成功 SDK 帧的持续分卷 recorder 仍在运行。当前控制器 desired
为历史任务终点，与实测不同；后续首次自动执行必须使用已有的原子实测同步
和有界观察前缀确认，不能直接调用未经交接的关节运动接口。

待追加：新二进制加载、候选扩展完整测试及加载、真实 Joint2 小步响应、
新跨姿态几何误差、完整接近/闭爪/抬升/视觉持物终态。当前不宣称成功。

### 20:18:58 响应边界修复已加载

隔离 ROS_MASTER_URI 的 `catkin_make --pkg alicia_d_driver -j4` 成功。
在 GUI false、task FAILED/inactive 的交回状态下，只 restart 原 driver service，
新 PID **56172**；`/proc/56172/exe` 与构建产物 SHA256 同为
`760ac06f909d19127e9ce344bb4e748e4e0103750416896dddec863c0d1fb934`。
header SHA256 `43a2949fbe5ddf138fd9a4766481b7d50936dc2a7252ba086464788deb0957f5`。
无参数变化，仍只启动正向使能，无 controller switch、GUI 重启或停止/失能。
新状态 PENDING:POSITIVE_ENABLE_REQUESTED，六轴实测与重载前完全相同：
[-1.9113400617,.5645049299,-.2699806187,-.0076699039,-.2285631374,-.0015339808]。
尚未发新运动，PENDING 不是失能命令，也不能伪造 CONFIRMED。

只读 keyframe recorder 增补 joint_commands、成功 SDK command、控制器状态、
actuation、GUI 模式和 audit 引用，使后续试验输入/输出/反馈保留在同一个
不滚动 bag；无发布器或控制接口。Python 编译及 diff 检查通过。

补充 driver/GUI Python 回归 **54 passed /1.21 s**，报告为上述本轮诊断目录
`driver_python_regression.xml`。新增 `watch_grasp_attempt_readonly.py` 是有界
纯订阅终端视图，区分输入/SDK/反馈/消息接收年龄，初始旧 FAILED 不提前结束，
不调用任何控制接口；它也明确心跳消息数不等于独立硬件采样数。

### 20:27:55 倾转候选加载，20:28 开始新完整尝试

remote 模块 389 passed（新增 27 例），完整 supervisor **2292 passed /3 skipped /
7 原有警告，86.16 s**，报告 `grasp_joint_geometry_20260911_2UuqFC/pytest_full_tilt.xml`。
独立终审确认精确计划绑定兼容；新增 tilt 超时不截断原 roll、缓存命中重新
检查侧面信息、名义距离先跨倾角交错、6 call/3 s 共享预算均有回归。
预算只限制开始下一次额外检查，不硬中断在途服务；进入源龄 reserve 后不
启动新倾转检查，原生调用超时与最终源授权仍生效。

只 restart `alicia-remote-path-20260911.service`，新 PID **60688**。
remote SHA256 `2c981fb34cfbc5093475ca5945dd15177f6721cae8412a003fb20ee05536bba3`，
config SHA256 `4c5b17527b0937e8be62f3b96bbf5e3339c83c4ff82bd5e04afa0d36e12bde55`。
服务名称/原版本环境标签沿用既有部署，实际版本以本次源码哈希与启动时间为准。
参数 before/after diff 仅新增 observation_camera_tilt_offsets_deg=[5,10,15]，
没有加载整份 YAML、改其他运行门限或重启 driver/controller/GUI。

新尝试目录 `.ros_log/grasp_attempt_20260911_support_tilt_KewwBe/`，纯订阅
watch/keyframes 先启动，随后原 FreshPreviewRunner 开始；新鲜窗口起点
**1789183722248704910**，源有效期仍 120 s，首个新鲜预览等待上限 300 s。
runner 仍要求已提交哈希审计、同一观察姿态和 exact plan_id，未直接下发裸
关节目标。后续结果追加，当前不能把推理开始当运动或持物成功。

### 20:31:41 本轮失败，以及新预算接线错误的纠正

far request27 / plan `a7382a1b28ff3b5d92658a58`，源戳1789183761841986656，
32.92 s 后收到新鲜预览；审计保存为 `far_audit_snapshot.json`，SHA256
`177b2d5bd01188196a8d2fa5686a820332cd0fee9511104fbdb12afd591875b6` 与
已提交引用一致。实际仍选 roll75°/tilt0°/21 cm，35个端点中25查、13可达。

**此次新增预算存在接线 bug，不能把所有测试通过写成无问题：**10个新增
tilt 都通过 CAD，却全部因 OBSERVATION_TILT_SNAPSHOT_RESERVE_REACHED
未做 IK。helper 错用会被 WSL health 覆盖的 MuJoCo snapshot 寿命，现场
只读 health 实值 **2 s**；减去既有 reserve30 s，任何新鲜源都会被跳过。
远场观察本不执行接触仿真，应使用原远场 plan validity120 s 的源授权。
先前389/2292测试及独立审查未覆盖此生产参数组合，已明确补回归，未将
原因伪装成 IK 不可达。限定修复使用已有 `_configured_execution_plan_validity_sec()`，
**不改任何时限值**；server2 s、近场90 s与其他门限不变。
修复后模块393 passed（新增server2/15/30/120与本地120/reserve30组合），
remote新哈希 `da105bd74397bbbe2037776f425b7032c466c5bed878dcd85e9abf2a1fd6ef3e`，
此刻尚未部署第二版。

20:29:54.989 bounded prefix开始，57.903真实方向响应CONFIRMED。J2当前构型
负向4 tick已响应；不等于原正向trim验证。观察本体55.459 s，于20:30:54.903
执行完成、55.817静定，随后camera约203 mm进入近场。静定后SDK−实测偏差
J2=9 tick/.7910°，J3=11 tick/.9668°，两轴整个观察均跟随，尚有稳态误差。

新独立路径问题：候选 strict 检查 maxdelta1.878 rad、LB23.474 s（J4限制），
执行前重规划改成 maxdelta2.773 rad（J6限制），实际重定时55.459 s。
LB与执行时间不能直接当同一指标；但起点变化仅约.016874 rad，不可能解释
maxdelta增加.895 rad，证明终点关节解/IK分支已变化。当前候选只绑定末端
Pose，gateway单槽缓存被其余候选覆盖，prefix/preflight/atomic又多次无seed
pose规划，丢失优选分支。正在修复为观察专用、有界、精确Pose→已检查q_goal
提示；仍从新鲜实测起点重新碰撞规划/FK复核/重定时，绝不复用旧轨迹授权。

20:31:41.118 FINAL_REFINE_3D_INVALID、41.121 inactive：远近法向差
**4.340014°**，共同局部锚点面距 **14.697758 mm**，OBB中心差19.7091 mm。
到达参考→near局部仍 VALID_3D，605 inliers、RMSE .499706 mm、最大修正
.115636 mm。没有contact pregrasp、端点trim、接近、闭爪、lift或持物验证。
关键帧bag正常关闭181帧。后续不能在当前75°姿态直接重置远场参考掩盖漂移；
需真实恢复原对准/标定覆盖姿态，再验证真正参与严格规划的小倾转。

### 20:49 第二轮加载与无运动生产接口验证

预算修复+观察分支 hint 全量 **2314 passed /3 skipped /7 原有警告，90.45 s**，
报告 `grasp_joint_geometry_20260911_2UuqFC/pytest_full_budget_branch.xml`。
hint 有18项新增回归和独立真实ROS消息/乱序关节/夹爪保留序列化离线复核，
仅 observation planner 启用，128项/120 s不续期、exact Pose与URDF/SRDF/group/
frame/tool签名；fresh start重新规划固定q_goal，不复用旧轨迹授权，失败不
在同次调用随机换解。FK使用生产 .0001 m位置、intrinsic XYZ各轴 .001 rad
与原 .02 rad缓存角度的各自语义，不把三轴容差误当相同SO(3)总角上限。

20:49:07 remote PID71319，已加载预算修复 da105bd…；20:49:08 gateway通过
原launch respawn恢复为PID71381（仅准确退出原 `/motion_gateway`），加载
planner `90ba13cf8c376194e9dd97972902b798fc5c027eb756df3082f6b5cd74aff875`、
gateway `08cbe4512240450e974e77259dad4ef2ee97f7bcfccd2f6cd7deceb6c58dedbd`。
driver/controller/GUI/task不重启，不发停止/失能。此次加载后没有启动抓取。

现场只读 compute_fk 对照：空frame与base_link均SUCCESS、结果base_link一致；
显式frame是契约加固，不应写成空frame在本机已失败。随后对同一个当前FK
Pose连续两次 `/supervisor/check_observation_pose_strict(execute=False)`：
第一次capture成功，goal hash f51401b48595185790b91620929be19fae127f25c0a3c31d0de813f7029770c9；
第二次报 FK_TARGET_MISMATCH，位置 .000113091 m、方向 .001145451 rad。
日志尚未区分seed/terminal FK，因此先不武断断言具体失败位置；排查重点为
joint-goal规划仍用原非零关节容差，再次采样改变终点。正在限定固定分支规划
目标为精确q_goal并加生产边界回归，不放宽原FK/位姿门限，也不事后改未经
碰撞检查的轨迹。两次服务均不执行，SDK保持目标未变。

### 实际观察净距与恢复限制（模型估算，不冒充实体碰撞）

同一URDF下请求→到达FK残差为(+.230,+4.891,-9.837) mm、10.988 mm/1.729°，
沿原固定支撑面下沉9.047 mm。右指净距9.817→2.031 mm，其中平移−9.047 mm，
旋转反而+1.261 mm；这是实际跟随误差引起的净距损失，与跨姿态15 mm感知
失配不同。继续静定后的最新q模型净距 **1.595 mm**（左指49.717、掌部30.321 mm），
没有模型OBB相交，但不满足3 mm支撑面门限，不能称为已确认真实碰撞。

恢复原对准q的2001点直线FK/CAD扫查（非MoveIt授权）：旧起点前约1%仍不足
3 mm，原对准终点净距71.258 mm。没有执行这条恢复路径或增加逃逸豁免。
原对准tool0基座位姿xyz/xyzw为
[-.0792844258,-.2244917368,.1460659224]/
[.7459678859,.5225052070,-.3357112425,.2404541194]，
camera_link原点[-.0615462610,-.1860163489,.2937953572]，原目标距离约313 mm。
需由操作者通过GUI退回原对准范围并交回控制，不能在75°观察位重建参考
来制造“跨姿态已改善”的假象。

另确认执行合同缺口：`grasp_task_node.py`远场endpoint检查 required=False，
只记录FK残差，主要用180–220 mm相机距离验收；**不存在已强制的独立15 mm
观察笛卡尔误差上限**。现CAD仅校验nominal端点3 mm，0.6 mm模型padding
不是跟随误差或手眼余量。本次失败已证明nominal合格不保证实测净距合格。
下一项必要工作是明确观察跟随误差合同、为规划留有依据的余量，到位后对
同冻结支撑面/OBB重验实际CAD，并验证完整路径；不能用相机距离替代。
新倾转有更大nominal净距（本轮几何15°/20 cm为22.276 mm，15°/22 cm为
39.951 mm），但本次实测误差不是未来上界，尚不能据此保证所有执行安全。

### 20:57–21 时：精确关节目标修复验证与用户确认重新上电

固定分支规划期间临时设置 joint-goal sampling tolerance=0，finally恢复原值；
终点与既有q_goal仅允许1e-12 rad浮点差，不事后修改轨迹，原Pose/FK门限不变。
新增失败/恢复异常清理及近边界回归，相关143 passed，branch23项；全量
**2319 passed /3 skipped /7既有警告，78.27 s**，报告
`grasp_joint_geometry_20260911_2UuqFC/pytest_full_exact_branch.xml`。
最终planner SHA256
`744b9a69eca839fc450fbf73f6b2564ef83816e0aec5acaf06be9bab0ca2d6b0`。
20:57:42仅gateway经原launch respawn加载为PID75211；remote71319、driver56172、
GUI45301、task6644与控制器均不重启。没有调用恢复/抓取/归零/失能接口。

20:58同一个现场FK目标连续3次只读strict检查全部通过：capture后两次均为
fresh_start_fixed_joint_goal，目标关节及SHA256始终
`78a75b88efd5357f74197917b5a958e880e7b506b8f629be74d3715edb57e63b`，日志保存为
`support_tilt_KewwBe/exact_branch_readonly.log`。**这些检查发生于下述电源重启
后的近零位，不是原失败姿态的完全同条件重放**；证明生产接口能保持固定
分支，不宣称先前FK异常唯一原因或真实抓取已解决。三次均execute=False。

实时遥测记录了先于gateway本次重载的外部状态变化：20:57:26附近J2反馈
变化，20:57:28驱动因硬件反馈超过1 s暂停位置流，20:57:45收到全六编码器
零值并转 UNCONFIRMED:ENCODER_FEEDBACK_NOT_READY。随后读数约
[-.004602,-.010738,-.010738,-.004602,-.010738,.006136]，目标检测丢失。
期间无新的joint_commands，最后SDK位置目标未改变且之后发送计数不再增长；
没有Received Zero Calibration或主动torque_off记录。观察到的E1/E2与温度
突变需按原始遥测保留，不据此单独认定过热/串口损坏。

用户随后明确：“我刚刚关闭了机械臂电源，已经重启了”。因此该事件已获
操作者电源操作确认，不能归因于只读规划或gateway重载。驱动保持原PID，
其失去真实运动确认与停止发送旧位置流**不等于发布机械臂失能命令**。
重启前1.595 mm净距与75°观察姿态只属于历史窗口，不能再描述为当前姿态。
当前需用户重新对准目标并交回控制；不能沿用重启前目标计划或擅自重放SDK
缓存。正向微调响应、原对准→小倾转跨姿态一致性、实际CAD验收合同及完整
持物终态仍未完成。主日志和技术路线继续保留这些事项，未标记抓取成功。

### 21:16 起：重新对准交权、反馈竞争与新倾转候选

用户明确“已对准，交回控制”。本轮目录为
`.ros_log/grasp_attempt_20260911_realigned_Swve2U/`，先启动纯订阅
watch/keyframes；没有复用断电前的目标计划。新对准实测六轴为
[-1.9282138504,.5292233718,-.2837864458,.0046019424,-.1426602133,.0061359232]，
carton 稳定识别，相机深度约312–314 mm。

21:16–18 看到反复 `Pausing SDK command stream ... stale`。**旧日志不能
直接证明反馈间隔超过1秒**：发现 send callback 先采 `now`，再等
`data_mutex_`；另线程可能在等待期间写入更晚的 accepted timestamp，
于是负龄被当 stale，旧日志又统一打印 timeout1s。修复为同锁时间/反馈
快照，保留负龄和原1s拒绝，日志区分 missing/zero/future/expired 并给实际龄。
新增3项RED复现，修后相关driver模块34项通过；第一次构建成功，尚未据此
宣称现场竞态已经消失或 Joint2 实体微调已经正常。

按本次明确交权于1789186770.904发布 GUI direct false并同步参数，driver
记录保留最后SDK hold，未发新关节目标或controller/torque命令。随后新订阅
却仍收到GUI自身旧latched True，而参数已False：GUI处理远端模式时此前
不刷新自己的latched publisher。已限定为实际模式变化时同步参数并刷新
自家发布；checkbox先更新，重复值和自回声立即返回，不循环、不发关节目标。
新增双向RED复现，GUI/driver四模块79项通过。随后heartbeat证据来源加固
至accepted timestamp，相关81项通过；现行SDK分派没有调用旧legacy伺服/
夹爪parser，不把潜在generic timestamp复用写成本轮已实测status-only续命。

在没有运动的情况下，请求一次新鲜观察候选并取得匹配已提交审计。
plan `4414d3486c03baaac4f54a25` / request26 / 源戳1789187080215157270：
**roll0°、tilt15°、21 cm、nominal支撑净距25.210648 mm**，
maxdelta.616 rad、硬件时长下界7.697 s（Joint2限制），28/30检查、17可达。
这次倾转确实参与严格规划，不再全部被MuJoCo 2s寿命错误跳过；不据此宣称
实测执行耗时7.697 s或已解决跨姿态失配。匹配报告保存在
`far_preview_only_audit.json`，这是从终端JSON重排提取的诊断副本，不冒充原
已提交文件的字节哈希。整个preview-only程序没有构造抓取/运动服务，结束
仅调用 request_plan(False) 结束候选计算；该preview不再作为后续执行授权。

正在补齐：独立accepted SDK反馈topic（不改变原heartbeat TF兼容）、实际
到位CAD冻结几何验收、最终重定时观察路径及bounded prefix的同冻结几何
检查。端点大净距不能冒充完整路径安全，离线q直线也不能替代真正提交轨迹。

### 21:30–21:33 GUI/driver 加载和接受帧证据

相关四模块 **84 passed /3.84 s**，报告
`driver_gui_handoff_regression.xml`；driver C++ **81 passed /617 ms**，
报告 `driver_cpp_regression.xml`。隔离ROS master的driver二次构建成功。
21:30:41只重载GUI服务，新PID89144；初始化期间首个3s等待超时，随后新建
3次订阅均收到False，参数也False，不能把初次初始化延迟写成latch修复失败。
21:32:02只重载driver服务，新PID89835，按既有auto_torque_on_startup发送
正向enable，未发失能。构造时DISABLED/NOT_REQUESTED与之后PENDING是软件
状态，不是torque_off。旧task、gateway、controller、remote保持运行。

实际部署driver二进制SHA256
`d03d82c27a9ff6908783abc3e65bef2b96993a7294ef601849151cdb4f9dd5a4`，
driver源文件 `2cb09234edde098df91e91473a50d2e95a6b53dcd84e2bcbcd0a8d35d5f61b44`；
GUI widget `cca90b7a076c77bdb3515b1bb60a9ee08317f3a3e060f640b3467334adbbdc6f`。
服务旧版本环境标签沿用，实际以哈希/时间/PID为准。

新增 `/alicia_d/accepted_joint_states` 只从完整且通过验收的SDK关节包发布，
包括同包六臂与right_finger，frame_id=sdk_measured，stamp为本机接受时间。
它不是固件采样时间或UART排队延迟证明；原/joint_states heartbeat与TF时间戳
不变。15秒只读窗口135接受帧，最大源间隔.209116s、最大接收龄.058881s，
各轴span均0、SDK位置写入数0，状态PENDING。报告
`accepted_feedback_after_reload.log`。尚未重新发位置流，故不能仅凭这段
无stale日志宣称send竞争实机验证完成，也不能把PENDING改写为已运动确认。

相机图 `aligned_camera.png` 源戳1789187557370950937：目标仍在桌垫中部，
左侧画面边缘可见手指。已告知用户并请求双手离开运动范围确认；收到前
继续纯软件/只读工作，不发起抓取运动。此提醒来自新画面证据，不是重复
套用启动前检查清单。

### 21:40–22:00 观察路径与实际到位合同离线验证（尚未加载）

新增纯数学 `observation_path_guard.py`，读取实际 URDF 串联链和关节限制，
按 ROS Noetic 控制器的 linear/cubic/quintic 插值重建最终重定时轨迹，包含
零时间戳轨迹丢弃 t=0 点后的控制器接续段。起始 desired 必须来自本 gateway
已经完成的单点同步 hold，且新鲜、位置一致、速度/加速度为零；不能用任意
一帧测量假定静止。采用 Bernstein 包络和递归细分，对 Link6/7/8 真实夹爪
CAD 全程验证冻结支撑面 3 mm 净距与无目标 OBB 相交；超预算/无法证明即拒绝。
模型、轨迹（含时间与导数）、场景与计划均记摘要，不更改原轨迹或速度参数。
观察 full execute 和 bounded prefix 均在最终重定时后、真正提交前调用。
门控拒绝不属于控制器执行失败，不额外调用 failure hold 或触发范围恢复。

这是**名义命令轨迹的夹爪几何证明**，不是全臂环境碰撞证明、运动中实测监控、
SDK 量化后的物理轨迹证明或闭环跟随误差界；同步 hold 本身也不在其证明内。
例如在本 URDF 下把六轴各一个 SDK count 当作独立最坏偏差，点位移上界约
5.286 mm（半 count 约 2.643 mm）；该估计没有被悄悄加入/扣除原 3 mm 门限。
保持 MoveIt 原碰撞检查，不能把本次名义证明写成实体全程 3 mm 保证。

task 新增实际观察到位 CAD 验收：原观察/复用到位、径向修正与允许的补拍
均在进入近场推理前，用冻结计划、<=0.5 s accepted SDK 六轴及夹爪开度、
同窗实际 tool0 TF 验证支撑净距和 OBB；拒绝缺失/重复/非有限关节、错误frame、
非法开度、零四元数及到位等待失败。TF 和接受帧分开记时，并非严格同帧 FK。
派生观察目标仅克隆为冻结源 header，Pose 字节不变；不改 xyz/姿态、标定或
原远场几何，接触服务请求不变。

复审纠正最初的生命周期实现：仅接受已提交 enriched plan，不用 preview
或 latest geometry；首次冻结保持原任务有效期，已冻结活动任务不在每阶段
重算 120 s 到期，允许同一冻结源的派生观察位，仍经完整 strict 6D/CAD检查。
同 track 后续计划不覆盖活动场景，终止、直控、撤销与换 track 要撤销旧证据。
22:00 全量 **2410 passed /3 skipped /7 warnings，159.58 s**，报告
`pytest_full_observation_contract.xml`。随后独立复审又复现：首冻结前同源但
不同完整计划被错误合并、FAILED(active=True) 终态未立即撤销、已检测断线后
重连可恢复旧捕获；另有 active topic 与紧接 RPC 的跨连接接收倒序竞态。
这些是**尚未部署的新门控缺陷**，正在限定修正并补回归，不归因于历史失败。

只读生产模型对照 `urdf_fk_production_readonly.json`：新 FK 与 `/compute_fk`
三组 q 最大矩阵误差 6.106e-16，URDF 摘要
`890592a24b8ad818d995e69beb670ea4254bf84f589615bfac3c8562f62449b7`。
`continuous_path_diagnostic_only.log` 为只规划诊断：拦截 execute/go/stop，
使用合成静止起点（不是现场 gateway 已完成 hold），规划重定时 12.340 s，
峰值 0.05 rad/s；11 段、23 区间、46 次 FK，名义证明下界 7.927437 mm，
计算约 0.494 s。该诊断不产生执行授权，更不是实机用时或抓取成功。
此前 preview-only 计划已过期；之后必须重新获取新鲜计划，不能重放。

### 22:03 实际到位 task 独立加载

确认旧 task 状态 FAILED/active=False，无在途启动请求。代码复核其没有
shutdown 动作回调；精确 `rosnode kill /grasp_task_node` 仅结束旧 PID6644，
没有调用同名 stop 服务。原 launch 的 task 不 respawn，故使用独立服务：

```bash
systemd-run --user --unit=alicia-task-observation-20260911.service \
  --property=WorkingDirectory=/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade \
  --setenv=ROS_MASTER_URI=http://localhost:11311 \
  --setenv=ROS_LOG_DIR=/home/zhuyupei/alicia_wa_full/.ros_log \
  --setenv=ALICIA_CODE_REV=task-observation-envelope-4ac71660 \
  /bin/bash -lc 'source /opt/ros/noetic/setup.bash
source /home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade/devel/setup.bash
exec rosrun alicia_flexible_grasp_supervisor grasp_task_node.py __name:=grasp_task_node __log:=/home/zhuyupei/alicia_wa_full/.ros_log/3308d82e-ae49-11f1-9701-516764d618aa/task_observation.log'
```

22:03:09 新 PID102630，22:03:14 IDLE ready；node info 确认新 accepted topic
实际连接到 driver。启动拒绝旧 latched plan 的 PLAN_STALE 是预期，不重新
使用已过期预览。新源码 SHA256
`4ac71660eed92835e1076101704aee7458957c3c1dde983ead0ea0753dc56bb5`，
task 测试文件 `236428e9183c417dea907641d2b86d1f96cedeb269126ea77cdae761084bab53`。
构造仅发近场规划 inactive 和任务 IDLE 元数据；driver/GUI/controller/camera/
remote 保持原 PID，无新 SDK 位置目标。gateway 仍等生命周期收尾后再加载。

### 下一次真机任务必须回答的问题（尚未完成）

1. 用新的远场源和小倾转候选执行原观察流程，分别记录 input、实际成功写出
   的 SDK 命令、accepted 编码器、控制器 desired/result；尤其核对 Joint2
   正向微调是否仍为多 count 目标、长时间零编码器响应。时间快照和比较边界
   修复不能代替这项物理验收，也不以更大反向/正向试探或虚构位置解除门控。
2. 到观察位后分别验收真实 CAD 与远近几何：前者识别实际跟随下沉，后者
   对比原冻结支撑面和新 RGBD 背景/目标对应。仍保留 4°/4 mm，不用替换远场
   参考、TF 偏移或放宽阈值把 15 mm 失配隐藏成通过。
3. 若小倾转仍出现背景与目标共同失配，应补实际工作姿态的手眼标定覆盖与
   跨姿态留出验证，并区分成像/关节时序、机械柔顺/回差、外参误差；不能仅凭
   两姿态差值写补偿。若 Joint2 命令编码正确而反馈继续不动，优先定位执行器
   死区/负载/回差等实体问题，不再归咎已保留的多关节同时运动逻辑。
4. 只有通过近场三帧融合、严格双侧接触、实测预抓取、直线接近、闭爪、抬升
   及可追溯持物证明，才把本任务记成功。当前既没有这一终态，也没有连续
   物理跟随误差上界；软件单测和候选净距改善均不能替代。

### 操作者再次确认双手离开；准备新鲜实机任务

用户随后明确回复“双手已离开”，前述人员入镜等待条件已由新的操作者确认
解除；不再沿用“尚未确认”描述当前状态。新一轮纯订阅watch/keyframes目录
为 `.ros_log/grasp_attempt_20260911_hands_clear_Ghhuv8/`。先完成限定生命周期
修复和加载，再获取新鲜计划，不复用先前预览。

最终四项生命周期修正已由 7 项 RED 复现并转 GREEN，额外覆盖终态/断连后
较新 active 不复活、真实 inactive→active 可重新入场、等待期间源过期仍
拒绝。五模块 **171 passed /8.61 s**：同源首次比较完整计划摘要；终态与
已观测断线锁存撤销；首次入场用释放 RLock 的 Condition 最多等待0.5秒真实
活动回调，monotonic期限，提交前复查不等待/不创造授权。geometry源冻结后
重新运行全量 `pytest_full_observation_lifecycle_final.xml`，结果待记录。

### 22:11 最终全量与 gateway 加载；22:13 新计划任务启动

最终全量 **2419 passed /3 skipped /7 warnings，179.39 s**；独立终审新版
guard 70/70，无新增锁反转或放宽有效期。22:11:24 gateway经原launch respawn
从PID75211更新为 **106028**（精确rosnode kill，仅进程重载）。node info
确认 /grasp/state、已提交plan、accepted反馈均连接；task102630、driver89835、
GUI89144、remote71319与controller保持不变，重载未发位置运动。

本次实际加载 SHA256：

- gateway `eb41769106a49f7f281f4711aaca46a795cb47a58a91c02baaa8aceaaffe7bc9`
- guard `46e0a03361612522310edb9929c657c48710eef829818e8017d92eedeab50432`
- planner `db3e7238ca3f58d0553b43d5e8307ed79f5cbd5906da4754b5f0a4b0c13256d4`

重载附近另观察到识别源龄8–13秒，未立即发运动。只读系统状态显示当前
主机仅2逻辑CPU，负载约16.6；测试并行/主机负载是候选因素，不能据此证明
感知队列bug或擅自结束操作者IDE。随后12秒同窗采样：accepted108帧，中位
源龄4.28ms/max27.21ms；RGB104帧，中位17.30ms/max159.24ms；检测3帧，
源龄1.739/3.246/4.024秒，已回原8秒内。未改任何时限或相机/推理配置。
使用原 `tools/run_fresh_grasp6d_after_alignment.py` 获取新鲜候选及精确计划
授权后进入任务，原始runner/watch/keyframes保存在hands_clear新目录。
本段只记录已发起新计划工作，实际运动与终态须据后续日志单独记载。

### 22:16 新鲜实机结果：观察成功，补拍前同步造成二次下沉（未抓取成功）

新计划 `df945be9d4d112da2c8b958d` 源戳1789190140892387151，入场源龄36.694秒。
22:16:17.839开始有界观察prefix，20.750转
`CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE`。22:16:22主观察轨迹重定时12.437秒、
maxdelta0.622rad、peak0.05rad/s，固定关节目标摘要
`ba392a5d2ebd25b9fdd9471cdb31d60522cb1cee44a4e59ff947d889be5b9724`。
22:16:35.559控制器返回成功，36.472静定结束；主轨迹名义连续净距下界
4.565248mm，prefix下界47.311486mm。二者仅证明命令夹爪几何，非实物误差界。

到位实测 q 为
[-1.9220779272,-.1411262325,.1902136177,-.0552233084,-.2331650798,.0674951547]，
FK误差10.5mm/1.64°；实际模型最小净距 **5.963657216mm**，通过原3mm。
22:16:39新鲜相机目标距离191.6mm，通过18–22cm，进入near phase1。
近场得到缺双侧表面证据的结果，按原一次补拍规则进入strict预规划；不是
本轮已通过原4°/4mm跨姿态验收或已证明Joint2正向微调恢复。

22:16:51.804新连续门控在补拍轨迹提交前拒绝：
`OBSERVATION_PATH_INVALID: support clearance -0.001308589m at dropped-start/left_finger`。
FAILED后runner退出6，候选计算结束（不是arm停止）；没有提交补拍轨迹、
闭爪或抬升，不能记完整抓取成功。keyframes关闭时 **214帧**，全部源证据保留。

**独立双重复现揭示真正新增根因，不是静置漂移或两套CAD误差：**

- 22:16:36–22:16:51.250944期间实测六轴完全不变，原自动desired持续保留。
- 22:16:51.429705，gateway `_synchronize_trajectory_controller_to_feedback`
  把上述实测角度当作新controller目标；51.445628 SDK成功发出对应目标。
- 51.549726新accepted反馈已让净距降至1.833653339mm；51.651181 Joint2再降
  10 count、Joint3降8 count、Joint5降2 count，q变为
  [-1.9220779272,-.1564660404,.1779417714,-.0552233084,-.2362330413,.0674951547]。
  同冻结场景/URDF下tool0 z从.087361831降到.078776449，**再降8.585382mm**；
  实际左指净距 **−1.308588628mm**，精确重现拒绝值。
- pure FK+CAD与task实测CAD对两组q计算一致（误差<=3e−16m），无plane/模型
  偷换；dropped-start恰是已经真实达到的q，删除这个门控不能消除实际低净距。

因此应精确描述：**补拍轨迹未执行，但此前所谓同步hold已引起实体运动。**
当前前置sync不在连续轨迹证明内，这次实验证明该缺口会触发失败。机械执行
存在命令/反馈残差时，把actual反复写成desired，会把残差叠加成逐阶段下沉。
既有failure-hold代码已保留desired避免二次阶跃，但普通strict执行入口仍无条件
feedback sync，遗漏了同样问题。之前“关节没有动所以可重发feedback保持”的
假设不成立，不能只调大路径容差或只删除dropped-start检查。

已停止发起后续动作（未发送任何失能/停止命令）。当前负值是CAD模型与冻结
平面的估算，不等于已确认实体碰撞；也不能因此自动设计逃逸、改TF或重建
参考。下一项修复要区分真正控制权交接与连续自动阶段：保留可追溯的当前
成功命令基线，并把接续段的几何、速度与新鲜实际起点纳入提交前证明；不能
简单跳过sync后任意重用旧desired。当前低净距姿态不作为继续自动试验起点。

用户提出电脑卡顿可解释延迟，与本机2CPU高负载证据吻合，但不是本轮同步
二次下沉的原因。只读代码另确认：YOLO标准CPU路径已经限制计算线程为1，
20余进程线程不能当作20路推理；LatestFrameBuffer没有无限FIFO，但同步推理
完成仍会发布旧源帧，RGB-D未严格验深度戳，perception旧图像还使用latestTF。
ROS相机戳是在read/align后打的，因此前述17ms只是ROS源龄，不是曝光延迟
证明。这些时序风险需分别处理，不能据此冒充已解释历史跨姿态15mm全部误差。

二次下降分解：J2单独使左指净距减少4.495830mm，J3减少2.641429mm，J5减少
.214745mm，非线性余项+.079758mm；最大影响是J2，其次J3。136个accepted帧
覆盖36.459727→51.354306静置窗口，每轴跨度0count，排除本窗口内缓慢漂移。

新候选主轨迹名义净距13.248mm而旧preview约25mm也已独立拆解：旧request26
选15°tilt/0°roll/21cm，新request60选15°/0°/20cm。旧pose放到新scene净距先
减少2.894mm，再换新目标pose减少9.070mm；新21cm候选标记
`OBSERVATION_TILT_CHECK_BUDGET_REACHED`、checked=False。gateway首次严格检查
包含MoveIt惰性初始化，耗尽原3秒软启动预算，因此不能将该候选写成不可达。
后续在最后一次gateway加载后、获取新鲜场景之前做只读规划预热即可先验证
这一冷启动因素，不加时限、不放松净距，不重放旧计划。

本轮未仓促新增全局一次sync闩锁：它会阻断prefix→主观察且重载后缺少可靠
owner证据，不能冒充完整修复。当前生产源码仍是上列已部署哈希，同步缺陷
尚未修复；自动runner已结束，未发起下一轮。当前实际低模型净距需要操作者
通过GUI恢复原安全对准位，不能直接自主逃逸；后续再实现并验证自动参考
连续性、controller接续段速度/CAD与实际跟随合同，之后才重试完整任务。

结束前只读新accepted源戳1789190741959783354（接收龄13.03ms）：q为
[-1.9220779272,-.1580000212,.1794757522,-.0552233084,-.2362330413,.0674951547]，
开度.04975m；相对失败时J2/J3各又有1count变化，不能称失败后永远精确静止。
task仍FAILED/active=False/success=False。后台纯订阅telemetry PID7666与trace
PID42042仍运行；本轮runner和214帧keyframe记录已自然结束，未宣称AI离线后
仍持续人工式监督。gateway源哈希仍eb417691…，未部署临时闩锁或隐藏改门限。

## 后续源码修复与操作者暂停指令

本轮操作者确认双手离开并提出“并退回原安全位”，随后两次要求“修复完不启动
6D抓取，直接暂停”。因此本轮仅修源码、隔离ROS master离线回归/构建、更新
文档，不重载节点、不发起抓取，不发送停止或失能命令。

前述同步重基准缺陷已有源码修复：驱动保持成功SDK冻结参考，网关不在连续
自动阶段发布actual hold；真实controller接续段加入连续速度证明，保留原
速度上限及几何要求。完成回调乱序、跨帧时间戳拼接和同值初始化命令被硬件
省略的边界也纳入回归。不会用微动、假ACK、全局一次运动限制或改变TF解锁。

详见[命令参考连续性修复记录](2026-09-11-command-reference-continuity-repair.md)，
最终离线数量、源码/二进制哈希及未部署边界以该记录和主运行日志为准。
本次软件修改尚未加载，先前实机失败不因此改写为成功；Joint2正向微调与
跨姿态约15mm的全部物理来源仍未完成修复后实机验收。后续按最新指令暂停，
不继续此前“直到抓取成功”的自动试验。

## 2026-09-12 操作者恢复、重新对准后的实机补记

用户本日明确恢复完整ROS并交回控制，昨日修复已成套加载。新计划第一观察
执行成功，近场返回NEAR_FIELD_NO_REACHABLE_CANDIDATE，无闭爪/抬升成功。
实际末端误差仍12.922mm/2.44°。观察后SDK参考保持不变，未重现此前补拍前
actual重基准二次下沉；但本次未执行第二条自动轨迹，不能声称全流程验收。

仅计算诊断已把本次新阻断缩小到固定接触姿态的运动学余量：抓取点可解而
预抓取/接近端点无解，Joint5抓取解接近原下限。没有修改关节限位或清空误差，
也没有改变TF/TCP。启动、159帧记录、精确fixture、原始结果与剩余工作见
[2026-09-12实机与IK边界记录](2026-09-12-startup-grasp-ik-boundary.md)。

本日后续仅修复remote的接触路径失败恢复与精确序列去重，未重新加载/运动。
当时accepted源帧的FK分项显示Joint3、Joint2、Joint5的单轴末端位置影响分别
约6.480、4.170、2.666mm，不能仅把12.922mm到位残差归咎Joint2；这些范数
不是可相加占比，也不确定硬件物理根因。Joint2微调、历史跨姿态15mm和最终
持物仍待分别验证，未放松误差合同，未修改标定、限位或驱动。详见
[2026-09-12接触路径恢复修复记录](2026-09-12-contact-path-recovery-repair.md)。
