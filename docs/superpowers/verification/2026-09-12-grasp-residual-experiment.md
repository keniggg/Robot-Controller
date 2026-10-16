# 2026-09-12 正常 6D 流程中的同步残差实验

## 执行前方案（19:33 PDT）

用户明确授权先设计排查实验，再执行一次 6D 抓取。现场新订阅确认 GUI direct
为 false、task inactive、真实 accepted 数据持续到达；carton 当前深度约
0.310m、confidence 0.904。不是使用 18:34 的旧对准图或旧计划执行。

先仅重载 remote 感知规划节点，使前轮通过回归的“全部接触路径不可达后的
一次注册补拍机会”和“精确重复完整序列去重”生效。driver、gateway、GUI、
task、控制器保持运行。不做额外试探运动、不修改使能状态，不发布任何停止/
失能命令。整个实验只通过既有 fresh runner 的全新源戳、审计及 plan_id 绑定
入口执行一次；正常终点精调仅由原接触阶段获取任务 lease，不全局开 trim。

### 假设与可区分证据

| 排查环节 | 同步记录 | 判据与不能下的结论 |
| --- | --- | --- |
| 规划→驱动发送 | action goal/result、controller desired/actual/error、joint_commands、control_reference/epoch、成功 sdk_command | 按 joint name 对齐，在同一目标稳定窗比较；成功 write 不冒充硬件 ACK/运动 |
| SDK→真实关节 | accepted 原始源戳/位置、SDK 原始值、run_status/温度 | 分别统计 Joint2/3/5 的 count 误差、方向响应、稳定残差、接收间隔；不能用重复 joint_states 心跳作独立反馈 |
| 任务终点闭环 | task 阶段、rosout 精调 lease/step/response/timeout、实际终点 FK | 区分“未进入接触故未请求”“有请求未产出步进”“已发步进但实际无响应”“响应后仍不收敛”；未走到该阶段就标未检验 |
| 视觉/几何 | 精确同源 RGB-D/mask/detection、TF、geometry、gate audit | 保留实测与冻结目标的差，不移动 TF/TCP、物体中心或清空残差；模型内 FK 不等于外部几何真值 |
| 上次恢复缺陷 | 每次 request 的 candidate/sequence、near_field_recovery、注册结果和 deadline | 核对一次补拍是否触发、同一 90 秒预算是否保留；去重不能使不可达变可达 |

基线为上一轮 12.9217mm/2.44°观察残差、规划→SDK 0.1815mm、SDK→反馈约
12.7980mm。两次起姿不同，所以本次不能直接将误差降低归因为 remote 修复。
这是一轮正常抓取的被动观测实验，不是舵机负载/背隙/方向反转的受控辨识。

### 执行与终止边界

1. 先启动每次独立、不分卷的 keyframe bag 和纯订阅终端 watch，保存参数、
   代码版本及审计。新增 reference/epoch、action goal/result/status 等被动
   话题，不增加控制发布器或高频在线 FK/IK 诊断负载。
2. fresh runner 获取新鲜图像的第一观察位审计，再调用 `/grasp/start`。观察
   18–22cm验收；残差原样记入近场不确定度，观察没有接触 precision lease。
3. 按原路近场三帧、至多一次注册补拍；若进入接触，记录任务级精调，将真实
   预抓取误差是否收敛到 6mm/5°与controller SUCCEEDED分开判定。
4. 保留CAD 3mm、注册4°/4mm、Cartesian≥.98、原速度/关节限位和90秒/20秒
   预算。不为收集数据绕过失败门控，不自动盲目重试，不追加校准/回退运动。
5. 全流程成功必须是闭爪、抬升后真实视觉持物证据。若提前失败，明确最后
   完成阶段、首个阻断、精调是否实际检验；失败不是完成成功抓取。

## 实际运行结果

### 部署、执行和证据

已按方案执行一次，不是只设计未调用。remote 旧 PID8394 正常退出，其 shutdown
只结束推理 worker；替换节点 PID40831，由
`alicia-remote-residual-20260912.service` 运行。已加载源码 SHA256
`ea39a2be0bb65f8669b576129c6e1dbade8feab8fedbdceab22fe2b6ed378f79`。
driver 原 PID8366、控制器、gateway、task、GUI 均没有重启；driver binary SHA
仍 `30edad32f5c2384cfc2928550eca8aaef5c2d6cf87b2ee93848ecb73b934c940`。

根工作树证据目录：
`.ros_log/grasp_residual_experiment_20260912_rOzvYw/`。
其中 `runner.log`、`watch.log`、`recorder.log`、`remote.log`、前后参数 YAML、
闭合 `keyframes.bag`、`keyframes_audits/*.json` 可追溯。采集和执行服务均已
自然终止；ROS主服务、替换remote及原旋转遥测服务保持运行。

19:35:16.059856 请求新鲜推理，最低允许源戳1789266916059855937；最终
request95生成计划 `811e4194ddca6b2c2fa71a7e`，源戳
**1789267030266447544**。执行审计归档 SHA256
`be5b34bc60f4b3fa6cd15ce37073ce32c0d7e573f264a08eb03e13f59824dbca`。
roll0°/tilt15°，规划相机距离0.21m，规划观察包络净距31.8266mm；这些是
计划值，不是实际到达值。观察方向比较33/37、33 reachable，名义硬件时间
下界7.224s；到达实体运动之前即失败，不能把此下界记成实测执行时长。

19:37:42已完成同源审计及plan_id绑定，并实际调用 `/grasp/start`。最终
`GRASP_RESULT success=False`，runner exit6。**没有执行观察主轨迹、近场补拍、
接近、闭爪或抬升，本轮没有成功抓取。** 正常交权同步曾发出一个与已发送
SDK位置相同的六轴 ros_control 命令；因此不能写成“完全没有位置发布”，但
这不改变成功SDK字或实体位置。没有停止/失能指令，也没有失败后自动回退/重试。

### 首个执行阻断：20ms同步完成时间假设与控制器异步接收不一致

| 源事件时间（Unix秒） | 真实记录 |
| --- | --- |
| 1789267062.6357002 | controller仍保持18:33旧终点，desired v/a均0；driver为sdk_handoff_required，SDK保持操作者后来的当前姿态 |
| 1789267062.6469483 | gateway确认控制器已运行，未重复启动 |
| 1789267062.6757088 | controller收到本轮同步后仍处于插值首帧；Joint2 desired velocity=0.07063664783247676rad/s，desired acceleration=8718.11697035524rad/s² |
| 1789267062.6949897 | gateway返回CONTROLLER_REFERENCE_INVALID: controller reference is not stationary |
| 1789267062.6957028 | 下一controller帧已到冻结SDK参考，六轴desired v/a严格为0 |
| 1789267062.695715046 | 唯一joint_commands为六轴ros_control终点，等于先前成功SDK位置word |
| 1789267062.6979356 | driver参考转为sdk_tracking；实际关节未变化 |

注意数值含义：两张controller帧相隔约20ms；**报错时间到下一张静止帧只有
约0.713ms**，不是“报错后又运动20ms”。该巨大加速度是旧controller参考到
冻结SDK参考的内部短桥插值，不是实体加速度；成功SDK及accepted完全未变。

源码定位：`motion_gateway_node.py::_synchronize_trajectory_controller_to_feedback`
把 `publish_time + bridge_duration(0.02s)` 当作已经完成同步的时刻；
`_controller_reference_admission_state`对时间晚于它的中间帧，看到任一非零
v/a立即失败。ROS发送、controller接收及状态回调并不同步。本轮恰好在真实
同步完成帧到来前拒绝；没有等原2秒deadline内的最终静止、同SDK字及ACK证据。
这是**交权同步时序/状态机缺陷**，不是6mm接触门限，也不是实体尚未静止。
负载/调度可能改变触发概率，但不应把用户电脑卡顿当作唯一根因。

还有后继风险：这次错误返回发生在记录 `_completed_command_reference` 之前，
而driver随后已经tracking。下一轮不能把迟到ACK冒充已完成gateway指令来源，
也不能重写实际位置、重启驱动或重复发保持命令来绕开。需一并设计有原请求
来源的迟到ACK收尾/恢复。当前没有进行第二轮重试。

### 对12.9mm实验问题的实际回答

bag长度171.9343s，146套精确同源RGB-D/mask/object，44份哈希验证审计，
1557帧accepted、1545帧SDK、8594帧controller状态。所有accepted六轴位置
完全一致，所有SDK六轴位置也完全一致；成功SDK减accepted为
`[-1,+4,+6,-2,+2,-1]`个4096分辨率count。accepted接收源戳间隔中位数100.004ms、
P95约200.000ms、最大203.152ms；温度32–38°C。无新action goal/result。

这次没有形成新的运动后终点样本，也没有获取contact precision lease。
`runtime_execution_error`仅录到旧计划951031cb...的12.9217mm参数，源戳仍是
上轮；不能把它当作本次又测得12.9mm。本轮**未检验**Joint2小步精调响应、
接触终点6mm收敛、跨姿态几何误差和近场一次补拍修复。它确实定位了更前端的
交权阻断，不代表原跟随精度问题已解决。

### 次要问题与后续工作

1. 首个计划等待约146s。44份审计中38份终态为GraspNet姿态约束拒绝、5份
   STABILITY_PENDING、1份PLAN_READY；多次桌面提议因CONTACT_SUPPORT_INVALID
   为空。远场设置contact_execution_gate_deferred=True，但提议层仍要求非空
   双侧公共高度，再进入后续观察构造。当前点云下这造成候选供给不稳定；最终
   仍生成了合法计划，所以不是本轮最终失败原因，也不证明所有方向不可抓。
   后续应把“观察目标/视角构造”与“接触证明”分开，不伪造接触表面或修改限位。
2. rosout还有5条MoveGroup wrapper的goalHandle未跟踪错误；保留为并发客户端/
   规划时延排查项。本轮明确返回的执行阻断是上述reference错误，不能仅凭这些
   actionlib日志替换根因。
3. 修复方向：在同一次冻结SDK交权事务内，根据真实终点/零v/a/同SDK字及新鲜
   ACK判完成；合法异步过渡仅在原deadline内等待。手动抢权、epoch变化、错误
   SDK字、陈旧反馈、真正不收敛继续拒绝。补“接收晚于20ms”和“失败返回后的
   迟到ACK”回归；不只是调大速度容差或把非零速度当静止。
4. 本轮未改/加载gateway生产逻辑，不能宣称上述时序bug已修复。下一轮正常
   6D执行前先完成该修复和验证，再按本方案测实际SDK→反馈→终点精调。

### 记录器验证和边界

本轮只改被动记录器：新增reference/epoch、action和状态话题、哈希绑定审计
归档及1Hz参数快照（bag内虚拟话题，绝不ROS发布）。启动前5秒实采取得5套
同源图像及连续遥测，单测验证审计匹配/防覆盖及无控制接口。
调查后发现同步hold使用`/alicia_controller/command`而不是action；已把该纯
订阅加入**后续版本**，本轮bag没有该话题，不能声称已采到本次hold原始消息。
本次时间线来自controller状态、唯一同值joint_commands、参考模式和源码。

最终隔离验证：`ROS_MASTER_URI=http://127.0.0.1:11319 python3 -m pytest -q
src/alicia_flexible_grasp_supervisor/tests/test_keyframe_audit_archive.py
src/alicia_flexible_grasp_supervisor/tests/test_fresh_grasp_runner.py`：**9 passed，
0.77s**，`git diff --check`通过。这是记录器/执行入口验证，不是gateway时序
修复回归，更不是实物抓取成功。最终只读状态仍CONFIRMED，GUI direct=false。
