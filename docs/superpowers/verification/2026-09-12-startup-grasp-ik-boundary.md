# 2026-09-12 最新节点启动与本轮近场IK边界失败

## 结论与当前状态

本轮已实际执行第一观察运动，但没有成功抓取。不是节点未启动，也不是第一
阶段门限把运动挡住：观察执行成功、实测约21cm、冻结几何净距7.246mm通过
原3mm；随后近场4个条目（只有2套不同接触Pose序列）均未通过严格运动学检查，
返回 `NEAR_FIELD_NO_REACHABLE_CANDIDATE`。没有补拍、接近、闭爪或抬升。

只读IK对照发现：两套固定接触姿态的抓取点可解，但预抓取/接近点均无解。
关闭碰撞仅作诊断也一样，不能将此归咎碰撞门限过严。一个抓取解的Joint5为
−1.566704rad，距URDF下限−1.57rad仅0.003296rad（约0.19°）；从抓取点沿原
直线反向退开2mm，当前求解即失败。该姿态的关节边界是强阻断证据，有限
初值搜索不证明整个物体、全部抓取方向或所有IK分支都不可达。

没有绕过关节限位、碰撞、接触、Cartesian .98、原预算或3mm净距；没有移物、
改TF/TCP或重放旧计划。本轮未发任何停止/失能命令。任务失败后未盲目启动
新运动，仍有只读监督/旋转记录，实物成功目标尚未达成。

## 启动、授权与证据

操作者本日先要求恢复完整ROS/正向使能，随后明确“已对准、交回控制”。
同一部署工作树为 `/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade`。
启动命令及服务详见[主运行日志](../../../src/alicia_flexible_grasp_supervisor/logs/2026-07-23-ros-latest-node-launch.md)。
driver PID8366运行二进制SHA256与昨日最终构建一致：
`30edad32f5c2384cfc2928550eca8aaef5c2d6cf87b2ee93848ecb73b934c940`。
gateway8389、task8398、GUI8405同套加载。driver正向enable请求及真实SDK包
已看到；用户GUI调节后确认态为 `CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE`。
18:30:07由GUI退出直控，driver冻结成功SDK参考；助手未发送新的滑条位置。

本轮根工作树证据目录：

- `.ros_log/ros_resume_20260912.log`：完整节点日志。
- `.ros_log/runtime_resume_20260912_cIaAKg/`：终端watch及128MB×8旋转bag，
  包含新control_reference、reset epoch、成功SDK与accepted反馈。
- `.ros_log/grasp_attempt_20260912_aligned_JiFKiE/`：runner.log、aligned_camera.png、
  keyframes.log、已正常闭合的159帧keyframes.bag、ik_offset_readonly.log和
  post_attempt_watch.log。首个watch随任务终态自然结束，随后另起纯订阅watch，
  不是发机械臂停止命令。新watch本次期限1800秒，不承诺AI离线后永久在线。
- 原近场审计 `/home/zhuyupei/.ros/grasp6d_gate_audit_latest.json`，request54，
  SHA256 `b43ef4d383f405c2ca3f2e5f0e98513f6653c3c81ed9ac1dd1204efbdbbf7754`。
  关键精确Pose、源纳秒、虚拟起点、profile、失败和缓存证据保存为
  [contact_ik_boundary_20260912.json](../../../src/alicia_flexible_grasp_supervisor/tests/fixtures/contact_ik_boundary_20260912.json)。
  逐字段与源JSON比对通过，不把该fixture作为执行授权；它不是完整点云副本。

## 实际时序

1. 对准后先做当前Pose的两个 `execute=False` 严格规划请求，预热独立观察和
   接触planner，均成功，没有位置发布。之后才建立新的源时间窗。
2. 使用已有 `tools/run_fresh_grasp6d_after_alignment.py`，不是老版通用执行器。
   新计划ID `951031cbfec381799b626e75`，源戳1789263157203821897；绑定同源
   审计与exact plan_id后才调用 `/grasp/start`。入场源龄约34.77秒。
3. 18:33:11.969观察选择roll−15°/tilt0°，严格候选检查52/60；18:33:12.532
   开始观察。真正交权同步短暂拒绝中间异字命令，最终基线准入成功。
4. 观察主轨迹名义11.296秒，最大关节位移0.565rad，峰值0.05rad/s，保留观察
   原轴限速。18:33:26.208严格执行成功；27.120静定结束。固定目标摘要
   `c269899bf457bd83ca120dd9fa39fae69c4b3264aacb19e17fe4a246f353bf68`。
5. 实测末端位置误差12.9217mm、姿态2.44°，误差向量约(+4.317,+6.318,−10.412)mm。
   冻结观察CAD净距7.2463858mm通过，目标距离约0.210m，18:33:28.261进入近场。
6. 近场3个独立源帧为1789263210430018186、1789263211168533563、
   1789263211769426345，源跨度1.3394秒。request54本地14个候选中仅2个桌面
   几何候选通过；GraspNet侧7个stage-profile拒绝、5个双侧表面证据拒绝。
   四个最终条目均hard-recheck通过，严格MoveIt可达数为0。
7. 原始pregrasp规划失败；姿态解析后approach直线fraction出现0.125和0.265，
   均远低于0.98。分支检查57或85个种子；一个分支重复求解相差3.1546rad被
   原1e−6rad一致性要求拒绝，未放宽。最后一条目复用了同源确定失败缓存，
   并非四条不同可达路径都被物理执行。
8. 18:34:18.763精确失败，随后释放任务槽；runner exit6、候选计算结束。
   从进入近场到失败约50.5秒，属于候选检查均失败，不是耗尽90秒主预算。

## 新旧问题应分开

- 本次观察后的416个accepted包覆盖45.80秒：Joint2–6跨度0；Joint1波动1count。
  同窗415个SDK包六轴位置完全不变，最后joint_commands在1789263206.116结束。
  没有昨天那种观察后重发actual导致Joint2/3/5再下沉的证据。但本次没有真正
  执行第二条自动轨迹，不能用它宣称全部多阶段接续已实机验证。
- 近场latest registration为REGISTERED，RMSE0.606mm、支撑法向差0.282°、
  平面分离0.331mm。这是到位视图与近场帧之间的证据，**不是**历史跨姿态
  15mm问题已解决，也不能替代远近所有参考关系验证。
- 当前实测12.9217mm执行误差仍进入既有profile：总不确定量15.9217mm，
  approach offset36.8468mm、pregrasp distance54.7685mm、lift45.9217mm。
  不能擅自清空误差、缩短退距或放松门限来“跑完流程”。

## 仅计算IK诊断及改进方向

通过 `/compute_ik`，每套姿态各用13组虚拟初值，超时0.05秒/次；分别打开
和关闭碰撞作对照，均未发执行服务、action或关节命令：

| 每套姿态的端点 | 碰撞开启 | 碰撞关闭（诊断） |
| --- | --- | --- |
| pregrasp | 0/13成功，NO_IK_SOLUTION | 0/13成功，NO_IK_SOLUTION |
| approach | 0/13成功，NO_IK_SOLUTION | 0/13成功，NO_IK_SOLUTION |
| grasp | 13/13成功 | 13/13成功 |

另将第二套grasp解作为连续虚拟起点，沿原退让轴扫描0–60mm、步长2mm：
仅0mm可解，2–60mm均返回−31。没有调整实际关节限位；不能把关闭碰撞的
诊断结果当执行许可。只完成有界搜索，不声称已数学证明全工作空间不可达。

下一修复应优先在近场候选排序前同时证明抓取点、固定接近端点和连续段的
可达性/关节余量，避免“接触几何成立但只能落在孤立关节极限点”的候选耗尽
运动学搜索；补充符合原双侧接触/支撑面/CAD规则的倾斜接触候选与相应观察
证据，而非任意改变已冻结接触姿态。12.9mm实体跟随残差仍需单独解决。
本轮只记录并定位这些新阻断，尚未完成上述候选改造、实机验证或完整抓取。

## 后续源码修复（尚未加载）

操作者随后要求解决本次失败。已复现并修复“有几何合格候选但全部运动学失败
时遗漏其他方向的注册补拍机会”，以及双重夹爪对称展开造成的完整序列重复
检查。真实审计驱动回归保留所有硬门控；没有将本节之前的失败改写为成功。
另用当时 accepted 源帧复算12.9217mm到位残差，位置影响主要来自 Joint3、
Joint2、Joint5，不能只查 Joint2，也不能把 FK 分项当作外部标定或硬件根因证明。
实施、测试、未完成项见
[接触路径恢复修复记录](2026-09-12-contact-path-recovery-repair.md)。
本轮未重启节点或执行新运动，未发布停止/失能命令；完整抓取仍待新现场证据验证。
