# 2026-09-10 ROS恢复及桌面拟合抽样优化

操作者要求恢复完整最新ROS和上位机、直接关节使能，随后由操作者重新
对准并确认，再执行6D抓取。9月8日的对准不作为本轮抓取触发。

01:07以9de3bd5启动 `alicia-ros-live-20260910.service`，工作目录为
`.worktrees/protocol-v3-upgrade`，执行：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch alicia_flexible_grasp_supervisor full_system.launch \
  start_real_arm:=true driver_port:=/dev/alicia_arm driver_baudrate:=1000000 \
  auto_torque_on_startup:=true self_check_poll_rate_hz:=0 \
  start_camera:=true start_tactile:=false start_gui:=false \
  use_remote_grasp6d:=true remote_grasp6d_url:=http://172.23.132.97:8000
```

单独的 `alicia-gui-live-20260910.service` 运行gui.launch，DISPLAY=:0、
XAUTHORITY=/run/user/1000/gdm/Xauthority。真实驱动、硬件接口、MoveIt、
motion_gateway、相机、感知、手眼TF、夹爪、抓取任务、规划器及日志节点
已启动；01:07:35 WSL返回backend=graspnet_baseline、loaded=True、protocol=3。

驱动启动请求正向torque_on，随后按操作者要求发布一次demonstration=false。
初始状态为PENDING:POSITIVE_ENABLE_REQUESTED；01:09实测变为
motion_enabled=True、CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE。
操作者开始移动，01:11关节约[-1.910,0.328,-0.210,0.003,-0.183,0.005]rad。
该时任务IDLE。短暂E1/E2和温度跳变拒绝均保留在日志中；未发布失能命令。

`alicia-feedback-recording-20260910.service`持续只读录制joint_states、
motion_enabled、run_status、actuation_status、grasp/state、near_field_phase、
grasp_6d/status、pipeline_metrics及perception/object，128MB分卷最多4卷。
文件位于根目录 `.ros_log/alignment_20260910/`。主启动输出为
`.ros_log/ros_live_20260910.log`，上位机输出为`.ros_log/gui_live_20260910.log`，
本轮ROS日志目录为`.ros_log/ac82f1b4-acee-11f1-8f6a-07de1f942d07/`。

## 消除每个平面假设的全点云随机重排

复盘9月8日最后一轮：远场准备仍出现约6秒的几何计算，运行器以规划服务
无响应异常退出，未进入抓取运动；没有证据证明9de3bd5已解决全部实机超时。

使用已保存的415/408点RGB-D数据分析，本地桌面RANSAC为每个假设调用
RandomState.choice(n,3,replace=False)，每次重排全部上下文点。两帧1024次
choice累计1.556秒。改为从n、n-1、n-2三个区间抽整数，再映射跳过占用索引；
每个有序三元组仍严格等概率，计算量只随假设数增长。保留固定种子、
96–512次假设、全部点残差评分、原4mm阈值、内点比例和两轮SVD精修。
随机假设序列改变，不宣称与旧算法逐值相同。

118项几何/多视角回归和310项流式回归通过。新回归穷举3、4、7点的压缩
索引组合证明无偏映射，拒绝不足3点，并检验带大量离群点的平面拟合及
可复现性。同一两帧离线回放耗时2.956/2.625秒变为1.569/0.862秒；上下文
存在并发负载，时间仅作诊断。保留目标点数415/408不变，中心最大分量差
0.000284/0.001645mm，尺寸最大分量差0.000334/0.002406mm。精确输出及
回放脚本保存于`.ros_log/alignment_20260910/geometry_replay/`。

这不是物理抓取成功证据；需在本轮对准确认后使用新鲜绑定计划验证。

01:14仅退出原full_system中的remote_grasp6d_node（无respawn），由
`alicia-remote-grasp6d-20260910.service`单独加载5ae00e8。该服务在同一
工作目录source ROS/devel后执行rosrun，节点日志为本轮目录下
`remote_grasp6d_repaired.log`，stdout为根目录
`.ros_log/remote_grasp6d_repaired_20260910.log`。主ROS、驱动、控制器、
上位机和录制服务持续运行。后续规划修复可在任务空闲时单独重载此服务。
截至本次保存，识别carton、使能已确认、任务IDLE，仍等待本轮操作者
“已对准”消息；未触发自动抓取。确认后使用
`tools/run_fresh_grasp6d_after_alignment.py`建立本轮新鲜快照、审计和计划绑定。

## 本轮对准后的第一次实机执行及腕部排序

操作者随后确认“已对准”。远场8e201800ecfa37501feda884于01:17:22启动，
严格观察轨迹10.477秒，到位误差8.7mm/1.89度；01:17:36.603进入近场阶段1，
实测相机距离19.21cm。近场请求49返回870253e74e50101fb0e4db5c，结果年龄
6.818秒，几何/配准1.159秒、候选1.067秒；431 inliers、84.18%重叠、
0.599mm RMSE、0.298mm最大点修正、0.078度。旧任务48及时取消。

近场四段严格规划通过，但任务仍在01:18:36超时：移动前的远场与当前近场
目标局部桌面差5.388mm，超过4mm；到位后参考观测与近场差为0.743mm。
两个时段的参照不同。该轮只完成观察移动，没有闭爪或抬升。完整反馈、
图像和两份绑定审计在`.ros_log/grasp_attempt_20260910_uniform_sampling/`。
后续重试在当前已到观察位重建新鲜远场基准，保留桌面连续性门。

审计中等价平行夹爪变体的几何余量仅存在浮点尾差，排序却把125.58度
的腕部修正排在64.28度前，已选序列的硬件时长下界105.788秒。将直接近场
排序的宽度键量化为ROS计划实际float32精度，再比较姿态变化；原始宽度
仍用于全部物理门。新增1e-17m宽度尾差回归在修复前失败、修复后通过，
1微米真实差及原40/48mm窄边优先规则保留。流式312项通过（30.50秒）。
这只修正候选顺序，实际更短轨迹及持物成功仍待下一轮实测。

## 当前观察位重建基准后的结果

316b5f0重试计划9691b7488fb8993cd9ac0c98于01:23:38接纳，复用19.19cm
观察位，无新增观察运动。01:23:41.081进入近场阶段2，01:23:47.172返回
NEAR_FIELD_NO_HARD_SAFE_CANDIDATE，没有闭爪或抬升。请求54配准成功：
462 inliers、91.49%重叠、0.471mm RMSE、最大点修正0.120mm，局部桌面
差0.185mm。当前4个有双侧证据的proposal在110/115/120/150度，单侧伸达
25.79–30.77mm，超过25mm物理界限。证据保存在
`.ros_log/grasp_attempt_20260910_reached_baseline/near_audit.json`及同目录
far_audit.execution.json、feedback.jsonl与图像。

## 实测点云抽稀导致窄边证据丢失

后续只读采集两组静止RGB-D窗口（stamp为1789028944102307081和
1789028947010818243ns），并对同一输入分别用2.5mm与1.5mm体素提取。
原配置目标点数507/498，融合556点，13个proposal但无物理通过候选；
1.5mm保留1272/1244点、融合1466点，17个proposal中6个候选通过完整
局部解析接触/碰撞门，分属95/100/105度及其对称姿态，所需开口
42.72–43.21mm。原本12点/侧、2mm接触、3mm桌面净空、50mm开口及配准
界限均保留；没有点云插值、复制点或补造遮挡面。离线注册到候选完成
0.286秒与2.162秒，仅作这一输入的性能诊断。

新增实测深度fixture和回归用例，重新执行几何提取、配准、候选生成及
解析门。原配置先出现assert accepted失败；将grasp_params.yaml中的
target_cloud_voxel_size_m改为0.0015后通过。fixture不包含RGB照片；
来源与SHA256记录在同名Markdown中。这里只证明离线候选保留，尚未
验证新分辨率下的实机MoveIt、最终视觉修正或持物成功。
最终streaming、object_geometry、multiview_surface、grasp_task_sequence
四组共629项通过（47.16秒），7条警告均为rospy调用已弃用warn接口的
DeprecationWarning。git diff --check通过。后台bag文件在两次本地读取中
从44,900,352增长到45,760,842字节，说明录制仍有写入；这不等同于
被审批拦住后的实时关节状态查询或模型持续监督。

## 01:48自动审批额度阻塞

操作者要求继续后，实时ROS只读命令被自动审批拒绝，理由为
“Automatic approval review failed: You've hit your usage limit”，并明确
禁止绕过。提示可购买额度或在6:06 AM后重试；该时间是审批返回文本，
不推断其时区。后续只完成可在工作区进行的离线修改与测试。

当前运行规划版本仍为316b5f0、体素仍为2.5mm。1.5mm配置、实测fixture、
回归及本记录已保存到工作树，尚未Git提交、尚未部署或实机重试。
审批恢复后先提交所有修改，再在任务空闲时仅设置
`/grasp_6d/remote/target_cloud_voxel_size_m=0.0015`并重启
`alicia-remote-grasp6d-20260910.service`，由新的运行器生成新鲜计划，
继续已授权的抓取和数据监督。不要重启驱动或发布失能命令。
