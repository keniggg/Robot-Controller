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
