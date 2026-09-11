# 2026-09-11 ROS恢复及近场观察恢复

操作者要求按日志启动最新完整ROS和上位机，直接正向使能，由操作者
手动对准并再次确认后才抓取。9月10日的对准不作为本轮触发。

00:08从`.worktrees/protocol-v3-upgrade`启动：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch alicia_flexible_grasp_supervisor full_system.launch \
  start_real_arm:=true driver_port:=/dev/alicia_arm driver_baudrate:=1000000 \
  auto_torque_on_startup:=true self_check_poll_rate_hz:=0 \
  start_camera:=true start_tactile:=false start_gui:=false \
  use_remote_grasp6d:=true remote_grasp6d_url:=http://172.23.132.97:8000
```

systemd用户服务`alicia-ros-live-20260911.service`保持独立于终端运行，
输出根目录`.ros_log/ros_live_20260911.log`。启动加载f0c3cd7及工作区
已通过针对性测试的30秒TF缓存修复；尚未加载随后完成的观察恢复。
启动时driver、硬件接口、MoveIt、motion_gateway、相机、感知、手眼TF、
夹爪、task、remote、日志等16个节点（含GUI/录制/rosout）已可见。

`alicia-gui-live-20260911.service`独立启动gui.launch，DISPLAY=:0，
XAUTHORITY=/run/user/1000/gdm/Xauthority，输出`.ros_log/gui_live_20260911.log`。
`alicia-feedback-recording-20260911.service`持续只读rosbag，128MB分卷、
最多4卷、buffsize32，主题与前一天相同，目录`.ros_log/alignment_20260911/`。
本轮ROS日志目录为`.ros_log/992b6aa8-adaf-11f1-ae72-33454ad18274/`。

串口/dev/alicia_arm指向ttyACM1，未重绑设备。驱动启动正向torque_on，
随后发布一次`/demonstration false`再次请求正向使能。00:18实测仍为
PENDING:POSITIVE_ENABLE_REQUESTED、motion_enabled=false，关节持续静止，
不能把成功写入使能命令当成已确认电机运动。状态IDLE，识别carton。
WSL /health返回loaded=true、GraspNet baseline、protocol3，MuJoCo就绪。
本轮未发送失能、停止服务调用或自动抓取命令。

## 修复范围

上轮失败表明：近场单窗口没有双侧接触证据时直接终止；既有clear-view
观察恢复仅接在最终视觉修正阶段，初始近场未接入。新增
NEAR_FIELD_SURFACE_VIEW_REQUIRED只在当前同目标、同时间戳且成功配准，
桌面proposal因缺少双侧实测接触而失败时产生。碰撞失败、配准失败、
缺失/过期证据等继续使用原终止路径。

任务只从冻结远场观察计划恢复一次，调用既有双方向严格MoveIt预规划、
按硬件时长/路径排序、必要时重规划获胜轨迹、执行前绑定校验和到位等待。
初始观察已在18–22cm范围，恢复最大横移40mm、径向退距0，并为22cm
相机范围预留5mm到位误差；没有可用空间即不运动。物理夹爪/CAD外包络
和80mm非接触边界保留。执行前要求原近场预算足以覆盖轨迹时长下界
及0.9秒到位等待，恢复不会延长原近场截止时间。

移动后先等待更新的同目标观测，再发布明确的新阶段，取消旧请求并
保留目标身份。仅在缺失接触的恢复标记仍绑定原生命周期时，把新实测
视角注册到移动前表面；所有原配准界限保留。常规最终视觉修正和新任务
仍按原规则建立新阶段。新视角过期、目标变更、配准失败或延长截止时间
均不能获得接触执行权。再次失败也不再移动。

TF缓存改为30秒，真实TF历史淘汰回归先红后绿，并验证延迟12.614秒的
快照取到当时的插值变换，而非最新位置。另修复原语义无关性测试误将
每次自然不同的耗时诊断要求逐值相等的问题；仍比较所有非计时诊断，
并要求各计时非负且有限。

本记录中的软件验证不代表实际持物成功；实机恢复、闭爪、抬升仍需本轮
对准后的新鲜绑定计划验证。

## 验证及检测表述更正

最终6组相关回归779项通过（67.93秒），7条警告均为rospy的warn弃用
提示。覆盖真实TF淘汰、恢复只执行一次、错误目标/过期视角拒绝、移动
预算和相机范围、移动前后实测点保留、正常最终视觉修正的新阶段规则。
git diff --check通过。

操作者指出相机未对准carton。此前“识别到carton”只是转述检测节点，
没有核对画面，不能当成找到了操作者目标。00:19日志检测框约
(594,212–215,45,25–28)，位于画面最右边；原模型的类别输出仍需实测
核验，不将其视为对准依据。00:22连续三帧ObjectPose明确detected=false、
confidence=0；label=carton来自perception_node发布前预填的object_label。
同次当前RGB画面朝向地面/椅子和大型开口纸箱，并未对准此前的小纸盒。
图像戳1789111342287551641ns，保存在
`.ros_log/alignment_20260911/current_camera.jpg`。未触发抓取。

## 00:24加载已提交修复

代码提交063bbbd后，读取任务IDLE、active=false，仅退出原full_system中
无respawn的grasp_task_node和remote_grasp6d_node，未重启驱动或控制器。
独立用户服务`alicia-grasp-task-20260911.service`及
`alicia-remote-grasp6d-20260911.service`从同一工作目录source ROS/devel后
分别rosrun对应脚本，环境ALICIA_CODE_REV=063bbbd，ROS_MASTER_URI为
http://localhost:11311，ROS_LOG_DIR为根目录`.ros_log`。

stdout分别为`.ros_log/grasp_task_repaired_20260911.log`和
`.ros_log/remote_grasp6d_repaired_20260911.log`；节点日志在本轮UUID目录下
`grasp_task_repaired.log`、`remote_grasp6d_repaired.log`。
00:25五个用户服务均active，remote确认WSL online。后台录制已增长至15MB。
仍未收到本轮对准确认、未执行抓取，等待操作者手动对准。
