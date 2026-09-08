# 2026-09-08 实机恢复和观察距离数值边界

03:00按操作者请求恢复上位机。原完整ROS服务、驱动、相机、感知、
MoveIt和339befd规划节点仍在运行；单独启动gui.launch。
发送 `/demonstration=false`，驱动映射为正向torque_on；未发送失能。
03:01实测 motion_enabled=True、CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE，
run_status=0。操作者移动后03:02确认对准。

后台 `alicia-feedback-recording-20260908.service` 持续只读录制关节、
使能、状态、感知与规划指标；128MB分卷、最多4卷。相关数据在根目录
`.ros_log/alignment_20260908_0300/`。

新鲜计划 c655a59c4aabff5700b93d38 通过规划器并提交执行槽，但运行器
错误拒绝 actual_camera_target_distance_m=0.22000000000000006；
配置上限为0.22。该ULP舍入来自刚体变换，未启动运动，随后运行器
等待授权超时。原始绑定audit已保存为 boundary_audit*.json。

修复运行器距离比较，使用1e-12m纯数值容差。配置区间仍严格为
18–22cm。回归覆盖两端ULP误差、两端实际超出1微米、NaN和运行器
推理生命周期：修复前2失败/4通过，修复后6通过。
尚未证明本轮真实持物成功。
