# 2026-09-29 卡顿问题离线收尾

按用户最新指令“如果需要实际RGB-D图像那就直接结束”，本轮停在实机验证之前。用户已报告关闭机械臂电源。代码和离线测试已完成保存；**底层卡顿尚未经过新修复的实机验证，不能认定已解决**。没有新采真实RGB-D、发送使能/失能/运动命令、重启ROS节点或关闭主机。

## 驱动修复

三个持锁路径内的夹爪行程同步参数查询改为读取共享原子快照。后台独立线程直接使用私有有界XML-RPC连接，读取结束后等待1秒，单次RPC截止250ms，不占驱动回调线程、数据锁或ROS参数全局锁。初始化在启动驱动timer前完成，stop/析构取消等待并回收线程。动态修改、删除/缺失、类型错误及失败默认0.05语义保留；数值不新增夹紧。

参数更新现在是采样传播，包含读取周期和调度延迟；一般主机名DNS仍只在构造期解析，不属于运行时硬实时保证。这移除了已确认的阻塞机会，尚未证明实际589ms缺口由参数RPC造成。

隔离编译成功，没有覆盖活动devel二进制。候选位于`.runtime/gripper_stroke_snapshot_repair/20260929T145258Z/build/devel/lib/alicia_d_driver/alicia_d_driver_node`。源码、备份、构建产物留在工作树；测试/增量差异/SHA256在外盘`ros_log/gripper_stroke_snapshot_repair_20260929T145258Z/`。

验证：新增假master4项、SDK证据15项、actuation124项、Python驱动契约49项，合计192项通过。覆盖动态修改/删除/重建、类型/RPC错误、IPv4/IPv6/路径、拒绝连接、接收但不响应的master、在飞行请求中的stop及析构。全部为离线或loopback模拟服务，没有连接现场ROS master或串口。新驱动未部署。

## 相机追踪与内存审核

`tools/diagnostics/camera_io_trace.c`及其测试、README已保存。Linux x86_64的进程期LD_PRELOAD原型透明记录select/pselect/V4L2 buffer事件，不读像素。热路径只写固定内存队列，慢writer时丢追踪事件并计数；真实调用的返回和errno保持。NODELETE和生命周期限制见README。14项离线检查通过，产物`.runtime/camera_io_trace/libcamera_io_trace.so`尚未加载到相机。

内存审核没有确认THP、compaction或VMware是根因；旧5秒超时前4.243秒缺同步资源记录。监测器`.runtime/passive_vm_memory_monitor.py`仅保存并通过语法检查，没有启动；未调内存/USB/虚拟机参数。相机此前5秒SDK等待、错配与反馈延迟仍需后续实机证据。

## 保存与结束条件

外盘本轮汇总：`ros_log/stall_offline_closeout_20260929T145947Z/closeout.json`，包含源码哈希、测试计数、候选二进制与未完成实机验证的边界。全部历史实验保留，local_only，不上传、不删除。最终`/tmp/codex_done`仅表示按用户最新停止条件完成离线收尾，不表示硬件卡顿已治愈。
