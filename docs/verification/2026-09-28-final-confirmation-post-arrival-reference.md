# 最终确认阶段的到达后参考（2026-09-29 UTC）

`GraspTaskNode._confirm_final_center_alignment()` 原先每次开启确认 phase 时只传 `reference_plan`，因此 `_set_near_field_active()` 会回退到 `latest_obj`。该对象可能采自纠正动作到达前；后续 Preview 的时间检查不能反向证明注册参考也来自到达后。

本次仅修改该确认函数：

- 每次确认 phase 复用已有 `_wait_for_final_refine_reference()`，要求参考 source 严格晚于函数开始时刻及纠正计划 source，并保留原接收延迟、最近回调、identity 条件。
- 把通过检查的对象显式传入 `reference_object`；不再依赖 phase 发布时可变化的 `latest_obj`。
- 首次等待、重复 phase、stream 请求及所有确认样本共用初始 monotonic/ROS deadline；phase 得到剩余预算与同一绝对截止时刻。等待不续时。
- 没有更改全局支撑面 4 mm/4°、目标关联条件或任何运动接口。

完整相关回归：292 项通过，3.28 秒；7 条既有 rospy 日志弃用警告。覆盖到达前缓存不可建 phase、参考等待/phase/stream/验证耗尽截止时刻、重复 phase 不延时，以及原确认采样、目标丢失和整个任务序列。

源码 SHA256：`5ec95bc63ef7bd42401aa4f6b8bc1c4e5644ca724125c5be5ba5422ee54ce246`；目标函数 SHA256：`2a8c7e8edd9ec75c96dc442e70b3c8886f45934833571b65d2842a2436a7be6c`。

冻结源码、函数、测试命令与结果归档：
`/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/restart_grasp_20260929T013841Z/post_move_confirmation_reference_patch/`。

该修复尚未由本代理重启加载，部署由主代理负责。它证明并封住一个缓存参考缺口，不构成此前约 11 mm 实体高度偏差的根因结论，也不意味着抓取已成功。
