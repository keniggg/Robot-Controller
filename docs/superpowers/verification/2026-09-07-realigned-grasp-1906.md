# 2026-09-07 19:06 重新对准后的执行记录

操作者重新对准后，runner 从源时间戳 `1788833176274600267` 开始
接收新计划，绑定 `1d87a19b61bc0d5a91111a23`，并于 19:06:59 调用
抓取服务。19:08:57 观察位运动完成，实测位置误差 0.0136 m、姿态
误差 2.40 deg；19:08:58 确认相机距目标 0.1929 m。

近场 phase 4 成功建立 405 点参考表面，请求 90 得到处理。
19:09:05 返回 `NEAR_FIELD_NO_HARD_SAFE_CANDIDATE`，抓取服务结果
`success=False`。未执行接近、闭爪或抬升。

拒绝分布：

- 29 个 `GRIPPER_SWEEP_COLLISION`，记录中的几何候选样例均为左/右
  夹指进入支撑面净空区。
- 3 个 `GRIPPER_WIDTH_INVALID`。
- 3 个 `BILATERAL_SURFACE_EVIDENCE_MISSING`。
- 1 个 `GRASPNET_STAGE_PROFILE_UNAVAILABLE`。

本次后续表面配准返回 `TRANSLATION_BOUND_EXCEEDED`：变换平移项
范数 0.0268086 m，超过既有 0.025 m 上限；变换旋转 3.50489 deg。
该数值来自基座坐标系刚体变换的平移项，不能直接作为目标实物位移的
测量结论。支撑面法向差 0.139928 deg，目标局部分离 0.000023119 m。
未接受此次配准，融合表面仍只包含参考视角的 405 个点。

原始审计与任务日志保存在工作区根目录
`.ros_log/grasp_attempt_20260907_1906/nearfield_audit.json` 和
`task_events.log`。本轮仅执行现有流程并保存数据，没有修改运行参数
或控制代码，没有发布机械臂去使能命令。结果为**未抓取成功**。
