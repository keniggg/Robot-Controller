# 2026-09-07 近距离实测表面交接修复

## 现场失败与根因

02:11 启动的计划 `fff56c29edff00bb55fb054b` 到达观察位，相机到目标
0.1918 m；02:12:25 在闭爪前以 `NEAR_FIELD_NO_HARD_SAFE_CANDIDATE` 结束。
本次审计 `multiview_surface.active=false`，tabletop 候选数为 0，出现
`BILATERAL_SURFACE_EVIDENCE_MISSING`；其他 GraspNet 候选另有
`GRASPNET_STAGE_PROFILE_UNAVAILABLE`。因此不能把本次故障归因于使能。

两条软件交接缺陷可离线复现：

1. 自动 runner 在调用 `/grasp/start` 前关闭推理流，`stop_streaming()`
   改变目标 epoch。稍后近距离阶段使用计划中的旧 track ID，无法匹配
   当前目标；启动新推理流又会产生新 generation/identity。
2. 任务发出的到达观察位参考时间戳来自实时感知，而节点只从最后一次
   推理快照读取 `_latest_target_observation`。该快照通常早于机械臂运动，
   不可能满足精确时间戳匹配。日志两轮均出现
   `Near-field phase has no exact reached-view measured surface`。

## 修复范围

- runner 保持推理流运行直至同步抓取服务返回；执行前继续核对精确 plan ID。
  目标/阶段管理仍由原有节点完成。
- 同步缓存新增指定纳秒时间戳、指定 target identity 的只读取样，沿用
  帧对应、有效 mask、推理延迟和完成后时效门；不回退到最新帧。
- 阶段切换时，优先使用已有的精确实测 observation；否则从缓存指定帧及其
  snapshot-time TF 重建 reference。只以该单帧建立实测参考，后续接触规划
  仍使用原有稳定窗口、配准、双侧接触、CAD、MoveIt 和 MuJoCo 条件。
- 等待缓存组件期间不持有 geometry lock；重建期间 target、phase 或
  generation 改变时不安装参考表面。
- 未修改夹爪、碰撞、倾角或配准阈值，未发送机械臂停止/取消使能命令。

## 本次验证

新增测试初次运行：5 failed，分别复现缺少精确取样、无法建立到达观察位
表面、runner 在服务调用前已改变目标生命周期。修复后 focused 5 passed；
随后补充 target/stop/phase 并发变化以及未来帧、延迟、frame ID 不匹配回归。

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider \
  src/alicia_flexible_grasp_supervisor/tests/test_rgbd_snapshot.py \
  src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py \
  src/alicia_flexible_grasp_supervisor/tests/test_fresh_grasp_runner.py \
  src/alicia_flexible_grasp_supervisor/tests/test_grasp_task_sequence.py \
  src/alicia_flexible_grasp_supervisor/tests/test_multiview_surface.py \
  src/alicia_flexible_grasp_supervisor/tests/test_gripper_geometry.py \
  src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_node.py
```

结果：**899 passed / 7 warnings，exit 0，37.58 s**。警告来自既有
`rospy.logwarn` 的 deprecated `warn` 调用。`git diff --check` 通过。

真实 ROS 只读验证使用独立内存对象，无控制发布器、无服务调用、无推理线程。
它订阅现有相机、目标、joint states 和 TF，执行修复后的参考帧重建及配准：

- 第一组成功建立 463 点实测表面；后续配准位移 43.70 mm 超过原有
  25 mm 上限，正确返回 `TRANSLATION_BOUND_EXCEEDED`，未加入融合表面。
- 第二组成功建立 507 点参考表面；后续帧 `REGISTERED`，融合后 578 点、
  2 个来源时间戳，399 内点，overlap 0.8788546，RMSE 0.0009220 m，
  translation 0.0008328 m，rotation 0.1152742 deg。
- 第二组 source stamps：1788773175297109365、1788773177819526433。
- 原始结果保存于工作区根目录
  `.ros_log/grasp_attempt_20260907_0212/repaired_surface_readonly.json`。

这些证据证明两处交接缺陷得到修复并可在真实传感器数据上建立、注册表面。
尚未用修复代码重新进行真机闭爪/抬升；实物接触与抓取成功仍需操作者重新
对准后的一次执行验证，不能从只读配准结果推断。
