# 2026-07-22 6D Preview/Execution 同步修复日志

## 现象

- GUI 生成 6D 候选后，Preview plan_id 和 Execution plan_id 不一致。
- 点击执行时 GUI 显示 `PLAN_SUPERSEDED_BY_PREVIEW`，阻止 `/grasp/start`。
- 旧 Execution 即使曾经可执行，也可能对应较早的候选；新 Preview 才是用户刚看到的候选。
- 一次失败执行日志显示 MoveIt/MuJoCo 预检通过后，真正执行在 `MOVE_PREGRASP` 阶段失败。

## 根因

1. remote 端原先用候选的 `grasp_pose` 做严格 MoveIt 预筛，但执行节点第一步规划的是 6D `pregrasp` 位姿。
2. GUI 已经正确发现 Preview 比 Execution 更新，不能再把旧 Execution 发给 `/grasp/start`。
3. `/grasp_6d/replan_execution` 之前只设置“下一帧 Preview 可提升”的标志；如果候选流已经停止，已有缓存 Preview 不会立刻同步成 Execution。

## 旧记录对照

上传的旧记录里有三个相似解法：

- 曾经把计划有效期临时放宽到 120 秒，解决“按钮点不了/计划过期”的问题；本次已持久化到 `grasp_6d.plan_validity_sec: 120.0`。
- 曾经把 `/grasp_6d/remote/moveit_top_n` 调到 10，让窄边候选进入 MoveIt 检查；本次已持久化到 `grasp_6d.remote.moveit_top_n: 10`。
- 曾经用 `/grasp_6d/replan_execution` 让当前窄边 Preview 获得 execution authority；本次修复把这个服务增强为“候选流停止时也能直接提升缓存 Preview”，不再必须等待未来 1-2 个推理周期。

## 持久化改动

- `scripts/remote_grasp6d_node.py`
  - 严格 MoveIt 预筛改为优先检查 `sequence.pregrasp`，没有 sequence 时才回退 `grasp_pose`。
  - Preview 发布时缓存同一帧的 `plan_id`、promotion `signature`、`score` 和几何 generation。
  - 候选流停止时调用 `/grasp_6d/replan_execution`，会尝试把缓存 Preview 立即提升为 Execution。
  - 缓存 Preview 提升前会把当前 Preview audit 派生为 `.execution` audit，并保持 plan_id 绑定。
  - 几何失效时清掉缓存 Preview proposal，防止旧目标被后续 service 捞起。
- `gui/widgets/grasp6d_control_widget.py`
  - Preview 比 Execution 更新时，执行按钮 fail-closed，显示 `PLAN_SUPERSEDED_BY_PREVIEW`，不会调用 `/grasp/start`。
- `src/alicia_flexible_grasp/grasp/grasp6d_pipeline.py`
  - 显式 replan 请求在 cooldown 后直接允许 `PROMOTE_REPLAN`，不再被 hysteresis 拦住。
- `config/grasp_params.yaml`
  - `grasp.pregrasp_distance_m: 0.04`
  - `grasp_6d.plan_validity_sec: 120.0`
  - `grasp_6d.target_observation_validity_sec: 3.0`
  - `grasp_6d.remote.moveit_top_n: 10`
  - `grasp_6d.remote.gripper_geometry.opening_fit_clearance_each_side_m: 0.0005`

## 自动化验证

```bash
source .worktrees/protocol-v3-upgrade/devel/setup.bash
python3 -m pytest -q \
  .worktrees/protocol-v3-upgrade/src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py \
  .worktrees/protocol-v3-upgrade/src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_pipeline.py \
  .worktrees/protocol-v3-upgrade/src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_control_widget.py
```

结果：`355 passed in 3.44s`

## ROS 运行态验证

- remote 节点已重启，最新 PID：`142486`。
- `/grasp_6d/request_plan trigger=true` 后生成有效候选：
  - Preview/Execution 首次对齐：`c4957c9936d3724fe3aab58f`
- 停止候选流后调用 `/grasp_6d/replan_execution trigger=true`：
  - service 返回：`cached Preview promoted to execution authority`
  - 最终 Preview plan_id：`f8c6e733dbe28df69d7a7b14`
  - 最终 Execution plan_id：`f8c6e733dbe28df69d7a7b14`
  - `ids_match=True`
- execution audit 已写入：
  - path：`/home/zhuyupei/.ros/grasp6d_gate_audit_latest.json.execution`
  - mode：`continuous_execution`
  - valid_plan：`True`
  - promotion_code：`PROMOTE_REPLAN`

本次验证没有调用 `/grasp/start`，未发送机械臂抓取运动指令。

## ROS 重启注意

- 代码已保存到 `.worktrees/protocol-v3-upgrade/src/alicia_flexible_grasp_supervisor/...`，`devel/lib` 的 rosrun wrapper 会加载这里的 Python。
- 参数已保存到 `config/grasp_params.yaml`，完整重启 ROS 后仍应使用 4 cm pregrasp 和 top-10 pregrasp MoveIt 筛选。
- 重启 remote 节点后，应确认 `/grasp_6d/replan_execution` 能在候选流停止且缓存 Preview 更新时同步 Execution plan_id。

## 2026-07-22 04:10 运行中 TARGET_LOST 撤销 authority 修复

### 现象

- 点击执行 6D 抓取后，MuJoCo gate 通过，机械臂已经运动到 6D pregrasp 附近。
- 在接触目标物前，`grasp_task_node` 失败：
  - `EXECUTION_AUTHORITY_REVOKED`
  - `frozen execution authority was revoked by a hard safety event before approach`
- 同一时间 remote 端发布：
  - `TARGET_LOST: target object is not detected`

### 根因

- 执行过程中手爪/机械臂可能短暂遮挡相机视野。
- `remote_grasp6d_node` 在 `robot_execution_active=True` 时仍把一次 `TARGET_LOST` 当成 execution authority tombstone 发布到 `/grasp_6d/plan_enriched`。
- `grasp_task_node` 收到 invalid rich plan 后按 fail-closed 设计撤销冻结计划，于是停在接触前。

### 持久化改动

- `scripts/remote_grasp6d_node.py`
  - `_publish_invalid_plan_pair()` 增加执行中冻结保护。
  - 当 `robot_execution_active=True` 且已有有效 Execution authority 时，几何失效仍更新 `/grasp_6d/object_geometry` 诊断，但不清空 `latest_rich_plan`/`latest_plan`，不 clear `execution_plan_controller`，不向 `/grasp_6d/plan_enriched` 发布 invalid tombstone，不撤销 `.execution` audit。
  - 非执行态的硬失效保持原有 fail-closed 行为。
- `tests/test_remote_grasp6d_streaming.py`
  - 新增回归测试 `test_target_loss_during_robot_execution_keeps_frozen_authority`。
- `launch/full_system.launch`、`launch/bringup.launch`、`real-arm/alicia_d_driver/launch/*.launch`
  - 增加 `self_check_poll_rate_hz` 参数。
  - 按现场要求启动时可使用 `self_check_poll_rate_hz:=0.0`，避免机械臂 self-check 查询；`auto_torque_on_startup:=true` 仍只发送 torque-on，不发送 torque-off。

### 自动化验证

```bash
source devel/setup.bash
python3 -m pytest -q \
  src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py::test_target_loss_during_robot_execution_keeps_frozen_authority \
  src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py::test_blocked_gate_audit_publish_allows_hard_invalidation \
  src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py::test_blocked_execution_publish_allows_hard_invalidation_and_recovers \
  src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py::test_stale_ticket_cannot_revoke_newer_execution_authority
```

结果：`5 passed`。

## 2026-07-22 04:30 当前候选无法生成的准确断点

### 现象

- GUI 点击“生成 6D 候选”后没有得到可执行 Preview/Execution。
- 现场视觉上目标物与机械臂相对位置良好，目标识别持续稳定。

### 当前证据

- `/home/zhuyupei/.ros/grasp6d_gate_audit_latest.json`
  - mode：`continuous_preview`
  - request_id：`17`
  - outcome：`GRIPPER_SWEEP_COLLISION:27`
  - stage_counts：
    - raw：`1024/1024`
    - nms：`64/1024`
    - remote_collision：`32/64`
    - locally_valid：`6/38`
    - stable：tabletop 候选进入稳定窗口
    - moveit_reachable：`0/10`
    - preview：`0`
    - promoted：`0`
  - rejection_counts：
    - `GRIPPER_SWEEP_COLLISION=27`
    - `MOVEIT_UNREACHABLE=10`
    - `GRIPPER_TOO_NARROW=4`
    - `COLLISION=4`
    - `CENTER_OUTSIDE_OBB=1`

### 准确根因

- WSL 端并非没有返回候选；远端推理正常返回 raw 候选。
- GraspNet 候选主要在本地几何/夹爪门控被筛掉。
- 6 个 tabletop 几何候选的几何门控全部通过，开口约 `38-47 mm`，未超过 50mm 夹爪上限。
- 这些 tabletop 候选在 MoveIt/KDL 的完整 6D 姿态检查中全部失败：
  - `MOVEIT_UNREACHABLE`
  - `/compute_ik` 返回 `NO_IK_SOLUTION(-31)`
  - `avoid_collisions=false` 时仍失败，排除碰撞场作为主因。
  - 同一目标位置做 position-only 规划可成功，说明不是位置不可达。
  - 30 个随机 IK seed 仍全部 `NO_IK_SOLUTION(-31)`，说明不是当前 seed 偶然失败。

结论：当前“候选点无法生成”的断点是 **tabletop 候选的严格 tool0 6D 姿态对 Alicia MoveIt/KDL 不可解**。它不是 WSL、目标识别、夹爪宽度或碰撞场问题。历史里的 `moveit_top_n=10` 已保存并生效；新的瓶颈是 tabletop 姿态生成需要产生 IK 可解的姿态备选，而不能只靠位置关系判断可抓。

### 运行态参数备注

- `/grasp_6d/remote/moveit_top_n=10` 已生效。
- `/grasp_6d/target_max_drift_m` 当前仍为 `0.02`，历史中曾临时调到 `0.04` 解决 pregrasp 后视觉漂移门控；这属于执行阶段问题，不是当前候选生成失败的主因。

## 2026-07-22 05:10 现场继续处理：目标被碰歪后的候选失效

### 现象

- 用户反馈刚才可能手动把目标物撞歪。
- 运行态出现 `TARGET_EPOCH_STALE`/`RESULT_EXPIRED`/`PENDING_REPLACED` 一类旧结果失效，属于目标位姿变化后的保护逻辑。
- 现场 audit 中目标仍能识别，bbox 置信度稳定；新 target epoch 下 tabletop 候选可以生成，但旧进程中的 stable hard recheck 曾把部分候选拒为 `GRIPPER_WIDTH_INVALID`。

### 根因和区分

- 目标物被碰歪会让旧 target epoch、旧 Preview/Execution plan 失效；这不是“物体不能抓”，而是防止旧计划继续执行的正常保护。
- 另一个代码层面的真实根因是 stable recheck 的宽度一致性过严：tabletop 候选生成时的融合宽度约 `38-39 mm`，当前点云投影可能变为约 `38 mm`，差值达到 `0.6-1.0 mm` 时会被 `GRIPPER_WIDTH_INVALID` 拒掉。
- 这类差异来自连续点云/目标姿态更新，不应直接判定为夹爪不可用；但如果当前重算宽度超过 50mm 夹爪能力，仍必须拒绝为 `GRIPPER_TOO_NARROW`。

### 持久化改动

- `scripts/remote_grasp6d_node.py`
  - stable tabletop hard recheck 先用候选原始 `required_open_width_m` 做显式几何门控。
  - 若仅因 `GRIPPER_WIDTH_INVALID`/`jaw_width` 失败，则读取当前点云投影重算出的 width，再执行一次显式门控。
  - 通过后把下游计划宽度保存为 `max(候选原始宽度, 当前点云宽度)`，保持保守开口，不放宽 50mm 上限和碰撞/支撑面门控。
- `src/alicia_flexible_grasp/grasp/tabletop_geometry_candidates.py`
  - 倾斜 approach 候选已持久化，`variant_index` 继续只表示夹爪 jaw flip 的 `0/1`，`source_index` 编码 approach tilt 备选，避免触发候选 contract。
- `config/grasp_params.yaml`
  - `grasp_6d.remote.tabletop_geometry_candidates.approach_tilt_degrees: [10.0, 15.0]` 已保存，ROS 重启后仍生效。

### 自动化验证

```bash
source devel/setup.bash
python3 -m py_compile src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py
python3 -m pytest -q \
  src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py \
  src/alicia_flexible_grasp_supervisor/tests/test_tabletop_geometry_candidates.py \
  src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_node.py \
  src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_pipeline.py \
  src/alicia_flexible_grasp_supervisor/tests/test_gripper_geometry.py
```

结果：`581 passed in 18.47s`。

### 运行备注

- 修改已保存到 worktree 源码；`devel/lib/.../remote_grasp6d_node.py` 是指向源码脚本的 relay，重启 `/remote_grasp6d_node` 后生效，不需要重新 catkin build。
- 为了加载宽度刷新修复，需要只重启 `/remote_grasp6d_node`；不重启机械臂 driver，不执行健康检查，不发布 torque-off 或 stop 指令。

## 2026-07-22 05:30 Preview 新于 Execution 时的服务层保护

### 现象

- 最新 runtime 中 Preview 和 Execution 再次出现不同步：
  - Preview：`7ebb5e7af8647435b1b187da`，约 `18s`
  - Execution：`9ec276b2d50e29e77ac7c4a0`，约 `67s`
- GUI 侧会显示 `PLAN_SUPERSEDED_BY_PREVIEW` 并禁用执行按钮，但 `/grasp/start` 服务层原本只检查 requested plan_id 是否等于当前 execution，没有比较最新 Preview 是否已经超越 execution。

### 根因

- `remote_grasp6d_node` 在 continuous preview 下允许新 Preview 不立即 promotion，旧 Execution authority 仍可能保留在 `/grasp_6d/plan_enriched`。
- GUI 有同源拦截，但服务层缺少同样的 Preview-vs-Execution 时间戳/plan_id 防线；如果 UI 状态或订阅时序异常，旧 execution plan 仍有被 start service 接收的风险。

### 持久化改动

- `scripts/grasp_task_node.py`
  - 新增 `_preview_supersedes_execution_locked()`。
  - `_copy_requested_grasp6d_plan()` 在冻结 execution 前检查最新 Preview：若 Preview 新于 Execution 且 plan_id 不同，返回 `PLAN_SUPERSEDED_BY_PREVIEW`，不进入执行。
  - `validate_plan_id_for_execution()` 也增加同样保护，便于后续诊断/调用保持一致。
- `tests/test_grasp_task_sequence.py`
  - 新增回归测试 `test_start_service_rejects_execution_superseded_by_newer_preview`。

### 自动化验证

```bash
source devel/setup.bash
python3 -m py_compile src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py
python3 -m pytest -q \
  src/alicia_flexible_grasp_supervisor/tests/test_grasp_task_sequence.py \
  src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_control_widget.py
```

结果：`119 passed, 4 warnings in 1.58s`。warnings 是已有 `rospy.core.warn` deprecation。

## 2026-07-22 05:40 假 active 导致 Preview 无法 promote

### 现象

- 最新 audit 显示候选链路正常：
  - tabletop stable/hard recheck 通过
  - MoveIt 有 reachable 候选
  - Preview 可生成
- 但 promotion 被拒为：
  - `EXECUTION_FROZEN: robot execution is active`
- 同时 `/grasp/state` 20 秒内没有新消息，说明 remote 仍保留旧的 `active=True` 内存状态。

### 根因

- `remote_grasp6d_node` 的 `robot_execution_active` 只由 `/grasp/state.active` 更新。
- `grasp_task_node` 重启后没有主动发布 `IDLE` 状态，`/grasp/state` 也不是 latched topic。
- 因此如果 remote 曾收到上一轮执行中的 `active=True`，而任务节点随后重启/沉默，remote 会一直认为机械臂仍在执行，拒绝把新 Preview promote 为 Execution。

### 持久化改动

- `scripts/grasp_task_node.py`
  - `/grasp/state` publisher 改为 `latch=True`。
  - `GraspTaskNode` 初始化完成后发布 `IDLE ready` 状态。
  - 这只发布状态诊断，不调用 `/grasp/stop`，不发送机械臂停止/下使能命令。
- `tests/test_grasp_task_sequence.py`
  - 新增 `test_grasp_state_is_latched_and_startup_publishes_idle`。

### 自动化验证

```bash
source devel/setup.bash
python3 -m py_compile src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py
python3 -m pytest -q \
  src/alicia_flexible_grasp_supervisor/tests/test_grasp_task_sequence.py \
  src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_control_widget.py
```

结果：`120 passed, 4 warnings in 2.55s`。warnings 是已有 `rospy.core.warn` deprecation。

## 2026-07-22 05:50 6D pregrasp 接触前停止

### 现象

- 点击执行 6D 抓取后，机械臂运动到目标物接触前停止，上位机显示 `FAILED`。
- GUI 后续显示旧 plan 已过期：`PLAN_STALE`，但实际失败发生在过期前的 `MOVE_PREGRASP`。

### 关键证据

- `grasp_task_node`：
  - `PLAN_PREGRASP MuJoCo simulation passed`
  - `MOVE_PREGRASP moving 6D pregrasp`
  - `FAILED 6D pregrasp failed: execute failed from cached plan (strict pose): target xyz=(-0.146, -0.439, 0.108)`
- `motion_gateway`：
  - `execute_pose_strict request execute=True frame=base_link xyz=(-0.146, -0.439, 0.108) q=(0.734, 0.663, -0.108, 0.102)`
  - `strict cached trajectory controller check passed`
- `move_group`：
  - `Controller 'alicia_controller' failed with error GOAL_TOLERANCE_VIOLATED: Joint2 goal error 0.026106`
  - `ABORTED: CONTROL_FAILED`
- `alicia_d_driver` 反馈显示硬件没有失能，末端动作停止在预抓取附近；没有发布 torque-off/stop 指令。

### 根因

- 本次不是目标物不可抓，也不是 grasp_task 在接触前主动停止。
- MoveIt 规划成功，底层轨迹 action 在终点判定时因为 Joint2 误差 `0.026106 rad` 略超 ros_control 配置的 `0.025 rad` goal 容差而返回 `ABORTED`。
- 对 Alicia-D 串口/平滑实机，这个边界过紧；已有 MoveIt start 容差为 `0.05 rad`、stopped velocity 容差为 `0.03`，因此预抓取终点应使用明确的实机终点复核容差 `0.03 rad`。

### 持久化改动

- `src/real-arm/alicia_d_driver/config/controllers.yaml`
  - `alicia_controller` 六个关节的 `goal` 容差从 `0.025` 调整为 `0.03`。
- `src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml`
  - 新增 `/robot/execution_goal_tolerance_rad: 0.03`。
- `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/moveit_planner.py`
  - 缓存轨迹执行返回失败后，用真实关节反馈和缓存轨迹终点做二次判定。
  - 仅当所有关节最终误差 `<= execution_goal_tolerance_rad` 时，才把这次 cached execution 判为成功；更大误差仍失败。

### 自动化验证

```bash
source devel/setup.bash
python3 -m py_compile \
  src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/moveit_planner.py \
  src/alicia_flexible_grasp_supervisor/scripts/motion_gateway_node.py
python3 -m pytest -q \
  src/alicia_flexible_grasp_supervisor/tests/test_moveit_planner_pose_feedback.py \
  src/alicia_flexible_grasp_supervisor/tests/test_moveit_trajectory_execution_config.py
```

结果：`23 passed in 0.34s`。

## 2026-07-22 05:58 运行 worktree 同步复核

### 现象

- 复核 `/motion_gateway` 后发现当前 ROS 节点加载的是 worktree：
  - `/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade/src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/moveit_planner.py`
- 该运行文件当时尚未包含 `_cached_plan_goal_reached_with_hardware_tolerance` 和 `execution_goal_tolerance_rad`。

### 处理

- 将 05:50 的持久化修复重新落到实际运行 worktree：
  - `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/moveit_planner.py`
  - `src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml`
  - `src/real-arm/alicia_d_driver/config/controllers.yaml`
  - `src/alicia_flexible_grasp_supervisor/tests/test_moveit_planner_pose_feedback.py`
  - `src/alicia_flexible_grasp_supervisor/tests/test_moveit_trajectory_execution_config.py`
- 重启 `/motion_gateway`，新 PID `201484`。
- 导入复核结果：
  - `'_cached_plan_goal_reached_with_hardware_tolerance' in MoveItPlanner == True`
  - `'execution_goal_tolerance_rad' in MoveItPlanner == True`
  - live `/robot/execution_goal_tolerance_rad == 0.03`
  - live `/alicia_controller/constraints/Joint2/goal == 0.03`

### 自动化验证

```bash
source devel/setup.bash
python3 -m py_compile \
  src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/moveit_planner.py \
  src/alicia_flexible_grasp_supervisor/scripts/motion_gateway_node.py
python3 -m pytest -q \
  src/alicia_flexible_grasp_supervisor/tests/test_moveit_planner_pose_feedback.py \
  src/alicia_flexible_grasp_supervisor/tests/test_moveit_trajectory_execution_config.py
```

结果：`26 passed in 0.37s`。

## 2026-07-22 06:10 6D 方向偏离与大关节翻转门限

### 现象

- 用户观察到执行后机械臂停在目标物接触前，且抓取方向看起来偏离。
- 05:29 实际失败发生在 `MOVE_PREGRASP`，预抓取点本来就距离接触点约 `pregrasp_distance_m=0.04m`。
- 05:29 选中候选的插入方向为：
  - `insertion_axis_base=[-0.032, -0.289, -0.957]`
  - 即接近自上而下但带侧向倾斜，pregrasp 会沿该方向反向偏移约 4 cm。
- 06:00 后重新只生成候选时，audit 暴露出另一个真实问题：
  - streaming 选择器可选到 `joint_max_delta=3.254/3.265 rad` 的大翻转候选。
  - 配置已有 `candidate_max_joint_delta_rad: 1.8`，但 `bounded_moveit_select()` 没有执行该硬门限。

### 根因

- “接触前停止”的直接原因仍是 05:29 的 `Joint2 goal error 0.026106` 超过旧 `0.025 rad` 容差。
- “方向偏离/姿态很飘”的筛选层风险是：streaming 选择器只把 MoveIt 运动量计入 soft cost，没有把 `candidate_max_joint_delta_rad` 当硬门拦截。
- 因此可达但需要大幅腕部/关节翻转的候选仍可能进入 Preview/Execution。

### 持久化改动

- `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/grasp6d_pipeline.py`
  - `bounded_moveit_select(..., max_joint_delta_rad=0.0)` 新增硬门限参数。
  - 当 MoveIt 可达但 `joint_max_delta_rad > max_joint_delta_rad` 时，候选不进入 reachable/selected，并记录 `MOVEIT_JOINT_DELTA_LIMIT`。
- `src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py`
  - streaming 调用 `bounded_moveit_select()` 时传入现有配置 `candidate_max_joint_delta_rad`。
- `src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_pipeline.py`
  - 新增回归测试：`3.254 rad` 大翻转候选被拒，`1.730 rad` 候选可胜出。
- `src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py`
  - 更新 fake selector 签名，并断言 remote 会传入 `candidate_max_joint_delta_rad=1.8`。

### 自动化验证

```bash
source devel/setup.bash
python3 -m py_compile \
  src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/grasp6d_pipeline.py \
  src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py
python3 -m pytest -q \
  src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_pipeline.py \
  src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py
```

结果：`329 passed in 9.25s`。

### ROS 实机验证

- 重启 `remote_grasp6d_node`，新 PID `208407`。
- live 参数：
  - `/grasp_6d/remote/candidate_max_joint_delta_rad == 1.8`
  - `/grasp_6d/remote/tabletop_geometry_candidates/approach_tilt_degrees == [10.0, 15.0]`
- 只触发 `/grasp_6d/request_plan`，未调用 `/grasp/start`，机械臂未执行运动。
- 最新 preview/execution plan 均为 `c0489cf65857f4a26852ac7a`。
- 最新 audit：
  - `MOVEIT_JOINT_DELTA_LIMIT` 已出现，`joint_max_delta 3.269rad > 1.800rad` 与 `3.265rad > 1.800rad` 的候选被拒。
  - selected candidate：`tabletop_geometry source_index=0 variant_index=1`
  - selected MoveIt：`joint_max_delta_rad=1.628 <= 1.8`，`joint_path_cost=2.115`
  - pregrasp pose：`(-0.183, -0.454, 0.120)`，contact/grasp pose：`(-0.188, -0.468, 0.083)`

## 2026-07-22 19:12 6D pregrasp 后目标漂移门限复现修正

### 现象

- 重新启动最新 ROS 节点后，用户对准目标并执行 6D 抓取。
- Preview/Execution 已同步为同一个 plan：`4bef1c2fb76cf8e4157f7f07`。
- MuJoCo 数字孪生通过，MoveIt 严格预抓取轨迹执行成功。
- 抓取流程在最终接近前失败：
  - `TARGET_DRIFT: live target drift 0.029m exceeds 0.020m before approach`

### 根因

- 这与上传历史记录中的 `TARGET_DRIFT 0.027m > 0.020m before approach` 是同类问题。
- 6D 富计划会在 pregrasp 到位后再次用眼在手相机复核目标中心。
- 近距离视角变化和实例分割抖动会让静止纸盒中心估计出现约 25-30 mm 偏移。
- 原 `target_max_drift_m=0.02` 对该阶段过紧，导致还没进入接触 approach 就被漂移保护中止。

### 持久化改动

- `src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml`
  - 将 `grasp_6d.target_max_drift_m` 从 `0.02` 调整为 `0.04`。
  - 保留漂移保护，不关闭门控。
  - 不改变 MuJoCo、MoveIt、候选选择、执行 plan 绑定、宽度或碰撞门控。

### 运行时同步

- 已同步设置运行时 ROS 参数：
  - `/grasp_6d/target_max_drift_m = 0.04`
  - `/grasp/target_max_drift_m = 0.04`（当前进程运行时覆盖；重启后由 `/grasp_6d/target_max_drift_m` 持久配置兜底）

### 自动化验证

```bash
source devel/setup.bash
python3 -m pytest -q src/alicia_flexible_grasp_supervisor/tests/test_grasp_task_sequence.py
```

结果：`91 passed, 4 warnings in 2.01s`。

## 2026-07-22 19:25 6D pregrasp 控制器目标容差边界修正

### 现象

- 用户重新对准目标并执行同步后的 6D plan：`cdd2945001527804cea12d48`。
- MuJoCo 数字孪生通过，严格 6D pregrasp 规划通过。
- 机械臂运动到接触前位置附近后，抓取流程停在 `FAILED`：
  - `6D pregrasp failed: execute failed from cached plan (strict pose): target xyz=(-0.191, -0.415, 0.111)`

### 根因

- MoveIt/ros_control 给出精确失败：
  - `GOAL_TOLERANCE_VIOLATED: Joint3 goal error 0.030005`
  - `ABORTED: CONTROL_FAILED`
- `/alicia_controller/state` 的实际误差为：
  - Joint2 `0.028009 rad`
  - Joint3 `0.0300047 rad`
- 这不是目标物不能抓，也不是候选点没生成；是硬件反馈量化/收敛边界比原 `0.030 rad` 容差多约 `4.7e-6 rad`，导致已经基本到位的 pregrasp 被严格判失败。
- 当时 `/alicia_d/motion_enabled=True`、`/alicia_d/feedback_ready=True`，没有失能。

### 持久化改动

- `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/moveit_planner.py`
  - 新增 `/robot/execution_goal_tolerance_slack_rad`，默认 `0.005`。
  - controller 返回执行失败时，如果最终关节反馈在 `execution_goal_tolerance_rad + execution_goal_tolerance_slack_rad` 内，则按到位成功处理，并在 message 中记录 `including slack`、`max_error` 和有效容差。
- `src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml`
  - 保存 `robot.execution_goal_tolerance_slack_rad: 0.005`。
  - 保留主容差 `robot.execution_goal_tolerance_rad: 0.03`。
- `src/real-arm/alicia_d_driver/config/controllers.yaml`
  - 将 Joint1..Joint6 的 controller `goal` 容差从 `0.03` 保存为 `0.035`，下次 ROS 全量启动后不再在同一量化边界 abort。
- `src/alicia_flexible_grasp_supervisor/tests/test_moveit_planner_pose_feedback.py`
  - 新增复现实机 `0.0300047 rad` 边界的回归测试。
- `src/alicia_flexible_grasp_supervisor/tests/test_moveit_trajectory_execution_config.py`
  - 新增配置持久化测试，确保 supervisor 有效容差和 controller goal 容差均覆盖 `0.035 rad` 边界。

### 自动化验证

```bash
source devel/setup.bash
python3 -m pytest -q \
  src/alicia_flexible_grasp_supervisor/tests/test_moveit_planner_pose_feedback.py \
  src/alicia_flexible_grasp_supervisor/tests/test_moveit_trajectory_execution_config.py
```

结果：`28 passed in 0.74s`。

```bash
source devel/setup.bash
python3 -m py_compile \
  src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/moveit_planner.py
python3 -m pytest -q \
  src/alicia_flexible_grasp_supervisor/tests/test_grasp_task_sequence.py
```

结果：`91 passed, 4 warnings in 2.21s`。

### 运行时状态

- 当前 ROS 端只读确认：
  - `/alicia_d/motion_enabled=True`
  - `/alicia_d/feedback_ready=True`
  - `/grasp/state=FAILED`，为上一轮失败残留状态。
- 未发布 `/grasp/stop`、未发布任何机械臂失能/停止使能命令。
- 由于用户重新移动过机械臂/目标，旧 plan `cdd2945001527804cea12d48` 已不可继续执行；修复生效后必须重新生成 6D 候选，并重新检查 Preview/Execution 同步。

## 2026-07-22 20:10 cached Preview audit 覆盖修正

### 现象

- 用户重新对准目标后，remote 6D 连续推理曾生成有效 Preview，但显式同步 Execution 时失败：
  - `PLAN_AUDIT_NOT_READY: cached Preview audit does not match plan_id`
- 现场审计文件显示：
  - `grasp6d_gate_audit_latest.json` 已被后一帧无选中候选覆盖，`plan_id=""`。
  - `.execution` 仍是旧 plan，例如 `61c603209ada9166d1a67e84`。
- 这会让 GUI 显示 Preview/Execution 不一致，执行按钮被正确拦住。

### 根因

- `latest_preview_rich_plan` 和 `_latest_preview_proposal` 会缓存最新有效 Preview。
- 但用于把 cached Preview 提升为 Execution 的 `_active_gate_audit_report` 会被下一次失败/空候选推理覆盖。
- 因此 plan 本体还在，plan 对应的 audit 证明已经被别的请求替换，导致显式 replan 无法原子提升同一个 Preview。

### 持久化改动

- `src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py`
  - 新增 `_latest_preview_gate_audit_report`，只在当前有效 Preview 的 `plan_id`、proposal 和 selected lineage 匹配时缓存。
  - `_write_cached_preview_execution_audit()` 优先使用绑定到 cached Preview 的 audit，再回退到当前 active audit。
  - 后续失败/空候选推理仍可更新 active audit 和诊断，但不会覆盖上一帧有效 Preview 的可提升凭证。
  - 几何硬失效时清理 `_latest_preview_gate_audit_report`，避免 stale Preview 被提升。
- `src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py`
  - 新增 `test_cached_preview_promotion_uses_bound_audit_after_active_audit_overwrite`，复现后一帧空 audit 覆盖 active audit 的场景。
  - 将 cached Preview audit fixture 抽成 `cached_preview_audit_report()`，覆盖普通 promotion 和覆盖后 promotion 两条路径。

### 自动化验证

```bash
source devel/setup.bash
python3 -m pytest -q \
  src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py::test_cached_preview_promotion_uses_bound_audit_after_active_audit_overwrite
```

RED 阶段结果：`cached Preview audit does not match plan_id`。

GREEN 阶段结果：`1 passed in 0.47s`。

```bash
source devel/setup.bash
python3 -m pytest -q src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py
```

结果：`166 passed in 1.65s`。

```bash
source devel/setup.bash
python3 -m py_compile src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py
```

结果：通过。

### 运行约束

- 修复和验证期间未发布 `/grasp/start`。
- 未发布 `/grasp/stop`，未发布机械臂 stop/disable/torque_off。
- 当前 ROS master 已不通，后续需要从当前 worktree 重新启动 ROS 端节点；启动参数必须继续保持 `auto_torque_on_startup:=true`、`self_check_poll_rate_hz:=0.0`。
