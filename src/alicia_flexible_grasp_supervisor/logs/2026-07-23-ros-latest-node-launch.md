# 2026-07-23 ROS latest-node launch log

## Logging rule

- Any later code change, launch-path correction, ROS parameter change, or node restart that affects the 6D grasp workflow must be recorded in this logs directory.
- Record the exact workspace/setup path, command or file changed, reason, and observed result.
- Do not publish stop, torque-off, disable, or `/grasp/stop` commands unless the user explicitly asks for them.

## 2026-07-23 02:42 PDT - latest workspace path correction

Reason:
- The user pointed out that ROS nodes had been launched from the wrong, older workspace.
- Existing conversation/problem logs reference the latest protocol-v3 worktree and patches under `.worktrees/protocol-v3-upgrade`.

Correct ROS setup path:

```bash
source /home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade/devel/setup.bash
```

Correct full-system launch command used:

```bash
roslaunch alicia_flexible_grasp_supervisor full_system.launch start_real_arm:=true driver_port:=/dev/alicia_arm driver_baudrate:=1000000 auto_torque_on_startup:=true self_check_poll_rate_hz:=0.0 start_camera:=true start_tactile:=false start_gui:=true use_remote_grasp6d:=true remote_grasp6d_url:=http://172.23.132.97:8000
```

Observed process paths:
- `alicia_d_driver_node`, `bessica_d_hw_interface`, `motion_gateway`, `supervisor_camera`, `perception_node`, `handeye_transform`, `remote_grasp6d_node`, `compliant_gripper_controller`, `grasp_task_node`, `safety_monitor_node`, `data_logger_node`, and `main_gui.py` are running from `/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade/devel/...`.
- `tactile_skin` is not running. `start_tactile:=false` was used.

Observed runtime status:
- `/joint_states`: about 59 Hz after hardware feedback recovered.
- `/supervisor/camera/color/image_raw`: about 15 Hz.
- `/supervisor/camera/depth/image_raw`: about 15 Hz.
- `supervisor_camera` publishes real camera frames.
- `perception_node` is detecting `carton` continuously.
- `remote_grasp6d_node` reports the remote v3 6D server online at `http://172.23.132.97:8000`.
- The driver initially rejected all-zero encoder feedback, then recovered real SDK feedback and accepted `/joint_commands`.

Current supervision note:
- Continue monitoring terminal output.
- Wait for the user to manually align the object and say "aligned" / "yi dui zhun" before instructing them to generate 6D candidates.

## 2026-07-23 17:25 PDT - latest full-system restart after WSL side started

Reason:
- Continue the interrupted 6D grasp workflow from the latest protocol-v3 ROS worktree.
- Start the ROS-side nodes directly from the log-documented path with real arm enabled on startup.
- Keep electronic skin disabled per operator request.

Setup path:

```bash
source /home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade/devel/setup.bash
```

Launch command used:

```bash
roslaunch alicia_flexible_grasp_supervisor full_system.launch start_real_arm:=true driver_port:=/dev/alicia_arm driver_baudrate:=1000000 auto_torque_on_startup:=true self_check_poll_rate_hz:=0.0 start_camera:=true start_tactile:=false start_gui:=true use_remote_grasp6d:=true remote_grasp6d_url:=http://172.23.132.97:8000
```

Observed node set:
- `alicia_d_driver_node`, `bessica_d_hw_interface`, `motion_gateway`, `supervisor_camera`, `perception_node`, `handeye_transform`, `remote_grasp6d_node`, `compliant_gripper_controller`, `grasp_task_node`, `safety_monitor_node`, `data_logger_node`, `move_group`, and `alicia_supervisor_gui` launched.
- `tactile_skin` was not launched; `start_tactile:=false` was used.

Observed runtime status:
- At launch, `/dev/alicia_arm` was initially missing and the driver kept reconnecting without publishing fake joint feedback.
- After WSL USB device attachment appeared, `/dev/alicia_arm -> ttyACM1`; the driver opened `/dev/alicia_arm` successfully.
- `auto_torque_on_startup` was active and the driver requested torque-on after reconnect.
- Real SDK feedback was received, with joint values near zero and gripper raw around 995.
- RealSense camera first reported no device, then recovered and started real color/depth publishing on `/supervisor/camera/color/image_raw` and `/supervisor/camera/depth/image_raw`.
- `remote_grasp6d_node` reported the protocol-v3 remote 6D server online at `http://172.23.132.97:8000`.
- `perception_node` detected target label `carton` once the camera stream recovered.

Current supervision note:
- No stop, torque-off, disable, or `/grasp/stop` command was published.
- Continue monitoring launch output while the operator manually moves the arm to align the target.
- After the operator says "已对准", instruct the operator to generate 6D candidates from the GUI and monitor for `PREVIEW_READY`.

## 2026-07-23 17:35 PDT - aligned target 6D candidate sync

Reason:
- Operator reported the target was aligned and generated 6D candidates from the GUI.
- Continuous candidate generation produced `PREVIEW_READY`, but later streaming updates could supersede the execution plan.

Observed result:
- Target label `carton` remained stable around image `uv=(320,205)` with depth near `0.29 m`.
- Remote 6D produced `PREVIEW_READY`.
- Some later metrics showed `promoted passed=0 rejected=1`, so execution was not started immediately.

Non-motion service actions:

```bash
rosservice call /grasp_6d/request_plan "trigger: false"
rosservice call /grasp_6d/replan_execution "trigger: true"
```

Observed service responses:
- `/grasp_6d/request_plan trigger=False`: `continuous remote 6D inference already stopped`.
- `/grasp_6d/replan_execution trigger=True`: `cached Preview promoted to execution authority`.

Current supervision note:
- No `/grasp/start`, `/grasp/stop`, torque-off, disable, or arm-stop command was published by the assistant.
- Operator was instructed to click the GUI execution button once.

## 2026-07-23 17:37 PDT - regenerated candidate and execution resync

Reason:
- Operator reported that the GUI execution button could not be clicked and chose to regenerate 6D candidates.
- Logs showed a fresh `PREVIEW_READY` candidate at 17:36 with `promoted passed=1` once, but later continuous streaming updates produced newer Preview candidates with `promoted passed=0 rejected=1`, making the GUI execution plan unavailable or superseded.

Non-motion service actions:

```bash
rosservice call /grasp_6d/request_plan "trigger: false"
rosservice call /grasp_6d/replan_execution "trigger: true"
```

Observed service responses:
- `/grasp_6d/request_plan trigger=False`: `continuous remote 6D inference stopped`.
- `/grasp_6d/replan_execution trigger=True`: `cached Preview promoted to execution authority`.

Current supervision note:
- No `/grasp/start`, `/grasp/stop`, torque-off, disable, or arm-stop command was published by the assistant.
- Operator was instructed to try the GUI execution button again after the execution resync.

## 2026-07-23 17:40 PDT - 6D execution result after regenerated candidate

Reason:
- Operator retried the execution after the regenerated Preview was promoted to Execution authority.

Observed execution sequence:
- `/grasp/start` was triggered from the GUI by the operator.
- `PLAN_PREGRASP`: MuJoCo digital twin check passed with score `100.000`.
- `MOVE_PREGRASP`: strict 6D pregrasp executed successfully to target `xyz=(-0.176, -0.434, 0.110)`.
- `APPROACH_TARGET`: first Cartesian approach executed successfully to `xyz=(-0.177, -0.441, 0.091)`.
- `APPROACH_TARGET`: second Cartesian motion to grasp pose executed successfully to `xyz=(-0.178, -0.448, 0.073)`.
- `COMPLIANT_CLOSE`: fixed gripper close completed.
- `LIFT_OBJECT`: Cartesian lift planned successfully with fraction `1.000`, distance `0.059 m`, target `xyz=(-0.178, -0.448, 0.123)`.
- `LIFT_OBJECT`: execution failed after MoveIt reported `CONTROL_FAILED`.

Root evidence:
- MoveIt reported: `Controller 'alicia_controller' failed with error GOAL_TOLERANCE_VIOLATED: Joint2 goal error 0.048685`.
- MoveIt then reported: `Controller handle alicia_controller reports status ABORTED`.
- Motion gateway reported: `execute failed from cached plan (cartesian): target xyz=(-0.178, -0.448, 0.123)`.
- Driver feedback during the failed lift showed the arm held near `joints_deg=[-113.8, -54.8, 97.1, -6.4, -71.2, 19.2]`, gripper raw near `4`, and intermittent `E1` status events treated as status-only; no sustained measured over-temperature torque-off was sent by the driver.

Interpretation:
- This run does not look like an immediate 6D generation or pregrasp planning failure: candidate promotion, MuJoCo check, pregrasp, approach, grasp pose, and gripper close all completed.
- The observed failure is at the lift execution/controller layer, with Joint2 missing the final goal tolerance by about `0.048685 rad` after the object/closed gripper load was present.
- A localization/TCP issue can still explain a physically skewed contact if the operator observed visible offset at the grasp pose, but the logged terminal failure for this run is controller tracking/tolerance failure on lift.

Current supervision note:
- No `/grasp/stop`, torque-off, disable, or arm-stop command was published by the assistant.
- The system remains under operator supervision for the next manual adjustment or next trial.

## 2026-07-23 17:49 PDT - physical grasp offset clarification

Operator clarification:
- The primary field failure is the physical grasp pose before close, not only the logged lift `CONTROL_FAILED`.
- At the stop/close pose the gripper was laterally offset to the left side of the target instead of centered on the object.
- The offset caused the gripper to touch the object's upper/side surface and miss the intended centered grasp.

Evidence from the execution audit:
- Execution plan id: `5fd6e64c1562e77f420cc074`.
- Execution source: `tabletop_geometry` / `carton_segment`.
- Target OBB center from the frozen geometry estimate: `base=(-0.177770, -0.444788, 0.072643)`.
- Tabletop geometry contact center in the audit: `base=(-0.178177, -0.439002, 0.080215)`.
- The tabletop contact center was therefore about `+5.8 mm` in base Y and `+7.6 mm` in base Z away from the target OBB center, with a total center distance of about `9.5 mm`.
- Candidate materialization solved `tool0` from that contact center and CAD support clearance, so the executed tool path can remain analytically valid while still looking visibly off-center on the real object.

Current root-cause hypothesis:
- The most likely cause is not a MoveIt execution drift during the final grasp pose. The motion gateway executed the commanded pregrasp, approach, and grasp-pose targets successfully.
- The stronger suspect is the `tabletop_geometry` center definition: it uses the median of the segmented visible target points as `contact_center_base`, while `object_geometry` separately computes an OBB center anchored to the support plane.
- For an RGB-D view dominated by the visible top/side surface of a box, the point-cloud median can be biased toward the visible surface, producing an off-center grasp that still passes OBB containment and MuJoCo gates.

Current supervision note:
- No `/grasp/stop`, torque-off, disable, or arm-stop command was published by the assistant.
- Do not patch live execution behavior blindly; the next diagnostic should compare OBB-center-based tabletop candidates against the existing median-center candidates before another physical execution.

## 2026-07-23 18:02 PDT - tabletop contact center corrected before next trial

Operator action:
- Operator clicked the GUI button to generate 6D candidates.

Latest audit evidence:
- Latest execution audit: `/home/zhuyupei/.ros/grasp6d_gate_audit_latest.json.execution`.
- Request id: `305`; generation: `5`; target epoch: `19`; plan id: `d3de3eaba6adc794cc16d987`.
- Outcome: `PLAN_READY`.
- Selected source: `tabletop_geometry`, source index `3`, variant index `1`.
- Frozen target center: `base=(-0.175351, -0.464422, 0.086055)`.
- Selected tabletop contact center: `base=(-0.173918, -0.461271, 0.092268)`.
- Contact-center delta from target center: about `(+1.4 mm, +3.2 mm, +6.2 mm)`, total about `7.1 mm`.
- Selected `tool0` translation: `base=(-0.172322, -0.454891, 0.083768)`, about `+9.5 mm` in base Y from the target center.

Root cause confirmed:
- The selected executable plan again came from `tabletop_geometry`.
- `tabletop_geometry_candidates.py` used `np.median(object_points_base, axis=0)` as the proposal `contact_center_base`.
- With a single RGB-D view of a box, the segmented points are biased toward visible top/side surfaces, so the median point can be displaced from the OBB/object center while still passing the geometry gates.

Code change:
- Updated `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/tabletop_geometry_candidates.py`.
- `generate_tabletop_proposals()` now uses the validated `obb_center_base` as `contact_center_base` for tabletop proposals instead of the visible-point median.

Regression test:
- Added `test_visible_cloud_bias_does_not_move_tabletop_contact_center()` in `src/alicia_flexible_grasp_supervisor/tests/test_tabletop_geometry_candidates.py`.
- The test first reproduced the bug: biased visible points produced contact center `[0.0, 0.0153125, 0.0073333]` instead of OBB center `[0.0, 0.0, 0.0055]`.
- After the fix, focused tabletop tests passed: `python3 -m pytest src/alicia_flexible_grasp_supervisor/tests/test_tabletop_geometry_candidates.py -q` -> `22 passed`.

Runtime supervision note:
- The current pre-fix execution plan should not be used for the next physical grasp trial.
- No `/grasp/stop`, torque-off, disable, or arm-stop command was published by the assistant.
- Next step is to restart only the remote 6D candidate node so the corrected tabletop center logic is used, then regenerate 6D candidates and verify the new audit before allowing execution.

## 2026-07-23 18:04 PDT - remote 6D node restarted with corrected center logic

Runtime action:
- `/remote_grasp6d_node` was no longer present in `rosnode list` after the previous generation burst, and no `remote_grasp6d_node.py` process was running.
- Started only `remote_grasp6d_node.py` from `/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade`.

Observed status:
- `rosnode list` again includes `/remote_grasp6d_node`.
- `/grasp_6d/status`: `remote 6D grasp waiting for RGB-D: http://172.23.132.97:8000 (manual trigger)`.
- `/grasp_6d/request_plan` and `/grasp_6d/replan_execution` services are registered.

Current supervision note:
- No `/grasp/stop`, torque-off, disable, or arm-stop command was published by the assistant.
- The arm driver, GUI, camera, MoveIt, motion gateway, and grasp task nodes were left running.
- Operator should click `Generate 6D Candidates` again so the corrected tabletop center logic can produce a fresh audit before execution.

## 2026-07-23 18:09 PDT - corrected 6D candidate verified

Fresh candidate evidence after restart:
- Latest execution audit: `/home/zhuyupei/.ros/grasp6d_gate_audit_latest.json.execution`.
- Request id: `61`; generation: `1`; target epoch: `1`.
- Outcome: `PLAN_READY`.
- Selected source: `tabletop_geometry`, source index `3`, variant index `1`.
- Frozen target center: `base=(-0.174625, -0.463772, 0.084074)`.
- Selected tabletop contact center: `base=(-0.174625, -0.463772, 0.084074)`.
- Contact-center delta from target center: `(0.0 mm, 0.0 mm, 0.0 mm)`.
- Selected `tool0` translation: `base=(-0.174163, -0.456463, 0.082634)`. This remaining tool-origin offset is expected from CAD/tool0 solving and is no longer the biased object contact center.

Current supervision note:
- The pre-fix execution plan should be ignored; the current request `61` plan is the corrected one.
- No `/grasp/stop`, torque-off, disable, or arm-stop command was published by the assistant.
- Operator may proceed with GUI execution for the next physical experiment while the assistant monitors logs.

## 2026-07-23 18:40 PDT - execution click failed at MuJoCo lift IK gate

Operator action:
- Operator clicked the GUI `Execute 6D Grasp` button.

Observed execution failure:
- GUI reported `FAILED`.
- `grasp_task_node` entered `PLAN_PREGRASP` and ran the MuJoCo digital-twin execution gate before physical motion.
- Failure: `MUJOCO_IK_FAILED: IK failed at lift: position error 0.0061m orientation error 0.0185`.
- MuJoCo audit: `/home/zhuyupei/.ros/grasp6d_mujoco_audit_latest.json`.
- Failed plan id: `b26f68118af08967b65611c7`.
- Failed plan source: `tabletop_geometry`.
- MuJoCo response summary: `ik_success=false`, `collision_free=false`, `contact_success=false`, `lift_success=false`, diagnosis `IK failed for lift`, score `15.0`.
- This failure happened in the pre-motion digital-twin gate, not after executing a physical grasp path.

Execution-plan evidence:
- Execution audit request id: `108`; generation: `3`; target epoch: `3`.
- Target/contact center was still corrected: `contact_center_base` exactly matched `target_cloud_center_base`.
- Selected insertion axis was tilted: `[0.0357, 0.1425, -0.9892]`, about `8.45 deg` away from vertical.
- The tilted candidate was produced because the live tabletop config still had `approach_tilt_degrees: [10.0, 15.0]`.

Code stability fix:
- Added a regression test in `src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py`:
  `test_submit_snapshot_after_coordinator_stop_is_dropped_not_crashed`.
- Updated `src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py` so a late stream snapshot submitted after the inference coordinator has stopped is dropped instead of raising `RuntimeError('inference coordinator is not running')` and killing `/remote_grasp6d_node`.
- Verification:
  - `python3 -m pytest src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py::test_submit_snapshot_after_coordinator_stop_is_dropped_not_crashed -q` -> `1 passed`.
  - `python3 -m pytest src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py -q` -> `167 passed`.

Candidate-generation config change:
- Removed `approach_tilt_degrees: [10.0, 15.0]` from `src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml`.
- This restores the production default to no tabletop approach tilt, so tabletop geometry produces vertical insertion candidates instead of tilted lift poses that can pass MoveIt but fail MuJoCo lift IK.
- Verification:
  - `python3 -m pytest src/alicia_flexible_grasp_supervisor/tests/test_graspnet_input_default_config.py -q` -> `4 passed`.
  - `python3 -m pytest src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_node.py -q` -> `147 passed`.

Live ROS update:
- `/remote_grasp6d_node` had exited after the coordinator RuntimeError and was restarted from `/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade`.
- Since ROS params retain launch-time values, the live param was also updated:
  `rosparam set /grasp_6d/remote/tabletop_geometry_candidates/approach_tilt_degrees []`.
- Confirmed live value: `[]`.
- Confirmed status: `/grasp_6d/status` reported `remote 6D grasp waiting for RGB-D: http://172.23.132.97:8000 (manual trigger)`.

Current supervision note:
- No `/grasp/stop`, torque-off, disable, or arm-stop command was published by the assistant.
- The next safe step is to regenerate 6D candidates from the GUI and inspect the new execution audit before attempting another execution.

## 2026-07-23 19:08 PDT - resumed supervision after pause

Runtime state:
- Continuing from `/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade`, which remains the correct latest ROS-side node path.
- `rosnode list` shows the real arm driver, GUI, camera, MoveIt, motion gateway, grasp task node, and `/remote_grasp6d_node` online.
- Electronic skin/tactile nodes remain off.
- `/grasp_6d/status` reported `continuous remote 6D inference started`.

Latest candidate audit:
- Audit file: `/home/zhuyupei/.ros/grasp6d_gate_audit_latest.json`.
- Stable snapshot request id: `435`; target epoch: `5`; no `plan_id`.
- Outcome: `valid_plan=false`, reason `GRIPPER_SWEEP_COLLISION:21`.
- Target center in base: `(-0.167476, -0.468898, 0.084745)`.
- No selected, promoted, preview, or executable candidate was present.
- Pipeline evidence: GraspNet candidates were dominated by gripper sweep collision, while tabletop geometry candidates were stable but failed MoveIt reachability around target xyz near `(-0.166, -0.463, 0.120)`.

Current supervision note:
- Operator should not click `Execute 6D Grasp` while no selected/promoted plan exists.
- To reproduce the last reachable `PLAN_READY` view, adjust the eye-in-hand view so the target bbox moves from the current `[300, 272, 356, 336]` / center x about `328 px` back toward the previous `[312, 272, 368, 336]` / center x about `340 px`.
- No `/grasp/stop`, torque-off, disable, or arm-stop command was published by the assistant.

## 2026-07-23 19:25 PDT - no-motion baseline sampling

Diagnostic-only sampling:
- Kept the current physical state untouched and sampled `/home/zhuyupei/.ros/grasp6d_gate_audit_latest.json`.
- Request `666`: target bbox `[300, 272, 356, 336]`, target center `(-0.166722, -0.467942, 0.083797)`, outcome `MOVEIT_UNREACHABLE:10`, no selected plan.
- Request `674`: target bbox `[300, 272, 356, 336]`, target center `(-0.166613, -0.467539, 0.085087)`, outcome `GRIPPER_SWEEP_COLLISION:18`, no selected plan.
- Observed target-center span across the samples: about `0.11 mm` in x, `0.40 mm` in y, and `1.29 mm` in z; total drift from the first sample about `1.36 mm`.

Conclusion from this sample:
- The current no-candidate state is not caused by target segmentation loss or one-off target-center jitter.
- The target is segmented stably, but all checked candidates are rejected by the planning/collision gates.
- No `/grasp/stop`, torque-off, disable, or arm-stop command was published by the assistant.

## 2026-07-24 01:49 PDT - latest full-system restart after WSL v3 ready

Reason:
- Continue from the latest `7.24.txt` conversation state.
- Restart ROS-side nodes from the latest protocol-v3 worktree after the operator reported WSL was already started.
- Keep the real arm enabled on startup and do not publish any stop, torque-off, disable, or `/grasp/stop` command.

WSL health check before ROS launch:
- `http://172.23.132.97:8000/health` returned `ok=true`.
- `protocol_version=3`.
- `digital_twin.model_xml=/home/lv/grasp6d_ws/Robot-Controller-v3/src/arm-mujoco/synriard/mjcf/Alicia_D_v5_6/Alicia_D_v5_6_gripper_50mm.xml`.

Setup path:

```bash
source /home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade/devel/setup.bash
```

Launch command used:

```bash
roslaunch alicia_flexible_grasp_supervisor full_system.launch start_real_arm:=true driver_port:=/dev/alicia_arm driver_baudrate:=1000000 auto_torque_on_startup:=true self_check_poll_rate_hz:=0.0 start_camera:=true start_tactile:=false start_gui:=true use_remote_grasp6d:=true remote_grasp6d_url:=http://172.23.132.97:8000
```

Observed startup:
- No previous ROS master or old ROS node processes were running before launch.
- Nodes launched from `/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade/devel/...`.
- Initial `/dev/alicia_arm` and RealSense devices were absent; driver and camera node waited/retried.
- RealSense later recovered and started publishing `/supervisor/camera/color/image_raw` and `/supervisor/camera/depth/image_raw`.
- `/dev/alicia_arm -> ttyACM1` appeared; driver opened `/dev/alicia_arm` and, because `auto_torque_on_startup:=true`, requested torque-on.

Observed runtime status after device recovery:
- `/alicia_d/motion_enabled`: `True`.
- `/alicia_d/feedback_ready`: `True`.
- `/alicia_d/run_status`: `0`.
- `/joint_states`: about `59 Hz`.
- Temperatures: about `27-30 C`.
- `perception_node` detected `carton` with confidence about `0.88`, bbox around `x=279 y=202 w=71 h=60`, depth about `0.266 m`, base pose around `(-0.181, -0.419, 0.076)`.
- `remote_grasp6d_node` reported `remote 6D grasp waiting for RGB-D: http://172.23.132.97:8000 (manual trigger)`.

Current supervision note:
- No `/grasp/start`, `/grasp/stop`, torque-off, disable, or arm-stop command was published by the assistant.
- Operator can manually move the arm to find/align the target.
- Next diagnostic should be planning-only/read-only until the operator reports the target is aligned.

## 2026-07-24 02:18 PDT - aligned target candidate recovery

Operator update:
- Operator reported the target was aligned.
- Continued with planning-only diagnostics. No physical grasp start command was sent.

Initial fresh-start observation:
- Target detection was stable near image center: bbox around `(282, 200, 78, 90)`, uv around `(320, 246)`.
- Target base pose was around `(-0.181, -0.404, 0.076)`.
- `/alicia_d/motion_enabled=True`, `/alicia_d/feedback_ready=True`, `/alicia_d/run_status=0`.
- `/joint_states` was publishing about `53-65 Hz` during the later health check.

Candidate-generation failure and fix:
- Manual `/grasp_6d/request_plan trigger:true` produced valid tabletop candidates and strict MoveIt checked `10/10`, but execution reachability was initially `0/10` because every candidate exceeded `candidate_max_joint_delta_rad=1.8`.
- The valid tabletop alternatives were just over the old bound, around `1.83-1.85 rad`; the unsafe wrist-flip alternatives remained much larger, around `2.7-3.0 rad`.
- Runtime param updated:
  `rosparam set /grasp_6d/remote/candidate_max_joint_delta_rad 1.85`.
- Persisted in `src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml`.
- Added `test_production_default_keeps_joint_flip_gate_tightly_bounded` in `src/alicia_flexible_grasp_supervisor/tests/test_graspnet_input_default_config.py`.
- Verification:
  - `python3 -m pytest -q src/alicia_flexible_grasp_supervisor/tests/test_graspnet_input_default_config.py src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_pipeline.py::test_moveit_joint_delta_limit_rejects_large_wrist_flip_before_selection` -> `6 passed`.

Preview/execution synchronization:
- Continuous inference was stopped with `/grasp_6d/request_plan trigger:false`; this only stops the candidate stream and does not disable or stop the mechanical arm.
- `/grasp_6d/replan_execution trigger:true` promoted the cached valid Preview to execution authority.
- Old plan `e2afd66bc89c204eae4eaf37` was valid but later rejected by direct MuJoCo probing as `PLAN_STALE` because its snapshot age was about `316 s`.
- A fresh 6D candidate stream was started again and allowed to run long enough for the 3-of-5 stability window.
- Fresh plan id: `3de2f5ab7e55f1b2e1b23a65`.
- Preview and execution enriched topics both latched the fresh plan id with stamp `1784884474.8321483`.
- Execution audit request id: `230`.
- Outcome: `PLAN_READY`.
- Selected source: `tabletop_geometry`.
- Stability: hit count `3`, hit request ids `[228, 229, 230]`, track id `1`.
- MoveIt selected evidence: `STRICT_SERVICE_SUCCESS`, target xyz about `(-0.181, -0.405, 0.104)`, joint path cost `2.515`, max joint delta `1.831 rad`.

Center and width evidence:
- Selected tabletop contact center matched the object model center:
  `(-0.181641, -0.408134, 0.070580)`.
- Required open width was about `0.0378-0.0379 m`, within the fixed `0.050 m` gripper contract.
- No evidence of the earlier visible-cloud median-center offset recurring in this fresh selected plan.

MuJoCo digital-twin probe:
- Direct no-motion probe used the same `build_mujoco_payload` and `validate_mujoco_gate_response` code path as `grasp_task_node`.
- Bound plan id: `3de2f5ab7e55f1b2e1b23a65`.
- Payload object center: `(-0.181641, -0.408134, 0.070580)`.
- Payload grasp pose: `(-0.181941, -0.409326, 0.063951)`.
- Payload object size: `(0.055014, 0.035433, 0.021819)`.
- MuJoCo response echoed the same plan id.
- MuJoCo flags: `simulation_ok=true`, `ik_success=true`, `collision_free=true`, `contact_success=true`, `lift_success=true`.
- Gate score: `100.0`; validation passed.

Current supervision note:
- No `/grasp/start`, `/grasp/stop`, torque-off, disable, or arm-stop command was published by the assistant.
- ROS launch session remains active with the real arm enabled.
- The fresh plan is valid now, but it is still subject to the 120 s snapshot freshness gate; regenerate candidates if execution is delayed long enough for the plan to become stale.

## 2026-07-24 02:50 PDT - latest code restart after close-range miss analysis

Operator artifact:
- Video reviewed: `/home/zhuyupei/Videos/6e83699aae2b44d41cc75784bb6825e2.mp4`.
- The physical trajectory moved to the 6D pregrasp, then performed the short approach and stopped before the final grasp/close phase.

Previous physical attempt evidence:
- Execution plan id: `50213af9fc026dc06b200914`.
- MuJoCo gate passed before motion with score `100.0`.
- `grasp_task_node` reached the 6D pregrasp and the linear 6D approach.
- Execution then failed before grasp pose with `EXECUTION_AUTHORITY_REVOKED` after close-range target loss.
- The visual target was still within the drift gate during approach, then became occluded/lost near the gripper.
- This was not the earlier visible-cloud median-center offset: selected tabletop contact center matched the object/OBB center.
- The selected tabletop source for that run was wider/nearer the gripper limit (`required_width` about `0.0461 m`); a narrower `0.0367 m` tabletop source existed but lost in soft ranking.

Code fixes now present in the restarted nodes:
- `scripts/grasp_task_node.py`
  - Allows close-range target occlusion after the 6D approach checkpoint without revoking the frozen execution authority.
  - Keeps the original fail-closed behavior before the approach checkpoint and for hard drift before the gripper/camera occlusion window.
- `scripts/remote_grasp6d_node.py`
  - Scores real remaining gripper opening margin as `gripper_open - required_width` instead of clamping it by support clearance.
  - This lets narrower tabletop candidates beat near-limit wide candidates when other source-neutral costs are comparable.

Verification before restart:
- `python3 -m pytest -q src/alicia_flexible_grasp_supervisor/tests/test_grasp_task_sequence.py src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py::test_common_soft_features_rewards_real_opening_margin_not_support_floor src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py::test_tabletop_stable_recheck_refreshes_width_from_current_cloud src/alicia_flexible_grasp_supervisor/tests/test_hybrid_grasp_candidates.py` -> `123 passed`.

Restart command used:
```bash
roslaunch alicia_flexible_grasp_supervisor full_system.launch start_real_arm:=true driver_port:=/dev/alicia_arm driver_baudrate:=1000000 auto_torque_on_startup:=true self_check_poll_rate_hz:=0.0 start_camera:=true start_tactile:=false start_gui:=true use_remote_grasp6d:=true remote_grasp6d_url:=http://172.23.132.97:8000
```

Observed restart status:
- Previous ROS master was unavailable, so a new ROS master was auto-started by roslaunch.
- Nodes online: `alicia_d_driver_node`, `bessica_d_hw_interface`, `motion_gateway`, `supervisor_camera`, `perception_node`, `handeye_transform`, `remote_grasp6d_node`, `compliant_gripper_controller`, `grasp_task_node`, `safety_monitor_node`, `data_logger_node`, `move_group`, `robot_state_publisher`, and `alicia_supervisor_gui`.
- Startup params confirmed:
  - `/alicia_d_driver_node/auto_torque_on_startup=true`.
  - `/alicia_d_driver_node/self_check_poll_rate_hz=0.0`.
- Runtime status confirmed:
  - `/alicia_d/motion_enabled=True`.
  - `/alicia_d/feedback_ready=True`.
  - `/alicia_d/run_status=0`.
  - `/joint_states` about `60 Hz`.
  - `/grasp/state`: `IDLE`, message `ready`.
  - `/grasp_6d/status`: `remote 6D grasp waiting for RGB-D: http://172.23.132.97:8000 (manual trigger)`.
  - `/perception/object`: detected `carton`, confidence about `0.90`, base pose about `(-0.182, -0.404, 0.079)`.

Safety boundary:
- No `/grasp/stop`, torque-off, disable, arm-stop, or mechanical-arm-stop command was published by the assistant.
- No mechanical-arm self-check command was requested; `self_check_poll_rate_hz` remains `0.0`.
- Next step is operator manual alignment, then a fresh manual 6D candidate generation and read-only audit before any physical execution.

## 2026-07-24 02:56 PDT - aligned target fresh 6D plan audit

Operator update:
- Operator reported the target was aligned.
- Assistant performed candidate generation and read-only audits only; no physical grasp execution was started.

Pre-plan state:
- `/grasp/state`: `IDLE`, message `ready`.
- `/alicia_d/motion_enabled=True`.
- `/perception/object`: detected `carton`, confidence about `0.90`, base pose about `(-0.182, -0.405, 0.078)`.

6D planning observations:
- Manual candidate generation started with `/grasp_6d/request_plan trigger:true`; this starts remote 6D inference only and does not move or disable the arm.
- Initial valid plan: `31597cacfb69ea58fb703883`, `tabletop_geometry`, required open width about `0.0376 m`.
- Latest preview audit selected tabletop candidates with stable `5/5` tracking hits and strict MoveIt success.
- Wider near-limit tabletop alternatives around `0.043-0.048 m` were not selected; selected candidates stayed around `0.0378-0.0384 m`.
- Contact centers matched the object/OBB center region, with no recurrence of the earlier visible-cloud median-center offset.

Execution authority refresh:
- The first execution authority plan was close to the 120 s snapshot freshness gate, so it was not used for physical execution.
- `/grasp_6d/replan_execution trigger:true` refreshed execution authority from the latest preview. This does not move the arm.
- Final MuJoCo probe used execution plan `9fae51c049cddd0668b15170`.

MuJoCo no-motion probe for `9fae51c049cddd0668b15170`:
- Snapshot age at probe: about `22.5 s`.
- Required open width: `0.0377599 m`.
- Payload object center: `(-0.182310, -0.406321, 0.072753)`.
- Payload grasp pose: `(-0.183983, -0.407605, 0.067812)`.
- Payload object size: `(0.049698, 0.034674, 0.019328)`.
- Candidate source: `tabletop_geometry`.
- MuJoCo response echoed the same plan id.
- MuJoCo flags: `simulation_ok=true`, `ik_success=true`, `collision_free=true`, `contact_success=true`, `lift_success=true`.
- Gate score: `100.0`; validation passed.

Current execution note:
- Do not execute if the plan approaches or exceeds the 120 s snapshot freshness gate; refresh execution authority and rerun MuJoCo first.
- Still no `/grasp/stop`, torque-off, disable, arm-stop, or mechanical-arm-stop command was published by the assistant.

## 2026-07-24 03:07 PDT - physical execution reached close, failed at 50 mm lift

Execution start:
- Operator requested execution.
- First `/grasp/start execute:true` attempt with plan `37f817a1207ec6fd0140b560` was rejected before motion:
  `PLAN_SUPERSEDED_BY_PREVIEW`.
- Continuous 6D inference was then stopped with `/grasp_6d/request_plan trigger:false` to freeze candidate promotion. This stops only the remote candidate stream; it is not a mechanical-arm stop or disable command.
- Latest cached Preview was promoted to execution authority with `/grasp_6d/replan_execution trigger:true`.
- Execution plan used for physical motion: `069e229c4b20d33181927412`.
- Prestart age was about `55.3 s`; MuJoCo gate passed with score `100.0`.

Physical execution timeline:
- MuJoCo execution gate passed inside `grasp_task_node`.
- `MOVE_PREGRASP`: strict cached 6D pregrasp planned and executed.
- `APPROACH_TARGET`: linear 6D approach planned and executed.
- `APPROACH_TARGET`: final linear 6D grasp pose planned and executed.
- Close-range occlusion preservation worked:
  `Preserving frozen 6D execution authority through close-range target occlusion`.
- `COMPLIANT_CLOSE`: fixed gripper close command was issued.
- `LIFT_OBJECT`: 50 mm linear 6D lift planning failed before lift motion:
  `Cartesian path incomplete fraction=0.833 < 0.980; target xyz=(-0.184, -0.409, 0.116)`.
- Final `/grasp/state`: `FAILED`, message `execution slot released: failed`.
- Post-failure status remained `/alicia_d/motion_enabled=True`, `/alicia_d/run_status=0`.

Operator video:
- Video reviewed: `/home/zhuyupei/Videos/7.mp4`.
- Video duration: about `36.37 s`.
- The trajectory reached final contact and closed the gripper, but the carton was visibly off-center between the fingers and became a diagonal/side-biased grasp.
- This confirms a remaining lateral contact bias in the physical grasp pose. The old candidate generation failures and close-range authority revocation are fixed, but final contact still needs a small correction.

Low-risk fix applied:
- Runtime param changed:
  `rosparam set /grasp/lift_height_m 0.035`.
- Persisted in `src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml`.
- Rationale: the real-arm 50 mm lift produced a Cartesian fraction of only `0.833`; a 35 mm lift keeps the post-close lift inside the reachable arc while still requiring MuJoCo and MoveIt checks.
- Added `test_production_default_uses_reachable_short_real_arm_lift`.
- Verification:
  `python3 -m pytest -q src/alicia_flexible_grasp_supervisor/tests/test_graspnet_input_default_config.py src/alicia_flexible_grasp_supervisor/tests/test_grasp_task_sequence.py::GraspTaskSequenceTest::test_close_range_target_occlusion_after_approach_preserves_frozen_execution src/alicia_flexible_grasp_supervisor/tests/test_grasp_task_sequence.py::GraspTaskSequenceTest::test_close_range_occlusion_allows_stale_cached_target_after_approach src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py::test_common_soft_features_rewards_real_opening_margin_not_support_floor` -> `9 passed`.

Remaining diagnosis:
- The selected plan was still a narrow tabletop grasp (`required_open_width_m` about `0.0385 m`), so the remaining video bias is not the old near-limit wide-candidate problem.
- Execution-time perception drift in the log moved the visible carton estimate away from the frozen OBB/contact pose during close approach, consistent with eye-in-hand close-range bias/occlusion.
- Next candidate correction should be conservative: use pregrasp-only visual micro-correction or an explicit calibrated planning offset, and revalidate before any physical motion.
- Still no `/grasp/stop`, torque-off, disable, arm-stop, or mechanical-arm-stop command was published by the assistant.

## 2026-07-24 03:17 PDT - aligned view produced no fresh executable preview

Operator state:
- Operator reported the arm/camera was aligned again.
- The arm remained enabled: `/alicia_d/motion_enabled=True`, `/alicia_d/run_status=0`.
- Runtime lift height remained `/grasp/lift_height_m=0.035`.

Observed perception and candidate state:
- Target was detected as `carton`, confidence about `0.908`, UV around `(308, 202)`.
- Current target base estimate was about `(-0.202, -0.444, 0.083)`.
- The last executable physical plan `069e229c4b20d33181927412` remained latched on `/grasp_6d/preview_plan_enriched`; its stamp was old and must not be reused as a fresh preview.
- New gate audits had `selected=null` / `preview_valid=false`; no new executable preview was available.
- Main rejection/failure modes were `GRIPPER_SWEEP_COLLISION` and `MOVEIT_UNREACHABLE`.
- MoveIt strict checks repeatedly failed for pregrasp targets around `(-0.204, -0.444, 0.110-0.112)`, so no `/grasp/start` command was issued.

Safety boundary:
- No mechanical stop, torque-off, disable, `/grasp/stop`, or arm-stop command was published.
- Next step is to manually reposition the object/arm view into a reachable candidate region, then wait for a fresh preview whose timestamp is newer than this alignment attempt before MuJoCo validation and any physical execution.

## 2026-07-24 03:46 PDT - corrected strategy: all-pose optimal grasp selection

Operator correction:
- The target object is in the camera frame and within the robot reachable area.
- The system must not require repositioning the target object as the primary fix.
- Correct strategy: allow the grasp generator to consider broader task-space
  grasp poses, then select the best pose that still passes the hard gates
  (geometry, gripper width, MoveIt, MuJoCo, and execution binding).
- Position-only MoveIt fallback remains unsuitable as a grasp pose because it
  can reach the point with an arbitrary tool orientation; it is planning
  evidence only, not execution authority.

Root cause found in candidate selection:
- Tabletop geometry materialization produced more pose variants when approach
  tilt samples were enabled, but `remote_grasp6d_node.py` still hard-capped the
  materialized result with `materialized[:8]`.
- Stable tabletop tracks could contain repeated source/variant poses, so the
  bounded MoveIt shortlist was often consumed by duplicates before other
  candidate poses were checked.
- This made the live system look as if candidates had disappeared or the target
  was unreachable, even though a broader set of task poses should have been
  compared.

Code changes applied:
- `src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py`
  - Added `MOVEIT_TOP_N_MAX = 24` and raised runtime `moveit_top_n` clamp from
    `3..10` to `3..24`.
  - Added `TABLETOP_GEOMETRY_MAX_CANDIDATES = 32`.
  - Changed tabletop defaults to `max_candidates=24` and
    `approach_tilt_degrees=(10, 20, 30, 40)`.
  - Replaced the fixed `materialized[:8]` cap with the configured
    `self.tabletop_geometry_config.max_candidates`.
  - Added `materialized_total_count` diagnostics so future audits can show
    whether generation was truncated.
  - Added a narrow MoveIt-preselection dedupe for `tabletop_geometry` candidates
    keyed by `(source_index, source_variant_index, evaluation_variant_index)`;
    this keeps the best-scored duplicate and leaves GraspNet tracks unmerged.
- `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/grasp6d_pipeline.py`
  - Raised the pure bounded MoveIt selector clamp to `24`.
- `src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml`
  - Persisted `moveit_top_n: 24`.
  - Persisted tabletop `max_candidates: 24`.
  - Persisted tabletop `approach_tilt_degrees: [10.0, 20.0, 30.0, 40.0]`.
- `src/alicia_flexible_grasp_supervisor/tools/benchmark_tabletop_geometry.py`
  - Production tabletop benchmark loading now preserves
    `approach_tilt_degrees`.
- Tests updated/added for the new contract and dedupe behavior.

Verification:
- Syntax:
  `python3 -m py_compile ...remote_grasp6d_node.py ...grasp6d_pipeline.py ...benchmark_tabletop_geometry.py`
  passed.
- Focused regression:
  `10 passed`.
- Full `test_remote_grasp6d_node.py`:
  `149 passed`.
- Full `test_remote_grasp6d_streaming.py`:
  `168 passed`.
- Related geometry/pipeline/default-config batch:
  `216 passed`, `2 failed`.
  The two failures require the genuine fixture file
  `tests/fixtures/carton_tabletop_cloud.json`, which is missing in this
  worktree; they are not behavior regressions from this change.

Safety boundary:
- No `/grasp/start` command was issued after this code change.
- No mechanical stop, torque-off, disable, `/grasp/stop`, or arm-stop command
  was published.
- Next runtime step is to restart only the remote candidate-generation node so
  this new selection code loads. Do not restart/stop the arm driver or publish
  any command that disables the arm.

## 2026-07-24 18:45 PDT - resumed full ROS bringup for aligned-target run

Additional runtime issue found before the operator pause:
- After raising tabletop generation to 24 candidates, one lower validation
  layer still rejected `max_candidates=24` with
  `max_candidates must be between 1 and 8`.
- `tabletop_geometry_candidates.py` was updated so production tabletop
  generation accepts up to 32 candidates.
- A later candidate-node restart showed the expanded chain working:
  `tabletop_materialized entered=24 passed=24`,
  `stable entered=22 passed=22`,
  `moveit_checked entered=24 passed=24`,
  `moveit_reachable passed=4..5`, and fresh tabletop previews were published.
- The earlier streaming stall was traced to `_poll_stream_snapshot()` using a
  zero-second wait while `planning_snapshot_max_age_sec=0.35`; at low request
  rates it could miss every synchronized RGB-D/object/mask window.
- `_poll_stream_snapshot()` now waits up to one current request period, bounded
  by `planning_snapshot_timeout_sec`, before submitting a fresh streaming
  snapshot.

Verification:
- `python3 -m py_compile src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py`
  passed.
- `source devel/setup.bash; python3 -m pytest -q
  src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py::test_stream_poll_waits_for_fresh_window_and_submits_only_once`
  passed.

Latest ROS-side bringup:
- Full launch command used:
  `roslaunch alicia_flexible_grasp_supervisor full_system.launch start_real_arm:=true driver_port:=/dev/alicia_arm driver_baudrate:=1000000 auto_torque_on_startup:=true self_check_poll_rate_hz:=0.0 start_camera:=true start_tactile:=false start_gui:=true use_remote_grasp6d:=true remote_grasp6d_url:=http://172.23.132.97:8000`
- Launch session: Codex terminal session `92005`.
- ROS log directory for this run:
  `/home/zhuyupei/.ros/log/6a52a6fc-87ca-11f1-a8fe-61bcb2230137/`.
- Startup parameters confirmed in roslaunch output:
  `/alicia_d_driver_node/auto_torque_on_startup=True`.
  `/alicia_d_driver_node/self_check_poll_rate_hz=0.0`.
  `/grasp_6d/remote/moveit_top_n=24`.
  `/grasp_6d/remote/tabletop_geometry_candidates/max_candidates=24`.
  `/grasp_6d/remote/tabletop_geometry_candidates/approach_tilt_degrees=[10,20,30,40]`.
- Driver opened `/dev/alicia_arm` and reported
  `auto_torque_on_startup is enabled; torque-on will be requested`.
- Real SDK feedback is streaming with `status=0x00`.
- Camera started in real mode, perception detector loaded for `carton`, remote
  6D server reported online, and MoveGroup printed `You can start planning now!`.

Current operational boundary:
- No arm health/self-check was requested.
- No candidate generation has been requested in this resumed run yet.
- No `/grasp/start` command has been issued.
- No mechanical stop, torque-off, disable, `/grasp/stop`, or arm-stop command
  was published.
- Next step: operator manually moves/alines the arm/target in view and reports
  `已对准`; then generate a fresh 6D candidate preview, run no-motion MuJoCo
  validation, and ask for explicit execution confirmation before any physical
  grasp command.

## 2026-07-24 19:05 PDT - aligned run fresh candidate and MuJoCo gate

Operator state:
- Operator reported `已对准`.
- The target remained visible and stable in the camera image as label `carton`,
  around `uv=(290,190)`, depth about `0.282..0.284 m`, and base position near
  `(-0.175, -0.419, 0.080)`.

Runtime actions:
- Ensured continuous 6D generation was running with
  `/grasp_6d/request_plan trigger=true`.
- Increased runtime result tolerance earlier in the run to
  `/grasp_6d/remote/result_max_age_sec=15.0`.
- Reduced `/grasp_6d/remote/request_hz` from `0.5` while diagnosing slow WSL
  inference. A trial value of `0.03` was rejected by the production contract
  with `request_hz must be >= 0.1`; it was immediately corrected to `0.1`.
- The `0.03` error affected only candidate generation and did not command arm
  motion.

Observed candidate behavior:
- An older preview `483926bb6c3daf5a029a081f` was rejected by direct MuJoCo
  probing as `PLAN_STALE` because the snapshot age had grown to about `153 s`.
- A newer fresh plan was then produced:
  `plan_id=914b2e0957d6451c8c35574b`.
- Source: `tabletop_geometry`.
- Required opening: `0.039907 m`.
- Object center: `base=(-0.175923, -0.424435, 0.074412)`.
- Grasp pose: `base=(-0.176333, -0.438680, 0.081861)`.
- Lift target: `base=(-0.176333, -0.438680, 0.116861)`.
- Pipeline evidence for this generation included
  `stable passed=22`, `moveit_checked passed=24`,
  `moveit_reachable passed=7`, `preview passed=1`, and `promoted passed=1`.

No-motion MuJoCo validation:
- Direct offline MuJoCo gate check used the same
  `build_mujoco_payload -> /simulate_grasp -> validate_mujoco_gate_response`
  path as `grasp_task_node`, but did not call `/grasp/start`.
- MuJoCo accepted plan `914b2e0957d6451c8c35574b` with score `100.0`.
- Response booleans were all true:
  `simulation_ok`, `ik_success`, `collision_free`, `contact_success`, and
  `lift_success`.
- Plan age after the MuJoCo response was about `33.6 s`, so this was not the
  stale cached preview.

Remaining issue to keep after this trial:
- WSL inference plus ROS-side MoveIt selection can still make later continuous
  results old enough to hit TF extrapolation or MuJoCo `PLAN_STALE`.
- Candidate generation should keep using a legal low request rate (`0.1 Hz`)
  for this slow server, and a future code change should make slow-inference
  time-window handling explicit rather than relying on operator timing.

Current operational boundary:
- No `/grasp/start` command has been issued by Codex for this validated plan.
- No mechanical stop, torque-off, disable, `/grasp/stop`, or arm-stop command
  was published.
- Physical execution still requires explicit operator confirmation.

## 2026-07-24 20:14 PDT - real-run drift diagnosis and near-field replan fix

Real execution result:
- Codex later started one real `/grasp/start execute=true` only after a fresh
  rich plan was available; no `/grasp/stop`, torque-off, disable, or arm-stop
  command was published.
- The executed plan was `1620c942c9c7c26ee7f012bd`, source
  `tabletop_geometry`, snapshot stamp `1784947750.2002249`.
- MuJoCo accepted the bound plan with score `100.0`
  (`simulation_ok`, `ik_success`, `collision_free`, `contact_success`, and
  `lift_success` all true).
- The arm executed the 6D pregrasp, linear 6D approach, and linear 6D grasp
  pose, then failed before gripper close with:
  `TARGET_DRIFT: live target drift 0.046m exceeds 0.040m`.
- The operator photo confirmed the gripper was still skewed relative to the
  carton. The important point is that the guard stopped the close; widening the
  drift gate would only allow a visibly offset close.

Diagnosis:
- The plan geometry was generated from the far-field view around target center
  `base=(-0.175, -0.424, 0.074)`.
- After the wrist/camera moved near the object, live perception saw the same
  carton around `base=(-0.191, -0.471, 0.080)`, about `46 mm` away from the
  bound plan geometry.
- This should not be handled by moving/adjusting the physical target, and the
  code must not translate an old 6D plan to chase the new detection.
- Correct behavior: use the far-field plan only to reach a coarse pregrasp
  viewpoint, then generate/select a new near-field 6D pose from the current
  RGB-D target geometry and bind that fresh plan for the final approach/close.

Code/config changes:
- `grasp_params.yaml`
  - Restored `grasp_6d.target_max_drift_m` to `0.04`.
  - Added `grasp.near_field_replan_enabled: true`.
  - Added required near-field preview settings:
    `near_field_replan_required`, `near_field_replan_request_stream`,
    `near_field_replan_timeout_sec`, `near_field_replan_poll_sec`, and
    `near_field_replan_snapshot_slack_sec`.
- `grasp_task_node.py`
  - After the initial 6D pregrasp is reached, the node now waits for a fresh
    `/grasp_6d/preview_plan_enriched` plan from the near-field camera view.
  - The Preview plan must be newer than the old execution plan and the
    near-field request window, must pass rich-plan integrity/freshness checks,
    and must pass live target drift checks against the current target geometry.
  - Preview vetting uses a non-mutating target-drift check so a bad Preview
    cannot revoke the current frozen execution authority.
  - A good near-field Preview is frozen as the new execution plan, rechecked by
    the same MuJoCo execution gate, and then the arm moves to the near-field
    pregrasp before final approach, grasp pose, close, and lift.
  - The implementation requests `/grasp_6d/request_plan trigger=true` only to
    keep candidate generation running; it does not call `trigger=false`,
    `/grasp_6d/replan_execution`, `/grasp/stop`, torque-off, disable, or any
    arm-stop service.

Verification so far:
- `python3 -m py_compile src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py`
  passed.
- Focused near-field tests passed:
  `test_near_field_preview_drift_rejects_without_revoking_bound_plan` and
  `test_near_field_replan_binds_preview_and_runs_mujoco_gate`.

Runtime state after diagnosis:
- `/grasp/state` reports `FAILED`, `active=False`,
  `message="execution slot released: failed"`.
- `/grasp_6d/status` reports continuous remote 6D inference already running.
- A later near-field Preview existed from the current close view:
  plan `67388f6fe23990584eb62cc1`, object center approximately
  `base=(-0.185, -0.450, 0.078)`, grasp pose approximately
  `base=(-0.188, -0.468, 0.090)`.
- No new real grasp was started after this diagnosis.

## 2026-07-24 20:22 PDT - loaded near-field replan supervisor code

Runtime parameter update:
- Set `/grasp/near_field_replan_enabled=true`.
- Set `/grasp/near_field_replan_required=true`.
- Set `/grasp/near_field_replan_request_stream=true`.
- Set `/grasp/near_field_replan_timeout_sec=35.0`.
- Set `/grasp/near_field_replan_poll_sec=0.05`.
- Set `/grasp/near_field_replan_snapshot_slack_sec=0.25`.
- Set `/grasp/near_field_replan_service_timeout_sec=3.0`.
- Confirmed `/grasp_6d/target_max_drift_m=0.04`.

Node reload:
- Reloaded only `/grasp_task_node` so the modified supervisor code is active.
- Did not restart the arm driver, did not run arm health/self-check, did not
  call `/grasp/stop`, and did not publish torque-off/disable/arm-stop commands.
- New `/grasp_task_node` PID `44026` registered `/grasp/start` and
  `/grasp/stop`, subscribed to `/grasp_6d/preview_plan_enriched`, and published
  `/grasp/state` as `IDLE`, `active=False`, `message="ready"`.

Verification:
- Full grasp task sequence regression passed:
  `python3 -m pytest -q src/alicia_flexible_grasp_supervisor/tests/test_grasp_task_sequence.py`
  -> `95 passed`.
- Syntax checks passed for:
  `grasp_task_node.py`, `remote_grasp6d_node.py`, and `grasp6d_pipeline.py`.
- YAML parse confirmed the persisted config has
  `grasp.near_field_replan_enabled=True` and
  `grasp_6d.target_max_drift_m=0.04`.

Current planning observation:
- Candidate stream remains running.
- Current near-field Preview sample after reload:
  plan `1b6b86f3c8c47a20629a7610`, source `tabletop_geometry`,
  object center approximately `base=(-0.185, -0.451, 0.078)`,
  grasp pose approximately `base=(-0.186, -0.456, 0.077)`,
  required opening approximately `0.03995 m`.
- No physical grasp execution has been started after loading this fix.

## 2026-07-24 20:26 PDT - planning-only execution plan sync

Planning-only action:
- Called `/grasp_6d/replan_execution trigger=true` to synchronize the current
  Preview into `/grasp_6d/plan_enriched`.
- This was not a motion command and did not call `/grasp/start`,
  `/grasp/stop`, torque-off, disable, or arm-stop.
- Service response:
  `success=True`, `message="explicit execution replan requested"`.

Result:
- `/grasp_6d/plan_enriched` published valid plan
  `af223bf8710f1b439038c09f`, source `tabletop_geometry`.
- Required opening: `0.039738 m`.
- Object center:
  `base=(-0.185574, -0.451145, 0.078605)`.
- Grasp pose:
  `base=(-0.185856, -0.456236, 0.078509)`.
- `/grasp/state` remained `IDLE`, `active=False`, `message="ready"`.

Next safe operator step:
- The system is ready for planning/Preview generation in the current view.
- Do not execute a real grasp until the operator confirms alignment and Codex
  explicitly starts the next controlled execution step.

## 2026-07-24 21:09 PDT - side-sweep grasp bias fix loaded

Latest physical observation:
- After the previous real `/grasp/start execute=true` attempt, the user photo
  showed the gripper still skewed beside the carton.
- The supervisor failed at the 6D strict pregrasp execution target
  `base=(-0.196, -0.443, 0.117)` before final close/near-field rebind.
- The selected a836 plan was not an object-pose problem: the target remained
  in view and inside reach. The issue was that the candidate approach itself
  swept sideways into the carton: pregrasp-to-grasp travel was about
  `dx=+0.005m, dy=-0.024m, dz=-0.031m`, roughly `24.9mm` lateral motion on the
  support plane.

Code/config changes:
- Added a runtime candidate gate in
  `src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py`:
  `TABLETOP_APPROACH_LATERAL_SWEEP` rejects pregrasp-to-grasp support-plane
  lateral sweep above `/grasp_6d/remote/candidate_max_final_approach_lateral_m`.
- The gate is applied to GraspNet analytical geometry, tabletop geometry
  normalization, and stable-candidate recheck before audit recording.
- Fixed the gate helper to copy `support_normal_base` before normalization;
  the first live reload exposed a real read-only numpy array with
  `ValueError: output array is read-only`.
- Updated production config:
  - `/grasp_6d/remote/tabletop_geometry_candidates/approach_tilt_degrees`
    from `[10.0, 20.0, 30.0, 40.0]` to `[10.0, 20.0]`.
  - `/grasp_6d/remote/candidate_max_final_approach_lateral_m=0.018`.
  - Initially tried `/grasp_6d/remote/candidate_min_downward_approach_cos=0.80`;
    live audit showed zero current candidates passing that baseline, so it was
    relaxed to `0.65` while retaining the `18mm` lateral-sweep hard gate.

Verification:
- `python3 -m py_compile src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py`
  passed.
- `python3 -m pytest -q src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_node.py`
  -> `150 passed`.
- Focused production-config test passed:
  `test_production_default_enables_bounded_tabletop_geometry_candidates`.
- Added regression coverage using the failed a836 geometry: about `24.898mm`
  lateral sweep is rejected as `TABLETOP_APPROACH_LATERAL_SWEEP`, including when
  the support normal is a read-only numpy array.

Runtime actions:
- Set ROS params live:
  - `/grasp_6d/remote/tabletop_geometry_candidates/approach_tilt_degrees=[10.0, 20.0]`
  - `/grasp_6d/remote/candidate_min_downward_approach_cos=0.65`
  - `/grasp_6d/remote/candidate_max_final_approach_lateral_m=0.018`
- Restarted only `/remote_grasp6d_node` twice to load the code fix and the
  read-only-array fix.
- Did not restart the arm driver, did not run health/self-check, did not call
  `/grasp/stop`, and did not publish torque-off, disable, or arm-stop commands.

Current planning result:
- Pure planning `/grasp_6d/request_plan trigger=true` is running.
- Pure planning `/grasp_6d/replan_execution trigger=true` succeeded.
- Current execution plan:
  `plan_id=c4bd180d63b7165647f2da8c`, source `tabletop_geometry`,
  required opening `0.039583m`.
- Object center:
  `base=(-0.193246, -0.450233, 0.080747)`.
- Pregrasp:
  `base=(-0.192089, -0.443476, 0.119313)`.
- Grasp:
  `base=(-0.196750, -0.461514, 0.083917)`.
- Support normal:
  `(0.030032, 0.132882, 0.990677)`.
- The current plan's pregrasp-to-grasp support-plane lateral sweep is
  `0.013638m`, below the `0.018m` limit. Raw XY lateral is `0.018630m`, but the
  support-plane projection is the gate value.
- `/grasp/state` remains the previous `FAILED`, `active=False`,
  `message="execution slot released: failed"`; no new real grasp was started.

Next safe operator step:
- The candidate-generation side-sweep bug is fixed and a new execution plan is
  available.
- Before any real grasp, the operator should confirm the current camera view is
  still aligned because the plan stamp may age while reviewing.

## 2026-07-24 22:05 PDT - tighter side-sweep gate and stale-plan promotion fix

Latest physical observation:
- User uploaded `/home/zhuyupei/Videos/5f27cf385f500df548355e3114bf01bf.mp4`
  for the latest failed grasp. Extracted frames showed the gripper approaching
  from a side-biased direction and the gripper centerline landing near the
  carton side rather than across the carton center.
- This matched the live audit pattern: the earlier permissive final-approach
  lateral gate allowed 20 degree tabletop candidates whose final approach swept
  sideways by roughly 14-18 mm.

Code/config changes:
- Tightened `/grasp_6d/remote/candidate_max_final_approach_lateral_m` from
  `0.018` to `0.010` in `grasp_params.yaml` and the remote-node fallback.
  This rejects the observed 20 degree side-entry candidates while retaining
  near-vertical 0/10 degree candidates.
- Raised `/grasp_6d/remote/candidate_max_joint_delta_rad` from `1.85` to
  `2.15`. Live MoveIt audit showed the near-vertical candidates needed about
  `2.04-2.15 rad` from the current aligned wrist posture, while the unsafe
  wrist-flip variants remained much larger and were still rejected.
- Added audit evidence fields for tabletop candidates:
  `approach_variant_index`, `approach_tilt_deg`,
  `approach_tilt_polarity`, `downward_approach_cos`, and
  `final_approach_lateral_m`.
- Fixed a stale-plan promotion bug in `remote_grasp6d_node.py`: execution-plan
  expiry was comparing ROS/epoch plan stamps against `time.monotonic()`, so old
  execution plans could be incorrectly held as valid. Execution-plan validity
  now uses ROS time, while promotion throttling still uses the monotonic stream
  clock.
- Added regression coverage for the ROS-time/monotonic-time mismatch.

Verification:
- `python3 -m py_compile src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py`
  passed.
- `python3 -m pytest -q src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py::test_epoch_stamped_execution_plan_expires_against_ros_time_not_monotonic src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py::test_expired_execution_plan_allows_fresh_preview_promotion`
  -> `2 passed`.
- `python3 -m pytest -q src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_node.py -k "final_approach_lateral_gate"`
  -> `1 passed, 151 deselected`.
- `python3 -m pytest -q src/alicia_flexible_grasp_supervisor/tests/test_graspnet_input_default_config.py::GraspNetInputDefaultConfigTest::test_production_default_keeps_joint_flip_gate_tightly_bounded src/alicia_flexible_grasp_supervisor/tests/test_graspnet_input_default_config.py::GraspNetInputDefaultConfigTest::test_production_default_enables_bounded_tabletop_geometry_candidates`
  -> `2 passed`.
- Known unrelated full-file config test issue remains unchanged:
  `test_runtime_default_allows_measured_segmentation_latency` expects
  `target_observation_validity_sec == 3.0`, while current production config is
  `8.0`.

Runtime actions:
- ROS full system remained running with real arm enabled from the latest
  worktree launch. No arm health/self-check was run.
- Restarted only `/remote_grasp6d_node` to load the new code. Did not restart
  the arm driver, MoveIt, camera, or GUI.
- Used `/grasp_6d/request_plan trigger=true` for planning-only candidate
  generation.
- Used `/grasp_6d/replan_execution trigger=true` once to promote the current
  cached Preview when an older execution plan was still held.
- Used `/grasp_6d/request_plan trigger=false` only to stop/freeze the 6D
  candidate stream after a plan was promoted. This is not a mechanical-arm stop
  or torque command.
- Did not call `/grasp/start execute=true` after this fix, did not call
  `/grasp/stop`, and did not publish torque-off, disable, or arm-stop commands.

Live planning observations:
- With the 10 mm final-approach lateral gate, 20 degree side-entry tabletop
  variants were rejected as `TABLETOP_APPROACH_LATERAL_SWEEP`, e.g.
  `pregrasp-to-grasp lateral sweep 0.014m exceeds 0.010m`.
- A planning-only candidate before the final restart selected
  `plan_id=00d2bea303395a6b9cf0844a`, source `tabletop_geometry`, final
  approach lateral norm about `0.0057m`, and MuJoCo returned
  `simulation_ok=True`, `ik_success=True`, `collision_free=True`,
  `contact_success=True`, `lift_success=True`, `score=100`.
- After loading the stale-plan fix, a fresh plan was promoted without the old
  stale execution plan blocking it:
  `plan_id=57c9c51890303a610f30af1b`, source `tabletop_geometry`,
  required opening `0.038754m`, MoveIt strict result
  `joint_path_cost=2.660`, `joint_max_delta=1.985`, promotion
  `PROMOTE_INITIAL`.
- Plan `57c9c51890303a610f30af1b` poses:
  pregrasp `base=(-0.187333, -0.440352, 0.116799)`,
  approach `base=(-0.188657, -0.446115, 0.097693)`,
  grasp `base=(-0.189980, -0.451879, 0.078587)`,
  lift `base=(-0.189980, -0.451879, 0.113587)`.
- Final approach XY delta was approximately
  `(-0.001323, -0.005763)m`, lateral norm `0.005913m`.
- Object center was approximately
  `base=(-0.188238, -0.446562, 0.079321)`.
- MuJoCo execution-gate probe for `57c9c51890303a610f30af1b` returned
  `simulation_ok=True`, `ik_success=True`, `collision_free=True`,
  `contact_success=True`, `lift_success=True`, `score=100`, and exact plan-id
  echo.
- After logging/inspection, `57c9c51890303a610f30af1b` was already older than
  the 120 s validity window (`age_sec=124.555`) and must not be used directly
  for physical execution. A fresh 6D candidate should be regenerated immediately
  before the next real `/grasp/start execute=true`.

Next operator step:
- Keep the carton in the current camera view and confirm alignment again.
- When the operator says to execute, regenerate a fresh 6D execution plan first,
  verify MuJoCo, then start the real grasp with that fresh plan id only.

Additional fresh-plan refresh:
- After the log entry above, a new planning-only refresh produced current
  execution plan `026a5ffa9626a985a7376b12`, source `tabletop_geometry`,
  request `84`, generation `3`, target epoch `3`.
- Candidate stream was then stopped with `/grasp_6d/request_plan trigger=false`
  to freeze the plan; this did not stop or disable the arm.
- Required opening: `0.040177m`.
- Object center:
  `base=(-0.189501, -0.446315, 0.079367)`.
- Approach pose:
  `base=(-0.189571, -0.448834, 0.097589)`.
- Grasp pose:
  `base=(-0.191144, -0.454601, 0.078504)`.
- Final approach XY delta:
  `(-0.001573, -0.005768)m`, lateral norm `0.005979m`.
- MuJoCo execution-gate probe returned
  `simulation_ok=True`, `ik_success=True`, `collision_free=True`,
  `contact_success=True`, `lift_success=True`, `score=100`, and exact plan-id
  echo.
- The plan age after the probe was `77.550s`; it remains usable only inside
  the 120 s validity window. If the operator waits, regenerate before execution.

## 2026-07-24 resume refresh - fresh c79 execution plan

Resume action after interruption:
- Confirmed ROS nodes still online:
  `/alicia_d_driver_node`, `/grasp_task_node`, `/move_group`,
  `/supervisor_camera`, `/alicia_supervisor_gui`, and `/remote_grasp6d_node`.
- `/grasp_6d/status` showed the 6D candidate stream stopped.
- Existing execution plan `026a5ffa9626a985a7376b12` was `459.691s` old and
  was not used for execution.
- Started planning-only refresh with `/grasp_6d/request_plan trigger=true`.
  No arm motion was commanded.

Fresh execution plan:
- New execution authority:
  `plan_id=c79db17efa4f216b2b4b3c70`, request `108`, generation `5`,
  target epoch `5`.
- Promotion:
  `PLAN_READY`, `PROMOTE_INITIAL`, reason `no valid execution plan exists`.
- Candidate stream was then stopped with `/grasp_6d/request_plan trigger=false`
  only to freeze the 6D plan; this did not stop or disable the arm.
- Required opening: `0.039139m`.
- Object center:
  `base=(-0.187857, -0.448746, 0.080049)`.
- Pregrasp:
  `base=(-0.188951, -0.441849, 0.116773)`.
- Approach:
  `base=(-0.190257, -0.447864, 0.097744)`.
- Grasp:
  `base=(-0.191564, -0.453878, 0.078714)`.
- Final approach XY delta:
  `(-0.001306, -0.006014)m`, lateral norm `0.006155m`.
- MuJoCo execution-gate probe returned
  `simulation_ok=True`, `ik_success=True`, `collision_free=True`,
  `contact_success=True`, `lift_success=True`, `score=100`, and exact plan-id
  echo.
- Plan age after MuJoCo probe: `67.560s`, so it was valid at probe time but
  still must be regenerated if the 120 s validity window elapses before real
  execution.

Safety notes:
- No `/grasp/start execute=true` was called in this resume segment.
- No `/grasp/stop`, torque-off, disable, arm-stop, or health/self-check command
  was issued.

## 2026-07-24 conclusion - offset source audit

User photo/video conclusion:
- The frozen pregrasp photo shows the physical jaw opening center visibly away
  from the carton center normal.
- This is not evidence that the target object pose should be changed. It is
  evidence that the executable grasp candidate center/TCP-to-finger-center
  chain is not aligned to the segmented object's intended center.

Evidence checked:
- Live TF `tool0 -> camera_link` is currently published as
  `xyz=(-0.086, 0.014, -0.113)`, `q=(0.004, -0.709, 0.009, 0.705)`, matching
  the easy_handeye file
  `~/.ros/easy_handeye/d405_v4l2_charuco_handeyecalibration_eye_on_hand.yaml`.
- `/handeye` rosparam still contains older fallback values
  `xyz=(-0.084494, 0.010051, -0.123367)`, `q=(0.004411, -0.685270, 0.000055,
  0.728275)`, but the handeye node code loads `calibration_file` first when it
  exists, so current runtime TF uses the easy_handeye file rather than the
  fallback param values.
- No saved TCP result file like `tcp_calibration_*.yaml` exists under
  `/home/zhuyupei/alicia_wa_full/calibration_results`; the available
  `tcp_calibration.yaml` is only the calibration task configuration and the
  historical TCP log only shows the node becoming ready, not solve/save.
- Current 6D audit target cloud center was
  `(-0.190452, -0.458323, 0.083647)`. The GUI frozen-frame target base value
  reported about `(-0.200, -0.473, 0.094)`. The delta is about
  `(9.5 mm, 14.7 mm, -10.4 mm)`, with about `17.5 mm` horizontal error.
- Tabletop geometry candidates currently use the geometry/OBB target center as
  `contact_center_base`; in the latest audit the first tabletop row carried
  `contact_center_base=(-0.190452, -0.458323, 0.083647)`. This can place the
  jaw-center line on the biased OBB/target-cloud center instead of the visual
  carton center.

Conclusion:
- Primary suspected cause of the present grasp offset is the software
  center-selection chain for 6D tabletop candidates: segmented RGB-D target
  center, target-cloud foreground, OBB center, and candidate contact center are
  not staying coincident. The measured 17.5 mm horizontal split matches the
  visible physical offset.
- Hand-eye calibration can still contribute global error, but it is not the
  first explanation for this specific evidence because both perception and 6D
  target-cloud estimates are derived from the same RGB-D camera/TF chain, yet
  they disagree with each other.
- TCP calibration is also not the first explanation at this stage. The current
  execution uses MoveIt/URDF `tool0` plus fixed analytical gripper constants
  measured from the checked-in 50 mm gripper URDF/STL. No new TCP solve result
  was generated or loaded during this work.

Next fix direction:
- Keep the target pose fixed. Change candidate generation/scoring so the jaw
  center is explicitly aligned to the robust segmented-object center/center
  normal, while still searching arbitrary grasp orientations and selecting the
  best reachable, collision-free, MuJoCo-passing posture.

## 2026-07-24 23:00 PDT - frozen pregrasp continuation and center-normal ranking

Revised diagnosis from the full execution history:
- The earlier visible-point-median contact-center bug was already corrected on
  2026-07-23: tabletop proposals now use the support-anchored OBB center, and
  later execution audits showed `contact_center_base` equal to the OBB center.
- The latest operator photo was captured at the frozen pregrasp, not at the
  final grasp pose. A 10 degree inclined approach places the pregrasp about
  6 mm off the support-normal line by construction, then returns the jaw center
  to the contact-center normal at the final grasp pose.
- The freeze was a separate near-field handoff failure: after reaching the
  far-field pregrasp, the remote node checked each new near-field candidate's
  newly generated pregrasp. Those poses could all fail MoveIt even though the
  robot had already reached a usable pregrasp, so no fresh rich plan was handed
  back to the execution node.

Code changes:
- `remote_grasp6d_node.py`
  - During active near-field execution, strict MoveIt candidate screening now
    uses the already-reached execution pregrasp.
  - A promoted near-field Preview inherits that reached pregrasp and recomputes
    the rich `plan_id`, so MuJoCo and the task node validate the exact path that
    will execute.
  - The audit records whether the pregrasp was inherited.
- `grasp_task_node.py`
  - When the rebound plan contains the same reached pregrasp, execution skips a
    duplicate pregrasp motion and continues to the fresh approach/grasp poses.
- `grasp6d_pipeline.py` and `remote_grasp6d_node.py`
  - Added bounded `final_approach_lateral_m` soft cost.
  - A reachable center-normal/zero-sweep pose now outranks an otherwise
    equivalent inclined pose. Inclined task poses remain available when the
    center-normal pose is not reachable.
- `tabletop_geometry_candidates.py`
  - Materialization now explicitly verifies that the analytical finger-pair
    center lies on the proposal contact-center support normal.
  - `finger_pair_center_base` and
    `finger_pair_center_lateral_error_m` are included in candidate audits.
- `grasp_params.yaml`
  - Persisted the center-normal ranking weight/knee.
  - Restored the measured-throughput settings to `request_hz=0.1` and
    `result_max_age_sec=15.0`. The previous live `1.5 Hz / 5 s` settings were
    replacing requests faster than the 6-8 s planning path could finish and
    were a direct cause of missing/expired candidate results.

Verification:
- Python syntax checks passed for both ROS nodes and both modified grasp
  modules.
- `test_grasp6d_pipeline.py` plus
  `test_tabletop_geometry_candidates.py`: `189 passed`.
- `test_remote_grasp6d_streaming.py`: `172 passed`.
- `test_grasp_task_sequence.py`: `97 passed`.
- `test_remote_grasp6d_node.py`: `150 passed`.
- Three focused production-config regressions also passed.
- Total reported focused/regression results in this change: `611 passed`.
- `git diff --check` passed.

Live reload and planning-only evidence:
- Reloaded only `/grasp_task_node` and `/remote_grasp6d_node` from the current
  protocol-v3 worktree. The arm driver, motion gateway, main launch, and arm
  enable state were not changed.
- `/grasp/state` returned `IDLE`, `active=False`, `message="ready"`.
- Planning-only `/grasp_6d/request_plan trigger=true` is running at `0.1 Hz`;
  `/grasp/start` was not called.
- Live candidate audits measured
  `finger_pair_center_lateral_error_m` around `1e-17 m`, effectively zero at
  floating-point precision.
- 20 degree candidates with about 14 mm final lateral sweep continued to be
  rejected by the 10 mm hard gate.
- The latest completed batch generated 16 locally valid tabletop candidates,
  stabilized 12, and strictly checked 24 pose variants in MoveIt. All 24 are
  currently `MOVEIT_UNREACHABLE` from the present joint posture, so there is no
  executable Preview yet. This is no longer a missing-candidate or center-line
  generation failure.

Next operator step:
- Move only the arm/camera to a new viewing/joint posture while keeping the
  physical target where it is, then report `已对准`.
- Candidate streaming is already running. After alignment, inspect the fresh
  selected pose for zero/minimal final lateral sweep, run MuJoCo, and only then
  decide whether to start a real grasp.

Safety note:
- No `/grasp/stop`, torque-off, disable, arm-stop, or arm health/self-check
  command was issued.

## 2026-07-25 00:54 PDT - immediate pregrasp abort traced to stale-controller bridge

Latest bound execution:
- Planning-only generation selected and promoted exact rich plan
  `1a91f944c15dc391ea2508b2`; the task node acknowledged it as
  `validation=VALID`.
- The content-bound MuJoCo execution gate passed with score `100`.
- Strict pregrasp planning succeeded for target
  `(-0.154, -0.465, 0.118) m`, with joint path cost `2.344` and maximum
  joint delta `1.723 rad`.
- No physical pregrasp path was executed. MoveIt rejected the cached trajectory
  during its start-state validation about `18 ms` after the execution request:
  `Invalid Trajectory: start point deviates from current robot state more than
  0.05`; `Joint2 expected=0.529223 current=0.477068`, a
  `0.052155 rad` mismatch.
- The task therefore never reached near-field re-planning, approach, close, or
  lift. The arm remained enabled and the grasp service returned `failed`.

Root cause:
- The pre-execution synchronizer published a one-point `0.30 s` trajectory to
  `/alicia_controller/command`. ROS
  `JointTrajectoryController::initJointTrajectory()` bridges a new non-empty
  command from its currently cached desired trajectory, not directly from
  current hardware feedback.
- The previous desired endpoint was consequently replayed for a short interval.
  During this window the driver logged a target near the old pregrasp endpoint
  (`Joint2=-49.9 deg`) even though feedback was near `+30.3 deg`.
- The synchronizer accepted one sample at `0.030680 rad`, but `25 ms` later
  Joint2 had crossed MoveIt's fixed `0.05 rad` start-state gate. This is a
  desired-state race, not a candidate IK failure, hand-eye/TCP error, MuJoCo
  rejection, or measured over-temperature shutdown.

Fix in progress:
- Replace the non-empty bridge trajectory with the controller's native empty
  trajectory current-feedback hold. In the installed Noetic controller this
  invokes `setHoldPosition()`, whose hold builder reads each hardware joint's
  current position directly.
- Require desired and actual errors to remain within `0.02 rad` for a
  continuous `0.30 s` window before cached execution. A single transient
  in-tolerance sample can no longer authorize MoveIt execution.
- This synchronization keeps the trajectory controller running and does not
  change joint enable, torque, or driver state.

Safety note:
- No `/grasp/stop`, torque-off, disable, arm-stop, controller stop, or arm
  health/self-check command was issued.

## 2026-07-25 01:20 PDT - pregrasp execution recovered; near-field sequence mismatch isolated

Real execution result:
- The one-controller-period desired-state refresh worked on hardware. The
  synchronization gate passed with maximum desired/actual error
  `0.012272 rad <= 0.030000 rad` continuously for `0.303 s`.
- Strict pregrasp execution was retimed to `32.597 s` with maximum joint delta
  `1.630 rad`; MoveIt returned `SUCCEEDED`. The arm reached the exact bound
  pregrasp around `(-0.154, -0.463, 0.118)` and remained enabled.
- The task then entered `PLAN_PREGRASP waiting for near-field 6D preview`.
  It never executed approach, gripper close, or lift. At `01:17:33` it failed
  with `NEAR_FIELD_REPLAN_TIMEOUT` after the configured `150.0 s`.

Near-field evidence:
- Requests `31`, `32`, `34`, `36`, `37`, and `39` each produced a full
  shortlist of 24 stable candidates, so candidate generation is working.
  Every batch ended with `moveit_reachable passed=0 rejected=24`; no Preview
  was published and therefore no physical approach was attempted.
- Motion-gateway logs show the first remaining-stage check for all candidates
  was the low approach pose, typically around
  `(-0.167, -0.489, 0.099)`. Every strict plan timed out before a grasp-pose
  check could run. This classifies the current `MOVEIT_UNREACHABLE` failures as
  near-field `approach` failures.
- The attached `6a1516228ae0501e0e99f578b8bdac4a.mp4` is `41.6 s`,
  `720x1280`, `30 fps`. Sampled frames show only the smooth pregrasp motion and
  the final stationary wait; the target remains visible, the gripper never
  closes, and no contact occurs. The `01:17` camera screenshot likewise keeps
  the carton fully inside the RGB and depth views.

Root cause:
- Each new near-field candidate contains its own higher pregrasp pose with the
  same candidate orientation. `grasp_task_node.py` already supports moving from
  the reached far-field pregrasp to that candidate-specific pregrasp before
  approach.
- `remote_grasp6d_node.py` currently defeats that capability by replacing the
  candidate-specific pregrasp with the old reached pregrasp. It then asks
  MoveIt to rotate directly from the old posture into the low approach pose.
  That is not the sequence that should be executed and explains the systematic
  24/24 approach rejection.
- The correction is to retain the candidate-specific pregrasp and validate the
  actual ordered path: current state to new pregrasp with strict pose planning,
  then pregrasp to approach and approach to grasp with collision-aware
  Cartesian planning from each preceding virtual end state. Only a completely
  valid sequence may publish a near-field Preview.

Safety note:
- No `/grasp/stop`, torque-off, disable, arm-stop, controller stop, or arm
  health/self-check command was issued.

## 2026-07-25 00:35 PDT - controller-synchronized pregrasp succeeded; near-field reachability is now the blocker

Successful real pregrasp:
- After the controller desired-state synchronization fix, generation 13
  produced plan `58ee8ed2af3538e43d589bf6`. The task node acknowledged this
  exact ID as its current rich plan with `validation=VALID`.
- The content-bound MuJoCo gate passed with score `100`.
- The pre-execution hold synchronized the running trajectory controller to
  current joint feedback. Strict execution was then retimed to
  `duration=33.653 s`, `joint_max_delta=1.683 rad`, and
  `start_error=0.029146 rad`.
- `/motion_gateway` returned success for the complete strict pregrasp path.
  The task node observed stable settling for `0.90 s` and entered
  `PLAN_PREGRASP waiting for near-field 6D preview`.
- No `PATH_TOLERANCE_VIOLATED` or residual post-failure endpoint motion
  occurred in this attempt. This confirms that the previous freeze was caused
  by the stale ros_control desired endpoint, and that the execution-layer fix
  works on the real arm.
- Video `1fddd2c941ca013bb934d5df0a191e59.mp4` is `32.5 s`,
  `720x1280`, and records the full smooth motion from the far posture to the
  pregrasp freeze above the carton.

Near-field failure after successful pregrasp:
- The task did not execute approach, gripper close, or lift. It waited for a
  fresh close-range rich plan and then failed with
  `NEAR_FIELD_REPLAN_TIMEOUT: no fresh near-field 6D preview after 90.0s;
  last=NEAR_FIELD_PLAN_UNCHANGED`.
- Close-range requests currently require about `30-43 s` per complete
  stability/MoveIt batch. The near-field timeout was therefore increased from
  `90.0 s` to `150.0 s`; the matching focused tests passed
  (`4 passed, 93 deselected`), and the live ROS parameter is updated.
- The operator screenshot at the pregrasp confirms that the carton remains
  fully visible in both RGB and depth. Detection is stable near pixel
  `(402, 430)`, depth `0.127 m`, confidence `0.910`, and base position
  approximately `(-0.166, -0.490, 0.094) m`. The failure is not target loss.

Latest candidate audit:
- The close-range target cloud center is approximately
  `(-0.1663, -0.4897, 0.0851) m`; the support normal is approximately
  `(0.0021, 0.1583, 0.9874)`.
- Candidate gripper-center lateral error relative to the target center normal
  remains effectively zero (`~1e-17 m`), so candidate generation is still
  centered on the object rather than carrying the old visual offset.
- Stable candidates are being generated, but the latest batches report all
  `24/24` strict pregrasp pose variants as `MOVEIT_UNREACHABLE`. A typical
  15-degree pregrasp is near `(-0.167, -0.484, 0.120) m`, while the reached
  tool pose is approximately `(-0.154, -0.455, 0.096) m`.
- The stationary carton estimate shifted about `29 mm` farther in base `y`
  between the reachable far-field plan and the near-field estimate. The
  current blocker is therefore the near-field pose/orientation reachability
  boundary, not missing candidates, center-line scoring, MuJoCo validation, or
  controller execution.

Next diagnostic:
- Run planning-only strict MoveIt probes over a broader set of tabletop grasp
  tilts, jaw-flip branches, and pregrasp distances at the current near-field
  target. No pose execution will be requested during this search.
- Only a pose that preserves the CAD-derived contact center and the unchanged
  center/lateral-error hard gates may be promoted for another real attempt.

Safety note:
- The arm remained enabled and the running trajectory controller was not
  stopped.
- No `/grasp/stop`, torque-off, disable, arm-stop, or arm health/self-check
  command was issued.

## 2026-07-25 00:12 PDT - execution-plan synchronization made observable

Resume attempt:
- Generation 9, target epoch 25 reached `PREVIEW_READY` on request `66`.
  Eighteen candidates passed cross-frame stability, 24 strict pose variants
  were checked by MoveIt, four were reachable, and one Preview was selected.
- Planning-only streaming was frozen and the Preview was promoted. A subscriber
  attached before promotion captured the exact promoted rich plan
  `e89e26218ef75f5b09fd64dc`, with carton center about
  `(-0.157, -0.474, 0.084) m` and required opening about `41.8 mm`.
- Two `/grasp/start` calls with that exact plan ID were rejected before any
  movement with `PLAN_ID_MISMATCH: requested plan_id is not the current rich
  plan`. Waiting before the second call ruled out a short subscriber callback
  delay.
- ROS connection inspection confirmed that `/grasp_task_node` is connected to
  the single `/remote_grasp6d_node` publisher on both
  `/grasp_6d/plan_enriched` and `/grasp_6d/preview_plan_enriched`.

Synchronization/diagnostic fix:
- Added read-only `/grasp/current_plan` using `TriggerZero`. It reports the
  task node's actual cached execution `plan_id`, source timestamp, tombstone
  state, and current validation result.
- The execution-plan callback now logs every plan that is actually accepted
  into task-node authority, including `plan_id` and source timestamp.
- `PLAN_ID_MISMATCH` now reports both the requested and task-node current IDs.
  Promotion can therefore be followed by an explicit task-node acknowledgement
  instead of assuming that publishing alone granted execution authority.
- Python compilation passed and the full grasp-task sequence suite passed:
  `97 passed`.

Next action:
- Hot-restart only `/grasp_task_node`, generate a new Preview, promote it, then
  require `/grasp/current_plan` to return the same ID with `validation=VALID`
  before calling `/grasp/start`.

Safety note:
- The two rejected start calls caused no physical motion.
- No `/grasp/stop`, torque-off, disable, arm-stop, or arm health/self-check
  command was issued.

## 2026-07-25 00:20 PDT - stale ros_control desired endpoint isolated

Acknowledged-plan execution:
- Hot-restarted only `/grasp_task_node`; its new PID accepted the read-only
  `/grasp/current_plan` service. The old latched plan was correctly rejected as
  stale on startup.
- Generation 11, target epoch 27 reached `PREVIEW_READY` on request `73`.
- After promotion, task-node logs showed two accepted execution publications.
  The authoritative query returned
  `plan_id=86453312d5c50c7e71857f4d ... validation=VALID`; this confirmed that
  reading the Preview ID alone had caused the earlier mismatch.
- `/grasp/start` used the task-node-acknowledged ID and passed its content-bound
  MuJoCo gate at score `100`.
- Strict MoveIt planned a two-state path with
  `joint_max_delta=1.728 rad`. Execution-time retiming produced
  `duration=34.566 s` and measured `start_error=0.000000 rad`.
- Despite the correct start state and timing, FollowJointTrajectory aborted
  about 28 ms later with
  `PATH_TOLERANCE_VIOLATED: Joint2 path error -1.336281`.

Physical/log correlation:
- Immediately after the new action was submitted, the hardware driver first
  received the previous trajectory endpoint
  `Joint2=-51.9 deg`, then received values close to current feedback around
  `Joint2=22.1 deg`.
- The controller action therefore evaluated one stale desired endpoint before
  the new interpolation became active. That single approximately `1.34 rad`
  discrepancy exceeded the unchanged `0.12 rad` path tolerance and aborted ROS
  execution even though the serial driver continued slewing toward the old
  endpoint.
- Video `ad226bdf0d162e3af56a853282a4d175.mp4` is 37 seconds long and confirms
  this exact behavior: service failure occurs early, the arm continues its
  residual pregrasp move, and it freezes above the carton without insertion,
  close, or lift.
- After the residual motion, `/alicia_controller/state` still reported the old
  endpoint as `desired`; actual feedback had eventually converged to within
  about `0.030 rad`. This is an execution-controller state problem, not RGB-D,
  hand-eye, TCP, candidate center, MoveIt reachability, or trajectory timing.

Controller-state synchronization fix:
- Added a current-feedback hold trajectory on
  `/alicia_controller/command` immediately before every strict cached
  execution. It updates JointTrajectoryController's internal desired state
  without stopping the controller or changing arm enable state.
- The hold uses current `/joint_states`, requires feedback age at most `0.5 s`,
  and waits up to `2.0 s` for both controller desired and actual positions to
  match the hold within `0.05 rad`.
- Strict physical execution fails closed before submitting the action if the
  hold is not acknowledged.
- Added joint-feedback receipt timestamps to `JointCommander`.
- Python compilation and the motion gateway, gripper preservation, MoveIt
  planner, and trajectory configuration suites passed: `50 passed`.

Safety note:
- No controller-stop request, `/grasp/stop`, torque-off, disable, arm-stop, or
  arm health/self-check command was issued.

## 2026-07-24 23:23 PDT - workspace-edge tilt sampling gap

Live evidence after the operator reported `已对准`:
- Target detection remained stable at about
  `base=(-0.189, -0.453, 0.092)`, with the carton fully inside the RGB-D image.
- Fresh request `71` and later requests produced 16 stable tabletop candidates,
  but the existing `0/10 degree` candidates had no IK at pregrasp. The
  configured `20 degree` candidates were rejected before MoveIt because their
  40 mm pregrasp generated about 14 mm lateral travel, above the fixed 10 mm
  gate.
- The current `tool0` pose was about
  `(-0.105, -0.245, 0.207)`. Historical runtime evidence showed strict plans
  succeeding near `y=-0.445`; the current target near `y=-0.455` is at the
  workspace/orientation boundary.

Planning-only diagnosis:
- Direct `/compute_ik` scans used the same CAD-grounded contact center and
  support plane. No arm motion was requested.
- `0-14 degree` candidates had no IK for pregrasp, approach, or grasp.
- At `15 degrees`, the reachable tilt polarity and its equivalent jaw-flipped
  branch had IK for all three stages, both with and without collision checking.
- A 15 degree candidate with a 38 mm pregrasp passed strict MoveIt at all three
  stages:
  - pregrasp: `(-0.189, -0.453, 0.120)`
  - approach: `(-0.190, -0.459, 0.103)`
  - grasp: `(-0.191, -0.466, 0.085)`
- The best jaw-flipped pregrasp branch had
  `joint_max_delta=1.614 rad`, below the unchanged `2.15 rad` hard gate.
- The 38 mm/15 degree lateral travel is about `9.835 mm`, below the unchanged
  10 mm hard gate. Analytical finger-center normal error remained about
  `3.3e-17 m`.

Configuration fix:
- Changed production tabletop tilt samples from `[10, 20]` to `[10, 15]`.
  This closes the measured reachability gap without allowing the already
  rejected 20 degree lateral sweep.
- Changed `grasp.pregrasp_distance_m` from `0.040` to `0.038`. Final approach
  and contact poses remain CAD-derived and centered; the target pose is not
  translated.
- Updated the code fallback tilt defaults and production fixture/config tests
  to match.

Verification so far:
- `remote_grasp6d_node.py` compiles.
- Focused config assertions: `2 passed`.
- Production RealSense tabletop fixture assertion: `1 passed`.
- Tabletop runtime-config node tests: `4 passed`.

Safety note:
- Every direct MoveIt/IK probe was planning-only or read-only.
- No `/grasp/start` has been called yet.
- No `/grasp/stop`, torque-off, disable, arm-stop, or arm health/self-check
  command was issued.

## 2026-07-24 23:49 PDT - real pregrasp freeze and strict trajectory retiming

Final candidate configuration and validation:
- Reduced `grasp.pregrasp_distance_m` from the intermediate `0.038 m` to
  `0.036 m`. The measured 15 degree pregrasp-to-grasp lateral travel is about
  `9.317 mm`, below the unchanged `10 mm` hard gate.
- Reduced tabletop tilt samples from the intermediate `[10, 15]` to `[15]`.
  Vertical candidates are still always generated separately; removing the
  unreachable 10 degree variants prevents them from consuming the bounded
  24-slot MoveIt shortlist before the reachable 15 degree variants.
- Focused production config, RealSense fixture, and runtime config tests all
  passed (`7 passed` total for that final candidate configuration).
- Live request `3` checked 24 strict MoveIt variants, found 4 reachable, and
  published a valid `PREVIEW_READY`.

First real execution:
- Promoted fresh execution plan `3d221b139d2e6898729a181e`.
- Direct no-motion MuJoCo probing returned score `100` with
  `simulation_ok`, `ik_success`, `collision_free`, `contact_success`, and
  `lift_success` all true.
- `/grasp/start execute=true` passed the same MuJoCo gate, then the MoveIt
  controller returned `CONTROL_FAILED` during the strict pregrasp segment.
- The arm nevertheless moved from about
  `tool0=(-0.105, -0.245, 0.207)` to
  `tool0=(-0.186, -0.449, 0.099)` and then froze before approach, close,
  and lift.
- Video `3eaeb72dd5d7dea15e0823ca55cf865e.mp4` confirms the far-field motion
  followed by the pregrasp freeze.

Near-field retry:
- Close-range perception shifted the stationary carton estimate from about
  `(-0.191, -0.459, 0.085)` to about
  `(-0.166, -0.465, 0.080)`, a roughly 25 mm lateral correction consistent
  with the visible center-line offset.
- An independent MuJoCo probe for plan `bff69bddfed647550725207b` passed at
  score `100`, but `/grasp/start` correctly rejected it as `PLAN_STALE`
  before physical motion.
- A subsequent fresh plan `439e06f3035140c5804a627c` passed the built-in
  MuJoCo gate at score `100`. Its strict pregrasp controller action failed,
  but the hardware again continued moving and settled near
  `tool0=(-0.165, -0.451, 0.101)`.
- Video `7a8cb31ed744f33771a08676a53ede0c.mp4` shows this approximately 20 second
  physical motion continuing after the ROS service had already reported
  failure, followed by the same freeze before final insertion.

Controller root cause:
- The exact action result was
  `PATH_TOLERANCE_VIOLATED: Joint2 path error -1.272929`, followed by MoveIt
  `ABORTED: CONTROL_FAILED`.
- The first logged `/joint_commands` value was already near the strict
  trajectory endpoint while real feedback was still about 1.27 rad behind.
  The physical SDK accepts about 15 deg/s motion, so the under-timed cached
  trajectory exceeded the controller's `0.12 rad` path tolerance almost
  immediately even though the hardware continued following the commanded
  endpoint.
- This explains both symptoms together: genuine physical movement followed
  by an immediate grasp-task failure and no approach/close/lift stages.

Execution-layer fix:
- Added strict cached-path retiming immediately before physical execution.
  The MoveIt joint path geometry and endpoint must remain exactly unchanged.
- The retimed trajectory must have positive timing and a total duration at
  least `joint_max_delta / 0.08 rad/s`; an unavailable retiming API, changed
  path, empty trajectory, or shorter duration fails closed before motion.
- Production parameters:
  `strict_execution_retime_enabled=true`,
  `strict_execution_velocity_scaling=0.20`,
  `strict_execution_acceleration_scaling=0.30`, and
  `strict_execution_max_joint_velocity_rad_s=0.08`.
- Python compilation passed. MoveIt planner and trajectory config regression
  tests passed: `31 passed`.
- Hot-reloaded only `/motion_gateway` to PID `39476`. The arm driver,
  ros_control hardware interface, MoveIt, camera, main launch, and arm enable
  state were not restarted or changed.

Safety note:
- Candidate-stream `trigger=false` was used only to freeze planning snapshots;
  it is not an arm command.
- No `/grasp/stop`, torque-off, disable, arm-stop, or arm health/self-check
  command was issued.

## 2026-07-25 00:04 PDT - stale trajectory start state isolated

Third real pregrasp attempt:
- Fresh plan `47c03a6b84cdb3c4d6f2d7ad` passed the built-in exact-plan MuJoCo
  execution gate at score `100`.
- Strict-path retiming ran, but the FollowJointTrajectory action still aborted
  almost immediately with
  `PATH_TOLERANCE_VIOLATED: Joint2 path error -1.463402`, followed by MoveIt
  `ABORTED: CONTROL_FAILED`.
- Video `2e2ca1feb084cc88d02092eb367f2557.mp4` records the resulting behavior:
  the grasp service had already returned failure while the physical controller
  continued moving for about 40 seconds toward the residual pregrasp endpoint.
  The arm then froze before insertion, close, and lift.

Refined root cause:
- Retiming alone did not remove the approximately `1.46 rad` first-command
  discrepancy, so speed was not the only fault.
- The cached strict trajectory was planned against a stale/default MoveIt start
  state. At execution time its first trajectory point did not match live Alicia
  joint feedback, immediately exceeding the controller's `0.12 rad` path
  tolerance.
- The hardware continued following the endpoint after the ROS action had
  aborted, which explains why every failed service call still produced a long
  physical pregrasp motion and then stopped before later grasp stages.

Execution start-state fix:
- Every MoveIt plan now calls `set_start_state_to_current_state()` before
  planning when that API is available.
- Immediately before strict cached execution, the retimed trajectory's first
  point is compared with live `get_current_joint_values()` feedback.
- Execution is rejected before any motion when the maximum joint start error is
  above `strict_execution_start_tolerance_rad=0.08`. This threshold remains
  below the controller's `0.12 rad` path tolerance and returns an explicit
  `replan from current joint feedback` diagnostic.
- A permitted trajectory logs duration, maximum joint delta, velocity limit,
  and measured start error so the next real attempt is directly auditable.
- Python compilation and focused MoveIt planner/config regression tests passed:
  `32 passed`.
- Only `/motion_gateway` was hot-reloaded, to PID `40270`, so the new planner
  logic and ROS parameters are live. The arm driver, ros_control, MoveIt,
  camera, main launch, and joint enable state were not restarted or changed.

Resume state:
- After a stable read-only joint observation, generation 7 produced fresh
  `PREVIEW_READY` results at requests `58` and `60`.
- Continuous candidate generation was then frozen with planning-only
  `/grasp_6d/request_plan trigger=false`, and the cached Preview was promoted.
- The task node did not expose a current rich plan after promotion; its last
  visible grasp state was the preceding failed execution, and recent task-node
  messages included `TARGET_LOST`. Therefore `/grasp/start` was deliberately
  not called with an unverified or stale plan.
- Work paused at the operator's request with candidate streaming off and no
  physical execution pending. On resume, generate a new target epoch and require
  a fresh rich plan to be accepted by the task node before promotion/execution.

Safety note:
- No `/grasp/stop`, torque-off, disable, arm-stop, or arm health/self-check
  command was issued.

## 2026-07-25 00:44 PDT - near-field handoff succeeded; remaining-stage reachability gap isolated

Fourth controller-synchronized real attempt:
- Planning stream generation 15 produced and promoted
  `9de5e55ed977b26509138c57`; `/grasp/current_plan` acknowledged the exact
  plan ID with `validation=VALID`.
- Initial target geometry was centered near
  `(-0.1553, -0.4695, 0.0809) m`; initial pregrasp was
  `(-0.1542, -0.4632, 0.1150) m`.
- The content-bound MuJoCo gate passed at score `100`.
- Controller desired-state synchronization passed with maximum hold error
  `0.047553 rad`. Strict pregrasp execution was retimed to `31.319 s` with
  `joint_max_delta=1.566 rad` and `start_error=0.047553 rad`.
- The complete pregrasp path succeeded without
  `PATH_TOLERANCE_VIOLATED`, the arm settled, and the task entered the required
  near-field Preview wait.
- Video `a2ecbe738e6e0da4f64d047b35f6cc61.mp4` is `28.6 s`,
  `720x1280`, and records this smooth pregrasp segment. Contact-sheet review
  shows continuous motion toward the carton followed by a stationary pregrasp;
  it does not show approach, close, lift, or post-failure residual movement.
- The operator's close-range GUI screenshot shows the carton fully visible at
  pixel about `(390,427)`, depth `0.126 m`, confidence `0.894`, perception
  base position about `(-0.167,-0.491,0.094) m`, and visual-fallback pregrasp
  about `(-0.159,-0.496,0.173) m`. This matches the near-field coordinate
  change in ROS logs and rules out target disappearance at the freeze.

Near-field result:
- The live `150 s` timeout allowed the close-range pipeline to complete. A
  fresh Preview was rebound after about `38 s` as plan
  `ff4d87c26c33a70ddf915c5a`; the inherited already-reached pregrasp passed
  every strict MoveIt check with only about `0.026-0.029 rad` joint delta.
- Before any approach motion, gripper close, or lift, the mandatory MuJoCo gate
  rejected the rebound plan with
  `MUJOCO_IK_FAILED: IK failed at approach: position error 0.0333m orientation
  error 0.1018`.
- The task released its execution slot and the arm remained at the reached
  pregrasp. No post-failure residual motion occurred.

New root cause:
- During active near-field rebinding,
  `_check_moveit_stable_candidate()` intentionally substitutes the currently
  reached pregrasp for each candidate's newly generated pregrasp. This prevents
  an unnecessary second pregrasp move.
- That substitution also made candidate selection validate only the identical
  reached pose. It did not validate each candidate's new approach and grasp
  poses, so all near-field candidates appeared reachable even when the selected
  candidate's approach IK was invalid.
- The timeout and pregrasp inheritance fixes are working. The remaining
  software gap is stage-incomplete reachability checking for the actual
  remaining near-field motion.

Next fix:
- When execution is active, score a stable near-field candidate only if both
  its approach and grasp poses pass planning-only strict MoveIt checks from the
  current settled posture. Keep the existing pregrasp-only strict cache for
  idle/far-field selection.
- Add regression tests proving that an approach or grasp failure rejects the
  near-field candidate before Preview publication and MuJoCo execution.

Calibration/TCP evidence:
- Runtime `tool0 -> camera_link` comes from the saved easy_handeye file
  `~/.ros/easy_handeye/d405_v4l2_charuco_handeyecalibration_eye_on_hand.yaml`,
  last modified on 2026-07-12. It was not generated by the current grasp work.
- No solved TCP calibration result exists under `calibration_results`; the
  checked-in `tcp_calibration.yaml` contains only calibration-task settings.
- The static carton's base estimate still changes with camera posture by about
  `20-30 mm`. This keeps hand-eye/extrinsic or close-range RGB-D localization
  bias as a real accuracy concern, but it did not cause this attempt's terminal
  failure: the terminal failure is the missing approach/grasp reachability
  filter described above.

Implemented remaining-stage filter:
- Idle/far-field candidate selection still performs one strict pregrasp check
  and preserves that exact cached trajectory for physical execution.
- While `/grasp/state.active=True`, each near-field stable candidate now checks
  its own approach pose and grasp pose with planning-only
  `/supervisor/check_pose_strict`. The candidate is rejected immediately with
  a stage-qualified diagnostic if either pose is unreachable.
- A successful near-field result aggregates both planning costs, takes the
  maximum joint delta, and carries
  `STRICT_REMAINING_STAGES_SUCCESS` evidence. It no longer ranks candidates by
  the identical already-reached pregrasp.
- Stream-ticket checks surround both planning calls and commit diagnostics only
  after the ticket remains current, preserving stop/epoch invalidation
  atomicity.
- Python compilation passed. The complete remote streaming suite passed
  (`174 passed`), and the remote-node plus 6D-pipeline suites passed
  (`315 passed`), for `489` passing regression tests.
- Hot-restarted only `/remote_grasp6d_node`; the fixed process is PID `50672`.
  Candidate generation was restarted. The arm driver, trajectory controller,
  MoveIt, camera, task node, motion gateway, and arm enable state were not
  restarted or changed.

Safety note:
- No `/grasp/stop`, torque-off, disable, arm-stop, or arm health/self-check
  command was issued.

## 2026-07-25 01:01 PDT - native feedback-hold synchronization deployed

Implementation and verification:
- `/motion_gateway` now publishes an empty `JointTrajectory` for strict
  pre-execution synchronization. In Noetic this selects the controller's native
  current-hardware-position hold instead of bridging from a stale desired
  endpoint.
- Synchronization now requires both controller desired and actual states to
  remain within `0.02 rad` of the captured feedback for a continuous `0.30 s`.
  Any out-of-band sample resets the settling window.
- Added regression coverage for the empty native-hold message, delayed
  acknowledgement, and settling-window reset after an error excursion.
- Python compilation and whitespace validation passed. Focused motion gateway
  tests passed (`18 passed`); MoveIt feedback and trajectory execution tests
  passed (`32 passed`).
- Runtime parameter
  `/robot/strict_execution_controller_sync_tolerance_rad` is live at `0.02`.
- Hot-reloaded only the respawn-managed `/motion_gateway`; the new PID is
  `52419`, with all expected services and controller-state connections online.
  The main launch, real-arm driver, ros_control hardware interface, trajectory
  controllers, MoveIt, task node, remote candidate node, camera, and arm enable
  state remained running.
- Current grasp state is inactive `FAILED` from the completed prior attempt, so
  there is no residual execution worker. The prior plan will not be reused; a
  fresh target-bound rich plan is required before the next real attempt.

Safety note:
- No `/grasp/stop`, torque-off, disable, arm-stop, controller stop, or arm
  health/self-check command was issued.

## 2026-07-25 01:07 PDT - stale bridge removed; settle band matched to real quantization

Fresh-plan attempts:
- Generation `3` produced `PREVIEW_READY` plan
  `7cfbbad48781cafc668f8a10`, but the task rejected it as `PLAN_STALE` before
  any execution because its source stamp crossed the configured `120 s`
  validity window during inspection.
- Generation `5` then produced `PREVIEW_READY` plan
  `c4aa01d41413c923e0a8a27c`; 2 of 24 strict MoveIt candidates were reachable
  and one was promoted. The exact current plan was `validation=VALID` when
  submitted.
- Its content-bound MuJoCo check passed with score `100`. Strict pregrasp
  planning also passed with joint path cost `2.200` and maximum joint delta
  `1.624 rad`.
- The new native controller feedback hold correctly prevented stale-endpoint
  replay: no old pregrasp endpoint appeared on `/joint_commands`, and no
  pregrasp trajectory was sent to MoveIt.
- Execution was deliberately blocked at the synchronization gate because the
  settled real-hardware quantization error remained `0.026078 rad`, above the
  provisional `0.02 rad` band for the full `2.0 s` timeout.

Diagnosis and adjustment:
- The prior failure mechanism is removed: the controller no longer bridges
  through an old desired endpoint, and the arm stayed around its initial
  alignment (`Joint2` about `29.4 deg`, `Joint3` about `3.3 deg`).
- The remaining `0.026078 rad` is consistent with the already measured
  Alicia-D endpoint quantization (`0.026-0.030 rad`), not continuing motion.
- Set the synchronization tolerance to `0.03 rad`, while retaining the
  continuous `0.30 s` settling window. This remains below MoveIt's fixed
  `0.05 rad` start-state gate and cannot pass on a single transient sample.
- Plan `c4aa01d41413c923e0a8a27c` will not be retried; a fresh target-bound plan
  is required after the runtime parameter update.

Safety note:
- No `/grasp/stop`, torque-off, disable, arm-stop, controller stop, or arm
  health/self-check command was issued.

## 2026-07-25 01:10 PDT - controller desired-state refresh made explicit

Generation `7` result:
- Fresh plan `9cfceddebe55c235a63cc57e` was submitted while
  `validation=VALID`. MuJoCo passed with score `100`, and strict pregrasp
  planning passed with path cost `2.127` and maximum joint delta `1.573 rad`.
- Again, no pregrasp trajectory was sent. The synchronization gate blocked
  before MoveIt because the error remained `0.038350 rad` for the full `2 s`.
- Read-only controller state isolated the error to `Joint3`: desired
  `0.082835 rad`, actual `0.044485 rad`; Joint2 differed by `0.023010 rad`.
  Real SDK feedback was steady near Joint2 `29.0 deg`, Joint3 `2.5 deg`.

Correction to the native-hold assumption:
- On this deployed PositionJointInterface controller instance, the empty
  command did not replace the stale desired state. The state timestamp kept
  updating, but desired remained at the older Joint2/Joint3 values.
- Do not widen the synchronization band toward MoveIt's `0.05 rad` validation
  threshold. The gate must first prove that desired was actually refreshed.

Implemented follow-up:
- Publish one explicit current-feedback trajectory point with a `0.02 s`
  duration, exactly one 50 Hz controller period. This replaces the stale
  desired target without the former `0.30 s` bridge through the old endpoint.
- Keep the independent `0.30 s` continuous settling requirement and
  `0.03 rad` error band after the one-cycle refresh. Any later error excursion
  still resets the settling timer.
- Add regression assertions for the explicit feedback positions, zero
  velocities, one-point message, and `0.02 s` bridge duration.

Safety note:
- No `/grasp/stop`, torque-off, disable, arm-stop, controller stop, or arm
  health/self-check command was issued.

## 2026-07-25 01:28 PDT - operator proposals for coarse-to-fine visual closure

Operator proposals recorded verbatim in technical form:
- Treat the first GraspNet result only as coarse target approach authority,
  instead of allowing one RGB-D frame to determine the final contact pose.
  Suggested observation/pregrasp distances are `12-15 cm` for large objects,
  `8-12 cm` for medium/small objects such as a mouse or carton, and about
  `10 cm` for small or depth-noisy targets.
- At the observation pose, fuse `5-10` consecutive RGB-D frames before the
  second localization because a static target can still vary by millimetres to
  centimetres in a single-frame estimate.
- Consider continuous closed-loop feedback during approach. At least recheck
  once at `3-5 cm` before contact, updating only grasp-center `x/y/z`, vertical
  `yaw`, mask center, and surface depth. Freeze roll/pitch, bound translation
  and yaw corrections, and forbid switching to an unrelated grasp candidate.

Decision:
- Adopt a staged coarse-to-fine closed loop. First make the far-field
  pregrasp an approximately `10 cm` observation pose for this carton, then
  collect a stable `5-10` frame near-field window and select the best
  candidate. Preserve that candidate's own pregrasp and validate the ordered
  pregrasp/approach/grasp sequence before any new motion.
- Add the final `3-5 cm` same-candidate correction as the next layer after the
  ordered sequence planner is proven. It must be bounded `xyz + yaw` only,
  keep roll/pitch frozen, retain candidate lineage, and pass the same collision
  and reachability gates before execution.
- Do not start with unrestricted continuous visual servoing. The current
  hand-eye/depth estimate has measured posture-dependent drift, so feeding
  every frame directly into motion could amplify jitter and candidate
  switching. A staged stable-window update gives auditable corrections and a
  clear failure boundary.

Current visual evidence:
- The target remains detectable at the reached pregrasp, but it is close to
  the lower image boundary and does not have the generous observation margin
  needed for arbitrary candidate orientations. This can degrade near-field
  estimation, but it is not the sole cause of the last timeout: the immediate
  deterministic failure was the old-pregrasp substitution followed by direct
  low approach planning.

Safety note:
- No `/grasp/stop`, torque-off, disable, arm-stop, controller stop, or arm
  health/self-check command was issued.

## 2026-07-25 01:49 PDT - observation and contact pregrasp distances separated

Planning-only runtime evidence:
- A fresh manual-trigger stream collected the new five-frame RGB-D window
  without invoking `/grasp/start` or publishing a trajectory.
- With the provisional single `0.100 m` pregrasp value, 15 degree tabletop
  variants produced about `0.026 m` of pregrasp-to-grasp lateral travel and
  were correctly rejected by the existing `0.010 m` final-approach gate.
- The remaining almost-vertical variants requested pregrasp poses around
  `xyz=(-0.155,-0.462,0.180)` and all failed strict MoveIt reachability in the
  current workspace. This explains why the coarse-distance change alone could
  not publish a preview: one distance was incorrectly serving two different
  stages.

Correction:
- `observation_pregrasp_distance_m=0.100` now belongs only to the first,
  far-field observation move. Its MoveIt check and published initial rich plan
  use that observation sequence.
- `pregrasp_distance_m=0.036` now belongs to candidate-specific contact
  planning and the active near-field ordered
  `pregrasp -> approach -> grasp` check. At 15 degrees its lateral travel is
  about `0.0093 m`, so the original `0.010 m` bounded-approach gate remains in
  force instead of being widened.
- Candidate geometry, support-plane checks, and scoring use the close-range
  sequence. Far-field visibility and reachability use the separate observation
  sequence. The selected plan audit records both distances and whether it is a
  far-field observation plan.

Verification:
- Python compilation passed.
- Focused configuration and streaming tests passed (`180 passed`).
- The broader task, remote 6D, MoveIt, motion-gateway, pipeline, and tabletop
  geometry suites passed (`664 passed`, five existing rospy deprecation
  warnings).

Safety note:
- This phase was planning-only. No `/grasp/start`, `/grasp/stop`, trajectory,
  torque-off, disable, arm-stop, controller stop, or arm health/self-check
  command was issued.

## 2026-07-25 01:54 PDT - split-distance runtime preview proven

Runtime hot update:
- Reloaded the grasp parameters and hot-restarted only
  `/remote_grasp6d_node`; the new PID is `65624`.
- `/grasp/pregrasp_distance_m=0.036` and
  `/grasp/observation_pregrasp_distance_m=0.100` are live. The real-arm driver,
  ros_control hardware interface, trajectory controllers, MoveIt, motion
  gateway, task node, camera, and arm enable state were not restarted or
  changed.

Fresh planning-only stream:
- Batch `1` passed `18/30` local candidates and entered
  `STABILITY_PENDING`; the prior `TABLETOP_APPROACH_LATERAL_SWEEP` rejection
  disappeared without widening the `0.010 m` gate.
- Batch `3` formed the stable window, checked 24 candidates with strict
  MoveIt, found two reachable, and published `PREVIEW_READY`. Batch `5`
  independently repeated two reachable candidates and `PREVIEW_READY`.
- The published rich plan `7b4ba31941d4bed9c714f371` records
  `fused_frames=5`. Its first pose is the far-field observation pose around
  `(-0.153,-0.447,0.180)`, while approach and grasp remain around
  `(-0.157,-0.476,0.106)` and `(-0.158,-0.484,0.087)`.
- Motion-gateway evidence confirms the observation pose and 15 degree
  orientation are strictly reachable. The selected branch planned with
  `joint_path_cost=2.497` and `joint_max_delta=1.853 rad`.
- Some concurrent frames expired after TF buffer time jumps, but successful
  five-frame fused previews repeated despite that environmental noise.

Conclusion:
- The earlier candidate-generation regression was caused by conflating the
  coarse observation standoff with the final candidate pregrasp standoff.
  Separating them restores arbitrary reachable tilted candidates while keeping
  the bounded final-approach gate intact.

Safety note:
- No real execution was requested. No `/grasp/start`, `/grasp/stop`,
  trajectory, torque-off, disable, arm-stop, controller stop, or arm
  health/self-check command was issued.

## 2026-07-25 03:20 PDT - bounded final 36 mm visual refinement implemented

Pre-implementation proof:
- Reconstructed the contact sequence for published plan
  `7b4ba31941d4bed9c714f371` and called only
  `/supervisor/check_pose_sequence_strict`.
- The ordered current-state to `36 mm` pregrasp, `20 mm` approach, and grasp
  sequence planned successfully with `joint_path_cost=2.871` and
  `joint_max_delta=1.882 rad`. No trajectory was executed.

Final refinement layer:
- After the arm reaches the candidate-specific `36 mm` pregrasp, the task now
  requests one fresh close-range Preview window before the last `20 mm`.
- A Preview is eligible only when target label, model choice, candidate source,
  and candidate source lineage remain identical to the already bound plan.
- The observed grasp-center correction is limited to `12 mm`; support-normal
  yaw correction is limited to `10 deg`; an observed roll/pitch change over
  `4 deg` is rejected. Accepted output keeps the prior roll/pitch exactly and
  applies only the bounded xyz and yaw correction to all four poses.
- A corrected rich plan receives a new canonical content-derived plan ID,
  retains the latest object geometry snapshot, and uses the more conservative
  of the old/new required gripper widths.
- Before authority is rebound, the corrected
  `pregrasp -> approach -> grasp` sequence must pass
  `/supervisor/check_pose_sequence_strict`. The task then performs at most the
  small corrected-pregrasp move and continues with linear approach only after
  all normal execution checkpoints pass.
- Unrelated candidate switches, large translation/yaw changes, roll/pitch
  jumps, malformed plans, stale previews, and failed strict planning remain
  fail-closed.

Configuration and tests:
- Enabled `final_visual_refine_enabled` and
  `final_visual_refine_required`, with a `150 s` stable-preview window matching
  measured near-field inference latency.
- Added tests for accepted xyz/yaw correction, frozen roll/pitch, translation,
  yaw and tilt limits, canonical plan integrity, and ordered strict-service
  arguments.
- Focused tests passed (`108 passed`). The broader task, remote 6D, MoveIt,
  motion-gateway, pipeline, and tabletop suites passed (`669 passed`, five
  existing rospy deprecation warnings). Python compilation and whitespace
  validation passed.

Safety note:
- No `/grasp/start`, `/grasp/stop`, trajectory, torque-off, disable, arm-stop,
  controller stop, or arm health/self-check command was issued.

## 2026-07-25 04:05 PDT - full latest ROS stack restarted for aligned-target trial

Launch:
- Started the complete current stack with:
  `roslaunch alicia_flexible_grasp_supervisor full_system.launch
  start_real_arm:=true driver_port:=/dev/alicia_arm
  driver_baudrate:=1000000 auto_torque_on_startup:=true
  self_check_poll_rate_hz:=0.0 start_camera:=true start_tactile:=false
  start_gui:=true use_remote_grasp6d:=true
  remote_grasp6d_url:=http://172.23.132.97:8000`.
- The live launch terminal session is `39107`; the ROS run log directory is
  `/home/zhuyupei/.ros/log/708b0e76-8818-11f1-a512-b1f26583047d/`.
- The real driver opened `/dev/alicia_arm`, requested automatic torque-on, and
  continuously returned real SDK joint, gripper, status, and temperature
  feedback. Both `alicia_controller` and `hand_controller` were loaded for
  trajectory execution.
- MoveIt loaded OMPL and Pilz, exposed the planning and Cartesian-path
  services, and reported `You can start planning now!`.
- The camera, carton segmentation, hand-eye transform, remote 6D planner,
  motion gateway, grasp task, safety monitor, logger, and GUI are online.
- The newly implemented final visual-refinement parameters are live in this
  launch.

Live perception before the user's alignment confirmation:
- Carton recognition stabilized near pixel `(313,276)`, depth `0.337 m`,
  base-frame center `(-0.156,-0.471,0.089)`, and confidence about `0.90`.
- The user confirmed that the target was aligned after this startup. The next
  operation will request a fresh planning-only 6D candidate window and inspect
  the five-frame fused result before any execution authority is used.

Safety note:
- No arm health/self-check was requested. No `/grasp/start`, `/grasp/stop`,
  trajectory, torque-off, disable, arm-stop, or controller-stop command was
  issued during startup and alignment.

## 2026-07-25 04:08 PDT - aligned trial exposed missing lift validation

Planning and execution request:
- After the user confirmed alignment, planning-only candidate generation
  reached `PREVIEW_READY`. The stable batch checked 24 candidates with strict
  MoveIt, found four reachable candidates, and selected one.
- The first direct `/grasp/start` request was rejected with
  `PLAN_ID_MISSING`; no motion occurred. The current rich plan ID was then
  read from `/grasp_6d/plan_enriched` and rebound explicitly as
  `88031471a510eec2a488057e`.
- The bound request entered the execution gate but was rejected before any
  real trajectory was sent. The MuJoCo audit is
  `/home/zhuyupei/.ros/grasp6d_mujoco_audit_latest.json`, with payload SHA-256
  `5cc9594c2ac1f179b2d75c40fe467cd06152a9c8ca169952449fd9307d4905ce`.

New failure and root cause:
- MuJoCo returned `MUJOCO_IK_FAILED` at the `lift` pose, with position error
  `0.0068 m` and orientation error `0.0203 rad`.
- The candidate publisher's ordered strict validation currently proves only
  `pregrasp -> approach -> grasp`. The rich plan contains a fourth `lift`
  pose, but that transition is not part of the publisher-side strict sequence.
  A candidate can therefore be advertised as `PREVIEW_READY` even though the
  execution gate later proves that its lift is not feasible.
- This is a planning-coverage defect, not evidence of a new RGB-D center
  offset. The contact plan must require the full ordered
  `pregrasp -> approach -> grasp -> lift` sequence before publication, while
  retaining the MuJoCo gate as independent defense.

Next action:
- Extend candidate-specific strict sequence validation to include the lift
  pose, add a regression test that rejects a candidate whose first three poses
  pass but lift fails, hot-update only the affected planning nodes, and request
  a new rich plan before retrying automatic execution.

Implemented resolution:
- The first implementation required a five-stage far-field sequence. Runtime
  first exposed the old strict-service maximum of four targets; that limit was
  extended to five and covered by a virtual-start-state regression test.
- After the service update, no five-stage far-field candidate was reachable:
  most failed at the observation pose, some at Cartesian grasp insertion, and
  the candidates that passed those stages failed at lift. This showed that
  forcing an old single-frame contact trajectory onto the coarse observation
  phase contradicts the intended two-stage localization design.
- The final architecture now gives each phase explicit authority. A far-field
  rich plan is marked `FAR_FIELD_OBSERVATION_PLAN` and proves only the `100 mm`
  observation pose. It may move only to that observation pose; the initial
  full-contact MuJoCo gate is deferred.
- Once the observation pose is reached, only a new
  `CONTACT_EXECUTION_PLAN` may be rebound. Its candidate-specific
  `36 mm pregrasp -> 20 mm approach -> grasp -> lift` sequence must pass
  `/supervisor/check_pose_sequence_strict`, and the same bound contact plan
  must pass MuJoCo before any contact motion continues.
- Final approach and insertion remain strict Cartesian transitions. Lift is a
  strict pose-planned transition during candidate screening and remains a
  linear executed transition after closure. No execution-capable service is
  used during candidate screening.
- Streaming audits record `observation_stage_checked`,
  `full_execution_sequence_checked`, and the exact strict stage list, so
  future evidence cannot silently conflate the two phases or omit lift.
- `grasp_task_node.py` now includes lift in the strict ordered check performed
  after bounded final visual refinement. The corrected plan cannot be rebound
  when its lift is not reachable.
- Regression coverage includes observation-only far-field planning, all four
  possible near-field failed stages including lift, deferred coarse-plan
  simulation, contact-plan phase binding, the five-target service contract,
  and the final-refinement four-stage service contract. Stream-generation and
  target-epoch invalidation barriers remain active for in-flight checks.
- Python compilation passed. The MoveIt planner, motion gateway, remote
  streaming, and grasp-task sequence suites passed (`326 passed`, five
  existing rospy deprecation warnings).

Safety note:
- The task failed before motion. No real trajectory, `/grasp/stop`,
  torque-off, disable, arm-stop, controller-stop, or arm health/self-check
  command was issued. The real driver and joint enable state remain online.

## 2026-07-25 04:56 PDT - first two-stage runtime exposed a cache race and preflight arm motion

Runtime result:
- The hot-updated remote node is running in terminal session `85074`; the
  task node is running in session `79353`; the full launch remains in session
  `39107`.
- A fresh stream reached `PREVIEW_READY`. Of 24 stable candidates, four were
  strictly MoveIt-reachable and one was promoted.
- The bound rich plan was `cc74468090be7fa36d7b7d59`, marked
  `FAR_FIELD_OBSERVATION_PLAN`. Its geometry used five fused frames and its
  observation pose was about `(-0.1514,-0.4471,0.1769) m`.
- `/grasp/start` was called with that exact plan ID. The task correctly
  deferred contact MuJoCo simulation and attempted only the observation
  pregrasp, but returned `failed` with:
  `strict cached execute blocked: no cached pose plan`.

New root cause:
- The task performs a strict planning request and a later cached execution
  request as two separate ROS service calls.
- Continuous candidate screening calls the same strict MoveIt planner between
  those requests. A later candidate check can replace or invalidate the single
  cached pose plan before the task executes it.
- Therefore candidate screening and physical execution currently race over
  one mutable planner cache. The required correction is an atomic strict
  plan-and-execute gateway operation, serialized against every other MoveIt
  planning operation.

Physical-motion correction:
- The initial interpretation that the failed task caused no physical motion
  was wrong. The operator observed several joints move, and driver feedback
  confirms it.
- Before task start, feedback was approximately
  `[-110.7,24.1,-2.7,0.3,-15.3,0.3] deg`.
- Opening the gripper published a full `/joint_commands` hold target at that
  arm position. Feedback then moved through about
  `[-110.7,23.6,-3.3,0.3,-15.3,0.3] deg`.
- The strict-execution controller synchronization published another hold at
  the changed feedback position. Final feedback settled near
  `[-110.7,23.2,-4.0,0.3,-15.4,0.3] deg`.
- Thus the full observation trajectory was not executed, but joints 2 and 3
  changed by about `0.9 deg` and `1.3 deg` during pre-execution gripper/hold
  activity. Future status reporting must distinguish "no target trajectory
  executed" from "no physical motion".

Implemented resolution:
- Added `/supervisor/plan_and_execute_pose_strict`. Under one gateway
  `RLock`, it verifies the trajectory controllers, synchronizes them to fresh
  feedback, strictly replans the requested full pose without fallbacks, and
  immediately executes that exact cached trajectory.
- Planning-only pose checks, strict sequence checks, generic pose motion,
  linear pose motion, joint motion, jog motion, the legacy cached executor,
  and the new atomic executor now share the same planner lock. Candidate
  screening cannot enter MoveIt while an execution operation owns the lock.
- The task now uses the atomic endpoint for every rich-plan pregrasp instead
  of `/supervisor/execute_pose_strict`. The legacy endpoint remains available
  but is no longer used by 6D task execution.
- A far-field observation plan performs a fresh strict planning-only
  preflight before the initial gripper action. An unreachable observation
  pose therefore fails before any gripper/arm hold packet is published.
- Initial opening now compares `right_finger` feedback with
  `open_position_m`. When already within `open_skip_tolerance_m=0.001`, the
  bound action is validated but the duplicate gripper command and wait are
  skipped. This avoids the unnecessary full-joint hold packet seen in this
  trial while preserving opening when the gripper is actually closed.
- All collision, controller synchronization, plan-ID, phase, target-drift,
  target-epoch, MuJoCo, and final lift checks remain enabled.
- Python compilation passed. The MoveIt planner, motion gateway, remote
  streaming, and grasp-task sequence suites passed (`330 passed`, five
  existing rospy deprecation warnings). `git diff --check` passed.

Safety note:
- No `/grasp/stop`, torque-off, disable, arm-stop, controller-stop, or arm
  health/self-check command was issued. Joint enable remains active.

## 2026-07-25 05:24 PDT - atomic observation move passed; near-field contact plan timed out

Hot update and bound plan:
- The updated motion gateway respawned as PID `24532` and advertised
  `/supervisor/plan_and_execute_pose_strict`.
- The updated task node runs in terminal session `57837`.
- A new five-frame far-field plan,
  `7a0d90339663e2910c22e328`, was bound. Its observation pose was approximately
  `(-0.1570,-0.4458,0.1763) m`, and its coarse contact center was
  `(-0.1572,-0.4824,0.0832) m`.

First-stage runtime proof:
- Current-state observation preflight passed.
- The task logged `open gripper ... skipped`, proving that the new
  `right_finger` feedback check suppressed the duplicate full-joint gripper
  packet.
- Atomic strict planning and execution completed without the former
  `no cached pose plan` race. The observation trajectory settled, and the
  task entered `waiting for near-field 6D preview`.
- The operator supplied
  `/home/zhuyupei/Videos/5d92da70f0491db47a796cdf37e1ea09.mp4`
  (`36.1 s`, `720x1280`) plus the first-stage RGB-D screenshot. The video
  shows one continuous observation motion followed by a stable hold.
- The screenshot shows the entire carton remains visible in near-field RGB
  and depth. GUI perception was about pixel `(382,422)`, depth `0.181 m`,
  base center `(-0.162,-0.478,0.093) m`, confidence `0.901`. Therefore the
  earlier concern that the first stage necessarily makes the object
  unobservable is not supported by this trial.

Near-field result:
- The task waited the configured `150 s` and failed closed with
  `NEAR_FIELD_REPLAN_TIMEOUT`; no approach, closure, or lift motion followed.
- Initial status text suggested a phase-handshake error because the last
  remembered rejection was `Preview is not a contact execution plan`.
  Full gateway evidence corrects that interpretation: the remote node did
  switch to near-field mode and repeatedly called
  `/supervisor/check_pose_sequence_strict` with the four contact stages.
- Most candidates failed at strict `pregrasp`. Several reached later stages
  but failed at Cartesian `approach` or `grasp` fractions; multiple candidates
  passed those stages and failed strict `lift` planning. No candidate passed
  all four stages, so no `CONTACT_EXECUTION_PLAN` was published.
- The first stage and phase handshake are now proven. The next defect is
  candidate/sequence feasibility from the reached observation posture,
  especially lift construction after otherwise feasible contact paths.

Safety note:
- The arm remains enabled and stationary at the first-stage observation
  pose. No target approach, gripper closure, lift, `/grasp/stop`, torque-off,
  disable, arm-stop, controller-stop, or arm health/self-check command was
  issued.

## 2026-07-25 05:36 PDT - lift validation now matches physical execution

Current evidence:
- The latest far-field preview after the timed-out task is plan
  `0ee528a6d7f3526dcfb1ccc4`. Its five-frame geometry still finds the carton
  at about `(-0.1548,-0.4691,0.0826) m`, with `90.5%` valid target depth and
  `0.0035 m` depth MAD.
- From the already reached observation posture, the corresponding contact
  sequence was reconstructed as approximately:
  pregrasp `(-0.1564,-0.4641,0.1183) m`, approach
  `(-0.1563,-0.4704,0.1036) m`, grasp
  `(-0.1562,-0.4782,0.0852) m`, and lift
  `(-0.1562,-0.4782,0.1202) m`.
- A live planning-only call to
  `/supervisor/check_pose_sequence_strict` passed all four stages when the
  lift was checked as Cartesian linear motion. The same current sample also
  passed when lift was checked as a normal pose plan.
- This proves that a feasible contact sequence now exists at the current
  observation pose. It does not prove that the fixed base-Z lift caused the
  prior 150-second timeout, so no diagonal retreat and no lift-height change
  has been introduced without runtime evidence.

Confirmed software mismatch and resolution:
- Real 6D execution uses `/supervisor/move_to_pose_linear` for approach,
  grasp, and lift.
- Remote candidate screening and the final visual-refinement recheck
  previously marked only approach and grasp as linear; lift was checked with
  a different planner contract. This could reject candidates differently
  from execution and could also admit a lift that the physical execution
  path had never checked.
- Remote near-field strict sequence requests now mark
  `[pregrasp, approach, grasp, lift]` as
  `[pose, linear, linear, linear]`.
- The task's bounded final visual-refinement sequence uses the same flags.
  A regression test records the exact remote service request so this contract
  cannot silently diverge again.
- Focused remote/task tests passed (`280 passed`, five existing rospy
  deprecation warnings). The motion gateway, MoveIt planner, remote node,
  streaming, task sequence, and sequence-construction group passed
  (`487 passed`, the same five warnings).

Safety note:
- All live service calls in this section were planning-only. The arm remains
  enabled at the first-stage observation pose. No trajectory, gripper
  closure, `/grasp/stop`, torque-off, disable, arm-stop, controller-stop, or
  arm health/self-check command was issued.

## 2026-07-25 05:51 PDT - reachable near-field candidates were mislabeled invalid

Second live two-stage attempt:
- Hot-loaded processes were remote 6D PID `32640` and task PID `32535`;
  motion gateway PID `24532` and the main launch PID `6303` were unchanged.
- The task bound far-field plan `e881d1f3bca68cfb5c873304`. Observation
  preflight passed, the duplicate open command was skipped, and the atomic
  observation trajectory completed successfully in about `30.8 s`.
- The reached observation target was about
  `(-0.1554,-0.4366,0.1756) m`. Perception remained stable there at about
  `(-0.161,-0.476,0.091) m` with confidence near `0.90`.
- During near-field evaluation, the corrected Cartesian-lift check exposed
  several failed lift fractions, but it also produced at least four complete
  four-stage successes. Two low-cost examples reported joint path cost about
  `0.733` and maximum joint delta about `0.258 rad`.
- Despite those successes, the pipeline reported `MOVEIT_RESULT_INVALID`
  and did not publish `CONTACT_EXECUTION_PLAN`. The task consequently reached
  its `150 s` near-field timeout and failed closed. It did not execute
  approach, closure, or lift.

Root cause:
- `_strict_moveit_sequence_evaluation()` returned all four structured hard
  states as `True` (`collision_free`, `within_joint_limits`, `ik_valid`, and
  `planning_success`) and also set
  `evidence_code=STRICT_POSE_SEQUENCE_SUCCESS`.
- The shared `MoveItResult` contract intentionally permits either structured
  hard-state evidence with an empty evidence code, or generic service
  evidence with all hard states unset. Mixing both forms is invalid.
- Therefore every truly successful near-field sequence was discarded by
  `_validated_moveit_result()` as `MOVEIT_RESULT_INVALID`. This was the direct
  reason candidates could not be generated after the observation move.

Implemented resolution:
- Successful strict sequence checks keep the four structured hard states and
  now leave `failure_code` and `evidence_code` empty.
- Regression coverage asserts the complete success payload as well as the
  linear flags `[False, True, True, True]`.
- The pipeline, motion gateway, MoveIt planner, remote node, streaming,
  task-sequence, and sequence-construction test group passed
  (`652 passed`, five existing rospy deprecation warnings).

Safety note:
- The arm remains enabled and stationary at the first-stage observation
  pose. No approach, gripper closure, lift, `/grasp/stop`, torque-off,
  disable, arm-stop, controller-stop, or arm health/self-check command was
  issued after the task failed closed.

## 2026-07-25 06:04 PDT - repeated attempts must preserve a reached observation view

Third live two-stage attempt:
- After correcting the MoveIt evidence contract, the restarted remote node
  no longer reported `MOVEIT_RESULT_INVALID`.
- Far-field plan `0a51c60d51addb5e7e70d2eb` was bound. Although the arm was
  already in the same observation region, the plan supplied another wrist
  orientation and the task spent about `31 s` moving to that alternative
  joint solution.
- The operator's first-stage screenshot at `05:59` shows the carton remains
  fully visible. GUI perception was pixel `(369,404)`, depth `0.187 m`,
  camera coordinates about `(0.187,-0.022,-0.068) m`, base center about
  `(-0.164,-0.477,0.094) m`, confidence `0.900`, and visual fallback
  pregrasp about `(-0.160,-0.477,0.174) m`.
- This again rules out target disappearance at the observation distance.
  The near-field batches contained no `MOVEIT_RESULT_INVALID`, but their
  stable candidates were rejected by current geometry/width checks or by
  the complete MoveIt sequence. The task timed out before contact and failed
  closed.

New continuity defect:
- The earlier reached observation solution that produced multiple complete
  four-stage successes settled near
  `[-112.9,-44.5,93.8,-9.5,-75.3,23.0] deg`.
- The later alternative observation solution settled near
  `[-111.1,-44.3,94.2,-5.2,-75.3,5.8] deg`. Position remained in the same
  camera region, but wrist joints 4 and 6 changed substantially.
- Far-field selection intentionally proves only target visibility at the
  observation pose. Replacing an already reached view with every new
  candidate orientation therefore changes the near-field kinematic branch
  without proving that the branch can continue to contact.

Implemented resolution:
- Added `observation_reuse_position_tolerance_m=0.025`.
- For a far-field observation plan, the task still performs all plan-ID,
  target, epoch, drift, and execution checkpoints. When the current TCP is
  outside the observation region it also keeps the existing strict
  preflight and atomic move.
- When the TCP is already within `25 mm` of the new observation position,
  the task preserves the current reached camera pose, waits for stable joint
  feedback, and immediately requests a new five-frame near-field contact
  plan. A different far-field candidate wrist orientation no longer causes
  another physical observation move.
- The actual current pose, rather than the skipped candidate pose, is used
  when deciding whether a rebound contact pregrasp still requires motion.
- Regression coverage proves that a repeated far-field plan does not invoke
  observation preflight or observation execution, while a distinct rebound
  contact pregrasp is still planned and executed normally.
- The related pipeline, motion gateway, MoveIt, remote, streaming, task, and
  sequence test group passed (`653 passed`, five existing rospy deprecation
  warnings).

Safety note:
- The third task did not execute approach, closure, or lift. No
  `/grasp/stop`, torque-off, disable, arm-stop, controller-stop, or arm
  health/self-check command was issued.

## 2026-07-25 18:21 PDT - latest full ROS chain restarted with joint enable active

Startup:
- No ROS master or old ROS grasp processes were present, so the complete
  current system was launched directly from
  `/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade`.
- Launch command:
  `roslaunch alicia_flexible_grasp_supervisor full_system.launch
  start_real_arm:=true driver_port:=/dev/alicia_arm
  driver_baudrate:=1000000 auto_torque_on_startup:=true
  self_check_poll_rate_hz:=0.0 start_camera:=true start_tactile:=false
  start_gui:=true use_remote_grasp6d:=true
  remote_grasp6d_url:=http://172.23.132.97:8000`.
- The live launch terminal session is `91325`; roslaunch PID is `7253`.
  Current key PIDs are driver `7301`, MoveIt `7315`, motion gateway `7316`,
  camera `7328`, perception `7331`, remote 6D `7333`, task `7336`, and GUI
  `7346`.

Runtime:
- `/dev/alicia_arm` opened at `1000000` baud. The driver reported that
  `auto_torque_on_startup` was active and requested torque-on.
- `/alicia_d/motion_enabled=True`, `/alicia_d/feedback_ready=True`, and
  `/alicia_d/run_status=0`.
- Initial real feedback was approximately
  `[-0.3,-0.5,-0.6,-0.2,-0.5,0.4] deg`; the gripper was open at
  `right_finger=0.04975 m`.
- The trajectory controllers, MoveIt, RGB-D camera, perception, hand-eye TF,
  remote GraspNet protocol-v3 node, grasp task, and GUI are online. The remote
  server reported `backend=graspnet_baseline loaded=True protocol=3`.
- `/grasp/start`, `/supervisor/check_pose_sequence_strict`, and
  `/supervisor/plan_and_execute_pose_strict` are advertised. The task is
  `IDLE ready`.

Safety/operation constraint:
- No health/self-check was run. No `/grasp/stop`, torque-off, disable,
  arm-stop, controller-stop, or other command that stops joint enable was
  issued. The launch remains active while the operator manually aligns the
  target.

## 2026-07-25 18:36 PDT - final-refinement freeze isolated to close-range cropping

Latest real two-stage execution:
- The aligned target was stable for five far-field frames at pixel
  `(272,236)`, depth `0.281-0.282 m`, confidence `0.902-0.904`, and base
  observation point about `(-0.151,-0.436,0.082) m`.
- Remote inference produced 18 stable tabletop candidates; 24 MoveIt poses
  were checked and two complete candidates were reachable. Far-field
  execution plan `f782c92c3e99c6ba87a54caa` was bound.
- The observation trajectory completed with MoveIt/controller `SUCCEEDED`
  in about `27.9 s`. Near-field planning then produced several complete
  strict `[pregrasp, approach, grasp, lift]` successes, with low-cost samples
  around `0.531-0.559` and maximum joint deltas around `0.17-0.19 rad`.
- Contact plan `798d4a8ee7e515a13264825a` was rebound, passed MuJoCo with
  score `100`, and its pregrasp was reached. Its required opening is
  `42.64 mm`; geometry size is about
  `46.24 x 35.73 x 19.11 mm`.

Freeze evidence:
- At the contact pregrasp the RGB-D depth remained stable at
  `0.130-0.132 m`, inside the Intel D405C stated ideal range
  `0.07-0.50 m`. Detection confidence remained about `0.80-0.88`.
- The target remained visible, but its box was approximately
  `(306,388,130,92)` in a `640 x 480` image: the lower edge landed exactly
  on row `480`. Thus the camera was not outside its depth working range; the
  target silhouette was clipped by the image boundary.
- Every newly generated full close-range candidate was rejected by the
  width/projection consistency gate (`GRIPPER_WIDTH_INVALID`). This is an
  expected consequence of incomplete target-cloud width, not evidence that
  the already validated contact plan became unreachable.
- The task waited for a new full same-candidate plan for `150 s`, failed
  closed, released the execution slot, and returned `success: false`. It did
  not execute approach, closure, or lift. The arm remains enabled at the
  contact pregrasp with the gripper open.

Coordinate-semantics finding:
- The live `/perception/object` point at the freeze was about
  `(-0.1541,-0.4635,0.0890) m`.
- The contact plan OBB center was
  `(-0.1503,-0.4456,0.0772) m`, with height `0.0191 m` and support normal
  approximately `(0.016,0.147,0.989)`.
- Live perception reports a visible surface point, while
  `object_geometry.pose_base` is an OBB center. Directly subtracting these
  values would introduce roughly half the object height as a false upward
  correction. Comparing the live point with the planned support-facing
  surface center instead gives a real correction of about `20 mm`, dominated
  by the image-plane center-line offset, with only about `2-3 mm` vertical
  correction.

Resolution decision:
- Keep the third/contact stage. Current depth data is valid, so bypassing the
  final check entirely would discard useful correction evidence.
- Prefer a fresh complete same-candidate Preview exactly as before.
- When close-range clipping prevents full width regeneration, allow a
  center-only fallback after at least five distinct fresh observations.
  Use coordinate medians, require no more than `6 mm` sample spread, and
  compare the observed surface point with the plan's support-facing OBB
  surface.
- Apply one common translation to pregrasp, approach, grasp, lift, and the
  OBB center. Preserve every orientation, roll/pitch, yaw, required opening,
  candidate source, and source lineage. Bound total translation to `25 mm`.
- The corrected four-stage sequence must pass the strict MoveIt sequence
  again and must pass MuJoCo before any physical continuation. If either gate
  fails, the task remains fail-closed and does not touch the object.

Implemented:
- Added `build_bounded_final_center_refinement()`. It computes the planned
  support-facing surface point as
  `OBB center + support normal * object height / 2`, then translates all four
  poses and the OBB center by the same bounded delta.
- The fallback preserves all four pose quaternions byte-for-byte, as well as
  score, required opening, candidate source, and source lineage. It emits a
  new canonical plan ID and current source timestamp.
- The final-refinement loop first waits `3 s` for a complete same-candidate
  Preview. It then collects five distinct source-stamped target observations,
  computes coordinate medians, requires maximum sample spread no greater than
  `6 mm`, and permits at most `25 mm` total translation.
- A center-only candidate is not execution authority by construction. The
  task reruns strict `[pregrasp, approach, grasp, lift]` planning with linear
  flags `[false,true,true,true]`, freezes the new canonical plan only after
  that succeeds, and then reruns the MuJoCo execution gate.
- Default configuration and regression coverage were updated for surface
  versus OBB-center semantics, translation-limit rejection, fresh same-label
  sample admission, and the complete five-frame fallback path.

Verification:
- Python compilation and `git diff --check` passed.
- Focused task/config tests passed: `115 passed`.
- The full related pipeline, motion gateway, MoveIt, remote node, streaming,
  task, and default-config group passed: `662 passed`; the only output was
  five existing rospy deprecation warnings.
- While testing, remote inference produced a fresh far-field execution plan
  `fe50ea2ed747e56254f50f21` at source stamp `1785030036.553111314` and a
  newer far-field Preview `1250acfab01099d5f2192ffd` at
  `1785030108.916953086`. Recovery therefore does not need to execute the
  expired contact plan from the failed task.

Safety note:
- No `/grasp/stop`, torque-off, disable, arm-stop, controller-stop, or arm
  health/self-check command was issued. Continuous joint enable remains
  active.

## 2026-07-25 18:48 PDT - second-stage start blocked by inconsistent hold tolerance

Real retry:
- The hot-loaded task node accepted fresh far-field plan
  `9605f499ff2c5c613278baa5`. The observation trajectory completed and
  settled; target depth was about `0.1915 m`, confidence `0.901`, and the
  target box ended at image row `478/480`.
- Near-field contact plan `ae791a7f74e2bd2e58bca5b5` was generated from
  five fused frames. It passed the complete strict four-stage sequence and
  MuJoCo with score `100`.
- At second-stage start, the controller-sync bridge published its one-cycle
  `0.02 s` feedback-hold trajectory. The operator correctly observed a short
  joint motion. The task then stopped further progression and returned
  failure before the formal near-field pregrasp trajectory continued.

Root cause:
- `/alicia_controller/state` showed the largest endpoint residual on Joint2:
  desired `-0.5798447 rad`, actual `-0.6105244 rad`, error
  `0.0306796 rad`.
- The preceding trajectory is allowed to succeed within
  `execution_goal_tolerance_rad=0.030` plus
  `execution_goal_tolerance_slack_rad=0.005`, for an effective `0.035 rad`
  endpoint band.
- The following controller-sync gate nevertheless used `0.030 rad`.
  Therefore a state already accepted as a successful endpoint was rejected
  by the next-segment gate by only `0.000680 rad`.
- The observed short motion must not be recorded as "no second-stage
  movement": it was the feedback-hold bridge, not the planned contact
  trajectory.

Resolution:
- Set `strict_execution_controller_sync_tolerance_rad=0.035`, matching the
  existing effective endpoint tolerance.
- Keep the continuous `0.30 s` stable-window requirement, `2.0 s` timeout,
  fresh controller feedback requirement, strict start tolerance `0.08 rad`,
  and all collision/IK/sequence/MuJoCo gates unchanged.
- Added a configuration regression requiring the sync tolerance to cover the
  accepted endpoint band while remaining strictly below the strict execution
  start tolerance.

Safety note:
- The failed retry did not execute approach, closure, or lift. No
  `/grasp/stop`, torque-off, disable, arm-stop, controller-stop, or health
  check command was issued; joint enable remains active.

## 2026-07-25 18:53 PDT - center fallback passed live planning; newest stamp was ahead of WSL

Real retry after controller-sync alignment:
- The controller-sync runtime parameter was updated to `0.035 rad`. The
  second-stage contact plan `f43d281afb87605414d041a3` was generated,
  rebound, and passed MuJoCo with score `100`.
- Its near-field pregrasp executed and settled. Unlike the previous attempt,
  the controller-sync bridge did not block formal second-stage motion.
- At the final visual checkpoint, full candidate regeneration remained
  affected by lower-edge clipping, so the new center-only fallback activated.
  Five distinct samples had only `0.3 mm` maximum spread.
- The measured support-surface correction was
  `(-4.5,-18.3,+0.5) mm`, total `18.8 mm`. This independently confirms that
  the visible error is predominantly a lateral center-line bias, not an
  object-height or camera minimum-range error.
- Corrected plan `d40ca5d15f6113602415492d` preserved orientation/width and
  passed the complete strict four-stage MoveIt check with path cost `0.668`
  and maximum joint delta `0.246 rad`.

New failure and root cause:
- The corrected plan was then rejected by MuJoCo before physical continuation:
  `PLAN_STALE: snapshot_stamp_sec is stale or from the future
  (age=-0.321561s)`.
- The fallback used the newest of the five ROS camera source stamps. The WSL
  simulation server clock was about `0.322 s` behind that newest stamp at
  request time, so the unchanged strict future-time gate correctly rejected
  it.
- This was not a geometry, MoveIt, or camera failure. No corrected pregrasp,
  approach, closure, or lift was executed after the rejection.

Resolution:
- Keep the MuJoCo timestamp-age gate unchanged.
- Timestamp the coordinate-median correction with the temporal middle sample
  from the same five-frame window instead of the newest frame. This matches
  the median-position semantics and provides elapsed acquisition time across
  the host/WSL clock offset without fabricating a timestamp.
- Regression coverage now asserts that a five-sample fallback uses the third
  sample's source timestamp.

Safety note:
- No `/grasp/stop`, torque-off, disable, arm-stop, controller-stop, or health
  check command was issued; the arm remains enabled with the gripper open.

## 2026-07-25 19:00 PDT - require visual confirmation after correction motion

Operator observation and stage clarification:
- At the reached near-field pregrasp, the operator still saw a small
  center-line offset. The earlier near-field candidate selection had improved
  alignment but had not eliminated it.
- The computed `18.8 mm` final center correction had not physically executed
  in that attempt: it passed strict MoveIt, then MuJoCo rejected its newest
  source timestamp before the corrected pregrasp command was issued.
- Therefore the observed residual was the input to the final correction, not
  evidence that the final correction itself had already failed.

New contact-before-confirmation gap:
- The task already moves to a changed refined pregrasp before approach.
  However, after that corrected motion settled it previously proceeded
  directly to approach without measuring the actual residual from the new
  camera pose.
- A pose-dependent hand-eye/TCP bias could leave a residual after one
  correction. Assuming one correction is sufficient would not provide the
  evidence needed for contact.

Implemented resolution:
- After a final plan rebound and corrected-pregrasp motion, collect five
  entirely new source-stamped target observations.
- Require no more than `6 mm` sample spread, then compare their coordinate
  median with the corrected plan's support-facing OBB surface center.
- Permit approach only when total residual is at most `6 mm`.
- If residual is larger, fail closed at the corrected pregrasp with
  `FINAL_REFINE_RESIDUAL`; if five stable post-motion frames are unavailable
  within `12 s`, fail with `FINAL_REFINE_CONFIRM_TIMEOUT`.
- This first live confirmation intentionally does not apply another automatic
  correction. Its measured residual will determine whether a bounded second
  iteration is justified, avoiding an unmeasured oscillating correction.

Safety note:
- The new confirmation publishes no trajectory or gripper command. It can
  only permit an already validated approach or block contact. No stop,
  disable, torque-off, controller-stop, or health-check command was added.

## 2026-07-25 19:05 PDT - actual correction left 20.1 mm residual; contact blocked

First real post-correction measurement:
- Fresh near-field plan `b543a34c198d140a4eeffec2` passed MuJoCo with
  score `100` and its pregrasp executed successfully.
- The center-only fallback measured
  `(-2.6,-17.4,+1.5) mm`, total `17.6 mm`, from five frames with `0.9 mm`
  spread. Corrected plan `d317ac1d7869f8b6aa62d2ee` passed the complete
  strict four-stage sequence with path cost `0.584`, maximum joint delta
  `0.215 rad`, and MuJoCo score `100`.
- The corrected pregrasp trajectory physically executed and settled. The new
  post-motion confirmation then collected five entirely new frames.
- Measured residual after correction was
  `(-6.2,-10.2,-16.2) mm`, total `20.1 mm`, with `2.3 mm` spread.
- The new gate returned `FINAL_REFINE_RESIDUAL` and refused contact. No
  approach, closure, or lift followed. The arm remains enabled at the
  corrected pregrasp and the gripper remains open.

Conclusion from the measured response:
- The y component improved from `-17.4 mm` to `-10.2 mm`, but did not
  converge in one correction.
- The x component changed from `-2.6 mm` to `-6.2 mm`, moving away from zero.
- The apparent z component changed from `+1.5 mm` to `-16.2 mm`. A static
  object did not physically move by this amount, so this is pose/viewpoint
  dependent coordinate bias, not ordinary frame noise.
- The five-frame spread remained small in both observations. The discrepancy
  is systematic and repeatable, not single-frame RGB-D jitter.
- A second blind xyz correction is not justified: the measured response is
  not contractive in all axes and the large z term could drive the grasp
  toward the support surface.

Operational decision:
- Keep the final/contact stage paused. Do not bypass the residual gate and do
  not execute the first two stages directly into contact.
- Preserve the current enabled, open-gripper posture while separating
  hand-eye/base-camera pose bias from TCP/gripper-center offset.
- The changing perceived base-frame target under wrist-pose changes is direct
  evidence against treating this as only a TCP-center problem. TCP error can
  offset the physical jaw center, but it does not by itself make a static
  target's estimated base coordinates shift with camera pose.

Safety note:
- No `/grasp/stop`, torque-off, disable, arm-stop, controller-stop, or health
  check command was issued. Failure was produced by the contact permission
  gate only; joint enable remains active.

## 2026-07-25 19:10 PDT - operator observed collision during correction pregrasp

Authoritative physical observation:
- The operator observed that the first-stage pose was off center.
- The second-stage motion appeared to correct in the same direction as the
  existing offset, increasing rather than reducing the physical jaw-center
  error.
- During the next descending correction motion, the gripper/arm physically
  struck the target.
- This collision must not be described as "no contact". Software logs show
  that the later approach command was blocked, but the corrected-pregrasp
  trajectory itself had already executed and was the collision-producing
  physical motion.

Correlation with software evidence:
- The software-commanded center correction was
  `(-2.6,-17.4,+1.5) mm`, while the post-motion visual residual became
  `(-6.2,-10.2,-16.2) mm`.
- x moved farther from zero and apparent z changed by `17.7 mm` relative to
  the pre-correction estimate. This is consistent with the operator seeing
  correction in the wrong physical direction and a descending collision.
- The five-frame spreads were low (`0.9 mm` before and `2.3 mm` after), so
  random RGB-D jitter cannot explain the sign/direction failure.
- The live hand-eye TF is loaded from
  `~/.ros/easy_handeye/d405_v4l2_charuco_handeyecalibration_eye_on_hand.yaml`
  as tool0 -> camera_link:
  translation about `[-86.47,+13.62,-112.59] mm`, quaternion about
  `[0.00420,-0.70881,0.00948,0.70532]`.
- The fallback values in repository `config/handeye.yaml` are different:
  translation about `[-84.49,+10.05,-123.37] mm`, quaternion about
  `[0.00441,-0.68527,0.00005,0.72828]`. The file is intentionally preferred
  at runtime, so `/handeye` fallback parameters do not describe the actual
  live TF.
- There is no separate runtime TCP offset parameter in the grasp chain.
  Candidate generation, MoveIt, and gripper CAD all use URDF `tool0`.

Root-cause conclusion:
- A static target's estimated base coordinates change materially with wrist
  pose, and a base-frame correction does not produce the expected visual
  response. This directly implicates the eye-on-hand transform/kinematic
  chain before a pure TCP-center error.
- TCP/gripper-center error may still add a fixed jaw-center offset, but it
  cannot by itself explain a static target moving in estimated base xyz when
  camera pose changes.
- MoveIt and MuJoCo validated geometry expressed through the same incorrect
  transform. Their success therefore cannot prove real-world clearance when
  the external calibration is wrong.

Immediate operational decision:
- Pause all further automatic grasp motion. Do not execute another far-field,
  near-field, correction, approach, close, or lift trajectory.
- Keep the arm enabled and stationary with the gripper open. Do not call
  `/grasp/stop`, torque-off, disable, arm-stop, or controller-stop.
- Do not tune correction gains, increase translation limits, or apply a
  second visual correction from this dataset. First verify/recalibrate the
  live tool0 -> camera_link transform and then verify the physical jaw center
  relative to URDF tool0.

## 2026-07-25 19:21 PDT - repeated camera-left miss and calibration interlock

Additional operator observation:
- During the first observation approach, the jaw center consistently moved to
  the left of the target in the camera image.
- The second-stage correction again appeared to move toward the same
  camera-left side, increasing the physical offset instead of eliminating it.
- The repeated image-direction sign across separate stages is incompatible
  with zero-mean RGB-D frame noise. It is a systematic coordinate-chain or
  tool-center error.

Root-cause classification:
- The strongest current diagnosis is an invalid eye-on-hand extrinsic or an
  incorrect transform convention/direction in the camera-to-base chain.
- Evidence is stronger than the camera-left appearance alone: before and
  after a small wrist-pose change, a static target's estimated base-frame
  residual changed from `(-2.6,-17.4,+1.5) mm` to
  `(-6.2,-10.2,-16.2) mm`, while each five-frame group had only
  `0.9-2.3 mm` spread. A static target cannot make that pose-dependent change.
- A TCP/jaw-center offset may coexist and can explain a repeatable fixed jaw
  miss. It cannot by itself explain the estimated target moving in base xyz as
  the eye-on-hand camera pose changes. Therefore TCP is secondary until the
  hand-eye invariance test passes.
- The `tool0 -> camera_link` live TF is still the July 12 easy_handeye file,
  not the different repository fallback values. No new TCP pivot result is
  currently applied to the runtime chain.

Implemented containment:
- Added `/grasp/calibration_interlock_active=true` with reason
  `HAND_EYE_UNVERIFIED_AFTER_CORRECTION_COLLISION`.
- `grasp_task_node.start_cb` now rejects every new execution request with
  `CALIBRATION_INTERLOCK` before binding a plan or issuing any physical
  action.
- The interlock does not call `/grasp/stop`, does not stop a controller, and
  does not disable torque or joint enable. It only prevents a new automatic
  grasp from starting while calibration is unresolved.
- Added a regression proving that an interlocked start invokes no execution
  action and leaves the task inactive.

## 2026-07-25 19:34 PDT - corrected diagnosis: clipped-mask center drove collision motion

New evidence from synchronized perception and task logs:
- Immediately before center-only correction, five stable observations were
  around `uv=(399,431)`, depth `0.129-0.130 m`, and
  `base=(-0.153,-0.463,+0.088..0.089) m`.
- Their detection boxes were around `(337,381,133,99)` in a `640x480`
  image. `381 + 99 = 480`: the target mask was already truncated exactly at
  the image bottom.
- After the correction motion, observations stabilized near `uv=(408,454)`,
  depth `0.139-0.142 m`, with boxes around `(344,423,140,57)`. These boxes
  also ended at row `480` and were more severely truncated.
- The task's center-only fallback used `/perception/object.pose_base`. That
  point is formed from the 2D instance-mask centroid pixel plus the mask-wide
  median depth; it is not the complete 3D OBB center. A clipped visible-mask
  centroid is viewpoint-dependent and cannot be treated as a physical object
  center.

Revised root-cause conclusion:
- The immediate cause of the wrong second correction and descending contact
  is now confirmed as a final-refinement gating bug: the code intentionally
  allowed center-only motion when the full cloud was clipped, but the same
  clipping invalidated the 2D centroid used by that fallback.
- The standard OpenCV-optical to ROS-camera-link conversion is
  `[z,-x,-y]`; no simple left/right sign inversion was found in that code.
- Repeated camera-left miss still justifies an independent hand-eye and TCP
  audit, but the collision trajectory cannot be used as proof that hand-eye
  alone is wrong. The estimator itself supplied a view-dependent center.
- The previous "hand-eye is the direct root cause" statement is superseded
  by this two-part diagnosis: confirmed clipped-center algorithm failure,
  plus unresolved calibration/TCP contribution.

Implemented resolution:
- Disabled `final_visual_refine_center_fallback_enabled` in production
  configuration.
- Added a mandatory four-edge clearance gate. If a center-only fallback is
  ever re-enabled, a target box closer than `4 px` to any image edge returns
  `FINAL_REFINE_CENTER_CLIPPED` and cannot create a correction plan.
- Added a regression reproducing the real `(337,381,133,99)` box and proving
  that it is rejected with zero motion.
- The separate calibration interlock remains active. Neither change sends a
  stop, disable, torque-off, controller-stop, gripper, or trajectory command.

## 2026-07-25 20:10 PDT - passive ChArUco hand-eye invariance experiment

Operator request:
- A ChArUco board has been mounted, and the current task is to verify the
  precision of the live eye-on-hand calibration before any further grasp
  motion.

Experiment design:
- Keep the ChArUco board physically fixed throughout the experiment.
- The operator manually moves the wrist camera to several settled poses around
  the same board. The software does not command robot motion.
- For each settled pose, compute
  `T_base_board = T_base_tool0 * T_tool0_camera_link * T_camera_link_board`.
  A correct hand-eye transform should make this board pose nearly invariant in
  `base_link`, even when the wrist pose changes.
- Project-specific pass thresholds for this bench setup are:
  position RMS <= `3 mm`, position max <= `5 mm`,
  orientation RMS <= `1 deg`, orientation max <= `2 deg`, with at least six
  distinct manual poses.

Implemented for this experiment:
- Added `handeye_invariance_verifier.py`, a passive ROS node with Trigger
  services `capture`, `report`, and `clear`. It only reads TF, ChArUco quality,
  and `/joint_states`; it publishes no trajectory, gripper, stop, disable,
  torque-off, or controller command.
- Added `handeye_invariance_metrics.py` for quaternion-safe transform
  averaging and RMS/max drift statistics.
- Extended `charuco_tracker.py` so it can reuse the current running image
  stream with camera intrinsics loaded from `d405_factory_camera.yaml`, because
  the latest grasp runtime exposes `/supervisor/camera/color/image_raw` but no
  matching live `CameraInfo` topic.
- `charuco_tracker.py` now publishes `/charuco/quality` with pose status,
  marker count, ChArUco corner count, image-edge clearance, and reprojection
  RMS. The verifier rejects samples when the board is clipped, too sparse, too
  stale, or reprojection error is high.
- Added passive launch file
  `d405_current_stream_handeye_invariance.launch`; it does not start a new
  camera and does not republish easy_handeye calibration.

Verification:
- `python3 -m unittest
  src/real-arm/alicia_d_calibration/tests/test_handeye_invariance_metrics.py`
  passed.
- `python3 -m py_compile` for the new/modified calibration scripts passed.
- `git diff --check` passed.
- `catkin_make --pkg alicia_d_calibration -j2` passed.

## 2026-07-25 20:29 PDT - operator-facing hand-eye verification UI correction

Operator feedback:
- Starting verification without first showing the RGB-D view, without a clear
  posture-adjustment procedure, and without a minimal manual joint panel was
  premature.

Correction:
- Added `handeye_joint_pose_panel.py`, a small operator-guided joint jog UI
  for hand-eye verification poses. It has no stop/disable control and does not
  call MoveIt stop after each jog.
- The existing `joint_jog_gui.py` is not used for this experiment because it
  contains a Stop button and calls `group.stop()`, which conflicts with the
  operator's explicit "do not publish stop/disable" constraint.
- Opened the annotated RGB view on `/charuco/result` and the depth view on
  `/supervisor/camera/depth/image_raw` before any sampling.
- Started the passive `handeye_invariance_verifier` service node separately
  from the already running ChArUco tracker. It only reads TF, quality, and
  joint state.

Current observation:
- `/charuco/quality` reports `pose_ok=true`, reprojection around `0.39 px`,
  and large edge clearance, but only about `13` ChArUco corners. This is not
  enough for a high-confidence hand-eye precision check.
- The immediate operator target is to adjust the wrist/camera pose so the
  board occupies roughly one third to one half of the RGB image while staying
  fully visible. Aim for at least `35` detected ChArUco corners before calling
  `capture`.

## 2026-07-25 20:43 PDT - RGB-D latency root cause and first hand-eye sample

Operator observation:
- The RGB-D display still appeared highly delayed during hand-eye
  verification setup.

Latency diagnosis:
- Raw camera topics were fresh: `/supervisor/camera/color/image_raw` around
  `19-26 ms`, `/supervisor/camera/depth/image_raw` around `18-31 ms`.
- The derived ChArUco stream was stale: `/charuco/result` around `2963 ms`
  and `/charuco/quality` around `3208 ms`.
- Therefore the main delay was not the D405 source stream; it was the
  ChArUco tracker processing old image frames from its subscriber queue.

Resolution:
- Closed the full Alicia supervisor GUI by ending only its GUI process. The
  GUI was subscribing to RGB, depth, TF, plans, and grasp state; closing it
  reduces image/render load. No `/grasp/stop`, torque-off, disable, controller
  stop, or arm-stop command was sent.
- Added a stale-frame gate to `charuco_tracker.py`: images older than
  `0.35 s` are dropped before CV processing. The subscriber also uses
  `queue_size=1`, a large image buffer, and TCP no-delay.
- Restarted only the passive ChArUco detection process. Camera, hand-eye TF,
  motion gateway, grasp task node, and arm enable were left running.
- After the fix, latency improved to: raw RGB `26 ms`, raw depth `18 ms`,
  `/charuco/result` `183 ms`, `/charuco/quality` `211 ms`.
- Detection quality also improved after pose adjustment: about `51-57`
  ChArUco corners and reprojection around `0.30-0.37 px`.

Verifier fix:
- The first capture attempt was rejected because the verifier treated the
  static `tool0 -> camera_link` hand-eye TF stamp `0` as stale.
- Fixed `handeye_invariance_verifier.py` so stamp `0` static transforms do
  not fail the freshness gate. Dynamic TF and ChArUco board TF still must be
  fresh.

Sample progress:
- Hand-eye invariance sample 1/6 succeeded:
  window RMS `0.14 mm`, window max `0.23 mm`, detected corners `57`.
- Next operator action: manually move to a second settled viewpoint while
  keeping the board fully visible and roughly `20-30 cm` from the camera.

## 2026-07-25 20:56 PDT - hand-eye invariance samples 2 and 3

Sample progress:
- Sample 2/6 succeeded:
  window RMS `0.24 mm`, window max `0.52 mm`, detected corners `58`.
- The operator changed to a third arbitrary settled viewpoint and reported
  "adjustment complete".
- Pre-capture quality for sample 3 was healthy:
  `/charuco/quality` age about `275 ms`, `65` ChArUco corners,
  edge clearance about `50 px`, reprojection about `0.399 px`.
- Sample 3/6 succeeded:
  window RMS `0.03 mm`, window max `0.05 mm`, detected corners `63`.

Current instruction to operator:
- Move to a fourth distinct settled viewpoint, keep the board fully visible,
  then report "adjustment complete". No automatic sampling is performed until
  the operator reports the next pose is ready.

## 2026-07-25 20:59 PDT - hand-eye invariance sample 4

Sample progress:
- The operator changed to a fourth settled viewpoint and reported adjustment
  complete.
- Pre-capture quality was healthy:
  `/charuco/quality` age about `154 ms`, `68` ChArUco corners,
  edge clearance about `41 px`, reprojection about `0.454 px`.
- Sample 4/6 succeeded:
  window RMS `0.08 mm`, window max `0.09 mm`, detected corners `68`.

Current instruction to operator:
- Move to a fifth distinct settled viewpoint, keep the board fully visible,
  then report "adjustment complete". Sampling remains operator-triggered.

## 2026-07-25 21:02 PDT - hand-eye invariance sample 5

Sample progress:
- The operator changed to a fifth settled viewpoint and reported adjustment
  complete.
- Pre-capture quality was acceptable:
  `/charuco/quality` age about `394 ms`, `70` ChArUco corners,
  edge clearance about `53 px`, reprojection about `1.066 px`.
- Sample 5/6 succeeded:
  window RMS `0.11 mm`, window max `0.39 mm`, detected corners `68`.

Current instruction to operator:
- Move to a sixth distinct settled viewpoint, preferably visibly different
  from the previous five while keeping the board fully visible, then report
  "adjustment complete". After sample 6, generate the invariance report.

## 2026-07-25 21:08 PDT - hand-eye invariance final report

Sample progress:
- The operator changed to a sixth settled viewpoint and reported adjustment
  complete.
- Pre-capture quality was healthy:
  `/charuco/quality` age about `441 ms`, `64` ChArUco corners,
  edge clearance about `80.8 px`, reprojection about `0.648 px`.
- Sample 6/6 succeeded:
  window RMS `0.04 mm`, window max `0.08 mm`, detected corners `68`.

Final invariance report:
- Report file:
  `/home/zhuyupei/.ros/alicia_d_calibration/handeye_verification/handeye_invariance_20260725_203843.yaml`
- Result: `FAIL`.
- Across six settled wrist poses, the fixed ChArUco board was estimated in
  `base_link` with:
  position RMS `6.74 mm`, position max `12.29 mm`,
  orientation RMS `1.46 deg`, orientation max `2.62 deg`.
- Thresholds were:
  position RMS <= `3 mm`, position max <= `5 mm`,
  orientation RMS <= `1 deg`, orientation max <= `2 deg`.

Per-sample base-frame board error from the six-pose mean:
- Sample 1: `6.40 mm`, window RMS `0.14 mm`, corners `57`.
- Sample 2: `3.49 mm`, window RMS `0.24 mm`, corners `58`.
- Sample 3: `2.07 mm`, window RMS `0.03 mm`, corners `63`.
- Sample 4: `3.30 mm`, window RMS `0.08 mm`, corners `68`.
- Sample 5: `12.29 mm`, window RMS `0.11 mm`, corners `68`.
- Sample 6: `7.30 mm`, window RMS `0.04 mm`, corners `68`.

Interpretation:
- Each individual one-second sample window was very stable, so the failure is
  not explained by frame-to-frame RGB-D noise.
- Removing the largest sample-5 outlier still leaves position RMS about
  `4.28 mm` and max about `6.45 mm`, so the failure is not a single bad sample.
- The evidence supports a pose-dependent systematic error in the live
  `tool0 -> camera_link` hand-eye/camera model chain.
- The live TF is the easy_handeye static transform, approximately translation
  `[-86, +14, -113] mm` and quaternion `[0.004, -0.709, 0.009, 0.705]`.
  This differs from the repository fallback `/handeye` values and confirms the
  runtime is using the saved easy_handeye calibration file.

Operational decision:
- Do not run another automatic grasp until the hand-eye calibration is
  recalibrated or a new transform passes this invariance test.
- The calibration interlock remains appropriate. No stop, disable, torque-off,
  controller-stop, or arm-stop command was sent during the verification.

## 2026-07-25 21:51 PDT - start manual eye-on-hand recalibration

Operator request:
- Re-run the eye-on-hand calibration after the invariance test failed.

Recalibration launch choice:
- Started only the easy_handeye calibration backend in namespace
  `/d405_v4l2_charuco_recalib_20260725_eye_on_hand`.
- Launch flags used:
  `eye_on_hand=true`, `freehand_robot_movement=true`,
  `start_sampling_gui=false`, `start_rviz=false`, `publish_dummy=false`.
- This avoids automatic MoveIt calibration motions, avoids rqt/RViz, avoids
  opening another camera, avoids starting another ChArUco tracker, and avoids
  publishing a duplicate dummy `tool0 -> camera_link` TF.
- The active ChArUco tracker and minimal joint pose panel remain the operator
  interfaces.

Current state:
- Available calibration algorithms are:
  `OpenCV/Tsai-Lenz`, `OpenCV/Park`, `OpenCV/Horaud`,
  `OpenCV/Andreff`, `OpenCV/Daniilidis`.
- The default active algorithm is `OpenCV/Tsai-Lenz`.
- First recalibration sample was not taken because `/charuco/quality` reported
  `pose_ok=false`, `markers=0`, `corners=0`. Sampling was rejected instead of
  recording a bad frame.

Operator instruction:
- Reposition the wrist/camera until the annotated RGB view `/charuco/result`
  shows the ChArUco board clearly with many green corners, then report
  "adjustment complete". Sampling remains operator-triggered and quality-gated.

## 2026-07-25 22:02 PDT - add panel-driven stable/save sampling flow

Operator request:
- Avoid repeating chat messages for every recalibration pose.
- Add a Save button, a "stable" button, and a live "can save" display to the
  minimal joint panel.
- Desired flow: move to a pose, click "stable"; the panel checks ChArUco
  quality; if it passes, the panel shows that saving is allowed; then click
  Save to take the easy_handeye sample.

Implemented:
- Updated `handeye_joint_pose_panel.py` with:
  `已稳定` button, `保存` button, live `Can save`, `Samples`, and `Quality`
  status fields.
- The panel subscribes to `/charuco/quality` and gates saving on:
  pose detected, age < `500 ms`, corners >= `35`, edge clearance > `12 px`,
  reprojection RMS < `1.5 px`.
- The `已稳定` button checks the quality for a short stable window before
  enabling `保存`.
- The `保存` button calls only
  `/d405_v4l2_charuco_recalib_20260725_eye_on_hand/take_sample` and updates
  the sample count. It does not command arm motion.
- Restarted only the minimal joint panel so the new UI appears. No stop,
  disable, torque-off, controller-stop, or arm-stop command was sent.

Verification:
- `python3 -m py_compile handeye_joint_pose_panel.py` passed.
- `git diff --check` passed.
- `catkin_make --pkg alicia_d_calibration -j2` passed.
- A code scan found no actual stop/disable/torque-off command in the panel.
- The easy_handeye sample list is still empty, confirming that no first sample
  was taken while the board was not aligned.

Current operator flow:
- Align the ChArUco board in `/charuco/result`.
- Click `已稳定`.
- If the panel shows `Can save: yes`, click `保存`.
- Move to the next distinct pose and repeat until enough samples are collected.

## 2026-07-25 22:35 PDT - compute and validate recalibration samples

Operator clarification:
- The requested task is not to continue collecting samples. The requested task
  is to compute the TF from the samples already collected and validate its
  precision.

Sample set:
- easy_handeye namespace:
  `/d405_v4l2_charuco_recalib_20260725_eye_on_hand`.
- Collected sample count: `33`.
- No `save_calibration` was called, and no live TF was overwritten.

Validation method:
- For each candidate `T_tool0_camera_link`, compute
  `T_base_board = T_base_tool0 * T_tool0_camera_link * T_camera_link_board`
  for all saved samples.
- A good calibration should make the fixed board pose invariant in
  `base_link`. The same thresholds are used as before:
  position RMS <= `3 mm`, max <= `5 mm`, orientation RMS <= `1 deg`,
  max <= `2 deg`.

Current old live TF on these 33 samples:
- Translation about `[-86.47, +13.62, -112.59] mm`.
- Quaternion about `[0.00420, -0.70881, 0.00948, 0.70532]`.
- Residual: position RMS `12.21 mm`, max `46.61 mm`,
  orientation RMS `2.14 deg`, max `6.11 deg`.

Computed candidates using all 33 samples:
- `OpenCV/Tsai-Lenz`:
  xyz `[-85.99, +11.45, -111.35] mm`,
  q `[0.00279, -0.70895, 0.00563, 0.70523]`,
  residual RMS `12.24 mm`, max `46.50 mm`,
  orientation RMS `2.14 deg`, max `6.09 deg`.
- `OpenCV/Park`:
  xyz `[-84.90, +12.49, -119.50] mm`,
  q `[0.01881, -0.71214, 0.01767, 0.70157]`,
  residual RMS `11.98 mm`, max `47.55 mm`,
  orientation RMS `2.12 deg`, max `6.17 deg`.
- `OpenCV/Horaud`:
  xyz `[-84.91, +12.50, -119.52] mm`,
  q `[0.01883, -0.71222, 0.01768, 0.70148]`,
  residual RMS `11.98 mm`, max `47.56 mm`,
  orientation RMS `2.12 deg`, max `6.17 deg`.
- `OpenCV/Andreff`:
  xyz `[-1.54, +23.76, -359.74] mm`,
  q `[0.02592, -0.71927, 0.02569, 0.69377]`,
  residual RMS `29.03 mm`, max `67.20 mm`,
  orientation RMS `2.14 deg`, max `6.21 deg`.
- `OpenCV/Daniilidis`:
  xyz `[-90.95, +15.42, -143.28] mm`,
  q `[0.00546, -0.71270, 0.00843, 0.70140]`,
  residual RMS `9.04 mm`, max `40.92 mm`,
  orientation RMS `2.13 deg`, max `6.09 deg`.

Outlier finding:
- Sample 32 is the dominant outlier for every reasonable algorithm.
- With the all-sample Daniilidis result, sample 32 contributes about
  `40.9 mm` error. The old live TF also shows sample 32 as the worst sample
  at about `46.6 mm`.

Offline recomputation without outliers:
- Recomputing without sample 32 improves the best candidate but still fails:
  Daniilidis RMS `6.54 mm`, max `11.53 mm`,
  orientation RMS `1.87 deg`, max `3.18 deg`.
- Recomputing without samples 1, 2, 32, and 33 gives the best offline result:
  Daniilidis xyz `[-91.55, +13.01, -132.28] mm`,
  q `[0.00059, -0.70750, 0.00931, 0.70665]`,
  residual RMS `5.86 mm`, max `9.81 mm`,
  orientation RMS `1.68 deg`, max `2.89 deg`.
- This still fails the acceptance thresholds.

Conclusion:
- The new calibration computed from the current 33 samples is not accurate
  enough to replace the running hand-eye TF.
- Do not call `save_calibration` and do not deploy this TF to the live grasp
  stack.
- The sample set contains at least one severe bad sample and, even after
  removing obvious outliers, the remaining set does not reach the required
  hand-eye precision.
- Further recalibration should improve sample quality/diversity and should
  include an independent invariance verification pass before applying any new
  TF.

## 2026-07-25 22:52 PDT - remove known bad recalibration samples before appending more

Operator decision:
- Remove samples `1`, `2`, `32`, and `33` from the 33-sample recalibration
  set, keep the other samples, then collect about 30 more samples and compute
  the TF from the combined set.

Action taken:
- easy_handeye uses zero-based removal indices. Original samples
  `1`, `2`, `32`, and `33` correspond to indices `0`, `1`, `31`, and `32`.
- Removed them in descending order to avoid index shift:
  `32`, `31`, `1`, `0`.
- Sample count changed:
  `33 -> 32 -> 31 -> 30 -> 29`.
- This operation only edited the easy_handeye in-memory sample list. It did
  not save calibration, replace the live TF, command robot motion, call stop,
  disable torque, or stop any controller.

Panel state:
- Closed an old duplicate minimal panel and restarted the current panel so the
  operator uses only the updated stable/save UI.
- After restart, the sample count read back as `31`, meaning two new samples
  had already been appended after the four removals. These were not deleted.
- Target total for the operator's plan is about `59` samples:
  the retained `29` plus `30` new samples. With the count currently at `31`,
  about `28` more samples remain.

## 2026-07-25 23:05 PDT - validate combined 59-sample recalibration set

Operator update:
- Additional sampling is complete. Recompute the TF from the current samples
  and validate its precision.

Current sample set:
- easy_handeye namespace:
  `/d405_v4l2_charuco_recalib_20260725_eye_on_hand`.
- Sample count: `59`.
- This is the intended combined set after removing original samples
  `1`, `2`, `32`, and `33`, then appending new samples.
- No `save_calibration` was called. The live `tool0 -> camera_link` TF was not
  overwritten.

Full-set validation:
- Old live TF on the 59-sample set:
  residual RMS `32.32 mm`, max `237.63 mm`,
  orientation RMS `20.10 deg`, max `153.63 deg`.
- `OpenCV/Tsai-Lenz`:
  xyz `[-85.43, +2.12, -118.60] mm`,
  q `[0.02940, -0.70344, 0.03261, 0.70940]`,
  residual RMS `34.92 mm`, max `256.02 mm`,
  orientation RMS `20.09 deg`, max `153.68 deg`.
- `OpenCV/Park`:
  xyz `[-70.27, +25.12, -133.31] mm`,
  q `[0.05572, -0.69324, 0.02457, 0.71813]`,
  residual RMS `35.44 mm`, max `254.53 mm`,
  orientation RMS `20.14 deg`, max `153.92 deg`.
- `OpenCV/Horaud`:
  xyz `[-70.41, +25.88, -133.80] mm`,
  q `[0.04921, -0.69854, 0.03052, 0.71322]`,
  residual RMS `33.90 mm`, max `243.04 mm`,
  orientation RMS `20.11 deg`, max `153.78 deg`.
- `OpenCV/Andreff`:
  xyz `[+9.63, +31.46, -366.09] mm`,
  q `[0.08774, -0.73015, 0.10121, 0.67003]`,
  residual RMS `45.13 mm`, max `239.38 mm`,
  orientation RMS `20.05 deg`, max `152.93 deg`.
- `OpenCV/Daniilidis` produced a NaN calibration on the full set
  (`Eigenvalues did not converge`), so it is invalid for the full 59 samples.

Outlier diagnosis:
- Sample `53` is the dominant outlier for the old TF and every valid algorithm.
- Full-set max errors around `237-256 mm` and orientation max around `153 deg`
  strongly suggest a bad ChArUco pose sample such as a flipped/mirrored board
  pose, stale/unstable pose, or an accidental save during poor observation.

Offline robust recomputation without changing the service sample list:
- Excluding sample `53`:
  best candidate is `OpenCV/Daniilidis`,
  xyz `[-89.85, +7.16, -136.00] mm`,
  q `[0.02244, -0.70489, 0.02512, 0.70852]`,
  residual RMS `7.10 mm`, max `12.74 mm`,
  orientation RMS `1.83 deg`, max `2.87 deg`.
- Excluding samples `52` and `53`:
  best candidate is `OpenCV/Daniilidis`,
  xyz `[-89.95, +7.84, -136.78] mm`,
  q `[0.02389, -0.70655, 0.02498, 0.70681]`,
  residual RMS `7.06 mm`, max `12.62 mm`,
  orientation RMS `1.83 deg`, max `2.75 deg`.
- These still fail the acceptance thresholds:
  RMS <= `3 mm`, max <= `5 mm`, orientation RMS <= `1 deg`,
  orientation max <= `2 deg`.

Conclusion:
- The current combined 59-sample recalibration set does not produce a TF that
  is accurate enough to apply.
- Sample `53` must be removed or recollected, but removing it alone is not
  sufficient. The remaining set still has millimeter-to-centimeter pose-
  dependent drift.
- Do not call `save_calibration`, do not overwrite the easy_handeye file, and
  do not deploy any of these candidates into the grasp stack.

## 2026-07-25 23:15 PDT - compare old TF against filtered reasonable samples

Question:
- Would the TF computed from the currently reasonable samples be better than
  the old live TF?

Method:
- Read the current easy_handeye sample list only; no sample deletion, no robot
  motion, no controller command, and no `save_calibration`.
- Compare the old live `tool0 -> camera_link` TF against OpenCV hand-eye
  candidates on the same filtered sample subsets.
- Report saved to:
  `/home/zhuyupei/.ros/alicia_d_calibration/handeye_verification/handeye_reasonable_sample_compare_20260725_221518.json`.

Results:
- Full `59` samples are not usable because sample `53` dominates the error:
  old live TF residual RMS `32.32 mm`, max `237.63 mm`,
  orientation RMS `20.10 deg`, max `153.63 deg`.
- Excluding only sample `53`:
  old live TF residual RMS `8.50 mm`, max `13.86 mm`,
  orientation RMS `1.91 deg`, max `2.87 deg`;
  best new candidate is `OpenCV/Daniilidis` with
  xyz `[-89.85, +7.16, -136.00] mm`,
  q `[0.02244, -0.70489, 0.02512, 0.70852]`,
  residual RMS `7.10 mm`, max `12.74 mm`,
  orientation RMS `1.83 deg`, max `2.87 deg`.
- Excluding samples `52` and `53`:
  old live TF residual RMS `8.51 mm`, max `13.75 mm`,
  orientation RMS `1.90 deg`, max `2.91 deg`;
  best new candidate is `OpenCV/Daniilidis` with
  xyz `[-89.94, +7.84, -136.78] mm`,
  q `[0.02389, -0.70655, 0.02498, 0.70681]`,
  residual RMS `7.05 mm`, max `12.62 mm`,
  orientation RMS `1.83 deg`, max `2.75 deg`.
- Excluding samples `49`, `50`, `52`, and `53`:
  old live TF residual RMS `8.48 mm`, max `13.91 mm`,
  orientation RMS `1.89 deg`, max `2.84 deg`;
  best new candidate is `OpenCV/Daniilidis` with
  xyz `[-90.96, +6.14, -136.17] mm`,
  q `[0.02564, -0.70237, 0.02661, 0.71086]`,
  residual RMS `7.31 mm`, max `13.02 mm`,
  orientation RMS `1.79 deg`, max `3.11 deg`.

Conclusion:
- On filtered reasonable subsets, the new candidate is slightly better in
  translation RMS than the old live TF, but the improvement is modest:
  about `1.2-1.5 mm` RMS.
- It is not a reliable replacement yet. The best filtered candidate still
  fails the acceptance thresholds of RMS <= `3 mm`, max <= `5 mm`,
  orientation RMS <= `1 deg`, and orientation max <= `2 deg`.
- Therefore the answer is: the reasonable-sample TF is probably somewhat
  better than the old TF in average translational consistency, but it is not
  accurate enough to deploy for grasping. Keep the calibration interlock active
  and do not overwrite the live TF.

## 2026-07-25 23:19 PDT - diagnose why standard-looking samples produced an unusable TF

Operator question:
- The samples were collected using the standard process. Why can the computed
  TF still be unusable, and where is the likely fault?

Read-only diagnostics:
- Current sample count: `59`.
- Diagnostic report:
  `/home/zhuyupei/.ros/alicia_d_calibration/handeye_verification/handeye_bad_tf_cause_diagnostics_20260725_221901.json`.
- No `save_calibration`, no sample deletion, no robot motion, no controller
  command, and no stop/disable command were issued.

Evidence:
- With all samples, sample `53` is the dominant outlier:
  old live TF residual `237.63 mm` and `153.63 deg`; every valid OpenCV
  algorithm also has sample `53` as the worst sample.
- Around the bad region:
  sample `52` robot xyz is `[-69.71, -355.35, 111.76] mm`, camera-marker xyz
  is `[242.83, -123.03, -114.59] mm`;
  sample `53` robot xyz is `[-70.69, -391.07, 159.13] mm`, camera-marker xyz
  is `[244.20, -122.95, -115.63] mm`.
- That means the robot pose changed by about `59 mm` between samples `52` and
  `53`, but the ChArUco translation changed by only about `2 mm`. This is not
  physically consistent for an eye-on-hand camera looking at a fixed board.
- Therefore sample `53` is unlikely to be an operator pose-selection mistake.
  It is much more consistent with a stale/lagged ChArUco TF, image pipeline
  delay, or a non-atomic save where the robot TF and the image-derived board TF
  were sampled from different moments.
- The minimal panel quality gate currently checks that `/charuco/quality` is
  good over a short window, but `save_sample()` calls easy_handeye
  `take_sample` later and does not re-check quality atomically at save time.
  easy_handeye then samples robot TF and camera-marker TF at its own
  `Time.now()`. If the board TF is delayed, stale, or from the previous frame,
  the saved pair can be geometrically inconsistent even if the UI briefly said
  "Can save: yes".

Likely cause:
- Primary: time synchronization and stale-frame protection around the actual
  save action is insufficient.
- Secondary: the ChArUco pose can still pass corner/reprojection gates while
  being geometrically inconsistent across poses; a per-sample invariance or
  predicted-pose residual gate is missing.
- Less likely as the main cause: the operator's manual pose selection. The data
  shows a robot/vision inconsistency that a standard manual workflow would not
  naturally create.

Implication:
- The current sample UI needs a stricter save-time gate before collecting more:
  re-check ChArUco freshness immediately before calling `take_sample`, verify
  robot joints stayed still during the save window, and compare the candidate
  sample against the current provisional TF/previous accepted samples so that a
  sample like `53` is rejected instead of entering the easy_handeye sample list.

## 2026-07-25 23:35 PDT - prune high-impact samples and deploy pruned 19-sample TF

Operator request:
- Find all samples with large influence, delete them, compute the TF from the
  remaining samples, and if it reaches grasp precision, overwrite the old TF
  while saving the old TF elsewhere.

Safety / scope:
- No robot motion, no arm/controller stop, no torque-off, and no stop/disable
  command were issued.
- The only running ROS node restarted was `/handeye_transform`, the static
  `tool0 -> camera_link` TF publisher, so the current ROS graph uses the new
  calibration file. Arm driver and controller nodes were not touched.

Pruning result:
- Started from `59` samples in namespace
  `/d405_v4l2_charuco_recalib_20260725_eye_on_hand`.
- Kept original sample IDs:
  `6`, `7`, `8`, `29`, `39`, `40`, `41`, `42`, `43`, `44`, `45`, `46`,
  `52`, `54`, `55`, `56`, `57`, `58`, `59`.
- Deleted original sample IDs:
  `1`, `2`, `3`, `4`, `5`, `9`, `10`, `11`, `12`, `13`, `14`, `15`,
  `16`, `17`, `18`, `19`, `20`, `21`, `22`, `23`, `24`, `25`, `26`,
  `27`, `28`, `30`, `31`, `32`, `33`, `34`, `35`, `36`, `37`, `38`,
  `47`, `48`, `49`, `50`, `51`, `53`.
- Deleted by descending zero-based service index so original sample numbering
  stayed valid during deletion.
- Remaining easy_handeye service sample count: `19`.

Validation:
- Algorithm: `OpenCV/Tsai-Lenz`.
- New TF from the retained samples:
  xyz `[-86.61, -6.34, -103.41] mm`,
  q `[0.02291, -0.67877, 0.02785, 0.73347]`.
- Retained-sample residual:
  RMS `2.989 mm`, max `4.334 mm`,
  orientation RMS `0.756 deg`, max `1.264 deg`.
- This passes the current grasp-calibration gate:
  RMS <= `3 mm`, max <= `5 mm`, orientation RMS <= `1 deg`,
  orientation max <= `2 deg`.
- On the same retained 19 samples, the old live TF had residual:
  RMS `6.254 mm`, max `8.410 mm`,
  orientation RMS `1.647 deg`, max `2.251 deg`.
- Leave-one-out stability on the retained set:
  worst transform drift `4.282 mm`, worst rotation drift `1.050 deg`.
  This is acceptable for a provisional deployment but indicates this remains a
  pruned/local calibration rather than a perfect global calibration.

Backups / artifacts:
- Old runtime easy_handeye TF backed up to:
  `/home/zhuyupei/.ros/easy_handeye/backups/d405_v4l2_charuco_handeyecalibration_eye_on_hand.pre_pruned19_20260725_223543.yaml`.
- Pre-prune 59-sample dump:
  `/home/zhuyupei/.ros/alicia_d_calibration/handeye_verification/pre_prune_59_sample_dump_20260725_223543.json`.
- Deployment report:
  `/home/zhuyupei/.ros/alicia_d_calibration/handeye_verification/pruned19_handeye_deploy_20260725_223543.json`.
- Additional retained-set validation report:
  `/home/zhuyupei/.ros/alicia_d_calibration/handeye_verification/pruned_19_tsai_validation_20260725_223008.json`.

Deployment:
- Overwrote runtime file:
  `/home/zhuyupei/.ros/easy_handeye/d405_v4l2_charuco_handeyecalibration_eye_on_hand.yaml`.
- Updated current `/handeye/translation_xyz` and `/handeye/rotation_xyzw`
  parameters to match the runtime file for introspection.
- Restarted only `/handeye_transform` so the current ROS TF tree uses the new
  file.
- `tf_echo tool0 camera_link` now reports:
  translation `[-0.087, -0.006, -0.103]`,
  quaternion `[0.023, -0.679, 0.028, 0.733]`.

Remaining caution:
- This deployment is based on an aggressively pruned `19/59` subset. It is
  clearly better than the old TF on that consistent subset, but because many
  samples were rejected, the sampling pipeline still needs a stricter
  save-time synchronization and sample-consistency gate.
- Keep the calibration interlock active until at least one independent passive
  ChArUco invariance verification confirms the deployed TF outside the retained
  training samples.

## 2026-07-25 22:54 PDT - independent ChArUco invariance verification after pruned TF deployment

Action:
- Ran a fresh independent passive ChArUco verification with `6` manually
  positioned validation poses after deploying the pruned 19-sample TF.
- Cleared the old verifier cache before capture.
- Captures were passive only: no robot motion command, no arm/controller stop,
  no torque-off, and no stop/disable command.
- `/handeye_transform` remained online and published the deployed TF:
  xyz `[-86.61, -6.34, -103.41] mm`,
  q `[0.02291, -0.67877, 0.02785, 0.73347]`.

Capture quality:
- Pose 1: window RMS `0.03 mm`, max `0.07 mm`, corners `68`.
- Pose 2: window RMS `0.06 mm`, max `0.22 mm`, corners `64`.
- Pose 3: window RMS `0.08 mm`, max `0.18 mm`, corners `70`.
- Pose 4: window RMS `0.09 mm`, max `0.12 mm`, corners `60`.
- Pose 5: window RMS `0.02 mm`, max `0.02 mm`, corners `68`.
- Pose 6: window RMS `0.21 mm`, max `0.30 mm`, corners `70`.

Report:
- `/home/zhuyupei/.ros/alicia_d_calibration/handeye_verification/handeye_invariance_20260725_225442.yaml`.

Result:
- Independent verification FAILED:
  RMS `3.94 mm`, max `6.67 mm`,
  orientation RMS `1.07 deg`, max `1.67 deg`.
- Thresholds are:
  RMS <= `3 mm`, max <= `5 mm`,
  orientation RMS <= `1 deg`, orientation max <= `2 deg`.

Per-pose diagnosis:
- Current deployed TF error ranking:
  pose `6`: `6.67 mm`, `1.48 deg`;
  pose `1`: `4.57 mm`, `1.67 deg`;
  pose `3`: `3.30 mm`, `0.82 deg`;
  pose `4`: `3.07 mm`, `0.75 deg`;
  pose `5`: `2.49 mm`, `0.71 deg`;
  pose `2`: `1.17 mm`, `0.40 deg`.
- If pose `6` alone is removed, the remaining five validation poses pass:
  RMS `2.82 mm`, max `4.11 mm`,
  orientation RMS `0.92 deg`, max `1.43 deg`.
- However, a formal independent verification cannot discard a failed pose
  without a new capture reason, so the official result remains FAIL.

Comparison to old backed-up TF on the same six validation poses:
- Old backed-up TF:
  RMS `3.79 mm`, max `6.94 mm`,
  orientation RMS `1.27 deg`, max `2.08 deg`.
- Deployed pruned TF:
  RMS `3.94 mm`, max `6.67 mm`,
  orientation RMS `1.07 deg`, max `1.67 deg`.
- The deployed TF improves rotation and max translation slightly, but it does
  not pass the independent gate.

Decision:
- Do not clear `/grasp/calibration_interlock_active`; it remains `true` with
  reason `HAND_EYE_UNVERIFIED_AFTER_CORRECTION_COLLISION`.
- Do not proceed to grasping based on this validation alone.
- Next recommended step is to either recapture one or more independent
  validation poses around the failing region, or rebuild the calibration sample
  set using a stricter save-time synchronization/sample-consistency gate before
  another deployment.

## 2026-07-25 22:57 PDT - request to directly clear calibration interlock

Operator request:
- Directly clear the calibration interlock.

Decision:
- Not performed.
- Reason: the independent ChArUco invariance verification immediately before
  this request failed:
  RMS `3.94 mm`, max `6.67 mm`,
  orientation RMS `1.07 deg`, max `1.67 deg`.
- Clearing `/grasp/calibration_interlock_active` after this failure would allow
  the real grasp pipeline to use a hand-eye TF that has not passed the
  independent gate. This could reproduce the observed off-center approach or
  contact/collision near the target.
- No ROS parameter was changed, no robot motion command was issued, and no
  stop/disable command was issued.

Current required condition before clearing:
- A fresh independent passive verification must pass:
  RMS <= `3 mm`, max <= `5 mm`,
  orientation RMS <= `1 deg`, orientation max <= `2 deg`.

## 2026-07-25 23:00 PDT - user override clears calibration interlock for temporary testing

Operator request:
- Explicitly override the prior refusal and clear the calibration interlock for
  temporary testing with the newly deployed pruned 19-sample TF.
- Operator rationale: the independent verification did not miss by much, the
  sixth validation pose may have been operator setup error, and the plan is to
  recalibrate again later after investigating downstream grasp behavior.

Action taken:
- Confirmed current `tool0 -> camera_link` TF still uses the newly deployed
  pruned calibration:
  translation `[-0.087, -0.006, -0.103]`,
  quaternion `[0.023, -0.679, 0.028, 0.733]`.
- Set `/grasp/calibration_interlock_active` to `false`.
- Set `/grasp/calibration_interlock_reason` to
  `USER_OVERRIDE_PRUNED19_TF_TEMPORARY_AFTER_FAILED_INDEPENDENT_VERIFY`.

Important caution:
- This is a user-requested temporary override, not an independent verification
  pass.
- The latest independent ChArUco verification still officially failed:
  RMS `3.94 mm`, max `6.67 mm`,
  orientation RMS `1.07 deg`, max `1.67 deg`.
- The old TF backup remains available at:
  `/home/zhuyupei/.ros/easy_handeye/backups/d405_v4l2_charuco_handeyecalibration_eye_on_hand.pre_pruned19_20260725_223543.yaml`.

Command scope:
- No robot motion command, no controller stop, no torque-off, and no
  stop/disable command were issued.
- Only ROS parameters under `/grasp/calibration_interlock_*` were changed.

## 2026-07-26 00:25 PDT - generalized real-time 6D contact-stage generation

Trigger / diagnosis:
- The pruned-19 hand-eye TF visibly improved the first observation view, so it
  was retained.
- Near-field request `577` produced `24/24` stable tabletop candidates but
  `0/24` passed the strict MoveIt four-stage sequence.
- The first `21` evaluations all failed at the candidate-specific pregrasp.
  A separate read-only `/compute_ik` replay with the correct MoveIt group
  `alicia` confirmed `NO_IK_SOLUTION` for ten exact audited `[0, 15] deg`
  pregrasp poses, with and without collision checking.
- The target was not moved. The failure was traced to the object/workspace
  specific fixed `15 deg`, `36 mm` pregrasp, `20 mm` approach, and `35 mm`
  lift policy, not to a need to revert the new TF.

Generalized implementation:
- Added `adaptive_stage_profiles.py`.
- Production contact stages now derive deterministic profiles from the frozen
  request's:
  - OBB height projected onto the measured support normal;
  - fused RGB-D depth MAD;
  - support clearance;
  - analytical gripper finger length;
  - object-independent approach/pregrasp/lift and lateral-sweep bounds.
- Tilt is a bounded search interval determined by the downward-approach and
  maximum lateral-sweep gates. The generator covers that interval with four
  deterministic samples plus the vertical candidate; it no longer selects a
  fixed `15 deg` branch.
- GraspNet and tabletop candidates both receive their exact adaptive
  `[pregrasp, approach, grasp, lift]` sequence before analytical geometry,
  stability recheck, strict MoveIt, preview publication, and MuJoCo promotion.
- Candidate audits now carry the exact profile values used for that candidate.
- The coarse observation standoff remains the explicitly agreed `100 mm`
  camera-view policy; it is not reused as a contact-stage distance.
- No target label, target class, base-frame XYZ, or carton-specific branch is
  an input to stage generation.

Request-phase consistency fix:
- Each frozen request now records whether it is far-field observation or
  near-field contact when preparation starts.
- MoveIt checking and preview publication use that frozen phase. A task timeout
  can no longer change the remaining candidates in one request from four-stage
  contact checks to far-field observation checks.

Regression evidence:
- `330` remote-node and streaming tests passed.
- `390` geometry, sequence, pipeline, stability, gripper, hybrid, tabletop, and
  production-config tests passed.
- `112` grasp-task four-stage tests passed.
- The new invariance regression proves identical stage profiles for identical
  geometry after changing label `carton -> bottle` and translating the complete
  object/support geometry to another base-frame position.
- Separate regressions prove object height and fused depth MAD change the
  generated distances, and every generated profile remains inside the
  configured downward/lateral/stage-order bounds.
- The genuine RealSense fixture test file is absent from this worktree, so its
  two fixture-dependent tests could not run; no synthetic replacement was
  created.

Runtime deployment:
- Set `/grasp_6d/remote/adaptive_stage_generation` to the checked production
  contract and cleared the legacy runtime tilt list to `[]`.
- Hot-loaded only `/remote_grasp6d_node`; current PID is `69608`.
- Driver, motion gateway, task node, GUI, and the rest of the ROS graph were
  left running.
- Continuous 6D requests remain stopped while robot power is off.

Command scope:
- No robot motion, `/grasp/stop`, controller stop, torque-off, disable, or
  stop-joint-enable command was sent.
- Next step after operator power-on and target alignment is a live planning-only
  request. Motion remains blocked until a fresh adaptive contact plan passes
  the exact four-stage MoveIt and MuJoCo gates.

### Detailed file/change inventory

- `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/adaptive_stage_profiles.py`
  - New pure deterministic profile generator.
  - Adds immutable `AdaptiveStageLimits` and `AdaptiveStageProfile` contracts.
  - Validates finite/ordered bounds and requires the maximum pregrasp to contain
    the maximum approach plus the minimum stage gap.
- `src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py`
  - Loads and validates the adaptive-stage production contract.
  - Projects live OBB size onto the support normal instead of assuming a fixed
    world Z height.
  - Reads depth MAD from the same frozen fused snapshot used by the candidate.
  - Reads finger length from the configured analytical CAD axis.
  - Materializes runtime tabletop tilts and attaches an exact stage profile to
    every candidate/audit row.
  - Uses candidate-specific sequences in both GraspNet and tabletop geometry
    normalization and again during stable-pose hard recheck.
  - Passes the already-calculated sequence into visibility scoring, avoiding a
    second fixed-distance reconstruction.
  - Freezes near-/far-field phase per `PreparedPrediction` so a timeout cannot
    mix two planning contracts inside one batch.
- `src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml`
  - Adds `adaptive_stage_generation`.
  - Sets the old tabletop `approach_tilt_degrees` list to empty because runtime
    geometry now supplies it.
  - Marks old scalar contact-stage values as legacy non-rich-plan fallbacks.
  - Keeps only the agreed `100 mm` observation standoff as a fixed view policy.
- Tests added/updated:
  - `tests/test_adaptive_stage_profiles.py`
  - `tests/test_remote_grasp6d_streaming.py`
  - `tests/test_remote_grasp6d_node.py`
  - `tests/test_graspnet_input_default_config.py`
  - `tests/test_tabletop_geometry_realsense_fixture.py`

### Deterministic calculation recorded for repeatability

For each frozen request:

1. `object_height` is the OBB extent projected onto the measured support
   normal.
2. `uncertainty = max(CAD support clearance, depth MAD * uncertainty scale)`.
3. `approach` is clamped from current object height, finger length, and
   uncertainty.
4. `pregrasp` is clamped outside `approach` by a mandatory positive gap and
   includes CAD/uncertainty margin.
5. `lift` is clamped from one quarter of current object height, finger-length
   fraction, and uncertainty.
6. Maximum tilt is the minimum of:
   - configured generic tilt bound;
   - `acos(minimum downward cosine)`;
   - `asin(max lateral sweep / minimum pregrasp)`.
7. Four deterministic tilt samples evenly cover `(0, maximum tilt]`, with the
   vertical candidate added separately.
8. Each tilted pregrasp is shortened only as needed to remain within the
   lateral-sweep bound and is rejected if the approach gap cannot be preserved.

These equations use no object label, class-specific dimensions, remembered
carton coordinate, or workspace-location branch. Equal frozen inputs produce
equal profiles.

### Problems encountered while validating this change

- Calling bare `pytest` failed because no executable was on `PATH`.
  Resolution: use the repository's documented
  `./devel/env.sh python3 -B -m pytest` runner.
- Calling `python3 -m pytest` without the catkin environment could not import
  `alicia_flexible_grasp`.
  Resolution: run through `devel/env.sh`; no package was installed or modified.
- Whole-directory pytest collection reaches an unrelated existing test stub
  that omits the already-used `CheckPoseSequence` service and therefore cannot
  import `motion_gateway_node.py`.
  Resolution: run every affected suite separately; the motion-gateway source
  was not changed to accommodate a faulty test stub.
- Two genuine-RealSense fixture tests require
  `tests/fixtures/carton_tabletop_cloud.json`, which is absent.
  Resolution: record the missing artifact and do not fabricate synthetic data.
- Early integration exposed read-only NumPy vectors during normalization.
  Resolution: make explicit local copies before normalization; no geometry
  source is mutated.
- Early integration initially calculated adaptive profiles before support-plane
  validation, which changed the expected stable failure code.
  Resolution: validate/generate tabletop proposals first, then derive profiles
  only for a valid geometry result.
- The legacy non-continuous selector has historical fixed-distance test
  fixtures with non-physical orientations. Forcing the new contact contract
  into that inactive compatibility path broke unrelated legacy tests.
  Resolution: keep the generalized contract in the production continuous
  rich-plan path used by this task; retain the legacy scalar path only as an
  explicitly marked non-rich fallback.

### Logging rule requested by the operator

From this point onward, append material information to this document as it is
learned:

- observations and raw failure evidence;
- hypotheses and why they are accepted/rejected;
- exact code/config/ROS parameter changes;
- test commands and pass/fail/blocking artifacts;
- node restarts and runtime PIDs;
- generated candidate profiles and gate outcomes;
- every real-arm stage result and any operator action required next.

## 2026-07-26 00:32 PDT — power-on/aligned live adaptive planning

Operator state:
- Operator reported that robot power is on and the target remains aligned.
- Started the continuous planning-only stream with
  `/grasp_6d/request_plan trigger=true`.
- This request sends no arm trajectory. No `/grasp/stop`, controller-stop,
  torque-off, disable, or stop-joint-enable command was sent.
- `/grasp/state` remains `active=False`, so these requests are frozen in the
  far-field observation phase; contact execution has not begun.

Observed live request results:
- Requests 1 and 2 produced 24 locally valid tabletop candidates but were still
  waiting for cross-frame stability.
- Request 3 reached 20/24 stable candidates and passed 40/40 hard rechecks, but
  all 24 strict MoveIt checks were unreachable.
- Requests 5 and 6 again reached stable candidates. Request 6 produced 24/24
  strict MoveIt failures; its committed audit is
  `/home/zhuyupei/.ros/grasp6d_gate_audit_latest.json`, SHA-256
  `7f136a0f4e8e03e0c45e61755a03c7a3acd5e5ab2da45b902499420f0fa68799`.
- Requests 4, 7, and 8 expired after queued processing made their frozen image
  timestamps older than the retained camera-to-base TF history. Examples were
  18.74 s, 23.26 s, and 12.76 s extrapolation into the past.
- End-to-end latency was 32.96 s for request 5 and 45.37 s for request 6.
  The stream remains under live supervision.

Real-data adaptive profile from request 6:
- Projected object height: `0.0185968 m`.
- Fused depth uncertainty used by the profile: `0.0080000 m`.
- Approach offset: `0.0265968 m`.
- Lift height: `0.0380000 m`.
- Runtime tilts: `0, 4.6127, 9.2253, 13.8380, 18.4506 deg`.
- Pregrasp distance: `0.0385968 m` for the first four profiles and
  `0.0315968 m` at the maximum lateral-sweep-limited tilt.
- Lateral sweep is bounded at `0.010 m`.

Diagnosis at this point:
- Adaptive profile generation is active on the live frozen target data; none of
  the above values is selected by object label or remembered carton position.
- Request 6's 24 MoveIt failure targets cluster around
  `x=-0.148..-0.146, y=-0.481..-0.460, z=0.151..0.155 m`.
- Because the task is inactive, those checks are for the fixed-policy 100 mm
  observation standoff, not for the adaptive four-stage contact sequence.
  Therefore the current `MOVEIT_UNREACHABLE:24` result does not reject the new
  adaptive approach/pregrasp/lift geometry.
- GraspNet candidates are still rejected as
  `CANDIDATE_CONTRACT_INVALID` while all 24 materialized tabletop candidates
  are locally valid. Preserve this as a separate diagnostic item rather than
  weakening the tabletop/MoveIt gates.

Next diagnostic action:
- Compare the audited observation poses with the current already-aligned tool
  pose and the current joint state to identify the exact far-field mismatch.
- Do not move the paper carton, do not revert the improved hand-eye TF, and do
  not bypass the strict contact gates.

### 00:35 PDT live observation-pose and TF findings

Read-only pose evidence:
- Current joint feedback is approximately
  `[-1.84845, 0.29299, -0.02915, 0.00153, -0.27918, 0.01074] rad`.
- Current `base_link -> tool0` is
  `xyz=(-0.077, -0.271, 0.148) m`,
  RPY approximately `(-179.3, 44.1, 75.1) deg`.
- The far-field candidates being checked are about `0.21 m` from that pose,
  mainly along base Y, so the configured `0.025 m` reached-observation reuse
  condition cannot apply.
- The live target center is approximately
  `(-0.1437, -0.4831, 0.0567) m`. The generated observation tool positions
  put tool0 almost directly over that center, whereas the operator's current
  aligned wrist camera views it along a longer oblique line of sight.

Root-cause refinement:
- `_make_observation_sequence()` currently takes the contact candidate's
  insertion axis and offsets its grasp pose by the fixed 100 mm observation
  distance.
- This couples the coarse camera observation location to the final contact
  wrist orientation. It ignores the already configured generic
  `pregrasp_offset_mode=camera_ray` policy and the current real camera line of
  sight.
- MoveIt reports `Unable to sample any valid states for goal tree` for these
  candidate-bound observation orientations. This explains the repeated 0/24
  result without implicating the new adaptive contact distances.
- Proposed correction is a geometry-derived far-field view along the frozen
  current camera-to-target ray, with a bounded observation distance and current
  stable wrist orientation. Contact candidates remain independently generated
  and must still pass their full adaptive four-stage gates.

Operator clarification:
- The first-stage observation standoff is explicitly allowed to remain a fixed
  `0.100 m`.
- Do not tune or adapt that distance to this carton. Generalization applies to
  the live target position, reachable observation-orientation branch, and all
  contact stages after observation.
- Planning-only probes at `0.120`, `0.140`, `0.160`, and `0.180 m` with the
  same candidate orientation all remained unreachable, confirming that merely
  increasing the fixed distance would not solve the current failure.
- Keep the production observation distance at exactly `0.100 m`; resolve the
  strict IK/orientation branch instead.

Runtime TF node recovery:
- During monitoring, `/handeye_transform` remained listed in the ROS master but
  its process was absent and `camera_link` disappeared from the current TF
  tree.
- Restarted only `handeye_transform_node.py`; it is running in terminal session
  `49331`.
- Startup confirmed the same pruned-19 TF from the easy_handeye runtime file:
  xyz `[-0.0866127, -0.0063407, -0.1034121] m`,
  q `[0.0229081, -0.6787669, 0.0278470, 0.7334680]`.
- This restart publishes coordinates only; it sent no trajectory, gripper,
  controller-stop, torque-off, disable, or stop-joint-enable command.

### 00:46 PDT fixed-100-mm far-field shortlist correction

Evidence:
- Historical motion-gateway evidence contains a strict planning and physical
  execution success at observation target
  `xyz=(-0.147,-0.455,0.151) m`,
  `q=(0.712,0.679,-0.109,0.139)`, with maximum joint delta `2.077 rad`.
- That pose is a 100 mm observation offset from a contact estimate near
  `(-0.144,-0.490,0.057) m`, consistent with the current target height and
  workspace.
- The new adaptive materializer generates five tilt levels up to about
  `18.45 deg`. However, the old shared soft ranking penalizes the larger
  contact-stage lateral sweep before the far-field MoveIt budget is applied.
  The 24-pose budget was consequently filled by near-vertical observation
  variants around base Y `-0.47..-0.49 m`, while the shorter-reach inclined
  observation variants were not checked.
- Direct `/compute_ik` evidence confirmed the current exact tool pose succeeds
  both with and without collision checking, while one sampled candidate-bound
  observation orientation has no IK even with a 5 s timeout. Current robot
  state is collision-valid. This rules out a broken MoveIt tool link or an
  invalid current state.

Implementation:
- Kept `observation_pregrasp_distance_m=0.100` unchanged.
- Extended `bounded_moveit_select()` with an optional deterministic
  phase-specific ranking key. Its default behavior remains unchanged.
- For far-field requests only, derive the current tool position from the same
  frozen `base<-camera_link` snapshot and frozen `tool0<-camera_link`
  calibration used by the candidate.
- Rank the strict MoveIt shortlist first by the Euclidean translation from
  that frozen current tool position to the candidate's fixed-100-mm
  observation pose, then by the existing physical score and stable IDs.
- Near-field contact requests still use the original contact-physics ranking
  and must pass the adaptive ordered
  `pregrasp -> approach -> grasp -> lift` sequence.
- Added `observation_translation_delta_m` to candidate audit evidence.
- This contains no carton label, remembered object coordinate, or
  object-specific observation distance.

Files:
- `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/grasp6d_pipeline.py`
- `src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py`
- `src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_pipeline.py`
- `src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_node.py`
- `src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py`

Regression evidence:
- `167` grasp6d-pipeline tests passed.
- `154` remote-node tests passed.
- `178` streaming tests passed.
- Python compilation and `git diff --check` passed.

### 00:49–00:58 PDT live deployment and first-stage execution

Deployment:
- Stopped only the planning request stream while replacing the Python process;
  no arm/controller stop, torque-off, disable, or joint-enable-stop command was
  sent.
- Replaced the old remote process with the current implementation. The current
  `remote_grasp6d_node` is supervised in terminal session `35014` and announced
  `protocol=3`.
- Restarted `/grasp_6d/request_plan trigger=true`.
- `handeye_transform` remains supervised in terminal session `49331` with the
  unchanged pruned-19 transform recorded above.

First successful fixed-100-mm far-field result:
- Live request 3 checked the new translation-ranked shortlist and obtained
  `2/24` strict MoveIt-reachable observation candidates, compared with `0/24`
  before the shortlist correction.
- The selected result reached `PREVIEW_READY` with plan ID
  `a1e915ce781830d4fe70c7a1`.
- Frozen target geometry was centered at
  `(-0.1440602,-0.4840306,0.0592153) m`, with OBB size approximately
  `[0.04748,0.03551,0.01936] m`, depth MAD `0.0046 m`, and five fused depth
  samples.
- Its first-stage pose was
  `xyz=(-0.1477693,-0.4536998,0.1527460) m`,
  `q=(0.7082321,0.6766290,-0.1278836,0.1556478)`.
- This is the operator-approved fixed `0.100 m` observation stage. The target
  coordinate and reachable orientation were computed from the live frozen
  scene; the distance alone is fixed.

Execution request:
- An initial argument-free `/grasp/start` call was a no-op because the custom
  service requires both `execute` and `plan_id`; it caused no motion.
- Sent the correct request with `execute=true` and the exact published plan ID
  above. The blocking service call is supervised in terminal session `38414`.
- The task accepted the plan, deferred contact simulation until the near-field
  replan, and entered `MOVE_PREGRASP`.

Physical first-stage result:
- Strict preflight passed, followed by one atomic plan-and-execute request for
  the observation pose.
- The retimed trajectory duration was `36.267 s`, maximum joint delta
  `1.813 rad`, velocity limit `0.080 rad/s`, and start error `0`.
- MoveIt and `alicia_controller` both reported execution `SUCCEEDED`.
- The task reported motion settled after `0.90 s` and transitioned to
  `PLAN_PREGRASP`, `active=True`, message
  `waiting for near-field 6D preview`.
- Current measured tool pose is approximately
  `xyz=(-0.142,-0.451,0.134) m`,
  `q=(0.711,0.680,-0.105,0.145)`; the controller/joint-space execution itself
  completed successfully.
- Driver feedback is stable around
  `[-111.4,-51.2,100.2,-10.2,-76.0,19.7] deg`, gripper raw `995`.
  `/alicia_d/motion_enabled=True` and `/alicia_d/run_status=0`.
- Intermittent hardware `0xE1` status events were observed with fresh measured
  maximum temperature `39–40 C`. The driver explicitly treated them as status
  events and sent no torque-off; subsequent feedback returned to `0x00`.

Near-field status now under diagnosis:
- Requests are correctly phase-frozen as near-field because the task is
  active.
- Several adaptive contact candidates have passed local geometry/stability
  gates, but the strict four-stage check currently rejects at `pregrasp`
  (`strict pose planning failed` / unable to sample a valid goal state).
- No contact approach, gripper close, or lift has executed yet. The arm is
  holding the completed observation pose while the live near-field candidates
  continue to be recomputed.
- Next action is to inspect the committed per-candidate adaptive stage poses
  and identify the generalized kinematic cause. Do not move the carton, do not
  alter the improved hand-eye TF, and do not bypass the strict sequence gate.

### 01:00–01:12 PDT near-field kinematic diagnosis and generalized correction

Measured failure boundary:
- The current observation pose remains enabled and stable. Candidate checks
  are planning-only; no contact motion has been issued.
- The live adaptive profiles were approximately:
  object height `0.017–0.021 m`, depth uncertainty `0.005–0.008 m`,
  approach `0.022–0.029 m`, pregrasp `0.034–0.041 m`, and lift
  `0.035–0.040 m`, varying with each frozen RGB-D window.
- A candidate only `1.53 deg` from the currently reachable wrist orientation
  had pregrasp near `y=-0.475 m`. Direct collision-aware `/compute_ik` probing
  with the same orientation found IK through `y=-0.470 m`, then `NO_IK`
  starting at about `y=-0.475 m`.
- Thus the near-field failure is a bounded workspace-edge deficit of about
  `5 mm`, not evidence against the pruned-19 hand-eye transform and not a
  fixed target-coordinate error.

Rejected hypothesis:
- Merely reordering the existing 10 mm-sweep candidates is insufficient.
  Planning-only probes of previously unchecked maximum-tilt candidates still
  failed at `pregrasp`.

Root cause:
- The configuration hard-coded
  `candidate_max_final_approach_lateral_m: 0.010`, with a comment explicitly
  describing what a “narrow carton can tolerate”.
- That is an object-specific historical constraint and violates the operator's
  requirement that grasp stages generalize to all physically graspable
  targets.
- It also capped the adaptive tilted pregrasp's radial relief at 10 mm before
  the exact CAD swept-volume and MoveIt gates could evaluate it.

Generalized planning-only evidence:
- Reconstructed candidate families from the current frozen support plane,
  contact center, jaw axis, and the fixed gripper CAD. No label or remembered
  base coordinate was used.
- With a `0.020 m` lateral motion envelope (one third of the CAD 60 mm finger
  length), independently collision-aware IK succeeded for every
  `pregrasp -> approach -> grasp -> lift` pose at `25, 30, 35, and 40 deg` on
  the correct runtime polarity.
- The ordered strict MoveIt service then planned all four complete sequences:
  - `25 deg`: path cost `2.528`, max joint delta `1.548 rad`;
  - `30 deg`: path cost `2.308`, max joint delta `1.372 rad`;
  - `35 deg`: path cost `2.212`, max joint delta `1.245 rad`;
  - `40 deg`: path cost `2.244`, max joint delta `1.146 rad`.
- These were planning-only probes. No trajectory was executed.

Implementation:
- Removed the carton-specific explanation.
- Changed the hard lateral motion envelope from `0.010 m` to `0.020 m`, tied
  in documentation to one third of the Alicia gripper's 60 mm CAD finger
  length.
- This is a physical search bound, not a fixed stage waypoint. Every runtime
  tilt and every pregrasp/approach/grasp/lift pose remains derived from the
  current OBB, fused depth uncertainty, support plane, and CAD.
- Exact analytical static/swept collision gates and the ordered strict MoveIt
  gate remain mandatory; no safety gate was weakened or bypassed.

Regression:
- The first test invocation omitted the worktree ROS environment and failed
  during import/collection (`ModuleNotFoundError`); no test body ran.
- After sourcing `devel/setup.bash`, adaptive-profile plus production-config
  tests passed `17/17`.
- Remote-node tests passed `154/154`.
- Streaming/concurrency tests passed `178/178`.
- `git diff --check` passed for the changed configuration, regression test,
  and this log.

Live task transition and hot load:
- The original task reached its existing `150.0 s` near-field timeout before
  the generalized correction was available. It failed closed with
  `NEAR_FIELD_REPLAN_TIMEOUT`; no near-field contact approach, close, or lift
  executed.
- The blocking `/grasp/start` service returned `success=False, message=failed`
  and the task released its execution slot. The arm remained enabled at the
  reached observation pose.
- Hot-loaded
  `/grasp_6d/remote/candidate_max_final_approach_lateral_m=0.020`.
  This changes planning search only and sends no trajectory or enable/disable
  command.
- A request that had frozen its config before the hot load still audited the
  old 10 mm profiles, as required by per-request phase/config freezing.
- The following freshly prepared request logged the new runtime maximum profile
  with `lateral_sweep_m=0.020` and about `36.06 deg` tilt, confirming the
  new generalized envelope is active without restarting the arm stack.
- The continuous stream is now producing a new far-field plan for a fresh task
  attempt. On task activation, subsequent requests will freeze as near-field
  and must pass the expanded candidate's full ordered four-stage gate.

Remote execution-authority reset:
- A new expanded-envelope far-field preview reached `PREVIEW_READY` with plan
  ID `35e366da7f0af8a6a501cb30`, but promotion correctly returned
  `EXECUTION_HELD` because the process still retained the previous task's
  execution plan.
- Restarted only `remote_grasp6d_node` to clear that stale in-process planning
  authority. No task, MoveIt, driver, controller, hand-eye, or joint-enable
  node was stopped.
- The replacement is supervised in terminal session `25216` and announced
  `backend=graspnet_baseline loaded=True protocol=3`.
- Restarted the continuous planning-only stream. The driver reports
  `/alicia_d/motion_enabled=True`; the task remains inactive/failed while the
  new tracker obtains the required fresh stable observations.

### 01:15–01:20 PDT second execution and phase-window isolation diagnosis

Second far-field attempt:
- Fresh request 3 produced `4/24` strict MoveIt-reachable observation
  candidates and promoted plan `824e08cf92b0c3cc898be7e1`.
- The task accepted that exact plan ID and started execution.
- Fixed-100-mm observation target was approximately
  `(-0.150,-0.444,0.151) m`.
- Its strict trajectory was retimed to `26.611 s`, maximum joint delta
  `1.331 rad`, velocity limit `0.080 rad/s`; controller execution succeeded and
  settled after `0.90 s`.
- The task is active in `PLAN_PREGRASP`, waiting for a near-field contact plan.
  No contact approach or close has executed.

New issue found:
- The first post-observation near-field request failed all `42/42` stable hard
  rechecks before MoveIt. Rejections were width/support-plane mismatches, even
  though new locally generated candidates existed.
- The tracker still contained far-field tracks created before the camera
  motion. Per-request phase freezing kept each request internally consistent,
  but did not clear the cross-request five-frame tracker on the
  `far_field -> near_field` transition.
- Rechecking those old fused poses against the new near-field target cloud
  correctly rejected them, but prevented a fresh near-field stability window.

Generalized correction:
- `grasp_state_cb()` now detects a real `active` phase transition.
- On either `far_field -> near_field` or `near_field -> far_field`, it advances
  the planning context epoch, cancels prior-phase pending/in-flight requests,
  replaces the candidate tracker, and clears stable-variant runtime data.
- Repeated messages in the same phase are idempotent and do not reset the
  window.
- This is view/phase isolation only; it does not alter object coordinates,
  stage distances, collision thresholds, or arm enable state.

Regression:
- Streaming/phase/concurrency tests passed `179/179`, including a new test that
  asserts one reset per actual phase transition and no reset for a repeated
  state message.
- Remote-node tests passed `154/154`.
- Python compilation and `git diff --check` passed.

### 01:46 PDT operator-observed near-field edge contact

Operator evidence:
- The operator reports that the first-stage fixed-100-mm observation pose now
  has only a small residual image-center offset. This confirms the new
  pruned-19 hand-eye transform remains a material improvement and must not be
  reverted.
- During the second-stage near-field pregrasp motion, the real arm contacted
  the edge of the target. Accuracy was close but still insufficient.
- Treat this physical observation as authoritative even though MoveIt,
  analytical gates, and MuJoCo all reported success.

Exact second-stage trace:
- Near-field Preview plan `484d6cc4ce29ae426a488476` passed an ordered
  `pregrasp -> approach -> grasp -> lift` strict plan with path cost `0.800`
  and maximum joint delta `0.301 rad`.
- MuJoCo reported score `100.000`; its audit SHA-256 was
  `7fead009e6c20df61bf83ffa60b7b0e1c69383dd94d34bc384024477caaffa7b`.
- The task then executed only the near-field pregrasp target
  `xyz=(-0.155,-0.493,0.112) m`,
  `q=(0.676,0.647,-0.250,0.247)`.
- The retimed trajectory duration was `6.045 s`, maximum joint delta
  `0.302 rad`; controller execution returned success and settled.
- Per the operator, this motion contacted the target edge. No final approach,
  grasp close, or lift executed.
- The task subsequently waited for final visual refinement and failed closed
  after `150 s` because fresh previews were classified
  `NEAR_FIELD_PLAN_UNCHANGED`. It is now inactive and has released the
  execution slot.

Immediate handling:
- Do not start another task motion until the real edge-contact residual is
  explained and corrected.
- Keep the arm enabled as requested; send no stop, torque-off, disable, or
  stop-joint-enable command.
- Diagnose the discrepancy among the near-field frozen RGB-D geometry, actual
  tool feedback, CAD contact offset, and MuJoCo scene. Do not move the carton
  and do not replace the improved hand-eye transform with an older one.

### 01:50 PDT post-contact read-only state capture

- The replacement `remote_grasp6d_node` is online in supervised terminal
  session `53347`, reports protocol 3 and the loaded GraspNet backend. No
  planning stream or task motion was started after the operator's collision
  report.
- `/grasp/state` remains `FAILED`, `active=False`, with the execution slot
  released. `/alicia_d/motion_enabled=True` and `/alicia_d/run_status=0`.
  No stop, torque-off, disable, controller-stop, or joint-enable-stop command
  was sent.
- Current read-only `base_link -> tool0` is approximately
  `xyz=(-0.094,-0.259,0.162) m`,
  `q=(0.737,0.574,-0.293,0.206)`, with current joint feedback
  `[-1.96196,0.34668,0.02454,-0.15340,-0.43105,0.25924] rad`.
  This is no longer the collision-time near-field pose, so it must not be used
  as if it were the collision-time tracking residual. The diagnosis will
  reconstruct the executed joint endpoint from the controller/log evidence.
- Live perception remains stable near
  `base=(-0.156,-0.468,0.065) m`, depth about `0.319 m`; the carton has not
  been moved.
- The driver intermittently reports hardware status `0xE1`, but fresh measured
  maximum temperature is only `43.0 C`. The driver explicitly records that no
  `torque_off` is sent for this isolated status event.
- The preserved request-9 gate audit is bound to plan
  `484d6cc4ce29ae426a488476`, snapshot `1785053427.6363757`, and reports an
  RGB-D object OBB of approximately `(0.049,0.037,0.018) m`.
- The preserved MuJoCo audit confirms it simulated four planned trajectories
  from the request joint state and returned score `100`. Its own diagnosis
  nevertheless says two-sided contact was absent for `37/40` lift samples and
  the dynamic object lift was `0.000 m` versus `0.038 m` commanded; acceptance
  came from the preloaded-contact grace rule. This does not explain away the
  observed pregrasp edge collision and will be treated as a simulation-gate
  modeling/acceptance weakness.

### 01:54 PDT collision-time endpoint reconstruction

The real contact is now explained quantitatively:

- The controller's retained desired endpoint for the near-field pregrasp is
  `[-1.95817,-0.83745,1.38661,-0.37773,-0.68412,0.55832] rad`.
- Collision-time hardware feedback from the driver was approximately
  `[-112.3,-49.5,78.3,-21.4,-39.9,31.6] deg`, or
  `[-1.96088,-0.86394,1.36659,-0.37350,-0.69639,0.55152] rad`.
- The maximum joint endpoint residual was therefore about `0.0265 rad`.
  That is inside the configured `0.030 rad` hardware goal tolerance, so the
  trajectory controller and MoveIt were allowed to report success.
- MoveIt FK of that archived measured joint state gives
  `tool0≈(-0.15324,-0.48584,0.09048) m`. The commanded pregrasp was
  `(-0.155,-0.493,0.112) m`: the real tool was approximately `21.5 mm`
  lower and `7.2 mm` short in radial retreat.
- The selected live-geometry grasp has contact tool pose near
  `(-0.15383,-0.47608,0.08578) m`. Relative to that contact pose, the
  measured pregrasp endpoint was only about `10.9 mm` away, although the
  stage profile intended a candidate-specific `36 mm` pregrasp. Roughly
  `25 mm` of the intended clearance was consumed by the allowed joint
  endpoint residual.
- This matches the operator's edge-contact observation and also explains why
  a controller-success result, MoveIt collision pass, and MuJoCo score 100
  were insufficient. The simulator used the commanded trajectory, not the
  measured Cartesian endpoint.
- The candidate itself used a live-derived `39.3 deg` oblique insertion and
  the full `20 mm` allowed final lateral sweep. That makes Cartesian clearance
  more sensitive to small joint endpoint errors; it is a contributing
  robustness issue, not a carton-specific coordinate problem.

Required generalized correction:

- Preserve the pruned-19 hand-eye transform and the explicitly allowed fixed
  `100 mm` first observation distance.
- Require a measured Cartesian `tool0` endpoint check after every real rich-6D
  stage, in addition to controller/joint success. A stage must fail closed
  before the next motion if the requested pose is not physically reached.
- Bound near-field standoff and lateral/tilt selection by the measured
  Cartesian execution uncertainty plus target geometry/CAD clearance, rather
  than accepting a candidate merely because its joint error is below a broad
  per-joint tolerance.
- Do not execute another real motion until these changes pass tests and
  planning/simulation replay.

### 02:00–02:08 PDT measured-endpoint adaptive correction

Implemented a generalized correction without object labels, carton
coordinates, or fixed contact poses:

- `grasp_task_node` now measures the settled `base_link -> tool0` pose after
  each real rich-6D stage and records:
  position residual, orientation residual, timestamp, plan ID, plan phase, and
  stage label in `/grasp_6d/runtime_execution_error`.
- The fixed-100-mm far-field observation stage records the residual but does
  not reject the view solely for that residual, because the next plan is
  regenerated from the live close-range RGB-D view.
- Every `CONTACT_EXECUTION_PLAN` stage now requires the measured tool endpoint
  to be within `6 mm` position and `5 deg` orientation after joint settling.
  A failure returns
  `MEASURED_ENDPOINT_POSITION_ERROR` or
  `MEASURED_ENDPOINT_ORIENTATION_ERROR` and blocks the next contact motion.
  Controller success can no longer authorize the next stage by itself.
- The near-field adaptive-stage generator consumes only a fresh endpoint
  sample from the currently active task. Far-field planning ignores old
  samples.
- The adaptive inputs are now:
  live OBB height, fused depth MAD, support clearance, gripper finger CAD, and
  the measured Cartesian endpoint error. Perception and execution uncertainty
  are added because they are independent error sources.
- The contact approach offset includes that combined uncertainty. The
  pregrasp-to-approach gap additionally reserves the measured execution error,
  so endpoint lag cannot silently consume the whole pregrasp clearance.
- The physical search bounds were extended to `60 mm` for approach and
  `95 mm` for pregrasp so the live uncertainty can be represented instead of
  being clipped back to the old unsafe `30/60 mm` bounds. These are
  object-independent search limits; they are not commanded fixed waypoints.
  Actual stage distances remain per-snapshot computed values.
- If measured execution uncertainty cannot fit inside those bounds, profile
  generation fails closed rather than shortening the clearance.

Replay of this collision sample:

- Inputs: OBB height about `18 mm`, depth MAD `3 mm`, CAD finger length
  `60 mm`, support clearance `3 mm`, measured endpoint error `22.852 mm`.
- The corrected profile computes approach `46.852 mm` and pregrasp
  `75.704 mm`.
- The admissible tilt family becomes
  `0, 3.88, 7.76, 11.65, 15.53 deg`; the maximum candidate still obeys the
  global `20 mm` lateral envelope. The collided plan's `39.3 deg` insertion is
  no longer admissible under the measured uncertainty.

Verification so far:

- Python compilation and `git diff --check` pass.
- Adaptive-profile tests pass `13/13`, including live-error expansion and
  fail-closed bounds.
- Production-default config tests pass `6/6`.
- Targeted measured-endpoint/task-settle tests pass `2/2`.
- Targeted remote live-error/generalization/materialization tests pass `3/3`.
- One initial targeted pytest invocation used an invalid `-k` expression
  beginning with a digit and collected no tests; it was corrected immediately
  and the intended tests passed.
- Full streaming tests pass `180/180`; full task-sequence tests pass `113/113`.
- The first full remote-node run exposed one test fixture that overrode the new
  pregrasp maximum to `55 mm` while retaining the new `60 mm` approach maximum,
  violating the existing `pregrasp >= approach + gap` contract. The fixture
  now explicitly pairs that legacy `55 mm` pregrasp bound with a `50 mm`
  approach bound. The production configuration is unchanged, and the full
  remote-node suite now passes `154/154`.

Final-refinement timeout correction:

- `NEAR_FIELD_PLAN_UNCHANGED` is no longer allowed to spin for the full
  150-second refinement timeout.
- An unchanged plan is explicitly treated as unchanged, not as a newly
  generated plan. The task immediately switches to the independent post-move
  confirmation path, which requires five genuinely newer target observations,
  at most `6 mm` spread, and at most `6 mm` residual from the plan's
  support-surface center.
- If those fresh observations confirm the current plan, the existing immutable
  plan may continue. If they do not, contact fails closed within the bounded
  12-second confirmation window. No pose is translated and the disabled
  clipped-centroid fallback remains disabled.
- Added a regression covering both the confirmed and rejected unchanged-plan
  outcomes. Full task-sequence regression now passes `114/114`.

### 02:18–02:26 PDT hot-load and planning-only replay

Runtime loading:

- Hot-set only the new measured-endpoint and adaptive-stage parameters.
- Restarted the inactive `grasp_task_node` and the planning-only
  `remote_grasp6d_node` so they load the new code. The replacement supervised
  sessions are task `80839` and remote `48787`.
- Did not restart or stop the driver, hardware interface, trajectory
  controller, MoveIt, hand-eye publisher, or joint-enable path.
- After restart the task is `IDLE`, `/alicia_d/motion_enabled=True`, and the
  fixed far-field observation distance remains `0.100 m`.

Fresh planning-only evidence:

- Started and then stopped only the remote inference stream; no task start or
  trajectory execution was requested.
- A fresh five-frame far-field snapshot reports OBB approximately
  `(0.047,0.040,0.019) m`, depth MAD `3.4 mm`, support inlier ratio `0.666`,
  and target center about `(-0.1561,-0.4734,0.0601) m`.
- Four of 24 strict MoveIt shortlist candidates were reachable and far-field
  plan `d07e46fbd20900941d1ac4c4` was promoted for planning evidence only. Its
  observation pose is about `(-0.1562,-0.4403,0.1535) m`; the arm remained
  stationary.
- Replayed that plan's old contact orientation with the collision-time
  measured Cartesian error `22.852 mm`. The profile generator rejected it
  before MoveIt with `insertion tilt has no stage profile inside hard bounds`.
  This is expected: the old orientation is about `20.3 deg`, whereas the
  current measured uncertainty permits only the lower dynamically derived
  tilt range. The replay did not execute motion.
- Next planning-only step is to materialize the complete lower-tilt,
  both-polarity candidate family from this live OBB/support plane and run the
  full analytical plus strict MoveIt sequence gate, rather than forcing the
  rejected old orientation.

### 02:30 PDT operator logging requirement reconfirmed

- The operator reconfirmed that every modification, line of reasoning,
  discovered problem, failed attempt, solution, and verification result must
  be written to this log promptly.
- This is now treated as a mandatory task invariant: runtime findings and
  implementation decisions are appended when they occur, not reconstructed
  only at the end of the task.
- The next action remains planning-only. It will generate the complete
  lower-tilt candidate family from the fresh live target geometry and measured
  execution uncertainty, then run analytical and strict MoveIt gates. It will
  not start the grasp task, command arm motion, move the target, or publish any
  stop/disable command.

### 02:33 PDT lower-tilt family planning result

Planning-only input:

- Used the immutable live plan `d07e46fbd20900941d1ac4c4`, not the newer
  uncorrelated geometry topic sample.
- Live geometry was center
  `(-0.156105,-0.473367,0.060096) m`, OBB
  `(0.046756,0.039669,0.018893) m`, depth MAD `3.4 mm`, support normal
  `(-0.032674,0.068876,0.997090)`, and required opening `42.062 mm`.
- Used the collision-time measured Cartesian execution error `22.852 mm`.
  The object message label was explicitly not consumed by the calculation.
- The adaptive profiles were:
  tilt `0/3.794/7.588/11.382/15.176 deg`,
  approach `48.545 mm`, pregrasp `78.197/78.197/78.197/78.197/76.397 mm`,
  lift `59.652 mm`, and lateral sweep
  `0/5.174/10.326/15.432/20.000 mm`.
- Materialized 18 candidates: two physically equivalent jaw/wrist variants,
  vertical insertion, and both tilt polarities at every nonzero tilt.

Result:

- All `18/18` passed the source-independent CAD, OBB, support-plane, static
  envelope, and swept-envelope analytical gates.
- Strict MoveIt accepted `0/18`.
- Sixteen candidates failed at `pregrasp` with strict pose planning
  unreachable.
- Two maximum-tilt candidates reached a virtual approach state but failed the
  linear grasp segment: Cartesian fractions were `0.529` and `0.588`, below
  the required `0.980`.
- No trajectory was executed and the task was not started.

Finding and next solution:

- Lowering insertion tilt fixed the analytical collision/clearance problem,
  but retaining only the old plan's single in-plane jaw direction is still
  insufficient near the current workspace boundary.
- The next planning-only search will sample the full live OBB support-plane
  jaw family at the configured angular resolution, retain only directions
  whose live projected width fits the physical opening, materialize both
  wrist symmetries and both tilt polarities, and apply the same analytical
  plus strict MoveIt sequence gates.
- This is a generalized yaw search driven by the current geometry; it adds no
  object label, carton coordinate, or fixed grasp pose.

### 02:36 PDT full in-plane yaw planning result and replay correction

Full OBB yaw search:

- Sampled the support-plane jaw direction every configured `15 deg` over
  `[0,180)`.
- The live OBB plus `0.5 mm` opening-fit clearance per side admitted only the
  `0 deg` and `90 deg` principal directions. Their required openings were
  `47.756 mm` and `40.669 mm`; every diagonal sample required more than the
  physical `50 mm` opening and was rejected before planning.
- The two width-valid directions, two wrist symmetries, and lower-tilt
  polarities produced 36 candidates.
- All `36/36` passed the generalized analytical gates. Strict MoveIt accepted
  `0/36`: 34 failed at pregrasp pose planning and two reached a virtual
  approach but failed the linear grasp segment at fractions `0.529` and
  `0.471`, below `0.980`.
- No motion or task start was requested.

Discovered replay-method problem:

- Both lower-tilt runs planned the first pregrasp directly from the robot's
  current physical state, which is still the previous collision-time
  near-field posture.
- A real new task does not follow that transition. It first moves to the
  explicitly allowed fixed `100 mm` observation pose, then freezes a new
  close-range estimate and plans the contact sequence from that settled
  observation state.
- Therefore `0/36` is valid evidence for the direct
  collision-posture-to-pregrasp transition, but it is not yet evidence that
  the complete task chain is unreachable.

Correction:

- Re-run the same candidate family through the strict planner with the current
  plan's fixed-100-mm observation pose as the first virtual stage, followed by
  pregrasp, approach, grasp, and lift. All stages remain planning-only.
- This preserves the allowed fixed observation point while every contact-stage
  pose and distance remains derived from the live geometry and measured
  uncertainty.

### 02:39 PDT observation-seeded strict replay result

- Added the immutable plan's fixed-100-mm observation pose as the first
  virtual stage of every strict sequence:
  `current -> observation -> pregrasp -> approach -> grasp -> lift`.
- The observation stage planned successfully for all 36 trials, with virtual
  path cost about `1.99 rad` and maximum joint delta about `1.50 rad`.
- The complete sequence still accepted `0/36`: 34 then failed at pregrasp,
  while two maximum-tilt variants failed at grasp with Cartesian fractions
  `0.529` and `0.471`.
- This rules out the current collision-time start state as the cause of the
  zero-reachability result. No real movement occurred.

New generalized path finding:

- The current sequence retreats the entire `76--78 mm` pregrasp distance along
  the tilted insertion axis. This couples a long collision-clearance distance
  to a large horizontal workspace displacement.
- The same full pregrasp distance is also used to cap tilt by a fixed lateral
  envelope. That excludes the approximately `20 deg` family before checking
  whether a safer centered pregrasp and shorter final insertion can satisfy
  both geometry and workspace constraints.
- Planning-only hypothesis to test: place pregrasp above the live contact
  center along the measured support-plane normal, while keeping only the final
  approach-to-grasp segment collinear with the candidate insertion axis.
- Under this formulation, the tilt limit is computed from the live approach
  distance and lateral bound; the longer pregrasp remains a normal-clearance
  waypoint. The complete pregrasp-to-approach diagonal and final insertion
  must still pass continuous analytical swept-volume gates and strict MoveIt.
- This contains no object label, target coordinate, or fixed contact pose. It
  uses the live support normal, OBB/depth uncertainty, gripper CAD, measured
  execution error, and object-independent physical limits.

### 02:43 PDT support-normal pregrasp planning-only proof

Prototype formulation:

- Kept the live adaptive approach `48.545 mm`, normal pregrasp clearance
  `78.197 mm`, lift `59.652 mm`, measured endpoint error `22.852 mm`, and
  physical lateral limit `20 mm`.
- Placed the pregrasp along the live support normal instead of retreating the
  full pregrasp distance along the tilted insertion vector.
- Kept approach-to-grasp exactly collinear with each candidate's insertion
  axis.
- Computed the tilt interval from the live approach distance:
  `asin(20/48.545) = 24.330 deg`, then sampled
  `0/6.082/12.165/18.247/24.330 deg` with both polarities and both wrist
  symmetries.
- Included the fixed-100-mm observation stage before every candidate.

Result:

- All `36/36` candidates passed the complete analytical static and swept
  gripper/OBB/support-plane gates, including the diagonal
  pregrasp-to-approach segment.
- Strict MoveIt accepted `2/36`, both at the dynamically derived
  `24.330 deg` boundary and the OBB narrow-axis jaw direction.
- Best candidate:
  jaw variant `1`, positive tilt polarity, required opening `40.669 mm`,
  grasp about `(-0.158101,-0.485785,0.064168) m`,
  approach about `(-0.157061,-0.462935,0.106986) m`,
  centered pregrasp about `(-0.160656,-0.480399,0.142137) m`,
  joint path cost `3.580 rad`, maximum joint delta `1.502 rad`.
  It satisfies the existing `2.15 rad` maximum-delta contract.
- The other reachable wrist symmetry had maximum joint delta `3.169 rad` and
  must remain rejected by the existing motion-cost limit.
- No physical trajectory was executed.

Conclusion and implementation decision:

- The collision-safe, reachable structure is not a target-specific pose. It
  is a general decomposition of clearance and contact motion:
  support-normal pregrasp for centered obstacle clearance, followed by a live
  insertion-axis approach and grasp.
- The lateral-sweep quantity must represent the final insertion's lateral
  component, `approach_offset * sin(tilt)`, because the longer pregrasp is now
  a support-normal clearance waypoint.
- The old collision candidate at `39.3 deg` remains inadmissible:
  its final-approach lateral component exceeds the same physical bound.
- Implement this decomposition in the common sequence builder and adaptive
  profile generator, then require compilation, adaptive/sequence/remote/task
  regressions, strict MoveIt replay, and MuJoCo before any real task motion.

### 02:47 PDT support-normal sequence implementation

Implementation:

- Extended the common sequence builder with an optional independent
  `pregrasp_direction_base`. Existing callers retain their original behavior
  when it is absent.
- Production contact sequence generation now sets the pregrasp direction to
  the negative live support normal, which places the actual pregrasp along the
  positive support normal. Approach remains on the candidate insertion axis.
- Adaptive tilt generation now applies the lateral limit to
  `approach_offset * sin(tilt)` and keeps the full independently derived
  normal pregrasp clearance for every tilt.
- Runtime tilt lookup uses the same final-approach definition.
- The hard lateral gate and soft-score feature now measure
  `approach -> grasp`, matching their existing "final approach" name, instead
  of measuring the entire pregrasp-to-grasp clearance path.
- Stage audits now explicitly record
  `pregrasp_direction_mode=support_normal` and
  `lateral_sweep_reference=final_approach`.
- Replaced remaining analytical failure text referring to a "carton OBB" with
  the generic "target OBB"; no geometry logic changed in that wording edit.
- Added sequence tests for independent support-normal pregrasp and invalid
  direction rejection, an adaptive-profile regression for final-approach
  lateral sweep, and updated the hard-gate regression to supply an explicit
  approach pose.

Implementation issue:

- The first multi-file patch attempt contained one incorrect path missing the
  package directory. The patch tool rejected the entire batch before changing
  any file. The path was corrected and the complete patch then applied
  successfully; there was no partial source state.

Initial verification:

- Common sequence plus adaptive-profile tests pass `22/22`.
- The updated final-approach lateral hard-gate regression passes `1/1`.
- Python compilation and `git diff --check` pass.
- Full remote-node regression passes `154/154`.
- Full streaming regression passes `180/180`.
- Full task-sequence regression passes `114/114` with only five existing
  `rospy.logwarn` deprecation warnings.
- Full gripper and tabletop geometry regression passes `109/109`.

### 02:50 PDT remote planner hot restart

- Cleanly interrupted only the old, idle `remote_grasp6d_node` supervised
  session `48787`.
- Started the updated node in supervised session `48031`.
- The node reports the GraspNet baseline loaded and protocol 3 server online
  at the configured WSL endpoint.
- Did not restart or command the driver, trajectory controller, MoveIt,
  hand-eye transform publisher, task node, or joint-enable path.
- No task start, robot trajectory, stop, or disable command was issued.

### 02:51–02:53 PDT updated-node fresh planning stream

- Started only continuous remote inference and supervised its terminal output;
  no task start or motion request was sent.
- Request 3 reached `PREVIEW_READY`; five of 24 strict MoveIt shortlist
  candidates were reachable, four were rejected by the joint-delta limit, and
  one far-field observation plan was promoted.
- Stopped continuous inference. Later queued requests were dropped as stale or
  failed acceptance and did not replace the promoted plan.
- Fresh immutable plan `96005f458ee360e6b2562ed1` has live center about
  `(-0.157447,-0.471665,0.059963) m`, OBB
  `(0.045706,0.040104,0.018739) m`, depth MAD `3.5 mm`, support normal
  `(-0.027628,0.079684,0.996437)`, support inlier ratio `0.596`, and required
  opening `42.702 mm`.
- The far-field observation pose is about
  `(-0.151159,-0.417144,0.137786) m`. This is planning evidence only; the arm
  remained stationary.
- Updated audits in the live node explicitly report
  `pregrasp_direction_mode=support_normal` and
  `lateral_sweep_reference=final_approach`, confirming the new code is loaded.

### 02:55 PDT updated-code strict replay on fresh live geometry

- Replayed the newly loaded common sequence/adaptive-profile code against
  fresh plan `96005f458ee360e6b2562ed1`, conservatively using the previous
  measured `22.852 mm` endpoint error.
- Live profiles were tilt
  `0/6.076/12.153/18.229/24.305 deg`, approach `48.591 mm`,
  support-normal pregrasp `78.443 mm`, lift `59.852 mm`, and maximum final
  approach lateral sweep `20 mm`.
- The two live OBB width-valid jaw axes, two wrist symmetries, and tilt
  polarities produced 36 candidates. All `36/36` passed analytical static and
  swept-volume gates.
- Strict full-sequence MoveIt accepted two. One wrist solution had maximum
  joint delta `4.060 rad` and remains rejected by the `2.15 rad` gate.
- The accepted best solution had:
  tilt `24.305 deg`, OBB narrow-axis jaw direction, positive polarity,
  required opening `41.104 mm`,
  support-normal pregrasp about
  `(-0.163439,-0.477359,0.142326) m`,
  approach about `(-0.157112,-0.460869,0.106901) m`,
  grasp about `(-0.161272,-0.483609,0.064162) m`,
  joint path cost `4.365 rad`, and maximum joint delta `1.088 rad`.
- No robot command was issued. Next gate is a task-equivalent MuJoCo request
  for this exact planning candidate.

### 02:57 PDT digital-twin generalization blocker discovered

- While preparing the task-equivalent MuJoCo payload, found that the runtime
  `/mujoco_digital_twin/object_model` still contains
  `label=carton`, fixed mass `0.08 kg`, and one fixed friction tuple
  `[1.2,0.08,0.02]`.
- The live OBB size and pose are already carried by the plan, but the fixed
  carton label/material parameters are not valid evidence for arbitrary
  graspable targets.
- This does not invalidate the analytical or strict MoveIt results, but it
  blocks treating a simulation pass under those parameters as a generalized
  scientific execution gate.
- Do not execute the task under a falsely generalized simulation claim.
  Inspect the existing payload/server protocol for generic live-OBB and
  conservative uncertainty support, then implement the smallest fail-closed
  correction before running the exact candidate simulation.

### 03:02 PDT generalized dynamics and strict lift-evidence correction

Generalized dynamics payload:

- Removed the runtime carton label and fixed `0.08 kg`/single-friction model.
- A MuJoCo request now derives conservative mass from every live OBB volume:
  `clamp(volume * density_upper_bound, mass_floor,
  operational_mass_ceiling)`.
- Production envelope is explicit and category-independent:
  density upper bound `20000 kg/m^3`, mass floor `0.02 kg`, operational mass
  ceiling `0.50 kg`, and friction lower bound
  `[0.10,0.005,0.0001]`.
- The payload records the live volume, unclamped mass, all bounds, and the
  derived mass. Legacy fixed `mass_kg` or `friction` configuration is rejected
  as category-specific.

Strict dynamic lift:

- Removed the MuJoCo server branch that accepted a carried lift when dynamic
  object displacement was below the `15 mm` minimum.
- Lift success now requires actual object displacement along the support
  normal to be at least `15 mm`.
- Two-sided contact accounting is structured and fail-closed:
  object lift, commanded lift, minimum lift, two-sided samples, lost-contact
  samples, total samples, maximum loss streak, and grace count.
- Across the 40 lift samples, both total lost-contact samples and maximum loss
  streak must remain within the configured three-sample grace.
- The ROS client requires `lift_evidence.contract_version=1`, verifies sample
  accounting and numeric thresholds, and rejects a nominal success from an
  older server that lacks this evidence.
- The durable task audit now includes the complete structured lift evidence.
- Server geometry identifiers and bounds were renamed from carton-specific to
  generic target-OBB terminology.

Verification:

- MuJoCo client/payload tests pass `80/80`.
- MuJoCo server protocol tests pass `115/115` with one optional real-MuJoCo
  smoke test skipped.
- Full task-sequence tests still pass `114/114` with the five existing
  `rospy.logwarn` deprecation warnings.
- Python compilation and `git diff --check` pass.

Runtime deployment fact:

- The WSL health response currently lacks
  `lift_evidence_contract_version=1`, so the running WSL process is still the
  older server code. SSH port 22 is not available on that WSL endpoint, so it
  cannot be updated/restarted from this ROS host.
- Until the operator updates and restarts that WSL server, any nominal
  simulation success is intentionally rejected by the ROS client and no
  physical task may proceed.

### 03:04 PDT first exact-candidate simulation diagnostic

- Built the exact best candidate with the new live-OBB conservative dynamics:
  live volume `3.4349e-5 m^3`, unclamped upper-envelope mass `0.6870 kg`,
  operationally clamped mass `0.500 kg`, and friction lower bound
  `[0.10,0.005,0.0001]`.
- The first request did not enter IK. WSL rejected the temporary diagnostic
  stamp as `PLAN_STALE` because it appeared `1.311 s` in the future relative
  to the WSL clock.
- This is a diagnostic timestamp/clock-offset failure, not a candidate
  collision or reachability result. No motion occurred.
- Re-run only this temporary planning diagnostic with its request stamp
  shifted back `1.5 s`; the trajectory, live geometry, joint state, mass, and
  friction remain unchanged.

### 02:10 PDT diagnostic-shell environment failure

- The first attempt to perform the `-1.5 s` timestamp replay exited before
  reading or publishing any ROS topic because the standalone shell had not
  sourced this worktree's `devel/setup.bash`.
- Exact failure: `ModuleNotFoundError: No module named
  'alicia_flexible_grasp_supervisor'` while importing the generated ROS
  message package.
- This is only a diagnostic-shell environment error. No MuJoCo request and no
  robot motion, stop, torque-disable, or joint-disable command occurred.
- Resolution: source the same worktree ROS overlay used by the live nodes,
  then rerun the otherwise unchanged no-motion replay.

### 02:11 PDT exact-candidate old-WSL result and fail-closed verdict

- Replayed the exact best updated-path candidate with no ROS publication and
  no robot motion. The temporary request stamp was shifted back `1.5 s`; all
  geometry, trajectory, live joint state, required width, and conservative
  dynamics remained unchanged.
- Replay plan ID:
  `codex-planning-only-e0b470fdd501`.
- WSL completed the request in `0.255 s`. Its old server reported all safety
  booleans true and score `100.0`, but its own diagnosis proves that this was
  a false lift acceptance:
  - dynamic object lift: `0.000 m`;
  - commanded lift: `0.060 m`;
  - two-sided contact present for only `5/40` lift samples;
  - two-sided contact missing for `35/40` samples;
  - maximum consecutive loss streak: `35`, versus grace `3`.
- The old server explicitly used its obsolete
  `carried-object lift accepted after preloaded two-sided contact` branch to
  return `lift_success=true`.
- The updated local ROS trust-boundary validator rejected that nominal
  100-point response as intended:
  `ok=false`, `code=WSL_UNAVAILABLE`, reason
  `MuJoCo response lacks structured strict dynamic-lift evidence`.
- The candidate's old-server IK did converge at all four poses, and its
  collision/contact checks reported true. That does **not** authorize physical
  execution because strict dynamic-object lift failed and the running WSL
  server has not yet been updated.
- Live-OBB dynamics sent in this replay were derived from measured geometry,
  not an object label or fixed carton values:
  volume `3.4349128e-5 m^3`, density-upper mass `0.6869826 kg`, clamped
  operational mass `0.500 kg`, and friction lower bound
  `[0.10,0.005,0.0001]`.
- No motion, stop, arm-disable, torque-disable, controller-stop, or joint-
  disable command was issued.
- Resolution required before any physical continuation: deploy the updated
  strict-lift server code to the WSL process, restart only that WSL
  inference/simulation service, then require its health and exact candidate
  response to include lift-evidence contract version `1`.

### 02:11 PDT WSL strict-server transfer prepared

- Started a temporary read-only HTTP file endpoint on the ROS host at
  `http://192.168.26.129:8765/`, rooted only at the repository `tools`
  directory, so the operator can transfer the updated server into WSL without
  manually copying source.
- Updated server artifact:
  `mujoco_digital_twin_server.py`, `91080` bytes, SHA-256
  `973f932bb41d8f01ca10a66fa24a763565d41bd97bfb808649c8e1ef7c2fbcbc`.
- Verified that downloading the artifact through the temporary endpoint
  produces the identical SHA-256.
- Only the WSL GraspNet+MuJoCo HTTP process must be replaced/restarted. The ROS
  driver, controller, MoveIt, motion gateway, hand-eye TF, joint enable, and
  task authority remain untouched.

### 02:16 PDT updated WSL strict server confirmed live

- The operator reported that WSL was restarted. ROS-side `/health` immediately
  confirmed the combined service is healthy:
  GraspNet protocol `3`, CUDA `11.8`, MuJoCo `3.2.3`, and no missing
  dependencies.
- New strict fields are present and exact:
  `lift_evidence_contract_version=1`,
  `strict_dynamic_object_lift_required=true`,
  `min_lift_success_m=0.015`, `grip_preload_m=0.003`,
  `preload_settle_steps=8`, and
  `lift_contact_loss_grace_steps=3`.
- The temporary ROS-host transfer server recorded the WSL download, and its
  local download hash had already matched the approved source artifact.
- The WSL server's snapshot-age contract is `2.0 s`; exact-candidate replay
  will again use the measured `-1.5 s` clock compensation without changing
  any trajectory or target geometry.
- No physical motion or robot-control command was issued.

### 02:17 PDT strict exact-candidate replay

- Replayed the same updated-path best candidate against the newly deployed
  strict WSL server. Plan ID:
  `codex-strict-replay-40134f3ba362`; response time `0.264 s`.
- All four MuJoCo IK stages converged and the trajectory collision check
  passed. Simultaneous two-sided contact was first found at a `0.0386 m`
  finger gap; the strict server applied the configured generic `0.003 m`
  preload and initially retained two-sided contact.
- Strict dynamic lift then correctly failed:
  - commanded lift `0.059639 m`;
  - measured dynamic-object lift `0.00000967 m`;
  - required object lift `0.015 m`;
  - two-sided lift samples `5/40`;
  - lost-contact samples `35/40`;
  - maximum loss streak `35`, versus grace `3`.
- Raw result:
  `simulation_ok=false`, `contact_success=false`, `lift_success=false`,
  score `55.0`, failure `MUJOCO_CONTACT_FAILED`.
- The updated ROS validator preserved the exact strict failure and did not
  authorize motion.
- Interpretation: the path decomposition fixed the observed approach-edge
  collision/reachability problem, but this candidate still does not establish
  a dynamically supportable grasp under the category-independent conservative
  live-OBB model. The next correction must improve candidate/contact mechanics;
  the strict lift/contact thresholds will not be weakened.
- No physical motion or robot-control command was issued.

### 02:18 PDT dynamics sensitivity diagnosis

- Ran a planning-only sensitivity matrix for the same geometry and trajectory:
  masses `[0.02,0.05,0.08,0.10,0.20,0.50] kg` crossed with sliding-friction
  coefficients `[0.10,0.30,0.60,1.20]`.
- All `24/24` combinations failed almost identically with only `4-5/40`
  two-sided lift samples and essentially zero object lift. Even `0.02 kg` at
  friction `1.20` failed. Therefore this failure is not presently
  mass/friction dominated and changing object-specific dynamics would only
  hide the defect.
- Code inspection found the defect: after detecting contact, the server's
  `_apply_gripper_inner_gap` writes both the finger joint positions and their
  actuator control targets to the same value on every sample. That creates
  zero sustained servo error/normal force. The lift loop also advances a
  roughly `60 mm` lift in only `40` single MuJoCo steps, which is far faster
  than the real strict-execution motion and causes kinematic teleportation.
- Resolution direction is category-independent:
  command finger actuator targets without overwriting their measured joint
  positions after contact, and integrate the lift over physical simulation
  time derived from commanded distance and the robot execution-speed policy.
  The object mass/friction envelope and strict lift threshold remain unchanged
  during this correction.
- No physical motion or robot-control command was issued.

### 02:19 PDT sustained-contact/time integration implementation

- Updated the generic MuJoCo backend:
  - added a contact-phase gripper command that changes actuator targets while
    preserving simulated finger joint positions and velocities;
  - retained kinematic finger positioning only for the pre-contact closure
    search;
  - changed preload and lift to the sustained actuator command;
  - derived lift sample count from commanded lift distance, a uniform
    `0.10 m/s` strict simulation speed, and the actual MuJoCo model timestep;
  - bounded the numerical integration to `40..4000` samples;
  - added the lift-speed policy to WSL health and the launch script.
- Added protocol tests proving that contact-phase commands do not teleport
  finger state and that a `60 mm` lift at `0.10 m/s` with a `2 ms` timestep
  uses `301` samples.
- Python compilation, shell syntax validation, and `git diff --check` passed.
- The first protocol-test invocation failed during collection because its
  standalone shell did not source the worktree overlay:
  `ModuleNotFoundError: alicia_flexible_grasp`. No test case ran. Resolution
  is to source `devel/setup.bash` and rerun the same suite.
- No physical motion or robot-control command was issued.

### 02:20 PDT mechanical-simulation correction verified

- With the worktree overlay sourced, MuJoCo server protocol tests pass
  `116/116` with one optional real-MuJoCo local smoke skipped.
- MuJoCo ROS client tests pass `80/80`.
- Full grasp task sequence tests pass `114/114`; only the five existing
  `rospy.logwarn` deprecation warnings remain.
- Updated WSL artifact is `93469` bytes with SHA-256
  `4778369aaee1a974d409f3574ac3e45a4aa4734c02da1f7232214516344e129d`.
  The temporary HTTP endpoint returns the identical hash.
- The running WSL process still contains the preceding source image until the
  operator downloads this artifact and restarts only the combined WSL
  inference/simulation process.

### 03:02 PDT corrected mechanical simulator confirmed live

- The operator completed the second WSL restart. ROS-side health confirms the
  new `lift_speed_m_s=0.1` field together with strict lift-evidence contract
  version `1`, MuJoCo `3.2.3`, GraspNet protocol `3`, and no missing
  dependencies.
- The temporary transfer endpoint recorded the second WSL download.
- The strict exact-candidate replay will now verify that lift integration uses
  physical-time sampling and sustained gripper actuator targets.
- No physical motion or robot-control command was issued.

### 03:03 PDT corrected mechanical replay result

- Exact candidate replay plan:
  `codex-corrected-replay-95651e675527`; WSL response time `0.303 s`.
- Physical-time integration is active: the `59.639 mm` commanded lift now uses
  `300` MuJoCo samples instead of the old fixed `40`.
- Four-stage IK and collision checking still pass, and closure still finds
  simultaneous contact at a `38.6 mm` gap.
- Under the current worst-case live-OBB dynamics envelope, the candidate still
  fails strictly: only `3/300` two-sided lift samples, `297/300` lost samples,
  maximum loss streak `270`, and dynamic object lift only `0.017 mm`.
- Result remains fail-closed:
  `MUJOCO_CONTACT_FAILED`, score `55.0`; no motion authority.
- A second dynamics-sensitivity matrix is required now that actuator state and
  physical time are corrected. It will determine whether contact behavior
  varies with physical dynamics or whether another geometry/actuator defect
  remains.

### 03:04 PDT corrected-time sensitivity result and command mismatch

- Repeated the same `24` mass/friction combinations after sustained actuator
  control and physical-time integration.
- All combinations still failed; `0.02 kg` produced only `8/300` two-sided
  samples and every heavier case produced roughly `2-4/300`. Friction from
  `0.10` through `1.20` had almost no effect because contact normal force was
  lost before friction could support the object.
- The remaining deterministic mismatch is between simulated and real generic
  gripper commands:
  - real task configuration commands the 50 mm gripper to its hardware close
    target `simple_close_position_m=0.0`;
  - WSL currently commands only `3 mm` beyond the first detected contact.
- A `3 mm` contact-relative preload is neither the configured real close
  command nor a live object-derived value. It under-models sustained normal
  force for every target.
- Resolution: bind MuJoCo to the same hardware close target `0.0 m`. This is a
  fixed gripper mechanism contract, analogous to its fixed `0.050 m` maximum
  opening, not a target-specific carton dimension, pose, or class rule.
- Strict dynamic lift/contact evidence and live target geometry remain
  mandatory. No physical motion or robot-control command was issued.

### 03:05 PDT request-bound close contract and lift-origin correction

- Added the real generic gripper command to every schema-v3 MuJoCo request:
  close target `0.0 m` and close-settle interval `0.8 s`. Both values describe
  the existing hardware command policy and are independent of object identity
  or dimensions.
- The WSL server now validates those request fields, commands the simulated
  actuator toward the request-bound target, derives settle samples from the
  request duration and model timestep, and records the target/sample count in
  structured lift evidence.
- MuJoCo client tests pass `80/80`.
- The first server protocol run found one intentional boundary-test failure:
  with the longer close-settle interval, the server's old lift origin counted
  object motion during closing as lift. That could create false-positive lift
  evidence.
- Corrected the measurement boundary: record the dynamic-object lift origin
  only after close settling is complete, so only subsequent lift-stage
  displacement can satisfy the `15 mm` threshold.
- This is a fail-closed measurement correction; no threshold was relaxed and
  no physical motion or robot-control command was issued.

### 03:06 PDT request-bound close implementation verified

- Server protocol tests pass `116/116` with one optional local real-MuJoCo
  smoke skipped.
- Full task sequence tests pass `114/114` with only the five existing
  deprecation warnings.
- Python compilation and `git diff --check` pass.
- Updated WSL server artifact is `95060` bytes, SHA-256
  `2e0a128559402e5ddd58fcf75d46f089fbb108023ff902e6c91b9156e4b7f54c`;
  the temporary transfer endpoint returns the identical hash.
- The WSL process must be reloaded once more before the request-bound `0.0 m`,
  `0.8 s` generic close contract can be exercised against real MuJoCo.

### 03:09 PDT request-bound close server confirmed live

- After the operator's third WSL restart, health confirms
  `request_bound_gripper_close_contract=true`, physical-time lift speed
  `0.1 m/s`, strict dynamic lift required, and lift-evidence contract `1`.
- GraspNet protocol `3`, MuJoCo `3.2.3`, CUDA, checkpoint, and mesh-backed
  model are healthy with no missing dependencies.
- The next exact-candidate request is built by the updated local client and
  therefore carries the hardware close target and settle interval explicitly.
- No physical motion or robot-control command was issued.

### 03:10 PDT request-bound close exact replay

- Replay plan `codex-bound-close-c1c1713f7526` carried the exact hardware
  contract: target inner gap `0.0 m`, close settle `0.8 s`.
- WSL confirmed `400` close-settle samples and `300` physical-time lift
  samples. Closure found first two-sided contact at `38.6 mm`.
- Full-close actuation materially improved lift contact from the preceding
  `3/300` samples to `31/300`, proving the request-bound actuator correction is
  active.
- The current worst-case `0.5 kg`, friction `0.10` envelope still failed:
  `269/300` lost-contact samples, `0.045 mm` object lift, score `55`,
  `MUJOCO_CONTACT_FAILED`.
- No motion authority was created. A final corrected mechanics sensitivity
  matrix will determine whether physically lighter/higher-friction hypotheses
  now produce real dynamic lift rather than the earlier invariant failure.

### 03:11 PDT request-bound close dynamics matrix

- Tested masses `[0.01,0.02,0.05,0.08,0.10,0.20,0.50] kg` crossed with
  sliding friction `[0.10,0.30,0.60,1.20]`, always using the same live OBB,
  candidate path, `0.0 m` close target, and `0.8 s` settle interval.
- Mass now affects dynamic response: measured lift decreased from about
  `0.819 mm` at `0.01 kg` to `0.045 mm` at `0.50 kg`. This confirms object
  inertia, sustained finger actuation, and physical-time stepping are active.
- No combination reached the strict `15 mm` lift requirement. All lost
  two-sided contact after roughly `26-32/300` samples.
- Friction changes had little effect. The likely remaining failure is contact
  normal-force/geometry behavior rather than a defensible object-dynamics
  choice.
- Do not choose a favorable mass/friction tuple: doing so would be
  target-specific tuning and would not resolve the common contact loss.
- Next diagnostic must record settled finger joint positions, commanded
  targets, actuator force, bilateral normal contact force, and the first loss
  state. These are generic simulated measurements and can distinguish force-
  direction/magnitude error from a candidate contact-surface error.
- No physical motion or robot-control command was issued.

### 03:12 PDT generic gripper-force telemetry implemented

- Added structured, non-authorizing MuJoCo telemetry at two physical events:
  after request-bound close settling, and at the first lift sample that loses
  simultaneous two-sided contact.
- Each state records measured finger joint positions, measured inner gap,
  actuator control targets, actuator forces, and left/right object normal
  contact forces. The first-loss state also records sample index and trajectory
  fraction.
- The values are appended to strict lift evidence and durable task audit; they
  do not change scores, thresholds, collision rules, or lift/contact success.
- Server protocol tests pass `116/116` with one optional real-MuJoCo local
  smoke skipped. Python compilation and `git diff --check` pass.
- Updated WSL artifact is `100633` bytes with SHA-256
  `610f96bb798d6d33bc6ede28fe9d101db4a604a3cc96b1562a3fd6a6e959690a`;
  HTTP transfer reproduces the same hash.
- No physical motion or robot-control command was issued.

### 03:16 PDT WSL multiline curl paste failure

- The operator reported `curl: (6) Could not resolve host:` while pasting the
  multiline transfer command. This indicates a shell line-continuation paste
  problem, not a server-side DNS dependency.
- The ROS-host endpoint remains online and returns the approved SHA-256.
- Its access log also recorded WSL-address downloads, so a partial/complete
  `/tmp` file may already exist; it must not be installed without the exact
  SHA check.
- Resolution: repeat the download as one physical shell line, then validate
  `/tmp/mujoco_digital_twin_server.py` against the approved hash before
  installation.
- No physical motion or robot-control command was issued.

### 03:17 PDT WSL combined service intentionally offline for update

- The operator confirmed the WSL GraspNet+MuJoCo service has been closed
  before retrying the telemetry artifact transfer.
- This does not stop or disable the ROS robot driver, controller, MoveIt,
  motion gateway, hand-eye TF, or joint enable.
- The ROS-host artifact endpoint remains independent of WSL and available for
  the one-line download and SHA verification.

### 03:18 PDT WSL telemetry artifact hash confirmed

- The operator reported exact SHA-256 match for
  `/tmp/mujoco_digital_twin_server.py`:
  `610f96bb798d6d33bc6ede28fe9d101db4a604a3cc96b1562a3fd6a6e959690a`.
- The downloaded telemetry server is therefore byte-identical to the locally
  tested artifact and is approved for installation into the WSL checkout.
- No physical motion or robot-control command was issued.

### 03:19 PDT force-telemetry WSL server live

- The operator started the hash-verified telemetry server. ROS-side health
  confirms the combined protocol-3 GraspNet and MuJoCo service is healthy,
  request-bound close is enabled, strict dynamic lift is required, and no
  dependency is missing.
- The next replay will use the same live geometry and trajectory while reading
  only the newly added force/contact evidence.
- No physical motion or robot-control command was issued.

### 03:20 PDT force replay blocked by invalid current rich-plan message

- The first telemetry replay exited in local payload construction before any
  WSL request because the currently received `/grasp_6d/plan_enriched`
  message had `object_geometry.valid=false`.
- Exact local rejection:
  `ValueError: plan.object_geometry.valid must be true`.
- This is likely an invalidation/tombstone published while WSL was offline,
  not a force result for the frozen plan. Inspect the current topic and remote
  terminal, then build the no-motion diagnostic from the previously logged,
  immutable live geometry if the latched topic is only a tombstone.
- No simulation request, physical motion, or robot-control command occurred.

### 03:20 PDT current topic confirmed as target-loss tombstone

- `/grasp_6d/plan_enriched` sequence `23` is an explicit invalid tombstone:
  no poses, empty plan ID, invalid geometry, diagnostic
  `TARGET_LOST: target object is not detected`.
- The task node rejected the tombstone and did not replace its execution
  authority with a valid plan.
- This tombstone cannot be used for any task or simulation authorization.
- Force diagnosis will instead reconstruct only a temporary planning request
  from the immutable geometry, support plane, candidate poses, and lineage
  already recorded for plan `96005f458ee360e6b2562ed1`. It receives a new
  diagnostic-only plan ID and current joint state; it is not published to ROS
  and cannot command motion.

### 03:21 PDT force telemetry isolates unilateral lift-path separation

- Reconstructed planning-only frozen request:
  `codex-frozen-force-75140974c513`. It was never published to ROS.
- After `400` close-settle samples:
  - measured inner gap `39.566 mm`;
  - left/right actuator force `-1.093/+1.083 N`;
  - left/right object normal force `1.094/1.082 N`.
- This proves the generic close command produces balanced, sustained,
  bilateral normal force before lift.
- First two-sided contact loss occurs at lift sample `31/300`, trajectory
  fraction `0.10368`:
  - left/right actuator force remains `-1.013/+1.036 N`;
  - left normal object force rises to `1.458 N`;
  - right normal object force becomes exactly `0 N`;
  - measured gap remains `39.237 mm`.
- Therefore contact fails by unilateral geometric separation during the lift,
  not by loss of actuator command or lack of initial bilateral force.
- Current MuJoCo lift uses linear interpolation between independently solved
  grasp/lift joint endpoints. Although endpoint orientations are close, this
  does not guarantee the straight, constant-orientation Cartesian lift
  required to preserve a pinch.
- Next action: inspect the real MoveIt contact-lift execution contract. If it
  is also unconstrained pose-to-pose joint planning, replace both real
  execution and MuJoCo with the same support-normal Cartesian lift contract;
  do not compensate with target-specific mass/friction values.
- Strict gate remains failed at score `55`; no physical motion or robot-
  control command was issued.

### 03:22 PDT operator disables electronic-skin feedback for current 6D task

- Operator requirement: the current 6D grasp must not use electronic-skin
  force feedback because the skin is not calibrated.
- Confirmed the execution code already selects the position-only branch when
  `/gripper/use_compliant_close=false`: it does not wait for or call
  `/supervisor/compliant_close`; it publishes the generic fixed gripper
  position target and waits the configured `0.8 s`.
- Corrected the contradictory residual configuration by changing
  `/gripper/require_tactile_feedback` from `true` to `false` and documenting
  that tactile state is not an execution or success condition.
- MuJoCo contact-force telemetry is internal simulated physics used to
  diagnose one-sided virtual contact loss. It is not electronic-skin data and
  does not enable real force feedback.
- No physical motion or robot-control command was issued.

### 03:23 PDT electronic-skin disable active in live ROS parameters

- Updated only the current ROS parameter server:
  `/gripper/use_compliant_close=false` and
  `/gripper/require_tactile_feedback=false`.
- Read-back confirmed both values are boolean false.
- Full grasp task sequence tests pass `114/114`; the five existing
  `rospy.logwarn` deprecation warnings are unchanged.
- `git diff --check` passes.
- The current 6D task will use fixed position close only. No electronic-skin
  force signal will be awaited, consumed, or used as success authority.
- No physical motion, stop, torque-disable, or joint-disable command was
  issued.

### 03:24 PDT first Cartesian-simulator patch rejected atomically

- Confirmed the real 6D contact stages already use
  `/supervisor/move_to_pose_linear`; the production path is Cartesian with
  collision checking and a minimum completion fraction.
- Prepared a MuJoCo correction to replace grasp-to-lift joint interpolation
  with the same straight Cartesian translation and interpolated orientation.
- The first multi-hunk patch referenced an outdated `LiftResult` field-order
  context. The patch tool rejected the complete patch before changing any
  source file.
- Resolution: read the current exact dataclass/method contexts and apply the
  same correction in bounded hunks. Electronic-skin feedback remains disabled
  and unrelated to this simulated path correction.

### 03:25 PDT Cartesian simulator initial test result

- Applied the correction in bounded hunks:
  configurable strict waypoint IK tolerances, Cartesian position
  interpolation, quaternion shortest-path interpolation, and fail-closed
  Cartesian waypoint IK reporting.
- Python compilation and `git diff --check` pass.
- Initial server protocol run: `113` passed, `3` failed, one optional smoke
  skipped.
- All three failures are old direct-lift test doubles that provide only an
  object position; the corrected lift contract now also requires left/right
  finger body positions and the tool orientation matrix. Production model
  metadata already contains these values.
- Resolution: update only the test fixtures to represent the Cartesian state
  and stub strict waypoint IK, then add rotation-interpolation tests.
- No electronic-skin input, physical motion, or robot-control command was
  used.

### 03:26 PDT Cartesian lift simulator verified

- Updated direct-lift test fixtures with the real Cartesian state contract and
  added quaternion shortest-path interpolation endpoint/midpoint coverage.
- Server protocol tests pass `117/117` with one optional local real-MuJoCo
  smoke skipped.
- Full grasp task tests pass `114/114` with only the five existing deprecation
  warnings.
- Python compilation and `git diff --check` pass.
- WSL health will expose:
  `lift_path_contract=cartesian_pose_interpolation`,
  position waypoint tolerance `0.0002 m`, and orientation waypoint tolerance
  `0.005 rad`.
- Updated server artifact is `107235` bytes, SHA-256
  `75d90c8a7cb14c3bede176a147aea196ced726f45d7509a9f138ee759c94b1bd`;
  the HTTP transfer endpoint returns the identical hash.
- Live ROS read-back still confirms electronic-skin paths disabled:
  `use_compliant_close=false`,
  `require_tactile_feedback=false`.
- No physical motion or robot-control command was issued.

### 03:30 PDT Cartesian WSL simulator confirmed live

- WSL health now reports
  `lift_path_contract=cartesian_pose_interpolation`,
  position waypoint tolerance `0.0002 m`, and orientation waypoint tolerance
  `0.005 rad`.
- Combined GraspNet protocol `3`, MuJoCo `3.2.3`, mesh model, strict
  request-bound close, and dynamic-lift evidence are healthy.
- Live ROS read-back again confirms electronic-skin feedback disabled:
  `use_compliant_close=false` and
  `require_tactile_feedback=false`.
- The next replay is planning-only from the previously frozen geometry because
  the current topic remains a target-loss tombstone.
- No physical motion or robot-control command was issued.

### 03:31 PDT Cartesian replay rules out joint-interpolation cause

- Planning-only replay:
  `codex-frozen-cartesian-51aabd8ad8`; response `0.767 s`.
- WSL diagnosis explicitly confirms Cartesian lift with `0.2 mm` position and
  `0.005 rad` orientation waypoint tolerances.
- Endpoint and Cartesian waypoint IK pass, but bilateral contact still fails
  at the same sample `31/300` (`alpha=0.10368`): right object normal contact
  becomes zero while both actuator commands remain active.
- Thus the previous joint-linear interpolation was a real simulator/real-path
  mismatch and is now corrected, but it was not the cause of this candidate's
  unilateral separation.
- Remaining diagnosis is candidate/OBB/finger contact geometry or simulated
  rigid-contact behavior. Do not tune mass/friction to authorize the task.
- Electronic-skin feedback remains disabled and was not used. Strict gate
  remains failed; no physical motion or robot-control command was issued.

### 03:32 PDT light-rigid Cartesian diagnosis

- Replayed masses `0.001/0.005/0.010/0.020/0.080 kg` with friction extremes
  `0.10` and `1.20`, using the same Cartesian path and live geometry.
- Every case failed bilateral contact. Even a `1 g` object followed only
  `3.013 mm` before loss; friction `0.10` versus `1.20` produced identical
  results.
- This rules out target mass as the primary failure and shows that selecting a
  favorable dynamic tuple cannot solve the candidate.
- The current runtime contact classifier only checks that each finger body has
  some contact with the target body. It does not yet prove the contacts lie on
  opposed side faces with normals aligned to the jaw closing axis.
- A top-edge or same-surface contact can therefore appear "two-sided" during
  closure, then separate during lift exactly as observed.
- Next diagnostic: record each finger/object contact position and the absolute
  contact-normal projection onto the live jaw axis before lift and at first
  loss. Electronic-skin feedback remains disabled.

### 03:34 PDT contact-geometry telemetry patch ready

- Added MuJoCo-only diagnostic telemetry for every finger/object contact:
  finger identity, contact position in base coordinates, contact normal,
  absolute contact-normal projection onto the live jaw-closing axis, normal
  force, and contact distance. The settled state and first contact-loss state
  can now show whether the two contacts are truly on opposed side faces.
- Added the live jaw axis and
  `contact_geometry_telemetry_version=1` to the diagnostic/health contract.
  This is telemetry only: no pass/fail gate, grasp threshold, trajectory,
  electronic-skin path, or robot-control behavior was changed.
- Server protocol tests pass `117/117` with one optional local real-MuJoCo
  smoke skipped; Python compilation and `git diff --check` pass.
- New server artifact is `108854` bytes, SHA-256
  `154e18d8de6b15224600775c1ffe1bb09c2d3d747e5a3394efcfb546f0411420`;
  the ROS-host HTTP endpoint returns the identical hash.
- The currently running WSL health endpoint still reports the preceding
  Cartesian version and does not yet expose
  `contact_geometry_telemetry_version=1`. A WSL server-only update/restart is
  required before the next planning-only replay.
- Electronic-skin force feedback remains excluded. No ROS topic/service was
  published or called, and no physical robot motion, stop, disable, or
  torque-off command was issued.

### 03:38 PDT contact-geometry WSL version verified

- WSL health now reports `contact_geometry_telemetry_version=1` together with
  `lift_path_contract=cartesian_pose_interpolation`, strict dynamic-object
  lift evidence, request-bound close, MuJoCo `3.2.3`, and no missing
  dependency.
- Live ROS parameters remain
  `/gripper/use_compliant_close=false` and
  `/gripper/require_tactile_feedback=false`.
- Only read-only health, parameter, process, and `/joint_states` observations
  were made. No ROS message/service was published or called and no physical
  motion, stop, disable, or torque-off command was issued.

### 03:39 PDT contact geometry identifies lift-edge transition

- Sent one diagnostic-only frozen request directly to WSL:
  `codex-frozen-contact-65781327d9e9a`; response `0.812 s`.
  It used the current read-only six-joint snapshot, the immutable accepted
  geometry/trajectory from plan `96005f458ee360e6b2562ed1`, conservative
  generalized live-OBB dynamics (`0.5 kg`, friction lower bound
  `[0.1,0.005,0.0001]`), and was never published to ROS.
- Strict result remains fail-closed at score `55`: Cartesian IK and collision
  checks pass, but bilateral contact is lost for `269/300` samples and the
  object lifts only `0.045 mm`, below the `15 mm` requirement.
- Settled closure is a real opposed-side pinch, not the suspected same-surface
  classifier false positive:
  - live jaw axis approximately `(0.96267,-0.26642,0.04779)`;
  - left/right contact projections relative to the OBB center are
    `-19.978/+19.710 mm`;
  - both absolute contact-normal/jaw-axis cosines are effectively `1.0`;
  - left/right normal forces are `1.094/1.082 N`.
- At first bilateral loss, sample `31/300`, both actuator commands remain
  active (`-0.995/+1.025 N`) but the right object contact is absent. The only
  remaining left contact has:
  - center-relative jaw projection `-20.005 mm`;
  - normal approximately `(-0.02802,0.07925,0.99646)`, almost the support
    normal;
  - absolute normal/jaw-axis cosine only `0.000475`.
- Conclusion: the initial geometric grasp is correctly opposed, but during
  lift the fingers slide relative to the stationary object until one side is
  lost and the other becomes an edge/support-normal contact. The classifier
  is not the root cause for this replay. The next investigation is whether
  MuJoCo is applying the requested generic object friction/contact dimensions
  and sufficient physical gripper actuator authority; do not alter
  target-specific mass/friction values to force a pass.
- Electronic-skin feedback was not read or used. This was simulation only; no
  physical motion or robot-control command was issued.

### 03:42 PDT zero-velocity Cartesian teleport defect found

- Inspected the active lift implementation after the contact-geometry result.
  Every Cartesian waypoint is solved by IK, then `_set_arm_qpos` directly
  overwrites all six arm joint positions, explicitly zeros all six joint
  velocities, calls `mj_forward`, and advances only one `mj_step`.
- Thus the new 300-sample path has correct spatial and nominal time spacing
  but is not a dynamically executed trajectory: the robot collision meshes
  are repeatedly relocated while their generalized velocities are reported
  as zero.
- MuJoCo's documented forward-dynamics pipeline computes body, actuator, and
  constraint velocities from `qvel`; contact/friction forces depend on the
  position-and-velocity stages. Zeroing `qvel` after every relocation removes
  the tangential surface motion that should transfer the gripper's upward
  movement through friction.
- This implementation defect explains all current evidence without changing
  target assumptions: valid opposed-side contacts at closure, negligible
  object lift, edge transition as the finger geometry passes upward, and weak
  sensitivity to object mass/friction.
- Resolution direction: keep Cartesian waypoint IK as the path reference, but
  execute those references through persistent MuJoCo arm position actuators
  while preserving integrated `qpos/qvel` state. Add measured tool-path
  telemetry and a fail-closed tracking requirement; do not use position
  teleportation as dynamic lift evidence.
- This is a general simulator correction for all live OBB targets. No
  electronic-skin input or target-specific constant is involved, and no
  physical robot-control command was issued.

### 03:42 PDT actuator-integrated Cartesian lift implemented

- Replaced the lift-loop arm-state overwrite with persistent MuJoCo position
  actuator commands. Cartesian IK still computes every live path reference,
  but lift samples now preserve and integrate the simulated arm's measured
  `qpos/qvel` through `mj_step`.
- The pre-contact IK/collision checks and close search remain kinematic by
  design. Only the phase claiming dynamic lift evidence was changed.
- Added non-authorizing evidence fields for measured tool lift, maximum/final
  Cartesian reference tracking error, and final orientation tracking error.
  Health now advertises
  `lift_execution_contract=position_actuator_integrated` while preserving
  `lift_path_contract=cartesian_pose_interpolation`.
- Extended the state-preservation regression to prove that arm actuator
  commands change only `ctrl` and do not rewrite measured joint positions or
  velocities.
- Server protocol tests pass `117/117` with one optional local real-MuJoCo
  smoke skipped. Client compatibility tests pass `80/80`. Python compilation
  and `git diff --check` pass.
- Two initial test collection attempts failed before running tests because the
  shell either lacked the package path or replaced, rather than extended, the
  ROS `PYTHONPATH`, hiding `geometry_msgs`. Re-running with both the workspace
  source path and the sourced ROS path produced the passing results above;
  neither collection failure indicated a product-code failure.
- New server artifact is `112100` bytes, SHA-256
  `4bef3ab246e9062a6954da976700b59e2686d3b37a053f21651dbc62805589f9`;
  the ROS-host HTTP endpoint returns the identical hash.
- This patch does not change the object mass/friction envelope, 15 mm
  dynamic-object lift threshold, collision/contact requirements, task
  trajectory, or any electronic-skin setting. No ROS command or physical
  motion was issued.

### 03:46 PDT actuator-integrated WSL replay reaches strict IK boundary

- WSL health confirms
  `lift_execution_contract=position_actuator_integrated`,
  `lift_path_contract=cartesian_pose_interpolation`, strict dynamic lift,
  contact-geometry telemetry, MuJoCo `3.2.3`, and no missing dependency.
- Live ROS electronic-skin parameters remain both false.
- Diagnostic-only frozen replay
  `codex-frozen-actuator-65781526d13d6` took `2.224 s` and failed closed
  before completing the lift: at Cartesian reference fraction `0.6288`, IK
  position residual was `0.201 mm`, about `1 µm` above the unchanged
  `0.200 mm` tolerance; orientation residual was only `0.000672 rad`.
- Score was `35`, with explicit `MUJOCO_IK_FAILED`. Because the lift aborted
  at that point, the response correctly did not claim contact or lift success
  and cannot be used to infer the final actuator/contact behavior.
- Closure evidence before the abort remained the same valid opposed pinch:
  settled gap `39.566 mm`, left/right normal force `1.094/1.082 N`, and
  opposed contact normals aligned with the jaw axis.
- Root cause in the new implementation: each Cartesian reference IK was
  seeded from the dynamically integrated arm state. Actuator lag and contact
  loads therefore progressively move the IK seed away from the previous
  reference, even though reference generation and plant tracking are
  logically separate.
- Correction: preserve an independent reference-state copy, solve each
  waypoint from the preceding reference IK solution, and send the result to
  the separate persistent actuator state. Keep the `0.200 mm` tolerance
  unchanged rather than hiding the issue by relaxing it.
- The request was direct simulation only and was never published to ROS. No
  electronic-skin input, physical motion, stop, disable, or torque-off command
  was used.

### 03:48 PDT Cartesian reference and dynamic plant separated

- Added an independent MuJoCo data copy for Cartesian reference generation.
  Every waypoint IK is now seeded by the preceding accepted reference IK
  solution, while the original data object remains the actuator-integrated
  dynamic plant under contact load.
- After each reference IK pass, only the reference copy is advanced
  kinematically; its target joint vector is sent to the dynamic plant through
  position actuator controls. No dynamic plant `qpos/qvel` is overwritten.
- Kept the Cartesian position/orientation tolerances at `0.200 mm` and
  `0.005 rad`; no threshold was relaxed.
- Extended the lift regression with distinct fake plant/reference objects and
  asserted that every waypoint solver receives only the reference object.
- Server protocol tests again pass `117/117` with one optional real-MuJoCo
  smoke skipped. Python compilation and `git diff --check` pass.
- Updated server artifact is `112672` bytes, SHA-256
  `9228c1051889b6b85b611b84642482770101700049d8817731fc2017cb3e4780`;
  the ROS-host HTTP endpoint returns the identical hash.
- This remains a simulator-only, target-independent correction. No ROS
  command, electronic-skin data, or physical robot motion was used.

### 03:56 PDT reference-separated actuator replay completes

- WSL health confirms the actuator-integrated Cartesian service is live and
  both electronic-skin ROS parameters remain false.
- Diagnostic-only frozen replay
  `codex-frozen-reference-657817279eab6` completed in `0.832 s`; the previous
  `alpha=0.6288` IK boundary failure is gone and all Cartesian reference IK
  samples pass.
- The strict result still fails closed at score `55`:
  - commanded tool lift `59.639 mm`;
  - actuator-integrated actual tool lift only `31.873 mm`;
  - maximum/final Cartesian tracking error `26.120/25.737 mm`;
  - final orientation error `0.02584 rad`;
  - dynamic object lift only `0.045 mm`;
  - bilateral contact present for `152/300` samples, lost for `148/300`,
    with maximum loss streak `136` versus grace `3`.
- The first bilateral loss is now sample `18/300`, trajectory fraction
  `0.0602`. The left contact is absent; the remaining right contact is still
  a valid side-face contact with normal/jaw-axis cosine effectively `1.0`.
  Contacts later reappear, consistent with an under-tracking/oscillating arm
  plant rather than a clean Cartesian lift.
- Conclusion: reference generation is now correct, but the arbitrary MuJoCo
  arm actuator gains/force limits do not reproduce the real trajectory
  controller. Tuning them until this target passes would be another
  uncalibrated fixed program and is prohibited.
- Next decision must be based on the real execution contract: either derive
  actuator parameters from measured controller dynamics, or model the robot
  as a velocity-consistent prescribed Cartesian path while keeping the object
  fully dynamic and retaining independent real feedback/tracking gates.
- Request was direct simulation only and never published to ROS. No
  electronic-skin input or physical robot-control command was issued.

### 04:00 PDT real-controller contract selects prescribed-path model

- Inspected the active real execution stack rather than tuning the failed
  MuJoCo actuator plant.
- The hardware uses a ROS `position_controllers/JointTrajectoryController`;
  its trajectory points are interpolated before being sent to the serial
  position interface. Driver-side secondary trapezoidal smoothing is
  explicitly disabled because it previously caused post-action lag.
- No measured hardware PID gains, motor torque constants, or joint torque
  feedback are available in this stack. The MuJoCo arm actuator gains
  (`kp/kv`) and `±5` force ranges are therefore not calibrated representations
  of the real controller and cannot scientifically authorize or reject a
  grasp based on plant tracking.
- The real task already provides independent execution evidence:
  retimed cached paths, a strict `0.08 rad/s` joint-speed ceiling, controller
  start synchronization, wait-for-settle from hardware `/joint_states`, and a
  required contact-phase measured `tool0` endpoint residual at most `6 mm`
  and `5 deg`.
- Decision: simulate the robot as the prescribed, controller-tracked path
  boundary while leaving the target object and contacts fully dynamic.
  Consecutive IK references will provide both joint position and consistent
  joint velocity to MuJoCo, so friction sees real tangential surface motion.
  Real tracking uncertainty remains governed by the independent hardware
  feedback gates.
- This is a category-independent execution contract, not target tuning. The
  15 mm object-lift and bilateral-contact gates remain unchanged. No physical
  command or electronic-skin input was used.

### 04:02 PDT velocity-consistent prescribed lift implemented

- Replaced dynamic-arm servo tracking during lift with a prescribed
  controller-tracked boundary. For each pair of consecutive Cartesian IK
  references, the simulator writes the preceding joint position and the
  finite-difference velocity `(q_next-q_prev)/dt`; `mj_step` then computes
  all target-object motion, contact impulses, friction, and collisions.
- This removes both invalid extremes encountered earlier: no zero-velocity
  geometric teleport and no uncalibrated MuJoCo joint-servo lag.
- Health now advertises
  `lift_execution_contract=velocity_consistent_prescribed_position`.
- Added the real strict execution joint-speed contract, `0.08 rad/s`, as a
  backend parameter and health field. Lift sample count is now the stricter
  of Cartesian-speed timing and endpoint joint-displacement timing. Measured
  local reference speed is reported and any remaining limit violation fails
  closed.
- Added unit coverage for joint-limited sample count and for the prescribed
  step preserving its start position while exposing the exact finite-
  difference velocity and next actuator target.
- Server protocol tests pass `117/117` with one optional real-MuJoCo smoke
  skipped. Python compilation and `git diff --check` pass.
- Updated server artifact is `118082` bytes, SHA-256
  `4e16a968defabe0bc4f89a7bd97c236f46488850af28104491f50929214562ad`;
  the ROS-host HTTP endpoint returns the identical hash.
- Object geometry, mass/friction envelope, close command, bilateral-contact
  grace, 15 mm object-lift threshold, and electronic-skin exclusions are
  unchanged. No ROS or physical robot command was issued.

### 04:04 PDT prescribed-path replay exposes local joint-speed bound

- WSL health confirms
  `lift_execution_contract=velocity_consistent_prescribed_position` and the
  real strict `0.08 rad/s` joint-speed limit.
- Diagnostic-only replay
  `codex-frozen-prescribed-6578190d7fac0` completed `1024` dynamic samples in
  `1.420 s`.
- Prescribed motion corrected the prior arm-plant mismatch:
  - actual tool lift `56.981 mm` for `59.639 mm` commanded;
  - final position tracking error `0.242 mm`;
  - final orientation error `0.000660 rad`.
- The server still failed closed because measured local reference speed
  reached `0.2025 rad/s`, above the real `0.08 rad/s` contract. Endpoint-only
  joint-displacement timing underestimates the peak derivative of the
  nonlinear Cartesian IK curve.
- Independent contact/lift evidence also remains negative: the object lifted
  only `0.045 mm`; bilateral contact was present for `24/1024` samples and
  absent for `1000/1024`. This cannot be accepted even after timing is fixed.
- Correct timing resolution: generate the complete live Cartesian IK
  reference first, measure its largest adjacent joint derivative, and
  adaptively increase the sample count until the entire path meets
  `0.08 rad/s` or the bounded sample ceiling is reached. Do not use a fixed
  target-specific multiplier.
- No ROS message/service, electronic-skin input, or physical motion command
  was used.

### 04:07 PDT full-path adaptive timing implemented

- Added a pre-execution pass over the complete live Cartesian IK reference.
  It measures the largest adjacent joint derivative, computes the required
  new sample count from the real `0.08 rad/s` limit, and regenerates the
  reference for up to four bounded passes or the global `4000`-sample ceiling.
- The dynamic object/contact simulation then executes the final reference
  without changing the resulting positions, orientations, or target
  geometry. Initial and final sample counts plus the number of adaptive passes
  are returned as telemetry.
- Corrected the lift's initial reference state to the actual simulated arm
  joint positions after the request-bound `0.8 s` close settle. Using the
  pre-close nominal `grasp_q` could create a false first-sample velocity spike
  from small controller settling motion.
- Existing finite-difference velocity, local speed fail-closed check, strict
  contact accounting, and 15 mm dynamic-object lift requirement remain
  unchanged.
- Server protocol tests pass `117/117` with one optional real-MuJoCo smoke
  skipped. Python compilation and `git diff --check` pass.
- Updated server artifact is `123314` bytes, SHA-256
  `3803fd0a4edc6a42aad20b898a556c3f240a66430d7b382c296a0d9cd01ab710`;
  the ROS-host HTTP endpoint returns the identical hash.
- This is target-independent timing correction only. No ROS command,
  physical motion, or electronic-skin input was used.

### 04:11 PDT adaptive timing reaches IK quantization floor

- Diagnostic-only replay
  `codex-frozen-adaptive-65781a935897b` completed in `10.943 s`.
- Adaptive timing executed exactly as designed: initial `1024` samples were
  expanded to the bounded `4000`-sample ceiling in three passes.
- Tool-path tracking remained accurate (`57.008 mm` actual lift,
  `0.222 mm` final position error, `0.000637 rad` orientation error), but
  local joint reference speed plateaued at `0.0905 rad/s`, still above the
  strict `0.08 rad/s` limit. The request failed closed at score `55`.
- At `4000` samples the Cartesian spacing is only about `15 µm`, yet reference
  IK still stops at the fixed `0.200 mm` acceptance tolerance. It can
  therefore retain one joint solution for several samples and then make a
  small corrective jump, creating a numerical staircase whose finite-
  difference speed no longer decreases with denser sampling.
- Resolution: retain `0.200 mm` as the external Cartesian acceptance bound,
  but derive the internal reference-solver tolerance from each live
  Cartesian/orientation step size so the generated joint curve is smooth.
  Do not raise the joint-speed limit.
- Physical contact remains independently invalid: object lift `0.045 mm`,
  bilateral contact lost `3766/4000` samples. Timing correction alone cannot
  authorize this candidate.
- No ROS command, electronic-skin input, or physical robot motion was used.

### 04:17 PDT orientation numerical floor corrected

- Changed only the internal step-scaled reference-IK orientation floor from
  `1e-6` to `1e-5 rad`. The external strict orientation acceptance remains
  `0.005 rad`, and the position reference still scales to micrometre steps.
- The internal orientation floor is therefore still 500 times tighter than
  the execution acceptance bound while remaining numerically attainable by
  the weighted solver.
- Server protocol tests pass `117/117` with one optional real-MuJoCo smoke
  skipped. Python compilation and `git diff --check` pass.
- Updated server artifact remains `125650` bytes, SHA-256
  `d4672c02af8a65730dcad6b118cef6b5db2c991160acdf78cdeb29df94b2c900`;
  the ROS-host HTTP endpoint returns the identical hash.
- No contact, lift, speed, object, or electronic-skin contract changed. No ROS
  or physical robot command was issued.

### 04:20 PDT smooth-reference implementation exceeds production timeout

- Sent two diagnostic-only requests after deploying the `10 µrad` numerical
  floor. The first used a `30 s` HTTP window; the second attempted a longer
  diagnostic window.
- Neither produced a response body within the local execution window. WSL
  `/health` remained responsive and healthy, showing the service did not
  crash; no simulation result is inferred from a missing response.
- The production ROS client timeout is `20 s`, so this implementation is
  unusable even if a late physical result would pass.
- Code-path cause: up to four complete adaptive IK passes are followed by a
  fifth complete IK pass during dynamic execution. At several thousand
  samples this repeats the expensive solve unnecessarily.
- Resolution: solve one smooth, dense Cartesian IK reference; compute the
  required time scaling from that curve; interpolate the already accepted
  joint curve onto the speed-compliant time grid; verify every interpolated
  point by fast FK against the unchanged Cartesian bounds; then reuse those
  references in the dynamic loop without another IK solve.
- Extending the production timeout is not accepted as a substitute. No ROS
  command, physical motion, or electronic-skin input was used.

### 04:14 PDT step-scaled reference IK removes velocity staircase

- Added an internal reference-IK tolerance derived from the live per-sample
  Cartesian and orientation increments: one quarter of each step, bounded
  above by the existing `0.200 mm`/`0.005 rad` acceptance tolerances and below
  by numerical `1e-6` floors.
- For a `60 mm` path with `4000` intervals, the position reference tolerance
  becomes `3.75 µm`; it is computed from every request and is not a target
  constant.
- Both the adaptive speed prepass and final dynamic execution use the same
  step-scaled tolerances and up to `160` IK iterations. The external
  Cartesian and `0.08 rad/s` acceptance limits are unchanged.
- Added the exact internal position/orientation tolerances to lift evidence and
  a regression for the `60 mm/4000` scaling case.
- Server protocol tests pass `117/117` with one optional real-MuJoCo smoke
  skipped. Python compilation and `git diff --check` pass.
- Updated server artifact is `125650` bytes, SHA-256
  `abbf45d519da1331bc6c6e280f5595dd6ec7f6255c4df877c31e4361a2966a37`;
  the ROS-host HTTP endpoint returns the identical hash.
- This corrects numerical path smoothness only. Contact/lift gates and
  electronic-skin exclusion remain unchanged; no ROS or physical robot command
  was issued.

### 04:16 PDT smooth-reference replay hits orientation numerical floor

- Diagnostic-only replay
  `codex-frozen-smooth-65781b962c0e1` ran `18.454 s` and failed closed during
  the reference prepass at `alpha=0.8651`.
- Reported position residual was below displayed precision (`0.000000 m`);
  orientation residual was approximately `1 µrad`, just above the internal
  `1 µrad` numerical floor. No dynamic lift was executed and score remained
  `35`, `MUJOCO_IK_FAILED`.
- The external orientation acceptance bound remains `0.005 rad`. Requiring
  the weighted iterative IK solver to converge to about `1 µrad` is roughly
  5000 times tighter and is not needed to smooth this almost constant-
  orientation lift.
- Resolution: retain step-scaled orientation precision but use a `10 µrad`
  numerical floor, still 500 times tighter than the external bound. Keep the
  step-scaled micrometre position tolerance because that removes the observed
  joint-position staircase.
- Closure before the abort remained a balanced opposed pinch. No conclusion
  about the final contact/lift phase is taken from this aborted replay.
- No ROS command, electronic-skin input, or physical robot motion was used.

### 04:27 PDT single-pass IK and FK-validated time scaling implemented

- Replaced the production-timeout path identified at `04:20`. The lift now
  solves exactly one dense Cartesian IK curve, beginning from the actual
  post-close simulated arm state. It no longer repeats up to four full IK
  prepasses and no longer solves IK inside the dynamic-contact loop.
- Added target-independent joint-curve time scaling. The largest adjacent
  derivative of the live IK curve determines the required uniform sample
  count under the unchanged real execution limit `0.08 rad/s`; accepted
  joint positions are piecewise-linearly interpolated onto that time grid.
  The global `4000`-sample ceiling and final fail-closed speed check remain.
- Added an independent FK gate over every interpolated point before dynamic
  execution. Each point is compared with the original Cartesian position and
  orientation interpolation under the unchanged external limits `0.200 mm`
  and `0.005 rad`; non-finite or excessive error returns
  `MUJOCO_IK_FAILED` before any lift simulation is accepted.
- The first interpolated point is the actual post-close arm state, preventing
  a synthetic first-sample velocity spike. The dynamic loop reuses the
  verified references and retains velocity-consistent prescribed position,
  request-bound fixed position close, dynamic-object physics, strict
  bilateral-contact accounting, and the `15 mm` lift threshold.
- Added a regression proving that a nonlinear joint reference is resampled
  once, preserves both endpoints, and meets its configured joint-speed
  bound. Lift component tests explicitly isolate the new FK verifier.
- Server protocol regression passes `118/118` with one optional real-MuJoCo
  smoke skipped in `8.23 s`. Python compilation and `git diff --check` pass.
- Updated server artifact is `130490` bytes, SHA-256
  `65039eae47cbfe66fd7f5c07ae56512d847c157b2e544643e65ae71d65ec40a7`;
  the ROS-host HTTP endpoint returns the identical hash.
- The running WSL service remains healthy but still runs the preceding
  numerical-smoothing build. No ROS message/service, physical robot command,
  stop/disable command, or electronic-skin input was used.

### 04:31 PDT single-pass WSL live and first replay rejected by clock skew

- After the operator started the single-pass build, read-only `/health`
  confirmed protocol `3`, MuJoCo `3.2.3`, strict dynamic object lift,
  request-bound close, Cartesian pose interpolation, velocity-consistent
  prescribed position, and the unchanged `0.08 rad/s` limit with no missing
  dependency.
- Read one current `/joint_states` sample for the diagnostic payload; the six
  arm joints are unchanged from the preceding frozen-plan diagnostics.
- Direct WSL-only request
  `codex-frozen-single-pass-18c5d39232026d29` was rejected in `0.009 s` with
  `PLAN_STALE` before IK or simulation. WSL measured the ROS-host-generated
  timestamp as `39.122 ms` in the future, exposing a small host/WSL clock
  skew rather than a trajectory result.
- Resolution: preserve every geometry, trajectory, dynamics, close, and joint
  field and backdate only the diagnostic snapshot timestamp by `0.1 s`, well
  inside the unchanged `2.0 s` freshness bound.
- The rejected plan was not published to ROS. No physical motion,
  electronic-skin input, stop/disable command, or robot-control service was
  used.

### 04:32 PDT single-pass replay meets timing and path contracts

- Replayed the identical frozen geometry, trajectory, conservative live-OBB
  dynamics, close contract, and current six-joint snapshot with only the
  diagnostic timestamp shifted back `0.1 s` to absorb the measured host/WSL
  clock skew. Direct WSL-only plan:
  `codex-frozen-single-pass-clock-18c5d39bcfd75fdf`.
- Full response arrived in `10.742 s`, inside the unchanged production client
  budget of `20 s`. The timeout defect is resolved without extending that
  budget.
- The new execution path is explicitly active:
  - one Cartesian IK curve with `1024` samples;
  - live derivative peak `0.1784 rad/s`;
  - one generic time interpolation to `2283` samples;
  - final measured peak `0.079985 rad/s`, below `0.08 rad/s`;
  - FK verification maximum error `0.015 mm` and `0.000022 rad`, below the
    unchanged `0.200 mm` and `0.005 rad` bounds.
- The prescribed tool followed `57.071 mm` of the commanded `59.639 mm`;
  final tracking error was `0.090 mm` and final orientation error
  `0.000010 rad`. IK, collision, speed, and interpolated Cartesian-reference
  contracts therefore pass.
- The strict physical result still fails closed at score `55`,
  `MUJOCO_CONTACT_FAILED`:
  - dynamic object lift `0.0451 mm`, below `15 mm`;
  - two-sided contact present for `86/2283` samples and absent for
    `2197/2283`;
  - maximum missing-contact streak `2050`, above grace `3`.
- Settled closure remains a valid opposed-side pinch: left/right
  normal-to-jaw cosines are effectively `1.0`, with simulated normal forces
  `1.094/1.082 N`. First bilateral loss is recorded at sample `4`,
  alpha `0.001753`; the right opposed contact remains at `2.189 N` while the
  left side becomes zero. This is not an IK, speed, timeout, same-face
  classifier, or electronic-skin result.
- Next read-only diagnosis is the MuJoCo gripper actuator/force contract. A
  conservative `0.5 kg`, friction `0.1` envelope requires approximately
  `24.5 N` normal force per opposed finger merely to balance gravity, whereas
  the current simulated settled force is about `1.1 N`. Do not alter mass,
  friction, or force to favor this target; first establish whether the model
  has a defensible hardware-calibrated force parameter.
- The request was never published to ROS. No physical motion, electronic-skin
  input, stop/disable command, or robot-control service was used.

### 04:36 PDT mass controls isolate an underpowered actuator model

- Inspected the production MuJoCo and URDF contracts. Each finger joint has a
  declared effort limit of `5 N`, but the injected position actuator uses only
  `kp=55 N/m` with no explicit force range. At the frozen target's roughly
  `20 mm` remaining close error per finger, this predicts about `1.1 N`,
  exactly matching measured settled actuator/contact force. The declared
  `5 N` capability is never approached.
- The production request's `0.5 kg` is generated from every live OBB using
  density upper bound `20000 kg/m^3`, then capped by the generic operational
  ceiling `0.5 kg`; it is not a hard-coded carton mass. With friction lower
  bound `0.1`, static gravity alone requires about `24.5 N` per finger, so
  this worst-case envelope is mathematically infeasible for both the current
  `1.1 N` servo and the declared `5 N` finger limit.
- Ran two non-authorizing sensitivity controls with all geometry, trajectory,
  friction, close, timing, and path fields unchanged:
  - `codex-frozen-single-pass-1g-18c5d3ca0b82c35b`, `1 g`, completed in
    `10.542 s`. The extremely low inertia made the close/contact solve
    pathological: the fingers nearly fully closed and the object penetrated
    the support by `4.152 mm`, so the collision gate rejected it. This is not
    usable lift evidence.
  - `codex-frozen-single-pass-20g-18c5d3d97b843369`, `20 g`, completed in
    `10.913 s`. This is the configured generic mass floor. It still lost
    bilateral contact for `2245/2281` samples and lifted only `0.576 mm`;
    settled force was `1.037 N` per side. Gravity at friction `0.1` requires
    about `0.981 N` per side before any dynamic margin, so failure at this
    marginal force is physically consistent.
- The repository contains a demo-only cube override using `900 N/m` and
  `180 N`, explicitly tuned for a `40 mm`, `60 g` high-friction cube. It is
  rejected for production because it is target-specific and far beyond the
  Alicia-D URDF's `5 N` declaration.
- General resolution: keep position-only closing and electronic skin disabled,
  but model the gripper as an effort-limited position servo whose maximum is
  exactly the existing URDF `5 N` hardware contract. Do not alter target mass
  or friction to manufacture a pass. The position stiffness/damping must be
  derived as a controller-model parameter and the force range must fail
  closed at `5 N`.
- Neither sensitivity request was published to ROS. No physical movement,
  stop/disable command, or robot-control service was used.

### 04:38 PDT URDF-limited position-servo model implemented

- Replaced only the uncalibrated simulated finger stiffness. The production
  model now derives position stiffness from existing hardware-description
  limits:
  - finger stroke `0.025 m` from the fixed 50 mm gripper geometry;
  - joint effort limit `5 N` from both Alicia-D URDF finger joints;
  - stiffness `5 / 0.025 = 200 N/m`;
  - existing damping `7 N·s/m` retained;
  - explicit actuator force range `[-5,+5] N`.
- This remains a fixed-position close toward `0.0 m` with the same `0.8 s`
  settle interval. It does not read force to control the real gripper and does
  not use electronic skin. The limit and stiffness are gripper-wide physical
  parameters, not target mass, friction, label, or geometry tuning.
- Added health telemetry:
  `gripper_effort_contract_version=1`,
  `gripper_joint_effort_limit_n=5`,
  `gripper_position_stiffness_n_m=200`, and
  `gripper_position_damping_n_s_m=7`.
- Added a regression that parses the injected MuJoCo actuators and verifies
  both fingers use the stroke-derived stiffness, retained damping, and exact
  URDF effort range.
- Server protocol tests pass `119/119` with one optional real-MuJoCo smoke
  skipped in `8.06 s`. Python compilation and `git diff --check` pass.
- Updated artifact is `131425` bytes, SHA-256
  `4d684072a63da4921b4ee8d06c143b80bc77050bdf6074aceb03bf0b96030ac8`;
  the ROS-host HTTP endpoint returns the identical hash.
- WSL still runs the preceding `kp=55` single-pass build until a server-only
  reload. No ROS command, physical motion, stop/disable command, or
  electronic-skin input was used.

### 04:41 PDT 5 N servo live and formal 0.5 kg replay

- Read-only WSL health confirms the new model is actually loaded:
  effort-contract version `1`, joint limit `5 N`, stiffness `200 N/m`,
  damping `7 N·s/m`, together with the unchanged single-pass Cartesian,
  `0.08 rad/s`, and strict dynamic-lift contracts.
- Direct diagnostic-only formal-envelope replay
  `codex-frozen-5n-500g-18c5d41d3a8485cb` completed in `10.812 s`.
- Timing/path evidence remains valid: `1024 -> 2281` live samples, measured
  maximum joint speed `0.079994 rad/s`, FK validation maximum `0.007 mm /
  0.000010 rad`, and actual tool lift `57.071 mm`.
- The derived servo materially changes contact mechanics as predicted:
  settled left/right force is now `3.691/3.697 N` instead of about `1.1 N`;
  bilateral lift contact lasts `232` samples instead of `86`; complete
  separation is delayed to alpha `0.10175`.
- The formal `0.5 kg`, friction `0.1` envelope still correctly fails. Available
  ideal tangential support is only about
  `2 * 3.69 * 0.1 = 0.738 N`, versus approximately `4.905 N` gravity.
  Dynamic object lift is only `0.061 mm`; score remains `55`,
  `MUJOCO_CONTACT_FAILED`.
- This result validates the effort-model correction but also proves that the
  existing `0.5 kg` generic envelope lies outside the declared gripper
  capability at its friction lower bound. The next unchanged-trajectory
  control uses the configured `20 g` mass floor to determine whether a
  capability-consistent object can now be retained.
- The plan was never published to ROS. No real movement, electronic-skin
  input, stop/disable command, or robot-control service was used.

### 04:43 PDT 20 g control exposes retained-contact geometry defect

- Capability-boundary control
  `codex-frozen-5n-20g-18c5d42a8832783d` completed in `11.080 s` with the
  identical path, friction, close, limits, and geometry, changing only mass
  from the formal `0.5 kg` envelope to the configured `0.02 kg` floor.
- It still failed closed at score `55`: object lift `1.108 mm`, bilateral
  contact missing for `2078/2281` samples, maximum streak `2061`.
- More importantly, the final settled state is not an opposed pinch:
  - measured gap collapsed to `0.447 mm`;
  - both reported finger/object contact normals are almost parallel to the
    support normal;
  - their absolute normal/jaw-axis cosines are only
    `0.000051/0.000052`, rather than approximately `1.0`.
- The existing classifier calls any simultaneous left-body/object and
  right-body/object contacts “two-sided”, even if both fingers touch a
  support-facing/top edge after the object has been squeezed out. The old
  preload loop also remembers whether valid contact occurred at any earlier
  sample instead of requiring it at the final settled sample.
- Resolution: define a generic opposed-contact gate. A valid pair must place
  left/right contact points on opposite sides of the live object center along
  the live jaw axis, and each contact normal must lie within `30 deg` of that
  axis in absolute cosine. Require this geometry at the final close-settle
  sample and throughout lift accounting. This uses only current contact
  geometry, not object label, fixed coordinates, mass, or target-specific
  tuning.
- This control was never published to ROS. No real motion, electronic-skin
  input, stop/disable command, or robot-control service was used.

### 04:44 PDT geometrically opposed contact gate implemented

- Added a target-independent opposed-contact predicate using each current
  MuJoCo contact position, the current dynamic object center, current jaw
  axis, and contact normal. A valid pair requires:
  - at least one contact from each finger;
  - contact-point projections on opposite sides of the object center along
    the live jaw axis;
  - absolute contact-normal/jaw-axis cosine at least
    `cos(30 deg) = 0.866025`.
- The `30 deg` angular tolerance is a common geometric acceptance cone. It
  does not contain target dimensions, target coordinates, mass, friction,
  label, or a paper-carton exception.
- Runtime broad body-contact classification is retained for collision
  diagnosis, but `two_sided=true` now additionally requires the opposed
  geometry predicate. Missing or malformed geometry fails closed.
- Corrected preload retention: the final sample after the full request-bound
  `0.8 s` settle must still be geometrically opposed. A transient valid
  contact earlier in the settle interval can no longer authorize lift after
  the object has escaped.
- Health now advertises `opposed_contact_gate_version=1` and
  `max_opposed_contact_normal_angle_deg=30`.
- Added regression cases for a valid opposed pair, same-side contact points,
  and support-normal contacts. Server protocol tests pass `120/120` with one
  optional real-MuJoCo smoke skipped in `6.41 s`; Python compilation and
  `git diff --check` pass.
- Updated artifact is `134183` bytes, SHA-256
  `58e2b82908cb5cbbb8781e23b6a13d5f11ee1ee45ba8d1fb793126b7623b71e6`;
  the ROS-host HTTP endpoint returns the identical hash.
- WSL still runs the prior 5 N build until server-only reload. No ROS command,
  real motion, electronic-skin input, stop/disable command, or robot-control
  service was used.

### 04:48 PDT opposed gate live; contact lies at the OBB top edge

- WSL health confirms
  `opposed_contact_gate_version=1`,
  `max_opposed_contact_normal_angle_deg=30`, and the previously verified
  `5 N / 200 N/m / 7 N·s/m` servo contract are all live.
- Replayed the `20 g` diagnostic boundary as
  `codex-frozen-opposed-20g-18c5d47ea502ec6b`. The new gate rejected it in
  `0.375 s` at the final close-settle sample with
  `preloaded grasp did not retain geometrically opposed two-sided contact`.
  No lift simulation ran. This replaces the previous false `two_sided=true`
  and avoids about `10.7 s` of invalid dynamic work.
- Converted the settled `0.5 kg` opposed contact points into the immutable live
  OBB frame. The object half-extents are approximately
  `(22.853,20.052,9.370) mm`. The contacts are correctly on opposite jaw
  faces, but their remaining margin to the support-normal/top boundary is
  only:
  - left `0.396 mm`;
  - right `1.578 mm`.
- Under the earlier `kp=55` replay one contact was effectively at/beyond the
  same top boundary (`-0.051 mm` computed margin), explaining why the contact
  changed into a support-normal edge contact during lift. The stronger
  effort-limited servo delays separation but does not create adequate
  interior-face margin.
- This is a live geometric defect, independent of the unknown target mass:
  the candidate closes near the top edge of the OBB rather than within a
  robust face interior. Correct resolution is a general OBB-local
  contact-margin gate/refinement during candidate selection, not choosing a
  favorable mass.
- The request was never published to ROS. No real motion, electronic-skin
  input, stop/disable command, or robot-control service was used.

### 04:52 PDT opposed-contact build restart acknowledged; selected-source audit corrected

- The operator reported `WSL对向接触门版已启动`. Live `/health` confirms
  protocol v3, MuJoCo 3.2.3, the URDF-bounded
  `5 N / 200 N/m / 7 N·s/m` position-servo contract, and
  `opposed_contact_gate_version=1` with a `30 deg` normal cone.
- The health endpoint's cached joint snapshot was already about `199 s` old
  and ROS synchronization is disabled on the WSL server. It was therefore
  treated as stale diagnostic metadata, not as current robot state. No
  motion request was issued.
- A targeted read of
  `/home/zhuyupei/.ros/grasp6d_gate_audit_latest.json` corrects an earlier
  working assumption: plan `96005f458ee360e6b2562ed1` selected
  `tabletop_geometry` source index 8/variant 0, not a GraspNet candidate.
  Its contact center is exactly the live OBB center
  `(-0.157447,-0.471665,0.059963) m`, with analytical
  `center_distance_m=0`.
- Consequently, a new gate based only on candidate-center distance from the
  OBB center would not catch this failure and would be the wrong
  generalization. The measured MuJoCo contacts are near the OBB top edge
  despite a centered semantic contact point because the selected approach is
  tilted `45 deg`; the relevant quantity is the physical finger/object
  contact band after applying the current tool pose and gripper CAD.
- Next correction direction: evaluate the CAD-derived finger contact band
  against the live OBB in its local frame, and require uncertainty-qualified
  interior-face support before selection. The audit already supplies a live
  `7.0 mm` depth-uncertainty value for this snapshot. This must be derived
  from measured geometry/uncertainty and must not encode a paper-carton
  label, coordinate, or one-off offset.
- This audit was read-only. No ROS publication, real motion, electronic-skin
  input, stop/disable command, or robot-control service was used.

### 05:02 PDT plan-layer distinction prevents a false contact-margin conclusion

- Further source and audit tracing shows that plan
  `96005f458ee360e6b2562ed1` is the promoted
  `FAR_FIELD_OBSERVATION_PLAN`. Its selected-row `45 deg`
  `T_base_tool0` was used to prove the fixed `100 mm` first observation stage,
  not as a near-field contact trajectory.
- The later no-motion strict replay regenerated a
  `CONTACT_EXECUTION_PLAN` candidate with the measured `22.852 mm` endpoint
  uncertainty. Its logged admissible tilt was about `24.305 deg`, and its
  four stage positions are not the observation audit row's four-stage
  contact sequence.
- Therefore the useful discovery that collision padding must not be treated
  as stable finger contact area remains generally valid, but the numerical
  `45 deg / 0.246 mm outside nominal CAD` result cannot yet be assigned to
  the real second-stage collision candidate. It will not be used as evidence
  for a code change.
- Correct next step is deterministic reconstruction from the same immutable
  live OBB, support plane, physical finger CAD, measured endpoint
  uncertainty, and adaptive-stage equations, then evaluate the reconstructed
  `24.305 deg` contact candidate. This avoids both target-specific tuning and
  mixing observation authority with contact authority.
- No WSL simulation request, ROS publication, real motion, electronic-skin
  input, stop/disable command, or robot-control service was used.

### 05:06 PDT real finger-contact-patch gate implemented and verified

- Deterministically reconstructed the near-field candidate from the immutable
  live OBB/support plane, the recorded `22.852 mm` endpoint residual, the
  fixed Alicia-D finger CAD, and the adaptive-stage equations. The resulting
  `24.3052667 deg` grasp translation matches the previously logged replay
  position to better than `0.1 um`, proving this is the contact candidate
  rather than the far-field observation row.
- `depth_mad_m` was inspected before considering it as a contact-margin
  threshold. The current value is a spatial MAD over all retained target
  pixels in the fused image; it contains real object surface depth variation
  as well as sensor noise. It is therefore not used as a new hard contact
  threshold.
- Parsed the checked-in Link7/Link8 binary STL collision meshes (`2650`
  triangles each) and extracted the connected planar inner-face boundary.
  The source mesh hashes are:
  - Link7:
    `d546b7ab908c74281a41261412089a2f83f7519c2444739afe8ffff46450d19d`;
  - Link8:
    `5e5b548734269b731c1d60502c742e3d70d5a97a1e8316db523e2232b4c5043b`.
- The physical inner face is a tapered 26-vertex polygon, not the entire
  `43.4 x 60.0 mm` analytical AABB. The old gate allowed collision-envelope
  padding to stand in for gripping surface. This is why a nominal AABB pass
  could still close on the finger tip and target edge.
- Added `ANALYTICAL_FINGER_CONTACT_PATCH_TOOL_XZ_M` as a hardware contract
  derived from those exact meshes, plus a signed point-to-polygon margin.
  The existing source-independent `finger_reach` stage now rejects a semantic
  contact center outside the real opposing face with
  `GRIPPER_CONTACT_PATCH_MISS`. There is no object label, target coordinate,
  mass, friction, depth-MAD threshold, or paper-carton exception in the gate.
- Frozen contact-candidate replay through the complete analytical gate:
  - `0 deg`: contact-patch margin `+5.270 mm`, pass;
  - `6.076 deg`: `+3.229 mm`, pass;
  - `12.153 deg`: `+1.113 mm`, pass;
  - `18.229 deg`: `-1.787 mm`, rejected;
  - `24.305 deg`: `-4.991 mm`, rejected.
  Both rejected candidates miss the tapered physical contact patch; the
  latter is the candidate used by the failed MuJoCo replay.
- Verification:
  - gripper geometry `88/88` passed;
  - remote-node plus streaming `334/334` passed;
  - tabletop core tests passed;
  - Python compilation and `git diff --check` passed.
  Two separate RealSense-fixture cases cannot run because
  `tests/fixtures/carton_tabletop_cloud.json` is absent; the fixture explicitly
  forbids substituting synthetic points, so no fake fixture was created.
- No WSL simulation request, ROS publication, real motion, electronic-skin
  input, stop/disable command, or robot-control service was used.

### 05:14 PDT opposed-contact build online; safe-candidate reachability isolated

- The operator again confirmed `WSL对向接触门版已启动`; its already verified
  live contract remains protocol v3, MuJoCo 3.2.3, the URDF-bounded
  `5 N / 200 N/m / 7 N·s/m` gripper servo, and opposed-contact gate v1.
- Replayed planning only through
  `/supervisor/check_pose_sequence_strict`, whose handler explicitly plans
  without moving or caching a trajectory. All ten contact-patch-safe
  variants on the narrow OBB jaw direction and all ten safe variants on the
  second generated jaw direction failed at `pregrasp` with
  `MOVEIT_UNREACHABLE`. No execution service was called.
- Tested a generalized lateral contact-center refinement for the previously
  reachable `24.305 deg` tilt. At the required `3 mm` support-plane
  clearance, the exact feasible line in tool X/Z has no intersection with
  the CAD-derived physical finger contact polygon. A lateral offset therefore
  cannot simultaneously preserve table clearance and move this tilt onto the
  gripping face; no offset or target-specific compensation was implemented.
- Candidate-generation audit exposed a separate coverage risk:
  five live tabletop proposals create 90 materialized variants, but the
  global sort followed by `[:24]` retains all 18 variants from the first
  proposal and only six from the second. Three live proposal directions do
  not reach later stability/MoveIt evaluation. This is not yet modified:
  an apparent inconsistency between one audit row's accepted required width
  and the independently projected OBB width must first be resolved, so a
  diversity change is not being claimed as a fix prematurely.
- Next diagnostic is to distinguish free-space tracking uncertainty from the
  `22.852 mm` endpoint residual recorded after the collision. If the latter
  was sampled from a collision-truncated stage, treating it as independent
  execution uncertainty can unnecessarily expand pregrasp beyond the
  reachable workspace.
- No real motion, WSL simulation request, electronic-skin input, stop/disable
  command, or robot-control execution service was used.

### 05:19 PDT endpoint provenance and tabletop-direction coverage resolved

- Traced the `22.852 mm` value back to the archived collision event. The
  controller had accepted the joint endpoint, while FK of the measured joints
  put `tool0` about `21.5 mm` below and `7.2 mm` short of the commanded
  pregrasp. The measured-endpoint recorder and fail-closed contact check were
  implemented after that event; the value was subsequently injected only
  into conservative offline replays.
- The current live ROS parameter
  `/grasp_6d/runtime_execution_error` is unset. Production near-field
  generation consumes a sample only while the real task is active, and the
  fixed-100-mm observation stage will publish a fresh settled endpoint
  residual before the near-field view is generated. Therefore the collision
  residual must remain a robustness regression case, not be asserted as the
  current free-space tracking error.
- Resolved the audit-row width discrepancy. The earlier independent check
  incorrectly used the support-function projection width of the OBB. Finger
  reach through a centered jaw line is instead bounded by the line/OBB
  intersection, `min(half_extent_i / |jaw_local_i|)`. With that correct
  geometry, audit row 46's cloud-derived `45.770 mm` opening and successful
  `50 mm` physical-gap gate are consistent.
- Consequently, the five generated tabletop proposals are genuinely
  aperture-valid live directions. The current global sort plus `[:24]`
  admits variants from only two of them and starves three valid measured
  directions before stability and MoveIt. The generalized correction will
  bound candidates by deterministic round-robin strata over source proposal,
  so every live direction receives an evaluation slot before extra
  tilt/wrist variants consume the remaining budget.
- No target label, fixed target coordinate, paper-carton branch, new physical
  threshold, real motion, WSL simulation request, electronic-skin input,
  stop/disable command, or robot-control execution service was used.

### 05:20 PDT proposal-stratified bounding implemented and regressed

- Replaced the global materialized-candidate prefix with a deterministic
  round-robin over the live proposal source index. Within each proposal the
  existing physical-score/source/variant order is preserved; across
  proposals, each measured jaw direction receives one slot before any
  direction receives a second tilt/wrist slot.
- Added audit fields `materialized_proposal_count` and
  `bounded_proposal_count`, plus a regression using five proposal directions
  and a five-candidate budget. It proves the bounded batch contains every
  direction exactly once before repeats.
- The first test invocation used bare `pytest`, which is absent from the
  shell PATH. A second invocation used `python3 -m pytest` without the
  catkin environment and correctly exposed the missing generated ROS message
  module. The test command was then corrected to source this worktree's
  `devel/setup.bash`; no package installation or environment mutation was
  performed.
- Verification after correction:
  - focused diversity/profile/fail-closed tests: `4/4` passed;
  - full streaming suite: `181/181` passed;
  - remote node, gripper geometry, and tabletop geometry suites:
    `266/266` passed;
  - Python compilation and `git diff --check` passed.
- This change contains no object label, target coordinate, fixed contact
  value, or paper-carton specialization. The ROS process has not yet been
  restarted with the change. No real motion, WSL simulation request,
  electronic-skin input, stop/disable command, or robot-control execution
  service was used.

### 05:30 PDT updated ROS candidate node live; nine directions reach the gate

- Gracefully terminated only the old idle `remote_grasp6d_node` process
  `88810` and started the updated node as PID `107969`. Its terminal confirms
  the GraspNet backend is loaded against protocol-v3 WSL endpoint
  `http://172.23.132.97:8000`. Driver, hardware interface, trajectory
  controller, MoveIt, hand-eye TF, task node, GUI, and joint-enable path were
  not restarted.
- The first two-second shutdown poll ended just before the old process was
  fully reaped and returned nonzero. A read-only process check immediately
  afterward confirmed PID `88810` was gone; no force-kill was used.
- Started only continuous candidate inference. The live terminal emitted the
  new `GRIPPER_CONTACT_PATCH_MISS` code, proving the CAD-derived Link7/Link8
  contact-patch gate is loaded.
- Frozen request 6 proves the new source stratification is also loaded:
  - nine aperture-valid live proposal directions;
  - 162 total tilt/wrist materializations;
  - 24 bounded candidates;
  - `materialized_proposal_count=9`;
  - `bounded_proposal_count=9`.
  The previous global-prefix behavior would have retained only the first two
  directions.
- For that frozen five-frame RGB-D estimate, 21 tabletop candidates passed
  the full analytical gate. The adaptive profile was entirely live-derived:
  object height `17.546 mm`, depth uncertainty `3.0 mm`, current execution
  residual `0`, approach offset `20.546 mm`, pregrasp distance `32.546 mm`,
  lift `33.0 mm`, and tilt family
  `0/11.25/22.5/33.75/45 deg`.
- Strict MoveIt attempted 24 diverse candidates and accepted none; every
  checked candidate failed at `pregrasp` as `MOVEIT_UNREACHABLE`. No invalid
  preview was promoted.
- Later queued frames also exposed expected fail-closed causes:
  cross-frame target-cloud width mismatch, CAD contact-patch misses, and
  target-instance mismatch. Candidate streaming was then stopped to freeze
  evidence; the updated ROS node remains online. This service controls only
  inference generation and is not the grasp stop/disable path.
- The latest published object geometry was center approximately
  `(-0.15114,-0.48094,0.06126) m`, size
  `(40.807,40.479,20.296) mm`, support normal
  `(-0.03569,0.09047,0.99526)`, depth MAD `2.5 mm`, five fused frames, and
  430 target points. The object label is audit metadata only and is not used
  by the candidate geometry calculation.
- No real motion, WSL simulation request, electronic-skin input, stop/disable
  command, or robot-control execution service was used.

### 05:33 PDT stage-isolated planning proves a pose/IK conflict, not standoff

- Used only `/supervisor/check_pose_sequence_strict`, whose implementation
  plans without execution or cache promotion, to isolate the three positions
  of four analytical-safe candidates from frozen request 6.
- Each candidate's exact `grasp`, `approach`, and `pregrasp` pose was planned
  separately as a non-linear strict-pose target. All `12/12` returned
  `MOVEIT_UNREACHABLE`. Therefore shortening the live-derived
  `32.546 mm` pregrasp distance cannot solve this case: the same orientation
  is already unreachable at the contact and approach positions.
- Reconstructed a second independent top-down family from the latest
  published geometry (support normal
  `[-0.03569,0.09047,0.99526]`, much closer to vertical), using both live OBB
  jaw axes and both wrist symmetries. Single-stage strict planning of grasp
  and pregrasp produced `0/8` reachable poses. The result is not caused only
  by request 6's more tilted support-plane estimate.
- Two initial read-only reconstruction scripts failed before any service call:
  the first imported `semantic_axes_to_tool_rotation` from the wrong module;
  the second used the wrong keyword name for
  `solve_tool0_translation_for_support_clearance`. Both were corrected to the
  production module/signature, and the resulting eight planning calls
  completed normally.
- Combined with the earlier reachable `24.305 deg` candidate, the evidence
  identifies a physical/kinematic conflict: low tilts land on the real finger
  patch but are not IK-reachable at this target; higher tilt is reachable but
  a centered OBB contact lands outside the tapered patch.
- Generalized next direction is not a fixed positional compensation. It is to
  solve a contact-height interval on the measured OBB side face from the
  intersection of live object geometry/point support and the fixed CAD
  contact polygon. This may let a reachable tilt contact a real supported
  side-face band without moving the target or encoding its label/coordinates.
- No real motion, WSL simulation request, electronic-skin input, stop/disable
  command, or robot-control execution service was used.

### 05:39 PDT contact-height relabel rejected; moderate-tilt reachability mapped

- Swept the semantic contact point along the live OBB support-normal extent
  while holding the CAD-cleared tool pose fixed. This measures the exact
  intersection between the target side-face height and the Link7/Link8 inner
  contact polygon.
- For frozen original geometry:
  - `18.229 deg` intersects from about `+3.373` to `+9.370 mm`;
  - `22.5 deg` intersects only from `+6.840` to `+9.370 mm`;
  - `24.305 deg` intersects only from `+8.058` to `+9.370 mm`.
  Thus the previously reachable tilt has only about `1.312 mm` of overlap at
  the target top boundary.
- For the latest published geometry:
  - `18.229 deg`: about `+2.588..+10.148 mm`;
  - `22.5 deg`: about `+6.089..+10.148 mm`;
  - `24.305 deg`: about `+7.306..+10.148 mm`.
  Tilts `33.75` and `45 deg` have no contact-patch/target-height
  intersection at all.
- Merely changing `candidate_center_base` to one of those high points does
  not change `solve_tool0_translation_for_support_clearance`: its normal
  translation is fixed by the table-clearance constraint. The real tool
  trajectory and physical contacts would remain at the target top edge.
  Therefore a contact-height relabel is not a physical correction and was not
  implemented.
- Planning-only reachability was then sampled for the latest geometry at
  `18.229/22.5/24.305 deg`, both OBB jaw axes, both wrist symmetries, and both
  tilt polarities (`24` strict pregrasp probes). Five pregrasp poses and their
  corresponding grasp poses were reachable.
- The sole reachable `18.229 deg` branch required about `3.33 rad` maximum
  joint delta and remains above the existing `2.15 rad` transition gate.
  Two useful lower-delta branches appeared at:
  - `22.5 deg`: maximum delta about `1.792 rad`;
  - `24.305 deg`: maximum delta about `1.705 rad`.
  Those orientations still have only a narrow top-edge physical overlap and
  cannot yet be promoted.
- This shows why expanding tilt diversity is necessary for kinematic
  coverage, but also why reachability alone must not override the physical
  finger/object overlap requirement.
- No real motion, WSL simulation request, electronic-skin input, stop/disable
  command, or robot-control execution service was used.

### 05:52 PDT exact IK boundary and support-normal lift defect isolated

- Queried only MoveIt's `/compute_ik` service with a `2 s` timeout, both with
  and without collision avoidance. All four reconstructed top-down pregrasp
  poses returned `NO_IK_SOLUTION (-31)`, while the `22.5 deg` control pose
  returned success in both modes. This proves the low-tilt failure is a true
  inverse-kinematics absence at the current measured geometry, not a planning
  timeout or collision-only rejection.
- For the useful live OBB axis/wrist/polarity branch, sampled the IK boundary
  from `18.0` through `22.5 deg` in `0.5 deg` increments. `18.0 deg` has no
  IK; every sample from `18.5` through `22.5 deg` has IK. The corresponding
  physical Link7/Link8 contact-patch overlap with the measured target height
  falls monotonically from about `7.327 mm` at `18.5 deg` to `4.079 mm` at
  `22.5 deg`.
- Ran the complete strict, planning-only sequence for the live-derived
  `18.5 deg` candidate. Pregrasp, approach, and grasp passed, but the existing
  fixed world-Z lift failed with Cartesian fraction `0.750 < 0.980`.
  Replacing only that lift direction with the measured support-plane normal
  made the complete sequence pass, with maximum joint delta about
  `2.1324 rad < 2.15 rad`.
- The generalized defect is therefore the lift reference frame: a tabletop
  retreat must follow the live support normal, because world Z is correct
  only for a perfectly level support plane. This is independent of object
  category, label, or position.
- No real motion, WSL simulation request, electronic-skin input, stop/disable
  command, or robot-control execution service was used.

### 05:56 PDT support-normal lift implemented in the common sequence API

- Extended the common 6D sequence builder with an optional validated
  `lift_direction_base`. Production tabletop sequence construction now passes
  the live unit support normal; callers that do not provide a direction retain
  the legacy world-Z behavior for compatibility.
- Added a unit test proving the same positive lift distance is applied along
  an arbitrary normalized live support direction. Focused sequence regression
  passed `9/9`; Python compilation and `git diff --check` also passed.
- The updated lift code is local only at this point. The live
  `remote_grasp6d_node` still has the earlier source-stratification/contact
  center build and has not yet been restarted with the lift correction.
- The next analytical correction is to replace the over-conservative
  single-center patch test with the actual intersection length between the
  fixed CAD contact polygon and the target's live bilateral side-face height.
  The minimum accepted length will be derived from the existing measured
  `6.0 mm` endpoint-position contract plus the fixed `0.5 mm` CAD contract
  tolerance, not from this target or its label.
- No real motion, WSL simulation request, electronic-skin input, stop/disable
  command, or robot-control execution service was used.

### 06:17 PDT opposed-contact height gate implemented; first focused run

- Added exact concave-polygon line intersection for the fixed Link7/Link8 CAD
  contact patch. The gate now measures the longest continuous intersection
  between that physical patch and a live support-normal target-height range;
  it no longer requires the semantic OBB center itself to land on the patch.
- Added a bilateral cloud-height estimator. It independently measures the
  target height range near the negative and positive jaw extremes and uses
  only their common interval, then the physical gate clamps it to the live OBB
  height. One-sided or non-overlapping point support therefore cannot pass.
- Production derives the required continuous overlap as the configured
  measured endpoint-position tolerance (`6.0 mm`) plus the immutable gripper
  CAD contract tolerance (`0.5 mm`), giving `6.5 mm` in the current runtime.
  The same live target cloud is now supplied to both tabletop and GraspNet
  candidate gates.
- Tabletop proposal audits now preserve their measured bilateral height
  bounds. No object category, label, fixed target pose, carton dimension, or
  target-specific offset is involved.
- First focused result:
  - tabletop geometry: `24/24` passed;
  - Python compilation passed;
  - gripper geometry: `89/90` passed. The sole failure was in the new test
    assertion because `pytest.approx` does not accept a nested tuple; the
    computed interval itself was the expected `[-10,+10] mm`. This is a test
    representation issue, not a gate result, and will be corrected directly.
- No ROS process was restarted and no real motion, WSL simulation request,
  electronic-skin input, stop/disable command, or robot-control execution
  service was used.

### 06:26 PDT live contact-boundary tilt generation added; zero-degree branch defect found

- Added proposal-local tilt boundary derivation. For each live jaw direction,
  wrist symmetry, and tilt polarity, it evaluates the CAD/target overlap at
  the geometry-derived stage-profile samples and bisects the last safe-to-
  unsafe interval. Only tilts retaining the same `6.5 mm` robust physical
  overlap enter the bounded planning batch.
- Reordered each proposal stratum so live-derived boundary tilts precede the
  usually unreachable top-down pose. The first branch is rotated by proposal
  index across the four wrist/polarity combinations, preserving both measured
  jaw-direction diversity and orientation-branch diversity within a finite
  MoveIt budget.
- Corrected the earlier nested-tuple test assertion; gripper geometry now
  passes `90/90`.
- The first boundary-focused streaming test produced only vertical candidates
  for an `11 mm` synthetic object. Diagnosis: the probe data stores the
  vertical pose under polarity `0`, while the four tilted branches are keyed
  by polarity `-1/+1`; therefore a branch that is safe at `0 deg` but already
  unsafe at the first coarse tilt has no safe lower endpoint for bisection.
  The physical threshold was not relaxed. The solver will explicitly seed
  each tilted branch with its corresponding wrist's zero-degree overlap.
- No ROS process was restarted and no real motion, WSL simulation request,
  electronic-skin input, stop/disable command, or robot-control execution
  service was used.

### 06:34 PDT boundary seed fixed; full regression found constructor-fixture compatibility gap

- Seeded each `-1/+1` tilted polarity branch with the matching wrist's measured
  zero-degree overlap. The boundary-focused tests now pass `3/3`; the solver
  finds a positive live-derived tilt even when the first coarse tilt sample is
  already unsafe.
- Full regression results before compatibility correction:
  - streaming: `177/182` passed;
  - remote node plus gripper/tabletop/sequence: `268/277` passed;
  - compilation and `git diff --check` passed.
- All fourteen failures share one cause: unit fixtures instantiate
  `RemoteGrasp6DNode` via `__new__` and intentionally omit normal constructor
  fields. New call sites directly accessed `tabletop_geometry_config` and
  `minimum_contact_patch_overlap_m`, raising `AttributeError` before the
  mocked/analytical candidate gate. The production constructor does populate
  both fields, so this is a compatibility gap in helper access rather than a
  physical-gate failure.
- The correction is to centralize validated accessors with the existing
  category-independent defaults (`contact_band_fraction=0.12` and the fixed
  CAD tolerance when no task-level measured tolerance exists). A fully
  constructed production node continues to use the derived `6.5 mm` value.
- No ROS process was restarted and no real motion, WSL simulation request,
  electronic-skin input, stop/disable command, or robot-control execution
  service was used.

### 06:40 PDT constructor compatibility fixed; sparse legacy geometry fixture rejected

- Added validated helper accessors for the contact-band fraction and minimum
  contact overlap. Production still reads the constructor-derived `6.5 mm`;
  isolated `__new__` tests use the existing category-independent defaults.
- Full streaming regression now passes `182/182`.
- The combined remote/gripper/tabletop/sequence run improved but remains
  `268/277`: all nine remaining failures use the legacy
  `make_geometry_estimate` fixture, whose target cloud contains only three
  sparse points at unrelated OBB interior locations. Such data cannot establish
  a shared negative/positive jaw-side height interval, so the new production
  gate correctly returns `GRIPPER_CONTACT_PATCH_MISS`.
- The fix will be confined to the test fixture: generate bilateral 3D OBB
  corner points from the fixture's requested center and size. Production will
  remain fail-closed when real target points cannot prove opposed contact.
  This preserves the new scientific contract instead of silently falling back
  to OBB-only height for sparse data.
- Compilation and `git diff --check` continue to pass. No ROS process was
  restarted and no real motion, WSL simulation request, electronic-skin input,
  stop/disable command, or robot-control execution service was used.

### 06:43 PDT bilateral test fixture corrected; one metadata expectation updated

- Replaced the three sparse legacy fixture points with the eight corners of
  the fixture's own live center/size OBB. Remote-node regression then passed
  `153/154`.
- The sole remaining failure expected the old fixture's
  `object_point_count=3`; the corrected bilateral fixture truthfully publishes
  `8`. The assertion will be updated to match the fixture data. No production
  behavior or safety threshold is being changed.
- No ROS process was restarted and no real motion, WSL simulation request,
  electronic-skin input, stop/disable command, or robot-control execution
  service was used.

### 06:46 PDT opposed-contact gate and live-boundary implementation fully regressed

- Updated the corrected fixture metadata expectation from three to eight
  points. The combined remote-node, gripper-geometry, tabletop-geometry, and
  sequence suites now pass `277/277`.
- The full streaming suite already passes `182/182`. Python compilation and
  `git diff --check` also pass.
- The verified production behavior is now:
  1. estimate a common negative/positive jaw-side height range from the frozen
     live target cloud;
  2. intersect it exactly with the concave Link7/Link8 CAD contact polygon;
  3. require continuous overlap of measured endpoint tolerance plus CAD
     tolerance (`6.5 mm` in the current configuration);
  4. derive the largest safe tilt independently for each live jaw/wrist/
     polarity branch;
  5. distribute those branches across the bounded candidate budget;
  6. lift along the live support normal.
- The currently running ROS candidate process still has the previous build.
  Next action is a graceful hot restart of only `remote_grasp6d_node`, leaving
  driver, controller, MoveIt, task node, pruned19 TF, GUI, and joint-enable
  state untouched.
- No real motion, WSL simulation request, electronic-skin input, stop/disable
  command, or robot-control execution service was used.

### 06:52 PDT updated ROS candidate node hot-restarted

- Sent `SIGTERM` only to the old idle `remote_grasp6d_node` PID `107969`.
  It exited cleanly with code `0`; no force kill was used.
- Started the updated node as PID `116972`, supervised in terminal session
  `69711`. Its terminal reports:
  `backend=graspnet_baseline loaded=True protocol=3
  url=http://172.23.132.97:8000`.
- This restart loads the bilateral `6.5 mm` contact-overlap gate,
  live-derived per-branch boundary tilts, proposal/branch stratification, and
  support-normal lift. Driver, hardware interface, trajectory controller,
  MoveIt, task node, GUI, pruned19 hand-eye TF, and the joint-enable path were
  not restarted or commanded.
- Candidate inference remains stopped until a fresh, explicitly bounded
  planning-only observation is requested. No real motion, WSL simulation
  request, electronic-skin input, stop/disable command, or robot-control
  execution service was used.

### 06:56 PDT candidate inference trigger corrected and started

- An initial one-shot `rostopic pub` was sent to the name
  `/grasp_6d/request_plan`. Runtime introspection showed this name is a
  `TriggerZero` service, not a subscribed topic, so that publication had no
  consumer and changed no node, robot, or task state.
- Called the actual service with `trigger: true`. It returned
  `success: True` and `continuous remote 6D inference started`.
- Read-only node introspection confirms the updated candidate node is connected
  to live color, depth, object mask, target pose, joint state, pruned19 TF,
  task state, GUI, and plan consumers. This was runtime supervision after the
  explicit start, not a prelaunch motion/safety check.
- No real motion, WSL simulation request, electronic-skin input, stop/disable
  command, or robot-control execution service was used.

### 07:01 PDT live contact-boundary stream reached stability; fused-axis drift found

- Supervised three fresh continuous requests against the online opposed-
  contact WSL build. The updated ROS node accepted live RGB-D, completed WSL
  inference, and materialized `18`, `12`, and `12` tabletop candidates in the
  first three requests. No physical motion was requested.
- Requests 1 and 2 were stability-pending. Request 3 produced twelve stable
  tabletop tracks, proving the new contact-overlap/boundary candidates survive
  multi-frame tracking.
- The final stable hard recheck rejected all variants before MoveIt with
  `jaw axis must be parallel to the support plane`. The original candidates
  are constructed exactly orthogonal; the multi-frame quaternion fusion
  introduced a small jaw/support dot-product drift, and the existing
  translation-only support-clearance reprojection cannot correct orientation.
- Stopped only continuous candidate inference using
  `/grasp_6d/request_plan trigger:false` to freeze request 3's audit. The
  service confirmed `continuous remote 6D inference stopped`; the candidate
  node remains online, and this is not `/grasp/stop`, a task stop, a
  controller stop, or a joint-disable command.
- Generalized correction: for fused tabletop candidates, reproject the fused
  insertion axis to its measured tilt while rebuilding a right-handed
  orientation from the live support normal and the candidate's preserved jaw
  direction, then solve translation clearance. Do not relax the orthogonality
  gate.
- No real motion, WSL simulation request, electronic-skin input, stop/disable
  command, or robot-control execution service was used.

### 07:08 PDT stable tabletop orientation reprojected without relaxing gates

- Extended the stable tabletop reprojection from translation-only to a full
  physical-contract reconstruction:
  - project the fused tool jaw axis into the live support plane;
  - project the independently fused insertion axis perpendicular to that jaw;
  - rebuild the exact right-handed tool rotation through the common semantic
    axis mapper;
  - solve the same CAD finger/support clearance with that rotation.
- The corrected rotation, insertion axis, and tool0 translation now replace
  the corresponding stable-candidate values used by subsequent safety,
  observation, scoring, and planning stages. Wrist variant polarity is
  preserved from the fused pose.
- Added a regression with both `0.25 mm` translation drift and `0.010 rad`
  local orientation drift. Both stable wrist variants pass, restore support
  clearance, and have jaw/support dot product at most `1e-10`.
- Focused stable-tabletop tests pass `3/3`; compilation and
  `git diff --check` pass. No physical threshold was relaxed.
- The live ROS node has not yet been restarted with this follow-up correction.
  No real motion, WSL simulation request, electronic-skin input, stop/disable
  command, or robot-control execution service was used.

### 07:12 PDT full regression isolated source-scope error in stable replacement

- Combined remote/gripper/tabletop/sequence regression passes `277/277`.
  Streaming passes `181/182`.
- The sole streaming failure is a GraspNet-only synthetic stable candidate:
  the newly corrected pose orientation/insertion was written back
  unconditionally after both source branches. For that legacy synthetic
  GraspNet pose, its identity quaternion's configured tool-Z disagrees with the
  explicitly supplied `-Z` approach, so the unrelated safety fixture is
  rejected.
- The orientation reconstruction is physically required only for
  `tabletop_geometry`, where support-plane/jaw contracts exist. The write-back
  will be scoped to that source; GraspNet retains its existing independent
  source contract and fused approach behavior.
- No ROS process was restarted and no real motion, WSL simulation request,
  electronic-skin input, stop/disable command, or robot-control execution
  service was used.

### 07:15 PDT source-scoped stable correction fully regressed

- Scoped corrected quaternion/insertion write-back to
  `tabletop_geometry`; GraspNet keeps its independent fused-source contract.
- Full streaming regression returns to `182/182`. The most recent combined
  remote/gripper/tabletop/sequence result remains `277/277`.
- The live node still contains the pre-correction build. It will be gracefully
  hot-restarted once more, then candidate inference will be rerun only through
  strict planning.
- No real motion, WSL simulation request, electronic-skin input, stop/disable
  command, or robot-control execution service was used.

### 07:24 PDT corrected live node produced reachable preview; MuJoCo payload blocked by legacy fixed dynamics

- Gracefully hot-restarted only the candidate node again; updated PID is
  `118400`, supervised in terminal session `41836`. WSL protocol 3 and
  GraspNet reported online.
- Fresh request 3 produced:
  - `12/24` stable variants passing the hard physical recheck;
  - `12/12` entering strict MoveIt;
  - `2/12` MoveIt reachable;
  - one bound `PREVIEW_READY` plan.
  This proves the stable jaw/support reconstruction works in the live stream.
- Stopped only continuous inference to freeze the preview. The latched plan is
  `af2bd2b6331c42f8212c2179`, source `tabletop_geometry`, required opening
  `41.473 mm`, live OBB size about
  `(40.540,36.141,19.262) mm`, and live support normal
  `(-0.03510,0.12124,0.99200)`.
- The exact preview contains the fixed `100 mm` observation pregrasp plus the
  live-derived approach, grasp, and support-normal `33.6 mm` lift. It remains a
  preview only; no robot motion was executed.
- Attempted to build the formal schema-v3 MuJoCo payload with the latched rich
  plan and a fresh `/joint_states` sample. The builder rejected locally before
  any WSL request:
  `object_model fixed friction,mass_kg is category-specific`.
- Diagnosis: the running `/mujoco_digital_twin` parameters still expose legacy
  fixed mass/friction fields, while the current generalized builder requires
  mass from live OBB volume plus a density/operational envelope and friction
  from a category-independent lower bound. These fixed fields must be removed,
  not bypassed.
- No real motion, successful WSL simulation request, electronic-skin input,
  stop/disable command, or robot-control execution service was used.

### 07:29 PDT generalized dynamics written; ROS parameter leaves require exact removal

- Set `/mujoco_digital_twin/object_model` to the checked-in generalized
  live-OBB envelope. A second payload build still rejected the legacy
  `mass_kg/friction`.
- Read-back showed why: ROS parameter-server hierarchy retained the old leaf
  parameters while adding the new fields, so the subtree contained both
  contracts. It also retained the target label `carton`.
- Next correction is exact removal of only the three obsolete leaves:
  `mass_kg`, `friction`, and `label`. The generalized `type`,
  density/mass bounds, and friction lower bound remain. This is recoverable
  from the checked-in YAML and does not touch robot control.
- No real motion, successful WSL simulation request, electronic-skin input,
  stop/disable command, or robot-control execution service was used.

### Offline continuation after operator hardware shutdown

- Operator explicitly reported that all hardware serial ports and the
  manipulator are powered off, and requested continuation only from preserved
  evidence without new real images.
- Work is now restricted to deterministic configuration cleanup, frozen-plan
  audit, offline payload/contract analysis, tests, and documentation.
- Camera, serial ports, robot driver, joint enable, perception acquisition,
  candidate acquisition, and real motion will not be started. The work must
  stop at the first point requiring a fresh image, current hardware joint
  state, or physical validation.

### Offline configuration authority verified

- Read-only `rosparam get` returned `Unable to communicate with master`, which
  is consistent with the operator's shutdown. Therefore the three stale
  in-memory parameter leaves cannot and need not be deleted from the stopped
  parameter server.
- Searched the checked-in launch/config authority. The only current object
  dynamics fields are the generalized live-OBB density/mass envelope and
  friction lower bound; there is no fixed `mass_kg`, `friction`, or target
  label in `grasp_params.yaml` or the launch files.
- On the next ROS launch, the correct checked-in subtree will be loaded
  without the obsolete runtime leaves. No file change is needed for this
  issue, and no ROS or hardware service was started.
- `git diff --check` passes.

### Frozen preview is far-field authority, not a final contact plan

- Rechecked the preserved rich preview evidence for plan
  `af2bd2b6331c42f8212c2179`. Its diagnostic is explicitly
  `FAR_FIELD_OBSERVATION_PLAN`.
- Its pregrasp is the agreed fixed `100 mm` observation standoff. Its lift
  pose also offsets only world Z, even though the associated live support
  normal is `(-0.03510,0.12124,0.99200)`.
- Therefore this frozen preview must not be presented as the final near-field
  contact authority or used to claim that the corrected support-normal lift
  passed MuJoCo. The task's second-stage plan requires a new close-range RGB-D
  estimate after reaching the observation pose.
- Offline work will verify how the task consumes a far-field plan and make
  any provable sequence-contract correction. Generating or validating the
  final near-field plan requires real images and is outside the current
  hardware-off phase.

### Far-field contact misuse now fails closed

- Verified the intended production sequence in `grasp_task_node`:
  1. a `FAR_FIELD_OBSERVATION_PLAN` defers MuJoCo and uses only its first
     `100 mm` observation pose;
  2. after reaching/reusing that view, the task requests a fresh near-field
     preview;
  3. only a different `CONTACT_EXECUTION_PLAN` may be rebound;
  4. that exact near-field plan must pass MuJoCo before contact execution.
- Found one configuration-failure path: if near-field replanning were disabled
  while a far-field plan was bound, the old code could simulate/execute its
  placeholder contact poses.
- Added an explicit fail-closed guard:
  `NEAR_FIELD_REPLAN_CONFIG_INVALID` is returned before simulation, gripper
  command, or motion whenever far-field authority is used without both
  `near_field_replan_enabled=true` and
  `near_field_replan_required=true`.
- Added a no-side-effect regression. Focused far-/near-field tests pass `4/4`;
  MuJoCo client/server protocol tests pass `200` with one expected skip;
  compilation and `git diff --check` pass.
- No ROS master, image source, hardware driver, simulation server request, or
  robot command was started.

### 07:04 PDT frozen far-field envelope audited without hardware

- Recomputed the preserved plan `af2bd2b6331c42f8212c2179` only from its
  recorded values; no new image, ROS state, WSL request, or hardware state was
  used.
- The recorded live OBB size
  `(0.0405395285, 0.0361411443, 0.0192621844) m` gives volume
  `0.000028221892 m^3`.
- Under the checked-in generalized contract, the conservative density upper
  bound `20000 kg/m^3` gives `0.564437844 kg` before clamping and the configured
  operational mass upper bound clamps this to `0.500000 kg`. Its corresponding
  weight is `4.905000 N`.
- With the configured category-independent friction lower bound `mu=0.1` and
  the current `5 N` per-finger limit, the best two-finger pure-friction capacity
  under that conservative contract is only
  `2 * 0.1 * 5 N = 1.000000 N`, below `4.905000 N`.
- Scientific interpretation: the current conservative mass/friction/force
  envelope cannot prove a pure-friction pinch lift for this frozen geometry.
  This is a capability-envelope limitation, not evidence for changing the
  grasp pose. Mass, friction, or force must not be tuned to this one target
  without independent measurement/calibration.
- Exact recorded displacements are:
  observation pregrasp to grasp `0.100000000 m`, approach to grasp
  `0.022862184 m`, and grasp to lift `0.033600000 m`. The last displacement is
  exactly `(0, 0, 0.0336) m`, confirming that this far-field preview contains
  a world-Z placeholder lift rather than the corrected live-support-normal
  contact lift.
- Consequently this frozen plan cannot be submitted as final contact
  authority or used to claim MuJoCo lift success. A fresh near-field RGB-D
  estimate and then-current joint state are required to build and validate the
  distinct `CONTACT_EXECUTION_PLAN`.

### 07:04 PDT offline regression closure

- Production-wiring regression now verifies that a non-world live support
  normal reaches the common contact sequence and that its lift delta equals
  `profile.lift_height_m * support_normal`, component by component.
- Remote streaming plus task-sequence tests pass `298/298`; five emitted
  messages are benign `rospy` deprecation warnings.
- Remote node, gripper geometry, tabletop geometry, and sequence tests pass
  `277/277`.
- MuJoCo client/server protocol tests pass `200`, with one expected skip.
- `py_compile` and `git diff --check` pass.
- The worktree contains extensive pre-existing/operator changes. None were
  discarded or rewritten outside the bounded grasp corrections and tests.
- This is the deterministic offline stopping boundary. The next unresolved
  evidence requires a new close-range real image and current real joint state,
  so work pauses here until the operator explicitly wakes the task.

### 2026-07-26 19:01 PDT full ROS runtime resumed at operator request

- The operator reported that the WSL protocol-v3 side was already started and
  explicitly requested immediate ROS-side bringup with joint enable, without
  pre-launch checks.
- Launched the current protocol-v3 worktree directly with:
  `roslaunch alicia_flexible_grasp_supervisor full_system.launch
  start_real_arm:=true driver_port:=/dev/alicia_arm
  driver_baudrate:=1000000 auto_torque_on_startup:=true
  self_check_poll_rate_hz:=0.0 start_camera:=true
  start_tactile:=false start_gui:=true use_remote_grasp6d:=true
  remote_grasp6d_url:=http://172.23.132.97:8000`.
- The supervised roslaunch terminal is session `56321`; ROS log run ID is
  `f682004c-895e-11f1-adda-5d2a5c05e9c9`.
- Direct startup evidence:
  - `/dev/alicia_arm` opened at `1000000` baud and the driver printed that
    `auto_torque_on_startup` is enabled and torque-on will be requested;
  - real SDK feedback is continuous with `status=0x00`, temperatures
    `27-28 C`, and gripper raw position `995`;
  - `alicia_controller` and `hand_controller` loaded and started;
  - the real camera started publishing color and depth;
  - the retained pruned-19 hand-eye TF is unchanged:
    xyz `[-0.0866127223,-0.0063407198,-0.1034120675] m`,
    q `[0.0229081321,-0.6787668594,0.0278470194,0.7334680031]`;
  - the remote server announced
    `backend=graspnet_baseline loaded=True protocol=3`;
  - MoveIt completed initialization and printed
    `You can start planning now!`;
  - GUI process PID `6396` is running from the current worktree;
  - electronic skin was not launched.
- Current 6D status is `TARGET_LOST` because no aligned target has yet been
  reported for this runtime. Candidate generation and task motion remain
  untriggered until the operator says `已对准`.
- No `/grasp/stop`, torque-off, disable, controller-stop, arm-stop, or
  stop-joint-enable command was published or called.

### 19:02 PDT live alignment supervision before candidate generation

- The operator used the GUI/manual joint-command path to align the camera. The
  assistant did not publish those joint targets.
- During the adjustment, real feedback moved from approximately the zero pose
  to `[-108.5,9.1,-2.0,0.0,-6.0,0.3] deg`, then stabilized with
  `status=0x00`.
- The target entered the real camera view and then remained stable over
  multiple observations near:
  - UV `(286-287,202-203)`;
  - depth `0.296-0.298 m`;
  - confidence `0.897-0.899`;
  - base position approximately `(-0.145,-0.483,0.063) m`;
  - instance-mask support approximately `4250-4450` pixels.
- The hardware stream emitted transient `E1/E2` status events and one
  single-channel anomalous temperature sample of `63 C`; the immediately
  following normal samples were approximately `29 C`. The driver explicitly
  logged that it treated the event as status telemetry and sent no torque-off.
  No conclusion beyond those recorded samples is inferred.
- Candidate generation remains untriggered. The assistant is waiting for the
  operator's explicit `已对准` report before beginning the planning-only 6D
  request.
- No `/grasp/stop`, torque-off, disable, controller-stop, arm-stop, or
  stop-joint-enable command was published or called.

### 19:03–19:10 PDT aligned-target far-field planning and execution

- After the operator explicitly reported `已对准`, started only continuous
  remote candidate inference.
- Live target evidence remained stable near UV `(286-287,202-203)`, depth
  `0.295-0.298 m`, and base position approximately
  `(-0.145,-0.483,0.064) m`.
- Request 4 produced the first fresh `PREVIEW_READY` plan
  `0101cdfd680951d18a12d8b4`: `12/12` stable, `8/24` hard-recheck pass,
  `8/8` strict MoveIt checked, and `1/8` reachable.
- Stopped only continuous inference to freeze that plan. An argument-free
  `/grasp/start` call returned `execute=false`; the custom service requires
  explicit `execute` and `plan_id`, so no motion occurred.
- The correct request was then rejected before motion by the persisted
  `CALIBRATION_INTERLOCK: HAND_EYE_UNVERIFIED_AFTER_CORRECTION_COLLISION`.
- The log records a prior explicit operator override on 2026-07-25 23:00 for
  temporary pruned-19 testing. The operator again explicitly requested
  automatic execution in this run. Restored only that temporary runtime
  override:
  `/grasp/calibration_interlock_active=false`, reason
  `USER_OVERRIDE_PRUNED19_TF_TEMPORARY_RENEWED_20260726_AUTOMATIC_EXECUTION`.
  The checked-in YAML remains interlocked and no independent calibration pass
  is claimed.
- The first exact-plan retry then returned `PLAN_STALE` before motion.
  Regenerated instead of reusing it.
- Request 11 produced new `PREVIEW_READY` plan
  `4672e492e197190c8c1eee4c`: `18` materialized, `12` stable, `8/24`
  hard-recheck pass, `8/8` strict MoveIt checked, and `2/8` reachable.
- Froze the candidate stream and called `/grasp/start` with
  `execute=true` and that exact plan ID.
- The task identified it as `FAR_FIELD_OBSERVATION_PLAN`, correctly deferred
  contact simulation, and executed only the fixed-100-mm observation stage.
  Strict execution was retimed to `41.791 s`, maximum joint delta
  `2.090 rad`, and returned `SUCCEEDED`.
- The final real joint feedback was approximately
  `[-110.1,-59.2,115.9,-7.7,-84.6,15.3] deg`, with normal feedback
  `status=0x00`. Measured observation endpoint residual was
  `20.3 mm / 2.86 deg`.
- No contact approach, gripper close, lift, `/grasp/stop`, torque-off,
  disable, controller-stop, arm-stop, or stop-joint-enable command occurred.

### 19:10–19:13 PDT near-field contact plan failed closed

- At the reached observation view, the real target remained detected around
  UV `(336-338,386-389)`, depth `0.161-0.163 m`, base position approximately
  `(-0.150,-0.497,0.088-0.089) m`, and confidence roughly `0.70-0.85`.
- The detection box repeatedly ended around row `460-462` of the 480-row
  image. It was close to the lower boundary but not accepted as authority by
  itself; the candidate and contact gates remained active.
- The task automatically entered near-field phase and requested fresh RGB-D
  plans. No far-field contact poses were reused.
- Representative complete near-field batches:
  - request 18: `18` materialized, `12` stable, `4/24` hard-recheck pass,
    `4/4` strict sequence checked, `0/4` reachable;
  - request 20: `12/12` stable, `10/24` hard-recheck pass,
    `10/10` strict sequence checked, `0/10` reachable.
  Every checked sequence failed at `pregrasp`, before any motion.
- The exact bilateral CAD contact gate also rejected near-threshold proposals
  whose continuous overlap was `6.142-6.452 mm`, below the required
  `6.500 mm`. The gate was not relaxed.
- After `150.0 s`, no `CONTACT_EXECUTION_PLAN` had been produced. The task
  failed closed with:
  `NEAR_FIELD_REPLAN_TIMEOUT: no fresh near-field 6D preview after 150.0s`.
- The blocking start service returned `success=False, message=failed` and the
  execution slot was released. Stopped only the remaining continuous
  inference request stream afterward.
- The arm remains enabled at the observation pose. No second-stage pregrasp,
  approach, close, lift, `/grasp/stop`, torque-off, disable, controller-stop,
  arm-stop, or stop-joint-enable command occurred.
- Next work is read-only diagnosis of the preserved near-field audits and
  candidate/phase code. Another real motion must not be requested until the
  no-reachable-contact-plan cause is explained without object-specific
  coordinates or relaxed physical gates.

### 19:18 PDT operator video evidence and continued diagnosis

- The operator supplied the 85.17-second real-execution video
  `/home/zhuyupei/Videos/7c91d709364d27c37143ec8787ede7ad.mp4`.
- The operator's direct observation is that the first-stage fixed-100-mm
  observation motion reached the target with only a small image-centering
  offset, while the second stage did not execute for a long time.
- This agrees with the already-recorded runtime phase sequence: the first
  stage completed, then the task waited for a fresh near-field
  `CONTACT_EXECUTION_PLAN`; all complete near-field batches had zero reachable
  strict sequences and the task timed out before issuing a second-stage motion.
- The video is being inspected as additional motion evidence. No conclusion
  about the exact geometric cause is made from the operator description alone.
- Current work remains generalized: no carton-specific coordinate, object
  label, fixed second-stage pose, contact-force assumption, or relaxed physical
  gate will be introduced. The only permitted fixed task geometry remains the
  agreed 100-mm first observation standoff.
- No `/grasp/stop`, torque-off, disable, controller-stop, arm-stop, or
  stop-joint-enable command was published or called.

### 19:24 PDT video/runtime correlation and endpoint-uncertainty audit

- Inspected the 85.17-second HEVC video at 10-second intervals and more densely
  at 3-second intervals over the first-stage transition. The arm completes the
  visible observation motion during the first part of the recording and then
  remains at the reached view through the end; there is no visible second-stage
  approach trajectory.
- This distinguishes a planning wait from a controller execution delay. ROS
  contains no second-stage execute request: all near-field strict checks failed
  at `pregrasp`, and the task later emitted
  `NEAR_FIELD_REPLAN_TIMEOUT`.
- Traced the first-stage measured endpoint consumer. While the task is active,
  the remote near-field node reads the fresh scalar
  `position_error_m=0.0202816251` from
  `/grasp_6d/runtime_execution_error` and supplies it to the adaptive-stage
  equations. It does not translate a target or use an object label.
- The scalar is deliberately treated as an isotropic uncertainty bound. The
  adaptive equations add it to perception uncertainty when computing approach,
  then reserve it again in the minimum pregrasp-to-approach gap. This protects
  clearance but cannot represent the measured residual direction.
- A reproducible counterfactual using the immediately following request-28
  close-range geometry (same reached view, object height
  `18.4197426 mm`, perception uncertainty `4.0 mm`) gives:
  - with execution error inactive: approach `22.4197425 mm`, support-normal
    pregrasp `34.4197425 mm`, lift `34.0 mm`;
  - with the task's measured `20.2816251 mm` error: approach
    `42.7013676 mm`, support-normal pregrasp `67.9829927 mm`, lift
    `54.2816250 mm`.
- Therefore the fresh execution residual can enlarge the representative
  support-normal pregrasp by about `33.56 mm`. That is a credible contributor
  to a workspace-boundary IK failure, but it is not yet claimed as the sole
  cause: request 20's exact stage poses were overwritten when the
  `*_latest.json` audit advanced to request 28.
- The latest-only audit overwrite is itself a diagnostic defect: it prevents
  exact post-run reproduction of an earlier complete near-field request.
  Per-request immutable audit retention is required before changing the
  physical equations from this evidence.
- Current read-only feedback shows the arm enabled and stationary at a
  different aligned view, approximately joints
  `[-105.3,18.1,-8.0,0.3,1.9,0.3] deg` and
  `base_link->tool0=(-0.077,-0.282,0.164) m`. No task or assistant-issued
  trajectory explains or is attributed to this position; the task remains
  inactive after the recorded timeout.
- No force-feedback/electronic-skin input is used. No `/grasp/stop`,
  torque-off, disable, controller-stop, arm-stop, or stop-joint-enable command
  was published or called.

### 19:28 PDT generalized observation-convergence decision and runtime pause

- Chosen correction direction: do not lower the contact-overlap gate, do not
  remove measured execution uncertainty, and do not add an object/workspace
  special case.
- Keep the first observation target exactly as the live 6D candidate pose with
  the agreed fixed 100-mm standoff. After the first strict execution, compare
  measured `tool0` with that same requested observation pose using the existing
  object-independent `6 mm / 5 deg` accuracy contract.
- If it is outside the contract, allow one bounded strict correction to the
  exact same observation pose. This is not a fixed carton target and does not
  alter the live-derived pose. If the correction still does not converge, fail
  closed with an observation-endpoint error before requesting near-field
  candidates.
- This prevents a large free-space tracking residual from being converted into
  an isotropically enlarged near-field family that waits for 150 seconds at the
  workspace boundary. It retains the conservative uncertainty equations for
  any residual that remains after convergence.
- Extend `/grasp_6d/runtime_execution_error` with requested/actual position,
  requested/actual quaternion, and signed position-error vector. This supplies
  directional evidence without using that vector to translate a grasp.
- At 19:28 the live driver began reporting stale hardware feedback and
  automatically paused its SDK command stream; feedback age exceeded 220
  seconds. Perception simultaneously reported `TARGET_LOST`. No new trajectory
  will be attempted while feedback is stale. Offline implementation and tests
  can continue.
- The driver pause is not a torque-off or stop-enable command, and the
  assistant issued no `/grasp/stop`, torque-off, disable, controller-stop,
  arm-stop, or stop-joint-enable command.

### 19:32 PDT bounded observation-endpoint convergence implemented

Implementation:

- Added `grasp/observation_endpoint_correction_attempts: 1`. The accepted range
  is the strict integer interval `[0,2]`; production uses exactly one bounded
  retry.
- After a newly executed far-field observation, the task now evaluates the
  actual `base_link -> tool0` endpoint against the same configured
  `6 mm / 5 deg` contract used before contact.
- If outside the contract, it uses the existing strict plan-and-execute path to
  retry the exact same immutable live-derived observation pose. It does not
  translate the target, alter the 100-mm observation distance, replace the
  wrist orientation, or invoke a position/orientation fallback.
- If the retry still lies outside the accuracy contract, the task fails closed
  with `OBSERVATION_ENDPOINT_NOT_CONVERGED` before requesting a near-field
  plan.
- A deliberately reused observation view now records a fresh zero-motion
  endpoint sample for the new active task. This prevents a recent prior task's
  scalar residual from being mistaken for an execution error in a new reused
  view.
- Extended `/grasp_6d/runtime_execution_error` with:
  `requested_position_m`, `actual_position_m`,
  `position_error_vector_m`, `requested_quaternion_xyzw`, and
  `actual_quaternion_xyzw`. The existing scalar fields and consumer remain
  compatible. The vector is diagnostic evidence only and does not retarget a
  grasp.

Verification:

- Full task-sequence regression passed `117/117`.
- Default production-config regression passed `6/6`.
- Python compilation passed.
- `git diff --check` passed for the changed task/config/test files.
- Added regressions proving one identical-pose retry can converge and that a
  persistent residual performs only one retry before failing closed.
- No ROS node was restarted and no real command was sent because hardware
  feedback remains stale. No force feedback, `/grasp/stop`, torque-off,
  disable, controller-stop, arm-stop, or stop-joint-enable command was used.

### 19:34 PDT hardware feedback restored and task-node hot-load prepared

- Real hardware feedback resumed without an assistant command. Current samples
  are continuous around joints
  `[-108.3,19.8,-8.0,0.2,-7.5,0.3] deg`, gripper raw `995`,
  `status=0x00`, and maximum temperature about `36 C`.
- Target detection is again stable near UV `(316,236)`, depth
  `0.313-0.315 m`, base position approximately
  `(-0.152,-0.454,0.063-0.065) m`, confidence about `0.90`. This is a
  substantially more centered far-field view than the prior close observation
  at approximately `(337,387)`.
- The old task node is inactive with its last state
  `FAILED / execution slot released: failed`; no execution action is in
  flight.
- The correction is Python source and requires only `grasp_task_node` to be
  hot-restarted. Driver, controllers, MoveIt, camera, perception, remote
  candidate node, hand-eye publisher, GUI, and joint-enable path will remain
  running.
- Source inspection found no shutdown hook in the task node. The restart will
  not call its `/grasp/stop` service and will not send a robot stop/disable
  command.

### 19:37 PDT corrected task node hot-loaded; candidate generation resumed

- The previous `grasp_task_node` process (`PID 6383`) was terminated cleanly
  with `SIGTERM` only so that the tested Python correction could be loaded.
  `roslaunch` reported a normal process finish. This did not invoke the task's
  `/grasp/stop` service and did not publish a robot stop, torque-off, disable,
  controller-stop, arm-stop, or stop-joint-enable command.
- The corrected task node is now running as `PID 9923` and reported
  `GraspTaskNode ready` followed by `[Grasp] IDLE ready`. The complete original
  ROS launch remains alive as `PID 6299`; the real-arm driver, controllers,
  MoveIt, camera, perception, remote 6D node, pruned19 hand-eye TF publisher,
  GUI, and joint-enable path were not restarted.
- The uploaded video and ROS event timing establish that the prior second
  stage was not sent to the controller: first-observation motion completed,
  then the task waited for a near-field contact plan while every strict
  candidate failed first-stage `pregrasp` reachability. The new bounded
  endpoint-convergence gate addresses the measured observation residual before
  any new near-field request; it does not weaken IK, collision, sequence,
  contact-overlap, or digital-twin gates.
- With the user's target still aligned and live detection centered, the next
  action is to resume continuous remote 6D inference. A fresh plan ID will be
  frozen before `/grasp/start`; no prior or overwritten candidate will be
  executed.

### 19:38 PDT fresh far-field 6D plan frozen for execution

- `/grasp_6d/request_plan trigger=true` succeeded and started continuous
  remote inference. Live target evidence remained stable near UV `(316,235)`,
  depth `0.312-0.315 m`, base position approximately
  `(-0.152,-0.454,0.063-0.065) m`, confidence about `0.90`.
- Requests 29 and 30 remained `STABILITY_PENDING`. Request 31 completed with
  `status=PREVIEW_READY`: 1024 raw candidates, 49 after NMS, 17 after remote
  collision, 6 locally valid/tabletop/stable, 4 after hard recheck and soft
  ranking, 4 strict MoveIt checks, 1 reachable, 1 selected/promoted/previewed.
- The corrected task node accepted the new rich execution plan
  `e74ac1b60a1293463da6d984`, with source snapshot
  `1785119913451879739 ns`. This is a new plan from the current aligned target,
  not the earlier far-field plan and not an overwritten latest-audit guess.
- Continuous inference was then frozen successfully with
  `/grasp_6d/request_plan trigger=false` so the exact accepted plan cannot be
  replaced during execution. This stops candidate generation only; it does not
  stop the arm, controller, torque, or joint enable.
- Intermittent isolated SDK `E1/E2` status events and isolated anomalous
  temperature samples were followed by normal feedback around `37 C` and
  `status=0x00`. The driver explicitly logged that it sent no torque-off.

### 19:40 PDT first live endpoint-convergence trial failed closed

- `/grasp/start` was called with `execute=true` and the exact frozen plan ID
  `e74ac1b60a1293463da6d984`. The task accepted the far-field observation
  phase, skipped redundant gripper opening because measured opening was already
  within `1 mm` of `50 mm`, and sent the strict observation pose
  `(-0.155,-0.436,0.151) m`,
  quaternion `(0.784,0.595,-0.111,0.136)`.
- The first observation trajectory was retimed to `36.141 s`, with
  `max_delta=1.807 rad`, `velocity_limit=0.080 rad/s`, and completed
  `SUCCEEDED`. After `0.90 s` settling, the new endpoint audit measured
  `20.7 mm / 3.03 deg`, outside the unchanged `6 mm / 5 deg` contract.
- The new generalized gate therefore performed its one permitted correction to
  the exact same immutable live-derived pose. Strict replanning succeeded with
  `joint_path_cost=0.041 rad` and `joint_max_delta=0.031 rad`; the execution
  path was retimed to `1.200 s` with `max_delta=0.060 rad`.
- The correction execution was rejected by the controller with
  `GOAL_TOLERANCE_VIOLATED`, specifically Joint2 goal error
  `0.053877 rad`. The task returned `success=false`, entered `FAILED`, and
  released the execution slot. No near-field request, second-stage motion,
  approach, gripper close, lift, or contact motion was executed.
- This is new measured evidence: the first observation's large path can
  complete while the short same-pose correction is too brief for the measured
  actuator/controller tracking response. The next diagnosis is restricted to
  the generic strict-trajectory timing/controller path; the 10-cm observation
  geometry, live target pose, pruned19 TF, candidate contract, collision gates,
  and contact-overlap threshold will not be altered.

### 19:50 PDT evidence-backed minimum timing added for short strict paths

Additional evidence and corrected interpretation:

- Historical real-arm logs from the same strict retiming implementation show
  successful short paths at `2.777 s / 0.139 rad`,
  `3.470 s / 0.174 rad`, `4.180 s / 0.209 rad`, and
  `6.045 s / 0.302 rad`. The new correction is the only recorded strict path
  at `1.200 s / 0.060 rad`, and it failed with Joint2 endpoint error
  `0.053877 rad`.
- The failed controller goal actually remained active for the configured
  `8 s` goal window after the `1.2 s` path, but Joint2 stayed about
  `0.0539 rad` outside the goal. Widening the endpoint or controller tolerance
  would therefore conceal a real tracking error and was rejected.
- About `110 s` after the task had already failed and released its execution
  slot, a new direct `/joint_commands` stream moved the hardware from the
  observation posture back toward a far-field/aligned posture. There is no
  corresponding task or MoveIt execute-service request in
  `motion_gateway-7.log`. `/joint_commands` currently has three possible
  publishers (`bessica_d_hw_interface`, `motion_gateway`, and the GUI), so the
  historical message source cannot be uniquely attributed. The later
  `base_link -> tool0` change is therefore real post-task joint motion, not a
  contradictory TF result and not evidence that the pruned19 transform
  changed.
- Current live feedback after that external/direct motion is stable around
  `[-111.9,29.0,-6.3,-6.6,-18.9,7.4] deg`. Independent TF and MoveIt
  `/compute_fk` agree at approximately
  `(-0.0908,-0.2436,0.1722) m`; the target is again visible near UV
  `(300,241)`, depth `0.338 m`, and base position about
  `(-0.153,-0.454,0.067) m`.

Implementation:

- Added the object-independent robot parameter
  `robot/strict_execution_min_duration_sec: 3.0`.
- After MoveIt retimes the exact cached strict path, a path shorter than this
  evidence-backed floor is deep-copied and stretched uniformly in time.
  Every joint path position remains byte-for-byte numerically identical;
  `time_from_start` is multiplied by the common scale, velocities are divided
  by it, and accelerations are divided by its square.
- The existing checks still reject a non-finite time scale, missing/invalid
  point timing, any changed joint-path geometry, a stale start state, or a
  trajectory whose stretched duration remains below the independent
  `max_delta / 0.08 rad/s` hardware-following minimum.
- This changes no object label, object coordinate, grasp pose, orientation,
  100-mm observation standoff, collision envelope, contact-overlap contract,
  IK gate, controller goal tolerance, hand-eye TF, or force-feedback policy.

Verification:

- Focused planner tests passed `31/31`, including a new regression that turns
  the measured `1.2 s` case into `3.0 s` with `time_scale=2.5`, preserves all
  joint positions, and scales velocity/acceleration consistently.
- Trajectory configuration tests passed `6/6`.
- Combined grasp task, motion-gateway controller, planner, and trajectory
  regression suite passed `175/175` with five existing `rospy` deprecation
  warnings.
- Python compilation and `git diff --check` passed.
- No real command was issued while implementing or testing this change. No
  `/grasp/stop`, torque-off, disable, controller-stop, arm-stop, or
  stop-joint-enable command was used.

### 19:51 PDT corrected motion gateway hot-loaded

- The failed task was inactive (`active=false`, `FAILED`, execution slot
  released). Runtime parameter
  `/robot/strict_execution_min_duration_sec=3.0` was installed.
- The previous motion gateway (`PID 6368`) had no shutdown hook and was
  terminated cleanly with `SIGTERM` only to load the tested planner timing
  correction. It did not call `/grasp/stop` and did not issue a torque-off,
  disable, controller-stop, arm-stop, or stop-joint-enable command.
- A corrected standalone `motion_gateway` is now running and reported
  `MotionGateway ready: commands -> /joint_commands`. The driver, real
  controllers, MoveIt, camera, perception, remote 6D node, hand-eye publisher,
  GUI, task node, and joint-enable path remained running.
- The next inference cycle must create a fresh plan from the current image and
  target epoch. The previously failed plan
  `e74ac1b60a1293463da6d984` will not be reused.

### 19:59 PDT gateway respawn correction and continued live candidate evidence

- Correction to the 19:51 runtime wording: the temporary standalone gateway
  first registered successfully, but the parent `roslaunch` then respawned the
  same node name and ROS shut down the duplicate standalone process. The
  corrected runtime gateway is `PID 12188`, parent launch `PID 6299`, and it
  reports `MotionGateway ready: commands -> /joint_commands`. This was normal
  ROS same-name arbitration, not an arm or controller stop.
- The uploaded first-stage video agrees with the ROS event sequence: the arm
  completed the observation movement and then stayed still. No second-stage
  trajectory had been submitted. The wait was candidate gating/planning, not
  a submitted trajectory being ignored by the controller.
- Continuous inference remained active and issued no arm command. Requests
  `67`, `70`, and `71` had respectively `12`, `18`, and `20` returned
  GraspNet candidates, but none passed the unchanged local gripper/contact
  contract. Their controlled-gate summaries had zero baseline-safe candidates;
  the measured finger-clearance distributions were predominantly negative.
- Requests `68` and `69` independently produced six valid generalized
  tabletop-geometry candidates from the current live mask and depth geometry.
  All six passed materialization and the mandatory local safety contract, but
  they had not yet accumulated the required `3` hits in the immutable
  `5`-frame stability window, so the exact result was
  `STABILITY_PENDING`; no preview or executable plan was published.
- The current target evidence remained stable near UV `(300,241)`, depth
  `0.337-0.340 m`, base position about
  `(-0.153,-0.454,0.067-0.069) m`, confidence about `0.907`, with roughly
  `3.2-3.4k` instance-mask points. Hardware feedback remained around
  `[-111.9,29.0,-6.3,-6.6,-18.9,7.4] deg`, gripper raw `995`,
  `status=0x00`, and maximum reported temperature `38-39 C`.
- No threshold, object coordinate, object label, grasp pose, 100-mm
  observation standoff, pruned19 transform, force policy, or controller
  tolerance was changed. The inference stream will continue until a candidate
  independently satisfies stability, strict MoveIt reachability, and preview
  publication. No `/grasp/stop`, torque-off, disable, controller-stop,
  arm-stop, or stop-joint-enable command was issued.

### 20:05 PDT contact-boundary evidence instrumentation

- Requests through `109` continued without arm motion. Generalized tabletop
  candidates appeared intermittently (`68`, `69`, `75`, `89`, `91`, and
  `104` are observed examples), but no five-request window accumulated the
  required three matching hits. GraspNet candidates continued to fail the
  existing local support/contact contract.
- A live `/grasp_6d/object_geometry` sample confirmed that each request already
  uses five-frame RGB-D fusion. The measured object geometry was valid with
  `538` object points, `99.6%` valid target depth, about `19.6 mm` estimated
  support-normal height, `9.4 mm` fused depth MAD, and `0.666` support-plane
  inlier ratio. Thus the observed candidate flicker cannot be described as a
  single-frame input path.
- The audit previously recorded only `maximum_safe_tilt_deg=null` when a
  branch could not reach the fixed `6.5 mm` bilateral CAD/contact overlap. It
  did not preserve how much overlap was actually observed, so changing any
  geometry or threshold from that evidence would be guesswork.
- Added audit-only fields to every tabletop contact branch:
  `required_contact_patch_overlap_m`, `sample_count`,
  `maximum_observed_contact_patch_overlap_m`, and
  `maximum_observed_tilt_deg`. Candidate generation, sampled poses, branch
  ordering, the `6.5 mm` threshold, and all motion behavior are unchanged.
- The focused live-boundary and bounded-strata tests passed `2/2`
  (`181` unrelated tests deselected). Python compilation and
  `git diff --check` also passed. The first attempted shell form could not find
  a `pytest` executable; running the installed module after sourcing the ROS
  workspace succeeded.
- The corrected remote planner must now be hot-loaded so subsequent immutable
  audit reports contain the missing quantitative evidence. This restart is
  planning-only and must not call `/grasp/stop`, torque-off, disable,
  controller-stop, arm-stop, or stop-joint-enable.

### 20:10 PDT 6.5-mm fixed contact threshold identified as invalid coupling

- The corrected remote planner was started manually after the parent launch
  did not respawn it. Continuous inference was explicitly resumed. The driver,
  controller, joint enable, task node, camera, perception, GUI, and arm posture
  remained unchanged; no arm command was sent.
- New per-branch evidence measured failed-frame maximum continuous bilateral
  contact overlaps including `3.735`, `4.397`, `4.736`, `4.978`, and
  `5.745 mm`. Other isolated frames crossed the threshold and materialized
  six or twelve candidates, but they did not repeat in three of five frames.
- The exact source of the `6.500 mm` requirement is now confirmed in code:
  `measured_endpoint_position_tolerance_m` (`6.0 mm`) was added to
  `GRIPPER_CONTRACT_TOLERANCE_M` (`0.5 mm`) and stored as
  `minimum_contact_patch_overlap_m`.
- This is an invalid semantic coupling. A Cartesian endpoint-error allowance
  is not a physical minimum finger/object contact length. Although the value
  is global rather than paper-box-specific, it is still a fixed contact rule
  and therefore cannot be claimed to generalize scientifically across object
  heights, shapes, materials, depth uncertainties, or available finger-patch
  coverage.
- The fixed Alicia-D CAD dimensions and measured calibration tolerances are
  legitimate hardware facts. The `6 mm + 0.5 mm` contact-length construction
  is not. It must not be lowered to another hand-picked fixed number merely to
  pass the current object.
- The immediate phase-correct repair is to remove contact-length authority
  from the `FAR_FIELD_OBSERVATION_PLAN`: only the live-derived 100-mm
  observation pose is executed in that phase, and task code already prohibits
  that phase from reaching approach/contact. A fresh
  `CONTACT_EXECUTION_PLAN` remains mandatory after the near-field image.
- For the near-field contact phase, replace the absolute fixed overlap gate
  with an uncertainty-aware, dimensionless live-geometry assessment and
  retain cross-frame repeatability, CAD collision, width, IK, and MoveIt
  gates. Without calibrated material/friction/force evidence, no absolute
  image-only contact length can honestly guarantee holding force; overlap
  should be treated as live coverage/confidence evidence rather than a
  universal object-independent constant.

### 20:24 PDT far-field contact-authority separation implemented

- Implemented the first phase-correct repair in
  `remote_grasp6d_node.py`. Tabletop and GraspNet candidate geometry checks
  now receive the immutable request phase. A
  `FAR_FIELD_OBSERVATION_PLAN` has no contact-execution authority, so its
  absolute contact-patch requirement is `0.0 m`, explicitly meaning
  "not applicable in this phase", not a new grasp-quality threshold.
- The far-field path still requires a live target, valid fused geometry,
  bilateral tabletop support evidence, gripper width fit, CAD support and
  swept-volume checks, target association, temporal stability, IK, and the
  strict MoveIt observation-stage check. Only the semantically invalid
  requirement that a hypothetical contact already satisfy an absolute length
  before moving to the 100-mm observation view was removed.
- The task executor remains fail-closed: a far-field plan can submit only the
  observation pose; after reaching it, the task requires a fresh plan whose
  diagnostic is exactly `CONTACT_EXECUTION_PLAN`. An observation plan cannot
  submit approach, grasp, or lift.
- Added phase evidence to generation diagnostics:
  `plan_phase`, `contact_execution_gate_deferred`,
  `required_contact_patch_overlap_m`, and
  `contact_overlap_requirement_source`. These make it auditable that contact
  authority was deferred rather than silently weakened.
- Added a regression test using a deliberately impossible `20 mm` absolute
  overlap requirement against an `11 mm` live object span. The contact phase
  correctly produces no candidate, while the observation phase still derives
  candidates from the same live geometry and records a zero/not-applicable
  requirement. Together with the existing live-boundary and stratification
  tests, `4/4` focused tests passed (`180` unrelated tests deselected).
  Python compilation and `git diff --check` also passed.
- No ROS motion, gripper, joint, torque, disable, stop, or enable-changing
  command was issued during this code change. The 100-mm observation
  standoff, pruned19 transform, target coordinates, object labels, force
  policy, controller tolerance, and near-field execution authority were not
  changed.

### 20:27 PDT corrected planner regression, hot-load, and fresh plan

- The first full streaming-test run exposed one pre-existing test double whose
  replacement method did not accept the new
  `contact_execution_phase` keyword. Runtime code compiled; the failure was a
  test-interface mismatch. The test double was updated to assert the default
  contact phase, and the complete file then passed `184/184`.
- The old standalone planning-only node (`PID 13340`) was terminated with its
  terminal interrupt and the corrected planning node was started in terminal
  session `52215`. The hardware driver, controller, joint enable, motion
  gateway, task node, camera, perception, GUI, and arm pose were unchanged.
  The replacement reported the remote GraspNet backend online.
- Continuous inference was restarted. With the task inactive, immutable
  requests `1`, `2`, and `3` produced respectively `18`, `12`, and `18`
  locally valid tabletop observation candidates, versus the previous
  intermittent zero-candidate frames caused by the `6.5 mm` gate. Request `3`
  reached temporal stability; its four shortlisted observation variants were
  strictly unreachable.
- Requests `4` and `5` then each produced three strictly MoveIt-reachable
  observation candidates. The reachable live-derived observation poses were
  around `(-0.151 to -0.160, -0.423, 0.149) m`; accepted branches had
  `joint_max_delta` about `1.216-1.285 rad`, below the unchanged `2.150 rad`
  limit. Both requests reached `PREVIEW_READY`.
- Execution authority was committed to fresh plan
  `c63e086e3e28b0772187d58b`, source stamp
  `1785122374663158178`, diagnostic
  `FAR_FIELD_OBSERVATION_PLAN`, candidate source
  `tabletop_geometry`, required live width `42.678 mm`, and observation pose
  `(-0.151409,-0.423032,0.149280) m`. The task node independently reported
  `validation=VALID` for the same plan ID.
- The later preview `ae039826a5c23be6141150d2` was not promoted over the
  already committed plan, consistent with selection hysteresis. Continuous
  inference was frozen using `/grasp_6d/request_plan trigger:false` so the
  exact validated authority cannot be replaced before start. This service
  changes planning-stream state only; it is not `/grasp/stop` and sends no arm
  or joint command.
- Latest joint feedback remained near
  `[-111.90,29.00,-6.33,-6.59,-18.90,7.38] deg`, with the gripper open at
  `49.75 mm`. No arm motion occurred during inference or plan binding.

### 20:29 PDT stale start rejected before motion

- `/grasp/start` was called with the exactly bound plan
  `c63e086e3e28b0772187d58b`. Between the last read-only validation and the
  start transaction, the source image crossed the configured plan-validity
  window. The task node rejected it with `PLAN_STALE` before preflight,
  gripper, or arm execution.
- The stale plan will not be reused and freshness will not be bypassed.
  Continuous planning must be resumed for a new image-bound plan ID, then
  frozen and started immediately. No arm, gripper, joint, torque, enable,
  disable, or stop command was produced by the rejected transaction.

### 20:35 PDT fresh observation run and actuator endpoint evidence

- Fresh far-field authority was regenerated from target epoch `3`, frozen,
  and accepted as plan `fa8e5b17db5d255a0d831b4d`. The observation pose was
  `(-0.158048,-0.420189,0.148562) m`. `/grasp/start` accepted it while fresh.
- The strict observation trajectory ran for `24.625 s` with
  `max_delta=1.231 rad`, `start_error=0.001534 rad`, and the unchanged
  `0.080 rad/s` limit. The controller completed it successfully. Hardware
  feedback stopped near
  `[-2.035593,-0.590583,1.090660,-0.360485,-0.855961,0.549165] rad`.
- The measured tool endpoint residual was `18.4 mm / 2.60 deg`, outside the
  unchanged `6 mm / 5 deg` bound, so the one permitted same-pose observation
  correction was planned. Its joint path was small:
  `joint_path_cost=0.034 rad`, `joint_max_delta=0.025 rad` before execution
  re-timing. The execution-time retimer produced an exact-path `3.000 s`
  trajectory with reported `max_delta=0.050 rad`, `start_error=0`, and
  `time_scale=2.978`.
- The controller rejected that correction after its `8 s` goal-time allowance:
  `GOAL_TOLERANCE_VIOLATED: Joint2 goal error 0.048842 rad`. The live
  controller state preserves the complete endpoint evidence:
  desired Joint2 `-0.541741 rad`, actual `-0.590583 rad`, error
  `+0.048842 rad`; all other joint errors were at most `0.020574 rad`.
  This is a smaller residual than the prior `0.053877 rad` failure but still
  above the unchanged `0.035 rad` controller goal tolerance.
- Driver command-flow evidence identifies a transport/actuator integration
  issue rather than a 6D target-coordinate issue. During the correction the
  driver received successive Joint2 targets of about `-32.3`, `-31.7`, and
  `-31.1 deg`, while real feedback remained about `-33.8 deg`. Once the raw
  final SDK frame stopped changing, no more `Streaming SDK command` records
  appeared for the rest of the controller goal-time window.
- Runtime driver parameters confirm
  `suppress_redundant_commands=true` together with
  `command_keepalive_rate_hz=0.0`. Therefore the final small endpoint target
  was not periodically reasserted after the last distinct SDK frame. The
  hardware remained roughly `2.8 deg` away even though the ROS controller
  continued to hold that desired endpoint.
- This evidence rejects another arbitrary object-space correction, relaxed
  Cartesian tolerance, or paper-box-specific offset. The next implementation
  target is the generalized actuator contract: retain redundant-frame
  suppression during motion, but periodically reassert the unchanged final
  hardware setpoint while a controller goal is holding. Its cadence must be
  tied to existing controller/driver timing evidence and verified in driver
  tests before another real execution.
- The task failed closed after the correction. It did not request a near-field
  plan and submitted no approach, grasp, lift, or contact motion.

### 20:38 PDT user reset and post-reset read-only state

- The user reported resetting the arm. At that time the task execution slot
  was already inactive (`FAILED`, `active=false`) and the remote planning
  stream was stopped. No automatic motion could start during the reset.
- A read-only sample still reported the last observation joint posture above,
  open gripper `49.75 mm`, `/alicia_d/motion_enabled=true`,
  run status `225`, and temperatures
  `[39,38,55,44,38,42,36,33,35,38] C`. The old plan was independently
  `PLAN_STALE` and cannot be executed.
- No `/grasp/stop`, torque-off, disable, controller-stop, arm-stop, joint
  command, or new trajectory was issued in response to the reset.

### 20:43 PDT feedback-cadenced SDK setpoint keepalive implemented offline

- Implemented a generalized driver transport correction. Redundant SDK frames
  remain suppressed, but an unchanged held joint/gripper setpoint is now
  reasserted at `10 Hz`, equal to the configured real hardware feedback poll
  rate and below the existing `50 Hz` command timer. This is a controller/
  actuator liveness contract; it does not depend on object label, pose,
  dimensions, target coordinates, or grasp stage.
- Updated both `alicia_d_driver.launch` and
  `alicia_d_bringup.launch` from
  `command_keepalive_rate_hz=0.0` to `10.0`. Updated the direct-node C++
  default and header default to the same value so `rosrun` and launch startup
  cannot silently diverge. The loaded value is clamped to
  `[0, command_rate_hz]`.
- Added regression coverage that parses both real-arm launch files and
  requires the keepalive to be positive, at least the real-feedback cadence,
  and no greater than the command timer. A second test binds the C++ direct
  startup default and its upper clamp.
- The MoveIt execution/configuration suite passed `39/39`. The real
  `alicia_d_driver_node` C++ target rebuilt successfully, and
  `git diff --check` passed.
- The rebuilt binary has **not** been hot-loaded. Restarting the serial driver
  while the user is resetting the arm would change the hardware connection
  and enable lifecycle, so real runtime remains on the old process until the
  user completes the reset and confirms the next run. No hardware command was
  issued by compilation or tests.

### 20:56 PDT user re-aligned; corrected driver hot-loaded

- The user reported the reset complete and the target re-aligned. The exact
  old driver was `PID 6347`; private runtime parameters confirmed
  `/dev/alicia_arm`, `1000000` baud,
  `auto_torque_on_startup=true`, and the old keepalive `0.0`.
- Installed runtime parameter
  `/alicia_d_driver_node/command_keepalive_rate_hz=10.0`, terminated only the
  old serial driver with `SIGTERM`, and started the rebuilt
  `alicia_d_driver_node` in terminal session `31521`. Its destructor only
  disconnected the serial port; no torque-off, disable, controller-stop,
  arm-stop, or `/grasp/stop` command was called.
- The replacement opened `/dev/alicia_arm` at `1000000` baud and, using the
  existing explicit startup setting, requested torque-on/joint enable. Real
  feedback immediately reported the reset posture near
  `[-114.9,24.7,-4.0,-8.8,-18.9,3.0] deg`, gripper raw `995`,
  status `0x00`. `/alicia_d/motion_enabled=true` and the live driver parameter
  is `10.0 Hz`.
- The hardware interface, trajectory controllers, MoveIt, motion gateway,
  camera, perception, task node, GUI, pruned19 transform, and remote planner
  were not restarted. Continuous 6D inference was resumed from the user's
  newly aligned live image. No arm trajectory or gripper position command was
  issued during the driver replacement.

### Post-20:56 PDT aligned run: keepalive falsified the missing-final-frame hypothesis

- After the user reported the target aligned, the planner produced fresh
  far-field plan `a0520e6a38b83dff0ada75a6` from live source stamp
  `1785123442812506437`, phase `FAR_FIELD_OBSERVATION_PLAN`, target epoch `9`.
  Its first observation pose was live-derived as
  `(-0.158676,-0.415346,0.147055) m`; only the already agreed first
  observation standoff is fixed at `0.100 m`. The plan was current and valid
  when `/grasp/start` accepted it.
- The full first observation trajectory completed successfully:
  `22.076 s`, maximum joint displacement `1.104 rad`, velocity limit
  `0.080 rad/s`, and zero reported start error. After `0.90 s` measured
  settling, the actual end-effector residual was `18.2 mm / 2.62 deg`.
  Because position exceeded the unchanged `6.0 mm` observation endpoint
  tolerance, the task requested its single strict retry of the **same**
  live-derived pose; it did not apply an object-space offset.
- The correction plan was small and internally consistent:
  joint path cost `0.034 rad`, maximum joint delta `0.023 rad`; it was retimed
  to the configured `3.000 s` minimum with final maximum delta `0.052 rad`.
  The controller then used its `8 s` goal window and aborted at ROS stamp
  `1785123561.007` with
  `GOAL_TOLERANCE_VIOLATED: Joint2 goal error 0.050138`.
- The final controller sample was desired
  `[-2.047741,-0.500561,1.021312,-0.394620,-0.775822,0.549868] rad`,
  actual
  `[-2.038660,-0.550699,1.000155,-0.389631,-0.776194,0.543029] rad`.
  Thus the errors were approximately
  `[-0.009080,+0.050138,+0.021157,-0.004988,+0.000372,+0.006839] rad`;
  Joint2 alone was `2.873 deg` away and exceeded the `0.035 rad` effective
  hardware goal tolerance.
- The new driver keepalive was directly observed throughout and well beyond
  the controller goal window: it repeatedly transmitted the identical final
  SDK target near
  `[-117.3,-28.7,58.5,-22.6,-44.5,31.5] deg` while feedback remained near
  `[-116.8,-31.6,57.3,-22.3,-44.5,31.1] deg`. Therefore the prior hypothesis
  that the actuator missed the correction merely because the final unchanged
  frame was not reasserted is falsified. The transport is alive; repeatedly
  sending the same final frame does not make Joint2 converge.
- The task failed closed and released its execution slot. It requested no
  near-field plan and submitted no approach, contact, grasp, or lift motion.
  No `/grasp/stop`, torque-off, disable, controller-stop, or arm-stop command
  was published.
- A separate generalized actuator-contract defect is now visible: during the
  observation move, the driver target used `gripper_raw=0` although reset
  feedback had been `gripper_raw=995` and the measured width was about
  `49.75 mm`. This suggests the ROS gripper “open” position may be mapped to
  the SDK raw endpoint with reversed semantics. It is recorded as evidence
  only; no mapping change will be made until the command conversion is traced
  end-to-end in source and existing logs.
- Intermittent SDK status `0xE1` events were recorded while fresh measured
  maximum temperature rose from about `46 C` to `51 C`. The driver explicitly
  treated these as status events and sent no torque-off because the measured
  over-temperature condition was not met. This run does not justify
  attributing the fixed Joint2 residual to heat without further source and
  controller evidence.

### User powered the arm off; real execution boundary restored

- The user explicitly reported that they powered the arm off after the
  aligned-run correction failure. The grasp task was already inactive and its
  execution slot had already been released before this report.
- From this point, work is restricted to source, protocol, historical-log,
  configuration, and test analysis. No new trajectory, gripper position,
  grasp start, `/grasp/stop`, torque-off, disable, controller-stop, or
  arm-stop command will be issued.
- Any conclusion that requires powered hardware feedback, a real image, or a
  physical motion will be recorded as an unverified boundary and deferred
  until the user explicitly powers the hardware again.

### Arm powered again; explicit joint enable restored without a stop command

- The user reported power restored but suspected that the arm was not
  enabled. Fresh hardware feedback was present at approximately
  `[-0.00460,-0.01074,-0.01227,-0.00307,-0.01074,+0.00460] rad`, with
  `gripper_raw=994..995`, but the driver explicitly reported
  `motion_enabled=false` and blocked its SDK motion stream.
- The live driver exposes no torque service. Its supported positive enable
  path is `/demonstration=false`; source inspection confirms that this clears
  the cached joint command before writing the exact SDK torque-on frame
  `AA 05 00 01 01 F9 FF`. The opposite Boolean value is torque-off and was
  never published.
- The first one-shot positive request arrived before fresh feedback and was
  rejected with `feedback ... stale (age 9.89s)`. After feedback resumed, a
  repeated positive request was accepted; the existing protection latch was
  cleared after its already elapsed healthy interval, and the driver logged
  `Disabling zero-torque mode with SDK torque_on frame`.
- `/alicia_d/motion_enabled` then latched `true`. Feedback continued at the
  powered-on near-zero posture with the gripper near `raw=995`; no grasp,
  arm trajectory, gripper position, `/grasp/stop`, torque-off, disable,
  controller-stop, or arm-stop command was sent.
- Work now resumes on the prior evidence-backed actuator endpoint and gripper
  semantic defects. A new 6D execution will not reuse the failed plan or its
  old target epoch.

### First arm-only command closed the gripper: measured-state merge defect fixed

- The task-side evidence proves its configured semantics were correct. Before
  the aligned run, real feedback was `gripper_raw=995` / `0.04975 m`; the task
  compared this with `open_position_m=0.050` and logged
  `open gripper before 6D motion skipped ... within 0.0010m`. Therefore the
  task did not request closure before the observation motion.
- At the first ros_control arm trajectory message, the driver instead logged
  a six-joint target with `gripper_rad=0.000`, immediately transmitted
  `gripper_raw=0`, and physical feedback fell from `995` through `376` to
  `4`. The arm trajectory publisher intentionally omits `right_finger`
  (`publish_gripper_command=false`), so the value came from the driver's
  partial-command merge state.
- Source tracing found the exact initialization defect:
  `parse_sdk_joint_state_frame()` seeded `cmd_gripper_rad_` from measured
  feedback but left `latest_gripper_rad_` at its constructor default `0`.
  `joint_command_callback()` correctly preserves a missing gripper field from
  `latest_gripper_rad_`; on the first arm-only command that meant preserving
  the uninitialized closed value rather than the measured open value.
- Fixed the generalized partial-command contract. While no command has yet
  been accepted, every fresh real feedback frame now seeds both
  `latest_joint_angles_` and `latest_gripper_rad_` from measured hardware,
  alongside the existing `cmd_*` seed. It deliberately leaves
  `has_latest_command_=false`, so feedback alone still cannot start SDK
  command streaming.
- Added a regression that binds both measured-state assignments and verifies
  that the seed block does not mark a command available. The first test
  revision incorrectly matched a wrapped comment and failed `1/9`; the test
  was corrected to inspect the actual seed block semantics. The final suite
  passed `9/9`, the full `alicia_d_driver` package rebuilt successfully, and
  `git diff --check` passed.
- The rebuilt binary is not yet loaded into the live serial process. No
  physical command or enable-state change was caused by editing, testing, or
  compilation.

### Corrected partial-command driver hot-loaded

- The exact old driver process was PID `18416`. It was terminated with
  `SIGTERM`; the driver destructor only disconnected the serial transport and
  contains no torque-off frame.
- Started the rebuilt binary as `/alicia_d_driver_node` in terminal session
  `6489`, reusing the preserved private parameters: `/dev/alicia_arm`,
  `1000000` baud, `auto_torque_on_startup=true`, and held-command keepalive
  `10 Hz`. It opened the serial port and explicitly requested torque-on.
- Fresh feedback stabilized near
  `[-1.919010,+0.282252,-0.047553,+0.007670,-0.184078,+0.007670] rad` with
  `right_finger=0.0499 m` (`gripper_raw=998`) and status `0x00`.
  `/alicia_d/motion_enabled=true`.
- No `/joint_commands` arrived during the observed startup interval, and the
  corrected driver emitted no SDK trajectory frame. Thus the hot-load neither
  replayed the failed 6D target nor changed the open gripper. No torque-off,
  disable, controller-stop, arm-stop, or `/grasp/stop` command was issued.

### Repeatable endpoint bias isolated; live-feedback trim implemented

- Three independent same-pose corrections at different live-derived
  observation targets preserve the same actuator pattern:
  - SDK Joint2 `-45.8 deg`, measured `-48.9 deg`, residual about `3.1 deg`;
  - SDK Joint2 `-31.1 deg`, measured `-33.8 deg`, residual about `2.7 deg`;
  - SDK Joint2 `-28.7 deg`, measured `-31.6 deg`, residual about `2.9 deg`.
  The controller errors were respectively `0.053877`, `0.048842`, and
  `0.050138 rad`. These are different object-space poses and two different
  driver keepalive modes, while the direction and magnitude repeat. This
  supports a stable actuator endpoint bias; it does not support changing the
  hand-eye TF or one object coordinate.
- A fixed Joint2 angle offset was explicitly rejected because it would be a
  calibration-specific dead program and could not adapt to posture, direction,
  load, temperature, or another joint. Relaxing the `6 mm` Cartesian endpoint
  audit or `0.035 rad` controller goal band was also rejected because it would
  hide the measured error.
- Implemented a joint-agnostic stable-endpoint feedback trim in the SDK driver.
  It records the live ROS joint reference and resets immediately whenever any
  reference changes by more than one SDK encoder quantum
  (`2*pi/4096 rad`). It can activate only after the reference has remained
  stable for `0.30 s`, the maximum measured error exceeds the existing
  controller goal band `0.035 rad`, and that error remains inside the existing
  controller path band `0.12 rad`.
- Once active, every SDK joint reference is computed as
  `ros_target + clamp(gain * (ros_target - measured_feedback), +/-0.12 rad)`,
  with dimensionless gain `1.0`. The correction is recomputed from every fresh
  hardware sample and persists down to zero error so it does not drop out at
  the activation boundary. A new moving reference clears it. There is no
  object label, object coordinate, grasp-stage coordinate, Joint2 branch, or
  stored angle offset.
- The launch contract now exposes
  `endpoint_feedback_trim_enabled`,
  `endpoint_feedback_trim_stable_sec`,
  `endpoint_feedback_trim_activation_error_rad`,
  `endpoint_feedback_trim_max_rad`, and
  `endpoint_feedback_trim_gain` with the values above. Direct-node defaults
  match both launch files.
- Added regressions that bind the trim stable time to the existing controller
  settle duration, its activation threshold to the controller goal bound, its
  maximum to the controller trajectory bound, gain to `(0,1]`, and the
  implementation to live measured error with no Joint2-specific branch.
  Configuration tests passed `11/11`; the full C++ driver rebuilt and linked
  successfully; `git diff --check` passed.
- This is still an unverified real-hardware controller change. Editing,
  testing, and compilation sent no motion command. It must be hot-loaded and
  judged from the next real endpoint trace; success will not be inferred from
  source tests alone.

### Endpoint-trim driver hot-loaded; target remains aligned

- Installed the five endpoint-trim runtime parameters under
  `/alicia_d_driver_node`: `enabled=true`, stable time `0.30 s`, activation
  error `0.035 rad`, maximum trim `0.12 rad`, and gain `1.0`.
- The prior partial-command-fix driver was PID `21420`. It was terminated with
  `SIGTERM`, which only disconnected serial and sent no torque-off. The newly
  rebuilt driver is running in terminal session `28710`, reopened
  `/dev/alicia_arm` at `1000000` baud, and used the preserved
  `auto_torque_on_startup=true` path to request torque-on.
- Fresh feedback remained stable near
  `[-110.0,16.2,-2.7,0.4,-10.5,0.4] deg`, open gripper
  `raw=998`, and ordinary status `0x00`; isolated `E1/E2` status events were
  explicitly handled without torque-off. `/alicia_d/motion_enabled=true`.
- No joint command arrived during startup, so neither endpoint trim nor an SDK
  trajectory activated. The stale task remains `FAILED`, `active=false`.
- Live target evidence is centered and stable near UV `(297,247)`, depth
  `0.305-0.306 m`, and base position about
  `(-0.153,-0.455,0.065) m`. This is adequate to generate a fresh far-field
  plan, but automatic execution is held until the remaining near-field
  absolute `6.5 mm` contact rule is replaced; otherwise the task could
  automatically cross from the observation validation into a non-generalized
  contact decision.

### Fixed 6.5 mm contact rule removed; near-field now resolves live uncertainty

- Source tracing confirmed that `6.500 mm` was not an independently measured
  contact requirement. It was constructed by adding the task's `6.0 mm`
  Cartesian endpoint pass/fail tolerance to the analytical gripper model's
  `0.5 mm` CAD contract tolerance. Those quantities have different semantics:
  an endpoint audit bound cannot scientifically define how much of an
  arbitrary target must overlap a finger contact patch.
- Removed that coupling from `remote_grasp6d_node.py`. The `6.0 mm` value
  remains exclusively in the measured endpoint audit; it is no longer copied,
  added, or read by contact candidate generation.
- The contact execution gate now derives its required overlap from the same
  frozen snapshot and observation execution used to generate the contact
  stages:

  `U = max(gripper support clearance, depth MAD * depth uncertainty scale)
       + measured observation endpoint position error`

  A candidate is contact-authorized only when its exact continuous
  intersection between the two-sided live target-height band and the Alicia
  CAD finger contact polygon satisfies `overlap >= U`. Equivalently, its
  dimensionless signal-to-uncertainty ratio must be at least `1`. This is a
  definition of a resolvable positive contact interval, not a tuned
  object-length threshold.
- The same live-derived `U` is now threaded through all four runtime paths:
  tabletop proposal/tilt-boundary generation, GraspNet analytical evaluation,
  temporal-stability recheck, and final analytical recheck before execution.
  A contact plan without an adaptive stage profile fails closed with
  `CONTACT_UNCERTAINTY_INVALID`. The far-field observation plan continues to
  defer this contact-only decision and retains its other geometry, collision,
  stability, IK, and MoveIt checks.
- Candidate and plan diagnostics now record the observed overlap, live
  uncertainty, signed uncertainty margin, signal-to-uncertainty ratio,
  bilateral live height span, coverage fraction, and the exact decision rule.
  No object label, paper-carton coordinate, stored grasp pose, or
  object-specific length enters this calculation. The agreed first
  observation standoff remains the sole fixed `100 mm` stage position.
- Regression coverage was rewritten to use two different snapshot depth MAD
  values and to verify the derived `3.0 mm` and `6.0 mm` requirements, ratio
  and margin audits, near/far phase separation, and fail-closed behavior when
  live uncertainty is missing. The first test command used bare `pytest`,
  which is not on this shell's `PATH`; Python syntax compilation and
  `git diff --check` passed. The existing catkin test runner
  `./devel/env.sh python3 -B -m pytest` is being used for the actual regression
  run.
- The first dynamic test used a `4.0 mm` depth MAD, hence an `8.0 mm`
  requirement for its `11.0 mm` high synthetic target. It correctly produced
  no contact candidate: the usable continuous live/CAD overlap was smaller
  than the measured uncertainty. This is evidence that the new gate fails
  closed rather than silently accepting every non-zero overlap. The passing
  branch now uses a different `3.0 mm` depth MAD (`6.0 mm` requirement), while
  the separate missing-profile test preserves the explicit failure case.
- This source/test work published no ROS topic or service request and caused no
  arm or gripper motion. The arm remains positively enabled; no torque-off,
  disable, controller-stop, arm-stop, or `/grasp/stop` command was issued.

### Live-overlap regression complete; legacy planner path made adaptive too

- The focused live-overlap suite passed `6/6`; analytical contact tests passed
  `5/5`; adaptive-stage tests passed `14/14`.
- Full validation then passed:
  - `test_remote_grasp6d_streaming.py`: `185 passed`;
  - `test_remote_grasp6d_node.py`: `154 passed`;
  - combined input-default, adaptive-stage, and analytical-gripper tests:
    `110 passed`;
  - Python compilation and `git diff --check`: passed.
- The old single-frame request path originally constructed its pregrasp,
  approach, and lift sequence directly from legacy fixed configuration before
  calling the analytical gate. That would have left a non-generalized route
  even though the continuous production stream was corrected. It now accepts
  a candidate-sequence factory, and `_process_frame()` supplies
  `_make_contact_sequence()` using the current geometry and the exact frozen
  snapshot. Consequently both request paths attach the same live
  `AdaptiveStageProfile` and contact uncertainty before any physical gate.
- Several old audit tests called candidates “good” while their raw orientation
  produced a horizontal insertion after the mandatory GraspNet-to-tool0 frame
  correction. The adaptive generator correctly rejected those at its physical
  tilt bound before the unrelated audit behavior could be exercised. The
  fixtures were corrected to use the strict model quaternion that yields a
  downward insertion, and their synthetic target height was reduced only to
  keep the intended palm-clearance pass path. Production limits and rejection
  logic were not relaxed.
- The planner is now eligible for a planning-only hot-load. Physical success
  is still unverified: the next fresh far-field observation must exercise the
  new endpoint feedback trim, and its measured residual must feed the new
  near-field overlap rule. No stale plan may be reused.

### New planner hot-loaded; first endpoint-trim trial improves but does not converge

- Replaced only the standalone `/remote_grasp6d_node` process. The new planner
  came online against protocol `3`; the real arm driver, ros_control,
  MoveIt, camera, task node, GUI, and positive joint enable were not restarted.
- Live target before planning remained centered at approximately UV
  `(298,246)`, depth `0.3043-0.3048 m`, confidence `0.900`, and base position
  `(-0.154,-0.456,0.065) m`. `/alicia_d/motion_enabled` was latched `true`,
  joint feedback was fresh, and the open gripper remained `0.0499 m`.
- The first empty `/grasp_6d/request_plan` call had the service's default
  `trigger=false`, so it merely kept already-stopped inference stopped. It sent
  no motion or enable command. The corrected `trigger=true` call started
  continuous inference.
- Fresh batches accumulated the required temporal window. Multiple batches
  produced strict `PREVIEW_READY` results with one selected candidate from two
  MoveIt-reachable candidates. A stale cached preview was not executed; the
  later plan `8b9d98872c843c14c95dd520`, source stamp
  `1785128107.956613779`, was accepted as a valid
  `FAR_FIELD_OBSERVATION_PLAN` with a live tabletop-geometry source and
  required opening `0.04307 m`.
- `/grasp/start` was invoked with that exact plan ID. The task kept the gripper
  open because measured feedback was already within `1 mm` of `50 mm`, then
  executed only the fixed `100 mm` far-field observation stage.
- First arrival settled after `0.91 s` but measured
  `17.4 mm / 2.55 deg`, above the independent
  `6.0 mm / 5.0 deg` endpoint contract. The one permitted same-pose correction
  settled after `0.90 s` and improved the residual to
  `10.1 mm / 1.40 deg`, but still failed closed with
  `OBSERVATION_ENDPOINT_NOT_CONVERGED`. The task released its slot and returned
  to far-field planning. No near-field plan, approach, contact, gripper close,
  or lift was executed.
- The driver trace proves the new feedback trim activated during the
  correction. At activation, the maximum joint error was `0.048345 rad`;
  its live trim was approximately
  `[-0.6,+2.8,+1.1,-0.3,+0.1,+0.4] deg`. Feedback improved, but the remaining
  Joint2 error stabilized near `1.0 deg`; the driver continued commanding
  approximately `1.0 deg` trim without driving that residual to zero.
- This is the expected steady-state defect of the implemented proportional
  algebra `u = r + (r-y)` when the actuator behaves approximately as
  `y = u-b`: the fixed point retains `b/2` error. The result therefore supports
  a generalized, bounded integral/iterative endpoint correction; it does not
  support a fixed Joint2 offset, target-space shift, hand-eye change, endpoint
  tolerance relaxation, or paper-carton-specific logic.
- Arm feedback remained ordinary (`status=0x00`, typical measured temperatures
  about `38-40 C`) apart from the previously observed isolated status/byte
  anomalies. The driver explicitly sent no torque-off. Joint enable remains
  on, and the gripper remained `raw=998`.

### Proportional half-bias defect replaced by response-gated accumulated trim

- The failed trial identified a controller-structure defect rather than a
  perception or object-specific error. With the old law
  `sdk_target = ros_reference + gain * current_error`, a constant actuator bias
  is fed back into a newly recomputed proportional term on every sample. For
  unity gain the closed-loop fixed point necessarily retains half of that
  bias, matching the observed reduction from about `2.8 deg` to about
  `1.0-1.4 deg` on Joint2.
- Replaced the recomputed proportional term with a joint-agnostic accumulated
  correction:

  `trim[k+1] = clamp(trim[k] + gain * (reference - measured), +/- path_bound)`

  It runs independently across all six joints. A changed ROS reference resets
  the accumulated vector, feedback anchor, response gate, and iteration count;
  there is no Joint2 branch, stored calibration offset, target label, paper
  carton coordinate, or grasp-pose constant.
- Integration is forbidden during a moving trajectory. Both the ROS reference
  and real SDK feedback must remain stable for the existing `0.30 s` controller
  dwell. Feedback stability is measured against an anchored window of one SDK
  encoder quantum (`2*pi/4096 rad`), rather than against an arbitrary
  object-space number.
- After each accumulated correction, the next iteration is locked until a
  fresh encoder sample proves at least one quantized joint response. Once a
  response begins, the feedback anchor is restarted and must settle again for
  `0.30 s`. This prevents blindly integrating during actuator latency or a
  stalled response. The accumulated vector remains bounded by the existing
  controller path bound `0.12 rad`, and initial activation still requires the
  existing goal-band residual `0.035 rad`.
- Persistent trim is applied while the actuator responds; it is not discarded
  merely because the new residual falls below the initial activation band.
  Iteration stops at the SDK encoder resolution. Runtime logs now distinguish
  an actual accumulated iteration from a held correction waiting for measured
  response.
- Updated both real-driver launch comments and source-contract regressions.
  Driver/config regressions passed `14/14`; `git diff --check` passed. No ROS
  command was published by these source tests.
- An attempted `catkin build alicia_d_driver --no-status` was rejected before
  compilation because this worktree's existing build space was created by
  `catkin_make`. Nothing was deleted or reconfigured; the package must be
  rebuilt with the workspace's original `catkin_make` mechanism.

### Uploaded motion video agrees with the planned observation trajectory

- Inspected
  `/home/zhuyupei/Videos/2e6b755fe3108d8f04c1079bf8ddae32.mp4`
  (`58.8 s`, `720x1280`, HEVC) at `4 fps` and as timestamped two-second
  montages. The visible motion is continuous during the initial approach,
  includes wrist reorientation on the way in, and then holds the first
  observation configuration. The clip contains no later near-field/contact
  motion.
- The exact task trace for accepted plan
  `8b9d98872c843c14c95dd520` proves the visible reorientation was commanded by
  the planned trajectory. Its executed observation pose was
  `xyz=(-0.162,-0.418,0.144) m`,
  `q=(0.680,0.657,-0.201,0.256)`. MoveIt generated one strict cached path,
  retimed to `21.928 s` with `0.080 rad/s` maximum joint speed.
- Along that path, the published target moved smoothly from approximately
  `[-110.0,+16.1,-2.6,+0.4,-10.6,+0.5] deg` to
  `[-119.6,-30.4,+59.4,-29.9,-45.5,+43.4] deg`; SDK feedback followed the same
  ordered sequence. The large-looking wrist change is therefore not an
  endpoint-trim transient or an unplanned final flip.
- Far-field observation uses the live candidate orientation so that the wrist
  camera and gripper are already aligned with that candidate's insertion axis.
  The same live orientation is carried through observation, pregrasp, grasp,
  and lift poses. Only the agreed first observation displacement is the fixed
  `100 mm` standoff; its orientation and target-space location remain derived
  from the current candidate and frozen snapshot.
- The video's container creation timestamp does not exactly coincide with the
  ROS execution stamps, so visual-to-ROS frame-level synchronization cannot be
  claimed. The commanded/feedback trajectory and the visible direction and
  endpoint nevertheless agree. This conclusion does not rely on guessing a
  joint pose from a single image.
- This inspection and code work sent no motion, gripper, stop, torque-off, or
  disable command. `/alicia_d/motion_enabled` was already latched `true`.
- Re-ran the build with this workspace's original mechanism:
  `catkin_make --pkg alicia_d_driver -j2`. The corrected
  `alicia_d_driver_node.cpp` compiled and linked successfully into
  `devel/lib/alicia_d_driver/alicia_d_driver_node`. The build itself published
  no ROS command and did not affect the still-running old driver process.

### Accumulated-trim driver hot-loaded at the restored initial pose

- Four consecutive real joint samples were identical before replacement:
  approximately
  `[-114.6,+19.5,-3.8,-1.4,-10.0,+2.7] deg`, with the gripper still open at
  `49.9 mm`. This is the user-restored initial configuration, not the failed
  observation endpoint.
- The old driver PID `22028` was ended with `SIGTERM`. Its destructor only
  disconnected the serial transport; no torque-off, demonstration-mode,
  controller-stop, arm-stop, disable, or `/grasp/stop` request was made.
- Started the newly linked binary as `/alicia_d_driver_node`, PID `31206`,
  preserving `/dev/alicia_arm`, `1000000` baud, `10 Hz` real feedback and
  keepalive, and all bounded trim parameters. The node opened the port and
  explicitly entered the configured `auto_torque_on_startup=true` torque-on
  path.
- Live post-start state is `/alicia_d/motion_enabled=true`,
  `/alicia_d/protection_latched=false`, ordinary `status=0x00` feedback at the
  same initial joints, `gripper_raw=998`, and measured maximum temperature
  `43-44 C`. Isolated E1/E2 bytes were logged as status events; the driver
  explicitly reported that no torque-off was sent without sustained measured
  over-temperature.
- No `/joint_commands` arrived during the observed new-driver startup, so the
  driver neither replayed the stale plan nor moved the arm/gripper. The next
  physical command must use a newly generated plan ID from the restored pose.

### First accumulated-trim execution succeeds physically but task audits early

- Continuous inference was already running. After the restored pose and new
  driver were stable, it published fresh far-field plan
  `3151cff997b034b978a52cc0`, source stamp
  `1785129043.706590652`, from the live tabletop-geometry path. Its observation
  pose was
  `xyz=(-0.164695,-0.421879,0.149384) m`,
  `q=(+0.697804,+0.652356,-0.185501,+0.230414)`, and its live required opening
  was `43.606 mm`.
- Started `/grasp/start` with `execute=true` and that exact plan ID. The task
  kept the measured `49.9 mm` gripper open and moved only through the
  far-field observation stage. The moving reference correctly prevented
  endpoint trim from activating during the trajectory.
- First observation audit, after the existing `0.90 s` joint-settle detector,
  measured `17.6 mm / 2.62 deg`. The one permitted correction replanned and
  executed the identical frozen observation pose; it did not translate the
  target or use a new object coordinate.
- Once the corrected ROS reference and feedback settled, the new driver
  produced the expected response-gated evidence:
  - iteration 1 at maximum joint error `0.050583 rad`, accumulated trim
    `[-0.7,+2.9,+1.1,-0.4,+0.2,+0.3] deg`;
  - after measured encoder response and another feedback dwell, iteration 2 at
    `0.013844 rad`, accumulated trim
    `[-0.4,+2.1,+0.9,-0.2,+0.4,+0.4] deg`;
  - further response-gated iterations reduced the observed maximum joint
    residual to about `0.006098 rad`.
- The task performed its final Cartesian audit at
  `1785129156.780948400`, between continuing outer-loop iterations. It had
  already improved to `6.6 mm / 0.87 deg`, only `0.6 mm` outside the unchanged
  `6.0 mm` position contract, but the task immediately failed with
  `OBSERVATION_ENDPOINT_NOT_CONVERGED` and released the execution slot.
  Consequently no near-field plan, contact approach, close, or lift occurred.
- The driver kept the same bounded target after task release and completed the
  physical endpoint convergence. A precision TF measurement against the
  frozen requested pose found:
  - actual
    `xyz=(-0.165348004,-0.421377486,+0.146684095) m`;
  - actual quaternion
    `(+0.698900662,+0.651977308,-0.183190062,+0.230010554)`;
  - residual `2.822179 mm / 0.299905 deg`.
  This is well inside the original `6.0 mm / 5.0 deg` contract and proves the
  accumulated correction works on real hardware.
- Root cause of the remaining task failure is therefore temporal: the generic
  joint-settle detector accepts a short low-delta interval between driver
  iterations, then `_converge_far_field_observation_endpoint()` samples FK only
  once and fails. It is not evidence for relaxing the endpoint bound, changing
  pruned19 TF, adding a fixed joint offset, or shifting the object.
- Planned fix: after the sole same-pose correction, use the existing
  `motion_settle_timeout_sec=5.0`, sample interval, and required-sample count to
  wait for consecutive live FK samples satisfying the same
  `6.0 mm / 5.0 deg` endpoint contract. This adds neither another motion nor a
  blind delay: success remains measurement-based, and timeout retains the same
  fail-closed result.
- Joint enable remained `true`; gripper remained open. No stop, torque-off, or
  disable command was issued.

### Far-field correction now waits on the unchanged Cartesian contract

- Implemented `_wait_for_measured_endpoint_contract()` in
  `grasp_task_node.py`. It is used only after the configured final same-pose
  observation correction would otherwise fail. It does not request another
  MoveIt path.
- The wait reuses the existing `/grasp/motion_settle_timeout_sec` (`5.0 s`),
  sample interval (`0.05 s`), and consecutive-sample count (`3`). It repeatedly
  evaluates live `base_link -> tool0` FK and succeeds only after all consecutive
  samples satisfy the unchanged `6.0 mm / 5.0 deg` contract. A sample outside
  either bound resets the count; unavailable/non-finite evidence cannot pass.
  Timeout preserves `OBSERVATION_ENDPOINT_NOT_CONVERGED`.
- On success, the task records a fresh
  `/grasp_6d/runtime_execution_error` sample. This ensures the near-field live
  overlap uncertainty consumes the converged observation error, not the
  transient `6.6 mm` audit captured between actuator iterations.
- Added regressions for the exact real trace pattern
  (`20 mm -> one identical correction -> 6.6 mm -> held live FK convergence`),
  the one-correction limit, consecutive in-contract samples, final evidence
  refresh, and fail-closed timeout result.
- Full `test_grasp_task_sequence.py` passed `119/119`; Python compilation and
  `git diff --check` passed. No physical command was sent by editing or tests.
- While the user returned the arm, the new reference naturally cleared the
  observation trim. Six consecutive live samples were identical at about
  `[-110.5,+19.2,-3.3,-1.9,-12.9,+0.6] deg`; gripper feedback remained
  `49.9 mm`. This is the next restored initial pose.

### Corrected task node hot-loaded and bound to a new live plan

- Ended only the inactive old `grasp_task_node` process and started the
  corrected installed script as PID `33608`. Restarting this task process
  published no arm, gripper, stop, torque-off, or disable command.
- The new node reported `GraspTaskNode ready` and `/grasp/state=IDLE`,
  `active=false`, `message=ready`. The real driver remained PID `31206` with
  `/alicia_d/motion_enabled=true` and
  `/alicia_d/protection_latched=false`.
- The continuously computed planner first supplied fresh plan
  `ba54b5c17d970ac59f9d09d4` to the restarted task. A newer live far-field
  plan was then sampled at source stamp `1785129604.632235765`:
  `ce74f477a46f4b5355ee58cc`, diagnostic
  `FAR_FIELD_OBSERVATION_PLAN`, source lineage `tabletop_geometry`.
- This latest plan derives its geometry from the live instance mask/depth
  evidence (`3375` valid depth points, `99.38%` valid depth ratio, five fused
  frames). Its first observation pose is
  `xyz=(-0.160391,-0.419106,+0.146631) m`,
  `q=(+0.684625,+0.656020,-0.197613,+0.248748)`, with live required opening
  `43.651 mm`. No fixed object coordinate, object-specific joint offset, or
  carton displacement is used; only the agreed first observation standoff is
  fixed at `100 mm`.
- The next automatic execution will be bound to an exact newly sampled plan
  ID immediately before `/grasp/start`, so continuous inference cannot make
  the service consume an unintended stale plan.

### Second accumulated-trim run exposes a real actuator-response stall

- `/grasp/start` accepted the exact live plan
  `ce74f477a46f4b5355ee58cc`. The task kept the measured gripper open at about
  `49.9 mm` and executed only the far-field observation motion.
- The first endpoint audit measured `16.8 mm / 2.46 deg`. As designed, the
  task requested its single bounded same-pose correction; it did not alter the
  frozen target pose or use any object-specific offset.
- The real driver then produced two response-gated accumulated corrections:
  - iteration 1 started at maximum joint error `0.048778 rad`, with trim
    `[-0.6,+2.8,+1.1,-0.3,-0.1,+0.5] deg`;
  - measured encoder motion followed, allowing iteration 2 at maximum residual
    `0.011047 rad`, with updated trim
    `[-0.5,+2.2,+0.9,-0.3,0.0,+0.4] deg`.
- After that second command, the feedback remained identically near
  `[-119.1,-30.0,+61.2,-27.7,-47.0,+40.6] deg` while the driver repeatedly
  transmitted approximately
  `[-119.5,-28.4,+61.8,-28.0,-46.9,+40.9] deg`. Because no new encoder quantum
  was measured, the response gate correctly refused to integrate another
  error sample.
- The corrected task wait was exercised exactly as intended. Its first audit
  after the same-pose correction measured `6.2 mm / 0.75 deg`; it then sampled
  live FK for the full `5.00 s`, but never obtained three consecutive samples
  inside the unchanged `6.0 mm / 5.0 deg` contract. It therefore returned
  `OBSERVATION_ENDPOINT_NOT_CONVERGED` and released the execution slot.
- No near-field plan, contact motion, gripper close, lift, stop, torque-off, or
  disable occurred. Joint enable remains on.
- This run rules out the previous early-audit race as the complete cause. The
  remaining evidence is a real generic endpoint response problem: the SDK
  accepts and streams the bounded second trim command, but the physical
  encoders do not move again. The next diagnosis must inspect the protocol and
  actuator command semantics against this trace; it must not relax the
  Cartesian contract by `0.2 mm`, add a carton-specific offset, or invent a
  joint-specific correction.

### Endpoint response-stall diagnosis and generic control direction

- Source inspection confirms each output frame is the vendor's combined
  `AA 06 03 1C` six-joint-plus-gripper position command. The logged `ok` means
  all serial bytes were written; this protocol path has no per-command
  actuator acknowledgement. Therefore `write=ok` cannot be treated as proof
  that a small physical correction occurred; fresh encoders remain the only
  admissible response evidence.
- The bundled vendor SDK exposes the same frame and 4096-count joint
  quantization. Its high-level `set_robot_state()` uses a default joint
  completion tolerance of `0.1 rad`, much looser than the grasp task's
  Cartesian endpoint contract. This does not justify relaxing the 6 mm task
  gate; it explains why an additional measured outer loop is necessary.
- The real trace distinguishes a lost frame from a repeatable small-signal
  response stall: the first accumulated update caused encoder motion in about
  one feedback interval, while a later, smaller change was re-sent for many
  seconds with fresh unchanged feedback. The existing response gate prevented
  runaway integration, but it also made this recoverable non-response
  permanent.
- Planned generic resolution is a bounded anti-windup integral recovery:
  preserve the encoder-response gate normally; when fresh feedback proves no
  response for longer than the arm's already measured response latency,
  permit another live-error integral update. Continue to require finite fresh
  feedback, stable reference, the existing `0.12 rad` correction bound, and an
  unchanged desired Cartesian pose. Any encoder quantum restores the normal
  settle-and-response cycle. This contains no object label, fixed grasp
  coordinate, fixed joint correction, or electronic-skin input.
- The non-response dwell will be derived from live response latency (with the
  existing feedback-stale timeout as the no-history fallback), rather than a
  carton- or joint-specific delay. Saturation without encoder response remains
  fail-closed, so a mechanically blocked joint cannot cause unbounded command
  growth.

### Adaptive stalled-response recovery implemented and built offline

- Added response timing state to the real driver. Every accumulated trim now
  records when it was applied. The first encoder quantum records the observed
  actuator response latency and returns the controller to its normal
  feedback-settle cycle.
- If the reference is still stable, feedback is fresh, the live error remains
  inside the existing `0.12 rad` trim window, and no encoder quantum appears,
  the driver computes the retry dwell as:
  `max(existing 0.30 s settle time, 2 * last measured response latency)`.
  Before any latency has been measured it uses the existing `1.0 s`
  feedback-stale timeout. Only after that dwell can the same joint-agnostic
  live error be integrated again.
- Each retry restarts the response timer. All updates retain
  `trim[k+1] = clamp(trim[k] + gain * (reference - measured), +/-0.12 rad)`.
  Thus a small firmware/deadband stall can be crossed progressively, while a
  blocked or unresponsive actuator reaches the anti-windup limit and cannot
  grow without bound.
- Reference motion by more than one encoder quantum clears the entire trim,
  latency, and stalled-retry state. The user returning the arm to the initial
  pose therefore cannot inherit any observation-pose compensation.
- Added regression assertions that stalled recovery requires a stable
  reference, a new feedback sample, in-window live error, an active
  response-wait state, and error above one SDK encoder quantum. The tests also
  retain the checks for joint-agnostic logic and the original controller
  bounds.
- Offline verification while the arm was physically powered off:
  `test_moveit_trajectory_execution_config.py` passed `11/11`,
  `git diff --check` passed, Python compilation passed, and
  `catkin_make --pkg alicia_d_driver -j2` compiled and linked successfully.
  The new binary SHA-256 is
  `9c2981824d2744251e27854f58cabb80b71f8cb4ef45d3cf5ce7f98250b14367`.
- A first direct invocation of `test_grasp_task_sequence.py` lacked the
  workspace environment and failed at import time with
  `ModuleNotFoundError: alicia_flexible_grasp_supervisor`; no test body or ROS
  command ran. Re-running after `source devel/setup.bash` passed all `119/119`
  task tests. This was an invocation-environment issue, not a code failure.
- Per the user's instruction, the newly built driver has **not** been
  hot-loaded while the arm is powered off. No ROS motion, enable-state change,
  stop, torque-off, or disable command was sent during this offline work.

### Adaptive driver hot-loaded after physical power-on

- After the user explicitly reported power restored, ended old driver PID
  `31206` with `SIGTERM`. Its shutdown only disconnected the serial transport;
  no torque-off, stop, demonstration-mode, or disable command was sent.
- Started the newly built binary as `/alicia_d_driver_node`, PID `35607`.
  It opened `/dev/alicia_arm` at `1000000` baud and entered the existing
  `auto_torque_on_startup=true` path. Live state is
  `/alicia_d/motion_enabled=true`,
  `/alicia_d/protection_latched=false`.
- Repeated feedback was stable at approximately
  `[-111.2,+26.1,+1.7,-2.2,-26.1,-1.1] deg`, with gripper raw `995`
  (about `49.8 mm`). Ordinary temperature frames peaked near `37 C`; one
  isolated `67 C` sample was not sustained and did not latch protection.
- Immediately after restart, perception correctly published invalid
  `TARGET_LOST` rather than reusing an old plan. It subsequently recovered
  from live RGB-D evidence and the task accepted fresh far-field plan
  `61bbe75bb46e67302f66c478`, source stamp
  `1785130342.463812589`, diagnostic
  `FAR_FIELD_OBSERVATION_PLAN`, lineage `tabletop_geometry`.
- The fresh plan's observation pose is
  `xyz=(-0.154430,-0.416844,+0.144482) m`,
  `q=(+0.682672,+0.656632,-0.205697,+0.245930)`, with live required opening
  `43.097 mm`. It is derived from five fused frames, `2849` valid depth
  points, `98.55%` valid depth ratio, and the current instance-mask geometry.

### Adaptive recovery succeeds at the first observation stage

- `/grasp/start` accepted the exact fresh far-field plan
  `61bbe75bb46e67302f66c478`, source stamp
  `1785130342.463812589`. The first endpoint audit measured
  `17.1 mm / 2.43 deg`; the task then issued its one permitted same-pose
  correction without changing the frozen target pose.
- The adaptive real driver used only measured encoder error. Its first
  accumulated correction began at maximum joint error `0.046164 rad` with
  trim approximately `[-0.7,+2.6,+1.1,0.0,-0.1,+0.3] deg`. Both normal
  encoder-response iterations and bounded stalled-response retries were
  observed. It converged with maximum joint error `0.000851 rad` and final
  trim approximately `[-0.4,+2.8,+1.0,-0.3,+0.1,-0.1] deg`.
- The task measured three consecutive FK samples inside the unchanged
  `6.0 mm / 5.0 deg` contract after `0.58 s`; final residual was
  `1.6 mm / 0.25 deg`. It therefore entered `PLAN_PREGRASP` and waited for a
  live near-field 6D contact plan. This is the first real execution evidence
  that the generic response-stall recovery resolves the previous observation
  endpoint stall without a fixed Cartesian or joint correction.

### Near-field generation fails closed; no second-stage motion occurred

- During the near-field wait, the remote pipeline continued using live RGB-D
  frames but repeatedly returned no valid contact execution plan. A
  representative committed audit reported `522` raw candidates, `43` after
  NMS, `28` after remote collision filtering, and zero locally valid
  candidates. Its rejection summary was
  `CANDIDATE_CONTRACT_INVALID:24`,
  `GRIPPER_CONTACT_PATCH_MISS:1`, and
  `GRIPPER_SWEEP_COLLISION:4`.
- That audit's live contact-overlap requirement was `13.236 mm`, composed of
  reported depth uncertainty `13.236 mm` and measured observation endpoint
  position error `1.636 mm` through the current max/support-clearance rule.
  The best observed tabletop overlaps were only `4.947 mm`, `5.185 mm`, and
  `2.033 mm`; consequently every fallback branch failed materialization.
  This number is not the removed fixed `6.500 mm`, but its scientific meaning
  must still be audited before it can be trusted as an estimator uncertainty.
- The remote input itself remained populated: one audit contained `10016`
  target points and `18103` support points, target fraction `0.3562`, and
  mask/bounding-box IoU `0.958`. Thus this run does not support a
  `TARGET_LOST` diagnosis. Other strict analytical rejections included
  non-parallel jaw axes and palm/support-clearance intersection.
- After `150.0 s` the task failed naturally with
  `NEAR_FIELD_REPLAN_TIMEOUT: no fresh near-field 6D preview; last=`
  `NEAR_FIELD_PLAN_PHASE_INVALID: Preview is not a contact execution plan`,
  returned service result `success: False, message: "failed"`, and released
  the execution slot. No second-stage Cartesian motion, contact, gripper
  close, lift, `/grasp/stop`, torque-off, or disable command occurred.
- A read-only audit query first attempted `jq`, which is not installed and
  returned `/bin/bash: jq: command not found`. The evidence above was then
  extracted with a read-only Python JSON parser; this tool invocation failure
  did not alter ROS or robot state.
- Source diagnosis now focuses on whether `depth_mad_m` represents temporal
  estimator repeatability or merely the spatial depth spread of the object's
  visible surface. Relaxing `13.236 mm` to make this object pass would be an
  object-specific workaround and is explicitly rejected. Any correction must
  remain label-free, coordinate-free, based on live measurements and gripper
  geometry, and repeatable across graspable objects.

### Root cause: spatial object shape was mislabelled as measurement uncertainty

- `fuse_stable_samples()` first takes a per-pixel median over the stable RGB-D
  frames. It then collects every retained depth on the fused target surface
  and computes `median(abs(depth - target_surface_median))`. This final value
  is published as `DepthQuality.depth_mad_m`.
- Consequently, `depth_mad_m` contains real spatial geometry: height
  variation, surface slope, curvature, and any remaining multi-surface depth.
  It does **not** measure the variation of repeated observations at the same
  pixel or the dispersion of a repeated boundary estimate.
- `_snapshot_depth_mad_m()` passes that spatial statistic unchanged into
  `derive_adaptive_stage_profiles()`, where it is multiplied by the generic
  `depth_uncertainty_scale=2.0`. The result is used both for stage clearances
  and, in contact phase, as the minimum bilateral contact-patch overlap.
  Therefore the `13.236 mm` run-time requirement was dynamic but
  statistically invalid: a more curved or sloped graspable object would be
  required to have more finger overlap merely because its genuine surface
  spans more depth.
- The existing five synchronized samples, stable identity/epoch contract,
  mask-IoU gate, centroid-shift gate, joint-motion gate, and per-pixel frame
  stack already provide the data needed to estimate repeatability without a
  new image or object-specific constant. The generic correction will preserve
  spatial MAD for geometry diagnostics/outlier filtering, add an explicitly
  temporal repeatability statistic, and use only that statistic (plus physical
  gripper/support clearance and measured endpoint error) as uncertainty.

### Spatial geometry and temporal repeatability are now separated

- `DepthQuality` now retains the existing `depth_mad_m` as the spatial fused
  surface statistic and adds `depth_repeatability_m`. For each target pixel
  surviving the target-depth filter, the new estimator subtracts that pixel's
  median across the stable frame window, pools only finite same-pixel absolute
  residuals with at least two observations, and multiplies the residual MAD
  by `1.4826` to obtain a robust standard-deviation estimate.
- This estimator is label-free and translation-independent. A static sloped,
  curved, or stepped object surface can have nonzero spatial MAD while its
  temporal repeatability remains zero. Actual cross-frame depth jitter
  increases the temporal estimate. Mask edges, support pixels, filled holes,
  invalid depths, and rejected fly points are excluded.
- Adaptive contact-stage generation now accepts
  `depth_repeatability_m`, not `depth_mad_m`. The live perception component is
  `max(gripper/support clearance, 2 * robust temporal sigma)`, after which the
  independently measured Cartesian endpoint error is added. The `2.0` factor
  remains the object-independent uncertainty multiplier; the physical
  `3 mm` support clearance remains a lower bound. No fixed `6.5 mm`, object
  coordinate, class name, joint offset, or electronic-skin input was added.
- Gate audits now publish all three meanings separately:
  `depth_spatial_mad_m`, `depth_repeatability_m`, and the resulting adaptive
  `depth_uncertainty_m`, plus estimator identity
  `2x_robust_same_pixel_temporal_sigma_with_physical_clearance`. This makes a
  later real run directly falsifiable rather than hiding the source of the
  overlap threshold.
- Added regression cases proving (1) genuine static surface shape does not
  inflate temporal uncertainty and (2) controlled same-pixel frame jitter
  produces the expected `1.4826 * MAD` result. Streaming tests use deliberately
  different spatial MAD values with identical repeatability and verify that
  stage profiles remain identical.
- One initial multi-file `apply_patch` used a mismatched docstring context and
  was rejected without modifying that file. Reapplying against the exact
  source context succeeded. The first test commands called a nonexistent
  standalone `pytest` executable and returned `pytest: command not found`
  before collection; using the repository's available
  `python3 -m pytest` entry point corrected the invocation.
- Verification results:
  - RGB-D snapshot fusion: `49/49` passed;
  - adaptive stage profiles: `14/14` passed;
  - remote streaming planner: `185/185` passed;
  - remote planner node: `154/154` passed;
  - object/tabletop/gripper geometry: `132/132` passed;
  - grasp task sequence: `119/119` passed (five existing rospy deprecation
    warnings only);
  - Python compilation and `git diff --check` passed.
- All implementation and verification above occurred with no ROS motion
  request, contact command, gripper close, stop, torque-off, or disable
  command. The previous task remained inactive while the user moved the arm
  manually to the initial position.

### Corrected planner hot-loaded; complete ROS stack remains enabled

- With the prior task already naturally failed and its execution slot
  released, ended only old `/remote_grasp6d_node` PID `25281` using its
  terminal interrupt and started the corrected source as a new
  `/remote_grasp6d_node` terminal session. No driver, MoveIt controller,
  motion gateway, task node, joint enable, or physical arm process was
  restarted.
- The corrected planner reported online with protocol `3`, loaded GraspNet
  backend, and WSL URL `http://172.23.132.97:8000`. Source hashes at hot-load:
  - `remote_grasp6d_node.py`:
    `25505d5caafcb7c325b02fe837b419e2d430c16ca0b97c6fe20835bbbea1a75d`;
  - `rgbd_snapshot.py`:
    `3eae879102eb902f9a90b2832db10cfc08f569e1b4338ba0aa1489b6e53f97cc`;
  - `adaptive_stage_profiles.py`:
    `5380d0ed39df939bf3970c9596ff4c18989fed87cd98f03098bc5fbeb66d80e4`.
- While the user returned the arm to the initial pose, perception published
  `TARGET_LOST` and the inactive task rejected those invalid plans. This is
  expected fail-closed behavior; it issued no motion.
- The complete stack remained present: real driver, hardware interface,
  gripper controller, controller spawner, data logger, task node, hand-eye
  TF, motion gateway, MoveIt, perception, corrected remote planner, robot
  state publisher, safety monitor, and camera. The GUI process alone had
  exited, so `gui.launch` was started directly and
  `/alicia_supervisor_gui` came online as PID `40888`. This did not restart
  or modify the arm driver or enable state.
- ROS reported the existing `~/.ros/log` directory above `1 GB` during GUI
  launch. No cleanup was performed because deleting logs was not requested
  and the runtime evidence must be preserved.
- No `/grasp/stop`, torque-off, disable, controller stop, gripper close, or
  arm motion command was sent during this hot-load and GUI recovery.

### User-requested joint re-enable completed

- The user explicitly requested that the arm be re-enabled. An initial call
  to `/alicia_d/torque_on` failed at the ROS service lookup layer with
  `Service [/alicia_d/torque_on] is not available`; the running driver exposes
  no torque service, so this attempt sent no serial frame and changed no
  hardware state.
- Source and live service inspection identified the actual driver contract:
  `/demonstration` is a `std_msgs/Bool` subscriber. `data: false` enters the
  driver's torque-on branch, whereas the prohibited `data: true` enters its
  torque-off branch.
- Published exactly one `/demonstration` message with `data: false`. The real
  driver logged `Disabling zero-torque mode with SDK torque_on frame.` at
  ROS time `1785132049.279457`, and the latched
  `/alicia_d/motion_enabled` state subsequently reported `data: True`.
- Fresh hardware feedback continued near
  `[-109.0,+26.7,-6.9,-6.4,-18.2,+6.7] deg`, gripper raw `995`, with ordinary
  temperature frames at `34-35 C`. Isolated `E1` status events returned to
  `0x00`; no sustained measured over-temperature latch occurred.
- No `data: true`, torque-off, stop, `/grasp/stop`, controller stop, gripper
  close, or task/motion command was published as part of this re-enable.

### Corrected live-uncertainty execution started after target alignment

- After the user reported the target aligned, the corrected planner was still
  in its post-restart `manual trigger` state. A first empty
  `/grasp_6d/request_plan` request defaulted to `trigger:false` and kept
  streaming stopped; it generated no candidate and caused no motion.
  Reissued the exact request with `trigger:true`, which returned
  `continuous remote 6D inference started`.
- A direct `rostopic echo` of the custom `Grasp6DPlan` initially failed with
  `Cannot load message class`; no ROS state changed. The authoritative task
  subscriber and planner audit were used instead.
- The first corrected far-field requests showed
  `depth_uncertainty_m=0.003000 m`, rather than the previous erroneous
  `0.013236 m`. Because no stage had yet executed, endpoint error was zero
  and the live lower bound was the real gripper/support clearance. This is
  real hardware evidence that spatial surface MAD no longer inflates the
  uncertainty.
- After cross-frame stability and strict MoveIt evaluation, request `4`
  produced `PREVIEW_READY`: three candidates were MoveIt-reachable and one
  was selected/promoted. The task accepted exact fresh plan
  `03909171b64495c031ef4148`, source stamp
  `1785132279184642076`.
- Called `/grasp/start` with `execute:true` and that exact plan ID. The gripper
  remained open because measured feedback was already within `1 mm` of the
  `50 mm` opening.
- The first observation execution measured `15.6 mm / 2.34 deg` endpoint
  error. The existing single same-pose correction then converged to
  `1.7 mm / 0.24 deg`; the task accepted it inside the unchanged
  `6.0 mm / 5.0 deg` contract and entered
  `PLAN_PREGRASP waiting for near-field 6D preview`.
- The driver converged generically from encoder feedback, ending near maximum
  joint residual `0.001230 rad` with accumulated trim approximately
  `[-0.1,+2.4,+1.1,-0.1,0.0,-0.1] deg`. No object coordinate or fixed joint
  correction was used.
- The first near-field request after arrival had no locally valid candidate
  and remained fail-closed. At this point no second-stage motion, contact,
  gripper close, lift, stop, torque-off, or disable had occurred.

### Near-field live gate evidence after first-observation convergence

- Continued supervising the active `/grasp/start` execution after the
  first-observation endpoint converged. The task remained in
  `PLAN_PREGRASP waiting for near-field 6D preview`; the driver held the
  observation pose with maximum joint residual approximately
  `0.001230 rad`, gripper raw `995`, and ordinary measured temperatures
  rising gradually through approximately `39-42 C`.
- The corrected uncertainty model was exercised by real near-field frames.
  Its dynamically computed continuous bilateral contact-overlap requirement
  was `0.004703 m`, not the former `0.006500 m`. This value is derived from
  the live `0.003000 m` gripper/support lower bound plus the measured
  `0.0017 m` endpoint position error; it is not an object-specific constant.
- Several tabletop-geometry tracks passed all six analytical gripper stages,
  but their stable rechecks measured bilateral overlap of `0.004541 m` or
  `0.004455 m`, respectively only `0.000162 m` and `0.000248 m` below the
  live requirement. The strict `GRIPPER_CONTACT_PATCH_MISS` gate therefore
  correctly withheld execution instead of rounding or tuning the threshold
  for the current carton.
- Near-field request `20` received `925` raw candidates, retained `73` after
  NMS, returned `43` after the remote collision stage, and had `12`
  locally-valid tabletop candidates. All 12 failed their stability recheck,
  so the outcome remained `STABILITY_PENDING` with no preview or promotion.
  A later request `23` received `768` raw candidates and failed earlier:
  `31 CANDIDATE_CONTRACT_INVALID`, `11 GRIPPER_SWEEP_COLLISION`, and one
  `GRIPPER_CONTACT_PATCH_MISS`, with no locally-valid candidate.
- The repeated `CANDIDATE_CONTRACT_INVALID` evidence is separate from the
  corrected overlap uncertainty: it affects raw GraspNet proposals, while
  the generated tabletop candidates demonstrate that the analytical
  geometry path itself can pass all six physical stages on this target.
- Isolated driver `E1` status samples continued to return immediately to
  ordinary `0x00` feedback; measured temperatures were not a sustained
  over-temperature condition. No torque-off was sent. No second-stage
  motion, contact, close, lift, `/grasp/stop`, controller stop, disable, or
  paper-carton movement command occurred.

### Active attempt ended at the near-field timeout without contact

- At ROS time `1785132559.601623`, the task naturally entered
  `FAILED NEAR_FIELD_REPLAN_TIMEOUT` after its unchanged `150.0 s` wait:
  `no fresh near-field 6D preview; last=NEAR_FIELD_PLAN_PHASE_INVALID:
  Preview is not a contact execution plan`. The execution slot was released
  as `failed`, and the blocking `/grasp/start` call returned
  `success: False, message: "failed"`.
- This was a fail-closed timeout, not a stop command. The arm remained
  enabled and held the converged first-observation pose; the gripper remained
  open at raw `995`. No second-stage motion, target contact, close, lift,
  torque-off, disable, `/grasp/stop`, controller stop, or paper-carton
  movement command occurred.
- After the slot was released, the continuously running planner returned to
  normal planning and the task subscriber accepted a different rich plan
  `3d490b366c480564b07ca605` at ROS time `1785132602.575555`
  (source stamp `1785132584448597192`). It has not been executed: a post-
  timeout far-field plan must not be confused with the expired task's
  near-field continuation.
- Driver telemetry continued to show approximately `0.001230 rad` maximum
  held joint residual and mostly `44-45 C` maximum temperature. One isolated
  vector contained a `79 C` element and was counted as only `1/3` over-limit;
  the immediately following temperature frame returned to maximum `45 C`
  and feedback status `0x00`. No torque-off was sent.
- Next diagnostic work is confined to recorded planner/task evidence and
  source semantics: distinguish insufficient near-field stability time from
  a candidate-geometry contract failure before changing any behavior. No
  threshold will be relaxed and no target-specific coordinates will be
  introduced.
- The user then stated that the arm would be moved manually back to the
  initial position. The previous task was already inactive. During this
  manual move, no automatic task, old observation-pose plan, gripper command,
  stop, torque-off, or disable command will be issued; offline evidence and
  generalized software diagnosis continue until the user reports arrival.
- The user subsequently reported arrival at the initial position. The
  expired observation-pose plan remains non-authoritative and will not be
  reused. A new grasp execution will remain withheld until the preserved
  near-field failure is diagnosed and any generalized correction is tested;
  the next execution must begin from a newly aligned target and a new,
  fresh far-field candidate window.

### Preserved near-field funnel identifies pregrasp reachability as the main blocker

- Parsed the immutable pipeline-metric lines for target epoch `11`, requests
  `11-23`, instead of inferring the failure from the final timeout message.
  Thirteen completed near-field requests were available. Their end-to-end
  processing times ranged from approximately `2.0 s` to `22.5 s`, so the
  `150 s` task window did contain repeated complete batches.
- Requests `15`, `17`, and `21` are decisive:
  - request `15`: six stable candidates, two passed the final hard recheck,
    both strict MoveIt candidates were unreachable;
  - request `17`: six stable candidates, four passed the final hard recheck,
    all four strict MoveIt candidates were unreachable;
  - request `21`: twelve stable candidates, eight passed the final hard
    recheck, all eight strict MoveIt candidates were unreachable.
- Thus at least fourteen candidates survived cross-frame stability and current
  physical geometry, but strict planning accepted `0/14`. The motion gateway
  independently logged all eight request-21 failures at exactly
  `failed_stage=pregrasp`, with
  `strict sequence pregrasp unreachable: strict pose planning failed`.
- Contact overlap remains a real secondary discriminator: request `20`
  rejected its fused stable branches at `4.541/4.455 mm` versus the live
  `4.703 mm` requirement. But it cannot explain the `0/14` MoveIt result for
  candidates that had already passed that hard geometry recheck.
- One request (`22`) also failed a historical TF lookup by `3.763 s`; adjacent
  requests completed normally, so it is not the dominant cause. The target
  was not globally lost, and request `21` alone contained `870` raw, `12`
  locally-valid, and `8` hard-rechecked candidates.
- The current diagnosis is therefore not “increase timeout” or “lower contact
  overlap”. It is the reachability of the safe near-field pregrasp pose.
  Source review shows that temporal tracking currently authorizes a synthetic
  fused pose: coordinate-wise weighted medians for center/tool position and a
  robust quaternion average, followed by physical reprojection. The newest
  real observed candidate is retained only as payload. Whether that fusion
  creates an otherwise avoidable unreachable intermediate pose is not
  preserved in the overwritten request audit, so changing execution behavior
  from this evidence alone would be guesswork.
- The next correction is audit-only: preserve and compare fused-versus-newest
  candidate pose and strict-planning outcomes on a future near-field window.
  No physical gate, overlap rule, timeout, object coordinate, target label,
  hand-eye TF, or trajectory will be changed without that evidence.

### Fused-versus-observed pose audit implemented and regressed

- Added audit-only `fusion_vs_latest_observation` evidence to every
  hard-rechecked stable variant. It records:
  source/evaluation request IDs and whether the newest track member belongs to
  the current frozen request; fused and latest-observed positions and
  quaternions; the signed fused-minus-latest position vector; its norm; and
  the sign-invariant rotation delta for the same wrist variant.
- The comparison uses the actual `NormalizedPlanningCandidate.T_base_tool0`
  retained as the track's newest measured payload and the exact physically
  reprojected fused pose sent to subsequent scoring/planning. The parallel-jaw
  half-turn is applied consistently before comparison.
- This evidence is appended to `stable_evaluations` in the immutable gate
  audit. It does not add a candidate, perform an extra MoveIt call, change
  ranking, select the latest pose, replace the fused pose, or authorize a
  plan. Any audit-computation exception is recorded as unavailable and cannot
  reject or accept the candidate.
- The first focused regression run failed two new test expectations:
  the test incorrectly expected zero translation even though the existing
  physical support-clearance reprojection may legitimately shift the fused
  pose, and a new direct helper fixture omitted its required ROS stamp.
  Production code compiled and `git diff --check` passed during that failure.
  The assertions were corrected to require finite nonnegative measured
  deltas, and the fixture was given the same complete frozen request context
  as production.
- Corrected verification:
  - focused audit tests: `2/2` passed;
  - full streaming planner: `186/186` passed;
  - full remote planner node: `154/154` passed;
  - Python compilation and `git diff --check`: passed.
- No ROS service/topic request, MoveIt request, arm/gripper motion, stop,
  torque-off, disable, controller stop, target-specific value, or threshold
  change was made during implementation and testing.

### Audit-enabled remote planner hot-loaded at the initial pose

- After the user reported the arm at its initial pose and the task remained
  inactive, interrupted only the old standalone `remote_grasp6d_node`
  terminal. Its in-flight planning request completed as
  `GENERATION_STALE`; no task, driver, controller, MoveIt process, camera,
  hand-eye TF, GUI, joint-enable path, or physical arm process was stopped.
- Started the tested planner source in terminal session `98421`. It reported
  GraspNet backend online, protocol `3`, against
  `http://172.23.132.97:8000`. Loaded source hash:
  `90cd09bb2dcaae051fa7ea901614bc39056bf3f5aac17c22559f7df658130f93`.
- The replacement planner remains in its configured untriggered/manual state;
  no inference request, candidate promotion, old-plan execution, ROS motion
  service, gripper command, stop, torque-off, disable, or arm motion occurred.
  The next candidate window must be generated from a newly aligned target.

### Fresh post-alignment candidate stream started

- After the user explicitly reported `已对准`, called
  `/grasp_6d/request_plan` with `trigger: true` on the audit-enabled planner.
  The service returned `success: True` and
  `continuous remote 6D inference started`.
- Candidate authority is deliberately limited to plans generated by this new
  post-alignment stream. No preview or plan produced by the replaced planner,
  or before this alignment report, will be sent to the task executor.
- No arm/gripper motion service, stop, torque-off, disable, controller stop,
  threshold change, or target-specific override was issued while starting the
  perception/planning stream. Live planner and task supervision began
  immediately.

### Fresh far-field plan accepted for automatic execution

- Post-alignment generation `1` began from snapshot stamp
  `1785133667.6848185`. The first request was correctly held as
  `STABILITY_PENDING`; the continuous stream was allowed to accumulate only
  current-window temporal evidence.
- The task node subsequently accepted rich execution plan
  `2182d4d6cca39fd560be836c` with source stamp
  `1785133711783233165`. Both the source stamp and acceptance occurred after
  the explicit post-alignment stream restart, so this is the first plan with
  valid authority for the current automatic execution.
- Hardware feedback at acceptance was live, gripper remained open
  (`gripper_raw=995` feedback), measured maximum temperature was `38 C`, and
  the arm was holding its aligned joint state. Isolated `E1`/`E2` status
  events were not accompanied by sustained measured over-temperature; the
  driver explicitly sent no torque-off.

### First start submission safely rejected after a live plan replacement

- Submitted the exact accepted ID `2182d4d6cca39fd560be836c` to
  `/grasp/start`. The service returned `success: False` with
  `PLAN_ID_MISMATCH`: the continuous post-alignment stream had already
  replaced the task node's current rich plan with
  `c2b9237dbc8315fa0e30795d`.
- This rejection caused no motion and confirms the start path does not silently
  execute a superseded preview. The returned current ID is still from the same
  fresh post-alignment generation; any retry must name that exact current ID
  and may not bypass the consistency check.

### Current plan started and first observation stage converged

- Retried `/grasp/start` with the task node's exact current post-alignment plan
  ID `c2b9237dbc8315fa0e30795d`. The call remained active and the task entered
  the real far-field observation workflow, confirming the guarded retry was
  accepted.
- The task identified the plan as a far-field observation plan and explicitly
  deferred contact simulation to the near-field replan. Opening was skipped
  because measured gripper feedback was already within `1 mm` of the requested
  `50 mm`; no close command was issued.
- The first arrival had measured Cartesian residual
  `17.4 mm / 2.56 deg`, outside the strict `6 mm / 5 deg` observation gate.
  The task therefore repeated the same live-derived observation pose exactly
  once. The corrected measured residual was
  `2.4 mm / 0.28 deg`, inside both limits, and the task transitioned to
  `waiting for near-field 6D preview`.
- At this checkpoint the arm is holding the observation pose; second-stage
  approach, contact, gripper close, and lift have not executed. No stop,
  torque-off, disable, or fixed object-specific pose was sent.

### Early near-field epoch-2 evidence under the converged observation pose

- The planner correctly advanced to target epoch `2`; requests `16` and `17`
  began by accumulating new near-field stability evidence rather than reusing
  far-field tracks.
- Request `18` produced `12` stable variants. Six survived the fresh
  geometry/hard recheck and all six were sent through strict MoveIt checking,
  but `0/6` were reachable. This reproduces the prior near-field bottleneck
  from the new, accurately reached observation pose.
- Request `19` contained stable tabletop branches, but their live bilateral
  target/CAD contact overlap was `1.769 mm`, below the per-observation
  uncertainty-derived requirement `5.432 mm`; all were correctly rejected.
  Later branches measured `5.205 mm`, only `0.227 mm` below that same live
  requirement, and were also rejected. The current requirement is therefore
  neither the removed fixed `6.500 mm` value nor an object-specific constant.
- Other near-field windows remained stability-pending or failed their live
  candidate/analytical-gripper contract before MoveIt. No contact preview has
  been published yet; the task remains at the converged observation pose with
  the gripper open.

### User-observed observation-pose oscillation and strict timeout

- The user reported that after reaching the first observation pose the arm did
  not become physically still, but continued small back-and-forth corrections.
  The terminal record independently confirms this is real control activity:
  after the task measured convergence at `2.4 mm / 0.28 deg`, the real driver
  continued `Endpoint feedback trim` iterations (observed from at least
  iteration `16` through `38`) and kept emitting slightly changing six-joint
  streaming commands.
- This is not a planned second-stage motion. The task state remained
  `waiting for near-field 6D preview`, while the driver-side endpoint
  correction loop continued chasing the held reference. Such motion can
  perturb near-field images, transforms, and temporal candidate association
  even though the high-level Cartesian endpoint is already inside its strict
  observation gate.
- No safe contact preview was produced within `150.0 s`. At
  `1785133995.445588` the task failed closed with
  `NEAR_FIELD_REPLAN_TIMEOUT`; the last reason remained
  `NEAR_FIELD_PLAN_PHASE_INVALID: Preview is not a contact execution plan`.
  The execution slot was released. No second-stage approach, gripper close,
  contact, or lift occurred.
- The planner then observed the task phase return to `far_field`, cancelled
  only its prior-phase inference requests, and reset its stability window.
  This planner phase change is not an arm/controller stop command.
- Working diagnosis to verify in source: the low-level endpoint feedback
  integrator lacks a bounded quiescence/exit condition tied to a settled
  reference, so it keeps applying encoder-quantized corrections after the
  high-level pose has converged. The next step is evidence-only source tracing
  before any change; no threshold, stop, torque-off, disable, or live
  controller mutation has been made.

### Endpoint-trim quiescence defect isolated in source

- While the user manually moves the arm, no ROS motion/gripper request,
  driver restart, stop, torque-off, or disable is being issued. Source and
  regression work will remain offline until the user reports a suitable
  position for any later hot-load.
- Source tracing confirms the exact mismatch:
  - trim activation correctly requires live maximum joint feedback error above
    the existing controller goal band `0.035 rad`;
  - after activation, both normal continuation and stalled-response retry
    instead require only error above one SDK encoder quantum,
    `2*pi/4096 rad` (about `0.001534 rad`);
  - the update is an accumulated integral
    `previous_trim + gain * live_error`, so encoder-quantized residuals keep
    changing the SDK reference even after the controller-level objective is
    satisfied.
- The real trace is consistent with that code path: post-convergence residuals
  varied around `0.002263-0.004427 rad`, safely below the `0.035 rad`
  controller goal but still above one-to-three encoder quanta, and therefore
  triggered repeated iterations and stalled retries.
- General correction selected for offline implementation: continuation and
  stalled retry will use the same existing controller goal-error condition as
  activation. Once live error enters that goal band, the last live,
  joint-agnostic trim remains held but its integrator becomes quiescent. It
  may update again only if live error leaves the goal band while the reference
  remains stable. This adds no object coordinate, joint-specific offset,
  grasp-stage pose, arbitrary iteration count, or relaxed Cartesian audit.

### Endpoint-trim goal-band quiescence implemented and built

- Updated the driver so one shared live predicate,
  `maximum_feedback_error_rad >
  endpoint_feedback_trim_activation_error_rad_`, controls activation,
  continuation, and stalled-response retry. Encoder movement of one quantum
  is still required as evidence that a correction actually produced a
  hardware response, but encoder quantization is no longer the controller
  objective that keeps the integrator moving.
- The last accumulated live trim remains applied while quiescent. A stable
  reference can therefore retain compensation for repeatable actuator bias
  without dropping the offset or moving back toward the biased raw target.
  If measured error later leaves the controller goal band, the same generic
  loop can resume from fresh feedback.
- Strengthened the regression contract: both real-driver launch files must set
  the trim activation/error objective equal to the minimum six-joint
  controller goal tolerance, and both continuation and stalled retry must use
  the shared goal-band predicate rather than the SDK quantum.
- Verification completed with no ROS publication or hardware mutation:
  configuration/implementation tests passed `11/11`; `git diff --check`
  passed; `catkin_make --pkg alicia_d_driver -j2` compiled and linked the C++
  driver. New binary:
  `devel/lib/alicia_d_driver/alicia_d_driver_node`, SHA-256
  `6717bc1ddf6a9f976b1e86827f146905e76283937d1b44883298c950f4c3278a`.
- The user then reported the target realigned. The new binary is not yet
  running at this log point; the existing driver remains enabled and no
  restart, motion, gripper, stop, torque-off, or disable command has been sent
  by this implementation/build step.

### Quiescence-corrected driver hot-loaded at the realigned target

- Ended only the old driver terminal with `SIGINT`. Its shutdown completed
  cleanly; it did not publish or transmit torque-off, stop, disable, or
  demonstration mode.
- Started the newly linked binary as `/alicia_d_driver_node` in terminal
  session `33513`. It reopened `/dev/alicia_arm` at `1000000` baud and reported
  the preserved `auto_torque_on_startup=true` path.
- Fresh hardware feedback remained stable near
  `[-113.4,+14.3,-6.8,-4.3,+0.6,+13.1] deg`, with open gripper
  `raw=995`, ordinary status `0x00`, and measured maximum temperature about
  `40-41 C`. No old joint command was replayed and no SDK trajectory frame was
  emitted during the observed startup interval.
- A one-sample read of `/alicia_d/motion_enabled` returned `data: True`,
  confirming positive enable after the hot-load. No stop, torque-off, disable,
  arm target, gripper target, or old grasp-plan execution was sent.

### Fresh post-fix, post-alignment candidate authority established

- With the task inactive after the prior strict timeout, stopped only the old
  continuous inference stream via `/grasp_6d/request_plan trigger:false`.
  The planner returned `continuous remote 6D inference stopped`, which clears
  its old candidate tracker/generation; this is not an arm, controller, or
  enable stop.
- Immediately started a new stream with `trigger:true`; the planner returned
  `continuous remote 6D inference started`. Only snapshots and temporal tracks
  captured after the quiescence-corrected driver was online and after the
  user's new alignment have execution authority.
- No previous rich-plan ID may be executed. The next `/grasp/start` call must
  name an exact plan newly accepted by the task from this restarted stream.
  No arm/gripper motion, stop, torque-off, disable, threshold relaxation, or
  object-specific override was issued by the stream reset.

### Fresh post-fix far-field plan accepted

- The restarted authority is planner generation `3`, target epoch `5`.
  Requests `68` and `69` correctly accumulated temporal evidence as
  `STABILITY_PENDING`; request `70` had `9` fresh hard-pass variants but
  strict MoveIt reachability `0/9`, so it published no preview.
- A subsequent fresh request produced a valid preview and the task accepted
  rich plan `e0fd488b0f2778e53c019915`, source stamp
  `1785134615830651760`, at `1785134628.268567`. This acceptance is after both
  the candidate-generation reset and quiescence-corrected driver hot-load.
- Later request `72` again failed strict reachability (`0/9`) and published no
  replacement preview. The accepted ID therefore remains the newest observed
  execution-authorized plan at the time of submission.

### Post-fix execution proves quiescence but falsifies goal-band precision

- `/grasp/start` accepted exact fresh plan
  `e0fd488b0f2778e53c019915`. The task entered the far-field observation
  workflow; opening was skipped because the gripper was already open.
- First arrival residual was `17.1 mm / 2.36 deg`, so the task issued its one
  bounded same-live-pose correction. Under the new driver, endpoint trim
  activated exactly once at maximum joint feedback error
  `0.044018 rad`, applying live per-joint offsets
  `[-0.6,+2.5,+1.1,-0.2,-0.1,+0.5] deg`.
- After the encoder response, maximum live joint error dropped to
  `0.014273 rad`, below the existing `0.035 rad` controller goal band. The new
  logic then held `iteration=1` and the exact same offsets for more than ten
  seconds. No additional trim iteration or stalled-response retry occurred.
  This directly verifies that the observed back-and-forth integrator motion
  was removed.
- The stricter Cartesian result falsifies the chosen quiescence boundary:
  measured endpoint residual stopped at `7.9 mm / 1.13 deg`, still `1.9 mm`
  outside the unchanged `6 mm` position contract. A five-second hold audit
  confirmed that it did not converge further. The task therefore failed
  closed with `OBSERVATION_ENDPOINT_NOT_CONVERGED` and released its execution
  slot without entering near-field planning.
- No second-stage motion, contact, close, or lift occurred. The `6 mm`
  Cartesian gate will not be relaxed. The evidence means controller goal
  tolerance is a valid activation bound but is too coarse to serve directly
  as the endpoint-trim quiescence threshold.
- Next design constraint: preserve the now-proven quiescent latch, but derive
  terminal precision from hardware encoder resolution and measured
  closed-loop response rather than the broad trajectory-controller goal band.
  It must remain joint/pose/object agnostic and must not return to indefinite
  single-quantum chasing.

### Quantization-floor hysteresis selected from paired real traces

- The user is moving and realigning the arm. No driver hot-load or ROS
  motion/gripper request will occur during that manual operation; source and
  tests proceed offline.
- The prior oscillatory trace gives the missing response sequence for the same
  class of endpoint correction: maximum joint error progressed from
  `0.040846` to `0.012843`, then `0.003640`, and later `0.002021 rad`.
  The new goal-band trace stopped after the analogous first response at
  `0.014273 rad`. Therefore the live controller has demonstrated that another
  bounded response can enter the few-quantum region; stopping at
  `0.035 rad` discarded measurable precision.
- The SDK command encoder and hardware feedback encoder each quantize at
  `2*pi/4096 rad`. Their round-trip comparison has a derived resolution floor
  of two such quanta, about `0.003068 rad`. This is hardware/protocol
  uncertainty, not a target-object or grasp-pose constant.
- Revised control design:
  - before quiescence, continue response-gated integral trim while stable live
    error exceeds the two-quantum round-trip floor;
  - latch quiescent once a fresh, settled response is at or below that floor
    and hold the last live-derived offsets;
  - while latched, ignore small residual motion/quantization fluctuations;
  - leave the latch only if stable live error exceeds the existing
    `0.035 rad` controller goal band.
- This Schmitt-style separation preserves precision and prevents chatter. It
  contains no fixed object coordinate, fixed joint offset, fixed number of
  iterations, or relaxed `6 mm` Cartesian contract.

### Quantization-floor hysteresis implemented offline while the user realigns

- Per the user's instruction, no ROS arm/gripper target, driver restart,
  stop, torque-off, disable, or candidate execution was issued while the arm
  is being moved and aligned manually.
- Added the protocol-derived constant
  `ENDPOINT_FEEDBACK_TRIM_ROUND_TRIP_FLOOR_RAD =
  2 * (2*pi/4096)`. The factor of two represents the independent quantization
  of the outgoing SDK command and returned joint feedback; it is not an
  object-specific position, learned carton offset, or tuned grasp-stage
  coordinate.
- Added explicit per-reference quiescence state. Before latching, a stable
  live endpoint continues response-gated integral correction only above the
  round-trip floor. At or below the floor, the driver latches quiescent and
  holds the last live-derived offsets. It leaves that latch only when fresh,
  stable error exceeds the existing controller activation band
  `0.035 rad`, providing hysteresis against encoder-level chatter.
- A new command reference, or either existing command-state reset path, clears
  the quiescence state together with the other trim state. Runtime logs now
  identify one-time latch entry/exit and include both latch state and the
  derived floor in the throttled hold trace.
- Regression assertions now require the two-quantum derivation, separate
  activation and continuation predicates, quiescence entry/exit conditions,
  state reset coverage, and continued one-quantum hardware-response evidence.
  The focused configuration/implementation suite passed `11/11`,
  `git diff --check` passed, and
  `catkin_make --pkg alicia_d_driver -j2` compiled and linked successfully.
- The resulting driver binary is
  `devel/lib/alicia_d_driver/alicia_d_driver_node`, size `4667440` bytes,
  SHA-256
  `6021850a72542e7487a5e1fae4695d1d881de6af3e84a2db8234955d9e1e76de`.
  It is built but is not yet hot-loaded into the live driver; that remains
  deferred until the user explicitly reports that manual alignment is
  complete.

### 2026-07-27 00:38 PDT full latest ROS stack launched after realignment

- After the user explicitly reported the target aligned, launched the complete
  current worktree directly with the log-documented command:
  `roslaunch alicia_flexible_grasp_supervisor full_system.launch
  start_real_arm:=true driver_port:=/dev/alicia_arm
  driver_baudrate:=1000000 auto_torque_on_startup:=true
  self_check_poll_rate_hz:=0.0 start_camera:=true start_tactile:=false
  start_gui:=true use_remote_grasp6d:=true
  remote_grasp6d_url:=http://172.23.132.97:8000`.
- The supervised launch terminal is session `24977`; roslaunch PID is `64620`
  and the newly built quantization-hysteresis driver is PID `64656`. The
  driver opened `/dev/alicia_arm` at `1000000` baud, entered the configured
  automatic torque-on path, and continuously reports real joints near
  `[-108.4,+27.8,-11.6,-3.7,-8.1,+1.1] deg`, open gripper raw `995`, and
  ordinary `0x00` status. It explicitly treats isolated `E1` bytes as status
  events and reports that no torque-off is sent without sustained measured
  over-temperature.
- MoveIt loaded OMPL, CHOMP, and Pilz and reported `You can start planning
  now!`; the task, remote planner, motion gateway, GUI, perception, and both
  trajectory controllers were started. Electronic skin remains excluded with
  `start_tactile:=false` and compliant close/force feedback remain disabled.
- The new launch asked the old same-name ROS camera and hand-eye processes to
  shut down. Their stdout was attached to a vanished terminal, so the old
  processes raised `OSError: [Errno 5]` and remained orphaned as PIDs `6374`
  and `6377`; old camera PID `6374` retained the RealSense device. Terminated
  only those two already-shutdown orphan processes with `SIGTERM`. No driver,
  hardware interface, controller, joint-enable process, arm target, gripper
  target, stop, disable, torque-off, or `/grasp/stop` was touched.
- The replacement camera then recovered by itself and started real aligned
  color/depth streams. Perception is stable near UV `(312,253-254)`, depth
  `0.336-0.339 m`, and base position about `(-0.149,-0.457,0.067) m`.
- The replacement hand-eye node loaded the required unchanged pruned-19
  calibration from the easy_handeye runtime file: xyz
  `[-0.0866127223,-0.0063407198,-0.1034120675] m`, quaternion
  `[0.0229081321,-0.6787668594,0.0278470194,0.7334680031]`.
- Full launch restored `/grasp/calibration_interlock_active=true`. An attempted
  compound request to renew the prior temporary pruned-19 override and reset
  candidate authority was rejected by the execution permission boundary
  before any segment ran, because disabling a calibration interlock requires
  separate explicit operator approval after the risk is stated. Therefore the
  interlock remains active and the candidate stream was not changed by that
  rejected request. Planning-only candidate generation may proceed with the
  interlock intact; physical execution must remain blocked unless the operator
  explicitly renews the temporary override.

### Fresh aligned-target candidate authority established under active interlock

- With the calibration interlock left active, requested the old planning stream
  off; it reported `already stopped`. Started a new continuous inference stream,
  which established planner `generation=1`, target `epoch=2`. These calls only
  reset/start image inference and did not issue an arm, gripper, controller,
  stop, torque-off, or disable command.
- Requests `1` and `2` accumulated the required temporal evidence as
  `STABILITY_PENDING`. Request `3` used snapshot stamp
  `1785138130.555996417`, passed eight stable candidates through live hard
  recheck, sent six through strict MoveIt, and found `2/6` reachable.
- The task accepted fresh rich execution plan
  `cafff4b270fe3269cf3dbd2a`, source stamp
  `1785138130555996417`. Pipeline status was `PREVIEW_READY`; the audit file
  was atomically committed with SHA-256
  `77bdb5f16facf0ac95668cb52f0978808b1e595963dd6d1939db7e91bbf2bf15`.
  Request `4` found no reachable replacement, so the named plan remains the
  latest accepted plan at this point.
- No execution was submitted because `/grasp/calibration_interlock_active`
  remains `true`. The permission boundary requires the operator to explicitly
  approve renewing the temporary pruned-19 override after being told that this
  removes the software calibration block for real-arm motion. Continuous
  planning remains active so a fresh replacement can be used after approval.

### Operator explicitly renewed the pruned-19 temporary interlock override

- The operator explicitly instructed the system to remove the calibration
  interlock according to the prior log after being informed that doing so
  permits real-arm motion using the pruned-19 hand-eye transform and that
  residual calibration error can cause positioning or collision risk.
- Set `/grasp/calibration_interlock_active=false` and recorded reason
  `USER_OVERRIDE_PRUNED19_TF_TEMPORARY_RENEWED_20260727_AUTOMATIC_EXECUTION`.
  The hand-eye TF itself was not changed. No arm/gripper trajectory, stop,
  torque-off, disable, controller-stop, or `/grasp/stop` command was sent by
  this parameter update.
- Because continuous inference remained active during the approval exchange,
  execution will bind only to the newest subsequently observed valid plan ID;
  it will not assume that the earlier preview is still current.
- Before submission, request `7` produced a newer valid preview: strict MoveIt
  found `1/6` reachable and the task accepted plan
  `221280546588438af69f8c0c`, source stamp
  `1785138241738666296`. Its audit SHA-256 is
  `b93f96e65bbad05c8adb2024f7cd3522786c00cc8886ec58e737ae18109b4582`.
  This newer exact ID supersedes `cafff4b270fe3269cf3dbd2a` for execution.

### Quantization-hysteresis real execution reached strict first observation

- Submitted `/grasp/start` with `execute=true` and exact plan
  `221280546588438af69f8c0c`. The task accepted it, skipped opening because the
  gripper was already open, and executed the live-derived fixed-100-mm
  observation pose. The main retimed trajectory took `24.750 s`, maximum joint
  delta `1.238 rad`, and completed `SUCCEEDED`.
- The first measured endpoint residual was `16.2 mm / 2.35 deg`, so the task
  used its single bounded retry of the identical live observation pose. The
  correction trajectory was `3.000 s` with only `0.048 rad` maximum delta and
  completed `SUCCEEDED`.
- The post-correction endpoint measured `2.9 mm / 0.46 deg`, inside the
  unchanged `6 mm / 5 deg` contract. The task therefore entered
  `PLAN_PREGRASP` and began waiting for a near-field plan; it did not bypass
  the measured endpoint gate.
- The new driver activated at live maximum joint error `0.040577 rad`. It
  continued response-gated corrections down through `0.010044` and
  `0.005296 rad`, then measured `0.001674 rad`, below the derived
  `0.003068 rad` round-trip floor. At iteration `7` it emitted the one-time
  `entered quantization-floor quiescence` event and has since held exactly the
  same live-derived offsets with `quiescent=true`, no new trim iterations, and
  no back-and-forth correction. This is the first real trace validating both
  the added precision and the chatter-prevention latch.
- Near-field generation is using target epoch `3`. Early requests are still
  `STABILITY_PENDING` or candidate-invalid; contact, close, and lift have not
  yet been commanded. No stop, torque-off, disable, controller-stop, or
  `/grasp/stop` command has been issued.

### 2026-07-27 near-field closed-loop timeout after a stable first observation

- The blocking `/grasp/start` call for plan
  `221280546588438af69f8c0c` returned `success: False`, `message: "failed"`.
  The task's exact terminal evidence at ROS time `1785138475.355711` was
  `NEAR_FIELD_REPLAN_TIMEOUT: no fresh near-field 6D preview after 150.0s;
  last=NEAR_FIELD_PLAN_PHASE_INVALID: Preview is not a contact execution
  plan`. The execution slot was released and the planner returned to
  `far_field`; it did not silently continue toward contact.
- The first-observation endpoint remained within the measured strict contract
  after its one bounded correction (`2.9 mm / 0.46 deg`, required at most
  `6 mm / 5 deg`). The driver remained in quantization-floor quiescence at
  `0.001674 rad`, holding the same live-derived offsets
  `[-0.3,+2.4,+0.9,-0.4,+0.4,+0.2] deg` with no additional trim iterations.
  Therefore this attempt supplies real evidence that the former first-stage
  endpoint error and continuous small correction are no longer the cause of
  the second-stage delay.
- During the 150-second near-field window, repeated fresh GraspNet results did
  contain candidates that passed the hard geometry recheck, but none became a
  valid reachable contact execution preview. Strict MoveIt repeatedly found
  the near-field pregrasp unreachable. Other returned candidates were rejected
  explicitly as `invalid analytical gripper input: jaw axis must be parallel
  to support plane`; they were not repaired with an object-specific pose.
- The strongest live bilateral-contact-overlap examples were `5.818 mm` and
  `5.400 mm` against a frame-derived requirement of `5.936 mm`. The closest
  case measured `5.932 mm`, only `0.004 mm` below that same live requirement,
  and was correctly rejected. `5.936 mm` was not a written fixed constant for
  this carton: it was the current geometry-derived value. Relaxing or rounding
  it merely to admit this one result would violate the generalized,
  repeatable-contact requirement, so no threshold change is justified by this
  evidence.
- Near-field visibility audits repeatedly reported `baseline-visible=0` while
  perception placed the target at the lower image boundary, approximately
  bbox `(299-300,347,95-96,133)` with center near UV `(349,415)` and depth
  about `0.192 m`. After task failure and phase reset, a far-field audit again
  observed visible variants. This correlation is a diagnostic lead, not yet a
  claimed cause: the next source/runtime analysis must determine whether the
  100-mm observation construction uses a live camera/frustum constraint and
  whether candidate reachability failure is independent of the clipped view.
- No contact trajectory, gripper close, lift, stop, torque-off, disable,
  controller-stop, or `/grasp/stop` command occurred. The arm remains enabled
  and stationary at the first observation pose. The next work is evidence-led
  offline inspection of the near-field visibility and strict-MoveIt candidate
  construction; another real execution must not be attempted until that
  generalized failure is either explained or corrected.

### Offline root-cause trace while the operator manually realigns the arm

- The operator reported that the arm is being moved and realigned. During this
  manual operation, no arm/gripper goal, driver restart, hot-load, stop,
  torque-off, disable, or new execution request is permitted. Existing
  perception/candidate generation may remain online, but it has no active task
  execution slot.
- The current streaming path constructs two different trajectories for every
  stable candidate: the adaptive contact sequence and the fixed-100-mm
  far-field observation sequence. Strict MoveIt correctly checks only the
  observation pose in far-field mode and the four adaptive stages in near-field
  mode. The visibility data path is not phase-consistent:
  - `_recheck_and_score_stable()` supplies the adaptive contact sequence to
    `_common_soft_features()` even while scoring a far-field observation;
  - the raw gate audit and the legacy hard visibility helper call
    `_candidate_visibility_metrics()` without the already materialized
    sequence, causing it to reconstruct legacy `pregrasp_distance_m` /
    `final_approach_offset_m` values instead of auditing the exact trajectory;
  - most importantly, the streaming selector never applies the configured
    camera-visibility result as a hard gate. It enters only a finite soft cost.
- Far-field MoveIt ranking then uses `observation_translation_delta_m` as the
  first sort key and the total soft score only as the second key. Therefore a
  shorter reachable observation move can win even if its predicted eye-in-hand
  view is outside the configured full-target margins. This explains the
  otherwise contradictory real trace: the selected far-field plan passed
  strict MoveIt, but the target moved from approximately UV `(312,254)` at
  depth `0.338 m` to UV `(349,415)` at depth `0.192 m`, with its bbox touching
  the 480-pixel image boundary. The target's base estimate remained in the
  same local workspace; no target motion is required to explain the pixel
  displacement.
- This is a generalized planning-contract defect, not a carton-specific
  offset. The intended correction is to evaluate the exact phase trajectory,
  require full target visibility at the actually executed 100-mm observation
  pose before a far-field candidate can enter MoveIt selection, and retain the
  live-derived adaptive sequence for near-field diagnostics. The fixed 100-mm
  observation standoff, pruned-19 TF, `6 mm / 5 deg` endpoint gate, live
  contact-overlap rule, and strict MoveIt rules will remain unchanged.

### Phase-exact visibility authority implemented and tested offline

- Extended the immutable per-variant `SafetyGateInput` with explicit
  `visibility_required` and `visibility_valid` evidence. When the configured
  visibility gate applies, anything other than strict `True` now fails the
  mandatory pre-MoveIt gate as `CAMERA_TARGET_OUT_OF_VIEW`. When visibility is
  diagnostic-only, a failed prediction remains diagnostic and does not become
  an accidental hard requirement.
- The streaming stable-candidate recheck now materializes both trajectories
  before scoring and chooses the exact one authorized by the phase:
  - far field uses the fixed-100-mm observation sequence;
  - near field uses the current geometry/depth-repeatability/endpoint-error
    derived contact sequence.
  That same selected sequence feeds the visibility hard evidence, visibility
  soft centering cost, and bounded audit data. A shorter MoveIt-reachable move
  can no longer outrank a candidate that fails full-target visibility.
- The raw GraspNet gate audit now accepts the current immutable
  `PreparedPrediction`, rebuilds the phase-exact sequence from the same frozen
  geometry/snapshot, and records its projected visibility metrics. Sequence
  construction failure is recorded as an explicit invisible reason rather
  than silently falling back to legacy `36 mm / 20 mm` stage values. Stable
  audit evidence additionally records phase, required/valid state, reason,
  and projected stage metrics.
- Added a warning for every stable variant that cannot complete the recheck;
  the prior broad exception handler silently discarded these variants and made
  runtime diagnosis unnecessarily ambiguous. This changes diagnostics only,
  not acceptance criteria.
- Regression results so far:
  - `test_grasp6d_pipeline.py`: `168 passed`, including required-visible,
    required-invisible, and diagnostic-only visibility contracts;
  - `test_remote_grasp6d_node.py`: `154 passed` before the final audit
    sequence-identity assertion was added;
  - `test_remote_grasp6d_streaming.py`: `186 passed`, including proof that
    far-field hard and soft visibility consumers receive the exact observation
    sequence and not the contact sequence;
  - `git diff --check` and Python bytecode compilation passed.
- Two first targeted ROS-aware pytest commands failed during collection because
  the new shells had not sourced this worktree's `devel/setup.bash`; no test
  body ran. Re-running in the correct environment passed. A subsequent test
  fixture temporarily exposed an intentionally inconsistent synthetic
  quaternion/approach pair (`+Z` pose versus `-Z` evidence); the test was made
  explicitly far-field and the production construction retained the fused
  candidate's live approach axis. No object-specific direction was added.
- These edits are offline only while the operator moves/re-aligns the arm. The
  live ROS Python process has not been restarted or hot-loaded, and no arm or
  gripper command has been sent.

### Broader offline regression boundary after phase-exact visibility changes

- Added an audit regression proving that `_candidate_gate_audit_row()` receives
  and records the caller's exact materialized sequence rather than rebuilding
  a legacy sequence. Its first attempt saw an extra `None` because the same
  test intentionally called the older target-match helper first; clearing the
  observation list at the audit boundary made the scope exact, and the test
  then passed.
- Rebuilt `alicia_flexible_grasp_supervisor` successfully with `catkin_make
  --pkg alicia_flexible_grasp_supervisor -j2`; this generated the existing
  `CheckPoseSequence` Python service artifacts in the worktree devel space.
  The build did not restart or signal any live ROS process.
- Full-package pytest collection then exposed that
  `test_motion_gateway_gripper_hold.py` still stubbed the older service import
  set. Added `CheckPoseSequence` and `CheckPoseSequenceResponse` to that test's
  local stub only; production service behavior was not changed.
- The subsequent full package run completed with `1589 passed`, `19 failed`,
  `1 skipped`. All directly affected suites remained green:
  `test_grasp6d_pipeline.py` `168/168`,
  `test_remote_grasp6d_node.py` `154/154`, and
  `test_remote_grasp6d_streaming.py` `186/186`.
- The remaining full-suite failures are outside the changed visibility path:
  local HTTP protocol tests received sandbox `PermissionError: [Errno 1]` when
  creating loopback sockets; two RealSense-fixture tests explicitly require
  the absent genuine file `tests/fixtures/carton_tabletop_cloud.json`; and one
  GUI/config test still expects older continuous-stream values such as
  `request_hz=1.5` while the current logged runtime configuration deliberately
  contains `0.1`. No production constant or grasp criterion will be altered to
  make those unrelated tests pass. The socket-dependent protocol modules will
  be rerun with loopback permission to separate environment failures from real
  regressions.

### Loopback protocol regressions and live hot-load boundary

- Re-ran the two socket-dependent protocol modules with permission for their
  local loopback HTTP fixtures. Result: `135 passed, 1 skipped in 6.68 s`.
  This proves the prior protocol failures were sandbox socket restrictions,
  not regressions in the GraspNet or MuJoCo request/response contracts.
- `git diff --check` remains clean. The launch definition confirms
  `/remote_grasp6d_node` is not configured with `respawn`; therefore the live
  update will stop and replace only this inference/planning process. The arm
  driver, motion gateway, controllers, joint torque/enable state, cameras, and
  GUI will remain untouched. No prior plan will be reused after replacement.
- The operator has reported the arm and target are aligned. The next authority
  must be generated after the planner replacement from that live view, and a
  physical request may proceed only if the new phase-exact far-field
  visibility gate and the unchanged strict planning gates accept it.

### Phase-exact planner hot-loaded on the newly aligned target

- Stopped only the old `/remote_grasp6d_node` and started the corrected node
  from this worktree. The launch file has no respawn for that node, so the new
  standalone PID is `76940`. The real-arm driver, motion gateway, controllers,
  MoveIt, task node, GUI, camera, perception, hand-eye publisher, arm pose, and
  joint enable/torque state were not restarted or commanded.
- The replacement reported the remote `graspnet_baseline` backend online with
  protocol version 3. It inherited manual-trigger mode, so explicitly started
  a new inference stream. This created `generation=1`, `target_epoch=1`; every
  previous plan became non-authoritative. The temporary pruned-19 calibration
  override remains `false` with the previously authorized reason.
- Through request 20 the new stream has published no preview and therefore no
  arm motion has been submitted. Typical settled frames have 12 stable
  tabletop variants, six passing the hard recheck and entering strict MoveIt,
  and `0/6` MoveIt-reachable. The task remains inactive.
- The new phase-exact evidence is working as designed. Rejected variants now
  explicitly include `CAMERA_TARGET_OUT_OF_VIEW`, while all six variants sent
  to MoveIt have `visibility_required=true`, `visibility_valid=true`, reason
  `visible`. Their predicted target centers at the fixed-100-mm observation
  stage are approximately `u=317-321 px`, `v=399-400 px`, with configured
  margins `56 px` and `63 px`; thus these six are not visibility false
  negatives.
- MoveIt's exact failure is instead `Unable to sample any valid states for goal
  tree` / `MOVEIT_UNREACHABLE`. The tested observation positions cluster near
  `xyz=(-0.162,-0.454,0.152) m` with different live-derived orientations.
  Current perception remains stable near UV `(299,219)`, depth about
  `0.306 m`, target base approximately `(-0.156,-0.454,0.066) m`, and bbox
  around `(269,184,63,72)`. No object motion or carton-specific coordinate is
  being assumed.
- The strict planning threshold, observation distance, target pose, pruned-19
  TF, collision constraints, and contact-overlap rule have not been relaxed.
  Diagnosis now compares this current generalized pose family against earlier
  reachable live families before deciding whether any code change is
  justified.

### Missing adaptive interior tilt family identified

- Historical ROS evidence from the same aligned joint state proves this is an
  endpoint-intersection problem, not simply insufficient MoveIt time. Before
  the visibility hard gate was loaded, request 124 checked 12 observation
  poses: three approximately 45-degree tilted poses near
  `y=-0.406 m, z=0.135 m` were strictly reachable, while the top-down and
  opposite tilted branches were unreachable. Reprojection with the current
  target and pruned-19 eye-in-hand transform shows those reachable endpoint
  poses would put the target beyond the bottom image boundary, so restoring
  them would repeat the prior failure mode and is not acceptable.
- The frozen request diagnostics expose the actual missing search space. The
  adaptive stage generator derives `0, 11.25, 22.5, 33.75, 45 deg` from the
  live object height, same-pixel depth repeatability, CAD finger length, and
  object-independent physical limits. However, far-field materialization
  reported only `materialized_total_count=24` using boundary tilt `45 deg`;
  `_contact_boundary_tilts()` replaced the adaptive interior grid instead of
  augmenting it. Thus visibility and strict IK were offered only endpoints,
  even though the runtime had already computed the scientifically relevant
  interior samples.
- Corrected materialization to use the deduplicated union of all adaptive
  profile tilts and each continuously solved contact-safe boundary. Existing
  overlap filtering still removes any unsafe near-field angle. The bounded
  per-proposal ordering now retains one vertical branch alongside the live
  interior grid, so neither an endpoint nor an interior sample can consume the
  entire candidate budget. No object label, workspace coordinate, carton
  dimension, fixed joint pose, or special grasp angle was added; only the
  explicitly agreed observation distance remains fixed at `100 mm`.
- Added a regression requiring every live adaptive profile tilt to be present
  in the bounded materialized candidate set. The visibility hard gate, CAD
  collision gates, live contact-overlap rule, and strict MoveIt acceptance
  remain unchanged. This edit is not yet loaded into the live planner and has
  issued no motion.
- The first complete streaming regression after this edit failed four existing
  generation tests (`182 passed, 4 failed`). All four returned an empty
  candidate tuple because the pure materializer still capped its input at four
  positive tilts; the new set can contain four adaptive samples plus up to four
  independently solved boundary samples. This was a deterministic internal
  contract failure, not a collision/visibility/MoveIt result, and nothing was
  hot-loaded.
- Raised only the internal materialization capacity to the structural maximum
  of eight (`4 adaptive + 4 boundary`). The ROS/user configuration parser
  remains capped at four tilt inputs, so this does not create an unbounded or
  manually tuned search. The physical range remains `(0,45] deg` and the final
  candidate batch remains bounded by the existing live `max_candidates`.
- The second streaming run improved to `185 passed, 1 failed`; the separate
  tabletop/node suites passed `178/178`, and bytecode compilation plus
  `git diff --check` passed. The remaining assertion had invoked the default
  near-field contact phase while requiring every unconstrained adaptive angle
  to survive. That is physically wrong: near-field must still discard angles
  beyond the live contact-overlap boundary. Restricted this new coverage
  assertion to `FAR_FIELD_OBSERVATION_PLAN`; the existing near-field tests
  continue to require every retained candidate's measured overlap to meet its
  live uncertainty.
- Final offline results after correcting the phase-specific assertion:
  `test_remote_grasp6d_streaming.py` passed `186/186`; the combined
  `test_grasp6d_pipeline.py`, `test_tabletop_geometry_candidates.py`, and
  `test_remote_grasp6d_node.py` run passed `346/346`. Python bytecode
  compilation and `git diff --check` also passed. The live planner is still the
  pre-interior-grid process at this checkpoint; no motion has been submitted.

### Adaptive interior grid produces the first visibility/reachability intersection

- Replaced only `/remote_grasp6d_node` again and started a new inference
  `generation=1`, `target_epoch=1`. The driver, controllers, enable state,
  task node, MoveIt, camera, GUI, and arm pose remained live and unchanged.
- Request diagnostics now prove the fix is active. Depending on the frozen
  target geometry, profiles were derived near
  `0,10.68,21.36,32.05,42.73 deg`; every proposal recorded that full
  `materialization_tilts_deg` set. The total materialized search expanded to
  54 while the existing stratified cap retained exactly 24 candidates.
- Request 6 sent 12 phase-visible/collision-free candidates into strict
  MoveIt. Ten were unreachable and two were plan-reachable but exceeded the
  unchanged `2.150 rad` maximum joint-delta gate, including one measured at
  `3.025 rad`; they were correctly rejected rather than used to justify
  weakening the limit.
- Request 9 then found a true constraint intersection: 12 candidates entered
  strict MoveIt, one was reachable inside the existing joint-delta limit, and
  the pipeline reported `PREVIEW_READY` with selected/promoted/preview counts
  all `1`. The new exact plan is `056c6f13030f99f52e61e298`, audit SHA-256
  `06ddaac785786faee593f243b134d4b8a1ca6fe3740827a0be39f8911f3b3a1d`.
- Its live-derived fixed-100-mm observation pose is approximately
  `xyz=(-0.156259,-0.428987,0.150681) m`, quaternion
  `(0.777066,0.588118,-0.151555,0.165278)`. Required live opening is
  `44.373 mm`; geometry came from five fused frames and 525 current object
  points. The target footprint passes the new hard visibility gate.
- Froze the candidate stream with `trigger=false`; this stops inference
  replacement only. The authoritative enriched plan after freezing still has
  the same exact ID and `FAR_FIELD_OBSERVATION_PLAN` diagnostic. The task is
  inactive and ready to bind that ID. No arm motion had begun at this
  checkpoint.
- The first `/grasp/start` CLI attempt supplied the two YAML fields as separate
  shell arguments. `rosservice` rejected it locally with `field execute is not
  a bool`; no valid service request reached the task and no motion occurred.
  The same frozen ID will be retried as one typed YAML mapping.
- The correctly typed retry was rejected as `PLAN_STALE` before execution; no
  motion occurred. Inspection shows the task uses the existing
  `/grasp/plan_validity_sec=120.0` configuration (the unrelated queried name
  `/grasp/grasp6d_plan_max_age_sec` is unset). The source frame aged past that
  window while the new intersection was being diagnosed. The expired ID is
  discarded; the validity window is not being extended or bypassed.

### Fresh-plan search after the expired preview (2026-07-27)

- Restarted only streaming inference after discarding the expired plan. The
  coordinator is now at `generation=3`, `target_epoch=3`; this invalidates all
  earlier authority and deliberately prevents reuse of plan
  `056c6f13030f99f52e61e298`. The arm driver, controller, GUI, enable state,
  camera, task node, MoveIt, and pruned-19 transform were not restarted or
  changed. No arm command was issued.
- A 110-second read-only watch produced no `PREVIEW_READY`. Request 25 provided
  11 hard-pass candidates to strict MoveIt: ten were unreachable and one was
  rejected by the unchanged `2.150 rad` joint-delta limit. Request 21 had the
  same physical pattern with 12 checks: ten unreachable and two over the
  joint-delta limit. The stream remained active and continued replacing stale
  frames; this is a sparse feasible-set search, not a frozen pipeline.
- After the user reconfirmed that the target is aligned, request 30 completed
  with current source stamp `1785141502.1949072`. Of 24 bounded materialized
  poses, 13 passed the phase-exact hard recheck and all 13 were submitted to
  strict MoveIt. Twelve were strictly unreachable and one exceeded the same
  joint-delta limit; preview/selected/promoted counts therefore correctly
  remained zero. The latest committed audit has SHA-256
  `38a584d902d7e494a7bb2abfbe977fc836c6937ae108f3d4c308f0c294afaf90`.
- No threshold, target coordinate, object size, grasp angle, joint pose, or
  plan-validity window was relaxed or fixed. The live search continues for a
  new plan that simultaneously satisfies current geometry, the full camera
  visibility margin, collision constraints, strict MoveIt reachability, and
  the existing joint-delta bound. The next valid plan will be consumed
  immediately so it cannot expire during diagnosis.

### Preview/execution authority race observed without motion

- Request 35 produced a new `PREVIEW_READY`: 14 candidates passed the
  phase-exact hard recheck, one was strictly MoveIt-reachable, and preview and
  selection counts were both one. Its preview ID was
  `2f40077d7de499972fe432fe`, source stamp `1785141557.0491698`, with audit
  SHA-256
  `ae4ea2891590d976232b5ededbc1198f14ee2e2d92b1e10909cd51cb9700cd29`.
- The stream was frozen and `/grasp/start` was called with that exact preview
  ID. The task rejected it before motion as `PLAN_ID_MISMATCH`: its current
  execution-rich authority was `2f47be8cd23dda1aa0d1c55d`. This is expected
  fail-closed binding: request 35's metrics explicitly show preview passed one
  but promotion passed zero, so a preview must not be treated as execution
  authority.
- Read the two distinct latched topics to remove ambiguity.
  `/grasp_6d/preview_plan_enriched` contained `2f400...` at source stamp
  `1785141557.0491698`; `/grasp_6d/plan_enriched` contained the actual task
  authority `2f47...` at source stamp `1785141530.586448431`. The latter was a
  valid far-field observation plan, but by the time its identity was confirmed
  it had crossed the unchanged 120-second validity window. A correctly typed
  retry with `2f47...` was rejected before motion as `PLAN_STALE`.
- Both attempts therefore issued no trajectory. The arm remains enabled and
  stationary. The corrective runtime procedure is to restart streaming and
  wait specifically for a fresh **promoted execution** result (or a fresh
  `/grasp_6d/plan_enriched` authority), then freeze and bind that execution ID;
  merely observing `PREVIEW_READY` is insufficient when promotion hysteresis
  retains a different execution authority. No gate or validity limit will be
  bypassed.

### Fresh promoted authority executed; first observation converged

- Restarted continuous inference at `generation=5`, `target_epoch=5` and
  watched specifically for `promoted.passed=1`. Request 42 provided 12
  candidates to the phase-exact hard/MoveIt path; one was strictly reachable,
  selected, previewed, and promoted. Audit SHA-256 was
  `8576322c475651fce556e7df75be5bf55150a14d476128630455440c0d1d149a`.
- Froze inference first, then read the execution authority from
  `/grasp_6d/plan_enriched`, not the preview topic. The exact fresh plan was
  `14f9311aab85e7e05fca102b`, source stamp
  `1785141779.681713581`, with live-derived observation pose approximately
  `xyz=(-0.155625,-0.429872,0.151102) m`, quaternion
  `(0.777081,0.588817,-0.153134,0.161213)`, and required opening
  `44.166 mm`. The task accepted that same ID; this resolved the earlier
  preview/execution race without changing a gate.
- The observation main trajectory completed. Its first measured endpoint
  residual was `19.5 mm / 2.78 deg`, so the task issued exactly its one
  permitted correction to the identical live-derived observation pose. The
  correction settled in `0.90 s` and measured `5.6 mm / 0.68 deg`, inside the
  unchanged `6.0 mm / 5.0 deg` contract. The state then changed to
  `PLAN_PREGRASP waiting for near-field 6D preview`; no repeated observation
  correction or back-and-forth high-level motion occurred.

### Current near-field run failed closed with live overlap and visibility evidence

- The task automatically restarted continuous inference for contact phase at
  `generation=7`, target epoch `8`. Representative requests 53-58 received
  populated live GraspNet output (hundreds of raw candidates) but generated no
  contact preview. Early requests had zero locally valid candidates; request
  58, for example, returned 37 candidates after remote collision filtering
  but retained none through the full local contract.
- The frozen near-field geometry measured object height about `18.55 mm`.
  Its live uncertainty requirement was `8.643 mm`: the physical `3 mm`
  gripper/support floor plus the measured `5.6 mm` observation endpoint error
  (the temporal repeatability term did not replace either physical term). This
  is not the removed fixed `6.500 mm` rule and was not entered for this carton.
- Tabletop boundary solving found maximum continuous bilateral contact-patch
  overlap about `5.106 mm` on the best recorded branch, below the live
  `8.643 mm` uncertainty. No safe tilt boundary existed, so the fallback
  correctly reported `GRIPPER_CONTACT_PATCH_MISS` and materialized zero
  tabletop contact candidates in that batch.
- Raw GraspNet candidates independently exposed a second constraint. Examples
  that otherwise matched the target predicted the target outside the required
  camera margins at the exact contact pregrasp (for example predicted
  `v=393.4 px` with a `116 px` vertical margin in a 480-pixel image, or beyond
  the image at `v=626.6 px`). Others had insertion tilt outside the physical
  stage-profile bounds, jaw axes not parallel to the support plane, or CAD
  palm/finger sweep collisions. These were not silently repaired.
- At ROS time `1785142037.896338224`, after the unchanged 150-second window,
  the task naturally failed with `NEAR_FIELD_REPLAN_TIMEOUT: no fresh
  near-field 6D preview; last=NEAR_FIELD_PLAN_PHASE_INVALID: Preview is not a
  contact execution plan`. The blocking start returned `success:false` and
  the execution slot was released. No second-stage trajectory, target
  contact, gripper close, lift, stop, torque-off, disable, controller stop, or
  `/grasp/stop` command occurred. The arm remains enabled at the converged
  observation pose.

### Evidence for an independent, target-centred observation construction

- Checked the full runtime endpoint record rather than inferring its direction
  from the scalar. The final observation error vector was
  `(-0.472,-0.425,+5.607) mm`, norm `5.643 mm`; it is almost entirely along
  base Z and nearly parallel to the support normal. Replacing the scalar norm
  by a directional projection would therefore not make this run pass and is
  not being used as a convenient threshold reduction.
- The immutable execution audit for plan `14f9...` exposes a more direct
  upstream defect. Its selected far-field observation predicted target
  `uv=(319.84,417.81)` at depth `208.21 mm`, with a live full-footprint vertical
  margin of `61 px` in the 480-pixel image. The allowed lower-center boundary
  was approximately `419 px`, leaving only `1.2 px` of reserve. The plan was
  technically visible but intentionally ranked/accepted at the very edge;
  after real execution the near-field input bbox extended to `y=476 px`.
- Source tracing confirms `_make_observation_sequence()` simply offsets the
  contact candidate by 100 mm along its insertion axis and preserves the
  contact wrist orientation. Although contact is replanned from a new image,
  the observation viewpoint is therefore not constructed as an independent
  camera-view problem. The soft visibility-center term can rank candidates,
  but it cannot create a centred reachable viewpoint when all candidate-tied
  viewpoints are poor.
- Performed a read-only analytic counterexample using the exact promoted
  plan, its live target center, and the preserved pruned-19 tool-to-camera
  transform. Holding the selected wrist orientation and requiring both (1)
  exact 100-mm Euclidean tool displacement from the grasp tool pose and (2)
  target location on the camera optical axis gives two line/sphere
  intersections. The nearer root has camera depth `64.22 mm` and places
  tool0 below the support plane, while the farther root is
  `xyz=(-0.130482,-0.378051,0.070720) m` at camera depth `152.81 mm` and
  projects the target exactly to the principal point.
- Submitted only the farther analytic root to
  `/supervisor/check_pose_strict` with `execute=false`. Strict MoveIt planning
  succeeded from the held observation state with joint-path cost `0.810` and
  maximum joint delta `0.626 rad`. Thus the edge view is not forced by the
  current workspace. This planning-only call issued no trajectory.
- Generalized correction selected for implementation: construct the
  far-field observation pose by the analytic optical-axis/100-mm sphere
  intersection using each live target, candidate orientation, current frozen
  hand-eye transform, and configured camera depth bounds; choose the farther
  positive root so the camera has greater clearance. Contact pose generation
  remains unchanged and is still recomputed near-field. Every resulting view
  must still pass full-footprint visibility, collision, strict MoveIt, and
  joint-delta gates. No target label, coordinate, object dimension, joint pose,
  or special angle is introduced.

### Analytic target-centred observation generator implemented and regressed

- Added `centered_observation_pose_at_standoff()`. For each live candidate it
  keeps the candidate wrist orientation, constructs the frozen camera optical
  axis through the current geometry center, analytically intersects that line
  with the exact configured tool0 standoff sphere, and selects the largest
  positive root inside the configured camera depth interval. The solution is
  accepted only if recomputation proves the standoff, optical-axis centring,
  and camera-depth residuals within `1e-8 m`; a missing intersection fails
  explicitly as `OBSERVATION_CENTERING_UNAVAILABLE`.
- `_make_observation_sequence()` now replaces only the far-field observation
  pose with that analytic solution. Contact pregrasp/approach/grasp/lift
  construction is unchanged and remains based on the fresh near-field image.
  The observation audit records construction identity, configured and actual
  standoff, selected camera depth, centring residual, valid-root count,
  selection rule, live target, and solved tool pose. It is copied into the
  immutable streaming/plan audit.
- During the operator's manual realignment, stopped only continuous candidate
  inference with `trigger=false` so no moving-frame preview could become
  authoritative. The task was already inactive; the driver and positive joint
  enable were untouched. No arm or gripper command was issued. The operator
  subsequently reported that the target is aligned; inference remains stopped
  until the corrected planner is loaded.
- The first test invocation without sourcing the catkin environment failed at
  collection because the generated ROS message package was absent from
  `PYTHONPATH`; no test ran. After sourcing `devel/setup.bash`, the first real
  run produced `183 passed, 5 failed`. One new fixture omitted the
  `CameraIntrinsics.depth_scale` argument, and four existing partial-node
  fixtures did not define a hand-eye transform because the former observation
  builder did not consume it. These are deterministic fixture deficiencies,
  not production failures.
- Added the required synthetic depth scale and an identity hand-eye fixture to
  those partial-node tests. The streaming suite then passed `188/188`,
  including two new geometric regressions: exact centre plus exact 100-mm
  standoff, and explicit failure when the optical line cannot intersect the
  standoff sphere.
- Broader focused validation also passed: remote node `154/154`; grasp task
  and sequence `128/128` (five existing rospy deprecation warnings); pipeline,
  tabletop, and CAD gripper geometry `282/282`; Python compilation and
  `git diff --check` passed. Total focused coverage is `752` passing tests.
  The corrected source is not yet hot-loaded at this checkpoint, and no motion
  has been requested.

### Corrected planner hot-loaded; centred observation validated on hardware

- Hot-replaced only `/remote_grasp6d_node`; the arm driver, controllers,
  MoveIt, camera, task node, GUI, enable state, and held arm pose were left
  untouched. Started fresh inference after the operator confirmed alignment.
- Corrected generation 1 request 3 produced 13 hard-rechecked candidates;
  strict MoveIt found three reachable and the pipeline selected and promoted
  one. Audit SHA-256 was
  `7d02340510e2eca146f41a85ffb61a919fbf4d7770937d16c27af1a3c2c04f66`.
  This is a wider strict intersection than the previous edge-view request,
  which had only one reachable candidate.
- Froze inference and read the execution topic. Fresh authoritative plan
  `275494838bca1ccc6eef1eef`, source stamp
  `1785142904.980681896`, used the live analytic observation pose
  `xyz=(-0.132011,-0.386814,0.115995) m`, quaternion
  `(0.791673,0.609081,-0.002173,0.047628)`. Its live required opening was
  `45.929 mm`; no stored carton coordinate or joint pose was used.
- The task accepted that exact ID and executed the observation. It performed
  only the one allowed same-pose correction and transitioned normally to
  near-field replanning. Final endpoint residual was
  `3.607 mm / 0.54 deg`, with vector approximately
  `(+0.408,-0.951,+3.455) mm`, improved from the prior run's `5.643 mm` norm.
- Most importantly, live perception at the reached view measured target
  centre `uv=(340,237)`, bbox `(274,154,133,172)`, depth `145 mm`. The full
  box is centred and contained, versus the previous near-field bbox extending
  to `y=476 px` with centre near `v=404`. This is direct real-image evidence
  that the analytic observation construction solved the edge-view defect.
- The new near-field uncertainty is approximately `6.607 mm`, composed of the
  physical `3.0 mm` floor plus this run's `3.607 mm` measured endpoint error;
  it is again live-derived, not the removed fixed `6.500 mm`. The first
  near-field request returned eight post-remote candidates but no locally
  valid result. The task remains active at the centred view while later
  stability/candidate windows are supervised; no contact, close, or lift has
  occurred at this checkpoint.

### Centred-view run completed: near-field local geometry is now the blocker

- Supervised the complete 150-second near-field window after the centred
  observation. Generation 3, target epoch 4 produced requests 6 through 19.
  Every request completed and returned between 6 and 14 post-remote candidates,
  but every request had `local_candidates=0`; consequently no hard-stable,
  strict-MoveIt, preview, or promoted near-field plan existed. This run was not
  blocked by MoveIt reachability because no candidate reached MoveIt.
- Per-request local rejection counts were: request 6 `14 -> 0`
  (`CANDIDATE_CONTRACT_INVALID=7`, `GRIPPER_CONTACT_PATCH_MISS=1`,
  `GRIPPER_SWEEP_COLLISION=7`); 7 `8 -> 0` (`3/1/5`); 8 `6 -> 0`
  (`2/1/4`); 9 `8 -> 0` (`4/1/4`); 10 `11 -> 0` (`4/1/7`); 11
  `9 -> 0` (`2/1/7`); 12 `11 -> 0` (`2/1/9`); 13 `9 -> 0`
  (`3/1/6`); 14 `14 -> 0` (`4/1/10`); 15 `7 -> 0` (`3/1/4`); 16
  `10 -> 0` (`2/1/8`); 17 `13 -> 0` (`4/1/9`); 18 `10 -> 0`
  (`5/1/5`); and 19 `11 -> 0` (`2/1/9`). The slash triples are in the
  same contract/contact/sweep order.
- The one tabletop geometry branch generated in each request failed the live
  bilateral continuous-contact-overlap gate. Across this window, all 94 raw
  GraspNet candidates rejected by the analytical gripper evaluator reported
  the exact reason `invalid analytical gripper input: jaw axis must be parallel
  to support plane`. These are two distinct sources of zero candidates and
  neither is being bypassed or threshold-relaxed without a physical derivation.
- At ROS time `1785143144.431`, the task ended naturally with
  `NEAR_FIELD_REPLAN_TIMEOUT: no fresh near-field 6D preview; last=NEAR_FIELD_PLAN_PHASE_INVALID:
  Preview is not a contact execution plan`. The start service returned
  `success:false`; no second-stage pregrasp/approach, contact, gripper close,
  lift, stop, torque-off, disable, controller-stop, or `/grasp/stop` command
  occurred. The arm remains enabled and stationary at the centred observation
  pose.
- After the task released its execution slot, stopped only continuous remote
  candidate inference (`trigger=false`) to prevent unrelated far-field plans
  from replacing this diagnostic context. This did not stop the arm, driver,
  controller, GUI, camera, or joint enable. Next work is constrained to source
  and evidence analysis until a generalized contact-geometry correction is
  proven; no object-specific pose or carton-specific constant will be added.

### Frozen centred-view audit rules out a simple uncertainty subtraction

- Located the last atomically committed planning audit from the same centred
  hardware view at `~/.ros/grasp6d_gate_audit_latest.json` instead of trying to
  reconstruct missing evidence. It is generation 3 request 27, snapshot stamp
  `1785143256.1931906`, target epoch 5. This request occurred after the failed
  task had released its slot, so it is far-field and correctly records
  `execution_position_error_m=0`; no task or arm action was taken from it.
- The frozen input is fully contained: bbox `[276,152,408,324]`, 15,614 target
  points, 40,345 context support points, target fraction `0.2790`, mask/bbox
  IoU `0.9736`. The live target geometry height was `18.781 mm`; temporal depth
  repeatability was `0.148 mm`, spatial depth MAD `0.600 mm`, and the physical
  perception/CAD uncertainty floor remained `3.000 mm`.
- The generalized tabletop branch found three live jaw directions and
  materialized 24 pose variants. Nineteen passed all six local physical gates
  with required openings approximately `39.616-49.683 mm`; 17 passed the hard
  recheck, and strict MoveIt found three reachable. This proves the centred
  view has feasible strict motion candidates when contact authority is
  correctly deferred.
- However, the three proposal families' maximum measured continuous bilateral
  cloud/CAD overlaps were only `1.688`, `1.465`, and `0.660 mm`. All are below
  even the `3.000 mm` perception/CAD uncertainty floor, before adding any
  measured execution error. Therefore removing the prior observation residual
  from `6.607 mm` would still not authorize contact on this frozen image. A
  scalar uncertainty subtraction is ruled out as a solution; no threshold was
  changed.
- The same audit contains 18 raw GraspNet pose variants. Their absolute
  jaw/support-normal dot products range from `0.00163` to `0.79797`; the current
  bilateral-height helper rejects every non-parallel raw pose before CAD
  envelope evaluation. Historical log tracing shows the exact reprojection
  correction was deliberately applied only to the `tabletop_geometry` fusion
  branch, while raw GraspNet retained its separate contract. The present 94
  rejections are therefore not the prior tabletop fusion-drift regression.
- Diagnostic boundary: a single centred top/oblique RGB-D view supplies only a
  narrow common-height band on the two extreme jaw sides, while raw 6D poses
  are excluded by an input precondition rather than evaluated with a general
  two-sided contact model. The next correction must improve the actual
  orientation-aware bilateral evidence or obtain another live view; it must
  not lower the `3 mm` physical floor, assume hidden side geometry, or add a
  target-specific pose. Candidate inference remains stopped while the operator
  manually moves and realigns the arm. No robot/gripper command was issued.

### Full ROS stack relaunched for a new operator alignment

- The operator reported WSL online and requested the complete latest real-arm
  stack, direct joint enable, GUI, live terminal supervision, and no stop or
  disable commands. Read-only process resolution showed that the previous
  launch and standalone planner terminals had ended. Only an orphaned
  `motion_gateway_node.py` plus ROS base processes remained; there was no
  `/alicia_d/motion_enabled` publisher, so starting a second hardware driver
  was not a risk.
- The first direct `roslaunch` invocation failed immediately before creating
  any node because that shell had not sourced the worktree and could not find
  package `alicia_flexible_grasp_supervisor`. Re-ran after sourcing
  `devel/setup.bash`; the successful full launch is terminal session `14575`:
  `full_system.launch start_real_arm:=true driver_port:=/dev/alicia_arm
  driver_baudrate:=1000000 auto_torque_on_startup:=true
  self_check_poll_rate_hz:=0.0 start_camera:=true start_tactile:=false
  start_gui:=true use_remote_grasp6d:=true
  remote_grasp6d_url:=http://172.23.132.97:8000`.
- The new driver opened `/dev/alicia_arm` at `1,000,000` baud and explicitly
  reported that auto torque-on was requested. `/alicia_d/motion_enabled`
  published `True`; live SDK joint feedback, command keepalive, and 38-39 C
  temperature frames are present. The GUI, real D405 camera, perception,
  MoveIt, task node, controllers, safety monitor, logger, and remote 6D node
  all started. Remote WSL health reported GraspNet Baseline loaded, protocol 3.
  Electronic skin remains excluded (`start_tactile=false`, compliant/required
  tactile close both false).
- The launch parameter summary initially displayed an older static fallback
  hand-eye tuple, but fallback is disabled. The actual hand-eye publisher
  loaded the easy_handeye runtime file and logged the required unchanged
  pruned-19 transform: xyz
  `[-0.0866127223,-0.0063407198,-0.1034120675] m`, quaternion
  `[0.0229081321,-0.6787668594,0.0278470194,0.7334680031]`. No TF edit or
  replacement was needed.
- Launch reset the software calibration interlock to its default true state.
  Under the operator's already explicit renewed authorization for this same
  automatic task, set `/grasp/calibration_interlock_active=false` and reason
  `USER_OVERRIDE_PRUNED19_TF_TEMPORARY_RENEWED_20260727_AUTOMATIC_EXECUTION`.
  This parameter update did not command any motion, stop, torque-off, disable,
  or controller change.
- Candidate inference remains untriggered while the operator manually moves
  and aligns the target; startup reports only `TARGET_LOST` tombstones and no
  authoritative plan. The driver remains enabled. The next operator action is
  to align the target and report `已对准`; source diagnosis can continue while
  no plan is generated.

### Live-derived observation information and CAD-envelope correction

- The operator reported the target aligned. Candidate inference was
  deliberately left off while correcting the previous second-stage blocker;
  the enabled arm remained stationary under the operator's reached joint
  target. Live perception was stable near UV `(340,251)`, depth about
  `0.322 m`, bbox about `(310,217,63,68)`.
- Reconstructed all three strict-MoveIt-reachable observation poses from the
  frozen centred-view request using their exact fused wrist quaternions and
  pruned-19 tool-camera TF. Their camera incidence angles away from the
  negative live support normal were `4.436`, `18.115`, and `29.362 deg`.
  All three centred the complete target; their observation translation deltas
  were `28.47`, `39.61`, and `70.02 mm`. The old translation-first rank chose
  `4.436 deg`.
- Derived a target-independent information metric
  `object_height * sin(camera/support incidence)`. It is the support-normal
  object extent projected into the image plane, and it is compared with the
  same live depth/CAD uncertainty already used by the adaptive stage profile.
  The rank now first minimizes only the unresolved information deficit; after
  the projected side evidence covers the uncertainty, it again minimizes
  observation translation. It does not maximize angle without need and
  contains no target label, fixed angle, coordinate, or joint pose.
- A read-only CAD replay exposed why the highest-angle reachable pose cannot
  simply be selected: minimum fully-open finger clearance over the live
  support plane was about `70.16 mm`, `15.49 mm`, and `-20.99 mm` for the
  three poses. Thus the `29.362 deg` pose is IK-reachable but physically enters
  the measured support plane. This is not represented in the current MoveIt
  scene.
- Added `evaluate_open_gripper_observation_envelope()`. It evaluates the
  fully-open Link6/Link7/Link8 conservative CAD boxes at a non-contact
  observation endpoint against the live support plane and live target OBB,
  allowing no target contact. It reports separate support/target collision
  codes and exact minimum support clearance. The far-field stable recheck now
  applies this as a hard pre-MoveIt gate and stores the result in the immutable
  selected-plan audit.
- Added full observation-side evidence to the stable runtime/audit and changed
  the far-field MoveIt rank key to
  `(live side-evidence deficit, translation delta, negative side evidence,
  existing score, track, variant)`. Configuration enables the observation CAD
  envelope by default. Near-field contact generation and all contact gates are
  unchanged. Tests and live hot-load have not yet occurred at this checkpoint;
  no robot or gripper command was issued by these source edits.

### Observation-information correction regression checkpoint

- Added three direct regression cases for the new open-gripper observation
  envelope: a physically clear endpoint passes, a Link6/7/8 support-plane
  intrusion returns `OBSERVATION_SUPPORT_COLLISION`, and a palm/target-OBB
  intersection returns `OBSERVATION_TARGET_COLLISION`. The complete gripper
  geometry module now passes `93/93` tests.
- Added two rank regressions. A shorter view whose projected side evidence is
  still below its live uncertainty loses to a longer view whose deficit is
  zero; among zero-deficit views the shorter translation wins. A candidate
  missing the evidence fields is ranked last rather than silently receiving
  contact authority. Both targeted cases passed after sourcing this worktree
  (`2 passed, 188 deselected`).
- An initial targeted rank-test command was intentionally recorded as a
  collection-environment failure: without `devel/setup.bash`, Python could not
  import the generated ROS message package. No test body ran and this was not
  a logic failure. Repeating in the same sourced environment used by the live
  stack passed.
- `py_compile` for both modified runtime modules and `git diff --check` passed.
  The pre-existing full streaming suite before adding the two new cases passed
  `188/188`; the updated full streaming suite is being rerun before hot-load.
- Live supervision during these tests showed the enabled arm holding near
  `[-105.8, 21.2, -8.1, -0.4, -7.1, 0.1] deg`, with the target stable near
  `uv=(340,251)` and depth `0.321-0.323 m`. Hardware E1 reports remained
  isolated `1/3` events with intervening measured temperatures near `38 C`;
  no stop, torque-off, disable, gripper, or motion command was issued.
- One approximately `0.42 s` backward ROS-time event caused TF buffers to
  clear and repeated-timestamp warnings. The live joint stream, perception,
  temperature stream, and target transform subsequently continued. This is
  retained as runtime evidence; no candidate generation or arm motion will be
  started until the hot-loaded planner observes a fresh post-event snapshot.
- Added a direct numerical regression for the information rule itself. With a
  live OBB height of `11 mm` and a live uncertainty of `3 mm`, a top-down
  `0 deg` observation produces `0 mm` projected side evidence and a `3 mm`
  deficit, while a `30 deg` observation produces `5.5 mm` evidence and zero
  deficit. It contains no label or object-specific threshold. The three
  evidence/rank tests passed together, and the final updated streaming suite
  passed `191/191`.
- The companion remote-node suite passed `154/154`; `py_compile` and
  `git diff --check` remained clean. The correction is now source-regression
  complete and ready for a planner-only hot-load. The real-arm driver,
  controllers, joint enable, and full-system terminal remain untouched.

### Corrected planner hot-loaded; candidate generation remains explicit

- Resolved the existing `/remote_grasp6d_node` as PID `95663` and confirmed
  continuous inference was false. The newly added YAML key was naturally not
  present in the already loaded parameter server; the new runtime still
  defaults `observation_envelope_gate_enabled` to true by code contract.
- Killed only `/remote_grasp6d_node` and started the current worktree script as
  a standalone hot-load terminal (session `53664`). The WSL endpoint reported
  GraspNet Baseline loaded and protocol 3; the replacement ROS node is PID
  `98614`, owns the expected plan/status/audit topics and request/replan
  services, and is connected to the unchanged camera, perception, joint, TF,
  task, GUI, and hand-eye publishers/subscribers.
- The launch-owned old planner exited cleanly. No driver, controller, task
  node, GUI, camera, gripper, joint enable, or full-system process was
  restarted. `/alicia_d/motion_enabled` remained `True`, and the authorized
  `/grasp/calibration_interlock_active` remained `false`.
- The replacement initially published the expected manual-trigger waiting
  status. No candidate or movement has yet been requested at this checkpoint;
  the next action is to enable candidate streaming and require a fresh
  post-hot-load audit containing the new observation envelope and side-evidence
  fields before any execution plan is used.

### First hot-loaded stream exposed and contained a read-only geometry defect

- Started continuous candidate generation. Post-time-jump request 1 and request
  2 used fresh target snapshots near `uv=(340,251)`, produced 21 locally valid
  tabletop candidates each, and remained `STABILITY_PENDING`; neither produced
  a preview, promoted plan, task start, or arm motion.
- At the first stable recheck, every otherwise eligible variant failed with the
  exact Python error `output array is read-only`. The failure was traced to
  `_observation_side_evidence()`: it used `np.asarray()` on the intentionally
  immutable frozen support normal and then performed in-place normalization.
  This is a deterministic implementation defect, not evidence for changing a
  geometry gate, threshold, object pose, or fixed observation value.
- Immediately set only `/grasp_6d/request_plan trigger=false` to stop further
  remote inference while correcting the planner. This did not stop the arm,
  driver, controller, GUI, perception, joint enable, or command keepalive.
- Changed the local support-normal construction to `np.array(..., copy=True)`
  before normalization. Extended the numerical side-evidence regression to
  mark the source normal read-only, exactly matching production frozen
  geometry. No planning threshold or physical rule changed.
- The first mechanical edit matched the earlier
  `_geometry_object_height_m()` support copy instead of the runtime-failing
  `_observation_side_evidence()` line. The new read-only regression therefore
  failed exactly as desired (`output array is read-only`) and prevented a bad
  hot-load. Applied the copy at the exact named function; the focused immutable
  input regression then passed, followed by the full streaming suite
  `191/191`, `py_compile`, and `git diff --check`.

### First corrected promotion exposed a second-stage ranking handoff defect

- Hot-loaded the precise immutable-array fix and restarted inference. Requests
  1 and 2 accumulated stability normally. Request 3 produced 21 locally valid
  candidates, 16 stable candidates, 12 hard-safe candidates, and four strict
  MoveIt-reachable candidates. The new observation endpoint envelope was live:
  it rejected multiple variants with measured Link7 support intrusion between
  approximately `-15.8` and `-25.6 mm`. No read-only-array failure recurred.
- Request 3 promoted plan `2b94a939ef652f6746063ab5`, bound to snapshot
  `1785145020.6627584`. Before execution, stopped candidate inference and read
  both preview and immutable execution audits. The chosen observation was
  strict-MoveIt reachable, but its projected side evidence was only
  `1.360 mm` against `3.000 mm` live uncertainty: deficit `1.640 mm`, incidence
  `3.835 deg`.
- The same frozen request had another strict-MoveIt-reachable candidate at
  incidence `18.657 deg`, projected side evidence `6.505 mm`, zero deficit, and
  observation translation `161.566 mm`. It was not chosen. Source inspection
  found the exact handoff error: the new evidence rank correctly ordered the
  MoveIt shortlist, but `bounded_moveit_select()` then selected among reachable
  results using the old generic final soft score, discarding the observation
  information ordering.
- The promoted plan was therefore not started. Added a far-field-only final
  choice that reapplies the identical live-information rank to the set that
  actually passed strict MoveIt; near-field contact selection is unchanged.
  Also added the passed observation envelope and side-evidence dictionary
  directly to every stable evaluation, especially the atomically committed
  `selected` execution row. No angle, object coordinate, target label, or
  fixed grasp pose was introduced.
- The targeted final-rank and atomic-audit regressions passed. The first full
  suite run then found four mock-only incompatibilities: old tests substituted
  strings or unscored `StableCandidate` objects for the production
  `ScoredStableCandidate` result guaranteed by `bounded_moveit_select()`.
  Restricted the second rank application to reachable results that carry both
  track and variant audit identity; production candidates satisfy this
  contract, while synthetic remote-failure tests retain their injected
  selection. The full streaming suite returned to `191/191`, the remote-node
  suite remained `154/154`, and `py_compile`/`git diff --check` passed.

### Final-rank live plan satisfies the generalized observation contract

- Hot-loaded the final ranking/audit changes, restarted inference, and allowed
  fresh requests to rebuild stability. Request 3, generation 1, target epoch 1,
  snapshot `1785145552.857475` produced 21 locally valid, 18 stable, 13
  hard-safe, and four strict-MoveIt-reachable candidates. The far-field final
  selection now follows the same live-information order after MoveIt.
- Froze inference after promotion. The atomically committed execution audit
  SHA-256 is
  `23088a39b0b692fd79493272ba9f7d0b06718fa320896bb1bc1dd4e8efe22d13`;
  it binds plan `8a1a7f0205a48f49e60e1ccf` to the same request, generation,
  epoch, and exact snapshot timestamp as the published enriched plan.
- Selected track 11 variant 0 is `tabletop_geometry`, strict-MoveIt reachable
  (`STRICT_SERVICE_SUCCESS`), target observation xyz approximately
  `(-0.1454,-0.3768,0.0609) m`, path cost `1.684`, maximum joint delta
  `0.956 rad`. Its open-gripper observation envelope is `ok=true` with
  `11.941 mm` minimum measured support clearance.
- Its generalized side-information audit is now explicit in the immutable
  selected row: live object height `20.338 mm`, camera/support incidence
  `18.566 deg`, projected side evidence `6.476 mm`, live uncertainty
  `3.000 mm`, signal/uncertainty `2.159`, and deficit exactly `0`. The decision
  rule remains `object_height * sin(incidence) >= live uncertainty`; these
  values were measured from this snapshot rather than fixed for the carton.
- Request 4 generated a non-promoted preview, and request 5 failed on a stale
  TF extrapolation caused by the recurring ROS-time jump. Neither replaced the
  request-3 execution authority. Candidate inference is frozen. No motion has
  occurred yet; this is the first plan in this session eligible to start the
  far-field observation phase.

### Fresh authority regenerated and first observation execution result

- The first attempt to invoke `/grasp/start` used positional-looking YAML whose
  `execute` field was not decoded as a boolean. The ROS client rejected that
  request before the service handler ran; it issued no motion. The valid
  service contract is one YAML mapping:
  `rosservice call /grasp/start "{execute: true, plan_id: '<id>'}"`.
- The corrected call for plan `8a1a7f0205a48f49e60e1ccf` reached the task
  node but was rejected `PLAN_STALE`. Read-back confirmed the unchanged live
  validity contracts `/grasp_6d/plan_validity_sec=120.0` and
  `/grasp_6d/target_observation_validity_sec=8.0`. These gates were not relaxed
  or bypassed, and the stale plan issued no motion.
- Regenerated from fresh live evidence. Request 10, generation 3, target epoch
  3 promoted plan `16a5721c456f9fbab880a692`, bound to snapshot
  `1785145929.1065874`. The selected observation passed strict MoveIt
  (`STRICT_SERVICE_SUCCESS`) and the fully open gripper envelope with
  `15.221 mm` minimum support clearance. Its generalized side evidence was
  `6.467 mm` against `3.000 mm` live uncertainty, incidence `18.218 deg`, and
  deficit exactly zero. Candidate inference was frozen before execution.
- Invoked `/grasp/start` with the valid YAML mapping and exact fresh plan ID.
  Strict preflight and atomic execution planning passed. The observation
  trajectory was retimed to `19.179 s`, with maximum joint delta `0.959 rad`
  and configured maximum velocity `0.080 rad/s`, then completed successfully.
  The target image evolved continuously from about `uv=(340,251)`, bbox
  `63 x 69`, depth `0.322 m`, to about `uv=(391,179)`, bbox `135 x 186`, depth
  `0.126 m`. This is direct evidence that the information-derived observation
  moved to a substantially closer oblique view rather than the former nearly
  top-down view.
- The commanded observation endpoint was approximately
  `xyz=(-0.143,-0.377,0.063) m`, quaternion
  `(0.702,0.675,-0.137,0.180)`. After the initial trajectory settled, live
  tool0 FK measured `11.5 mm / 1.70 deg` residual against that same endpoint,
  exceeding the unchanged `6.0 mm / 5.0 deg` measured endpoint contract.
- The existing bounded correction requested the same live-derived Cartesian
  endpoint once. Its `3.0 s` trajectory had maximum joint delta about
  `0.025 rad` and completed, but measured residual did not converge: the final
  held result was `13.9 mm / 2.05 deg`. The task therefore failed closed with
  `OBSERVATION_ENDPOINT_NOT_CONVERGED` and explicitly refused near-field
  planning.
- No near-field candidate generation, planned approach, gripper close, or lift
  occurred. The gripper remained open. At this checkpoint the ROS/task data
  alone did not report contact. The arm remained enabled and stationary near
  the observation pose at approximately
  `[-116.6,-34.2,45.8,-22.8,-38.6,40.1] deg`; the corresponding held command
  was approximately
  `[-117.2,-32.8,46.4,-22.9,-38.6,40.4] deg`. This joint-space difference is
  retained as evidence for diagnosing the Cartesian residual; no stop,
  torque-off, disable, return motion, or new task command was issued.
- Source inspection after the failure shows that the present “bounded
  correction” strictly resends the same desired pose. It does not transform
  the measured live Cartesian residual into a compensating setpoint. Therefore
  it is suitable for transient settling error, but the observed repeatable
  command/feedback offset can make it ineffective. This is a diagnosis
  checkpoint only: no tolerance has been widened and no compensation change
  has yet been made. The next step is to inspect the controller success and
  tracking criteria and implement only an evidence-derived, bounded,
  target-independent correction if those data support it.

### Operator photograph invalidates the first-stage physical-clearance result

- The operator supplied an external photograph taken after the arm reached the
  first observation pose. It is higher-authority physical evidence than the
  ROS task-state inference above: both open gripper fingertips visibly reach
  the tabletop, while the target box is displaced far from the gripper centre.
  Correct the earlier runtime statement accordingly: the task did not enter
  its *planned* near-field/contact phase, but the far-field observation motion
  itself produced unintended physical tabletop contact.
- The photographed displacement is far larger than the measured
  `13.9 mm / 2.05 deg` endpoint tracking residual. Therefore endpoint
  convergence cannot be treated as the primary cause and no Cartesian
  correction, tolerance relaxation, or repeated observation attempt is
  authorized from this pose.
- The immutable audit claimed `15.221 mm` minimum open-gripper support
  clearance, but the photograph shows physical contact. This falsifies at
  least one input/model assumption in the observation envelope chain:
  support-plane transform/sign/offset, Link6/7/8 CAD extent/transform, or the
  camera-to-tool/target geometry used to place the observation. Passing the
  current gate is not evidence of real clearance until those alternatives are
  discriminated from recorded transforms and geometry.
- The target remaining visible to the wrist camera at short reported depth
  does not validate gripper-to-target alignment: an eye-in-hand camera can
  centre the target while an erroneous camera/tool transform or incomplete
  gripper envelope places the fingers elsewhere. The external photograph
  specifically requires auditing camera optical geometry and the complete
  fingertip CAD endpoints against the same frozen support plane.
- All new automatic grasp motion is paused. No stop, torque-off, disable, or
  return command was published; the operator was asked to move the enabled arm
  manually away from the physical contact region. Further work is restricted
  to offline evidence diagnosis until the failed physical-clearance model is
  corrected and regression-tested.

### Hardware powered off; evidence-only geometry audit resumed

- The operator reported all hardware serial ports closed and arm power off,
  while the WSL inference side is available. Work is now strictly offline: no
  ROS real-arm launch, serial access, enable, motion, gripper, stop, torque-off,
  or disable command is authorized. Any conclusion requiring new real images
  or physical measurement will remain deferred for operator wake-up.
- The atomic gate audit still exists at
  `/home/zhuyupei/.ros/grasp6d_gate_audit_latest.json` (591171 bytes). It has
  since been atomically replaced by non-executed request 11, so it is not the
  exact request-10 execution authority. The exact execution evidence retained
  in ROS logs is plan `16a5721c456f9fbab880a692`, request 10, generation 3,
  target epoch 3, snapshot `1785145929.1065874`, with the execution-time
  observation pose and claimed `15.221 mm` clearance already recorded above.
  No value from request 11 will be silently substituted for request 10.
- An initial source comparison exposed a real model-contract difference: the
  analytical gate declares `Alicia_D_v5_6_gripper_50mm`, while the active
  MoveIt description is loaded from
  `alicia_d_descriptions/urdf/alicia_duo_with_gripper.urdf`, named
  `alicia_D_v5_5`, with 100 mm-class finger travel. Mesh hashes also differ:
  active v5.5 Link7/8 are
  `ff006151...` / `fd49b1a4...`, whereas the analytical v5.6 50 mm Link7/8 are
  `d546b7ab...` / `5e5b5487...`.
- Do **not** infer that this version/aperture mismatch caused the photographed
  tabletop contact. Direct binary-STL/URDF transformation into each model's
  tool0 frame disproved that simple explanation: the active v5.5/100 mm
  fingers reach `-60.000 mm` along tool0 Z, while the assumed v5.6/50 mm
  fingers reach `-60.200 mm`, only `0.200 mm` different. Their material
  extents are both approximately `43.3 x 28.6 x 60.0 mm`; the large difference
  is lateral open-jaw placement, not fingertip length. This cannot explain a
  claimed positive `15.221 mm` vertical clearance becoming physical contact.
- The model mismatch remains a separate contract defect that can affect
  lateral target/envelope tests, but it is not promoted to root cause without
  a numerical reproduction. Per the operator's instruction, no source change
  has been made from this finding. Next analysis is restricted to replaying
  the exact support-plane and tool0 transform algebra and auditing whether the
  observation envelope used the intended pose/frame at promotion.

### Exact request-10 execution audit recovered; endpoint error direction matters

- Correct the audit-retention statement above: the remote node intentionally
  keeps Preview and Execution reports at different paths. The file search had
  matched only names ending in `.json` and therefore missed
  `/home/zhuyupei/.ros/grasp6d_gate_audit_latest.json.execution`. Source at
  `_execution_audit_output_path()` confirms this separation. The recovered
  Execution report is exactly request 10, generation 3, target epoch 3, plan
  `16a5721c456f9fbab880a692`, and snapshot
  `1785145929.1065874`; it contains 61 rows and the selected source 13,
  variant 0 lineage. The unexecuted request-11 Preview report has not replaced
  this authority.
- The execution report independently reproduces the selected observation
  facts: strict MoveIt reason
  `planned: target xyz=(-0.143, -0.377, 0.063)`, open-gripper envelope
  `ok=true`, minimum nominal support clearance `15.221071 mm`, incidence
  `18.218107 deg`, projected side evidence `6.467323 mm`, and live depth
  uncertainty `3.000000 mm`. These are exact request-10 fields rather than a
  request-11 proxy.
- Transforming the report's exact support-plane normal and point with its
  frozen `T_base_camera_link` gives unit base normal
  `[-0.065125854, 0.058471248, 0.996162505]`, one base-plane point
  `[-0.001660729, 0.001491041, 0.025402471] m`, and plane offset
  `-0.025500329 m` for `n dot x + offset = 0`.
- Correct the earlier scalar inference that the photographed contact could not
  be caused primarily by the `13.9 mm` endpoint residual. The log-retained
  final command/feedback joints, evaluated offline with the active v5.5 URDF,
  yield approximate tool0 positions
  `[-0.143703293,-0.377193533,0.063159400] m` (command) and
  `[-0.137271074,-0.373322895,0.050681038] m` (feedback). Their
  actual-minus-command translation is
  `[+6.432,+3.871,-12.478] mm`; its projection on the exact request-10 support
  normal is `-12.623 mm`, toward the tabletop. Applied to the audited nominal
  minimum, the translation-only residual leaves approximately `2.598 mm`,
  already below the configured `3.000 mm` support-clearance contract.
- The FK figure is approximate because the driver log rounds each joint to
  `0.1 deg`; the task node's higher-authority norms remain `11.5 mm` initially
  and `13.9 mm` after correction. The directional reconstruction is supported
  by the close norm and by reproducing the logged requested Cartesian pose,
  but it is not yet a full physical-clearance result: fingertip mesh vertices
  and the measured orientation residual must be transformed at both poses.
- No controller, threshold, transform, or grasp code is changed at this
  checkpoint. The evidence now identifies a concrete missing contract to test:
  the observation envelope certifies only the nominal requested endpoint,
  while measured tracking error is checked only after the arm has already
  entered that endpoint. Whether this is sufficient to reproduce actual
  fingertip contact will be decided by the complete offline mesh replay, not
  by assumption.

### Full active-mesh replay disproves a tracking-only contact explanation

- Added the ROS-independent diagnostic
  `tools/replay_observation_clearance.py`. It reads a caller-selected frozen
  audit, URDF, binary collision STL set, and caller-supplied joint coordinates;
  it performs URDF FK and reports every collision-mesh vertex's signed distance
  to that audit's support plane. It has no ROS/device import, opens no serial
  port, and contains no request-10 joint values, object label, target
  coordinate, or production motion behavior. `py_compile` passed.
- Replayed the exact request-10 Execution audit with the active
  `alicia_duo_with_gripper.urdf` and its `Link6`, `Grasp_base`, `Link7`, and
  `Link8` collision meshes. Input hashes are retained by the tool: audit file
  `6173a940...`, URDF `04723e2f...`, Link7 `ff006151...`, and Link8
  `fd49b1a4...`. The rounded final command
  `[-117.2,-32.8,46.3,-22.9,-38.6,40.4] deg` reproduces tool0
  `[-0.143476,-0.376751,0.062704] m` and quaternion
  `[0.702612,0.674921,-0.137220,0.178853]`, consistent with the logged
  Cartesian target.
- At that nominal command, the active physical mesh's minimum support-plane
  distance is `22.138 mm` at Link7. At the rounded final feedback
  `[-116.6,-34.2,45.8,-22.8,-38.6,40.1] deg`, the minimum is
  `9.330 mm` at Link8 when evaluated at the URDF's `right_finger=0` endpoint.
  Repeating the same calculation at the opposite valid prismatic endpoint,
  `right_finger=0.05 m`, gives `10.072 mm`. Thus the uncertain gripper joint
  convention changes this support result by less than `0.75 mm` and does not
  make the replay contact the plane.
- This is a required correction to the preceding translation-only estimate:
  the measured endpoint error consumes approximately `12.8 mm` of active-mesh
  clearance after its orientation change is included, but the exact stored
  support plane plus active CAD still predicts roughly `9-10 mm` positive
  clearance. The endpoint residual is therefore a real safety-budget defect,
  but it is not by itself a complete numerical reproduction of the operator's
  photographed physical contact.
- The remaining discrepancy is not assigned by guess. At least one of the
  stored support plane in base, physical-versus-CAD fingertip extent, or
  reported-joint/URDF FK remains inaccurate by approximately the residual
  amount. Correction after reading `publish_joint_state()`: the driver does
  **not** publish its internal `0..100 deg` logical value directly. It converts
  that value back to `0..0.05 m` before publishing `right_finger`, consistent
  with the prismatic joint's units. The earlier out-of-range/radian statement
  was false and is superseded here.
- A different gripper-state contract mismatch is proven. The real driver and
  task define the physically open feedback as about `0.05 m` (`raw=995` gives
  `0.04975 m`). Direct transformation of the active URDF STL pair shows
  `right_finger=0` gives a `99.9996 mm` inner CAD gap, while
  `right_finger=0.05 m` gives approximately `0 mm` inner CAD gap. Therefore
  robot_state_publisher/MoveIt represent the physically open hand as
  geometrically closed. This does not explain the photographed vertical
  contact—the two endpoint replays differ in support clearance by less than
  `0.75 mm`—but it can invalidate target and lateral collision checking and
  remains a separate model-state defect.

### Existing independent calibration evidence cannot certify the residual gap

- The deployed pruned-19 transform is preserved unchanged. Its independent
  six-pose ChArUco report already exists at
  `handeye_invariance_20260725_225442.yaml` and formally records `passed:false`:
  translation RMS `3.942 mm`, translation max `6.671 mm`, orientation RMS
  `1.071 deg`, and orientation max `1.674 deg`. Each local capture window was
  stable (`0.02-0.21 mm` RMS), so this is measured pose-dependent systematic
  variation, not an inference from the collision photograph.
- Those measured maxima are of the same order as the replay's remaining
  `9.330 mm` clearance. They do not prove which signed error occurred at the
  request-10 pose and therefore are not subtracted as though their directions
  were known. They do prove that the failed calibration has no validated bound
  capable of certifying the replayed positive gap. The prior temporary
  calibration-interlock override remains an operator override, not a passed
  calibration result.
- Consequently no TF correction, fixed table-height shift, fingertip-length
  edit, or execution-margin constant is introduced from this evidence. A
  physical support-plane/CAD check or a passing independent hand-eye
  verification is required to discriminate the remaining alternatives. That
  work requires powered hardware/current images and is deferred as instructed.
  Offline work may still correct defects whose cause is already closed by the
  existing trace.

### Sub-goal-band endpoint-trim activation defect corrected offline

- The final request-10 SDK command/feedback pair has a maximum rounded joint
  difference of about `1.4 deg` (`0.0244 rad`). This is below the driver's
  `0.035 rad` initial trim-activation condition, and there is no endpoint-trim
  activation entry in the exact runtime interval. Nevertheless the task's
  measured Cartesian residual at the same endpoint was `13.9 mm`. This
  directly falsifies the assumption that being inside the broad joint
  controller goal band implies the `6 mm` Cartesian endpoint contract.
- Changed only the *initial* stable-reference trim condition from
  `joint error > 0.035 rad` to
  `joint error > 2*(2*pi/4096) = 0.003068 rad`, the already-derived SDK command
  plus feedback round-trip resolution floor. The response-gated accumulated
  live correction, fresh-feedback requirement, `0.30 s` reference/feedback
  dwell, `0.12 rad` per-joint anti-windup bound, and no-joint-specific algebra
  are unchanged.
- Existing hysteresis is preserved: after error reaches the round-trip floor,
  the last live trim latches quiescent; it can leave that latch only if error
  later exceeds the broader `0.035 rad` controller goal band. Thus a newly
  settled pose no longer ignores physically resolvable error merely because
  the joint trajectory controller accepted it, while a converged held pose
  does not resume encoder-level oscillation.
- Updated the source-contract regression to require round-trip-floor initial
  activation while retaining goal-band latch exit. Focused configuration and
  implementation tests passed `11/11`; `py_compile` for the offline replay
  tool passed; `git diff --check` passed. `catkin_make --pkg alicia_d_driver
  -j2` rebuilt and linked successfully. The offline-built binary is `4668608`
  bytes with SHA-256
  `704586c0ddbffa24a6a9f8a6b439de0d4f6532ebc895a53cb1b0c4633fd23864`.
- The binary has not been hot-loaded and no ROS node, device, serial port,
  controller, arm, gripper, enable, stop, torque-off, or disable path was
  touched. Hardware validation of convergence remains deferred until the
  operator powers and explicitly resumes the real system. This control fix
  does not by itself certify observation-stage collision safety; the failed
  hand-eye/physical-clearance contract above remains blocking evidence.

### Active MoveIt gripper coordinate reversed without changing its CAD poses

- Continued strictly offline after the operator removed hardware power and
  closed all hardware serial ports. No ROS graph, serial device, controller,
  torque, enable, stop, arm, or gripper command was accessed.
- The end-to-end source trace closes the direction mismatch without relying on
  an object-specific assumption. Active SDK-v6 feedback maps `raw=0..1000` to
  the driver's public `right_finger=0..0.05 m`; the retained real trace and
  photograph establish that about `raw=995` / `0.04975 m` is the physically
  open posture. The grasp configuration and task also use
  `open_position_m=0.05` and `close_limit_m=0`. The active MoveIt URDF and SRDF
  had the opposite interpretation: direct STL transformation gave a CAD inner
  gap of `99.9996 mm` at `right_finger=0` and approximately `0 mm` at
  `right_finger=0.05`, while the named SRDF states declared `open=0` and
  `close=0.05`.
- Reparameterized only the two active URDF prismatic joint transforms so the
  same CAD pose set now follows the measured public coordinate. For the right
  finger the joint origin Y changes from `-0.050498` to `-0.000498 m` and its
  local axis from `-Z` to `+Z`; for the mimicked left finger the origin Y
  changes from `+0.050502` to `+0.000502 m` and its local axis from `+Z` to
  `-Z`. Because the source RPY is the rounded `1.5708` rather than exact
  `pi/2`, the complete rotated-axis substitution also changes both origin Z
  values from `0.13118` to `0.131180183660255 m`. A stricter four-sample
  transform regression exposed this `0.183660255 um` term; it was retained
  instead of being discarded as negligible. Joint limits, meshes, mesh
  scales, arm links, `tool0`, and the left/right mimic relation are unchanged.
  Algebraically this is the exact coordinate substitution
  `q_old=0.05-q_public`, not a new aperture or a target-specific offset.
- Updated only the active semantic file's named hand states to
  `open=0.05`, `close=0`. The driver and task command/feedback conversion is
  intentionally unchanged, so service calls and real SDK values keep their
  already-measured semantics. The MuJoCo protocol-v3 server is also unchanged:
  it receives six arm joints only and independently converts a requested
  *inner gap* into its v5.6 50 mm model's left/right coordinates.
- Added an STL-based regression for the active URDF. Before the model change it
  failed with `closed_gap_m=0.099999563 m`, proving the original reversal. It
  now proves `right_finger=0` yields the centered/closed CAD pose,
  `right_finger=0.05` yields the original `0.100 m`-class open CAD pose, the
  gap grows with the public value, and active SRDF names agree. The complete
  gripper-geometry suite passes `94/94`; `git diff --check` passes. Current
  hashes are URDF `890592a2...`, SRDF `ac14d9a4...`, and regression source
  `0e284070...`. The related trajectory/controller-source suite passes
  `11/11`, and the gripper full-joint hold suites pass `3/3`.
- Replayed request 10 again at the physically open public value
  `right_finger=0.05`. Because the reparameterization preserves the original
  open CAD pose exactly, the nominal command still has `22.138 mm` minimum
  support clearance and the rounded final feedback still has `9.330 mm`.
  Therefore this fix repairs MoveIt/robot-state lateral finger geometry but
  does **not** numerically explain or clear the photographed tabletop contact.
- `0.100 m` above is only the active CAD mesh gap; it is not asserted as a new
  measurement of the real gripper. The production analytical/MuJoCo contract
  remains the conservative 50 mm model, and the unresolved physical-versus-CAD
  extent/support-plane/hand-eye discrepancy still requires powered hardware or
  a current calibrated image. No new 6D execution is authorized by this
  offline correction alone.

### 2026-07-27 latest full ROS stack started for renewed aligned-target trial

- At the operator's explicit request, launched the current protocol-v3
  worktree directly with the retained full-system command:
  `roslaunch alicia_flexible_grasp_supervisor full_system.launch
  start_real_arm:=true driver_port:=/dev/alicia_arm
  driver_baudrate:=1000000 auto_torque_on_startup:=true
  self_check_poll_rate_hz:=0.0 start_camera:=true start_tactile:=false
  start_gui:=true use_remote_grasp6d:=true
  remote_grasp6d_url:=http://172.23.132.97:8000`.
- The supervised roslaunch terminal is session `75903`; ROS run ID is
  `fe4fa65e-8a25-11f1-9772-9b3fc31051a2`. The driver PID is `6364` and the GUI
  PID is `6413`.
- The real driver opened `/dev/alicia_arm` at `1000000` baud and entered the
  configured automatic torque-on path. Continuous SDK feedback initially
  reported joints near zero, open gripper `raw=995`, ordinary status `0x00`,
  and measured maximum temperature `31-32 C`. Both `alicia_controller` and
  `hand_controller` loaded and started.
- The real camera, MoveIt, GUI, motion gateway, task, perception, remote 6D
  node, safety monitor, and logger started. MoveIt printed
  `You can start planning now!`; the remote service reported GraspNet Baseline
  loaded with protocol 3. Electronic skin remains excluded and no electronic
  force feedback is used.
- The deployed pruned-19 hand-eye transform is unchanged: xyz
  `[-0.0866127223,-0.0063407198,-0.1034120675] m`, quaternion
  `[0.0229081321,-0.6787668594,0.0278470194,0.7334680031]`.
- After startup, `/joint_commands` began changing multiple arm joints while
  the operator was in the manual-alignment phase; the driver preserved the
  open gripper and streamed `gripper_raw=1000`. The new sub-goal-band endpoint
  trim was observed activating on a settled reference: first iteration at
  maximum joint error `0.015748 rad`, then a second at `0.009476 rad`. This is
  direct runtime evidence that the newly built activation path is loaded; no
  convergence claim is made while the manual target continues changing.
- The terminal emitted a `0.46 s` backward ROS-time jump followed by repeated
  TF timestamp warnings. This event is recorded without assigning a cause; it
  occurred during changing joint commands and before target alignment was
  reported. Perception has alternated between `TARGET_LOST` and one low-
  confidence distant detection. No 6D candidate generation, `/grasp/start`,
  task execution, stop, torque-off, disable, controller-stop, or arm-stop was
  requested by the assistant. Live supervision continues while waiting for
  the operator to report `已对准`.
- During the same manual-alignment interval, real SDK feedback twice became
  stale for more than the configured `1.0 s`. The driver consequently paused
  its SDK command stream and suppressed synthetic heartbeat publication, as
  designed; one all-zero six-encoder frame was rejected. The first interval
  recovered without an assistant command and real feedback resumed. On a
  later stable reference, the endpoint trim reached quantization-floor
  quiescence after four iterations (`max_error=0.002621 rad` versus the
  `0.003068 rad` round-trip floor). A second stale-feedback interval was still
  present in the latest terminal sample. These are observed facts only; no
  serial/power/clock cause is assigned and no stop/disable command was sent.

### Aligned target, renewed calibration override, and fresh generation-1 audit

- The operator reported that the target was aligned and then explicitly
  authorized both clearing the calibration interlock and automatic execution
  of the 6D grasp. The retained independent calibration result is still a
  failure; it is not relabelled as a successful calibration. The runtime
  override is therefore recorded explicitly as a temporary operator override:
  `/grasp/calibration_interlock_active=false`, with reason
  `USER_OVERRIDE_PRUNED19_TF_TEMPORARY_RENEWED_20260727_AUTOMATIC_EXECUTION_AFTER_GRIPPER_MODEL_FIX`.
  The deployed pruned-19 hand-eye transform itself was not edited.
- Immediately before requesting a new plan, the live aligned target was
  stable around image coordinate `(311,289)`, depth `0.327-0.329 m`,
  confidence `0.918-0.920`, and base position approximately
  `(-0.114,-0.440,0.062) m`. Real feedback was stable around
  `[-105.0,29.4,-8.6,-0.1,-13.5,0.4] deg`, open-gripper feedback was
  `raw=995`, and endpoint trim was quiescent at `0.002233 rad`, below its
  measured `0.003068 rad` round-trip quantization floor.
- Runtime `robot_description` was read back and confirms the corrected public
  gripper coordinate is active: the left/right joint origins are approximately
  `[...,+0.000502,+0.131180184]` and
  `[...,-0.000498,+0.131180184]`, with opposite corrected axes and the
  retained `-1` mimic. The loaded model is `alicia_D_v5_5`; this is not an
  assumption based only on the source tree.
- Called `/grasp_6d/request_plan` with `trigger=true`. This began a wholly
  fresh protocol-v3 stream at `generation=1`, `target_epoch=4`; no old plan was
  reused. The first two remote responses each produced 1024 raw proposals,
  60 NMS proposals, 21 remote-collision survivors, and a live tabletop
  materialization/recheck set, but neither had a temporally stable promotable
  candidate (`stable 0/21`, `STABILITY_PENDING`). The corresponding audit was
  written to `/home/zhuyupei/.ros/grasp6d_gate_audit_latest.json`.
- Subsequent same-generation rechecks exposed a fail-closed geometric reason,
  not the calibration interlock: multiple candidates were rejected with
  `OBSERVATION_SUPPORT_COLLISION` because the corrected open left-finger
  geometry entered the support clearance. Observed predicted clearances
  include approximately `-12.297`, `-13.320`, `-14.350`, `-15.469`,
  `-17.609`, and `-52.803 mm`. Other candidates reached the configured
  `3.000 mm` minimum finger clearance but were independently rejected by
  `GRIPPER_SWEEP_COLLISION: palm enters support clearance`; several insertion
  tilts had no stage profile inside the hard bounds. A few strict observation
  pose checks were reachable while others were not, so reachability alone
  does not clear the complete observation envelope.
- No preview or exact executable plan ID has been promoted yet. Consequently
  no `/grasp/start`, arm trajectory, gripper-close, stop, torque-off, disable,
  controller-stop, or arm-stop command has been sent. The support-collision
  gate is intentionally not relaxed: the earlier photographed tabletop
  contact and the now-corrected active finger geometry make forced execution
  contrary to the available evidence. Supervision continues for a genuinely
  live-derived candidate that passes every gate.
- While the candidate stream was evaluating, the driver continued ordinary
  `0x00` feedback and quantization-floor holding. Isolated `0xE1` status
  events and isolated anomalous temperature samples (including a single
  `161 C` value amid ordinary roughly `31-43 C` samples) were observed but did
  not repeat for the driver's required consecutive count. The driver logged
  them as status events and explicitly sent no torque-off. Another roughly
  `0.52 s` backward ROS-time jump and repeated TF-timestamp warnings occurred;
  no cause is inferred from these events.

### Fresh plan promotion, attempted execution, and protection-latched failure

- The continuous generation-1 stream eventually produced live-derived
  `PREVIEW_READY` candidates. Request 10 reported one preview from four
  reachable shortlisted candidates, while its normal promotion policy still
  rejected that preview (`promoted 0/1`). The operator had already explicitly
  requested direct execution, so `/grasp_6d/replan_execution trigger=true`
  was called. It returned success with
  `cached Preview promoted to execution authority`; no collision or
  calibration gate was bypassed by that operation.
- The continuous stream kept updating after that promotion. The execution
  topic was therefore read back rather than reusing the earlier preview ID.
  Its latest bound plan was valid, model choice `carton_segment`, source
  `tabletop_geometry`, diagnostic `FAR_FIELD_OBSERVATION_PLAN`, and
  `plan_id=0f59954a110ddc8228e86ee8`. The live object estimate at that plan
  stamp was approximately size
  `(57.59,36.58,25.70) mm`, required open width `41.99 mm`, support normal
  `(-0.0826,0.0285,0.9962)`, and object position
  `(-0.1144,-0.4442,0.0574) m`. Its four pose translations were approximately
  observation `(-0.1121,-0.3581,0.0899)`, pregrasp
  `(-0.1166,-0.4437,0.0811)`, grasp
  `(-0.1144,-0.4513,0.0535)`, and lift
  `(-0.1144,-0.4513,0.0865) m`. These values were not written as a fixed
  object-specific program.
- Called `/grasp/start` with `execute=true` and that exact plan ID. Although
  the synchronous service ultimately returned `success: false, message:
  failed`, the task had accepted the plan and started its asynchronous
  physical sequence. The task changed the remote planner to the near-field
  phase, preflight strict planning for the observation pose succeeded,
  open-gripper motion was correctly skipped because measured opening was
  already within `1 mm` of `0.050 m`, and strict trajectory execution began.
  The planned first-stage duration was `20.862 s`, maximum joint delta
  `1.043 rad`, velocity cap `0.080 rad/s`, and target translation
  `(-0.112,-0.358,0.090) m`.
- During that first-stage motion the controller reported
  `PATH_TOLERANCE_VIOLATED: Joint6 path error -0.773124`, and MoveIt returned
  `ABORTED: CONTROL_FAILED`. The task entered `FAILED` with
  `6D pregrasp failed`; neither the near-field replan nor grasp/close/lift
  stage executed. Immediately before that controller abort, a real-feedback
  sample showed Joint6 jumping to `54.1 deg` while its neighboring command was
  about `10 deg`. This is recorded as observed data, not yet assigned a serial,
  encoder, controller, or mechanical cause.
- A second control defect was directly observed after the action abort: the
  driver's received `/joint_commands` continued advancing along the remaining
  precomputed trajectory instead of holding the failure point. The real arm
  therefore continued moving after the task had already declared failure.
  Commands advanced through roughly
  `[-112.3,-28.4,50.4,-12.2,-55.1,27.5] deg`; feedback subsequently reached
  approximately `[-111.7,-29.6,47.4,-0.3,-54.0,26.0] deg`. No second grasp
  was launched.
- The driver then received consecutive anomalous temperature evidence:
  a single `163 C` sample followed by two repeated samples containing
  `100 C`. On the driver's third consecutive over-limit count it logged
  `Protection status latched; sending torque_off and blocking all motion
  commands`. This torque-off was generated autonomously by the already-running
  driver protection path; the assistant did **not** publish or call a stop,
  torque-off, disable, controller-stop, or arm-stop command.
- After the autonomous protection action, real feedback briefly reported
  approximately `[-111.7,-58.5,86.0,-11.9,-92.0,27.0] deg` with the ordinary
  temperature maximum back near `36 C`. Feedback then became stale and the
  driver repeatedly reported both `Blocking SDK command stream: motion has not
  been explicitly enabled` and suppression of stale `/joint_states`
  heartbeats. Target depth had fallen from roughly `0.326 m` to `0.067 m`
  during the post-abort motion before target loss was rejected. No claim is
  made from that depth alone about physical contact.
- The protection latch must not be bypassed from software on the present
  evidence. Execution is paused before near-field/grasp stages; automatic
  re-enable and any new 6D start are blocked pending operator inspection of
  the real arm, gripper, target, and table and a report of whether physical
  contact or obstruction occurred. Live terminal supervision remains active.
- The operator's immediate physical observation is that the arm lost power
  directly while still approaching the first-stage observation pose. This
  corroborates the runtime ordering: it was not a successful arrival at the
  first stage, and no second-stage approach occurred. It is consistent with
  the driver's recorded autonomous protection-latched torque-off; it does not
  by itself distinguish a genuine thermal/mechanical condition from corrupted
  SDK status/temperature/encoder data. No software re-enable was attempted.

### Same-channel thermal confirmation replaces cross-channel global streak

- The operator asked whether the autonomous protection could be hard-disabled
  so it could never interfere with motion. A blanket disable was not
  implemented: it would also suppress torque-off for repeatable real
  over-temperature evidence. The observed false-positive mechanism was fixed
  instead, without a target/object-specific constant.
- Source inspection confirms every accepted SDK frame already passes the
  declared SDK-v6 length/end-byte and CRC32-low-byte checks before dispatch.
  The three temperature callbacks were independent roughly `1 Hz` responses,
  not one cached value incremented by the ordinary command timer. However, the
  old protection counter accumulated the maximum temperature from *any*
  channel into one global streak. The retained real sequence was:
  channel 10 at `163 C`, then channel 5 at `100 C`, channel 5 at `100 C`,
  followed immediately by all channels at at most `36 C`. The old global-max
  counter called that `1,2,3`; it did not require the same sensor/channel to
  remain hot.
- Replaced that global accumulation with one saturating consecutive counter
  per temperature channel. Every normal sample resets only its own channel;
  the protection decision uses the maximum current same-channel streak and
  records the one-based channel index in both warning and final protection
  evidence. A real channel that remains at or above the unchanged
  `60 C` limit for the unchanged three independent samples still latches
  protection and sends torque-off. High readings that move between channels
  no longer manufacture a sustained condition.
- Replaying the exact retained failure arrays against the new decision rule
  yields same-channel maximum streaks `[1,1,2,0]`, so that evidence would not
  latch at `3/3`. This is a sensor-channel temporal identity correction, not
  a carton-specific exception and not a permanent protection bypass.
- Added a regression that checks the driver's per-channel state and replays
  the exact four-frame sequence. The serial-driver resilience suite passes
  `4/4`; the related trajectory/driver configuration suite passes `11/11`;
  `git diff --check` passes. `catkin_make --pkg alicia_d_driver -j2` rebuilt
  and linked successfully. The new offline binary is `4,686,528` bytes with
  SHA-256
  `5b40087db5fbeb9eeef35693193c21812ca88a2b239148f3b94950bbdb725608`.
- The rebuilt binary was not hot-loaded. The current ROS driver remains the
  already-running, protection-latched old process; no node restart, torque-on,
  enable, stop, torque-off, serial write, or arm/gripper command accompanied
  this offline fix.
- Separately, the post-abort command-source investigation remains open. The
  current ROS graph has three `/joint_commands` publishers:
  `/bessica_d_hw_interface`, `/motion_gateway`, and
  `/alicia_supervisor_gui`. The ros_control hardware interface publishes the
  controller's changing desired positions, and after power loss its controller
  state still retains the observation endpoint as the desired position while
  actual feedback is stale. The retained logs do not identify the caller ID of
  each historical `/joint_commands` message, so no unsupported attribution is
  made yet for every post-abort command.

### New driver loaded and joint enable restored without trajectory replay

- After the operator powered the arm and explicitly requested joint enable,
  the powered hardware again supplied stable feedback near its initial pose:
  approximately `[-0.2,-0.6,-0.7,0.1,-0.5,-0.1] deg`, open gripper
  `raw=995`, ordinary status `0x00`, and maximum temperature around `36 C`.
  The already-running old driver remained protection-latched and blocked its
  command stream.
- Exited only the old `/alicia_d_driver_node` process and started the newly
  built `alicia_d_driver_node` under the same ROS name and retained private
  parameters. No `/grasp/stop`, arm stop, controller stop, disable, or
  torque-off command was sent. The new node opened `/dev/alicia_arm` at
  `1000000` baud and entered the retained
  `auto_torque_on_startup=true` path.
- The new same-channel temperature logic was observed live before enable
  confirmation: one channel-5 sample of `82 C` was logged as
  `same-channel max streak 1/3`, and the next response returned to a maximum
  of `36 C`. It did not latch or send torque-off. This directly proves the
  rebuilt source, not the old global counter, is running.
- Runtime state then confirmed `/alicia_d/motion_enabled=True` and
  `/alicia_d/protection_latched=False`. Feedback remained at the initial pose
  with ordinary temperatures. A five-second subscription observed no new
  `/joint_commands`, so the failed observation trajectory was not replayed
  when torque was restored.
- No new 6D plan or physical trajectory has been started. Perception remains
  target-lost at the initial camera pose. The next required operator action is
  to align the camera/arm with the target and report `已对准`; live supervision
  continues from the new driver terminal session `1463`.

### GUI post-move double correction traced to global driver endpoint trim

- During manual alignment, the operator reported that every GUI slider move
  was followed by two fixed-looking small arm motions. The correlated live
  trace proves this was not an assumed GUI animation: after a stable GUI
  endpoint, the driver repeatedly entered its global endpoint-feedback trim.
  One retained command produced iterations 1, 2, 3, a stalled-response retry
  counted as iteration 4, and iteration 5 before quantization-floor
  quiescence. Other slider endpoints visibly converged in two or three
  iterations. The accumulated offsets reached roughly `0.7-1.1 deg` on some
  joints. The two motions visible to the operator were the larger physical
  members of a longer internal correction sequence.
- This is the same mechanism as the previously reported small oscillatory
  correction after reaching the first observation pose. It applied to every
  `/joint_commands` source, including GUI/manual alignment, because the low-
  level driver had no task/source ownership boundary. Three publishers remain
  registered, but the exact trim-iteration log and the altered SDK command
  values close the cause of the post-settle moves inside the driver; no
  unsupported historical caller attribution is needed for that conclusion.
- Disabled `endpoint_feedback_trim_enabled` by default in the driver source,
  header, and both driver launch paths. The bounded implementation remains
  available only as an explicitly opted-in diagnostic mode; it is no longer
  part of ordinary GUI, ros_control, or grasp command delivery.
- Precision ownership is not removed or replaced by a fixed joint offset.
  The grasp task already has the object-independent measured-FK observation
  endpoint contract and retains exactly
  `observation_endpoint_correction_attempts: 1`. Thus a grasp observation may
  make at most one evidence-based Cartesian endpoint correction, while a GUI
  slider request no longer acquires an unrelated multi-iteration integral
  tail.
- Updated the configuration regression to require the ordinary driver path to
  keep trim disabled while retaining all bounds for explicit diagnostic use
  and requiring the task-level correction count to remain one. The driver/
  trajectory configuration suite passes `11/11`; serial resilience passes
  `4/4`; `git diff --check` passes; and
  `catkin_make --pkg alicia_d_driver -j2` rebuilt successfully.
- Set the retained private ROS parameter to false, exited the prior driver
  process, and started the rebuilt node in terminal session `70686`. It seeded
  the live pose around `[-102.7,10.5,-9.3,-0.2,3.3,0.0] deg`, retained the
  open gripper, and restored automatic joint enable without replaying a grasp
  trajectory. Runtime confirms
  `/alicia_d_driver_node/endpoint_feedback_trim_enabled=false`,
  `/alicia_d/motion_enabled=True`, and
  `/alicia_d/protection_latched=False`. No stop, disable, torque-off,
  controller-stop, arm-stop, or new grasp trajectory was sent.

### Operator confirmation and hard-gate review before the next execution

- The operator confirmed that, after disabling the driver-wide endpoint trim,
  one GUI slider command now produces exactly one visible arm movement. The
  operator then aligned the camera with the target again.
- The new live target belongs to target epoch `12` and remained stable near
  image coordinate `(345,274)`, depth `0.276-0.278 m`, and base position
  approximately `(-0.119,-0.440,0.060) m`, with confidence about `0.896`.
  Continuous inference produced a fresh preview, but no execution plan was
  promoted and `/grasp/start` was not called.
- Before that physical execution was started, the operator requested that
  overly restrictive hard gates be reduced and specifically stated that the
  grasp must not require excessive insertion depth. This request supersedes
  immediate execution of the existing preview. Physical execution is held
  while the retained gate audit is reviewed.
- The initial live evidence is not being treated as permission to delete
  collision boundaries: the candidate funnel showed many analytical
  `GRIPPER_SWEEP_COLLISION` / `OBSERVATION_SUPPORT_COLLISION` rejections, while
  some candidates reached strict MoveIt and at least one was reachable. The
  next analysis will separate invariant physical collision/finite-data
  requirements from tunable quality or ranking thresholds. Any adjustment
  must remain derived from live target geometry, gripper CAD, uncertainty, and
  support-plane evidence; it must not introduce a target-specific fixed depth.
- No stop, disable, torque-off, controller-stop, arm-stop, gripper command, or
  physical grasp trajectory was sent during this review transition.

### Evidence-based hard-gate reduction and wrist-symmetry binding fix

- The retained request-80 audit contains `45` locally evaluated candidates:
  `21` GraspNet and `24` tabletop-geometry candidates. It produced `21`
  locally valid tabletop candidates, `21` stable candidates and `32`
  materialized stable pose variants. The hard recheck passed only `15/32`.
- The dominant hard-recheck loss was not a physical collision:
  `SAFETY_BINDING_MISMATCH=15`. All evaluated variant-1 poses had no MoveIt
  result, while variant 0 proceeded to strict planning. Source inspection
  proved that tabletop support-plane reprojection stored an already
  materialized tool-`Rz(pi)` quaternion in `StableCandidate`; the generic
  `ScoredStableCandidate.variant_quaternion_xyzw` property then applied the
  same exact parallel-jaw symmetry a second time. The safety evidence retained
  the once-rotated physical pose, so every such variant falsely failed exact
  binding.
- Corrected only that representation boundary: after tabletop reprojection,
  variant 1 now stores its canonical quaternion by removing the materialized
  `Rz(pi)` once. The generic variant property then reconstructs the exact
  physical quaternion once. No pose tolerance, collision tolerance, target
  identity, or freshness rule was relaxed.
- The same audit showed `MOVEIT_JOINT_DELTA_LIMIT=3` hard rejections at the
  empirical `2.15 rad` cutoff. Joint path cost and maximum joint delta already
  have explicit soft-score components, while MoveIt joint limits, collision,
  IK, and planning failure are independently mandatory hard results. The
  default `candidate_max_joint_delta_rad` is therefore now `0.0`, disabling
  only this redundant preference cutoff and allowing both wrist symmetries to
  compete by measured motion score.
- Predicted camera centering/visibility is likewise now diagnostic plus soft
  ranking by default (`camera_visibility_gate_enabled: false`,
  `camera_visibility_diagnostic_enabled: true`). The first observation stage
  is followed by mandatory fresh near-field acquisition; predicted image
  margin alone no longer erases a collision-free candidate. Target presence,
  target epoch/identity and the actual fresh near-field plan remain mandatory.
- Physical hard gates remain unchanged: finite transform/depth contracts,
  physical jaw opening, target-instance absolute sanity bound, target/support
  geometry validity, finger/palm/static/swept-volume collision, observation
  support/target envelope, and strict MoveIt collision/joint-limit/IK/planning
  results.
- The apparent `25.2 mm` value in the current audit is
  `contact_approach_offset_m`: the distance from the near-field approach
  waypoint to the live-derived grasp pose. It is not a required penetration
  depth into the object. The selected tabletop candidate has no fabricated
  GraspNet depth; its contact line and tool pose are solved from the current
  object cloud, support plane and gripper CAD. Consequently this value was not
  blindly reduced or replaced by a fixed shallow depth.
- Added a regression that sends both reprojected tabletop wrist symmetries
  through the real mandatory safety-binding stage and requires both to reach
  the MoveIt checker. The streaming suite passes `191/191`; the core 6D
  pipeline suite passes `168/168`; the default-config suite passes `6/6`; and
  `git diff --check` passes.
- No physical execution was started, and no stop, disable, torque-off,
  controller-stop, arm-stop, or gripper command was sent during these changes.

### Clarification of “10 cm” and “too deep”

- The operator clarified that “cannot be too deep” referred to the *depth and
  strictness of the gate stack*—too many sequential hard gates—not to physical
  insertion depth. The preceding insertion-depth explanation remains a useful
  distinction but was not the operator's intended concern.
- Source inspection gives an exact answer for the fixed first-stage `0.100 m`
  value. It is **not** camera-to-target range. The construction intersects the
  target-centred camera optical-axis line with a sphere of radius `0.100 m`
  centred at the live candidate's final grasp `tool0` position. Therefore the
  invariant is:
  `|| observation_tool0 - candidate_grasp_tool0 || = 0.100 m`.
  It is most accurately described as a TCP/tool0 standoff from the
  live-derived final grasp TCP, not a Euclidean camera-to-object distance and
  not necessarily a Euclidean TCP-to-object-centre distance.
- The eye-in-hand transform is then used to choose the tool position on that
  sphere for which the camera optical axis passes through the live target.
  Camera-to-target optical depth is a derived value and can differ
  substantially from `0.100 m`; the request-80 selected audit reported about
  `0.152 m`. The implementation checks both the exact tool standoff residual
  and the optical-axis centring residual.

### Video evidence supersedes the 100 mm TCP-standoff observation policy

- The operator supplied
  `/home/zhuyupei/Videos/564909a11069220518f5836e1db67889.mp4`
  (SHA-256
  `ca064e7f3cca9631f64b133225e8f87fda10faed55aa3cc9a4a066507d7c394d`,
  `19.70 s`, `720x1280`, approximately `30 fps`). Eight time-distributed
  frames and the `17.5 s` full-resolution frame show that the completed
  far-field posture leaves the open gripper/installed wrist assembly in an
  unacceptably low, table-contacting posture. The operator directly observed
  table contact and additionally stated that the deployed RGB-D camera's
  working interval is `70-500 mm`.
- This is first-stage observation evidence, not contact-stage insertion
  evidence. It invalidates the old policy that optimized an exact
  `||observation_tool0 - grasp_tool0|| = 0.100 m` while allowing the actual
  camera range to be only about `0.152 m`. The prior analytical gripper/support
  envelope remains necessary, but a CAD pass does not override observed real
  contact or make the old observation objective acceptable.
- The operator replaced the first-stage requirement with a camera-to-target
  working-distance contract: nominal `0.200 m`, with the inclusive interval
  `[0.190, 0.210] m` accepted after the physical move. This supersedes the
  earlier permission to keep a fixed `100 mm` TCP standoff. The fixed value is
  a camera operating policy that applies to every live target; no object
  label, target coordinate, box dimension, joint pose, or paper-carton
  exception enters production pose generation.

### Camera-range observation generator and shallow gate chain

- Replaced `centered_observation_pose_at_standoff()` with
  `centered_observation_pose_at_camera_distance()`. For each current candidate
  orientation and live target, the solver uses the retained
  `tool0->camera_link` transform to place the camera optical centre exactly
  `0.200 m` back along its optical axis. It recomputes and requires optical-axis
  centring and Euclidean camera-to-target distance to `1e-8 m`. The resulting
  TCP-to-contact-TCP displacement is deliberately only an audit value; it is
  no longer a pose constraint.
- Production configuration now declares:
  `observation_camera_target_nominal_distance_m=0.200`,
  `observation_camera_target_min_distance_m=0.190`,
  `observation_camera_target_max_distance_m=0.210`, and
  `observation_camera_target_range_check_enabled=true`. The general RGB-D
  visibility interval was corrected from the old generic `35-1200 mm` fallback
  to the operator-specified `70-500 mm`.
- A fresh near-field preview is now eligible to replace the far-field plan
  only when the current `base_link->camera_link` TF and that preview's fresh
  target centre prove an actual Euclidean camera-to-target distance inside the
  inclusive `[0.190, 0.210] m` band. Thus `0.190` and `0.210 m` pass, while
  `0.189 m` fails as
  `OBSERVATION_CAMERA_TARGET_OUT_OF_RANGE`. This is the one requested
  observation-view gate, not another TCP endpoint approximation.
- Removed the far-field reuse of the `6 mm` contact endpoint gate and removed
  its second same-pose physical correction
  (`observation_endpoint_correction_attempts=0`). Far-field still records the
  actual tool0 position/orientation error in
  `/grasp_6d/runtime_execution_error`; near-field uncertainty consumes that
  evidence. The strict `6 mm / 5 deg` measured endpoint contract remains
  unchanged for the later contact-bearing stages.
- This removal is based on retained real execution evidence, not a relaxed
  guess. For plan `452d770c2c0cb19110764db8`, the first observation move
  settled with a `14.9 mm / 2.35 deg` error. The sole task-level correction
  then worsened it to `20.2 mm / 3.15 deg`; after the five-second hold it
  remained `19.5 mm / 3.05 deg` and failed
  `OBSERVATION_ENDPOINT_NOT_CONVERGED`. No near-field plan or contact stage
  executed. The new path records this free-space error and lets the fresh
  camera-range evidence, not a repeated motion, decide observation success.

### Reproducible current-plan clearance comparison

- A read-only `/grasp_6d/plan_enriched` sample supplied the current live target,
  candidate orientation, support plane, OBB, and old observation pose. Applying
  both policies offline with the unchanged retained hand-eye transform
  `xyz=(-0.0866127223,-0.0063407198,-0.1034120675)` and
  `q=(0.0229081321,-0.6787668594,0.0278470194,0.7334680031)` gave:
  old camera-to-target `0.152452463 m`; new `0.200000000 m`;
  old tool0 `z=0.068087025 m`; new tool0
  `xyz=(-0.091589815,-0.340241678,0.112521278) m`; and new derived
  tool0-to-grasp-tool0 displacement `0.128335548 m`.
- Running the unchanged open-gripper CAD/support-plane envelope on that same
  fixture gave old minimum support clearance `0.017840662 m` and new minimum
  support clearance `0.063109846 m`; both were analytically valid, but the new
  view increases modeled clearance by `45.269 mm`. This does not claim a
  completed real-arm verification. It proves offline that the new camera-range
  objective moves this retained sample in the expected safer direction while
  preserving the independent physical collision gate.
- Added this exact retained live sample only as a regression fixture; its
  coordinates do not occur in production logic. Added independent synthetic
  tests for exact centring, nominal distance, derived TCP displacement,
  invalid range configuration, inclusive `190/210 mm` task acceptance, and
  `189 mm` rejection.
- Validation passed: streaming/observation suite `192/192`; grasp task state
  machine `121/121`; remote-node core `154/154`; analytical gripper geometry
  `94/94`; production-default configuration `6/6`; Python compilation and
  `git diff --check` both pass.
- Source changes have not yet been used for a real trajectory. During video
  analysis, code changes, and offline validation, no `/grasp/start`, gripper
  motion, arm trajectory, `/grasp/stop`, controller stop, disable, torque-off,
  or stop-enable command was issued.

### Live reload and frozen 200 mm execution audit

- Loaded the new observation contract into the running ROS parameter server:
  nominal camera-to-target distance `0.200 m`, inclusive accepted interval
  `[0.190, 0.210] m`, actual-range check enabled, first-stage same-pose
  correction attempts `0`, and camera visibility interval `[0.070, 0.500] m`.
  Restarted only `grasp_task_node.py` and `remote_grasp6d_node.py` so their
  runtime code contains the new generator and audit fields. The arm driver,
  hardware interface, motion gateway, MoveIt, camera and GUI remained
  running; no arm stop, controller stop, disable, torque-off or stop-enable
  command was sent.
- A planning-only online generation produced and promoted request `3`, plan
  `89263b9ec90c367097bbf3f1`, from target epoch `1`. The frozen execution audit
  now carries the pose-construction evidence that was previously absent:
  `construction=camera_optical_axis_fixed_camera_target_distance`,
  nominal `0.200 m`, actual `0.20000000000000004 m`, inclusive limits
  `[0.190, 0.210] m`, optical-axis centre residual
  `6.94e-18 m`, and derived (non-constraining) tool0-to-grasp-tool0 distance
  `0.128702801 m`.
- The same selected candidate records observation tool0 position
  `(-0.089931209, -0.341075036, 0.112986387) m`, live target centre
  `(-0.117789369, -0.441943537, 0.055080975) m`, analytical minimum open-tool
  support clearance `0.062185639 m`, observation envelope accepted, and a
  strict planning-service success with reported joint path cost `1.005` and
  maximum joint delta `0.749 rad`. These values are plan-specific evidence
  computed from the current RGB-D target and retained transforms; none is a
  production target coordinate or fixed object-specific pose.
- The execution-audit publication path was corrected to copy
  `observation_view` into the promoted selected record. The streaming and
  observation regression suite was rerun and passed `192/192`; Python
  compilation and `git diff --check` passed. Continuous inference was then
  stopped to freeze this exact promoted audit rather than let a later preview
  overwrite the evidence.
- Read-only runtime confirmation after the freeze found the complete ROS graph
  still present, `/grasp_6d/status="continuous remote 6D inference stopped"`,
  and `/grasp/state` at `IDLE`, `active=false`, `message="ready"`. No physical
  trajectory has been started with plan `89263b9ec90c367097bbf3f1`; real
  table-clearance and the inclusive post-move camera-range gate therefore
  remain deliberately unclaimed until the operator confirms the arm and
  target are again in the intended starting arrangement.
- Continued terminal supervision showed the target stable near image centre at
  approximately `0.321-0.323 m` camera depth. The real driver continued
  streaming a stable command near
  `[-106.3, 30.0, -3.3, -9.6, -24.7, 8.3] deg`, with feedback near
  `[-106.4, 29.4, -4.0, -9.4, -25.4, 8.4] deg`, open-gripper feedback near
  raw `995`, and measured maximum temperature about `42-43 C`. Isolated
  hardware status events did not coincide with sustained same-channel
  over-temperature; the driver explicitly retained torque and emitted no
  torque-off. The frozen plan is audit evidence only and will be regenerated
  from a fresh image/target epoch before any later physical execution.

### Fresh 200 mm attempt: controller abort and exact torque-off cause

- After the operator confirmed the arm was at the intended initial position
  and the target was aligned, continuous inference was restarted. Generation
  `3`, request `12`, target epoch `3` reached `PREVIEW_READY`: `38/40` hard
  rechecks passed, `24` candidates reached strict MoveIt checking, `18` were
  reachable, and one stable candidate was promoted. The stream was frozen and
  the task node reported plan `99426918b0b0513d3cc20308`, source stamp
  `1785210013.921056270`, as `VALID` immediately before execution.
- `/grasp/start` was called with `execute=true` and that exact plan ID. The
  far-field target was the live-derived observation pose near
  `(-0.091,-0.340,0.113) m`; strict planning succeeded twice and the atomic
  controller sync passed. The cached trajectory was retimed to `14.798 s`,
  maximum joint delta `0.740 rad`, and velocity limit `0.080 rad/s`.
- At `1785210105.679957`, about `2.29 s` after execution began, MoveIt reported
  `PATH_TOLERANCE_VIOLATED: Joint6 path error 0.621719` and
  `ABORTED: CONTROL_FAILED`. The task entered `FAILED` and did not perform a
  near-field replan, gripper close, contact approach, or lift. However, the
  ros_control hardware interface continued publishing the remaining cached
  `/joint_commands`, reproducing the previously identified post-abort
  continuation defect. The driver naturally reached and held a command near
  `[-107.7,-13.6,31.6,-8.3,-39.1,7.2] deg`; no stop command was sent by
  Codex.
- The controller error did not match the real SDK feedback. For example, the
  driver reported the sixth joint command and feedback near `8 deg`, while the
  controller claimed a `0.621719 rad` sixth-joint path error. A later read-only
  sample showed `/alicia_controller/state.actual` disagreeing substantially
  with the contemporaneous SDK joint vector. `/joint_states` has exactly one
  publisher, `/alicia_d_driver_node`; `/bessica_d_hw_interface` consumes it
  and is the publisher of the trajectory action plus `/joint_commands`. This
  locates the control abort in the ros_control feedback/trajectory bridge, not
  in the 6D candidate selection.
- The arm then lost torque for a separately proven reason. Temperature channel
  `6` reported `174 C` at `1785210142.370`, again at `1785210143.370`, and a
  third time at `1785210144.477`. The driver counted this as a same-channel
  `3/3` over-limit streak. At `1785210144.577533` it logged
  `Protection evidence ... temperature_channel=6 streak=3`; at
  `1785210144.578195` it explicitly logged
  `Protection status latched; sending torque_off and blocking all motion
  commands`, set its software motion-enable state false, and wrote the SDK
  torque-off frame. This is the exact source of the observed loss of enable.
- The next temperature sample returned to a maximum of `45 C` in about one
  second, followed by ordinary `43-45 C` samples. A physical joint cannot cool
  from `174 C` to `45 C` in that interval. Therefore the retained evidence
  proves that the protection decision was triggered by an implausible
  telemetry/protocol value rather than a demonstrated physical overheat. The
  prior per-channel `3/3` mitigation is insufficient because the same corrupt
  value can repeat across polls.
- After observing the loss of enable, the operator switched off arm power.
  Subsequent missing `/joint_states`, stale-feedback messages, and target loss
  are consequences of that powered-off state and are not used to infer the
  earlier torque-off cause. No command to re-enable or move hardware will be
  issued while power remains off.

### Telemetry-only protection correction and operator-requested re-enable

- Removed the autonomous torque-removal side effect from the driver's
  temperature/status parser. A sustained high temperature byte sequence is
  now retained as a throttled diagnostic containing the channel, streak,
  payload and complete temperature vector, but it cannot set
  `motion_commands_enabled_=false`, latch protection, construct a torque-off
  frame, or write that frame. The explicit operator demonstration/zero-torque
  callback remains the only runtime torque-off path; this change does not add
  a hidden replacement stop path.
- Added a source-level regression asserting that the telemetry parser contains
  no autonomous `trigger_torque_off`, motion-disable assignment, or torque-off
  frame while the explicit operator callback still owns the manual torque-off
  path. The serial-driver resilience suite passed `5/5`, `git diff --check`
  passed, and `catkin_make --pkg alicia_d_driver -j2` built the driver target
  successfully.
- The rebuilt binary has deliberately not replaced the currently running
  driver yet. A hot restart while torque is on can reconnect multiple existing
  `/joint_commands` publishers to a driver with no command history. Retaining
  the known-stable process avoids turning that unresolved command-ownership
  condition into another physical movement. The running process therefore
  still has the old measured-temperature torque-off branch until a controlled
  driver reload is performed.
- After the operator restored arm power and explicitly requested enable,
  Codex published only `/demonstration=false`, which invokes the SDK
  `torque_on` path. The driver cleared the prior protection latch after its
  retained healthy-feedback interval and logged `Disabling zero-torque mode
  with SDK torque_on frame`. Codex did not publish `/grasp/stop`, stop a
  controller, disable motion, or send an SDK torque-off frame.
- Real SDK supervision after enable showed the arm settling and holding near
  command `[-104.4, 19.9, -8.7, 0.1, -11.0, -0.1] deg` with feedback near
  `[-104.5, 19.4, -9.1, 0.3, -11.7, 0.1] deg`. Ordinary measured maxima were
  approximately `37-40 C`; isolated `E1` status events reverted on the next
  frame and did not remove torque. The arm is presently enabled and stable.
- Immediately after enable, existing `/joint_commands` publishers replayed
  changing targets before the above hold was reached. The topic currently has
  three publishers: `/bessica_d_hw_interface`, `/motion_gateway`, and
  `/alicia_supervisor_gui`. Static source inspection proves that GUI feedback
  synchronization blocks slider signals and that the hardware interface emits
  controller command changes; it does not yet prove which publisher emitted
  each target in this transient. No object-specific or joint-specific
  compensation has been added. A publisher-identity trace is required before
  changing command arbitration.

### Controller replay, transient feedback fault, and residual-trajectory fix

- A simultaneous read-only sample corrected the earlier interpretation that
  the ros_control `actual` vector was persistently different from SDK state.
  `/joint_states` reported
  `[-1.817767,0.339010,-0.162602,0.004602,-0.202485,0.0] rad`, and
  `/alicia_controller/state.actual` contained the identical six values. The
  bridge's current joint-name mapping is therefore consistent. Its `desired`
  vector, however, remained at the failed trajectory endpoint
  `[-1.879667,-0.236742,0.551643,-0.144555,-0.681761,0.126491] rad` more than
  3000 seconds after the action abort, proving retained controller command
  state rather than a current physical-state mismatch.
- The retained failure sequence shows a separate deterministic replay before
  the new trajectory began. At `1785210103.403909` the gateway reported its
  controller-start check passed; only 17 ms later the driver received
  `[-106.8,-23.0,33.3,-9.7,-30.6,8.3] deg`, while the immediately preceding
  held command was `[-106.3,30.0,-3.3,-9.6,-24.7,8.3] deg`. The gateway had
  unconditionally sent a controller-manager start request even when both
  controllers were already running. This exposed retained position-controller
  state before the current-feedback bridge was installed.
- `_ensure_trajectory_controllers_started()` is now idempotent: it first lists
  controller states and returns without any switch request when all required
  controllers are already `running`. A switch request is reserved for an
  actually missing/stopped controller. This is state-derived and independent
  of target identity, joint pose, or object geometry.
- On any strict cached trajectory execution failure, the motion gateway now
  immediately replaces residual controller output with a one-point trajectory
  at fresh measured joint feedback and verifies that both controller desired
  and actual settle there. This keeps trajectory controllers and joint torque
  enabled; it does not call `/grasp/stop`, stop a controller, publish
  `/demonstration=true`, disable motion, or send `torque_off`. Its purpose is
  specifically to prevent the observed behaviour where an aborted action kept
  publishing the remainder of its old path.
- The `Joint6 path error 0.621719 rad` cannot describe the logged physical
  motion: Joint6 was commanded near `+8.3 deg` and accepted feedback was
  `+8.4 deg` at `1785210105.177`, while the next logged accepted sample was
  also `+8.4 deg`. Subtracting the controller error from its approximately
  `+8.3 deg` desired value implies a transient actual value near `-27.3 deg`,
  which is also the neighbouring Joint5 value at that instant. This supports
  one transient channel-like/decoded feedback sample; it does not support a
  real 35.6-degree Joint6 move in roughly half a second.
- Added a generic temporal-consistency filter to SDK joint feedback. Every
  joint uses the same bound
  `allowed_delta = 0.0123 rad + 1.2 rad/s * frame_interval`; `1.2 rad/s` is
  four times the configured `0.30 rad/s` command cap and `0.0123 rad` is eight
  encoder quanta. A larger discontinuity is withheld from `/joint_states`
  unless a second nearby sample confirms it within `0.05 rad`. Thus normal
  motion is not clamped and a genuine new pose is delayed by only one 10 Hz
  sample, while a lone impossible sample cannot trip ros_control path
  tolerance. The rule is applied uniformly to all joints and all tasks.
- Related motion-gateway and serial-driver tests passed `29/29`; Python
  compilation, `git diff --check`, and
  `catkin_make --pkg alicia_d_driver -j2` passed. No physical motion command
  was issued while diagnosing, editing, compiling, or testing. A read-only
  `/joint_commands` caller-ID watcher remains active for the next command.

### Fresh post-fix 6D plan generation and live deployment

- The rebuilt driver was deployed by cleanly terminating only the old driver
  process; its destructor disconnected the serial port and did not send a
  torque-off frame. The replacement driver started with the configured
  `auto_torque_on_startup=true`, reported `/alicia_d/motion_enabled=true`, and
  held stable feedback near `[-104.2,19.4,-9.3,0.2,-11.6,0.0] deg` with
  `status=0x00` and ordinary temperature samples near `38 C`. No stale
  `/joint_commands` replay occurred after this restart.
- The updated motion gateway was also deployed. Its ROS launch owner respawned
  the node from the rebuilt devel wrapper; the duplicate manual start shut
  itself down because the canonical node name was already owned. No controller
  stop, motion-disable, `/demonstration=true`, or torque-off command was sent.
- At `1785214114.709789514`, live perception confirmed the operator's current
  alignment: `label=carton`, confidence `0.897375`, image center `(323,199)`,
  depth `0.2843 m`, camera point `(0.2843,-0.003384,0.030061) m`, and base point
  `(-0.119778,-0.437987,0.061265) m`. Subsequent frames remained close to this
  measurement, so a fresh candidate stream was started; the prior failed plan
  was not reused.
- The first new generation returned 11 locally accepted candidates and stayed
  at `STABILITY_PENDING`. A later cross-frame result reported 21 stable inputs,
  19 passing stability, 24 MoveIt checks with 12 reachable, one preview passing
  final selection and one promoted plan. The terminal status was
  `PREVIEW_READY`; the task node accepted fresh plan
  `1fc0cb6c4103a4629ad3690e` with source stamp
  `1785214264689140796`. The continuous inference stream was then stopped to
  freeze this promoted plan before execution. The hard checks are derived from
  current object/support geometry and robot reachability; no target-specific
  coordinate or joint value was introduced.

### First live execution after controller/feedback fixes

- The first promoted plan became stale before execution and was rejected by
  the task node as `PLAN_STALE`; no motion command was sent for it. This was
  the configured 120-second source-image validity rule, not a planning or
  driver failure. A new stream was therefore generated rather than bypassing
  freshness or replaying the old plan.
- Fresh plan `c1416fca334e5e6f4c1b0578` (source stamp
  `1785214495269852399`) was accepted at `1785214522.060475`. Its promotion
  result had 17 of 21 candidates pass cross-frame stability, 9 of 24 pass
  MoveIt reachability, and one final candidate pass selection and promotion.
  Execution was bound to that exact plan ID while it was still valid.
- The far-field observation motion completed and settled. Measured endpoint
  error was `0.0132 m` in position and `2.24 deg` in orientation; the task then
  entered `waiting for near-field 6D preview`. The observed command stream was
  emitted by `/bessica_d_hw_interface`; no GUI or motion-gateway duplicate
  command source appeared in the caller-ID trace. Driver feedback remained
  stable near `[-107.9,-12.0,13.4,-27.0,-13.7,25.8] deg`, status returned to
  `0x00`, and no torque-off or motion-disable command was issued.
- Near-field inference did not produce an executable contact plan. Across the
  150-second wait, remote frames repeatedly returned 16-28 candidates after
  remote collision filtering but zero passed local contract materialization.
  Some frames had 6-10 baseline-safe candidates, proving this was not simply
  a lack of tabletop-clear candidates. The audit reports
  `GRIPPER_CONTACT_PATCH_MISS`: the dynamically computed required bilateral
  contact overlap was about `0.016236 m`, while the best observed overlaps for
  representative proposals were only `0.001190-0.001256 m`; all tabletop
  proposals therefore failed materialization. The task timed out at
  `1785214731.201157` with `NEAR_FIELD_REPLAN_TIMEOUT` and released its
  execution slot while leaving the enabled arm holding the observation pose.
- This evidence localizes the remaining failure to the near-field contact-patch
  geometry/units and phase contract, not the hand-eye transform, first-stage
  controller execution, joint feedback, or an autonomous disable path. No
  threshold is being relaxed until the derivation of required overlap and the
  measured patch projection are compared in the same geometric frame.

### General 6D contact-frame correction after the live timeout

- Re-read the earlier fixed-`6.5 mm`, centred-view, contact-boundary, and
  endpoint-error sections before editing. They already proved that merely
  subtracting the observation endpoint residual cannot create sufficient
  bilateral evidence on the tabletop fallback: prior centred-view maxima were
  only `0.660-1.688 mm`, below even the `3 mm` live perception/CAD floor. The
  same logs identified the unfinished independent defect: all raw GraspNet 6D
  poses were rejected before CAD evaluation when their jaw axes were not
  exactly parallel to the support plane.
- The old `bilateral_contact_height_bounds_m()` expressed target heights along
  the world support normal but also required that vector to lie in the finger
  contact plane. This is valid for horizontal jaw axes only and is not a
  general 6D contact model. A tilted raw candidate therefore failed with
  `jaw axis must be parallel to the support plane`, regardless of its actual
  two-sided target evidence or finger CAD coverage.
- Added a source-independent contact-height construction: project the live
  support normal into the plane perpendicular to each candidate's live jaw
  axis and normalize it. Negative/positive jaw-side point bands, target OBB
  extent, and the Link7/Link8 contact-patch intersection are now all measured
  along this same metric axis. For tabletop-parallel jaws this reduces exactly
  to the former support-normal calculation; for a tilted 6D jaw it evaluates
  the real contact plane instead of rejecting the orientation by definition.
  A jaw parallel to the support normal still fails closed because no
  support-derived in-pad height direction exists.
- Separated two previously conflated quantities. The measured Cartesian
  endpoint residual continues to expand adaptive pregrasp, approach, lateral
  sweep and lift clearances, because it is relevant to future motion. It no
  longer becomes a minimum finger/object overlap, because a past tool tracking
  residual is not uncertainty in the newly captured contact surface. Contact
  overlap must resolve only the frozen snapshot's live perception/CAD
  uncertainty. For this run that changes the semantic requirement from about
  `16.236 mm` to its measured perception component near `3 mm`; this is not a
  replacement fixed carton threshold, and the prior evidence proves the
  tabletop-only branch may still fail it. The orientation-aware raw 6D path is
  the substantive added evidence source.
- Verification passed: analytical gripper geometry `95/95`, full remote
  streaming/planning `193/193`, remote-node core `154/154`, tabletop plus
  adaptive-stage profiles `38/38`, Python compilation, and
  `git diff --check`. The new regression constructs a 45-degree jaw axis and
  proves that the projected height axis is unit length, orthogonal to the jaw,
  and preserves the known bilateral interval. Another regression proves a
  `13.2 mm` tracking residual remains in motion clearances but is excluded from
  a `3.0 mm` perception overlap requirement. No hardware command was issued
  during diagnosis, edits, or tests.

### Live execution after the general 6D contact-frame correction

- Generated and froze fresh plan `d3751477a1d420e0c8307cf1` with source stamp
  `1785215627010364532`; the task node reported the exact plan as `VALID` before
  `/grasp/start` was called. No earlier plan was replayed.
- The far-field observation move completed and settled after `0.90 s`. Measured
  endpoint error was `0.0125 m` and `2.08 deg`. The arm then held near
  `[-115.2,-16.6,32.4,-19.2,-41.4,33.7] deg`; driver status returned to
  `0x00`, the gripper remained open near raw `995`, and the caller-ID trace
  showed only `/bessica_d_hw_interface` as the joint-command source. There was
  no repeated correction of the same observation pose and no autonomous
  torque-off/disable event.
- The near-field phase timed out after `150 s` without moving the arm or closing
  the gripper. The terminal evidence was
  `NEAR_FIELD_REPLAN_TIMEOUT`, with the last preview rejected as
  `NEAR_FIELD_PLAN_PHASE_INVALID` because it was not a contact execution plan.
  The task released its execution slot while the enabled driver continued to
  hold the observation pose.
- This run contained three near-field requests (`11`, `13`, `22`) in which all
  14 locally valid candidates also passed tabletop materialization. Each ended
  at `STABILITY_PENDING`: `stable entered=14, passed=0, rejected=14`. Other
  requests had no materialized candidate. Across the same interval, the hard
  analytical rejections were 52 `GRIPPER_CONTACT_PATCH_MISS`, five
  `GRIPPER_TOO_NARROW`, and two `GRIPPER_SWEEP_COLLISION`. The physical 50 mm
  opening, collision geometry, bilateral contact evidence, and 3 mm live
  perception uncertainty therefore remain enforced.
- The target was not moving: 108 perception records stayed around image point
  `(355,200)`, depth `0.176-0.177 m`, and base centre
  `(-0.136,-0.443,0.067-0.068) m`, with confidence about `0.899-0.904`.
  Consequently the remaining timeout is localized to intermittent near-field
  contact evidence plus the cross-request temporal contract, not target drift,
  the corrected hand-eye transform, or first-stage execution.
- Code inspection found that each five-frame fused planning snapshot currently
  requires only its newest source stamp to advance. Four older samples may be
  reused by the next request (`wait_for_samples`, lines 611-618), while the
  candidate tracker independently requires three hits in five requests. It is
  therefore not scientifically valid to describe a two-snapshot rule as ten
  independent RGB-D frames without first changing the sampling contract. No
  temporal threshold has been relaxed on that unsupported assumption.

### Near-field independent temporal-evidence correction

- Corrected the sampling contract before changing the temporal decision. In
  the near-field execution phase, `wait_for_samples()` must now return five
  synchronized RGB-D/mask/object samples whose *oldest* source stamp is newer
  than the newest stamp submitted for the preceding request. Adjacent
  near-field planning windows therefore cannot share a source frame. Far-field
  planning retains the prior rolling-window behaviour and its three-hit rule.
- A near-field candidate now needs two geometrically matched hits from two
  disjoint five-frame windows: ten distinct source frames. It must be present
  in the current request, after which the fused candidate is still subjected
  to the unchanged current-snapshot analytical geometry recheck, physical
  50 mm jaw-width limit, bilateral contact evidence, 3 mm live perception
  uncertainty, support/sweep collision checks, exact phase sequence checks,
  and bounded MoveIt reachability. This is a phase-specific statistical
  evidence rule, not a carton coordinate, pose, width, or fixed grasp path.
- Added immutable `sample_stamp_ns` evidence to each fused snapshot and bounded
  tracker diagnostics (`window_request_ids`, `track_count`, `max_hit_count`,
  per-track hit request IDs and source). Pipeline metrics and the atomic gate
  audit now expose the five source timestamps, uniqueness count, source span,
  whether disjointness was required, and the actual hit count needed. The next
  live run can therefore verify both independence and candidate repeatability
  directly instead of inferring them from `STABILITY_PENDING`.
- Regression verification passed: candidate stability `66/66`, RGB-D snapshot
  fusion/buffering `51/51`, remote streaming/planning `195/195`, remote-node
  core `154/154`, Python compilation, and `git diff --check`. No hardware,
  trajectory, gripper, stop, controller-disable, demonstration, or torque-off
  command was issued during diagnosis, editing, or tests.

### First live audit telemetry and JSON content-binding fix

- Hot-loaded only the remote 6D planner and started a new far-field candidate
  generation. Requests 1-3 proved the new telemetry path: every fused snapshot
  contained five unique source timestamps; tracker evidence advanced from one
  to two to three matching hits. Request 3 produced 15 stable candidates and
  eight strict-MoveIt-reachable candidates, so a valid Preview was generated.
  No arm or gripper command was published.
- Execution promotion correctly failed closed with
  `PLAN_AUDIT_NOT_READY: execution audit is missing or no longer content-bound`.
  The exact cause was introduced by the new diagnostics: Python tuples in
  `tracking_evidence` serialize to JSON arrays, so the disk JSON decoded to
  lists and was not equal to the in-memory report. The SHA payload was valid,
  but the deliberate disk-report equality check rejected promotion. This was
  not a candidate, MoveIt, robot, or calibration failure.
- Stopped only the candidate inference stream and changed all newly added audit
  sequences (`window_request_ids`, per-track `hit_request_ids`, track rows, and
  source frame timestamps) to JSON-native lists before content binding. Added
  a strict JSON round-trip equality regression. Candidate stability `66/66`,
  remote streaming/planning `195/195`, Python compilation, and
  `git diff --check` pass after the fix. Mechanical-arm enable/hold was not
  changed and no stop/disable/torque-off command was issued.

### Fresh-plan validity boundary and event-driven execution handoff

- Two otherwise accepted far-field plans (`a4dcdc0656c625a2f0ede788` and
  `ccfc605013283a948845735f`) were rejected by `/grasp/start` as `PLAN_STALE`
  before any arm motion. Subsequent timestamp evidence corrected the initial
  suspicion of a current-plan/start inconsistency: `/grasp_6d/plan_validity_sec`
  is `120.0 s`, both service paths use the same configured validity, and the
  second plan source stamp was already about `222 s` old when checked later.
  A validity query performed near the boundary followed by manual analysis can
  therefore cross the same 120-second limit before execution. There is no
  evidence of two conflicting validity implementations.
- Replaced that timing-sensitive manual handoff with an event-driven, exact-ID
  helper. It listens only for newly promoted `/grasp_6d/plan_enriched` messages,
  rejects previously seen plan IDs and any message already older than `30 s`,
  verifies that `/grasp/current_plan` returns the identical `plan_id` with
  `success=True`, and immediately calls `/grasp/start` with `execute=True` and
  that exact ID. This helper does not publish trajectories, stop, disable,
  demonstration, or torque commands; the task node still applies its native
  120-second plan validity and every drift/phase/geometry gate.
- The helper accepted newly generated plan `9cc28c2dcd951416e2a25d7d`, source
  stamp `1785218295377449035`, at observed age `18.279 s` and immediately began
  the normal grasp-task execution. The far-field observation move settled after
  `0.95 s`; its measured endpoint errors were `0.0146 m` position and
  `2.49 deg` orientation. The task then entered `PLAN_PREGRASP waiting for
  near-field 6D preview` while the enabled controller held the observation
  pose. Near-field perception was stable near image point `(330,206)`, depth
  `0.175-0.176 m`, and base centre `(-0.129,-0.444,0.069-0.070) m`.
- Live near-field requests expose `disjoint_window_required=true`, five unique
  timestamps per snapshot, and `required_hits=2`. At the time of this entry,
  requests 24-25 had no candidate surviving current-snapshot analytical contact
  materialization (`max_hit_count=0`); observed rejections were bilateral
  contact-patch insufficiency or sweep/support clearance, so no second-stage
  motion had been authorized. This is a measured physical-gate outcome, not a
  hard-coded target pose or an assumed explanation. Execution monitoring
  remains active.

### Current live attempt ended fail-closed; user-requested pause

- Plan `9cc28c2dcd951416e2a25d7d` remained at the first observation pose until
  the unchanged `150.0 s` near-field deadline. The exact task result was
  `NEAR_FIELD_REPLAN_TIMEOUT: no fresh near-field 6D preview; last=`
  `NEAR_FIELD_PLAN_PHASE_INVALID: Preview is not a contact execution plan`;
  the event-driven caller returned `success=False, message=failed` and the
  execution slot was released. This was a natural fail-closed timeout, not a
  stop or disable command.
- Across requests 24-37, each near-field snapshot contained five unique source
  timestamps and required a disjoint window. The target stayed essentially
  fixed near `(330,206)`, depth `0.175-0.176 m`, base centre
  `(-0.129,-0.444,0.070) m`, confidence about `0.903-0.906`. No candidate
  reached temporal tracking because current-snapshot analytical
  materialization passed zero candidates. Repeated measured failures included
  continuous bilateral contact overlap around `1.7-2.3 mm` below the live
  `3.0 mm` requirement, absent common bilateral height, openings above the
  physical `50 mm` gap, one-sided finger reach, and support/sweep collision.
  No contact plan was promoted, so no second-stage arm motion or gripper close
  occurred.
- Source/history review confirms that the `3.0 mm` live floor is not the
  removed fixed `6.5 mm` rule: it is the physical gripper/support clearance
  lower bound in the existing live uncertainty model. The current observation
  policy is explicitly camera-to-target, configured as nominal `0.200 m` and
  accepted interval `0.190-0.210 m`; it is not a TCP-to-target offset. The
  fresh perception depth after arrival was about `0.175-0.176 m`, while the
  task recorded a `14.6 mm` tool endpoint residual. The exact relationship
  between that residual and the camera-range acceptance path still requires
  trace-level diagnosis; no gate or trajectory was changed on an assumption.
- When the user requested a pause, called only
  `/grasp_6d/request_plan {trigger:false}`. The service confirmed
  `continuous remote 6D inference stopped`. The grasp task was already
  inactive after its natural timeout. No `/grasp/stop`, controller stop,
  demonstration, torque-off, disable, arm trajectory, or gripper command was
  sent; the arm remains enabled and holds its existing pose. Further diagnosis
  and execution are paused until the user resumes.

### Post-arrival camera-range control-flow defect corrected offline

- After the user resumed, source tracing proved that the inclusive
  `0.190-0.210 m` camera-to-target validator existed but was called only inside
  `_copy_near_field_preview_candidate()`, after a near-field contact preview
  had already been generated. The failed run never produced such a preview,
  so its measured `0.175-0.176 m` view was never range-validated; the task
  waited the full 150 seconds instead. This contradicts the configuration
  comment that the reached observation itself is the success criterion and is
  the exact reason an out-of-range first stage could enter the near-field wait.
- Moved the range authorization boundary to immediately after the far-field
  observation motion settles and its endpoint error is recorded, before
  `_maybe_rebind_near_field_grasp6d_plan()` can request contact candidates.
  The task now waits for an object source timestamp captured after the reached
  view, checks freshness, label identity, finite live centre, and the existing
  40 mm target-drift contract, then computes Euclidean distance from the
  current `base_link->camera_link` TF to that same copied live centre. Only the
  existing inclusive `[0.190,0.210] m` interval authorizes near-field planning.
  An out-of-range view fails immediately while preserving arm enable; it does
  not make a second same-pose correction and does not wait 150 seconds.
- Added regressions proving that a fresh live centre at exactly `0.175 m` is
  rejected even if the frozen far-field plan centre would imply another
  distance, that a post-arrival source sample drives the decision, and that a
  range failure prevents the near-field request function from being called.
  Grasp task state-machine tests pass `124/124`; production default-config
  tests pass `6/6`; Python compilation and `git diff --check` pass.
- Read-only runtime evidence also confirmed that the prior endpoint record is
  still present and exact: plan `9cc28c2dcd951416e2a25d7d`, requested tool0
  `(-0.096834,-0.346488,0.118215) m`, actual
  `(-0.093832,-0.340123,0.105381) m`, error vector
  `(+3.002,+6.365,-12.833) mm`, norm `14.636 mm`, orientation error
  `2.488 deg`. No parameter was changed by this read.
- A later read-only sample, after the pause/resume interval, showed the target
  at camera coordinates `(0.3222,0.00425,-0.00635) m`, Euclidean range about
  `0.3223 m`, with the task already `FAILED`, `active=false`. This is no longer
  the prior failed 0.175 m observation state. Available evidence does not
  establish whether the robot or target was moved in the interval, so no cause
  is attributed. No arm, gripper, stop, disable, or torque command was issued
  during the diagnosis, edits, tests, or read-only ROS sampling.

### Live post-arrival range gate confirmed; single measured retreat added

- Hot-reloaded only `grasp_task_node.py` with the corrected post-arrival range
  authorization. The real driver, hardware interface, MoveIt, camera, remote
  planner and arm enable remained unchanged. Generated and executed fresh plan
  `73eaa603dac7f76269e639c0`, source stamp `1785219735862877607`.
- The first observation move settled after `0.90 s`. Its measured tool0 error
  was `12.852 mm / 2.133 deg`, with vector
  `(+4.149,+5.595,-10.801) mm`. The first post-arrival target sample then gave
  Euclidean camera-to-target distance `0.1785 m`; the new task logic immediately
  returned `OBSERVATION_CAMERA_TARGET_OUT_OF_RANGE` against the inclusive
  `[0.1900,0.2100] m` contract. It did not request a near-field preview, move a
  second contact stage, close the gripper, or wait 150 seconds. This is direct
  real-arm confirmation of the corrected gate placement.
- Stopped only the still-running candidate inference stream after the task
  failed. No arm stop, controller stop, disable, torque-off, demonstration,
  gripper, or extra trajectory command was sent. The arm remained enabled at
  the reached observation pose.
- The preceding run's endpoint vector was
  `(+3.002,+6.365,-12.833) mm`; the current vector is
  `(+4.149,+5.595,-10.801) mm`. Their difference norm is about `2.46 mm`, while
  both point toward the same repeatable first-stage execution bias. The current
  stable live target is at camera coordinates approximately
  `(0.1778,-0.00536,+0.01921) m`, and the current base-to-camera translation is
  approximately `(-0.121,-0.369,+0.232) m`.
- Added one general, measured correction for a too-close observation only. It
  preserves the current measured wrist orientation, computes the radial camera
  retreat from the current live target/camera positions to the configured
  nominal `0.200 m`, and subtracts the current run's measured Cartesian
  endpoint-error vector from the commanded translation as feed-forward. The
  derived correction is strictly planned through MoveIt, executed at most once,
  and followed by a new post-arrival source sample and the same inclusive range
  check. A too-far view is never advanced toward the target by this path.
- This correction contains no object label branch, carton coordinate, saved
  joint offset, fixed trajectory, or repeated same-pose servo loop. Its only
  fixed physical policy is the already agreed camera interval/nominal; every
  translation component comes from the current target, camera TF and measured
  endpoint sample bound to the active far-field plan. Added pure-geometry and
  strict-preflight/execution regressions, plus a production-default assertion.
  Grasp state-machine tests pass `126/126`, default-config tests `6/6`, Python
  compilation and `git diff --check` pass. No hardware command was issued while
  implementing or testing it.

### Live measured-retreat execution diagnosis

- The reached-observation reuse path was tightened before this run: TCP-pose
  proximity alone can no longer reuse a view. Reuse also requires the current
  live Euclidean `camera_link`-to-target distance to satisfy the same inclusive
  `[0.190,0.210] m` contract. This prevents the known approximately `0.1785 m`
  view from bypassing the post-arrival range gate. The grasp task suite passed
  `127/127` after this addition.
- Hot-loaded the current task node and enabled the one-time measured retreat
  parameter. Fresh plan `386a0b7202cf02ef0e0150a9`, source stamp
  `1785220515774415016`, was accepted and started by the exact-plan helper at
  an observed age of about `23.678 s`. The invalid prior too-close view was not
  reused; the normal first observation motion executed and settled after
  `0.90 s`.
- The first observation endpoint error was about `13.3 mm / 2.26 deg`. The
  fresh camera-to-target distance was `0.181770019 m`, so the task derived one
  real-time retreat from the current camera/target geometry and this run's
  measured endpoint vector. The resulting tool translation command was
  `[-0.003028,+0.002129,+0.028187] m` (norm `0.028429 m`), comprising radial
  retreat `[+0.000428,+0.007656,+0.016539] m` minus measured execution error
  `[-0.003457,-0.005527,+0.011648] m`. The current measured wrist orientation
  was preserved. Strict planning succeeded.
- Physical execution then returned failure for target
  `xyz=(-0.125,-0.330,0.131) m`. Exact controller evidence at
  `1785220563.736` is `GOAL_TOLERANCE_VIOLATED: Joint2 goal error 0.037009`;
  the current software endpoint band is `0.030+0.005=0.035 rad`, so the
  reported excess was `0.002009 rad`. MoveIt consequently reported
  `ABORTED: CONTROL_FAILED`. The driver remained motion-enabled and explicitly
  sent no torque-off. The task failed before near-field planning, contact
  approach, close, or lift. Continuous candidate inference was later stopped
  successfully without changing arm enable.
- Immediately before the controller abort, fresh target vectors were about
  `(0.188,-0.017,+0.019) m`, Euclidean distance approximately `0.1897 m`, at
  the lower edge of the observation contract. After the abort, the motion
  gateway installed its existing current-feedback hold. It changed the
  controller command from approximately Joint2 `-13.7 deg` to the captured
  feedback near `-15.8 deg`; the next hardware feedback moved to about
  `-17.1 deg`, and the target range fell back toward approximately `0.179 m`.
  This proves the post-failure feedback-as-command bridge introduces an extra
  physical step on hardware with a persistent command/feedback offset.
- A later change from approximately `0.18 m` to `0.306 m` must not be used as
  execution evidence: the operator explicitly reported moving the arm during
  that later interval. The earlier provisional inference that this later
  change proved continued retreat or overshoot is withdrawn.
- General correction direction: preserve the controller's last desired
  command after a failed trajectory instead of republishing current feedback
  as a new command. For this observation-only retreat, a controller failure
  may proceed only to the existing settle wait and a fresh real RGB-D range
  validation; it is accepted only if the measured view actually satisfies the
  unchanged inclusive range contract. This recovery will not apply to
  near-field contact, close, or lift stages, whose strict execution failures
  remain terminal. No arbitrary joint-tolerance relaxation is planned from
  this single result.

Implemented offline:
- Replaced the post-failure call to the pre-execution feedback synchronizer
  with a dedicated controller-desired hold. It reads the fresh
  `/alicia_controller/state.desired` vector and republishes that same vector as
  the one-point hold. It does not substitute `actual`/`/joint_states` values,
  so it does not intentionally introduce the measured command discontinuity.
  The controller and joint torque remain enabled; no controller-stop or
  torque command was added.
- The general strict-execution path still fails on a controller error. Only
  the far-field observation retreat may continue to the existing motion-settle
  wait after the exact strict-cached-controller failure message. It then has
  no success authority of its own: the caller must obtain a fresh target sample
  and pass the unchanged live Euclidean camera-distance interval before any
  near-field planning can begin. A planning/synchronization/communication
  failure, a contact-plan execution, close, or lift cannot use this path.
- Regression results: motion-gateway/controller tests `24/24`; grasp task
  state-machine tests `128/128`; Python compilation and `git diff --check`
  pass. The regression explicitly uses different desired and actual joint
  vectors and asserts that the failure hold publishes the desired vector.
  These checks were offline and issued no hardware command.

Live hot load:
- Restarted only `/motion_gateway` and `/grasp_task_node`. The main launch
  respawned the motion gateway as PID `69377`; the replacement task node is
  supervised in terminal session `88883`. Driver, ros_control hardware
  interface, trajectory controllers, MoveIt, camera, GUI, remote planner and
  hand-eye TF were not restarted.
- All strict planning/execution and grasp services are present. Read-only state
  is `/alicia_d/motion_enabled=True` and `/grasp/state=IDLE`, `active=False`,
  `message=ready`. Candidate inference remains stopped. No trajectory,
  gripper, stop, disable, demonstration, controller-stop, or torque command was
  sent during the hot load.
- The operator reported moving the arm before this hot load. Current target
  image/range data therefore describes the new operator-selected posture and
  is not used as a continuation of the failed retreat audit.

### Near-field phase semantics defect identified offline

- The operator confirmed that the later arm/target pose change was caused by
  a manual arm move. Any earlier interpretation of that later change as
  continued retreat motion is withdrawn; it is excluded from execution
  evidence and from the following correction.
- The latest near-field candidate audit recorded
  `execution_position_error_m=0.0`, even though the task later published the
  plan-bound runtime sample for plan `386a0b7202cf02ef0e0150a9` with
  `position_error_m=0.01334767325013406`, stamp
  `1785220551.2463427`, phase `FAR_FIELD_OBSERVATION_PLAN`.
- Source and timestamp tracing gives an exact control-flow explanation. The
  remote planner currently assigns `robot_execution_active=msg.active` in
  `/grasp/state` and uses that same boolean as its near-field planning phase.
  The task set `active=True` at approximately `1785220539.522`, before the
  far-field observation motion; the measured endpoint record was not
  available until approximately `1785220551.246`. Candidate generation was
  therefore reset to `near_field` about 11.7 seconds too early and could only
  read the default zero execution uncertainty.
- `/grasp/state.active` means that the complete task owns its execution slot;
  it is still required for frozen execution authority and occlusion handling.
  It does not prove that the observation range has passed or that near-field
  replanning is authorized. The two meanings must not share one boolean.
- Planned correction: add a dedicated latched Boolean
  `/grasp/near_field_active`. The task will publish false at startup and task
  boundaries, keep it false throughout observation planning/motion/range
  correction, and publish true only after a fresh real RGB-D camera-to-target
  sample passes the unchanged inclusive `[0.190,0.210] m` contract, immediately
  before requesting the near-field preview. The remote planner will use this
  dedicated signal only for planning-window reset, fresh-window collection,
  phase-specific candidate generation, and runtime endpoint-error consumption;
  execution-authority decisions will continue to use `/grasp/state.active`.
- This change adds no target-specific coordinate, joint value, candidate
  threshold, motion command, controller stop, torque-off, or disable path.
  Candidate inference remains stopped and no hardware command is issued during
  this offline correction.

Implemented and verified offline:

- `grasp_task_node.py` now publishes the latched Boolean
  `/grasp/near_field_active`. It publishes false on node startup, accepted task
  start, task completion/failure cleanup, and stop-state cleanup. During a
  far-field observation task it remains false through preflight, physical
  observation motion, measured endpoint recording, optional one-time retreat,
  and fresh range validation. It changes to true only after that validation
  returns success and immediately before `_maybe_rebind_near_field_grasp6d_plan`
  requests a new preview. This publisher contains no motion operation.
- `remote_grasp6d_node.py` now subscribes to the dedicated phase topic after
  its streaming state is initialized. `grasp_state_cb` only updates
  `robot_execution_active`, preserving frozen execution authority, promotion
  exclusion, and occlusion behavior. The new `near_field_state_cb` alone
  advances the target epoch and resets the multi-frame candidate window at a
  real planning-phase boundary.
- Runtime endpoint-error consumption, `PreparedPrediction.near_field`, and the
  requirement that every snapshot frame be newer than the phase boundary now
  use `near_field_planning_active`. Fallback phase selection for stable MoveIt
  and Preview materialization uses the same dedicated variable. The remaining
  `robot_execution_active` uses were audited and intentionally retained only
  for execution-authority/freeze/replan behavior.
- Regressions prove that task `active=True` alone does not reset the far-field
  tracker, the explicit false-to-true near-field transition resets it exactly
  once, repeated true messages are idempotent, and true-to-false resets the next
  far-field epoch. They also prove that a fresh nonzero runtime execution-error
  parameter is ignored while the task is active but near-field is false, then
  consumed when the explicit near-field signal becomes true.
- Task regressions prove the phase remains false during the first observation
  move, becomes true before the first contact-stage motion only after the range
  gate succeeds, remains false if the range gate fails, and publishes the
  latched sequence `false,true,false` without duplicate messages.
- Final offline results: remote streaming/planning tests `195/195`; grasp task
  state-machine tests `129/129`; Python compilation and `git diff --check`
  pass. No ROS service, trajectory, gripper, controller-stop, disable,
  torque-off, or candidate-generation command was issued by these edits/tests.

## 2026-07-28 - latest full ROS chain resumed for aligned-target execution

- At the operator's explicit request, started the latest complete real-arm
  chain directly from the current worktree with:
  `roslaunch alicia_flexible_grasp_supervisor full_system.launch
  start_real_arm:=true driver_port:=/dev/alicia_arm
  driver_baudrate:=1000000 auto_torque_on_startup:=true
  start_camera:=true start_tactile:=false start_gui:=true
  use_remote_grasp6d:=true
  remote_grasp6d_url:=http://172.23.132.97:8000`.
- The real serial driver opened `/dev/alicia_arm` at `1000000` baud; hardware
  feedback remained stable near
  `[-110.5,+25.9,-2.7,-3.2,-26.5,+5.9] deg`, open gripper raw `995`, and
  normal status `0x00`. The latest motion gateway, ros_control controllers,
  MoveIt, camera, perception, GUI, pruned-19 hand-eye TF, remote 6D planner,
  and updated task node all started. The task node published the new latched
  `/grasp/near_field_active=false` startup phase and remained `IDLE`.
- Automatic candidate generation is configured `auto_request=false` and was
  not triggered during startup. The WSL protocol-3 GraspNet server reported
  online and loaded at `http://172.23.132.97:8000`.
- In addition to the launch-time enable request, the operator explicitly
  requested immediate joint enable. Published exactly one
  `/demonstration=false`, the documented driver's positive SDK `torque_on`
  branch. At `1785223504.248833` the driver confirmed
  `Disabling zero-torque mode with SDK torque_on frame.` No
  `/demonstration=true`, torque-off, controller stop, `/grasp/stop`, disable,
  trajectory, or gripper command was sent.
- The operator reported that the target was already aligned. Live perception
  consistently identifies it near pixel `(281,218)`, confidence about
  `0.905-0.910`, camera coordinates approximately
  `(0.305,0.026,0.019) m`, and base coordinates approximately
  `(-0.125,-0.435,0.066) m`. These are live observations, not hard-coded
  execution coordinates.
- Full launch restored the checked-in calibration interlock default. Under the
  operator's renewed authorization to complete this same automatic 6D task,
  set `/grasp/calibration_interlock_active=false` and recorded reason
  `USER_OVERRIDE_PRUNED19_TF_TEMPORARY_RENEWED_20260728_AUTOMATIC_EXECUTION_AFTER_NEAR_FIELD_PHASE_FIX`.
  This remains an explicit temporary operator override; the failed independent
  calibration result is not relabelled as a pass and the deployed TF itself
  was not edited. The parameter update sent no physical command.

### First live run after explicit near-field phase fix

- Started continuous remote inference only after the operator reported the
  target aligned. Far-field requests 1, 2, and 3 each used five unique live
  RGB-D source frames with `near_field=false`; their stable tracks reached the
  configured `3/3` hits. As intended before observation motion, every audit
  recorded `execution_position_error_m=0.0` rather than prematurely consuming
  an old endpoint sample.
- Accepted fresh far-field plan `e80b0ff2847359e12c3bb866`, source stamp
  `1785223625702111721`. Its strict MoveIt observation pose was computed at
  `(-0.100,-0.338,0.115) m`; this is a live plan result, not a fixed object
  coordinate. Stopped only the inference stream to freeze that exact plan and
  submitted `/grasp/start` with the exact `plan_id`.
- The 13.616-second strict observation trajectory completed successfully and
  settled after `0.90 s`. Its measured endpoint error was `12.6 mm / 1.97 deg`.
  The fresh Euclidean camera-to-target range was `0.1808542744 m`, below the
  inclusive `[0.190,0.210] m` observation contract, so the task derived its
  one allowed real-time correction.
- Correction audit: radial camera retreat
  `[+0.0002315,+0.0083239,+0.0172400] m`, current endpoint-error feed-forward
  `[-0.0017471,-0.0058193,+0.0110849] m`, final tool translation
  `[-0.0015156,+0.0025046,+0.0283249] m`, norm `28.476 mm`. Strict planning
  and the 3.000-second execution both succeeded; no controller abort or
  feedback-as-command failure hold occurred.
- After settling, the correction endpoint error was `18.1 mm / 3.03 deg` and
  the fresh camera-to-target range was `0.1888 m`. This is only `1.2 mm` below
  the agreed lower bound, but it is still outside the literal contract. The
  current code permits only one range correction, so it correctly failed with
  `OBSERVATION_CAMERA_TARGET_OUT_OF_RANGE` rather than silently advancing.
  The task never published `near_field=true`, never requested a near-field
  contact plan, and never approached, closed, or lifted.
- The arm remains enabled and holding the reached pose. Candidate inference is
  stopped. No stop, torque-off, disable, controller stop, `/grasp/stop`, or
  target-move request was issued.
- Evidence-based correction direction: replace the single-attempt limit with
  a small bounded number of residual observation corrections. Each attempt
  must obtain a fresh post-settle target sample, recompute Euclidean range from
  the live camera TF, use the current plan-bound measured endpoint error, and
  stop immediately once the unchanged inclusive interval succeeds. This is a
  finite residual controller, not an ongoing servo loop; it adds no object
  label branch, saved pose, target coordinate, or fixed trajectory.

### 2026-07-28 00:32 PDT - operator observation contract changed to 18–22 cm

- The operator explicitly changed first-stage observation success to the
  inclusive camera-to-target interval `[0.180,0.220] m`. The nominal
  construction distance remains `0.200 m`; this is still Euclidean
  camera-to-live-target range, not TCP distance.
- Updated the production ROS parameters and both task/remote code fallbacks to
  the same `[0.180,0.220] m` contract. Updated the inclusive boundary/default
  tests and added the exact preceding live value `0.1808542744 m` to the
  accepted regression samples.
- This new operator contract supersedes the immediately preceding proposal to
  add another correction attempt. The first reached view in the recorded run,
  `0.1808542744 m`, now passes without any retreat. Therefore no second
  correction loop was added: current evidence no longer requires it, and
  avoiding an unnecessary motion is the smaller, generalized change.
- The existing one-shot, live-measured retreat remains available only when a
  future fresh view is genuinely below `0.180 m`; a too-far view still cannot
  advance toward the target. No object-specific coordinate, joint value,
  target label branch, stored correction, or fixed trajectory was added.
- While the operator moves the arm, candidate inference remains stopped and no
  task pose was sent. Joint enable remains unchanged; no disable, torque-off,
  controller-stop, `/grasp/stop`, or `/demonstration=true` command was sent.
- Offline verification completed: `test_grasp_task_sequence` passed `129/129`,
  `test_graspnet_input_default_config` passed `6/6`, both changed Python nodes
  passed `py_compile`, and `git diff --check` passed.
- The first attempted live `rosparam` update ran inside a network-isolated
  command environment and returned `Unable to communicate with master`; it was
  explicitly treated as not applied. Repeated through the authorized live ROS
  environment and read back:
  `/grasp/observation_camera_target_min_distance_m=0.18` and
  `/grasp/observation_camera_target_max_distance_m=0.22`.

### GUI slider did not move the arm while operator was repositioning

- Read-only live evidence: `/grasp/state.active=false`,
  `/alicia_d/motion_enabled=true`; `alicia_controller` and `hand_controller`
  are both `running`. The GUI has an established TCPROS connection to the real
  driver on `/joint_commands`.
- `/gui/default_joint_direct_control=false`. A five-second
  `/joint_commands` sample contained no messages while the slider was not
  actively manipulated. Source inspection confirms that in this default
  planning mode a slider only edits the target; it does not publish a direct
  command. The required GUI sequence is `同步当前关节`, adjust a small target,
  `规划当前目标`, then `执行规划目标`.
- The operator was told not to enable GUI direct-control mode because that UI
  mode requests stopping the trajectory controllers. No controller switch,
  task motion, disable, or torque command was sent by this session.

### 2026-07-28 00:45 PDT - 18–22 cm live run reached near field; WSL 2 s evidence lifetime blocked simulation

- After the operator reported the target aligned, hot-reloaded the updated task
  and remote planner nodes and started continuous candidate generation. The
  launch-owned instances did not respawn after the explicit node replacement,
  so the same current-worktree executables were started manually. The full
  launch, real driver, controllers, camera, MoveIt, and GUI remained running.
- Far-field requests 1–3 each consumed five unique live RGB-D frames with
  `near_field=false`; the candidate track reached the required `3/3` stable
  observations. The accepted exact plan was
  `0127c54cf5e8f4ed1327eea7`, with source stamp
  `1785224465122946977`. Candidate generation was stopped before execution so
  `/grasp/start` remained bound to that exact plan.
- The first observation target was computed from the live plan near
  `(-0.115,-0.338,0.114) m`, not read from a saved object-specific position.
  Its retimed `16.869 s` strict trajectory completed successfully and settled
  for `0.90 s`; measured endpoint error was `13.1 mm / 2.14 deg`.
- The first fresh camera-to-target range was `0.1794973519 m`, only
  `0.503 mm` below the inclusive `[0.180,0.220] m` contract. The existing
  generalized one-shot measured correction therefore ran. Its live radial
  retreat was `[+0.0007721,+0.0084353,+0.0186711] m`; live endpoint-error
  feed-forward was `[-0.0040520,-0.0053356,+0.0112275] m`; the final tool
  translation was `[-0.0032799,+0.0030996,+0.0298985] m`, norm
  `30.237 mm`.
- The correction controller returned `GOAL_TOLERANCE_VIOLATED` for Joint2
  error `0.037808 rad`. The task did not infer success from that result: it
  held the controller's reported desired endpoint, settled, and used a new
  range sample as the unchanged physical acceptance criterion. The resulting
  endpoint error was `19.1 mm / 3.17 deg`; fresh range was `0.1889 m`, which
  passed `[0.180,0.220] m`.
- Only then did the task publish `/grasp/near_field_active=true`. Near-field
  request 7 supplied five disjoint frames and stable hit `1/2`; request 8
  supplied another five disjoint frames and stable hit `2/2`. Four candidates
  passed the full-sequence MoveIt reachability check. The accepted exact
  near-field plan was `05b63059df85c0120dc9a8ec`, snapshot stamp
  `1785224574.198447`. A new range check measured `0.1967 m`, also inside the
  observation contract, and the task rebound that exact plan before the
  simulation gate.
- Before any contact, pregrasp, approach, gripper-close, or lift motion, the
  MuJoCo gate rejected the request with
  `PLAN_STALE: snapshot_stamp_sec is stale or from the future
  (age=26.478621s)`. The task failed closed, reset near field to false, released
  its execution slot, and did not perform any second-stage physical motion.
  Audit evidence is in `~/.ros/grasp6d_mujoco_audit_latest.json`, attempt start
  `1785224598.6420696`, completion `1785224598.6661701`.
- The remote candidate stream was explicitly turned off after task failure;
  this was only an inference request and not an arm-stop command. The arm
  remains enabled and holds the reached observation pose. No
  `/demonstration=true`, torque-off, disable, controller-stop, `/grasp/stop`,
  or autonomous protection-stop command was sent.
- Read-only WSL health at `http://172.23.132.97:8000/health` now gives exact
  runtime evidence: `digital_twin.max_snapshot_age_sec=2.0`. The current
  repository launcher passes
  `--max-snapshot-age-sec "${MUJOCO_MAX_SNAPSHOT_AGE_SEC:-120.0}"`, and the ROS
  planning evidence lifetime is also `120.0 s`. Thus a near-field plan needing
  about `26.48 s` for frame collection and generalized reachability validation
  is guaranteed to be rejected by the live WSL's two-second setting.
- Diagnosis: the current failure is a deployed WSL runtime-contract mismatch,
  not observation geometry, target alignment, contact depth, or the new
  18–22 cm range. The correct repair is to restart the WSL service with the
  repository's existing `120.0 s` snapshot-evidence lifetime and verify it
  through `/health`. The snapshot stamp must not be fabricated or refreshed
  without new sensor evidence, and candidate filtering must not be weakened
  merely to race a two-second server limit.
- The ROS host has no `wsl.exe` entry and the combined service exposes no
  authenticated runtime-reload endpoint, so this process-only restart cannot
  be performed from the ROS terminal. The exact next operator action is to
  interrupt only the running WSL HTTP server and, in
  `/home/lv/grasp6d_ws/Robot-Controller-v3`, run:
  `bash tools/start_mujoco_digital_twin_wsl.sh
  --max-snapshot-age-sec 120.0`.
  Passing the CLI value explicitly avoids relying on the WSL copy's older
  default. ROS, robot power, controller state, and joint enable must remain
  unchanged. Physical execution will resume only after `/health` reads back
  exactly `digital_twin.max_snapshot_age_sec=120.0`.

### 2026-07-28 - WSL 120-second server verified; operator powered arm off

- The operator reported the WSL service restarted with the explicit
  120-second argument and then reported that real-arm power was intentionally
  turned off until a later notice.
- Read-only `/health` verification succeeded:
  `digital_twin.max_snapshot_age_sec=120.0`, protocol version `3`, MuJoCo
  `3.2.3`, `lift_evidence_contract_version=1`,
  `strict_dynamic_object_lift_required=true`, and no missing dependency.
  `grasp_backend.loaded=false` is the normal post-restart, before-first-request
  lazy-load state; no candidate request was sent while the arm is powered off.
- The real driver subsequently reported stale hardware feedback and paused
  its SDK command stream. This matches the operator's declared power-off
  state. Candidate generation remains stopped and the task has no authorized
  contact plan.
- No joint-enable, trajectory, gripper, `/grasp/start`, `/grasp/stop`,
  `/demonstration=true`, torque-off, controller-stop, or disable command was
  issued. ROS nodes and terminal supervision remain online while waiting for
  the operator's explicit power-on notification.

### 2026-07-28 - power restored and fresh-state execution resumed

- The operator explicitly reported real-arm power restored. Published exactly
  one `/demonstration=false` enable request; the real driver confirmed
  `Disabling zero-torque mode with SDK torque_on frame` at
  `1785225312.438642`. No `/demonstration=true` or torque-off command was sent.
- Read-only confirmation after enable:
  `/alicia_d/motion_enabled=True`; current real joint feedback is approximately
  `[-110.3,+24.7,-2.7,-5.6,-25.4,+6.6] deg`, with gripper open at
  `0.04975 m`; recent hardware status is `0x00` and measured temperatures are
  approximately `39-40 C`.
- Power removal changed the joint configuration, so every pre-power-off pose,
  target transform, candidate, and plan ID remains invalid. Live perception
  still detects the target. The next generation must use the recovered joint
  feedback and new synchronized RGB-D evidence only.
- The previously authorized calibration interlock override remains active in
  the live task (`/grasp/calibration_interlock_active=false`). WSL continues
  to report the corrected `120.0 s` simulation evidence lifetime.

### 2026-07-28 - fresh-plan handoff, successful observation, and exact MuJoCo contact failure

- The first newly generated far-field plan after power recovery was
  `ce2f4500a6551e94594bc847`. It was not submitted promptly enough while its
  terminal evidence was being reviewed, so `/grasp/start` rejected it at the
  entry boundary with `PLAN_STALE`. No arm motion occurred. The age contract
  was not relaxed and the source timestamp was not rewritten.
- Repeated fresh generation with an automatic handoff that stopped inference
  and submitted the first new stable plan ID immediately. This bound
  `/grasp/start` to far-field plan `3d97f9717fcdff8fa72e783c`, source stamp
  `1785225714110505104`, without a human/log-review delay.
- The first observation trajectory completed and settled. Its measured
  endpoint error was `12.8 mm / 2.10 deg`. Fresh Euclidean
  camera-to-target range was `0.1789968069 m`, only `1.003 mm` below the
  inclusive `[0.180,0.220] m` contract, so the existing single generalized
  correction was derived from current measurements:
  radial retreat `[+0.00067484,+0.00935251,+0.01879387] m`, endpoint-error
  feed-forward `[-0.00055144,-0.00654512,+0.01096073] m`, final command
  `[+0.00012339,+0.00280738,+0.02975460] m`, norm `29.887 mm`.
- The correction controller reported failure. The task did not infer success
  from the command result: it held the reported desired endpoint, settled,
  and evaluated a new physical measurement. Endpoint error was then
  `18.7 mm / 3.05 deg`; fresh range was `0.1906 m`, which passed the unchanged
  observation interval. A second fresh check before simulation measured
  `0.1985 m` and also passed.
- Only after those measurements passed did the task enter near field and bind
  exact plan `f8a0f1a41f7d6e3eff50401a`. Its preview age was `20.117 s`.
  The WSL server accepted the evidence under the corrected `120.0 s`
  lifetime, proving the earlier two-second runtime mismatch was fixed.
- Before any near-field pregrasp, approach, grasp, close, or lift command,
  MuJoCo rejected this candidate with `MUJOCO_CONTACT_FAILED`. The task failed
  closed, released the execution slot, reset near field, and the candidate
  stream was explicitly stopped. The only physical motions in this run were
  the far-field observation trajectory and its measured range correction.
- Exact immutable audit evidence is in
  `~/.ros/grasp6d_mujoco_audit_latest.json`: attempt
  `1785225814.951714` through `1785225822.6966238`; payload schema `3`;
  candidate source `tabletop_geometry`; required open width
  `0.0377666391 m`; `ik_success=true`; `collision_free=true`.
  Initial two-sided contact existed at a `0.0343 m` gap and settled under
  preload at `0.0320 m`. Actuator forces were about
  `-3.207/+3.193 N`, and object-normal contact forces were
  `3.202/3.197 N`.
- The prescribed Cartesian lift solved all `579` IK samples and was resampled
  to `1167` samples at the `0.08 rad/s` joint-speed limit. Tool lift reached
  `0.04666 m`, but object lift was only `5.497e-05 m` (about `0.055 mm`),
  below the required `0.015 m`. Two-sided contact remained for only
  `97/1167` lift samples; first loss occurred at sample `97`
  (`alpha=0.08319`), followed by a `1070`-sample loss streak. The right
  contact-normal/jaw cosine fell to about `0.4915`; simulation score was `55`.
- This evidence rules out snapshot age, IK, collision, and observation-range
  gating as the current failure. It does not justify weakening the contact
  grace, required object lift, or response score. The evidence-supported
  generalized repair is simulation-aware selection among the already
  hard-gated and full-sequence MoveIt-reachable near-field candidates:
  deterministically evaluate them in the same final-score order, promote only
  the first candidate that passes the unchanged MuJoCo contract, and fail
  closed if none pass or the simulation authority is unavailable. This adds
  no object label branch, saved pose, fixed target coordinate, or
  object-specific trajectory.

### 2026-07-28 - latest operator power-on and enable restoration

- After the operator explicitly reported the arm powered on again, published
  exactly one `/demonstration=false` request to restore joint enable. The
  publisher completed normally. No `/demonstration=true`, torque-off,
  disable, controller-stop, `/grasp/stop`, or trajectory command was sent.
- The real driver remains online and continues reporting joint feedback and
  keepalive/holding behavior. Current raw telemetry repeatedly reports
  hardware status `0xE1`, commonly with a maximum reported channel value near
  `55 C` and occasional isolated implausible channel spikes. The driver
  explicitly logs that it treats this as a status event and sends no
  `torque_off`; motion enable is unchanged. No physical overheating cause is
  inferred from these inconsistent raw samples.
- No new physical grasp attempt will be started until the simulation-aware
  reachable-candidate selection change is implemented, verified offline, and
  hot-reloaded. The arm remains enabled; this implementation work sends no
  arm-motion command.

### 2026-07-28 - simulation-aware near-field candidate selection implemented

- The operator then explicitly powered the arm off again. From that notice
  onward no enable, trajectory, gripper, controller-switch, or other arm
  command was sent. ROS/WSL supervision and offline implementation continued.
- Refactored selected-plan construction into a side-effect-free bundle builder.
  A candidate can now be serialized into the exact immutable rich-plan form
  used for publication and MuJoCo without publishing it or granting motion
  authority. The normal publish transaction still rebuilds and commits the
  same deterministic plan after selection.
- Added near-field-only MuJoCo screening after all existing analytical gates,
  stability checks, and full `pregrasp/approach/grasp/lift` MoveIt checks.
  Reachable candidates are sorted by the existing generalized
  `(final_score, track_id, variant_index)` order. No target label, object
  coordinate, saved joint pose, or object-specific correction participates in
  the selection.
- An explicit candidate-mechanics failure
  (`MUJOCO_IK_FAILED`, `MUJOCO_COLLISION`,
  `MUJOCO_CONTACT_FAILED`, `MUJOCO_LIFT_FAILED`, or
  `MUJOCO_SCORE_BELOW_THRESHOLD`) rejects only that exact candidate and
  advances to the next reachable candidate. A network, strict-JSON,
  plan-ID, provenance, stale-evidence, internal-server, or other authority
  failure stops screening immediately and leaves no valid preview.
- The planner uses the current `/joint_states` message and the same
  `build_mujoco_payload` plus `validate_mujoco_gate_response` contract as the
  execution node. Screening is bounded by the full MoveIt shortlist maximum
  of `12` candidates and an `80.0 s` request-local computation budget. These
  are target-independent resource/evidence limits, not grasp-depth or
  object-geometry gates. The task node retains its independent pre-motion
  MuJoCo revalidation of the final selected plan.
- Every attempted candidate records plan ID, source lineage, live source
  timestamp, final score, response hash, explicit safety booleans, failure
  code/reason, returned score, and duration in that request's existing
  planning audit row. The funnel separately records `mujoco_screened` and
  `simulation_selected`; failed candidates are not silently discarded.
- Added regressions proving (1) a first candidate with
  `MUJOCO_CONTACT_FAILED` is rejected and the next passing reachable candidate
  is selected, and (2) `PLAN_ID_MISMATCH` immediately fails closed without
  trying another candidate.
- Offline verification after the change:
  `test_remote_grasp6d_node` passed `156/156`;
  `test_remote_grasp6d_streaming` passed `197/197`;
  `test_grasp_task_sequence` passed `129/129`;
  `test_mujoco_digital_twin_client` passed `2/2`;
  `test_graspnet_input_default_config` passed `6/6`;
  both changed test/production files passed `py_compile`; and
  `git diff --check` passed.
- The first attempt to invoke the streaming tests used a missing shell
  `pytest` executable and ran no tests. Running the installed module as
  `python3 -m pytest` executed the full suite successfully; the missing
  console entry point was not treated as product evidence.

### 2026-07-28 - post-implementation live run and near-field input diagnosis boundary

- After the operator explicitly reported power restored, published exactly
  one `/demonstration=false` enable request. The real driver confirmed at ROS
  time `1785227374.527548`:
  `Disabling zero-torque mode with SDK torque_on frame.` No
  `/demonstration=true`, torque-off, disable, controller-stop,
  `/grasp/stop`, or equivalent stop-enable command was sent.
- Continuous inference produced a fresh far-field plan
  `3e555fddc609f1d12b871c63` with source stamp
  `1785227605264899730`. The handoff stopped candidate generation as soon as
  that exact new stable plan was available and submitted only that immutable
  plan ID. No earlier plan was reused.
- The far-field observation trajectory completed and settled with a measured
  endpoint error of `0.0131 m / 2.17 deg`. The first fresh
  camera-to-target distance was `0.1790157707 m`, about `0.984 mm` below the
  inclusive `[0.180,0.220] m` contract. The one allowed generalized
  measurement-derived correction was:
  radial `[-0.0001491830,+0.0088520170,+0.0190251786] m`,
  endpoint feed-forward
  `[-0.0037418292,-0.0053395195,+0.0113857290] m`, and combined command
  `[-0.0038910123,+0.0035124975,+0.0304109076] m`, norm
  `0.0308593732 m`.
- The controller reported that correction as failed. The task did not assume
  success: it held the controller-reported desired endpoint, settled, and
  measured again. Post-correction endpoint error was
  `0.0183 m / 3.05 deg`; fresh camera-to-target range was `0.1916 m`, which
  passed the unchanged observation contract.
- Near-field planning then failed before stability selection, MoveIt
  sequence validation, MuJoCo screening, or any near-field arm motion.
  Successive observed requests returned:
  `28/0` raw/locally-valid candidates
  (`25 CANDIDATE_CONTRACT_INVALID`,
  `3 GRIPPER_CONTACT_PATCH_MISS`,
  `1 GRIPPER_SWEEP_COLLISION`);
  `27/0`
  (`23 CANDIDATE_CONTRACT_INVALID`,
  `4 GRIPPER_CONTACT_PATCH_MISS`,
  `1 GRIPPER_WIDTH_TOO_NARROW`);
  `24/0`
  (`20 CANDIDATE_CONTRACT_INVALID`,
  `3 GRIPPER_CONTACT_PATCH_MISS`,
  `2 GRIPPER_WIDTH_TOO_NARROW`); and
  `17/0`
  (`15 CANDIDATE_CONTRACT_INVALID`,
  `3 GRIPPER_CONTACT_PATCH_MISS`).
  Counts can overlap because one candidate can contribute more than one
  analytical rejection observation.
- At ROS time `1785227811.726481`, the task failed closed with
  `NEAR_FIELD_REPLAN_TIMEOUT` after `150.0 s`; its last state reason was
  `NEAR_FIELD_PLAN_PHASE_INVALID: Preview is not a contact execution plan`.
  It reset planning phase to far field and released the execution slot.
  There was no near-field pregrasp, approach, grasp, gripper-close, lift, or
  contact motion in this run. The handoff returned `success=false`.
- The per-request code already captured exact GraspNet exception samples, but
  only tabletop-source samples were emitted to the terminal; the latest JSON
  audit was subsequently overwritten by normal far-field streaming. Because
  that exact exception text is no longer available, no subcause is guessed
  from the generic `CANDIDATE_CONTRACT_INVALID` code.
- Added terminal emission of the existing exact GraspNet rejection samples:
  `source_index`, `variant_index`, failure code, exception type, and exact
  reason. This is observability only; it does not relax a gate, alter a
  score, change a pose, or authorize motion. The diagnostic planner was
  hot-reloaded with candidate streaming stopped. `py_compile`,
  `git diff --check`, and the two targeted streaming regressions passed.
  The first subsequent full-suite command omitted the worktree ROS
  environment and stopped during collection with
  `ModuleNotFoundError: alicia_flexible_grasp_supervisor`; no test ran in
  that invocation. After sourcing this worktree's `devel/setup.bash`, the
  complete `test_remote_grasp6d_streaming.py` suite passed `197/197` in
  `13.17 s`.
  A fresh planning-only near-field request is required to obtain the missing
  evidence before changing candidate normalization.

### 2026-07-28 - operator moved arm after the failed run

- The operator reported manually moving the arm. Live driver evidence shows a
  changed joint configuration near
  `[-108.5,32.6,-5.1,-3.1,-29.4,-0.7] deg`, hardware status `0x00`, and a
  visible target around `0.322 m` camera depth. These values document the new
  state only; they are not treated as an alignment confirmation or a grasp
  pose.
- This manual move invalidates every previously captured target transform,
  RGB-D snapshot, candidate, preview, plan ID, and observation endpoint.
  Candidate streaming remains stopped and the task remains idle. No motion,
  gripper, enable, disable, stop, or controller-switch command was sent in
  response.
- Once the operator explicitly confirms that the camera is aligned with the
  intended target, the next evidence step is a planning-only near-field
  request using new synchronized RGB-D and joint feedback. It will not start
  a grasp or move the arm. The new exact GraspNet rejection samples will
  determine whether a code change is justified; until then the generic
  contract failure is not interpreted.

### 2026-07-28 - planning-only exact rejection evidence after alignment

- After the operator explicitly confirmed alignment, published only the
  planning-phase signal `/grasp/near_field_active=true` and called only
  `/grasp_6d/request_plan {trigger:true}`. `/grasp/start`, trajectory,
  gripper, controller-switch, stop, disable, torque-off, and demonstration
  commands were not called.
- The first request used five unique current RGB-D frames and remained
  `STABILITY_PENDING`. The next completed request established stable
  tabletop candidates and reached the new simulation-aware selection:
  `40` candidates entered hard recheck, `32` passed, `24` entered the
  full-sequence MoveIt shortlist, `8` were reachable, and none passed MuJoCo.
- The newly emitted exact GraspNet rejection samples resolved the prior
  generic code. The repeated exception is:
  `ValueError: insertion tilt has no stage profile inside hard bounds`.
  Representative requests rejected `16-17` GraspNet candidates for that
  exact reason. This is now a measured input-contract failure; no other
  GraspNet subcause is inferred.
- All eight reachable tabletop selection attempts failed strictly with
  `MUJOCO_CONTACT_FAILED`. Four plans formed initial two-sided contact but
  lost it for most lift samples and achieved about `0.000 m` object lift.
  Four attempts ended closure without simultaneous two-sided contact. No
  candidate was promoted and no motion authority was granted.
- Stopped only continuous inference with
  `/grasp_6d/request_plan {trigger:false}` and restored the planning-phase
  signal to `false`. Both commands affect candidate computation only; they
  are not `/grasp/stop` or a robot stop/disable command.
- The frozen request-5 audit has `32` total inputs:
  `12` GraspNet, all locally invalid for the exact tilt/profile reason, and
  `20` tabletop candidates, of which `8` were MoveIt reachable and all `8`
  failed MuJoCo. Its funnel reports
  `CANDIDATE_CONTRACT_INVALID=12`,
  `MOVEIT_UNREACHABLE=12`, and `MUJOCO_CONTACT_FAILED=8`.
  The audit also shows repeated simulation of only four unique plan IDs,
  because several reachable tracked rows serialize to the same immutable
  trajectory. This is deterministic duplicate work and does not constitute
  additional independent contact evidence.

### 2026-07-28 - complete ROS stack relaunched and joint enable restored

- The operator requested the latest complete ROS stack and reported WSL
  online. All previously supervised terminal sessions had ended; the first
  explicit `/demonstration=false` attempt could not reach a ROS master and
  therefore changed no state.
- Started the checked-in unified launch:
  `full_system.launch`, real driver on `/dev/alicia_arm` at `1000000`,
  `auto_torque_on_startup=true`, real camera, GUI, MoveIt, motion gateway,
  task node, remote GraspNet/MuJoCo planner, and safety/data nodes.
  `start_tactile=false`, consistent with the current task's explicit
  no-electronic-skin policy.
- The real serial port opened, real feedback resumed near
  `[-108.4,32.6,-5.1,-3.2,-29.4,-0.6] deg`, both trajectory controllers
  started, the camera published real RGB-D, the target remained visible near
  `0.322 m`, MoveIt reported ready, and the remote server reported
  `backend=graspnet_baseline loaded=true protocol=3`.
- Published exactly one explicit `/demonstration=false` request after the
  master was online. The driver confirmed at ROS time
  `1785230240.009602`:
  `Disabling zero-torque mode with SDK torque_on frame.` Subsequent feedback
  is predominantly `status=0x00`, with measured temperatures about
  `38 C`. Intermittent single-frame `0xE1` events are logged as status only;
  the driver explicitly sends no torque-off.
- No `/demonstration=true`, torque-off, disable, controller stop,
  `/grasp/stop`, trajectory, gripper, or grasp-start command was sent during
  the relaunch and diagnosis. Candidate generation remains stopped while the
  exact candidate-profile and contact-mechanics evidence is resolved.

### 2026-07-28 - hardware-off offline GraspNet stage-profile repair

- The operator reported all hardware serial ports closed and arm power off,
  with only WSL available, and explicitly limited work to evidence-backed
  offline processing. From that notice onward no ROS master, serial port,
  driver, enable/disable, controller, trajectory, gripper, candidate-stream,
  grasp-start, or stop command was issued. No real image, TF, joint state, or
  remote WSL response was synthesized or treated as current evidence.
- The only candidate-normalization change addresses the exact previously
  observed exception
  `insertion tilt has no stage profile inside hard bounds`. GraspNet remains
  authoritative for contact centre, learned jaw direction, model width,
  score, and insertion depth. If its insertion direction is outside the
  maximum tilt produced by the current geometry/measurement-derived adaptive
  stage family, the planner computes the closest feasible insertion direction
  in the plane perpendicular to the learned jaw. It then rebuilds tool0 from
  the unchanged centre plus unchanged GraspNet depth. No target label, saved
  pose, object coordinate, fixed grasp angle, mass, friction, or
  object-specific correction is used.
- Projection is fail-closed when the live support normal is invalid, the
  adaptive profile family is empty, the learned jaw cannot admit a downward
  insertion direction, or the jaw geometry's minimum possible tilt is still
  outside the live stage bound. These cases report the explicit
  `GRASPNET_STAGE_PROFILE_UNAVAILABLE` code. Every projected candidate still
  has to pass the unchanged CAD/contact-patch gates, exact stage construction,
  full-sequence MoveIt validation, and MuJoCo contact/lift gate before it can
  become executable.
- Added bounded per-candidate projection audit fields for raw/projected tilt,
  live maximum tilt, minimum feasible tilt, signed offsets, correction angle,
  and jaw/support cosine. Expanded the existing strict MuJoCo response audit
  to preserve its returned diagnosis, bounded IK rows, complete lift evidence,
  and joint-state-source field. This preserves the exact contact-loss state
  for the next diagnosis instead of inferring it from a summary code.
- Removed a latent undefined fallback name in normalized-candidate audit
  construction. The projection helper always installs its own plain-dict
  audit before normalization, so this cleanup changes no pose or selection
  rule and avoids a possible audit-only `NameError`.
- Added offline geometric regressions proving that a learned `60 deg`
  insertion is minimally projected to a live `30 deg` boundary while
  preserving centre, depth, width, score, and tool0-to-centre distance; that
  a jaw parallel to the support normal fails closed; that an already
  admissible candidate remains unprojected; and that copied MuJoCo lift
  evidence is not altered if the original response object later changes.
  These constructed unit geometries validate the mathematics only and are not
  claimed as real-camera or real-arm evidence.
- One earlier adaptive-profile test invocation failed `1/14` only because the
  improved exception text no longer contained the test's stable phrase
  `hard bounds`; it did not expose a numerical or planning failure. The
  message was made both quantitative and contract-compatible:
  `insertion tilt <value> deg exceeds live hard bounds <value> deg`.
- Final offline verification in the hardware-off boundary:
  `test_adaptive_stage_profiles.py` passed `14/14`;
  the three new targeted streaming tests passed `3/3`;
  `test_remote_grasp6d_streaming.py` passed `200/200`;
  `test_remote_grasp6d_node.py` passed `156/156`;
  all changed Python files passed `py_compile`; and
  `git diff --check` passed.
- This does not establish that a live candidate passes contact or lift.
  Required next evidence, after the operator restores hardware and explicitly
  resumes, is a new synchronized RGB-D/TF/joint snapshot and a fresh
  planning-only near-field request. The audit must show whether projected
  GraspNet candidates pass analytical and full-sequence MoveIt checks and
  whether at least one exact fresh plan passes MuJoCo. Physical near-field
  motion remains outside the present offline evidence boundary.

### 2026-07-28 - exact frozen-audit projection-layer replay

- Performed a read-only offline replay against the existing frozen planning
  audit for historical request `5`, snapshot
  `1785228830.6978016`. This is historical evidence only; it is not presented
  as a current camera frame or current arm state. The audit records
  `12` GraspNet source candidates as `24` explicit orientation variants,
  the frozen base/camera transform, support normal, and the exact adaptive
  stage family generated for that request. Its recorded maximum stage tilt is
  `45.0 deg`.
- Reconstructed only those recorded candidate poses and ran the new
  orientation-projection layer with the recorded transform, support normal,
  depth, score, width, and recorded stage family. Of the `24` variants,
  `12` were minimally projectable to the live historical boundary; their
  projected tilts were `44.999999999999986` to
  `45.00000000000001 deg`. Maximum contact-centre residual was exactly
  `0.0 m`, maximum insertion-depth residual was
  `4.5102810375396984e-17 m`, and the minimum absolute dot product between
  original and projected jaw axes was `0.9999999999999998`. These are
  floating-point reconstruction residuals, not claimed sensor accuracy.
- The other `12` variants failed closed with
  `GRASPNET_STAGE_PROFILE_UNAVAILABLE`. Their learned jaw directions impose
  minimum possible insertion tilts above the recorded `45.0 deg` stage bound:
  source indices `1` and `9` require `61.686803 deg`; indices `4` and `5`
  require `52.973785 deg`; index `6` requires `46.685129 deg`; and index `7`
  requires `50.934324 deg`. Both recorded variants of each source have the
  same infeasibility. The replay therefore distinguishes a correctable
  insertion-direction excess from a jaw-geometry incompatibility instead of
  silently rotating the learned grasp into an unrelated pose.
- The frozen audit contains hashes and summary counts for the input depth and
  mask, but not the target point-cloud array required for the exact CAD/contact
  gate. The separate MuJoCo audit contains its request/response summary rather
  than the complete reconstructable planner and physics inputs. Consequently,
  this replay cannot establish CAD validity, MoveIt reachability, contact, or
  lift for the `12` projectable variants. No missing points, joints, TF, or
  physics fields were guessed.
- In particular, this evidence does not justify rotating the learned jaw,
  generating additional interior tilt variants, weakening contact-patch
  requirements, or changing mass/friction/force. Those decisions require a
  fresh synchronized request showing the next exact rejection stage. Offline
  work is now complete at the available evidence boundary.

### 2026-07-28 - full stack resumed, real feedback recovered, and joints enabled

- After the operator explicitly reported the hardware serial ports, arm
  power, and WSL online, the first direct `roslaunch` invocation exited before
  resolving the package because that shell had not sourced this worktree.
  It opened no ROS master or hardware connection. Sourced
  `devel/setup.bash` and immediately launched the logged unified stack with
  the real `/dev/alicia_arm` driver at `1000000`, automatic torque-on, real
  RGB-D camera, GUI, MoveIt, motion gateway, task node, current remote
  GraspNet/MuJoCo planner, and `start_tactile=false`.
- The serial port opened and both controllers started, but the driver initially
  had no valid encoder frame. The first explicit `/demonstration=false`
  request was therefore rejected at ROS time `1785288891.071104` with
  `real hardware feedback is unavailable or stale`; that request did not
  change the enable state. No opposite demonstration value, torque-off,
  controller stop, `/grasp/stop`, or other stop/disable command was sent.
- At `1785288915.300390` the driver rejected one all-zero encoder frame rather
  than publishing it. Valid real feedback began at
  `1785288916.400307`, then converged to stable hardware frames with
  `status=0x00`, gripper raw near `995`, and measured temperature
  `29-31 C`. Intermittent isolated `E1`/`E2` status frames were explicitly
  logged without torque-off because measured temperature was not over limit.
- Reissued exactly one `/demonstration=false` request after valid feedback was
  available. The driver confirmed at `1785288950.771394`:
  `Disabling zero-torque mode with SDK torque_on frame.` Subsequent GUI joint
  commands produced changing commanded and measured joint positions, proving
  that the real control path is active.
- Real perception reacquired the target while the operator was moving the arm.
  Observations progressed from about `uv=(559,167), depth=0.246 m` to
  `uv=(298,261), depth=0.305 m`. These moving observations are recorded only
  as evidence that camera/perception is live; they are not accepted as a
  completed alignment or planning snapshot.
- No 6D candidate request, calibration-interlock override, grasp start,
  trajectory issued by the grasp task, or gripper-close command has been sent.
  The next boundary is the operator's explicit `已对准` confirmation, followed
  by a fresh synchronized candidate request using the new code and current
  real evidence.

### 2026-07-28 - first live generalized-candidate evidence after alignment

- The operator explicitly reported `已对准`. At that boundary the measured
  arm feedback was stable near
  `[-107.8, 22.4, -15.6, 0.3, 6.0, 0.6] deg`, gripper raw near `995`.
  Perception repeatedly reported the carton near `uv=(296,261)`, depth
  `0.347-0.351 m`, base position approximately
  `(-0.138,-0.482,0.064) m`, and confidence about `0.905-0.907`.
  These are live measurements, not substituted or object-specific constants.
- Renewed the previously operator-authorized temporary calibration-interlock
  bypass by setting `/grasp/calibration_interlock_active=false` and reason
  `USER_OVERRIDE_TEMPORARY_RENEWED_20260728_FRESH_GENERALIZED_GRASP_AFTER_ALIGNMENT`.
  This is recorded as an operator override and is not a claim that calibration
  passed.
- Started one fresh far-field candidate stream with minimum accepted plan
  timestamp `1785289129725061178 ns`. The execution helper was restricted to
  a newly published valid four-stage `Preview`: it would stop candidate
  computation, promote that exact cached preview, verify the same plan ID
  through `/grasp/current_plan`, and only then call `/grasp/start`. It rejected
  all older plans and did not pre-authorize any motion.
- Request `1` completed from `5` unique source frames with `40` remote inputs,
  `21` locally valid candidates, and far-field status `STABILITY_PENDING`.
  Its best stability track reached `1/3`; no Preview or executable plan was
  published. Request `2` also completed from `5` unique frames with `43`
  remote inputs and `21` locally valid candidates. Its best track reached
  `2/3` across request IDs `[1,2]`; it likewise remained
  `STABILITY_PENDING`.
- This is the first live evidence that the new generalized insertion
  projection passed some GraspNet candidates beyond the earlier stage-profile
  exception and into the unchanged contact geometry audit. Those candidates
  reported the next exact rejection
  `GRIPPER_CONTACT_PATCH_MISS` because the live target cloud had no common
  bilateral contact height. Other learned jaw geometries correctly failed
  closed with `GRASPNET_STAGE_PROFILE_UNAVAILABLE`; recorded minimum required
  tilts included `49.902339`, `51.193510`, `51.758851`,
  `78.129113`, `78.246800`, and `78.420314 deg`, all above the
  live `45 deg` stage maximum. This preserves GraspNet's learned jaw instead
  of rotating it into a target-specific pose.
- Locally generated tabletop candidates passed generation but some were
  rejected by the unchanged `GRIPPER_SWEEP_COLLISION` audit at their measured
  high tilt/palm clearance. The far-field contact-overlap decision explicitly
  remained `deferred_to_near_field_contact_plan`; no near-field contact gate
  was incorrectly required before the observation motion.
- A third request did not publish completion evidence before the helper's
  `120 s` deadline. The terminal after request `2` continued to show live
  joint feedback and target perception but no third request metrics, so no
  cause is asserted without data. On timeout the helper called only
  `/grasp_6d/request_plan trigger:false`. The stream became
  `continuous remote 6D inference stopped`; no `/grasp/stop`, torque-off,
  disable, controller stop, trajectory, gripper command, plan promotion, or
  grasp start was issued.
- Source inspection confirms that both candidate-stream start and stop reset
  the stability tracker and advance its epoch. Therefore the former `2/3`
  evidence cannot be carried into a restarted stream; the next request must
  collect a new independent `3`-hit stability chain. No stability threshold,
  geometry gate, or target-specific value will be weakened to reuse it.

### 2026-07-28 - fresh 3-hit window reached Preview; extra 30 s runner limit diagnosed

- Started a completely new post-alignment evidence window at
  `1785289488243682146 ns`; no stability state from requests `1-2` was reused.
  Request `3` used `5` unique source frames, entered `40` local candidates,
  retained `21`, and reached `1/3`. Request `4` used `5` unique source
  frames, entered `42`, retained `21`, and reached `2/3`. Their exact
  stability window was `[3,4]`.
- Request `5` completed the required independent chain `[3,4,5]` at `3/3`.
  It retained `21` local candidates; `24` were submitted to full MoveIt
  checks, `8` were reachable, and one four-stage Preview was selected.
  The request remained planning-only throughout. Its exact primary failure
  distribution also showed `16` `MOVEIT_UNREACHABLE`, `2` `COLLISION`,
  `15` `GRASPNET_STAGE_PROFILE_UNAVAILABLE`, `11`
  `GRIPPER_CONTACT_PATCH_MISS`, and `5` `GRIPPER_SWEEP_COLLISION`.
- Request `6` independently kept the stable chain at `4` hits over
  `[3,4,5,6]`. Of `24` MoveIt-checked candidates, `7` were reachable and one
  four-stage Preview was selected. Its exact source snapshot was
  `1785289658.2200477`; Preview completion reported result age
  `34185.006856918335 ms`.
- The temporary execution runner incorrectly imposed an additional
  hard-coded `30 s` source-age acceptance limit. It therefore rejected
  request `5` at `50146.230936050415 ms` and request `6` at
  `34185.006856918335 ms`, even though the running ROS contract is
  `/grasp_6d/plan_validity_sec=120.0`. Source inspection confirms both the
  remote promotion authority and `grasp_task_node` read that same runtime
  parameter and revalidate source age; there is no separate
  `/grasp/plan_validity_sec` override. The 30-second helper check was thus an
  accidental stricter gate, not the configured 120-second WSL/ROS contract.
- The `240 s` runner deadline then expired. It stopped candidate computation
  only; pending request `7` was explicitly terminalized as
  `GENERATION_STALE`. No plan was promoted and no `/grasp/start`, trajectory,
  gripper, arm-stop, disable, torque-off, or controller-stop command was
  issued.
- Corrective action is to remove the runner's independent fixed age and read
  `/grasp_6d/plan_validity_sec` dynamically. A new window must again collect
  `3` independent hits because stopping the old stream reset its tracker.
  The accepted Preview must still be new relative to that window, valid,
  four-stage, within the runtime plan-validity contract, cached against the
  current geometry generation, promoted under the exact same plan ID, and
  revalidated by `grasp_task_node`. Near-field motion continues to require a
  new real-image replan; none of these corrections weakens geometry, MoveIt,
  MuJoCo, stability, or contact gates.

### 2026-07-28 19:00 PDT - slow far-field entry and Preview/audit race diagnosed

- After replacing the runner's accidental fixed `30 s` limit with the live
  `/grasp_6d/plan_validity_sec=120.0`, a fresh stream produced request `8`
  at `1/3`, request `9` at `2/3`, and request `10` produced valid four-stage
  Preview `plan_id=31647f5971d469680623ed95`. The runner received that
  Preview at source age `36.671350 s`, within the live 120-second contract.
- The runner then stopped candidate generation immediately from its Preview
  callback and called `/grasp_6d/replan_execution`. Promotion failed with
  `cached Preview audit does not match plan_id`. Exact audit evidence explains
  the mismatch: stopping the stream advanced its generation while request
  `10` was still committing its audit, so request `10` was terminalized as
  `GENERATION_STALE`. The atomically written audit file therefore still
  described request `9`, outcome `STABILITY_PENDING`, with an empty plan ID.
  This is an orchestration race, not a geometry, MoveIt, target-alignment, or
  WSL-inference failure.
- The generalized repair is to keep the stream alive after receiving Preview,
  wait until execution authority or the atomically committed audit is bound to
  that exact Preview plan ID, then stop candidate generation, re-verify the
  exact plan ID, and only then call `/grasp/start`. Candidate-stream stop is
  never an arm stop. No `/grasp/start`, trajectory, gripper command,
  `/grasp/stop`, controller stop, disable, torque-off, or
  `/demonstration=true` command was issued in this failed promotion attempt.
- The operator correctly observed that older runs entered the first stage much
  faster. The current remote planner is running at `request_hz=0.1`, so a
  failed five-frame stable-snapshot acquisition can defer the next attempt by
  another 10 seconds. Live source snapshots show request `8 -> 9` separated
  by about `104.19 s` and request `9 -> 10` by about `98.25 s`, while WSL
  inference itself completed in approximately `0.2..0.45 s`. The delay is
  therefore not supported as a WSL compute delay. The production manual
  specifies first live acceptance at `request_hz=1.5`; the checked-in
  `0.1 Hz` value came from an older July 24 run whose server was then thought
  to need `6..8 s` and showed stale results. Rate restoration will be based on
  the current synchronized-server evidence and will not alter any candidate,
  geometry, collision, stability, or execution gate.

### 2026-07-28 19:00 PDT - operator-requested WSL combined-server restart

- The operator requested one complete WSL restart command. The current
  documented combined service must be launched from
  `/home/lv/grasp6d_ws/Robot-Controller-v3`, use the existing
  `grasp6d118` environment, CUDA GraspNet baseline/checkpoint, Alicia-D
  50-mm model XML, pass score `80`, minimum simulated lift `0.015 m`, and the
  explicitly required `120.0 s` snapshot-evidence lifetime.
- The command supplied to the operator starts the combined process providing
  `/predict`, `/health`, and `/simulate_grasp`; it does not start a mock or the
  obsolete GraspNet-only server. Restarting this WSL HTTP process sends no
  ROS motion, arm-stop, disable, torque-off, controller-stop, gripper, or
  trajectory command. Candidate generation remains stopped while the WSL
  process is restarted.

### 2026-07-28 - WSL restart verified and first-stage latency repair deployed

- After the operator reported `WSL已启动`, ROS-host `/health` readback confirmed
  `ok=true`, protocol `3`, loaded official `graspnet_baseline`, CUDA available
  with torch `2.4.1+cu118`, and no missing GraspNet dependencies. The same
  process exposes healthy MuJoCo `3.2.3` using the Alicia-D 50-mm gripper XML;
  `digital_twin.max_snapshot_age_sec=120.0`,
  `min_lift_success_m=0.015`, strict dynamic lift evidence, request-bound
  gripper close, 5 N simulated joint-effort limit, opposed-contact geometry,
  and prescribed Cartesian lift contracts all read back from the live WSL
  process. This is the required combined non-mock service.
- Restored checked-in `/grasp_6d/remote/request_hz` from the old diagnostic
  `0.1` value to the documented first-live acceptance value `1.5`. The
  latest-only scheduler still permits only one in-flight request and one
  newest pending snapshot, so this does not create an unbounded request queue.
  It changes no stability hit count, candidate geometry, collision, MoveIt,
  MuJoCo, contact, target-drift, or execution gate.
- Corrected the live execution runner's ordering. A valid Preview no longer
  causes an immediate stream stop. The runner now waits for either exact
  execution authority on `/grasp/current_plan` or an atomically committed
  `/grasp_6d/gate_audit` whose disk payload SHA-256, selected lineage,
  `preview_valid=true`, and `plan_id` all match the Preview. Only a bound audit
  may trigger cached promotion while streaming remains active. Once exact
  execution authority exists, the runner stops candidate computation,
  re-verifies the same plan ID, and only then calls `/grasp/start`. Timeout
  cleanup still stops inference only and never calls `/grasp/stop`.
- Offline verification passed: the runner compiles, configuration tests pass
  `6/6`, and `git diff --check` passes. Live candidate status read
  `continuous remote 6D inference already stopped` before deployment.
- Set the live ROS parameter to `1.5`, terminated only the old
  `/remote_grasp6d_node`, and started the current-worktree replacement. The
  replacement reports the loaded protocol-3 GraspNet service online and is
  connected to the existing RGB-D, mask, object, joint, TF, task, and GUI
  graph. Live parameter readback is exactly `1.5`. The real driver,
  ros_control controllers, task node, camera, perception, GUI, arm enable, and
  joint hold were not restarted or altered. No trajectory, gripper,
  `/grasp/start`, `/grasp/stop`, controller-stop, disable, torque-off, or
  `/demonstration=true` command was issued during this deployment.

### 2026-07-28 - latest full chain restarted; fresh perception latency recovered

- The operator first requested a pause, then explicitly superseded it by
  requesting the latest complete ROS chain, direct joint enable, continued
  diagnosis, and automatic execution with the target already aligned. All
  prior terminal sessions had ended, so there was no live duplicate ROS graph.
- Started the current-worktree `full_system.launch` with the real Alicia-D
  serial driver on `/dev/alicia_arm` at `1000000`, automatic positive
  torque-on, real RGB-D camera, GUI, tactile disabled for this 6D task, and the
  combined WSL service at `http://172.23.132.97:8000`. Driver, hardware
  interface, ros_control controllers, MoveIt, camera, segmentation,
  hand-eye TF, motion gateway, remote 6D planner, task node, safety monitor,
  logger, and GUI all started in one launch.
- The real driver immediately produced stable hardware feedback near
  `[-107.8,+22.4,-15.6,+0.3,+6.0,+0.6] deg`, gripper raw `995`, ordinary
  measured temperatures about `38 C`, and normal status `0x00`. Isolated
  `0xE1` events remained single-frame events; the driver explicitly logged
  that it sent no torque-off without sustained measured over-temperature.
- In addition to launch-time positive torque-on, published exactly one
  `/demonstration=false` at the operator's direct request. The driver confirmed
  `Disabling zero-torque mode with SDK torque_on frame` at
  `1785291478.132886`. No `/demonstration=true`, torque-off, disable,
  controller stop, `/grasp/stop`, trajectory, or gripper command was sent.
- Full launch restored the checked-in calibration interlock default. Under the
  operator's renewed authorization for the same aligned automatic task, set
  `/grasp/calibration_interlock_active=false` and recorded
  `USER_OVERRIDE_TEMPORARY_RENEWED_20260728_ALIGNED_AUTOMATIC_6D_AFTER_FULL_RESTART`.
  This remains a temporary operator override and does not relabel calibration
  evidence as passed or alter the deployed TF.
- Before this full restart, read-only evidence had shown a real degraded input
  state: object-message delay `3.476 s`, mask rate about `0.238 Hz`, RGB about
  `13 Hz`, and depth about `4.4 Hz`. With the unchanged
  `planning_snapshot_max_inference_latency_sec=1.2`, that old process could not
  form a valid five-frame planning snapshot even after the request rate was
  restored to `1.5 Hz`.
- After the complete restart, a new 14-second measurement window showed object
  message delay averaging about `0.720 s` over 16 samples
  (`0.575..1.005 s`) and mask delay averaging about `0.772 s` over 17 samples
  (`0.571..1.602 s`). Most fresh samples again satisfy the unchanged
  `1.2 s` inference-latency contract, and the full launch terminal continuously
  reports stable aligned target observations near `uv=(296,259..260)` and
  depth `0.347..0.350 m`. The old 3.476-second backlog is therefore not used as
  justification to widen a safety/temporal gate. No threshold, frame count,
  geometry constraint, or target-specific value was changed.
- Candidate generation had not yet been requested during this recovery
  measurement. The next run may now start a new independent post-restart
  three-hit window using only fresh RGB-D/mask/object and current joint
  evidence. The corrected execution runner will keep inference active until
  the exact Preview plan's durable audit or execution authority is committed.

### 2026-07-28 - bound plan executed; far-field observation accepted at 19.03 cm

- Started a fresh post-alignment automatic run with the corrected runner.
  Candidate generation produced a four-stage Preview with exact plan ID
  `9d7235fe5c26a64664c0be8f`. The runner observed the matching atomically
  committed continuous-execution audit, stopped candidate computation only
  after execution authority existed, re-read the exact same valid plan ID,
  and then called `/grasp/start`. No stop, disable, zero-torque, controller
  stop, or torque-off command was issued.
- The first far-field observation trajectory was planned from the measured
  current joints and executed in `49.525 s` at the live `0.080 rad/s` limit.
  The controller reported `SUCCEEDED`. After settling, the measured endpoint
  error was `0.0140 m` and `1.99 deg`; the first fresh camera-to-target
  distance was `0.177274 m`, just below the operator's generalized accepted
  interval `[0.180, 0.220] m`.
- The task therefore used its one allowed measured radial correction rather
  than a fixed object-specific pose. Its audit recorded the current measured
  range, nominal `0.200 m`, measured execution-error feed-forward, radial
  retreat vector, and preservation of the measured tool orientation. The
  correction trajectory's controller result was
  `GOAL_TOLERANCE_VIOLATED` for joint 2 at `0.038597 rad`, but the driver and
  gateway preserved the final desired-command hold with joint torque enabled.
- After motion settled, independent live perception measured
  camera-to-target distance `0.1903 m`. The unchanged observation contract
  accepted this because it is inside `[0.1800, 0.2200] m`, and the task
  switched from far-field to near-field planning. This is a measurement-based
  success, not a hard-coded target pose and not acceptance of the controller
  error alone.
- Near-field candidate generation is active using new RGB-D observations from
  the accepted observation pose. The first completed near-field request
  returned 18 locally valid candidates and entered the independent `1/2`
  stability window. Subsequent stable-candidate rechecks have rejected
  examples with continuous bilateral contact overlap
  `0.002837..0.002847 m` below the generalized `0.003000 m` minimum, while
  MoveIt sequence checks have rejected other examples because their pregrasp
  goals are unreachable from the measured current joints. These are live
  candidate-specific results; no object-specific pose, width, depth, or
  contact value has been inserted and no generalized geometry or collision
  gate has been weakened. The planner is still traversing remaining
  candidates while the enabled arm holds the accepted observation pose.

### 2026-07-28 - contact-overlap baseline separated from support clearance

- The first automatic task reached and accepted the far-field observation,
  but its near-field batches repeatedly produced stable physical candidates
  whose complete pregrasp sequences were unreachable from that measured
  observation joint state. One representative completed batch had `24/24`
  stable shortlisted sequences rejected by strict MoveIt as
  `MOVEIT_UNREACHABLE`. No second-stage motion was submitted.
- The operator then manually moved the enabled arm back to a new aligned
  posture. That invalidated the old target/plan epoch as intended; the old
  runner terminated with `GRASP_RESULT success=False`. The new steady feedback
  was near `[-107.2, 28.2, -6.3, 2.2, -13.6, -1.1] deg`, target depth about
  `0.356..0.359 m`, and normal status `0x00`. This task termination was not an
  arm disable and no stop/disable/torque-off command was sent.
- At the operator's explicit direction, changed the generalized continuous
  bilateral target/CAD contact-overlap baseline from `3 mm` to `2 mm`.
  This was not implemented as an object-specific or permanently fixed
  acceptance value. `AdaptiveStageLimits` now exposes
  `contact_overlap_min_m: 0.002`, and every fresh stage profile computes:
  `required overlap = max(0.002 m, same-pixel depth repeatability *
  depth_uncertainty_scale)`.
- The existing `3 mm` support-plane clearance remains unchanged and continues
  to protect the finger/palm swept collision envelope. Contact-surface depth
  evidence and support-plane collision clearance are now separate contracts;
  Cartesian endpoint residual still expands motion clearances but cannot
  manufacture contact overlap.
- Added the derived contact-overlap requirement to every adaptive stage
  profile and audit. Updated focused regression coverage verifies the `2 mm`
  floor, an increase to `6 mm` when measured depth repeatability is `3 mm`
  with scale `2`, and independence from Cartesian endpoint residual.
  Verification passed: adaptive profiles `15/15`, contact-overlap contract
  `2/2`, adaptive configuration loading `2/2`, Python compilation, and
  `git diff --check`.
- Wrote live ROS parameter
  `/grasp_6d/remote/adaptive_stage_generation/contact_overlap_min_m=0.002`.
  Requested only `/grasp_6d/request_plan trigger=false`, which confirmed
  `continuous remote 6D inference stopped`; then terminated only
  `/remote_grasp6d_node` and started its current-worktree replacement. The
  replacement reports the official loaded protocol-3 GraspNet backend online.
  The real driver, hardware interface, controllers, task node, camera,
  perception, GUI, enabled joint hold, and all other ROS nodes remained
  running. No `/grasp/stop`, controller stop, arm disable, torque-off, or
  `/demonstration=true` command was issued.

### 2026-07-28 - fresh aligned retry exposed an execution-publication race

- After the operator confirmed the target was aligned, a new independent
  far-field window produced stable Preview plan
  `d5bd6f2da7ef8617ad7ff06b`. Its bound audit was committed and cached
  promotion returned success. The live adaptive profile in this run computed
  `contact_overlap_requirement_m=0.0023721599400741977` from current depth
  repeatability, proving that the new `2 mm` value is a generalized floor
  rather than a fixed acceptance constant. Support-plane swept clearance
  remained independently fixed at `3 mm`.
- No trajectory was submitted. The execution runner timed out while waiting
  for the exact plan authority and stopped candidate inference only. Read-only
  task-node logs establish the exact failure sequence: the task accepted
  `d5bd6f2da7ef8617ad7ff06b` at `19:33:43.678`; six milliseconds later it
  received `PROMOTION_ABORTED: PROMOTION_SUPERSEDED`, and the subsequent
  same-stamp authority replay was rejected by the anti-replay watermark.
  `/grasp/current_plan` then correctly reported `PLAN_MISSING`.
- This is a concurrent publication defect, not target misalignment, candidate
  geometry, arm disable, or a failed trajectory. A ROS service thread was
  publishing the cached Preview as execution authority while an older
  latest-only worker was already finalizing another promotion. The older
  worker observed its token had been superseded and published an abort
  tombstone over the newer committed authority.
- Added one shared re-entrant execution-publication transaction lock. Cached
  replan promotion and worker-token promotion now serialize their complete
  executable publication transactions; state, geometry-generation, and token
  checks remain unchanged inside the lock. This prevents an older worker from
  placing a `PROMOTION_ABORTED` tombstone between a newer rich-plan publish
  and its commit. Added a focused two-thread regression that holds the cached
  transaction and verifies the worker publication cannot enter until it
  finishes. No grasp geometry, stability, MoveIt, collision, support,
  contact, target-drift, force, or observation-range rule was changed.
- Verification passed: the focused cached/worker concurrency and cached-replan
  group passed `3/3`, the broader promotion/publication group passed `10/10`,
  the complete remote streaming suite passed `201/201`, Python compilation
  passed, and `git diff --check` passed.
- The first live replay after that deployment reproduced the same class of
  failure with a narrower ordering: the task accepted plan
  `f3779500010d8dd547d3d082`, then about `11 ms` later received
  `PROMOTION_ABORTED: PROMOTION_SUPERSEDED`. Review of the service path showed
  that message publication was serialized, but
  `ExecutionPlanController.request_replan()` still mutated the promotion token
  immediately before the service entered the publication lock. Thus the
  in-flight worker could be inside its locked rich-plan publish while the
  service invalidated its token. This explains the live trace without
  changing or speculating about perception or geometry.
- Extended the same transaction boundary around the complete
  `/grasp_6d/replan_execution` callback, including the replan-policy mutation.
  The cached publisher uses a re-entrant acquisition of that same lock. Added
  a second two-thread regression that holds a worker publication, invokes the
  replan service concurrently, and verifies both callback entry and the
  controller's explicit-replan state remain unchanged until the worker
  transaction commits.
- The failed replay submitted no trajectory; its runner stopped candidate
  inference and exited on exact-authority timeout. Verification of the
  extended transaction passed: the focused set passed `4/4`, the complete
  remote streaming suite passed `202/202`, Python compilation passed, and
  `git diff --check` passed.

### 2026-07-28 - serialized promotion succeeded live; near-field sequences remained unreachable

- A third fresh aligned replay proved the complete publication transaction
  fix on the live ROS graph. Far-field Preview plan
  `2933d9a60f50840e43a0a52b` acquired a matching atomically committed bound
  audit, cached promotion returned success, the exact plan remained `VALID`,
  and `/grasp/start` accepted the same rich execution plan. There was no
  superseding tombstone and no `PROMOTION_ABORTED` event. This closes the
  execution-publication race demonstrated by the two preceding live runs.
- The first observation trajectory targeted the live-derived pose near
  `(-0.114,-0.387,0.121) m`, ran for `20.210 s`, and the controller reported
  `SUCCEEDED`. Its settled measured endpoint error was about `0.0142 m /
  2.15 deg`. The first independent camera-to-target measurement was
  `0.171103 m`, below the operator's inclusive `[0.180,0.220] m` observation
  contract.
- The existing single measured radial correction therefore derived a retreat
  of norm `0.0401408 m` from the live camera range, nominal `0.200 m`,
  current measured endpoint residual, and measured tool orientation. The
  correction targeted approximately `(-0.112,-0.375,0.148) m`. Its
  controller result was `GOAL_TOLERANCE_VIOLATED` for joint 2 at
  `0.042704 rad`; the gateway preserved the desired-command hold with joint
  torque enabled and did not issue torque-off.
- After settling, fresh RGB-D perception measured camera-to-target distance
  about `0.1876 m`, inside `[0.180,0.220] m`, so the task accepted the
  observation and explicitly entered near-field planning. This is a
  measurement-based interval result, not a fixed target pose.
- Near-field request 19 produced 25 locally valid candidates and reached the
  first of two required independent stability hits. Request 20 reached the
  required `2/2` stability evidence and admitted 18 stable candidates.
  Complete sequence checking then evaluated 24 shortlisted variants; all
  `24/24` failed strict MoveIt at `pregrasp` with
  `MOVEIT_UNREACHABLE`. Later completed near-field batches reproduced the
  same `24/24` pregrasp-unreachable result while the target remained stable
  near `uv=(338,182)`, depth about `0.185 m`, confidence about `0.91`.
  Therefore no second-stage pregrasp, approach, grasp, close, or lift command
  was submitted.
- At `19:48:25.900` the task ended with the exact state
  `NEAR_FIELD_REPLAN_TIMEOUT: no fresh near-field 6D preview after 150.0s;
  last=NEAR_FIELD_PLAN_PHASE_INVALID: Preview is not a contact execution
  plan`, and the automatic caller returned
  `GRASP_RESULT success=False message=failed`. The task reset the near-field
  phase to false; this task failure did not disable the arm. The real driver
  continued streaming the same enabled joint hold. No `/grasp/stop`,
  controller stop, arm disable, zero-torque, torque-off, or
  `/demonstration=true` command was sent.
- The generalized contact-overlap floor was active at `0.002 m`; live
  candidates below it were rejected, while candidates above it reached the
  stability and MoveIt stages. The terminal cause of this run is thus the
  repeated complete-sequence `pregrasp` unreachability from the measured
  observation joint state, not the 2 mm overlap floor. No reachability,
  collision, support-clearance, or geometry gate has been weakened from this
  result.
- During the final observation hold, measured joints remained near
  `[-108.8,-24.0,52.7,-6.7,-55.9,6.2] deg` versus the commanded hold near
  `[-109.1,-21.5,53.7,-6.5,-55.8,5.8] deg`. Hardware feedback returned
  repeated `E1` status events while measured temperature samples rose to
  about `55 C`. The driver logged each as a status event and explicitly sent
  no torque-off; investigation must retain these measured facts without
  inventing an unobserved cause or bypassing the operator's no-disable
  constraint.

### 2026-07-28 - near-field fixed Top-24 blind spot removed

- The operator clarified that the physical target is reachable for grasping
  and moved the arm to a new aligned starting pose. Fresh feedback after that
  move was near `[-109.0,28.9,-6.3,-6.6,-15.9,6.4] deg`; perception placed
  the target near `(-0.140,-0.478,0.070) m` in base with camera depth about
  `0.355 m`. These observations are distinct from the failed task's
  `18.5 cm` observation joint state and were not substituted into the prior
  failure report.
- Read-only MoveIt traces around the same target show that strict targets near
  `(-0.149,-0.483,0.180) m` can plan successfully, while several farther
  variants at base Y about `-0.506..-0.582 m` report no valid goal state.
  This supports the operator's statement that the target region is not
  globally outside the robot workspace, but it does not by itself assign
  failures to position, orientation, collision, or joint limits.
- The decisive pipeline accounting exposed a separate completeness defect.
  The failed near-field batch had 18 stable candidates and 36 identity/Rz-180
  variants pass current hard recheck. `bounded_moveit_select()` nevertheless
  truncated the sorted set to the fixed `moveit_top_n=24`; the remaining 12
  hard-safe variants received no strict MoveIt call. Consequently `24/24
  unreachable` proved only that the truncated shortlist was unreachable, not
  that all stable candidates were unreachable. This is the same structural
  class as the older fixed Top-8 blind spot recorded above, now exposed after
  the candidate set grew beyond 24.
- Added an explicit `exhaustive` mode to `bounded_moveit_select()`. Default and
  far-field behavior remain bounded by the configured Top N. Near-field
  contact selection alone now enables exhaustive mode, so every deduplicated,
  stable candidate variant that has just passed the current mandatory safety
  binding, analytical geometry, swept collision, contact, and support checks
  receives the unchanged strict ordered MoveIt sequence check. Selection
  still uses the existing deterministic soft/final score and every unreachable
  candidate still fails closed.
- This is not a gate relaxation and introduces no object coordinate, label,
  target-specific width, pose, or distance. It removes an arbitrary
  completeness cutoff after the hard gates. Audit `moveit_shortlist`,
  `moveit_checked`, and `moveit_reachable` counts now cover the entire
  near-field hard-safe set, so a future all-unreachable result is repeatable
  evidence about the full batch rather than the first 24 entries.
- Added regression coverage proving that a reachable variant at rank 30 is
  evaluated and selected in exhaustive mode, while default Top-N behavior is
  unchanged, invalid non-boolean mode values are rejected, and the runtime
  passes exhaustive mode only in near field. Verification passed:
  grasp6d pipeline `170/170`, remote node `156/156`, streaming suite
  `202/202`, Python compilation, and `git diff --check`.

### 2026-07-28 - initial observation controller failure now defers to the live range contract

- The next aligned replay generated far-field plan
  `8aa73cd7cc97f0b5690cdc33`. Its bound audit was committed, exact authority
  verification returned `VALID`, strict planning to the live-derived
  observation pose near `(-0.114,-0.384,0.117) m` succeeded twice, and the
  approximately `19.548 s` retimed trajectory was submitted.
- During that trajectory the independent camera depth decreased continuously
  from about `0.357 m` through `0.223 m`, `0.212 m`, and `0.198 m`. At
  `0.198 m`, already inside the operator-defined inclusive
  `[0.180,0.220] m` camera-to-target interval, the trajectory controller
  aborted with `PATH_TOLERANCE_VIOLATED` because joint 3 path error was
  `0.120358 rad`. The gateway preserved the last controller-desired hold,
  explicitly kept controllers and joint torque enabled, and sent no
  torque-off. The task nevertheless immediately ended as failed, before its
  fresh post-arrival range check could run.
- Code review found an exact wiring omission. The existing generalized
  post-submission recovery path recognizes only a strict cached-path
  controller failure, waits for motion settling, and then leaves acceptance
  to the unchanged live camera-range contract. It was already enabled for the
  optional single measured observation retreat, but the initial far-field
  observation move did not pass the same opt-in flag. Contact-plan motions do
  not use this recovery path.
- The initial `6D pregrasp` call now passes
  `allow_post_failure_observation_validation=defer_contact_gate`. Therefore
  only a far-field observation plan may defer this controller result; it must
  still settle, satisfy measured-endpoint handling, receive a fresh matching
  target observation, and measure inside `[0.180,0.220] m` before near-field
  activation. A planning failure, non-recognized execution error, failure to
  settle, stale/mismatched target, or out-of-range measurement still fails
  closed. Near-field pregrasp, approach, grasp, close, lift, collision,
  support, contact, MoveIt, and MuJoCo gates are unchanged.
- Added sequence-level regression coverage proving that the initial
  far-field call enables the deferral flag. The complete grasp-task sequence
  suite passed `129/129`; Python compilation and `git diff --check` passed.
- After the failed run, camera-space perception held near `0.183..0.184 m`,
  but the serial joint feedback also produced repeated large discontinuities
  that were sometimes admitted after two nearby frames. Those admitted
  samples caused corresponding base-frame target jumps while camera-space
  depth remained stable. This records the observed correlation only; it does
  not infer a physical jump or an unverified byte-level cause. No near-field
  motion was started from that inconsistent feedback. The operator then began
  a manual move, during which only offline work continued.

### 2026-07-28 - repeated discontinuity is now checked against the actually streamed command

- The next aligned run committed far-field plan
  `11b8b7ee44922cc48b8cd952`. Its live batch contained 37 hard-safe variants;
  the configured bounded far-field MoveIt stage checked 24 and found 13
  reachable, then selected an observation pose near
  `(-0.137,-0.381,0.115) m`.
- During the approximately `22.269 s` observation trajectory, valid feedback
  placed joint 4 near `-13 deg` and the actually streamed command was near
  `-13.3 deg`. Two CRC-valid feedback frames then reported joint 4 near
  `179.9 deg`. The former two-frame temporal confirmation admitted the second
  nearby sample, which directly produced
  `PATH_TOLERANCE_VIOLATED: Joint4 path error 2.910217`.
- The observation-only post-submission recovery added above then behaved as
  designed: it kept the controller-desired hold and torque enabled, waited for
  settling, and deferred acceptance to a fresh camera measurement. That
  measurement was `0.2422 m`, outside `[0.180,0.220] m`, so the task failed
  closed and no near-field/contact motion started.
- Added a generalized consistency check for a temporally implausible joint
  sample that repeats closely enough to satisfy the existing confirmation
  rule. The driver now remembers only the most recent command that was
  successfully written through the real SDK. If that fresh command exists
  and the repeated feedback candidate is farther from it than the previously
  accepted feedback by more than the already-derived temporal/confirmation
  margin, the sample remains rejected. The same rule applies to every joint
  and contains no object label, pose, grasp coordinate, or target-specific
  value. A genuine GUI or trajectory move updates the actually streamed
  reference and therefore continues through the normal confirmation path.
- Driver-only restart also exposed a deterministic initialization gap:
  controller commands can already be present before the restarted driver has
  accepted its first serial feedback. The real driver now seeds its command
  interpolation state exactly once from the first valid real feedback before
  smoothing any controller target. This prevents interpolation from the
  constructor's zero vector and does not issue any stop, disable, or
  torque-off request.
- Regression verification passed: serial-driver resilience `7/7`,
  `git diff --check`, and
  `catkin_make --pkg alicia_d_driver -DCMAKE_BUILD_TYPE=Release`. Only the
  driver node was replaced; the full ROS launch, motion gateway, controllers,
  camera, perception, GUI, task node, and remote 6D node remained running.
  The replacement driver opened `/dev/alicia_arm` at `1000000 bps`, received
  stable feedback near `[-112.1,21.4,-11.0,-12.7,-1.8,20.6] deg`, and
  `/joint_states` remained near `59 Hz`. Its startup retained automatic
  torque-on; no stop/disable/torque-off command was sent.

### 2026-07-28 - live replay proves the command-consistency filter and isolates a separate tracking stall

- After the operator aligned the target, the automatic runner opened a fresh
  image window, obtained three-hit stability, committed plan
  `dabc8397e4444e49fc525dec`, verified the exact authority as `VALID`, and
  started the far-field observation move. The selected observation target was
  near `(-0.116,-0.375,0.099) m`; the strict cached trajectory was retimed to
  about `14.872 s`.
- The new driver rejected an isolated joint-3 sample that jumped from about
  `1.8 deg` to `47.9 deg` while the fresh actually streamed joint-3 command
  was about `4.4 deg`. It later rejected a joint-4 sample near `179.9 deg`
  while the streamed joint-4 command was about `-15.4 deg`. Neither
  inconsistent sample was published into controller feedback, proving the new
  check on live hardware without weakening trajectory, collision, or contact
  gates.
- A different, directly observed failure remained. Valid joint-3 feedback
  advanced to about `8.9 deg` and then repeated at the same value while
  successful SDK writes advanced the desired joint-3 command to about
  `15.8 deg`. The trajectory controller aborted at
  `PATH_TOLERANCE_VIOLATED: Joint3 path error 0.120118 rad`. The gateway
  preserved the last controller-desired hold while explicitly keeping the
  controllers and joint torque enabled; no torque-off was sent.
- Camera depth decreased monotonically from about `0.335 m` to
  `0.257..0.260 m` before the abort. After settling, the observation-only
  recovery evaluated a fresh camera-to-target distance of `0.2601 m`, outside
  the inclusive `[0.180,0.220] m` interval, and correctly refused near-field
  activation. No second-stage or contact trajectory was submitted.
- Concurrent serial evidence included repeated `E2` status bytes and
  temperature frames containing physically inconsistent spikes such as
  `104 C`, `122 C`, `175 C`, and `228 C`, interspersed with immediate samples
  near `40..51 C`. The driver reported these as telemetry diagnostics and
  left motion enable unchanged. This log records their temporal correlation
  with the valid-feedback stall only; it does not infer that `E2`, heat,
  framing, wiring, or any other unverified mechanism caused the stall.

### 2026-07-28 - prior observation successes separated from the current transient stall

- The operator correctly noted that earlier first-stage observation moves did
  not exhibit this failure. Retained runs confirm that observation
  trajectories of `13.616 s`, `16.869 s`, `20.210 s`, `36.141 s`, and
  `49.525 s` all completed under the same `0.080 rad/s` strict trajectory
  limit. Therefore the current `14.872 s` duration does not support a claim
  that the observation trajectory is inherently too fast or unreachable.
- The same joint-3 controller boundary has nevertheless appeared twice in the
  most recent runs: `0.120358 rad` and `0.120118 rad` against the configured
  `0.120 rad` path tolerance. The former run had already reached a fresh
  camera distance near `0.198 m`; the current run stopped earlier near
  `0.260 m`. This repeated boundary evidence is distinct from candidate
  geometry and the hand-eye transform, and no object-specific coordinate is
  justified by it.
- Repository-local vendor SDK evidence gives an exact meaning to the observed
  status byte: `alicia_d_sdk/hardware/data_parser.py` maps `0xE1` to
  `overheat` and `0xE2` to `overheat_protect`; the English API reference
  publishes the same status vocabulary. The current run's valid-feedback
  stall coincided with repeated `0xE2` frames. However, the temperature stream
  also contained immediate physically inconsistent spikes and recoveries, and
  isolated `E2` frames occurred before motion while the arm could still move.
  The evidence therefore establishes the reported hardware status semantics
  and temporal correlation, not an unverified causal diagnosis of real heat,
  firmware, transport, sensor, mechanical contact, or wiring.
- Replaying the new feedback filter against the retained sequence shows that
  ordinary increments remained below its generic temporal allowance and were
  accepted. It rejected only the recorded multi-radian contradictions such as
  joint 3 near `47.9 deg` and joint 4 near `179.9 deg`. The filter changes no
  SDK command frame and cannot account for the accepted feedback holding near
  joint 3 `8.9 deg`.
- The operator then moved the arm and switched off its power. No further
  hardware command, torque command, trajectory, or candidate execution is
  permitted until a later explicit power-on/alignment report. Offline work
  below used only retained evidence, source, and unit tests.

### 2026-07-28 - one-shot observation correction generalized for both range directions

- The current failure exposed an exact state-machine gap after the existing
  observation-only controller-failure recovery. The task correctly measured
  `0.2601 m` after settling, but
  `_maybe_execute_observation_camera_retreat()` returned immediately for every
  distance at or above the lower bound. Its helper explicitly accepted only a
  too-close camera. Thus a partially completed first-stage motion could use a
  live radial correction when it overshot too close, but could not continue
  from a measured too-far view even though the same strict planner and
  unchanged `[0.180,0.220] m` contract remained available.
- Generalized the existing single correction without adding another attempt.
  It now computes the signed residual on the current measured
  camera-to-target ray:

  `radial_delta = nominal_distance - measured_distance`

  Positive residual retreats a too-close camera; negative residual continues
  a too-far camera toward the same live target ray. The command still subtracts
  the current plan-bound measured tool endpoint-error vector, preserves the
  current measured tool orientation, runs the unchanged strict MoveIt
  preflight and cached execution path, and obtains a new post-settle target
  sample. Near field remains forbidden unless that new Euclidean distance is
  inside the inclusive `[0.180,0.220] m` interval.
- The nominal `0.200 m` is the previously agreed observation setpoint, not the
  success equality and not an object coordinate. The accepted range remains
  18–22 cm. The correction is still finite (one attempt), recomputed from
  current measurements, and independent of object label, saved joint angles,
  object dimensions, contact depth, force, or tactile data. Contact pregrasp,
  approach, grasp, close, lift, collision, support-clearance, overlap, MoveIt,
  and MuJoCo gates are unchanged.
- Retained audit compatibility is preserved through the old signed
  `radial_retreat_xyz_m` field; new audits also state
  `radial_correction_xyz_m` and explicit direction `retreat` or `approach`.
  Added regression coverage for the current too-far case: measured distance
  `0.260 m`, nominal `0.200 m`, live radial component `-0.060 m`, current
  endpoint error `+0.010 m`, and resulting single command translation
  `-0.070 m`. The test proves exactly one strict preflight and one task motion
  call; it does not execute hardware.
- Verification passed after sourcing the ROS workspace: complete grasp-task
  sequence `131/131`, focused too-far tests `2/2`, default configuration
  `6/6`, Python compilation, and `git diff --check`. The initial unsourced
  unittest invocation failed only at import time because generated ROS Python
  messages were absent from `PYTHONPATH`; no test body ran in that invocation.
  The powered-off live task node has not been restarted and therefore has not
  loaded this offline change.

### 2026-07-28 - live bidirectional observation correction passed; exhaustive near-field sequences remained unreachable

- After the operator reported power on and alignment, hot-loaded only the
  updated `grasp_task_node.py`. The real-arm driver, motion gateway,
  controllers, camera, GUI, hand-eye publisher, remote planner, and joint
  enable state remained running. No stop, disable, torque-off,
  `/demonstration=true`, controller-stop, or `/grasp/stop` command was sent.
- A fresh three-hit far-field window generated plan
  `e9358c9fc1e108f9ec1db254`; its bound audit was committed, exact authority
  verification returned `VALID`, and the task accepted that same plan. The
  selected live observation target was approximately
  `(-0.118,-0.385,0.120) m`. Its strict trajectory was retimed to
  `20.188 s` and the controller reported `SUCCEEDED`.
- Independent perception depth decreased continuously from about `0.335 m`
  through `0.220`, `0.208`, `0.193`, and `0.177 m`. After settling, the
  measured tool endpoint error was `0.0143 m / 2.10 deg`; the fresh Euclidean
  camera-to-target range was `0.1720246 m`, below the inclusive
  `[0.180,0.220] m` contract.
- The new generalized one-shot correction therefore selected direction
  `retreat`. It derived a `0.0393079 m` command translation from the current
  target ray, nominal `0.200 m` range, measured endpoint-error feed-forward,
  and current measured tool orientation. The correction used strict planning
  and cached execution; it introduced no object coordinate or saved joint
  pose.
- The correction controller ended with
  `GOAL_TOLERANCE_VIOLATED: Joint2 goal error 0.041272 rad`. The gateway
  installed the controller-desired hold while keeping controllers and joint
  torque enabled. After settling, the measured endpoint error was
  `0.0220 m / 3.47 deg`, and an independent fresh target sample measured
  `0.1862 m`. The unchanged observation contract accepted this range and
  explicitly switched the task to near-field planning. This proves the new
  correction path on real hardware; acceptance came from RGB-D range, not
  from ignoring the controller result.
- Near-field perception remained stable at about `0.184..0.185 m`, target
  base approximately `(-0.149,-0.483,0.076) m`, and confidence about `0.906`.
  Request `100` established the first of two required disjoint stability hits.
  Request `101` obtained the required stability but strict MoveIt rejected
  every `28/28` hard-safe variants at the first `pregrasp` stage. Request
  `106` repeated the result over the now-exhaustive set: every `34/34`
  hard-safe stable variants was checked and rejected as
  `MOVEIT_UNREACHABLE`, with MoveIt reporting that it could not sample a valid
  goal state. Thus the earlier fixed Top-24 blind spot is no longer involved.
- The `2 mm` contact-overlap floor remained independent and active. Examples
  measuring `1.819`, `1.932`, `1.939`, and `1.993 mm` were rejected before
  MoveIt, while other candidates above the live contact requirement reached
  the exhaustive sequence check. No second-stage pregrasp, approach, grasp,
  gripper close, or lift trajectory was submitted.
- The atomic audit for request `106` recorded 19 stable candidates, 38
  variant evaluations, 34 hard recheck passes, and all 34 strict MoveIt
  failures. The adaptive geometry was live-derived: object height about
  `0.01913 m`, depth uncertainty `0.025036 m`, pregrasp distance
  `0.071202 m`, approach offset `0.044166 m`, lift height `0.055036 m`, and
  tilt family through about `26.926 deg`. These values include the measured
  `0.022036 m` Cartesian endpoint residual; they were not selected by object
  label.
- Serial feedback stayed near the enabled observation hold while status bytes
  progressed from intermittent `E1/E2` to repeated `E1`. Measured temperature
  samples rose from about `37 C` to approximately `53..54 C`. The driver
  logged the vendor-defined status events and explicitly sent no torque-off.
  This records correlation only and does not assign an unverified hardware
  cause to the MoveIt goal-state failures.
- Before manually moving the arm, the operator instructed continued offline
  work. Candidate generation was paused only through
  `/grasp_6d/request_plan trigger=false`; the service confirmed
  `continuous remote 6D inference stopped`. This was not an arm-stop command.
  The active task then naturally ended after `150.0 s` with
  `NEAR_FIELD_REPLAN_TIMEOUT`; it reset its planning phase and released the
  task slot without disabling the arm.
- After the operator finished moving and realigning, new steady perception
  settled near UV `(291,246)`, depth `0.334..0.337 m`, target base
  `(-0.141,-0.480,0.069) m`, and confidence about `0.903..0.906`. The failed
  near-field plan is not reusable after this motion. The next run must create
  a fresh far-field authority from these new observations, while retained
  request-101/106 audits remain the evidence set for the generalized
  near-field reachability diagnosis.

### 2026-07-28 - fresh realignment run repeated the near-field MoveIt boundary

- After the operator reported the target aligned again, a new automatic runner
  established source-time floor `1785296005472205638 ns`. It did not reuse the
  preceding failed authority. A ROS-time discontinuity of approximately
  `0.5 s` occurred while the runner was waiting, producing
  `TF_REPEATED_DATA` warnings and a TF-buffer clear. Perception subsequently
  returned to steady output; this is recorded as an observed clock event only,
  without assigning it as the cause of any later planning result.
- A fresh three-hit far-field window produced plan
  `1c55a1a0b0451e4f7876a02c` with source stamp
  `1785296018005986928 ns`, later than the runner floor. The runner committed
  the bound audit, stopped continuous candidate generation, verified that
  exact plan ID as `VALID`, and invoked the grasp task with that same
  authority.
- The observation trajectory completed its approximately `20 s` execution and
  settled with measured endpoint error `0.0151 m / 2.23 deg`. The fresh
  Euclidean camera-to-target range was `0.1732999 m`, so the generalized
  one-shot policy selected `retreat`. It derived a
  `0.0387318 m` translation from the live target ray, the `0.200 m` nominal
  setpoint, and current measured endpoint-error feed-forward while preserving
  measured tool orientation.
- The correction controller reported
  `GOAL_TOLERANCE_VIOLATED: Joint2 goal error 0.044749 rad`. The gateway
  installed the last desired-command hold without stopping controllers or
  torque. After settling, the endpoint error was
  `0.0232 m / 3.57 deg`; independent perception stabilized at approximately
  `0.184 m`, UV `(330,178..179)`, target base
  `(-0.149,-0.483,0.076) m`, and confidence about `0.899..0.901`.
  This lies inside the inclusive `[0.180,0.220] m` observation contract, so the
  task entered near-field planning and did not issue another observation
  correction.
- Near-field requests `162` and `163` supplied the required two independent
  stability hits. Request `163` recorded 18 stable candidates and expanded
  them to 36 generalized variants. All `36/36` passed the hard recheck and
  were submitted to strict MoveIt sequence feasibility; all `36/36` failed as
  `MOVEIT_UNREACHABLE`. The committed audit contains 58 rows and SHA-256
  `f2e78956447e7a894318e3dbd23e93fcbf09389053c9e9f484bb0e232440f588`.
  No near-field arm trajectory, gripper close, or lift was submitted.
- The `2 mm` continuous bilateral contact-overlap floor was active but was not
  the limiting gate for the stable set: the 36 variants above were already
  hard-safe before the MoveIt check. Separate raw candidates below
  `2.000 mm`, for example `0.345`, `1.213`, and `1.454 mm`, were correctly
  rejected as contact-patch misses; this is independent of the exhaustive
  MoveIt result.
- During the enabled hold, SDK feedback remained close to the streamed command
  while hardware status `0xE1` repeated. Ordinary temperature samples showed
  a peak near `55..56 C`. Isolated `150 C` samples appeared on different
  channels and did not repeat on one channel; the driver's same-channel
  persistence counter reset, so those samples are recorded as anomalous and
  are not asserted to be real temperatures. The driver explicitly logged that
  no torque-off was sent. No stop, disable, controller-stop,
  `/demonstration=true`, or `/grasp/stop` command was issued.
- A later independent near-field window strengthened the same boundary:
  request `170` recorded 24 stable candidates; 42 of 48 variants passed hard
  recheck and all `42/42` strict MoveIt checks failed as
  `MOVEIT_UNREACHABLE`. Its atomic audit contains 56 rows with SHA-256
  `c76c9a6f20a4a41917271e791771686f728a90d505b0aecd22e3571f37d8419a`.
  This is a second exhaustive result in the same unchanged observation hold,
  not a parameter guess.
- As ordinary temperature samples rose to about `59 C` with persistent `E1`,
  continuous 6D candidate generation was paused through
  `/grasp_6d/request_plan trigger=false`; the service returned
  `continuous remote 6D inference stopped`. This prevented an unreviewed new
  stage motion while preserving the arm driver, controllers, torque policy,
  and enabled hold. It was not `/grasp/stop` and did not stop or disable the
  arm.
- The task then reached its unchanged `150.0 s` near-field wait limit and
  ended as `NEAR_FIELD_REPLAN_TIMEOUT`; it released the execution slot and
  reset the planning phase to far field. The automatic runner returned
  `success=False`. No near-field motion occurred.
- Immediately before and after task release, the driver obtained repeated
  same-channel evidence at `60..61 C` with persistence `3/3`, and emitted an
  over-temperature telemetry diagnostic. The diagnostic explicitly states
  that autonomous torque-off is disabled and motion enable is unchanged.
  Hardware feedback then became stale. In response, the driver suppressed its
  `/joint_states` heartbeat and paused the SDK command stream once feedback
  age exceeded `1.0 s`; it did not send torque-off. Perception subsequently
  rejected new far-field outputs as `TARGET_LOST`. These are ordered
  observations only: without new serial/hardware evidence, this log does not
  assert whether the feedback loss came from device thermal protection,
  cabling, power, or another cause.

### 2026-07-28 - powered-off replay isolated orientation reachability and generalized the geometry gate

- The operator explicitly reported that arm power remained off. All work in
  this section used retained request-170 audit data, planning-only ROS
  services with `execute=false`, source inspection, and unit tests. No
  trajectory, gripper, enable, disable, torque, controller-stop,
  `/demonstration`, or `/grasp/stop` command was sent.
- Reconstructed all 42 request-170 hard-safe sequences from the committed
  snapshot transform, live support normal, adaptive stage profile, and
  candidate poses. Exact planning-only replay reproduced `0/42` reachable
  support-normal pregrasp poses. Direct `/compute_ik` returned `-31` both with
  and without collision checking for the same pose, while position-only
  planning reached that position. This separates full-orientation IK failure
  from workspace position and collision causes.
- Keeping the exact live contact orientation, direct planning-only IK reached
  both approach and grasp for `10/42` reconstructed variants. Their live
  candidate tilts were approximately `19.42..21.66 deg`. The unchanged
  contact orientation at the support-normal lift endpoint remained `0/42`
  reachable. Thus the retained evidence supports resolving free-space
  pregrasp/lift orientations independently while keeping approach/grasp at the
  perception-derived contact orientation; it does not justify a fixed joint
  pose, Cartesian coordinate, object label, or tilt.
- One planning-only existence experiment on source candidate 8 variant 0
  found a complete strict four-stage sequence after interpolating toward
  position-only terminal orientations. The particular sampled interpolation
  ratios were `0.9` for pregrasp and `0.1` for lift, but the resulting maximum
  joint change was about `3.314 rad`. These numbers are evidence of existence
  only and are explicitly not accepted as production constants or as
  real-hardware-safe output. A production resolver must derive poses from the
  current/virtual joint state, check the exact complete sequence, enforce
  bounded motion metrics, and rank live alternatives.
- Added prerequisite generalization in `gripper_geometry.py`. The physical
  contact orientation remains mandatory at `approach` and `grasp`, while the
  validated `pregrasp` and `lift` transforms may use independently resolved
  orientations. All four endpoints and all interpolated segment transforms
  still pass through the analytical finger/palm CAD, support-plane, and
  target-OBB checks; allowing a free-space orientation cannot bypass the
  swept-volume gate.
- Corrected the grasp-to-lift target model from translation-only transport to
  a full rigid transform. At every lift endpoint/interpolation sample, the
  target OBB center and axes are recomputed as
  `R_current * R_grasp^T` applied to the grasp-time relative pose. Therefore a
  rotating lift preserves the measured object/tool relative pose instead of
  checking a fictitious world-fixed OBB orientation.
- Added regression tests proving: an approach orientation different from the
  live contact candidate fails closed; safe distinct pregrasp/lift rotations
  pass all six gates; a pair of individually safe endpoints is rejected when
  the rotating pregrasp sweep makes a finger intersect the target OBB; and
  rigidly transported target center/orientation remain invariant in tool
  coordinates.
- Offline verification passed: complete gripper-geometry suite `99/99`,
  Python compilation, and `git diff --check`. This geometry change has not
  been loaded into any live ROS process and authorizes no hardware motion.
  The next offline step is a planning-only orientation-resolver API and fake
  planner tests. Real-arm promotion remains blocked until a later explicit
  power-on report and a separately reviewed bounded strict sequence.

### 2026-07-28 - planning-only free-space orientation seed API

- Added `ResolveFreeSpaceOrientations.srv` and the planning-only endpoint
  `/supervisor/resolve_free_space_orientations`. The request contains only
  ordered pose targets, stage names, linear flags, and per-stage orientation
  resolution flags. It deliberately has no `execute` field and its gateway
  handler performs no controller-start, controller-sync, trajectory execute,
  joint command, gripper command, or torque operation.
- A flagged stage is planned position-only from the exact current or prior
  virtual terminal `RobotState`. The terminal joint state is then evaluated
  through `/compute_fk`; only the resulting orientation is copied to the
  caller's unchanged Cartesian target position. The response also exposes
  seed-planning joint path cost, maximum joint delta, and FK position
  residual. It does not return a trajectory or store a cached trajectory.
- Unflagged stages retain their exact supplied pose and are planned in order
  using the same free/Cartesian semantics as the strict execution sequence.
  This permits a lift seed to be resolved specifically from a successfully
  planned virtual grasp state rather than from the real arm's initial state.
  A stage cannot simultaneously request position-only orientation resolution
  and Cartesian execution semantics; that contradictory request fails closed.
- The API output is explicitly an orientation seed, not motion authority.
  The original Cartesian position is preserved even if FK has a small
  position residual. Before any later promotion, a consumer must construct an
  exact candidate sequence, rerun the full-rotation analytical CAD/OBB gate,
  and pass that same exact sequence through
  `/supervisor/check_pose_sequence_strict`. No position-only plan can be
  executed through this API.
- Added planner tests proving that: pregrasp uses a position target rather
  than the supplied orientation; FK orientation alone is copied while XYZ is
  unchanged; lift resolution starts from the virtual grasp terminal joints;
  contradictory linear+resolve metadata is rejected without invoking a
  planner; and no trajectory cache is created. Added gateway coverage proving
  the resolver service does not perform a controller check.
- The existing worktree was generated by `catkin_make`; an attempted
  `catkin build` correctly refused to mix build-space owners. No build
  directory was removed. Rebuilding with the existing `catkin_make --pkg`
  workflow succeeded and generated 7 messages / 11 services.
- Offline verification passed after message generation: motion-gateway suite
  `25/25`, MoveIt-planner plus gripper-geometry suites `133/133`, Python
  compilation, and `git diff --check`. The newly generated service has not
  been loaded into the already-running gateway process, and no live service
  call was made while arm power was off.
- Production candidate selection has not yet been changed. In particular the
  retained example's `0.9/0.1` interpolation ratios and quaternions were not
  copied into source or configuration. The remaining consumer must derive its
  search from the live seed/contact geodesics, geometry-check every exact
  rotating sequence, strict-plan every promoted sequence, and rank by measured
  joint motion. A fixed ratio or object-specific pose is not acceptable.

### 2026-07-28 - cached-state planning replay exposed seed nondeterminism

- Host-side read-only service introspection confirmed that `/compute_fk`,
  `/compute_cartesian_path`, and
  `/supervisor/check_pose_sequence_strict` remained available. The already
  running gateway had not been restarted, so the newly built resolver service
  was not live. Offline scripts instantiated the new planner class directly
  and called only position-only planning, FK, and the existing strict
  planning-only sequence check. They did not call any execution, controller,
  joint, gripper, enable, disable, stop, or torque interface.
- The old request-170 audit predates the new audit fields. Its 48 stable
  evaluations contain neither `execution_sequence` nor
  `moveit_input_joint_state`. Therefore the replay did not claim to reproduce
  the request-170 planning start. It explicitly used MoveIt's retained cached
  state
  `[-1.8775924844, 0.4862719098, -0.1503301172, -0.1902136177,
  -0.2653786763, 0.1718058482]` for the six arm joints and marked
  `state_is_current_feedback=false`.
- Reconstructed source-8 variant-0 stage positions from recorded contact pose,
  snapshot transform/support normal, and the already retained adaptive
  profile (`pregrasp=0.074162936065 m`,
  `approach=0.045922512491 m`, `lift=0.056240423574 m`). These are replay
  inputs from the prior audit analysis, not new object constants and not
  production configuration.
- The first resolver replay failed because `_plan_position_from_start_state`
  reused the `0.25 s` strict pose-check budget; MoveIt reported
  `ABORTED: TIMED_OUT`. This did not establish position unreachability.
  Corrected the resolver to use the existing general MoveIt
  `/robot/planning_time` budget (default/current `2.0 s`) while final exact
  strict stages continue using `strict_pose_planning_time`. No target-specific
  timeout was introduced. Planner regression remained `34/34`.
- With the corrected generic budget, the same cached-state replay resolved a
  seed with `0.058 mm` FK position residual, joint path cost `3.441`, and
  maximum joint delta `2.452 rad`. A later call resolved another seed with
  `0.069 mm` residual, path cost `4.330`, and maximum delta `3.155 rad`.
  This difference triggered a repeatability audit rather than production use.
- A planning-only geodesic search used the configured generic
  `0.02 rad` cached-orientation tolerance as the maximum sample spacing.
  The live contact-to-seed distance produced 109 pregrasp samples; `22/109`
  exact `pregrasp -> approach -> grasp` prefixes passed strict MoveIt.
  Resolving lift from the best virtual grasp state produced 20 lift samples,
  of which `11/20` complete four-stage sequences passed strict MoveIt. The
  lowest path-cost result in this single replay had cost `4.820`, maximum
  joint delta `3.253 rad`, and data-derived fractions
  `0.10185185 / 0.26315789`. Those fractions and quaternions were printed only
  as replay evidence and were not written into production source/config.
- This result is not an execution candidate. The old audit does not retain the
  exact OBB axes/size and object point cloud needed to rerun the newly
  generalized rotating CAD/OBB gate, and the replay start was cached rather
  than fresh hardware feedback. Consequently
  `geometry_authority=false` was recorded regardless of strict MoveIt success.
- Five further position-only resolutions from the identical cached state all
  succeeded but were not repeatable: pairwise seed orientation spread was
  `1.113..2.849 rad`, path cost `2.047..4.916`, and maximum joint delta
  `1.311..3.097 rad`; FK residuals remained approximately
  `0.051..0.092 mm`. Local MoveIt headers show dedicated
  `~ik_constraint_sampler_random_seed` and
  `~joint_constraint_sampler_random_seed` inputs, while the active launch does
  not configure them. This proves a single position-only terminal pose is a
  stochastic sample, not a scientifically repeatable resolver output.
- No random-seed parameter was changed on the running MoveIt process and no
  production remote-planner consumer was connected. Before promotion, the
  resolver must either use a verified deterministic seed/search policy or
  return and rank a reproducible bounded seed set, then validate the exact
  sequence against fresh full geometry and strict MoveIt. The current
  single-seed API remains planning-only scaffolding and cannot authorize
  hardware motion.
- Final powered-off regression passed across the complete affected suites:
  gripper geometry, MoveIt planner, and motion gateway `158/158`, plus Python
  compilation and `git diff --check`. The worktree contains extensive
  pre-existing changes; this pass did not reset, stage, commit, or rewrite any
  unrelated file.

### 2026-07-28 - powered/aligned live candidate-only evidence after resolver reload

- The operator explicitly reported that arm power was on and the RGB-D camera
  was aligned with the target. Fresh `/joint_states` feedback was present,
  `/alicia_d/motion_enabled=true`, and
  `/alicia_d/protection_latched=false`. Because the driver already reported
  motion enabled, no redundant `/demonstration=false` was published; no
  disable, stop, torque-off, trajectory, gripper, or `/grasp/start` command
  was sent.
- The calibration interlock was already released:
  `/grasp/calibration_interlock_active=false` with reason
  `USER_OVERRIDE_TEMPORARY_RENEWED_20260728_ALIGNED_AUTOMATIC_6D_AFTER_FULL_RESTART`.
  The task node was idle in the prior failed terminal state and the execution
  slot was released.
- Hot-reloaded `motion_gateway_node` exposes the new planning-only
  `/supervisor/resolve_free_space_orientations` service. The active
  `remote_grasp6d_node` reports GraspNet Baseline loaded with protocol 3. This
  did not restart or command the driver, controller, MoveIt, task node, GUI,
  or joint-enable path.
- An initial one-shot publication to `/grasp_6d/request_plan` had no consumer:
  runtime introspection proved that name is a `TriggerZero` service rather
  than a topic. The publication therefore changed no ROS node, task state, or
  robot state. Calling the actual service with `trigger=true` then started
  candidate-only continuous inference.
- Fresh requests 1 and 2 reached stability hit counts `1/3` and `2/3`.
  Request 3 produced stable tracks and committed a far-field observation
  preview. The selected observation was derived from the live target and had
  camera-target distance `0.20000000000000004 m` inside the configured
  `[0.18, 0.22] m` interval, center residual below `8e-18 m`, analytical
  minimum support clearance `0.061853570883767514 m`, strict planning joint
  path cost `1.616 rad`, and maximum joint delta `1.016 rad`.
- The observation target was
  `xyz=(-0.07924957150508587, -0.3897010344623809,
  0.11891428708786192)` with quaternion
  `(0.8516933770590895, 0.4836749426892491,
  -0.15127603159002415, 0.1333885435658147)`. These values are an immutable
  record of this live preview, not configuration constants and not
  object-specific source values.
- The same live evidence did not authorize near-field grasp execution.
  Stable recheck rejected one branch whose measured minimum insertion tilt
  was `45.078638 deg`, only `0.078638 deg` above the current generic
  `45.000000 deg` hard bound. Other candidates were independently rejected
  for missing common bilateral contact height, palm/support collision,
  finger/target intrusion, width mismatch, or support clearance below the
  generic `3 mm` floor. The configured live contact-overlap requirement in
  the selected profile was `2 mm`; no `6.5 mm` object-specific constant was
  used in this run.
- Continuous request submission at `1.5 Hz` exceeded the observed
  approximately `2.95..5.98 s` end-to-end processing time, so later tickets
  were intentionally replaced with `PENDING_REPLACED` rather than supplying
  additional independent evidence. Called `/grasp_6d/request_plan` with
  `trigger=false` only to stop submitting new inference requests. This was
  not an arm stop, disable, torque, controller, or task-stop command.
- The fresh audit now records the exact selected execution sequence and the
  latest joint-state sample used immediately before strict planning. However,
  its top-level rows still do not serialize the complete target OBB
  center/axes/extents and frozen target point cloud needed to replay the
  full-rotation analytical gate after resolving free-space orientations.
  Also, the previously measured position-only orientation seed remains
  nondeterministic. Therefore `/grasp/start` remains intentionally uncalled
  until both replay authority and a reproducible bounded resolver policy are
  implemented and verified.

### 2026-07-28 - request-bound full geometry replay evidence added

- Added `replay_geometry` to every continuous request's on-disk planning
  audit. It contains the exact frozen `base_link` target point cloud, point
  count and SHA-256, OBB center, `R_base_obb`, OBB extents, support normal,
  support offset, support inlier ratio, and source segmentation mode used by
  that request. Invalid or unavailable geometry fails closed with an explicit
  reason instead of emitting guessed values.
- The full point cloud is not included in the bounded ROS gate-audit topic;
  only the atomic on-disk report receives it. This preserves live ROS traffic
  bounds while making later full-rotation CAD/OBB replay possible against the
  same immutable geometry.
- Added an integration assertion to the streaming audit test proving exact
  numeric round-trip of the point set, OBB, support plane, point count, and
  point-cloud SHA-256. The targeted test passed `1/1` after loading the
  existing catkin environment. An earlier invocation without
  `devel/setup.bash` did not collect because generated ROS Python messages
  were absent from `PYTHONPATH`; it was not a source or test failure.
- This change has not yet been hot-loaded into the running candidate node and
  does not authorize motion. No robot, gripper, controller, enable, disable,
  stop, or torque command was sent.

### 2026-07-28 - deterministic bounded free-space orientation resolver integrated

- Replaced the resolver's stochastic MoveGroup position-only constraint target
  with a request-local deterministic quaternion geodesic. For every flagged
  free-space stage it samples exact orientations between the
  perception-supplied contact orientation and a reachable orientation derived
  from the current/virtual joint state. Sampling step `0.02 rad`, maximum
  count `96`, IK timeout `0.05 s`, and repeatability tolerance `1e-6 rad` are
  generic numerical/search bounds in `joint_limits.yaml`; none is an object
  pose, label, Cartesian coordinate, or object-specific grasp constant.
- Each exact orientation uses collision-aware `/compute_ik` from the same
  explicit virtual joint seed. Candidates are ranked by measured joint-space
  path norm, maximum joint change, then geodesic fraction. The selected
  orientation is solved a second time from the identical seed; different
  joint name sets, a missing repeated solution, or joint difference above
  `1e-6 rad` fails closed as `MOVEIT_RESOLVE_NONDETERMINISTIC`.
- The resolver copies only FK-derived orientation onto the caller's unchanged
  Cartesian position. It returns no trajectory, caches no plan, calls no
  controller, and cannot execute. Intermediate `approach` and `grasp` remain
  the exact perception-derived contact orientation. A resolved `lift` uses
  the same request's resolved pregrasp orientation as its deterministic
  free-space anchor while starting IK from the virtual grasp terminal joints.
- Integrated the resolver into near-field candidate checking as a fallback
  only after the original strict four-stage sequence fails. The consumer
  requires the resolver message to attest
  `policy=deterministic_geodesic_collision_ik`, verifies that all four XYZ
  positions are unchanged, verifies exact approach/grasp quaternion
  preservation, and bounds the reported FK position residual by the existing
  generic cached-position tolerance.
- Before any resolved sequence is strict-planned, the exact pregrasp,
  approach, grasp, and lift transforms are rerun through the same request's
  frozen target points, OBB, support plane, finger/palm CAD, bilateral contact,
  and full interpolated rotation sweeps. Only a geometry-passing exact
  sequence is submitted to the existing planning-only strict sequence
  service. Resolver policy/metrics and the exact resulting sequence are added
  to the candidate and streaming audits.
- A newly added test initially exposed that both the local four-stage data
  class and the generated ROS message were named `Grasp6DPlan`. The ROS import
  shadowed the local class. The local class now has the explicit internal alias
  `Grasp6DSequence`; no message schema or runtime wire protocol changed.
- Regression passed across every affected suite: deterministic MoveIt
  resolver plus gateway `60/60`, remote candidate node `156/156`, and
  streaming state machine plus full-rotation gripper geometry `304/304`,
  total `520/520`. Python compilation also passed. No ROS hardware service,
  motion, gripper, enable, disable, stop, or torque command was used.

### 2026-07-28 - hot reload and live planning-only repeatability proof

- With the task execution slot idle and
  `/alicia_d/motion_enabled=true`, allowed the existing respawning
  `motion_gateway` to exit normally. The unchanged main `roslaunch` replaced
  PID `39721` with PID `44327`; the deterministic resolver service returned
  online. No driver, hardware interface, controller, MoveIt, task, GUI, or
  joint-enable process was restarted or commanded.
- Stopped only the idle, manual-trigger `remote_grasp6d_node` PID `39723` and
  started the updated Python node as PID `44422`, supervised in terminal
  session `62542`. It reports GraspNet Baseline loaded, protocol 3, and the
  expected WSL URL. Candidate inference remains manual and stopped.
- Called the planning-only resolver twice with the exact same real request:
  the previously selected live observation Cartesian point
  `(-0.07924957150508587, -0.3897010344623809,
  0.11891428708786192)` and its live quaternion. Both calls used the same
  unchanged real joint feedback and returned byte-for-byte equal numeric
  results:
  resolved quaternion
  `(0.7730464126518943, 0.5267967034495104,
  -0.27087816803355685, 0.22695703382700086)`,
  joint path cost `1.2632687611739102`, maximum joint delta
  `0.7785374283205193 rad`, and FK position residual
  `7.709120481933112e-09 m`.
- Each call tested 27 deterministic geodesic orientations and reported
  `max_repeatability_error=0.000000000`. The returned XYZ exactly matched the
  requested XYZ in both calls. This closes the previously observed
  `1.113..2.849 rad` single-position-seed spread for this live input without
  setting MoveIt's process-global random sampler seed.
- These were service calls to an API with no execute field. No trajectory was
  cached or executed, and no gripper, enable, disable, stop, torque, joint, or
  controller command was sent.

### 2026-07-28 - stale far-field plan root cause and exact early convergence fix

- The operator reported the arm powered and the camera aligned. The prior
  far-field observation preview was not executed: a read-only
  `/grasp/current_plan` request rejected it as `PLAN_STALE`. Its request spent
  about `26.1 s` strictly checking all 24 information-ranked observation
  candidates, exceeding the independently enforced `15.0 s` result-age
  limit. This was a scheduling/freshness failure, not evidence of a geometry,
  pose, calibration, or reachability failure.
- The far-field final selector already uses the same deterministic tuple as
  its pre-MoveIt ordering: measured side-evidence deficit, frozen observation
  translation, projected side evidence, request-local soft score, track ID,
  and variant index. Therefore, after strictly rejecting every higher-ranked
  item, the first strictly reachable item is mathematically the exact optimum
  for this phase. Checking a lower-ranked tail cannot change the selected
  observation pose.
- Added the generic `first_reachable_by_rank` policy to
  `bounded_moveit_select`. It requires an explicit ranking function, rejects
  ambiguous use with exhaustive mode, and stops only after a candidate has
  returned structured strict-MoveIt success. If none is reachable, the whole
  bounded shortlist is still checked; the result is never falsely described
  as exhaustive when an unchecked tail exists.
- Enabled that policy only for the far-field observation phase. Near-field
  contact planning remains exhaustive across the complete current hard-safe,
  deduplicated candidate set and retains exact four-stage geometry, collision,
  IK, joint-limit, strict planning, and MuJoCo gates.
- The on-disk request audit now records the selection policy, configured and
  actual shortlist counts, actual strict-check and reachable counts, whether
  convergence stopped early, and
  `FIRST_REACHABLE_BY_AUTHORITATIVE_RANK`. Unchecked lower-ranked observations
  are explicitly identified as neither tested nor rejected.
- Regression passed: pipeline selection `175/175`, remote candidate node
  `156/156`, and continuous streaming/state machine `205/205`, total
  `536/536`; Python compilation and whitespace validation also passed.
- The updated candidate node has not yet been hot-reloaded and no new live
  candidate has yet been generated. No arm enable, disable, stop, torque,
  controller, trajectory, gripper, or `/grasp/start` command was sent during
  this diagnosis and offline verification.

### 2026-07-28 - live early-convergence proof and exact-plan runner hardening

- Hot-reloaded only `remote_grasp6d_node`; the replacement connected to
  GraspNet Baseline protocol 3 at the configured WSL endpoint. Driver,
  controller, MoveIt, task, GUI, and arm-enable processes were not restarted
  or commanded.
- Live far-field request 11 proved the new scheduling rule: its 24-item
  shortlist entered exactly one strict MoveIt check, that first candidate was
  reachable, and the audit reported
  `FIRST_REACHABLE_BY_AUTHORITATIVE_RANK`. End-to-end latency was about
  `10.026 s` and result age at completion about `11.426 s`, versus the prior
  approximately `26.1 s` all-candidate pass.
- The request promoted plan `f5c8e7342d1f1815b87426a9`. Its latched rich
  plan was a `FAR_FIELD_OBSERVATION_PLAN` derived from the live carton
  geometry, not a fixed Cartesian pose. The execution node initially returned
  `validation=VALID`, then correctly returned `PLAN_STALE` after the
  configured `120.0 s` plan-validity window elapsed while additional
  evidence was inspected. It was not executed.
- Continuous `1.5 Hz` submission still produced `PENDING_REPLACED` events
  while each accepted request consumed multiple seconds. Candidate
  computation was stopped with `/grasp_6d/request_plan trigger=false`; this
  stops only new inference submission and is not an arm/controller stop.
- Hardened `tools/run_fresh_grasp6d_after_alignment.py` so an automatically
  promoted plan cannot bypass its atomically committed same-`plan_id` audit.
  The runner now requires hashed request-bound replay geometry, a non-empty
  point cloud and SHA-256, authoritative far-field selection, exactly one
  reachable optimum, strict MoveIt evidence, exactly one audited observation
  execution stage, camera-target distance inside the live `[0.18, 0.22] m`
  band, and open-gripper support clearance at least `0.003 m`.
- The runner also compares the audited observation position and quaternion
  against the exact first pose of the same rich preview to `1e-9`, then
  rechecks that `/grasp/current_plan` returns the same `plan_id` as valid after
  candidate computation is frozen. Only that path can call `/grasp/start`.
  Python compilation, whitespace validation, and an offline check against the
  latest real atomic audit all passed; the audit validator returned no error.
- No arm enable, disable, stop, torque, controller, trajectory, gripper, or
  `/grasp/start` command was sent in this live validation pass.

### 2026-07-28 - exact-plan start reached controller; Joint3 path tracking aborted

- The hardened runner generated and atomically validated fresh far-field plan
  `7ce685be7728c2ed125a10fb`. Its preview age was `11.190959 s`, the task node
  returned the same `plan_id` as `validation=VALID`, and the bound audit was
  committed in `continuous_execution` mode before candidate inference was
  stopped.
- The bound request used the authoritative first-reachable policy with one
  strict check and one reachable result, exact request-bound replay geometry,
  a `0.200000 m` camera-target observation distance inside `[0.18, 0.22] m`,
  and a passing open-gripper support envelope above the generic `3 mm`
  clearance. The runner then made the authorized `/grasp/start` call.
- Planning and pre-execution synchronization passed. The exact observation
  trajectory was retimed to `20.734 s`, with maximum joint path delta
  `1.037 rad`, velocity bound `0.080 rad/s`, execution start error
  `0.001534 rad`, and controller feedback synchronization error
  `0.009204 rad <= 0.035 rad`.
- Physical execution aborted at the trajectory controller, not at perception
  or candidate geometry:
  `PATH_TOLERANCE_VIOLATED: Joint3 path error 0.120959`. The active controller
  configuration has a `0.120 rad` trajectory tolerance on every arm joint, so
  the measured Joint3 error exceeded that exact boundary by `0.000959 rad`.
- The serial trace shows Joint3 feedback/command progression in degrees:
  `-9.1/-8.5`, `-8.3/-5.6`, `-5.5/-2.7`, `-3.9/+0.2`, then
  `-3.9/+~3.0`. Thus Joint3 stopped advancing for the final feedback interval
  while the time-based desired reference continued, producing the reported
  approximately `6.93 deg` path error. Joint feedback remained online and
  temperatures later reported a maximum of `43 C`.
- The gateway preserved the last controller-desired command while keeping
  controllers and torque enabled; it did not issue a disable. After settling,
  the task measured observation endpoint error `0.1330 m / 20.75 deg`.
  Because an eye-on-hand camera moved without reaching its planned endpoint,
  the subsequent live target coordinate shifted; the task then reported
  `TARGET_DRIFT 0.041 m > 0.040 m`. This is downstream evidence of incomplete
  arm motion and must not be interpreted as the carton physically moving.
- The operator then stated they were manually moving the arm. Candidate
  inference was already stopped. No further trajectory, grasp start, enable,
  disable, stop, or torque command will be sent while that manual move is in
  progress. Offline diagnosis continues against the recorded controller and
  serial evidence.

### 2026-07-28 - post-alignment command ownership and power-cycle enable evidence

- After the operator finished repositioning and reported the target aligned,
  no candidate or trajectory was started. Read-only `/joint_commands`
  introspection found three concurrent publishers:
  `/bessica_d_hw_interface`, `/alicia_supervisor_gui`, and `/motion_gateway`.
  This matches the source architecture and provides a concrete mechanism for
  the previously reported extra motion after GUI slider use; it does not by
  itself assign each historical message to one publisher.
- The stronger live discrepancy was between the driver's successfully written
  SDK hold and independent real feedback. The outgoing hold contained Joint2
  near `-9.8 deg`, while repeated real frames held Joint2 near `+15.8 deg`, a
  separation of about `25.6 deg` (`0.447 rad`). This proves the hardware was
  not following the retained pre-power-cycle command and is not a perception,
  target-alignment, or 6D-candidate result.
- `/alicia_d/motion_enabled=true` was only the driver's retained software
  state. Source inspection confirmed that the positive enable callback clears
  the stored SDK command before writing `torque_on`; therefore re-enabling
  cannot immediately replay that retained Joint2 target. Under the operator's
  standing explicit request for joint enable, published exactly one
  `/demonstration=false`. No opposite value, torque-off, controller stop,
  `/grasp/stop`, gripper command, or trajectory was sent.
- At ROS time `1785302748.388114190` the real driver confirmed
  `Disabling zero-torque mode with SDK torque_on frame.` The old SDK command
  keepalive then ceased, rather than moving the arm toward `-9.8 deg`.
  Read-only confirmation returned `/alicia_d/motion_enabled=true`,
  `/alicia_d/run_status=0`, and current joints approximately
  `[-109.1,+15.8,+1.8,-12.3,-22.7,+14.7] deg`. Candidate inference remains
  stopped and the arm remains at the operator-aligned pose.
- Raw feedback is arriving at approximately `59 Hz` on `/joint_states`.
  Hardware frames remain mostly status `0x00` with intermittent vendor-status
  `E1/E2`; ordinary measured maxima were approximately `40..44 C`, with
  isolated higher samples. The driver records those as telemetry and sends no
  autonomous torque-off. This section does not infer a physical thermal cause.
- The remaining execution diagnosis is separate: three recent strict
  observation trajectories terminated at Joint3 path errors
  `0.120358`, `0.120118`, and `0.120959 rad` against the unchanged
  `0.120 rad` controller boundary. Widening that boundary is rejected because
  the latest partial execution ended with `0.1330 m / 20.75 deg` tool error.
  The next offline correction must constrain the actual local velocity of the
  unchanged joint path, rather than relying only on endpoint delta divided by
  total duration or hiding physical lag behind a larger tolerance.

### 2026-07-28 - local joint-velocity contract now preserves path and scales time

- The strict real-arm retimer previously logged a
  `strict_execution_max_joint_velocity_rad_s` contract but enforced only
  `endpoint max_delta / total duration`. A nonlinear or unevenly timed path
  could have a faster local segment while satisfying that endpoint-average
  inequality. This is the same mathematical defect already isolated and
  corrected in the WSL prescribed-path timing work; it is independent of the
  target object.
- Added local peak measurement over every retimed joint-trajectory segment.
  The validator combines declared point velocities with finite differences
  of adjacent positions and timestamps. Non-finite values, inconsistent
  widths, missing timing, or non-increasing timing fail closed. When any local
  peak exceeds its applicable limit, the complete trajectory is uniformly
  time-stretched; positions and the full joint path remain bit-for-bit
  unchanged, velocities scale inversely with time, and accelerations scale
  with inverse time squared.
- Added a generic optional per-hardware-joint limit map. The unchanged global
  ceiling remains `0.08 rad/s`. Joint3 is tightened to `0.02 rad/s`, derived
  from the latest retained raw trace: valid Joint3 feedback moved from
  `-9.1 deg` at `1785302258.008` to `-3.9 deg` at
  `1785302262.405`, or approximately `0.0206 rad/s` sustained, while the
  reference advanced near `0.052 rad/s` and feedback repeated for the final
  interval. The configured value is the measured hardware following rate
  rounded downward, not an object coordinate, stage pose, or candidate
  constant. It applies to every target and strict execution stage.
- Added regression coverage for three contracts: a globally under-timed path
  is stretched rather than merely rejected; a late local speed peak that
  endpoint-average timing misses is detected and stretched; and a per-joint
  limit selects the correct limiting joint while preserving all path points.
  The complete MoveIt pose/feedback suite passed `37/37`, the focused
  production configuration test passed, Python compilation passed, and
  `git diff --check` passed.
- The full trajectory-configuration file still has one pre-existing unrelated
  failure: it expects `observation_endpoint_correction_attempts=1` while the
  current worktree configuration is `0`. That setting was not changed to make
  this patch pass because it belongs to a separate observation-recovery
  policy. The local-speed tests and production limit assertion pass.
- The new planner code and parameter have not yet been loaded into the live
  motion gateway. Candidate inference remains stopped. No trajectory,
  gripper, stop, disable, controller switch, or additional enable command was
  sent during implementation and testing.

### 2026-07-28 - live local-speed attempt exposed no actuation and invalid recovery motion

- Hot-loaded the local-speed retimer by respawning only `/motion_gateway`
  under the existing launch supervisor, then set the live generic parameter
  `/robot/strict_execution_joint_velocity_limits_rad_s={Joint3: 0.02}`.
  The driver, arm power/enable state, camera, GUI, task node, and other ROS
  nodes were not stopped. No disable or torque-off command was published.
- With the operator reporting the camera aligned, the fresh automatic runner
  bound promoted plan `7edc3dea3e0fa255cbd4747e`. The first observation
  trajectory proves the new implementation was active:
  `duration=36.999 s`, `max_delta=0.740 rad`,
  `peak_velocity=0.020 rad/s`, `limiting_joint=Joint3`, and
  `velocity_time_scale=2.500`. The exact joint path was preserved and only
  time was stretched.
- This run did not validate physical following at the new speed. Real SDK
  feedback remained fixed at approximately
  `[-109.1,+15.8,+1.8,-12.3,-22.7,+14.7] deg` while controller/driver command
  values advanced. After about six seconds the controller aborted at
  `Joint3 path error 0.120399 rad`. The settled FK residual to the requested
  observation pose was `0.1164 m / 15.99 deg`. The result was therefore
  `GRASP_RESULT success=False`; it is an actuation/following failure, not a
  candidate-pose or local-retimer acceptance result.
- The state machine then exposed a separate recovery defect. A recoverable
  far-field controller failure was allowed to flow into the live range logic
  without remembering that execution had failed. With the camera-target range
  still `0.3067 m`, it combined the `0.1164 m` endpoint error with the radial
  term and generated a second `0.2163 m` correction command. That command also
  saw no real joint movement and aborted at
  `Joint2 path error -0.120439 rad`. The unchanged target did not move; the
  large correction was produced from the arm's measured failure to reach the
  requested first pose.
- Corrected this generically without adding an object or workspace constant.
  A far-field controller failure may still be treated as a controller-side
  false negative only when the subsequent fresh, live camera-target
  measurement itself already satisfies the configured user contract
  `[0.18, 0.22] m`. If it remains outside that interval, the task now returns
  `OBSERVATION_CORRECTION_AFTER_FAILED_EXECUTION_FORBIDDEN` and refuses a
  second physical trajectory. A normally successful first move retains the
  single live-derived radial correction. Any failure of that correction now
  fails immediately instead of entering the same recovery path again.
- Added regression tests for the retained in-range false-negative path, the
  post-failure correction prohibition, and the normal successful correction
  path. The complete task-sequence suite passed `132/132`; Python compilation
  and `git diff --check` also passed.
- After the operator reported a new alignment, read-only live samples showed
  `/alicia_d/motion_enabled=true`, `/alicia_d/protection_latched=false`, and
  `/alicia_d/run_status=0`. The controller still retained the aborted
  correction's desired vector: Joint2 desired/actual residual was exactly
  `-0.120439 rad` and Joint3 residual was `+0.095996 rad`. No
  `/joint_commands` message arrived during a five-second sample, so no current
  trajectory was being sent. This retained desired state must be synchronized
  from fresh feedback by the existing controller-start bridge before any
  later automatic execution. No new movement or enable/disable command was
  sent while recording this evidence.
- The existing `grasp_system.launch` definition does not mark
  `/grasp_task_node` for respawn. After ending only that inactive node to load
  the corrected branch, the main launch retained a stale registration but did
  not recreate the process. Started only the updated task node from the
  current worktree. New PID `53704` reported `GraspTaskNode ready`, registered
  `/grasp/start` and `/grasp/current_plan`, entered `IDLE`, and rejected the
  previously latched plan as `PLAN_STALE`. The driver, motion gateway,
  controllers, remote inference node, camera, GUI, and joint enable state were
  unchanged; no trajectory or enable/disable command was sent by this reload.

### 2026-07-28 - repeated positive enable proved firmware actuation absent; latent-target recovery corrected

- The first fresh task after the task-node recovery fix used promoted plan
  `cb7a6e2d4745f10dab17a3c8`. Its first observation trajectory was retimed to
  `37.385 s` and the Joint3 local limit was active. Controller desired
  positions advanced while real feedback remained fixed at approximately
  `[-109.1,+15.8,+1.8,-12.3,-22.7,+14.7] deg`; the controller aborted at the
  Joint3 path boundary. The new task-node branch then returned
  `OBSERVATION_CORRECTION_AFTER_FAILED_EXECUTION_FORBIDDEN` with live
  camera-target distance about `0.3066 m`, proving that it did not issue the
  invalid second physical correction. The task result was false.
- Published one positive `/demonstration=false` request, which is the existing
  driver command for joint torque enable. No `true`, stop, controller-stop, or
  torque-off request was published. Source inspection established an important
  limitation: the driver writes the vendor frame and sets its software
  `/alicia_d/motion_enabled=true` state, but the vendor parser exposes no torque
  acknowledgement. The vendor SDK's apparent `wait=True` path also does not
  wait for a torque event because torque-on is absent from its event map.
  Therefore `motion_enabled=true` is permission/write evidence, not proof that
  the actuator firmware accepted torque.
- Repeated the task only after that positive enable, using fresh promoted plan
  `da719b66d68423ac240d9d1a`. Controller synchronization started with zero
  measured error and the first observation trajectory was retimed to
  `36.959 s`; its desired Joint3 advanced from about `1.8 deg` through
  `2.1, 3.7, 4.9, 6.1, 7.3, 8.4/8.7 deg`. Driver SDK writes reported success,
  but all six real joint positions remained fixed. The controller aborted at
  Joint3 error `0.120002 rad`. The settled Cartesian residual was about
  `0.1158 m / 16.26 deg`, and the task correctly blocked a second observation
  correction with live range about `0.3073 m`. This repeated zero-response
  trace proves that the active blocker is physical/firmware actuation
  acceptance; candidate inference, TF, the `[0.18,0.22] m` observation
  contract, and the new local retimer did not cause the zero motion.
- The failed controller still retained desired positions
  `[-1.913890,+0.167222,+0.152216,-0.222095,-0.453620,+0.279937] rad` while
  actual remained
  `[-1.903670,+0.276117,+0.032214,-0.214757,-0.395767,+0.256175] rad`.
  The maximum residual was Joint3 `+0.120002 rad`; a read-only sample received
  no `/joint_commands` messages. A later positive enable cleared the driver's
  outgoing command stream but, as expected, did not rewrite the trajectory
  controller's retained desired state.
- Corrected the generic failed-execution hold policy. Each strict execution now
  captures the fresh measured arm vector immediately before execution. On
  failure it compares that vector with fresh controller actual and desired
  positions. Only when every measured joint remains within the hardware's
  two-count round-trip quantization band while desired motion reaches the
  controller path-error boundary does it replace the latent desired vector
  with current measured feedback. If any joint actually moved beyond that
  band, the existing desired-command hold remains in force so a partially
  executed path cannot create a second step.
- The actual-motion evidence limit is
  `4*pi/4096 = 0.0030679615757712823 rad`, derived from the driver's documented
  `4096` counts/revolution and independent command/feedback quantization. The
  desired-motion evidence limit is `0.12 rad`, exactly the same uniform
  per-joint path tolerance configured for Joint1 through Joint6. These are
  controller/hardware contracts and apply to all targets; no object position,
  observation pose, or paper-box constant was introduced.
- Added regression coverage for all three branches: a normal direct hold still
  republishes desired rather than feedback; proven zero actuation publishes
  actual feedback; and measured partial actuation preserves desired. The
  motion-gateway suite passed `27/27`, Python compilation passed, and
  `git diff --check` passed. A broader `48`-test MoveIt/configuration run has
  one pre-existing unrelated assertion about the count of the driver endpoint
  feedback-trim guard; it was not altered to hide this failure.
- The operator then reported the target aligned again. No new grasp trajectory
  was issued before the above implementation, tests, and documentation were
  complete. The next live step is to hot-load only the motion gateway, publish
  a positive enable request, and supervise one fresh task. The corrected branch
  must either show real measured motion or install the new no-actuation
  feedback hold at the first controller failure; it must not issue a second
  range-correction trajectory.

### 2026-07-28 - power-cycle restored actuation; near-field batch reached MuJoCo

- Restarted the latest full ROS launch after the operator power-cycled the
  arm. The launch requested only positive torque enable. At ROS time
  `1785306443.388907985` the driver logged
  `Disabling zero-torque mode with SDK torque_on frame.` No torque-off,
  demonstration/disable, task stop, controller stop, or emergency-stop command
  was sent.
- The operator explicitly authorized temporary calibration-interlock release.
  The fresh-plan runner bound exact far-field plan
  `d9a88d137495f43a7ad6003c`, verified its atomically committed same-plan audit,
  and started the task.
- The first real observation trajectory completed successfully. Measured arm
  feedback moved from approximately
  `[-108.8,14.9,1.4,-12.1,-22.9,14.9] deg` to
  `[-109.6,-20.0,25.1,-22.1,-17.4,22.7] deg`; the controller returned
  `SUCCEEDED`. The local Joint3 `0.02 rad/s` limit was active, the retimed
  duration was about `21.436 s`, and the measured endpoint error was about
  `0.0149 m / 2.63 deg`. A fresh camera observation then measured
  `0.1813 m`, inside the configured `[0.18,0.22] m` range. This proves that
  the earlier post-write zero-actuation blocker was absent after the power
  cycle.
- Near-field mode remained continuously active from `23:34:38` until the
  configured 150 s replan timeout. It did not fall back to far-field while the
  task was waiting. Near-field requests 61 and 62 supplied the required two
  disjoint-window stability hits.
- Request 62 exhaustively checked all 24 current hard-safe, deduplicated
  candidates. Four exact four-stage sequences passed strict MoveIt and all four
  reached MuJoCo. Two were rejected because closure ended without simultaneous
  two-sided finger/object contact; two were rejected because the preloaded
  grasp did not retain geometrically opposed two-sided contact. Thus these
  MoveIt successes were not lost or silently truncated.
- That complete near-field request took about `93.4 s`. After the first
  stability-only request, it left only about 41 s for the next exhaustive
  request, which was still in progress when the task correctly failed closed
  with `NEAR_FIELD_REPLAN_TIMEOUT`. No contact trajectory, gripper close, lift,
  disable, or stop command followed.
- The operator then stated they were manually moving the arm. From that point
  onward no ROS control topic, motion/gripper service, enable/disable command,
  task start, or candidate execution was used. The remaining work below was
  performed offline.

### 2026-07-28 - exact-sequence planning reuse removes duplicate near-field work

- The live strict-planning trace showed exact duplicate work inside request 62:
  Cartesian failure fractions appeared in pairs such as `0.941/0.941`,
  `0.399/0.399`, and `0.305/0.305`. The four reachable candidate rows produced
  only two unique canonical rich-plan IDs. The existing source-index dedupe
  therefore preserved different candidate lineages that materialized the same
  exact four poses, then unnecessarily repeated the same deterministic
  orientation resolver.
- Added a request-local exact-sequence planning cache. Its key binds request
  ID/generation/target epoch, every bit of all four pose positions and
  quaternions, frame IDs, joint names, and the exact live joint vector.
  Results are stored only when the joint vector is bit-for-bit unchanged before
  and after the planning call.
- Original strict-MoveIt failures are never cached, because repeated planning
  attempts may legitimately explore different paths. Only a structured strict
  success may be reused. The deterministic free-space resolver may cache a
  success or `MOVEIT_UNREACHABLE` only when the gateway message explicitly
  attests `policy=deterministic_geodesic_collision_ik`; timeouts, transport
  failures, unattested results, and other error codes are never cached.
- Reusing a resolver result does not reuse candidate geometry or simulation.
  Every candidate lineage still reruns its own request-frozen target cloud,
  OBB/support-plane, finger/palm, contact-overlap, and full rotation-sweep
  gates. Every MoveIt-reachable lineage still enters the existing MuJoCo
  selection gate. Cache eligibility, hit/store state, unchanged-joint proof,
  and an exact-key SHA-256 are serialized into the request audit.
- The motion gateway now appends the deterministic policy attestation to both
  successful and failed resolver responses after the resolver actually runs.
  A not-ready gateway response remains unattested and cannot enter the cache.
- Updated the persisted near-field wait budget from `150 s` to `300 s`. The
  latest uncached complete batch measured about `93 s`; 300 s budgets two full
  uncached batches plus stability collection and margin. This changes only how
  long the task waits for a fully gated contact plan. It does not relax any
  geometry, MoveIt, MuJoCo, joint, freshness, or execution requirement.
- Regression passed across the complete affected suites: streaming state
  machine `207/207`, remote candidate node `156/156`, and motion
  gateway/MoveIt planner `65/65`, total `428/428`. Focused cache/policy tests
  passed `6/6`; Python compilation, YAML parsing, and `git diff --check` also
  passed. None of these tests sent a hardware command.

### 2026-07-29 - powered run isolated a snapshot-lifetime scheduling defect

- After the operator aligned the target and explicitly released the temporary
  calibration interlock, exactly one positive
  `std_msgs/Bool(data=false)` enable request was delivered to the driver:
  `POSITIVE_ENABLE_PUBLISHED_ONCE connections=1`. No disable, torque-off,
  task-stop, controller-stop, or emergency-stop request was sent.
- Fresh far-field plan `a8cb374940e66b4ec47efd3b` executed its retimed
  `52.519 s` observation trajectory. The measured endpoint residual was about
  `14.6 mm / 2.28 deg`. A fresh camera measurement of `0.1708 m` was below the
  inclusive `[0.180,0.220] m` observation interval, so the task issued its one
  permitted measured radial correction of about `40.5 mm`.
- The correction physically executed: maximum measured joint change was
  `0.141126 rad` for a requested `0.141007 rad`, and the subsequent camera
  range was about `0.185 m`. The controller nevertheless returned
  `ABORTED: CONTROL_FAILED`; the task classified the result as
  partial/indeterminate and failed closed before near-field contact. Torque and
  controller hold remained enabled. No task stop or disable followed.
- A second fresh task reused the now-reached observation pose under plan
  `f2c4d5e92b63611699534bf5`; measured reuse error was zero and the camera
  range was `0.1886 m`. It entered near field at ROS time
  `1785309398.600`. The first request correctly reported
  `STABILITY_PENDING`; the next obtained `24/24` stable candidates, `32/32`
  current hard rechecks, and `8/32` strict MoveIt-reachable sequences.
- The remote `/health` contract advertised
  `digital_twin.max_snapshot_age_sec=120.0`. The first strict reachable
  sequence appeared about `88 s` after the source snapshot, but the old policy
  continued exhaustive evaluation of the remaining candidates. MuJoCo was not
  called until total age reached about `129.080622 s`, so it correctly rejected
  the selected candidate with `PLAN_STALE`. The task later reached its `300 s`
  near-field wait limit and failed closed with
  `NEAR_FIELD_REPLAN_TIMEOUT`; no contact approach, gripper close, lift,
  disable, or stop command was issued.
- This evidence isolates a scheduling/validity mismatch rather than a failed
  individual safety gate: reachable candidates existed while the snapshot was
  valid, but the client consumed the server's lifetime before invoking MuJoCo.
  Lowering collision, joint-limit, IK, geometry, stability, or simulation
  requirements would not address that defect.

### 2026-07-29 - snapshot-budgeted near-field MoveIt selection

- The client now binds its near-field MoveIt budget to the actual
  `digital_twin.max_snapshot_age_sec` returned by the remote health endpoint.
  A default `30.0 s` reserve stops starting new strict MoveIt evaluations before
  that server deadline, leaving the configured `20.0 s` MuJoCo request timeout
  plus scheduling margin.
- The continuation gate runs immediately before each expensive strict MoveIt
  check. It never interrupts an in-flight check and never changes a candidate's
  geometry, collision, joint-limit, IK, planning, stability, or MuJoCo result.
  Strictly reachable candidates already found remain eligible for MuJoCo;
  the unvisited tail is explicitly audited as unchecked rather than rejected.
  An invalid timestamp, server lifetime, reserve, or current clock fails closed
  before another check.
- Streaming audits now report policy
  `SNAPSHOT_BUDGETED_CONTACT_SEQUENCE_FINAL_SCORE`, exact checked/reachable
  counts, `terminated_early`, and
  `MUJOCO_SNAPSHOT_RESERVE_REACHED` when the reserve is consumed. If no
  candidate has passed before the deadline, that reason is propagated instead
  of being mislabeled as universal MoveIt unreachability.
- Offline regression passed: candidate pipeline `181/181`, streaming state
  machine `213/213`, remote candidate node `156/156`, and persisted
  configuration `6/6`, total `556/556`. Python compilation and
  `git diff --check` also passed.
- The operator powered the arm off before this offline repair. No ROS hardware
  command of any kind was published after that instruction. The next powered
  run must hot-load the updated remote 6D node, send only the explicitly
  authorized positive enable request, and start one fresh task from the current
  aligned view. Expected evidence is either a MuJoCo decision before the
  120-second snapshot limit or a fail-closed
  `MUJOCO_SNAPSHOT_RESERVE_REACHED` followed by a fresh request.

### 2026-07-29 - powered validation reached current MuJoCo; task deadline was too short

- When power returned, the still-running old driver immediately resumed its
  retained pre-power-cycle SDK target and moved the manually aligned arm toward
  that old pose. This was latent command-stream state, not a new trajectory,
  stop, disable, or torque command issued by the assistant.
- Hot-loaded only the updated remote 6D node. At ROS time
  `1785310820.428749`, exactly one explicitly authorized positive
  `std_msgs/Bool(data=false)` request reached the driver, which logged
  `Disabling zero-torque mode with SDK torque_on frame.` No opposite
  demonstration request, torque-off frame, `/grasp/stop`, controller stop, or
  emergency stop was sent.
- The first fresh task bound plan `0637b68608733e21f0d6aa27`. Its retimed
  `46.382 s` far-field observation trajectory physically completed and reduced
  measured camera range from about `0.333 m` to `0.173 m`; measured endpoint
  residual was about `14.9 mm / 2.24 deg`. The one permitted measured radial
  correction of about `38.06 mm` also physically executed and ended near
  `0.185 m`, but the controller aborted on Joint2 goal error `0.040717 rad`.
  Measured and desired joint deltas were respectively `0.138058 rad` and
  `0.136878 rad`, so the task correctly classified partial/indeterminate
  execution and failed closed before near-field planning.
- A second fresh task bound plan `732b5bc211bdd0d9132ad72a`, reused the reached
  observation pose, and entered near field at ROS time
  `1785311064.423573`. Request 29 had 38 current hard-safe candidates; it
  strictly checked 17, found none reachable, and stopped at the new snapshot
  reserve after about `89.676 s`. Request 39 similarly checked 16 of 36 and
  stopped after about `90.664 s`. The audit correctly retained the unchecked
  tails, but the published request status was incorrectly overwritten as
  `MOVEIT_UNREACHABLE:17` and `MOVEIT_UNREACHABLE:16`.
- The next processed batch, request 49, found four strict-MoveIt-reachable
  sequences while the snapshot was still eligible for MuJoCo. Its first two
  MuJoCo candidates were rejected fail-closed as `MUJOCO_IK_FAILED` when lift
  IK failed around alpha `0.7204`. The task-wide `300 s` near-field deadline
  then expired before the remaining two reachable candidates could be
  simulated. The task ended `NEAR_FIELD_REPLAN_TIMEOUT` and released execution
  authority. No contact approach, gripper close, lift, disable, or stop command
  ran.
- This validates the snapshot-lifetime repair: strict reachable candidates now
  enter MuJoCo before the server's `120 s` snapshot limit. It also supplies
  direct timing evidence for the remaining independent defect. Two legitimate
  approximately 90-second all-unreachable batches can precede a third batch
  that first finds reachable candidates at task age about 294 seconds, after
  which the configured MuJoCo selector still needs up to 80 seconds.
- Later hardware telemetry changed materially from the earlier 44-50 C trace:
  status remained `0xE1` while plausible temperature channels persisted around
  60-61 C for hundreds of samples. Separate impossible 164/165 C and 226 C
  fields coincided with impossible joint raw values and were rejected by the
  parser/command-consistency filters. Because the plausible sustained trace was
  new evidence, no third motion task was started. The operator then powered the
  arm off. The driver did not autonomously torque off, consistent with the
  explicit operator-only torque policy, and the assistant sent no hardware
  command after the power-off instruction.

### 2026-07-29 - offline deadline/status repair and release verification

- The final streaming status now replaces
  `NO_REACHABLE_STABLE_CANDIDATE` with an aggregate primary rejection count
  only while the result is otherwise unclassified. A precise early termination
  such as `MUJOCO_SNAPSHOT_RESERVE_REACHED`, or a precise MuJoCo terminal
  status, remains authoritative. This prevents a checked unreachable subset
  from being presented as evidence that its explicitly unchecked tail was also
  unreachable.
- Added an end-to-end streaming regression reproducing the live 17-of-20
  shape: 17 checked candidates may all contribute
  `MOVEIT_UNREACHABLE`, but an early snapshot-reserve stop must remain the
  published terminal state while the funnel keeps the exact rejection count.
- Increased the persisted task-wide near-field wait horizon from `300 s` to
  `450 s`. The value covers the live third-batch eligibility at about 294
  seconds, the complete configured 80-second MuJoCo selection budget, and
  stability/scheduling margin. This only changes how long the task waits for a
  fully gated contact plan. It does not relax geometry, collision, joint,
  strict-MoveIt, stability, freshness, MuJoCo, or execution checks.
- Synchronized two stale configuration assertions with already documented
  production behavior: remote result freshness is `15 s`, while
  far-field same-pose endpoint correction attempts remain `0` because retained
  real execution evidence showed that retry could worsen the endpoint. Added
  generated catkin metadata and raw ROS console captures to ignore rules;
  curated Markdown evidence remains versioned.
- Complete offline verification passed with `1695 passed, 3 skipped`. Two
  skips explicitly identify the absent genuine
  `tests/fixtures/carton_tabletop_cloud.json` RealSense capture; the tests
  automatically run when that real fixture is present, and no synthetic data
  was substituted. The two HTTP protocol files separately passed
  `135 passed, 1 skipped` with local loopback sockets enabled. Focused affected
  suites passed `214/214` streaming, `337/337` pipeline/remote node,
  `138/138` task/config, `72/72` driver/motion planning, and `40/40`
  configuration/GUI tests. All Python files compiled, 28 YAML files and 42
  launch XML files parsed, `git diff --check` passed, and full-workspace
  `catkin_make -j2` rebuilt the C++ driver, new services, messages, and nodes
  successfully. No ROS node or hardware interface was started by any offline
  verification command.

### 2026-07-29 - GUI command-path proof and unconfirmed physical actuation

- The GUI slider target reached the driver without being overwritten by the
  grasp task or controller hold. Joint2's SDK target changed from about
  `-14.9 deg` to the requested `-28.8 deg`, and the gripper target changed
  from `995` to `1000`, while fresh encoder feedback stayed near
  `-16.8 deg`. This reproduces the GUI no-motion symptom below the GUI and ROS
  command layers: the command was streamed, but the actuator response remained
  zero.
- Exactly one additional explicitly authorized positive
  `std_msgs/Bool(data=false)` request reached the driver at ROS time
  `1785316280.782145890`. The driver logged
  `Disabling zero-torque mode with SDK torque_on frame.` No
  `/demonstration=true`, torque-off, `/grasp/stop`, controller stop, disable,
  or emergency-stop request was sent. Encoder feedback still showed no
  qualifying motion after the positive frame.
- `/alicia_d/motion_enabled=true` did not prove torque acceptance. Source
  inspection showed that startup, reconnect, and the demonstration callback
  publish `true` immediately after or even before writing the torque-on frame;
  the protocol provides no firmware torque acknowledgement. The existing topic
  therefore described a software request rather than measured actuation.
- The live driver reported run status `0xE1`. Earlier samples around
  `55-58 C` contained isolated ambiguous spikes, but the same plausible
  temperature channel later remained around `60-63 C` for at least three
  consecutive fresh frames while `0xE1` persisted. This is materially
  different from the earlier task's `44-50 C` trace and is consistent with the
  firmware refusing positive torque under sustained temperature protection.
  It does not retroactively explain the earlier zero-response trajectory.
- The operator used the GUI's `同步当前关节` action before power-off. The
  driver's target returned from about `-28.8 deg` to about `-15.2 deg`, close
  to the `-16.8 deg` measured feedback, so the large stale GUI target was no
  longer pending. The operator then powered the arm off. No hardware command
  was sent after that instruction.

### 2026-07-29 - approved actuation-confirmation and direct near-field route

- The operator approved separating positive-enable request state from measured
  actuator confirmation. The driver will publish confirmed motion only after
  fresh encoder feedback moves measurably in the commanded direction. Startup
  and reconnect will discard pre-feedback commands and require a
  near-feedback synchronization command before admitting later motion. A
  sustained same-channel over-limit trace with `0xE1`/`0xE2` will report
  `OVERHEAT_BLOCKED` without sending torque-off, stop, or disable.
- The latest second-stage task reached near field and found strict-MoveIt
  reachable sequences, including batches with four and eight reachable
  candidates. MuJoCo then rejected those candidates as `IK_FAILED` or
  `CONTACT_FAILED`, and the task waited until the configured `450 s`
  `NEAR_FIELD_REPLAN_TIMEOUT`. This proves the target was not generally outside
  the MoveIt-reachable workspace; the production path had accumulated
  cross-request stability, exhaustive planning, per-candidate MuJoCo
  selection, duplicate task simulation, and a later final visual-refine wait
  beyond the intended second-stage behavior.
- The approved production route now uses one fused near-field RGB-D snapshot
  as the second visual correction, deterministically ranks current hard-safe
  candidates, and stops at the first complete sequence that passes strict
  MoveIt. Its snapshot-and-selection budget is `30.0 s`. It preserves geometry,
  support, collision, joint-limit, freshness, plan-ID, endpoint, and controller
  gates while removing cross-request stability, near-field MuJoCo execution
  authority, duplicate task simulation, and post-rebind final visual refine.
- Added the canonical route document
  `src/alicia_flexible_grasp_supervisor/docs/grasp_task_technical_route.md`.
  Its body always describes the current effective proof route and every route
  change is appended there with a date. Individual changes, problems,
  evidence, fixes, and verification results remain in this runtime log.
- At the time of this entry the design and route were approved but the new
  behavior had not yet been implemented or powered-validated. All work in this
  phase remained offline while the arm was powered off.

### 2026-07-29 - offline implementation progress before operator-requested pause

- Implemented the protocol-independent measured-actuation state machine and
  integrated it into the real-arm driver. Positive torque-on now enters
  `PENDING`; only a fresh, correctly directed encoder response to a later
  non-trivial streamed target can enter `CONFIRMED`. Zero response, opposite
  response, unrelated-joint motion, stale confirmation, write failure, and
  sustained same-channel temperature protection have explicit fail-closed
  states. Startup/reconnect also clear retained targets and require fresh
  feedback plus near-feedback synchronization before later commands are
  admitted. No automatic torque-off, stop, disable, or controller-stop path
  was added.
- Added `/alicia_d/actuation_status` and changed
  `/alicia_d/motion_enabled` to represent fresh measured confirmation rather
  than a successful software request. Automatic grasp now requires a locally
  fresh `CONFIRMED` status. The C++ state-machine suite passed `11/11`, the
  existing serial-driver resilience suite passed `7/7`, the affected task
  suite passed `134/134` at that checkpoint, configuration tests passed
  `6/6`, and the complete catkin workspace build succeeded. These changes are
  recorded in commits `6e511ca`, `189f34c`, and `dcf1710`.
- Persisted the approved direct production policy:
  `near_field_strategy=single_snapshot_direct`, a `30.0 s` near-field budget,
  and both redundant final-visual-refine flags disabled. This is commit
  `383cfd4`.
- Implemented the remote direct near-field selector. The current fused request
  is adapted to a one-hit stable-candidate contract for downstream hard
  rechecking; retained tracker hits remain diagnostic only. Current hard-safe
  candidates are deterministically ranked, every current candidate remains
  eligible within the budget, and the first strict-MoveIt-reachable sequence
  is selected. Direct mode does not call the MuJoCo selector. Exact terminal
  states now include `NEAR_FIELD_NO_HARD_SAFE_CANDIDATE`,
  `NEAR_FIELD_NO_REACHABLE_CANDIDATE`, and
  `NEAR_FIELD_DIRECT_TIMEOUT`.
- Remote selection first reproduced three expected failures against the old
  implementation, then passed all five focused direct-mode tests. The full
  streaming suite passed `217/217` before the final two terminal-status cases
  were added; those two and the other three direct cases subsequently passed
  together as `5/5`. The remote-node suite passed `156/156`, Python compilation
  and `git diff --check` passed, and the remote implementation is commit
  `0e1e6b1`.
- Implemented, but have not yet committed, the task-layer direct path. It
  requests one near-field stream, freezes the accepted plan, immediately
  requests stream disable, skips duplicate task-level simulation, skips the
  post-rebind final visual-refine layer, and retains the existing execution
  checkpoints and motion order: near-field pregrasp, linear approach, linear
  grasp pose, gripper close, and linear lift. Direct timeout is reported as
  `NEAR_FIELD_DIRECT_TIMEOUT`; explicit legacy mode retains the former
  simulation/refinement behavior.
- The three new task contracts first failed for the expected old behaviors,
  then passed after implementation. The complete task sequence suite passed
  `137/137`, including legacy MuJoCo and final-refinement regressions.
- Per the operator's instruction, work is paused at this exact point. The task
  node and its new tests, plus this log entry, remain uncommitted. Remaining
  offline work is source-format review, complete affected verification,
  updating the canonical route status/changelog, committing documentation and
  task changes, and pushing the authorized branch. Powered validation has not
  begun.
- No ROS node, ROS master, hardware interface, serial write, torque request,
  motion request, stop, disable, or other hardware command was started or sent
  during any work recorded in this entry.

### 2026-07-29 - offline work resumed with all hardware ports and arm power off

- The operator explicitly confirmed that all hardware serial ports and arm
  power were off before work resumed. This phase used only source inspection,
  compilation, local unit/protocol tests, configuration parsing, Git, and
  documentation.
- Source review found one remaining mismatch between the approved route and
  the implementation. `/grasp_6d/request_plan=true` starts continuous polling;
  although direct selection used only the current request, the poll loop could
  submit another newer fused near-field snapshot after the first request
  failed. A new regression supplied two successively newer fused snapshots and
  first failed because both were submitted.
- Direct near-field polling now records the stream generation that consumed
  its first valid fused snapshot. Further polls in the same near-field
  generation return before collecting another window. The latch resets only
  at a planning-phase boundary or a new/terminated stream generation. Failed
  fusion or a snapshot rejected before submission does not consume the latch.
  The focused direct suite then passed `6/6`, including exact one-snapshot
  submission.
- The task node now freezes the accepted direct plan, requests candidate-stream
  disable, skips duplicate task-level MuJoCo simulation and final visual
  refinement, and keeps the existing ordered execution checkpoints for
  near-field pregrasp, linear approach, linear grasp pose, gripper close, and
  linear lift. Direct no-preview expiry remains
  `NEAR_FIELD_DIRECT_TIMEOUT`; explicit legacy mode retains its prior gates.
  These task/one-shot changes and their tests are commit `c6aa55b`.
- Complete affected suites passed: remote streaming `220/220`, remote node
  `156/156`, task sequence `137/137`, and combined persisted-config plus serial
  resilience `13/13`. The complete Python suite initially reported
  `1687 passed, 16 failed, 3 skipped`; every failure was the sandbox denying a
  temporary `127.0.0.1` socket with `PermissionError`. Re-running the two
  protocol files with local-loopback permission passed `135/135` with one
  declared skip, making the equivalent complete result
  `1703 passed, 3 skipped`.
- Two skips require the absent genuine
  `tests/fixtures/carton_tabletop_cloud.json` RealSense capture and explicitly
  forbid synthetic substitution. The third is the opt-in real MuJoCo/mesh
  smoke test guarded by `MUJOCO_SMOKE=1`; neither skip was treated as passing
  real evidence.
- Full `catkin_make -DCATKIN_ENABLE_TESTING=ON -j2` completed successfully.
  The actuation-confirmation gtest passed `11/11`; both modified Python scripts
  compiled; 28 YAML files and 42 launch XML files parsed; and
  `git diff --check` was silent.
- No ROS node or ROS master was started. No serial device, camera, tactile
  device, actuator, torque service, motion service, stop, disable, or other
  hardware command was accessed or sent. Powered validation remains pending
  and no true hardware-response or successful-grasp claim is made here.

### 2026-07-29 - powered direct-route validation and first-stage radial-correction failure

- The latest full system was launched from commit `0435746` with the real arm
  on `/dev/alicia_arm` at `1000000` baud, the real camera, GUI, and remote
  protocol-v3 6D service at `http://172.23.132.97:8000`; tactile was disabled.
  Startup requested only the existing positive torque-on path. No
  `/grasp/stop`, torque-off, disable, controller stop, or emergency-stop
  request was sent.
- After the operator synchronized current joints and made a GUI motion, fresh
  encoder feedback established
  `CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE`. Temperatures during task setup
  were approximately `26-36 C`; status was predominantly `0x00`. Isolated
  `0xE1` events were logged without any automatic torque-off because measured
  temperature did not establish sustained over-temperature.
- The operator aligned a detected `carton`. One representative perception
  message reported confidence `0.9163`, depth `0.3179 m`, and base-frame
  position approximately `(-0.1051, -0.4652, 0.0612) m`.
- The operator explicitly authorized sending the task RGB-D/point-cloud
  payload to `http://172.23.132.97:8000`. The remote service accepted
  `/grasp_6d/request_plan trigger:true`. A representative plan used five fused
  frames, `3311` valid depth points, depth-valid ratio about `0.9991`, and a
  strict-MoveIt-reachable tabletop candidate.
- Continuous inference legitimately changed execution authority while the
  first start request was being prepared. The start service rejected stale
  plan `fe8de3312cbd9581f8869f74` with
  `PLAN_ID_MISMATCH` and reported current rich plan
  `898d248c414b43d7648babe8`; no motion ran for that rejected request.
  Source documentation confirmed that
  `/grasp_6d/request_plan trigger:false` only stops new candidate inference
  and retains Execution readiness. After freezing the stream, the retained
  execution plan was `d94be0261d64908dd67e2f38`.
- `/grasp/start` accepted the frozen plan. The `46.5 s` far-field observation
  trajectory physically moved the arm from approximately
  `[-103.2, +19.9, -10.9, +0.4, -1.1, +0.4] deg` to the observation region.
  The measured endpoint residual was `0.0134 m / 2.10 deg`; measured
  camera-target range was `0.173114 m`, so the existing one permitted radial
  correction planned a `0.036761 m` retreat while preserving measured tool
  orientation.
- The radial-correction trajectory also produced real encoder motion, but
  MoveIt received
  `GOAL_TOLERANCE_VIOLATED: Joint2 goal error 0.037171` and
  `ABORTED: CONTROL_FAILED`. The motion gateway installed its existing
  enabled-controller desired-command hold. Measured and desired maximum joint
  deltas were `0.144194 rad` and `0.143758 rad`, but the steady Joint2
  endpoint error remained `0.037171 rad`, above the configured
  `execution_goal_tolerance_rad + slack = 0.035 rad`. The task therefore
  correctly failed before opening a near-field snapshot and released the
  execution slot. Actuation remained
  `CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE`; no stop or disable command ran.
- This run does not prove the second-stage selector failed and does not prove
  the target was unreachable: the task never entered near field. It exposes a
  narrower first-stage integration defect. The task already permits a strict
  observation trajectory whose controller reports failure to be judged by the
  unchanged live camera-range contract after feedback settles, but the single
  radial-correction call did not opt into that same bounded recovery path.
- The operator approved the evidence-based repair. A controller-failed
  observation or radial correction may continue only after motion settles and
  a fresh observation proves the same target is inside the existing camera
  range contract. Joint endpoint tolerance remains `0.035 rad`; driver
  endpoint trim remains disabled; no second correction is added; and no
  contact, close, or lift failure can use this observation-only recovery.
- At the time of this entry, the route change is documented but production
  code is unchanged. The arm remains powered and enabled at the failed
  correction endpoint. No further motion, candidate generation, stop,
  disable, torque-off, or emergency command was sent while documenting the
  evidence.

### 2026-07-29 - offline radial-correction observation-contract repair

- The approved implementation remained scoped to the existing observation-only
  recovery argument. No controller, driver, range, timing, correction-count,
  contact-stage, or persisted configuration value changed.
- The first baseline test invocation omitted `source devel/setup.bash` and did
  not enter the suite because ROS-generated Python messages were not on
  `PYTHONPATH`. Re-running in the worktree environment passed the unchanged
  task sequence suite `137/137`.
- The focused regression changed only the radial-correction caller expectation
  to require `allow_post_failure_observation_validation=True`. Before the
  production change, it failed exactly with `AssertionError: False is not
  true`, proving the old caller still denied the approved recovery.
- Production commit `783d1a2` changes the radial-correction call from
  `allow_post_failure_observation_validation=False` to `True`. The existing
  callee still requires a frozen `FAR_FIELD_OBSERVATION_PLAN`, recognizes only
  the exact cached strict-execution failure prefix, waits for motion settle,
  records the failure marker, and leaves the following fresh camera-range
  check authoritative.
- Three focused tests then passed: the radial correction opts into the
  observation contract, a recoverable far-field controller failure waits for
  feedback settle, and a failed main observation whose live range remains
  outside the contract cannot issue another physical correction.
- The complete task sequence suite passed `137/137`; both modified Python
  files compiled; `git diff --check` was silent. The scoped production/test
  diff contained exactly the two one-line boolean expectation changes.
- All work in this entry used local source, Git, Python import/compile, and
  offline unit tests. It did not call a ROS service, publish a ROS topic,
  access a serial device, generate a candidate, move a joint, or send stop,
  disable, torque-off, controller-stop, or emergency commands.

### 2026-07-29 - powered observation recovery proof and near-field deadline mismatch

- The updated task node replaced only the old `/grasp_task_node`; the new PID
  was `12878` and published `IDLE ready`. The driver, serial connection,
  controllers, MoveIt, camera, perception, remote 6D node, and ROS master were
  not restarted. Runtime calibration interlock remained `false`, actuation
  remained `CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE`, and the target remained
  a detected `carton` with confidence about `0.917`.
- Remote candidate generation produced two fresh valid far-field plans. New
  inference was then frozen using `/grasp_6d/request_plan trigger:false`,
  which does not stop the robot or clear Execution readiness. The retained
  plan `5499f45aa6c09ce328dce88e`, source stamp
  `1785388702.860067367`, was bound to the live task.
- The far-field observation trajectory physically completed. Its measured
  endpoint residual was `0.0139 m / 2.26 deg`. Fresh camera-target distance
  was `0.177538 m`, below the existing `0.180 m` minimum, so the task issued
  its one permitted measured radial correction, a `0.032766 m` retreat.
- MoveIt reported
  `GOAL_TOLERANCE_VIOLATED: Joint2 goal error 0.037051` and
  `ABORTED: CONTROL_FAILED`. The measured and desired maximum joint deltas
  were `0.122718 rad` and `0.123350 rad`. The updated task did not accept
  those deltas as endpoint success and did not change the `0.035 rad`
  hardware review limit. It recorded the controller failure and waited
  `0.90 s` for measured motion settle.
- A newer observation then measured the same target at `0.1899 m`, inside the
  unchanged inclusive `[0.1800, 0.2200] m` range. The task accepted the live
  observation contract and switched `/grasp/near_field_active` to `true` at
  ROS time `1785388824.067`. This is the first powered proof that commit
  `783d1a2` fixes the radial-correction false terminal without relaxing joint
  tolerance.
- The remote node received the near-field phase change and cancelled prior
  phase requests. Its one direct near-field request used snapshot stamp
  `1785388826.249668`. The live gate audit began with 32 candidates and
  reported eight baseline-safe/visible candidates before strict MoveIt
  sequence checks.
- The task's outer 30-second wait began near
  `1785388824.069` and expired at `1785388854.110`. No new contact-phase
  Preview had arrived, so the last visible Preview was still the earlier
  far-field plan and the exact task terminal was
  `NEAR_FIELD_DIRECT_TIMEOUT`, with last validation reason
  `NEAR_FIELD_PLAN_PHASE_INVALID`. The task switched back to far field and
  released the execution slot.
- The near-field worker finished only at about `1785388859.556`. Its metrics
  reported request `151`, end-to-end time `32701.604 ms`, snapshot age
  `33297.446 ms`, ROS preparation `3926.325 ms`, and
  `drop_reason=GENERATION_STALE` because the task had already closed that
  generation. MoveIt logs show strict checks still completing after the
  task-side deadline, including an orientation-resolved approach result at
  about `1785388859.497`.
- Source inspection proves the timer mismatch. The task starts its
  `near_field_replan_timeout_sec` stopwatch before requesting the stream,
  while `_direct_near_field_deadline_gate` computes a second 30-second
  deadline from the later fused `snapshot_stamp_sec`. It checks only before
  starting another candidate; an in-flight strict MoveIt service call may
  finish after that deadline. Therefore the outer task can expire and
  invalidate the generation before the remote node publishes either a
  selected contact plan or its exact bounded terminal.
- This run is not evidence that the target was outside the robot workspace or
  that all eight current hard-safe candidates were unreachable. The stale
  result never committed its near-field funnel, and at least one strict check
  was still active after task expiry. No near-field pregrasp, contact
  approach, grasp pose, gripper close, lift, `/grasp/stop`, torque-off,
  disable, controller-stop, or emergency command ran.
