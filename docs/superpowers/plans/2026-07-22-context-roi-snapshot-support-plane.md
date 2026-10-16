# Context ROI Same-Snapshot Support Plane Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the first continuous `context_roi` request use the valid support plane computed from its own frozen RGB-D snapshot.

**Architecture:** Convert the current `GeometryEstimate` support plane from base coordinates to ROS `camera_link` coordinates using the current frozen `T_base_optical`. Pass that request-local pair explicitly into the GraspNet input builder while retaining the mutable cache only for activated telemetry and legacy synchronous callers.

**Tech Stack:** ROS Noetic, Python 3, NumPy, unittest/pytest.

## Global Constraints

- Do not relax support-plane thresholds or switch away from `context_roi`.
- Do not use support-plane state from another snapshot in the continuous worker.
- Do not publish `/grasp/stop`, torque-off, disable, stop-motion, or physical execution commands during verification.
- Preserve stale-request cancellation and delayed geometry activation.

---

### Task 1: Bind Context ROI to Its Frozen Snapshot Plane

**Files:**
- Modify: `src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py:4362`
- Test: `src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_node.py:579`

**Interfaces:**
- Consumes: `GeometryEstimate`, frozen `T_base_optical`, `FrozenGraspNetInputConfig`.
- Produces: `_support_plane_camera_from_geometry(estimate, transform) -> (point, normal)` and optional request-local `support_plane_point_camera` / `support_plane_normal_camera` keyword arguments on `_build_frozen_graspnet_input`.

- [ ] **Step 1: Write the failing streaming preparation test**

Add a test that leaves `latest_support_plane_camera_*` empty, returns a valid
same-snapshot geometry estimate and identity transform, captures the keyword
arguments passed into `_build_frozen_graspnet_input`, and asserts the captured
normal is ROS `camera_link` `+X` rather than absent or stale.

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```bash
source /opt/ros/noetic/setup.bash
source /home/zhuyupei/alicia_wa_full/devel/setup.bash
python3 src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_node.py \
  RemoteGrasp6DNodeTest.test_prepare_uses_current_snapshot_support_plane_before_activation -v
```

Expected: FAIL because the builder receives no request-local support-plane
keyword arguments.

- [ ] **Step 3: Add the shared conversion helper**

Extract the existing support point/normal conversion from `_activate_geometry`
into `_support_plane_camera_from_geometry`. Validate the returned arrays are
finite three-vectors and normalize the camera-frame normal.

- [ ] **Step 4: Pass the plane through the continuous request**

When `input_config.requires_support_plane` is true, call the helper in
`_prepare_and_predict` and pass both arrays to `_build_frozen_graspnet_input`.
Extend the builder with optional keyword arguments; explicit values take
precedence over cache values. Keep the existing cache fallback for the legacy
synchronous caller.

- [ ] **Step 5: Reuse the helper during geometry activation**

Replace the duplicated conversion in `_activate_geometry` with the helper and
continue populating `latest_support_plane_camera_*` only after prepared geometry
is accepted.

- [ ] **Step 6: Verify GREEN and related regressions**

Run the focused test, all `context_roi` node tests, and the streaming suite.
Expected: all selected tests pass with no new failures.

- [ ] **Step 7: Reload ROS and perform Preview-only validation**

Restart the v3 ROS launch/node so the edited Python is loaded. Confirm a fresh
continuous request no longer reports the builder-level
`SUPPORT_PLANE_INVALID`, reaches WSL (`wsl_total_ms > 0` when inference
completes), and publishes stable Preview candidates. Leave physical execution
for the operator's next explicit step.
