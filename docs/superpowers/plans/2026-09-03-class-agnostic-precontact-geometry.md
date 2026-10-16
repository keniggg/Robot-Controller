# Class-Agnostic Precontact Geometry Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Complete the known-carton grasp without any class-specific downstream authority by replacing the temporary solid-OBB repair and clipped-centroid final refinement with measured multi-view surfaces, bounded 3D registration, and one collision-checked clear-view reacquisition.

**Architecture:** Keep the trained carton network only behind a common `TargetObservation` adapter. Bind all later state to an opaque `(epoch, track_id)` identity. Fuse reached far-field and near-field RGB-D target surfaces in base coordinates with per-point view provenance. Generate bilateral contacts only from measured opposing surfaces. Remote planning publishes structured registration evidence in the rich plan; the task node either accepts a strictly rechecked 3D correction or performs one duration-ranked, no-contact clear-view move and replans.

**Tech Stack:** ROS 1 Noetic, rospy, Python 3, NumPy, OpenCV, catkin messages, MoveIt, existing generic pretrained GraspNet service.

**Spec:** `docs/superpowers/specs/2026-09-03-known-to-unknown-class-agnostic-grasp-design.md`, sections “Architecture and authority boundary” through “Precontact visual refinement”.

## Global Constraints

- In phase 1, the trained carton model may produce the mask and diagnostic label only.
- Geometry, candidate construction/ranking, collision checks, strict planning, final refinement, close, lift, and verification must not branch on label, class index, model filename, or known-object lists.
- Do not fill unseen volume from an OBB. Bilateral contact requires measured support on both jaw sides.
- A clipped mask centroid cannot translate or rotate the plan. Valid interior depth points may participate in 3D registration.
- Every correction preserves target track, plan ID, candidate lineage, support plane, and grasp family and reruns the full strict pregrasp/approach/grasp/lift sequence.
- At most one clear-view reacquisition is allowed per task. It stays outside the contact envelope and is strictly planned before execution.
- Failure remains precontact and does not invoke stop, torque-off, disable, controller-stop, emergency, `/demonstration=true`, or `/grasp/stop` commands.
- No powered attempt occurs until all offline tests in this plan and the endpoint-trim plan pass.

---

### Task 1: Introduce the common observation contract and opaque track identity

**Files:**
- Create: `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/target_observation.py`
- Modify: `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/rgbd_snapshot.py`
- Modify: `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/grasp6d_stability.py`
- Modify: `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/grasp6d_pipeline.py`
- Modify: `src/alicia_flexible_grasp_supervisor/msg/ObjectGeometry.msg`
- Modify: `src/alicia_flexible_grasp_supervisor/msg/Grasp6DPlan.msg`
- Modify: `src/alicia_flexible_grasp_supervisor/msg/NearFieldPlanningPhase.msg`
- Modify: `src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py`
- Modify: `src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py`
- Modify: `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/rich_plan_integrity.py`
- Test: `src/alicia_flexible_grasp_supervisor/tests/test_rgbd_snapshot.py`
- Test: `src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_stability.py`
- Test: `src/alicia_flexible_grasp_supervisor/tests/test_rich_plan_candidate_source.py`
- Test: `src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py`
- Test: `src/alicia_flexible_grasp_supervisor/tests/test_grasp_task_sequence.py`

**Interfaces:**

```python
@dataclass(frozen=True)
class TargetTrackIdentity:
    epoch: int
    track_id: str

@dataclass(frozen=True)
class TargetObservation:
    identity: TargetTrackIdentity
    stamp_ns: int
    frame_id: str
    points_base: np.ndarray
    support_normal_base: np.ndarray
    support_offset_m: float
    bbox_xywh: tuple
    image_shape_hw: tuple
    edge_clearance_px: int
    source_kind: str
    source_label: str = ''
```

Add these ROS fields:

```text
# ObjectGeometry.msg
string target_track_id

# Grasp6DPlan.msg
string target_track_id
string refinement_status
uint32 refinement_inlier_count
float32 refinement_overlap_fraction
float32 refinement_rmse_m
float32 refinement_translation_m
float32 refinement_rotation_deg
bool refinement_source_clipped
uint32 fused_view_count

# NearFieldPlanningPhase.msg
string reference_target_track_id
```

- [ ] **Step 1: Write label-invariance and identity tests**

Use identical geometry with labels `carton`, `bottle`, and `''`. Assert the adapter creates the same opaque track ID for the same epoch/associated component, tracker decisions are identical, changing only the label does not advance the epoch, and changing component association beyond the existing `target_instance_association_threshold_m` does.

Update stability fixtures from `(epoch, label, model_choice)` to `TargetTrackIdentity(epoch, track_id)`. Keep label and model choice as diagnostics on candidates, but remove them from equality gates.

- [ ] **Step 2: Write rich-message integrity tests**

Assert that target track ID and every refinement-authority field changes `compute_plan_id()`. Diagnostic source label changes must not change geometric decisions, although the complete published message may still display it.

- [ ] **Step 3: Run focused tests and verify RED**

```bash
python3 -m pytest -q src/alicia_flexible_grasp_supervisor/tests/test_rgbd_snapshot.py src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_stability.py src/alicia_flexible_grasp_supervisor/tests/test_rich_plan_candidate_source.py src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py src/alicia_flexible_grasp_supervisor/tests/test_grasp_task_sequence.py
```

Expected: failures show the current triple identity, `reference_label` authority, and missing message fields.

- [ ] **Step 4: Implement and propagate the observation contract**

Build `track_id` as a non-semantic string scoped by the current stream generation and association epoch, for example `g{generation}-t{epoch}`. `_current_stream_target_identity()` returns only `TargetTrackIdentity`. Remove label from `identity_changed`, `_validated_target_identity`, near-field anchor admission, candidate stability matching, and safety equality. The near-field phase validates `reference_target_track_id` plus source stamp and geometry center.

Populate `TargetObservation` only after RGB/depth/mask dimensions, timestamps, TF, support plane, finite points, and edge clearance are known. `source_kind` and `source_label` are audit-only.

- [ ] **Step 5: Update canonical plan bytes and rebuild messages**

Pack `target_track_id` with a length prefix and pack refinement metrics at ROS float32 precision in `canonical_plan_bytes()`. Reject non-finite metrics and inconsistent states (for example `refinement_status=VALID` with zero inliers).

```bash
source /opt/ros/noetic/setup.bash
catkin_make -j2
```

- [ ] **Step 6: Run focused tests and commit**

Run Step 3 again. Expected: all pass.

```bash
git add src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/target_observation.py src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/rgbd_snapshot.py src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/grasp6d_stability.py src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/grasp6d_pipeline.py src/alicia_flexible_grasp_supervisor/msg/ObjectGeometry.msg src/alicia_flexible_grasp_supervisor/msg/Grasp6DPlan.msg src/alicia_flexible_grasp_supervisor/msg/NearFieldPlanningPhase.msg src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/rich_plan_integrity.py src/alicia_flexible_grasp_supervisor/tests/test_rgbd_snapshot.py src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_stability.py src/alicia_flexible_grasp_supervisor/tests/test_rich_plan_candidate_source.py src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py src/alicia_flexible_grasp_supervisor/tests/test_grasp_task_sequence.py
git commit -m "refactor: bind grasp plans to opaque target tracks"
```

---

### Task 2: Fuse registered measured surfaces with view provenance

**Files:**
- Create: `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/multiview_surface.py`
- Create: `src/alicia_flexible_grasp_supervisor/tests/test_multiview_surface.py`
- Modify: `src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py`
- Modify: `src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_graspnet_input_default_config.py`

**Interfaces:**

```python
@dataclass(frozen=True)
class SurfaceView:
    identity: TargetTrackIdentity
    stamp_ns: int
    points_base: np.ndarray
    support_normal_base: np.ndarray
    support_offset_m: float
    edge_clearance_px: int

@dataclass(frozen=True)
class RegistrationConfig:
    correspondence_max_m: float = 0.008
    minimum_inliers: int = 80
    minimum_overlap_fraction: float = 0.30
    maximum_rmse_m: float = 0.004
    maximum_translation_m: float = 0.025
    maximum_yaw_deg: float = 10.0
    maximum_support_normal_angle_deg: float = 4.0
    maximum_support_offset_delta_m: float = 0.004
    maximum_iterations: int = 12

@dataclass(frozen=True)
class RegistrationResult:
    ok: bool
    code: str
    transform_base: np.ndarray
    inlier_count: int
    overlap_fraction: float
    rmse_m: float
    support_normal_angle_deg: float

@dataclass(frozen=True)
class FusedTargetSurface:
    identity: TargetTrackIdentity
    points_base: np.ndarray
    view_indices: np.ndarray
    view_stamps_ns: tuple
```

- [ ] **Step 1: Write synthetic partial-view registration tests**

Construct a generic rectangular prism surface without any semantic label. Use a top/front reference view and a translated/yawed top/side partial view. Assert successful bounded registration, deterministic results, preserved per-point view indices, voxel deduplication, and immutable arrays.

Also assert failures for mixed identity, stale/non-monotonic timestamps, support-normal disagreement, support-offset disagreement, fewer than 80 inliers, overlap below `0.30`, RMSE above `0.004 m`, and corrections outside the translation/yaw bounds.

- [ ] **Step 2: Reproduce the bottom-clipped observation**

Create a `640x480` observation with bbox `(232,364,115,116)` and only valid interior depth points. Assert the view is marked clipped but registration is decided from the 3D evidence; no bbox centroid enters `transform_base`.

- [ ] **Step 3: Run tests and verify RED**

```bash
python3 -m pytest -q src/alicia_flexible_grasp_supervisor/tests/test_multiview_surface.py src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py
```

- [ ] **Step 4: Implement bounded pure-NumPy registration**

Use support-plane-aligned coordinates, deterministic point ordering, chunked nearest-neighbor distances, trimmed correspondences, and SVD rigid updates constrained to support-normal yaw plus bounded translation. Do not add SciPy/Open3D as a runtime dependency. Fuse only inlier points from accepted views and store the integer source-view index beside every point.

- [ ] **Step 5: Integrate reached-view and near-field fusion**

Replace `_near_field_geometry_anchor` with a per-phase `FusedTargetSurface`. Capture the reached far-field view when the task publishes the reference track/stamp, register each accepted near-field view, and reset the surface on phase, generation, or track change. Publish registration metrics in `Grasp6DPlan` and audit source stamps/view indices.

Add the exact defaults from `RegistrationConfig` beneath `grasp_6d/remote/multiview`. Config validation must fail closed rather than silently widening a bound.

- [ ] **Step 6: Run tests and commit**

```bash
python3 -m pytest -q src/alicia_flexible_grasp_supervisor/tests/test_multiview_surface.py src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py src/alicia_flexible_grasp_supervisor/tests/test_graspnet_input_default_config.py
```

```bash
git add src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/multiview_surface.py src/alicia_flexible_grasp_supervisor/tests/test_multiview_surface.py src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py src/alicia_flexible_grasp_supervisor/tests/test_graspnet_input_default_config.py
git commit -m "feat: fuse registered target surface views"
```

---

### Task 3: Require measured bilateral jaw support and delete the carton branch

**Files:**
- Modify: `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/gripper_geometry.py`
- Modify: `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/tabletop_geometry_candidates.py`
- Modify: `src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_gripper_geometry.py`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_tabletop_geometry_candidates.py`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py`

**Interface:**

```python
@dataclass(frozen=True)
class BilateralSurfaceEvidence:
    ok: bool
    code: str
    negative_jaw_points: int
    positive_jaw_points: int
    negative_view_count: int
    positive_view_count: int
    measured_width_m: float
    contact_height_m: float

def evaluate_bilateral_surface_evidence(
    fused_surface, contact_center_base, jaw_axis_base,
    insertion_axis_base, finger_geometry, minimum_points_per_side=12
) -> BilateralSurfaceEvidence
```

- [ ] **Step 1: Write failing class-invariance tests**

Run the same fused points through labels `carton`, `canton`, `unknown`, and `''`; require byte-equivalent evidence and candidate transforms. Assert that a top-only view returns `BILATERAL_SURFACE_EVIDENCE_MISSING`, two registered opposing views pass, and an OBB around the same top-only points cannot make it pass.

- [ ] **Step 2: Implement measured contact bands**

Project fused points onto jaw, insertion, and support-normal axes. Count physical finger-band points near each measured jaw extreme, require at least 12 per side, derive contact height only from their robust quantiles, and preserve contributing view provenance in audit data.

- [ ] **Step 3: Delete semantic execution authority**

Remove `_resolve_tabletop_contact_height` paths named `support_anchored_solid_obb_carton` and every `target_label` condition that changes contact height or candidate validity. Retain the old run values only as label-randomized regression fixtures.

- [ ] **Step 4: Run tests and scan for forbidden branches**

```bash
python3 -m pytest -q src/alicia_flexible_grasp_supervisor/tests/test_gripper_geometry.py src/alicia_flexible_grasp_supervisor/tests/test_tabletop_geometry_candidates.py src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py
rg -n "label.*carton|carton.*label|solid_obb_carton" src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp
```

Expected: tests pass; the scan returns no class-specific execution branch.

- [ ] **Step 5: Commit measured bilateral evidence**

```bash
git add src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/gripper_geometry.py src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/tabletop_geometry_candidates.py src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py src/alicia_flexible_grasp_supervisor/tests/test_gripper_geometry.py src/alicia_flexible_grasp_supervisor/tests/test_tabletop_geometry_candidates.py src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py
git commit -m "fix: require measured bilateral grasp surfaces"
```

---

### Task 4: Replace centroid refinement with structured 3D evidence

**Files:**
- Modify: `src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py`
- Modify: `src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py`
- Modify: `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/rich_plan_integrity.py`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_grasp_task_sequence.py`
- Modify: `src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml`

- [ ] **Step 1: Convert the live clipped failure into tests**

Replace the current expectation that bbox edge clearance `0 px < 4 px` always times out. Assert:

- clipped + sufficient 3D registration produces `refinement_status=VALID_3D`;
- clipped + insufficient 3D evidence produces `CLEAR_VIEW_REQUIRED` and no pose correction;
- target-track mismatch, stale stamp, support disagreement, low overlap/inliers, or excessive RMSE produces `FINAL_REFINE_3D_INVALID`;
- no path calls `_copy_final_center_sample()` or derives translation from a 2D centroid.

- [ ] **Step 2: Publish plan-bound registration evidence**

Remote planning applies an accepted registration transform to the chosen candidate family and publishes all structured metrics. Task-side `_maybe_final_refine_grasp6d_plan()` accepts only the same track ID and lineage, validates the metrics, recomputes the plan ID, and calls the existing full strict-sequence service for pregrasp, approach, grasp, and lift before motion.

- [ ] **Step 3: Remove center fallback as execution authority**

Set `final_visual_refine_center_fallback_enabled: false`, remove the center-fallback correction branch, and retain bbox clearance only as `refinement_source_clipped` audit evidence. Post-move confirmation also consumes 3D registration residual rather than mask-centroid jitter.

- [ ] **Step 4: Run refinement tests**

```bash
python3 -m pytest -q src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py src/alicia_flexible_grasp_supervisor/tests/test_grasp_task_sequence.py src/alicia_flexible_grasp_supervisor/tests/test_rich_plan_candidate_source.py
```

- [ ] **Step 5: Commit 3D refinement**

```bash
git add src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/rich_plan_integrity.py src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py src/alicia_flexible_grasp_supervisor/tests/test_grasp_task_sequence.py src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml
git commit -m "fix: gate final grasp refinement on 3D registration"
```

---

### Task 5: Add one duration-ranked no-contact clear-view reacquisition

**Files:**
- Modify: `src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py`
- Modify: `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/moveit_planner.py`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_grasp_task_sequence.py`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_moveit_planner_pose_feedback.py`
- Modify: `src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml`

**Interface:**

```python
def make_clear_view_reacquisition_poses(
    current_pose, target_center_base, support_normal_base,
    lateral_offset_m=0.060, radial_retreat_m=0.040,
    minimum_contact_clearance_m=0.080
) -> tuple
```

- [ ] **Step 1: Write bounded-attempt tests**

For `CLEAR_VIEW_REQUIRED`, assert two symmetric observation candidates are generated outside the contact envelope, each gets one strict MoveIt preflight, and only reachable candidates remain. Selection uses `joint_duration_lower_bound_sec`, then path cost, then a deterministic side preference. Exactly one candidate may execute and the per-task attempt counter prevents a second move.

Assert that no reachable candidate returns `CLEAR_VIEW_REACQUISITION_FAILED` before contact and that a successful observation must rebuild/rebind/recheck the complete plan before returning to pregrasp.

- [ ] **Step 2: Implement and integrate the observation detour**

Orient the camera optical axis toward the fixed target center while translating along the support-plane tangent by `+/- lateral_offset_m` and radially out by `radial_retreat_m`. Reject poses whose camera/tool envelope enters `minimum_contact_clearance_m`. Reuse the existing planner duration metric; do not create object-class waypoints.

- [ ] **Step 3: Run sequence tests**

```bash
python3 -m pytest -q src/alicia_flexible_grasp_supervisor/tests/test_grasp_task_sequence.py src/alicia_flexible_grasp_supervisor/tests/test_moveit_planner_pose_feedback.py
```

- [ ] **Step 4: Commit reacquisition**

```bash
git add src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/moveit_planner.py src/alicia_flexible_grasp_supervisor/tests/test_grasp_task_sequence.py src/alicia_flexible_grasp_supervisor/tests/test_moveit_planner_pose_feedback.py src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml
git commit -m "feat: add bounded clear-view grasp reacquisition"
```

---

### Task 6: Offline acceptance, documentation, and powered-run gate

**Files:**
- Modify: `src/alicia_flexible_grasp_supervisor/docs/grasp_task_technical_route.md`
- Modify: `src/alicia_flexible_grasp_supervisor/logs/2026-07-23-ros-latest-node-launch.md`
- Create: `docs/superpowers/verification/2026-09-03-known-mask-class-agnostic-precontact.md`

- [ ] **Step 1: Run the complete offline suite**

```bash
source /opt/ros/noetic/setup.bash
catkin_make -DCATKIN_ENABLE_TESTING=ON -j2
python3 -m unittest discover -s src/alicia_flexible_grasp_supervisor/tests -q
catkin_make run_tests_alicia_d_driver_gtest_actuation_confirmation_test
git diff --check
```

Expected: all Python, message-generation, build, and driver tests pass.

- [ ] **Step 2: Record auditable evidence**

Document test counts, label-invariance fixtures, live clipped-bbox regression, registration thresholds, clear-view attempt count, plan-ID binding, forbidden-command scan, and endpoint continuity results.

- [ ] **Step 3: Hot-load only after an idle command boundary**

Restart only affected nodes after confirming no task is executing; preserve the existing arm enable state and do not emit any stop/disable command. Monitor target track, fused views, registration metrics, strict MoveIt stages, trim phase, SDK composed target, encoder response, gripper command/feedback, and task state.

- [ ] **Step 4: Run one operator-authorized known-carton acceptance**

After the operator reports alignment, request one fresh plan. Acceptance requires far-field observation, near-field measured bilateral plan, pregrasp endpoint confirmation, valid 3D refinement or one clear-view reacquisition, approach, grasp pose, plan-bound close, linear lift, and post-lift contradiction check. On any invalid evidence, fail before contact and record the exact code; do not issue an automatic hardware stop/disable.

- [ ] **Step 5: Commit verification records**

```bash
git add docs/superpowers/verification/2026-09-03-known-mask-class-agnostic-precontact.md src/alicia_flexible_grasp_supervisor/docs/grasp_task_technical_route.md src/alicia_flexible_grasp_supervisor/logs/2026-07-23-ros-latest-node-launch.md
git commit -m "docs: record known-mask class-agnostic grasp acceptance"
```
