# Unknown-Object Perception Adapter Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** After the known-carton route passes, replace only the trained semantic-mask source with online class-agnostic RGB-D target extraction while leaving every downstream geometry, planning, control, and verification decision unchanged.

**Architecture:** Add a `geometric_unknown` adapter beside the phase-one `trained_instance_mask` adapter. It removes the measured support plane, applies configured camera/workspace bounds, builds connected RGB-D components, and tracks the stable component nearest the optical axis. Both adapters emit the identical `TargetObservation` contract and opaque track identity introduced by the precontact-geometry plan.

**Tech Stack:** ROS 1 Noetic, rospy, Python 3, NumPy, OpenCV connected components, RealSense aligned RGB-D, existing TF and support-plane estimation.

**Spec:** `docs/superpowers/specs/2026-09-03-known-to-unknown-class-agnostic-grasp-design.md`, section “Phase 2: unknown-object perception replacement”.

## Global Constraints

- Do not start this plan until the phase-one known-carton powered acceptance is documented as complete.
- Do not modify downstream fusion, GraspNet/tabletop candidate logic, safety gates, MoveIt sequence, endpoint control, close, lift, or result verification to accommodate this adapter.
- Do not use class labels, detector class indices, model filenames, learned per-class dimensions, or a known-object list for target selection.
- Default selection is the stable eligible component nearest the camera optical axis after operator alignment.
- Transparent, reflective, too-wide, unstable, clipped-with-insufficient-3D, or poorly observed targets fail deterministically; they do not receive geometry fallbacks.
- This work adds no automatic stop, torque-off, disable, controller-stop, emergency, `/demonstration=true`, or `/grasp/stop` command.

---

### Task 1: Implement pure support-plane foreground extraction

**Files:**
- Create: `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/unknown_target_extractor.py`
- Create: `src/alicia_flexible_grasp_supervisor/tests/test_unknown_target_extractor.py`

**Interfaces:**

```python
@dataclass(frozen=True)
class UnknownExtractionConfig:
    minimum_depth_m: float = 0.15
    maximum_depth_m: float = 0.80
    plane_clearance_m: float = 0.008
    minimum_component_points: int = 150
    maximum_component_points: int = 120000
    maximum_depth_gap_m: float = 0.012
    minimum_stable_frames: int = 3
    maximum_track_center_jump_m: float = 0.030

@dataclass(frozen=True)
class GeometricComponent:
    mask: np.ndarray
    points_camera: np.ndarray
    centroid_uv: tuple
    median_depth_m: float
    optical_axis_distance_px: float
    edge_clearance_px: int

def extract_geometric_components(
    depth_raw, intrinsics, depth_scale,
    support_normal_camera, support_offset_m, config
) -> tuple

def select_optical_axis_component(components) -> GeometricComponent
```

- [ ] **Step 1: Write synthetic RGB-D tests**

Cover one object, two objects at different optical-axis distances, a large support plane, depth-disconnected touching image blobs, workspace/range rejection, too few/many points, invalid depth, edge clipping, deterministic ties, and immutable output arrays.

- [ ] **Step 2: Verify RED**

```bash
python3 -m pytest -q src/alicia_flexible_grasp_supervisor/tests/test_unknown_target_extractor.py
```

- [ ] **Step 3: Implement plane removal and 3D-connected components**

Deproject bounded valid depth, remove points within `plane_clearance_m` of the measured support plane, and connect neighboring image pixels only when their metric depth gap is within `maximum_depth_gap_m`. Rank eligible components by optical-axis distance, then larger valid-point count, then top-left pixel for deterministic ties.

- [ ] **Step 4: Run tests and commit**

```bash
python3 -m pytest -q src/alicia_flexible_grasp_supervisor/tests/test_unknown_target_extractor.py
git add src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/unknown_target_extractor.py src/alicia_flexible_grasp_supervisor/tests/test_unknown_target_extractor.py
git commit -m "feat: extract unknown RGB-D target components"
```

---

### Task 2: Add temporal component tracking and opaque identity lifecycle

**Files:**
- Modify: `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/unknown_target_extractor.py`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_unknown_target_extractor.py`

**Interface:**

```python
class GeometricTargetTracker:
    def update(self, components, stamp_ns) -> TargetObservation
    def selected_observation(self) -> TargetObservation
    def reset(self, reason) -> None
```

- [ ] **Step 1: Write failing temporal tests**

Assert selection requires three monotonic frames, small center/depth changes retain the same track ID, a jump above `0.030 m`, non-monotonic stamp, lost component, or selected-component replacement advances identity. Labels are absent from every input.

- [ ] **Step 2: Implement deterministic association**

Associate by bounded 3D center distance and mask/depth overlap, keep the optical-axis-nearest stable component selected, and generate opaque IDs using the adapter generation plus monotonically increasing track epoch. Never recycle an ID within a process.

- [ ] **Step 3: Run tests and commit**

```bash
python3 -m pytest -q src/alicia_flexible_grasp_supervisor/tests/test_unknown_target_extractor.py
git add src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/unknown_target_extractor.py src/alicia_flexible_grasp_supervisor/tests/test_unknown_target_extractor.py
git commit -m "feat: track optical-axis unknown targets"
```

---

### Task 3: Integrate a switchable perception adapter without downstream changes

**Files:**
- Modify: `src/alicia_flexible_grasp_supervisor/scripts/perception_node.py`
- Modify: `src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml`
- Modify: `src/alicia_flexible_grasp_supervisor/config/gui_config.yaml`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_perception_detector_selection.py`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_graspnet_input_default_config.py`
- Modify: `src/alicia_flexible_grasp_supervisor/README.md`

- [ ] **Step 1: Write adapter-equivalence tests**

Feed a trained-mask fixture and a geometric-component fixture that describe the same RGB-D points. Assert both produce equivalent `TargetObservation` geometry, support data, timestamps, boundary visibility, and downstream plan results. The geometric adapter emits an empty diagnostic label.

- [ ] **Step 2: Add the explicit adapter mode**

Support exactly:

```yaml
perception:
  target_adapter: trained_instance_mask
  unknown_target:
    minimum_depth_m: 0.15
    maximum_depth_m: 0.80
    plane_clearance_m: 0.008
    minimum_component_points: 150
    maximum_component_points: 120000
    maximum_depth_gap_m: 0.012
    minimum_stable_frames: 3
    maximum_track_center_jump_m: 0.030
```

The phase-two operator change is only `target_adapter: geometric_unknown`. Invalid modes fail closed and publish no detected target/mask.

- [ ] **Step 3: Prove downstream code is unchanged**

Record the pre-change hashes/diff of the downstream files from the completed phase-one acceptance. After integration, verify no modification to multi-view fusion, candidate generation/ranking, safety, MoveIt sequence, driver, gripper close, lift, or post-lift verification.

- [ ] **Step 4: Run perception/config tests and commit**

```bash
python3 -m pytest -q src/alicia_flexible_grasp_supervisor/tests/test_unknown_target_extractor.py src/alicia_flexible_grasp_supervisor/tests/test_perception_detector_selection.py src/alicia_flexible_grasp_supervisor/tests/test_graspnet_input_default_config.py
git add src/alicia_flexible_grasp_supervisor/scripts/perception_node.py src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml src/alicia_flexible_grasp_supervisor/config/gui_config.yaml src/alicia_flexible_grasp_supervisor/tests/test_perception_detector_selection.py src/alicia_flexible_grasp_supervisor/tests/test_graspnet_input_default_config.py src/alicia_flexible_grasp_supervisor/README.md
git commit -m "feat: add unknown-object perception adapter"
```

---

### Task 4: Offline regression and unknown-object acceptance protocol

**Files:**
- Modify: `src/alicia_flexible_grasp_supervisor/docs/grasp_task_technical_route.md`
- Modify: `src/alicia_flexible_grasp_supervisor/logs/2026-07-23-ros-latest-node-launch.md`
- Create: `docs/superpowers/verification/2026-09-03-unknown-object-perception-adapter.md`

- [ ] **Step 1: Run complete offline verification**

```bash
source /opt/ros/noetic/setup.bash
catkin_make -DCATKIN_ENABLE_TESTING=ON -j2
python3 -m unittest discover -s src/alicia_flexible_grasp_supervisor/tests -q
catkin_make run_tests_alicia_d_driver_gtest_actuation_confirmation_test
git diff --check
```

- [ ] **Step 2: Run recorded class-agnostic fixtures**

Use at least three opaque rigid tabletop shapes with different aspect ratios and empty labels. Confirm identical downstream behavior for equivalent geometry and deterministic precontact rejection for transparent/reflective, aperture-exceeding, unstable, or insufficient-coverage fixtures.

- [ ] **Step 3: Document the powered acceptance protocol**

Do not initiate powered validation without a new operator alignment/authorization. The protocol is the same full stage order used by known-carton acceptance, with the perception mode and opaque track ID added to the audit. No new object-specific tuning is permitted during acceptance.

- [ ] **Step 4: Commit documentation**

```bash
git add docs/superpowers/verification/2026-09-03-unknown-object-perception-adapter.md src/alicia_flexible_grasp_supervisor/docs/grasp_task_technical_route.md src/alicia_flexible_grasp_supervisor/logs/2026-07-23-ros-latest-node-launch.md
git commit -m "docs: define unknown-object grasp acceptance"
```
