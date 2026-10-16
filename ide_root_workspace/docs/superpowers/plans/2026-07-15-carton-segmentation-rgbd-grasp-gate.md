# Carton Segmentation, RGB-D Localization, and MuJoCo Grasp Gate Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add switchable Carton instance segmentation, mask-driven RGB-D object geometry, gripper-aware 6D candidate selection, and a fail-closed WSL MuJoCo gate before real-arm execution.

**Architecture:** Keep one active YOLO profile and publish a full-resolution `mono8` mask for the selected segment instance. Build a synchronized three-frame RGB-D snapshot, estimate a support plane and carton OBB, then carry that immutable geometry and the selected jaw width in a rich plan through candidate gating and MuJoCo simulation. Preserve the legacy `PoseArray` only for visualization; real execution consumes the rich plan and requires every MuJoCo result flag to pass.

**Tech Stack:** ROS Noetic/catkin, Python 3.8, NumPy, OpenCV, PyQt5, Ultralytics YOLOv8, Intel RealSense `pyrealsense2`, HTTP/NPZ protocol, MuJoCo Python.

## Global Constraints

- Treat `docs/superpowers/specs/2026-07-15-carton-segmentation-rgbd-grasp-gate-design.md` as the source of truth.
- Preserve all three profiles: `original`, `carton`, and `carton_segment`; never run or merge two models simultaneously.
- `carton_segment` is fail-closed: missing, stale, empty, or wrong-sized masks must not fall back to bbox depth.
- `original` and `carton` remain detect profiles and continue to use the existing bbox-depth path.
- Publish segmentation masks at the original RGB resolution as `sensor_msgs/Image` with `mono8` encoding and the source frame timestamp.
- Preserve the D405 hardware scale `0.0001 m/unit` end-to-end; do not convert integer depth to millimetres implicitly.
- Do not add Open3D, trimesh, SciPy, or another heavy runtime dependency; geometry and denoising use NumPy/OpenCV.
- Use `max_inner_gap_m=0.05`; add 2 mm clearance per jaw side when computing required opening.
- Keep `mujoco_digital_twin.execution_gate_enabled=true` and `allow_execution_on_error=false` in the delivered configuration.
- Automated tests must not call real-arm motion services or require a connected RealSense camera.
- Do not change TCP, URDF, hand-eye calibration, tactile processing, force control, or slip control.
- Preserve user-owned `carton_segment_model/` weights and unrelated dirty bytecode files; never stage them.

Stable primary failure codes by stage:

- model/mask: `MODEL_TASK_MISMATCH`, `MASK_MISSING`, `MASK_STALE`, `MASK_EMPTY`, `MASK_SIZE_MISMATCH`;
- depth/geometry: `DEPTH_UNSTABLE`, `DEPTH_INSUFFICIENT`, `SUPPORT_PLANE_INVALID`, `OBB_INVALID`;
- remote prediction: `WSL_UNAVAILABLE`, `WSL_PREDICT_FAILED`, `NO_RAW_CANDIDATE`, `NO_GEOMETRIC_CANDIDATE`;
- gripper: `GRIPPER_MODEL_MISMATCH`, `GRIPPER_TOO_NARROW`, `GRIPPER_SWEEP_COLLISION`;
- MuJoCo: `MUJOCO_IK_FAILED`, `MUJOCO_COLLISION`, `MUJOCO_CONTACT_FAILED`, `MUJOCO_LIFT_FAILED`;
- plan lifecycle: `PLAN_INVALID`, `PLAN_STALE`, `PLAN_REPLACED`, `PLAN_ID_MISMATCH`.

Each failed operation publishes one primary code plus a concise reason. Additional gate counts and metrics belong in diagnostics and logs, not in competing primary codes.

## File Structure

New focused units:

- `msg/ObjectGeometry.msg`: immutable OBB, support plane, and depth-quality contract.
- `msg/Grasp6DPlan.msg`: four poses plus plan ID, width, model profile, and embedded geometry.
- `src/alicia_flexible_grasp/vision/rgbd_snapshot.py`: timestamp pairing, stability checks, mask erosion, multi-frame fusion, and MAD filtering.
- `src/alicia_flexible_grasp/vision/object_geometry.py`: support-plane estimation, point-cloud cleanup, and carton OBB fitting.
- `src/alicia_flexible_grasp/grasp/gripper_geometry.py`: 50 mm analytical prefilter and swept finger/palm envelope checks.
- `tests/test_rgbd_snapshot.py`, `tests/test_object_geometry.py`, `tests/test_gripper_geometry.py`: pure NumPy/OpenCV tests for the new units.

Existing integration points:

- `vision/model_selection.py`, `vision/yolov8_detector.py`, `scripts/perception_node.py`: model task and instance-mask production.
- `vision/realsense_manager.py`, `scripts/camera_node.py`: conservative continuous depth filtering and shared RGB/depth timestamps.
- `gui/widgets/perception_widget.py`, `gui/widgets/camera_widget.py`: target-page-only mask contour.
- `scripts/remote_grasp6d_node.py`: stable snapshot, OBB publication, GraspNet request, candidate gates, and rich plan publication.
- `scripts/grasp_task_node.py`, `vision/mujoco_digital_twin_client.py`: rich-plan validation and fail-closed simulation request.
- `tools/mujoco_digital_twin_server.py`: dynamic carton/support plane and exact mesh simulation.

---

### Task 1: Add Message Contracts and the Third Model Profile

**Files:**
- Create: `src/alicia_flexible_grasp_supervisor/msg/ObjectGeometry.msg`
- Create: `src/alicia_flexible_grasp_supervisor/msg/Grasp6DPlan.msg`
- Modify: `src/alicia_flexible_grasp_supervisor/CMakeLists.txt:21-28`
- Modify: `src/alicia_flexible_grasp_supervisor/config/camera.yaml:23-70`
- Modify: `src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml:90-252`
- Modify: `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/model_selection.py:5-51`
- Test: `src/alicia_flexible_grasp_supervisor/tests/test_perception_model_selection.py`

**Interfaces:**
- Consumes: existing `normalize_model_profiles(perception_cfg)` and `select_yolo_model(perception_cfg, choice, description_target_class)`.
- Produces: profiles containing `task: "detect"|"segment"` and `require_instance_mask: bool`; generated ROS classes `ObjectGeometry` and `Grasp6DPlan`.

- [ ] **Step 1: Write failing profile tests**

Add these cases to `test_perception_model_selection.py`:

```python
def test_segment_profile_exposes_task_and_requires_instance_mask(self):
    profiles = normalize_model_profiles({
        'yolo_models': {
            'carton_segment': {
                'display_name': 'Carton 分割模型',
                'model_path': 'carton_segment_model/best.pt',
                'task': 'segment',
                'target_class_mode': 'fixed',
                'target_class': 'carton',
                'require_instance_mask': True,
            },
        },
    })
    self.assertEqual(profiles['carton_segment']['task'], 'segment')
    self.assertTrue(profiles['carton_segment']['require_instance_mask'])

def test_profile_rejects_unknown_yolo_task(self):
    with self.assertRaisesRegex(ValueError, 'Invalid YOLO task'):
        normalize_model_profiles({
            'yolo_models': {
                'broken': {
                    'model_path': 'broken.pt',
                    'task': 'classify',
                    'target_class_mode': 'fixed',
                    'target_class': 'carton',
                },
            },
        })
```

- [ ] **Step 2: Run the tests and verify failure**

Run:

```bash
python3 -m pytest src/alicia_flexible_grasp_supervisor/tests/test_perception_model_selection.py -q
```

Expected: the segment profile lacks normalized `task`/`require_instance_mask`, and `classify` is not rejected.

- [ ] **Step 3: Add exact ROS message definitions**

Create `ObjectGeometry.msg`:

```text
std_msgs/Header header
bool valid
string label
string source_mode
geometry_msgs/Pose pose_base
geometry_msgs/Vector3 size_xyz_m
geometry_msgs/Vector3 support_normal_base
float32 support_offset_m
uint32 valid_depth_points
float32 valid_depth_ratio
float32 depth_mad_m
uint32 fused_frames
float32 support_inlier_ratio
uint32 object_point_count
string failure_reason
```

Create `Grasp6DPlan.msg`:

```text
std_msgs/Header header
bool valid
geometry_msgs/Pose[] poses
float32 score
float32 candidate_width_m
float32 required_open_width_m
alicia_flexible_grasp_supervisor/ObjectGeometry object_geometry
string model_choice
string plan_id
string diagnostic
```

Add both filenames inside the existing `add_message_files` `FILES` list in `CMakeLists.txt`.

- [ ] **Step 4: Normalize and validate model task metadata**

Extend `DEFAULT_MODEL_PROFILES` with `task: 'detect'` on both existing profiles and add:

```python
'carton_segment': {
    'display_name': 'Carton 分割模型',
    'model_path': 'carton_segment_model/best.pt',
    'task': 'segment',
    'target_class_mode': 'fixed',
    'target_class': 'carton',
    'require_instance_mask': True,
},
```

Inside `normalize_model_profiles`, normalize with this exact rule:

```python
task = str(profile.get('task', 'detect')).strip().lower()
if task not in ('detect', 'segment'):
    raise ValueError('Invalid YOLO task for %s: %s' % (choice, task))
require_mask = bool(profile.get('require_instance_mask', task == 'segment'))
if require_mask and task != 'segment':
    raise ValueError('require_instance_mask needs segment task for %s' % choice)
profile['task'] = task
profile['require_instance_mask'] = require_mask
```

- [ ] **Step 5: Add delivered configuration values**

Add `task: "detect"` to the two current `camera.yaml` profiles and add the exact `carton_segment` profile from the spec. Add these planning/gripper settings under `/grasp_6d/remote`:

```yaml
planning_snapshot_frames: 3
planning_snapshot_timeout_sec: 1.0
planning_snapshot_max_age_sec: 0.35
planning_mask_min_iou: 0.85
planning_mask_max_centroid_shift_px: 5.0
planning_max_joint_delta_rad: 0.01
mask_erosion_px: 2
mask_internal_hole_max_area_px: 25
depth_mad_scale: 3.5
depth_mad_absolute_floor_m: 0.002
target_cloud_voxel_size_m: 0.0025
target_cloud_outlier_neighbors: 16
target_cloud_outlier_std_ratio: 2.0
max_gripper_width_m: 0.05
candidate_width_tolerance_m: 0.0
gripper_geometry:
  max_inner_gap_m: 0.05
  width_safety_margin_per_side_m: 0.002
  tool_jaw_axis: "y"
  tool_finger_length_axis: "z"
  finger_box_xyz_m: [0.0434, 0.0286, 0.0600]
  palm_box_xyz_m: [0.1175, 0.1550, 0.0774]
```

Change the MuJoCo configuration to `execution_gate_enabled: true`, retain `allow_execution_on_error: false`, and replace the fixed mouse object with:

```yaml
object_model:
  type: "carton_box"
  label: "carton"
  mass_kg: 0.08
  friction: [1.2, 0.08, 0.02]
gripper_model:
  name: "Alicia_D_v5_6_gripper_50mm"
  max_inner_gap_m: 0.05
```

Also explicitly retain `enabled: true`, `require_object_pose: true`, `send_joint_state_in_request: true`, `open_width_m: 0.05`, and `min_score: 80`. These safety settings must not depend on older local defaults.

- [ ] **Step 6: Run profile tests and build generated messages**

Run:

```bash
python3 -m pytest src/alicia_flexible_grasp_supervisor/tests/test_perception_model_selection.py -q
catkin_make --pkg alicia_flexible_grasp_supervisor
```

Expected: profile tests pass and catkin reports successful generation of `ObjectGeometry.py` and `Grasp6DPlan.py` under `devel/lib/python3/dist-packages/alicia_flexible_grasp_supervisor/msg/`.

- [ ] **Step 7: Commit**

```bash
git add src/alicia_flexible_grasp_supervisor/msg/ObjectGeometry.msg src/alicia_flexible_grasp_supervisor/msg/Grasp6DPlan.msg src/alicia_flexible_grasp_supervisor/CMakeLists.txt src/alicia_flexible_grasp_supervisor/config/camera.yaml src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/model_selection.py src/alicia_flexible_grasp_supervisor/tests/test_perception_model_selection.py
git commit -m "feat: define carton segmentation grasp contracts"
```

### Task 2: Return the Selected YOLO Instance Mask

**Files:**
- Modify: `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/yolov8_detector.py:1-153`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_yolov8_detector.py`

**Interfaces:**
- Consumes: `YOLOv8ObjectDetector(model_path, target_class, conf, iou, device, imgsz, expected_task, require_instance_mask, model_backend, model_loader)`.
- Produces: `detect(bgr, preferred_uv=None, max_preferred_distance_px=None) -> (detection: dict|None, mask: np.ndarray|None)` where a segment detection contains `mask`, `mask_area`, and `mask_centroid` from the same result index as its bbox.

- [ ] **Step 1: Add failing same-index and resize tests**

Extend the fake result with masks and add:

```python
class FakeMasks:
    def __init__(self, data):
        self.data = FakeTensor(data)

def test_selected_bbox_uses_mask_from_same_instance(self):
    backend = FakeBackend()
    first = np.zeros((8, 13), dtype=np.float32)
    second = np.zeros((8, 13), dtype=np.float32)
    first[1:4, 1:4] = 1.0
    second[2:7, 7:12] = 1.0
    result = backend.predict(np.zeros((16, 26, 3), dtype=np.uint8))[0]
    result.masks = FakeMasks([first, second])
    backend.predict = lambda image, **kwargs: [result]
    detector = YOLOv8ObjectDetector(
        model_backend=backend,
        target_class='bottle',
        expected_task='segment',
        require_instance_mask=True,
    )

    detection, mask = detector.detect(np.zeros((16, 26, 3), dtype=np.uint8))

    self.assertEqual(detection['class_id'], 1)
    self.assertEqual(mask.shape, (16, 26))
    self.assertEqual(mask.dtype, np.uint8)
    self.assertGreater(np.count_nonzero(mask[:, 14:]), 0)
    self.assertEqual(np.count_nonzero(mask[:, :6]), 0)
    self.assertEqual(detection['mask_area'], int(np.count_nonzero(mask)))

def test_segment_task_rejects_detect_checkpoint(self):
    backend = FakeBackend()
    backend.task = 'detect'
    with self.assertRaisesRegex(RuntimeError, 'task mismatch'):
        YOLOv8ObjectDetector(
            model_backend=backend,
            expected_task='segment',
            require_instance_mask=True,
        )

def test_letterboxed_mask_restores_to_original_image_coordinates(self):
    original_shape = (360, 640)
    padded_mask = np.zeros((640, 640), dtype=np.float32)
    padded_mask[240:400, 160:480] = 1.0
    restored = YOLOv8ObjectDetector._restore_mask(padded_mask, original_shape)
    self.assertEqual(restored.shape, original_shape)
    self.assertGreater(np.count_nonzero(restored[55:305, 150:490]), 0)
    self.assertEqual(np.count_nonzero(restored[:20]), 0)
```

- [ ] **Step 2: Run tests and verify failure**

```bash
python3 -m pytest src/alicia_flexible_grasp_supervisor/tests/test_yolov8_detector.py -q
```

Expected: constructor rejects the new keywords or the returned mask is `None`.

- [ ] **Step 3: Add task validation and full-resolution mask extraction**

Add `expected_task='detect'` and `require_instance_mask=False` constructor parameters. After loading the backend, validate:

```python
self.expected_task = str(expected_task or 'detect').strip().lower()
self.require_instance_mask = bool(require_instance_mask)
actual_task = str(getattr(self.model, 'task', '') or '').strip().lower()
if actual_task and actual_task != self.expected_task:
    raise RuntimeError(
        'YOLO checkpoint task mismatch: expected %s, got %s'
        % (self.expected_task, actual_task)
    )
```

Extract by the original box index before candidate filtering. If the mask already has the original aspect ratio, resize directly; otherwise remove centered letterbox padding before nearest-neighbour resize:

```python
@classmethod
def _instance_mask(cls, result, index, image_shape):
    masks = getattr(result, 'masks', None)
    data = cls._to_numpy(getattr(masks, 'data', [])) if masks is not None else np.asarray([])
    if data.ndim != 3 or index >= data.shape[0]:
        return None
    restored = cls._restore_mask(np.asarray(data[index], dtype=np.float32), image_shape[:2])
    binary = np.where(restored >= 0.5, 255, 0).astype(np.uint8)
    return binary if np.any(binary) else None

@staticmethod
def _restore_mask(mask, original_shape):
    import cv2
    src_h, src_w = mask.shape[:2]
    dst_h, dst_w = [int(value) for value in original_shape[:2]]
    if abs((src_w / float(src_h)) - (dst_w / float(dst_h))) > 1e-3:
        gain = min(src_w / float(dst_w), src_h / float(dst_h))
        used_w = int(round(dst_w * gain))
        used_h = int(round(dst_h * gain))
        left = max(0, (src_w - used_w) // 2)
        top = max(0, (src_h - used_h) // 2)
        mask = mask[top:top + used_h, left:left + used_w]
    return cv2.resize(mask, (dst_w, dst_h), interpolation=cv2.INTER_NEAREST)

@staticmethod
def _mask_metrics(mask):
    ys, xs = np.nonzero(mask)
    if len(xs) == 0:
        return 0, None
    return int(len(xs)), (int(round(float(np.mean(xs)))), int(round(float(np.mean(ys)))))
```

Store `instance_index`, `mask`, `mask_area`, and `mask_centroid` in each candidate. Reject a segment mask if fewer than 80% of its nonzero pixels intersect its bbox or if the centroid falls outside a 2 px expanded bbox. In `detect`, return the chosen candidate and `chosen.get('mask')`; if `require_instance_mask` and it is missing or inconsistent, return `(None, None)`.

- [ ] **Step 4: Run detector tests**

```bash
python3 -m pytest src/alicia_flexible_grasp_supervisor/tests/test_yolov8_detector.py -q
```

Expected: all existing detect tests and the two new segment tests pass.

- [ ] **Step 5: Commit**

```bash
git add src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/yolov8_detector.py src/alicia_flexible_grasp_supervisor/tests/test_yolov8_detector.py
git commit -m "feat: return selected YOLO instance mask"
```

### Task 3: Publish Mask-Synchronized Object Localization

**Files:**
- Modify: `src/alicia_flexible_grasp_supervisor/scripts/perception_node.py:1-595`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_perception_depth_selection.py`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_perception_detector_selection.py`

**Interfaces:**
- Consumes: detector profile `task`/`require_instance_mask` and detector `(detection, mask)` output.
- Produces: `/perception/object_mask` (`mono8`) and `/perception/object` with the same source timestamp; `depth_m_at_mask(mask) -> float`.

- [ ] **Step 1: Add failing mask-depth and fail-closed tests**

Add to `test_perception_depth_selection.py`:

```python
def test_instance_mask_depth_excludes_nearer_bbox_background(self):
    module = load_perception_node()
    depth = np.full((40, 60), 5000, dtype=np.uint16)
    depth[10:30, 20:40] = 2200
    mask = np.zeros((40, 60), dtype=np.uint8)
    mask[10:30, 20:40] = 255
    node = make_node(module, depth)

    z = module.PerceptionNode.depth_m_at_mask(node, mask)

    self.assertAlmostEqual(z, 0.22, places=3)
    self.assertEqual(node._last_depth_source, 'instance_mask')
```

Add to `test_perception_detector_selection.py` a fake segment detector returning `det, None`, then assert `pub_obj.messages[-1].detected` is false and the mask publisher receives an all-zero `mono8` image with the supplied stamp.

- [ ] **Step 2: Run tests and verify failure**

```bash
python3 -m pytest src/alicia_flexible_grasp_supervisor/tests/test_perception_depth_selection.py src/alicia_flexible_grasp_supervisor/tests/test_perception_detector_selection.py -q
```

Expected: `depth_m_at_mask` and the mask publisher do not exist.

- [ ] **Step 3: Add the mask publisher and strict segment state**

Create the publisher during node initialization:

```python
self.pub_mask = rospy.Publisher(
    pcfg.get('output_mask_topic', '/perception/object_mask'),
    Image,
    queue_size=1,
)
self.detector_task = 'detect'
self.require_instance_mask = False
```

In `refresh_detector`, use the selected profile and include `task` and `require_instance_mask` in the signature and constructor:

```python
yolo_task = selected_model['task']
require_instance_mask = bool(selected_model['require_instance_mask'])
self.detector_task = yolo_task
self.require_instance_mask = require_instance_mask
```

Pass `expected_task=yolo_task` and `require_instance_mask=require_instance_mask` to `YOLOv8ObjectDetector`.

When detector construction raises a checkpoint task mismatch, publish detector error status beginning with `MODEL_TASK_MISMATCH`, publish a zero mask/current undetected object, and invalidate the previous mask and plan generation. Other load failures retain the existing detector-load error category.

- [ ] **Step 4: Implement mask depth and synchronized publishing**

Add:

```python
def depth_m_at_mask(self, mask):
    self._last_depth_source = 'none'
    self._last_depth_valid_count = 0
    binary = np.asarray(mask) > 0
    if binary.shape != np.asarray(self.depth).shape[:2] or not np.any(binary):
        return 0.0
    values = np.asarray(self.depth)[binary]
    values_m = values.astype(np.float32)
    if not np.issubdtype(values.dtype, np.floating):
        values_m *= float(self.depth_scale)
    valid = values_m[np.isfinite(values_m)]
    valid = valid[(valid >= self.depth_min_m) & (valid <= self.depth_max_m)]
    if valid.size < self.depth_min_valid_px:
        return 0.0
    median = float(np.median(valid))
    mad = float(np.median(np.abs(valid - median)))
    threshold = max(0.002, 3.5 * 1.4826 * mad)
    kept = valid[np.abs(valid - median) <= threshold]
    if kept.size < self.depth_min_valid_px:
        return 0.0
    self._last_depth_source = 'instance_mask'
    self._last_depth_valid_count = int(kept.size)
    return float(np.median(kept))

def _publish_mask(self, mask, stamp, frame_id):
    shape = np.asarray(self.color).shape[:2]
    output = np.zeros(shape, dtype=np.uint8)
    if mask is not None and np.asarray(mask).shape == shape:
        output[np.asarray(mask) > 0] = 255
    message = self.bridge.cv2_to_imgmsg(output, encoding='mono8')
    message.header.stamp = stamp
    message.header.frame_id = str(frame_id or self.color_frame_id)
    self.pub_mask.publish(message)
    return output
```

In `try_detect`, publish a zero mask on every invalid segment result. For a valid segment result, use `det['mask_centroid']` as `(u, v)`, call `depth_m_at_mask(mask)`, and publish the mask with `stamp`. Never call `depth_m_at_detection` as a segment fallback.

For segment mode, publish the raw current `ObjectPose` rather than the held stabilizer result, while still updating the stabilizer for preferred-instance tracking:

```python
if self.require_instance_mask:
    self.stabilizer.update(obj, rospy.get_time())
    self.pub_raw_detected.publish(Bool(bool(obj.detected)))
    self.pub_obj.publish(obj)
    self.pub_detected.publish(Bool(bool(obj.detected)))
    return
```

This prevents a held bbox/object timestamp from being paired with a newer mask.

- [ ] **Step 5: Run perception tests**

```bash
python3 -m pytest src/alicia_flexible_grasp_supervisor/tests/test_perception_depth_selection.py src/alicia_flexible_grasp_supervisor/tests/test_perception_detector_selection.py src/alicia_flexible_grasp_supervisor/tests/test_detection_stability.py -q
```

Expected: all pass; detect-mode holding remains unchanged and segment-mode missing masks fail closed.

- [ ] **Step 6: Commit**

```bash
git add src/alicia_flexible_grasp_supervisor/scripts/perception_node.py src/alicia_flexible_grasp_supervisor/tests/test_perception_depth_selection.py src/alicia_flexible_grasp_supervisor/tests/test_perception_detector_selection.py
git commit -m "feat: publish mask-driven object localization"
```

### Task 4: Draw the Instance Contour Only on the Target Page

**Files:**
- Modify: `src/alicia_flexible_grasp_supervisor/gui/widgets/camera_widget.py:14-296`
- Modify: `src/alicia_flexible_grasp_supervisor/gui/widgets/perception_widget.py:1-585`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_camera_overlay.py`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_perception_model_selection_widget.py`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_gui_ros_lifecycle.py`

**Interfaces:**
- Consumes: `/perception/object_mask` and `/perception/object` with identical timestamps.
- Produces: `CameraWidget.set_detection_overlay(bbox, label, color, contour_xy=None)`; contour is an `N x 2` integer array drawn with 2 px thickness.

- [ ] **Step 1: Add failing contour tests**

Add to `test_camera_overlay.py`:

```python
def test_detection_overlay_draws_two_pixel_instance_contour(self):
    rgb = np.zeros((60, 80, 3), dtype=np.uint8)
    contour = np.array([[20, 15], [50, 15], [50, 40], [20, 40]], dtype=np.int32)
    drawn = CameraWidget._draw_detection_overlay(
        rgb,
        {
            'bbox': (20, 15, 30, 25),
            'label': 'carton',
            'color': (80, 255, 120),
            'contour_xy': contour,
        },
    )
    self.assertTrue(np.any(drawn[15, 20:51] != 0))
    self.assertTrue(np.any(drawn[16, 20:51] != 0))
    self.assertTrue(np.all(rgb == 0))
```

Add GUI lifecycle coverage for `_mask_subscriber`, and add a model-combo assertion that `carton_segment` displays as `Carton 分割模型`.

- [ ] **Step 2: Run tests and verify failure**

```bash
QT_QPA_PLATFORM=offscreen python3 -m pytest src/alicia_flexible_grasp_supervisor/tests/test_camera_overlay.py src/alicia_flexible_grasp_supervisor/tests/test_perception_model_selection_widget.py src/alicia_flexible_grasp_supervisor/tests/test_gui_ros_lifecycle.py -q
```

Expected: contour is ignored, the third profile assertion fails, or `_mask_subscriber` is not unregistered.

- [ ] **Step 3: Extend the reusable overlay without changing normal camera pages**

Change `set_detection_overlay` to accept `contour_xy=None` and store a copied `int32` array. In `_draw_detection_overlay`, after the bbox:

```python
contour = overlay.get('contour_xy')
if contour is not None:
    points = np.asarray(contour, dtype=np.int32).reshape(-1, 1, 2)
    if len(points) >= 3:
        cv2.polylines(drawn, [points], True, color, 2, cv2.LINE_AA)
```

Do not add a mask subscriber to `CameraWidget`; only `PerceptionWidget` supplies `contour_xy`, so other camera pages remain unchanged.

- [ ] **Step 4: Subscribe, timestamp-match, and build the target contour**

In `PerceptionWidget`, add a Qt mask signal, `CvBridge`, `_latest_mask`, `_latest_mask_stamp`, a `mask_status_chip`, and a subscriber to `/perception/object_mask`. Use this exact stamp key:

```python
@staticmethod
def _stamp_key(header):
    stamp = getattr(header, 'stamp', None)
    if stamp is None:
        return None
    try:
        return int(stamp.to_nsec())
    except Exception:
        return (int(getattr(stamp, 'secs', 0)), int(getattr(stamp, 'nsecs', 0)))
```

Convert `mono8`, reject zero/wrong-sized masks, and choose the largest external contour:

```python
binary = np.where(np.asarray(mask) > 0, 255, 0).astype(np.uint8)
contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
contour_xy = None
if contours:
    contour_xy = max(contours, key=cv2.contourArea).reshape(-1, 2)
```

Only pass the contour to `camera_preview` when its stamp key equals `last_object.header`; otherwise draw bbox only and set the chip to `mask stale`. Clear mask state on model loading/error and unregister `_mask_subscriber` in `_shutdown_ros`.

- [ ] **Step 5: Run GUI tests**

```bash
QT_QPA_PLATFORM=offscreen python3 -m pytest src/alicia_flexible_grasp_supervisor/tests/test_camera_overlay.py src/alicia_flexible_grasp_supervisor/tests/test_perception_model_selection_widget.py src/alicia_flexible_grasp_supervisor/tests/test_gui_ros_lifecycle.py -q
```

Expected: all pass and existing bbox overlay tests remain green.

- [ ] **Step 6: Commit**

```bash
git add src/alicia_flexible_grasp_supervisor/gui/widgets/camera_widget.py src/alicia_flexible_grasp_supervisor/gui/widgets/perception_widget.py src/alicia_flexible_grasp_supervisor/tests/test_camera_overlay.py src/alicia_flexible_grasp_supervisor/tests/test_perception_model_selection_widget.py src/alicia_flexible_grasp_supervisor/tests/test_gui_ros_lifecycle.py
git commit -m "feat: show carton instance contour in target view"
```

### Task 5: Add Conservative RealSense Filtering and Shared RGB-D Timestamps

**Files:**
- Modify: `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/realsense_manager.py:1-71`
- Modify: `src/alicia_flexible_grasp_supervisor/scripts/camera_node.py:10-141`
- Modify: `src/alicia_flexible_grasp_supervisor/config/camera.yaml:1-22`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_realsense_depth_scale.py`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_camera_node_recovery.py`

**Interfaces:**
- Consumes: `RealSenseManager(depth_filter_cfg=None)` with the existing camera stream parameters preserved as named arguments if already present.
- Produces: spatially filtered, range-clipped, aligned `uint16` depth; RGB and depth messages from one acquisition share one ROS timestamp.

- [ ] **Step 1: Add failing filter and timestamp tests**

In `test_realsense_depth_scale.py`, add fake `spatial_filter`, `set_option`, and `process` objects and assert options `filter_magnitude=2`, `filter_smooth_alpha=0.5`, and `filter_smooth_delta=20` are set and `process` is called once. Add a fake that raises for `filter_smooth_delta`; assert the warning names that option and the unsupported spatial filter is omitted rather than half-configured.

In `test_camera_node_recovery.py`, publish one color/depth pair through a fake bridge and assert:

```python
self.assertEqual(node.pub_color.messages[-1].header.stamp, 'pair-stamp')
self.assertEqual(node.pub_depth.messages[-1].header.stamp, 'pair-stamp')
```

- [ ] **Step 2: Run tests and verify failure**

```bash
python3 -m pytest src/alicia_flexible_grasp_supervisor/tests/test_realsense_depth_scale.py src/alicia_flexible_grasp_supervisor/tests/test_camera_node_recovery.py -q
```

Expected: the manager has no filter configuration and camera images receive separate `Time.now()` values.

- [ ] **Step 3: Configure optional SDK filters**

Add `depth_filter_cfg=None` to `RealSenseManager.__init__`, store the dict, and construct an ordered filter list after starting the pipeline:

```python
def _configure_depth_filters(self):
    self.depth_filters = []
    cfg = dict(self.depth_filter_cfg or {})
    if bool(cfg.get('spatial_enabled', True)):
        spatial = self.rs.spatial_filter()
        values = (
            ('filter_magnitude', int(cfg.get('spatial_magnitude', 2))),
            ('filter_smooth_alpha', float(cfg.get('spatial_smooth_alpha', 0.5))),
            ('filter_smooth_delta', float(cfg.get('spatial_smooth_delta', 20))),
        )
        for option_name, value in values:
            spatial.set_option(getattr(self.rs.option, option_name), value)
        self.depth_filters.append(spatial)
    if bool(cfg.get('temporal_enabled', False)):
        self.depth_filters.append(self.rs.temporal_filter())
    if bool(cfg.get('hole_filling_enabled', False)):
        self.depth_filters.append(self.rs.hole_filling_filter())
```

Wrap each filter's complete option setup in `try/except`; on an unsupported constructor or option, issue `warnings.warn('RealSense depth filter disabled: <filter>/<option>: <error>')` and omit that filter. In `read`, align first, then pass only the depth frame through each configured filter. Convert to NumPy and zero values outside `depth_min_m/depth_max_m` using the hardware `depth_scale`. Keep the original shape and dtype.

- [ ] **Step 4: Pass configuration and publish paired timestamps**

Pass `self.cfg.get('depth_filter', {})` from `CameraNode._make_camera`. Change `publish_image` to accept `stamp`, and in `spin` use:

```python
stamp = rospy.Time.now()
if color is not None:
    self.publish_image(self.pub_color, color, 'bgr8', stamp)
if depth is not None:
    self.publish_image(self.pub_depth, depth, '16UC1', stamp)
```

Add the exact filter defaults from the spec under `camera.depth_filter` in `camera.yaml`.

- [ ] **Step 5: Run camera tests**

```bash
python3 -m pytest src/alicia_flexible_grasp_supervisor/tests/test_realsense_depth_scale.py src/alicia_flexible_grasp_supervisor/tests/test_camera_node_recovery.py -q
```

Expected: all pass, including existing recovery/fallback behavior.

- [ ] **Step 6: Commit**

```bash
git add src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/realsense_manager.py src/alicia_flexible_grasp_supervisor/scripts/camera_node.py src/alicia_flexible_grasp_supervisor/config/camera.yaml src/alicia_flexible_grasp_supervisor/tests/test_realsense_depth_scale.py src/alicia_flexible_grasp_supervisor/tests/test_camera_node_recovery.py
git commit -m "feat: filter and synchronize RealSense depth"
```

### Task 6: Build Stable Three-Frame Planning Snapshots

**Files:**
- Create: `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/rgbd_snapshot.py`
- Create: `src/alicia_flexible_grasp_supervisor/tests/test_rgbd_snapshot.py`
- Modify: `src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py:155-190,529-540,1029-1117,1312-1940`

**Interfaces:**
- Produces: `RgbdSample`, `SnapshotResult`, `SynchronizedRgbdBuffer`, `mask_iou`, `fuse_stable_samples`.
- Consumes later: `SnapshotResult.depth_raw`, `object_mask`, `quality`, `source_mode`, `object_msg`, `stamp_sec`, and `frame_id`.

- [ ] **Step 1: Write failing pure snapshot tests**

Create `test_rgbd_snapshot.py` with these central cases:

```python
import numpy as np
from alicia_flexible_grasp.vision.rgbd_snapshot import RgbdSample, fuse_stable_samples

def sample(depth_value, mask, stamp, joints=None):
    return RgbdSample(
        color_bgr=np.zeros((20, 30, 3), dtype=np.uint8),
        depth_raw=np.full((20, 30), depth_value, dtype=np.uint16),
        object_mask=mask.copy(),
        bbox=(5, 4, 20, 12),
        object_msg=None,
        stamp_sec=float(stamp),
        frame_id='camera_link',
        joint_positions=np.asarray(joints or [0.0] * 6, dtype=float),
    )

def test_three_stable_masks_fuse_by_pixel_median():
    mask = np.zeros((20, 30), dtype=np.uint8)
    mask[5:15, 8:22] = 255
    result = fuse_stable_samples(
        [sample(2190, mask, 1.00), sample(2200, mask, 1.03), sample(2210, mask, 1.06)],
        require_mask=True,
        min_mask_iou=0.85,
        max_centroid_shift_px=5.0,
        max_joint_delta_rad=0.01,
        erosion_px=2,
        depth_scale=0.0001,
        depth_min_m=0.03,
        depth_max_m=2.0,
        mad_scale=3.5,
        mad_absolute_floor_m=0.002,
    )
    assert result.ok
    assert result.quality.fused_frames == 3
    assert int(np.median(result.depth_raw[result.object_mask > 0])) == 2200

def test_joint_motion_rejects_window_instead_of_smearing_depth():
    mask = np.ones((20, 30), dtype=np.uint8) * 255
    moving = sample(2200, mask, 1.03, joints=[0.02, 0, 0, 0, 0, 0])
    result = fuse_stable_samples(
        [sample(2190, mask, 1.00), moving, sample(2210, mask, 1.06)],
        require_mask=True,
        min_mask_iou=0.85,
        max_centroid_shift_px=5.0,
        max_joint_delta_rad=0.01,
        erosion_px=2,
        depth_scale=0.0001,
        depth_min_m=0.03,
        depth_max_m=2.0,
        mad_scale=3.5,
        mad_absolute_floor_m=0.002,
    )
    assert not result.ok
    assert result.failure_code == 'DEPTH_UNSTABLE'
```

Also test mask IoU below 0.85, centroid shift above 5 px, wrong mask shape, bbox detect mode without a mask, edge erosion, MAD fly-point removal, a fully enclosed 9 px depth hole that is filled, and a hole touching the mask edge that remains zero.

- [ ] **Step 2: Run tests and verify import failure**

```bash
python3 -m pytest src/alicia_flexible_grasp_supervisor/tests/test_rgbd_snapshot.py -q
```

Expected: module import fails because `rgbd_snapshot.py` does not exist.

- [ ] **Step 3: Implement the pure data contract and fusion**

Use these exact public types:

```python
from dataclasses import dataclass
import numpy as np

@dataclass
class RgbdSample:
    color_bgr: np.ndarray
    depth_raw: np.ndarray
    object_mask: np.ndarray
    bbox: tuple
    object_msg: object
    stamp_sec: float
    frame_id: str
    joint_positions: np.ndarray

@dataclass(frozen=True)
class DepthQuality:
    fused_frames: int
    mask_area: int
    valid_depth_points: int
    valid_depth_ratio: float
    depth_median_m: float
    depth_mad_m: float

@dataclass(frozen=True)
class SnapshotResult:
    ok: bool
    failure_code: str
    failure_reason: str
    color_bgr: np.ndarray
    depth_raw: np.ndarray
    target_depth_raw: np.ndarray
    object_mask: np.ndarray
    bbox: tuple
    object_msg: object
    stamp_sec: float
    frame_id: str
    quality: DepthQuality
    source_mode: str
```

Implement `mask_iou`, centroid calculation, bbox overlap for detect mode, `cv2.erode`, three-frame `np.median`, and MAD filtering exactly against the thresholds passed to `fuse_stable_samples`. Add `internal_hole_max_area_px` to the fusion arguments. Use connected components on zero-depth pixels inside the eroded mask; fill a component only when it does not touch the eroded-mask boundary, its area is at most the configured limit, and its one-pixel ring contains valid depths. Fill with the ring median. Keep both depth arrays in the input integer units. `depth_raw` retains the range-clipped fused frame so the expanded non-mask bbox can estimate the support plane; `target_depth_raw` contains only eroded mask/bbox foreground pixels that survive MAD filtering and is zero everywhere else. Only `target_depth_raw` may be sent to GraspNet.

- [ ] **Step 4: Implement the synchronized callback buffer**

Add `SynchronizedRgbdBuffer` with a `threading.Condition`, partial entries keyed by integer nanosecond timestamp, and these methods:

```python
def update_color(self, color_bgr, stamp_sec, frame_id):
def update_depth(self, depth_raw, stamp_sec, frame_id):
def update_mask(self, object_mask, stamp_sec, frame_id):
def update_object(self, object_msg, stamp_sec):
def update_joints(self, joint_positions):
def wait_for_samples(self, count, timeout_sec, require_mask, max_age_sec):
```

`wait_for_samples` must return the newest complete samples with distinct timestamps. A complete segment entry has color, depth, mask, and a detected object; a complete detect entry has color, depth, and a detected object. Copy arrays before returning and prune entries older than two seconds.

- [ ] **Step 5: Integrate one-click waiting in `RemoteGrasp6DNode`**

Replace `LatestRgbdBuffer` with `SynchronizedRgbdBuffer`. Pass header timestamps from color, depth, mask, and object callbacks; subscribe to `/perception/object_mask` and `/joint_states`. In `request_plan_cb`, determine `require_mask` from the active normalized profile, wait up to 1.0 s for three samples, and call `fuse_stable_samples`.

Return exactly one structured prefix on failure:

```python
message = '%s: %s' % (result.failure_code, result.failure_reason)
self.status_pub.publish(String(message))
return TriggerZeroResponse(False, message)
```

Pass the resulting immutable snapshot to `_process_frame`; do not reread `latest_object`, latest mask, or current depth during that request.

- [ ] **Step 6: Run snapshot and remote-node regression tests**

```bash
python3 -m pytest src/alicia_flexible_grasp_supervisor/tests/test_rgbd_snapshot.py src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_node.py -q
```

Expected: all snapshot tests pass and existing remote candidate tests remain green after their fixtures use timestamped depth updates.

- [ ] **Step 7: Commit**

```bash
git add src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/rgbd_snapshot.py src/alicia_flexible_grasp_supervisor/tests/test_rgbd_snapshot.py src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_node.py
git commit -m "feat: fuse stable RGB-D planning snapshots"
```

---

### Task 7: Estimate the Support Plane and Carton OBB

**Files:**
- Create: `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/object_geometry.py`
- Create: `src/alicia_flexible_grasp_supervisor/tests/test_object_geometry.py`
- Modify: `src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_node.py`

**Interfaces:**
- Consumes: immutable `SnapshotResult`, camera intrinsics, `depth_scale`, and the snapshot-time `base_link <- camera` transform.
- Produces: `GeometryEstimate`, a target-only point cloud for GraspNet, and `ObjectGeometry.msg` in `base_link`.

- [ ] **Step 1: Write synthetic geometry failures and success cases**

Create `test_object_geometry.py` with a pinhole projection helper that renders a tilted support plane and a `0.24 x 0.16 x 0.10 m` box. Add these assertions:

```python
def test_mask_geometry_recovers_box_and_excludes_support_points():
    estimate = estimate_object_geometry(
        depth_raw=scene.full_depth_raw,
        target_depth_raw=scene.masked_box_depth_raw,
        object_mask=scene.mask,
        bbox=scene.bbox,
        intrinsics=scene.intrinsics,
        depth_scale=0.0001,
        T_base_camera=scene.T_base_camera,
        source_mode='instance_mask',
        support_bbox_expand_ratio=0.30,
        support_distance_threshold_m=0.004,
        voxel_size_m=0.0025,
        min_support_points=200,
        min_object_points=120,
        min_size_m=0.005,
        max_size_m=0.600,
        max_height_m=0.500,
        previous_axes_base=None,
    )
    assert estimate.ok
    np.testing.assert_allclose(np.sort(estimate.size_xyz_m), [0.10, 0.16, 0.24], atol=0.018)
    assert estimate.support_inlier_ratio >= 0.70
    signed = estimate.object_points_base @ estimate.support_normal_base + estimate.support_offset_m
    assert float(np.min(signed)) > -0.005

def test_support_plane_never_uses_mask_pixels():
    corrupted = scene.full_depth_raw.copy()
    corrupted[scene.mask > 0] = 1
    args = geometry_kwargs(scene)
    args['depth_raw'] = corrupted
    estimate = estimate_object_geometry(**args)
    assert estimate.ok
    np.testing.assert_allclose(estimate.support_normal_base, scene.support_normal_base, atol=0.04)

def test_too_few_context_points_reports_support_plane_invalid():
    args = geometry_kwargs(scene)
    args['depth_raw'] = np.where(scene.mask > 0, scene.full_depth_raw, 0)
    result = estimate_object_geometry(**args)
    assert not result.ok
    assert result.failure_code == 'SUPPORT_PLANE_INVALID'

def test_degenerate_object_reports_obb_invalid():
    args = geometry_kwargs(scene)
    args['target_depth_raw'] = one_pixel_wide_target(scene)
    result = estimate_object_geometry(**args)
    assert not result.ok
    assert result.failure_code == 'OBB_INVALID'
```

Define `geometry_kwargs(scene)` with the complete argument dictionary shown in the first test and define `one_pixel_wide_target(scene)` in the fixture module. Also cover NaN transforms, an out-of-range dimension, and axis-sign stabilization against `previous_axes_base`.

- [ ] **Step 2: Run the test and verify import failure**

```bash
python3 -m pytest src/alicia_flexible_grasp_supervisor/tests/test_object_geometry.py -q
```

Expected: collection fails because `object_geometry.py` does not exist.

- [ ] **Step 3: Implement deprojection and context-only support fitting**

Use these public definitions:

```python
from dataclasses import dataclass
import numpy as np

@dataclass(frozen=True)
class GeometryEstimate:
    ok: bool
    failure_code: str
    failure_reason: str
    center_base: np.ndarray
    axes_base: np.ndarray
    size_xyz_m: np.ndarray
    support_normal_base: np.ndarray
    support_offset_m: float
    support_inlier_ratio: float
    object_points_base: np.ndarray
    source_mode: str

def deproject_depth(depth_raw, pixel_mask, intrinsics, depth_scale):
    """Return Nx3 camera-frame points and matching integer uv pixels."""

def estimate_object_geometry(
        depth_raw, target_depth_raw, object_mask, bbox, intrinsics,
        depth_scale, T_base_camera, source_mode,
        support_bbox_expand_ratio, support_distance_threshold_m,
        voxel_size_m, min_support_points, min_object_points,
        min_size_m, max_size_m, max_height_m, previous_axes_base):
    """Return GeometryEstimate with one stable base-frame OBB."""
```

Build the support mask from the expanded bbox minus the object mask/bbox foreground. Fit a plane with deterministic RANSAC (`numpy.random.RandomState(0)`), refine its normal with SVD over all inliers, and calculate `normal.dot(point) + offset == 0`. Reject planes with non-finite coefficients, too few inliers, or inlier ratio below the configured `0.35`. Flip the normal so the median target point has positive signed distance.

- [ ] **Step 4: Implement point cleanup and the OBB convention**

Remove the support plane and tolerance band from target points. Quantize into 2.5 mm voxels and retain one centroid per occupied voxel. Apply a chunked NumPy neighbour-distance filter so memory remains bounded and no SciPy/Open3D dependency is introduced.

Project cleaned points to the support plane. Compute a robust 2D PCA axis after trimming points outside the 2nd–98th projected percentiles, then call `cv2.minAreaRect` on the retained 2D coordinates. Use the PCA axis to choose the equivalent minimum-area rectangle orientation with the smallest angular difference. Construct axes with these invariants:

```text
axes_base[:, 0] = in-plane long axis
axes_base[:, 1] = in-plane short axis
axes_base[:, 2] = support normal pointing toward the carton
det(axes_base) = +1
```

Use the 2nd and 98th signed-height percentiles for bottom/top, put the OBB center halfway between them, and set `size_xyz_m` in long/short/height order. Choose the equivalent x/y sign pair having the largest trace against `previous_axes_base`; this prevents 180-degree flips. Reject non-finite, degenerate, smaller-than-5-mm, larger-than-600-mm, or taller-than-500-mm OBBs with `OBB_INVALID`.

- [ ] **Step 5: Integrate geometry into the remote node**

After stable snapshot fusion, resolve TF at `snapshot.stamp_sec`, estimate geometry, and publish `/grasp_6d/object_geometry`. Map the estimate to `ObjectGeometry.msg`; convert `axes_base` to a normalized quaternion and copy the snapshot header, depth-quality fields, `fused_frames`, `support_inlier_ratio`, and `object_point_count`.

For `carton_segment`, pass `snapshot.target_depth_raw` to `RemoteGrasp6DClient.predict`; never add support pixels back into that request. For detect profiles, keep the existing bbox foreground computation but feed its result through the same `GeometryEstimate`/message path with `source_mode='bbox_depth'`.

Do not call `/predict` when `target_depth_raw` has fewer than the configured valid points; return `DEPTH_INSUFFICIENT`. Map connection/timeout failures to `WSL_UNAVAILABLE`, valid HTTP responses that cannot be decoded to `WSL_PREDICT_FAILED`, and an empty decoded candidate list to `NO_RAW_CANDIDATE`.

On model reload, target loss, snapshot failure, TF failure, support failure, or OBB failure, publish `ObjectGeometry(valid=False, failure_reason=failure_code + ': ' + failure_reason)` and invalidate every cached plan and previous OBB axis.

- [ ] **Step 6: Run focused geometry integration tests**

```bash
python3 -m pytest src/alicia_flexible_grasp_supervisor/tests/test_object_geometry.py src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_node.py -q
```

Expected: synthetic dimensions pass within tolerance; invalid support/OBB conditions publish no executable plan; the fake GraspNet request contains zeros outside the instance mask.

- [ ] **Step 7: Commit**

```bash
git add src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/object_geometry.py src/alicia_flexible_grasp_supervisor/tests/test_object_geometry.py src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_node.py
git commit -m "feat: estimate mask-driven carton geometry"
```

---

### Task 8: Add the 50 mm Gripper Geometry Candidate Gate

**Files:**
- Create: `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/gripper_geometry.py`
- Create: `src/alicia_flexible_grasp_supervisor/tests/test_gripper_geometry.py`
- Modify: `src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_candidate_selection.py`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_node.py`

**Interfaces:**
- Consumes: candidate center/rotation/width, base-frame OBB, support plane, and the four generated trajectory poses.
- Produces: `CandidateGateResult`, required opening, rejection code, clearance metrics, and geometry-first candidate order.

- [ ] **Step 1: Write failing analytical gripper tests**

Create tests using the measured mesh envelopes and 50 mm inner opening:

```python
GRIPPER = GripperGeometry(
    max_inner_gap_m=0.050,
    jaw_clearance_each_side_m=0.002,
    finger_size_xyz_m=np.array([0.0434, 0.0286, 0.0600]),
    palm_size_xyz_m=np.array([0.1175, 0.1550, 0.0774]),
    support_clearance_m=0.003,
)

def test_projected_carton_width_includes_both_clearances():
    required = required_open_width_m(
        obb_size_xyz_m=np.array([0.20, 0.040, 0.10]),
        R_base_obb=np.eye(3),
        jaw_axis_base=np.array([0.0, 1.0, 0.0]),
        clearance_each_side_m=0.002,
    )
    assert required == pytest.approx(0.044)

def test_51_mm_required_opening_is_rejected():
    args = candidate_fixture(obb_size_xyz_m=np.array([0.20, 0.047, 0.10]))
    result = evaluate_candidate(**args)
    assert not result.ok
    assert result.failure_code == 'GRIPPER_TOO_NARROW'

def test_center_below_support_or_outside_obb_is_rejected():
    below = candidate_fixture(center_base=np.array([0.0, 0.0, -0.004]))
    outside = candidate_fixture(center_base=np.array([0.30, 0.0, 0.05]))
    assert evaluate_candidate(**below).failure_code == 'GRIPPER_SWEEP_COLLISION'
    assert evaluate_candidate(**outside).failure_code == 'CENTER_OUTSIDE_OBB'

def test_palm_sweep_through_tilted_plane_is_rejected():
    args = candidate_fixture(
        support_normal_base=unit(np.array([0.20, 0.0, 0.98])),
        support_offset_m=0.0,
        pregrasp_center_base=np.array([-0.08, 0.0, -0.01]),
    )
    result = evaluate_candidate(**args)
    assert not result.ok
    assert result.failure_code == 'GRIPPER_SWEEP_COLLISION'
```

Define `candidate_fixture(**overrides)` with explicit identity rotations, OBB center/size, support coefficients, and four stage centers. Also test a valid 40 mm cross-section, a jaw axis that misses the OBB, one-sided finger reach, and a high model-score candidate that fails a hard gate.

- [ ] **Step 2: Run the test and verify import failure**

```bash
python3 -m pytest src/alicia_flexible_grasp_supervisor/tests/test_gripper_geometry.py -q
```

Expected: collection fails because `gripper_geometry.py` does not exist.

- [ ] **Step 3: Implement the immutable gripper contract**

```python
from dataclasses import dataclass

@dataclass(frozen=True)
class GripperGeometry:
    max_inner_gap_m: float
    jaw_clearance_each_side_m: float
    finger_size_xyz_m: np.ndarray
    palm_size_xyz_m: np.ndarray
    support_clearance_m: float

@dataclass(frozen=True)
class CandidateGateResult:
    ok: bool
    failure_code: str
    failure_reason: str
    required_open_width_m: float
    center_distance_m: float
    support_clearance_m: float
    jaw_alignment: float
    motion_cost: float
    geometry_cost: float
```

Resolve `tool_jaw_axis` through the existing axis parser and treat the delivered tool-frame +Y axis as the jaw-closing axis after `model_grasp_to_tool` alignment. Add a coordinate-contract test proving Link7/Link8 move oppositely along this base-transformed axis and that the finger's 60 mm envelope is tool +Z. Compute the carton projection along the jaw axis as:

```python
half = 0.5 * obb_size_xyz_m
projected_width = 2.0 * np.dot(np.abs(R_base_obb.T @ jaw_axis_base), half)
required_width = projected_width + 2.0 * gripper.jaw_clearance_each_side_m
```

The physical check always enforces `required_width <= 0.050`; do not retain the old `max_gripper_width_m <= 0` bypass.

- [ ] **Step 4: Implement hard gates and swept-volume sampling**

Check in this order and return the first stable code. Width failures use `GRIPPER_TOO_NARROW`; finger/palm/support/approach envelope failures use `GRIPPER_SWEEP_COLLISION`; center-only failures use `CENTER_OUTSIDE_OBB`:

1. finite candidate transform and right-handed rotation;
2. center within the OBB plus 3 mm tolerance and above the support clearance;
3. jaw line intersects both sides of the OBB and the required opening is at most 50 mm;
4. left/right finger center reach brackets the OBB cross-section;
5. the eight corners of both finger boxes and palm box remain above the support plane;
6. repeat box-corner checks at 11 evenly spaced transforms for pregrasp→approach, approach→grasp, and grasp→lift.

Use linear translation plus normalized quaternion SLERP for interpolation. During the grasp segment, permit finger-box overlap with the OBB only in the intended jaw-contact slabs; reject palm intrusion and approach from the support side. This is an analytical prefilter; exact STL collision remains MuJoCo's authority.

- [ ] **Step 5: Make geometry gates precede model score**

In `remote_grasp6d_node.py`, transform every GraspNet candidate to `base_link`, create all four stage poses, and call `evaluate_candidate` before score ranking. Count candidates after each hard gate and publish counts in the diagnostic string. Sort survivors lexicographically by:

```python
(
    gate.geometry_cost,
    -gate.support_clearance_m,
    -gate.jaw_alignment,
    gate.motion_cost,
    -float(candidate.score),
)
```

An invalid candidate can be logged but cannot be restored because of model score. Store `required_open_width_m` on the selected candidate separately from GraspNet's suggested `candidate.width_m`.

If raw candidates exist but none survives all hard gates, return `NO_GEOMETRIC_CANDIDATE` and include the per-gate rejection counts in the diagnostic text. A 50 mm MJCF/joint contract mismatch returns `GRIPPER_MODEL_MISMATCH` before candidate evaluation.

- [ ] **Step 6: Run candidate-selection regression tests**

```bash
python3 -m pytest src/alicia_flexible_grasp_supervisor/tests/test_gripper_geometry.py src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_candidate_selection.py src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_node.py -q
```

Expected: all over-width/support/palm failures are rejected; valid candidates retain the existing IK and joint-jump gates; geometry beats a higher invalid model score.

- [ ] **Step 7: Commit**

```bash
git add src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/gripper_geometry.py src/alicia_flexible_grasp_supervisor/tests/test_gripper_geometry.py src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_candidate_selection.py src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_node.py
git commit -m "feat: gate candidates with real gripper geometry"
```

---

### Task 9: Publish an Immutable Rich Plan and Migrate Execution Consumers

**Files:**
- Modify: `src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py`
- Modify: `src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py`
- Modify: `src/alicia_flexible_grasp_supervisor/gui/widgets/grasp6d_control_widget.py`
- Modify: `src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_node.py`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_control_widget.py`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_sequence.py`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_grasp_task_sequence.py`

**Interfaces:**
- Visualization topic: existing `/grasp_6d/plan` as `geometry_msgs/PoseArray`.
- Execution topic: new `/grasp_6d/plan_enriched` as `alicia_flexible_grasp_supervisor/Grasp6DPlan`.
- Fixed pose order: index 0 pregrasp, 1 approach, 2 grasp, 3 lift.

- [ ] **Step 1: Write rich-plan lifecycle tests**

Add tests that assert:

```python
def test_remote_plan_copies_geometry_width_and_snapshot_identity():
    plan = build_rich_plan(selected, geometry, snapshot_stamp=123.5, model_choice='carton_segment')
    assert plan.valid
    assert len(plan.poses) == 4
    assert plan.required_open_width_m == pytest.approx(0.044)
    assert plan.object_geometry.source_mode == 'instance_mask'
    assert plan.plan_id
    assert plan.header.stamp.to_sec() == pytest.approx(123.5)

def test_model_switch_publishes_invalid_rich_plan_and_disables_execute():
    node.model_choice_cb(String('original'))
    assert not node.rich_plan_pub.messages[-1].valid
    assert not widget.state.plan_ready

def test_grasp_task_ignores_legacy_pose_array_for_execution():
    node.grasp6d_legacy_plan_cb(PoseArray(poses=[finite_pose()] * 4))
    assert node.latest_grasp6d_plan is None

def test_expired_or_replaced_plan_cannot_execute():
    node.grasp6d_plan_cb(rich_plan(plan_id='first', stamp_sec=1.0))
    node.grasp6d_plan_cb(rich_plan(plan_id='second', stamp_sec=2.0))
    assert not node.validate_plan_id_for_execution('first').ok
```

Write the full fake ROS messages and callback calls for each case. GUI tests must verify that a legacy visualization plan alone never enables “执行 6D 抓取”.

- [ ] **Step 2: Run focused tests and verify failure**

```bash
python3 -m pytest src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_control_widget.py src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_sequence.py src/alicia_flexible_grasp_supervisor/tests/test_grasp_task_sequence.py src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_node.py -q
```

Expected: tests fail because consumers still use `PoseArray` as execution authority.

- [ ] **Step 3: Build and publish both plan forms atomically**

Add `build_rich_plan(selected_candidate, geometry_msg, snapshot_header, model_choice)` to `remote_grasp6d_node.py`. Generate `plan_id` as the SHA-256 digest of the canonical bytes for snapshot nanoseconds, model choice, four pose position/quaternion arrays, candidate width, required width, OBB pose/size, and support plane; publish the first 24 hexadecimal characters.

Publish the rich plan and its derived visualization `PoseArray` under one node lock. Every invalidation path publishes:

```python
invalid = Grasp6DPlan()
invalid.header = current_header
invalid.valid = False
invalid.diagnostic = failure_code + ': ' + failure_reason
self.rich_plan_pub.publish(invalid)
self.plan_pub.publish(PoseArray(header=current_header, poses=[]))
```

Never mutate a published plan object. The cached plan is a deep copy, and execution callbacks take another deep copy before validation.

- [ ] **Step 4: Migrate the GUI to rich-plan readiness**

Subscribe `Grasp6DControlWidget` to `/grasp_6d/plan_enriched`. Enable execute only if `valid`, exactly four finite poses, a non-empty `plan_id`, valid embedded geometry, `0 < required_open_width_m <= 0.05`, and age no greater than `/grasp_6d/plan_validity_sec`.

Show the current stage/failure code in the status label. Generation failure, model reload, target loss, empty mask, timeout, a replacement plan, or expiry immediately clears readiness. Keep the legacy topic subscription only for any current RViz/display behavior; it must not affect readiness.

- [ ] **Step 5: Migrate `GraspTaskNode` and visual retargeting**

Replace the execution-side `PoseArray` subscriber with `Grasp6DPlan`. Validate the same invariants again server-side. Build the visual-servo target from `plan.object_geometry.pose_base`, not a newly read `/perception/object_pose`; this binds center correction to the same mask/OBB snapshot as the candidate.

Keep any live target observation only as a drift guard: if its center differs from the plan OBB center by more than the configured `target_max_drift_m`, invalidate and require regeneration. Do not retarget the grasp with a later bbox center.

The fallback `scripts/grasp6d_node.py` remains allowed to publish legacy visualization poses, but cannot enable real execution because it does not produce a rich plan. On launch with `use_remote_grasp6d=false`, the GUI must show “本地旧版候选仅供显示，执行需要富计划” instead of silently presenting an executable plan.

- [ ] **Step 6: Run lifecycle and sequence tests**

```bash
python3 -m pytest src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_control_widget.py src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_sequence.py src/alicia_flexible_grasp_supervisor/tests/test_grasp_task_sequence.py src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_node.py -q
```

Expected: only a fresh, valid, four-pose rich plan enables execution; stale/replaced/legacy plans remain fail-closed.

- [ ] **Step 7: Commit**

```bash
git add src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py src/alicia_flexible_grasp_supervisor/gui/widgets/grasp6d_control_widget.py src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_node.py src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_control_widget.py src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_sequence.py src/alicia_flexible_grasp_supervisor/tests/test_grasp_task_sequence.py
git commit -m "feat: make rich 6D plans execution authority"
```

---

### Task 10: Make the ROS-to-WSL MuJoCo Gate Strict and Fail-Closed

**Files:**
- Modify: `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/mujoco_digital_twin_client.py`
- Modify: `src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py`
- Modify: `src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_mujoco_digital_twin_client.py`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_grasp_task_sequence.py`

**Interfaces:**
- Sends: rich plan identity, snapshot timestamp, joints, four poses, required opening, dynamic carton OBB, and dynamic support plane.
- Accepts: a response only when the echoed plan ID matches and all five safety booleans are explicitly `true`.

- [ ] **Step 1: Write serialization and fail-closed response tests**

Add exact payload assertions for:

```python
payload = build_mujoco_payload(plan, joint_names, joint_positions, gripper_config)
assert payload['plan_id'] == plan.plan_id
assert payload['candidate_width_m'] == pytest.approx(plan.candidate_width_m)
assert payload['required_open_width_m'] == pytest.approx(plan.required_open_width_m)
assert payload['object_model']['type'] == 'carton_box'
assert payload['object_model']['size_xyz_m'] == pytest.approx([0.24, 0.16, 0.10])
assert len(payload['trajectory']) == 4
assert payload['support_plane']['normal_base'] == pytest.approx([0.0, 0.0, 1.0])
```

Parameterize response tests over a missing/mismatched `plan_id`, each absent boolean, and each false boolean in `simulation_ok`, `ik_success`, `collision_free`, `contact_success`, and `lift_success`. Every case must block the motion service. Add timeout, malformed JSON, non-finite score, and score-below-threshold cases.

- [ ] **Step 2: Run tests and verify current permissive behavior fails them**

```bash
python3 -m pytest src/alicia_flexible_grasp_supervisor/tests/test_mujoco_digital_twin_client.py src/alicia_flexible_grasp_supervisor/tests/test_grasp_task_sequence.py -q
```

Expected: tests expose the current incomplete payload and permissive boolean handling.

- [ ] **Step 3: Serialize the immutable rich-plan payload**

Add a pure `build_mujoco_payload` helper. Use JSON-compatible finite floats only, quaternion order `[x, y, z, w]`, dimensions in metres, and this schema:

```python
{
    'schema_version': 2,
    'plan_id': plan.plan_id,
    'snapshot_stamp_sec': plan.header.stamp.to_sec(),
    'model_choice': plan.model_choice,
    'joint_names': list(joint_names),
    'joint_positions': list(joint_positions),
    'trajectory': [pose_to_dict(p) for p in plan.poses],
    'candidate_width_m': plan.candidate_width_m,
    'required_open_width_m': plan.required_open_width_m,
    'gripper': {
        'model_name': 'Alicia_D_v5_6_gripper_50mm',
        'max_inner_gap_m': 0.050,
        'finger_size_xyz_m': [0.0434, 0.0286, 0.0600],
        'palm_size_xyz_m': [0.1175, 0.1550, 0.0774],
    },
    'object_model': {
        'type': 'carton_box',
        'pose_base': pose_to_dict(plan.object_geometry.pose_base),
        'size_xyz_m': vector3_to_list(plan.object_geometry.size_xyz_m),
        'mass_kg': 0.08,
        'friction': [1.2, 0.08, 0.02],
    },
    'support_plane': {
        'normal_base': vector3_to_list(plan.object_geometry.support_normal_base),
        'offset_m': plan.object_geometry.support_offset_m,
    },
}
```

- [ ] **Step 4: Validate the complete response before any motion**

Implement `validate_mujoco_gate_response(response, expected_plan_id, min_score)` so it rejects absent keys rather than applying defaults. Require exact plan ID equality, finite score at or above threshold, and all five explicit booleans equal to Python `True`. Return a structured code/reason and surface WSL `failure_code`/`failure_reason` when present.

Normalize component failures to `MUJOCO_IK_FAILED`, `MUJOCO_COLLISION`, `MUJOCO_CONTACT_FAILED`, or `MUJOCO_LIFT_FAILED`; a missing/mismatched echoed ID is `PLAN_ID_MISMATCH`. Connection and timeout failures are `WSL_UNAVAILABLE` and remain blocking.

In `GraspTaskNode`, perform this validation immediately before execution while holding the copied plan. Confirm that the currently cached plan still has the same `plan_id`; otherwise reject with `PLAN_REPLACED`. Network exceptions and malformed responses reject because `allow_execution_on_error=false`.

- [ ] **Step 5: Run gate tests**

```bash
python3 -m pytest src/alicia_flexible_grasp_supervisor/tests/test_mujoco_digital_twin_client.py src/alicia_flexible_grasp_supervisor/tests/test_grasp_task_sequence.py -q
```

Expected: every missing/false component blocks execution and the happy path calls the fake motion service exactly once.

- [ ] **Step 6: Commit**

```bash
git add src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/mujoco_digital_twin_client.py src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml src/alicia_flexible_grasp_supervisor/tests/test_mujoco_digital_twin_client.py src/alicia_flexible_grasp_supervisor/tests/test_grasp_task_sequence.py
git commit -m "feat: enforce fail-closed MuJoCo grasp gate"
```

---

### Task 11: Simulate the Dynamic Carton, Support Plane, and Two-Sided Closure in WSL

**Files:**
- Modify: `tools/mujoco_digital_twin_server.py`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_mujoco_digital_twin_server_protocol.py`
- Modify: `src/alicia_flexible_grasp_supervisor/docs/mujoco_wsl_digital_twin.md`

**Interfaces:**
- Consumes: schema-version-2 payload from Task 10.
- Produces: plan-correlated component results with a single stable failure code.

- [ ] **Step 1: Write protocol and XML-injection tests first**

Add tests for these pure helpers and fake-MuJoCo paths:

```python
def test_carton_size_is_quantized_to_one_millimetre_for_cache_key():
    assert _model_cache_key(payload(size=[0.2004, 0.1004, 0.0804])) == \
           _model_cache_key(payload(size=[0.20049, 0.10049, 0.08049]))

def test_injected_carton_uses_half_extents_and_dynamic_support_body():
    object_model = {
        'size_xyz_m': [0.20, 0.10, 0.08],
        'mass_kg': 0.08,
        'friction': [1.2, 0.08, 0.02],
    }
    xml = _inject_dynamic_scene(BASE_XML, object_model)
    root = ElementTree.fromstring(xml)
    assert root.find(".//geom[@name='target_carton']").attrib['size'] == '0.10000 0.05000 0.04000'
    assert root.find(".//geom[@name='target_carton']").attrib['mass'] == '0.08000'
    assert root.find(".//geom[@name='target_carton']").attrib['friction'] == '1.20000 0.08000 0.02000'
    assert root.find(".//body[@name='detected_support']").attrib['mocap'] == 'true'

def test_response_echoes_plan_and_requires_all_components():
    response = server.simulate(valid_payload())
    assert response['plan_id'] == valid_payload()['plan_id']
    assert response['simulation_ok'] == all(response[k] for k in (
        'ik_success', 'collision_free', 'contact_success', 'lift_success'))

def test_single_finger_contact_never_passes():
    contacts = [FakeContact('Link7', 'target_object')]
    result = _classify_close_contacts(contacts, left_body='Link7', right_body='Link8')
    assert result.left_contact
    assert not result.right_contact
    assert not result.two_sided
```

Define `FakeContact(first_body, second_body)` in the test module. Also test invalid normal length, non-finite pose/size/joints, empty plan ID, wrong pose count, over-50-mm required opening, stale snapshot, palm-first contact, object/support penetration, and lift failure.

Add a gripper-contract fixture that loads the Link7/Link8 joint limits and facing collision bounds. At `left_finger=0`, `right_finger=0`, the current transformed mesh surfaces are `y=-0.02496875 m` and `y=+0.02496875 m`, giving a measured net gap of `0.04993750 m`. The fixture must accept this as the 50 mm contract and return `GRIPPER_MODEL_MISMATCH` when either a finger range or facing-surface gap differs from the configured value by more than 0.5 mm.

Add an explicit joint-mapping regression:

```python
def test_requested_inner_gap_maps_open_to_zero_joint_travel_and_closed_to_limits():
    assert _finger_qpos_for_inner_gap(0.050, max_gap_m=0.050) == pytest.approx((0.0, 0.0))
    assert _finger_qpos_for_inner_gap(0.000, max_gap_m=0.050) == pytest.approx((-0.025, 0.025))
```

This test replaces the current direct `qpos = +/- width/2` behavior, whose direction is reversed for this MJCF.

- [ ] **Step 2: Run protocol tests and verify failures**

```bash
python3 -m pytest src/alicia_flexible_grasp_supervisor/tests/test_mujoco_digital_twin_server_protocol.py -q
```

Expected: dynamic-carton, dynamic-plane, plan-correlation, and strict-component tests fail against the current fixed mouse/floor behavior.

- [ ] **Step 3: Validate schema before loading a MuJoCo model**

Create `_validate_v2_payload(payload, now_sec)` and reject with one of:

```text
PLAN_INVALID
PLAN_STALE
JOINT_STATE_INVALID
GRIPPER_TOO_NARROW
GRIPPER_MODEL_MISMATCH
SUPPORT_PLANE_INVALID
OBB_INVALID
```

Require schema 2, non-empty plan ID, exactly four finite poses, finite fresh timestamp, finite joints, normalized non-zero quaternion, three positive carton dimensions within configured limits, unit support normal after normalization, and `0 < required_open_width_m <= 0.050`.

- [ ] **Step 4: Inject and place a dynamic carton and support plane**

Replace fixed mouse injection for schema 2 with `_inject_dynamic_scene(xml, object_model)`. Add a free-joint `target_object` body containing:

```xml
<geom name="target_carton" type="box" size="HALF_X HALF_Y HALF_Z"
      mass="0.08000" rgba="0.72 0.52 0.28 1" friction="1.20000 0.08000 0.02000"/>
```

The snippet shows the delivered defaults; format `size`, `mass`, and `friction` from the validated `object_model` values for every compiled cache entry.

Add a mocap body named `detected_support` with an infinite plane geom whose local +Z normal is transformed to the requested support normal. Set its base-frame point to `-offset_m * normal`, and calculate a normalized quaternion rotating `[0, 0, 1]` to the normal. Disable collision on the original fixed `floor` geom for schema 2 so there is only one support authority.

Use the validated payload defaults `mass_kg=0.08` and `friction=[1.2, 0.08, 0.02]`. Quantize each full carton dimension to 1 mm for `_model_cache_key`; include mass and friction in the key because they alter compiled geometry, while applying each request's object pose and support pose to fresh `MjData`. At startup, set the two finger joints to their open endpoints, run `mj_forward`, project the facing Link7/Link8 collision-mesh bounds onto the configured jaw axis, and verify both the 25 mm-per-side joint travel and 50 mm net facing-surface gap. Reject startup with `GRIPPER_MODEL_MISMATCH` if either contract differs by more than 0.5 mm.

Replace `_apply_gripper_width` with an inner-gap mapping derived from those validated endpoints:

```python
closing_travel = 0.5 * (max_inner_gap_m - requested_inner_gap_m)
left_qpos = -closing_travel
right_qpos = closing_travel
```

Clamp only after payload validation. Thus 50 mm means fully open at `(0, 0)`, while 0 mm means the closed travel endpoints `(-0.025, +0.025)`.

- [ ] **Step 5: Stop closure at first valid two-sided contact**

Open to the validated physical maximum of exactly 50 mm. Close in at least 35 monotonic increments, call `mj_forward` at each increment, and inspect contacts after every step:

- reject immediately on palm/object, robot/support, or disallowed object/support penetration;
- continue past a single finger contact only while the object remains stable;
- capture and retain the first width with simultaneous left and right finger/object contact;
- use that captured width throughout lift simulation instead of forcing zero width.

The lift succeeds only if object displacement follows the commanded lift by the configured minimum while two-sided contact is retained and no disallowed collision appears.

- [ ] **Step 6: Make the response component-complete**

Echo `plan_id`, `failure_code`, and `failure_reason`. Set:

```python
simulation_ok = bool(
    ik_success and collision_free and contact_success and lift_success
    and np.isfinite(score) and score >= pass_score
)
```

Never derive success from score alone. Catch validation and MuJoCo exceptions into explicit false component flags; do not return HTTP/network success as simulation success.

- [ ] **Step 7: Run protocol tests and an optional local MuJoCo smoke test**

```bash
python3 -m pytest src/alicia_flexible_grasp_supervisor/tests/test_mujoco_digital_twin_server_protocol.py -q
python3 tools/mujoco_digital_twin_server.py --help
```

Expected: protocol tests pass and the server CLI lists its bind/config arguments. If `import mujoco` succeeds in this environment, run the test module with `MUJOCO_SMOKE=1` and require dynamic model compilation; otherwise record that full physics smoke must run on the configured WSL server before enabling arm motion.

- [ ] **Step 8: Update WSL documentation**

Document schema 2, required geometry fields, one-millimetre model cache, dynamic support plane, two-sided closure, the five required result flags, and a `curl` example containing a non-empty plan ID and four trajectory poses. State explicitly that no force/tactile feedback is part of this phase.

- [ ] **Step 9: Commit**

```bash
git add tools/mujoco_digital_twin_server.py src/alicia_flexible_grasp_supervisor/tests/test_mujoco_digital_twin_server_protocol.py src/alicia_flexible_grasp_supervisor/docs/mujoco_wsl_digital_twin.md
git commit -m "feat: simulate dynamic carton grasps in MuJoCo"
```

---

### Task 12: Run Checkpoint, Build, Regression, and Safety Acceptance

**Files:**
- Modify: `src/alicia_flexible_grasp_supervisor/docs/remote_grasp6d_wsl2.md`
- Create: `docs/superpowers/verification/2026-07-15-carton-segmentation-rgbd-grasp-gate.md`

- [ ] **Step 1: Verify the supplied segmentation checkpoint without changing it**

Run from the workspace root:

```bash
python3 - <<'PY'
from ultralytics import YOLO
import cv2
model = YOLO('carton_segment_model/best.pt')
assert model.task == 'segment', model.task
names = model.names
assert 'carton' in set(names.values()), names
carton_ids = {int(index) for index, name in names.items() if name == 'carton'}
capture = cv2.VideoCapture('/home/zhuyupei/Videos/915d0f0873b3fa8217a0d8582932b44d.mp4')
assert capture.isOpened(), 'uploaded grasp-path video cannot be opened'
found = None
for frame_index in range(0, 300, 15):
    capture.set(cv2.CAP_PROP_POS_FRAMES, frame_index)
    ok, frame = capture.read()
    if not ok:
        break
    result = model.predict(frame, conf=0.25, verbose=False)[0]
    classes = result.boxes.cls.cpu().numpy().astype(int) if result.boxes is not None else []
    if result.masks is not None and any(class_id in carton_ids for class_id in classes):
        found = (frame_index, tuple(frame.shape), tuple(result.masks.data.shape))
        break
capture.release()
assert found is not None, 'no Carton instance mask found in sampled uploaded-video frames'
print({'task': model.task, 'names': names, 'segment_smoke': found})
PY
```

Expected: task is `segment`, at least one class name is `carton`, and a sampled uploaded-video frame returns a non-empty instance-mask tensor. Do not add or commit the weight file or video.

- [ ] **Step 2: Run pure and ROS-mocked tests**

```bash
python3 -m pytest \
  src/alicia_flexible_grasp_supervisor/tests/test_perception_model_selection.py \
  src/alicia_flexible_grasp_supervisor/tests/test_yolov8_detector.py \
  src/alicia_flexible_grasp_supervisor/tests/test_perception_depth_selection.py \
  src/alicia_flexible_grasp_supervisor/tests/test_camera_overlay.py \
  src/alicia_flexible_grasp_supervisor/tests/test_realsense_depth_scale.py \
  src/alicia_flexible_grasp_supervisor/tests/test_rgbd_snapshot.py \
  src/alicia_flexible_grasp_supervisor/tests/test_object_geometry.py \
  src/alicia_flexible_grasp_supervisor/tests/test_gripper_geometry.py \
  src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_candidate_selection.py \
  src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_node.py \
  src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_control_widget.py \
  src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_sequence.py \
  src/alicia_flexible_grasp_supervisor/tests/test_grasp_task_sequence.py \
  src/alicia_flexible_grasp_supervisor/tests/test_mujoco_digital_twin_client.py \
  src/alicia_flexible_grasp_supervisor/tests/test_mujoco_digital_twin_server_protocol.py -q
```

Expected: every named test passes without a camera, robot, GraspNet server, or WSL MuJoCo server.

- [ ] **Step 3: Build generated ROS messages and run the full regression suite**

```bash
catkin_make --pkg alicia_flexible_grasp_supervisor
python3 -m pytest src/alicia_flexible_grasp_supervisor/tests -q
```

Expected: catkin generates `ObjectGeometry`/`Grasp6DPlan` successfully and the complete existing suite passes.

- [ ] **Step 4: Perform a camera-only bench acceptance**

With arm execution disabled at the motion gateway, start the normal launch and verify:

1. all three model choices appear and can switch repeatedly;
2. only the target-detection page draws the selected Carton mask contour;
3. RGB and depth headers have identical timestamps;
4. stable three-frame capture succeeds on one button press;
5. the log reports depth valid ratio/MAD, support inlier ratio, OBB dimensions, gate counts, required width, and plan ID;
6. occlusion, empty mask, moving camera/arm, width over 50 mm, or invalid support immediately clears the executable plan.

Record the exact command, ROS parameters, one success log, and each failure code in the verification document.

- [ ] **Step 5: Perform WSL MuJoCo acceptance before allowing arm motion**

Send the same rich plan payload to the WSL `/simulate_grasp` endpoint and inspect the MuJoCo view/log. Confirm carton size/pose, detected support plane, 50 mm open limit, exact Link7/Link8 collision meshes, first two-sided closure width, and lift. Test at least one deliberate failure for each component flag.

The arm remains blocked unless the response echoes the same plan ID and explicitly returns all of:

```text
simulation_ok=true
ik_success=true
collision_free=true
contact_success=true
lift_success=true
score >= configured pass_score
```

- [ ] **Step 6: Update operating documentation**

In `remote_grasp6d_wsl2.md`, add the three-model workflow, segmentation fail-closed behavior, one-click stable capture, depth-quality diagnostics, rich-plan lifecycle, the 50 mm physical limit, and the rule that the local legacy 6D node is visualization-only. Keep recovery instructions for restarting WSL services without weakening `allow_execution_on_error=false`.

- [ ] **Step 7: Review the final diff for scope and safety**

```bash
git status --short
git diff --check
git diff --stat HEAD~11..HEAD
git log --oneline -12
```

Expected: no whitespace errors; `carton_segment_model/` and bytecode remain untracked/unstaged; there are no TCP, URDF, hand-eye, tactile, force, or slip-control changes.

- [ ] **Step 8: Commit verification documentation**

```bash
git add src/alicia_flexible_grasp_supervisor/docs/remote_grasp6d_wsl2.md docs/superpowers/verification/2026-07-15-carton-segmentation-rgbd-grasp-gate.md
git commit -m "docs: record segmented grasp gate verification"
```

Do not call the real-arm execution service until Tasks 1–11 pass, camera-only acceptance is recorded, and WSL returns the complete matching success response for a newly generated plan.
