# YOLOv8 与 Carton 模型切换 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 ROS 目标识别 GUI 中运行时切换 `yolov8n.pt` 与工作空间 `carton_model/best.pt`，并让 Carton 模型始终使用固定类别 `carton`。

**Architecture:** 新建一个无 ROS 状态的模型选择辅助模块，统一模型配置、类别策略和路径解析。GUI 通过一次 `/perception` 字典更新提交选择，感知节点沿用检测器签名热重载权重，并通过 latched 状态主题反馈加载结果。

**Tech Stack:** Python 3、ROS Noetic `rospy`/参数服务器/`std_msgs`、PyQt5、Ultralytics YOLOv8、PyYAML、pytest/unittest。

## Global Constraints

- 默认选择必须保持 `original`，未操作新控件时继续使用 `yolov8n.pt` 和当前描述类别逻辑。
- Carton 选择必须始终得到 `yolo_target_class: carton`；任何目标描述都不能覆盖它。
- `carton_model/best.pt` 相对于 catkin 工作空间根目录解析，不依赖启动进程的当前目录。
- 模型加载失败时不得继续使用旧检测器或静默回退，ROS 节点必须保持运行。
- 运行期选择不写回 YAML；完整 ROS 重启后恢复 `camera.yaml` 默认值。
- 测试不得启动相机、执行推理循环或发送任何机械臂/夹爪运动命令。
- 不修改或提交用户现有的 `grasp_params.yaml`、`remote_grasp6d_node.py`、对应测试及生成的 `__pycache__` 改动。
- 不把 `carton_model/best.pt` 二进制权重加入提交；只引用用户已放置的本地文件。

---

## File Map

- Create: `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/model_selection.py` — 模型 profile、固定/描述类别策略、自定义权重路径解析。
- Create: `src/alicia_flexible_grasp_supervisor/tests/test_perception_model_selection.py` — 辅助模块的纯逻辑测试。
- Modify: `src/alicia_flexible_grasp_supervisor/config/camera.yaml` — 声明原模型和 Carton 模型 profile 及默认选择。
- Modify: `src/alicia_flexible_grasp_supervisor/scripts/perception_node.py` — 解析模型路径、清除旧检测器、发布加载状态。
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_perception_detector_selection.py` — 覆盖热重载成功/失败与状态消息。
- Modify: `src/alicia_flexible_grasp_supervisor/gui/widgets/perception_widget.py` — 下拉框、确定按钮、类别锁定、状态订阅。
- Create: `src/alicia_flexible_grasp_supervisor/tests/test_perception_model_selection_widget.py` — GUI 参数写入和状态转换测试。
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_gui_ros_lifecycle.py` — 覆盖新增 ROS 订阅者注销。
- Modify: `src/alicia_flexible_grasp_supervisor/README.md` — 使用方法和错误排查。

---

### Task 1: 共享模型配置、类别策略和路径解析

**Files:**
- Create: `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/model_selection.py`
- Create: `src/alicia_flexible_grasp_supervisor/tests/test_perception_model_selection.py`
- Modify: `src/alicia_flexible_grasp_supervisor/config/camera.yaml:24-35`

**Interfaces:**
- Consumes: `/perception` Python 字典；profile 字段 `display_name`、`model_path`、`target_class_mode`、可选 `target_class`。
- Produces: `normalize_model_profiles(perception_cfg: dict) -> dict`。
- Produces: `select_yolo_model(perception_cfg: dict, choice: str, description_target_class: str) -> dict`，返回 `choice`、`display_name`、`model_path`、`target_class_mode`、`target_class`。
- Produces: `resolve_yolo_model_path(model_path: str, package_path: Optional[str] = None, cwd: Optional[str] = None) -> str`。

- [ ] **Step 1: 写模型选择与路径解析的失败测试**

Create `src/alicia_flexible_grasp_supervisor/tests/test_perception_model_selection.py`:

```python
#!/usr/bin/env python3
import pathlib
import sys
import tempfile
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
SRC_DIR = ROOT / 'src'
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from alicia_flexible_grasp.vision.model_selection import (
    normalize_model_profiles,
    resolve_yolo_model_path,
    select_yolo_model,
)


MODEL_CONFIG = {
    'yolo_models': {
        'original': {
            'display_name': 'YOLOv8 原模型',
            'model_path': 'yolov8n.pt',
            'target_class_mode': 'description',
        },
        'carton': {
            'display_name': 'Carton 模型',
            'model_path': 'carton_model/best.pt',
            'target_class_mode': 'fixed',
            'target_class': 'carton',
        },
    },
}


class PerceptionModelSelectionTest(unittest.TestCase):
    def test_original_model_uses_description_target_class(self):
        selected = select_yolo_model(MODEL_CONFIG, 'original', 'bottle')

        self.assertEqual(selected['model_path'], 'yolov8n.pt')
        self.assertEqual(selected['target_class'], 'bottle')
        self.assertEqual(selected['target_class_mode'], 'description')

    def test_carton_model_always_uses_fixed_carton_class(self):
        selected = select_yolo_model(MODEL_CONFIG, 'carton', 'mouse')

        self.assertEqual(selected['model_path'], 'carton_model/best.pt')
        self.assertEqual(selected['target_class'], 'carton')
        self.assertEqual(selected['target_class_mode'], 'fixed')

    def test_default_profiles_exist_when_yaml_profiles_are_absent(self):
        profiles = normalize_model_profiles({})

        self.assertEqual(list(profiles), ['original', 'carton'])
        self.assertEqual(profiles['carton']['target_class'], 'carton')

    def test_resolves_carton_path_from_catkin_workspace_root(self):
        with tempfile.TemporaryDirectory() as tmp:
            workspace = pathlib.Path(tmp) / 'catkin_ws'
            package = workspace / 'src' / 'alicia_flexible_grasp_supervisor'
            model = workspace / 'carton_model' / 'best.pt'
            package.mkdir(parents=True)
            model.parent.mkdir(parents=True)
            model.write_bytes(b'weights')

            resolved = resolve_yolo_model_path(
                'carton_model/best.pt',
                package_path=str(package),
                cwd=str(pathlib.Path(tmp) / 'other'),
            )

            self.assertEqual(resolved, str(model.resolve()))

    def test_standard_ultralytics_weight_name_remains_unmodified(self):
        self.assertEqual(resolve_yolo_model_path('yolov8n.pt'), 'yolov8n.pt')

    def test_invalid_target_class_mode_is_rejected(self):
        invalid = {
            'yolo_models': {
                'broken': {
                    'display_name': 'Broken',
                    'model_path': 'broken.pt',
                    'target_class_mode': 'automatic',
                },
            },
        }

        with self.assertRaises(ValueError) as ctx:
            normalize_model_profiles(invalid)

        self.assertIn('target_class_mode', str(ctx.exception))

    def test_missing_custom_weight_path_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            package = pathlib.Path(tmp) / 'catkin_ws' / 'src' / 'alicia_flexible_grasp_supervisor'
            package.mkdir(parents=True)

            with self.assertRaises(FileNotFoundError) as ctx:
                resolve_yolo_model_path(
                    'carton_model/best.pt',
                    package_path=str(package),
                    cwd=tmp,
                )

        self.assertIn('carton_model/best.pt', str(ctx.exception))


if __name__ == '__main__':
    unittest.main()
```

- [ ] **Step 2: 运行测试并确认因辅助模块缺失而失败**

Run:

```bash
python3 -m pytest -q src/alicia_flexible_grasp_supervisor/tests/test_perception_model_selection.py
```

Expected: collection FAIL with `ModuleNotFoundError: No module named 'alicia_flexible_grasp.vision.model_selection'`。

- [ ] **Step 3: 实现最小共享辅助模块**

Create `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/model_selection.py`:

```python
from copy import deepcopy
from pathlib import Path


DEFAULT_MODEL_PROFILES = {
    'original': {
        'display_name': 'YOLOv8 原模型',
        'model_path': 'yolov8n.pt',
        'target_class_mode': 'description',
    },
    'carton': {
        'display_name': 'Carton 模型',
        'model_path': 'carton_model/best.pt',
        'target_class_mode': 'fixed',
        'target_class': 'carton',
    },
}


def normalize_model_profiles(perception_cfg):
    configured = dict((perception_cfg or {}).get('yolo_models') or {})
    source = configured or DEFAULT_MODEL_PROFILES
    profiles = {}
    for choice, raw_profile in source.items():
        profile = deepcopy(dict(raw_profile or {}))
        mode = str(profile.get('target_class_mode', '')).strip().lower()
        model_path = str(profile.get('model_path', '')).strip()
        if mode not in ('description', 'fixed'):
            raise ValueError('Invalid target_class_mode for %s: %s' % (choice, mode))
        if not model_path:
            raise ValueError('Missing model_path for %s' % choice)
        if mode == 'fixed' and not str(profile.get('target_class', '')).strip():
            raise ValueError('Missing fixed target_class for %s' % choice)
        profile['display_name'] = str(profile.get('display_name', choice))
        profile['model_path'] = model_path
        profile['target_class_mode'] = mode
        profile['target_class'] = str(profile.get('target_class', '')).strip()
        profiles[str(choice)] = profile
    return profiles


def select_yolo_model(perception_cfg, choice, description_target_class):
    profiles = normalize_model_profiles(perception_cfg)
    choice = str(choice or 'original')
    if choice not in profiles:
        raise ValueError('Unknown YOLO model choice: %s' % choice)
    profile = deepcopy(profiles[choice])
    if profile['target_class_mode'] == 'description':
        profile['target_class'] = str(description_target_class or '').strip()
    profile['choice'] = choice
    return profile


def _discover_package_path():
    try:
        import rospkg
        return rospkg.RosPack().get_path('alicia_flexible_grasp_supervisor')
    except Exception:
        return None


def resolve_yolo_model_path(model_path, package_path=None, cwd=None):
    raw_path = str(model_path or '').strip()
    if not raw_path:
        raise ValueError('YOLO model path is empty')
    expanded = Path(raw_path).expanduser()
    if expanded.is_absolute():
        if not expanded.is_file():
            raise FileNotFoundError('YOLO model file not found: %s' % raw_path)
        return str(expanded.resolve())
    if len(expanded.parts) == 1:
        return raw_path

    roots = [Path(cwd).resolve() if cwd is not None else Path.cwd().resolve()]
    package = package_path or _discover_package_path()
    if package:
        package_root = Path(package).resolve()
        roots.append(package_root)
        if package_root.parent.name == 'src':
            roots.append(package_root.parent.parent)

    checked = []
    for root in roots:
        candidate = (root / expanded).resolve()
        if candidate in checked:
            continue
        checked.append(candidate)
        if candidate.is_file():
            return str(candidate)
    raise FileNotFoundError('YOLO model file not found: %s' % raw_path)
```

- [ ] **Step 4: 在 YAML 中登记两个模型 profile**

Modify `src/alicia_flexible_grasp_supervisor/config/camera.yaml` immediately after `detector: "yolov8"`:

```yaml
  yolo_model_choice: "original"
  yolo_models:
    original:
      display_name: "YOLOv8 原模型"
      model_path: "yolov8n.pt"
      target_class_mode: "description"
    carton:
      display_name: "Carton 模型"
      model_path: "carton_model/best.pt"
      target_class_mode: "fixed"
      target_class: "carton"
```

Keep the existing flat values below unchanged:

```yaml
  yolo_model: "yolov8n.pt"
  yolo_target_class: "mouse"
```

- [ ] **Step 5: 运行共享逻辑测试并确认通过**

Run:

```bash
python3 -m pytest -q src/alicia_flexible_grasp_supervisor/tests/test_perception_model_selection.py
```

Expected: `7 passed`。

- [ ] **Step 6: 提交共享模型选择逻辑**

```bash
git add src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/model_selection.py \
  src/alicia_flexible_grasp_supervisor/tests/test_perception_model_selection.py \
  src/alicia_flexible_grasp_supervisor/config/camera.yaml
git commit -m "feat: add selectable YOLO model profiles"
```

---

### Task 2: 感知节点安全热重载和状态发布

**Files:**
- Modify: `src/alicia_flexible_grasp_supervisor/scripts/perception_node.py:1-24,164-237,475-535`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_perception_detector_selection.py`

**Interfaces:**
- Consumes: Task 1 的 `resolve_yolo_model_path(model_path) -> str`。
- Consumes: `/perception/yolo_model_choice`、`/perception/yolo_model` 和 `/perception/yolo_target_class`。
- Produces: latched `/perception/detector_status` `std_msgs/String`，格式为 `loading:<choice>`、`ready:<choice>:<path>` 或 `error:<choice>:<message>`。

- [ ] **Step 1: 扩展 detector selection 测试桩和成功状态断言**

Modify `src/alicia_flexible_grasp_supervisor/tests/test_perception_detector_selection.py`:

```python
from unittest import mock


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(str(getattr(msg, 'data', msg)))
```

Update `test_yolov8_config_creates_yolo_detector` so its config includes:

```python
'yolo_model_choice': 'carton',
```

Give the node a publisher:

```python
status_pub = FakePublisher()
node = types.SimpleNamespace(
    detector=None,
    detector_signature=None,
    detector_status_pub=status_pub,
    stabilizer=module.DetectionStabilizer(),
)
```

Wrap the refresh call and update the assertions:

```python
with mock.patch.object(module, 'resolve_yolo_model_path', return_value='/resolved/models/custom.pt'):
    module.PerceptionNode.refresh_detector(node, force=True)

self.assertEqual(FakeYOLODetector.created[0]['model_path'], '/resolved/models/custom.pt')
self.assertEqual(status_pub.messages, [
    'loading:carton',
    'ready:carton:/resolved/models/custom.pt',
])
```

- [ ] **Step 2: 写加载失败会清空旧检测状态的失败测试**

Add this method inside the existing `PerceptionDetectorSelectionTest` class in the same test file:

```python
    def test_failed_model_switch_clears_old_detector_and_publishes_error(self):
        module = load_perception_node()
        module.rospy = FakeRospy({
            'enabled': True,
            'detector': 'yolov8',
            'yolo_model_choice': 'carton',
            'yolo_model': 'carton_model/best.pt',
            'yolo_target_class': 'carton',
        })
        stabilizer = module.DetectionStabilizer()
        stabilizer.last_detection = object()
        status_pub = FakePublisher()
        node = types.SimpleNamespace(
            detector=object(),
            detector_signature=None,
            detector_status_pub=status_pub,
            stabilizer=stabilizer,
        )

        class BrokenYOLODetector:
            def __init__(self, **kwargs):
                raise RuntimeError('broken weights')

        module.YOLOv8ObjectDetector = BrokenYOLODetector
        with mock.patch.object(module, 'resolve_yolo_model_path', return_value='/workspace/carton_model/best.pt'):
            module.PerceptionNode.refresh_detector(node, force=True)

        self.assertIsNone(node.detector)
        self.assertIsNone(node.stabilizer.last_detection)
        self.assertEqual(status_pub.messages[0], 'loading:carton')
        self.assertEqual(status_pub.messages[1], 'error:carton:broken weights')
```

- [ ] **Step 3: 运行测试并确认新断言失败**

Run:

```bash
python3 -m pytest -q src/alicia_flexible_grasp_supervisor/tests/test_perception_detector_selection.py
```

Expected: FAIL because `resolve_yolo_model_path`、`detector_status_pub` behavior and status messages are not implemented。

- [ ] **Step 4: 实现状态 publisher 和安全模型交换**

Modify imports in `src/alicia_flexible_grasp_supervisor/scripts/perception_node.py`:

```python
from std_msgs.msg import Bool, String
from alicia_flexible_grasp.vision.model_selection import resolve_yolo_model_path
```

In `PerceptionNode.__init__`, create the publisher before the first `refresh_detector(force=True)` call:

```python
self.detector_status_pub = rospy.Publisher(
    '/perception/detector_status',
    String,
    queue_size=1,
    latch=True,
)
```

Add this method above `refresh_detector`:

```python
def _publish_detector_status(self, state, choice, detail=''):
    publisher = getattr(self, 'detector_status_pub', None)
    if publisher is None:
        return
    message = '%s:%s' % (str(state), str(choice))
    if detail:
        message += ':' + str(detail)
    publisher.publish(String(data=message))
```

In `refresh_detector`, read choice and resolve only YOLO paths:

```python
yolo_choice = str(pcfg.get('yolo_model_choice', 'original'))
yolo_model = str(pcfg.get('yolo_model', 'yolov8n.pt'))
resolved_yolo_model = yolo_model
path_error = None
if detector_kind in ('yolo', 'yolov8'):
    try:
        resolved_yolo_model = resolve_yolo_model_path(yolo_model)
    except Exception as exc:
        path_error = exc
```

Use `resolved_yolo_model` in the detector signature. After the unchanged-signature early return, replace the current detector setup block with:

```python
self.enabled = enabled
self.label = label
self.detector_kind = detector_kind
self.detector_error = ''
self.detector = None
self.stabilizer = DetectionStabilizer(hold_seconds, max_jump_px, switch_confirmations)
self._publish_detector_status('loading', yolo_choice)
try:
    if path_error is not None:
        raise path_error
    if detector_kind in ('yolo', 'yolov8'):
        self.detector = YOLOv8ObjectDetector(
            model_path=resolved_yolo_model,
            target_class=yolo_target_class,
            conf=yolo_conf,
            iou=yolo_iou,
            device=yolo_device,
            imgsz=yolo_imgsz,
        )
        self._publish_detector_status('ready', yolo_choice, resolved_yolo_model)
    else:
        self.detector_kind = 'simple_hsv'
        self.detector = HSVObjectDetector(lower, upper, min_area, normalized_ranges, shape)
        self._publish_detector_status('ready', 'simple_hsv')
except Exception as exc:
    self.detector = None
    self.detector_error = str(exc)
    self._publish_detector_status('error', yolo_choice, self.detector_error)
    rospy.logwarn_throttle(2.0, 'Failed to initialize %s detector: %s', detector_kind, exc)
self.detector_signature = signature
```

Keep the existing final `rospy.loginfo` call. Do not publish a held `ObjectPose` inside `refresh_detector`; replacing `DetectionStabilizer` is the stale-result barrier.

- [ ] **Step 5: 运行 detector selection 和检测稳定性测试**

Run:

```bash
python3 -m pytest -q \
  src/alicia_flexible_grasp_supervisor/tests/test_perception_detector_selection.py \
  src/alicia_flexible_grasp_supervisor/tests/test_detection_stability.py
```

Expected: all tests PASS；`test_perception_detector_selection.py` includes the new success/error status assertions。

- [ ] **Step 6: 提交感知节点热切换**

```bash
git add src/alicia_flexible_grasp_supervisor/scripts/perception_node.py \
  src/alicia_flexible_grasp_supervisor/tests/test_perception_detector_selection.py
git commit -m "feat: hot reload selected perception model"
```

---

### Task 3: GUI 模型下拉框、确定按钮和状态反馈

**Files:**
- Modify: `src/alicia_flexible_grasp_supervisor/gui/widgets/perception_widget.py:1-18,34-38,107-220,313-380`
- Create: `src/alicia_flexible_grasp_supervisor/tests/test_perception_model_selection_widget.py`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_gui_ros_lifecycle.py`

**Interfaces:**
- Consumes: Task 1 的 `normalize_model_profiles`、`select_yolo_model`、`resolve_yolo_model_path`。
- Consumes: Task 2 的 `/perception/detector_status` String protocol。
- Produces: `PerceptionWidget.confirm_model_selection()`。
- Produces: `PerceptionWidget._update_detector_status(text: str)` 和安全状态回调。

- [ ] **Step 1: 写 GUI 模型切换的失败测试和最小测试桩**

Create `src/alicia_flexible_grasp_supervisor/tests/test_perception_model_selection_widget.py`:

```python
#!/usr/bin/env python3
from copy import deepcopy
import pathlib
import sys
import unittest
from unittest import mock


ROOT = pathlib.Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / 'src'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import gui.widgets.perception_widget as widget_module
from gui.widgets.perception_widget import PerceptionWidget


MODEL_CONFIG = {
    'yolo_model_choice': 'original',
    'yolo_model': 'yolov8n.pt',
    'yolo_target_class': 'mouse',
    'yolo_models': {
        'original': {
            'display_name': 'YOLOv8 原模型',
            'model_path': 'yolov8n.pt',
            'target_class_mode': 'description',
        },
        'carton': {
            'display_name': 'Carton 模型',
            'model_path': 'carton_model/best.pt',
            'target_class_mode': 'fixed',
            'target_class': 'carton',
        },
    },
}


class FakeText:
    def __init__(self, value=''):
        self.value = value
        self.enabled = True

    def text(self):
        return self.value

    def setText(self, value):
        self.value = str(value)

    def setEnabled(self, enabled):
        self.enabled = bool(enabled)


class FakeCombo:
    def __init__(self, choice=None):
        self.choice = choice
        self.items = []

    def addItem(self, label, choice):
        self.items.append((str(label), str(choice)))

    def findData(self, choice):
        for index, item in enumerate(self.items):
            if item[1] == choice:
                return index
        return -1

    def setCurrentIndex(self, index):
        self.choice = self.items[index][1]

    def currentData(self):
        return self.choice


class FakeRospy:
    def __init__(self, perception_cfg):
        self.params = {'/perception': deepcopy(perception_cfg)}
        self.set_calls = []

    def get_param(self, name, default=None):
        return deepcopy(self.params.get(name, default))

    def set_param(self, name, value):
        self.params[name] = deepcopy(value)
        self.set_calls.append((name, deepcopy(value)))


def make_widget(choice, description='鼠标'):
    widget = PerceptionWidget.__new__(PerceptionWidget)
    widget.model_combo = FakeCombo(choice)
    widget.description_edit = FakeText(description)
    widget.yolo_class_edit = FakeText()
    widget.status = FakeText()
    widget.model_status_chip = FakeText()
    widget.model_profiles = {}
    widget._clear_locked_grasp_target = lambda: None
    return widget


class PerceptionModelSelectionWidgetTest(unittest.TestCase):
    def test_model_combo_initializes_from_current_ros_choice(self):
        widget = make_widget(None)
        profiles = MODEL_CONFIG['yolo_models']

        widget._populate_model_choices(profiles, 'carton')

        self.assertEqual(widget.model_combo.currentData(), 'carton')
        self.assertEqual(widget.model_combo.items[1], ('Carton 模型', 'carton'))

    def test_confirm_carton_writes_fixed_class_and_disables_editor(self):
        fake_rospy = FakeRospy(MODEL_CONFIG)
        widget = make_widget('carton', '鼠标')
        with mock.patch.object(widget_module, 'rospy', fake_rospy), mock.patch.object(
            widget_module,
            'resolve_yolo_model_path',
            return_value='/workspace/carton_model/best.pt',
        ):
            widget.confirm_model_selection()

        perception = fake_rospy.params['/perception']
        self.assertEqual(perception['yolo_model_choice'], 'carton')
        self.assertEqual(perception['yolo_model'], 'carton_model/best.pt')
        self.assertEqual(perception['yolo_target_class'], 'carton')
        self.assertEqual(widget.yolo_class_edit.text(), 'carton')
        self.assertFalse(widget.yolo_class_edit.enabled)

    def test_switching_to_original_restores_description_class(self):
        config = deepcopy(MODEL_CONFIG)
        config['yolo_model_choice'] = 'carton'
        fake_rospy = FakeRospy(config)
        widget = make_widget('original', '绿色瓶子')
        with mock.patch.object(widget_module, 'rospy', fake_rospy):
            widget.confirm_model_selection()

        self.assertEqual(fake_rospy.params['/perception']['yolo_target_class'], 'bottle')
        self.assertEqual(widget.yolo_class_edit.text(), 'bottle')
        self.assertTrue(widget.yolo_class_edit.enabled)

    def test_missing_carton_file_preserves_active_ros_config(self):
        fake_rospy = FakeRospy(MODEL_CONFIG)
        widget = make_widget('carton')
        with mock.patch.object(widget_module, 'rospy', fake_rospy), mock.patch.object(
            widget_module,
            'resolve_yolo_model_path',
            side_effect=FileNotFoundError('carton_model/best.pt'),
        ):
            widget.confirm_model_selection()

        self.assertEqual(fake_rospy.params['/perception']['yolo_model_choice'], 'original')
        self.assertEqual(fake_rospy.set_calls, [])
        self.assertIn('carton_model/best.pt', widget.status.text())

    def test_detector_status_is_translated_for_gui(self):
        widget = make_widget('carton')
        widget.model_profiles = {
            'carton': {'display_name': 'Carton 模型'},
        }

        widget._update_detector_status('ready:carton:/workspace/carton_model/best.pt')

        self.assertEqual(widget.model_status_chip.text(), 'Carton 模型已就绪')


if __name__ == '__main__':
    unittest.main()
```

- [ ] **Step 2: 写“开始识别”保持 Carton 类别的失败测试**

Add the following fakes and test to the same file:

```python
class FakeCheck:
    def __init__(self, checked):
        self.checked = checked

    def isChecked(self):
        return self.checked


class FakeValue:
    def __init__(self, value):
        self.stored = value

    def value(self):
        return self.stored


def test_start_recognition_does_not_override_carton_class(self):
    config = deepcopy(MODEL_CONFIG)
    config['yolo_model_choice'] = 'carton'
    config['yolo_model'] = 'carton_model/best.pt'
    config['yolo_target_class'] = 'carton'
    fake_rospy = FakeRospy(config)
    widget = make_widget('carton', '绿色瓶子')
    widget.enabled = FakeCheck(True)
    widget.lower_edit = FakeText('28,35,35')
    widget.upper_edit = FakeText('95,255,255')
    widget.label_edit = FakeText()
    widget.min_area = FakeValue(300)
    widget.pregrasp = FakeValue(0.08)
    widget.interpret_chip = FakeText()
    with mock.patch.object(widget_module, 'rospy', fake_rospy):
        widget.apply_params()

    self.assertEqual(fake_rospy.params['/perception']['yolo_target_class'], 'carton')
    self.assertEqual(widget.yolo_class_edit.text(), 'carton')
```

- [ ] **Step 3: 运行 GUI 测试并确认缺少控件行为**

Run:

```bash
QT_QPA_PLATFORM=offscreen python3 -m pytest -q \
  src/alicia_flexible_grasp_supervisor/tests/test_perception_model_selection_widget.py
```

Expected: FAIL because `confirm_model_selection`、`model_status_chip` update behavior and fixed-class application do not exist。

- [ ] **Step 4: 增加 GUI imports、信号和模型选择控件**

Modify imports in `perception_widget.py`:

```python
from std_msgs.msg import String
from alicia_flexible_grasp.vision.model_selection import (
    normalize_model_profiles,
    resolve_yolo_model_path,
    select_yolo_model,
)
```

Add the signal to `PerceptionWidget`:

```python
detector_status_signal = QtCore.pyqtSignal(str)
```

At initialization, set the subscriber attribute:

```python
self._detector_status_subscriber = None
```

Immediately before the existing target description `command_row`, build the model controls from `/perception`:

```python
perception_cfg = rospy.get_param('/perception', {})
self.model_profiles = normalize_model_profiles(perception_cfg)
model_row = QtWidgets.QHBoxLayout()
model_row.setSpacing(10)
model_row.addWidget(QtWidgets.QLabel('检测模型'))
self.model_combo = QtWidgets.QComboBox()
current_choice = str(perception_cfg.get('yolo_model_choice', 'original'))
self._populate_model_choices(self.model_profiles, current_choice)
confirm_model_btn = QtWidgets.QPushButton('确定模型')
confirm_model_btn.setObjectName('PrimaryButton')
confirm_model_btn.clicked.connect(self.confirm_model_selection)
model_row.addWidget(self.model_combo, 1)
model_row.addWidget(confirm_model_btn)
body.addLayout(model_row)
self.model_status_chip = metric_chip('等待模型状态', accent=True)
body.addWidget(self.model_status_chip)
```

After creating `self.yolo_class_edit`, synchronize its initial fixed/editable state:

```python
initial_selected = select_yolo_model(
    perception_cfg,
    current_choice if current_choice in self.model_profiles else 'original',
    perception_cfg.get('yolo_target_class', ''),
)
self._sync_model_class_editor(initial_selected)
```

- [ ] **Step 5: 实现确定按钮、类别编辑状态和模型状态转换**

Add these methods immediately before `apply_params`:

```python
def _populate_model_choices(self, profiles, current_choice):
    for choice, profile in profiles.items():
        self.model_combo.addItem(profile['display_name'], choice)
    current_index = self.model_combo.findData(str(current_choice))
    self.model_combo.setCurrentIndex(current_index if current_index >= 0 else 0)

def _sync_model_class_editor(self, selected):
    self.yolo_class_edit.setText(selected['target_class'])
    self.yolo_class_edit.setEnabled(selected['target_class_mode'] != 'fixed')

def confirm_model_selection(self):
    try:
        description = self.description_edit.text().strip() or '目标物体'
        parsed = self._parse_description(description)
        perception_cfg = rospy.get_param('/perception', {})
        choice = str(self.model_combo.currentData() or 'original')
        selected = select_yolo_model(
            perception_cfg,
            choice,
            parsed.get('yolo_target_class', ''),
        )
        resolve_yolo_model_path(selected['model_path'])
        perception_cfg.update({
            'yolo_model_choice': selected['choice'],
            'yolo_model': selected['model_path'],
            'yolo_target_class': selected['target_class'],
        })
        rospy.set_param('/perception', perception_cfg)
        self._sync_model_class_editor(selected)
        self._clear_locked_grasp_target()
        self.model_status_chip.setText('%s正在加载' % selected['display_name'])
        self.status.setText('模型选择已提交，视觉节点正在刷新')
    except Exception as exc:
        self.status.setText('模型切换失败：%s' % exc)

def _emit_detector_status_if_alive(self, msg):
    if not self.__dict__.get('_alive', False):
        return
    try:
        self.detector_status_signal.emit(str(getattr(msg, 'data', msg)))
    except RuntimeError:
        self._shutdown_ros()

def _update_detector_status(self, text):
    parts = str(text or '').split(':', 2)
    state = parts[0] if parts else ''
    choice = parts[1] if len(parts) > 1 else ''
    detail = parts[2] if len(parts) > 2 else ''
    profile = self.model_profiles.get(choice, {})
    display_name = str(profile.get('display_name', choice or '检测模型'))
    if state == 'loading':
        self.model_status_chip.setText('%s正在加载' % display_name)
    elif state == 'ready':
        self.model_status_chip.setText('%s已就绪' % display_name)
    elif state == 'error':
        self.model_status_chip.setText('%s加载失败' % display_name)
        self.status.setText('模型加载失败：%s' % detail)
```

- [ ] **Step 6: 让“开始识别”服从已确认的模型策略**

Inside `apply_params`, replace direct `parsed['yolo_target_class']` selection and individual `/perception/*` writes with:

```python
perception_cfg = rospy.get_param('/perception', {})
active_choice = str(perception_cfg.get('yolo_model_choice', 'original'))
selected = select_yolo_model(
    perception_cfg,
    active_choice,
    parsed.get('yolo_target_class', ''),
)
yolo_target_class = selected['target_class']
self._sync_model_class_editor(selected)
label = description
self.label_edit.setText(label)
perception_cfg.update({
    'enabled': bool(self.enabled.isChecked()),
    'object_label': label,
    'target_description': description,
    'yolo_target_class': yolo_target_class,
    'hsv_lower': lower,
    'hsv_upper': upper,
    'hsv_ranges': hsv_ranges,
    'shape': parsed['shape'],
    'min_area': int(self.min_area.value()),
})
rospy.set_param('/perception', perception_cfg)
rospy.set_param('/grasp/pregrasp_distance', float(self.pregrasp.value()))
```

Keep the existing lock clearing, interpretation summary, success status and exception handler after these writes.

- [ ] **Step 7: 订阅状态主题并安全注销两个订阅者**

Near the existing signal connections and object subscriber:

```python
self.detector_status_signal.connect(self._update_detector_status)
self._detector_status_subscriber = rospy.Subscriber(
    '/perception/detector_status',
    String,
    self._emit_detector_status_if_alive,
    queue_size=1,
)
```

Replace the single-subscriber block in `_shutdown_ros` with:

```python
for attribute in ('_subscriber', '_detector_status_subscriber'):
    subscriber = self.__dict__.get(attribute, None)
    flag = attribute + '_unregistered'
    if subscriber is not None and not self.__dict__.get(flag, False):
        try:
            subscriber.unregister()
        except Exception:
            pass
        self.__dict__[flag] = True
```

- [ ] **Step 8: 扩展 GUI 生命周期测试**

Modify the test path setup so both the package root and Python source directory are available, then import `PerceptionWidget`:

```python
for path in (ROOT, ROOT / 'src'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from gui.widgets.perception_widget import PerceptionWidget
```

Add this method inside the existing `GuiRosLifecycleTest` class:

```python
    def test_perception_widget_unregisters_object_and_detector_status_subscribers(self):
        widget = PerceptionWidget.__new__(PerceptionWidget)
        widget._alive = True
        widget._planning_active = False
        widget._plan_token = 0
        widget._subscriber = FakeSubscriber()
        widget._detector_status_subscriber = FakeSubscriber()

        PerceptionWidget._shutdown_ros(widget)

        self.assertTrue(widget._subscriber.unregistered)
        self.assertTrue(widget._detector_status_subscriber.unregistered)
```

- [ ] **Step 9: 运行 GUI 相关测试并确认通过**

Run:

```bash
QT_QPA_PLATFORM=offscreen python3 -m pytest -q \
  src/alicia_flexible_grasp_supervisor/tests/test_perception_model_selection_widget.py \
  src/alicia_flexible_grasp_supervisor/tests/test_perception_visual_alignment.py \
  src/alicia_flexible_grasp_supervisor/tests/test_gui_ros_lifecycle.py
```

Expected: all tests PASS, including fixed Carton class, original description class, missing-file preservation, status text and both subscriber cleanup。

- [ ] **Step 10: 提交 GUI 模型选择功能**

```bash
git add src/alicia_flexible_grasp_supervisor/gui/widgets/perception_widget.py \
  src/alicia_flexible_grasp_supervisor/tests/test_perception_model_selection_widget.py \
  src/alicia_flexible_grasp_supervisor/tests/test_gui_ros_lifecycle.py
git commit -m "feat: select perception model from GUI"
```

---

### Task 4: 文档、真实权重冒烟验证和回归测试

**Files:**
- Modify: `src/alicia_flexible_grasp_supervisor/README.md:101-108`

**Interfaces:**
- Consumes: Tasks 1-3 完成后的 GUI、参数和状态主题。
- Produces: 用户可执行的启动后切换说明与验证证据。

- [ ] **Step 1: 更新 README 使用说明**

Append these bullets under `## YOLOv8 目标识别说明`:

```markdown
- GUI“目标识别”页的“检测模型”下拉框可选择“YOLOv8 原模型”或“Carton 模型”；选择后点击“确定模型”，感知节点会在不重启 ROS 的情况下重新加载权重。
- “Carton 模型”读取工作空间根目录的 `carton_model/best.pt`，类别固定为 `carton`；“YOLOv8 原模型”恢复根据目标描述解析 COCO 类别。
- 模型选择只对当前 ROS 运行有效。重新启动整套系统后恢复 `config/camera.yaml` 中的 `yolo_model_choice: original`。
- GUI 显示“加载失败”时先确认 `carton_model/best.pt` 存在，再查看 `/perception/detector_status` 和感知节点日志中的 Ultralytics/Torch 错误。
```

- [ ] **Step 2: 运行语法和针对性测试**

Run:

```bash
python3 -m py_compile \
  src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/model_selection.py \
  src/alicia_flexible_grasp_supervisor/scripts/perception_node.py \
  src/alicia_flexible_grasp_supervisor/gui/widgets/perception_widget.py
QT_QPA_PLATFORM=offscreen python3 -m pytest -q \
  src/alicia_flexible_grasp_supervisor/tests/test_perception_model_selection.py \
  src/alicia_flexible_grasp_supervisor/tests/test_perception_detector_selection.py \
  src/alicia_flexible_grasp_supervisor/tests/test_detection_stability.py \
  src/alicia_flexible_grasp_supervisor/tests/test_perception_model_selection_widget.py \
  src/alicia_flexible_grasp_supervisor/tests/test_perception_visual_alignment.py \
  src/alicia_flexible_grasp_supervisor/tests/test_gui_ros_lifecycle.py \
  src/alicia_flexible_grasp_supervisor/tests/test_yolov8_detector.py
```

Expected: compilation succeeds and all selected tests PASS with no warnings or errors from the new feature。

- [ ] **Step 3: 用真实 Carton 权重做只加载元数据的冒烟验证**

Run:

```bash
python3 -c "from pathlib import Path; from ultralytics import YOLO; p=Path('carton_model/best.pt').resolve(); m=YOLO(str(p)); assert m.task == 'detect'; assert dict(m.names) == {0: 'carton'}; print(p, m.task, m.names)"
```

Expected: prints the absolute `best.pt` path followed by `detect {0: 'carton'}`。该命令不得调用 `predict()`。

- [ ] **Step 4: 运行功能包完整测试集**

Run:

```bash
QT_QPA_PLATFORM=offscreen python3 -m pytest -q src/alicia_flexible_grasp_supervisor/tests
```

Expected: all tests PASS。若已有与本功能无关的环境失败，记录准确测试名和原始错误，并确认 Task 1-3 的针对性测试仍全部通过。

- [ ] **Step 5: 检查改动范围和空白错误**

Run:

```bash
git diff --check
git status --short
git diff --name-only HEAD~3..HEAD
```

Expected: `git diff --check` has no output；功能改动只包含 File Map 中列出的源码、配置、测试和 README；用户原有脏文件仍未进入本功能提交。

- [ ] **Step 6: 提交文档**

```bash
git add src/alicia_flexible_grasp_supervisor/README.md
git commit -m "docs: explain runtime detector selection"
```

- [ ] **Step 7: 最终提交后重复关键验证**

Run:

```bash
QT_QPA_PLATFORM=offscreen python3 -m pytest -q \
  src/alicia_flexible_grasp_supervisor/tests/test_perception_model_selection.py \
  src/alicia_flexible_grasp_supervisor/tests/test_perception_detector_selection.py \
  src/alicia_flexible_grasp_supervisor/tests/test_perception_model_selection_widget.py \
  src/alicia_flexible_grasp_supervisor/tests/test_yolov8_detector.py
git status --short
```

Expected: key tests PASS；status output only contains the user’s pre-existing unrelated changes, generated caches, and untracked `carton_model/` directory。
