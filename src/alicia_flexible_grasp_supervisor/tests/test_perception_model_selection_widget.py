#!/usr/bin/env python3
from copy import deepcopy
import pathlib
import sys
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / 'src'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import gui.widgets.perception_widget as widget_module
from gui.widgets.camera_widget import CameraWidget
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
        'carton_segment': {
            'display_name': 'Carton 分割模型',
            'model_path': 'carton_segment_model/best.pt',
            'task': 'segment',
            'target_class_mode': 'fixed',
            'target_class': 'carton',
            'require_instance_mask': True,
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


class FakeButton:
    def __init__(self):
        self.enabled = True

    def setEnabled(self, enabled):
        self.enabled = bool(enabled)


class FakeStamp:
    def __init__(self, nanoseconds):
        self.nanoseconds = int(nanoseconds)

    def to_nsec(self):
        return self.nanoseconds


class FakeBridge:
    def __init__(self, mask):
        self.mask = mask

    def imgmsg_to_cv2(self, _msg, desired_encoding):
        if desired_encoding != 'mono8':
            raise AssertionError('mask must be converted as mono8')
        return self.mask


class FailingBridge:
    def imgmsg_to_cv2(self, _msg, desired_encoding):
        if desired_encoding != 'mono8':
            raise AssertionError('mask must be converted as mono8')
        raise ValueError('invalid mask encoding')


class FakeCameraPreview:
    def __init__(self, image_shape=(60, 80, 3)):
        self._last_color_rgb = np.zeros(image_shape, dtype=np.uint8)
        self.overlay = None

    def set_detection_overlay(self, bbox=None, label='', color=(80, 255, 120), contour_xy=None):
        self.overlay = {
            'bbox': bbox,
            'label': label,
            'color': color,
            'contour_xy': contour_xy,
        }


class FakeCallbackSignal:
    def __init__(self):
        self.callbacks = []

    def connect(self, callback):
        self.callbacks.append(callback)

    def emit(self, value):
        for callback in list(self.callbacks):
            callback(value)


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
    widget.execute_pregrasp_btn = FakeButton()
    widget.model_profiles = {}
    return widget


def make_mask_widget(mask, object_stamp=123, image_shape=(60, 80, 3)):
    widget = PerceptionWidget.__new__(PerceptionWidget)
    widget._alive = True
    widget.bridge = FakeBridge(mask)
    widget._latest_mask = None
    widget._latest_mask_stamp = None
    widget._current_object_stamp = int(object_stamp)
    widget._current_object_detected = True
    widget.mask_status_chip = FakeText('mask waiting')
    widget.camera_preview = FakeCameraPreview(image_shape)
    widget.label_edit = FakeText('carton')
    widget.description_edit = FakeText('carton')
    widget.last_object = SimpleNamespace(
        detected=True,
        header=SimpleNamespace(stamp=FakeStamp(object_stamp)),
        label='carton',
        bbox_x=5,
        bbox_y=5,
        bbox_width=70,
        bbox_height=50,
    )
    return widget


def make_mask_message(mask, stamp=123):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=FakeStamp(stamp)),
        height=int(mask.shape[0]),
        width=int(mask.shape[1]),
    )


class PerceptionModelSelectionWidgetTest(unittest.TestCase):
    def test_matching_mask_uses_largest_external_contour_without_mutating_mask(self):
        mask = np.zeros((60, 80), dtype=np.uint8)
        mask[5:10, 5:10] = 255
        mask[20:51, 30:61] = 255
        original = mask.copy()
        widget = make_mask_widget(mask)

        PerceptionWidget.update_mask(widget, make_mask_message(mask))

        contour = widget.camera_preview.overlay['contour_xy']
        self.assertIsNotNone(contour)
        self.assertEqual(contour.dtype, np.int32)
        self.assertEqual((int(contour[:, 0].min()), int(contour[:, 0].max())), (30, 60))
        self.assertEqual((int(contour[:, 1].min()), int(contour[:, 1].max())), (20, 50))
        self.assertEqual(widget.mask_status_chip.text(), 'mask ready')
        self.assertTrue(np.array_equal(mask, original))

    def test_unlocalized_segment_evidence_draws_mask_bbox_and_confidence(self):
        mask = np.zeros((60, 80), dtype=np.uint8)
        mask[20:51, 30:61] = 255
        widget = make_mask_widget(mask)
        widget._current_object_detected = False
        widget._current_visual_detected = True
        widget.last_object = SimpleNamespace(
            detected=False,
            header=SimpleNamespace(stamp=FakeStamp(123)),
            label='carton',
            confidence=0.934,
            depth_m=0.235,
            u=45,
            v=35,
            bbox_x=5,
            bbox_y=5,
            bbox_width=70,
            bbox_height=50,
        )

        PerceptionWidget.update_mask(widget, make_mask_message(mask))

        self.assertEqual(widget.camera_preview.overlay['bbox'], (5, 5, 70, 50))
        self.assertEqual(widget.camera_preview.overlay['label'], 'carton 0.934')
        self.assertIsNotNone(widget.camera_preview.overlay['contour_xy'])
        self.assertEqual(widget.mask_status_chip.text(), 'mask ready')

    def test_locked_no_detection_preserves_bbox_but_invalidates_ready_contour(self):
        mask = np.zeros((60, 80), dtype=np.uint8)
        mask[20:51, 30:61] = 255
        widget = make_mask_widget(mask)
        PerceptionWidget.update_mask(widget, make_mask_message(mask))
        locked_object = widget.last_object
        widget._grasp_active = True
        widget._has_recent_locked_grasp_target = lambda: True
        widget._planning_active = False
        widget._status_hold_until = 0.0
        widget.detected_chip = FakeText()
        widget.status = FakeText()

        PerceptionWidget.update_object(
            widget,
            SimpleNamespace(
                detected=False,
                header=SimpleNamespace(stamp=FakeStamp(124)),
                label='carton',
            ),
        )

        self.assertIs(widget.last_object, locked_object)
        self.assertEqual(widget.camera_preview.overlay['bbox'], (5, 5, 70, 50))
        self.assertEqual(widget.camera_preview.overlay['label'], 'carton')
        self.assertIsNone(widget.camera_preview.overlay['contour_xy'])
        self.assertEqual(widget.mask_status_chip.text(), 'mask stale')

    def test_late_old_mask_cannot_restore_contour_after_locked_no_detection(self):
        mask = np.zeros((60, 80), dtype=np.uint8)
        mask[20:51, 30:61] = 255
        widget = make_mask_widget(mask, object_stamp=123)
        PerceptionWidget.update_mask(widget, make_mask_message(mask, stamp=123))
        widget._grasp_active = True
        widget._has_recent_locked_grasp_target = lambda: True
        widget._planning_active = False
        widget._status_hold_until = 0.0
        widget.detected_chip = FakeText()
        widget.status = FakeText()

        PerceptionWidget.update_object(
            widget,
            SimpleNamespace(
                detected=False,
                header=SimpleNamespace(stamp=FakeStamp(124)),
                label='carton',
            ),
        )
        PerceptionWidget.update_mask(widget, make_mask_message(mask, stamp=123))

        self.assertEqual(widget._current_object_stamp, 124)
        self.assertFalse(widget._current_object_detected)
        self.assertEqual(widget.camera_preview.overlay['bbox'], (5, 5, 70, 50))
        self.assertEqual(widget.camera_preview.overlay['label'], 'carton')
        self.assertIsNone(widget.camera_preview.overlay['contour_xy'])
        self.assertEqual(widget.mask_status_chip.text(), 'mask stale')

    def test_normal_no_detection_clears_overlay_and_invalidates_ready_contour(self):
        mask = np.zeros((60, 80), dtype=np.uint8)
        mask[20:51, 30:61] = 255
        widget = make_mask_widget(mask)
        PerceptionWidget.update_mask(widget, make_mask_message(mask))
        widget._grasp_active = False
        widget._has_recent_locked_grasp_target = lambda: False
        widget._planned_pregrasp_pose = None
        widget._planning_active = False
        widget._status_hold_until = 0.0
        widget.pregrasp_pose = object()
        widget._reset_target_stability = lambda: None
        widget.detected_chip = FakeText()
        widget.status = FakeText()

        PerceptionWidget.update_object(
            widget,
            SimpleNamespace(
                detected=False,
                header=SimpleNamespace(stamp=FakeStamp(124)),
                label='carton',
            ),
        )

        self.assertIsNone(widget.camera_preview.overlay['bbox'])
        self.assertIsNone(widget.camera_preview.overlay['contour_xy'])
        self.assertEqual(widget.mask_status_chip.text(), 'mask stale')

    def test_mismatched_mask_stamp_keeps_bbox_only_and_reports_stale(self):
        mask = np.zeros((60, 80), dtype=np.uint8)
        mask[20:51, 30:61] = 255
        widget = make_mask_widget(mask, object_stamp=124)

        PerceptionWidget.update_mask(widget, make_mask_message(mask, stamp=123))

        self.assertEqual(widget.camera_preview.overlay['bbox'], (5, 5, 70, 50))
        self.assertIsNone(widget.camera_preview.overlay['contour_xy'])
        self.assertEqual(widget.mask_status_chip.text(), 'mask stale')

    def test_mask_received_before_current_object_reports_stale(self):
        mask = np.zeros((60, 80), dtype=np.uint8)
        mask[20:51, 30:61] = 255
        widget = make_mask_widget(mask)
        widget.last_object = None

        PerceptionWidget.update_mask(widget, make_mask_message(mask))

        self.assertIsNone(widget.camera_preview.overlay['bbox'])
        self.assertEqual(widget.mask_status_chip.text(), 'mask stale')

    def test_empty_mask_is_rejected_with_visible_status(self):
        mask = np.zeros((60, 80), dtype=np.uint8)
        widget = make_mask_widget(mask)

        PerceptionWidget.update_mask(widget, make_mask_message(mask))

        self.assertIsNone(widget._latest_mask)
        self.assertIsNone(widget._latest_mask_stamp)
        self.assertIsNone(widget.camera_preview.overlay['contour_xy'])
        self.assertEqual(widget.mask_status_chip.text(), 'mask empty')

    def test_wrong_size_mask_is_rejected_with_visible_status(self):
        mask = np.full((30, 40), 255, dtype=np.uint8)
        widget = make_mask_widget(mask, image_shape=(60, 80, 3))

        PerceptionWidget.update_mask(widget, make_mask_message(mask))

        self.assertIsNone(widget._latest_mask)
        self.assertIsNone(widget._latest_mask_stamp)
        self.assertEqual(widget.camera_preview.overlay['bbox'], (5, 5, 70, 50))
        self.assertIsNone(widget.camera_preview.overlay['contour_xy'])
        self.assertEqual(widget.mask_status_chip.text(), 'mask size mismatch')

    def test_cached_mask_is_rejected_if_camera_size_becomes_incompatible(self):
        mask = np.full((60, 80), 255, dtype=np.uint8)
        widget = make_mask_widget(mask)
        widget.camera_preview._last_color_rgb = None
        PerceptionWidget.update_mask(widget, make_mask_message(mask))
        widget.camera_preview._last_color_rgb = np.zeros((30, 40, 3), dtype=np.uint8)

        PerceptionWidget._refresh_detection_overlay(widget)

        self.assertIsNone(widget._latest_mask)
        self.assertIsNone(widget._latest_mask_stamp)
        self.assertIsNone(widget.camera_preview.overlay['contour_xy'])
        self.assertEqual(widget.mask_status_chip.text(), 'mask size mismatch')

    def test_first_rgb_frame_revalidates_mask_without_manual_perception_refresh(self):
        mask = np.full((60, 80), 255, dtype=np.uint8)
        widget = make_mask_widget(mask)
        camera = CameraWidget.__new__(CameraWidget)
        camera._alive = True
        camera._last_color_rgb = None
        camera._detection_overlay = None
        camera._refresh_color_pixmap = lambda: None
        camera._render_pixmaps = lambda *_args: None
        camera.color_frame_updated = FakeCallbackSignal()
        camera.color_frame_updated.connect(
            lambda rgb: PerceptionWidget._on_camera_color_frame(widget, rgb)
        )
        widget.camera_preview = camera
        PerceptionWidget.update_mask(widget, make_mask_message(mask))
        self.assertIsNotNone(camera._detection_overlay['contour_xy'])

        CameraWidget.update_color_image(
            camera,
            np.zeros((30, 40, 3), dtype=np.uint8),
        )

        self.assertIsNone(widget._latest_mask)
        self.assertIsNone(widget._latest_mask_stamp)
        self.assertIsNone(camera._detection_overlay['contour_xy'])
        self.assertEqual(widget.mask_status_chip.text(), 'mask size mismatch')

    def test_mask_conversion_error_immediately_removes_previous_contour(self):
        mask = np.zeros((60, 80), dtype=np.uint8)
        mask[20:51, 30:61] = 255
        widget = make_mask_widget(mask)
        PerceptionWidget.update_mask(widget, make_mask_message(mask))
        self.assertIsNotNone(widget.camera_preview.overlay['contour_xy'])
        widget.bridge = FailingBridge()

        with mock.patch.object(widget_module.rospy, 'logwarn_throttle'):
            PerceptionWidget.update_mask(widget, make_mask_message(mask, stamp=124))

        self.assertIsNone(widget._latest_mask)
        self.assertIsNone(widget._latest_mask_stamp)
        self.assertEqual(widget.camera_preview.overlay['bbox'], (5, 5, 70, 50))
        self.assertIsNone(widget.camera_preview.overlay['contour_xy'])
        self.assertEqual(widget.mask_status_chip.text(), 'mask error')

    def test_model_combo_initializes_from_current_ros_choice(self):
        widget = make_widget(None)
        profiles = MODEL_CONFIG['yolo_models']

        widget._populate_model_choices(profiles, 'carton')

        self.assertEqual(widget.model_combo.currentData(), 'carton')
        self.assertEqual(widget.model_combo.items[1], ('Carton 模型', 'carton'))
        self.assertEqual(widget.model_combo.items[2], ('Carton 分割模型', 'carton_segment'))

    def test_confirm_carton_writes_fixed_class_and_disables_editor(self):
        fake_rospy = FakeRospy(MODEL_CONFIG)
        widget = make_widget('carton', '鼠标')
        widget._latest_mask = np.full((60, 80), 255, dtype=np.uint8)
        widget._latest_mask_stamp = 123
        widget.mask_status_chip = FakeText('mask ready')
        widget.camera_preview = FakeCameraPreview()
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
        self.assertEqual(perception['yolo_reload_generation'], 1)
        self.assertEqual(widget.yolo_class_edit.text(), 'carton')
        self.assertFalse(widget.yolo_class_edit.enabled)
        self.assertIsNone(widget._latest_mask)
        self.assertIsNone(widget._latest_mask_stamp)
        self.assertEqual(widget.mask_status_chip.text(), 'mask waiting')
        self.assertIsNone(widget.camera_preview.overlay['bbox'])

    def test_confirming_same_choice_increments_reload_generation_each_time(self):
        config = deepcopy(MODEL_CONFIG)
        config['yolo_reload_generation'] = 7
        fake_rospy = FakeRospy(config)
        widget = make_widget('original', '鼠标')
        with mock.patch.object(widget_module, 'rospy', fake_rospy):
            widget.confirm_model_selection()
            widget.confirm_model_selection()

        self.assertEqual(fake_rospy.params['/perception']['yolo_reload_generation'], 9)
        self.assertEqual(len(fake_rospy.set_calls), 2)

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
        config = deepcopy(MODEL_CONFIG)
        config['yolo_reload_generation'] = 4
        fake_rospy = FakeRospy(config)
        widget = make_widget('carton')
        with mock.patch.object(widget_module, 'rospy', fake_rospy), mock.patch.object(
            widget_module,
            'resolve_yolo_model_path',
            side_effect=FileNotFoundError('carton_model/best.pt'),
        ):
            widget.confirm_model_selection()

        self.assertEqual(fake_rospy.params['/perception']['yolo_model_choice'], 'original')
        self.assertEqual(fake_rospy.params['/perception']['yolo_reload_generation'], 4)
        self.assertEqual(fake_rospy.set_calls, [])
        self.assertIn('carton_model/best.pt', widget.status.text())

    def test_detector_status_is_translated_for_gui(self):
        widget = make_widget('carton')
        widget.model_profiles = {
            'carton': {'display_name': 'Carton 模型'},
        }

        widget._update_detector_status('ready:carton:/workspace/carton_model/best.pt')

        self.assertEqual(widget.model_status_chip.text(), 'Carton 模型已就绪')

    def test_loading_and_error_status_clear_every_actionable_target_state(self):
        widget = make_widget('carton')
        widget.model_profiles = {'carton': {'display_name': 'Carton 模型'}}
        widget.last_object = object()
        widget.pregrasp_pose = object()
        widget._planned_pregrasp_pose = object()
        widget._planned_pregrasp_executable = True
        widget._planned_pregrasp_time = 1.0
        widget._planned_target_base_xyz = (0.1, 0.2, 0.3)
        widget._locked_grasp_target_base_xyz = (0.1, 0.2, 0.3)
        widget._locked_grasp_target_time = 1.0
        widget._pending_plan_pose = object()
        widget._pending_plan_token = 3
        widget._planning_active = True
        widget._plan_token = 3
        widget._last_object_receive_time = 1.0
        widget._object_stable_count = 3
        widget._last_object_base_xyz = (0.1, 0.2, 0.3)
        widget._last_target_signature = {'label': 'carton'}
        widget._latest_mask = np.full((60, 80), 255, dtype=np.uint8)
        widget._latest_mask_stamp = 123
        widget._current_object_stamp = 123
        widget._current_object_detected = True
        widget._mask_status = 'mask ready'
        widget.mask_status_chip = FakeText('mask ready')
        widget.camera_preview = FakeCameraPreview()
        widget.camera_preview.set_detection_overlay(
            (5, 5, 70, 50),
            'carton',
            contour_xy=np.array([[20, 15], [50, 15], [50, 40]], dtype=np.int32),
        )

        widget._update_detector_status('loading:carton')

        self.assertIsNone(widget.last_object)
        self.assertIsNone(widget.pregrasp_pose)
        self.assertIsNone(widget._planned_pregrasp_pose)
        self.assertIsNone(widget._locked_grasp_target_base_xyz)
        self.assertIsNone(widget._pending_plan_pose)
        self.assertFalse(widget._planning_active)
        self.assertEqual(widget._object_stable_count, 0)
        self.assertIsNone(widget._latest_mask)
        self.assertIsNone(widget._latest_mask_stamp)
        self.assertIsNone(widget._current_object_stamp)
        self.assertFalse(widget._current_object_detected)
        self.assertEqual(widget.mask_status_chip.text(), 'mask waiting')
        self.assertIsNone(widget.camera_preview.overlay['bbox'])

        widget.last_object = object()
        widget.pregrasp_pose = object()
        widget._latest_mask = np.full((60, 80), 255, dtype=np.uint8)
        widget._latest_mask_stamp = 124
        widget._current_object_stamp = 124
        widget._current_object_detected = True
        widget._update_detector_status('error:carton:torch: invalid archive: eof')

        self.assertIsNone(widget.last_object)
        self.assertIsNone(widget.pregrasp_pose)
        self.assertIsNone(widget._latest_mask)
        self.assertIsNone(widget._latest_mask_stamp)
        self.assertIsNone(widget._current_object_stamp)
        self.assertFalse(widget._current_object_detected)
        self.assertEqual(widget.status.text(), '模型加载失败：torch: invalid archive: eof')

    def test_malformed_and_unknown_detector_status_are_visible(self):
        widget = make_widget('carton')

        widget._update_detector_status('malformed')

        self.assertEqual(widget.model_status_chip.text(), '检测模型状态未知')
        self.assertIn('malformed', widget.status.text())

        widget._update_detector_status('paused:carton:maintenance')

        self.assertEqual(widget.model_status_chip.text(), '检测模型状态未知')
        self.assertIn('paused:carton:maintenance', widget.status.text())


def test_start_recognition_does_not_override_carton_class():
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

    assert fake_rospy.params['/perception']['yolo_target_class'] == 'carton'
    assert widget.yolo_class_edit.text() == 'carton'


def test_start_recognition_uses_active_ros_choice_not_unconfirmed_dropdown():
    config = deepcopy(MODEL_CONFIG)
    config['yolo_model_choice'] = 'carton'
    config['yolo_model'] = 'carton_model/best.pt'
    config['yolo_target_class'] = 'carton'
    fake_rospy = FakeRospy(config)
    widget = make_widget('original', '绿色瓶子')
    widget.enabled = FakeCheck(True)
    widget.lower_edit = FakeText('28,35,35')
    widget.upper_edit = FakeText('95,255,255')
    widget.label_edit = FakeText()
    widget.min_area = FakeValue(300)
    widget.pregrasp = FakeValue(0.08)
    widget.interpret_chip = FakeText()
    with mock.patch.object(widget_module, 'rospy', fake_rospy):
        widget.apply_params()

    assert widget.model_combo.currentData() == 'original'
    assert fake_rospy.params['/perception']['yolo_model_choice'] == 'carton'
    assert fake_rospy.params['/perception']['yolo_target_class'] == 'carton'


if __name__ == '__main__':
    unittest.main()
