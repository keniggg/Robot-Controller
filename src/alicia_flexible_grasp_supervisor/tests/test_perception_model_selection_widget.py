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
        self.assertEqual(perception['yolo_reload_generation'], 1)
        self.assertEqual(widget.yolo_class_edit.text(), 'carton')
        self.assertFalse(widget.yolo_class_edit.enabled)

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

        widget._update_detector_status('loading:carton')

        self.assertIsNone(widget.last_object)
        self.assertIsNone(widget.pregrasp_pose)
        self.assertIsNone(widget._planned_pregrasp_pose)
        self.assertIsNone(widget._locked_grasp_target_base_xyz)
        self.assertIsNone(widget._pending_plan_pose)
        self.assertFalse(widget._planning_active)
        self.assertEqual(widget._object_stable_count, 0)

        widget.last_object = object()
        widget.pregrasp_pose = object()
        widget._update_detector_status('error:carton:torch: invalid archive: eof')

        self.assertIsNone(widget.last_object)
        self.assertIsNone(widget.pregrasp_pose)
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
