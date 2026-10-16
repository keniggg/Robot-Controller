#!/usr/bin/env python3
import pathlib
import sys
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / 'src'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from gui.widgets.grasp_task_widget import GraspTaskWidget
from gui.widgets.perception_widget import PerceptionWidget
from gui.widgets.robot_state_widget import RobotStateWidget
from gui.widgets.tactile_widget import TactileWidget


class FakeSignal:
    def __init__(self):
        self.messages = []

    def emit(self, msg):
        self.messages.append(msg)


class FakeSubscriber:
    def __init__(self):
        self.unregistered = False

    def unregister(self):
        self.unregistered = True


class GuiRosLifecycleTest(unittest.TestCase):
    def assert_widget_callback_stops_after_shutdown(self, cls):
        widget = cls.__new__(cls)
        widget._alive = True
        widget.sig = FakeSignal()
        widget._subscriber = FakeSubscriber()

        cls._emit_if_alive(widget, 'first')
        cls._shutdown_ros(widget)
        cls._emit_if_alive(widget, 'second')

        self.assertEqual(widget.sig.messages, ['first'])
        self.assertFalse(widget._alive)
        self.assertTrue(widget._subscriber.unregistered)

    def test_tactile_widget_ignores_callbacks_after_shutdown(self):
        self.assert_widget_callback_stops_after_shutdown(TactileWidget)

    def test_robot_state_widget_ignores_callbacks_after_shutdown(self):
        self.assert_widget_callback_stops_after_shutdown(RobotStateWidget)

    def test_grasp_task_widget_ignores_callbacks_after_shutdown(self):
        self.assert_widget_callback_stops_after_shutdown(GraspTaskWidget)

    def test_perception_widget_unregisters_object_detector_status_and_mask_subscribers(self):
        widget = PerceptionWidget.__new__(PerceptionWidget)
        widget._alive = True
        widget._planning_active = False
        widget._plan_token = 0
        widget._subscriber = FakeSubscriber()
        widget._detector_status_subscriber = FakeSubscriber()
        widget._mask_subscriber = FakeSubscriber()

        PerceptionWidget._shutdown_ros(widget)

        self.assertTrue(widget._subscriber.unregistered)
        self.assertTrue(widget._detector_status_subscriber.unregistered)
        self.assertTrue(widget._mask_subscriber.unregistered)

    def test_perception_widget_ignores_mask_callbacks_after_shutdown(self):
        widget = PerceptionWidget.__new__(PerceptionWidget)
        widget._alive = True
        widget._planning_active = False
        widget._plan_token = 0
        widget.mask_signal = FakeSignal()
        widget._subscriber = None
        widget._detector_status_subscriber = None
        widget._mask_subscriber = FakeSubscriber()

        PerceptionWidget._emit_mask_if_alive(widget, 'first')
        PerceptionWidget._shutdown_ros(widget)
        PerceptionWidget._emit_mask_if_alive(widget, 'second')

        self.assertEqual(widget.mask_signal.messages, ['first'])
        self.assertTrue(widget._mask_subscriber.unregistered)


if __name__ == '__main__':
    unittest.main()
