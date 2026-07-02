#!/usr/bin/env python3
import pathlib
import sys
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from gui.widgets.grasp_task_widget import GraspTaskWidget
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


if __name__ == '__main__':
    unittest.main()
