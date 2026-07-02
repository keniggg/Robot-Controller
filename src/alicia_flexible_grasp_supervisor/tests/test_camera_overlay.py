#!/usr/bin/env python3
import pathlib
import sys
import unittest

import numpy as np


ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from gui.widgets.camera_widget import CameraWidget


class CameraOverlayTest(unittest.TestCase):
    def test_detection_overlay_draws_bbox_on_rgb_image(self):
        rgb = np.zeros((80, 100, 3), dtype=np.uint8)

        drawn = CameraWidget._draw_detection_overlay(
            rgb,
            {'bbox': (20, 15, 30, 25), 'label': '绿色圆形', 'color': (80, 255, 120)},
        )

        self.assertTrue(np.any(drawn[15, 20:50] != 0))
        self.assertTrue(np.any(drawn[15:40, 20] != 0))
        self.assertTrue(np.all(rgb == 0))

    def test_frame_gate_drops_pending_and_throttled_frames(self):
        widget = CameraWidget.__new__(CameraWidget)
        widget._alive = True
        widget.color_display_hz = 10.0
        widget.depth_display_hz = 5.0
        widget._color_pending = False
        widget._depth_pending = False
        widget._last_color_display_time = 0.0
        widget._last_depth_display_time = 0.0

        self.assertTrue(CameraWidget._begin_frame(widget, 'color', 1.0))
        self.assertTrue(widget._color_pending)
        self.assertFalse(CameraWidget._begin_frame(widget, 'color', 1.2))

        CameraWidget._end_frame(widget, 'color')
        self.assertFalse(CameraWidget._begin_frame(widget, 'color', 1.05))
        self.assertTrue(CameraWidget._begin_frame(widget, 'color', 1.11))

    def test_shutdown_unregisters_camera_subscribers(self):
        class FakeSubscriber:
            def __init__(self):
                self.unregistered = False

            def unregister(self):
                self.unregistered = True

        first = FakeSubscriber()
        second = FakeSubscriber()
        widget = CameraWidget.__new__(CameraWidget)
        widget._alive = True
        widget._subscribers = [first, second]

        CameraWidget._shutdown_ros(widget)

        self.assertFalse(widget._alive)
        self.assertTrue(first.unregistered)
        self.assertTrue(second.unregistered)


if __name__ == '__main__':
    unittest.main()
