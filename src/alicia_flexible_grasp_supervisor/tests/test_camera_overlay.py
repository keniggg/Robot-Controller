#!/usr/bin/env python3
import pathlib
import sys
import unittest

import numpy as np
from PyQt5 import QtCore


ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from gui.widgets.camera_widget import CameraWidget


class CameraOverlayTest(unittest.TestCase):
    def test_color_update_renders_only_color_stream(self):
        class FakeSignal:
            def emit(self, *_args):
                pass

        widget = CameraWidget.__new__(CameraWidget)
        widget._alive = True
        widget.color_frame_updated = FakeSignal()
        widget._refresh_color_pixmap = lambda: None
        rendered_streams = []
        widget._render_pixmaps = (
            lambda stream=None: rendered_streams.append(stream)
        )

        CameraWidget.update_color_image(
            widget,
            np.zeros((60, 80, 3), dtype=np.uint8),
        )

        self.assertEqual(rendered_streams, ['color'])

    def test_detection_overlay_draws_bbox_on_rgb_image(self):
        rgb = np.zeros((80, 100, 3), dtype=np.uint8)

        drawn = CameraWidget._draw_detection_overlay(
            rgb,
            {'bbox': (20, 15, 30, 25), 'label': '绿色圆形', 'color': (80, 255, 120)},
        )

        self.assertTrue(np.any(drawn[15, 20:50] != 0))
        self.assertTrue(np.any(drawn[15:40, 20] != 0))
        self.assertTrue(np.all(rgb == 0))

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

    def test_detection_overlay_draws_contour_away_from_bbox(self):
        rgb = np.zeros((60, 80, 3), dtype=np.uint8)
        contour = np.array([[20, 15], [50, 15], [50, 40], [20, 40]], dtype=np.int32)

        drawn = CameraWidget._draw_detection_overlay(
            rgb,
            {
                'bbox': (5, 5, 70, 50),
                'label': '',
                'color': (80, 255, 120),
                'contour_xy': contour,
            },
        )

        self.assertTrue(np.any(drawn[15, 20:51] != 0))
        self.assertTrue(np.any(drawn[16, 20:51] != 0))
        self.assertTrue(np.all(rgb == 0))

    def test_overlay_change_rebuilds_cached_color_pixmap(self):
        widget = CameraWidget.__new__(CameraWidget)
        widget._alive = True
        widget._last_color_rgb = np.zeros((80, 100, 3), dtype=np.uint8)
        refreshed = []
        widget._refresh_color_pixmap = lambda: refreshed.append(True)
        widget._render_pixmaps = lambda *_args: None

        CameraWidget.set_detection_overlay(
            widget,
            (20, 15, 30, 25),
            'mouse',
            (80, 255, 120),
        )

        self.assertEqual(refreshed, [True])

    def test_detection_overlay_copies_contour_points(self):
        widget = CameraWidget.__new__(CameraWidget)
        widget._alive = True
        widget._last_color_rgb = None
        widget._render_pixmaps = lambda *_args: None
        contour = np.array([[20, 15], [50, 15], [50, 40]], dtype=np.int64)

        CameraWidget.set_detection_overlay(
            widget,
            (20, 15, 30, 25),
            'carton',
            (80, 255, 120),
            contour,
        )
        contour[0] = (0, 0)

        stored = widget._detection_overlay['contour_xy']
        self.assertEqual(stored.dtype, np.int32)
        self.assertEqual(tuple(stored[0]), (20, 15))

    def test_color_display_rgb_redraws_overlay_from_raw_frame(self):
        widget = CameraWidget.__new__(CameraWidget)
        widget._last_color_rgb = np.zeros((80, 100, 3), dtype=np.uint8)
        widget._detection_overlay = {
            'bbox': (20, 15, 30, 25),
            'label': 'mouse',
            'color': (80, 255, 120),
        }

        drawn = CameraWidget._color_display_rgb(widget)

        self.assertTrue(np.any(drawn[15, 20:50] != 0))
        self.assertTrue(np.all(widget._last_color_rgb == 0))

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

    def test_scale_transform_defaults_to_fast_for_lower_latency(self):
        widget = CameraWidget.__new__(CameraWidget)
        widget.scale_smooth = False

        self.assertEqual(CameraWidget._scale_transform_mode(widget), QtCore.Qt.FastTransformation)

        widget.scale_smooth = True
        self.assertEqual(CameraWidget._scale_transform_mode(widget), QtCore.Qt.SmoothTransformation)

    def test_hidden_camera_widget_does_not_process_stream_frames(self):
        widget = CameraWidget.__new__(CameraWidget)
        widget.isVisible = lambda: False

        self.assertFalse(CameraWidget._stream_visible(widget))

        widget.isVisible = lambda: True
        self.assertTrue(CameraWidget._stream_visible(widget))

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
