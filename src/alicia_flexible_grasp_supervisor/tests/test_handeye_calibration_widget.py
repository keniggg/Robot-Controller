#!/usr/bin/env python3
import os
import pathlib
import sys
import unittest
from unittest import mock


os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

ROOT = pathlib.Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / 'src'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from PyQt5 import QtWidgets

from gui.widgets import handeye_calibration_widget as module
from gui.widgets.handeye_calibration_widget import (
    OnDemandRgbView,
    evaluate_charuco_quality,
    format_charuco_quality,
)


class FakeSubscriber:
    def __init__(self):
        self.unregistered = False

    def unregister(self):
        self.unregistered = True


class HandeyeCalibrationWidgetTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance()
        if cls.app is None:
            cls.app = QtWidgets.QApplication(['handeye-widget-test'])

    def test_cc200_quality_gate_accepts_fresh_well_observed_board(self):
        quality = {
            'stamp': 99.8,
            'pose_ok': True,
            'charuco_count': 60,
            'marker_count': 42,
            'edge_clearance_px': 20.0,
            'reprojection_rms_px': 0.42,
        }

        ok, reason, detail = evaluate_charuco_quality(quality, 100.0)

        self.assertTrue(ok)
        self.assertEqual(reason, '质量合格')
        self.assertAlmostEqual(detail['age_ms'], 200.0)
        self.assertIn('corners 60', format_charuco_quality(ok, reason, detail))

    def test_quality_gate_rejects_stale_or_edge_clipped_board(self):
        base = {
            'stamp': 99.0,
            'pose_ok': True,
            'charuco_count': 60,
            'marker_count': 42,
            'edge_clearance_px': 20.0,
            'reprojection_rms_px': 0.42,
        }
        ok, reason, _detail = evaluate_charuco_quality(base, 100.0)
        self.assertFalse(ok)
        self.assertEqual(reason, '质量数据已过期')

        base['stamp'] = 99.8
        base['edge_clearance_px'] = 8.0
        ok, reason, _detail = evaluate_charuco_quality(base, 100.0)
        self.assertFalse(ok)
        self.assertEqual(reason, '标定板距离图像边缘过近')

        base['edge_clearance_px'] = 20.0
        base['reprojection_rms_px'] = float('nan')
        ok, reason, _detail = evaluate_charuco_quality(base, 100.0)
        self.assertFalse(ok)
        self.assertEqual(reason, '重投影误差过大')

    def test_rgb_view_subscribes_only_between_open_and_close(self):
        fake_subscriber = FakeSubscriber()
        with mock.patch.object(module, 'CvBridge', return_value=object()), mock.patch.object(
            module, 'cv2', object()
        ), mock.patch.object(
            module.rospy, 'Subscriber', return_value=fake_subscriber
        ) as subscriber_factory:
            view = OnDemandRgbView('/charuco/result')
            self.assertFalse(view.streaming)
            subscriber_factory.assert_not_called()

            self.assertTrue(view.start())
            self.assertTrue(view.streaming)
            subscriber_factory.assert_called_once()

            view.stop()
            self.assertFalse(view.streaming)
            self.assertTrue(fake_subscriber.unregistered)
            view.close()


if __name__ == '__main__':
    unittest.main()
