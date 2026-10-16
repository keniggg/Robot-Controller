#!/usr/bin/env python3
import math
import pathlib
import sys
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from gui.widgets.tcp_calibration_widget import TcpCalibrationWidget


class TcpCalibrationWidgetTest(unittest.TestCase):
    def test_translation_step_maps_in_meters(self):
        request = TcpCalibrationWidget.jog_request('Y-', 0.5, 2.0)
        self.assertEqual(request[:3], (0.0, -0.0005, 0.0))
        self.assertEqual(request[3:6], (0.0, 0.0, 0.0))
        self.assertTrue(request[-1])

    def test_rotation_step_maps_in_radians(self):
        request = TcpCalibrationWidget.jog_request('Rz+', 1.0, 2.5)
        self.assertAlmostEqual(request[5], math.radians(2.5))
        self.assertEqual(request[:5], (0.0, 0.0, 0.0, 0.0, 0.0))

    def test_rejects_unknown_axis(self):
        with self.assertRaises(ValueError):
            TcpCalibrationWidget.jog_request('Q+', 1.0, 2.0)


if __name__ == '__main__':
    unittest.main()
