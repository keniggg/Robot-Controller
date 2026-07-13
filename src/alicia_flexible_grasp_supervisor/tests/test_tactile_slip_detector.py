#!/usr/bin/env python3
import unittest
from unittest import mock

from alicia_flexible_grasp.tactile.tactile_filter import SlipDetector


class SlipDetectorTest(unittest.TestCase):
    def test_static_center_noise_does_not_trigger(self):
        detector = SlipDetector(
            window_sec=0.25,
            drop_ratio=0.30,
            center_shift_threshold=0.05,
        )
        centers = [
            (1.90, 4.90, 1.75, 5.23),
            (1.91, 4.90, 1.76, 5.23),
            (1.91, 4.91, 1.76, 5.24),
        ]
        with mock.patch(
            'alicia_flexible_grasp.tactile.tactile_filter.time.time',
            side_effect=[0.0, 0.12, 0.25],
        ):
            self.assertFalse(detector.update(2800.0, centers[0]))
            self.assertFalse(detector.update(2802.0, centers[1]))
            self.assertFalse(detector.update(2798.0, centers[2]))

    def test_center_motion_triggers_without_force_drop(self):
        detector = SlipDetector(
            window_sec=0.25,
            drop_ratio=0.30,
            center_shift_threshold=0.05,
        )
        with mock.patch(
            'alicia_flexible_grasp.tactile.tactile_filter.time.time',
            side_effect=[0.0, 0.12, 0.25],
        ):
            self.assertFalse(detector.update(2800.0, (1.90, 4.90, 1.75, 5.23)))
            self.assertFalse(detector.update(3400.0, (1.91, 4.90, 1.76, 5.23)))
            self.assertTrue(detector.update(4300.0, (1.96, 4.90, 1.76, 5.23)))

    def test_force_drop_remains_supported(self):
        detector = SlipDetector(
            window_sec=0.25,
            drop_ratio=0.30,
            center_shift_threshold=0.05,
        )
        with mock.patch(
            'alicia_flexible_grasp.tactile.tactile_filter.time.time',
            side_effect=[0.0, 0.12, 0.25],
        ):
            self.assertFalse(detector.update(3000.0))
            self.assertFalse(detector.update(3000.0))
            self.assertTrue(detector.update(2000.0))


if __name__ == '__main__':
    unittest.main()
