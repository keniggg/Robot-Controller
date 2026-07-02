#!/usr/bin/env python3
import pathlib
import sys
import unittest

import cv2
import numpy as np


ROOT = pathlib.Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / 'src'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from gui.widgets.perception_widget import PerceptionWidget
from alicia_flexible_grasp.vision.object_detector import HSVObjectDetector


def hsv_is_inside_any_range(hsv, ranges):
    h, s, v = hsv
    for lower, upper in ranges:
        if lower[0] <= h <= upper[0] and lower[1] <= s <= upper[1] and lower[2] <= v <= upper[2]:
            return True
    return False


class GreenCircleDetectionTest(unittest.TestCase):
    def test_green_circle_description_includes_olive_green(self):
        parsed = PerceptionWidget._parse_description(PerceptionWidget, '绿色圆形')

        self.assertEqual(parsed['shape'], 'circle')
        self.assertTrue(
            hsv_is_inside_any_range((32, 184, 100), parsed['hsv_ranges']),
            parsed['hsv_ranges'],
        )
        self.assertFalse(
            hsv_is_inside_any_range((23, 180, 150), parsed['hsv_ranges']),
            parsed['hsv_ranges'],
        )

    def test_imperfect_yellow_green_circle_passes_circle_filter(self):
        hsv_color = np.uint8([[[32, 184, 100]]])
        bgr_color = cv2.cvtColor(hsv_color, cv2.COLOR_HSV2BGR)[0, 0]
        img = np.zeros((160, 160, 3), dtype=np.uint8)
        cv2.circle(img, (80, 80), 34, tuple(int(v) for v in bgr_color), -1)
        cv2.rectangle(img, (80, 46), (88, 80), (0, 0, 0), -1)

        detector = HSVObjectDetector(
            hsv_ranges=[([20, 25, 35], [95, 255, 255])],
            min_area=300,
            shape='circle',
        )
        det, _ = detector.detect(img)

        self.assertIsNotNone(det)
        self.assertGreater(det['area'], 2500)

    def test_circle_filter_prefers_true_circle_over_larger_oval_distractor(self):
        hsv_color = np.uint8([[[32, 184, 100]]])
        bgr_color = cv2.cvtColor(hsv_color, cv2.COLOR_HSV2BGR)[0, 0]
        img = np.zeros((180, 260, 3), dtype=np.uint8)
        cv2.circle(img, (62, 90), 22, tuple(int(v) for v in bgr_color), -1)
        cv2.ellipse(img, (175, 90), (38, 28), 0, 0, 360, tuple(int(v) for v in bgr_color), -1)

        detector = HSVObjectDetector(
            hsv_ranges=[([28, 35, 35], [95, 255, 255])],
            min_area=300,
            shape='circle',
        )
        det, _ = detector.detect(img)

        self.assertIsNotNone(det)
        self.assertLess(det['u'], 100)

    def test_preferred_center_keeps_locked_target_over_larger_far_candidate(self):
        hsv_color = np.uint8([[[42, 190, 130]]])
        bgr_color = cv2.cvtColor(hsv_color, cv2.COLOR_HSV2BGR)[0, 0]
        img = np.zeros((180, 280, 3), dtype=np.uint8)
        cv2.circle(img, (70, 90), 18, tuple(int(v) for v in bgr_color), -1)
        cv2.circle(img, (205, 90), 30, tuple(int(v) for v in bgr_color), -1)

        detector = HSVObjectDetector(
            hsv_ranges=[([28, 35, 35], [95, 255, 255])],
            min_area=300,
            shape='circle',
        )
        det, _ = detector.detect(img, preferred_uv=(70, 90), max_preferred_distance_px=70)

        self.assertIsNotNone(det)
        self.assertLess(det['u'], 100)


if __name__ == '__main__':
    unittest.main()
