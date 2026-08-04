#!/usr/bin/env python3

import importlib.util
from pathlib import Path
import unittest

import numpy as np


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "charuco_tracker.py"
SPEC = importlib.util.spec_from_file_location("charuco_tracker", str(SCRIPT))
TRACKER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(TRACKER)


class CharucoTrackerHelpersTest(unittest.TestCase):
    def test_detection_upscale_preserves_source_image(self):
        source = np.arange(12, dtype=np.uint8).reshape(3, 4)
        scaled, scale = TRACKER.prepare_detection_image(source, 2.0)
        self.assertEqual(scaled.shape, (6, 8))
        self.assertEqual(scale, 2.0)
        np.testing.assert_array_equal(source, np.arange(12, dtype=np.uint8).reshape(3, 4))

    def test_detection_scale_one_reuses_source_image(self):
        source = np.zeros((3, 4), dtype=np.uint8)
        scaled, scale = TRACKER.prepare_detection_image(source, 1.0)
        self.assertIs(scaled, source)
        self.assertEqual(scale, 1.0)

    def test_reprojects_scaled_corners_to_source_pixels(self):
        corners = np.array([[[20.0, 10.0], [40.0, 30.0]]], dtype=np.float32)
        restored = TRACKER.rescale_corners_to_source(corners, 2.0)
        np.testing.assert_allclose(
            restored,
            np.array([[[10.0, 5.0], [20.0, 15.0]]], dtype=np.float32),
        )

    def test_reprojects_cropped_scaled_corners_to_source_pixels(self):
        corners = np.array([[[20.0, 10.0], [40.0, 30.0]]], dtype=np.float32)
        restored = TRACKER.rescale_corners_to_source(
            corners, 2.0, offset_xy=(100.0, 50.0)
        )
        np.testing.assert_allclose(
            restored,
            np.array([[[110.0, 55.0], [120.0, 65.0]]], dtype=np.float32),
        )

    def test_marker_roi_adds_padding_and_clamps_to_image(self):
        corners = [
            np.array([[[2.0, 3.0], [20.0, 3.0], [20.0, 15.0], [2.0, 15.0]]])
        ]
        self.assertEqual(
            TRACKER.marker_roi(corners, (30, 40), 10.0),
            (0, 0, 31, 26),
        )

    def test_marker_roi_rejects_missing_or_nonfinite_corners(self):
        self.assertIsNone(TRACKER.marker_roi([], (30, 40), 10.0))
        invalid = [np.array([[[float("nan"), 1.0]]])]
        self.assertIsNone(TRACKER.marker_roi(invalid, (30, 40), 10.0))

    def test_rejects_unbounded_detection_scale(self):
        source = np.zeros((3, 4), dtype=np.uint8)
        for scale in (0.5, 5.0, float("nan")):
            with self.subTest(scale=scale):
                with self.assertRaises(ValueError):
                    TRACKER.prepare_detection_image(source, scale)


if __name__ == "__main__":
    unittest.main()
