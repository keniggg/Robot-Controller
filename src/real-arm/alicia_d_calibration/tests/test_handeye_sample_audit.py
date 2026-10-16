#!/usr/bin/env python3

import importlib.util
import math
from pathlib import Path
import unittest

import cv2
import numpy as np


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "handeye_sample_audit.py"
SPEC = importlib.util.spec_from_file_location("handeye_sample_audit", str(SCRIPT))
AUDIT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(AUDIT)


def rotation_xyz(rx, ry, rz):
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    rxm = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]], dtype=float)
    rym = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], dtype=float)
    rzm = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]], dtype=float)
    return rzm.dot(rym).dot(rxm)


def transform(rotation, translation):
    value = np.eye(4, dtype=float)
    value[:3, :3] = rotation
    value[:3, 3] = translation
    return value


class HandeyeSampleAuditTest(unittest.TestCase):
    def setUp(self):
        self.expected = transform(
            rotation_xyz(0.12, -0.35, 0.08), [-0.08, 0.01, -0.12]
        )
        fixed_board = transform(rotation_xyz(0.03, 0.02, -0.01), [0.4, -0.2, 0.1])
        self.samples = []
        for index in range(15):
            base_tool = transform(
                rotation_xyz(
                    -0.45 + 0.07 * index,
                    0.30 * math.sin(index * 0.7),
                    -0.35 + 0.05 * index,
                ),
                [
                    -0.15 + 0.018 * index,
                    -0.24 + 0.035 * math.sin(index * 0.5),
                    0.18 + 0.012 * math.cos(index * 0.9),
                ],
            )
            camera_board = np.linalg.inv(self.expected).dot(
                np.linalg.inv(base_tool)
            ).dot(fixed_board)
            self.samples.append(
                {"T_base_tool": base_tool, "T_camera_board": camera_board}
            )

    def test_recovers_known_eye_on_hand_transform(self):
        candidate = AUDIT.compute_candidate(
            self.samples, cv2.CALIB_HAND_EYE_PARK
        )
        np.testing.assert_allclose(candidate, self.expected, atol=1e-7)

    def test_full_set_board_invariance_is_near_zero(self):
        candidate = AUDIT.compute_candidate(
            self.samples, cv2.CALIB_HAND_EYE_PARK
        )
        summary = AUDIT.summarize_transforms(
            AUDIT.board_transforms(self.samples, candidate)
        )
        self.assertLess(summary["translation_max_m"], 1e-7)
        self.assertLess(summary["orientation_max_deg"], 1e-5)

    def test_cross_validation_keeps_each_sample_out_once(self):
        result = AUDIT.cross_validate(
            self.samples, cv2.CALIB_HAND_EYE_PARK, fold_count=5
        )
        held_out = []
        for fold in result["folds"]:
            held_out.extend(fold["test_indices_1based"])
        self.assertEqual(sorted(held_out), list(range(1, 16)))
        self.assertLess(result["summary"]["translation_max_m"], 1e-7)


if __name__ == "__main__":
    unittest.main()
