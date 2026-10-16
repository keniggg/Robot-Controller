import math
import pathlib
import sys
import unittest

import numpy as np

SCRIPTS = pathlib.Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from handeye_invariance_metrics import (  # noqa: E402
    average_quaternion,
    matrix_from_quaternion,
    quaternion_from_matrix,
    summarize_transforms,
    transform_from_translation_quaternion,
)


class HandeyeInvarianceMetricsTest(unittest.TestCase):
    def test_average_quaternion_handles_sign_flip(self):
        q = np.array([0.0, 0.0, math.sin(0.2), math.cos(0.2)])
        avg = average_quaternion([q, -q, q])
        self.assertAlmostEqual(abs(float(np.dot(avg, q))), 1.0, places=9)

    def test_transform_summary_reports_translation_drift(self):
        transforms = [
            transform_from_translation_quaternion([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
            transform_from_translation_quaternion([0.003, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
            transform_from_translation_quaternion([-0.003, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
        ]
        summary = summarize_transforms(transforms)
        self.assertEqual(summary["count"], 3)
        self.assertAlmostEqual(summary["translation_rms_m"], math.sqrt(6e-6), places=10)
        self.assertAlmostEqual(summary["translation_max_m"], 0.003, places=10)

    def test_quaternion_matrix_round_trip(self):
        q = np.array([0.1, -0.2, 0.3, 0.92])
        q = q / np.linalg.norm(q)
        matrix = np.eye(4)
        matrix[:3, :3] = matrix_from_quaternion(q)
        recovered = quaternion_from_matrix(matrix)
        self.assertAlmostEqual(abs(float(np.dot(q, recovered))), 1.0, places=8)


if __name__ == "__main__":
    unittest.main()
