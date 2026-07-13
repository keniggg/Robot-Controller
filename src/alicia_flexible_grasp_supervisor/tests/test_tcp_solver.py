#!/usr/bin/env python3
import math
import pathlib
import sys
import unittest

import numpy as np
from tf.transformations import euler_matrix


ROOT = pathlib.Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / 'src'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from alicia_flexible_grasp.calibration.tcp_solver import TcpCalibrationError, solve_tcp_pivot


def samples_for(tcp, fixed_point, angle_scale=1.0):
    samples = []
    angles = [
        (0.0, 0.0, 0.0),
        (0.3, 0.0, 0.0),
        (-0.3, 0.1, 0.0),
        (0.0, 0.35, 0.0),
        (0.1, -0.35, 0.2),
        (0.0, 0.0, 0.4),
        (-0.2, 0.2, -0.3),
        (0.25, -0.2, 0.3),
    ]
    for roll, pitch, yaw in angles:
        rotation = euler_matrix(roll * angle_scale, pitch * angle_scale, yaw * angle_scale)[:3, :3]
        translation = fixed_point - rotation.dot(tcp)
        samples.append({'rotation': rotation, 'translation': translation})
    return samples


class TcpSolverTest(unittest.TestCase):
    def test_recovers_tcp_and_fixed_point(self):
        tcp = np.array([-0.0002, -0.0003, 0.13118])
        fixed = np.array([0.32, -0.18, 0.11])
        result = solve_tcp_pivot(samples_for(tcp, fixed))

        np.testing.assert_allclose(result['tcp_translation'], tcp, atol=1e-9)
        np.testing.assert_allclose(result['fixed_point'], fixed, atol=1e-9)
        self.assertLess(result['rms_error'], 1e-9)
        self.assertEqual(result['sample_count'], 8)

    def test_rejects_insufficient_orientation_span(self):
        tcp = np.array([0.0, 0.0, 0.13])
        fixed = np.array([0.3, 0.0, 0.1])
        with self.assertRaises(TcpCalibrationError):
            solve_tcp_pivot(
                samples_for(tcp, fixed, angle_scale=0.05),
                min_orientation_separation_rad=math.radians(20.0),
            )

    def test_reports_small_noise_as_residual(self):
        tcp = np.array([0.001, -0.002, 0.14])
        fixed = np.array([0.25, -0.12, 0.08])
        samples = samples_for(tcp, fixed)
        samples[3]['translation'] = samples[3]['translation'] + np.array([0.001, 0.0, 0.0])
        result = solve_tcp_pivot(samples)

        self.assertGreater(result['rms_error'], 0.0)
        self.assertLess(result['rms_error'], 0.001)


if __name__ == '__main__':
    unittest.main()
