#!/usr/bin/env python3
import pathlib
import sys
import unittest

import numpy as np


ROOT = pathlib.Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / 'src'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from alicia_flexible_grasp.vision.grasp6d_adapter import CameraIntrinsics
from alicia_flexible_grasp.vision.remote_grasp6d_client import (
    decode_remote_grasp_response,
    decode_rgbd_payload,
    encode_rgbd_payload,
)


class RemoteGrasp6DClientTest(unittest.TestCase):
    def test_rgbd_payload_round_trip_preserves_arrays_and_intrinsics(self):
        color = np.arange(18, dtype=np.uint8).reshape(2, 3, 3)
        depth = np.array([[1000, 1001, 0], [1200, 1300, 1400]], dtype=np.uint16)
        intrinsics = CameraIntrinsics(width=3, height=2, fx=438.6, fy=438.4, cx=1.2, cy=0.7, depth_scale=0.0001)

        payload = encode_rgbd_payload(
            color,
            depth,
            intrinsics,
            frame_id='camera_link',
            stamp_sec=12.25,
            max_candidates=8,
        )
        decoded = decode_rgbd_payload(payload)

        self.assertEqual(decoded['frame_id'], 'camera_link')
        self.assertAlmostEqual(decoded['stamp_sec'], 12.25)
        self.assertEqual(decoded['max_candidates'], 8)
        self.assertEqual(decoded['intrinsics']['depth_scale'], 0.0001)
        np.testing.assert_array_equal(decoded['color_bgr'], color)
        np.testing.assert_array_equal(decoded['depth_raw'], depth)

    def test_response_accepts_rotation_matrix_and_normalizes_quaternion(self):
        response = {
            'ok': True,
            'candidates': [
                {
                    'score': 0.91,
                    'width_m': 0.045,
                    'translation_m': [0.1, 0.2, 0.3],
                    'rotation_matrix': [
                        [1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0],
                        [0.0, 0.0, 1.0],
                    ],
                }
            ],
        }

        candidates = decode_remote_grasp_response(response)

        self.assertEqual(len(candidates), 1)
        self.assertAlmostEqual(candidates[0].score, 0.91)
        self.assertAlmostEqual(candidates[0].width_m, 0.045)
        np.testing.assert_allclose(candidates[0].translation_m, [0.1, 0.2, 0.3])
        np.testing.assert_allclose(candidates[0].quaternion_xyzw, [0.0, 0.0, 0.0, 1.0])

    def test_response_rejects_backend_error(self):
        with self.assertRaisesRegex(RuntimeError, 'checkpoint missing'):
            decode_remote_grasp_response({'ok': False, 'error': 'checkpoint missing'})


if __name__ == '__main__':
    unittest.main()
