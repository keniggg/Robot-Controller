#!/usr/bin/env python3
import pathlib
import sys
import unittest

import numpy as np


ROOT = pathlib.Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / 'src'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from alicia_flexible_grasp.vision.grasp6d_adapter import (
    CameraIntrinsics,
    build_graspnet_input_from_rgbd,
    check_grasp6d_dependencies,
)


class Grasp6DAdapterTest(unittest.TestCase):
    def test_dependency_check_reports_missing_optional_runtime_packages(self):
        def importer(name):
            if name in ('open3d', 'MinkowskiEngine', 'graspnetAPI'):
                raise ImportError(name)
            return object()

        status = check_grasp6d_dependencies(importer=importer)

        self.assertFalse(status.available)
        self.assertIn('open3d', status.missing)
        self.assertIn('MinkowskiEngine', status.missing)
        self.assertIn('graspnetAPI', status.missing)

    def test_build_graspnet_input_reuses_grasp6d_point_cloud_projection(self):
        color_bgr = np.array(
            [
                [[0, 0, 255], [0, 255, 0]],
                [[255, 0, 0], [255, 255, 255]],
            ],
            dtype=np.uint8,
        )
        depth_raw = np.array([[1000, 0], [2000, 3000]], dtype=np.uint16)
        workspace_mask = np.array([[1, 0], [1, 0]], dtype=np.uint8)
        intrinsics = CameraIntrinsics(width=2, height=2, fx=1.0, fy=1.0, cx=0.0, cy=0.0, depth_scale=0.001)

        result = build_graspnet_input_from_rgbd(
            color_bgr,
            depth_raw,
            intrinsics,
            workspace_mask=workspace_mask,
            num_points=4,
            voxel_size=0.005,
            rng=np.random.default_rng(7),
        )

        self.assertEqual(result.model_input['point_clouds'].shape, (4, 3))
        self.assertEqual(result.model_input['coors'].shape, (4, 3))
        self.assertEqual(result.model_input['feats'].shape, (4, 3))
        self.assertTrue(np.all(result.scene_points[:, 2] > 0.0))
        self.assertTrue(np.any(np.all(np.isclose(result.scene_points, [0.0, 0.0, 1.0]), axis=1)))
        self.assertTrue(np.any(np.all(np.isclose(result.scene_points, [0.0, 2.0, 2.0]), axis=1)))


if __name__ == '__main__':
    unittest.main()
