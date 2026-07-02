#!/usr/bin/env python3
import importlib.util
import pathlib
import sys
import unittest

import numpy as np


ROOT = pathlib.Path(__file__).resolve().parents[1]
PERCEPTION_NODE = ROOT / 'scripts' / 'perception_node.py'
SRC_DIR = ROOT / 'src'
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))


def load_perception_node():
    spec = importlib.util.spec_from_file_location('perception_node_under_test', str(PERCEPTION_NODE))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def make_node(module, depth):
    node = module.PerceptionNode.__new__(module.PerceptionNode)
    node.depth = depth
    node.depth_scale = 0.0001
    node.depth_roi_center_fraction = 0.5
    node.depth_roi_percentile = 50.0
    node.depth_min_m = 0.03
    node.depth_max_m = 2.0
    node.depth_min_valid_px = 24
    node._last_depth_source = 'none'
    node._last_depth_valid_count = 0
    return node


class PerceptionDepthSelectionTest(unittest.TestCase):
    def test_center_roi_depth_wins_over_bad_center_pixel(self):
        module = load_perception_node()
        depth = np.full((80, 80), 3000, dtype=np.uint16)
        depth[30:50, 30:50] = 2000
        depth[40, 40] = 900
        node = make_node(module, depth)

        z = module.PerceptionNode.depth_m_at_detection(
            node,
            {'bbox': (20, 20, 40, 40)},
            40,
            40,
        )

        self.assertAlmostEqual(z, 0.2, places=3)
        self.assertEqual(node._last_depth_source, 'center_roi')

    def test_bbox_roi_is_used_when_center_roi_has_no_valid_depth(self):
        module = load_perception_node()
        depth = np.zeros((80, 80), dtype=np.uint16)
        depth[20:60, 20:60] = 1800
        depth[30:50, 30:50] = 0
        node = make_node(module, depth)

        z = module.PerceptionNode.depth_m_at_detection(
            node,
            {'bbox': (20, 20, 40, 40)},
            40,
            40,
        )

        self.assertAlmostEqual(z, 0.18, places=3)
        self.assertEqual(node._last_depth_source, 'bbox_roi')

    def test_float_depth_images_are_treated_as_meters(self):
        module = load_perception_node()
        depth = np.full((80, 80), 0.45, dtype=np.float32)
        depth[30:50, 30:50] = 0.22
        node = make_node(module, depth)

        z = module.PerceptionNode.depth_m_at_detection(
            node,
            {'bbox': (20, 20, 40, 40)},
            40,
            40,
        )

        self.assertAlmostEqual(z, 0.22, places=3)
        self.assertEqual(node._last_depth_source, 'center_roi')

    def test_ros_camera_link_projection_converts_from_opencv_optical(self):
        module = load_perception_node()
        node = module.PerceptionNode.__new__(module.PerceptionNode)
        node.projection_frame_convention = 'ros_camera_link'

        converted = module.PerceptionNode._projected_point_for_camera_frame(node, [0.1, 0.2, 0.3])

        self.assertEqual(converted, [0.3, -0.1, -0.2])

    def test_opencv_optical_projection_keeps_projected_point(self):
        module = load_perception_node()
        node = module.PerceptionNode.__new__(module.PerceptionNode)
        node.projection_frame_convention = 'opencv_optical'

        converted = module.PerceptionNode._projected_point_for_camera_frame(node, [0.1, 0.2, 0.3])

        self.assertEqual(converted, [0.1, 0.2, 0.3])


if __name__ == '__main__':
    unittest.main()
