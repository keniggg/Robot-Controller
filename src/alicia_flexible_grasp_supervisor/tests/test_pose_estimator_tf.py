#!/usr/bin/env python3
import pathlib
import sys
import types
import unittest

import rospy

ROOT = pathlib.Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / 'src'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from alicia_flexible_grasp.vision.pose_estimator import PoseEstimator


class FakeTfBuffer:
    def __init__(self):
        self.calls = []

    def lookup_transform(self, target_frame, source_frame, stamp, timeout):
        self.calls.append((target_frame, source_frame, stamp, timeout))
        return types.SimpleNamespace(
            transform=types.SimpleNamespace(
                translation=types.SimpleNamespace(x=1.0, y=2.0, z=3.0),
                rotation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        )


class PoseEstimatorTfTest(unittest.TestCase):
    def test_make_poses_prefers_tf_lookup_for_base_pose(self):
        tf_buffer = FakeTfBuffer()
        estimator = PoseEstimator(
            'camera_link',
            'base_link',
            [9.0, 9.0, 9.0],
            [0.0, 0.0, 0.0, 1.0],
            [0.0, 0.0, 0.0, 1.0],
            tf_buffer=tf_buffer,
            tf_timeout_sec=0.05,
        )

        pose_cam, pose_base = estimator.make_poses([0.1, 0.2, 0.3], stamp=rospy.Time(0))

        self.assertEqual(pose_cam.header.frame_id, 'camera_link')
        self.assertEqual(pose_base.header.frame_id, 'base_link')
        self.assertAlmostEqual(pose_base.pose.position.x, 1.1)
        self.assertAlmostEqual(pose_base.pose.position.y, 2.2)
        self.assertAlmostEqual(pose_base.pose.position.z, 3.3)
        self.assertEqual(tf_buffer.calls[0][0], 'base_link')
        self.assertEqual(tf_buffer.calls[0][1], 'camera_link')


if __name__ == '__main__':
    unittest.main()
