#!/usr/bin/env python3
import pathlib
import sys
import types
import unittest

import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler

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

    def test_make_pose_transforms_6d_camera_pose_with_static_fallback(self):
        estimator = PoseEstimator(
            'camera_link',
            'base_link',
            [1.0, 2.0, 3.0],
            [0.0, 0.0, 0.0, 1.0],
            [0.0, 0.0, 0.0, 1.0],
            tf_buffer=None,
        )
        q_camera = quaternion_from_euler(0.0, 0.0, 0.5)

        pose_base = estimator.make_base_pose_from_camera_pose([0.1, 0.2, 0.3], q_camera, stamp=rospy.Time(0))

        self.assertEqual(pose_base.header.frame_id, 'base_link')
        self.assertAlmostEqual(pose_base.pose.position.x, 1.1)
        self.assertAlmostEqual(pose_base.pose.position.y, 2.2)
        self.assertAlmostEqual(pose_base.pose.position.z, 3.3)
        yaw = euler_from_quaternion([
            pose_base.pose.orientation.x,
            pose_base.pose.orientation.y,
            pose_base.pose.orientation.z,
            pose_base.pose.orientation.w,
        ])[2]
        self.assertAlmostEqual(yaw, 0.5)

    def test_make_pose_applies_handeye_rotation_to_6d_orientation(self):
        base_from_camera = quaternion_from_euler(0.0, 0.0, 1.0)
        estimator = PoseEstimator(
            'camera_link',
            'base_link',
            [0.0, 0.0, 0.0],
            base_from_camera,
            [0.0, 0.0, 0.0, 1.0],
            tf_buffer=None,
        )

        pose_base = estimator.make_base_pose_from_camera_pose(
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            stamp=rospy.Time(0),
        )

        self.assertAlmostEqual(pose_base.pose.position.x, 0.5403023058681398, places=6)
        self.assertAlmostEqual(pose_base.pose.position.y, 0.8414709848078965, places=6)
        yaw = euler_from_quaternion([
            pose_base.pose.orientation.x,
            pose_base.pose.orientation.y,
            pose_base.pose.orientation.z,
            pose_base.pose.orientation.w,
        ])[2]
        self.assertAlmostEqual(yaw, 1.0)


if __name__ == '__main__':
    unittest.main()
