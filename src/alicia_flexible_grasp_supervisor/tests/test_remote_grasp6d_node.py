#!/usr/bin/env python3
import importlib.util
import pathlib
import sys
import unittest

import numpy as np
from geometry_msgs.msg import PoseStamped


ROOT = pathlib.Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / 'src'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from alicia_flexible_grasp.vision.remote_grasp6d_client import RemoteGraspCandidate


SCRIPT = ROOT / 'scripts' / 'remote_grasp6d_node.py'
spec = importlib.util.spec_from_file_location('remote_grasp6d_node', str(SCRIPT))
remote_node = importlib.util.module_from_spec(spec)
spec.loader.exec_module(remote_node)


class FakePoseEstimator:
    def make_base_pose_from_camera_pose(self, xyz, quat, stamp=None, camera_frame=None):
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.header.stamp = stamp
        pose.pose.position.x = float(xyz[0]) + 1.0
        pose.pose.position.y = float(xyz[1])
        pose.pose.position.z = float(xyz[2])
        pose.pose.orientation.x = float(quat[0])
        pose.pose.orientation.y = float(quat[1])
        pose.pose.orientation.z = float(quat[2])
        pose.pose.orientation.w = float(quat[3])
        return pose


class RecordingPoseEstimator:
    def __init__(self):
        self.calls = []

    def make_base_pose_from_camera_pose(self, xyz, quat, stamp=None, camera_frame=None):
        self.calls.append((np.asarray(xyz, dtype=float), np.asarray(quat, dtype=float), camera_frame))
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.header.stamp = stamp
        pose.pose.position.x = float(xyz[0])
        pose.pose.position.y = float(xyz[1])
        pose.pose.position.z = float(xyz[2])
        pose.pose.orientation.x = float(quat[0])
        pose.pose.orientation.y = float(quat[1])
        pose.pose.orientation.z = float(quat[2])
        pose.pose.orientation.w = float(quat[3])
        return pose


class FakeServiceResponse:
    def __init__(self, success=True, message='ok'):
        self.success = bool(success)
        self.message = message


class RemoteGrasp6DNodeTest(unittest.TestCase):
    def test_latest_rgbd_buffer_supports_manual_snapshot_after_auto_consumption(self):
        buffer = remote_node.LatestRgbdBuffer()
        color = np.zeros((2, 2, 3), dtype=np.uint8)
        depth = np.ones((2, 2), dtype=np.uint16)

        buffer.update_color(color, stamp='stamp', frame_id='camera_link')
        buffer.update_depth(depth)

        first = buffer.take_latest()
        second = buffer.take_latest()
        manual = buffer.snapshot_latest()

        self.assertIsNotNone(first)
        self.assertIsNone(second)
        self.assertIsNotNone(manual)
        np.testing.assert_array_equal(manual[0], color)
        np.testing.assert_array_equal(manual[1], depth)
        self.assertEqual(manual[2], 'stamp')
        self.assertEqual(manual[3], 'camera_link')

    def test_selects_first_reachable_candidate_after_camera_to_base_transform(self):
        candidates = [
            RemoteGraspCandidate(0.99, np.array([0.10, 0.0, 0.2]), np.array([0.0, 0.0, 0.0, 1.0]), 0.04),
            RemoteGraspCandidate(0.80, np.array([0.65, 0.0, 0.2]), np.array([0.0, 0.0, 0.0, 1.0]), 0.04),
        ]

        selected, pose = remote_node.select_first_reachable_candidate(
            candidates,
            FakePoseEstimator(),
            lambda pose: pose.pose.position.x > 1.5,
            stamp=None,
            camera_frame='camera_link',
            candidate_frame_convention='ros_camera_link',
        )

        self.assertIs(selected, candidates[1])
        self.assertAlmostEqual(pose.pose.position.x, 1.65)

    def test_converts_opencv_optical_candidate_to_ros_camera_link(self):
        candidate = RemoteGraspCandidate(
            score=0.9,
            translation_m=np.array([0.01, 0.02, 0.30]),
            quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
            width_m=0.05,
        )

        converted = remote_node.convert_candidate_to_camera_link(candidate, 'opencv_optical')
        converted_rotation = remote_node.quaternion_matrix(converted.quaternion_xyzw)[:3, :3]

        np.testing.assert_allclose(converted.translation_m, np.array([0.30, -0.01, -0.02]))
        np.testing.assert_allclose(converted_rotation, remote_node.OPTICAL_TO_ROS_CAMERA, atol=1e-7)
        self.assertAlmostEqual(converted.score, candidate.score)
        self.assertAlmostEqual(converted.width_m, candidate.width_m)

    def test_reachability_receives_converted_opencv_optical_candidate(self):
        candidate = RemoteGraspCandidate(
            score=0.9,
            translation_m=np.array([0.01, 0.02, 0.30]),
            quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
            width_m=0.05,
        )
        estimator = RecordingPoseEstimator()

        selected, pose = remote_node.select_first_reachable_candidate(
            [candidate],
            estimator,
            lambda pose: pose.pose.position.x > 0.29 and pose.pose.position.z < 0.0,
            stamp=None,
            camera_frame='camera_link',
            candidate_frame_convention='opencv_optical',
        )

        self.assertIsNotNone(selected)
        np.testing.assert_allclose(estimator.calls[0][0], np.array([0.30, -0.01, -0.02]))
        self.assertAlmostEqual(pose.pose.position.x, 0.30)
        self.assertAlmostEqual(pose.pose.position.z, -0.02)

    def test_pose_array_contains_pregrasp_approach_grasp_and_lift(self):
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.pose.position.x = 0.4
        pose.pose.position.y = 0.1
        pose.pose.position.z = 0.2
        pose.pose.orientation.w = 1.0

        array = remote_node.make_grasp_plan_pose_array(
            pose,
            stamp=None,
            grasp_config={
                'pregrasp_distance_m': 0.08,
                'final_approach_offset_m': 0.015,
                'lift_height_m': 0.05,
            },
        )

        self.assertEqual(array.header.frame_id, 'base_link')
        self.assertEqual(len(array.poses), 4)
        self.assertAlmostEqual(array.poses[0].position.x, 0.32)
        self.assertAlmostEqual(array.poses[1].position.x, 0.385)
        self.assertAlmostEqual(array.poses[2].position.x, 0.4)
        self.assertAlmostEqual(array.poses[3].position.z, 0.25)

    def test_plan_reachable_rejects_position_only_fallback_when_execute_disallows_it(self):
        node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
        node.allow_position_only_fallback = False
        node._position_only_rejected_count = 0

        original_wait_for_service = remote_node.rospy.wait_for_service
        original_service_proxy = remote_node.rospy.ServiceProxy
        original_logwarn_throttle = remote_node.rospy.logwarn_throttle
        remote_node.rospy.wait_for_service = lambda *args, **kwargs: None
        remote_node.rospy.ServiceProxy = lambda *_args, **_kwargs: (
            lambda _pose, _execute: FakeServiceResponse(
                True,
                'planned with position-only fallback: target xyz=(0.1, 0.2, 0.3)',
            )
        )
        remote_node.rospy.logwarn_throttle = lambda *args, **kwargs: None
        try:
            self.assertFalse(node._plan_reachable(PoseStamped()))
        finally:
            remote_node.rospy.wait_for_service = original_wait_for_service
            remote_node.rospy.ServiceProxy = original_service_proxy
            remote_node.rospy.logwarn_throttle = original_logwarn_throttle

        self.assertEqual(node._position_only_rejected_count, 1)

    def test_plan_reachable_accepts_strict_pose_plan(self):
        node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
        node.allow_position_only_fallback = False
        node._position_only_rejected_count = 0

        original_wait_for_service = remote_node.rospy.wait_for_service
        original_service_proxy = remote_node.rospy.ServiceProxy
        remote_node.rospy.wait_for_service = lambda *args, **kwargs: None
        remote_node.rospy.ServiceProxy = lambda *_args, **_kwargs: (
            lambda _pose, _execute: FakeServiceResponse(
                True,
                'planned with candidate orientation current: target xyz=(0.1, 0.2, 0.3)',
            )
        )
        try:
            self.assertTrue(node._plan_reachable(PoseStamped()))
        finally:
            remote_node.rospy.wait_for_service = original_wait_for_service
            remote_node.rospy.ServiceProxy = original_service_proxy

        self.assertEqual(node._position_only_rejected_count, 0)


if __name__ == '__main__':
    unittest.main()
