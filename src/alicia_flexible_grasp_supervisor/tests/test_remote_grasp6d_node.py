#!/usr/bin/env python3
import importlib.util
import pathlib
import sys
import types
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


class IdentityPointPoseEstimator:
    camera_frame = 'camera_link'

    def make_poses(self, xyz, stamp=None, camera_frame=None):
        pose_cam = PoseStamped()
        pose_base = PoseStamped()
        pose_cam.header.frame_id = camera_frame or self.camera_frame
        pose_base.header.frame_id = 'base_link'
        for pose in (pose_cam, pose_base):
            pose.header.stamp = stamp
            pose.pose.position.x = float(xyz[0])
            pose.pose.position.y = float(xyz[1])
            pose.pose.position.z = float(xyz[2])
            pose.pose.orientation.w = 1.0
        return pose_cam, pose_base


class FakeServiceResponse:
    def __init__(self, success=True, message='ok'):
        self.success = bool(success)
        self.message = message


class DummyLock:
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        return False


class FakeTf2Module:
    class Buffer:
        pass

    class TransformListener:
        def __init__(self, buffer):
            self.buffer = buffer


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

    def test_selects_next_candidate_when_target_gate_rejects_first(self):
        candidates = [
            RemoteGraspCandidate(0.99, np.array([0.10, 0.0, 0.2]), np.array([0.0, 0.0, 0.0, 1.0]), 0.04),
            RemoteGraspCandidate(0.80, np.array([0.20, 0.0, 0.2]), np.array([0.0, 0.0, 0.0, 1.0]), 0.04),
        ]
        seen_x = []

        def candidate_filter(_candidate, _camera_candidate, pose):
            seen_x.append(pose.pose.position.x)
            return pose.pose.position.x > 1.15

        selected, pose = remote_node.select_first_reachable_candidate(
            candidates,
            FakePoseEstimator(),
            lambda _pose: True,
            stamp=None,
            camera_frame='camera_link',
            candidate_frame_convention='ros_camera_link',
            candidate_filter_fn=candidate_filter,
        )

        self.assertIs(selected, candidates[1])
        self.assertAlmostEqual(pose.pose.position.x, 1.20)
        self.assertEqual(seen_x, [1.10, 1.20])

    def test_target_rank_prefers_closest_reachable_candidate_over_higher_score(self):
        candidates = [
            RemoteGraspCandidate(0.99, np.array([0.035, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0]), 0.04),
            RemoteGraspCandidate(0.70, np.array([0.010, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0]), 0.04),
        ]
        target_x = 1.0

        def rank_by_target_distance(_candidate, _camera_candidate, pose):
            return abs(float(pose.pose.position.x) - target_x)

        selected, pose = remote_node.select_first_reachable_candidate(
            candidates,
            FakePoseEstimator(),
            lambda _pose: True,
            stamp=None,
            camera_frame='camera_link',
            candidate_frame_convention='ros_camera_link',
            candidate_rank_fn=rank_by_target_distance,
        )

        self.assertIs(selected, candidates[1])
        self.assertAlmostEqual(pose.pose.position.x, 1.01)

    def test_converts_opencv_optical_candidate_to_ros_camera_link(self):
        candidate = RemoteGraspCandidate(
            score=0.9,
            translation_m=np.array([0.01, 0.02, 0.30]),
            quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
            width_m=0.05,
            depth_m=0.03,
        )

        converted = remote_node.convert_candidate_to_camera_link(candidate, 'opencv_optical')
        converted_rotation = remote_node.quaternion_matrix(converted.quaternion_xyzw)[:3, :3]

        np.testing.assert_allclose(converted.translation_m, np.array([0.30, -0.01, -0.02]))
        np.testing.assert_allclose(converted_rotation, remote_node.OPTICAL_TO_ROS_CAMERA, atol=1e-7)
        self.assertAlmostEqual(converted.score, candidate.score)
        self.assertAlmostEqual(converted.width_m, candidate.width_m)
        self.assertAlmostEqual(converted.depth_m, candidate.depth_m)

    def test_model_grasp_x_axis_is_aligned_to_physical_tool_z_axis(self):
        candidate = RemoteGraspCandidate(
            score=0.9,
            translation_m=np.array([0.30, 0.0, 0.0]),
            quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
            width_m=0.05,
            depth_m=0.03,
        )
        estimator = RecordingPoseEstimator()
        correction = remote_node.quaternion_from_euler(0.0, np.pi * 0.5, 0.0)

        selected, _pose = remote_node.select_first_reachable_candidate(
            [candidate],
            estimator,
            lambda _pose: True,
            stamp=None,
            camera_frame='camera_link',
            candidate_frame_convention='ros_camera_link',
            model_grasp_to_tool_quaternion=correction,
        )

        rotation = remote_node.quaternion_matrix(estimator.calls[0][1])[:3, :3]
        self.assertIsNotNone(selected)
        np.testing.assert_allclose(rotation[:, 2], [1.0, 0.0, 0.0], atol=1e-7)
        np.testing.assert_allclose(rotation[:, 1], [0.0, 1.0, 0.0], atol=1e-7)
        self.assertAlmostEqual(selected.depth_m, 0.03)

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

    def test_eye_in_hand_projection_places_forward_target_at_image_center(self):
        tool_pose = PoseStamped()
        tool_pose.pose.orientation.w = 1.0
        intrinsics = remote_node.CameraIntrinsics(
            width=640,
            height=480,
            fx=440.0,
            fy=440.0,
            cx=320.0,
            cy=240.0,
            depth_scale=0.001,
        )

        u, v, depth = remote_node.project_base_target_at_tool_pose(
            tool_pose,
            [0.50, 0.0, 0.0],
            np.eye(4),
            intrinsics,
        )

        self.assertAlmostEqual(u, 320.0)
        self.assertAlmostEqual(v, 240.0)
        self.assertAlmostEqual(depth, 0.50)

    def test_eye_in_hand_projection_reports_target_behind_camera(self):
        tool_pose = PoseStamped()
        tool_pose.pose.orientation.z = 1.0
        tool_pose.pose.orientation.w = 0.0
        intrinsics = remote_node.CameraIntrinsics(
            width=640,
            height=480,
            fx=440.0,
            fy=440.0,
            cx=320.0,
            cy=240.0,
            depth_scale=0.001,
        )

        u, v, depth = remote_node.project_base_target_at_tool_pose(
            tool_pose,
            [0.50, 0.0, 0.0],
            np.eye(4),
            intrinsics,
        )

        self.assertTrue(np.isnan(u))
        self.assertTrue(np.isnan(v))
        self.assertLess(depth, 0.0)

    def test_visibility_gate_checks_pregrasp_and_approach(self):
        node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
        node.tf_buffer = None
        node.handeye_translation_xyz = [0.0, 0.0, 0.0]
        node.handeye_rotation_xyzw = [0.0, 0.0, 0.0, 1.0]
        node._cached_tool_from_camera = None
        node.grasp_config = {
            'pregrasp_distance_m': 0.08,
            'final_approach_offset_m': 0.045,
            'lift_height_m': 0.05,
        }
        node.camera_visibility_require_approach = True
        node.camera_visibility_margin_px = 36
        node.camera_visibility_min_depth_m = 0.035
        node.camera_visibility_max_depth_m = 1.20
        node._camera_intrinsics = lambda: remote_node.CameraIntrinsics(
            width=640,
            height=480,
            fx=440.0,
            fy=440.0,
            cx=320.0,
            cy=240.0,
            depth_scale=0.001,
        )
        grasp_pose = PoseStamped()
        grasp_pose.pose.position.x = 0.50
        grasp_pose.pose.orientation.w = 1.0

        visible, metrics, reason = node._candidate_visibility_metrics(grasp_pose, [0.50, 0.0, 0.0])

        self.assertTrue(visible, reason)
        self.assertEqual([item['stage'] for item in metrics], ['pregrasp', 'approach'])
        self.assertAlmostEqual(metrics[0]['depth_m'], 0.08)
        self.assertAlmostEqual(metrics[1]['depth_m'], 0.045)

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

    def test_plan_reachable_rejects_orientation_fallback_when_execute_disallows_it(self):
        node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
        node.allow_position_only_fallback = False
        node.allow_orientation_fallback = False
        node._position_only_rejected_count = 0
        node._orientation_fallback_rejected_count = 0

        original_wait_for_service = remote_node.rospy.wait_for_service
        original_service_proxy = remote_node.rospy.ServiceProxy
        original_logwarn_throttle = remote_node.rospy.logwarn_throttle
        remote_node.rospy.wait_for_service = lambda *args, **kwargs: None
        remote_node.rospy.ServiceProxy = lambda *_args, **_kwargs: (
            lambda _pose, _execute: FakeServiceResponse(
                True,
                'planned with candidate orientation current: target xyz=(0.1, 0.2, 0.3)',
            )
        )
        remote_node.rospy.logwarn_throttle = lambda *args, **kwargs: None
        try:
            self.assertFalse(node._plan_reachable(PoseStamped()))
        finally:
            remote_node.rospy.wait_for_service = original_wait_for_service
            remote_node.rospy.ServiceProxy = original_service_proxy
            remote_node.rospy.logwarn_throttle = original_logwarn_throttle

        self.assertEqual(node._position_only_rejected_count, 0)
        self.assertEqual(node._orientation_fallback_rejected_count, 1)

    def test_plan_reachable_accepts_strict_pose_plan(self):
        node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
        node.allow_position_only_fallback = False
        node.allow_orientation_fallback = False
        node._position_only_rejected_count = 0
        node._orientation_fallback_rejected_count = 0

        original_wait_for_service = remote_node.rospy.wait_for_service
        original_service_proxy = remote_node.rospy.ServiceProxy
        remote_node.rospy.wait_for_service = lambda *args, **kwargs: None
        remote_node.rospy.ServiceProxy = lambda *_args, **_kwargs: (
            lambda _pose, _execute: FakeServiceResponse(
                True,
                'planned: target xyz=(0.1, 0.2, 0.3)',
            )
        )
        try:
            self.assertTrue(node._plan_reachable(PoseStamped()))
        finally:
            remote_node.rospy.wait_for_service = original_wait_for_service
            remote_node.rospy.ServiceProxy = original_service_proxy

        self.assertEqual(node._position_only_rejected_count, 0)
        self.assertEqual(node._orientation_fallback_rejected_count, 0)

    def test_masks_depth_to_expanded_target_roi(self):
        depth = np.arange(25, dtype=np.uint16).reshape(5, 5)

        roi = remote_node.expanded_bbox_roi(depth.shape, bbox_x=2, bbox_y=2, bbox_width=1, bbox_height=1, margin_px=1)
        masked = remote_node.mask_depth_to_roi(depth, roi)

        self.assertEqual(roi, (1, 1, 4, 4))
        self.assertEqual(int(masked[0, 0]), 0)
        self.assertEqual(int(masked[2, 2]), int(depth[2, 2]))
        self.assertEqual(remote_node.valid_depth_count(masked), 9)

    def test_depth_for_remote_requires_detected_target_roi(self):
        node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
        node.use_perception_roi = True
        node.require_perception_roi = True
        node.roi_margin_px = 0
        node.roi_max_age_sec = 1.0
        node.roi_min_valid_depth_px = 1
        node._object_lock = DummyLock()
        node.latest_object = types.SimpleNamespace(detected=False)
        node.latest_object_time = None

        with self.assertRaisesRegex(RuntimeError, 'waiting for target ROI'):
            node._depth_for_remote(np.ones((3, 3), dtype=np.uint16))

    def test_depth_for_remote_segments_foreground_cloud_from_bbox_depth(self):
        node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
        node.use_perception_roi = True
        node.require_perception_roi = True
        node.roi_margin_px = 0
        node.roi_max_age_sec = 1.0
        node.roi_min_valid_depth_px = 4
        node.target_cloud_enabled = True
        node.target_cloud_roi_margin_px = 0
        node.target_cloud_foreground_percentile = 35.0
        node.target_cloud_depth_window_m = 0.02
        node.target_cloud_min_points = 4
        node.target_cloud_max_points_for_gate = 100
        node.target_projection_frame_convention = 'ros_camera_link'
        node.pose_estimator = IdentityPointPoseEstimator()
        node._object_lock = DummyLock()
        node.latest_object = types.SimpleNamespace(
            detected=True,
            bbox_x=2,
            bbox_y=2,
            bbox_width=4,
            bbox_height=2,
        )
        depth = np.zeros((6, 6), dtype=np.uint16)
        depth[2:4, 2:4] = 1000
        depth[2:4, 4:6] = 2000

        original_get_param = remote_node.rospy.get_param
        original_time_now = remote_node.rospy.Time.now
        remote_node.rospy.Time.now = staticmethod(lambda: remote_node.rospy.Time.from_sec(10.0))
        node.latest_object_time = remote_node.rospy.Time.from_sec(10.0)
        remote_node.rospy.get_param = lambda name, default=None: {
            '/camera': {
                'width': 6,
                'height': 6,
                'fx': 100.0,
                'fy': 100.0,
                'cx': 2.0,
                'cy': 2.0,
                'depth_scale': 0.001,
            },
            '/perception': {
                'depth_min_m': 0.03,
                'depth_max_m': 3.0,
            },
        }.get(name, default)
        try:
            masked, message = node._depth_for_remote(depth, frame_id='camera_link')
        finally:
            remote_node.rospy.get_param = original_get_param
            remote_node.rospy.Time.now = original_time_now

        self.assertEqual(remote_node.valid_depth_count(masked), 4)
        self.assertEqual(int(masked[2, 2]), 1000)
        self.assertEqual(int(masked[2, 4]), 0)
        self.assertIn('target_cloud=4', message)
        np.testing.assert_allclose(
            node.latest_target_cloud_base_xyz,
            np.array([1.0, -0.005, -0.005]),
            atol=1e-6,
        )

    def test_candidate_gate_prefers_depth_cloud_target_over_stale_object_center(self):
        node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
        node.candidate_target_gate_enabled = True
        node.candidate_max_target_distance_m = 0.04
        node.candidate_min_relative_z_m = -0.015
        node.candidate_max_relative_z_m = 0.08
        node.target_cloud_enabled = True
        node.target_cloud_max_age_sec = 10.0
        node.target_cloud_candidate_max_point_distance_m = 0.055
        node.latest_target_cloud_time = remote_node.rospy.Time.from_sec(10.0)
        node.latest_target_cloud_source = 'roi_depth_foreground'
        node.latest_target_cloud_base_xyz = np.array([1.0, 0.0, 0.10])
        node.latest_target_cloud_camera_points = np.array([[0.3, 0.0, 0.0]], dtype=np.float32)
        node._target_gate_rejected_count = 0
        node._object_lock = DummyLock()

        stale_object_pose = PoseStamped()
        stale_object_pose.pose.position.x = 0.0
        stale_object_pose.pose.position.y = 0.0
        stale_object_pose.pose.position.z = 0.10
        node.latest_object = types.SimpleNamespace(detected=True, pose_base=stale_object_pose)
        node.latest_object_time = object()

        near_cloud = PoseStamped()
        near_cloud.pose.position.x = 1.01
        near_cloud.pose.position.y = 0.0
        near_cloud.pose.position.z = 0.105
        camera_candidate = types.SimpleNamespace(score=0.9, translation_m=np.array([0.3, 0.0, 0.0]))
        original_logwarn_throttle = remote_node.rospy.logwarn_throttle
        original_time_now = remote_node.rospy.Time.now
        remote_node.rospy.logwarn_throttle = lambda *args, **kwargs: None
        remote_node.rospy.Time.now = staticmethod(lambda: remote_node.rospy.Time.from_sec(10.0))
        try:
            self.assertTrue(node._candidate_matches_target(None, camera_candidate, near_cloud))
        finally:
            remote_node.rospy.logwarn_throttle = original_logwarn_throttle
            remote_node.rospy.Time.now = original_time_now

    def test_candidate_target_gate_rejects_far_or_low_grasp_pose(self):
        node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
        node.candidate_target_gate_enabled = True
        node.candidate_max_target_distance_m = 0.04
        node.candidate_min_relative_z_m = -0.015
        node.candidate_max_relative_z_m = 0.08
        node._target_gate_rejected_count = 0
        node._object_lock = DummyLock()

        target_pose = PoseStamped()
        target_pose.pose.position.x = -0.129
        target_pose.pose.position.y = -0.506
        target_pose.pose.position.z = 0.110
        node.latest_object = types.SimpleNamespace(detected=True, pose_base=target_pose)
        node.latest_object_time = object()

        far_and_low = PoseStamped()
        far_and_low.pose.position.x = -0.107
        far_and_low.pose.position.y = -0.480
        far_and_low.pose.position.z = 0.081
        original_logwarn_throttle = remote_node.rospy.logwarn_throttle
        remote_node.rospy.logwarn_throttle = lambda *args, **kwargs: None
        try:
            self.assertFalse(node._candidate_matches_target(None, None, far_and_low))
            self.assertEqual(node._target_gate_rejected_count, 1)

            close_enough = PoseStamped()
            close_enough.pose.position.x = -0.124
            close_enough.pose.position.y = -0.503
            close_enough.pose.position.z = 0.105
            self.assertTrue(node._candidate_matches_target(None, None, close_enough))
            self.assertEqual(node._target_gate_rejected_count, 1)
        finally:
            remote_node.rospy.logwarn_throttle = original_logwarn_throttle

    def test_object_callback_ignores_low_confidence_and_jump_outliers(self):
        node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
        node._object_lock = DummyLock()
        node.latest_object = None
        node.latest_object_time = None

        def obj(x, confidence):
            pose = PoseStamped()
            pose.pose.position.x = float(x)
            pose.pose.orientation.w = 1.0
            return types.SimpleNamespace(detected=True, confidence=float(confidence), pose_base=pose)

        original_get_param = remote_node.rospy.get_param
        original_time_now = remote_node.rospy.Time.now
        original_logwarn_throttle = remote_node.rospy.logwarn_throttle
        remote_node.rospy.get_param = lambda name, default=None: {
            '/grasp': {
                'min_object_confidence': 0.50,
                'max_object_jump_m': 0.12,
                'object_jump_filter_window_sec': 4.0,
            },
        }.get(name, default)
        remote_node.rospy.Time.now = staticmethod(lambda: remote_node.rospy.Time.from_sec(10.0))
        remote_node.rospy.logwarn_throttle = lambda *args, **kwargs: None
        try:
            node.object_cb(obj(0.10, 0.90))
            self.assertAlmostEqual(node.latest_object.pose_base.pose.position.x, 0.10)

            node.object_cb(obj(0.11, 0.40))
            self.assertAlmostEqual(node.latest_object.pose_base.pose.position.x, 0.10)

            node.object_cb(obj(1.70, 0.90))
            self.assertAlmostEqual(node.latest_object.pose_base.pose.position.x, 0.10)

            node.object_cb(obj(0.11, 0.90))
            self.assertAlmostEqual(node.latest_object.pose_base.pose.position.x, 0.11)
        finally:
            remote_node.rospy.get_param = original_get_param
            remote_node.rospy.Time.now = original_time_now
            remote_node.rospy.logwarn_throttle = original_logwarn_throttle

    def test_remote_pose_estimator_uses_tf_buffer_when_enabled(self):
        estimator, tf_buffer, tf_listener = remote_node.make_remote_pose_estimator(
            cam_cfg={'frame_id': 'camera_link'},
            hcfg={
                'use_tf': True,
                'camera_frame': 'camera_link',
                'base_frame': 'base_link',
                'translation_xyz': [1.0, 2.0, 3.0],
                'rotation_xyzw': [0.0, 0.0, 0.0, 1.0],
                'allow_static_fallback': False,
            },
            gcfg={'default_orientation_xyzw': [0.0, 0.0, 0.0, 1.0]},
            tf2_module=FakeTf2Module,
        )

        self.assertIsNotNone(tf_buffer)
        self.assertIs(tf_listener.buffer, tf_buffer)
        self.assertIs(estimator.tf_buffer, tf_buffer)
        self.assertFalse(estimator.allow_static_fallback)


if __name__ == '__main__':
    unittest.main()
