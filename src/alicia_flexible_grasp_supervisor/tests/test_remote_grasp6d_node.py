#!/usr/bin/env python3
import importlib.util
import pathlib
import sys
import threading
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
    def test_gate_audit_exposes_independent_filter_intersection(self):
        rows = [
            {
                'depth_ok': True,
                'width_ok': True,
                'target_ok': True,
                'finger_clearance_m': 0.006,
                'center_clearance_m': 0.020,
                'jaw_normal_cos': 0.20,
                'approach_cos': 0.60,
                'cloud_distance_m': 0.001,
                'visibility_ok': True,
            },
            {
                'depth_ok': True,
                'width_ok': True,
                'target_ok': True,
                'finger_clearance_m': -0.004,
                'center_clearance_m': 0.010,
                'jaw_normal_cos': 0.70,
                'approach_cos': 0.70,
                'cloud_distance_m': 0.002,
                'visibility_ok': True,
            },
            {
                'depth_ok': True,
                'width_ok': True,
                'target_ok': True,
                'finger_clearance_m': 0.008,
                'center_clearance_m': 0.018,
                'jaw_normal_cos': 0.10,
                'approach_cos': 0.10,
                'cloud_distance_m': 0.003,
                'visibility_ok': False,
            },
        ]

        summary = remote_node.summarize_candidate_gate_audit(
            rows,
            clearance_thresholds_m=[0.003, -0.010],
            approach_thresholds=[0.45, -0.20],
        )

        self.assertEqual(summary['target_pass'], 3)
        profiles = {
            (item['clearance_m'], item['approach_cos']): item
            for item in summary['profiles']
        }
        self.assertEqual(profiles[(0.003, 0.45)]['pass_count'], 1)
        self.assertEqual(profiles[(-0.010, 0.45)]['pass_count'], 2)
        self.assertEqual(profiles[(0.003, -0.20)]['pass_count'], 2)
        self.assertEqual(profiles[(0.003, 0.45)]['visible_count'], 1)

    def test_gate_audit_handles_missing_support_metrics(self):
        summary = remote_node.summarize_candidate_gate_audit(
            [
                {
                    'depth_ok': True,
                    'width_ok': True,
                    'target_ok': True,
                    'finger_clearance_m': None,
                    'approach_cos': 0.8,
                    'visibility_ok': True,
                }
            ],
            clearance_thresholds_m=[0.003],
            approach_thresholds=[0.45],
        )

        self.assertEqual(summary['target_pass'], 1)
        self.assertEqual(summary['profiles'][0]['pass_count'], 0)

    def test_gate_audit_separates_parallel_jaw_symmetry_variants(self):
        rows = [
            {
                'variant_index': 0,
                'depth_ok': True,
                'width_ok': True,
                'target_ok': True,
                'finger_clearance_m': 0.006,
                'approach_cos': 0.60,
                'visibility_ok': False,
            },
            {
                'variant_index': 1,
                'depth_ok': True,
                'width_ok': True,
                'target_ok': True,
                'finger_clearance_m': 0.006,
                'approach_cos': 0.60,
                'visibility_ok': True,
            },
        ]

        summary = remote_node.summarize_candidate_gate_audit(
            rows,
            clearance_thresholds_m=[0.003],
            approach_thresholds=[0.45],
            baseline_clearance_m=0.003,
            baseline_approach_cos=0.45,
        )

        variants = {
            item['variant_index']: item
            for item in summary['variant_profiles']
        }
        self.assertEqual(variants[0]['safe_count'], 1)
        self.assertEqual(variants[0]['visible_count'], 0)
        self.assertEqual(variants[1]['safe_count'], 1)
        self.assertEqual(variants[1]['visible_count'], 1)

    def test_support_plane_segmentation_removes_table_from_mouse_roi(self):
        height, width = 60, 80
        depth_m = np.full((height, width), 0.50, dtype=np.float32)
        depth_m[15:50, 20:62] = 0.46
        valid = np.ones_like(depth_m, dtype=bool)
        intrinsics = remote_node.CameraIntrinsics(
            width=width,
            height=height,
            fx=120.0,
            fy=120.0,
            cx=40.0,
            cy=30.0,
            depth_scale=0.001,
        )

        mask, plane, diagnostic = remote_node.segment_foreground_above_support_plane(
            depth_m,
            valid,
            (0, 0, width, height),
            intrinsics,
            min_points=80,
            iterations=64,
            far_percentile=55.0,
            inlier_distance_m=0.002,
            min_height_m=0.004,
            min_inlier_ratio=0.08,
        )

        self.assertIsNotNone(mask)
        self.assertIsNotNone(plane)
        self.assertGreater(int(np.count_nonzero(mask[15:50, 20:62])), 1300)
        self.assertEqual(int(np.count_nonzero(mask[:10, :])), 0)
        self.assertIn('support_plane=inliers:', diagnostic)
        self.assertGreater(float(plane['plane_depth_m'] - plane['foreground_depth_m']), 0.03)

    def test_support_plane_uses_context_outside_tight_mouse_bbox(self):
        height, width = 100, 120
        depth_m = np.full((height, width), 0.50, dtype=np.float32)
        target_roi = (40, 30, 80, 70)
        depth_m[30:70, 40:80] = 0.46
        intrinsics = remote_node.CameraIntrinsics(
            width=width,
            height=height,
            fx=180.0,
            fy=180.0,
            cx=60.0,
            cy=50.0,
            depth_scale=0.001,
        )
        node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
        node.target_cloud_min_points = 80
        node.target_cloud_support_plane_enabled = True
        node.target_cloud_support_plane_context_margin_px = 20
        node.target_cloud_support_plane_ransac_iterations = 64
        node.target_cloud_support_plane_far_percentile = 55.0
        node.target_cloud_support_plane_inlier_distance_m = 0.002
        node.target_cloud_support_plane_min_height_m = 0.004
        node.target_cloud_support_plane_min_inlier_ratio = 0.08
        node.target_projection_frame_convention = 'ros_camera_link'
        node._camera_intrinsics = lambda: intrinsics

        original_get_param = remote_node.rospy.get_param
        remote_node.rospy.get_param = lambda name, default=None: {
            '/perception': {'depth_min_m': 0.03, 'depth_max_m': 2.0},
        }.get(name, default)
        try:
            mask, count = node._foreground_mask_for_roi(
                depth_m,
                target_roi,
                min_points=80,
            )
        finally:
            remote_node.rospy.get_param = original_get_param

        self.assertEqual(mask.shape, (40, 40))
        self.assertGreater(count, 1500)
        self.assertIsNotNone(node.latest_support_plane_camera_point)
        self.assertIsNotNone(node.latest_support_plane_camera_normal)
        self.assertIn('context-margin:20px', node.latest_target_cloud_segmentation)

    def test_resolves_protocol_fields_from_unified_wsl_health_payload(self):
        resolved = remote_node.resolve_grasp_backend_health(
            {
                'ok': True,
                'backend': 'graspnet_baseline',
                'grasp_backend': {
                    'backend': 'graspnet_baseline',
                    'protocol_version': 2,
                    'candidate_fields': ['score', 'depth_m'],
                },
            }
        )

        self.assertEqual(resolved['protocol_version'], 2)
        self.assertIn('depth_m', resolved['candidate_fields'])

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

    def test_nonfinite_ranked_candidate_is_not_selected(self):
        candidate = RemoteGraspCandidate(
            0.9,
            np.array([0.01, 0.0, 0.0]),
            np.array([0.0, 0.0, 0.0, 1.0]),
            0.04,
        )

        selected, pose = remote_node.select_first_reachable_candidate(
            [candidate],
            FakePoseEstimator(),
            lambda _pose: True,
            stamp=None,
            camera_frame='camera_link',
            candidate_frame_convention='ros_camera_link',
            candidate_rank_fn=lambda *_args: float('inf'),
        )

        self.assertIsNone(selected)
        self.assertIsNone(pose)

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

    def test_parallel_jaw_symmetry_preserves_approach_and_swaps_jaw_axis(self):
        candidate = RemoteGraspCandidate(
            score=0.9,
            translation_m=np.array([0.30, 0.0, 0.0]),
            quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
            width_m=0.05,
            depth_m=0.03,
        )
        estimator = RecordingPoseEstimator()
        model_to_tool = remote_node.quaternion_from_euler(0.0, np.pi * 0.5, 0.0)
        tool_z_half_turn = remote_node.quaternion_from_euler(0.0, 0.0, np.pi)

        selected, pose = remote_node.select_first_reachable_candidate(
            [candidate],
            estimator,
            lambda _pose: False,
            stamp=None,
            camera_frame='camera_link',
            candidate_frame_convention='ros_camera_link',
            orientation_variant_quaternions=[
                np.array([0.0, 0.0, 0.0, 1.0]),
                tool_z_half_turn,
            ],
            model_grasp_to_tool_quaternion=model_to_tool,
        )

        self.assertIsNone(selected)
        self.assertIsNone(pose)
        self.assertEqual(len(estimator.calls), 2)
        first = remote_node.quaternion_matrix(estimator.calls[0][1])[:3, :3]
        symmetric = remote_node.quaternion_matrix(estimator.calls[1][1])[:3, :3]
        np.testing.assert_allclose(symmetric[:, 2], first[:, 2], atol=1e-7)
        np.testing.assert_allclose(symmetric[:, 1], -first[:, 1], atol=1e-7)
        np.testing.assert_allclose(symmetric[:, 0], -first[:, 0], atol=1e-7)

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

    def test_visibility_margin_scales_with_target_bbox_and_depth(self):
        node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
        node.camera_visibility_margin_px = 36
        node.camera_visibility_bbox_padding_px = 8
        node._object_lock = threading.Lock()
        node.latest_object_time = None
        node.latest_object = types.SimpleNamespace(
            detected=True,
            depth_m=0.32,
            bbox_width=84,
            bbox_height=112,
        )

        margin_x, margin_y = node._camera_visibility_margins_px(0.16)

        self.assertEqual(margin_x, 92)
        self.assertEqual(margin_y, 120)

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

    def test_support_plane_cloud_allows_valid_elongated_object_grasp_outside_center_sphere(self):
        node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
        node.grasp_config = {'tool_approach_axis': 'z'}
        node.candidate_target_gate_enabled = True
        node.candidate_max_target_distance_m = 0.04
        node.candidate_min_relative_z_m = -0.015
        node.candidate_max_relative_z_m = 0.08
        node.target_cloud_enabled = True
        node.target_cloud_max_age_sec = 1.0
        node.target_cloud_candidate_max_point_distance_m = 0.030
        node.target_cloud_candidate_min_support_clearance_m = -0.002
        node.latest_target_cloud_base_xyz = np.array([0.0, 0.0, 0.10])
        node.latest_target_cloud_camera_points = np.array([[0.30, 0.06, 0.0]], dtype=np.float32)
        node.latest_target_cloud_time = remote_node.rospy.Time.from_sec(1.0)
        node.latest_target_cloud_source = 'roi_depth_foreground'
        node.latest_target_cloud_segmentation = 'support_plane=inliers:500 foreground:300 gap:0.030m'
        node.latest_support_plane_camera_point = None
        node.latest_support_plane_camera_normal = None
        node._target_cloud_request_active = True
        node.camera_visibility_gate_enabled = False
        node.require_candidate_depth = True
        node.candidate_min_downward_approach_cos = 0.55
        node._target_gate_rejected_count = 0
        node._approach_gate_rejected_count = 0
        node._candidate_approach_downward_cos = lambda _candidate, _pose: 0.8

        grasp_pose = PoseStamped()
        grasp_pose.pose.position.x = 0.0
        grasp_pose.pose.position.y = 0.06
        grasp_pose.pose.position.z = 0.10
        camera_candidate = types.SimpleNamespace(
            score=0.8,
            width_m=0.05,
            depth_m=0.02,
            translation_m=np.array([0.30, 0.06, 0.0]),
        )

        original_logwarn_throttle = remote_node.rospy.logwarn_throttle
        remote_node.rospy.logwarn_throttle = lambda *args, **kwargs: None
        try:
            accepted = node._candidate_matches_target(None, camera_candidate, grasp_pose)
        finally:
            remote_node.rospy.logwarn_throttle = original_logwarn_throttle

        self.assertTrue(accepted)
        self.assertGreater(0.06, node.candidate_max_target_distance_m)
        self.assertEqual(node._target_gate_rejected_count, 0)

    def test_candidate_approach_uses_observed_support_plane_normal(self):
        node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
        node.grasp_config = {'tool_approach_axis': 'z'}
        node.latest_support_plane_camera_normal = np.array([-1.0, 0.0, 0.0])
        candidate = types.SimpleNamespace(
            quaternion_xyzw=remote_node.quaternion_from_euler(0.0, np.pi * 0.5, 0.0)
        )
        pose = PoseStamped()
        pose.pose.orientation.w = 1.0

        downward = node._candidate_approach_downward_cos(candidate, pose)

        self.assertAlmostEqual(downward, 1.0, places=6)

    def test_support_geometry_rejects_vertical_jaw_that_straddles_table(self):
        node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
        node.latest_support_plane_camera_point = np.array([0.0, 0.0, 0.0])
        node.latest_support_plane_camera_normal = np.array([0.0, 0.0, 1.0])
        candidate = types.SimpleNamespace(
            translation_m=np.array([0.0, 0.0, 0.025]),
            quaternion_xyzw=remote_node.quaternion_from_euler(np.pi * 0.5, 0.0, 0.0),
            width_m=0.070,
        )

        metrics = node._candidate_support_geometry_metrics(candidate)

        self.assertAlmostEqual(metrics['jaw_normal_cos'], 1.0, places=6)
        self.assertAlmostEqual(metrics['min_finger_clearance_m'], -0.010, places=6)

    def test_support_geometry_accepts_horizontal_jaw_above_table(self):
        node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
        node.latest_support_plane_camera_point = np.array([0.0, 0.0, 0.0])
        node.latest_support_plane_camera_normal = np.array([0.0, 0.0, 1.0])
        candidate = types.SimpleNamespace(
            translation_m=np.array([0.0, 0.0, 0.025]),
            quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
            width_m=0.090,
        )

        metrics = node._candidate_support_geometry_metrics(candidate)

        self.assertAlmostEqual(metrics['jaw_normal_cos'], 0.0, places=6)
        self.assertAlmostEqual(metrics['min_finger_clearance_m'], 0.025, places=6)

    def test_support_geometry_clamps_model_width_to_physical_gripper_stroke(self):
        node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
        node.latest_support_plane_camera_point = np.array([0.0, 0.0, 0.0])
        node.latest_support_plane_camera_normal = np.array([0.0, 0.0, 1.0])
        node.gripper_physical_open_width_m = 0.050
        candidate = types.SimpleNamespace(
            translation_m=np.array([0.0, 0.0, 0.011]),
            quaternion_xyzw=remote_node.quaternion_from_euler(np.arcsin(0.233), 0.0, 0.0),
            width_m=0.083,
        )

        metrics = node._candidate_support_geometry_metrics(candidate)

        self.assertAlmostEqual(metrics['model_width_m'], 0.083, places=6)
        self.assertAlmostEqual(metrics['geometry_width_m'], 0.050, places=6)
        self.assertGreater(metrics['min_finger_clearance_m'], 0.005)

    def test_support_geometry_allows_inclined_jaw_when_both_fingers_clear_table(self):
        node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
        node.candidate_max_jaw_normal_cos = 0.35
        node.candidate_min_finger_support_clearance_m = 0.003
        metrics = {
            'jaw_normal_cos': 0.55,
            'min_finger_clearance_m': 0.006,
        }

        self.assertFalse(node._candidate_support_geometry_collides(metrics))

    def test_support_geometry_rejects_vertical_jaw_when_finger_crosses_table(self):
        node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
        node.candidate_max_jaw_normal_cos = 0.35
        node.candidate_min_finger_support_clearance_m = 0.003
        metrics = {
            'jaw_normal_cos': 1.0,
            'min_finger_clearance_m': -0.010,
        }

        self.assertTrue(node._candidate_support_geometry_collides(metrics))

    def test_active_request_keeps_its_cloud_after_wall_clock_age_limit(self):
        node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
        node.target_cloud_enabled = True
        node.target_cloud_max_age_sec = 1.0
        node.latest_target_cloud_time = remote_node.rospy.Time.from_sec(1.0)
        node.latest_target_cloud_source = 'roi_depth_foreground'
        node.latest_target_cloud_base_xyz = np.array([1.0, 2.0, 3.0])
        node._target_cloud_request_active = True
        node._object_lock = DummyLock()
        stale_pose = PoseStamped()
        stale_pose.pose.position.x = 9.0
        node.latest_object = types.SimpleNamespace(detected=True, pose_base=stale_pose)
        node.latest_object_time = object()

        original_time_now = remote_node.rospy.Time.now
        remote_node.rospy.Time.now = staticmethod(lambda: remote_node.rospy.Time.from_sec(20.0))
        try:
            target, source = node._target_base_xyz()
        finally:
            remote_node.rospy.Time.now = original_time_now

        np.testing.assert_allclose(target, [1.0, 2.0, 3.0])
        self.assertEqual(source, 'roi_depth_foreground')

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

    def test_tabletop_gate_rejects_failed_mouse_shallow_approach(self):
        node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
        node.grasp_config = {'tool_approach_axis': 'z'}
        node.candidate_target_gate_enabled = True
        node.candidate_max_target_distance_m = 0.04
        node.candidate_min_relative_z_m = -0.015
        node.candidate_max_relative_z_m = 0.08
        node.candidate_min_downward_approach_cos = 0.45
        node.target_cloud_enabled = False
        node.target_cloud_candidate_max_point_distance_m = 0.0
        node.camera_visibility_gate_enabled = False
        node.require_candidate_depth = False
        node._target_gate_rejected_count = 0
        node._approach_gate_rejected_count = 0
        node._object_lock = DummyLock()

        target = PoseStamped()
        target.pose.position.x = -0.172
        target.pose.position.y = -0.414
        target.pose.position.z = 0.085
        node.latest_object = types.SimpleNamespace(detected=True, pose_base=target)
        node.latest_object_time = object()

        failed_grasp = PoseStamped()
        failed_grasp.pose.position.x = -0.148
        failed_grasp.pose.position.y = -0.408
        failed_grasp.pose.position.z = 0.070
        failed_grasp.pose.orientation.x = 0.662
        failed_grasp.pose.orientation.y = 0.489
        failed_grasp.pose.orientation.z = -0.548
        failed_grasp.pose.orientation.w = -0.149
        camera_candidate = types.SimpleNamespace(score=0.517, width_m=0.078, depth_m=0.030)

        original_logwarn_throttle = remote_node.rospy.logwarn_throttle
        remote_node.rospy.logwarn_throttle = lambda *args, **kwargs: None
        try:
            downward = node._tool_approach_downward_cos(failed_grasp)
            accepted = node._candidate_matches_target(None, camera_candidate, failed_grasp)
        finally:
            remote_node.rospy.logwarn_throttle = original_logwarn_throttle

        self.assertAlmostEqual(downward, 0.355, places=3)
        self.assertFalse(accepted)
        self.assertEqual(node._approach_gate_rejected_count, 1)

    def test_candidate_rank_includes_model_motion_and_approach_quality(self):
        node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
        pose = PoseStamped()
        pose.pose.orientation.w = 1.0
        node._candidate_target_distance = lambda *_args: 0.020
        node._tool_approach_downward_cos = lambda _pose: 0.75
        node.candidate_model_score_weight_m = 0.010
        node.candidate_joint_path_cost_weight_m = 0.004
        node.candidate_downward_approach_weight_m = 0.020
        node.camera_visibility_rank_weight_m = 0.0
        node.camera_visibility_gate_enabled = False
        node._candidate_plan_metrics = {
            node._pose_key(pose): {'joint_path_cost': 2.0, 'joint_max_delta': 1.0},
        }
        candidate = types.SimpleNamespace(score=0.50)

        rank = node._candidate_rank(candidate, candidate, pose)

        self.assertAlmostEqual(rank, 0.020 - 0.005 + 0.008 + 0.005)

    def test_candidate_rank_rejects_large_single_joint_flip(self):
        node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
        pose = PoseStamped()
        pose.pose.orientation.w = 1.0
        node._candidate_target_distance = lambda *_args: 0.010
        node.candidate_model_score_weight_m = 0.0
        node.candidate_joint_path_cost_weight_m = 0.0
        node.candidate_downward_approach_weight_m = 0.0
        node.candidate_max_joint_delta_rad = 1.8
        node._joint_motion_gate_rejected_count = 0
        node._candidate_plan_metrics = {
            node._pose_key(pose): {'joint_path_cost': 3.194, 'joint_max_delta': 2.588},
        }
        candidate = types.SimpleNamespace(score=0.50)

        original_logwarn_throttle = remote_node.rospy.logwarn_throttle
        remote_node.rospy.logwarn_throttle = lambda *args, **kwargs: None
        try:
            rank = node._candidate_rank(candidate, candidate, pose)
        finally:
            remote_node.rospy.logwarn_throttle = original_logwarn_throttle

        self.assertTrue(np.isinf(rank))
        self.assertEqual(node._joint_motion_gate_rejected_count, 1)

    def test_plan_metrics_are_parsed_from_strict_reachability_message(self):
        metrics = remote_node.RemoteGrasp6DNode._parse_plan_metrics(
            'planned target joint_path_cost=2.345 joint_max_delta=0.678'
        )

        self.assertEqual(metrics, {'joint_path_cost': 2.345, 'joint_max_delta': 0.678})

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
