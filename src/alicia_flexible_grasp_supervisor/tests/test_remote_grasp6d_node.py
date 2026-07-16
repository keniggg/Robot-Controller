#!/usr/bin/env python3
import importlib.util
import pathlib
import sys
import threading
import types
import unittest
import urllib.error

import numpy as np
from geometry_msgs.msg import PoseStamped


ROOT = pathlib.Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / 'src'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from alicia_flexible_grasp.vision.remote_grasp6d_client import RemoteGraspCandidate
from alicia_flexible_grasp.vision.rgbd_snapshot import (
    DepthQuality,
    RgbdSample,
    SnapshotResult,
)


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
        self.camera_frame = 'camera_link'

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


class NonBlockingLock:
    def __init__(self):
        self.locked = False

    def acquire(self, blocking=True):
        del blocking
        if self.locked:
            return False
        self.locked = True
        return True

    def release(self):
        self.locked = False


class RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(getattr(message, 'data', message))


class RecordingClient:
    def __init__(self, candidates=None, error=None):
        self.candidates = list(candidates or [])
        self.error = error
        self.calls = []
        self.last_diagnostics = {}

    def predict(self, color, depth, intrinsics, **kwargs):
        self.calls.append(
            {
                'color': np.asarray(color).copy(),
                'depth': np.asarray(depth).copy(),
                'intrinsics': intrinsics,
                'kwargs': dict(kwargs),
            }
        )
        if self.error is not None:
            raise self.error
        return list(self.candidates)


class SequenceSampleBuffer:
    def __init__(self, windows):
        self.windows = list(windows)
        self.discarded = []

    def wait_for_samples(self, count, timeout_sec, require_mask, max_age_sec):
        del count, timeout_sec, require_mask, max_age_sec
        return self.windows.pop(0) if self.windows else []

    def discard_through(self, stamp_sec):
        raise AssertionError('retry discard must use exact integer nanoseconds, got %r' % stamp_sec)

    def discard_through_ns(self, stamp_ns):
        self.discarded.append(int(stamp_ns))


class FakeTf2Module:
    class Buffer:
        pass

    class TransformListener:
        def __init__(self, buffer):
            self.buffer = buffer


def make_snapshot(target_depth, source_mode='instance_mask', stamp_ns=10_000_000_123):
    target_depth = np.asarray(target_depth, dtype=np.uint16)
    full_depth = np.full(target_depth.shape, 2400, dtype=np.uint16)
    full_depth[target_depth > 0] = target_depth[target_depth > 0]
    mask = np.where(target_depth > 0, 255, 0).astype(np.uint8)
    ys, xs = np.nonzero(mask)
    if xs.size:
        bbox = (
            int(np.min(xs)),
            int(np.min(ys)),
            int(np.max(xs) - np.min(xs) + 1),
            int(np.max(ys) - np.min(ys) + 1),
        )
    else:
        bbox = (0, 0, target_depth.shape[1], target_depth.shape[0])
    return SnapshotResult(
        ok=True,
        failure_code='',
        failure_reason='',
        color_bgr=np.zeros(target_depth.shape + (3,), dtype=np.uint8),
        depth_raw=full_depth,
        target_depth_raw=target_depth,
        object_mask=mask,
        bbox=bbox,
        object_msg=types.SimpleNamespace(detected=True, label='carton'),
        stamp_sec=float(stamp_ns) * 1e-9,
        frame_id='camera_link',
        quality=DepthQuality(
            fused_frames=3,
            mask_area=int(np.count_nonzero(mask)),
            valid_depth_points=int(np.count_nonzero(target_depth)),
            valid_depth_ratio=1.0 if np.any(target_depth) else 0.0,
            depth_median_m=0.22,
            depth_mad_m=0.0015,
        ),
        source_mode=source_mode,
        stamp_ns=int(stamp_ns),
    )


def make_geometry_estimate(
    ok=True,
    code='',
    reason='',
    source_mode='instance_mask',
    center_base=None,
    axes_base=None,
    size_xyz_m=None,
):
    return remote_node.GeometryEstimate(
        ok=bool(ok),
        failure_code=str(code),
        failure_reason=str(reason),
        center_base=np.asarray(
            [0.40, -0.10, 0.22] if center_base is None else center_base
        ),
        axes_base=np.asarray(np.eye(3) if axes_base is None else axes_base),
        size_xyz_m=np.asarray(
            [0.24, 0.16, 0.10] if size_xyz_m is None else size_xyz_m
        ),
        support_normal_base=np.asarray([0.0, 0.0, 1.0]),
        support_offset_m=0.0,
        support_inlier_ratio=0.82,
        object_points_base=np.asarray(
            [[0.39, -0.10, 0.20], [0.41, -0.10, 0.21], [0.40, -0.09, 0.22]]
        ),
        source_mode=source_mode,
    )


def make_processing_node(client=None):
    node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
    node._request_lock = NonBlockingLock()
    node._geometry_state_lock = threading.RLock()
    node._geometry_invalidation_generation = 0
    node._last_geometry_invalidation_code = ''
    node._refresh_runtime_params = lambda: None
    node.plan_pub = RecordingPublisher()
    node.geometry_pub = RecordingPublisher()
    node.status_pub = RecordingPublisher()
    node.client = client or RecordingClient()
    node.pose_estimator = types.SimpleNamespace(
        camera_frame='camera_link',
        base_frame='base_link',
        tf_buffer=None,
        allow_static_fallback=True,
        translation_xyz=[0.0, 0.0, 0.0],
        rotation_xyzw=[0.0, 0.0, 0.0, 1.0],
    )
    node.tf_buffer = None
    node.failure_backoff_sec = 0.0
    node.last_error = ''
    node._backoff_until = remote_node.rospy.Time(0)
    node._cached_tool_from_camera = None
    node.previous_object_axes_base = np.eye(3)
    node.latest_object_geometry = None
    node.target_cloud_enabled = True
    node.target_projection_frame_convention = 'ros_camera_link'
    node.geometry_support_bbox_expand_ratio = 0.30
    node.geometry_support_distance_threshold_m = 0.004
    node.geometry_voxel_size_m = 0.0025
    node.geometry_min_support_points = 3
    node.geometry_min_object_points = 1
    node.geometry_min_size_m = 0.005
    node.geometry_max_size_m = 0.600
    node.geometry_max_height_m = 0.500
    node.geometry_outlier_neighbors = 2
    node.geometry_outlier_std_ratio = 2.0
    node.max_candidates = 20
    node.max_gripper_width_m = 0.05
    node.candidate_width_tolerance_m = 0.0
    node.gripper_physical_open_width_m = 0.05
    node.gripper_geometry = remote_node.GripperGeometry(
        max_inner_gap_m=0.050,
        jaw_clearance_each_side_m=0.002,
        finger_size_xyz_m=np.array([0.0434, 0.0286, 0.0600]),
        palm_size_xyz_m=np.array([0.1175, 0.1550, 0.0774]),
        support_clearance_m=0.003,
    )
    node.gripper_tool_jaw_axis = 'y'
    node.gripper_tool_finger_length_axis = 'z'
    node.twin_gripper_model_name = 'Alicia_D_v5_6_gripper_50mm'
    node.twin_max_inner_gap_m = 0.05
    node._geometry_gate_counts = {}
    node._geometry_rejection_counts = {}
    node._selected_candidate_gate = None
    node.selected_required_open_width_m = None
    node.grasp_config = {
        'pregrasp_distance_m': 0.08,
        'final_approach_offset_m': 0.020,
        'lift_height_m': 0.05,
        'tool_approach_axis': 'z',
    }
    node.rank_by_target_distance = False
    node.candidate_target_gate_enabled = False
    node.camera_visibility_gate_enabled = False
    node.camera_visibility_diagnostic_enabled = False
    node.require_candidate_depth = False
    node.orientation_variant_quaternions = [np.array([0.0, 0.0, 0.0, 1.0])]
    node.model_grasp_to_tool_quaternion = np.array([0.0, 0.0, 0.0, 1.0])
    node.candidate_frame_convention = 'ros_camera_link'
    node.gate_audit_enabled = False
    node._camera_intrinsics = lambda: remote_node.CameraIntrinsics(
        width=4,
        height=3,
        fx=100.0,
        fy=100.0,
        cx=1.5,
        cy=1.0,
        depth_scale=0.0001,
    )
    return node


class RemoteGrasp6DNodeTest(unittest.TestCase):
    def test_ros_source_clock_preserves_exact_nanoseconds(self):
        class Stamp:
            def to_nsec(self):
                return 1_700_000_000_000_000_123

        original_now = remote_node.rospy.Time.now
        remote_node.rospy.Time.now = staticmethod(lambda: Stamp())
        try:
            self.assertEqual(
                remote_node.RemoteGrasp6DNode._ros_source_clock_ns(),
                1_700_000_000_000_000_123,
            )
        finally:
            remote_node.rospy.Time.now = original_now

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

    def test_remote_node_exports_synchronized_rgbd_buffer(self):
        self.assertEqual(remote_node.SynchronizedRgbdBuffer.__name__, 'SynchronizedRgbdBuffer')

    def test_one_request_retries_unstable_window_until_stable_frames_arrive(self):
        mask = np.zeros((20, 30), dtype=np.uint8)
        mask[5:15, 8:22] = 255

        def rgbd(stamp, joints):
            item = RgbdSample(
                color_bgr=np.zeros((20, 30, 3), dtype=np.uint8),
                depth_raw=np.full((20, 30), 2200, dtype=np.uint16),
                object_mask=mask.copy(),
                bbox=(5, 4, 20, 12),
                object_msg=types.SimpleNamespace(detected=True, snapshot_stamp=stamp),
                stamp_sec=float(stamp),
                frame_id='camera_link',
                joint_positions=np.asarray(joints, dtype=float),
            )
            item.stamp_ns = int(round(float(stamp) * 1e9))
            return item

        unstable = [
            rgbd(1.00, [0.0] * 6),
            rgbd(1.03, [0.02, 0, 0, 0, 0, 0]),
            rgbd(1.06, [0.0] * 6),
        ]
        stable = [
            rgbd(1.10, [0.0] * 6),
            rgbd(1.13, [0.0] * 6),
            rgbd(1.16, [0.0] * 6),
        ]
        node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
        node.enabled = True
        node.frames = SequenceSampleBuffer([unstable, stable])
        node.status_pub = RecordingPublisher()
        node.planning_snapshot_frames = 3
        node.planning_snapshot_timeout_sec = 1.0
        node.planning_snapshot_max_age_sec = 0.35
        node.planning_mask_min_iou = 0.85
        node.planning_mask_max_centroid_shift_px = 5.0
        node.planning_max_joint_delta_rad = 0.01
        node.mask_erosion_px = 2
        node.mask_internal_hole_max_area_px = 25
        node.depth_mad_scale = 3.5
        node.depth_mad_absolute_floor_m = 0.002
        node._active_profile_requires_mask = lambda: True
        node._snapshot_depth_config = lambda: (0.0001, 0.03, 2.0)
        processed = []
        node._process_frame = lambda snapshot, manual=False: (
            processed.append((snapshot, manual)) or (True, 'planned')
        )

        response = node.request_plan_cb(types.SimpleNamespace(trigger=True))

        self.assertTrue(response.success)
        self.assertEqual(response.message, 'planned')
        self.assertEqual(node.frames.discarded, [1_060_000_000])
        self.assertEqual(len(processed), 1)
        self.assertTrue(processed[0][0].ok)
        self.assertEqual(processed[0][0].object_msg.snapshot_stamp, 1.16)
        self.assertTrue(processed[0][1])

    def test_snapshot_failure_is_published_with_one_structured_prefix(self):
        good_mask = np.ones((20, 30), dtype=np.uint8) * 255
        wrong_mask = np.ones((10, 10), dtype=np.uint8) * 255

        def rgbd(mask, stamp):
            return RgbdSample(
                color_bgr=np.zeros((20, 30, 3), dtype=np.uint8),
                depth_raw=np.full((20, 30), 2200, dtype=np.uint16),
                object_mask=mask,
                bbox=(5, 4, 20, 12),
                object_msg=types.SimpleNamespace(detected=True),
                stamp_sec=float(stamp),
                frame_id='camera_link',
                joint_positions=np.zeros(6, dtype=float),
            )

        node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
        node.enabled = True
        node.frames = SequenceSampleBuffer(
            [[rgbd(good_mask, 1.00), rgbd(wrong_mask, 1.03), rgbd(good_mask, 1.06)]]
        )
        node.status_pub = RecordingPublisher()
        node.plan_pub = RecordingPublisher()
        node.geometry_pub = RecordingPublisher()
        node.previous_object_axes_base = np.eye(3)
        node.planning_snapshot_frames = 3
        node.planning_snapshot_timeout_sec = 1.0
        node.planning_snapshot_max_age_sec = 0.35
        node.planning_mask_min_iou = 0.85
        node.planning_mask_max_centroid_shift_px = 5.0
        node.planning_max_joint_delta_rad = 0.01
        node.mask_erosion_px = 2
        node.mask_internal_hole_max_area_px = 25
        node.depth_mad_scale = 3.5
        node.depth_mad_absolute_floor_m = 0.002
        node._active_profile_requires_mask = lambda: True
        node._snapshot_depth_config = lambda: (0.0001, 0.03, 2.0)

        response = node.request_plan_cb(types.SimpleNamespace(trigger=True))

        self.assertFalse(response.success)
        self.assertTrue(response.message.startswith('MASK_SIZE_MISMATCH: '))
        self.assertEqual(response.message.count('MASK_SIZE_MISMATCH:'), 1)
        self.assertEqual(node.status_pub.messages[-1], response.message)
        self.assertFalse(node.geometry_pub.messages[-1].valid)
        self.assertEqual(node.geometry_pub.messages[-1].failure_reason, response.message)
        self.assertIsNone(node.previous_object_axes_base)
        self.assertEqual(node.plan_pub.messages[-1].poses, [])

    def test_segment_snapshot_sends_only_target_depth_to_remote_path(self):
        context_depth = np.full((4, 5), 2200, dtype=np.uint16)
        target_depth = np.zeros((4, 5), dtype=np.uint16)
        target_depth[1:3, 2:4] = 2200
        snapshot = SnapshotResult(
            ok=True,
            failure_code='',
            failure_reason='',
            color_bgr=np.zeros((4, 5, 3), dtype=np.uint8),
            depth_raw=context_depth,
            target_depth_raw=target_depth,
            object_mask=np.where(target_depth > 0, 255, 0).astype(np.uint8),
            bbox=(1, 1, 3, 2),
            object_msg=types.SimpleNamespace(detected=True),
            stamp_sec=10.0,
            frame_id='camera_link',
            quality=DepthQuality(3, 4, 4, 1.0, 0.22, 0.0),
            source_mode='instance_mask',
        )
        node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
        node.use_perception_roi = True

        remote_depth, _message = node._depth_for_remote(snapshot)

        np.testing.assert_array_equal(remote_depth, target_depth)
        self.assertEqual(int(remote_depth[0, 0]), 0)

    def test_geometry_message_maps_obb_and_snapshot_quality(self):
        snapshot = make_snapshot(
            np.asarray(
                [
                    [0, 0, 0, 0],
                    [0, 2200, 2200, 0],
                    [0, 2200, 2200, 0],
                ],
                dtype=np.uint16,
            )
        )
        estimate = make_geometry_estimate()
        stamp = remote_node.rospy.Time(10, 123)

        message = remote_node.geometry_estimate_to_message(
            estimate,
            snapshot=snapshot,
            stamp=stamp,
            label='carton',
        )

        self.assertTrue(message.valid)
        self.assertEqual(message.header.frame_id, 'base_link')
        self.assertEqual(message.header.stamp.to_nsec(), 10_000_000_123)
        self.assertEqual(message.label, 'carton')
        self.assertEqual(message.source_mode, 'instance_mask')
        self.assertAlmostEqual(message.pose_base.position.x, 0.40)
        self.assertAlmostEqual(message.size_xyz_m.x, 0.24)
        self.assertAlmostEqual(message.size_xyz_m.y, 0.16)
        self.assertAlmostEqual(message.size_xyz_m.z, 0.10)
        self.assertEqual(message.valid_depth_points, 4)
        self.assertAlmostEqual(message.valid_depth_ratio, 1.0)
        self.assertAlmostEqual(message.depth_mad_m, 0.0015)
        self.assertEqual(message.fused_frames, 3)
        self.assertAlmostEqual(message.support_inlier_ratio, 0.82)
        self.assertEqual(message.object_point_count, 3)
        quaternion = np.asarray(
            [
                message.pose_base.orientation.x,
                message.pose_base.orientation.y,
                message.pose_base.orientation.z,
                message.pose_base.orientation.w,
            ]
        )
        self.assertAlmostEqual(float(np.linalg.norm(quaternion)), 1.0)
        np.testing.assert_allclose(
            remote_node.quaternion_matrix(quaternion)[:3, :3],
            estimate.axes_base,
            atol=1e-7,
        )

    def test_snapshot_geometry_tf_uses_exact_stamp_and_optical_convention(self):
        class TfBuffer:
            def __init__(self):
                self.calls = []

            def lookup_transform(self, target, source, stamp, timeout):
                self.calls.append((target, source, stamp, timeout))
                return types.SimpleNamespace(
                    transform=types.SimpleNamespace(
                        translation=types.SimpleNamespace(x=0.1, y=0.2, z=0.3),
                        rotation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
                    )
                )

        snapshot = make_snapshot(np.ones((3, 4), dtype=np.uint16) * 2200)
        stamp = remote_node.rospy.Time(10, 123)
        node = make_processing_node()
        buffer = TfBuffer()
        node.tf_buffer = buffer
        node.pose_estimator.tf_buffer = buffer
        node.pose_estimator.tf_timeout_sec = 0.2
        node.pose_estimator.base_frame = 'map'

        transform = node._snapshot_base_optical_transform(snapshot, stamp)

        self.assertEqual(len(buffer.calls), 1)
        self.assertEqual(buffer.calls[0][0], 'base_link')
        self.assertEqual(buffer.calls[0][1], 'camera_link')
        self.assertEqual(buffer.calls[0][2].to_nsec(), 10_000_000_123)
        np.testing.assert_allclose(transform[:3, :3], remote_node.OPTICAL_TO_ROS_CAMERA)
        np.testing.assert_allclose(transform[:3, 3], [0.1, 0.2, 0.3])
        message = remote_node.geometry_estimate_to_message(
            make_geometry_estimate(),
            snapshot=snapshot,
            stamp=stamp,
            label='carton',
        )
        self.assertEqual(message.header.frame_id, 'base_link')
        self.assertAlmostEqual(message.pose_base.position.x, 0.40)

    def test_snapshot_geometry_tf_never_uses_static_fallback(self):
        snapshot = make_snapshot(np.ones((3, 4), dtype=np.uint16) * 2200)
        stamp = remote_node.rospy.Time(10, 123)
        node = make_processing_node()
        node.tf_buffer = None
        node.pose_estimator.tf_buffer = None
        node.pose_estimator.allow_static_fallback = True
        node.pose_estimator.translation_xyz = [9.0, 8.0, 7.0]

        with self.assertRaisesRegex(RuntimeError, 'snapshot-time TF'):
            node._snapshot_base_optical_transform(snapshot, stamp)

    def test_snapshot_geometry_tf_lookup_failure_reports_tf_unavailable(self):
        class FailingBuffer:
            def lookup_transform(self, *_args, **_kwargs):
                raise RuntimeError('exact transform missing')

        snapshot = make_snapshot(np.ones((3, 4), dtype=np.uint16) * 2200)
        client = RecordingClient()
        node = make_processing_node(client)
        node.tf_buffer = FailingBuffer()
        node.pose_estimator.tf_buffer = node.tf_buffer

        ok, message = node._process_frame(snapshot, manual=True)

        self.assertFalse(ok)
        self.assertEqual(message, 'TF_UNAVAILABLE: exact transform missing')
        self.assertEqual(client.calls, [])
        self.assertFalse(node.geometry_pub.messages[-1].valid)
        self.assertEqual(node.geometry_pub.messages[-1].failure_reason, message)
        self.assertIsNone(node.previous_object_axes_base)

    def test_process_frame_rejects_insufficient_depth_without_tf_or_predict(self):
        snapshot = make_snapshot(
            np.asarray(
                [
                    [0, 0, 0, 0],
                    [0, 2200, 0, 0],
                    [0, 0, 0, 0],
                ],
                dtype=np.uint16,
            )
        )
        client = RecordingClient()
        node = make_processing_node(client)
        node.geometry_min_object_points = 3
        node._snapshot_base_optical_transform = lambda *_args: (_ for _ in ()).throw(
            AssertionError('TF must not be resolved for insufficient target depth')
        )

        ok, message = node._process_frame(snapshot, manual=True)

        self.assertFalse(ok)
        self.assertTrue(message.startswith('DEPTH_INSUFFICIENT: '))
        self.assertEqual(client.calls, [])
        self.assertFalse(node.geometry_pub.messages[-1].valid)
        self.assertEqual(node.geometry_pub.messages[-1].failure_reason, message)
        self.assertIsNone(node.previous_object_axes_base)
        self.assertEqual(node.plan_pub.messages[-1].poses, [])

    def test_process_frame_rejects_snapshot_whose_locked_target_is_lost(self):
        snapshot = make_snapshot(np.ones((3, 4), dtype=np.uint16) * 2200)
        snapshot.object_msg.detected = False
        client = RecordingClient()
        node = make_processing_node(client)

        ok, message = node._process_frame(snapshot, manual=True)

        self.assertFalse(ok)
        self.assertTrue(message.startswith('TARGET_LOST: '))
        self.assertEqual(client.calls, [])
        self.assertFalse(node.geometry_pub.messages[-1].valid)
        self.assertIsNone(node.previous_object_axes_base)
        self.assertEqual(node.plan_pub.messages[-1].poses, [])

    def test_process_frame_geometry_failure_invalidates_plan_and_axes(self):
        snapshot = make_snapshot(np.ones((3, 4), dtype=np.uint16) * 2200)
        for code in ('SUPPORT_PLANE_INVALID', 'OBB_INVALID'):
            with self.subTest(code=code):
                client = RecordingClient()
                node = make_processing_node(client)
                original_estimator = remote_node.estimate_object_geometry
                remote_node.estimate_object_geometry = lambda **_kwargs: make_geometry_estimate(
                    ok=False,
                    code=code,
                    reason='synthetic geometry failure',
                )
                node._snapshot_base_optical_transform = lambda *_args: np.eye(4)
                try:
                    ok, message = node._process_frame(snapshot, manual=True)
                finally:
                    remote_node.estimate_object_geometry = original_estimator

                self.assertFalse(ok)
                self.assertEqual(
                    message,
                    '%s: synthetic geometry failure' % code,
                )
                self.assertEqual(client.calls, [])
                self.assertFalse(node.geometry_pub.messages[-1].valid)
                self.assertEqual(node.geometry_pub.messages[-1].failure_reason, message)
                self.assertIsNone(node.previous_object_axes_base)
                self.assertEqual(node.plan_pub.messages[-1].poses, [])

    def test_process_frame_publishes_geometry_and_sends_only_target_depth(self):
        target_depth = np.asarray(
            [
                [0, 0, 0, 0],
                [0, 2200, 2200, 0],
                [0, 2200, 2200, 0],
            ],
            dtype=np.uint16,
        )
        snapshot = make_snapshot(target_depth)
        client = RecordingClient(candidates=[])
        node = make_processing_node(client)
        original_estimator = remote_node.estimate_object_geometry
        remote_node.estimate_object_geometry = lambda **_kwargs: make_geometry_estimate()
        node._snapshot_base_optical_transform = lambda *_args: np.eye(4)
        try:
            ok, message = node._process_frame(snapshot, manual=True)
        finally:
            remote_node.estimate_object_geometry = original_estimator

        self.assertFalse(ok)
        self.assertTrue(message.startswith('NO_RAW_CANDIDATE: '))
        self.assertEqual(len(client.calls), 1)
        np.testing.assert_array_equal(client.calls[0]['depth'], target_depth)
        self.assertEqual(int(client.calls[0]['depth'][0, 0]), 0)
        self.assertFalse(node.geometry_pub.messages[-1].valid)
        self.assertEqual(node.geometry_pub.messages[-1].failure_reason, message)
        self.assertEqual(node.geometry_pub.messages[-1].source_mode, 'instance_mask')
        self.assertIsNone(node.previous_object_axes_base)
        self.assertIsNone(node.latest_target_cloud_base_xyz)

    def test_gripper_contract_mismatch_fails_before_wsl_candidate_request(self):
        snapshot = make_snapshot(np.ones((3, 4), dtype=np.uint16) * 2200)
        client = RecordingClient(
            candidates=[
                RemoteGraspCandidate(
                    0.9,
                    np.array([0.40, -0.10, 0.25]),
                    np.array([0.0, 0.0, 0.0, 1.0]),
                    0.04,
                )
            ]
        )
        node = make_processing_node(client)
        node.gripper_contract_validator = lambda *_args, **_kwargs: 'synthetic 49 mm mismatch'
        original_estimator = remote_node.estimate_object_geometry
        remote_node.estimate_object_geometry = lambda **_kwargs: make_geometry_estimate(
            center_base=[0.40, -0.10, 0.25],
            size_xyz_m=[0.08, 0.04, 0.06],
        )
        node._snapshot_base_optical_transform = lambda *_args: np.eye(4)
        try:
            ok, message = node._process_frame(snapshot, manual=True)
        finally:
            remote_node.estimate_object_geometry = original_estimator

        self.assertFalse(ok)
        self.assertEqual(
            message,
            'GRIPPER_MODEL_MISMATCH: synthetic 49 mm mismatch',
        )
        self.assertEqual(client.calls, [])
        self.assertFalse(node.geometry_pub.messages[-1].valid)
        self.assertEqual(node.geometry_pub.messages[-1].failure_reason, message)
        self.assertEqual(node.plan_pub.messages[-1].poses, [])

    def test_all_analytical_rejections_return_counts_and_invalidate_geometry(self):
        snapshot = make_snapshot(np.ones((3, 4), dtype=np.uint16) * 2200)
        client = RecordingClient(
            candidates=[
                RemoteGraspCandidate(
                    0.99,
                    np.array([0.40, -0.10, 0.25]),
                    np.array([0.0, 0.0, 0.0, 1.0]),
                    0.001,
                )
            ]
        )
        node = make_processing_node(client)
        node.pose_estimator = RecordingPoseEstimator()
        node._plan_reachable = lambda _pose: (_ for _ in ()).throw(
            AssertionError('IK must not run after analytical rejection')
        )
        original_estimator = remote_node.estimate_object_geometry
        remote_node.estimate_object_geometry = lambda **_kwargs: make_geometry_estimate(
            center_base=[0.40, -0.10, 0.25],
            size_xyz_m=[0.08, 0.047, 0.06],
        )
        node._snapshot_base_optical_transform = lambda *_args: np.eye(4)
        try:
            ok, message = node._process_frame(snapshot, manual=True)
        finally:
            remote_node.estimate_object_geometry = original_estimator

        self.assertFalse(ok)
        self.assertTrue(message.startswith('NO_GEOMETRIC_CANDIDATE: '))
        self.assertIn('raw=1', message)
        self.assertIn('after_transform=1', message)
        self.assertIn('after_center=1', message)
        self.assertIn('after_jaw_width=0', message)
        self.assertIn('GRIPPER_TOO_NARROW=1', message)
        self.assertFalse(node.geometry_pub.messages[-1].valid)
        self.assertEqual(node.geometry_pub.messages[-1].failure_reason, message)
        self.assertEqual(node.plan_pub.messages[-1].poses, [])

    def test_process_frame_keeps_model_width_diagnostic_and_stores_required_width(self):
        snapshot = make_snapshot(np.ones((3, 4), dtype=np.uint16) * 2200)
        client = RecordingClient(
            candidates=[
                RemoteGraspCandidate(
                    0.99,
                    np.array([0.65, -0.10, 0.25]),
                    np.array([0.0, 0.0, 0.0, 1.0]),
                    0.001,
                ),
                RemoteGraspCandidate(
                    0.40,
                    np.array([0.40, -0.10, 0.25]),
                    np.array([0.0, 0.0, 0.0, 1.0]),
                    0.090,
                ),
            ]
        )
        node = make_processing_node(client)
        node.pose_estimator = RecordingPoseEstimator()
        node._plan_reachable = lambda _pose: True
        original_estimator = remote_node.estimate_object_geometry
        remote_node.estimate_object_geometry = lambda **_kwargs: make_geometry_estimate(
            center_base=[0.40, -0.10, 0.25],
            size_xyz_m=[0.08, 0.04, 0.06],
        )
        node._snapshot_base_optical_transform = lambda *_args: np.eye(4)
        try:
            ok, message = node._process_frame(snapshot, manual=True)
        finally:
            remote_node.estimate_object_geometry = original_estimator

        self.assertTrue(ok)
        self.assertEqual(len(node.plan_pub.messages[-1].poses), 4)
        self.assertAlmostEqual(node.selected_required_open_width_m, 0.044)
        self.assertAlmostEqual(node._selected_candidate_gate.required_open_width_m, 0.044)
        self.assertIn('model_width=0.090', message)
        self.assertIn('required_open=0.044', message)
        self.assertEqual(client.calls[0]['kwargs']['max_gripper_width_m'], 0.0)

    def test_plan_publish_exception_invalidates_geometry_and_selected_gate(self):
        snapshot = make_snapshot(np.ones((3, 4), dtype=np.uint16) * 2200)
        client = RecordingClient(
            candidates=[
                RemoteGraspCandidate(
                    0.90,
                    np.array([0.40, -0.10, 0.25]),
                    np.array([0.0, 0.0, 0.0, 1.0]),
                    0.04,
                )
            ]
        )
        node = make_processing_node(client)
        node.pose_estimator = RecordingPoseEstimator()
        node._plan_reachable = lambda _pose: True

        class FailNonemptyPlanPublisher:
            def __init__(self):
                self.messages = []

            def publish(self, message):
                if len(getattr(message, 'poses', ())) > 0:
                    raise RuntimeError('synthetic nonempty plan publish failure')
                self.messages.append(message)

        node.plan_pub = FailNonemptyPlanPublisher()
        original_estimator = remote_node.estimate_object_geometry
        remote_node.estimate_object_geometry = lambda **_kwargs: make_geometry_estimate(
            center_base=[0.40, -0.10, 0.25],
            size_xyz_m=[0.08, 0.04, 0.06],
        )
        node._snapshot_base_optical_transform = lambda *_args: np.eye(4)
        try:
            ok, message = node._process_frame(snapshot, manual=True)
        finally:
            remote_node.estimate_object_geometry = original_estimator

        self.assertFalse(ok)
        self.assertIn('synthetic nonempty plan publish failure', message)
        self.assertIsNone(node._selected_candidate_gate)
        self.assertIsNone(node.selected_required_open_width_m)
        self.assertFalse(node.latest_object_geometry.valid)
        self.assertEqual(node.plan_pub.messages[-1].poses, [])

    def test_candidate_rank_exception_invalidates_the_current_geometry_generation(self):
        snapshot = make_snapshot(np.ones((3, 4), dtype=np.uint16) * 2200)
        client = RecordingClient(
            candidates=[
                RemoteGraspCandidate(
                    0.90,
                    np.array([0.40, -0.10, 0.25]),
                    np.array([0.0, 0.0, 0.0, 1.0]),
                    0.04,
                )
            ]
        )
        node = make_processing_node(client)
        node.pose_estimator = RecordingPoseEstimator()
        node._plan_reachable = lambda _pose: True
        node._candidate_rank = lambda *_args: (_ for _ in ()).throw(
            RuntimeError('rank exploded')
        )
        original_estimator = remote_node.estimate_object_geometry
        remote_node.estimate_object_geometry = lambda **_kwargs: make_geometry_estimate(
            center_base=[0.40, -0.10, 0.25],
            size_xyz_m=[0.08, 0.04, 0.06],
        )
        node._snapshot_base_optical_transform = lambda *_args: np.eye(4)
        try:
            ok, message = node._process_frame(snapshot, manual=True)
        finally:
            remote_node.estimate_object_geometry = original_estimator

        self.assertFalse(ok)
        self.assertEqual(
            message,
            'PLAN_FAILED: remote 6D planning failed: rank exploded',
        )
        self.assertEqual(node._last_geometry_invalidation_code, 'PLAN_FAILED')
        self.assertFalse(node.latest_object_geometry.valid)
        self.assertIsNone(node._latest_geometry_estimate)
        self.assertIsNone(node.previous_object_axes_base)
        self.assertIsNone(node._selected_candidate_gate)
        self.assertIsNone(node.selected_required_open_width_m)
        self.assertEqual(node.plan_pub.messages[-1].poses, [])

    def test_post_publish_failure_preserves_concurrent_target_loss_reason(self):
        snapshot = make_snapshot(np.ones((3, 4), dtype=np.uint16) * 2200)
        client = RecordingClient(
            candidates=[
                RemoteGraspCandidate(
                    0.90,
                    np.array([0.40, -0.10, 0.25]),
                    np.array([0.0, 0.0, 0.0, 1.0]),
                    0.04,
                )
            ]
        )
        node = make_processing_node(client)
        node.pose_estimator = RecordingPoseEstimator()
        node._plan_reachable = lambda _pose: True
        plan_published = threading.Event()
        diagnostic_started = threading.Event()
        invalidation_done = threading.Event()

        class MarkPlanPublisher:
            def __init__(self):
                self.messages = []

            def publish(self, message):
                self.messages.append(message)
                if len(getattr(message, 'poses', ())) > 0:
                    plan_published.set()

        node.plan_pub = MarkPlanPublisher()
        original_support_metrics = node._candidate_support_geometry_metrics

        def fail_diagnostic_after_plan(candidate):
            if plan_published.is_set():
                diagnostic_started.set()
                if not invalidation_done.wait(2.0):
                    raise AssertionError('concurrent invalidation did not finish')
                raise RuntimeError('synthetic post-publication diagnostic failure')
            return original_support_metrics(candidate)

        node._candidate_support_geometry_metrics = fail_diagnostic_after_plan

        def invalidate_after_plan():
            if diagnostic_started.wait(2.0):
                node._invalidate_geometry(
                    'TARGET_LOST',
                    'concurrent target loss after plan publication',
                    stamp=remote_node.rospy.Time(10, 123),
                    snapshot=snapshot,
                )
                invalidation_done.set()

        invalidator = threading.Thread(target=invalidate_after_plan)
        invalidator.start()
        original_estimator = remote_node.estimate_object_geometry
        remote_node.estimate_object_geometry = lambda **_kwargs: make_geometry_estimate(
            center_base=[0.40, -0.10, 0.25],
            size_xyz_m=[0.08, 0.04, 0.06],
        )
        node._snapshot_base_optical_transform = lambda *_args: np.eye(4)
        try:
            ok, message = node._process_frame(snapshot, manual=True)
        finally:
            remote_node.estimate_object_geometry = original_estimator
            invalidator.join(2.0)

        self.assertFalse(ok)
        self.assertEqual(
            message,
            'TARGET_LOST: concurrent target loss after plan publication',
        )
        self.assertEqual(node._last_geometry_invalidation_code, 'TARGET_LOST')
        self.assertIsNone(node._selected_candidate_gate)
        self.assertIsNone(node.selected_required_open_width_m)
        self.assertFalse(node.latest_object_geometry.valid)
        self.assertEqual(node.plan_pub.messages[-1].poses, [])

    def test_detect_snapshot_uses_bbox_foreground_with_same_geometry_contract(self):
        snapshot = make_snapshot(
            np.zeros((3, 4), dtype=np.uint16),
            source_mode='bbox_depth',
        )
        foreground_roi = np.asarray(
            [
                [False, False, False, False],
                [False, True, True, False],
                [False, True, True, False],
            ]
        )
        client = RecordingClient(candidates=[])
        node = make_processing_node(client)
        node.geometry_min_object_points = 2
        node._foreground_mask_for_roi = (
            lambda _depth, _roi, min_points: (foreground_roi, 4)
        )
        captured = {}
        original_estimator = remote_node.estimate_object_geometry

        def fake_estimator(**kwargs):
            captured.update(kwargs)
            return make_geometry_estimate(source_mode='bbox_depth')

        remote_node.estimate_object_geometry = fake_estimator
        node._snapshot_base_optical_transform = lambda *_args: np.eye(4)
        try:
            ok, message = node._process_frame(snapshot, manual=True)
        finally:
            remote_node.estimate_object_geometry = original_estimator

        expected_depth = np.where(foreground_roi, snapshot.depth_raw, 0)
        self.assertFalse(ok)
        self.assertTrue(message.startswith('NO_RAW_CANDIDATE: '))
        np.testing.assert_array_equal(captured['target_depth_raw'], expected_depth)
        self.assertEqual(captured['source_mode'], 'bbox_depth')
        np.testing.assert_array_equal(client.calls[0]['depth'], expected_depth)
        self.assertEqual(node.geometry_pub.messages[-1].source_mode, 'bbox_depth')
        self.assertFalse(node.geometry_pub.messages[-1].valid)
        self.assertEqual(node.geometry_pub.messages[-1].failure_reason, message)

    def test_target_loss_during_geometry_estimate_blocks_late_activation(self):
        snapshot = make_snapshot(np.ones((3, 4), dtype=np.uint16) * 2200)
        client = RecordingClient(candidates=[])
        node = make_processing_node(client)

        def invalidating_estimator(**_kwargs):
            node._invalidate_geometry(
                'TARGET_LOST',
                'target disappeared during geometry estimation',
                stamp=remote_node.rospy.Time(10, 123),
                snapshot=snapshot,
            )
            return make_geometry_estimate()

        original_estimator = remote_node.estimate_object_geometry
        remote_node.estimate_object_geometry = invalidating_estimator
        node._snapshot_base_optical_transform = lambda *_args: np.eye(4)
        try:
            ok, message = node._process_frame(snapshot, manual=True)
        finally:
            remote_node.estimate_object_geometry = original_estimator

        self.assertFalse(ok)
        self.assertTrue(message.startswith('TARGET_LOST: '))
        self.assertEqual(client.calls, [])
        self.assertFalse(node.geometry_pub.messages[-1].valid)
        self.assertEqual(
            node.geometry_pub.messages[-1].failure_reason,
            'TARGET_LOST: target disappeared during geometry estimation',
        )
        self.assertIsNone(node.previous_object_axes_base)
        self.assertIsNone(node.latest_target_cloud_base_xyz)

    def test_target_loss_serializes_with_activation_and_finishes_invalid(self):
        snapshot = make_snapshot(np.ones((3, 4), dtype=np.uint16) * 2200)
        node = make_processing_node()

        class BlockingValidPublisher:
            def __init__(self):
                self.messages = []
                self.valid_started = threading.Event()
                self.release_valid = threading.Event()

            def publish(self, message):
                if bool(getattr(message, 'valid', False)):
                    self.valid_started.set()
                    self.release_valid.wait(2.0)
                self.messages.append(message)

        publisher = BlockingValidPublisher()
        node.geometry_pub = publisher
        activation_done = threading.Event()
        invalidation_done = threading.Event()

        def activate():
            node._activate_geometry(
                make_geometry_estimate(),
                snapshot,
                remote_node.rospy.Time(10, 123),
                np.eye(4),
                expected_generation=0,
            )
            activation_done.set()

        def invalidate():
            node._invalidate_geometry(
                'TARGET_LOST',
                'concurrent target loss',
                stamp=remote_node.rospy.Time(10, 123),
                snapshot=snapshot,
            )
            invalidation_done.set()

        activation_thread = threading.Thread(target=activate)
        activation_thread.start()
        self.assertTrue(publisher.valid_started.wait(1.0))
        invalidation_thread = threading.Thread(target=invalidate)
        invalidation_thread.start()
        invalidation_done.wait(0.1)
        publisher.release_valid.set()
        activation_thread.join(2.0)
        invalidation_thread.join(2.0)

        self.assertTrue(activation_done.is_set())
        self.assertTrue(invalidation_done.is_set())
        self.assertFalse(publisher.messages[-1].valid)
        self.assertEqual(
            publisher.messages[-1].failure_reason,
            'TARGET_LOST: concurrent target loss',
        )
        self.assertIsNone(node.previous_object_axes_base)
        self.assertIsNone(node.latest_target_cloud_base_xyz)

    def test_target_loss_serializes_with_plan_publication_and_clears_legacy_plan(self):
        snapshot = make_snapshot(np.ones((3, 4), dtype=np.uint16) * 2200)
        node = make_processing_node()

        class BlockingPlanPublisher:
            def __init__(self):
                self.messages = []
                self.plan_started = threading.Event()
                self.release_plan = threading.Event()

            def publish(self, message):
                if len(getattr(message, 'poses', ())) > 0:
                    self.plan_started.set()
                    self.release_plan.wait(2.0)
                self.messages.append(message)

        publisher = BlockingPlanPublisher()
        node.plan_pub = publisher
        plan = remote_node.PoseArray()
        plan.poses = [PoseStamped().pose]
        publication_done = threading.Event()
        invalidation_done = threading.Event()

        def publish_plan():
            node._publish_legacy_plan_if_current(plan, expected_generation=0)
            publication_done.set()

        def invalidate():
            node._invalidate_geometry(
                'TARGET_LOST',
                'concurrent target loss',
                stamp=remote_node.rospy.Time(10, 123),
                snapshot=snapshot,
            )
            invalidation_done.set()

        publication_thread = threading.Thread(target=publish_plan)
        publication_thread.start()
        self.assertTrue(publisher.plan_started.wait(1.0))
        invalidation_thread = threading.Thread(target=invalidate)
        invalidation_thread.start()
        invalidation_done.wait(0.1)
        publisher.release_plan.set()
        publication_thread.join(2.0)
        invalidation_thread.join(2.0)

        self.assertTrue(publication_done.is_set())
        self.assertTrue(invalidation_done.is_set())
        self.assertEqual(publisher.messages[-1].poses, [])

    def test_target_loss_during_predict_cannot_resurrect_a_plan(self):
        target_depth = np.asarray(
            [
                [0, 0, 0, 0],
                [0, 2200, 2200, 0],
                [0, 2200, 2200, 0],
            ],
            dtype=np.uint16,
        )
        snapshot = make_snapshot(target_depth)
        node = make_processing_node()

        class InvalidatingClient(RecordingClient):
            def predict(self, color, depth, intrinsics, **kwargs):
                self.calls.append({'depth': np.asarray(depth).copy()})
                node._invalidate_geometry(
                    'TARGET_LOST',
                    'target disappeared during inference',
                    stamp=remote_node.rospy.Time(10, 123),
                    snapshot=snapshot,
                )
                return []

        node.client = InvalidatingClient()
        original_estimator = remote_node.estimate_object_geometry
        remote_node.estimate_object_geometry = lambda **_kwargs: make_geometry_estimate()
        node._snapshot_base_optical_transform = lambda *_args: np.eye(4)
        try:
            ok, message = node._process_frame(snapshot, manual=True)
        finally:
            remote_node.estimate_object_geometry = original_estimator

        self.assertFalse(ok)
        self.assertTrue(message.startswith('TARGET_LOST: '))
        self.assertFalse(node.geometry_pub.messages[-1].valid)
        self.assertIsNone(node.previous_object_axes_base)
        self.assertEqual(node.plan_pub.messages[-1].poses, [])

    def test_model_reload_after_activation_finishes_with_invalid_geometry(self):
        snapshot = make_snapshot(np.ones((3, 4), dtype=np.uint16) * 2200)
        node = make_processing_node()

        class ReloadingClient(RecordingClient):
            def predict(self, color, depth, intrinsics, **kwargs):
                self.calls.append({'depth': np.asarray(depth).copy()})
                node._invalidate_geometry(
                    'MODEL_RELOADED',
                    'model changed during inference',
                    stamp=remote_node.rospy.Time(10, 123),
                    snapshot=snapshot,
                )
                return []

        node.client = ReloadingClient()
        original_estimator = remote_node.estimate_object_geometry
        remote_node.estimate_object_geometry = lambda **_kwargs: make_geometry_estimate()
        node._snapshot_base_optical_transform = lambda *_args: np.eye(4)
        try:
            ok, message = node._process_frame(snapshot, manual=True)
        finally:
            remote_node.estimate_object_geometry = original_estimator

        self.assertFalse(ok)
        self.assertTrue(message.startswith('MODEL_RELOADED: '))
        self.assertFalse(node.geometry_pub.messages[-1].valid)
        self.assertEqual(
            node.geometry_pub.messages[-1].failure_reason,
            'MODEL_RELOADED: model changed during inference',
        )
        self.assertIsNone(node.previous_object_axes_base)

    def test_remote_prediction_error_codes_follow_exception_cause(self):
        connection = RuntimeError('wrapped connection')
        connection.__cause__ = urllib.error.URLError('refused')
        http = RuntimeError('wrapped HTTP')
        http.__cause__ = urllib.error.HTTPError(
            'http://wsl/predict',
            500,
            'failure',
            {},
            None,
        )

        self.assertEqual(
            remote_node.remote_prediction_failure_code(connection),
            'WSL_UNAVAILABLE',
        )
        self.assertEqual(
            remote_node.remote_prediction_failure_code(TimeoutError('timed out')),
            'WSL_UNAVAILABLE',
        )
        self.assertEqual(
            remote_node.remote_prediction_failure_code(http),
            'WSL_PREDICT_FAILED',
        )
        self.assertEqual(
            remote_node.remote_prediction_failure_code(ValueError('bad protocol')),
            'WSL_PREDICT_FAILED',
        )

    def test_process_frame_publishes_structured_remote_error_codes(self):
        snapshot = make_snapshot(np.ones((3, 4), dtype=np.uint16) * 2200)
        connection = RuntimeError('connection failed')
        connection.__cause__ = urllib.error.URLError('refused')
        cases = (
            (connection, 'WSL_UNAVAILABLE'),
            (ValueError('invalid decoded candidate'), 'WSL_PREDICT_FAILED'),
        )
        original_estimator = remote_node.estimate_object_geometry
        remote_node.estimate_object_geometry = lambda **_kwargs: make_geometry_estimate()
        try:
            for error, expected_code in cases:
                with self.subTest(expected_code=expected_code):
                    node = make_processing_node(RecordingClient(error=error))
                    node._snapshot_base_optical_transform = lambda *_args: np.eye(4)

                    ok, message = node._process_frame(snapshot, manual=True)

                    self.assertFalse(ok)
                    self.assertTrue(message.startswith(expected_code + ': '))
                    self.assertEqual(node.plan_pub.messages[-1].poses, [])
                    self.assertFalse(node.geometry_pub.messages[-1].valid)
                    self.assertEqual(
                        node.geometry_pub.messages[-1].failure_reason,
                        message,
                    )
                    self.assertIsNone(node.previous_object_axes_base)
                    self.assertIsNone(node.latest_target_cloud_base_xyz)
        finally:
            remote_node.estimate_object_geometry = original_estimator

    def test_target_loss_between_wsl_check_and_invalidation_is_preserved(self):
        snapshot = make_snapshot(np.ones((3, 4), dtype=np.uint16) * 2200)
        node = make_processing_node()
        stringify_started = threading.Event()
        target_loss_done = threading.Event()

        class WindowError(RuntimeError):
            def __str__(self):
                stringify_started.set()
                target_loss_done.wait(1.0)
                return 'connection failed after target loss'

        error = WindowError()
        error.__cause__ = urllib.error.URLError('refused')
        node.client = RecordingClient(error=error)

        def invalidate_target():
            stringify_started.wait(1.0)
            node._invalidate_geometry(
                'TARGET_LOST',
                'target lost inside WSL failure window',
                stamp=remote_node.rospy.Time(10, 123),
                snapshot=snapshot,
            )
            target_loss_done.set()

        invalidation_thread = threading.Thread(target=invalidate_target)
        invalidation_thread.start()
        original_estimator = remote_node.estimate_object_geometry
        remote_node.estimate_object_geometry = lambda **_kwargs: make_geometry_estimate()
        node._snapshot_base_optical_transform = lambda *_args: np.eye(4)
        try:
            ok, message = node._process_frame(snapshot, manual=True)
        finally:
            remote_node.estimate_object_geometry = original_estimator
            invalidation_thread.join(2.0)

        self.assertFalse(ok)
        self.assertTrue(target_loss_done.is_set())
        self.assertEqual(
            message,
            'TARGET_LOST: target lost inside WSL failure window',
        )
        self.assertFalse(node.geometry_pub.messages[-1].valid)
        self.assertEqual(node.geometry_pub.messages[-1].failure_reason, message)
        self.assertIsNone(node.previous_object_axes_base)
        self.assertIsNone(node.latest_target_cloud_base_xyz)
        self.assertEqual(node.plan_pub.messages[-1].poses, [])

    def test_model_reload_between_empty_check_and_invalidation_is_preserved(self):
        snapshot = make_snapshot(np.ones((3, 4), dtype=np.uint16) * 2200)
        node = make_processing_node(RecordingClient(candidates=[]))
        diagnostics_started = threading.Event()
        reload_done = threading.Event()

        def diagnostics(_remote_diagnostics):
            diagnostics_started.set()
            reload_done.wait(1.0)
            return ' [diagnostic-window]'

        node._candidate_failure_diagnostics = diagnostics

        def invalidate_model():
            diagnostics_started.wait(1.0)
            node._invalidate_geometry(
                'MODEL_RELOADED',
                'model changed inside empty-candidate window',
                stamp=remote_node.rospy.Time(10, 123),
                snapshot=snapshot,
            )
            reload_done.set()

        invalidation_thread = threading.Thread(target=invalidate_model)
        invalidation_thread.start()
        original_estimator = remote_node.estimate_object_geometry
        remote_node.estimate_object_geometry = lambda **_kwargs: make_geometry_estimate()
        node._snapshot_base_optical_transform = lambda *_args: np.eye(4)
        try:
            ok, message = node._process_frame(snapshot, manual=True)
        finally:
            remote_node.estimate_object_geometry = original_estimator
            invalidation_thread.join(2.0)

        self.assertFalse(ok)
        self.assertTrue(reload_done.is_set())
        self.assertEqual(
            message,
            'MODEL_RELOADED: model changed inside empty-candidate window',
        )
        self.assertFalse(node.geometry_pub.messages[-1].valid)
        self.assertEqual(node.geometry_pub.messages[-1].failure_reason, message)
        self.assertIsNone(node.previous_object_axes_base)
        self.assertIsNone(node.latest_target_cloud_base_xyz)
        self.assertEqual(node.plan_pub.messages[-1].poses, [])

    def test_target_loss_publishes_invalid_geometry_and_clears_plan(self):
        node = make_processing_node()
        node._object_lock = DummyLock()
        node.latest_object = types.SimpleNamespace(detected=True)
        node.latest_object_time = remote_node.rospy.Time.from_sec(9.0)
        lost = types.SimpleNamespace(
            detected=False,
            label='carton',
            header=types.SimpleNamespace(stamp=remote_node.rospy.Time.from_sec(10.0)),
        )

        original_now = remote_node.rospy.Time.now
        remote_node.rospy.Time.now = staticmethod(
            lambda: remote_node.rospy.Time.from_sec(10.0)
        )
        try:
            node.object_cb(lost)
        finally:
            remote_node.rospy.Time.now = original_now

        self.assertFalse(node.geometry_pub.messages[-1].valid)
        self.assertTrue(
            node.geometry_pub.messages[-1].failure_reason.startswith('TARGET_LOST: ')
        )
        self.assertIsNone(node.previous_object_axes_base)
        self.assertEqual(node.plan_pub.messages[-1].poses, [])

    def test_model_choice_change_invalidates_previous_geometry(self):
        node = make_processing_node()
        node._last_model_choice = 'original'
        original_get_param = remote_node.rospy.get_param
        remote_node.rospy.get_param = lambda name, default=None: {
            '/perception': {
                'detector': 'yolo',
                'yolo_model_choice': 'carton_segment',
                'yolo_target_class': 'carton',
            },
        }.get(name, default)
        try:
            requires_mask = node._active_profile_requires_mask()
        finally:
            remote_node.rospy.get_param = original_get_param

        self.assertTrue(requires_mask)
        self.assertFalse(node.geometry_pub.messages[-1].valid)
        self.assertTrue(
            node.geometry_pub.messages[-1].failure_reason.startswith('MODEL_RELOADED: ')
        )
        self.assertIsNone(node.previous_object_axes_base)

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

    def test_nonfinite_rank_tuple_is_a_candidate_hard_reject(self):
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
            candidate_rank_fn=lambda *_args: (0.1, float('inf')),
        )

        self.assertIsNone(selected)
        self.assertIsNone(pose)

    def test_invalid_rank_tuple_conversion_propagates(self):
        candidate = RemoteGraspCandidate(
            0.9,
            np.array([0.01, 0.0, 0.0]),
            np.array([0.0, 0.0, 0.0, 1.0]),
            0.04,
        )

        with self.assertRaises((TypeError, ValueError)):
            remote_node.select_first_reachable_candidate(
                [candidate],
                FakePoseEstimator(),
                lambda _pose: True,
                stamp=None,
                camera_frame='camera_link',
                candidate_frame_convention='ros_camera_link',
                candidate_rank_fn=lambda *_args: ('not-a-number',),
            )

    def test_analytical_geometry_gate_runs_before_reachability_and_high_score(self):
        candidates = [
            RemoteGraspCandidate(
                0.99,
                np.array([0.30, 0.0, 0.20]),
                np.array([0.0, 0.0, 0.0, 1.0]),
                0.01,
            ),
            RemoteGraspCandidate(
                0.40,
                np.array([0.20, 0.0, 0.20]),
                np.array([0.0, 0.0, 0.0, 1.0]),
                0.09,
            ),
        ]
        events = []

        def geometry_gate(raw, converted, pose, plan):
            events.append(('geometry', raw.score, len((plan.pregrasp, plan.approach, plan.grasp, plan.lift))))
            if raw.score > 0.9:
                return remote_node.CandidateGateResult(
                    False,
                    'CENTER_OUTSIDE_OBB',
                    'synthetic off-center candidate',
                    0.044,
                    0.10,
                    0.010,
                    1.0,
                    0.0,
                    0.10,
                    'center',
                    1,
                )
            return remote_node.CandidateGateResult(
                True, '', '', 0.044, 0.001, 0.010, 1.0, 0.0, 0.001, '', 6
            )

        def reachable(pose):
            events.append(('reachability', pose.pose.position.x))
            return True

        selected, pose = remote_node.select_first_reachable_candidate(
            candidates,
            RecordingPoseEstimator(),
            reachable,
            stamp=None,
            camera_frame='camera_link',
            candidate_frame_convention='ros_camera_link',
            candidate_geometry_fn=geometry_gate,
            grasp_config={
                'pregrasp_distance_m': 0.08,
                'final_approach_offset_m': 0.02,
                'lift_height_m': 0.05,
                'tool_approach_axis': 'z',
            },
        )

        self.assertAlmostEqual(selected.score, 0.40)
        self.assertAlmostEqual(selected.width_m, 0.09)
        self.assertAlmostEqual(selected.required_open_width_m, 0.044)
        self.assertAlmostEqual(pose.pose.position.x, 0.20)
        self.assertEqual(
            events,
            [
                ('geometry', 0.99, 4),
                ('geometry', 0.40, 4),
                ('reachability', 0.20),
            ],
        )

    def test_selector_uses_geometry_tuple_with_motion_before_model_score(self):
        candidates = [
            RemoteGraspCandidate(
                0.99,
                np.array([0.10, 0.0, 0.20]),
                np.array([0.0, 0.0, 0.0, 1.0]),
                0.04,
            ),
            RemoteGraspCandidate(
                0.10,
                np.array([0.20, 0.0, 0.20]),
                np.array([0.0, 0.0, 0.0, 1.0]),
                0.04,
            ),
        ]

        def geometry_gate(raw, _converted, _pose, _plan):
            motion_cost = 2.0 if raw.score > 0.9 else 1.0
            return remote_node.CandidateGateResult(
                True,
                '',
                '',
                0.044,
                0.001,
                0.010,
                1.0,
                motion_cost,
                0.001,
                '',
                6,
            )

        selected, _pose = remote_node.select_first_reachable_candidate(
            candidates,
            RecordingPoseEstimator(),
            lambda _pose: True,
            stamp=None,
            camera_frame='camera_link',
            candidate_frame_convention='ros_camera_link',
            candidate_geometry_fn=geometry_gate,
            grasp_config={'tool_approach_axis': 'z'},
        )

        self.assertAlmostEqual(selected.score, 0.10)

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
