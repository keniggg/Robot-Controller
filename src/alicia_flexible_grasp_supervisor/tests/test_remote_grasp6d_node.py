#!/usr/bin/env python3
from copy import deepcopy
import hashlib
import io
import importlib.util
import json
import pathlib
import sys
import tempfile
import threading
import types
import unittest
import urllib.error

import numpy as np
from geometry_msgs.msg import PoseStamped
from alicia_flexible_grasp_supervisor.msg import Grasp6DPlan


ROOT = pathlib.Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / 'src'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from alicia_flexible_grasp.vision.remote_grasp6d_client import RemoteGraspCandidate
from alicia_flexible_grasp.vision import mujoco_digital_twin_client
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
    def __init__(self, windows, mask_timeout_code='MASK_MISSING'):
        self.windows = list(windows)
        self.discarded = []
        self.collection_spans = []
        self.inference_latency_limits = []
        self.mask_timeout_code = str(mask_timeout_code)

    def wait_for_samples(
        self,
        count,
        timeout_sec,
        require_mask,
        max_age_sec,
        collection_span_sec=0.0,
        max_inference_latency_sec=None,
    ):
        del count, timeout_sec, require_mask, max_age_sec
        self.collection_spans.append(float(collection_span_sec))
        self.inference_latency_limits.append(float(max_inference_latency_sec))
        return self.windows.pop(0) if self.windows else []

    def discard_through(self, stamp_sec):
        raise AssertionError('retry discard must use exact integer nanoseconds, got %r' % stamp_sec)

    def discard_through_ns(self, stamp_ns):
        self.discarded.append(int(stamp_ns))

    def mask_timeout_failure(
        self,
        count,
        max_age_sec,
        max_inference_latency_sec=None,
    ):
        del count, max_age_sec, max_inference_latency_sec
        reasons = {
            'MASK_MISSING': 'no instance mask has been observed',
            'MASK_EMPTY': 'latest instance mask is empty',
            'MASK_STALE': 'instance masks were observed but no fresh exact timestamp-matched window is available',
        }
        return self.mask_timeout_code, reasons[self.mask_timeout_code]


class FakeTf2Module:
    class Buffer:
        pass

    class TransformListener:
        def __init__(self, buffer):
            self.buffer = buffer


def identity_base_camera_snapshot_transform():
    """Return T_base_optical for an identity base<-camera_link pose."""
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = remote_node.OPTICAL_TO_ROS_CAMERA
    return transform


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


def make_geometry_message(stamp_ns=10_000_000_123):
    snapshot = make_snapshot(
        np.ones((3, 4), dtype=np.uint16) * 2200,
        stamp_ns=stamp_ns,
    )
    return remote_node.geometry_estimate_to_message(
        make_geometry_estimate(),
        snapshot=snapshot,
        stamp=remote_node.RemoteGrasp6DNode._snapshot_ros_stamp(snapshot),
        label='carton',
    )


def make_selected_candidate(required_width_m=0.044):
    selected = RemoteGraspCandidate(
        score=0.91,
        translation_m=np.asarray([0.40, -0.10, 0.22]),
        quaternion_xyzw=np.asarray([0.0, 0.0, 0.0, 1.0]),
        width_m=0.039,
        depth_m=0.030,
    )
    grasp_pose = PoseStamped()
    grasp_pose.header.frame_id = 'base_link'
    grasp_pose.pose.position.x = 0.40
    grasp_pose.pose.position.y = -0.10
    grasp_pose.pose.position.z = 0.22
    grasp_pose.pose.orientation.w = 1.0
    selected._grasp_sequence = remote_node.make_grasp_sequence_from_grasp_pose(
        grasp_pose,
        pregrasp_distance_m=0.08,
        approach_offset_m=0.02,
        lift_height_m=0.05,
        tool_approach_axis='z',
    )
    selected.required_open_width_m = float(required_width_m)
    selected.candidate_source = 'graspnet'
    selected.source_lineage = ('graspnet',)
    return selected


def make_processing_node(client=None):
    node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
    node._request_lock = NonBlockingLock()
    node._geometry_state_lock = threading.RLock()
    node._geometry_invalidation_generation = 0
    node._last_geometry_invalidation_code = ''
    node._refresh_runtime_params = lambda: None
    node.plan_pub = RecordingPublisher()
    node.rich_plan_pub = RecordingPublisher()
    node.geometry_pub = RecordingPublisher()
    node.gate_audit_pub = RecordingPublisher()
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
    node._cached_tool_from_camera_stamp_ns = 0
    node.handeye_parent_frame = 'tool0'
    node.handeye_camera_frame = 'camera_link'
    node.handeye_translation_xyz = [0.0, 0.0, 0.0]
    node.handeye_rotation_xyzw = [0.0, 0.0, 0.0, 1.0]
    node.handeye_allow_static_fallback = True
    node.previous_object_axes_base = np.eye(3)
    node.latest_object_geometry = None
    node.latest_rich_plan = None
    node._last_model_choice = 'carton_segment'
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
    node.require_candidate_depth = True
    node.orientation_variant_quaternions = [np.array([0.0, 0.0, 0.0, 1.0])]
    node.model_grasp_to_tool_quaternion = np.asarray(
        remote_node.STRICT_MODEL_GRASP_TO_TOOL_QUATERNION,
        dtype=float,
    )
    node.candidate_frame_convention = 'ros_camera_link'
    node.gate_audit_enabled = True
    node.gate_audit_output_path = str(
        pathlib.Path(tempfile.gettempdir())
        / ('alicia-remote-grasp6d-test-audit-%x.json' % id(node))
    )
    node.mujoco_audit_output_path = str(
        pathlib.Path(tempfile.gettempdir())
        / ('alicia-remote-grasp6d-test-mujoco-audit-%x.json' % id(node))
    )
    node._latest_gate_audit_summary = {}
    node._latest_gate_audit_reference = {}
    node._active_gate_audit_report = None
    node.camera_depth_scale = 0.0001
    node.graspnet_input_mode = remote_node.MASKED_TARGET
    node.graspnet_input_context_margin_px = 1.0
    node.graspnet_input_context_expand_ratio = 0.0
    node.graspnet_input_context_max_margin_px = 2.0
    node.graspnet_input_target_guard_px = 0
    node.graspnet_input_support_band_m = 0.006
    node.graspnet_input_min_target_points = 1
    node.graspnet_input_min_support_points = 1
    node.graspnet_input_min_total_points = 1
    # Quality gates are never disabled with zero, including in offline tests.
    node.graspnet_input_min_target_fraction = 0.01
    node.graspnet_input_bbox_min_iou = 0.01
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


def frozen_input_config(node, mode=None, **overrides):
    values = {
        'mode': str(mode or node.graspnet_input_mode),
        'context_margin_px': node.graspnet_input_context_margin_px,
        'context_expand_ratio': node.graspnet_input_context_expand_ratio,
        'context_max_margin_px': node.graspnet_input_context_max_margin_px,
        'target_guard_px': node.graspnet_input_target_guard_px,
        'support_band_m': node.graspnet_input_support_band_m,
        'min_target_points': node.graspnet_input_min_target_points,
        'min_support_points': node.graspnet_input_min_support_points,
        'min_total_points': node.graspnet_input_min_total_points,
        'min_target_fraction': node.graspnet_input_min_target_fraction,
        'bbox_min_iou': node.graspnet_input_bbox_min_iou,
        'candidate_target_gate_enabled': node.candidate_target_gate_enabled,
    }
    values.update(overrides)
    return remote_node.FrozenGraspNetInputConfig(**values)


class RemoteGrasp6DNodeTest(unittest.TestCase):
    def test_build_rich_plan_copies_geometry_width_and_exact_snapshot_identity(self):
        stamp_ns = 1_700_000_000_000_000_123
        geometry = make_geometry_message(stamp_ns)
        selected = make_selected_candidate(required_width_m=0.044)

        plan = remote_node.build_rich_plan(
            selected,
            geometry,
            deepcopy(geometry.header),
            'carton_segment',
        )

        self.assertIsInstance(plan, Grasp6DPlan)
        self.assertTrue(plan.valid)
        self.assertEqual(len(plan.poses), 4)
        self.assertAlmostEqual(plan.score, 0.91)
        self.assertEqual(plan.candidate_source, 'graspnet')
        self.assertEqual(plan.candidate_source_lineage, ['graspnet'])
        self.assertTrue(plan.has_candidate_model_width)
        self.assertAlmostEqual(plan.candidate_width_m, 0.039)
        self.assertAlmostEqual(plan.required_open_width_m, 0.044)
        self.assertEqual(plan.object_geometry.source_mode, 'instance_mask')
        self.assertEqual(plan.model_choice, 'carton_segment')
        self.assertRegex(plan.plan_id, r'^[0-9a-f]{24}$')
        self.assertEqual(plan.header.stamp.to_nsec(), stamp_ns)

        geometry.pose_base.position.x = 99.0
        selected.required_open_width_m = 0.049
        self.assertAlmostEqual(plan.object_geometry.pose_base.position.x, 0.40)
        self.assertAlmostEqual(plan.required_open_width_m, 0.044)

    def test_build_rich_plan_preserves_geometry_source_without_model_width(self):
        geometry = make_geometry_message()
        selected = make_selected_candidate()
        normalized = types.SimpleNamespace(
            candidate_source='tabletop_geometry',
            source_lineage=('tabletop_geometry',),
            model_width_m=None,
            model_score=None,
            source_local_score=0.82,
            required_open_width_m=selected.required_open_width_m,
            grasp_sequence=selected._grasp_sequence,
        )

        plan = remote_node.build_rich_plan(
            normalized,
            geometry,
            deepcopy(geometry.header),
            'geometry_fallback',
        )

        self.assertEqual(plan.candidate_source, 'tabletop_geometry')
        self.assertEqual(plan.candidate_source_lineage, ['tabletop_geometry'])
        self.assertFalse(plan.has_candidate_model_width)
        self.assertEqual(plan.candidate_width_m, 0.0)
        self.assertAlmostEqual(plan.score, 0.82)

    def test_build_rich_plan_rejects_cross_snapshot_geometry_headers(self):
        selected = make_selected_candidate()
        geometry = make_geometry_message()
        snapshot_header = deepcopy(geometry.header)

        wrong_frame = deepcopy(geometry)
        wrong_frame.header.frame_id = 'map'
        with self.assertRaisesRegex(ValueError, 'frame'):
            remote_node.build_rich_plan(
                selected,
                wrong_frame,
                snapshot_header,
                'carton_segment',
            )

        wrong_stamp = deepcopy(geometry)
        wrong_stamp.header.stamp = remote_node.rospy.Time(10, 124)
        with self.assertRaisesRegex(ValueError, 'stamp'):
            remote_node.build_rich_plan(
                selected,
                wrong_stamp,
                snapshot_header,
                'carton_segment',
            )

    def test_rich_plan_id_is_stable_and_changes_for_every_bound_field(self):
        geometry = make_geometry_message()
        selected = make_selected_candidate()

        baseline = remote_node.build_rich_plan(
            selected, geometry, deepcopy(geometry.header), 'carton_segment'
        )
        duplicate = remote_node.build_rich_plan(
            deepcopy(selected), deepcopy(geometry), deepcopy(geometry.header), 'carton_segment'
        )
        self.assertEqual(duplicate.plan_id, baseline.plan_id)

        mutations = []
        changed = deepcopy(selected)
        changed.width_m += 0.001
        mutations.append((changed, deepcopy(geometry), deepcopy(geometry.header), 'carton_segment'))
        changed = deepcopy(selected)
        changed.required_open_width_m += 0.001
        mutations.append((changed, deepcopy(geometry), deepcopy(geometry.header), 'carton_segment'))
        changed = deepcopy(selected)
        changed._grasp_sequence.grasp.pose.position.x += 0.001
        mutations.append((changed, deepcopy(geometry), deepcopy(geometry.header), 'carton_segment'))
        changed_geometry = deepcopy(geometry)
        changed_geometry.pose_base.position.x += 0.001
        mutations.append((deepcopy(selected), changed_geometry, deepcopy(geometry.header), 'carton_segment'))
        changed_geometry = deepcopy(geometry)
        changed_geometry.size_xyz_m.y += 0.001
        mutations.append((deepcopy(selected), changed_geometry, deepcopy(geometry.header), 'carton_segment'))
        changed_geometry = deepcopy(geometry)
        changed_geometry.support_offset_m += 0.001
        mutations.append((deepcopy(selected), changed_geometry, deepcopy(geometry.header), 'carton_segment'))
        changed_header = deepcopy(geometry.header)
        changed_header.stamp = remote_node.rospy.Time(10, 124)
        changed_geometry = deepcopy(geometry)
        changed_geometry.header = deepcopy(changed_header)
        mutations.append((deepcopy(selected), changed_geometry, changed_header, 'carton_segment'))
        mutations.append((deepcopy(selected), deepcopy(geometry), deepcopy(geometry.header), 'original'))

        for candidate, geometry_value, header, model in mutations:
            changed_plan = remote_node.build_rich_plan(
                candidate, geometry_value, header, model
            )
            self.assertNotEqual(changed_plan.plan_id, baseline.plan_id)

    def test_rich_plan_id_survives_ros_float32_wire_round_trip(self):
        geometry = make_geometry_message()
        geometry.support_offset_m = 0.0137
        selected = make_selected_candidate(required_width_m=0.0437)
        selected.width_m = 0.0389
        plan = remote_node.build_rich_plan(
            selected,
            geometry,
            deepcopy(geometry.header),
            'carton_segment',
        )

        wire = io.BytesIO()
        plan.serialize(wire)
        received = Grasp6DPlan()
        received.deserialize(wire.getvalue())

        self.assertEqual(remote_node.compute_plan_id(received), plan.plan_id)
        self.assertEqual(received.plan_id, plan.plan_id)

    def test_rich_plan_publication_and_cache_are_deep_copied_atomically(self):
        node = make_processing_node()
        geometry = make_geometry_message()
        plan = remote_node.build_rich_plan(
            make_selected_candidate(),
            geometry,
            deepcopy(geometry.header),
            'carton_segment',
        )

        published, reason = node._publish_plan_pair_if_current(
            plan,
            expected_generation=0,
        )

        self.assertTrue(published, reason)
        self.assertEqual(node.rich_plan_pub.messages[-1].plan_id, plan.plan_id)
        self.assertEqual(len(node.plan_pub.messages[-1].poses), 4)
        plan.poses[0].position.x = 99.0
        self.assertNotEqual(node.rich_plan_pub.messages[-1].poses[0].position.x, 99.0)
        cached = node._latest_rich_plan_copy()
        cached.poses[0].position.x = -99.0
        self.assertNotEqual(node._latest_rich_plan_copy().poses[0].position.x, -99.0)

    def test_legacy_publication_failure_never_exposes_valid_rich_authority(self):
        class FailingLegacyPublisher(RecordingPublisher):
            def publish(self, message):
                super().publish(message)
                raise RuntimeError('synthetic legacy visualization failure')

        node = make_processing_node()
        node.plan_pub = FailingLegacyPublisher()
        geometry = make_geometry_message()
        plan = remote_node.build_rich_plan(
            make_selected_candidate(),
            geometry,
            deepcopy(geometry.header),
            'carton_segment',
        )

        with self.assertRaisesRegex(
            RuntimeError,
            'synthetic legacy visualization failure',
        ):
            node._publish_plan_pair_if_current(plan, expected_generation=0)

        self.assertEqual(len(node.plan_pub.messages), 1)
        self.assertEqual(node.rich_plan_pub.messages, [])
        self.assertIsNone(node._latest_rich_plan_copy())

    def test_geometry_invalidation_publishes_invalid_rich_and_empty_legacy(self):
        node = make_processing_node()
        geometry = make_geometry_message()
        valid = remote_node.build_rich_plan(
            make_selected_candidate(), geometry, deepcopy(geometry.header), 'carton_segment'
        )
        node._publish_plan_pair_if_current(valid, expected_generation=0)

        node._invalidate_geometry(
            'MODEL_RELOADED',
            'model changed',
            stamp=geometry.header.stamp,
            label='carton',
        )

        invalid = node.rich_plan_pub.messages[-1]
        self.assertFalse(invalid.valid)
        self.assertEqual(invalid.diagnostic, 'MODEL_RELOADED: model changed')
        self.assertEqual(invalid.header.stamp.to_nsec(), geometry.header.stamp.to_nsec())
        self.assertEqual(node.plan_pub.messages[-1].poses, [])
        self.assertIsNone(node._latest_rich_plan_copy())

    def test_invalidation_clears_authority_and_isolates_each_publisher_failure(self):
        class FailingPublisher:
            def publish(self, _message):
                raise RuntimeError('synthetic invalidation publish failure')

        for failing_attribute in ('geometry_pub', 'rich_plan_pub', 'plan_pub'):
            with self.subTest(publisher=failing_attribute):
                node = make_processing_node()
                geometry = make_geometry_message()
                node.latest_object_geometry = deepcopy(geometry)
                node.latest_rich_plan = remote_node.build_rich_plan(
                    make_selected_candidate(),
                    geometry,
                    deepcopy(geometry.header),
                    'carton_segment',
                )
                node.latest_plan = remote_node.rich_plan_to_legacy(
                    node.latest_rich_plan
                )
                node._latest_geometry_estimate = make_geometry_estimate()
                node._selected_candidate_gate = object()
                node.selected_required_open_width_m = 0.044
                setattr(node, failing_attribute, FailingPublisher())

                node._invalidate_geometry(
                    'TARGET_LOST',
                    'publisher failure probe',
                    stamp=geometry.header.stamp,
                    label='carton',
                )

                self.assertIsNone(node.latest_rich_plan)
                self.assertIsNone(node.latest_plan)
                self.assertIsNone(node._latest_geometry_estimate)
                self.assertIsNone(node._selected_candidate_gate)
                self.assertIsNone(node.selected_required_open_width_m)
                self.assertFalse(node.latest_object_geometry.valid)
                if failing_attribute != 'geometry_pub':
                    self.assertFalse(node.geometry_pub.messages[-1].valid)
                if failing_attribute != 'rich_plan_pub':
                    self.assertFalse(node.rich_plan_pub.messages[-1].valid)
                if failing_attribute != 'plan_pub':
                    self.assertEqual(node.plan_pub.messages[-1].poses, [])

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
        node.planning_snapshot_max_inference_latency_sec = 1.2
        node.planning_snapshot_max_span_sec = 3.0
        node.planning_mask_min_iou = 0.85
        node.planning_mask_max_centroid_shift_px = 5.0
        node.planning_max_joint_delta_rad = 0.01
        node.mask_erosion_px = 2
        node.mask_internal_hole_max_area_px = 25
        node.depth_mad_scale = 3.5
        node.depth_mad_absolute_floor_m = 0.002
        node._active_profile_requires_mask = lambda: True
        node._snapshot_depth_config = lambda: (0.0001, 0.03, 2.0)
        snapshot, failure_code, failure_reason = node._wait_for_stable_snapshot(
            True
        )

        self.assertEqual(failure_code, '')
        self.assertEqual(failure_reason, '')
        self.assertEqual(node.frames.discarded, [1_060_000_000])
        self.assertEqual(node.frames.collection_spans, [3.0, 3.0])
        self.assertEqual(node.frames.inference_latency_limits, [1.2, 1.2])
        self.assertTrue(snapshot.ok)
        self.assertEqual(snapshot.object_msg.snapshot_stamp, 1.16)

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
        node.planning_snapshot_max_inference_latency_sec = 1.2
        node.planning_snapshot_max_span_sec = 3.0
        node.planning_mask_min_iou = 0.85
        node.planning_mask_max_centroid_shift_px = 5.0
        node.planning_max_joint_delta_rad = 0.01
        node.mask_erosion_px = 2
        node.mask_internal_hole_max_area_px = 25
        node.depth_mad_scale = 3.5
        node.depth_mad_absolute_floor_m = 0.002
        node._active_profile_requires_mask = lambda: True
        node._snapshot_depth_config = lambda: (0.0001, 0.03, 2.0)

        _snapshot, failure_code, failure_reason = (
            node._wait_for_stable_snapshot(True)
        )

        self.assertEqual(failure_code, 'MASK_SIZE_MISMATCH')
        self.assertNotIn('MASK_SIZE_MISMATCH:', failure_reason)

    def test_stale_mask_timeout_is_published_with_stable_failure_code(self):
        node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
        node.enabled = True
        node.frames = SequenceSampleBuffer([], mask_timeout_code='MASK_STALE')
        node.status_pub = RecordingPublisher()
        node.plan_pub = RecordingPublisher()
        node.geometry_pub = RecordingPublisher()
        node.previous_object_axes_base = np.eye(3)
        node.planning_snapshot_frames = 3
        node.planning_snapshot_timeout_sec = 0.0
        node.planning_snapshot_max_age_sec = 0.35
        node.planning_snapshot_max_inference_latency_sec = 1.2
        node.planning_snapshot_max_span_sec = 3.0
        node._active_profile_requires_mask = lambda: True

        _snapshot, failure_code, failure_reason = (
            node._wait_for_stable_snapshot(True)
        )

        self.assertEqual(failure_code, 'MASK_STALE')
        self.assertTrue(failure_reason)

    def test_missing_and_empty_mask_timeouts_are_published_without_remapping(self):
        for expected_code in ('MASK_MISSING', 'MASK_EMPTY'):
            with self.subTest(expected_code=expected_code):
                node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
                node.enabled = True
                node.frames = SequenceSampleBuffer([], mask_timeout_code=expected_code)
                node.status_pub = RecordingPublisher()
                node.plan_pub = RecordingPublisher()
                node.geometry_pub = RecordingPublisher()
                node.previous_object_axes_base = np.eye(3)
                node.planning_snapshot_frames = 3
                node.planning_snapshot_timeout_sec = 0.0
                node.planning_snapshot_max_age_sec = 0.35
                node.planning_snapshot_max_inference_latency_sec = 1.2
                node.planning_snapshot_max_span_sec = 3.0
                node._active_profile_requires_mask = lambda: True

                _snapshot, failure_code, failure_reason = (
                    node._wait_for_stable_snapshot(True)
                )

                self.assertEqual(failure_code, expected_code)
                self.assertTrue(failure_reason)

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

    def test_derives_base_camera_link_from_nontrivial_frozen_optical_transform(self):
        base_from_camera = remote_node.transform_matrix(
            [0.37, -0.21, 0.48],
            remote_node.quaternion_from_euler(0.23, -0.31, 0.47),
        )
        camera_from_optical = np.eye(4, dtype=float)
        camera_from_optical[:3, :3] = remote_node.OPTICAL_TO_ROS_CAMERA
        base_from_optical = base_from_camera.dot(camera_from_optical)

        derived = remote_node.derive_base_camera_link_transform(
            base_from_optical
        )
        optical_point = np.asarray([0.13, -0.07, 0.62, 1.0])
        camera_point = camera_from_optical.dot(optical_point)

        np.testing.assert_allclose(derived, base_from_camera, atol=1e-9)
        np.testing.assert_allclose(
            derived.dot(camera_point),
            base_from_optical.dot(optical_point),
            atol=1e-9,
        )
        self.assertFalse(derived.flags.writeable)

    def test_frozen_snapshot_transform_drives_center_tool0_and_orientation(self):
        stamp = remote_node.rospy.Time(10, 123)
        base_from_camera = remote_node.transform_matrix(
            [0.41, -0.16, 0.29],
            remote_node.quaternion_from_euler(-0.17, 0.29, 0.38),
        )
        camera_from_optical = np.eye(4, dtype=float)
        camera_from_optical[:3, :3] = remote_node.OPTICAL_TO_ROS_CAMERA
        base_from_optical = base_from_camera.dot(camera_from_optical)
        raw_candidate = RemoteGraspCandidate(
            score=0.93,
            translation_m=np.asarray([0.08, -0.04, 0.56]),
            quaternion_xyzw=np.asarray(
                remote_node.quaternion_from_euler(0.11, -0.22, 0.33)
            ),
            width_m=0.04,
            depth_m=0.03,
        )
        camera_candidate = remote_node.convert_candidate_to_camera_link(
            raw_candidate,
            'opencv_optical',
        )
        camera_candidate = remote_node.align_candidate_to_tool_frame(
            camera_candidate,
            remote_node.STRICT_MODEL_GRASP_TO_TOOL_QUATERNION,
        )
        estimator = remote_node.FrozenSnapshotCandidatePoseEstimator(
            base_from_optical,
            stamp,
            'camera_link',
        )

        pose, center_base = remote_node.make_candidate_base_pose_and_center(
            camera_candidate,
            estimator,
            stamp,
            'camera_link',
        )

        expected_center = base_from_camera.dot(
            np.r_[camera_candidate.translation_m, 1.0]
        )[:3]
        expected_tool0 = base_from_camera.dot(
            np.r_[remote_node.candidate_tool0_translation(camera_candidate), 1.0]
        )[:3]
        expected_rotation = base_from_camera[:3, :3].dot(
            remote_node.quaternion_matrix(
                camera_candidate.quaternion_xyzw
            )[:3, :3]
        )
        np.testing.assert_allclose(center_base, expected_center, atol=1e-8)
        np.testing.assert_allclose(
            remote_node.pose_matrix(pose)[:3, 3],
            expected_tool0,
            atol=1e-8,
        )
        np.testing.assert_allclose(
            remote_node.pose_matrix(pose)[:3, :3],
            expected_rotation,
            atol=1e-8,
        )
        self.assertEqual(pose.header.stamp.to_nsec(), stamp.to_nsec())
        self.assertEqual(estimator.audit_metadata()['snapshot_source_frame'], 'camera_link')
        self.assertEqual(
            estimator.audit_metadata()['raw_candidate_convention'],
            'opencv_optical',
        )
        self.assertEqual(len(estimator.transform_sha256), 64)
        with self.assertRaises(ValueError):
            estimator.T_base_camera_link[0, 0] = 0.0
        selected, selected_pose = remote_node.select_first_reachable_candidate(
            [raw_candidate],
            estimator,
            lambda _pose: True,
            stamp=stamp,
            camera_frame='camera_link',
            candidate_frame_convention='opencv_optical',
            model_grasp_to_tool_quaternion=(
                remote_node.STRICT_MODEL_GRASP_TO_TOOL_QUATERNION
            ),
            grasp_config={'tool_approach_axis': 'z'},
            require_candidate_depth=True,
        )
        self.assertIsNotNone(selected)
        np.testing.assert_allclose(
            selected._center_base_xyz,
            expected_center,
            atol=1e-8,
        )
        np.testing.assert_allclose(
            remote_node.pose_matrix(selected_pose),
            remote_node.pose_matrix(pose),
            atol=1e-8,
        )
        invalid_calls = (
            {'stamp': remote_node.rospy.Time(11, 123), 'camera_frame': 'camera_link'},
            {'stamp': stamp, 'camera_frame': 'camera_color_optical_frame'},
        )
        for invalid in invalid_calls:
            with self.subTest(invalid=invalid):
                with self.assertRaises(remote_node.CandidateContractError) as raised:
                    estimator.make_base_pose_from_camera_pose(
                        remote_node.candidate_tool0_translation(camera_candidate),
                        camera_candidate.quaternion_xyzw,
                        **invalid
                    )
                self.assertEqual(
                    raised.exception.code,
                    'SNAPSHOT_TRANSFORM_INCONSISTENT',
                )

    def test_frozen_snapshot_transform_rejects_invalid_authority_inputs(self):
        valid_transform = identity_base_camera_snapshot_transform()
        valid_stamp = remote_node.rospy.Time(10, 123)
        non_rigid_transform = np.asarray(valid_transform).copy()
        non_rigid_transform[0, :3] *= 1.5
        cases = (
            ('missing_stamp', valid_transform, None, 'camera_link'),
            (
                'zero_stamp',
                valid_transform,
                remote_node.rospy.Time(0),
                'camera_link',
            ),
            ('negative_stamp', valid_transform, -1.0, 'camera_link'),
            ('empty_source_frame', valid_transform, valid_stamp, ''),
            ('non_rigid_matrix', non_rigid_transform, valid_stamp, 'camera_link'),
        )
        for name, transform, stamp, source_frame in cases:
            with self.subTest(name=name):
                with self.assertRaises(remote_node.CandidateContractError) as raised:
                    remote_node.FrozenSnapshotCandidatePoseEstimator(
                        transform,
                        stamp,
                        source_frame,
                    )
                self.assertEqual(
                    raised.exception.code,
                    'SNAPSHOT_TRANSFORM_INVALID',
                )

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

    def test_wsl_latency_cannot_switch_audit_or_selector_to_latest_tf(self):
        snapshot = make_snapshot(np.ones((3, 4), dtype=np.uint16) * 2200)
        stamp = remote_node.RemoteGrasp6DNode._snapshot_ros_stamp(snapshot)
        base_from_camera = remote_node.transform_matrix(
            [0.33, -0.19, 0.44],
            remote_node.quaternion_from_euler(0.18, -0.27, 0.36),
        )

        class ExactSnapshotBuffer:
            def __init__(self):
                self.inference_finished = False
                self.calls = []

            def lookup_transform(self, target, source, query_stamp, timeout):
                del timeout
                self.calls.append(
                    (target, source, int(query_stamp.to_nsec()))
                )
                if self.inference_finished:
                    raise AssertionError(
                        'candidate base conversion queried TF after WSL inference'
                    )
                quaternion = remote_node._normalize_quaternion(
                    remote_node.quaternion_from_matrix(base_from_camera)
                )
                return types.SimpleNamespace(
                    transform=types.SimpleNamespace(
                        translation=types.SimpleNamespace(
                            x=float(base_from_camera[0, 3]),
                            y=float(base_from_camera[1, 3]),
                            z=float(base_from_camera[2, 3]),
                        ),
                        rotation=types.SimpleNamespace(
                            x=float(quaternion[0]),
                            y=float(quaternion[1]),
                            z=float(quaternion[2]),
                            w=float(quaternion[3]),
                        ),
                    )
                )

        buffer = ExactSnapshotBuffer()

        class InferenceClient(RecordingClient):
            def predict(self, *args, **kwargs):
                result = super().predict(*args, **kwargs)
                buffer.inference_finished = True
                return result

        candidate = RemoteGraspCandidate(
            score=0.91,
            translation_m=np.asarray([0.40, -0.10, 0.25]),
            quaternion_xyzw=np.asarray([0.0, 0.0, 0.0, 1.0]),
            width_m=0.04,
            depth_m=0.03,
        )
        client = InferenceClient(candidates=[candidate])
        node = make_processing_node(client)
        node.tf_buffer = buffer
        node.pose_estimator.tf_buffer = buffer
        node.pose_estimator.tf_timeout_sec = 0.2
        node.gate_audit_enabled = True
        node.camera_visibility_gate_enabled = True
        audit_estimators = []
        selector_estimators = []
        selected_centers = []
        original_audit = node._run_candidate_gate_audit
        original_selector = remote_node.select_first_reachable_candidate
        original_geometry_estimator = remote_node.estimate_object_geometry

        def audit_with_identity(*args, **kwargs):
            audit_estimators.append(kwargs['candidate_pose_estimator'])
            return original_audit(*args, **kwargs)

        def selector_without_moveit(
            candidates,
            pose_estimator,
            _reachability_fn,
            **kwargs
        ):
            selector_estimators.append(pose_estimator)
            aligned = remote_node.align_candidate_to_tool_frame(
                remote_node.convert_candidate_to_camera_link(
                    candidates[0],
                    kwargs['candidate_frame_convention'],
                ),
                kwargs['model_grasp_to_tool_quaternion'],
            )
            _pose, center = remote_node.make_candidate_base_pose_and_center(
                aligned,
                pose_estimator,
                kwargs['stamp'],
                kwargs['camera_frame'],
            )
            selected_centers.append(center)
            return None, None

        node._run_candidate_gate_audit = audit_with_identity
        remote_node.select_first_reachable_candidate = selector_without_moveit
        remote_node.estimate_object_geometry = lambda **_kwargs: make_geometry_estimate()
        try:
            ok, message = node._process_frame(snapshot, manual=True)
        finally:
            remote_node.select_first_reachable_candidate = original_selector
            remote_node.estimate_object_geometry = original_geometry_estimator

        self.assertFalse(ok)
        self.assertTrue(message.startswith('NO_GEOMETRIC_CANDIDATE: '), message)
        self.assertEqual(
            buffer.calls,
            [
                ('base_link', 'camera_link', int(stamp.to_nsec())),
                ('tool0', 'camera_link', int(stamp.to_nsec())),
            ],
        )
        self.assertEqual(len(audit_estimators), 1)
        self.assertEqual(len(selector_estimators), 1)
        self.assertIs(audit_estimators[0], selector_estimators[0])
        expected_center = base_from_camera.dot(np.r_[candidate.translation_m, 1.0])[:3]
        np.testing.assert_allclose(selected_centers[0], expected_center, atol=1e-7)
        metadata = selector_estimators[0].audit_metadata()
        self.assertEqual(metadata['snapshot_stamp_ns'], snapshot.stamp_ns)
        self.assertEqual(metadata['snapshot_source_frame'], 'camera_link')
        self.assertEqual(metadata['canonical_candidate_frame'], 'camera_link')
        self.assertEqual(len(metadata['transform_sha256']), 64)

    def test_visibility_transform_accessor_never_falls_back_to_live_latest_tf(self):
        class ForbiddenBuffer:
            def __init__(self):
                self.calls = []

            def lookup_transform(self, *args, **kwargs):
                self.calls.append((args, kwargs))
                raise AssertionError('live/latest TF lookup is forbidden')

        node = make_processing_node()
        node.tf_buffer = ForbiddenBuffer()
        node._planning_snapshot_active = True
        node._cached_tool_from_camera = None

        with self.assertRaises(remote_node.CandidateContractError) as raised:
            node._tool_from_camera_matrix()

        self.assertEqual(
            raised.exception.code,
            'VISIBILITY_TRANSFORM_NOT_FROZEN',
        )
        self.assertEqual(node.tf_buffer.calls, [])

    def test_visibility_calibration_fallback_is_finite_rigid_and_read_only(self):
        node = make_processing_node()
        node.tf_buffer = None
        node.handeye_translation_xyz = [0.01, -0.02, 0.03]
        node.handeye_rotation_xyzw = remote_node.quaternion_from_euler(
            0.1,
            -0.2,
            0.3,
        )
        stamp = remote_node.rospy.Time(10, 123)

        matrix = node._freeze_tool_from_camera_matrix(stamp)

        self.assertFalse(matrix.flags.writeable)
        np.testing.assert_allclose(matrix[3], [0.0, 0.0, 0.0, 1.0])
        np.testing.assert_allclose(
            matrix[:3, :3].T.dot(matrix[:3, :3]),
            np.eye(3),
            atol=1e-8,
        )
        self.assertAlmostEqual(np.linalg.det(matrix[:3, :3]), 1.0)
        self.assertEqual(node._cached_tool_from_camera_stamp_ns, stamp.to_nsec())

        node._cached_tool_from_camera = None
        node.handeye_translation_xyz = [np.nan, 0.0, 0.0]
        with self.assertRaises(remote_node.CandidateContractError) as raised:
            node._freeze_tool_from_camera_matrix(stamp)
        self.assertEqual(raised.exception.code, 'VISIBILITY_TRANSFORM_INVALID')

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

    def test_masked_target_default_payload_is_byte_exact_and_audited(self):
        target_depth = np.asarray(
            [
                [0, 0, 0, 0],
                [0, 2200, 2200, 0],
                [0, 2200, 2200, 0],
            ],
            dtype=np.uint16,
        )
        snapshot = make_snapshot(target_depth)
        snapshot.color_bgr.setflags(write=True)
        snapshot.color_bgr[:] = np.arange(36, dtype=np.uint8).reshape(3, 4, 3)
        snapshot.color_bgr.setflags(write=False)
        client = RecordingClient(candidates=[])
        node = make_processing_node(client)
        self.assertEqual(node._freeze_graspnet_input_config().mode, 'masked_target')
        original_estimator = remote_node.estimate_object_geometry
        remote_node.estimate_object_geometry = lambda **_kwargs: make_geometry_estimate()
        node._snapshot_base_optical_transform = lambda *_args: np.eye(4)
        try:
            ok, message = node._process_frame(snapshot, manual=True)
        finally:
            remote_node.estimate_object_geometry = original_estimator

        self.assertFalse(ok)
        self.assertTrue(message.startswith('NO_RAW_CANDIDATE: '))
        expected_color = np.zeros_like(snapshot.color_bgr)
        expected_color[target_depth > 0] = snapshot.color_bgr[target_depth > 0]
        self.assertEqual(client.calls[0]['depth'].tobytes(), target_depth.tobytes())
        self.assertEqual(client.calls[0]['color'].tobytes(), expected_color.tobytes())
        audit = node._active_graspnet_input_audit
        self.assertEqual(audit['mode'], 'masked_target')
        self.assertEqual(
            audit['depth_sha256'],
            hashlib.sha256(target_depth.tobytes()).hexdigest(),
        )
        self.assertEqual(
            audit['mask_sha256'],
            hashlib.sha256(np.asarray(snapshot.object_mask).tobytes()).hexdigest(),
        )

    def test_context_roi_adds_only_same_snapshot_support_without_polluting_geometry(self):
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
        node.candidate_target_gate_enabled = True
        captured_geometry = {}
        original_estimator = remote_node.estimate_object_geometry

        def fake_estimator(**kwargs):
            captured_geometry.update(kwargs)
            return make_geometry_estimate()

        remote_node.estimate_object_geometry = fake_estimator
        node._snapshot_base_optical_transform = lambda *_args: np.eye(4)
        original_activate = node._activate_geometry

        def activate(*args, **kwargs):
            activated = original_activate(*args, **kwargs)
            if activated:
                node.latest_support_plane_camera_point = np.asarray([0.24, 0.0, 0.0])
                node.latest_support_plane_camera_normal = np.asarray([1.0, 0.0, 0.0])
            return activated

        node._activate_geometry = activate
        config = frozen_input_config(
            node,
            remote_node.CONTEXT_ROI,
            context_margin_px=1.0,
            context_max_margin_px=2.0,
            min_support_points=1,
        )
        try:
            ok, message = node._process_frame(
                snapshot,
                manual=True,
                graspnet_input_config=config,
            )
        finally:
            remote_node.estimate_object_geometry = original_estimator

        self.assertFalse(ok)
        self.assertTrue(message.startswith('NO_RAW_CANDIDATE: '))
        np.testing.assert_array_equal(
            captured_geometry['target_depth_raw'],
            target_depth,
        )
        remote_depth = client.calls[0]['depth']
        np.testing.assert_array_equal(remote_depth[target_depth > 0], target_depth[target_depth > 0])
        self.assertGreater(
            np.count_nonzero((remote_depth > 0) & (target_depth == 0)),
            0,
        )
        self.assertEqual(node._active_graspnet_input_audit['mode'], 'context_roi')
        self.assertGreater(node._active_graspnet_input_audit['support_points'], 0)
        self.assertEqual(node._active_graspnet_input_audit['target_points'], 4)
        self.assertEqual(
            node._active_graspnet_input_audit['detected_bbox_mask_iou'],
            1.0,
        )

    def test_context_roi_prerequisites_and_builder_errors_fail_before_wsl(self):
        target_depth = np.asarray(
            [[0, 0, 0, 0], [0, 2200, 2200, 0], [0, 2200, 2200, 0]],
            dtype=np.uint16,
        )
        original_estimator = remote_node.estimate_object_geometry
        remote_node.estimate_object_geometry = lambda **_kwargs: make_geometry_estimate()
        try:
            cases = (
                ('bbox_mask', 'INSTANCE_MASK_REQUIRED'),
                ('target_gate', 'CANDIDATE_TARGET_GATE_REQUIRED'),
                ('plane_frame', 'SUPPORT_PLANE_FRAME_INVALID'),
                ('bad_config', 'CONFIG_INVALID'),
                ('zero_quality_gate', 'CONFIG_INVALID'),
                ('support_points', 'SUPPORT_POINTS_INSUFFICIENT'),
            )
            for case, expected_code in cases:
                with self.subTest(case=case):
                    source_mode = 'bbox_depth' if case == 'bbox_mask' else 'instance_mask'
                    snapshot = make_snapshot(target_depth, source_mode=source_mode)
                    client = RecordingClient(candidates=[])
                    node = make_processing_node(client)
                    node._snapshot_base_optical_transform = lambda *_args: np.eye(4)
                    original_activate = node._activate_geometry

                    def activate(*args, **kwargs):
                        activated = original_activate(*args, **kwargs)
                        if activated:
                            point = 0.50 if case == 'support_points' else 0.24
                            node.latest_support_plane_camera_point = np.asarray([point, 0.0, 0.0])
                            node.latest_support_plane_camera_normal = np.asarray([1.0, 0.0, 0.0])
                        return activated

                    node._activate_geometry = activate
                    # context_roi requires this production safety gate.  Only
                    # the dedicated target_gate case intentionally disables it;
                    # the generic offline fixture defaults to false so every
                    # other case must opt into the prerequisite explicitly.
                    node.candidate_target_gate_enabled = case != 'target_gate'
                    if case == 'plane_frame':
                        node.target_projection_frame_convention = 'opencv_optical'
                    config = frozen_input_config(
                        node,
                        remote_node.CONTEXT_ROI,
                        context_margin_px=(3.0 if case == 'bad_config' else 1.0),
                        context_max_margin_px=2.0,
                        min_support_points=(0 if case == 'zero_quality_gate' else 1),
                    )

                    ok, message = node._process_frame(
                        snapshot,
                        manual=True,
                        graspnet_input_config=config,
                    )

                    self.assertFalse(ok)
                    self.assertTrue(message.startswith(expected_code + ': '), message)
                    self.assertEqual(client.calls, [])
                    self.assertFalse(node.rich_plan_pub.messages[-1].valid)
        finally:
            remote_node.estimate_object_geometry = original_estimator

    def test_context_modes_require_instance_mask_before_runtime_geometry_tf_or_wsl(self):
        snapshot = make_snapshot(
            np.ones((3, 4), dtype=np.uint16) * 2200,
            source_mode='bbox_depth',
        )
        for mode in (remote_node.CONTEXT_ROI, remote_node.FULL_SCENE):
            with self.subTest(mode=mode):
                client = RecordingClient(candidates=[])
                node = make_processing_node(client)
                node.candidate_target_gate_enabled = True
                calls = []
                node._refresh_runtime_params = lambda: calls.append('runtime')
                node._prepare_snapshot_geometry = (
                    lambda *_args, **_kwargs: calls.append('geometry')
                )
                node._snapshot_base_optical_transform = (
                    lambda *_args, **_kwargs: calls.append('tf')
                )

                ok, message = node._process_frame(
                    snapshot,
                    manual=True,
                    graspnet_input_config=frozen_input_config(node, mode),
                )

                self.assertFalse(ok)
                self.assertTrue(message.startswith('INSTANCE_MASK_REQUIRED: '))
                self.assertEqual(calls, [])
                self.assertEqual(client.calls, [])

    def test_full_scene_is_always_diagnostic_and_never_enters_selector(self):
        target_depth = np.asarray(
            [[0, 0, 0, 0], [0, 2200, 2200, 0], [0, 2200, 2200, 0]],
            dtype=np.uint16,
        )
        snapshot = make_snapshot(target_depth)
        candidate = RemoteGraspCandidate(
            score=0.99,
            translation_m=np.asarray([0.40, -0.10, 0.25]),
            quaternion_xyzw=np.asarray([0.0, 0.0, 0.0, 1.0]),
            width_m=0.04,
            depth_m=0.03,
        )
        client = RecordingClient(candidates=[candidate])
        node = make_processing_node(client)
        node.pose_estimator = types.SimpleNamespace(
            camera_frame='camera_link',
            make_base_pose_from_camera_pose=lambda *_args, **_kwargs: (
                (_ for _ in ()).throw(
                    AssertionError('full_scene audit must use frozen snapshot TF')
                )
            ),
        )
        node.candidate_target_gate_enabled = False
        node._plan_reachable = lambda _pose: (_ for _ in ()).throw(
            AssertionError('reachability/MoveIt must not run in full_scene')
        )
        original_estimator = remote_node.estimate_object_geometry
        original_selector = remote_node.select_first_reachable_candidate
        remote_node.estimate_object_geometry = lambda **_kwargs: make_geometry_estimate()
        remote_node.select_first_reachable_candidate = lambda *_args, **_kwargs: (
            (_ for _ in ()).throw(
                AssertionError('selector must not run in full_scene')
            )
        )
        node._snapshot_base_optical_transform = (
            lambda *_args: identity_base_camera_snapshot_transform()
        )
        original_activate = node._activate_geometry

        def activate_without_plane(*args, **kwargs):
            activated = original_activate(*args, **kwargs)
            node.latest_support_plane_camera_point = None
            node.latest_support_plane_camera_normal = None
            return activated

        node._activate_geometry = activate_without_plane
        captured_reports = []
        original_write_audit = node._write_gate_audit_report

        def capture_audit(report):
            captured_reports.append(deepcopy(report))
            return original_write_audit(report)

        node._write_gate_audit_report = capture_audit
        config = frozen_input_config(
            node,
            remote_node.FULL_SCENE,
            candidate_target_gate_enabled=False,
        )
        try:
            ok, message = node._process_frame(
                snapshot,
                manual=True,
                graspnet_input_config=config,
            )
        finally:
            remote_node.estimate_object_geometry = original_estimator
            remote_node.select_first_reachable_candidate = original_selector

        self.assertFalse(ok)
        self.assertTrue(
            message.startswith(remote_node.FULL_SCENE_DIAGNOSTIC_CODE + ': '),
            message,
        )
        self.assertEqual(len(client.calls), 1)
        np.testing.assert_array_equal(client.calls[0]['depth'], snapshot.depth_raw)
        self.assertTrue(all(len(message.poses) == 0 for message in node.plan_pub.messages))
        self.assertTrue(all(not message.valid for message in node.rich_plan_pub.messages))
        self.assertEqual(
            node._latest_gate_audit_summary['graspnet_input']['mode'],
            'full_scene',
        )
        self.assertEqual(node._latest_gate_audit_reference['row_count'], 1)
        transform_summary = node._latest_gate_audit_summary['snapshot_transform']
        self.assertEqual(transform_summary['snapshot_stamp_ns'], snapshot.stamp_ns)
        self.assertEqual(transform_summary['snapshot_source_frame'], 'camera_link')
        self.assertEqual(transform_summary['canonical_candidate_frame'], 'camera_link')
        self.assertEqual(len(transform_summary['transform_sha256']), 64)
        self.assertEqual(len(captured_reports), 1)
        transform_report = captured_reports[0]['snapshot_transform']
        self.assertEqual(np.asarray(transform_report['T_base_optical']).shape, (4, 4))
        self.assertEqual(
            np.asarray(transform_report['T_base_camera_link']).shape,
            (4, 4),
        )

    def test_graspnet_input_mode_and_parameters_are_frozen_for_request(self):
        target_depth = np.asarray(
            [[0, 0, 0, 0], [0, 2200, 2200, 0], [0, 2200, 2200, 0]],
            dtype=np.uint16,
        )
        snapshot = make_snapshot(target_depth)
        client = RecordingClient(candidates=[])
        node = make_processing_node(client)
        config = frozen_input_config(node, remote_node.MASKED_TARGET)

        def mutate_runtime_values():
            node.graspnet_input_mode = remote_node.FULL_SCENE
            node.graspnet_input_context_margin_px = 99.0
            node.graspnet_input_min_total_points = 999999
            node.candidate_target_gate_enabled = False

        node._refresh_runtime_params = mutate_runtime_values
        original_estimator = remote_node.estimate_object_geometry
        remote_node.estimate_object_geometry = lambda **_kwargs: make_geometry_estimate()
        node._snapshot_base_optical_transform = lambda *_args: np.eye(4)
        try:
            ok, message = node._process_frame(
                snapshot,
                manual=True,
                graspnet_input_config=config,
            )
        finally:
            remote_node.estimate_object_geometry = original_estimator

        self.assertFalse(ok)
        self.assertTrue(message.startswith('NO_RAW_CANDIDATE: '), message)
        self.assertEqual(node._active_graspnet_input_audit['mode'], 'masked_target')
        self.assertEqual(
            node._active_graspnet_input_audit['frozen_config']['context_margin_px'],
            1.0,
        )
        np.testing.assert_array_equal(client.calls[0]['depth'], target_depth)

    def test_only_known_contract_exceptions_keep_their_stable_code(self):
        snapshot = make_snapshot(np.ones((3, 4), dtype=np.uint16) * 2200)

        class CodedOrdinaryError(RuntimeError):
            code = 'MUST_NOT_ESCAPE'

        cases = (
            (
                lambda: remote_node.validate_execution_tool0_contract(
                    False,
                    remote_node.STRICT_MODEL_GRASP_TO_TOOL_QUATERNION,
                    {'tool_approach_axis': 'z'},
                ),
                'DEPTH_CONTRACT_DISABLED',
            ),
            (
                lambda: remote_node.validate_execution_tool0_contract(
                    True,
                    [0.0, 0.0, 0.0, 1.0],
                    {'tool_approach_axis': 'z'},
                ),
                'TOOL_FRAME_CONTRACT_INVALID',
            ),
            (
                lambda: remote_node.validate_execution_tool0_contract(
                    True,
                    remote_node.STRICT_MODEL_GRASP_TO_TOOL_QUATERNION,
                    {'tool_approach_axis': 'x'},
                ),
                'TOOL_FRAME_CONTRACT_INVALID',
            ),
            (lambda: (_ for _ in ()).throw(CodedOrdinaryError('boom')), 'PLAN_FAILED'),
        )
        for action, expected_code in cases:
            with self.subTest(expected_code=expected_code):
                node = make_processing_node(RecordingClient(candidates=[]))
                node._refresh_runtime_params = action

                ok, message = node._process_frame(snapshot, manual=True)

                self.assertFalse(ok)
                self.assertTrue(message.startswith(expected_code + ': '), message)
                self.assertEqual(node.client.calls, [])

    def test_production_candidate_convention_rejects_startup_misconfiguration(self):
        self.assertEqual(
            remote_node.validate_production_candidate_frame_convention('opencv'),
            'opencv_optical',
        )
        for value in ('ros_camera_link', 'camera_link', 'unknown'):
            with self.subTest(value=value):
                with self.assertRaises(remote_node.CandidateContractError) as raised:
                    remote_node.validate_production_candidate_frame_convention(
                        value
                    )
                self.assertEqual(
                    raised.exception.code,
                    'CANDIDATE_FRAME_CONVENTION_INVALID',
                )

    def test_mandatory_planning_audit_rejects_non_boolean_or_missing_authority(self):
        self.assertEqual(
            remote_node.validate_mandatory_planning_audit(
                True,
                '/tmp/grasp6d-audit.json',
            ),
            '/tmp/grasp6d-audit.json',
        )
        invalid = (
            (False, '/tmp/grasp6d-audit.json'),
            ('false', '/tmp/grasp6d-audit.json'),
            (1, '/tmp/grasp6d-audit.json'),
            (np.bool_(True), '/tmp/grasp6d-audit.json'),
            (True, ''),
            (True, '   '),
            (True, ['/tmp/grasp6d-audit.json']),
            (True, None),
        )
        for enabled, path in invalid:
            with self.subTest(enabled=enabled, path=path):
                with self.assertRaises(
                    remote_node.CandidateContractError
                ) as raised:
                    remote_node.validate_mandatory_planning_audit(
                        enabled,
                        path,
                    )
                self.assertEqual(
                    raised.exception.code,
                    remote_node.PLANNING_AUDIT_CONFIG_INVALID,
                )

    def test_planning_and_mujoco_audit_paths_are_canonical_and_distinct(self):
        planning, mujoco = remote_node.validate_distinct_audit_paths(
            '/tmp/alicia-audits/../planning.json',
            '/tmp/alicia-audits/execution.json',
        )
        self.assertEqual(planning, '/tmp/planning.json')
        self.assertEqual(mujoco, '/tmp/alicia-audits/execution.json')

        equivalent_pairs = (
            ('/tmp/alicia-audit.json', '/tmp/./alicia-audit.json'),
            (
                '/tmp/alicia-audits/../alicia-audit.json',
                '/tmp/alicia-audit.json',
            ),
        )
        for planning_path, mujoco_path in equivalent_pairs:
            with self.subTest(
                planning_path=planning_path,
                mujoco_path=mujoco_path,
            ):
                with self.assertRaises(
                    remote_node.CandidateContractError
                ) as raised:
                    remote_node.validate_distinct_audit_paths(
                        planning_path,
                        mujoco_path,
                    )
                self.assertEqual(
                    raised.exception.code,
                    remote_node.AUDIT_PATH_CONFLICT,
                )

        for invalid_mujoco_path in ('', '   ', None, []):
            with self.subTest(invalid_mujoco_path=invalid_mujoco_path):
                with self.assertRaises(
                    remote_node.CandidateContractError
                ) as raised:
                    remote_node.validate_distinct_audit_paths(
                        '/tmp/planning.json',
                        invalid_mujoco_path,
                    )
                self.assertEqual(
                    raised.exception.code,
                    remote_node.PLANNING_AUDIT_CONFIG_INVALID,
                )

    def test_audit_path_conflict_fails_before_geometry_tf_or_wsl(self):
        snapshot = make_snapshot(np.ones((3, 4), dtype=np.uint16) * 2200)
        client = RecordingClient(candidates=[])
        node = make_processing_node(client)
        node.mujoco_audit_output_path = str(node.gate_audit_output_path)
        node._prepare_snapshot_geometry = lambda *_args: (
            (_ for _ in ()).throw(
                AssertionError('audit path conflict must fail before geometry/TF')
            )
        )

        ok, message = node._process_frame(snapshot, manual=True)

        self.assertFalse(ok)
        self.assertTrue(
            message.startswith(remote_node.AUDIT_PATH_CONFLICT + ': '),
            message,
        )
        self.assertEqual(client.calls, [])

    def test_startup_audit_path_conflict_fails_before_live_node_surfaces(self):
        shared = '/tmp/alicia-startup-shared-audit.json'
        original_get_param = remote_node.rospy.get_param
        original_bridge = remote_node.CvBridge
        remote_node.rospy.get_param = lambda name, default=None: {
            '/grasp_6d/enabled': True,
            '/grasp_6d/remote': {
                'gate_audit_enabled': True,
                'gate_audit_output_path': shared,
            },
            '/mujoco_digital_twin': {'audit_output_path': shared},
        }.get(name, default)
        remote_node.CvBridge = lambda: (_ for _ in ()).throw(
            AssertionError('CvBridge must not be created after path conflict')
        )
        try:
            with self.assertRaises(
                remote_node.CandidateContractError
            ) as raised:
                remote_node.RemoteGrasp6DNode()
        finally:
            remote_node.rospy.get_param = original_get_param
            remote_node.CvBridge = original_bridge

        self.assertEqual(raised.exception.code, remote_node.AUDIT_PATH_CONFLICT)

    def test_startup_continuous_config_fails_before_live_node_surfaces(self):
        original_get_param = remote_node.rospy.get_param
        original_bridge = remote_node.CvBridge
        remote_node.rospy.get_param = lambda name, default=None: {
            '/grasp_6d/remote': {'request_hz': True},
        }.get(name, default)
        remote_node.CvBridge = lambda: (_ for _ in ()).throw(
            AssertionError('CvBridge must not be created after config failure')
        )
        try:
            with self.assertRaises(
                remote_node.CandidateContractError
            ) as raised:
                remote_node.RemoteGrasp6DNode()
        finally:
            remote_node.rospy.get_param = original_get_param
            remote_node.CvBridge = original_bridge

        self.assertEqual(
            raised.exception.code,
            remote_node.CONTINUOUS_CONFIG_INVALID,
        )
        self.assertIn('request_hz', str(raised.exception))

    def test_startup_continuous_config_rejects_nonmapping_remote_namespace(self):
        original_get_param = remote_node.rospy.get_param
        original_bridge = remote_node.CvBridge
        remote_node.rospy.get_param = lambda name, default=None: {
            '/grasp_6d/remote': [],
        }.get(name, default)
        remote_node.CvBridge = lambda: (_ for _ in ()).throw(
            AssertionError('CvBridge must not be created after config failure')
        )
        try:
            with self.assertRaises(
                remote_node.CandidateContractError
            ) as raised:
                remote_node.RemoteGrasp6DNode()
        finally:
            remote_node.rospy.get_param = original_get_param
            remote_node.CvBridge = original_bridge

        self.assertEqual(
            raised.exception.code,
            remote_node.CONTINUOUS_CONFIG_INVALID,
        )
        self.assertIn('mapping', str(raised.exception))

    def test_runtime_continuous_config_failure_has_no_partial_update(self):
        node = make_processing_node()
        node._initialize_streaming_state(
            result_max_age_sec=1.2,
            performance_window_size=100,
            tracking_config=remote_node.TrackingConfig(),
            start_worker=False,
        )
        node.rate_hz = 1.5
        node.target_instance_association_threshold_m = 0.08
        node.target_absolute_sanity_distance_m = 0.15
        node.moveit_top_n = 5
        node.candidate_frame_convention = 'opencv_optical'
        node.execution_plan_controller = remote_node.ExecutionPlanController()
        node.execution_plan_controller.commit_execution(
            'plan-A', 'target-A', score=1.0, now_sec=5.0
        )
        node._refresh_runtime_params = types.MethodType(
            remote_node.RemoteGrasp6DNode._refresh_runtime_params,
            node,
        )
        before_tracker = node.tracker
        before_controller = node.execution_plan_controller
        before_metrics = node.pipeline_metrics
        original_get_param = remote_node.rospy.get_param
        remote_node.rospy.get_param = lambda name, default=None: {
            '/grasp_6d/remote': {
                'request_hz': 2.0,
                'tracking_position_threshold_m': float('nan'),
            },
        }.get(name, default)
        try:
            with self.assertRaises(
                remote_node.CandidateContractError
            ) as raised:
                node._refresh_runtime_params()
        finally:
            remote_node.rospy.get_param = original_get_param
            node.shutdown_streaming_worker()

        self.assertEqual(
            raised.exception.code,
            remote_node.CONTINUOUS_CONFIG_INVALID,
        )
        self.assertEqual(node.rate_hz, 1.5)
        self.assertIs(node.tracker, before_tracker)
        self.assertIs(node.execution_plan_controller, before_controller)
        self.assertEqual(before_controller.execution_plan_id, 'plan-A')
        self.assertIs(node.pipeline_metrics, before_metrics)

    def test_later_runtime_config_failure_does_not_apply_continuous_values(self):
        node = make_processing_node()
        node._initialize_streaming_state(
            result_max_age_sec=1.2,
            performance_window_size=100,
            tracking_config=remote_node.TrackingConfig(),
            start_worker=False,
        )
        node.rate_hz = 1.5
        node.target_instance_association_threshold_m = 0.08
        node.target_absolute_sanity_distance_m = 0.15
        node.moveit_top_n = 5
        node.candidate_frame_convention = 'opencv_optical'
        node.execution_plan_controller = remote_node.ExecutionPlanController()
        node._refresh_runtime_params = types.MethodType(
            remote_node.RemoteGrasp6DNode._refresh_runtime_params,
            node,
        )
        controller = node.execution_plan_controller
        original_get_param = remote_node.rospy.get_param
        remote_node.rospy.get_param = lambda name, default=None: {
            '/grasp_6d/remote': {
                'request_hz': 2.0,
                'replan_cooldown_sec': 2.0,
                'geometry_min_size_m': 'not-a-number',
            },
        }.get(name, default)
        try:
            with self.assertRaises(ValueError):
                node._refresh_runtime_params()
        finally:
            remote_node.rospy.get_param = original_get_param
            node.shutdown_streaming_worker()

        self.assertEqual(node.rate_hz, 1.5)
        self.assertIs(node.execution_plan_controller, controller)
        self.assertEqual(controller.replan_cooldown_sec, 1.0)

    def test_runtime_audit_path_refresh_rejects_conflict_without_partial_update(self):
        node = make_processing_node()
        node._refresh_runtime_params = types.MethodType(
            remote_node.RemoteGrasp6DNode._refresh_runtime_params,
            node,
        )
        original_planning = node.gate_audit_output_path
        original_mujoco = node.mujoco_audit_output_path
        shared = '/tmp/alicia-runtime-shared-audit.json'
        original_get_param = remote_node.rospy.get_param
        remote_node.rospy.get_param = lambda name, default=None: {
            '/grasp_6d/remote': {
                'gate_audit_enabled': True,
                'gate_audit_output_path': shared,
            },
            '/mujoco_digital_twin': {'audit_output_path': shared},
            '/grasp_6d/remote/gate_audit_enabled': True,
            '/grasp_6d/remote/gate_audit_output_path': shared,
            '/mujoco_digital_twin/audit_output_path': shared,
        }.get(name, default)
        try:
            with self.assertRaises(
                remote_node.CandidateContractError
            ) as raised:
                node._refresh_runtime_params()
        finally:
            remote_node.rospy.get_param = original_get_param

        self.assertEqual(raised.exception.code, remote_node.AUDIT_PATH_CONFLICT)
        self.assertEqual(node.gate_audit_output_path, original_planning)
        self.assertEqual(node.mujoco_audit_output_path, original_mujoco)

    def test_runtime_planning_audit_relaxation_fails_before_geometry_tf_or_wsl(self):
        snapshot = make_snapshot(np.ones((3, 4), dtype=np.uint16) * 2200)
        for enabled, path in (
            (False, '/tmp/grasp6d-audit.json'),
            ('false', '/tmp/grasp6d-audit.json'),
            (True, ''),
            (True, []),
        ):
            with self.subTest(enabled=enabled, path=path):
                client = RecordingClient(candidates=[])
                node = make_processing_node(client)
                node.gate_audit_enabled = enabled
                node.gate_audit_output_path = path
                node._prepare_snapshot_geometry = lambda *_args: (
                    (_ for _ in ()).throw(
                        AssertionError('audit config must fail before geometry/TF')
                    )
                )

                ok, message = node._process_frame(snapshot, manual=True)

                self.assertFalse(ok)
                self.assertTrue(
                    message.startswith(
                        remote_node.PLANNING_AUDIT_CONFIG_INVALID + ': '
                    ),
                    message,
                )
                self.assertEqual(client.calls, [])

    def test_production_fallback_contract_rejects_startup_relaxation(self):
        self.assertEqual(
            remote_node.validate_production_execution_fallback_contract(
                False,
                False,
            ),
            (False, False),
        )
        for position_only, orientation, expected_code in (
            (True, False, 'POSITION_ONLY_FALLBACK_FORBIDDEN'),
            (False, True, 'ORIENTATION_FALLBACK_FORBIDDEN'),
            (True, True, 'POSITION_ONLY_FALLBACK_FORBIDDEN'),
        ):
            with self.subTest(
                position_only=position_only,
                orientation=orientation,
            ):
                with self.assertRaises(
                    remote_node.CandidateContractError
                ) as raised:
                    remote_node.validate_production_execution_fallback_contract(
                        position_only,
                        orientation,
                    )
                self.assertEqual(raised.exception.code, expected_code)

    def test_production_orientation_variants_are_exact_ordered_unique_symmetries(self):
        identity = np.asarray([0.0, 0.0, 0.0, -1.0])
        half_turn = -np.asarray(
            remote_node.quaternion_from_euler(0.0, 0.0, np.pi),
            dtype=float,
        )
        accepted = remote_node.validate_production_orientation_variant_quaternions(
            [identity, half_turn]
        )

        self.assertEqual(len(accepted), 2)
        self.assertTrue(all(not value.flags.writeable for value in accepted))
        np.testing.assert_allclose(
            remote_node.quaternion_matrix(accepted[0])[:3, :3],
            np.eye(3),
            atol=1e-7,
        )
        np.testing.assert_allclose(
            remote_node.quaternion_matrix(accepted[1])[:3, :3],
            remote_node.TOOL_Z_HALF_TURN_ROTATION,
            atol=1e-7,
        )

        rz90 = remote_node.quaternion_from_euler(0.0, 0.0, np.pi * 0.5)
        canonical_identity = np.asarray([0.0, 0.0, 0.0, 1.0])
        canonical_half_turn = remote_node.quaternion_from_euler(
            0.0,
            0.0,
            np.pi,
        )
        invalid = (
            [canonical_identity],
            [canonical_identity, canonical_half_turn, canonical_identity],
            [canonical_identity, canonical_identity],
            [canonical_half_turn, canonical_identity],
            [canonical_identity, rz90],
        )
        for variants in invalid:
            with self.subTest(count=len(variants)):
                with self.assertRaises(
                    remote_node.CandidateContractError
                ) as raised:
                    remote_node.validate_production_orientation_variant_quaternions(
                        variants
                    )
                self.assertEqual(
                    raised.exception.code,
                    'ORIENTATION_VARIANT_CONTRACT_INVALID',
                )

        with self.assertRaises(remote_node.CandidateContractError) as raised:
            remote_node.RemoteGrasp6DNode._parse_production_orientation_variant_quaternions(
                [[0.0, 0.0, 0.0], [0.0, 180.0]]
            )
        self.assertEqual(
            raised.exception.code,
            'ORIENTATION_VARIANT_CONTRACT_INVALID',
        )

    def test_runtime_strict_contract_misconfiguration_fails_before_geometry_tf_or_wsl(self):
        snapshot = make_snapshot(np.ones((3, 4), dtype=np.uint16) * 2200)
        cases = (
            (
                {'accept_position_only_fallback': True},
                'POSITION_ONLY_FALLBACK_FORBIDDEN',
            ),
            (
                {'accept_orientation_fallback': True},
                'ORIENTATION_FALLBACK_FORBIDDEN',
            ),
            (
                {'orientation_variants_rpy_deg': [[0.0, 0.0, 0.0]]},
                'ORIENTATION_VARIANT_CONTRACT_INVALID',
            ),
            (
                {
                    'orientation_variants_rpy_deg': [
                        [0.0, 0.0, 0.0],
                        [0.0, 0.0, 180.0],
                        [0.0, 0.0, 360.0],
                    ],
                },
                'ORIENTATION_VARIANT_CONTRACT_INVALID',
            ),
        )
        for override, expected_code in cases:
            with self.subTest(expected_code=expected_code):
                client = RecordingClient(candidates=[])
                node = make_processing_node(client)
                node.candidate_frame_convention = 'opencv_optical'
                node._refresh_runtime_params = types.MethodType(
                    remote_node.RemoteGrasp6DNode._refresh_runtime_params,
                    node,
                )
                node._prepare_snapshot_geometry = (
                    lambda *_args: (_ for _ in ()).throw(
                        AssertionError(
                            'geometry/TF must not run after strict contract rejection'
                        )
                    )
                )
                remote_cfg = {
                    'candidate_frame_convention': 'opencv_optical',
                    'orientation_variants_rpy_deg': [
                        [0.0, 0.0, 0.0],
                        [0.0, 0.0, 180.0],
                    ],
                }
                remote_cfg.update(override)
                original_get_param = remote_node.rospy.get_param
                remote_node.rospy.get_param = lambda name, default=None: {
                    '/grasp_6d/remote': remote_cfg,
                }.get(name, default)
                try:
                    ok, message = node._process_frame(snapshot, manual=True)
                finally:
                    remote_node.rospy.get_param = original_get_param

                self.assertFalse(ok)
                self.assertTrue(
                    message.startswith(expected_code + ': '),
                    message,
                )
                self.assertEqual(client.calls, [])

    def test_runtime_candidate_convention_misconfiguration_fails_before_wsl(self):
        snapshot = make_snapshot(np.ones((3, 4), dtype=np.uint16) * 2200)
        client = RecordingClient(candidates=[])
        node = make_processing_node(client)
        node._refresh_runtime_params = types.MethodType(
            remote_node.RemoteGrasp6DNode._refresh_runtime_params,
            node,
        )
        node._prepare_snapshot_geometry = lambda *_args: (_ for _ in ()).throw(
            AssertionError('geometry/TF must not run after runtime convention rejection')
        )
        original_get_param = remote_node.rospy.get_param
        remote_node.rospy.get_param = lambda name, default=None: {
            '/grasp_6d/remote': {
                'candidate_frame_convention': 'ros_camera_link',
            },
            '/grasp_6d/remote/candidate_frame_convention': 'ros_camera_link',
        }.get(name, default)
        try:
            ok, message = node._process_frame(snapshot, manual=True)
        finally:
            remote_node.rospy.get_param = original_get_param

        self.assertFalse(ok)
        self.assertTrue(
            message.startswith('CANDIDATE_FRAME_CONVENTION_INVALID: '),
            message,
        )
        self.assertEqual(client.calls, [])

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
            size_xyz_m=[0.04, 0.04, 0.06],
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
                    depth_m=0.03,
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
        node._snapshot_base_optical_transform = (
            lambda *_args: identity_base_camera_snapshot_transform()
        )
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
                    depth_m=0.03,
                ),
                RemoteGraspCandidate(
                    0.40,
                    np.array([0.40, -0.10, 0.25]),
                    np.array([0.0, 0.0, 0.0, 1.0]),
                    0.090,
                    depth_m=0.03,
                ),
            ]
        )
        node = make_processing_node(client)
        node.pose_estimator = RecordingPoseEstimator()
        node._plan_reachable = lambda _pose: True
        original_estimator = remote_node.estimate_object_geometry
        remote_node.estimate_object_geometry = lambda **_kwargs: make_geometry_estimate(
            center_base=[0.40, -0.10, 0.25],
            size_xyz_m=[0.04, 0.04, 0.06],
        )
        node._snapshot_base_optical_transform = (
            lambda *_args: identity_base_camera_snapshot_transform()
        )
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
        final = node.rich_plan_pub.messages[-1]
        self.assertFalse(
            any(
                str(getattr(item, 'diagnostic', '')).startswith(
                    'PLAN_PENDING:'
                )
                for item in node.rich_plan_pub.messages
            )
        )
        self.assertTrue(final.valid)
        self.assertEqual(final.header.stamp.to_nsec(), snapshot.stamp_ns)
        self.assertGreater(client.calls[0]['kwargs']['request_id'], 0)
        self.assertEqual(
            client.calls[0]['kwargs']['snapshot_stamp_sec'],
            snapshot.stamp_sec,
        )

    def test_plan_publish_exception_invalidates_geometry_and_selected_gate(self):
        snapshot = make_snapshot(np.ones((3, 4), dtype=np.uint16) * 2200)
        client = RecordingClient(
            candidates=[
                RemoteGraspCandidate(
                    0.90,
                    np.array([0.40, -0.10, 0.25]),
                    np.array([0.0, 0.0, 0.0, 1.0]),
                    0.04,
                    depth_m=0.03,
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
            size_xyz_m=[0.04, 0.04, 0.06],
        )
        node._snapshot_base_optical_transform = (
            lambda *_args: identity_base_camera_snapshot_transform()
        )
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
        audit = json.loads(pathlib.Path(node.gate_audit_output_path).read_text())
        self.assertFalse(audit['outcome']['valid_plan'])
        self.assertEqual(audit['outcome']['code'], 'PLAN_PUBLISH_FAILED')

    def test_rich_plan_publish_exception_finishes_with_invalid_rich_and_empty_legacy(self):
        snapshot = make_snapshot(np.ones((3, 4), dtype=np.uint16) * 2200)
        client = RecordingClient(
            candidates=[
                RemoteGraspCandidate(
                    0.90,
                    np.array([0.40, -0.10, 0.25]),
                    np.array([0.0, 0.0, 0.0, 1.0]),
                    0.04,
                    depth_m=0.03,
                )
            ]
        )
        node = make_processing_node(client)
        node.pose_estimator = RecordingPoseEstimator()
        node._plan_reachable = lambda _pose: True

        class FailValidRichPublisher:
            def __init__(self):
                self.messages = []

            def publish(self, message):
                if bool(getattr(message, 'valid', False)):
                    raise RuntimeError('synthetic valid rich publish failure')
                self.messages.append(message)

        node.rich_plan_pub = FailValidRichPublisher()
        original_estimator = remote_node.estimate_object_geometry
        remote_node.estimate_object_geometry = lambda **_kwargs: make_geometry_estimate(
            center_base=[0.40, -0.10, 0.25],
            size_xyz_m=[0.04, 0.04, 0.06],
        )
        node._snapshot_base_optical_transform = (
            lambda *_args: identity_base_camera_snapshot_transform()
        )
        try:
            ok, message = node._process_frame(snapshot, manual=True)
        finally:
            remote_node.estimate_object_geometry = original_estimator

        self.assertFalse(ok)
        self.assertIn('synthetic valid rich publish failure', message)
        self.assertFalse(node.rich_plan_pub.messages[-1].valid)
        self.assertEqual(node.plan_pub.messages[-1].poses, [])
        self.assertIsNone(node._latest_rich_plan_copy())
        audit = json.loads(pathlib.Path(node.gate_audit_output_path).read_text())
        self.assertFalse(audit['outcome']['valid_plan'])
        self.assertEqual(audit['outcome']['code'], 'PLAN_PUBLISH_FAILED')

    def test_concurrent_publication_invalidation_rewrites_ready_audit_as_unpublished(self):
        snapshot = make_snapshot(np.ones((3, 4), dtype=np.uint16) * 2200)
        client = RecordingClient(
            candidates=[
                RemoteGraspCandidate(
                    0.90,
                    np.array([0.40, -0.10, 0.25]),
                    np.array([0.0, 0.0, 0.0, 1.0]),
                    0.04,
                    depth_m=0.03,
                )
            ]
        )
        node = make_processing_node(client)
        node.pose_estimator = RecordingPoseEstimator()
        node._plan_reachable = lambda _pose: True
        node._publish_plan_pair_if_current = lambda *_args, **_kwargs: (
            False,
            'TARGET_LOST',
        )
        original_estimator = remote_node.estimate_object_geometry
        remote_node.estimate_object_geometry = lambda **_kwargs: make_geometry_estimate(
            center_base=[0.40, -0.10, 0.25],
            size_xyz_m=[0.04, 0.04, 0.06],
        )
        node._snapshot_base_optical_transform = (
            lambda *_args: identity_base_camera_snapshot_transform()
        )
        try:
            ok, message = node._process_frame(snapshot, manual=True)
        finally:
            remote_node.estimate_object_geometry = original_estimator

        self.assertFalse(ok)
        self.assertTrue(message.startswith('TARGET_LOST: '), message)
        self.assertTrue(
            all(
                not bool(getattr(item, 'valid', False))
                for item in node.rich_plan_pub.messages
            )
        )
        audit = json.loads(pathlib.Path(node.gate_audit_output_path).read_text())
        self.assertFalse(audit['outcome']['valid_plan'])
        self.assertEqual(audit['outcome']['code'], 'TARGET_LOST')
        self.assertRegex(audit['plan_id'], r'^[0-9a-f]{24}$')

    def test_candidate_rank_exception_is_audited_and_invalidates_generation(self):
        snapshot = make_snapshot(np.ones((3, 4), dtype=np.uint16) * 2200)
        client = RecordingClient(
            candidates=[
                RemoteGraspCandidate(
                    0.90,
                    np.array([0.40, -0.10, 0.25]),
                    np.array([0.0, 0.0, 0.0, 1.0]),
                    0.04,
                    depth_m=0.03,
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
            size_xyz_m=[0.04, 0.04, 0.06],
        )
        node._snapshot_base_optical_transform = (
            lambda *_args: identity_base_camera_snapshot_transform()
        )
        try:
            ok, message = node._process_frame(snapshot, manual=True)
        finally:
            remote_node.estimate_object_geometry = original_estimator

        self.assertFalse(ok)
        self.assertTrue(
            message.startswith('NO_EXECUTABLE_CANDIDATE: '),
            message,
        )
        self.assertEqual(
            node._last_geometry_invalidation_code,
            'NO_EXECUTABLE_CANDIDATE',
        )
        self.assertFalse(node.latest_object_geometry.valid)
        self.assertIsNone(node._latest_geometry_estimate)
        self.assertIsNone(node.previous_object_axes_base)
        self.assertIsNone(node._selected_candidate_gate)
        self.assertIsNone(node.selected_required_open_width_m)
        self.assertEqual(node.plan_pub.messages[-1].poses, [])
        audit = json.loads(pathlib.Path(node.gate_audit_output_path).read_text())
        self.assertFalse(audit['outcome']['valid_plan'])
        self.assertEqual(
            audit['rows'][0]['planning_evaluation']['rank_error'][
                'failure_code'
            ],
            'RANK_EXCEPTION',
        )
        self.assertIn(
            'rank exploded',
            audit['rows'][0]['planning_evaluation']['rank_error'][
                'failure_reason'
            ],
        )

    def test_plan_diagnostics_finish_before_atomic_rich_and_legacy_publication(self):
        snapshot = make_snapshot(np.ones((3, 4), dtype=np.uint16) * 2200)
        client = RecordingClient(
            candidates=[
                RemoteGraspCandidate(
                    0.90,
                    np.array([0.40, -0.10, 0.25]),
                    np.array([0.0, 0.0, 0.0, 1.0]),
                    0.04,
                    depth_m=0.03,
                )
            ]
        )
        node = make_processing_node(client)
        node.pose_estimator = RecordingPoseEstimator()
        node._plan_reachable = lambda _pose: True
        plan_published = threading.Event()
        diagnostics_saw_publication = []

        class MarkPlanPublisher:
            def __init__(self):
                self.messages = []

            def publish(self, message):
                self.messages.append(message)
                if len(getattr(message, 'poses', ())) > 0:
                    plan_published.set()

        node.plan_pub = MarkPlanPublisher()
        original_support_metrics = node._candidate_support_geometry_metrics

        def record_diagnostic_order(candidate):
            diagnostics_saw_publication.append(plan_published.is_set())
            return original_support_metrics(candidate)

        node._candidate_support_geometry_metrics = record_diagnostic_order
        original_estimator = remote_node.estimate_object_geometry
        remote_node.estimate_object_geometry = lambda **_kwargs: make_geometry_estimate(
            center_base=[0.40, -0.10, 0.25],
            size_xyz_m=[0.04, 0.04, 0.06],
        )
        node._snapshot_base_optical_transform = (
            lambda *_args: identity_base_camera_snapshot_transform()
        )
        try:
            ok, message = node._process_frame(snapshot, manual=True)
        finally:
            remote_node.estimate_object_geometry = original_estimator

        self.assertTrue(ok, message)
        self.assertGreaterEqual(len(diagnostics_saw_publication), 1)
        self.assertTrue(all(value is False for value in diagnostics_saw_publication))
        self.assertTrue(plan_published.is_set())
        self.assertTrue(node.rich_plan_pub.messages[-1].valid)
        self.assertEqual(len(node.plan_pub.messages[-1].poses), 4)

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
        node.graspnet_input_bbox_min_iou = 0.50
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
        self.assertIsNone(
            node._active_graspnet_input_audit['detected_bbox_mask_iou']
        )
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

    def test_late_obb_failure_does_not_overwrite_concurrent_target_loss(self):
        snapshot = make_snapshot(np.ones((3, 4), dtype=np.uint16) * 2200)
        node = make_processing_node(RecordingClient(candidates=[]))

        def target_loss_then_failed_estimate(**_kwargs):
            node._invalidate_geometry(
                'TARGET_LOST',
                'target disappeared during geometry estimation',
                stamp=remote_node.rospy.Time(10, 123),
                snapshot=snapshot,
            )
            return make_geometry_estimate(
                ok=False,
                code='OBB_INVALID',
                reason='late geometry estimator failure',
            )

        original_estimator = remote_node.estimate_object_geometry
        remote_node.estimate_object_geometry = target_loss_then_failed_estimate
        node._snapshot_base_optical_transform = lambda *_args: np.eye(4)
        try:
            ok, message = node._process_frame(snapshot, manual=True)
        finally:
            remote_node.estimate_object_geometry = original_estimator

        self.assertFalse(ok)
        self.assertEqual(node._last_geometry_invalidation_code, 'TARGET_LOST')
        self.assertEqual(
            message,
            'TARGET_LOST: target disappeared during geometry estimation',
        )
        self.assertEqual(
            node.latest_object_geometry.failure_reason,
            'TARGET_LOST: target disappeared during geometry estimation',
        )

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
        self.assertEqual(
            remote_node.remote_prediction_failure_code(
                remote_node.GraspNetInputContextError(
                    'CONTEXT_TEST_FAILURE',
                    'synthetic context failure',
                )
            ),
            'CONTEXT_TEST_FAILURE',
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
            RemoteGraspCandidate(0.99, np.array([0.10, 0.0, 0.2]), np.array([0.0, 0.0, 0.0, 1.0]), 0.04, depth_m=0.03),
            RemoteGraspCandidate(0.80, np.array([0.65, 0.0, 0.2]), np.array([0.0, 0.0, 0.0, 1.0]), 0.04, depth_m=0.03),
        ]

        selected, pose = remote_node.select_first_reachable_candidate(
            candidates,
            FakePoseEstimator(),
            lambda pose: pose.pose.position.x > 1.5,
            stamp=None,
            camera_frame='camera_link',
            candidate_frame_convention='ros_camera_link',
        )

        self.assertAlmostEqual(selected.score, candidates[1].score)
        np.testing.assert_allclose(selected.translation_m, candidates[1].translation_m)
        self.assertAlmostEqual(pose.pose.position.x, 1.68)

    def test_selects_next_candidate_when_target_gate_rejects_first(self):
        candidates = [
            RemoteGraspCandidate(0.99, np.array([0.10, 0.0, 0.2]), np.array([0.0, 0.0, 0.0, 1.0]), 0.04, depth_m=0.03),
            RemoteGraspCandidate(0.80, np.array([0.20, 0.0, 0.2]), np.array([0.0, 0.0, 0.0, 1.0]), 0.04, depth_m=0.03),
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

        self.assertAlmostEqual(selected.score, candidates[1].score)
        np.testing.assert_allclose(selected.translation_m, candidates[1].translation_m)
        self.assertAlmostEqual(pose.pose.position.x, 1.23)
        self.assertEqual(seen_x, [1.13, 1.23])

    def test_target_rank_prefers_closest_reachable_candidate_over_higher_score(self):
        candidates = [
            RemoteGraspCandidate(0.99, np.array([0.035, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0]), 0.04, depth_m=0.03),
            RemoteGraspCandidate(0.70, np.array([0.010, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0]), 0.04, depth_m=0.03),
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

        self.assertAlmostEqual(selected.score, candidates[1].score)
        np.testing.assert_allclose(selected.translation_m, candidates[1].translation_m)
        self.assertAlmostEqual(pose.pose.position.x, 1.04)

    def test_nonfinite_rank_tuple_is_a_candidate_hard_reject(self):
        candidate = RemoteGraspCandidate(
            0.9,
            np.array([0.01, 0.0, 0.0]),
            np.array([0.0, 0.0, 0.0, 1.0]),
            0.04,
            depth_m=0.03,
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

    def test_invalid_rank_tuple_conversion_is_audited_as_candidate_rejection(self):
        candidate = RemoteGraspCandidate(
            0.9,
            np.array([0.01, 0.0, 0.0]),
            np.array([0.0, 0.0, 0.0, 1.0]),
            0.04,
            depth_m=0.03,
        )

        records = []
        selected, pose = remote_node.select_first_reachable_candidate(
            [candidate],
            FakePoseEstimator(),
            lambda _pose: True,
            stamp=None,
            camera_frame='camera_link',
            candidate_frame_convention='ros_camera_link',
            candidate_rank_fn=lambda *_args: ('not-a-number',),
            evaluation_record_sink=records.append,
        )

        self.assertIsNone(selected)
        self.assertIsNone(pose)
        self.assertEqual(len(records), 1)
        self.assertEqual(records[0]['rank_error']['failure_code'], 'RANK_EXCEPTION')

    def test_analytical_geometry_gate_runs_before_reachability_and_high_score(self):
        candidates = [
            RemoteGraspCandidate(
                0.99,
                np.array([0.30, 0.0, 0.20]),
                np.array([0.0, 0.0, 0.0, 1.0]),
                0.01,
                depth_m=0.03,
            ),
            RemoteGraspCandidate(
                0.40,
                np.array([0.20, 0.0, 0.20]),
                np.array([0.0, 0.0, 0.0, 1.0]),
                0.09,
                depth_m=0.03,
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
        self.assertAlmostEqual(pose.pose.position.x, 0.23)
        self.assertEqual(
            events,
            [
                ('geometry', 0.99, 4),
                ('geometry', 0.40, 4),
                ('reachability', 0.23),
            ],
        )

    def test_selector_emits_exact_complete_lineage_when_each_key_stage_raises(self):
        candidate = RemoteGraspCandidate(
            0.90,
            np.array([0.20, 0.0, 0.20]),
            np.array([0.0, 0.0, 0.0, 1.0]),
            0.04,
            depth_m=0.03,
        )
        variants = [
            np.array([0.0, 0.0, 0.0, 1.0]),
            remote_node.quaternion_from_euler(0.0, 0.0, np.pi),
        ]

        class FailingPoseEstimator:
            def make_base_pose_from_camera_pose(self, *_args, **_kwargs):
                raise RuntimeError('synthetic snapshot pose failure')

        def raises(message):
            def action(*_args, **_kwargs):
                raise RuntimeError(message)

            return action

        cases = (
            (
                'snapshot_pose',
                FailingPoseEstimator(),
                {'reachability_fn': lambda _pose: True},
                lambda row: self.assertEqual(
                    row['candidate_contract']['stage'],
                    'snapshot_pose',
                ),
            ),
            (
                'analytical_geometry',
                RecordingPoseEstimator(),
                {
                    'reachability_fn': lambda _pose: True,
                    'candidate_geometry_fn': raises(
                        'synthetic analytical geometry failure'
                    ),
                },
                lambda row: self.assertEqual(
                    row['analytical_result']['failure_code'],
                    'ANALYTICAL_GEOMETRY_EXCEPTION',
                ),
            ),
            (
                'target_filter',
                RecordingPoseEstimator(),
                {
                    'reachability_fn': lambda _pose: True,
                    'candidate_filter_fn': raises(
                        'synthetic target filter failure'
                    ),
                },
                lambda row: self.assertEqual(
                    row['target_filter']['failure_code'],
                    'TARGET_FILTER_EXCEPTION',
                ),
            ),
            (
                'strict_reachability',
                RecordingPoseEstimator(),
                {
                    'reachability_fn': raises(
                        'synthetic strict reachability failure'
                    ),
                },
                lambda row: self.assertEqual(
                    row['strict_reachability']['failure_code'],
                    'STRICT_REACHABILITY_EXCEPTION',
                ),
            ),
            (
                'rank',
                RecordingPoseEstimator(),
                {
                    'reachability_fn': lambda _pose: True,
                    'candidate_rank_fn': raises('synthetic rank failure'),
                },
                lambda row: self.assertEqual(
                    row['rank_error']['failure_code'],
                    'RANK_EXCEPTION',
                ),
            ),
        )
        required_keys = {
            'candidate_index',
            'variant_index',
            'candidate_contract',
            'analytical_result',
            'target_filter',
            'strict_reachability',
            'rank',
            'rank_valid',
            'selected',
        }
        for stage, estimator, kwargs, assert_stage in cases:
            with self.subTest(stage=stage):
                records = []
                selected, pose = remote_node.select_first_reachable_candidate(
                    [candidate],
                    estimator,
                    kwargs.pop('reachability_fn'),
                    stamp=None,
                    camera_frame='camera_link',
                    candidate_frame_convention='ros_camera_link',
                    orientation_variant_quaternions=variants,
                    evaluation_record_sink=records.append,
                    grasp_config={'tool_approach_axis': 'z'},
                    **kwargs,
                )

                self.assertIsNone(selected)
                self.assertIsNone(pose)
                self.assertEqual(len(records), 2)
                self.assertEqual(
                    [
                        (row['candidate_index'], row['variant_index'])
                        for row in records
                    ],
                    [(0, 0), (0, 1)],
                )
                for row in records:
                    self.assertTrue(required_keys.issubset(row))
                    self.assertFalse(row['selected'])
                    assert_stage(row)

    def test_selector_uses_geometry_tuple_with_motion_before_model_score(self):
        candidates = [
            RemoteGraspCandidate(
                0.99,
                np.array([0.10, 0.0, 0.20]),
                np.array([0.0, 0.0, 0.0, 1.0]),
                0.04,
                depth_m=0.03,
            ),
            RemoteGraspCandidate(
                0.10,
                np.array([0.20, 0.0, 0.20]),
                np.array([0.0, 0.0, 0.0, 1.0]),
                0.04,
                depth_m=0.03,
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

    def test_selector_equal_rank_is_stable_and_preserves_wsl_lineage(self):
        candidates = [
            RemoteGraspCandidate(
                0.90,
                np.array([0.10, 0.0, 0.20]),
                np.array([0.0, 0.0, 0.0, 1.0]),
                0.04,
                depth_m=0.03,
            ),
            RemoteGraspCandidate(
                0.90,
                np.array([0.20, 0.0, 0.20]),
                np.array([0.0, 0.0, 0.0, 1.0]),
                0.04,
                depth_m=0.03,
            ),
        ]
        variants = [
            np.array([0.0, 0.0, 0.0, 1.0]),
            remote_node.quaternion_from_euler(0.0, 0.0, np.pi),
        ]
        reachability_calls = []

        selected, _pose = remote_node.select_first_reachable_candidate(
            candidates,
            RecordingPoseEstimator(),
            lambda pose: reachability_calls.append(pose) or True,
            stamp=None,
            camera_frame='camera_link',
            candidate_frame_convention='ros_camera_link',
            orientation_variant_quaternions=variants,
            candidate_rank_fn=lambda *_args: (1.0,),
            grasp_config={},
        )

        self.assertEqual(len(reachability_calls), 4)
        self.assertEqual(selected._raw_candidate_index, 0)
        self.assertEqual(selected._variant_index, 0)
        np.testing.assert_array_equal(
            selected.translation_m,
            candidates[0].translation_m,
        )
        self.assertAlmostEqual(selected.score, candidates[0].score)
        self.assertAlmostEqual(selected.width_m, candidates[0].width_m)
        self.assertAlmostEqual(selected.depth_m, candidates[0].depth_m)

        subset, _pose = remote_node.select_first_reachable_candidate(
            candidates,
            RecordingPoseEstimator(),
            lambda _pose: True,
            stamp=None,
            camera_frame='camera_link',
            candidate_frame_convention='ros_camera_link',
            orientation_variant_quaternions=variants,
            candidate_filter_fn=lambda raw, *_args: raw is candidates[1],
            candidate_rank_fn=lambda *_args: (1.0,),
            grasp_config={},
        )
        self.assertEqual(subset._raw_candidate_index, 1)
        self.assertEqual(subset._variant_index, 0)
        np.testing.assert_array_equal(subset.translation_m, candidates[1].translation_m)

        # With candidate 0 / variant 0 removed, a stable WSL-lineage tie-break
        # selects candidate 0 / variant 1 before candidate 1 / variant 0.
        lineage_tie, _pose = remote_node.select_first_reachable_candidate(
            candidates,
            RecordingPoseEstimator(),
            lambda _pose: True,
            stamp=None,
            camera_frame='camera_link',
            candidate_frame_convention='ros_camera_link',
            orientation_variant_quaternions=variants,
            candidate_filter_fn=lambda raw, converted, _pose: not (
                raw is candidates[0]
                and int(getattr(converted, '_variant_index', -1)) == 0
            ),
            candidate_rank_fn=lambda *_args: (1.0,),
            grasp_config={},
        )
        self.assertEqual(lineage_tie._raw_candidate_index, 0)
        self.assertEqual(lineage_tie._variant_index, 1)

    def test_selector_missing_axis_defaults_four_stage_path_to_tool_plus_z(self):
        candidate = RemoteGraspCandidate(
            0.90,
            np.array([0.20, 0.0, 0.20]),
            np.array([0.0, 0.0, 0.0, 1.0]),
            0.04,
            depth_m=0.03,
        )
        observed = {}

        def geometry_gate(_raw, converted, pose, plan):
            observed['converted'] = converted
            observed['pose'] = pose
            observed['plan'] = plan
            return remote_node.CandidateGateResult(
                True, '', '', 0.044, 0.0, 0.01, 1.0, 0.0, 0.0, '', 6
            )

        selected, pose = remote_node.select_first_reachable_candidate(
            [candidate],
            RecordingPoseEstimator(),
            lambda _pose: True,
            stamp=None,
            camera_frame='camera_link',
            candidate_frame_convention='ros_camera_link',
            candidate_geometry_fn=geometry_gate,
            grasp_config={
                'pregrasp_distance_m': 0.08,
                'final_approach_offset_m': 0.02,
                'lift_height_m': 0.05,
            },
        )

        self.assertIsNotNone(selected)
        tool_rotation = remote_node.pose_matrix(pose)[:3, :3]
        grasp_xyz = remote_node.pose_matrix(observed['plan'].grasp)[:3, 3]
        pregrasp_xyz = remote_node.pose_matrix(observed['plan'].pregrasp)[:3, 3]
        approach_xyz = remote_node.pose_matrix(observed['plan'].approach)[:3, 3]
        np.testing.assert_allclose(
            grasp_xyz - pregrasp_xyz,
            0.08 * tool_rotation[:, 2],
            atol=1e-8,
        )
        np.testing.assert_allclose(
            grasp_xyz - approach_xyz,
            0.02 * tool_rotation[:, 2],
            atol=1e-8,
        )

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
        np.testing.assert_allclose(selected.translation_m, [0.30, 0.0, 0.0])
        np.testing.assert_allclose(selected.tool0_translation_m, [0.33, 0.0, 0.0])
        np.testing.assert_allclose(estimator.calls[0][0], [0.33, 0.0, 0.0])
        self.assertAlmostEqual(selected.depth_m, 0.03)

    def test_execution_tool0_config_rejects_depth_disable_wrong_rotation_and_axis(self):
        strict = remote_node.STRICT_MODEL_GRASP_TO_TOOL_QUATERNION
        cases = (
            (False, strict, {'tool_approach_axis': 'z'}, 'DEPTH_CONTRACT_DISABLED'),
            (True, [0.0, 0.0, 0.0, 1.0], {'tool_approach_axis': 'z'}, 'TOOL_FRAME_CONTRACT_INVALID'),
            (True, strict, {'tool_approach_axis': 'x'}, 'TOOL_FRAME_CONTRACT_INVALID'),
            (True, strict, {'tool_approach_axis': '-z'}, 'TOOL_FRAME_CONTRACT_INVALID'),
        )
        for require_depth, correction, config, expected_code in cases:
            with self.subTest(expected_code=expected_code, config=config):
                with self.assertRaises(remote_node.CandidateContractError) as context:
                    remote_node.validate_execution_tool0_contract(
                        require_depth,
                        correction,
                        config,
                    )
                self.assertEqual(context.exception.code, expected_code)

    def test_depth_offsets_tool0_in_positive_model_x_for_all_graspnet_depth_bins(self):
        correction = remote_node.quaternion_from_euler(0.0, np.pi * 0.5, 0.0)
        center = np.array([0.30, -0.02, 0.10])

        for depth in (0.01, 0.02, 0.03, 0.04):
            with self.subTest(depth=depth):
                candidate = RemoteGraspCandidate(
                    score=0.9,
                    translation_m=center.copy(),
                    quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
                    width_m=0.04,
                    depth_m=depth,
                )

                aligned = remote_node.align_candidate_to_tool_frame(
                    candidate,
                    correction,
                    require_depth=True,
                )
                offset = aligned.tool0_translation_m - aligned.translation_m
                rotation = remote_node.quaternion_matrix(
                    aligned.quaternion_xyzw
                )[:3, :3]

                np.testing.assert_allclose(offset, [depth, 0.0, 0.0], atol=1e-9)
                np.testing.assert_allclose(offset, depth * rotation[:, 2], atol=1e-9)
                self.assertAlmostEqual(float(np.linalg.norm(offset)), depth)

    def test_existing_consistent_tool0_is_not_offset_by_depth_twice(self):
        depth = 0.03
        center = np.array([0.30, 0.0, 0.0])
        candidate = RemoteGraspCandidate(
            score=0.9,
            translation_m=center,
            quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
            width_m=0.04,
            depth_m=depth,
            tool0_translation_m=center + np.array([depth, 0.0, 0.0]),
        )
        correction = remote_node.quaternion_from_euler(0.0, np.pi * 0.5, 0.0)

        aligned = remote_node.align_candidate_to_tool_frame(
            candidate,
            correction,
            require_depth=True,
        )

        np.testing.assert_allclose(aligned.tool0_translation_m, [0.33, 0.0, 0.0])

    def test_strict_depth_contract_rejects_missing_nonfinite_and_nonpositive_depth(self):
        correction = remote_node.quaternion_from_euler(0.0, np.pi * 0.5, 0.0)
        reached = []
        for depth in (None, 0.0, -0.01, float('nan'), float('inf')):
            with self.subTest(depth=depth):
                candidate = RemoteGraspCandidate(
                    score=0.9,
                    translation_m=np.array([0.30, 0.0, 0.0]),
                    quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
                    width_m=0.04,
                    depth_m=depth,
                )
                selected, pose = remote_node.select_first_reachable_candidate(
                    [candidate],
                    RecordingPoseEstimator(),
                    lambda _pose: reached.append(True) or True,
                    stamp=None,
                    camera_frame='camera_link',
                    candidate_frame_convention='ros_camera_link',
                    model_grasp_to_tool_quaternion=correction,
                    require_candidate_depth=True,
                )
                self.assertIsNone(selected)
                self.assertIsNone(pose)
        self.assertEqual(reached, [])

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
        np.testing.assert_allclose(estimator.calls[0][0], [0.33, 0.0, 0.0])
        np.testing.assert_allclose(estimator.calls[1][0], estimator.calls[0][0])
        first = remote_node.quaternion_matrix(estimator.calls[0][1])[:3, :3]
        symmetric = remote_node.quaternion_matrix(estimator.calls[1][1])[:3, :3]
        np.testing.assert_allclose(symmetric[:, 2], first[:, 2], atol=1e-7)
        np.testing.assert_allclose(symmetric[:, 1], -first[:, 1], atol=1e-7)
        np.testing.assert_allclose(symmetric[:, 0], -first[:, 0], atol=1e-7)

    def test_identity_variants_are_distinct_objects_so_lineage_cannot_be_overwritten(self):
        candidate = RemoteGraspCandidate(
            score=0.9,
            translation_m=np.array([0.30, 0.0, 0.0]),
            quaternion_xyzw=remote_node.STRICT_MODEL_GRASP_TO_TOOL_QUATERNION,
            width_m=0.04,
            depth_m=0.03,
            tool0_translation_m=np.array([0.33, 0.0, 0.0]),
        )
        identity = np.asarray([0.0, 0.0, 0.0, 1.0])

        first = remote_node.make_parallel_jaw_variant(candidate, identity)
        second = remote_node.make_parallel_jaw_variant(candidate, identity)
        first._raw_candidate_index = 7
        first._variant_index = 0

        self.assertIsNot(first, candidate)
        self.assertIsNot(second, candidate)
        self.assertIsNot(first, second)
        self.assertFalse(hasattr(second, '_raw_candidate_index'))
        self.assertFalse(hasattr(second, '_variant_index'))
        self.assertFalse(
            np.shares_memory(first.translation_m, second.translation_m)
        )
        self.assertFalse(
            np.shares_memory(
                first.tool0_translation_m,
                second.tool0_translation_m,
            )
        )

    def test_shared_parallel_jaw_variant_rejects_non_rz180_corrections(self):
        aligned = RemoteGraspCandidate(
            score=0.9,
            translation_m=np.array([0.30, 0.0, 0.0]),
            quaternion_xyzw=remote_node.STRICT_MODEL_GRASP_TO_TOOL_QUATERNION,
            width_m=0.04,
            depth_m=0.03,
            tool0_translation_m=np.array([0.33, 0.0, 0.0]),
        )
        for correction in (
            remote_node.quaternion_from_euler(np.pi, 0.0, 0.0),
            remote_node.quaternion_from_euler(0.0, np.pi, 0.0),
            remote_node.quaternion_from_euler(0.0, 0.0, np.pi * 0.5),
        ):
            with self.subTest(correction=correction):
                with self.assertRaises(remote_node.CandidateContractError) as context:
                    remote_node.make_parallel_jaw_variant(aligned, correction)
                self.assertEqual(
                    context.exception.code,
                    'ORIENTATION_VARIANT_INVALID',
                )

    def test_center_is_recovered_from_tool0_under_nonidentity_base_transform(self):
        center_camera = np.array([0.30, 0.10, 0.20])
        tool_rotation_camera = remote_node.STRICT_MODEL_GRASP_TO_TOOL_ROTATION
        tool0_camera = center_camera + 0.03 * tool_rotation_camera[:, 2]
        candidate = RemoteGraspCandidate(
            score=0.9,
            translation_m=center_camera,
            quaternion_xyzw=remote_node.STRICT_MODEL_GRASP_TO_TOOL_QUATERNION,
            width_m=0.04,
            depth_m=0.03,
            tool0_translation_m=tool0_camera,
        )
        base_from_camera = remote_node.transform_matrix(
            [1.0, 2.0, 3.0],
            remote_node.quaternion_from_euler(0.0, 0.0, np.pi * 0.5),
        )
        expected_center = (
            base_from_camera[:3, :3].dot(center_camera)
            + base_from_camera[:3, 3]
        )

        for variant in (
            candidate,
            remote_node.make_parallel_jaw_variant(
                candidate,
                remote_node.quaternion_from_euler(0.0, 0.0, np.pi),
            ),
        ):
            base_from_tool = np.eye(4, dtype=float)
            base_from_tool[:3, :3] = base_from_camera[:3, :3].dot(
                remote_node.quaternion_matrix(variant.quaternion_xyzw)[:3, :3]
            )
            base_from_tool[:3, 3] = (
                base_from_camera[:3, :3].dot(tool0_camera)
                + base_from_camera[:3, 3]
            )
            pose = PoseStamped()
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = (
                base_from_tool[:3, 3]
            )
            quaternion = remote_node.quaternion_from_matrix(base_from_tool)
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]

            recovered = remote_node.candidate_center_base_from_tool0_pose(
                variant,
                pose,
            )
            np.testing.assert_allclose(recovered, expected_center, atol=1e-8)

    def test_tool0_drives_geometry_reachability_and_four_stage_trajectory(self):
        candidate = RemoteGraspCandidate(
            score=0.9,
            translation_m=np.array([0.30, 0.0, 0.10]),
            quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
            width_m=0.04,
            depth_m=0.03,
        )
        correction = remote_node.quaternion_from_euler(0.0, np.pi * 0.5, 0.0)
        observed = {}

        def geometry_gate(_raw, converted, pose, plan):
            observed['center_camera'] = converted.translation_m.copy()
            observed['tool0_camera'] = converted.tool0_translation_m.copy()
            observed['center_base'] = converted._center_base_xyz.copy()
            observed['geometry_pose'] = np.array(
                [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
            )
            observed['trajectory_x'] = np.array(
                [
                    plan.pregrasp.pose.position.x,
                    plan.approach.pose.position.x,
                    plan.grasp.pose.position.x,
                    plan.lift.pose.position.x,
                ]
            )
            return remote_node.CandidateGateResult(
                True, '', '', 0.044, 0.0, 0.01, 1.0, 0.0, 0.0, '', 6
            )

        selected, pose = remote_node.select_first_reachable_candidate(
            [candidate],
            RecordingPoseEstimator(),
            lambda reachable_pose: observed.setdefault(
                'reachable_x', reachable_pose.pose.position.x
            ) is not None,
            stamp=None,
            camera_frame='camera_link',
            candidate_frame_convention='ros_camera_link',
            model_grasp_to_tool_quaternion=correction,
            candidate_geometry_fn=geometry_gate,
            grasp_config={
                'pregrasp_distance_m': 0.08,
                'final_approach_offset_m': 0.015,
                'lift_height_m': 0.05,
                'tool_approach_axis': 'z',
            },
            require_candidate_depth=True,
        )

        self.assertIsNotNone(selected)
        np.testing.assert_allclose(observed['center_camera'], [0.30, 0.0, 0.10])
        np.testing.assert_allclose(observed['center_base'], [0.30, 0.0, 0.10])
        np.testing.assert_allclose(observed['tool0_camera'], [0.33, 0.0, 0.10])
        np.testing.assert_allclose(observed['geometry_pose'], [0.33, 0.0, 0.10])
        np.testing.assert_allclose(observed['trajectory_x'], [0.25, 0.315, 0.33, 0.33])
        self.assertAlmostEqual(observed['reachable_x'], 0.33)
        self.assertAlmostEqual(pose.pose.position.x, 0.33)

        geometry = make_geometry_message()
        rich_plan = remote_node.build_rich_plan(
            selected,
            geometry,
            geometry.header,
            'carton_segment',
        )
        payload = mujoco_digital_twin_client.build_mujoco_payload(
            rich_plan,
            ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6'],
            [0.0] * 6,
            {
                'name': 'Alicia_D_v5_6_gripper_50mm',
                'max_inner_gap_m': 0.050,
            },
        )
        serialized_x = [
            item['position_m'][0]
            for item in payload['trajectory']
        ]
        np.testing.assert_allclose(serialized_x, [0.25, 0.315, 0.33, 0.33])
        self.assertNotIn(0.30, serialized_x)

    def test_target_gate_and_audit_use_center_while_recording_tool0(self):
        node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
        node.require_candidate_depth = True
        node.candidate_target_gate_enabled = True
        node.candidate_max_target_distance_m = 0.005
        node.candidate_min_relative_z_m = -0.005
        node.candidate_max_relative_z_m = 0.005
        node.target_cloud_candidate_max_point_distance_m = 0.0
        node.target_cloud_candidate_min_support_clearance_m = -0.002
        node.latest_support_plane_camera_point = None
        node.latest_support_plane_camera_normal = None
        node.camera_visibility_gate_enabled = False
        node._target_gate_rejected_count = 0
        node._approach_gate_rejected_count = 0
        node._target_base_xyz = lambda: (np.array([0.30, 0.0, 0.10]), 'test')
        node._camera_candidate_cloud_distance = lambda _candidate: float('inf')
        node._candidate_approach_downward_cos = lambda _candidate, _pose: 1.0

        candidate = RemoteGraspCandidate(
            score=0.9,
            translation_m=np.array([0.30, 0.0, 0.10]),
            quaternion_xyzw=remote_node.quaternion_from_euler(
                0.0, np.pi * 0.5, 0.0
            ),
            width_m=0.04,
            depth_m=0.03,
            tool0_translation_m=np.array([0.33, 0.0, 0.10]),
        )
        candidate._center_base_xyz = np.array([0.30, 0.0, 0.10])
        tool0_pose = PoseStamped()
        tool0_pose.pose.position.x = 0.33
        tool0_pose.pose.position.z = 0.10
        q = candidate.quaternion_xyzw
        tool0_pose.pose.orientation.x = q[0]
        tool0_pose.pose.orientation.y = q[1]
        tool0_pose.pose.orientation.z = q[2]
        tool0_pose.pose.orientation.w = q[3]

        self.assertTrue(
            node._candidate_matches_target(None, candidate, tool0_pose)
        )
        row = node._candidate_gate_audit_row(0, 0, candidate, tool0_pose)

        self.assertTrue(row['target_ok'])
        self.assertAlmostEqual(row['center_distance_m'], 0.0)
        np.testing.assert_allclose(row['center_camera_m'], [0.30, 0.0, 0.10])
        np.testing.assert_allclose(row['tool0_camera_m'], [0.33, 0.0, 0.10])
        np.testing.assert_allclose(row['center_base_m'], [0.30, 0.0, 0.10])
        np.testing.assert_allclose(row['tool0_base_m'], [0.33, 0.0, 0.10])
        np.testing.assert_allclose(row['tool0_offset_camera_m'], [0.03, 0.0, 0.0])
        np.testing.assert_allclose(row['tool0_offset_base_m'], [0.03, 0.0, 0.0])
        self.assertAlmostEqual(row['tool0_contract_residual_camera_m'], 0.0)
        self.assertAlmostEqual(row['tool0_contract_residual_base_m'], 0.0)
        np.testing.assert_allclose(row['translation_camera_m'], row['center_camera_m'])
        self.assertAlmostEqual(row['depth_offset_m'], 0.03)

    def test_full_gate_audit_is_atomic_on_disk_and_topic_stays_bounded(self):
        node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
        node.gate_audit_pub = RecordingPublisher()
        compact_transform = {
            'snapshot_stamp_ns': 10_000_000_123,
            'snapshot_source_frame': 'camera_link',
            'raw_candidate_convention': 'opencv_optical',
            'canonical_candidate_frame': 'camera_link',
            'transform_sha256': 'a' * 64,
        }
        node._latest_gate_audit_summary = {
            'base_candidates': 936,
            'profiles': [],
            'snapshot_transform': compact_transform,
        }
        selected = {
            'candidate_index': 7,
            'variant_index': 1,
            'score': 0.9,
            'depth_m': 0.03,
            'center_camera_m': [0.30, 0.0, 0.10],
            'tool0_camera_m': [0.33, 0.0, 0.10],
            'center_base_m': [0.40, -0.10, 0.20],
            'tool0_base_m': [0.43, -0.10, 0.20],
            'tool0_offset_camera_m': [0.03, 0.0, 0.0],
            'tool0_offset_base_m': [0.03, 0.0, 0.0],
            'tool0_contract_residual_camera_m': 0.0,
            'tool0_contract_residual_base_m': 0.0,
        }
        rows = [dict(selected, candidate_index=index) for index in range(64)]
        report = {
            'summary': node._latest_gate_audit_summary,
            'snapshot_transform': dict(
                compact_transform,
                T_base_optical=np.eye(4).tolist(),
                T_base_camera_link=np.eye(4).tolist(),
            ),
            'rows': rows,
        }

        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / 'gate-audit.json'
            node.gate_audit_output_path = str(path)
            reference = node._write_gate_audit_report(report)
            node._latest_gate_audit_reference = reference
            node._publish_bounded_gate_audit(selected)

            payload = path.read_bytes()
            on_disk = json.loads(payload.decode('utf-8'))
            topic = json.loads(node.gate_audit_pub.messages[-1])

            self.assertEqual(len(on_disk['rows']), 64)
            self.assertEqual(on_disk['rows'][0]['center_base_m'], selected['center_base_m'])
            self.assertEqual(on_disk['rows'][0]['tool0_base_m'], selected['tool0_base_m'])
            self.assertIn('tool0_contract_residual_base_m', on_disk['rows'][0])
            self.assertEqual(reference['report_sha256'], hashlib.sha256(payload).hexdigest())
            self.assertEqual(topic['row_count'], 64)
            self.assertNotIn('rows', topic)
            self.assertEqual(topic['selected']['tool0_base_m'], selected['tool0_base_m'])
            self.assertEqual(topic['selected']['candidate_index'], 7)
            self.assertEqual(topic['selected']['variant_index'], 1)
            self.assertEqual(
                topic['summary']['snapshot_transform']['transform_sha256'],
                'a' * 64,
            )
            self.assertIn('T_base_optical', on_disk['snapshot_transform'])
            self.assertNotIn('T_base_optical', node.gate_audit_pub.messages[-1])
            self.assertLess(len(node.gate_audit_pub.messages[-1]), 4096)
            self.assertEqual(list(path.parent.glob('*.tmp-*')), [])

    def test_valid_plan_requires_complete_final_audit_with_plan_id_before_publish(self):
        snapshot = make_snapshot(np.ones((3, 4), dtype=np.uint16) * 2200)
        candidate = RemoteGraspCandidate(
            0.90,
            np.array([0.40, -0.10, 0.25]),
            np.array([0.0, 0.0, 0.0, 1.0]),
            0.04,
            depth_m=0.03,
        )
        node = make_processing_node(RecordingClient(candidates=[candidate]))
        node.pose_estimator = RecordingPoseEstimator()
        node._plan_reachable = lambda _pose: True
        node.orientation_variant_quaternions = [
            np.asarray([0.0, 0.0, 0.0, 1.0]),
            remote_node.quaternion_from_euler(0.0, 0.0, np.pi),
        ]
        original_estimator = remote_node.estimate_object_geometry
        remote_node.estimate_object_geometry = lambda **_kwargs: make_geometry_estimate(
            center_base=[0.40, -0.10, 0.25],
            size_xyz_m=[0.04, 0.04, 0.06],
        )
        node._snapshot_base_optical_transform = (
            lambda *_args: identity_base_camera_snapshot_transform()
        )

        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / 'final-audit.json'
            node.gate_audit_output_path = str(path)

            class AuditBeforeRichPublisher(RecordingPublisher):
                def publish(self, message):
                    if bool(getattr(message, 'valid', False)):
                        self.assert_audit(message)
                    super().publish(message)

                @staticmethod
                def assert_audit(message):
                    if not path.is_file():
                        raise AssertionError('audit must exist before rich plan')
                    payload = json.loads(path.read_text())
                    if payload.get('plan_id') != message.plan_id:
                        raise AssertionError('audit plan_id must bind rich plan')

            node.rich_plan_pub = AuditBeforeRichPublisher()
            try:
                ok, message = node._process_frame(snapshot, manual=True)
            finally:
                remote_node.estimate_object_geometry = original_estimator

            self.assertTrue(ok, message)
            report = json.loads(path.read_text())
            self.assertEqual(report['plan_id'], node.rich_plan_pub.messages[-1].plan_id)
            self.assertTrue(report['outcome']['valid_plan'])
            self.assertEqual(report['outcome']['code'], 'PLAN_READY')
            self.assertEqual(len(report['rows']), 2)
            self.assertEqual(len(report['lineage']), 2)
            selected = [
                row for row in report['rows'] if bool(row.get('selected', False))
            ]
            self.assertEqual(len(selected), 1)
            self.assertEqual(
                report['selected']['candidate_index'],
                selected[0]['candidate_index'],
            )
            self.assertEqual(
                report['selected']['variant_index'],
                selected[0]['variant_index'],
            )
            for row in report['rows']:
                evaluation = row['planning_evaluation']
                self.assertTrue(evaluation['candidate_contract']['ok'])
                analytical = evaluation['analytical_result']
                self.assertTrue(analytical['checked'])
                self.assertEqual(analytical['total_gate_count'], 6)
                self.assertEqual(analytical['passed_gate_count'], 6)
                self.assertTrue(evaluation['target_filter']['checked'])
                self.assertTrue(evaluation['target_filter']['ok'])
                self.assertTrue(evaluation['strict_reachability']['checked'])
                self.assertTrue(evaluation['strict_reachability']['ok'])

            bounded = json.loads(node.gate_audit_pub.messages[-1])
            self.assertNotIn('rows', bounded)
            self.assertEqual(
                bounded['report_sha256'],
                hashlib.sha256(path.read_bytes()).hexdigest(),
            )
            self.assertEqual(bounded['selected']['candidate_index'], 0)

    def test_audit_finalizer_requires_exact_valid_lineage_and_completes_failures(self):
        node = make_processing_node()
        rows = [
            {'candidate_index': 0, 'variant_index': 0},
            {'candidate_index': 0, 'variant_index': 1},
        ]

        def install_base_report():
            node._active_gate_audit_report = {
                'report_version': 2,
                'summary': {},
                'rows': deepcopy(rows),
                'lineage': [],
                'selected': None,
                'plan_id': '',
                'outcome': {
                    'code': '',
                    'reason': '',
                    'valid_plan': False,
                },
            }

        one_evaluation = remote_node.failed_planning_evaluation(
            0,
            0,
            'SYNTHETIC_FAILURE',
            'synthetic selector failure',
        )
        install_base_report()
        with self.assertRaises(
            remote_node.CandidateContractError
        ) as raised:
            node._finalize_gate_audit_report(
                evaluation_records=[one_evaluation],
                selected_candidate=None,
                selected_pose=None,
                plan_id='a' * 24,
                outcome_code='PLAN_READY',
                outcome_reason='synthetic ready plan',
                valid_plan=True,
            )
        self.assertEqual(
            raised.exception.code,
            remote_node.PLANNING_AUDIT_FAILED,
        )

        install_base_report()
        node._finalize_gate_audit_report(
            evaluation_records=[one_evaluation],
            selected_candidate=None,
            selected_pose=None,
            plan_id='',
            outcome_code='PLAN_FAILED',
            outcome_reason='synthetic failed plan',
            valid_plan=False,
        )
        failed_report = node._active_gate_audit_report
        self.assertFalse(failed_report['outcome']['valid_plan'])
        self.assertEqual(failed_report['outcome']['code'], 'PLAN_FAILED')
        self.assertEqual(
            failed_report['rows'][1]['planning_evaluation'][
                'candidate_contract'
            ]['failure_code'],
            'PLANNING_EVALUATION_MISSING',
        )
        self.assertEqual(
            {
                (
                    row['planning_evaluation']['candidate_index'],
                    row['planning_evaluation']['variant_index'],
                )
                for row in failed_report['rows']
            },
            {(0, 0), (0, 1)},
        )

        install_base_report()
        node._finalize_gate_audit_report(
            evaluation_records=[one_evaluation, deepcopy(one_evaluation)],
            selected_candidate=None,
            selected_pose=None,
            plan_id='',
            outcome_code='PLAN_FAILED',
            outcome_reason='synthetic failed plan',
            valid_plan=False,
        )
        inconsistent_report = node._active_gate_audit_report
        self.assertFalse(inconsistent_report['outcome']['valid_plan'])
        self.assertEqual(
            inconsistent_report['outcome']['code'],
            remote_node.PLANNING_AUDIT_FAILED,
        )
        self.assertIn(
            'duplicate audit lineage',
            inconsistent_report['outcome']['reason'],
        )

        selected_candidate = types.SimpleNamespace(
            _raw_candidate_index=0,
            _variant_index=0,
        )
        selected_pose = object()
        complete_evaluations = [
            remote_node.failed_planning_evaluation(
                0,
                variant_index,
                'SYNTHETIC_FAILURE',
                'synthetic complete selector row',
            )
            for variant_index in (0, 1)
        ]
        invalid_selected_cases = []
        invalid_selected_cases.append(
            ('zero-selected', deepcopy(complete_evaluations))
        )
        multiple = deepcopy(complete_evaluations)
        multiple[0]['selected'] = True
        multiple[1]['selected'] = True
        invalid_selected_cases.append(('multiple-selected', multiple))
        wrong = deepcopy(complete_evaluations)
        wrong[1]['selected'] = True
        invalid_selected_cases.append(('wrong-selected', wrong))
        non_boolean = deepcopy(complete_evaluations)
        non_boolean[0]['selected'] = 1
        invalid_selected_cases.append(('non-boolean-selected', non_boolean))

        for label, evaluations in invalid_selected_cases:
            with self.subTest(selected_contract=label):
                install_base_report()
                with self.assertRaises(
                    remote_node.CandidateContractError
                ) as raised:
                    node._finalize_gate_audit_report(
                        evaluation_records=evaluations,
                        selected_candidate=selected_candidate,
                        selected_pose=selected_pose,
                        plan_id='a' * 24,
                        outcome_code='PLAN_READY',
                        outcome_reason='synthetic ready plan',
                        valid_plan=True,
                    )
                self.assertEqual(
                    raised.exception.code,
                    remote_node.PLANNING_AUDIT_FAILED,
                )

        valid_evaluations = deepcopy(complete_evaluations)
        valid_evaluations[0]['selected'] = True
        install_base_report()
        node._candidate_gate_audit_row = lambda *_args: {
            'candidate_index': 0,
            'variant_index': 0,
        }
        node._finalize_gate_audit_report(
            evaluation_records=valid_evaluations,
            selected_candidate=selected_candidate,
            selected_pose=selected_pose,
            plan_id='a' * 24,
            outcome_code='PLAN_READY',
            outcome_reason='synthetic ready plan',
            valid_plan=True,
        )
        valid_report = node._active_gate_audit_report
        self.assertTrue(valid_report['outcome']['valid_plan'])
        self.assertEqual(
            [row['selected'] for row in valid_report['lineage']],
            [True, False],
        )
        self.assertEqual(valid_report['selected']['candidate_index'], 0)
        self.assertEqual(valid_report['selected']['variant_index'], 0)

    def test_bad_raw_candidate_and_good_candidate_keep_complete_two_variant_lineage(self):
        snapshot = make_snapshot(np.ones((3, 4), dtype=np.uint16) * 2200)
        bad = RemoteGraspCandidate(
            0.99,
            np.array([0.40, -0.10, 0.25]),
            np.array([0.0, 0.0, 0.0, 1.0]),
            0.04,
            depth_m=0.025,
        )
        good = RemoteGraspCandidate(
            0.80,
            np.array([0.40, -0.10, 0.25]),
            np.array([0.0, 0.0, 0.0, 1.0]),
            0.04,
            depth_m=0.03,
        )
        node = make_processing_node(RecordingClient(candidates=[bad, good]))
        node.pose_estimator = RecordingPoseEstimator()
        node._plan_reachable = lambda _pose: True
        node.orientation_variant_quaternions = [
            np.asarray([0.0, 0.0, 0.0, 1.0]),
            remote_node.quaternion_from_euler(0.0, 0.0, np.pi),
        ]
        original_estimator = remote_node.estimate_object_geometry
        remote_node.estimate_object_geometry = lambda **_kwargs: make_geometry_estimate(
            center_base=[0.40, -0.10, 0.25],
            size_xyz_m=[0.04, 0.04, 0.06],
        )
        node._snapshot_base_optical_transform = (
            lambda *_args: identity_base_camera_snapshot_transform()
        )

        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / 'mixed-audit.json'
            node.gate_audit_output_path = str(path)
            try:
                ok, message = node._process_frame(snapshot, manual=True)
            finally:
                remote_node.estimate_object_geometry = original_estimator

            self.assertTrue(ok, message)
            report = json.loads(path.read_text())
            self.assertEqual(
                [
                    (row['candidate_index'], row['variant_index'])
                    for row in report['rows']
                ],
                [(0, 0), (0, 1), (1, 0), (1, 1)],
            )
            for row in report['rows'][:2]:
                contract = row['planning_evaluation']['candidate_contract']
                self.assertFalse(contract['ok'])
                self.assertEqual(contract['failure_code'], 'DEPTH_OUT_OF_RANGE')
            self.assertEqual(report['selected']['candidate_index'], 1)
            self.assertEqual(
                node.rich_plan_pub.messages[-1].score,
                good.score,
            )

    def test_audit_internal_or_atomic_write_failure_blocks_every_valid_plan(self):
        snapshot = make_snapshot(np.ones((3, 4), dtype=np.uint16) * 2200)
        candidate = RemoteGraspCandidate(
            0.90,
            np.array([0.40, -0.10, 0.25]),
            np.array([0.0, 0.0, 0.0, 1.0]),
            0.04,
            depth_m=0.03,
        )
        original_estimator = remote_node.estimate_object_geometry
        remote_node.estimate_object_geometry = lambda **_kwargs: make_geometry_estimate(
            center_base=[0.40, -0.10, 0.25],
            size_xyz_m=[0.04, 0.04, 0.06],
        )
        try:
            with tempfile.TemporaryDirectory() as directory:
                node = make_processing_node(
                    RecordingClient(candidates=[candidate])
                )
                node.gate_audit_output_path = str(
                    pathlib.Path(directory) / 'audit-failure.json'
                )
                node._snapshot_base_optical_transform = (
                    lambda *_args: identity_base_camera_snapshot_transform()
                )
                node._run_candidate_gate_audit = lambda *_args, **_kwargs: (
                    (_ for _ in ()).throw(RuntimeError('audit exploded'))
                )

                ok, message = node._process_frame(snapshot, manual=True)

                self.assertFalse(ok)
                self.assertTrue(
                    message.startswith(remote_node.PLANNING_AUDIT_FAILED + ': '),
                    message,
                )
                self.assertTrue(
                    all(
                        not bool(getattr(item, 'valid', False))
                        for item in node.rich_plan_pub.messages
                    )
                )
                report = json.loads(
                    pathlib.Path(node.gate_audit_output_path).read_text()
                )
                self.assertEqual(
                    report['outcome']['code'],
                    remote_node.PLANNING_AUDIT_FAILED,
                )
                self.assertFalse(report['outcome']['valid_plan'])

            with tempfile.TemporaryDirectory() as directory:
                node = make_processing_node(RecordingClient(candidates=[]))
                node.gate_audit_output_path = directory
                node._snapshot_base_optical_transform = (
                    lambda *_args: identity_base_camera_snapshot_transform()
                )

                ok, message = node._process_frame(snapshot, manual=True)

                self.assertFalse(ok)
                self.assertTrue(
                    message.startswith(
                        remote_node.PLANNING_AUDIT_WRITE_FAILED + ': '
                    ),
                    message,
                )
                self.assertTrue(
                    all(
                        not bool(getattr(item, 'valid', False))
                        for item in node.rich_plan_pub.messages
                    )
                )
        finally:
            remote_node.estimate_object_geometry = original_estimator

    def test_atomic_audit_writer_rejects_non_finite_json(self):
        node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
        node.gate_audit_enabled = True
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / 'strict.json'
            node.gate_audit_output_path = str(path)

            with self.assertRaises(remote_node.CandidateContractError) as raised:
                node._write_gate_audit_report(
                    {'rows': [{'score': float('nan')}]}
                )

            self.assertEqual(
                raised.exception.code,
                remote_node.PLANNING_AUDIT_WRITE_FAILED,
            )
            self.assertFalse(path.exists())

    def test_reachability_receives_converted_opencv_optical_candidate(self):
        candidate = RemoteGraspCandidate(
            score=0.9,
            translation_m=np.array([0.01, 0.02, 0.30]),
            quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
            width_m=0.05,
            depth_m=0.03,
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
        np.testing.assert_allclose(estimator.calls[0][0], np.array([0.30, -0.04, -0.02]))
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
        self.assertAlmostEqual(array.poses[0].position.x, 0.4)
        self.assertAlmostEqual(array.poses[0].position.z, 0.12)
        self.assertAlmostEqual(array.poses[1].position.x, 0.4)
        self.assertAlmostEqual(array.poses[1].position.z, 0.185)
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
            'tool_approach_axis': 'x',
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

    def test_visibility_sequence_missing_axis_also_defaults_to_tool_plus_z(self):
        node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
        node.tf_buffer = None
        node.handeye_translation_xyz = [0.0, 0.0, 0.0]
        node.handeye_rotation_xyzw = [0.0, 0.0, 0.0, 1.0]
        node._cached_tool_from_camera = None
        node.grasp_config = {
            'pregrasp_distance_m': 0.08,
            'final_approach_offset_m': 0.02,
            'lift_height_m': 0.05,
        }
        node.camera_visibility_require_approach = False
        node.camera_visibility_margin_px = 0
        node.camera_visibility_min_depth_m = 0.0
        node.camera_visibility_max_depth_m = 2.0
        node._camera_intrinsics = lambda: remote_node.CameraIntrinsics(
            width=640,
            height=480,
            fx=440.0,
            fy=440.0,
            cx=320.0,
            cy=240.0,
            depth_scale=0.001,
        )
        pose = PoseStamped()
        pose.pose.orientation.w = 1.0
        observed = []
        original_builder = remote_node.make_grasp_sequence_from_grasp_pose

        def recording_builder(*args, **kwargs):
            observed.append(kwargs.get('tool_approach_axis'))
            return original_builder(*args, **kwargs)

        remote_node.make_grasp_sequence_from_grasp_pose = recording_builder
        try:
            _visible, metrics, _reason = node._candidate_visibility_metrics(
                pose,
                [0.5, 0.0, 0.0],
            )
        finally:
            remote_node.make_grasp_sequence_from_grasp_pose = original_builder

        self.assertEqual(observed, ['z'])
        self.assertEqual(len(metrics), 1)
        self.assertAlmostEqual(metrics[0]['depth_m'], 0.5)

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

    def test_plan_reachable_never_calls_motion_service_even_if_flags_are_relaxed(self):
        node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
        node.allow_position_only_fallback = True
        node.allow_orientation_fallback = True
        node._position_only_rejected_count = 0
        node._orientation_fallback_rejected_count = 0
        service_names = []

        original_wait_for_service = remote_node.rospy.wait_for_service
        original_service_proxy = remote_node.rospy.ServiceProxy
        remote_node.rospy.wait_for_service = (
            lambda name, **_kwargs: service_names.append(('wait', name))
        )

        def service_proxy(name, *_args, **_kwargs):
            service_names.append(('proxy', name))
            return lambda _pose, _execute: FakeServiceResponse(
                True,
                'planned: target xyz=(0.1, 0.2, 0.3)',
            )

        remote_node.rospy.ServiceProxy = service_proxy
        try:
            self.assertTrue(node._plan_reachable(PoseStamped()))
        finally:
            remote_node.rospy.wait_for_service = original_wait_for_service
            remote_node.rospy.ServiceProxy = original_service_proxy

        self.assertEqual(
            service_names,
            [
                ('wait', '/supervisor/check_pose_strict'),
                ('proxy', '/supervisor/check_pose_strict'),
            ],
        )
        self.assertNotIn(
            '/supervisor/move_to_pose',
            [name for _kind, name in service_names],
        )

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
        camera_candidate = RemoteGraspCandidate(
            score=0.9,
            translation_m=np.array([0.3, 0.0, 0.0]),
            quaternion_xyzw=remote_node.STRICT_MODEL_GRASP_TO_TOOL_QUATERNION,
            width_m=0.04,
            depth_m=0.03,
            tool0_translation_m=np.array([0.33, 0.0, 0.0]),
        )
        camera_candidate._center_base_xyz = np.array([1.01, 0.0, 0.105])
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
        camera_candidate = RemoteGraspCandidate(
            score=0.8,
            width_m=0.05,
            depth_m=0.02,
            translation_m=np.array([0.30, 0.06, 0.0]),
            quaternion_xyzw=remote_node.STRICT_MODEL_GRASP_TO_TOOL_QUATERNION,
            tool0_translation_m=np.array([0.32, 0.06, 0.0]),
        )
        camera_candidate._center_base_xyz = np.array([0.0, 0.06, 0.10])

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
        far_candidate = RemoteGraspCandidate(
            score=0.8,
            translation_m=np.zeros(3),
            quaternion_xyzw=remote_node.STRICT_MODEL_GRASP_TO_TOOL_QUATERNION,
            width_m=0.04,
            depth_m=0.03,
            tool0_translation_m=np.array([0.03, 0.0, 0.0]),
        )
        far_candidate._center_base_xyz = np.array([-0.107, -0.480, 0.081])
        original_logwarn_throttle = remote_node.rospy.logwarn_throttle
        remote_node.rospy.logwarn_throttle = lambda *args, **kwargs: None
        try:
            self.assertFalse(
                node._candidate_matches_target(None, far_candidate, far_and_low)
            )
            self.assertEqual(node._target_gate_rejected_count, 1)

            close_enough = PoseStamped()
            close_enough.pose.position.x = -0.124
            close_enough.pose.position.y = -0.503
            close_enough.pose.position.z = 0.105
            close_candidate = RemoteGraspCandidate(
                score=0.8,
                translation_m=np.zeros(3),
                quaternion_xyzw=remote_node.STRICT_MODEL_GRASP_TO_TOOL_QUATERNION,
                width_m=0.04,
                depth_m=0.03,
                tool0_translation_m=np.array([0.03, 0.0, 0.0]),
            )
            close_candidate._center_base_xyz = np.array(
                [-0.124, -0.503, 0.105]
            )
            self.assertTrue(
                node._candidate_matches_target(
                    None,
                    close_candidate,
                    close_enough,
                )
            )
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
        node.require_candidate_depth = True
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
        camera_candidate = RemoteGraspCandidate(
            score=0.517,
            translation_m=np.zeros(3),
            quaternion_xyzw=np.array(
                [
                    failed_grasp.pose.orientation.x,
                    failed_grasp.pose.orientation.y,
                    failed_grasp.pose.orientation.z,
                    failed_grasp.pose.orientation.w,
                ]
            ),
            width_m=0.078,
            depth_m=0.030,
            tool0_translation_m=np.array([0.03, 0.0, 0.0]),
        )
        camera_candidate._center_base_xyz = np.array([-0.148, -0.408, 0.070])

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
