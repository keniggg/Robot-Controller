#!/usr/bin/env python3
import json
import math
import os
import re
import socket
import threading
import time
import urllib.error
from copy import deepcopy

import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler, quaternion_from_matrix, quaternion_matrix, quaternion_multiply

try:
    import tf2_ros
except Exception:
    tf2_ros = None

from alicia_flexible_grasp.grasp.grasp6d_sequence import make_grasp_sequence_from_grasp_pose
from alicia_flexible_grasp.grasp.gripper_geometry import (
    CandidateGateResult,
    GripperGeometry,
    candidate_rank_key,
    candidate_with_motion_cost,
    evaluate_candidate,
    gripper_contract_mismatch_reason,
)
from alicia_flexible_grasp.robot.planning_feedback import (
    is_orientation_fallback_message,
    is_position_only_fallback_message,
)
from alicia_flexible_grasp.vision.grasp6d_adapter import CameraIntrinsics
from alicia_flexible_grasp.vision.model_selection import select_yolo_model
from alicia_flexible_grasp.vision.object_geometry import (
    GeometryEstimate,
    estimate_object_geometry,
)
from alicia_flexible_grasp.vision.pose_estimator import PoseEstimator
from alicia_flexible_grasp.vision.remote_grasp6d_client import RemoteGrasp6DClient, RemoteGraspCandidate
from alicia_flexible_grasp.vision.rgbd_snapshot import (
    SnapshotResult,
    SynchronizedRgbdBuffer,
    fuse_stable_samples,
)
from alicia_flexible_grasp_supervisor.msg import ObjectGeometry, ObjectPose
from alicia_flexible_grasp_supervisor.srv import SetTargetPose, TriggerZero, TriggerZeroResponse


OPTICAL_TO_ROS_CAMERA = np.asarray(
    [
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ],
    dtype=float,
)


def segment_foreground_above_support_plane(
    z_m,
    valid_mask,
    roi,
    intrinsics,
    min_points=80,
    iterations=96,
    far_percentile=55.0,
    inlier_distance_m=0.0035,
    min_height_m=0.004,
    min_inlier_ratio=0.08,
):
    """Split a tabletop object from its support plane in an RGB-D ROI."""
    z_m = np.asarray(z_m, dtype=np.float32)
    valid = np.asarray(valid_mask, dtype=bool)
    ys, xs = np.nonzero(valid)
    min_points = max(3, int(min_points))
    if len(xs) < max(min_points * 2, 12):
        return None, None, 'support_plane=insufficient-depth'

    z = z_m[ys, xs].astype(np.float64)
    x0, y0, _x1, _y1 = roi
    u = xs.astype(np.float64) + float(x0)
    v = ys.astype(np.float64) + float(y0)
    points = np.stack(
        [
            (u - float(intrinsics.cx)) * z / float(intrinsics.fx),
            (v - float(intrinsics.cy)) * z / float(intrinsics.fy),
            z,
        ],
        axis=1,
    )

    far_cutoff = float(np.percentile(z, min(90.0, max(40.0, float(far_percentile)))))
    sample_indices = np.flatnonzero(z >= far_cutoff)
    if len(sample_indices) < max(min_points, 12):
        return None, None, 'support_plane=insufficient-far-depth'

    rng = np.random.default_rng(0)
    threshold = max(0.0005, float(inlier_distance_m))
    best_inliers = None
    best_score = None
    for _ in range(max(12, int(iterations))):
        chosen = rng.choice(sample_indices, 3, replace=False)
        first, second, third = points[chosen]
        normal = np.cross(second - first, third - first)
        norm = float(np.linalg.norm(normal))
        if norm <= 1e-9:
            continue
        normal /= norm
        distances = np.abs((points - first).dot(normal))
        inliers = distances <= threshold
        count = int(np.count_nonzero(inliers))
        if count < 3:
            continue
        plane_depth = float(np.median(z[inliers]))
        if plane_depth < far_cutoff - threshold:
            continue
        score = (count, plane_depth)
        if best_score is None or score > best_score:
            best_score = score
            best_inliers = inliers

    min_inliers = max(min_points, int(math.ceil(float(min_inlier_ratio) * len(points))))
    if best_inliers is None or int(np.count_nonzero(best_inliers)) < min_inliers:
        return None, None, 'support_plane=no-dominant-plane'

    plane_points = points[best_inliers]
    plane_center = np.mean(plane_points, axis=0)
    _u, _s, vh = np.linalg.svd(plane_points - plane_center, full_matrices=False)
    normal = np.asarray(vh[-1], dtype=np.float64)
    normal /= max(float(np.linalg.norm(normal)), 1e-12)
    # Point the plane normal toward the camera origin. Positive signed distance
    # then means that a point is physically above the support surface.
    if float(np.dot(-plane_center, normal)) < 0.0:
        normal = -normal
    signed_height = (points - plane_center).dot(normal)
    foreground_points = signed_height >= max(0.0005, float(min_height_m))
    foreground_count = int(np.count_nonzero(foreground_points))
    if foreground_count < min_points:
        return None, None, 'support_plane=no-object-above-plane'

    foreground_depth = float(np.median(z[foreground_points]))
    plane_depth = float(np.median(z[np.abs(signed_height) <= threshold]))
    if plane_depth - foreground_depth < max(0.001, float(min_height_m) * 0.5):
        return None, None, 'support_plane=insufficient-depth-separation'

    mask = np.zeros_like(valid, dtype=bool)
    mask[ys[foreground_points], xs[foreground_points]] = True
    model = {
        'point_optical': plane_center.astype(float),
        'normal_optical': normal.astype(float),
        'inliers': int(np.count_nonzero(np.abs(signed_height) <= threshold)),
        'foreground': foreground_count,
        'plane_depth_m': plane_depth,
        'foreground_depth_m': foreground_depth,
    }
    diagnostic = (
        'support_plane=inliers:%d foreground:%d gap:%.3fm'
        % (model['inliers'], foreground_count, plane_depth - foreground_depth)
    )
    return mask, model, diagnostic


def resolve_grasp_backend_health(health):
    """Return GraspNet capabilities from direct or unified WSL health payloads."""
    payload = health if isinstance(health, dict) else {}
    nested = payload.get('grasp_backend')
    return nested if isinstance(nested, dict) else payload


def remote_prediction_failure_code(exception):
    """Classify WSL prediction failures by their causal transport exception."""
    current = exception
    visited = set()
    while current is not None and id(current) not in visited:
        visited.add(id(current))
        if isinstance(current, urllib.error.HTTPError):
            return 'WSL_PREDICT_FAILED'
        if isinstance(
            current,
            (
                urllib.error.URLError,
                ConnectionError,
                TimeoutError,
                socket.timeout,
            ),
        ):
            return 'WSL_UNAVAILABLE'
        current = getattr(current, '__cause__', None) or getattr(current, '__context__', None)
    return 'WSL_PREDICT_FAILED'


def normalize_candidate_frame_convention(convention):
    value = str(convention or 'opencv_optical').strip().lower()
    aliases = {
        'ros': 'ros_camera_link',
        'camera_link': 'ros_camera_link',
        'ros_link': 'ros_camera_link',
        'ros_camera': 'ros_camera_link',
        'opencv': 'opencv_optical',
        'optical': 'opencv_optical',
        'camera_optical': 'opencv_optical',
        'color_optical': 'opencv_optical',
    }
    return aliases.get(value, value)


def convert_candidate_to_camera_link(candidate, convention='opencv_optical'):
    convention = normalize_candidate_frame_convention(convention)
    if convention == 'ros_camera_link':
        return candidate
    if convention != 'opencv_optical':
        raise ValueError('unknown remote grasp candidate frame convention: %s' % convention)

    translation = OPTICAL_TO_ROS_CAMERA.dot(np.asarray(candidate.translation_m, dtype=float))
    optical_from_grasp = quaternion_matrix(np.asarray(candidate.quaternion_xyzw, dtype=float))
    camera_from_grasp = np.eye(4, dtype=float)
    camera_from_grasp[:3, :3] = OPTICAL_TO_ROS_CAMERA.dot(optical_from_grasp[:3, :3])
    quaternion = _normalize_quaternion(np.asarray(quaternion_from_matrix(camera_from_grasp), dtype=float))
    return RemoteGraspCandidate(
        score=float(candidate.score),
        translation_m=translation,
        quaternion_xyzw=quaternion,
        width_m=float(candidate.width_m),
        height_m=getattr(candidate, 'height_m', None),
        depth_m=getattr(candidate, 'depth_m', None),
    )


def align_candidate_to_tool_frame(candidate, model_grasp_to_tool_quaternion=None):
    """Convert GraspNet's grasp-frame orientation into the MoveIt tool frame."""
    correction = np.asarray(
        model_grasp_to_tool_quaternion
        if model_grasp_to_tool_quaternion is not None
        else [0.0, 0.0, 0.0, 1.0],
        dtype=float,
    )
    correction = _normalize_quaternion(correction)
    if np.allclose(correction, np.asarray([0.0, 0.0, 0.0, 1.0], dtype=float)):
        return candidate
    quaternion = _normalize_quaternion(
        np.asarray(quaternion_multiply(candidate.quaternion_xyzw, correction), dtype=float)
    )
    return RemoteGraspCandidate(
        score=float(candidate.score),
        translation_m=np.asarray(candidate.translation_m, dtype=float),
        quaternion_xyzw=quaternion,
        width_m=float(candidate.width_m),
        height_m=getattr(candidate, 'height_m', None),
        depth_m=getattr(candidate, 'depth_m', None),
    )


def summarize_candidate_gate_audit(
    rows,
    clearance_thresholds_m,
    approach_thresholds,
    baseline_clearance_m=None,
    baseline_approach_cos=None,
):
    """Summarize independent gate results for one immutable candidate batch."""
    rows = list(rows or [])
    clearance_thresholds = [float(value) for value in clearance_thresholds_m]
    approach_limits = [float(value) for value in approach_thresholds]
    base_rows = [
        row for row in rows
        if bool(row.get('depth_ok')) and bool(row.get('width_ok')) and bool(row.get('target_ok'))
    ]

    def _finite_values(key):
        values = []
        for row in base_rows:
            try:
                value = float(row.get(key))
            except (TypeError, ValueError):
                continue
            if np.isfinite(value):
                values.append(value)
        return values

    def _number(row, key, default):
        try:
            value = float(row.get(key))
        except (TypeError, ValueError):
            return float(default)
        return value if np.isfinite(value) else float(default)

    def _quantiles(key):
        values = _finite_values(key)
        if not values:
            return {}
        percentiles = (0, 10, 25, 50, 75, 90, 100)
        result = np.percentile(np.asarray(values, dtype=float), percentiles)
        return {'p%d' % percentile: float(value) for percentile, value in zip(percentiles, result)}

    profiles = []
    for clearance in clearance_thresholds:
        for approach in approach_limits:
            geometric = [
                row for row in base_rows
                if _number(row, 'finger_clearance_m', float('-inf')) >= clearance
                and _number(row, 'approach_cos', -1.0) >= approach
            ]
            profiles.append(
                {
                    'clearance_m': clearance,
                    'approach_cos': approach,
                    'pass_count': len(geometric),
                    'visible_count': sum(bool(row.get('visibility_ok')) for row in geometric),
                }
            )

    baseline_clearance = float(
        clearance_thresholds[0]
        if baseline_clearance_m is None and clearance_thresholds
        else baseline_clearance_m if baseline_clearance_m is not None else float('inf')
    )
    baseline_approach = float(
        approach_limits[0]
        if baseline_approach_cos is None and approach_limits
        else baseline_approach_cos if baseline_approach_cos is not None else float('inf')
    )
    variant_profiles = []
    variant_indices = sorted(
        set(int(row.get('variant_index', 0)) for row in rows)
    )
    for variant_index in variant_indices:
        variant_rows = [
            row for row in base_rows
            if int(row.get('variant_index', 0)) == variant_index
        ]
        safe_rows = [
            row for row in variant_rows
            if _number(row, 'finger_clearance_m', float('-inf')) >= baseline_clearance
            and _number(row, 'approach_cos', -1.0) >= baseline_approach
        ]
        variant_profiles.append(
            {
                'variant_index': int(variant_index),
                'target_count': len(variant_rows),
                'safe_count': len(safe_rows),
                'visible_count': sum(bool(row.get('visibility_ok')) for row in safe_rows),
            }
        )

    return {
        'candidate_variants': len(rows),
        'depth_pass': sum(bool(row.get('depth_ok')) for row in rows),
        'width_pass': sum(bool(row.get('width_ok')) for row in rows),
        'target_pass': len(base_rows),
        'profiles': profiles,
        'variant_profiles': variant_profiles,
        'finger_clearance_quantiles_m': _quantiles('finger_clearance_m'),
        'center_clearance_quantiles_m': _quantiles('center_clearance_m'),
        'jaw_normal_quantiles': _quantiles('jaw_normal_cos'),
        'approach_quantiles': _quantiles('approach_cos'),
        'cloud_distance_quantiles_m': _quantiles('cloud_distance_m'),
    }


def _normalize_quaternion(quaternion):
    norm = float(np.linalg.norm(quaternion))
    if norm <= 1e-12:
        raise ValueError('remote grasp candidate quaternion has zero norm')
    quaternion = quaternion / norm
    if quaternion[3] < 0.0:
        quaternion = -quaternion
    return quaternion


def expanded_bbox_roi(image_shape, bbox_x, bbox_y, bbox_width, bbox_height, margin_px=0):
    height, width = image_shape[:2]
    margin = max(0, int(margin_px))
    x0 = max(0, int(bbox_x) - margin)
    y0 = max(0, int(bbox_y) - margin)
    x1 = min(int(width), int(bbox_x) + max(0, int(bbox_width)) + margin)
    y1 = min(int(height), int(bbox_y) + max(0, int(bbox_height)) + margin)
    if x1 <= x0 or y1 <= y0:
        raise ValueError('invalid target ROI bbox')
    return x0, y0, x1, y1


def mask_depth_to_roi(depth, roi):
    x0, y0, x1, y1 = roi
    masked = np.zeros_like(depth)
    masked[y0:y1, x0:x1] = depth[y0:y1, x0:x1]
    return masked


def valid_depth_count(depth):
    values = np.asarray(depth)
    if np.issubdtype(values.dtype, np.floating):
        return int(np.count_nonzero(np.isfinite(values) & (values > 0.0)))
    return int(np.count_nonzero(values > 0))


def transform_matrix(translation_xyz, quaternion_xyzw):
    matrix = quaternion_matrix([float(value) for value in quaternion_xyzw])
    matrix[:3, 3] = np.asarray(translation_xyz, dtype=float).reshape(3)
    return matrix


def pose_matrix(pose_stamped):
    pose = pose_stamped.pose
    return transform_matrix(
        [pose.position.x, pose.position.y, pose.position.z],
        [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
    )


def project_base_target_at_tool_pose(tool_pose, target_base_xyz, tool_from_camera, intrinsics):
    base_from_camera = pose_matrix(tool_pose).dot(np.asarray(tool_from_camera, dtype=float).reshape(4, 4))
    camera_from_base = np.linalg.inv(base_from_camera)
    target_base = np.ones(4, dtype=float)
    target_base[:3] = np.asarray(target_base_xyz, dtype=float).reshape(3)
    target_camera = camera_from_base.dot(target_base)[:3]

    # camera_link follows ROS convention: x forward, y left, z up.
    optical_z = float(target_camera[0])
    if optical_z <= 1e-9:
        return float('nan'), float('nan'), optical_z
    optical_x = -float(target_camera[1])
    optical_y = -float(target_camera[2])
    u = float(intrinsics.cx) + float(intrinsics.fx) * optical_x / optical_z
    v = float(intrinsics.cy) + float(intrinsics.fy) * optical_y / optical_z
    return u, v, optical_z


def make_remote_pose_estimator(cam_cfg, hcfg, gcfg, tf2_module=None):
    tf_module = tf2_module if tf2_module is not None else tf2_ros
    tf_buffer = None
    tf_listener = None
    if bool(hcfg.get('use_tf', True)) and tf_module is not None:
        tf_buffer = tf_module.Buffer()
        tf_listener = tf_module.TransformListener(tf_buffer)
    elif bool(hcfg.get('use_tf', True)):
        rospy.logwarn('tf2_ros is unavailable; remote 6D grasp will use configured static handeye transform')

    pose_estimator = PoseEstimator(
        hcfg.get('camera_frame', cam_cfg.get('frame_id', 'camera_link')),
        hcfg.get('base_frame', 'base_link'),
        hcfg.get('translation_xyz', [0.0, 0.0, 0.0]),
        hcfg.get('rotation_xyzw', [0.0, 0.0, 0.0, 1.0]),
        gcfg.get('default_orientation_xyzw', [0.0, 0.7071, 0.0, 0.7071]),
        tf_buffer=tf_buffer,
        tf_timeout_sec=hcfg.get('tf_timeout_sec', 0.2),
        tf_lookup_latest=hcfg.get('tf_lookup_latest', True),
        allow_static_fallback=hcfg.get('allow_static_fallback', True),
    )
    return pose_estimator, tf_buffer, tf_listener


def select_first_reachable_candidate(
    candidates,
    pose_estimator,
    reachability_fn,
    stamp,
    camera_frame,
    candidate_frame_convention='opencv_optical',
    candidate_filter_fn=None,
    candidate_rank_fn=None,
    orientation_variant_quaternions=None,
    model_grasp_to_tool_quaternion=None,
    candidate_geometry_fn=None,
    grasp_config=None,
):
    variants = list(orientation_variant_quaternions or [])
    if not variants:
        variants = [np.asarray([0.0, 0.0, 0.0, 1.0], dtype=float)]
    ranked = []
    for candidate in candidates:
        camera_candidate = convert_candidate_to_camera_link(candidate, candidate_frame_convention)
        camera_candidate = align_candidate_to_tool_frame(
            camera_candidate,
            model_grasp_to_tool_quaternion,
        )
        for variant_index, correction in enumerate(variants):
            if np.allclose(np.asarray(correction, dtype=float), np.asarray([0.0, 0.0, 0.0, 1.0], dtype=float)):
                variant_candidate = camera_candidate
            else:
                variant_quat = _normalize_quaternion(
                    np.asarray(quaternion_multiply(camera_candidate.quaternion_xyzw, correction), dtype=float)
                )
                variant_candidate = RemoteGraspCandidate(
                    score=float(camera_candidate.score),
                    translation_m=np.asarray(camera_candidate.translation_m, dtype=float),
                    quaternion_xyzw=variant_quat,
                    width_m=float(camera_candidate.width_m),
                    height_m=getattr(camera_candidate, 'height_m', None),
                    depth_m=getattr(camera_candidate, 'depth_m', None),
                )
            pose = pose_estimator.make_base_pose_from_camera_pose(
                variant_candidate.translation_m,
                variant_candidate.quaternion_xyzw,
                stamp=stamp,
                camera_frame=camera_frame,
            )
            gate_result = None
            if candidate_geometry_fn is not None:
                config = dict(grasp_config or {})
                plan = make_grasp_sequence_from_grasp_pose(
                    pose,
                    pregrasp_distance_m=float(
                        config.get('pregrasp_distance_m', 0.08)
                    ),
                    approach_offset_m=float(
                        config.get('final_approach_offset_m', 0.015)
                    ),
                    lift_height_m=float(
                        config.get('lift_height_m', 0.05)
                    ),
                    tool_approach_axis=str(
                        config.get('tool_approach_axis', 'x')
                    ),
                )
                gate_result = candidate_geometry_fn(
                    candidate,
                    variant_candidate,
                    pose,
                    plan,
                )
                if not isinstance(gate_result, CandidateGateResult):
                    raise ValueError(
                        'candidate_geometry_fn must return CandidateGateResult'
                    )
                setattr(variant_candidate, '_geometry_gate_result', gate_result)
                setattr(variant_candidate, '_grasp_sequence', plan)
                setattr(
                    variant_candidate,
                    'required_open_width_m',
                    float(gate_result.required_open_width_m),
                )
                if not gate_result.ok:
                    continue
            if candidate_filter_fn is not None and not bool(candidate_filter_fn(candidate, variant_candidate, pose)):
                continue
            if bool(reachability_fn(pose)):
                if candidate_rank_fn is None and gate_result is None:
                    return variant_candidate, pose
                if candidate_rank_fn is None:
                    rank = candidate_rank_key(
                        gate_result,
                        float(getattr(variant_candidate, 'score', 0.0)),
                    )
                else:
                    rank = candidate_rank_fn(
                        candidate,
                        variant_candidate,
                        pose,
                    )
                if np.isscalar(rank):
                    rank = (float(rank),)
                else:
                    rank = tuple(float(value) for value in rank)
                if not rank or not all(math.isfinite(value) for value in rank):
                    continue
                ranked.append(
                    (
                        rank,
                        variant_index,
                        -float(getattr(variant_candidate, 'score', 0.0)),
                        variant_candidate,
                        pose,
                    )
                )
    if ranked:
        ranked.sort(key=lambda item: (item[0], item[1], item[2]))
        return ranked[0][3], ranked[0][4]
    return None, None


def make_grasp_plan_pose_array(grasp_pose, stamp, grasp_config):
    plan = make_grasp_sequence_from_grasp_pose(
        grasp_pose,
        pregrasp_distance_m=float(grasp_config.get('pregrasp_distance_m', 0.08)),
        approach_offset_m=float(grasp_config.get('final_approach_offset_m', 0.015)),
        lift_height_m=float(grasp_config.get('lift_height_m', 0.05)),
        tool_approach_axis=str(grasp_config.get('tool_approach_axis', 'x')),
    )
    msg = PoseArray()
    msg.header.frame_id = grasp_pose.header.frame_id
    msg.header.stamp = stamp if stamp is not None else rospy.Time(0)
    msg.poses = [plan.pregrasp.pose, plan.approach.pose, plan.grasp.pose, plan.lift.pose]
    return msg


def geometry_estimate_to_message(estimate, snapshot=None, stamp=None, label=''):
    """Map one immutable geometry estimate into the base-frame ROS contract."""
    message = ObjectGeometry()
    message.header.frame_id = 'base_link'
    message.header.stamp = stamp if stamp is not None else rospy.Time(0)
    message.valid = bool(getattr(estimate, 'ok', False))
    message.label = str(label or '')
    message.source_mode = str(
        getattr(estimate, 'source_mode', '')
        or getattr(snapshot, 'source_mode', '')
        or ''
    )
    center = np.asarray(getattr(estimate, 'center_base', np.zeros(3)), dtype=float).reshape(3)
    axes = np.asarray(getattr(estimate, 'axes_base', np.eye(3)), dtype=float).reshape(3, 3)
    size = np.asarray(getattr(estimate, 'size_xyz_m', np.zeros(3)), dtype=float).reshape(3)
    normal = np.asarray(
        getattr(estimate, 'support_normal_base', np.zeros(3)),
        dtype=float,
    ).reshape(3)
    if message.valid:
        rotation = np.eye(4, dtype=float)
        rotation[:3, :3] = axes
        quaternion = _normalize_quaternion(
            np.asarray(quaternion_from_matrix(rotation), dtype=float)
        )
    else:
        quaternion = np.asarray([0.0, 0.0, 0.0, 1.0], dtype=float)
    message.pose_base.position.x = float(center[0])
    message.pose_base.position.y = float(center[1])
    message.pose_base.position.z = float(center[2])
    message.pose_base.orientation.x = float(quaternion[0])
    message.pose_base.orientation.y = float(quaternion[1])
    message.pose_base.orientation.z = float(quaternion[2])
    message.pose_base.orientation.w = float(quaternion[3])
    message.size_xyz_m.x = float(size[0])
    message.size_xyz_m.y = float(size[1])
    message.size_xyz_m.z = float(size[2])
    message.support_normal_base.x = float(normal[0])
    message.support_normal_base.y = float(normal[1])
    message.support_normal_base.z = float(normal[2])
    message.support_offset_m = float(getattr(estimate, 'support_offset_m', 0.0))
    quality = getattr(snapshot, 'quality', None)
    message.valid_depth_points = int(getattr(quality, 'valid_depth_points', 0))
    message.valid_depth_ratio = float(getattr(quality, 'valid_depth_ratio', 0.0))
    message.depth_mad_m = float(getattr(quality, 'depth_mad_m', 0.0))
    message.fused_frames = int(getattr(quality, 'fused_frames', 0))
    message.support_inlier_ratio = float(
        getattr(estimate, 'support_inlier_ratio', 0.0)
    )
    object_points = np.asarray(
        getattr(estimate, 'object_points_base', np.zeros((0, 3))),
        dtype=float,
    )
    message.object_point_count = int(len(object_points.reshape(-1, 3)))
    if message.valid:
        message.failure_reason = ''
    else:
        code = str(getattr(estimate, 'failure_code', '') or 'OBB_INVALID')
        reason = str(getattr(estimate, 'failure_reason', '') or 'geometry is invalid')
        message.failure_reason = '%s: %s' % (code, reason)
    return message


class RemoteGrasp6DNode:
    def __init__(self):
        self.enabled = bool(rospy.get_param('/grasp_6d/enabled', True))
        self.bridge = CvBridge()
        self.frames = SynchronizedRgbdBuffer(source_clock_ns=self._ros_source_clock_ns)
        self.latest_object = None
        self.latest_object_time = None
        self._planning_snapshot_active = False
        self._planning_object_msg = None
        self._planning_object_time = None
        self._object_lock = threading.Lock()
        self._geometry_state_lock = threading.RLock()
        self.last_error = ''
        self._request_lock = threading.Lock()
        self._backoff_until = rospy.Time(0)
        self.plan_pub = rospy.Publisher(rospy.get_param('/grasp/grasp6d_plan_topic', '/grasp_6d/plan'), PoseArray, queue_size=1)
        self.status_pub = rospy.Publisher('/grasp_6d/status', String, queue_size=1, latch=True)
        self.gate_audit_pub = rospy.Publisher('/grasp_6d/gate_audit', String, queue_size=1, latch=True)
        self.geometry_pub = rospy.Publisher(
            '/grasp_6d/object_geometry',
            ObjectGeometry,
            queue_size=1,
            latch=True,
        )
        self.previous_object_axes_base = None
        self.latest_object_geometry = None
        self._geometry_invalidation_generation = 0
        self._last_geometry_invalidation_code = ''

        cam_cfg = rospy.get_param('/camera', {})
        pcfg = rospy.get_param('/perception', {})
        hcfg = rospy.get_param('/handeye', {})
        gcfg = rospy.get_param('/grasp', {})
        remote_cfg = rospy.get_param('/grasp_6d/remote', {})
        gripper_cfg = rospy.get_param('/gripper', {})
        twin_cfg = rospy.get_param('/mujoco_digital_twin', {})
        gripper_geometry_cfg = dict(remote_cfg.get('gripper_geometry', {}) or {})
        self.gripper_geometry = GripperGeometry(
            max_inner_gap_m=float(
                gripper_geometry_cfg.get('max_inner_gap_m', 0.050)
            ),
            jaw_clearance_each_side_m=float(
                gripper_geometry_cfg.get(
                    'width_safety_margin_per_side_m',
                    0.002,
                )
            ),
            finger_size_xyz_m=np.asarray(
                gripper_geometry_cfg.get(
                    'finger_box_xyz_m',
                    [0.0434, 0.0286, 0.0600],
                ),
                dtype=float,
            ),
            palm_size_xyz_m=np.asarray(
                gripper_geometry_cfg.get(
                    'palm_box_xyz_m',
                    [0.1175, 0.1550, 0.0774],
                ),
                dtype=float,
            ),
            support_clearance_m=float(
                gripper_geometry_cfg.get('support_clearance_m', 0.003)
            ),
        )
        self.gripper_tool_jaw_axis = str(
            gripper_geometry_cfg.get('tool_jaw_axis', 'y')
        )
        self.gripper_tool_finger_length_axis = str(
            gripper_geometry_cfg.get('tool_finger_length_axis', 'z')
        )
        twin_gripper_cfg = dict(twin_cfg.get('gripper_model', {}) or {})
        self.twin_gripper_model_name = str(
            twin_gripper_cfg.get(
                'name',
                'Alicia_D_v5_6_gripper_50mm',
            )
        )
        self.twin_max_inner_gap_m = float(
            twin_gripper_cfg.get('max_inner_gap_m', 0.050)
        )
        self._latest_geometry_estimate = None
        self._geometry_gate_counts = {}
        self._geometry_rejection_counts = {}
        self._selected_candidate_gate = None
        self.selected_required_open_width_m = None
        self._last_model_choice = str(pcfg.get('yolo_model_choice', 'original'))
        self.planning_snapshot_frames = max(
            1,
            int(
                rospy.get_param(
                    '/grasp_6d/remote/planning_snapshot_frames',
                    remote_cfg.get('planning_snapshot_frames', 3),
                )
            ),
        )
        self.planning_snapshot_timeout_sec = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/planning_snapshot_timeout_sec',
                    remote_cfg.get('planning_snapshot_timeout_sec', 1.0),
                )
            ),
        )
        self.planning_snapshot_max_age_sec = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/planning_snapshot_max_age_sec',
                    remote_cfg.get('planning_snapshot_max_age_sec', 0.35),
                )
            ),
        )
        self.planning_mask_min_iou = float(
            rospy.get_param(
                '/grasp_6d/remote/planning_mask_min_iou',
                remote_cfg.get('planning_mask_min_iou', 0.85),
            )
        )
        self.planning_mask_max_centroid_shift_px = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/planning_mask_max_centroid_shift_px',
                    remote_cfg.get('planning_mask_max_centroid_shift_px', 5.0),
                )
            ),
        )
        self.planning_max_joint_delta_rad = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/planning_max_joint_delta_rad',
                    remote_cfg.get('planning_max_joint_delta_rad', 0.01),
                )
            ),
        )
        self.mask_erosion_px = max(
            0,
            int(
                rospy.get_param(
                    '/grasp_6d/remote/mask_erosion_px',
                    remote_cfg.get('mask_erosion_px', 2),
                )
            ),
        )
        self.mask_internal_hole_max_area_px = max(
            0,
            int(
                rospy.get_param(
                    '/grasp_6d/remote/mask_internal_hole_max_area_px',
                    remote_cfg.get('mask_internal_hole_max_area_px', 25),
                )
            ),
        )
        self.depth_mad_scale = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/depth_mad_scale',
                    remote_cfg.get('depth_mad_scale', 3.5),
                )
            ),
        )
        self.depth_mad_absolute_floor_m = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/depth_mad_absolute_floor_m',
                    remote_cfg.get('depth_mad_absolute_floor_m', 0.002),
                )
            ),
        )
        self.geometry_support_bbox_expand_ratio = max(
            0.0,
            float(remote_cfg.get('support_bbox_expand_ratio', 0.30)),
        )
        self.geometry_support_distance_threshold_m = max(
            0.0001,
            float(
                remote_cfg.get(
                    'support_distance_threshold_m',
                    remote_cfg.get('target_cloud_support_plane_inlier_distance_m', 0.004),
                )
            ),
        )
        self.geometry_voxel_size_m = max(
            0.0001,
            float(remote_cfg.get('target_cloud_voxel_size_m', 0.0025)),
        )
        self.geometry_outlier_neighbors = max(
            1,
            int(remote_cfg.get('target_cloud_outlier_neighbors', 16)),
        )
        self.geometry_outlier_std_ratio = max(
            0.0,
            float(remote_cfg.get('target_cloud_outlier_std_ratio', 2.0)),
        )
        self.geometry_min_support_points = max(
            3,
            int(remote_cfg.get('geometry_min_support_points', 200)),
        )
        self.geometry_min_object_points = max(
            3,
            int(
                remote_cfg.get(
                    'geometry_min_object_points',
                    remote_cfg.get('target_cloud_min_points', 120),
                )
            ),
        )
        self.geometry_min_size_m = max(
            0.0001,
            float(remote_cfg.get('geometry_min_size_m', 0.005)),
        )
        self.geometry_max_size_m = max(
            self.geometry_min_size_m,
            float(remote_cfg.get('geometry_max_size_m', 0.600)),
        )
        self.geometry_max_height_m = max(
            self.geometry_min_size_m,
            float(remote_cfg.get('geometry_max_height_m', 0.500)),
        )
        self.gate_audit_enabled = bool(remote_cfg.get('gate_audit_enabled', True))
        self.gate_audit_output_path = os.path.expanduser(
            str(remote_cfg.get('gate_audit_output_path', '~/.ros/grasp6d_gate_audit_latest.json'))
        )
        self._latest_gate_audit_summary = {}
        self.grasp_config = dict(gcfg or {})
        self.handeye_parent_frame = str(hcfg.get('parent_frame', 'tool0'))
        self.handeye_camera_frame = str(hcfg.get('camera_frame', cam_cfg.get('frame_id', 'camera_link')))
        self.handeye_translation_xyz = list(hcfg.get('translation_xyz', [0.0, 0.0, 0.0]))
        self.handeye_rotation_xyzw = list(hcfg.get('rotation_xyzw', [0.0, 0.0, 0.0, 1.0]))
        self._cached_tool_from_camera = None
        server_url = rospy.get_param('/grasp_6d/remote/server_url', remote_cfg.get('server_url', 'http://172.23.132.97:8000'))
        timeout_sec = float(rospy.get_param('/grasp_6d/remote/timeout_sec', remote_cfg.get('timeout_sec', 3.0)))
        self.max_candidates = int(rospy.get_param('/grasp_6d/remote/max_candidates', remote_cfg.get('max_candidates', 20)))
        self.auto_request = bool(rospy.get_param('/grasp_6d/remote/auto_request', remote_cfg.get('auto_request', False)))
        self.failure_backoff_sec = max(0.0, float(rospy.get_param('/grasp_6d/remote/failure_backoff_sec', remote_cfg.get('failure_backoff_sec', 8.0))))
        self.candidate_frame_convention = normalize_candidate_frame_convention(
            rospy.get_param('/grasp_6d/remote/candidate_frame_convention', remote_cfg.get('candidate_frame_convention', 'opencv_optical'))
        )
        self.orientation_variant_quaternions = self._parse_orientation_variant_quaternions(
            rospy.get_param('/grasp_6d/remote/orientation_variants_rpy_deg', remote_cfg.get('orientation_variants_rpy_deg', [[0.0, 0.0, 0.0]]))
        )
        self.model_grasp_to_tool_quaternion = self._parse_orientation_variant_quaternions(
            [rospy.get_param(
                '/grasp_6d/remote/model_grasp_to_tool_rpy_deg',
                remote_cfg.get('model_grasp_to_tool_rpy_deg', [0.0, 0.0, 0.0]),
            )]
        )[0]
        self.require_candidate_depth = bool(
            rospy.get_param(
                '/grasp_6d/remote/require_candidate_depth',
                remote_cfg.get('require_candidate_depth', False),
            )
        )
        self.allow_position_only_fallback = bool(
            rospy.get_param(
                '/grasp_6d/remote/accept_position_only_fallback',
                remote_cfg.get(
                    'accept_position_only_fallback',
                    rospy.get_param('/robot/position_only_execute_enabled', False),
                ),
            )
        )
        self.allow_orientation_fallback = bool(
            rospy.get_param(
                '/grasp_6d/remote/accept_orientation_fallback',
                remote_cfg.get('accept_orientation_fallback', False),
            )
        )
        self._position_only_rejected_count = 0
        self._orientation_fallback_rejected_count = 0
        self.use_perception_roi = bool(
            rospy.get_param('/grasp_6d/remote/use_perception_roi', remote_cfg.get('use_perception_roi', True))
        )
        self.require_perception_roi = bool(
            rospy.get_param('/grasp_6d/remote/require_perception_roi', remote_cfg.get('require_perception_roi', True))
        )
        self.roi_margin_px = int(
            rospy.get_param('/grasp_6d/remote/perception_roi_margin_px', remote_cfg.get('perception_roi_margin_px', 20))
        )
        self.roi_max_age_sec = max(
            0.0,
            float(rospy.get_param('/grasp_6d/remote/perception_roi_max_age_sec', remote_cfg.get('perception_roi_max_age_sec', 1.0))),
        )
        self.roi_min_valid_depth_px = max(
            1,
            int(rospy.get_param('/grasp_6d/remote/perception_roi_min_valid_depth_px', remote_cfg.get('perception_roi_min_valid_depth_px', 250))),
        )
        self.candidate_target_gate_enabled = bool(
            rospy.get_param(
                '/grasp_6d/remote/candidate_target_gate_enabled',
                remote_cfg.get('candidate_target_gate_enabled', True),
            )
        )
        self.camera_visibility_gate_enabled = bool(
            rospy.get_param(
                '/grasp_6d/remote/camera_visibility_gate_enabled',
                remote_cfg.get('camera_visibility_gate_enabled', True),
            )
        )
        self.camera_visibility_diagnostic_enabled = bool(
            rospy.get_param(
                '/grasp_6d/remote/camera_visibility_diagnostic_enabled',
                remote_cfg.get('camera_visibility_diagnostic_enabled', True),
            )
        )
        self.camera_visibility_require_approach = bool(
            rospy.get_param(
                '/grasp_6d/remote/camera_visibility_require_approach',
                remote_cfg.get('camera_visibility_require_approach', True),
            )
        )
        self.camera_visibility_margin_px = max(
            0,
            int(
                rospy.get_param(
                    '/grasp_6d/remote/camera_visibility_margin_px',
                    remote_cfg.get('camera_visibility_margin_px', 36),
                )
            ),
        )
        self.camera_visibility_bbox_padding_px = max(
            0,
            int(
                rospy.get_param(
                    '/grasp_6d/remote/camera_visibility_bbox_padding_px',
                    remote_cfg.get('camera_visibility_bbox_padding_px', 8),
                )
            ),
        )
        self.camera_visibility_min_depth_m = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/camera_visibility_min_depth_m',
                    remote_cfg.get('camera_visibility_min_depth_m', 0.035),
                )
            ),
        )
        self.camera_visibility_max_depth_m = max(
            self.camera_visibility_min_depth_m,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/camera_visibility_max_depth_m',
                    remote_cfg.get('camera_visibility_max_depth_m', 1.20),
                )
            ),
        )
        self.camera_visibility_rank_weight_m = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/camera_visibility_rank_weight_m',
                    remote_cfg.get('camera_visibility_rank_weight_m', 0.012),
                )
            ),
        )
        self.rank_by_target_distance = bool(
            rospy.get_param(
                '/grasp_6d/remote/rank_by_target_distance',
                remote_cfg.get('rank_by_target_distance', True),
            )
        )
        self.target_position_refine_enabled = bool(
            rospy.get_param(
                '/grasp_6d/remote/target_position_refine_enabled',
                remote_cfg.get('target_position_refine_enabled', False),
            )
        )
        self.target_position_refine_blend = self._clamp01(
            rospy.get_param(
                '/grasp_6d/remote/target_position_refine_blend',
                remote_cfg.get('target_position_refine_blend', 0.0),
            )
        )
        self.target_position_refine_max_m = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/target_position_refine_max_m',
                    remote_cfg.get('target_position_refine_max_m', 0.04),
                )
            ),
        )
        self.target_position_refine_max_age_sec = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/target_position_refine_max_age_sec',
                    remote_cfg.get('target_position_refine_max_age_sec', 1.0),
                )
            ),
        )
        self.target_position_refine_offset_xyz_m = self._parse_xyz_param(
            rospy.get_param(
                '/grasp_6d/remote/target_position_refine_offset_xyz_m',
                remote_cfg.get('target_position_refine_offset_xyz_m', [0.0, 0.0, 0.0]),
            )
        )
        self.target_cloud_enabled = bool(
            rospy.get_param('/grasp_6d/remote/target_cloud_enabled', remote_cfg.get('target_cloud_enabled', True))
        )
        self.target_cloud_roi_margin_px = int(
            rospy.get_param('/grasp_6d/remote/target_cloud_roi_margin_px', remote_cfg.get('target_cloud_roi_margin_px', 0))
        )
        self.target_cloud_foreground_percentile = self._clamp_range(
            rospy.get_param(
                '/grasp_6d/remote/target_cloud_foreground_percentile',
                remote_cfg.get('target_cloud_foreground_percentile', 35.0),
            ),
            1.0,
            95.0,
        )
        self.target_cloud_depth_window_m = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/target_cloud_depth_window_m',
                    remote_cfg.get('target_cloud_depth_window_m', 0.055),
                )
            ),
        )
        self.target_cloud_min_points = max(
            1,
            int(rospy.get_param('/grasp_6d/remote/target_cloud_min_points', remote_cfg.get('target_cloud_min_points', 80))),
        )
        self.target_cloud_max_age_sec = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/target_cloud_max_age_sec',
                    remote_cfg.get('target_cloud_max_age_sec', 1.0),
                )
            ),
        )
        self.target_cloud_max_points_for_gate = max(
            1,
            int(
                rospy.get_param(
                    '/grasp_6d/remote/target_cloud_max_points_for_gate',
                    remote_cfg.get('target_cloud_max_points_for_gate', 2500),
                )
            ),
        )
        self.target_cloud_candidate_max_point_distance_m = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/target_cloud_candidate_max_point_distance_m',
                    remote_cfg.get('target_cloud_candidate_max_point_distance_m', 0.055),
                )
            ),
        )
        self.target_cloud_support_plane_enabled = bool(
            rospy.get_param(
                '/grasp_6d/remote/target_cloud_support_plane_enabled',
                remote_cfg.get(
                    'target_cloud_support_plane_enabled',
                    getattr(self, 'target_cloud_support_plane_enabled', True),
                ),
            )
        )
        self.target_cloud_support_plane_ransac_iterations = max(
            12,
            int(
                rospy.get_param(
                    '/grasp_6d/remote/target_cloud_support_plane_ransac_iterations',
                    remote_cfg.get(
                        'target_cloud_support_plane_ransac_iterations',
                        getattr(self, 'target_cloud_support_plane_ransac_iterations', 96),
                    ),
                )
            ),
        )
        self.target_cloud_support_plane_far_percentile = self._clamp_range(
            rospy.get_param(
                '/grasp_6d/remote/target_cloud_support_plane_far_percentile',
                remote_cfg.get(
                    'target_cloud_support_plane_far_percentile',
                    getattr(self, 'target_cloud_support_plane_far_percentile', 55.0),
                ),
            ),
            40.0,
            90.0,
        )
        self.target_cloud_support_plane_inlier_distance_m = max(
            0.0005,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/target_cloud_support_plane_inlier_distance_m',
                    remote_cfg.get(
                        'target_cloud_support_plane_inlier_distance_m',
                        getattr(self, 'target_cloud_support_plane_inlier_distance_m', 0.0035),
                    ),
                )
            ),
        )
        self.target_cloud_support_plane_min_height_m = max(
            0.0005,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/target_cloud_support_plane_min_height_m',
                    remote_cfg.get(
                        'target_cloud_support_plane_min_height_m',
                        getattr(self, 'target_cloud_support_plane_min_height_m', 0.004),
                    ),
                )
            ),
        )
        self.target_cloud_support_plane_min_inlier_ratio = self._clamp_range(
            rospy.get_param(
                '/grasp_6d/remote/target_cloud_support_plane_min_inlier_ratio',
                remote_cfg.get(
                    'target_cloud_support_plane_min_inlier_ratio',
                    getattr(self, 'target_cloud_support_plane_min_inlier_ratio', 0.08),
                ),
            ),
            0.01,
            0.9,
        )
        self.target_cloud_candidate_min_support_clearance_m = float(
            rospy.get_param(
                '/grasp_6d/remote/target_cloud_candidate_min_support_clearance_m',
                remote_cfg.get(
                    'target_cloud_candidate_min_support_clearance_m',
                    getattr(self, 'target_cloud_candidate_min_support_clearance_m', -0.002),
                ),
            )
        )
        self.target_projection_frame_convention = normalize_candidate_frame_convention(
            rospy.get_param(
                '/grasp_6d/remote/target_projection_frame_convention',
                remote_cfg.get('target_projection_frame_convention', 'ros_camera_link'),
            )
        )
        self.latest_target_cloud_base_xyz = None
        self.latest_target_cloud_camera_center = None
        self.latest_target_cloud_camera_points = None
        self.latest_target_cloud_time = None
        self.latest_target_cloud_count = 0
        self.latest_target_cloud_source = 'none'
        self.latest_support_plane_camera_point = None
        self.latest_support_plane_camera_normal = None
        self.latest_target_cloud_segmentation = 'depth-window'
        self._target_cloud_request_active = False
        self.target_cloud_support_plane_enabled = bool(
            rospy.get_param(
                '/grasp_6d/remote/target_cloud_support_plane_enabled',
                remote_cfg.get('target_cloud_support_plane_enabled', True),
            )
        )
        self.target_cloud_support_plane_context_margin_px = max(
            0,
            int(
                rospy.get_param(
                    '/grasp_6d/remote/target_cloud_support_plane_context_margin_px',
                    remote_cfg.get('target_cloud_support_plane_context_margin_px', 24),
                )
            ),
        )
        self.target_cloud_support_plane_ransac_iterations = max(
            12,
            int(
                rospy.get_param(
                    '/grasp_6d/remote/target_cloud_support_plane_ransac_iterations',
                    remote_cfg.get('target_cloud_support_plane_ransac_iterations', 96),
                )
            ),
        )
        self.target_cloud_support_plane_far_percentile = self._clamp_range(
            rospy.get_param(
                '/grasp_6d/remote/target_cloud_support_plane_far_percentile',
                remote_cfg.get('target_cloud_support_plane_far_percentile', 55.0),
            ),
            40.0,
            90.0,
        )
        self.target_cloud_support_plane_inlier_distance_m = max(
            0.0005,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/target_cloud_support_plane_inlier_distance_m',
                    remote_cfg.get('target_cloud_support_plane_inlier_distance_m', 0.0035),
                )
            ),
        )
        self.target_cloud_support_plane_min_height_m = max(
            0.0005,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/target_cloud_support_plane_min_height_m',
                    remote_cfg.get('target_cloud_support_plane_min_height_m', 0.004),
                )
            ),
        )
        self.target_cloud_support_plane_min_inlier_ratio = self._clamp_range(
            rospy.get_param(
                '/grasp_6d/remote/target_cloud_support_plane_min_inlier_ratio',
                remote_cfg.get('target_cloud_support_plane_min_inlier_ratio', 0.08),
            ),
            0.01,
            0.9,
        )
        self.target_cloud_candidate_min_support_clearance_m = float(
            rospy.get_param(
                '/grasp_6d/remote/target_cloud_candidate_min_support_clearance_m',
                remote_cfg.get('target_cloud_candidate_min_support_clearance_m', -0.002),
            )
        )
        self.candidate_max_target_distance_m = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/candidate_max_target_distance_m',
                    remote_cfg.get('candidate_max_target_distance_m', 0.04),
                )
            ),
        )
        self.candidate_min_relative_z_m = float(
            rospy.get_param(
                '/grasp_6d/remote/candidate_min_relative_z_m',
                remote_cfg.get('candidate_min_relative_z_m', -0.015),
            )
        )
        self.candidate_max_relative_z_m = float(
            rospy.get_param(
                '/grasp_6d/remote/candidate_max_relative_z_m',
                remote_cfg.get('candidate_max_relative_z_m', 0.08),
            )
        )
        self.candidate_center_distance_weight = max(
            0.0,
            float(remote_cfg.get('candidate_center_distance_weight', 1.0)),
        )
        self.candidate_model_score_weight_m = max(
            0.0,
            float(remote_cfg.get('candidate_model_score_weight_m', 0.015)),
        )
        self.candidate_joint_path_cost_weight_m = max(
            0.0,
            float(remote_cfg.get('candidate_joint_path_cost_weight_m', 0.004)),
        )
        self.candidate_downward_approach_weight_m = max(
            0.0,
            float(remote_cfg.get('candidate_downward_approach_weight_m', 0.020)),
        )
        self.candidate_min_downward_approach_cos = float(
            remote_cfg.get('candidate_min_downward_approach_cos', 0.55)
        )
        self.candidate_max_jaw_normal_cos = self._clamp_range(
            remote_cfg.get('candidate_max_jaw_normal_cos', 0.35),
            0.0,
            1.0,
        )
        self.candidate_jaw_tilt_rank_weight_m = max(
            0.0,
            float(remote_cfg.get('candidate_jaw_tilt_rank_weight_m', 0.010)),
        )
        self.candidate_min_finger_support_clearance_m = float(
            remote_cfg.get('candidate_min_finger_support_clearance_m', 0.003)
        )
        self.candidate_max_joint_delta_rad = max(
            0.0,
            float(remote_cfg.get('candidate_max_joint_delta_rad', 1.8)),
        )
        self._candidate_plan_metrics = {}
        self._approach_gate_rejected_count = 0
        self._table_geometry_gate_rejected_count = 0
        self._joint_motion_gate_rejected_count = 0
        self.gripper_physical_open_width_m = max(
            0.0,
            float(gripper_cfg.get('open_position_m', 0.05)),
        )
        default_max_width = float(
            twin_cfg.get(
                'open_width_m',
                gripper_cfg.get('open_position_m', remote_cfg.get('max_gripper_width_m', 0.05)),
            )
        )
        self.max_gripper_width_m = max(
            0.0,
            float(rospy.get_param('/grasp_6d/remote/max_gripper_width_m', remote_cfg.get('max_gripper_width_m', default_max_width))),
        )
        self.candidate_width_tolerance_m = max(
            0.0,
            float(rospy.get_param('/grasp_6d/remote/candidate_width_tolerance_m', remote_cfg.get('candidate_width_tolerance_m', 0.003))),
        )
        self._target_gate_rejected_count = 0
        self._visibility_gate_rejected_count = 0
        self._width_gate_rejected_count = 0
        self._depth_gate_rejected_count = 0
        self._width_gate_rejected_keys = set()
        try:
            self.client = RemoteGrasp6DClient(server_url, timeout_sec=timeout_sec)
        except ValueError as exc:
            rospy.logfatal('invalid remote 6D grasp server URL: %s', exc)
            raise
        self.pose_estimator, self.tf_buffer, self.tf_listener = make_remote_pose_estimator(cam_cfg, hcfg, gcfg)

        rospy.Subscriber(cam_cfg.get('color_topic', '/supervisor/camera/color/image_raw'), Image, self.color_cb, queue_size=1, buff_size=2**24, tcp_nodelay=True)
        rospy.Subscriber(cam_cfg.get('depth_topic', '/supervisor/camera/depth/image_raw'), Image, self.depth_cb, queue_size=1, buff_size=2**24, tcp_nodelay=True)
        rospy.Subscriber(
            pcfg.get('output_mask_topic', '/perception/object_mask'),
            Image,
            self.mask_cb,
            queue_size=1,
            buff_size=2**24,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            pcfg.get('output_object_topic', '/perception/object'),
            ObjectPose,
            self.object_cb,
            queue_size=1,
        )
        rospy.Subscriber('/joint_states', JointState, self.joint_cb, queue_size=1)
        self.rate_hz = max(0.1, float(rospy.get_param('/grasp_6d/remote/request_hz', remote_cfg.get('request_hz', rospy.get_param('/grasp_6d/plan_hz', 1.0)))))
        rospy.Service('/grasp_6d/request_plan', TriggerZero, self.request_plan_cb)
        self._check_remote_health()
        mode = 'auto %.2f Hz' % self.rate_hz if self.auto_request else 'manual trigger'
        self.status_pub.publish(String('remote 6D grasp waiting for RGB-D: %s (%s)' % (server_url, mode)))

    def color_cb(self, msg):
        self.frames.update_color(self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'), msg.header.stamp, msg.header.frame_id)

    @staticmethod
    def _ros_source_clock_ns():
        now = rospy.Time.now()
        if hasattr(now, 'to_nsec'):
            return int(now.to_nsec())
        return int(now.secs) * 1_000_000_000 + int(getattr(now, 'nsecs', 0))

    def depth_cb(self, msg):
        self.frames.update_depth(
            self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough'),
            msg.header.stamp,
            msg.header.frame_id,
        )

    def mask_cb(self, msg):
        self.frames.update_mask(
            self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8'),
            msg.header.stamp,
            msg.header.frame_id,
        )

    def joint_cb(self, msg):
        positions = np.asarray(getattr(msg, 'position', ()), dtype=float).reshape(-1)
        names = list(getattr(msg, 'name', ()) or ())
        if names and positions.size == len(names):
            by_name = {str(name).lower(): positions[index] for index, name in enumerate(names)}
            ordered = [
                by_name.get('joint%d' % index)
                for index in range(1, 7)
            ]
            if all(value is not None for value in ordered):
                positions = np.asarray(ordered, dtype=float)
        self.frames.update_joints(positions[:6])

    @staticmethod
    def _invalid_geometry_estimate(code, reason, source_mode=''):
        return GeometryEstimate(
            ok=False,
            failure_code=str(code),
            failure_reason=str(reason),
            center_base=np.zeros(3, dtype=float),
            axes_base=np.eye(3, dtype=float),
            size_xyz_m=np.zeros(3, dtype=float),
            support_normal_base=np.zeros(3, dtype=float),
            support_offset_m=0.0,
            support_inlier_ratio=0.0,
            object_points_base=np.zeros((0, 3), dtype=float),
            source_mode=str(source_mode or ''),
        )

    @staticmethod
    def _safe_time(value=None):
        if value is not None:
            return value
        try:
            return rospy.Time.now()
        except Exception:
            return rospy.Time(0)

    @staticmethod
    def _snapshot_ros_stamp(snapshot):
        stamp_ns = int(getattr(snapshot, 'stamp_ns', 0) or 0)
        if stamp_ns > 0:
            seconds, nanoseconds = divmod(stamp_ns, 1_000_000_000)
            return rospy.Time(seconds, nanoseconds)
        return rospy.Time.from_sec(float(getattr(snapshot, 'stamp_sec', 0.0) or 0.0))

    def _publish_empty_legacy_plan(self, stamp=None):
        invalid_plan = PoseArray()
        invalid_plan.header.stamp = self._safe_time(stamp)
        invalid_plan.header.frame_id = 'base_link'
        publisher = getattr(self, 'plan_pub', None)
        if publisher is not None:
            publisher.publish(invalid_plan)
        self.latest_plan = None

    def _geometry_state_guard(self):
        lock = getattr(self, '_geometry_state_lock', None)
        if lock is None:
            lock = threading.RLock()
            self._geometry_state_lock = lock
        return lock

    def _capture_geometry_generation(self):
        with self._geometry_state_guard():
            return int(getattr(self, '_geometry_invalidation_generation', 0))

    def _geometry_invalidation_state(self, expected_generation):
        with self._geometry_state_guard():
            current = int(getattr(self, '_geometry_invalidation_generation', 0))
            if current == int(expected_generation):
                return False, ''
            code = str(
                getattr(self, '_last_geometry_invalidation_code', 'PLAN_STALE')
                or 'PLAN_STALE'
            )
            return True, code

    def _invalidate_geometry_if_current(
        self,
        expected_generation,
        failure_code,
        failure_reason,
        stamp=None,
        snapshot=None,
        label='',
    ):
        with self._geometry_state_guard():
            current = int(getattr(self, '_geometry_invalidation_generation', 0))
            if current != int(expected_generation):
                message = str(
                    getattr(
                        getattr(self, 'latest_object_geometry', None),
                        'failure_reason',
                        '',
                    )
                    or ''
                )
                if not message:
                    code = str(
                        getattr(
                            self,
                            '_last_geometry_invalidation_code',
                            'PLAN_STALE',
                        )
                        or 'PLAN_STALE'
                    )
                    message = '%s: geometry was invalidated concurrently' % code
                return False, message
            invalid = self._invalidate_geometry(
                failure_code,
                failure_reason,
                stamp=stamp,
                snapshot=snapshot,
                label=label,
            )
            return True, str(invalid.failure_reason)

    def _publish_legacy_plan_if_current(self, plan, expected_generation):
        with self._geometry_state_guard():
            current = int(getattr(self, '_geometry_invalidation_generation', 0))
            if current != int(expected_generation):
                code = str(
                    getattr(
                        self,
                        '_last_geometry_invalidation_code',
                        'PLAN_STALE',
                    )
                    or 'PLAN_STALE'
                )
                return False, code
            self.plan_pub.publish(plan)
            return True, ''

    def _commit_selected_gate_if_current(
        self,
        candidate,
        gate,
        expected_generation,
    ):
        with self._geometry_state_guard():
            current = int(getattr(self, '_geometry_invalidation_generation', 0))
            if current != int(expected_generation):
                return False
            self._selected_candidate_gate = gate
            self.selected_required_open_width_m = float(
                gate.required_open_width_m
            )
            setattr(
                candidate,
                'required_open_width_m',
                self.selected_required_open_width_m,
            )
            return True

    def _clear_geometry_cache(self):
        self._latest_geometry_estimate = None
        self._selected_candidate_gate = None
        self.selected_required_open_width_m = None
        self.latest_target_cloud_base_xyz = None
        self.latest_target_cloud_camera_center = None
        self.latest_target_cloud_camera_points = None
        self.latest_target_cloud_time = None
        self.latest_target_cloud_count = 0
        self.latest_target_cloud_source = 'none'
        self.latest_support_plane_camera_point = None
        self.latest_support_plane_camera_normal = None
        self.latest_target_cloud_segmentation = 'depth-window'

    def _invalidate_geometry(
        self,
        failure_code,
        failure_reason,
        stamp=None,
        snapshot=None,
        label='',
    ):
        with self._geometry_state_guard():
            self._geometry_invalidation_generation = (
                int(getattr(self, '_geometry_invalidation_generation', 0)) + 1
            )
            self._last_geometry_invalidation_code = str(failure_code)
            source_mode = str(getattr(snapshot, 'source_mode', '') or '')
            estimate = self._invalid_geometry_estimate(
                failure_code,
                failure_reason,
                source_mode,
            )
            if not label:
                obj = getattr(snapshot, 'object_msg', None)
                label = str(getattr(obj, 'label', '') or '')
            message = geometry_estimate_to_message(
                estimate,
                snapshot=snapshot,
                stamp=self._safe_time(stamp),
                label=label,
            )
            publisher = getattr(self, 'geometry_pub', None)
            if publisher is not None:
                publisher.publish(message)
            self.latest_object_geometry = message
            self.previous_object_axes_base = None
            self._clear_geometry_cache()
            self._publish_empty_legacy_plan(stamp)
            return message

    def _snapshot_geometry_inputs(self, snapshot):
        if snapshot.source_mode == 'instance_mask':
            return snapshot.target_depth_raw, snapshot.object_mask
        if snapshot.source_mode != 'bbox_depth':
            raise ValueError('unsupported snapshot source_mode: %s' % snapshot.source_mode)
        bbox = tuple(snapshot.bbox or ())
        if len(bbox) != 4:
            raise ValueError('bbox_depth snapshot bbox is invalid')
        roi = expanded_bbox_roi(
            np.asarray(snapshot.depth_raw).shape,
            int(bbox[0]),
            int(bbox[1]),
            int(bbox[2]),
            int(bbox[3]),
            0,
        )
        foreground_roi, _count = self._foreground_mask_for_roi(
            snapshot.depth_raw,
            roi,
            min_points=getattr(self, 'geometry_min_object_points', 120),
        )
        target_depth = np.zeros_like(snapshot.depth_raw)
        foreground = np.zeros_like(snapshot.object_mask, dtype=np.uint8)
        x0, y0, x1, y1 = roi
        depth_roi = np.asarray(snapshot.depth_raw)[y0:y1, x0:x1]
        target_depth[y0:y1, x0:x1][foreground_roi] = depth_roi[foreground_roi]
        foreground[y0:y1, x0:x1][foreground_roi] = 255
        return target_depth, foreground

    def _snapshot_base_optical_transform(self, snapshot, stamp):
        source_frame = str(getattr(snapshot, 'frame_id', '') or '')
        if not source_frame:
            raise RuntimeError('snapshot camera frame is empty')
        pose_estimator = getattr(self, 'pose_estimator', None)
        base_frame = 'base_link'
        tf_buffer = getattr(self, 'tf_buffer', None)
        if tf_buffer is None:
            tf_buffer = getattr(pose_estimator, 'tf_buffer', None)
        if tf_buffer is not None:
            transform = tf_buffer.lookup_transform(
                base_frame,
                source_frame,
                stamp,
                rospy.Duration(float(getattr(pose_estimator, 'tf_timeout_sec', 0.2))),
            )
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            base_from_source = transform_matrix(
                [translation.x, translation.y, translation.z],
                [rotation.x, rotation.y, rotation.z, rotation.w],
            )
        else:
            raise RuntimeError(
                'snapshot-time TF %s <- %s is unavailable'
                % (base_frame, source_frame)
            )
        if 'optical' in source_frame.lower():
            return base_from_source
        source_from_optical = np.eye(4, dtype=float)
        source_from_optical[:3, :3] = OPTICAL_TO_ROS_CAMERA
        return base_from_source.dot(source_from_optical)

    def _prepare_snapshot_geometry(self, snapshot, stamp):
        try:
            target_depth, object_mask = self._snapshot_geometry_inputs(snapshot)
        except Exception as exc:
            return (
                self._invalid_geometry_estimate(
                    'DEPTH_INSUFFICIENT',
                    str(exc),
                    getattr(snapshot, 'source_mode', ''),
                ),
                np.zeros_like(snapshot.depth_raw),
                None,
            )
        valid_points = valid_depth_count(target_depth)
        minimum = max(1, int(getattr(self, 'geometry_min_object_points', 120)))
        if valid_points < minimum:
            return (
                self._invalid_geometry_estimate(
                    'DEPTH_INSUFFICIENT',
                    'target depth has too few valid points %d < %d'
                    % (valid_points, minimum),
                    getattr(snapshot, 'source_mode', ''),
                ),
                target_depth,
                None,
            )
        try:
            transform = self._snapshot_base_optical_transform(snapshot, stamp)
        except Exception as exc:
            return (
                self._invalid_geometry_estimate(
                    'TF_UNAVAILABLE',
                    str(exc),
                    getattr(snapshot, 'source_mode', ''),
                ),
                target_depth,
                None,
            )
        intrinsics = self._camera_intrinsics()
        estimate = estimate_object_geometry(
            depth_raw=snapshot.depth_raw,
            target_depth_raw=target_depth,
            object_mask=object_mask,
            bbox=snapshot.bbox,
            intrinsics=intrinsics,
            depth_scale=float(intrinsics.depth_scale),
            T_base_camera=transform,
            source_mode=snapshot.source_mode,
            support_bbox_expand_ratio=float(
                getattr(self, 'geometry_support_bbox_expand_ratio', 0.30)
            ),
            support_distance_threshold_m=float(
                getattr(self, 'geometry_support_distance_threshold_m', 0.004)
            ),
            voxel_size_m=float(getattr(self, 'geometry_voxel_size_m', 0.0025)),
            min_support_points=int(
                getattr(self, 'geometry_min_support_points', 200)
            ),
            min_object_points=minimum,
            min_size_m=float(getattr(self, 'geometry_min_size_m', 0.005)),
            max_size_m=float(getattr(self, 'geometry_max_size_m', 0.600)),
            max_height_m=float(getattr(self, 'geometry_max_height_m', 0.500)),
            previous_axes_base=getattr(self, 'previous_object_axes_base', None),
            outlier_neighbors=int(getattr(self, 'geometry_outlier_neighbors', 16)),
            outlier_std_ratio=float(
                getattr(self, 'geometry_outlier_std_ratio', 2.0)
            ),
        )
        return estimate, target_depth, transform

    def _activate_geometry(
        self,
        estimate,
        snapshot,
        stamp,
        transform,
        expected_generation=None,
    ):
        with self._geometry_state_guard():
            if (
                expected_generation is not None
                and int(getattr(self, '_geometry_invalidation_generation', 0))
                != int(expected_generation)
            ):
                return False
            label = str(
                getattr(getattr(snapshot, 'object_msg', None), 'label', '') or ''
            )
            message = geometry_estimate_to_message(
                estimate,
                snapshot=snapshot,
                stamp=stamp,
                label=label,
            )
            publisher = getattr(self, 'geometry_pub', None)
            if publisher is not None:
                publisher.publish(message)
            self.latest_object_geometry = message
            self._latest_geometry_estimate = estimate
            self.previous_object_axes_base = np.asarray(
                estimate.axes_base,
                dtype=float,
            ).copy()
            self.latest_target_cloud_base_xyz = np.asarray(
                estimate.center_base,
                dtype=float,
            ).copy()
            self.latest_target_cloud_time = stamp
            self.latest_target_cloud_count = len(estimate.object_points_base)
            self.latest_target_cloud_source = '%s_geometry' % estimate.source_mode
            self.latest_target_cloud_segmentation = (
                'support_plane=inliers:%.3f object:%d obb=(%.3f,%.3f,%.3f)m'
                % (
                    float(estimate.support_inlier_ratio),
                    len(estimate.object_points_base),
                    float(estimate.size_xyz_m[0]),
                    float(estimate.size_xyz_m[1]),
                    float(estimate.size_xyz_m[2]),
                )
            )
            optical_from_base = np.linalg.inv(np.asarray(transform, dtype=float))
            base_points = np.asarray(estimate.object_points_base, dtype=float)
            optical_points = (
                base_points @ optical_from_base[:3, :3].T
                + optical_from_base[:3, 3]
            )
            camera_points = self._project_points_for_camera_frame(optical_points)
            self.latest_target_cloud_camera_points = np.asarray(
                camera_points,
                dtype=np.float32,
            )
            center_optical = (
                np.asarray(estimate.center_base, dtype=float)
                @ optical_from_base[:3, :3].T
                + optical_from_base[:3, 3]
            )
            self.latest_target_cloud_camera_center = (
                self._project_points_for_camera_frame(
                    center_optical.reshape(1, 3)
                )[0]
            )
            support_point_base = (
                -float(estimate.support_offset_m)
                * np.asarray(estimate.support_normal_base, dtype=float)
            )
            support_point_optical = (
                support_point_base @ optical_from_base[:3, :3].T
                + optical_from_base[:3, 3]
            )
            support_normal_optical = (
                optical_from_base[:3, :3]
                @ np.asarray(estimate.support_normal_base, dtype=float)
            )
            self.latest_support_plane_camera_point = (
                self._project_points_for_camera_frame(
                    support_point_optical.reshape(1, 3)
                )[0]
            )
            support_normal_camera = self._project_vectors_for_camera_frame(
                support_normal_optical.reshape(1, 3)
            )[0]
            support_normal_camera /= max(
                float(np.linalg.norm(support_normal_camera)),
                1e-12,
            )
            self.latest_support_plane_camera_normal = support_normal_camera
            self._target_cloud_request_active = True
            return True

    def object_cb(self, msg):
        now = rospy.Time.now()
        source_stamp = getattr(getattr(msg, 'header', None), 'stamp', now)
        if not bool(getattr(msg, 'detected', False)):
            with self._object_lock:
                self.latest_object = msg
                self.latest_object_time = now
            if hasattr(self, 'frames'):
                self.frames.update_object(msg, source_stamp)
            self._invalidate_geometry(
                'TARGET_LOST',
                'target object is not detected',
                stamp=source_stamp,
                label=str(getattr(msg, 'label', '') or ''),
            )
            return

        gcfg = rospy.get_param('/grasp', {})
        confidence = float(getattr(msg, 'confidence', 1.0) or 0.0)
        min_confidence = self._cfg_float(gcfg, 'min_object_confidence', 0.50)
        if confidence < min_confidence:
            rospy.logwarn_throttle(
                1.0,
                'remote 6D ignored low-confidence object %.3f < %.3f',
                confidence,
                min_confidence,
            )
            return

        with self._object_lock:
            previous = self.latest_object
            previous_time = self.latest_object_time
        max_jump = self._cfg_float(gcfg, 'max_object_jump_m', 0.12)
        jump_window = self._cfg_float(gcfg, 'object_jump_filter_window_sec', 4.0)
        if (
            previous is not None
            and bool(getattr(previous, 'detected', False))
            and previous_time is not None
            and max_jump > 0.0
        ):
            try:
                age = (now - previous_time).to_sec()
            except Exception:
                age = float('inf')
            jump = self._object_pose_distance(previous, msg)
            if age <= jump_window and jump > max_jump:
                rospy.logwarn_throttle(
                    1.0,
                    'remote 6D ignored object jump %.3f m > %.3f m within %.1fs',
                    jump,
                    max_jump,
                    jump_window,
                )
                return

        with self._object_lock:
            self.latest_object = msg
            self.latest_object_time = now
        if hasattr(self, 'frames'):
            self.frames.update_object(msg, source_stamp)

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if self.enabled and self.auto_request:
                self._process_latest_frame()
            rate.sleep()

    def request_plan_cb(self, req):
        if not bool(getattr(req, 'trigger', False)):
            return TriggerZeroResponse(False, 'trigger=false')
        if not self.enabled:
            return TriggerZeroResponse(False, 'remote 6D disabled')
        try:
            require_mask = self._active_profile_requires_mask()
        except Exception as exc:
            message = 'MODEL_TASK_MISMATCH: %s' % exc
            self._invalidate_geometry('MODEL_TASK_MISMATCH', str(exc))
            self.status_pub.publish(String(message))
            return TriggerZeroResponse(False, message)
        snapshot, failure_code, failure_reason = self._wait_for_stable_snapshot(require_mask)
        if snapshot is None or not snapshot.ok:
            message = '%s: %s' % (failure_code, failure_reason)
            stamp = (
                self._snapshot_ros_stamp(snapshot)
                if snapshot is not None
                else None
            )
            self._invalidate_geometry(
                failure_code,
                failure_reason,
                stamp=stamp,
                snapshot=snapshot,
            )
            self.status_pub.publish(String(message))
            return TriggerZeroResponse(False, message)
        ok, message = self._process_frame(snapshot, manual=True)
        return TriggerZeroResponse(bool(ok), str(message))

    def _process_latest_frame(self):
        if rospy.Time.now() < self._backoff_until:
            return
        try:
            require_mask = self._active_profile_requires_mask()
        except Exception as exc:
            self._invalidate_geometry('MODEL_TASK_MISMATCH', str(exc))
            self.status_pub.publish(String('MODEL_TASK_MISMATCH: %s' % exc))
            return
        snapshot, failure_code, failure_reason = self._wait_for_stable_snapshot(require_mask)
        if snapshot is None or not snapshot.ok:
            stamp = (
                self._snapshot_ros_stamp(snapshot)
                if snapshot is not None
                else None
            )
            self._invalidate_geometry(
                failure_code,
                failure_reason,
                stamp=stamp,
                snapshot=snapshot,
            )
            self.status_pub.publish(String('%s: %s' % (failure_code, failure_reason)))
            return
        self._process_frame(snapshot, manual=False)

    def _active_profile_requires_mask(self):
        pcfg = rospy.get_param('/perception', {})
        model_choice = str(pcfg.get('yolo_model_choice', 'original'))
        previous_choice = getattr(self, '_last_model_choice', None)
        if previous_choice is not None and model_choice != previous_choice:
            self._invalidate_geometry(
                'MODEL_RELOADED',
                'model choice changed from %s to %s'
                % (previous_choice, model_choice),
            )
        self._last_model_choice = model_choice
        detector_kind = str(pcfg.get('detector', 'simple_hsv')).strip().lower()
        if detector_kind not in ('yolo', 'yolov8'):
            return False
        profile = select_yolo_model(
            pcfg,
            model_choice,
            pcfg.get('yolo_target_class', pcfg.get('object_label', 'target')),
        )
        return bool(profile.get('require_instance_mask', False))

    @staticmethod
    def _snapshot_depth_config():
        cam_cfg = rospy.get_param('/camera', {})
        pcfg = rospy.get_param('/perception', {})
        depth_scale = float(cam_cfg.get('depth_scale', 0.001))
        depth_min_m = float(pcfg.get('depth_min_m', 0.03))
        depth_max_m = float(pcfg.get('depth_max_m', 2.0))
        return depth_scale, depth_min_m, depth_max_m

    def _wait_for_stable_snapshot(self, require_mask):
        deadline = time.monotonic() + max(0.0, float(self.planning_snapshot_timeout_sec))
        last_failure = None
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            samples = self.frames.wait_for_samples(
                self.planning_snapshot_frames,
                remaining,
                require_mask=bool(require_mask),
                max_age_sec=self.planning_snapshot_max_age_sec,
            )
            if len(samples) < self.planning_snapshot_frames:
                break
            depth_scale, depth_min_m, depth_max_m = self._snapshot_depth_config()
            result = fuse_stable_samples(
                samples,
                require_mask=bool(require_mask),
                min_mask_iou=self.planning_mask_min_iou,
                max_centroid_shift_px=self.planning_mask_max_centroid_shift_px,
                max_joint_delta_rad=self.planning_max_joint_delta_rad,
                erosion_px=self.mask_erosion_px,
                depth_scale=depth_scale,
                depth_min_m=depth_min_m,
                depth_max_m=depth_max_m,
                mad_scale=self.depth_mad_scale,
                mad_absolute_floor_m=self.depth_mad_absolute_floor_m,
                internal_hole_max_area_px=self.mask_internal_hole_max_area_px,
            )
            if result.ok:
                return result, '', ''
            if result.failure_code != 'DEPTH_UNSTABLE':
                return result, result.failure_code, result.failure_reason
            last_failure = result
            self.frames.discard_through_ns(samples[-1].stamp_ns)

        if last_failure is not None:
            return last_failure, last_failure.failure_code, last_failure.failure_reason
        if require_mask:
            return None, 'MASK_MISSING', 'no three fresh timestamp-matched RGB-D-mask-object samples'
        return None, 'DEPTH_UNSTABLE', 'no three fresh timestamp-matched RGB-D-object samples'

    def _process_frame(self, snapshot, manual=False):
        if not self._request_lock.acquire(False):
            message = 'remote 6D request already running'
            if manual:
                self.status_pub.publish(String(message))
            return False, message
        if not isinstance(snapshot, SnapshotResult) or not snapshot.ok:
            self._request_lock.release()
            return False, 'remote 6D request requires a valid planning snapshot'
        stamp = self._snapshot_ros_stamp(snapshot)
        if not bool(getattr(snapshot.object_msg, 'detected', False)):
            message = 'TARGET_LOST: locked planning snapshot target is not detected'
            self._invalidate_geometry(
                'TARGET_LOST',
                'locked planning snapshot target is not detected',
                stamp=stamp,
                snapshot=snapshot,
            )
            self._publish_error(message)
            self._request_lock.release()
            return False, message
        color = snapshot.color_bgr
        frame_id = snapshot.frame_id
        request_invalidation_generation = self._capture_geometry_generation()
        self._planning_snapshot_active = True
        self._planning_object_msg = snapshot.object_msg
        self._planning_object_time = stamp
        try:
            self._refresh_runtime_params()
            self._cached_tool_from_camera = None
            self._publish_empty_legacy_plan(stamp)
            # Every filter below must use geometry from this exact RGB-D
            # request, even when remote inference takes several seconds.
            self._target_cloud_request_active = False
            self._clear_geometry_cache()
            estimate, depth_for_remote, transform = self._prepare_snapshot_geometry(
                snapshot,
                stamp,
            )
            if not estimate.ok:
                message = '%s: %s' % (
                    estimate.failure_code,
                    estimate.failure_reason,
                )
                self._invalidate_geometry(
                    estimate.failure_code,
                    estimate.failure_reason,
                    stamp=stamp,
                    snapshot=snapshot,
                )
                self._publish_error(message)
                return False, message
            activated = self._activate_geometry(
                estimate,
                snapshot,
                stamp,
                transform,
                expected_generation=request_invalidation_generation,
            )
            if not activated:
                _invalidated, failure_code = self._geometry_invalidation_state(
                    request_invalidation_generation
                )
                message = (
                    '%s: planning request was invalidated during geometry estimation'
                    % (failure_code or 'PLAN_STALE')
                )
                self._publish_error(message)
                return False, message
            contract_mismatch = self._gripper_contract_mismatch_reason()
            if contract_mismatch:
                _applied, message = self._invalidate_geometry_if_current(
                    request_invalidation_generation,
                    'GRIPPER_MODEL_MISMATCH',
                    contract_mismatch,
                    stamp=stamp,
                    snapshot=snapshot,
                )
                self._publish_error(message)
                return False, message
            bbox = tuple(snapshot.bbox or (0, 0, 0, 0))
            roi_message = (
                '%s x=%d y=%d w=%d h=%d target_depth=%d'
                % (
                    snapshot.source_mode,
                    int(bbox[0]) if len(bbox) > 0 else 0,
                    int(bbox[1]) if len(bbox) > 1 else 0,
                    int(bbox[2]) if len(bbox) > 2 else 0,
                    int(bbox[3]) if len(bbox) > 3 else 0,
                    valid_depth_count(depth_for_remote),
                )
            )
            status = 'remote 6D requesting candidates...'
            if roi_message:
                status += ' ' + roi_message
                rospy.loginfo('remote 6D request geometry: %s', roi_message)
            self.status_pub.publish(String(status))
            invalidated, failure_code = self._geometry_invalidation_state(
                request_invalidation_generation
            )
            if invalidated:
                message = (
                    '%s: planning request was invalidated before remote inference'
                    % failure_code
                )
                self._publish_error(message)
                return False, message
            try:
                candidates = self.client.predict(
                    color,
                    depth_for_remote,
                    self._camera_intrinsics(),
                    frame_id=frame_id or self.pose_estimator.camera_frame,
                    stamp_sec=float(snapshot.stamp_sec),
                    max_candidates=self.max_candidates,
                    # GraspNet's width is a model suggestion. Keep every raw
                    # proposal for the mandatory local OBB/50 mm hard gate.
                    max_gripper_width_m=0.0,
                    candidate_width_tolerance_m=self.candidate_width_tolerance_m,
                )
            except Exception as exc:
                failure_code = remote_prediction_failure_code(exc)
                failure_reason = str(exc)
                applied, message = self._invalidate_geometry_if_current(
                    request_invalidation_generation,
                    failure_code,
                    failure_reason,
                    stamp=stamp,
                    snapshot=snapshot,
                )
                self._publish_error(message)
                if applied and self.failure_backoff_sec > 0.0:
                    self._backoff_until = (
                        rospy.Time.now()
                        + rospy.Duration(self.failure_backoff_sec)
                    )
                return False, message
            remote_diagnostics = dict(getattr(self.client, 'last_diagnostics', {}) or {})
            if not candidates:
                failure_reason = 'remote GraspNet returned no candidates'
                failure_reason += self._candidate_failure_diagnostics(remote_diagnostics)
                _applied, message = self._invalidate_geometry_if_current(
                    request_invalidation_generation,
                    'NO_RAW_CANDIDATE',
                    failure_reason,
                    stamp=stamp,
                    snapshot=snapshot,
                )
                self._publish_error(message)
                return False, message
            invalidated, failure_code = self._geometry_invalidation_state(
                request_invalidation_generation
            )
            if invalidated:
                message = (
                    '%s: planning request was invalidated during remote inference'
                    % failure_code
                )
                self._publish_error(message)
                return False, message
            self._position_only_rejected_count = 0
            self._orientation_fallback_rejected_count = 0
            self._target_gate_rejected_count = 0
            self._visibility_gate_rejected_count = 0
            self._approach_gate_rejected_count = 0
            self._table_geometry_gate_rejected_count = 0
            self._joint_motion_gate_rejected_count = 0
            self._width_gate_rejected_count = 0
            self._depth_gate_rejected_count = 0
            self._width_gate_rejected_keys = set()
            self._candidate_plan_metrics = {}
            self._best_candidate_approach_cos = -1.0
            self._closest_candidate_cloud_distance = float('inf')
            self._closest_candidate_center_distance = float('inf')
            self._reset_geometry_gate_audit(len(candidates))
            if bool(getattr(self, 'gate_audit_enabled', True)):
                try:
                    self._run_candidate_gate_audit(
                        candidates,
                        stamp=stamp,
                        camera_frame=frame_id or self.pose_estimator.camera_frame,
                        remote_diagnostics=remote_diagnostics,
                    )
                except Exception as exc:
                    self._latest_gate_audit_summary = {}
                    rospy.logwarn('remote 6D gate audit skipped after internal error: %s', exc)
            selected, grasp_pose = select_first_reachable_candidate(
                candidates,
                self.pose_estimator,
                self._plan_reachable,
                stamp=stamp,
                camera_frame=frame_id or self.pose_estimator.camera_frame,
                candidate_frame_convention=self.candidate_frame_convention,
                candidate_filter_fn=self._candidate_matches_target,
                candidate_rank_fn=self._candidate_rank,
                orientation_variant_quaternions=self.orientation_variant_quaternions,
                model_grasp_to_tool_quaternion=self.model_grasp_to_tool_quaternion,
                candidate_geometry_fn=self._evaluate_candidate_geometry,
                grasp_config=self.grasp_config,
            )
            invalidated, failure_code = self._geometry_invalidation_state(
                request_invalidation_generation
            )
            if invalidated:
                message = (
                    '%s: planning request was invalidated during candidate selection'
                    % failure_code
                )
                self._publish_error(message)
                return False, message
            if selected is None:
                geometric_pass = int(
                    getattr(self, '_geometry_gate_counts', {}).get(
                        'after_swept_envelope',
                        0,
                    )
                )
                if geometric_pass == 0:
                    failure_reason = (
                        'raw candidates were all rejected by analytical gripper geometry; '
                        + self._geometry_gate_diagnostics()
                    )
                    _applied, message = self._invalidate_geometry_if_current(
                        request_invalidation_generation,
                        'NO_GEOMETRIC_CANDIDATE',
                        failure_reason,
                        stamp=stamp,
                        snapshot=snapshot,
                    )
                    self._publish_error(message)
                    return False, message
                if (
                    not candidates
                    and int(remote_diagnostics.get('width_rejected', 0) or 0) > 0
                    and int(remote_diagnostics.get('after_width', 0) or 0) == 0
                ):
                    message = (
                        'remote 6D WSL filtered all candidates by gripper width '
                        '(raw=%d, after_collision=%d, width>%0.3fm rejected=%d)'
                        % (
                            int(remote_diagnostics.get('raw_candidates', 0) or 0),
                            int(remote_diagnostics.get('after_collision', 0) or 0),
                            float(remote_diagnostics.get('width_limit_m', self.max_gripper_width_m) or self.max_gripper_width_m),
                            int(remote_diagnostics.get('width_rejected', 0) or 0),
                        )
                    )
                elif (
                    self._width_gate_rejected_count > 0
                    or self._depth_gate_rejected_count > 0
                    or self._target_gate_rejected_count > 0
                    or self._visibility_gate_rejected_count > 0
                    or self._approach_gate_rejected_count > 0
                    or self._table_geometry_gate_rejected_count > 0
                    or self._joint_motion_gate_rejected_count > 0
                    or self._position_only_rejected_count > 0
                    or self._orientation_fallback_rejected_count > 0
                ):
                    message = (
                        'remote 6D returned %d candidates, none executable '
                        '(width>%0.3fm: %d, missing-depth: %d, off-target: %d, '
                        'bad-approach: %d, table-geometry: %d, excessive-joint-motion: %d, '
                        'target-out-of-view: %d, position-only: %d, orientation-fallback: %d, '
                        'best-approach: %.3f required: %.3f)'
                        % (
                            len(candidates),
                            self.max_gripper_width_m,
                            self._width_gate_rejected_count,
                            self._depth_gate_rejected_count,
                            self._target_gate_rejected_count,
                            self._approach_gate_rejected_count,
                            self._table_geometry_gate_rejected_count,
                            self._joint_motion_gate_rejected_count,
                            self._visibility_gate_rejected_count,
                            self._position_only_rejected_count,
                            self._orientation_fallback_rejected_count,
                            float(getattr(self, '_best_candidate_approach_cos', -1.0)),
                            float(getattr(self, 'candidate_min_downward_approach_cos', -1.0)),
                        )
                    )
                else:
                    message = 'remote 6D returned %d candidates, none reachable' % len(candidates)
                message += self._candidate_failure_diagnostics(remote_diagnostics)
                message += self._candidate_gate_audit_diagnostics()
                self._publish_error(message)
                return False, message
            final_target_delta = self._candidate_target_distance(None, None, grasp_pose)
            selected_gate = getattr(selected, '_geometry_gate_result', None)
            if not isinstance(selected_gate, CandidateGateResult) or not selected_gate.ok:
                failure_reason = 'selected candidate has no valid analytical gripper result'
                _applied, message = self._invalidate_geometry_if_current(
                    request_invalidation_generation,
                    'NO_GEOMETRIC_CANDIDATE',
                    failure_reason,
                    stamp=stamp,
                    snapshot=snapshot,
                )
                self._publish_error(message)
                return False, message
            plan_msg = make_grasp_plan_pose_array(
                grasp_pose,
                stamp,
                self.grasp_config,
            )
            published, failure_code = self._publish_legacy_plan_if_current(
                plan_msg,
                request_invalidation_generation,
            )
            if not published:
                message = '%s: planning request was invalidated before publication' % failure_code
                self._publish_error(message)
                return False, message
            selected_target_xyz, target_source = self._target_base_xyz()
            visibility_message = ''
            if bool(getattr(self, 'camera_visibility_diagnostic_enabled', True)) and selected_target_xyz is not None:
                visible, metrics, reason = self._candidate_visibility_metrics(grasp_pose, selected_target_xyz)
                if metrics:
                    stage_text = ','.join(
                        '%s:u%.0f/v%.0f/z%.3f'
                        % (item['stage'], item['u'], item['v'], item['depth_m'])
                        for item in metrics
                    )
                    visibility_message = ' predicted_view=%s(%s)' % ('visible' if visible else 'lost', stage_text)
                elif not visible:
                    visibility_message = ' predicted_view=unknown(%s)' % reason
            depth_text = 'missing' if selected.depth_m is None else '%.3f' % float(selected.depth_m)
            selected_metrics = self._candidate_plan_metrics.get(self._pose_key(grasp_pose), {})
            approach_down = self._candidate_approach_downward_cos(selected, grasp_pose)
            message = 'remote 6D plan ready score=%.3f model_width=%.3f required_open=%.3f depth=%s target_delta=%.3fm target=%s strict_orientation=1 tool_aligned=1 approach_down=%.3f joint_path_cost=%.3f joint_max_delta=%.3f' % (
                selected.score,
                selected.width_m,
                selected_gate.required_open_width_m,
                depth_text,
                final_target_delta,
                target_source,
                approach_down,
                float(selected_metrics.get('joint_path_cost', 0.0)),
                float(selected_metrics.get('joint_max_delta', 0.0)),
            ) + visibility_message + ' [' + self._geometry_gate_diagnostics() + ']'
            support_metrics = self._candidate_support_geometry_metrics(selected)
            if support_metrics is not None:
                message += ' jaw_normal=%.3f finger_clearance=%.3fm geometry_width=%.3fm' % (
                    support_metrics['jaw_normal_cos'],
                    support_metrics['min_finger_clearance_m'],
                    support_metrics['geometry_width_m'],
                )
            self.status_pub.publish(String(message))
            if not self._commit_selected_gate_if_current(
                selected,
                selected_gate,
                request_invalidation_generation,
            ):
                _applied, message = self._invalidate_geometry_if_current(
                    request_invalidation_generation,
                    'PLAN_STALE',
                    'planning request was invalidated before selected gate commit',
                    stamp=stamp,
                    snapshot=snapshot,
                )
                self._publish_error(message)
                return False, message
            self.last_error = ''
            self._backoff_until = rospy.Time(0)
            return True, message
        except Exception as exc:
            failure_reason = 'remote 6D planning failed: %s' % exc
            applied, message = self._invalidate_geometry_if_current(
                request_invalidation_generation,
                'PLAN_FAILED',
                failure_reason,
                stamp=stamp,
                snapshot=snapshot,
            )
            self._publish_error(message)
            if applied and self.failure_backoff_sec > 0.0:
                self._backoff_until = rospy.Time.now() + rospy.Duration(self.failure_backoff_sec)
            return False, message
        finally:
            self._target_cloud_request_active = False
            self._planning_snapshot_active = False
            self._planning_object_msg = None
            self._planning_object_time = None
            self._request_lock.release()

    def _depth_for_remote(self, depth, stamp=None, frame_id=''):
        if isinstance(depth, SnapshotResult):
            valid = int(depth.quality.valid_depth_points)
            bbox = tuple(depth.bbox or (0, 0, 0, 0))
            return depth.target_depth_raw, (
                '%s x=%d y=%d w=%d h=%d target_depth=%d'
                % (
                    depth.source_mode,
                    int(bbox[0]) if len(bbox) > 0 else 0,
                    int(bbox[1]) if len(bbox) > 1 else 0,
                    int(bbox[2]) if len(bbox) > 2 else 0,
                    int(bbox[3]) if len(bbox) > 3 else 0,
                    valid,
                )
            )
        if not self.use_perception_roi:
            return depth, ''
        try:
            obj, obj_time = self._planning_object_snapshot()
            masked, roi, valid = self._masked_depth_for_object(depth, obj, obj_time)
            cloud_message = ''
            if bool(getattr(self, 'target_cloud_enabled', True)):
                cloud_message = ' ' + self._update_target_cloud_estimate(depth, obj, obj_time, stamp, frame_id)
            return masked, 'target ROI x=%d y=%d w=%d h=%d foreground_depth=%d%s' % (
                roi[0],
                roi[1],
                roi[2] - roi[0],
                roi[3] - roi[1],
                valid,
                cloud_message,
            )
        except Exception as exc:
            message = 'remote 6D waiting for target ROI: %s' % exc
            if self.require_perception_roi:
                raise RuntimeError(message)
            rospy.logwarn_throttle(2.0, '%s; falling back to full RGB-D frame', message)
            return depth, ''

    def _latest_object_snapshot(self):
        with self._object_lock:
            return self.latest_object, self.latest_object_time

    def _planning_object_snapshot(self):
        if bool(getattr(self, '_planning_snapshot_active', False)):
            return self._planning_object_msg, self._planning_object_time
        return self._latest_object_snapshot()

    @staticmethod
    def _cfg_float(cfg, key, default):
        try:
            if isinstance(cfg, dict) and key in cfg:
                return float(cfg.get(key))
        except Exception:
            pass
        return float(default)

    def _refresh_runtime_params(self):
        remote_cfg = rospy.get_param('/grasp_6d/remote', {})
        self.grasp_config = dict(rospy.get_param('/grasp', getattr(self, 'grasp_config', {})) or {})
        self.geometry_support_bbox_expand_ratio = max(
            0.0,
            float(
                remote_cfg.get(
                    'support_bbox_expand_ratio',
                    getattr(self, 'geometry_support_bbox_expand_ratio', 0.30),
                )
            ),
        )
        self.geometry_support_distance_threshold_m = max(
            0.0001,
            float(
                remote_cfg.get(
                    'support_distance_threshold_m',
                    remote_cfg.get(
                        'target_cloud_support_plane_inlier_distance_m',
                        getattr(self, 'geometry_support_distance_threshold_m', 0.004),
                    ),
                )
            ),
        )
        self.geometry_voxel_size_m = max(
            0.0001,
            float(
                remote_cfg.get(
                    'target_cloud_voxel_size_m',
                    getattr(self, 'geometry_voxel_size_m', 0.0025),
                )
            ),
        )
        self.geometry_outlier_neighbors = max(
            1,
            int(
                remote_cfg.get(
                    'target_cloud_outlier_neighbors',
                    getattr(self, 'geometry_outlier_neighbors', 16),
                )
            ),
        )
        self.geometry_outlier_std_ratio = max(
            0.0,
            float(
                remote_cfg.get(
                    'target_cloud_outlier_std_ratio',
                    getattr(self, 'geometry_outlier_std_ratio', 2.0),
                )
            ),
        )
        self.geometry_min_support_points = max(
            3,
            int(
                remote_cfg.get(
                    'geometry_min_support_points',
                    getattr(self, 'geometry_min_support_points', 200),
                )
            ),
        )
        self.geometry_min_object_points = max(
            3,
            int(
                remote_cfg.get(
                    'geometry_min_object_points',
                    remote_cfg.get(
                        'target_cloud_min_points',
                        getattr(self, 'geometry_min_object_points', 120),
                    ),
                )
            ),
        )
        self.geometry_min_size_m = max(
            0.0001,
            float(
                remote_cfg.get(
                    'geometry_min_size_m',
                    getattr(self, 'geometry_min_size_m', 0.005),
                )
            ),
        )
        self.geometry_max_size_m = max(
            self.geometry_min_size_m,
            float(
                remote_cfg.get(
                    'geometry_max_size_m',
                    getattr(self, 'geometry_max_size_m', 0.600),
                )
            ),
        )
        self.geometry_max_height_m = max(
            self.geometry_min_size_m,
            float(
                remote_cfg.get(
                    'geometry_max_height_m',
                    getattr(self, 'geometry_max_height_m', 0.500),
                )
            ),
        )
        self.candidate_frame_convention = normalize_candidate_frame_convention(
            rospy.get_param(
                '/grasp_6d/remote/candidate_frame_convention',
                remote_cfg.get('candidate_frame_convention', self.candidate_frame_convention),
            )
        )
        self.orientation_variant_quaternions = self._parse_orientation_variant_quaternions(
            rospy.get_param(
                '/grasp_6d/remote/orientation_variants_rpy_deg',
                remote_cfg.get('orientation_variants_rpy_deg', [[0.0, 0.0, 0.0]]),
            )
        )
        self.model_grasp_to_tool_quaternion = self._parse_orientation_variant_quaternions(
            [rospy.get_param(
                '/grasp_6d/remote/model_grasp_to_tool_rpy_deg',
                remote_cfg.get('model_grasp_to_tool_rpy_deg', [0.0, 0.0, 0.0]),
            )]
        )[0]
        self.require_candidate_depth = bool(
            rospy.get_param(
                '/grasp_6d/remote/require_candidate_depth',
                remote_cfg.get('require_candidate_depth', getattr(self, 'require_candidate_depth', False)),
            )
        )
        self.max_candidates = int(
            rospy.get_param('/grasp_6d/remote/max_candidates', remote_cfg.get('max_candidates', self.max_candidates))
        )
        self.allow_position_only_fallback = bool(
            rospy.get_param(
                '/grasp_6d/remote/accept_position_only_fallback',
                remote_cfg.get('accept_position_only_fallback', self.allow_position_only_fallback),
            )
        )
        self.allow_orientation_fallback = bool(
            rospy.get_param(
                '/grasp_6d/remote/accept_orientation_fallback',
                remote_cfg.get('accept_orientation_fallback', self.allow_orientation_fallback),
            )
        )
        self.candidate_target_gate_enabled = bool(
            rospy.get_param(
                '/grasp_6d/remote/candidate_target_gate_enabled',
                remote_cfg.get('candidate_target_gate_enabled', self.candidate_target_gate_enabled),
            )
        )
        self.camera_visibility_gate_enabled = bool(
            rospy.get_param(
                '/grasp_6d/remote/camera_visibility_gate_enabled',
                remote_cfg.get(
                    'camera_visibility_gate_enabled',
                    getattr(self, 'camera_visibility_gate_enabled', True),
                ),
            )
        )
        self.camera_visibility_diagnostic_enabled = bool(
            rospy.get_param(
                '/grasp_6d/remote/camera_visibility_diagnostic_enabled',
                remote_cfg.get(
                    'camera_visibility_diagnostic_enabled',
                    getattr(self, 'camera_visibility_diagnostic_enabled', True),
                ),
            )
        )
        self.camera_visibility_require_approach = bool(
            rospy.get_param(
                '/grasp_6d/remote/camera_visibility_require_approach',
                remote_cfg.get(
                    'camera_visibility_require_approach',
                    getattr(self, 'camera_visibility_require_approach', True),
                ),
            )
        )
        self.camera_visibility_margin_px = max(
            0,
            int(
                rospy.get_param(
                    '/grasp_6d/remote/camera_visibility_margin_px',
                    remote_cfg.get(
                        'camera_visibility_margin_px',
                        getattr(self, 'camera_visibility_margin_px', 36),
                    ),
                )
            ),
        )
        self.camera_visibility_bbox_padding_px = max(
            0,
            int(
                rospy.get_param(
                    '/grasp_6d/remote/camera_visibility_bbox_padding_px',
                    remote_cfg.get(
                        'camera_visibility_bbox_padding_px',
                        getattr(self, 'camera_visibility_bbox_padding_px', 8),
                    ),
                )
            ),
        )
        self.camera_visibility_min_depth_m = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/camera_visibility_min_depth_m',
                    remote_cfg.get(
                        'camera_visibility_min_depth_m',
                        getattr(self, 'camera_visibility_min_depth_m', 0.035),
                    ),
                )
            ),
        )
        self.camera_visibility_max_depth_m = max(
            self.camera_visibility_min_depth_m,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/camera_visibility_max_depth_m',
                    remote_cfg.get(
                        'camera_visibility_max_depth_m',
                        getattr(self, 'camera_visibility_max_depth_m', 1.20),
                    ),
                )
            ),
        )
        self.camera_visibility_rank_weight_m = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/camera_visibility_rank_weight_m',
                    remote_cfg.get(
                        'camera_visibility_rank_weight_m',
                        getattr(self, 'camera_visibility_rank_weight_m', 0.012),
                    ),
                )
            ),
        )
        self.rank_by_target_distance = bool(
            rospy.get_param(
                '/grasp_6d/remote/rank_by_target_distance',
                remote_cfg.get('rank_by_target_distance', getattr(self, 'rank_by_target_distance', True)),
            )
        )
        self.target_position_refine_enabled = bool(
            rospy.get_param(
                '/grasp_6d/remote/target_position_refine_enabled',
                remote_cfg.get('target_position_refine_enabled', getattr(self, 'target_position_refine_enabled', False)),
            )
        )
        self.target_position_refine_blend = self._clamp01(
            rospy.get_param(
                '/grasp_6d/remote/target_position_refine_blend',
                remote_cfg.get('target_position_refine_blend', getattr(self, 'target_position_refine_blend', 0.0)),
            )
        )
        self.target_position_refine_max_m = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/target_position_refine_max_m',
                    remote_cfg.get('target_position_refine_max_m', getattr(self, 'target_position_refine_max_m', 0.04)),
                )
            ),
        )
        self.target_position_refine_max_age_sec = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/target_position_refine_max_age_sec',
                    remote_cfg.get(
                        'target_position_refine_max_age_sec',
                        getattr(self, 'target_position_refine_max_age_sec', 1.0),
                    ),
                )
            ),
        )
        self.target_position_refine_offset_xyz_m = self._parse_xyz_param(
            rospy.get_param(
                '/grasp_6d/remote/target_position_refine_offset_xyz_m',
                remote_cfg.get(
                    'target_position_refine_offset_xyz_m',
                    getattr(self, 'target_position_refine_offset_xyz_m', [0.0, 0.0, 0.0]),
                ),
            )
        )
        self.target_cloud_enabled = bool(
            rospy.get_param(
                '/grasp_6d/remote/target_cloud_enabled',
                remote_cfg.get('target_cloud_enabled', getattr(self, 'target_cloud_enabled', True)),
            )
        )
        self.target_cloud_roi_margin_px = int(
            rospy.get_param(
                '/grasp_6d/remote/target_cloud_roi_margin_px',
                remote_cfg.get('target_cloud_roi_margin_px', getattr(self, 'target_cloud_roi_margin_px', 0)),
            )
        )
        self.target_cloud_support_plane_context_margin_px = max(
            0,
            int(
                rospy.get_param(
                    '/grasp_6d/remote/target_cloud_support_plane_context_margin_px',
                    remote_cfg.get(
                        'target_cloud_support_plane_context_margin_px',
                        getattr(self, 'target_cloud_support_plane_context_margin_px', 24),
                    ),
                )
            ),
        )
        self.target_cloud_foreground_percentile = self._clamp_range(
            rospy.get_param(
                '/grasp_6d/remote/target_cloud_foreground_percentile',
                remote_cfg.get(
                    'target_cloud_foreground_percentile',
                    getattr(self, 'target_cloud_foreground_percentile', 35.0),
                ),
            ),
            1.0,
            95.0,
        )
        self.target_cloud_depth_window_m = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/target_cloud_depth_window_m',
                    remote_cfg.get('target_cloud_depth_window_m', getattr(self, 'target_cloud_depth_window_m', 0.055)),
                )
            ),
        )
        self.target_cloud_min_points = max(
            1,
            int(
                rospy.get_param(
                    '/grasp_6d/remote/target_cloud_min_points',
                    remote_cfg.get('target_cloud_min_points', getattr(self, 'target_cloud_min_points', 80)),
                )
            ),
        )
        self.target_cloud_max_age_sec = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/target_cloud_max_age_sec',
                    remote_cfg.get('target_cloud_max_age_sec', getattr(self, 'target_cloud_max_age_sec', 1.0)),
                )
            ),
        )
        self.target_cloud_max_points_for_gate = max(
            1,
            int(
                rospy.get_param(
                    '/grasp_6d/remote/target_cloud_max_points_for_gate',
                    remote_cfg.get(
                        'target_cloud_max_points_for_gate',
                        getattr(self, 'target_cloud_max_points_for_gate', 2500),
                    ),
                )
            ),
        )
        self.target_cloud_candidate_max_point_distance_m = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/target_cloud_candidate_max_point_distance_m',
                    remote_cfg.get(
                        'target_cloud_candidate_max_point_distance_m',
                        getattr(self, 'target_cloud_candidate_max_point_distance_m', 0.055),
                    ),
                )
            ),
        )
        self.target_projection_frame_convention = normalize_candidate_frame_convention(
            rospy.get_param(
                '/grasp_6d/remote/target_projection_frame_convention',
                remote_cfg.get(
                    'target_projection_frame_convention',
                    getattr(self, 'target_projection_frame_convention', 'ros_camera_link'),
                ),
            )
        )
        self.candidate_max_target_distance_m = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/candidate_max_target_distance_m',
                    remote_cfg.get('candidate_max_target_distance_m', self.candidate_max_target_distance_m),
                )
            ),
        )
        self.candidate_min_relative_z_m = float(
            rospy.get_param(
                '/grasp_6d/remote/candidate_min_relative_z_m',
                remote_cfg.get('candidate_min_relative_z_m', self.candidate_min_relative_z_m),
            )
        )
        self.candidate_max_relative_z_m = float(
            rospy.get_param(
                '/grasp_6d/remote/candidate_max_relative_z_m',
                remote_cfg.get('candidate_max_relative_z_m', self.candidate_max_relative_z_m),
            )
        )
        self.candidate_center_distance_weight = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/candidate_center_distance_weight',
                    remote_cfg.get(
                        'candidate_center_distance_weight',
                        getattr(self, 'candidate_center_distance_weight', 1.0),
                    ),
                )
            ),
        )
        self.candidate_model_score_weight_m = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/candidate_model_score_weight_m',
                    remote_cfg.get(
                        'candidate_model_score_weight_m',
                        getattr(self, 'candidate_model_score_weight_m', 0.015),
                    ),
                )
            ),
        )
        self.candidate_joint_path_cost_weight_m = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/candidate_joint_path_cost_weight_m',
                    remote_cfg.get(
                        'candidate_joint_path_cost_weight_m',
                        getattr(self, 'candidate_joint_path_cost_weight_m', 0.004),
                    ),
                )
            ),
        )
        self.candidate_downward_approach_weight_m = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/candidate_downward_approach_weight_m',
                    remote_cfg.get(
                        'candidate_downward_approach_weight_m',
                        getattr(self, 'candidate_downward_approach_weight_m', 0.020),
                    ),
                )
            ),
        )
        self.candidate_min_downward_approach_cos = float(
            rospy.get_param(
                '/grasp_6d/remote/candidate_min_downward_approach_cos',
                remote_cfg.get(
                        'candidate_min_downward_approach_cos',
                        getattr(self, 'candidate_min_downward_approach_cos', 0.55),
                ),
            )
        )
        self.candidate_max_jaw_normal_cos = self._clamp_range(
            rospy.get_param(
                '/grasp_6d/remote/candidate_max_jaw_normal_cos',
                remote_cfg.get(
                    'candidate_max_jaw_normal_cos',
                    getattr(self, 'candidate_max_jaw_normal_cos', 0.35),
                ),
            ),
            0.0,
            1.0,
        )
        self.candidate_jaw_tilt_rank_weight_m = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/candidate_jaw_tilt_rank_weight_m',
                    remote_cfg.get(
                        'candidate_jaw_tilt_rank_weight_m',
                        getattr(self, 'candidate_jaw_tilt_rank_weight_m', 0.010),
                    ),
                )
            ),
        )
        self.candidate_min_finger_support_clearance_m = float(
            rospy.get_param(
                '/grasp_6d/remote/candidate_min_finger_support_clearance_m',
                remote_cfg.get(
                    'candidate_min_finger_support_clearance_m',
                    getattr(self, 'candidate_min_finger_support_clearance_m', 0.003),
                ),
            )
        )
        self.candidate_max_joint_delta_rad = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/candidate_max_joint_delta_rad',
                    remote_cfg.get(
                        'candidate_max_joint_delta_rad',
                        getattr(self, 'candidate_max_joint_delta_rad', 1.8),
                    ),
                )
            ),
        )
        gripper_cfg = rospy.get_param('/gripper', {})
        twin_cfg = rospy.get_param('/mujoco_digital_twin', {})
        gripper_geometry_cfg = dict(remote_cfg.get('gripper_geometry', {}) or {})
        self.gripper_geometry = GripperGeometry(
            max_inner_gap_m=float(
                gripper_geometry_cfg.get(
                    'max_inner_gap_m',
                    getattr(self.gripper_geometry, 'max_inner_gap_m', 0.050),
                )
            ),
            jaw_clearance_each_side_m=float(
                gripper_geometry_cfg.get(
                    'width_safety_margin_per_side_m',
                    getattr(
                        self.gripper_geometry,
                        'jaw_clearance_each_side_m',
                        0.002,
                    ),
                )
            ),
            finger_size_xyz_m=np.asarray(
                gripper_geometry_cfg.get(
                    'finger_box_xyz_m',
                    getattr(
                        self.gripper_geometry,
                        'finger_size_xyz_m',
                        [0.0434, 0.0286, 0.0600],
                    ),
                ),
                dtype=float,
            ),
            palm_size_xyz_m=np.asarray(
                gripper_geometry_cfg.get(
                    'palm_box_xyz_m',
                    getattr(
                        self.gripper_geometry,
                        'palm_size_xyz_m',
                        [0.1175, 0.1550, 0.0774],
                    ),
                ),
                dtype=float,
            ),
            support_clearance_m=float(
                gripper_geometry_cfg.get(
                    'support_clearance_m',
                    getattr(
                        self.gripper_geometry,
                        'support_clearance_m',
                        0.003,
                    ),
                )
            ),
        )
        self.gripper_tool_jaw_axis = str(
            gripper_geometry_cfg.get(
                'tool_jaw_axis',
                getattr(self, 'gripper_tool_jaw_axis', 'y'),
            )
        )
        self.gripper_tool_finger_length_axis = str(
            gripper_geometry_cfg.get(
                'tool_finger_length_axis',
                getattr(self, 'gripper_tool_finger_length_axis', 'z'),
            )
        )
        twin_gripper_cfg = dict(twin_cfg.get('gripper_model', {}) or {})
        self.twin_gripper_model_name = str(
            twin_gripper_cfg.get(
                'name',
                getattr(
                    self,
                    'twin_gripper_model_name',
                    'Alicia_D_v5_6_gripper_50mm',
                ),
            )
        )
        self.twin_max_inner_gap_m = float(
            twin_gripper_cfg.get(
                'max_inner_gap_m',
                getattr(self, 'twin_max_inner_gap_m', 0.050),
            )
        )
        self.gripper_physical_open_width_m = max(
            0.0,
            float(
                rospy.get_param(
                    '/gripper/open_position_m',
                    gripper_cfg.get(
                        'open_position_m',
                        getattr(self, 'gripper_physical_open_width_m', 0.05),
                    ),
                )
            ),
        )
        default_max_width = float(
            twin_cfg.get(
                'open_width_m',
                gripper_cfg.get('open_position_m', getattr(self, 'max_gripper_width_m', 0.05)),
            )
        )
        self.max_gripper_width_m = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/max_gripper_width_m',
                    remote_cfg.get('max_gripper_width_m', default_max_width),
                )
            ),
        )
        self.candidate_width_tolerance_m = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/candidate_width_tolerance_m',
                    remote_cfg.get('candidate_width_tolerance_m', getattr(self, 'candidate_width_tolerance_m', 0.003)),
                )
            ),
        )

    @staticmethod
    def _parse_orientation_variant_quaternions(value):
        variants = []
        for item in list(value or []):
            if isinstance(item, str):
                parts = [float(part.strip()) for part in item.replace(',', ' ').split() if part.strip()]
            else:
                parts = [float(part) for part in item]
            if len(parts) != 3:
                raise ValueError('orientation variant must be [roll_deg, pitch_deg, yaw_deg]')
            roll, pitch, yaw = [math.radians(part) for part in parts]
            variants.append(_normalize_quaternion(np.asarray(quaternion_from_euler(roll, pitch, yaw), dtype=float)))
        if not variants:
            variants.append(np.asarray([0.0, 0.0, 0.0, 1.0], dtype=float))
        return variants

    @staticmethod
    def _object_pose_distance(first, second):
        try:
            a = first.pose_base.pose.position
            b = second.pose_base.pose.position
            return math.sqrt(
                (float(a.x) - float(b.x)) ** 2
                + (float(a.y) - float(b.y)) ** 2
                + (float(a.z) - float(b.z)) ** 2
            )
        except Exception:
            return float('inf')

    def _masked_depth_for_object(self, depth, obj, obj_time):
        if obj is None or not bool(getattr(obj, 'detected', False)):
            raise RuntimeError('no detected target object')
        if obj_time is None:
            raise RuntimeError('target object timestamp unavailable')
        age = (rospy.Time.now() - obj_time).to_sec()
        if age > self.roi_max_age_sec:
            raise RuntimeError('target ROI is stale %.2fs > %.2fs' % (age, self.roi_max_age_sec))
        roi = expanded_bbox_roi(
            np.asarray(depth).shape,
            int(getattr(obj, 'bbox_x', 0)),
            int(getattr(obj, 'bbox_y', 0)),
            int(getattr(obj, 'bbox_width', 0)),
            int(getattr(obj, 'bbox_height', 0)),
            self.roi_margin_px,
        )
        foreground_mask, valid = self._foreground_mask_for_roi(depth, roi)
        masked = np.zeros_like(depth)
        x0, y0, x1, y1 = roi
        masked_roi = masked[y0:y1, x0:x1]
        depth_roi = np.asarray(depth)[y0:y1, x0:x1]
        masked_roi[foreground_mask] = depth_roi[foreground_mask]
        if valid < self.roi_min_valid_depth_px:
            raise RuntimeError('target foreground has too few valid depth pixels %d < %d' % (valid, self.roi_min_valid_depth_px))
        return masked, roi, valid

    def _foreground_mask_for_roi(self, depth, roi, min_points=None, use_support_context=True):
        min_count = max(
            1,
            int(min_points if min_points is not None else getattr(self, 'target_cloud_min_points', 80)),
        )
        context_margin = max(
            0,
            int(getattr(self, 'target_cloud_support_plane_context_margin_px', 0)),
        )
        if (
            use_support_context
            and bool(getattr(self, 'target_cloud_support_plane_enabled', True))
            and context_margin > 0
        ):
            context_roi = expanded_bbox_roi(
                np.asarray(depth).shape,
                roi[0],
                roi[1],
                roi[2] - roi[0],
                roi[3] - roi[1],
                context_margin,
            )
            if context_roi != roi:
                context_mask, _context_count = self._foreground_mask_for_roi(
                    depth,
                    context_roi,
                    min_points=min_count,
                    use_support_context=False,
                )
                if (
                    getattr(self, 'latest_support_plane_camera_point', None) is not None
                    and getattr(self, 'latest_support_plane_camera_normal', None) is not None
                ):
                    offset_x = roi[0] - context_roi[0]
                    offset_y = roi[1] - context_roi[1]
                    width = roi[2] - roi[0]
                    height = roi[3] - roi[1]
                    cropped = np.asarray(context_mask)[
                        offset_y:offset_y + height,
                        offset_x:offset_x + width,
                    ].copy()
                    cropped_count = int(np.count_nonzero(cropped))
                    if cropped_count >= min_count:
                        self.latest_target_cloud_segmentation += ' context-margin:%dpx' % context_margin
                        return cropped, cropped_count

        x0, y0, x1, y1 = roi
        depth_roi = np.asarray(depth)[y0:y1, x0:x1]
        if depth_roi.size == 0:
            raise RuntimeError('empty target depth ROI')
        z_m = self._depth_to_meters(depth_roi)
        pcfg = rospy.get_param('/perception', {})
        depth_min = max(0.0, float(pcfg.get('depth_min_m', 0.03)))
        depth_max = max(depth_min, float(pcfg.get('depth_max_m', 2.0)))
        valid = np.isfinite(z_m) & (z_m >= depth_min) & (z_m <= depth_max)
        if int(np.count_nonzero(valid)) < min_count:
            return np.zeros_like(valid, dtype=bool), int(np.count_nonzero(valid))
        self.latest_support_plane_camera_point = None
        self.latest_support_plane_camera_normal = None
        self.latest_target_cloud_segmentation = 'depth-window'
        if bool(getattr(self, 'target_cloud_support_plane_enabled', True)):
            plane_mask, plane_model, diagnostic = segment_foreground_above_support_plane(
                z_m,
                valid,
                roi,
                self._camera_intrinsics(),
                min_points=min_count,
                iterations=getattr(self, 'target_cloud_support_plane_ransac_iterations', 96),
                far_percentile=getattr(self, 'target_cloud_support_plane_far_percentile', 55.0),
                inlier_distance_m=getattr(self, 'target_cloud_support_plane_inlier_distance_m', 0.0035),
                min_height_m=getattr(self, 'target_cloud_support_plane_min_height_m', 0.004),
                min_inlier_ratio=getattr(self, 'target_cloud_support_plane_min_inlier_ratio', 0.08),
            )
            if plane_mask is not None and plane_model is not None:
                point = self._project_points_for_camera_frame(
                    np.asarray(plane_model['point_optical'], dtype=np.float32).reshape(1, 3)
                )[0]
                normal = self._project_vectors_for_camera_frame(
                    np.asarray(plane_model['normal_optical'], dtype=np.float32).reshape(1, 3)
                )[0]
                normal /= max(float(np.linalg.norm(normal)), 1e-12)
                self.latest_support_plane_camera_point = np.asarray(point, dtype=float)
                self.latest_support_plane_camera_normal = np.asarray(normal, dtype=float)
                self.latest_target_cloud_segmentation = diagnostic
                return plane_mask, int(np.count_nonzero(plane_mask))
            self.latest_target_cloud_segmentation = diagnostic
        values = z_m[valid]
        foreground_z = float(np.percentile(values, float(getattr(self, 'target_cloud_foreground_percentile', 35.0))))
        window = max(0.0, float(getattr(self, 'target_cloud_depth_window_m', 0.055)))
        foreground = valid & (z_m <= foreground_z + window)
        if int(np.count_nonzero(foreground)) < min_count:
            widened = valid & (z_m <= foreground_z + max(window * 2.0, 0.08))
            if int(np.count_nonzero(widened)) >= min_count:
                foreground = widened
            else:
                foreground = valid
        return foreground, int(np.count_nonzero(foreground))

    def _candidate_failure_diagnostics(self, remote_diagnostics):
        diagnostics = dict(remote_diagnostics or {})

        def _count(name):
            try:
                return int(diagnostics.get(name, -1))
            except Exception:
                return -1

        closest_cloud = float(getattr(self, '_closest_candidate_cloud_distance', float('inf')))
        closest_center = float(getattr(self, '_closest_candidate_center_distance', float('inf')))
        cloud_text = 'inf' if not np.isfinite(closest_cloud) else '%.3f' % closest_cloud
        center_text = 'inf' if not np.isfinite(closest_center) else '%.3f' % closest_center
        segmentation = str(getattr(self, 'latest_target_cloud_segmentation', 'unknown'))
        return (
            ' [WSL raw=%d nms=%d collision=%d returned=%d; '
            'target-cloud=%d closest-cloud=%sm closest-center=%sm; %s]'
            % (
                _count('raw_candidates'),
                _count('after_nms'),
                _count('after_collision'),
                _count('returned'),
                int(getattr(self, 'latest_target_cloud_count', 0)),
                cloud_text,
                center_text,
                segmentation,
            )
        )

    def _reset_geometry_gate_audit(self, raw_candidates):
        self._geometry_gate_counts = {
            'raw': int(raw_candidates),
            'raw_variants': 0,
            'after_transform': 0,
            'after_center': 0,
            'after_jaw_width': 0,
            'after_finger_reach': 0,
            'after_static_envelope': 0,
            'after_swept_envelope': 0,
        }
        self._geometry_rejection_counts = {}
        self._selected_candidate_gate = None
        self.selected_required_open_width_m = None

    def _record_geometry_gate_result(self, result):
        if not isinstance(result, CandidateGateResult):
            raise ValueError('analytical gripper gate returned an invalid result')
        counts = getattr(self, '_geometry_gate_counts', None)
        if not isinstance(counts, dict):
            self._reset_geometry_gate_audit(0)
            counts = self._geometry_gate_counts
        counts['raw_variants'] = int(counts.get('raw_variants', 0)) + 1
        stage_names = (
            'after_transform',
            'after_center',
            'after_jaw_width',
            'after_finger_reach',
            'after_static_envelope',
            'after_swept_envelope',
        )
        for index in range(min(len(stage_names), int(result.passed_gate_count))):
            name = stage_names[index]
            counts[name] = int(counts.get(name, 0)) + 1
        if not result.ok:
            rejections = getattr(self, '_geometry_rejection_counts', None)
            if not isinstance(rejections, dict):
                rejections = {}
                self._geometry_rejection_counts = rejections
            code = str(result.failure_code or 'GRIPPER_SWEEP_COLLISION')
            rejections[code] = int(rejections.get(code, 0)) + 1

    def _geometry_gate_diagnostics(self):
        counts = dict(getattr(self, '_geometry_gate_counts', {}) or {})
        rejections = dict(
            getattr(self, '_geometry_rejection_counts', {}) or {}
        )
        if not counts:
            return ''
        order = (
            'raw',
            'raw_variants',
            'after_transform',
            'after_center',
            'after_jaw_width',
            'after_finger_reach',
            'after_static_envelope',
            'after_swept_envelope',
        )
        count_text = ' '.join(
            '%s=%d' % (name, int(counts.get(name, 0)))
            for name in order
        )
        rejection_text = ','.join(
            '%s=%d' % (code, int(rejections[code]))
            for code in sorted(rejections)
        ) or 'none'
        return '%s rejected=%s' % (count_text, rejection_text)

    def _gripper_contract_mismatch_reason(self):
        validator = getattr(self, 'gripper_contract_validator', None)
        if callable(validator):
            return str(
                validator(
                    getattr(self, 'gripper_geometry', None),
                    getattr(self, 'max_gripper_width_m', 0.0),
                    getattr(self, 'gripper_physical_open_width_m', 0.0),
                    getattr(self, 'twin_gripper_model_name', ''),
                    getattr(self, 'twin_max_inner_gap_m', 0.0),
                )
                or ''
            )
        return gripper_contract_mismatch_reason(
            getattr(self, 'gripper_geometry', None),
            getattr(self, 'max_gripper_width_m', 0.0),
            getattr(self, 'gripper_physical_open_width_m', 0.0),
            getattr(self, 'twin_gripper_model_name', ''),
            getattr(self, 'twin_max_inner_gap_m', 0.0),
            tool_jaw_axis=getattr(self, 'gripper_tool_jaw_axis', 'y'),
            tool_finger_length_axis=getattr(
                self,
                'gripper_tool_finger_length_axis',
                'z',
            ),
        )

    def _evaluate_candidate_geometry(
        self,
        _raw_candidate,
        camera_candidate,
        grasp_pose,
        plan,
    ):
        estimate = getattr(self, '_latest_geometry_estimate', None)
        if estimate is None or not bool(getattr(estimate, 'ok', False)):
            result = CandidateGateResult(
                ok=False,
                failure_code='GRIPPER_SWEEP_COLLISION',
                failure_reason='base-frame object geometry is unavailable',
                required_open_width_m=0.0,
                center_distance_m=0.0,
                support_clearance_m=-1.0e6,
                jaw_alignment=0.0,
                motion_cost=0.0,
                geometry_cost=0.0,
                failed_gate='transform',
                passed_gate_count=0,
            )
            self._record_geometry_gate_result(result)
            return result
        grasp_transform = pose_matrix(grasp_pose)
        result = evaluate_candidate(
            gripper=self.gripper_geometry,
            candidate_center_base=grasp_transform[:3, 3],
            R_base_tool=grasp_transform[:3, :3],
            candidate_width_m=float(
                getattr(camera_candidate, 'width_m', 0.0) or 0.0
            ),
            obb_center_base=estimate.center_base,
            R_base_obb=estimate.axes_base,
            obb_size_xyz_m=estimate.size_xyz_m,
            support_normal_base=estimate.support_normal_base,
            support_offset_m=estimate.support_offset_m,
            pregrasp_T_base_tool=pose_matrix(plan.pregrasp),
            approach_T_base_tool=pose_matrix(plan.approach),
            grasp_T_base_tool=grasp_transform,
            lift_T_base_tool=pose_matrix(plan.lift),
            tool_jaw_axis=getattr(self, 'gripper_tool_jaw_axis', 'y'),
            tool_finger_length_axis=getattr(
                self,
                'gripper_tool_finger_length_axis',
                'z',
            ),
            motion_cost=0.0,
        )
        self._record_geometry_gate_result(result)
        if not result.ok:
            rospy.logwarn(
                (
                    'remote 6D analytical gripper rejection: '
                    'gate=%s code=%s required=%.3fm model_width=%.3fm reason=%s'
                ),
                result.failed_gate,
                result.failure_code,
                result.required_open_width_m,
                float(getattr(camera_candidate, 'width_m', 0.0) or 0.0),
                result.failure_reason,
            )
        return result

    def _run_candidate_gate_audit(self, candidates, stamp, camera_frame, remote_diagnostics=None):
        rows = []
        variants = list(getattr(self, 'orientation_variant_quaternions', []) or [])
        if not variants:
            variants = [np.asarray([0.0, 0.0, 0.0, 1.0], dtype=float)]
        for candidate_index, candidate in enumerate(candidates):
            camera_candidate = convert_candidate_to_camera_link(
                candidate,
                self.candidate_frame_convention,
            )
            camera_candidate = align_candidate_to_tool_frame(
                camera_candidate,
                self.model_grasp_to_tool_quaternion,
            )
            for variant_index, correction in enumerate(variants):
                variant_candidate = camera_candidate
                if not np.allclose(
                    np.asarray(correction, dtype=float),
                    np.asarray([0.0, 0.0, 0.0, 1.0], dtype=float),
                ):
                    variant_candidate = RemoteGraspCandidate(
                        score=float(camera_candidate.score),
                        translation_m=np.asarray(camera_candidate.translation_m, dtype=float),
                        quaternion_xyzw=_normalize_quaternion(
                            np.asarray(
                                quaternion_multiply(camera_candidate.quaternion_xyzw, correction),
                                dtype=float,
                            )
                        ),
                        width_m=float(camera_candidate.width_m),
                        height_m=getattr(camera_candidate, 'height_m', None),
                        depth_m=getattr(camera_candidate, 'depth_m', None),
                    )
                pose = self.pose_estimator.make_base_pose_from_camera_pose(
                    variant_candidate.translation_m,
                    variant_candidate.quaternion_xyzw,
                    stamp=stamp,
                    camera_frame=camera_frame,
                )
                rows.append(
                    self._candidate_gate_audit_row(
                        candidate_index,
                        variant_index,
                        variant_candidate,
                        pose,
                    )
                )

        clearance_thresholds = self._audit_thresholds(
            getattr(self, 'candidate_min_finger_support_clearance_m', 0.003),
            (0.003, 0.0, -0.003, -0.010),
        )
        approach_thresholds = self._audit_thresholds(
            getattr(self, 'candidate_min_downward_approach_cos', 0.45),
            (0.45, 0.20, 0.0, -0.20),
        )
        summary = summarize_candidate_gate_audit(
            rows,
            clearance_thresholds,
            approach_thresholds,
            baseline_clearance_m=getattr(
                self,
                'candidate_min_finger_support_clearance_m',
                0.003,
            ),
            baseline_approach_cos=getattr(
                self,
                'candidate_min_downward_approach_cos',
                0.45,
            ),
        )
        summary['baseline_clearance_m'] = float(
            getattr(self, 'candidate_min_finger_support_clearance_m', 0.003)
        )
        summary['baseline_approach_cos'] = float(
            getattr(self, 'candidate_min_downward_approach_cos', 0.45)
        )
        report = {
            'stamp_sec': float(stamp.to_sec()) if stamp is not None else 0.0,
            'camera_frame': str(camera_frame),
            'target_cloud_segmentation': str(
                getattr(self, 'latest_target_cloud_segmentation', 'unknown')
            ),
            'support_plane_point_camera': self._json_vector(
                getattr(self, 'latest_support_plane_camera_point', None)
            ),
            'support_plane_normal_camera': self._json_vector(
                getattr(self, 'latest_support_plane_camera_normal', None)
            ),
            'target_cloud_center_camera': self._json_vector(
                getattr(self, 'latest_target_cloud_camera_center', None)
            ),
            'target_cloud_center_base': self._json_vector(
                getattr(self, 'latest_target_cloud_base_xyz', None)
            ),
            'remote_diagnostics': dict(remote_diagnostics or {}),
            'summary': summary,
            'rows': rows,
        }
        self._latest_gate_audit_summary = summary
        summary_text = self._format_gate_audit_summary(summary)
        self.gate_audit_pub.publish(String(json.dumps(summary, ensure_ascii=True, sort_keys=True)))
        rospy.logwarn('remote 6D controlled gate audit: %s', summary_text)
        self._write_gate_audit_report(report)

    def _candidate_gate_audit_row(self, candidate_index, variant_index, candidate, grasp_pose):
        try:
            depth = float(getattr(candidate, 'depth_m', float('nan')))
        except (TypeError, ValueError):
            depth = float('nan')
        depth_ok = (
            not bool(getattr(self, 'require_candidate_depth', False))
            or (np.isfinite(depth) and depth > 0.0)
        )
        width = float(getattr(candidate, 'width_m', 0.0) or 0.0)
        # The model width is diagnostic only. The mandatory physical width
        # result comes from the base-frame OBB and fixed 50 mm gripper.
        width_ok = True

        target_xyz, target_source = self._target_base_xyz()
        target_ok = target_xyz is not None
        center_distance = float('inf')
        relative_z = float('nan')
        if target_xyz is not None:
            position = grasp_pose.pose.position
            delta = np.asarray(
                [float(position.x), float(position.y), float(position.z)],
                dtype=float,
            ) - np.asarray(target_xyz, dtype=float)
            center_distance = float(np.linalg.norm(delta))
            relative_z = float(delta[2])

        cloud_distance = self._camera_candidate_cloud_distance(candidate)
        max_cloud_distance = float(
            getattr(self, 'target_cloud_candidate_max_point_distance_m', 0.055) or 0.0
        )
        if np.isfinite(cloud_distance) and max_cloud_distance > 0.0:
            target_ok = target_ok and cloud_distance <= max_cloud_distance

        support_metrics = self._candidate_support_geometry_metrics(candidate) or {}
        center_clearance = float(support_metrics.get('center_clearance_m', float('nan')))
        finger_clearance = float(support_metrics.get('min_finger_clearance_m', float('nan')))
        support_minimum = float(
            getattr(self, 'target_cloud_candidate_min_support_clearance_m', -0.002)
        )
        if np.isfinite(center_clearance):
            target_ok = target_ok and center_clearance >= support_minimum

        cloud_primary = (
            np.isfinite(cloud_distance)
            and max_cloud_distance > 0.0
            and str(getattr(self, 'latest_target_cloud_segmentation', '')).startswith(
                'support_plane=inliers:'
            )
        )
        if target_xyz is not None and not cloud_primary:
            target_ok = target_ok and center_distance <= max(
                0.0,
                float(getattr(self, 'candidate_max_target_distance_m', 0.04)),
            )
            target_ok = target_ok and relative_z >= float(
                getattr(self, 'candidate_min_relative_z_m', -0.015)
            )
            target_ok = target_ok and relative_z <= float(
                getattr(self, 'candidate_max_relative_z_m', 0.08)
            )

        approach_cos = float(self._candidate_approach_downward_cos(candidate, grasp_pose))
        visibility_ok = True
        visibility_reason = 'disabled'
        if bool(getattr(self, 'camera_visibility_gate_enabled', False)) and target_xyz is not None:
            visibility_ok, _visibility_metrics, visibility_reason = self._candidate_visibility_metrics(
                grasp_pose,
                target_xyz,
            )
        return {
            'candidate_index': int(candidate_index),
            'variant_index': int(variant_index),
            'score': float(getattr(candidate, 'score', 0.0) or 0.0),
            'width_m': width,
            'depth_m': depth if np.isfinite(depth) else None,
            'translation_camera_m': self._json_vector(candidate.translation_m),
            'quaternion_camera_xyzw': self._json_vector(candidate.quaternion_xyzw),
            'depth_ok': bool(depth_ok),
            'width_ok': bool(width_ok),
            'target_ok': bool(target_ok),
            'target_source': str(target_source),
            'cloud_distance_m': cloud_distance if np.isfinite(cloud_distance) else None,
            'center_distance_m': center_distance if np.isfinite(center_distance) else None,
            'relative_z_m': relative_z if np.isfinite(relative_z) else None,
            'center_clearance_m': center_clearance if np.isfinite(center_clearance) else None,
            'finger_clearance_m': finger_clearance if np.isfinite(finger_clearance) else None,
            'jaw_normal_cos': (
                float(support_metrics['jaw_normal_cos'])
                if 'jaw_normal_cos' in support_metrics
                and np.isfinite(float(support_metrics['jaw_normal_cos']))
                else None
            ),
            'approach_cos': approach_cos,
            'visibility_ok': bool(visibility_ok),
            'visibility_reason': str(visibility_reason),
        }

    @staticmethod
    def _audit_thresholds(primary, defaults):
        values = [float(primary)] + [float(value) for value in defaults]
        return sorted(set(round(value, 6) for value in values), reverse=True)

    @staticmethod
    def _json_vector(values):
        if values is None:
            return None
        return [float(value) for value in np.asarray(values, dtype=float).reshape(-1)]

    def _write_gate_audit_report(self, report):
        path = str(getattr(self, 'gate_audit_output_path', '') or '').strip()
        if not path:
            return
        try:
            directory = os.path.dirname(path)
            if directory:
                os.makedirs(directory, exist_ok=True)
            with open(path, 'w', encoding='utf-8') as handle:
                json.dump(report, handle, ensure_ascii=True, indent=2, sort_keys=True)
        except Exception as exc:
            rospy.logwarn('remote 6D could not write gate audit %s: %s', path, exc)

    @staticmethod
    def _profile_from_summary(summary, clearance, approach):
        for profile in list(summary.get('profiles', []) or []):
            if (
                abs(float(profile.get('clearance_m', 0.0)) - float(clearance)) < 1e-9
                and abs(float(profile.get('approach_cos', 0.0)) - float(approach)) < 1e-9
            ):
                return profile
        return {}

    def _format_gate_audit_summary(self, summary):
        clearance = float(summary.get('baseline_clearance_m', 0.003))
        approach = float(summary.get('baseline_approach_cos', 0.45))
        baseline = self._profile_from_summary(summary, clearance, approach)
        no_clearance = self._profile_from_summary(summary, -0.010, approach)
        no_approach = self._profile_from_summary(summary, clearance, -0.20)
        variant_text = ','.join(
            'v%d:%d/%d'
            % (
                int(profile.get('variant_index', 0)),
                int(profile.get('safe_count', 0)),
                int(profile.get('visible_count', 0)),
            )
            for profile in list(summary.get('variant_profiles', []) or [])
        ) or 'n/a'
        return (
            'base=%d baseline-safe=%d baseline-visible=%d '
            'clearance>=-10mm:%d approach>=-0.20:%d '
            'variants-safe/visible=%s finger-clearance=%s approach=%s'
            % (
                int(summary.get('target_pass', 0)),
                int(baseline.get('pass_count', 0)),
                int(baseline.get('visible_count', 0)),
                int(no_clearance.get('pass_count', 0)),
                int(no_approach.get('pass_count', 0)),
                variant_text,
                self._compact_quantiles(summary.get('finger_clearance_quantiles_m', {}), scale=1000.0),
                self._compact_quantiles(summary.get('approach_quantiles', {})),
            )
        )

    @staticmethod
    def _compact_quantiles(values, scale=1.0):
        data = dict(values or {})
        if not data:
            return 'n/a'
        return 'p10=%.3f,p50=%.3f,p90=%.3f' % tuple(
            float(data.get(key, float('nan'))) * float(scale)
            for key in ('p10', 'p50', 'p90')
        )

    def _candidate_gate_audit_diagnostics(self):
        summary = dict(getattr(self, '_latest_gate_audit_summary', {}) or {})
        if not summary:
            return ''
        return ' [gate-audit %s]' % self._format_gate_audit_summary(summary)

    def _update_target_cloud_estimate(self, depth, obj, obj_time, stamp=None, frame_id=''):
        if obj is None or not bool(getattr(obj, 'detected', False)):
            raise RuntimeError('no detected target object for target cloud')
        if obj_time is None:
            raise RuntimeError('target cloud timestamp unavailable')
        age = (rospy.Time.now() - obj_time).to_sec()
        if age > self.roi_max_age_sec:
            raise RuntimeError('target cloud ROI is stale %.2fs > %.2fs' % (age, self.roi_max_age_sec))
        roi = expanded_bbox_roi(
            np.asarray(depth).shape,
            int(getattr(obj, 'bbox_x', 0)),
            int(getattr(obj, 'bbox_y', 0)),
            int(getattr(obj, 'bbox_width', 0)),
            int(getattr(obj, 'bbox_height', 0)),
            self.target_cloud_roi_margin_px,
        )
        foreground_mask, valid = self._foreground_mask_for_roi(depth, roi, min_points=self.target_cloud_min_points)
        if valid < self.target_cloud_min_points:
            raise RuntimeError('target cloud has too few foreground points %d < %d' % (valid, self.target_cloud_min_points))
        points_camera = self._target_cloud_points_camera(depth, roi, foreground_mask)
        if points_camera.size == 0:
            raise RuntimeError('target cloud projection produced no points')
        center_camera = np.median(points_camera, axis=0).astype(float)
        _pose_cam, pose_base = self.pose_estimator.make_poses(
            center_camera,
            stamp=stamp,
            camera_frame=frame_id or self.pose_estimator.camera_frame,
        )
        position = pose_base.pose.position
        max_points = max(1, int(getattr(self, 'target_cloud_max_points_for_gate', 2500)))
        if len(points_camera) > max_points:
            indices = np.linspace(0, len(points_camera) - 1, max_points).astype(int)
            gate_points = points_camera[indices]
        else:
            gate_points = points_camera
        self.latest_target_cloud_base_xyz = np.asarray([position.x, position.y, position.z], dtype=float)
        self.latest_target_cloud_camera_center = center_camera
        self.latest_target_cloud_camera_points = np.asarray(gate_points, dtype=np.float32)
        self.latest_target_cloud_time = rospy.Time.now()
        self.latest_target_cloud_count = int(len(points_camera))
        self.latest_target_cloud_source = 'roi_depth_foreground'
        self._target_cloud_request_active = True
        return 'target_cloud=%d center_base=(%.3f,%.3f,%.3f) %s' % (
            int(len(points_camera)),
            float(position.x),
            float(position.y),
            float(position.z),
            str(getattr(self, 'latest_target_cloud_segmentation', 'depth-window')),
        )

    def _target_cloud_points_camera(self, depth, roi, foreground_mask):
        x0, y0, x1, y1 = roi
        depth_roi = np.asarray(depth)[y0:y1, x0:x1]
        z_m = self._depth_to_meters(depth_roi)
        ys, xs = np.nonzero(foreground_mask)
        if len(xs) == 0:
            return np.zeros((0, 3), dtype=np.float32)
        intrinsics = self._camera_intrinsics()
        u = xs.astype(np.float32) + float(x0)
        v = ys.astype(np.float32) + float(y0)
        z = z_m[ys, xs].astype(np.float32)
        x_cv = (u - float(intrinsics.cx)) * z / float(intrinsics.fx)
        y_cv = (v - float(intrinsics.cy)) * z / float(intrinsics.fy)
        points_cv = np.stack([x_cv, y_cv, z], axis=1).astype(np.float32)
        return self._project_points_for_camera_frame(points_cv)

    def _project_vectors_for_camera_frame(self, vectors_cv):
        vectors = np.asarray(vectors_cv, dtype=np.float32)
        convention = normalize_candidate_frame_convention(
            getattr(self, 'target_projection_frame_convention', 'ros_camera_link')
        )
        if convention == 'opencv_optical':
            return vectors
        if convention == 'ros_camera_link':
            return vectors.dot(OPTICAL_TO_ROS_CAMERA.T)
        raise ValueError('unknown target projection frame convention: %s' % convention)

    def _depth_to_meters(self, depth_values):
        values = np.asarray(depth_values)
        if np.issubdtype(values.dtype, np.floating):
            return values.astype(np.float32)
        return values.astype(np.float32) * float(self._camera_intrinsics().depth_scale)

    def _candidate_matches_target(self, _candidate, _camera_candidate, grasp_pose):
        depth = getattr(_camera_candidate, 'depth_m', None)
        if bool(getattr(self, 'require_candidate_depth', False)):
            try:
                depth_valid = depth is not None and np.isfinite(float(depth)) and float(depth) > 0.0
            except Exception:
                depth_valid = False
            if not depth_valid:
                self._depth_gate_rejected_count += 1
                rospy.logwarn_throttle(
                    1.0,
                    'remote 6D candidate rejected: WSL response has no valid GraspNet depth_m; sync and restart graspnet_baseline_server.py',
                )
                return False
        if not getattr(self, 'candidate_target_gate_enabled', True):
            return True
        target_xyz, target_source = self._target_base_xyz()
        if target_xyz is None:
            self._target_gate_rejected_count += 1
            rospy.logwarn_throttle(1.0, 'remote 6D candidate rejected: no locked detection target')
            return False
        try:
            grasp = grasp_pose.pose.position
            dx = float(grasp.x) - float(target_xyz[0])
            dy = float(grasp.y) - float(target_xyz[1])
            dz = float(grasp.z) - float(target_xyz[2])
            distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        except Exception as exc:
            self._target_gate_rejected_count += 1
            rospy.logwarn_throttle(1.0, 'remote 6D candidate rejected: cannot compare target pose: %s', exc)
            return False
        cloud_distance = self._camera_candidate_cloud_distance(_camera_candidate)
        max_cloud_distance = float(getattr(self, 'target_cloud_candidate_max_point_distance_m', 0.055) or 0.0)
        if np.isfinite(cloud_distance):
            self._closest_candidate_cloud_distance = min(
                float(getattr(self, '_closest_candidate_cloud_distance', float('inf'))),
                float(cloud_distance),
            )
        self._closest_candidate_center_distance = min(
            float(getattr(self, '_closest_candidate_center_distance', float('inf'))),
            float(distance),
        )
        if np.isfinite(cloud_distance) and max_cloud_distance > 0.0 and cloud_distance > max_cloud_distance:
            self._target_gate_rejected_count += 1
            rospy.logwarn_throttle(
                1.0,
                (
                    'remote 6D candidate rejected by target cloud: '
                    'cloud_dist=%.3fm limit=%.3fm target_source=%s score=%.3f'
                ),
                cloud_distance,
                max_cloud_distance,
                target_source,
                float(getattr(_camera_candidate, 'score', 0.0) or 0.0),
            )
            return False
        support_point = getattr(self, 'latest_support_plane_camera_point', None)
        support_normal = getattr(self, 'latest_support_plane_camera_normal', None)
        support_clearance = None
        if support_point is not None and support_normal is not None and _camera_candidate is not None:
            try:
                support_clearance = float(
                    np.dot(
                        np.asarray(_camera_candidate.translation_m, dtype=float) - np.asarray(support_point, dtype=float),
                        np.asarray(support_normal, dtype=float),
                    )
                )
            except Exception:
                support_clearance = None
        min_support_clearance = float(
            getattr(self, 'target_cloud_candidate_min_support_clearance_m', -0.002)
        )
        if support_clearance is not None and support_clearance < min_support_clearance:
            self._target_gate_rejected_count += 1
            rospy.logwarn_throttle(
                1.0,
                'remote 6D candidate rejected below support plane: clearance=%.3fm < %.3fm score=%.3f',
                support_clearance,
                min_support_clearance,
                float(getattr(_camera_candidate, 'score', 0.0) or 0.0),
            )
            return False
        support_metrics = self._candidate_support_geometry_metrics(_camera_candidate)
        if (
            support_metrics is not None
            and not isinstance(
                getattr(_camera_candidate, '_geometry_gate_result', None),
                CandidateGateResult,
            )
        ):
            min_finger_clearance = float(
                getattr(self, 'candidate_min_finger_support_clearance_m', -float('inf'))
            )
            finger_clearance = support_metrics['min_finger_clearance_m']
            if self._candidate_support_geometry_collides(support_metrics):
                self._table_geometry_gate_rejected_count += 1
                rospy.logwarn_throttle(
                    1.0,
                    (
                        'remote 6D candidate rejected by support-plane gripper geometry: '
                        'jaw_normal=%.3f preferred<=%.3f min_finger_clearance=%.3fm limit=%.3fm '
                        'center_clearance=%.3fm model_width=%.3fm geometry_width=%.3fm score=%.3f'
                    ),
                    support_metrics['jaw_normal_cos'],
                    float(getattr(self, 'candidate_max_jaw_normal_cos', 1.0)),
                    finger_clearance,
                    min_finger_clearance,
                    support_metrics['center_clearance_m'],
                    support_metrics['model_width_m'],
                    support_metrics['geometry_width_m'],
                    float(getattr(_camera_candidate, 'score', 0.0) or 0.0),
                )
                return False
        max_distance = max(0.0, float(getattr(self, 'candidate_max_target_distance_m', 0.04)))
        min_z = float(getattr(self, 'candidate_min_relative_z_m', -0.015))
        max_z = float(getattr(self, 'candidate_max_relative_z_m', 0.08))
        cloud_primary = (
            np.isfinite(cloud_distance)
            and max_cloud_distance > 0.0
            and str(getattr(self, 'latest_target_cloud_segmentation', '')).startswith('support_plane=inliers:')
        )
        if not cloud_primary and (distance > max_distance or dz < min_z or dz > max_z):
            self._target_gate_rejected_count += 1
            rospy.logwarn_throttle(
                1.0,
                (
                    'remote 6D candidate rejected by target gate: '
                    'candidate=(%.3f, %.3f, %.3f) target=(%.3f, %.3f, %.3f) '
                    'delta=(%.3f, %.3f, %.3f) dist=%.3f limits dist<=%.3f z=[%.3f, %.3f] source=%s'
                ),
                float(grasp.x),
                float(grasp.y),
                float(grasp.z),
                float(target_xyz[0]),
                float(target_xyz[1]),
                float(target_xyz[2]),
                dx,
                dy,
                dz,
                distance,
                max_distance,
                min_z,
                max_z,
                target_source,
            )
            return False
        downward_cos = self._candidate_approach_downward_cos(_camera_candidate, grasp_pose)
        self._best_candidate_approach_cos = max(
            float(getattr(self, '_best_candidate_approach_cos', -1.0)),
            float(downward_cos),
        )
        min_downward_cos = float(getattr(self, 'candidate_min_downward_approach_cos', -1.0))
        if downward_cos < min_downward_cos:
            self._approach_gate_rejected_count += 1
            rospy.logwarn_throttle(
                1.0,
                (
                    'remote 6D candidate rejected by tabletop approach gate: '
                    'downward_cos=%.3f < %.3f score=%.3f'
                ),
                downward_cos,
                min_downward_cos,
                float(getattr(_camera_candidate, 'score', 0.0) or 0.0),
            )
            return False
        if bool(getattr(self, 'camera_visibility_gate_enabled', False)):
            visible, _metrics, reason = self._candidate_visibility_metrics(grasp_pose, target_xyz)
            if not visible:
                self._visibility_gate_rejected_count += 1
                rospy.logwarn_throttle(
                    1.0,
                    'remote 6D candidate rejected by eye-in-hand visibility gate: %s score=%.3f',
                    reason,
                    float(getattr(_camera_candidate, 'score', 0.0) or 0.0),
                )
                return False
        return True

    def _candidate_rank(self, candidate, camera_candidate, grasp_pose):
        gate = getattr(camera_candidate, '_geometry_gate_result', None)
        if isinstance(gate, CandidateGateResult):
            if not gate.ok:
                return (float('inf'),)
            metrics = getattr(self, '_candidate_plan_metrics', {}).get(
                self._pose_key(grasp_pose),
                {},
            )
            max_joint_delta = max(
                0.0,
                float(
                    getattr(self, 'candidate_max_joint_delta_rad', 0.0)
                    or 0.0
                ),
            )
            joint_delta = float(metrics.get('joint_max_delta', 0.0) or 0.0)
            if max_joint_delta > 0.0 and joint_delta > max_joint_delta:
                self._joint_motion_gate_rejected_count += 1
                rospy.logwarn_throttle(
                    1.0,
                    (
                        'remote 6D candidate rejected by joint-motion gate: '
                        'joint_max_delta=%.3frad > %.3frad'
                    ),
                    joint_delta,
                    max_joint_delta,
                )
                return (float('inf'),)
            updated_gate = candidate_with_motion_cost(
                gate,
                max(0.0, float(metrics.get('joint_path_cost', 0.0) or 0.0)),
            )
            setattr(camera_candidate, '_geometry_gate_result', updated_gate)
            return candidate_rank_key(
                updated_gate,
                float(getattr(camera_candidate, 'score', 0.0) or 0.0),
            )
        rank = self._candidate_target_distance(candidate, camera_candidate, grasp_pose)
        score_weight = max(0.0, float(getattr(self, 'candidate_model_score_weight_m', 0.0) or 0.0))
        rank -= score_weight * float(getattr(camera_candidate, 'score', 0.0) or 0.0)
        metrics = getattr(self, '_candidate_plan_metrics', {}).get(self._pose_key(grasp_pose), {})
        max_joint_delta = max(
            0.0,
            float(getattr(self, 'candidate_max_joint_delta_rad', 0.0) or 0.0),
        )
        joint_delta = float(metrics.get('joint_max_delta', 0.0) or 0.0)
        if max_joint_delta > 0.0 and joint_delta > max_joint_delta:
            self._joint_motion_gate_rejected_count += 1
            rospy.logwarn_throttle(
                1.0,
                'remote 6D candidate rejected by joint-motion gate: joint_max_delta=%.3frad > %.3frad',
                joint_delta,
                max_joint_delta,
            )
            return float('inf')
        path_weight = max(0.0, float(getattr(self, 'candidate_joint_path_cost_weight_m', 0.0) or 0.0))
        rank += path_weight * float(metrics.get('joint_path_cost', 0.0) or 0.0)
        downward_weight = max(
            0.0,
            float(getattr(self, 'candidate_downward_approach_weight_m', 0.0) or 0.0),
        )
        downward_cos = min(
            1.0,
            max(-1.0, self._candidate_approach_downward_cos(camera_candidate, grasp_pose)),
        )
        rank += downward_weight * (1.0 - downward_cos)
        support_metrics = self._candidate_support_geometry_metrics(camera_candidate)
        if support_metrics is not None:
            preferred = float(getattr(self, 'candidate_max_jaw_normal_cos', 1.0))
            normal_span = max(1e-6, 1.0 - preferred)
            normalized_excess = max(
                0.0,
                (float(support_metrics['jaw_normal_cos']) - preferred) / normal_span,
            )
            rank += max(
                0.0,
                float(getattr(self, 'candidate_jaw_tilt_rank_weight_m', 0.0)),
            ) * normalized_excess
        weight = max(0.0, float(getattr(self, 'camera_visibility_rank_weight_m', 0.0) or 0.0))
        if weight <= 0.0 or not bool(getattr(self, 'camera_visibility_gate_enabled', False)):
            return float(rank)
        target_xyz, _source = self._target_base_xyz()
        if target_xyz is None:
            return float('inf')
        visible, metrics, _reason = self._candidate_visibility_metrics(grasp_pose, target_xyz)
        if not visible or not metrics:
            return float('inf')
        center_cost = max(float(item['center_cost']) for item in metrics)
        return float(rank) + weight * center_cost

    def _tool_approach_downward_cos(self, grasp_pose):
        try:
            q = grasp_pose.pose.orientation
            matrix = quaternion_matrix([float(q.x), float(q.y), float(q.z), float(q.w)])
            name = str(self.grasp_config.get('tool_approach_axis', 'z') or 'z').strip().lower()
            sign = -1.0 if name.startswith('-') else 1.0
            name = name.lstrip('+-')
            axis_index = {'x': 0, 'y': 1, 'z': 2}[name]
            axis = sign * np.asarray(matrix[:3, axis_index], dtype=float)
            norm = float(np.linalg.norm(axis))
            if norm <= 1e-9:
                return -1.0
            return float(-axis[2] / norm)
        except Exception:
            return -1.0

    def _candidate_approach_downward_cos(self, camera_candidate, grasp_pose):
        support_normal = getattr(self, 'latest_support_plane_camera_normal', None)
        if support_normal is None or camera_candidate is None:
            return self._tool_approach_downward_cos(grasp_pose)
        try:
            matrix = quaternion_matrix(np.asarray(camera_candidate.quaternion_xyzw, dtype=float))
            name = str(self.grasp_config.get('tool_approach_axis', 'z') or 'z').strip().lower()
            sign = -1.0 if name.startswith('-') else 1.0
            axis_index = {'x': 0, 'y': 1, 'z': 2}[name.lstrip('+-')]
            approach = sign * np.asarray(matrix[:3, axis_index], dtype=float)
            normal = np.asarray(support_normal, dtype=float)
            approach /= max(float(np.linalg.norm(approach)), 1e-12)
            normal /= max(float(np.linalg.norm(normal)), 1e-12)
            # The fitted normal points away from the table toward the camera;
            # insertion toward the object/table therefore follows -normal.
            return float(np.dot(approach, -normal))
        except Exception:
            return self._tool_approach_downward_cos(grasp_pose)

    def _candidate_support_geometry_metrics(self, camera_candidate):
        """Measure a parallel-jaw candidate against the observed support plane."""
        support_point = getattr(self, 'latest_support_plane_camera_point', None)
        support_normal = getattr(self, 'latest_support_plane_camera_normal', None)
        if support_point is None or support_normal is None or camera_candidate is None:
            return None
        try:
            translation = np.asarray(camera_candidate.translation_m, dtype=float).reshape(3)
            normal = np.asarray(support_normal, dtype=float).reshape(3)
            normal /= max(float(np.linalg.norm(normal)), 1e-12)
            matrix = quaternion_matrix(np.asarray(camera_candidate.quaternion_xyzw, dtype=float))
            # GraspNet and Alicia tool0 both use +Y as the jaw-closing axis.
            jaw_axis = np.asarray(matrix[:3, 1], dtype=float)
            jaw_axis /= max(float(np.linalg.norm(jaw_axis)), 1e-12)
            jaw_normal_cos = abs(float(np.dot(jaw_axis, normal)))
            center_clearance = float(
                np.dot(translation - np.asarray(support_point, dtype=float).reshape(3), normal)
            )
            model_width = max(0.0, float(getattr(camera_candidate, 'width_m', 0.0) or 0.0))
            physical_width = max(
                0.0,
                float(getattr(self, 'gripper_physical_open_width_m', 0.0) or 0.0),
            )
            # Width filtering can be disabled for trajectory diagnostics, but
            # the real fingers still cannot move beyond their physical stroke.
            # Clamp only the support-plane geometry so a wide model proposal
            # does not create fictitious finger positions below the table.
            geometry_width = min(model_width, physical_width) if physical_width > 0.0 else model_width
            min_finger_clearance = center_clearance - 0.5 * geometry_width * jaw_normal_cos
            return {
                'jaw_normal_cos': jaw_normal_cos,
                'center_clearance_m': center_clearance,
                'min_finger_clearance_m': min_finger_clearance,
                'model_width_m': model_width,
                'geometry_width_m': geometry_width,
                # Preserve the old key for callers outside this package.
                'width_m': geometry_width,
            }
        except Exception:
            return None

    def _candidate_support_geometry_collides(self, support_metrics):
        """Reject only geometry that physically enters the observed support plane."""
        if support_metrics is None:
            return False
        minimum = float(
            getattr(self, 'candidate_min_finger_support_clearance_m', -float('inf'))
        )
        return float(support_metrics['min_finger_clearance_m']) < minimum

    @staticmethod
    def _pose_key(pose_stamped):
        try:
            pose = pose_stamped.pose
            values = (
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )
            return tuple(round(float(value), 5) for value in values)
        except Exception:
            return ()

    @staticmethod
    def _parse_plan_metrics(message):
        text = str(message or '')
        metrics = {}
        for key in ('joint_path_cost', 'joint_max_delta'):
            match = re.search(r'%s=([-+0-9.eE]+)' % key, text)
            if match:
                try:
                    metrics[key] = float(match.group(1))
                except Exception:
                    pass
        return metrics

    def _candidate_visibility_metrics(self, grasp_pose, target_base_xyz):
        try:
            tool_from_camera = self._tool_from_camera_matrix()
            plan = make_grasp_sequence_from_grasp_pose(
                grasp_pose,
                pregrasp_distance_m=float(self.grasp_config.get('pregrasp_distance_m', 0.08)),
                approach_offset_m=float(self.grasp_config.get('final_approach_offset_m', 0.015)),
                lift_height_m=float(self.grasp_config.get('lift_height_m', 0.05)),
                tool_approach_axis=str(self.grasp_config.get('tool_approach_axis', 'x')),
            )
            stages = [('pregrasp', plan.pregrasp)]
            if bool(getattr(self, 'camera_visibility_require_approach', True)):
                stages.append(('approach', plan.approach))
            intrinsics = self._camera_intrinsics()
            base_margin = max(0, int(getattr(self, 'camera_visibility_margin_px', 36)))
            min_depth = max(0.0, float(getattr(self, 'camera_visibility_min_depth_m', 0.035)))
            max_depth = max(min_depth, float(getattr(self, 'camera_visibility_max_depth_m', 1.20)))
            metrics = []
            for stage_name, tool_pose in stages:
                u, v, depth = project_base_target_at_tool_pose(
                    tool_pose,
                    target_base_xyz,
                    tool_from_camera,
                    intrinsics,
                )
                margin_x, margin_y = self._camera_visibility_margins_px(depth, base_margin)
                x_limit = max(1.0, float(intrinsics.width) * 0.5 - margin_x)
                y_limit = max(1.0, float(intrinsics.height) * 0.5 - margin_y)
                center_cost = math.sqrt(
                    ((float(u) - float(intrinsics.cx)) / x_limit) ** 2
                    + ((float(v) - float(intrinsics.cy)) / y_limit) ** 2
                ) if np.isfinite(u) and np.isfinite(v) else float('inf')
                metric = {
                    'stage': stage_name,
                    'u': float(u),
                    'v': float(v),
                    'depth_m': float(depth),
                    'center_cost': float(center_cost),
                    'margin_x_px': int(margin_x),
                    'margin_y_px': int(margin_y),
                }
                metrics.append(metric)
                inside = (
                    np.isfinite(u)
                    and np.isfinite(v)
                    and float(depth) >= min_depth
                    and float(depth) <= max_depth
                    and float(u) >= margin_x
                    and float(u) < float(intrinsics.width) - margin_x
                    and float(v) >= margin_y
                    and float(v) < float(intrinsics.height) - margin_y
                )
                if not inside:
                    return False, metrics, (
                        '%s predicts uv=(%.1f,%.1f) depth=%.3fm outside margins=(%d,%d) image=%dx%d'
                        % (
                            stage_name,
                            float(u),
                            float(v),
                            float(depth),
                            margin_x,
                            margin_y,
                            int(intrinsics.width),
                            int(intrinsics.height),
                        )
                    )
            return True, metrics, 'visible'
        except Exception as exc:
            return False, [], 'visibility transform failed: %s' % exc

    def _camera_visibility_margins_px(self, predicted_depth_m, base_margin=None):
        margin = max(
            0,
            int(
                getattr(self, 'camera_visibility_margin_px', 36)
                if base_margin is None
                else base_margin
            ),
        )
        try:
            obj, _obj_time = self._planning_object_snapshot()
        except Exception:
            obj = None
        if obj is None or not bool(getattr(obj, 'detected', False)):
            return margin, margin

        current_depth = float(getattr(obj, 'depth_m', 0.0) or 0.0)
        predicted_depth = float(predicted_depth_m)
        bbox_width = float(getattr(obj, 'bbox_width', 0.0) or 0.0)
        bbox_height = float(getattr(obj, 'bbox_height', 0.0) or 0.0)
        if (
            not np.isfinite(current_depth)
            or not np.isfinite(predicted_depth)
            or current_depth <= 1e-6
            or predicted_depth <= 1e-6
            or bbox_width <= 0.0
            or bbox_height <= 0.0
        ):
            return margin, margin

        # Preserve the complete target footprint, not only its center. Under a
        # pinhole model the apparent box size grows inversely with depth.
        scale = current_depth / predicted_depth
        padding = max(0, int(getattr(self, 'camera_visibility_bbox_padding_px', 8)))
        margin_x = max(margin, int(math.ceil(0.5 * bbox_width * scale)) + padding)
        margin_y = max(margin, int(math.ceil(0.5 * bbox_height * scale)) + padding)
        return margin_x, margin_y

    def _tool_from_camera_matrix(self):
        cached = getattr(self, '_cached_tool_from_camera', None)
        if cached is not None:
            return cached
        matrix = None
        tf_buffer = getattr(self, 'tf_buffer', None)
        if tf_buffer is not None:
            try:
                transform = tf_buffer.lookup_transform(
                    self.handeye_parent_frame,
                    self.handeye_camera_frame,
                    rospy.Time(0),
                    rospy.Duration(0.1),
                )
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                matrix = transform_matrix(
                    [translation.x, translation.y, translation.z],
                    [rotation.x, rotation.y, rotation.z, rotation.w],
                )
            except Exception as exc:
                rospy.logwarn_throttle(
                    2.0,
                    'eye-in-hand visibility TF %s <- %s unavailable; using calibrated fallback: %s',
                    self.handeye_parent_frame,
                    self.handeye_camera_frame,
                    exc,
                )
        if matrix is None:
            matrix = transform_matrix(self.handeye_translation_xyz, self.handeye_rotation_xyzw)
        self._cached_tool_from_camera = matrix
        return matrix

    def _candidate_target_distance(self, _candidate, _camera_candidate, grasp_pose):
        target_xyz, _target_source = self._target_base_xyz()
        if target_xyz is None:
            return float('inf')
        try:
            grasp = grasp_pose.pose.position
            dx = float(grasp.x) - float(target_xyz[0])
            dy = float(grasp.y) - float(target_xyz[1])
            dz = float(grasp.z) - float(target_xyz[2])
            base_distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        except Exception:
            return float('inf')
        cloud_distance = self._camera_candidate_cloud_distance(_camera_candidate)
        if np.isfinite(cloud_distance):
            center_weight = max(
                0.0,
                float(getattr(self, 'candidate_center_distance_weight', 1.0) or 0.0),
            )
            return float(cloud_distance) + center_weight * float(base_distance)
        return base_distance

    def _target_base_xyz(self):
        if bool(getattr(self, 'target_cloud_enabled', True)):
            cloud_xyz = getattr(self, 'latest_target_cloud_base_xyz', None)
            cloud_time = getattr(self, 'latest_target_cloud_time', None)
            if cloud_xyz is not None:
                fresh = bool(getattr(self, '_target_cloud_request_active', False))
                max_age = float(getattr(self, 'target_cloud_max_age_sec', 1.0) or 0.0)
                if not fresh and max_age > 0.0 and cloud_time is not None:
                    try:
                        fresh = (rospy.Time.now() - cloud_time).to_sec() <= max_age
                    except Exception:
                        fresh = False
                if fresh:
                    return np.asarray(cloud_xyz, dtype=float), getattr(self, 'latest_target_cloud_source', 'roi_depth_foreground')
        obj, _obj_time = self._planning_object_snapshot()
        if obj is None or not bool(getattr(obj, 'detected', False)):
            return None, 'none'
        try:
            p = obj.pose_base.pose.position
            return np.asarray([float(p.x), float(p.y), float(p.z)], dtype=float), 'perception_object'
        except Exception:
            return None, 'none'

    def _camera_candidate_cloud_distance(self, camera_candidate):
        if camera_candidate is None:
            return float('inf')
        points = getattr(self, 'latest_target_cloud_camera_points', None)
        if points is None:
            return float('inf')
        try:
            cloud = np.asarray(points, dtype=float)
            if cloud.size == 0:
                return float('inf')
            candidate_xyz = np.asarray(camera_candidate.translation_m, dtype=float).reshape(1, 3)
            deltas = cloud - candidate_xyz
            return float(np.min(np.linalg.norm(deltas, axis=1)))
        except Exception:
            return float('inf')

    def _refine_grasp_pose_towards_target(self, grasp_pose):
        if not bool(getattr(self, 'target_position_refine_enabled', False)):
            return grasp_pose, ''
        blend = float(getattr(self, 'target_position_refine_blend', 0.0) or 0.0)
        if blend <= 0.0:
            return grasp_pose, ''
        obj, obj_time = self._planning_object_snapshot()
        if obj is None or not bool(getattr(obj, 'detected', False)):
            return grasp_pose, ''
        if obj_time is not None:
            try:
                age = (rospy.Time.now() - obj_time).to_sec()
            except Exception:
                age = float('inf')
            max_age = float(getattr(self, 'target_position_refine_max_age_sec', 1.0) or 0.0)
            if max_age > 0.0 and age > max_age:
                rospy.logwarn_throttle(
                    1.0,
                    'remote 6D skipped target position refinement: target age %.2fs > %.2fs',
                    age,
                    max_age,
                )
                return grasp_pose, ''
        refined = deepcopy(grasp_pose)
        target = obj.pose_base.pose.position
        current = refined.pose.position
        offset = np.asarray(getattr(self, 'target_position_refine_offset_xyz_m', [0.0, 0.0, 0.0]), dtype=float)
        delta = np.asarray(
            [
                float(target.x) + float(offset[0]) - float(current.x),
                float(target.y) + float(offset[1]) - float(current.y),
                float(target.z) + float(offset[2]) - float(current.z),
            ],
            dtype=float,
        )
        raw_norm = float(np.linalg.norm(delta))
        max_step = float(getattr(self, 'target_position_refine_max_m', 0.04) or 0.0)
        clipped_norm = raw_norm
        if max_step > 0.0 and raw_norm > max_step:
            delta *= max_step / raw_norm
            clipped_norm = max_step
        applied = delta * blend
        current.x = float(current.x) + float(applied[0])
        current.y = float(current.y) + float(applied[1])
        current.z = float(current.z) + float(applied[2])
        message = (
            'refined_to_target blend=%.2f applied=(%.3f,%.3f,%.3f)m raw=%.3fm clipped=%.3fm'
            % (
                blend,
                float(applied[0]),
                float(applied[1]),
                float(applied[2]),
                raw_norm,
                clipped_norm,
            )
        )
        rospy.loginfo('remote 6D %s', message)
        return refined, message

    def _camera_intrinsics(self):
        cam_cfg = rospy.get_param('/camera', {})
        return CameraIntrinsics(
            width=int(cam_cfg.get('width', 640)),
            height=int(cam_cfg.get('height', 480)),
            fx=float(cam_cfg.get('fx', 615.0)),
            fy=float(cam_cfg.get('fy', 615.0)),
            cx=float(cam_cfg.get('cx', cam_cfg.get('width', 640) / 2.0)),
            cy=float(cam_cfg.get('cy', cam_cfg.get('height', 480) / 2.0)),
            depth_scale=float(cam_cfg.get('depth_scale', 0.001)),
        )

    def _project_points_for_camera_frame(self, points_cv):
        points = np.asarray(points_cv, dtype=np.float32)
        if self.target_projection_frame_convention == 'opencv_optical':
            return points
        if self.target_projection_frame_convention == 'ros_camera_link':
            converted = np.empty_like(points)
            converted[:, 0] = points[:, 2]
            converted[:, 1] = -points[:, 0]
            converted[:, 2] = -points[:, 1]
            return converted
        rospy.logwarn_throttle(
            2.0,
            'Unknown target_projection_frame_convention=%s; using OpenCV optical coordinates',
            self.target_projection_frame_convention,
        )
        return points

    @staticmethod
    def _clamp01(value):
        try:
            return min(1.0, max(0.0, float(value)))
        except Exception:
            return 0.0

    @staticmethod
    def _clamp_range(value, low, high):
        try:
            number = float(value)
        except Exception:
            number = float(low)
        return min(float(high), max(float(low), number))

    @staticmethod
    def _parse_xyz_param(value):
        if isinstance(value, str):
            parts = [float(part.strip()) for part in value.replace(',', ' ').split() if part.strip()]
        else:
            parts = [float(part) for part in list(value or [])]
        if len(parts) != 3:
            raise ValueError('xyz parameter must contain exactly 3 values')
        return parts

    def _plan_reachable(self, grasp_pose):
        try:
            strict_pose = (
                not bool(getattr(self, 'allow_position_only_fallback', False))
                and not bool(getattr(self, 'allow_orientation_fallback', False))
            )
            service_name = '/supervisor/check_pose_strict' if strict_pose else '/supervisor/move_to_pose'
            rospy.wait_for_service(service_name, timeout=0.25)
            move_pose = rospy.ServiceProxy(service_name, SetTargetPose)
            response = move_pose(grasp_pose, False)
            if not bool(response.success):
                return False
            metrics = self._parse_plan_metrics(getattr(response, 'message', ''))
            if not hasattr(self, '_candidate_plan_metrics'):
                self._candidate_plan_metrics = {}
            self._candidate_plan_metrics[self._pose_key(grasp_pose)] = metrics
            if is_position_only_fallback_message(getattr(response, 'message', '')) and not self.allow_position_only_fallback:
                self._position_only_rejected_count += 1
                rospy.logwarn_throttle(
                    2.0,
                    'remote 6D candidate rejected: position-only fallback is not executable: %s',
                    getattr(response, 'message', ''),
                )
                return False
            if is_orientation_fallback_message(getattr(response, 'message', '')) and not self.allow_orientation_fallback:
                self._orientation_fallback_rejected_count += 1
                rospy.logwarn_throttle(
                    2.0,
                    'remote 6D candidate rejected: candidate orientation fallback is not executable: %s',
                    getattr(response, 'message', ''),
                )
                return False
            return True
        except Exception:
            return False

    def _check_remote_health(self):
        try:
            health = self.client.health()
        except Exception as exc:
            rospy.logwarn('remote 6D health check failed: %s', exc)
            self.status_pub.publish(String('remote 6D health check failed: %s' % exc))
            return
        if bool(health.get('ok', False)):
            backend_health = resolve_grasp_backend_health(health)
            candidate_fields = set(str(item) for item in (backend_health.get('candidate_fields') or []))
            if bool(getattr(self, 'require_candidate_depth', False)) and 'depth_m' not in candidate_fields:
                message = (
                    'remote 6D server protocol is outdated: depth_m is missing; '
                    'sync tools/graspnet_baseline_server.py to WSL and restart port 8000'
                )
                rospy.logwarn('%s', message)
                self.status_pub.publish(String(message))
                return
            rospy.loginfo(
                'remote 6D server online: backend=%s loaded=%s protocol=%s url=%s',
                backend_health.get('backend', health.get('backend', 'unknown')),
                backend_health.get('loaded', health.get('loaded', 'unknown')),
                backend_health.get('protocol_version', health.get('protocol_version', 'unknown')),
                self.client.server_url,
            )
        else:
            rospy.logwarn('remote 6D server unhealthy: %s', health)
            self.status_pub.publish(String('remote 6D server unhealthy: %s' % health))

    def _publish_error(self, message):
        if message != self.last_error:
            self.last_error = message
            rospy.logwarn('%s', message)
            self.status_pub.publish(String(message))
        else:
            rospy.logwarn_throttle(5.0, '%s', message)


if __name__ == '__main__':
    rospy.init_node('remote_grasp6d_node')
    RemoteGrasp6DNode().spin()
