#!/usr/bin/env python3
import hashlib
import json
import math
import os
import re
import socket
import threading
import time
import urllib.error
from collections import Counter, deque
from copy import deepcopy
from dataclasses import asdict, dataclass, replace

import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseArray, PoseStamped
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler, quaternion_from_matrix, quaternion_matrix, quaternion_multiply

try:
    import tf2_ros
except Exception:
    tf2_ros = None

from alicia_flexible_grasp.grasp.grasp6d_sequence import make_grasp_sequence_from_grasp_pose
from alicia_flexible_grasp.grasp.rich_plan_integrity import (
    compute_plan_id,
    required_open_width_is_valid,
    validate_plan_header_binding,
    validate_rich_geometry,
)
from alicia_flexible_grasp.grasp.gripper_geometry import (
    CandidateGateResult,
    GripperGeometry,
    candidate_rank_key,
    candidate_with_motion_cost,
    evaluate_candidate,
    gripper_contract_mismatch_reason,
)
from alicia_flexible_grasp.grasp.grasp6d_stability import (
    CandidateObservation,
    CandidateTracker,
    StableCandidate,
    TrackingConfig,
)
from alicia_flexible_grasp.grasp.grasp6d_pipeline import (
    CandidateStageFunnel,
    ExecutionPlanController,
    MoveItResult,
    PromotionDecision,
    SafetyGateInput,
    ScoredStableCandidate,
    SoftCandidateFeatures,
    SoftScoreWeights,
    bounded_moveit_select,
    mandatory_safety_gate,
    soft_candidate_cost,
)
from alicia_flexible_grasp.robot.planning_feedback import (
    is_orientation_fallback_message,
    is_position_only_fallback_message,
)
from alicia_flexible_grasp.vision.grasp6d_adapter import CameraIntrinsics
from alicia_flexible_grasp.vision.graspnet_input_context import (
    CONTEXT_ROI,
    DEFAULT_CONTEXT_EXPAND_RATIO,
    DEFAULT_CONTEXT_MARGIN_PX,
    DEFAULT_CONTEXT_MAX_MARGIN_PX,
    DEFAULT_CONTEXT_PLANE_DISTANCE_M,
    DEFAULT_DETECTED_BBOX_MIN_IOU,
    DEFAULT_MIN_SUPPORT_POINTS,
    DEFAULT_MIN_TARGET_FRACTION,
    DEFAULT_MIN_TARGET_POINTS,
    DEFAULT_MIN_TOTAL_POINTS,
    DEFAULT_TARGET_GUARD_PX,
    FULL_SCENE,
    MASKED_TARGET,
    VALID_MODES,
    GraspNetInputContextError,
    build_graspnet_input_context,
)
from alicia_flexible_grasp.vision.model_selection import select_yolo_model
from alicia_flexible_grasp.vision.latest_only_inference import (
    LatestOnlyInferenceCoordinator,
)
from alicia_flexible_grasp.vision.object_geometry import (
    GeometryEstimate,
    estimate_object_geometry,
)
from alicia_flexible_grasp.vision.pose_estimator import PoseEstimator
from alicia_flexible_grasp.vision.remote_grasp6d_client import (
    CandidateContractError,
    RemoteGrasp6DClient,
    RemoteGraspCandidate,
    validate_graspnet_depth_m,
)
from alicia_flexible_grasp.vision.rgbd_snapshot import (
    SnapshotResult,
    SynchronizedRgbdBuffer,
    fuse_stable_samples,
)
from alicia_flexible_grasp_supervisor.msg import (
    Grasp6DPlan,
    GraspState,
    ObjectGeometry,
    ObjectPose,
)
from alicia_flexible_grasp_supervisor.srv import SetTargetPose, TriggerZero, TriggerZeroResponse


OPTICAL_TO_ROS_CAMERA = np.asarray(
    [
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ],
    dtype=float,
)
PRODUCTION_CANDIDATE_FRAME_CONVENTION = 'opencv_optical'
CANONICAL_CANDIDATE_CAMERA_FRAME = 'camera_link'
STRICT_MODEL_GRASP_TO_TOOL_QUATERNION = np.asarray(
    quaternion_from_euler(0.0, math.pi * 0.5, 0.0),
    dtype=float,
)
STRICT_MODEL_GRASP_TO_TOOL_ROTATION = quaternion_matrix(
    STRICT_MODEL_GRASP_TO_TOOL_QUATERNION
)[:3, :3]
TOOL_Z_HALF_TURN_ROTATION = quaternion_matrix(
    quaternion_from_euler(0.0, 0.0, math.pi)
)[:3, :3]
STRICT_ORIENTATION_VARIANTS_RPY_DEG = (
    (0.0, 0.0, 0.0),
    (0.0, 0.0, 180.0),
)
PIPELINE_COUNTER_FIELDS = (
    'submitted',
    'started',
    'completed',
    'accepted',
    'failed',
    'expired',
    'stale',
    'replaced',
    'busy',
)
PIPELINE_METRICS_MAX_BYTES = 12000


class StreamResultCancelled(RuntimeError):
    def __init__(self, code):
        self.code = str(code or 'GENERATION_STALE')
        super().__init__(self.code)


def _finite_pipeline_number(value, default=0.0):
    try:
        converted = float(value)
    except (TypeError, ValueError, OverflowError):
        return float(default)
    return converted if math.isfinite(converted) else float(default)


def _pipeline_count(value):
    if isinstance(value, bool):
        return 0
    try:
        converted = int(value)
    except (TypeError, ValueError, OverflowError):
        return 0
    return max(0, converted)


def _rolling_latency_percentiles(latency_history_ms):
    finite = []
    for value in tuple(latency_history_ms or ())[-100:]:
        try:
            converted = float(value)
        except (TypeError, ValueError, OverflowError):
            continue
        if math.isfinite(converted):
            finite.append(converted)
    if not finite:
        return 0.0, 0.0
    return (
        float(np.percentile(finite, 50.0)),
        float(np.percentile(finite, 95.0)),
    )


def build_pipeline_metrics(
    *,
    event,
    request_id,
    generation,
    target_epoch,
    snapshot_stamp_sec,
    status,
    drop_reason,
    counters,
    pending_replacements,
    ros_prepare_ms,
    transport_ms,
    decode_ms,
    remote_performance,
    end_to_end_ms,
    result_age_ms,
    latency_history_ms,
    funnel=None,
    encode_ms=0.0,
):
    """Build one stable, finite operational event dictionary."""

    counters = dict(counters or {})
    performance = dict(remote_performance or {})
    funnel = dict(funnel or {})
    p50_ms, p95_ms = _rolling_latency_percentiles(latency_history_ms)
    output = {
        'event': str(event or ''),
        'request_id': _pipeline_count(request_id),
        'generation': _pipeline_count(generation),
        'target_epoch': _pipeline_count(target_epoch),
        'snapshot_stamp_sec': _finite_pipeline_number(snapshot_stamp_sec),
        'status': str(status or ''),
        'drop_reason': str(drop_reason or ''),
        'pending_replacements': _pipeline_count(pending_replacements),
        'ros_prepare_ms': _finite_pipeline_number(ros_prepare_ms),
        'encode_ms': _finite_pipeline_number(encode_ms),
        'transport_ms': _finite_pipeline_number(transport_ms),
        'decode_ms': _finite_pipeline_number(decode_ms),
        'wsl_preprocess_ms': _finite_pipeline_number(
            performance.get('preprocess_ms', 0.0)
        ),
        'wsl_inference_ms': _finite_pipeline_number(
            performance.get('inference_ms', 0.0)
        ),
        'wsl_postprocess_ms': _finite_pipeline_number(
            performance.get('postprocess_ms', 0.0)
        ),
        'wsl_total_ms': _finite_pipeline_number(
            performance.get('server_total_ms', 0.0)
        ),
        'end_to_end_ms': _finite_pipeline_number(end_to_end_ms),
        'result_age_ms': _finite_pipeline_number(result_age_ms),
        'latency_p50_ms': _finite_pipeline_number(p50_ms),
        'latency_p95_ms': _finite_pipeline_number(p95_ms),
        'gpu_allocated_mb': _finite_pipeline_number(
            performance.get('gpu_allocated_mb', 0.0)
        ),
        'gpu_reserved_mb': _finite_pipeline_number(
            performance.get('gpu_reserved_mb', 0.0)
        ),
        'gpu_peak_allocated_mb': _finite_pipeline_number(
            performance.get('gpu_peak_allocated_mb', 0.0)
        ),
        'stage_counts': dict(funnel.get('stage_counts', {}) or {}),
        'rejection_counts': dict(
            funnel.get('rejection_counts', {}) or {}
        ),
        'rejection_ratios': dict(
            funnel.get('rejection_ratios', {}) or {}
        ),
        'primary_failure': funnel.get('primary_failure'),
    }
    for name in PIPELINE_COUNTER_FIELDS:
        output[name] = _pipeline_count(counters.get(name, 0))
    return output


def strict_metrics_json(metrics):
    """Serialize operational metrics without NaN/Infinity extensions."""

    return json.dumps(
        dict(metrics or {}),
        allow_nan=False,
        sort_keys=True,
        separators=(',', ':'),
    )


def _metrics_original_totals(payload):
    stages = dict(payload.get('stage_counts', {}) or {})
    rejection_counts = dict(payload.get('rejection_counts', {}) or {})

    def stage_total(field):
        return sum(
            _pipeline_count(dict(value or {}).get(field, 0))
            for value in stages.values()
            if isinstance(value, dict)
        )

    return {
        'stage_count': len(stages),
        'stage_entered_total': stage_total('entered'),
        'stage_passed_total': stage_total('passed'),
        'stage_rejected_total': stage_total('rejected'),
        'rejection_code_count': len(rejection_counts),
        'rejection_total': sum(
            _pipeline_count(value) for value in rejection_counts.values()
        ),
    }


def bounded_metrics_json(metrics, max_bytes=PIPELINE_METRICS_MAX_BYTES):
    """Return valid strict JSON bounded for the ROS operational topic."""

    limit = max(512, int(max_bytes))
    payload = dict(metrics or {})
    encoded = strict_metrics_json(payload)
    if len(encoded.encode('utf-8')) <= limit:
        return encoded
    payload['metrics_truncated'] = True
    payload['metrics_original_totals'] = _metrics_original_totals(payload)
    if 'error' in payload:
        payload['error'] = str(payload['error'])[:512]
        encoded = strict_metrics_json(payload)
        if len(encoded.encode('utf-8')) <= limit:
            return encoded
    # Full stage/audit data remains in ``pipeline_metrics`` and the atomic
    # audit.  The operational topic retains stable field names and totals.
    for field in ('stage_counts', 'rejection_ratios', 'rejection_counts'):
        values = dict(payload.get(field, {}) or {})
        payload[field] = {
            key: values[key]
            for key in sorted(values)[:32]
        }
        encoded = strict_metrics_json(payload)
        if len(encoded.encode('utf-8')) <= limit:
            return encoded
    payload['stage_counts'] = {}
    payload['rejection_ratios'] = {}
    payload['rejection_counts'] = {}
    encoded = strict_metrics_json(payload)
    if len(encoded.encode('utf-8')) <= limit:
        return encoded
    payload.pop('error', None)
    for field in ('status', 'drop_reason', 'primary_failure', 'event'):
        payload[field] = str(payload.get(field, '') or '')[:128]
    encoded = strict_metrics_json(payload)
    if len(encoded.encode('utf-8')) <= limit:
        return encoded
    minimal = {
        'event': 'metrics_truncated',
        'request_id': _pipeline_count(payload.get('request_id', 0)),
        'generation': _pipeline_count(payload.get('generation', 0)),
        'target_epoch': _pipeline_count(payload.get('target_epoch', 0)),
        'metrics_truncated': True,
        'metrics_original_totals': dict(
            payload.get('metrics_original_totals', {}) or {}
        ),
    }
    for field in PIPELINE_COUNTER_FIELDS:
        if field in payload:
            minimal[field] = _pipeline_count(payload.get(field, 0))
    audit_reference = dict(payload.get('audit_reference', {}) or {})
    if audit_reference:
        minimal['audit_reference'] = audit_reference
    encoded = strict_metrics_json(minimal)
    if len(encoded.encode('utf-8')) > limit and audit_reference:
        minimal['audit_reference'] = {
            'report_sha256': str(
                audit_reference.get('report_sha256', '') or ''
            ),
            'row_count': _pipeline_count(
                audit_reference.get('row_count', 0)
            ),
        }
        encoded = strict_metrics_json(minimal)
    if len(encoded.encode('utf-8')) > limit:
        raise ValueError('pipeline metrics byte limit is too small')
    return encoded


@dataclass(frozen=True)
class FrozenGraspNetInputConfig:
    """One request's immutable GraspNet RGB-D input contract."""

    mode: str = MASKED_TARGET
    context_margin_px: object = DEFAULT_CONTEXT_MARGIN_PX
    context_expand_ratio: object = DEFAULT_CONTEXT_EXPAND_RATIO
    context_max_margin_px: object = DEFAULT_CONTEXT_MAX_MARGIN_PX
    target_guard_px: object = DEFAULT_TARGET_GUARD_PX
    support_band_m: object = DEFAULT_CONTEXT_PLANE_DISTANCE_M
    min_target_points: object = DEFAULT_MIN_TARGET_POINTS
    min_support_points: object = DEFAULT_MIN_SUPPORT_POINTS
    min_total_points: object = DEFAULT_MIN_TOTAL_POINTS
    min_target_fraction: object = DEFAULT_MIN_TARGET_FRACTION
    bbox_min_iou: object = DEFAULT_DETECTED_BBOX_MIN_IOU
    candidate_target_gate_enabled: bool = True

    @property
    def requires_instance_mask(self):
        return self.mode in (CONTEXT_ROI, FULL_SCENE)

    @property
    def requires_support_plane(self):
        return self.mode == CONTEXT_ROI

    @property
    def requires_candidate_target_gate(self):
        return self.mode == CONTEXT_ROI


@dataclass(frozen=True)
class PreparedPrediction:
    ticket: object
    snapshot: SnapshotResult
    stamp: object
    geometry: object
    pose_estimator: object
    graspnet_input: object
    candidates: tuple
    remote_diagnostics: dict
    remote_performance: dict
    graspnet_input_audit: dict = None
    request_invalidation_generation: int = 0
    graspnet_input_config: object = None
    model_choice: str = ''
    ros_prepare_ms: float = 0.0
    encode_ms: float = 0.0
    transport_ms: float = 0.0
    decode_ms: float = 0.0


@dataclass(frozen=True)
class LocalCandidatePayload:
    raw_candidate_index: int
    variant_index: int
    raw_candidate: object
    camera_candidate: object
    grasp_pose: object
    geometry_gate: CandidateGateResult
    soft_features: SoftCandidateFeatures
    score_components: dict


FULL_SCENE_DIAGNOSTIC_CODE = 'GRASPNET_FULL_SCENE_DIAGNOSTIC_ONLY'
PLANNING_AUDIT_CONFIG_INVALID = 'PLANNING_AUDIT_CONFIG_INVALID'
PLANNING_AUDIT_FAILED = 'PLANNING_AUDIT_FAILED'
PLANNING_AUDIT_WRITE_FAILED = 'PLANNING_AUDIT_WRITE_FAILED'
AUDIT_PATH_CONFLICT = 'AUDIT_PATH_CONFLICT'
MUJOCO_AUDIT_DEFAULT_PATH = '~/.ros/grasp6d_mujoco_audit_latest.json'


def validate_mandatory_planning_audit(enabled, output_path):
    """Validate the production planning-audit authority contract."""
    if type(enabled) is not bool or enabled is not True:
        raise CandidateContractError(
            PLANNING_AUDIT_CONFIG_INVALID,
            'gate_audit_enabled must be the boolean true',
        )
    if not isinstance(output_path, str):
        raise CandidateContractError(
            PLANNING_AUDIT_CONFIG_INVALID,
            'gate_audit_output_path must be a string',
        )
    path = os.path.expanduser(output_path.strip())
    if not path:
        raise CandidateContractError(
            PLANNING_AUDIT_CONFIG_INVALID,
            'gate_audit_output_path must be non-empty',
        )
    return path


def validate_distinct_audit_paths(planning_output_path, mujoco_output_path):
    """Return canonical audit paths and reject shared file authority."""
    planning_path = validate_mandatory_planning_audit(
        True,
        planning_output_path,
    )
    if not isinstance(mujoco_output_path, str) or not mujoco_output_path.strip():
        raise CandidateContractError(
            PLANNING_AUDIT_CONFIG_INVALID,
            'mujoco_digital_twin.audit_output_path must be a non-empty string',
        )
    mujoco_path = os.path.expanduser(mujoco_output_path.strip())
    planning_path = os.path.realpath(os.path.abspath(planning_path))
    mujoco_path = os.path.realpath(os.path.abspath(mujoco_path))
    same_path = os.path.normcase(planning_path) == os.path.normcase(mujoco_path)
    if not same_path and os.path.exists(planning_path) and os.path.exists(mujoco_path):
        try:
            same_path = os.path.samefile(planning_path, mujoco_path)
        except OSError:
            same_path = False
    if same_path:
        raise CandidateContractError(
            AUDIT_PATH_CONFLICT,
            'planning and MuJoCo execution audits must use different files: %s'
            % planning_path,
        )
    return planning_path, mujoco_path


def candidate_gate_result_audit(result):
    """Return the complete six-stage analytical result as JSON data."""
    stage_names = (
        'transform',
        'center',
        'jaw_width',
        'finger_reach',
        'static_envelope',
        'swept_envelope',
    )
    if not isinstance(result, CandidateGateResult):
        return {
            'checked': False,
            'ok': False,
            'failure_code': '',
            'failure_reason': '',
            'failed_gate': '',
            'passed_gate_count': 0,
            'total_gate_count': 6,
            'stages': [
                {'name': name, 'status': 'not_checked'}
                for name in stage_names
            ],
        }
    passed = int(result.passed_gate_count)
    stages = []
    for index, name in enumerate(stage_names):
        if index < passed:
            status = 'passed'
        elif not bool(result.ok) and (
            name == str(result.failed_gate or '') or index == passed
        ):
            status = 'failed'
        else:
            status = 'not_checked'
        stages.append({'name': name, 'status': status})
    return {
        'checked': True,
        'ok': bool(result.ok),
        'failure_code': str(result.failure_code or ''),
        'failure_reason': str(result.failure_reason or ''),
        'failed_gate': str(result.failed_gate or ''),
        'passed_gate_count': passed,
        'total_gate_count': 6,
        'stages': stages,
        'required_open_width_m': float(result.required_open_width_m),
        'center_distance_m': float(result.center_distance_m),
        'support_clearance_m': float(result.support_clearance_m),
        'jaw_alignment': float(result.jaw_alignment),
        'motion_cost': float(result.motion_cost),
        'geometry_cost': float(result.geometry_cost),
    }


def failed_planning_evaluation(
    candidate_index,
    variant_index,
    failure_code,
    failure_reason,
    stage='audit_finalization',
):
    """Build one complete fail-closed selector evaluation record."""
    return {
        'candidate_index': int(candidate_index),
        'variant_index': int(variant_index),
        'candidate_contract': {
            'ok': False,
            'stage': str(stage),
            'failure_code': str(failure_code),
            'failure_reason': str(failure_reason),
        },
        'analytical_result': candidate_gate_result_audit(None),
        'target_filter': {
            'checked': False,
            'ok': False,
            'failure_code': '',
            'failure_reason': '',
        },
        'strict_reachability': {
            'checked': False,
            'ok': False,
            'failure_code': '',
            'failure_reason': '',
        },
        'rank': None,
        'rank_valid': False,
        'selected': False,
    }


def planning_evaluation_schema_error(evaluation, expected_key):
    """Return an empty string only for a complete lineage-bound record."""
    if not isinstance(evaluation, dict):
        return 'planning evaluation is not a dictionary'
    required = {
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
    missing = sorted(required - set(evaluation))
    if missing:
        return 'planning evaluation is missing fields %s' % missing
    try:
        actual_key = (
            int(evaluation.get('candidate_index')),
            int(evaluation.get('variant_index')),
        )
    except (TypeError, ValueError, OverflowError):
        return 'planning evaluation has non-integral lineage indices'
    if actual_key != tuple(expected_key):
        return 'planning evaluation lineage %s does not match row %s' % (
            actual_key,
            tuple(expected_key),
        )
    nested_required = {
        'candidate_contract': {'ok', 'stage', 'failure_code', 'failure_reason'},
        'analytical_result': {
            'checked',
            'ok',
            'failure_code',
            'failure_reason',
            'failed_gate',
            'passed_gate_count',
            'total_gate_count',
            'stages',
        },
        'target_filter': {
            'checked',
            'ok',
            'failure_code',
            'failure_reason',
        },
        'strict_reachability': {
            'checked',
            'ok',
            'failure_code',
            'failure_reason',
        },
    }
    for name, fields in nested_required.items():
        value = evaluation.get(name)
        if not isinstance(value, dict):
            return '%s is not a dictionary' % name
        missing_nested = sorted(fields - set(value))
        if missing_nested:
            return '%s is missing fields %s' % (name, missing_nested)
    if type(evaluation.get('selected')) is not bool:
        return 'selected must be an exact boolean'
    if type(evaluation.get('rank_valid')) is not bool:
        return 'rank_valid must be an exact boolean'
    return ''


def array_sha256(value):
    """Hash exact C-order payload bytes for the gate audit."""
    return hashlib.sha256(np.ascontiguousarray(value).tobytes(order='C')).hexdigest()


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
        if isinstance(
            current,
            (CandidateContractError, GraspNetInputContextError),
        ):
            return str(current.code)
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


def validate_production_candidate_frame_convention(convention):
    """Keep the GraspNet wire result in its one production coordinate frame."""
    normalized = normalize_candidate_frame_convention(convention)
    if normalized != PRODUCTION_CANDIDATE_FRAME_CONVENTION:
        raise CandidateContractError(
            'CANDIDATE_FRAME_CONVENTION_INVALID',
            (
                'production GraspNet candidates are fixed to opencv_optical; '
                'got %s'
            )
            % normalized,
        )
    return PRODUCTION_CANDIDATE_FRAME_CONVENTION


def validate_production_execution_fallback_contract(
    accept_position_only_fallback,
    accept_orientation_fallback,
):
    """Forbid pose replacement in the production remote 6D path."""
    if bool(accept_position_only_fallback):
        raise CandidateContractError(
            'POSITION_ONLY_FALLBACK_FORBIDDEN',
            (
                'production remote 6D planning cannot enable '
                'accept_position_only_fallback'
            ),
        )
    if bool(accept_orientation_fallback):
        raise CandidateContractError(
            'ORIENTATION_FALLBACK_FORBIDDEN',
            (
                'production remote 6D planning cannot enable '
                'accept_orientation_fallback'
            ),
        )
    return False, False


def validate_production_orientation_variant_quaternions(variants):
    """Freeze the one exact ordered parallel-jaw symmetry contract."""
    try:
        values = list([] if variants is None else variants)
    except Exception as exc:
        raise CandidateContractError(
            'ORIENTATION_VARIANT_CONTRACT_INVALID',
            'orientation variants must be an ordered two-item sequence',
        ) from exc
    if len(values) != 2:
        raise CandidateContractError(
            'ORIENTATION_VARIANT_CONTRACT_INVALID',
            (
                'production orientation variants must contain exactly two '
                'items: identity followed by tool Rz(180 deg)'
            ),
        )
    try:
        normalized = tuple(
            _normalize_quaternion(np.asarray(value, dtype=float))
            for value in values
        )
    except (TypeError, ValueError, OverflowError) as exc:
        raise CandidateContractError(
            'ORIENTATION_VARIANT_CONTRACT_INVALID',
            'orientation variants must be finite non-zero quaternions',
        ) from exc

    rotations = tuple(
        quaternion_matrix(quaternion)[:3, :3]
        for quaternion in normalized
    )
    expected = (np.eye(3, dtype=float), TOOL_Z_HALF_TURN_ROTATION)
    if not all(
        np.allclose(actual, required, atol=1e-7)
        for actual, required in zip(rotations, expected)
    ):
        raise CandidateContractError(
            'ORIENTATION_VARIANT_CONTRACT_INVALID',
            (
                'production orientation variants must be unique and ordered '
                'as identity, tool Rz(180 deg); no other wrist rotations are allowed'
            ),
        )

    frozen = []
    for quaternion in normalized:
        item = np.array(quaternion, dtype=float, copy=True)
        item.setflags(write=False)
        frozen.append(item)
    return tuple(frozen)


def validate_execution_tool0_contract(
    require_candidate_depth,
    model_grasp_to_tool_quaternion,
    grasp_config,
):
    """Reject any runtime configuration that can reinterpret center as tool0."""
    if not bool(require_candidate_depth):
        raise CandidateContractError(
            'DEPTH_CONTRACT_DISABLED',
            'production remote 6D planning cannot disable candidate depth',
        )
    correction = _normalize_quaternion(
        np.asarray(model_grasp_to_tool_quaternion, dtype=float)
    )
    if not np.allclose(
        quaternion_matrix(correction)[:3, :3],
        STRICT_MODEL_GRASP_TO_TOOL_ROTATION,
        atol=1e-7,
    ):
        raise CandidateContractError(
            'TOOL_FRAME_CONTRACT_INVALID',
            'production model_grasp_to_tool must be exactly Ry(+90 deg)',
        )
    approach_axis = str(
        dict(grasp_config or {}).get('tool_approach_axis', 'z') or 'z'
    ).strip().lower()
    if approach_axis not in ('z', '+z'):
        raise CandidateContractError(
            'TOOL_FRAME_CONTRACT_INVALID',
            'production grasp tool_approach_axis must be +Z',
        )
    return correction


def convert_candidate_to_camera_link(candidate, convention='opencv_optical'):
    convention = normalize_candidate_frame_convention(convention)
    if convention == 'ros_camera_link':
        return candidate
    if convention != 'opencv_optical':
        raise ValueError('unknown remote grasp candidate frame convention: %s' % convention)

    translation = OPTICAL_TO_ROS_CAMERA.dot(
        _finite_vector3(candidate.translation_m, 'candidate center')
    )
    tool0_translation = getattr(candidate, 'tool0_translation_m', None)
    if tool0_translation is not None:
        tool0_translation = OPTICAL_TO_ROS_CAMERA.dot(
            _finite_vector3(tool0_translation, 'candidate tool0')
        )
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
        tool0_translation_m=tool0_translation,
    )


def align_candidate_to_tool_frame(
    candidate,
    model_grasp_to_tool_quaternion=None,
    require_depth=True,
    legacy_nonexecuting=False,
):
    """Convert model orientation and derive tool0 without consuming depth twice.

    ``translation_m`` remains GraspNet's grasp center.  GraspNet insertion
    depth is measured along model +X, so the physical Alicia tool0 origin is
    ``center + depth * R_model[:, 0]``.  With the configured Ry(+90 deg)
    model-to-tool correction this is also ``center + depth * R_tool[:, 2]``.
    """
    correction = np.asarray(
        model_grasp_to_tool_quaternion
        if model_grasp_to_tool_quaternion is not None
        else STRICT_MODEL_GRASP_TO_TOOL_QUATERNION,
        dtype=float,
    )
    correction = _normalize_quaternion(correction)
    strict = not bool(legacy_nonexecuting)
    if strict and not bool(require_depth):
        raise CandidateContractError(
            'DEPTH_CONTRACT_DISABLED',
            'execution candidates cannot disable the GraspNet depth contract',
        )
    if strict:
        correction_rotation = quaternion_matrix(correction)[:3, :3]
        if not np.allclose(
            correction_rotation,
            STRICT_MODEL_GRASP_TO_TOOL_ROTATION,
            atol=1e-7,
        ):
            raise CandidateContractError(
                'TOOL_FRAME_CONTRACT_INVALID',
                'model_grasp_to_tool must be exactly Ry(+90 deg)',
            )
    center = _finite_vector3(candidate.translation_m, 'candidate center')
    model_rotation = quaternion_matrix(
        _normalize_quaternion(np.asarray(candidate.quaternion_xyzw, dtype=float))
    )[:3, :3]
    quaternion = _normalize_quaternion(
        np.asarray(quaternion_multiply(candidate.quaternion_xyzw, correction), dtype=float)
    )
    tool_rotation = quaternion_matrix(quaternion)[:3, :3]

    depth = validate_graspnet_depth_m(
        getattr(candidate, 'depth_m', None),
        required=bool(require_depth),
    )
    existing_tool0 = getattr(candidate, 'tool0_translation_m', None)
    if depth is None:
        if strict:
            raise CandidateContractError(
                'DEPTH_MISSING',
                'execution candidate is missing GraspNet depth_m',
            )
        tool0_translation = (
            center.copy()
            if existing_tool0 is None
            else _finite_vector3(existing_tool0, 'candidate tool0')
        )
    else:
        expected_model_tool0 = _finite_vector3(
            center + depth * model_rotation[:, 0],
            'derived candidate tool0 from model +X',
        )
        expected_tool_tool0 = _finite_vector3(
            center + depth * tool_rotation[:, 2],
            'derived candidate tool0 from tool +Z',
        )
        if strict and not np.allclose(
            expected_model_tool0,
            expected_tool_tool0,
            atol=1e-7,
        ):
            raise CandidateContractError(
                'TOOL_FRAME_CONTRACT_INVALID',
                'Ry(+90 deg) must map Alicia tool +Z to GraspNet model +X',
            )
        if existing_tool0 is None:
            tool0_translation = expected_model_tool0
        else:
            tool0_translation = _finite_vector3(
                existing_tool0,
                'candidate tool0',
            )
            if not np.allclose(
                tool0_translation,
                expected_model_tool0,
                atol=1e-7,
            ):
                raise CandidateContractError(
                    'TOOL0_INCONSISTENT',
                    'candidate tool0 is inconsistent with center/depth/model approach'
                )
    return RemoteGraspCandidate(
        score=float(candidate.score),
        translation_m=center,
        quaternion_xyzw=quaternion,
        width_m=float(candidate.width_m),
        height_m=getattr(candidate, 'height_m', None),
        depth_m=depth,
        tool0_translation_m=tool0_translation,
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
    quaternion = np.asarray(quaternion, dtype=float)
    if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)):
        raise ValueError('remote grasp candidate quaternion must contain 4 finite values')
    norm = float(np.linalg.norm(quaternion))
    if norm <= 1e-12:
        raise ValueError('remote grasp candidate quaternion has zero norm')
    quaternion = quaternion / norm
    if quaternion[3] < 0.0:
        quaternion = -quaternion
    return quaternion


def _finite_vector3(values, name):
    vector = np.asarray(values, dtype=float)
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        raise ValueError('%s must contain 3 finite values' % str(name))
    return vector.copy()


def candidate_tool0_translation(candidate, legacy_nonexecuting=False):
    """Return an explicit derived tool0 origin without reinterpreting center."""
    tool0 = getattr(candidate, 'tool0_translation_m', None)
    if tool0 is None:
        if not bool(legacy_nonexecuting):
            raise CandidateContractError(
                'TOOL0_MISSING',
                'candidate tool0 must be derived from center and GraspNet depth',
            )
        tool0 = getattr(candidate, 'translation_m', None)
    return _finite_vector3(tool0, 'candidate tool0')


def candidate_center_base_from_tool0_pose(candidate, tool0_pose):
    """Recover the base-frame GraspNet center from one transformed tool0 pose."""
    center_camera = _finite_vector3(candidate.translation_m, 'candidate center')
    tool0_camera = candidate_tool0_translation(candidate)
    camera_from_tool = quaternion_matrix(
        _normalize_quaternion(np.asarray(candidate.quaternion_xyzw, dtype=float))
    )[:3, :3]
    base_from_tool = pose_matrix(tool0_pose)[:3, :3]
    base_from_camera = base_from_tool.dot(camera_from_tool.T)
    tool_position = pose_matrix(tool0_pose)[:3, 3]
    center_base = tool_position + base_from_camera.dot(center_camera - tool0_camera)
    return _finite_vector3(center_base, 'candidate center in base')


def make_parallel_jaw_variant(candidate, correction):
    """Build only identity/Rz(180) variants while preserving one tool0."""
    correction_quaternion = _normalize_quaternion(
        np.asarray(correction, dtype=float)
    )
    correction_rotation = quaternion_matrix(correction_quaternion)[:3, :3]
    identity = np.eye(3, dtype=float)
    is_identity = np.allclose(correction_rotation, identity, atol=1e-7)
    if not is_identity and not np.allclose(
        correction_rotation, TOOL_Z_HALF_TURN_ROTATION, atol=1e-7
    ):
        raise CandidateContractError(
            'ORIENTATION_VARIANT_INVALID',
            'only identity and Rz(180 deg) are valid parallel-jaw variants',
        )
    depth = validate_graspnet_depth_m(
        getattr(candidate, 'depth_m', None),
        required=True,
    )
    center = _finite_vector3(candidate.translation_m, 'candidate center')
    tool0 = candidate_tool0_translation(candidate)
    source_quaternion = _normalize_quaternion(
        np.asarray(candidate.quaternion_xyzw, dtype=float)
    )
    quaternion = (
        source_quaternion
        if is_identity
        else _normalize_quaternion(
            np.asarray(
                quaternion_multiply(
                    source_quaternion,
                    correction_quaternion,
                ),
                dtype=float,
            )
        )
    )
    expected_offset = depth * quaternion_matrix(quaternion)[:3, 2]
    actual_offset = tool0 - center
    if not np.allclose(actual_offset, expected_offset, atol=1e-7):
        raise CandidateContractError(
            'TOOL0_INCONSISTENT',
            'orientation variant changed the physical insertion axis',
        )
    # Always allocate a fresh candidate, including for identity.  Variant
    # lineage is attached later; sharing the identity object with another
    # variant would let a later setattr overwrite the earlier audit indices.
    return RemoteGraspCandidate(
        score=float(candidate.score),
        translation_m=center,
        quaternion_xyzw=quaternion,
        width_m=float(candidate.width_m),
        height_m=getattr(candidate, 'height_m', None),
        depth_m=depth,
        tool0_translation_m=tool0,
    )


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


def _readonly_rigid_transform(value, name):
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise CandidateContractError(
            'SNAPSHOT_TRANSFORM_INVALID',
            '%s must be a finite 4x4 rigid transform' % name,
        )
    if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-9):
        raise CandidateContractError(
            'SNAPSHOT_TRANSFORM_INVALID',
            '%s has an invalid homogeneous last row' % name,
        )
    rotation = matrix[:3, :3]
    if (
        not np.allclose(rotation.T.dot(rotation), np.eye(3), atol=1e-7)
        or not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1e-7)
    ):
        raise CandidateContractError(
            'SNAPSHOT_TRANSFORM_INVALID',
            '%s rotation is not orthonormal with determinant +1' % name,
        )
    frozen = np.array(matrix, dtype=float, copy=True, order='C')
    frozen.setflags(write=False)
    return frozen


def derive_base_camera_link_transform(T_base_optical):
    """Derive base<-camera_link from one frozen base<-optical snapshot TF."""
    base_from_optical = _readonly_rigid_transform(
        T_base_optical,
        'T_base_optical',
    )
    camera_link_from_optical = np.eye(4, dtype=float)
    camera_link_from_optical[:3, :3] = OPTICAL_TO_ROS_CAMERA
    base_from_camera_link = base_from_optical.dot(
        np.linalg.inv(camera_link_from_optical)
    )
    return _readonly_rigid_transform(
        base_from_camera_link,
        'T_base_camera_link',
    )


def _stamp_to_nsec(stamp):
    if stamp is None:
        return 0
    if hasattr(stamp, 'to_nsec'):
        return int(stamp.to_nsec())
    return int(round(float(stamp) * 1.0e9))


class FrozenSnapshotCandidatePoseEstimator:
    """Transform one candidate batch without any live/latest TF lookup."""

    def __init__(
        self,
        T_base_optical,
        snapshot_stamp,
        snapshot_source_frame,
        raw_candidate_convention=PRODUCTION_CANDIDATE_FRAME_CONVENTION,
        camera_frame=CANONICAL_CANDIDATE_CAMERA_FRAME,
        base_frame='base_link',
    ):
        self._T_base_optical = _readonly_rigid_transform(
            T_base_optical,
            'T_base_optical',
        )
        self._T_base_camera_link = derive_base_camera_link_transform(
            self._T_base_optical
        )
        if snapshot_stamp is None:
            raise CandidateContractError(
                'SNAPSHOT_TRANSFORM_INVALID',
                'frozen snapshot stamp is missing',
            )
        try:
            snapshot_stamp_ns = _stamp_to_nsec(snapshot_stamp)
        except (TypeError, ValueError, OverflowError) as exc:
            raise CandidateContractError(
                'SNAPSHOT_TRANSFORM_INVALID',
                'frozen snapshot stamp is invalid: %s' % exc,
            )
        if snapshot_stamp_ns <= 0:
            raise CandidateContractError(
                'SNAPSHOT_TRANSFORM_INVALID',
                (
                    'frozen snapshot stamp must be strictly positive; '
                    'ROS Time(0) is forbidden because it means latest TF'
                ),
            )
        self.snapshot_stamp = snapshot_stamp
        self.snapshot_stamp_ns = int(snapshot_stamp_ns)
        self.snapshot_source_frame = str(snapshot_source_frame or '')
        if not self.snapshot_source_frame:
            raise CandidateContractError(
                'SNAPSHOT_TRANSFORM_INVALID',
                'snapshot source frame is empty',
            )
        self.raw_candidate_convention = normalize_candidate_frame_convention(
            raw_candidate_convention
        )
        self.camera_frame = str(camera_frame)
        self.base_frame = str(base_frame)
        if self.camera_frame != CANONICAL_CANDIDATE_CAMERA_FRAME:
            raise CandidateContractError(
                'SNAPSHOT_TRANSFORM_INVALID',
                'frozen candidate frame must be camera_link',
            )
        if self.base_frame != 'base_link':
            raise CandidateContractError(
                'SNAPSHOT_TRANSFORM_INVALID',
                'frozen candidate base frame must be base_link',
            )
        transform_payload = {
            'snapshot_stamp_ns': int(self.snapshot_stamp_ns),
            'snapshot_source_frame': self.snapshot_source_frame,
            'canonical_candidate_frame': self.camera_frame,
            'T_base_optical': self._T_base_optical.tolist(),
            'T_base_camera_link': self._T_base_camera_link.tolist(),
        }
        encoded = json.dumps(
            transform_payload,
            ensure_ascii=True,
            separators=(',', ':'),
            sort_keys=True,
        ).encode('utf-8')
        self.transform_sha256 = hashlib.sha256(encoded).hexdigest()

    @property
    def T_base_optical(self):
        return self._T_base_optical

    @property
    def T_base_camera_link(self):
        return self._T_base_camera_link

    def _validate_call(self, stamp, camera_frame):
        requested_frame = str(camera_frame or self.camera_frame)
        if requested_frame != self.camera_frame:
            raise CandidateContractError(
                'SNAPSHOT_TRANSFORM_INCONSISTENT',
                'candidate frame %s does not match frozen %s'
                % (requested_frame, self.camera_frame),
            )
        requested_stamp_ns = _stamp_to_nsec(
            self.snapshot_stamp if stamp is None else stamp
        )
        if requested_stamp_ns != self.snapshot_stamp_ns:
            raise CandidateContractError(
                'SNAPSHOT_TRANSFORM_INCONSISTENT',
                'candidate stamp does not match frozen planning snapshot',
            )

    def transform_camera_point_to_base(self, xyz):
        point = np.ones(4, dtype=float)
        point[:3] = _finite_vector3(xyz, 'candidate camera point')
        return _finite_vector3(
            self._T_base_camera_link.dot(point)[:3],
            'candidate base point',
        )

    def transform_camera_rotation_to_base(self, rotation):
        camera_rotation = np.asarray(rotation, dtype=float)
        if camera_rotation.shape != (3, 3) or not np.all(np.isfinite(camera_rotation)):
            raise CandidateContractError(
                'SNAPSHOT_TRANSFORM_INCONSISTENT',
                'candidate camera rotation must be finite 3x3',
            )
        return self._T_base_camera_link[:3, :3].dot(camera_rotation)

    def make_base_pose_from_camera_pose(
        self,
        xyz,
        quaternion_xyzw,
        stamp=None,
        camera_frame=None,
    ):
        self._validate_call(stamp, camera_frame)
        camera_from_tool = transform_matrix(
            _finite_vector3(xyz, 'candidate tool0'),
            _normalize_quaternion(np.asarray(quaternion_xyzw, dtype=float)),
        )
        base_from_tool = self._T_base_camera_link.dot(camera_from_tool)
        quaternion = _normalize_quaternion(
            np.asarray(quaternion_from_matrix(base_from_tool), dtype=float)
        )
        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.header.stamp = self.snapshot_stamp
        pose.pose.position.x = float(base_from_tool[0, 3])
        pose.pose.position.y = float(base_from_tool[1, 3])
        pose.pose.position.z = float(base_from_tool[2, 3])
        pose.pose.orientation.x = float(quaternion[0])
        pose.pose.orientation.y = float(quaternion[1])
        pose.pose.orientation.z = float(quaternion[2])
        pose.pose.orientation.w = float(quaternion[3])
        return pose

    def audit_metadata(self, include_matrices=False):
        metadata = {
            'snapshot_stamp_ns': int(self.snapshot_stamp_ns),
            'snapshot_source_frame': self.snapshot_source_frame,
            'raw_candidate_convention': self.raw_candidate_convention,
            'canonical_candidate_frame': self.camera_frame,
            'base_frame': self.base_frame,
            'transform_sha256': self.transform_sha256,
        }
        if include_matrices:
            metadata['T_base_optical'] = self._T_base_optical.tolist()
            metadata['T_base_camera_link'] = self._T_base_camera_link.tolist()
        return metadata


def make_candidate_base_pose_and_center(
    candidate,
    pose_estimator,
    stamp,
    camera_frame,
):
    """Transform candidate tool0/center with one estimator and cross-check it."""
    pose = pose_estimator.make_base_pose_from_camera_pose(
        candidate_tool0_translation(candidate),
        candidate.quaternion_xyzw,
        stamp=stamp,
        camera_frame=camera_frame,
    )
    recovered_center = candidate_center_base_from_tool0_pose(candidate, pose)
    point_transform = getattr(
        pose_estimator,
        'transform_camera_point_to_base',
        None,
    )
    if not callable(point_transform):
        return pose, recovered_center

    direct_center = _finite_vector3(
        point_transform(candidate.translation_m),
        'direct candidate center in base',
    )
    direct_tool0 = _finite_vector3(
        point_transform(candidate_tool0_translation(candidate)),
        'direct candidate tool0 in base',
    )
    actual_tool0 = pose_matrix(pose)[:3, 3]
    if (
        not np.allclose(direct_center, recovered_center, atol=1e-7)
        or not np.allclose(direct_tool0, actual_tool0, atol=1e-7)
    ):
        raise CandidateContractError(
            'SNAPSHOT_TRANSFORM_INCONSISTENT',
            'candidate center/tool0 disagree under the frozen snapshot transform',
        )
    rotation_transform = getattr(
        pose_estimator,
        'transform_camera_rotation_to_base',
        None,
    )
    if callable(rotation_transform):
        camera_rotation = quaternion_matrix(candidate.quaternion_xyzw)[:3, :3]
        direct_rotation = np.asarray(
            rotation_transform(camera_rotation),
            dtype=float,
        )
        if not np.allclose(
            direct_rotation,
            pose_matrix(pose)[:3, :3],
            atol=1e-7,
        ):
            raise CandidateContractError(
                'SNAPSHOT_TRANSFORM_INCONSISTENT',
                'candidate orientation disagrees under the frozen snapshot transform',
            )
    return pose, direct_center


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
    require_candidate_depth=True,
    candidate_rejection_fn=None,
    evaluation_record_sink=None,
):
    evaluation_records = []

    def record(evaluation):
        evaluation_records.append(evaluation)

    def emit_all():
        if callable(evaluation_record_sink):
            for evaluation in evaluation_records:
                evaluation_record_sink(evaluation)

    def failure_details(exc, default_code):
        return (
            str(getattr(exc, 'code', default_code) or default_code),
            str(exc),
        )

    def base_evaluation(candidate_index, variant_index):
        return {
            'candidate_index': int(candidate_index),
            'variant_index': int(variant_index),
            'candidate_contract': {
                'ok': True,
                'stage': 'tool0_and_parallel_jaw_variant',
                'failure_code': '',
                'failure_reason': '',
            },
            'analytical_result': candidate_gate_result_audit(None),
            'target_filter': {
                'checked': False,
                'ok': False,
                'failure_code': '',
                'failure_reason': '',
            },
            'strict_reachability': {
                'checked': False,
                'ok': False,
                'failure_code': '',
                'failure_reason': '',
            },
            'rank': None,
            'rank_valid': False,
            'selected': False,
        }

    def contract_failure_record(candidate_index, variant_index, exc, stage):
        evaluation = base_evaluation(candidate_index, variant_index)
        code, reason = failure_details(exc, 'CANDIDATE_CONTRACT_INVALID')
        evaluation['candidate_contract'] = {
            'ok': False,
            'stage': str(stage),
            'failure_code': code,
            'failure_reason': reason,
        }
        return evaluation

    def safely_record_rejection(candidate, variant_index, exc, stage):
        if callable(candidate_rejection_fn):
            try:
                candidate_rejection_fn(candidate, variant_index, exc, stage)
            except Exception:
                # Rejection counters are diagnostic-only.  They cannot be
                # allowed to suppress the authoritative per-lineage record.
                pass

    normalized_convention = normalize_candidate_frame_convention(
        candidate_frame_convention
    )
    if normalized_convention not in ('opencv_optical', 'ros_camera_link'):
        raise ValueError(
            'unknown remote grasp candidate frame convention: %s'
            % normalized_convention
        )
    variants = list(orientation_variant_quaternions or [])
    if not variants:
        variants = [np.asarray([0.0, 0.0, 0.0, 1.0], dtype=float)]
    model_to_tool = validate_execution_tool0_contract(
        require_candidate_depth,
        (
            model_grasp_to_tool_quaternion
            if model_grasp_to_tool_quaternion is not None
            else STRICT_MODEL_GRASP_TO_TOOL_QUATERNION
        ),
        grasp_config,
    )
    ranked = []
    unranked_reachable = []
    for candidate_index, candidate in enumerate(candidates):
        try:
            camera_candidate = convert_candidate_to_camera_link(
                candidate,
                normalized_convention,
            )
            camera_candidate = align_candidate_to_tool_frame(
                camera_candidate,
                model_to_tool,
                require_depth=require_candidate_depth,
            )
        except Exception as exc:
            # A malformed candidate must never fall back to treating its cloud
            # center as tool0.  Other candidates from the immutable batch may
            # still be evaluated normally.
            safely_record_rejection(candidate, None, exc, 'alignment')
            for variant_index in range(len(variants)):
                record(
                    contract_failure_record(
                        candidate_index,
                        variant_index,
                        exc,
                        'alignment',
                    )
                )
            continue
        for variant_index, correction in enumerate(variants):
            evaluation = base_evaluation(candidate_index, variant_index)
            try:
                variant_candidate = make_parallel_jaw_variant(
                    camera_candidate,
                    correction,
                )
            except Exception as exc:
                safely_record_rejection(
                    candidate,
                    variant_index,
                    exc,
                    'orientation_variant',
                )
                evaluation = contract_failure_record(
                    candidate_index,
                    variant_index,
                    exc,
                    'orientation_variant',
                )
                record(evaluation)
                continue
            setattr(variant_candidate, '_raw_candidate_index', int(candidate_index))
            setattr(variant_candidate, '_variant_index', int(variant_index))
            try:
                pose, center_base = make_candidate_base_pose_and_center(
                    variant_candidate,
                    pose_estimator,
                    stamp,
                    camera_frame,
                )
            except Exception as exc:
                safely_record_rejection(
                    candidate,
                    variant_index,
                    exc,
                    'snapshot_pose',
                )
                evaluation = contract_failure_record(
                    candidate_index,
                    variant_index,
                    exc,
                    'snapshot_pose',
                )
                record(evaluation)
                continue
            setattr(
                variant_candidate,
                '_center_base_xyz',
                center_base,
            )
            gate_result = None
            if candidate_geometry_fn is not None:
                try:
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
                            config.get('tool_approach_axis', 'z')
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
                except Exception as exc:
                    code, reason = failure_details(
                        exc,
                        'ANALYTICAL_GEOMETRY_EXCEPTION',
                    )
                    analytical = candidate_gate_result_audit(None)
                    analytical.update(
                        {
                            'checked': True,
                            'failure_code': code,
                            'failure_reason': reason,
                            'failed_gate': 'exception',
                        }
                    )
                    evaluation['analytical_result'] = analytical
                    record(evaluation)
                    continue
                setattr(variant_candidate, '_geometry_gate_result', gate_result)
                setattr(variant_candidate, '_grasp_sequence', plan)
                setattr(
                    variant_candidate,
                    'required_open_width_m',
                    float(gate_result.required_open_width_m),
                )
                evaluation['analytical_result'] = candidate_gate_result_audit(
                    gate_result
                )
                if not gate_result.ok:
                    record(evaluation)
                    continue
            filter_ok = True
            if candidate_filter_fn is not None:
                try:
                    filter_ok = bool(
                        candidate_filter_fn(candidate, variant_candidate, pose)
                    )
                except Exception as exc:
                    code, reason = failure_details(exc, 'TARGET_FILTER_EXCEPTION')
                    evaluation['target_filter'] = {
                        'checked': True,
                        'ok': False,
                        'failure_code': code,
                        'failure_reason': reason,
                    }
                    record(evaluation)
                    continue
            evaluation['target_filter'] = {
                'checked': candidate_filter_fn is not None,
                'ok': bool(filter_ok),
                'failure_code': '',
                'failure_reason': '',
            }
            if not filter_ok:
                record(evaluation)
                continue
            try:
                reachable = bool(reachability_fn(pose))
            except Exception as exc:
                code, reason = failure_details(
                    exc,
                    'STRICT_REACHABILITY_EXCEPTION',
                )
                evaluation['strict_reachability'] = {
                    'checked': True,
                    'ok': False,
                    'failure_code': code,
                    'failure_reason': reason,
                }
                record(evaluation)
                continue
            evaluation['strict_reachability'] = {
                'checked': True,
                'ok': bool(reachable),
                'failure_code': '',
                'failure_reason': '',
            }
            if reachable:
                if candidate_rank_fn is None and gate_result is None:
                    evaluation['rank'] = []
                    evaluation['rank_valid'] = True
                    unranked_reachable.append(
                        (
                            int(candidate_index),
                            int(variant_index),
                            variant_candidate,
                            pose,
                            evaluation,
                        )
                    )
                    continue
                try:
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
                except Exception as exc:
                    code, reason = failure_details(exc, 'RANK_EXCEPTION')
                    evaluation['rank_error'] = {
                        'failure_code': code,
                        'failure_reason': reason,
                    }
                    record(evaluation)
                    continue
                # The rank function may add strict joint-motion cost to the
                # immutable analytical result.  Audit the final result that
                # actually participated in selection.
                evaluation['analytical_result'] = candidate_gate_result_audit(
                    getattr(
                        variant_candidate,
                        '_geometry_gate_result',
                        gate_result,
                    )
                )
                if not rank or not all(math.isfinite(value) for value in rank):
                    evaluation['rank'] = None
                    evaluation['rank_valid'] = False
                    evaluation['rank_error'] = {
                        'failure_code': 'RANK_INVALID',
                        'failure_reason': 'rank must contain only finite values',
                    }
                    record(evaluation)
                    continue
                evaluation['rank'] = [float(value) for value in rank]
                evaluation['rank_valid'] = True
                ranked.append(
                    (
                        rank,
                        -float(getattr(variant_candidate, 'score', 0.0)),
                        int(candidate_index),
                        int(variant_index),
                        variant_candidate,
                        pose,
                        evaluation,
                    )
                )
            else:
                record(evaluation)
    selected_item = None
    if ranked:
        # Equal analytical/model ranks preserve the immutable WSL batch order,
        # then the identity/Rz(180) symmetry order.  These indices are also
        # carried into the selected audit row as end-to-end lineage evidence.
        ranked.sort(key=lambda item: (item[0], item[1], item[2], item[3]))
        selected_item = (ranked[0][4], ranked[0][5], ranked[0][6])
        for item in ranked:
            record(item[6])
    elif unranked_reachable:
        unranked_reachable.sort(key=lambda item: (item[0], item[1]))
        selected_item = (
            unranked_reachable[0][2],
            unranked_reachable[0][3],
            unranked_reachable[0][4],
        )
        for item in unranked_reachable:
            record(item[4])
    if selected_item is not None:
        selected_item[2]['selected'] = True
    evaluation_records.sort(
        key=lambda item: (
            int(item.get('candidate_index', -1)),
            int(item.get('variant_index', -1)),
        )
    )
    emit_all()
    if selected_item is not None:
        return selected_item[0], selected_item[1]
    return None, None


def make_grasp_plan_pose_array(grasp_pose, stamp, grasp_config):
    plan = make_grasp_sequence_from_grasp_pose(
        grasp_pose,
        pregrasp_distance_m=float(grasp_config.get('pregrasp_distance_m', 0.08)),
        approach_offset_m=float(grasp_config.get('final_approach_offset_m', 0.015)),
        lift_height_m=float(grasp_config.get('lift_height_m', 0.05)),
        tool_approach_axis=str(grasp_config.get('tool_approach_axis', 'z')),
    )
    msg = PoseArray()
    msg.header.frame_id = grasp_pose.header.frame_id
    msg.header.stamp = stamp if stamp is not None else rospy.Time(0)
    msg.poses = [plan.pregrasp.pose, plan.approach.pose, plan.grasp.pose, plan.lift.pose]
    return msg


def _candidate_plan_poses(selected_candidate):
    sequence = getattr(selected_candidate, '_grasp_sequence', None)
    if sequence is None:
        raise ValueError('selected candidate has no four-stage grasp sequence')
    stamped = (
        getattr(sequence, 'pregrasp', None),
        getattr(sequence, 'approach', None),
        getattr(sequence, 'grasp', None),
        getattr(sequence, 'lift', None),
    )
    if any(item is None or getattr(item, 'pose', None) is None for item in stamped):
        raise ValueError('selected candidate grasp sequence is incomplete')
    return [deepcopy(item.pose) for item in stamped]


def build_rich_plan(
    selected_candidate,
    geometry_msg,
    snapshot_header,
    model_choice,
):
    """Build one immutable-by-copy execution plan from one RGB-D snapshot."""
    score = float(getattr(selected_candidate, 'score', float('nan')))
    candidate_width = float(
        getattr(selected_candidate, 'width_m', float('nan'))
    )
    required_width = float(
        getattr(selected_candidate, 'required_open_width_m', float('nan'))
    )
    if not math.isfinite(score):
        raise ValueError('selected candidate score is non-finite')
    if not math.isfinite(candidate_width) or candidate_width < 0.0:
        raise ValueError('selected candidate width is invalid')
    if not required_open_width_is_valid(required_width):
        raise ValueError('required open width must be in (0, 0.050] m')
    model = str(model_choice or '').strip()
    if not model:
        raise ValueError('model choice must be non-empty')

    message = Grasp6DPlan()
    message.header = deepcopy(snapshot_header)
    message.valid = True
    message.poses = _candidate_plan_poses(selected_candidate)
    if len(message.poses) != 4:
        raise ValueError('rich plan must contain exactly four poses')
    message.score = score
    message.candidate_width_m = candidate_width
    message.required_open_width_m = required_width
    message.object_geometry = deepcopy(geometry_msg)
    message.model_choice = model
    message.diagnostic = ''
    validate_rich_geometry(message.object_geometry)
    validate_plan_header_binding(message)
    message.plan_id = compute_plan_id(message)
    return message


def rich_plan_to_legacy(plan):
    legacy = PoseArray()
    legacy.header = deepcopy(plan.header)
    legacy.poses = [deepcopy(pose) for pose in plan.poses]
    return legacy


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
        # Audit-file authority is a startup prerequisite.  Resolve it before
        # constructing publishers, subscribers, clients, or any other live
        # node surface so a shared planning/execution path cannot start.
        remote_cfg = rospy.get_param('/grasp_6d/remote', {})
        twin_cfg = rospy.get_param('/mujoco_digital_twin', {})
        startup_gate_audit_enabled = remote_cfg.get(
            'gate_audit_enabled',
            True,
        )
        startup_gate_audit_path = validate_mandatory_planning_audit(
            startup_gate_audit_enabled,
            remote_cfg.get(
                'gate_audit_output_path',
                '~/.ros/grasp6d_gate_audit_latest.json',
            ),
        )
        (
            startup_gate_audit_path,
            startup_mujoco_audit_path,
        ) = validate_distinct_audit_paths(
            startup_gate_audit_path,
            twin_cfg.get('audit_output_path', MUJOCO_AUDIT_DEFAULT_PATH),
        )
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
        self.rich_plan_pub = rospy.Publisher(
            rospy.get_param(
                '/grasp/grasp6d_enriched_plan_topic',
                '/grasp_6d/plan_enriched',
            ),
            Grasp6DPlan,
            queue_size=1,
            latch=True,
        )
        self.preview_plan_pub = rospy.Publisher(
            '/grasp_6d/preview_plan', PoseArray, queue_size=1
        )
        self.preview_rich_plan_pub = rospy.Publisher(
            '/grasp_6d/preview_plan_enriched',
            Grasp6DPlan,
            queue_size=1,
            latch=True,
        )
        self.pipeline_metrics_pub = rospy.Publisher(
            '/grasp_6d/pipeline_metrics', String, queue_size=10
        )
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
        self.latest_rich_plan = None
        self.latest_preview_rich_plan = None
        self._geometry_invalidation_generation = 0
        self._last_geometry_invalidation_code = ''

        cam_cfg = rospy.get_param('/camera', {})
        pcfg = rospy.get_param('/perception', {})
        hcfg = rospy.get_param('/handeye', {})
        gcfg = rospy.get_param('/grasp', {})
        gripper_cfg = rospy.get_param('/gripper', {})
        self.camera_depth_scale = float(cam_cfg.get('depth_scale', 0.001))
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
        self.robot_execution_active = False
        self.execution_plan_controller = ExecutionPlanController(
            replan_cooldown_sec=remote_cfg.get(
                'replan_cooldown_sec', 1.0
            ),
            selection_hysteresis_ratio=remote_cfg.get(
                'selection_hysteresis_ratio', 0.12
            ),
            candidate_consecutive_invalidations=remote_cfg.get(
                'candidate_consecutive_invalidations', 2
            ),
            replan_position_delta_m=remote_cfg.get(
                'replan_position_delta_m', 0.012
            ),
            replan_orientation_delta_deg=remote_cfg.get(
                'replan_orientation_delta_deg', 12.0
            ),
            replan_target_drift_m=remote_cfg.get(
                'replan_target_drift_m', 0.025
            ),
        )
        self.target_instance_association_threshold_m = max(
            0.0,
            float(
                remote_cfg.get(
                    'target_instance_association_threshold_m', 0.08
                )
            ),
        )
        self.target_absolute_sanity_distance_m = max(
            0.001,
            float(remote_cfg.get('target_absolute_sanity_distance_m', 0.15)),
        )
        self.moveit_top_n = min(
            10, max(3, int(remote_cfg.get('moveit_top_n', 5)))
        )
        default_soft_weights = SoftScoreWeights()
        soft_weight_cfg = dict(
            remote_cfg.get('soft_score_weights', {}) or {}
        )
        self.soft_score_weights = SoftScoreWeights(
            **{
                name: soft_weight_cfg.get(
                    name, getattr(default_soft_weights, name)
                )
                for name in default_soft_weights.__dataclass_fields__
            }
        )
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
                    remote_cfg.get('planning_snapshot_timeout_sec', 4.0),
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
        self.planning_snapshot_max_inference_latency_sec = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/planning_snapshot_max_inference_latency_sec',
                    remote_cfg.get(
                        'planning_snapshot_max_inference_latency_sec',
                        1.2,
                    ),
                )
            ),
        )
        self.planning_snapshot_max_span_sec = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/planning_snapshot_max_span_sec',
                    remote_cfg.get('planning_snapshot_max_span_sec', 3.0),
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
        self.gate_audit_enabled = startup_gate_audit_enabled
        self.gate_audit_output_path = startup_gate_audit_path
        self.mujoco_audit_output_path = startup_mujoco_audit_path
        self.gate_audit_enabled = True
        self._latest_gate_audit_summary = {}
        self._latest_gate_audit_reference = {}
        self._active_gate_audit_report = None
        # These values are copied into FrozenGraspNetInputConfig at the very
        # start of each request.  Keep them uncoerced here so the pure builder
        # can reject bool/non-finite/non-integral ROS values fail-closed.
        self.graspnet_input_mode = rospy.get_param(
            '/grasp_6d/remote/graspnet_input_mode',
            remote_cfg.get('graspnet_input_mode', MASKED_TARGET),
        )
        self.graspnet_input_context_margin_px = rospy.get_param(
            '/grasp_6d/remote/graspnet_input_context_margin_px',
            remote_cfg.get(
                'graspnet_input_context_margin_px',
                DEFAULT_CONTEXT_MARGIN_PX,
            ),
        )
        self.graspnet_input_context_expand_ratio = rospy.get_param(
            '/grasp_6d/remote/graspnet_input_context_expand_ratio',
            remote_cfg.get(
                'graspnet_input_context_expand_ratio',
                DEFAULT_CONTEXT_EXPAND_RATIO,
            ),
        )
        self.graspnet_input_context_max_margin_px = rospy.get_param(
            '/grasp_6d/remote/graspnet_input_context_max_margin_px',
            remote_cfg.get(
                'graspnet_input_context_max_margin_px',
                DEFAULT_CONTEXT_MAX_MARGIN_PX,
            ),
        )
        self.graspnet_input_target_guard_px = rospy.get_param(
            '/grasp_6d/remote/graspnet_input_target_guard_px',
            remote_cfg.get(
                'graspnet_input_target_guard_px',
                DEFAULT_TARGET_GUARD_PX,
            ),
        )
        self.graspnet_input_support_band_m = rospy.get_param(
            '/grasp_6d/remote/graspnet_input_support_band_m',
            remote_cfg.get(
                'graspnet_input_support_band_m',
                DEFAULT_CONTEXT_PLANE_DISTANCE_M,
            ),
        )
        self.graspnet_input_min_target_points = rospy.get_param(
            '/grasp_6d/remote/graspnet_input_min_target_points',
            remote_cfg.get(
                'graspnet_input_min_target_points',
                DEFAULT_MIN_TARGET_POINTS,
            ),
        )
        self.graspnet_input_min_support_points = rospy.get_param(
            '/grasp_6d/remote/graspnet_input_min_support_points',
            remote_cfg.get(
                'graspnet_input_min_support_points',
                DEFAULT_MIN_SUPPORT_POINTS,
            ),
        )
        self.graspnet_input_min_total_points = rospy.get_param(
            '/grasp_6d/remote/graspnet_input_min_total_points',
            remote_cfg.get(
                'graspnet_input_min_total_points',
                DEFAULT_MIN_TOTAL_POINTS,
            ),
        )
        self.graspnet_input_min_target_fraction = rospy.get_param(
            '/grasp_6d/remote/graspnet_input_min_target_fraction',
            remote_cfg.get(
                'graspnet_input_min_target_fraction',
                DEFAULT_MIN_TARGET_FRACTION,
            ),
        )
        self.graspnet_input_bbox_min_iou = rospy.get_param(
            '/grasp_6d/remote/graspnet_input_bbox_min_iou',
            remote_cfg.get(
                'graspnet_input_bbox_min_iou',
                DEFAULT_DETECTED_BBOX_MIN_IOU,
            ),
        )
        self._active_graspnet_input_audit = {}
        self.grasp_config = dict(gcfg or {})
        self.handeye_parent_frame = str(hcfg.get('parent_frame', 'tool0'))
        self.handeye_camera_frame = str(hcfg.get('camera_frame', cam_cfg.get('frame_id', 'camera_link')))
        self.handeye_translation_xyz = list(hcfg.get('translation_xyz', [0.0, 0.0, 0.0]))
        self.handeye_rotation_xyzw = list(hcfg.get('rotation_xyzw', [0.0, 0.0, 0.0, 1.0]))
        self.handeye_allow_static_fallback = bool(
            hcfg.get('allow_static_fallback', True)
        )
        self._cached_tool_from_camera = None
        self._cached_tool_from_camera_stamp_ns = 0
        server_url = rospy.get_param('/grasp_6d/remote/server_url', remote_cfg.get('server_url', 'http://172.23.132.97:8000'))
        timeout_sec = float(rospy.get_param('/grasp_6d/remote/timeout_sec', remote_cfg.get('timeout_sec', 3.0)))
        self.max_candidates = int(rospy.get_param('/grasp_6d/remote/max_candidates', remote_cfg.get('max_candidates', 20)))
        self.auto_request = bool(rospy.get_param('/grasp_6d/remote/auto_request', remote_cfg.get('auto_request', False)))
        self.failure_backoff_sec = max(0.0, float(rospy.get_param('/grasp_6d/remote/failure_backoff_sec', remote_cfg.get('failure_backoff_sec', 8.0))))
        self.candidate_frame_convention = validate_production_candidate_frame_convention(
            rospy.get_param('/grasp_6d/remote/candidate_frame_convention', remote_cfg.get('candidate_frame_convention', 'opencv_optical'))
        )
        self.orientation_variant_quaternions = self._parse_production_orientation_variant_quaternions(
            rospy.get_param(
                '/grasp_6d/remote/orientation_variants_rpy_deg',
                remote_cfg.get(
                    'orientation_variants_rpy_deg',
                    STRICT_ORIENTATION_VARIANTS_RPY_DEG,
                ),
            )
        )
        self.model_grasp_to_tool_quaternion = self._parse_orientation_variant_quaternions(
            [rospy.get_param(
                '/grasp_6d/remote/model_grasp_to_tool_rpy_deg',
                remote_cfg.get('model_grasp_to_tool_rpy_deg', [0.0, 90.0, 0.0]),
            )]
        )[0]
        self.require_candidate_depth = bool(
            rospy.get_param(
                '/grasp_6d/remote/require_candidate_depth',
                remote_cfg.get('require_candidate_depth', True),
            )
        )
        self.model_grasp_to_tool_quaternion = validate_execution_tool0_contract(
            self.require_candidate_depth,
            self.model_grasp_to_tool_quaternion,
            self.grasp_config,
        )
        accept_position_only_fallback = bool(
            rospy.get_param(
                '/grasp_6d/remote/accept_position_only_fallback',
                remote_cfg.get('accept_position_only_fallback', False),
            )
        )
        accept_orientation_fallback = bool(
            rospy.get_param(
                '/grasp_6d/remote/accept_orientation_fallback',
                remote_cfg.get('accept_orientation_fallback', False),
            )
        )
        (
            self.allow_position_only_fallback,
            self.allow_orientation_fallback,
        ) = validate_production_execution_fallback_contract(
            accept_position_only_fallback,
            accept_orientation_fallback,
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
            self.client = RemoteGrasp6DClient(
                server_url,
                timeout_sec=timeout_sec,
                require_candidate_depth=True,
            )
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
        rospy.Subscriber('/grasp/state', GraspState, self.grasp_state_cb, queue_size=1)
        self.rate_hz = max(
            0.1,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/request_hz',
                    remote_cfg.get('request_hz', 1.5),
                )
            ),
        )
        tracking_config = TrackingConfig(
            window_size=int(remote_cfg.get('stability_window_size', 5)),
            min_hits=int(remote_cfg.get('stability_min_hits', 3)),
            position_threshold_m=float(
                remote_cfg.get('tracking_position_threshold_m', 0.025)
            ),
            orientation_threshold_deg=float(
                remote_cfg.get('tracking_orientation_threshold_deg', 25.0)
            ),
            approach_threshold_deg=float(
                remote_cfg.get('tracking_approach_threshold_deg', 20.0)
            ),
            width_threshold_m=float(
                remote_cfg.get('tracking_width_threshold_m', 0.008)
            ),
        )
        self._initialize_streaming_state(
            result_max_age_sec=float(
                remote_cfg.get('result_max_age_sec', 1.2)
            ),
            performance_window_size=int(
                remote_cfg.get('performance_window_size', 100)
            ),
            tracking_config=tracking_config,
        )
        rospy.on_shutdown(self.shutdown_streaming_worker)
        rospy.Service('/grasp_6d/request_plan', TriggerZero, self.request_plan_cb)
        rospy.Service(
            '/grasp_6d/replan_execution',
            TriggerZero,
            self.replan_execution_cb,
        )
        self._publish_invalid_plan_pair(
            'INITIALIZING',
            'no rich 6D plan has been generated',
        )
        self._check_remote_health()
        if self.enabled and self.auto_request:
            self.start_streaming()
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

    def grasp_state_cb(self, msg):
        """Track execution activity only; it does not control inference."""

        with self._geometry_state_guard():
            self.robot_execution_active = bool(getattr(msg, 'active', False))

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

    def _latest_rich_plan_copy(self):
        with self._geometry_state_guard():
            cached = getattr(self, 'latest_rich_plan', None)
            return deepcopy(cached) if cached is not None else None

    def _publish_invalid_plan_pair(
        self,
        failure_code,
        failure_reason,
        stamp=None,
        header=None,
        invalid_geometry=None,
    ):
        with self._geometry_state_guard():
            current_header = deepcopy(header) if header is not None else None
            if current_header is None:
                current_header = PoseArray().header
                current_header.frame_id = 'base_link'
                current_header.stamp = self._safe_time(stamp)
            invalid = Grasp6DPlan()
            invalid.header = deepcopy(current_header)
            invalid.valid = False
            invalid.diagnostic = '%s: %s' % (
                str(failure_code or 'PLAN_INVALID'),
                str(failure_reason or 'plan is invalid'),
            )
            legacy = PoseArray()
            legacy.header = deepcopy(current_header)
            legacy.poses = []

            # Invalidation authority is committed before any ROS publisher is
            # called. A broken transport must never leave an executable cache.
            self.latest_rich_plan = None
            self.latest_plan = None
            controller = getattr(self, 'execution_plan_controller', None)
            if controller is not None:
                controller.clear_execution()
            self.latest_object_geometry = (
                deepcopy(invalid_geometry)
                if invalid_geometry is not None
                else None
            )
            self.previous_object_axes_base = None
            self._clear_geometry_cache()

            rich_publisher = getattr(self, 'rich_plan_pub', None)
            legacy_publisher = getattr(self, 'plan_pub', None)
            self._publish_invalidation_safely(
                rich_publisher,
                deepcopy(invalid),
                'rich plan',
            )
            self._publish_invalidation_safely(
                legacy_publisher,
                deepcopy(legacy),
                'legacy plan',
            )
            return invalid

    @staticmethod
    def _publish_invalidation_safely(publisher, message, channel_name):
        if publisher is None:
            return False
        try:
            publisher.publish(message)
            return True
        except Exception as exc:
            try:
                rospy.logerr(
                    'Failed to publish %s invalidation: %s',
                    channel_name,
                    exc,
                )
            except Exception:
                pass
            return False

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

    def _publish_plan_pair_if_current(self, rich_plan, expected_generation):
        with self._geometry_state_guard():
            current = int(getattr(self, '_geometry_invalidation_generation', 0))
            if current != int(expected_generation):
                code = str(
                    getattr(self, '_last_geometry_invalidation_code', 'PLAN_STALE')
                    or 'PLAN_STALE'
                )
                return False, code
            if not bool(getattr(rich_plan, 'valid', False)):
                return False, 'PLAN_INVALID'
            # Construct both outgoing messages before publishing either one.
            outgoing_rich = deepcopy(rich_plan)
            outgoing_legacy = rich_plan_to_legacy(outgoing_rich)
            cached = deepcopy(outgoing_rich)
            rich_publisher = getattr(self, 'rich_plan_pub', None)
            if rich_publisher is None:
                return False, 'RICH_PLAN_PUBLISHER_UNAVAILABLE'
            # The legacy PoseArray is visualization-only.  Publish it first,
            # then commit the latched rich plan as the sole execution
            # authority.  A legacy publisher failure can therefore never
            # leave a valid rich command visible to an executor.
            self.plan_pub.publish(deepcopy(outgoing_legacy))
            rich_publisher.publish(deepcopy(outgoing_rich))
            self.latest_rich_plan = cached
            self.latest_plan = deepcopy(outgoing_legacy)
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
            self._publish_invalid_plan_pair(
                failure_code,
                failure_reason,
                header=message.header,
                invalid_geometry=message,
            )
            self._publish_invalidation_safely(
                getattr(self, 'geometry_pub', None),
                deepcopy(message),
                'object geometry',
            )
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
                previous = self.latest_object
            with self._object_lock:
                self.latest_object = msg
                self.latest_object_time = now
            if bool(getattr(previous, 'detected', False)):
                self._advance_target_instance_epoch('TARGET_LOST')
            if hasattr(self, 'frames'):
                self.frames.update_object(
                    msg,
                    source_stamp,
                    target_epoch=int(
                        getattr(self, 'target_instance_epoch', 0)
                    ),
                    target_identity=self._current_stream_target_identity(),
                )
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

        identity_changed = bool(
            previous is not None
            and bool(getattr(previous, 'detected', False))
            and (
                str(getattr(previous, 'label', '') or '')
                != str(getattr(msg, 'label', '') or '')
                or self._object_pose_distance(previous, msg)
                > float(
                    getattr(
                        self,
                        'target_instance_association_threshold_m',
                        0.08,
                    )
                )
            )
        )
        with self._object_lock:
            self.latest_object = msg
            self.latest_object_time = now
        if identity_changed:
            self._advance_target_instance_epoch('TARGET_INSTANCE_CHANGED')
        if hasattr(self, 'frames'):
            self.frames.update_object(
                msg,
                source_stamp,
                target_epoch=int(getattr(self, 'target_instance_epoch', 0)),
                target_identity=self._current_stream_target_identity(),
            )

    def _current_stream_target_identity(self):
        target = getattr(self, 'latest_object', None)
        return (
            int(getattr(self, 'target_instance_epoch', 0)),
            str(getattr(target, 'label', '') or ''),
            str(getattr(self, '_last_model_choice', '') or ''),
        )

    def _stream_ticket_cancellation_code_locked(self, ticket):
        if (
            self._stream_shutdown.is_set()
            or not self.streaming_enabled
            or int(ticket.generation) != int(self._stream_generation)
        ):
            return 'GENERATION_STALE'
        if int(ticket.target_epoch) != int(self.target_instance_epoch):
            return 'TARGET_EPOCH_STALE'
        return ''

    def _require_stream_ticket_current_locked(self, ticket):
        code = self._stream_ticket_cancellation_code_locked(ticket)
        if code:
            raise StreamResultCancelled(code)

    def _require_stream_ticket_current(self, ticket):
        with self._stream_condition:
            self._require_stream_ticket_current_locked(ticket)

    def _advance_target_instance_epoch(self, reason):
        """Invalidate pending tracking when the observed target identity changes."""

        condition = getattr(self, '_stream_condition', None)
        if condition is None:
            return False
        pending_request_id = None
        pending_terminal_claimed = False
        with condition:
            pending_request_id = self._pending_request_id
            if pending_request_id is not None:
                pending_terminal_claimed = self._claim_terminal_request_locked(
                    pending_request_id,
                    'TARGET_EPOCH_STALE',
                )
                self._pending_request_id = None
            self.target_instance_epoch += 1
            self.tracker = CandidateTracker(self._tracking_config)
            self._stable_variant_runtime = {}
            self.inference_coordinator.reset_target_epoch(
                self.target_instance_epoch
            )
            self._last_target_epoch_reason = str(reason or '')
            condition.notify_all()
        if pending_request_id is not None:
            self._emit_pending_drop_metrics(
                pending_request_id,
                'TARGET_EPOCH_STALE',
                terminal_claimed=pending_terminal_claimed,
            )
        return True

    def _initialize_streaming_state(
        self,
        result_max_age_sec=1.2,
        performance_window_size=100,
        source_clock=None,
        tracking_config=None,
        start_worker=True,
    ):
        """Initialize the single-worker, latest-only streaming state."""

        window_size = max(1, int(performance_window_size))
        self._stream_source_clock = source_clock or (
            lambda: float(self._ros_source_clock_ns()) / 1e9
        )
        self.inference_coordinator = LatestOnlyInferenceCoordinator(
            result_max_age_sec=result_max_age_sec,
            clock=self._stream_source_clock,
        )
        self._tracking_config = (
            TrackingConfig()
            if tracking_config is None
            else tracking_config
        )
        self.tracker = CandidateTracker(self._tracking_config)
        self._stable_variant_runtime = {}
        self._stream_condition = threading.Condition(threading.RLock())
        self._stream_shutdown = threading.Event()
        self._stream_worker_ticket = None
        self._stream_worker = None
        self._stream_worker_busy = False
        self.streaming_enabled = False
        self.last_submitted_stamp_ns = 0
        self.target_instance_epoch = int(
            getattr(self, 'target_instance_epoch', 0)
        )
        self._pipeline_counters = Counter(
            {name: 0 for name in PIPELINE_COUNTER_FIELDS}
        )
        self._pending_replacements = 0
        self._pending_request_id = None
        self._request_telemetry = {}
        self._terminal_request_ids = set()
        self._terminal_request_order = deque()
        self._terminal_request_limit = max(8, window_size * 2)
        self._stream_generation = 0
        self._latency_history_ms = deque(maxlen=window_size)
        self.pipeline_metrics = deque(maxlen=window_size)
        if start_worker:
            self._stream_worker = threading.Thread(
                target=self._stream_worker_main,
                name='remote-grasp6d-latest-only',
                daemon=True,
            )
            self._stream_worker.start()

    def start_streaming(self):
        with self._stream_condition:
            if self.streaming_enabled:
                return False
            self.target_instance_epoch += 1
            self.tracker = CandidateTracker(self._tracking_config)
            self._stable_variant_runtime = {}
            self._stream_generation = self.inference_coordinator.start()
            self.streaming_enabled = True
            self._stream_condition.notify_all()
            return True

    def stop_streaming(self):
        pending_request_id = None
        pending_terminal_claimed = False
        with self._stream_condition:
            if not self.streaming_enabled:
                return False
            pending_request_id = self._pending_request_id
            if pending_request_id is not None:
                pending_terminal_claimed = self._claim_terminal_request_locked(
                    pending_request_id,
                    'GENERATION_STALE',
                )
                self._pending_request_id = None
            self.streaming_enabled = False
            self.target_instance_epoch += 1
            self.inference_coordinator.stop()
            self.tracker = CandidateTracker(self._tracking_config)
            self._stable_variant_runtime = {}
            self._stream_condition.notify_all()
        if pending_request_id is not None:
            self._emit_pending_drop_metrics(
                pending_request_id,
                'GENERATION_STALE',
                terminal_claimed=pending_terminal_claimed,
            )
        return True

    def submit_stream_snapshot(self, snapshot, graspnet_input_config=None):
        stamp_sec = _finite_pipeline_number(
            getattr(snapshot, 'stamp_sec', float('nan')),
            default=float('nan'),
        )
        stamp_ns = int(getattr(snapshot, 'stamp_ns', 0) or 0)
        if not math.isfinite(stamp_sec) or stamp_sec <= 0.0 or stamp_ns <= 0:
            raise ValueError('stream snapshot must have a positive finite stamp')
        replaced_request_id = None
        with self._stream_condition:
            if not self.streaming_enabled:
                return False
            snapshot_identity = tuple(
                getattr(snapshot, 'target_identity', ()) or ()
            )
            snapshot_epoch = getattr(snapshot, 'target_epoch', None)
            current_identity = self._current_stream_target_identity()
            if (
                snapshot_identity
                and snapshot_identity != current_identity
            ) or (
                snapshot_epoch is not None
                and int(snapshot_epoch) != int(self.target_instance_epoch)
            ) or (
                isinstance(snapshot, SnapshotResult)
                and snapshot_identity != current_identity
            ):
                return False
            if stamp_ns <= self.last_submitted_stamp_ns:
                return False
            payload = (snapshot, graspnet_input_config)
            decision = self.inference_coordinator.submit(
                payload,
                stamp_sec,
                target_epoch=self.target_instance_epoch,
            )
            self.last_submitted_stamp_ns = stamp_ns
            self._pipeline_counters['submitted'] += 1
            if decision.ticket_to_start is None:
                self._pipeline_counters['busy'] += 1
            if decision.replaced_request_id is not None:
                self._pipeline_counters['replaced'] += 1
                self._pending_replacements += 1
                replaced_request_id = decision.replaced_request_id
                replaced_terminal_claimed = (
                    self._claim_terminal_request_locked(
                        replaced_request_id,
                        'PENDING_REPLACED',
                    )
                )
            submitted_request_id = (
                decision.ticket_to_start.request_id
                if decision.ticket_to_start is not None
                else decision.pending_request_id
            )
            self._request_telemetry[int(submitted_request_id)] = {
                'request_id': int(submitted_request_id),
                'generation': int(self._stream_generation),
                'target_epoch': int(self.target_instance_epoch),
                'snapshot_stamp_sec': float(stamp_sec),
                'submitted_sec': float(self._stream_source_clock()),
            }
            if decision.ticket_to_start is not None:
                self._stream_worker_ticket = decision.ticket_to_start
                self._stream_condition.notify_all()
            else:
                self._pending_request_id = int(decision.pending_request_id)
        if replaced_request_id is not None:
            self._emit_pending_drop_metrics(
                replaced_request_id,
                'PENDING_REPLACED',
                terminal_claimed=replaced_terminal_claimed,
            )
        return True

    def _claim_terminal_request_locked(
        self,
        request_id,
        status,
        accepted=False,
    ):
        request_id = int(request_id)
        if request_id in self._terminal_request_ids:
            return False
        # The coordinator exposes at most one active and one pending request;
        # once a terminal event is synchronously emitted, no lifecycle owner
        # retains that request.  Keeping a small recent window is therefore
        # sufficient for the only possible stop/worker hand-off races.
        while (
            len(self._terminal_request_order)
            >= self._terminal_request_limit
        ):
            expired_request_id = self._terminal_request_order.popleft()
            self._terminal_request_ids.discard(expired_request_id)
        self._terminal_request_ids.add(request_id)
        self._terminal_request_order.append(request_id)
        self._pipeline_counters['completed'] += 1
        if accepted:
            self._pipeline_counters['accepted'] += 1
        elif str(status) in ('PREDICT_FAILED', 'ACCEPT_FAILED'):
            self._pipeline_counters['failed'] += 1
        elif str(status) == 'RESULT_EXPIRED':
            self._pipeline_counters['expired'] += 1
        else:
            self._pipeline_counters['stale'] += 1
        return True

    def _emit_pending_drop_metrics(
        self,
        request_id,
        code,
        terminal_claimed=False,
    ):
        request_id = int(request_id)
        if not terminal_claimed:
            with self._stream_condition:
                if not self._claim_terminal_request_locked(
                    request_id,
                    code,
                ):
                    return None
        metadata = self._request_telemetry.pop(int(request_id), {})
        now_sec = float(self._stream_source_clock())
        end_to_end_ms = max(
            0.0,
            (now_sec - float(metadata.get('submitted_sec', now_sec)))
            * 1000.0,
        )
        self._latency_history_ms.append(end_to_end_ms)
        result_age_ms = max(
            0.0,
            (
                now_sec
                - float(metadata.get('snapshot_stamp_sec', now_sec))
            )
            * 1000.0,
        )
        metrics = build_pipeline_metrics(
            event='request_dropped',
            request_id=metadata.get('request_id', request_id),
            generation=metadata.get('generation', self._stream_generation),
            target_epoch=metadata.get(
                'target_epoch', self.target_instance_epoch
            ),
            snapshot_stamp_sec=metadata.get('snapshot_stamp_sec', 0.0),
            status=code,
            drop_reason=code,
            counters=self._pipeline_counters,
            pending_replacements=self._pending_replacements,
            ros_prepare_ms=0.0,
            encode_ms=0.0,
            transport_ms=0.0,
            decode_ms=0.0,
            remote_performance={},
            end_to_end_ms=end_to_end_ms,
            result_age_ms=result_age_ms,
            latency_history_ms=self._latency_history_ms,
            funnel={},
        )
        self.pipeline_metrics.append(metrics)
        encoded = bounded_metrics_json(metrics)
        publisher = getattr(self, 'pipeline_metrics_pub', None)
        if publisher is not None:
            try:
                publisher.publish(String(encoded))
            except Exception:
                pass
        try:
            rospy.loginfo('remote 6D pipeline metrics: %s', encoded)
        except Exception:
            pass
        return metrics

    def _predict_remote(self, ticket, graspnet_input, intrinsics, frame_id):
        predict_bundle = getattr(self.client, 'predict_bundle', None)
        if callable(predict_bundle):
            bundle = predict_bundle(
                graspnet_input.color_bgr,
                graspnet_input.depth_raw,
                intrinsics,
                request_id=int(ticket.request_id),
                snapshot_stamp_sec=float(ticket.snapshot_stamp_sec),
                frame_id=str(frame_id or 'camera_link'),
                stamp_sec=float(ticket.snapshot_stamp_sec),
                max_candidates=int(self.max_candidates),
                max_gripper_width_m=0.0,
                candidate_width_tolerance_m=float(
                    self.candidate_width_tolerance_m
                ),
            )
            return (
                tuple(bundle.candidates),
                dict(bundle.diagnostics),
                dict(bundle.performance),
                max(0.0, _finite_pipeline_number(bundle.encode_ms)),
                max(0.0, _finite_pipeline_number(bundle.transport_ms)),
                max(0.0, _finite_pipeline_number(bundle.decode_ms)),
            )

        # Compatibility for injected legacy clients.  Production uses the
        # request-local bundle above and never reads shared ``last_*`` state.
        started = time.perf_counter()
        candidates = self.client.predict(
            graspnet_input.color_bgr,
            graspnet_input.depth_raw,
            intrinsics,
            request_id=int(ticket.request_id),
            snapshot_stamp_sec=float(ticket.snapshot_stamp_sec),
            frame_id=str(frame_id or 'camera_link'),
            stamp_sec=float(ticket.snapshot_stamp_sec),
            max_candidates=int(self.max_candidates),
            max_gripper_width_m=0.0,
            candidate_width_tolerance_m=float(
                self.candidate_width_tolerance_m
            ),
        )
        elapsed_ms = max(0.0, (time.perf_counter() - started) * 1000.0)
        diagnostics = dict(
            getattr(self.client, 'last_diagnostics', {}) or {}
        )
        performance = dict(
            getattr(self.client, 'last_performance', {}) or {}
        )
        server_ms = _finite_pipeline_number(
            performance.get('server_total_ms', 0.0)
        )
        transport_ms = max(0.0, elapsed_ms - server_ms)
        return (
            tuple(candidates),
            diagnostics,
            performance,
            0.0,
            transport_ms,
            0.0,
        )

    def _prepare_and_predict(self, ticket):
        """Prepare immutable request facts and perform one correlated WSL call."""

        prepare_started = time.perf_counter()
        try:
            snapshot, input_config = ticket.payload
        except (TypeError, ValueError):
            raise ValueError('inference ticket payload must contain snapshot/config')
        if not isinstance(snapshot, SnapshotResult) or not snapshot.ok:
            raise ValueError('inference ticket requires a valid planning snapshot')
        if not bool(getattr(snapshot.object_msg, 'detected', False)):
            raise CandidateContractError(
                'TARGET_LOST',
                'locked planning snapshot target is not detected',
            )
        if input_config is None:
            input_config = self._freeze_graspnet_input_config()
        request_invalidation_generation = self._capture_geometry_generation()
        self._require_graspnet_input_prerequisites(snapshot, input_config)
        self._refresh_runtime_params()
        self.candidate_target_gate_enabled = bool(
            input_config.candidate_target_gate_enabled
        )
        stamp = self._snapshot_ros_stamp(snapshot)
        estimate, depth_for_remote, transform = self._prepare_snapshot_geometry(
            snapshot,
            stamp,
        )
        if not estimate.ok:
            raise CandidateContractError(
                estimate.failure_code,
                estimate.failure_reason,
            )
        pose_estimator = FrozenSnapshotCandidatePoseEstimator(
            transform,
            stamp,
            snapshot.frame_id,
            raw_candidate_convention=self.candidate_frame_convention,
        )
        contract_mismatch = self._gripper_contract_mismatch_reason()
        if contract_mismatch:
            raise CandidateContractError(
                'GRIPPER_MODEL_MISMATCH', contract_mismatch
            )
        graspnet_input, graspnet_input_audit = self._build_frozen_graspnet_input(
            snapshot,
            depth_for_remote,
            input_config,
            commit_audit=False,
        )
        model_choice = str(getattr(self, '_last_model_choice', '') or '')
        if not model_choice:
            raise CandidateContractError(
                'MODEL_TASK_MISMATCH', 'model choice is empty'
            )
        ros_prepare_ms = max(
            0.0,
            (time.perf_counter() - prepare_started) * 1000.0,
        )
        self._require_stream_ticket_current(ticket)
        (
            candidates,
            diagnostics,
            performance,
            encode_ms,
            transport_ms,
            decode_ms,
        ) = self._predict_remote(
            ticket,
            graspnet_input,
            self._camera_intrinsics(),
            snapshot.frame_id or self.pose_estimator.camera_frame,
        )
        return PreparedPrediction(
            ticket=ticket,
            snapshot=snapshot,
            stamp=stamp,
            geometry=estimate,
            pose_estimator=pose_estimator,
            graspnet_input=graspnet_input,
            candidates=tuple(candidates),
            remote_diagnostics=dict(diagnostics),
            remote_performance=dict(performance),
            graspnet_input_audit=dict(graspnet_input_audit),
            request_invalidation_generation=request_invalidation_generation,
            graspnet_input_config=input_config,
            model_choice=model_choice,
            ros_prepare_ms=ros_prepare_ms,
            encode_ms=encode_ms,
            transport_ms=transport_ms,
            decode_ms=decode_ms,
        )

    def _update_candidate_tracker(
        self,
        request_id,
        observations,
        target_identity,
        ticket=None,
    ):
        if ticket is None:
            return self.tracker.update(
                int(request_id),
                tuple(observations),
                target_identity=tuple(target_identity),
            )
        with self._stream_condition:
            self._require_stream_ticket_current_locked(ticket)
            return self.tracker.update(
                int(request_id),
                tuple(observations),
                target_identity=tuple(target_identity),
            )

    @staticmethod
    def _candidate_safety_input(
        *,
        request_id,
        snapshot_stamp_sec,
        target_identity,
        track_id,
        variant_index,
        center_base_xyz,
        tool0_position_xyz,
        quaternion_xyzw,
        approach_base_xyz,
        target_present,
        same_target_instance,
        target_absolute_distance_m,
        target_absolute_limit_m,
        required_open_width_m,
        physical_open_width_m,
        depth_valid,
        transform_valid,
        geometry_valid,
        collision_free,
        snapshot_context_revision,
    ):
        target_epoch, target_label, model_choice = tuple(target_identity)
        return SafetyGateInput(
            depth_valid=depth_valid,
            transform_valid=transform_valid,
            target_present=target_present,
            same_target_instance=same_target_instance,
            target_absolute_distance_m=target_absolute_distance_m,
            target_absolute_limit_m=target_absolute_limit_m,
            required_open_width_m=required_open_width_m,
            physical_open_width_m=physical_open_width_m,
            geometry_valid=geometry_valid,
            collision_free=collision_free,
            request_id=int(request_id),
            snapshot_stamp_sec=float(snapshot_stamp_sec),
            target_epoch=int(target_epoch),
            target_label=str(target_label),
            model_choice=str(model_choice),
            track_id=int(track_id),
            variant_index=int(variant_index),
            center_base_xyz=center_base_xyz,
            tool0_position_xyz=tool0_position_xyz,
            quaternion_xyzw=quaternion_xyzw,
            approach_base_xyz=approach_base_xyz,
            snapshot_context_revision=snapshot_context_revision,
        )

    @classmethod
    def _candidate_safety_gate(cls, **kwargs):
        return mandatory_safety_gate(cls._candidate_safety_input(**kwargs))

    @staticmethod
    def _candidate_soft_features(
        *,
        model_score,
        cloud_distance_m,
        center_distance_m,
        downward_approach_cos,
        visibility_center_cost,
        support_margin_m,
        jaw_tilt_cos,
        geometry_margin_m,
        stability_hit_ratio,
        joint_path_cost=0.0,
        joint_max_delta_rad=0.0,
        position_dispersion_m=0.0,
        orientation_dispersion_rad=0.0,
    ):
        return SoftCandidateFeatures(
            model_score=model_score,
            cloud_distance_m=cloud_distance_m,
            center_distance_m=center_distance_m,
            downward_approach_cos=downward_approach_cos,
            visibility_center_cost=visibility_center_cost,
            support_margin_m=support_margin_m,
            jaw_tilt_cos=jaw_tilt_cos,
            geometry_margin_m=geometry_margin_m,
            joint_path_cost=joint_path_cost,
            joint_max_delta_rad=joint_max_delta_rad,
            stability_hit_ratio=stability_hit_ratio,
            position_dispersion_m=position_dispersion_m,
            orientation_dispersion_rad=orientation_dispersion_rad,
        )

    def _activate_prepared_geometry(self, prepared):
        with self._stream_condition:
            self._require_stream_ticket_current_locked(prepared.ticket)
            invalidated, _code = self._geometry_invalidation_state(
                prepared.request_invalidation_generation
            )
            self._require_stream_ticket_current_locked(prepared.ticket)
            if invalidated:
                return False
            self._planning_snapshot_active = True
            self._planning_object_msg = prepared.snapshot.object_msg
            self._planning_object_time = prepared.stamp
            self._target_cloud_request_active = False
            self._clear_geometry_cache()
            activated = self._activate_geometry(
                prepared.geometry,
                prepared.snapshot,
                prepared.stamp,
                prepared.pose_estimator.T_base_optical,
                expected_generation=prepared.request_invalidation_generation,
            )
            self._require_stream_ticket_current_locked(prepared.ticket)
            if not activated:
                return False
            self._current_prepared_prediction = prepared
            self._active_graspnet_input_audit = dict(
                getattr(prepared, 'graspnet_input_audit', {}) or {}
            )
            if bool(
                getattr(self, 'camera_visibility_gate_enabled', False)
                or getattr(
                    self,
                    'camera_visibility_diagnostic_enabled',
                    False,
                )
            ):
                self._freeze_tool_from_camera_matrix(prepared.stamp)
            return True

    def _pose_approach_base_xyz(self, grasp_pose):
        matrix = pose_matrix(grasp_pose)
        name = str(
            self.grasp_config.get('tool_approach_axis', 'z') or 'z'
        ).strip().lower()
        sign = -1.0 if name.startswith('-') else 1.0
        axis_index = {'x': 0, 'y': 1, 'z': 2}[name.lstrip('+-')]
        approach = sign * np.asarray(matrix[:3, axis_index], dtype=float)
        norm = float(np.linalg.norm(approach))
        if not math.isfinite(norm) or norm <= 1e-12:
            raise CandidateContractError(
                'TRANSFORM_INVALID', 'candidate approach axis is invalid'
            )
        return approach / norm

    @staticmethod
    def _funnel_stage(entered, passed):
        entered = _pipeline_count(entered)
        passed = min(entered, _pipeline_count(passed))
        return {
            'entered': entered,
            'passed': passed,
            'rejected': entered - passed,
        }

    def _evaluate_local_candidates(self, prepared):
        """Apply current hard facts and compute soft costs before tracking."""

        candidates = tuple(prepared.candidates)
        diagnostics = dict(prepared.remote_diagnostics or {})
        rejections = Counter()
        observations = []
        weights = getattr(self, 'soft_score_weights', SoftScoreWeights())
        self._reset_geometry_gate_audit(len(candidates))
        model_to_tool = validate_execution_tool0_contract(
            self.require_candidate_depth,
            self.model_grasp_to_tool_quaternion,
            self.grasp_config,
        )
        target_label = str(
            getattr(prepared.snapshot.object_msg, 'label', '') or ''
        )
        target_identity = (
            int(prepared.ticket.target_epoch),
            target_label,
            str(prepared.model_choice),
        )
        target_xyz = np.asarray(
            prepared.geometry.center_base,
            dtype=float,
        ).reshape(3)
        context_revision = str(prepared.pose_estimator.transform_sha256)
        for candidate_index, raw_candidate in enumerate(candidates):
            try:
                camera_candidate = convert_candidate_to_camera_link(
                    raw_candidate,
                    self.candidate_frame_convention,
                )
                camera_candidate = align_candidate_to_tool_frame(
                    camera_candidate,
                    model_to_tool,
                    require_depth=self.require_candidate_depth,
                )
                setattr(camera_candidate, '_raw_candidate_index', candidate_index)
                setattr(camera_candidate, '_variant_index', 0)
                grasp_pose, center_base = make_candidate_base_pose_and_center(
                    camera_candidate,
                    prepared.pose_estimator,
                    prepared.stamp,
                    CANONICAL_CANDIDATE_CAMERA_FRAME,
                )
                setattr(camera_candidate, '_center_base_xyz', center_base)
                plan = make_grasp_sequence_from_grasp_pose(
                    grasp_pose,
                    pregrasp_distance_m=float(
                        self.grasp_config.get('pregrasp_distance_m', 0.08)
                    ),
                    approach_offset_m=float(
                        self.grasp_config.get('final_approach_offset_m', 0.015)
                    ),
                    lift_height_m=float(
                        self.grasp_config.get('lift_height_m', 0.05)
                    ),
                    tool_approach_axis=str(
                        self.grasp_config.get('tool_approach_axis', 'z')
                    ),
                )
                gate = self._evaluate_candidate_geometry(
                    raw_candidate,
                    camera_candidate,
                    grasp_pose,
                    plan,
                )
                setattr(camera_candidate, '_geometry_gate_result', gate)
                setattr(camera_candidate, '_grasp_sequence', plan)
                setattr(
                    camera_candidate,
                    'required_open_width_m',
                    float(gate.required_open_width_m),
                )
                pose_transform = pose_matrix(grasp_pose)
                tool0_position = pose_transform[:3, 3]
                quaternion = np.asarray(
                    [
                        grasp_pose.pose.orientation.x,
                        grasp_pose.pose.orientation.y,
                        grasp_pose.pose.orientation.z,
                        grasp_pose.pose.orientation.w,
                    ],
                    dtype=float,
                )
                approach = self._pose_approach_base_xyz(grasp_pose)
                target_distance = float(
                    np.linalg.norm(np.asarray(center_base) - target_xyz)
                )
                safety_input = self._candidate_safety_input(
                    request_id=prepared.ticket.request_id,
                    snapshot_stamp_sec=prepared.ticket.snapshot_stamp_sec,
                    target_identity=target_identity,
                    track_id=candidate_index + 1,
                    variant_index=0,
                    center_base_xyz=center_base,
                    tool0_position_xyz=tool0_position,
                    quaternion_xyzw=quaternion,
                    approach_base_xyz=approach,
                    target_present=bool(
                        getattr(prepared.snapshot.object_msg, 'detected', False)
                    ),
                    same_target_instance=(
                        prepared.ticket.target_epoch
                        == self.target_instance_epoch
                    ),
                    target_absolute_distance_m=target_distance,
                    target_absolute_limit_m=float(
                        self.target_absolute_sanity_distance_m
                    ),
                    required_open_width_m=gate.required_open_width_m,
                    physical_open_width_m=self.gripper_physical_open_width_m,
                    depth_valid=(
                        validate_graspnet_depth_m(
                            getattr(camera_candidate, 'depth_m', None),
                            required=True,
                        )
                        is not None
                    ),
                    transform_valid=True,
                    geometry_valid=isinstance(gate, CandidateGateResult),
                    collision_free=bool(gate.ok),
                    snapshot_context_revision=context_revision,
                )
                hard_decision = mandatory_safety_gate(safety_input)
                if not hard_decision.ok:
                    rejections[hard_decision.code] += 1
                    continue

                cloud_distance = self._camera_candidate_cloud_distance(
                    camera_candidate
                )
                if not math.isfinite(cloud_distance):
                    cloud_distance = None
                visibility_cost = 0.0
                if bool(
                    getattr(self, 'camera_visibility_gate_enabled', False)
                    or getattr(self, 'camera_visibility_diagnostic_enabled', False)
                ):
                    _visible, visibility, _reason = (
                        self._candidate_visibility_metrics(
                            grasp_pose,
                            target_xyz,
                        )
                    )
                    visibility_cost = (
                        max(float(item['center_cost']) for item in visibility)
                        if visibility
                        else 1.0
                    )
                    if not math.isfinite(visibility_cost):
                        visibility_cost = 1.0
                support_metrics = self._candidate_support_geometry_metrics(
                    camera_candidate
                )
                jaw_tilt_cos = 1.0
                if support_metrics is not None:
                    jaw_tilt_cos = 1.0 - min(
                        1.0,
                        max(0.0, float(support_metrics['jaw_normal_cos'])),
                    )
                geometry_margin = max(
                    0.0,
                    min(
                        float(gate.support_clearance_m),
                        float(self.gripper_physical_open_width_m)
                        - float(gate.required_open_width_m),
                    ),
                )
                features = self._candidate_soft_features(
                    model_score=float(camera_candidate.score),
                    cloud_distance_m=cloud_distance,
                    center_distance_m=target_distance,
                    downward_approach_cos=float(-approach[2]),
                    visibility_center_cost=visibility_cost,
                    support_margin_m=max(0.0, float(gate.support_clearance_m)),
                    jaw_tilt_cos=jaw_tilt_cos,
                    geometry_margin_m=geometry_margin,
                    stability_hit_ratio=1.0
                    / float(max(1, self._tracking_config.window_size)),
                )
                score = soft_candidate_cost(features, weights)
                payload = LocalCandidatePayload(
                    raw_candidate_index=candidate_index,
                    variant_index=0,
                    raw_candidate=raw_candidate,
                    camera_candidate=camera_candidate,
                    grasp_pose=grasp_pose,
                    geometry_gate=gate,
                    soft_features=features,
                    score_components=dict(score.components),
                )
                observations.append(
                    CandidateObservation(
                        request_id=prepared.ticket.request_id,
                        snapshot_stamp_sec=prepared.ticket.snapshot_stamp_sec,
                        target_epoch=prepared.ticket.target_epoch,
                        target_label=target_label,
                        model_choice=prepared.model_choice,
                        center_base_xyz=center_base,
                        tool0_position_xyz=tool0_position,
                        quaternion_xyzw=quaternion,
                        approach_base_xyz=approach,
                        required_open_width_m=gate.required_open_width_m,
                        model_width_m=camera_candidate.width_m,
                        model_score=camera_candidate.score,
                        geometry_margin_m=geometry_margin,
                        pre_moveit_score=score.total,
                        payload=payload,
                    )
                )
            except Exception as exc:
                code = str(
                    getattr(exc, 'code', 'CANDIDATE_CONTRACT_INVALID')
                    or 'CANDIDATE_CONTRACT_INVALID'
                )
                rejections[code] += 1

        returned = len(candidates)
        locally_valid = len(observations)
        if returned == 0:
            rejections['REMOTE_NO_CANDIDATES'] += 1
        raw_count = _pipeline_count(
            diagnostics.get('raw_candidates', returned)
        )
        nms_count = _pipeline_count(diagnostics.get('after_nms', returned))
        collision_count = _pipeline_count(
            diagnostics.get('after_collision', returned)
        )
        stage_counts = {
            'raw': self._funnel_stage(raw_count, raw_count),
            'nms': self._funnel_stage(raw_count, min(raw_count, nms_count)),
            'remote_collision': self._funnel_stage(
                nms_count, min(nms_count, collision_count)
            ),
            'returned': self._funnel_stage(collision_count, returned),
            'locally_valid': self._funnel_stage(returned, locally_valid),
        }
        denominator = float(max(1, returned))
        return tuple(observations), {
            'input_count': returned,
            'stage_counts': stage_counts,
            'rejection_counts': dict(sorted(rejections.items())),
            'rejection_ratios': {
                code: count / denominator
                for code, count in sorted(rejections.items())
            },
            'primary_failure': (
                min(rejections, key=lambda code: (-rejections[code], code))
                if rejections
                else None
            ),
        }

    def _poll_stream_snapshot(self):
        """Poll one already-buffered rolling window without waiting."""

        if not self.enabled or not self.streaming_enabled:
            return False
        if self._safe_time() < getattr(self, '_backoff_until', rospy.Time(0)):
            return False
        try:
            input_config = self._freeze_graspnet_input_config()
            require_mask = bool(
                self._active_profile_requires_mask()
                or input_config.requires_instance_mask
            )
        except Exception:
            return False
        with self._stream_condition:
            if not self.streaming_enabled:
                return False
            target_identity = self._current_stream_target_identity()
            newest_after_ns = self.last_submitted_stamp_ns
        samples = self.frames.wait_for_samples(
            self.planning_snapshot_frames,
            0.0,
            require_mask=require_mask,
            max_age_sec=self.planning_snapshot_max_age_sec,
            collection_span_sec=self.planning_snapshot_max_span_sec,
            max_inference_latency_sec=(
                self.planning_snapshot_max_inference_latency_sec
            ),
            newest_after_ns=newest_after_ns,
            target_identity=target_identity,
        )
        if len(samples) < self.planning_snapshot_frames:
            return False
        depth_scale, depth_min_m, depth_max_m = self._snapshot_depth_config()
        snapshot = fuse_stable_samples(
            samples,
            require_mask=require_mask,
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
        if not snapshot.ok:
            return False
        return self.submit_stream_snapshot(
            snapshot,
            graspnet_input_config=input_config,
        )

    def _promotion_controller(self):
        controller = getattr(self, 'execution_plan_controller', None)
        if controller is None:
            controller = ExecutionPlanController()
            self.execution_plan_controller = controller
        return controller

    def _promotion_now_sec(self):
        clock = getattr(self, '_stream_source_clock', None)
        return float(clock()) if callable(clock) else float(time.monotonic())

    @staticmethod
    def _pose_position_and_quaternion(pose):
        position = getattr(pose, 'position', None)
        orientation = getattr(pose, 'orientation', None)
        xyz = np.asarray(
            [position.x, position.y, position.z], dtype=float
        )
        quaternion = np.asarray(
            [
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            ],
            dtype=float,
        )
        if not np.all(np.isfinite(xyz)) or not np.all(
            np.isfinite(quaternion)
        ):
            raise ValueError('execution plan pose is non-finite')
        norm = float(np.linalg.norm(quaternion))
        if norm <= 1e-12:
            raise ValueError('execution plan quaternion is invalid')
        return xyz, quaternion / norm

    @classmethod
    def _execution_plan_drift(cls, current_plan, preview_plan):
        current_poses = tuple(getattr(current_plan, 'poses', ()) or ())
        preview_poses = tuple(getattr(preview_plan, 'poses', ()) or ())
        if len(current_poses) != 4 or len(preview_poses) != 4:
            raise ValueError('execution and preview plans need four poses')
        current_xyz, current_q = cls._pose_position_and_quaternion(
            current_poses[2]
        )
        preview_xyz, preview_q = cls._pose_position_and_quaternion(
            preview_poses[2]
        )
        position_delta_m = float(np.linalg.norm(preview_xyz - current_xyz))
        preview_symmetric_q = np.asarray(
            quaternion_multiply(preview_q, (0.0, 0.0, 1.0, 0.0)),
            dtype=float,
        )
        preview_symmetric_q /= max(
            float(np.linalg.norm(preview_symmetric_q)), 1e-12
        )
        dots = (
            abs(float(np.dot(current_q, preview_q))),
            abs(float(np.dot(current_q, preview_symmetric_q))),
        )
        dot = min(1.0, max(0.0, max(dots)))
        orientation_delta_deg = float(math.degrees(2.0 * math.acos(dot)))

        def target_center(plan):
            geometry = getattr(plan, 'object_geometry', None)
            pose_base = getattr(geometry, 'pose_base', None)
            position = getattr(pose_base, 'position', None)
            center = np.asarray(
                [position.x, position.y, position.z], dtype=float
            )
            if not np.all(np.isfinite(center)):
                raise ValueError('execution target center is non-finite')
            return center

        target_drift_m = float(
            np.linalg.norm(target_center(preview_plan) - target_center(current_plan))
        )
        return position_delta_m, orientation_delta_deg, target_drift_m

    def _observe_execution_candidate_invalid(self, now_sec=None, ticket=None):
        def observe():
            with self._geometry_state_guard():
                controller = self._promotion_controller()
                decision = controller.observe_invalid(
                    now_sec=(
                        self._promotion_now_sec()
                        if now_sec is None
                        else now_sec
                    ),
                    robot_active=bool(
                        getattr(self, 'robot_execution_active', False)
                    ),
                )
                self._latest_promotion_decision = decision
                return decision

        if ticket is None:
            return observe()
        with self._stream_condition:
            self._require_stream_ticket_current_locked(ticket)
            return observe()

    def _record_preview_promotion(self, ticket, decision):
        with self._stream_condition:
            self._require_stream_ticket_current_locked(ticket)
            audit = deepcopy(
                dict(getattr(self, '_latest_streaming_audit', {}) or {})
            )
            audit['promotion'] = {
                'promote': bool(decision.promote),
                'code': str(decision.code),
                'reason': str(decision.reason),
            }
            self._latest_streaming_audit = audit
            self._latest_preview_promotion = (
                int(ticket.request_id),
                int(ticket.generation),
                int(ticket.target_epoch),
                decision,
            )

    def _request_promotion_count(self, ticket):
        recorded = getattr(self, '_latest_preview_promotion', None)
        if not isinstance(recorded, tuple) or len(recorded) != 4:
            return 0
        request_id, generation, target_epoch, decision = recorded
        if (
            int(request_id) != int(ticket.request_id)
            or int(generation) != int(ticket.generation)
            or int(target_epoch) != int(ticket.target_epoch)
        ):
            return 0
        return int(bool(getattr(decision, 'promote', False)))

    def _execution_promotion_audit_ready(self, rich_plan):
        plan_id = str(getattr(rich_plan, 'plan_id', '') or '')
        report = dict(getattr(self, '_active_gate_audit_report', {}) or {})
        outcome = dict(report.get('outcome', {}) or {})
        reference = dict(
            getattr(self, '_latest_gate_audit_reference', {}) or {}
        )
        expected_sha256 = str(reference.get('report_sha256', '') or '')
        if (
            not plan_id
            or report.get('plan_id') != plan_id
            or not bool(outcome.get('valid_plan', False))
            or not expected_sha256
            or not bool(reference.get('atomic_committed', False))
        ):
            return False
        try:
            payload = json.dumps(
                report,
                allow_nan=False,
                ensure_ascii=True,
                indent=2,
                sort_keys=True,
            ).encode('utf-8')
        except (TypeError, ValueError):
            return False
        return bool(
            hashlib.sha256(payload).hexdigest() == expected_sha256
        )

    def _decide_preview_promotion(self, rich_plan, signature, score):
        controller = self._promotion_controller()
        now_sec = self._promotion_now_sec()
        robot_active = bool(
            getattr(self, 'robot_execution_active', False)
        )
        if controller.has_execution and not robot_active:
            current = getattr(self, 'latest_rich_plan', None)
            if current is not None:
                try:
                    drift = self._execution_plan_drift(current, rich_plan)
                except (AttributeError, TypeError, ValueError):
                    drift = None
                if drift is not None:
                    controller.observe_drift(
                        position_delta_m=drift[0],
                        orientation_delta_deg=drift[1],
                        target_drift_m=drift[2],
                        now_sec=now_sec,
                        robot_active=False,
                    )
        decision = controller.observe_preview(
            signature,
            score=score,
            now_sec=now_sec,
            robot_active=robot_active,
        )
        self._latest_promotion_decision = decision
        return decision, now_sec

    def _publish_decided_promotion(
        self,
        rich_plan,
        signature,
        score,
        decision,
        now_sec,
        expected_generation=None,
    ):
        if not decision.promote:
            return decision
        plan_id = str(getattr(rich_plan, 'plan_id', '') or '').strip()
        if not plan_id or not bool(getattr(rich_plan, 'valid', False)):
            failed = PromotionDecision(
                False,
                'PLAN_INVALID',
                'preview is not a valid content-bound rich plan',
            )
            self._latest_promotion_decision = failed
            return failed
        if not self._execution_promotion_audit_ready(rich_plan):
            failed = PromotionDecision(
                False,
                'PLAN_AUDIT_NOT_READY',
                'final planning audit does not bind this execution plan',
            )
            self._latest_promotion_decision = failed
            return failed
        generation = (
            int(expected_generation)
            if expected_generation is not None
            else int(getattr(self, '_geometry_invalidation_generation', 0))
        )
        try:
            published, failure_code = self._publish_plan_pair_if_current(
                deepcopy(rich_plan), generation
            )
        except Exception as exc:
            failed = PromotionDecision(
                False,
                'PLAN_PUBLICATION_FAILED',
                'execution plan publication failed: {}'.format(exc),
            )
            self._latest_promotion_decision = failed
            return failed
        if not published:
            failed = PromotionDecision(
                False,
                'PLAN_PUBLICATION_FAILED',
                'execution plan publication rejected: {}'.format(
                    failure_code or 'unknown failure'
                ),
            )
            self._latest_promotion_decision = failed
            return failed
        self._promotion_controller().commit_execution(
            plan_id,
            signature,
            score=score,
            now_sec=now_sec,
        )
        return decision

    def _maybe_promote_preview(
        self,
        rich_plan,
        signature,
        score,
        ticket=None,
        expected_generation=None,
    ):
        """Promote one already-audited Preview and commit only on success."""

        def decide_and_publish():
            with self._geometry_state_guard():
                decision, now_sec = self._decide_preview_promotion(
                    rich_plan, signature, score
                )
                return self._publish_decided_promotion(
                    rich_plan,
                    signature,
                    score,
                    decision,
                    now_sec,
                    expected_generation=expected_generation,
                )

        if ticket is None:
            return decide_and_publish()
        with self._stream_condition:
            self._require_stream_ticket_current_locked(ticket)
            return decide_and_publish()

    def _finalize_promotion_transaction(
        self,
        prepared,
        selection,
        proposal,
        local_funnel,
        moveit_funnel,
        stable_count,
        status,
        base_report,
    ):
        """Write final evidence, then publish/commit execution authority."""

        ticket = prepared.ticket
        proposal_ticket = proposal.get('ticket')
        if (
            proposal_ticket is None
            or int(proposal_ticket.request_id) != int(ticket.request_id)
            or int(proposal_ticket.generation) != int(ticket.generation)
            or int(proposal_ticket.target_epoch) != int(ticket.target_epoch)
        ):
            raise StreamResultCancelled('GENERATION_STALE')
        with self._stream_condition:
            self._require_stream_ticket_current_locked(ticket)
            with self._geometry_state_guard():
                decision, now_sec = self._decide_preview_promotion(
                    proposal['rich_plan'],
                    proposal['signature'],
                    proposal['score'],
                )
                self._record_preview_promotion(ticket, decision)
        funnel = self._merge_pipeline_funnel(
            local_funnel,
            moveit_funnel,
            stable_count=stable_count,
            preview_count=1,
            promotion_count=int(decision.promote),
        )
        result_holder = {}

        def publish_after_audit():
            with self._geometry_state_guard():
                if bool(getattr(self, 'robot_execution_active', False)):
                    actual = PromotionDecision(
                        False,
                        'EXECUTION_FROZEN',
                        'robot execution became active before promotion',
                    )
                else:
                    actual = self._publish_decided_promotion(
                        proposal['rich_plan'],
                        proposal['signature'],
                        proposal['score'],
                        decision,
                        now_sec,
                        expected_generation=proposal[
                            'expected_generation'
                        ],
                    )
                result_holder['decision'] = actual
                self._record_preview_promotion(ticket, actual)

        self._finalize_streaming_gate_audit(
            prepared,
            selection,
            funnel,
            status,
            base_report=base_report,
            lifecycle_commit_callback=(
                publish_after_audit if decision.promote else None
            ),
            promotion_decision=decision,
        )
        actual = result_holder.get('decision', decision)
        if decision.promote and not actual.promote:
            replaced_audit_sha256 = str(
                dict(
                    getattr(self, '_latest_gate_audit_reference', {}) or {}
                ).get('report_sha256', '')
                or ''
            )
            funnel = self._merge_pipeline_funnel(
                local_funnel,
                moveit_funnel,
                stable_count=stable_count,
                preview_count=1,
                promotion_count=0,
            )
            self._finalize_streaming_gate_audit(
                prepared,
                selection,
                funnel,
                status,
                base_report=base_report,
                promotion_decision=actual,
                replace_audit_sha256=replaced_audit_sha256,
            )
        return funnel, actual

    def _publish_preview_plan(
        self,
        rich_plan,
        legacy_plan,
        ticket=None,
        commit_callback=None,
    ):
        """Publish preview-only copies; never touch execution authority."""

        outgoing_rich = deepcopy(rich_plan)
        outgoing_legacy = deepcopy(legacy_plan)
        if ticket is None:
            self.preview_rich_plan_pub.publish(outgoing_rich)
            self.preview_plan_pub.publish(outgoing_legacy)
            self.latest_preview_rich_plan = deepcopy(outgoing_rich)
            if commit_callback is not None:
                commit_callback()
            return
        with self._stream_condition:
            self._require_stream_ticket_current_locked(ticket)
            self.preview_rich_plan_pub.publish(outgoing_rich)
            self.preview_plan_pub.publish(outgoing_legacy)
            self.latest_preview_rich_plan = deepcopy(outgoing_rich)
            if commit_callback is not None:
                commit_callback()

    @staticmethod
    def _merge_pipeline_funnel(
        local_funnel,
        moveit_funnel=None,
        stable_count=0,
        preview_count=0,
        promotion_count=0,
    ):
        local = dict(local_funnel or {})
        moveit = dict(moveit_funnel or {})
        stage_counts = dict(local.get('stage_counts', {}) or {})
        stage_counts.update(dict(moveit.get('stage_counts', {}) or {}))
        locally_valid = 0
        local_stage = stage_counts.get('locally_valid')
        if isinstance(local_stage, dict):
            locally_valid = _pipeline_count(local_stage.get('passed', 0))
        stage_counts['stable'] = {
            'entered': locally_valid,
            'passed': _pipeline_count(stable_count),
            'rejected': max(0, locally_valid - _pipeline_count(stable_count)),
        }
        reachable_stage = stage_counts.get('moveit_reachable', {})
        reachable = (
            _pipeline_count(reachable_stage.get('passed', 0))
            if isinstance(reachable_stage, dict)
            else 0
        )
        stage_counts['preview'] = {
            'entered': reachable,
            'passed': _pipeline_count(preview_count),
            'rejected': max(0, reachable - _pipeline_count(preview_count)),
        }
        promoted = min(
            _pipeline_count(preview_count),
            _pipeline_count(promotion_count),
        )
        stage_counts['promoted'] = {
            'entered': _pipeline_count(preview_count),
            'passed': promoted,
            'rejected': _pipeline_count(preview_count) - promoted,
        }
        rejection_counts = Counter(
            dict(local.get('rejection_counts', {}) or {})
        )
        rejection_counts.update(
            dict(moveit.get('rejection_counts', {}) or {})
        )
        denominator = max(
            1,
            _pipeline_count(local.get('input_count', 0)),
        )
        ratios = {
            code: float(count) / float(denominator)
            for code, count in sorted(rejection_counts.items())
        }
        primary = None
        if rejection_counts:
            primary = min(
                rejection_counts,
                key=lambda code: (-rejection_counts[code], code),
            )
        return {
            'stage_counts': stage_counts,
            'rejection_counts': dict(sorted(rejection_counts.items())),
            'rejection_ratios': ratios,
            'primary_failure': primary,
        }

    @staticmethod
    def _stable_variant_pose(stable_candidate, variant_index, stamp):
        """Materialize one fused physical pose variant in ``base_link``."""

        if int(variant_index) not in (0, 1):
            raise ValueError('parallel-jaw variant index must be 0 or 1')
        quaternion = np.asarray(
            stable_candidate.quaternion_xyzw,
            dtype=float,
        )
        if int(variant_index) == 1:
            quaternion = np.asarray(
                quaternion_multiply(
                    quaternion,
                    np.asarray([0.0, 0.0, 1.0, 0.0], dtype=float),
                ),
                dtype=float,
            )
        quaternion = _normalize_quaternion(quaternion)
        position = np.asarray(
            stable_candidate.tool0_position_xyz,
            dtype=float,
        ).reshape(3)
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.header.stamp = stamp
        pose.pose.position.x = float(position[0])
        pose.pose.position.y = float(position[1])
        pose.pose.position.z = float(position[2])
        pose.pose.orientation.x = float(quaternion[0])
        pose.pose.orientation.y = float(quaternion[1])
        pose.pose.orientation.z = float(quaternion[2])
        pose.pose.orientation.w = float(quaternion[3])
        return pose

    def _recheck_and_score_stable(self, prepared, stable_candidates):
        """Re-evaluate stable fused poses against the latest accepted facts."""

        current_identity = (
            int(prepared.ticket.target_epoch),
            str(getattr(prepared.snapshot.object_msg, 'label', '') or ''),
            str(prepared.model_choice or ''),
        )
        target_xyz = np.asarray(
            prepared.geometry.center_base,
            dtype=float,
        ).reshape(3)
        context_revision = str(prepared.pose_estimator.transform_sha256)
        weights = getattr(self, 'soft_score_weights', SoftScoreWeights())
        runtime = {}
        scored = []
        for stable in tuple(stable_candidates):
            payload = stable.payload
            if not isinstance(payload, LocalCandidatePayload):
                continue
            for variant_index in (0, 1):
                try:
                    grasp_pose = self._stable_variant_pose(
                        stable,
                        variant_index,
                        prepared.stamp,
                    )
                    sequence = make_grasp_sequence_from_grasp_pose(
                        grasp_pose,
                        pregrasp_distance_m=float(
                            self.grasp_config.get('pregrasp_distance_m', 0.08)
                        ),
                        approach_offset_m=float(
                            self.grasp_config.get(
                                'final_approach_offset_m', 0.015
                            )
                        ),
                        lift_height_m=float(
                            self.grasp_config.get('lift_height_m', 0.05)
                        ),
                        tool_approach_axis=str(
                            self.grasp_config.get('tool_approach_axis', 'z')
                        ),
                    )
                    camera_candidate = deepcopy(payload.camera_candidate)
                    camera_candidate.width_m = float(stable.model_width_m)
                    setattr(
                        camera_candidate,
                        '_center_base_xyz',
                        np.asarray(stable.center_base_xyz, dtype=float),
                    )
                    setattr(camera_candidate, '_variant_index', variant_index)
                    gate = self._evaluate_candidate_geometry(
                        payload.raw_candidate,
                        camera_candidate,
                        grasp_pose,
                        sequence,
                    )
                    latest_required_width = _finite_pipeline_number(
                        getattr(gate, 'required_open_width_m', 0.0)
                    )
                    conservative_required_width = max(
                        float(stable.required_open_width_m),
                        latest_required_width,
                    )
                    current_stable = replace(
                        stable,
                        required_open_width_m=conservative_required_width,
                    )
                    target_distance = float(
                        np.linalg.norm(
                            np.asarray(stable.center_base_xyz) - target_xyz
                        )
                    )
                    latest_safety = self._candidate_safety_input(
                        request_id=prepared.ticket.request_id,
                        snapshot_stamp_sec=(
                            prepared.ticket.snapshot_stamp_sec
                        ),
                        target_identity=current_identity,
                        track_id=stable.track_id,
                        variant_index=variant_index,
                        center_base_xyz=current_stable.center_base_xyz,
                        tool0_position_xyz=current_stable.tool0_position_xyz,
                        quaternion_xyzw=self._pose_quaternion_xyzw(
                            grasp_pose
                        ),
                        approach_base_xyz=current_stable.approach_base_xyz,
                        target_present=bool(
                            getattr(
                                prepared.snapshot.object_msg,
                                'detected',
                                False,
                            )
                        ),
                        same_target_instance=(
                            current_identity
                            == (
                                current_stable.target_epoch,
                                current_stable.target_label,
                                current_stable.model_choice,
                            )
                            and prepared.ticket.target_epoch
                            == self.target_instance_epoch
                        ),
                        target_absolute_distance_m=target_distance,
                        target_absolute_limit_m=float(
                            self.target_absolute_sanity_distance_m
                        ),
                        required_open_width_m=(
                            current_stable.required_open_width_m
                        ),
                        physical_open_width_m=self.gripper_physical_open_width_m,
                        depth_valid=(
                            validate_graspnet_depth_m(
                                getattr(camera_candidate, 'depth_m', None),
                                required=True,
                            )
                            is not None
                        ),
                        transform_valid=True,
                        geometry_valid=isinstance(gate, CandidateGateResult),
                        collision_free=bool(gate.ok),
                        snapshot_context_revision=context_revision,
                    )
                    hit_ratio = float(stable.hit_count) / float(
                        max(1, stable.window_count)
                    )
                    visibility_cost = 0.0
                    if bool(
                        getattr(self, 'camera_visibility_gate_enabled', False)
                        or getattr(
                            self,
                            'camera_visibility_diagnostic_enabled',
                            False,
                        )
                    ):
                        _visible, visibility, _reason = (
                            self._candidate_visibility_metrics(
                                grasp_pose,
                                target_xyz,
                            )
                        )
                        visibility_cost = (
                            max(
                                float(item['center_cost'])
                                for item in visibility
                            )
                            if visibility
                            else 1.0
                        )
                    geometry_margin = max(
                        0.0,
                        min(
                            float(gate.support_clearance_m),
                            float(self.gripper_physical_open_width_m)
                            - float(gate.required_open_width_m),
                        ),
                    )
                    cloud_distance = self._camera_candidate_cloud_distance(
                        camera_candidate
                    )
                    if not math.isfinite(cloud_distance):
                        cloud_distance = None
                    features = replace(
                        payload.soft_features,
                        model_score=float(stable.model_score),
                        cloud_distance_m=cloud_distance,
                        center_distance_m=target_distance,
                        downward_approach_cos=float(
                            -np.asarray(stable.approach_base_xyz)[2]
                        ),
                        visibility_center_cost=max(
                            0.0,
                            _finite_pipeline_number(visibility_cost),
                        ),
                        support_margin_m=max(
                            0.0, float(gate.support_clearance_m)
                        ),
                        geometry_margin_m=geometry_margin,
                        stability_hit_ratio=hit_ratio,
                        position_dispersion_m=float(
                            stable.position_dispersion_m
                        ),
                        orientation_dispersion_rad=float(
                            stable.orientation_dispersion_rad
                        ),
                    )
                    candidate = ScoredStableCandidate(
                        stable_candidate=current_stable,
                        variant_index=variant_index,
                        latest_safety=latest_safety,
                        soft_features=features,
                        score_weights=weights,
                        evaluation_request_id=prepared.ticket.request_id,
                        evaluation_snapshot_stamp_sec=(
                            prepared.ticket.snapshot_stamp_sec
                        ),
                        evaluation_context_revision=context_revision,
                    )
                    runtime[(stable.track_id, variant_index)] = {
                        'prepared': prepared,
                        'grasp_pose': grasp_pose,
                        'sequence': sequence,
                        'camera_candidate': camera_candidate,
                        'geometry_gate': gate,
                        'scored_candidate': candidate,
                        'soft_evidence': {
                            'cloud_distance_resolved': (
                                cloud_distance is not None
                            ),
                            'cloud_distance_m': cloud_distance,
                            'position_dispersion_m': float(
                                stable.position_dispersion_m
                            ),
                            'orientation_dispersion_rad': float(
                                stable.orientation_dispersion_rad
                            ),
                        },
                    }
                    scored.append(candidate)
                except Exception:
                    continue
        condition = getattr(self, '_stream_condition', None)
        if condition is None:
            self._stable_variant_runtime = runtime
        else:
            with condition:
                self._require_stream_ticket_current_locked(prepared.ticket)
                self._stable_variant_runtime = runtime
        return tuple(scored)

    @staticmethod
    def _pose_quaternion_xyzw(grasp_pose):
        orientation = grasp_pose.pose.orientation
        return (
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
        )

    def _check_moveit_stable_candidate(self, candidate):
        runtime = getattr(self, '_stable_variant_runtime', {}).get(
            (candidate.track_id, candidate.variant_index)
        )
        grasp_pose = (
            runtime.get('grasp_pose') if isinstance(runtime, dict) else None
        )
        if grasp_pose is None:
            return MoveItResult(
                reachable=False,
                joint_path_cost=0.0,
                joint_max_delta_rad=0.0,
                reason='stable variant runtime pose is unavailable',
                failure_code='MOVEIT_CHECK_ERROR',
            )
        prepared = runtime.get('prepared')
        self._require_stream_ticket_current(prepared.ticket)
        evaluation = self._strict_moveit_evaluation(grasp_pose)
        with self._stream_condition:
            self._require_stream_ticket_current_locked(prepared.ticket)
            return self._commit_strict_moveit_evaluation(
                grasp_pose,
                evaluation,
            )

    @staticmethod
    def _preview_target_signature(stable):
        """Identify target authority, not a jittery tracker/pose realization."""

        payload = {
            'target_epoch': int(stable.target_epoch),
            'target_label': str(stable.target_label),
            'model_choice': str(stable.model_choice),
        }
        return hashlib.sha256(
            json.dumps(
                payload,
                sort_keys=True,
                separators=(',', ':'),
                allow_nan=False,
            ).encode('utf-8')
        ).hexdigest()

    def _publish_selected_preview(self, selected):
        runtime = getattr(self, '_stable_variant_runtime', {}).get(
            (selected.track_id, selected.variant_index)
        )
        if not isinstance(runtime, dict):
            raise RuntimeError('selected stable variant runtime is unavailable')
        prepared = runtime['prepared']
        candidate = deepcopy(runtime['camera_candidate'])
        stable = selected.stable_candidate
        candidate.score = float(stable.model_score)
        candidate.width_m = float(stable.model_width_m)
        setattr(
            candidate,
            'required_open_width_m',
            float(stable.required_open_width_m),
        )
        setattr(candidate, '_grasp_sequence', runtime['sequence'])
        setattr(candidate, '_track_id', int(stable.track_id))
        setattr(candidate, '_variant_index', int(selected.variant_index))
        geometry_message = geometry_estimate_to_message(
            prepared.geometry,
            snapshot=prepared.snapshot,
            stamp=prepared.stamp,
            label=stable.target_label,
        )
        rich_plan = build_rich_plan(
            candidate,
            geometry_message,
            geometry_message.header,
            stable.model_choice,
        )
        legacy_plan = rich_plan_to_legacy(rich_plan)
        streaming_audit = {
            'request_id': int(selected.evaluation_request_id),
            'snapshot_stamp_sec': float(
                selected.evaluation_snapshot_stamp_sec
            ),
            'target_epoch': int(stable.target_epoch),
            'target_label': stable.target_label,
            'model_choice': stable.model_choice,
            'raw_candidate_index': int(selected.payload.raw_candidate_index),
            'track_id': int(stable.track_id),
            'variant_index': int(selected.variant_index),
            'hit_count': int(stable.hit_count),
            'hit_request_ids': list(stable.hit_request_ids),
            'pre_moveit_score': float(selected.pre_moveit_score),
            'final_score': float(selected.final_score),
            'score_components': dict(
                soft_candidate_cost(
                    replace(
                        selected.soft_features,
                        joint_path_cost=(
                            selected.moveit_result.joint_path_cost
                        ),
                        joint_max_delta_rad=(
                            selected.moveit_result.joint_max_delta_rad
                        ),
                    ),
                    selected.score_weights,
                ).components
            ),
            'plan_id': str(rich_plan.plan_id),
            'lineage_binding': {
                'source_request_id': int(stable.request_id),
                'source_snapshot_stamp_sec': float(
                    stable.snapshot_stamp_sec
                ),
                'source_raw_candidate_index': int(
                    selected.payload.raw_candidate_index
                ),
                'source_variant_index': int(
                    selected.payload.variant_index
                ),
                'evaluation_request_id': int(
                    selected.evaluation_request_id
                ),
                'evaluation_snapshot_stamp_sec': float(
                    selected.evaluation_snapshot_stamp_sec
                ),
                'evaluation_variant_index': int(selected.variant_index),
            },
        }

        def commit_streaming_audit():
            self._latest_streaming_audit = deepcopy(streaming_audit)

        self._publish_preview_plan(
            rich_plan,
            legacy_plan,
            ticket=prepared.ticket,
            commit_callback=commit_streaming_audit,
        )
        return {
            'rich_plan': rich_plan,
            'signature': self._preview_target_signature(stable),
            'score': float(selected.final_score),
            'ticket': prepared.ticket,
            'expected_generation': int(
                prepared.request_invalidation_generation
            ),
        }

    def _finalize_streaming_gate_audit(
        self,
        prepared,
        selection,
        funnel,
        status,
        base_report=None,
        lifecycle_commit_callback=None,
        promotion_decision=None,
        replace_audit_sha256=None,
    ):
        """Atomically enrich the existing full audit with streaming lineage."""

        report = deepcopy(base_report)
        if base_report is None:
            report = deepcopy(
                getattr(self, '_active_gate_audit_report', None)
            )
        if not isinstance(report, dict):
            raise CandidateContractError(
                PLANNING_AUDIT_FAILED,
                'continuous preview audit has no request-local base report',
            )
        checked = {
            (item.track_id, item.variant_index): item
            for item in tuple(getattr(selection, 'checked', ()) or ())
        }
        selected = getattr(selection, 'selected', None)
        selected_lineage = None
        stable_evaluations = []
        current_row_annotations = {}
        runtime_items = sorted(
            dict(getattr(self, '_stable_variant_runtime', {}) or {}).items()
        )
        for (track_id, variant_index), candidate_runtime in runtime_items:
            runtime_prepared = candidate_runtime.get('prepared')
            runtime_ticket = getattr(runtime_prepared, 'ticket', None)
            if (
                runtime_ticket is None
                or int(runtime_ticket.request_id)
                != int(prepared.ticket.request_id)
                or int(runtime_ticket.generation)
                != int(prepared.ticket.generation)
                or int(runtime_ticket.target_epoch)
                != int(prepared.ticket.target_epoch)
                or abs(
                    float(runtime_ticket.snapshot_stamp_sec)
                    - float(prepared.ticket.snapshot_stamp_sec)
                )
                > 1e-9
            ):
                continue
            scored_candidate = candidate_runtime.get('scored_candidate')
            candidate_payload = getattr(scored_candidate, 'payload', None)
            if not isinstance(candidate_payload, LocalCandidatePayload):
                continue
            evaluated = checked.get(
                (track_id, variant_index), scored_candidate
            )
            if evaluated is None:
                continue
            stable = evaluated.stable_candidate
            tracking = {
                'track_id': int(track_id),
                'hit_count': int(stable.hit_count),
                'window_count': int(stable.window_count),
                'hit_request_ids': list(stable.hit_request_ids),
            }
            score_features = evaluated.soft_features
            if evaluated.moveit_result is not None:
                score_features = replace(
                    score_features,
                    joint_path_cost=evaluated.moveit_result.joint_path_cost,
                    joint_max_delta_rad=(
                        evaluated.moveit_result.joint_max_delta_rad
                    ),
                )
            score_components = dict(
                soft_candidate_cost(
                    score_features,
                    evaluated.score_weights,
                ).components
            )
            final_score = (
                None
                if evaluated.final_score is None
                else float(evaluated.final_score)
            )
            moveit = (
                None
                if evaluated.moveit_result is None
                else asdict(evaluated.moveit_result)
            )
            evaluation_request_id = int(
                getattr(
                    evaluated,
                    'evaluation_request_id',
                    prepared.ticket.request_id,
                )
            )
            evaluation_snapshot_stamp_sec = float(
                getattr(
                    evaluated,
                    'evaluation_snapshot_stamp_sec',
                    prepared.ticket.snapshot_stamp_sec,
                )
            )
            lineage_binding = {
                'source_request_id': int(stable.request_id),
                'source_snapshot_stamp_sec': float(
                    stable.snapshot_stamp_sec
                ),
                'source_raw_candidate_index': int(
                    candidate_payload.raw_candidate_index
                ),
                'source_variant_index': int(
                    candidate_payload.variant_index
                ),
                'evaluation_request_id': evaluation_request_id,
                'evaluation_snapshot_stamp_sec': (
                    evaluation_snapshot_stamp_sec
                ),
                'evaluation_variant_index': int(variant_index),
            }
            is_selected = bool(
                selected is not None
                and selected.track_id == track_id
                and selected.variant_index == variant_index
            )
            evaluation_record = {
                'candidate_index': int(
                    candidate_payload.raw_candidate_index
                ),
                'variant_index': int(variant_index),
                'selected': is_selected,
                'tracking': tracking,
                'lineage_binding': lineage_binding,
                'score_components': score_components,
                'pre_moveit_score': float(evaluated.pre_moveit_score),
                'final_score': final_score,
                'moveit': moveit,
                'latest_soft_evidence': deepcopy(
                    candidate_runtime.get('soft_evidence', {})
                ),
            }
            stable_evaluations.append(evaluation_record)
            if is_selected:
                selected_lineage = deepcopy(evaluation_record)
            source_is_evaluation = bool(
                int(stable.request_id) == evaluation_request_id
                and abs(
                    float(stable.snapshot_stamp_sec)
                    - evaluation_snapshot_stamp_sec
                )
                <= 1e-9
            )
            if source_is_evaluation:
                current_row_annotations[
                    (
                        int(candidate_payload.raw_candidate_index),
                        int(variant_index),
                    )
                ] = evaluation_record

        for row in list(report.get('rows', []) or []):
            row['selected'] = False
            lineage = (
                int(row.get('candidate_index', -1)),
                int(row.get('variant_index', -1)),
            )
            evaluation_record = current_row_annotations.get(lineage)
            if evaluation_record is None:
                continue
            row.update(deepcopy(evaluation_record))

        report['report_version'] = 3
        report['mode'] = 'continuous_preview'
        report['request_id'] = int(prepared.ticket.request_id)
        report['generation'] = int(prepared.ticket.generation)
        report['target_epoch'] = int(prepared.ticket.target_epoch)
        report['pipeline_funnel'] = deepcopy(dict(funnel or {}))
        report.setdefault('summary', {})['pipeline_funnel'] = deepcopy(
            dict(funnel or {})
        )
        report['candidate_row_lineage'] = [
            {
                'candidate_index': int(row.get('candidate_index', -1)),
                'variant_index': int(row.get('variant_index', -1)),
                'selected': bool(row.get('selected', False)),
                'tracking': deepcopy(row.get('tracking')),
                'lineage_binding': deepcopy(
                    row.get('lineage_binding')
                ),
            }
            for row in list(report.get('rows', []) or [])
        ]
        report['stable_evaluations'] = deepcopy(stable_evaluations)
        report['lineage'] = deepcopy(stable_evaluations)
        report['selected'] = selected_lineage
        preview_audit = dict(
            getattr(self, '_latest_streaming_audit', {}) or {}
        )
        report['plan_id'] = (
            str(preview_audit.get('plan_id', '') or '')
            if selected is not None
            and int(preview_audit.get('request_id', -1))
            == int(prepared.ticket.request_id)
            else ''
        )
        promotes_execution = bool(
            isinstance(promotion_decision, PromotionDecision)
            and promotion_decision.promote
            and report.get('plan_id')
        )
        promotion_failure = bool(
            isinstance(promotion_decision, PromotionDecision)
            and not promotion_decision.promote
            and str(promotion_decision.code).startswith('PLAN_')
        )
        report['promotion'] = (
            None
            if not isinstance(promotion_decision, PromotionDecision)
            else {
                'promote': bool(promotion_decision.promote),
                'code': str(promotion_decision.code),
                'reason': str(promotion_decision.reason),
            }
        )
        report['outcome'] = {
            'code': (
                'PLAN_READY'
                if promotes_execution
                else (
                    str(promotion_decision.code)
                    if promotion_failure
                    else str(status or '')
                )
            ),
            'reason': (
                str(promotion_decision.reason)
                if promotes_execution or promotion_failure
                else str(status or '')
            ),
            'valid_plan': promotes_execution,
            'preview_valid': selected is not None,
        }
        summary = {
            'pipeline_funnel': deepcopy(dict(funnel or {})),
            'status': str(status or ''),
            'promotion': deepcopy(report['promotion']),
        }

        def commit_audit(reference):
            self._active_gate_audit_report = deepcopy(report)
            self._latest_gate_audit_summary = deepcopy(summary)
            self._latest_gate_audit_reference = dict(reference)
            self._publish_bounded_gate_audit(
                selected_lineage,
                summary=summary,
                reference=reference,
            )
            if lifecycle_commit_callback is not None:
                lifecycle_commit_callback()

        self._write_gate_audit_report(
            report,
            ticket=prepared.ticket,
            commit_callback=commit_audit,
            replace_reference_sha256=replace_audit_sha256,
        )

    def _accept_prediction(self, prepared):
        """Accept one correlated result into tracking and preview selection."""

        ticket = prepared.ticket
        self._require_stream_ticket_current(ticket)
        if not self._activate_prepared_geometry(prepared):
            raise RuntimeError('prepared prediction geometry is stale')
        base_audit_report = None
        if isinstance(prepared, PreparedPrediction):
            base_audit_report = self._run_candidate_gate_audit(
                prepared.candidates,
                prepared.stamp,
                CANONICAL_CANDIDATE_CAMERA_FRAME,
                remote_diagnostics=prepared.remote_diagnostics,
                candidate_pose_estimator=prepared.pose_estimator,
                commit_state=False,
                graspnet_input_audit=prepared.graspnet_input_audit,
            )
        target_label = str(
            getattr(prepared.snapshot.object_msg, 'label', '') or ''
        )
        target_identity = (
            int(ticket.target_epoch),
            target_label,
            str(
                getattr(
                    prepared,
                    'model_choice',
                    getattr(self, '_last_model_choice', ''),
                )
                or ''
            ),
        )
        observations, local_funnel = self._evaluate_local_candidates(prepared)
        stable = self._update_candidate_tracker(
            request_id=ticket.request_id,
            observations=observations,
            target_identity=target_identity,
            ticket=ticket,
        )
        # A tracker may retain a previously stable track across one missed
        # request.  It is useful to record that miss, but a request with zero
        # locally valid candidates must report its current hard failure and
        # must not revalidate or publish the retained pose as fresh evidence.
        if not observations or not stable:
            with self._stream_condition:
                self._require_stream_ticket_current_locked(ticket)
                self._stable_variant_runtime = {}
            funnel = self._merge_pipeline_funnel(
                local_funnel,
                stable_count=0,
                preview_count=0,
            )
            status = 'STABILITY_PENDING'
            local_stage = dict(
                funnel.get('stage_counts', {}).get(
                    'locally_valid',
                    {},
                )
                or {}
            )
            if _pipeline_count(local_stage.get('passed', 0)) == 0:
                primary = funnel.get('primary_failure')
                if primary:
                    count = int(
                        funnel.get('rejection_counts', {}).get(primary, 0)
                    )
                    status = '%s:%d' % (primary, count)
            if isinstance(prepared, PreparedPrediction):
                self._finalize_streaming_gate_audit(
                    prepared,
                    None,
                    funnel,
                    status,
                    base_report=base_audit_report,
                    lifecycle_commit_callback=lambda: (
                        self._observe_execution_candidate_invalid(
                            ticket=ticket
                        )
                    ),
                )
            else:
                self._observe_execution_candidate_invalid(ticket=ticket)
            return {'status': status, 'funnel': funnel}

        scored = self._recheck_and_score_stable(prepared, stable)
        selection = bounded_moveit_select(
            scored,
            self._check_moveit_stable_candidate,
            top_n=int(self.moveit_top_n),
        )
        moveit_funnel = selection.funnel.to_dict()
        preview_count = 0
        promotion_count = 0
        promotion_proposal = None
        status = 'NO_REACHABLE_STABLE_CANDIDATE'
        if selection.selected is not None:
            promotion_proposal = self._publish_selected_preview(
                selection.selected
            )
            preview_count = 1
            status = 'PREVIEW_READY'
        promotion_decision = None
        if (
            isinstance(promotion_proposal, dict)
            and isinstance(prepared, PreparedPrediction)
        ):
            funnel, promotion_decision = (
                self._finalize_promotion_transaction(
                    prepared,
                    selection,
                    promotion_proposal,
                    local_funnel,
                    moveit_funnel,
                    len(stable),
                    status,
                    base_audit_report,
                )
            )
            return {'status': status, 'funnel': funnel}

        funnel = self._merge_pipeline_funnel(
            local_funnel,
            moveit_funnel,
            stable_count=len(stable),
            preview_count=preview_count,
            promotion_count=promotion_count,
        )
        primary = funnel.get('primary_failure')
        if preview_count == 0 and primary:
            count = int(funnel['rejection_counts'].get(primary, 0))
            status = '%s:%d' % (primary, count)
        if isinstance(prepared, PreparedPrediction):
            self._finalize_streaming_gate_audit(
                prepared,
                selection,
                funnel,
                status,
                base_report=base_audit_report,
                lifecycle_commit_callback=(
                    (lambda: self._observe_execution_candidate_invalid(
                        ticket=ticket
                    ))
                    if preview_count == 0
                    else None
                ),
                promotion_decision=promotion_decision,
            )
        elif preview_count == 0:
            self._observe_execution_candidate_invalid(ticket=ticket)
        return {'status': status, 'funnel': funnel}

    def shutdown_streaming_worker(self, timeout_sec=2.0):
        queued_ticket = None
        pending_request_id = None
        queued_terminal_claimed = False
        pending_terminal_claimed = False
        with self._stream_condition:
            queued_ticket = self._stream_worker_ticket
            pending_request_id = self._pending_request_id
            self.streaming_enabled = False
            self.inference_coordinator.stop()
            if queued_ticket is not None:
                try:
                    self.inference_coordinator.complete(
                        queued_ticket,
                        now_sec=float(self._stream_source_clock()),
                        target_epoch=self.target_instance_epoch,
                    )
                except Exception:
                    pass
                queued_terminal_claimed = (
                    self._claim_terminal_request_locked(
                        queued_ticket.request_id,
                        'GENERATION_STALE',
                    )
                )
            if pending_request_id is not None:
                pending_terminal_claimed = (
                    self._claim_terminal_request_locked(
                        pending_request_id,
                        'GENERATION_STALE',
                    )
                )
            self._stream_worker_ticket = None
            self._pending_request_id = None
            self._stream_shutdown.set()
            self._stream_condition.notify_all()
        if queued_ticket is not None:
            self._emit_pending_drop_metrics(
                queued_ticket.request_id,
                'GENERATION_STALE',
                terminal_claimed=queued_terminal_claimed,
            )
        if pending_request_id is not None:
            self._emit_pending_drop_metrics(
                pending_request_id,
                'GENERATION_STALE',
                terminal_claimed=pending_terminal_claimed,
            )
        worker = self._stream_worker
        if worker is not None and worker is not threading.current_thread():
            worker.join(max(0.0, float(timeout_sec)))
        return worker is None or not worker.is_alive()

    def _stream_worker_main(self):
        while True:
            with self._stream_condition:
                while (
                    self._stream_worker_ticket is None
                    and not self._stream_shutdown.is_set()
                ):
                    self._stream_condition.wait(0.5)
                if self._stream_shutdown.is_set():
                    return
                ticket = self._stream_worker_ticket
                self._stream_worker_ticket = None
                self._stream_worker_busy = True
                self._pipeline_counters['started'] += 1

            prepared = None
            error = None
            ticket_stale_before_prediction = bool(
                not self.streaming_enabled
                or int(ticket.generation) != int(self._stream_generation)
                or int(ticket.target_epoch) != int(self.target_instance_epoch)
            )
            if not ticket_stale_before_prediction:
                try:
                    prepared = self._prepare_and_predict(ticket)
                except Exception as exc:
                    error = exc

            with self._stream_condition:
                completion_now_sec = float(self._stream_source_clock())
                completion = self.inference_coordinator.complete(
                    ticket,
                    now_sec=completion_now_sec,
                    target_epoch=self.target_instance_epoch,
                )
                if completion.next_ticket is not None:
                    self._pending_request_id = None
            self._request_telemetry.pop(int(ticket.request_id), None)
            funnel = {}
            status = completion.code
            final_accepted = bool(completion.accepted)
            if completion.accepted and prepared is not None and error is None:
                try:
                    accepted_result = self._accept_prediction(prepared)
                    if isinstance(accepted_result, dict):
                        funnel = dict(accepted_result.get('funnel', {}) or {})
                        status = str(
                            accepted_result.get('status', 'ACCEPTED')
                        )
                    else:
                        status = 'ACCEPTED'
                except StreamResultCancelled as exc:
                    final_accepted = False
                    status = exc.code
                    error = None
                except Exception as exc:
                    final_accepted = False
                    error = exc
                    status = 'ACCEPT_FAILED'
            elif error is not None and completion.accepted:
                final_accepted = False
                status = 'PREDICT_FAILED'

            final_now_sec = float(self._stream_source_clock())
            with self._stream_condition:
                terminal_claimed = self._claim_terminal_request_locked(
                    ticket.request_id,
                    status,
                    accepted=final_accepted,
                )
            end_to_end_ms = max(
                0.0,
                (
                    final_now_sec
                    - float(ticket.submitted_monotonic_sec)
                )
                * 1000.0,
            )
            self._latency_history_ms.append(end_to_end_ms)
            trusted_performance = (
                dict(getattr(prepared, 'remote_performance', {}) or {})
                if final_accepted and prepared is not None
                else {}
            )
            metrics = build_pipeline_metrics(
                event='request_completed',
                request_id=ticket.request_id,
                generation=ticket.generation,
                target_epoch=ticket.target_epoch,
                snapshot_stamp_sec=ticket.snapshot_stamp_sec,
                status=status,
                drop_reason='' if final_accepted else status,
                counters=self._pipeline_counters,
                pending_replacements=self._pending_replacements,
                ros_prepare_ms=getattr(prepared, 'ros_prepare_ms', 0.0),
                encode_ms=getattr(prepared, 'encode_ms', 0.0),
                transport_ms=getattr(prepared, 'transport_ms', 0.0),
                decode_ms=getattr(prepared, 'decode_ms', 0.0),
                remote_performance=trusted_performance,
                end_to_end_ms=end_to_end_ms,
                result_age_ms=max(
                    0.0,
                    (
                        final_now_sec
                        - float(ticket.snapshot_stamp_sec)
                    )
                    * 1000.0,
                ),
                latency_history_ms=self._latency_history_ms,
                funnel=funnel,
            )
            if error is not None:
                metrics['error'] = str(error)
            active_audit = getattr(self, '_active_gate_audit_report', None)
            if (
                final_accepted
                and isinstance(prepared, PreparedPrediction)
                and isinstance(active_audit, dict)
                and active_audit.get('mode') == 'continuous_preview'
                and int(active_audit.get('request_id', -1))
                == int(ticket.request_id)
            ):
                try:
                    active_audit = deepcopy(active_audit)
                    active_audit['pipeline_metrics'] = deepcopy(metrics)

                    def commit_audit_metrics(reference):
                        self._active_gate_audit_report = deepcopy(
                            active_audit
                        )
                        self._latest_gate_audit_reference = dict(reference)

                    audit_reference = self._write_gate_audit_report(
                        active_audit,
                        ticket=ticket,
                        commit_callback=commit_audit_metrics,
                    )
                    metrics['audit_reference'] = dict(audit_reference)
                except StreamResultCancelled as exc:
                    final_accepted = False
                    status = exc.code
                    with self._stream_condition:
                        self._pipeline_counters['accepted'] = max(
                            0,
                            int(self._pipeline_counters['accepted']) - 1,
                        )
                        self._pipeline_counters['stale'] += 1
                    metrics.update(
                        {
                            'status': status,
                            'drop_reason': status,
                            'accepted': int(
                                self._pipeline_counters['accepted']
                            ),
                            'stale': int(
                                self._pipeline_counters['stale']
                            ),
                        }
                    )
                except Exception as exc:
                    metrics['audit_metrics_error'] = str(exc)
            with self._stream_condition:
                if not terminal_claimed:
                    metrics = None
                else:
                    self.pipeline_metrics.append(metrics)
            if metrics is None:
                publisher = None
                encoded_metrics = ''
            else:
                publisher = getattr(self, 'pipeline_metrics_pub', None)
                encoded_metrics = bounded_metrics_json(metrics)
            if publisher is not None:
                try:
                    publisher.publish(String(encoded_metrics))
                except Exception as exc:
                    try:
                        rospy.logwarn(
                            'pipeline metrics publication failed: %s', exc
                        )
                    except Exception:
                        pass
            try:
                rospy.loginfo('remote 6D pipeline metrics: %s', encoded_metrics)
            except Exception:
                pass

            self._planning_snapshot_active = False
            self._planning_object_msg = None
            self._planning_object_time = None
            self._target_cloud_request_active = False
            self._current_prepared_prediction = None

            with self._stream_condition:
                self._stream_worker_busy = False
                cancellation_code = ''
                next_terminal_claimed = False
                if completion.next_ticket is not None:
                    cancellation_code = (
                        self._stream_ticket_cancellation_code_locked(
                            completion.next_ticket
                        )
                    )
                    if not cancellation_code:
                        self._stream_worker_ticket = completion.next_ticket
                        self._stream_condition.notify_all()
                    else:
                        try:
                            self.inference_coordinator.complete(
                                completion.next_ticket,
                                now_sec=float(self._stream_source_clock()),
                                target_epoch=self.target_instance_epoch,
                            )
                        except Exception:
                            pass
                        next_terminal_claimed = (
                            self._claim_terminal_request_locked(
                                completion.next_ticket.request_id,
                                cancellation_code,
                            )
                        )
            if completion.next_ticket is not None and cancellation_code:
                self._emit_pending_drop_metrics(
                    completion.next_ticket.request_id,
                    cancellation_code,
                    terminal_claimed=next_terminal_claimed,
                )

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if self.enabled and self.streaming_enabled:
                self._poll_stream_snapshot()
            rate.sleep()

    def request_plan_cb(self, req):
        trigger = bool(getattr(req, 'trigger', False))
        if trigger and not self.enabled:
            return TriggerZeroResponse(False, 'remote 6D disabled')
        if trigger:
            changed = self.start_streaming()
            message = (
                'continuous remote 6D inference started'
                if changed
                else 'continuous remote 6D inference already running'
            )
        else:
            changed = self.stop_streaming()
            message = (
                'continuous remote 6D inference stopped'
                if changed
                else 'continuous remote 6D inference already stopped'
            )
        publisher = getattr(self, 'status_pub', None)
        if publisher is not None:
            publisher.publish(String(message))
        return TriggerZeroResponse(True, message)

    def replan_execution_cb(self, req):
        """Request a future idle Preview promotion; never control inference."""

        if type(getattr(req, 'trigger', None)) is not bool or not req.trigger:
            return TriggerZeroResponse(
                False,
                'replan_execution requires trigger=true',
            )
        with self._geometry_state_guard():
            decision = self._promotion_controller().request_replan(
                robot_active=bool(
                    getattr(self, 'robot_execution_active', False)
                )
            )
            self._latest_promotion_decision = decision
        success = decision.code == 'REPLAN_REQUESTED'
        return TriggerZeroResponse(success, decision.reason)

    def _process_latest_frame(self):
        if rospy.Time.now() < self._backoff_until:
            return
        try:
            input_config = self._freeze_graspnet_input_config()
            require_mask = bool(
                self._active_profile_requires_mask()
                or input_config.requires_instance_mask
            )
        except GraspNetInputContextError as exc:
            self._invalidate_geometry(exc.code, exc.reason)
            self.status_pub.publish(String('%s: %s' % (exc.code, exc.reason)))
            return
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
        self._process_frame(
            snapshot,
            manual=False,
            graspnet_input_config=input_config,
        )

    def _active_profile_requires_mask(self):
        pcfg = rospy.get_param('/perception', {})
        model_choice = str(pcfg.get('yolo_model_choice', 'original'))
        previous_choice = getattr(self, '_last_model_choice', None)
        if previous_choice is not None and model_choice != previous_choice:
            self._last_model_choice = model_choice
            self._invalidate_geometry(
                'MODEL_RELOADED',
                'model choice changed from %s to %s'
                % (previous_choice, model_choice),
            )
            self._advance_target_instance_epoch('MODEL_RELOADED')
        else:
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

    def _snapshot_depth_config(self):
        if not bool(rospy.core.is_initialized()):
            return float(getattr(self, 'camera_depth_scale', 0.001)), 0.03, 2.0
        cam_cfg = rospy.get_param('/camera', {})
        pcfg = rospy.get_param('/perception', {})
        depth_scale = float(cam_cfg.get('depth_scale', 0.001))
        depth_min_m = float(pcfg.get('depth_min_m', 0.03))
        depth_max_m = float(pcfg.get('depth_max_m', 2.0))
        return depth_scale, depth_min_m, depth_max_m

    def _freeze_graspnet_input_config(self):
        """Read every input-mode parameter once for one planning request."""
        def request_param(name, default):
            # Offline unit tests construct the node without rospy.init_node;
            # production always reads the live parameter server here.
            if not bool(rospy.core.is_initialized()):
                return default
            return rospy.get_param(name, default)

        mode = str(
            request_param(
                '/grasp_6d/remote/graspnet_input_mode',
                getattr(self, 'graspnet_input_mode', MASKED_TARGET),
            )
            or ''
        ).strip().lower()
        if mode not in VALID_MODES:
            raise GraspNetInputContextError(
                'MODE_INVALID',
                'graspnet_input_mode must be one of %s, got %r'
                % (', '.join(sorted(VALID_MODES)), mode),
            )

        def read(suffix, attribute, default):
            return request_param(
                '/grasp_6d/remote/' + suffix,
                getattr(self, attribute, default),
            )

        return FrozenGraspNetInputConfig(
            mode=mode,
            context_margin_px=read(
                'graspnet_input_context_margin_px',
                'graspnet_input_context_margin_px',
                DEFAULT_CONTEXT_MARGIN_PX,
            ),
            context_expand_ratio=read(
                'graspnet_input_context_expand_ratio',
                'graspnet_input_context_expand_ratio',
                DEFAULT_CONTEXT_EXPAND_RATIO,
            ),
            context_max_margin_px=read(
                'graspnet_input_context_max_margin_px',
                'graspnet_input_context_max_margin_px',
                DEFAULT_CONTEXT_MAX_MARGIN_PX,
            ),
            target_guard_px=read(
                'graspnet_input_target_guard_px',
                'graspnet_input_target_guard_px',
                DEFAULT_TARGET_GUARD_PX,
            ),
            support_band_m=read(
                'graspnet_input_support_band_m',
                'graspnet_input_support_band_m',
                DEFAULT_CONTEXT_PLANE_DISTANCE_M,
            ),
            min_target_points=read(
                'graspnet_input_min_target_points',
                'graspnet_input_min_target_points',
                DEFAULT_MIN_TARGET_POINTS,
            ),
            min_support_points=read(
                'graspnet_input_min_support_points',
                'graspnet_input_min_support_points',
                DEFAULT_MIN_SUPPORT_POINTS,
            ),
            min_total_points=read(
                'graspnet_input_min_total_points',
                'graspnet_input_min_total_points',
                DEFAULT_MIN_TOTAL_POINTS,
            ),
            min_target_fraction=read(
                'graspnet_input_min_target_fraction',
                'graspnet_input_min_target_fraction',
                DEFAULT_MIN_TARGET_FRACTION,
            ),
            bbox_min_iou=read(
                'graspnet_input_bbox_min_iou',
                'graspnet_input_bbox_min_iou',
                DEFAULT_DETECTED_BBOX_MIN_IOU,
            ),
            candidate_target_gate_enabled=bool(
                request_param(
                    '/grasp_6d/remote/candidate_target_gate_enabled',
                    getattr(self, 'candidate_target_gate_enabled', True),
                )
            ),
        )

    @staticmethod
    def _require_graspnet_input_prerequisites(snapshot, config):
        if not isinstance(config, FrozenGraspNetInputConfig):
            raise GraspNetInputContextError(
                'CONFIG_INVALID',
                'request has no frozen GraspNet input configuration',
            )
        if config.requires_instance_mask:
            if str(getattr(snapshot, 'source_mode', '') or '') != 'instance_mask':
                raise GraspNetInputContextError(
                    'INSTANCE_MASK_REQUIRED',
                    '%s requires an instance-mask planning snapshot'
                    % config.mode,
                )
            mask = np.asarray(getattr(snapshot, 'object_mask', ()))
            if mask.ndim != 2 or not np.any(mask > 0):
                raise GraspNetInputContextError(
                    'INSTANCE_MASK_REQUIRED',
                    '%s requires a non-empty instance mask' % config.mode,
                )
            if (
                config.requires_candidate_target_gate
                and not bool(config.candidate_target_gate_enabled)
            ):
                raise GraspNetInputContextError(
                    'CANDIDATE_TARGET_GATE_REQUIRED',
                    '%s requires candidate_target_gate_enabled=true'
                    % config.mode,
                )

    def _build_frozen_graspnet_input(
        self,
        snapshot,
        target_depth,
        config,
        commit_audit=True,
    ):
        self._require_graspnet_input_prerequisites(snapshot, config)
        if config.requires_support_plane:
            convention = normalize_candidate_frame_convention(
                getattr(
                    self,
                    'target_projection_frame_convention',
                    'ros_camera_link',
                )
            )
            if convention != 'ros_camera_link':
                raise GraspNetInputContextError(
                    'SUPPORT_PLANE_FRAME_INVALID',
                    'context_roi support plane must be expressed in '
                    'ros_camera_link, got %s' % convention,
                )
            plane_point = getattr(
                self,
                'latest_support_plane_camera_point',
                None,
            )
            plane_normal = getattr(
                self,
                'latest_support_plane_camera_normal',
                None,
            )
            try:
                point = np.asarray(plane_point, dtype=float)
                normal = np.asarray(plane_normal, dtype=float)
                plane_valid = (
                    point.shape == (3,)
                    and normal.shape == (3,)
                    and np.all(np.isfinite(point))
                    and np.all(np.isfinite(normal))
                    and float(np.linalg.norm(normal)) > 1e-12
                )
            except Exception:
                plane_valid = False
            if not plane_valid:
                raise GraspNetInputContextError(
                    'SUPPORT_PLANE_INVALID',
                    '%s requires the valid support plane from this snapshot'
                    % config.mode,
                )
        else:
            plane_point = None
            plane_normal = None

        depth_scale, depth_min_m, depth_max_m = self._snapshot_depth_config()
        effective_mask = np.asarray(snapshot.object_mask)
        if str(getattr(snapshot, 'source_mode', '') or '') == 'bbox_depth':
            # Legacy detect snapshots have no instance mask; masked_target
            # still uses the fail-closed foreground isolated by the existing
            # bbox/support-depth geometry path.
            effective_mask = np.where(
                np.asarray(target_depth) > 0,
                255,
                0,
            ).astype(np.uint8)
        result = build_graspnet_input_context(
            mode=config.mode,
            target_depth_raw=target_depth,
            object_mask=effective_mask,
            full_depth_raw=snapshot.depth_raw,
            color_bgr=snapshot.color_bgr,
            intrinsics=self._camera_intrinsics(),
            support_plane_point_camera=plane_point,
            support_plane_normal_camera=plane_normal,
            depth_scale=depth_scale,
            depth_min_m=depth_min_m,
            depth_max_m=depth_max_m,
            context_plane_distance_m=config.support_band_m,
            context_margin_px=config.context_margin_px,
            context_expand_ratio=config.context_expand_ratio,
            context_max_margin_px=config.context_max_margin_px,
            target_guard_px=config.target_guard_px,
            detected_bbox_xywh=(
                snapshot.bbox
                if str(getattr(snapshot, 'source_mode', '') or '')
                == 'instance_mask'
                else None
            ),
            detected_bbox_min_iou=config.bbox_min_iou,
            min_target_points=config.min_target_points,
            min_support_points=config.min_support_points,
            min_total_points=config.min_total_points,
            min_target_fraction=config.min_target_fraction,
        )
        audit = asdict(result.audit)
        depth_hash = array_sha256(result.depth_raw)
        mask_hash = array_sha256(np.asarray(effective_mask, dtype=np.uint8))
        audit.update(
            {
                'input_depth_sha256': depth_hash,
                'object_mask_sha256': mask_hash,
                # Concise aliases used by offline gate-report tooling.
                'depth_sha256': depth_hash,
                'mask_sha256': mask_hash,
                'input_depth_shape': list(result.depth_raw.shape),
                'input_depth_dtype': str(result.depth_raw.dtype),
                'candidate_target_gate_enabled': bool(
                    config.candidate_target_gate_enabled
                ),
                'frozen_config': asdict(config),
            }
        )
        if bool(commit_audit):
            self._active_graspnet_input_audit = audit
            return result
        return result, audit

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
                collection_span_sec=self.planning_snapshot_max_span_sec,
                max_inference_latency_sec=(
                    self.planning_snapshot_max_inference_latency_sec
                ),
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
            failure_code, failure_reason = self.frames.mask_timeout_failure(
                self.planning_snapshot_frames,
                self.planning_snapshot_max_age_sec,
                self.planning_snapshot_max_inference_latency_sec,
            )
            return None, failure_code, failure_reason
        return None, 'DEPTH_UNSTABLE', 'no three fresh timestamp-matched RGB-D-object samples'

    def _process_frame(
        self,
        snapshot,
        manual=False,
        graspnet_input_config=None,
    ):
        if not self._request_lock.acquire(False):
            message = 'remote 6D request already running'
            if manual:
                self.status_pub.publish(String(message))
            return False, message
        if not isinstance(snapshot, SnapshotResult) or not snapshot.ok:
            self._request_lock.release()
            return False, 'remote 6D request requires a valid planning snapshot'
        stamp = self._snapshot_ros_stamp(snapshot)
        if graspnet_input_config is None:
            try:
                graspnet_input_config = self._freeze_graspnet_input_config()
            except GraspNetInputContextError as exc:
                message = '%s: %s' % (exc.code, exc.reason)
                self._invalidate_geometry(
                    exc.code,
                    exc.reason,
                    stamp=stamp,
                    snapshot=snapshot,
                )
                self._publish_error(message)
                self._request_lock.release()
                return False, message
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
        frame_id = snapshot.frame_id
        request_invalidation_generation = self._capture_geometry_generation()
        self._planning_snapshot_active = True
        self._planning_object_msg = snapshot.object_msg
        self._planning_object_time = stamp
        try:
            self._active_graspnet_input_audit = {}
            # The mode-specific mask/gate contract is request-local and must
            # fail before runtime refresh, geometry estimation, TF lookup, or
            # a WSL request.  In particular, bbox_depth can never masquerade
            # as an instance mask in context_roi/full_scene.
            self._require_graspnet_input_prerequisites(
                snapshot,
                graspnet_input_config,
            )
            self._refresh_runtime_params()
            (
                self.gate_audit_output_path,
                self.mujoco_audit_output_path,
            ) = validate_distinct_audit_paths(
                validate_mandatory_planning_audit(
                    getattr(self, 'gate_audit_enabled', True),
                    getattr(self, 'gate_audit_output_path', ''),
                ),
                getattr(
                    self,
                    'mujoco_audit_output_path',
                    MUJOCO_AUDIT_DEFAULT_PATH,
                ),
            )
            # Runtime refresh may observe newer ROS values, but this request
            # must keep the gate state captured before snapshot collection.
            self.candidate_target_gate_enabled = bool(
                graspnet_input_config.candidate_target_gate_enabled
            )
            self._cached_tool_from_camera = None
            self._cached_tool_from_camera_stamp_ns = 0
            self._active_gate_audit_report = None
            # Every filter below must use geometry from this exact RGB-D
            # request, even when remote inference takes several seconds.
            self._target_cloud_request_active = False
            self._clear_geometry_cache()
            estimate, depth_for_remote, transform = self._prepare_snapshot_geometry(
                snapshot,
                stamp,
            )
            if not estimate.ok:
                _applied, message = self._invalidate_geometry_if_current(
                    request_invalidation_generation,
                    estimate.failure_code,
                    estimate.failure_reason,
                    stamp=stamp,
                    snapshot=snapshot,
                )
                self._publish_error(message)
                return False, message
            frozen_candidate_pose_estimator = FrozenSnapshotCandidatePoseEstimator(
                transform,
                stamp,
                frame_id,
                raw_candidate_convention=self.candidate_frame_convention,
            )
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
            graspnet_input = self._build_frozen_graspnet_input(
                snapshot,
                depth_for_remote,
                graspnet_input_config,
            )
            if bool(
                getattr(self, 'camera_visibility_gate_enabled', False)
                or getattr(self, 'camera_visibility_diagnostic_enabled', False)
            ):
                # Eye-in-hand visibility is part of candidate filtering and
                # audit.  Resolve it once at the same strictly-positive
                # planning snapshot stamp before WSL inference; after this
                # boundary every consumer is cache-only.
                self._freeze_tool_from_camera_matrix(stamp)
            bbox = tuple(snapshot.bbox or (0, 0, 0, 0))
            roi_message = (
                '%s input=%s x=%d y=%d w=%d h=%d target_depth=%d remote_depth=%d'
                % (
                    snapshot.source_mode,
                    graspnet_input.mode,
                    int(bbox[0]) if len(bbox) > 0 else 0,
                    int(bbox[1]) if len(bbox) > 1 else 0,
                    int(bbox[2]) if len(bbox) > 2 else 0,
                    int(bbox[3]) if len(bbox) > 3 else 0,
                    valid_depth_count(depth_for_remote),
                    valid_depth_count(graspnet_input.depth_raw),
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
                self._legacy_request_id = int(
                    getattr(self, '_legacy_request_id', 0)
                ) + 1
                candidates = self.client.predict(
                    graspnet_input.color_bgr,
                    graspnet_input.depth_raw,
                    self._camera_intrinsics(),
                    request_id=self._legacy_request_id,
                    snapshot_stamp_sec=float(snapshot.stamp_sec),
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
                try:
                    self._write_planning_audit_failure_report(
                        (),
                        stamp,
                        CANONICAL_CANDIDATE_CAMERA_FRAME,
                        frozen_candidate_pose_estimator,
                        dict(
                            getattr(self.client, 'last_diagnostics', {}) or {}
                        ),
                        failure_code,
                        failure_reason,
                    )
                except CandidateContractError as audit_exc:
                    failure_code = str(audit_exc.code)
                    failure_reason = str(audit_exc)
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

            if bool(graspnet_input.diagnostic_only):
                # full_scene is useful only for controlled WSL diagnostics.
                # It may decode and audit the exact WSL batch, but it must stop
                # before selector/reachability/MoveIt and never publish a plan.
                self._reset_geometry_gate_audit(len(candidates))
                failure_reason = (
                    'full_scene is permanently diagnostic-only; '
                    'WSL candidates=%d and no reachability, MoveIt, MuJoCo, '
                    'or valid plan publication was attempted'
                    % len(candidates)
                )
                try:
                    self._run_candidate_gate_audit(
                        candidates,
                        stamp=stamp,
                        camera_frame=CANONICAL_CANDIDATE_CAMERA_FRAME,
                        remote_diagnostics=remote_diagnostics,
                        candidate_pose_estimator=frozen_candidate_pose_estimator,
                        finalize_report=True,
                        outcome_code=FULL_SCENE_DIAGNOSTIC_CODE,
                        outcome_reason=failure_reason,
                    )
                except Exception as exc:
                    audit_reason = (
                        'full-scene planning audit failed: %s' % exc
                    )
                    self._write_planning_audit_failure_report(
                        candidates,
                        stamp,
                        CANONICAL_CANDIDATE_CAMERA_FRAME,
                        frozen_candidate_pose_estimator,
                        remote_diagnostics,
                        PLANNING_AUDIT_FAILED,
                        audit_reason,
                    )
                    raise CandidateContractError(
                        PLANNING_AUDIT_FAILED,
                        audit_reason,
                    )
                _applied, message = self._invalidate_geometry_if_current(
                    request_invalidation_generation,
                    FULL_SCENE_DIAGNOSTIC_CODE,
                    failure_reason,
                    stamp=stamp,
                    snapshot=snapshot,
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
            try:
                self._run_candidate_gate_audit(
                    candidates,
                    stamp=stamp,
                    camera_frame=CANONICAL_CANDIDATE_CAMERA_FRAME,
                    remote_diagnostics=remote_diagnostics,
                    candidate_pose_estimator=frozen_candidate_pose_estimator,
                    finalize_report=False,
                )
            except Exception as exc:
                if (
                    isinstance(exc, CandidateContractError)
                    and exc.code == PLANNING_AUDIT_WRITE_FAILED
                ):
                    raise
                audit_reason = 'candidate planning audit failed: %s' % exc
                self._write_planning_audit_failure_report(
                    candidates,
                    stamp,
                    CANONICAL_CANDIDATE_CAMERA_FRAME,
                    frozen_candidate_pose_estimator,
                    remote_diagnostics,
                    PLANNING_AUDIT_FAILED,
                    audit_reason,
                )
                raise CandidateContractError(
                    PLANNING_AUDIT_FAILED,
                    audit_reason,
                )

            if not candidates:
                failure_reason = 'remote GraspNet returned no candidates'
                failure_reason += self._candidate_failure_diagnostics(remote_diagnostics)
                self._finalize_gate_audit_report(
                    evaluation_records=(),
                    selected_candidate=None,
                    selected_pose=None,
                    plan_id='',
                    outcome_code='NO_RAW_CANDIDATE',
                    outcome_reason=failure_reason,
                    valid_plan=False,
                )
                _applied, message = self._invalidate_geometry_if_current(
                    request_invalidation_generation,
                    'NO_RAW_CANDIDATE',
                    failure_reason,
                    stamp=stamp,
                    snapshot=snapshot,
                )
                self._publish_error(message)
                return False, message
            planning_evaluation_records = []
            try:
                selected, grasp_pose = select_first_reachable_candidate(
                    candidates,
                    frozen_candidate_pose_estimator,
                    self._plan_reachable,
                    stamp=stamp,
                    camera_frame=CANONICAL_CANDIDATE_CAMERA_FRAME,
                    candidate_frame_convention=self.candidate_frame_convention,
                    candidate_filter_fn=self._candidate_matches_target,
                    candidate_rank_fn=self._candidate_rank,
                    orientation_variant_quaternions=self.orientation_variant_quaternions,
                    model_grasp_to_tool_quaternion=self.model_grasp_to_tool_quaternion,
                    candidate_geometry_fn=self._evaluate_candidate_geometry,
                    grasp_config=self.grasp_config,
                    require_candidate_depth=self.require_candidate_depth,
                    candidate_rejection_fn=self._record_candidate_contract_rejection,
                    evaluation_record_sink=planning_evaluation_records.append,
                )
            except Exception as exc:
                selection_reason = 'candidate selection failed: %s' % exc
                self._finalize_gate_audit_report(
                    evaluation_records=planning_evaluation_records,
                    selected_candidate=None,
                    selected_pose=None,
                    plan_id='',
                    outcome_code='PLAN_FAILED',
                    outcome_reason=selection_reason,
                    valid_plan=False,
                )
                raise
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
                    self._finalize_gate_audit_report(
                        evaluation_records=planning_evaluation_records,
                        selected_candidate=None,
                        selected_pose=None,
                        plan_id='',
                        outcome_code='NO_GEOMETRIC_CANDIDATE',
                        outcome_reason=failure_reason,
                        valid_plan=False,
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
                self._finalize_gate_audit_report(
                    evaluation_records=planning_evaluation_records,
                    selected_candidate=None,
                    selected_pose=None,
                    plan_id='',
                    outcome_code='NO_EXECUTABLE_CANDIDATE',
                    outcome_reason=message,
                    valid_plan=False,
                )
                _applied, message = self._invalidate_geometry_if_current(
                    request_invalidation_generation,
                    'NO_EXECUTABLE_CANDIDATE',
                    message,
                    stamp=stamp,
                    snapshot=snapshot,
                )
                self._publish_error(message)
                return False, message
            final_target_delta = self._candidate_target_distance(
                None,
                selected,
                grasp_pose,
            )
            selected_gate = getattr(selected, '_geometry_gate_result', None)
            if not isinstance(selected_gate, CandidateGateResult) or not selected_gate.ok:
                failure_reason = 'selected candidate has no valid analytical gripper result'
                self._finalize_gate_audit_report(
                    evaluation_records=planning_evaluation_records,
                    selected_candidate=None,
                    selected_pose=None,
                    plan_id='',
                    outcome_code='NO_GEOMETRIC_CANDIDATE',
                    outcome_reason=failure_reason,
                    valid_plan=False,
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
                self._finalize_gate_audit_report(
                    evaluation_records=planning_evaluation_records,
                    selected_candidate=None,
                    selected_pose=None,
                    plan_id='',
                    outcome_code='PLAN_STALE',
                    outcome_reason=(
                        'planning request was invalidated before selected gate commit'
                    ),
                    valid_plan=False,
                )
                _applied, message = self._invalidate_geometry_if_current(
                    request_invalidation_generation,
                    'PLAN_STALE',
                    'planning request was invalidated before selected gate commit',
                    stamp=stamp,
                    snapshot=snapshot,
                )
                self._publish_error(message)
                return False, message
            rich_plan = build_rich_plan(
                selected,
                deepcopy(self.latest_object_geometry),
                deepcopy(self.latest_object_geometry.header),
                str(getattr(self, '_last_model_choice', '') or ''),
            )
            # The final atomic report is a publication prerequisite.  It
            # binds every evaluated lineage row and the selected row to the
            # immutable rich-plan ID before either executable topic is sent.
            self._finalize_gate_audit_report(
                evaluation_records=planning_evaluation_records,
                selected_candidate=selected,
                selected_pose=grasp_pose,
                plan_id=rich_plan.plan_id,
                outcome_code='PLAN_READY',
                outcome_reason=message,
                valid_plan=True,
            )
            try:
                published, failure_code = self._publish_plan_pair_if_current(
                    rich_plan,
                    request_invalidation_generation,
                )
            except Exception as exc:
                publish_reason = 'rich/legacy plan publication failed: %s' % exc
                self._finalize_gate_audit_report(
                    evaluation_records=planning_evaluation_records,
                    selected_candidate=selected,
                    selected_pose=grasp_pose,
                    plan_id=rich_plan.plan_id,
                    outcome_code='PLAN_PUBLISH_FAILED',
                    outcome_reason=publish_reason,
                    valid_plan=False,
                )
                raise
            if not published:
                message = (
                    '%s: planning request was invalidated before publication'
                    % failure_code
                )
                self._finalize_gate_audit_report(
                    evaluation_records=planning_evaluation_records,
                    selected_candidate=selected,
                    selected_pose=grasp_pose,
                    plan_id=rich_plan.plan_id,
                    outcome_code=failure_code,
                    outcome_reason=message,
                    valid_plan=False,
                )
                self._publish_error(message)
                return False, message
            self.last_error = ''
            self._backoff_until = rospy.Time(0)
            return True, message
        except (CandidateContractError, GraspNetInputContextError) as exc:
            failure_code = str(exc.code)
            failure_reason = str(getattr(exc, 'reason', '') or str(exc))
            applied, message = self._invalidate_geometry_if_current(
                request_invalidation_generation,
                failure_code,
                failure_reason,
                stamp=stamp,
                snapshot=snapshot,
            )
            self._publish_error(message)
            if applied and self.failure_backoff_sec > 0.0:
                self._backoff_until = rospy.Time.now() + rospy.Duration(self.failure_backoff_sec)
            return False, message
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
        twin_cfg = rospy.get_param('/mujoco_digital_twin', {})
        gate_audit_enabled = rospy.get_param(
            '/grasp_6d/remote/gate_audit_enabled',
            remote_cfg.get(
                'gate_audit_enabled',
                getattr(self, 'gate_audit_enabled', True),
            ),
        )
        gate_audit_output_path = validate_mandatory_planning_audit(
            gate_audit_enabled,
            rospy.get_param(
                '/grasp_6d/remote/gate_audit_output_path',
                remote_cfg.get(
                    'gate_audit_output_path',
                    getattr(self, 'gate_audit_output_path', ''),
                ),
            ),
        )
        (
            gate_audit_output_path,
            mujoco_audit_output_path,
        ) = validate_distinct_audit_paths(
            gate_audit_output_path,
            rospy.get_param(
                '/mujoco_digital_twin/audit_output_path',
                twin_cfg.get(
                    'audit_output_path',
                    getattr(
                        self,
                        'mujoco_audit_output_path',
                        MUJOCO_AUDIT_DEFAULT_PATH,
                    ),
                ),
            ),
        )
        refreshed_grasp_config = dict(
            rospy.get_param('/grasp', getattr(self, 'grasp_config', {})) or {}
        )
        candidate_frame_convention = validate_production_candidate_frame_convention(
            rospy.get_param(
                '/grasp_6d/remote/candidate_frame_convention',
                remote_cfg.get(
                    'candidate_frame_convention',
                    getattr(
                        self,
                        'candidate_frame_convention',
                        PRODUCTION_CANDIDATE_FRAME_CONVENTION,
                    ),
                ),
            )
        )
        orientation_variant_quaternions = (
            self._parse_production_orientation_variant_quaternions(
                rospy.get_param(
                    '/grasp_6d/remote/orientation_variants_rpy_deg',
                    remote_cfg.get(
                        'orientation_variants_rpy_deg',
                        STRICT_ORIENTATION_VARIANTS_RPY_DEG,
                    ),
                )
            )
        )
        model_grasp_to_tool_quaternion = self._parse_orientation_variant_quaternions(
            [rospy.get_param(
                '/grasp_6d/remote/model_grasp_to_tool_rpy_deg',
                remote_cfg.get('model_grasp_to_tool_rpy_deg', [0.0, 90.0, 0.0]),
            )]
        )[0]
        require_candidate_depth = bool(
            rospy.get_param(
                '/grasp_6d/remote/require_candidate_depth',
                remote_cfg.get(
                    'require_candidate_depth',
                    getattr(self, 'require_candidate_depth', True),
                ),
            )
        )
        model_grasp_to_tool_quaternion = validate_execution_tool0_contract(
            require_candidate_depth,
            model_grasp_to_tool_quaternion,
            refreshed_grasp_config,
        )
        accept_position_only_fallback = bool(
            rospy.get_param(
                '/grasp_6d/remote/accept_position_only_fallback',
                remote_cfg.get('accept_position_only_fallback', False),
            )
        )
        accept_orientation_fallback = bool(
            rospy.get_param(
                '/grasp_6d/remote/accept_orientation_fallback',
                remote_cfg.get('accept_orientation_fallback', False),
            )
        )
        (
            allow_position_only_fallback,
            allow_orientation_fallback,
        ) = validate_production_execution_fallback_contract(
            accept_position_only_fallback,
            accept_orientation_fallback,
        )

        # Publish the refreshed critical contract only after every field has
        # passed.  A bad runtime override cannot leave a partially relaxed
        # production policy behind for the next request.
        self.grasp_config = refreshed_grasp_config
        self.candidate_frame_convention = candidate_frame_convention
        self.orientation_variant_quaternions = orientation_variant_quaternions
        self.model_grasp_to_tool_quaternion = model_grasp_to_tool_quaternion
        self.require_candidate_depth = require_candidate_depth
        self.allow_position_only_fallback = allow_position_only_fallback
        self.allow_orientation_fallback = allow_orientation_fallback
        self.gate_audit_enabled = gate_audit_enabled
        self.gate_audit_output_path = gate_audit_output_path
        self.mujoco_audit_output_path = mujoco_audit_output_path
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
        self.max_candidates = int(
            rospy.get_param('/grasp_6d/remote/max_candidates', remote_cfg.get('max_candidates', self.max_candidates))
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
    def _parse_production_orientation_variant_quaternions(value):
        try:
            variants = RemoteGrasp6DNode._parse_orientation_variant_quaternions(
                value
            )
        except (TypeError, ValueError, OverflowError) as exc:
            raise CandidateContractError(
                'ORIENTATION_VARIANT_CONTRACT_INVALID',
                'orientation_variants_rpy_deg must contain finite RPY triples',
            ) from exc
        return validate_production_orientation_variant_quaternions(variants)

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
        self._candidate_contract_rejection_counts = {}
        self._selected_candidate_gate = None
        self.selected_required_open_width_m = None

    def _record_candidate_contract_rejection(
        self,
        _candidate,
        _variant_index,
        exception,
        _stage,
    ):
        code = str(
            getattr(exception, 'code', 'CANDIDATE_CONTRACT_INVALID')
            or 'CANDIDATE_CONTRACT_INVALID'
        )
        counts = getattr(self, '_candidate_contract_rejection_counts', None)
        if not isinstance(counts, dict):
            counts = {}
            self._candidate_contract_rejection_counts = counts
        counts[code] = int(counts.get(code, 0)) + 1
        if code.startswith('DEPTH_'):
            self._depth_gate_rejected_count = int(
                getattr(self, '_depth_gate_rejected_count', 0)
            ) + 1

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
        contract_rejections = dict(
            getattr(self, '_candidate_contract_rejection_counts', {}) or {}
        )
        contract_text = ','.join(
            '%s=%d' % (code, int(contract_rejections[code]))
            for code in sorted(contract_rejections)
        ) or 'none'
        return '%s rejected=%s contract-rejected=%s' % (
            count_text,
            rejection_text,
            contract_text,
        )

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
        candidate_center_base = self._candidate_center_base_xyz(
            camera_candidate,
            grasp_pose,
        )
        result = evaluate_candidate(
            gripper=self.gripper_geometry,
            # Grasp center drives OBB/jaw-line gates; the four transforms are
            # physical tool0 poses used for static and swept boxes.
            candidate_center_base=candidate_center_base,
            candidate_tool0_base=grasp_transform[:3, 3],
            candidate_depth_m=validate_graspnet_depth_m(
                getattr(camera_candidate, 'depth_m', None),
                required=True,
            ),
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

    def _run_candidate_gate_audit(
        self,
        candidates,
        stamp,
        camera_frame,
        remote_diagnostics=None,
        candidate_pose_estimator=None,
        finalize_report=False,
        outcome_code='',
        outcome_reason='',
        commit_state=True,
        graspnet_input_audit=None,
    ):
        if bool(finalize_report) and not bool(commit_state):
            raise ValueError(
                'finalize_report requires committed audit state'
            )
        rows = []
        pose_estimator = (
            candidate_pose_estimator
            if candidate_pose_estimator is not None
            else self.pose_estimator
        )
        variants = list(getattr(self, 'orientation_variant_quaternions', []) or [])
        if not variants:
            variants = [np.asarray([0.0, 0.0, 0.0, 1.0], dtype=float)]
        for candidate_index, candidate in enumerate(candidates):
            try:
                camera_candidate = convert_candidate_to_camera_link(
                    candidate,
                    self.candidate_frame_convention,
                )
                camera_candidate = align_candidate_to_tool_frame(
                    camera_candidate,
                    self.model_grasp_to_tool_quaternion,
                    require_depth=bool(
                        getattr(self, 'require_candidate_depth', False)
                    ),
                )
            except (TypeError, ValueError, OverflowError) as exc:
                for variant_index in range(len(variants)):
                    rows.append(
                        self._invalid_candidate_gate_audit_row(
                            candidate_index,
                            variant_index,
                            candidate,
                            exc,
                        )
                    )
                continue
            for variant_index, correction in enumerate(variants):
                try:
                    variant_candidate = make_parallel_jaw_variant(
                        camera_candidate,
                        correction,
                    )
                except (TypeError, ValueError, OverflowError) as exc:
                    rows.append(
                        self._invalid_candidate_gate_audit_row(
                            candidate_index,
                            variant_index,
                            camera_candidate,
                            exc,
                        )
                    )
                    continue
                pose, center_base = make_candidate_base_pose_and_center(
                    variant_candidate,
                    pose_estimator,
                    stamp,
                    camera_frame,
                )
                setattr(
                    variant_candidate,
                    '_center_base_xyz',
                    center_base,
                )
                setattr(
                    variant_candidate,
                    '_raw_candidate_index',
                    int(candidate_index),
                )
                setattr(
                    variant_candidate,
                    '_variant_index',
                    int(variant_index),
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
        input_audit = dict(
            getattr(self, '_active_graspnet_input_audit', {}) or {}
            if graspnet_input_audit is None
            else graspnet_input_audit
        )
        summary['graspnet_input'] = input_audit
        audit_metadata_fn = getattr(pose_estimator, 'audit_metadata', None)
        if callable(audit_metadata_fn):
            snapshot_transform_summary = dict(
                audit_metadata_fn(include_matrices=False)
            )
            snapshot_transform_report = dict(
                audit_metadata_fn(include_matrices=True)
            )
        else:
            snapshot_transform_summary = {
                'snapshot_stamp_ns': _stamp_to_nsec(stamp),
                'snapshot_source_frame': str(camera_frame),
                'raw_candidate_convention': normalize_candidate_frame_convention(
                    self.candidate_frame_convention
                ),
                'canonical_candidate_frame': str(camera_frame),
                'transform_sha256': '',
            }
            snapshot_transform_report = dict(snapshot_transform_summary)
        summary['snapshot_transform'] = snapshot_transform_summary
        report = {
            'report_version': 2,
            'stamp_sec': float(stamp.to_sec()) if stamp is not None else 0.0,
            'camera_frame': str(camera_frame),
            'snapshot_transform': snapshot_transform_report,
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
            'graspnet_input': input_audit,
            'summary': summary,
            'rows': rows,
            'selected': None,
            'lineage': [
                {
                    'candidate_index': int(row.get('candidate_index', -1)),
                    'variant_index': int(row.get('variant_index', -1)),
                    'selected': False,
                }
                for row in rows
            ],
            'plan_id': '',
            'outcome': {
                'code': str(outcome_code or ''),
                'reason': str(outcome_reason or ''),
                'valid_plan': False,
            },
        }
        if bool(commit_state):
            self._latest_gate_audit_summary = summary
            self._active_gate_audit_report = report
        summary_text = self._format_gate_audit_summary(summary)
        if bool(finalize_report):
            self._finalize_gate_audit_report(
                evaluation_records=(),
                selected_candidate=None,
                selected_pose=None,
                plan_id='',
                outcome_code=outcome_code,
                outcome_reason=outcome_reason,
                valid_plan=False,
            )
        rospy.logwarn('remote 6D controlled gate audit: %s', summary_text)
        return report

    def _finalize_gate_audit_report(
        self,
        evaluation_records,
        selected_candidate,
        selected_pose,
        plan_id,
        outcome_code,
        outcome_reason,
        valid_plan,
    ):
        report = deepcopy(getattr(self, '_active_gate_audit_report', None))
        if not isinstance(report, dict):
            raise CandidateContractError(
                PLANNING_AUDIT_FAILED,
                'planning audit has no request-local base report',
            )
        rows = list(report.get('rows', []) or [])
        rows_by_lineage = {}
        for row in rows:
            if not isinstance(row, dict):
                raise CandidateContractError(
                    PLANNING_AUDIT_FAILED,
                    'planning audit contains a non-dictionary candidate row',
                )
            key = (
                int(row.get('candidate_index', -1)),
                int(row.get('variant_index', -1)),
            )
            if key in rows_by_lineage:
                raise CandidateContractError(
                    PLANNING_AUDIT_FAILED,
                    'planning audit contains duplicate candidate row %s' % (key,),
                )
            rows_by_lineage[key] = row

        expected_keys = set(rows_by_lineage)
        emitted_keys = set()
        consistency_errors = []
        for evaluation in list(evaluation_records or []):
            if not isinstance(evaluation, dict):
                consistency_errors.append(
                    'selector emitted a non-dictionary audit record'
                )
                continue
            try:
                candidate_index = int(evaluation.get('candidate_index', -1))
                variant_index = int(evaluation.get('variant_index', -1))
            except (TypeError, ValueError, OverflowError):
                consistency_errors.append(
                    'selector emitted non-integral lineage indices'
                )
                continue
            key = (candidate_index, variant_index)
            row = rows_by_lineage.get(key)
            if row is None:
                consistency_errors.append(
                    'selector audit lineage %s has no candidate row' % (key,)
                )
                continue
            if key in emitted_keys:
                consistency_errors.append(
                    'selector emitted duplicate audit lineage %s' % (key,)
                )
                continue
            emitted_keys.add(key)
            row['planning_evaluation'] = deepcopy(evaluation)

        for key, row in rows_by_lineage.items():
            schema_error = planning_evaluation_schema_error(
                row.get('planning_evaluation'),
                key,
            )
            if not schema_error:
                continue
            if bool(valid_plan):
                consistency_errors.append(
                    'selector audit lineage %s is incomplete: %s'
                    % (key, schema_error)
                )
            else:
                row['planning_evaluation'] = failed_planning_evaluation(
                    key[0],
                    key[1],
                    'PLANNING_EVALUATION_MISSING',
                    schema_error,
                )

        if bool(valid_plan) and (
            consistency_errors or emitted_keys != expected_keys
        ):
            missing = sorted(expected_keys - emitted_keys)
            extra = sorted(emitted_keys - expected_keys)
            details = list(consistency_errors)
            if missing:
                details.append('missing selector lineages=%s' % missing)
            if extra:
                details.append('extra selector lineages=%s' % extra)
            raise CandidateContractError(
                PLANNING_AUDIT_FAILED,
                'valid planning audit requires exactly one evaluation for '
                'every candidate row: %s' % '; '.join(details),
            )

        selected_key = None
        if selected_candidate is not None or selected_pose is not None:
            if selected_candidate is None or selected_pose is None:
                raise CandidateContractError(
                    PLANNING_AUDIT_FAILED,
                    'selected audit candidate and pose must be provided together',
                )
            selected_key = (
                int(getattr(selected_candidate, '_raw_candidate_index', -1)),
                int(getattr(selected_candidate, '_variant_index', -1)),
            )
            if selected_key not in rows_by_lineage:
                raise CandidateContractError(
                    PLANNING_AUDIT_FAILED,
                    'selected candidate lineage %s is absent from audit rows'
                    % (selected_key,),
                )

        if bool(valid_plan):
            if selected_key is None:
                raise CandidateContractError(
                    PLANNING_AUDIT_FAILED,
                    'valid planning audit requires one selected candidate lineage',
                )
            selected_evaluation_keys = {
                key
                for key, row in rows_by_lineage.items()
                if row.get('planning_evaluation', {}).get('selected') is True
            }
            if selected_evaluation_keys != {selected_key}:
                raise CandidateContractError(
                    PLANNING_AUDIT_FAILED,
                    'valid planning audit selected evaluation lineages %s '
                    'must exactly match selected candidate lineage %s'
                    % (sorted(selected_evaluation_keys), selected_key),
                )

        if not bool(valid_plan):
            if consistency_errors:
                original_code = str(outcome_code or '')
                original_reason = str(outcome_reason or '')
                outcome_code = PLANNING_AUDIT_FAILED
                outcome_reason = (
                    '%s%sselector audit consistency failure: %s'
                    % (
                        original_reason,
                        '; ' if original_reason else '',
                        '; '.join(consistency_errors),
                    )
                )
                if original_code and original_code != PLANNING_AUDIT_FAILED:
                    outcome_reason += ' (original outcome=%s)' % original_code
                selected_candidate = None
                selected_pose = None
                selected_key = None
                plan_id = ''

        lineage = [
            {
                'candidate_index': int(key[0]),
                'variant_index': int(key[1]),
                'selected': bool(
                    row.get('planning_evaluation', {}).get('selected', False)
                ),
            }
            for key, row in rows_by_lineage.items()
        ]

        selected_row = None
        if selected_key is not None:
            selected_row = rows_by_lineage.get(selected_key)
            # Recompute the selected row after ranking so its analytical
            # result and motion cost match the executable rich plan.
            refreshed = self._candidate_gate_audit_row(
                selected_key[0],
                selected_key[1],
                selected_candidate,
                selected_pose,
            )
            planning_evaluation = selected_row.get('planning_evaluation')
            selected_row.clear()
            selected_row.update(refreshed)
            if planning_evaluation is not None:
                selected_row['planning_evaluation'] = planning_evaluation
            selected_row['selected'] = True

        normalized_plan_id = str(plan_id or '')
        if bool(valid_plan) and not normalized_plan_id:
            raise CandidateContractError(
                PLANNING_AUDIT_FAILED,
                'valid planning audit requires the final plan_id',
            )
        report['rows'] = rows
        report['lineage'] = lineage
        report['selected'] = deepcopy(selected_row)
        report['plan_id'] = normalized_plan_id
        report['outcome'] = {
            'code': str(outcome_code or ''),
            'reason': str(outcome_reason or ''),
            'valid_plan': bool(valid_plan),
        }
        self._active_gate_audit_report = deepcopy(report)
        self._latest_gate_audit_reference = self._write_gate_audit_report(report)
        self._publish_bounded_gate_audit(selected_row)
        return deepcopy(self._latest_gate_audit_reference)

    def _write_planning_audit_failure_report(
        self,
        candidates,
        stamp,
        camera_frame,
        pose_estimator,
        remote_diagnostics,
        failure_code,
        failure_reason,
    ):
        """Best-effort complete atomic evidence for a failed WSL request."""
        variants = list(
            getattr(self, 'orientation_variant_quaternions', []) or []
        )
        if not variants:
            variants = [np.asarray([0.0, 0.0, 0.0, 1.0], dtype=float)]
        rows = []
        for candidate_index, candidate in enumerate(list(candidates or [])):
            try:
                center = self._json_vector(
                    getattr(candidate, 'translation_m', None)
                )
            except Exception:
                center = None
            for variant_index in range(len(variants)):
                rows.append(
                    {
                        'candidate_index': int(candidate_index),
                        'variant_index': int(variant_index),
                        'score': self._finite_json_number(
                            getattr(candidate, 'score', None)
                        ),
                        'width_m': self._finite_json_number(
                            getattr(candidate, 'width_m', None)
                        ),
                        'depth_m': self._finite_json_number(
                            getattr(candidate, 'depth_m', None)
                        ),
                        'center_camera_m': center,
                        'planning_evaluation': failed_planning_evaluation(
                            candidate_index,
                            variant_index,
                            failure_code,
                            failure_reason,
                            stage='planning_audit',
                        ),
                    }
                )
        metadata_fn = getattr(pose_estimator, 'audit_metadata', None)
        if callable(metadata_fn):
            transform_summary = dict(metadata_fn(include_matrices=False))
            transform_report = dict(metadata_fn(include_matrices=True))
        else:
            transform_summary = {
                'snapshot_stamp_ns': _stamp_to_nsec(stamp),
                'snapshot_source_frame': str(camera_frame),
                'transform_sha256': '',
            }
            transform_report = dict(transform_summary)
        summary = {
            'base_candidates': int(len(list(candidates or []))),
            'graspnet_input': dict(
                getattr(self, '_active_graspnet_input_audit', {}) or {}
            ),
            'snapshot_transform': transform_summary,
            'audit_error': str(failure_reason),
        }
        self._latest_gate_audit_summary = summary
        self._active_gate_audit_report = {
            'report_version': 2,
            'stamp_sec': float(stamp.to_sec()) if stamp is not None else 0.0,
            'camera_frame': str(camera_frame),
            'snapshot_transform': transform_report,
            'remote_diagnostics': dict(remote_diagnostics or {}),
            'graspnet_input': dict(
                getattr(self, '_active_graspnet_input_audit', {}) or {}
            ),
            'summary': summary,
            'rows': rows,
            'selected': None,
            'lineage': [
                {
                    'candidate_index': int(row['candidate_index']),
                    'variant_index': int(row['variant_index']),
                    'selected': False,
                }
                for row in rows
            ],
            'plan_id': '',
            'outcome': {
                'code': str(failure_code),
                'reason': str(failure_reason),
                'valid_plan': False,
            },
        }
        return self._finalize_gate_audit_report(
            evaluation_records=(),
            selected_candidate=None,
            selected_pose=None,
            plan_id='',
            outcome_code=failure_code,
            outcome_reason=failure_reason,
            valid_plan=False,
        )

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
        center_base = self._candidate_center_base_xyz(candidate, grasp_pose)
        tool0_base = pose_matrix(grasp_pose)[:3, 3]
        center_camera = _finite_vector3(
            candidate.translation_m,
            'candidate center',
        )
        tool0_camera = candidate_tool0_translation(candidate)
        camera_offset = tool0_camera - center_camera
        base_offset = tool0_base - center_base
        camera_rotation = quaternion_matrix(candidate.quaternion_xyzw)[:3, :3]
        base_rotation = pose_matrix(grasp_pose)[:3, :3]
        camera_contract_residual = float(
            np.linalg.norm(camera_offset - depth * camera_rotation[:, 2])
        )
        base_contract_residual = float(
            np.linalg.norm(base_offset - depth * base_rotation[:, 2])
        )

        target_xyz, target_source = self._target_base_xyz()
        target_ok = target_xyz is not None
        center_distance = float('inf')
        relative_z = float('nan')
        if target_xyz is not None:
            delta = center_base - np.asarray(target_xyz, dtype=float)
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
            'center_camera_m': self._json_vector(center_camera),
            'tool0_camera_m': self._json_vector(tool0_camera),
            'center_base_m': self._json_vector(center_base),
            'tool0_base_m': self._json_vector(tool0_base),
            'tool0_offset_camera_m': self._json_vector(camera_offset),
            'tool0_offset_base_m': self._json_vector(base_offset),
            'depth_offset_m': float(np.linalg.norm(camera_offset)),
            'tool0_contract_residual_camera_m': camera_contract_residual,
            'tool0_contract_residual_base_m': base_contract_residual,
            # Backward-compatible alias; this remains the model center.
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

    def _invalid_candidate_gate_audit_row(
        self,
        candidate_index,
        variant_index,
        candidate,
        exception,
    ):
        try:
            depth = float(getattr(candidate, 'depth_m', float('nan')))
        except (TypeError, ValueError, OverflowError):
            depth = float('nan')
        try:
            center = self._json_vector(candidate.translation_m)
        except Exception:
            center = None
        return {
            'candidate_index': int(candidate_index),
            'variant_index': int(variant_index),
            'score': float(getattr(candidate, 'score', 0.0) or 0.0),
            'width_m': float(getattr(candidate, 'width_m', 0.0) or 0.0),
            'depth_m': depth if np.isfinite(depth) else None,
            'center_camera_m': center,
            'tool0_camera_m': None,
            'center_base_m': None,
            'tool0_base_m': None,
            'tool0_offset_camera_m': None,
            'tool0_offset_base_m': None,
            'depth_offset_m': None,
            'tool0_contract_residual_camera_m': None,
            'tool0_contract_residual_base_m': None,
            'translation_camera_m': center,
            'quaternion_camera_xyzw': None,
            'depth_ok': False,
            'width_ok': True,
            'target_ok': False,
            'target_source': 'none',
            'cloud_distance_m': None,
            'center_distance_m': None,
            'relative_z_m': None,
            'center_clearance_m': None,
            'finger_clearance_m': None,
            'jaw_normal_cos': None,
            'approach_cos': None,
            'visibility_ok': False,
            'visibility_reason': '%s: %s'
            % (
                str(
                    getattr(
                        exception,
                        'code',
                        'CANDIDATE_CONTRACT_INVALID',
                    )
                ),
                exception,
            ),
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

    @staticmethod
    def _finite_json_number(value):
        try:
            number = float(value)
        except (TypeError, ValueError, OverflowError):
            return None
        return number if math.isfinite(number) else None

    def _write_gate_audit_report(
        self,
        report,
        ticket=None,
        commit_callback=None,
        replace_reference_sha256=None,
    ):
        path, mujoco_path = validate_distinct_audit_paths(
            validate_mandatory_planning_audit(
                getattr(self, 'gate_audit_enabled', True),
                getattr(self, 'gate_audit_output_path', ''),
            ),
            getattr(
                self,
                'mujoco_audit_output_path',
                MUJOCO_AUDIT_DEFAULT_PATH,
            ),
        )
        self.gate_audit_output_path = path
        self.mujoco_audit_output_path = mujoco_path
        temporary_path = '%s.tmp-%d-%d' % (
            path,
            int(os.getpid()),
            int(threading.get_ident()),
        )
        try:
            directory = os.path.dirname(path)
            if directory:
                os.makedirs(directory, exist_ok=True)
            payload = json.dumps(
                report,
                allow_nan=False,
                ensure_ascii=True,
                indent=2,
                sort_keys=True,
            ).encode('utf-8')
            with open(temporary_path, 'wb') as handle:
                handle.write(payload)
                handle.flush()
                os.fsync(handle.fileno())
            reference = {
                'report_path': path,
                'report_sha256': hashlib.sha256(payload).hexdigest(),
                'row_count': int(
                    len(list(report.get('rows', []) or []))
                ),
                'atomic_committed': True,
            }
            replacement_sha256 = str(replace_reference_sha256 or '')
            if replacement_sha256:
                with self._stream_condition:
                    current_sha256 = str(
                        dict(
                            getattr(
                                self,
                                '_latest_gate_audit_reference',
                                {},
                            )
                            or {}
                        ).get('report_sha256', '')
                        or ''
                    )
                    if current_sha256 != replacement_sha256:
                        raise StreamResultCancelled('AUDIT_SUPERSEDED')
                    os.replace(temporary_path, path)
                    if commit_callback is not None:
                        commit_callback(reference)
            elif ticket is None:
                os.replace(temporary_path, path)
                if commit_callback is not None:
                    commit_callback(reference)
            else:
                with self._stream_condition:
                    self._require_stream_ticket_current_locked(ticket)
                    os.replace(temporary_path, path)
                    if commit_callback is not None:
                        commit_callback(reference)
            return reference
        except StreamResultCancelled:
            try:
                if os.path.exists(temporary_path):
                    os.unlink(temporary_path)
            except Exception:
                pass
            raise
        except Exception as exc:
            try:
                if os.path.exists(temporary_path):
                    os.unlink(temporary_path)
            except Exception:
                pass
            raise CandidateContractError(
                PLANNING_AUDIT_WRITE_FAILED,
                'could not atomically write planning audit %s: %s'
                % (path, exc),
            )

    def _publish_bounded_gate_audit(
        self,
        selected_row=None,
        summary=None,
        reference=None,
    ):
        reference = dict(
            getattr(self, '_latest_gate_audit_reference', {}) or {}
            if reference is None
            else reference
        )
        payload = {
            'summary': dict(
                getattr(self, '_latest_gate_audit_summary', {}) or {}
                if summary is None
                else summary
            ),
            'report_path': str(reference.get('report_path', '') or ''),
            'report_sha256': str(reference.get('report_sha256', '') or ''),
            'row_count': int(reference.get('row_count', 0) or 0),
            'selected': None,
        }
        if isinstance(selected_row, dict):
            selected_keys = (
                'candidate_index',
                'variant_index',
                'score',
                'depth_m',
                'center_camera_m',
                'tool0_camera_m',
                'center_base_m',
                'tool0_base_m',
                'tool0_offset_camera_m',
                'tool0_offset_base_m',
                'tool0_contract_residual_camera_m',
                'tool0_contract_residual_base_m',
            )
            payload['selected'] = {
                key: selected_row.get(key)
                for key in selected_keys
            }
        publisher = getattr(self, 'gate_audit_pub', None)
        if publisher is not None:
            try:
                publisher.publish(
                    String(
                        json.dumps(
                            payload,
                            allow_nan=False,
                            ensure_ascii=True,
                            sort_keys=True,
                        )
                    )
                )
            except Exception as exc:
                raise CandidateContractError(
                    PLANNING_AUDIT_FAILED,
                    'could not publish bounded planning audit reference: %s'
                    % exc,
                )

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
            center_base = self._candidate_center_base_xyz(
                _camera_candidate,
                grasp_pose,
            )
            dx = float(center_base[0]) - float(target_xyz[0])
            dy = float(center_base[1]) - float(target_xyz[1])
            dz = float(center_base[2]) - float(target_xyz[2])
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
                float(center_base[0]),
                float(center_base[1]),
                float(center_base[2]),
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
    def _candidate_center_base_xyz(camera_candidate, tool0_pose):
        """Return the immutable GraspNet center, never the offset tool0 origin."""
        center = getattr(camera_candidate, '_center_base_xyz', None)
        if center is not None:
            return _finite_vector3(center, 'candidate center in base')
        if camera_candidate is None:
            raise CandidateContractError(
                'CENTER_TRANSFORM_MISSING',
                'candidate is required to recover center from tool0 pose',
            )
        return candidate_center_base_from_tool0_pose(
            camera_candidate,
            tool0_pose,
        )

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
                tool_approach_axis=str(self.grasp_config.get('tool_approach_axis', 'z')),
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

    def _configured_tool_from_camera_matrix(self):
        try:
            translation = _finite_vector3(
                self.handeye_translation_xyz,
                'configured handeye translation',
            )
            quaternion = _normalize_quaternion(
                np.asarray(self.handeye_rotation_xyzw, dtype=float)
            )
            return _readonly_rigid_transform(
                transform_matrix(translation, quaternion),
                'configured tool_from_camera',
            )
        except Exception as exc:
            raise CandidateContractError(
                'VISIBILITY_TRANSFORM_INVALID',
                'configured handeye transform is not a finite rigid transform: %s'
                % exc,
            )

    def _freeze_tool_from_camera_matrix(self, snapshot_stamp):
        try:
            stamp_ns = _stamp_to_nsec(snapshot_stamp)
        except (TypeError, ValueError, OverflowError) as exc:
            raise CandidateContractError(
                'VISIBILITY_TRANSFORM_INVALID',
                'visibility snapshot stamp is invalid: %s' % exc,
            )
        if stamp_ns <= 0:
            raise CandidateContractError(
                'VISIBILITY_TRANSFORM_INVALID',
                'visibility TF requires a strictly positive snapshot stamp',
            )
        cached = getattr(self, '_cached_tool_from_camera', None)
        if (
            cached is not None
            and int(getattr(self, '_cached_tool_from_camera_stamp_ns', 0))
            == int(stamp_ns)
        ):
            return cached

        matrix = None
        lookup_error = None
        tf_buffer = getattr(self, 'tf_buffer', None)
        if tf_buffer is not None:
            try:
                transform = tf_buffer.lookup_transform(
                    self.handeye_parent_frame,
                    self.handeye_camera_frame,
                    snapshot_stamp,
                    rospy.Duration(
                        float(
                            getattr(
                                getattr(self, 'pose_estimator', None),
                                'tf_timeout_sec',
                                0.2,
                            )
                        )
                    ),
                )
            except Exception as exc:
                lookup_error = exc
            else:
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                try:
                    matrix = _readonly_rigid_transform(
                        transform_matrix(
                            [translation.x, translation.y, translation.z],
                            _normalize_quaternion(
                                np.asarray(
                                    [
                                        rotation.x,
                                        rotation.y,
                                        rotation.z,
                                        rotation.w,
                                    ],
                                    dtype=float,
                                )
                            ),
                        ),
                        'snapshot tool_from_camera',
                    )
                except Exception as exc:
                    raise CandidateContractError(
                        'VISIBILITY_TRANSFORM_INVALID',
                        'snapshot handeye TF is not a finite rigid transform: %s'
                        % exc,
                    )

        if matrix is None:
            allow_fallback = bool(
                getattr(
                    self,
                    'handeye_allow_static_fallback',
                    getattr(
                        getattr(self, 'pose_estimator', None),
                        'allow_static_fallback',
                        True,
                    ),
                )
            )
            if tf_buffer is not None and not allow_fallback:
                raise CandidateContractError(
                    'VISIBILITY_TRANSFORM_UNAVAILABLE',
                    'exact snapshot handeye TF is unavailable: %s'
                    % lookup_error,
                )
            if lookup_error is not None:
                rospy.logwarn_throttle(
                    2.0,
                    (
                        'snapshot eye-in-hand TF %s <- %s unavailable; '
                        'freezing calibrated fallback: %s'
                    ),
                    self.handeye_parent_frame,
                    self.handeye_camera_frame,
                    lookup_error,
                )
            matrix = self._configured_tool_from_camera_matrix()

        self._cached_tool_from_camera = matrix
        self._cached_tool_from_camera_stamp_ns = int(stamp_ns)
        return matrix

    def _tool_from_camera_matrix(self):
        """Return a frozen visibility transform without querying live TF."""
        cached = getattr(self, '_cached_tool_from_camera', None)
        if cached is not None:
            return cached
        if bool(getattr(self, '_planning_snapshot_active', False)):
            raise CandidateContractError(
                'VISIBILITY_TRANSFORM_NOT_FROZEN',
                'planning visibility transform was not frozen before WSL inference',
            )
        # Non-planning diagnostics may use the configured calibration, but it
        # is still copied, rigid-validated, and made immutable.  This method
        # intentionally contains no TF lookup and can never request latest.
        matrix = self._configured_tool_from_camera_matrix()
        self._cached_tool_from_camera = matrix
        self._cached_tool_from_camera_stamp_ns = 0
        return matrix

    def _candidate_target_distance(self, _candidate, _camera_candidate, grasp_pose):
        target_xyz, _target_source = self._target_base_xyz()
        if target_xyz is None:
            return float('inf')
        try:
            center_base = self._candidate_center_base_xyz(
                _camera_candidate,
                grasp_pose,
            )
            dx = float(center_base[0]) - float(target_xyz[0])
            dy = float(center_base[1]) - float(target_xyz[1])
            dz = float(center_base[2]) - float(target_xyz[2])
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

    def _strict_moveit_evaluation(self, grasp_pose):
        """Run strict MoveIt without committing request-shared diagnostics."""

        try:
            # Production remote 6D reachability is always a read-only strict
            # check.  Configuration is validated earlier, but this hard-coded
            # endpoint prevents a partially constructed/test node from ever
            # turning candidate screening into a motion-capable service call.
            service_name = '/supervisor/check_pose_strict'
            try:
                rospy.wait_for_service(service_name, timeout=0.25)
            except Exception as exc:
                return (
                    MoveItResult(
                        reachable=False,
                        joint_path_cost=0.0,
                        joint_max_delta_rad=0.0,
                        reason=str(exc),
                        failure_code='MOVEIT_TIMEOUT',
                    ),
                    None,
                    '',
                )
            move_pose = rospy.ServiceProxy(service_name, SetTargetPose)
            response = move_pose(grasp_pose, False)
            message = str(getattr(response, 'message', '') or '')
            metrics = self._parse_plan_metrics(message)
            path_cost = max(
                0.0,
                _finite_pipeline_number(
                    metrics.get('joint_path_cost', 0.0)
                ),
            )
            max_delta = max(
                0.0,
                _finite_pipeline_number(
                    metrics.get('joint_max_delta', 0.0)
                ),
            )
            success = getattr(response, 'success', None)
            if type(success) is not bool:
                return (
                    MoveItResult(
                        reachable=False,
                        joint_path_cost=path_cost,
                        joint_max_delta_rad=max_delta,
                        reason=(
                            'strict MoveIt response has invalid success state'
                        ),
                        failure_code='MOVEIT_CHECK_ERROR',
                    ),
                    metrics,
                    '',
                )
            if success and is_position_only_fallback_message(message):
                return (
                    MoveItResult(
                        reachable=False,
                        joint_path_cost=path_cost,
                        joint_max_delta_rad=max_delta,
                        reason=message,
                        failure_code='MOVEIT_UNREACHABLE',
                    ),
                    metrics,
                    'position_only',
                )
            if success and is_orientation_fallback_message(message):
                return (
                    MoveItResult(
                        reachable=False,
                        joint_path_cost=path_cost,
                        joint_max_delta_rad=max_delta,
                        reason=message,
                        failure_code='MOVEIT_UNREACHABLE',
                    ),
                    metrics,
                    'orientation_fallback',
                )

            hard_state_names = (
                'collision_free',
                'within_joint_limits',
                'ik_valid',
                'planning_success',
            )
            hard_states = tuple(
                getattr(response, name, None) for name in hard_state_names
            )
            if all(type(state) is bool for state in hard_states):
                return (
                    MoveItResult(
                        reachable=bool(success and all(hard_states)),
                        joint_path_cost=path_cost,
                        joint_max_delta_rad=max_delta,
                        reason=message,
                        collision_free=hard_states[0],
                        within_joint_limits=hard_states[1],
                        ik_valid=hard_states[2],
                        planning_success=hard_states[3],
                    ),
                    metrics,
                    '',
                )
            if not all(state is None for state in hard_states):
                return (
                    MoveItResult(
                        reachable=False,
                        joint_path_cost=path_cost,
                        joint_max_delta_rad=max_delta,
                        reason=(
                            'strict MoveIt response has partial hard-state '
                            'evidence'
                        ),
                        failure_code='MOVEIT_CHECK_ERROR',
                    ),
                    metrics,
                    '',
                )
            if success:
                return (
                    MoveItResult(
                        reachable=True,
                        joint_path_cost=path_cost,
                        joint_max_delta_rad=max_delta,
                        reason=message,
                        evidence_code='STRICT_SERVICE_SUCCESS',
                    ),
                    metrics,
                    '',
                )
            explicit_code = str(
                getattr(response, 'failure_code', '') or ''
            )
            if explicit_code not in {
                'MOVEIT_UNREACHABLE',
                'MOVEIT_CHECK_ERROR',
                'MOVEIT_TIMEOUT',
            }:
                explicit_code = 'MOVEIT_UNREACHABLE'
            return (
                MoveItResult(
                    reachable=False,
                    joint_path_cost=path_cost,
                    joint_max_delta_rad=max_delta,
                    reason=message,
                    failure_code=explicit_code,
                ),
                metrics,
                '',
            )
        except TimeoutError as exc:
            return (
                MoveItResult(
                    reachable=False,
                    joint_path_cost=0.0,
                    joint_max_delta_rad=0.0,
                    reason=str(exc),
                    failure_code='MOVEIT_TIMEOUT',
                ),
                None,
                '',
            )
        except Exception as exc:
            return (
                MoveItResult(
                    reachable=False,
                    joint_path_cost=0.0,
                    joint_max_delta_rad=0.0,
                    reason=str(exc),
                    failure_code='MOVEIT_CHECK_ERROR',
                ),
                None,
                '',
            )

    def _commit_strict_moveit_evaluation(self, grasp_pose, evaluation):
        result, metrics, fallback_code = evaluation
        if metrics is not None:
            if not hasattr(self, '_candidate_plan_metrics'):
                self._candidate_plan_metrics = {}
            self._candidate_plan_metrics[self._pose_key(grasp_pose)] = dict(
                metrics
            )
        if fallback_code == 'position_only':
            self._position_only_rejected_count += 1
            rospy.logwarn_throttle(
                2.0,
                'remote 6D candidate rejected: position-only fallback is '
                'not executable: %s',
                result.reason,
            )
        elif fallback_code == 'orientation_fallback':
            self._orientation_fallback_rejected_count += 1
            rospy.logwarn_throttle(
                2.0,
                'remote 6D candidate rejected: candidate orientation '
                'fallback is not executable: %s',
                result.reason,
            )
        return result

    def _strict_moveit_result(self, grasp_pose):
        """Compatibility wrapper that commits one synchronous evaluation."""

        return self._commit_strict_moveit_evaluation(
            grasp_pose,
            self._strict_moveit_evaluation(grasp_pose),
        )

    def _plan_reachable(self, grasp_pose):
        return bool(self._strict_moveit_result(grasp_pose).reachable)

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
