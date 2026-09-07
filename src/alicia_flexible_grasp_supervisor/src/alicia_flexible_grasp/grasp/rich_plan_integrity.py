"""Canonical integrity and snapshot-binding checks for rich 6D plans."""

import hashlib
import hmac
import math
import re
import struct
from collections.abc import Mapping
from numbers import Integral, Real

from alicia_flexible_grasp.vision.target_observation import validate_track_id
from alicia_flexible_grasp.vision.multiview_surface import RegistrationConfig


_CANONICAL_PREFIX = b'ALICIA_GRASP6D_PLAN_V2\x00'
_PLAN_ID_PATTERN = re.compile(r'^[0-9a-f]{24}$')
SUPPORTED_GEOMETRY_SOURCE_MODES = frozenset(('instance_mask', 'bbox_depth'))
SUPPORTED_CANDIDATE_SOURCES = frozenset(('graspnet', 'tabletop_geometry'))
GRIPPER_MAX_OPEN_WIDTH_M = 0.050


def float32_wire_value(value):
    """Return the value a ROS float32 field carries across the wire."""
    return struct.unpack('>f', struct.pack('>f', float(value)))[0]


GRIPPER_MAX_OPEN_WIDTH_F32 = float32_wire_value(GRIPPER_MAX_OPEN_WIDTH_M)

# Keep the task-side refinement contract aligned with the measured multiview
# registration contract.  ``minimum_fused_view_count`` is plan provenance
# rather than an ICP setting, so it is kept here as the one additional field.
REFINEMENT_REGISTRATION_FIELDS = (
    'minimum_inliers',
    'minimum_fused_view_count',
    'minimum_overlap_fraction',
    'maximum_rmse_m',
    'maximum_translation_m',
    'maximum_yaw_deg',
    'maximum_support_normal_angle_deg',
    'maximum_support_offset_delta_m',
)
_REFINEMENT_DEFAULTS = {
    'minimum_inliers': int(RegistrationConfig().minimum_inliers),
    'minimum_fused_view_count': 2,
    'minimum_overlap_fraction': float(RegistrationConfig().minimum_overlap_fraction),
    'maximum_rmse_m': float(RegistrationConfig().maximum_rmse_m),
    'maximum_translation_m': float(RegistrationConfig().maximum_translation_m),
    'maximum_yaw_deg': float(RegistrationConfig().maximum_yaw_deg),
    'maximum_support_normal_angle_deg': float(
        RegistrationConfig().maximum_support_normal_angle_deg
    ),
    'maximum_support_offset_delta_m': float(
        RegistrationConfig().maximum_support_offset_delta_m
    ),
}


def refinement_registration_policy(config=None):
    """Return a validated refinement policy; runtime values may only tighten.

    The returned dictionary intentionally contains no ``maximum_iterations`` or
    correspondence setting: those are producer-side registration controls and
    are not evidence acceptance claims carried by ``Grasp6DPlan``.
    """
    if config is None:
        raw = {}
    elif isinstance(config, RegistrationConfig):
        raw = {
            name: getattr(config, name)
            for name in REFINEMENT_REGISTRATION_FIELDS
            if hasattr(config, name)
        }
    elif isinstance(config, Mapping):
        raw = dict(config)
    else:
        raise ValueError('refinement registration policy must be a mapping')
    unknown = set(raw).difference(REFINEMENT_REGISTRATION_FIELDS)
    if unknown:
        raise ValueError(
            'refinement registration policy has unknown fields: %s'
            % ', '.join(sorted(str(name) for name in unknown))
        )
    policy = dict(_REFINEMENT_DEFAULTS)
    policy.update(raw)
    integer_fields = ('minimum_inliers', 'minimum_fused_view_count')
    for name in integer_fields:
        value = policy[name]
        if isinstance(value, bool) or not isinstance(value, Integral):
            raise ValueError('%s must be an integer' % name)
        if int(value) < 1:
            raise ValueError('%s must be >= 1' % name)
        policy[name] = int(value)
    for name in set(REFINEMENT_REGISTRATION_FIELDS).difference(integer_fields):
        value = policy[name]
        if isinstance(value, bool) or not isinstance(value, Real):
            raise ValueError('%s must be a finite number' % name)
        value = float(value)
        if not math.isfinite(value):
            raise ValueError('%s must be finite' % name)
        if name == 'minimum_overlap_fraction' and not 0.0 < value <= 1.0:
            raise ValueError('%s must be in (0, 1]' % name)
        if name == 'maximum_yaw_deg' and not 0.0 < value <= 180.0:
            raise ValueError('%s must be in (0, 180]' % name)
        if name == 'maximum_support_normal_angle_deg' and not 0.0 <= value <= 180.0:
            raise ValueError('%s must be in [0, 180]' % name)
        if name not in ('minimum_overlap_fraction', 'maximum_yaw_deg',
                        'maximum_support_normal_angle_deg') and value < 0.0:
            raise ValueError('%s must be nonnegative' % name)
        policy[name] = value
    for name in ('minimum_inliers', 'minimum_fused_view_count'):
        if policy[name] < _REFINEMENT_DEFAULTS[name]:
            raise ValueError('%s may only be tightened' % name)
    for name in (
        'minimum_overlap_fraction',
    ):
        if policy[name] < _REFINEMENT_DEFAULTS[name]:
            raise ValueError('%s may only be tightened' % name)
    for name in (
        'maximum_rmse_m', 'maximum_translation_m', 'maximum_yaw_deg',
        'maximum_support_normal_angle_deg',
        'maximum_support_offset_delta_m',
    ):
        if policy[name] > _REFINEMENT_DEFAULTS[name]:
            raise ValueError('%s may only be tightened' % name)
    return policy


def required_open_width_is_valid(value):
    """Validate the physical opening after ROS float32 quantization."""
    try:
        wire_value = float32_wire_value(value)
    except (TypeError, ValueError, OverflowError, struct.error):
        return False
    return (
        math.isfinite(wire_value)
        and wire_value > 0.0
        and wire_value <= GRIPPER_MAX_OPEN_WIDTH_F32
    )


def stamp_nanoseconds(stamp):
    if stamp is None:
        return 0
    if hasattr(stamp, 'to_nsec'):
        return int(stamp.to_nsec())
    if hasattr(stamp, 'seconds'):
        return int(round(float(stamp.seconds) * 1_000_000_000.0))
    return (
        int(getattr(stamp, 'secs', 0)) * 1_000_000_000
        + int(getattr(stamp, 'nsecs', 0))
    )


def stamp_seconds(stamp):
    return float(stamp_nanoseconds(stamp)) * 1e-9


def pose_values(pose):
    try:
        return (
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        )
    except Exception as exc:
        raise ValueError('pose fields are unavailable') from exc


def validate_finite_pose(pose, name='pose'):
    values = pose_values(pose)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('%s contains non-finite values' % name)
    if sum(value * value for value in values[3:]) <= 1e-24:
        raise ValueError('%s quaternion has zero norm' % name)
    return values


def validate_rich_geometry(geometry):
    if geometry is None or not bool(getattr(geometry, 'valid', False)):
        raise ValueError('object geometry is invalid')
    validate_track_id(getattr(geometry, 'target_track_id', None))
    source_mode = str(getattr(geometry, 'source_mode', '') or '')
    if source_mode not in SUPPORTED_GEOMETRY_SOURCE_MODES:
        raise ValueError(
            'object geometry source_mode must be instance_mask or bbox_depth'
        )
    pose = validate_finite_pose(geometry.pose_base, 'object geometry pose')
    try:
        size = (
            float(geometry.size_xyz_m.x),
            float(geometry.size_xyz_m.y),
            float(geometry.size_xyz_m.z),
        )
        support = (
            float(geometry.support_normal_base.x),
            float(geometry.support_normal_base.y),
            float(geometry.support_normal_base.z),
            float(geometry.support_offset_m),
        )
    except Exception as exc:
        raise ValueError('object geometry fields are unavailable') from exc
    if not all(math.isfinite(value) and value > 0.0 for value in size):
        raise ValueError('object geometry size must be finite and positive')
    if not all(math.isfinite(value) for value in support):
        raise ValueError('support plane contains non-finite values')
    if sum(value * value for value in support[:3]) <= 1e-24:
        raise ValueError('support plane normal has zero norm')
    return pose, size, support


def validate_plan_header_binding(plan, base_frame='base_link'):
    header = getattr(plan, 'header', None)
    geometry = getattr(plan, 'object_geometry', None)
    geometry_header = getattr(geometry, 'header', None)
    plan_frame = str(getattr(header, 'frame_id', '') or '')
    geometry_frame = str(getattr(geometry_header, 'frame_id', '') or '')
    if plan_frame != str(base_frame):
        raise ValueError('plan header frame_id must be base_link')
    if geometry_frame != str(base_frame):
        raise ValueError('object geometry header frame_id must be base_link')
    plan_stamp_ns = stamp_nanoseconds(getattr(header, 'stamp', None))
    geometry_stamp_ns = stamp_nanoseconds(
        getattr(geometry_header, 'stamp', None)
    )
    if plan_stamp_ns <= 0:
        raise ValueError('plan header stamp must be non-zero')
    if geometry_stamp_ns != plan_stamp_ns:
        raise ValueError(
            'object geometry header stamp must exactly match plan header stamp'
        )
    return plan_stamp_ns


def validate_candidate_source(source, lineage):
    canonical = str(source or '')
    items = tuple(str(item or '') for item in lineage or ())
    if canonical not in SUPPORTED_CANDIDATE_SOURCES:
        raise ValueError('candidate source is unsupported')
    if not items or tuple(sorted(set(items))) != items:
        raise ValueError('candidate source lineage must be sorted and unique')
    if canonical not in items or any(
        item not in SUPPORTED_CANDIDATE_SOURCES for item in items
    ):
        raise ValueError('candidate source lineage is inconsistent')
    return canonical, items


def validate_candidate_model_width(plan):
    source, _lineage = validate_candidate_source(
        plan.candidate_source,
        plan.candidate_source_lineage,
    )
    present = bool(plan.has_candidate_model_width)
    width = float(plan.candidate_width_m)
    if not math.isfinite(width) or width < 0.0:
        raise ValueError('candidate model width is invalid')
    if source == 'graspnet' and (not present or width <= 0.0):
        raise ValueError('GraspNet plan requires its model width')
    if source == 'tabletop_geometry' and (present or width != 0.0):
        raise ValueError('geometry plan must encode absent model width')
    return None if not present else width


def canonical_plan_bytes(plan):
    """Return Task 9 canonical bytes; header binding is validated separately."""
    stamp_ns = stamp_nanoseconds(
        getattr(getattr(plan, 'header', None), 'stamp', None)
    )
    if stamp_ns <= 0:
        raise ValueError('snapshot header stamp must be non-zero')
    model_text = str(getattr(plan, 'model_choice', '') or '')
    model = model_text.encode('utf-8')
    source_text, lineage = validate_candidate_source(
        getattr(plan, 'candidate_source', None),
        getattr(plan, 'candidate_source_lineage', None),
    )
    source = source_text.encode('utf-8')
    lineage_bytes = tuple(item.encode('utf-8') for item in lineage)
    candidate_width = validate_candidate_model_width(plan)
    candidate_width_value = float(plan.candidate_width_m)
    poses = list(getattr(plan, 'poses', ()) or ())
    if len(poses) != 4:
        raise ValueError('rich plan must contain exactly four poses')

    payload = bytearray(_CANONICAL_PREFIX)
    track = validate_track_id(getattr(plan, 'target_track_id', None))
    if track != getattr(getattr(plan, 'object_geometry', None), 'target_track_id', None):
        raise ValueError('plan and geometry target track must match')
    track_bytes = track.encode('utf-8')
    payload.extend(struct.pack('>I', len(track_bytes)))
    payload.extend(track_bytes)
    status, counts, metrics, clipped = validate_refinement_evidence(plan)
    status_bytes = status.encode('ascii')
    payload.extend(struct.pack('>I', len(status_bytes)))
    payload.extend(status_bytes)
    payload.extend(struct.pack('>II4f?', *counts, *metrics, clipped))
    payload.extend(struct.pack('>qI', stamp_ns, len(model)))
    payload.extend(model)
    payload.extend(struct.pack('>II', len(source), len(lineage_bytes)))
    payload.extend(source)
    for item in lineage_bytes:
        payload.extend(struct.pack('>I', len(item)))
        payload.extend(item)
    payload.extend(
        struct.pack(
            '>?f',
            candidate_width is not None,
            candidate_width_value,
        )
    )
    for pose in poses:
        payload.extend(
            struct.pack('>7d', *validate_finite_pose(pose, 'plan pose'))
        )
    try:
        required_width = float(plan.required_open_width_m)
    except Exception as exc:
        raise ValueError('required plan width field is unavailable') from exc
    if not math.isfinite(required_width):
        raise ValueError('required plan width field must be finite')
    # Message width fields are ROS float32 values. Pack at wire precision so
    # the producer and a subscriber recompute the same digest.
    payload.extend(struct.pack('>f', required_width))
    geometry_pose, geometry_size, support = validate_rich_geometry(
        getattr(plan, 'object_geometry', None)
    )
    payload.extend(struct.pack('>7d', *geometry_pose))
    payload.extend(struct.pack('>3d', *geometry_size))
    payload.extend(struct.pack('>3d', *support[:3]))
    # ObjectGeometry.support_offset_m is also float32 on the ROS wire.
    payload.extend(struct.pack('>f', support[3]))
    return bytes(payload)


def validate_refinement_evidence(plan, registration_config=None):
    """Validate structured evidence at ROS wire precision, never infer success.

    ``registration_config`` is an optional task/runtime policy.  It is parsed
    through :func:`refinement_registration_policy`, which rejects malformed or
    widened values and keeps the wire-boundary comparisons deterministic.
    """
    policy = refinement_registration_policy(registration_config)
    status = getattr(plan, 'refinement_status', None)
    if status not in ('NOT_EVALUATED', 'VALID_3D', 'CLEAR_VIEW_REQUIRED', 'INVALID_3D'):
        raise ValueError('unsupported refinement status')
    counts = tuple(getattr(plan, name, None) for name in (
        'refinement_inlier_count', 'fused_view_count'))
    if any(isinstance(value, bool) or not isinstance(value, Integral)
           or not 0 <= value <= 0xffffffff for value in counts):
        raise ValueError('refinement counts must be uint32 values')
    try:
        metrics = tuple(float32_wire_value(getattr(plan, name)) for name in (
            'refinement_overlap_fraction', 'refinement_rmse_m',
            'refinement_translation_m', 'refinement_rotation_deg'))
    except (AttributeError, TypeError, ValueError, OverflowError, struct.error) as exc:
        raise ValueError('refinement metrics must be finite float32 values') from exc
    if any(not math.isfinite(value) or value < 0 for value in metrics):
        raise ValueError('refinement metrics must be finite and nonnegative')
    if metrics[0] > 1 or metrics[3] > 180:
        raise ValueError('refinement overlap or rotation outside physical bounds')
    clipped = getattr(plan, 'refinement_source_clipped', None)
    if not isinstance(clipped, bool):
        raise ValueError('refinement_source_clipped must be bool')
    if status == 'VALID_3D':
        # Acceptance is deliberately strict at the float32 wire boundary.  A
        # forged VALID_3D marker with weak registration evidence must fail
        # closed before it can become execution authority.
        inliers, views = counts
        overlap, rmse, translation, rotation = metrics
        if (
            inliers < policy['minimum_inliers']
            or views < policy['minimum_fused_view_count']
            or overlap < float32_wire_value(policy['minimum_overlap_fraction'])
        ):
            raise ValueError(
                'VALID_3D registration counts/overlap below configured thresholds'
            )
        if rmse > float32_wire_value(policy['maximum_rmse_m']):
            raise ValueError('VALID_3D registration RMSE exceeds threshold')
        if translation > float32_wire_value(policy['maximum_translation_m']):
            raise ValueError('VALID_3D translation exceeds threshold')
        if rotation > float32_wire_value(policy['maximum_yaw_deg']):
            raise ValueError('VALID_3D rotation exceeds threshold')
    if status == 'CLEAR_VIEW_REQUIRED':
        # A clear view is an observation-quality remedy only.  It must not be
        # used for malformed, stale, mismatched, or over-bound registrations.
        if not clipped:
            raise ValueError('CLEAR_VIEW_REQUIRED requires a clipped source')
        if not (
            counts[0] < policy['minimum_inliers']
            or counts[1] < policy['minimum_fused_view_count']
            or metrics[0] < float32_wire_value(policy['minimum_overlap_fraction'])
        ):
            raise ValueError(
                'CLEAR_VIEW_REQUIRED is only valid for insufficient evidence'
            )
    if status == 'NOT_EVALUATED' and (any(counts) or any(metrics) or clipped):
        raise ValueError('NOT_EVALUATED must not assert refinement evidence')
    return status, counts, metrics, clipped


def compute_plan_id(plan):
    return hashlib.sha256(canonical_plan_bytes(plan)).hexdigest()[:24]


def strict_plan_id_equal(first, second):
    if not isinstance(first, str) or not isinstance(second, str):
        return False
    if (
        _PLAN_ID_PATTERN.fullmatch(first) is None
        or _PLAN_ID_PATTERN.fullmatch(second) is None
    ):
        return False
    return hmac.compare_digest(first, second)


def plan_id_matches_content(plan):
    claimed = getattr(plan, 'plan_id', None)
    if not isinstance(claimed, str) or _PLAN_ID_PATTERN.fullmatch(claimed) is None:
        return False
    try:
        expected = compute_plan_id(plan)
    except Exception:
        return False
    return strict_plan_id_equal(claimed, expected)
