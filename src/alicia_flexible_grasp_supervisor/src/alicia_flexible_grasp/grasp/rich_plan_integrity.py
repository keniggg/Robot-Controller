"""Canonical integrity and snapshot-binding checks for rich 6D plans."""

import hashlib
import hmac
import math
import re
import struct


_CANONICAL_PREFIX = b'ALICIA_GRASP6D_PLAN_V1\x00'
_PLAN_ID_PATTERN = re.compile(r'^[0-9a-f]{24}$')
SUPPORTED_GEOMETRY_SOURCE_MODES = frozenset(('instance_mask', 'bbox_depth'))
GRIPPER_MAX_OPEN_WIDTH_M = 0.050


def float32_wire_value(value):
    """Return the value a ROS float32 field carries across the wire."""
    return struct.unpack('>f', struct.pack('>f', float(value)))[0]


GRIPPER_MAX_OPEN_WIDTH_F32 = float32_wire_value(GRIPPER_MAX_OPEN_WIDTH_M)


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
    label = str(getattr(geometry, 'label', '') or '')
    if not label or label != label.strip():
        raise ValueError('object geometry label must be non-empty and canonical')
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


def canonical_plan_bytes(plan):
    """Return Task 9 canonical bytes; header binding is validated separately."""
    stamp_ns = stamp_nanoseconds(
        getattr(getattr(plan, 'header', None), 'stamp', None)
    )
    if stamp_ns <= 0:
        raise ValueError('snapshot header stamp must be non-zero')
    model_text = str(getattr(plan, 'model_choice', '') or '')
    if not model_text or model_text != model_text.strip():
        raise ValueError('model choice must be non-empty and canonical')
    model = model_text.encode('utf-8')
    poses = list(getattr(plan, 'poses', ()) or ())
    if len(poses) != 4:
        raise ValueError('rich plan must contain exactly four poses')

    payload = bytearray(_CANONICAL_PREFIX)
    payload.extend(struct.pack('>qI', stamp_ns, len(model)))
    payload.extend(model)
    for pose in poses:
        payload.extend(
            struct.pack('>7d', *validate_finite_pose(pose, 'plan pose'))
        )
    try:
        candidate_width = float(plan.candidate_width_m)
        required_width = float(plan.required_open_width_m)
    except Exception as exc:
        raise ValueError('plan width fields are unavailable') from exc
    if not math.isfinite(candidate_width) or not math.isfinite(required_width):
        raise ValueError('plan width fields must be finite')
    # These two message fields are ROS float32 values. Pack at wire precision
    # so the producer and a subscriber recompute the same digest.
    payload.extend(struct.pack('>2f', candidate_width, required_width))
    geometry_pose, geometry_size, support = validate_rich_geometry(
        getattr(plan, 'object_geometry', None)
    )
    payload.extend(struct.pack('>7d', *geometry_pose))
    payload.extend(struct.pack('>3d', *geometry_size))
    payload.extend(struct.pack('>3d', *support[:3]))
    # ObjectGeometry.support_offset_m is also float32 on the ROS wire.
    payload.extend(struct.pack('>f', support[3]))
    return bytes(payload)


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
