"""Versioned measured contact geometry frozen into an executable rich plan.

Bounds are signed projections relative to contact_center_base along the fixed
contact_height_axis_base. They are measured bilateral surface bounds, never
OBB dimensions. The support plane remains in the signed ObjectGeometry.
"""
import json
import math
from numbers import Real

import numpy as np
from tf.transformations import quaternion_matrix

from alicia_flexible_grasp.grasp.gripper_geometry import ANALYTICAL_GRIPPER_MODEL_NAME

_FIELDS = frozenset((
    'schema_version', 'contact_center_base', 'contact_height_axis_base',
    'contact_height_bounds_m', 'minimum_contact_overlap_m',
    'grasp_tool_pose_base', 'measurement_source', 'gripper_model',
    'tool_jaw_axis', 'tool_finger_length_axis',
))


def _vector(values, size, label):
    try:
        if (not isinstance(values, (list, tuple, np.ndarray)) or len(values) != size
                or any(isinstance(v, (bool, np.bool_)) or not isinstance(v, Real)
                       or not math.isfinite(float(v)) for v in values)):
            raise ValueError('invalid numeric vector')
        return [float(v) for v in values]
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError('%s must have %d finite numbers' % (label, size)) from exc


def _pose_values(pose):
    pose = getattr(pose, 'pose', pose)
    try:
        return [float(pose.position.x), float(pose.position.y), float(pose.position.z),
                float(pose.orientation.x), float(pose.orientation.y),
                float(pose.orientation.z), float(pose.orientation.w)]
    except (AttributeError, TypeError, ValueError, OverflowError) as exc:
        raise ValueError('contact grasp pose is unavailable') from exc


def _validated(evidence):
    if not isinstance(evidence, dict) or set(evidence) != _FIELDS:
        raise ValueError('contact geometry evidence fields do not match schema 1')
    if type(evidence['schema_version']) is not int or evidence['schema_version'] != 1:
        raise ValueError('unsupported contact geometry evidence schema')
    for key, expected in (
        ('measurement_source', 'fused_measured_bilateral_surface'),
        ('gripper_model', ANALYTICAL_GRIPPER_MODEL_NAME),
        ('tool_jaw_axis', '+Y'), ('tool_finger_length_axis', '+Z'),
    ):
        if evidence[key] != expected:
            raise ValueError('contact geometry %s mismatch' % key)
    result = dict(evidence)
    for name, size in (('contact_center_base', 3), ('contact_height_axis_base', 3),
                       ('contact_height_bounds_m', 2), ('grasp_tool_pose_base', 7)):
        result[name] = _vector(evidence[name], size, name)
    axis = np.asarray(result['contact_height_axis_base'])
    if abs(float(np.linalg.norm(axis)) - 1.) > 1e-9:
        raise ValueError('contact height axis must be unit length')
    pose = result['grasp_tool_pose_base']
    if abs(float(np.linalg.norm(pose[3:])) - 1.) > 1e-6:
        raise ValueError('contact grasp orientation must be a unit quaternion')
    if not np.allclose(axis, quaternion_matrix(pose[3:])[:3, 2], atol=1e-9, rtol=0.):
        raise ValueError('contact height axis differs from final grasp +Z')
    lower, upper = result['contact_height_bounds_m']
    if lower >= upper:
        raise ValueError('measured contact height bounds must have positive extent')
    overlap = _vector([evidence['minimum_contact_overlap_m']], 1,
                      'minimum_contact_overlap_m')[0]
    if overlap <= 0.:
        raise ValueError('minimum contact overlap must be finite and positive')
    result['minimum_contact_overlap_m'] = overlap
    return result


def encode_contact_evidence(*, contact_center_base, contact_height_axis_base,
                            contact_height_bounds_m, minimum_contact_overlap_m,
                            grasp_pose):
    evidence = _validated(dict(
        schema_version=1,
        contact_center_base=contact_center_base,
        contact_height_axis_base=contact_height_axis_base,
        contact_height_bounds_m=contact_height_bounds_m,
        minimum_contact_overlap_m=minimum_contact_overlap_m,
        grasp_tool_pose_base=_pose_values(grasp_pose),
        measurement_source='fused_measured_bilateral_surface',
        gripper_model=ANALYTICAL_GRIPPER_MODEL_NAME,
        tool_jaw_axis='+Y', tool_finger_length_axis='+Z',
    ))
    return json.dumps(evidence, sort_keys=True, separators=(',', ':'), allow_nan=False)


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('duplicate contact geometry evidence field')
        result[key] = value
    return result


def decode_contact_evidence(plan, *, required=None):
    """Return validated frozen measurements, or None for an optional empty plan.

    Execution callers should pass required=True for unknown CONTACT execution.
    Plan hashing deliberately remains compatible with old non-contact plans.
    """
    if required is None:
        required = (str(getattr(plan, 'model_choice', '')) == 'unknown_tabletop'
                    and str(getattr(plan, 'diagnostic', '')) == 'CONTACT_EXECUTION_PLAN')
    encoded = getattr(plan, 'contact_geometry_evidence', '')
    if not isinstance(encoded, str):
        raise ValueError('contact geometry evidence must be a JSON string')
    if not encoded:
        if required:
            raise ValueError('measured contact geometry evidence is missing')
        return None
    if len(encoded) > 16384:
        raise ValueError('contact geometry evidence is oversized')
    try:
        parsed = json.loads(encoded, object_pairs_hook=_unique_object)
    except (TypeError, ValueError) as exc:
        raise ValueError('contact geometry evidence is malformed JSON') from exc
    result = _validated(parsed)
    canonical = json.dumps(result, sort_keys=True, separators=(',', ':'), allow_nan=False)
    if encoded != canonical:
        raise ValueError('contact geometry evidence is not canonical JSON')
    poses = list(getattr(plan, 'poses', ()) or ())
    if len(poses) != 4:
        raise ValueError('contact geometry evidence requires four final poses')
    final_pose = _pose_values(poses[2])
    if (not all(math.isfinite(v) for v in final_pose)
            or not np.allclose(final_pose, result['grasp_tool_pose_base'], atol=1e-12, rtol=0.)):
        raise ValueError('contact geometry evidence does not match the final grasp pose')
    return result
