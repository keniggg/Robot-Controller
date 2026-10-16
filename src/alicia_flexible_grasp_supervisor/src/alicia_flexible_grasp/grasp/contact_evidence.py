"""Versioned contact geometry frozen into an executable rich plan.

Bounds are signed projections relative to contact_center_base along the fixed
contact_height_axis_base. They are measured bilateral surface bounds, never
OBB dimensions for schemas 1/2. Schema 3 explicitly represents a qualified
regular-box inference and retains its measured top/support provenance. It
never labels inferred sides as measured. The signed plan binds both kinds.
"""
import json
import math
from numbers import Real

import numpy as np
from tf.transformations import quaternion_matrix

from alicia_flexible_grasp.grasp.gripper_geometry import ANALYTICAL_GRIPPER_MODEL_NAME
from alicia_flexible_grasp.grasp.rich_plan_integrity import float32_wire_value

_FIELDS = frozenset((
    'schema_version', 'contact_center_base', 'contact_height_axis_base',
    'contact_height_bounds_m', 'minimum_contact_overlap_m',
    'grasp_tool_pose_base', 'measurement_source', 'gripper_model',
    'tool_jaw_axis', 'tool_finger_length_axis',
))
_OPENING_FIELDS = frozenset((
    'measured_contact_width_m', 'opening_fit_clearance_each_side_m',
    'required_open_width_m',
))
_INFERRED_FIELDS = frozenset((
    'inferred_contact_width_m', 'width_uncertainty_m',
    'opening_fit_clearance_each_side_m', 'required_open_width_m',
    'box_model_evidence',
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
    if not isinstance(evidence, dict):
        raise ValueError('contact geometry evidence must be an object')
    version = evidence.get('schema_version')
    if type(version) is not int or version not in (1, 2, 3):
        raise ValueError('unsupported contact geometry evidence schema')
    expected = _FIELDS | ({1: frozenset(), 2: _OPENING_FIELDS,
                           3: _INFERRED_FIELDS}[version])
    if set(evidence) != expected:
        raise ValueError('contact geometry evidence fields do not match schema %d' % version)
    for key, expected in (
        ('measurement_source', ('regular_box_model_inference' if version == 3
                                else 'fused_measured_bilateral_surface')),
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
        raise ValueError('contact height bounds must have positive extent')
    overlap = _vector([evidence['minimum_contact_overlap_m']], 1,
                      'minimum_contact_overlap_m')[0]
    if overlap <= 0.:
        raise ValueError('minimum contact overlap must be finite and positive')
    result['minimum_contact_overlap_m'] = overlap
    if version == 2:
        for name in _OPENING_FIELDS:
            result[name] = _vector([evidence[name]], 1, name)[0]
        width = result['measured_contact_width_m']
        clearance = result['opening_fit_clearance_each_side_m']
        opened = result['required_open_width_m']
        if (width <= 0. or clearance < 0. or not 0. < opened <= .05
                or abs(opened - (width + 2. * clearance)) > 1e-12):
            raise ValueError('contact opening must equal measured width plus its actual fit clearance')
    if version == 3:
        from alicia_flexible_grasp.vision.regular_box_inference import validate_box_model_evidence
        result['box_model_evidence'] = validate_box_model_evidence(evidence['box_model_evidence'])
        for name in _INFERRED_FIELDS - {'box_model_evidence'}:
            result[name] = _vector([evidence[name]], 1, name)[0]
        width = result['inferred_contact_width_m']
        uncertainty = result['width_uncertainty_m']
        clearance = result['opening_fit_clearance_each_side_m']
        opened = result['required_open_width_m']
        if (width <= 0. or uncertainty < 0. or clearance < 0.
                or not 0. < opened <= .05
                or abs(opened - (width + uncertainty + 2. * clearance)) > 1e-12):
            raise ValueError('inferred opening must include the complete width uncertainty and fit clearance')
        _validate_inferred_projection(result)
    return result


def _validate_inferred_projection(evidence):
    """Recompute contact geometry, rather than trusting caller-written bounds."""
    from alicia_flexible_grasp.vision.regular_box_inference import RegularBoxInference
    from .gripper_geometry import (
        GripperGeometry, evaluate_inferred_box_contact,
        ANALYTICAL_MAX_INNER_GAP_M, ANALYTICAL_JAW_CLEARANCE_EACH_SIDE_M,
        ANALYTICAL_FINGER_SIZE_XYZ_M, ANALYTICAL_PALM_SIZE_XYZ_M,
        ANALYTICAL_SUPPORT_CLEARANCE_M,
    )
    model = RegularBoxInference.from_evidence_dict(evidence['box_model_evidence'])
    rotation = quaternion_matrix(evidence['grasp_tool_pose_base'][3:])[:3, :3]
    gripper = GripperGeometry(ANALYTICAL_MAX_INNER_GAP_M,
        ANALYTICAL_JAW_CLEARANCE_EACH_SIDE_M, ANALYTICAL_FINGER_SIZE_XYZ_M,
        ANALYTICAL_PALM_SIZE_XYZ_M, ANALYTICAL_SUPPORT_CLEARANCE_M)
    projected = evaluate_inferred_box_contact(model, evidence['contact_center_base'],
        rotation[:, 1], rotation[:, 2], gripper,
        opening_fit_clearance_each_side_m=evidence['opening_fit_clearance_each_side_m'])
    if not projected.ok:
        raise ValueError('inferred contact projection requires additional evidence: ' + projected.code)
    for name, actual in (
            ('inferred_contact_width_m', projected.inferred_width_m),
            ('width_uncertainty_m', projected.width_uncertainty_m),
            ('contact_height_bounds_m', projected.contact_height_bounds_m),
            ('contact_height_axis_base', projected.contact_height_axis_base)):
        if not np.allclose(evidence[name], actual, rtol=0., atol=1e-9):
            raise ValueError('inferred contact projection mismatch: ' + name)


def _validate_inferred_plan_source(plan, evidence):
    """Bind the source plane and corrected object to the unchanged plan scene."""
    from .rich_plan_integrity import stamp_nanoseconds
    from alicia_flexible_grasp.vision.regular_box_inference import (
        RegularBoxInference, inferred_box_enclosure_vertices,
    )
    model = evidence['box_model_evidence']
    source = model['source']
    geometry = getattr(plan, 'object_geometry', None)
    if (source['stamp_ns'] != stamp_nanoseconds(
            getattr(getattr(plan, 'header', None), 'stamp', None))
            or source['target_identity']['track_id'] != getattr(plan, 'target_track_id', None)
            or source['target_identity']['track_id'] != getattr(geometry, 'target_track_id', None)):
        raise ValueError('inferred box source does not match the final plan')
    try:
        normal = geometry.support_normal_base
        normal = [normal.x, normal.y, normal.z]
        offset = float32_wire_value(geometry.support_offset_m)
    except (AttributeError, TypeError, ValueError, OverflowError) as exc:
        raise ValueError('inferred box plan support is unavailable') from exc
    # The object and model receive the accepted registration correction, but
    # physical support clearance always uses the current source measurement.
    # Model validation independently proves its registered plane is exactly
    # the transform of this retained source plane; do not conflate the two.
    if (not np.allclose(normal, model['source_support_normal_base'], atol=1e-8, rtol=0.)
            or offset != float32_wire_value(model['source_support_offset_m'])):
        raise ValueError('inferred box source support differs from the final plan')
    try:
        pose = _vector(_pose_values(geometry.pose_base), 7, 'planning object pose')
        size = _vector([geometry.size_xyz_m.x, geometry.size_xyz_m.y,
                        geometry.size_xyz_m.z], 3, 'planning object size')
    except (AttributeError, TypeError, ValueError, OverflowError) as exc:
        raise ValueError('inferred box planning enclosure is unavailable') from exc
    if (not getattr(geometry, 'valid', False) or min(size) <= 0.
            or abs(float(np.linalg.norm(pose[3:])) - 1.) > 1e-6):
        raise ValueError('inferred box planning enclosure is invalid')
    vertices = inferred_box_enclosure_vertices(RegularBoxInference.from_evidence_dict(model))
    local = (vertices - np.asarray(pose[:3])) @ quaternion_matrix(pose[3:])[:3, :3]
    # Pose/Vector3 use float64 on the wire. This is numerical tolerance only;
    # the complete empirical shape budget belongs inside collision geometry.
    if np.any(np.abs(local) > np.asarray(size) / 2. + 1e-8):
        raise ValueError('planning enclosure excludes inferred box uncertainty')


def encode_contact_evidence(*, contact_center_base, contact_height_axis_base,
                            contact_height_bounds_m, minimum_contact_overlap_m,
                            grasp_pose, measured_contact_width_m=None,
                            opening_fit_clearance_each_side_m=None,
                            required_open_width_m=None):
    opening = dict(measured_contact_width_m=measured_contact_width_m,
        opening_fit_clearance_each_side_m=opening_fit_clearance_each_side_m,
        required_open_width_m=required_open_width_m)
    supplied = [value is not None for value in opening.values()]
    if any(supplied) and not all(supplied):
        raise ValueError('complete measured opening provenance is required')
    evidence = dict(
        schema_version=1,
        contact_center_base=contact_center_base,
        contact_height_axis_base=contact_height_axis_base,
        contact_height_bounds_m=contact_height_bounds_m,
        minimum_contact_overlap_m=minimum_contact_overlap_m,
        grasp_tool_pose_base=_pose_values(grasp_pose),
        measurement_source='fused_measured_bilateral_surface',
        gripper_model=ANALYTICAL_GRIPPER_MODEL_NAME,
        tool_jaw_axis='+Y', tool_finger_length_axis='+Z',
    )
    if all(supplied):
        evidence.update(opening, schema_version=2)
    evidence = _validated(evidence)
    return json.dumps(evidence, sort_keys=True, separators=(',', ':'), allow_nan=False)


def encode_inferred_contact_evidence(*, contact_center_base, contact_height_axis_base,
                                    contact_height_bounds_m, minimum_contact_overlap_m,
                                    grasp_pose, inferred_contact_width_m,
                                    width_uncertainty_m, opening_fit_clearance_each_side_m,
                                    required_open_width_m, box_model_evidence):
    """Freeze a model estimate without inventing measured bilateral samples.

    ``width_uncertainty_m`` is the total added width allowance, including
    orientation projection; it is not a per-side allowance or grip proof.
    """
    evidence = dict(schema_version=3,
        contact_center_base=contact_center_base,
        contact_height_axis_base=contact_height_axis_base,
        contact_height_bounds_m=contact_height_bounds_m,
        minimum_contact_overlap_m=minimum_contact_overlap_m,
        grasp_tool_pose_base=_pose_values(grasp_pose),
        measurement_source='regular_box_model_inference',
        gripper_model=ANALYTICAL_GRIPPER_MODEL_NAME,
        tool_jaw_axis='+Y', tool_finger_length_axis='+Z',
        inferred_contact_width_m=inferred_contact_width_m,
        width_uncertainty_m=width_uncertainty_m,
        opening_fit_clearance_each_side_m=opening_fit_clearance_each_side_m,
        required_open_width_m=required_open_width_m,
        box_model_evidence=box_model_evidence)
    return json.dumps(_validated(evidence), sort_keys=True, separators=(',', ':'), allow_nan=False)


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('duplicate contact geometry evidence field')
        result[key] = value
    return result


def decode_contact_evidence(plan, *, required=None):
    """Return explicitly typed frozen geometry, or None for an optional plan.

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
    if result['schema_version'] in (2, 3):
        opened = _vector([getattr(plan, 'required_open_width_m', None)], 1,
                         'plan required opening')[0]
        try:
            matches = (float32_wire_value(opened)
                       == float32_wire_value(result['required_open_width_m']))
        except (ValueError, OverflowError):
            matches = False
        if not matches:
            raise ValueError('contact opening provenance does not match the final plan opening')
    if result['schema_version'] == 3:
        _validate_inferred_plan_source(plan, result)
    return result
