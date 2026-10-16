"""Pure geometry for the measured endpoint immediately before closing.

The target contact band belongs to the frozen, validated plan.  It must never
be recomputed from a newer target or replaced with the height of its OBB.  The
caller owns plan/model identity, feedback freshness and motion authorization.
Passing this check establishes modelled overlap, not physical contact or an
independent camera-to-robot calibration measurement.
"""

from dataclasses import dataclass
import math

import numpy as np

from .gripper_geometry import (
    ANALYTICAL_GRIPPER_MODEL_NAME,
    ANALYTICAL_MAX_INNER_GAP_M,
    GripperGeometry,
    _box_corners,
    _stage_boxes,
    _validated_transform,
    finger_contact_patch_overlap_m,
    gripper_contract_mismatch_reason,
)


@dataclass(frozen=True)
class MeasuredContactResult:
    ok: bool
    code: str
    reason: str
    planned_overlap_m: object = None
    actual_overlap_m: object = None
    minimum_contact_overlap_m: object = None
    minimum_support_clearance_m: object = None
    support_clearances_m: tuple = ()


def _vector(value, size, name):
    values = np.asarray(value, dtype=float)
    if values.shape != (size,) or not np.all(np.isfinite(values)):
        raise ValueError('%s must contain %d finite coordinates' % (name, size))
    return values


def _unit(value, name):
    values = _vector(value, 3, name)
    if not math.isclose(float(np.linalg.norm(values)), 1.0,
                        rel_tol=0.0, abs_tol=1e-6):
        raise ValueError('%s must be unit length' % name)
    return values


def evaluate_measured_contact(
    *,
    gripper,
    actual_T_base_tool0,
    planned_T_base_tool0,
    contact_center_base,
    contact_height_axis_base,
    contact_height_bounds_m,
    minimum_contact_overlap_m,
    support_normal_base,
    support_offset_m,
    opening_width_m,
):
    """Compare a measured tool pose with the unchanged planned contact band.

    Bounds are relative to ``contact_center_base`` along the frozen plan's
    tool +Z axis, as returned by the measured bilateral-surface evaluator.
    They are not bounds relative to the moving tool or support-plane heights.
    All three physical CAD boxes retain the existing support-clearance gate.
    This check does not authorize a path, a lower target, or a gripper command.
    """
    try:
        if not isinstance(gripper, GripperGeometry):
            raise ValueError('gripper must be a GripperGeometry')
        mismatch = gripper_contract_mismatch_reason(
            gripper, ANALYTICAL_MAX_INNER_GAP_M, ANALYTICAL_MAX_INNER_GAP_M,
            ANALYTICAL_GRIPPER_MODEL_NAME, ANALYTICAL_MAX_INNER_GAP_M,
        )
        if mismatch:
            raise ValueError(mismatch)
        actual = _validated_transform(actual_T_base_tool0, 'actual_T_base_tool0')
        planned = _validated_transform(planned_T_base_tool0, 'planned_T_base_tool0')
        center = _vector(contact_center_base, 3, 'contact_center_base')
        axis = _unit(contact_height_axis_base, 'contact_height_axis_base')
        if not np.allclose(axis, planned[:3, 2], rtol=0.0, atol=1e-7):
            raise ValueError('contact height axis must equal frozen planned tool +Z')
        bounds = _vector(contact_height_bounds_m, 2, 'contact_height_bounds_m')
        if bounds[1] <= bounds[0]:
            raise ValueError('contact height bounds must have positive extent')
        minimum = float(minimum_contact_overlap_m)
        if not math.isfinite(minimum) or minimum <= 0.0:
            raise ValueError('minimum contact overlap must be finite and positive')
        normal = _unit(support_normal_base, 'support_normal_base')
        offset = float(support_offset_m)
        opening = float(opening_width_m)
        if not math.isfinite(offset) or not math.isfinite(opening):
            raise ValueError('support offset and measured opening must be finite')
        if not 0.0 <= opening <= min(gripper.max_inner_gap_m,
                                     ANALYTICAL_MAX_INNER_GAP_M):
            raise ValueError('measured opening is outside the physical gripper range')
        planned_overlap = finger_contact_patch_overlap_m(
            center, planned[:3, 3], planned[:3, :3], axis, *bounds)
        actual_overlap = finger_contact_patch_overlap_m(
            center, actual[:3, 3], actual[:3, :3], axis, *bounds)
        clearances = tuple(
            (name, float(np.min(_box_corners(box_center, actual[:3, :3], size)
                                @ normal + offset)))
            for name, box_center, size in _stage_boxes(
                actual, gripper, opening, 'y', 'z')
        )
        support_clearance = min(value for _name, value in clearances)
        if not all(math.isfinite(value) for value in
                   (planned_overlap, actual_overlap, support_clearance)):
            raise ValueError('derived measured contact metrics are not finite')
    except (TypeError, ValueError, OverflowError) as exc:
        return MeasuredContactResult(False, 'MEASURED_CONTACT_INPUT_INVALID', str(exc))

    metrics = dict(
        planned_overlap_m=float(planned_overlap),
        actual_overlap_m=float(actual_overlap),
        minimum_contact_overlap_m=minimum,
        minimum_support_clearance_m=support_clearance,
        support_clearances_m=clearances,
    )
    if planned_overlap + 1e-9 < minimum:
        return MeasuredContactResult(
            False, 'FROZEN_CONTACT_EVIDENCE_INVALID',
            'frozen planned contact overlap %.6fm is below its %.6fm requirement'
            % (planned_overlap, minimum), **metrics)
    if support_clearance < gripper.support_clearance_m - 1e-9:
        return MeasuredContactResult(
            False, 'MEASURED_CONTACT_SUPPORT_CLEARANCE',
            'measured CAD support clearance %.6fm is below unchanged %.6fm requirement'
            % (support_clearance, gripper.support_clearance_m), **metrics)
    if actual_overlap + 1e-9 < minimum:
        return MeasuredContactResult(
            False, 'MEASURED_CONTACT_PATCH_MISS',
            'measured bilateral target/CAD contact overlap %.6fm is below frozen %.6fm requirement'
            % (actual_overlap, minimum), **metrics)
    return MeasuredContactResult(
        True, 'MEASURED_CONTACT_GEOMETRY_OK',
        'measured modelled contact overlap and support clearance passed; physical grip unverified',
        **metrics)
