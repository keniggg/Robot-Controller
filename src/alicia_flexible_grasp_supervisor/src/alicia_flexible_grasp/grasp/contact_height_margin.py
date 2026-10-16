"""Continuous one-dimensional contact sensitivity, never motion authority.

An empirical calibration error is an evidence limit, not a certified bound on
future error. This necessary condition covers translations along the support
normal only; it proves neither full 3D contact nor physical grip. The existing
centering override is intentionally not read or changed here.
"""
from dataclasses import asdict
import math

import numpy as np
from .contact_execution import frozen_contact_geometry
from .gripper_geometry import (
    ANALYTICAL_FINGER_CONTACT_PATCH_TOOL_XZ_M as PATCH,
    ANALYTICAL_FINGER_PAIR_CENTER_TOOL_XYZ_M as PAIR_CENTER,
    _signed_polygon_margin,
)
from .measured_contact import evaluate_measured_contact


_EPS = 1e-12


def _merge(intervals):
    result = []
    for lower, upper in sorted(intervals):
        if upper < lower - _EPS:
            continue
        if result and lower <= result[-1][1] + _EPS:
            result[-1] = (result[-1][0], max(result[-1][1], upper))
        else:
            result.append((lower, upper))
    return result


def _line_intervals(point, direction, polygon=PATCH):
    """Exact polygon-edge events for a line, including the concave pad notch."""
    squared = float(direction @ direction)
    if squared <= 1e-24:
        return [(-math.inf, math.inf)] if _signed_polygon_margin(point, polygon) >= 0 else []
    projected = (polygon-point) @ direction / squared
    events = [float(min(projected)), float(max(projected))]
    for start, end in zip(polygon, np.roll(polygon, -1, axis=0)):
        edge, offset = end-start, start-point
        denominator = float(direction[0]*edge[1]-direction[1]*edge[0])
        if abs(denominator) > 1e-14:
            value = float((offset[0]*edge[1]-offset[1]*edge[0])/denominator)
            fraction = float((offset[0]*direction[1]-offset[1]*direction[0])/denominator)
            if -_EPS <= fraction <= 1.+_EPS:
                events.append(value)
        elif abs(float(offset[0]*direction[1]-offset[1]*direction[0])) <= 1e-12:
            events.extend((float((start-point) @ direction/squared),
                           float((end-point) @ direction/squared)))
    ordered = sorted(set(events))
    intervals = [(a, b) for a, b in zip(ordered, ordered[1:])
                 if b-a > _EPS and _signed_polygon_margin(point+direction*((a+b)/2), polygon) >= -1e-12]
    return _merge(intervals)


def _clip_linear(lower, upper, slope, constant):
    """Intersect with slope * delta + constant <= 0."""
    if abs(slope) <= 1e-14:
        return (lower, upper) if constant <= _EPS else (1., 0.)
    boundary = -constant/slope
    return (lower, min(upper, boundary)) if slope > 0 else (max(lower, boundary), upper)


def _contact_translation_intervals(point, height_direction, normal_direction,
                                   band_lower, band_upper, minimum):
    """Analytic feasible deltas for a finite height segment in a simple polygon.

    In each delta slab between polygon vertices, every connected height
    intersection is bounded by two affine functions. Three linear inequalities
    require its intersection with the frozen height band to be >= minimum.
    This treats concavity exactly; convexifying the finger pad would fill its
    notch and incorrectly create contact surface.
    """
    if band_upper-band_lower < minimum-_EPS:
        return []
    a, b = height_direction, normal_direction
    a_squared = float(a @ a)
    if a_squared <= 1e-24:
        return _line_intervals(point, -b)
    determinant = float(a[0]*b[1]-a[1]*b[0])
    if abs(determinant) <= 1e-12*max(float(np.linalg.norm(a)*np.linalg.norm(b)), 1e-12):
        coefficient = float(a @ b/a_squared)
        if np.linalg.norm(b-coefficient*a) > 1e-12:
            raise ValueError('normal/height projection is ill-conditioned')
        intervals = []
        for lo, hi in _line_intervals(point, a):
            if hi-lo < minimum-_EPS:
                continue
            if abs(coefficient) <= 1e-14:
                if min(hi, band_upper)-max(lo, band_lower) >= minimum-_EPS:
                    return [(-math.inf, math.inf)]
            else:
                ends = ((band_lower-hi+minimum)/coefficient,
                        (band_upper-lo-minimum)/coefficient)
                intervals.append((min(ends), max(ends)))
        return _merge(intervals)
    projection = np.column_stack((a, -b))
    if np.linalg.cond(projection) > 1e10:
        raise ValueError('normal/height projection is ill-conditioned')
    coordinates = np.linalg.solve(projection, (PATCH-point).T).T
    events = sorted(set(float(value) for value in coordinates[:, 1]))
    intervals = []
    for left, right in zip(events, events[1:]):
        if right-left <= _EPS:
            continue
        midpoint = (left+right)/2
        crossings = []
        for first, second in zip(coordinates, np.roll(coordinates, -1, axis=0)):
            if not min(first[1], second[1]) < midpoint < max(first[1], second[1]):
                continue
            slope = float((second[0]-first[0])/(second[1]-first[1]))
            intercept = float(first[0]-slope*first[1])
            crossings.append((slope*midpoint+intercept, slope, intercept))
        crossings.sort()
        if len(crossings) % 2:
            raise ValueError('contact polygon intersection is unresolved')
        for lower_edge, upper_edge in zip(crossings[::2], crossings[1::2]):
            _, ml, cl = lower_edge
            _, mu, cu = upper_edge
            lo, hi = left, right
            for slope, constant in ((ml-mu, cl+minimum-cu),
                                    (ml, cl+minimum-band_upper),
                                    (-mu, band_lower+minimum-cu)):
                lo, hi = _clip_linear(lo, hi, slope, constant)
            if lo <= hi+_EPS:
                intervals.append((lo, hi))
    # Isolated feasible vertex deltas must not disappear between open slabs.
    for value in events:
        if any(min(hi, band_upper)-max(lo, band_lower) >= minimum-_EPS
               for lo, hi in _line_intervals(point-b*value, a)):
            intervals.append((value, value))
    return _merge(intervals)


def _json_intervals(intervals):
    return [[float(a) if math.isfinite(a) else None,
             float(b) if math.isfinite(b) else None] for a, b in intervals]



def _contact_metrics_audit(result):
    """Keep in-memory evidence identical to its persisted JSON structure."""
    metrics = asdict(result)
    # The measured-contact dataclass uses immutable pairs. JSON stores lists;
    # retaining tuples in the frozen report breaks exact audit equality.
    metrics['support_clearances_m'] = [list(pair) for pair in metrics['support_clearances_m']]
    return metrics


def evaluate_contact_height_margin(*, translation_evidence_limit_m, **geometry):
    """Return the complete allowed interval for normal translation, in metres.

    Positive delta translates the gripper along the support normal relative to
    the frozen target and plane. No recommended motion/offset is returned.
    None at an interval end denotes an unbounded end, never missing evidence.
    """
    report = dict(scope='support_normal_translation_1d_necessary_condition',
        diagnostic_only=True, motion_authorized=False, physical_grip_verified=False,
        full_3d_robustness_proven=False, uncertainty_is_certified_bound=False,
        evidence_limit_semantics='caller_supplied_empirical_translation_evidence',
        translation_convention='positive_gripper_translation_along_support_normal',
        algorithm='finite_nonconvex_patch_piecewise_linear_projection',
        evaluated=False, ok=False)
    try:
        limit = translation_evidence_limit_m
        if isinstance(limit, (bool, np.bool_)) or not isinstance(limit, (int, float, np.number)):
            raise ValueError('translation evidence limit must be a finite nonnegative number')
        limit = float(limit)
        if not math.isfinite(limit) or limit < 0:
            raise ValueError('translation evidence limit must be a finite nonnegative number')
        baseline = _contact_metrics_audit(evaluate_measured_contact(**geometry))
        report.update(translation_evidence_limit_m=limit, nominal=baseline)
        if baseline['actual_overlap_m'] is None:
            raise ValueError(baseline['reason'])
        actual = np.asarray(geometry['actual_T_base_tool0'], dtype=float)
        rotation = actual[:3, :3]
        point = (rotation.T @ (np.asarray(geometry['contact_center_base'])-
                 actual[:3, 3])-PAIR_CENTER)[[0, 2]]
        height = (rotation.T @ np.asarray(geometry['contact_height_axis_base']))[[0, 2]]
        normal = np.asarray(geometry['support_normal_base'])
        projected_normal = (rotation.T @ normal)[[0, 2]]
        band_lower, band_upper = geometry['contact_height_bounds_m']
        minimum = float(geometry['minimum_contact_overlap_m'])
        contact = _contact_translation_intervals(point, height, projected_normal,
                                                 band_lower, band_upper, minimum)
        required_clearance = float(geometry['gripper'].support_clearance_m)
        support_lower = required_clearance-baseline['minimum_support_clearance_m']
        allowed = _merge([(max(a, support_lower), b) for a, b in contact
                          if max(a, support_lower) <= b+_EPS])
        containing_zero = next(((a, b) for a, b in allowed if a <= _EPS and b >= -_EPS), None)
        symmetric = (max(0., min(-containing_zero[0], containing_zero[1]))
                     if containing_zero is not None else 0.)
        covered = bool(baseline['ok'] and containing_zero is not None
                       and containing_zero[0] <= -limit+_EPS
                       and containing_zero[1] >= limit-_EPS)
        endpoints = []
        for delta in (-limit, limit):
            shifted = actual.copy()
            shifted[:3, 3] += delta*normal
            metrics = _contact_metrics_audit(evaluate_measured_contact(**dict(geometry, actual_T_base_tool0=shifted)))
            endpoints.append(dict(normal_translation_m=delta, **metrics))
        reason = ('support-normal translation evidence limit +/-%.6f mm; '
                  'maximum symmetric contact/support margin %.6f mm. ' % (limit*1000., symmetric*1000.))
        if not baseline['ok']:
            reason += 'Nominal contact geometry is invalid: %s. ' % baseline['code']
        if covered:
            reason += 'This one-dimensional necessary condition passes; full 3D contact and physical grip remain unverified.'
        else:
            reason += ('Registration calibration/evidence must resolve a tighter normal-direction error budget; '
                       'a historical maximum error is not a certified bound on future error.')
        report.update(evaluated=True, ok=covered, reason=reason,
            code=('CONTACT_HEIGHT_NORMAL_MARGIN_OK' if covered else 'CONTACT_HEIGHT_NORMAL_MARGIN_INSUFFICIENT'),
            contact_normal_translation_intervals_m=_json_intervals(contact),
            allowed_normal_translation_intervals_m=_json_intervals(allowed),
            support_minimum_normal_translation_m=float(support_lower),
            minimum_contact_overlap_m=minimum, minimum_support_clearance_m=required_clearance,
            maximum_symmetric_normal_translation_m=float(symmetric),
            covers_entire_normal_translation_interval=covered,
            endpoint_diagnostics=endpoints)
    except (TypeError, ValueError, KeyError, IndexError, OverflowError, np.linalg.LinAlgError) as exc:
        report.update(code='CONTACT_HEIGHT_MARGIN_UNAVAILABLE', reason=str(exc))
    return report


def plan_contact_height_margin(plan, gripper_config, opening_width_m,
                               translation_evidence_limit_m, actual_T_base_tool0=None):
    """Read-only adapter for the existing immutable plan and CAD contract."""
    geometry = frozen_contact_geometry(plan, gripper_config, opening_width_m)
    planned = geometry['planned_T_base_tool0']
    report = evaluate_contact_height_margin(
        translation_evidence_limit_m=translation_evidence_limit_m,
        actual_T_base_tool0=(planned if actual_T_base_tool0 is None else actual_T_base_tool0),
        **geometry)
    report['plan_id'] = str(plan.plan_id)
    return report


def contact_height_evidence_limit(grasp_config):
    """Shared strict policy for final correction and the independent close gate."""
    cfg = grasp_config if isinstance(grasp_config, dict) else {}
    enabled = cfg.get('contact_height_registration_margin_enabled', False)
    if type(enabled) is not bool:
        raise ValueError('CONTACT_HEIGHT_MARGIN_CONFIG_INVALID: contact height margin policy must be boolean')
    if not enabled:
        return None
    limit = cfg.get('handeye_translation_max_error_m')
    if type(limit) not in (int, float) or not math.isfinite(limit) or limit <= 0.:
        raise ValueError('CONTACT_HEIGHT_MARGIN_CONFIG_INVALID: positive empirical hand-eye translation evidence is required')
    return float(limit)
