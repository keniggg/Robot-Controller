"""Exact concave finger-pad intersections for unknown near-field proposals.

The production input validators and CAD polygon are unchanged. Scalar edge
arithmetic avoids allocating several NumPy arrays per edge during tilt search.
"""
import math
import numpy as np
from alicia_flexible_grasp.grasp.gripper_geometry import (
    ANALYTICAL_FINGER_CONTACT_PATCH_TOOL_XZ_M,
    ANALYTICAL_FINGER_PAIR_CENTER_TOOL_XYZ_M,
    _readonly_vector, _validated_rotation, _finite_number,
)

_POLYGON = tuple(tuple(float(v) for v in p)
                 for p in ANALYTICAL_FINGER_CONTACT_PATCH_TOOL_XZ_M)
_EDGES = tuple((a[0], a[1], b[0], b[1], b[0] - a[0], b[1] - a[1])
               for a, b in zip(_POLYGON, _POLYGON[1:] + _POLYGON[:1]))


def _inside_or_boundary(x, y, tolerance):
    inside = False
    for ax, ay, bx, by, ex, ey in _EDGES:
        dx, dy = x - ax, y - ay
        fraction = min(1., max(0., (dx * ex + dy * ey) / (ex * ex + ey * ey)))
        rx, ry = dx - fraction * ex, dy - fraction * ey
        if math.hypot(rx, ry) <= tolerance:
            return True
        if (ay > y) != (by > y):
            if x < (ax - bx) * (y - by) / (ay - by) + bx:
                inside = not inside
    return inside


def contact_intervals_2d(point, direction, lower, upper):
    """The original edge-crossing algorithm, including boundary tolerances."""
    px, py = map(float, point)
    dx, dy = map(float, direction)
    norm_squared = dx * dx + dy * dy
    if norm_squared <= 1e-18:
        return ((lower, upper),) if _inside_or_boundary(px, py, 1e-12) else ()
    breakpoints = [lower, upper]
    for ax, ay, bx, by, ex, ey in _EDGES:
        ox, oy = ax - px, ay - py
        denominator = dx * ey - dy * ex
        if abs(denominator) > 1e-14:
            height = (ox * ey - oy * ex) / denominator
            fraction = (ox * dy - oy * dx) / denominator
            if lower - 1e-12 <= height <= upper + 1e-12 and -1e-12 <= fraction <= 1. + 1e-12:
                breakpoints.append(min(upper, max(lower, height)))
        elif abs(ox * dy - oy * dx) <= 1e-12:
            for x, y in ((ax, ay), (bx, by)):
                height = ((x - px) * dx + (y - py) * dy) / norm_squared
                if lower - 1e-12 <= height <= upper + 1e-12:
                    breakpoints.append(min(upper, max(lower, height)))
    ordered = []
    for value in sorted(breakpoints):
        if not ordered or abs(value - ordered[-1]) > 1e-11:
            ordered.append(value)
    intervals = []
    for first, second in zip(ordered, ordered[1:]):
        if second - first <= 1e-12:
            continue
        midpoint = .5 * (first + second)
        if not _inside_or_boundary(px + midpoint * dx, py + midpoint * dy, 1e-10):
            continue
        if intervals and first - intervals[-1][1] <= 1e-10:
            intervals[-1] = (intervals[-1][0], second)
        else:
            intervals.append((first, second))
    return tuple(intervals)


def finger_contact_patch_overlap_m(center, tool0, rotation, support, lower, upper):
    center = _readonly_vector(center, 'candidate_center_base')
    tool0 = _readonly_vector(tool0, 'candidate_tool0_base')
    rotation = _validated_rotation(rotation, 'R_base_tool')
    support = _readonly_vector(support, 'support_normal_base')
    if not math.isclose(float(np.linalg.norm(support)), 1., rel_tol=0., abs_tol=1e-6):
        raise ValueError('support_normal_base must be a unit vector')
    lower = _finite_number(lower, 'height_min_m')
    upper = _finite_number(upper, 'height_max_m')
    if upper < lower:
        raise ValueError('contact height bounds must be ordered')
    pair_center = tool0 + rotation @ ANALYTICAL_FINGER_PAIR_CENTER_TOOL_XYZ_M
    local_center = rotation.T @ (center - pair_center)
    local_direction = rotation.T @ support
    intervals = contact_intervals_2d(local_center[[0, 2]], local_direction[[0, 2]], lower, upper)
    return max((second - first for first, second in intervals), default=0.)
