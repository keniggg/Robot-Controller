"""Conservative support-aligned enclosures of every measured object point.

This changes only a box representation. It does not remove points, invent
surface evidence, or qualify any grasp. Callers keep all original contact,
clearance, reachability and execution gates.
"""
from dataclasses import replace
import hashlib
import math

import numpy as np


def _convex_hull(points):
    """Deterministic monotone-chain hull in float64, with no quantile trim."""
    ordered = sorted(set(map(tuple, points)))
    if len(ordered) < 3:
        raise ValueError('measured support-plane hull is degenerate')

    def turn(a, b, c):
        return ((b[0] - a[0]) * (c[1] - a[1])
                - (b[1] - a[1]) * (c[0] - a[0]))

    lower, upper = [], []
    for point in ordered:
        while len(lower) >= 2 and turn(lower[-2], lower[-1], point) <= 0.0:
            lower.pop()
        lower.append(point)
    for point in reversed(ordered):
        while len(upper) >= 2 and turn(upper[-2], upper[-1], point) <= 0.0:
            upper.pop()
        upper.append(point)
    hull = np.asarray(lower[:-1] + upper[:-1], dtype=np.float64)
    if len(hull) < 3:
        raise ValueError('measured support-plane hull is degenerate')
    return hull


def minimum_area_measured_enclosure(estimate, *, min_size_m=.005,
                                    max_size_m=.600, max_height_m=.500):
    """Return an immutable geometry estimate and an auditable full-point box.

    The minimum-area support-plane rectangle is found over all hull-edge
    orientations. Height retains the original support-resting convention:
    the bottom is at the support plane or the lowest measured point, whichever
    is lower, and the top contains the highest measured point. A top-only
    observation above the plane remains a geometry estimate; it still needs
    independently measured bilateral contact evidence to authorize a grasp.

    Invalid or out-of-bounds measurements raise ValueError; callers must not
    silently replace them with a smaller or previously valid enclosure.
    """
    if not bool(getattr(estimate, 'ok', False)):
        raise ValueError('a valid measured geometry estimate is required')
    limits = np.asarray([min_size_m, max_size_m, max_height_m], dtype=float)
    if (not np.all(np.isfinite(limits)) or np.any(limits <= 0.0)
            or min_size_m > max_size_m):
        raise ValueError('enclosure size limits must be finite and positive')
    points = np.asarray(estimate.object_points_base, dtype=np.float64)
    normal = np.asarray(estimate.support_normal_base, dtype=np.float64)
    offset = float(estimate.support_offset_m)
    if (points.ndim != 2 or points.shape[1:] != (3,) or len(points) < 3
            or not np.all(np.isfinite(points))):
        raise ValueError('measured points must be finite Nx3 coordinates')
    if (normal.shape != (3,) or not np.all(np.isfinite(normal))
            or not math.isfinite(offset)
            or not math.isclose(float(np.linalg.norm(normal)), 1.0,
                                rel_tol=0.0, abs_tol=1e-10)):
        raise ValueError('measured support normal must be unit length')

    seed = np.eye(3)[int(np.argmin(np.abs(normal)))]
    first = seed - np.dot(seed, normal) * normal
    first /= np.linalg.norm(first)
    second = np.cross(normal, first)
    second /= np.linalg.norm(second)
    basis = np.column_stack((first, second))
    coordinates = points @ basis
    hull = _convex_hull(coordinates)
    boxes = []
    for edge in np.roll(hull, -1, axis=0) - hull:
        direction = edge / np.linalg.norm(edge)
        axis = basis @ direction
        perpendicular = np.cross(normal, axis)
        perpendicular /= np.linalg.norm(perpendicular)
        spans = np.ptp(points @ np.column_stack((axis, perpendicular)), axis=0)
        if spans[0] < spans[1]:
            axis = perpendicular
        # Fix signs without the sample order or previous frame as authority.
        significant = np.flatnonzero(np.abs(axis) > 1e-12)
        if axis[significant[0]] < 0.0:
            axis = -axis
        perpendicular = np.cross(normal, axis)
        perpendicular /= np.linalg.norm(perpendicular)
        axes = np.column_stack((axis, perpendicular, normal))
        projected = points @ axes[:, :2]
        lower, upper = np.min(projected, axis=0), np.max(projected, axis=0)
        spans = upper - lower
        boxes.append((float(spans[0] * spans[1]), axes, lower, upper))
    minimum_area = min(box[0] for box in boxes)
    # Equivalent rectangle symmetries can differ by floating-point roundoff.
    # Pick one canonical orientation only within machine-precision area ties.
    tolerance = 32.0 * np.finfo(float).eps * max(minimum_area, 1e-12)
    tied = [box for box in boxes if box[0] <= minimum_area + tolerance]
    area, axes, lower, upper = max(
        tied, key=lambda box: tuple(np.round(box[1][:, 0], 12)))
    signed_heights = points @ normal + offset
    bottom = min(0.0, float(np.min(signed_heights)))
    top = float(np.max(signed_heights))
    height = top - bottom
    size = np.r_[upper - lower, height]
    if (not np.all(np.isfinite(size)) or np.any(size < float(min_size_m))
            or np.any(size > float(max_size_m)) or height > float(max_height_m)):
        raise ValueError('full measured enclosure exceeds configured size limits')
    center = (axes[:, :2] @ (0.5 * (lower + upper))
              + normal * (0.5 * (bottom + top) - offset))
    enclosed = (points - center) @ axes
    if (not np.allclose(axes.T @ axes, np.eye(3), atol=1e-10, rtol=0.0)
            or not math.isclose(float(np.linalg.det(axes)), 1.0,
                                abs_tol=1e-10, rel_tol=0.0)
            or np.any(np.abs(enclosed) > 0.5 * size + 1e-10)):
        raise ValueError('full measured enclosure failed its containment contract')
    for value in (center, axes, size):
        value.setflags(write=False)
    result = replace(estimate, center_base=center, axes_base=axes, size_xyz_m=size)
    points_bytes = np.ascontiguousarray(points, dtype='<f8').tobytes()
    audit = {
        'method': 'support_plane_full_measured_minimum_area_rectangle',
        'reason': 'retain all measured boundary points without axis-wise quantile trimming',
        'point_count': int(len(points)),
        'object_points_sha256': hashlib.sha256(points_bytes).hexdigest(),
        'point_checksum_encoding': 'little_endian_float64_row_major_xyz',
        'all_measured_points_enclosed': True,
        'points_removed': 0,
        'support_height_bounds_m': [bottom, top],
        'support_plane_area_m2': area,
        'previous_size_xyz_m': np.asarray(estimate.size_xyz_m, dtype=float).tolist(),
        'size_xyz_m': size.tolist(),
    }
    return result, audit
