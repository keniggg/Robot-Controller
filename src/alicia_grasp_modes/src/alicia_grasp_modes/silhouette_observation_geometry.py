"""Conservative collision envelope for diagnostic observation only.

A fresh estimate must not discard the occupied box that caused the initial
contact search to fail. This module encloses both boxes and both measured point
sets in the fresh axes; it never selects a smaller silhouette, creates points, admits contact,
or replaces the caller's current source/target/TF and loss-latch checks.
"""
from dataclasses import replace
import hashlib
import itertools
import json
from numbers import Real

import numpy as np

from alicia_flexible_grasp.vision.object_geometry import GeometryEstimate
from .tabletop import Config, _normalized_plane, _validate_support_plane_update


_CORNER_SIGNS = np.asarray(tuple(itertools.product((-1., 1.), repeat=3)))
_NUMERICAL_TOLERANCE_M = 1e-12


def _array_hash(values):
    array = np.ascontiguousarray(values, dtype='<f8')
    return hashlib.sha256(array.tobytes()).hexdigest()


def _validated_geometry(value, name):
    if not isinstance(value, GeometryEstimate) or value.ok is not True:
        raise ValueError('SILHOUETTE_ENVELOPE_%s_GEOMETRY_INVALID' % name.upper())
    center = np.asarray(value.center_base, dtype=float)
    axes = np.asarray(value.axes_base, dtype=float)
    size = np.asarray(value.size_xyz_m, dtype=float)
    points = np.asarray(value.object_points_base, dtype=float)
    ratio = float(value.support_inlier_ratio)
    plane = _normalized_plane(np.r_[value.support_normal_base, value.support_offset_m])
    if (center.shape != (3,) or axes.shape != (3, 3) or size.shape != (3,)
            or not all(np.all(np.isfinite(v)) for v in (center, axes, size, points))
            or not np.all(size > 0.)
            or not np.allclose(axes.T @ axes, np.eye(3), atol=1e-6, rtol=0.)
            or abs(float(np.linalg.det(axes)) - 1.) > 1e-6
            or abs(float(axes[:, 2] @ plane[:3]) - 1.) > 1e-6
            or points.ndim != 2 or points.shape[1:] != (3,) or len(points) == 0
            or not np.isfinite(ratio) or not 0. < ratio <= 1.
            or value.source_mode not in ('instance_mask', 'bbox_depth')):
        raise ValueError('SILHOUETTE_ENVELOPE_%s_GEOMETRY_INVALID' % name.upper())
    corners = center + (_CORNER_SIGNS * (size * .5)) @ axes.T
    return corners, plane


def _geometry_record(geometry, corners):
    data = dict(center_base_m=geometry.center_base.tolist(),
        axes_base=geometry.axes_base.tolist(), size_xyz_m=geometry.size_xyz_m.tolist(),
        support_plane_base=np.r_[geometry.support_normal_base, geometry.support_offset_m].tolist(),
        support_inlier_ratio=float(geometry.support_inlier_ratio), source_mode=geometry.source_mode,
        object_points_count=int(len(geometry.object_points_base)),
        object_points_sha256=_array_hash(geometry.object_points_base),
        obb_corners_base_m=corners.tolist())
    data['geometry_sha256'] = hashlib.sha256(json.dumps(
        data, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()
    return data


def conservative_silhouette_observation_geometry(
        original, fresh, *, original_source_stamp_ns, fresh_source_stamp_ns,
        max_size_m, max_height_m, support_config=None):
    """Return ``(expanded_fresh_geometry, audit)`` for a no-contact prefix.

    Source stamps are exact integers; identity, freshness, TF, phase and
    immutable original-request binding remain caller-owned. The additional
    support check covers the expanded collision envelope projected onto the
    original support. These are model vertices, not measured support samples;
    this does not replace perception's fixed-anchor support gate (7 mm by default).
    All dimensions, including the combined envelope, retain existing limits.
    ``object_points_base`` contains only the fresh measured points, unchanged.
    """
    if (type(original_source_stamp_ns) is not int or original_source_stamp_ns <= 0
            or type(fresh_source_stamp_ns) is not int
            or fresh_source_stamp_ns <= original_source_stamp_ns):
        raise ValueError('SILHOUETTE_ENVELOPE_SOURCE_ORDER_INVALID')
    if (isinstance(max_size_m, bool) or isinstance(max_height_m, bool)
            or not isinstance(max_size_m, Real) or not isinstance(max_height_m, Real)
            or not np.isfinite(max_size_m) or not np.isfinite(max_height_m)
            or max_size_m <= 0. or max_height_m <= 0.):
        raise ValueError('SILHOUETTE_ENVELOPE_SIZE_LIMIT_INVALID')
    maximum_size, maximum_height = float(max_size_m), float(max_height_m)
    cfg = Config() if support_config is None else support_config
    baseline = Config()
    if (not isinstance(cfg, Config)
            or isinstance(cfg.support_max_local_shift_m, bool)
            or isinstance(cfg.support_max_normal_change_deg, bool)
            or not isinstance(cfg.support_max_local_shift_m, Real)
            or not isinstance(cfg.support_max_normal_change_deg, Real)
            or not np.isfinite(cfg.support_max_local_shift_m)
            or not np.isfinite(cfg.support_max_normal_change_deg)
            or not 0. < cfg.support_max_local_shift_m <= baseline.support_max_local_shift_m
            or not 0. < cfg.support_max_normal_change_deg <= baseline.support_max_normal_change_deg):
        raise ValueError('SILHOUETTE_ENVELOPE_SUPPORT_LIMIT_INVALID')
    old_corners, old_plane = _validated_geometry(original, 'original')
    new_corners, new_plane = _validated_geometry(fresh, 'fresh')
    for geometry in (original, fresh):
        if (np.any(geometry.size_xyz_m > maximum_size + _NUMERICAL_TOLERANCE_M)
                or geometry.size_xyz_m[2] > maximum_height + _NUMERICAL_TOLERANCE_M):
            raise ValueError('SILHOUETTE_ENVELOPE_INPUT_SIZE_LIMIT_EXCEEDED')
    # OBB fitting may trim measured tails (for example the 2--98 percentiles).
    # Preserve those real points as occupied bounds too, without appending old
    # measurements or model corners to fresh.object_points_base.
    bound_points = np.vstack((old_corners, new_corners,
        original.object_points_base, fresh.object_points_base))
    local = (bound_points - fresh.center_base) @ fresh.axes_base
    low = np.nextafter(np.min(local, axis=0), -np.inf)
    high = np.nextafter(np.max(local, axis=0), np.inf)
    size = high - low
    if (np.any(size > maximum_size + _NUMERICAL_TOLERANCE_M)
            or size[2] > maximum_height + _NUMERICAL_TOLERANCE_M):
        raise ValueError('SILHOUETTE_ENVELOPE_COMBINED_SIZE_LIMIT_EXCEEDED')
    combined = replace(fresh,
        center_base=fresh.center_base + fresh.axes_base @ (.5 * (low + high)),
        size_xyz_m=size)
    combined_corners, _ = _validated_geometry(combined, 'combined')
    # A linear support-plane delta reaches its maximum at an envelope vertex.
    # Check the entire new occupied box, including measured tails outside both
    # input OBBs; these projected model vertices are not support measurements.
    footprint = combined_corners - (
        combined_corners @ old_plane[:3] + old_plane[3])[:, None] * old_plane[:3]
    aligned_plane, support = _validate_support_plane_update(
        old_plane, new_plane, original.center_base, cfg, footprint)
    excess = np.abs((bound_points - combined.center_base) @ combined.axes_base) - .5 * combined.size_xyz_m
    maximum_excess = float(np.max(excess))
    if maximum_excess > _NUMERICAL_TOLERANCE_M:
        raise ValueError('SILHOUETTE_ENVELOPE_CONTAINMENT_FAILED')
    original_preserved = bool(np.all(excess[:8] <= _NUMERICAL_TOLERANCE_M))
    fresh_preserved = bool(np.all(excess[8:16] <= _NUMERICAL_TOLERANCE_M))
    old_points_end = 16 + len(original.object_points_base)
    old_points_preserved = bool(np.all(excess[16:old_points_end] <= _NUMERICAL_TOLERANCE_M))
    fresh_points_preserved = bool(np.all(excess[old_points_end:] <= _NUMERICAL_TOLERANCE_M))
    audit = dict(version=1, code='SILHOUETTE_OBSERVATION_CONSERVATIVE_ENVELOPE',
        diagnostic_observation_only=True, executable_contact_geometry=False,
        motion_authorized=False,
        source=dict(original_stamp_ns=original_source_stamp_ns, fresh_stamp_ns=fresh_source_stamp_ns),
        original=_geometry_record(original, old_corners), fresh=_geometry_record(fresh, new_corners),
        envelope=_geometry_record(combined, combined_corners),
        containment=dict(original_corners_preserved=original_preserved,
            fresh_corners_preserved=fresh_preserved,
            original_measured_points_preserved=old_points_preserved,
            fresh_measured_points_preserved=fresh_points_preserved,
            maximum_corner_excess_m=float(np.max(excess[:16])),
            maximum_measured_point_excess_m=float(np.max(excess[16:])),
            maximum_input_excess_m=maximum_excess,
            numerical_tolerance_m=_NUMERICAL_TOLERANCE_M),
        collision_bound_sources=['original_obb', 'fresh_obb',
            'original_measured_points', 'fresh_measured_points'],
        measured_points=dict(source='fresh_measurement_only', count=int(len(combined.object_points_base)),
            sha256=_array_hash(combined.object_points_base), synthetic_points_added=0),
        support_consistency=dict(support, reference_plane_base=old_plane.tolist(),
            aligned_current_plane_base=aligned_plane.tolist(),
            maximum_local_shift_m=float(cfg.support_max_local_shift_m),
            maximum_normal_change_deg=float(cfg.support_max_normal_change_deg),
            footprint_kind='projected_expanded_collision_envelope_not_measured_support',
            footprint_base_m=footprint.tolist(), perception_fixed_anchor_gate_replaced=False),
        size_limits=dict(max_size_m=maximum_size, max_height_m=maximum_height))
    return combined, audit
