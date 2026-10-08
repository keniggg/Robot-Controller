"""Conservative OBB enclosure of already registered, measured target points.

This module neither registers observations nor changes their coordinate frame.
The caller must supply the surface in the geometry's current snapshot frame.
An enclosing box is collision geometry, not proof of contact or free space.
"""
from dataclasses import replace
import hashlib
import json
from numbers import Real

import numpy as np

from .multiview_surface import FusedTargetSurface
from .object_geometry import GeometryEstimate
from .target_observation import TargetTrackIdentity


def _finite_array(value, shape, name):
    array = np.asarray(value)
    if array.dtype.kind not in 'iuf' or array.shape != shape:
        raise ValueError('FUSED_SCENE_GEOMETRY_INVALID: ' + name)
    array = np.asarray(array, dtype=np.float64)
    if not np.all(np.isfinite(array)):
        raise ValueError('FUSED_SCENE_GEOMETRY_INVALID: nonfinite ' + name)
    return array


def _points(value, name):
    array = np.asarray(value)
    if array.ndim != 2 or array.shape[1:] != (3,) or not len(array):
        raise ValueError('FUSED_SCENE_GEOMETRY_INVALID: empty or malformed ' + name)
    return _finite_array(array, array.shape, name)


def _number(value, name, positive=False):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Real):
        raise ValueError('FUSED_SCENE_GEOMETRY_INVALID: ' + name)
    value = float(value)
    if not np.isfinite(value) or (positive and value <= 0.):
        raise ValueError('FUSED_SCENE_GEOMETRY_INVALID: ' + name)
    return value


def _identity(value):
    if not isinstance(value, TargetTrackIdentity):
        raise ValueError('FUSED_SCENE_GEOMETRY_INVALID: target identity type')
    # Revalidate even frozen dataclasses, whose contents may have crossed an
    # untrusted serialization or a caller that bypassed __post_init__.
    return TargetTrackIdentity(value.epoch, value.track_id)


def _array_digest(value, dtype='<f8'):
    array = np.ascontiguousarray(value, dtype=dtype)
    digest = hashlib.sha256()
    digest.update(str(array.shape).encode('ascii'))
    digest.update(b'\0' + array.dtype.str.encode('ascii') + b'\0')
    digest.update(array.tobytes())
    return digest.hexdigest()


def enclose_registered_surface(geometry, surface, *, max_size_m=.6,
                               max_height_m=.5, expected_identity=None):
    """Return an outward-only OBB and exact union of admitted measurements.

    Keep the original center, axes and support plane. Enclose every distinct
    input point and all eight corners of the original box by increasing only
    its half extents. Exact duplicates are one spatial sample; no quantization,
    outlier trimming, percentile fitting, synthetic points or TF changes occur.
    ``max_size_m`` bounds each OBB dimension and ``max_height_m`` additionally
    bounds axis 2, matching ``estimate_object_geometry``. Invalid/oversize
    inputs raise ValueError instead of returning an apparently valid fallback.

    GeometryEstimate carries no identity. Pass ``expected_identity`` from the
    current snapshot to validate that binding; source-stamp/registration
    authority remains the caller's responsibility. This function validates the
    fused type and provenance, but cannot independently prove registration.
    """
    if not isinstance(geometry, GeometryEstimate) or geometry.ok is not True:
        raise ValueError('FUSED_SCENE_GEOMETRY_INVALID: valid GeometryEstimate required')
    if not isinstance(surface, FusedTargetSurface):
        raise ValueError('FUSED_SCENE_GEOMETRY_INVALID: FusedTargetSurface required')
    maximum = _number(max_size_m, 'max_size_m', positive=True)
    height_maximum = _number(max_height_m, 'max_height_m', positive=True)
    center = _finite_array(geometry.center_base, (3,), 'center_base')
    axes = _finite_array(geometry.axes_base, (3, 3), 'axes_base')
    if (not np.allclose(axes.T @ axes, np.eye(3), rtol=0., atol=1e-8)
            or not np.isclose(np.linalg.det(axes), 1., rtol=0., atol=1e-8)):
        raise ValueError('FUSED_SCENE_GEOMETRY_INVALID: axes must be right-handed orthonormal')
    size = _finite_array(geometry.size_xyz_m, (3,), 'size_xyz_m')
    if np.any(size <= 0.):
        raise ValueError('FUSED_SCENE_GEOMETRY_INVALID: size must be positive')
    normal = _finite_array(geometry.support_normal_base, (3,), 'support_normal_base')
    if not np.isclose(np.linalg.norm(normal), 1., rtol=0., atol=1e-6):
        raise ValueError('FUSED_SCENE_GEOMETRY_INVALID: support normal must be unit')
    if not np.allclose(axes[:, 2], normal, rtol=0., atol=1e-6):
        raise ValueError('FUSED_SCENE_GEOMETRY_INVALID: height axis must match support normal')
    offset = _number(geometry.support_offset_m, 'support_offset_m')
    ratio = _number(geometry.support_inlier_ratio, 'support_inlier_ratio')
    if not 0. <= ratio <= 1.:
        raise ValueError('FUSED_SCENE_GEOMETRY_INVALID: support inlier ratio')
    if geometry.failure_code or geometry.failure_reason:
        raise ValueError('FUSED_SCENE_GEOMETRY_INVALID: failed geometry evidence')
    if not isinstance(geometry.source_mode, str) or not geometry.source_mode.strip():
        raise ValueError('FUSED_SCENE_GEOMETRY_INVALID: geometry source mode')
    raw = _points(geometry.object_points_base, 'object_points_base')
    identity = _identity(surface.identity)
    if expected_identity is not None and identity != _identity(expected_identity):
        raise ValueError('FUSED_SCENE_GEOMETRY_INVALID: target identity mismatch')
    fused = _points(surface.points_base, 'surface.points_base')
    # Reuse the existing strict source-index and increasing-stamp contract.
    validated_surface = FusedTargetSurface(identity, fused, surface.view_indices,
        surface.view_stamps_ns, view_sources=surface.view_sources)
    combined = np.concatenate((raw, validated_surface.points_base), axis=0)
    # Exact XYZ union preserves first occurrence order and all distinct points.
    # No point is averaged, projected to a plane, cropped, or voxel-pruned.
    _, first_indices = np.unique(combined, axis=0, return_index=True)
    points = combined[np.sort(first_indices)]
    with np.errstate(over='ignore', invalid='ignore'):
        measured_half_extent = np.max(np.abs((points - center) @ axes), axis=0)
        expanded_size = 2. * np.maximum(.5 * size, measured_half_extent)
    if not np.all(np.isfinite(expanded_size)) or np.any(expanded_size <= 0.):
        raise ValueError('FUSED_SCENE_GEOMETRY_INVALID: nonfinite enclosure')
    if np.any(expanded_size > maximum) or expanded_size[2] > height_maximum:
        raise ValueError('FUSED_SCENE_GEOMETRY_OVERSIZE: enclosure exceeds caller bounds')
    result = replace(geometry, size_xyz_m=expanded_size, object_points_base=points)
    source_counts = [0] * len(validated_surface.view_stamps_ns)
    for sources in validated_surface.view_sources:
        for index in sources:
            source_counts[index] += 1
    audit = dict(
        schema_version=1, policy='registered_measured_surface_outward_enclosure_v1',
        target_epoch=identity.epoch, target_track_id=identity.track_id,
        expected_identity_checked=expected_identity is not None,
        coordinate_contract='caller_current_snapshot_registered_surface',
        source='accepted_measured_current_raw_and_registered_fused_points',
        registration_performed=False, tf_modified=False, synthetic_points_added=0,
        contact_or_motion_authorized=False, original_obb_corner_count=8,
        original_obb_preserved=True, center_preserved=True, axes_preserved=True,
        support_plane_preserved=True, center_base_m=center.tolist(),
        axes_base=axes.tolist(), support_normal_base=normal.tolist(),
        support_offset_m=offset, old_size_xyz_m=size.tolist(),
        new_size_xyz_m=expanded_size.tolist(), expanded=bool(np.any(expanded_size > size)),
        max_size_m=maximum, max_height_m=height_maximum,
        current_raw_point_count=len(raw), fused_point_count=len(fused),
        enclosed_point_count=len(points), exact_duplicates_removed=len(combined)-len(points),
        point_union_policy='exact_xyz_keep_first_no_downsampling',
        current_raw_points_sha256=_array_digest(raw),
        fused_points_sha256=_array_digest(fused),
        enclosed_points_sha256=_array_digest(points),
        fused_view_indices_sha256=_array_digest(validated_surface.view_indices, '<i8'),
        view_stamps_ns=list(validated_surface.view_stamps_ns),
        fused_view_sources_sha256=hashlib.sha256(json.dumps(
            validated_surface.view_sources, separators=(',', ':')).encode('ascii')).hexdigest(),
        per_view_point_counts=source_counts,
        per_view_representative_point_counts=np.bincount(validated_surface.view_indices,
            minlength=len(validated_surface.view_stamps_ns)).tolist(),
        limitations=['Enclosure does not prove unseen shape, registration correctness or contact support.',
                     'The caller must retain current snapshot and registration source binding.'])
    return result, audit
