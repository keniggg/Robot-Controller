"""Separate planned projection and image-bound RGB-D working-volume checks.

Depth is optical-forward ``x`` in ROS camera_link (x forward, y left, z up).
All eight OBB corners must fit the effective depth interval and image margin.
A centre-to-camera Euclidean distance is reported only for diagnosis.

Measured-view callers must obtain the transform at the actual image timestamp and
independently establish image freshness, target identity and calibration. Source
and returned TF timestamps record that binding; this function cannot verify a TF
lookup. Passing this geometric check does not certify measured depth quality,
information sufficiency, occlusion, collision safety or grasp success.
"""
from alicia_flexible_grasp.robot.observation_view_limits import MINIMUM_OBSERVATION_IMAGE_MARGIN_PX
from copy import deepcopy
import itertools
import math
from numbers import Integral, Real

import numpy as np

from alicia_flexible_grasp.robot.observation_view_limits import (
    effective_observation_view_limits,
)


class ObservationWorkingVolumeError(ValueError):
    def __init__(self, reason, audit=None):
        self.audit = deepcopy(audit or {})
        self.audit.update(ok=False, reason=str(reason))
        super().__init__(str(reason))


def _array(value, shape, label):
    # Reject coercible strings/bools as well as complex, ragged and nonfinite data.
    raw = np.asarray(value, dtype=object)
    if raw.shape != shape or any(isinstance(v, (bool, np.bool_)) or
            not isinstance(v, Real) for v in raw.flat):
        raise ObservationWorkingVolumeError(label + ' must contain finite real numbers with shape ' + str(shape))
    result = np.asarray(value, dtype=float)
    if not np.all(np.isfinite(result)):
        raise ObservationWorkingVolumeError(label + ' must contain finite real numbers')
    return result


def _rotation(value, label):
    rotation = _array(value, (3, 3), label)
    with np.errstate(over='ignore', invalid='ignore'):
        valid = (np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6, rtol=0.)
                 and abs(float(np.linalg.det(rotation)) - 1.) <= 1e-6)
    if not valid:
        raise ObservationWorkingVolumeError(label + ' must be a proper rigid rotation')
    return rotation


def observation_obb_corners(center_base, axes_base, size_xyz_m):
    """Return eight model corners from frozen OBB fields, never measured points."""
    center = _array(center_base, (3,), 'OBB center')
    axes = _rotation(axes_base, 'OBB axes')
    size = _array(size_xyz_m, (3,), 'OBB size')
    if np.any(size <= 0.):
        raise ObservationWorkingVolumeError('OBB dimensions must be positive')
    with np.errstate(over='ignore', invalid='ignore'):
        corners = center + (np.asarray(tuple(itertools.product((-1., 1.), repeat=3))) * (size * .5)) @ axes.T
    if not np.all(np.isfinite(corners)):
        raise ObservationWorkingVolumeError('OBB corner calculation must be finite')
    return corners


def _stamp(value, label):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Integral) or value <= 0:
        raise ObservationWorkingVolumeError(label + ' must be positive integer nanoseconds')
    return int(value)


def _scalar(value, label):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Real) or not math.isfinite(value):
        raise ObservationWorkingVolumeError(label + ' must be finite numeric')
    return float(value)


def _project(points, transform, k):
    # Same camera_link convention as robot.observation_view_guard.
    with np.errstate(over='ignore', divide='ignore', invalid='ignore'):
        camera = (points - transform[:3, 3]) @ transform[:3, :3]
        depth = camera[:, 0]
        uv = np.column_stack((k[2] - k[0] * camera[:, 1] / depth,
                              k[3] - k[1] * camera[:, 2] / depth))
    return uv, depth


def _working_volume_audit(kind):
    return dict(kind=kind, ok=False,
        camera_frame_convention='ros_camera_link_x_forward_y_left_z_up',
        depth_metric='optical_forward_x', target_model_corner_count=8,
        radial_distance_is_acceptance_gate=False,
        certifies_information_sufficiency=False, certifies_grasp_success=False,
        certifies_collision_tracking_calibration_or_occlusion=False,
        certifies_source_freshness_or_tf_lookup=False,
        requires_independent_information_quality_check=True,
        visible_surface_points_are_complete_geometry_evidence=False,
        visible_surface_points_available=False)


def _validate_working_volume_geometry(
        target_corners_base, base_from_camera, intrinsics, image_width, image_height,
        audit, roi_margin_px, minimum_depth_m, maximum_depth_m):
    """Project model corners without inventing image or transform timestamps."""
    for value in (image_width, image_height):
        if isinstance(value, (bool, np.bool_)) or not isinstance(value, Integral) or not 1 <= value <= 16384:
            raise ObservationWorkingVolumeError('invalid camera image dimensions')
    width, height = int(image_width), int(image_height)
    limits = effective_observation_view_limits({
        'camera_visibility_margin_px': _scalar(roi_margin_px, 'image margin'),
        'camera_visibility_min_depth_m': _scalar(minimum_depth_m, 'minimum depth'),
        'camera_visibility_max_depth_m': _scalar(maximum_depth_m, 'maximum depth')})
    margin, near, far = (limits['roi_margin_px'], limits['minimum_depth_m'], limits['maximum_depth_m'])
    if margin >= (min(width, height) - 1.) / 2.:
        raise ObservationWorkingVolumeError('image margin leaves no camera working volume')
    audit.update(required_image_margin_px=margin, minimum_camera_depth_m=near,
                 maximum_camera_depth_m=far, image_width=width, image_height=height)
    k_shape = np.shape(intrinsics)
    if k_shape == (3, 3):
        k_matrix = _array(intrinsics, (3, 3), 'camera intrinsics')
        expected = [[k_matrix[0, 0], 0., k_matrix[0, 2]],
                    [0., k_matrix[1, 1], k_matrix[1, 2]], [0., 0., 1.]]
        if not np.allclose(k_matrix, expected, atol=1e-9, rtol=0.):
            raise ObservationWorkingVolumeError('unsupported camera intrinsics')
        k = k_matrix[[0, 1, 0, 1], [0, 1, 2, 2]]
    else:
        k = _array(intrinsics, (4,), 'camera intrinsics')
    if np.any(k[:2] <= 0.) or not (0. <= k[2] < width and 0. <= k[3] < height):
        raise ObservationWorkingVolumeError('invalid camera intrinsics')
    transform = _array(base_from_camera, (4, 4), 'base_from_camera')
    _rotation(transform[:3, :3], 'base_from_camera rotation')
    if not np.allclose(transform[3], [0., 0., 0., 1.], atol=1e-9, rtol=0.):
        raise ObservationWorkingVolumeError('base_from_camera must be a rigid transform')
    corners = _array(target_corners_base, (8, 3), 'target OBB corners')
    with np.errstate(over='ignore', invalid='ignore'):
        center = corners.mean(axis=0)
        centered = corners - center
    if (not np.all(np.isfinite(centered)) or len(np.unique(corners, axis=0)) != 8
            or np.linalg.matrix_rank(centered) != 3):
        raise ObservationWorkingVolumeError('target OBB requires eight distinct nondegenerate corners')
    uv, depth = _project(corners, transform, k)
    if not np.all(np.isfinite(depth)):
        raise ObservationWorkingVolumeError('target OBB camera depths must be finite')
    radial_distance = math.hypot(*(center - transform[:3, 3]))
    if not math.isfinite(radial_distance):
        raise ObservationWorkingVolumeError('target OBB radial diagnostic must be finite')
    audit.update(minimum_depth_m=float(np.min(depth)), maximum_depth_m=float(np.max(depth)),
                 target_corner_depths_m=depth.tolist(), target_center_radial_distance_m=radial_distance)
    if np.all(np.isfinite(uv)):
        edge_margin = float(np.min(np.column_stack((uv[:, 0], width - 1. - uv[:, 0],
                                                    uv[:, 1], height - 1. - uv[:, 1]))))
        audit.update(minimum_image_edge_distance_px=edge_margin,
                     target_uv_min_px=uv.min(axis=0).tolist(), target_uv_max_px=uv.max(axis=0).tolist())
    if np.min(depth) < near or np.max(depth) > far:
        raise ObservationWorkingVolumeError('target OBB exceeds camera depth range')
    if not np.all(np.isfinite(uv)):
        raise ObservationWorkingVolumeError('target OBB image projection must be finite')
    if edge_margin < margin:
        raise ObservationWorkingVolumeError('target OBB leaves required camera image margin')
    return transform, k, width, height, margin, near, far


def validate_planned_observation_working_volume(
        target_corners_base, planned_base_from_camera, intrinsics, image_width, image_height,
        *, snapshot_stamp_ns, roi_margin_px=MINIMUM_OBSERVATION_IMAGE_MARGIN_PX, minimum_depth_m=.070, maximum_depth_m=.500):
    """Check a proposed endpoint against geometry frozen in one source snapshot.

    The supplied transform is a derived future camera pose, not a TF measurement.
    ``snapshot_stamp_ns`` identifies only the frozen input geometry. The returned
    audit cannot replace :func:`validate_observation_working_volume` after motion,
    and deliberately contains no actual image/TF binding or measured point data.
    """
    audit = _working_volume_audit('planned_observation_working_volume')
    audit.update(evidence_kind='planned_endpoint_geometry',
                 camera_transform_source='planned_camera_pose',
                 actual_arrival_measurement_required=True)
    try:
        audit['snapshot_stamp_ns'] = _stamp(snapshot_stamp_ns, 'frozen geometry snapshot timestamp')
        _validate_working_volume_geometry(
            target_corners_base, planned_base_from_camera, intrinsics, image_width, image_height,
            audit, roi_margin_px, minimum_depth_m, maximum_depth_m)
        audit['ok'] = True
        return audit
    except (TypeError, ValueError, OverflowError, np.linalg.LinAlgError) as exc:
        audit.update(getattr(exc, 'audit', {}))
        raise ObservationWorkingVolumeError(str(exc), audit) from exc


def validate_observation_working_volume(
        target_corners_base, base_from_camera, intrinsics, image_width, image_height,
        *, source_stamp_ns, camera_transform_stamp_ns, roi_margin_px=MINIMUM_OBSERVATION_IMAGE_MARGIN_PX,
        minimum_depth_m=.070, maximum_depth_m=.500,
        visible_surface_points_base=None, visible_surface_stamp_ns=None,
        camera_transform_stamp_tolerance_ns=0):
    """Return an audit, or raise ``ObservationWorkingVolumeError`` with an audit.

    ``target_corners_base`` contains the frozen OBB's eight model corners; use
    :func:`observation_obb_corners` for center/axes/size geometry. Effective
    limits may tighten the shared D405 defaults, never relax them. Exact range
    endpoints are inclusive. No radial target or motion command is produced.
    TF stamps normally match exactly. An explicit nanosecond tolerance may
    only cover half an IEEE double ULP plus integer rounding at the source
    epoch, as used by the ROS TF Python/C++ boundary; it is never an age limit.

    Optional visible surface points must come from this same image. They are
    diagnostic observations, never evidence for a complete object shape or a
    substitute for the independent perception/information-quality gates. Their
    absence is explicitly recorded and does not create synthetic evidence.
    """
    audit = _working_volume_audit('image_bound_observation_working_volume')
    try:
        source_stamp = _stamp(source_stamp_ns, 'source image timestamp')
        transform_stamp = _stamp(camera_transform_stamp_ns, 'camera transform timestamp')
        tolerance = camera_transform_stamp_tolerance_ns
        quantization_bound = int(math.ceil(float(np.spacing(source_stamp * 1e-9)) * 1e9 * .5)) + 1
        if (isinstance(tolerance, (bool, np.bool_)) or not isinstance(tolerance, Integral)
                or not 0 <= tolerance <= quantization_bound):
            raise ObservationWorkingVolumeError('invalid camera transform timestamp quantization tolerance')
        audit.update(source_stamp_ns=source_stamp, camera_transform_stamp_ns=transform_stamp,
                     camera_transform_stamp_delta_ns=transform_stamp - source_stamp,
                     camera_transform_stamp_tolerance_ns=int(tolerance))
        if abs(transform_stamp - source_stamp) > tolerance:
            raise ObservationWorkingVolumeError('camera transform must be bound to source image timestamp')
        transform, k, width, height, margin, near, far = _validate_working_volume_geometry(
            target_corners_base, base_from_camera, intrinsics, image_width, image_height,
            audit, roi_margin_px, minimum_depth_m, maximum_depth_m)
        if visible_surface_points_base is None:
            if visible_surface_stamp_ns is not None:
                raise ObservationWorkingVolumeError('visible surface timestamp without measured points')
            audit.update(visible_surface_point_count=0, visible_surface_points_in_working_volume=0)
        else:
            if _stamp(visible_surface_stamp_ns, 'visible surface timestamp') != source_stamp:
                raise ObservationWorkingVolumeError('visible surface points must be bound to source image timestamp')
            shape = np.shape(visible_surface_points_base)
            if len(shape) != 2 or shape[1:] != (3,) or not 1 <= shape[0] <= 2000000:
                raise ObservationWorkingVolumeError('invalid or empty visible surface points')
            points = _array(visible_surface_points_base, shape, 'visible surface points')
            surface_uv, surface_depth = _project(points, transform, k)
            valid = (np.all(np.isfinite(surface_uv), axis=1) & np.isfinite(surface_depth)
                     & (surface_depth >= near) & (surface_depth <= far)
                     & (surface_uv[:, 0] >= margin) & (surface_uv[:, 0] <= width - 1. - margin)
                     & (surface_uv[:, 1] >= margin) & (surface_uv[:, 1] <= height - 1. - margin))
            audit.update(visible_surface_points_available=True, visible_surface_stamp_ns=source_stamp,
                         visible_surface_point_count=int(len(points)),
                         visible_surface_points_in_working_volume=int(np.sum(valid)))
        audit['ok'] = True
        return audit
    except (TypeError, ValueError, OverflowError, np.linalg.LinAlgError) as exc:
        audit.update(getattr(exc, 'audit', {}))
        raise ObservationWorkingVolumeError(str(exc), audit) from exc
