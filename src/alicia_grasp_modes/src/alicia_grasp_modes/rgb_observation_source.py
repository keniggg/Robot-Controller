"""Source binding for RGB-only observation evidence; no ROS or motion I/O."""
from copy import deepcopy
import hashlib
import json

import numpy as np

from alicia_grasp_modes.tabletop import _support_quadrants


JOINT_NAMES = tuple('Joint%d' % index for index in range(1, 7))


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def camera_geometry_snapshot(camera):
    """Image formation only: changing filter telemetry is not new geometry."""
    keys = ('fx', 'fy', 'cx', 'cy', 'depth_scale', 'width', 'height', 'frame_id',
            'align_depth_to_color', 'intrinsics_source', 'model', 'coeffs',
            'distortion_model', 'distortion_coefficients', 'color_topic', 'depth_topic')
    result = {key: deepcopy(camera.get(key)) for key in keys}
    profile = camera.get('runtime_profile')
    if isinstance(profile, dict):
        result['runtime_profile_geometry'] = {key: deepcopy(profile.get(key)) for key in (
            'device', 'geometry_intrinsics_source', 'geometry_intrinsics',
            'color_intrinsics', 'native_depth_intrinsics', 'alignment',
            'depth_to_color_extrinsics', 'depth_scale_m_per_unit')}
        if (profile.get('rectification') is not None
                or 'rectified' in str(profile.get('geometry_intrinsics_source', ''))
                or 'rectified' in str(camera.get('intrinsics_source', ''))
                or bool(camera.get('pair_rectification'))):
            from alicia_flexible_grasp.vision.rgbd_rectification import rectification_contract
            contract = rectification_contract(camera)
            if contract is not None:
                result['runtime_profile_geometry']['rectification'] = deepcopy(contract)
        if profile.get('geometry_activation_stamp_ns') is not None:
            result['runtime_profile_geometry']['geometry_activation_stamp_ns'] = profile['geometry_activation_stamp_ns']
    elif ('rectified' in str(camera.get('intrinsics_source', ''))
            or bool(camera.get('pair_rectification'))):
        from alicia_flexible_grasp.vision.rgbd_rectification import rectification_contract
        rectification_contract(camera)  # Never erase an incomplete active declaration.
    return result


def require_fresh_source(stamp_ns, now_ns, max_age_seconds=.8):
    if (type(stamp_ns) is not int or type(now_ns) is not int or stamp_ns <= 0
            or isinstance(max_age_seconds, bool) or not np.isfinite(max_age_seconds)
            or not 0. < max_age_seconds <= .8):
        raise ValueError('invalid_observation_source_age_contract')
    age = (now_ns - stamp_ns) * 1e-9
    if not 0. <= age <= max_age_seconds:
        raise ValueError('source_exceeds_existing_frame_limit')
    return age


def bracket_feedback(records, stamp_ns, epoch_ns):
    """Interpolate only between measured same-epoch samples, at most 200 ms."""
    if (type(stamp_ns) is not int or type(epoch_ns) is not int
            or not 0 < epoch_ns < stamp_ns):
        raise ValueError('source_predates_selection_or_epoch')
    by_stamp = {}
    for ns, positions in records:
        values = np.asarray(positions, dtype=float)
        if (type(ns) is not int or ns < epoch_ns or values.shape != (6,)
                or not np.all(np.isfinite(values))):
            continue
        if ns in by_stamp and not np.array_equal(by_stamp[ns], values):
            raise ValueError('conflicting_accepted_feedback_source')
        by_stamp[ns] = values
    before = [ns for ns in by_stamp if ns <= stamp_ns]
    after = [ns for ns in by_stamp if ns >= stamp_ns]
    if not before or not after:
        raise ValueError('accepted_feedback_does_not_bracket_source')
    lo, hi = max(before), min(after)
    if hi - lo > 200_000_000:
        raise ValueError('accepted_feedback_does_not_bracket_source')
    weight = (stamp_ns - lo) / max(1, hi - lo)
    return ((1. - weight) * by_stamp[lo] + weight * by_stamp[hi],
            (lo, by_stamp[lo].tolist()), (hi, by_stamp[hi].tolist()))


def measured_support_coverage(depth, intrinsics, transform, plane, config, outer):
    """Count the original segmenter's same-frame ring and support quadrants.

    Evaluate the same signed plane equation with sparse pixel grids, avoiding a
    second full HxWx3 point-cloud allocation or an additional plane fit.
    """
    if not np.isfinite(outer) or outer <= 0:
        raise ValueError('invalid_measured_support_ring')
    fx, fy, cx, cy = np.asarray(intrinsics, dtype=float)
    if not np.all(np.isfinite((fx, fy, cx, cy))) or min(fx, fy) <= 0.:
        raise ValueError('invalid_camera_intrinsics')
    yy, xx = np.ogrid[:depth.shape[0], :depth.shape[1]]
    center = [depth.shape[1] / 2., depth.shape[0] / 2.]
    radius = np.hypot(xx - center[0], yy - center[1])
    scale = min(depth.shape[1] / 640., depth.shape[0] / 480.)
    n = np.asarray(transform).T @ np.asarray(plane)
    distance = (depth * n[0] - (xx - cx) * depth * n[1] / fx
                - (yy - cy) * depth * n[2] / fy + n[3])
    valid = np.isfinite(depth) & (depth >= config.depth_min_m) & (depth <= config.depth_max_m)
    supporting = (valid & (radius >= 70 * scale) & (radius <= outer)
                  & (np.abs(distance) <= config.support_threshold_m))
    return dict(spatial_quadrants=_support_quadrants(supporting, xx, yy, center),
                measured_support_pixel_count=int(np.count_nonzero(supporting)))


def empty_observation_interest(reason, *, source_stamp_ns=None):
    """A tombstone: latched JSON must never retain a preceding ready interest."""
    return dict(version=1, reason=str(reason), observation_interest_ready=False,
                observation_only=True, metric_geometry_authority=False,
                object_identity_confirmed=False, motion_authorized=False,
                contact_geometry_authorized=False, source_stamp_ns=source_stamp_ns)
