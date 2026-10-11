"""Unknown-object observation preferences shared by planner and executor.

Working distance is a view preference, not contact clearance. All original
measured visibility, CAD, tracking, IK and final execution gates still apply.
Carton and current-view direct planning retain their original configuration.
"""

from alicia_flexible_grasp.robot.observation_view_limits import MINIMUM_OBSERVATION_IMAGE_MARGIN_PX
from alicia_flexible_grasp.robot.observation_view_limits import effective_observation_view_limits

WORKING_VOLUME_POLICY = 'rgbd_working_volume_v1'


def observation_config(config, selection):
    if (selection or {}).get('mode') != 'unknown' or (selection or {}).get('strategy') not in ('two_stage', 'observe_return'):
        return config
    # These values only construct candidate views. Actual arrival uses all
    # target corners and FOV at the image source time, never a radial band.
    limits = effective_observation_view_limits(config)
    return dict(config,
                observation_acceptance_policy=WORKING_VOLUME_POLICY,
                observation_camera_target_nominal_distance_m=0.26,
                observation_camera_target_min_distance_m=limits['minimum_depth_m'],
                observation_camera_target_max_distance_m=limits['maximum_depth_m'])


def observation_distance_preferences(corners_base, center_base, camera_rotation,
                                     current_camera_position, intrinsics, width, height,
                                     *, roi_margin_px=MINIMUM_OBSERVATION_IMAGE_MARGIN_PX, minimum_depth_m=.070,
                                     maximum_depth_m=.500):
    """Propose at most five centred views from this orientation's full FOV.

    The feasible interval depends on the target's corners and camera, not an
    object category or fixed near-field band. These are proposals only; strict
    trajectory, measured image and information-gain checks remain mandatory.
    """
    import math
    import numpy as np
    from numbers import Real, Integral

    def array(value, shape):
        raw = np.asarray(value, dtype=object)
        if raw.shape != shape or any(isinstance(x, (bool, np.bool_)) or
                not isinstance(x, Real) for x in raw.flat):
            raise ValueError('invalid observation distance geometry')
        result = np.asarray(value, dtype=float)
        if not np.all(np.isfinite(result)):
            raise ValueError('nonfinite observation distance geometry')
        return result

    corners = array(corners_base, (8, 3)); center = array(center_base, (3,))
    rotation = array(camera_rotation, (3, 3)); position = array(current_camera_position, (3,))
    k = array(intrinsics, (4,))
    if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6, rtol=0.)
            or abs(float(np.linalg.det(rotation))-1.) > 1e-6):
        raise ValueError('observation camera rotation is not rigid')
    if any(isinstance(v, (bool, np.bool_)) or not isinstance(v, Integral)
           or not 1 <= v <= 16384 for v in (width, height)):
        raise ValueError('invalid observation image dimensions')
    limits = effective_observation_view_limits(dict(camera_visibility_margin_px=roi_margin_px,
        camera_visibility_min_depth_m=minimum_depth_m, camera_visibility_max_depth_m=maximum_depth_m))
    margin = limits['roi_margin_px']; fx, fy, cx, cy = k
    edges = np.array([cx-margin, width-1.-margin-cx, cy-margin, height-1.-margin-cy])
    if fx <= 0. or fy <= 0. or np.any(edges <= 0.):
        raise ValueError('camera principal point must lie inside observation ROI')
    offsets = (corners-center) @ rotation
    x, y, z = offsets.T
    lower = float(np.max(np.maximum.reduce([
        minimum_depth_m-x,
        np.where(y >= 0., fx*y/edges[0], -fx*y/edges[1])-x,
        np.where(z >= 0., fy*z/edges[2], -fy*z/edges[3])-x])))
    lower = max(minimum_depth_m, lower)
    upper = min(maximum_depth_m, float(np.min(maximum_depth_m-x)))
    if not math.isfinite(lower+upper) or upper <= lower:
        return ()
    # Interior proposals avoid floating-point boundary churn. Arrival is never
    # judged against this proposal interval or forced to its midpoint.
    inset = min(1e-6, (upper-lower)*.01)
    lower += inset; upper -= inset
    current = float(np.linalg.norm(center-position))
    preferred = float(np.clip(current, lower, upper))
    values = (preferred, lower+(upper-lower)*.25, (lower+upper)*.5,
              lower+(upper-lower)*.75, lower if preferred > (lower+upper)*.5 else upper)
    result = []
    for value in values:
        if not any(abs(value-prior) <= 1e-9 for prior in result):
            result.append(value)
    return tuple(result)
