"""Pure, bounded sampled camera visibility checks for observation trajectories.

Samples the actual Noetic polynomial and stationary desired-state bridge. This
is NOT a continuous visibility proof, an occlusion test, a collision check, or
an assurance about tracking/calibration. All existing motion checks still apply.
"""
from copy import deepcopy
import math
import time

import numpy as np

from alicia_flexible_grasp.grasp.rich_plan_integrity import stamp_nanoseconds
from alicia_flexible_grasp.robot.observation_path_guard import (
    ObservationPathError, SerialUrdfFk, segment_coefficients, _polynomial,
    _polynomial_bounds, wire_digest,
)


class ObservationViewError(ObservationPathError):
    def __init__(self, reason, audit=None):
        self.audit = deepcopy(audit or {})
        self.audit.update(ok=False, reason=str(reason))
        super().__init__(str(reason))


def _transform(value, label):
    value = np.asarray(value, dtype=float)
    if (value.shape != (4, 4) or not np.all(np.isfinite(value))
            or not np.allclose(value[3], [0., 0., 0., 1.], atol=1e-9, rtol=0.)
            or not np.allclose(value[:3, :3].T @ value[:3, :3], np.eye(3), atol=1e-6, rtol=0.)
            or abs(float(np.linalg.det(value[:3, :3]))-1.) > 1e-6):
        raise ObservationViewError(label+' must be a finite rigid transform')
    return value


def _camera(intrinsics, width, height, margin, near, far):
    if (isinstance(width, bool) or isinstance(height, bool)
            or not isinstance(width, (int, np.integer)) or not isinstance(height, (int, np.integer))
            or not 1 <= width <= 16384 or not 1 <= height <= 16384):
        raise ObservationViewError('invalid camera image dimensions')
    k = np.asarray(intrinsics, dtype=float)
    if k.shape == (3, 3):
        if not np.allclose(k, [[k[0, 0], 0., k[0, 2]], [0., k[1, 1], k[1, 2]], [0., 0., 1.]], atol=1e-9, rtol=0.):
            raise ObservationViewError('unsupported camera intrinsics')
        k = k[[0, 1, 0, 1], [0, 1, 2, 2]]
    if (k.shape != (4,) or not np.all(np.isfinite(k)) or np.any(k[:2] <= 0.)
            or not 0. <= k[2] < width or not 0. <= k[3] < height):
        raise ObservationViewError('invalid camera intrinsics')
    if (not all(math.isfinite(float(v)) for v in (margin, near, far))
            or not 0. <= margin < (min(width, height)-1.)/2.
            or not 0. < near < far):
        raise ObservationViewError('invalid camera visibility limits')
    return k


def _corners(value):
    result = np.asarray(value, dtype=float)
    if (result.shape != (8, 3) or not np.all(np.isfinite(result))
            or np.linalg.matrix_rank(result-result.mean(axis=0)) != 3):
        raise ObservationViewError('target OBB requires eight finite nondegenerate corners')
    return result


def _project(points, transform, k):
    # ROS camera_link is x forward, y left, z up (not optical z forward).
    camera = (points-transform[:3, 3]) @ transform[:3, :3]
    depth = camera[:, 0]
    with np.errstate(divide='ignore', invalid='ignore', over='ignore'):
        uv = np.column_stack((k[2]-k[0]*camera[:, 1]/depth,
                              k[3]-k[1]*camera[:, 2]/depth))
    return uv, depth


def _target_projection(corners, transform, k, width, height, margin, near, far):
    uv, depth = _project(corners, transform, k)
    if (not np.all(np.isfinite(uv)) or not np.all(np.isfinite(depth))
            or np.min(depth) < near or np.max(depth) > far):
        raise ObservationViewError('target OBB exceeds camera depth range', {
            'minimum_depth_m': float(np.min(depth)) if np.all(np.isfinite(depth)) else None,
            'maximum_depth_m': float(np.max(depth)) if np.all(np.isfinite(depth)) else None})
    minimum = float(np.min(np.column_stack((uv[:, 0], width-1.-uv[:, 0],
                                           uv[:, 1], height-1.-uv[:, 1]))))
    report = {'minimum_image_edge_distance_px': minimum,
              'target_uv_min_px': np.min(uv, axis=0).tolist(),
              'target_uv_max_px': np.max(uv, axis=0).tolist(),
              'minimum_depth_m': float(np.min(depth)), 'maximum_depth_m': float(np.max(depth))}
    if minimum < margin:
        raise ObservationViewError('target OBB leaves required camera image margin', report)
    return report


def validate_observation_target_trajectory(
        plan, fk, tool_from_camera, target_corners_base, intrinsics,
        image_width, image_height, controller_desired, *, roi_margin_px=36.,
        minimum_depth_m=.070, maximum_depth_m=.500, max_time_step_sec=.05,
        max_joint_span_rad=.02, max_samples=16384, max_depth=20,
        max_seconds=5., clock=time.monotonic):
    """Reject any sampled whole-OBB visibility loss along actual command motion.

    Both maximum interval time and polynomial joint span bound subdivision.
    A finite sampling budget is never treated as a successful visibility test.
    The returned endpoint transform is only for further independent checks.
    """
    audit = {'kind': 'bounded_sampled_observation_target_visibility', 'ok': False,
             'certifies_continuous_visibility': False,
             'certifies_collision_tracking_calibration_or_occlusion': False,
             'endpoint_support_checked': False, 'sample_count': 0}
    started = clock()
    try:
        if (not isinstance(fk, SerialUrdfFk)
                or not all(math.isfinite(float(v)) and float(v) > 0.
                           for v in (max_time_step_sec, max_joint_span_rad, max_seconds))
                or isinstance(max_samples, bool) or not isinstance(max_samples, int) or not 1 <= max_samples <= 1000000
                or isinstance(max_depth, bool) or not isinstance(max_depth, int) or not 0 <= max_depth <= 30):
            raise ObservationViewError('invalid camera trajectory model or sampling budget')
        k = _camera(intrinsics, image_width, image_height, roi_margin_px, minimum_depth_m, maximum_depth_m)
        camera_mount = _transform(tool_from_camera, 'tool_from_camera')
        corners = _corners(target_corners_base)
        trajectory = plan.joint_trajectory
        names = tuple(trajectory.joint_names)
        if names != fk.names or len(set(names)) != len(names):
            raise ObservationViewError('trajectory/FK joint order mismatch')
        if stamp_nanoseconds(trajectory.header.stamp) != 0:
            raise ObservationViewError('nonzero trajectory start stamp is unsupported')
        if getattr(getattr(plan, 'multi_dof_joint_trajectory', None), 'points', ()):
            raise ObservationViewError('multi-DOF observation path is unsupported')
        points = list(trajectory.points)
        if not 2 <= len(points) <= 4096:
            raise ObservationViewError('observation trajectory must contain 2..4096 waypoints')
        times = [float(p.time_from_start.to_sec()) for p in points]
        if (not all(math.isfinite(t) and t >= 0. for t in times)
                or any(b <= a for a, b in zip(times, times[1:]))):
            raise ObservationViewError('observation trajectory timing is invalid')
        for field in ('positions', 'velocities', 'accelerations'):
            values = np.asarray(getattr(controller_desired, field, ()), dtype=float)
            if (values.shape != (len(names),) or not np.all(np.isfinite(values))
                    or (field != 'positions' and np.any(values != 0.))):
                raise ObservationViewError('controller desired state must be a complete stationary hold')
        first = next((i for i, t in enumerate(times) if t > 0.), None)
        if first is None:
            raise ObservationViewError('no positive-time controller waypoint')
        segments = [(segment_coefficients(controller_desired, points[first], times[first], len(names)),
                     'bridge', 0., times[first])]
        if first:
            segment_coefficients(points[0], points[first], times[first], len(names))
        for i in range(first, len(points)-1):
            duration = times[i+1]-times[i]
            segments.append((segment_coefficients(points[i], points[i+1], duration, len(names)),
                             str(i), times[i], duration))
        audit.update(trajectory_sha256=wire_digest(plan), robot_description_sha256=fk.model_sha256,
                     joint_names=list(names), bridge='stationary_desired_to_first_positive_time_waypoint',
                     bridge_start_joint_positions_rad=list(controller_desired.positions),
                     required_image_margin_px=float(roi_margin_px),
                     minimum_camera_depth_m=float(minimum_depth_m), maximum_camera_depth_m=float(maximum_depth_m),
                     configured_max_time_step_sec=float(max_time_step_sec),
                     configured_max_joint_span_rad=float(max_joint_span_rad),
                     maximum_sample_interval_sec=0., maximum_leaf_joint_span_rad=0.,
                     minimum_sampled_image_edge_distance_px=None)
        endpoint = None
        for coefficients, label, segment_start, duration in segments:
            seen = set()
            stack = [(0., 1., 0)]
            while stack:
                if clock()-started > max_seconds:
                    raise ObservationViewError('camera trajectory time budget exhausted')
                lo, hi, level = stack.pop()
                midpoint = (lo+hi)*.5
                for s in (lo, midpoint, hi):
                    if s in seen:
                        continue
                    if audit['sample_count'] >= max_samples:
                        raise ObservationViewError('camera trajectory sample budget exhausted')
                    seen.add(s)
                    joints = _polynomial(coefficients, s)
                    audit['sample_count'] += 1
                    try:
                        transform = _transform(fk(joints) @ camera_mount, 'base_from_camera')
                        view = _target_projection(corners, transform, k, image_width, image_height,
                                                  roi_margin_px, minimum_depth_m, maximum_depth_m)
                    except (ObservationPathError, ValueError) as exc:
                        detail = dict(getattr(exc, 'audit', {}))
                        detail.update(segment=label, trajectory_time_sec=float(segment_start+s*duration),
                                      normalized_segment_time=float(s), joint_positions_rad=joints.tolist())
                        raise ObservationViewError(str(exc), detail) from exc
                    previous_minimum = audit['minimum_sampled_image_edge_distance_px']
                    audit['minimum_sampled_image_edge_distance_px'] = (view['minimum_image_edge_distance_px']
                        if previous_minimum is None else min(previous_minimum, view['minimum_image_edge_distance_px']))
                    if label == segments[-1][1] and s == 1.:
                        endpoint = transform
                lower, upper = _polynomial_bounds(coefficients, lo, hi)
                span = float(np.max(upper-lower))
                elapsed = float((hi-lo)*duration)
                if not math.isfinite(span):
                    raise ObservationViewError('nonfinite camera trajectory joint enclosure')
                if span <= max_joint_span_rad and elapsed <= max_time_step_sec:
                    # Lo/mid/hi have been checked, so actual sample spacing <= half the leaf duration.
                    audit['maximum_sample_interval_sec'] = max(audit['maximum_sample_interval_sec'], elapsed*.5)
                    audit['maximum_leaf_joint_span_rad'] = max(audit['maximum_leaf_joint_span_rad'], span)
                else:
                    if level >= max_depth:
                        raise ObservationViewError('camera trajectory subdivision budget exhausted')
                    stack.extend(((midpoint, hi, level+1), (lo, midpoint, level+1)))
        if clock()-started > max_seconds:
            raise ObservationViewError('camera trajectory time budget exhausted')
        audit.update(ok=True, endpoint_transform_base_camera=endpoint.tolist(), elapsed_sec=float(clock()-started))
        return audit
    except (AttributeError, TypeError, ValueError, OverflowError) as exc:
        detail = dict(audit)
        detail.update(getattr(exc, 'audit', {}))
        detail['elapsed_sec'] = float(clock()-started)
        raise ObservationViewError(str(exc), detail) from exc


def validate_observation_support_projection(
        transform_base_camera, measured_support_points_base, target_center_base,
        intrinsics, image_width, image_height, *, roi_margin_px=36.,
        minimum_depth_m=.070, maximum_depth_m=.500, target_corners_base=None):
    """Predict endpoint support coverage from actual measured points only.

    Keep the existing 70..170 px scaled ring, >=120 unique support pixels,
    and >=3 quadrants with >=20 pixels each. No unseen plane is synthesized.
    This projection cannot replace arrival-time measured support/plane gates.
    """
    audit = {'kind': 'measured_support_endpoint_projection', 'ok': False,
             'certifies_new_depth_or_occlusion': False,
             'requires_arrival_measured_support_check': True,
             'minimum_support_pixels': 120, 'minimum_quadrants': 3,
             'minimum_pixels_per_quadrant': 20}
    try:
        k = _camera(intrinsics, image_width, image_height, roi_margin_px, minimum_depth_m, maximum_depth_m)
        transform = _transform(transform_base_camera, 'base_from_camera')
        if measured_support_points_base is None:
            raise ObservationViewError('measured support points unavailable')
        points = np.asarray(measured_support_points_base, dtype=float)
        if (points.ndim != 2 or points.shape[1:] != (3,) or not 1 <= len(points) <= 2000000
                or not np.all(np.isfinite(points))):
            raise ObservationViewError('invalid or empty measured support points')
        center = np.asarray(target_center_base, dtype=float)
        if center.shape != (3,) or not np.all(np.isfinite(center)):
            raise ObservationViewError('invalid frozen target center')
        center_uv, center_depth = _project(center[None], transform, k)
        anchor = center_uv[0]
        if (not np.all(np.isfinite(anchor)) or not minimum_depth_m <= center_depth[0] <= maximum_depth_m
                or not roi_margin_px <= anchor[0] <= image_width-1.-roi_margin_px
                or not roi_margin_px <= anchor[1] <= image_height-1.-roi_margin_px):
            raise ObservationViewError('support target center outside camera view')
        uv, depth = _project(points, transform, k)
        valid = (np.all(np.isfinite(uv), axis=1) & (depth >= minimum_depth_m) & (depth <= maximum_depth_m)
                 & (uv[:, 0] >= roi_margin_px) & (uv[:, 0] <= image_width-1.-roi_margin_px)
                 & (uv[:, 1] >= roi_margin_px) & (uv[:, 1] <= image_height-1.-roi_margin_px))
        if target_corners_base is not None:
            corners = _corners(target_corners_base)
            _target_projection(corners, transform, k, image_width, image_height,
                               roi_margin_px, minimum_depth_m, maximum_depth_m)
            target_uv, _ = _project(corners, transform, k)
            lo, hi = target_uv.min(axis=0), target_uv.max(axis=0)
            valid &= ~np.all((uv >= lo) & (uv <= hi), axis=1)
        pixels = np.unique(np.rint(uv[valid]).astype(np.int64), axis=0)
        delta = pixels-anchor
        scale = min(float(image_width)/640., float(image_height)/480.)
        radius = np.linalg.norm(delta, axis=1)
        delta = delta[(radius >= 70.*scale) & (radius <= 170.*scale)]
        quadrants = [int(np.sum((delta[:, 0] < 0.) & (delta[:, 1] < 0.))),
                     int(np.sum((delta[:, 0] < 0.) & (delta[:, 1] >= 0.))),
                     int(np.sum((delta[:, 0] >= 0.) & (delta[:, 1] < 0.))),
                     int(np.sum((delta[:, 0] >= 0.) & (delta[:, 1] >= 0.)))]
        audit.update(measured_input_points=int(len(points)), support_pixels=int(len(delta)),
                     quadrant_pixels=dict(zip(('left_top', 'left_bottom', 'right_top', 'right_bottom'), quadrants)),
                     qualifying_quadrants=int(sum(n >= 20 for n in quadrants)),
                     target_center_uv_px=anchor.tolist(), ring_inner_px=70.*scale, ring_outer_px=170.*scale,
                     excludes_projected_target_box=target_corners_base is not None)
        if len(delta) < 120:
            raise ObservationViewError('insufficient projected measured support pixels')
        if audit['qualifying_quadrants'] < 3:
            raise ObservationViewError('insufficient projected measured support spatial coverage')
        audit['ok'] = True
        return audit
    except (AttributeError, TypeError, ValueError, OverflowError) as exc:
        audit.update(getattr(exc, 'audit', {}))
        raise ObservationViewError(str(exc), audit) from exc


def validate_observation_view_trajectory(
        plan, fk, tool_from_camera, target_corners_base, target_center_base,
        measured_support_points_base, intrinsics, image_width, image_height,
        controller_desired, **kwargs):
    """Compose whole-path target and endpoint measured-support projection checks."""
    target = validate_observation_target_trajectory(
        plan, fk, tool_from_camera, target_corners_base, intrinsics, image_width,
        image_height, controller_desired, **kwargs)
    view_limits = {key: kwargs[key] for key in ('roi_margin_px', 'minimum_depth_m', 'maximum_depth_m') if key in kwargs}
    try:
        support = validate_observation_support_projection(
            target['endpoint_transform_base_camera'], measured_support_points_base,
            target_center_base, intrinsics, image_width, image_height,
            target_corners_base=target_corners_base, **view_limits)
    except ObservationViewError as exc:
        raise ObservationViewError(str(exc), {'target_trajectory': target, 'endpoint_support': exc.audit}) from exc
    return {'kind': 'bounded_sampled_observation_view_and_measured_support', 'ok': True,
            'target_trajectory': target, 'endpoint_support': support,
            'certifies_continuous_visibility_or_collision': False}
