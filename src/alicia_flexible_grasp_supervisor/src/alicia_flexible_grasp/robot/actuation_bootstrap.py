"""A bounded prefix of an already collision-checked observation trajectory."""
from copy import deepcopy
import math
from types import SimpleNamespace

import numpy as np

from .observation_path_guard import segment_coefficients, _polynomial, _polynomial_bounds


PENDING_BOOTSTRAP_STATES = frozenset((
    'PENDING:POSITIVE_ENABLE_REQUESTED', 'PENDING:COMMAND_SYNCHRONIZED',
))


def _positions(values, label):
    result = np.asarray(values, dtype=float)
    if result.shape != (6,) or not np.all(np.isfinite(result)):
        raise ValueError('bootstrap %s must contain six finite positions' % label)
    return result


def _origins(points, controller_reference, measured_origin):
    measured = _positions(points[0].positions if measured_origin is None else measured_origin,
                          'measured origin')
    if controller_reference is None:
        controller_reference = SimpleNamespace(positions=measured, velocities=[0.]*6,
                                               accelerations=[0.]*6)
    desired = _positions(controller_reference.positions, 'controller reference')
    for field in ('velocities', 'accelerations'):
        if np.any(_positions(getattr(controller_reference, field), field) != 0.):
            raise ValueError('bootstrap controller reference must be stationary')
    return measured, desired, controller_reference


def _bounds(coefficients, end=1.):
    # Bound the continuous polynomial, including extrema between waypoints.
    intervals = np.linspace(0., end, 9)
    bounds = [_polynomial_bounds(coefficients, a, b)
              for a, b in zip(intervals, intervals[1:])]
    return np.min([v[0] for v in bounds], axis=0), np.max([v[1] for v in bounds], axis=0)


def _inside(coefficients, lower, upper, end=1.):
    lo, hi = _bounds(coefficients, end)
    return np.all(lo >= lower) and np.all(hi <= upper)


def _require_response(endpoint, start, desired):
    command = endpoint-desired
    progress = endpoint-start
    # A preexisting desired/feedback discrepancy is not directional motion.
    if not np.any((np.abs(command) >= .023-1e-9) & (command*progress > 0.)):
        raise ValueError('observation prefix has insufficient directional command delta')


def bounded_observation_prefix(plan, max_delta_rad=0.025, *,
                               controller_reference=None, measured_origin=None):
    """Propose a prefix on the source curve inside both stationary origins.

    This is geometry for mandatory retiming, not execution permission. Source
    spline extrema determine the first cut; retiming and the real controller
    bridge must subsequently pass validate_bounded_observation_prefix and the
    existing velocity, scene, tracking and stopping gates.
    """
    if isinstance(max_delta_rad, bool) or not math.isfinite(max_delta_rad) or not 0.023 <= max_delta_rad <= 0.025:
        raise ValueError('bootstrap joint bound must be within [0.023, 0.025] rad')
    result = deepcopy(plan)
    trajectory = result.joint_trajectory
    points = list(trajectory.points)
    if len(trajectory.joint_names) != 6 or len(set(trajectory.joint_names)) != 6:
        raise ValueError('bootstrap requires exactly six distinct arm joints')
    if not 2 <= len(points) <= 4096 or getattr(getattr(plan, 'multi_dof_joint_trajectory', None), 'points', []):
        raise ValueError('bootstrap requires a nonempty arm-only trajectory')
    for point in points:
        if len(point.positions) != 6 or not all(math.isfinite(v) for v in point.positions):
            raise ValueError('bootstrap trajectory positions are incomplete or nonfinite')
    origin = _positions(points[0].positions, 'planned origin')
    measured, desired, _ = _origins(points, controller_reference, measured_origin)
    lower = np.maximum(measured, desired)-max_delta_rad
    upper = np.minimum(measured, desired)+max_delta_rad
    if (np.any(np.abs(measured-desired) > max_delta_rad)
            or np.any(origin < lower) or np.any(origin > upper)):
        raise ValueError('bootstrap origins or controller bridge exceed joint bound')
    times = [float(point.time_from_start.to_sec()) for point in points]
    if (not all(math.isfinite(t) for t in times) or times[0] < 0.
            or any(b <= a for a, b in zip(times, times[1:]))):
        raise ValueError('bootstrap trajectory times are invalid')
    prefix = [points[0]]
    curves = []
    for index, point in enumerate(points[1:]):
        coefficients = segment_coefficients(points[index], point, times[index+1]-times[index], 6)
        if _inside(coefficients, lower-1e-12, upper+1e-12):
            prefix.append(point)
            curves.append((coefficients, 1.))
            continue
        # Prefix-enclosure bisection cannot jump across an out-and-back spline
        # excursion. The tiny inward margin is numerical, never extra motion.
        lo, hi = 0., 1.
        for _ in range(48):
            middle = (lo+hi)/2.
            if _inside(coefficients, lower+1e-10, upper-1e-10, middle):
                lo = middle
            else:
                hi = middle
        if lo <= 1e-12:
            raise ValueError('bootstrap prefix boundary is invalid')
        point.positions = _polynomial(coefficients, lo).tolist()
        prefix.append(point)
        curves.append((coefficients, lo))
        break
    endpoint = np.asarray(prefix[-1].positions)
    _require_response(endpoint, measured, desired)
    direction = np.sign(endpoint-origin)
    for coefficients, end in curves:
        derivative = coefficients[:, 1:]*np.arange(1, coefficients.shape[1])
        low, high = _bounds(derivative, end)
        if (np.any(np.where(direction >= 0., low, -high) < -1e-12)
                or np.any((direction == 0.) & (np.maximum(abs(low), abs(high)) > 1e-12))):
            raise ValueError('observation prefix reverses a joint')
    for point in prefix:
        point.velocities = []
        point.accelerations = []
        point.effort = []
    trajectory.points = prefix
    return result


def validate_bounded_observation_prefix(plan, controller_reference, measured_origin):
    """Prove final command splines and actual desired bridge against both origins."""
    points = list(plan.joint_trajectory.points)
    names = plan.joint_trajectory.joint_names
    if (not 2 <= len(points) <= 4096 or len(names) != 6 or len(set(names)) != 6
            or getattr(getattr(plan, 'multi_dof_joint_trajectory', None), 'points', [])):
        raise ValueError('bootstrap final trajectory is incomplete')
    measured, desired, reference = _origins(points, controller_reference, measured_origin)
    times = [float(point.time_from_start.to_sec()) for point in points]
    if (not all(math.isfinite(t) for t in times) or times[0] < 0.
            or any(b <= a for a, b in zip(times, times[1:]))):
        raise ValueError('bootstrap final trajectory times are invalid')
    first_positive = next((p for p, t in zip(points, times) if t > 0.), None)
    if first_positive is None:
        raise ValueError('bootstrap final bridge is missing')
    segments = [(reference, first_positive, first_positive.time_from_start.to_sec())]
    segments += [(a, b, tb-ta) for a, b, ta, tb in zip(points, points[1:], times, times[1:])]
    lower, upper = np.maximum(measured, desired)-.025, np.minimum(measured, desired)+.025
    for first, second, duration in segments:
        if not _inside(segment_coefficients(first, second, duration, 6), lower-1e-12, upper+1e-12):
            raise ValueError('bootstrap final continuous curve exceeds joint bound')
    _require_response(_positions(points[-1].positions, 'endpoint'), measured, desired)
