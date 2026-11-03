"""Optional timing refinement of a monotone joint-space line, never authority.

Original waypoints remain exact and ordered. A two-point line may gain two
interior ramp knots; the resulting complete controller spline must remain on
the same line without reversal. Existing reference, speed, stopping, tracking,
and CAD validators still validate the final submitted trajectory.
"""
from copy import deepcopy

import numpy as np

from .observation_path_guard import segment_coefficients, _polynomial_bounds
from .observation_timing import smooth_collinear_observation


def _line(plan):
    tr = plan.joint_trajectory
    names = tuple(tr.joint_names)
    if (len(names) != 6 or len(set(names)) != 6
            or not 2 <= len(tr.points) <= 4096
            or getattr(plan.multi_dof_joint_trajectory, 'points', ())):
        return None
    q = np.asarray([p.positions for p in tr.points], dtype=float)
    if q.shape != (len(tr.points), 6) or not np.all(np.isfinite(q)):
        return None
    delta = q[-1]-q[0]
    axis = int(np.argmax(abs(delta)))
    if abs(delta[axis]) < 1e-10:
        return None
    alpha = (q[:, axis]-q[0, axis])/delta[axis]
    if (np.any(np.diff(alpha) <= 1e-9)
            or np.max(abs(q-(q[0]+alpha[:, None]*delta))) > 1e-12):
        return None
    return names, q, delta, axis, alpha


def validate_collinear_refinement(original, candidate):
    """Prove waypoint retention plus continuous, monotone line membership.

    This does not relax the general planner's point-for-point geometry check.
    It is used only for this explicit timing profile, before the unchanged
    final trajectory proof. Derivative checks enclose every polynomial interval.
    """
    old, new = _line(original), _line(candidate)
    if old is None or new is None or old[0] != new[0]:
        raise ValueError('STRICT_TIMING_NOT_A_COMMON_MONOTONE_LINE')
    names, q, delta, axis, _ = old
    nq = new[1]
    if not np.array_equal(q[0], nq[0]) or not np.array_equal(q[-1], nq[-1]):
        raise ValueError('STRICT_TIMING_ENDPOINT_CHANGED')
    indices, cursor = [], 0
    for point in q:
        while cursor < len(nq) and not np.array_equal(point, nq[cursor]):
            cursor += 1
        if cursor == len(nq):
            raise ValueError('STRICT_TIMING_ORIGINAL_WAYPOINT_MISSING')
        indices.append(cursor)
        cursor += 1
    alpha = (nq[:, axis]-q[0, axis])/delta[axis]
    if np.max(abs(nq-(q[0]+alpha[:, None]*delta))) > 1e-12:
        raise ValueError('STRICT_TIMING_REFINEMENT_LEAVES_ORIGINAL_LINE')
    points = candidate.joint_trajectory.points
    for a, b in zip(points, points[1:]):
        duration = (b.time_from_start-a.time_from_start).to_sec()
        if duration <= 0.:
            raise ValueError('STRICT_TIMING_DURATION_INVALID')
        coefficients = segment_coefficients(a, b, duration, 6)
        scalar = coefficients[axis].copy()
        scalar[0] -= q[0, axis]
        scalar /= delta[axis]
        expected = delta[:, None]*scalar
        expected[:, 0] += q[0]
        if np.max(abs(coefficients-expected)) > 1e-11:
            raise ValueError('STRICT_TIMING_SPLINE_LEAVES_ORIGINAL_LINE')
        derivative = scalar[1:]*np.arange(1, len(scalar))
        # The generated C2 ramps have nonnegative scalar speed. A position
        # sample or endpoint check alone cannot certify an arbitrary spline.
        for lo, hi in zip(np.linspace(0., 1., 65)[:-1], np.linspace(0., 1., 65)[1:]):
            lower, _ = _polynomial_bounds(derivative[None, :], lo, hi)
            if lower[0] < -1e-10:
                raise ValueError('STRICT_TIMING_SPLINE_REVERSES')
    return dict(policy='retained_waypoints_monotone_collinear_C2_timing',
                original_points=len(q), refined_points=len(nq),
                original_waypoint_indices=indices, physical_tracking_verified=False)


def smooth_strict_collinear(plan, velocity_limits, minimum_duration=3.):
    """Propose faster ramp/cruise timing under the original per-joint limits.

    Nonlinear/reversing paths are ineligible and keep their original retimer.
    Invalid generated refinements raise; they must not hide a safety rejection.
    """
    line = _line(plan)
    if line is None:
        return None
    seed = deepcopy(plan)
    if len(seed.joint_trajectory.points) == 2:
        first, last = seed.joint_trajectory.points
        q, delta = line[1][0], line[2]
        interior = []
        for alpha in (.05, .95):
            point = deepcopy(first)
            point.positions = (q+alpha*delta).tolist()
            interior.append(point)
        seed.joint_trajectory.points = [first, *interior, last]
    result = smooth_collinear_observation(seed, velocity_limits,
        minimum_duration=minimum_duration)
    if result is None:
        return None
    validate_collinear_refinement(plan, result)
    return result
