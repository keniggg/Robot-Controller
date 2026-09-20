"""Request-local observation tracking budgets, not hardware certification.

Read-only geometry may consider tighter per-goal tracking tolerances only
when an executor will send those exact tolerances and install a controller
hold after failure. This module does not grant motion authority. It also
reserves the commanded Noetic stopping excursion; it cannot certify servo
overshoot, feedback latency, calibration, or a physical stopping distance.
"""
import math
from collections.abc import Mapping

import numpy as np

from .observation_path_guard import (
    ObservationPathError, _polynomial_bounds, segment_coefficients,
    observation_tracking_error_bounds,
)

POLICY = 'per_goal_tracking_with_commanded_stop_reserve_v1'
MAX_VELOCITY_RAD_S = .08
MAX_ACCELERATION_RAD_S2 = .30
MAX_STOP_DURATION_SEC = .5
SDK_HALF_COUNT_RAD = math.pi / 4096.


def commanded_stop_reserve(stop_duration_sec):
    """Bound Noetic StopTrajectoryBuilder's commanded position excursion.

    Its symmetric quintic on [0, 2*T] has, for s in [0, 1/2], position
    p0 + v0*T*(2*s-4*s**3+2*s**4) + a0*T**2*2*s**2*(1-s)**3.
    The two nonnegative kernels have maxima 5/8 and 216/3125 (< .07).
    This is a command bound under the checked v/a limits, not a servo bound.
    """
    if (isinstance(stop_duration_sec, bool)
            or not isinstance(stop_duration_sec, (int, float))
            or not math.isfinite(stop_duration_sec)
            or not 0. < stop_duration_sec <= MAX_STOP_DURATION_SEC):
        raise ObservationPathError('unsupported controller stop duration')
    t = float(stop_duration_sec)
    return (.625 * MAX_VELOCITY_RAD_S * t
            + .07 * MAX_ACCELERATION_RAD_S2 * t*t + 1e-12)


def tracking_contract_candidates(constraints, names, stop_duration_sec):
    """Finite generic search; never enlarge the configured path tolerance.

    The smallest ordinary candidate is the existing 0.035-rad endpoint
    scale. This is still a PATH tolerance that must be explicitly submitted,
    not the substitution of an endpoint tolerance for a running controller.
    Different scenes/poses use the same sequence, without target offsets.
    """
    names = tuple(names)
    if not names or len(set(names)) != len(names):
        raise ObservationPathError('tracking contract requires distinct joints')
    observation_tracking_error_bounds(constraints, names)  # original validation
    if not isinstance(constraints, Mapping):
        raise ObservationPathError('controller constraints unavailable')
    caps = np.array([constraints[name]['trajectory'] for name in names], dtype=float)
    reserve = commanded_stop_reserve(stop_duration_sec)
    seen, result = set(), []
    for maximum in (.09, .06, .05, .035):
        values = tuple(np.minimum(caps, maximum).tolist())
        if values in seen:
            continue
        seen.add(values)
        result.append(dict(policy=POLICY, joint_names=list(names),
            path_position_tolerance_rad=list(values),
            joint_error_bounds_rad=(np.array(values)+reserve+SDK_HALF_COUNT_RAD).tolist(),
            commanded_stop_reserve_rad=reserve,
            stop_trajectory_duration_sec=float(stop_duration_sec),
            maximum_command_velocity_rad_s=MAX_VELOCITY_RAD_S,
            maximum_command_acceleration_rad_s2=MAX_ACCELERATION_RAD_S2,
            certifies_hardware_tracking_stopping_or_calibration=False))
    return tuple(result)


def command_derivative_bounds(plan, hold):
    """Bound both derivatives on the exact bridge and commanded segments.

    Bernstein enclosures can reject an unresolved curve conservatively; they
    never authorize a curve from a handful of derivative samples.
    Trajectory structure/limits/CAD are checked by the existing path proof.
    """
    points = list(plan.joint_trajectory.points)
    n = len(plan.joint_trajectory.joint_names)
    if not 2 <= len(points) <= 4096:
        raise ObservationPathError('tracking contract requires a bounded trajectory')
    times = [float(p.time_from_start.to_sec()) for p in points]
    if (not all(math.isfinite(t) and t >= 0. for t in times)
            or any(b <= a for a, b in zip(times, times[1:]))):
        raise ObservationPathError('invalid tracking contract trajectory timing')
    first = next((i for i, t in enumerate(times) if t > 0.), None)
    if first is None:
        raise ObservationPathError('no positive-time waypoint')
    pairs = [(hold, points[first], times[first])]
    pairs += [(points[i], points[i+1], times[i+1]-times[i])
              for i in range(first, len(points)-1)]
    peak_v, peak_a = np.zeros(n), np.zeros(n)
    for start, end, duration in pairs:
        c = segment_coefficients(start, end, duration, n)
        velocity = c[:, 1:] * np.arange(1, c.shape[1]) / duration
        acceleration = velocity[:, 1:] * np.arange(1, velocity.shape[1]) / duration
        # Fixed subdivisions tighten the derivative enclosure without any
        # unbounded search. Every subinterval is enclosed, not sampled.
        for lo, hi in zip(np.linspace(0., 1., 17)[:-1], np.linspace(0., 1., 17)[1:]):
            v_lo, v_hi = _polynomial_bounds(velocity, lo, hi)
            a_lo, a_hi = _polynomial_bounds(acceleration, lo, hi)
            peak_v = np.maximum(peak_v, np.maximum(abs(v_lo), abs(v_hi)))
            peak_a = np.maximum(peak_a, np.maximum(abs(a_lo), abs(a_hi)))
    if np.any(~np.isfinite(peak_v)) or np.any(~np.isfinite(peak_a)):
        raise ObservationPathError('nonfinite command derivative bound')
    return dict(maximum_velocity_rad_s=peak_v.tolist(),
                maximum_acceleration_rad_s2=peak_a.tolist())


def validate_command_derivatives(plan, hold):
    report = command_derivative_bounds(plan, hold)
    if (max(report['maximum_velocity_rad_s']) > MAX_VELOCITY_RAD_S
            or max(report['maximum_acceleration_rad_s2']) > MAX_ACCELERATION_RAD_S2):
        raise ObservationPathError('command derivatives exceed stopping-envelope contract: %s' % report)
    return report


def required_command_time_scale(plan, hold):
    report = command_derivative_bounds(plan, hold)
    return max(1., max(report['maximum_velocity_rad_s']) / MAX_VELOCITY_RAD_S,
               math.sqrt(max(report['maximum_acceleration_rad_s2']) / MAX_ACCELERATION_RAD_S2))


def qualify_contract_path(plan, hold, scene, fk, gripper, opening, constraints, stop_duration):
    """Search the same finite tracking contracts for every pose/target.

    A returned candidate is still only a conditional geometric certificate.
    Execution must bind its exact tolerances, final path, model, scene and
    stationary reference separately, and preserve manual control priority.
    """
    from .observation_preview import qualify_candidate_path
    derivatives = validate_command_derivatives(plan, hold)
    attempts = []
    for contract in tracking_contract_candidates(constraints, fk.names, stop_duration):
        report = qualify_candidate_path(plan, hold, scene, fk, gripper, opening,
                                        contract['joint_error_bounds_rad'])
        attempts.append(dict(path_position_tolerance_rad=contract['path_position_tolerance_rad'],
                             ok=report['ok'], failure_location=report.get('failure_location'),
                             reason=report.get('reason', '')))
        if report['ok']:
            report['execution_tracking_contract'] = contract
            report['command_derivative_bounds'] = derivatives
            report['tracking_contract_search'] = attempts
            return report
    report['tracking_contract_search'] = attempts
    return report
