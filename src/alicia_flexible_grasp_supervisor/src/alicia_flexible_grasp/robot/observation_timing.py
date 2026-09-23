"""Timing preference only; continuous velocity/CAD proofs remain authoritative."""
import json
import math


def observation_timing(selection, config, baseline):
    """Return a mode-bound cache key and scalings, preserving legacy defaults."""
    if not isinstance(config, dict) or config.get('enabled') is not True:
        return ('baseline',), tuple(baseline)
    if isinstance(selection, str):
        selection = json.loads(selection)
    if selection is None:
        return ('baseline',), tuple(baseline)
    if not isinstance(selection, dict):
        raise ValueError('invalid observation mode selection')
    if selection.get('mode') == 'carton':
        return ('baseline',), tuple(baseline)
    if selection.get('mode') != 'unknown':
        raise ValueError('invalid observation mode')
    generation, stamp = selection.get('generation'), selection.get('stamp_ns')
    if isinstance(stamp, str) and stamp.isdecimal():
        stamp = int(stamp)
    strategy = selection.get('strategy', 'two_stage')
    if (type(generation) is not int or generation < 1
            or type(stamp) is not int or stamp < 1
            or strategy not in ('direct', 'two_stage')):
        raise ValueError('invalid observation mode generation')
    velocity = config.get('velocity_scaling', .30)
    acceleration = config.get('acceleration_scaling', .20)
    for value, upper in ((velocity, .32), (acceleration, .30)):
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or not .01 <= value <= upper):
            raise ValueError('invalid unknown observation timing scaling')
    values = (float(velocity), float(acceleration))
    return ('unknown', strategy, generation, stamp, *values), values


def smooth_collinear_observation(plan, velocity_limits, acceleration_limit=.15,
                                 minimum_duration=3.):
    """Preserve every position; retime a monotone line with C2 quartic ramps.

    The controller's quintic interpolant represents each quartic ramp exactly.
    Each ramp's velocity is cubic smoothstep, with zero acceleration at both
    ends. Cruise knots have constant velocity. Non-collinear paths return None.
    Full continuous reference/CAD validation still follows this proposal.
    """
    from copy import deepcopy
    import numpy as np
    import rospy
    points = list(plan.joint_trajectory.points)
    names = list(plan.joint_trajectory.joint_names)
    if len(points) < 3 or len(names) != 6:
        return None
    q = np.asarray([p.positions for p in points], dtype=float)
    if q.shape != (len(points), len(names)) or not np.all(np.isfinite(q)):
        return None
    delta = q[-1] - q[0]
    axis = int(np.argmax(abs(delta)))
    if abs(delta[axis]) < 1e-10:
        return None
    alpha = (q[:, axis] - q[0, axis]) / delta[axis]
    if (np.any(np.diff(alpha) <= 1e-9)
            or np.max(abs(q - (q[0] + alpha[:, None]*delta))) > 1e-12):
        return None
    limits = np.asarray([velocity_limits[n] for n in names], dtype=float)
    if (not np.all(np.isfinite(limits)) or np.any(limits <= 0.)
            or not math.isfinite(acceleration_limit) or acceleration_limit <= 0.
            or not math.isfinite(minimum_duration) or minimum_duration < 0.):
        raise ValueError('invalid smooth observation timing limits')
    moving = abs(delta) > 1e-12
    ramp_span = min(float(alpha[1]), float(1. - alpha[-2]))
    # Peak ramp acceleration is .75*abs(delta)*rate**2/ramp_span.
    rate = .95 * min(float(np.min(limits[moving]/abs(delta[moving]))),
                    math.sqrt(acceleration_limit*ramp_span / (.75*max(abs(delta)))))
    distance_factor = 2. + float(alpha[1] - alpha[-2])
    if minimum_duration > 0.:
        rate = min(rate, distance_factor/minimum_duration)
    ramp = 2.*float(alpha[1])/rate
    times = [0.] + [ramp + float(a-alpha[1])/rate for a in alpha[1:-1]]
    times.append(distance_factor/rate)
    result = deepcopy(plan)
    for i, (point, seconds) in enumerate(zip(result.joint_trajectory.points, times)):
        point.time_from_start = rospy.Duration.from_sec(seconds)
        point.velocities = ([0.]*len(names) if i in (0, len(points)-1)
                            else (delta*rate).tolist())
        point.accelerations = [0.]*len(names)
    actual_times = [p.time_from_start.to_sec() for p in result.joint_trajectory.points]
    if any(b <= a for a, b in zip(actual_times, actual_times[1:])):
        return None
    return result
