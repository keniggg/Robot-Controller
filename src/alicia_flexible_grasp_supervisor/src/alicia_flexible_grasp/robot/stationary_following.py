"""Pure, measured stationary following diagnostics; never motion authority.

An unchanged SDK write is not an acknowledgement of physical arrival.  A
reused camera view has no newly executed endpoint, but its SDK/encoder gap
must still reach near-field uncertainty accounting instead of a self-pose 0.
"""
import math

import numpy as np

from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk


ARM_NAMES = tuple('Joint%d' % i for i in range(1, 7))
SDK_QUANTUM_RAD = 2 * math.pi / 4096


def _window(messages, frame, names, now, epoch_ns, maximum_age, start=None):
    rows = []
    previous = 0
    for msg in messages:
        stamp_ns = int(msg.header.stamp.to_nsec())
        if stamp_ns < epoch_ns:
            continue
        if stamp_ns <= previous or stamp_ns <= 0:
            raise ValueError('duplicate or reversed %s source stamps' % frame)
        previous = stamp_ns
        if msg.header.frame_id != frame:
            raise ValueError('incorrect %s source frame' % frame)
        keys, values = list(msg.name), list(msg.position)
        if (len(keys) != len(values) or len(keys) != len(set(keys))
                or set(keys) != set(names)):
            raise ValueError('ambiguous %s joint names/positions' % frame)
        mapping = dict(zip(keys, values))
        q = np.asarray([float(mapping[name]) for name in ARM_NAMES])
        if not np.all(np.isfinite(q)) or np.any(np.abs(q) > math.pi):
            raise ValueError('invalid %s arm positions' % frame)
        if 'right_finger' in names:
            opening = float(mapping['right_finger'])
            if not math.isfinite(opening) or not 0 <= opening <= .05:
                raise ValueError('invalid measured gripper opening')
        rows.append((stamp_ns, q))
    if not rows or not 0 <= now - rows[-1][0]*1e-9 <= maximum_age:
        raise ValueError('missing, stale or future %s evidence' % frame)
    if start is None:
        return rows
    before = [i for i, row in enumerate(rows) if row[0] <= start]
    if not before:
        raise ValueError('insufficient stationary %s window' % frame)
    rows = rows[before[-1]:]
    if len(rows) < 2 or any((b[0]-a[0])*1e-9 > maximum_age
                           for a, b in zip(rows, rows[1:])):
        raise ValueError('missing independent %s samples' % frame)
    return rows


def accepted_stationary_window(measured_messages, *, now_sec, epoch_ns,
                               after_ns, duration_sec, position_epsilon_rad,
                               minimum_samples=3, maximum_age_sec=.5,
                               maximum_gap_sec=.25):
    """Certify a complete post-action encoder window, never polling repeats.

    The range of all six named arm joints is bounded across the whole window.
    A small difference between adjacent polls cannot establish this: an old
    cached message, slow drift or an oscillation between polls could pass it.
    This proves measured stillness only, not arrival or physical contact.
    """
    now, duration = float(now_sec), float(duration_sec)
    epsilon, maximum = float(position_epsilon_rad), float(maximum_age_sec)
    gap = float(maximum_gap_sec)
    if (not all(math.isfinite(v) for v in (now, duration, epsilon, maximum, gap))
            or now <= 0 or duration <= 0 or epsilon < 0
            or not 0 < gap <= maximum <= .5
            or type(minimum_samples) is not int or minimum_samples < 2
            or type(epoch_ns) is not int or not 0 < epoch_ns <= int(now*1e9)
            or type(after_ns) is not int or after_ns < epoch_ns):
        raise ValueError('invalid accepted stationary window contract')
    measured_messages = tuple(measured_messages)
    rows = _window(measured_messages, 'sdk_measured',
                   ARM_NAMES+('right_finger',), now, epoch_ns, maximum)
    end = rows[-1][0]
    start = end - int(math.ceil(duration*1e9))
    if start < after_ns:
        raise ValueError('insufficient post-action stationary duration')
    rows = _window(measured_messages, 'sdk_measured',
                   ARM_NAMES+('right_finger',), now, epoch_ns, maximum, start)
    if rows[0][0] < after_ns:
        raise ValueError('stationary window crosses action boundary')
    if (len(rows) < minimum_samples
            or any((b[0]-a[0])*1e-9 > gap + 1e-12
                   for a, b in zip(rows, rows[1:]))):
        raise ValueError('insufficient continuous accepted samples')
    span = np.ptp(np.asarray([q for _, q in rows]), axis=0)
    if np.any(span > epsilon + 1e-12):
        raise ValueError('accepted feedback range is not stationary')
    return dict(epoch_ns=epoch_ns, accepted_stamp_ns=end,
                stationary_window_start_ns=rows[0][0],
                stationary_window_end_ns=end, accepted_sample_count=len(rows),
                accepted_positions_rad=rows[-1][1].tolist(),
                joint_peak_to_peak_rad=span.tolist())


def stationary_following_error(fk, sdk_messages, measured_messages, *,
                               now_sec, epoch_ns, maximum_age_sec=.5):
    """Compare FK of successful wire targets and real accepted encoder data.

    Require a common >= 0.3 s stationary window in one positive-enable epoch.
    No TF, requested joint commands, wall-time heartbeat, prior task residual
    or calibration correction substitutes for either source.  The result is
    a local model-space following sample, not contact clearance or proof that
    a new small command elicited a response.  No publishers/services/ROS I/O.
    """
    now, maximum = float(now_sec), float(maximum_age_sec)
    if (not isinstance(fk, SerialUrdfFk) or tuple(fk.names) != ARM_NAMES
            or not math.isfinite(now) or now <= 0
            or not math.isfinite(maximum) or not 0 < maximum <= .5
            or type(epoch_ns) is not int or not 0 < epoch_ns <= int(now*1e9)):
        raise ValueError('invalid following model, clock, epoch or freshness contract')
    # Freeze the input sequence once: both window passes and the gripper
    # evidence below must reference the exact same accepted message.
    sdk_messages, measured_messages = tuple(sdk_messages), tuple(measured_messages)
    sdk = _window(sdk_messages, 'sdk_transmitted', ARM_NAMES,
                  now, epoch_ns, maximum)
    measured = _window(measured_messages, 'sdk_measured', ARM_NAMES+('right_finger',),
                       now, epoch_ns, maximum)
    end = min(sdk[-1][0], measured[-1][0])
    start = end - 300_000_000
    sdk = _window(sdk_messages, 'sdk_transmitted', ARM_NAMES,
                  now, epoch_ns, maximum, start)
    measured = _window(measured_messages, 'sdk_measured', ARM_NAMES+('right_finger',),
                       now, epoch_ns, maximum, start)
    if any(not np.array_equal(q, sdk[-1][1]) for _, q in sdk):
        raise ValueError('SDK target changed in reused observation window')
    measured_q = np.asarray([q for _, q in measured])
    if np.any(np.ptp(measured_q, axis=0) > SDK_QUANTUM_RAD+1e-12):
        raise ValueError('accepted feedback is not stationary')
    accepted_stamp_ns = measured[-1][0]
    accepted_message = next(
        msg for msg in reversed(measured_messages)
        if int(msg.header.stamp.to_nsec()) == accepted_stamp_ns)
    accepted_opening = float(dict(zip(accepted_message.name,
                                      accepted_message.position))['right_finger'])
    target, actual = fk(sdk[-1][1]), fk(measured[-1][1])
    error = actual[:3, 3] - target[:3, 3]
    angle = math.acos(float(np.clip(
        (np.trace(target[:3, :3].T @ actual[:3, :3])-1)/2, -1., 1.)))
    return {
        'measurement_kind': 'stationary_sdk_following_not_executed_endpoint',
        'stamp_sec': now,
        'epoch_ns': epoch_ns,
        'model_sha256': fk.model_sha256,
        'sdk_stamp_ns': sdk[-1][0],
        'accepted_stamp_ns': accepted_stamp_ns,
        'accepted_gripper_opening_m': accepted_opening,
        'opening_stamp_ns': accepted_stamp_ns,
        'stationary_window_start_ns': start,
        'stationary_window_end_ns': end,
        'sdk_sample_count': len(sdk),
        'accepted_sample_count': len(measured),
        'joint_names': list(ARM_NAMES),
        'sdk_positions_rad': sdk[-1][1].tolist(),
        'accepted_positions_rad': measured[-1][1].tolist(),
        'joint_error_sdk_counts': ((measured[-1][1]-sdk[-1][1])/SDK_QUANTUM_RAD).tolist(),
        'position_error_m': float(np.linalg.norm(error)),
        'position_error_vector_m': error.tolist(),
        'orientation_error_rad': angle,
        'sdk_tool_position_m': target[:3, 3].tolist(),
        'accepted_tool_position_m': actual[:3, 3].tolist(),
        'certifies_command_response_or_calibration': False,
    }
