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
        'accepted_stamp_ns': measured[-1][0],
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
