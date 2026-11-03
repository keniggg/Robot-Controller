"""Read-only terminal-task validation for returning ownership to the GUI.

This is not automatic-motion readiness. An unconfirmed actuator stays
unconfirmed; no enable, reference reset, motion or temperature I/O is performed.
"""
import json
import math


def validate_idle_router(payload):
    """Read the actual router schema: pending is a mode string, not a flag."""
    def unique(pairs):
        value = {}
        for key, item in pairs:
            if key in value:
                raise ValueError('router contains duplicate fields')
            value[key] = item
        return value

    mode = json.loads(payload, object_pairs_hook=unique)
    if (type(mode) is not dict or not {'active', 'pending', 'start_reserved'} <= set(mode)
            or type(mode['active']) is not bool or type(mode['start_reserved']) is not bool
            or type(mode['pending']) is not str):
        raise ValueError('router state fields missing or malformed')
    if mode['active'] or mode['start_reserved'] or mode['pending']:
        raise ValueError('router still active')
    return mode


def validate_manual_return_snapshot(samples, states, now_ns, epoch_ns, owners):
    def require(ok, reason):
        if not ok:
            raise ValueError('MANUAL_RETURN_BLOCKED: ' + reason)

    require(set(states) == set(owners), 'state owners incomplete')
    require(type(epoch_ns) is int and 0 < epoch_ns <= now_ns, 'invalid epoch')
    for key, (message, received) in states.items():
        require(message._connection_header.get('callerid') == owners[key], key + ' owner changed')
        require(0 <= now_ns - received < 1_000_000_000, key + ' receive state stale')
    require(states['epoch'][0].stamp.to_nsec() == epoch_ns, 'epoch changed')
    require(states['task'][0].active is False, 'task still active')
    try:
        mode = validate_idle_router(states['mode'][0].data)
    except (ValueError, TypeError) as exc:
        raise ValueError('MANUAL_RETURN_BLOCKED: ' + str(exc)) from exc
    require(states['manual'][0]._connection_header.get('latching') == '1', 'GUI acknowledgement not latched')
    actions = {}
    for key in ('arm', 'hand'):
        message = states[key][0]
        require(0 <= now_ns-message.header.stamp.to_nsec() <= 2_000_000_000, key + ' action state stale')
        require(not any(goal.status in (0, 1, 6, 7) for goal in message.status_list), key + ' action still active')
        actions[key] = [goal.status for goal in message.status_list]
    require(len(samples) == 20, 'stationary sample window incomplete')
    names = samples[-1]['names']
    require({'Joint%d' % i for i in range(1, 7)} <= set(names)
            and len(names) == len(set(names)), 'joint names invalid')
    previous = 0
    for row in samples:
        stamp, received = row['stamp_ns'], row['received_ns']
        require(row['caller'] == owners['joint'] and row['frame_id'] == 'sdk_measured', 'measured feedback owner invalid')
        require(row['names'] == names and len(row['positions']) == len(names)
                and all(math.isfinite(q) for q in row['positions']), 'joint values invalid')
        require(epoch_ns <= stamp <= received < stamp + 200_000_000
                and previous < stamp, 'feedback time or epoch invalid')
        previous = stamp
    spans = {name: max(r['positions'][i] for r in samples)-min(r['positions'][i] for r in samples)
             for i, name in enumerate(names)}
    require(max(spans.values()) <= .004 and 0 <= now_ns-previous < 200_000_000, 'arm not freshly stationary')
    status = states['actuation'][0].data
    require(isinstance(status, str) and bool(status), 'actuation status missing')
    require(type(states['enabled'][0].data) is bool and type(states['ready'][0].data) is bool,
            'driver state malformed')
    return dict(purpose='return_finished_stationary_task_to_manual', automatic_motion_authorized=False,
        epoch_ns=epoch_ns, sampled_ns=now_ns, sample_spans_rad=spans, actions=actions, router=mode,
        actuation_status=status, motion_enabled=states['enabled'][0].data,
        feedback_ready=states['ready'][0].data,
        state_received_ns={key: row[1] for key, row in states.items()})
