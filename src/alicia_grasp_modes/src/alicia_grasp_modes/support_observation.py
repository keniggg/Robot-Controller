"""Keep positive table-geometry contradictions separate from missing vision.

This consumes the existing exact-RGB-D detector verdict; it does not change
its thresholds, calibrate a transform, modify a target, or authorize motion.
"""
import math
import re

from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES, SDK_QUANTUM_RAD


def _source_window(messages, frame, names, source_ns, epoch_ns, coverage_end_ns=None):
    """Select a complete original-source window using integer nanoseconds."""
    start = source_ns - 300_000_000
    if start < epoch_ns:
        raise ValueError('source_window_crosses_control_epoch')
    stamped = []
    previous = 0
    for message in messages:
        stamp = message.header.stamp.to_nsec()
        if type(stamp) is not int or stamp <= 0:
            raise ValueError('invalid_%s_source_stamp' % frame)
        if stamp < epoch_ns:
            continue
        if stamp <= previous:
            raise ValueError('duplicate_or_reversed_%s_source_stamp' % frame)
        previous = stamp
        stamped.append((stamp, message))
    before_start = [i for i, (stamp, _) in enumerate(stamped) if stamp <= start]
    after_source = [i for i, (stamp, _) in enumerate(stamped)
                    if stamp > source_ns and (coverage_end_ns is None or stamp >= coverage_end_ns)]
    if not before_start or not after_source:
        raise ValueError('incomplete_%s_source_window' % frame)
    selected = stamped[before_start[-1]:after_source[0] + 1]
    if (len(selected) < 3 or any(b[0] - a[0] > 250_000_000
                                for a, b in zip(selected, selected[1:]))):
        raise ValueError('discontinuous_%s_source_window' % frame)
    rows = []
    for stamp, message in selected:
        keys, values = list(message.name), list(message.position)
        if (message.header.frame_id != frame or len(keys) != len(values)
                or len(keys) != len(set(keys)) or set(keys) != set(names)
                or any(isinstance(v, bool) for v in values)):
            raise ValueError('invalid_%s_source_schema' % frame)
        mapping = dict(zip(keys, values))
        q = tuple(float(mapping[name]) for name in ARM_NAMES)
        if any(not math.isfinite(v) or abs(v) > math.pi for v in q):
            raise ValueError('invalid_%s_source_positions' % frame)
        if 'right_finger' in names:
            opening = float(mapping['right_finger'])
            if not math.isfinite(opening) or not 0. <= opening <= .05:
                raise ValueError('invalid_accepted_gripper_opening')
        rows.append((stamp, q))
    before = max(i for i, (stamp, _) in enumerate(rows) if stamp <= source_ns)
    bracket = rows[before:before + 2]
    if frame == 'sdk_measured' and bracket[1][0] - bracket[0][0] > 200_000_000:
        raise ValueError('accepted_feedback_does_not_bracket_source')
    span = [max(q[j] for _, q in rows) - min(q[j] for _, q in rows) for j in range(6)]
    return rows, bracket, span


def qualify_support_source(sdk_messages, accepted_messages, *, source_stamp_ns,
                           epoch_ns, expected_epoch_ns):
    """Classify only the original image-time feedback, never current stillness.

    This is neither target identity nor new geometry/motion authority. Missing
    evidence is explicitly unqualified, not a reason to assume camera motion.
    """
    result = dict(version=1, state='unqualified', source_stamp_ns=source_stamp_ns,
                  epoch_ns=epoch_ns, motion_authorized=False,
                  exposure_time_calibrated=False)
    try:
        if (type(source_stamp_ns) is not int or type(epoch_ns) is not int
                or type(expected_epoch_ns) is not int
                or not 0 < epoch_ns < source_stamp_ns or epoch_ns != expected_epoch_ns):
            raise ValueError('support_source_control_epoch_unqualified')
        accepted, bracket, accepted_span = _source_window(tuple(accepted_messages),
            'sdk_measured', ARM_NAMES + ('right_finger',), source_stamp_ns, epoch_ns)
        sdk, sdk_bracket, sdk_span = _source_window(tuple(sdk_messages),
            'sdk_transmitted', ARM_NAMES, source_stamp_ns, epoch_ns,
            coverage_end_ns=bracket[1][0])
        result.update(window_start_ns=source_stamp_ns - 300_000_000,
            sdk_window_ns=[sdk[0][0], sdk[-1][0]],
            accepted_window_ns=[accepted[0][0], accepted[-1][0]],
            sdk_sample_count=len(sdk), accepted_sample_count=len(accepted),
            sdk_source_bracket_ns=[row[0] for row in sdk_bracket],
            accepted_source_bracket_ns=[row[0] for row in bracket],
            sdk_peak_to_peak_rad=sdk_span, accepted_peak_to_peak_rad=accepted_span)
        if (all(v == 0. for v in sdk_span)
                and all(v <= SDK_QUANTUM_RAD + 1e-12 for v in accepted_span)):
            result.update(state='stationary', reason='original_source_window_stationary')
            return result
        sdk_lo = max(i for i, row in enumerate(sdk) if row[0] <= bracket[0][0])
        sdk_hi = min(i for i, row in enumerate(sdk) if row[0] >= bracket[1][0])
        sdk_change_hi = max(i for i, row in enumerate(sdk) if row[0] <= bracket[1][0])
        if sdk[sdk_hi][0] - sdk[sdk_lo][0] > 200_000_000:
            raise ValueError('sdk_does_not_cover_accepted_source_bracket_within_200ms')
        result['sdk_accepted_common_window_ns'] = [sdk[sdk_lo][0], sdk[sdk_hi][0]]
        result['sdk_motion_evidence_window_ns'] = [sdk[sdk_lo][0], sdk[sdk_change_hi][0]]
        # Compare SDK commands at the encoder's time resolution. The nearest
        # SDK pair may lie on one quantized 20 ms plateau, whereas the 100 ms
        # encoder bracket spans genuine motion. Earlier movement outside this
        # common source bracket cannot exempt current one-count sensor jitter.
        moving = [ARM_NAMES[j] for j in range(6)
            if accepted_span[j] > SDK_QUANTUM_RAD + 1e-12
            and abs(bracket[1][1][j] - bracket[0][1][j]) > 1e-12
            and (sdk[sdk_change_hi][1][j] - sdk[sdk_lo][1][j])
                * (bracket[1][1][j] - bracket[0][1][j]) > 0.]
        if moving:
            result.update(state='moving', reason='original_source_window_measured_motion',
                          moving_joint_names=moving)
            return result
        raise ValueError('source_window_neither_stationary_nor_proven_moving')
    except (AttributeError, KeyError, TypeError, ValueError, OverflowError) as exc:
        result['reason'] = str(exc)
        return result

_PATTERN = re.compile(
    r'^perception_failure: (support_plane_(?:local_distance|normal)_inconsistent): '
    r'([0-9.eE+-]+) > ([0-9.eE+-]+) (m|deg)$')


def support_contradiction(status, selection):
    try:
        if (not isinstance(status, dict) or not isinstance(selection, dict)
                or selection.get('mode') != 'unknown'
                or status.get('mode') != 'unknown'
                or status.get('state') != 'unavailable'
                or status.get('target_locked') is not True
                or status.get('exact_rgbd') is not True):
            return None
        generation = status['generation']
        cutoff = status['selection_stamp_ns']
        source = status['source_stamp_ns']
        if (any(type(v) is not int for v in (generation, cutoff, source))
                or generation != selection['generation']
                or cutoff != int(selection['stamp_ns']) or source <= cutoff
                or any(type(status.get(key)) is not int or status[key] != source
                       for key in ('color_stamp_ns', 'depth_stamp_ns', 'tf_lookup_stamp_ns'))):
            return None
        match = _PATTERN.fullmatch(status.get('detail', ''))
        if not match:
            return None
        code, value, limit, unit = match.groups()
        value, limit = float(value), float(limit)
        if (not math.isfinite(value) or not math.isfinite(limit)
                or not value > limit > 0.
                or unit != ('m' if 'local_distance' in code else 'deg')):
            return None
        return dict(generation=generation, selection_stamp_ns=cutoff,
                    source_stamp_ns=source, code=code, value=value,
                    limit=limit, unit=unit, detail=status['detail'])
    except (KeyError, TypeError, ValueError, OverflowError):
        return None


def contradiction_after_plan(fault, selection, plan_stamp_ns):
    """Old generations/snapshots cannot revoke a newer target or plan."""
    return bool(fault and selection and selection.get('mode') == 'unknown'
        and fault['generation'] == selection.get('generation')
        and fault['selection_stamp_ns'] == int(selection['stamp_ns'])
        and fault['source_stamp_ns'] > plan_stamp_ns)
