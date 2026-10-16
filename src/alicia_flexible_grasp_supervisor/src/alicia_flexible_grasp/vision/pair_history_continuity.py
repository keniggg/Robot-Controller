"""Private depth-history continuity across one interleaved SDK packet.

These checks never admit an image pair or establish stationarity. A caller must
still apply its original pairing, encoder and visual gates. All clocks remain
in their original domains; no SDK timestamp is converted to a ROS exposure.
"""
import math
import numbers


def frame_profile_signature(frame):
    profile = frame.get_profile()
    intrinsics = profile.as_video_stream_profile().get_intrinsics()
    result = {name: str(getattr(profile, name)()) for name in ('stream_type', 'format')}
    result.update({name: getattr(profile, name)() for name in ('stream_index', 'unique_id', 'fps')})
    result.update({name: getattr(intrinsics, name) for name in
                   ('width', 'height', 'fx', 'fy', 'ppx', 'ppy')})
    result.update(coeffs=list(intrinsics.coeffs), model=str(intrinsics.model))
    _profile(result)
    return result


def _real(value):
    return (not isinstance(value, bool) and isinstance(value, numbers.Real)
            and math.isfinite(value))


def _profile(value):
    if not isinstance(value, dict):
        raise ValueError('profile_missing')
    for name in ('stream_type', 'format', 'model'):
        if not isinstance(value.get(name), str) or not value[name]:
            raise ValueError('profile_invalid')
    for name in ('stream_index', 'unique_id', 'width', 'height'):
        v = value.get(name)
        if isinstance(v, bool) or not isinstance(v, numbers.Integral) or v < 0:
            raise ValueError('profile_invalid')
    if min(value['width'], value['height']) <= 0:
        raise ValueError('profile_invalid')
    for name in ('fps', 'fx', 'fy', 'ppx', 'ppy'):
        if not _real(value.get(name)):
            raise ValueError('profile_invalid')
    if not 1 <= value['fps'] <= 240 or min(value['fx'], value['fy']) <= 0:
        raise ValueError('profile_invalid')
    if not isinstance(value.get('coeffs'), (list, tuple)) or len(value['coeffs']) != 5:
        raise ValueError('profile_invalid')
    if not all(_real(v) for v in value['coeffs']):
        raise ValueError('profile_invalid')


def _stream(value):
    if not isinstance(value, dict) or value.get('present') is not True:
        raise ValueError('stream_missing')
    n, stamp, fps = (value.get(k) for k in ('frame_number', 'device_timestamp_ms', 'stream_fps'))
    domain = str(value.get('timestamp_domain', '')).rsplit('.', 1)[-1]
    if isinstance(n, bool) or not isinstance(n, numbers.Integral) or n < 0:
        raise ValueError('frame_number_invalid')
    if not _real(stamp) or stamp < 0 or not _real(fps) or not 1 <= fps <= 240:
        raise ValueError('source_clock_invalid')
    if domain not in ('hardware_clock', 'global_time'):
        raise ValueError('source_domain_invalid')
    _profile(value.get('profile_signature'))
    if value['profile_signature']['fps'] != fps:
        raise ValueError('profile_fps_mismatch')
    return n, stamp, domain, fps


def _inputs(previous, current, previous_host, current_host):
    if not isinstance(previous, dict) or not isinstance(current, dict):
        raise ValueError('raw_pair_missing')
    if not _real(previous_host) or not _real(current_host) or min(previous_host, current_host) < 0:
        raise ValueError('host_clock_invalid')
    keys = {}
    for name in ('color', 'depth'):
        before, after = _stream(previous.get(name)), _stream(current.get(name))
        if before[2:] != after[2:] or previous[name]['profile_signature'] != current[name]['profile_signature']:
            raise ValueError(name+'_profile_or_clock_changed')
        # Same original host-gap bound as both existing stationary windows.
        if not 0 < current_host-previous_host <= max(.1, 2.5/before[3]):
            raise ValueError(name+'_host_discontinuity')
        keys[name] = (before, after)
    if keys['color'][0][2] != keys['depth'][0][2]:
        raise ValueError('pair_domain_mismatch')
    old_skew = abs(keys['color'][0][1]-keys['depth'][0][1])
    limit = 500./min(keys['color'][0][3], keys['depth'][0][3])
    if old_skew > limit:
        raise ValueError('previous_pair_not_qualified')
    return keys, limit


def _consecutive(before, after):
    # Existing regional/visual device interval, now independently for BOTH
    # raw streams, rather than inferring depth continuity from RGB alone.
    return (after[0] == before[0]+1
            and .25/before[3] <= (after[1]-before[1])/1000. <= 1.75/before[3])


def interleaved_pair_reason(previous, current, previous_host, current_host):
    """None only for one identical old stream plus one strictly next stream."""
    try:
        keys, limit = _inputs(previous, current, previous_host, current_host)
        if abs(keys['color'][1][1]-keys['depth'][1][1]) <= limit:
            return 'not_timestamp_skew_rejection'
        repeated = [name for name in keys if current[name] == previous[name]]
        if len(repeated) != 1:
            return 'not_one_identical_previous_stream'
        advanced = 'depth' if repeated[0] == 'color' else 'color'
        if not _consecutive(*keys[advanced]):
            return advanced+'_not_immediate_successor'
        return None
    except (ValueError, TypeError, KeyError, OverflowError) as exc:
        return str(exc) or 'invalid_pair_history_evidence'


def successor_pair_reason(previous, current, previous_host, current_host):
    """None only when the next accepted pair has BOTH immediate successors."""
    try:
        keys, limit = _inputs(previous, current, previous_host, current_host)
        if abs(keys['color'][1][1]-keys['depth'][1][1]) > limit:
            return 'next_pair_not_qualified'
        for name, pair in keys.items():
            if not _consecutive(*pair):
                return name+'_not_immediate_successor'
        return None
    except (ValueError, TypeError, KeyError, OverflowError) as exc:
        return str(exc) or 'invalid_pair_history_evidence'
