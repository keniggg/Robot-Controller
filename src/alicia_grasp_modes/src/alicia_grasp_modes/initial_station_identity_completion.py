"""Bounded completion of timely captured identity evidence, never old geometry."""
from copy import deepcopy
import math


POLICY = 'initial_static_captured_identity_completion'


def _positive(value):
    if type(value) is not int or value <= 0:
        raise ValueError('identity_completion_invalid_integer')
    return value


def station_from_marker(marker):
    return dict(source_stamp_ns=int(marker['source_stamp_ns']), epoch_ns=int(marker['epoch_ns']),
        source_id=marker['source_id'], q_station_rad=list(marker['q_source']))


def station_from_receipt(receipt):
    return dict(source_stamp_ns=receipt['station_source_stamp_ns'], epoch_ns=receipt['epoch_ns'],
        source_id=receipt['source_id'], q_station_rad=list(receipt['q_station_rad']))


def completion_contract(reference_ns, maximum_sec, template, station):
    _positive(reference_ns)
    if (isinstance(maximum_sec, bool) or not isinstance(maximum_sec, (float, int))
            or not math.isfinite(maximum_sec) or not 0 < maximum_sec <= .8):
        raise ValueError('identity_completion_invalid_maximum')
    if (not isinstance(template, dict) or not isinstance(station, dict)
            or _positive(station['source_stamp_ns']) != template['source_stamp_ns']
            or _positive(station['epoch_ns']) != template['epoch_ns']
            or not station['epoch_ns'] <= station['source_stamp_ns'] <= reference_ns
            or station['source_id'] != template['source_id']
            or len(station['q_station_rad']) != 6
            or any(isinstance(q, bool) or not isinstance(q, (int, float)) or not math.isfinite(q)
                   for q in station['q_station_rad'])):
        raise ValueError('identity_completion_station_template_mismatch')
    maximum_ns = round(maximum_sec*1e9)
    return dict(version=1, policy=POLICY, reference_source_stamp_ns=reference_ns,
        maximum_frame_age_seconds=maximum_sec,
        geometry_deadline_ns=reference_ns+maximum_ns,
        capture_deadline_ns=reference_ns+maximum_ns,
        identity_completion_deadline_ns=reference_ns+2*maximum_ns,
        identity_template_source=deepcopy(template), station=deepcopy(station),
        metric_geometry_authority=False, motion_authorized=False, reference_refreshed=False)


def validate_completion(value, reference_ns, maximum_sec, template, station, now_ns, source_ns=None):
    expected = completion_contract(reference_ns, maximum_sec, template, station)
    if (not isinstance(value, dict) or value != expected or type(value.get('version')) is not int
            or any(value.get(k) is not False for k in ('metric_geometry_authority',
                'motion_authorized', 'reference_refreshed'))
            or any(type(value.get(k)) is not int for k in ('reference_source_stamp_ns',
                'geometry_deadline_ns', 'capture_deadline_ns', 'identity_completion_deadline_ns'))
            or not reference_ns <= _positive(now_ns) < expected['identity_completion_deadline_ns']):
        raise ValueError('identity_completion_contract_changed_or_expired')
    if source_ns is not None:
        if not reference_ns < _positive(source_ns) < expected['capture_deadline_ns']:
            raise ValueError('identity_completion_source_not_captured_in_geometry_window')
        if not 0 <= now_ns-source_ns < round(maximum_sec*1e9):
            raise ValueError('identity_completion_current_source_stale_or_future')
    return expected['identity_completion_deadline_ns']
