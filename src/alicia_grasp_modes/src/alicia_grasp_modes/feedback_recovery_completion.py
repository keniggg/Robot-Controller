"""One timely measured feedback-recovery frame may finish, without geometry authority."""
from copy import deepcopy

import numpy as np

from .initial_station_identity_completion import completion_contract
from .feedback_bracket_gap import validate_span_gap_evidence
from .rgb_observation_source import bracket_feedback


FIELD = 'feedback_recovery_completion'
STATE = 'feedback_recovery_processing'
GAP_KEYS = ('first_unqualified_source_stamp_ns', 'last_unqualified_source_stamp_ns',
            'feedback_gap_source_stamp_ns', 'feedback_gap_evidence')


def make_completion(reference_ns, maximum_sec, template, station, source_ns,
                    start_ns, entry_ns, accepted, gap):
    value = completion_contract(reference_ns, maximum_sec, template, station)
    maximum_ns = round(maximum_sec*1e9)
    if (any(type(n) is not int for n in (source_ns, start_ns, entry_ns))
            or not reference_ns < source_ns <= start_ns <= entry_ns < reference_ns+maximum_ns):
        raise ValueError('feedback_recovery_not_timely')
    first, last = gap['first_unqualified_source_stamp_ns'], gap['last_unqualified_source_stamp_ns']
    if (type(first) is not int or type(last) is not int
            or not reference_ns < first <= last < source_ns
            or gap['feedback_gap_source_stamp_ns'] != last):
        raise ValueError('feedback_recovery_gap_changed')
    validate_span_gap_evidence(gap['feedback_gap_evidence'], last, reference_ns,
        station['epoch_ns'], station['q_station_rad'], entry_ns)
    measured, before, after = bracket_feedback(accepted, source_ns, station['epoch_ns'])
    if (after[0] > entry_ns or before[0] <= last
            or any(np.max(np.abs(np.asarray(q)-station['q_station_rad'])) > .004
                   for q in (measured, before[1], after[1]))):
        raise ValueError('feedback_recovery_station_bracket_changed')
    value.update(policy='initial_feedback_recovery_single_frame_completion',
        source_stamp_ns=source_ns, processing_start_ns=start_ns, qualified_entry_ns=entry_ns,
        identity_completion_deadline_ns=min(source_ns+maximum_ns, reference_ns+2*maximum_ns),
        requalified_accepted_feedback=deepcopy(accepted),
        **{key: deepcopy(gap[key]) for key in GAP_KEYS})
    return value


def validate_feedback_completion(value, reference_ns, maximum_sec, template, station,
                                 now_ns, source_ns=None):
    if not isinstance(value, dict):
        raise ValueError('feedback_recovery_completion_missing')
    expected = make_completion(reference_ns, maximum_sec, template, station,
        value['source_stamp_ns'], value['processing_start_ns'], value['qualified_entry_ns'],
        value['requalified_accepted_feedback'], value)
    if (value != expected or type(value.get('version')) is not int
            or any(type(value.get(k)) is not int for k in ('reference_source_stamp_ns',
                'geometry_deadline_ns', 'capture_deadline_ns', 'identity_completion_deadline_ns'))
            or any(value.get(k) is not False for k in
                   ('metric_geometry_authority', 'motion_authorized', 'reference_refreshed'))
            or type(now_ns) is not int
            or not value['qualified_entry_ns'] <= now_ns < value['identity_completion_deadline_ns']
            or source_ns is not None and source_ns != value['source_stamp_ns']
            or not 0 <= now_ns-value['source_stamp_ns'] < round(maximum_sec*1e9)):
        raise ValueError('feedback_recovery_completion_changed_or_expired')
    return value['identity_completion_deadline_ns']
