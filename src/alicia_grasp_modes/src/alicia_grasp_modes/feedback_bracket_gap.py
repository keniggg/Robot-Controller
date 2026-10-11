"""Non-authorizing initial-station frame rejection, not feedback interpolation."""
from copy import deepcopy
import math

import numpy as np

KIND = 'accepted_feedback_bracket_gap'
MAXIMUM_SPAN_NS = 200_000_000


class InitialFeedbackBracketGap(ValueError):
    def __init__(self, reference, evidence):
        super().__init__(KIND)
        self.reference, self.evidence = reference, deepcopy(evidence)


def validate_gap_evidence(evidence, source_ns, reference_ns, epoch_ns, station_q, now_ns, maximum_sec):
    """Both real sides exist, but their span remains strictly unacceptable.

    Equality of positions never qualifies interpolation. Only the unchanged
    original reference lifetime permits awaiting a different measured frame.
    """
    if (isinstance(maximum_sec, bool) or not isinstance(maximum_sec, (int, float))
            or not math.isfinite(maximum_sec) or not 0. < maximum_sec <= .8
            or type(reference_ns) is not int or type(now_ns) is not int
            or not now_ns < reference_ns+round(maximum_sec*1e9)):
        raise ValueError('feedback_gap_reference_expired_or_changed')
    return validate_span_gap_evidence(evidence, source_ns, reference_ns, epoch_ns, station_q, now_ns)


def validate_span_gap_evidence(evidence, source_ns, reference_ns, epoch_ns, station_q, now_ns):
    """Rejected-bracket evidence only; a caller must enforce its own lifetime.

    This grants neither interpolation nor source or identity authority.
    """
    def integer(value):
        if type(value) is not int or value <= 0:
            raise ValueError('feedback_gap_invalid_stamp')
        return value
    def positions(value):
        raw = np.asarray(value)
        if raw.dtype.kind not in 'iuf' or raw.shape != (6,) or not np.all(np.isfinite(raw)):
            raise ValueError('feedback_gap_invalid_positions')
        return np.asarray(raw, dtype=float)
    source_ns, reference_ns, epoch_ns, now_ns = map(integer, (source_ns, reference_ns, epoch_ns, now_ns))
    if not epoch_ns <= reference_ns < source_ns <= now_ns:
        raise ValueError('feedback_gap_reference_expired_or_changed')
    if (not isinstance(evidence, dict) or evidence.get('bracket_qualified') is not False
            or type(evidence.get('maximum_span_ns')) is not int
            or evidence['maximum_span_ns'] != MAXIMUM_SPAN_NS):
        raise ValueError('feedback_gap_invalid_evidence')
    before, after = evidence.get('accepted_before'), evidence.get('accepted_after')
    if (not isinstance(before, (tuple, list)) or len(before) != 2
            or not isinstance(after, (tuple, list)) or len(after) != 2):
        raise ValueError('feedback_gap_requires_both_samples')
    lo, hi = integer(before[0]), integer(after[0])
    if (not reference_ns <= lo < source_ns < hi <= now_ns
            or type(evidence.get('span_ns')) is not int
            or evidence['span_ns'] != hi-lo or hi-lo <= MAXIMUM_SPAN_NS):
        raise ValueError('feedback_gap_not_excess_span')
    q = positions(station_q)
    if any(np.max(np.abs(positions(row[1])-q)) > .004 for row in (before, after)):
        raise ValueError('feedback_gap_measured_pose_changed')
    return deepcopy(evidence)


def validate_station_feedback_gap(value, *, contract, now_ns, recovery_source_ns=None):
    """Validate exact discarded-source tombstones under an audited parent.

    The consumer must independently validate the parent/recovery contract and
    the new complete measurement before granting any geometry authority.
    """
    if (not isinstance(value, dict) or any(value.get(key) is not False for key in
            ('source_qualified', 'identity_verified', 'metric_geometry_authority',
             'motion_authorized', 'reference_refreshed'))):
        raise ValueError('station_feedback_gap_authority_invalid')
    stamps = value.get('discarded_source_stamps_ns')
    if (not isinstance(stamps, list) or not 1 <= len(stamps) <= 32
            or any(type(ns) is not int or ns <= contract['source_ns'] for ns in stamps)
            or any(a >= b for a, b in zip(stamps, stamps[1:]))
            or value.get('discarded_source_stamp_ns') != stamps[-1]
            or type(value.get('discarded_source_stamp_ns')) is not int
            or stamps[-1] >= contract['identity_deadline_ns']
            or recovery_source_ns is not None and stamps[-1] >= recovery_source_ns):
        raise ValueError('station_feedback_gap_sources_invalid')
    binding = contract['binding']
    evidence = value.get('discarded_source_evidence')
    if not isinstance(evidence, list) or len(evidence) != len(stamps):
        raise ValueError('station_feedback_gap_evidence_missing')
    for stamp, row in zip(stamps, evidence):
        if not isinstance(row, dict) or type(row.get('source_stamp_ns')) is not int or row['source_stamp_ns'] != stamp:
            raise ValueError('station_feedback_gap_evidence_source_changed')
        validate_span_gap_evidence(row.get('feedback_gap_evidence'), stamp,
            contract['source_ns'], binding['control_epoch_ns'], binding['station_q_rad'], now_ns)
    if value.get('feedback_gap_evidence') != evidence[-1]['feedback_gap_evidence']:
        raise ValueError('station_feedback_gap_last_evidence_changed')
    return tuple(stamps)
