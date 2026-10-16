"""Validate a pre-submission rejection receipt; this grants no motion authority."""
from copy import deepcopy
import math

from .selection import parse_selection


def _ns(value):
    if type(value) is int:
        return value
    if type(value) is str and value.isdigit():
        return int(value)
    raise ValueError('prefix_preflight_invalid_timestamp')


def _number(value):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError('prefix_preflight_invalid_number')
    return value


def _identifier(value):
    return (type(value) is str and 1 <= len(value) <= 128
            and not any(char.isspace() for char in value))


def validate_prefix_preflight_receipt(ledger, *, now_ns=None, monotonic_now=None):
    """Require an explicit, internally bound no-submission receipt.

    Consumers must additionally bind this to their existing immutable RGB
    reference. The gateway must also retain its private, live retry capability;
    a serialized receipt alone cannot authorize a retry after restart.
    """
    try:
        proof = ledger['actuation_prefix_preflight']
        if (ledger.get('phase') != 'preflight_rejected' or not isinstance(proof, dict)
                or proof.get('kind') != 'rgb_prefix_preflight_no_submission_v1'
                or proof.get('submission_started') is not False
                or proof.get('original_reference_preserved') is not True
                or ledger.get('actuation_prefix_submission_started') is not False
                or ledger.get('actuation_prefix_attempted') is not True
                or ledger.get('contact_authorized') is not False):
            raise ValueError('prefix_preflight_no_submission_proof_missing')
        for key in ('request_id', 'reference_id', 'source_id'):
            if not _identifier(proof.get(key)) or proof[key] != ledger.get(key):
                raise ValueError('prefix_preflight_identity_changed')
        for key in ('reference_id', 'source_id'):
            if len(proof[key]) != 64 or any(c not in '0123456789abcdef' for c in proof[key]):
                raise ValueError('prefix_preflight_digest_invalid')
        selection = parse_selection(proof['selection'])
        if selection != parse_selection(ledger['selection']):
            raise ValueError('prefix_preflight_selection_changed')
        for key in ('attempts', 'max_observations', 'observation_index'):
            if (type(proof.get(key)) is not int or type(ledger.get(key)) is not int
                    or proof[key] != ledger[key]):
                raise ValueError('prefix_preflight_budget_changed')
        if not (1 <= proof['attempts'] <= proof['max_observations'] <= 3
                and proof['observation_index'] == 1):
            raise ValueError('prefix_preflight_budget_invalid')
        for key in ('created_ns', 'deadline_ns'):
            if _ns(proof[key]) != _ns(ledger[key]):
                raise ValueError('prefix_preflight_deadline_changed')
        for key in ('created_monotonic', 'deadline_monotonic', 'duration_sec'):
            if _number(proof[key]) != _number(ledger[key]):
                raise ValueError('prefix_preflight_deadline_changed')
        created, deadline = _ns(proof['created_ns']), _ns(proof['deadline_ns'])
        duration = proof['duration_sec']
        if (not 0 < duration <= 180 or not 0 < created < deadline
                or deadline-created != int(duration*1e9)
                or abs(proof['deadline_monotonic']-proof['created_monotonic']-duration) > 1e-6):
            raise ValueError('prefix_preflight_deadline_invalid')
        epoch, source = _ns(proof['epoch_ns']), _ns(proof['last_capture_source_stamp_ns'])
        q = proof['q_source']
        if (epoch <= 0 or not epoch < source < deadline
                or not isinstance(q, (list, tuple)) or len(q) != 6
                or any(not -math.pi <= _number(value) <= math.pi for value in q)):
            raise ValueError('prefix_preflight_source_invalid')
        reservation = proof['failed_reservation_request_id']
        history = ledger['request_ids']
        if (not _identifier(reservation)
                or reservation != ledger.get('reservation_request_id', ledger['request_id'])
                or not isinstance(history, list) or len(history) != proof['attempts']
                or any(not _identifier(value) for value in history)
                or len(set(history)) != len(history) or history[-1] != reservation
                or history[0] != proof['request_id']):
            raise ValueError('prefix_preflight_reservation_changed')
        if now_ns is not None and not created <= _ns(now_ns) < deadline:
            raise ValueError('prefix_preflight_expired')
        if now_ns is not None and source > _ns(now_ns):
            raise ValueError('prefix_preflight_future_source')
        if (monotonic_now is not None and not proof['created_monotonic'] <=
                _number(monotonic_now) < proof['deadline_monotonic']):
            raise ValueError('prefix_preflight_expired')
        return deepcopy(proof)
    except (KeyError, TypeError, OverflowError) as exc:
        raise ValueError('prefix_preflight_receipt_invalid') from exc
