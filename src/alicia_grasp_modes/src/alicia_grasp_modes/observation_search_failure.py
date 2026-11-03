"""Bind a non-executable search-budget result to the current station.

This receipt can only end a planning wait. It never validates a pose, extends
an input lifetime, or grants contact or motion authority.
"""
import math
from copy import deepcopy

from .station_compute_reacquisition import validate_contract, validate_registration


CODE = 'OBSERVATION_SEARCH_BUDGET_EXHAUSTED'
SOURCE = 'observation_search_budget_exhausted'
POLICY = 'same_station_observation_search_budget'
REASON = 'insufficient_original_station_budget_for_next_strict_check'
FIELDS = frozenset((
    'version', 'policy', 'execution_authority', 'reason', 'binding',
    'phase_deadline_ns', 'session_deadline_ns', 'request', 'emitted_ns',
    'remaining_sec', 'estimated_strict_sec', 'publication_reserve_sec',
    'attempted_count', 'unchecked_count', 'timeout_count'))


def validate_search_budget_failure(value, contract, *, now_ns, header_ns):
    """Validate classification against an independently registered station.

    The caller still checks live selection, phase, control epoch and station
    ownership. A pending metric frame does not prevent accepting a failure.
    """
    if (not isinstance(value, dict) or set(value) != FIELDS
            or type(value['version']) is not int or value['version'] != 1
            or value['policy'] != POLICY or value['reason'] != REASON
            or value['execution_authority'] is not False
            or value['binding'] != contract['binding']):
        raise ValueError('observation_search_failure_binding_changed')
    for key in ('phase_deadline_ns', 'session_deadline_ns'):
        if type(value[key]) is not int or value[key] != contract[key]:
            raise ValueError('observation_search_failure_deadline_changed')
    emitted = value['emitted_ns']
    deadline = min(contract['phase_deadline_ns'], contract['session_deadline_ns'])
    if (type(emitted) is not int or type(now_ns) is not int
            or type(header_ns) is not int or header_ns != emitted
            or not contract['binding']['phase_started_ns'] < emitted <= now_ns < deadline):
        raise ValueError('observation_search_failure_outside_current_phase')
    request = value['request']
    if (not isinstance(request, dict)
            or set(request) != {'request_id', 'generation', 'target_epoch', 'snapshot_stamp_ns'}
            or any(type(request[k]) is not int or request[k] < 0 for k in request)
            or request['request_id'] <= 0
            or request['target_epoch'] != contract['binding']['target_epoch']
            or not contract['binding']['phase_reference_source_ns']
                   <= request['snapshot_stamp_ns'] <= emitted):
        raise ValueError('observation_search_failure_request_changed')
    # Stream generation and selection generation have separate namespaces.
    # Selection ownership comes from the full station binding above.
    for key in ('remaining_sec', 'estimated_strict_sec', 'publication_reserve_sec'):
        if type(value[key]) not in (int, float) or not math.isfinite(value[key]) or value[key] <= 0.:
            raise ValueError('observation_search_failure_invalid_budget')
    if (abs(value['remaining_sec'] - (deadline-emitted)/1e9) > 1e-6
            or value['remaining_sec'] > value['estimated_strict_sec'] + value['publication_reserve_sec']):
        raise ValueError('observation_search_failure_budget_not_exhausted')
    for key in ('attempted_count', 'unchecked_count', 'timeout_count'):
        if type(value[key]) is not int or value[key] < 0:
            raise ValueError('observation_search_failure_invalid_count')
    if value['timeout_count'] > value['attempted_count']:
        raise ValueError('observation_search_failure_invalid_count')
    return value


CONTACT_POLICY = 'same_station_completed_contact_search_failure'
CONTACT_CODE = 'NEAR_FIELD_NO_REACHABLE_CANDIDATE'
CONTACT_SOURCE = 'station_contact_search_terminal'
CONTACT_FIELDS = frozenset(('version', 'policy', 'execution_authority',
    'metric_geometry_authority', 'code', 'reason', 'binding', 'phase_deadline_ns',
    'session_deadline_ns', 'request', 'emitted_ns', 'sources', 'current', 'search',
    'audit_sha256'))


def _validate_contact_search_failure(value, contract, *, now_ns, header_ns):
    """Validate a negative result; pending geometry never grants motion.

    Historical inputs are checked at their original capture clocks. The
    consumer must additionally match each witness to its own registered source
    ledger and validate its *current* station identity and control context.
    """
    if (not isinstance(value, dict) or set(value) != CONTACT_FIELDS
            or type(value['version']) is not int or value['version'] != 1
            or value['policy'] != CONTACT_POLICY or value['code'] != CONTACT_CODE
            or value['execution_authority'] is not False
            or value['metric_geometry_authority'] is not False
            or not isinstance(value['reason'], str) or not value['reason']
            or value['binding'] != contract['binding']):
        raise ValueError('contact_search_failure_binding_changed')
    for key in ('phase_deadline_ns', 'session_deadline_ns'):
        if type(value[key]) is not int or value[key] != contract[key]:
            raise ValueError('contact_search_failure_deadline_changed')
    emitted = value['emitted_ns']
    if (type(emitted) is not int or type(now_ns) is not int
            or type(header_ns) is not int or header_ns != emitted
            or not contract['binding']['phase_started_ns'] < emitted <= now_ns
                < min(contract['phase_deadline_ns'], contract['session_deadline_ns'])):
        raise ValueError('contact_search_failure_outside_current_phase')
    request = value['request']
    if (not isinstance(request, dict) or set(request) !=
            {'request_id', 'generation', 'target_epoch', 'snapshot_stamp_ns'}
            or any(type(v) is not int or v <= 0 for v in request.values())
            or request['target_epoch'] != contract['binding']['target_epoch']
            or not contract['binding']['phase_reference_source_ns']
                <= request['snapshot_stamp_ns'] <= emitted):
        raise ValueError('contact_search_failure_request_changed')
    search = value['search']
    if (not isinstance(search, dict) or set(search) != {'checked_count',
            'shortlist_count', 'reachable_count', 'selected', 'terminated_early'}
            or search['selected'] is not False or search['terminated_early'] is not False
            or any(type(search[k]) is not int for k in
                ('checked_count', 'shortlist_count', 'reachable_count'))
            or search['checked_count'] <= 0
            or search['shortlist_count'] != search['checked_count']
            or search['reachable_count'] != 0):
        raise ValueError('contact_search_failure_incomplete_search')
    digest = value['audit_sha256']
    if (not isinstance(digest, str) or len(digest) != 64
            or any(c not in '0123456789abcdef' for c in digest)):
        raise ValueError('contact_search_failure_audit_invalid')
    sources, current = value['sources'], value['current']
    if not isinstance(sources, list) or len(sources) != 3:
        raise ValueError('contact_search_failure_three_sources_required')
    for witness in sources + [current]:
        if (not isinstance(witness, dict) or set(witness) !=
                {'contract', 'object_sha256', 'mask'}
                or witness['contract']['binding'] != value['binding']
                or not isinstance(witness['object_sha256'], str)
                or len(witness['object_sha256']) != 64
                or any(c not in '0123456789abcdef' for c in witness['object_sha256'])
                or not isinstance(witness['mask'], dict)):
            raise ValueError('contact_search_failure_source_changed')
        validate_registration(witness['contract'], binding=value['binding'],
            now_ns=witness['contract']['source_ns'])
    stamps = [row['contract']['source_ns'] for row in sources]
    if (not all(a < b for a, b in zip(stamps, stamps[1:]))
            or stamps[-1] != request['snapshot_stamp_ns']
            or current['contract']['source_ns'] < stamps[-1]):
        raise ValueError('contact_search_failure_snapshot_changed')
    # This is an issuance witness only, not a renewed geometry/identity clock.
    validate_contract(current['contract'], binding=value['binding'], now_ns=emitted)
    return deepcopy(value)


def validate_contact_search_failure(value, contract, *, now_ns, header_ns):
    """Keep malformed wire values on the typed validation error boundary."""
    try:
        return _validate_contact_search_failure(value, contract,
            now_ns=now_ns, header_ns=header_ns)
    except (KeyError, TypeError, AttributeError, OverflowError) as exc:
        raise ValueError('contact_search_failure_malformed') from exc
