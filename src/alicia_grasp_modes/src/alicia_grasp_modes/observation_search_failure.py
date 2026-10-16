"""Bind a non-executable search-budget result to the current station.

This receipt can only end a planning wait. It never validates a pose, extends
an input lifetime, or grants contact or motion authority.
"""
import math


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
