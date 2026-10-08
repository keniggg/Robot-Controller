"""Strict, explicit observation/return session contract; no robot commands."""
from copy import deepcopy
import json
import re

from .selection import parse_selection
from .active_observation_policy import observation_budget

SESSION_TOPIC = '/grasp_mode/observe_return_session'
ACK_TOPIC = '/grasp_mode/observe_return_ack'
_PHASES = ('observing', 'returning', 'returned')
_IMMUTABLE = ('session_id', 'selection', 'deadline_ns', 'target_epoch',
              'target_track_id', 'origin_plan_id', 'origin_snapshot_stamp_ns')
VIEW_FIELDS = ('view_index', 'view_plan_id', 'view_source_stamp_ns',
               'view_request_id', 'previous_gain_sha256')


def exact_ns(value, name, minimum=1):
    if isinstance(value, str) and value.isdecimal():
        value = int(value)
    if type(value) is not int or value < minimum:
        raise ValueError('%s must be an exact integer >= %d' % (name, minimum))
    return value


def parse_session(value, selection, now_ns, previous=None):
    if isinstance(value, str):
        value = json.loads(value)
    if not isinstance(value, dict) or type(value.get('version')) is not int or value['version'] not in (1, 2):
        raise ValueError('observe_return session version must be 1 or 2')
    if previous is not None:
        previous = deepcopy(previous)
        previous['selection'] = parse_selection(previous['selection'])
        for key in ('stamp_ns', 'deadline_ns', 'origin_snapshot_stamp_ns',
                    'return_verified_stamp_ns', 'contact_deadline_ns'):
            previous[key] = exact_ns(previous.get(key, 0), key, 0)
        if previous['version'] == 2:
            previous['view_source_stamp_ns'] = exact_ns(previous['view_source_stamp_ns'], 'view_source_stamp_ns')
    result = deepcopy(value)
    result['selection'] = parse_selection(value.get('selection'))
    if result['selection'] != parse_selection(selection) or result['selection']['strategy'] != 'observe_return':
        raise ValueError('observe_return selection mismatch')
    for key in ('session_id', 'target_track_id', 'origin_plan_id'):
        item = value.get(key)
        if not isinstance(item, str) or not item or item != item.strip() or any(c.isspace() for c in item):
            raise ValueError('invalid '+key)
    result['target_epoch'] = exact_ns(value.get('target_epoch'), 'target_epoch', 0)
    for key in ('stamp_ns', 'deadline_ns', 'origin_snapshot_stamp_ns'):
        result[key] = exact_ns(value.get(key), key)
    for key in ('return_verified_stamp_ns', 'contact_deadline_ns'):
        result[key] = exact_ns(value.get(key, 0), key, 0)
    phase = result.get('phase')
    if phase not in _PHASES:
        raise ValueError('invalid observe_return phase')
    if not result['selection']['stamp_ns'] < result['origin_snapshot_stamp_ns'] <= result['stamp_ns'] <= now_ns < result['deadline_ns']:
        raise ValueError('session source, publication or deadline is invalid')
    if result['version'] == 2:
        result['max_observations'] = observation_budget(value.get('max_observations'))
        result['view_index'] = exact_ns(value.get('view_index'), 'view_index')
        result['view_source_stamp_ns'] = exact_ns(value.get('view_source_stamp_ns'), 'view_source_stamp_ns')
        if (result['view_index'] > result['max_observations']
                or not result['origin_snapshot_stamp_ns'] <= result['view_source_stamp_ns'] <= result['stamp_ns']):
            raise ValueError('observation step budget or source is invalid')
        for name in ('view_plan_id', 'view_request_id'):
            item = value.get(name)
            if not isinstance(item, str) or not item or any(c.isspace() for c in item):
                raise ValueError('invalid ' + name)
        if not isinstance(value.get('previous_gain_sha256'), str) or not re.fullmatch('[0-9a-f]{64}', value['previous_gain_sha256']):
            raise ValueError('invalid previous gain digest')
    elif any(key in value for key in (*VIEW_FIELDS, 'max_observations')):
        raise ValueError('version 1 cannot carry supplemental observation authority')
    if previous is None:
        if phase != 'observing' or result['deadline_ns'] - result['stamp_ns'] > 600_000_000_000:
            raise ValueError('session must start observing with at most 600 seconds')
        if result['version'] == 2 and (result['view_index'] != 1
                or result['view_plan_id'] != result['origin_plan_id']
                or result['view_source_stamp_ns'] != result['origin_snapshot_stamp_ns']
                or result['previous_gain_sha256'] != '0' * 64):
            raise ValueError('initial observation step must bind the original plan')
    else:
        if (result['version'] != previous['version']
                or any(result[key] != previous[key] for key in _IMMUTABLE)
                or (result['version'] == 2 and result['max_observations'] != previous['max_observations'])):
            raise ValueError('session immutable binding changed')
        if result['stamp_ns'] <= previous['stamp_ns']:
            raise ValueError('session update is stale')
        supplement = result['version'] == 2 and phase == previous['phase'] == 'observing'
        if supplement:
            if (result['view_index'] != previous['view_index'] + 1
                    or result['view_source_stamp_ns'] <= previous['view_source_stamp_ns']
                    or result['view_plan_id'] == previous['view_plan_id']
                    or result['view_request_id'] == previous['view_request_id']
                    or result['previous_gain_sha256'] == '0' * 64):
                raise ValueError('supplemental observation step is not new and sequential')
        elif _PHASES.index(phase) != _PHASES.index(previous['phase']) + 1:
            raise ValueError('session phase transition is not sequential')
        elif result['version'] == 2 and any(result[key] != previous[key] for key in VIEW_FIELDS):
            raise ValueError('return transition changed the last observation step')
    if phase == 'returned':
        if (previous is None or not previous['stamp_ns'] < result['return_verified_stamp_ns'] <= result['stamp_ns']
                or not result['stamp_ns'] < result['contact_deadline_ns'] <= result['deadline_ns']):
            raise ValueError('returned phase lacks fresh verified return and finite contact deadline')
    elif result['return_verified_stamp_ns'] or result['contact_deadline_ns']:
        raise ValueError('return evidence is forbidden before returned phase')
    return result


def structured_view_deficit(diagnostics):
    """A geometric overlap/IK/simulation failure is not missing visual data."""
    if not isinstance(diagnostics, dict):
        diagnostics = dict(diagnostics or {})
    table = dict(diagnostics.get('tabletop_geometry', {}) or {})
    if (table.get('plan_phase') == 'CONTACT_EXECUTION_PLAN'
            and table.get('contact_execution_gate_deferred') is False
            and table.get('failure_code') == 'BILATERAL_SURFACE_EVIDENCE_MISSING'
            and int(table.get('materialized_count', 0)) == 0):
        return 'BILATERAL_SURFACE_EVIDENCE_MISSING'
    return ''
