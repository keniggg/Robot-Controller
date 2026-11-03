"""Task selection notice for an already audited/frozen observed CONTACT plan.

A notice is not a motion permit. The gateway checks its independently captured
session and request payload before turning it into a published-plan receipt.
"""
from copy import deepcopy
import json
import re

FROZEN_CONTACT_TOPIC = '/grasp_6d/frozen_contact_plan'
_FIELDS = {'version', 'plan_id', 'plan_sha256', 'source_ns', 'epoch_ns',
           'issued_ns', 'selection', 'session', 'phase_id', 'phase_deadline_ns'}


def exact_ns(value, minimum=1):
    if isinstance(value, str) and value.isdecimal():
        value = int(value)
    if type(value) is not int or value < minimum:
        raise ValueError('CONTACT_FREEZE_EXACT_INTEGER_REQUIRED')
    return value


def selection_binding(value):
    if not isinstance(value, dict):
        raise ValueError('CONTACT_FREEZE_SELECTION_REQUIRED')
    result = {k: value[k] for k in ('mode', 'strategy', 'generation', 'stamp_ns')}
    result['generation'] = exact_ns(result['generation'])
    result['stamp_ns'] = exact_ns(result['stamp_ns'])
    if result['mode'] != 'unknown' or result['strategy'] != 'observe_return':
        raise ValueError('CONTACT_FREEZE_SELECTION_INVALID')
    return result


def session_binding(value):
    if not isinstance(value, dict) or value.get('version') != 2 or value.get('phase') != 'observing':
        raise ValueError('CONTACT_FREEZE_OBSERVED_SESSION_REQUIRED')
    result = {k: value[k] for k in ('version', 'phase', 'session_id', 'target_track_id',
        'target_epoch', 'origin_plan_id', 'origin_snapshot_stamp_ns', 'stamp_ns', 'deadline_ns',
        'view_index', 'view_plan_id', 'view_source_stamp_ns', 'view_request_id',
        'previous_gain_sha256', 'max_observations')}
    for k in ('version', 'origin_snapshot_stamp_ns', 'stamp_ns', 'deadline_ns', 'view_index',
              'view_source_stamp_ns', 'max_observations'):
        result[k] = exact_ns(result[k])
    result['target_epoch'] = exact_ns(result['target_epoch'], 0)
    result['selection'] = selection_binding(value['selection'])
    for k in ('session_id', 'target_track_id', 'origin_plan_id', 'view_plan_id', 'view_request_id'):
        if not isinstance(result[k], str) or not result[k] or any(c.isspace() for c in result[k]):
            raise ValueError('CONTACT_FREEZE_SESSION_IDENTIFIER_INVALID')
    if not isinstance(result['previous_gain_sha256'], str) or not re.fullmatch('[0-9a-f]{64}', result['previous_gain_sha256']):
        raise ValueError('CONTACT_FREEZE_SESSION_GAIN_INVALID')
    if (not 1 <= result['view_index'] <= result['max_observations'] <= 3
            or not result['selection']['stamp_ns'] < result['origin_snapshot_stamp_ns']
            <= result['view_source_stamp_ns'] <= result['stamp_ns'] < result['deadline_ns']
            or exact_ns(value.get('return_verified_stamp_ns', 0), 0) != 0
            or exact_ns(value.get('contact_deadline_ns', 0), 0) != 0):
        raise ValueError('CONTACT_FREEZE_SESSION_ORDER_INVALID')
    return result


def parse_contact_freeze_notice(value):
    if isinstance(value, str):
        if len(value) > 16384:
            raise ValueError('CONTACT_FREEZE_NOTICE_TOO_LARGE')
        value = json.loads(value)
    if not isinstance(value, dict) or set(value) != _FIELDS or type(value['version']) is not int or value['version'] != 1:
        raise ValueError('CONTACT_FREEZE_NOTICE_SCHEMA_INVALID')
    result = deepcopy(value)
    for k in ('source_ns', 'epoch_ns', 'issued_ns', 'phase_id', 'phase_deadline_ns'):
        result[k] = exact_ns(result[k])
    for k, size in (('plan_id', 24), ('plan_sha256', 64)):
        if not isinstance(result[k], str) or not re.fullmatch('[0-9a-f]{%d}' % size, result[k]):
            raise ValueError('CONTACT_FREEZE_PLAN_BINDING_INVALID')
    result['selection'] = selection_binding(result['selection'])
    result['session'] = session_binding(result['session'])
    session = result['session']
    if (session['selection'] != result['selection']
            or not result['selection']['stamp_ns'] < result['source_ns']
            or not session['view_source_stamp_ns'] <= result['source_ns'] <= result['issued_ns']
            or not session['stamp_ns'] <= result['issued_ns']
            or not result['epoch_ns'] <= result['issued_ns'] < result['phase_deadline_ns'] <= session['deadline_ns']):
        raise ValueError('CONTACT_FREEZE_NOTICE_ORDER_INVALID')
    return result


def build_contact_freeze_notice(plan, *, selection, session, epoch_ns, issued_ns,
                               phase_id, phase_deadline_ns):
    from alicia_flexible_grasp.grasp.rich_plan_integrity import plan_id_matches_content
    from alicia_flexible_grasp.robot.observation_path_guard import committed_plan_digest
    if (plan.valid is not True or plan.model_choice != 'unknown_tabletop'
            or plan.diagnostic != 'CONTACT_EXECUTION_PLAN' or plan.refinement_status != 'VALID_3D'
            or plan.fused_view_count < 2 or not plan_id_matches_content(plan)
            or plan.target_track_id != session['target_track_id']):
        raise ValueError('CONTACT_FREEZE_PLAN_NOT_ADMITTED')
    return parse_contact_freeze_notice(dict(version=1, plan_id=plan.plan_id,
        plan_sha256=committed_plan_digest(plan), source_ns=plan.header.stamp.to_nsec(),
        epoch_ns=epoch_ns, issued_ns=issued_ns, selection=selection, session=session,
        phase_id=phase_id, phase_deadline_ns=phase_deadline_ns))
