"""Retire a qualified observation station at a strict contact submission.

This removes stationary-only privileges. It does not hold a missing target,
publish geometry, authorize motion, or certify that a controller moved.
"""
from copy import deepcopy
import json
import re
import time

from .selection import parse_selection
from .observe_return_measurement import measurement_digest

REQUEST_TOPIC = '/grasp_mode/contact_station_retirement_request'
ACK_TOPIC = '/grasp_mode/contact_station_retirement_ack'
PARTICIPANTS = ('unknown', 'remote')


def parse_request(value, selection, now_ns, previous=None):
    value = json.loads(value) if isinstance(value, str) else deepcopy(value)
    if not isinstance(value, dict) or type(value.get('version')) is not int or value['version'] != 1:
        raise ValueError('CONTACT_STATION_VERSION_INVALID')
    # XML-RPC carries exact nanoseconds as decimal text. Compare both owner
    # and time bounds against the same canonical selection; keep the request
    # bytes/values unchanged for its two-owner digest and ACK contract.
    selection = parse_selection(selection)
    if (parse_selection(value.get('selection')) != selection
            or selection['mode'] != 'unknown' or selection['strategy'] != 'observe_return'):
        raise ValueError('CONTACT_STATION_SELECTION_CHANGED')
    if value.get('state') not in ('prepare', 'retire') or value.get('stage') != 'MOVE_PREGRASP':
        raise ValueError('CONTACT_STATION_STAGE_INVALID')
    for key in ('request_id', 'session_id', 'target_track_id', 'plan_id'):
        if not isinstance(value.get(key), str) or not value[key] or any(c.isspace() for c in value[key]):
            raise ValueError('CONTACT_STATION_BINDING_INVALID: ' + key)
    for key in ('measurement_sha256', 'plan_sha256'):
        if not isinstance(value.get(key), str) or not re.fullmatch('[0-9a-f]{64}', value[key]):
            raise ValueError('CONTACT_STATION_DIGEST_INVALID')
    for key in ('target_epoch', 'control_epoch_ns', 'plan_source_ns', 'phase_id',
                'phase_started_ns', 'stamp_ns', 'deadline_ns'):
        if type(value.get(key)) is not int or value[key] < (0 if key == 'target_epoch' else 1):
            raise ValueError('CONTACT_STATION_EXACT_BINDING_REQUIRED: ' + key)
    if (not selection['stamp_ns'] < value['plan_source_ns'] <= value['stamp_ns'] <= now_ns < value['deadline_ns']
            or not value['control_epoch_ns'] < value['stamp_ns']
            or not value['phase_started_ns'] <= value['stamp_ns']
            or value['deadline_ns'] - value['stamp_ns'] > 800_000_000):
        raise ValueError('CONTACT_STATION_TIME_INVALID')
    if (value.get('metric_geometry_authority') is not False or value.get('motion_authorized') is not False
            or value.get('controller_motion_confirmed') is not False):
        raise ValueError('CONTACT_STATION_AUTHORITY_FORBIDDEN')
    if previous is None:
        if value['state'] != 'prepare':
            raise ValueError('CONTACT_STATION_PREPARATION_REQUIRED')
    else:
        if (previous['state'] != 'prepare' or value['state'] != 'retire'
                or value['stamp_ns'] < previous['stamp_ns']
                or any(value.get(k) != v for k, v in previous.items() if k not in ('state', 'stamp_ns'))):
            raise ValueError('CONTACT_STATION_PREPARATION_CHANGED')
        receipts = value.get('prepared_acks', {})
        for participant in PARTICIPANTS:
            validate_ack(receipts.get(participant), previous, participant)
    measurement_digest(value)  # Extra fields must be finite JSON, too.
    return value


def ack(request, participant, accepted=True, reason='OK'):
    return dict(version=1, request_id=request['request_id'], state=request['state'],
        participant=participant, request_sha256=measurement_digest(request), accepted=bool(accepted),
        reason=str(reason), metric_geometry_authority=False, motion_authorized=False,
        controller_motion_confirmed=False)


def validate_ack(value, request, participant):
    if (not isinstance(value, dict) or value.get('version') != 1
            or value.get('participant') != participant or participant not in PARTICIPANTS
            or value.get('request_id') != request['request_id'] or value.get('state') != request['state']
            or value.get('request_sha256') != measurement_digest(request) or value.get('accepted') is not True
            or any(value.get(k) is not False for k in (
                'metric_geometry_authority', 'motion_authorized', 'controller_motion_confirmed'))):
        raise ValueError('CONTACT_STATION_ACK_REJECTED: ' + str((value or {}).get('reason', 'missing receipt')))
    return value


def wait_for_pregrasp_state(request, read_state, still_current, now_ns):
    """Join independently delivered task-state and prepare messages, unlocked.

    Only the active preceding planning state may await its active motion state;
    cancellation, manual ownership and other stages are never pending here.
    The request's original short deadline is not renewed.
    """
    if request['state'] != 'prepare':
        return
    cutoff = time.monotonic() + max(0., (request['deadline_ns']-now_ns())*1e-9)
    while True:
        if not still_current() or now_ns() >= request['deadline_ns'] or time.monotonic() >= cutoff:
            raise ValueError('CONTACT_STATION_PREPARE_OWNER_OR_DEADLINE_CHANGED')
        state, active = read_state()
        if active is True and state == 'MOVE_PREGRASP':
            return
        if active is not True or state != 'PLAN_PREGRASP':
            raise ValueError('CONTACT_STATION_PREPARE_STAGE_CHANGED')
        time.sleep(.005)
