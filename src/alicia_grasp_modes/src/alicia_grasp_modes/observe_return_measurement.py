"""Identity-only measurement handoff for one already bound observation move.

These receipts never authorize motion or replace a measured geometric source.
The existing observe-return session remains paused until its usual reached-view
registration. Ordinary target retirement is unchanged outside this handoff.
"""
from copy import deepcopy
import hashlib
import json
import re
import math

from .selection import parse_selection
from .observe_return_session import exact_ns

REQUEST_TOPIC = '/grasp_mode/observe_return_measurement_request'
ACK_TOPIC = '/grasp_mode/observe_return_measurement_ack'
PARTICIPANTS = ('unknown', 'remote')
STATES = ('executing', 'requalifying', 'qualified', 'revoked')
BINDING_FIELDS = ('lease_id', 'selection', 'session_id', 'target_epoch',
    'target_track_id', 'origin_plan_id', 'origin_snapshot_stamp_ns',
    'view_plan_id', 'view_source_stamp_ns', 'view_index', 'plan_sha256',
    'control_epoch_ns', 'deadline_ns', 'motion_kind')

STATION_REQUALIFICATION = 'observation_station_requalification'
ACTUATION_BOOTSTRAP = 'actuation_bootstrap'


def bootstrap_source_deadline(request, validity_sec):
    """Check a bootstrap against the participant's original plan lifetime."""
    if (type(validity_sec) not in (int, float) or not math.isfinite(validity_sec)
            or not 0. < validity_sec <= 600.):
        raise ValueError('ACTUATION_BOOTSTRAP_SOURCE_VALIDITY_INVALID')
    deadline = request['view_source_stamp_ns'] + round(validity_sec * 1e9)
    if (request.get(ACTUATION_BOOTSTRAP, {}).get('source_deadline_ns') != deadline
            or request['deadline_ns'] != deadline):
        raise ValueError('ACTUATION_BOOTSTRAP_SOURCE_DEADLINE_CHANGED')
    return deadline


def station_requalification_key(contract):
    b = contract['binding']
    return (measurement_digest(b['selection']), b['session_id'], b['view_plan_id'], b['view_index'])


def station_requalification_witness(contract, stamp_ns, q, now_ns):
    """Describe a measured drift, never establish a local registered parent."""
    from .station_compute_reacquisition import validate_contract
    validate_contract(contract, binding=contract['binding'], now_ns=now_ns)
    q = list(q)
    if (type(stamp_ns) is not int or not contract['source_ns'] < stamp_ns <= now_ns
            or now_ns-stamp_ns >= 800_000_000 or len(q) != 6
            or any(type(v) not in (int, float) or not math.isfinite(v) for v in q)
            or max(abs(a-b) for a,b in zip(q, contract['binding']['station_q_rad'])) <= .004):
        raise ValueError('STATION_REQUALIFICATION_DRIFT_REQUIRED')
    return dict(version=1, parent_contract=deepcopy(contract), feedback_stamp_ns=stamp_ns,
        feedback_q_rad=q, deadline_ns=min(contract['identity_deadline_ns'],
            contract['phase_deadline_ns'], contract['session_deadline_ns']))


def validate_station_requalification(value, *, parent, feedback, now_ns):
    """Require a caller-owned parent and actual local SDK feedback, not claims."""
    if not isinstance(value, dict) or value.get('parent_contract') != parent:
        raise ValueError('STATION_REQUALIFICATION_PARENT_UNREGISTERED')
    expected = station_requalification_witness(parent, value.get('feedback_stamp_ns'),
        value.get('feedback_q_rad', ()), value.get('feedback_stamp_ns', 0))
    if value != expected or type(value.get('version')) is not int:
        raise ValueError('STATION_REQUALIFICATION_WITNESS_CHANGED')
    if not value['feedback_stamp_ns'] <= now_ns < value['deadline_ns']:
        raise ValueError('STATION_REQUALIFICATION_EXPIRED')
    if not any(ns == value['feedback_stamp_ns'] and list(q) == value['feedback_q_rad']
               for ns,q in feedback):
        raise ValueError('STATION_REQUALIFICATION_FEEDBACK_UNPROVEN')
    return deepcopy(value)


def measurement_digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
        allow_nan=False).encode('utf-8')).hexdigest()


def parse_measurement_request(value, selection, now_ns, previous=None):
    """Validate and normalize a task receipt; never infer missing authority."""
    if isinstance(value, str):
        value = json.loads(value)
    if not isinstance(value, dict) or type(value.get('version')) is not int or value['version'] != 1:
        raise ValueError('OBSERVATION_MEASUREMENT_VERSION_INVALID')
    result = deepcopy(value)
    result['selection'] = parse_selection(value.get('selection'))
    if (result['selection'] != parse_selection(selection)
            or result['selection']['mode'] != 'unknown'
            or result['selection']['strategy'] != 'observe_return'):
        raise ValueError('OBSERVATION_MEASUREMENT_SELECTION_CHANGED')
    if value.get('state') not in STATES or value.get('motion_kind') not in ('observation', 'return', ACTUATION_BOOTSTRAP):
        raise ValueError('OBSERVATION_MEASUREMENT_STATE_INVALID')
    for key in ('lease_id', 'request_id', 'session_id', 'target_track_id', 'origin_plan_id', 'view_plan_id'):
        text = value.get(key)
        if not isinstance(text, str) or not text or any(c.isspace() for c in text):
            raise ValueError('OBSERVATION_MEASUREMENT_BINDING_INVALID: ' + key)
    if not isinstance(value.get('plan_sha256'), str) or not re.fullmatch('[0-9a-f]{64}', value['plan_sha256']):
        raise ValueError('OBSERVATION_MEASUREMENT_PLAN_DIGEST_INVALID')
    for key in ('origin_snapshot_stamp_ns', 'view_source_stamp_ns', 'control_epoch_ns', 'deadline_ns', 'stamp_ns', 'view_index'):
        result[key] = exact_ns(value.get(key), key)
    result['target_epoch'] = exact_ns(value.get('target_epoch'), 'target_epoch', 0)
    for key in ('arrival_stamp_ns', 'capture_deadline_ns'):
        result[key] = exact_ns(value.get(key, 0), key, 0)
    if (type(now_ns) is not int
            or not result['selection']['stamp_ns'] < result['origin_snapshot_stamp_ns']
            <= result['view_source_stamp_ns'] <= result['stamp_ns'] <= now_ns
            or not result['control_epoch_ns'] < result['stamp_ns']
            or (value['state'] != 'revoked' and not result['stamp_ns'] < result['deadline_ns'])
            or result['deadline_ns'] - result['stamp_ns'] > 600_000_000_000
            or (value['state'] != 'revoked' and now_ns >= result['deadline_ns'])):
        raise ValueError('OBSERVATION_MEASUREMENT_TIME_INVALID')
    if (value.get('metric_geometry_authority') is not False
            or value.get('motion_authorized') is not False):
        raise ValueError('OBSERVATION_MEASUREMENT_AUTHORITY_FORBIDDEN')
    bootstrap = value.get(ACTUATION_BOOTSTRAP)
    if value['motion_kind'] == ACTUATION_BOOTSTRAP:
        if (not isinstance(bootstrap, dict)
                or set(bootstrap) != {'version', 'source_deadline_ns'}
                or type(bootstrap.get('version')) is not int or bootstrap['version'] != 1
                or type(bootstrap.get('source_deadline_ns')) is not int
                or bootstrap['source_deadline_ns'] != result['deadline_ns']
                or not result['session_id'].startswith('bootstrap:')
                or result['session_id'] == 'bootstrap:'
                or result['view_index'] != 1
                or result['origin_plan_id'] != result['view_plan_id']
                or result['origin_snapshot_stamp_ns'] != result['view_source_stamp_ns']
                or value.get(STATION_REQUALIFICATION) is not None):
            raise ValueError('ACTUATION_BOOTSTRAP_BINDING_INVALID')
    elif bootstrap is not None or ACTUATION_BOOTSTRAP in value:
        raise ValueError('ACTUATION_BOOTSTRAP_KIND_REQUIRED')
    if previous is not None and bootstrap != previous.get(ACTUATION_BOOTSTRAP):
        raise ValueError('ACTUATION_BOOTSTRAP_BINDING_CHANGED')
    restation = value.get(STATION_REQUALIFICATION)
    if restation is not None:
        parent = restation.get('parent_contract', {}) if isinstance(restation, dict) else {}
        # This is wire validation only. Every participant must additionally
        # compare the parent and feedback with its independently held record.
        validate_station_requalification(restation, parent=parent,
            feedback=[(restation.get('feedback_stamp_ns'), restation.get('feedback_q_rad'))],
            now_ns=(min(now_ns, restation['deadline_ns']-1) if value['state']=='revoked' else now_ns))
        binding = parent['binding']
        if (result['motion_kind'] != 'observation' or result['lease_id'] == binding['lease_id']
                or any(result[k] != binding[k] for k in ('selection','session_id','target_epoch',
                    'target_track_id','view_plan_id','view_index','control_epoch_ns'))
                or result['deadline_ns'] != parent['session_deadline_ns']
                or result['stamp_ns'] <= restation['feedback_stamp_ns']
                or result['capture_deadline_ns'] > restation['deadline_ns']):
            raise ValueError('STATION_REQUALIFICATION_REQUEST_CHANGED')
    if previous is not None and value.get(STATION_REQUALIFICATION) != previous.get(STATION_REQUALIFICATION):
        raise ValueError('STATION_REQUALIFICATION_BINDING_CHANGED')
    if previous is None:
        if value['state'] != 'executing' or result['arrival_stamp_ns'] or result['capture_deadline_ns']:
            raise ValueError('OBSERVATION_MEASUREMENT_MUST_START_EXECUTING')
    else:
        if (any(result[key] != previous[key] for key in BINDING_FIELDS)
                or result['stamp_ns'] <= previous['stamp_ns']
                or result['request_id'] == previous['request_id']):
            raise ValueError('OBSERVATION_MEASUREMENT_BINDING_CHANGED')
        transitions = {'executing': ('requalifying', 'revoked'),
                       'requalifying': ('qualified', 'revoked'), 'qualified': (), 'revoked': ()}
        if value['state'] not in transitions[previous['state']]:
            raise ValueError('OBSERVATION_MEASUREMENT_TRANSITION_INVALID')
    if value['state'] in ('requalifying', 'qualified'):
        if not (result['view_source_stamp_ns'] < result['arrival_stamp_ns'] <= result['stamp_ns']
                < result['capture_deadline_ns'] <= result['deadline_ns']
                and result['capture_deadline_ns'] - result['arrival_stamp_ns'] <= 10_000_000_000):
            raise ValueError('OBSERVATION_MEASUREMENT_ARRIVAL_INVALID')
        settle = result.get('settle')
        if (not isinstance(settle, dict)
                or exact_ns(settle.get('epoch_ns'), 'settle epoch') != result['control_epoch_ns']
                or not 0 < exact_ns(settle.get('stationary_window_start_ns'), 'settle start')
                < exact_ns(settle.get('stationary_window_end_ns'), 'settle end') <= result['arrival_stamp_ns']):
            raise ValueError('OBSERVATION_MEASUREMENT_SETTLE_INVALID')
        if (previous is not None and previous['state'] == 'executing'
                and exact_ns(settle['stationary_window_start_ns'], 'settle start') <= previous['stamp_ns']):
            raise ValueError('OBSERVATION_MEASUREMENT_SETTLE_PREDATES_MOTION')
        if previous is not None and previous['state'] == 'requalifying':
            if any(result.get(k) != previous.get(k) for k in ('arrival_stamp_ns', 'capture_deadline_ns', 'settle')):
                raise ValueError('OBSERVATION_MEASUREMENT_CAPTURE_BINDING_CHANGED')
        if value['state'] == 'qualified':
            qualified_source_stamps(result.get('source_stamps_ns'), result, now_ns)
    return result


def qualified_source_stamps(values, request, now_ns):
    if (not isinstance(values, (tuple, list)) or len(values) < 3
            or any(type(ns) is not int for ns in values)
            or any(b <= a for a, b in zip(values, values[1:]))
            or not request['arrival_stamp_ns'] < values[0] <= values[-1] <= now_ns
            or values[-1] >= request['capture_deadline_ns']):
        raise ValueError('OBSERVATION_MEASUREMENT_FRESH_FRAMES_REQUIRED')
    return list(values)


def measurement_ack(request, participant, *, accepted, reason='OK', qualified=False,
                    source_stamps_ns=None):
    if participant not in PARTICIPANTS or type(accepted) is not bool or type(qualified) is not bool:
        raise ValueError('OBSERVATION_MEASUREMENT_ACK_INVALID')
    return dict(version=1, participant=participant, request_id=request['request_id'],
        lease_id=request['lease_id'], state=request['state'],
        request_sha256=measurement_digest(request), accepted=accepted, reason=str(reason),
        qualified=qualified, source_stamps_ns=list(source_stamps_ns or []),
        metric_geometry_authority=False, motion_authorized=False)


def validate_measurement_ack(ack, request, participant, now_ns, *, require_qualified=False):
    if (not isinstance(ack, dict) or type(ack.get('version')) is not int or ack['version'] != 1
            or ack.get('participant') != participant or participant not in PARTICIPANTS
            or any(ack.get(key) != request[key] for key in ('request_id', 'lease_id', 'state'))
            or ack.get('request_sha256') != measurement_digest(request)
            or ack.get('accepted') is not True or type(ack.get('qualified')) is not bool
            or ack.get('metric_geometry_authority') is not False
            or ack.get('motion_authorized') is not False):
        reason = ack.get('reason', 'binding mismatch') if isinstance(ack, dict) else 'binding mismatch'
        raise ValueError('OBSERVATION_MEASUREMENT_ACK_REJECTED: ' + str(reason))
    if now_ns >= request['deadline_ns']:
        raise ValueError('OBSERVATION_MEASUREMENT_EXPIRED')
    recovery = request.get(STATION_REQUALIFICATION)
    if recovery is not None and request['state'] != 'revoked' and now_ns >= recovery['deadline_ns']:
        raise ValueError('STATION_REQUALIFICATION_EXPIRED')
    if require_qualified:
        if ack['qualified'] is not True or request['state'] != 'requalifying' or now_ns >= request['capture_deadline_ns']:
            raise ValueError('OBSERVATION_MEASUREMENT_NOT_QUALIFIED')
        qualified_source_stamps(ack.get('source_stamps_ns'), request, now_ns)
    return deepcopy(ack)
