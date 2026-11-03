"""Pure, non-authoritative index of recorded task handoffs.

A recorder receiving a publication does not prove that its intended consumer
received or accepted it. Missing observations remain unconfirmed. This module
never produces a plan, changes a task state, or authorizes an action.
"""
from copy import deepcopy
import hashlib
import json
import re


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def integer(value):
    if isinstance(value, bool) or isinstance(value, float):
        return None
    try:
        parsed = int(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return parsed if parsed >= 0 and str(parsed) == str(value) else None


def _selection(value):
    if not isinstance(value, dict):
        return None
    generation, stamp = integer(value.get('generation')), integer(value.get('stamp_ns'))
    if generation is None or not stamp:
        return None
    return dict(generation=generation, stamp_ns=stamp,
                mode=value.get('mode'), strategy=value.get('strategy'))


def _same_selection(candidate, selected):
    return (candidate['generation'] == selected['generation']
            and candidate['stamp_ns'] == selected['stamp_ns']
            and all(candidate.get(k) in (None, selected.get(k)) for k in ('mode', 'strategy')))


def _first_int(*values):
    return next((integer(v) for v in values if integer(v)), None)


def _code(value):
    # Diagnostic prefixes are retained separately; only the task's own terminal
    # code is used to identify its corresponding failed planning report.
    text = str(value or '')
    text = text.removeprefix('execution slot released: ') if hasattr(text, 'removeprefix') else (
        text[len('execution slot released: '):] if text.startswith('execution slot released: ') else text)
    match = re.match(r'([A-Z][A-Z0-9_]+)(?:\b|:)', text)
    return match.group(1) if match else None


def _task_event(p):
    """Validate an observational mirror, never an execution certificate."""
    selected = _selection(p.get('selection'))
    if (p.get('kind') != 'grasp_task_event_v1' or type(p.get('schema_version')) is not int
            or p['schema_version'] != 1 or not isinstance(p.get('task_run_id'), str)
            or re.fullmatch('[0-9a-f]{32}', p['task_run_id']) is None
            or type(p.get('event_seq')) is not int or p['event_seq'] < 1
            or type(p.get('stamp_ns')) is not int or p['stamp_ns'] <= 0
            or type(p.get('stage')) is not int or p['stage'] < 0
            or not isinstance(p.get('state'), str) or not p['state']
            or not isinstance(p.get('frame_id'), str)
            or not isinstance(p.get('message'), str)
            or type(p.get('active')) is not bool or type(p.get('success')) is not bool
            or selected is None or not selected['mode'] or not selected['strategy']
            or p.get('authority') != 'diagnostic_only' or p.get('execution_authorized') is not False):
        raise ValueError('invalid diagnostic task event')
    for name in ('session_id', 'bound_plan_id', 'target_epoch', 'target_track_id',
                 'view_index', 'phase_id', 'planning_stage', 'source_stamp_ns',
                 'measurement_request_id', 'measurement_lease_id'):
        if name not in p:
            raise ValueError('task event lacks explicit nullable context: '+name)
    for name in ('session_id', 'bound_plan_id', 'target_track_id', 'planning_stage',
                 'measurement_request_id', 'measurement_lease_id'):
        if p[name] is not None and (not isinstance(p[name], str) or not p[name]):
            raise ValueError('invalid task context string: '+name)
    for name in ('target_epoch', 'view_index', 'phase_id', 'source_stamp_ns'):
        if p[name] is not None and (type(p[name]) is not int or p[name] < 0):
            raise ValueError('invalid task context integer: '+name)


def _state_key(event):
    summary = event['summary']
    fields = ('frame_id', 'state', 'stage', 'active', 'success', 'message')
    if any(k not in summary for k in fields):
        return None
    return digest([event['event_ns']] + [summary[k] for k in fields])


def normalize_event(topic, payload, locator):
    """Index one original JSON-compatible message without inventing bindings."""
    p = deepcopy(payload)
    if not isinstance(p, dict) or not isinstance(locator, dict):
        raise ValueError('message and evidence locator must be objects')
    task_event = topic == '/grasp/task_event'
    if task_event:
        _task_event(p)
    explicit = _selection(p.get('selection', p.get('grasp_mode_selection')))
    if explicit is None and 'selection_stamp_ns' in p:
        explicit = _selection(dict(generation=p.get('generation'),
                                   stamp_ns=p.get('selection_stamp_ns'), mode=p.get('mode')))
    observe = p.get('observe_return') or {}
    station = p.get('station_compute_wait') or {}
    if not isinstance(observe, dict):
        observe = {}
    if not isinstance(station, dict):
        station = {}
    source = _first_int(p.get('source_stamp_ns'), p.get('snapshot_stamp_ns'),
                        p.get('header_stamp_ns'), p.get('source_ns'))
    event_ns = _first_int(p.get('stamp_ns'), p.get('header_stamp_ns'),
                          p.get('goal_ns'), locator.get('recorded_ns'))
    binding = dict(selection=explicit,
        session_id=p.get('session_id') or observe.get('session_id') or station.get('session_id'),
        target_epoch=integer(p.get('target_epoch', observe.get('target_epoch', station.get('target_epoch')))),
        target_track_id=p.get('target_track_id') or observe.get('target_track_id') or station.get('target_track_id'),
        request_id=p.get('request_id'), request_generation=integer(p.get('generation')) if 'snapshot_stamp_ns' in p else None,
        view_index=integer(p.get('view_index', observe.get('view_index', station.get('view_index')))),
        phase_id=integer(p.get('phase_id', station.get('phase_id'))),
        phase=p.get('phase') or p.get('planning_stage') or observe.get('phase'),
        source_ns=source, lease_id=p.get('lease_id'), token_id=p.get('token_id'),
        plan_id=p.get('plan_id'), goal_id=p.get('goal_id'),
        task_run_id=p.get('task_run_id') if task_event else None,
        event_seq=p.get('event_seq') if task_event else None)
    if task_event:
        binding.update(plan_id=p['bound_plan_id'], request_id=p['measurement_request_id'],
                       lease_id=p['measurement_lease_id'])
    observations = dict(produced='recorded_publication', delivered='recorder_only',
                        accepted=None, rejected=None, completed=None, terminal=None)
    summary = {k: p[k] for k in ('action', 'state', 'participant', 'valid', 'diagnostic',
               'active', 'success', 'reason', 'message', 'qualified', 'code', 'status',
               'error_code', 'error_string', 'commit_reference', 'detected',
               'target_locked', 'target_lost', 'detail') if k in p}
    if 'stage' in p:
        summary['stage'] = p['stage']
    if 'frame_id' in p or 'header_frame_id' in p:
        summary['frame_id'] = p.get('frame_id', p.get('header_frame_id'))
    stage = 'other'
    if 'initial_preflight' in topic:
        stage = 'initial_admission'
    elif 'measurement_request' in topic or 'measurement_ack' in topic:
        stage = 'observation_measurement'
    elif topic.endswith('/observe_return_session'):
        stage = 'session'
    elif topic.endswith('/detector_status') or topic in (
            '/perception/object', '/perception/object_mask', '/grasp_6d/object_geometry'):
        stage = 'current_measurement'
    elif topic.endswith('/near_field_phase'):
        stage = 'contact_phase'
    elif topic.endswith('/frozen_contact_plan'):
        stage = 'contact_freeze'
    elif topic.endswith('/grasp/state') or task_event:
        stage = 'task_state'
        if p.get('state') in ('FAILED', 'SUCCESS', 'HOLDING', 'EMERGENCY_STOP', 'DONE', 'SUCCEEDED', 'CANCELLED'):
            observations['terminal'] = dict(state=p.get('state'), code=_code(p.get('message')),
                success=p.get('success'), active=p.get('active'), physical_success='unverified')
    elif topic.startswith(('/alicia_controller/follow_joint_trajectory/',
                           '/hand_controller/follow_joint_trajectory/')):
        stage = 'arm_motion' if topic.startswith('/alicia_controller/') else 'hand_motion'
        if topic.endswith('/result'):
            observations['completed'] = dict(success=p.get('status') == 3 and p.get('error_code') == 0,
                status=p.get('status'), error_code=p.get('error_code'))
    elif topic == 'runner:result':
        stage = 'runner_result'
        observations['terminal'] = dict(success=p.get('success'), code=_code(p.get('message')))
    elif topic == 'audit:planning':
        outcome = p.get('outcome') or {}
        stage = 'contact_plan' if p.get('planning_origin') == 'observation_view_contact_preview' else 'observation_plan'
        summary.update(planning_origin=p.get('planning_origin'), outcome=outcome,
                       selected=p.get('selected') is not None,
                       stage_counts=(p.get('pipeline_funnel') or {}).get('stage_counts', {}))
        observations['rejected'] = outcome.get('preview_valid') is False
    elif 'plan_enriched' in topic:
        stage = 'contact_plan' if p.get('diagnostic') == 'CONTACT_EXECUTION_PLAN' else 'plan_publication'
        summary['code'] = _code(p.get('diagnostic'))
    if topic.endswith('_ack') and isinstance(p.get('accepted'), bool):
        # An explicit participant ACK is evidence of that participant alone.
        observations.update(accepted=p['accepted'], rejected=not p['accepted'],
                            delivered='explicit_participant_ack')
    identity = dict(topic=topic, message_sha256=digest(p), locator=deepcopy(locator))
    return dict(event_id=digest(identity), **identity, event_ns=event_ns,
                message_sha256_semantics='canonical indexed payload; original bytes are bound by locator file_sha256',
                binding=binding, stage=stage, observations=observations, summary=summary)


def build_task_chain(events, selection, attempt_end_ns=None, task_run_id=None):
    """Correlate a complete finite collection, independent of input ordering.

    ACKs may arrive before requests in the recorder. Correlation is a two-pass
    exact-key join, not a latest-message cache. All duplicate records are kept.
    """
    selected = _selection(selection)
    if selected is None or selected['mode'] is None or selected['strategy'] is None:
        raise ValueError('complete attempt selection is required')
    if attempt_end_ns is not None and (integer(attempt_end_ns) is None or attempt_end_ns < selected['stamp_ns']):
        raise ValueError('attempt end must bound the selected attempt')
    if task_run_id is not None and (not isinstance(task_run_id, str)
            or re.fullmatch('[0-9a-f]{32}', task_run_id) is None):
        raise ValueError('task_run_id must be an exact recorded run ID')
    rows = deepcopy(list(events))
    mirrors, mirror_sequences, mirror_conflicts = {}, {}, set()
    for e in rows:
        if e['topic'] != '/grasp/task_event':
            continue
        b = e['binding']
        pair = (b['task_run_id'], b['event_seq'])
        mirror_sequences.setdefault(pair, set()).add(e['message_sha256'])
    mirror_conflicts = {key for key, hashes in mirror_sequences.items() if len(hashes) != 1}
    for e in rows:
        if e['topic'] == '/grasp/task_event':
            b = e['binding']
            stamp, recorded = e['event_ns'], e['locator'].get('recorded_ns', 0)
            clock_valid = (stamp >= selected['stamp_ns']
                and (attempt_end_ns is None or stamp <= attempt_end_ns)
                and (recorded <= 0 or stamp <= recorded))
            if (b['task_run_id'], b['event_seq']) not in mirror_conflicts and clock_valid:
                mirrors.setdefault(_state_key(e), []).append(e)
    # Request IDs are only trusted when their request itself explicitly names
    # this selection. Ambiguous reused IDs are never joined.
    requests = {}
    for e in rows:
        b = e['binding']
        if e['topic'].endswith('_request') and b['request_id'] is not None:
            requests.setdefault(str(b['request_id']), []).append(e)
    current, excluded, unbound = [], [], []
    for e in rows:
        b = e['binding']; explicit = b['selection']; basis = None; reason = None
        state_matches = mirrors.get(_state_key(e), []) if e['topic'] == '/grasp/state' else []
        mirror_pairs = {(x['binding']['task_run_id'], x['binding']['event_seq']) for x in state_matches}
        mirror = state_matches[0] if len(mirror_pairs) == 1 else None
        if e['topic'] == '/grasp/task_event' and (b['task_run_id'], b['event_seq']) in mirror_conflicts:
            reason = 'conflicting_task_event_sequence'
        elif e['topic'] == '/grasp/state' and len(mirror_pairs) > 1:
            reason = 'ambiguous_task_state_mirror'
        elif mirror is not None:
            mb = mirror['binding']
            if not _same_selection(mb['selection'], selected):
                reason = 'task_state_for_different_selection'
            elif task_run_id is not None and mb['task_run_id'] != task_run_id:
                reason = 'different_task_run'
            else:
                basis = 'exact_task_state_mirror_join'
                e['task_event_id'] = mirror['event_id']
                e['joined_task_binding'] = deepcopy(mb)
        elif explicit is not None:
            if _same_selection(explicit, selected):
                if task_run_id is not None and b['task_run_id'] is not None and b['task_run_id'] != task_run_id:
                    reason = 'different_task_run'
                else:
                    basis = 'explicit_task_run' if e['topic'] == '/grasp/task_event' else 'explicit_selection'
            else:
                reason = 'different_selection'
        elif e['topic'].endswith('_ack') and b['request_id'] is not None:
            matches = requests.get(str(b['request_id']), [])
            identities = {digest(x['binding']) for x in matches}
            request = matches[0] if len(identities) == 1 else None
            if request is not None and request['binding']['selection'] is not None:
                rb = request['binding']
                if not _same_selection(rb['selection'], selected):
                    reason = 'ack_for_different_selection'
                elif any(b[k] is not None and rb[k] is not None and b[k] != rb[k]
                         for k in ('lease_id', 'token_id', 'target_epoch', 'target_track_id', 'view_index')) or any(
                         e['summary'].get(k) is not None and request['summary'].get(k) is not None
                         and e['summary'][k] != request['summary'][k] for k in ('state', 'action')):
                    reason = 'ack_binding_conflict'
                else:
                    basis = 'exact_request_ack_join'
                    e['request_event_id'] = request['event_id']
                    # Expose inherited metadata separately, never as original fields.
                    e['joined_request_binding'] = deepcopy(rb)
        else:
            source_required = e['stage'] in ('task_state', 'plan_publication', 'contact_plan',
                                           'contact_phase', 'current_measurement')
            stamp = b['source_ns'] if source_required else (b['source_ns'] or e['event_ns'])
            if stamp is not None and stamp < selected['stamp_ns']:
                reason = 'source_precedes_selection'
            elif stamp is not None and attempt_end_ns is not None and stamp > attempt_end_ns:
                reason = 'source_after_attempt_window'
            elif stamp is not None and e['locator'].get('recorded_ns', 0) > 0 and stamp > e['locator']['recorded_ns']:
                reason = 'source_after_recorder_clock'
            elif stamp is not None and e['stage'] in ('task_state', 'arm_motion', 'hand_motion',
                    'plan_publication', 'contact_plan', 'contact_phase', 'contact_freeze',
                    'current_measurement', 'runner_result'):
                basis = 'time_window_only_not_full_identity'
        if e['topic'] == '/grasp/task_event' or mirror is not None:
            # The mirrored state's stamp is its own event time, not the bound
            # contact plan's possibly older source stamp.
            stamp = e['event_ns']
            if stamp is not None and stamp < selected['stamp_ns']:
                reason = reason or 'source_precedes_selection'
            elif stamp is not None and attempt_end_ns is not None and stamp > attempt_end_ns:
                reason = reason or 'source_after_attempt_window'
            elif stamp is not None and e['locator'].get('recorded_ns', 0) > 0 and stamp > e['locator']['recorded_ns']:
                reason = reason or 'source_after_recorder_clock'
        if reason:
            e['excluded_reason'] = reason; excluded.append(e)
        elif basis:
            e['correlation_basis'] = basis; current.append(e)
        else:
            e['unconfirmed_reason'] = 'no_exact_selection_or_request_join'; unbound.append(e)
    key = lambda e: (e['event_ns'] if e['event_ns'] is not None else 2**63,
                     e['locator'].get('recorded_ns', 0), e['event_id'])
    current.sort(key=key); excluded.sort(key=key); unbound.sort(key=key)
    duplicates = {}
    for e in current:
        duplicates.setdefault(e['topic'] + ':' + e['message_sha256'], []).append(e['event_id'])
    task_runs = sorted({e['binding']['task_run_id'] for e in current
                        if e['topic'] == '/grasp/task_event'})
    effective_run = task_run_id or (task_runs[0] if len(task_runs) == 1 else None)
    exact_present = any(e['topic'] == '/grasp/task_event'
        and _same_selection(e['binding']['selection'], selected) for e in rows) or task_run_id is not None
    task_terminals = [e for e in current if e['stage'] == 'task_state' and e['observations']['terminal']
        and (not exact_present or (effective_run is not None
            and (e.get('joined_task_binding') or e['binding']).get('task_run_id') == effective_run))]
    if exact_present:
        # A simulator or a real clock may give successive states equal stamps.
        # The producer's run-local sequence, not recorder delivery, orders them.
        task_terminals.sort(key=lambda e: (
            (e.get('joined_task_binding') or e['binding'])['event_seq'],
            e['topic'] != '/grasp/task_event', e['event_id']))
    terminal = task_terminals[0] if task_terminals else None
    first = None
    if terminal is not None and terminal['observations']['terminal']['success'] is False:
        code = terminal['observations']['terminal']['code']
        terminal_binding = terminal.get('joined_task_binding') or terminal['binding']
        causes = [e for e in current if e['stage'] == 'contact_plan'
                  and e['correlation_basis'] == 'explicit_selection'
                  and e['summary'].get('outcome', {}).get('code') == code
                  and e['summary'].get('selected') is False
                  and e['observations']['rejected'] is True
                  and e['binding']['source_ns'] is not None
                  and (effective_run is None or terminal_binding.get('session_id') is not None
                       and e['binding']['session_id'] == terminal_binding['session_id'])
                  and selected['stamp_ns'] <= e['binding']['source_ns'] <= terminal['event_ns']]
        if causes:
            cause = max(causes, key=lambda e:e['binding']['source_ns'])
            first = dict(stage='contact_plan', classification='not_produced_recorded_rejection',
                code=code, evidence_event_id=cause['event_id'], terminal_event_id=terminal['event_id'],
                incorrect_rejection_proven=False,
                explanation='Current planning report selected no candidate; matching task terminal exists. '
                            'This does not prove a physical collision or an erroneous gate.')
        else:
            first = dict(stage='unconfirmed', classification='terminal_without_confirmed_upstream_cause',
                         code=code, terminal_event_id=terminal['event_id'])
    stages = []
    for stage in ('initial_admission', 'observation_plan', 'arm_motion', 'observation_measurement',
                  'current_measurement', 'contact_phase', 'contact_plan', 'contact_freeze', 'hand_motion', 'task_state', 'runner_result'):
        group = [e for e in current if e['stage'] == stage]
        stages.append(dict(stage=stage, evidence_event_ids=[e['event_id'] for e in group],
            produced='recorded' if group else 'unconfirmed',
            delivered_to_consumer='explicit_ack_present' if any(e['observations']['delivered']=='explicit_participant_ack' for e in group) else 'unconfirmed',
            accepted_event_ids=[e['event_id'] for e in group if e['observations']['accepted'] is True],
            completed_event_ids=[e['event_id'] for e in group if e['observations']['completed'] is not None],
            rejected_event_ids=[e['event_id'] for e in group if e['observations']['rejected'] is True]))
    return dict(schema=1, kind='recorded_task_chain', selection=deepcopy(selection), attempt_end_ns=attempt_end_ns,
        task_run_id=effective_run, recorded_task_run_ids=task_runs,
        task_run_ambiguity=len(task_runs)>1 and task_run_id is None,
        conflicting_task_event_sequences=[dict(task_run_id=r, event_seq=s) for r,s in sorted(mirror_conflicts)],
        events=current, excluded_events=excluded, unbound_events=unbound, stages=stages,
        duplicate_record_groups=[v for v in duplicates.values() if len(v)>1],
        first_task_terminal=terminal, first_interruption=first,
        later_failure_event_ids=[e['event_id'] for e in current if terminal is not None
            and e['event_ns'] is not None and e['event_ns']>terminal['event_ns']
            and (e['observations']['rejected'] is True or e['summary'].get('code')=='TARGET_LOST'
                 or (e['observations']['terminal'] or {}).get('success') is False)],
        post_terminal_recorded_failure_event_ids=[e['event_id'] for e in current if terminal is not None
            and e['locator'].get('recorded_ns', 0)>terminal['locator'].get('recorded_ns', 0)
            and (e['observations']['rejected'] is True or e['summary'].get('code')=='TARGET_LOST'
                 or e['summary'].get('outcome', {}).get('code')=='TARGET_LOST'
                 or (e['observations']['terminal'] or {}).get('success') is False)],
        contact_or_motion_authorized=False, physical_grasp_success='not_assessed',
        limitations=['Recorder arrival is not consumer delivery or callback acceptance.',
            'Missing records are unconfirmed, not proof that a publication never occurred.',
            'Time-window-only events cannot establish selection/session/target identity alone; an interleaved attempt requires explicit additional evidence.',
            'Task-event mirrors identify state publications only; they do not grant motion or prove another node accepted them.',
            'Request generation is not selection generation; source time is not bag arrival time.',
            'A later invalid preview cannot replace the first recorded task terminal.'])
