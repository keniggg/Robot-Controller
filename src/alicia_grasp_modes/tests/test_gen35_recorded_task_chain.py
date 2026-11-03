"""Offline evidence indexing: messages cannot manufacture consumer authority."""
from copy import deepcopy
import importlib.util
from pathlib import Path

import pytest

from alicia_grasp_modes.recorded_task_chain import build_task_chain, normalize_event


SELECTION = dict(mode='unknown', strategy='observe_return', generation=6, stamp_ns=1000)
CODE = 'NEAR_FIELD_NO_HARD_SAFE_CANDIDATE'


def event(topic, payload, recorded=2000, index=0):
    return normalize_event(topic, payload, dict(path='closed.bag', file_sha256='a'*64,
        topic=topic, topic_message_index=index, recorded_ns=recorded))


def request(**changes):
    body = dict(selection=deepcopy(SELECTION), request_id='r1', lease_id='same-lease',
                target_epoch=3, target_track_id='g1-t3', view_index=1, state='qualified', stamp_ns=1200)
    body.update(changes)
    return event('/grasp_mode/observe_return_measurement_request', body, 2200)


def ack(**changes):
    body = dict(request_id='r1', lease_id='same-lease', participant='remote',
                state='qualified', accepted=True)
    body.update(changes)
    return event('/grasp_mode/observe_return_measurement_ack', body, 2100)


def planning(**changes):
    body = dict(grasp_mode_selection=deepcopy(SELECTION), generation=7, request_id=15,
        snapshot_stamp_ns=1500, target_epoch=3, planning_origin='observation_view_contact_preview',
        selected=None, outcome=dict(code=CODE, preview_valid=False, valid_plan=False),
        observe_return=dict(session_id='current-session', target_track_id='g1-t3', view_index=1))
    body.update(changes)
    return event('audit:planning', body, 1700)


def terminal(stamp=1800, code=CODE, **changes):
    body = dict(header_stamp_ns=stamp, state='FAILED', active=False, success=False,
                message='execution slot released: '+code+': rejected')
    body.update(changes)
    return event('/grasp/state', body, stamp+100)


def test_delayed_out_of_order_request_ack_join_is_exact_and_order_independent():
    rows = [ack(), request(), planning(), terminal()]
    first = build_task_chain(rows, SELECTION)
    second = build_task_chain(list(reversed(rows)), SELECTION)
    assert first == second
    joined = next(e for e in first['events'] if e['topic'].endswith('_ack'))
    assert joined['correlation_basis'] == 'exact_request_ack_join'
    assert joined['binding']['selection'] is None
    assert joined['joined_request_binding']['selection'] == SELECTION
    assert joined['observations']['accepted'] is True
    assert first['first_interruption']['classification'] == 'not_produced_recorded_rejection'


def test_duplicate_recording_is_retained_and_never_becomes_a_second_action():
    first = request(); second = deepcopy(first)
    second['locator']['topic_message_index'] = 1
    second = event(first['topic'], dict(selection=SELECTION, request_id='r1', lease_id='same-lease',
        target_epoch=3, target_track_id='g1-t3', view_index=1, state='qualified', stamp_ns=1200), 2200, 1)
    result = build_task_chain([first, second, ack()], SELECTION)
    assert len(result['events']) == 3
    assert len(result['duplicate_record_groups']) == 1
    assert not result['contact_or_motion_authorized']


@pytest.mark.parametrize('field,value', [('lease_id','other'), ('target_epoch',4),
                                       ('target_track_id','other'), ('view_index',2)])
def test_ack_cannot_join_conflicting_request_binding(field, value):
    result = build_task_chain([request(), ack(**{field:value})], SELECTION)
    assert result['excluded_events'][0]['excluded_reason'] == 'ack_binding_conflict'
    assert not any(e['observations']['accepted'] for e in result['events'])


def test_reused_request_id_across_selections_is_ambiguous_not_latest_wins():
    old = dict(SELECTION, generation=5, stamp_ns=100)
    result = build_task_chain([request(), request(selection=old), ack()], SELECTION)
    assert len(result['unbound_events']) == 1
    assert result['unbound_events'][0]['topic'].endswith('_ack')


def test_old_latched_generation_and_source_cannot_replace_current_terminal():
    old_selection = dict(SELECTION, generation=5, stamp_ns=100)
    old_terminal = terminal(500, 'TARGET_LOST')
    result = build_task_chain([request(selection=old_selection), old_terminal,
        planning(), terminal(), terminal(1900, 'TARGET_LOST')], SELECTION)
    assert len(result['excluded_events']) == 2
    assert result['first_task_terminal']['observations']['terminal']['code'] == CODE
    assert result['first_interruption']['code'] == CODE
    assert result['later_failure_event_ids']


def test_old_preview_delivered_after_terminal_keeps_source_age_and_cannot_overwrite():
    old = event('/grasp_6d/preview_plan_enriched', dict(header_stamp_ns=1200, valid=False,
        diagnostic='TARGET_LOST: old observation plan'), 2500)
    result = build_task_chain([planning(), terminal(), old], SELECTION)
    indexed = next(e for e in result['events'] if e['topic']==old['topic'])
    assert indexed['event_ns'] == 1200 and indexed['locator']['recorded_ns'] == 2500
    assert result['first_interruption']['code'] == CODE
    assert indexed['event_id'] in result['post_terminal_recorded_failure_event_ids']


def test_object_and_mask_recording_never_proves_a_ready_join():
    rows = [event(t, dict(header_stamp_ns=1500, detected=True), 1600)
            for t in ('/perception/object', '/perception/object_mask')]
    result = build_task_chain(rows, SELECTION)
    stage = next(s for s in result['stages'] if s['stage']=='current_measurement')
    assert len(stage['evidence_event_ids']) == 2
    assert stage['delivered_to_consumer'] == 'unconfirmed'
    assert stage['accepted_event_ids'] == []


def test_current_request_generation_is_not_misread_as_selection_generation():
    result = build_task_chain([planning(), terminal()], SELECTION)
    row = next(e for e in result['events'] if e['stage']=='contact_plan')
    assert row['binding']['request_generation'] == 7
    assert row['binding']['selection']['generation'] == 6
    assert result['first_interruption']['code'] == CODE


def test_recorded_publication_does_not_prove_task_delivery_or_acceptance():
    result = build_task_chain([planning()], SELECTION)
    stage = next(s for s in result['stages'] if s['stage']=='contact_plan')
    assert stage['produced'] == 'recorded'
    assert stage['delivered_to_consumer'] == 'unconfirmed'
    assert not stage['accepted_event_ids']
    assert result['first_interruption'] is None


@pytest.mark.parametrize('rows', [[], [terminal()], [planning(snapshot_stamp_ns=900), terminal()]])
def test_missing_or_stale_upstream_proof_never_invents_a_failure_cause(rows):
    result = build_task_chain(rows, SELECTION)
    if rows:
        assert result['first_interruption']['classification'] == 'terminal_without_confirmed_upstream_cause'
    else:
        assert result['first_interruption'] is None
    assert next(s for s in result['stages'] if s['stage']=='contact_freeze')['produced'] == 'unconfirmed'


def test_matching_failure_code_does_not_claim_erroneous_rejection_or_collision():
    result = build_task_chain([planning(), terminal()], SELECTION)
    assert result['first_interruption']['incorrect_rejection_proven'] is False
    assert result['physical_grasp_success'] == 'not_assessed'


def test_input_and_output_are_owned_copies():
    rows = [planning(), terminal()]; before = deepcopy(rows)
    result = build_task_chain(rows, SELECTION)
    result['events'][0]['binding']['selection']['generation'] = 999
    assert rows == before and SELECTION['generation'] == 6


def test_inexact_float_selection_stamp_is_rejected():
    with pytest.raises(ValueError):
        build_task_chain([], dict(SELECTION, stamp_ns=1000.0))


@pytest.mark.parametrize('stamp', [None, 0])
def test_missing_task_source_cannot_use_recorder_time_to_become_current_terminal(stamp):
    missing = event('/grasp/state', dict(header_stamp_ns=stamp, state='FAILED',
        active=False, success=False, message='TARGET_LOST'), recorded=2000)
    result = build_task_chain([missing], SELECTION)
    assert len(result['unbound_events']) == 1
    assert result['first_task_terminal'] is None and result['first_interruption'] is None


@pytest.mark.parametrize('state', ['SUCCESS', 'HOLDING'])
def test_project_terminal_states_only_report_software_state_not_physical_success(state):
    result = build_task_chain([terminal(state=state, success=True)], SELECTION)
    assert result['first_task_terminal']['observations']['terminal']['state'] == state
    assert result['first_task_terminal']['observations']['terminal']['physical_success'] == 'unverified'
    assert result['physical_grasp_success'] == 'not_assessed'


def test_action_result_is_completion_not_admission_and_hand_is_not_arm():
    payload = dict(goal_id='goal', goal_ns=1200, header_stamp_ns=1700, status=3, error_code=0)
    arm = event('/alicia_controller/follow_joint_trajectory/result', payload)
    hand = event('/hand_controller/follow_joint_trajectory/result', payload)
    assert arm['observations']['completed']['success']
    assert arm['observations']['accepted'] is None
    assert hand['stage'] == 'hand_motion'
    assert hand['observations']['completed']['success']


@pytest.mark.parametrize('recorded,end,reason', [(2000,None,'source_after_recorder_clock'),
                                              (4000,2500,'source_after_attempt_window')])
def test_future_terminal_cannot_replace_attempt_terminal(recorded, end, reason):
    future = event('/grasp/state', dict(header_stamp_ns=3000, state='FAILED', active=False,
        success=False, message='TARGET_LOST'), recorded)
    result = build_task_chain([planning(), terminal(), future], SELECTION, attempt_end_ns=end)
    assert result['excluded_events'][0]['excluded_reason'] == reason
    assert result['first_interruption']['code'] == CODE


def test_cli_import_is_inert_and_topic_allowlist_excludes_temperature_diagnostics():
    path = Path(__file__).resolve().parents[1]/'tools/inspect_recorded_task_chain.py'
    spec = importlib.util.spec_from_file_location('gen35_index_cli', str(path))
    cli = importlib.util.module_from_spec(spec); spec.loader.exec_module(cli)
    assert not any('diagnostic' in topic or 'temperature' in topic for topic in cli.TOPICS)
    assert '/alicia_d/accepted_joint_states' in cli.TOPICS
    assert '/grasp_6d/frozen_contact_plan' in cli.TOPICS
    assert '/hand_controller/follow_joint_trajectory/goal' in cli.TOPICS
    assert '/hand_controller/follow_joint_trajectory/result' in cli.TOPICS
    assert '/grasp/task_event' in cli.JSON_TOPICS


RUN = '1'*32


def task_event_body(**changes):
    body = dict(kind='grasp_task_event_v1', schema_version=1, task_run_id=RUN, event_seq=1,
        stamp_ns=1800, frame_id='', state='FAILED', stage=9, active=False, success=False,
        message='execution slot released: '+CODE+': rejected', selection=deepcopy(SELECTION),
        session_id='current-session', bound_plan_id='plan', target_epoch=3, target_track_id='g1-t3',
        view_index=1, phase_id=4, planning_stage='contact_replan', source_stamp_ns=1500,
        measurement_request_id='r1', measurement_lease_id='same-lease',
        authority='diagnostic_only', execution_authorized=False)
    body.update(changes)
    return body


def mirrored_state(body):
    return event('/grasp/state', dict(header_stamp_ns=body['stamp_ns'],
        header_frame_id=body['frame_id'], **{k:body[k] for k in
        ('state','stage','active','success','message')}), recorded=2100)


def test_task_event_exact_state_join_survives_arrival_order_and_keeps_source_distinct():
    body=task_event_body()
    mirror=event('/grasp/task_event',body,recorded=2200)
    state=mirrored_state(body)
    result=build_task_chain([mirror,planning(),state],SELECTION)
    assert result == build_task_chain([state,planning(),mirror],SELECTION)
    joined=next(e for e in result['events'] if e['topic']=='/grasp/state')
    assert joined['correlation_basis']=='exact_task_state_mirror_join'
    assert joined['joined_task_binding']['task_run_id']==RUN
    assert joined['binding']['task_run_id'] is None
    assert mirror['event_ns']==1800 and mirror['binding']['source_ns']==1500
    assert result['first_interruption']['code']==CODE
    assert result['task_run_id']==RUN and not result['task_run_ambiguity']


@pytest.mark.parametrize('changes',[dict(execution_authorized=True),dict(schema_version=True),
    dict(event_seq=0),dict(active=1),dict(task_run_id='bad'),dict(stamp_ns=0),
    dict(session_id=[]),dict(source_stamp_ns=1.5)])
def test_task_event_malformed_or_authoritative_metadata_is_not_indexed(changes):
    with pytest.raises(ValueError):
        event('/grasp/task_event',task_event_body(**changes))


def test_task_event_requires_explicit_nullable_fields_and_accepts_actual_null_context():
    body=task_event_body()
    del body['measurement_lease_id']
    with pytest.raises(ValueError): event('/grasp/task_event',body)
    body=task_event_body(**{k:None for k in ('session_id','bound_plan_id','target_epoch','target_track_id',
        'view_index','phase_id','planning_stage','source_stamp_ns','measurement_request_id','measurement_lease_id')})
    result=build_task_chain([event('/grasp/task_event',body)],SELECTION)
    assert result['first_task_terminal']['event_ns']==1800
    assert result['first_interruption']['classification']=='terminal_without_confirmed_upstream_cause'


def test_conflicting_same_run_sequence_never_borrows_state_or_selects_a_terminal():
    body=task_event_body()
    rows=[event('/grasp/task_event',body),event('/grasp/task_event',dict(body,message='TARGET_LOST')),
          mirrored_state(body)]
    result=build_task_chain(rows,SELECTION)
    assert len(result['conflicting_task_event_sequences'])==1
    assert len(result['excluded_events'])==2
    assert result['first_task_terminal'] is None
    assert not result['task_run_id']


def test_duplicate_task_event_is_kept_but_exact_state_identity_remains_unique():
    body=task_event_body()
    result=build_task_chain([event('/grasp/task_event',body,index=0),
        event('/grasp/task_event',body,index=1),mirrored_state(body)],SELECTION)
    assert len(result['duplicate_record_groups'])==1
    assert len([e for e in result['events'] if e['correlation_basis']=='exact_task_state_mirror_join'])==1
    assert result['first_task_terminal']['observations']['terminal']['physical_success']=='unverified'


def test_two_runs_same_selection_need_explicit_run_choice_and_do_not_cross_join():
    a=task_event_body(); b=task_event_body(task_run_id='2'*32,stamp_ns=1900,message='TARGET_LOST')
    rows=[event('/grasp/task_event',a),event('/grasp/task_event',b),mirrored_state(a),mirrored_state(b)]
    result=build_task_chain(rows,SELECTION)
    assert result['task_run_ambiguity'] and result['first_task_terminal'] is None
    chosen=build_task_chain(rows,SELECTION,task_run_id=RUN)
    assert chosen['first_task_terminal']['observations']['terminal']['code']==CODE
    assert len(chosen['excluded_events'])==2
    assert build_task_chain(rows,SELECTION,task_run_id='3'*32)['first_task_terminal'] is None


def test_identical_state_under_two_runs_is_ambiguous_and_not_assigned_latest():
    a=task_event_body(); b=task_event_body(task_run_id='2'*32)
    result=build_task_chain([event('/grasp/task_event',a),event('/grasp/task_event',b),
                            mirrored_state(a)],SELECTION)
    assert result['excluded_events'][0]['excluded_reason']=='ambiguous_task_state_mirror'
    assert result['first_task_terminal'] is None


def test_wrong_selection_or_future_mirror_cannot_label_a_current_state():
    old=task_event_body(selection=dict(SELECTION,generation=5,stamp_ns=100))
    result=build_task_chain([event('/grasp/task_event',old),mirrored_state(old)],SELECTION)
    assert len(result['excluded_events'])==2 and not result['first_task_terminal']
    body=task_event_body()
    result=build_task_chain([event('/grasp/task_event',body,recorded=1700),mirrored_state(body)],SELECTION)
    assert result['excluded_events'][0]['excluded_reason']=='source_after_recorder_clock'
    assert result['first_task_terminal'] is None


def test_unmirrored_changed_state_cannot_overwrite_exact_terminal_or_supply_identity():
    body=task_event_body()
    changed=mirrored_state(dict(body,stamp_ns=1700,message='TARGET_LOST'))
    result=build_task_chain([event('/grasp/task_event',body),changed,planning()],SELECTION)
    assert next(e for e in result['events'] if e['topic']=='/grasp/state')['correlation_basis'].startswith('time_window')
    assert result['first_task_terminal']['observations']['terminal']['code']==CODE
    other=planning(observe_return=dict(session_id='other',target_track_id='g1-t3',view_index=1))
    result=build_task_chain([event('/grasp/task_event',body),other],SELECTION)
    assert result['first_interruption']['classification']=='terminal_without_confirmed_upstream_cause'


def test_same_stamp_terminal_order_uses_original_run_sequence_not_recorder_arrival():
    first=task_event_body(event_seq=7)
    later=task_event_body(event_seq=8,message='TARGET_LOST')
    result=build_task_chain([event('/grasp/task_event',later,recorded=1900),
        event('/grasp/task_event',first,recorded=2300),mirrored_state(first)],SELECTION)
    assert result['first_task_terminal']['binding']['event_seq']==7
    assert result['first_task_terminal']['observations']['terminal']['code']==CODE
