"""Task-owned observe/return handoff with inert ROS and real audit-file binding."""
from copy import deepcopy
import hashlib
import json
import types

import pytest

from alicia_grasp_modes import observe_return_task as task
from alicia_grasp_modes.observe_return_session import parse_session
from test_runner_observe_return import observe_evidence
from test_task_modes import module, base


def at_ns(value):
    return module.rospy.Time(value // 1000000000, value % 1000000000)


def evidence(phase='observation_prefix'):
    selection, report, plan = observe_evidence(phase)
    plan.header.stamp = at_ns(report['snapshot_stamp_ns'])
    plan.fused_view_count = 2
    return selection, report, plan


def audit_reference(tmp_path, report):
    path = tmp_path / (report['plan_id'] + '.json')
    payload = json.dumps(report, allow_nan=False).encode()
    path.write_bytes(payload)
    return types.SimpleNamespace(data=json.dumps(dict(report_path=str(path),
        report_sha256=hashlib.sha256(payload).hexdigest())))


@pytest.fixture
def context(monkeypatch, tmp_path):
    selection, report, plan = evidence()
    now = [22000000000]
    params = {module.SELECTION_PARAM: deepcopy(selection)}
    monkeypatch.setattr(module.rospy, 'get_param', lambda name, default=None: params.get(name, default))
    monkeypatch.setattr(module.rospy.Time, 'now', staticmethod(lambda: at_ns(now[0])))
    monkeypatch.setattr(module.rospy, 'is_shutdown', lambda: False)
    monkeypatch.setattr(module.rospy, 'loginfo', lambda *a, **kw: None)
    monkeypatch.setattr(module.rospy, 'wait_for_service', lambda *a, **kw: None)
    node = module.ModeAwareGraspTaskNode.__new__(module.ModeAwareGraspTaskNode)
    node._initialize_mode_boundary()
    node._mode_selection = deepcopy(selection)
    node._mode_bound_selection = deepcopy(selection)
    node._mode_ready = True
    node.active = True
    node._validate_bound_plan_locked = lambda *a: types.SimpleNamespace(ok=True, code='BOUND', reason='')
    # This fixture isolates the existing session/return contract with fake
    # plans and services. Real measurement handoff transport is exercised in
    # test_observe_return_measurement, including a separate ordinary return.
    node._observation_measurement_scope = lambda plan: False
    states, publications, gateways, replans = [], [], [], []
    node.set_state = lambda *a, **kw: states.append(a)
    node._set_near_field_active = lambda *a, **kw: replans.append((a, kw))
    node._observe_return_audit_cb(audit_reference(tmp_path, report))
    proof = dict(code='OBSERVE_RETURN_ORIGIN_CAPTURED', physical_motion_submitted=False,
        session_id='f'*64, plan_id=plan.plan_id, target_track_id=plan.target_track_id,
        plan_digest='a'*64, model_sha256='b'*64, origin_goal_sha256='c'*64,
        epoch_ns=20000000000, origin_accepted_stamp_ns=21500000000,
        origin_joint_positions_rad=[0., .1, -.2, .3, -.4, .5],
        origin_tool_pose=dict(frame_id='base_link', position_m=[.1,.2,.3],
                              quaternion_xyzw=[0.,0.,0.,1.]))
    transport = dict(capture=proof, return_proof=None, rejected=None, mutate_ack=None, previous=None)

    def publish(message):
        raw = json.loads(message.data)
        publications.append(raw)
        normalized = parse_session(raw, selection, now[0], previous=transport['previous'])
        transport['previous'] = normalized
        ack = dict(normalized, accepted=True, reason='OK')
        if transport['rejected'] is not None:
            ack.update(accepted=False, reason=transport['rejected'])
        if transport['mutate_ack']:
            transport['mutate_ack'](ack)
        node._observe_return_ack_cb(types.SimpleNamespace(data=json.dumps(ack)))

    node._observe_return_pub = types.SimpleNamespace(publish=publish)

    def proxy(name, _service):
        def invoke(request_plan, execute):
            gateways.append((name, request_plan.plan_id, execute))
            if name == '/supervisor/capture_observe_return_origin':
                value = transport['capture']
            elif name == '/supervisor/return_observe_return_origin':
                now[0] = 26000000000
                value = transport['return_proof'] or dict(proof,
                    code='OBSERVE_RETURN_MEASURED_ARRIVAL', post_command_feedback_valid=True,
                    controller_action_completed=True, command_submission_attempted=True,
                    completion_stamp_ns=25000000000, accepted_stamp_ns=25500000000)
            else:
                pytest.fail('unexpected hardware service: ' + name)
            return types.SimpleNamespace(success=True, message=json.dumps(value))
        return invoke

    monkeypatch.setattr(module.rospy, 'ServiceProxy', proxy)
    return types.SimpleNamespace(node=node, selection=selection, report=report, plan=plan,
        now=now, params=params, states=states, publications=publications, gateways=gateways,
        replans=replans, proof=proof, transport=transport, tmp_path=tmp_path)


def begin(c):
    c.node._begin_observe_return(c.plan, {'unknown_observe_return_session_timeout_sec':180.})


def reach_return(c):
    begin(c)
    _, _, candidate = evidence('initial_contact')
    candidate.plan_id = 'd'*24
    candidate.header.stamp = at_ns(23000000000)
    c.now[0] = 24000000000
    result = c.node._before_near_field_contact_freeze(candidate, c.plan, {'unknown_observe_return_return_to_origin': True})
    return candidate, result


def returned_evidence(c):
    selection, report, plan = evidence('returned_contact')
    plan.plan_id = report['plan_id'] = 'b'*24
    report['selected']['final_strict_plan_id'] = plan.plan_id
    report['snapshot_stamp_ns'] = 27000000000
    plan.header.stamp = at_ns(report['snapshot_stamp_ns'])
    session = c.node._observe_return_session
    for key in ('session_id', 'origin_plan_id', 'origin_snapshot_stamp_ns',
                'return_verified_stamp_ns', 'contact_deadline_ns', 'deadline_ns'):
        value = session[key]
        report['observe_return'][key] = int(value) if key.endswith('_ns') else value
    report['observe_return']['fused_source_stamps_ns'] = [19500000000,23000000000,27000000000]
    c.now[0] = 27000000001
    return report, plan


def test_real_sha_bound_audit_reference_is_loaded_and_full_report_bypass_is_rejected(context):
    c = context
    assert c.node._observe_return_reports[c.plan.plan_id] == c.report
    assert c.node._observe_return_report(c.plan) == c.report
    c.node._observe_return_reports.clear()
    c.node._observe_return_audit_cb(types.SimpleNamespace(data=json.dumps(c.report)))
    assert not c.node._observe_return_reports


@pytest.mark.parametrize('mutation', ['wrong_hash', 'changed_file', 'missing_file', 'malformed_json'])
def test_audit_callback_rejects_unverified_file_contents(context, mutation):
    c = context
    reference = json.loads(audit_reference(c.tmp_path, c.report).data)
    if mutation == 'wrong_hash':
        reference['report_sha256'] = '0'*64
    elif mutation == 'changed_file':
        (c.tmp_path / (c.plan.plan_id + '.json')).write_text('{}')
    elif mutation == 'missing_file':
        reference['report_path'] += '.missing'
    elif mutation == 'malformed_json':
        reference = []
    c.node._observe_return_reports.clear()
    c.node._observe_return_audit_cb(types.SimpleNamespace(data=json.dumps(reference)))
    assert not c.node._observe_return_reports


def test_origin_is_measured_without_motion_before_session_and_uses_shared_timeout_key(context):
    c = context
    begin(c)
    assert c.gateways == [('/supervisor/capture_observe_return_origin', c.plan.plan_id, True)]
    assert [p['phase'] for p in c.publications] == ['observing']
    session = c.node._observe_return_session
    assert int(session['deadline_ns']) - int(session['stamp_ns']) == 180000000000
    assert c.node._observe_return_origin_proof == c.proof
    assert not c.replans


@pytest.mark.parametrize('mutation', ['missing_measurement', 'future_measurement', 'predates_plan',
    'wrong_plan', 'wrong_target', 'motion_submitted', 'bad_pose', 'nan_joint', 'missing_digest'])
def test_capture_requires_measured_origin_and_exact_plan_binding_before_observation(context, mutation):
    c = context
    proof = deepcopy(c.proof)
    if mutation == 'missing_measurement':
        del proof['origin_accepted_stamp_ns']
    elif mutation == 'future_measurement':
        proof['origin_accepted_stamp_ns'] = c.now[0] + 1
    elif mutation == 'predates_plan':
        proof['origin_accepted_stamp_ns'] = c.plan.header.stamp.to_nsec() - 1
    elif mutation == 'wrong_plan':
        proof['plan_id'] = 'e'*24
    elif mutation == 'wrong_target':
        proof['target_track_id'] = 'another'
    elif mutation == 'motion_submitted':
        proof['physical_motion_submitted'] = True
    elif mutation == 'bad_pose':
        proof['origin_tool_pose']['quaternion_xyzw'] = [0.,0.,0.,0.]
    elif mutation == 'nan_joint':
        proof['origin_joint_positions_rad'][0] = float('nan')
    elif mutation == 'missing_digest':
        del proof['origin_goal_sha256']
    c.transport['capture'] = proof
    with pytest.raises(ValueError, match='PROOF_INVALID|BINDING_INVALID'):
        begin(c)
    assert not c.publications and c.node._observe_return_session is None


@pytest.mark.parametrize('field', ['selection', 'version', 'target_epoch', 'target_track_id',
    'origin_plan_id', 'origin_snapshot_stamp_ns', 'deadline_ns', 'return_verified_stamp_ns',
    'contact_deadline_ns'])
def test_remote_ack_must_echo_full_normalized_session_not_just_three_lookup_keys(context, field):
    c = context
    def mutate(ack):
        if field == 'selection':
            ack[field]['generation'] += 1
        elif isinstance(ack[field], int):
            ack[field] += 1
        else:
            ack[field] = 'different'
    c.transport['mutate_ack'] = mutate
    with pytest.raises(ValueError, match='ACK_BINDING_INVALID'):
        begin(c)
    assert len(c.gateways) == 1 and not c.replans


def test_remote_rejection_is_not_treated_as_acknowledged_transition(context):
    c = context
    c.transport['rejected'] = 'surface target changed'
    with pytest.raises(ValueError, match='REMOTE_REJECTED: surface target changed'):
        begin(c)
    assert len(c.gateways) == 1


def test_exact_ack_keys_do_not_match_an_old_or_different_session(context, monkeypatch):
    c = context
    c.transport['mutate_ack'] = lambda ack: ack.update(session_id='old-session')
    # Wrong lookup keys remain unacknowledged through the original session
    # lifetime; five seconds now emits a warning, not a task failure.
    clock = [0.]
    monkeypatch.setattr(task.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(c.node._observe_return_condition, 'wait',
                        lambda _seconds: clock.__setitem__(0, clock[0] + 30.))
    with pytest.raises(ValueError, match='ACK_TIMEOUT'):
        begin(c)
    assert len(c.gateways) == 1


def test_epoch_zero_remains_a_legal_target_identity(context):
    c = context
    c.report['target_epoch'] = c.report['observe_return']['target_epoch'] = 0
    c.node._observe_return_audit_cb(audit_reference(c.tmp_path, c.report))
    begin(c)
    assert c.node._observe_return_session['target_epoch'] == 0


def test_pre_return_contact_triggers_return_and_is_never_frozen(context):
    c = context
    candidate, result = reach_return(c)
    assert result['retry'] is True
    assert result['minimum_stamp_ns'] == 26000000000
    assert [p['phase'] for p in c.publications] == ['observing', 'returning', 'returned']
    assert c.gateways[-1] == ('/supervisor/return_observe_return_origin', c.plan.plan_id, True)
    assert len(c.replans) == 1
    # Replaying the pre-return snapshot cannot be mistaken for permission to freeze.
    report, fresh = returned_evidence(c)
    c.node._observe_return_audit_cb(audit_reference(c.tmp_path, report))
    assert c.node._before_near_field_contact_freeze(fresh, c.plan, {'unknown_observe_return_return_to_origin': True}) is None
    candidate.plan_id = fresh.plan_id
    assert c.node._before_near_field_contact_freeze(candidate, c.plan, {'unknown_observe_return_return_to_origin': True}) == {'failed':True}


@pytest.mark.parametrize('mutation', ['no_feedback', 'no_action_completion', 'no_submission',
    'different_origin', 'different_epoch', 'different_session', 'stale_feedback'])
def test_failed_or_foreign_return_proof_never_releases_a_contact_candidate(context, mutation):
    c = context
    begin(c)
    proof = dict(c.proof, code='OBSERVE_RETURN_MEASURED_ARRIVAL', post_command_feedback_valid=True,
        controller_action_completed=True, command_submission_attempted=True,
        completion_stamp_ns=25000000000, accepted_stamp_ns=25500000000)
    if mutation == 'no_feedback':
        proof['post_command_feedback_valid'] = False
    elif mutation == 'no_action_completion':
        proof['controller_action_completed'] = False
    elif mutation == 'no_submission':
        proof['command_submission_attempted'] = False
    elif mutation == 'different_origin':
        proof['origin_goal_sha256'] = 'd'*64
    elif mutation == 'different_epoch':
        proof['epoch_ns'] += 1
    elif mutation == 'different_session':
        proof['session_id'] = 'other'
    elif mutation == 'stale_feedback':
        proof['accepted_stamp_ns'] = proof['completion_stamp_ns']
    c.transport['return_proof'] = proof
    _, _, candidate = evidence('initial_contact')
    c.now[0] = 24000000000
    result = c.node._before_near_field_contact_freeze(candidate, c.plan, {'unknown_observe_return_return_to_origin': True})
    assert result == {'failed':True}
    assert c.states[-1][0] == base.GraspStages.FAILED
    assert not c.replans
    assert [p['phase'] for p in c.publications] == ['observing', 'returning']


@pytest.mark.parametrize('mutation', ['different_epoch', 'different_session', 'different_origin',
    'different_origin_stamp', 'different_deadline', 'different_contact_deadline',
    'expired_contact', 'stale_snapshot', 'foreign_generation', 'foreign_target',
    'noncontact_phase', 'physical_gate_failed'])
def test_only_fresh_same_session_epoch_contact_can_freeze_after_return(context, mutation):
    c = context
    _, result = reach_return(c)
    assert result['retry']
    report, plan = returned_evidence(c)
    meta = report['observe_return']
    if mutation == 'different_epoch':
        # Internally self-consistent foreign epoch must still fail the task session.
        meta['target_epoch'] += 1
        report['target_epoch'] = meta['target_epoch']
    elif mutation == 'different_session':
        meta['session_id'] = 'other-session'
    elif mutation == 'different_origin':
        meta['origin_plan_id'] = 'e'*24
    elif mutation == 'different_origin_stamp':
        meta['origin_snapshot_stamp_ns'] += 1
    elif mutation == 'different_deadline':
        meta['deadline_ns'] += 1
    elif mutation == 'different_contact_deadline':
        meta['contact_deadline_ns'] -= 1
    elif mutation == 'expired_contact':
        c.now[0] = int(c.node._observe_return_session['contact_deadline_ns'])
    elif mutation == 'stale_snapshot':
        report['snapshot_stamp_ns'] = meta['return_verified_stamp_ns']
        plan.header.stamp = at_ns(report['snapshot_stamp_ns'])
    elif mutation == 'foreign_generation':
        report['grasp_mode_selection']['generation'] += 1
    elif mutation == 'foreign_target':
        meta['target_track_id'] = plan.target_track_id = 'another-object'
    elif mutation == 'noncontact_phase':
        meta['phase'] = 'observation_contact'
    elif mutation == 'physical_gate_failed':
        report['selected']['moveit']['collision_free'] = False
    c.node._observe_return_audit_cb(audit_reference(c.tmp_path, report))
    assert c.node._before_near_field_contact_freeze(plan, c.plan, {'unknown_observe_return_return_to_origin': True}) == {'failed':True}
    assert c.states[-1][0] == base.GraspStages.FAILED
    assert len(c.gateways) == 2  # No retry motion or close after failed handoff.


def test_new_inactive_attempt_retires_expired_same_generation_session_and_origin(context, monkeypatch):
    c = context
    c.node.active = False
    c.node._observe_return_session = dict(deadline_ns='1', session_id='old')
    c.node._observe_return_entry_phase = 'FAR_FIELD_OBSERVATION_PLAN'
    c.node._observe_return_origin_proof = {'old':True}
    def start(self, _req):
        assert self._observe_return_session is None
        assert self._observe_return_entry_phase is None
        assert self._observe_return_origin_proof is None
        assert self._observe_return_reports[c.plan.plan_id] == c.report
        return base.StartGraspResponse(True, 'fresh attempt')
    monkeypatch.setattr(base.GraspTaskNode, 'start_cb', start)
    assert c.node.start_cb(types.SimpleNamespace(execute=True, plan_id=c.plan.plan_id)).success


def test_active_attempt_cannot_be_cleared_by_another_start(context, monkeypatch):
    c = context
    begin(c)
    session, proof = deepcopy(c.node._observe_return_session), deepcopy(c.node._observe_return_origin_proof)
    monkeypatch.setattr(base.GraspTaskNode, 'start_cb', lambda *_: pytest.fail('active task reentered'))
    assert not c.node.start_cb(types.SimpleNamespace(execute=True)).success
    assert c.node._observe_return_session == session
    assert c.node._observe_return_origin_proof == proof


@pytest.mark.parametrize('phase', ['initial_contact', 'returned_contact'])
def test_direct_contact_entry_requires_initial_audit_even_without_runner(context, monkeypatch, phase):
    c = context
    selection, report, plan = evidence(phase)
    c.node._observe_return_audit_cb(audit_reference(c.tmp_path, report))
    executed = []
    monkeypatch.setattr(base.GraspTaskNode, '_execute_grasp6d_plan',
                        lambda *_args, **_kwargs: executed.append('original contact flow') or True)
    result = c.node._execute_grasp6d_plan({}, {}, .05, None, None, None,
                                       lambda *_: pytest.fail('closed on invalid entry'), plan)
    assert result is (phase == 'initial_contact')
    assert bool(executed) == (phase == 'initial_contact')


def test_capture_failure_stops_task_before_original_grasp_or_close(context, monkeypatch):
    c = context
    c.transport['capture']['physical_motion_submitted'] = True
    monkeypatch.setattr(base.GraspTaskNode, '_execute_grasp6d_plan',
                        lambda *_a, **_kw: pytest.fail('failed prefix entered grasp executor'))
    assert not c.node._execute_grasp6d_plan({}, {}, .05, None, None, None,
        lambda *_: pytest.fail('closed despite failed prefix'), c.plan)
    assert c.states[-1][0] == base.GraspStages.FAILED
