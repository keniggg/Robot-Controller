"""The real task prefix stays behind dual ACKs and fresh measured requalification."""
from copy import deepcopy
import json
import threading
from types import SimpleNamespace as NS

import pytest
from sensor_msgs.msg import JointState

from alicia_grasp_modes import observe_return_task as task
from alicia_grasp_modes.observe_return_measurement import PARTICIPANTS, measurement_ack
from alicia_grasp_modes.initial_preflight_attempt import InitialPreflightAttemptMixin
from alicia_grasp_modes.initial_preflight_task import InitialPreflightTaskMixin
from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id
from test_task_modes import base
import test_grasp_task_sequence as sequence_fixture


class RealTask(InitialPreflightAttemptMixin, task.ObserveReturnTask, base.GraspTaskNode):
    pass


@pytest.fixture
def bootstrap(monkeypatch):
    now = [4_000_000_000]
    mono = [1.]
    at = lambda ns: task.rospy.Time(ns // 10**9, ns % 10**9)
    params = {}
    monkeypatch.setattr(task.rospy.Time, 'now', staticmethod(lambda: at(now[0])))
    monkeypatch.setattr(task.rospy, 'get_param', lambda key, default=None: params.get(key, default))
    monkeypatch.setattr(task.rospy, 'is_shutdown', lambda: False)
    monkeypatch.setattr(task.rospy, 'loginfo', lambda *a, **k: None)
    monkeypatch.setattr(task.rospy, 'wait_for_service', lambda *a, **k: None)
    monkeypatch.setattr(task.time, 'monotonic', lambda: mono[0])
    node = RealTask.__new__(RealTask)
    node._initialize_observe_return_state()
    node._initialize_initial_preflight()
    node._initial_preflight_token = None
    node._initial_preflight_collector = None
    node.active = True
    selection = dict(mode='unknown', strategy='observe_return', generation=3, stamp_ns=10**9)
    node._mode_bound_selection = selection
    node._selection_current = lambda value: value == selection
    node._following_evidence_lock = threading.RLock()
    node._following_epoch_ns = 1_500_000_000
    feedback = JointState()
    feedback.header.stamp = at(now[0])
    feedback.header.frame_id = 'sdk_measured'
    feedback.name = ['Joint%d' % i for i in range(1, 7)]
    feedback.position = [0.] * 6
    node._following_accepted_history = [feedback]
    plan = sequence_fixture.GraspTaskSequenceTest()._rich_plan(stamp_sec=3.5)
    plan.model_choice = 'unknown_tabletop'
    plan.diagnostic = 'FAR_FIELD_OBSERVATION_PLAN'
    plan.plan_id = compute_plan_id(plan)
    node._initial_preflight_begin(NS(plan_id=plan.plan_id))
    node._freeze_execution_plan(plan)
    node.latest_obj = node._bound_object_pose_from_plan(plan)
    node.latest_obj_time = at(now[0])
    node.latest_obj.header.stamp = node.latest_obj.pose_base.header.stamp = at(now[0])
    report = dict(observe_return=dict(phase='observation_prefix', target_epoch=2,
        target_track_id=plan.target_track_id, origin_snapshot_stamp_ns=plan.header.stamp.to_nsec()))
    node._observe_return_report = lambda unused: deepcopy(report)
    node.latest_actuation_status = 'PENDING:POSITIVE_ENABLE_REQUESTED'
    node.latest_actuation_status_time = at(now[0])
    gcfg = dict(auto_confirm_actuation_from_observation=True, use_grasp6d_plan=True,
        require_actuation_confirmation=True, target_observation_validity_sec=.8,
        grasp6d_plan_validity_sec=120.)
    node._configured_plan_validity = lambda unused: 120.
    events, messages = [], []
    control = dict(before_ack=None, after_prefix=None, after_settle=None,
        reject=None, prefix_success=True, settle_success=True, confirmation=True,
        omit_object=False, bad_object=False)
    def state(stage, label='', *args):
        node.stage, node._last_state_message = stage, label
        events.append(('state', stage))
    node.set_state = state
    def publish(message):
        request = json.loads(message.data)
        messages.append(request)
        events.append(('request', request['state']))
        if request['state'] == 'revoked':
            return
        stamps = []
        if request['state'] == 'requalifying':
            stamps = [request['arrival_stamp_ns'] + i * 10_000_000 for i in (1, 2, 3)]
            now[0] = stamps[-1] + 1
            if not control['omit_object']:
                node.latest_obj.header.stamp = node.latest_obj.pose_base.header.stamp = at(stamps[-1])
                node.latest_obj_time = at(stamps[-1])
                if control['bad_object']:
                    node.latest_obj.pose_base.pose.position.x += 1.
        if control['before_ack']:
            control['before_ack'](request)
        for participant in PARTICIPANTS:
            ack = measurement_ack(request, participant, accepted=participant != control['reject'],
                qualified=request['state'] == 'requalifying', source_stamps_ns=stamps)
            node._observation_measurement_ack_cb(NS(data=json.dumps(ack)))
            events.append(('ack', participant, request['state']))
        if request['state'] == 'requalifying' and control['omit_object']:
            now[0] = request['capture_deadline_ns']
    node._observation_measurement_pub = NS(publish=publish)
    def prefix(pose, execute):
        assert execute is True
        assert node._observe_return_session is None
        assert node._observe_return_origin_proof is None
        assert all(('ack', p, 'executing') in events for p in PARTICIPANTS)
        assert node._initial_preflight_attempt['physical_action_submitted']
        events.append(('prefix',))
        now[0] += 3_000_000_000
        mono[0] += 3.
        if control['confirmation']:
            node.latest_actuation_status = 'CONFIRMED:encoder_response'
            node.latest_actuation_status_time = at(now[0])
        if control['after_prefix']:
            control['after_prefix']()
        return NS(success=control['prefix_success'], message='bounded original prefix result')
    def proxy(name, kind):
        assert name == '/supervisor/confirm_actuation_for_observation'
        return prefix
    monkeypatch.setattr(task.rospy, 'ServiceProxy', proxy)
    def settle(reason):
        events.append(('settle',))
        now[0] += 800_000_000
        mono[0] += .8
        node._last_motion_settle_sample = dict(epoch_ns=node._following_epoch_ns,
            stationary_window_start_ns=now[0]-700_000_000,
            stationary_window_end_ns=now[0]-1)
        if control['after_settle']:
            control['after_settle']()
        return control['settle_success']
    node._wait_for_motion_settle = settle
    node._ensure_qualified_station_phase = lambda *a: pytest.fail('bootstrap cannot start a contact phase')
    return NS(node=node, plan=plan, gcfg=gcfg, report=report, now=now, mono=mono,
        params=params, events=events, messages=messages, control=control,
        run=lambda: node._bootstrap_bound_actuation(plan, gcfg))


def test_real_prefix_runs_once_after_dual_ack_then_requires_fresh_same_target(bootstrap):
    c = bootstrap
    assert c.run(), c.node._last_state_message
    assert sum(e[0] == 'prefix' for e in c.events) == 1
    assert [m['state'] for m in c.messages] == ['executing', 'requalifying', 'qualified']
    start, arrival, done = c.messages
    assert start['deadline_ns'] == 123_500_000_000
    assert all(m['deadline_ns'] == start['deadline_ns'] for m in c.messages)
    assert arrival['settle']['stationary_window_start_ns'] > start['stamp_ns']
    assert done['source_stamps_ns'][0] > arrival['arrival_stamp_ns']
    assert c.node._observe_return_session is None and c.node._observe_return_origin_proof is None
    assert c.node._qualified_station_receipt is None and c.node._qualified_station_phase is None
    assert c.node._actuation_bootstrap_receipt['contact_authorized'] is False
    assert c.node._actuation_bootstrap_receipt['counts_as_observation_view'] is False
    assert c.node._initial_preflight_attempt['physical_action_submitted'] is True
    assert not InitialPreflightTaskMixin._initial_preflight_arm(c.node, c.plan, c.gcfg)


@pytest.mark.parametrize('participant', PARTICIPANTS)
def test_either_rejected_ack_blocks_reference_sync_and_prefix(bootstrap, participant):
    c = bootstrap
    c.control['reject'] = participant
    assert not c.run()
    assert not any(e[0] == 'prefix' for e in c.events)
    assert c.messages[-1]['state'] == 'revoked'


@pytest.mark.parametrize('fault', ['revoked', 'epoch', 'manual', 'selection', 'plan', 'ros_expired', 'mono_expired'])
def test_changes_during_dual_ack_never_reach_original_service(bootstrap, fault):
    c = bootstrap
    def mutate(unused):
        if fault == 'revoked': c.node._execution_authority_revoked = True
        elif fault == 'epoch': c.node._following_epoch_ns += 1
        elif fault == 'manual': c.params['/gui/joint_direct_mode'] = True
        elif fault == 'selection': c.node._mode_bound_selection = dict(c.node._mode_bound_selection, generation=4)
        elif fault == 'plan': c.node._bound_execution_plan.target_track_id += '_changed'
        elif fault == 'ros_expired': c.now[0] = 123_500_000_000
        else: c.mono[0] = 200.
    c.control['before_ack'] = mutate
    assert not c.run()
    assert not any(e[0] == 'prefix' for e in c.events)


@pytest.mark.parametrize('stage', ['prefix', 'settle', 'qualification'])
def test_real_revocation_after_prefix_blocks_every_later_stage(bootstrap, stage):
    c = bootstrap
    revoke = lambda: setattr(c.node, '_execution_authority_revoked', True)
    if stage == 'prefix': c.control['after_prefix'] = revoke
    elif stage == 'settle': c.control['after_settle'] = revoke
    else: c.control['before_ack'] = lambda request: revoke() if request['state'] == 'qualified' else None
    assert not c.run()
    assert sum(e[0] == 'prefix' for e in c.events) == 1
    assert c.node._actuation_bootstrap_receipt is None
    assert c.node._observe_return_session is None
    if stage == 'prefix': assert not any(e[0] == 'settle' for e in c.events)


@pytest.mark.parametrize('fault', ['prefix', 'confirmation', 'settle', 'missing_object', 'source_expired'])
def test_no_success_from_service_ack_without_complete_measured_handoff(bootstrap, fault):
    c = bootstrap
    if fault == 'prefix': c.control['prefix_success'] = False
    elif fault == 'confirmation': c.control['confirmation'] = False
    elif fault == 'settle': c.control['settle_success'] = False
    elif fault == 'missing_object': c.control['omit_object'] = True
    else: c.control['after_prefix'] = lambda: c.now.__setitem__(0, 123_500_000_000)
    assert not c.run()
    assert c.node._actuation_bootstrap_receipt is None
    assert c.node._qualified_station_receipt is None


@pytest.mark.parametrize('field,value', [('phase', 'supplemental_observation'),
    ('diagnostic_observation_only', True), ('origin_snapshot_stamp_ns', 3_000_000_000),
    ('target_track_id', 'other'), ('target_epoch', True)])
def test_provisional_receipt_cannot_claim_another_origin_or_observation(bootstrap, field, value):
    c = bootstrap
    c.report['observe_return'][field] = value
    assert not c.run()
    assert c.messages == [] and not any(e[0] == 'prefix' for e in c.events)


def test_bootstrap_receipt_cannot_be_reused_for_a_second_prefix(bootstrap):
    c = bootstrap
    assert c.run()
    c.node.latest_actuation_status = 'PENDING:COMMAND_SYNCHRONIZED'
    assert not c.run()
    assert sum(e[0] == 'prefix' for e in c.events) == 1


def test_reference_sync_joint3_change_retires_initial_station_optimization_only(bootstrap):
    c = bootstrap
    assert c.node._initial_preflight_attempt['disqualifications'] == []
    def feedback_after_prefix():
        message = deepcopy(c.node._following_accepted_history[-1])
        message.header.stamp = task.rospy.Time.from_sec(c.now[0] / 1e9)
        message.position[2] = .00767
        c.node._initial_preflight_feedback(message)
    c.control['after_prefix'] = feedback_after_prefix
    assert c.run(), c.node._last_state_message
    assert c.node._initial_preflight_attempt['disqualifications'] == ['measured_pose_changed']
    assert not c.node._execution_authority_revoked
    assert not InitialPreflightTaskMixin._initial_preflight_scope(c.node)
