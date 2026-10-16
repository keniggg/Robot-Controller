"""Real task wrapper, frozen-plan gates and receipt lifecycle; inert motion IO."""
from copy import deepcopy
import json
import threading
from types import SimpleNamespace as NS

import pytest
from geometry_msgs.msg import PoseStamped

from alicia_grasp_modes import observe_return_task as task
from alicia_grasp_modes.observe_return_measurement import PARTICIPANTS, measurement_ack
from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id
from test_task_modes import base
import test_grasp_task_sequence as sequence_fixture


class RealTask(task.ObserveReturnTask, base.GraspTaskNode):
    pass


@pytest.fixture
def prepared_task(monkeypatch):
    now = [4_000_000_000]
    at = lambda ns: task.rospy.Time(ns // 10**9, ns % 10**9)
    params = {}
    monkeypatch.setattr(task.rospy.Time, 'now', staticmethod(lambda: at(now[0])))
    monkeypatch.setattr(task.rospy, 'get_param', lambda key, default=None: params.get(key, default))
    monkeypatch.setattr(task.rospy, 'is_shutdown', lambda: False)
    monkeypatch.setattr(task.rospy, 'loginfo', lambda *a, **k: None)
    node = RealTask.__new__(RealTask)
    node._initialize_observe_return_state()
    node.active = True
    selection = dict(mode='unknown', strategy='observe_return', generation=3, stamp_ns=10**9)
    node._mode_bound_selection = selection
    node._selection_current = lambda value: value == selection
    node._following_evidence_lock = threading.RLock()
    node._following_epoch_ns = 1_500_000_000
    plan = sequence_fixture.GraspTaskSequenceTest()._rich_plan(stamp_sec=3.5)
    plan.model_choice = 'unknown_tabletop'
    plan.diagnostic = 'FAR_FIELD_OBSERVATION_PLAN'
    plan.plan_id = compute_plan_id(plan)
    node._observe_return_session = dict(version=2, phase='observing', session_id='session', selection=selection,
        target_epoch=2, target_track_id=plan.target_track_id, origin_plan_id='origin',
        origin_snapshot_stamp_ns=2_000_000_000, view_plan_id=plan.plan_id,
        view_source_stamp_ns=plan.header.stamp.to_nsec(), view_index=2,
        deadline_ns=30_000_000_000)
    node._observe_return_origin_proof = dict(epoch_ns=node._following_epoch_ns)
    node._freeze_execution_plan(plan)
    node.latest_obj = node._bound_object_pose_from_plan(plan)
    node.latest_obj_time = at(now[0])
    node.latest_obj.header.stamp = node.latest_obj.pose_base.header.stamp = at(now[0])
    events, messages = [], []
    control = dict(planning_failure=False, planning_advance_ns=0, reject=None, before_ack=None,
                   after_planning=None, motion_success=True, strict_missing=False,
                   stage=task.GraspStages.PLAN_PREGRASP)
    node.stage = task.GraspStages.PLAN_PREGRASP
    node._last_state_message = 'admitting supplemental observation view'
    def state(stage, label='', **unused):
        node.stage, node._last_state_message = stage, label
        control['stage'] = stage
        events.append(('state', stage, label))
    node.set_state = state
    def publish(message):
        request = json.loads(message.data)
        messages.append(request)
        events.append(('request', request['state'], control['stage']))
        if request['state'] == 'revoked':
            return
        stamps = []
        if request['state'] == 'requalifying':
            stamps = [request['arrival_stamp_ns'] + i * 10_000_000 for i in (1, 2, 3)]
            now[0] = stamps[-1] + 1
            node.latest_obj.header.stamp = node.latest_obj.pose_base.header.stamp = at(stamps[-1])
            node.latest_obj_time = at(stamps[-1])
        if control['before_ack']:
            control['before_ack'](request)
        for participant in PARTICIPANTS:
            response = measurement_ack(request, participant,
                accepted=participant != control['reject'],
                qualified=request['state'] == 'requalifying', source_stamps_ns=stamps)
            node._observation_measurement_ack_cb(NS(data=json.dumps(response)))
            events.append(('ack', participant, control['stage']))
    node._observation_measurement_pub = NS(publish=publish)
    def settle(reason):
        now[0] += 30_000_000
        node._last_motion_settle_sample = dict(epoch_ns=node._following_epoch_ns,
            stationary_window_start_ns=now[0]-20_000_000,
            stationary_window_end_ns=now[0]-10_000_000)
        events.append(('settle', reason))
        return True
    node._wait_for_motion_settle = settle
    def planning(pose, execute):
        assert execute is False
        events.append(('planning', control['stage']))
        now[0] += control['planning_advance_ns']
        if control['after_planning']:
            control['after_planning']()
        return NS(success=not control['planning_failure'], message='fixture planning')
    def strict(pose, execute):
        assert execute is True
        events.append(('submit', control['stage']))
        return NS(success=control['motion_success'], message='fixture strict execution')
    def execute():
        return node._plan_and_execute_pose(task.GraspStages.MOVE_PREGRASP,
            'supplemental observation 2', PoseStamped(header=deepcopy(plan.header), pose=deepcopy(plan.poses[0])),
            planning, 'supplemental observation', execution_plan=plan,
            execution_plan_id=plan.plan_id, execute_pose=None if control['strict_missing'] else strict,
            gcfg=dict(target_observation_validity_sec=.8))
    return NS(node=node, plan=plan, now=now, params=params, events=events,
              messages=messages, control=control, execute=execute)


def test_real_wrapper_consumes_one_prepared_lease_after_both_acks_before_move(prepared_task):
    c = prepared_task
    prepared = c.node._observation_measurement_prepare(c.plan)
    assert c.execute(), c.events
    assert prepared['consumed']
    assert [m['state'] for m in c.messages] == ['executing', 'requalifying', 'qualified']
    assert len([e for e in c.events if e[0] == 'submit']) == 1
    prefix = c.events[:3]
    assert [e[0] for e in prefix] == ['request', 'ack', 'ack']
    assert all(e[-1] == task.GraspStages.PLAN_PREGRASP for e in prefix)
    assert c.messages[0]['lease_id'] == c.messages[-1]['lease_id']
    assert c.messages[0]['deadline_ns'] == c.messages[-1]['deadline_ns'] == 30_000_000_000
    assert all(m['motion_authorized'] is False and m['metric_geometry_authority'] is False for m in c.messages)


@pytest.mark.parametrize('fault', ['planning', 'target_age', 'manual', 'epoch', 'plan', 'request',
                                  'ack', 'selection', 'session', 'deadline', 'duplicate_consume'])
def test_real_planning_and_commit_failures_revoke_without_submission(prepared_task, fault):
    c = prepared_task
    prepared = c.node._observation_measurement_prepare(c.plan)
    if fault == 'planning': c.control['planning_failure'] = True
    elif fault == 'target_age': c.control['planning_advance_ns'] = 800_000_001
    else:
        def change():
            if fault == 'manual': c.params['/gui/joint_direct_mode'] = True
            elif fault == 'epoch': c.node._following_epoch_ns += 1
            elif fault == 'plan': c.plan.poses[0].position.x += .001
            elif fault == 'request': c.node._observation_measurement_request['lease_id'] = 'other'
            elif fault == 'ack':
                key = (prepared['request']['request_id'], 'remote')
                c.node._observation_measurement_acks[key]['accepted'] = False
            elif fault == 'selection': c.node._mode_bound_selection = dict(c.node._mode_bound_selection, generation=4)
            elif fault == 'session': c.node._observe_return_session['view_index'] = 3
            elif fault == 'deadline': c.now[0] = 30_000_000_000
            else: prepared['consumed'] = True
        c.control['after_planning'] = change
    assert c.execute() is False, c.events
    assert not [e for e in c.events if e[0] == 'submit']
    assert c.messages[-1]['state'] == 'revoked'
    assert len([m for m in c.messages if m['state'] == 'executing']) == 1
    if fault == 'target_age':
        assert any('TARGET_STALE' in str(e) for e in c.events), c.events


@pytest.mark.parametrize('participant', list(PARTICIPANTS))
def test_prepare_rejection_never_reaches_move_or_original_planning(prepared_task, participant):
    c = prepared_task
    c.control['reject'] = participant
    with pytest.raises(ValueError):
        c.node._observation_measurement_prepare(c.plan)
    # The surrounding supplemental transaction owns this same failure cleanup.
    c.node._observation_measurement_revoke('supplemental preparation rejected')
    assert c.messages[-1]['state'] == 'revoked'
    assert not [e for e in c.events if e[0] in ('planning', 'submit', 'state')]


def test_missing_supplement_preparation_cannot_fall_back_to_second_begin(prepared_task):
    c = prepared_task
    assert c.execute() is False
    assert not c.messages
    assert not [e for e in c.events if e[0] == 'submit']


def test_second_wrapper_call_cannot_reuse_consumed_preparation(prepared_task):
    c = prepared_task
    c.node._observation_measurement_prepare(c.plan)
    assert c.execute()
    assert c.execute() is False
    assert len([e for e in c.events if e[0] == 'submit']) == 1
    assert len([m for m in c.messages if m['state'] == 'executing']) == 1


def test_missing_strict_executor_revokes_and_clears_preparation_before_planning(prepared_task):
    c = prepared_task
    c.node._observation_measurement_prepare(c.plan)
    c.control['strict_missing'] = True
    assert c.execute() is False
    assert c.node._observation_measurement_prepared is None
    assert c.messages[-1]['state'] == 'revoked'
    assert not [e for e in c.events if e[0] in ('planning', 'submit')]
