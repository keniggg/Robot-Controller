"""Service/callback lifecycle regression with recorded loss-message ordering.

The planner and encoder plant are explicit doubles from the existing gateway
fixture. The service, context ownership, committed-plan callbacks, independent
stationary-window check and compensation admission are production functions.
This does not claim physical motion, collision or grasp success.
"""
from collections import deque
from copy import deepcopy
import json
from pathlib import Path
import threading
import time
from types import MethodType, SimpleNamespace as NS
from unittest import mock

import pytest
import rospy
from std_msgs.msg import String

from test_contact_pregrasp_gateway import case, captured_session, at_current_pose, module
from test_motion_gateway_controller_start import MotionGateway
from test_motion_settle import measured, task
from alicia_flexible_grasp.grasp.grasp_state_machine import GraspStages
from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id
from alicia_flexible_grasp.robot.observation_path_guard import (
    FrozenObservationScene, ObservationPathError,
)


RECORD = json.loads((Path(__file__).parent / 'fixtures' /
                    'contact_pregrasp_handoff_20261024.json').read_text())


def captured_long_session(c, budget_sec):
    """Synthetic origin, with its one initial session admitted by the real callback."""
    c.params['/grasp']['unknown_observe_return_session_version'] = 2
    observation = deepcopy(c.plan)
    observation.diagnostic = 'FAR_FIELD_OBSERVATION_PLAN'
    observation.header.stamp = rospy.Time.from_sec(1.8)
    observation.object_geometry.header = deepcopy(observation.header)
    observation.plan_id = compute_plan_id(observation)
    scene = FrozenObservationScene.from_plan(observation)
    origin = dict(scene=scene, plan=observation,
        selection=deepcopy(c.params['/grasp_mode/selection']),
        epoch=c.node._control_reference_epoch_ns, revocation=c.node._observation_contexts.revocation,
        identity='d'*64, deadline=time.monotonic()+budget_sec,
        config=deepcopy(c.params['/grasp']), model=c.params['/robot_description'],
        session_id='captured-session')
    c.node._observe_return_origin = origin
    raw = dict(version=2, phase='observing', selection=deepcopy(origin['selection']),
        session_id=origin['session_id'], origin_plan_id=scene.plan_id,
        origin_snapshot_stamp_ns=scene.source_ns, target_track_id=scene.track_id, target_epoch=0,
        stamp_ns=int(c.state['now']*1e9), deadline_ns=int((c.state['now']+budget_sec)*1e9),
        max_observations=3, view_index=1, view_plan_id=scene.plan_id,
        view_source_stamp_ns=scene.source_ns, view_request_id='initial:'+scene.plan_id,
        previous_gain_sha256='0'*64, return_verified_stamp_ns=0, contact_deadline_ns=0)
    c.node._observe_return_view_session_cb(String(data=json.dumps(raw)))
    assert origin['view_session']['deadline_ns'] == raw['deadline_ns']
    c.node._committed_observation_plan_cb(deepcopy(c.plan))
    return origin


def lifecycle_case(monkeypatch, strategy='observe_return', already=False, session_sec=None):
    c = case(monkeypatch)
    c.state['connected'] = True
    monkeypatch.setattr(time, 'monotonic', lambda: 100. + c.state['now'])
    monkeypatch.setattr(rospy.Time, 'now', staticmethod(
        lambda: rospy.Time.from_sec(c.state['now'])))
    c.node._observation_task_sub = NS(get_num_connections=lambda: int(c.state['connected']))
    c.node._observation_task_state_stamp_ns = 0
    c.node._observation_task_requires_inactive = False
    c.node._require_live_observation_task = MethodType(
        MotionGateway._require_live_observation_task, c.node)
    c.params['/grasp_mode/selection']['strategy'] = strategy
    if strategy == 'observe_return':
        c.origin = (captured_session(c, monkeypatch) if session_sec is None
                    else captured_long_session(c, session_sec))
    else:
        c.origin = None
        c.node._committed_observation_plan_cb(deepcopy(c.plan))
    if already:
        at_current_pose(c)
        def proof(*args):
            args[-1].update(success=True, predicted_pregrasp={
                'predicted_command_positions_rad': list(args[2]['sdk_positions_rad'])})
        module.predict_held_approach.side_effect = proof
    contexts = []
    original = c.node._bind_contact_pregrasp_session
    def bind(context):
        original(context)
        contexts.append(context)
    c.node._bind_contact_pregrasp_session = bind
    c.contexts = contexts
    monkeypatch.setattr(rospy, 'logwarn', mock.Mock())
    monkeypatch.setattr(rospy, 'logwarn_throttle', mock.Mock())
    return c


def loss(c, *, row=None):
    msg = deepcopy(c.plan)
    msg.valid = False
    msg.diagnostic = 'TARGET_LOST: target object is not detected'
    msg.plan_id = ''
    msg.target_track_id = ''
    msg.header.stamp = rospy.Time.from_sec(c.state['now'])
    if row is not None:
        msg.valid = row['valid']
        msg.diagnostic = row['diagnostic']
        msg.plan_id = row['plan_id']
        msg.target_track_id = row['track']
        msg.header.frame_id = row['header']['frame_id']
        delta = row['header']['stamp']['nanoseconds'] - RECORD['controller_completion_ns']
        msg.header.stamp = rospy.Time.from_sec(c.completed_ros + delta * 1e-9)
    return msg


def execute_pregrasp(c):
    result = c.node.handle_contact_pregrasp(NS(plan=c.plan, execute=True))
    assert result.success, result.message
    c.completed_ros = c.state['now']
    return json.loads(result.message)


def settle_with_recorded_callbacks(c, monkeypatch):
    """Real task settle loop; unique synthetic encoder sources and bag ordering."""
    owner = task.GraspTaskNode.__new__(task.GraspTaskNode)
    owner.active = True
    owner._following_evidence_lock = threading.RLock()
    owner._following_epoch_ns = c.node._control_reference_epoch_ns
    owner._following_accepted_history = deque(maxlen=128)
    c.node._endpoint_feedback_lock = threading.RLock()
    c.node._endpoint_feedback_history = {'sdk': deque(maxlen=128), 'accepted': deque(maxlen=128)}
    c.node._sdk_command_feedback = None
    c.params.update({'/grasp/motion_settle_min_sec': .8,
                     '/grasp/motion_settle_sample_sec': .1})
    pending = list(RECORD['loss_messages'])
    delivered = []
    def sleep(seconds):
        c.state['now'] += seconds
        elapsed_ns = round((c.state['now'] - c.completed_ros) * 1e9)
        while pending and pending[0]['bag_timestamp_ns'] - RECORD['controller_completion_ns'] <= elapsed_ns:
            row = pending.pop(0)
            c.node._committed_observation_plan_cb(loss(c, row=row))
            delivered.append(row['wire_sha256'])
        # Encoder source precedes reception by 1 ms. Rounding reception to
        # nanoseconds can otherwise make a synthetic source 1 ns future.
        sample = measured(rospy.Time.now().to_nsec() - 1_000_000)
        sample.position = list(c.state['actual']) + [.04975]
        owner._following_accepted_history.append(sample)
        c.node._observation_joint_feedback_cb(sample)
        sdk = deepcopy(sample)
        sdk.header.frame_id = 'sdk_transmitted'
        sdk.name = sdk.name[:6]
        sdk.position = list(c.state['sdk'])
        c.node._sdk_command_feedback_cb(sdk)
    monkeypatch.setattr(rospy, 'sleep', sleep)
    assert owner._wait_for_motion_settle('near-field pregrasp handoff'), rospy.logwarn.call_args_list
    assert len(delivered) == 5 and not pending
    assert owner._last_motion_settle_sample['accepted_sample_count'] >= 3
    assert max(owner._last_motion_settle_sample['joint_peak_to_peak_rad']) == 0.
    return owner._last_motion_settle_sample


@pytest.mark.parametrize('strategy', ['direct', 'observe_return'])
def test_successful_service_gap_keeps_same_owner_through_real_settle_and_fresh_compensation(
        monkeypatch, strategy):
    c = lifecycle_case(monkeypatch, strategy)
    report = execute_pregrasp(c)
    assert report['code'] == 'CONTACT_PREGRASP_EXECUTED'
    old = c.contexts[-1]
    original_deadline = old['deadline']
    receipt = deepcopy(c.node._contact_pregrasp_receipts)
    settle = settle_with_recorded_callbacks(c, monkeypatch)
    assert not c.node._observation_contexts.blocked
    assert c.node._observation_contexts.revocation == old['revocation']
    assert c.node._contact_pregrasp_receipts == receipt
    snapshots = []
    original = MethodType(MotionGateway._endpoint_following_snapshot, c.node)
    def fresh(*args, **kwargs):
        sample = original(*args, **kwargs)
        snapshots.append((args[2], sample))
        # This regression tests the real admission boundary, not simulated
        # compensation convergence or physical geometry using a fake plant.
        raise ObservationPathError('HANDOFF_TEST_FRESH_FEEDBACK_BOUNDARY')
    c.node._endpoint_following_snapshot = fresh
    result = c.node.handle_compensate_pregrasp(NS(plan=c.plan, execute=True))
    assert not result.success and 'HANDOFF_TEST_FRESH_FEEDBACK_BOUNDARY' in result.message
    assert len(snapshots) == 1
    new, sample = snapshots[0]
    assert new is not old and new['scene'].digest == old['scene'].digest
    assert sample['accepted_stamp_ns'] >= settle['stationary_window_end_ns']
    assert new['deadline'] <= original_deadline
    assert c.state['moves'] == 1 and c.node._compensation_visual_context is None
    assert c.planner._last_pose_plan is None


def test_already_satisfied_true_call_keeps_handoff_and_compensation_uses_fresh_measurement(monkeypatch):
    c = lifecycle_case(monkeypatch, already=True)
    report = execute_pregrasp(c)
    assert report['code'] == 'CONTACT_PREGRASP_ALREADY_SATISFIED'
    assert report['no_motion_submitted'] and not report['execution_submitted']
    settle_with_recorded_callbacks(c, monkeypatch)
    c.node._endpoint_following_snapshot = MethodType(MotionGateway._endpoint_following_snapshot, c.node)
    result = c.node.handle_compensate_pregrasp(NS(plan=c.plan, execute=True))
    assert result.success, result.message
    assert json.loads(result.message)['code'] == 'PREGRASP_MEASURED_CONVERGED'
    assert c.state['moves'] == 0 and not c.calls
    assert c.node._compensation_visual_context is None


@pytest.mark.parametrize('start_offset,action_extra_sec', [(0., 90.), (590., 0.)])
def test_handoff_outlives_old_phase_but_never_renews_original_session(
        monkeypatch, start_offset, action_extra_sec):
    c = lifecycle_case(monkeypatch, session_sec=600.)
    session = deepcopy(c.origin['view_session'])
    deadline = c.origin['deadline']
    owner = task.GraspTaskNode.__new__(task.GraspTaskNode)
    owner._set_near_field_active(True, budget_sec=90.)  # no publisher on this offline owner
    phase = deepcopy(owner._published_near_field_phase)
    c.state['now'] += start_offset
    original_execute = c.planner.plan_and_execute_joint_probe.side_effect
    def execute(*args, **kwargs):
        result = original_execute(*args, **kwargs)
        c.state['now'] += action_extra_sec  # action/scheduling delay; no new computation budget
        return result
    c.planner.plan_and_execute_joint_probe.side_effect = execute
    execute_pregrasp(c)
    old = c.contexts[-1]
    if action_extra_sec:
        assert time.monotonic() > old['contact_deadlines']['compute_deadline']
    assert c.state['now'] > phase.deadline.to_sec()
    settle = settle_with_recorded_callbacks(c, monkeypatch)
    original_snapshot = MethodType(MotionGateway._endpoint_following_snapshot, c.node)
    admitted = []
    def snapshot(*args, **kwargs):
        report = original_snapshot(*args, **kwargs)
        admitted.append(args[2])
        assert report['accepted_stamp_ns'] >= settle['stationary_window_end_ns']
        assert c.origin['view_session'] == session and c.origin['deadline'] == deadline
        assert args[2]['deadline'] == min(time.monotonic()+60., old['deadline'])
        raise ObservationPathError('HANDOFF_TEST_FRESH_FEEDBACK_BOUNDARY')
    c.node._endpoint_following_snapshot = snapshot
    result = c.node.handle_compensate_pregrasp(NS(plan=c.plan, execute=True))
    assert not result.success and 'HANDOFF_TEST_FRESH_FEEDBACK_BOUNDARY' in result.message
    assert len(admitted) == 1
    assert admitted[0]['deadline'] <= deadline
    assert c.state['now']*1e9 < session['deadline_ns']
    assert c.node._compensation_visual_context is None


@pytest.mark.parametrize('fault', ['ros_deadline', 'monotonic_deadline', 'ros_rollback',
                                  'monotonic_rollback', 'session_replaced', 'session_extended'])
def test_new_compensation_context_continues_original_session_checks(monkeypatch, fault):
    c = lifecycle_case(monkeypatch)
    execute_pregrasp(c)
    old = c.contexts[-1]
    new = c.node._pregrasp_context(c.plan)
    assert new is not old and new['deadline'] <= old['deadline']
    mono = time.monotonic()
    if fault == 'ros_deadline':
        c.state['now'] = c.origin['view_session']['deadline_ns'] * 1e-9
        monkeypatch.setattr(time, 'monotonic', lambda: mono)
    if fault == 'monotonic_deadline':
        monkeypatch.setattr(time, 'monotonic', lambda: old['deadline'])
    if fault == 'ros_rollback':
        c.state['now'] -= .1
        monkeypatch.setattr(time, 'monotonic', lambda: mono)
    if fault == 'monotonic_rollback':
        monkeypatch.setattr(time, 'monotonic', lambda: mono-.1)
    if fault == 'session_replaced': c.node._observe_return_origin = deepcopy(c.origin)
    if fault == 'session_extended': c.origin['deadline'] += 60.
    with pytest.raises(ObservationPathError):
        c.node._validate_pregrasp_context(new)
    assert not c.node._compensation_allows_visual_loss(loss(c))


@pytest.mark.parametrize('change', ['transport_seq', 'new_same_track_measurement'])
def test_new_same_track_measurement_does_not_replace_frozen_owner(monkeypatch, change):
    c = lifecycle_case(monkeypatch)
    execute_pregrasp(c)
    old = c.contexts[-1]
    msg = deepcopy(c.plan)
    if change == 'transport_seq':
        msg.header.seq += 1
    else:
        # This is a new candidate, not evidence that the physical target moved.
        # Do not invent a 1 mm rejection threshold for ordinary measurement noise.
        msg.object_geometry.pose_base.position.x += .001
        msg.header.stamp = rospy.Time.from_sec(c.state['now'])
        msg.object_geometry.header = deepcopy(msg.header)
        msg.plan_id = compute_plan_id(msg)
    c.node._committed_observation_plan_cb(msg)
    c.node._committed_observation_plan_cb(loss(c))
    new = c.node._pregrasp_context(c.plan)
    assert new['scene'].digest == old['scene'].digest and new['deadline'] <= old['deadline']


@pytest.mark.parametrize('change', ['plan', 'pose_index'])
def test_compensation_cannot_replace_pending_handoff_with_a_new_request(monkeypatch, change):
    c = lifecycle_case(monkeypatch)
    execute_pregrasp(c)
    plan = deepcopy(c.plan)
    if change == 'plan':
        plan.object_geometry.pose_base.position.x += .001
        plan.plan_id = compute_plan_id(plan)
    with pytest.raises(ObservationPathError, match='HANDOFF_PLAN_CHANGED'):
        c.node._pregrasp_context(plan, 1 if change == 'pose_index' else 0)
    assert c.state['moves'] == 1


def test_compensation_handoff_does_not_replace_fresh_feedback_with_frozen_context(monkeypatch):
    c = lifecycle_case(monkeypatch, already=True)
    execute_pregrasp(c)
    settle_with_recorded_callbacks(c, monkeypatch)
    c.node._endpoint_following_snapshot = MethodType(MotionGateway._endpoint_following_snapshot, c.node)
    c.state['now'] += .6
    monkeypatch.setattr(rospy, 'sleep', lambda seconds: c.state.update(now=c.state['now']+seconds))
    result = c.node.handle_compensate_pregrasp(NS(plan=c.plan, execute=True))
    assert not result.success and 'ENDPOINT_CORRECTION_FEEDBACK_UNAVAILABLE' in result.message
    assert c.state['moves'] == 0 and c.node._compensation_visual_context is None


@pytest.mark.parametrize('success', [True, False])
def test_old_service_finally_never_clears_a_replacement_context(monkeypatch, success):
    c = lifecycle_case(monkeypatch)
    replacement = {'replacement_identity_only_not_motion_authority': True}
    factory = module.CompensatePregraspResponse
    def replace(*args):
        result = factory(*args)
        c.node._compensation_visual_context = replacement
        return result
    monkeypatch.setattr(module, 'CompensatePregraspResponse', replace)
    if not success:
        c.planner.plan_and_execute_joint_probe.side_effect = lambda *_a, **_k: (False, 'rejected')
    result = c.node.handle_contact_pregrasp(NS(plan=c.plan, execute=True))
    assert result.success is success
    assert c.node._compensation_visual_context is replacement


@pytest.mark.parametrize('mode', ['dry_run', 'already_dry_run', 'failed_execution'])
def test_no_admitted_success_does_not_preserve_visual_loss(monkeypatch, mode):
    c = lifecycle_case(monkeypatch, already=mode == 'already_dry_run')
    if mode == 'failed_execution':
        c.planner.plan_and_execute_joint_probe.side_effect = lambda *_a, **_k: (False, 'rejected')
    result = c.node.handle_contact_pregrasp(NS(plan=c.plan, execute=mode == 'failed_execution'))
    assert result.success is (mode != 'failed_execution')
    assert c.node._compensation_visual_context is None
    c.node._committed_observation_plan_cb(loss(c))
    assert c.node._observation_contexts.blocked
    with pytest.raises(ObservationPathError):
        c.node._pregrasp_context(c.plan)


@pytest.mark.parametrize('fault', [
    'manual', 'manual_callback', 'epoch', 'stage', 'selection', 'model', 'config',
    'gripper_geometry', 'gripper_config', 'actuation', 'disconnect',
    'terminal', 'inactive', 'nonvisual', 'different_track', 'valid_different_track',
    'frozen_plan_tampered', 'future', 'old', 'frame', 'monotonic_deadline',
    'session_ros_deadline', 'session_replaced', 'session_extended',
])
def test_retained_handoff_never_overrides_real_revocation(monkeypatch, fault):
    c = lifecycle_case(monkeypatch)
    execute_pregrasp(c)
    msg = loss(c)
    if fault == 'manual': c.state['manual'] = True
    if fault == 'manual_callback':
        c.node._rgb_acquisition_revoke_event = mock.Mock()
        c.node._gui_mode_cb(NS(data=True))
    if fault == 'epoch': c.node._control_reference_epoch_ns += 1
    if fault == 'stage': c.node._pregrasp_task_stage = GraspStages.PLAN_PREGRASP
    if fault == 'selection': c.params['/grasp_mode/selection']['generation'] += 1
    if fault == 'model': c.params['/robot_description'] += ' '
    if fault == 'config': c.params['/grasp']['changed'] = True
    if fault == 'gripper_geometry': c.params['/grasp_6d/remote/gripper_geometry']['changed'] = True
    if fault == 'gripper_config': c.params['/gripper']['changed'] = True
    if fault == 'actuation': c.node._fresh_actuation_status = lambda: 'PENDING'
    if fault == 'disconnect': c.state['connected'] = False
    if fault in ('terminal', 'inactive'):
        event = NS(header=NS(stamp=rospy.Time.from_sec(c.state['now'])), active=False,
                   stage=GraspStages.FAILED if fault == 'terminal' else GraspStages.IDLE,
                   state='FAILED' if fault == 'terminal' else 'IDLE', message='cancelled')
        c.node._observation_task_state_cb(event)
    if fault == 'nonvisual': msg.diagnostic = 'TARGET_GEOMETRY_CHANGED'
    if fault == 'different_track': msg.target_track_id = 'changed-target'
    if fault == 'valid_different_track':
        msg = deepcopy(c.plan); msg.target_track_id = 'changed-target'
    if fault == 'frozen_plan_tampered':
        c.contexts[-1]['plan'].object_geometry.pose_base.position.x += .001
    if fault == 'future': msg.header.stamp = rospy.Time.from_sec(c.state['now'] + 1.)
    if fault == 'old': msg.header.stamp = rospy.Time.from_sec(1.)
    if fault == 'frame': msg.header.frame_id = 'camera_link'
    if fault == 'monotonic_deadline': c.contexts[-1]['deadline'] = time.monotonic() - 1.
    if fault == 'session_ros_deadline': c.state['now'] = c.origin['view_session']['deadline_ns'] * 1e-9 + .001
    if fault == 'session_replaced': c.node._observe_return_origin = deepcopy(c.origin)
    if fault == 'session_extended': c.origin['deadline'] += 60.
    c.node._committed_observation_plan_cb(msg)
    with pytest.raises(ObservationPathError):
        c.node._pregrasp_context(c.plan)
    assert c.state['moves'] == 1


def test_task_guard_reports_actual_disconnected_inactive_latched_or_blocked_condition(monkeypatch):
    c = lifecycle_case(monkeypatch)
    for fault, expected in [('blocked', 'OBSERVATION_CONTEXT_BLOCKED'),
                            ('inactive', 'OBSERVATION_TASK_INACTIVE'),
                            ('latched', 'OBSERVATION_TASK_REQUIRES_INACTIVE'),
                            ('disconnected', 'OBSERVATION_TASK_PUBLISHER_DISCONNECTED')]:
        c.node._observation_contexts.active = fault != 'inactive'
        c.node._observation_contexts.blocked = fault == 'blocked'
        c.node._observation_task_requires_inactive = fault == 'latched'
        c.state['connected'] = fault != 'disconnected'
        with pytest.raises(ObservationPathError, match=expected):
            c.node._require_live_observation_task()
