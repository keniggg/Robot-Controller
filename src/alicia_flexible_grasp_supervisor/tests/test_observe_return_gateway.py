"""Offline return orchestration: no ROS master, controller or serial connection."""
from copy import deepcopy
import json
import math
import threading
from types import SimpleNamespace as NS
from unittest import mock

import numpy as np
import pytest
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import RobotTrajectory

from test_motion_gateway_controller_start import MotionGateway
import test_grasp_task_sequence as task_fixture
from test_stationary_following import XML
from test_endpoint_correction import sample
from alicia_flexible_grasp.robot import observe_return_gateway as module
from alicia_flexible_grasp.robot.observation_path_guard import CommittedObservationContexts
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES
from alicia_flexible_grasp.grasp.grasp_state_machine import GraspStages
from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id


def fixture(monkeypatch):
    plan = task_fixture.GraspTaskSequenceTest()._rich_plan(stamp_sec=9.)
    plan.model_choice = 'unknown_tabletop'
    plan.diagnostic = 'FAR_FIELD_OBSERVATION_PLAN'
    plan.plan_id = compute_plan_id(plan)
    state = dict(now=10., sdk=np.array([.006]*6), actual=np.zeros(6),
                 moves=0, sample_mutator=None, response=True, path_mutator=None,
                 before_submit_mutator=None, planner_ok=True)
    config = dict(plan_validity_sec=20., unknown_measured_endpoint_position_tolerance_m=.01,
                  measured_endpoint_position_tolerance_m=.006,
                  measured_endpoint_orientation_tolerance_deg=5.)
    params = {'/grasp':config, '/grasp_mode/selection':dict(mode='unknown',
        strategy='observe_return', generation=1, stamp_ns='2000000000'),
        '/robot_description':XML, '/robot/observation_tracking_contract_enabled':True}
    monkeypatch.setattr(module.rospy, 'get_param', lambda k,d=None:params.get(k,d))
    monkeypatch.setattr(module.rospy, 'set_param', lambda k,v:params.update({k:v}))
    monkeypatch.setattr(module.rospy, 'get_time', lambda:state['now'])
    monkeypatch.setattr(module.time, 'monotonic', lambda:state['now'])
    monkeypatch.setattr(module.time, 'sleep', lambda sec:state.update(now=state['now']+sec))
    node = MotionGateway.__new__(MotionGateway)
    node.joint_names = list(ARM_NAMES)+['right_finger']
    node._planner_lock = threading.RLock()
    node._observation_context_lock = threading.RLock()
    node._observation_contexts = CommittedObservationContexts()
    node._observation_contexts.set_active(True)
    node._observation_contexts.ingest(plan)
    node._require_live_observation_task = mock.Mock()
    node._observation_task_connection_available = mock.Mock(return_value=True)
    node._observation_task_state_stamp_ns = 9_500_000_000
    node._observation_task_requires_inactive = False
    node._manual_control_active = lambda:False
    node._pregrasp_task_stage = GraspStages.MOVE_PREGRASP
    node._compensation_task_label = module.CAPTURE_LABEL
    node._control_reference_epoch_ns = 1_000_000_000
    node._fresh_actuation_status = lambda:'CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE'
    node._command_reference_snapshot = lambda:NS(positions=[
        state['sdk'][ARM_NAMES.index(n)] for n in node.joint_names[:-1]])
    def snapshot(fk, epoch, context, after_ns=0, context_validator=None):
        context_validator()
        if after_ns:
            state['now'] += 1.
        result = sample(fk, state['sdk'], state['actual'], state['now'])
        if state['sample_mutator']:
            state['sample_mutator'](result, after_ns)
        return result
    node._endpoint_following_snapshot = snapshot
    node._ensure_trajectory_controllers_started = mock.Mock(return_value=(True, 'ready'))
    node._synchronize_trajectory_controller_to_feedback = mock.Mock(return_value=(True, 'ready'))
    node._configure_controller_reference_guard = mock.Mock()
    node._configure_observation_timing = mock.Mock(return_value=('observe_return',))
    node._observation_timing_current = mock.Mock(return_value=True)
    def validate_path(ctx, trajectory, planner, context_validator):
        context_validator()
        return dict(controller_constraints_snapshot={'goal_time':.5},
                    execution_tracking_contract={'policy':'test_bound'},
                    trajectory_sha256='offline_fixture')
    node._validate_frozen_observation_path = mock.Mock(side_effect=validate_path)
    node._revalidate_observation_contract_submission = mock.Mock()
    planner = NS(observation_path_guard_required=False, observation_path_validator=None,
        observation_tracking_contract_required=False, observation_trajectory_executor=NS(),
        strict_execution_max_joint_velocity_rad_s=.12,
        strict_execution_joint_velocity_limits_rad_s={'Joint1':.1})
    calls = []
    def execute(goal, execute, start_joint_positions=None, before_execute=None):
        calls.append(dict(goal=list(goal), execute=execute, start=list(start_joint_positions)))
        assert planner.observation_path_guard_required and planner.observation_tracking_contract_required
        assert planner.observation_execution_authorized()
        if not execute:
            return True, 'planned only'
        trajectory = RobotTrajectory(joint_trajectory=JointTrajectory(joint_names=list(ARM_NAMES),
            points=[JointTrajectoryPoint(positions=list(start_joint_positions), time_from_start=rospy.Duration(0.)),
                    JointTrajectoryPoint(positions=list(goal), time_from_start=rospy.Duration(4.))]))
        if state['path_mutator']:
            state['path_mutator'](trajectory)
        try:
            audit = planner.observation_path_validator(trajectory)
            if state['before_submit_mutator']:
                state['before_submit_mutator']()
            before_execute()
            planner.observation_submission_revalidate(trajectory, audit)
        except Exception as exc:
            return False, str(exc)
        if not state['planner_ok']:
            return False, 'controller rejected'
        state['moves'] += 1
        state['now'] += 5.
        state['sdk'] = np.array(goal)
        if callable(state['response']):
            state['actual'] = state['response'](np.array(goal))
        elif state['response']:
            state['actual'] = np.array(goal)
        return True, 'action succeeded'
    planner.plan_and_execute_joint_probe = execute
    node._ensure_planner = lambda:planner
    return node, planner, plan, state, params, calls


def capture(node, plan, execute=False):
    return node.handle_capture_observe_return_origin(NS(plan=plan, execute=execute))


def begin_return(node, state):
    node._compensation_task_label = module.RETURN_LABEL
    state['now'] = 20.
    state['sdk'] = np.array([.15]*6)
    state['actual'] = np.array([.14]*6)


def run_return(node, plan, execute=True):
    return node.handle_return_observe_return_origin(NS(plan=plan, execute=execute))


def test_capture_is_readonly_and_uses_measured_origin_not_old_sdk(monkeypatch):
    node, planner, plan, state, params, calls = fixture(monkeypatch)
    result = capture(node, plan, execute=True)
    report = json.loads(result.message)
    assert result.success, report
    assert report['code'] == 'OBSERVE_RETURN_ORIGIN_CAPTURED'
    assert report['origin_joint_positions_rad'] == [0.]*6
    assert report['origin_joint_positions_rad'] != state['sdk'].tolist()
    assert report['origin_tool_pose']['frame_id'] == 'base_link'
    assert report['physical_motion_submitted'] is False
    assert not calls and state['moves'] == 0
    node._ensure_trajectory_controllers_started.assert_not_called()
    node._synchronize_trajectory_controller_to_feedback.assert_not_called()
    initial = deepcopy(report)
    state['actual'] = np.array([.1]*6)
    repeated = capture(node, plan)
    assert repeated.success
    assert json.loads(repeated.message)['origin_joint_positions_rad'] == initial['origin_joint_positions_rad']
    assert json.loads(repeated.message)['session_id'] == initial['session_id']


def test_return_replans_from_current_reference_binds_full_path_and_verifies_new_samples(monkeypatch):
    node, planner, plan, state, params, calls = fixture(monkeypatch)
    assert capture(node, plan).success
    initial_sdk = state['sdk'].copy()
    begin_return(node, state)
    baseline = state['sdk'].copy()
    result = run_return(node, plan)
    report = json.loads(result.message)
    assert result.success, report
    assert report['code'] == 'OBSERVE_RETURN_MEASURED_ARRIVAL'
    assert calls == [dict(goal=[0.]*6, execute=True, start=baseline.tolist())]
    assert calls[0]['goal'] != initial_sdk.tolist()
    assert state['moves'] == 1 and report['post_command_feedback_valid']
    assert report['stationary_window_start_ns'] > report['completion_stamp_ns']
    assert report['position_error_m'] == 0. and report['maximum_joint_error_rad'] == 0.
    assert report['real_grasp_success'] is False
    node._validate_frozen_observation_path.assert_called_once()
    node._revalidate_observation_contract_submission.assert_called_once()
    assert planner.strict_execution_max_joint_velocity_rad_s == .12
    assert planner.strict_execution_joint_velocity_limits_rad_s == {'Joint1':.1}
    assert planner.observation_path_guard_required is False
    assert not hasattr(planner, 'observation_execution_authorized')
    repeated = run_return(node, plan)
    assert not repeated.success and 'ALREADY_SUBMITTED' in repeated.message
    assert state['moves'] == 1


def test_return_dry_run_never_starts_controllers_or_consumes_episode(monkeypatch):
    node, planner, plan, state, params, calls = fixture(monkeypatch)
    assert capture(node, plan).success
    begin_return(node, state)
    result = run_return(node, plan, execute=False)
    assert result.success and json.loads(result.message)['code'] == 'OBSERVE_RETURN_PLANNED_ONLY'
    assert state['moves'] == 0 and calls[0]['execute'] is False
    assert not node._observe_return_origin['submitted']
    assert not any(k.startswith('/supervisor/observe_return_episodes/') for k in params)
    node._ensure_trajectory_controllers_started.assert_not_called()
    node._synchronize_trajectory_controller_to_feedback.assert_not_called()


@pytest.mark.parametrize('mutation, reason', [
    (lambda n,p,s,ps:setattr(n, '_manual_control_active', lambda:True), 'MANUAL'),
    (lambda n,p,s,ps:setattr(n, '_control_reference_epoch_ns', 2_000_000_000), 'EPOCH_CHANGED'),
    (lambda n,p,s,ps:ps['/grasp_mode/selection'].update(strategy='direct'), 'SELECTION'),
    (lambda n,p,s,ps:ps['/grasp_mode/selection'].update(generation=2), 'SELECTION_CHANGED'),
    (lambda n,p,s,ps:ps.update({'/robot_description':XML+' '}), 'BOUND_INPUT_CHANGED'),
    (lambda n,p,s,ps:ps['/grasp'].update(plan_validity_sec=25.), 'BOUND_INPUT_CHANGED'),
    (lambda n,p,s,ps:setattr(n, '_compensation_task_label', 'moving pregrasp'), 'STAGE_CHANGED'),
    (lambda n,p,s,ps:setattr(n, '_pregrasp_task_stage', GraspStages.APPROACH_TARGET), 'STAGE_CHANGED'),
    (lambda n,p,s,ps:n._observation_contexts.clear(), 'revoked'),
    (lambda n,p,s,ps:s.update(now=700.), 'SESSION_EXPIRED'),
    (lambda n,p,s,ps:s.update(now=8.), 'SOURCE_EPOCH_INVALID'),
    (lambda n,p,s,ps:setattr(n, '_fresh_actuation_status', lambda:'UNCONFIRMED:timeout'), 'UNCONFIRMED'),
    (lambda n,p,s,ps:setattr(p, 'target_track_id', 'another'), 'REQUEST_PLAN_CHANGED'),
])
def test_changed_authority_rejects_return_before_movement(monkeypatch, mutation, reason):
    node, planner, plan, state, params, calls = fixture(monkeypatch)
    assert capture(node, plan).success
    begin_return(node, state)
    mutation(node, plan, state, params)
    result = run_return(node, plan)
    assert not result.success and reason in result.message, result.message
    assert not calls and state['moves'] == 0


@pytest.mark.parametrize('key,value', [
    ('unknown_measured_endpoint_position_tolerance_m', .011),
    ('measured_endpoint_orientation_tolerance_deg', 6.),
    ('unknown_observe_return_joint_tolerance_rad', .036),
    ('unknown_observe_return_session_timeout_sec', 0.),
])
def test_invalid_capture_configuration_never_records_origin(monkeypatch, key, value):
    node, _, plan, state, params, calls = fixture(monkeypatch)
    params['/grasp'][key] = value
    result = capture(node, plan)
    assert not result.success and not hasattr(node, '_observe_return_origin')
    assert not calls


@pytest.mark.parametrize('mutator', [
    lambda report,after:report.update(accepted_stamp_ns=1_000_000_000),
    lambda report,after:report.update(accepted_stamp_ns=99_000_000_000),
    lambda report,after:report.update(accepted_sample_count=1),
    lambda report,after:report.update(model_sha256='different'),
    lambda report,after:report.update(epoch_ns=2_000_000_000),
    lambda report,after:report.update(joint_names=['Joint1']*6),
])
def test_stale_or_ambiguous_capture_feedback_cannot_be_an_origin(monkeypatch, mutator):
    node, _, plan, state, _, calls = fixture(monkeypatch)
    state['sample_mutator'] = mutator
    result = capture(node, plan)
    assert not result.success and 'MEASUREMENT' in result.message
    assert not calls and not hasattr(node, '_observe_return_origin')


def test_old_post_action_sample_cannot_be_return_success(monkeypatch):
    node, _, plan, state, _, calls = fixture(monkeypatch)
    assert capture(node, plan).success
    begin_return(node, state)
    def old_sample(report, after):
        if after:
            report['stationary_window_start_ns'] = after
    state['sample_mutator'] = old_sample
    result = run_return(node, plan)
    report = json.loads(result.message)
    assert not result.success and 'PRE_COMMAND' in report['code']
    assert state['moves'] == 1 and report['controller_action_completed']
    assert report['post_command_feedback_valid'] is False
    assert not run_return(node, plan).success and state['moves'] == 1


def test_controller_success_without_real_arrival_cannot_continue_or_retry(monkeypatch):
    node, _, plan, state, _, _ = fixture(monkeypatch)
    assert capture(node, plan).success
    begin_return(node, state)
    state['response'] = False
    result = run_return(node, plan)
    report = json.loads(result.message)
    assert not result.success and report['code'] == 'OBSERVE_RETURN_MEASURED_ENDPOINT_OUTSIDE_TOLERANCE'
    assert report['controller_action_completed'] and report['post_command_feedback_valid']
    assert report['position_error_m'] > .01
    assert state['moves'] == 1 and not run_return(node, plan).success


def test_changed_joint_endpoint_rejects_before_path_proof_or_submission(monkeypatch):
    node, _, plan, state, _, _ = fixture(monkeypatch)
    assert capture(node, plan).success
    begin_return(node, state)
    state['path_mutator'] = lambda tr:tr.joint_trajectory.points[-1].positions.__setitem__(2, .001)
    result = run_return(node, plan)
    assert not result.success and 'PATH_GOAL_CHANGED' in result.message
    node._validate_frozen_observation_path.assert_not_called()
    assert state['moves'] == 0 and not node._observe_return_origin['submitted']


def test_reference_change_during_final_proof_blocks_submission(monkeypatch):
    node, _, plan, state, _, _ = fixture(monkeypatch)
    assert capture(node, plan).success
    begin_return(node, state)
    state['before_submit_mutator'] = lambda:state['sdk'].__setitem__(1, .17)
    result = run_return(node, plan)
    assert not result.success and 'START_CHANGED' in result.message
    assert state['moves'] == 0 and not node._observe_return_origin['submitted']


def test_geometry_failure_never_submits_return(monkeypatch):
    node, _, plan, state, _, _ = fixture(monkeypatch)
    assert capture(node, plan).success
    begin_return(node, state)
    node._validate_frozen_observation_path.side_effect = ValueError('support clearance rejected')
    result = run_return(node, plan)
    assert not result.success and 'support clearance rejected' in result.message
    assert state['moves'] == 0 and not node._observe_return_origin['submitted']


def test_joint_name_permutation_does_not_replay_wrong_axis(monkeypatch):
    node, _, plan, state, _, calls = fixture(monkeypatch)
    node.joint_names = list(reversed(ARM_NAMES))+['right_finger']
    state['sdk'] = np.arange(6)*.001
    assert capture(node, plan).success
    begin_return(node, state)
    state['sdk'] = np.arange(6)*.01
    state['actual'] = state['sdk'].copy()
    expected = state['sdk'].tolist()
    result = run_return(node, plan)
    assert result.success, result.message
    assert calls[0]['start'] == expected and calls[0]['goal'] == [0.]*6


@pytest.mark.parametrize('mutation', [
    lambda n,p,s,ps:ps['/grasp_mode/selection'].update(mode='carton'),
    lambda n,p,s,ps:ps['/grasp_mode/selection'].update(strategy='two_stage'),
    lambda n,p,s,ps:ps['/grasp_mode/selection'].update(stamp_ns='999000000000'),
    lambda n,p,s,ps:setattr(n, '_compensation_task_label', module.RETURN_LABEL),
    lambda n,p,s,ps:setattr(n, '_manual_control_active', lambda:True),
    lambda n,p,s,ps:n._observation_contexts.set_active(False),
    lambda n,p,s,ps:setattr(p, 'model_choice', 'carton'),
])
def test_capture_cannot_create_authority_from_request_alone(monkeypatch, mutation):
    node, _, plan, state, params, calls = fixture(monkeypatch)
    mutation(node, plan, state, params)
    result = capture(node, plan)
    assert not result.success and not hasattr(node, '_observe_return_origin')
    assert calls == []


def test_capture_rejects_a_different_committed_geometry_digest(monkeypatch):
    node, _, plan, _, _, calls = fixture(monkeypatch)
    plan.poses[0].position.x += .001
    plan.plan_id = compute_plan_id(plan)
    result = capture(node, plan)
    assert not result.success and 'COMMITTED_PLAN_MISMATCH' in result.message
    assert calls == []


@pytest.mark.parametrize('residual, expected', [(.03, True), (.04, False)])
def test_unknown_ten_mm_policy_keeps_separate_joint_config_limit(monkeypatch, residual, expected):
    node, _, plan, state, _, _ = fixture(monkeypatch)
    assert capture(node, plan).success
    begin_return(node, state)
    state['response'] = lambda goal:goal+np.array([residual, 0., 0., 0., 0., 0.])
    result = run_return(node, plan)
    report = json.loads(result.message)
    assert result.success is expected, report
    assert .006 < report['position_error_m'] <= .01
    assert report['orientation_error_rad'] < math.radians(5)
    assert report['maximum_joint_error_rad'] == pytest.approx(residual)
    assert report['position_tolerance_m'] == .01


def test_mutated_cached_origin_cannot_change_return_goal(monkeypatch):
    node, _, plan, state, _, calls = fixture(monkeypatch)
    assert capture(node, plan).success
    begin_return(node, state)
    node._observe_return_origin['origin_joints'] = (.01,)*6
    result = run_return(node, plan)
    assert not result.success and 'ORIGIN_GOAL_CHANGED' in result.message
    assert not calls


def test_insufficient_path_time_and_submission_failure_never_claim_arrival(monkeypatch):
    node, _, plan, state, _, _ = fixture(monkeypatch)
    assert capture(node, plan).success
    begin_return(node, state)
    state['path_mutator'] = lambda tr:setattr(tr.joint_trajectory.points[-1],
                                             'time_from_start', rospy.Duration(200.))
    result = run_return(node, plan)
    assert not result.success and 'PATH_TIME_BUDGET' in result.message
    assert state['moves'] == 0
    state['path_mutator'] = None
    state['planner_ok'] = False
    result = run_return(node, plan)
    report = json.loads(result.message)
    assert not result.success and report['command_submission_attempted']
    assert 'controller_action_completed' not in report and 'position_error_m' not in report
    assert not run_return(node, plan).success and state['moves'] == 0


@pytest.mark.parametrize('operation', ['capture', 'return'])
def test_service_can_precede_its_task_label_on_another_connection(monkeypatch, operation):
    node, _, plan, state, _, _ = fixture(monkeypatch)
    if operation == 'return':
        assert capture(node, plan).success
        begin_return(node, state)
    expected = module.RETURN_LABEL if operation == 'return' else module.CAPTURE_LABEL
    node._compensation_task_label = 'previous same-task phase'
    started = state['now']
    def state_arrives(seconds):
        state['now'] += seconds
        node._compensation_task_label = expected
    monkeypatch.setattr(module.time, 'sleep', state_arrives)
    result = run_return(node, plan) if operation == 'return' else capture(node, plan)
    assert result.success, result.message
    assert state['now'] >= started+.05


@pytest.mark.parametrize('change', ['epoch', 'selection', 'inactive'])
def test_label_wait_cannot_borrow_authority_from_another_session(monkeypatch, change):
    node, _, plan, state, params, calls = fixture(monkeypatch)
    assert capture(node, plan).success
    begin_return(node, state)
    node._compensation_task_label = 'previous same-task phase'
    def changed(seconds):
        state['now'] += seconds
        node._compensation_task_label = module.RETURN_LABEL
        if change == 'epoch':
            node._control_reference_epoch_ns += 1
        elif change == 'selection':
            params['/grasp_mode/selection']['generation'] += 1
        else:
            node._observation_contexts.set_active(False)
    monkeypatch.setattr(module.time, 'sleep', changed)
    result = run_return(node, plan)
    assert not result.success and not calls and state['moves'] == 0


def test_ros_clock_before_saved_origin_rejects_even_if_plan_source_is_older(monkeypatch):
    node, _, plan, state, _, calls = fixture(monkeypatch)
    assert capture(node, plan).success
    begin_return(node, state)
    state['now'] = 9.5  # Plan source=9, but captured accepted source=9.951.
    result = run_return(node, plan)
    assert not result.success and 'CLOCK_PREDATES_ORIGIN' in result.message
    assert not calls


def test_new_selection_cannot_capture_older_plan(monkeypatch):
    node, planner, plan, state, params, calls = fixture(monkeypatch)
    params['/grasp_mode/selection']['stamp_ns'] = '9500000000'
    result = capture(node, plan)
    assert not result.success
    assert 'SOURCE_EPOCH_INVALID' in result.message
    assert not calls
    node._ensure_trajectory_controllers_started.assert_not_called()


def test_stage_label_arriving_after_handshake_deadline_is_rejected(monkeypatch):
    node, planner, plan, state, params, calls = fixture(monkeypatch)
    node._compensation_task_label = 'moving to observation pose'
    def delayed_label(_seconds):
        state['now'] += 1.1
        node._compensation_task_label = module.CAPTURE_LABEL
    monkeypatch.setattr(module.time, 'sleep', delayed_label)
    result = capture(node, plan)
    assert not result.success
    assert 'TASK_STAGE_CHANGED_OR_NOT_RECEIVED' in result.message
    assert not calls


def actual_task_state_callback(node, state, *, active=True, stamp=None, label=None):
    stamp = state['now'] if stamp is None else stamp
    message = NS(header=NS(stamp=rospy.Time.from_sec(stamp)),
        stage=GraspStages.MOVE_PREGRASP, state='MOVE_PREGRASP', active=active,
        message=module.CAPTURE_LABEL if label is None else label)
    node._observation_task_state_cb(message)


def test_capture_waits_for_first_active_callback_before_registry_capture(monkeypatch):
    node, planner, plan, state, params, calls = fixture(monkeypatch)
    node._observation_contexts.set_active(False)
    node._observation_task_state_stamp_ns = 8_000_000_000
    node._compensation_task_label = ''
    node._observation_task_sub = NS(get_num_connections=lambda:1)
    del node._require_live_observation_task
    del node._observation_task_connection_available
    def deliver_active(seconds):
        state['now'] += seconds
        actual_task_state_callback(node, state)
    monkeypatch.setattr(module.time, 'sleep', deliver_active)
    result = capture(node, plan)
    assert result.success, result.message
    assert state['now'] == pytest.approx(10.05)
    assert node._observation_contexts.active_scene.digest == module.committed_plan_digest(plan)
    assert not calls
    node._ensure_trajectory_controllers_started.assert_not_called()


def test_old_active_callback_cannot_supply_new_capture_authority(monkeypatch):
    node, planner, plan, state, params, calls = fixture(monkeypatch)
    node._observation_task_state_stamp_ns = 8_000_000_000
    result = capture(node, plan)
    assert not result.success
    assert 'ACTIVE_TASK_NOT_RECEIVED' in result.message
    assert state['now'] == pytest.approx(10.5)
    assert node._observation_contexts.active_scene is None
    assert not calls


@pytest.mark.parametrize('mutation,reason', [
    ('selection', 'SELECTION_CHANGED'), ('epoch', 'CONTROL_EPOCH_CHANGED'),
    ('manual', 'MANUAL_CONTROL_ACTIVE'), ('clear', 'TASK_ACTIVATION_CHANGED'),
    ('replacement_task', 'TASK_ACTIVATION_CHANGED'), ('disconnected', 'ACTIVE_TASK_UNAVAILABLE')])
def test_first_active_wait_rejects_authority_change(monkeypatch, mutation, reason):
    node, planner, plan, state, params, calls = fixture(monkeypatch)
    node._observation_contexts.set_active(False)
    node._observation_task_state_stamp_ns = 8_000_000_000
    node._compensation_task_label = ''
    def deliver_changed_task(seconds):
        state['now'] += seconds
        if mutation == 'selection':
            params['/grasp_mode/selection']['generation'] += 1
        elif mutation == 'epoch':
            node._control_reference_epoch_ns += 1
        elif mutation == 'manual':
            node._manual_control_active = lambda:True
        elif mutation == 'clear':
            node._observation_contexts.clear()
        elif mutation == 'replacement_task':
            actual_task_state_callback(node, state)
            state['now'] += .001
            actual_task_state_callback(node, state, active=False)
            state['now'] += .001
            actual_task_state_callback(node, state)
        elif mutation == 'disconnected':
            node._observation_task_connection_available = lambda:False
        if mutation != 'replacement_task':
            actual_task_state_callback(node, state)
    monkeypatch.setattr(module.time, 'sleep', deliver_changed_task)
    result = capture(node, plan)
    assert not result.success
    assert reason in result.message
    assert not calls
    node._ensure_trajectory_controllers_started.assert_not_called()


def test_activation_revision_is_rechecked_before_registry_capture(monkeypatch):
    node, planner, plan, state, params, calls = fixture(monkeypatch)
    original_wait = node._await_observe_return_active
    def replace_after_wait(*args):
        revision = original_wait(*args)
        node._observation_contexts.set_active(False)
        node._observation_contexts.set_active(True)
        return revision
    node._await_observe_return_active = replace_after_wait
    result = capture(node, plan)
    assert not result.success
    assert 'TASK_ACTIVATION_CHANGED' in result.message
    assert node._observation_contexts.active_scene is None
    assert not calls
