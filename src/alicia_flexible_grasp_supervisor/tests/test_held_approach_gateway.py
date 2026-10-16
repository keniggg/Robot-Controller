"""Atomic pose1 held-axes route; synthetic responses are not hardware evidence."""
from copy import deepcopy
from pathlib import Path
import json
import time
from types import SimpleNamespace as NS
from unittest import mock

import numpy as np
import pytest

from alicia_flexible_grasp.grasp.grasp_state_machine import GraspStages
from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id, pose_values
from alicia_flexible_grasp.robot import pregrasp_gateway as module
from alicia_flexible_grasp.robot.endpoint_correction import CorrectionStep, sdk_counts
from alicia_flexible_grasp.robot.observation_path_guard import FrozenObservationScene
from alicia_flexible_grasp.robot.pregrasp_compensation import PregraspCompensation
from alicia_flexible_grasp.robot.pregrasp_feedforward import PregraspFeedforward
from alicia_flexible_grasp.robot.stationary_following import SDK_QUANTUM_RAD as Q
from test_gateway_tracking_probe import path_fixture
from test_pregrasp_feedforward import held_gateway_case
from test_pregrasp_gateway import real_context


APPROACH_CONFIG = dict(unknown_approach_held_axes_feedforward_enabled=True,
    unknown_contact_cartesian_compensation_enabled=True,
    contact_endpoint_precision_enabled=True, measured_endpoint_check_enabled=True)


def approach_case(monkeypatch, response=True):
    node, planner, plan, state, calls, params, context = held_gateway_case(monkeypatch, response)
    record = json.loads((Path(__file__).parent/'fixtures/contact_no_response_20260927.json').read_text())
    state['sdk'] = np.array(record['before']['sdk_positions_rad'])
    state['actual'] = np.array(record['before']['accepted_positions_rad'])
    goal = plan.poses[1]
    values = record['fixed_goal_pose']
    goal.position.x, goal.position.y, goal.position.z = values[:3]
    goal.orientation.x, goal.orientation.y, goal.orientation.z, goal.orientation.w = values[3:]
    plan.poses[0].position.z += .05  # A wrong pose0 lookup must fail this test.
    plan.plan_id = compute_plan_id(plan)
    context.update(pose_index=1, deadline=time.monotonic()+60.)
    context['scene'].plan_id = plan.plan_id
    context['config'].update(APPROACH_CONFIG)
    # Approach must force held axes, independently of the pregrasp switches.
    context['config'].update(unknown_pregrasp_feedforward_enabled=False,
                            unknown_pregrasp_held_axes_feedforward_enabled=False)
    node._pregrasp_context = lambda _plan, index=0: context
    return node, planner, plan, state, calls, params, context


def test_atomic_approach_uses_frozen_pose1_single_episode_and_strict_executor(monkeypatch):
    node, planner, plan, state, calls, params, context = approach_case(monkeypatch)
    fixed = deepcopy(plan)
    original_execute = planner.plan_and_execute_joint_probe
    def execute(*args, **kwargs):
        assert planner.strict_execution_max_joint_velocity_rad_s == .002
        assert planner.strict_collinear_timing_enabled is True
        assert planner.observation_path_guard_required
        assert planner.observation_tracking_contract_required
        assert callable(planner.observation_submission_revalidate)
        return original_execute(*args, **kwargs)
    planner.plan_and_execute_joint_probe = execute
    monkeypatch.setattr(PregraspCompensation, 'evaluate', mock.Mock(
        side_effect=AssertionError('small-step fallback is forbidden')))
    result = node.handle_compensate_approach(NS(plan=plan, execute=True))
    report = json.loads(result.message)
    assert result.success, report
    assert report['code'] == 'PREGRASP_MEASURED_CONVERGED'
    assert report['execution_policy'] == 'held_axes_approach'
    assert report['approach_convergence_policy'] == 'bounded_approach_max_6mm'
    assert report['position_tolerance_m'] == .006
    assert context['config']['unknown_measured_endpoint_position_tolerance_m'] == .010
    assert report['fixed_goal_pose_index'] == 1
    assert report['fixed_goal_pose'] == list(pose_values(plan.poses[1]))
    assert report['plan_id'] == plan.plan_id and report['epoch_ns'] == context['epoch']
    assert report['accepted_stamp_ns'] > 12_000_000_000
    assert report['model_sha256'] and report['hardware_response_verified']
    assert report['proposed_delta_counts'] == [-4,0,-18,0,0,32]
    assert state['moves'] == len(calls) == len(params) == 1
    assert plan == fixed
    episode = next(iter(params.values()))
    assert episode['fixed_goal_pose_index'] == 1 and episode['steps_submitted'] == 1
    assert episode['execution_policy'] == 'held_axes_approach'
    assert not node.handle_compensate_approach(NS(plan=plan, execute=True)).success
    assert len(calls) == 1
    assert planner.strict_execution_max_joint_velocity_rad_s == .12
    assert planner.observation_tracking_contract_required is False
    assert not hasattr(planner, 'strict_collinear_timing_enabled')
    planner._plan_cartesian_from_start_state.assert_not_called()


def test_atomic_approach_rejects_dry_run_without_starting_an_episode(monkeypatch):
    node, planner, plan, state, calls, params, _ = approach_case(monkeypatch)
    result = node.handle_compensate_approach(NS(plan=plan, execute=False))
    assert not result.success and 'ATOMIC_EXECUTION_REQUIRED' in result.message
    assert not calls and not params and state['moves'] == 0
    node._ensure_trajectory_controllers_started.assert_not_called()
    planner._plan_cartesian_from_start_state.assert_not_called()


@pytest.mark.parametrize('value', ['true', 1, None, {}])
def test_malformed_approach_switch_cannot_fall_back(monkeypatch, value):
    node, planner, plan, state, calls, params, context = approach_case(monkeypatch)
    context['config']['unknown_approach_held_axes_feedforward_enabled'] = value
    result = node.handle_compensate_approach(NS(plan=plan, execute=True))
    assert not result.success and 'CONFIG_INVALID' in result.message
    assert not calls and not params


@pytest.mark.parametrize('fault', ['stall', 'held_axis', 'freshness', 'epoch', 'model', 'budget'])
def test_failed_actual_response_never_becomes_arrival_or_another_episode(monkeypatch, fault):
    def response(delta):
        if fault == 'stall': return np.zeros(6)
        if fault == 'held_axis': delta[1] = 3*Q
        return delta
    node, planner, plan, state, calls, params, _ = approach_case(monkeypatch, response)
    original_snapshot = node._endpoint_following_snapshot
    def snapshot(*args, **kwargs):
        result = original_snapshot(*args, **kwargs)
        if kwargs.get('after_ns'):
            if fault == 'freshness': result['stationary_window_start_ns'] = kwargs['after_ns']
            if fault == 'epoch': result['epoch_ns'] += 1
            if fault == 'model': result['model_sha256'] = 'changed'
            if fault == 'budget': result['stamp_sec'] += 61.
        return result
    node._endpoint_following_snapshot = snapshot
    result = node.handle_compensate_approach(NS(plan=plan, execute=True))
    assert not result.success, result.message
    assert json.loads(result.message)['execution_policy'] == 'held_axes_approach'
    assert len(calls) == state['moves'] == len(params) == 1
    assert not node.handle_compensate_approach(NS(plan=plan, execute=True)).success
    assert len(calls) == 1
    planner._plan_cartesian_from_start_state.assert_not_called()


@pytest.mark.parametrize('fault', ['feedback', 'reference', 'authority', 'guide', 'execution'])
def test_changed_start_or_rejected_path_never_uses_an_alternative(monkeypatch, fault):
    node, planner, plan, state, calls, params, _ = approach_case(monkeypatch)
    original_execute = planner.plan_and_execute_joint_probe
    def execute(*args, **kwargs):
        if fault == 'feedback': state['actual'][0] += 3*Q
        if fault == 'reference': state['sdk'][0] += Q
        if fault == 'authority':
            node._validate_pregrasp_context.side_effect = ValueError('authority changed')
        if fault == 'execution':
            kwargs['before_execute']()
            return False, 'proof-bound trajectory failed'
        return original_execute(*args, **kwargs)
    planner.plan_and_execute_joint_probe = execute
    if fault == 'guide':
        monkeypatch.setattr(PregraspFeedforward, 'propose_held_axes', mock.Mock(
            side_effect=ValueError('HELD_AXES_GUIDE_DOES_NOT_REACH_FIXED_GOAL')))
    result = node.handle_compensate_approach(NS(plan=plan, execute=True))
    assert not result.success, result.message
    assert state['moves'] == 0 and len(calls) <= 1
    planner._plan_cartesian_from_start_state.assert_not_called()
    if fault == 'execution':
        assert len(params) == 1
        assert not node.handle_compensate_approach(NS(plan=plan, execute=True)).success
    else:
        assert not params


def test_zero_step_approach_reports_same_fixed_goal_and_actual_sample(monkeypatch):
    node, _, plan, state, calls, params, context = approach_case(monkeypatch)
    fk = module.SerialUrdfFk(context['model'], module.ARM_NAMES)
    from tf.transformations import quaternion_from_matrix
    pose = fk(state['actual'])
    goal = plan.poses[1]
    goal.position.x, goal.position.y, goal.position.z = pose[:3, 3]
    goal.orientation.x, goal.orientation.y, goal.orientation.z, goal.orientation.w = quaternion_from_matrix(pose)
    plan.plan_id = compute_plan_id(plan)
    context['scene'].plan_id = plan.plan_id
    result = node.handle_compensate_approach(NS(plan=plan, execute=True))
    report = json.loads(result.message)
    assert result.success, report
    assert report['steps_completed'] == 0 and not calls and not params
    assert report['execution_policy'] == 'held_axes_approach'
    assert report['approach_convergence_policy'] == 'bounded_approach_max_6mm'
    assert report['position_tolerance_m'] == .006
    assert context['config']['unknown_measured_endpoint_position_tolerance_m'] == .010
    assert report['fixed_goal_pose_index'] == 1
    assert report['fixed_goal_pose'] == list(pose_values(plan.poses[1]))
    assert report['model_sha256'] == fk.model_sha256
    assert report['accepted_stamp_ns'] > context['epoch']
    assert not report['real_grasp_success']


@pytest.mark.parametrize('fault', ['none', 'known', 'precision', 'measured', 'contact',
                                  'switch', 'wrong_label', 'wrong_stage', 'epoch', 'deadline'])
def test_live_context_retains_original_gates_and_explicit_bounded_label(monkeypatch, fault):
    node, context, params = real_context(monkeypatch)
    plan = context['plan']
    plan.model_choice = 'unknown_tabletop'
    plan.plan_id = compute_plan_id(plan)
    context.update(pose_index=1, scene=FrozenObservationScene.from_contact_plan(plan))
    params['/grasp'].update(APPROACH_CONFIG)
    node._pregrasp_task_stage = GraspStages.APPROACH_TARGET
    node._compensation_task_label = 'moving bounded 6D approach'
    if fault == 'known': plan.model_choice = 'known'
    if fault == 'precision': params['/grasp']['contact_endpoint_precision_enabled'] = False
    if fault == 'measured': params['/grasp']['measured_endpoint_check_enabled'] = False
    if fault == 'contact': params['/grasp']['unknown_contact_cartesian_compensation_enabled'] = False
    if fault == 'switch': params['/grasp']['unknown_approach_held_axes_feedforward_enabled'] = 1
    if fault == 'wrong_label': node._compensation_task_label = 'moving bounded 6D grasp pose'
    if fault == 'wrong_stage': node._pregrasp_task_stage = GraspStages.MOVE_PREGRASP
    if fault == 'epoch': node._control_reference_epoch_ns += 1
    if fault == 'deadline': context['deadline'] = time.monotonic()-1
    context['config'] = deepcopy(params['/grasp'])
    if fault == 'none': node._validate_pregrasp_context(context)
    else:
        with pytest.raises(ValueError): node._validate_pregrasp_context(context)


@pytest.mark.parametrize('fault', ['none', 'pose2', 'disabled', 'held_target', 'held_excursion',
                                  'count_box', 'deadline', 'proof'])
def test_approach_path_keeps_held_cells_original_total_box_and_full_certificate(monkeypatch, fault):
    node, planner, scene, trajectory = path_fixture(monkeypatch)
    target = np.array([-4,0,-18,0,0,32])
    if fault == 'held_target': target[1] = 1
    trajectory.joint_trajectory.points[-1].positions = (target*Q).tolist()
    step = CorrectionStep((2048,)*6, tuple((target+2048).tolist()), (2048,)*6, 1, 1)
    node._validate_pregrasp_context = mock.Mock()
    node._pregrasp_reference = lambda: (2048,)*6
    node._validate_frozen_observation_path = mock.Mock(return_value={
        'controller_constraints_snapshot': {'goal_time': .5}})
    context = dict(scene=scene, revocation=1, deadline=time.monotonic()+60.,
        initial_sdk_counts=(2048,)*6, pose_index=1,
        plan=NS(model_choice='unknown_tabletop'), config=deepcopy(APPROACH_CONFIG),
        feedforward_method='held_axes_following_feedforward')
    if fault == 'pose2': context['pose_index'] = 2
    if fault == 'disabled': context['config']['unknown_approach_held_axes_feedforward_enabled'] = False
    if fault == 'held_excursion': trajectory.joint_trajectory.points[-1].velocities[4] = .01
    if fault == 'count_box': trajectory.joint_trajectory.points[-1].velocities[5] = -.1
    if fault == 'deadline': context['deadline'] = time.monotonic()+1.
    if fault == 'proof': node._validate_frozen_observation_path.side_effect = ValueError('support proof failed')
    if fault == 'none':
        report = node._validate_pregrasp_step_path(context, step, trajectory, planner, feedforward=True)
        assert report['compensation_pose_index'] == 1
        assert report['compensation_method'] == 'held_axes_following_feedforward'
        node._validate_frozen_observation_path.assert_called_once()
        assert callable(node._validate_frozen_observation_path.call_args.kwargs['context_validator'])
    else:
        with pytest.raises(ValueError):
            node._validate_pregrasp_step_path(context, step, trajectory, planner, feedforward=True)


@pytest.mark.parametrize('fault', ['delayed', 'timeout', 'manual', 'shutdown', 'task_stopped'])
def test_bounded_phase_topic_may_arrive_after_rpc_without_weakening_admission(monkeypatch, fault):
    node, _, plan, state, calls, params, context = approach_case(monkeypatch)
    params['/grasp'] = context['config']
    node._pregrasp_task_stage = GraspStages.MOVE_PREGRASP
    node._compensation_task_label = 'moving pregrasp'
    node._require_live_observation_task = mock.Mock()
    node._manual_control_active = lambda: fault == 'manual'
    monkeypatch.setattr(module.rospy, 'is_shutdown', lambda: fault == 'shutdown')
    clock = [0.]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    def sleep(_duration):
        clock[0] += .25
        if fault == 'delayed':
            node._pregrasp_task_stage = GraspStages.APPROACH_TARGET
            node._compensation_task_label = 'moving bounded 6D approach'
        if fault == 'task_stopped':
            node._require_live_observation_task.side_effect = ValueError('task stopped')
    monkeypatch.setattr(module.rospy, 'sleep', sleep)
    result = node.handle_compensate_approach(NS(plan=plan, execute=True))
    assert result.success is (fault == 'delayed'), result.message
    assert state['moves'] == len(calls) == int(fault == 'delayed')
    if fault != 'delayed':
        assert len(params) == 1  # Configuration only; no episode or controller call.
        node._ensure_trajectory_controllers_started.assert_not_called()
    assert clock[0] <= 1.


@pytest.mark.parametrize('fault', ['revoked', 'expired', 'reference'])
def test_success_is_rechecked_after_measured_evaluation(monkeypatch, fault):
    node, _, plan, state, calls, params, _ = approach_case(monkeypatch)
    original_evaluate = PregraspFeedforward.evaluate
    def evaluate(recovery, sample):
        result = original_evaluate(recovery, sample)
        assert result[0]
        if fault == 'revoked':
            node._validate_pregrasp_context.side_effect = ValueError('authority revoked after FK')
        if fault == 'expired': state['now'] += 1.
        if fault == 'reference': state['sdk'][0] += Q
        return result
    monkeypatch.setattr(PregraspFeedforward, 'evaluate', evaluate)
    result = node.handle_compensate_approach(NS(plan=plan, execute=True))
    assert not result.success, result.message
    assert len(calls) == state['moves'] == len(params) == 1
    assert not node.handle_compensate_approach(NS(plan=plan, execute=True)).success


@pytest.mark.parametrize('fault', ['delay', 'revocation'])
def test_success_reference_lookup_cannot_hide_expired_or_revoked_evidence(monkeypatch, fault):
    node, _, plan, state, calls, params, _ = approach_case(monkeypatch)
    original_reference = node._pregrasp_reference
    def reference():
        result = original_reference()
        if state['moves']:
            if fault == 'delay': state['now'] += 1.
            else: node._validate_pregrasp_context.side_effect = ValueError('revoked during reference')
        return result
    node._pregrasp_reference = reference
    result = node.handle_compensate_approach(NS(plan=plan, execute=True))
    assert not result.success, result.message
    assert len(calls) == state['moves'] == len(params) == 1


def test_approach_cap_preserves_an_already_stricter_configured_tolerance(monkeypatch):
    node, _, plan, state, calls, _, context = approach_case(monkeypatch)
    context['config']['unknown_measured_endpoint_position_tolerance_m'] = .004
    result = node.handle_compensate_approach(NS(plan=plan, execute=True))
    report = json.loads(result.message)
    assert result.success, report
    assert report['position_tolerance_m'] == .004
    assert report['position_error_m'] <= .004
    assert len(calls) == state['moves'] == 1
