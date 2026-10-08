from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace as NS
from unittest import mock

import numpy as np
import pytest
from genpy.message import fill_message_args
from moveit_msgs.msg import RobotState, RobotTrajectory

from alicia_flexible_grasp.robot.pregrasp_feedforward import PregraspFeedforward
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES, SDK_QUANTUM_RAD as Q
from alicia_flexible_grasp.grasp.rich_plan_integrity import pose_values
from test_pregrasp_compensation import recorded_fixture, sample
from test_pregrasp_gateway import make_gateway


def case():
    from geometry_msgs.msg import Pose
    record = json.loads((Path(__file__).parent/'fixtures/pregrasp_feedforward_20260925.json').read_text())
    fk, sdk, actual, values = recorded_fixture('20260925_actual')
    goal = Pose()
    goal.position.x, goal.position.y, goal.position.z = values[:3]
    goal.orientation.x, goal.orientation.y, goal.orientation.z, goal.orientation.w = values[3:]
    nominal = RobotTrajectory()
    fill_message_args(nominal, [record['nominal_trajectory']])
    recovery = PregraspFeedforward(fk, sample(fk, sdk, actual), goal)
    return recovery, nominal, sdk, actual


def test_recorded_start_has_bounded_ik_correction_without_spending_only_joint3():
    recovery, nominal, sdk, actual = case()
    goal = pose_values(recovery.goal)
    step, evidence = recovery.propose(nominal)
    delta = np.asarray(step.target_counts) - step.baseline_counts
    assert delta.tolist() == [-5, 16, 16, 2, -5, 5]
    assert max(abs(delta)) <= 32
    assert evidence['position_error_m'] == pytest.approx(.017482113202982607)
    assert evidence['predicted_position_error_m'] < .0015
    assert not evidence['hardware_response_verified']
    assert not evidence['real_grasp_success']
    recovery.committed(step, 12.)
    # Conditional constant-bias plant, NOT a recorded hardware response.
    success, result = recovery.evaluate(sample(recovery.contract.fk, step.positions,
                                               actual + delta*Q, 14.))
    assert success and result['position_error_m'] < .006
    assert result['code'] == 'PREGRASP_MEASURED_CONVERGED'
    assert pose_values(recovery.goal) == goal
    assert not result['real_grasp_success']
    with pytest.raises(ValueError, match='RESPONSE_NOT_PENDING'):
        recovery.evaluate(sample(recovery.contract.fk, step.positions, actual + delta*Q, 15.))
    with pytest.raises(ValueError, match='ALREADY_PROPOSED'):
        recovery.propose(nominal)


@pytest.mark.parametrize('fault', ['stall', 'reverse', 'overshoot', 'stale', 'epoch',
                                  'model', 'before_completion', 'changed_sdk', 'remaining_error'])
def test_failed_or_invalid_feedback_never_accepts_arrival(fault):
    recovery, nominal, sdk, actual = case()
    step, _ = recovery.propose(nominal)
    recovery.committed(step, 12.)
    delta = np.asarray(step.target_counts) - step.baseline_counts
    response = delta.copy()
    if fault == 'stall': response[:] = 0
    if fault == 'reverse': response[3] = -2
    if fault == 'overshoot': response[3] = 6
    if fault == 'remaining_error': response[[1, 2]] = 4
    after = sample(recovery.contract.fk, step.positions, actual + response*Q, 14.)
    if fault == 'stale': after['stamp_sec'] = 100.
    if fault == 'epoch': after['epoch_ns'] += 1
    if fault == 'model': after['model_sha256'] = 'other'
    if fault == 'before_completion': after['stationary_window_start_ns'] = 11_999_999_999
    if fault == 'changed_sdk': after['sdk_positions_rad'][0] += Q
    try:
        success, evidence = recovery.evaluate(after)
    except ValueError:
        pass
    else:
        assert not success and evidence['code'] != 'PREGRASP_MEASURED_CONVERGED'
    with pytest.raises(ValueError, match='RESPONSE_NOT_PENDING'):
        recovery.evaluate(after)


@pytest.mark.parametrize('fault', ['start', 'goal', 'bound', 'capture'])
def test_invalid_or_large_proposal_is_rejected(fault):
    recovery, nominal, sdk, actual = case()
    if fault == 'start': nominal.joint_trajectory.points[0].positions[0] += Q
    if fault == 'goal': nominal.joint_trajectory.points[-1].positions[2] += .1
    if fault == 'bound': recovery.contract.initial_counts = tuple(np.array(recovery.contract.initial_counts)-32)
    if fault == 'capture': recovery.contract.tolerances = (.006, .00001)
    with pytest.raises(ValueError): recovery.propose(nominal)


def gateway_case(monkeypatch, response=True):
    recovery, nominal, _, _ = case()
    node, planner, plan, state, calls, parameters = make_gateway(
        monkeypatch, response=response, day='20260925_actual')
    plan.model_choice = 'unknown_tabletop'
    context = node._pregrasp_context(plan)
    context['config']['unknown_pregrasp_feedforward_enabled'] = True
    start = RobotState()
    start.joint_state.name = list(ARM_NAMES)+['left_finger','right_finger']
    start.joint_state.position = state['actual'].tolist()+[-.04975,.04975]
    planner.robot = NS(get_current_state=lambda:deepcopy(start))
    planner._plan_cartesian_from_start_state = mock.Mock(return_value=(nominal, 'planned'))
    node._observation_measured_opening = lambda:.04975
    return node, planner, plan, state, calls, parameters, context


@pytest.mark.parametrize('execute', [False, True])
def test_gateway_uses_one_episode_and_never_appends_old_small_steps(monkeypatch, execute):
    node, planner, plan, state, calls, parameters, context = gateway_case(monkeypatch)
    result = node.handle_compensate_pregrasp(NS(plan=plan, execute=execute))
    assert result.success, result.message
    report = json.loads(result.message)
    assert len(calls) == 1 and state['moves'] == int(execute)
    assert report['method'] == 'measured_ik_following_feedforward'
    assert not report['real_grasp_success']
    assert report['code'] == ('PREGRASP_MEASURED_CONVERGED' if execute else 'PREGRASP_PLANNED_ONLY')
    assert planner.observation_path_guard_required is False
    assert planner.observation_tracking_contract_required is False
    if execute:
        assert len(parameters) == 1
        assert not node.handle_compensate_pregrasp(NS(plan=plan, execute=True)).success
        assert len(calls) == 1
    else:
        assert not parameters
        node._ensure_trajectory_controllers_started.assert_not_called()


def test_gateway_no_response_is_failure_without_micro_step_fallback(monkeypatch):
    node, planner, plan, state, calls, parameters, context = gateway_case(monkeypatch, False)
    result = node.handle_compensate_pregrasp(NS(plan=plan, execute=True))
    assert not result.success
    assert len(calls) == 1 and state['moves'] == 1
    assert len(parameters) == 1


def test_already_arrived_sample_certifies_same_goal_without_any_correction(monkeypatch):
    node, planner, plan, state, calls, parameters, context = gateway_case(monkeypatch)
    state['actual'] = state['sdk'].copy()
    result = node.handle_compensate_pregrasp(NS(plan=plan, execute=True))
    assert result.success, result.message
    evidence = json.loads(result.message)
    assert evidence['code'] == 'PREGRASP_MEASURED_CONVERGED'
    assert evidence['fixed_goal_pose'] == list(pose_values(plan.poses[0]))
    assert evidence['plan_id'] == plan.plan_id and evidence['steps_completed'] == 0
    assert not calls and not parameters


def test_gateway_changed_feedback_before_submission_sends_nothing(monkeypatch):
    node, planner, plan, state, calls, parameters, context = gateway_case(monkeypatch)
    original = planner._plan_cartesian_from_start_state
    def planning(*args):
        state['actual'][0] += 3*Q
        return original(*args)
    planner._plan_cartesian_from_start_state = planning
    result = node.handle_compensate_pregrasp(NS(plan=plan, execute=True))
    assert not result.success and 'START_CHANGED' in result.message
    assert state['moves'] == 0 and not parameters


@pytest.mark.parametrize('fault', ['none', 'contact', 'known', 'disabled', 'overshoot', 'legacy'])
def test_complete_correction_keeps_scope_and_original_continuous_box(monkeypatch, fault):
    import time
    from test_gateway_tracking_probe import path_fixture
    from alicia_flexible_grasp.robot.endpoint_correction import CorrectionStep
    node, planner, scene, trajectory = path_fixture(monkeypatch)
    target = np.array([0, 16, 16, 0, 0, 0])
    trajectory.joint_trajectory.points[-1].positions = (target*Q).tolist()
    step = CorrectionStep((2048,)*6, tuple((target+2048).tolist()), (2048,)*6, 1, 1)
    node._validate_pregrasp_context = mock.Mock()
    node._pregrasp_reference = lambda:(2048,)*6
    node._validate_frozen_observation_path = mock.Mock(return_value={
        'controller_constraints_snapshot':{'goal_time':.5}})
    context = dict(scene=scene,revocation=1,deadline=time.monotonic()+60.,
        initial_sdk_counts=(2048,)*6,pose_index=0,plan=NS(model_choice='unknown_tabletop'),
        config={'unknown_pregrasp_feedforward_enabled':True})
    if fault == 'contact': context['pose_index'] = 1
    if fault == 'known': context['plan'].model_choice = 'known'
    if fault == 'disabled': context['config']['unknown_pregrasp_feedforward_enabled'] = False
    if fault == 'overshoot': trajectory.joint_trajectory.points[-1].velocities[1] = 1.
    if fault == 'none':
        report = node._validate_pregrasp_step_path(context,step,trajectory,planner,feedforward=True)
        assert report['compensation_method'] == 'measured_ik_following_feedforward'
        node._validate_frozen_observation_path.assert_called_once()
    else:
        with pytest.raises(ValueError):
            node._validate_pregrasp_step_path(context,step,trajectory,planner,feedforward=fault!='legacy')


def test_readonly_recorded_curve_passes_full_continuous_tracking_contract():
    from alicia_flexible_grasp_supervisor.msg import Grasp6DPlan
    from alicia_flexible_grasp.robot.observation_path_guard import FrozenObservationScene
    from alicia_flexible_grasp.robot.observation_tracking_contract import qualify_contract_path
    from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry
    base = Path(__file__).parent/'fixtures'
    record = json.loads((base/'pregrasp_following_20260925_actual.json').read_text())
    data = json.loads((base/'pregrasp_feedforward_20260925.json').read_text())
    plan = Grasp6DPlan(); fill_message_args(plan, [record['frozen_plan']])
    tr = RobotTrajectory(); fill_message_args(tr, [data['conditional_command_curve']])
    cfg = data['gripper_geometry']
    gripper = GripperGeometry(cfg['max_inner_gap_m'], cfg['width_safety_margin_per_side_m'],
        np.array(cfg['finger_box_xyz_m']),np.array(cfg['palm_box_xyz_m']),cfg.get('support_clearance_m',.003))
    recovery, *_ = case()
    report = qualify_contract_path(tr,tr.joint_trajectory.points[0],
        FrozenObservationScene.from_contact_plan(plan),recovery.contract.fk,gripper,.04975,
        data['controller_constraints'],data['stop_trajectory_duration_sec'],trajectory_stop_reserve=True)
    assert report['ok'], report
    assert max(report['command_derivative_bounds']['maximum_velocity_rad_s']) <= .003
    assert not report['execution_tracking_contract']['certifies_hardware_tracking_stopping_or_calibration']


def gen6_held_axes_case():
    """Actual gen6 start/goal; later response remains an explicit hypothesis.

    Source: 20260926T024831.329194Z_5b2ed413acb441daa6f01b68421ab37e,
    plan cbb323e6c8a4e411de5b93e8, logged proposal at 1790391068.339.
    The identical robot-description SHA is kept by the existing real fixture.
    """
    from geometry_msgs.msg import Pose
    fk, _, _, _ = recorded_fixture('20260925_actual')
    assert fk.model_sha256 == '890592a24b8ad818d995e69beb670ea4254bf84f589615bfac3c8562f62449b7'
    sdk = (np.array([779,1628,2883,1980,1215,2127])-2048)*Q
    actual = (np.array([784,1613,2870,1984,1217,2130])-2048)*Q
    values = [-0.13034992846880536,-0.3811181570750176,0.11762373776483867,
              0.7950302939674306,0.6001639313758793,-0.03549493499799694,0.08043753314838745]
    goal = Pose()
    goal.position.x,goal.position.y,goal.position.z = values[:3]
    goal.orientation.x,goal.orientation.y,goal.orientation.z,goal.orientation.w = values[3:]
    recovery = PregraspFeedforward(fk, sample(fk, sdk, actual), goal, position_tolerance_m=.010)
    return recovery, sdk, actual


def test_gen6_full_guide_preserves_weak_axes_and_requires_real_measured_arrival():
    recovery, sdk, actual = gen6_held_axes_case()
    fixed = pose_values(recovery.goal)
    step, report = recovery.propose_held_axes()
    delta = np.array(step.target_counts)-step.baseline_counts
    assert delta.tolist() == [-4,0,28,-2,0,-32]
    assert report['method'] == 'held_axes_following_feedforward'
    assert report['proposal_policy'] == 'held_axes_full_budget_guide'
    assert report['held_joints'] == ['Joint2','Joint5']
    assert report['position_error_m'] == pytest.approx(.015162695123072598)
    assert report['predicted_position_error_m'] == pytest.approx(.005367069596768243)
    assert report['predicted_orientation_error_rad'] == pytest.approx(.04231349276557822)
    assert not report['conditional_budget_guide']['physical_convergence_proven']
    assert not report['conditional_budget_guide']['path_authorized']
    assert not report['hardware_response_verified']
    recovery.committed(step, 12.)
    # Conditional constant-offset response only, never recorded grasp success.
    success, measured = recovery.evaluate(sample(recovery.contract.fk, step.positions,
                                                actual+delta*Q, 14.))
    assert success and measured['position_error_m'] <= .010
    assert not measured['real_grasp_success']
    assert pose_values(recovery.goal) == fixed
    with pytest.raises(ValueError, match='ALREADY_PROPOSED'):
        recovery.propose_held_axes()


@pytest.mark.parametrize('fault', ['stall', 'held_axis_moved', 'reverse', 'stale'])
def test_held_axes_conditional_prediction_never_replaces_actual_response(fault):
    recovery, sdk, actual = gen6_held_axes_case()
    step, _ = recovery.propose_held_axes()
    recovery.committed(step, 12.)
    response = np.array(step.target_counts)-step.baseline_counts
    if fault == 'stall': response[:] = 0
    elif fault == 'held_axis_moved': response[1] = 3
    elif fault == 'reverse': response[0] = 2
    after = sample(recovery.contract.fk, step.positions, actual+response*Q, 14.)
    if fault == 'stale': after['stationary_window_start_ns'] = 11_999_999_999
    try:
        success, report = recovery.evaluate(after)
    except ValueError:
        pass
    else:
        assert not success
        assert report['code'] != 'PREGRASP_MEASURED_CONVERGED'
    with pytest.raises(ValueError, match='RESPONSE_NOT_PENDING'):
        recovery.evaluate(after)


@pytest.mark.parametrize('fault', ['held_joint2', 'held_joint5', 'count_budget', 'no_progress', 'missed_goal'])
def test_invalid_full_guide_does_not_open_a_fallback_to_weak_axes(monkeypatch, fault):
    recovery, _, _ = gen6_held_axes_case()
    guide = np.array([-4,0,28,-2,0,-32])
    if fault == 'held_joint2': guide[1] = 1
    elif fault == 'held_joint5': guide[4] = 1
    elif fault == 'count_budget': guide[2] = 33
    elif fault == 'no_progress': guide[:] = 0
    elif fault == 'missed_goal': guide = np.array([-4,0,4,-2,0,-4])
    monkeypatch.setattr(recovery.contract, '_remaining_budget_guide', lambda _actual:(guide,0.))
    with pytest.raises(ValueError, match='HELD_AXES_GUIDE'):
        recovery.propose_held_axes()
    assert recovery.step is None


def held_gateway_case(monkeypatch, response=True):
    from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id
    recovery, sdk, actual = gen6_held_axes_case()
    node, planner, plan, state, calls, params = make_gateway(monkeypatch,
        response=response, day='20260925_actual')
    plan.model_choice = 'unknown_tabletop'
    plan.poses[0] = deepcopy(recovery.goal)
    plan.plan_id = compute_plan_id(plan)
    state['sdk'],state['actual'] = sdk.copy(),actual.copy()
    context = node._pregrasp_context(plan)
    context['scene'].plan_id = plan.plan_id
    context['config'].update(unknown_measured_endpoint_position_tolerance_m=.010,
        unknown_pregrasp_feedforward_enabled=True, unknown_pregrasp_held_axes_feedforward_enabled=True)
    planner._plan_cartesian_from_start_state = mock.Mock(side_effect=AssertionError('unconstrained IK fallback'))
    return node,planner,plan,state,calls,params,context


@pytest.mark.parametrize('execute', [False,True])
def test_held_gateway_uses_single_existing_strict_joint_path_without_unconstrained_ik(monkeypatch, execute):
    node,planner,plan,state,calls,params,context = held_gateway_case(monkeypatch)
    fixed = pose_values(plan.poses[0])
    result = node.handle_compensate_pregrasp(NS(plan=plan,execute=execute))
    assert result.success, result.message
    report = json.loads(result.message)
    assert report['method'] == 'held_axes_following_feedforward'
    assert report['proposal_policy'] == 'held_axes_full_budget_guide'
    assert report['proposed_delta_counts'] == [-4,0,28,-2,0,-32]
    assert context['feedforward_method'] == report['method']
    assert all(value['method'] == report['method'] for value in params.values())
    assert len(calls) == 1 and state['moves'] == int(execute)
    planner._plan_cartesian_from_start_state.assert_not_called()
    assert pose_values(plan.poses[0]) == fixed
    assert planner.strict_execution_max_joint_velocity_rad_s == .12
    assert not planner.observation_path_guard_required
    assert not planner.observation_tracking_contract_required
    if execute:
        assert len(params) == 1
        again = node.handle_compensate_pregrasp(NS(plan=plan,execute=True))
        assert not again.success and len(calls) == 1
    else:
        assert not params
        node._ensure_trajectory_controllers_started.assert_not_called()


def test_held_gateway_nonresponse_does_not_integrate_or_fallback(monkeypatch):
    node,planner,plan,state,calls,params,context = held_gateway_case(monkeypatch,False)
    result = node.handle_compensate_pregrasp(NS(plan=plan,execute=True))
    assert not result.success
    assert len(calls) == 1 and state['moves'] == 1 and len(params) == 1
    planner._plan_cartesian_from_start_state.assert_not_called()
    assert not node.handle_compensate_pregrasp(NS(plan=plan,execute=True)).success
    assert len(calls) == 1


def test_held_gateway_failed_guide_sends_no_path_or_unconstrained_fallback(monkeypatch):
    node,planner,plan,state,calls,params,context = held_gateway_case(monkeypatch)
    monkeypatch.setattr(PregraspFeedforward,'propose_held_axes',mock.Mock(
        side_effect=ValueError('PREGRASP_HELD_AXES_GUIDE_DOES_NOT_REACH_FIXED_GOAL')))
    result = node.handle_compensate_pregrasp(NS(plan=plan,execute=True))
    assert not result.success and 'HELD_AXES_GUIDE' in result.message
    assert not calls and not params
    planner._plan_cartesian_from_start_state.assert_not_called()
    node._ensure_trajectory_controllers_started.assert_not_called()


def test_held_gateway_rejects_malformed_switch_without_silent_legacy_fallback(monkeypatch):
    node,planner,plan,state,calls,params,context = held_gateway_case(monkeypatch)
    context['config']['unknown_pregrasp_held_axes_feedforward_enabled'] = 'true'
    result = node.handle_compensate_pregrasp(NS(plan=plan,execute=True))
    assert not result.success and 'HELD_AXES_CONFIG_INVALID' in result.message
    assert not calls
    planner._plan_cartesian_from_start_state.assert_not_called()


def test_september28_real_response_identifies_joint5_drift_without_relaxing_gate():
    from geometry_msgs.msg import Pose
    row=json.loads((Path(__file__).parent/'fixtures/pregrasp_feedforward_20260928_actual.json').read_text())['evidence']
    fk,*_=recorded_fixture('20260925_actual')
    assert fk.model_sha256==row['model_sha256']
    sdk=np.array(row['command_start_positions_rad']);actual=np.array(row['measured_start_positions_rad'])
    goal=Pose();goal.position.x,goal.position.y,goal.position.z=row['fixed_goal_pose'][:3]
    goal.orientation.x,goal.orientation.y,goal.orientation.z,goal.orientation.w=row['fixed_goal_pose'][3:]
    recovery=PregraspFeedforward(fk,sample(fk,sdk,actual),goal,position_tolerance_m=.010)
    step,proposal=recovery.propose_held_axes()
    assert proposal['proposed_delta_counts']==row['proposed_delta_counts']
    recovery.committed(step,12.)
    # Commands and encoders are measured values; only test timestamps are rebased.
    ok,evidence=recovery.evaluate(sample(fk,(np.array(row['sdk_counts'])-2048)*Q,
        (np.array(row['measured_counts'])-2048)*Q,14.))
    assert not ok
    assert evidence['code']=='PREGRASP_NO_BOUNDED_DIRECTIONAL_RESPONSE'
    assert evidence['position_error_m']==pytest.approx(row['position_error_m'])
    assert evidence['last_step_response']['uncommanded_motion_joints']==['Joint5']
    assert evidence['last_step_response']['reversed_response_joints']==[]
    assert evidence['last_step_response']['overshoot_joints']==[]
    assert evidence['last_step_response']['uncommanded_motion_limit_counts']==2
    assert not evidence['hardware_response_verified']
    assert not evidence['real_grasp_success']
