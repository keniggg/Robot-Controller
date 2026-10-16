"""Sealed GEN20 endpoint counterexample and read-only planning boundaries."""
from copy import deepcopy
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS
from unittest import mock

import numpy as np
import pytest
import rospy
from moveit_msgs.msg import RobotTrajectory, RobotState
from trajectory_msgs.msg import JointTrajectoryPoint

from alicia_flexible_grasp.robot import held_pregrasp_selection as module
from alicia_flexible_grasp.robot import contact_sequence_precheck as precheck
from alicia_flexible_grasp.robot.held_axes_prediction import HeldAxesPredictionError
from alicia_flexible_grasp.robot.stationary_following import SerialUrdfFk, ARM_NAMES, SDK_QUANTUM_RAD as Q


def recorded_case(index=0):
    data = json.loads((Path(__file__).parent/'fixtures/held_pregrasp_gen20.json').read_text())
    work = Path(__file__).resolve().parents[3]
    fk = SerialUrdfFk((work/data['model_path']).read_text(), ARM_NAMES)
    assert fk.model_sha256 == data['model_sha256']
    return fk, deepcopy(data['cases'][index])


def select(fk, case, **kwargs):
    return module.select_held_pregrasp_endpoint(fk, case['sample'], *case['poses'][:2],
        case['nominal_command'], position_tolerance_m=.010,
        orientation_tolerance_rad=math.radians(5), **kwargs)


@pytest.mark.parametrize('index', range(3))
def test_recorded_failure_has_quantized_joint_start_inside_unchanged_pose_gates(index):
    fk, case = recorded_case(index)
    before = deepcopy(case)
    old = case['original']['predicted_approach']
    with pytest.raises(HeldAxesPredictionError, match='DOES_NOT_REACH_FIXED_GOAL'):
        module.predict_held_axes_endpoint(fk,
            case['original']['predicted_pregrasp']['predicted_command_positions_rad'],
            old['virtual_start_positions_rad'], old['fixed_goal_pose'],
            position_tolerance_m=.006, orientation_tolerance_rad=math.radians(5),
            allow_already_converged=True)
    command, report = select(fk, case)
    assert report['reselected'] is True
    for name, tolerance in [('predicted_pregrasp', .010), ('predicted_approach', .006)]:
        result = report[name]
        assert result['predicted_position_error_m'] <= tolerance
        assert result['predicted_orientation_error_rad'] <= math.radians(5)
        assert max(abs(np.asarray(result['proposed_delta_counts']))) <= 32
        assert result['proposed_delta_counts'][1] == result['proposed_delta_counts'][4] == 0
    np.testing.assert_allclose(command/Q, np.rint(command/Q), atol=1e-12)
    assert report['execution_authorized'] is False and report['trajectory_checked'] is False
    assert report['future_following_verified'] is False
    assert case == before


def test_an_already_qualified_original_endpoint_never_enters_optimizer(monkeypatch):
    fk, case = recorded_case()
    case['poses'][1] = deepcopy(case['poses'][0])
    monkeypatch.setattr(module, 'minimize', mock.Mock(side_effect=AssertionError('unnecessary search')))
    command, report = select(fk, case)
    assert report['reselected'] is False
    np.testing.assert_array_equal(command, case['nominal_command'])
    module.minimize.assert_not_called()


@pytest.mark.parametrize('fault', ['model', 'joint_names', 'following', 'nonfinite'])
def test_invalid_source_cannot_enter_optimization(monkeypatch, fault):
    fk, case = recorded_case()
    if fault == 'model': case['sample']['model_sha256'] = 'other'
    if fault == 'joint_names': case['sample']['joint_names'] = list(reversed(ARM_NAMES))
    if fault == 'following': case['sample']['sdk_positions_rad'][0] += .1
    if fault == 'nonfinite': case['sample']['accepted_positions_rad'][0] = float('nan')
    monkeypatch.setattr(module, 'minimize', mock.Mock(side_effect=AssertionError('invalid input search')))
    with pytest.raises(ValueError): select(fk, case)
    module.minimize.assert_not_called()


@pytest.mark.parametrize('where', ['objective', 'constraints', 'post_quantization'])
def test_cancellation_or_deadline_is_not_swallowed_as_a_failed_solver_trial(monkeypatch, where):
    fk, case = recorded_case()
    state = dict(expired=False)
    class Cancelled(ValueError): pass
    def deadline():
        if state['expired']: raise Cancelled('deadline / ownership cancelled')
    def optimizer(fun, seed, **kwargs):
        if where == 'objective':
            state['expired'] = True; fun(seed)
        elif where == 'constraints':
            state['expired'] = True; kwargs['constraints'][0]['fun'](seed)
        else:
            state['expired'] = True
        return NS(success=False, message='synthetic solver return', nit=1, x=seed)
    monkeypatch.setattr(module, 'minimize', optimizer)
    with pytest.raises(Cancelled, match='deadline / ownership'):
        select(fk, case, deadline_check=deadline)


def test_near_joint_limit_probe_is_infeasible_without_aborting_search(monkeypatch):
    fk, case = recorded_case()
    def optimizer(fun, seed, **kwargs):
        unit = kwargs['bounds'][6][1]/32  # fallback expresses counts in radians
        probe = seed.copy(); probe[0] = fk.limits[0,1]; probe[6] = 32*unit
        assert fun(probe) == pytest.approx(1.e6 if unit == 1. else 1.e6/.006**2)
        assert min(kwargs['constraints'][0]['fun'](probe)) < 0
        return NS(success=False, message='no qualified candidate', nit=1, x=seed)
    monkeypatch.setattr(module, 'minimize', optimizer)
    with pytest.raises(HeldAxesPredictionError, match='DOES_NOT_REACH_FIXED_GOAL'):
        select(fk, case)


def test_exact_joint_sequence_planner_preserves_goal_and_never_executes():
    from test_observation_branch_hint import fixture
    planner, arm, _, _ = fixture()
    goal = [.006]*6
    path, reason = planner._plan_joint_from_start_state(planner.robot.get_current_state(), goal)
    assert path is not None, reason
    assert arm.joint_targets[-1] == dict(zip(ARM_NAMES, goal))
    assert arm.pose_targets == [] and not arm.executed_plans
    assert arm.goal_joint_tolerance == .0001 and planner._last_pose_plan is None
    arm.wrong_endpoint = True
    path, reason = planner._plan_joint_from_start_state(planner.robot.get_current_state(), goal)
    assert path is None and 'GOAL_CHANGED' in reason
    assert arm.goal_joint_tolerance == .0001 and planner._last_pose_plan is None


@pytest.mark.parametrize('fault', ['none', 'predicted_collision', 'command_start', 'deadline'])
def test_selected_pregrasp_checks_both_whole_curves_before_authority(monkeypatch, fault):
    fk, case = recorded_case()
    sample = case['sample']; path = RobotTrajectory()
    path.joint_trajectory.joint_names = list(ARM_NAMES)
    for i, q in enumerate([sample['sdk_positions_rad'], case['nominal_command']]):
        path.joint_trajectory.points.append(JointTrajectoryPoint(positions=list(q),
            velocities=[0.]*6, accelerations=[0.]*6, time_from_start=rospy.Duration(i*3.)))
    original = deepcopy(path)
    state = RobotState(); state.joint_state.name = list(ARM_NAMES)+['right_finger']
    state.joint_state.position = sample['sdk_positions_rad']+[.04975]
    geometry = []
    def audit(curve, *args, **kwargs):
        geometry.append(deepcopy(curve))
        if fault == 'predicted_collision' and len(geometry) == 2:
            raise ValueError('predicted real pregrasp collision')
        return dict(complete_curve=True)
    monkeypatch.setattr(precheck, 'validate_observation_trajectory', audit)
    collision = mock.Mock(return_value=7)
    planner = NS(_retime_cartesian_plan=mock.Mock(side_effect=AssertionError('must preserve timed curve')))
    def remaining():
        if fault == 'deadline': raise ValueError('deadline')
        return 5.
    if fault == 'command_start': path.joint_trajectory.points[0].positions[0] += Q
    call = lambda: precheck.validate_selected_pregrasp_path(path, fk, sample, None,
        None, .04975, planner, state, collision, remaining)
    if fault != 'none':
        with pytest.raises(ValueError): call()
        return
    evidence = call()
    assert set(evidence) >= {'command', 'predicted_actual'}
    assert collision.call_count == 2 and len(geometry) == 2
    np.testing.assert_allclose(geometry[1].joint_trajectory.points[0].positions,
                               sample['accepted_positions_rad'], atol=1e-14)
    assert path == original and evidence['execution_authorized'] is False
    planner._retime_cartesian_plan.assert_not_called()


def test_final_timed_nonzero_derivatives_are_audited_without_spline_substitution(monkeypatch):
    fk, case = recorded_case()
    sample = case['sample']; path = RobotTrajectory()
    path.joint_trajectory.joint_names = list(ARM_NAMES)
    for i,q in enumerate([sample['sdk_positions_rad'], case['nominal_command']]):
        path.joint_trajectory.points.append(JointTrajectoryPoint(positions=list(q),
            velocities=[.012]*6, accelerations=[-.003]*6, time_from_start=rospy.Duration(i*3.)))
    planner = NS(_retime_cartesian_plan=mock.Mock(side_effect=AssertionError('different spline')))
    def audit(curve, *args, **kwargs):
        assert [p.time_from_start for p in curve.joint_trajectory.points] == [rospy.Duration(0), rospy.Duration(3)]
        assert all(list(p.velocities) == [.012]*6 and list(p.accelerations) == [-.003]*6
                   for p in curve.joint_trajectory.points)
        raise ValueError('the actual timed spline has a collision')
    monkeypatch.setattr(precheck, 'validate_observation_trajectory', audit)
    with pytest.raises(ValueError, match='actual timed spline'):
        precheck.validate_selected_pregrasp_path(path, fk, sample, None, None, .04975,
            planner, RobotState(), mock.Mock(), lambda: 5.)
    planner._retime_cartesian_plan.assert_not_called()


@pytest.mark.parametrize('fault', ['none', 'held_target', 'held_spline', 'budget_spline',
                                  'endpoint', 'time'])
def test_future_held_guide_whole_spline_keeps_original_budget_and_fixed_cells(fault):
    path = RobotTrajectory(); path.joint_trajectory.joint_names = list(ARM_NAMES)
    initial, target = np.zeros(6), np.array([10, 0, -20, 10, 0, -32])*Q
    for i,q in enumerate([initial, target]):
        path.joint_trajectory.points.append(JointTrajectoryPoint(positions=q.tolist(),
            velocities=[0.]*6, accelerations=[0.]*6, time_from_start=rospy.Duration(i*3.)))
    if fault == 'held_target':
        target[1] = Q; path.joint_trajectory.points[-1].positions[1] = Q
    if fault == 'held_spline': path.joint_trajectory.points[-1].velocities[1] = .1
    if fault == 'budget_spline': path.joint_trajectory.points[-1].velocities[0] = 2.
    if fault == 'endpoint': path.joint_trajectory.points[-1].positions[0] += Q
    if fault == 'time': path.joint_trajectory.points[-1].time_from_start = rospy.Duration(0)
    if fault == 'none':
        report = precheck.validate_held_guide_path_bounds(path, initial, target)
        assert report['complete_spline_checked'] and not report['execution_authorized']
    else:
        with pytest.raises(ValueError): precheck.validate_held_guide_path_bounds(path, initial, target)


@pytest.mark.parametrize('fault', ['none', 'pose0_correction', 'pregrasp_collision', 'held_curve_collision',
                                  'held_curve_moves_fixed_axis', 'joint_plan_changed'])
def test_real_precheck_uses_selected_joint_start_and_audits_future_guide(monkeypatch, fault):
    from test_contact_sequence_precheck import gateway_case
    from test_gen14_held_approach_precheck import config, pose
    from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id
    node, planner, req, params, sample = gateway_case(monkeypatch)
    fk, case = recorded_case()
    params['/robot_description'] = (Path(__file__).resolve().parents[3]/
        'deployment/2026-10-08/runtime/candidate_models/expanded_metric_20260929/'
        'alicia_expanded_metric_candidate.urdf').read_text()
    sample.clear(); sample.update(case['sample']); sample['epoch_ns'] = 1
    params['/grasp'].update(config())
    req.plan.poses = [pose(v) for v in case['poses']]
    if fault == 'pose0_correction': req.plan.poses[1] = deepcopy(req.plan.poses[0])
    req.plan.plan_id = compute_plan_id(req.plan)
    state = planner.robot.get_current_state()
    state.joint_state.position = sample['accepted_positions_rad']+[.04975]
    planner.robot.get_current_state = lambda: deepcopy(state)
    calls = []
    def curve(start, end):
        result = RobotTrajectory(); result.joint_trajectory.joint_names = list(ARM_NAMES)
        for i,q in enumerate([start, end]):
            result.joint_trajectory.points.append(JointTrajectoryPoint(positions=list(q),
                velocities=[0.]*6, accelerations=[0.]*6, time_from_start=rospy.Duration(i*3.)))
        return result
    planner._plan_pose_from_start_state = mock.Mock(return_value=(
        curve(sample['accepted_positions_rad'], case['nominal_command']), 'original seed'))
    def joint_path(start, end, **kwargs):
        begin = dict(zip(start.joint_state.name, start.joint_state.position))
        calls.append((deepcopy(begin), list(end)))
        result = curve([begin[n] for n in ARM_NAMES], end)
        if fault == 'joint_plan_changed' and len(calls) == 1:
            return None, 'exact joint endpoint changed'
        if fault == 'held_curve_moves_fixed_axis' and len(calls) == 2:
            result.joint_trajectory.points[-1].velocities[1] = .1
        return result, 'fresh read-only joint plan'
    planner._plan_joint_from_start_state = joint_path
    planner._plan_cartesian_from_start_state = mock.Mock(side_effect=lambda s,t:
        (curve(s.joint_state.position[:6], s.joint_state.position[:6]), 'nominal no-motion reachability only'))
    checked = []
    def geometry(path, *args, **kwargs):
        checked.append(deepcopy(path))
        if fault == 'pregrasp_collision' and len(checked) == 2:
            raise ValueError('predicted pregrasp geometry rejected')
        if fault == 'held_curve_collision' and len(checked) == 4:
            raise ValueError('predicted held guide geometry rejected')
        return dict(complete_curve=True)
    monkeypatch.setattr(precheck, 'validate_observation_trajectory', geometry)
    contact = mock.Mock(return_value=dict(execution_authorized=False))
    monkeypatch.setattr(precheck, 'validate_contact_prediction', contact)
    def sequence(targets, names, linear, **kw):
        current = deepcopy(state)
        try:
            first, reason = kw['pregrasp_planner'](current, targets[0])
            if first is None: raise ValueError(reason)
            for index,name in enumerate(names):
                if index == 0:
                    path = first
                elif index == 1:
                    path, reason = kw['approach_planner'](current, targets[index])
                    if path is None: raise ValueError(reason)
                else:
                    path = curve(current.joint_state.position[:6], current.joint_state.position[:6])
                kw['stage_validator'](name, current, path, targets[index])
                current.joint_state.position[:6] = path.joint_trajectory.points[-1].positions
                current = kw['stage_state_transform'](name, current)
        except Exception as exc:
            return False, getattr(exc, 'code', 'MOVEIT_CHECK_ERROR'), 'pregrasp', {}, str(exc)
        return True, '', '', dict(path_cost=.5, max_delta=.2), 'offline sequence'
    planner.check_pose_sequence.side_effect = sequence
    result = node.handle_contact_sequence_precheck(req)
    assert result.success is (fault in ('none', 'pose0_correction')), result.message
    audit = json.loads(result.evidence_json)
    if fault in ('none', 'pose0_correction'):
        assert audit['held_pregrasp_selection']['reselected'] is (fault == 'none')
        key = 'guide_path_prediction' if fault == 'none' else 'pose0_correction_path_prediction'
        assert audit['held_approach_prediction'][key]['bounds']['complete_spline_checked']
        assert len(calls) == 2 and len(checked) == 4
        np.testing.assert_allclose([calls[0][0][n] for n in ARM_NAMES], sample['sdk_positions_rad'])
        np.testing.assert_allclose(checked[1].joint_trajectory.points[0].positions, sample['accepted_positions_rad'])
        assert contact.call_count == 1
        actual_approach = audit['held_approach_prediction']['predicted_approach']['predicted_positions_rad']
        np.testing.assert_allclose(contact.call_args[0][8].joint_state.position[:6], actual_approach)
        if fault == 'none': planner._plan_cartesian_from_start_state.assert_not_called()
    else:
        contact.assert_not_called()
        assert not getattr(node, '_contact_precheck_cache', {})
    node._ensure_trajectory_controllers_started.assert_not_called()
    node._synchronize_trajectory_controller_to_feedback.assert_not_called()


def test_real_sequence_advances_from_predicted_actual_after_held_command_path():
    from test_moveit_planner_pose_feedback import (
        MoveItPlannerPoseFeedbackTest, FakeManipulator, JointPlan, make_pose)
    fixture = MoveItPlannerPoseFeedbackTest(); planner = fixture.make_planner(FakeManipulator())
    planner.robot = NS(get_current_state=lambda: fixture.robot_state(.1, .1))
    seen = []
    def pre(state,target):
        return JointPlan([[.1,.1],[.3,.3]]), 'nominal command'
    def approach(state,target):
        seen.append(('held_start', list(state.joint_state.position)))
        return JointPlan([[.3,.3],[.4,.4]]), 'exact held command'
    def actual(stage,state):
        if stage in ('pregrasp','approach'):
            state.joint_state.position = [v-.01 for v in state.joint_state.position]
        return state
    def contact(state,target):
        seen.append(('contact_start', list(state.joint_state.position)))
        return JointPlan([[.39,.39],[.5,.5]]), 'nominal contact from predicted actual'
    planner._plan_cartesian_from_start_state = contact
    planner._plan_pose_from_start_state = mock.Mock(side_effect=AssertionError('ordinary pose bypass'))
    result = planner.check_pose_sequence([make_pose()]*3, ['pregrasp','approach','grasp'],
        [False,True,True], pregrasp_planner=pre, approach_planner=approach,
        stage_state_transform=actual)
    assert result[0], result
    np.testing.assert_allclose(seen[0][1], [.29,.29])
    np.testing.assert_allclose(seen[1][1], [.39,.39])
    assert planner._last_pose_plan is None
