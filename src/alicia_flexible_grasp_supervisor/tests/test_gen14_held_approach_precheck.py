"""Offline real-source regression and conditional planning contract tests."""
from copy import deepcopy
import json
import math
from pathlib import Path
from unittest import mock

import numpy as np
import pytest
from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from tf.transformations import quaternion_from_matrix

from alicia_flexible_grasp.robot import contact_sequence_precheck as module
from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id
from alicia_flexible_grasp.robot.held_axes_prediction import (
    HeldAxesPredictionError, predict_held_axes_endpoint)
from alicia_flexible_grasp.robot.pregrasp_feedforward import PregraspFeedforward
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES, SDK_QUANTUM_RAD as Q, SerialUrdfFk
from test_contact_sequence_precheck import gateway_case


def pose(values):
    result = Pose()
    result.position.x, result.position.y, result.position.z = values[:3]
    result.orientation.x, result.orientation.y, result.orientation.z, result.orientation.w = values[3:]
    return result


def fk_pose(fk, q):
    transform = fk(q)
    return pose(list(transform[:3, 3])+list(quaternion_from_matrix(transform)))


def record():
    data = json.loads((Path(__file__).parent/'fixtures/gen14_held_approach_20261010.json').read_text())
    return data, SerialUrdfFk(data['robot_description'], ARM_NAMES)


def config(correction=True):
    return dict(pregrasp_cartesian_compensation_enabled=correction,
        unknown_pregrasp_feedforward_enabled=True,
        unknown_pregrasp_held_axes_feedforward_enabled=True,
        unknown_approach_held_axes_feedforward_enabled=True,
        unknown_measured_endpoint_position_tolerance_m=.010,
        measured_endpoint_orientation_tolerance_deg=5.)


def path(q):
    result = RobotTrajectory()
    result.joint_trajectory.joint_names = list(ARM_NAMES)
    result.joint_trajectory.points = [JointTrajectoryPoint(positions=list(q)),
                                    JointTrajectoryPoint(positions=list(q))]
    return result


def test_exact_gen14_execution_failure_keeps_initial_and_guide_residuals():
    data, fk = record()
    before = deepcopy(data['sample'])
    recovery = PregraspFeedforward(fk, data['sample'], pose(data['poses'][1]), position_tolerance_m=.006)
    with pytest.raises(HeldAxesPredictionError, match=data['failure']['code']) as caught:
        recovery.propose_held_axes()
    report = caught.value.evidence
    assert report['position_error_m'] == pytest.approx(.013822515443860484, abs=1e-14)
    assert report['predicted_position_error_m'] == pytest.approx(.007664287090261585, abs=1e-14)
    assert report['proposed_delta_counts'] == [-2, 0, -20, 10, 0, 32]
    assert report['predicted_orientation_error_rad'] < math.radians(5)
    assert recovery.step is None and data['sample'] == before
    assert not report['execution_authorized'] and not report['globally_optimal']


def test_gen14_successful_pose0_retains_exact_guide_and_predicted_result():
    data, fk = record()
    old = data['pregrasp_completed']
    target, report = predict_held_axes_endpoint(fk, old['command_start_positions_rad'],
        old['measured_start_positions_rad'], old['fixed_goal_pose'],
        position_tolerance_m=.010, orientation_tolerance_rad=math.radians(5))
    assert report['proposed_delta_counts'] == old['proposed_delta_counts']
    assert target.tolist() == old['sdk_counts']
    assert report['predicted_position_error_m'] == pytest.approx(old['predicted_position_error_m'], abs=1e-14)
    assert report['predicted_orientation_error_rad'] == pytest.approx(old['predicted_orientation_error_rad'], abs=1e-14)


def test_recorded_pre_move_bias_predicts_failed_approach_before_motion():
    data, fk = record()
    # This is the recorded EXECUTED nominal pregrasp endpoint, with the real
    # earlier planning-station bias. It is not an archived preflight path.
    nominal = path(data['executed_trajectories'][1]['points'][-1]['positions'])
    audit = {}
    with pytest.raises(module.ContactPrecheckError, match='held approach prediction:.*DOES_NOT_REACH'):
        module.predict_held_approach(nominal, fk, data['precheck_source_sample'],
            pose(data['poses'][0]), pose(data['poses'][1]), config(), audit)
    assert audit['predicted_pregrasp']['success'] is True
    assert audit['predicted_pregrasp']['position_tolerance_m'] == .010
    assert audit['predicted_approach']['position_tolerance_m'] == .006
    assert audit['predicted_pregrasp']['predicted_position_error_m'] < .005
    assert audit['predicted_approach']['predicted_position_error_m'] == pytest.approx(.007855361023547648)
    np.testing.assert_allclose(audit['predicted_pregrasp']['sdk_minus_encoder_rad'],
                               audit['predicted_approach']['sdk_minus_encoder_rad'], atol=1e-15)
    assert not audit['success'] and not audit['future_following_verified']
    assert 'stamp_sec' not in audit['predicted_approach']


@pytest.mark.parametrize('correction', [False, True])
def test_optional_pose0_correction_and_already_converged_are_only_predictions(correction):
    data, fk = record()
    sample = data['sample']
    q, sdk = sample['accepted_positions_rad'], sample['sdk_positions_rad']
    target = fk_pose(fk, q)
    audit = {}
    module.predict_held_approach(path(sdk), fk, sample, target, target, config(correction), audit)
    assert audit['success'] and audit['predicted_approach']['steps_predicted'] == 0
    assert audit['predicted_approach']['code'] == 'PREGRASP_CONDITIONALLY_CONVERGED'
    assert not audit['hardware_response_verified'] and not audit['execution_authorized']
    if correction:
        assert audit['predicted_pregrasp']['steps_predicted'] == 0
    else:
        assert audit['predicted_pregrasp']['correction_enabled'] is False


@pytest.mark.parametrize('fault', ['other_pose0_policy', 'nonfinite', 'following', 'names'])
def test_incomplete_or_unmodeled_conditional_inputs_never_pass(fault):
    data, fk = record()
    sample, settings = data['sample'], config()
    nominal = path(sample['sdk_positions_rad'])
    if fault == 'other_pose0_policy': settings['unknown_pregrasp_feedforward_enabled'] = False
    if fault == 'nonfinite': sample['accepted_positions_rad'][0] = float('nan')
    if fault == 'following': sample['accepted_positions_rad'][0] += .1
    if fault == 'names': nominal.joint_trajectory.joint_names.reverse()
    with pytest.raises(ValueError):
        module.predict_held_approach(nominal, fk, sample, pose(data['poses'][0]),
                                    pose(data['poses'][1]), settings, {})


def integrated_case(monkeypatch):
    node, planner, req, params, sample = gateway_case(monkeypatch)
    params['/grasp'].update(config(False))
    fk = SerialUrdfFk(params['/robot_description'], ARM_NAMES)
    req.plan.poses[0] = req.plan.poses[1] = fk_pose(fk, sample['accepted_positions_rad'])
    req.plan.plan_id = compute_plan_id(req.plan)
    nominal = path(sample['sdk_positions_rad'])
    contact = mock.Mock(return_value=dict(scope='independent_contact_fixture', execution_authorized=False))
    monkeypatch.setattr(module, 'validate_contact_prediction', contact)
    def sequence(targets, names, linear, **kw):
        for index, stage in enumerate(names):
            try:
                kw['stage_validator'](stage, planner.robot.get_current_state(), nominal, targets[index])
            except Exception as exc:
                return False, getattr(exc, 'code', 'MOVEIT_CHECK_ERROR'), stage, {}, str(exc)
        return True, '', '', dict(path_cost=.5, max_delta=.2), 'sequence checked'
    planner.check_pose_sequence.side_effect = sequence
    return node, planner, req, params, sample, contact


def test_real_gateway_uses_and_caches_conditional_approach_without_motion(monkeypatch):
    node, planner, req, params, sample, contact = integrated_case(monkeypatch)
    first = node.handle_contact_sequence_precheck(req)
    assert first.success, first.message
    first_audit = json.loads(first.evidence_json)
    assert first_audit['held_approach_prediction']['success']
    second = node.handle_contact_sequence_precheck(req)
    assert second.success, second.message
    assert json.loads(second.evidence_json)['prediction_cache_hit']
    assert planner.check_pose_sequence.call_count == 1 and contact.call_count == 1
    node._ensure_trajectory_controllers_started.assert_not_called()
    node._synchronize_trajectory_controller_to_feedback.assert_not_called()


def test_cache_without_actual_policy_prediction_must_replan(monkeypatch):
    node, planner, req, _, _, _ = integrated_case(monkeypatch)
    assert node.handle_contact_sequence_precheck(req).success
    for entry in node._contact_precheck_cache.values(): entry.pop('held_approach_prediction')
    assert node.handle_contact_sequence_precheck(req).success
    assert planner.check_pose_sequence.call_count == 2


def test_nominal_success_without_approach_validator_is_not_enough(monkeypatch):
    node, planner, req, _, _, _ = integrated_case(monkeypatch)
    def missing(targets, names, linear, **kw):
        kw['stage_validator']('grasp', planner.robot.get_current_state(), path([0.]*6), targets[2])
        return True, '', '', dict(path_cost=.5,max_delta=.2), 'nominal only'
    planner.check_pose_sequence.side_effect = missing
    result = node.handle_contact_sequence_precheck(req)
    assert not result.success and 'held approach validator was not executed' in result.message
    assert not getattr(node, '_contact_precheck_cache', {})


def test_failed_approach_is_reported_before_contact_checker_and_not_cached(monkeypatch):
    node, planner, req, _, _, contact = integrated_case(monkeypatch)
    req.plan.poses[1].position.z += .1
    req.plan.plan_id = compute_plan_id(req.plan)
    result = node.handle_contact_sequence_precheck(req)
    assert not result.success and result.failed_stage == 'approach'
    evidence = json.loads(result.evidence_json)
    assert not evidence['held_approach_prediction']['success']
    assert 'OUTSIDE_LOCAL_CAPTURE_RANGE' in result.message
    contact.assert_not_called()
    assert not getattr(node, '_contact_precheck_cache', {})


def test_real_gateway_serializes_guide_failure_without_submitting_motion(monkeypatch):
    from types import SimpleNamespace as NS
    from test_held_approach_gateway import approach_case
    node, planner, plan, state, calls, params, _ = approach_case(monkeypatch)
    evidence = dict(position_error_m=.013822515443860484,
                    predicted_position_error_m=.007664287090261585,
                    proposed_delta_counts=[-2,0,-20,10,0,32], execution_authorized=False)
    monkeypatch.setattr(PregraspFeedforward, 'propose_held_axes', mock.Mock(side_effect=
        HeldAxesPredictionError('PREGRASP_HELD_AXES_GUIDE_DOES_NOT_REACH_FIXED_GOAL', evidence)))
    result = node.handle_compensate_approach(NS(plan=plan, execute=True))
    assert not result.success
    assert json.loads(result.message)['held_axes_prediction'] == evidence
    assert state['moves'] == 0 and not calls and not params
    planner._plan_cartesian_from_start_state.assert_not_called()
