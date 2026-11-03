"""Recorded September 27 failure and an explicitly hypothetical alternative.

All inputs are local fixtures. No ROS initialization, service, publication, or
MoveIt execution is used. A conditional CAD certificate is not hardware or
full-robot collision evidence.
"""
from copy import deepcopy
import json
from pathlib import Path

import numpy as np
import pytest
from genpy.message import fill_message_args
from moveit_msgs.msg import RobotTrajectory
from rospy import Duration
from trajectory_msgs.msg import JointTrajectoryPoint

from alicia_flexible_grasp_supervisor.msg import Grasp6DPlan
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry
from alicia_flexible_grasp.grasp.rich_plan_integrity import pose_values
from alicia_flexible_grasp.robot.observation_path_guard import (
    FrozenObservationScene, SerialUrdfFk, segment_coefficients,
    _polynomial_bounds,
)
from alicia_flexible_grasp.robot.observation_tracking_contract import qualify_contract_path
from alicia_flexible_grasp.robot.pregrasp_feedforward import PregraspFeedforward
from alicia_flexible_grasp.robot.stationary_following import (
    ARM_NAMES, SDK_QUANTUM_RAD as Q, stationary_following_error,
)
from test_stationary_following import message


FIXTURES = Path(__file__).parent / 'fixtures'


def recorded_case():
    record = json.loads((FIXTURES / 'held_approach_counterfactual_20260927.json').read_text())
    failure = json.loads((FIXTURES / record['source_fixture']).read_text())
    xml = json.loads((FIXTURES / record['robot_description_fixture']).read_text())['robot_description']
    fk = SerialUrdfFk(xml, ARM_NAMES)
    assert fk.model_sha256 == record['model_sha256'] == failure['model_sha256']
    plan = Grasp6DPlan()
    fill_message_args(plan, [record['frozen_plan']])
    assert plan.plan_id == failure['plan_id']
    goal = plan.poses[record['counterfactual']['frozen_goal_pose_index']]
    recovery = PregraspFeedforward(fk, record['before_stationary'], goal,
                                   position_tolerance_m=.010)
    return record, failure, plan, recovery


def synthetic_response(fk, sdk, actual, *, at, epoch):
    """A hypothetical stationary plant response, never recorded telemetry."""
    times = np.linspace(at - .45, at - .05, 5)
    measured = [message(t + .001, 'sdk_measured', actual, True) for t in times]
    for row in measured:
        row.position[-1] = .04975
    return stationary_following_error(
        fk, [message(t, 'sdk_transmitted', sdk) for t in times], measured,
        now_sec=at, epoch_ns=epoch,
    )


def test_recorded_failure_remains_outside_unchanged_following_contract():
    _, failure, plan, recovery = recorded_case()
    after = failure['post_failure_stationary']
    gap = np.array(after['sdk_positions_rad']) - after['accepted_positions_rad']
    assert abs(gap[1]) == pytest.approx(.04908738521234046)
    assert abs(gap[1]) > failure['goal_tolerance_rad'] == .035
    assert failure['observed_response']['joint2_command_delta_counts'] == 15
    assert failure['observed_response']['joint2_accepted_delta_counts'] == 0
    assert after['stationary_window_start_ns'] > failure['action_result_stamp_ns']
    with pytest.raises(ValueError, match='PREGRASP_FOLLOWING_OUTSIDE_CONTRACT'):
        PregraspFeedforward(recovery.contract.fk, after, plan.poses[1],
                            position_tolerance_m=.010)


def test_recorded_start_can_propose_original_budget_with_joint2_and_joint5_held():
    record, _, _, recovery = recorded_case()
    frozen = pose_values(recovery.goal)
    initial = deepcopy(record['before_stationary'])
    step, report = recovery.propose_held_axes()
    delta = np.array(step.target_counts) - step.baseline_counts
    assert delta.tolist() == record['counterfactual']['delta_counts'] == [-4, 0, -18, 0, 0, 32]
    assert max(abs(delta)) <= 32
    assert delta[[1, 4]].tolist() == [0, 0]
    assert report['held_joints'] == ['Joint2', 'Joint5']
    assert report['predicted_position_error_m'] == pytest.approx(
        record['counterfactual']['predicted_position_error_m'])
    assert report['predicted_orientation_error_rad'] == pytest.approx(
        record['counterfactual']['predicted_orientation_error_rad'])
    assert report['predicted_position_error_m'] < .001
    assert not report['conditional_budget_guide']['physical_convergence_proven']
    assert not report['conditional_budget_guide']['path_authorized']
    assert not report['hardware_response_verified']
    assert not report['real_grasp_success']
    assert pose_values(recovery.goal) == frozen
    assert record['before_stationary'] == initial


def test_only_hypothesized_matching_response_can_converge_for_alternative():
    record, _, _, recovery = recorded_case()
    step, _ = recovery.propose_held_axes()
    initial = record['before_stationary']
    delta = np.array(step.target_counts) - step.baseline_counts
    actual = np.array(initial['accepted_positions_rad'])
    completed = initial['stamp_sec'] + record['counterfactual']['duration_sec']
    recovery.committed(step, completed)
    # This constant-bias response has NOT occurred on hardware.
    hypothetical = synthetic_response(
        recovery.contract.fk, step.positions, actual + delta * Q,
        at=completed + 1., epoch=initial['epoch_ns'])
    success, report = recovery.evaluate(hypothetical)
    assert success and report['code'] == 'PREGRASP_MEASURED_CONVERGED'
    assert report['position_error_m'] < .001
    assert not report['real_grasp_success']


def test_original_failed_motion_cannot_be_relabelled_as_alternative_response():
    record, failure, _, recovery = recorded_case()
    step, _ = recovery.propose_held_axes()
    recovery.committed(step, record['before_stationary']['stamp_sec'] + 47.)
    # Preserve the real timestamps and different SDK target. This evidence
    # cannot acknowledge a hypothetical command, even with a small FK error.
    with pytest.raises(ValueError, match='PREGRASP_SDK_TARGET_CHANGED'):
        recovery.evaluate(failure['post_failure_stationary'])


def test_counterfactual_47sec_curve_preserves_continuous_box_and_original_clearance():
    record, _, plan, recovery = recorded_case()
    step, _ = recovery.propose_held_axes()
    initial = np.array(record['before_stationary']['sdk_positions_rad'])
    duration = record['counterfactual']['duration_sec']
    curve = RobotTrajectory()
    curve.joint_trajectory.joint_names = list(ARM_NAMES)
    for positions, at in ((initial, 0.), (step.positions, duration)):
        curve.joint_trajectory.points.append(JointTrajectoryPoint(
            positions=list(positions), velocities=[0.] * 6,
            accelerations=[0.] * 6, time_from_start=Duration.from_sec(at)))
    hold = JointTrajectoryPoint()
    fill_message_args(hold, [record['controller_hold']])
    low, high = _polynomial_bounds(
        segment_coefficients(hold, curve.joint_trajectory.points[-1], duration, 6), 0., 1.)
    assert np.all(low >= initial - 32 * Q - 1e-12)
    assert np.all(high <= initial + 32 * Q + 1e-12)
    assert np.all(low[[1, 4]] >= initial[[1, 4]] - Q / 2)
    assert np.all(high[[1, 4]] < initial[[1, 4]] + Q / 2)
    cfg = record['gripper_geometry']
    gripper = GripperGeometry(
        cfg['max_inner_gap_m'], cfg['width_safety_margin_per_side_m'],
        np.array(cfg['finger_box_xyz_m']), np.array(cfg['palm_box_xyz_m']),
        cfg['support_clearance_m'])
    scene = FrozenObservationScene.from_contact_plan(plan)
    report = qualify_contract_path(
        curve, hold, scene, recovery.contract.fk, gripper, record['opening_m'],
        record['controller_constraints'], record['stop_trajectory_duration_sec'],
        trajectory_stop_reserve=True)
    assert report['ok'], report
    derivatives = report['command_derivative_bounds']
    assert max(derivatives['maximum_velocity_rad_s']) <= .002
    assert max(derivatives['maximum_acceleration_rad_s2']) <= .30
    contract = report['execution_tracking_contract']
    assert contract['path_position_tolerance_rad'] == [.035] * 6
    support = report['continuous_path']['following_support']
    assert support['includes_controller_bridge']
    assert support['minimum_support_clearance_lower_bound_m'] >= .003
    assert duration + record['controller_constraints']['goal_time'] + 2. < 60.
    # The real executor must still check its remaining deadline and fresh
    # scene/reference, perform full state-validity checks, and measure motion.
    assert not contract['certifies_hardware_tracking_stopping_or_calibration']
    assert not support['target_obb_tracking_uncertainty_checked']
    assert not record['limitations']['moveit_state_validity_checked']
    assert not record['limitations']['full_robot_collision_checked']
    assert not record['limitations']['physical_path_executed']
    assert not record['limitations']['hardware_response_verified']
