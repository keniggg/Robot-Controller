"""No ROS master or hardware: contact-only tolerance and provenance contracts."""
from copy import deepcopy

import numpy as np
import pytest
import rospy
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from alicia_flexible_grasp.robot.contact_tracking_contract import (
    POLICY, build_contact_tracking_contract, revalidate_contact_tracking_contract,
    validate_contact_tracking_contract_binding,
)
from alicia_flexible_grasp.robot.observation_path_guard import ObservationPathError
from alicia_flexible_grasp.robot.observation_tracking_contract import (
    SDK_HALF_COUNT_RAD, commanded_stop_reserve,
)
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES, SDK_QUANTUM_RAD as Q
from test_stationary_following import fixture


def inputs():
    fk, sdk, accepted = fixture()
    start = np.array(sdk[-1].position)
    delta = np.array([0, 0, 0, 2, 0, 4])*Q
    plan = RobotTrajectory()
    plan.joint_trajectory.joint_names = list(ARM_NAMES)
    plan.joint_trajectory.points = [
        JointTrajectoryPoint(positions=start.tolist(), velocities=[0.]*6,
            accelerations=(2*delta/9).tolist(), time_from_start=rospy.Duration(0)),
        JointTrajectoryPoint(positions=(start+delta).tolist(), velocities=[0.]*6,
            accelerations=(-2*delta/9).tolist(), time_from_start=rospy.Duration(3)),
    ]
    hold = JointTrajectoryPoint(positions=start.tolist(), velocities=[0.]*6, accelerations=[0.]*6)
    constraints = {name: dict(trajectory=.12, goal=.035) for name in ARM_NAMES}
    constraints.update(goal_time=1., stopped_velocity_tolerance=.01)
    return [plan, hold, fk, sdk, accepted, constraints, .5]


def build(args, **kwargs):
    options = dict(now_sec=10.05, epoch_ns=9_000_000_000)
    options.update(kwargs)
    return build_contact_tracking_contract(*args, **options)


def binding(contract, args):
    plan, hold, _, _, _, constraints, stop = args
    return validate_contact_tracking_contract_binding(contract, plan, hold, constraints, stop)


def fresh(contract, args, **kwargs):
    options = dict(now_sec=10.05, epoch_ns=9_000_000_000)
    options.update(kwargs)
    return revalidate_contact_tracking_contract(contract, *args, **options)


def test_per_axis_contract_retains_original_stop_half_count_and_full_quantum_slack():
    args = inputs()
    original = deepcopy((args[0], args[1], args[3], args[4], args[5]))
    contract = build(args)
    assert contract['policy'] == POLICY
    path = np.array(contract['path_position_tolerance_rad'])
    goal = np.array(contract['goal_position_tolerance_rad'])
    assert np.all(path > 0.) and np.all(path <= .035)
    assert np.all(goal > 0.) and np.all(goal <= path)
    entry = np.array(contract['entry_absolute_error_rad'])
    phase = np.array(contract['command_phase_age_reserve_rad'])
    np.testing.assert_array_equal(path, np.maximum(Q, entry+Q+phase))
    assert np.all(np.array(contract['entry_slack_rad']) >= Q)
    stop = commanded_stop_reserve(.5, velocity=contract['maximum_command_velocity_rad_s'],
                                  acceleration=contract['maximum_command_acceleration_rad_s2'])
    np.testing.assert_array_equal(contract['joint_error_bounds_rad'], path+stop+SDK_HALF_COUNT_RAD)
    assert contract['command_phase_reserve_is_servo_motion_bound'] is False
    assert contract['certifies_hardware_tracking_stopping_or_calibration'] is False
    assert contract['continuous_accepted_feedback_monitor_required'] is True
    assert contract['geometry_proof_required'] is True
    assert contract['motion_authorized'] is False
    assert original == (args[0], args[1], args[3], args[4], args[5])
    assert binding(contract, args)
    assert fresh(contract, args)['sdk_stamp_ns'] == contract['entry_feedback']['sdk_stamp_ns']


def test_held_axes_keep_real_bias_and_positive_allowance():
    contract = build(inputs())
    # Joint2 was not commanded, yet its measured persistent error is retained.
    assert contract['entry_absolute_error_rad'][1] == pytest.approx(8*Q)
    assert contract['path_position_tolerance_rad'][1] >= 9*Q
    assert contract['path_position_tolerance_rad'][4] >= Q


def test_goal_tolerance_cannot_revert_to_wider_configured_endpoint_limit():
    args = inputs()
    contract = build(args)
    contract['goal_position_tolerance_rad'][0] = .035
    with pytest.raises(ObservationPathError, match='BOUND_CONTRACT_INVALID'):
        binding(contract, args)


def test_smaller_existing_goal_limit_is_preserved():
    args = inputs()
    args[5]['Joint1']['goal'] = .001
    contract = build(args)
    assert contract['goal_position_tolerance_rad'][0] == .001


@pytest.mark.parametrize('options', [
    dict(pose_index=0), dict(pose_index=1), dict(pose_index=True), dict(pose_index=2.),
    dict(minimum_path_tolerance_rad=0.), dict(minimum_path_tolerance_rad=-1.),
    dict(minimum_path_tolerance_rad=Q/2), dict(minimum_path_tolerance_rad=True),
    dict(minimum_path_tolerance_rad=float('nan')), dict(minimum_path_tolerance_rad='0.003'),
    dict(maximum_age_sec=0.), dict(maximum_age_sec=.5001), dict(maximum_age_sec=True),
    dict(maximum_age_sec=float('nan')), dict(epoch_ns=0), dict(epoch_ns=True),
    dict(now_sec=11.), dict(now_sec=9.5),
])
def test_invalid_scope_or_limits_do_not_create_zero_or_default_tolerance(options):
    with pytest.raises(ObservationPathError):
        build(inputs(), **options)


@pytest.mark.parametrize('mutation', [
    lambda x: x[3].clear(),
    lambda x: x[4].clear(),
    lambda x: x[3].__delitem__(slice(None, -1)),
    lambda x: setattr(x[4][-1].header, 'frame_id', 'heartbeat'),
    lambda x: x[4][-1].name.__setitem__(0, 'Joint2'),
    lambda x: x[4][-1].position.__setitem__(0, float('nan')),
    lambda x: x[3][-1].position.__setitem__(0, Q),
    lambda x: x[4][-1].position.__setitem__(0, 2*Q),
    lambda x: setattr(x[4][-1].header, 'stamp', x[4][-2].header.stamp),
    lambda x: setattr(x[0].joint_trajectory, 'joint_names', list(reversed(ARM_NAMES))),
    lambda x: x[1].velocities.__setitem__(0, .001),
    lambda x: x[1].positions.__setitem__(0, Q),
    lambda x: x[0].joint_trajectory.points[-1].positions.__setitem__(0, 5*Q),
    lambda x: x[0].joint_trajectory.points[-1].positions.__setitem__(0, .5*Q),
    lambda x: x[0].joint_trajectory.points[0].positions.__setitem__(0, Q),
    lambda x: x[5]['Joint1'].__setitem__('trajectory', 0.),
    lambda x: x[5]['Joint2'].__setitem__('trajectory', .005),
    lambda x: x[5]['Joint1'].__setitem__('goal', 0.),
])
def test_incomplete_source_or_changed_microstep_fails_closed(mutation):
    args = inputs()
    mutation(args)
    with pytest.raises(ObservationPathError):
        build(args)


def test_configured_floor_is_not_clipped_to_fit_geometry_or_controller_cap():
    args = inputs()
    contract = build(args, minimum_path_tolerance_rad=.03)
    assert min(contract['path_position_tolerance_rad']) == .03
    with pytest.raises(ObservationPathError, match='SLACK_EXCEEDS'):
        build(args, minimum_path_tolerance_rad=.036)


def test_submission_drift_rejects_without_widening_or_rebasing_contract():
    args = inputs()
    contract = build(args)
    saved = deepcopy(contract)
    for message in args[4]:
        message.position[1] -= Q
    with pytest.raises(ObservationPathError, match='ENVELOPE_OR_ENTRY_CHANGED'):
        fresh(contract, args)
    assert contract == saved


def test_submission_improved_feedback_uses_same_frozen_tolerances():
    args = inputs()
    contract = build(args)
    saved = deepcopy(contract)
    for message in args[4]:
        message.position[1] += Q
    measured = fresh(contract, args)
    assert measured['accepted_positions_rad'][1] == pytest.approx(-7*Q)
    assert contract == saved


@pytest.mark.parametrize('field,value', [
    ('policy', 'existing_configured_path_goal_limits_no_following_geometry_claim'),
    ('pose_index', True), ('trajectory_sha256', '0'*64), ('model_sha256', '0'*64),
    ('epoch_ns', 1), ('additional_sdk_slack_rad', 0.), ('maximum_feedback_age_sec', .1),
    ('commanded_stop_reserve_rad', 0.), ('command_phase_age_sec', 0.),
    ('geometry_proof_required', False), ('continuous_accepted_feedback_monitor_required', False),
    ('command_phase_reserve_is_servo_motion_bound', True), ('motion_authorized', True),
    ('path_position_tolerance_rad', [.035]*6), ('joint_error_bounds_rad', [.035]*6),
])
def test_action_binding_recomputes_frozen_envelope_and_does_not_trust_scalar_report(field, value):
    args = inputs()
    contract = build(args)
    contract[field] = value
    with pytest.raises(ObservationPathError, match='BOUND_CONTRACT_INVALID'):
        binding(contract, args)


def test_binding_does_not_claim_current_feedback_and_submission_rechecks_age():
    args = inputs()
    contract = build(args)
    assert binding(contract, args)
    with pytest.raises(ObservationPathError, match='ENTRY_EVIDENCE'):
        fresh(contract, args, now_sec=11.)


def test_curve_moving_uncommanded_axis_is_rejected_even_when_endpoints_match():
    args = inputs()
    args[0].joint_trajectory.points[-1].velocities[0] = .01
    with pytest.raises(ObservationPathError, match='CURVE_'):
        build(args)
