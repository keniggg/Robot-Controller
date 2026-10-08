"""Independent entry/phase and immutable-envelope review, with no ROS I/O."""
from copy import deepcopy

import numpy as np
import pytest
import rospy

from alicia_flexible_grasp.robot.observation_path_guard import ObservationPathError
from alicia_flexible_grasp.robot.observation_tracking_contract import SDK_HALF_COUNT_RAD
from alicia_flexible_grasp.robot.stationary_following import SDK_QUANTUM_RAD as Q
from test_contact_tracking_contract import inputs, build, binding, fresh


def test_full_configured_age_is_reserved_even_when_capture_is_fresh():
    args = inputs()
    contract = build(args, now_sec=10.001)
    velocity = np.array(contract['command_derivative_bounds']['maximum_velocity_rad_s'])
    acceleration = np.array(contract['command_derivative_bounds']['maximum_acceleration_rad_s2'])
    # The SDK sample is only 1 ms old; the reserve must still use the entire
    # admitted 500 ms interval, never just the current observed sample age.
    expected_phase = velocity*.5 + acceleration*.5**2/2
    np.testing.assert_array_equal(contract['command_phase_age_reserve_rad'], expected_phase)
    assert np.any(expected_phase > velocity*.001 + acceleration*.001**2/2)
    stop = .625*max(velocity)*.5 + .07*max(acceleration)*.5**2 + 1e-12
    np.testing.assert_array_equal(contract['joint_error_bounds_rad'],
        np.array(contract['path_position_tolerance_rad']) + stop + SDK_HALF_COUNT_RAD)
    assert binding(contract, args)


def test_new_current_stationary_window_does_not_rebase_the_frozen_contract():
    args = inputs()
    contract = build(args)
    saved = deepcopy(contract)
    for messages in args[3:5]:
        for message in messages:
            message.header.stamp += rospy.Duration.from_sec(1.)
    measured = fresh(contract, args, now_sec=11.05)
    assert measured['accepted_stamp_ns'] > contract['entry_feedback']['accepted_stamp_ns']
    assert contract == saved


@pytest.mark.parametrize('axis', range(6))
def test_entry_can_only_consume_existing_slack_not_silently_widen_any_axis(axis):
    args = inputs()
    contract = build(args)
    original = deepcopy(contract)
    for message in args[4]:
        message.position[axis] -= 2*Q
    with pytest.raises(ObservationPathError):
        fresh(contract, args)
    assert contract == original


@pytest.mark.parametrize('field', [
    'command_phase_age_reserve_rad', 'joint_error_bounds_rad',
    'path_position_tolerance_rad', 'entry_slack_rad',
])
def test_one_axis_reduction_cannot_survive_binding_even_without_changing_total(field):
    args = inputs()
    contract = build(args)
    contract[field][0] -= Q/8
    contract[field][1] += Q/8
    with pytest.raises(ObservationPathError):
        binding(contract, args)


def test_changing_only_fractional_controller_hold_invalidates_frozen_proof():
    args = inputs()
    contract = build(args)
    args[1].positions[3] += Q/4
    with pytest.raises(ObservationPathError):
        binding(contract, args)
    with pytest.raises(ObservationPathError):
        fresh(contract, args)


def test_stationary_source_does_not_create_a_servo_motion_bound_or_motion_authority():
    contract = build(inputs())
    assert contract['entry_feedback']['measurement_kind'] == 'stationary_sdk_following_not_executed_endpoint'
    assert contract['entry_feedback']['certifies_command_response_or_calibration'] is False
    assert contract['certifies_hardware_tracking_stopping_or_calibration'] is False
    assert contract['command_phase_reserve_is_servo_motion_bound'] is False
    assert contract['motion_authorized'] is False
    assert contract['geometry_proof_required'] is True
    assert contract['continuous_accepted_feedback_monitor_required'] is True
