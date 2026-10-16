"""Stricter per-axis JTC tolerances for an already admitted contact microstep.

Stationary feedback establishes entry admissibility, not a future following
bound. Execution must enforce these positive path AND goal tolerances and
continuously monitor fresh accepted feedback. The existing complete geometry
proof, count budgets, ownership and measured contact gates remain mandatory.
This pure module creates neither motion authority nor a servo-speed claim.
"""
from copy import deepcopy
import math
import re
from collections.abc import Mapping

import numpy as np

from .observation_path_guard import (
    ObservationPathError, _polynomial_bounds, segment_coefficients, wire_digest,
)
from .observation_tracking_contract import (
    SDK_HALF_COUNT_RAD, commanded_stop_reserve, tracking_contract_candidates,
    validate_command_derivatives,
)
from .stationary_following import ARM_NAMES, SDK_QUANTUM_RAD as Q, stationary_following_error

POLICY = 'contact_microstep_per_axis_jtc_entry_slack_v1'
SCOPE = 'contact_endpoint_microstep_only'
MAX_AGE_SEC = .5


def _positive(value, label):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or value <= 0.):
        raise ObservationPathError('CONTACT_TRACKING_INVALID_' + label)
    return float(value)


def _vector(values, label):
    result = np.asarray(values, dtype=float)
    if result.shape != (6,) or not np.all(np.isfinite(result)):
        raise ObservationPathError('CONTACT_TRACKING_INVALID_' + label)
    return result


def _microstep(trajectory, hold, sdk):
    tr = trajectory.joint_trajectory
    if tuple(tr.joint_names) != ARM_NAMES or getattr(trajectory.multi_dof_joint_trajectory, 'points', ()):
        raise ObservationPathError('CONTACT_TRACKING_CANONICAL_SIX_JOINTS_REQUIRED')
    q = _vector(hold.positions, 'HOLD')
    if (np.max(abs(q-sdk)) > Q/2+1e-9
            or not np.array_equal(_vector(hold.velocities, 'HOLD_VELOCITIES'), np.zeros(6))
            or not np.array_equal(_vector(hold.accelerations, 'HOLD_ACCELERATIONS'), np.zeros(6))):
        raise ObservationPathError('CONTACT_TRACKING_STATIONARY_SDK_HOLD_REQUIRED')
    points = tuple(tr.points)
    if not 2 <= len(points) <= 4096:
        raise ObservationPathError('CONTACT_TRACKING_MICROSTEP_REQUIRED')
    start = _vector(points[0].positions, 'START')
    goal = _vector(points[-1].positions, 'GOAL')
    if not np.allclose(start, sdk, atol=1e-12, rtol=0.):
        raise ObservationPathError('CONTACT_TRACKING_SDK_START_CHANGED')
    counts = np.rint(goal/Q)
    if np.max(abs(goal-counts*Q)) > 1e-12:
        raise ObservationPathError('CONTACT_TRACKING_QUANTIZED_TARGET_REQUIRED')
    delta = goal-sdk
    if np.max(abs(delta)) > 4*Q+1e-12 or np.max(abs(delta)) < Q/2:
        raise ObservationPathError('CONTACT_TRACKING_FOUR_COUNT_MICROSTEP_REQUIRED')
    times = [float(p.time_from_start.to_sec()) for p in points]
    if (times[0] != 0. or any(not math.isfinite(t) or t < 0. for t in times)
            or any(b <= a for a, b in zip(times, times[1:]))):
        raise ObservationPathError('CONTACT_TRACKING_INVALID_TIMING')
    unchanged = abs(delta) < Q/2
    for a, b, duration in [(hold, points[1], times[1])] + [
            (a, b, tb-ta) for a, b, ta, tb in zip(points[1:], points[2:], times[1:], times[2:])]:
        low, high = _polynomial_bounds(segment_coefficients(a, b, duration, 6), 0., 1.)
        if np.any(low < sdk-4*Q-1e-9) or np.any(high > sdk+4*Q+1e-9):
            raise ObservationPathError('CONTACT_TRACKING_CURVE_LEAVES_FOUR_COUNT_BOX')
        if np.any(low[unchanged] < sdk[unchanged]-Q/2) or np.any(high[unchanged] >= sdk[unchanged]+Q/2):
            raise ObservationPathError('CONTACT_TRACKING_CURVE_MOVES_UNCOMMANDED_AXIS')
    return q


def build_contact_tracking_contract(trajectory, hold, fk, sdk_messages, accepted_messages,
                                    constraints, stop_duration, *, now_sec, epoch_ns,
                                    pose_index=2, minimum_path_tolerance_rad=Q,
                                    maximum_age_sec=MAX_AGE_SEC):
    """Return a prospective conditional contract; caller still proves geometry.

    The path lower bound includes observed entry error, one *additional* SDK
    quantum on EVERY axis, and command-phase movement over the full admitted
    freshness interval. The latter is not a bound on unobserved servo motion.
    Stop/half-count reserves retain the original proof's scalar quantities.
    """
    if type(pose_index) is not int or pose_index != 2:
        raise ObservationPathError('CONTACT_TRACKING_CONTACT_ENDPOINT_ONLY')
    floor = _positive(minimum_path_tolerance_rad, 'CONFIGURED_FLOOR')
    age = _positive(maximum_age_sec, 'MAXIMUM_AGE')
    if floor < Q or age > MAX_AGE_SEC:
        raise ObservationPathError('CONTACT_TRACKING_FLOOR_OR_FRESHNESS_RELAXED')
    if tuple(fk.names) != ARM_NAMES:
        raise ObservationPathError('CONTACT_TRACKING_CANONICAL_FK_REQUIRED')
    try:
        sample = stationary_following_error(fk, sdk_messages, accepted_messages,
            now_sec=now_sec, epoch_ns=epoch_ns, maximum_age_sec=age)
    except (TypeError, ValueError) as exc:
        raise ObservationPathError('CONTACT_TRACKING_ENTRY_EVIDENCE: ' + str(exc)) from exc
    sdk = _vector(sample['sdk_positions_rad'], 'SDK')
    if np.max(abs(sdk/Q-np.rint(sdk/Q))) > 1e-9:
        raise ObservationPathError('CONTACT_TRACKING_QUANTIZED_SDK_REQUIRED')
    q = _microstep(trajectory, hold, sdk)
    derivatives = validate_command_derivatives(trajectory, hold)
    # Keep all original config validation and the narrowest existing cap.
    choices = tracking_contract_candidates(constraints, ARM_NAMES, stop_duration,
                                            command_bounds=derivatives)
    cap = _vector(choices[-1]['path_position_tolerance_rad'], 'CONFIGURED_CAP')
    actual = _vector(sample['accepted_positions_rad'], 'ACCEPTED')
    entry_error = abs(q-actual)
    velocity = np.asarray(derivatives['maximum_velocity_rad_s'])
    acceleration = np.asarray(derivatives['maximum_acceleration_rad_s2'])
    phase = velocity*age + .5*acceleration*age**2
    path = np.maximum(floor, entry_error+Q+phase)
    if np.any(path > cap):
        raise ObservationPathError('CONTACT_TRACKING_ENTRY_SLACK_EXCEEDS_ORIGINAL_CAP')
    if not isinstance(constraints, Mapping):
        raise ObservationPathError('CONTACT_TRACKING_CONSTRAINTS_REQUIRED')
    goal_caps = np.array([_positive(constraints[name].get('goal'), 'GOAL_TOLERANCE')
                          for name in ARM_NAMES])
    goal = np.minimum(goal_caps, path)
    stop = commanded_stop_reserve(stop_duration, velocity=float(max(velocity)),
                                  acceleration=float(max(acceleration)))
    return dict(policy=POLICY, scope=SCOPE, pose_index=pose_index,
        joint_names=list(ARM_NAMES), trajectory_sha256=wire_digest(trajectory),
        model_sha256=fk.model_sha256, epoch_ns=epoch_ns,
        controller_hold_positions_rad=q.tolist(), sdk_positions_rad=sdk.tolist(),
        path_position_tolerance_rad=path.tolist(), goal_position_tolerance_rad=goal.tolist(),
        joint_error_bounds_rad=(path+stop+SDK_HALF_COUNT_RAD).tolist(),
        configured_path_caps_rad=cap.tolist(), controller_constraints_snapshot=deepcopy(constraints),
        minimum_path_tolerance_rad=floor, maximum_feedback_age_sec=age,
        entry_feedback=deepcopy(sample), entry_absolute_error_rad=entry_error.tolist(),
        entry_slack_rad=(path-entry_error).tolist(), additional_sdk_slack_rad=Q,
        command_phase_age_reserve_rad=phase.tolist(), command_phase_age_sec=age,
        command_phase_reserve_is_servo_motion_bound=False,
        commanded_stop_reserve_rad=stop, stop_trajectory_duration_sec=float(stop_duration),
        maximum_command_velocity_rad_s=float(max(velocity)),
        maximum_command_acceleration_rad_s2=float(max(acceleration)),
        command_derivative_bounds=derivatives,
        certifies_hardware_tracking_stopping_or_calibration=False,
        geometry_proof_required=True, continuous_accepted_feedback_monitor_required=True,
        motion_authorized=False)


def revalidate_contact_tracking_contract(contract, trajectory, hold, fk, sdk_messages,
                                         accepted_messages, constraints, stop_duration,
                                         *, now_sec, epoch_ns, pose_index=2):
    """Check a frozen contract against new complete entry feedback; no widening.

    An encoder change may consume slack, never silently modify the selected
    tolerances or their geometry proof. Runtime source/scene/owner checks are
    still independent and must run immediately before sending the exact goal.
    """
    if not isinstance(contract, dict) or contract.get('policy') != POLICY:
        raise ObservationPathError('CONTACT_TRACKING_POLICY_MISMATCH')
    validate_contact_tracking_contract_binding(contract, trajectory, hold, constraints, stop_duration)
    fresh = build_contact_tracking_contract(trajectory, hold, fk, sdk_messages,
        accepted_messages, constraints, stop_duration, now_sec=now_sec, epoch_ns=epoch_ns,
        pose_index=pose_index, minimum_path_tolerance_rad=contract.get('minimum_path_tolerance_rad'),
        maximum_age_sec=contract.get('maximum_feedback_age_sec'))
    variable = {'entry_feedback', 'entry_absolute_error_rad', 'entry_slack_rad',
                'path_position_tolerance_rad', 'goal_position_tolerance_rad', 'joint_error_bounds_rad'}
    for key, value in fresh.items():
        if key not in variable and contract.get(key) != value:
            raise ObservationPathError('CONTACT_TRACKING_BOUND_CONTEXT_CHANGED: ' + key)
    path = _vector(contract.get('path_position_tolerance_rad'), 'BOUND_PATH')
    goal = _vector(contract.get('goal_position_tolerance_rad'), 'BOUND_GOAL')
    caps = np.asarray(fresh['configured_path_caps_rad'])
    required = np.asarray(fresh['path_position_tolerance_rad'])
    expected_goal = np.minimum(path, [constraints[name]['goal'] for name in ARM_NAMES])
    expected_errors = path+fresh['commanded_stop_reserve_rad']+SDK_HALF_COUNT_RAD
    if (np.any(path <= 0.) or np.any(path > caps) or np.any(path < required)
            or np.any(goal <= 0.) or not np.array_equal(goal, expected_goal)
            or not np.array_equal(_vector(contract.get('joint_error_bounds_rad'), 'BOUND_ERRORS'), expected_errors)):
        raise ObservationPathError('CONTACT_TRACKING_FROZEN_ENVELOPE_OR_ENTRY_CHANGED')
    return fresh['entry_feedback']


def validate_contact_tracking_contract_binding(contract, trajectory, hold, constraints, stop_duration):
    """Recompute the frozen envelope without asserting current source freshness.

    For the action builder, which has no raw feedback access. Submission must
    separately call revalidate_contact_tracking_contract with current history.
    An internally consistent summary is not a substitute for that provenance.
    """
    try:
        if not isinstance(contract, dict):
            raise ValueError('contract missing')
        required = dict(policy=POLICY, scope=SCOPE, pose_index=2,
            joint_names=list(ARM_NAMES), trajectory_sha256=wire_digest(trajectory),
            controller_constraints_snapshot=constraints,
            stop_trajectory_duration_sec=float(stop_duration),
            command_phase_reserve_is_servo_motion_bound=False,
            certifies_hardware_tracking_stopping_or_calibration=False,
            geometry_proof_required=True, continuous_accepted_feedback_monitor_required=True,
            motion_authorized=False)
        for key, value in required.items():
            if contract.get(key) != value or (type(value) is bool and type(contract.get(key)) is not bool):
                raise ValueError('context changed: ' + key)
        if type(contract.get('pose_index')) is not int:
            raise ValueError('invalid pose index')
        sample = contract['entry_feedback']
        epoch = contract['epoch_ns']
        if (not isinstance(sample, dict) or type(epoch) is not int or epoch <= 0
                or sample.get('epoch_ns') != epoch
                or not re.fullmatch('[0-9a-f]{64}', str(contract.get('model_sha256', '')))
                or sample.get('model_sha256') != contract['model_sha256']
                or sample.get('joint_names') != list(ARM_NAMES)
                or sample.get('measurement_kind') != 'stationary_sdk_following_not_executed_endpoint'
                or sample.get('certifies_command_response_or_calibration') is not False):
            raise ValueError('entry model/epoch/source mismatch')
        floor = _positive(contract['minimum_path_tolerance_rad'], 'CONFIGURED_FLOOR')
        age = _positive(contract['maximum_feedback_age_sec'], 'MAXIMUM_AGE')
        if floor < Q or age > MAX_AGE_SEC:
            raise ValueError('floor or freshness relaxed')
        stamps = [sample[key] for key in ('sdk_stamp_ns', 'accepted_stamp_ns',
                  'stationary_window_start_ns', 'stationary_window_end_ns')]
        if (any(type(stamp) is not int or stamp < epoch for stamp in stamps)
                or stamps[3] != min(stamps[:2]) or stamps[3]-stamps[2] < 300000000
                or any(type(sample[key]) is not int or sample[key] < 2
                       for key in ('sdk_sample_count', 'accepted_sample_count'))):
            raise ValueError('incomplete frozen stationary window')
        captured = _positive(sample['stamp_sec'], 'CAPTURE_TIME')
        if any(not 0. <= captured-stamp*1e-9 <= age for stamp in stamps[:2]):
            raise ValueError('entry was stale at capture')
        if sample['opening_stamp_ns'] != sample['accepted_stamp_ns']:
            raise ValueError('opening source changed')
        opening = sample['accepted_gripper_opening_m']
        if isinstance(opening, bool) or not math.isfinite(opening) or not 0. <= opening <= .05:
            raise ValueError('invalid entry opening')
        sdk = _vector(sample['sdk_positions_rad'], 'ENTRY_SDK')
        actual = _vector(sample['accepted_positions_rad'], 'ENTRY_ACCEPTED')
        if np.max(abs(sdk/Q-np.rint(sdk/Q))) > 1e-9:
            raise ValueError('SDK is not quantized')
        q = _microstep(trajectory, hold, sdk)
        derivatives = validate_command_derivatives(trajectory, hold)
        choices = tracking_contract_candidates(constraints, ARM_NAMES, stop_duration,
                                                command_bounds=derivatives)
        caps = np.array(choices[-1]['path_position_tolerance_rad'])
        velocity = np.array(derivatives['maximum_velocity_rad_s'])
        acceleration = np.array(derivatives['maximum_acceleration_rad_s2'])
        phase = velocity*age + .5*acceleration*age**2
        entry_error = abs(q-actual)
        path = np.maximum(floor, entry_error+Q+phase)
        if np.any(path > caps):
            raise ValueError('entry slack exceeds original caps')
        goals = np.array([_positive(constraints[name].get('goal'), 'GOAL_TOLERANCE')
                          for name in ARM_NAMES])
        stop = commanded_stop_reserve(stop_duration, velocity=float(max(velocity)),
                                      acceleration=float(max(acceleration)))
        recomputed = dict(controller_hold_positions_rad=q.tolist(), sdk_positions_rad=sdk.tolist(),
            path_position_tolerance_rad=path.tolist(), goal_position_tolerance_rad=np.minimum(path, goals).tolist(),
            joint_error_bounds_rad=(path+stop+SDK_HALF_COUNT_RAD).tolist(),
            configured_path_caps_rad=caps.tolist(), command_derivative_bounds=derivatives,
            entry_absolute_error_rad=entry_error.tolist(), entry_slack_rad=(path-entry_error).tolist(),
            additional_sdk_slack_rad=Q, command_phase_age_sec=age,
            command_phase_age_reserve_rad=phase.tolist(), commanded_stop_reserve_rad=stop,
            maximum_command_velocity_rad_s=float(max(velocity)),
            maximum_command_acceleration_rad_s2=float(max(acceleration)))
        for key, value in recomputed.items():
            if contract.get(key) != value:
                raise ValueError('frozen envelope changed: ' + key)
    except (AttributeError, KeyError, TypeError, ValueError, OverflowError) as exc:
        raise ObservationPathError('CONTACT_TRACKING_BOUND_CONTRACT_INVALID: ' + str(exc)) from exc
    return True
