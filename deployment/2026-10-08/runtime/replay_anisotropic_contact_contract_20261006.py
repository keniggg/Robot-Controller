#!/usr/bin/env python3
"""Offline geometric feasibility only; no actual servo bound or ROS I/O.

The rejected trajectory was not published. Reconstruct its two-point ITP
curve and verify its complete command-derivative bounds against the saved
gateway log. Its wire digest is intentionally not claimed to be the original.
"""
import argparse
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import rosbag
from genpy import Duration, Time
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from alicia_flexible_grasp.grasp.contact_execution import frozen_contact_geometry
from alicia_flexible_grasp.robot.observation_path_guard import (
    FrozenObservationScene, SerialUrdfFk, wire_digest,
)
from alicia_flexible_grasp.robot.observation_preview import qualify_candidate_path
from alicia_flexible_grasp.robot.observation_tracking_contract import (
    SDK_HALF_COUNT_RAD, commanded_stop_reserve, command_derivative_bounds,
)
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES, SDK_QUANTUM_RAD as Q
from alicia_flexible_grasp.robot.contact_tracking_contract import (
    build_contact_tracking_contract, validate_contact_tracking_contract_binding,
)

STORE = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
BASE = STORE / 'ros_log/grasp_explicit_request_20261006T045404Z/contact_contract_audit_20261006T061000Z/report.json'
PARAMS = STORE / 'ros_log/grasp_support_fixed_20261006T040304Z/contact_margin_replay/current_runtime_parameters.json'
LOG = STORE / 'ros_log/ros_live_20260922_session/ros.log'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', required=True, type=Path)
    args = parser.parse_args()
    output = args.output_dir.resolve()
    if (not Path('/media/zhuyupei/SanStand').is_mount()
            or STORE.resolve() not in output.parents or output.exists()):
        raise ValueError('Require mounted SanStand and a new external report directory')
    report = json.loads(BASE.read_text())
    params = json.loads(PARAMS.read_text())['parameters']
    original_hash = report['original_path_audit']['trajectory_sha256']
    logged, logged_line = None, None
    with LOG.open(errors='replace') as lines:
        for number, line in enumerate(lines, 1):
            prefix = 'Controller reference continuous speed audit: '
            if prefix in line and original_hash in line:
                logged = json.loads(line.split(prefix, 1)[1])
                logged_line = number
    assert logged is not None and logged['segments'] == 1
    assert logged['segment_bounds'][0]['segment'] == 'bridge'
    result = json.loads(Path(report['source_evidence']['result']['path']).read_text())
    failure = json.loads(result['failure_reason'].split(': ', 1)[1])
    sdk_history, accepted_history = [], []
    with rosbag.Bag(report['source_evidence']['bag']['path'], 'r') as bag:
        plan = next(msg for _, msg, _ in bag.read_messages(topics=['/grasp_6d/plan_enriched'])
                    if msg.plan_id == report['plan_id'])
        second = failure['accepted_stamp_ns'] // 1000000000
        for topic, msg, _ in bag.read_messages(
                topics=['/alicia_d/sdk_command', '/alicia_d/accepted_joint_states'],
                start_time=Time(second-1), end_time=Time(second+2)):
            if topic == '/alicia_d/sdk_command' and msg.header.stamp.to_nsec() <= failure['sdk_stamp_ns']:
                sdk_history.append(msg)
            if topic == '/alicia_d/accepted_joint_states' and msg.header.stamp.to_nsec() <= failure['accepted_stamp_ns']:
                accepted_history.append(msg)
    scene = FrozenObservationScene.from_contact_plan(plan)
    fk = SerialUrdfFk(params['/robot_description'], ARM_NAMES)
    assert fk.model_sha256 == report['model_sha256'] and scene.digest == report['scene_sha256']
    gripper = frozen_contact_geometry(plan, params['/grasp_6d/remote/gripper_geometry'],
                                      report['opening_width_m'])['gripper']
    hold_q = np.array(report['positions_rad']['controller_hold'])
    measured = np.array(report['positions_rad']['accepted_measured'])
    sdk = np.array(report['positions_rad']['sdk_command_quantized'])
    delta = np.array(failure['proposed_delta_counts']) * Q
    endpoint = sdk + delta
    duration = logged['final_duration_sec']
    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.joint_names = list(ARM_NAMES)
    trajectory.joint_trajectory.header.frame_id = 'base_link'
    trajectory.joint_trajectory.points = [
        JointTrajectoryPoint(positions=sdk.tolist(), velocities=[0.] * 6,
                             accelerations=(2. * delta / duration**2).tolist(),
                             time_from_start=Duration(0)),
        JointTrajectoryPoint(positions=endpoint.tolist(), velocities=[0.] * 6,
                             accelerations=(-2. * delta / duration**2).tolist(),
                             time_from_start=Duration.from_sec(duration)),
    ]
    hold = JointTrajectoryPoint(positions=hold_q.tolist(), velocities=[0.] * 6,
                               accelerations=[0.] * 6)
    derivatives = command_derivative_bounds(trajectory, hold)
    difference = {key: float(np.max(np.abs(np.array(values) -
                           logged['stopping_contract_derivatives'][key])))
                  for key, values in derivatives.items()}
    assert max(difference.values()) < 1e-10
    v = np.array(derivatives['maximum_velocity_rad_s'])
    acceleration = np.array(derivatives['maximum_acceleration_rad_s2'])
    stop = commanded_stop_reserve(.5, velocity=float(max(v)), acceleration=float(max(acceleration)))
    assert abs(stop - report['rejected_budget']['commanded_stop_reserve_rad']) < 1e-10
    rows = []
    # Age scenarios are diagnostics, not a proposed runtime freshness policy.
    for age in (0., .05, .1, .25, .5):
        phase_reserve = v * age + .5 * acceleration * age**2
        lower = abs(hold_q - measured) + Q + phase_reserve
        assert np.all(lower > 0.) and np.all(lower < .035)
        errors = lower + SDK_HALF_COUNT_RAD + stop
        audit = qualify_candidate_path(trajectory, hold, scene, fk, gripper,
                                       report['opening_width_m'], errors)
        rows.append(dict(command_phase_age_sec=age,
                         minimum_path_tolerances_rad=lower.tolist(),
                         desired_minus_measured_rad=(hold_q-measured).tolist(),
                         additional_command_phase_reserve_rad=phase_reserve.tolist(),
                         raw_entry_slack_rad=(lower-abs(hold_q-measured)).tolist(),
                         permitted_goal_tolerances_must_not_exceed_path=True,
                         full_geometry_audit=audit))
    # Exercise the production pure builder against actual raw historical
    # provenance. Config values below are the recorded action limits; this
    # does not assert that any current live configuration has these values.
    constraints = {name: dict(trajectory=.12, goal=.035) for name in ARM_NAMES}
    constraints.update(goal_time=1., stopped_velocity_tolerance=.01)
    module_contract = build_contact_tracking_contract(
        trajectory, hold, fk, sdk_history, accepted_history, constraints, .5,
        now_sec=max(failure['sdk_stamp_ns'], failure['accepted_stamp_ns'])*1e-9,
        epoch_ns=report['epoch_ns'])
    assert validate_contact_tracking_contract_binding(
        module_contract, trajectory, hold, constraints, .5)
    module_geometry = qualify_candidate_path(trajectory, hold, scene, fk, gripper,
        report['opening_width_m'], module_contract['joint_error_bounds_rad'])
    assert module_geometry['ok']
    final = dict(
        created_utc=datetime.now(timezone.utc).isoformat(),
        scope='offline_reconstructed_curve_anisotropic_contract_feasibility',
        motion_authorized=False, source_feedback_is_not_future_response_bound=True,
        hardware_tracking_or_stopping_certified=False,
        source_report=str(BASE), source_report_sha256=digest(BASE),
        source_parameters=str(PARAMS), source_parameters_sha256=digest(PARAMS),
        source_gateway_log=str(LOG), source_gateway_line=logged_line,
        original_trajectory_serialized_payload_available=False,
        original_rejected_trajectory_sha256=original_hash,
        reconstructed_trajectory_sha256=wire_digest(trajectory),
        reconstruction='2-point ITP zero endpoint velocity, endpoint acceleration +/-2*delta/T^2; checked against recorded bridge derivatives',
        logged_derivatives=logged['stopping_contract_derivatives'],
        reconstructed_derivatives=derivatives,
        derivative_reconstruction_max_abs_difference=difference,
        original_scalar_commanded_stop_reserve_rad=stop,
        sdk_quantum_rad=Q, sdk_half_quantum_rad=SDK_HALF_COUNT_RAD,
        plan_id=plan.plan_id, model_sha256=fk.model_sha256,
        required_support_clearance_m=gripper.support_clearance_m,
        held_axes_not_assumed_stationary=True, results=rows,
        pure_module_historical_raw_source_contract=module_contract,
        pure_module_full_geometry_audit=module_geometry,
        pure_module_config_source='offline candidate original tolerance caps; not a fresh runtime parameter read',
        limitations=[
            'Positive results are conditional geometric feasibility; tighter action tolerances may abort if actual following changes.',
            'The age term bounds command phase movement only, not unsampled physical servo movement.',
            'Runtime must use its original final serialized path and hold, not this reconstruction.',
            'Bind per-axis goal tolerances <= per-axis path tolerances; the existing uniform .035 endpoint tolerances are insufficient for this new envelope.',
            'Fresh independent accepted feedback, epoch/SDK/owner binding, pre-send revalidation, and existing torque-preserving stopping transport remain required.',
            'All original count, held command cell, support, collision, target, calibration, contact, time and measured-response gates remain unchanged.',
        ])
    output.mkdir(parents=True)
    (output / 'report.json').write_text(json.dumps(final, indent=2, ensure_ascii=False) + '\n')
    print(json.dumps(dict(output=str(output), cases=[
        dict(age=row['command_phase_age_sec'],
             ok=row['full_geometry_audit']['ok'],
             failure=row['full_geometry_audit'].get('reason'),
             minimum=row['full_geometry_audit'].get('continuous_path', {}).get(
                 'following_support', {}).get('minimum_support_clearance_lower_bound_m'))
        for row in rows])))


if __name__ == '__main__':
    main()
