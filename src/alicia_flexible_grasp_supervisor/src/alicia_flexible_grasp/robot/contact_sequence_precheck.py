"""Read-only contact candidate check with the current measured following bias.

This is a constant-bias prediction at virtual future poses, not future measured
feedback. It cannot authorize execution or replace the fresh contact gateway.
"""
from copy import deepcopy
import json
import math
import hashlib
import os
from pathlib import Path
import time

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest
from trajectory_msgs.msg import JointTrajectoryPoint

from alicia_flexible_grasp_supervisor.srv import CheckContactSequenceResponse
from alicia_flexible_grasp.grasp.gripper_geometry import (
    GripperGeometry, gripper_contract_mismatch_reason, ANALYTICAL_GRIPPER_MODEL_NAME,
    _stage_boxes, _box_corners, _obb_overlap)
from alicia_flexible_grasp.grasp.endpoint_tolerance import measured_position_tolerance
from alicia_flexible_grasp.grasp.rich_plan_integrity import pose_values
from .contact_linear_compensation import compensate_cartesian_path
from .contact_replay_receipt import (
    DiagnosticStore, ReplayStorageUnavailable, best_effort_failure_receipt,
    contact_failure_data,
)
from .endpoint_correction import sdk_counts
from .held_axes_prediction import predict_held_axes_endpoint, HeldAxesPredictionError
from .held_pregrasp_selection import select_held_pregrasp_endpoint
from .observation_path_guard import (
    FrozenObservationScene, ObservationPathError, SerialUrdfFk,
    validate_observation_trajectory, wire_digest, segment_coefficients, _polynomial_bounds,
    gripper_target_overlap)
from .pregrasp_compensation import PregraspCompensation
from .pregrasp_timing import temporary_attribute
from .stationary_following import (
    stationary_following_error, _window, ARM_NAMES, SDK_QUANTUM_RAD as Q)
from .following_model import following_offset, following_evidence, validate_source_bias_model


class ContactPrecheckError(ObservationPathError):
    def __init__(self, message, code='CONTACT_SEQUENCE_PRECHECK_FAILED'):
        super().__init__(message)
        self.code = code


def predict_held_approach(pregrasp_path, fk, sample, pregrasp_goal, approach_goal, config, audit,
                          *, following_model=None):
    """Predict the actual held-axis policy, without inventing future feedback.

    Nominal pregrasp commands are quantized just as on the wire. Their future
    encoders use the present observed SDK-minus-encoder bias conditionally.
    Any enabled pose0 correction must be propagated before predicting pose1.
    This endpoint screen supplements existing path checks; execution still
    requires fresh real feedback and its own complete path validation.
    """
    trajectory = pregrasp_path.joint_trajectory
    if tuple(trajectory.joint_names) != ARM_NAMES or not trajectory.points:
        raise ContactPrecheckError('held approach requires canonical pregrasp trajectory')
    if following_model is not None:
        validate_source_bias_model(following_model, sample)
        offset = following_offset(sample['sdk_positions_rad'], sample['accepted_positions_rad'],
                                  following_model=following_model)
    else:
        offset = np.asarray(sample['sdk_positions_rad'])-np.asarray(sample['accepted_positions_rad'])
    mapped = {'following_model': following_model} if following_model is not None else {}
    command = (sdk_counts(trajectory.points[-1].positions)-2048)*Q
    virtual = command-offset
    tolerance = measured_position_tolerance(config, 'unknown_tabletop')
    angle = math.radians(float(config.get('measured_endpoint_orientation_tolerance_deg', 5.)))
    audit.update(scope='constant_current_bias_held_approach_prediction',
        future_following_verified=False, execution_authorized=False,
        hardware_response_verified=False, actual_path_requires_fresh_execution_check=True,
        source_sdk_stamp_ns=sample.get('sdk_stamp_ns'),
        source_accepted_stamp_ns=sample.get('accepted_stamp_ns'),
        sdk_minus_encoder_rad=(np.asarray(sample['sdk_positions_rad'])-
            np.asarray(sample['accepted_positions_rad'])).tolist(),
        nominal_pregrasp_command_rad=command.tolist())
    if following_model is not None:
        audit['following_reference'] = following_evidence(sample['sdk_positions_rad'],
            sample['accepted_positions_rad'], following_model=following_model)
    # Disabled pose0 compensation predicts the uncorrected nominal endpoint.
    # Other correction algorithms must not be silently impersonated here.
    correction = config.get('pregrasp_cartesian_compensation_enabled', False)
    if type(correction) is not bool:
        raise ContactPrecheckError('PREGRASP_COMPENSATION_CONFIG_INVALID')
    if correction:
        if (config.get('unknown_pregrasp_feedforward_enabled') is not True
                or config.get('unknown_pregrasp_held_axes_feedforward_enabled') is not True):
            raise ContactPrecheckError('HELD_APPROACH_PREGRASP_PREDICTION_POLICY_UNSUPPORTED')
        try:
            _, result = predict_held_axes_endpoint(fk, command, virtual, pose_values(pregrasp_goal),
                position_tolerance_m=tolerance, orientation_tolerance_rad=angle,
                allow_already_converged=True, **mapped)
        except HeldAxesPredictionError as exc:
            audit.update(predicted_pregrasp=exc.evidence, success=False)
            raise ContactPrecheckError('held approach pregrasp prediction: '+str(exc)) from exc
        audit['predicted_pregrasp'] = result
        command = np.asarray(result['predicted_command_positions_rad'])
        virtual = np.asarray(result['predicted_positions_rad'])
    else:
        audit['predicted_pregrasp'] = dict(correction_enabled=False,
            predicted_command_positions_rad=command.tolist(), predicted_positions_rad=virtual.tolist())
    try:
        _, result = predict_held_axes_endpoint(fk, command, virtual, pose_values(approach_goal),
            position_tolerance_m=min(tolerance, .012), orientation_tolerance_rad=angle,
            allow_already_converged=True, **mapped)
    except HeldAxesPredictionError as exc:
        audit.update(predicted_approach=exc.evidence, success=False)
        raise ContactPrecheckError('held approach prediction: '+str(exc)) from exc
    audit.update(predicted_approach=result, success=True)
    return audit


def predict_contact_command_path(nominal, fk, sample, goal, *, following_model=None):
    """Apply an observed bias to a virtual start, explicitly as a prediction."""
    if not nominal.joint_trajectory.points:
        raise ContactPrecheckError('missing nominal contact trajectory')
    measured = np.asarray(sample['accepted_positions_rad'], dtype=float)
    sdk = np.asarray(sample['sdk_positions_rad'], dtype=float)
    virtual_start = np.asarray(nominal.joint_trajectory.points[0].positions, dtype=float)
    if (measured.shape != (6,) or sdk.shape != (6,)
            or virtual_start.shape != (6,)
            or not np.all(np.isfinite(measured)) or not np.all(np.isfinite(sdk))):
        raise ContactPrecheckError('invalid measured following sample')
    if following_model is not None:
        validate_source_bias_model(following_model, sample)
        offset = following_offset(sdk, measured, following_model=following_model)
    else:
        offset = sdk-measured
    projection = dict(model_sha256=sample['model_sha256'],
        joint_names=sample['joint_names'], accepted_positions_rad=virtual_start.tolist(),
        sdk_positions_rad=(virtual_start+offset).tolist())
    shifted, derivation = compensate_cartesian_path(nominal, fk, projection, goal,
        **({'following_model': following_model} if following_model is not None else {}))
    # The arithmetic helper names its inputs as measured. Do not mislabel a
    # virtual future state in the audit or pass it to an execution gateway.
    derivation['virtual_nominal_start_positions_rad'] = derivation.pop('measured_start_positions_rad')
    derivation['predicted_command_start_positions_rad'] = derivation.pop('command_start_positions_rad')
    derivation.update(scope='constant_current_bias_prediction',
                      future_following_verified=False, execution_authorized=False)
    return shifted, derivation


def contact_precheck_gripper(config):
    g = GripperGeometry(float(config.get('max_inner_gap_m', .05)),
        float(config.get('width_safety_margin_per_side_m', .002)),
        np.asarray(config.get('finger_box_xyz_m', [.0434,.0286,.06])),
        np.asarray(config.get('palm_box_xyz_m', [.1175,.155,.0774])),
        float(config.get('support_clearance_m', .003)))
    mismatch = gripper_contract_mismatch_reason(g, .05, .05,
        ANALYTICAL_GRIPPER_MODEL_NAME, .05,
        tool_jaw_axis=config.get('tool_jaw_axis', 'y'),
        tool_finger_length_axis=config.get('tool_finger_length_axis', 'z'))
    if mismatch:
        raise ContactPrecheckError('gripper contract: '+mismatch)
    return g


def _emit_contact_replay_failure(sink, error, curve, hold, curve_kind, stage):
    """The sink is passive; ordinary diagnostic faults preserve the rejection."""
    if sink is None:
        return None
    try:
        receipt = sink(error, curve, hold, curve_kind, stage)
    except Exception as exc:
        receipt = dict(status='unavailable', diagnostic_only=True, execution_authorized=False,
            diagnostic_error_type=type(exc).__name__, diagnostic_error_message=str(exc))
    try:
        error.contact_replay_receipt = receipt
    except Exception:
        pass
    return receipt


class _ContactReplayLogStore:
    def put(self, data):
        directory = os.environ.get('ROS_LOG_DIR')
        if not directory:
            raise ReplayStorageUnavailable('ROS_LOG_DIR is unavailable; no replay storage fallback')
        # DiagnosticStore independently requires the actual SanStand mount
        # and confines this resolved directory to the external archive.
        return DiagnosticStore(Path(directory)/'contact_replay_receipts').put(data)


def _contact_replay_failure_sink(audit, plan, scene, model, epoch, selection,
                                 following_sample, geometry, config, gripper_config, gripper,
                                 opening, opening_source):
    """Freeze caller-local context without turning diagnostics into a gate."""
    context_error = None
    try:
        binding = dict(plan_id=str(plan.plan_id), plan_sha256=scene.digest,
            source_stamp_ns=scene.source_ns, target_track_id=scene.track_id,
            control_epoch_ns=epoch, selection=deepcopy(selection))
        frozen_config = deepcopy(dict(grasp=config, gripper_geometry=geometry, gripper=gripper_config,
            evaluated_gripper=dict(max_inner_gap_m=gripper.max_inner_gap_m,
                jaw_clearance_each_side_m=gripper.jaw_clearance_each_side_m,
                finger_size_xyz_m=gripper.finger_size_xyz_m.tolist(),
                palm_size_xyz_m=gripper.palm_size_xyz_m.tolist(),
                support_clearance_m=gripper.support_clearance_m)))
        frozen_sample = deepcopy(following_sample)
        frozen_plan = dict(poses_xyz_xyzw=[list(pose_values(p)) for p in plan.poses],
            contact_geometry_evidence=str(plan.contact_geometry_evidence),
            diagnostic=str(plan.diagnostic), model_choice=str(plan.model_choice))
    except Exception as exc:
        context_error = '%s: %s' % (type(exc).__name__, exc)
    def sink(error, curve, hold, curve_kind, stage):
        def build():
            if context_error is not None:
                raise ValueError('replay context capture unavailable: '+context_error)
            first = deepcopy(getattr(error, 'contact_replay_sample', None))
            if not isinstance(first, dict):
                first = dict(available=False, reason='checker supplied no first-failure FK sample')
            first.update(stage=str(stage), curve=str(curve_kind))
            # A compact same-curve endpoint diagnostic can seed a new planning
            # proposal without reading a replay archive. It grants no motion
            # authority and does not describe other IK branches or a path.
            if (first.get('endpoint_screen_only') is True
                    and str(stage) == 'grasp' and str(curve_kind) == 'predicted_command'
                    and np.array_equal(first.get('joint_positions_rad'),
                                       curve.joint_trajectory.points[-1].positions)):
                audit.setdefault('contact_terminal_failure', dict(schema_version=1,
                    plan_id=str(plan.plan_id), source_stamp_ns=scene.source_ns,
                    model_sha256=hashlib.sha256(model.encode('utf-8')).hexdigest(),
                    stage='grasp', curve='predicted_command', endpoint_screen_only=True,
                    execution_authorized=False,
                    command_endpoint_rad=list(curve.joint_trajectory.points[-1].positions),
                    trajectory_sha256=wire_digest(curve)))
            captured_config = deepcopy(frozen_config)
            captured_config['checker_options'] = first.get('checker_options', {})
            source = Path(__file__)
            captured_config['replay_source_sha256'] = {
                source.name: hashlib.sha256(source.read_bytes()).hexdigest(),
                'observation_path_guard.py': hashlib.sha256(
                    source.with_name('observation_path_guard.py').read_bytes()).hexdigest(),
                'finger_cad_separation.py': hashlib.sha256(
                    source.with_name('finger_cad_separation.py').read_bytes()).hexdigest(),
                'gripper_geometry.py': hashlib.sha256(
                    (source.parent.parent/'grasp'/'gripper_geometry.py').read_bytes()).hexdigest()}
            return contact_failure_data(binding=binding, checked_curve=curve,
                controller_desired=hold, scene=scene, model_xml=model,
                opening_m=opening, opening_source=opening_source,
                following_sample=frozen_sample, config=captured_config,
                first_failed_sample=first,
                failure=dict(code=str(getattr(error, 'code', 'MOVEIT_CHECK_ERROR')),
                    message=str(error), stage=str(stage)), plan_data=frozen_plan)
        receipt = best_effort_failure_receipt(error, build, _ContactReplayLogStore())
        receipt.update(stage=str(stage), curve=str(curve_kind))
        # Do not let later exception flattening erase the first failing curve.
        audit.setdefault('contact_failure_replay', receipt)
        return receipt
    return sink


def contact_curve_terminal_collision(trajectory, fk, scene, gripper, opening):
    """Reject an exact endpoint of this proposed curve; never accept a path.

    Retiming below must preserve these joint positions. An overlapping terminal
    is therefore a necessary failure of the existing continuous dual-curve
    gate for THIS proposed IK branch. It says nothing about other IK branches,
    candidate poses or physical reachability. Noncanonical test/provider
    objects retain the original complete checker instead of a guessed screen.
    """
    if (not isinstance(fk, SerialUrdfFk)
            or not isinstance(scene, FrozenObservationScene)
            or not isinstance(gripper, GripperGeometry)):
        return dict(checked=False, reason='canonical frozen geometry unavailable')
    tr = getattr(trajectory, 'joint_trajectory', None)
    points = getattr(tr, 'points', ())
    if (tuple(getattr(tr, 'joint_names', ())) != tuple(fk.names)
            or not 2 <= len(points) <= 4096
            or not math.isfinite(opening) or not 0. <= opening <= gripper.max_inner_gap_m):
        return dict(checked=False, reason='retain original complete trajectory validation')
    q = np.asarray(points[-1].positions, dtype=float)
    transform = fk(q)  # Original finite-value and URDF limit validation.
    label = '%d-end' % (len(points)-2)
    cad_proofs = []
    for body, center, size in _stage_boxes(transform, gripper, opening, 'y', 'z'):
        overlap, proof = gripper_target_overlap(
            fk, gripper, opening, body, transform, center, size, scene)
        if proof is not None:
            cad_proofs.append(dict(body=body, proof=proof))
        if overlap:
            clearance = float(np.min(_box_corners(center, transform[:3, :3], size)
                                     @ scene.normal + scene.offset))
            error = ObservationPathError('target OBB collision at %s/%s' % (label, body))
            error.candidate_rejection_code = 'TARGET_OBB_COLLISION'
            error.candidate_rejection_at_start = False
            error.contact_replay_sample = dict(available=True, label=label,
                joint_positions_rad=q.tolist(), T_base_tool0=transform.tolist(),
                displacement_bound_m=0., body=body, body_center_base_m=np.asarray(center).tolist(),
                body_size_xyz_m=np.asarray(size).tolist(), support_clearance_m=clearance,
                checker_options={}, validation_kind='terminal_endpoint_screen',
                checked_point_count=1, continuous_validation_executed=False,
                endpoint_screen_only=True, other_ik_branches_rejected=False,
                sample_selection_policy='first_tested_terminal_not_first_collision_along_curve',
                first_collision_along_curve_claimed=False,
                trajectory_point_index=len(points)-1,
                trajectory_point_time_ns=int(points[-1].time_from_start.to_nsec()))
            if proof is not None:
                error.contact_replay_sample['cad_target_projection'] = deepcopy(proof)
            raise error
    return dict(checked=True, collision_at_terminal=False,
                scope='same_proposed_curve_endpoint_only', path_verified=False,
                cad_target_projection_proofs=cad_proofs)


def validate_contact_prediction(nominal, fk, sample, goal, scene, gripper, opening,
                                planner, state, collision_check, remaining, *, failure_sink=None):
    """Use the same full-curve validators as execution, without caching motion."""
    shifted, evidence = predict_contact_command_path(nominal, fk, sample, goal)
    # The terminal state is unchanged by the retiming/geometry equality gate.
    # Screen only exact same-branch counterexamples before interval proofs;
    # every nonrejected proposal still runs the original full validators.
    screens = {}
    for name, trajectory in (('nominal', nominal), ('predicted_command', shifted)):
        remaining()
        try:
            screens[name] = contact_curve_terminal_collision(
                trajectory, fk, scene, gripper, opening)
        except ObservationPathError as exc:
            # Preserve the existing replay contract: stored curves and holds
            # describe the actual retimed proposal, even for a cheap reject.
            before = deepcopy(trajectory)
            remaining()
            retimed = planner._retime_cartesian_plan(deepcopy(trajectory))
            if not planner._same_joint_path_geometry(before, retimed):
                raise ContactPrecheckError('retiming changed '+name+' path')
            remaining()
            try:
                contact_curve_terminal_collision(retimed, fk, scene, gripper, opening)
            except ObservationPathError as retimed_error:
                cause = retimed_error
            else:
                raise ContactPrecheckError('terminal collision changed after unchanged retiming') from exc
            error = ContactPrecheckError(name+' continuous geometry: '+str(cause))
            if hasattr(cause, 'contact_replay_sample'):
                error.contact_replay_sample = deepcopy(cause.contact_replay_sample)
            hold = JointTrajectoryPoint(positions=list(retimed.joint_trajectory.points[0].positions),
                                        velocities=[0.]*6, accelerations=[0.]*6)
            _emit_contact_replay_failure(failure_sink, error, retimed, hold, name, 'grasp')
            raise error from cause
    evidence['same_curve_terminal_screens'] = screens
    for name, trajectory in (('nominal', nominal), ('predicted_command', shifted)):
        before = deepcopy(trajectory)
        remaining()
        retimed = planner._retime_cartesian_plan(deepcopy(trajectory))
        if not planner._same_joint_path_geometry(before, retimed):
            raise ContactPrecheckError('retiming changed '+name+' path')
        hold = JointTrajectoryPoint(positions=list(retimed.joint_trajectory.points[0].positions),
                                    velocities=[0.]*6, accelerations=[0.]*6)
        try:
            report = validate_observation_trajectory(retimed, scene, fk, gripper,
                opening, hold, max_seconds=min(10., remaining()))
        except ObservationPathError as exc:
            error = ContactPrecheckError(name+' continuous geometry: '+str(exc))
            if hasattr(exc, 'contact_replay_sample'):
                error.contact_replay_sample = deepcopy(exc.contact_replay_sample)
            _emit_contact_replay_failure(failure_sink, error, retimed, hold, name, 'grasp')
            raise error from exc
        evidence[name+'_trajectory_sha256'] = wire_digest(retimed)
        evidence[name+'_geometry'] = report
        remaining()
    evidence['collision_state_checks'] = collision_check(shifted, state, deadline_check=remaining)
    remaining()
    return evidence


def validate_selected_pregrasp_path(path, fk, sample, scene, gripper, opening,
                                   planner, state, collision_check, remaining, *,
                                   command_path_validator=None, failure_sink=None, replay_stage='pregrasp',
                                   following_model=None, scheduled_start_ns=None):
    """Audit command and conditional actual curves, including displaced pose0.

    Preserve the supplied times, velocities and accelerations. In execution
    this is already the final timed curve; retiming it here would audit a
    different spline even if every waypoint position remained identical.
    The command curve starts at the current SDK reference; subtracting the
    current measured bias therefore starts the predicted curve at the actual
    accepted feedback. Neither curve establishes future physical following.
    """
    names = tuple(path.joint_trajectory.joint_names)
    if len(names) != 6 or set(names) != set(ARM_NAMES) or not path.joint_trajectory.points:
        raise ContactPrecheckError('HELD_PREGRASP_PATH_JOINTS_INVALID')
    if following_model is not None:
        validate_source_bias_model(following_model, sample)
        bias = following_offset(sample['sdk_positions_rad'], sample['accepted_positions_rad'],
                                following_model=following_model)
    else:
        bias = np.asarray(sample['sdk_positions_rad'])-np.asarray(sample['accepted_positions_rad'])
    offset = dict(zip(ARM_NAMES, bias))
    sdk = dict(zip(ARM_NAMES, sample['sdk_positions_rad']))
    if max(abs(path.joint_trajectory.points[0].positions[i]-sdk[n])
           for i,n in enumerate(names)) > 1e-9:
        raise ContactPrecheckError('HELD_PREGRASP_COMMAND_START_CHANGED')
    actual_path = deepcopy(path)
    for point in actual_path.joint_trajectory.points:
        point.positions = [v-offset[n] for n,v in zip(names, point.positions)]
    evidence = dict(scope='constant_bias_full_pregrasp_curves', execution_authorized=False,
                    future_following_verified=False)
    if following_model is not None:
        evidence['following_reference'] = following_evidence(sample['sdk_positions_rad'],
            sample['accepted_positions_rad'], following_model=following_model)
    for label, original in (('command', path), ('predicted_actual', actual_path)):
        remaining()
        timed = deepcopy(original)
        command_bounds = (command_path_validator(timed)
                          if label == 'command' and command_path_validator is not None else None)
        hold = JointTrajectoryPoint(positions=list(timed.joint_trajectory.points[0].positions),
                                    velocities=[0.]*6, accelerations=[0.]*6)
        try:
            schedule = ({} if scheduled_start_ns is None else
                        dict(scheduled_start_ns=scheduled_start_ns))
            report = validate_observation_trajectory(timed, scene, fk, gripper, opening, hold,
                                                    max_seconds=min(10., remaining()), **schedule)
        except ObservationPathError as exc:
            _emit_contact_replay_failure(failure_sink, exc, timed, hold, label, replay_stage)
            raise
        curve_state = deepcopy(state)
        values = dict(zip(names, timed.joint_trajectory.points[0].positions))
        curve_state.joint_state.position = [values.get(n,v) for n,v in
            zip(curve_state.joint_state.name, curve_state.joint_state.position)]
        evidence[label] = dict(geometry=report, trajectory_sha256=wire_digest(timed),
            collision_state_checks=collision_check(timed, curve_state, deadline_check=remaining))
        if command_bounds is not None:
            evidence[label]['sdk_path_bounds'] = command_bounds
        remaining()
    return evidence


def validate_held_guide_path_bounds(path, initial_command, target_command):
    """Check the same finite SDK box and held cells on every spline segment."""
    names = tuple(path.joint_trajectory.joint_names)
    points = list(path.joint_trajectory.points)
    if len(names) != 6 or set(names) != set(ARM_NAMES) or len(points) < 2:
        raise ContactPrecheckError('HELD_APPROACH_PATH_INVALID')
    initial, target = dict(zip(ARM_NAMES, initial_command)), dict(zip(ARM_NAMES, target_command))
    origin = np.asarray([initial[n] for n in names])
    if any(abs(target[ARM_NAMES[i]]-initial[ARM_NAMES[i]]) > 1e-12
           for i in PregraspCompensation.HELD_AXES):
        raise ContactPrecheckError('HELD_APPROACH_PATH_MOVES_HELD_AXIS')
    if (max(abs(points[0].positions[i]-initial[n]) for i,n in enumerate(names)) > 1e-9
            or max(abs(points[-1].positions[i]-target[n]) for i,n in enumerate(names)) > 1e-12):
        raise ContactPrecheckError('HELD_APPROACH_PATH_ENDPOINT_CHANGED')
    positive = next((i for i,p in enumerate(points) if p.time_from_start.to_sec() > 0), None)
    if positive is None:
        raise ContactPrecheckError('HELD_APPROACH_PATH_TIMING_INVALID')
    hold = JointTrajectoryPoint(positions=origin.tolist(), velocities=[0.]*6, accelerations=[0.]*6)
    pairs = [(hold, points[positive], points[positive].time_from_start.to_sec())]
    pairs.extend((left, right, (right.time_from_start-left.time_from_start).to_sec())
                 for left,right in zip(points[positive:], points[positive+1:]))
    for left,right,duration in pairs:
        if duration <= 0:
            raise ContactPrecheckError('HELD_APPROACH_PATH_TIMING_INVALID')
        coefficients = segment_coefficients(left, right, duration, 6)
        low, high = _polynomial_bounds(coefficients, 0., 1.)
        cap = PregraspCompensation.MAX_TOTAL_COUNTS*Q
        if np.any(low < origin-cap-1e-9) or np.any(high > origin+cap+1e-9):
            raise ContactPrecheckError('HELD_APPROACH_PATH_LEAVES_TOTAL_COUNT_BOX')
        for name in ARM_NAMES:
            if target[name] != initial[name]:
                continue
            i = names.index(name)
            if low[i] < origin[i]-Q/2 or high[i] >= origin[i]+Q/2:
                raise ContactPrecheckError('HELD_APPROACH_PATH_MOVES_HELD_AXIS')
    return dict(maximum_total_counts=PregraspCompensation.MAX_TOTAL_COUNTS,
                held_joints=['Joint2', 'Joint5'],
                complete_spline_checked=True, execution_authorized=False)


class ContactSequencePrecheckGateway:
    def _contact_precheck_sample(self, fk, epoch, *, remaining=None,
                                 audit=None, selection=None):
        """Wait only for a new independent SDK window, never manufacture one.

        This is still read-only candidate evidence. The original validator,
        source-age bound and epoch stay unchanged. No deadline or source stamp
        is refreshed by polling; each retry needs progress in both real feeds.
        """
        started = time.monotonic()
        wait_deadline = None
        last_attempt_sources = None
        retry_reason = 'missing independent sdk_transmitted samples'
        evidence = dict(scope='read_only_feedback_collection', maximum_wait_sec=.5,
                        epoch_ns=epoch, attempts=0, waited=False)
        if audit is not None:
            audit.setdefault('following_sample_acquisition', []).append(evidence)

        def summary(messages):
            stamps = [int(msg.header.stamp.to_nsec()) for msg in messages]
            return dict(history_count=len(stamps),
                        first_source_ns=stamps[0] if stamps else None,
                        last_source_ns=stamps[-1] if stamps else None,
                        maximum_source_gap_sec=max(
                            [(b-a)*1e-9 for a,b in zip(stamps, stamps[1:])],
                            default=0.))

        def context():
            if epoch != getattr(self, '_control_reference_epoch_ns', None):
                raise ContactPrecheckError('control reference epoch changed while collecting feedback')
            if rospy.is_shutdown():
                raise ContactPrecheckError('gateway shutdown while collecting feedback')
            return remaining() if remaining is not None else float('inf')

        def wait_owner(authoritative=False):
            # The healthy fast path keeps its original read-only semantics,
            # including use from manual preview. Only this optional wait needs
            # an unchanged automatic owner. Cached polling avoids repeated RPC
            # inside the short budget; entry/return still query the real owner.
            get_param = rospy.get_param if authoritative else rospy.get_param_cached
            manual = (self._manual_control_active() if authoritative else
                (getattr(self, '_gui_direct_mode', False) or
                 get_param('/gui/joint_direct_mode', False) is True))
            if manual:
                raise ContactPrecheckError('manual control active while collecting feedback')
            if (selection is not None and
                    selection != get_param('/grasp_mode/selection', {})):
                raise ContactPrecheckError('candidate selection changed while collecting feedback')

        try:
            while True:
                with self._endpoint_feedback_lock:
                    sdk = tuple(self._endpoint_feedback_history['sdk'])
                    accepted = tuple(self._endpoint_feedback_history['accepted'])
                snapshot = dict(sdk=summary(sdk), accepted=summary(accepted))
                evidence['final_snapshot'] = snapshot
                evidence.setdefault('initial_snapshot', snapshot)
                budget = context()
                if wait_deadline is not None:
                    wait_owner()
                    budget = context()
                if wait_deadline is not None and time.monotonic() >= wait_deadline:
                    evidence['wait_exhausted'] = True
                    raise ValueError(retry_reason)
                now = rospy.get_time()
                sources = (snapshot['sdk']['last_source_ns'],
                           snapshot['accepted']['last_source_ns'])
                if last_attempt_sources is not None:
                    # Inspect malformed/new stale inputs even when the other
                    # feed has not advanced. The wait cannot hide bad data.
                    _window(sdk, 'sdk_transmitted', ARM_NAMES, now, epoch, .5)
                    _window(accepted, 'sdk_measured', ARM_NAMES+('right_finger',),
                            now, epoch, .5)
                advanced = (last_attempt_sources is None or all(
                    new is not None and old is not None and new > old
                    for new,old in zip(sources, last_attempt_sources)))
                if advanced:
                    evidence['attempts'] += 1
                    try:
                        sample = stationary_following_error(fk, sdk, accepted,
                            now_sec=now, epoch_ns=epoch)
                    except ValueError as exc:
                        if str(exc) != retry_reason or remaining is None:
                            raise
                        if wait_deadline is None:
                            wait_deadline = time.monotonic()+1.
                            wait_owner(authoritative=True)
                            context()
                        last_attempt_sources = sources
                        evidence.update(waited=True, initial_retry_reason=retry_reason)
                    else:
                        # FK/ROS lookups may consume time or race a new epoch.
                        # Recheck both clocks and source freshness at return.
                        context()
                        if wait_deadline is not None:
                            wait_owner(authoritative=True)
                            context()
                        if wait_deadline is not None and time.monotonic() >= wait_deadline:
                            evidence['wait_exhausted'] = True
                            raise ValueError(retry_reason)
                        fresh_now = rospy.get_time()
                        _window(sdk, 'sdk_transmitted', ARM_NAMES, fresh_now, epoch, .5)
                        _window(accepted, 'sdk_measured', ARM_NAMES+('right_finger',),
                                fresh_now, epoch, .5)
                        evidence.update(success=True,
                            recovered=last_attempt_sources is not None)
                        return sample
                # Feedback callbacks use _endpoint_feedback_lock; never sleep
                # with it held. The enclosing planner lock remains deliberate
                # serialization and is not used by either source callback.
                budget = context()
                delay = min(.02, budget, wait_deadline-time.monotonic())
                if delay > 0.:
                    time.sleep(delay)
        except Exception as exc:
            evidence.update(success=False, reason=str(exc))
            raise
        finally:
            evidence['elapsed_sec'] = max(0., time.monotonic()-started)

    def _contact_precheck_scene_digest(self):
        # Bind reuse to collision geometry, attached bodies, transforms, ACM
        # and padding. Robot joint feedback is bound separately below.
        components = PlanningSceneComponents()
        components.components = (components.SCENE_SETTINGS | components.ROBOT_STATE_ATTACHED_OBJECTS
            | components.WORLD_OBJECT_GEOMETRY | components.OCTOMAP | components.TRANSFORMS
            | components.ALLOWED_COLLISION_MATRIX | components.LINK_PADDING_AND_SCALING)
        rospy.wait_for_service('/get_planning_scene', timeout=.5)
        scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)(
            GetPlanningSceneRequest(components=components)).scene
        return wire_digest(scene)

    def handle_contact_sequence_precheck(self, req):
        audit = dict(scope='read_only_candidate_constant_bias_prediction',
                     execution_authorized=False, hardware_response_verified=False,
                     real_grasp_success=False, plan_id=str(req.plan.plan_id))
        metrics = {}
        failed_stage = ''
        with self._planner_operation_lock():
            try:
                plan = deepcopy(req.plan)
                if plan.model_choice != 'unknown_tabletop':
                    raise ContactPrecheckError('unknown contact plan required')
                scene = FrozenObservationScene.from_contact_plan(plan)
                now = rospy.get_time()
                requested = req.deadline.to_sec()
                if not math.isfinite(requested) or requested < 0.:
                    raise ContactPrecheckError('invalid candidate deadline')
                deadline = min(requested, now+30.) if requested > 0. else now+30.
                wall_deadline = time.monotonic() + (deadline-now)
                def remaining():
                    value = min(deadline-rospy.get_time(), wall_deadline-time.monotonic())
                    if not math.isfinite(value) or value <= 0.:
                        raise ContactPrecheckError('candidate precheck deadline consumed', 'MOVEIT_TIMEOUT')
                    return value
                remaining()
                epoch = getattr(self, '_control_reference_epoch_ns', None)
                selection = deepcopy(rospy.get_param('/grasp_mode/selection', {}))
                model = rospy.get_param('/robot_description')
                geometry = deepcopy(rospy.get_param('/grasp_6d/remote/gripper_geometry', {}))
                config = deepcopy(rospy.get_param('/grasp', {}))
                gripper_config = deepcopy(rospy.get_param('/gripper', {}))
                if selection.get('mode') != 'unknown':
                    raise ContactPrecheckError('unknown selection required')
                if not config.get('unknown_contact_cartesian_compensation_enabled', False):
                    raise ContactPrecheckError('contact compensation is not enabled')
                fk = SerialUrdfFk(model, ARM_NAMES)
                sample = self._contact_precheck_sample(
                    fk, epoch, remaining=remaining, audit=audit, selection=selection)
                audit.update(epoch_ns=epoch, model_sha256=fk.model_sha256,
                             selection=selection, source_sample=sample)
                g = contact_precheck_gripper(geometry)
                opened = float(gripper_config.get('open_position_m', .05))
                tolerance = float(gripper_config.get('open_skip_tolerance_m', .001))
                measured_opening = float(sample['accepted_gripper_opening_m'])
                if (not math.isfinite(opened) or not 0. < opened <= .05
                        or not math.isfinite(tolerance) or not 0. <= tolerance < opened):
                    raise ContactPrecheckError('invalid configured open posture')
                # Match the existing open/skip decision. Any future opening is
                # marked predicted; execution must still use fresh encoders.
                opening = measured_opening if abs(measured_opening-opened) <= tolerance else opened
                audit.update(opening_m=opening, opening_source=(
                    'current_accepted_open_posture' if opening == measured_opening
                    else 'configured_future_open_posture_prediction'))
                replay_sink = _contact_replay_failure_sink(audit, plan, scene, model, epoch,
                    selection, sample, geometry, config, gripper_config, g, opening,
                    audit['opening_source'])
                planner = self._ensure_planner()
                if planner is None:
                    raise ContactPrecheckError(self._moveit_not_ready_message(), 'MOVEIT_CHECK_ERROR')
                start = deepcopy(planner.robot.get_current_state())
                if not set(ARM_NAMES+('right_finger',)).issubset(start.joint_state.name):
                    raise ContactPrecheckError('incomplete current robot state')
                values = dict(zip(ARM_NAMES, sample['accepted_positions_rad']), right_finger=opening)
                start.joint_state.position = [values.get(n, v) for n, v in
                    zip(start.joint_state.name, start.joint_state.position)]
                start.is_diff = False
                names = ('pregrasp', 'approach', 'grasp', 'lift')
                if config.get('unknown_lift_after_close_enabled', True) is False:
                    names = names[:3]
                audit['checked_stage_names'] = list(names)
                targets = []
                for pose in plan.poses[:len(names)]:
                    target = PoseStamped(); target.header.frame_id = 'base_link'
                    target.pose = deepcopy(pose); targets.append(target)
                held_approach = config.get('unknown_approach_held_axes_feedforward_enabled', False)
                if type(held_approach) is not bool:
                    raise ContactPrecheckError('PREGRASP_HELD_AXES_CONFIG_INVALID')
                pregrasp_prediction_path = []
                held_paths = {}
                lookahead = (held_approach is True and all(config.get(key) is True for key in (
                    'pregrasp_cartesian_compensation_enabled', 'unknown_pregrasp_feedforward_enabled',
                    'unknown_pregrasp_held_axes_feedforward_enabled')))
                def plan_pregrasp(state, target):
                    nominal, reason = planner._plan_pose_from_start_state(state, target.pose,
                        planning_time_limit_sec=remaining())
                    if nominal is None:
                        return None, reason
                    tr = nominal.joint_trajectory
                    if len(tr.joint_names) != 6 or set(tr.joint_names) != set(ARM_NAMES) or not tr.points:
                        raise ContactPrecheckError('HELD_PREGRASP_NOMINAL_PATH_INVALID')
                    mapping = dict(zip(tr.joint_names, tr.points[-1].positions))
                    try:
                        selected, report = select_held_pregrasp_endpoint(fk, sample,
                            pose_values(plan.poses[0]), pose_values(plan.poses[1]),
                            [mapping[n] for n in ARM_NAMES],
                            position_tolerance_m=measured_position_tolerance(config, 'unknown_tabletop'),
                            orientation_tolerance_rad=math.radians(float(config.get(
                                'measured_endpoint_orientation_tolerance_deg', 5.))), deadline_check=remaining)
                    except HeldAxesPredictionError as exc:
                        # The generic sequence wrapper preserves its existing
                        # rejection classification. Retain the typed cause and
                        # numerical inputs here before it flattens exceptions.
                        failure = deepcopy(exc.evidence)
                        audit['held_pregrasp_selection_failure'] = dict(
                            code=str(exc), callback_stage='pregrasp', evidence=failure,
                            nominal_pregrasp_command_rad=[mapping[n] for n in ARM_NAMES],
                            fixed_pregrasp_goal=pose_values(plan.poses[0]),
                            fixed_approach_goal=pose_values(plan.poses[1]))
                        if 'pregrasp_endpoint_selection' in failure:
                            audit['held_pregrasp_selection'] = deepcopy(failure['pregrasp_endpoint_selection'])
                        raise
                    audit['held_pregrasp_selection'] = report
                    command_start = deepcopy(state)
                    values = dict(zip(ARM_NAMES, sample['sdk_positions_rad']))
                    command_start.joint_state.position = [values.get(n,v) for n,v in
                        zip(command_start.joint_state.name, command_start.joint_state.position)]
                    command_path, reason = planner._plan_joint_from_start_state(command_start,
                        selected, planning_time_limit_sec=remaining())
                    if command_path is None:
                        return None, reason
                    report['full_path'] = validate_selected_pregrasp_path(command_path, fk, sample,
                        scene, g, opening, planner, command_start,
                        self._contact_command_collision_check, remaining, failure_sink=replay_sink)
                    return command_path, reason
                def plan_approach(state, target):
                    # Certify the actual held policy before selecting this
                    # stage's command path, rather than planning ordinary IK
                    # first and accidentally advancing from its unrelated q.
                    validate('approach', state, None, target)
                    if 'guide_path_prediction' in held_paths:
                        return held_paths['guide_path_prediction'], 'conditional held approach command path'
                    # An already converged approach sends no new command.
                    # Retain the old conservative nominal reachability check,
                    # but never use its endpoint as the predicted actual state.
                    return planner._plan_cartesian_from_start_state(state, target)
                def actual_stage_state(stage, state):
                    if stage == 'pregrasp':
                        prediction = audit['held_pregrasp_selection']['predicted_pregrasp']
                    elif stage == 'approach':
                        prediction = audit['held_approach_prediction']['predicted_approach']
                    else:
                        return state
                    values = dict(zip(ARM_NAMES, prediction['predicted_positions_rad']))
                    result = deepcopy(state)
                    result.joint_state.position = [values.get(n,v) for n,v in
                        zip(result.joint_state.name, result.joint_state.position)]
                    result.joint_state.velocity = []
                    result.joint_state.effort = []
                    audit.setdefault('stage_start_prediction', {})[stage] = dict(
                        predicted_actual_joint_positions_rad=prediction['predicted_positions_rad'],
                        predicted_command_joint_positions_rad=prediction['predicted_command_positions_rad'],
                        scope='conditional_next_stage_actual_start', future_following_verified=False,
                        execution_authorized=False)
                    return result
                def validate(stage, state, path, target):
                    remaining()
                    if held_approach and stage == 'pregrasp':
                        pregrasp_prediction_path[:] = [deepcopy(path)]
                    if (held_approach and stage == 'approach'
                            and (not lookahead or 'held_approach_prediction' not in audit)):
                        if not pregrasp_prediction_path:
                            raise ContactPrecheckError('held approach pregrasp validator was not executed')
                        audit['held_approach_prediction'] = {}
                        predict_held_approach(pregrasp_prediction_path[0], fk, sample,
                            plan.poses[0], target.pose, config, audit['held_approach_prediction'])
                        if lookahead:
                            prediction = audit['held_approach_prediction']
                            pre = prediction['predicted_pregrasp']
                            end = prediction['predicted_approach']
                            segments = []
                            if pre['steps_predicted']:
                                original = prediction['nominal_pregrasp_command_rad']
                                offset = np.asarray(prediction['sdk_minus_encoder_rad'])
                                segments.append(('pose0_correction_path_prediction', original,
                                    (np.asarray(original)-offset).tolist(), pre['predicted_command_positions_rad']))
                            if end['steps_predicted']:
                                segments.append(('guide_path_prediction', pre['predicted_command_positions_rad'],
                                    pre['predicted_positions_rad'], end['predicted_command_positions_rad']))
                            for key, begin_sdk, begin_actual, final_sdk in segments:
                                guide_start = deepcopy(state)
                                values = dict(zip(ARM_NAMES, begin_sdk))
                                guide_start.joint_state.position = [values.get(n,v) for n,v in
                                    zip(guide_start.joint_state.name, guide_start.joint_state.position)]
                                guide, reason = planner._plan_joint_from_start_state(guide_start,
                                    final_sdk, planning_time_limit_sec=remaining())
                                if guide is None:
                                    raise ContactPrecheckError('held approach path prediction: '+reason)
                                bounds = validate_held_guide_path_bounds(guide, begin_sdk, final_sdk)
                                # Explicit virtual positions only: no stamps,
                                # epoch or fake measured-following receipt.
                                projection = dict(sdk_positions_rad=begin_sdk,
                                    accepted_positions_rad=begin_actual)
                                curves = validate_selected_pregrasp_path(guide, fk, projection, scene,
                                    g, opening, planner, guide_start,
                                    self._contact_command_collision_check, remaining,
                                    command_path_validator=lambda path: validate_held_guide_path_bounds(path,
                                        begin_sdk, final_sdk), failure_sink=replay_sink,
                                    replay_stage='approach')
                                prediction[key] = dict(bounds=bounds, curves=curves,
                                    scope='conditional_future_held_correction_curve',
                                    future_following_verified=False, execution_authorized=False)
                                held_paths[key] = guide
                    if stage == 'grasp':
                        audit['contact_prediction'] = validate_contact_prediction(
                            path, fk, sample, target.pose, scene, g, opening, planner, state,
                            self._contact_command_collision_check, remaining, failure_sink=replay_sink)
                scene_digest = self._contact_precheck_scene_digest()
                remaining()
                # Publication rebuilds the same selected candidate. Reuse only
                # an exact success while both independent feedback channels,
                # opening, epoch, frozen target and MoveIt scene are unchanged.
                # This cache stores predictions, never executable trajectories.
                cache_key = json.dumps(dict(plan_digest=scene.digest, model=fk.model_sha256,
                    epoch=epoch, selection=selection, geometry=geometry, config=config,
                    gripper_config=gripper_config, sdk=sample['sdk_positions_rad'],
                    accepted=sample['accepted_positions_rad'], opening=measured_opening,
                    scene_digest=scene_digest), sort_keys=True, allow_nan=False)
                cache = getattr(self, '_contact_precheck_cache', {})
                entry = cache.get(cache_key)
                cached = bool(entry and 0. <= time.monotonic()-entry['time'] <= 30.
                    and (not held_approach or entry.get('held_approach_prediction', {}).get('success') is True)
                    and (not lookahead or entry.get('held_pregrasp_selection', {}).get('full_path')))
                if cached:
                    metrics = deepcopy(entry['metrics'])
                    message = entry['message']
                    audit['contact_prediction'] = deepcopy(entry['prediction'])
                    if held_approach:
                        audit['held_approach_prediction'] = deepcopy(entry['held_approach_prediction'])
                    if lookahead:
                        audit['held_pregrasp_selection'] = deepcopy(entry['held_pregrasp_selection'])
                        audit['stage_start_prediction'] = deepcopy(entry['stage_start_prediction'])
                else:
                    profile = self._unknown_pregrasp_velocity_limits(planner)
                    with temporary_attribute(planner, 'unknown_pregrasp_joint_velocity_limits_rad_s', profile):
                        ok, code, failed_stage, metrics, message = planner.check_pose_sequence(
                            targets, names, [False]+[True]*(len(names)-1), deadline_sec=deadline,
                            stage_validator=validate, start_state_override=start,
                            **(dict(pregrasp_planner=plan_pregrasp, approach_planner=plan_approach,
                                    stage_state_transform=actual_stage_state) if lookahead else {}))
                    if not ok:
                        raise ContactPrecheckError(message, code)
                audit.update(prediction_cache_hit=cached, planning_scene_digest=scene_digest)
                if 'contact_prediction' not in audit:
                    raise ContactPrecheckError('contact validator was not executed')
                if held_approach and audit.get('held_approach_prediction', {}).get('success') is not True:
                    raise ContactPrecheckError('held approach validator was not executed')
                remaining()
                current = self._contact_precheck_sample(
                    fk, epoch, remaining=remaining, audit=audit, selection=selection)
                if (epoch != getattr(self, '_control_reference_epoch_ns', None)
                        or selection != rospy.get_param('/grasp_mode/selection', {})
                        or model != rospy.get_param('/robot_description')
                        or geometry != rospy.get_param('/grasp_6d/remote/gripper_geometry', {})
                        or config != rospy.get_param('/grasp', {})
                        or gripper_config != rospy.get_param('/gripper', {})):
                    raise ContactPrecheckError('candidate context changed while planning')
                for key, tolerance in (('sdk_positions_rad', Q/2), ('accepted_positions_rad', Q)):
                    if max(abs(a-b) for a,b in zip(current[key], sample[key])) > tolerance+1e-12:
                        raise ContactPrecheckError('following feedback changed while planning')
                if abs(current['accepted_gripper_opening_m']-measured_opening) > 1e-9:
                    raise ContactPrecheckError('gripper changed while planning')
                remaining()
                if self._contact_precheck_scene_digest() != scene_digest:
                    raise ContactPrecheckError('MoveIt scene changed while planning')
                remaining()
                if not cached:
                    cache[cache_key] = dict(time=time.monotonic(), metrics=deepcopy(metrics),
                        message=message, prediction=deepcopy(audit['contact_prediction']))
                    if held_approach:
                        cache[cache_key]['held_approach_prediction'] = deepcopy(audit['held_approach_prediction'])
                    if lookahead:
                        cache[cache_key]['held_pregrasp_selection'] = deepcopy(audit['held_pregrasp_selection'])
                        cache[cache_key]['stage_start_prediction'] = deepcopy(audit['stage_start_prediction'])
                    while len(cache) > 16:
                        del cache[next(iter(cache))]
                    self._contact_precheck_cache = cache
                audit.update(success=True, completion_sample=current)
                return CheckContactSequenceResponse(True, '', '', metrics['path_cost'],
                    metrics['max_delta'], message, json.dumps(audit, sort_keys=True))
            except Exception as exc:
                code = str(getattr(exc, 'code', 'CONTACT_SEQUENCE_PRECHECK_FAILED'))
                audit.update(success=False, failure_code=code, failed_stage=failed_stage, reason=str(exc))
                return CheckContactSequenceResponse(False, code, failed_stage,
                    float(metrics.get('path_cost', 0.)), float(metrics.get('max_delta', 0.)),
                    str(exc), json.dumps(audit, sort_keys=True))
