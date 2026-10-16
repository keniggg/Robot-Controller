"""Preserve measured servo following offsets across short contact motions.

The visual goal and nominal Cartesian path stay fixed. Only the actuator path
receives the freshly observed SDK-minus-encoder offset. This is feedforward,
not proof of hardware response; the unchanged measured endpoint gate decides
arrival after each segment. Manual motion invalidates the whole request.
"""
from copy import deepcopy
import json
import math
import time

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest

from alicia_flexible_grasp_supervisor.srv import CompensatePregraspResponse
from alicia_flexible_grasp.grasp.rich_plan_integrity import pose_values
from alicia_flexible_grasp.grasp.gripper_geometry import (
    GripperGeometry, gripper_contract_mismatch_reason, ANALYTICAL_GRIPPER_MODEL_NAME)
from .observation_path_guard import (
    ObservationPathError, SerialUrdfFk, committed_plan_digest,
    validate_observation_trajectory, wire_digest,
)
from .stationary_following import ARM_NAMES, SDK_QUANTUM_RAD as Q
from .pregrasp_compensation import pose_matrix


def compensate_cartesian_path(nominal, fk, sample, goal):
    """Translate a complete nominal joint path by one bounded, frozen offset."""
    names = tuple(nominal.joint_trajectory.joint_names)
    points = nominal.joint_trajectory.points
    if (sample.get('model_sha256') != fk.model_sha256
            or tuple(sample.get('joint_names', ())) != ARM_NAMES
            or names != ARM_NAMES or len(points) < 2
            or getattr(nominal.multi_dof_joint_trajectory, 'points', ())):
        raise ObservationPathError('CONTACT_LINEAR_PATH_CONTEXT_INVALID')
    sdk = np.asarray(sample['sdk_positions_rad'], dtype=float)
    measured = np.asarray(sample['accepted_positions_rad'], dtype=float)
    if (sdk.shape != (6,) or measured.shape != (6,)
            or not np.all(np.isfinite(sdk)) or not np.all(np.isfinite(measured))):
        raise ObservationPathError('CONTACT_LINEAR_FEEDBACK_INVALID')
    offset = sdk-measured
    if np.max(np.abs(offset)) > .035:
        raise ObservationPathError('CONTACT_LINEAR_FOLLOWING_OUTSIDE_CONTRACT')
    positions = np.asarray([p.positions for p in points], dtype=float)
    if positions.shape != (len(points), 6) or not np.all(np.isfinite(positions)):
        raise ObservationPathError('CONTACT_LINEAR_WAYPOINT_INVALID')
    if np.max(np.abs(positions[0]-measured)) > 1e-9:
        raise ObservationPathError('CONTACT_LINEAR_START_NOT_MEASURED')
    goal_matrix = pose_matrix(pose_values(goal))
    end_matrix = fk(positions[-1])
    angle = math.acos(float(np.clip((np.trace(goal_matrix[:3,:3].T @ end_matrix[:3,:3])-1)/2, -1., 1.)))
    if np.linalg.norm(goal_matrix[:3,3]-end_matrix[:3,3]) > .0005 or angle > .005:
        raise ObservationPathError('CONTACT_LINEAR_NOMINAL_GOAL_CHANGED')
    shifted = positions+offset
    if np.any(shifted < fk.limits[:,0]) or np.any(shifted > fk.limits[:,1]):
        raise ObservationPathError('CONTACT_LINEAR_COMPENSATED_JOINT_LIMIT')
    result = deepcopy(nominal)
    for p, q in zip(result.joint_trajectory.points, shifted):
        p.positions = q.tolist()
    evidence = dict(sdk_minus_encoder_rad=offset.tolist(),
        measured_start_positions_rad=measured.tolist(),
        command_start_positions_rad=sdk.tolist(),
        fixed_goal_pose=list(pose_values(goal)),
        nominal_trajectory_sha256=wire_digest(nominal),
        compensated_trajectory_sha256=wire_digest(result),
        hardware_response_verified=False, real_grasp_success=False)
    return result, evidence


def _diagnostic_json_value(value):
    """Keep an exception audit serializable without altering validation."""
    if isinstance(value, dict):
        return {str(key): _diagnostic_json_value(item) for key, item in value.items()}
    if isinstance(value, (tuple, list, np.ndarray)):
        return [_diagnostic_json_value(item) for item in value]
    if isinstance(value, np.generic):
        return _diagnostic_json_value(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    if value is None or isinstance(value, (str, bool, int, float)):
        return value
    return repr(value)


def contact_failure_following(initial, final, *, epoch, model_sha256,
                              after_ns, now_sec):
    """Describe measured endpoints; never authorize recovery or infer cause.

    Require a complete stationary window after the failed execution. The
    initial sample is historical evidence, never a substitute for that window.
    Four transmitted counts are enough to diagnose fewer than two measured
    counts, matching the existing bounded-step response scale.
    """
    if (type(epoch) is not int or epoch <= 0 or type(after_ns) is not int
            or after_ns < epoch or not math.isfinite(now_sec)):
        raise ObservationPathError('CONTACT_FAILURE_DIAGNOSTIC_CLOCK_INVALID')
    positions = []
    for sample in (initial, final):
        if (sample.get('measurement_kind') != 'stationary_sdk_following_not_executed_endpoint'
                or sample.get('model_sha256') != model_sha256
                or sample.get('epoch_ns') != epoch
                or tuple(sample.get('joint_names', ())) != ARM_NAMES):
            raise ObservationPathError('CONTACT_FAILURE_DIAGNOSTIC_CONTEXT_CHANGED')
        stamps = [sample.get(key) for key in ('sdk_stamp_ns', 'accepted_stamp_ns',
                    'stationary_window_start_ns', 'stationary_window_end_ns')]
        if (any(type(value) is not int or value < epoch for value in stamps)
                or stamps[3]-stamps[2] < 300_000_000
                or min(stamps[:2]) < stamps[3]):
            raise ObservationPathError('CONTACT_FAILURE_DIAGNOSTIC_SAMPLE_INVALID')
        sdk = np.asarray(sample['sdk_positions_rad'], dtype=float)
        measured = np.asarray(sample['accepted_positions_rad'], dtype=float)
        if (sdk.shape != (6,) or measured.shape != (6,)
                or not np.all(np.isfinite(sdk)) or not np.all(np.isfinite(measured))):
            raise ObservationPathError('CONTACT_FAILURE_DIAGNOSTIC_POSITIONS_INVALID')
        positions.append((sdk, measured))
    if (final['stationary_window_start_ns'] <= after_ns
            or any(final[key] <= initial[key] or not 0. <= now_sec-final[key]*1e-9 <= .5
                   for key in ('sdk_stamp_ns', 'accepted_stamp_ns'))
            or any(initial[key] > after_ns for key in ('sdk_stamp_ns', 'accepted_stamp_ns'))):
        raise ObservationPathError('CONTACT_FAILURE_DIAGNOSTIC_NOT_FRESH_POST_FAILURE')
    (initial_sdk, initial_measured), (sdk, measured) = positions
    command_delta, measured_delta = sdk-initial_sdk, measured-initial_measured
    command_counts = (np.rint(sdk/Q)-np.rint(initial_sdk/Q)).astype(int)
    measured_counts = (np.rint(measured/Q)-np.rint(initial_measured/Q)).astype(int)
    initial_following, following = initial_sdk-initial_measured, sdk-measured
    rows = []
    for i, name in enumerate(ARM_NAMES):
        rows.append(dict(joint_name=name,
            transmitted_command_delta_rad=float(command_delta[i]),
            transmitted_command_delta_counts=int(command_counts[i]),
            measured_delta_rad=float(measured_delta[i]),
            measured_delta_counts=int(measured_counts[i]),
            initial_sdk_minus_encoder_rad=float(initial_following[i]),
            final_sdk_minus_encoder_rad=float(following[i]),
            initial_sdk_minus_encoder_counts=float(initial_following[i]/Q),
            final_sdk_minus_encoder_counts=float(following[i]/Q),
            following_outside_original_contract=bool(abs(following[i]) > .035),
            no_measured_response=bool(abs(command_counts[i]) >= 4 and abs(measured_counts[i]) < 2)))
    return dict(status='available', epoch_ns=epoch,
        model_sha256=model_sha256, failure_return_stamp_ns=after_ns,
        initial_sdk_stamp_ns=initial['sdk_stamp_ns'],
        initial_accepted_stamp_ns=initial['accepted_stamp_ns'],
        sdk_stamp_ns=final['sdk_stamp_ns'], accepted_stamp_ns=final['accepted_stamp_ns'],
        stationary_window_start_ns=final['stationary_window_start_ns'],
        stationary_window_end_ns=final['stationary_window_end_ns'],
        original_following_limit_rad=.035,
        maximum_initial_following_error_rad=float(np.max(abs(initial_following))),
        maximum_final_following_error_rad=float(np.max(abs(following))),
        following_outside_original_contract=bool(np.max(abs(following)) > .035),
        minimum_command_delta_counts=4, minimum_measured_response_counts=2,
        measurement_scope='stationary_endpoint_deltas',
        continuous_motion_history_checked=False, joints=rows)


class ContactLinearCompensationGateway:
    def handle_contact_approach(self, req):
        return self._handle_contact_linear(req, 1)

    def handle_contact_grasp_pose(self, req):
        return self._handle_contact_linear(req, 2)

    def _contact_command_collision_check(self, trajectory, start_state, *, deadline_check=None):
        """Recheck the shifted actuator curve; the nominal path had MoveIt IK checks."""
        if deadline_check is not None:
            deadline_check()
        rospy.wait_for_service('/check_state_validity', timeout=2.)
        check = rospy.ServiceProxy('/check_state_validity', GetStateValidity)
        state = deepcopy(start_state)
        indices = [list(state.joint_state.name).index(n) for n in ARM_NAMES]
        points = trajectory.joint_trajectory.points
        checks = 0
        for first, second in zip(points, points[1:]):
            a, b = np.asarray(first.positions), np.asarray(second.positions)
            count = max(1, int(math.ceil(float(np.max(np.abs(b-a)))/.002)))
            if checks+count+1 > 4096:
                raise ObservationPathError('CONTACT_LINEAR_COLLISION_CHECK_BUDGET')
            for q in np.linspace(a, b, count+1):
                if deadline_check is not None:
                    deadline_check()
                positions = list(state.joint_state.position)
                for i, value in zip(indices, q):
                    positions[i] = float(value)
                state.joint_state.position = positions
                request = GetStateValidityRequest(robot_state=state, group_name=self.manipulator_group)
                if not check(request).valid:
                    raise ObservationPathError('CONTACT_LINEAR_COMPENSATED_PATH_COLLISION')
                checks += 1
                if deadline_check is not None:
                    deadline_check()
        return checks

    def _release_contact_visual_context(self, context):
        if context is None:
            return
        with self._observation_context_lock:
            if getattr(self, '_compensation_visual_context', None) is context:
                self._compensation_visual_context = None

    def _contact_failure_following_diagnostic(self, context, initial, fk, record):
        report = dict(plan_id=record['plan_id'], goal_index=record['goal_index'],
            trajectory_sha256=record['trajectory_sha256'], diagnostic_only=True,
            motion_authorized=False, recovery_authorized=False,
            real_grasp_success=False, hardware_cause_determined=False)
        try:
            # Read-only: no stop, sync, reference rewrite, retry or correction.
            after_ns = int(rospy.get_time()*1e9)
            validate = lambda:self._validate_pregrasp_context(context)
            validate()
            sample = self._endpoint_following_snapshot(fk, context['epoch'], context,
                after_ns=after_ns, context_validator=validate)
            validate()
            report.update(contact_failure_following(initial, sample,
                epoch=context['epoch'], model_sha256=fk.model_sha256,
                after_ns=after_ns, now_sec=rospy.get_time()))
        except Exception as exc:
            # Never mask the rejection or reuse a pre-motion snapshot when
            # post-motion evidence is unavailable.
            report.update(status='diagnostic_unavailable', reason=str(exc),
                          exception_type=type(exc).__name__)
        return report

    def _handle_contact_linear(self, req, index):
        with self._planner_operation_lock():
            planner, saved, context = None, {}, None
            diagnostic = dict(plan_id=str(getattr(req.plan, 'plan_id', '') or ''),
                goal_index=index, execute=bool(req.execute),
                validation_stage='request_context', trajectory_sha256='')
            path_guard_failure = None

            def stage(name, trajectory=None):
                diagnostic['validation_stage'] = name
                if trajectory is not None:
                    try:
                        diagnostic['trajectory_sha256'] = wire_digest(trajectory)
                        diagnostic.pop('trajectory_digest_error', None)
                    except Exception as digest_error:
                        # Diagnostic serialization must not turn an otherwise
                        # unchanged validator into a new admission gate.
                        diagnostic['trajectory_sha256'] = ''
                        diagnostic['trajectory_digest_error'] = str(digest_error)

            def failure(exc):
                record = dict(diagnostic, code='CONTACT_LINEAR_COMPENSATION_FAILED',
                              reason=str(exc), exception_type=type(exc).__name__)
                if getattr(exc, 'code', None) is not None:
                    record['exception_code'] = str(exc.code)
                if hasattr(exc, 'audit'):
                    record['exception_audit'] = _diagnostic_json_value(exc.audit)
                return record

            def rejected(record):
                text = json.dumps(record, sort_keys=True, allow_nan=False)
                rospy.logwarn('Contact linear compensation rejected: %s', text)
                return CompensatePregraspResponse(False, text)

            try:
                if not req.execute:
                    previous = getattr(self, '_contact_linear_cache', None)
                    self._contact_linear_cache = None
                    if previous is not None:
                        self._release_contact_visual_context(previous.get('context'))
                    # State publication and service delivery use independent ROS
                    # connections. Wait for this phase rather than reading the
                    # previous latched pregrasp message as an authority change.
                    deadline = time.monotonic()+1.
                    labels = tuple(prefix + ('linear 6D approach' if index == 1
                                             else 'linear 6D grasp pose')
                                   for prefix in ('planning ', 'moving '))
                    while getattr(self, '_compensation_task_label', '') not in labels:
                        if time.monotonic() >= deadline or self._manual_control_active():
                            raise ObservationPathError('CONTACT_LINEAR_TASK_PHASE_UNAVAILABLE')
                        rospy.sleep(.02)
                    context = self._pregrasp_context(deepcopy(req.plan), index)
                else:
                    cached = getattr(self, '_contact_linear_cache', None)
                    self._contact_linear_cache = None  # consume before any submission
                    # Claim the consumed episode before validation so a wrong
                    # index/plan also retires only that episode's visual scope.
                    context = cached.get('context') if cached is not None else None
                    if (cached is None or cached['index'] != index
                            or committed_plan_digest(req.plan) != context['scene'].digest):
                        raise ObservationPathError('CONTACT_LINEAR_CACHE_MISMATCH')
                validate = lambda:self._validate_pregrasp_context(context)
                stage('context_authority')
                validate()
                planner = self._ensure_planner()
                if planner is None:
                    raise ObservationPathError(self._moveit_not_ready_message())
                fk = SerialUrdfFk(context['model'], ARM_NAMES)
                sample = self._endpoint_following_snapshot(fk, context['epoch'], context,
                                                           context_validator=validate)
                goal = PoseStamped()
                goal.header.frame_id = 'base_link'
                goal.pose = deepcopy(context['plan'].poses[index])
                if not req.execute:
                    planner._last_pose_plan = None
                    start = deepcopy(planner.robot.get_current_state())
                    values = dict(zip(ARM_NAMES, sample['accepted_positions_rad']))
                    values['right_finger'] = self._observation_measured_opening()
                    start.joint_state.position = [values.get(n, v) for n, v in
                        zip(start.joint_state.name, start.joint_state.position)]
                    if not set(ARM_NAMES+('right_finger',)).issubset(start.joint_state.name):
                        raise ObservationPathError('CONTACT_LINEAR_START_STATE_INCOMPLETE')
                    start.is_diff = False
                    if np.linalg.norm(fk(sample['accepted_positions_rad'])[:3,3]-
                                      pose_matrix(pose_values(goal.pose))[:3,3]) > .08:
                        raise ObservationPathError('CONTACT_LINEAR_SEGMENT_TOO_LONG')
                    stage('nominal_cartesian_plan')
                    nominal, message = planner._plan_cartesian_from_start_state(start, goal)
                    if nominal is None:
                        raise ObservationPathError(message)
                    stage('compensated_path_construction', nominal)
                    shifted, evidence = compensate_cartesian_path(nominal, fk, sample, goal.pose)
                    stage('compensated_collision', shifted)
                    checks = self._contact_command_collision_check(shifted, start)
                    # Both continuous gripper paths are checked against the frozen
                    # support/object geometry. This is the contact commanded-path
                    # contract, not the free-space joint-error-box certificate.
                    stage('gripper_geometry_contract', shifted)
                    geometry = deepcopy(rospy.get_param('/grasp_6d/remote/gripper_geometry', {}))
                    g = GripperGeometry(float(geometry.get('max_inner_gap_m', .05)),
                        float(geometry.get('width_safety_margin_per_side_m', .002)),
                        np.asarray(geometry.get('finger_box_xyz_m', [.0434,.0286,.06])),
                        np.asarray(geometry.get('palm_box_xyz_m', [.1175,.155,.0774])),
                        float(geometry.get('support_clearance_m', .003)))
                    mismatch = gripper_contract_mismatch_reason(g, .05, .05,
                        ANALYTICAL_GRIPPER_MODEL_NAME, .05,
                        tool_jaw_axis=geometry.get('tool_jaw_axis','y'),
                        tool_finger_length_axis=geometry.get('tool_finger_length_axis','z'))
                    if mismatch:
                        raise ObservationPathError('CONTACT_LINEAR_GRIPPER_CONTRACT: '+mismatch)
                    opening = values['right_finger']
                    stage('nominal_retiming', nominal)
                    nominal_before_retime = deepcopy(nominal)
                    nominal = planner._retime_cartesian_plan(nominal)
                    stage('nominal_retiming', nominal)
                    if not planner._same_joint_path_geometry(nominal_before_retime, nominal):
                        raise ObservationPathError('CONTACT_LINEAR_RETIMING_CHANGED_NOMINAL_PATH')
                    stage('nominal_continuous_geometry', nominal)
                    validate_observation_trajectory(nominal, context['scene'], fk, g,
                        opening, nominal.joint_trajectory.points[0])
                    stage('compensated_retiming', shifted)
                    shifted_before_retime = deepcopy(shifted)
                    shifted = planner._retime_cartesian_plan(shifted)
                    stage('compensated_retiming', shifted)
                    if not planner._same_joint_path_geometry(shifted_before_retime, shifted):
                        raise ObservationPathError('CONTACT_LINEAR_RETIMING_CHANGED_COMMAND_PATH')
                    stage('compensated_continuous_geometry', shifted)
                    validate_observation_trajectory(shifted, context['scene'], fk, g,
                        opening, shifted.joint_trajectory.points[0])
                    stage('planned_context_authority', shifted)
                    validate()
                    evidence.update(code='CONTACT_LINEAR_PLANNED', plan_id=req.plan.plan_id,
                        fixed_goal_pose_index=index, collision_state_checks=checks,
                        compensated_trajectory_sha256=wire_digest(shifted))
                    self._contact_linear_cache = dict(context=context,index=index,
                        trajectory=deepcopy(shifted),sample=deepcopy(sample),evidence=evidence,
                        geometry=geometry, gripper=g, opening=opening)
                    rospy.loginfo('Contact linear compensation planned: %s', json.dumps(evidence))
                    return CompensatePregraspResponse(True,json.dumps(evidence))
                # A new measured start may not silently rewrite a cached offset.
                stage('cached_feedback_and_geometry', cached['trajectory'])
                for key in ('sdk_positions_rad','accepted_positions_rad'):
                    if max(abs(a-b) for a,b in zip(sample[key],cached['sample'][key])) > Q/2:
                        raise ObservationPathError('CONTACT_LINEAR_FEEDBACK_CHANGED_REPLAN_REQUIRED')
                if rospy.get_param('/grasp_6d/remote/gripper_geometry', {}) != cached['geometry']:
                    raise ObservationPathError('CONTACT_LINEAR_GEOMETRY_CHANGED')
                names = ('observation_path_guard_required','observation_path_validator',
                         'observation_tracking_contract_required')
                saved = {n:(hasattr(planner,n),getattr(planner,n,None)) for n in names}
                planner.observation_path_guard_required = True
                planner.observation_tracking_contract_required = False
                def path_guard(trajectory):
                    nonlocal path_guard_failure
                    path_guard_failure = None
                    stage('execution_path_context', trajectory)
                    try:
                        validate()
                        if (self._observation_measured_opening() != cached['opening']
                                or rospy.get_param('/grasp_6d/remote/gripper_geometry', {}) != cached['geometry']):
                            raise ObservationPathError('CONTACT_LINEAR_GRIPPER_CHANGED')
                        if not planner._same_joint_path_geometry(cached['trajectory'],trajectory):
                            raise ObservationPathError('CONTACT_LINEAR_COMMAND_PATH_CHANGED')
                        hold = self._observation_controller_hold(ARM_NAMES)
                        if max(abs(a-b) for a,b in zip(hold.positions,sample['sdk_positions_rad'])) > Q/2:
                            raise ObservationPathError('CONTACT_LINEAR_COMMAND_REFERENCE_CHANGED')
                        duration = trajectory.joint_trajectory.points[-1].time_from_start.to_sec()
                        if time.monotonic()+duration+2. >= context['deadline']:
                            raise ObservationPathError('CONTACT_LINEAR_EXECUTION_TIME_BUDGET')
                        stage('execution_continuous_geometry', trajectory)
                        audit = validate_observation_trajectory(trajectory, context['scene'], fk,
                            cached['gripper'], cached['opening'], hold)
                        stage('execution_context_authority', trajectory)
                        validate()
                        stage('execution_after_geometry', trajectory)
                        return audit
                    except Exception as exc:
                        # MoveIt converts guard exceptions to a (False, text)
                        # result. Retain the original bound audit before that.
                        path_guard_failure = failure(exc)
                        raise
                planner.observation_path_validator = path_guard
                stage('controller_start', cached['trajectory'])
                ok,message = self._ensure_trajectory_controllers_started()
                if not ok:
                    raise ObservationPathError(message)
                validate()
                planner._remember_pose_plan(goal.pose, deepcopy(cached['trajectory']), 'cartesian')
                stage('execute_cached_pose_plan', cached['trajectory'])
                ok,message = planner._execute_cached_pose_plan(goal.pose,
                    planner._pose_xyz_text(goal.pose), required_kind='cartesian',
                    before_execute=validate)
                if not ok:
                    record = path_guard_failure or failure(ObservationPathError(str(message)))
                    if path_guard_failure is not None:
                        record = dict(record, planner_reason=str(message))
                    elif (diagnostic['validation_stage'] == 'execution_after_geometry'
                            and str(message).startswith('execute failed from cached plan (cartesian):')):
                        record['following_failure_diagnostic'] = self._contact_failure_following_diagnostic(
                            context, cached['sample'], fk, record)
                    return rejected(record)
                return CompensatePregraspResponse(ok, message)
            except Exception as exc:
                self._contact_linear_cache = None
                return rejected(path_guard_failure or failure(exc))
            finally:
                retained = getattr(self, '_contact_linear_cache', None)
                if (req.execute or retained is None
                        or retained.get('context') is not context):
                    self._release_contact_visual_context(context)
                if planner is not None:
                    if req.execute:
                        planner._last_pose_plan = None
                    for name,(existed,value) in saved.items():
                        if existed:
                            setattr(planner,name,value)
                        elif hasattr(planner,name):
                            delattr(planner,name)
