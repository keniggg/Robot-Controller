"""Frozen-contact nominal pregrasp selection, distinct from local correction.

Every call plans from current measured/controller state. No selected joint
goal or executable trajectory survives the call. Physical arrival remains
the task's independent measured endpoint check.
"""
from copy import deepcopy
import hashlib
import json
import math
import time

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from alicia_flexible_grasp_supervisor.srv import CompensatePregraspResponse
from alicia_flexible_grasp.grasp.grasp_state_machine import GraspStages
from alicia_flexible_grasp.grasp.rich_plan_integrity import pose_values, is_silhouette_observation_plan
from alicia_flexible_grasp.grasp.endpoint_tolerance import measured_position_tolerance
from .stationary_following import ARM_NAMES, SDK_QUANTUM_RAD as Q
from .observation_path_guard import ObservationPathError, SerialUrdfFk, committed_plan_digest
from .observation_path_guard import FrozenObservationScene
from .endpoint_correction import sdk_counts
from .pregrasp_compensation import pose_matrix
from .held_axes_prediction import HeldAxesPredictionError
from .held_pregrasp_selection import select_held_pregrasp_endpoint
from .contact_sequence_precheck import (
    contact_precheck_gripper, predict_held_approach, validate_selected_pregrasp_path)


POLICY = 'frozen_contact_held_approach_pregrasp_start'


class ContactPregraspGateway:
    def _capture_contact_pregrasp_plan(self, msg):
        """A published-plan receipt, never a joint hint or executable cache."""
        if msg.valid is not True:
            self._contact_pregrasp_receipts = {}
            return
        if str(msg.diagnostic) != 'CONTACT_EXECUTION_PLAN':
            return
        scene = FrozenObservationScene.from_contact_plan(msg)
        receipt = dict(plan_sha256=scene.digest, source_ns=scene.source_ns,
                       epoch=getattr(self, '_control_reference_epoch_ns', None),
                       selection=deepcopy(rospy.get_param('/grasp_mode/selection', {})))
        receipts = getattr(self, '_contact_pregrasp_receipts', {})
        receipts[scene.plan_id] = receipt
        while len(receipts) > 16:
            del receipts[next(iter(receipts))]
        self._contact_pregrasp_receipts = receipts

    def _validate_contact_pregrasp_context(self, context):
        self._validate_pregrasp_context(context)
        cfg = context['config']
        if (context['plan'].model_choice != 'unknown_tabletop'
                or context['pose_index'] != 0
                or any(cfg.get(k) is not True for k in (
                    'unknown_approach_held_axes_feedforward_enabled',
                    'unknown_contact_cartesian_compensation_enabled',
                    'unknown_pregrasp_feedforward_enabled',
                    'unknown_pregrasp_held_axes_feedforward_enabled',
                    'measured_endpoint_check_enabled', 'contact_endpoint_precision_enabled'))
                or context['selection'] != rospy.get_param('/grasp_mode/selection', {})
                or context['selection'].get('mode') != 'unknown'
                or context['scene'].source_ns <= int(context['selection'].get('stamp_ns', 0))
                or context['geometry'] != rospy.get_param('/grasp_6d/remote/gripper_geometry', {})
                or context['gripper_config'] != rospy.get_param('/gripper', {})
                or committed_plan_digest(context['plan']) != context['scene'].digest):
            raise ObservationPathError('CONTACT_PREGRASP_CONTEXT_CHANGED')
        self._validate_contact_pregrasp_session(context)

    def _bind_contact_pregrasp_session(self, context):
        # Reuse the origin captured by the real gateway and its already
        # validated task-session callback. No parameter can create a lease.
        with self._observation_context_lock:
            origin = getattr(self, '_observe_return_origin', None)
            if origin is None or origin.get('selection') != context['selection']:
                context['outer_observation_session'] = None
                return
            if (origin['epoch'] != context['epoch']
                    or origin['revocation'] != context['revocation']
                    or origin['scene'].track_id != context['scene'].track_id
                    or origin['scene'].source_ns > context['scene'].source_ns
                    or origin['model'] != context['model']
                    or origin['config'] != context['config']
                    or committed_plan_digest(origin['plan']) != origin['scene'].digest
                    or time.monotonic() >= origin['deadline']):
                raise ObservationPathError('CONTACT_PREGRASP_SESSION_BINDING_CHANGED')
            session = deepcopy(origin.get('view_session'))
            expects_v2 = (origin['config'].get('unknown_observe_return_session_version', 2) == 2
                          and not is_silhouette_observation_plan(origin['plan']))
            if session is None and expects_v2:
                raise ObservationPathError('CONTACT_PREGRASP_QUALIFIED_SESSION_REQUIRED')
            if session is not None:
                session = self._validated_observe_return_view_session(origin, session)
                if session['view_source_stamp_ns'] > context['scene'].source_ns:
                    raise ObservationPathError('CONTACT_PREGRASP_PLAN_PREDATES_SESSION_VIEW')
            outer = dict(origin=origin, identity=origin['identity'],
                origin_deadline=origin['deadline'], origin_scene_digest=origin['scene'].digest,
                view_session=session)
            context['outer_observation_session'] = outer
            context['deadline'] = min(context['deadline'], origin['deadline'])
            if session is not None:
                # Convert once only; neither another sample nor a ROS-clock
                # delay may renew the original absolute outer deadline.
                context['deadline'] = min(context['deadline'], time.monotonic()+
                    (session['deadline_ns']-int(rospy.get_time()*1e9))*1e-9)

    def _validate_contact_pregrasp_session(self, context):
        with self._observation_context_lock:
            outer = context['outer_observation_session']
            current = getattr(self, '_observe_return_origin', None)
            if outer is None:
                if current is not None and current.get('selection') == context['selection']:
                    raise ObservationPathError('CONTACT_PREGRASP_SESSION_APPEARED')
                return
            origin = outer['origin']
            if (current is not origin or origin.get('identity') != outer['identity']
                    or origin.get('deadline') != outer['origin_deadline']
                    or origin.get('view_session') != outer['view_session']
                    or origin.get('selection') != context['selection']
                    or origin.get('epoch') != context['epoch']
                    or origin.get('revocation') != context['revocation']
                    or origin['scene'].track_id != context['scene'].track_id
                    or origin['scene'].digest != outer['origin_scene_digest']
                    or committed_plan_digest(origin['plan']) != outer['origin_scene_digest']
                    or origin['model'] != context['model']
                    or origin['config'] != context['config']):
                raise ObservationPathError('CONTACT_PREGRASP_SESSION_CHANGED')
            session = outer['view_session']
            if session is not None and int(rospy.get_time()*1e9) >= session['deadline_ns']:
                raise ObservationPathError('CONTACT_PREGRASP_SESSION_EXPIRED')

    def _wait_for_contact_pregrasp_phase(self):
        deadline = time.monotonic()+1.
        while True:
            self._require_live_observation_task()
            if rospy.is_shutdown() or self._manual_control_active():
                raise ObservationPathError('CONTACT_PREGRASP_TASK_PHASE_UNAVAILABLE')
            if getattr(self, '_pregrasp_task_stage', None) == GraspStages.MOVE_PREGRASP:
                return
            if time.monotonic() >= deadline:
                raise ObservationPathError('CONTACT_PREGRASP_TASK_PHASE_UNAVAILABLE')
            rospy.sleep(.02)

    def _wait_for_contact_pregrasp_receipt(self, context):
        # Task-plan and gateway-plan subscribers are independent connections.
        # Wait within this call's original deadline; no lease is renewed.
        expected = dict(plan_sha256=context['scene'].digest,
            source_ns=context['scene'].source_ns, epoch=context['epoch'],
            selection=context['selection'])
        deadline = min(context['deadline'], time.monotonic()+1.)
        while True:
            self._validate_contact_pregrasp_context(context)
            with self._observation_context_lock:
                receipt = deepcopy(getattr(self, '_contact_pregrasp_receipts', {}).get(
                    context['scene'].plan_id))
            if receipt is not None:
                if receipt != expected:
                    raise ObservationPathError('CONTACT_PREGRASP_PUBLISHED_PLAN_MISMATCH')
                self._validate_contact_pregrasp_context(context)
                return
            if rospy.is_shutdown() or time.monotonic() >= deadline:
                raise ObservationPathError('CONTACT_PREGRASP_PUBLISHED_PLAN_REQUIRED')
            rospy.sleep(.01)

    @staticmethod
    def _contact_pregrasp_start(state, positions):
        state = deepcopy(state)
        js = state.joint_state
        if (len(js.name) != len(js.position) or len(set(js.name)) != len(js.name)
                or not set(ARM_NAMES+('right_finger',)).issubset(js.name)):
            raise ObservationPathError('CONTACT_PREGRASP_START_INCOMPLETE')
        values = dict(zip(ARM_NAMES, positions))
        js.position = [values.get(n, v) for n, v in zip(js.name, js.position)]
        state.is_diff = False
        return state

    @staticmethod
    def _current_contact_pregrasp_proof(fk, sample, context):
        # A same-pose shortcut must prove the actual current joint start and
        # the unchanged held approach; the nominal pose alone proves neither.
        cfg = context['config']
        goal = pose_matrix(pose_values(context['plan'].poses[0]))
        actual = fk(sample['accepted_positions_rad'])
        pe = float(np.linalg.norm(actual[:3, 3]-goal[:3, 3]))
        ae = math.acos(float(np.clip((np.trace(actual[:3, :3].T@goal[:3, :3])-1)/2, -1., 1.)))
        if (pe > measured_position_tolerance(cfg, 'unknown_tabletop')
                or ae > math.radians(float(cfg['measured_endpoint_orientation_tolerance_deg']))):
            return None
        endpoint = RobotTrajectory()
        endpoint.joint_trajectory.joint_names = list(ARM_NAMES)
        endpoint.joint_trajectory.points = [JointTrajectoryPoint(positions=list(sample['sdk_positions_rad']))]
        proof = {}
        try:
            predict_held_approach(endpoint, fk, sample, context['plan'].poses[0],
                context['plan'].poses[1], cfg, proof)
        except Exception as exc:
            cause = exc.__cause__
            if (isinstance(cause, HeldAxesPredictionError)
                    and str(cause) == 'PREGRASP_HELD_AXES_GUIDE_DOES_NOT_REACH_FIXED_GOAL'):
                return None
            raise
        # No pose0 correction is allowed to masquerade as no-motion reuse.
        predicted = proof.get('predicted_pregrasp', {}).get('predicted_command_positions_rad', [])
        if np.asarray(predicted).shape != (6,) or not np.array_equal(
                sdk_counts(predicted), sdk_counts(sample['sdk_positions_rad'])):
            return None
        return dict(position_error_m=pe, orientation_error_rad=ae, held_prediction=proof)

    def handle_contact_pregrasp(self, req):
        audit = dict(execution_policy=POLICY, plan_id=str(req.plan.plan_id),
                     execution_requested=bool(req.execute), real_grasp_success=False,
                     hardware_response_verified=False, endpoint_confirmation_required=True)
        planner, saved, context = None, {}, None
        with self._planner_operation_lock():
            try:
                self._wait_for_contact_pregrasp_phase()
                context = self._pregrasp_context(deepcopy(req.plan), 0)
                context['geometry'] = deepcopy(rospy.get_param('/grasp_6d/remote/gripper_geometry', {}))
                context['gripper_config'] = deepcopy(rospy.get_param('/gripper', {}))
                self._bind_contact_pregrasp_session(context)
                self._wait_for_contact_pregrasp_receipt(context)
                validate = lambda: self._validate_contact_pregrasp_context(context)
                def remaining():
                    validate()
                    left = context['deadline']-time.monotonic()
                    if left <= 0.:
                        raise ObservationPathError('CONTACT_PREGRASP_DEADLINE_EXPIRED')
                    return left
                remaining()
                key = hashlib.sha256(('%s:%s' % (
                    context['epoch'], context['scene'].digest)).encode()).hexdigest()
                consumed = '/supervisor/contact_pregrasp_executions/'+key
                if rospy.get_param(consumed, None) is not None:
                    raise ObservationPathError('CONTACT_PREGRASP_ALREADY_SUBMITTED')
                fk = SerialUrdfFk(context['model'], ARM_NAMES)
                sample = self._endpoint_following_snapshot(fk, context['epoch'], context,
                                                           context_validator=validate)
                outer = context['outer_observation_session']
                if outer is not None:
                    audit['outer_session_deadline'] = dict(origin_identity=outer['identity'],
                        monotonic_deadline=context['deadline'],
                        session_id=(outer['view_session'] or {}).get('session_id'),
                        ros_deadline_ns=(outer['view_session'] or {}).get('deadline_ns'),
                        source='captured_gateway_origin_and_validated_task_session')
                audit.update(fixed_goal_pose=list(pose_values(context['plan'].poses[0])),
                    plan_source_ns=context['scene'].source_ns, plan_sha256=context['scene'].digest,
                    epoch_ns=context['epoch'], model_sha256=fk.model_sha256, source_sample=sample)
                already = self._current_contact_pregrasp_proof(fk, sample, context)
                if already is not None:
                    fresh = self._endpoint_following_snapshot(fk, context['epoch'], context,
                                                              context_validator=validate)
                    remaining()
                    if (self._pregrasp_reference() != tuple(sdk_counts(sample['sdk_positions_rad']))
                            or max(abs(np.asarray(fresh['sdk_positions_rad'])-sample['sdk_positions_rad'])) > Q/2
                            or max(abs(np.asarray(fresh['accepted_positions_rad'])-sample['accepted_positions_rad'])) > Q
                            or fresh['accepted_gripper_opening_m'] != sample['accepted_gripper_opening_m']):
                        raise ObservationPathError('CONTACT_PREGRASP_REUSE_REFERENCE_CHANGED')
                    current = self._current_contact_pregrasp_proof(fk, fresh, context)
                    if current is None:
                        raise ObservationPathError('CONTACT_PREGRASP_REUSE_PROOF_CHANGED')
                    remaining()
                    audit.update(code='CONTACT_PREGRASP_ALREADY_SATISFIED' if req.execute
                                 else 'CONTACT_PREGRASP_CHECKED_ONLY',
                        current_joint_start_verified=True, current_joint_start_proof=current,
                        no_motion_submitted=True, execution_submitted=False,
                        complete_execution_path_checked=False, execution_authorized=False)
                    return CompensatePregraspResponse(True, json.dumps(audit))
                planner = self._ensure_planner()
                if planner is None:
                    raise ObservationPathError(self._moveit_not_ready_message())
                planner._last_pose_plan = None
                cfg = context['config']
                goal = PoseStamped(); goal.header.frame_id = 'base_link'
                goal.pose = deepcopy(context['plan'].poses[0])
                real_start = self._contact_pregrasp_start(
                    planner.robot.get_current_state(), sample['accepted_positions_rad'])
                seed_path, reason = planner._plan_pose_from_start_state(real_start, goal,
                    planning_time_limit_sec=min(remaining(), 2.))
                if seed_path is None:
                    raise ObservationPathError('CONTACT_PREGRASP_SEED_FAILED: '+str(reason))
                jt = seed_path.joint_trajectory
                if set(jt.joint_names) != set(ARM_NAMES) or len(jt.joint_names) != 6 or not jt.points:
                    raise ObservationPathError('CONTACT_PREGRASP_SEED_INVALID')
                seed = dict(zip(jt.joint_names, jt.points[-1].positions))
                if len(seed) != 6:
                    raise ObservationPathError('CONTACT_PREGRASP_SEED_INVALID')
                target, selection = select_held_pregrasp_endpoint(fk, sample,
                    pose_values(context['plan'].poses[0]), pose_values(context['plan'].poses[1]),
                    [seed[n] for n in ARM_NAMES],
                    position_tolerance_m=measured_position_tolerance(cfg, 'unknown_tabletop'),
                    orientation_tolerance_rad=math.radians(float(cfg['measured_endpoint_orientation_tolerance_deg'])),
                    deadline_check=remaining)
                remaining()
                target = np.asarray(target, dtype=float)
                if target.shape != (6,) or not np.all(np.isfinite(target)):
                    raise ObservationPathError('CONTACT_PREGRASP_SELECTED_TARGET_INVALID')
                target = target.copy(); target.setflags(write=False)
                gripper = contact_precheck_gripper(context['geometry'])
                opening = float(sample['accepted_gripper_opening_m'])
                audit.update(selection=selection, command_goal_positions_rad=target.tolist(),
                    fixed_goal_pose=list(pose_values(goal.pose)),
                    plan_source_ns=context['scene'].source_ns,
                    plan_sha256=context['scene'].digest, epoch_ns=context['epoch'],
                    model_sha256=fk.model_sha256, source_sample=sample)

                def fresh_start():
                    remaining()
                    fresh = self._endpoint_following_snapshot(fk, context['epoch'], context,
                                                              context_validator=validate)
                    for field, limit in (('sdk_positions_rad', Q/2), ('accepted_positions_rad', Q)):
                        if max(abs(np.asarray(fresh[field])-sample[field])) > limit+1e-12:
                            raise ObservationPathError('CONTACT_PREGRASP_START_CHANGED')
                    if (fresh['accepted_gripper_opening_m'] != opening
                            or self._pregrasp_reference() != tuple(sdk_counts(sample['sdk_positions_rad']))):
                        raise ObservationPathError('CONTACT_PREGRASP_REFERENCE_CHANGED')
                    # One endpoint is enough for the pure predictor; this is
                    # never represented as a planned or executable trajectory.
                    endpoint = RobotTrajectory()
                    endpoint.joint_trajectory.joint_names = list(ARM_NAMES)
                    endpoint.joint_trajectory.points = [JointTrajectoryPoint(positions=target.tolist())]
                    proof = {}
                    predict_held_approach(endpoint, fk, fresh, context['plan'].poses[0],
                        context['plan'].poses[1], cfg, proof)
                    audit['fresh_fixed_endpoint_prediction'] = proof
                    return fresh

                def path_guard(path):
                    remaining()
                    tr = path.joint_trajectory
                    terminal = dict(zip(tr.joint_names, tr.points[-1].positions)) if tr.points else {}
                    if set(terminal) != set(ARM_NAMES) or any(
                            abs(terminal[n]-target[i]) > 1e-12 for i,n in enumerate(ARM_NAMES)):
                        raise ObservationPathError('CONTACT_PREGRASP_PATH_TARGET_CHANGED')
                    fresh = fresh_start()
                    current_state = self._contact_pregrasp_start(
                        planner.robot.get_current_state(), fresh['accepted_positions_rad'])
                    audit['selected_path_prediction'] = validate_selected_pregrasp_path(
                        path, fk, fresh, context['scene'], gripper, opening, planner,
                        current_state, self._contact_command_collision_check, remaining)
                    result = self._validate_frozen_observation_path(
                        (context['scene'], context['revocation']), path, planner,
                        context_validator=validate)
                    duration = tr.points[-1].time_from_start.to_sec()
                    if duration+float(result['controller_constraints_snapshot']['goal_time'])+2. >= remaining():
                        raise ObservationPathError('CONTACT_PREGRASP_INSUFFICIENT_EXECUTION_TIME')
                    audit['complete_execution_path_checked'] = True
                    return result

                if not req.execute:
                    command_start = self._contact_pregrasp_start(real_start, sample['sdk_positions_rad'])
                    path, reason = planner._plan_joint_from_start_state(command_start, target,
                        planning_time_limit_sec=min(remaining(), 2.))
                    if path is None:
                        raise ObservationPathError('CONTACT_PREGRASP_JOINT_PLAN_FAILED: '+str(reason))
                    audit['selected_path_prediction'] = validate_selected_pregrasp_path(
                        path, fk, sample, context['scene'], gripper, opening, planner,
                        real_start, self._contact_command_collision_check, remaining)
                    fresh_start()
                    audit.update(code='CONTACT_PREGRASP_CHECKED_ONLY', execution_authorized=False)
                    return CompensatePregraspResponse(True, json.dumps(audit))

                attributes = ('observation_path_guard_required','observation_path_validator',
                    'observation_tracking_contract_required','observation_execution_authorized',
                    'observation_submission_revalidate','observation_execution_monitor',
                    'strict_execution_max_joint_velocity_rad_s',
                    'unknown_pregrasp_joint_velocity_limits_rad_s')
                saved = {k:(hasattr(planner,k),getattr(planner,k,None)) for k in attributes}
                planner.observation_path_guard_required = True
                planner.observation_tracking_contract_required = True
                planner.observation_path_validator = path_guard
                planner.observation_execution_authorized = lambda: self._contact_pregrasp_authorized(context)
                planner.observation_execution_monitor = None
                planner.observation_submission_revalidate = lambda path, report: self._revalidate_observation_contract_submission(
                    (context['scene'],context['revocation']), path, report, context_validator=validate)
                planner.unknown_pregrasp_joint_velocity_limits_rad_s = self._unknown_pregrasp_velocity_limits(planner)
                self._configure_controller_reference_guard(planner)
                ok, reason = self._ensure_trajectory_controllers_started()
                if not ok: raise ObservationPathError(reason)
                ok, reason = self._synchronize_trajectory_controller_to_feedback()
                if not ok: raise ObservationPathError(reason)
                from .contracted_trajectory import ContractedTrajectoryExecutor
                if getattr(planner, 'observation_trajectory_executor', None) is None:
                    planner.observation_trajectory_executor = ContractedTrajectoryExecutor.connect()
                def before_execute():
                    fresh_start()
                    if rospy.get_param(consumed, None) is not None:
                        raise ObservationPathError('CONTACT_PREGRASP_ALREADY_SUBMITTED')
                    rospy.set_param(consumed, dict(plan_id=context['scene'].plan_id,
                        plan_sha256=context['scene'].digest, source_ns=str(context['scene'].source_ns),
                        epoch_ns=str(context['epoch']), state='consumed_before_submission',
                        command_goal_positions_rad=target.tolist(), execution_policy=POLICY))
                ok, reason = planner.plan_and_execute_joint_probe(target.tolist(), execute=True,
                    start_joint_positions=sample['sdk_positions_rad'], before_execute=before_execute)
                if not ok:
                    raise ObservationPathError('CONTACT_PREGRASP_EXECUTION_REJECTED: '+str(reason))
                validate()
                audit.update(code='CONTACT_PREGRASP_EXECUTED', execution_submitted=True,
                             endpoint_confirmation_required=True)
                return CompensatePregraspResponse(True, json.dumps(audit))
            except Exception as exc:
                audit.update(code='CONTACT_PREGRASP_REJECTED', reason=str(exc), success=False)
                return CompensatePregraspResponse(False, json.dumps(audit))
            finally:
                if planner is not None:
                    planner._last_pose_plan = None
                    for name,(existed,value) in saved.items():
                        if existed: setattr(planner,name,value)
                        elif hasattr(planner,name): delattr(planner,name)
                # The next local correction acquires its own fresh context.
                if getattr(self, '_compensation_visual_context', None) is context:
                    with self._observation_context_lock:
                        if self._compensation_visual_context is context:
                            self._compensation_visual_context = None

    def _contact_pregrasp_authorized(self, context):
        try:
            self._validate_contact_pregrasp_context(context)
            return True
        except Exception:
            return False
