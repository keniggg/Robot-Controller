"""Task-only pregrasp compensation using the existing strict path executor."""
from copy import deepcopy
import hashlib
import json
import math
import time

import numpy as np
import rospy

from alicia_flexible_grasp_supervisor.srv import CompensatePregraspResponse
from alicia_flexible_grasp.grasp.grasp_state_machine import GraspStages
from alicia_flexible_grasp.grasp.rich_plan_integrity import pose_values
from .observation_path_guard import (
    FrozenObservationScene, ObservationPathError, SerialUrdfFk,
    committed_plan_digest, segment_coefficients, _polynomial_bounds,
)
from .pregrasp_compensation import PregraspCompensation
from .stationary_following import ARM_NAMES, SDK_QUANTUM_RAD as Q


class PregraspCompensationGateway:
    """Mixin: one finite episode per frozen plan and control-reference epoch.

    No diagnostic epoch is reset. No driver precision lease is acquired here.
    The task selects this route instead of the driver trim for this pregrasp.
    """
    def _pregrasp_context(self, plan):
        with self._observation_context_lock:
            self._require_live_observation_task()
            scene = FrozenObservationScene.from_contact_plan(plan)
            context = dict(scene=scene, plan=deepcopy(plan), epoch=self._control_reference_epoch_ns,
                revocation=self._observation_contexts.revocation,
                model=rospy.get_param('/robot_description', ''),
                config=deepcopy(rospy.get_param('/grasp', {})),
                deadline=time.monotonic()+PregraspCompensation.MAX_SECONDS)
        self._validate_pregrasp_context(context)
        return context

    def _validate_pregrasp_context(self, context):
        with self._observation_context_lock:
            self._require_live_observation_task()
            if (self._manual_control_active()
                    or getattr(self, '_pregrasp_task_stage', None) != GraspStages.MOVE_PREGRASP
                    or self._control_reference_epoch_ns != context['epoch']
                    or self._observation_contexts.revocation != context['revocation']):
                raise ObservationPathError('PREGRASP_AUTHORITY_CHANGED')
        cfg = context['config']
        if (cfg.get('pregrasp_cartesian_compensation_enabled') is not True
                or cfg.get('measured_endpoint_check_enabled') is not True
                or rospy.get_param('/grasp', {}) != cfg
                or rospy.get_param('/robot/observation_tracking_contract_enabled', False) is not True
                or rospy.get_param('/alicia_d_driver_node/endpoint_feedback_trim_enabled', False) is not False):
            raise ObservationPathError('PREGRASP_CONFIGURATION_INVALID_OR_CHANGED')
        if (type(context['epoch']) is not int or context['epoch'] <= 0
                or context['scene'].source_ns < context['epoch']
                or context['scene'].source_ns > int(rospy.get_time()*1e9)
                or rospy.get_param('/robot_description', '') != context['model']
                or committed_plan_digest(context['plan']) != context['scene'].digest
                or time.monotonic() >= context['deadline']
                or not self._fresh_actuation_status().startswith('CONFIRMED:')):
            raise ObservationPathError('PREGRASP_CONTEXT_EXPIRED_OR_UNCONFIRMED')

    def _pregrasp_reference(self):
        reference = self._command_reference_snapshot()
        if reference is None:
            raise ObservationPathError('PREGRASP_REFERENCE_MISSING')
        mapping = dict(zip(self.joint_names[:-1], reference.positions))
        return tuple(self._sdk_arm_words([mapping[n] for n in ARM_NAMES]))

    def _validate_pregrasp_step_path(self, context, step, trajectory, planner):
        self._validate_pregrasp_context(context)
        if self._pregrasp_reference() != step.baseline_counts:
            raise ObservationPathError('PREGRASP_REFERENCE_CHANGED')
        names = tuple(trajectory.joint_trajectory.joint_names)
        if len(names) != 6 or set(names) != set(ARM_NAMES):
            raise ObservationPathError('PREGRASP_PATH_JOINTS_INVALID')
        points = list(trajectory.joint_trajectory.points)
        target = dict(zip(ARM_NAMES, step.positions))
        if (len(points) < 2 or len(points[-1].positions) != 6
                or any(abs(points[-1].positions[i]-target[n]) > 1e-12 for i, n in enumerate(names))):
            raise ObservationPathError('PREGRASP_PATH_TARGET_CHANGED')
        hold = self._observation_controller_hold(names)
        baseline = dict(zip(ARM_NAMES, [(v-2048)*Q for v in step.baseline_counts]))
        b = np.array([baseline[n] for n in names])
        initial = dict(zip(ARM_NAMES, [(v-2048)*Q for v in context['initial_sdk_counts']]))
        origin = np.array([initial[n] for n in names])
        if max(abs(np.asarray(hold.positions)-b)) > Q/2+1e-9:
            raise ObservationPathError('PREGRASP_CONTROLLER_REFERENCE_CHANGED')
        positive = next((i for i,p in enumerate(points) if p.time_from_start.to_sec()>0), None)
        if positive is None:
            raise ObservationPathError('PREGRASP_PATH_TIMING_INVALID')
        pairs = [(hold, points[positive], points[positive].time_from_start.to_sec())]
        pairs.extend((a,b_,b_.time_from_start.to_sec()-a.time_from_start.to_sec())
                     for a,b_ in zip(points[positive:],points[positive+1:]))
        for a,b_,duration in pairs:
            low, high = _polynomial_bounds(segment_coefficients(a,b_,duration,6),0.,1.)
            if np.any(low < b-4*Q-1e-9) or np.any(high > b+4*Q+1e-9):
                raise ObservationPathError('PREGRASP_PATH_LEAVES_FOUR_COUNT_BOX')
            if np.any(low < origin-32*Q-1e-9) or np.any(high > origin+32*Q+1e-9):
                raise ObservationPathError('PREGRASP_PATH_LEAVES_TOTAL_COUNT_BOX')
            # The controller reference can be fractional while the wire SDK
            # target is quantized. Every uncommanded axis, including one retired
            # after weak response, must stay in the SAME encoder command cell.
            for axis, name in enumerate(ARM_NAMES):
                if step.target_counts[axis] != step.baseline_counts[axis]:
                    continue
                j = names.index(name)
                if low[j] < b[j]-Q/2 or high[j] >= b[j]+Q/2:
                    raise ObservationPathError('PREGRASP_PATH_MOVES_HELD_AXIS: '+name)
        audit = self._validate_frozen_observation_path(
            (context['scene'], context['revocation']), trajectory, planner,
            context_validator=lambda:self._validate_pregrasp_context(context))
        duration = points[-1].time_from_start.to_sec()
        grace = float(audit['controller_constraints_snapshot']['goal_time'])
        if time.monotonic()+duration+grace+2. >= context['deadline']:
            raise ObservationPathError('PREGRASP_INSUFFICIENT_STEP_TIME')
        audit['pregrasp_fixed_plan_id'] = context['scene'].plan_id
        return audit

    def _pregrasp_still_authorized(self, context):
        try:
            self._validate_pregrasp_context(context)
            return True
        except Exception:
            return False

    def handle_compensate_pregrasp(self, req):
        with self._planner_operation_lock():
            correction, evidence, saved, planner = None, {}, {}, None
            try:
                context = self._pregrasp_context(deepcopy(req.plan))
                key = hashlib.sha256(('%s:%s' % (context['epoch'],context['scene'].plan_id)).encode()).hexdigest()
                episode_param = '/supervisor/pregrasp_compensation_episodes/'+key
                if req.execute and rospy.get_param(episode_param, None) is not None:
                    raise ObservationPathError('PREGRASP_EPISODE_ALREADY_CONSUMED')
                fk = SerialUrdfFk(context['model'], ARM_NAMES)
                validate = lambda:self._validate_pregrasp_context(context)
                sample = self._endpoint_following_snapshot(fk, context['epoch'], context,
                                                           context_validator=validate)
                cfg = context['config']
                correction = PregraspCompensation(fk, sample, pose_values(context['plan'].poses[0]),
                    position_tolerance_m=float(cfg['measured_endpoint_position_tolerance_m']),
                    orientation_tolerance_rad=math.radians(float(cfg['measured_endpoint_orientation_tolerance_deg'])),
                    joint2_response_probe_enabled=(
                        cfg.get('unknown_pregrasp_joint2_response_probe_enabled', False)
                        if getattr(context['plan'], 'model_choice', '') == 'unknown_tabletop'
                        else False))
                context['initial_sdk_counts'] = correction.initial_counts
                planner = self._ensure_planner()
                if planner is None:
                    raise ObservationPathError(self._moveit_not_ready_message())
                attributes = ('observation_path_guard_required','observation_path_validator',
                    'observation_tracking_contract_required','observation_execution_authorized',
                    'observation_submission_revalidate','strict_execution_max_joint_velocity_rad_s')
                saved = {name:(hasattr(planner,name),getattr(planner,name,None)) for name in attributes}
                planner.observation_path_guard_required = True
                planner.observation_tracking_contract_required = True
                planner.strict_execution_max_joint_velocity_rad_s = min(
                    float(planner.strict_execution_max_joint_velocity_rad_s), .08)
                self._configure_controller_reference_guard(planner)
                planner.observation_execution_authorized = lambda:self._pregrasp_still_authorized(context)
                planner.observation_submission_revalidate = lambda trajectory,audit: (
                    self._revalidate_observation_contract_submission(
                        (context['scene'],context['revocation']),trajectory,audit,context_validator=validate))
                for _ in range(PregraspCompensation.MAX_STEPS+1):
                    validate()
                    code, step, evidence = correction.evaluate(sample)
                    evidence.update(code=code,plan_id=context['scene'].plan_id,epoch_ns=context['epoch'],
                                    execution_requested=bool(req.execute),model_sha256=fk.model_sha256)
                    rospy.loginfo('Pregrasp Cartesian compensation: %s',json.dumps(evidence))
                    if step is None:
                        return CompensatePregraspResponse(True,json.dumps(evidence))
                    if self._pregrasp_reference() != step.baseline_counts:
                        raise ObservationPathError('PREGRASP_REFERENCE_CHANGED')
                    planner.observation_path_validator = lambda tr: self._validate_pregrasp_step_path(
                        context,step,tr,planner)
                    if req.execute:
                        if correction.steps == 0:
                            ok,message = self._ensure_trajectory_controllers_started()
                            if not ok:
                                raise ObservationPathError(message)
                            ok,message = self._synchronize_trajectory_controller_to_feedback()
                            if not ok:
                                raise ObservationPathError(message)
                            from .contracted_trajectory import ContractedTrajectoryExecutor
                            if getattr(planner,'observation_trajectory_executor',None) is None:
                                planner.observation_trajectory_executor = ContractedTrajectoryExecutor.connect()
                        def before_execute():
                            validate()
                            if self._pregrasp_reference() != step.baseline_counts:
                                raise ObservationPathError('PREGRASP_REFERENCE_CHANGED')
                            rospy.set_param(episode_param,dict(plan_id=context['scene'].plan_id,
                                epoch_ns=str(context['epoch']),state='consumed_before_submission',
                                steps_submitted=correction.steps+1))
                    else:
                        before_execute = None
                    ok,message = planner.plan_and_execute_joint_probe(step.positions,execute=bool(req.execute),
                        start_joint_positions=[(v-2048)*Q for v in step.baseline_counts],
                        before_execute=before_execute)
                    if not ok:
                        raise ObservationPathError('PREGRASP_STEP_REJECTED: '+message)
                    if not req.execute:
                        evidence.update(code='PREGRASP_PLANNED_ONLY',physical_path_authorized=False)
                        return CompensatePregraspResponse(True,json.dumps(evidence))
                    completed = rospy.get_time()
                    correction.committed(step,completed)
                    sample = self._endpoint_following_snapshot(fk,context['epoch'],context,
                        after_ns=int(completed*1e9),context_validator=validate)
                raise ObservationPathError('PREGRASP_STEP_BUDGET')
            except Exception as exc:
                if correction is not None and correction.last_evidence is not None:
                    # Do not mix a previous proposal's prediction with the
                    # current failed feedback snapshot.
                    metadata = {k:evidence[k] for k in ('plan_id','epoch_ns','execution_requested',
                                'model_sha256') if k in evidence}
                    evidence = dict(deepcopy(correction.last_evidence), **metadata)
                    if correction.pending is not None and evidence.get('steps_completed', -1) < correction.steps:
                        # A completed action is not a validated response. When
                        # stationary acquisition fails, keep the old sample
                        # labelled as old; never report it as the final pose.
                        evidence = dict(metadata,
                            fixed_goal_pose=list(correction.goal_values),
                            steps_completed=correction.steps,
                            completed_command_target_counts=list(correction.expected),
                            post_command_feedback_valid=False,
                            last_validated_evidence=deepcopy(correction.last_evidence))
                evidence.update(code=str(exc),success=False,real_grasp_success=False,
                                no_implicit_rollback_or_torque_disable=True)
                rospy.logwarn('Pregrasp compensation stopped: %s',json.dumps(evidence))
                return CompensatePregraspResponse(False,json.dumps(evidence))
            finally:
                if planner is not None:
                    for name,(existed,value) in saved.items():
                        if existed:
                            setattr(planner,name,value)
                        elif hasattr(planner,name):
                            delattr(planner,name)
