"""Task-only fixed-pose compensation using the existing strict path executor."""
from copy import deepcopy
import hashlib
import json
import math
import time

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped

from alicia_flexible_grasp_supervisor.srv import CompensatePregraspResponse
from alicia_flexible_grasp.grasp.grasp_state_machine import GraspStages
from alicia_flexible_grasp.grasp.rich_plan_integrity import pose_values
from alicia_flexible_grasp.grasp.endpoint_tolerance import measured_position_tolerance
from .observation_path_guard import (
    FrozenObservationScene, ObservationPathError, SerialUrdfFk,
    committed_plan_digest, segment_coefficients, _polynomial_bounds,
)
from .pregrasp_compensation import PregraspCompensation
from .pregrasp_feedforward import PregraspFeedforward
from .feedforward_timing import resolve_unknown_feedforward_timing
from .endpoint_correction import sdk_counts
from alicia_flexible_grasp.grasp.contact_execution import plan_contact_evaluator
from alicia_flexible_grasp.grasp.contact_height_margin import contact_height_evidence_limit
from .stationary_following import ARM_NAMES, SDK_QUANTUM_RAD as Q, _window
from .contact_tracking_contract import (
    POLICY as CONTACT_TRACKING_POLICY, revalidate_contact_tracking_contract,
)


class PregraspCompensationGateway:
    """Mixin: one finite episode per frozen plan and control-reference epoch.

    No diagnostic epoch is reset. No driver precision lease is acquired here.
    The task selects this route instead of the driver trim for this pregrasp.
    """
    def _contact_tracking_feedback_monitor(self, context):
        """Keep real sources fresh during a step and check its settled envelope.

        JTC enforces path/goal error in motion. Feedback faults raise through
        the executor's owned stopping-hold path, distinct from manual takeover.
        """
        contract = context.get('active_contact_tracking_contract')
        if contract is None:
            return
        if 'contact_tracking_fault' in context:
            raise ObservationPathError(context['contact_tracking_fault'])
        try:
            self._validate_pregrasp_context(context)
            with self._endpoint_feedback_lock:
                sdk = tuple(self._endpoint_feedback_history['sdk'])
                accepted = tuple(self._endpoint_feedback_history['accepted'])
            now, age = rospy.get_time(), contract['maximum_feedback_age_sec']
            rows = _window(accepted, 'sdk_measured', ARM_NAMES+('right_finger',),
                           now, context['epoch'], age)
            wire = _window(sdk, 'sdk_transmitted', ARM_NAMES, now, context['epoch'], age)
            anchors = context.setdefault('contact_tracking_monitored_stamps', {})
            for history, source in ((rows, 'accepted'), (wire, 'sdk')):
                start = anchors.get(source, contract['entry_feedback'][source+'_stamp_ns'])
                recent = [r for r in history if r[0] >= start]
                if (not recent or (recent[0][0]-start)*1e-9 > age
                        or any((b[0]-a[0])*1e-9 > age for a,b in zip(recent,recent[1:]))):
                    raise ValueError('contact tracking source continuity lost: '+source)
                anchors[source] = history[-1][0]
            after = context.get('contact_tracking_completed_after_ns')
            if after is not None:
                goal = np.asarray(context['contact_tracking_goal_positions'])
                tolerance = np.asarray(contract['goal_position_tolerance_rad'])
                if any(np.any(abs(q-goal) > tolerance) for stamp,q in rows if stamp > after):
                    raise ValueError('contact tracking goal envelope exceeded during settle')
        except (KeyError, TypeError, ValueError, ObservationPathError) as exc:
            context['contact_tracking_fault'] = 'CONTACT_TRACKING_FEEDBACK_INVALID: '+str(exc)
            raise ObservationPathError(context['contact_tracking_fault']) from exc

    def _revalidate_pregrasp_tracking_submission(self, context, trajectory, audit):
        validate = lambda:self._validate_pregrasp_context(context)
        self._revalidate_observation_contract_submission(
            (context['scene'],context['revocation']), trajectory, audit, context_validator=validate)
        contract = audit['execution_tracking_contract']
        context.pop('active_contact_tracking_contract', None)
        context.pop('contact_tracking_completed_after_ns', None)
        context.pop('contact_tracking_monitored_stamps', None)
        if contract.get('policy') != CONTACT_TRACKING_POLICY:
            return
        if context.get('pose_index') != 2 or context['plan'].model_choice != 'unknown_tabletop':
            raise ObservationPathError('CONTACT_TRACKING_SCOPE_INVALID')
        fk = SerialUrdfFk(context['model'], ARM_NAMES)
        hold = self._observation_controller_hold(ARM_NAMES)
        with self._endpoint_feedback_lock:
            sdk = tuple(self._endpoint_feedback_history['sdk'])
            accepted = tuple(self._endpoint_feedback_history['accepted'])
        revalidate_contact_tracking_contract(contract, trajectory, hold, fk, sdk, accepted,
            audit['controller_constraints_snapshot'], contract['stop_trajectory_duration_sec'],
            now_sec=rospy.get_time(), epoch_ns=context['epoch'])
        validate()
        context['active_contact_tracking_contract'] = deepcopy(contract)
        context['contact_tracking_goal_positions'] = list(trajectory.joint_trajectory.points[-1].positions)
        self._contact_tracking_feedback_monitor(context)

    def _pregrasp_context(self, plan, pose_index=0):
        with self._observation_context_lock:
            self._require_live_observation_task()
            scene = FrozenObservationScene.from_contact_plan(plan)
            context = dict(scene=scene, plan=deepcopy(plan), epoch=self._control_reference_epoch_ns,
                revocation=self._observation_contexts.revocation,
                model=rospy.get_param('/robot_description', ''),
                config=deepcopy(rospy.get_param('/grasp', {})),
                deadline=time.monotonic()+PregraspCompensation.MAX_SECONDS,
                pose_index=pose_index)
        with self._observation_context_lock:
            self._validate_pregrasp_context(context)
            context['selection'] = deepcopy(rospy.get_param('/grasp_mode/selection', {}))
            self._compensation_visual_context = context
        return context

    def _compensation_allows_visual_loss(self, msg):
        """Allow only visual loss inside an admitted frozen contact episode.

        The caller holds the context lock. This never restores revoked
        authority and does not apply to far-field observation plans.
        """
        context = getattr(self, '_compensation_visual_context', None)
        if (context is None or msg.valid is not False
                or str(msg.diagnostic).split(':', 1)[0].strip() != 'TARGET_LOST'):
            return False
        selection = context.get('selection', {})
        if (not isinstance(selection, dict) or selection.get('mode') != 'unknown'
                or selection.get('strategy') not in ('direct', 'observe_return')
                or rospy.get_param('/grasp_mode/selection', {}) != selection
                or getattr(context['plan'], 'model_choice', '') != 'unknown_tabletop'
                or int(msg.header.stamp.to_nsec()) < context['scene'].source_ns
                or msg.header.frame_id != context['plan'].header.frame_id
                or msg.target_track_id not in ('', context['scene'].track_id)):
            return False
        return self._pregrasp_still_authorized(context)

    def _validate_pregrasp_context(self, context):
        index = context.get('pose_index', 0)
        if type(index) is not int or index not in (0, 1, 2):
            raise ObservationPathError('CONTACT_COMPENSATION_STAGE_INVALID')
        contact_endpoint = index != 0
        stage = GraspStages.APPROACH_TARGET if contact_endpoint else GraspStages.MOVE_PREGRASP
        with self._observation_context_lock:
            self._require_live_observation_task()
            authority = (
                ('manual_control_active', self._manual_control_active(), False),
                ('task_stage', getattr(self, '_pregrasp_task_stage', None), stage),
                ('control_reference_epoch_ns', self._control_reference_epoch_ns, context['epoch']),
                ('revocation', self._observation_contexts.revocation, context['revocation']),
            )
            for key, actual, expected in authority:
                if actual != expected:
                    raise ObservationPathError('PREGRASP_AUTHORITY_CHANGED: %s=%r expected=%r' %
                                               (key, actual, expected))
        cfg = context['config']
        held_approach = cfg.get('unknown_approach_held_axes_feedforward_enabled', False)
        if index == 1 and type(held_approach) is not bool:
            raise ObservationPathError('APPROACH_HELD_AXES_CONFIG_INVALID')
        if ('contact_gripper_config' in context and rospy.get_param(
                '/grasp_6d/remote/gripper_geometry', {}) != context['contact_gripper_config']):
            raise ObservationPathError('MEASURED_CONTACT_GRIPPER_CONFIG_CHANGED')
        enabled_key = ('unknown_contact_cartesian_compensation_enabled' if contact_endpoint
                       else 'pregrasp_cartesian_compensation_enabled')
        labels = tuple(prefix + ('linear 6D approach' if index == 1 else 'linear 6D grasp pose')
                       for prefix in ('planning ', 'moving '))
        if index == 1 and held_approach is True:
            labels += ('planning bounded 6D approach', 'moving bounded 6D approach')
        if contact_endpoint and (
                getattr(context['plan'], 'model_choice', '') != 'unknown_tabletop'
                or getattr(self, '_compensation_task_label', '') not in labels):
            raise ObservationPathError('CONTACT_COMPENSATION_GOAL_NOT_CURRENT')
        configuration_checks = (
            (enabled_key, lambda: cfg.get(enabled_key) is True),
            ('measured_endpoint_check_enabled', lambda: cfg.get('measured_endpoint_check_enabled') is True),
            ('grasp_configuration_changed', lambda: rospy.get_param('/grasp', {}) == cfg),
            ('observation_tracking_contract_enabled', lambda: rospy.get_param(
                '/robot/observation_tracking_contract_enabled', False) is True),
            ('driver_endpoint_trim_enabled', lambda: rospy.get_param(
                '/alicia_d_driver_node/endpoint_feedback_trim_enabled', False) is False),
        )
        for reason, accepted in configuration_checks:
            if not accepted():
                raise ObservationPathError('PREGRASP_CONFIGURATION_INVALID_OR_CHANGED: '+reason)
        if index == 1 and held_approach is True and cfg.get('contact_endpoint_precision_enabled') is not True:
            raise ObservationPathError('APPROACH_HELD_AXES_PRECISION_REQUIRED')
        context_checks = (
            ('invalid_epoch', lambda: type(context['epoch']) is int and context['epoch'] > 0),
            ('source_predates_epoch', lambda: context['scene'].source_ns >= context['epoch']),
            ('source_is_future', lambda: context['scene'].source_ns <= int(rospy.get_time()*1e9)),
            ('robot_model_changed', lambda: rospy.get_param('/robot_description', '') == context['model']),
            ('plan_digest_changed', lambda: committed_plan_digest(context['plan']) == context['scene'].digest),
            ('deadline_expired', lambda: time.monotonic() < context['deadline']),
        )
        for reason, accepted in context_checks:
            if not accepted():
                raise ObservationPathError('PREGRASP_CONTEXT_EXPIRED_OR_UNCONFIRMED: '+reason)
        status = self._fresh_actuation_status()
        if not status.startswith('CONFIRMED:'):
            raise ObservationPathError('PREGRASP_CONTEXT_EXPIRED_OR_UNCONFIRMED: actuation_status=%r' % status)

    def _pregrasp_reference(self):
        reference = self._command_reference_snapshot()
        if reference is None:
            raise ObservationPathError('PREGRASP_REFERENCE_MISSING')
        mapping = dict(zip(self.joint_names[:-1], reference.positions))
        return tuple(self._sdk_arm_words([mapping[n] for n in ARM_NAMES]))

    def _validate_pregrasp_step_path(self, context, step, trajectory, planner, *, feedforward=False):
        self._validate_pregrasp_context(context)
        index = context.get('pose_index', 0)
        cfg = context['config'] if feedforward else {}
        if feedforward and not (
                context['plan'].model_choice == 'unknown_tabletop'
                and ((index == 0 and cfg.get('unknown_pregrasp_feedforward_enabled') is True)
                     or (index == 1 and cfg.get('unknown_approach_held_axes_feedforward_enabled') is True
                         and cfg.get('unknown_contact_cartesian_compensation_enabled') is True
                         and cfg.get('contact_endpoint_precision_enabled') is True
                         and cfg.get('measured_endpoint_check_enabled') is True))):
            raise ObservationPathError('PREGRASP_FEEDFORWARD_SCOPE_INVALID')
        if feedforward and index == 1 and any(
                step.target_counts[axis] != step.baseline_counts[axis]
                for axis in PregraspCompensation.HELD_AXES):
            raise ObservationPathError('APPROACH_HELD_AXES_TARGET_CHANGED')
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
            step_bound = (32 if feedforward else 4)*Q
            if np.any(low < b-step_bound-1e-9) or np.any(high > b+step_bound+1e-9):
                if feedforward:
                    raise ObservationPathError('PREGRASP_PATH_LEAVES_TOTAL_COUNT_BOX')
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
        options = ({'contact_tracking_context': context} if index == 2 and not feedforward
                   and context['plan'].model_choice == 'unknown_tabletop' else {})
        audit = self._validate_frozen_observation_path(
            (context['scene'], context['revocation']), trajectory, planner,
            context_validator=lambda:self._validate_pregrasp_context(context), **options)
        duration = points[-1].time_from_start.to_sec()
        grace = float(audit['controller_constraints_snapshot']['goal_time'])
        if time.monotonic()+duration+grace+2. >= context['deadline']:
            raise ObservationPathError('PREGRASP_INSUFFICIENT_STEP_TIME')
        audit['pregrasp_fixed_plan_id'] = context['scene'].plan_id
        audit['compensation_pose_index'] = context.get('pose_index', 0)
        audit['compensation_method'] = (context.get('feedforward_method', 'measured_ik_following_feedforward') if feedforward
                                        else 'bounded_cartesian_steps')
        return audit

    def _run_pregrasp_feedforward(self, context, sample, planner, req, episode_param):
        """One complete correction, never appended to a spent small-step episode."""
        index = context.get('pose_index', 0)
        evidence = dict(plan_id=context['scene'].plan_id, epoch_ns=context['epoch'],
            fixed_goal_pose_index=index, execution_requested=bool(req.execute),
            real_grasp_success=False, no_implicit_rollback_or_torque_disable=True)
        if index == 1:
            evidence['execution_policy'] = 'held_axes_approach'
        recovery = None
        try:
            validate = lambda:self._validate_pregrasp_context(context)
            def validate_measured_result(result_sample):
                if self._pregrasp_reference() != tuple(sdk_counts(result_sample['sdk_positions_rad'])):
                    raise ObservationPathError('PREGRASP_REFERENCE_CHANGED')
                validate()
                now = rospy.get_time()
                if any(not 0 <= now-result_sample[key]*1e-9 <= .5
                       for key in ('sdk_stamp_ns', 'accepted_stamp_ns')):
                    raise ObservationPathError('PREGRASP_FEEDBACK_EXPIRED_DURING_VALIDATION')
            fk = SerialUrdfFk(context['model'], ARM_NAMES)
            cfg = context['config']
            if index == 1:
                if (type(cfg.get('unknown_approach_held_axes_feedforward_enabled', False)) is not bool
                        or cfg.get('unknown_approach_held_axes_feedforward_enabled') is not True
                        or context['plan'].model_choice != 'unknown_tabletop'
                        or any(cfg.get(key) is not True for key in (
                            'unknown_contact_cartesian_compensation_enabled',
                            'contact_endpoint_precision_enabled', 'measured_endpoint_check_enabled'))):
                    raise ObservationPathError('PREGRASP_FEEDFORWARD_SCOPE_INVALID')
                if not req.execute:
                    raise ObservationPathError('APPROACH_HELD_AXES_ATOMIC_EXECUTION_REQUIRED')
            elif index != 0:
                raise ObservationPathError('PREGRASP_FEEDFORWARD_SCOPE_INVALID')
            velocity_cap, timing_audit = resolve_unknown_feedforward_timing(
                cfg, context['plan'].model_choice, index,
                planner.strict_execution_max_joint_velocity_rad_s,
                getattr(planner, 'strict_execution_joint_velocity_limits_rad_s', {}))
            evidence['feedforward_timing'] = timing_audit
            goal = PoseStamped()
            goal.header.frame_id = 'base_link'
            goal.pose = deepcopy(context['plan'].poses[index])
            position_tolerance = measured_position_tolerance(cfg, context['plan'].model_choice)
            if index == 1:
                # This approach must reach the original 6 mm base contract;
                # the unknown-object 10 mm allowance must not skip the move.
                position_tolerance = min(position_tolerance, .006)
                evidence.update(approach_convergence_policy='bounded_approach_max_6mm',
                                position_tolerance_m=position_tolerance)
            recovery = PregraspFeedforward(fk, sample, goal.pose,
                position_tolerance_m=position_tolerance,
                orientation_tolerance_rad=math.radians(float(cfg['measured_endpoint_orientation_tolerance_deg'])))
            evidence.update(recovery.evidence)
            evidence.update(model_sha256=fk.model_sha256,
                sdk_stamp_ns=sample['sdk_stamp_ns'], accepted_stamp_ns=sample['accepted_stamp_ns'])
            position, angle = recovery.contract.residual(sample['accepted_positions_rad'])
            if position <= recovery.contract.tolerances[0] and angle <= recovery.contract.tolerances[1]:
                evidence.update(code='PREGRASP_MEASURED_CONVERGED', position_error_m=position,
                    orientation_error_rad=angle, steps_completed=0)
                validate_measured_result(sample)
                return CompensatePregraspResponse(True, json.dumps(evidence))
            if position > .025 or angle > recovery.contract.tolerances[1]:
                raise ObservationPathError('PREGRASP_OUTSIDE_LOCAL_CAPTURE_RANGE')
            held_axes = (True if index == 1 else
                         cfg.get('unknown_pregrasp_held_axes_feedforward_enabled', False))
            if type(held_axes) is not bool:
                raise ObservationPathError('PREGRASP_HELD_AXES_CONFIG_INVALID')
            if held_axes:
                step, report = recovery.propose_held_axes()
            else:
                start = deepcopy(planner.robot.get_current_state())
                values = dict(zip(ARM_NAMES, sample['accepted_positions_rad']))
                opening = self._observation_measured_opening()
                values.update(right_finger=opening, left_finger=-opening)
                if not set(ARM_NAMES+('right_finger',)).issubset(start.joint_state.name):
                    raise ObservationPathError('PREGRASP_FEEDFORWARD_START_INCOMPLETE')
                start.joint_state.position = [values.get(n, v) for n, v in
                    zip(start.joint_state.name, start.joint_state.position)]
                start.is_diff = False
                nominal, message = planner._plan_cartesian_from_start_state(start, goal)
                if nominal is None:
                    raise ObservationPathError(message)
                step, report = recovery.propose(nominal)
            evidence.update(report)
            context['feedforward_method'] = report['method']
            # Retiming changes only the cap. The same complete curve, held
            # axes, tracking/stop reserve and 60-second episode remain checked.
            planner.strict_execution_max_joint_velocity_rad_s = velocity_cap
            planner.strict_collinear_timing_enabled = True
            planner.observation_path_validator = lambda tr:self._validate_pregrasp_step_path(
                context, step, tr, planner, feedforward=True)
            if req.execute:
                ok, message = self._ensure_trajectory_controllers_started()
                if not ok:
                    raise ObservationPathError(message)
                ok, message = self._synchronize_trajectory_controller_to_feedback()
                if not ok:
                    raise ObservationPathError(message)
                from .contracted_trajectory import ContractedTrajectoryExecutor
                if getattr(planner, 'observation_trajectory_executor', None) is None:
                    planner.observation_trajectory_executor = ContractedTrajectoryExecutor.connect()
            def before_execute():
                validate()
                if self._pregrasp_reference() != step.baseline_counts:
                    raise ObservationPathError('PREGRASP_REFERENCE_CHANGED')
                fresh = self._endpoint_following_snapshot(fk, context['epoch'], context,
                                                         context_validator=validate)
                if max(abs(np.asarray(fresh['accepted_positions_rad'])-
                           sample['accepted_positions_rad'])) > Q+1e-12:
                    raise ObservationPathError('PREGRASP_FEEDFORWARD_START_CHANGED')
                rospy.set_param(episode_param, dict(plan_id=context['scene'].plan_id,
                    epoch_ns=str(context['epoch']), state='consumed_before_submission',
                    steps_submitted=1, fixed_goal_pose_index=index,
                    method=report['method'],
                    **({'execution_policy':'held_axes_approach'} if index == 1 else {})))
            rospy.loginfo('Pregrasp feedforward proposal: %s', json.dumps(evidence))
            ok, message = planner.plan_and_execute_joint_probe(step.positions, execute=bool(req.execute),
                start_joint_positions=[(v-2048)*Q for v in step.baseline_counts],
                before_execute=before_execute if req.execute else None)
            if not ok:
                if 'authority_rejection' in context:
                    evidence['authority_rejection'] = deepcopy(context['authority_rejection'])
                raise ObservationPathError('PREGRASP_FEEDFORWARD_PATH_REJECTED: '+message)
            if not req.execute:
                evidence.update(code='PREGRASP_PLANNED_ONLY', physical_path_authorized=False)
                return CompensatePregraspResponse(True, json.dumps(evidence))
            completed = rospy.get_time()
            recovery.committed(step, completed)
            after = self._endpoint_following_snapshot(fk, context['epoch'], context,
                after_ns=int(completed*1e9), context_validator=validate)
            ok, report = recovery.evaluate(after)
            evidence.update(report)
            if ok:
                validate_measured_result(after)
            rospy.loginfo('Pregrasp feedforward measured result: %s', json.dumps(evidence))
            return CompensatePregraspResponse(ok, json.dumps(evidence))
        except Exception as exc:
            evidence.update(code=str(exc), success=False, post_command_feedback_valid=False)
            rospy.logwarn('Pregrasp feedforward stopped: %s', json.dumps(evidence))
            return CompensatePregraspResponse(False, json.dumps(evidence))

    def _pregrasp_still_authorized(self, context):
        try:
            self._validate_pregrasp_context(context)
            return True
        except Exception as exc:
            if 'authority_rejection' not in context:
                # Save the first cause before cancellation/FAILED callbacks
                # change ownership again. This diagnostic cannot grant motion.
                record = dict(reason=str(exc), exception_type=type(exc).__name__,
                    plan_id=getattr(context.get('scene'), 'plan_id', ''),
                    pose_index=context.get('pose_index', 0),
                    expected_epoch_ns=context.get('epoch'),
                    actual_epoch_ns=getattr(self, '_control_reference_epoch_ns', None),
                    expected_revocation=context.get('revocation'),
                    actual_revocation=getattr(getattr(self, '_observation_contexts', None), 'revocation', None),
                    task_stage=getattr(self, '_pregrasp_task_stage', None),
                    task_label=getattr(self, '_compensation_task_label', ''),
                    gui_direct_mode=getattr(self, '_gui_direct_mode', None),
                    actuation_status=getattr(self, '_actuation_status', ''),
                    actuation_received_sec=getattr(self, '_actuation_received_sec', None),
                    rejected_ros_sec=float(rospy.get_time()),
                    deadline_monotonic=context.get('deadline'),
                    rejected_monotonic=time.monotonic())
                context['authority_rejection'] = record
                rospy.logwarn('Pregrasp authority rejected: %s', json.dumps(record))
            return False

    def handle_compensate_pregrasp(self, req):
        return self._handle_compensate_endpoint(req, 0)

    def handle_compensate_approach(self, req):
        return self._handle_compensate_endpoint(req, 1)

    def handle_compensate_grasp_pose(self, req):
        return self._handle_compensate_endpoint(req, 2)

    @staticmethod
    def _measured_contact_acceptance(context, sample):
        if (sample.get('opening_stamp_ns') != sample.get('accepted_stamp_ns')
                or 'accepted_gripper_opening_m' not in sample):
            raise ObservationPathError('MEASURED_CONTACT_OPENING_SOURCE_MISMATCH')
        limit = contact_height_evidence_limit(context.get('config', {}))
        options = {} if limit is None else dict(translation_evidence_limit_m=limit)
        return plan_contact_evaluator(context['plan'], context['contact_gripper_config'],
            sample['accepted_gripper_opening_m'], **options)

    def _wait_for_held_approach_phase(self):
        # Task-state publication and this RPC use independent ROS connections.
        # Wait only at admission; submission still validates exact live authority.
        deadline = time.monotonic()+1.
        labels = ('planning bounded 6D approach', 'moving bounded 6D approach')
        while True:
            self._require_live_observation_task()
            if rospy.is_shutdown() or self._manual_control_active():
                raise ObservationPathError('APPROACH_HELD_AXES_TASK_PHASE_UNAVAILABLE')
            if (getattr(self, '_pregrasp_task_stage', None) == GraspStages.APPROACH_TARGET
                    and getattr(self, '_compensation_task_label', '') in labels):
                return
            if time.monotonic() >= deadline:
                raise ObservationPathError('APPROACH_HELD_AXES_TASK_PHASE_UNAVAILABLE')
            rospy.sleep(.02)

    def _handle_compensate_endpoint(self, req, pose_index):
        with self._planner_operation_lock():
            correction, evidence, saved, planner, context = None, {}, {}, None, None
            try:
                if (pose_index == 1 and getattr(req.plan, 'model_choice', '') == 'unknown_tabletop'
                        and rospy.get_param('/grasp', {}).get(
                            'unknown_approach_held_axes_feedforward_enabled') is True):
                    self._wait_for_held_approach_phase()
                context = (self._pregrasp_context(deepcopy(req.plan)) if pose_index == 0
                           else self._pregrasp_context(deepcopy(req.plan), pose_index))
                identity = '%s:%s' % (context['epoch'], context['scene'].plan_id)
                if pose_index:
                    identity += ':pose_%d' % pose_index
                key = hashlib.sha256(identity.encode()).hexdigest()
                episode_param = '/supervisor/pregrasp_compensation_episodes/'+key
                if req.execute and rospy.get_param(episode_param, None) is not None:
                    raise ObservationPathError('PREGRASP_EPISODE_ALREADY_CONSUMED')
                fk = SerialUrdfFk(context['model'], ARM_NAMES)
                validate = lambda:self._validate_pregrasp_context(context)
                sample = self._endpoint_following_snapshot(fk, context['epoch'], context,
                                                           context_validator=validate)
                cfg = context['config']
                if pose_index == 1 and type(cfg.get(
                        'unknown_approach_held_axes_feedforward_enabled', False)) is not bool:
                    raise ObservationPathError('APPROACH_HELD_AXES_CONFIG_INVALID')
                contact_acceptance = None
                if pose_index == 2 and context['plan'].model_choice == 'unknown_tabletop':
                    context['contact_gripper_config'] = deepcopy(rospy.get_param(
                        '/grasp_6d/remote/gripper_geometry', {}))
                    contact_acceptance = self._measured_contact_acceptance(context, sample)
                correction = PregraspCompensation(fk, sample, pose_values(context['plan'].poses[pose_index]),
                    position_tolerance_m=measured_position_tolerance(cfg, getattr(context['plan'], 'model_choice', '')),
                    orientation_tolerance_rad=math.radians(float(cfg['measured_endpoint_orientation_tolerance_deg'])),
                    endpoint_acceptance=contact_acceptance,
                    joint2_response_probe_enabled=(
                        cfg.get('unknown_contact_joint2_response_probe_enabled' if pose_index
                                else 'unknown_pregrasp_joint2_response_probe_enabled', False)
                        if getattr(context['plan'], 'model_choice', '') == 'unknown_tabletop'
                        else False))
                context['initial_sdk_counts'] = correction.initial_counts
                planner = self._ensure_planner()
                if planner is None:
                    raise ObservationPathError(self._moveit_not_ready_message())
                attributes = ('observation_path_guard_required','observation_path_validator',
                    'observation_tracking_contract_required','observation_execution_authorized',
                    'observation_execution_monitor',
                    'observation_submission_revalidate','strict_execution_max_joint_velocity_rad_s',
                    'strict_collinear_timing_enabled')
                saved = {name:(hasattr(planner,name),getattr(planner,name,None)) for name in attributes}
                planner.observation_path_guard_required = True
                planner.observation_tracking_contract_required = True
                planner.strict_execution_max_joint_velocity_rad_s = min(
                    float(planner.strict_execution_max_joint_velocity_rad_s), .08)
                self._configure_controller_reference_guard(planner)
                planner.observation_execution_authorized = lambda:self._pregrasp_still_authorized(context)
                planner.observation_execution_monitor = lambda:self._contact_tracking_feedback_monitor(context)
                planner.observation_submission_revalidate = lambda trajectory,audit: (
                    self._revalidate_pregrasp_tracking_submission(context,trajectory,audit))
                if ((pose_index == 0 and getattr(context['plan'], 'model_choice', '') == 'unknown_tabletop'
                        and cfg.get('unknown_pregrasp_feedforward_enabled') is True)
                        or (pose_index == 1
                            and cfg.get('unknown_approach_held_axes_feedforward_enabled') is True)):
                    return self._run_pregrasp_feedforward(context, sample, planner, req, episode_param)
                for _ in range(PregraspCompensation.MAX_STEPS+1):
                    validate()
                    if contact_acceptance is not None:
                        correction.endpoint_acceptance = self._measured_contact_acceptance(context, sample)
                    code, step, evidence = correction.evaluate(sample)
                    evidence.update(code=code,plan_id=context['scene'].plan_id,epoch_ns=context['epoch'],
                                    execution_requested=bool(req.execute),model_sha256=fk.model_sha256,
                                    fixed_goal_pose_index=pose_index)
                    rospy.loginfo('Pregrasp Cartesian compensation: %s',json.dumps(evidence))
                    if step is None:
                        if contact_acceptance is not None:
                            validate()
                            now = rospy.get_time()
                            if any(not 0 <= now-sample[key]*1e-9 <= .5 for key in
                                   ('sdk_stamp_ns', 'accepted_stamp_ns', 'opening_stamp_ns')):
                                raise ObservationPathError('MEASURED_CONTACT_FEEDBACK_EXPIRED_DURING_VALIDATION')
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
                                steps_submitted=correction.steps+1, fixed_goal_pose_index=pose_index))
                    else:
                        before_execute = None
                    ok,message = planner.plan_and_execute_joint_probe(step.positions,execute=bool(req.execute),
                        start_joint_positions=[(v-2048)*Q for v in step.baseline_counts],
                        before_execute=before_execute)
                    if not ok:
                        if 'authority_rejection' in context:
                            evidence['authority_rejection'] = deepcopy(context['authority_rejection'])
                        raise ObservationPathError('PREGRASP_STEP_REJECTED: '+message)
                    if not req.execute:
                        evidence.update(code='PREGRASP_PLANNED_ONLY',physical_path_authorized=False)
                        return CompensatePregraspResponse(True,json.dumps(evidence))
                    completed = rospy.get_time()
                    if context.get('active_contact_tracking_contract') is not None:
                        context['contact_tracking_completed_after_ns'] = int(completed*1e9)
                    correction.committed(step,completed)
                    def validate_settle():
                        validate()
                        self._contact_tracking_feedback_monitor(context)
                    sample = self._endpoint_following_snapshot(fk,context['epoch'],context,
                        after_ns=int(completed*1e9),context_validator=validate_settle)
                raise ObservationPathError('PREGRASP_STEP_BUDGET')
            except Exception as exc:
                cleanup_hold_error = None
                if (context is not None and planner is not None
                        and context.get('active_contact_tracking_contract') is not None
                        and context.get('contact_tracking_completed_after_ns') is not None
                        and self._pregrasp_still_authorized(context)):
                    # The action has already returned, so its exception handler
                    # no longer owns cleanup of a later settle failure.
                    from trajectory_msgs.msg import JointTrajectory
                    executor = getattr(planner, 'observation_trajectory_executor', None)
                    if executor is not None:
                        try:
                            executor.hold_publisher.publish(JointTrajectory())
                        except Exception as cleanup_exc:
                            cleanup_hold_error = repr(cleanup_exc)
                if correction is not None and correction.last_evidence is not None:
                    # Do not mix a previous proposal's prediction with the
                    # current failed feedback snapshot.
                    metadata = {k:evidence[k] for k in ('plan_id','epoch_ns','execution_requested',
                                'model_sha256','fixed_goal_pose_index','authority_rejection') if k in evidence}
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
                if cleanup_hold_error is not None:
                    evidence['contact_tracking_hold_error'] = cleanup_hold_error
                rospy.logwarn('Pregrasp compensation stopped: %s',json.dumps(evidence))
                return CompensatePregraspResponse(False,json.dumps(evidence))
            finally:
                if context is not None and getattr(self, '_compensation_visual_context', None) is context:
                    with self._observation_context_lock:
                        if self._compensation_visual_context is context:
                            self._compensation_visual_context = None
                if planner is not None:
                    for name,(existed,value) in saved.items():
                        if existed:
                            setattr(planner,name,value)
                        elif hasattr(planner,name):
                            delattr(planner,name)
