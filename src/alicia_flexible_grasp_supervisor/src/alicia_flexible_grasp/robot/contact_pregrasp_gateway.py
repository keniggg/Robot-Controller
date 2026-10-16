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
from alicia_flexible_grasp.grasp.frozen_contact_handoff import (
    parse_contact_freeze_notice, selection_binding, session_binding)
from alicia_flexible_grasp.grasp.rich_plan_integrity import pose_values, is_silhouette_observation_plan
from alicia_flexible_grasp.grasp.endpoint_tolerance import measured_position_tolerance
from alicia_flexible_grasp.grasp.contact_evidence import decode_contact_evidence
from .stationary_following import ARM_NAMES, SDK_QUANTUM_RAD as Q
from .observation_path_guard import ObservationPathError, SerialUrdfFk, committed_plan_digest
from .observation_path_guard import FrozenObservationScene
from .endpoint_correction import sdk_counts
from .pregrasp_compensation import pose_matrix
from .held_axes_prediction import HeldAxesPredictionError
from .held_pregrasp_selection import select_held_pregrasp_endpoint
from .contact_sequence_precheck import (
    contact_precheck_gripper, predict_held_approach, validate_selected_pregrasp_path)
from .direct_contact_authority import DirectContactTaskAuthority, authority_bindings


POLICY = 'frozen_contact_held_approach_pregrasp_start'


class ContactPregraspGateway:
    def _direct_contact_task_owner(self):
        owner = getattr(self, '_direct_contact_authority', None)
        if owner is None:
            owner = self._direct_contact_authority = DirectContactTaskAuthority()
        return owner

    def _direct_contact_task_bindings(self):
        return authority_bindings(rospy.get_param('/grasp_mode/selection', {}),
            self._control_reference_epoch_ns, self._observation_contexts.revocation,
            rospy.get_param('/robot_description', ''), rospy.get_param('/grasp', {}))

    def _direct_contact_task_state(self, msg, terminal):
        """Called only by the real, source-ordered task-state subscriber."""
        owner = self._direct_contact_task_owner()
        try:
            bindings, budget = None, None
            if msg.active and not terminal and not getattr(self, '_gui_direct_mode', False):
                bindings = self._direct_contact_task_bindings()
                budget = rospy.get_param('/grasp', {}).get(
                    'unknown_observe_return_session_timeout_sec')
            owner.observe(active=bool(msg.active), terminal=bool(terminal),
                stamp_ns=msg.header.stamp.to_nsec(), now_ns=int(rospy.get_time()*1e9),
                monotonic_now=time.monotonic(), bindings=bindings, budget_sec=budget,
                manual=bool(getattr(self, '_gui_direct_mode', False)))
            if not msg.active:
                self._direct_contact_plan_receipts = {}
        except (AttributeError, KeyError, TypeError, ValueError) as exc:
            self._revoke_direct_contact_task(str(exc))
            rospy.logwarn('Initial-contact task budget unavailable: %s', exc)

    def _revoke_direct_contact_task(self, reason):
        owner = getattr(self, '_direct_contact_authority', None)
        if owner is not None:
            owner.revoke(reason)
        self._direct_contact_plan_receipts = {}
        self._direct_contact_publications = {}

    def _record_direct_contact_publication(self, msg):
        """Remember the actual committed publisher, including before start.

        This cache has no task budget or motion authority. Only a later live
        task and its exact service request can admit one of these receipts.
        """
        if msg.valid is not True:
            self._direct_contact_publications = {}
            return
        if msg.diagnostic != 'CONTACT_EXECUTION_PLAN' or msg.model_choice != 'unknown_tabletop':
            return
        scene = FrozenObservationScene.from_contact_plan(msg)
        try:
            binding = self._direct_contact_task_bindings()
        except (AttributeError, KeyError, TypeError, ValueError):
            # The optional direct-task budget must not change legacy local
            # receipt ingestion when no canonical mode selection exists.
            self._direct_contact_publications = {}
            return
        binding.pop('revocation')  # An inactive -> active transition changes it.
        publications = getattr(self, '_direct_contact_publications', {})
        publications[scene.plan_id] = dict(plan=deepcopy(msg), binding=binding,
            source_ns=scene.source_ns, plan_sha256=scene.digest)
        while len(publications) > 16:
            del publications[next(iter(publications))]
        self._direct_contact_publications = publications

    def _admit_initial_contact_publication(self, context):
        if context.get('direct_contact_task') is None:
            return
        publication = getattr(self, '_direct_contact_publications', {}).get(context['scene'].plan_id)
        if publication is None:
            return
        binding = self._direct_contact_task_bindings()
        binding.pop('revocation')
        validity = context['config'].get('plan_validity_sec',
            rospy.get_param('/grasp_6d/plan_validity_sec', 2.))
        age = (int(rospy.get_time()*1e9)-context['scene'].source_ns)*1e-9
        if (publication['binding'] != binding
                or publication['source_ns'] != context['scene'].source_ns
                or publication['plan_sha256'] != context['scene'].digest
                or committed_plan_digest(publication['plan']) != context['scene'].digest
                or type(validity) not in (int, float) or not math.isfinite(validity)
                or not 0. <= age <= validity):
            raise ObservationPathError('DIRECT_CONTACT_PUBLICATION_INVALID_OR_EXPIRED')
        self._capture_contact_pregrasp_plan(publication['plan'])

    def _bind_direct_contact_task(self, context):
        owner = getattr(self, '_direct_contact_authority', None)
        # No captured transition retains the original local budget. A plan,
        # parameter, or service request alone cannot create a longer lifetime.
        if owner is None or not owner.has_budget:
            return
        lease = owner.capture(bindings=self._direct_contact_task_bindings(),
            now_ns=int(rospy.get_time()*1e9), monotonic_now=time.monotonic())
        if (context['selection'].get('mode') != 'unknown'
                or context['selection'].get('strategy') not in ('direct', 'observe_return')):
            raise ObservationPathError('DIRECT_CONTACT_TASK_SCOPE_INVALID')
        context['direct_contact_task'] = lease
        context['deadline'] = lease['monotonic_deadline']

    def _validate_direct_contact_task(self, context):
        lease = context.get('direct_contact_task')
        if lease is None:
            return
        try:
            self._direct_contact_task_owner().validate(lease,
                bindings=self._direct_contact_task_bindings(),
                now_ns=int(rospy.get_time()*1e9), monotonic_now=time.monotonic())
            receipt = context.get('direct_contact_plan_receipt')
            if receipt is not None and receipt != getattr(self, '_direct_contact_plan_receipts', {}).get(
                    context['scene'].plan_id):
                raise ValueError('DIRECT_CONTACT_PUBLISHED_RECEIPT_CHANGED')
        except (AttributeError, KeyError, TypeError, ValueError) as exc:
            raise ObservationPathError(str(exc)) from exc

    def _frozen_contact_plan_cb(self, msg):
        """Cache explicit task selection only; raw Preview never enters here."""
        try:
            notice = parse_contact_freeze_notice(msg.data)
            with self._observation_context_lock:
                now_ns = int(rospy.get_time() * 1e9)
                if (notice['epoch_ns'] != getattr(self, '_control_reference_epoch_ns', None)
                        or notice['selection'] != selection_binding(rospy.get_param('/grasp_mode/selection', {}))
                        or not notice['issued_ns'] <= now_ns < notice['phase_deadline_ns']
                        or self._manual_control_active()
                        or getattr(self, '_observation_task_requires_inactive', False)):
                    raise ValueError('CONTACT_FREEZE_NOTICE_CONTEXT_CHANGED')
                notices = getattr(self, '_contact_frozen_notifications', {})
                previous = notices.get(notice['plan_id'])
                if previous is not None and previous != notice:
                    raise ValueError('CONTACT_FREEZE_NOTICE_REPLACED')
                notices[notice['plan_id']] = notice
                while len(notices) > 16:
                    del notices[next(iter(notices))]
                self._contact_frozen_notifications = notices
        except (ValueError, KeyError, TypeError, AttributeError) as exc:
            rospy.logwarn('Rejected frozen contact notice: %s', exc)

    def _admit_task_frozen_contact_receipt(self, context):
        """Bind a cached task decision to this gateway's existing live session."""
        notice = getattr(self, '_contact_frozen_notifications', {}).get(context['scene'].plan_id)
        if notice is None:
            return
        self._require_live_observation_task()
        self._validate_contact_pregrasp_context(context)
        outer = context['outer_observation_session']
        now_ns = int(rospy.get_time() * 1e9)
        if (outer is None or outer['view_session'] is None
                or notice['session'] != session_binding(outer['view_session'])
                or notice['selection'] != selection_binding(context['selection'])
                or notice['epoch_ns'] != context['epoch']
                or notice['source_ns'] != context['scene'].source_ns
                or notice['plan_sha256'] != context['scene'].digest
                or notice['session']['target_track_id'] != context['scene'].track_id
                or not notice['issued_ns'] <= now_ns < notice['phase_deadline_ns']
                or self._pregrasp_task_stage != GraspStages.MOVE_PREGRASP):
            raise ObservationPathError('CONTACT_PREGRASP_FROZEN_NOTICE_MISMATCH')
        self._capture_contact_pregrasp_plan(context['plan'])
        admissions = getattr(self, '_contact_frozen_admissions', {})
        admissions[context['scene'].plan_id] = deepcopy(notice)
        while len(admissions) > 16:
            del admissions[next(iter(admissions))]
        self._contact_frozen_admissions = admissions

    def _validate_task_frozen_contact_admission(self, context):
        notice = getattr(self, '_contact_frozen_admissions', {}).get(context['scene'].plan_id)
        if notice is None:
            return
        outer = context['outer_observation_session']
        if (outer is None or outer['view_session'] is None
                or notice['session'] != session_binding(outer['view_session'])
                or notice['selection'] != selection_binding(context['selection'])
                or notice['epoch_ns'] != context['epoch']
                or notice['source_ns'] != context['scene'].source_ns
                or notice['plan_sha256'] != context['scene'].digest):
            raise ObservationPathError('CONTACT_PREGRASP_FROZEN_ADMISSION_CHANGED')
        # The original phase deadline bound first admission, not a new motion
        # lease. Existing station prepare/retire and captured outer-session
        # checks still guard each actual submission and never renew that lease.

    def _capture_contact_pregrasp_plan(self, msg):
        """A published-plan receipt, never a joint hint or executable cache."""
        if msg.valid is not True:
            self._contact_pregrasp_receipts = {}
            self._contact_frozen_notifications = {}
            self._contact_frozen_admissions = {}
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
        owner = getattr(self, '_direct_contact_authority', None)
        if owner is not None and owner.has_budget:
            binding = self._direct_contact_task_bindings()
            lease = owner.capture(bindings=binding, now_ns=int(rospy.get_time()*1e9),
                                  monotonic_now=time.monotonic())
            task_receipts = getattr(self, '_direct_contact_plan_receipts', {})
            task_receipts[scene.plan_id] = owner.bind_plan(lease,
                plan_sha256=scene.digest, source_ns=scene.source_ns, track_id=scene.track_id,
                bindings=binding, now_ns=int(rospy.get_time()*1e9), monotonic_now=time.monotonic())
            while len(task_receipts) > 16:
                del task_receipts[next(iter(task_receipts))]
            self._direct_contact_plan_receipts = task_receipts

    def _validate_contact_pregrasp_context(self, context):
        self._validate_pregrasp_context(context)
        strict = context['config'].get('unknown_require_measured_bilateral_contact', False)
        if type(strict) is not bool:
            raise ObservationPathError('CONTACT_EVIDENCE_POLICY_CHANGED')
        if strict:
            evidence = decode_contact_evidence(context['plan'], required=False)
            if evidence is not None and evidence.get('schema_version') == 3:
                raise ObservationPathError('MEASURED_BILATERAL_CONTACT_REQUIRED')
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
            compute_deadline = context['deadline']
            origin = getattr(self, '_observe_return_origin', None)
            if origin is None or origin.get('selection') != context['selection']:
                context['outer_observation_session'] = None
                self._bind_direct_contact_task(context)
                self._capture_contact_pregrasp_deadlines(context, compute_deadline)
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
            # This service moves the full nominal trajectory, not a bounded
            # local correction. Only the authenticated original outer session
            # can fund that motion; computation retains its separate 60 s cap.
            context['deadline'] = origin['deadline']
            if session is not None:
                # Convert once only; neither another sample nor a ROS-clock
                # delay may renew the original absolute outer deadline.
                context['deadline'] = min(context['deadline'], time.monotonic()+
                    (session['deadline_ns']-int(rospy.get_time()*1e9))*1e-9)
            self._capture_contact_pregrasp_deadlines(context, compute_deadline)

    def _capture_contact_pregrasp_deadlines(self, context, compute_deadline):
        mono, ros_ns = time.monotonic(), int(rospy.get_time()*1e9)
        execution_deadline = context['deadline']
        if (any(type(value) not in (int, float) or not math.isfinite(value)
                for value in (mono, compute_deadline, execution_deadline))
                or not mono < min(compute_deadline, execution_deadline) or ros_ns <= 0):
            raise ObservationPathError('CONTACT_PREGRASP_DEADLINE_INVALID')
        context['compute_deadline'] = min(compute_deadline, execution_deadline)
        # Shared visual/context readers keep the original computation limit
        # until the fully checked trajectory reaches before_execute.
        context['deadline'] = context['compute_deadline']
        outer = context['outer_observation_session']
        session = None if outer is None else outer['view_session']
        direct = context.get('direct_contact_task')
        context['contact_deadlines'] = dict(
            compute_deadline=context['compute_deadline'], execution_deadline=execution_deadline,
            execution_admitted=False,
            ros_deadline_ns=(session['deadline_ns'] if session is not None else
                             direct['ros_deadline_ns'] if direct is not None else None),
            last_monotonic=mono, last_ros_ns=ros_ns,
            source=('captured_outer_session' if outer is not None else
                    'captured_initial_contact_task' if direct is not None else 'original_local_budget'))

    def _contact_pregrasp_remaining(self, context, *, compute=False):
        """Read fixed clocks; neither progress nor another check renews them."""
        with self._observation_context_lock:
            mono, ros_ns = time.monotonic(), int(rospy.get_time()*1e9)
            budget = context.get('contact_deadlines')
            if budget is None:
                # Existing directly constructed local contexts gain no budget.
                left = context['deadline']-mono
            else:
                if (not math.isfinite(mono) or mono < budget['last_monotonic']
                        or ros_ns < budget['last_ros_ns']):
                    raise ObservationPathError('CONTACT_PREGRASP_CLOCK_ROLLBACK')
                expected = budget['execution_deadline'] if budget['execution_admitted'] else budget['compute_deadline']
                if (context['deadline'] != expected
                        or context['compute_deadline'] != budget['compute_deadline']):
                    raise ObservationPathError('CONTACT_PREGRASP_DEADLINE_CHANGED')
                budget['last_monotonic'], budget['last_ros_ns'] = mono, ros_ns
                left = budget['execution_deadline']-mono
                if budget['ros_deadline_ns'] is not None:
                    left = min(left, (budget['ros_deadline_ns']-ros_ns)*1e-9)
                if compute:
                    left = min(left, budget['compute_deadline']-mono)
            if not math.isfinite(left) or left <= 0.:
                raise ObservationPathError('CONTACT_PREGRASP_' +
                    ('COMPUTE_' if compute else '') + 'DEADLINE_EXPIRED')
            return left

    def _contact_pregrasp_execution_time_check(self, context, path, report, audit, stage):
        self._validate_contact_pregrasp_context(context)
        self._contact_pregrasp_remaining(context, compute=True)
        duration = path.joint_trajectory.points[-1].time_from_start.to_sec()
        grace = report['controller_constraints_snapshot']['goal_time']
        if (type(grace) not in (int, float) or not math.isfinite(grace) or not 0. < grace <= 30.
                or not math.isfinite(duration) or duration <= 0.):
            raise ObservationPathError('CONTACT_PREGRASP_EXECUTION_TIME_INVALID')
        left = self._contact_pregrasp_remaining(context)
        timing = dict(stage=stage, trajectory_duration_sec=duration,
            controller_goal_time_sec=grace, executor_margin_sec=2.,
            required_execution_sec=duration+grace+2., execution_remaining_sec=left,
            compute_remaining_sec=self._contact_pregrasp_remaining(context, compute=True))
        audit['execution_timing'] = timing
        if timing['required_execution_sec'] >= left:
            timing['rejection_reason'] = 'CONTACT_PREGRASP_INSUFFICIENT_EXECUTION_TIME'
            raise ObservationPathError(timing['rejection_reason'])

    def _validate_contact_pregrasp_session(self, context):
        with self._observation_context_lock:
            self._contact_pregrasp_remaining(context)
            outer = context['outer_observation_session']
            current = getattr(self, '_observe_return_origin', None)
            if outer is None:
                if current is not None and current.get('selection') == context['selection']:
                    raise ObservationPathError('CONTACT_PREGRASP_SESSION_APPEARED')
                self._validate_direct_contact_task(context)
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
                if context['scene'].plan_id not in getattr(self, '_contact_pregrasp_receipts', {}):
                    self._admit_initial_contact_publication(context)
                    self._admit_task_frozen_contact_receipt(context)
                receipt = deepcopy(getattr(self, '_contact_pregrasp_receipts', {}).get(
                    context['scene'].plan_id))
            if receipt is not None:
                self._validate_task_frozen_contact_admission(context)
                if receipt != expected:
                    raise ObservationPathError('CONTACT_PREGRASP_PUBLISHED_PLAN_MISMATCH')
                direct = context.get('direct_contact_task')
                if direct is not None:
                    wanted = dict(task_key=direct['task_key'], plan_sha256=context['scene'].digest,
                        source_ns=context['scene'].source_ns, target_track_id=context['scene'].track_id)
                    actual = getattr(self, '_direct_contact_plan_receipts', {}).get(context['scene'].plan_id)
                    if actual != wanted:
                        raise ObservationPathError('DIRECT_CONTACT_PUBLISHED_TASK_RECEIPT_REQUIRED')
                    context['direct_contact_plan_receipt'] = deepcopy(actual)
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

    def _contact_pregrasp_compute_guard(self, context, audit):
        """Bound arithmetic without repeating parameter RPCs for each derivative.

        Local revocation/deadline checks run on every call. The unchanged full
        context check runs at least every 100 ms during computation; all the
        caller's original stage, return and physical submission checks remain.
        This callback is not an execution permission or a feedback snapshot.
        """
        evidence = dict(local_checks=0, full_checks=0,
                        full_check_interval_sec=.1, full_check_elapsed_sec=0.)
        audit['compute_guard'] = evidence
        last_full = [None]

        def local_remaining():
            evidence['local_checks'] += 1
            with self._observation_context_lock:
                self._require_live_observation_task()
                if (rospy.is_shutdown() or getattr(self, '_gui_direct_mode', False)
                        or self._pregrasp_task_stage != GraspStages.MOVE_PREGRASP
                        or self._control_reference_epoch_ns != context['epoch']
                        or self._observation_contexts.revocation != context['revocation']):
                    raise ObservationPathError('CONTACT_PREGRASP_COMPUTE_AUTHORITY_CHANGED')
                outer = context['outer_observation_session']
                current = getattr(self, '_observe_return_origin', None)
                if outer is None:
                    if current is not None and current.get('selection') == context['selection']:
                        raise ObservationPathError('CONTACT_PREGRASP_SESSION_APPEARED')
                else:
                    origin = outer['origin']
                    if (current is not origin or origin.get('identity') != outer['identity']
                            or origin.get('deadline') != outer['origin_deadline']
                            or origin.get('epoch') != context['epoch']
                            or origin.get('revocation') != context['revocation']
                            or origin.get('selection') != context['selection']
                            or origin.get('view_session') != outer['view_session']):
                        raise ObservationPathError('CONTACT_PREGRASP_SESSION_CHANGED')
                    session = outer['view_session']
                    if (session is not None
                            and int(rospy.get_time()*1e9) >= session['deadline_ns']):
                        raise ObservationPathError('CONTACT_PREGRASP_SESSION_EXPIRED')
            if not self._fresh_actuation_status().startswith('CONFIRMED:'):
                raise ObservationPathError('CONTACT_PREGRASP_COMPUTE_ACTUATION_UNCONFIRMED')
            return self._contact_pregrasp_remaining(context, compute=True)

        def remaining():
            left = local_remaining()
            now = time.monotonic()
            if last_full[0] is None or now-last_full[0] >= .1:
                started = now
                evidence['full_checks'] += 1
                try:
                    self._validate_contact_pregrasp_context(context)
                finally:
                    evidence['full_check_elapsed_sec'] += time.monotonic()-started
                last_full[0] = started
                left = local_remaining()
            return left
        return remaining

    def handle_contact_pregrasp(self, req):
        audit = dict(execution_policy=POLICY, plan_id=str(req.plan.plan_id),
                     execution_requested=bool(req.execute), real_grasp_success=False,
                     hardware_response_verified=False, endpoint_confirmation_required=True,
                     stage_timings=[])
        def timed(stage, action):
            started = time.monotonic()
            completed = False
            try:
                result = action()
                completed = True
                return result
            finally:
                audit['stage_timings'].append(dict(stage=stage,
                    elapsed_sec=time.monotonic()-started, completed=completed))
        planner, saved, context = None, {}, None
        with self._planner_operation_lock():
            try:
                self._wait_for_contact_pregrasp_phase()
                context = self._pregrasp_context(deepcopy(req.plan), 0)
                context['geometry'] = deepcopy(rospy.get_param('/grasp_6d/remote/gripper_geometry', {}))
                context['gripper_config'] = deepcopy(rospy.get_param('/gripper', {}))
                self._bind_contact_pregrasp_session(context)
                budget = context['contact_deadlines']
                audit['deadline_budget'] = dict(
                    compute_deadline_monotonic=budget['compute_deadline'],
                    execution_deadline_monotonic=budget['execution_deadline'],
                    execution_deadline_ros_ns=budget['ros_deadline_ns'],
                    source=budget['source'])
                self._wait_for_contact_pregrasp_receipt(context)
                validate = lambda: self._validate_contact_pregrasp_context(context)
                def remaining():
                    validate()
                    return self._contact_pregrasp_remaining(context, compute=True)
                remaining()
                compute_remaining = self._contact_pregrasp_compute_guard(context, audit)
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
                        monotonic_deadline=budget['execution_deadline'],
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
                seed_path, reason = timed('moveit_seed', lambda:
                    planner._plan_pose_from_start_state(real_start, goal,
                        planning_time_limit_sec=min(remaining(), 2.)))
                if seed_path is None:
                    raise ObservationPathError('CONTACT_PREGRASP_SEED_FAILED: '+str(reason))
                jt = seed_path.joint_trajectory
                if set(jt.joint_names) != set(ARM_NAMES) or len(jt.joint_names) != 6 or not jt.points:
                    raise ObservationPathError('CONTACT_PREGRASP_SEED_INVALID')
                seed = dict(zip(jt.joint_names, jt.points[-1].positions))
                if len(seed) != 6:
                    raise ObservationPathError('CONTACT_PREGRASP_SEED_INVALID')
                target, selection = timed('held_endpoint_selection', lambda:
                    select_held_pregrasp_endpoint(fk, sample,
                    pose_values(context['plan'].poses[0]), pose_values(context['plan'].poses[1]),
                    [seed[n] for n in ARM_NAMES],
                    position_tolerance_m=measured_position_tolerance(cfg, 'unknown_tabletop'),
                    orientation_tolerance_rad=math.radians(float(cfg['measured_endpoint_orientation_tolerance_deg'])),
                    deadline_check=compute_remaining))
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
                    remaining()
                    audit['fresh_fixed_endpoint_prediction'] = proof
                    return fresh

                checked_execution = []
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
                    audit['selected_path_prediction'] = timed('selected_path_validation', lambda:
                        validate_selected_pregrasp_path(
                            path, fk, fresh, context['scene'], gripper, opening, planner,
                            current_state, self._contact_command_collision_check, compute_remaining))
                    result = timed('frozen_path_validation', lambda:
                        self._validate_frozen_observation_path(
                            (context['scene'], context['revocation']), path, planner,
                            context_validator=validate))
                    self._contact_pregrasp_execution_time_check(context, path, result, audit,
                                                               'validated_path')
                    checked_execution[:] = [path, result]
                    audit['complete_execution_path_checked'] = True
                    return result

                if not req.execute:
                    command_start = self._contact_pregrasp_start(real_start, sample['sdk_positions_rad'])
                    path, reason = timed('moveit_joint_plan', lambda:
                        planner._plan_joint_from_start_state(command_start, target,
                            planning_time_limit_sec=min(remaining(), 2.)))
                    if path is None:
                        raise ObservationPathError('CONTACT_PREGRASP_JOINT_PLAN_FAILED: '+str(reason))
                    audit['selected_path_prediction'] = timed('selected_path_validation', lambda:
                        validate_selected_pregrasp_path(
                            path, fk, sample, context['scene'], gripper, opening, planner,
                            real_start, self._contact_command_collision_check, compute_remaining))
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
                def submission_revalidate(path, report):
                    remaining()
                    self._revalidate_observation_contract_submission(
                        (context['scene'],context['revocation']), path, report, context_validator=validate)
                    # The final transport can wait after path validation.
                    # Recheck computation and the same final timed trajectory
                    # immediately before send, against the original clocks.
                    self._contact_pregrasp_execution_time_check(context, path, report, audit,
                                                               'before_send')
                planner.observation_submission_revalidate = submission_revalidate
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
                    if len(checked_execution) != 2:
                        raise ObservationPathError('CONTACT_PREGRASP_CHECKED_PATH_REQUIRED')
                    self._contact_pregrasp_execution_time_check(
                        context, *checked_execution, audit, 'before_execute')
                    if rospy.get_param(consumed, None) is not None:
                        raise ObservationPathError('CONTACT_PREGRASP_ALREADY_SUBMITTED')
                    rospy.set_param(consumed, dict(plan_id=context['scene'].plan_id,
                        plan_sha256=context['scene'].digest, source_ns=str(context['scene'].source_ns),
                        epoch_ns=str(context['epoch']), state='consumed_before_submission',
                        command_goal_positions_rad=target.tolist(), execution_policy=POLICY))
                    with self._observation_context_lock:
                        remaining()
                        budget['execution_admitted'] = True
                        context['deadline'] = budget['execution_deadline']
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
                if 'deadline_budget' in audit:
                    audit['deadline_budget']['rejection_reason'] = str(exc)
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
