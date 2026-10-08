"""Task-bound observation return to a measured, immutable initial arm pose.

Capture never commands hardware. Return replans from the current successful
SDK reference; it neither replays the outward path nor reuses its old targets.
"""
from copy import deepcopy
import hashlib
import json
import math
import re
import time

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_matrix

from alicia_flexible_grasp_supervisor.srv import CompensatePregraspResponse
from alicia_flexible_grasp.grasp.endpoint_tolerance import measured_position_tolerance
from alicia_flexible_grasp.grasp.rich_plan_integrity import is_silhouette_observation_plan
from alicia_flexible_grasp.grasp.grasp_state_machine import GraspStages
from .observation_path_guard import (
    FrozenObservationScene, ObservationPathError, SerialUrdfFk, committed_plan_digest,
)
from .stationary_following import ARM_NAMES, SDK_QUANTUM_RAD


CAPTURE_LABEL = 'capturing initial observation pose'
RETURN_LABEL = 'returning to initial observation pose'
ADMIT_VIEW_LABEL = 'admitting supplemental observation view'


def _bounded_number(value, maximum, label):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or not 0. < value <= maximum):
        raise ObservationPathError('OBSERVE_RETURN_INVALID_' + label)
    return float(value)


def _digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


class ObserveReturnGateway:
    """One capture and one strict return submission per observation session."""

    def _observe_return_selection(self):
        selection = deepcopy(rospy.get_param('/grasp_mode/selection', {}))
        if (not isinstance(selection, dict) or selection.get('mode') != 'unknown'
                or selection.get('strategy') != 'observe_return'
                or type(selection.get('generation')) is not int
                or selection['generation'] < 1):
            raise ObservationPathError('OBSERVE_RETURN_SELECTION_INVALID')
        stamp = selection.get('stamp_ns')
        if isinstance(stamp, str) and stamp.isdecimal():
            stamp = int(stamp)
        if type(stamp) is not int or not 0 < stamp <= int(rospy.get_time()*1e9):
            raise ObservationPathError('OBSERVE_RETURN_SELECTION_STAMP_INVALID')
        return selection

    def _validate_observe_return_context(self, context, *, returning=False, require_stage=True):
        if returning and is_silhouette_observation_plan(context['plan']):
            raise ObservationPathError('DIAGNOSTIC_OBSERVATION_RETURN_FORBIDDEN')
        with self._observation_context_lock:
            self._require_live_observation_task()
            self._observation_contexts.validate_capture(
                context.get('active_view_scene', context['scene']), context['revocation'])
            if self._manual_control_active():
                raise ObservationPathError('OBSERVE_RETURN_MANUAL_CONTROL_ACTIVE')
            if require_stage and (getattr(self, '_pregrasp_task_stage', None) != GraspStages.MOVE_PREGRASP
                    or getattr(self, '_compensation_task_label', '') !=
                    (RETURN_LABEL if returning else CAPTURE_LABEL)):
                raise ObservationPathError('OBSERVE_RETURN_TASK_STAGE_CHANGED')
            if self._control_reference_epoch_ns != context['epoch']:
                raise ObservationPathError('OBSERVE_RETURN_CONTROL_EPOCH_CHANGED')
        if self._observe_return_selection() != context['selection']:
            raise ObservationPathError('OBSERVE_RETURN_SELECTION_CHANGED')
        now = float(rospy.get_time())
        if (not math.isfinite(now) or int(now*1e9) < context['scene'].source_ns
                or type(context['epoch']) is not int or context['epoch'] <= 0
                or context['scene'].source_ns < context['epoch']
                or context['scene'].source_ns < int(context['selection']['stamp_ns'])):
            raise ObservationPathError('OBSERVE_RETURN_SOURCE_EPOCH_INVALID')
        if ('origin_sample' in context
                and int(now*1e9) < context['origin_sample']['accepted_stamp_ns']):
            raise ObservationPathError('OBSERVE_RETURN_CLOCK_PREDATES_ORIGIN')
        deadline = min(context['deadline'], context.get('return_deadline', context['deadline']))
        session = context.get('view_session')
        if (session is not None and int(now * 1e9) >= session['deadline_ns']):
            raise ObservationPathError('OBSERVE_RETURN_VIEW_SESSION_EXPIRED')
        if time.monotonic() >= deadline:
            raise ObservationPathError('OBSERVE_RETURN_SESSION_EXPIRED')
        if (rospy.get_param('/robot_description', '') != context['model']
                or rospy.get_param('/grasp', {}) != context['config']
                or committed_plan_digest(context['plan']) != context['scene'].digest):
            raise ObservationPathError('OBSERVE_RETURN_BOUND_INPUT_CHANGED')
        if (rospy.get_param('/robot/observation_tracking_contract_enabled', False) is not True
                or rospy.get_param('/alicia_d_driver_node/endpoint_feedback_trim_enabled', False) is not False):
            raise ObservationPathError('OBSERVE_RETURN_EXECUTION_CONTRACT_UNAVAILABLE')
        if 'origin_joints' in context and _digest(dict(joint_names=list(ARM_NAMES),
                joint_positions_rad=list(context['origin_joints']),
                model_sha256=context['fk'].model_sha256)) != context['goal_digest']:
            raise ObservationPathError('OBSERVE_RETURN_ORIGIN_GOAL_CHANGED')
        if not self._fresh_actuation_status().startswith('CONFIRMED:'):
            raise ObservationPathError('OBSERVE_RETURN_ACTUATION_UNCONFIRMED')

    def _await_observe_return_active(self, scene, selection, epoch):
        """Admit only the pending activation for this source and authority."""
        deadline = time.monotonic()+.5
        with self._observation_context_lock:
            initial_active = self._observation_contexts.active
            initial_revocation = self._observation_contexts.revocation
        source_floor = max(scene.source_ns, int(selection['stamp_ns']), epoch)
        if scene.source_ns < max(int(selection['stamp_ns']), epoch):
            raise ObservationPathError('OBSERVE_RETURN_SOURCE_EPOCH_INVALID')
        while True:
            if self._observe_return_selection() != selection:
                raise ObservationPathError('OBSERVE_RETURN_SELECTION_CHANGED')
            with self._observation_context_lock:
                if self._control_reference_epoch_ns != epoch:
                    raise ObservationPathError('OBSERVE_RETURN_CONTROL_EPOCH_CHANGED')
                if self._manual_control_active():
                    raise ObservationPathError('OBSERVE_RETURN_MANUAL_CONTROL_ACTIVE')
                if (not self._observation_task_connection_available()
                        or getattr(self, '_observation_task_requires_inactive', False)):
                    raise ObservationPathError('OBSERVE_RETURN_ACTIVE_TASK_UNAVAILABLE')
                active = self._observation_contexts.active
                expected_revocation = initial_revocation + int(not initial_active and active)
                if self._observation_contexts.revocation != expected_revocation:
                    raise ObservationPathError('OBSERVE_RETURN_TASK_ACTIVATION_CHANGED')
                remaining = deadline-time.monotonic()
                if remaining <= 0.:
                    raise ObservationPathError('OBSERVE_RETURN_ACTIVE_TASK_NOT_RECEIVED')
                if (active and not self._observation_contexts.blocked
                        and getattr(self, '_observation_task_state_stamp_ns', 0) >= source_floor):
                    self._require_live_observation_task()
                    return expected_revocation
                condition = getattr(self, '_observation_context_condition', None)
                if condition is not None:
                    condition.wait(min(.05, remaining))
            if condition is None:
                time.sleep(min(.05, remaining))

    def _await_observe_return_stage(self, context, *, returning=False):
        # Task state and the service request use separate ROS connections. Wait
        # only for this admitted session's label; any other authority change
        # rejects immediately and never borrows a later task's active state.
        deadline = min(context['deadline'], time.monotonic()+1.)
        label = RETURN_LABEL if returning else CAPTURE_LABEL
        while True:
            self._validate_observe_return_context(context, returning=returning, require_stage=False)
            with self._observation_context_lock:
                remaining = deadline-time.monotonic()
                if remaining <= 0.:
                    raise ObservationPathError('OBSERVE_RETURN_TASK_STAGE_CHANGED_OR_NOT_RECEIVED')
                if (getattr(self, '_pregrasp_task_stage', None) == GraspStages.MOVE_PREGRASP
                        and getattr(self, '_compensation_task_label', '') == label):
                    return
                condition = getattr(self, '_observation_context_condition', None)
                if condition is not None:
                    condition.wait(min(.05, remaining))
            if condition is None:
                time.sleep(min(.05, remaining))

    def _observe_return_reference(self):
        reference = self._command_reference_snapshot()
        names = tuple(self.joint_names[:-1])
        if (reference is None or len(names) != 6 or set(names) != set(ARM_NAMES)
                or len(reference.positions) != 6):
            raise ObservationPathError('OBSERVE_RETURN_REFERENCE_UNAVAILABLE')
        mapping = dict(zip(names, reference.positions))
        return tuple(float(mapping[name]) for name in ARM_NAMES)

    def _observe_return_view_session_cb(self, message):
        """Receive a task-owned proposal; a topic message alone never changes a scene."""
        try:
            raw = json.loads(message.data)
            if not isinstance(raw, dict) or raw.get('phase') != 'observing' or raw.get('version') != 2:
                return
            with self._observation_context_lock:
                context = getattr(self, '_observe_return_origin', None)
                if context is None:
                    return
                session = self._validated_observe_return_view_session(context, raw)
                previous = context.get('view_session')
                if previous is None:
                    if (session['view_index'] != 1 or session['view_plan_id'] != context['scene'].plan_id
                            or session['view_source_stamp_ns'] != context['scene'].source_ns
                            or session['previous_gain_sha256'] != '0' * 64):
                        raise ObservationPathError('OBSERVE_RETURN_INITIAL_VIEW_BINDING_INVALID')
                    context['view_session'] = session
                elif session == previous:
                    return
                elif (session['view_index'] == previous['view_index'] + 1
                        and session['stamp_ns'] > previous['stamp_ns']
                        and session['view_source_stamp_ns'] > previous['view_source_stamp_ns']
                        and session['view_plan_id'] != previous['view_plan_id']
                        and session['view_request_id'] != previous['view_request_id']
                        and session['previous_gain_sha256'] != '0' * 64):
                    context['pending_view_session'] = session
                else:
                    raise ObservationPathError('OBSERVE_RETURN_VIEW_UPDATE_NOT_SEQUENTIAL')
                condition = getattr(self, '_observation_context_condition', None)
                if condition is not None:
                    condition.notify_all()
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            rospy.logwarn('Rejected supplemental observation session: %s', exc)

    def _validated_observe_return_view_session(self, context, value):
        session = deepcopy(value)
        def integer(item, name, minimum=1):
            if isinstance(item, str) and item.isdecimal():
                item = int(item)
            if type(item) is not int or item < minimum:
                raise ObservationPathError('OBSERVE_RETURN_VIEW_' + name.upper() + '_INVALID')
            return item
        for key in ('stamp_ns', 'deadline_ns', 'origin_snapshot_stamp_ns', 'view_source_stamp_ns'):
            session[key] = integer(session[key], key)
        for key in ('version', 'view_index', 'max_observations'):
            session[key] = integer(session[key], key)
        session['target_epoch'] = integer(session['target_epoch'], 'target_epoch', 0)
        for key in ('session_id', 'origin_plan_id', 'target_track_id', 'view_plan_id', 'view_request_id'):
            if (not isinstance(session[key], str) or not session[key]
                    or any(c.isspace() for c in session[key])):
                raise ObservationPathError('OBSERVE_RETURN_VIEW_IDENTIFIER_INVALID')
        if not isinstance(session.get('previous_gain_sha256'), str) or not re.fullmatch(
                '[0-9a-f]{64}', session['previous_gain_sha256']):
            raise ObservationPathError('OBSERVE_RETURN_VIEW_GAIN_DIGEST_INVALID')
        selection, expected = deepcopy(session['selection']), deepcopy(context['selection'])
        selection['stamp_ns'] = integer(selection['stamp_ns'], 'selection_stamp')
        expected['stamp_ns'] = integer(expected['stamp_ns'], 'selection_stamp')
        session['selection'] = selection
        now_ns = int(rospy.get_time() * 1e9)
        configured_budget = context['config'].get('unknown_observe_return_max_observations', 3)
        if (type(configured_budget) is not int or not 1 <= configured_budget <= 3
                or session['version'] != 2 or session['phase'] != 'observing'
                or selection != expected or session['session_id'] != context['session_id']
                or session['origin_plan_id'] != context['scene'].plan_id
                or session['origin_snapshot_stamp_ns'] != context['scene'].source_ns
                or session['target_track_id'] != context['scene'].track_id
                or not 1 <= session['view_index'] <= session['max_observations'] <= configured_budget
                or not context['scene'].source_ns <= session['view_source_stamp_ns'] <= session['stamp_ns'] <= now_ns < session['deadline_ns']
                or session['deadline_ns'] - session['stamp_ns'] > 600_000_000_000
                or integer(session.get('return_verified_stamp_ns', 0), 'return_stamp', 0) != 0
                or integer(session.get('contact_deadline_ns', 0), 'contact_deadline', 0) != 0):
            raise ObservationPathError('OBSERVE_RETURN_VIEW_SESSION_BINDING_INVALID')
        previous = context.get('view_session')
        if previous is not None and any(session[key] != previous[key] for key in (
                'session_id', 'selection', 'origin_plan_id', 'origin_snapshot_stamp_ns',
                'target_epoch', 'target_track_id', 'deadline_ns', 'max_observations')):
            raise ObservationPathError('OBSERVE_RETURN_VIEW_IMMUTABLE_BINDING_CHANGED')
        return session

    def handle_admit_observe_return_view(self, req):
        """Authorize one complete supplemental scene without any hardware command."""
        evidence = dict(physical_motion_submitted=False, real_grasp_success=False)
        with self._planner_operation_lock():
            try:
                if req.execute is not True:
                    raise ObservationPathError('OBSERVE_RETURN_VIEW_ADMISSION_REQUIRES_EXPLICIT_COMMIT')
                plan = deepcopy(req.plan)
                if (getattr(plan, 'model_choice', '') != 'unknown_tabletop'
                        or is_silhouette_observation_plan(plan)):
                    raise ObservationPathError('OBSERVE_RETURN_VIEW_ORDINARY_UNKNOWN_PLAN_REQUIRED')
                scene = FrozenObservationScene.from_plan(plan)
                context = getattr(self, '_observe_return_origin', None)
                if context is None:
                    raise ObservationPathError('OBSERVE_RETURN_ORIGIN_MISSING')
                # Topic/session and service use independent connections. Wait
                # only for this plan's explicit proposal and admission stage.
                deadline = min(context['deadline'], time.monotonic() + 1.)
                while True:
                    self._validate_observe_return_context(context, require_stage=False)
                    with self._observation_context_lock:
                        pending = context.get('pending_view_session')
                        ready = (isinstance(pending, dict) and pending['view_plan_id'] == scene.plan_id
                            and getattr(self, '_pregrasp_task_stage', None) == GraspStages.MOVE_PREGRASP
                            and getattr(self, '_compensation_task_label', '') == ADMIT_VIEW_LABEL)
                        if ready:
                            break
                        remaining = deadline - time.monotonic()
                        if remaining <= 0.:
                            raise ObservationPathError('OBSERVE_RETURN_VIEW_REQUEST_OR_STAGE_UNAVAILABLE')
                        condition = getattr(self, '_observation_context_condition', None)
                        if condition is not None:
                            condition.wait(min(.05, remaining))
                    if condition is None:
                        time.sleep(min(.05, remaining))
                with self._observation_context_lock:
                    self._validate_observe_return_context(context, require_stage=False)
                    if context.get('submitted'):
                        raise ObservationPathError('OBSERVE_RETURN_ALREADY_SUBMITTED')
                    session = self._validated_observe_return_view_session(context, pending)
                    previous = context['view_session']
                    current_scene = context.get('active_view_scene', context['scene'])
                    if (session['view_source_stamp_ns'] != scene.source_ns
                            or session['view_plan_id'] != scene.plan_id
                            or scene.track_id != context['scene'].track_id
                            or previous['view_plan_id'] != current_scene.plan_id
                            or session['view_index'] != previous['view_index'] + 1
                            or context.get('pending_view_session') != pending
                            or getattr(self, '_pregrasp_task_stage', None) != GraspStages.MOVE_PREGRASP
                            or getattr(self, '_compensation_task_label', '') != ADMIT_VIEW_LABEL):
                        raise ObservationPathError('OBSERVE_RETURN_VIEW_PLAN_BINDING_INVALID')
                    validity = _bounded_number(context['config'].get('plan_validity_sec',
                        rospy.get_param('/grasp_6d/plan_validity_sec', 2.)), 600., 'PLAN_VALIDITY')
                    if not 0. <= float(rospy.get_time()) - scene.source_ns * 1e-9 <= validity:
                        raise ObservationPathError('OBSERVE_RETURN_VIEW_SOURCE_STALE_OR_FUTURE')
                    self._observation_contexts.admit_task_view(scene, current_scene, context['revocation'])
                    context['active_view_scene'] = scene
                    context['admitted_view_scenes'] = tuple(context.get('admitted_view_scenes', (context['scene'],))) + (scene,)
                    context['view_session'] = session
                    context.pop('pending_view_session', None)
                    evidence.update(code='OBSERVE_RETURN_VIEW_ADMITTED', session_id=context['session_id'],
                        plan_id=scene.plan_id, plan_digest=scene.digest, target_track_id=scene.track_id,
                        origin_plan_id=context['scene'].plan_id, origin_plan_digest=context['scene'].digest,
                        origin_goal_sha256=context['goal_digest'], previous_view_plan_id=current_scene.plan_id,
                        view_index=session['view_index'], view_source_stamp_ns=scene.source_ns,
                        view_request_id=session['view_request_id'], previous_gain_sha256=session['previous_gain_sha256'],
                        deadline_ns=session['deadline_ns'], epoch_ns=context['epoch'])
                    return CompensatePregraspResponse(True, json.dumps(evidence, allow_nan=False))
            except Exception as exc:
                evidence.update(code=str(exc), success=False)
                return CompensatePregraspResponse(False, json.dumps(evidence, allow_nan=False))

    def _observe_return_sample(self, context, *, returning=False, after_ns=0):
        validate = lambda: self._validate_observe_return_context(context, returning=returning)
        sample = self._endpoint_following_snapshot(
            context['fk'], context['epoch'], context, after_ns=after_ns,
            context_validator=validate)
        validate()
        now = float(rospy.get_time())
        if (sample.get('measurement_kind') != 'stationary_sdk_following_not_executed_endpoint'
                or sample.get('epoch_ns') != context['epoch']
                or sample.get('model_sha256') != context['fk'].model_sha256
                or tuple(sample.get('joint_names', ())) != ARM_NAMES
                or sample.get('accepted_sample_count', 0) < 2
                or sample.get('sdk_sample_count', 0) < 2):
            raise ObservationPathError('OBSERVE_RETURN_MEASUREMENT_CONTEXT_INVALID')
        stamps = [sample.get(key) for key in (
            'stationary_window_start_ns', 'stationary_window_end_ns',
            'sdk_stamp_ns', 'accepted_stamp_ns')]
        if (any(type(stamp) is not int or stamp < context['epoch'] for stamp in stamps)
                or stamps[0] <= after_ns or stamps[1]-stamps[0] < 300_000_000
                or stamps[1] > min(stamps[2:])
                or any(not 0. <= now-stamp*1e-9 <= .5 for stamp in stamps[2:])):
            raise ObservationPathError('OBSERVE_RETURN_MEASUREMENT_STALE_OR_PRE_COMMAND')
        for key in ('accepted_positions_rad', 'sdk_positions_rad'):
            values = np.asarray(sample.get(key), dtype=float)
            if values.shape != (6,) or not np.all(np.isfinite(values)):
                raise ObservationPathError('OBSERVE_RETURN_MEASUREMENT_JOINTS_INVALID')
            context['fk'](values)  # Existing URDF joint limits, including measured origin.
        return sample

    @staticmethod
    def _observe_return_report(context):
        return dict(session_id=context['session_id'], plan_id=context['scene'].plan_id,
                    plan_digest=context['scene'].digest, target_track_id=context['scene'].track_id,
                    epoch_ns=context['epoch'], model_sha256=context['fk'].model_sha256,
                    origin_accepted_stamp_ns=context['origin_sample']['accepted_stamp_ns'],
                    origin_joint_positions_rad=list(context['origin_joints']),
                    origin_tool_pose=deepcopy(context['origin_tool_pose']),
                    origin_goal_sha256=context['goal_digest'],
                    real_grasp_success=False, no_implicit_rollback_or_torque_disable=True)

    def handle_capture_observe_return_origin(self, req):
        evidence = dict(physical_motion_submitted=False, real_grasp_success=False)
        with self._planner_operation_lock():
            try:
                plan = deepcopy(req.plan)
                if getattr(plan, 'model_choice', '') != 'unknown_tabletop':
                    raise ObservationPathError('OBSERVE_RETURN_UNKNOWN_OBJECT_REQUIRED')
                requested_scene = FrozenObservationScene.from_plan(plan)
                selection = self._observe_return_selection()
                with self._observation_context_lock:
                    epoch = self._control_reference_epoch_ns
                if type(epoch) is not int or epoch <= 0:
                    raise ObservationPathError('OBSERVE_RETURN_SOURCE_EPOCH_INVALID')
                activation_revision = self._await_observe_return_active(requested_scene, selection, epoch)
                with self._observation_context_lock:
                    self._require_live_observation_task()
                    if self._observation_contexts.revocation != activation_revision:
                        raise ObservationPathError('OBSERVE_RETURN_TASK_ACTIVATION_CHANGED')
                    if self._control_reference_epoch_ns != epoch:
                        raise ObservationPathError('OBSERVE_RETURN_CONTROL_EPOCH_CHANGED')
                    if self._observe_return_selection() != selection:
                        raise ObservationPathError('OBSERVE_RETURN_SELECTION_CHANGED')
                    # Bind only a committed task scene; the request is not itself
                    # allowed to install or replace observation geometry.
                    target = PoseStamped(header=deepcopy(plan.header), pose=deepcopy(plan.poses[0]))
                    config = deepcopy(rospy.get_param('/grasp', {}))
                    validity = _bounded_number(config.get('plan_validity_sec',
                        rospy.get_param('/grasp_6d/plan_validity_sec', 2.)), 600., 'PLAN_VALIDITY')
                    scene, revocation = self._observation_contexts.capture(target, rospy.get_time(), validity)
                    if scene.digest != requested_scene.digest:
                        raise ObservationPathError('OBSERVE_RETURN_COMMITTED_PLAN_MISMATCH')
                identity = _digest([scene.digest, selection, epoch, revocation])
                previous = getattr(self, '_observe_return_origin', None)
                if (previous is not None and previous['revocation'] == revocation
                        and previous['identity'] != identity):
                    raise ObservationPathError('OBSERVE_RETURN_ORIGIN_ALREADY_BOUND')
                if previous is not None and previous['identity'] == identity:
                    self._await_observe_return_stage(previous)
                    if previous.get('submitted'):
                        raise ObservationPathError('OBSERVE_RETURN_ALREADY_SUBMITTED')
                    evidence.update(self._observe_return_report(previous), code='OBSERVE_RETURN_ORIGIN_REUSED')
                    return CompensatePregraspResponse(True, json.dumps(evidence))
                session_seconds = _bounded_number(config.get(
                    'unknown_observe_return_session_timeout_sec', 600.), 600., 'SESSION_TIMEOUT')
                model = rospy.get_param('/robot_description', '')
                context = dict(scene=scene, revocation=revocation, plan=plan,
                    selection=selection, epoch=epoch, config=config, model=model,
                    fk=SerialUrdfFk(model, ARM_NAMES), identity=identity,
                    deadline=time.monotonic()+session_seconds, submitted=False)
                context['tolerances'] = (
                    measured_position_tolerance(config, 'unknown_tabletop'),
                    math.radians(_bounded_number(config.get(
                        'measured_endpoint_orientation_tolerance_deg', 5.), 5., 'ORIENTATION_TOLERANCE')),
                    _bounded_number(config.get('unknown_observe_return_joint_tolerance_rad', .035),
                                    .035, 'JOINT_TOLERANCE'))
                self._await_observe_return_stage(context)
                sample = self._observe_return_sample(context)
                reference = self._observe_return_reference()
                if self._sdk_arm_words(reference) != self._sdk_arm_words(sample['sdk_positions_rad']):
                    raise ObservationPathError('OBSERVE_RETURN_CAPTURE_REFERENCE_CHANGED')
                joints = tuple(float(q) for q in sample['accepted_positions_rad'])
                matrix = context['fk'](joints)
                goal = dict(joint_names=list(ARM_NAMES), joint_positions_rad=list(joints),
                            model_sha256=context['fk'].model_sha256)
                context.update(origin_sample=deepcopy(sample), origin_joints=joints,
                    origin_tool_matrix=matrix.copy(), origin_tool_pose=dict(frame_id='base_link',
                        position_m=matrix[:3, 3].tolist(),
                        quaternion_xyzw=quaternion_from_matrix(matrix).tolist()),
                    origin_sdk_positions_rad=tuple(sample['sdk_positions_rad']),
                    goal_digest=_digest(goal),
                    session_id=_digest([identity, sample['accepted_stamp_ns'], goal]))
                self._validate_observe_return_context(context)
                self._observe_return_origin = context
                evidence.update(self._observe_return_report(context), code='OBSERVE_RETURN_ORIGIN_CAPTURED')
                return CompensatePregraspResponse(True, json.dumps(evidence))
            except Exception as exc:
                evidence.update(code=str(exc), success=False)
                return CompensatePregraspResponse(False, json.dumps(evidence))

    def _observe_return_scenes(self, context):
        scenes = tuple(context.get('admitted_view_scenes', (context['scene'],)))
        if (not 1 <= len(scenes) <= 3 or scenes[0] is not context['scene']
                or scenes[-1] is not context.get('active_view_scene', context['scene'])
                or len({scene.digest for scene in scenes}) != len(scenes)
                or any(scene.track_id != context['scene'].track_id for scene in scenes)
                or any(b.source_ns <= a.source_ns for a, b in zip(scenes, scenes[1:]))):
            raise ObservationPathError('OBSERVE_RETURN_ADMITTED_SCENES_CHANGED')
        return scenes

    @staticmethod
    def _observe_return_tightest_path(paths):
        """Retain every frozen-scene proof; never widen any proved error bound."""
        audits = [item['path_audit'] for item in paths]
        contracts = [audit['execution_tracking_contract'] for audit in audits]
        if all(contract == contracts[0] for contract in contracts):
            return audits[0]
        varying = ('path_position_tolerance_rad', 'joint_error_bounds_rad')
        fixed = [{key: value for key, value in contract.items() if key not in varying}
                 for contract in contracts]
        if any(item != fixed[0] for item in fixed):
            raise ObservationPathError('OBSERVE_RETURN_SCENE_CONTRACTS_INCOMPATIBLE')
        bounds = [np.asarray([contract.get(key, []) for key in varying], dtype=float)
                  for contract in contracts]
        if any(bound.shape != (2, 6) or not np.all(np.isfinite(bound))
               or np.any(bound <= 0.) for bound in bounds):
            raise ObservationPathError('OBSERVE_RETURN_SCENE_CONTRACT_BOUNDS_INVALID')
        for index, bound in enumerate(bounds):
            if all(np.all(bound <= other) for other in bounds):
                return audits[index]
        raise ObservationPathError('OBSERVE_RETURN_SCENE_CONTRACTS_NOT_NESTED')

    def _observe_return_path(self, context, baseline, trajectory, planner):
        self._validate_observe_return_context(context, returning=True)
        names = tuple(trajectory.joint_trajectory.joint_names)
        points = list(trajectory.joint_trajectory.points)
        goal = dict(zip(ARM_NAMES, context['origin_joints']))
        if (len(names) != 6 or set(names) != set(ARM_NAMES) or len(points) < 2
                or len(points[-1].positions) != 6
                or any(not math.isfinite(float(q)) or abs(q-goal[n]) > 1e-12
                       for n, q in zip(names, points[-1].positions))):
            raise ObservationPathError('OBSERVE_RETURN_PATH_GOAL_CHANGED')
        reference = self._observe_return_reference()
        if self._sdk_arm_words(reference) != self._sdk_arm_words(baseline):
            raise ObservationPathError('OBSERVE_RETURN_START_REFERENCE_CHANGED')
        scenes = self._observe_return_scenes(context)
        paths = [dict(plan_id=scene.plan_id, scene_digest=scene.digest,
            path_audit=self._validate_frozen_observation_path(
                (scene, context['revocation']), trajectory, planner,
                context_validator=lambda: self._validate_observe_return_context(context, returning=True)))
            for scene in scenes]
        audit = deepcopy(self._observe_return_tightest_path(paths))
        if len(paths) > 1:
            audit['observe_return_scene_paths'] = paths
        duration = float(points[-1].time_from_start.to_sec())
        grace = float(audit['controller_constraints_snapshot']['goal_time'])
        if (not math.isfinite(duration) or duration <= 0. or not math.isfinite(grace) or grace < 0.
                or time.monotonic()+duration+grace+2. >= context['return_deadline']):
            raise ObservationPathError('OBSERVE_RETURN_PATH_TIME_BUDGET')
        audit.update(observe_return_session_id=context['session_id'],
                     observe_return_origin_goal_sha256=context['goal_digest'])
        return audit

    def handle_return_observe_return_origin(self, req):
        evidence = dict(real_grasp_success=False, command_submission_attempted=False,
                        no_implicit_rollback_or_torque_disable=True)
        saved, planner = {}, None
        with self._planner_operation_lock():
            try:
                context = getattr(self, '_observe_return_origin', None)
                if context is None:
                    raise ObservationPathError('OBSERVE_RETURN_ORIGIN_MISSING')
                evidence.update(self._observe_return_report(context))
                if (getattr(req.plan, 'model_choice', '') != 'unknown_tabletop'
                        or committed_plan_digest(req.plan) != context['scene'].digest):
                    raise ObservationPathError('OBSERVE_RETURN_REQUEST_PLAN_CHANGED')
                self._await_observe_return_stage(context, returning=True)
                episode_key = '/supervisor/observe_return_episodes/' + _digest(
                    [context['scene'].digest, context['selection'], context['epoch']])
                if context['submitted'] or rospy.get_param(episode_key, None) is not None:
                    raise ObservationPathError('OBSERVE_RETURN_ALREADY_SUBMITTED')
                timeout = _bounded_number(context['config'].get(
                    'unknown_observe_return_timeout_sec', 180.), 300., 'RETURN_TIMEOUT')
                context['return_deadline'] = min(context['deadline'], time.monotonic()+timeout)
                if 'view_session' in context:
                    remaining = context['view_session']['deadline_ns'] * 1e-9 - float(rospy.get_time())
                    context['return_deadline'] = min(context['return_deadline'], time.monotonic()+remaining)
                before = self._observe_return_sample(context, returning=True)
                baseline = tuple(before['sdk_positions_rad'])
                planner = self._ensure_planner()
                if planner is None:
                    raise ObservationPathError(self._moveit_not_ready_message())
                attributes = ('observation_path_guard_required', 'observation_path_validator',
                    'observation_tracking_contract_required', 'observation_execution_authorized',
                    'observation_submission_revalidate', 'strict_execution_max_joint_velocity_rad_s',
                    'strict_execution_joint_velocity_limits_rad_s', 'observation_branch_hints_enabled',
                    'controller_reference_guard_required', 'controller_reference_provider',
                    'controller_reference_completion_callback', 'observation_smooth_timing_enabled',
                    'strict_execution_velocity_scaling', 'strict_execution_acceleration_scaling',
                    '_observation_timing_baseline', '_observation_timing_key')
                saved = {key:(hasattr(planner, key), getattr(planner, key, None)) for key in attributes}
                planner.observation_path_guard_required = True
                planner.observation_tracking_contract_required = True
                planner.observation_branch_hints_enabled = False
                planner.strict_execution_max_joint_velocity_rad_s = min(.08,
                    float(planner.strict_execution_max_joint_velocity_rad_s))
                planner.strict_execution_joint_velocity_limits_rad_s = {}
                timing_key = self._configure_observation_timing(planner)
                self._configure_controller_reference_guard(planner)
                def validate():
                    self._validate_observe_return_context(context, returning=True)
                    if not self._observation_timing_current(planner, timing_key):
                        raise ObservationPathError('OBSERVE_RETURN_TIMING_CHANGED')
                def authorized():
                    try:
                        validate()
                        return True
                    except Exception:
                        return False
                planner.observation_execution_authorized = authorized
                planner.observation_path_validator = lambda tr: self._observe_return_path(context, baseline, tr, planner)
                def revalidate(trajectory, audit):
                    validate()
                    if (audit.get('observe_return_session_id') != context['session_id']
                            or audit.get('observe_return_origin_goal_sha256') != context['goal_digest']):
                        raise ObservationPathError('OBSERVE_RETURN_PATH_BINDING_CHANGED')
                    scenes = self._observe_return_scenes(context)
                    if len(scenes) == 1:
                        return self._revalidate_observation_contract_submission(
                            (scenes[0], context['revocation']), trajectory, audit,
                            context_validator=validate)
                    paths = audit.get('observe_return_scene_paths')
                    if (not isinstance(paths, list) or len(paths) != len(scenes)
                            or any(item.get('plan_id') != scene.plan_id or item.get('scene_digest') != scene.digest
                                   for scene, item in zip(scenes, paths))):
                        raise ObservationPathError('OBSERVE_RETURN_SCENE_PATH_BINDING_CHANGED')
                    selected = self._observe_return_tightest_path(paths)
                    if audit['execution_tracking_contract'] != selected['execution_tracking_contract']:
                        raise ObservationPathError('OBSERVE_RETURN_SCENE_CONTRACT_CHANGED')
                    for scene, item in zip(scenes, paths):
                        self._revalidate_observation_contract_submission(
                            (scene, context['revocation']), trajectory, item['path_audit'],
                            context_validator=validate)
                planner.observation_submission_revalidate = revalidate
                if req.execute:
                    for action in (self._ensure_trajectory_controllers_started,
                                   self._synchronize_trajectory_controller_to_feedback):
                        ok, message = action()
                        if not ok:
                            raise ObservationPathError(message)
                    from .contracted_trajectory import ContractedTrajectoryExecutor
                    if getattr(planner, 'observation_trajectory_executor', None) is None:
                        planner.observation_trajectory_executor = ContractedTrajectoryExecutor.connect()
                def before_execute():
                    validate()
                    if context['submitted'] or rospy.get_param(episode_key, None) is not None:
                        raise ObservationPathError('OBSERVE_RETURN_ALREADY_SUBMITTED')
                    fresh = self._observe_return_sample(context, returning=True)
                    if (self._sdk_arm_words(fresh['sdk_positions_rad']) != self._sdk_arm_words(baseline)
                            or max(abs(np.asarray(fresh['accepted_positions_rad'])-
                                       before['accepted_positions_rad'])) > SDK_QUANTUM_RAD+1e-12):
                        raise ObservationPathError('OBSERVE_RETURN_START_CHANGED')
                    rospy.set_param(episode_key, dict(session_id=context['session_id'],
                        state='consumed_before_submission', origin_goal_sha256=context['goal_digest']))
                    context['submitted'] = True
                    evidence['command_submission_attempted'] = True
                ok, message = planner.plan_and_execute_joint_probe(
                    context['origin_joints'], execute=bool(req.execute),
                    start_joint_positions=baseline,
                    before_execute=before_execute if req.execute else None)
                if not ok:
                    raise ObservationPathError('OBSERVE_RETURN_PATH_REJECTED: ' + message)
                if not req.execute:
                    evidence.update(code='OBSERVE_RETURN_PLANNED_ONLY', physical_path_authorized=False)
                    return CompensatePregraspResponse(True, json.dumps(evidence))
                if not context['submitted']:
                    raise ObservationPathError('OBSERVE_RETURN_SUBMISSION_NOT_RECORDED')
                completed_ns = int(rospy.get_time()*1e9)
                evidence.update(controller_action_completed=True, completion_stamp_ns=completed_ns,
                                post_command_feedback_valid=False)
                after = self._observe_return_sample(context, returning=True, after_ns=completed_ns)
                actual = np.asarray(after['accepted_positions_rad'])
                achieved = context['fk'](actual)
                target = context['origin_tool_matrix']
                position = float(np.linalg.norm(achieved[:3, 3]-target[:3, 3]))
                angle = math.acos(float(np.clip((np.trace(target[:3, :3].T @ achieved[:3, :3])-1.)/2., -1., 1.)))
                joint_error = float(np.max(np.abs(actual-context['origin_joints'])))
                position_limit, angle_limit, joint_limit = context['tolerances']
                if self._sdk_arm_words(after['sdk_positions_rad']) != self._sdk_arm_words(context['origin_joints']):
                    raise ObservationPathError('OBSERVE_RETURN_COMPLETED_TARGET_CHANGED')
                validate()
                evidence.update(post_command_feedback_valid=True,
                    accepted_stamp_ns=after['accepted_stamp_ns'],
                    stationary_window_start_ns=after['stationary_window_start_ns'],
                    stationary_window_end_ns=after['stationary_window_end_ns'],
                    measured_joint_positions_rad=actual.tolist(),
                    position_error_m=position, orientation_error_rad=angle,
                    maximum_joint_error_rad=joint_error, position_tolerance_m=position_limit,
                    orientation_tolerance_rad=angle_limit, joint_tolerance_rad=joint_limit)
                reached = position <= position_limit and angle <= angle_limit and joint_error <= joint_limit
                evidence['code'] = ('OBSERVE_RETURN_MEASURED_ARRIVAL' if reached
                                    else 'OBSERVE_RETURN_MEASURED_ENDPOINT_OUTSIDE_TOLERANCE')
                return CompensatePregraspResponse(reached, json.dumps(evidence))
            except Exception as exc:
                evidence.update(code=str(exc), success=False)
                return CompensatePregraspResponse(False, json.dumps(evidence))
            finally:
                if planner is not None:
                    for key, (existed, value) in saved.items():
                        if existed:
                            setattr(planner, key, value)
                        elif hasattr(planner, key):
                            delattr(planner, key)
