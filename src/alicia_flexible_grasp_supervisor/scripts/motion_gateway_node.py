#!/usr/bin/env python3
import math
import json
import hashlib
import threading
import time
from collections import deque
from copy import deepcopy
from types import SimpleNamespace

import numpy as np

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import (
    JointTrajectoryControllerState, FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from std_msgs.msg import Bool, Header, String
from sensor_msgs.msg import JointState
from alicia_flexible_grasp_supervisor.msg import Grasp6DPlan, GraspState
from alicia_flexible_grasp.robot.observation_path_guard import (
    CommittedObservationContexts, ObservationPathError, SerialUrdfFk,
    validate_observation_trajectory, wire_digest,
    observation_endpoint_support_bound, segment_coefficients, _polynomial_bounds,
    observation_tracking_error_bounds,
    FOLLOWING_PATH_MAX_CHECKS, FOLLOWING_PATH_MAX_SECONDS,
)
from alicia_flexible_grasp.grasp.gripper_geometry import (
    GripperGeometry, gripper_contract_mismatch_reason,
    ANALYTICAL_GRIPPER_MODEL_NAME,
)
from alicia_flexible_grasp.grasp.grasp_state_machine import GraspStages, STATE_NAMES
from alicia_flexible_grasp.robot.actuation_bootstrap import PENDING_BOOTSTRAP_STATES
from alicia_flexible_grasp.robot.stationary_following import stationary_following_error, ARM_NAMES
from alicia_flexible_grasp.robot.observation_preview import encode_path_evidence
from alicia_flexible_grasp.robot.endpoint_correction import EndpointCorrection
from alicia_flexible_grasp.robot.pregrasp_gateway import PregraspCompensationGateway
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from alicia_flexible_grasp.robot.joint_commander import JointCommander
from alicia_flexible_grasp.robot.gripper_commander import GripperCommander
from alicia_flexible_grasp.robot.moveit_planner import MoveItPlanner
from alicia_flexible_grasp.robot.cartesian_controller import CartesianJogger
from alicia_flexible_grasp_supervisor.srv import (
    CartesianJog,
    CartesianJogResponse,
    CompensatePregrasp,
    CheckPoseSequence,
    CheckPoseSequenceResponse,
    ResolveFreeSpaceOrientations,
    ResolveFreeSpaceOrientationsResponse,
    SetFloat,
    SetFloatResponse,
    SetJointCommand,
    SetJointCommandResponse,
    SetTargetPose,
    SetTargetPoseResponse,
    TriggerZero,
    TriggerZeroResponse,
)

try:
    from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest, ListControllers
except Exception:
    SwitchController = None
    SwitchControllerRequest = None
    ListControllers = None

class MotionGateway(PregraspCompensationGateway):
    def __init__(self):
        cfg = rospy.get_param('/robot', {})
        self.joint_names = cfg.get('joint_names', ['Joint1','Joint2','Joint3','Joint4','Joint5','Joint6','right_finger'])
        self.trajectory_controller_names = cfg.get('trajectory_controller_names', ['alicia_controller', 'hand_controller'])
        self.joint_cmd = JointCommander(cfg.get('joint_command_topic','/joint_commands'), self.joint_names)
        self.gripper = GripperCommander(self.joint_cmd, len(self.joint_names)-1, cfg.get('gripper_min_m',0.0), cfg.get('gripper_max_m',0.05))
        self.trajectory_command_pub = rospy.Publisher(
            cfg.get(
                'trajectory_controller_command_topic',
                '/alicia_controller/command',
            ),
            JointTrajectory,
            queue_size=1,
        )
        self._trajectory_controller_state = None
        self.trajectory_state_sub = rospy.Subscriber(
            cfg.get(
                'trajectory_controller_state_topic',
                '/alicia_controller/state',
            ),
            JointTrajectoryControllerState,
            self._trajectory_controller_state_cb,
            queue_size=1,
        )
        self.zero_pub = rospy.Publisher(cfg.get('zero_calibrate_topic','/zero_calibrate'), Bool, queue_size=1)
        self.demo_pub = rospy.Publisher(cfg.get('demonstration_topic','/demonstration'), Bool, queue_size=1)
        self.manipulator_group = rospy.get_param('~manipulator_group','alicia')
        self.gripper_group = rospy.get_param('~gripper_group','hand')
        self.velocity = rospy.get_param('~velocity',0.3)
        self.planner = None
        self._planner_lock = threading.RLock()
        self._endpoint_feedback_lock = threading.RLock()
        self._endpoint_feedback_history = {'sdk': deque(maxlen=128), 'accepted': deque(maxlen=128)}
        self._endpoint_correction_consumed_epoch = None
        self._observation_context_lock = threading.RLock()
        self._observation_context_condition = threading.Condition(self._observation_context_lock)
        self._observation_contexts = CommittedObservationContexts()
        self._observation_joint_feedback = None
        self._sdk_command_feedback = None
        self._driver_control_reference = None
        self._command_reference_lock = threading.RLock()
        self._positive_enable_reference_epoch_sec = None
        self._control_reference_epoch_ns = None
        self._completed_command_reference = None
        self._sdk_reference_sub = rospy.Subscriber(
            '/alicia_d/sdk_command', JointState, self._sdk_command_feedback_cb,
            queue_size=1)
        self._driver_reference_sub = rospy.Subscriber(
            '/alicia_d/control_reference', JointState, self._driver_control_reference_cb,
            queue_size=1)
        self._driver_reference_epoch_sub = rospy.Subscriber(
            '/alicia_d/control_reference_epoch', Header, self._control_reference_epoch_cb,
            queue_size=1)
        self._observation_task_state_stamp_ns = 0
        self._observation_task_requires_inactive = False
        self._observation_plan_sub = rospy.Subscriber(
            '/grasp_6d/plan_enriched', Grasp6DPlan,
            self._committed_observation_plan_cb, queue_size=10)
        self._observation_feedback_sub = rospy.Subscriber(
            '/alicia_d/accepted_joint_states', JointState, self._observation_joint_feedback_cb,
            queue_size=1)
        self._observation_task_sub = rospy.Subscriber(
            '/grasp/state', GraspState, self._observation_task_state_cb, queue_size=10)
        self._last_planner_error = 'MoveIt not initialized'
        self._planner_retry_period_sec = float(rospy.get_param('~planner_retry_period_sec', 2.0))
        self._last_planner_attempt = 0.0
        self._gripper_arm_hold_positions = None
        self._last_gripper_command_time = 0.0
        self._actuation_status = ''
        self._actuation_received_sec = 0.0
        self._actuation_sub = rospy.Subscriber('/alicia_d/actuation_status', String, self._actuation_cb, queue_size=1)
        self._gui_direct_mode = rospy.get_param('/gui/joint_direct_mode', False) is True
        self._gui_mode_sub = rospy.Subscriber('/gui/joint_direct_mode', Bool, self._gui_mode_cb, queue_size=1)
        self.jogger = CartesianJogger(self.planner)
        rospy.Service('/supervisor/move_to_joints', SetJointCommand, self.handle_joints)
        rospy.Service('/supervisor/probe_joint_tracking', SetJointCommand, self.handle_tracking_probe)
        rospy.Service('/supervisor/characterize_joint_response', SetJointCommand,
                      self.handle_joint_response_characterization)
        # Explicit diagnostic only. Nothing in the automatic grasp route calls
        # this endpoint. trigger=false is plan-only; execution additionally
        # requires an opt-in and the existing idle-task/free-space probe gates.
        rospy.Service('/supervisor/refine_endpoint_feedback', TriggerZero,
                      self.handle_endpoint_feedback_correction)
        rospy.Service('/supervisor/compensate_pregrasp', CompensatePregrasp,
                      self.handle_compensate_pregrasp)
        rospy.Service('/supervisor/set_gripper', SetFloat, self.handle_gripper)
        rospy.Service('/supervisor/move_to_pose', SetTargetPose, self.handle_pose)
        rospy.Service('/supervisor/move_to_pose_linear', SetTargetPose, self.handle_pose_linear)
        rospy.Service('/supervisor/check_pose_strict', SetTargetPose, self.handle_pose_strict)
        rospy.Service('/supervisor/check_observation_pose_strict', SetTargetPose,
                      self.handle_observation_pose_strict)
        rospy.Service('/supervisor/execute_observation_pose_strict', SetTargetPose,
                      self.handle_observation_pose_strict_execute)
        rospy.Service('/supervisor/plan_and_execute_observation_pose_strict', SetTargetPose,
                      self.handle_observation_pose_strict_plan_execute)
        rospy.Service(
            '/supervisor/check_pose_sequence_strict',
            CheckPoseSequence,
            self.handle_pose_sequence_strict,
        )
        rospy.Service(
            '/supervisor/resolve_free_space_orientations',
            ResolveFreeSpaceOrientations,
            self.handle_resolve_free_space_orientations,
        )
        rospy.Service('/supervisor/execute_pose_strict', SetTargetPose, self.handle_pose_strict_execute)
        rospy.Service(
            '/supervisor/plan_and_execute_pose_strict',
            SetTargetPose,
            self.handle_pose_strict_plan_execute,
        )
        rospy.Service('/supervisor/cartesian_jog', CartesianJog, self.handle_jog)
        rospy.Service('/supervisor/confirm_actuation_for_observation', SetTargetPose,
                      lambda req: self._with_observation_planner(req, self.handle_observation_actuation))
        rospy.Service('/supervisor/trigger_zero', TriggerZero, self.handle_zero)
        rospy.loginfo('MotionGateway ready: commands -> %s', cfg.get('joint_command_topic','/joint_commands'))

    def _gui_mode_cb(self, msg):
        was_manual = getattr(self, '_gui_direct_mode', False)
        self._gui_direct_mode = bool(msg.data)
        if self._gui_direct_mode != was_manual:
            self._completed_command_reference = None
        if self._gui_direct_mode and hasattr(self, '_observation_context_lock'):
            with self._observation_context_lock:
                self._observation_contexts.clear()
                self._observation_contexts.set_active(False)
        if self._gui_direct_mode and not was_manual:
            # Cancel only MoveIt's active trajectory; controllers and joint
            # torque remain enabled. The driver independently installs hold
            # and rejects all non-slider SDK targets while the mode is active.
            planner = getattr(self, 'planner', None)
            if planner is not None and getattr(planner, 'ready', False):
                executor = getattr(planner, 'observation_trajectory_executor', None)
                if executor is not None:
                    executor.cancel()
                planner.manipulator.stop()

    def _with_observation_planner(self, req, handler):
        """Keep observation speed and cached paths separate from contact paths."""
        if req.execute and self._manual_control_active():
            return SetTargetPoseResponse(False, 'MANUAL_CONTROL_ACTIVE')
        with self._planner_operation_lock():
            original = self.planner
            planner = getattr(self, '_observation_planner', None)
            if planner is None or not planner.ready:
                planner = MoveItPlanner(self.manipulator_group, self.gripper_group, self.velocity)
                if not planner.ready:
                    return SetTargetPoseResponse(False, planner.error or 'observation MoveIt unavailable')
                limit = rospy.get_param('/robot/observation_max_joint_velocity_rad_s', 0.08)
                if (isinstance(limit, bool) or not isinstance(limit, (int, float))
                        or not math.isfinite(limit) or not 0.0 < limit <=
                        planner.strict_execution_max_joint_velocity_rad_s):
                    return SetTargetPoseResponse(False, 'invalid observation joint velocity limit')
                planner.strict_execution_max_joint_velocity_rad_s = float(limit)
                planner.strict_execution_joint_velocity_limits_rad_s = {}
                planner.observation_branch_hints_enabled = True
                self._observation_planner = planner
            planner.observation_path_guard_required = True
            self._configure_controller_reference_guard(planner)
            planner.observation_tracking_contract_required = (
                rospy.get_param('/robot/observation_tracking_contract_enabled', False) is True)
            previous_validator = getattr(planner, 'observation_path_validator', None)
            if req.execute:
                try:
                    if planner.observation_tracking_contract_required:
                        from alicia_flexible_grasp.robot.contracted_trajectory import ContractedTrajectoryExecutor
                        if getattr(planner, 'observation_trajectory_executor', None) is None:
                            planner.observation_trajectory_executor = ContractedTrajectoryExecutor.connect()
                    # Topic delivery and the following RPC use different
                    # connections. Wait only at admission, never at submission.
                    self._wait_for_active_observation_task()
                    with self._observation_context_lock:
                        self._require_live_observation_task()
                        grasp_config = rospy.get_param('/grasp', {})
                        validity = grasp_config.get('plan_validity_sec',
                            rospy.get_param('/grasp_6d/plan_validity_sec', 2.))
                        context = self._observation_contexts.capture(
                            req.target, float(rospy.get_time()), float(validity))
                except Exception as exc:
                    return SetTargetPoseResponse(False, 'OBSERVATION_PATH_CONTEXT_INVALID: ' + str(exc))
                planner.observation_path_validator = lambda trajectory: (
                    self._validate_frozen_observation_path(context, trajectory, planner))
                planner.observation_execution_authorized = self._observation_execution_authorized
                planner.observation_submission_revalidate = lambda trajectory, audit: (
                    self._revalidate_observation_contract_submission(context, trajectory, audit))
            try:
                self.planner = planner
                result = handler(req)
                if not req.execute and result.success:
                    try:
                        result.message += ' observation_path_evidence=' + (
                            self._observation_preview_path_evidence(planner, req.target))
                    except Exception as exc:
                        planner._last_pose_plan = None
                        result = SetTargetPoseResponse(False,
                            'OBSERVATION_PREVIEW_PATH_UNAVAILABLE: ' + str(exc))
                result.message += ' speed_profile=observation joint_limit_rad_s=%.3f' % (
                    planner.strict_execution_max_joint_velocity_rad_s)
                return result
            finally:
                planner.observation_path_validator = previous_validator
                self.planner = original

    def _observation_preview_path_evidence(self, planner, target):
        """Export a timed candidate with the same prospective handoff baseline.

        Called under the planner lock. Selecting the reference only reads
        telemetry; this must never call the synchronization publisher, start
        controllers, change enable, or execute the candidate. A handoff may
        still be pending; the preview is conditional, not admission evidence.
        """
        cached = planner._last_pose_plan
        if not isinstance(cached, dict) or cached.get('kind') != 'strict pose':
            raise ObservationPathError('strict candidate trajectory unavailable')
        branch = cached.get('observation_branch', {})
        epoch = getattr(self, '_control_reference_epoch_ns', None)
        if not isinstance(epoch, int) or epoch <= 0:
            raise ObservationPathError('driver reference epoch unavailable')
        names = tuple(cached['plan'].joint_trajectory.joint_names)
        if len(names) != 6 or set(names) != set(self.joint_names[:-1]):
            raise ObservationPathError('candidate joint names mismatch')
        positions, mode = self._select_controller_sync_reference()
        by_name = dict(zip(self.joint_names[:-1], positions))
        desired = SimpleNamespace(positions=[by_name[name] for name in names],
                                  velocities=[0.] * 6, accelerations=[0.] * 6)
        opening = self._observation_measured_opening()
        signature = planner._observation_branch_signature()[0]
        original_provider = planner.controller_reference_provider
        try:
            # Pure retiming/velocity math on a COPY. It is never installed as
            # an executable cache or mistaken for a completed controller hold.
            planner.controller_reference_provider = lambda order: deepcopy(desired) if tuple(order) == names else None
            plan, reason = planner._retime_strict_execution_plan(deepcopy(cached['plan']))
            if plan is None:
                raise ObservationPathError(reason)
            plan, _, reason = planner._prepare_controller_reference_timing(plan)
            if plan is None or reason:
                raise ObservationPathError(reason)
        finally:
            planner.controller_reference_provider = original_provider
        after, after_mode = self._select_controller_sync_reference()
        if (after != positions or after_mode != mode
                or epoch != getattr(self, '_control_reference_epoch_ns', None)
                or self._observation_measured_opening() != opening
                or planner._observation_branch_signature()[0] != signature
                or self._manual_control_active()):
            raise ObservationPathError('candidate reference/model changed during preview')
        return encode_path_evidence(target, plan, desired, opening, mode, epoch,
                                    branch['robot_description_sha256'])

    def _committed_observation_plan_cb(self, msg):
        with self._observation_context_lock:
            try:
                if float(msg.header.stamp.to_sec()) > float(rospy.get_time()):
                    raise ObservationPathError('committed geometry source is future')
                self._observation_contexts.ingest(msg)
            except Exception as exc:
                self._observation_contexts.clear()
                rospy.logwarn('Rejected committed observation geometry: %s', exc)

    def _observation_task_state_cb(self, msg):
        with self._observation_context_lock:
            stamp_ns = int(msg.header.stamp.to_nsec())
            if stamp_ns <= 0 or float(msg.header.stamp.to_sec()) > float(rospy.get_time()):
                self._revoke_observation_task()
                return
            if stamp_ns <= self._observation_task_state_stamp_ns:
                return
            self._observation_task_state_stamp_ns = stamp_ns
            self._pregrasp_task_stage = int(msg.stage)
            terminal_stages = (GraspStages.SUCCESS, GraspStages.FAILED,
                               GraspStages.EMERGENCY_STOP)
            terminal = (msg.stage in terminal_stages or
                        msg.state in tuple(STATE_NAMES[stage] for stage in terminal_stages))
            if terminal:
                self._revoke_observation_task()
            if not bool(msg.active):
                # A real inactive message is the only way to release the
                # terminal/disconnect latch; reconnecting alone is not enough.
                self._observation_contexts.set_active(False)
                self._observation_task_requires_inactive = False
            elif getattr(self, '_gui_direct_mode', False):
                self._revoke_observation_task()
            elif not terminal and not getattr(self, '_observation_task_requires_inactive', False):
                self._observation_contexts.set_active(True)
            self._notify_observation_task_state()

    def _notify_observation_task_state(self):
        condition = getattr(self, '_observation_context_condition', None)
        if condition is not None:
            condition.notify_all()  # caller holds the context RLock

    def _revoke_observation_task(self):
        self._observation_contexts.clear()
        self._observation_contexts.set_active(False)
        self._observation_task_requires_inactive = True
        self._notify_observation_task_state()

    def _observation_task_connection_available(self):
        subscriber = getattr(self, '_observation_task_sub', None)
        if subscriber is not None and subscriber.get_num_connections() > 0:
            return True
        self._revoke_observation_task()
        # Reject an old latched active message if the transport reconnects.
        now_ns = int(rospy.Time.from_sec(float(rospy.get_time())).to_nsec())
        self._observation_task_state_stamp_ns = max(
            self._observation_task_state_stamp_ns, now_ns)
        return False

    def _require_live_observation_task(self):
        if (not self._observation_task_connection_available()
                or not self._observation_contexts.active
                or self._observation_contexts.blocked
                or getattr(self, '_observation_task_requires_inactive', False)):
            raise ObservationPathError('active task publisher is unavailable')

    def _wait_for_active_observation_task(self, max_wait_sec=.5):
        duration = float(max_wait_sec)
        if not math.isfinite(duration) or not 0. <= duration <= .5:
            raise ObservationPathError('invalid active task admission wait')
        deadline = time.monotonic() + duration
        with self._observation_context_condition:
            while not self._observation_contexts.active:
                if (not self._observation_task_connection_available()
                        or getattr(self, '_observation_task_requires_inactive', False)):
                    raise ObservationPathError('active task publisher is unavailable')
                remaining = deadline - time.monotonic()
                if remaining <= 0.:
                    raise ObservationPathError('active task admission notification timed out')
                # Condition.wait releases the RLock for the real callback.
                self._observation_context_condition.wait(remaining)
            self._require_live_observation_task()

    def _observation_joint_feedback_cb(self, msg):
        self._observation_joint_feedback = msg
        self._record_endpoint_feedback('accepted', msg)

    def _sdk_command_feedback_cb(self, msg):
        with self._command_reference_lock:
            old = self._sdk_command_feedback
            if old is not None and msg.header.stamp.to_nsec() < old.header.stamp.to_nsec():
                return
            self._sdk_command_feedback = msg
        self._record_endpoint_feedback('sdk', msg)

    def _record_endpoint_feedback(self, kind, msg):
        lock = getattr(self, '_endpoint_feedback_lock', None)
        if lock is not None:
            with lock:
                self._endpoint_feedback_history[kind].append(deepcopy(msg))

    def _driver_control_reference_cb(self, msg):
        with self._command_reference_lock:
            old = self._driver_control_reference
            if old is not None and msg.header.stamp.to_nsec() < old.header.stamp.to_nsec():
                return
            self._driver_control_reference = msg

    def _control_reference_epoch_cb(self, msg):
        stamp_ns = msg.stamp.to_nsec()
        if msg.frame_id != 'sdk_reference_epoch' or stamp_ns <= 0:
            return
        with self._command_reference_lock:
            old = getattr(self, '_control_reference_epoch_ns', None)
            if old is not None and stamp_ns <= old:
                return
            self._control_reference_epoch_ns = stamp_ns
            self._positive_enable_reference_epoch_sec = float(msg.stamp.to_sec())
            self._sdk_command_feedback = None
            self._driver_control_reference = None
            self._completed_command_reference = None

    @staticmethod
    def _sdk_arm_words(positions):
        values = [float(value) for value in positions]
        if not all(math.isfinite(value) and -math.pi <= value <= math.pi for value in values):
            raise ObservationPathError('arm reference exceeds SDK position domain')
        # Match sdk_joint_position_encode, including positive lround ties.
        return tuple(min(4095, max(0, int(math.floor(
            ((value * 180.0 / math.pi + 180.0) / 360.0 * 4096.0) + .5))))
                     for value in values)

    def _fresh_named_reference(self, message, frame, names):
        if message is None or message.header.frame_id != frame:
            raise ObservationPathError('missing or incorrect '+frame+' reference')
        now = float(rospy.get_time())
        age = now - float(message.header.stamp.to_sec())
        maximum = float(rospy.get_param(
            '/robot/strict_execution_controller_sync_max_feedback_age_sec', .5))
        if (message.header.stamp.to_nsec() <= 0 or not math.isfinite(age)
                or not math.isfinite(maximum) or not 0. < maximum <= .5
                or not 0. <= age <= maximum):
            raise ObservationPathError(frame+' reference is stale or future')
        msg_names, positions = list(message.name), list(message.position)
        if (len(msg_names) != len(positions) or len(msg_names) != len(set(msg_names))
                or set(msg_names) != set(names)
                or not all(math.isfinite(float(value)) for value in positions)):
            raise ObservationPathError(frame+' joint names or positions are invalid')
        values = dict(zip(msg_names, positions))
        return [float(values[name]) for name in names]

    def _fresh_accepted_reference(self):
        positions = self._fresh_named_reference(
            deepcopy(getattr(self, '_observation_joint_feedback', None)),
            'sdk_measured', self.joint_names)
        self._sdk_arm_words(positions[:-1])
        if not 0. <= positions[-1] <= .05:
            raise ObservationPathError('accepted gripper opening outside physical range')
        return positions

    def _command_reference_snapshot(self):
        """Read paired successful-write telemetry, never a requested target.

        The two topics have separate TCP connections. A short bounded wait
        permits their same-stamp callbacks to arrive, but never selects an
        older matching pair in place of a newer, unmatched command.
        """
        deadline = time.monotonic() + .10
        while True:
            with self._command_reference_lock:
                sdk = deepcopy(self._sdk_command_feedback)
                reference = deepcopy(self._driver_control_reference)
            if sdk is None and reference is None:
                return None
            if (sdk is not None and reference is not None
                    and sdk.header.stamp.to_nsec() == reference.header.stamp.to_nsec()):
                break
            if time.monotonic() >= deadline:
                raise ObservationPathError('successful SDK/reference stamps are not paired')
            time.sleep(.002)
        arm = self._fresh_named_reference(sdk, 'sdk_transmitted', self.joint_names[:-1])
        mode = str(reference.header.frame_id)
        if mode not in ('sdk_tracking', 'sdk_handoff_required'):
            raise ObservationPathError('driver reference is not admitted for automatic control: '+mode)
        full = self._fresh_named_reference(reference, mode, self.joint_names)
        if self._sdk_arm_words(arm) != self._sdk_arm_words(full[:-1]):
            raise ObservationPathError('successful SDK/control reference words disagree')
        if not 0. <= full[-1] <= .05:
            raise ObservationPathError('successful gripper opening outside physical range')
        status = self._fresh_actuation_status()
        if not (status.startswith('CONFIRMED:') or status in PENDING_BOOTSTRAP_STATES):
            raise ObservationPathError('driver reference has no fresh actuation epoch: '+status)
        epoch = getattr(self, '_control_reference_epoch_ns', None)
        if epoch is None:
            raise ObservationPathError('driver command epoch telemetry is unavailable')
        if sdk.header.stamp.to_nsec() < epoch:
            raise ObservationPathError('successful reference predates the positive-enable epoch')
        return SimpleNamespace(positions=full[:-1], opening=full[-1], mode=mode,
                               stamp_sec=float(sdk.header.stamp.to_sec()))

    def _stationary_controller_reference(self, joint_names, require_stationary=True):
        state = deepcopy(getattr(self, '_trajectory_controller_state', None))
        if state is None:
            raise ObservationPathError('controller desired state unavailable')
        age = float(rospy.get_time()) - float(state.header.stamp.to_sec())
        maximum = float(rospy.get_param(
            '/robot/strict_execution_controller_sync_max_feedback_age_sec', .5))
        if (not math.isfinite(age) or not math.isfinite(maximum)
                or not 0. < maximum <= .5 or not 0. <= age <= maximum):
            raise ObservationPathError('controller desired state is stale or future')
        names = list(state.joint_names)
        if len(names) != len(set(names)) or set(names) != set(joint_names):
            raise ObservationPathError('controller desired joint names mismatch')
        desired = SimpleNamespace()
        for field in ('positions', 'velocities', 'accelerations'):
            values = list(getattr(state.desired, field, ()))
            if len(values) != len(names) or not all(math.isfinite(float(v)) for v in values):
                raise ObservationPathError('controller desired '+field+' unavailable')
            by_name = dict(zip(names, values))
            setattr(desired, field, [float(by_name[name]) for name in joint_names])
        if require_stationary and any(value != 0. for value in desired.velocities + desired.accelerations):
            raise ObservationPathError('controller reference is not stationary')
        self._sdk_arm_words(desired.positions)
        # Keep time and q/v/a from one callback snapshot. Reading the live
        # header later can incorrectly lend a newer frame's time to old q.
        desired.controller_stamp_sec = float(state.header.stamp.to_sec())
        return desired

    def _validated_controller_sdk_reference(self, joint_names):
        epoch = getattr(self, '_control_reference_epoch_ns', None)
        if self._manual_control_active():
            raise ObservationPathError('manual control owns the arm')
        self._fresh_accepted_reference()
        reference = self._command_reference_snapshot()
        if reference is None or reference.mode != 'sdk_tracking':
            raise ObservationPathError('automatic SDK reference has not been admitted')
        desired = self._stationary_controller_reference(joint_names)
        wire_by_name = dict(zip(self.joint_names[:-1], reference.positions))
        wire = [wire_by_name[name] for name in joint_names]
        if self._sdk_arm_words(desired.positions) != self._sdk_arm_words(wire):
            raise ObservationPathError('controller desired differs from successful SDK baseline')
        if self._manual_control_active() or epoch != getattr(self, '_control_reference_epoch_ns', None):
            raise ObservationPathError('control ownership or epoch changed while reading reference')
        desired.sdk_stamp_sec = reference.stamp_sec
        return desired

    def _controller_reference_for_execution(self, joint_names):
        completed = deepcopy(getattr(self, '_completed_command_reference', None))
        if not isinstance(completed, dict) or set(completed['names']) != set(joint_names):
            raise ObservationPathError('stationary reference lacks completed gateway command provenance')
        expected = dict(zip(completed['names'], completed['positions']))
        deadline = time.monotonic() + .10
        while True:
            desired = self._validated_controller_sdk_reference(joint_names)
            if completed != getattr(self, '_completed_command_reference', None):
                raise ObservationPathError('completed gateway command provenance changed')
            if any(abs(value - expected[name]) > 1e-12
                   for name, value in zip(joint_names, desired.positions)):
                raise ObservationPathError('controller reference is not the completed gateway command')
            if desired.controller_stamp_sec >= completed['end_sec']:
                break
            # execute(wait=True) can return just before the next controller
            # callback. Only this timestamp-order gap may wait; every other
            # input is revalidated without publishing or changing provenance.
            remaining = deadline - time.monotonic()
            if remaining <= 0.:
                raise ObservationPathError('post-completion controller snapshot timed out')
            time.sleep(min(.002, remaining))
        self._observation_sync_hold_evidence = {
            'names': tuple(joint_names), 'positions': tuple(desired.positions),
            'end_sec': completed['end_sec'], 'kind': completed['kind'],
            'sdk_words': self._sdk_arm_words(desired.positions),
            'completion': completed,
        }
        return desired

    def _record_completed_controller_reference(self, plan):
        trajectory = plan.joint_trajectory
        names = list(trajectory.joint_names)
        if len(names) != len(set(names)) or set(names) != set(self.joint_names[:-1]) or not trajectory.points:
            raise ObservationPathError('completed trajectory joint names are invalid')
        positions = list(trajectory.points[-1].positions)
        if len(positions) != len(names):
            raise ObservationPathError('completed trajectory endpoint is incomplete')
        self._sdk_arm_words(positions)
        self._completed_command_reference = {
            'names': tuple(names), 'positions': tuple(positions),
            'end_sec': float(rospy.get_time()), 'kind': 'completed_gateway_trajectory',
            'trajectory_sha256': wire_digest(plan),
        }

    def _configure_controller_reference_guard(self, planner):
        planner.controller_reference_guard_required = True
        planner.controller_reference_provider = self._controller_reference_for_execution
        planner.controller_reference_completion_callback = self._record_completed_controller_reference

    def _observation_controller_hold(self, joint_names):
        if getattr(self, '_driver_reference_sub', None) is not None:
            return self._controller_reference_for_execution(joint_names)
        state = deepcopy(getattr(self, '_trajectory_controller_state', None))
        if state is None:
            raise ObservationPathError('controller desired state is unavailable')
        cfg = rospy.get_param('/robot', {})
        maximum_age = float(cfg.get('strict_execution_controller_sync_max_feedback_age_sec', .5))
        age = float(rospy.get_time()) - float(state.header.stamp.to_sec())
        if not math.isfinite(age) or not 0. <= age <= maximum_age:
            raise ObservationPathError('controller desired state is stale')
        names = list(state.joint_names)
        if len(names) != len(set(names)) or set(names) != set(joint_names):
            raise ObservationPathError('controller desired joint names mismatch')
        desired = SimpleNamespace()
        for field in ('positions', 'velocities', 'accelerations'):
            values = list(getattr(state.desired, field, ()))
            if len(values) != len(names) or not all(math.isfinite(float(v)) for v in values):
                raise ObservationPathError('controller desired '+field+' unavailable')
            mapping = dict(zip(names, values))
            setattr(desired, field, [float(mapping[name]) for name in joint_names])
        if any(v != 0. for v in desired.velocities + desired.accelerations):
            raise ObservationPathError('controller desired is not a stationary hold')
        evidence = getattr(self, '_observation_sync_hold_evidence', None)
        if (not isinstance(evidence, dict) or set(evidence['names']) != set(joint_names)
                or float(state.header.stamp.to_sec()) < evidence['end_sec']
                or max(abs(a-dict(zip(evidence['names'], evidence['positions']))[name])
                       for a, name in zip(desired.positions, joint_names)) > 1e-12):
            raise ObservationPathError('controller state is not the completed gateway-issued hold')
        return desired

    def _observation_measured_opening(self):
        feedback = deepcopy(getattr(self, '_observation_joint_feedback', None))
        if feedback is None:
            raise ObservationPathError('measured gripper feedback is unavailable')
        if str(feedback.header.frame_id) != 'sdk_measured':
            raise ObservationPathError('gripper feedback is not accepted SDK evidence')
        age = float(rospy.get_time()) - float(feedback.header.stamp.to_sec())
        maximum_age = float(rospy.get_param('/robot/strict_execution_controller_sync_max_feedback_age_sec', .5))
        if not math.isfinite(age) or not 0. <= age <= maximum_age:
            raise ObservationPathError('measured gripper feedback is stale')
        names, positions = list(feedback.name), list(feedback.position)
        if len(names) != len(positions) or len(names) != len(set(names)) or 'right_finger' not in names:
            raise ObservationPathError('measured right_finger feedback is missing')
        opening = float(positions[names.index('right_finger')])
        if not math.isfinite(opening) or not 0. <= opening <= .050:
            raise ObservationPathError('measured gripper gap is outside the physical 50 mm range')
        return opening

    def _validate_frozen_observation_path(self, context, trajectory, planner,
                                        context_validator=None):
        scene, revocation = context
        with self._observation_context_lock:
            self._require_live_observation_task()
            if context_validator is None:
                self._observation_contexts.validate_capture(scene, revocation)
            else:
                context_validator()
        if self._manual_control_active():
            raise ObservationPathError('manual control owns motion')
        names = tuple(trajectory.joint_trajectory.joint_names)
        if len(names) != 6 or set(names) != set(self.joint_names[:-1]):
            raise ObservationPathError('observation requires the six configured arm joints')
        if (planner.manipulator.get_planning_frame() != 'base_link'
                or planner.manipulator.get_end_effector_link() != 'tool0'):
            raise ObservationPathError('observation FK requires base_link/tool0')
        description = rospy.get_param('/robot_description', '')
        if not isinstance(description, str) or not description.strip():
            raise ObservationPathError('runtime robot description is unavailable')
        model = SerialUrdfFk(description, names)
        if not np.all(np.isfinite(model.limits)):
            raise ObservationPathError('runtime observation URDF joint limits are unavailable')
        model_signature = planner._observation_branch_signature()[0]
        cfg = deepcopy(rospy.get_param('/grasp_6d/remote/gripper_geometry', {}))
        if cfg.get('tool_jaw_axis', 'y') != 'y' or cfg.get('tool_finger_length_axis', 'z') != 'z':
            raise ObservationPathError('observation CAD requires the fixed Alicia tool axes')
        gripper = GripperGeometry(
            float(cfg.get('max_inner_gap_m', .05)),
            float(cfg.get('width_safety_margin_per_side_m', .002)),
            np.asarray(cfg.get('finger_box_xyz_m', [.0434, .0286, .0600])),
            np.asarray(cfg.get('palm_box_xyz_m', [.1175, .1550, .0774])),
            float(cfg.get('support_clearance_m', .003)))
        mismatch = gripper_contract_mismatch_reason(
            gripper, .05, .05, ANALYTICAL_GRIPPER_MODEL_NAME, .05,
            tool_jaw_axis='y', tool_finger_length_axis='z')
        if mismatch:
            raise ObservationPathError('gripper CAD contract: ' + mismatch)
        opening = self._observation_measured_opening()
        desired = self._observation_controller_hold(names)
        constraints = deepcopy(rospy.get_param('/alicia_controller/constraints', None))
        tracking_allowance = observation_tracking_error_bounds(constraints, names)
        contracted = getattr(planner, 'observation_tracking_contract_required', False)
        stop_duration = rospy.get_param('/alicia_controller/stop_trajectory_duration', None)
        original_digest = wire_digest(trajectory)
        if contracted:
            from alicia_flexible_grasp.robot.observation_tracking_contract import qualify_contract_path
            result = qualify_contract_path(trajectory, desired, scene, model, gripper,
                                           opening, constraints, stop_duration,
                                           trajectory_stop_reserve=context_validator is not None)
            if not result['ok']:
                raise ObservationPathError(str(result))
            audit = result['continuous_path']
            audit['execution_tracking_contract'] = result['execution_tracking_contract']
            audit['command_derivative_bounds'] = result['command_derivative_bounds']
            audit['tracking_contract_search'] = result['tracking_contract_search']
        else:
            audit = validate_observation_trajectory(trajectory, scene, model, gripper,
                opening, desired, joint_error_bounds_rad=tracking_allowance,
                max_checks=FOLLOWING_PATH_MAX_CHECKS, max_seconds=FOLLOWING_PATH_MAX_SECONDS)
        # Planning, proof computation and callbacks do not authorize changed
        # geometry, a changed hold, a different model or a mutated trajectory.
        with self._observation_context_lock:
            self._require_live_observation_task()
            if context_validator is None:
                self._observation_contexts.validate_capture(scene, revocation)
            else:
                context_validator()
        after = self._observation_controller_hold(names)
        if (after.positions != desired.positions
                or self._observation_measured_opening() != opening
                or rospy.get_param('/robot_description', '') != description
                or rospy.get_param('/grasp_6d/remote/gripper_geometry', {}) != cfg
                or rospy.get_param('/alicia_controller/constraints', None) != constraints
                or (contracted and (rospy.get_param('/robot/observation_tracking_contract_enabled', False) is not True
                    or rospy.get_param('/alicia_controller/stop_trajectory_duration', None) != stop_duration))
                or planner._observation_branch_signature()[0] != model_signature
                or wire_digest(trajectory) != original_digest
                or self._manual_control_active()):
            raise ObservationPathError('observation path inputs changed before submission')
        audit['model_group_frame_tool_sha256'] = model_signature
        audit['following_allowance_source'] = (audit['execution_tracking_contract']['policy'] if contracted
            else 'configured_controller_path_band_floor_0.12_plus_sdk_half_count')
        audit['controller_constraints_snapshot'] = constraints
        audit['gripper_geometry_snapshot'] = cfg
        audit['controller_hold_provenance'] = deepcopy(self._observation_sync_hold_evidence)
        return audit

    def _observation_execution_authorized(self):
        try:
            with self._observation_context_lock:
                self._require_live_observation_task()
            return not self._manual_control_active()
        except Exception:
            return False

    def _revalidate_observation_contract_submission(self, context, trajectory, audit,
                                                  context_validator=None):
        scene, revocation = context
        with self._observation_context_lock:
            self._require_live_observation_task()
            if context_validator is None:
                self._observation_contexts.validate_capture(scene, revocation)
            else:
                context_validator()
        contract = audit['execution_tracking_contract']
        if (not self._observation_execution_authorized()
                or rospy.get_param('/robot/observation_tracking_contract_enabled', False) is not True
                or rospy.get_param('/alicia_controller/constraints', None) != audit['controller_constraints_snapshot']
                or rospy.get_param('/alicia_controller/stop_trajectory_duration', None) != contract['stop_trajectory_duration_sec']
                or rospy.get_param('/grasp_6d/remote/gripper_geometry', {}) != audit['gripper_geometry_snapshot']
                or self._observation_measured_opening() != audit['opening_width_m']
                or hashlib.sha256(rospy.get_param('/robot_description', '').encode('utf-8')).hexdigest() != audit['model_sha256']
                or wire_digest(trajectory) != audit['trajectory_sha256']):
            raise ObservationPathError('observation action contract changed before submission')

    def handle_observation_pose_strict(self, req):
        return self._with_observation_planner(req, self.handle_pose_strict)

    def handle_observation_pose_strict_execute(self, req):
        return self._with_observation_planner(req, self.handle_pose_strict_execute)

    def handle_observation_pose_strict_plan_execute(self, req):
        return self._with_observation_planner(req, self.handle_pose_strict_plan_execute)

    def _manual_control_active(self):
        return (getattr(self, '_gui_direct_mode', False) or
                rospy.get_param('/gui/joint_direct_mode', False) is True)

    def _actuation_cb(self, msg):
        # Status is also a heartbeat on every accepted SDK feedback packet.
        # Only the driver's separately stamped reset event revokes an epoch.
        self._actuation_status = str(msg.data).strip()
        self._actuation_received_sec = float(rospy.get_time())

    def _fresh_actuation_status(self):
        age = float(rospy.get_time()) - getattr(self, '_actuation_received_sec', 0.0)
        return getattr(self, '_actuation_status', '') if 0.0 <= age <= 2.0 else ''

    def handle_observation_actuation(self, req):
        if self._manual_control_active():
            return SetTargetPoseResponse(False, 'MANUAL_CONTROL_ACTIVE: joint sliders own motion')
        if not req.execute:
            return SetTargetPoseResponse(False, 'ACTUATION_PREFIX requires execute=true')
        with self._planner_operation_lock():
            status = self._fresh_actuation_status()
            if status.startswith('CONFIRMED:'):
                return SetTargetPoseResponse(True, 'actuation already confirmed')
            if status not in PENDING_BOOTSTRAP_STATES:
                return SetTargetPoseResponse(False, 'ACTUATION_PREFIX_NOT_PERMITTED: ' + status)
            planner = self._ensure_planner()
            if planner is None:
                return SetTargetPoseResponse(False, self._moveit_not_ready_message())
            ok, message = self._ensure_trajectory_controllers_started()
            if not ok:
                return SetTargetPoseResponse(False, message)
            ok, message = self._synchronize_trajectory_controller_to_feedback()
            if not ok:
                return SetTargetPoseResponse(False, message)
            # The controller bridge already sends a synchronized reference.
            # Sending encoder positions again ratchets servo setpoint/feedback
            # offset into a second real move. Plan only after measured settling.
            ok, message = self._wait_for_stationary_arm_feedback()
            if not ok:
                return SetTargetPoseResponse(False, message)
            ok, message = planner.move_to_pose(req.target, execute=False, allow_fallbacks=False)
            if not ok:
                return SetTargetPoseResponse(False, 'ACTUATION_PREFIX_PLAN_FAILED: ' + message)
            status = self._fresh_actuation_status()
            if status.startswith('CONFIRMED:'):
                return SetTargetPoseResponse(True, 'actuation confirmed during synchronization')
            if status not in PENDING_BOOTSTRAP_STATES:
                return SetTargetPoseResponse(False, 'ACTUATION_PREFIX_STATE_CHANGED: ' + status)
            rospy.logwarn('Executing bounded observation prefix to establish measured actuation response')
            if self._manual_control_active():
                return SetTargetPoseResponse(False, 'MANUAL_CONTROL_ACTIVE')
            ok, message = planner.execute_cached_observation_prefix(req.target)
            if not ok:
                return SetTargetPoseResponse(False, message)
            deadline = float(rospy.get_time()) + 1.0
            while not rospy.is_shutdown() and float(rospy.get_time()) < deadline:
                if self._fresh_actuation_status().startswith('CONFIRMED:'):
                    return SetTargetPoseResponse(True, message + '; measured actuation confirmed')
                rospy.sleep(0.02)
            return SetTargetPoseResponse(False, 'ACTUATION_PREFIX_NO_RESPONSE: ' + self._fresh_actuation_status())

    def _wait_for_stationary_arm_feedback(self):
        started = float(rospy.get_time())
        stable_since = started
        anchor = None
        while not rospy.is_shutdown() and float(rospy.get_time()) - started <= 2.0:
            if self._manual_control_active():
                return False, 'MANUAL_CONTROL_ACTIVE'
            now = float(rospy.get_time())
            stamp = getattr(self.joint_cmd, 'last_state_time_sec', None)
            measured = self._current_arm_positions_snapshot()
            if (stamp is None or not 0.0 <= now-float(stamp) <= 0.5 or
                    len(measured) != 6 or not all(math.isfinite(v) for v in measured)):
                return False, 'ACTUATION_PREFIX_FEEDBACK_STALE'
            if anchor is None or max(abs(a-b) for a,b in zip(anchor, measured)) > 2.0*math.pi/4096.0 + 1e-9:
                anchor, stable_since = list(measured), now
            if now-stable_since >= 0.25:
                return True, 'measured arm positions stationary after controller synchronization'
            rospy.sleep(0.02)
        return False, 'ACTUATION_PREFIX_FEEDBACK_NOT_SETTLED'

    @staticmethod
    def _tracking_probe_scene_max_age():
        config = rospy.get_param('/grasp', {})
        validity = float(config.get('plan_validity_sec',
            rospy.get_param('/grasp_6d/plan_validity_sec', 2.)))
        if not math.isfinite(validity) or not 0 < validity <= 120.:
            raise ObservationPathError('invalid existing scene freshness contract')
        return validity

    def _tracking_probe_context(self):
        """New explicit diagnostic authority, never resurrection of a grasp.

        Only an idle connected task and fresh committed post-epoch geometry
        can support a <=4-count probe. A terminal grasp's active scene is not
        reused, and no task-active flag or old trajectory is manufactured.
        """
        epoch = getattr(self, '_control_reference_epoch_ns', None)
        if self._manual_control_active() or epoch is None:
            raise ObservationPathError('probe requires released manual control and current epoch')
        with self._observation_context_lock:
            if (not self._observation_task_connection_available()
                    or self._observation_task_state_stamp_ns <= 0
                    or self._observation_task_requires_inactive
                    or self._observation_contexts.active):
                raise ObservationPathError('probe requires a connected explicitly inactive task')
            scenes = [s for s in self._observation_contexts.entries.values() if s is not None]
            if not scenes:
                raise ObservationPathError('probe requires fresh committed scene geometry')
            scene = max(scenes, key=lambda s: s.source_ns)
            if (scene.source_ns < epoch
                    or not 0 <= rospy.get_time()-scene.source_ns*1e-9 <= self._tracking_probe_scene_max_age()):
                raise ObservationPathError('probe scene predates epoch or is stale/future')
            return scene, epoch, self._observation_contexts.revocation

    def _validate_tracking_probe_context(self, context):
        scene, epoch, revocation = context
        newest, now_epoch, now_revocation = self._tracking_probe_context()
        if (now_epoch != epoch or now_revocation != revocation
                or newest.track_id != scene.track_id
                or not 0 <= rospy.get_time()-scene.source_ns*1e-9 <= self._tracking_probe_scene_max_age()):
            raise ObservationPathError('probe geometry/ownership context changed')

    def _validate_tracking_probe_path(self, context, baseline, goal, trajectory, planner):
        self._validate_tracking_probe_context(context)
        scene = context[0]
        names = tuple(trajectory.joint_trajectory.joint_names)
        if set(names) != set(self.joint_names[:-1]) or len(names) != 6:
            raise ObservationPathError('probe trajectory joint names differ')
        if (planner.manipulator.get_planning_frame() != 'base_link'
                or planner.manipulator.get_end_effector_link() != 'tool0'):
            raise ObservationPathError('probe requires base_link/tool0')
        canonical = tuple('Joint%d' % i for i in range(1, 7))
        baseline_by_name = dict(zip(canonical, baseline))
        goal_by_name = dict(zip(canonical, goal))
        b = np.array([baseline_by_name[n] for n in names])
        points = list(trajectory.joint_trajectory.points)
        if (len(points) < 2 or any(len(p.positions) != 6 for p in points)
                or max(abs(points[-1].positions[i]-goal_by_name[n]) for i, n in enumerate(names)) > 1e-12):
            raise ObservationPathError('probe joint goal changed')
        description = rospy.get_param('/robot_description', '')
        model = SerialUrdfFk(description, names)
        cfg = deepcopy(rospy.get_param('/grasp_6d/remote/gripper_geometry', {}))
        gripper = GripperGeometry(float(cfg.get('max_inner_gap_m', .05)),
            float(cfg.get('width_safety_margin_per_side_m', .002)),
            np.asarray(cfg.get('finger_box_xyz_m', [.0434, .0286, .0600])),
            np.asarray(cfg.get('palm_box_xyz_m', [.1175, .1550, .0774])),
            float(cfg.get('support_clearance_m', .003)))
        mismatch = gripper_contract_mismatch_reason(gripper, .05, .05,
            ANALYTICAL_GRIPPER_MODEL_NAME, .05, tool_jaw_axis=cfg.get('tool_jaw_axis', 'y'),
            tool_finger_length_axis=cfg.get('tool_finger_length_axis', 'z'))
        if mismatch:
            raise ObservationPathError(mismatch)
        opening = self._observation_measured_opening()
        desired = self._observation_controller_hold(names)
        if self._sdk_arm_words([desired.positions[names.index(n)] for n in canonical]) != self._sdk_arm_words(baseline):
            raise ObservationPathError('probe baseline changed before final path proof')
        audit = validate_observation_trajectory(trajectory, scene, model, gripper, opening, desired)
        # Bound the final timed spline, not just its waypoints. Nominal .012
        # plus the unchanged .035 goal error fits inside a .05 local box.
        segments = list(zip(points, points[1:]))
        first_positive = next((p for p in points if p.time_from_start.to_sec() > 0.), None)
        if first_positive is None:
            raise ObservationPathError('probe has no positive-time waypoint')
        bridge = SimpleNamespace(positions=desired.positions, velocities=[0.]*6,
                                 accelerations=[0.]*6, time_from_start=rospy.Duration(0.))
        segments.append((bridge, first_positive))
        for first, second in segments:
            duration = second.time_from_start.to_sec()-first.time_from_start.to_sec()
            coefficients = segment_coefficients(first, second, duration, 6)
            low, high = _polynomial_bounds(coefficients, 0., 1.)
            if np.any(low < b-.012) or np.any(high > b+.012):
                raise ObservationPathError('probe continuous path leaves local .012 rad box')
        support = observation_endpoint_support_bound(model, b, [.05]*6,
            scene.normal, scene.offset, gripper, opening)
        away = model(b)[:3, 3]-scene.center
        away /= np.linalg.norm(away)
        offset = -float(away @ scene.center)-float(np.sum(np.abs(away @ scene.rotation)*scene.size/2))
        target = observation_endpoint_support_bound(model, b, [.05]*6, away, offset, gripper, opening)
        if not support['ok'] or not target['ok']:
            raise ObservationPathError('probe lacks support/target uncertainty margin')
        self._validate_tracking_probe_context(context)
        after = self._observation_controller_hold(names)
        if (after.positions != desired.positions or self._observation_measured_opening() != opening
                or rospy.get_param('/robot_description', '') != description
                or rospy.get_param('/grasp_6d/remote/gripper_geometry', {}) != cfg):
            raise ObservationPathError('probe model/opening/reference changed during proof')
        deadline = getattr(self, '_endpoint_correction_deadline', None)
        if deadline is not None:
            duration = points[-1].time_from_start.to_sec()
            if (rospy.get_time() + duration + 2. >= deadline[0]
                    or time.monotonic() + duration + 2. >= deadline[1]):
                raise ObservationPathError('ENDPOINT_CORRECTION_INSUFFICIENT_STEP_BUDGET')
        audit.update(diagnostic_only=True, endpoint_support_bound=support,
                     endpoint_target_separation_bound=target)
        return audit

    def handle_tracking_probe(self, req):
        """Explicit one-shot tiny multi-joint diagnostic, never a grasp retry."""
        return self._handle_bounded_joint_probe(req, max_counts=4)

    def handle_joint_response_characterization(self, req):
        """One opt-in measurement per handoff, inside the existing .012 rad box.

        Seven encoder counts are 0.010738 rad. This separate diagnostic can
        characterize a small-step dead zone without changing the production
        four-count correction rule or integrating through missing responses.
        A reload cannot rearm a consumed handoff. No automatic grasp calls it.
        """
        return self._handle_bounded_joint_probe(req, max_counts=7,
                                              characterization=True)

    def _handle_bounded_joint_probe(self, req, *, max_counts, characterization=False):
        with self._planner_operation_lock():
            planner = self._ensure_planner()
            if planner is None:
                return SetJointCommandResponse(False, self._moveit_not_ready_message())
            original_guard = planner.observation_path_guard_required
            original_validator = planner.observation_path_validator
            try:
                if (characterization and req.execute and rospy.get_param(
                        '~enable_joint_response_characterization', False) is not True):
                    raise ObservationPathError('JOINT_RESPONSE_CHARACTERIZATION_NOT_ENABLED')
                context = self._tracking_probe_context()
                reference = self._command_reference_snapshot()
                if reference is None:
                    raise ObservationPathError('probe SDK baseline unavailable')
                canonical = tuple('Joint%d' % i for i in range(1, 7))
                reference_by_name = dict(zip(self.joint_names[:-1], reference.positions))
                baseline = [reference_by_name[n] for n in canonical]
                goal = list(req.positions)
                delta = np.asarray(self._sdk_arm_words(goal))-np.asarray(self._sdk_arm_words(baseline))
                if len(goal) != 6 or not np.any(delta) or np.max(np.abs(delta)) > max_counts:
                    raise ObservationPathError('probe requires nonzero <=%d SDK counts per joint'
                                               % max_counts)
                if max(abs(a-b) for a, b in zip(goal, baseline)) > .012:
                    raise ObservationPathError('probe target leaves local .012 rad box')
                actual_by_name = dict(zip(self.joint_names, self._fresh_accepted_reference()))
                actual = [actual_by_name[n] for n in canonical]
                if max(abs(a-b) for a, b in zip(actual, baseline)) > .035:
                    raise ObservationPathError('probe initial following exceeds existing .035 rad contract')
                if req.execute:
                    if characterization:
                        key = '/supervisor/joint_response_characterization_episode'
                        prior = rospy.get_param(key, {})
                        if (str(prior.get('epoch_ns', '')) == str(context[1])
                                and prior.get('state') != 'rejected_before_submission_verified'):
                            raise ObservationPathError('JOINT_RESPONSE_CHARACTERIZATION_EPOCH_CONSUMED')
                    ok, message = self._ensure_trajectory_controllers_started()
                    if not ok:
                        raise ObservationPathError(message)
                    ok, message = self._synchronize_trajectory_controller_to_feedback()
                    if not ok:
                        raise ObservationPathError(message)
                    self._validate_tracking_probe_context(context)
                    after = self._command_reference_snapshot()
                    after_by_name = dict(zip(self.joint_names[:-1], after.positions))
                    if self._sdk_arm_words([after_by_name[n] for n in canonical]) != self._sdk_arm_words(baseline):
                        raise ObservationPathError('probe baseline changed at handoff')
                    planner.observation_path_guard_required = True
                    planner.observation_path_validator = lambda trajectory: self._validate_tracking_probe_path(
                        context, baseline, goal, trajectory, planner)
                def consume_before_execution():
                    self._validate_tracking_probe_context(context)
                    rospy.set_param('/supervisor/joint_response_characterization_episode', {
                        'epoch_ns': str(context[1]),
                        'baseline_counts': list(self._sdk_arm_words(baseline)),
                        'goal_counts': list(self._sdk_arm_words(goal)),
                        'max_counts_per_axis': max_counts,
                        'scene_plan_id': context[0].plan_id,
                        'state': 'consumed_before_submission_no_automatic_rearm',
                    })
                options = {'start_joint_positions': baseline}
                if characterization and req.execute:
                    options['before_execute'] = consume_before_execution
                ok, message = planner.plan_and_execute_joint_probe(
                    goal, execute=bool(req.execute), **options)
                return SetJointCommandResponse(ok, message)
            except Exception as exc:
                return SetJointCommandResponse(False, 'JOINT_TRACKING_PROBE_REJECTED: '+str(exc))
            finally:
                planner.observation_path_guard_required = original_guard
                planner.observation_path_validator = original_validator

    def handle_joints(self, req):
        if req.execute and self._manual_control_active():
            return SetJointCommandResponse(False, 'MANUAL_CONTROL_ACTIVE')
        with self._planner_operation_lock():
            planner = self._ensure_planner()
            if planner is None:
                return SetJointCommandResponse(False, self._moveit_not_ready_message())
            ok,msg = planner.move_to_joints(req.positions, execute=req.execute)
        return SetJointCommandResponse(ok, msg)

    def _endpoint_following_snapshot(self, fk, epoch, context, *, after_ns=0,
                                     context_validator=None):
        deadline = time.monotonic() + 2.
        last = 'no stationary post-command samples'
        sampling = {}
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if context_validator is None:
                self._validate_tracking_probe_context(context)
            else:
                context_validator()
            if time.monotonic() >= deadline:
                last = 'sampling deadline expired during context validation'
                break
            capture_started = time.monotonic()
            with self._endpoint_feedback_lock:
                # The callback already owns a deep copy of each message and
                # only appends/replaces deque entries. Readers never mutate
                # those messages. Freeze their references without blocking
                # incoming SDK samples while cloning the full history again.
                sdk = tuple(self._endpoint_feedback_history['sdk'])
                accepted = tuple(self._endpoint_feedback_history['accepted'])
            now_sec = rospy.get_time()
            sampling = {
                'now_sec': now_sec, 'epoch_ns': epoch,
                'capture_sec': time.monotonic() - capture_started,
                'sdk_count': len(sdk), 'accepted_count': len(accepted),
                'sdk_latest_stamp_ns': sdk[-1].header.stamp.to_nsec() if sdk else None,
                'accepted_latest_stamp_ns': accepted[-1].header.stamp.to_nsec() if accepted else None,
            }
            try:
                report = stationary_following_error(fk, sdk, accepted,
                    now_sec=now_sec, epoch_ns=epoch)
            except ValueError as exc:
                last = str(exc)
            else:
                if report['stationary_window_start_ns'] > after_ns:
                    # Validation/FK can run across a callback or scheduler
                    # delay. A sample fresh at capture is not automatically
                    # fresh when handed to the correction planner.
                    if context_validator is None:
                        self._validate_tracking_probe_context(context)
                    else:
                        context_validator()
                    completed_sec = rospy.get_time()
                    sampling['completed_sec'] = completed_sec
                    sampling['validation_sec'] = time.monotonic() - capture_started
                    if time.monotonic() >= deadline:
                        last = 'sampling deadline expired during evidence validation'
                        break
                    if (math.isfinite(completed_sec)
                            and all(0. <= completed_sec - report[key] * 1e-9 <= .5
                                    for key in ('sdk_stamp_ns', 'accepted_stamp_ns'))):
                        return report
                    last = 'SDK or accepted evidence expired during validation'
                else:
                    last = 'stationary window does not follow the required command boundary'
            remaining = deadline - time.monotonic()
            if remaining > 0.:
                rospy.sleep(min(.05, remaining))
        raise ObservationPathError('ENDPOINT_CORRECTION_FEEDBACK_UNAVAILABLE: '+last
                                   + '; snapshot=' + json.dumps(sampling, sort_keys=True))

    def handle_endpoint_feedback_correction(self, req):
        """Bounded opt-in free-space experiment; no automatic grasp authority.

        Keep one planner lock through the episode. Each step uses the existing
        strict probe's fresh ownership, geometry, whole-path, velocity and
        reference handoff checks. A controller success alone never grants the
        next correction. A consumed epoch prevents accidental repeat calls
        from freezing the compensated SDK target as a NEW physical goal.
        """
        with self._planner_operation_lock():
            evidence = {}
            sample = correction = None
            try:
                execute = bool(req.trigger)
                if execute and rospy.get_param('~enable_endpoint_correction_experiments', False) is not True:
                    raise ObservationPathError('ENDPOINT_CORRECTION_EXPERIMENT_NOT_ENABLED')
                context = self._tracking_probe_context()
                epoch = context[1]
                consumed = rospy.get_param('/supervisor/endpoint_correction_episode', {})
                if execute and (getattr(self, '_endpoint_correction_consumed_epoch', None) == epoch
                                or str(consumed.get('epoch_ns', '')) == str(epoch)):
                    raise ObservationPathError('ENDPOINT_CORRECTION_EPOCH_ALREADY_CONSUMED')
                description = rospy.get_param('/robot_description', '')
                fk = SerialUrdfFk(description, ARM_NAMES)
                sample = self._endpoint_following_snapshot(fk, epoch, context)
                correction = EndpointCorrection(fk, sample)
                reference = self._command_reference_snapshot()
                if reference is None:
                    raise ObservationPathError('ENDPOINT_CORRECTION_REFERENCE_MISSING')
                named = dict(zip(self.joint_names[:-1], reference.positions))
                if self._sdk_arm_words([named[n] for n in ARM_NAMES]) != correction.goal_counts:
                    raise ObservationPathError('ENDPOINT_CORRECTION_REFERENCE_CHANGED')
                # The original goal is saved BEFORE motion and never follows
                # the compensated controller setpoint. No rollback on error.
                if execute:
                    # Store nanoseconds as a string: ROS XML-RPC integers are
                    # only 32-bit. Preserve this latch across gateway reload.
                    rospy.set_param('/supervisor/endpoint_correction_episode', {
                        'epoch_ns': str(epoch), 'fixed_goal_counts': list(correction.goal_counts),
                        'scene_plan_id': context[0].plan_id,
                        'state': 'consumed_before_any_step_no_automatic_rearm'})
                    self._endpoint_correction_consumed_epoch = epoch
                wall_deadline = time.monotonic() + 30.
                if execute:
                    self._endpoint_correction_deadline = (correction.started + 30., wall_deadline)
                for _ in range(7):
                    self._validate_tracking_probe_context(context)
                    if (time.monotonic() >= wall_deadline
                            or rospy.get_param('/robot_description', '') != description):
                        raise ObservationPathError('ENDPOINT_CORRECTION_DEADLINE_OR_MODEL_CHANGED')
                    code, step, evidence = correction.evaluate(sample)
                    evidence.update(code=code, diagnostic_only=True,
                                    scene_plan_id=context[0].plan_id, epoch_ns=epoch,
                                    execution_requested=execute)
                    if step is None:
                        rospy.loginfo('Fixed-goal endpoint correction: %s', json.dumps(evidence))
                        return TriggerZeroResponse(True, json.dumps(evidence))
                    evidence['proposed_sdk_delta_counts'] = [b-a for a,b in
                        zip(step.baseline_counts, step.target_counts)]
                    rospy.loginfo('Fixed-goal endpoint correction: %s', json.dumps(evidence))
                    # Do not replace a changed reference with the planned step.
                    reference = self._command_reference_snapshot()
                    if reference is None:
                        raise ObservationPathError('ENDPOINT_CORRECTION_REFERENCE_MISSING')
                    named = dict(zip(self.joint_names[:-1], reference.positions))
                    if self._sdk_arm_words([named[n] for n in ARM_NAMES]) != step.baseline_counts:
                        raise ObservationPathError('ENDPOINT_CORRECTION_REFERENCE_CHANGED')
                    result = self.handle_tracking_probe(SimpleNamespace(
                        positions=list(step.positions), execute=execute))
                    if not result.success:
                        raise ObservationPathError('ENDPOINT_CORRECTION_STEP_REJECTED: '+result.message)
                    if not execute:
                        evidence['code'] = 'ENDPOINT_CORRECTION_PLANNED_ONLY'
                        evidence['physical_path_authorized'] = False
                        return TriggerZeroResponse(True, json.dumps(evidence))
                    completed = rospy.get_time()
                    correction.committed(step, completed)
                    sample = self._endpoint_following_snapshot(
                        fk, epoch, context, after_ns=int(completed*1e9))
                raise ObservationPathError('ENDPOINT_CORRECTION_STEP_BUDGET')
            except Exception as exc:
                if correction is not None and correction.last_evidence is not None:
                    evidence.update(deepcopy(correction.last_evidence))
                    evidence['measurement_scope'] = 'last_valid_evaluated_sample_at_explicit_source_stamps'
                evidence.update(code=str(exc), success=False, diagnostic_only=True,
                                no_implicit_rollback_or_torque_disable=True)
                if sample is not None:
                    evidence['last_measured_report'] = sample
                if correction is not None:
                    evidence['steps_completed'] = correction.steps
                rospy.logwarn('Fixed-goal endpoint correction ended: %s', json.dumps(evidence))
                return TriggerZeroResponse(False, json.dumps(evidence))
            finally:
                self._endpoint_correction_deadline = None

    def handle_gripper(self, req):
        if self._manual_control_active():
            return SetFloatResponse(False, 'MANUAL_CONTROL_ACTIVE')
        try:
            arm_positions = self._gripper_arm_positions_for_command()
        except ObservationPathError as exc:
            return SetFloatResponse(False, 'CONTROLLER_REFERENCE_INVALID: '+str(exc))
        self.gripper.set_position(req.value, arm_positions=arm_positions)
        return SetFloatResponse(True, 'gripper command published')

    def handle_pose(self, req):
        if req.execute and self._manual_control_active():
            return SetTargetPoseResponse(False, 'MANUAL_CONTROL_ACTIVE')
        self._log_pose_request(req)
        with self._planner_operation_lock():
            planner = self._ensure_planner()
            if planner is None:
                msg = self._moveit_not_ready_message()
                rospy.logwarn('move_to_pose result success=False message=%s', msg)
                return SetTargetPoseResponse(False, msg)
            if req.execute:
                controllers_ok, controller_msg = self._ensure_trajectory_controllers_started()
                if not controllers_ok:
                    msg = 'execute blocked: %s' % controller_msg
                    rospy.logwarn('move_to_pose result success=False message=%s', msg)
                    return SetTargetPoseResponse(False, msg)
                rospy.loginfo('trajectory controller check passed: %s', controller_msg)
            ok,msg = planner.move_to_pose(req.target, execute=req.execute)
        if ok:
            rospy.loginfo('move_to_pose result success=True message=%s', msg)
        else:
            rospy.logwarn('move_to_pose result success=False message=%s', msg)
        return SetTargetPoseResponse(ok, msg)

    def handle_pose_strict(self, req):
        self._log_pose_request(req, operation='check_pose_strict')
        if req.execute:
            return SetTargetPoseResponse(False, 'strict pose service is planning-only')
        with self._planner_operation_lock():
            planner = self._ensure_planner()
            if planner is None:
                msg = self._moveit_not_ready_message()
                rospy.logwarn('check_pose_strict result success=False message=%s', msg)
                return SetTargetPoseResponse(False, msg)
            ok, msg = planner.move_to_pose(
                req.target,
                execute=False,
                allow_fallbacks=False,
            )
        if ok:
            rospy.loginfo('check_pose_strict result success=True message=%s', msg)
        else:
            rospy.logwarn('check_pose_strict result success=False message=%s', msg)
        return SetTargetPoseResponse(ok, msg)

    def handle_pose_sequence_strict(self, req):
        with self._planner_operation_lock():
            planner = self._ensure_planner()
            if planner is None:
                message = self._moveit_not_ready_message()
                rospy.logwarn(
                    'check_pose_sequence_strict result success=False message=%s',
                    message,
                )
                return CheckPoseSequenceResponse(
                    False,
                    'MOVEIT_CHECK_ERROR',
                    '',
                    0.0,
                    0.0,
                    message,
                )
            ok, code, failed_stage, metrics, message = planner.check_pose_sequence(
                req.targets,
                req.stage_names,
                req.linear,
                deadline_sec=self._request_deadline_sec(req),
            )
        response = CheckPoseSequenceResponse(
            bool(ok),
            str(code or ''),
            str(failed_stage or ''),
            float(metrics.get('path_cost', 0.0)),
            float(metrics.get('max_delta', 0.0)),
            str(message or ''),
        )
        log = rospy.loginfo if response.success else rospy.logwarn
        log(
            'check_pose_sequence_strict result success=%s failed_stage=%s '
            'message=%s',
            response.success,
            response.failed_stage,
            response.message,
        )
        return response

    def handle_resolve_free_space_orientations(self, req):
        with self._planner_operation_lock():
            planner = self._ensure_planner()
            if planner is None:
                message = self._moveit_not_ready_message()
                rospy.logwarn(
                    'resolve_free_space_orientations result success=False '
                    'message=%s',
                    message,
                )
                return ResolveFreeSpaceOrientationsResponse(
                    False,
                    'MOVEIT_RESOLVE_ERROR',
                    '',
                    [],
                    0.0,
                    0.0,
                    0.0,
                    message,
                )
            (
                ok,
                code,
                failed_stage,
                resolved,
                metrics,
                message,
            ) = planner.resolve_free_space_orientations(
                req.targets,
                req.stage_names,
                req.linear,
                req.resolve_orientation,
                deadline_sec=self._request_deadline_sec(req),
            )
        policy_attestation = (
            'policy=deterministic_geodesic_collision_ik'
        )
        message = str(message or '')
        if policy_attestation not in message:
            message = '%s; %s' % (policy_attestation, message)
        response = ResolveFreeSpaceOrientationsResponse(
            bool(ok),
            str(code or ''),
            str(failed_stage or ''),
            list(resolved or ()),
            float(metrics.get('path_cost', 0.0)),
            float(metrics.get('max_delta', 0.0)),
            float(metrics.get('max_position_error', 0.0)),
            message,
        )
        log = rospy.loginfo if response.success else rospy.logwarn
        log(
            'resolve_free_space_orientations result success=%s '
            'failed_stage=%s message=%s',
            response.success,
            response.failed_stage,
            response.message,
        )
        return response

    @staticmethod
    def _request_deadline_sec(req):
        deadline = getattr(req, 'deadline', None)
        converter = getattr(deadline, 'to_sec', None)
        if not callable(converter):
            return 0.0
        try:
            deadline_sec = float(converter())
        except (TypeError, ValueError, OverflowError):
            return 0.0
        return (
            deadline_sec
            if math.isfinite(deadline_sec) and deadline_sec > 0.0
            else 0.0
        )

    def handle_pose_strict_execute(self, req):
        if self._manual_control_active():
            return SetTargetPoseResponse(False, 'MANUAL_CONTROL_ACTIVE')
        self._log_pose_request(req, operation='execute_pose_strict')
        if not req.execute:
            return SetTargetPoseResponse(
                False,
                'strict cached pose execution service requires execute=true',
            )
        with self._planner_operation_lock():
            planner = self._ensure_planner()
            if planner is None:
                msg = self._moveit_not_ready_message()
                rospy.logwarn('execute_pose_strict result success=False message=%s', msg)
                return SetTargetPoseResponse(False, msg)
            controllers_ok, controller_msg = self._ensure_trajectory_controllers_started()
            if not controllers_ok:
                msg = 'strict cached execute blocked: %s' % controller_msg
                rospy.logwarn('execute_pose_strict result success=False message=%s', msg)
                return SetTargetPoseResponse(False, msg)
            rospy.loginfo('strict cached trajectory controller check passed: %s', controller_msg)
            sync_ok, sync_message = (
                self._synchronize_trajectory_controller_to_feedback()
            )
            if not sync_ok:
                msg = 'strict cached execute blocked: %s' % sync_message
                rospy.logwarn('execute_pose_strict result success=False message=%s', msg)
                return SetTargetPoseResponse(False, msg)
            rospy.loginfo('strict cached trajectory controller sync passed: %s', sync_message)
            execution_start_positions = (
                self._current_arm_positions_snapshot()
            )
            ok, msg = planner.execute_cached_strict_pose(req.target)
            if not ok and not any(code in str(msg) for code in (
                    'OBSERVATION_PATH_INVALID:', 'CONTROLLER_REFERENCE_INVALID',
                    'CONTROLLER_BRIDGE_INVALID')):
                msg = self._hold_feedback_after_failed_execution(
                    'execute_pose_strict',
                    msg,
                    execution_start_positions,
                )
        if ok:
            rospy.loginfo('execute_pose_strict result success=True message=%s', msg)
        else:
            rospy.logwarn('execute_pose_strict result success=False message=%s', msg)
        return SetTargetPoseResponse(ok, msg)

    def handle_pose_strict_plan_execute(self, req):
        if self._manual_control_active():
            return SetTargetPoseResponse(False, 'MANUAL_CONTROL_ACTIVE')
        operation = 'plan_and_execute_pose_strict'
        self._log_pose_request(req, operation=operation)
        if not req.execute:
            return SetTargetPoseResponse(
                False,
                'atomic strict pose execution service requires execute=true',
            )
        with self._planner_operation_lock():
            planner = self._ensure_planner()
            if planner is None:
                msg = self._moveit_not_ready_message()
                rospy.logwarn('%s result success=False message=%s', operation, msg)
                return SetTargetPoseResponse(False, msg)
            controllers_ok, controller_msg = self._ensure_trajectory_controllers_started()
            if not controllers_ok:
                msg = 'atomic strict execute blocked: %s' % controller_msg
                rospy.logwarn('%s result success=False message=%s', operation, msg)
                return SetTargetPoseResponse(False, msg)
            rospy.loginfo(
                'atomic strict trajectory controller check passed: %s',
                controller_msg,
            )
            sync_ok, sync_message = (
                self._synchronize_trajectory_controller_to_feedback()
            )
            if not sync_ok:
                msg = 'atomic strict execute blocked: %s' % sync_message
                rospy.logwarn('%s result success=False message=%s', operation, msg)
                return SetTargetPoseResponse(False, msg)
            rospy.loginfo(
                'atomic strict trajectory controller sync passed: %s',
                sync_message,
            )
            planned, plan_message = planner.move_to_pose(
                req.target,
                execute=False,
                allow_fallbacks=False,
            )
            if not planned:
                msg = 'atomic strict planning failed: %s' % plan_message
                rospy.logwarn('%s result success=False message=%s', operation, msg)
                return SetTargetPoseResponse(False, msg)
            execution_start_positions = (
                self._current_arm_positions_snapshot()
            )
            ok, msg = planner.execute_cached_strict_pose(req.target)
            if not ok and not any(code in str(msg) for code in (
                    'OBSERVATION_PATH_INVALID:', 'CONTROLLER_REFERENCE_INVALID',
                    'CONTROLLER_BRIDGE_INVALID')):
                msg = self._hold_feedback_after_failed_execution(
                    operation,
                    msg,
                    execution_start_positions,
                )
        if ok:
            rospy.loginfo('%s result success=True message=%s', operation, msg)
        else:
            rospy.logwarn('%s result success=False message=%s', operation, msg)
        return SetTargetPoseResponse(ok, msg)

    def handle_pose_linear(self, req):
        if req.execute and self._manual_control_active():
            return SetTargetPoseResponse(False, 'MANUAL_CONTROL_ACTIVE')
        self._log_pose_request(req, operation='move_to_pose_linear')
        with self._planner_operation_lock():
            planner = self._ensure_planner()
            if planner is None:
                msg = self._moveit_not_ready_message()
                rospy.logwarn('move_to_pose_linear result success=False message=%s', msg)
                return SetTargetPoseResponse(False, msg)
            if req.execute:
                controllers_ok, controller_msg = self._ensure_trajectory_controllers_started()
                if not controllers_ok:
                    msg = 'linear execute blocked: %s' % controller_msg
                    rospy.logwarn('move_to_pose_linear result success=False message=%s', msg)
                    return SetTargetPoseResponse(False, msg)
                rospy.loginfo('linear trajectory controller check passed: %s', controller_msg)
            ok, msg = planner.move_to_pose_linear(req.target, execute=req.execute)
        if ok:
            rospy.loginfo('move_to_pose_linear result success=True message=%s', msg)
        else:
            rospy.logwarn('move_to_pose_linear result success=False message=%s', msg)
        return SetTargetPoseResponse(ok, msg)

    def handle_jog(self, req):
        if self._manual_control_active():
            return CartesianJogResponse(False, 'MANUAL_CONTROL_ACTIVE')
        with self._planner_operation_lock():
            planner = self._ensure_planner()
            if planner is None:
                return CartesianJogResponse(False, self._moveit_not_ready_message())
            self.jogger.planner = planner
            ok,msg = self.jogger.jog(req.dx, req.dy, req.dz, req.droll, req.dpitch, req.dyaw, execute=req.execute)
        return CartesianJogResponse(ok, msg)

    def _planner_operation_lock(self):
        lock = getattr(self, '_planner_lock', None)
        if lock is None:
            lock = threading.RLock()
            self._planner_lock = lock
        return lock

    def handle_zero(self, req):
        if req.trigger:
            self.zero_pub.publish(Bool(True))
        return TriggerZeroResponse(True, 'zero command sent')

    def _ensure_planner(self, force=False):
        planner = getattr(self, 'planner', None)
        if planner is not None and getattr(planner, 'ready', True):
            self._configure_controller_reference_guard(planner)
            return planner
        now = rospy.get_time() if not rospy.is_shutdown() else 0.0
        retry_period = float(getattr(self, '_planner_retry_period_sec', 2.0))
        if not force and now - float(getattr(self, '_last_planner_attempt', 0.0)) < retry_period:
            return None
        self._last_planner_attempt = now
        try:
            planner = MoveItPlanner(self.manipulator_group, self.gripper_group, self.velocity)
        except Exception as exc:
            self.planner = None
            self._last_planner_error = str(exc)
            rospy.logwarn_throttle(5.0, 'MoveItPlanner initialization failed: %s', exc)
            return None
        if getattr(planner, 'ready', False):
            self._configure_controller_reference_guard(planner)
            self.planner = planner
            self._last_planner_error = ''
            if hasattr(self, 'jogger'):
                self.jogger.planner = planner
            rospy.loginfo('MoveItPlanner ready: manipulator=%s gripper=%s', self.manipulator_group, self.gripper_group)
            return planner
        self.planner = None
        self._last_planner_error = getattr(planner, 'error', '') or 'MoveIt not ready'
        rospy.logwarn_throttle(5.0, 'MoveItPlanner not ready yet: %s', self._last_planner_error)
        return None

    def _moveit_not_ready_message(self):
        detail = getattr(self, '_last_planner_error', '') or 'robot_description/move_group is not available'
        return 'MoveIt not ready: %s' % detail

    def _ensure_trajectory_controllers_started(self):
        if SwitchController is None or SwitchControllerRequest is None:
            return True, 'controller_manager_msgs unavailable; skipped controller switch'
        controller_names = list(self.trajectory_controller_names)

        # Re-sending a start request to an already-running position controller
        # can expose its retained command before the feedback-hold bridge is
        # installed.  Query first and make this operation idempotent.
        if ListControllers is not None:
            try:
                rospy.wait_for_service(
                    '/controller_manager/list_controllers',
                    timeout=1.0,
                )
                list_srv = rospy.ServiceProxy(
                    '/controller_manager/list_controllers',
                    ListControllers,
                )
                list_res = list_srv()
                states = {
                    str(controller.name): str(controller.state)
                    for controller in getattr(list_res, 'controller', [])
                }
                missing = self._non_running_controllers(
                    states,
                    controller_names,
                )
                if not missing:
                    return True, (
                        'trajectory controllers already running; '
                        'start request not repeated: %s'
                        % ', '.join(controller_names)
                    )
            except Exception as exc:
                rospy.logwarn(
                    'trajectory controller pre-start state query failed; '
                    'falling back to switch request: %s',
                    exc,
                )
        try:
            rospy.wait_for_service('/controller_manager/switch_controller', timeout=1.0)
            srv = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
            req = SwitchControllerRequest()
            req.start_controllers = controller_names
            req.stop_controllers = []
            req.strictness = SwitchControllerRequest.BEST_EFFORT
            req.start_asap = True
            req.timeout = 2.0
            res = srv(req)
            if not getattr(res, 'ok', False):
                return False, 'trajectory controller switch request was rejected'
            return self._verify_trajectory_controllers_running(controller_names)
        except Exception as exc:
            return False, 'trajectory controller switch failed: %s' % exc

    def _hold_feedback_after_failed_execution(
        self,
        operation,
        message,
        execution_start_positions=None,
    ):
        """Install a continuous failure hold without removing joint torque."""
        hold_ok, hold_message = (
            self._hold_controller_command_after_failed_execution(
                execution_start_positions,
            )
        )
        enable_message = Bool()
        enable_message.data = False
        try:
            self.demo_pub.publish(enable_message)
            reenable_message = 'positive joint enable re-requested'
            rospy.logwarn(
                '%s failed; %s after the measured failure hold; no disable, '
                'stop, or torque-off command was published',
                operation,
                reenable_message,
            )
        except Exception as exc:
            reenable_message = (
                'positive joint enable re-request failed: %s' % exc
            )
            rospy.logerr('%s failed; %s', operation, reenable_message)
        if hold_ok:
            rospy.logwarn(
                '%s failed; installed a feedback-evidenced controller hold '
                'while keeping controllers and joint torque enabled: %s',
                operation,
                hold_message,
            )
            return (
                '%s; controller failure hold installed: %s; %s'
            ) % (
                message,
                hold_message,
                reenable_message,
            )
        rospy.logerr(
            '%s failed and controller failure hold was not confirmed: %s',
            operation,
            hold_message,
        )
        return (
            '%s; controller failure hold unconfirmed: %s; %s'
        ) % (
            message,
            hold_message,
            reenable_message,
        )

    def _hold_controller_command_after_failed_execution(
        self,
        execution_start_positions,
    ):
        """Choose feedback or desired hold from measured execution response.

        When fresh controller feedback proves that the hardware did not move
        even though the desired trajectory advanced to the configured path
        error boundary, retaining that desired vector leaves a latent target.
        In that one case, synchronize the controller back to measured actual
        positions. If any arm joint moved beyond the hardware quantization
        band, preserve the latest desired vector so partial execution remains
        continuous and does not produce a second step.
        """

        if execution_start_positions is None:
            return self._hold_controller_desired_command()

        cfg = rospy.get_param('/robot', {})
        arm_names = list(self.joint_names[:-1])
        start_positions = list(execution_start_positions)
        state = getattr(self, '_trajectory_controller_state', None)
        if (
            state is None
            or not arm_names
            or len(start_positions) != len(arm_names)
        ):
            return self._hold_controller_desired_command()

        now_sec = float(rospy.get_time())
        max_age_sec = max(
            0.0,
            float(
                cfg.get(
                    'strict_execution_controller_sync_max_feedback_age_sec',
                    0.5,
                )
            ),
        )
        try:
            state_stamp_sec = float(state.header.stamp.to_sec())
            state_age_sec = now_sec - state_stamp_sec
            desired_by_name = dict(
                zip(state.joint_names, state.desired.positions)
            )
            actual_by_name = dict(
                zip(state.joint_names, state.actual.positions)
            )
            desired_positions = [
                float(desired_by_name[name]) for name in arm_names
            ]
            actual_positions = [
                float(actual_by_name[name]) for name in arm_names
            ]
            start_positions = [
                float(value) for value in start_positions
            ]
        except Exception:
            return self._hold_controller_desired_command()
        values = start_positions + desired_positions + actual_positions
        if (
            state_age_sec < 0.0
            or state_age_sec > max_age_sec
            or not all(math.isfinite(value) for value in values)
        ):
            return self._hold_controller_desired_command()

        # The SDK has 4096 counts/revolution. Command and feedback are
        # independently quantized, so two counts are the smallest defensible
        # round-trip motion evidence band.
        actual_delta_limit_rad = max(
            0.0,
            float(
                cfg.get(
                    'strict_execution_no_actuation_actual_delta_rad',
                    4.0 * math.pi / 4096.0,
                )
            ),
        )
        desired_delta_min_rad = max(
            actual_delta_limit_rad,
            float(
                cfg.get(
                    'strict_execution_no_actuation_min_desired_delta_rad',
                    0.12,
                )
            ),
        )
        actual_delta_rad = max(
            abs(actual - start)
            for actual, start in zip(
                actual_positions,
                start_positions,
            )
        )
        desired_delta_rad = max(
            abs(desired - start)
            for desired, start in zip(
                desired_positions,
                start_positions,
            )
        )
        if (
            actual_delta_rad <= actual_delta_limit_rad
            and desired_delta_rad >= desired_delta_min_rad
        ):
            bridge_duration_sec = max(
                0.02,
                float(
                    cfg.get(
                        'strict_execution_controller_sync_bridge_duration_sec',
                        0.02,
                    )
                ),
            )
            hold = self._controller_feedback_hold_trajectory(
                arm_names,
                actual_positions,
                bridge_duration_sec,
            )
            self.trajectory_command_pub.publish(hold)
            return True, (
                'no-actuation feedback hold published '
                'actual_delta=%.6frad <= %.6frad '
                'desired_delta=%.6frad >= %.6frad '
                'age=%.3fs duration=%.3fs'
                % (
                    actual_delta_rad,
                    actual_delta_limit_rad,
                    desired_delta_rad,
                    desired_delta_min_rad,
                    state_age_sec,
                    bridge_duration_sec,
                )
            )

        hold_ok, hold_message = self._hold_controller_desired_command()
        if not hold_ok:
            return hold_ok, hold_message
        return True, (
            '%s; partial/indeterminate execution evidence '
            'actual_delta=%.6frad desired_delta=%.6frad'
            % (
                hold_message,
                actual_delta_rad,
                desired_delta_rad,
            )
        )

    def _hold_controller_desired_command(self):
        """Keep the pre-failure command continuous instead of re-commanding feedback.

        Alicia-D hardware has a measurable command/feedback residual. Publishing
        the current feedback as a new position command therefore creates a
        second physical step. After a trajectory failure, retain the controller's
        latest desired joint vector so the low-level command remains continuous.
        """

        cfg = rospy.get_param('/robot', {})
        arm_names = list(self.joint_names[:-1])
        state = getattr(self, '_trajectory_controller_state', None)
        if state is None or not arm_names:
            return False, 'controller desired state is unavailable'
        now_sec = float(rospy.get_time())
        max_age_sec = max(
            0.0,
            float(
                cfg.get(
                    'strict_execution_controller_sync_max_feedback_age_sec',
                    0.5,
                )
            ),
        )
        try:
            state_stamp_sec = float(state.header.stamp.to_sec())
            state_age_sec = now_sec - state_stamp_sec
            desired_by_name = dict(
                zip(state.joint_names, state.desired.positions)
            )
            desired_positions = [
                float(desired_by_name[name]) for name in arm_names
            ]
        except Exception:
            return False, 'controller desired state is incomplete'
        if state_age_sec < 0.0 or state_age_sec > max_age_sec:
            return False, (
                'controller desired state age %.3fs outside [0, %.3f]s'
                % (state_age_sec, max_age_sec)
            )
        if not all(math.isfinite(value) for value in desired_positions):
            return False, 'controller desired state is not finite'

        bridge_duration_sec = max(
            0.02,
            float(
                cfg.get(
                    'strict_execution_controller_sync_bridge_duration_sec',
                    0.02,
                )
            ),
        )
        hold = self._controller_feedback_hold_trajectory(
            arm_names,
            desired_positions,
            bridge_duration_sec,
        )
        self.trajectory_command_pub.publish(hold)
        return True, (
            'continuous desired-command hold published age=%.3fs duration=%.3fs'
            % (state_age_sec, bridge_duration_sec)
        )

    def _verify_trajectory_controllers_running(self, controller_names):
        if ListControllers is None:
            return True, 'controller_manager list service unavailable; switch request accepted'
        try:
            rospy.wait_for_service('/controller_manager/list_controllers', timeout=1.0)
            srv = rospy.ServiceProxy('/controller_manager/list_controllers', ListControllers)
            res = srv()
            states = {str(controller.name): str(controller.state) for controller in getattr(res, 'controller', [])}
            missing = self._non_running_controllers(states, controller_names)
            if missing:
                return False, 'trajectory controllers not running: %s' % ', '.join(missing)
            return True, 'trajectory controllers running: %s' % ', '.join(controller_names)
        except Exception as exc:
            return False, 'trajectory controller state check failed: %s' % exc

    def _trajectory_controller_state_cb(self, msg):
        self._trajectory_controller_state = msg

    @staticmethod
    def _is_own_frozen_reference_transition(desired, origin, endpoint):
        """Recognize only the collinear zero-end-velocity bridge we just issued.

        This permits waiting, never execution or a nonzero-velocity admission.
        The driver/SDK baseline is independently revalidated on every call.
        """
        if origin is None or len(origin) != len(endpoint):
            return False
        if all(value == 0. for value in desired.velocities + desired.accelerations):
            # A completed spline can be one ULP short of endpoint, giving
            # progress=0.9999999999999998 forever despite a real SDK ACK.
            # This is NOT admission: let the caller enforce stationary q,
            # exact SDK words, fresh source stamps and current ownership.
            return False
        delta = [b-a for a, b in zip(origin, endpoint)]
        pivot = max(range(len(delta)), key=lambda i: abs(delta[i]))
        if abs(delta[pivot]) <= 1e-12:
            return False
        progress = (desired.positions[pivot]-origin[pivot]) / delta[pivot]
        rate = desired.velocities[pivot] / delta[pivot]
        acceleration = desired.accelerations[pivot] / delta[pivot]
        if not (0. <= progress < 1. and rate >= 0.):
            return False
        for observed, expected in (
            (desired.positions, [a+progress*d for a, d in zip(origin, delta)]),
            (desired.velocities, [rate*d for d in delta]),
            (desired.accelerations, [acceleration*d for d in delta]),
        ):
            if any(abs(a-b) > 1e-9 * max(1., abs(a), abs(b))
                   for a, b in zip(observed, expected)):
                return False
        return True

    def _controller_reference_admission_state(self, names, positions, mode, epoch,
                                             require_stationary, controller_after_sec=0.,
                                             transition_origin=None):
        """Revalidate a single pending initial/handoff admission, without writes."""
        if self._manual_control_active():
            raise ObservationPathError('manual control owns the arm')
        if epoch is None or epoch != getattr(self, '_control_reference_epoch_ns', None):
            raise ObservationPathError('positive-enable epoch changed during reference admission')
        if epoch > rospy.Time.from_sec(float(rospy.get_time())).to_nsec():
            raise ObservationPathError('driver command epoch is in the future')
        if rospy.get_param('/bessica_d_hw_interface/publish_gripper_command', False) is not False:
            raise ObservationPathError('reference admission requires the arm-only hardware stream')
        measured = self._fresh_accepted_reference()
        reference = self._command_reference_snapshot()
        if reference is None:
            if (mode != 'initial_positive_enable'
                    or self._fresh_actuation_status() != 'PENDING:POSITIVE_ENABLE_REQUESTED'):
                raise ObservationPathError('initial reference epoch is no longer pending')
            if any(abs(a-b) > .003 for a, b in zip(measured[:-1], positions)):
                raise ObservationPathError('initial measured reference changed before admission')
        elif self._sdk_arm_words(reference.positions) != self._sdk_arm_words(positions):
            raise ObservationPathError('successful SDK reference changed during admission')
        if require_stationary:
            desired = self._stationary_controller_reference(names, require_stationary=False)
            if desired.controller_stamp_sec < controller_after_sec:
                reference = SimpleNamespace(mode='controller_frame_pending')
            elif (mode == 'frozen_sdk_handoff'
                  and reference is not None
                  and reference.mode == 'sdk_handoff_required'
                  and transition_origin is not None
                  and self._sdk_arm_words(desired.positions) != self._sdk_arm_words(positions)
                  and list(desired.positions) == list(transition_origin)
                  and all(value == 0. for value in
                          desired.velocities + desired.accelerations)):
                # A controller can publish its unchanged old hold after
                # publish+duration, before consuming the new command. While
                # the driver still gates all but the frozen SDK word vector,
                # wait within the SAME admission deadline, without resending.
                # This pending state never authorizes trajectory execution.
                reference = SimpleNamespace(mode='controller_frame_pending')
            elif (mode == 'frozen_sdk_handoff'
                  and self._is_own_frozen_reference_transition(
                      desired, transition_origin, positions)):
                # publish+duration is not controller receipt+duration. A
                # source-stamped intermediate frame can legitimately be newer
                # than that wall-clock estimate. Do not declare completion;
                # wait within the SAME deadline without another publication.
                reference = SimpleNamespace(mode='controller_frame_pending')
            elif any(value != 0. for value in desired.velocities + desired.accelerations):
                raise ObservationPathError('controller reference is not stationary')
            elif self._sdk_arm_words(desired.positions) != self._sdk_arm_words(positions):
                raise ObservationPathError('controller has not reached the selected reference')
        if self._manual_control_active() or epoch != getattr(self, '_control_reference_epoch_ns', None):
            raise ObservationPathError('control ownership or epoch changed while reading admission')
        return reference

    def _reacquire_stationary_sdk_reference(self):
        """Establish NEW command provenance with a proven zero-displacement goal.

        Used after gateway reload or a failed handoff's late SDK ACK. Merely
        seeing stationary telemetry does not claim a prior command completed.
        A uniquely acknowledged action must complete, with the original raw
        controller q (not actual and not rounded SDK q) unchanged throughout.
        No cancellation/stop/enable command is issued, including on timeout.
        """
        names = self.joint_names[:-1]
        epoch = getattr(self, '_control_reference_epoch_ns', None)
        cfg = rospy.get_param('/robot', {})
        timeout = float(cfg.get('strict_execution_controller_sync_timeout_sec', 2.))
        duration = float(cfg.get('strict_execution_controller_sync_duration_sec', .30))
        if not (math.isfinite(timeout) and math.isfinite(duration)
                and .05 <= duration < timeout <= 3.):
            return False, 'CONTROLLER_REFERENCE_INVALID: invalid reacquisition deadline'
        deadline = time.monotonic() + timeout
        try:
            baseline = self._validated_controller_sdk_reference(names)
            if getattr(self, '_completed_command_reference', None) is not None:
                raise ObservationPathError('existing completion must not be replaced by reacquisition')
            client = getattr(self, '_reference_action_client', None)
            if client is None:
                client = actionlib.SimpleActionClient(
                    '/alicia_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
                self._reference_action_client = client
            if not client.wait_for_server(rospy.Duration.from_sec(min(.5, timeout))):
                raise ObservationPathError('reference action server unavailable')
            goal = FollowJointTrajectoryGoal()
            goal.trajectory = self._controller_feedback_hold_trajectory(
                names, baseline.positions, duration)
            goal.trajectory.points[0].accelerations = [0.] * len(names)

            def unchanged():
                current = self._validated_controller_sdk_reference(names)
                if epoch != getattr(self, '_control_reference_epoch_ns', None):
                    raise ObservationPathError('epoch changed during stationary reacquisition')
                if any(abs(a-b) > 1e-12 for a, b in zip(
                        current.positions, baseline.positions)):
                    raise ObservationPathError('controller target changed during stationary reacquisition')
                return current

            unchanged()
            if time.monotonic() >= deadline:
                raise ObservationPathError('reference reacquisition deadline exhausted before send')
            sent_at = float(rospy.get_time())
            client.send_goal(goal)
            rospy.loginfo('Reference reacquisition: issued zero-displacement action at %.9f; '
                          'preserving controller q and successful SDK words', sent_at)
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                current = unchanged()
                status = client.get_state()
                if status == GoalStatus.SUCCEEDED:
                    result = client.get_result()
                    if result is None or result.error_code != 0:
                        raise ObservationPathError('reference action success lacks successful result')
                    if min(current.controller_stamp_sec, current.sdk_stamp_sec) <= sent_at:
                        rospy.sleep(.02)
                        continue
                    self._completed_command_reference = {
                        'names': tuple(names), 'positions': tuple(current.positions),
                        'end_sec': float(rospy.get_time()),
                        'kind': 'completed_stationary_sdk_reacquisition_action',
                    }
                    # Require a post-completion same-q/v/a frame through the
                    # normal provider, including its independent time bound.
                    self._controller_reference_for_execution(names)
                    return True, 'stationary SDK reference reacquired by acknowledged zero-displacement action'
                if status not in (GoalStatus.PENDING, GoalStatus.ACTIVE):
                    raise ObservationPathError('reference action failed with status %s' % status)
                rospy.sleep(.02)
            raise ObservationPathError('reference reacquisition action timed out; no completion claimed')
        except Exception as exc:
            self._completed_command_reference = None
            return False, 'CONTROLLER_REFERENCE_INVALID: '+str(exc)

    def _synchronize_trajectory_controller_to_feedback(self):
        self._observation_sync_hold_evidence = None
        admission_epoch = getattr(self, '_control_reference_epoch_ns', None)
        try:
            selected, mode = self._select_controller_sync_reference()
        except Exception as exc:
            return False, 'CONTROLLER_REFERENCE_INVALID: '+str(exc)
        if mode == 'preserved':
            return True, 'controller reference preserved; no feedback rebase or hold published'
        if mode == 'stationary_sdk_reacquisition':
            return self._reacquire_stationary_sdk_reference()
        cfg = rospy.get_param('/robot', {})
        if not bool(cfg.get('strict_execution_controller_sync_enabled', True)):
            return False, 'CONTROLLER_REFERENCE_INVALID: reference admission cannot skip synchronization'

        now_sec = float(rospy.get_time())
        state_time_sec = getattr(
            self.joint_cmd,
            'last_state_time_sec',
            None,
        )
        max_feedback_age_sec = max(
            0.0,
            float(
                cfg.get(
                    'strict_execution_controller_sync_max_feedback_age_sec',
                    0.5,
                )
            ),
        )
        if state_time_sec is None:
            return False, 'trajectory controller sync has no joint feedback'
        feedback_age_sec = now_sec - float(state_time_sec)
        if (
            feedback_age_sec < 0.0
            or feedback_age_sec > max_feedback_age_sec
        ):
            return False, (
                'trajectory controller sync joint feedback age %.3fs '
                'outside [0, %.3f]s'
                % (feedback_age_sec, max_feedback_age_sec)
            )

        arm_names = list(self.joint_names[:-1])
        arm_positions = list(selected)
        if (
            not arm_names
            or len(arm_positions) != len(arm_names)
            or not all(math.isfinite(float(value)) for value in arm_positions)
        ):
            return False, 'trajectory controller sync feedback is incomplete'

        settle_duration_sec = max(
            0.05,
            float(
                cfg.get(
                    'strict_execution_controller_sync_duration_sec',
                    0.30,
                )
            ),
        )
        timeout_sec = max(
            settle_duration_sec,
            float(
                cfg.get(
                    'strict_execution_controller_sync_timeout_sec',
                    2.0,
                )
            ),
        )
        tolerance_rad = max(
            0.0,
            float(
                cfg.get(
                    'strict_execution_controller_sync_tolerance_rad',
                    0.03,
                )
            ),
        )
        bridge_duration_sec = max(
            0.02,
            float(
                cfg.get(
                    'strict_execution_controller_sync_bridge_duration_sec',
                    0.02,
                )
            ),
        )

        hold = self._controller_feedback_hold_trajectory(
            arm_names,
            arm_positions,
            bridge_duration_sec,
        )
        try:
            transition_origin = self._stationary_controller_reference(arm_names).positions
            self._controller_reference_admission_state(
                arm_names, arm_positions, mode, admission_epoch, False)
        except Exception as exc:
            return False, 'CONTROLLER_REFERENCE_INVALID: '+str(exc)
        # A new issued admission supersedes old completion provenance even if
        # this transaction fails before a delayed ACK arrives. A later attempt
        # must establish its own acknowledged, zero-displacement provenance.
        self._completed_command_reference = None
        publish_time_sec = float(rospy.get_time())
        self.trajectory_command_pub.publish(hold)

        deadline = publish_time_sec + timeout_sec
        wall_deadline = time.monotonic() + timeout_sec
        explicit_sent = False
        acknowledgment_after_sec = publish_time_sec
        pending_ack = False
        last_error = float('inf')
        stable_since_sec = None
        stable_duration_sec = 0.0
        while not rospy.is_shutdown():
            loop_time_sec = float(rospy.get_time())
            if loop_time_sec > deadline or time.monotonic() > wall_deadline:
                break
            try:
                reference = self._controller_reference_admission_state(
                    arm_names, arm_positions, mode, admission_epoch,
                    loop_time_sec >= publish_time_sec + bridge_duration_sec,
                    publish_time_sec + bridge_duration_sec,
                    transition_origin=transition_origin)
            except Exception as exc:
                return False, 'CONTROLLER_REFERENCE_INVALID: '+str(exc)
            if reference is not None and reference.mode == 'controller_frame_pending':
                pending_ack = True
                stable_since_sec = None
                stable_duration_sec = 0.
                rospy.sleep(.02)
                continue
            state = getattr(self, '_trajectory_controller_state', None)
            error = self._controller_hold_error_rad(
                state,
                arm_names,
                arm_positions,
                publish_time_sec,
            )
            if error is not None:
                last_error = error
                if error <= tolerance_rad:
                    if stable_since_sec is None:
                        stable_since_sec = loop_time_sec
                    stable_duration_sec = max(
                        0.0,
                        loop_time_sec - stable_since_sec,
                    )
                    if stable_duration_sec >= settle_duration_sec:
                        if reference is None or reference.mode != 'sdk_tracking':
                            if not explicit_sent:
                                # HW publish_only_on_change may swallow an
                                # exactly unchanged controller hold. Send one
                                # arm-only admission request, not a new target.
                                # The driver's dedicated receiver rechecks its
                                # current epoch and wire words under its lock.
                                message = JointState()
                                acknowledgment_after_sec = float(rospy.get_time())
                                message.header.stamp = rospy.Time.from_sec(acknowledgment_after_sec)
                                message.header.frame_id = 'motion_gateway_reference_sync'
                                message.name = list(arm_names)
                                message.position = list(arm_positions)
                                self.joint_cmd.pub.publish(message)
                                explicit_sent = True
                            pending_ack = True
                            rospy.sleep(.02)
                            continue
                        try:
                            admitted = self._validated_controller_sdk_reference(arm_names)
                            if self._sdk_arm_words(admitted.positions) != self._sdk_arm_words(arm_positions):
                                raise ObservationPathError('admitted reference changed during handoff')
                            if (admitted.sdk_stamp_sec < acknowledgment_after_sec
                                    or admitted.controller_stamp_sec < publish_time_sec + bridge_duration_sec):
                                pending_ack = True
                                rospy.sleep(.02)
                                continue
                        except Exception as exc:
                            return False, 'CONTROLLER_REFERENCE_INVALID: '+str(exc)
                        self._completed_command_reference = {
                            'names': tuple(arm_names), 'positions': tuple(admitted.positions),
                            'end_sec': publish_time_sec + bridge_duration_sec,
                            'kind': 'completed_'+mode,
                        }
                        self._observation_sync_hold_evidence = {
                            'names': tuple(arm_names), 'positions': tuple(arm_positions),
                            'end_sec': publish_time_sec + bridge_duration_sec,
                            'kind': mode,
                        }
                        return True, (
                            'controller feedback hold stable '
                            'max_error=%.6frad <= %.6frad '
                            'stable_for=%.3fs feedback_age=%.3fs'
                            % (
                                error,
                                tolerance_rad,
                                stable_duration_sec,
                                feedback_age_sec,
                            )
                        )
                else:
                    stable_since_sec = None
                    stable_duration_sec = 0.0
            rospy.sleep(0.02)
        if pending_ack:
            return False, 'CONTROLLER_REFERENCE_INVALID: reference admission acknowledgment timed out'
        return False, (
            'trajectory controller feedback hold was not stable within %.3fs '
            '(last_error=%s stable_for=%.3fs)'
            % (
                timeout_sec,
                (
                    'unavailable'
                    if not math.isfinite(last_error)
                    else '%.6frad' % last_error
                ),
                stable_duration_sec,
            )
        )

    def _select_controller_sync_reference(self):
        """Separate initial admission / gated handoff from auto continuation.

        Existing tracking references are never rewritten from actual. A
        manual handoff may synchronize ros_control to the already-transmitted
        baseline only while the driver rejects every other SDK word vector.
        """
        if self._manual_control_active():
            raise ObservationPathError('manual control owns the arm')
        measured = self._fresh_accepted_reference()
        reference = self._command_reference_snapshot()
        if reference is not None:
            if reference.mode == 'sdk_tracking':
                if getattr(self, '_completed_command_reference', None) is None:
                    desired = self._validated_controller_sdk_reference(self.joint_names[:-1])
                    return desired.positions, 'stationary_sdk_reacquisition'
                desired = self._controller_reference_for_execution(self.joint_names[:-1])
                return desired.positions, 'preserved'
            # This handoff relies on the arm-only ros_control stream. The
            # driver's missing gripper field retains the frozen successful
            # gripper word; it must not be overwritten by another controller.
            if rospy.get_param('/bessica_d_hw_interface/publish_gripper_command', False) is not False:
                raise ObservationPathError('handoff requires the arm-only hardware command stream')
            self._stationary_controller_reference(self.joint_names[:-1])
            return list(reference.positions), 'frozen_sdk_handoff'
        if self._fresh_actuation_status() != 'PENDING:POSITIVE_ENABLE_REQUESTED':
            raise ObservationPathError('missing SDK reference is not a proven initial positive-enable epoch')
        if getattr(self, '_control_reference_epoch_ns', None) is None:
            raise ObservationPathError('initial admission requires driver command epoch telemetry')
        return measured[:-1], 'initial_positive_enable'

    @staticmethod
    def _controller_feedback_hold_trajectory(
        joint_names,
        joint_positions,
        bridge_duration_sec,
    ):
        hold = JointTrajectory()
        hold.header.stamp = rospy.Time(0)
        hold.joint_names = list(joint_names)
        point = JointTrajectoryPoint()
        point.positions = list(joint_positions)
        point.velocities = [0.0] * len(joint_names)
        point.time_from_start = rospy.Duration.from_sec(bridge_duration_sec)
        hold.points = [point]
        return hold

    @staticmethod
    def _controller_hold_error_rad(
        state,
        joint_names,
        hold_positions,
        publish_time_sec,
    ):
        if state is None:
            return None
        try:
            state_stamp_sec = float(state.header.stamp.to_sec())
            if state_stamp_sec + 1e-6 < float(publish_time_sec):
                return None
            desired = dict(
                zip(state.joint_names, state.desired.positions)
            )
            actual = dict(
                zip(state.joint_names, state.actual.positions)
            )
            errors = []
            for name, hold_position in zip(joint_names, hold_positions):
                if name not in desired or name not in actual:
                    return None
                errors.append(
                    abs(float(desired[name]) - float(hold_position))
                )
                errors.append(
                    abs(float(actual[name]) - float(hold_position))
                )
        except Exception:
            return None
        if not errors or not all(math.isfinite(value) for value in errors):
            return None
        return max(errors)

    @staticmethod
    def _non_running_controllers(states, controller_names):
        missing = []
        for name in controller_names:
            state = states.get(str(name), 'missing')
            if state != 'running':
                missing.append('%s=%s' % (name, state))
        return missing

    def _gripper_arm_positions_for_command(self):
        gcfg = rospy.get_param('/gripper', {})
        if not bool(gcfg.get('hold_arm_during_gripper_commands', True)):
            return None

        # Closing/opening the jaws must not add a second arm step either.
        # Read successful wire words on every request, never timeout-rebase
        # a measured arm vector into a new target.
        self._fresh_accepted_reference()
        reference = self._command_reference_snapshot()
        if reference is None or reference.mode != 'sdk_tracking':
            raise ObservationPathError('gripper command requires an admitted successful arm reference')
        return list(reference.positions)

    def _current_arm_positions_snapshot(self):
        positions = list(getattr(self.joint_cmd, 'last_positions', []) or [])
        gripper_index = int(getattr(self.gripper, 'gripper_index', max(0, len(self.joint_names) - 1)))
        if len(positions) < gripper_index:
            positions += [0.0] * (gripper_index - len(positions))
        return positions[:gripper_index]

    def _log_pose_request(self, req, operation='move_to_pose'):
        try:
            target = req.target
            frame_id = getattr(getattr(target, 'header', None), 'frame_id', '') or '<empty>'
            pose = getattr(target, 'pose', target)
            p = pose.position
            q = pose.orientation
            rospy.loginfo(
                '%s request execute=%s frame=%s xyz=(%.3f, %.3f, %.3f) q=(%.3f, %.3f, %.3f, %.3f)',
                operation,
                bool(req.execute),
                frame_id,
                float(p.x),
                float(p.y),
                float(p.z),
                float(q.x),
                float(q.y),
                float(q.z),
                float(q.w),
            )
        except Exception as exc:
            rospy.logwarn('%s request log failed: %s', operation, exc)

if __name__ == '__main__':
    rospy.init_node('motion_gateway_node')
    MotionGateway()
    rospy.spin()
