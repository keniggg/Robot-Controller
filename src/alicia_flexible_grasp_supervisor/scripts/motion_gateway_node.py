#!/usr/bin/env python3
import math
import threading

import rospy
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from alicia_flexible_grasp.robot.joint_commander import JointCommander
from alicia_flexible_grasp.robot.gripper_commander import GripperCommander
from alicia_flexible_grasp.robot.moveit_planner import MoveItPlanner
from alicia_flexible_grasp.robot.cartesian_controller import CartesianJogger
from alicia_flexible_grasp_supervisor.srv import (
    CartesianJog,
    CartesianJogResponse,
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

class MotionGateway:
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
        self._last_planner_error = 'MoveIt not initialized'
        self._planner_retry_period_sec = float(rospy.get_param('~planner_retry_period_sec', 2.0))
        self._last_planner_attempt = 0.0
        self._gripper_arm_hold_positions = None
        self._last_gripper_command_time = 0.0
        self.jogger = CartesianJogger(self.planner)
        rospy.Service('/supervisor/move_to_joints', SetJointCommand, self.handle_joints)
        rospy.Service('/supervisor/set_gripper', SetFloat, self.handle_gripper)
        rospy.Service('/supervisor/move_to_pose', SetTargetPose, self.handle_pose)
        rospy.Service('/supervisor/move_to_pose_linear', SetTargetPose, self.handle_pose_linear)
        rospy.Service('/supervisor/check_pose_strict', SetTargetPose, self.handle_pose_strict)
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
        rospy.Service('/supervisor/trigger_zero', TriggerZero, self.handle_zero)
        rospy.loginfo('MotionGateway ready: commands -> %s', cfg.get('joint_command_topic','/joint_commands'))

    def handle_joints(self, req):
        with self._planner_operation_lock():
            planner = self._ensure_planner()
            if planner is None:
                return SetJointCommandResponse(False, self._moveit_not_ready_message())
            ok,msg = planner.move_to_joints(req.positions, execute=req.execute)
        return SetJointCommandResponse(ok, msg)

    def handle_gripper(self, req):
        arm_positions = self._gripper_arm_positions_for_command()
        self.gripper.set_position(req.value, arm_positions=arm_positions)
        return SetFloatResponse(True, 'gripper command published')

    def handle_pose(self, req):
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

    def handle_pose_strict_execute(self, req):
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
            if not ok:
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
            if not ok:
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
        if hold_ok:
            rospy.logwarn(
                '%s failed; installed a feedback-evidenced controller hold '
                'while keeping controllers and joint torque enabled: %s',
                operation,
                hold_message,
            )
            return '%s; controller failure hold installed: %s' % (
                message,
                hold_message,
            )
        rospy.logerr(
            '%s failed and controller failure hold was not confirmed: %s',
            operation,
            hold_message,
        )
        return '%s; controller failure hold unconfirmed: %s' % (
            message,
            hold_message,
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

    def _synchronize_trajectory_controller_to_feedback(self):
        cfg = rospy.get_param('/robot', {})
        if not bool(cfg.get('strict_execution_controller_sync_enabled', True)):
            return True, 'controller feedback sync disabled'

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
        arm_positions = self._current_arm_positions_snapshot()
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
        publish_time_sec = float(rospy.get_time())
        self.trajectory_command_pub.publish(hold)

        deadline = publish_time_sec + timeout_sec
        last_error = float('inf')
        stable_since_sec = None
        stable_duration_sec = 0.0
        while not rospy.is_shutdown():
            loop_time_sec = float(rospy.get_time())
            if loop_time_sec > deadline:
                break
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

        now = rospy.get_time() if not rospy.is_shutdown() else 0.0
        timeout = max(0.0, float(gcfg.get('arm_hold_timeout_sec', 0.5)))
        last_time = float(getattr(self, '_last_gripper_command_time', 0.0))
        hold_positions = getattr(self, '_gripper_arm_hold_positions', None)
        hold_expired = hold_positions is None or (timeout > 0.0 and now - last_time > timeout)
        if hold_expired:
            hold_positions = self._current_arm_positions_snapshot()
            self._gripper_arm_hold_positions = hold_positions
        self._last_gripper_command_time = now
        return hold_positions

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
