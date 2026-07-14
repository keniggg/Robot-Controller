#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
from alicia_flexible_grasp.robot.joint_commander import JointCommander
from alicia_flexible_grasp.robot.gripper_commander import GripperCommander
from alicia_flexible_grasp.robot.moveit_planner import MoveItPlanner
from alicia_flexible_grasp.robot.cartesian_controller import CartesianJogger
from alicia_flexible_grasp_supervisor.srv import SetJointCommand, SetJointCommandResponse, SetFloat, SetFloatResponse, SetTargetPose, SetTargetPoseResponse, CartesianJog, CartesianJogResponse, TriggerZero, TriggerZeroResponse

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
        self.zero_pub = rospy.Publisher(cfg.get('zero_calibrate_topic','/zero_calibrate'), Bool, queue_size=1)
        self.demo_pub = rospy.Publisher(cfg.get('demonstration_topic','/demonstration'), Bool, queue_size=1)
        self.manipulator_group = rospy.get_param('~manipulator_group','alicia')
        self.gripper_group = rospy.get_param('~gripper_group','hand')
        self.velocity = rospy.get_param('~velocity',0.3)
        self.planner = None
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
        rospy.Service('/supervisor/cartesian_jog', CartesianJog, self.handle_jog)
        rospy.Service('/supervisor/trigger_zero', TriggerZero, self.handle_zero)
        rospy.loginfo('MotionGateway ready: commands -> %s', cfg.get('joint_command_topic','/joint_commands'))

    def handle_joints(self, req):
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
        planner = self._ensure_planner()
        if planner is None:
            msg = self._moveit_not_ready_message()
            rospy.logwarn('check_pose_strict result success=False message=%s', msg)
            return SetTargetPoseResponse(False, msg)
        ok, msg = planner.move_to_pose(req.target, execute=False, allow_fallbacks=False)
        if ok:
            rospy.loginfo('check_pose_strict result success=True message=%s', msg)
        else:
            rospy.logwarn('check_pose_strict result success=False message=%s', msg)
        return SetTargetPoseResponse(ok, msg)

    def handle_pose_linear(self, req):
        self._log_pose_request(req, operation='move_to_pose_linear')
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
        planner = self._ensure_planner()
        if planner is None:
            return CartesianJogResponse(False, self._moveit_not_ready_message())
        self.jogger.planner = planner
        ok,msg = self.jogger.jog(req.dx, req.dy, req.dz, req.droll, req.dpitch, req.dyaw, execute=req.execute)
        return CartesianJogResponse(ok, msg)

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
        try:
            rospy.wait_for_service('/controller_manager/switch_controller', timeout=1.0)
            srv = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
            req = SwitchControllerRequest()
            controller_names = list(self.trajectory_controller_names)
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
