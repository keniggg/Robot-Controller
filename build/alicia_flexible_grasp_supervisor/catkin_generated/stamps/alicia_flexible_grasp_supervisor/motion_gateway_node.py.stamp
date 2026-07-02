#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
from alicia_flexible_grasp.robot.joint_commander import JointCommander
from alicia_flexible_grasp.robot.gripper_commander import GripperCommander
from alicia_flexible_grasp.robot.moveit_planner import MoveItPlanner
from alicia_flexible_grasp.robot.cartesian_controller import CartesianJogger
from alicia_flexible_grasp_supervisor.srv import SetJointCommand, SetJointCommandResponse, SetFloat, SetFloatResponse, SetTargetPose, SetTargetPoseResponse, CartesianJog, CartesianJogResponse, TriggerZero, TriggerZeroResponse

try:
    from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
except Exception:
    SwitchController = None
    SwitchControllerRequest = None

class MotionGateway:
    def __init__(self):
        cfg = rospy.get_param('/robot', {})
        self.joint_names = cfg.get('joint_names', ['Joint1','Joint2','Joint3','Joint4','Joint5','Joint6','right_finger'])
        self.trajectory_controller_names = cfg.get('trajectory_controller_names', ['alicia_controller', 'hand_controller'])
        self.joint_cmd = JointCommander(cfg.get('joint_command_topic','/joint_commands'), self.joint_names)
        self.gripper = GripperCommander(self.joint_cmd, len(self.joint_names)-1, cfg.get('gripper_min_m',0.0), cfg.get('gripper_max_m',0.05))
        self.zero_pub = rospy.Publisher(cfg.get('zero_calibrate_topic','/zero_calibrate'), Bool, queue_size=1)
        self.demo_pub = rospy.Publisher(cfg.get('demonstration_topic','/demonstration'), Bool, queue_size=1)
        self.planner = MoveItPlanner(rospy.get_param('~manipulator_group','alicia'), rospy.get_param('~gripper_group','hand'), rospy.get_param('~velocity',0.3))
        self.jogger = CartesianJogger(self.planner)
        rospy.Service('/supervisor/move_to_joints', SetJointCommand, self.handle_joints)
        rospy.Service('/supervisor/set_gripper', SetFloat, self.handle_gripper)
        rospy.Service('/supervisor/move_to_pose', SetTargetPose, self.handle_pose)
        rospy.Service('/supervisor/cartesian_jog', CartesianJog, self.handle_jog)
        rospy.Service('/supervisor/trigger_zero', TriggerZero, self.handle_zero)
        rospy.loginfo('MotionGateway ready: commands -> %s', cfg.get('joint_command_topic','/joint_commands'))

    def handle_joints(self, req):
        ok,msg = self.planner.move_to_joints(req.positions, execute=req.execute)
        return SetJointCommandResponse(ok, msg)

    def handle_gripper(self, req):
        self.gripper.set_position(req.value)
        return SetFloatResponse(True, 'gripper command published')

    def handle_pose(self, req):
        self._log_pose_request(req)
        if req.execute:
            controllers_ok, controller_msg = self._ensure_trajectory_controllers_started()
            if not controllers_ok:
                msg = 'execute blocked: %s' % controller_msg
                rospy.logwarn('move_to_pose result success=False message=%s', msg)
                return SetTargetPoseResponse(False, msg)
        ok,msg = self.planner.move_to_pose(req.target, execute=req.execute)
        if ok:
            rospy.loginfo('move_to_pose result success=True message=%s', msg)
        else:
            rospy.logwarn('move_to_pose result success=False message=%s', msg)
        return SetTargetPoseResponse(ok, msg)

    def handle_jog(self, req):
        ok,msg = self.jogger.jog(req.dx, req.dy, req.dz, req.droll, req.dpitch, req.dyaw, execute=req.execute)
        return CartesianJogResponse(ok, msg)

    def handle_zero(self, req):
        if req.trigger:
            self.zero_pub.publish(Bool(True))
        return TriggerZeroResponse(True, 'zero command sent')

    def _ensure_trajectory_controllers_started(self):
        if SwitchController is None or SwitchControllerRequest is None:
            return True, 'controller_manager_msgs unavailable; skipped controller switch'
        try:
            rospy.wait_for_service('/controller_manager/switch_controller', timeout=1.0)
            srv = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
            req = SwitchControllerRequest()
            req.start_controllers = list(self.trajectory_controller_names)
            req.stop_controllers = []
            req.strictness = SwitchControllerRequest.BEST_EFFORT
            req.start_asap = True
            req.timeout = 2.0
            res = srv(req)
            if getattr(res, 'ok', False):
                return True, 'trajectory controllers started'
            return False, 'trajectory controllers did not start'
        except Exception as exc:
            return False, 'trajectory controller switch failed: %s' % exc

    def _log_pose_request(self, req):
        try:
            target = req.target
            frame_id = getattr(getattr(target, 'header', None), 'frame_id', '') or '<empty>'
            pose = getattr(target, 'pose', target)
            p = pose.position
            q = pose.orientation
            rospy.loginfo(
                'move_to_pose request execute=%s frame=%s xyz=(%.3f, %.3f, %.3f) q=(%.3f, %.3f, %.3f, %.3f)',
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
            rospy.logwarn('move_to_pose request log failed: %s', exc)

if __name__ == '__main__':
    rospy.init_node('motion_gateway_node')
    MotionGateway()
    rospy.spin()
