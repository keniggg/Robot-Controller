#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from alicia_duo_driver.msg import ArmJointState
from alicia_flexible_grasp_supervisor.srv import SetTargetPose, SetTargetPoseResponse
from alicia_flexible_grasp.robot.moveit_planner import MoveItPlanner


class MotionGatewayNode:
    def __init__(self):
        self.arm_pub = rospy.Publisher('/arm_joint_command', ArmJointState, queue_size=10)
        self.gripper_pub = rospy.Publisher('/gripper_control', Float32, queue_size=10)
        self.planner = MoveItPlanner(rospy.get_param('~move_group', 'alicia'))
        rospy.Service('/motion/set_target_pose', SetTargetPose, self.handle_target_pose)

    def handle_target_pose(self, req):
        ok, msg = self.planner.move_to_pose(req.pose, execute=req.execute)
        return SetTargetPoseResponse(ok=ok, message=msg)


if __name__ == '__main__':
    rospy.init_node('motion_gateway_node')
    MotionGatewayNode()
    rospy.spin()
