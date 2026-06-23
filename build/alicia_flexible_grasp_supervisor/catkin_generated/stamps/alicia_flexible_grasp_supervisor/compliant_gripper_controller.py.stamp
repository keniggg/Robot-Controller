#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from alicia_flexible_grasp_supervisor.msg import TactileState
from alicia_flexible_grasp.grasp.compliant_grasp import CompliantGraspController
from alicia_flexible_grasp.robot.gripper_commander import GripperCommander


class CompliantGripperNode:
    def __init__(self):
        self.force = 0.0
        rospy.Subscriber('/tactile/state', TactileState, self.tactile_cb, queue_size=10)
        self.gripper = GripperCommander('/gripper_control')
        self.ctrl = CompliantGraspController(
            self.gripper, lambda: self.force,
            open_position=rospy.get_param('~open_position', 0.12),
            contact_threshold=rospy.get_param('~contact_threshold_mn', 200.0),
            target_force=rospy.get_param('~target_force_mn', 1500.0),
            max_force=rospy.get_param('~max_force_mn', 4000.0),
            close_step_fast=rospy.get_param('~close_step_fast', 0.003),
            close_step_slow=rospy.get_param('~close_step_slow', 0.001),
        )

    def tactile_cb(self, msg):
        self.force = msg.total_grip_force


if __name__ == '__main__':
    rospy.init_node('compliant_gripper_controller')
    CompliantGripperNode()
    rospy.spin()
