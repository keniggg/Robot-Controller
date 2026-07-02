#!/usr/bin/env python3
import rospy
from alicia_flexible_grasp_supervisor.msg import TactileState
if __name__ == '__main__':
    rospy.init_node('test_tactile_read')
    msg = rospy.wait_for_message('/tactile/state', TactileState, timeout=5)
    print('total_force_mn=', msg.total_grip_force_mn)
