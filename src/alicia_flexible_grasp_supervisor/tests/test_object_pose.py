#!/usr/bin/env python3
import rospy
from alicia_flexible_grasp_supervisor.msg import ObjectPose
if __name__ == '__main__':
    rospy.init_node('test_object_pose')
    msg = rospy.wait_for_message('/perception/object', ObjectPose, timeout=10)
    print('detected=', msg.detected, 'depth=', msg.depth_m)
