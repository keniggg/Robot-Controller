#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState, Image
from alicia_flexible_grasp_supervisor.msg import TactileState

def wait_topic(topic, typ, timeout=3.0):
    try:
        rospy.wait_for_message(topic, typ, timeout=timeout)
        rospy.loginfo('[OK] %s', topic)
    except Exception:
        rospy.logwarn('[MISS] %s', topic)
if __name__=='__main__':
    rospy.init_node('system_bringup_check')
    wait_topic('/joint_states', JointState)
    wait_topic('/tactile/state', TactileState)
    wait_topic('/supervisor/camera/color/image_raw', Image)
