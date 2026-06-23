#!/usr/bin/env python3
import rospy


if __name__ == '__main__':
    rospy.init_node('system_bringup_check')
    required = [
        '/arm_joint_state',
        '/tactile/state',
        '/perception/object_pose_base',
        '/grasp/state',
    ]
    rospy.sleep(2.0)
    topics = dict(rospy.get_published_topics())
    for t in required:
        rospy.loginfo('%s : %s', t, 'OK' if t in topics else 'MISSING')
