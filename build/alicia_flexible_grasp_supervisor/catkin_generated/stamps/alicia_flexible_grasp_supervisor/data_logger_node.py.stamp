#!/usr/bin/env python3
import rospy
from alicia_flexible_grasp_supervisor.msg import TactileState, GraspState

class DataLogger:
    def __init__(self):
        rospy.Subscriber('/tactile/state', TactileState, self.tactile_cb, queue_size=10)
        rospy.Subscriber('/grasp/state', GraspState, self.grasp_cb, queue_size=10)
    def tactile_cb(self,msg):
        pass
    def grasp_cb(self,msg):
        rospy.loginfo_throttle(2.0, 'grasp state: %s %s', msg.state, msg.message)
if __name__=='__main__':
    rospy.init_node('data_logger_node'); DataLogger(); rospy.spin()
