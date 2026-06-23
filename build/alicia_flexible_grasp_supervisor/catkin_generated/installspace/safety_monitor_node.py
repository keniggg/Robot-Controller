#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool, String
from alicia_flexible_grasp_supervisor.msg import TactileState, SafetyState


class SafetyMonitorNode:
    def __init__(self):
        self.max_force = rospy.get_param('~max_force_mn', 4500.0)
        self.last_tactile = None
        self.pub = rospy.Publisher('/safety/status', SafetyState, queue_size=10)
        self.pub_estop = rospy.Publisher('/safety/emergency_stop', Bool, queue_size=10)
        self.pub_warning = rospy.Publisher('/safety/warning', String, queue_size=10)
        rospy.Subscriber('/tactile/state', TactileState, self.tactile_cb, queue_size=10)
        rospy.Timer(rospy.Duration(0.1), self.tick)

    def tactile_cb(self, msg):
        self.last_tactile = msg

    def tick(self, _):
        msg = SafetyState()
        msg.header.stamp = rospy.Time.now()
        msg.ok = True
        msg.level = 'OK'
        msg.message = 'normal'
        if self.last_tactile and self.last_tactile.total_grip_force > self.max_force:
            msg.ok = False
            msg.force_over_limit = True
            msg.level = 'ERROR'
            msg.message = 'force over limit'
            self.pub_estop.publish(Bool(data=True))
            self.pub_warning.publish(String(data=msg.message))
        self.pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('safety_monitor_node')
    SafetyMonitorNode()
    rospy.spin()
