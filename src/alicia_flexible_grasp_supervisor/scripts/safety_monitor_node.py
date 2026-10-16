#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState, Image
from alicia_flexible_grasp_supervisor.msg import TactileState, SafetyState

class SafetyMonitor:
    def __init__(self):
        cfg = rospy.get_param('/safety', {})
        self.max_force = float(cfg.get('max_total_force_mn',4500))
        self.last_joint = rospy.Time(0); self.last_tactile = rospy.Time(0); self.last_camera = rospy.Time(0)
        self.force = 0.0
        self.pub = rospy.Publisher('/safety/status', SafetyState, queue_size=10)
        rospy.Subscriber('/joint_states', JointState, lambda m: setattr(self,'last_joint',rospy.Time.now()), queue_size=1)
        rospy.Subscriber('/tactile/state', TactileState, self.tactile_cb, queue_size=1)
        rospy.Subscriber('/supervisor/camera/color/image_raw', Image, lambda m: setattr(self,'last_camera',rospy.Time.now()), queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.2), self.tick)
    def tactile_cb(self,msg):
        self.last_tactile = rospy.Time.now(); self.force = msg.total_grip_force_mn
    def tick(self,event):
        now=rospy.Time.now(); st=SafetyState(); st.header.stamp=now; st.ok=True; st.level='OK'; st.message='normal'
        if self.force > self.max_force:
            st.ok=False; st.emergency_stop=True; st.level='ERROR'; st.message='force over limit'
        self.pub.publish(st)
if __name__=='__main__':
    rospy.init_node('safety_monitor_node'); SafetyMonitor(); rospy.spin()
