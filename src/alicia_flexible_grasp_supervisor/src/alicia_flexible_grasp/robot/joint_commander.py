import rospy
from sensor_msgs.msg import JointState

class JointCommander:
    def __init__(self, topic='/joint_commands', joint_names=None):
        self.topic = topic
        self.joint_names = joint_names or ['Joint1','Joint2','Joint3','Joint4','Joint5','Joint6','right_finger']
        self.pub = rospy.Publisher(topic, JointState, queue_size=10)
        self.last_positions = [0.0] * len(self.joint_names)

    def publish(self, positions):
        if len(positions) < len(self.joint_names):
            positions = list(positions) + self.last_positions[len(positions):]
        positions = list(positions[:len(self.joint_names)])
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = self.joint_names
        msg.position = positions
        self.last_positions = positions
        self.pub.publish(msg)
        return True
