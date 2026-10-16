import rospy
from sensor_msgs.msg import JointState

class JointCommander:
    def __init__(self, topic='/joint_commands', joint_names=None, state_topic='/joint_states'):
        self.topic = topic
        self.joint_names = joint_names or ['Joint1','Joint2','Joint3','Joint4','Joint5','Joint6','right_finger']
        self._name_to_index = {name: index for index, name in enumerate(self.joint_names)}
        self.pub = rospy.Publisher(topic, JointState, queue_size=10)
        self.last_positions = [0.0] * len(self.joint_names)
        self.state_sub = None
        if state_topic:
            try:
                self.state_sub = rospy.Subscriber(state_topic, JointState, self.joint_state_cb, queue_size=1)
            except Exception:
                self.state_sub = None

    def joint_state_cb(self, msg):
        positions = self.last_positions[:]
        changed = False
        for name, position in zip(getattr(msg, 'name', []), getattr(msg, 'position', [])):
            index = self._name_to_index.get(name)
            if index is None or index >= len(positions):
                continue
            positions[index] = position
            changed = True
        if changed:
            self.last_positions = positions

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
