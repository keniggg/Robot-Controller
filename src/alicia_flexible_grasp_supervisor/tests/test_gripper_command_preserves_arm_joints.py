#!/usr/bin/env python3
import pathlib
import sys
import types
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / 'src'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from alicia_flexible_grasp.robot import joint_commander as joint_commander_module
from alicia_flexible_grasp.robot.gripper_commander import GripperCommander


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class FakeRospy:
    def __init__(self):
        self.publisher = FakePublisher()
        self.subscriptions = []
        self.Time = types.SimpleNamespace(now=lambda: 0.0)

    def Publisher(self, topic, msg_type, queue_size=10):
        return self.publisher

    def Subscriber(self, topic, msg_type, callback, queue_size=10):
        self.subscriptions.append((topic, msg_type, callback, queue_size))
        return types.SimpleNamespace(unregister=lambda: None)


class GripperCommandPreservesArmJointsTest(unittest.TestCase):
    def test_gripper_command_uses_latest_joint_state_for_arm_joints(self):
        fake_rospy = FakeRospy()
        original_rospy = joint_commander_module.rospy
        joint_commander_module.rospy = fake_rospy
        try:
            commander = joint_commander_module.JointCommander(
                '/joint_commands',
                ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6', 'right_finger'],
            )
            current = [-1.8, -0.26, 0.58, -0.08, -0.34, 0.09, 0.01]
            state_msg = types.SimpleNamespace(
                name=['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6', 'right_finger'],
                position=current,
            )

            commander.joint_state_cb(state_msg)
            GripperCommander(commander, gripper_index=6, min_m=0.0, max_m=0.05).set_position(0.04)

            self.assertEqual(list(fake_rospy.publisher.messages[-1].position[:6]), current[:6])
            self.assertAlmostEqual(fake_rospy.publisher.messages[-1].position[6], 0.04)
        finally:
            joint_commander_module.rospy = original_rospy

    def test_gripper_command_can_hold_arm_snapshot_while_closing(self):
        fake_rospy = FakeRospy()
        original_rospy = joint_commander_module.rospy
        joint_commander_module.rospy = fake_rospy
        try:
            commander = joint_commander_module.JointCommander(
                '/joint_commands',
                ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6', 'right_finger'],
            )
            arm_snapshot = [-1.8, -0.26, 0.58, -0.08, -0.34, 0.09]
            moving_arm_state = [-1.9, -0.40, 0.80, -0.12, -0.50, 0.20, 0.01]
            state_msg = types.SimpleNamespace(
                name=['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6', 'right_finger'],
                position=moving_arm_state,
            )

            commander.joint_state_cb(state_msg)
            gripper = GripperCommander(commander, gripper_index=6, min_m=0.0, max_m=0.05)
            gripper.set_position(0.02, arm_positions=arm_snapshot)
            gripper.set_position(0.04, arm_positions=arm_snapshot)

            self.assertEqual(list(fake_rospy.publisher.messages[-1].position[:6]), arm_snapshot)
            self.assertAlmostEqual(fake_rospy.publisher.messages[-1].position[6], 0.04)
        finally:
            joint_commander_module.rospy = original_rospy


if __name__ == '__main__':
    unittest.main()
