#!/usr/bin/env python3
import importlib.util
import pathlib
import sys
import types
import unittest
import threading
from copy import deepcopy


ROOT = pathlib.Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / 'src'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

SCRIPT = ROOT / 'scripts' / 'motion_gateway_node.py'
spec = importlib.util.spec_from_file_location('motion_gateway_node', str(SCRIPT))
motion_gateway_node = importlib.util.module_from_spec(spec)
# Use the generated msg/srv types; only the hardware-facing instance is faked.
spec.loader.exec_module(motion_gateway_node)


class FakeGripper:
    def __init__(self):
        self.calls = []

    def set_position(self, value, arm_positions=None):
        snapshot = None if arm_positions is None else list(arm_positions)
        self.calls.append((float(value), snapshot))
        return True


class MotionGatewayGripperHoldTest(unittest.TestCase):
    def test_gripper_burst_preserves_successful_wire_reference_not_feedback(self):
        node = motion_gateway_node.MotionGateway.__new__(motion_gateway_node.MotionGateway)
        node.joint_names = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6', 'right_finger']
        node.joint_cmd = types.SimpleNamespace(last_positions=[1, 2, 3, 4, 5, 6, 0.0])
        node.gripper = FakeGripper()
        node._gripper_arm_hold_positions = None
        node._last_gripper_command_time = 0.0
        node._fresh_accepted_reference = lambda: node.joint_cmd.last_positions
        from sensor_msgs.msg import JointState
        positions = [.1, .2, .3, .4, .5, .6]
        node._command_reference_lock = threading.RLock()
        node._control_reference_epoch_ns = 500000000
        node._sdk_command_feedback = JointState()
        node._sdk_command_feedback.header.stamp = motion_gateway_node.rospy.Time.from_sec(1.)
        node._sdk_command_feedback.header.frame_id = 'sdk_transmitted'
        node._sdk_command_feedback.name = node.joint_names[:-1]
        node._sdk_command_feedback.position = positions
        node._driver_control_reference = JointState()
        node._driver_control_reference.header.stamp = node._sdk_command_feedback.header.stamp
        node._driver_control_reference.header.frame_id = 'sdk_tracking'
        node._driver_control_reference.name = node.joint_names
        node._driver_control_reference.position = positions + [.05]
        node._observation_joint_feedback = deepcopy(node._driver_control_reference)
        node._observation_joint_feedback.header.frame_id = 'sdk_measured'
        node._actuation_status = 'CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE'
        node._actuation_received_sec = 1.
        node._command_reference_snapshot = lambda: types.SimpleNamespace(
            mode='sdk_tracking', positions=positions, opening=.05)

        now = [1.]
        original_get_time = motion_gateway_node.rospy.get_time
        original_get_param = motion_gateway_node.rospy.get_param
        motion_gateway_node.rospy.get_time = lambda: now[0]
        motion_gateway_node.rospy.get_param = lambda name, default=None: {
            '/gripper': {
                'hold_arm_during_gripper_commands': True,
                'arm_hold_timeout_sec': 0.5,
            }
        }.get(name, default)
        try:
            node.handle_gripper(types.SimpleNamespace(value=0.01))
            node.joint_cmd.last_positions = [10, 20, 30, 40, 50, 60, 0.01]
            now[0] = 1.2
            node.handle_gripper(types.SimpleNamespace(value=0.02))
            now[0] = 2.
            for message in (node._sdk_command_feedback, node._driver_control_reference,
                            node._observation_joint_feedback):
                message.header.stamp = motion_gateway_node.rospy.Time.from_sec(now[0])
            node.handle_gripper(types.SimpleNamespace(value=0.03))
        finally:
            motion_gateway_node.rospy.get_time = original_get_time
            motion_gateway_node.rospy.get_param = original_get_param

        self.assertEqual(node.gripper.calls[0], (0.01, positions))
        self.assertEqual(node.gripper.calls[1], (0.02, positions))
        self.assertEqual(node.gripper.calls[2], (0.03, positions))


if __name__ == '__main__':
    unittest.main()
