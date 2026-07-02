#!/usr/bin/env python3
import importlib.util
import pathlib
import sys
import types
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / 'src'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

_MISSING = object()
_original_supervisor_pkg = sys.modules.get('alicia_flexible_grasp_supervisor', _MISSING)
_original_supervisor_srv = sys.modules.get('alicia_flexible_grasp_supervisor.srv', _MISSING)

srv_module = types.ModuleType('alicia_flexible_grasp_supervisor.srv')
for name in (
    'SetJointCommand',
    'SetFloat',
    'SetTargetPose',
    'CartesianJog',
    'TriggerZero',
):
    setattr(srv_module, name, type(name, (), {}))
for name in (
    'SetJointCommandResponse',
    'SetFloatResponse',
    'SetTargetPoseResponse',
    'CartesianJogResponse',
    'TriggerZeroResponse',
):
    setattr(srv_module, name, lambda success=False, message='': types.SimpleNamespace(success=success, message=message))
pkg_module = types.ModuleType('alicia_flexible_grasp_supervisor')
pkg_module.__path__ = []
sys.modules.setdefault('alicia_flexible_grasp_supervisor', pkg_module)
sys.modules['alicia_flexible_grasp_supervisor.srv'] = srv_module

SCRIPT = ROOT / 'scripts' / 'motion_gateway_node.py'
spec = importlib.util.spec_from_file_location('motion_gateway_node', str(SCRIPT))
motion_gateway_node = importlib.util.module_from_spec(spec)
try:
    spec.loader.exec_module(motion_gateway_node)
finally:
    if _original_supervisor_pkg is _MISSING:
        sys.modules.pop('alicia_flexible_grasp_supervisor', None)
    else:
        sys.modules['alicia_flexible_grasp_supervisor'] = _original_supervisor_pkg
    if _original_supervisor_srv is _MISSING:
        sys.modules.pop('alicia_flexible_grasp_supervisor.srv', None)
    else:
        sys.modules['alicia_flexible_grasp_supervisor.srv'] = _original_supervisor_srv


class FakeGripper:
    def __init__(self):
        self.calls = []

    def set_position(self, value, arm_positions=None):
        snapshot = None if arm_positions is None else list(arm_positions)
        self.calls.append((float(value), snapshot))
        return True


class MotionGatewayGripperHoldTest(unittest.TestCase):
    def test_gripper_command_burst_reuses_initial_arm_snapshot(self):
        node = motion_gateway_node.MotionGateway.__new__(motion_gateway_node.MotionGateway)
        node.joint_names = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6', 'right_finger']
        node.joint_cmd = types.SimpleNamespace(last_positions=[1, 2, 3, 4, 5, 6, 0.0])
        node.gripper = FakeGripper()
        node._gripper_arm_hold_positions = None
        node._last_gripper_command_time = 0.0

        times = iter([1.0, 1.2, 2.0])
        original_get_time = motion_gateway_node.rospy.get_time
        original_get_param = motion_gateway_node.rospy.get_param
        motion_gateway_node.rospy.get_time = lambda: next(times)
        motion_gateway_node.rospy.get_param = lambda name, default=None: {
            '/gripper': {
                'hold_arm_during_gripper_commands': True,
                'arm_hold_timeout_sec': 0.5,
            }
        }.get(name, default)
        try:
            node.handle_gripper(types.SimpleNamespace(value=0.01))
            node.joint_cmd.last_positions = [10, 20, 30, 40, 50, 60, 0.01]
            node.handle_gripper(types.SimpleNamespace(value=0.02))
            node.handle_gripper(types.SimpleNamespace(value=0.03))
        finally:
            motion_gateway_node.rospy.get_time = original_get_time
            motion_gateway_node.rospy.get_param = original_get_param

        self.assertEqual(node.gripper.calls[0], (0.01, [1, 2, 3, 4, 5, 6]))
        self.assertEqual(node.gripper.calls[1], (0.02, [1, 2, 3, 4, 5, 6]))
        self.assertEqual(node.gripper.calls[2], (0.03, [10, 20, 30, 40, 50, 60]))


if __name__ == '__main__':
    unittest.main()
