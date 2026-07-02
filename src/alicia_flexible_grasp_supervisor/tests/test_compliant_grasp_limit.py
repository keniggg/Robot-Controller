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

from alicia_flexible_grasp.grasp.compliant_grasp import CompliantGraspLogic


class FakeDuration:
    def __init__(self, seconds):
        self.seconds = float(seconds)

    def to_sec(self):
        return self.seconds


class FakeStamp:
    def __init__(self, seconds):
        self.seconds = float(seconds)

    def __sub__(self, other):
        return FakeDuration(self.seconds - float(other.seconds))


class FakeRate:
    def __init__(self, _hz):
        pass

    def sleep(self):
        pass


class FakeLogic:
    def next_position(self, current_pos, force):
        return current_pos, 'no_contact_limit'


class CompliantGraspLimitTest(unittest.TestCase):
    def test_logic_reports_no_contact_at_close_limit(self):
        logic = CompliantGraspLogic(
            contact_threshold=200.0,
            target_force=1500.0,
            max_force=4000.0,
            close_step_fast=0.002,
            close_step_slow=0.0007,
            open_step_safe=0.003,
            gripper_min=0.0,
            gripper_max=0.05,
        )

        pos, state = logic.next_position(0.05, 0.0)

        self.assertEqual(pos, 0.05)
        self.assertEqual(state, 'no_contact_limit')

    def _load_controller_module(self):
        missing = object()
        original_pkg = sys.modules.get('alicia_flexible_grasp_supervisor', missing)
        original_msg = sys.modules.get('alicia_flexible_grasp_supervisor.msg', missing)
        original_srv = sys.modules.get('alicia_flexible_grasp_supervisor.srv', missing)
        msg_module = types.ModuleType('alicia_flexible_grasp_supervisor.msg')
        msg_module.TactileState = type('TactileState', (), {})
        srv_module = types.ModuleType('alicia_flexible_grasp_supervisor.srv')
        srv_module.StartGrasp = type('StartGrasp', (), {})
        srv_module.SetFloat = type('SetFloat', (), {})
        srv_module.StartGraspResponse = lambda success=False, message='': types.SimpleNamespace(
            success=success,
            message=message,
        )
        pkg_module = types.ModuleType('alicia_flexible_grasp_supervisor')
        pkg_module.__path__ = []
        sys.modules.setdefault('alicia_flexible_grasp_supervisor', pkg_module)
        sys.modules['alicia_flexible_grasp_supervisor.msg'] = msg_module
        sys.modules['alicia_flexible_grasp_supervisor.srv'] = srv_module

        script = ROOT / 'scripts' / 'compliant_gripper_controller.py'
        spec = importlib.util.spec_from_file_location('compliant_gripper_controller', str(script))
        module = importlib.util.module_from_spec(spec)
        try:
            spec.loader.exec_module(module)
        finally:
            if original_pkg is missing:
                sys.modules.pop('alicia_flexible_grasp_supervisor', None)
            else:
                sys.modules['alicia_flexible_grasp_supervisor'] = original_pkg
            if original_msg is missing:
                sys.modules.pop('alicia_flexible_grasp_supervisor.msg', None)
            else:
                sys.modules['alicia_flexible_grasp_supervisor.msg'] = original_msg
            if original_srv is missing:
                sys.modules.pop('alicia_flexible_grasp_supervisor.srv', None)
            else:
                sys.modules['alicia_flexible_grasp_supervisor.srv'] = original_srv
        return module

    def test_controller_returns_no_contact_instead_of_timeout_at_limit(self):
        module = self._load_controller_module()
        node = module.CompliantGripperController.__new__(module.CompliantGripperController)
        node.logic = FakeLogic()
        node.force = 0.0
        node.gripper_pos = 0.05
        node.rate_hz = 25.0
        node.tactile_valid = True
        node.last_tactile_time = FakeStamp(0.0)
        node.set_gripper = lambda value: types.SimpleNamespace(success=True, message='ok')

        stamps = iter([FakeStamp(0.0), FakeStamp(0.0), FakeStamp(0.0), FakeStamp(9.0)])
        original_rate = module.rospy.Rate
        original_time = module.rospy.Time
        original_is_shutdown = module.rospy.is_shutdown
        original_get_param = module.rospy.get_param
        module.rospy.Rate = FakeRate
        module.rospy.Time = types.SimpleNamespace(now=lambda: next(stamps))
        module.rospy.is_shutdown = lambda: False
        module.rospy.get_param = lambda name, default=None: default
        try:
            response = module.CompliantGripperController.handle_close(
                node,
                types.SimpleNamespace(execute=True),
            )
        finally:
            module.rospy.Rate = original_rate
            module.rospy.Time = original_time
            module.rospy.is_shutdown = original_is_shutdown
            module.rospy.get_param = original_get_param

        self.assertFalse(response.success)
        self.assertIn('no contact', response.message)

    def test_controller_refuses_to_close_without_tactile_feedback(self):
        module = self._load_controller_module()
        node = module.CompliantGripperController.__new__(module.CompliantGripperController)
        node.logic = FakeLogic()
        node.force = 0.0
        node.gripper_pos = 0.0
        node.rate_hz = 25.0
        node.tactile_valid = False
        node.last_tactile_time = None
        set_gripper_calls = []
        node.set_gripper = lambda value: set_gripper_calls.append(value)

        original_time = module.rospy.Time
        original_get_param = module.rospy.get_param
        module.rospy.Time = types.SimpleNamespace(now=lambda: FakeStamp(1.0))
        module.rospy.get_param = lambda name, default=None: default
        try:
            response = module.CompliantGripperController.handle_close(
                node,
                types.SimpleNamespace(execute=True),
            )
        finally:
            module.rospy.Time = original_time
            module.rospy.get_param = original_get_param

        self.assertFalse(response.success)
        self.assertIn('tactile feedback unavailable', response.message)
        self.assertEqual(set_gripper_calls, [])


if __name__ == '__main__':
    unittest.main()
