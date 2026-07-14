#!/usr/bin/env python3
import importlib.util
import pathlib
import sys
import types
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / 'src', ROOT / 'scripts'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

MODULE_PATH = ROOT / 'scripts' / 'motion_gateway_node.py'
spec = importlib.util.spec_from_file_location('motion_gateway_node', str(MODULE_PATH))
motion_gateway_node = importlib.util.module_from_spec(spec)
spec.loader.exec_module(motion_gateway_node)
MotionGateway = motion_gateway_node.MotionGateway


class FakePlanner:
    def __init__(self):
        self.calls = []

    def move_to_pose(self, target, execute=True, allow_fallbacks=True):
        self.calls.append((target, execute, allow_fallbacks))
        return True, 'executed'

    def move_to_pose_linear(self, target, execute=True):
        self.calls.append(('linear', target, execute))
        return True, 'linear executed'


class FakeJogger:
    def __init__(self, planner):
        self.planner = planner


class MotionGatewayControllerStartTest(unittest.TestCase):
    def make_gateway(self, controller_result=(True, 'controllers started')):
        gateway = MotionGateway.__new__(MotionGateway)
        gateway.planner = FakePlanner()
        gateway._ensure_planner = lambda: gateway.planner
        gateway._log_pose_request = lambda req, *args, **kwargs: None
        gateway._ensure_trajectory_controllers_started = lambda: controller_result
        return gateway

    def test_execute_pose_starts_trajectory_controllers_before_moveit(self):
        gateway = self.make_gateway()
        req = types.SimpleNamespace(target='pose', execute=True)

        res = MotionGateway.handle_pose(gateway, req)

        self.assertTrue(res.success)
        self.assertEqual(gateway.planner.calls, [('pose', True, True)])

    def test_execute_pose_stops_before_moveit_when_controllers_do_not_start(self):
        gateway = self.make_gateway((False, 'controllers stopped'))
        req = types.SimpleNamespace(target='pose', execute=True)

        res = MotionGateway.handle_pose(gateway, req)

        self.assertFalse(res.success)
        self.assertIn('controllers stopped', res.message)
        self.assertEqual(gateway.planner.calls, [])

    def test_plan_pose_does_not_require_trajectory_controllers(self):
        gateway = self.make_gateway((False, 'controllers stopped'))
        req = types.SimpleNamespace(target='pose', execute=False)

        res = MotionGateway.handle_pose(gateway, req)

        self.assertTrue(res.success)
        self.assertEqual(gateway.planner.calls, [('pose', False, True)])

    def test_pose_service_retries_planner_initialization(self):
        gateway = MotionGateway.__new__(MotionGateway)
        gateway.planner = None
        planner = FakePlanner()
        gateway._ensure_planner = lambda: planner
        gateway._log_pose_request = lambda req, *args, **kwargs: None
        gateway._ensure_trajectory_controllers_started = lambda: (True, 'controllers started')
        req = types.SimpleNamespace(target='pose', execute=False)

        res = MotionGateway.handle_pose(gateway, req)

        self.assertTrue(res.success)
        self.assertEqual(planner.calls, [('pose', False, True)])

    def test_pose_service_reports_moveit_not_ready_without_crashing(self):
        gateway = MotionGateway.__new__(MotionGateway)
        gateway.planner = None
        gateway._ensure_planner = lambda: None
        gateway._log_pose_request = lambda req, *args, **kwargs: None
        gateway._ensure_trajectory_controllers_started = lambda: (True, 'controllers started')
        req = types.SimpleNamespace(target='pose', execute=False)

        res = MotionGateway.handle_pose(gateway, req)

        self.assertFalse(res.success)
        self.assertIn('MoveIt not ready', res.message)

    def test_strict_pose_service_disables_all_fallbacks(self):
        gateway = self.make_gateway()
        req = types.SimpleNamespace(target='pose', execute=False)

        res = MotionGateway.handle_pose_strict(gateway, req)

        self.assertTrue(res.success)
        self.assertEqual(gateway.planner.calls, [('pose', False, False)])

    def test_strict_pose_service_refuses_execution(self):
        gateway = self.make_gateway()
        req = types.SimpleNamespace(target='pose', execute=True)

        res = MotionGateway.handle_pose_strict(gateway, req)

        self.assertFalse(res.success)
        self.assertIn('planning-only', res.message)
        self.assertEqual(gateway.planner.calls, [])

    def test_execute_linear_pose_starts_controllers_and_uses_linear_planner(self):
        gateway = self.make_gateway()
        req = types.SimpleNamespace(target='pose', execute=True)

        res = MotionGateway.handle_pose_linear(gateway, req)

        self.assertTrue(res.success)
        self.assertEqual(gateway.planner.calls, [('linear', 'pose', True)])

    def test_linear_pose_stops_when_controllers_are_unavailable(self):
        gateway = self.make_gateway((False, 'controllers stopped'))
        req = types.SimpleNamespace(target='pose', execute=True)

        res = MotionGateway.handle_pose_linear(gateway, req)

        self.assertFalse(res.success)
        self.assertIn('controllers stopped', res.message)
        self.assertEqual(gateway.planner.calls, [])

    def test_controller_state_check_reports_stopped_or_missing_controllers(self):
        missing = MotionGateway._non_running_controllers(
            {
                'alicia_controller': 'running',
                'hand_controller': 'stopped',
            },
            ['alicia_controller', 'hand_controller', 'extra_controller'],
        )

        self.assertEqual(missing, ['hand_controller=stopped', 'extra_controller=missing'])


if __name__ == '__main__':
    unittest.main()
