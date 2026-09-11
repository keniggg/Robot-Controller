#!/usr/bin/env python3
import importlib.util
import pathlib
import sys
import types
import unittest
from unittest import mock


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

    def execute_cached_strict_pose(self, target):
        self.calls.append(('strict_cached', target))
        return True, 'strict cached executed'

    def check_pose_sequence(
        self,
        targets,
        stage_names,
        linear,
        deadline_sec=0.0,
    ):
        self.calls.append(
            (
                'strict_sequence',
                list(targets),
                list(stage_names),
                list(linear),
                float(deadline_sec),
            )
        )
        return (
            True,
            '',
            '',
            {'path_cost': 0.7, 'max_delta': 0.4},
            'strict sequence planned',
        )

    def resolve_free_space_orientations(
        self,
        targets,
        stage_names,
        linear,
        resolve_orientation,
        deadline_sec=0.0,
    ):
        self.calls.append(
            (
                'resolve_orientations',
                list(targets),
                list(stage_names),
                list(linear),
                list(resolve_orientation),
                float(deadline_sec),
            )
        )
        return (
            True,
            '',
            '',
            tuple(targets),
            {
                'path_cost': 0.8,
                'max_delta': 0.5,
                'max_position_error': 0.001,
            },
            'orientation seeds resolved',
        )


class FakeJogger:
    def __init__(self, planner):
        self.planner = planner


class MotionGatewayControllerStartTest(unittest.TestCase):
    def setUp(self):
        # Unit cases must not depend on the live operator's control checkbox.
        patcher = mock.patch.object(motion_gateway_node.rospy, 'get_param',
                                    side_effect=lambda name, default=None: default)
        patcher.start()
        self.addCleanup(patcher.stop)

    def make_gateway(
        self,
        controller_result=(True, 'controllers started'),
        sync_result=(True, 'controller synchronized'),
    ):
        gateway = MotionGateway.__new__(MotionGateway)
        gateway.planner = FakePlanner()
        gateway._ensure_planner = lambda: gateway.planner
        gateway._log_pose_request = lambda req, *args, **kwargs: None
        gateway.controller_checks = 0

        def check_controllers():
            gateway.controller_checks += 1
            return controller_result

        gateway.controller_syncs = 0

        def sync_controller():
            gateway.controller_syncs += 1
            return sync_result

        gateway._ensure_trajectory_controllers_started = check_controllers
        gateway._synchronize_trajectory_controller_to_feedback = (
            sync_controller
        )
        gateway.failure_holds = 0
        gateway.failure_hold_start_positions = []
        gateway.positive_enable_requests = []
        gateway.demo_pub = types.SimpleNamespace(
            publish=lambda msg: gateway.positive_enable_requests.append(
                bool(msg.data)
            )
        )
        gateway._current_arm_positions_snapshot = (
            lambda: [0.1, -0.2]
        )

        def hold_after_failure(start_positions):
            gateway.failure_holds += 1
            gateway.failure_hold_start_positions.append(
                list(start_positions)
            )
            return True, 'desired command preserved'

        gateway._hold_controller_command_after_failed_execution = (
            hold_after_failure
        )
        return gateway

    def test_observation_profile_has_separate_cache_and_restores_contact_planner(self):
        gateway = self.make_gateway()
        contact = gateway.planner
        observation = FakePlanner()
        observation.ready = True
        observation.strict_execution_max_joint_velocity_rad_s = 0.08
        gateway._observation_planner = observation
        req = types.SimpleNamespace(target='observation', execute=False)
        result = gateway.handle_observation_pose_strict(req)
        self.assertTrue(result.success)
        self.assertIn('speed_profile=observation', result.message)
        self.assertEqual(observation.calls, [('observation', False, False)])
        self.assertEqual(contact.calls, [])
        self.assertIs(gateway.planner, contact)
        gateway.handle_pose_strict(req)
        self.assertEqual(contact.calls, [('observation', False, False)])
        with self.assertRaisesRegex(RuntimeError, 'test error'):
            gateway._with_observation_planner(req, mock.Mock(side_effect=RuntimeError('test error')))
        self.assertIs(gateway.planner, contact)

    def test_observation_profile_initialization_preserves_contact_limits(self):
        gateway = self.make_gateway()
        contact = gateway.planner
        contact.strict_execution_joint_velocity_limits_rad_s = {'Joint6': 0.02}
        gateway.manipulator_group, gateway.gripper_group, gateway.velocity = 'alicia', 'hand', 0.3
        observation = FakePlanner()
        observation.ready = True
        observation.strict_execution_max_joint_velocity_rad_s = 0.08
        observation.strict_execution_joint_velocity_limits_rad_s = {'Joint6': 0.02}
        with mock.patch.object(motion_gateway_node, 'MoveItPlanner', return_value=observation):
            result = gateway.handle_observation_pose_strict(types.SimpleNamespace(target='pose', execute=False))
        self.assertTrue(result.success)
        self.assertEqual(observation.strict_execution_joint_velocity_limits_rad_s, {})
        self.assertEqual(contact.strict_execution_joint_velocity_limits_rad_s, {'Joint6': 0.02})

    def test_observation_execution_never_initializes_when_manual_control_owns_arm(self):
        gateway = self.make_gateway()
        gateway._gui_direct_mode = True
        with mock.patch.object(motion_gateway_node, 'MoveItPlanner') as constructor:
            for handler in (gateway.handle_observation_pose_strict_execute,
                            gateway.handle_observation_pose_strict_plan_execute):
                result = handler(types.SimpleNamespace(target='pose', execute=True))
                self.assertFalse(result.success)
                self.assertIn('MANUAL_CONTROL_ACTIVE', result.message)
            constructor.assert_not_called()

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

    def test_strict_sequence_service_is_planning_only_and_structured(self):
        gateway = self.make_gateway()
        req = types.SimpleNamespace(
            targets=['pregrasp', 'approach', 'grasp'],
            stage_names=['pregrasp', 'approach', 'grasp'],
            linear=[False, True, True],
            deadline=types.SimpleNamespace(to_sec=lambda: 42.0),
        )

        res = MotionGateway.handle_pose_sequence_strict(gateway, req)

        self.assertTrue(res.success)
        self.assertEqual(res.failure_code, '')
        self.assertEqual(res.failed_stage, '')
        self.assertAlmostEqual(res.joint_path_cost, 0.7)
        self.assertAlmostEqual(res.joint_max_delta, 0.4)
        self.assertEqual(
            gateway.planner.calls,
            [(
                'strict_sequence',
                ['pregrasp', 'approach', 'grasp'],
                ['pregrasp', 'approach', 'grasp'],
                [False, True, True],
                42.0,
            )],
        )

    def test_cached_strict_execute_requires_execute_true(self):
        gateway = self.make_gateway()
        req = types.SimpleNamespace(target='pose', execute=False)

        res = MotionGateway.handle_pose_strict_execute(gateway, req)

        self.assertFalse(res.success)
        self.assertIn('requires execute=true', res.message)
        self.assertEqual(gateway.controller_checks, 0)
        self.assertEqual(gateway.planner.calls, [])
        self.assertEqual(gateway.controller_checks, 0)
        self.assertEqual(gateway.planner.calls, [])

    def test_cached_strict_execute_checks_controllers_then_uses_cached_only_api(self):
        gateway = self.make_gateway()
        req = types.SimpleNamespace(target='pose', execute=True)

        res = MotionGateway.handle_pose_strict_execute(gateway, req)

        self.assertTrue(res.success)
        self.assertEqual(gateway.controller_checks, 1)
        self.assertEqual(gateway.controller_syncs, 1)
        self.assertEqual(gateway.planner.calls, [('strict_cached', 'pose')])

    def test_cached_strict_execute_stops_before_planner_on_controller_failure(self):
        gateway = self.make_gateway((False, 'controllers stopped'))
        req = types.SimpleNamespace(target='pose', execute=True)

        res = MotionGateway.handle_pose_strict_execute(gateway, req)

        self.assertFalse(res.success)
        self.assertIn('controllers stopped', res.message)
        self.assertEqual(gateway.controller_checks, 1)
        self.assertEqual(gateway.controller_syncs, 0)
        self.assertEqual(gateway.planner.calls, [])

    def test_cached_strict_execute_stops_before_planner_on_sync_failure(self):
        gateway = self.make_gateway(
            sync_result=(False, 'controller hold not acknowledged')
        )
        req = types.SimpleNamespace(target='pose', execute=True)

        res = MotionGateway.handle_pose_strict_execute(gateway, req)

        self.assertFalse(res.success)
        self.assertIn('controller hold not acknowledged', res.message)
        self.assertEqual(gateway.controller_checks, 1)
        self.assertEqual(gateway.controller_syncs, 1)
        self.assertEqual(gateway.planner.calls, [])

    def test_atomic_strict_execute_replans_and_executes_under_one_gateway_call(self):
        gateway = self.make_gateway()
        req = types.SimpleNamespace(target='pose', execute=True)

        res = MotionGateway.handle_pose_strict_plan_execute(gateway, req)

        self.assertTrue(res.success)
        self.assertEqual(gateway.controller_checks, 1)
        self.assertEqual(gateway.controller_syncs, 1)
        self.assertEqual(
            gateway.planner.calls,
            [
                ('pose', False, False),
                ('strict_cached', 'pose'),
            ],
        )

    def test_atomic_strict_execute_requires_execute_true(self):
        gateway = self.make_gateway()
        req = types.SimpleNamespace(target='pose', execute=False)

        res = MotionGateway.handle_pose_strict_plan_execute(gateway, req)

        self.assertFalse(res.success)
        self.assertIn('requires execute=true', res.message)

    def test_atomic_strict_execution_failure_preserves_desired_command(self):
        gateway = self.make_gateway()
        gateway.planner.execute_cached_strict_pose = (
            lambda _target: (False, 'controller path failure')
        )
        req = types.SimpleNamespace(target='pose', execute=True)

        res = MotionGateway.handle_pose_strict_plan_execute(gateway, req)

        self.assertFalse(res.success)
        self.assertIn('controller failure hold installed', res.message)
        self.assertEqual(gateway.controller_syncs, 1)
        self.assertEqual(gateway.failure_holds, 1)
        self.assertEqual(
            gateway.failure_hold_start_positions,
            [[0.1, -0.2]],
        )
        self.assertEqual(gateway.positive_enable_requests, [False])
        self.assertIn('positive joint enable re-requested', res.message)

    def test_failed_execution_hold_republishes_desired_not_feedback(self):
        gateway = MotionGateway.__new__(MotionGateway)
        gateway.joint_names = ['Joint1', 'Joint2', 'right_finger']
        gateway._trajectory_controller_state = types.SimpleNamespace(
            header=types.SimpleNamespace(
                stamp=types.SimpleNamespace(to_sec=lambda: 10.0),
            ),
            joint_names=['Joint1', 'Joint2'],
            desired=types.SimpleNamespace(positions=[0.4, -0.3]),
            actual=types.SimpleNamespace(positions=[0.37, -0.337]),
        )
        published = []
        gateway.trajectory_command_pub = types.SimpleNamespace(
            publish=published.append,
        )
        config = {
            'strict_execution_controller_sync_max_feedback_age_sec': 0.5,
            'strict_execution_controller_sync_bridge_duration_sec': 0.02,
        }

        with mock.patch.object(
            motion_gateway_node.rospy,
            'get_param',
            return_value=config,
        ), mock.patch.object(
            motion_gateway_node.rospy,
            'get_time',
            return_value=10.1,
        ):
            ok, message = gateway._hold_controller_desired_command()

        self.assertTrue(ok)
        self.assertIn('continuous desired-command hold', message)
        self.assertEqual(len(published), 1)
        self.assertEqual(published[0].joint_names, ['Joint1', 'Joint2'])
        self.assertEqual(published[0].points[0].positions, [0.4, -0.3])
        self.assertNotEqual(
            published[0].points[0].positions,
            [0.37, -0.337],
        )

    def test_failed_execution_without_measured_motion_holds_actual_feedback(self):
        gateway = MotionGateway.__new__(MotionGateway)
        gateway.joint_names = ['Joint1', 'Joint2', 'right_finger']
        gateway._trajectory_controller_state = types.SimpleNamespace(
            header=types.SimpleNamespace(
                stamp=types.SimpleNamespace(to_sec=lambda: 10.0),
            ),
            joint_names=['Joint1', 'Joint2'],
            desired=types.SimpleNamespace(positions=[0.5001, -0.3]),
            actual=types.SimpleNamespace(positions=[0.3705, -0.337]),
        )
        published = []
        gateway.trajectory_command_pub = types.SimpleNamespace(
            publish=published.append,
        )
        config = {
            'strict_execution_controller_sync_max_feedback_age_sec': 0.5,
            'strict_execution_controller_sync_bridge_duration_sec': 0.02,
            'strict_execution_no_actuation_actual_delta_rad': 0.0031,
            'strict_execution_no_actuation_min_desired_delta_rad': 0.12,
        }

        with mock.patch.object(
            motion_gateway_node.rospy,
            'get_param',
            return_value=config,
        ), mock.patch.object(
            motion_gateway_node.rospy,
            'get_time',
            return_value=10.1,
        ):
            ok, message = (
                gateway._hold_controller_command_after_failed_execution(
                    [0.37, -0.337],
                )
            )

        self.assertTrue(ok)
        self.assertIn('no-actuation feedback hold', message)
        self.assertIn('actual_delta=0.000500rad', message)
        self.assertEqual(len(published), 1)
        self.assertEqual(published[0].joint_names, ['Joint1', 'Joint2'])
        self.assertEqual(
            published[0].points[0].positions,
            [0.3705, -0.337],
        )

    def test_failed_execution_after_partial_motion_preserves_desired(self):
        gateway = MotionGateway.__new__(MotionGateway)
        gateway.joint_names = ['Joint1', 'Joint2', 'right_finger']
        gateway._trajectory_controller_state = types.SimpleNamespace(
            header=types.SimpleNamespace(
                stamp=types.SimpleNamespace(to_sec=lambda: 10.0),
            ),
            joint_names=['Joint1', 'Joint2'],
            desired=types.SimpleNamespace(positions=[0.5001, -0.3]),
            actual=types.SimpleNamespace(positions=[0.38, -0.337]),
        )
        published = []
        gateway.trajectory_command_pub = types.SimpleNamespace(
            publish=published.append,
        )
        config = {
            'strict_execution_controller_sync_max_feedback_age_sec': 0.5,
            'strict_execution_controller_sync_bridge_duration_sec': 0.02,
            'strict_execution_no_actuation_actual_delta_rad': 0.0031,
            'strict_execution_no_actuation_min_desired_delta_rad': 0.12,
        }

        with mock.patch.object(
            motion_gateway_node.rospy,
            'get_param',
            return_value=config,
        ), mock.patch.object(
            motion_gateway_node.rospy,
            'get_time',
            return_value=10.1,
        ):
            ok, message = (
                gateway._hold_controller_command_after_failed_execution(
                    [0.37, -0.337],
                )
            )

        self.assertTrue(ok)
        self.assertIn('continuous desired-command hold', message)
        self.assertIn('actual_delta=0.010000rad', message)
        self.assertEqual(len(published), 1)
        self.assertEqual(
            published[0].points[0].positions,
            [0.5001, -0.3],
        )

    def test_running_controllers_do_not_receive_redundant_start_request(self):
        gateway = MotionGateway.__new__(MotionGateway)
        gateway.trajectory_controller_names = [
            'alicia_controller',
            'hand_controller',
        ]
        response = types.SimpleNamespace(
            controller=[
                types.SimpleNamespace(
                    name='alicia_controller',
                    state='running',
                ),
                types.SimpleNamespace(
                    name='hand_controller',
                    state='running',
                ),
            ],
        )
        service_names = []

        def service_proxy(name, _service_type):
            service_names.append(name)
            return lambda: response

        with mock.patch.object(
            motion_gateway_node,
            'SwitchController',
            object(),
        ), mock.patch.object(
            motion_gateway_node,
            'SwitchControllerRequest',
            object(),
        ), mock.patch.object(
            motion_gateway_node,
            'ListControllers',
            object(),
        ), mock.patch.object(
            motion_gateway_node.rospy,
            'wait_for_service',
            return_value=None,
        ), mock.patch.object(
            motion_gateway_node.rospy,
            'ServiceProxy',
            side_effect=service_proxy,
        ):
            ok, message = gateway._ensure_trajectory_controllers_started()

        self.assertTrue(ok)
        self.assertIn('start request not repeated', message)
        self.assertEqual(
            service_names,
            ['/controller_manager/list_controllers'],
        )

    def test_controller_sync_uses_one_cycle_feedback_bridge_until_settled(self):
        gateway = MotionGateway.__new__(MotionGateway)
        gateway.joint_names = ['Joint1', 'Joint2', 'right_finger']
        gateway.joint_cmd = types.SimpleNamespace(
            last_state_time_sec=1.0,
            last_positions=[0.4, -0.2, 0.03],
        )
        gateway.gripper = types.SimpleNamespace(gripper_index=2)
        published = []
        gateway.trajectory_command_pub = types.SimpleNamespace(
            publish=published.append,
        )
        trajectory_state = types.SimpleNamespace(
            header=types.SimpleNamespace(
                stamp=types.SimpleNamespace(to_sec=lambda: 1.0),
            ),
            joint_names=['Joint1', 'Joint2'],
            desired=types.SimpleNamespace(positions=[0.4, -0.2]),
            actual=types.SimpleNamespace(positions=[0.4, -0.2]),
        )
        gateway._trajectory_controller_state = trajectory_state
        times = iter([1.0, 1.0, 1.0, 1.10, 1.21])
        config = {
            'strict_execution_controller_sync_bridge_duration_sec': 0.02,
            'strict_execution_controller_sync_duration_sec': 0.20,
            'strict_execution_controller_sync_timeout_sec': 1.0,
            'strict_execution_controller_sync_tolerance_rad': 0.01,
            'strict_execution_controller_sync_max_feedback_age_sec': 0.5,
        }

        with mock.patch.object(
            motion_gateway_node.rospy,
            'get_param',
            return_value=config,
        ), mock.patch.object(
            motion_gateway_node.rospy,
            'get_time',
            side_effect=lambda: next(times),
        ), mock.patch.object(
            motion_gateway_node.rospy,
            'is_shutdown',
            return_value=False,
        ), mock.patch.object(
            motion_gateway_node.rospy,
            'sleep',
            return_value=None,
        ):
            ok, message = (
                gateway._synchronize_trajectory_controller_to_feedback()
            )

        self.assertTrue(ok)
        self.assertIn('stable_for=0.210s', message)
        self.assertEqual(len(published), 1)
        self.assertEqual(published[0].joint_names, ['Joint1', 'Joint2'])
        self.assertEqual(len(published[0].points), 1)
        self.assertEqual(published[0].points[0].positions, [0.4, -0.2])
        self.assertEqual(
            published[0].points[0].time_from_start.to_sec(),
            0.02,
        )

    def test_controller_sync_restarts_settle_window_after_error_excursion(self):
        gateway = MotionGateway.__new__(MotionGateway)
        gateway.joint_names = ['Joint1', 'Joint2', 'right_finger']
        gateway.joint_cmd = types.SimpleNamespace(
            last_state_time_sec=1.0,
            last_positions=[0.4, -0.2, 0.03],
        )
        gateway.gripper = types.SimpleNamespace(gripper_index=2)
        gateway.trajectory_command_pub = types.SimpleNamespace(
            publish=lambda _message: None,
        )
        actual = types.SimpleNamespace(positions=[0.4, -0.2])
        gateway._trajectory_controller_state = types.SimpleNamespace(
            header=types.SimpleNamespace(
                stamp=types.SimpleNamespace(to_sec=lambda: 1.0),
            ),
            joint_names=['Joint1', 'Joint2'],
            desired=types.SimpleNamespace(positions=[0.4, -0.2]),
            actual=actual,
        )
        times = iter([1.0, 1.0, 1.0, 1.10, 1.20, 1.30, 1.55])
        actual_sequence = iter([
            [0.43, -0.2],
            [0.4, -0.2],
            [0.4, -0.2],
            [0.4, -0.2],
        ])

        def advance_feedback(_duration):
            actual.positions = next(actual_sequence)

        config = {
            'strict_execution_controller_sync_bridge_duration_sec': 0.02,
            'strict_execution_controller_sync_duration_sec': 0.20,
            'strict_execution_controller_sync_timeout_sec': 1.0,
            'strict_execution_controller_sync_tolerance_rad': 0.01,
            'strict_execution_controller_sync_max_feedback_age_sec': 0.5,
        }
        with mock.patch.object(
            motion_gateway_node.rospy,
            'get_param',
            return_value=config,
        ), mock.patch.object(
            motion_gateway_node.rospy,
            'get_time',
            side_effect=lambda: next(times),
        ), mock.patch.object(
            motion_gateway_node.rospy,
            'is_shutdown',
            return_value=False,
        ), mock.patch.object(
            motion_gateway_node.rospy,
            'sleep',
            side_effect=advance_feedback,
        ):
            ok, message = (
                gateway._synchronize_trajectory_controller_to_feedback()
            )

        self.assertTrue(ok)
        self.assertIn('stable_for=0.350s', message)

    def test_cached_strict_execute_reports_moveit_not_ready(self):
        gateway = self.make_gateway()
        gateway.planner = None
        gateway._ensure_planner = lambda: None
        gateway._moveit_not_ready_message = lambda: 'MoveIt not ready: unavailable'
        req = types.SimpleNamespace(target='pose', execute=True)

        res = MotionGateway.handle_pose_strict_execute(gateway, req)

        self.assertFalse(res.success)
        self.assertIn('MoveIt not ready', res.message)
        self.assertEqual(gateway.controller_checks, 0)

    def test_cached_strict_service_name_is_registered(self):
        self.assertIn(
            '/supervisor/execute_pose_strict',
            MotionGateway.__init__.__code__.co_consts,
        )
        self.assertIn(
            '/supervisor/plan_and_execute_pose_strict',
            MotionGateway.__init__.__code__.co_consts,
        )
        self.assertIn(
            '/supervisor/check_pose_sequence_strict',
            MotionGateway.__init__.__code__.co_consts,
        )
        self.assertIn(
            '/supervisor/resolve_free_space_orientations',
            MotionGateway.__init__.__code__.co_consts,
        )

    def test_free_space_orientation_service_is_planning_only(self):
        gateway = self.make_gateway()
        req = types.SimpleNamespace(
            targets=['pregrasp'],
            stage_names=['pregrasp'],
            linear=[False],
            resolve_orientation=[True],
            deadline=types.SimpleNamespace(to_sec=lambda: 43.0),
        )

        res = MotionGateway.handle_resolve_free_space_orientations(
            gateway,
            req,
        )

        self.assertTrue(res.success)
        self.assertEqual(res.resolved_targets, ['pregrasp'])
        self.assertAlmostEqual(res.joint_path_cost, 0.8)
        self.assertAlmostEqual(res.joint_max_delta, 0.5)
        self.assertAlmostEqual(res.max_position_error, 0.001)
        self.assertIn(
            'policy=deterministic_geodesic_collision_ik',
            res.message,
        )
        self.assertEqual(gateway.controller_checks, 0)
        self.assertEqual(
            gateway.planner.calls,
            [
                (
                    'resolve_orientations',
                    ['pregrasp'],
                    ['pregrasp'],
                    [False],
                    [True],
                    43.0,
                )
            ],
        )

    def test_free_space_orientation_failure_attests_resolver_policy(self):
        gateway = self.make_gateway()
        gateway.planner.resolve_free_space_orientations = (
            lambda *_args, **_kwargs: (
                False,
                'MOVEIT_UNREACHABLE',
                'approach',
                (),
                {
                    'path_cost': 0.3,
                    'max_delta': 0.2,
                    'max_position_error': 0.0,
                },
                'Cartesian fraction below the hard bound',
            )
        )
        req = types.SimpleNamespace(
            targets=['pregrasp'],
            stage_names=['pregrasp'],
            linear=[False],
            resolve_orientation=[True],
        )

        res = MotionGateway.handle_resolve_free_space_orientations(
            gateway,
            req,
        )

        self.assertFalse(res.success)
        self.assertEqual(res.failure_code, 'MOVEIT_UNREACHABLE')
        self.assertEqual(res.failed_stage, 'approach')
        self.assertIn(
            'policy=deterministic_geodesic_collision_ik',
            res.message,
        )
        self.assertEqual(gateway.controller_checks, 0)

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
