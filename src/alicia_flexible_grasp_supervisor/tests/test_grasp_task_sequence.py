#!/usr/bin/env python3
import importlib.util
import pathlib
import sys
import types
import unittest

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray


ROOT = pathlib.Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / 'src'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

SCRIPT = ROOT / 'scripts' / 'grasp_task_node.py'
spec = importlib.util.spec_from_file_location('grasp_task_node', str(SCRIPT))
grasp_task_node = importlib.util.module_from_spec(spec)
spec.loader.exec_module(grasp_task_node)


class FakeServiceResponse:
    def __init__(self, success=True, message='ok'):
        self.success = bool(success)
        self.message = message


class FakeDuration:
    def __init__(self, seconds):
        self.seconds = float(seconds)

    def to_sec(self):
        return self.seconds


class FakeTime:
    def __init__(self, seconds=0.0):
        self.seconds = float(seconds)

    def __sub__(self, other):
        return FakeDuration(self.seconds - float(other.seconds))


class GraspTaskSequenceTest(unittest.TestCase):
    def _pose(self, x, y=0.0, z=0.20):
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.w = 1.0
        return pose

    def _object(self):
        return types.SimpleNamespace(detected=True, pose_base=self._pose(0.40, 0.0, 0.20))

    def _object_at(self, x, y=0.0, z=0.20):
        return types.SimpleNamespace(detected=True, pose_base=self._pose(x, y, z))

    def _pose_array(self, xs):
        array = PoseArray()
        array.header.frame_id = 'base_link'
        for x in xs:
            array.poses.append(self._pose(x).pose)
        return array

    def test_full_grasp_uses_6d_plan_sequence_when_enabled(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node.latest_obj = None
        node.latest_grasp6d_plan = self._pose_array([0.10, 0.20, 0.30, 0.40])
        node.latest_grasp6d_plan_time = FakeTime(1.0)
        node.active = True
        node._wait_for_motion_settle = lambda reason='motion': calls.append(('settle', reason))
        node.set_state = lambda *args, **kwargs: None

        calls = []

        def fake_service_proxy(name, _srv_type):
            if name == '/supervisor/move_to_pose':
                def move_pose(pose, execute):
                    calls.append(('move', pose.pose.position.x, bool(execute)))
                    return FakeServiceResponse(True, 'moved')
                return move_pose
            if name == '/supervisor/set_gripper':
                def set_gripper(value):
                    calls.append(('set_gripper', float(value)))
                    return FakeServiceResponse(True, 'open')
                return set_gripper
            if name == '/supervisor/compliant_close':
                def close(execute):
                    calls.append(('close', bool(execute)))
                    return FakeServiceResponse(True, 'closed')
                return close
            raise AssertionError('unexpected service %s' % name)

        original_wait_for_service = grasp_task_node.rospy.wait_for_service
        original_service_proxy = grasp_task_node.rospy.ServiceProxy
        original_get_param = grasp_task_node.rospy.get_param
        original_sleep = grasp_task_node.rospy.sleep
        original_time_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.wait_for_service = lambda *args, **kwargs: None
        grasp_task_node.rospy.ServiceProxy = fake_service_proxy
        grasp_task_node.rospy.get_param = lambda name, default=None: {
            '/grasp': {
                'use_grasp6d_plan': True,
                'grasp6d_plan_max_age_sec': 5.0,
                'lift_height_m': 0.05,
            },
            '/gripper': {
                'open_position_m': 0.0,
            },
        }.get(name, default)
        grasp_task_node.rospy.sleep = lambda *_args, **_kwargs: None
        grasp_task_node.rospy.Time.now = staticmethod(lambda: FakeTime(1.0))
        try:
            self.assertTrue(grasp_task_node.GraspTaskNode.execute(node))
        finally:
            grasp_task_node.rospy.wait_for_service = original_wait_for_service
            grasp_task_node.rospy.ServiceProxy = original_service_proxy
            grasp_task_node.rospy.get_param = original_get_param
            grasp_task_node.rospy.sleep = original_sleep
            grasp_task_node.rospy.Time.now = original_time_now

        self.assertEqual(calls[0], ('set_gripper', 0.0))
        move_calls = [call for call in calls if call[0] == 'move']
        self.assertEqual(
            [(round(call[1], 2), call[2]) for call in move_calls],
            [
                (0.10, False), (0.10, True),
                (0.20, False), (0.20, True),
                (0.30, False), (0.30, True),
                (0.40, False), (0.40, True),
            ],
        )
        self.assertIn(('close', True), calls)

    def test_6d_plan_honors_configured_simple_open_and_close_positions(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node.latest_obj = None
        node.latest_grasp6d_plan = self._pose_array([0.10, 0.20, 0.30, 0.40])
        node.latest_grasp6d_plan_time = FakeTime(1.0)
        node.active = True
        node._wait_for_motion_settle = lambda reason='motion': calls.append(('settle', reason))
        node.set_state = lambda *args, **kwargs: None

        calls = []

        def fake_service_proxy(name, _srv_type):
            if name == '/supervisor/move_to_pose':
                def move_pose(pose, execute):
                    calls.append(('move', pose.pose.position.x, bool(execute)))
                    return FakeServiceResponse(True, 'planned')
                return move_pose
            if name == '/supervisor/set_gripper':
                def set_gripper(value):
                    calls.append(('set_gripper', float(value)))
                    return FakeServiceResponse(True, 'ok')
                return set_gripper
            if name == '/supervisor/compliant_close':
                def close(execute):
                    calls.append(('close', bool(execute)))
                    return FakeServiceResponse(True, 'closed')
                return close
            raise AssertionError('unexpected service %s' % name)

        original_wait_for_service = grasp_task_node.rospy.wait_for_service
        original_service_proxy = grasp_task_node.rospy.ServiceProxy
        original_get_param = grasp_task_node.rospy.get_param
        original_sleep = grasp_task_node.rospy.sleep
        original_time_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.wait_for_service = lambda *args, **kwargs: None
        grasp_task_node.rospy.ServiceProxy = fake_service_proxy
        grasp_task_node.rospy.get_param = lambda name, default=None: {
            '/grasp': {
                'use_grasp6d_plan': True,
                'grasp6d_plan_max_age_sec': 5.0,
            },
            '/gripper': {
                'open_position_m': 0.05,
                'close_limit_m': 0.0,
                'use_compliant_close': False,
                'simple_close_position_m': 0.0,
                'simple_close_wait_sec': 0.0,
                'open_wait_sec': 0.0,
            },
        }.get(name, default)
        grasp_task_node.rospy.sleep = lambda *_args, **_kwargs: None
        grasp_task_node.rospy.Time.now = staticmethod(lambda: FakeTime(1.0))
        try:
            self.assertTrue(grasp_task_node.GraspTaskNode.execute(node))
        finally:
            grasp_task_node.rospy.wait_for_service = original_wait_for_service
            grasp_task_node.rospy.ServiceProxy = original_service_proxy
            grasp_task_node.rospy.get_param = original_get_param
            grasp_task_node.rospy.sleep = original_sleep
            grasp_task_node.rospy.Time.now = original_time_now

        self.assertEqual(calls[0], ('set_gripper', 0.05))
        self.assertIn(('set_gripper', 0.0), calls)
        self.assertNotIn(('close', True), calls)

    def test_6d_plan_blocks_execution_when_mujoco_digital_twin_rejects(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node.latest_obj = self._object()
        node.latest_grasp6d_plan = self._pose_array([0.10, 0.20, 0.30, 0.40])
        node.latest_grasp6d_plan_time = FakeTime(1.0)
        node.latest_joint_state = None
        node.active = True
        states = []
        calls = []
        node._wait_for_motion_settle = lambda reason='motion': calls.append(('settle', reason))
        node.set_state = lambda *args, **kwargs: states.append(args)

        class RejectingTwinClient:
            def __init__(self, *args, **kwargs):
                pass

            def simulate_grasp(self, payload):
                calls.append(('simulate', len(payload.get('grasp_sequence_base') or [])))
                return {
                    'simulation_ok': False,
                    'score': 42,
                    'failure_reason': 'gripper would collide with table',
                }

        def fake_service_proxy(name, _srv_type):
            if name == '/supervisor/move_to_pose':
                def move_pose(pose, execute):
                    calls.append(('move', pose.pose.position.x, bool(execute)))
                    return FakeServiceResponse(True, 'planned')
                return move_pose
            if name == '/supervisor/set_gripper':
                def set_gripper(value):
                    calls.append(('set_gripper', float(value)))
                    return FakeServiceResponse(True, 'ok')
                return set_gripper
            if name == '/supervisor/compliant_close':
                return lambda execute: FakeServiceResponse(True, 'closed')
            raise AssertionError('unexpected service %s' % name)

        original_wait_for_service = grasp_task_node.rospy.wait_for_service
        original_service_proxy = grasp_task_node.rospy.ServiceProxy
        original_get_param = grasp_task_node.rospy.get_param
        original_sleep = grasp_task_node.rospy.sleep
        original_time_now = grasp_task_node.rospy.Time.now
        original_client = grasp_task_node.MujocoDigitalTwinClient
        grasp_task_node.rospy.wait_for_service = lambda *args, **kwargs: None
        grasp_task_node.rospy.ServiceProxy = fake_service_proxy
        grasp_task_node.rospy.get_param = lambda name, default=None: {
            '/grasp': {
                'use_grasp6d_plan': True,
                'grasp6d_plan_max_age_sec': 5.0,
            },
            '/gripper': {
                'open_position_m': 0.05,
                'use_compliant_close': False,
            },
            '/mujoco_digital_twin': {
                'enabled': True,
                'execution_gate_enabled': True,
                'server_url': 'http://127.0.0.1:8000',
                'min_score': 80,
                'require_object_pose': True,
                'send_joint_state_in_request': False,
            },
        }.get(name, default)
        grasp_task_node.rospy.sleep = lambda *_args, **_kwargs: None
        grasp_task_node.rospy.Time.now = staticmethod(lambda: FakeTime(1.0))
        grasp_task_node.MujocoDigitalTwinClient = RejectingTwinClient
        try:
            self.assertFalse(grasp_task_node.GraspTaskNode.execute(node))
        finally:
            grasp_task_node.rospy.wait_for_service = original_wait_for_service
            grasp_task_node.rospy.ServiceProxy = original_service_proxy
            grasp_task_node.rospy.get_param = original_get_param
            grasp_task_node.rospy.sleep = original_sleep
            grasp_task_node.rospy.Time.now = original_time_now
            grasp_task_node.MujocoDigitalTwinClient = original_client

        self.assertIn(('simulate', 4), calls)
        self.assertEqual([call for call in calls if call[0] == 'move'], [])
        self.assertEqual(states[-1][0], grasp_task_node.GraspStages.FAILED)
        self.assertIn('gripper would collide with table', states[-1][1])

    def test_6d_plan_rejects_position_only_fallback_before_execute(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node.active = True
        states = []
        calls = []
        node.set_state = lambda *args, **kwargs: states.append(args)
        node._wait_for_motion_settle = lambda reason='motion': calls.append(('settle', reason))

        def move_pose(_pose, execute):
            calls.append(('move', bool(execute)))
            return FakeServiceResponse(
                True,
                'planned with position-only fallback: target xyz=(0.1, 0.2, 0.3)',
            )

        original_get_param = grasp_task_node.rospy.get_param
        grasp_task_node.rospy.get_param = lambda name, default=None: {
            '/robot/position_only_execute_enabled': False,
        }.get(name, default)
        try:
            result = grasp_task_node.GraspTaskNode._plan_and_execute_pose(
                node,
                grasp_task_node.GraspStages.MOVE_PREGRASP,
                '6D pregrasp',
                self._pose(0.10),
                move_pose,
                '6D pregrasp',
            )
        finally:
            grasp_task_node.rospy.get_param = original_get_param

        self.assertFalse(result)
        self.assertEqual(calls, [('move', False)])
        self.assertEqual(states[-1][0], grasp_task_node.GraspStages.FAILED)
        self.assertIn('position-only fallback', states[-1][1])

    def test_6d_plan_rejects_orientation_fallback_before_execute(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node.active = True
        states = []
        calls = []
        node.set_state = lambda *args, **kwargs: states.append(args)
        node._wait_for_motion_settle = lambda reason='motion': calls.append(('settle', reason))

        def move_pose(_pose, execute):
            calls.append(('move', bool(execute)))
            return FakeServiceResponse(
                True,
                'planned with candidate orientation current: target xyz=(0.1, 0.2, 0.3)',
            )

        original_get_param = grasp_task_node.rospy.get_param
        grasp_task_node.rospy.get_param = lambda name, default=None: {
            '/grasp/accept_orientation_fallback': False,
        }.get(name, default)
        try:
            result = grasp_task_node.GraspTaskNode._plan_and_execute_pose(
                node,
                grasp_task_node.GraspStages.MOVE_PREGRASP,
                '6D pregrasp',
                self._pose(0.10),
                move_pose,
                '6D pregrasp',
            )
        finally:
            grasp_task_node.rospy.get_param = original_get_param

        self.assertFalse(result)
        self.assertEqual(calls, [('move', False)])
        self.assertEqual(states[-1][0], grasp_task_node.GraspStages.FAILED)
        self.assertIn('candidate orientation', states[-1][1])

    def test_full_grasp_approaches_target_after_pregrasp_before_closing(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node.latest_obj = self._object()
        node.active = True
        node._lookup_camera_pose_base = lambda: self._pose(0.0, 0.0, 0.20)
        node._current_tool_pose_base = lambda: self._pose(10.0, 0.0, 0.20)
        node._wait_for_motion_settle = lambda reason='motion': calls.append(('settle', reason))
        node.set_state = lambda *args, **kwargs: None

        calls = []

        def fake_service_proxy(name, _srv_type):
            if name == '/supervisor/move_to_pose':
                def move_pose(pose, execute):
                    calls.append(('move', pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, bool(execute)))
                    return FakeServiceResponse(True, 'moved')
                return move_pose
            if name == '/supervisor/set_gripper':
                def set_gripper(value):
                    calls.append(('set_gripper', float(value)))
                    return FakeServiceResponse(True, 'open')
                return set_gripper
            if name == '/supervisor/compliant_close':
                def close(execute):
                    calls.append(('close', bool(execute)))
                    return FakeServiceResponse(True, 'closed')
                return close
            raise AssertionError('unexpected service %s' % name)

        original_wait_for_service = grasp_task_node.rospy.wait_for_service
        original_service_proxy = grasp_task_node.rospy.ServiceProxy
        original_get_param = grasp_task_node.rospy.get_param
        original_sleep = grasp_task_node.rospy.sleep
        original_time_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.wait_for_service = lambda *args, **kwargs: None
        grasp_task_node.rospy.ServiceProxy = fake_service_proxy
        grasp_task_node.rospy.get_param = lambda name, default=None: {
            '/grasp': {
                'pregrasp_distance_m': 0.08,
                'final_approach_offset_m': 0.01,
                'pregrasp_offset_mode': 'camera_ray',
                'lift_height_m': 0.05,
                'pregrasp_reached_tolerance_m': 0.03,
            },
            '/gripper': {
                'open_position_m': 0.0,
            },
        }.get(name, default)
        grasp_task_node.rospy.sleep = lambda *_args, **_kwargs: None
        grasp_task_node.rospy.Time.now = staticmethod(lambda: FakeTime(1.0))
        try:
            self.assertTrue(grasp_task_node.GraspTaskNode.execute(node))
        finally:
            grasp_task_node.rospy.wait_for_service = original_wait_for_service
            grasp_task_node.rospy.ServiceProxy = original_service_proxy
            grasp_task_node.rospy.get_param = original_get_param
            grasp_task_node.rospy.sleep = original_sleep
            grasp_task_node.rospy.Time.now = original_time_now

        self.assertEqual(calls[0][0], 'move')
        self.assertAlmostEqual(calls[0][1], 0.32)
        self.assertFalse(calls[0][4])
        self.assertEqual(calls[1][0], 'move')
        self.assertAlmostEqual(calls[1][1], 0.32)
        self.assertTrue(calls[1][4])
        self.assertEqual(calls[2], ('settle', 'pregrasp'))
        self.assertEqual(calls[3][0], 'set_gripper')
        self.assertEqual(calls[4], ('settle', 'before approach'))
        self.assertEqual(calls[5][0], 'move')
        self.assertAlmostEqual(calls[5][1], 0.39)
        self.assertFalse(calls[5][4])
        self.assertEqual(calls[6][0], 'move')
        self.assertAlmostEqual(calls[6][1], 0.39)
        self.assertTrue(calls[6][4])
        self.assertEqual(calls[7], ('settle', 'approach'))
        self.assertEqual(calls[8], ('close', True))
        self.assertEqual(calls[9][0], 'move')
        self.assertAlmostEqual(calls[9][1], 0.39)
        self.assertAlmostEqual(calls[9][3], 0.25)

    def test_full_grasp_plans_pregrasp_before_executing_it(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node.latest_obj = self._object()
        node.active = True
        node._lookup_camera_pose_base = lambda: self._pose(0.0, 0.0, 0.20)
        node._current_tool_pose_base = lambda: self._pose(10.0, 0.0, 0.20)
        node._wait_for_motion_settle = lambda reason='motion': None
        node.set_state = lambda *args, **kwargs: None

        calls = []

        def fake_service_proxy(name, _srv_type):
            if name == '/supervisor/move_to_pose':
                def move_pose(pose, execute):
                    calls.append(('move', pose.pose.position.x, bool(execute)))
                    return FakeServiceResponse(True, 'moved')
                return move_pose
            if name == '/supervisor/set_gripper':
                return lambda value: FakeServiceResponse(True, 'open')
            if name == '/supervisor/compliant_close':
                return lambda execute: FakeServiceResponse(True, 'closed')
            raise AssertionError('unexpected service %s' % name)

        original_wait_for_service = grasp_task_node.rospy.wait_for_service
        original_service_proxy = grasp_task_node.rospy.ServiceProxy
        original_get_param = grasp_task_node.rospy.get_param
        original_sleep = grasp_task_node.rospy.sleep
        original_time_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.wait_for_service = lambda *args, **kwargs: None
        grasp_task_node.rospy.ServiceProxy = fake_service_proxy
        grasp_task_node.rospy.get_param = lambda name, default=None: {
            '/grasp': {
                'pregrasp_distance_m': 0.08,
                'final_approach_offset_m': 0.01,
                'pregrasp_offset_mode': 'camera_ray',
                'lift_height_m': 0.05,
                'pregrasp_reached_tolerance_m': 0.03,
            },
            '/gripper': {
                'open_position_m': 0.0,
            },
        }.get(name, default)
        grasp_task_node.rospy.sleep = lambda *_args, **_kwargs: None
        grasp_task_node.rospy.Time.now = staticmethod(lambda: FakeTime(1.0))
        try:
            self.assertTrue(grasp_task_node.GraspTaskNode.execute(node))
        finally:
            grasp_task_node.rospy.wait_for_service = original_wait_for_service
            grasp_task_node.rospy.ServiceProxy = original_service_proxy
            grasp_task_node.rospy.get_param = original_get_param
            grasp_task_node.rospy.sleep = original_sleep
            grasp_task_node.rospy.Time.now = original_time_now

        pregrasp_moves = [call for call in calls if call[0] == 'move' and abs(call[1] - 0.32) < 1e-6]
        self.assertEqual([call[2] for call in pregrasp_moves[:2]], [False, True])

    def test_full_grasp_skips_pregrasp_when_tool_is_already_at_pregrasp(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node.latest_obj = self._object()
        node.active = True
        node._lookup_camera_pose_base = lambda: self._pose(0.0, 0.0, 0.20)
        node._current_tool_pose_base = lambda: self._pose(0.321, 0.0, 0.20)
        node._wait_for_motion_settle = lambda reason='motion': calls.append(('settle', reason))
        node.set_state = lambda *args, **kwargs: None

        calls = []

        def fake_service_proxy(name, _srv_type):
            if name == '/supervisor/move_to_pose':
                def move_pose(pose, execute):
                    calls.append(('move', pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, bool(execute)))
                    return FakeServiceResponse(True, 'moved')
                return move_pose
            if name == '/supervisor/set_gripper':
                def set_gripper(value):
                    calls.append(('set_gripper', float(value)))
                    return FakeServiceResponse(True, 'open')
                return set_gripper
            if name == '/supervisor/compliant_close':
                def close(execute):
                    calls.append(('close', bool(execute)))
                    return FakeServiceResponse(True, 'closed')
                return close
            raise AssertionError('unexpected service %s' % name)

        original_wait_for_service = grasp_task_node.rospy.wait_for_service
        original_service_proxy = grasp_task_node.rospy.ServiceProxy
        original_get_param = grasp_task_node.rospy.get_param
        original_sleep = grasp_task_node.rospy.sleep
        original_time_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.wait_for_service = lambda *args, **kwargs: None
        grasp_task_node.rospy.ServiceProxy = fake_service_proxy
        grasp_task_node.rospy.get_param = lambda name, default=None: {
            '/grasp': {
                'pregrasp_distance_m': 0.08,
                'final_approach_offset_m': 0.01,
                'pregrasp_offset_mode': 'camera_ray',
                'lift_height_m': 0.05,
                'pregrasp_reached_tolerance_m': 0.03,
            },
            '/gripper': {
                'open_position_m': 0.0,
            },
        }.get(name, default)
        grasp_task_node.rospy.sleep = lambda *_args, **_kwargs: None
        grasp_task_node.rospy.Time.now = staticmethod(lambda: FakeTime(1.0))
        try:
            self.assertTrue(grasp_task_node.GraspTaskNode.execute(node))
        finally:
            grasp_task_node.rospy.wait_for_service = original_wait_for_service
            grasp_task_node.rospy.ServiceProxy = original_service_proxy
            grasp_task_node.rospy.get_param = original_get_param
            grasp_task_node.rospy.sleep = original_sleep
            grasp_task_node.rospy.Time.now = original_time_now

        pregrasp_moves = [call for call in calls if call[0] == 'move' and abs(call[1] - 0.32) < 1e-6]
        self.assertEqual(pregrasp_moves, [])
        approach_moves = [call for call in calls if call[0] == 'move' and abs(call[1] - 0.39) < 1e-6]
        self.assertEqual([call[4] for call in approach_moves[:2]], [False, True])

    def test_full_grasp_uses_locked_target_when_detection_jumps_after_pregrasp(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node.latest_obj = self._object_at(0.40)
        node.active = True
        node._lookup_camera_pose_base = lambda: self._pose(0.0, 0.0, 0.20)
        node._current_tool_pose_base = lambda: self._pose(10.0, 0.0, 0.20)
        node._wait_for_motion_settle = lambda reason='motion': calls.append(('settle', reason))
        node.set_state = lambda *args, **kwargs: None

        calls = []

        def fake_service_proxy(name, _srv_type):
            if name == '/supervisor/move_to_pose':
                def move_pose(pose, execute):
                    calls.append(('move', pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, bool(execute)))
                    if bool(execute) and abs(pose.pose.position.x - 0.32) < 1e-6:
                        node.latest_obj = self._object_at(-2.0, 0.0, 0.20)
                    return FakeServiceResponse(True, 'moved')
                return move_pose
            if name == '/supervisor/set_gripper':
                def set_gripper(value):
                    calls.append(('set_gripper', float(value)))
                    return FakeServiceResponse(True, 'open')
                return set_gripper
            if name == '/supervisor/compliant_close':
                def close(execute):
                    calls.append(('close', bool(execute)))
                    return FakeServiceResponse(True, 'closed')
                return close
            raise AssertionError('unexpected service %s' % name)

        original_wait_for_service = grasp_task_node.rospy.wait_for_service
        original_service_proxy = grasp_task_node.rospy.ServiceProxy
        original_get_param = grasp_task_node.rospy.get_param
        original_sleep = grasp_task_node.rospy.sleep
        original_time_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.wait_for_service = lambda *args, **kwargs: None
        grasp_task_node.rospy.ServiceProxy = fake_service_proxy
        grasp_task_node.rospy.get_param = lambda name, default=None: {
            '/grasp': {
                'pregrasp_distance_m': 0.08,
                'final_approach_offset_m': 0.01,
                'pregrasp_offset_mode': 'camera_ray',
                'lift_height_m': 0.05,
                'pregrasp_reached_tolerance_m': 0.03,
                'max_locked_target_refine_m': 0.06,
            },
            '/gripper': {
                'open_position_m': 0.0,
            },
        }.get(name, default)
        grasp_task_node.rospy.sleep = lambda *_args, **_kwargs: None
        grasp_task_node.rospy.Time.now = staticmethod(lambda: FakeTime(1.0))
        try:
            self.assertTrue(grasp_task_node.GraspTaskNode.execute(node))
        finally:
            grasp_task_node.rospy.wait_for_service = original_wait_for_service
            grasp_task_node.rospy.ServiceProxy = original_service_proxy
            grasp_task_node.rospy.get_param = original_get_param
            grasp_task_node.rospy.sleep = original_sleep
            grasp_task_node.rospy.Time.now = original_time_now

        approach_moves = [
            call for call in calls
            if call[0] == 'move' and bool(call[4]) is False and abs(call[1] - 0.39) < 1e-6
        ]
        self.assertEqual(len(approach_moves), 1)
        self.assertAlmostEqual(approach_moves[0][1], 0.39)
        self.assertNotAlmostEqual(approach_moves[0][1], -1.99)


if __name__ == '__main__':
    unittest.main()
