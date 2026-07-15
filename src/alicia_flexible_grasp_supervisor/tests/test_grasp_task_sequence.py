#!/usr/bin/env python3
import importlib.util
import pathlib
import sys
import types
import unittest

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import JointState


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

    def _joint_state(self):
        msg = JointState()
        msg.name = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6', 'right_finger']
        msg.position = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.05]
        return msg

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
            if name in ('/supervisor/move_to_pose', '/supervisor/move_to_pose_linear'):
                def move_pose(pose, execute):
                    mode = 'linear' if name.endswith('_linear') else 'joint'
                    calls.append((mode, pose.pose.position.x, bool(execute)))
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
        move_calls = [call for call in calls if call[0] in ('joint', 'linear')]
        self.assertEqual(
            [(call[0], round(call[1], 2), call[2]) for call in move_calls],
            [
                ('joint', 0.10, False), ('joint', 0.10, True),
                ('linear', 0.20, False), ('linear', 0.20, True),
                ('linear', 0.30, False), ('linear', 0.30, True),
                ('linear', 0.40, False), ('linear', 0.40, True),
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
            if name in ('/supervisor/move_to_pose', '/supervisor/move_to_pose_linear'):
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
        node.latest_joint_state = self._joint_state()
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
                calls.append(('joint_payload', len(payload.get('joint_names') or [])))
                return {
                    'simulation_ok': False,
                    'score': 42,
                    'failure_reason': 'gripper would collide with table',
                }

        def fake_service_proxy(name, _srv_type):
            if name in ('/supervisor/move_to_pose', '/supervisor/move_to_pose_linear'):
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
                'server_url': 'http://172.23.132.97:8000',
                'min_score': 80,
                'require_object_pose': True,
                'send_joint_state_in_request': True,
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
        self.assertIn(('joint_payload', 7), calls)
        self.assertEqual([call for call in calls if call[0] == 'move'], [])
        self.assertEqual(states[-1][0], grasp_task_node.GraspStages.FAILED)
        self.assertIn('gripper would collide with table', states[-1][1])

    def test_6d_plan_blocks_mujoco_simulation_when_joint_state_payload_required_but_missing(self):
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

        def fake_service_proxy(name, _srv_type):
            if name in ('/supervisor/move_to_pose', '/supervisor/move_to_pose_linear'):
                def move_pose(pose, execute):
                    calls.append(('move', bool(execute)))
                    return FakeServiceResponse(True, 'planned')
                return move_pose
            if name == '/supervisor/set_gripper':
                return lambda value: FakeServiceResponse(True, 'ok')
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
                'server_url': 'http://172.23.132.97:8000',
                'require_object_pose': True,
                'send_joint_state_in_request': True,
            },
        }.get(name, default)
        grasp_task_node.rospy.sleep = lambda *_args, **_kwargs: None
        grasp_task_node.rospy.Time.now = staticmethod(lambda: FakeTime(1.0))
        try:
            self.assertFalse(grasp_task_node.GraspTaskNode.execute(node))
        finally:
            grasp_task_node.rospy.wait_for_service = original_wait_for_service
            grasp_task_node.rospy.ServiceProxy = original_service_proxy
            grasp_task_node.rospy.get_param = original_get_param
            grasp_task_node.rospy.sleep = original_sleep
            grasp_task_node.rospy.Time.now = original_time_now

        self.assertEqual([call for call in calls if call[0] == 'move'], [])
        self.assertEqual(states[-1][0], grasp_task_node.GraspStages.FAILED)
        self.assertIn('no /joint_states', states[-1][1])

    def test_6d_plan_rejects_execution_when_locked_object_drifted(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node.latest_grasp6d_plan = self._pose_array([0.10, 0.20, 0.30, 0.40])
        node.latest_grasp6d_plan_time = FakeTime(1.0)
        node.latest_grasp6d_plan_object = self._object_at(0.40, 0.0, 0.20)
        node.latest_obj = self._object_at(0.46, 0.0, 0.20)

        original_get_param = grasp_task_node.rospy.get_param
        original_time_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.get_param = lambda name, default=None: default
        grasp_task_node.rospy.Time.now = staticmethod(lambda: FakeTime(2.0))
        try:
            result = grasp_task_node.GraspTaskNode._fresh_grasp6d_plan(
                node,
                {
                    'grasp6d_plan_max_age_sec': 180.0,
                    'grasp6d_plan_max_object_drift_m': 0.03,
                },
            )
        finally:
            grasp_task_node.rospy.get_param = original_get_param
            grasp_task_node.rospy.Time.now = original_time_now

        self.assertIsNone(result)

    def test_6d_plan_remains_fresh_with_longer_manual_confirmation_window(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        plan = self._pose_array([0.10, 0.20, 0.30, 0.40])
        node.latest_grasp6d_plan = plan
        node.latest_grasp6d_plan_time = FakeTime(1.0)
        node.latest_grasp6d_plan_object = self._object_at(0.40, 0.0, 0.20)
        node.latest_obj = self._object_at(0.41, 0.0, 0.20)

        original_get_param = grasp_task_node.rospy.get_param
        original_time_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.get_param = lambda name, default=None: default
        grasp_task_node.rospy.Time.now = staticmethod(lambda: FakeTime(121.0))
        try:
            result = grasp_task_node.GraspTaskNode._fresh_grasp6d_plan(
                node,
                {
                    'grasp6d_plan_max_age_sec': 180.0,
                    'grasp6d_plan_max_object_drift_m': 0.03,
                },
            )
        finally:
            grasp_task_node.rospy.get_param = original_get_param
            grasp_task_node.rospy.Time.now = original_time_now

        self.assertIs(result, plan)

    def test_negative_object_invalidation_removes_cached_targets_and_plan_without_motion(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node.active = True
        node.latest_obj = None
        node.latest_obj_time = None
        node.latest_visual_obj = None
        node.latest_visual_obj_time = None
        node.latest_grasp6d_plan = None
        node.latest_grasp6d_plan_time = None
        node.latest_grasp6d_plan_object = None

        original_get_param = grasp_task_node.rospy.get_param
        original_time_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.get_param = lambda name, default=None: default
        grasp_task_node.rospy.Time.now = staticmethod(lambda: FakeTime(1.0))
        try:
            detected = self._object_at(0.40, 0.0, 0.20)
            grasp_task_node.GraspTaskNode.obj_cb(node, detected)
            locked_for_active_flow = grasp_task_node.deepcopy(node.latest_obj)
            grasp_task_node.GraspTaskNode.grasp6d_plan_cb(node, self._pose_array([0.10, 0.20, 0.30, 0.40]))
            grasp_task_node.GraspTaskNode.obj_cb(node, types.SimpleNamespace(detected=False))
            available_plan = grasp_task_node.GraspTaskNode._fresh_grasp6d_plan(
                node,
                {'grasp6d_plan_max_age_sec': 180.0},
            )
        finally:
            grasp_task_node.rospy.get_param = original_get_param
            grasp_task_node.rospy.Time.now = original_time_now

        self.assertIsNone(node.latest_obj)
        self.assertIsNone(node.latest_obj_time)
        self.assertIsNone(node.latest_visual_obj)
        self.assertIsNone(node.latest_grasp6d_plan)
        self.assertIsNone(available_plan)
        active_target = grasp_task_node.GraspTaskNode._target_for_approach(
            node,
            locked_for_active_flow,
            {},
        )
        self.assertTrue(active_target.detected)
        self.assertAlmostEqual(active_target.pose_base.pose.position.x, 0.40)

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

    def test_6d_motion_stops_when_real_joint_feedback_does_not_settle(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node.active = True
        states = []
        calls = []
        node.set_state = lambda *args, **kwargs: states.append(args)
        node._wait_for_motion_settle = lambda _reason='motion': False

        def move_pose(_pose, execute):
            calls.append(bool(execute))
            return FakeServiceResponse(True, 'ok')

        self.assertFalse(
            node._plan_and_execute_pose(
                grasp_task_node.GraspStages.MOVE_PREGRASP,
                '6D pregrasp',
                self._pose(0.10),
                move_pose,
                '6D pregrasp',
            )
        )
        self.assertEqual(calls, [False, True])
        self.assertEqual(states[-1][0], grasp_task_node.GraspStages.FAILED)
        self.assertIn('did not settle', states[-1][1])

    def test_visual_retarget_translates_remaining_path_and_preserves_6d_rotation(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node.active = True
        node.set_state = lambda *args, **kwargs: None
        reference = self._object_at(0.40, -0.10, 0.20)
        live = self._object_at(0.41, -0.08, 0.19)
        reference.label = 'mouse'
        live.label = 'mouse'
        node._wait_for_stable_visual_target = lambda *_args, **_kwargs: live
        poses = [self._pose(0.20, 0.10, 0.30), self._pose(0.30, 0.20, 0.40)]
        poses[0].pose.orientation.x = 0.25
        poses[0].pose.orientation.w = 0.75

        result = node._visual_retarget_6d_poses(
            reference,
            poses,
            {
                'visual_retarget_enabled': True,
                'visual_retarget_max_correction_m': 0.04,
                'visual_retarget_deadband_m': 0.0,
            },
            'pregrasp',
            required=True,
        )

        self.assertIsNotNone(result)
        shifted, updated_reference = result
        self.assertIs(updated_reference, live)
        self.assertAlmostEqual(shifted[0].pose.position.x, 0.21)
        self.assertAlmostEqual(shifted[0].pose.position.y, 0.12)
        self.assertAlmostEqual(shifted[0].pose.position.z, 0.29)
        self.assertAlmostEqual(shifted[1].pose.position.x, 0.31)
        self.assertAlmostEqual(shifted[0].pose.orientation.x, 0.25)
        self.assertAlmostEqual(shifted[0].pose.orientation.w, 0.75)

    def test_visual_retarget_rejects_large_handeye_inconsistency(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node.active = True
        states = []
        node.set_state = lambda *args, **kwargs: states.append(args)
        reference = self._object_at(0.40, 0.0, 0.20)
        live = self._object_at(0.47, 0.0, 0.20)
        node._wait_for_stable_visual_target = lambda *_args, **_kwargs: live

        result = node._visual_retarget_6d_poses(
            reference,
            [self._pose(0.20)],
            {
                'visual_retarget_enabled': True,
                'visual_retarget_max_correction_m': 0.04,
            },
            'pregrasp',
            required=True,
        )

        self.assertIsNone(result)
        self.assertEqual(states[-1][0], grasp_task_node.GraspStages.FAILED)
        self.assertIn('hand-eye consistency failed', states[-1][1])

    def test_visual_retarget_accepts_close_range_detection_below_plan_threshold(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node.active = True
        node.latest_obj = self._object_at(0.40, 0.0, 0.20)
        node.latest_obj.label = 'mouse'
        node.latest_obj.confidence = 0.90
        node.latest_obj_time = FakeTime(0.5)
        node.latest_visual_obj = self._object_at(0.41, 0.0, 0.20)
        node.latest_visual_obj.label = 'mouse'
        node.latest_visual_obj.confidence = 0.40
        node.latest_visual_obj_time = FakeTime(1.0)
        node.latest_raw_detection = True
        node.latest_raw_detection_time = FakeTime(1.0)

        original_time_now = grasp_task_node.rospy.Time.now
        original_is_shutdown = grasp_task_node.rospy.is_shutdown
        grasp_task_node.rospy.Time.now = staticmethod(lambda: FakeTime(1.0))
        grasp_task_node.rospy.is_shutdown = lambda: False
        try:
            result = node._wait_for_stable_visual_target(
                node.latest_obj,
                {
                    'min_object_confidence': 0.50,
                    'visual_retarget_min_object_confidence': 0.35,
                    'visual_retarget_timeout_sec': 0.2,
                    'visual_retarget_required_samples': 1,
                    'visual_retarget_raw_max_age_sec': 0.30,
                },
                'pregrasp',
            )
        finally:
            grasp_task_node.rospy.Time.now = original_time_now
            grasp_task_node.rospy.is_shutdown = original_is_shutdown

        self.assertIsNotNone(result)
        self.assertAlmostEqual(result.confidence, 0.40)
        self.assertAlmostEqual(result.pose_base.pose.position.x, 0.41)

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
            if name in ('/supervisor/move_to_pose', '/supervisor/move_to_pose_linear'):
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
            if name in ('/supervisor/move_to_pose', '/supervisor/move_to_pose_linear'):
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
            if name in ('/supervisor/move_to_pose', '/supervisor/move_to_pose_linear'):
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
            if name in ('/supervisor/move_to_pose', '/supervisor/move_to_pose_linear'):
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
            if (
                call[0] == 'move'
                and bool(call[4]) is False
                and abs(call[1] - 0.39) < 1e-6
                and abs(call[3] - 0.20) < 1e-6
            )
        ]
        self.assertEqual(len(approach_moves), 1)
        self.assertAlmostEqual(approach_moves[0][1], 0.39)
        self.assertNotAlmostEqual(approach_moves[0][1], -1.99)


if __name__ == '__main__':
    unittest.main()
