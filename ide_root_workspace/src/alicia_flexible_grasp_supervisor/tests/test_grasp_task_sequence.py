#!/usr/bin/env python3
import hashlib
import io
import importlib.util
import json
import os
import pathlib
import sys
import tempfile
import threading
import types
import unittest

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import JointState
from alicia_flexible_grasp_supervisor.msg import Grasp6DPlan


ROOT = pathlib.Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / 'src'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id

SCRIPT = ROOT / 'scripts' / 'grasp_task_node.py'
spec = importlib.util.spec_from_file_location('grasp_task_node', str(SCRIPT))
grasp_task_node = importlib.util.module_from_spec(spec)
spec.loader.exec_module(grasp_task_node)


_MISSING_CONFIG_VALUE = object()


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
    def setUp(self):
        self._mujoco_audit_directory = tempfile.TemporaryDirectory()
        self._mujoco_audit_sequence = 0
        self._mujoco_audit_path = os.path.join(
            self._mujoco_audit_directory.name,
            'mujoco-audit.json',
        )

    def tearDown(self):
        self._mujoco_audit_directory.cleanup()

    def _next_mujoco_audit_path(self):
        self._mujoco_audit_sequence += 1
        return os.path.join(
            self._mujoco_audit_directory.name,
            'mujoco-audit-%d.json' % self._mujoco_audit_sequence,
        )

    def _pose(self, x, y=0.0, z=0.20):
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.w = 1.0
        return pose

    def _object(self, stamp_sec=1.0):
        return self._object_at(0.40, 0.0, 0.20, stamp_sec=stamp_sec)

    def _object_at(self, x, y=0.0, z=0.20, stamp_sec=1.0):
        stamp = grasp_task_node.rospy.Time.from_sec(float(stamp_sec))
        pose_base = self._pose(x, y, z)
        pose_base.header.stamp = stamp
        return types.SimpleNamespace(
            header=types.SimpleNamespace(stamp=stamp),
            detected=True,
            label='carton',
            pose_base=pose_base,
        )

    def _pose_array(self, xs):
        array = PoseArray()
        array.header.frame_id = 'base_link'
        for x in xs:
            array.poses.append(self._pose(x).pose)
        return array

    def _rich_plan(self, xs=None, plan_id='plan-a', stamp_sec=1.0):
        xs = [0.10, 0.20, 0.30, 0.40] if xs is None else list(xs)
        source_stamp_sec = float(stamp_sec)
        canonical_stamp_sec = source_stamp_sec if source_stamp_sec > 0.0 else 1.0
        plan = Grasp6DPlan()
        plan.header.frame_id = 'base_link'
        plan.header.stamp = grasp_task_node.rospy.Time.from_sec(canonical_stamp_sec)
        plan.valid = True
        plan.score = 0.9
        plan.candidate_width_m = 0.039
        plan.required_open_width_m = 0.044
        plan.model_choice = 'carton_segment:' + str(plan_id)
        plan.poses = [self._pose(x).pose for x in xs]
        geometry = plan.object_geometry
        geometry.header.frame_id = plan.header.frame_id
        geometry.header.stamp = plan.header.stamp
        geometry.valid = True
        geometry.label = 'carton'
        geometry.source_mode = 'instance_mask'
        geometry.pose_base.position.x = 0.40
        geometry.pose_base.position.y = 0.0
        geometry.pose_base.position.z = 0.20
        geometry.pose_base.orientation.w = 1.0
        geometry.size_xyz_m.x = 0.08
        geometry.size_xyz_m.y = 0.04
        geometry.size_xyz_m.z = 0.06
        geometry.support_normal_base.z = 1.0
        plan.plan_id = compute_plan_id(plan)
        if source_stamp_sec <= 0.0:
            plan.header.stamp = grasp_task_node.rospy.Time.from_sec(
                source_stamp_sec
            )
            plan.object_geometry.header.stamp = (
                grasp_task_node.rospy.Time.from_sec(source_stamp_sec)
            )
        return plan

    def _joint_state(self):
        msg = JointState()
        msg.name = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6', 'right_finger']
        msg.position = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.05]
        return msg

    @staticmethod
    def _passing_mujoco_response(plan_id):
        return {
            'plan_id': plan_id,
            'score': 95.0,
            'simulation_ok': True,
            'ik_success': True,
            'collision_free': True,
            'contact_success': True,
            'lift_success': True,
        }

    def _run_mujoco_gate_to_first_physical_action(
        self,
        response=None,
        request_error=None,
        during_request=None,
        allow_execution_on_error=False,
        audit_output_path=None,
        payload_build_error=None,
        position_only_execute_enabled=False,
        mujoco_enabled=True,
        execution_gate_enabled=True,
        planning_audit_output_path=None,
    ):
        node = grasp_task_node.GraspTaskNode.__new__(
            grasp_task_node.GraspTaskNode
        )
        plan = self._rich_plan(stamp_sec=1.0)
        node.latest_grasp6d_plan = plan
        node.latest_obj = self._object(stamp_sec=1.0)
        node.latest_obj_time = grasp_task_node.rospy.Time.from_sec(1.0)
        node.latest_joint_state = self._joint_state()
        node.active = True
        states = []
        physical_actions = []
        payloads = []
        node.set_state = lambda *args, **kwargs: states.append(args)
        node._command_gripper_position = lambda *args, **kwargs: (
            physical_actions.append('open-gripper') or False
        )
        configured_audit_path = (
            self._next_mujoco_audit_path()
            if audit_output_path is None
            else audit_output_path
        )
        configured_planning_audit_path = (
            os.path.join(
                self._mujoco_audit_directory.name,
                'planning-gate-audit.json',
            )
            if planning_audit_output_path is None
            else planning_audit_output_path
        )

        class TwinClient:
            def __init__(self, *args, **kwargs):
                pass

            def simulate_grasp(self, payload):
                payloads.append(payload)
                if during_request is not None:
                    during_request(node, plan)
                if request_error is not None:
                    raise request_error
                if callable(response):
                    return response(plan)
                return response

        original_get_param = grasp_task_node.rospy.get_param
        original_time_now = grasp_task_node.rospy.Time.now
        original_client = grasp_task_node.MujocoDigitalTwinClient
        original_payload_builder = grasp_task_node.build_mujoco_payload
        twin_config = {
            'audit_output_path': configured_audit_path,
            'server_url': 'http://172.23.132.97:8000',
            'timeout_sec': 0.01,
            'min_score': 80,
            'allow_execution_on_error': bool(allow_execution_on_error),
            'require_object_pose': True,
            'send_joint_state_in_request': True,
            'object_model': {
                'type': 'carton_box',
                'mass_kg': 0.08,
                'friction': [1.2, 0.08, 0.02],
            },
            'gripper_model': {
                'name': 'Alicia_D_v5_6_gripper_50mm',
                'max_inner_gap_m': 0.050,
            },
        }
        if mujoco_enabled is not _MISSING_CONFIG_VALUE:
            twin_config['enabled'] = mujoco_enabled
        if execution_gate_enabled is not _MISSING_CONFIG_VALUE:
            twin_config['execution_gate_enabled'] = execution_gate_enabled
        grasp_task_node.rospy.get_param = lambda name, default=None: {
            '/robot/position_only_execute_enabled': bool(
                position_only_execute_enabled
            ),
            '/grasp_6d/remote/gate_audit_output_path': (
                configured_planning_audit_path
            ),
            '/mujoco_digital_twin': twin_config,
        }.get(name, default)
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: FakeTime(1.0)
        )
        grasp_task_node.MujocoDigitalTwinClient = TwinClient
        if payload_build_error is not None:
            def fail_payload_build(*_args, **_kwargs):
                raise payload_build_error
            grasp_task_node.build_mujoco_payload = fail_payload_build
        try:
            result = node._execute_grasp6d_plan(
                {'plan_validity_sec': 5.0},
                {'open_position_m': 0.050, 'use_compliant_close': False},
                0.050,
                None,
                None,
                None,
                None,
                plan,
                strict_execute_pose=lambda *_args, **_kwargs: FakeServiceResponse(
                    True,
                    'strict cached executed',
                ),
            )
        finally:
            grasp_task_node.rospy.get_param = original_get_param
            grasp_task_node.rospy.Time.now = original_time_now
            grasp_task_node.MujocoDigitalTwinClient = original_client
            grasp_task_node.build_mujoco_payload = original_payload_builder
        node._test_mujoco_audit_path = configured_audit_path
        node._test_mujoco_audit_bytes = None
        node._test_mujoco_audit = None
        if isinstance(configured_audit_path, str) and os.path.isfile(
            configured_audit_path
        ):
            with open(configured_audit_path, 'rb') as handle:
                node._test_mujoco_audit_bytes = handle.read()
            node._test_mujoco_audit = json.loads(
                node._test_mujoco_audit_bytes.decode('utf-8')
            )
        return result, physical_actions, states, payloads, node, plan

    def test_legacy_pose_array_is_never_execution_authority(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node.latest_grasp6d_plan = None
        node.latest_grasp6d_legacy_plan = None

        node.grasp6d_legacy_plan_cb(
            self._pose_array([0.10, 0.20, 0.30, 0.40])
        )

        self.assertIsNone(node.latest_grasp6d_plan)
        self.assertEqual(len(node.latest_grasp6d_legacy_plan.poses), 4)

    def test_rich_callback_deep_copies_and_replacement_invalidates_old_id(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node.latest_grasp6d_plan = None
        original_get_param = grasp_task_node.rospy.get_param
        original_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.get_param = lambda name, default=None: {
            '/grasp_6d/plan_validity_sec': 2.0,
        }.get(name, default)
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(2.0)
        )
        try:
            first = self._rich_plan(plan_id='first', stamp_sec=1.0)
            first_id = first.plan_id
            node.grasp6d_plan_cb(first)
            first.poses[0].position.x = 99.0
            self.assertAlmostEqual(node.latest_grasp6d_plan.poses[0].position.x, 0.10)

            second = self._rich_plan(plan_id='second', stamp_sec=1.5)
            node.grasp6d_plan_cb(second)
            old = node.validate_plan_id_for_execution(first_id)
            current = node.validate_plan_id_for_execution(second.plan_id)
        finally:
            grasp_task_node.rospy.get_param = original_get_param
            grasp_task_node.rospy.Time.now = original_now

        self.assertFalse(old.ok)
        self.assertEqual(old.code, 'PLAN_REPLACED')
        self.assertTrue(current.ok)

    def test_start_service_rejects_old_clicked_id_after_new_plan_is_cached(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node.active = False
        node.latest_grasp6d_plan = self._rich_plan(
            plan_id='new-plan',
            stamp_sec=9.5,
        )
        actions = []
        node.execute = lambda *args, **kwargs: actions.append((args, kwargs)) or True
        node.set_state = lambda *args, **kwargs: None
        request = types.SimpleNamespace(execute=True, plan_id='old-clicked-plan')
        original_get_param = grasp_task_node.rospy.get_param
        grasp_task_node.rospy.get_param = lambda name, default=None: {
            '/grasp': {'use_grasp6d_plan': True, 'plan_validity_sec': 2.0},
        }.get(name, default)
        try:
            response = node.start_cb(request)
        finally:
            grasp_task_node.rospy.get_param = original_get_param

        self.assertFalse(response.success)
        self.assertIn('PLAN_', response.message)
        self.assertEqual(actions, [])

    def test_start_service_bound_copy_cannot_switch_to_replacement_plan(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node.active = False
        old_plan = self._rich_plan(plan_id='old-click', stamp_sec=9.0)
        new_plan = self._rich_plan(plan_id='new-cache', stamp_sec=9.5)
        node.latest_grasp6d_plan = old_plan
        node.latest_obj = self._object()
        node.latest_obj_time = grasp_task_node.rospy.Time.from_sec(9.9)
        node.set_state = lambda *args, **kwargs: None
        actions = []

        def replace_before_execution(grasp6d_plan=None):
            self.assertIsNotNone(grasp6d_plan)
            self.assertEqual(grasp6d_plan.plan_id, old_plan.plan_id)
            node.latest_grasp6d_plan = new_plan
            if node._execution_checkpoint(
                grasp6d_plan,
                {'plan_validity_sec': 2.0, 'target_max_drift_m': 0.02},
                'race probe',
            ):
                actions.append('motion')
            return False

        node.execute = replace_before_execution
        request = types.SimpleNamespace(execute=True, plan_id=old_plan.plan_id)
        original_get_param = grasp_task_node.rospy.get_param
        original_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.get_param = lambda name, default=None: {
            '/grasp': {
                'use_grasp6d_plan': True,
                'plan_validity_sec': 2.0,
                'target_max_drift_m': 0.02,
            },
        }.get(name, default)
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(10.0)
        )
        try:
            response = node.start_cb(request)
        finally:
            grasp_task_node.rospy.get_param = original_get_param
            grasp_task_node.rospy.Time.now = original_now

        self.assertFalse(response.success)
        self.assertEqual(actions, [])
        self.assertEqual(node.latest_grasp6d_plan.plan_id, new_plan.plan_id)

    def test_start_service_execute_flag_and_non6d_empty_id_remain_compatible(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node.active = False
        node.set_state = lambda *args, **kwargs: None
        actions = []
        node.execute = lambda grasp6d_plan=None: actions.append(grasp6d_plan) or True
        original_get_param = grasp_task_node.rospy.get_param
        grasp_config = {'use_grasp6d_plan': False}
        grasp_task_node.rospy.get_param = lambda name, default=None: {
            '/grasp': grasp_config,
        }.get(name, default)
        try:
            disabled = node.start_cb(
                types.SimpleNamespace(execute=False, plan_id='')
            )
            enabled = node.start_cb(
                types.SimpleNamespace(execute=True, plan_id='')
            )
            grasp_config['use_grasp6d_plan'] = True
            missing_id = node.start_cb(
                types.SimpleNamespace(execute=True, plan_id='')
            )
        finally:
            grasp_task_node.rospy.get_param = original_get_param

        self.assertFalse(disabled.success)
        self.assertTrue(enabled.success)
        self.assertFalse(missing_id.success)
        self.assertIn('PLAN_ID_MISSING', missing_id.message)
        self.assertEqual(actions, [None])

    def test_stop_does_not_release_start_execution_slot(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node.active = False
        node.set_state = lambda *args, **kwargs: None
        entered = threading.Event()
        release = threading.Event()
        calls = []
        active_seen_by_old_execution = []

        def execute(grasp6d_plan=None):
            calls.append(grasp6d_plan)
            if len(calls) == 1:
                entered.set()
                release.wait(2.0)
                active_seen_by_old_execution.append(node.active)
            return False

        node.execute = execute
        request = types.SimpleNamespace(execute=True, plan_id='')
        original_get_param = grasp_task_node.rospy.get_param
        grasp_task_node.rospy.get_param = lambda name, default=None: {
            '/grasp': {'use_grasp6d_plan': False},
        }.get(name, default)
        responses = []
        first = threading.Thread(target=lambda: responses.append(node.start_cb(request)))
        try:
            first.start()
            self.assertTrue(entered.wait(1.0))
            node.stop_cb(types.SimpleNamespace(emergency=False))
            second = node.start_cb(request)
            self.assertFalse(second.success)
            self.assertIn('active', second.message.lower())
            self.assertEqual(len(calls), 1)
            release.set()
            first.join(2.0)
        finally:
            release.set()
            first.join(2.0)
            grasp_task_node.rospy.get_param = original_get_param

        self.assertFalse(first.is_alive())
        self.assertEqual(active_seen_by_old_execution, [False])
        self.assertFalse(node.active)

    def test_stop_waits_for_inflight_plan_bound_action_commit(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node.active = True
        node._start_inflight = True
        plan = self._rich_plan(stamp_sec=9.0)
        node.latest_grasp6d_plan = plan
        node.latest_obj = self._object(stamp_sec=9.9)
        node.latest_obj_time = grasp_task_node.rospy.Time.from_sec(9.9)
        node.set_state = lambda *args, **kwargs: None
        action_entered = threading.Event()
        action_release = threading.Event()
        stop_done = threading.Event()
        action_result = []
        order = []

        def action():
            action_entered.set()
            action_release.wait(2.0)
            order.append('action')
            return FakeServiceResponse(True, 'committed')

        def invoke():
            action_result.append(
                node._invoke_plan_bound_action(
                    plan,
                    {
                        'target_max_drift_m': 0.02,
                        'target_observation_validity_sec': 1.5,
                    },
                    'threaded action',
                    action,
                )
            )

        def stop():
            node.stop_cb(types.SimpleNamespace(emergency=False))
            order.append('stop')
            stop_done.set()

        original_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(10.0)
        )
        action_thread = threading.Thread(target=invoke)
        stop_thread = threading.Thread(target=stop)
        try:
            action_thread.start()
            self.assertTrue(action_entered.wait(1.0))
            stop_thread.start()
            stop_returned_before_commit = stop_done.wait(0.05)
            action_release.set()
            action_thread.join(2.0)
            stop_thread.join(2.0)
        finally:
            action_release.set()
            action_thread.join(2.0)
            if stop_thread.ident is not None:
                stop_thread.join(2.0)
            grasp_task_node.rospy.Time.now = original_now

        self.assertFalse(stop_returned_before_commit)
        self.assertFalse(action_thread.is_alive())
        self.assertFalse(stop_thread.is_alive())
        self.assertTrue(action_result[0][0].ok)
        self.assertEqual(order, ['action', 'stop'])
        self.assertFalse(node.active)
        self.assertTrue(node._start_inflight)

    def test_stop_before_plan_bound_actions_cancels_move_open_and_close(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node.active = True
        node._start_inflight = True
        plan = self._rich_plan(stamp_sec=9.0)
        node.latest_grasp6d_plan = plan
        node.latest_obj = self._object(stamp_sec=9.9)
        node.latest_obj_time = grasp_task_node.rospy.Time.from_sec(9.9)
        node.set_state = lambda *args, **kwargs: None
        node._wait_for_motion_settle = lambda *_args, **_kwargs: True
        calls = []
        stop_thread = threading.Thread(
            target=lambda: node.stop_cb(types.SimpleNamespace(emergency=False))
        )
        stop_thread.start()
        stop_thread.join(2.0)
        self.assertFalse(stop_thread.is_alive())

        validation, response = node._invoke_plan_bound_action(
            plan,
            {},
            'cancel probe',
            lambda: calls.append('direct') or FakeServiceResponse(True),
        )
        self.assertFalse(validation.ok)
        self.assertEqual(validation.code, 'EXECUTION_CANCELLED')
        self.assertIsNone(response)
        self.assertFalse(
            node._plan_and_execute_pose(
                grasp_task_node.GraspStages.MOVE_PREGRASP,
                'cancelled move',
                self._pose(0.10),
                lambda _pose, _execute: (
                    calls.append('move') or FakeServiceResponse(True)
                ),
                'cancelled move',
                execution_plan=plan,
                gcfg={},
            )
        )
        self.assertFalse(
            node._command_gripper_position(
                lambda _position: (
                    calls.append('open') or FakeServiceResponse(True)
                ),
                0.05,
                'cancelled open',
                0.0,
                execution_plan=plan,
                gcfg={},
            )
        )
        closed, message = node._close_gripper(
            {'use_compliant_close': True},
            lambda _position: FakeServiceResponse(True),
            lambda **_kwargs: (
                calls.append('close') or FakeServiceResponse(True)
            ),
            execution_plan=plan,
            gcfg={},
        )
        self.assertFalse(closed)
        self.assertIn('EXECUTION_CANCELLED', message)
        self.assertEqual(calls, [])
        self.assertTrue(node._start_inflight)

    def test_start_grasp_service_contract_includes_plan_id(self):
        service = ROOT / 'srv' / 'StartGrasp.srv'
        request_fields = service.read_text(encoding='utf-8').split('---', 1)[0]
        self.assertIn('string plan_id', request_fields.splitlines())

    def test_stale_future_or_malformed_rich_callback_clears_authority(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node.latest_grasp6d_plan = None
        original_get_param = grasp_task_node.rospy.get_param
        original_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.get_param = lambda name, default=None: {
            '/grasp_6d/plan_validity_sec': 2.0,
        }.get(name, default)
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(10.0)
        )
        try:
            for plan in (
                self._rich_plan(plan_id='stale', stamp_sec=7.0),
                self._rich_plan(plan_id='future', stamp_sec=10.1),
                self._rich_plan(plan_id='zero', stamp_sec=0.0),
            ):
                node.grasp6d_plan_cb(plan)
                self.assertIsNone(node.latest_grasp6d_plan)

            malformed = self._rich_plan(plan_id='malformed', stamp_sec=9.0)
            malformed.poses[2].orientation.w = 0.0
            node.grasp6d_plan_cb(malformed)
        finally:
            grasp_task_node.rospy.get_param = original_get_param
            grasp_task_node.rospy.Time.now = original_now

        self.assertIsNone(node.latest_grasp6d_plan)

    def test_older_source_stamp_replay_clears_execution_authority(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node.latest_grasp6d_plan = None
        original_get_param = grasp_task_node.rospy.get_param
        original_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.get_param = lambda name, default=None: {
            '/grasp_6d/plan_validity_sec': 2.0,
        }.get(name, default)
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(10.0)
        )
        try:
            newer = self._rich_plan(plan_id='newer', stamp_sec=9.5)
            node.grasp6d_plan_cb(newer)
            self.assertEqual(node.latest_grasp6d_plan.plan_id, newer.plan_id)
            node.grasp6d_plan_cb(
                self._rich_plan(plan_id='replayed', stamp_sec=9.0)
            )
        finally:
            grasp_task_node.rospy.get_param = original_get_param
            grasp_task_node.rospy.Time.now = original_now

        self.assertIsNone(node.latest_grasp6d_plan)

    def test_server_replay_watermark_survives_invalid_clear_and_repeat(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node.latest_grasp6d_plan = None
        original_get_param = grasp_task_node.rospy.get_param
        original_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.get_param = lambda name, default=None: {
            '/grasp_6d/plan_validity_sec': 2.0,
        }.get(name, default)
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(10.0)
        )
        try:
            newer = self._rich_plan(plan_id='newer', stamp_sec=9.5)
            node.grasp6d_plan_cb(newer)
            invalid = self._rich_plan(plan_id='invalid', stamp_sec=9.6)
            invalid.valid = False
            invalid.diagnostic = 'TARGET_LOST: invalidation tombstone'
            node.grasp6d_plan_cb(invalid)
            for suffix in ('first', 'second'):
                node.grasp6d_plan_cb(
                    self._rich_plan(plan_id='older-' + suffix, stamp_sec=9.0)
                )
                self.assertIsNone(node.latest_grasp6d_plan)

            newest = self._rich_plan(plan_id='newest', stamp_sec=9.8)
            node.grasp6d_plan_cb(newest)
            self.assertEqual(node.latest_grasp6d_plan.plan_id, newest.plan_id)
            node._clear_grasp6d_authority()
            node.grasp6d_plan_cb(newest)
            self.assertIsNone(node.latest_grasp6d_plan)
        finally:
            grasp_task_node.rospy.get_param = original_get_param
            grasp_task_node.rospy.Time.now = original_now

    def test_server_repeated_replay_stays_tombstoned(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node.latest_grasp6d_plan = None
        original_get_param = grasp_task_node.rospy.get_param
        original_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.get_param = lambda name, default=None: {
            '/grasp_6d/plan_validity_sec': 2.0,
        }.get(name, default)
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(10.0)
        )
        try:
            newer = self._rich_plan(plan_id='newer', stamp_sec=9.5)
            replay = self._rich_plan(plan_id='replay', stamp_sec=9.0)
            node.grasp6d_plan_cb(newer)
            node.grasp6d_plan_cb(replay)
            node.grasp6d_plan_cb(replay)
            self.assertIsNone(node.latest_grasp6d_plan)
        finally:
            grasp_task_node.rospy.get_param = original_get_param
            grasp_task_node.rospy.Time.now = original_now

    def test_zero_stamp_pending_preserves_strict_server_source_watermark(self):
        def pending(stamp_sec):
            message = Grasp6DPlan()
            message.header.frame_id = 'base_link'
            message.header.stamp = grasp_task_node.rospy.Time.from_sec(stamp_sec)
            message.valid = False
            message.diagnostic = 'PLAN_PENDING: planning snapshot in progress'
            return message

        def node_with_no_plan():
            node = grasp_task_node.GraspTaskNode.__new__(
                grasp_task_node.GraspTaskNode
            )
            node.latest_grasp6d_plan = None
            return node

        original_get_param = grasp_task_node.rospy.get_param
        original_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.get_param = lambda name, default=None: {
            '/grasp_6d/plan_validity_sec': 2.0,
        }.get(name, default)
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(10.0)
        )
        try:
            first = node_with_no_plan()
            first.grasp6d_plan_cb(pending(0.0))
            initial = self._rich_plan(stamp_sec=9.0, plan_id='initial')
            first.grasp6d_plan_cb(initial)
            self.assertEqual(first.latest_grasp6d_plan.plan_id, initial.plan_id)

            newer = self._rich_plan(stamp_sec=9.5, plan_id='newer')
            older = node_with_no_plan()
            older.grasp6d_plan_cb(newer)
            older.grasp6d_plan_cb(pending(0.0))
            older.grasp6d_plan_cb(
                self._rich_plan(stamp_sec=9.0, plan_id='older')
            )
            self.assertIsNone(older.latest_grasp6d_plan)

            successor = node_with_no_plan()
            successor.grasp6d_plan_cb(newer)
            successor.grasp6d_plan_cb(pending(0.0))
            next_plan = self._rich_plan(stamp_sec=9.8, plan_id='successor')
            successor.grasp6d_plan_cb(next_plan)
            self.assertEqual(
                successor.latest_grasp6d_plan.plan_id, next_plan.plan_id
            )

            same_stamp = node_with_no_plan()
            same_stamp.grasp6d_plan_cb(newer)
            same_stamp.grasp6d_plan_cb(pending(9.5))
            same_stamp.grasp6d_plan_cb(newer)
            self.assertIsNone(same_stamp.latest_grasp6d_plan)
        finally:
            grasp_task_node.rospy.get_param = original_get_param
            grasp_task_node.rospy.Time.now = original_now

    def test_object_jump_and_low_confidence_revoke_execution_authority(self):
        original_get_param = grasp_task_node.rospy.get_param
        original_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.get_param = lambda name, default=None: {
            '/grasp': {
                'min_object_confidence': 0.5,
                'max_object_jump_m': 0.12,
                'object_jump_filter_window_sec': 4.0,
            },
        }.get(name, default)
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(10.0)
        )
        try:
            for name, incoming in (
                ('jump', self._object_at(0.60, stamp_sec=9.9)),
                ('low-confidence', self._object_at(0.41, stamp_sec=9.9)),
            ):
                with self.subTest(case=name):
                    node = grasp_task_node.GraspTaskNode.__new__(
                        grasp_task_node.GraspTaskNode
                    )
                    previous = self._object_at(0.40)
                    previous.confidence = 0.9
                    incoming.confidence = 0.1 if name == 'low-confidence' else 0.9
                    node.latest_obj = previous
                    node.latest_obj_time = grasp_task_node.rospy.Time.from_sec(9.9)
                    node.latest_grasp6d_plan = self._rich_plan(stamp_sec=9.0)
                    node.obj_cb(incoming)
                    self.assertIsNone(node.latest_obj)
                    self.assertIsNone(node.latest_obj_time)
                    self.assertIsNone(node.latest_grasp6d_plan)
                    self.assertIs(node.latest_visual_obj, incoming)
        finally:
            grasp_task_node.rospy.get_param = original_get_param
            grasp_task_node.rospy.Time.now = original_now

    def test_object_callback_uses_source_stamp_not_receipt_time_for_authority(self):
        original_get_param = grasp_task_node.rospy.get_param
        original_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.get_param = lambda name, default=None: {
            '/grasp': {
                'min_object_confidence': 0.5,
                'max_object_jump_m': 0.12,
                'object_jump_filter_window_sec': 4.0,
            },
            '/grasp_6d/target_observation_validity_sec': 1.5,
        }.get(name, default)
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(10.0)
        )
        try:
            for name, stamp_sec in (
                ('zero', 0.0),
                ('stale', 1.0),
                ('future', 11.0),
            ):
                with self.subTest(case=name):
                    node = grasp_task_node.GraspTaskNode.__new__(
                        grasp_task_node.GraspTaskNode
                    )
                    node.latest_obj = self._object(stamp_sec=9.5)
                    node.latest_obj_time = grasp_task_node.rospy.Time.from_sec(9.5)
                    node.latest_grasp6d_plan = self._rich_plan(stamp_sec=9.0)
                    incoming = self._object_at(0.40, stamp_sec=stamp_sec)
                    incoming.confidence = 0.9
                    node.obj_cb(incoming)
                    self.assertIsNone(node.latest_obj)
                    self.assertIsNone(node.latest_obj_time)
                    self.assertIsNone(node.latest_grasp6d_plan)

            valid = grasp_task_node.GraspTaskNode.__new__(
                grasp_task_node.GraspTaskNode
            )
            valid.latest_obj = None
            valid.latest_obj_time = None
            valid.latest_grasp6d_plan = self._rich_plan(stamp_sec=9.0)
            incoming = self._object_at(0.40, stamp_sec=9.5)
            incoming.confidence = 0.9
            valid.obj_cb(incoming)
            self.assertIs(valid.latest_obj, incoming)
            self.assertEqual(valid.latest_obj_time.to_nsec(), 9_500_000_000)
            self.assertIsNotNone(valid.latest_grasp6d_plan)

            fallback = grasp_task_node.GraspTaskNode.__new__(
                grasp_task_node.GraspTaskNode
            )
            fallback.latest_obj = None
            fallback.latest_obj_time = None
            fallback.latest_grasp6d_plan = self._rich_plan(stamp_sec=9.0)
            incoming = self._object_at(0.40, stamp_sec=9.5)
            incoming.header.stamp = grasp_task_node.rospy.Time(0)
            incoming.confidence = 0.9
            fallback.obj_cb(incoming)
            self.assertIs(fallback.latest_obj, incoming)
            self.assertEqual(
                fallback.latest_obj_time.to_nsec(), 9_500_000_000
            )
        finally:
            grasp_task_node.rospy.get_param = original_get_param
            grasp_task_node.rospy.Time.now = original_now

    def test_live_target_observation_must_be_recent_and_not_future(self):
        plan = self._rich_plan(stamp_sec=9.0)
        original_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(10.0)
        )
        try:
            for name, stamp in (
                ('missing', None),
                ('expired', grasp_task_node.rospy.Time.from_sec(8.0)),
                ('future', grasp_task_node.rospy.Time.from_sec(10.1)),
            ):
                with self.subTest(case=name):
                    node = grasp_task_node.GraspTaskNode.__new__(
                        grasp_task_node.GraspTaskNode
                    )
                    node.latest_grasp6d_plan = plan
                    node.latest_obj = self._object()
                    node.latest_obj_time = stamp
                    result = node._bound_target_drift_result(
                        plan,
                        {
                            'target_max_drift_m': 0.02,
                            'target_observation_validity_sec': 1.5,
                        },
                    )
                    self.assertFalse(result.ok)
                    self.assertEqual(result.code, 'TARGET_STALE')
                    self.assertIsNone(node.latest_grasp6d_plan)
        finally:
            grasp_task_node.rospy.Time.now = original_now

    def test_float32_wire_width_limit_accepts_exact_50mm_only(self):
        exact = self._rich_plan(stamp_sec=9.0)
        exact.required_open_width_m = 0.050
        exact.plan_id = compute_plan_id(exact)
        wire = io.BytesIO()
        exact.serialize(wire)
        received = Grasp6DPlan()
        received.deserialize(wire.getvalue())
        self.assertTrue(
            grasp_task_node.validate_execution_plan(received, 10.0, 2.0).ok
        )

        over = self._rich_plan(stamp_sec=9.0)
        over.required_open_width_m = 0.0501
        over.plan_id = compute_plan_id(over)
        self.assertFalse(
            grasp_task_node.validate_execution_plan(over, 10.0, 2.0).ok
        )

    def test_bound_execution_checkpoint_ignores_original_plan_lease_age(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        plan = self._rich_plan(stamp_sec=9.0)
        node.latest_grasp6d_plan = plan
        node.latest_obj = self._object()
        node.latest_obj_time = grasp_task_node.rospy.Time.from_sec(19.5)
        node.active = True
        node.set_state = lambda *args, **kwargs: None
        original_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(20.0)
        )
        try:
            self.assertTrue(
                node._execution_checkpoint(
                    plan,
                    {
                        'plan_validity_sec': 2.0,
                        'target_observation_validity_sec': 1.5,
                    },
                    'late stage',
                )
            )
        finally:
            grasp_task_node.rospy.Time.now = original_now

    def test_fresh_plan_uses_obb_center_only_as_drift_reference_without_retarget(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node.latest_grasp6d_plan = self._rich_plan(stamp_sec=9.0)
        node.latest_obj = self._object_at(0.41, 0.0, 0.20)
        node.latest_obj_time = grasp_task_node.rospy.Time.from_sec(9.9)
        original_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(10.0)
        )
        try:
            plan = node._fresh_grasp6d_plan(
                {
                    'plan_validity_sec': 2.0,
                    'target_max_drift_m': 0.02,
                }
            )
        finally:
            grasp_task_node.rospy.Time.now = original_now

        self.assertIsNotNone(plan)
        self.assertAlmostEqual(plan.poses[0].position.x, 0.10)
        self.assertAlmostEqual(plan.object_geometry.pose_base.position.x, 0.40)
        plan.poses[0].position.x = -99.0
        self.assertAlmostEqual(node.latest_grasp6d_plan.poses[0].position.x, 0.10)

    def test_live_drift_guard_requires_detected_same_nonempty_label(self):
        cases = (
            ('missing', None),
            ('not-detected', types.SimpleNamespace(detected=False, label='carton')),
            ('empty-label', self._object_at(0.40, 0.0, 0.20)),
            ('wrong-label', self._object_at(0.40, 0.0, 0.20)),
        )
        cases[2][1].label = ''
        cases[3][1].label = 'bottle'
        original_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(10.0)
        )
        try:
            for name, live_target in cases:
                with self.subTest(case=name):
                    node = grasp_task_node.GraspTaskNode.__new__(
                        grasp_task_node.GraspTaskNode
                    )
                    node.latest_grasp6d_plan = self._rich_plan(stamp_sec=9.0)
                    node.latest_obj = live_target
                    result = node._fresh_grasp6d_plan(
                        {
                            'plan_validity_sec': 2.0,
                            'target_max_drift_m': 0.02,
                        }
                    )
                    self.assertIsNone(result)
                    self.assertIsNone(node.latest_grasp6d_plan)
        finally:
            grasp_task_node.rospy.Time.now = original_now

    def test_server_rejects_cross_snapshot_headers(self):
        header_cases = []
        wrong_plan_frame = self._rich_plan(stamp_sec=9.0)
        wrong_plan_frame.header.frame_id = 'map'
        header_cases.append(wrong_plan_frame)
        wrong_geometry_frame = self._rich_plan(stamp_sec=9.0)
        wrong_geometry_frame.object_geometry.header.frame_id = 'map'
        header_cases.append(wrong_geometry_frame)
        wrong_geometry_stamp = self._rich_plan(stamp_sec=9.0)
        wrong_geometry_stamp.object_geometry.header.stamp = grasp_task_node.rospy.Time(9, 1)
        header_cases.append(wrong_geometry_stamp)

        for plan in header_cases:
            with self.subTest(header=plan.object_geometry.header.frame_id):
                result = grasp_task_node.validate_execution_plan(plan, 10.0, 2.0)
                self.assertFalse(result.ok)

    def test_server_geometry_semantics_bbox_mode_and_digest_tampering(self):
        for field, value in (
            ('label', ''),
            ('source_mode', ''),
            ('source_mode', 'unknown'),
        ):
            plan = self._rich_plan(stamp_sec=9.0)
            setattr(plan.object_geometry, field, value)
            with self.subTest(field=field, value=value):
                self.assertFalse(
                    grasp_task_node.validate_execution_plan(
                        plan,
                        10.0,
                        2.0,
                    ).ok
                )

        bbox_plan = self._rich_plan(stamp_sec=9.0)
        bbox_plan.object_geometry.source_mode = 'bbox_depth'
        self.assertTrue(
            grasp_task_node.validate_execution_plan(
                bbox_plan,
                10.0,
                2.0,
            ).ok
        )

        for field in ('pose', 'width', 'geometry'):
            plan = self._rich_plan(stamp_sec=9.0)
            if field == 'pose':
                plan.poses[0].position.x += 0.01
            elif field == 'width':
                plan.candidate_width_m += 0.001
            else:
                plan.object_geometry.pose_base.position.z += 0.01
            with self.subTest(field=field):
                result = grasp_task_node.validate_execution_plan(plan, 10.0, 2.0)
                self.assertFalse(result.ok)

    def test_plan_bound_object_reference_is_deep_copied_from_obb_not_live_bbox(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        plan = self._rich_plan(stamp_sec=9.0)
        node.latest_obj = self._object_at(0.41, 0.0, 0.20)

        reference = node._bound_object_pose_from_plan(plan)
        plan.object_geometry.pose_base.position.x = 99.0

        self.assertTrue(reference.detected)
        self.assertEqual(reference.label, 'carton')
        self.assertAlmostEqual(reference.pose_base.pose.position.x, 0.40)
        self.assertNotAlmostEqual(
            reference.pose_base.pose.position.x,
            node.latest_obj.pose_base.pose.position.x,
        )

    def test_motion_stage_stops_if_plan_is_replaced_after_planning(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node.active = True
        first = self._rich_plan(plan_id='first', stamp_sec=9.0)
        node.latest_grasp6d_plan = first
        node.latest_obj = self._object()
        node.latest_obj_time = grasp_task_node.rospy.Time.from_sec(9.9)
        node.set_state = lambda *args, **kwargs: None
        node._wait_for_motion_settle = lambda *_args, **_kwargs: True
        calls = []

        def move_pose(_pose, execute):
            calls.append(bool(execute))
            if not execute:
                node.latest_grasp6d_plan = self._rich_plan(
                    plan_id='second', stamp_sec=9.5
                )
            return FakeServiceResponse(True, 'ok')

        original_now = grasp_task_node.rospy.Time.now
        original_get_param = grasp_task_node.rospy.get_param
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(10.0)
        )
        grasp_task_node.rospy.get_param = lambda _name, default=None: default
        try:
            result = node._plan_and_execute_pose(
                grasp_task_node.GraspStages.MOVE_PREGRASP,
                '6D pregrasp',
                self._pose(0.10),
                move_pose,
                '6D pregrasp',
                execution_plan_id=first.plan_id,
                gcfg={'plan_validity_sec': 2.0},
            )
        finally:
            grasp_task_node.rospy.Time.now = original_now
            grasp_task_node.rospy.get_param = original_get_param

        self.assertFalse(result)
        self.assertEqual(calls, [False])

    def test_plan_bound_action_linearizes_with_replacement_callback(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node.active = True
        first = self._rich_plan(plan_id='first', stamp_sec=9.0)
        second = self._rich_plan(plan_id='second', stamp_sec=9.5)
        node.latest_grasp6d_plan = first
        node.latest_obj = self._object()
        node.latest_obj_time = grasp_task_node.rospy.Time.from_sec(9.9)
        action_entered = threading.Event()
        action_release = threading.Event()
        replacement_done = threading.Event()
        order = []
        results = []

        def action():
            action_entered.set()
            action_release.wait(2.0)
            order.append('action')
            return FakeServiceResponse(True, 'committed')

        def invoke():
            results.append(
                node._invoke_plan_bound_action(
                    first,
                    {
                        'target_max_drift_m': 0.02,
                        'target_observation_validity_sec': 1.5,
                    },
                    'threaded action',
                    action,
                )
            )

        def replace():
            node.grasp6d_plan_cb(second)
            order.append('replacement')
            replacement_done.set()

        original_get_param = grasp_task_node.rospy.get_param
        original_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.get_param = lambda name, default=None: {
            '/grasp_6d/plan_validity_sec': 2.0,
        }.get(name, default)
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(10.0)
        )
        action_thread = threading.Thread(target=invoke)
        replacement_thread = threading.Thread(target=replace)
        replacement_started = False
        try:
            action_thread.start()
            self.assertTrue(action_entered.wait(1.0))
            replacement_thread.start()
            replacement_started = True
            self.assertFalse(replacement_done.wait(0.05))
            action_release.set()
            action_thread.join(2.0)
            if replacement_started:
                replacement_thread.join(2.0)
        finally:
            action_release.set()
            action_thread.join(2.0)
            replacement_thread.join(2.0)
            grasp_task_node.rospy.get_param = original_get_param
            grasp_task_node.rospy.Time.now = original_now

        self.assertFalse(action_thread.is_alive())
        self.assertFalse(replacement_thread.is_alive())
        self.assertTrue(results[0][0].ok)
        self.assertTrue(results[0][1].success)
        self.assertEqual(order, ['action', 'replacement'])

    def test_plan_bound_action_rejects_already_replaced_plan_without_call(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node.active = True
        first = self._rich_plan(plan_id='first', stamp_sec=9.0)
        node.latest_grasp6d_plan = self._rich_plan(
            plan_id='second', stamp_sec=9.5
        )
        node.latest_obj = self._object()
        node.latest_obj_time = grasp_task_node.rospy.Time.from_sec(9.9)
        calls = []
        result, response = node._invoke_plan_bound_action(
            first,
            {
                'target_max_drift_m': 0.02,
                'target_observation_validity_sec': 1.5,
            },
            'replaced action',
            lambda: calls.append(True) or FakeServiceResponse(True),
        )

        self.assertFalse(result.ok)
        self.assertEqual(result.code, 'PLAN_REPLACED')
        self.assertIsNone(response)
        self.assertEqual(calls, [])

    def test_move_open_and_close_do_not_commit_an_already_replaced_plan(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node.active = True
        first = self._rich_plan(plan_id='first', stamp_sec=9.0)
        node.latest_grasp6d_plan = self._rich_plan(
            plan_id='second', stamp_sec=9.5
        )
        node.latest_obj = self._object()
        node.latest_obj_time = grasp_task_node.rospy.Time.from_sec(9.9)
        node.set_state = lambda *args, **kwargs: None
        node._wait_for_motion_settle = lambda *_args, **_kwargs: True
        calls = []
        gcfg = {
            'target_max_drift_m': 0.02,
            'target_observation_validity_sec': 1.5,
        }

        self.assertFalse(
            node._plan_and_execute_pose(
                grasp_task_node.GraspStages.MOVE_PREGRASP,
                'bound move',
                self._pose(0.10),
                lambda _pose, execute: (
                    calls.append(('move', bool(execute)))
                    or FakeServiceResponse(True)
                ),
                'bound move',
                execution_plan=first,
                gcfg=gcfg,
            )
        )
        self.assertFalse(
            node._command_gripper_position(
                lambda _position: (
                    calls.append(('open', True)) or FakeServiceResponse(True)
                ),
                0.05,
                'bound open',
                0.0,
                execution_plan=first,
                gcfg=gcfg,
            )
        )
        closed, _message = node._close_gripper(
            {'use_compliant_close': True},
            lambda _position: FakeServiceResponse(True),
            lambda **_kwargs: (
                calls.append(('close', True)) or FakeServiceResponse(True)
            ),
            execution_plan=first,
            gcfg=gcfg,
        )
        self.assertFalse(closed)
        self.assertEqual(calls, [])

    def test_direct_rich_execution_requires_cached_strict_executor_before_any_action(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node.active = True
        plan = self._rich_plan(stamp_sec=1.0)
        states = []
        actions = []
        node.set_state = lambda *args, **kwargs: states.append(args)
        node._bound_target_drift_result = lambda *_args: (
            grasp_task_node.PlanValidationResult(True)
        )
        node._simulate_grasp6d_plan_if_required = lambda *_args: (
            actions.append('simulate') or True
        )
        node._command_gripper_position = lambda *_args, **_kwargs: (
            actions.append('open') or True
        )

        original_get_param = grasp_task_node.rospy.get_param
        original_time_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.get_param = lambda _name, default=None: default
        grasp_task_node.rospy.Time.now = staticmethod(lambda: FakeTime(1.0))
        try:
            result = node._execute_grasp6d_plan(
                {'plan_validity_sec': 5.0},
                {'use_compliant_close': False},
                0.05,
                lambda *_args: actions.append('generic-pose'),
                lambda *_args: actions.append('linear-pose'),
                lambda *_args: actions.append('gripper'),
                None,
                plan,
                strict_execute_pose=None,
            )
        finally:
            grasp_task_node.rospy.get_param = original_get_param
            grasp_task_node.rospy.Time.now = original_time_now

        self.assertFalse(result)
        self.assertEqual(actions, [])
        self.assertEqual(states[-1][0], grasp_task_node.GraspStages.FAILED)
        self.assertIn('STRICT_CACHED_EXECUTOR_UNAVAILABLE', states[-1][1])

    def test_rich_pregrasp_cache_failure_never_falls_back_to_generic_execute(self):
        failures = (
            'no cached pose plan',
            "cached plan kind 'position-only' is not 'strict pose'",
        )
        original_get_param = grasp_task_node.rospy.get_param
        grasp_task_node.rospy.get_param = lambda _name, default=None: default
        try:
            for failure in failures:
                with self.subTest(failure=failure):
                    node = grasp_task_node.GraspTaskNode.__new__(
                        grasp_task_node.GraspTaskNode
                    )
                    node.active = True
                    states = []
                    calls = []
                    node.set_state = lambda *args, **kwargs: states.append(args)
                    node._wait_for_motion_settle = lambda *_args: (
                        calls.append(('settle', True)) or True
                    )
                    node._validate_bound_plan = lambda *_args: (
                        grasp_task_node.PlanValidationResult(True)
                    )
                    node._invoke_plan_bound_action = (
                        lambda _plan, _gcfg, _label, action: (
                            grasp_task_node.PlanValidationResult(True),
                            action(),
                        )
                    )

                    def strict_plan(_pose, execute):
                        calls.append(('strict-plan', bool(execute)))
                        return FakeServiceResponse(True, 'planned: strict pose')

                    def strict_execute(_pose, execute):
                        calls.append(('strict-execute', bool(execute)))
                        return FakeServiceResponse(False, failure)

                    result = node._plan_and_execute_pose(
                        grasp_task_node.GraspStages.MOVE_PREGRASP,
                        '6D pregrasp',
                        self._pose(0.10),
                        strict_plan,
                        '6D pregrasp',
                        execution_plan=types.SimpleNamespace(
                            plan_id='strict-rich-plan'
                        ),
                        gcfg={},
                        execute_pose=strict_execute,
                    )

                    self.assertFalse(result)
                    self.assertEqual(
                        calls,
                        [('strict-plan', False), ('strict-execute', True)],
                    )
                    self.assertIn(failure, states[-1][1])
                    self.assertNotIn('fallback', states[-1][1].lower())
        finally:
            grasp_task_node.rospy.get_param = original_get_param

    def test_full_grasp_uses_6d_plan_sequence_when_enabled(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node.latest_obj = self._object()
        node.latest_obj_time = grasp_task_node.rospy.Time.from_sec(3.5)
        node.latest_grasp6d_plan = self._rich_plan(stamp_sec=1.0)
        node.active = True
        node._wait_for_motion_settle = lambda reason='motion': calls.append(('settle', reason))
        node._simulate_grasp6d_plan_if_required = lambda *_args: True
        node.set_state = lambda *args, **kwargs: None

        calls = []
        proxy_names = []

        def fake_service_proxy(name, _srv_type):
            proxy_names.append(name)
            if name in (
                '/supervisor/check_pose_strict',
                '/supervisor/execute_pose_strict',
                '/supervisor/move_to_pose_linear',
            ):
                def move_pose(pose, execute):
                    mode = {
                        '/supervisor/check_pose_strict': 'strict-plan',
                        '/supervisor/execute_pose_strict': 'strict-execute',
                        '/supervisor/move_to_pose_linear': 'linear',
                    }[name]
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
                'plan_validity_sec': 2.0,
                'target_observation_validity_sec': 1.5,
                'lift_height_m': 0.05,
            },
            '/gripper': {
                'open_position_m': 0.0,
            },
        }.get(name, default)
        grasp_task_node.rospy.sleep = lambda *_args, **_kwargs: None
        grasp_task_node.rospy.Time.now = staticmethod(lambda: FakeTime(4.0))
        try:
            self.assertTrue(
                grasp_task_node.GraspTaskNode.execute(
                    node,
                    grasp6d_plan=node.latest_grasp6d_plan,
                )
            )
        finally:
            grasp_task_node.rospy.wait_for_service = original_wait_for_service
            grasp_task_node.rospy.ServiceProxy = original_service_proxy
            grasp_task_node.rospy.get_param = original_get_param
            grasp_task_node.rospy.sleep = original_sleep
            grasp_task_node.rospy.Time.now = original_time_now

        self.assertEqual(calls[0], ('set_gripper', 0.0))
        move_calls = [
            call for call in calls
            if call[0] in ('strict-plan', 'strict-execute', 'linear')
        ]
        self.assertEqual(
            [(call[0], round(call[1], 2), call[2]) for call in move_calls],
            [
                ('strict-plan', 0.10, False),
                ('strict-execute', 0.10, True),
                ('linear', 0.20, False), ('linear', 0.20, True),
                ('linear', 0.30, False), ('linear', 0.30, True),
                ('linear', 0.40, False), ('linear', 0.40, True),
            ],
        )
        self.assertIn(('close', True), calls)
        self.assertNotIn('/supervisor/move_to_pose', proxy_names)

    def test_6d_plan_honors_configured_simple_open_and_close_positions(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node.latest_obj = self._object()
        node.latest_obj_time = grasp_task_node.rospy.Time.from_sec(1.0)
        node.latest_grasp6d_plan = self._rich_plan(stamp_sec=1.0)
        node.active = True
        node._wait_for_motion_settle = lambda reason='motion': calls.append(('settle', reason))
        node._simulate_grasp6d_plan_if_required = lambda *_args: True
        node.set_state = lambda *args, **kwargs: None

        calls = []

        def fake_service_proxy(name, _srv_type):
            if name in (
                '/supervisor/check_pose_strict',
                '/supervisor/execute_pose_strict',
                '/supervisor/move_to_pose_linear',
            ):
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
                'plan_validity_sec': 5.0,
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
            self.assertTrue(
                grasp_task_node.GraspTaskNode.execute(
                    node,
                    grasp6d_plan=node.latest_grasp6d_plan,
                )
            )
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
        node.latest_obj_time = grasp_task_node.rospy.Time.from_sec(1.0)
        node.latest_grasp6d_plan = self._rich_plan(stamp_sec=1.0)
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
                calls.append(('simulate', len(payload.get('trajectory') or [])))
                calls.append(('joint_payload', len(payload.get('joint_names') or [])))
                return {
                    'plan_id': node.latest_grasp6d_plan.plan_id,
                    'simulation_ok': True,
                    'ik_success': True,
                    'collision_free': False,
                    'contact_success': True,
                    'lift_success': True,
                    'score': 95,
                    'failure_code': 'MUJOCO_COLLISION',
                    'failure_reason': 'gripper would collide with table',
                }

        def fake_service_proxy(name, _srv_type):
            if name in (
                '/supervisor/check_pose_strict',
                '/supervisor/execute_pose_strict',
                '/supervisor/move_to_pose_linear',
            ):
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
                'plan_validity_sec': 5.0,
            },
            '/gripper': {
                'open_position_m': 0.05,
                'use_compliant_close': False,
            },
            '/mujoco_digital_twin': {
                'enabled': True,
                'execution_gate_enabled': True,
                'audit_output_path': self._mujoco_audit_path,
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
            self.assertFalse(
                grasp_task_node.GraspTaskNode.execute(
                    node,
                    grasp6d_plan=node.latest_grasp6d_plan,
                )
            )
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

    def test_strict_mujoco_gate_complete_pass_reaches_first_physical_action_once(self):
        result, physical, states, payloads, node, plan = (
            self._run_mujoco_gate_to_first_physical_action(
                response=lambda bound: self._passing_mujoco_response(
                    bound.plan_id
                )
            )
        )

        self.assertFalse(result)  # the first-action stub stops the sequence
        self.assertEqual(physical, ['open-gripper'])
        self.assertEqual(len(payloads), 1)
        self.assertEqual(payloads[0]['schema_version'], 2)
        self.assertEqual(payloads[0]['plan_id'], plan.plan_id)
        self.assertEqual(len(payloads[0]['trajectory']), 4)
        self.assertIn('MuJoCo simulation passed', states[-2][1])
        audit = node._test_mujoco_audit
        self.assertIsNotNone(audit)
        self.assertEqual(audit['request_plan_id'], plan.plan_id)
        self.assertEqual(audit['payload']['plan_id'], plan.plan_id)
        self.assertEqual(audit['payload']['summary']['trajectory_count'], 4)
        self.assertEqual(audit['response']['raw_echo_plan_id'], plan.plan_id)
        self.assertTrue(audit['response']['strict_json_serializable'])
        for key in grasp_task_node._MUJOCO_SAFETY_KEYS:
            self.assertIs(audit['response'][key], True)
        self.assertEqual(audit['response']['score'], 95.0)
        self.assertTrue(audit['authority_after_network']['ok'])
        self.assertTrue(audit['gate_validation']['ok'])
        self.assertTrue(audit['final_validation']['ok'])
        self.assertEqual(
            audit['final_validation']['code'],
            'MUJOCO_GATE_PASSED',
        )
        self.assertGreaterEqual(audit['attempt']['duration_sec'], 0.0)
        expected_hash = hashlib.sha256(node._test_mujoco_audit_bytes).hexdigest()
        self.assertIn('audit_path=' + node._test_mujoco_audit_path, states[-2][1])
        self.assertIn('audit_sha256=' + expected_hash, states[-2][1])

    def test_strict_mujoco_gate_requires_exact_boolean_true_authority_flags(self):
        cases = (
            ('enabled-false', {'mujoco_enabled': False}, 'enabled'),
            (
                'enabled-missing',
                {'mujoco_enabled': _MISSING_CONFIG_VALUE},
                'enabled',
            ),
            ('enabled-none', {'mujoco_enabled': None}, 'enabled'),
            ('enabled-integer', {'mujoco_enabled': 1}, 'enabled'),
            (
                'execution-false',
                {'execution_gate_enabled': False},
                'execution_gate_enabled',
            ),
            (
                'execution-missing',
                {'execution_gate_enabled': _MISSING_CONFIG_VALUE},
                'execution_gate_enabled',
            ),
            (
                'execution-string',
                {'execution_gate_enabled': 'true'},
                'execution_gate_enabled',
            ),
        )
        for label, kwargs, parameter in cases:
            with self.subTest(case=label):
                _result, physical, states, payloads, _node, _plan = (
                    self._run_mujoco_gate_to_first_physical_action(**kwargs)
                )
                self.assertEqual(payloads, [])
                self.assertEqual(physical, [])
                self.assertEqual(
                    states[-1][0], grasp_task_node.GraspStages.FAILED
                )
                self.assertIn('MUJOCO_GATE_CONFIG_INVALID', states[-1][1])
                self.assertIn(parameter, states[-1][1])

    def test_mujoco_and_planning_audit_canonical_path_conflict_blocks_before_network(self):
        shared_path = os.path.join(
            self._mujoco_audit_directory.name,
            'shared-audit.json',
        )
        alias_path = os.path.join(
            self._mujoco_audit_directory.name,
            'not-created',
            '..',
            'shared-audit.json',
        )

        _result, physical, states, payloads, _node, _plan = (
            self._run_mujoco_gate_to_first_physical_action(
                audit_output_path=alias_path,
                planning_audit_output_path=shared_path,
            )
        )

        self.assertEqual(payloads, [])
        self.assertEqual(physical, [])
        self.assertEqual(states[-1][0], grasp_task_node.GraspStages.FAILED)
        self.assertIn('MUJOCO_AUDIT_PATH_CONFLICT', states[-1][1])
        self.assertFalse(os.path.exists(shared_path))

        with open(shared_path, 'wb') as handle:
            handle.write(b'{"planning_audit":"evidence"}')
        hardlink_path = os.path.join(
            self._mujoco_audit_directory.name,
            'hardlinked-mujoco-audit.json',
        )
        os.link(shared_path, hardlink_path)
        _result, physical, states, payloads, _node, _plan = (
            self._run_mujoco_gate_to_first_physical_action(
                audit_output_path=hardlink_path,
                planning_audit_output_path=shared_path,
            )
        )

        self.assertEqual(payloads, [])
        self.assertEqual(physical, [])
        self.assertIn('MUJOCO_AUDIT_PATH_CONFLICT', states[-1][1])
        with open(shared_path, 'rb') as handle:
            self.assertEqual(
                handle.read(),
                b'{"planning_audit":"evidence"}',
            )

    def test_strict_mujoco_gate_rejects_nonfinite_value_hidden_in_response(self):
        for label, nonfinite in (
            ('nan', float('nan')),
            ('positive-infinity', float('inf')),
            ('negative-infinity', float('-inf')),
        ):
            with self.subTest(case=label):
                def non_strict_response(bound, value=nonfinite):
                    response = self._passing_mujoco_response(bound.plan_id)
                    response['unexpected_diagnostic'] = value
                    return response

                _result, physical, states, _payloads, node, _plan = (
                    self._run_mujoco_gate_to_first_physical_action(
                        response=non_strict_response,
                    )
                )

                self.assertEqual(physical, [])
                self.assertIn('WSL_UNAVAILABLE', states[-1][1])
                self.assertFalse(
                    node._test_mujoco_audit['response'][
                        'strict_json_serializable'
                    ]
                )
                self.assertFalse(
                    node._test_mujoco_audit['gate_validation']['ok']
                )
                self.assertIn(
                    'strict-JSON',
                    node._test_mujoco_audit['gate_validation']['reason'],
                )

    def test_mujoco_audit_writer_is_atomic_fsynced_strict_json_in_temporary_directory(self):
        path = self._next_mujoco_audit_path()
        with open(path, 'wb') as handle:
            handle.write(b'old-audit')
        report = grasp_task_node._new_mujoco_execution_audit(
            types.SimpleNamespace(plan_id='audit-plan')
        )
        reference = grasp_task_node._finalize_mujoco_execution_audit(
            report,
            path,
            True,
            'MUJOCO_GATE_PASSED',
            'passed',
            score=95.0,
        )

        with open(path, 'rb') as handle:
            encoded = handle.read()
        parsed = json.loads(
            encoded.decode('utf-8'),
            parse_constant=lambda value: (_ for _ in ()).throw(
                AssertionError('non-strict JSON constant %s' % value)
            ),
        )
        self.assertEqual(parsed['request_plan_id'], 'audit-plan')
        self.assertTrue(parsed['final_validation']['ok'])
        self.assertEqual(reference['path'], os.path.abspath(path))
        self.assertEqual(reference['sha256'], hashlib.sha256(encoded).hexdigest())
        self.assertEqual(
            [name for name in os.listdir(os.path.dirname(path)) if '.tmp-' in name],
            [],
        )

        invalid_path = self._next_mujoco_audit_path()
        with self.assertRaises(ValueError):
            grasp_task_node.write_mujoco_execution_audit(
                invalid_path,
                {'not_strict': float('nan')},
            )
        self.assertFalse(os.path.exists(invalid_path))

    def test_strict_mujoco_gate_rejection_audit_preserves_bounded_response_evidence(self):
        def rejected(bound):
            return dict(
                self._passing_mujoco_response(bound.plan_id),
                collision_free=False,
                failure_code='MUJOCO_COLLISION',
                failure_reason='gripper would collide with table',
            )

        _result, physical, states, _payloads, node, plan = (
            self._run_mujoco_gate_to_first_physical_action(
                response=rejected,
            )
        )

        self.assertEqual(physical, [])
        self.assertIn('MUJOCO_COLLISION', states[-1][1])
        audit = node._test_mujoco_audit
        self.assertEqual(audit['request_plan_id'], plan.plan_id)
        self.assertEqual(audit['response']['raw_echo_plan_id'], plan.plan_id)
        self.assertIs(audit['response']['collision_free'], False)
        self.assertEqual(audit['response']['failure_code'], 'MUJOCO_COLLISION')
        self.assertEqual(
            audit['response']['failure_reason'],
            'gripper would collide with table',
        )
        self.assertFalse(audit['gate_validation']['ok'])
        self.assertEqual(
            audit['final_validation']['code'],
            'MUJOCO_COLLISION',
        )

    def test_strict_mujoco_gate_keeps_full_reason_in_file_but_bounds_ros_state(self):
        full_reason = 'collision:' + ('x' * 5000) + ':response-tail'

        def rejected(bound):
            return dict(
                self._passing_mujoco_response(bound.plan_id),
                collision_free=False,
                failure_code='MUJOCO_COLLISION',
                failure_reason=full_reason,
            )

        _result, physical, states, _payloads, node, _plan = (
            self._run_mujoco_gate_to_first_physical_action(response=rejected)
        )

        self.assertEqual(physical, [])
        self.assertEqual(
            node._test_mujoco_audit['response']['failure_reason'],
            full_reason,
        )
        self.assertEqual(
            node._test_mujoco_audit['response']['failure_reason_length'],
            len(full_reason),
        )
        self.assertNotIn('response-tail', states[-1][1])
        self.assertLess(len(states[-1][1]), 900)

    def test_strict_mujoco_gate_audits_build_and_network_failures(self):
        cases = (
            (
                'build',
                {'payload_build_error': ValueError('payload rejected')},
                ('payload', 'build_error'),
                'ValueError',
            ),
            (
                'network',
                {'request_error': TimeoutError('request timed out')},
                ('response', 'network_error'),
                'TimeoutError',
            ),
        )
        for label, kwargs, location, expected_type in cases:
            with self.subTest(case=label):
                _result, physical, states, _payloads, node, _plan = (
                    self._run_mujoco_gate_to_first_physical_action(**kwargs)
                )
                self.assertEqual(physical, [])
                self.assertIn('WSL_UNAVAILABLE', states[-1][1])
                evidence = node._test_mujoco_audit[location[0]][location[1]]
                self.assertEqual(evidence['type'], expected_type)
                self.assertFalse(
                    node._test_mujoco_audit['final_validation']['ok']
                )
                if label == 'network':
                    self.assertTrue(
                        node._test_mujoco_audit[
                            'authority_after_network'
                        ]['checked']
                    )

    def test_strict_mujoco_gate_plan_id_mismatch_is_audited_and_blocks_motion(self):
        _result, physical, states, _payloads, node, plan = (
            self._run_mujoco_gate_to_first_physical_action(
                response=lambda bound: dict(
                    self._passing_mujoco_response(bound.plan_id),
                    plan_id='different-plan-id',
                )
            )
        )

        self.assertEqual(physical, [])
        self.assertIn('PLAN_ID_MISMATCH', states[-1][1])
        audit = node._test_mujoco_audit
        self.assertEqual(audit['request_plan_id'], plan.plan_id)
        self.assertEqual(audit['payload']['plan_id'], plan.plan_id)
        self.assertEqual(
            audit['response']['raw_echo_plan_id'],
            'different-plan-id',
        )
        self.assertEqual(
            audit['final_validation']['code'],
            'PLAN_ID_MISMATCH',
        )

    def test_strict_mujoco_gate_nan_response_still_writes_strict_json_audit(self):
        def non_strict_response(bound):
            response = self._passing_mujoco_response(bound.plan_id)
            response['score'] = float('nan')
            return response

        _result, physical, states, _payloads, node, _plan = (
            self._run_mujoco_gate_to_first_physical_action(
                response=non_strict_response,
            )
        )

        self.assertEqual(physical, [])
        self.assertIn('WSL_UNAVAILABLE', states[-1][1])
        self.assertIsNone(node._test_mujoco_audit['response']['score'])
        self.assertFalse(
            node._test_mujoco_audit['response']['strict_json_serializable']
        )
        parsed = json.loads(
            node._test_mujoco_audit_bytes.decode('utf-8'),
            parse_constant=lambda value: (_ for _ in ()).throw(
                AssertionError('non-strict JSON constant %s' % value)
            ),
        )
        self.assertIsNone(parsed['response']['score'])

    def test_strict_mujoco_gate_huge_integer_score_is_audited_fail_closed(self):
        def huge_score_response(bound):
            response = self._passing_mujoco_response(bound.plan_id)
            response['score'] = 10 ** 400
            return response

        _result, physical, states, _payloads, node, _plan = (
            self._run_mujoco_gate_to_first_physical_action(
                response=huge_score_response,
            )
        )

        self.assertEqual(physical, [])
        self.assertIn('WSL_UNAVAILABLE', states[-1][1])
        audit = node._test_mujoco_audit
        self.assertTrue(audit['response']['strict_json_serializable'])
        self.assertIsNone(audit['response']['score'])
        self.assertTrue(audit['gate_validation']['checked'])
        self.assertFalse(audit['gate_validation']['ok'])
        self.assertIn(
            'response validation failed',
            audit['gate_validation']['reason'],
        )
        self.assertEqual(
            audit['final_validation']['code'],
            'WSL_UNAVAILABLE',
        )

    def test_strict_mujoco_gate_non_object_response_keeps_exact_strict_json_hash(self):
        _result, physical, states, _payloads, node, _plan = (
            self._run_mujoco_gate_to_first_physical_action(response=[])
        )

        self.assertEqual(physical, [])
        self.assertIn('WSL_UNAVAILABLE', states[-1][1])
        response_audit = node._test_mujoco_audit['response']
        self.assertTrue(response_audit['received'])
        self.assertFalse(response_audit['json_object'])
        self.assertTrue(response_audit['strict_json_serializable'])
        self.assertEqual(
            response_audit['sha256'],
            hashlib.sha256(b'[]').hexdigest(),
        )

    def test_strict_mujoco_gate_passing_response_audit_write_failure_blocks_motion(self):
        _result, physical, states, payloads, node, _plan = (
            self._run_mujoco_gate_to_first_physical_action(
                response=lambda bound: self._passing_mujoco_response(
                    bound.plan_id
                ),
                # Replacing an existing directory with the audit file fails.
                audit_output_path=self._mujoco_audit_directory.name,
            )
        )

        self.assertEqual(len(payloads), 1)
        self.assertEqual(physical, [])
        self.assertIsNone(node._test_mujoco_audit)
        self.assertEqual(states[-1][0], grasp_task_node.GraspStages.FAILED)
        self.assertIn('MUJOCO_AUDIT_WRITE_FAILED', states[-1][1])
        self.assertIn('MUJOCO_GATE_PASSED', states[-1][1])

    def test_strict_mujoco_gate_rejection_audit_write_failure_is_visible(self):
        def rejected(bound):
            return dict(
                self._passing_mujoco_response(bound.plan_id),
                collision_free=False,
                failure_code='MUJOCO_COLLISION',
                failure_reason='collision evidence',
            )

        _result, physical, states, _payloads, _node, _plan = (
            self._run_mujoco_gate_to_first_physical_action(
                response=rejected,
                audit_output_path=self._mujoco_audit_directory.name,
            )
        )

        self.assertEqual(physical, [])
        self.assertEqual(states[-1][0], grasp_task_node.GraspStages.FAILED)
        self.assertIn('MUJOCO_COLLISION', states[-1][1])
        self.assertIn('MUJOCO_AUDIT_WRITE_FAILED', states[-1][1])

    def test_strict_mujoco_gate_empty_audit_path_fails_before_network(self):
        _result, physical, states, payloads, _node, _plan = (
            self._run_mujoco_gate_to_first_physical_action(
                response=lambda bound: self._passing_mujoco_response(
                    bound.plan_id
                ),
                audit_output_path='   ',
            )
        )

        self.assertEqual(payloads, [])
        self.assertEqual(physical, [])
        self.assertEqual(states[-1][0], grasp_task_node.GraspStages.FAILED)
        self.assertIn('MUJOCO_AUDIT_PATH_INVALID', states[-1][1])

    def test_strict_mujoco_gate_revalidates_bound_plan_after_network_return(self):
        def replace_plan(node, _bound):
            node.latest_grasp6d_plan = self._rich_plan(
                plan_id='replacement',
                stamp_sec=1.5,
            )

        _result, physical, states, _payloads, _node, _plan = (
            self._run_mujoco_gate_to_first_physical_action(
                response=lambda bound: self._passing_mujoco_response(
                    bound.plan_id
                ),
                during_request=replace_plan,
            )
        )

        self.assertEqual(physical, [])
        self.assertIn('PLAN_REPLACED', states[-1][1])

    def test_strict_mujoco_gate_stop_during_network_returns_to_idle(self):
        def stop_execution(node, _bound):
            node.active = False

        _result, physical, states, _payloads, _node, _plan = (
            self._run_mujoco_gate_to_first_physical_action(
                response=lambda bound: self._passing_mujoco_response(
                    bound.plan_id
                ),
                during_request=stop_execution,
            )
        )

        self.assertEqual(physical, [])
        self.assertEqual(states[-1][0], grasp_task_node.GraspStages.IDLE)
        self.assertIn('EXECUTION_CANCELLED', states[-1][1])

    def test_strict_mujoco_gate_authority_change_precedes_network_error(self):
        def stop_execution(node, _bound):
            node.active = False

        def replace_plan(node, _bound):
            node.latest_grasp6d_plan = self._rich_plan(
                plan_id='replacement-on-error',
                stamp_sec=1.5,
            )

        cases = (
            (
                'stop-timeout',
                stop_execution,
                TimeoutError('request timed out'),
                grasp_task_node.GraspStages.IDLE,
                'EXECUTION_CANCELLED',
            ),
            (
                'replacement-malformed',
                replace_plan,
                ValueError('malformed JSON'),
                grasp_task_node.GraspStages.FAILED,
                'PLAN_REPLACED',
            ),
        )
        for label, mutation, error, expected_stage, expected_code in cases:
            with self.subTest(case=label):
                _result, physical, states, _payloads, _node, _plan = (
                    self._run_mujoco_gate_to_first_physical_action(
                        request_error=error,
                        during_request=mutation,
                    )
                )
                self.assertEqual(physical, [])
                self.assertEqual(states[-1][0], expected_stage)
                self.assertIn(expected_code, states[-1][1])

    def test_strict_mujoco_gate_errors_malformed_and_incomplete_responses_block_motion(self):
        def response_without(key):
            def make(bound):
                value = self._passing_mujoco_response(bound.plan_id)
                value.pop(key)
                return value
            return make

        cases = [
            ('timeout', None, TimeoutError('request timed out'), 'WSL_UNAVAILABLE'),
            ('malformed-json', None, ValueError('malformed JSON'), 'WSL_UNAVAILABLE'),
            ('non-object', lambda _bound: [], None, 'WSL_UNAVAILABLE'),
            ('missing-plan-id', response_without('plan_id'), None, 'PLAN_ID_MISMATCH'),
            (
                'mismatched-plan-id',
                lambda bound: dict(
                    self._passing_mujoco_response(bound.plan_id),
                    plan_id='different-plan-id',
                ),
                None,
                'PLAN_ID_MISMATCH',
            ),
        ]
        cases.extend(
            ('missing-' + key, response_without(key), None, 'WSL_UNAVAILABLE')
            for key in (
                'simulation_ok',
                'ik_success',
                'collision_free',
                'contact_success',
                'lift_success',
            )
        )

        for label, response, error, expected_code in cases:
            with self.subTest(case=label):
                _result, physical, states, _payloads, _node, _plan = (
                    self._run_mujoco_gate_to_first_physical_action(
                        response=response,
                        request_error=error,
                    )
                )
                self.assertEqual(physical, [])
                self.assertIn(expected_code, states[-1][1])

    def test_strict_mujoco_gate_ignores_allow_on_error_for_timeout_and_malformed(self):
        for label, error in (
            ('timeout', TimeoutError('request timed out')),
            ('malformed', ValueError('malformed JSON')),
        ):
            with self.subTest(case=label):
                _result, physical, states, _payloads, _node, _plan = (
                    self._run_mujoco_gate_to_first_physical_action(
                        request_error=error,
                        allow_execution_on_error=True,
                    )
                )
                self.assertEqual(physical, [])
                self.assertIn('WSL_UNAVAILABLE', states[-1][1])

    def test_6d_plan_blocks_mujoco_simulation_when_joint_state_payload_required_but_missing(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node.latest_obj = self._object()
        node.latest_obj_time = grasp_task_node.rospy.Time.from_sec(1.0)
        node.latest_grasp6d_plan = self._rich_plan(stamp_sec=1.0)
        node.latest_joint_state = None
        node.active = True
        states = []
        calls = []
        node._wait_for_motion_settle = lambda reason='motion': calls.append(('settle', reason))
        node.set_state = lambda *args, **kwargs: states.append(args)

        def fake_service_proxy(name, _srv_type):
            if name in (
                '/supervisor/check_pose_strict',
                '/supervisor/execute_pose_strict',
                '/supervisor/move_to_pose_linear',
            ):
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
                'plan_validity_sec': 5.0,
            },
            '/gripper': {
                'open_position_m': 0.05,
                'use_compliant_close': False,
            },
            '/mujoco_digital_twin': {
                'enabled': True,
                'execution_gate_enabled': True,
                'audit_output_path': self._mujoco_audit_path,
                'server_url': 'http://172.23.132.97:8000',
                'require_object_pose': True,
                'send_joint_state_in_request': True,
            },
        }.get(name, default)
        grasp_task_node.rospy.sleep = lambda *_args, **_kwargs: None
        grasp_task_node.rospy.Time.now = staticmethod(lambda: FakeTime(1.0))
        try:
            self.assertFalse(
                grasp_task_node.GraspTaskNode.execute(
                    node,
                    grasp6d_plan=node.latest_grasp6d_plan,
                )
            )
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
        node.latest_grasp6d_plan = self._rich_plan(stamp_sec=1.0)
        node.latest_obj = self._object_at(0.46, 0.0, 0.20)

        original_get_param = grasp_task_node.rospy.get_param
        original_time_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.get_param = lambda name, default=None: default
        grasp_task_node.rospy.Time.now = staticmethod(lambda: FakeTime(2.0))
        try:
            result = grasp_task_node.GraspTaskNode._fresh_grasp6d_plan(
                node,
                {
                    'plan_validity_sec': 2.0,
                    'target_max_drift_m': 0.03,
                },
            )
        finally:
            grasp_task_node.rospy.get_param = original_get_param
            grasp_task_node.rospy.Time.now = original_time_now

        self.assertIsNone(result)

    def test_6d_plan_rejects_long_manual_confirmation_from_source_timestamp(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        plan = self._rich_plan(stamp_sec=1.0)
        node.latest_grasp6d_plan = plan
        node.latest_obj = self._object_at(0.41, 0.0, 0.20)

        original_get_param = grasp_task_node.rospy.get_param
        original_time_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.get_param = lambda name, default=None: default
        grasp_task_node.rospy.Time.now = staticmethod(lambda: FakeTime(121.0))
        try:
            result = grasp_task_node.GraspTaskNode._fresh_grasp6d_plan(
                node,
                {
                    'plan_validity_sec': 2.0,
                    'target_max_drift_m': 0.03,
                },
            )
        finally:
            grasp_task_node.rospy.get_param = original_get_param
            grasp_task_node.rospy.Time.now = original_time_now

        self.assertIsNone(result)
        self.assertIsNone(node.latest_grasp6d_plan)

    def test_negative_object_invalidation_removes_cached_targets_and_plan_without_motion(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node.active = True
        node.latest_obj = None
        node.latest_obj_time = None
        node.latest_visual_obj = None
        node.latest_visual_obj_time = None
        node.latest_grasp6d_plan = None

        original_get_param = grasp_task_node.rospy.get_param
        original_time_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.get_param = lambda name, default=None: default
        grasp_task_node.rospy.Time.now = staticmethod(lambda: FakeTime(1.0))
        try:
            detected = self._object_at(0.40, 0.0, 0.20)
            grasp_task_node.GraspTaskNode.obj_cb(node, detected)
            locked_for_active_flow = grasp_task_node.deepcopy(node.latest_obj)
            grasp_task_node.GraspTaskNode.grasp6d_plan_cb(
                node,
                self._rich_plan(stamp_sec=1.0),
            )
            grasp_task_node.GraspTaskNode.obj_cb(node, types.SimpleNamespace(detected=False))
            available_plan = grasp_task_node.GraspTaskNode._fresh_grasp6d_plan(
                node,
                {'plan_validity_sec': 2.0},
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

    def test_rich_6d_sequence_global_position_only_true_fails_before_simulation_or_action(self):
        _result, physical, states, payloads, node, _plan = (
            self._run_mujoco_gate_to_first_physical_action(
                response=lambda bound: self._passing_mujoco_response(
                    bound.plan_id
                ),
                position_only_execute_enabled=True,
            )
        )

        self.assertEqual(payloads, [])
        self.assertEqual(physical, [])
        self.assertIsNone(node._test_mujoco_audit)
        self.assertEqual(states[-1][0], grasp_task_node.GraspStages.FAILED)
        self.assertIn('POSITION_ONLY_FALLBACK_FORBIDDEN', states[-1][1])

    def test_rich_6d_plan_rejects_position_only_fallback_even_when_global_compatibility_is_enabled(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node.active = True
        states = []
        calls = []
        node.set_state = lambda *args, **kwargs: states.append(args)
        node._validate_bound_plan = lambda *_args: (
            grasp_task_node.PlanValidationResult(True)
        )
        node._wait_for_motion_settle = lambda reason='motion': calls.append(
            ('settle', reason)
        )

        def move_pose(_pose, execute):
            calls.append(('move', bool(execute)))
            return FakeServiceResponse(
                True,
                'planned with position-only fallback: target xyz=(0.1, 0.2, 0.3)',
            )

        original_get_param = grasp_task_node.rospy.get_param
        grasp_task_node.rospy.get_param = lambda name, default=None: {
            '/robot/position_only_execute_enabled': True,
        }.get(name, default)
        try:
            result = grasp_task_node.GraspTaskNode._plan_and_execute_pose(
                node,
                grasp_task_node.GraspStages.MOVE_PREGRASP,
                '6D pregrasp',
                self._pose(0.10),
                move_pose,
                '6D pregrasp',
                execution_plan=types.SimpleNamespace(plan_id='strict-rich-plan'),
                gcfg={},
            )
        finally:
            grasp_task_node.rospy.get_param = original_get_param

        self.assertFalse(result)
        self.assertEqual(calls, [])
        self.assertEqual(states[-1][0], grasp_task_node.GraspStages.FAILED)
        self.assertIn('POSITION_ONLY_FALLBACK_FORBIDDEN', states[-1][1])

    def test_non_6d_motion_keeps_explicit_position_only_compatibility(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node.active = True
        states = []
        calls = []
        node.set_state = lambda *args, **kwargs: states.append(args)
        node._wait_for_motion_settle = lambda reason='motion': (
            calls.append(('settle', reason)) or True
        )

        def move_pose(_pose, execute):
            calls.append(('move', bool(execute)))
            message = (
                'planned with position-only fallback: target xyz=(0.1, 0.2, 0.3)'
                if not execute
                else 'executed'
            )
            return FakeServiceResponse(True, message)

        original_get_param = grasp_task_node.rospy.get_param
        grasp_task_node.rospy.get_param = lambda name, default=None: {
            '/robot/position_only_execute_enabled': True,
        }.get(name, default)
        try:
            result = grasp_task_node.GraspTaskNode._plan_and_execute_pose(
                node,
                grasp_task_node.GraspStages.MOVE_PREGRASP,
                'legacy pregrasp',
                self._pose(0.10),
                move_pose,
                'legacy pregrasp',
            )
        finally:
            grasp_task_node.rospy.get_param = original_get_param

        self.assertTrue(result)
        self.assertEqual(
            calls,
            [
                ('move', False),
                ('move', True),
                ('settle', 'legacy pregrasp'),
            ],
        )

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

    def test_rich_6d_plan_rejects_orientation_fallback_even_when_global_compatibility_is_enabled(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node.active = True
        states = []
        calls = []
        node.set_state = lambda *args, **kwargs: states.append(args)
        node._validate_bound_plan = lambda *_args: (
            grasp_task_node.PlanValidationResult(True)
        )
        node._wait_for_motion_settle = lambda reason='motion': calls.append(
            ('settle', reason)
        )

        def move_pose(_pose, execute):
            calls.append(('move', bool(execute)))
            return FakeServiceResponse(
                True,
                'planned with candidate orientation current: target xyz=(0.1, 0.2, 0.3)',
            )

        original_get_param = grasp_task_node.rospy.get_param
        grasp_task_node.rospy.get_param = lambda name, default=None: {
            '/grasp/accept_orientation_fallback': True,
        }.get(name, default)
        try:
            result = grasp_task_node.GraspTaskNode._plan_and_execute_pose(
                node,
                grasp_task_node.GraspStages.MOVE_PREGRASP,
                '6D pregrasp',
                self._pose(0.10),
                move_pose,
                '6D pregrasp',
                execution_plan=types.SimpleNamespace(plan_id='strict-rich-plan'),
                gcfg={},
            )
        finally:
            grasp_task_node.rospy.get_param = original_get_param

        self.assertFalse(result)
        self.assertEqual(calls, [('move', False)])
        self.assertEqual(states[-1][0], grasp_task_node.GraspStages.FAILED)
        self.assertIn('candidate orientation', states[-1][1])

    def test_non_6d_motion_keeps_explicit_orientation_fallback_compatibility(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node.active = True
        states = []
        calls = []
        node.set_state = lambda *args, **kwargs: states.append(args)
        node._wait_for_motion_settle = lambda reason='motion': (
            calls.append(('settle', reason)) or True
        )

        def move_pose(_pose, execute):
            calls.append(('move', bool(execute)))
            message = (
                'planned with candidate orientation current: target xyz=(0.1, 0.2, 0.3)'
                if not execute
                else 'executed'
            )
            return FakeServiceResponse(True, message)

        original_get_param = grasp_task_node.rospy.get_param
        grasp_task_node.rospy.get_param = lambda name, default=None: {
            '/grasp/accept_orientation_fallback': True,
        }.get(name, default)
        try:
            result = grasp_task_node.GraspTaskNode._plan_and_execute_pose(
                node,
                grasp_task_node.GraspStages.MOVE_PREGRASP,
                'legacy pregrasp',
                self._pose(0.10),
                move_pose,
                'legacy pregrasp',
            )
        finally:
            grasp_task_node.rospy.get_param = original_get_param

        self.assertTrue(result)
        self.assertEqual(
            calls,
            [
                ('move', False),
                ('move', True),
                ('settle', 'legacy pregrasp'),
            ],
        )

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

    def test_visual_retarget_entry_point_only_checks_drift_and_never_translates(self):
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
                'target_max_drift_m': 0.04,
            },
            'pregrasp',
            required=True,
        )

        self.assertIsNotNone(result)
        guarded, updated_reference = result
        self.assertIs(updated_reference, reference)
        self.assertAlmostEqual(guarded[0].pose.position.x, 0.20)
        self.assertAlmostEqual(guarded[0].pose.position.y, 0.10)
        self.assertAlmostEqual(guarded[0].pose.position.z, 0.30)
        self.assertAlmostEqual(guarded[1].pose.position.x, 0.30)
        self.assertAlmostEqual(guarded[0].pose.orientation.x, 0.25)
        self.assertAlmostEqual(guarded[0].pose.orientation.w, 0.75)

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
                'target_max_drift_m': 0.04,
            },
            'pregrasp',
            required=True,
        )

        self.assertIsNone(result)
        self.assertEqual(states[-1][0], grasp_task_node.GraspStages.FAILED)
        self.assertIn('TARGET_DRIFT', states[-1][1])

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
