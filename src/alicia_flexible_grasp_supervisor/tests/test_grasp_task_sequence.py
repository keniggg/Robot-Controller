#!/usr/bin/env python3
import hashlib
import io
import importlib.util
import json
import math
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
            u=320,
            v=240,
            bbox_x=300,
            bbox_y=220,
            bbox_width=40,
            bbox_height=40,
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
        plan.candidate_source = 'graspnet'
        plan.candidate_source_lineage = ['graspnet']
        plan.has_candidate_model_width = True
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
            'candidate_source': 'graspnet',
            'candidate_source_lineage': ['graspnet'],
            'score': 95.0,
            'simulation_ok': True,
            'ik_success': True,
            'collision_free': True,
            'contact_success': True,
            'lift_success': True,
            'lift_evidence': {
                'contract_version': 1,
                'object_lift_m': 0.020,
                'commanded_lift_m': 0.050,
                'minimum_lift_m': 0.015,
                'two_sided_lift_samples': 39,
                'lost_contact_samples': 1,
                'lift_sample_count': 40,
                'max_lost_contact_streak': 1,
                'contact_loss_grace_samples': 3,
            },
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
        plan = node._freeze_execution_plan(plan)
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
                'type': 'obb_box',
                'density_upper_bound_kg_m3': 20000.0,
                'mass_floor_kg': 0.02,
                'operational_mass_ceiling_kg': 0.50,
                'friction_lower_bound': [0.10, 0.005, 0.0001],
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

    def test_preview_rich_topic_is_subscribed_as_diagnostics_only(self):
        source = SCRIPT.read_text(encoding='utf-8')
        self.assertIn("'/grasp_6d/preview_plan_enriched'", source)
        self.assertIn('self.grasp6d_preview_plan_cb', source)

    def test_grasp_state_is_latched_and_startup_publishes_idle(self):
        source = SCRIPT.read_text(encoding='utf-8')
        self.assertIn("'/grasp/state'", source)
        self.assertIn('latch=True', source)
        self.assertIn("self.set_state(GraspStages.IDLE, 'ready')", source)

    def test_near_field_phase_signal_is_latched_and_idempotent(self):
        class RecordingPublisher:
            def __init__(self):
                self.messages = []

            def publish(self, message):
                self.messages.append(message)

        node = grasp_task_node.GraspTaskNode.__new__(
            grasp_task_node.GraspTaskNode
        )
        node._near_field_active = None
        node.near_field_pub = RecordingPublisher()

        self.assertTrue(node._set_near_field_active(False, force=True))
        self.assertFalse(node._set_near_field_active(False))
        self.assertTrue(node._set_near_field_active(True))
        self.assertTrue(node._set_near_field_active(False))

        self.assertEqual(
            [message.data for message in node.near_field_pub.messages],
            [False, True, False],
        )
        source = SCRIPT.read_text(encoding='utf-8')
        self.assertIn("'/grasp/near_field_active'", source)

    def test_direct_near_field_phase_publishes_one_absolute_deadline(self):
        class RecordingPublisher:
            def __init__(self):
                self.messages = []

            def publish(self, message):
                self.messages.append(message)

        node = grasp_task_node.GraspTaskNode.__new__(
            grasp_task_node.GraspTaskNode
        )
        node._near_field_active = False
        node._near_field_phase_id = 0
        node._near_field_phase_deadline_sec = 0.0
        node.near_field_pub = RecordingPublisher()
        node.near_field_phase_pub = RecordingPublisher()
        original_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(100.0)
        )
        try:
            changed = node._set_near_field_active(
                True,
                budget_sec=30.0,
            )
        finally:
            grasp_task_node.rospy.Time.now = original_now

        self.assertTrue(changed)
        self.assertEqual(len(node.near_field_phase_pub.messages), 1)
        phase = node.near_field_phase_pub.messages[0]
        self.assertTrue(phase.active)
        self.assertEqual(phase.phase_id, 1)
        self.assertAlmostEqual(phase.header.stamp.to_sec(), 100.0)
        self.assertAlmostEqual(phase.deadline.to_sec(), 130.0)
        self.assertAlmostEqual(node._near_field_phase_deadline_sec, 130.0)

    def test_fresh_direct_terminal_preview_returns_exact_failure(self):
        node = grasp_task_node.GraspTaskNode.__new__(
            grasp_task_node.GraspTaskNode
        )
        bound = self._rich_plan(plan_id='bound', stamp_sec=9.0)
        bound.diagnostic = grasp_task_node._FAR_FIELD_OBSERVATION_PLAN
        bound.plan_id = compute_plan_id(bound)
        terminal = Grasp6DPlan()
        terminal.header.frame_id = 'base_link'
        terminal.header.stamp = grasp_task_node.rospy.Time.from_sec(10.1)
        terminal.valid = False
        terminal.candidate_source = 'near_field_terminal'
        terminal.diagnostic = (
            'NEAR_FIELD_DIRECT_TIMEOUT: shared phase deadline consumed'
        )
        node.latest_grasp6d_preview_plan = terminal
        node._grasp6d_plan_lock = threading.RLock()

        original_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(10.2)
        )
        try:
            result, candidate = node._copy_near_field_preview_candidate(
                bound,
                int(10.0 * 1e9),
                {
                    'near_field_strategy': 'single_snapshot_direct',
                    'plan_validity_sec': 5.0,
                },
            )
        finally:
            grasp_task_node.rospy.Time.now = original_now

        self.assertIsNone(candidate)
        self.assertFalse(result.ok)
        self.assertEqual(result.code, 'NEAR_FIELD_DIRECT_TIMEOUT')
        self.assertIn('shared phase deadline', result.reason)

    def test_active_execution_ignores_new_valid_plan_replacement(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        bound = self._rich_plan(plan_id='bound', stamp_sec=9.0)
        replacement = self._rich_plan(plan_id='replacement', stamp_sec=9.5)
        node.latest_grasp6d_plan = bound
        node.latest_obj = self._object(stamp_sec=9.9)
        node.latest_obj_time = grasp_task_node.rospy.Time.from_sec(9.9)
        node.active = True
        node._freeze_execution_plan(bound)
        original_get_param = grasp_task_node.rospy.get_param
        original_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.get_param = lambda name, default=None: {
            '/grasp_6d/plan_validity_sec': 2.0,
        }.get(name, default)
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(10.0)
        )
        try:
            node.grasp6d_plan_cb(replacement)
            result = node._validate_bound_plan(
                bound,
                {
                    'target_max_drift_m': 0.02,
                    'target_observation_validity_sec': 1.5,
                },
            )
        finally:
            grasp_task_node.rospy.get_param = original_get_param
            grasp_task_node.rospy.Time.now = original_now

        self.assertTrue(result.ok)
        self.assertEqual(node.latest_grasp6d_plan.plan_id, bound.plan_id)
        self.assertEqual(node._bound_execution_plan.plan_id, bound.plan_id)
        self.assertEqual(node._last_execution_plan_event, 'EXECUTION_FROZEN')

    def test_active_execution_ignores_unacceptable_valid_replacement(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        bound = self._rich_plan(plan_id='bound', stamp_sec=9.0)
        stale = self._rich_plan(plan_id='stale', stamp_sec=1.0)
        malformed = self._rich_plan(plan_id='malformed', stamp_sec=9.5)
        malformed.poses[0].position.x += 0.01
        node.latest_grasp6d_plan = bound
        node.latest_obj = self._object(stamp_sec=9.9)
        node.latest_obj_time = grasp_task_node.rospy.Time.from_sec(9.9)
        node.active = True
        node._freeze_execution_plan(bound)
        original_get_param = grasp_task_node.rospy.get_param
        original_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.get_param = lambda name, default=None: {
            '/grasp_6d/plan_validity_sec': 2.0,
        }.get(name, default)
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(10.0)
        )
        try:
            node.grasp6d_plan_cb(stale)
            node.grasp6d_plan_cb(malformed)
            result = node._validate_bound_plan(
                bound,
                {
                    'target_max_drift_m': 0.02,
                    'target_observation_validity_sec': 1.5,
                },
            )
        finally:
            grasp_task_node.rospy.get_param = original_get_param
            grasp_task_node.rospy.Time.now = original_now

        self.assertTrue(result.ok)
        self.assertFalse(node._execution_authority_revoked)
        self.assertEqual(node.latest_grasp6d_plan.plan_id, bound.plan_id)
        self.assertEqual(node._last_execution_plan_event, 'EXECUTION_FROZEN')

    def test_preview_callback_cannot_change_execution_authority(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        authority = self._rich_plan(plan_id='authority', stamp_sec=9.0)
        preview = self._rich_plan(plan_id='preview', stamp_sec=9.5)
        node.latest_grasp6d_plan = authority
        node.latest_grasp6d_preview_plan = None
        node.active = True
        node._freeze_execution_plan(authority)

        node.grasp6d_preview_plan_cb(preview)
        preview.poses[0].position.x = 99.0

        self.assertEqual(node.latest_grasp6d_plan.plan_id, authority.plan_id)
        self.assertEqual(node._bound_execution_plan.plan_id, authority.plan_id)
        self.assertEqual(node.latest_grasp6d_preview_plan.plan_id, preview.plan_id)
        self.assertAlmostEqual(
            node.latest_grasp6d_preview_plan.poses[0].position.x,
            0.10,
        )
        self.assertTrue(node.active)

    def test_final_visual_refinement_updates_only_bounded_xyz_and_yaw(self):
        current = self._rich_plan(plan_id='same', stamp_sec=9.0)
        observed = self._rich_plan(plan_id='same', stamp_sec=9.5)
        yaw = math.radians(5.0)
        for pose in observed.poses:
            pose.position.x += 0.005
            pose.position.y -= 0.003
            pose.orientation.z = math.sin(0.5 * yaw)
            pose.orientation.w = math.cos(0.5 * yaw)
        observed.object_geometry.pose_base.position.x += 0.005
        observed.object_geometry.pose_base.position.y -= 0.003
        observed.plan_id = compute_plan_id(observed)

        result, refined, metrics = (
            grasp_task_node.build_bounded_final_visual_refinement(
                current,
                observed,
                max_translation_m=0.012,
                max_yaw_rad=math.radians(10.0),
                max_roll_pitch_change_rad=math.radians(4.0),
            )
        )

        self.assertTrue(result.ok, result.reason)
        self.assertIsNotNone(refined)
        self.assertTrue(
            grasp_task_node.plan_id_matches_content(refined)
        )
        self.assertEqual(refined.header.stamp, observed.header.stamp)
        self.assertAlmostEqual(refined.poses[2].position.x, 0.305)
        self.assertAlmostEqual(refined.poses[2].position.y, -0.003)
        self.assertAlmostEqual(refined.poses[2].position.z, 0.2)
        self.assertAlmostEqual(
            metrics['translation_m'],
            math.sqrt(0.005 ** 2 + 0.003 ** 2),
        )
        self.assertAlmostEqual(metrics['yaw_rad'], yaw)
        approach_axis = grasp_task_node._rotate_vector(
            grasp_task_node._quaternion_xyzw(refined.poses[2]),
            (0.0, 0.0, 1.0),
        )
        self.assertAlmostEqual(approach_axis[0], 0.0, places=7)
        self.assertAlmostEqual(approach_axis[1], 0.0, places=7)
        self.assertAlmostEqual(approach_axis[2], 1.0, places=7)

    def test_final_visual_refinement_rejects_large_translation(self):
        current = self._rich_plan(plan_id='same', stamp_sec=9.0)
        observed = self._rich_plan(plan_id='same', stamp_sec=9.5)
        for pose in observed.poses:
            pose.position.x += 0.020
        observed.plan_id = compute_plan_id(observed)

        result, refined, _metrics = (
            grasp_task_node.build_bounded_final_visual_refinement(
                current,
                observed,
                max_translation_m=0.012,
                max_yaw_rad=math.radians(10.0),
                max_roll_pitch_change_rad=math.radians(4.0),
            )
        )

        self.assertFalse(result.ok)
        self.assertEqual(result.code, 'FINAL_REFINE_TRANSLATION_LIMIT')
        self.assertIsNone(refined)

    def test_final_visual_refinement_rejects_candidate_yaw_jump(self):
        current = self._rich_plan(plan_id='same', stamp_sec=9.0)
        observed = self._rich_plan(plan_id='same', stamp_sec=9.5)
        yaw = math.radians(25.0)
        for pose in observed.poses:
            pose.orientation.z = math.sin(0.5 * yaw)
            pose.orientation.w = math.cos(0.5 * yaw)
        observed.plan_id = compute_plan_id(observed)

        result, refined, _metrics = (
            grasp_task_node.build_bounded_final_visual_refinement(
                current,
                observed,
                max_translation_m=0.012,
                max_yaw_rad=math.radians(10.0),
                max_roll_pitch_change_rad=math.radians(4.0),
            )
        )

        self.assertFalse(result.ok)
        self.assertEqual(result.code, 'FINAL_REFINE_YAW_LIMIT')
        self.assertIsNone(refined)

    def test_final_visual_refinement_rejects_roll_pitch_change(self):
        current = self._rich_plan(plan_id='same', stamp_sec=9.0)
        observed = self._rich_plan(plan_id='same', stamp_sec=9.5)
        roll = math.radians(8.0)
        for pose in observed.poses:
            pose.orientation.x = math.sin(0.5 * roll)
            pose.orientation.w = math.cos(0.5 * roll)
        observed.plan_id = compute_plan_id(observed)

        result, refined, _metrics = (
            grasp_task_node.build_bounded_final_visual_refinement(
                current,
                observed,
                max_translation_m=0.012,
                max_yaw_rad=math.radians(10.0),
                max_roll_pitch_change_rad=math.radians(4.0),
            )
        )

        self.assertFalse(result.ok)
        self.assertEqual(result.code, 'FINAL_REFINE_ROLL_PITCH_LIMIT')
        self.assertIsNone(refined)

    def test_final_center_refinement_uses_surface_not_obb_center(self):
        current = self._rich_plan(plan_id='same', stamp_sec=9.0)
        stamp = grasp_task_node.rospy.Time.from_sec(9.5)
        original_poses = grasp_task_node.deepcopy(current.poses)
        original_width = current.required_open_width_m

        result, refined, metrics = (
            grasp_task_node.build_bounded_final_center_refinement(
                current,
                observed_surface_center_xyz=(0.405, -0.003, 0.232),
                observed_stamp=stamp,
                max_translation_m=0.025,
            )
        )

        self.assertTrue(result.ok, result.reason)
        self.assertTrue(
            grasp_task_node.plan_id_matches_content(refined)
        )
        self.assertEqual(refined.header.stamp, stamp)
        self.assertAlmostEqual(metrics['translation_xyz'][0], 0.005)
        self.assertAlmostEqual(metrics['translation_xyz'][1], -0.003)
        self.assertAlmostEqual(metrics['translation_xyz'][2], 0.002)
        self.assertAlmostEqual(
            metrics['translation_m'],
            math.sqrt(0.005 ** 2 + 0.003 ** 2 + 0.002 ** 2),
        )
        for original, corrected in zip(original_poses, refined.poses):
            self.assertAlmostEqual(
                corrected.position.x,
                original.position.x + 0.005,
            )
            self.assertAlmostEqual(
                corrected.position.y,
                original.position.y - 0.003,
            )
            self.assertAlmostEqual(
                corrected.position.z,
                original.position.z + 0.002,
            )
            self.assertEqual(corrected.orientation, original.orientation)
        self.assertAlmostEqual(
            refined.object_geometry.pose_base.position.x,
            0.405,
        )
        self.assertAlmostEqual(
            refined.object_geometry.pose_base.position.y,
            -0.003,
        )
        self.assertAlmostEqual(
            refined.object_geometry.pose_base.position.z,
            0.202,
        )
        self.assertEqual(refined.required_open_width_m, original_width)
        self.assertEqual(
            refined.candidate_source_lineage,
            current.candidate_source_lineage,
        )

    def test_final_center_refinement_rejects_large_translation(self):
        current = self._rich_plan(plan_id='same', stamp_sec=9.0)

        result, refined, _metrics = (
            grasp_task_node.build_bounded_final_center_refinement(
                current,
                observed_surface_center_xyz=(0.430, 0.0, 0.230),
                observed_stamp=grasp_task_node.rospy.Time.from_sec(9.5),
                max_translation_m=0.025,
            )
        )

        self.assertFalse(result.ok)
        self.assertEqual(result.code, 'FINAL_REFINE_TRANSLATION_LIMIT')
        self.assertIsNone(refined)

    def test_final_center_sample_requires_fresh_same_label_target(self):
        node = grasp_task_node.GraspTaskNode.__new__(
            grasp_task_node.GraspTaskNode
        )
        current = self._rich_plan(plan_id='same', stamp_sec=9.0)
        node.latest_obj = self._object_at(
            0.405,
            -0.003,
            0.232,
            stamp_sec=9.9,
        )
        node.latest_obj_time = grasp_task_node.rospy.Time.from_sec(9.9)
        node._grasp6d_plan_lock = threading.RLock()
        original_now = grasp_task_node.rospy.Time.now
        original_get_param = grasp_task_node.rospy.get_param
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(10.0)
        )
        grasp_task_node.rospy.get_param = lambda name, default=None: {
            '/camera': {'width': 640, 'height': 480},
        }.get(name, default)
        try:
            result, token, xyz, stamp = node._copy_final_center_sample(
                current,
                grasp_task_node.rospy.Time.from_sec(9.5).to_nsec(),
                {'target_observation_validity_sec': 1.5},
            )
        finally:
            grasp_task_node.rospy.Time.now = original_now
            grasp_task_node.rospy.get_param = original_get_param

        self.assertTrue(result.ok, result.reason)
        self.assertEqual(token, stamp.to_nsec())
        self.assertEqual(xyz, (0.405, -0.003, 0.232))

    def test_final_center_sample_rejects_mask_clipped_at_image_bottom(self):
        node = grasp_task_node.GraspTaskNode.__new__(
            grasp_task_node.GraspTaskNode
        )
        current = self._rich_plan(plan_id='same', stamp_sec=9.0)
        node.latest_obj = self._object_at(
            0.405,
            -0.003,
            0.232,
            stamp_sec=9.9,
        )
        node.latest_obj.bbox_x = 337
        node.latest_obj.bbox_y = 381
        node.latest_obj.bbox_width = 133
        node.latest_obj.bbox_height = 99
        node.latest_obj_time = grasp_task_node.rospy.Time.from_sec(9.9)
        node._grasp6d_plan_lock = threading.RLock()
        original_now = grasp_task_node.rospy.Time.now
        original_get_param = grasp_task_node.rospy.get_param
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(10.0)
        )
        grasp_task_node.rospy.get_param = lambda name, default=None: {
            '/camera': {'width': 640, 'height': 480},
        }.get(name, default)
        try:
            result, token, xyz, stamp = node._copy_final_center_sample(
                current,
                grasp_task_node.rospy.Time.from_sec(9.5).to_nsec(),
                {
                    'target_observation_validity_sec': 1.5,
                    'final_visual_refine_center_fallback_edge_margin_px': 4,
                },
            )
        finally:
            grasp_task_node.rospy.Time.now = original_now
            grasp_task_node.rospy.get_param = original_get_param

        self.assertFalse(result.ok)
        self.assertEqual(result.code, 'FINAL_REFINE_CENTER_CLIPPED')
        self.assertIn('clearance 0px', result.reason)
        self.assertIsNone(token)
        self.assertIsNone(xyz)
        self.assertIsNone(stamp)

    def test_final_refinement_falls_back_after_five_stable_centers(self):
        node = grasp_task_node.GraspTaskNode.__new__(
            grasp_task_node.GraspTaskNode
        )
        current = self._rich_plan(plan_id='same', stamp_sec=9.0)
        node.active = True
        node._grasp6d_plan_lock = threading.RLock()
        node.set_state = lambda *_args, **_kwargs: None
        node._request_near_field_preview_stream = lambda _gcfg: True
        node._copy_near_field_preview_candidate = (
            lambda *_args, **_kwargs: (
                grasp_task_node.PlanValidationResult(
                    False,
                    'GRIPPER_WIDTH_INVALID',
                    'target cloud is clipped',
                ),
                None,
            )
        )
        samples = [
            (0.4050, -0.0030, 0.2320),
            (0.4054, -0.0028, 0.2321),
            (0.4048, -0.0032, 0.2319),
            (0.4052, -0.0031, 0.2322),
            (0.4051, -0.0029, 0.2320),
        ]
        sample_index = [0]

        def copy_center(*_args, **_kwargs):
            index = min(sample_index[0], len(samples) - 1)
            sample_index[0] += 1
            stamp = grasp_task_node.rospy.Time.from_sec(
                10.01 + index * 0.01
            )
            return (
                grasp_task_node.PlanValidationResult(True),
                stamp.to_nsec(),
                samples[index],
                stamp,
            )

        node._copy_final_center_sample = copy_center
        checks = []
        simulations = []
        node._check_final_refine_sequence = (
            lambda candidate, _gcfg: (
                checks.append(candidate)
                or grasp_task_node.PlanValidationResult(
                    True,
                    reason='ordered sequence planned',
                )
            )
        )
        node._simulate_grasp6d_plan_if_required = (
            lambda _gcfg, _gripper_cfg, candidate: (
                simulations.append(candidate) or True
            )
        )
        original_now = grasp_task_node.rospy.Time.now
        original_sleep = grasp_task_node.rospy.sleep
        original_monotonic = grasp_task_node.time.monotonic
        clock = [0.0]

        def monotonic():
            clock[0] += 0.01
            return clock[0]

        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(10.0)
        )
        grasp_task_node.rospy.sleep = lambda *_args, **_kwargs: None
        grasp_task_node.time.monotonic = monotonic
        try:
            refined = node._maybe_final_refine_grasp6d_plan(
                {
                    'final_visual_refine_enabled': True,
                    'final_visual_refine_required': True,
                    'final_visual_refine_timeout_sec': 1.0,
                    'final_visual_refine_poll_sec': 0.02,
                    'final_visual_refine_snapshot_slack_sec': 1.0,
                    'final_visual_refine_max_translation_m': 0.025,
                    'final_visual_refine_center_fallback_enabled': True,
                    'final_visual_refine_center_fallback_delay_sec': 0.0,
                    'final_visual_refine_center_fallback_required_samples': 5,
                    'final_visual_refine_center_fallback_max_jitter_m': 0.006,
                },
                {},
                current,
            )
        finally:
            grasp_task_node.rospy.Time.now = original_now
            grasp_task_node.rospy.sleep = original_sleep
            grasp_task_node.time.monotonic = original_monotonic

        self.assertIsNotNone(refined)
        self.assertEqual(len(checks), 1)
        self.assertEqual(len(simulations), 1)
        self.assertEqual(refined.plan_id, checks[0].plan_id)
        self.assertEqual(refined.plan_id, simulations[0].plan_id)
        self.assertEqual(
            refined.poses[2].orientation,
            current.poses[2].orientation,
        )
        self.assertEqual(
            refined.required_open_width_m,
            current.required_open_width_m,
        )
        self.assertEqual(
            refined.header.stamp.to_nsec(),
            grasp_task_node.rospy.Time.from_sec(10.03).to_nsec(),
        )

    def test_unchanged_final_candidate_requires_fresh_post_move_confirmation(self):
        for confirmed in (True, False):
            with self.subTest(confirmed=confirmed):
                node = grasp_task_node.GraspTaskNode.__new__(
                    grasp_task_node.GraspTaskNode
                )
                current = self._rich_plan(
                    plan_id='unchanged',
                    stamp_sec=9.0,
                )
                node.active = True
                node._grasp6d_plan_lock = threading.RLock()
                node.set_state = lambda *_args, **_kwargs: None
                node._request_near_field_preview_stream = lambda _gcfg: True
                node._copy_near_field_preview_candidate = (
                    lambda *_args, **_kwargs: (
                        grasp_task_node.PlanValidationResult(
                            False,
                            'NEAR_FIELD_PLAN_UNCHANGED',
                            'same bound candidate',
                        ),
                        None,
                    )
                )
                confirmations = []
                node._confirm_final_center_alignment = (
                    lambda plan, _gcfg: (
                        confirmations.append(plan.plan_id) or confirmed
                    )
                )
                original_now = grasp_task_node.rospy.Time.now
                grasp_task_node.rospy.Time.now = staticmethod(
                    lambda: grasp_task_node.rospy.Time.from_sec(10.0)
                )
                try:
                    result = node._maybe_final_refine_grasp6d_plan(
                        {
                            'final_visual_refine_enabled': True,
                            'final_visual_refine_required': True,
                            'final_visual_refine_timeout_sec': 1.0,
                            'final_visual_refine_accept_unchanged_after_post_move_confirm': True,
                        },
                        {},
                        current,
                    )
                finally:
                    grasp_task_node.rospy.Time.now = original_now

                self.assertEqual(confirmations, [current.plan_id])
                if confirmed:
                    self.assertIs(result, current)
                else:
                    self.assertIsNone(result)

    def test_post_correction_confirmation_enforces_residual_limit(self):
        original_sleep = grasp_task_node.rospy.sleep
        original_monotonic = grasp_task_node.time.monotonic
        try:
            for offset_m, expected in ((0.004, True), (0.008, False)):
                with self.subTest(offset_m=offset_m):
                    node = grasp_task_node.GraspTaskNode.__new__(
                        grasp_task_node.GraspTaskNode
                    )
                    plan = self._rich_plan(
                        plan_id='confirm',
                        stamp_sec=9.0,
                    )
                    node.active = True
                    node.latest_obj_time = (
                        grasp_task_node.rospy.Time.from_sec(9.9)
                    )
                    node._grasp6d_plan_lock = threading.RLock()
                    states = []
                    node.set_state = (
                        lambda stage, message='', *_args, **_kwargs: (
                            states.append((stage, message))
                        )
                    )
                    sample_index = [0]

                    def copy_center(*_args, **_kwargs):
                        index = sample_index[0]
                        sample_index[0] += 1
                        stamp = grasp_task_node.rospy.Time.from_sec(
                            10.01 + index * 0.01
                        )
                        return (
                            grasp_task_node.PlanValidationResult(True),
                            stamp.to_nsec(),
                            (0.4 + offset_m, 0.0, 0.23),
                            stamp,
                        )

                    node._copy_final_center_sample = copy_center
                    clock = [0.0]

                    def monotonic():
                        clock[0] += 0.01
                        return clock[0]

                    grasp_task_node.rospy.sleep = (
                        lambda *_args, **_kwargs: None
                    )
                    grasp_task_node.time.monotonic = monotonic
                    result = node._confirm_final_center_alignment(
                        plan,
                        {
                            'final_visual_refine_post_move_confirm_enabled': True,
                            'final_visual_refine_post_move_confirm_timeout_sec': 1.0,
                            'final_visual_refine_post_move_confirm_required_samples': 5,
                            'final_visual_refine_post_move_confirm_max_jitter_m': 0.006,
                            'final_visual_refine_post_move_confirm_max_residual_m': 0.006,
                        },
                    )

                    self.assertIs(result, expected)
                    if expected:
                        self.assertIn(
                            'center confirmed',
                            states[-1][1],
                        )
                    else:
                        self.assertIn(
                            'FINAL_REFINE_RESIDUAL',
                            states[-1][1],
                        )
        finally:
            grasp_task_node.rospy.sleep = original_sleep
            grasp_task_node.time.monotonic = original_monotonic

    def test_final_refinement_strict_check_uses_full_ordered_service(self):
        node = grasp_task_node.GraspTaskNode.__new__(
            grasp_task_node.GraspTaskNode
        )
        plan = self._rich_plan(plan_id='same', stamp_sec=9.0)
        observed = []
        original_wait = grasp_task_node.rospy.wait_for_service
        original_proxy = grasp_task_node.rospy.ServiceProxy
        grasp_task_node.rospy.wait_for_service = (
            lambda name, timeout=None: observed.append(
                ('wait', name, timeout)
            )
        )

        def service_proxy(name, service_type):
            observed.append(('proxy', name, service_type))

            def invoke(targets, stage_names, linear):
                observed.append(
                    (
                        'invoke',
                        targets,
                        list(stage_names),
                        list(linear),
                    )
                )
                return types.SimpleNamespace(
                    success=True,
                    message='ordered sequence planned',
                    failure_code='',
                    failed_stage='',
                )

            return invoke

        grasp_task_node.rospy.ServiceProxy = service_proxy
        try:
            result = node._check_final_refine_sequence(
                plan,
                {'final_visual_refine_service_timeout_sec': 2.0},
            )
        finally:
            grasp_task_node.rospy.wait_for_service = original_wait
            grasp_task_node.rospy.ServiceProxy = original_proxy

        self.assertTrue(result.ok, result.reason)
        invoke = [item for item in observed if item[0] == 'invoke']
        self.assertEqual(len(invoke), 1)
        self.assertEqual(
            invoke[0][2],
            ['pregrasp', 'approach', 'grasp', 'lift'],
        )
        self.assertEqual(invoke[0][3], [False, True, True, True])

    def test_hard_tombstone_revokes_frozen_execution(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        bound = self._rich_plan(plan_id='bound', stamp_sec=9.0)
        node.latest_grasp6d_plan = bound
        node.latest_obj = self._object(stamp_sec=9.9)
        node.latest_obj_time = grasp_task_node.rospy.Time.from_sec(9.9)
        node.active = True
        node._freeze_execution_plan(bound)
        tombstone = self._rich_plan(plan_id='lost', stamp_sec=9.5)
        tombstone.valid = False
        tombstone.diagnostic = 'TARGET_LOST: target disappeared'
        original_get_param = grasp_task_node.rospy.get_param
        original_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.get_param = lambda name, default=None: {
            '/grasp_6d/plan_validity_sec': 2.0,
        }.get(name, default)
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(10.0)
        )
        try:
            node.grasp6d_plan_cb(tombstone)
            result = node._validate_bound_plan(bound, {})
        finally:
            grasp_task_node.rospy.get_param = original_get_param
            grasp_task_node.rospy.Time.now = original_now

        self.assertFalse(result.ok)
        self.assertEqual(result.code, 'EXECUTION_AUTHORITY_REVOKED')
        self.assertTrue(node._execution_authority_revoked)

    def test_explicit_stop_revokes_frozen_execution_without_inference_control(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        bound = self._rich_plan(plan_id='bound', stamp_sec=9.0)
        node.latest_grasp6d_plan = bound
        node.latest_obj = self._object(stamp_sec=9.9)
        node.latest_obj_time = grasp_task_node.rospy.Time.from_sec(9.9)
        node.active = True
        node._start_inflight = True
        node.set_state = lambda *args, **kwargs: None
        node._freeze_execution_plan(bound)
        inference_calls = []
        node.request_plan = lambda *_args, **_kwargs: inference_calls.append(True)

        response = node.stop_cb(types.SimpleNamespace(emergency=True))
        result = node._validate_bound_plan(bound, {})

        self.assertTrue(response.success)
        self.assertFalse(node.active)
        self.assertTrue(node._execution_authority_revoked)
        self.assertFalse(result.ok)
        self.assertEqual(result.code, 'EXECUTION_CANCELLED')
        self.assertEqual(inference_calls, [])
        self.assertIsNotNone(node._bound_execution_plan)

    def test_hard_target_loss_revokes_frozen_execution(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        bound = self._rich_plan(plan_id='bound', stamp_sec=9.0)
        node.latest_grasp6d_plan = bound
        node.latest_obj = self._object(stamp_sec=9.9)
        node.latest_obj_time = grasp_task_node.rospy.Time.from_sec(9.9)
        node.latest_visual_obj = node.latest_obj
        node.latest_visual_obj_time = node.latest_obj_time
        node.active = True
        node._freeze_execution_plan(bound)

        lost = self._object(stamp_sec=10.0)
        lost.detected = False
        original_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(10.0)
        )
        try:
            node.obj_cb(lost)
            result = node._validate_bound_plan(bound, {})
        finally:
            grasp_task_node.rospy.Time.now = original_now

        self.assertTrue(node._execution_authority_revoked)
        self.assertFalse(result.ok)
        self.assertEqual(result.code, 'EXECUTION_AUTHORITY_REVOKED')

    def test_close_range_target_occlusion_after_approach_preserves_frozen_execution(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        bound = self._rich_plan(plan_id='bound', stamp_sec=9.0)
        visible = self._object(stamp_sec=9.9)
        node.latest_grasp6d_plan = bound
        node.latest_obj = visible
        node.latest_obj_time = grasp_task_node.rospy.Time.from_sec(9.9)
        node.latest_visual_obj = visible
        node.latest_visual_obj_time = node.latest_obj_time
        node.active = True
        node._freeze_execution_plan(bound)
        node._bound_target_occlusion_allowed = True

        lost = self._object(stamp_sec=10.0)
        lost.detected = False
        original_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(10.0)
        )
        try:
            node.obj_cb(lost)
            result = node._validate_bound_plan(
                bound,
                {
                    'target_max_drift_m': 0.02,
                    'target_observation_validity_sec': 1.5,
                },
            )
        finally:
            grasp_task_node.rospy.Time.now = original_now

        self.assertFalse(node._execution_authority_revoked)
        self.assertTrue(result.ok)
        self.assertIsNotNone(node.latest_obj)
        self.assertTrue(node.latest_obj.detected)

    def test_close_range_occlusion_allows_stale_cached_target_after_approach(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        bound = self._rich_plan(plan_id='bound', stamp_sec=9.0)
        node.latest_grasp6d_plan = bound
        node.latest_obj = self._object(stamp_sec=7.0)
        node.latest_obj_time = grasp_task_node.rospy.Time.from_sec(7.0)
        node.active = True
        node._freeze_execution_plan(bound)
        node._bound_target_occlusion_allowed = True
        original_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(10.0)
        )
        try:
            result = node._validate_bound_plan(
                bound,
                {
                    'target_max_drift_m': 0.02,
                    'target_observation_validity_sec': 1.5,
                },
            )
        finally:
            grasp_task_node.rospy.Time.now = original_now

        self.assertFalse(node._execution_authority_revoked)
        self.assertTrue(result.ok)

    def test_near_field_preview_drift_rejects_without_revoking_bound_plan(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        bound = self._rich_plan(plan_id='bound', stamp_sec=9.0)
        preview = self._rich_plan(plan_id='preview', stamp_sec=10.0)
        preview.object_geometry.pose_base.position.x = 0.80
        preview.plan_id = compute_plan_id(preview)
        node.latest_grasp6d_plan = bound
        node.latest_grasp6d_preview_plan = preview
        node.latest_obj = self._object_at(0.40, 0.0, 0.20, stamp_sec=10.0)
        node.latest_obj_time = grasp_task_node.rospy.Time.from_sec(10.0)
        node.active = True
        node._freeze_execution_plan(bound)

        original_now = grasp_task_node.rospy.Time.now
        original_get_param = grasp_task_node.rospy.get_param
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(10.1)
        )
        grasp_task_node.rospy.get_param = lambda _name, default=None: default
        try:
            result, candidate = node._copy_near_field_preview_candidate(
                bound,
                grasp_task_node.rospy.Time.from_sec(9.5).to_nsec(),
                {
                    'plan_validity_sec': 5.0,
                    'target_max_drift_m': 0.02,
                    'target_observation_validity_sec': 1.5,
                },
            )
        finally:
            grasp_task_node.rospy.Time.now = original_now
            grasp_task_node.rospy.get_param = original_get_param

        self.assertFalse(result.ok)
        self.assertEqual(result.code, 'TARGET_DRIFT')
        self.assertIsNone(candidate)
        self.assertFalse(node._execution_authority_revoked)
        self.assertEqual(node._bound_execution_plan.plan_id, bound.plan_id)

    def test_observation_camera_target_range_is_inclusive(self):
        node = grasp_task_node.GraspTaskNode.__new__(
            grasp_task_node.GraspTaskNode
        )
        plan = self._rich_plan(plan_id='observation-range')
        config = {
            'observation_camera_target_range_check_enabled': True,
            'observation_camera_target_min_distance_m': 0.180,
            'observation_camera_target_max_distance_m': 0.220,
        }
        for distance in (0.180, 0.1808542744, 0.200, 0.220):
            with self.subTest(distance=distance):
                node._current_camera_pose_base = lambda distance=distance: (
                    self._pose(0.40 - distance, 0.0, 0.20)
                )
                result = node._observation_camera_target_range_result(
                    plan,
                    config,
                )
                self.assertTrue(result.ok)

    def test_observation_camera_target_range_rejects_too_close_view(self):
        node = grasp_task_node.GraspTaskNode.__new__(
            grasp_task_node.GraspTaskNode
        )
        plan = self._rich_plan(plan_id='observation-too-close')
        node._current_camera_pose_base = lambda: self._pose(
            0.40 - 0.179,
            0.0,
            0.20,
        )

        result = node._observation_camera_target_range_result(
            plan,
            {
                'observation_camera_target_range_check_enabled': True,
                'observation_camera_target_min_distance_m': 0.180,
                'observation_camera_target_max_distance_m': 0.220,
            },
        )

        self.assertFalse(result.ok)
        self.assertEqual(
            result.code,
            'OBSERVATION_CAMERA_TARGET_OUT_OF_RANGE',
        )
        self.assertIn('0.1790m', result.reason)

    def test_observation_camera_target_range_uses_fresh_live_center(self):
        node = grasp_task_node.GraspTaskNode.__new__(
            grasp_task_node.GraspTaskNode
        )
        plan = self._rich_plan(plan_id='observation-live-center')
        node._current_camera_pose_base = lambda: self._pose(
            0.20,
            0.0,
            0.20,
        )

        result = node._observation_camera_target_range_result(
            plan,
            {
                'observation_camera_target_range_check_enabled': True,
                'observation_camera_target_min_distance_m': 0.180,
                'observation_camera_target_max_distance_m': 0.220,
            },
            target_xyz=(0.375, 0.0, 0.20),
        )

        self.assertFalse(result.ok)
        self.assertEqual(
            result.code,
            'OBSERVATION_CAMERA_TARGET_OUT_OF_RANGE',
        )
        self.assertIn('0.1750m', result.reason)

    def test_out_of_range_observation_view_is_not_reusable(self):
        node = grasp_task_node.GraspTaskNode.__new__(
            grasp_task_node.GraspTaskNode
        )
        plan = self._rich_plan(plan_id='reuse-range')
        node._grasp6d_plan_lock = threading.RLock()
        node.latest_obj = self._object_at(0.40, 0.0, 0.20)
        node._current_camera_pose_base = lambda: self._pose(
            0.225,
            0.0,
            0.20,
        )

        reusable = node._current_observation_view_reusable(
            plan,
            {
                'observation_camera_target_range_check_enabled': True,
                'observation_camera_target_min_distance_m': 0.180,
                'observation_camera_target_max_distance_m': 0.220,
            },
        )

        self.assertFalse(reusable)

    def test_observation_retreat_is_live_radial_and_error_compensated(self):
        current = self._pose(1.0, 2.0, 3.0)
        current.pose.orientation.x = 0.25
        current.pose.orientation.w = math.sqrt(1.0 - 0.25 ** 2)

        corrected, audit = (
            grasp_task_node.make_observation_camera_retreat_pose(
                current,
                camera_position_xyz=(0.18, 0.0, 0.0),
                target_position_xyz=(0.0, 0.0, 0.0),
                nominal_distance_m=0.20,
                execution_error_vector_xyz=(0.01, -0.02, 0.03),
            )
        )

        self.assertAlmostEqual(corrected.pose.position.x, 1.01)
        self.assertAlmostEqual(corrected.pose.position.y, 2.02)
        self.assertAlmostEqual(corrected.pose.position.z, 2.97)
        self.assertAlmostEqual(
            corrected.pose.orientation.x,
            current.pose.orientation.x,
        )
        self.assertAlmostEqual(
            corrected.pose.orientation.w,
            current.pose.orientation.w,
        )
        self.assertEqual(
            audit['correction_policy'],
            'single_out_of_range_radial_correction_with_current_endpoint_error_feedforward',
        )
        self.assertEqual(audit['correction_direction'], 'retreat')
        for actual, expected in zip(
            audit['radial_correction_xyz_m'],
            (0.02, 0.0, 0.0),
        ):
            self.assertAlmostEqual(actual, expected)
        for actual, expected in zip(
            audit['execution_error_feedforward_xyz_m'],
            (-0.01, 0.02, -0.03),
        ):
            self.assertAlmostEqual(actual, expected)

    def test_observation_too_far_correction_is_live_radial_and_bounded_to_nominal(self):
        current = self._pose(1.0, 2.0, 3.0)

        corrected, audit = (
            grasp_task_node.make_observation_camera_retreat_pose(
                current,
                camera_position_xyz=(0.26, 0.0, 0.0),
                target_position_xyz=(0.0, 0.0, 0.0),
                nominal_distance_m=0.20,
                execution_error_vector_xyz=(0.01, 0.0, 0.0),
            )
        )

        self.assertAlmostEqual(corrected.pose.position.x, 0.93)
        self.assertAlmostEqual(corrected.pose.position.y, 2.0)
        self.assertAlmostEqual(corrected.pose.position.z, 3.0)
        self.assertEqual(audit['correction_direction'], 'approach')
        for actual, expected in zip(
            audit['radial_correction_xyz_m'],
            (-0.06, 0.0, 0.0),
        ):
            self.assertAlmostEqual(actual, expected)
        self.assertAlmostEqual(
            audit['command_translation_norm_m'],
            0.07,
        )

    def test_observation_retreat_executes_one_strict_preflight_and_move(self):
        node = grasp_task_node.GraspTaskNode.__new__(
            grasp_task_node.GraspTaskNode
        )
        plan = self._rich_plan(plan_id='retreat-plan', stamp_sec=9.0)
        plan.diagnostic = grasp_task_node._FAR_FIELD_OBSERVATION_PLAN
        node._last_observation_camera_range_evidence = {
            'camera_position_m': [0.18, 0.0, 0.20],
            'target_position_m': [0.0, 0.0, 0.20],
            'distance_m': 0.18,
        }
        node._last_measured_endpoint_sample = {
            'plan_id': plan.plan_id,
            'plan_phase': grasp_task_node._FAR_FIELD_OBSERVATION_PLAN,
            'position_error_vector_m': [0.01, 0.0, 0.0],
        }
        node._current_tool_pose_base = lambda: self._pose(1.0, 2.0, 3.0)
        node.set_state = lambda *_args, **_kwargs: None
        preflight = []
        executed = []
        execute_kwargs = []

        def plan_pose(pose, execute):
            preflight.append((pose, bool(execute)))
            return FakeServiceResponse(True, 'planned')

        node._plan_and_execute_pose = (
            lambda _stage, _state, pose, *_args, **kwargs: (
                execute_kwargs.append(kwargs)
                or executed.append(pose)
                or True
            )
        )
        result = node._maybe_execute_observation_camera_retreat(
            plan,
            {
                'observation_camera_target_retreat_correction_enabled': True,
                'observation_camera_target_min_distance_m': 0.190,
                'observation_camera_target_nominal_distance_m': 0.200,
            },
            grasp_task_node.PlanValidationResult(
                False,
                'OBSERVATION_CAMERA_TARGET_OUT_OF_RANGE',
                'too close',
            ),
            plan_pose,
            object(),
        )

        self.assertTrue(result)
        self.assertEqual(len(preflight), 1)
        self.assertFalse(preflight[0][1])
        self.assertEqual(len(executed), 1)
        self.assertTrue(
            execute_kwargs[0][
                'allow_post_failure_observation_validation'
            ]
        )
        self.assertAlmostEqual(executed[0].pose.position.x, 1.01)
        self.assertAlmostEqual(executed[0].pose.position.y, 2.0)
        self.assertAlmostEqual(executed[0].pose.position.z, 3.0)

    def test_observation_too_far_executes_one_strict_radial_correction(self):
        node = grasp_task_node.GraspTaskNode.__new__(
            grasp_task_node.GraspTaskNode
        )
        plan = self._rich_plan(plan_id='approach-plan', stamp_sec=9.0)
        plan.diagnostic = grasp_task_node._FAR_FIELD_OBSERVATION_PLAN
        node._last_observation_camera_range_evidence = {
            'camera_position_m': [0.26, 0.0, 0.20],
            'target_position_m': [0.0, 0.0, 0.20],
            'distance_m': 0.26,
        }
        node._last_measured_endpoint_sample = {
            'plan_id': plan.plan_id,
            'plan_phase': grasp_task_node._FAR_FIELD_OBSERVATION_PLAN,
            'position_error_vector_m': [0.01, 0.0, 0.0],
        }
        node._current_tool_pose_base = lambda: self._pose(1.0, 2.0, 3.0)
        node.set_state = lambda *_args, **_kwargs: None
        preflight = []
        executed = []

        def plan_pose(pose, execute):
            preflight.append((pose, bool(execute)))
            return FakeServiceResponse(True, 'planned')

        node._plan_and_execute_pose = (
            lambda _stage, _state, pose, *_args, **_kwargs: (
                executed.append(pose) or True
            )
        )
        result = node._maybe_execute_observation_camera_retreat(
            plan,
            {
                'observation_camera_target_retreat_correction_enabled': True,
                'observation_camera_target_min_distance_m': 0.180,
                'observation_camera_target_max_distance_m': 0.220,
                'observation_camera_target_nominal_distance_m': 0.200,
            },
            grasp_task_node.PlanValidationResult(
                False,
                'OBSERVATION_CAMERA_TARGET_OUT_OF_RANGE',
                'too far',
            ),
            plan_pose,
            object(),
        )

        self.assertTrue(result)
        self.assertEqual(len(preflight), 1)
        self.assertFalse(preflight[0][1])
        self.assertEqual(len(executed), 1)
        self.assertAlmostEqual(executed[0].pose.position.x, 0.93)
        self.assertAlmostEqual(executed[0].pose.position.y, 2.0)
        self.assertAlmostEqual(executed[0].pose.position.z, 3.0)

    def test_post_arrival_range_wait_uses_new_source_sample(self):
        node = grasp_task_node.GraspTaskNode.__new__(
            grasp_task_node.GraspTaskNode
        )
        plan = self._rich_plan(plan_id='post-arrival-range', stamp_sec=9.0)
        node.active = True
        node._grasp6d_plan_lock = threading.RLock()
        node._bound_target_occlusion_allowed = False
        node.latest_obj = self._object_at(
            0.40,
            0.0,
            0.20,
            stamp_sec=10.1,
        )
        node.latest_obj_time = grasp_task_node.rospy.Time.from_sec(10.1)
        node._current_camera_pose_base = lambda: self._pose(
            0.225,
            0.0,
            0.20,
        )

        original_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(10.2)
        )
        try:
            result = node._wait_for_fresh_observation_camera_target_range(
                plan,
                {
                    'observation_camera_target_range_check_enabled': True,
                    'observation_camera_target_min_distance_m': 0.190,
                    'observation_camera_target_max_distance_m': 0.210,
                    'target_observation_validity_sec': 1.5,
                    'target_max_drift_m': 0.04,
                },
                grasp_task_node.rospy.Time.from_sec(10.0).to_nsec(),
            )
        finally:
            grasp_task_node.rospy.Time.now = original_now

        self.assertFalse(result.ok)
        self.assertEqual(
            result.code,
            'OBSERVATION_CAMERA_TARGET_OUT_OF_RANGE',
        )
        self.assertIn('0.1750m', result.reason)

    def test_far_field_range_failure_prevents_near_field_request(self):
        node = grasp_task_node.GraspTaskNode.__new__(
            grasp_task_node.GraspTaskNode
        )
        plan = self._rich_plan(plan_id='far-range-fail', stamp_sec=9.0)
        plan.diagnostic = grasp_task_node._FAR_FIELD_OBSERVATION_PLAN
        node.active = True
        node._near_field_active = False
        node._grasp6d_plan_lock = threading.RLock()
        node._bound_target_occlusion_allowed = False
        states = []
        node.set_state = lambda *args, **kwargs: states.append(args)
        node._position_only_execute_globally_enabled = lambda: False
        node._execution_checkpoint = lambda *_args, **_kwargs: True
        node._command_gripper_position = lambda *_args, **_kwargs: True
        node._converge_far_field_observation_endpoint = (
            lambda *_args, **_kwargs: True
        )
        node._wait_for_fresh_observation_camera_target_range = (
            lambda *_args, **_kwargs: grasp_task_node.PlanValidationResult(
                False,
                'OBSERVATION_CAMERA_TARGET_OUT_OF_RANGE',
                'fresh camera-to-target distance 0.1750m is outside the interval',
            )
        )
        near_field_calls = []
        node._maybe_rebind_near_field_grasp6d_plan = (
            lambda *_args, **_kwargs: near_field_calls.append(True) or plan
        )
        move_labels = []
        move_kwargs = []
        node._plan_and_execute_pose = (
            lambda _stage, label, *_args, **kwargs: (
                move_labels.append(label)
                or move_kwargs.append(kwargs)
                or True
            )
        )

        original_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(10.0)
        )
        try:
            result = node._execute_grasp6d_plan(
                {
                    'plan_validity_sec': 5.0,
                    'near_field_replan_enabled': True,
                    'near_field_replan_required': True,
                    'observation_camera_target_range_check_enabled': True,
                },
                {'open_position_m': 0.05},
                0.05,
                lambda _pose, _execute: FakeServiceResponse(
                    True,
                    'observation preflight planned',
                ),
                object(),
                object(),
                None,
                plan,
                strict_execute_pose=lambda *_args, **_kwargs: None,
            )
        finally:
            grasp_task_node.rospy.Time.now = original_now

        self.assertFalse(result)
        self.assertEqual(move_labels, ['6D pregrasp'])
        self.assertTrue(
            move_kwargs[0]['allow_post_failure_observation_validation']
        )
        self.assertEqual(near_field_calls, [])
        self.assertFalse(node._near_field_active)
        self.assertIn(
            'OBSERVATION_CAMERA_TARGET_OUT_OF_RANGE',
            states[-1][1],
        )

    def test_near_field_replan_binds_preview_and_runs_mujoco_gate(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        bound = self._rich_plan(plan_id='bound', stamp_sec=9.0)
        preview = self._rich_plan(plan_id='preview', stamp_sec=10.0)
        node.latest_grasp6d_plan = bound
        node.latest_grasp6d_preview_plan = preview
        node.latest_obj = self._object_at(0.40, 0.0, 0.20, stamp_sec=10.0)
        node.latest_obj_time = grasp_task_node.rospy.Time.from_sec(10.0)
        node.active = True
        node._near_field_phase_started_sec = 10.0
        node._near_field_phase_deadline_sec = 40.0
        node.set_state = lambda *args, **kwargs: None
        node._freeze_execution_plan(bound)
        calls = []
        node._request_near_field_preview_stream = (
            lambda _gcfg: calls.append('request') or True
        )
        node._simulate_grasp6d_plan_if_required = (
            lambda _gcfg, _gripper_cfg, plan: (
                calls.append(('simulate', plan.plan_id)) or True
            )
        )

        original_now = grasp_task_node.rospy.Time.now
        original_get_param = grasp_task_node.rospy.get_param
        original_sleep = grasp_task_node.rospy.sleep
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(10.05)
        )
        grasp_task_node.rospy.get_param = lambda _name, default=None: default
        grasp_task_node.rospy.sleep = lambda *_args, **_kwargs: None
        try:
            result = node._maybe_rebind_near_field_grasp6d_plan(
                {
                    'near_field_replan_enabled': True,
                    'near_field_replan_required': True,
                    'near_field_replan_timeout_sec': 0.1,
                    'near_field_replan_snapshot_slack_sec': 1.0,
                    'plan_validity_sec': 5.0,
                    'target_max_drift_m': 0.02,
                    'target_observation_validity_sec': 1.5,
                },
                {},
                bound,
            )
        finally:
            grasp_task_node.rospy.Time.now = original_now
            grasp_task_node.rospy.get_param = original_get_param
            grasp_task_node.rospy.sleep = original_sleep

        self.assertIsNotNone(result)
        self.assertEqual(result.plan_id, preview.plan_id)
        self.assertEqual(node._bound_execution_plan.plan_id, preview.plan_id)
        self.assertEqual(calls, ['request', ('simulate', preview.plan_id)])

    def test_direct_near_field_replan_freezes_once_without_duplicate_simulation(
        self,
    ):
        node = grasp_task_node.GraspTaskNode.__new__(
            grasp_task_node.GraspTaskNode
        )
        bound = self._rich_plan(plan_id='bound', stamp_sec=9.0)
        bound.diagnostic = grasp_task_node._FAR_FIELD_OBSERVATION_PLAN
        bound.plan_id = compute_plan_id(bound)
        preview = self._rich_plan(plan_id='preview', stamp_sec=10.0)
        preview.diagnostic = grasp_task_node._CONTACT_EXECUTION_PLAN
        preview.plan_id = compute_plan_id(preview)
        node.latest_grasp6d_plan = bound
        node.latest_grasp6d_preview_plan = preview
        node.latest_obj = self._object_at(
            0.40,
            0.0,
            0.20,
            stamp_sec=10.0,
        )
        node.latest_obj_time = grasp_task_node.rospy.Time.from_sec(10.0)
        node.active = True
        node._near_field_phase_started_sec = 10.0
        node._near_field_phase_deadline_sec = 40.0
        node.set_state = lambda *args, **kwargs: None
        node._freeze_execution_plan(bound)
        stream_calls = []
        node._set_near_field_preview_stream = (
            lambda _gcfg, enabled: (
                stream_calls.append(bool(enabled)) or True
            )
        )
        node._request_near_field_preview_stream = (
            lambda _gcfg: self.fail(
                'direct mode must use the explicit stream state setter'
            )
        )
        node._simulate_grasp6d_plan_if_required = (
            lambda *_args, **_kwargs: self.fail(
                'direct near-field selection already performed strict '
                'MoveIt and must not repeat task-level MuJoCo'
            )
        )

        original_now = grasp_task_node.rospy.Time.now
        original_get_param = grasp_task_node.rospy.get_param
        original_sleep = grasp_task_node.rospy.sleep
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(10.05)
        )
        grasp_task_node.rospy.get_param = lambda _name, default=None: default
        grasp_task_node.rospy.sleep = lambda *_args, **_kwargs: None
        try:
            result = node._maybe_rebind_near_field_grasp6d_plan(
                {
                    'near_field_strategy': 'single_snapshot_direct',
                    'near_field_replan_enabled': True,
                    'near_field_replan_required': True,
                    'near_field_replan_timeout_sec': 30.0,
                    'near_field_replan_snapshot_slack_sec': 1.0,
                    'plan_validity_sec': 5.0,
                    'target_max_drift_m': 0.02,
                    'target_observation_validity_sec': 1.5,
                },
                {},
                bound,
            )
        finally:
            grasp_task_node.rospy.Time.now = original_now
            grasp_task_node.rospy.get_param = original_get_param
            grasp_task_node.rospy.sleep = original_sleep

        self.assertIsNotNone(result)
        self.assertEqual(result.plan_id, preview.plan_id)
        self.assertEqual(node._bound_execution_plan.plan_id, preview.plan_id)
        self.assertEqual(stream_calls, [True, False])
        self.assertTrue(node._bound_target_occlusion_allowed)

    def test_direct_near_field_occlusion_preserves_frozen_authority(
        self,
    ):
        node = grasp_task_node.GraspTaskNode.__new__(
            grasp_task_node.GraspTaskNode
        )
        bound = self._rich_plan(stamp_sec=9.0)
        bound.diagnostic = grasp_task_node._CONTACT_EXECUTION_PLAN
        bound.plan_id = compute_plan_id(bound)
        visible = self._object(stamp_sec=9.9)
        node.latest_grasp6d_plan = bound
        node.latest_obj = visible
        node.latest_obj_time = grasp_task_node.rospy.Time.from_sec(9.9)
        node.latest_visual_obj = visible
        node.latest_visual_obj_time = node.latest_obj_time
        node.active = True
        node._freeze_execution_plan(
            bound,
            allow_target_occlusion=True,
        )

        lost = self._object(stamp_sec=10.0)
        lost.detected = False
        tombstone = self._rich_plan(stamp_sec=10.0)
        tombstone.valid = False
        tombstone.diagnostic = 'TARGET_LOST: expected near-field occlusion'
        original_now = grasp_task_node.rospy.Time.now
        original_get_param = grasp_task_node.rospy.get_param
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(10.1)
        )
        grasp_task_node.rospy.get_param = lambda _name, default=None: default
        try:
            node.obj_cb(lost)
            node.grasp6d_plan_cb(tombstone)
            result = node._validate_bound_plan(
                bound,
                {
                    'target_max_drift_m': 0.02,
                    'target_observation_validity_sec': 1.5,
                },
            )
        finally:
            grasp_task_node.rospy.Time.now = original_now
            grasp_task_node.rospy.get_param = original_get_param

        self.assertFalse(node._execution_authority_revoked)
        self.assertIs(node.latest_obj, visible)
        self.assertTrue(result.ok, result.reason)

    def test_direct_near_field_timeout_has_exact_status_and_stops_stream(self):
        node = grasp_task_node.GraspTaskNode.__new__(
            grasp_task_node.GraspTaskNode
        )
        bound = self._rich_plan(plan_id='bound', stamp_sec=9.0)
        node.latest_grasp6d_plan = bound
        node.latest_grasp6d_preview_plan = None
        node.active = True
        node._near_field_phase_started_sec = 9.95
        node._near_field_phase_deadline_sec = 10.20
        states = []
        node.set_state = lambda stage, message='', *_args, **_kwargs: (
            states.append((stage, message))
        )
        node._freeze_execution_plan(bound)
        stream_calls = []
        node._set_near_field_preview_stream = (
            lambda _gcfg, enabled: (
                stream_calls.append(bool(enabled)) or True
            )
        )
        node._request_near_field_preview_stream = lambda _gcfg: True
        node._copy_near_field_preview_candidate = (
            lambda *_args, **_kwargs: (
                grasp_task_node.PlanValidationResult(
                    False,
                    'NEAR_FIELD_PLAN_WAITING',
                    'no current direct preview',
                ),
                None,
            )
        )

        original_now = grasp_task_node.rospy.Time.now
        original_sleep = grasp_task_node.rospy.sleep
        clock = [10.0]

        def now():
            clock[0] += 0.06
            return grasp_task_node.rospy.Time.from_sec(clock[0])

        grasp_task_node.rospy.Time.now = staticmethod(
            now
        )
        grasp_task_node.rospy.sleep = lambda *_args, **_kwargs: None
        try:
            result = node._maybe_rebind_near_field_grasp6d_plan(
                {
                    'near_field_strategy': 'single_snapshot_direct',
                    'near_field_replan_enabled': True,
                    'near_field_replan_required': True,
                    'near_field_replan_timeout_sec': 0.1,
                    'near_field_replan_snapshot_slack_sec': 1.0,
                    'plan_validity_sec': 5.0,
                },
                {},
                bound,
            )
        finally:
            grasp_task_node.rospy.Time.now = original_now
            grasp_task_node.rospy.sleep = original_sleep

        self.assertIsNone(result)
        self.assertEqual(stream_calls, [True, False])
        self.assertEqual(states[-1][0], grasp_task_node.GraspStages.FAILED)
        self.assertIn('NEAR_FIELD_DIRECT_TIMEOUT', states[-1][1])

    def test_direct_near_field_terminal_preview_ends_wait_immediately(self):
        node = grasp_task_node.GraspTaskNode.__new__(
            grasp_task_node.GraspTaskNode
        )
        bound = self._rich_plan(plan_id='bound', stamp_sec=9.0)
        bound.diagnostic = grasp_task_node._FAR_FIELD_OBSERVATION_PLAN
        bound.plan_id = compute_plan_id(bound)
        node.active = True
        node._near_field_phase_started_sec = 9.5
        node._near_field_phase_deadline_sec = 40.0
        states = []
        node.set_state = lambda stage, message='', *_args, **_kwargs: (
            states.append((stage, message))
        )
        stream_calls = []
        node._set_near_field_preview_stream = (
            lambda _gcfg, enabled: (
                stream_calls.append(bool(enabled)) or True
            )
        )
        node._copy_near_field_preview_candidate = (
            lambda *_args, **_kwargs: (
                grasp_task_node.PlanValidationResult(
                    False,
                    'NEAR_FIELD_NO_REACHABLE_CANDIDATE',
                    'all checked candidates failed strict MoveIt',
                ),
                None,
            )
        )
        original_now = grasp_task_node.rospy.Time.now
        original_sleep = grasp_task_node.rospy.sleep
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(10.0)
        )
        grasp_task_node.rospy.sleep = lambda *_args, **_kwargs: self.fail(
            'fresh direct terminal must not wait for the phase deadline'
        )
        try:
            result = node._maybe_rebind_near_field_grasp6d_plan(
                {
                    'near_field_strategy': 'single_snapshot_direct',
                    'near_field_replan_enabled': True,
                    'near_field_replan_required': True,
                    'near_field_replan_timeout_sec': 30.0,
                },
                {},
                bound,
            )
        finally:
            grasp_task_node.rospy.Time.now = original_now
            grasp_task_node.rospy.sleep = original_sleep

        self.assertIsNone(result)
        self.assertEqual(stream_calls, [True, False])
        self.assertEqual(states[-1][0], grasp_task_node.GraspStages.FAILED)
        self.assertEqual(
            states[-1][1],
            (
                'NEAR_FIELD_NO_REACHABLE_CANDIDATE: all checked candidates '
                'failed strict MoveIt'
            ),
        )

    def test_direct_near_field_preview_window_starts_at_phase_boundary(self):
        node = grasp_task_node.GraspTaskNode.__new__(
            grasp_task_node.GraspTaskNode
        )
        bound = self._rich_plan(plan_id='bound', stamp_sec=9.0)
        node.active = True
        node._near_field_phase_started_sec = 10.0
        node._near_field_phase_deadline_sec = 40.0
        node.set_state = lambda *_args, **_kwargs: None
        node._set_near_field_preview_stream = (
            lambda _gcfg, _enabled: True
        )
        observed_minimum_stamps = []

        def terminal_result(_plan, minimum_stamp_ns, _gcfg):
            observed_minimum_stamps.append(int(minimum_stamp_ns))
            return (
                grasp_task_node.PlanValidationResult(
                    False,
                    'NEAR_FIELD_NO_REACHABLE_CANDIDATE',
                    'current phase terminal',
                ),
                None,
            )

        node._copy_near_field_preview_candidate = terminal_result
        original_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(10.25)
        )
        try:
            result = node._maybe_rebind_near_field_grasp6d_plan(
                {
                    'near_field_strategy': 'single_snapshot_direct',
                    'near_field_replan_enabled': True,
                    'near_field_replan_required': True,
                    'near_field_replan_timeout_sec': 30.0,
                    'near_field_replan_snapshot_slack_sec': 3.0,
                },
                {},
                bound,
            )
        finally:
            grasp_task_node.rospy.Time.now = original_now

        self.assertIsNone(result)
        self.assertEqual(observed_minimum_stamps, [int(10.0 * 1e9)])

    def test_direct_near_field_executes_frozen_plan_without_final_refine(self):
        node = grasp_task_node.GraspTaskNode.__new__(
            grasp_task_node.GraspTaskNode
        )
        plan = self._rich_plan(plan_id='far', stamp_sec=9.0)
        plan.diagnostic = grasp_task_node._FAR_FIELD_OBSERVATION_PLAN
        rebound = grasp_task_node.deepcopy(plan)
        rebound.poses[0].position.x += 0.03
        rebound.diagnostic = grasp_task_node._CONTACT_EXECUTION_PLAN
        rebound.plan_id = compute_plan_id(rebound)
        reached = grasp_task_node.deepcopy(
            grasp_task_node.split_rich_plan_poses(plan)[0]
        )
        node.active = True
        node._near_field_active = False
        node._grasp6d_plan_lock = threading.RLock()
        node._bound_target_occlusion_allowed = False
        node.set_state = lambda *_args, **_kwargs: None
        node._position_only_execute_globally_enabled = lambda: False
        node._execution_checkpoint = lambda *_args, **_kwargs: True
        node._current_tool_pose_base = lambda: reached
        node._current_observation_view_reusable = (
            lambda *_args, **_kwargs: True
        )
        node._wait_for_motion_settle = lambda *_args, **_kwargs: None
        node._command_gripper_position = lambda *_args, **_kwargs: True
        node._wait_for_fresh_observation_camera_target_range = (
            lambda *_args, **_kwargs: grasp_task_node.PlanValidationResult(
                True
            )
        )
        node._set_near_field_active = (
            lambda active, **_kwargs: setattr(
                node,
                '_near_field_active',
                bool(active),
            )
        )
        node._maybe_rebind_near_field_grasp6d_plan = (
            lambda *_args, **_kwargs: rebound
        )
        node._simulate_grasp6d_plan_if_required = (
            lambda *_args, **_kwargs: self.fail(
                'direct execution must not repeat simulation'
            )
        )
        node._maybe_final_refine_grasp6d_plan = (
            lambda *_args, **_kwargs: self.fail(
                'the fused near-field snapshot is the final visual correction'
            )
        )
        events = []

        def move(_stage, label, *_args, **_kwargs):
            events.append(label)
            return True

        node._plan_and_execute_pose = move
        node._close_gripper = lambda *_args, **_kwargs: (
            events.append('gripper close') or (True, 'closed')
        )

        original_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(10.0)
        )
        try:
            result = node._execute_grasp6d_plan(
                {
                    'near_field_strategy': 'single_snapshot_direct',
                    'plan_validity_sec': 5.0,
                    'near_field_replan_enabled': True,
                    'near_field_replan_required': True,
                    'observation_reuse_position_tolerance_m': 0.025,
                },
                {
                    'open_position_m': 0.05,
                    'use_compliant_close': False,
                },
                0.05,
                object(),
                object(),
                object(),
                None,
                plan,
                strict_execute_pose=lambda *_args, **_kwargs: None,
            )
        finally:
            grasp_task_node.rospy.Time.now = original_now

        self.assertTrue(result)
        self.assertEqual(
            events,
            [
                '6D near-field pregrasp',
                'linear 6D approach',
                'linear 6D grasp pose',
                'gripper close',
                'linear 6D lift',
            ],
        )

    def test_near_field_rebind_reuses_identical_reached_pregrasp(self):
        node = grasp_task_node.GraspTaskNode.__new__(
            grasp_task_node.GraspTaskNode
        )
        plan = self._rich_plan(plan_id='far', stamp_sec=9.0)
        plan.diagnostic = grasp_task_node._FAR_FIELD_OBSERVATION_PLAN
        rebound = grasp_task_node.deepcopy(plan)
        rebound.poses[1].position.x += 0.01
        rebound.diagnostic = grasp_task_node._CONTACT_EXECUTION_PLAN
        rebound.plan_id = compute_plan_id(rebound)
        node.active = True
        node._near_field_active = False
        node._grasp6d_plan_lock = threading.RLock()
        node._bound_target_occlusion_allowed = False
        node.set_state = lambda *_args, **_kwargs: None
        node._position_only_execute_globally_enabled = lambda: False
        node._execution_checkpoint = lambda *_args, **_kwargs: True
        simulation_calls = []
        node._simulate_grasp6d_plan_if_required = lambda *args, **kwargs: (
            simulation_calls.append(args) or True
        )
        node._command_gripper_position = lambda *_args, **_kwargs: True
        node._maybe_rebind_near_field_grasp6d_plan = (
            lambda *_args, **_kwargs: rebound
        )
        labels = []
        phase_at_move = []

        def move(stage, state_message, *_args, **_kwargs):
            labels.append(state_message)
            phase_at_move.append(
                (state_message, bool(node._near_field_active))
            )
            return state_message != 'linear 6D approach'

        node._plan_and_execute_pose = move
        original_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(10.0)
        )
        try:
            result = node._execute_grasp6d_plan(
                {
                    'plan_validity_sec': 5.0,
                    'near_field_replan_enabled': True,
                    'near_field_replan_required': True,
                },
                {'open_position_m': 0.05},
                0.05,
                lambda _pose, _execute: FakeServiceResponse(
                    True,
                    'observation preflight planned',
                ),
                object(),
                object(),
                None,
                plan,
                strict_execute_pose=lambda *_args, **_kwargs: None,
            )
        finally:
            grasp_task_node.rospy.Time.now = original_now

        self.assertFalse(result)
        self.assertEqual(
            labels,
            ['6D pregrasp', 'linear 6D approach'],
        )
        self.assertEqual(
            phase_at_move,
            [('6D pregrasp', False), ('linear 6D approach', True)],
        )
        self.assertEqual(simulation_calls, [])

    def test_far_field_preflight_failure_happens_before_gripper_command(self):
        node = grasp_task_node.GraspTaskNode.__new__(
            grasp_task_node.GraspTaskNode
        )
        plan = self._rich_plan(plan_id='far', stamp_sec=9.0)
        plan.diagnostic = grasp_task_node._FAR_FIELD_OBSERVATION_PLAN
        node.active = True
        node._grasp6d_plan_lock = threading.RLock()
        node._bound_target_occlusion_allowed = False
        node.set_state = lambda *_args, **_kwargs: None
        node._position_only_execute_globally_enabled = lambda: False
        node._execution_checkpoint = lambda *_args, **_kwargs: True
        physical = []
        node._command_gripper_position = lambda *_args, **_kwargs: (
            physical.append('gripper') or True
        )

        original_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(10.0)
        )
        try:
            result = node._execute_grasp6d_plan(
                {
                    'plan_validity_sec': 5.0,
                    'near_field_replan_enabled': True,
                    'near_field_replan_required': True,
                },
                {'open_position_m': 0.05},
                0.05,
                lambda _pose, _execute: FakeServiceResponse(
                    False,
                    'observation unreachable from current joints',
                ),
                object(),
                object(),
                None,
                plan,
                strict_execute_pose=lambda *_args, **_kwargs: None,
            )
        finally:
            grasp_task_node.rospy.Time.now = original_now

        self.assertFalse(result)
        self.assertEqual(physical, [])

    def test_far_field_plan_cannot_execute_when_near_field_replan_is_disabled(self):
        node = grasp_task_node.GraspTaskNode.__new__(
            grasp_task_node.GraspTaskNode
        )
        plan = self._rich_plan(plan_id='far', stamp_sec=9.0)
        plan.diagnostic = grasp_task_node._FAR_FIELD_OBSERVATION_PLAN
        node.active = True
        node._grasp6d_plan_lock = threading.RLock()
        node._bound_target_occlusion_allowed = False
        states = []
        node.set_state = lambda stage, message='', *_args, **_kwargs: (
            states.append((stage, message))
        )
        node._position_only_execute_globally_enabled = lambda: False
        node._execution_checkpoint = lambda *_args, **_kwargs: True
        side_effects = []
        node._simulate_grasp6d_plan_if_required = (
            lambda *_args, **_kwargs: side_effects.append('simulate') or True
        )
        node._command_gripper_position = (
            lambda *_args, **_kwargs: side_effects.append('gripper') or True
        )

        original_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(10.0)
        )
        try:
            result = node._execute_grasp6d_plan(
                {
                    'plan_validity_sec': 5.0,
                    'near_field_replan_enabled': False,
                    'near_field_replan_required': True,
                },
                {'open_position_m': 0.05},
                0.05,
                lambda *_args, **_kwargs: (
                    side_effects.append('move')
                    or FakeServiceResponse(True, 'unexpected move')
                ),
                object(),
                object(),
                None,
                plan,
                strict_execute_pose=lambda *_args, **_kwargs: None,
            )
        finally:
            grasp_task_node.rospy.Time.now = original_now

        self.assertFalse(result)
        self.assertEqual(side_effects, [])
        self.assertEqual(states[-1][0], grasp_task_node.GraspStages.FAILED)
        self.assertIn(
            'NEAR_FIELD_REPLAN_CONFIG_INVALID',
            states[-1][1],
        )

    def test_repeated_far_field_plan_preserves_reached_observation_view(self):
        node = grasp_task_node.GraspTaskNode.__new__(
            grasp_task_node.GraspTaskNode
        )
        plan = self._rich_plan(plan_id='far', stamp_sec=9.0)
        plan.diagnostic = grasp_task_node._FAR_FIELD_OBSERVATION_PLAN
        rebound = grasp_task_node.deepcopy(plan)
        rebound.poses[0].position.x += 0.03
        rebound.diagnostic = grasp_task_node._CONTACT_EXECUTION_PLAN
        rebound.plan_id = compute_plan_id(rebound)
        reached = grasp_task_node.deepcopy(
            grasp_task_node.split_rich_plan_poses(plan)[0]
        )
        reached.pose.position.x += 0.005
        reached.pose.orientation.x = 1.0
        reached.pose.orientation.y = 0.0
        reached.pose.orientation.z = 0.0
        reached.pose.orientation.w = 0.0
        node.active = True
        node._grasp6d_plan_lock = threading.RLock()
        node._bound_target_occlusion_allowed = False
        node.set_state = lambda *_args, **_kwargs: None
        node._position_only_execute_globally_enabled = lambda: False
        node._execution_checkpoint = lambda *_args, **_kwargs: True
        node._current_tool_pose_base = lambda: reached
        node._command_gripper_position = lambda *_args, **_kwargs: True
        node._maybe_rebind_near_field_grasp6d_plan = (
            lambda *_args, **_kwargs: rebound
        )
        node._simulate_grasp6d_plan_if_required = (
            lambda *_args, **_kwargs: True
        )
        settled = []
        node._wait_for_motion_settle = (
            lambda label: settled.append(label)
        )
        labels = []
        preflight = []

        def move(stage, state_message, *_args, **_kwargs):
            labels.append(state_message)
            return state_message != 'linear 6D approach'

        node._plan_and_execute_pose = move
        original_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(10.0)
        )
        try:
            result = node._execute_grasp6d_plan(
                {
                    'plan_validity_sec': 5.0,
                    'near_field_replan_enabled': True,
                    'near_field_replan_required': True,
                    'observation_reuse_position_tolerance_m': 0.025,
                },
                {'open_position_m': 0.05},
                0.05,
                lambda _pose, _execute: (
                    preflight.append(True)
                    or FakeServiceResponse(True, 'planned')
                ),
                object(),
                object(),
                None,
                plan,
                strict_execute_pose=lambda *_args, **_kwargs: None,
            )
        finally:
            grasp_task_node.rospy.Time.now = original_now

        self.assertFalse(result)
        self.assertEqual(preflight, [])
        self.assertEqual(settled, ['reused 6D observation'])
        self.assertEqual(
            labels,
            ['6D near-field pregrasp', 'linear 6D approach'],
        )

    def test_open_command_skips_service_when_feedback_is_already_open(self):
        node = grasp_task_node.GraspTaskNode.__new__(
            grasp_task_node.GraspTaskNode
        )
        node.latest_joint_state = self._joint_state()
        calls = []

        result = node._command_gripper_position(
            lambda position: (
                calls.append(position) or FakeServiceResponse(True, 'opened')
            ),
            0.05,
            'open gripper',
            1.0,
            skip_if_reached=True,
            reached_tolerance=0.001,
        )

        self.assertTrue(result)
        self.assertEqual(calls, [])

    def test_bound_execution_digest_mutation_fails_closed(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        bound = self._rich_plan(plan_id='bound', stamp_sec=9.0)
        node.latest_grasp6d_plan = bound
        node.latest_obj = self._object(stamp_sec=9.9)
        node.latest_obj_time = grasp_task_node.rospy.Time.from_sec(9.9)
        node.active = True
        node._freeze_execution_plan(bound)

        node._bound_execution_plan.score = 0.1
        original_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(10.0)
        )
        try:
            result = node._validate_bound_plan(
                bound,
                {
                    'target_max_drift_m': 0.02,
                    'target_observation_validity_sec': 1.5,
                },
            )
        finally:
            grasp_task_node.rospy.Time.now = original_now

        self.assertFalse(result.ok)
        self.assertEqual(result.code, 'EXECUTION_PLAN_INTEGRITY_CHANGED')
        self.assertTrue(node._execution_authority_revoked)

    def test_execution_copy_is_distinct_and_tampering_fails_closed(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        authority = self._rich_plan(plan_id='authority', stamp_sec=9.0)
        node.latest_grasp6d_plan = authority
        node.latest_obj = self._object(stamp_sec=9.9)
        node.latest_obj_time = grasp_task_node.rospy.Time.from_sec(9.9)
        node.active = True
        states = []
        node.set_state = lambda *args, **kwargs: states.append(args)
        execution_copy = node._freeze_execution_plan(authority)

        self.assertIsNot(execution_copy, node._bound_execution_plan)
        execution_copy.score = 0.1
        self.assertAlmostEqual(node._bound_execution_plan.score, 0.9)
        original_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(10.0)
        )
        try:
            result = node._execution_checkpoint(
                execution_copy, {}, 'tampered execution copy'
            )
        finally:
            grasp_task_node.rospy.Time.now = original_now

        self.assertFalse(result)
        self.assertIn('EXECUTION_PLAN_INTEGRITY_CHANGED', states[-1][1])
        self.assertTrue(node._execution_authority_revoked)

    def test_stored_execution_authority_tampering_fails_closed(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        authority = self._rich_plan(plan_id='authority', stamp_sec=9.0)
        node.latest_grasp6d_plan = authority
        node.latest_obj = self._object(stamp_sec=9.9)
        node.latest_obj_time = grasp_task_node.rospy.Time.from_sec(9.9)
        node.active = True
        states = []
        node.set_state = lambda *args, **kwargs: states.append(args)
        execution_copy = node._freeze_execution_plan(authority)
        node._bound_execution_plan.score = 0.1
        original_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(10.0)
        )
        try:
            result = node._execution_checkpoint(
                execution_copy, {}, 'tampered stored authority'
            )
        finally:
            grasp_task_node.rospy.Time.now = original_now

        self.assertFalse(result)
        self.assertIn('EXECUTION_PLAN_INTEGRITY_CHANGED', states[-1][1])
        self.assertTrue(node._execution_authority_revoked)

    def test_direct_rich_execute_cannot_mint_authority_from_arbitrary_plan(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node.active = True
        node.latest_grasp6d_plan = self._rich_plan(
            plan_id='latest-a', stamp_sec=9.0
        )
        arbitrary = self._rich_plan(plan_id='arbitrary-b', stamp_sec=9.5)
        node.latest_obj = self._object(stamp_sec=9.9)
        node.latest_obj_time = grasp_task_node.rospy.Time.from_sec(9.9)
        states = []
        actions = []
        node.set_state = lambda *args, **kwargs: states.append(args)
        node._simulate_grasp6d_plan_if_required = lambda *_args: (
            actions.append('simulate') or True
        )
        node._command_gripper_position = lambda *_args, **_kwargs: (
            actions.append('physical') or False
        )
        original_get_param = grasp_task_node.rospy.get_param
        original_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.get_param = lambda _name, default=None: default
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(10.0)
        )
        try:
            result = node._execute_grasp6d_plan(
                {'plan_validity_sec': 2.0},
                {'use_compliant_close': False},
                0.05,
                lambda *_args: FakeServiceResponse(True),
                lambda *_args: FakeServiceResponse(True),
                lambda *_args: FakeServiceResponse(True),
                None,
                arbitrary,
                strict_execute_pose=lambda *_args: FakeServiceResponse(True),
            )
        finally:
            grasp_task_node.rospy.get_param = original_get_param
            grasp_task_node.rospy.Time.now = original_now

        self.assertFalse(result)
        self.assertEqual(actions, [])
        self.assertIsNone(getattr(node, '_bound_execution_plan', None))
        self.assertIn('EXECUTION_PLAN_NOT_FROZEN', states[-1][1])

    def test_start_freezes_deep_copy_and_finally_clears_bound_execution(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        authority = self._rich_plan(plan_id='authority', stamp_sec=9.0)
        node.latest_grasp6d_plan = authority
        node.latest_obj = self._object(stamp_sec=9.9)
        node.latest_obj_time = grasp_task_node.rospy.Time.from_sec(9.9)
        node.active = False
        node.set_state = lambda *args, **kwargs: None
        observations = []

        def execute(grasp6d_plan=None):
            observations.append(
                (
                    grasp6d_plan.plan_id,
                    node._bound_execution_plan.plan_id,
                    node._bound_execution_plan.poses[0].position.x,
                )
            )
            node.latest_grasp6d_plan.poses[0].position.x = 99.0
            observations.append(node._bound_execution_plan.poses[0].position.x)
            return True

        node.execute = execute
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
            response = node.start_cb(
                types.SimpleNamespace(execute=True, plan_id=authority.plan_id)
            )
        finally:
            grasp_task_node.rospy.get_param = original_get_param
            grasp_task_node.rospy.Time.now = original_now

        self.assertTrue(response.success)
        self.assertEqual(
            observations,
            [(authority.plan_id, authority.plan_id, 0.10), 0.10],
        )
        self.assertIsNone(node._bound_execution_plan)
        self.assertEqual(node._bound_execution_plan_id, '')
        self.assertEqual(node._bound_execution_plan_digest, '')
        self.assertFalse(node._execution_authority_revoked)

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

    def test_start_service_rejects_execution_superseded_by_newer_preview(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node.active = False
        execution = self._rich_plan(plan_id='execution', stamp_sec=9.0)
        preview = self._rich_plan(plan_id='preview', stamp_sec=9.5)
        node.latest_grasp6d_plan = execution
        node.latest_grasp6d_preview_plan = preview
        node.latest_obj = self._object(stamp_sec=9.8)
        node.latest_obj_time = grasp_task_node.rospy.Time.from_sec(9.8)
        node.set_state = lambda *args, **kwargs: None
        actions = []
        node.execute = lambda *args, **kwargs: actions.append((args, kwargs)) or True
        request = types.SimpleNamespace(execute=True, plan_id=execution.plan_id)
        original_get_param = grasp_task_node.rospy.get_param
        original_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.get_param = lambda name, default=None: {
            '/grasp': {
                'use_grasp6d_plan': True,
                'plan_validity_sec': 2.0,
                'target_max_drift_m': 0.02,
            },
            '/grasp_6d/plan_validity_sec': 2.0,
        }.get(name, default)
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(10.0)
        )
        try:
            response = node.start_cb(request)
            old_validation = node.validate_plan_id_for_execution(
                execution.plan_id,
                {'plan_validity_sec': 2.0},
            )
            preview_validation = node.validate_plan_id_for_execution(
                preview.plan_id,
                {'plan_validity_sec': 2.0},
            )
        finally:
            grasp_task_node.rospy.get_param = original_get_param
            grasp_task_node.rospy.Time.now = original_now

        self.assertFalse(response.success)
        self.assertIn('PLAN_SUPERSEDED_BY_PREVIEW', response.message)
        self.assertFalse(old_validation.ok)
        self.assertEqual(old_validation.code, 'PLAN_SUPERSEDED_BY_PREVIEW')
        self.assertFalse(preview_validation.ok)
        self.assertEqual(preview_validation.code, 'PLAN_REPLACED')
        self.assertEqual(actions, [])

    def test_start_allows_newer_preview_when_near_field_replan_required(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node.active = False
        execution = self._rich_plan(plan_id='execution', stamp_sec=9.0)
        preview = self._rich_plan(plan_id='preview', stamp_sec=9.5)
        node.latest_grasp6d_plan = execution
        node.latest_grasp6d_preview_plan = preview
        node.latest_obj = self._object(stamp_sec=9.8)
        node.latest_obj_time = grasp_task_node.rospy.Time.from_sec(9.8)
        node.set_state = lambda *args, **kwargs: None
        actions = []

        def execute(grasp6d_plan=None):
            actions.append(grasp6d_plan.plan_id)
            return True

        node.execute = execute
        request = types.SimpleNamespace(execute=True, plan_id=execution.plan_id)
        original_get_param = grasp_task_node.rospy.get_param
        original_now = grasp_task_node.rospy.Time.now
        gcfg = {
            'use_grasp6d_plan': True,
            'plan_validity_sec': 2.0,
            'target_max_drift_m': 0.02,
            'near_field_replan_enabled': True,
            'near_field_replan_required': True,
        }
        grasp_task_node.rospy.get_param = lambda name, default=None: {
            '/grasp': gcfg,
            '/grasp_6d/plan_validity_sec': 2.0,
        }.get(name, default)
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(10.0)
        )
        try:
            response = node.start_cb(request)
            validation = node.validate_plan_id_for_execution(
                execution.plan_id,
                gcfg,
            )
        finally:
            grasp_task_node.rospy.get_param = original_get_param
            grasp_task_node.rospy.Time.now = original_now

        self.assertTrue(response.success)
        self.assertTrue(validation.ok)
        self.assertEqual(actions, [execution.plan_id])

    def test_start_service_bound_copy_stays_frozen_after_cache_replacement(self):
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
        self.assertEqual(actions, ['motion'])
        self.assertEqual(node.latest_grasp6d_plan.plan_id, new_plan.plan_id)
        self.assertIsNone(node._bound_execution_plan)

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

    def test_start_service_calibration_interlock_rejects_before_any_action(self):
        node = grasp_task_node.GraspTaskNode.__new__(
            grasp_task_node.GraspTaskNode
        )
        node.active = False
        actions = []
        node.execute = lambda *args, **kwargs: actions.append(
            (args, kwargs)
        ) or True
        original_get_param = grasp_task_node.rospy.get_param
        original_logerr_throttle = grasp_task_node.rospy.logerr_throttle
        grasp_task_node.rospy.get_param = lambda name, default=None: {
            '/grasp': {
                'use_grasp6d_plan': True,
                'calibration_interlock_active': True,
                'calibration_interlock_reason': (
                    'HAND_EYE_UNVERIFIED_AFTER_CORRECTION_COLLISION'
                ),
            },
        }.get(name, default)
        grasp_task_node.rospy.logerr_throttle = lambda *args, **kwargs: None
        try:
            response = node.start_cb(
                types.SimpleNamespace(execute=True, plan_id='any-plan')
            )
        finally:
            grasp_task_node.rospy.get_param = original_get_param
            grasp_task_node.rospy.logerr_throttle = original_logerr_throttle

        self.assertFalse(response.success)
        self.assertEqual(
            response.message,
            (
                'CALIBRATION_INTERLOCK: '
                'HAND_EYE_UNVERIFIED_AFTER_CORRECTION_COLLISION'
            ),
        )
        self.assertEqual(actions, [])
        self.assertFalse(node.active)

    def test_automatic_actuation_gate_requires_fresh_confirmed_status(self):
        node = grasp_task_node.GraspTaskNode.__new__(
            grasp_task_node.GraspTaskNode
        )
        gcfg = {
            'require_actuation_confirmation': True,
            'actuation_confirmation_freshness_sec': 2.0,
        }
        original_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(10.0)
        )
        try:
            node.actuation_status_cb(
                types.SimpleNamespace(
                    data='CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE'
                )
            )
        finally:
            grasp_task_node.rospy.Time.now = original_now

        accepted, reason = node._automatic_actuation_gate(
            gcfg,
            now_sec=11.9,
        )
        self.assertTrue(accepted)
        self.assertEqual(reason, '')

        accepted, reason = node._automatic_actuation_gate(
            gcfg,
            now_sec=12.01,
        )
        self.assertFalse(accepted)
        self.assertIn('ACTUATION_UNCONFIRMED', reason)
        self.assertIn('exceeds 2.000s', reason)

        for status in (
            '',
            'PENDING:POSITIVE_ENABLE_REQUESTED',
            'UNCONFIRMED:ENCODER_RESPONSE_TIMEOUT',
            'OVERHEAT_BLOCKED:SUSTAINED_SAME_CHANNEL_TEMPERATURE',
        ):
            node.latest_actuation_status = status
            node.latest_actuation_status_time = (
                grasp_task_node.rospy.Time.from_sec(11.0)
            )
            accepted, reason = node._automatic_actuation_gate(
                gcfg,
                now_sec=11.1,
            )
            self.assertFalse(accepted, status)
            self.assertTrue(
                reason.startswith('ACTUATION_UNCONFIRMED:'),
                reason,
            )

    def test_start_service_rejects_unconfirmed_actuation_before_execution(self):
        node = grasp_task_node.GraspTaskNode.__new__(
            grasp_task_node.GraspTaskNode
        )
        node.active = False
        node.latest_actuation_status = (
            'UNCONFIRMED:ENCODER_RESPONSE_TIMEOUT'
        )
        node.latest_actuation_status_time = (
            grasp_task_node.rospy.Time.from_sec(10.0)
        )
        actions = []
        node.execute = lambda *args, **kwargs: actions.append(
            (args, kwargs)
        ) or True
        original_get_param = grasp_task_node.rospy.get_param
        original_now = grasp_task_node.rospy.Time.now
        original_logerr_throttle = grasp_task_node.rospy.logerr_throttle
        grasp_task_node.rospy.get_param = lambda name, default=None: {
            '/grasp': {
                'use_grasp6d_plan': True,
                'calibration_interlock_active': False,
                'require_actuation_confirmation': True,
                'actuation_confirmation_freshness_sec': 2.0,
            },
        }.get(name, default)
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(10.1)
        )
        grasp_task_node.rospy.logerr_throttle = (
            lambda *args, **kwargs: None
        )
        try:
            response = node.start_cb(
                types.SimpleNamespace(execute=True, plan_id='any-plan')
            )
        finally:
            grasp_task_node.rospy.get_param = original_get_param
            grasp_task_node.rospy.Time.now = original_now
            grasp_task_node.rospy.logerr_throttle = (
                original_logerr_throttle
            )

        self.assertFalse(response.success)
        self.assertTrue(
            response.message.startswith('ACTUATION_UNCONFIRMED:'),
            response.message,
        )
        self.assertEqual(actions, [])
        self.assertFalse(node.active)

    def test_start_service_publishes_inactive_release_state_after_failure(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node.active = False
        node.stage = grasp_task_node.GraspStages.PLAN_PREGRASP
        states = []

        def record_state(stage, message='', success=False):
            states.append((stage, message, bool(success), bool(node.active)))

        node.set_state = record_state
        node.execute = lambda grasp6d_plan=None: False
        original_get_param = grasp_task_node.rospy.get_param
        grasp_task_node.rospy.get_param = lambda name, default=None: {
            '/grasp': {'use_grasp6d_plan': False},
        }.get(name, default)
        try:
            response = node.start_cb(
                types.SimpleNamespace(execute=True, plan_id='')
            )
        finally:
            grasp_task_node.rospy.get_param = original_get_param

        self.assertFalse(response.success)
        self.assertFalse(node.active)
        self.assertTrue(states)
        self.assertEqual(states[-1][0], grasp_task_node.GraspStages.PLAN_PREGRASP)
        self.assertEqual(states[-1][1], 'execution slot released: failed')
        self.assertFalse(states[-1][2])
        self.assertFalse(states[-1][3])

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
        node._freeze_execution_plan(plan)
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
                    node.active = True
                    node._freeze_execution_plan(node.latest_grasp6d_plan)
                    node.obj_cb(incoming)
                    self.assertIsNone(node.latest_obj)
                    self.assertIsNone(node.latest_obj_time)
                    self.assertIsNone(node.latest_grasp6d_plan)
                    self.assertIs(node.latest_visual_obj, incoming)
                    self.assertTrue(node._execution_authority_revoked)
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

    def test_live_target_observation_allows_measured_segmentation_latency(self):
        plan = self._rich_plan(stamp_sec=9.0)
        original_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(10.0)
        )
        try:
            node = grasp_task_node.GraspTaskNode.__new__(
                grasp_task_node.GraspTaskNode
            )
            node.latest_grasp6d_plan = plan
            node.latest_obj = self._object(stamp_sec=7.8)
            node.latest_obj_time = grasp_task_node.rospy.Time.from_sec(7.8)
            result = node._bound_target_drift_result(
                plan,
                {
                    'target_max_drift_m': 0.02,
                    'target_observation_validity_sec': 3.0,
                },
            )
            self.assertTrue(result.ok)
            self.assertIs(node.latest_grasp6d_plan, plan)
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

    def test_execution_validation_accepts_geometry_source_without_model_width(self):
        plan = self._rich_plan(stamp_sec=9.0)
        plan.candidate_source = 'tabletop_geometry'
        plan.candidate_source_lineage = ['tabletop_geometry']
        plan.has_candidate_model_width = False
        plan.candidate_width_m = 0.0
        plan.plan_id = compute_plan_id(plan)

        self.assertTrue(
            grasp_task_node.validate_execution_plan(plan, 10.0, 2.0).ok
        )

    def test_execution_validation_rejects_invalid_candidate_provenance(self):
        mutations = (
            lambda plan: setattr(plan, 'has_candidate_model_width', False),
            lambda plan: setattr(plan, 'candidate_source', 'unknown'),
            lambda plan: setattr(plan, 'candidate_source_lineage', []),
            lambda plan: (
                setattr(plan, 'candidate_source', 'tabletop_geometry'),
                setattr(
                    plan,
                    'candidate_source_lineage',
                    ['tabletop_geometry'],
                ),
            ),
        )
        for mutation in mutations:
            plan = self._rich_plan(stamp_sec=9.0)
            mutation(plan)
            with self.subTest(
                source=plan.candidate_source,
                lineage=list(plan.candidate_source_lineage),
                present=plan.has_candidate_model_width,
            ):
                self.assertFalse(
                    grasp_task_node.validate_execution_plan(
                        plan,
                        10.0,
                        2.0,
                    ).ok
                )

    def test_bound_execution_checkpoint_ignores_original_plan_lease_age(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        plan = self._rich_plan(stamp_sec=9.0)
        node.latest_grasp6d_plan = plan
        node.latest_obj = self._object()
        node.latest_obj_time = grasp_task_node.rospy.Time.from_sec(19.5)
        node.active = True
        node.set_state = lambda *args, **kwargs: None
        node._freeze_execution_plan(plan)
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

    def test_motion_stage_requires_frozen_plan_before_planning(self):
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
        self.assertEqual(calls, [])

    def test_plan_bound_action_linearizes_with_replacement_callback(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node.active = True
        first = self._rich_plan(plan_id='first', stamp_sec=9.0)
        second = self._rich_plan(plan_id='second', stamp_sec=9.5)
        node.latest_grasp6d_plan = first
        node.latest_obj = self._object()
        node.latest_obj_time = grasp_task_node.rospy.Time.from_sec(9.9)
        node._freeze_execution_plan(first)
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

    def test_plan_bound_action_rejects_unfrozen_plan_without_call(self):
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
        self.assertEqual(result.code, 'EXECUTION_PLAN_NOT_FROZEN')
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
        plan = node._freeze_execution_plan(plan)
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

    def test_far_field_observation_may_defer_controller_failure_to_live_range(self):
        node = grasp_task_node.GraspTaskNode.__new__(
            grasp_task_node.GraspTaskNode
        )
        node.active = True
        states = []
        calls = []
        node.set_state = lambda *args, **kwargs: states.append(args)
        node._wait_for_motion_settle = lambda reason='motion': (
            calls.append(('settle', reason)) or True
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
        plan = types.SimpleNamespace(
            plan_id='observation-plan',
            diagnostic=grasp_task_node._FAR_FIELD_OBSERVATION_PLAN,
        )

        def strict_plan(_pose, execute):
            calls.append(('strict-plan', bool(execute)))
            return FakeServiceResponse(True, 'planned: strict pose')

        def strict_execute(_pose, execute):
            calls.append(('strict-execute', bool(execute)))
            return FakeServiceResponse(
                False,
                'execute failed from cached plan (strict pose): target xyz=(0,0,0); '
                'controller-desired hold installed',
            )

        original_get_param = grasp_task_node.rospy.get_param
        grasp_task_node.rospy.get_param = lambda _name, default=None: default
        try:
            result = node._plan_and_execute_pose(
                grasp_task_node.GraspStages.MOVE_PREGRASP,
                'observation retreat',
                self._pose(0.10),
                strict_plan,
                'observation retreat',
                execution_plan=plan,
                gcfg={},
                execute_pose=strict_execute,
                allow_post_failure_observation_validation=True,
            )
        finally:
            grasp_task_node.rospy.get_param = original_get_param

        self.assertTrue(result)
        self.assertEqual(
            calls,
            [
                ('strict-plan', False),
                ('strict-execute', True),
                ('settle', 'observation retreat'),
            ],
        )
        self.assertTrue(states)
        self.assertNotEqual(states[-1][0], grasp_task_node.GraspStages.FAILED)
        self.assertEqual(
            node._last_observation_execution_failure['plan_id'],
            'observation-plan',
        )
        self.assertEqual(
            node._last_observation_execution_failure['stage_label'],
            'observation retreat',
        )

    def test_observation_correction_is_blocked_after_controller_failure(self):
        node = grasp_task_node.GraspTaskNode.__new__(
            grasp_task_node.GraspTaskNode
        )
        plan = self._rich_plan(plan_id='failed-observation', stamp_sec=9.0)
        plan.diagnostic = grasp_task_node._FAR_FIELD_OBSERVATION_PLAN
        node._last_observation_camera_range_evidence = {
            'camera_position_m': [0.31, 0.0, 0.20],
            'target_position_m': [0.0, 0.0, 0.20],
            'distance_m': 0.31,
        }
        node._last_measured_endpoint_sample = {
            'plan_id': plan.plan_id,
            'plan_phase': grasp_task_node._FAR_FIELD_OBSERVATION_PLAN,
            'position_error_vector_m': [0.10, 0.0, 0.0],
        }
        node._last_observation_execution_failure = {
            'plan_id': plan.plan_id,
            'stage_label': '6D pregrasp',
            'message': 'controller failed',
        }
        states = []
        node.set_state = (
            lambda stage, message='', *_args, **_kwargs: (
                states.append((stage, message))
            )
        )
        preflight = []

        result = node._maybe_execute_observation_camera_retreat(
            plan,
            {
                'observation_camera_target_retreat_correction_enabled': True,
                'observation_camera_target_min_distance_m': 0.180,
                'observation_camera_target_max_distance_m': 0.220,
                'observation_camera_target_nominal_distance_m': 0.200,
            },
            grasp_task_node.PlanValidationResult(
                False,
                'OBSERVATION_CAMERA_TARGET_OUT_OF_RANGE',
                'too far',
            ),
            lambda *args, **kwargs: preflight.append((args, kwargs)),
            object(),
        )

        self.assertIsNone(result)
        self.assertFalse(preflight)
        self.assertEqual(states[-1][0], grasp_task_node.GraspStages.FAILED)
        self.assertIn(
            'OBSERVATION_CORRECTION_AFTER_FAILED_EXECUTION_FORBIDDEN',
            states[-1][1],
        )

    def test_full_grasp_uses_6d_plan_sequence_when_enabled(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node.latest_obj = self._object()
        node.latest_obj_time = grasp_task_node.rospy.Time.from_sec(3.5)
        node.latest_grasp6d_plan = self._rich_plan(stamp_sec=1.0)
        node.active = True
        execution_plan = node._freeze_execution_plan(
            node.latest_grasp6d_plan
        )
        node._wait_for_motion_settle = lambda reason='motion': calls.append(('settle', reason))
        node._simulate_grasp6d_plan_if_required = lambda *_args: True
        node.set_state = lambda *args, **kwargs: None

        calls = []
        proxy_names = []

        def fake_service_proxy(name, _srv_type):
            proxy_names.append(name)
            if name in (
                '/supervisor/check_pose_strict',
                '/supervisor/plan_and_execute_pose_strict',
                '/supervisor/move_to_pose_linear',
            ):
                def move_pose(pose, execute):
                    mode = {
                        '/supervisor/check_pose_strict': 'strict-plan',
                        '/supervisor/plan_and_execute_pose_strict': 'strict-execute',
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
                    grasp6d_plan=execution_plan,
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
        execution_plan = node._freeze_execution_plan(
            node.latest_grasp6d_plan
        )
        node._wait_for_motion_settle = lambda reason='motion': calls.append(('settle', reason))
        node._simulate_grasp6d_plan_if_required = lambda *_args: True
        node.set_state = lambda *args, **kwargs: None

        calls = []

        def fake_service_proxy(name, _srv_type):
            if name in (
                '/supervisor/check_pose_strict',
                '/supervisor/plan_and_execute_pose_strict',
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
                    grasp6d_plan=execution_plan,
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
        execution_plan = node._freeze_execution_plan(
            node.latest_grasp6d_plan
        )
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
                    'candidate_source': (
                        node.latest_grasp6d_plan.candidate_source
                    ),
                    'candidate_source_lineage': list(
                        node.latest_grasp6d_plan.candidate_source_lineage
                    ),
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
                '/supervisor/plan_and_execute_pose_strict',
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
                    grasp6d_plan=execution_plan,
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
        self.assertEqual(payloads[0]['schema_version'], 3)
        self.assertEqual(payloads[0]['plan_id'], plan.plan_id)
        self.assertEqual(payloads[0]['candidate_source'], 'graspnet')
        self.assertEqual(
            payloads[0]['candidate_source_lineage'], ['graspnet']
        )
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

    def test_strict_mujoco_gate_ignores_latest_plan_after_network_return(self):
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

        self.assertEqual(physical, ['open-gripper'])
        self.assertNotIn('PLAN_REPLACED', states[-1][1])

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
                'WSL_UNAVAILABLE',
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
        execution_plan = node._freeze_execution_plan(
            node.latest_grasp6d_plan
        )
        states = []
        calls = []
        node._wait_for_motion_settle = lambda reason='motion': calls.append(('settle', reason))
        node.set_state = lambda *args, **kwargs: states.append(args)

        def fake_service_proxy(name, _srv_type):
            if name in (
                '/supervisor/check_pose_strict',
                '/supervisor/plan_and_execute_pose_strict',
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
                    grasp6d_plan=execution_plan,
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

    def test_invalid_plan_rejection_warning_is_throttled(self):
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node.latest_grasp6d_plan = None
        node.active = False
        invalid = self._rich_plan(plan_id='target-lost', stamp_sec=9.6)
        invalid.valid = False
        invalid.diagnostic = 'TARGET_LOST: target object is not detected'
        original_get_param = grasp_task_node.rospy.get_param
        original_now = grasp_task_node.rospy.Time.now
        original_logwarn = grasp_task_node.rospy.logwarn
        original_logwarn_throttle = grasp_task_node.rospy.logwarn_throttle
        plain_warnings = []
        throttled_warnings = []
        grasp_task_node.rospy.get_param = lambda name, default=None: {
            '/grasp_6d/plan_validity_sec': 2.0,
        }.get(name, default)
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(10.0)
        )
        grasp_task_node.rospy.logwarn = lambda *args: plain_warnings.append(args)
        grasp_task_node.rospy.logwarn_throttle = (
            lambda *args: throttled_warnings.append(args)
        )
        try:
            node.grasp6d_plan_cb(invalid)
        finally:
            grasp_task_node.rospy.get_param = original_get_param
            grasp_task_node.rospy.Time.now = original_now
            grasp_task_node.rospy.logwarn = original_logwarn
            grasp_task_node.rospy.logwarn_throttle = original_logwarn_throttle

        self.assertEqual(plain_warnings, [])
        self.assertEqual(len(throttled_warnings), 1)
        self.assertEqual(throttled_warnings[0][0], 1.0)

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

    def test_measured_tool_endpoint_records_observation_and_blocks_contact_error(self):
        node = grasp_task_node.GraspTaskNode.__new__(
            grasp_task_node.GraspTaskNode
        )
        states = []
        samples = []
        node.set_state = (
            lambda stage, message='', *_args, **_kwargs: (
                states.append((stage, message))
            )
        )
        requested = self._pose(0.10, -0.40, 0.20)
        actual = self._pose(0.10, -0.40, 0.2228)
        node._current_tool_pose_base = lambda: actual
        far_plan = types.SimpleNamespace(
            plan_id='far-plan',
            diagnostic=grasp_task_node._FAR_FIELD_OBSERVATION_PLAN,
        )
        contact_plan = types.SimpleNamespace(
            plan_id='contact-plan',
            diagnostic=grasp_task_node._CONTACT_EXECUTION_PLAN,
        )
        config = {
            'measured_endpoint_position_tolerance_m': 0.006,
            'measured_endpoint_orientation_tolerance_deg': 5.0,
        }
        original_set_param = grasp_task_node.rospy.set_param
        original_now = grasp_task_node.rospy.Time.now
        grasp_task_node.rospy.set_param = (
            lambda name, value: samples.append((name, value))
        )
        grasp_task_node.rospy.Time.now = staticmethod(
            lambda: grasp_task_node.rospy.Time.from_sec(10.0)
        )
        try:
            observation_ok = node._record_and_validate_measured_endpoint(
                requested,
                far_plan,
                config,
                '6D observation',
                required=False,
            )
            contact_ok = node._record_and_validate_measured_endpoint(
                requested,
                contact_plan,
                config,
                '6D near-field pregrasp',
                required=True,
            )
        finally:
            grasp_task_node.rospy.set_param = original_set_param
            grasp_task_node.rospy.Time.now = original_now

        self.assertTrue(observation_ok)
        self.assertFalse(contact_ok)
        self.assertEqual(
            samples[-1][0],
            '/grasp_6d/runtime_execution_error',
        )
        self.assertAlmostEqual(
            samples[-1][1]['position_error_m'],
            0.0228,
        )
        self.assertEqual(
            samples[-1][1]['requested_position_m'],
            [0.10, -0.40, 0.20],
        )
        self.assertEqual(
            samples[-1][1]['actual_position_m'],
            [0.10, -0.40, 0.2228],
        )
        self.assertEqual(
            samples[-1][1]['position_error_vector_m'][:2],
            [0.0, 0.0],
        )
        self.assertAlmostEqual(
            samples[-1][1]['position_error_vector_m'][2],
            0.0228,
        )
        self.assertEqual(
            len(samples[-1][1]['requested_quaternion_xyzw']),
            4,
        )
        self.assertEqual(
            len(samples[-1][1]['actual_quaternion_xyzw']),
            4,
        )
        self.assertIn(
            'MEASURED_ENDPOINT_POSITION_ERROR',
            states[-1][1],
        )

    def test_far_field_observation_records_residual_without_second_motion(self):
        node = grasp_task_node.GraspTaskNode.__new__(
            grasp_task_node.GraspTaskNode
        )
        requested = self._pose(0.10, -0.40, 0.20)
        plan = types.SimpleNamespace(
            plan_id='far-plan',
            diagnostic=grasp_task_node._FAR_FIELD_OBSERVATION_PLAN,
        )
        records = []
        node._record_and_validate_measured_endpoint = (
            lambda pose, recorded_plan, _cfg, label, required: (
                records.append(
                    (pose, recorded_plan, label, required)
                )
                or True
            )
        )

        result = node._converge_far_field_observation_endpoint(
            requested,
            plan,
            {
                'measured_endpoint_check_enabled': True,
                'measured_endpoint_position_tolerance_m': 0.006,
                'measured_endpoint_orientation_tolerance_deg': 5.0,
                'observation_endpoint_correction_attempts': 0,
            },
            object(),
            object(),
        )

        self.assertTrue(result)
        self.assertEqual(
            records,
            [(requested, plan, '6D far-field observation', False)],
        )

    def test_far_field_observation_propagates_residual_record_failure(self):
        node = grasp_task_node.GraspTaskNode.__new__(
            grasp_task_node.GraspTaskNode
        )
        requested = self._pose(0.10, -0.40, 0.20)
        plan = types.SimpleNamespace(
            plan_id='far-plan',
            diagnostic=grasp_task_node._FAR_FIELD_OBSERVATION_PLAN,
        )
        node._record_and_validate_measured_endpoint = (
            lambda *_args, **_kwargs: False
        )

        result = node._converge_far_field_observation_endpoint(
            requested,
            plan,
            {
                'measured_endpoint_check_enabled': True,
                'measured_endpoint_position_tolerance_m': 0.006,
                'measured_endpoint_orientation_tolerance_deg': 5.0,
                'observation_endpoint_correction_attempts': 0,
            },
            object(),
            object(),
        )

        self.assertFalse(result)

    def test_far_field_observation_does_not_reuse_contact_tolerance(self):
        node = grasp_task_node.GraspTaskNode.__new__(
            grasp_task_node.GraspTaskNode
        )
        requested = self._pose(0.10, -0.40, 0.20)
        plan = types.SimpleNamespace(
            plan_id='far-plan',
            diagnostic=grasp_task_node._FAR_FIELD_OBSERVATION_PLAN,
        )
        recorded = []
        node._record_and_validate_measured_endpoint = (
            lambda _pose, _plan, _cfg, label, required: (
                recorded.append((label, required)) or True
            )
        )

        result = node._converge_far_field_observation_endpoint(
            requested,
            plan,
            {
                'measured_endpoint_check_enabled': True,
                'measured_endpoint_position_tolerance_m': 0.0,
                'measured_endpoint_orientation_tolerance_deg': 0.0,
                'observation_endpoint_correction_attempts': 2,
            },
            object(),
            object(),
        )

        self.assertTrue(result)
        self.assertEqual(
            recorded,
            [('6D far-field observation', False)],
        )

    def test_measured_endpoint_contract_requires_consecutive_live_fk_samples(self):
        node = grasp_task_node.GraspTaskNode.__new__(
            grasp_task_node.GraspTaskNode
        )
        node.active = True
        requested = self._pose(0.10, -0.40, 0.20)
        residuals = [
            (self._pose(0.10, -0.40, 0.2066), 0.0066, math.radians(0.8)),
            (self._pose(0.10, -0.40, 0.2055), 0.0055, math.radians(0.7)),
            (self._pose(0.10, -0.40, 0.2054), 0.0054, math.radians(0.6)),
            (self._pose(0.10, -0.40, 0.2053), 0.0053, math.radians(0.5)),
        ]
        node._measured_endpoint_residual = lambda _requested: residuals.pop(0)
        original_get_param = grasp_task_node.rospy.get_param
        original_sleep = grasp_task_node.rospy.sleep
        original_is_shutdown = grasp_task_node.rospy.is_shutdown
        grasp_task_node.rospy.get_param = (
            lambda name, default=None: {
                '/grasp/motion_settle_timeout_sec': 5.0,
                '/grasp/motion_settle_sample_sec': 0.05,
                '/grasp/motion_settle_required_samples': 3,
            }.get(name, default)
        )
        grasp_task_node.rospy.sleep = lambda _duration: None
        grasp_task_node.rospy.is_shutdown = lambda: False
        try:
            result = node._wait_for_measured_endpoint_contract(
                requested,
                0.006,
                5.0,
                'test hold',
            )
        finally:
            grasp_task_node.rospy.get_param = original_get_param
            grasp_task_node.rospy.sleep = original_sleep
            grasp_task_node.rospy.is_shutdown = original_is_shutdown

        self.assertTrue(result[0])
        self.assertAlmostEqual(result[1], 0.0053)
        self.assertAlmostEqual(result[2], 0.5)
        self.assertEqual(residuals, [])

    def test_contact_pose_uses_scoped_precision_lease_and_stable_fk_contract(self):
        node = grasp_task_node.GraspTaskNode.__new__(
            grasp_task_node.GraspTaskNode
        )
        node.active = True
        node._bound_execution_plan = None
        node._position_only_execute_globally_enabled = lambda: False
        node._validate_bound_plan = (
            lambda *_args, **_kwargs: grasp_task_node.PlanValidationResult(True)
        )
        node._invoke_plan_bound_action = (
            lambda _plan, _cfg, _label, action: (
                grasp_task_node.PlanValidationResult(True),
                action(),
            )
        )
        node._wait_for_motion_settle = lambda _reason: True
        node.set_state = lambda *args, **kwargs: None
        plan = self._rich_plan()
        plan.diagnostic = grasp_task_node._CONTACT_EXECUTION_PLAN
        lease_calls = []
        event_order = []
        endpoint_waits = []
        endpoint_records = []
        node._set_contact_endpoint_precision = (
            lambda enabled, _cfg: (
                lease_calls.append(bool(enabled))
                or event_order.append(('lease', bool(enabled)))
                or True
            )
        )
        node._wait_for_measured_endpoint_contract = (
            lambda requested, position_tolerance, orientation_tolerance, reason: (
                endpoint_waits.append(
                    (
                        requested,
                        position_tolerance,
                        orientation_tolerance,
                        reason,
                    )
                )
                or (True, 0.004, 1.0)
            )
        )
        node._record_and_validate_measured_endpoint = (
            lambda requested, bound, cfg, label, required: (
                endpoint_records.append(
                    (requested, bound, cfg, label, required)
                )
                or True
            )
        )
        move_calls = []

        def move_pose(_pose, execute):
            move_calls.append(bool(execute))
            event_order.append(('move', bool(execute)))
            return FakeServiceResponse(True, 'ok')

        requested = self._pose(0.10, -0.40, 0.20)
        result = node._plan_and_execute_pose(
            grasp_task_node.GraspStages.MOVE_PREGRASP,
            '6D near-field pregrasp',
            requested,
            move_pose,
            '6D near-field pregrasp',
            execution_plan=plan,
            gcfg={
                'measured_endpoint_check_enabled': True,
                'measured_endpoint_position_tolerance_m': 0.006,
                'measured_endpoint_orientation_tolerance_deg': 5.0,
                'contact_endpoint_precision_enabled': True,
            },
            execute_pose=move_pose,
        )

        self.assertTrue(result)
        self.assertEqual(move_calls, [False, True])
        self.assertEqual(lease_calls, [True, False])
        self.assertEqual(
            event_order[:3],
            [('move', False), ('move', True), ('lease', True)],
        )
        self.assertEqual(len(endpoint_waits), 1)
        self.assertAlmostEqual(endpoint_waits[0][1], 0.006)
        self.assertAlmostEqual(endpoint_waits[0][2], 5.0)
        self.assertEqual(len(endpoint_records), 1)
        self.assertTrue(endpoint_records[0][-1])

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
