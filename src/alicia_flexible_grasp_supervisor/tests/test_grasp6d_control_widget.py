#!/usr/bin/env python3
import io
import inspect
import pathlib
import sys
import unittest
import xml.etree.ElementTree as ET

import rospy
import yaml
from geometry_msgs.msg import Pose, PoseArray
from alicia_flexible_grasp_supervisor.msg import Grasp6DPlan


ROOT = pathlib.Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / 'src'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id
from gui.widgets import grasp6d_control_widget as control_widget
from gui.widgets.grasp6d_control_widget import (
    Grasp6DGuiState,
    format_grasp6d_plan_state,
    grasp6d_button_labels,
)


class Grasp6DControlWidgetTest(unittest.TestCase):
    @staticmethod
    def _rich_plan(stamp_sec=9.0, plan_id='plan-a'):
        source_stamp_sec = float(stamp_sec)
        canonical_stamp_sec = source_stamp_sec if source_stamp_sec > 0.0 else 1.0
        plan = Grasp6DPlan()
        plan.header.frame_id = 'base_link'
        plan.header.stamp = rospy.Time.from_sec(canonical_stamp_sec)
        plan.valid = True
        plan.model_choice = 'carton_segment:' + str(plan_id)
        plan.score = 0.9
        plan.candidate_source = 'graspnet'
        plan.candidate_source_lineage = ['graspnet']
        plan.has_candidate_model_width = True
        plan.candidate_width_m = 0.039
        plan.required_open_width_m = 0.044
        for index in range(4):
            pose = Pose()
            pose.position.x = 0.1 * index
            pose.position.y = -0.1
            pose.position.z = 0.2
            pose.orientation.w = 1.0
            plan.poses.append(pose)
        geometry = plan.object_geometry
        geometry.header.frame_id = plan.header.frame_id
        geometry.header.stamp = plan.header.stamp
        geometry.valid = True
        geometry.label = 'carton'
        geometry.source_mode = 'instance_mask'
        geometry.pose_base.position.x = 0.4
        geometry.pose_base.position.y = -0.1
        geometry.pose_base.position.z = 0.2
        geometry.pose_base.orientation.w = 1.0
        geometry.size_xyz_m.x = 0.08
        geometry.size_xyz_m.y = 0.04
        geometry.size_xyz_m.z = 0.06
        geometry.support_normal_base.z = 1.0
        geometry.support_offset_m = 0.0
        plan.plan_id = compute_plan_id(plan)
        if source_stamp_sec <= 0.0:
            plan.header.stamp = rospy.Time.from_sec(source_stamp_sec)
            plan.object_geometry.header.stamp = rospy.Time.from_sec(source_stamp_sec)
        return plan

    def test_enriched_source_stamp_controls_readiness_and_legacy_never_enables(self):
        tracker = control_widget.Grasp6DReadinessTracker(validity_sec=2.0)

        legacy = PoseArray()
        legacy.poses = [Pose() for _ in range(4)]
        tracker.update_legacy(legacy)

        self.assertFalse(tracker.state(now_sec=10.0).fresh)
        self.assertEqual(tracker.legacy_pose_count, 4)

        plan = self._rich_plan(stamp_sec=9.0)
        tracker.update_enriched(plan, now_sec=10.0)
        state = tracker.state(now_sec=10.0)
        self.assertTrue(state.fresh)
        self.assertEqual(tracker.plan_id, plan.plan_id)
        self.assertIn(plan.plan_id, state.text)

        self.assertFalse(tracker.state(now_sec=12.01).fresh)
        self.assertEqual(tracker.plan_id, '')

    def test_invalid_future_zero_and_malformed_enriched_plans_clear_readiness(self):
        tracker = control_widget.Grasp6DReadinessTracker(validity_sec=2.0)
        tracker.update_enriched(self._rich_plan(stamp_sec=9.0), now_sec=10.0)
        self.assertTrue(tracker.state(now_sec=10.0).fresh)

        cases = [
            self._rich_plan(stamp_sec=0.0, plan_id='zero'),
            self._rich_plan(stamp_sec=10.1, plan_id='future'),
            self._rich_plan(stamp_sec=7.0, plan_id='stale'),
            self._rich_plan(stamp_sec=9.0, plan_id='bad-width'),
            self._rich_plan(stamp_sec=9.0, plan_id='bad-geometry'),
            self._rich_plan(stamp_sec=9.0, plan_id='invalid'),
        ]
        cases[3].required_open_width_m = 0.051
        cases[4].object_geometry.valid = False
        cases[5].valid = False
        cases[5].diagnostic = 'TARGET_LOST: object disappeared'

        for plan in cases:
            tracker.update_enriched(plan, now_sec=10.0)
            state = tracker.state(now_sec=10.0)
            self.assertFalse(state.fresh)
            self.assertEqual(tracker.plan_id, '')

        self.assertIn('TARGET_LOST', tracker.state(now_sec=10.0).text)

    def test_gui_rejects_cross_snapshot_headers(self):
        header_cases = []
        wrong_plan_frame = self._rich_plan(stamp_sec=9.0)
        wrong_plan_frame.header.frame_id = 'map'
        header_cases.append(wrong_plan_frame)
        wrong_geometry_frame = self._rich_plan(stamp_sec=9.0)
        wrong_geometry_frame.object_geometry.header.frame_id = 'map'
        header_cases.append(wrong_geometry_frame)
        wrong_geometry_stamp = self._rich_plan(stamp_sec=9.0)
        wrong_geometry_stamp.object_geometry.header.stamp = rospy.Time(9, 1)
        header_cases.append(wrong_geometry_stamp)

        for plan in header_cases:
            with self.subTest(frame=plan.header.frame_id):
                state = control_widget.validate_enriched_plan(
                    plan,
                    now_sec=10.0,
                    validity_sec=2.0,
                )
                self.assertFalse(state.fresh)

    def test_gui_geometry_semantics_bbox_mode_and_digest_tampering(self):
        for field, value in (
            ('label', ''),
            ('source_mode', ''),
            ('source_mode', 'unknown'),
        ):
            plan = self._rich_plan(stamp_sec=9.0)
            setattr(plan.object_geometry, field, value)
            with self.subTest(field=field, value=value):
                self.assertFalse(
                    control_widget.validate_enriched_plan(
                        plan,
                        now_sec=10.0,
                        validity_sec=2.0,
                    ).fresh
                )

        bbox_plan = self._rich_plan(stamp_sec=9.0)
        bbox_plan.object_geometry.source_mode = 'bbox_depth'
        self.assertTrue(
            control_widget.validate_enriched_plan(
                bbox_plan,
                now_sec=10.0,
                validity_sec=2.0,
            ).fresh
        )

        for field in ('pose', 'width', 'geometry'):
            plan = self._rich_plan(stamp_sec=9.0)
            if field == 'pose':
                plan.poses[2].position.x += 0.01
            elif field == 'width':
                plan.required_open_width_m += 0.001
            else:
                plan.object_geometry.size_xyz_m.x += 0.01
            with self.subTest(field=field):
                state = control_widget.validate_enriched_plan(
                    plan,
                    now_sec=10.0,
                    validity_sec=2.0,
                )
                self.assertFalse(state.fresh)

    def test_start_service_request_carries_captured_plan_id(self):
        widget = control_widget.Grasp6DControlWidget.__new__(
            control_widget.Grasp6DControlWidget
        )
        widget._execution_plan_id = 'captured-plan-id'
        widget._readiness = type(
            'Ready',
            (),
            {'matches_current': lambda self, plan_id: plan_id == 'captured-plan-id'},
        )()
        results = []
        widget._emit_command_result_if_alive = (
            lambda ok, message: results.append((ok, message))
        )
        calls = []
        original_wait = control_widget.rospy.wait_for_service
        original_proxy = control_widget.rospy.ServiceProxy
        control_widget.rospy.wait_for_service = lambda *_args, **_kwargs: None
        control_widget.rospy.ServiceProxy = lambda *_args, **_kwargs: (
            lambda **kwargs: (
                calls.append(kwargs)
                or type('Response', (), {'success': True, 'message': 'ok'})()
            )
        )
        try:
            widget._run_start_grasp()
        finally:
            control_widget.rospy.wait_for_service = original_wait
            control_widget.rospy.ServiceProxy = original_proxy

        self.assertEqual(
            calls,
            [{'execute': True, 'plan_id': 'captured-plan-id'}],
        )
        self.assertEqual(results, [(True, 'ok')])

    def test_successful_start_response_is_not_overwritten_by_post_call_expiry(self):
        widget = control_widget.Grasp6DControlWidget.__new__(
            control_widget.Grasp6DControlWidget
        )
        widget._execution_plan_id = 'captured-plan-id'
        checks = []

        def matches_current(_plan_id):
            checks.append(True)
            return len(checks) <= 2

        widget._readiness = type(
            'Ready', (), {'matches_current': lambda self, plan_id: matches_current(plan_id)}
        )()
        results = []
        widget._emit_command_result_if_alive = (
            lambda ok, message: results.append((ok, message))
        )
        original_wait = control_widget.rospy.wait_for_service
        original_proxy = control_widget.rospy.ServiceProxy
        control_widget.rospy.wait_for_service = lambda *_args, **_kwargs: None
        control_widget.rospy.ServiceProxy = lambda *_args, **_kwargs: (
            lambda **_kwargs: type(
                'Response', (), {'success': True, 'message': 'completed'}
            )()
        )
        try:
            widget._run_start_grasp()
        finally:
            control_widget.rospy.wait_for_service = original_wait
            control_widget.rospy.ServiceProxy = original_proxy

        self.assertEqual(len(checks), 2)
        self.assertEqual(results, [(True, 'completed')])

    def test_float32_wire_width_limit_accepts_exact_50mm_only(self):
        exact = self._rich_plan(stamp_sec=9.0)
        exact.required_open_width_m = 0.050
        exact.plan_id = compute_plan_id(exact)
        wire = io.BytesIO()
        exact.serialize(wire)
        received = Grasp6DPlan()
        received.deserialize(wire.getvalue())

        self.assertTrue(
            control_widget.validate_enriched_plan(
                received, now_sec=10.0, validity_sec=2.0
            ).fresh
        )

        over = self._rich_plan(stamp_sec=9.0)
        over.required_open_width_m = 0.0501
        over.plan_id = compute_plan_id(over)
        self.assertFalse(
            control_widget.validate_enriched_plan(
                over, now_sec=10.0, validity_sec=2.0
            ).fresh
        )

    def test_gui_accepts_geometry_source_without_model_width(self):
        plan = self._rich_plan(stamp_sec=9.0)
        plan.candidate_source = 'tabletop_geometry'
        plan.candidate_source_lineage = ['tabletop_geometry']
        plan.has_candidate_model_width = False
        plan.candidate_width_m = 0.0
        plan.plan_id = compute_plan_id(plan)

        self.assertTrue(
            control_widget.validate_enriched_plan(
                plan,
                now_sec=10.0,
                validity_sec=2.0,
            ).fresh
        )

    def test_gui_rejects_invalid_candidate_provenance(self):
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
                    control_widget.validate_enriched_plan(
                        plan,
                        now_sec=10.0,
                        validity_sec=2.0,
                    ).fresh
                )

    def test_replacement_changes_ready_plan_identity_without_aliasing(self):
        tracker = control_widget.Grasp6DReadinessTracker(validity_sec=2.0)
        first = self._rich_plan(stamp_sec=9.0, plan_id='first')
        first_id = first.plan_id
        tracker.update_enriched(first, now_sec=10.0)
        first.plan_id = 'mutated-after-callback'
        self.assertEqual(tracker.plan_id, first_id)

        second = self._rich_plan(stamp_sec=9.5, plan_id='second')
        tracker.update_enriched(second, now_sec=10.0)
        self.assertEqual(tracker.plan_id, second.plan_id)
        self.assertFalse(tracker.matches_current(first_id, now_sec=10.0))
        self.assertTrue(tracker.matches_current(second.plan_id, now_sec=10.0))

    def test_older_source_stamp_replay_clears_gui_readiness(self):
        tracker = control_widget.Grasp6DReadinessTracker(validity_sec=2.0)
        tracker.update_enriched(
            self._rich_plan(stamp_sec=9.5, plan_id='newer'),
            now_sec=10.0,
        )

        state = tracker.update_enriched(
            self._rich_plan(stamp_sec=9.0, plan_id='replayed'),
            now_sec=10.0,
        )

        self.assertFalse(state.fresh)
        self.assertEqual(tracker.plan_id, '')
        self.assertIn('PLAN_REPLAYED', tracker.state(now_sec=10.0).text)

    def test_gui_replay_watermark_survives_invalid_clear_and_repeated_replay(self):
        tracker = control_widget.Grasp6DReadinessTracker(validity_sec=2.0)
        newer = self._rich_plan(stamp_sec=9.5, plan_id='newer')
        tracker.update_enriched(newer, now_sec=10.0)

        invalid = self._rich_plan(stamp_sec=9.6, plan_id='invalid')
        invalid.valid = False
        invalid.diagnostic = 'TARGET_LOST: invalidation tombstone'
        tracker.update_enriched(invalid, now_sec=10.0)
        for suffix in ('first', 'second'):
            state = tracker.update_enriched(
                self._rich_plan(stamp_sec=9.0, plan_id='older-' + suffix),
                now_sec=10.0,
            )
            self.assertFalse(state.fresh)
            self.assertEqual(tracker.plan_id, '')
            self.assertIn('PLAN_REPLAYED', state.text)

        newest = self._rich_plan(stamp_sec=9.8, plan_id='newest')
        self.assertTrue(tracker.update_enriched(newest, now_sec=10.0).fresh)

    def test_gui_replay_tombstone_survives_replay_local_clear_and_expiry(self):
        tracker = control_widget.Grasp6DReadinessTracker(validity_sec=2.0)
        newer = self._rich_plan(stamp_sec=9.5, plan_id='newer')
        replay = self._rich_plan(stamp_sec=9.0, plan_id='replay')
        tracker.update_enriched(newer, now_sec=10.0)
        tracker.update_enriched(replay, now_sec=10.0)
        self.assertFalse(tracker.update_enriched(replay, now_sec=10.0).fresh)

        tracker.update_enriched(newer, now_sec=10.0)
        tracker.clear('local clear')
        self.assertFalse(tracker.update_enriched(newer, now_sec=10.0).fresh)

        newest = self._rich_plan(stamp_sec=9.8, plan_id='newest')
        tracker.update_enriched(newest, now_sec=10.0)
        self.assertFalse(tracker.state(now_sec=12.1).fresh)
        self.assertFalse(tracker.update_enriched(newest, now_sec=10.0).fresh)

    def test_zero_stamp_pending_preserves_strict_gui_source_watermark(self):
        def pending(stamp_sec):
            message = Grasp6DPlan()
            message.header.frame_id = 'base_link'
            message.header.stamp = rospy.Time.from_sec(stamp_sec)
            message.valid = False
            message.diagnostic = 'PLAN_PENDING: planning snapshot in progress'
            return message

        first = control_widget.Grasp6DReadinessTracker(validity_sec=2.0)
        first.update_enriched(pending(0.0), now_sec=10.0)
        initial = self._rich_plan(stamp_sec=9.0, plan_id='initial')
        self.assertTrue(first.update_enriched(initial, now_sec=10.0).fresh)

        older = control_widget.Grasp6DReadinessTracker(validity_sec=2.0)
        newer = self._rich_plan(stamp_sec=9.5, plan_id='newer')
        older.update_enriched(newer, now_sec=10.0)
        older.update_enriched(pending(0.0), now_sec=10.0)
        self.assertFalse(
            older.update_enriched(
                self._rich_plan(stamp_sec=9.0, plan_id='older'),
                now_sec=10.0,
            ).fresh
        )

        successor = control_widget.Grasp6DReadinessTracker(validity_sec=2.0)
        successor.update_enriched(newer, now_sec=10.0)
        successor.update_enriched(pending(0.0), now_sec=10.0)
        self.assertTrue(
            successor.update_enriched(
                self._rich_plan(stamp_sec=9.8, plan_id='successor'),
                now_sec=10.0,
            ).fresh
        )

        same_stamp = control_widget.Grasp6DReadinessTracker(validity_sec=2.0)
        same_stamp.update_enriched(newer, now_sec=10.0)
        same_stamp.update_enriched(pending(9.5), now_sec=10.0)
        self.assertFalse(same_stamp.update_enriched(newer, now_sec=10.0).fresh)

    def test_local_legacy_mode_is_explicitly_visualization_only(self):
        self.assertEqual(
            control_widget.local_plan_execution_notice(False),
            '本地旧版候选仅供显示，执行需要富计划',
        )
        self.assertEqual(control_widget.local_plan_execution_notice(True), '')

    def test_latched_startup_status_distinguishes_local_legacy_from_remote(self):
        self.assertTrue(
            control_widget.is_local_legacy_status('6D grasp waiting for RGB-D')
        )
        self.assertTrue(
            control_widget.is_local_legacy_status(
                '6D grasp backend unavailable: checkpoint missing'
            )
        )
        self.assertFalse(
            control_widget.is_local_legacy_status(
                'remote 6D grasp waiting for RGB-D: manual trigger'
            )
        )

    def test_plan_state_reports_waiting_fresh_and_stale(self):
        waiting = format_grasp6d_plan_state(None, now_sec=10.0, max_age_sec=2.0)
        fresh = format_grasp6d_plan_state(9.2, now_sec=10.0, max_age_sec=2.0)
        stale = format_grasp6d_plan_state(4.0, now_sec=10.0, max_age_sec=2.0)

        self.assertFalse(waiting.fresh)
        self.assertEqual(waiting.text, '等待 6D 抓取候选')
        self.assertTrue(fresh.fresh)
        self.assertIn('0.8s', fresh.text)
        self.assertFalse(stale.fresh)
        self.assertIn('已过期', stale.text)

    def test_button_labels_make_remote_6d_flow_explicit(self):
        labels = grasp6d_button_labels()

        self.assertEqual(labels.check_remote, '检查远程推理端')
        self.assertEqual(labels.request_plan, '生成 6D 候选')
        self.assertEqual(labels.stop_inference, '停止生成候选')
        self.assertEqual(labels.execute_grasp, '执行 6D 抓取流程')
        self.assertEqual(labels.stop, '停止抓取')

    def test_generate_button_toggles_streaming_without_using_stop_grasp(self):
        widget = control_widget.Grasp6DControlWidget.__new__(
            control_widget.Grasp6DControlWidget
        )
        widget._readiness = control_widget.Grasp6DReadinessTracker(
            validity_sec=5.0
        )
        execution = self._rich_plan(stamp_sec=9.0, plan_id='execution')
        widget._readiness.update_enriched(execution, now_sec=10.0)
        results = []
        widget._emit_stream_result_if_alive = (
            lambda enabled, ok, message: results.append((enabled, ok, message))
        )
        request_calls = []
        stop_grasp_calls = []
        original_wait = control_widget.rospy.wait_for_service
        original_proxy = control_widget.rospy.ServiceProxy
        control_widget.rospy.wait_for_service = lambda *_args, **_kwargs: None

        def service_proxy(name, *_args, **_kwargs):
            if name == '/grasp/stop':
                return lambda *_args, **_kwargs: stop_grasp_calls.append(True)
            self.assertEqual(name, '/grasp_6d/request_plan')
            return lambda trigger: (
                request_calls.append(bool(trigger))
                or type(
                    'Response',
                    (),
                    {'success': True, 'message': 'stream state updated'},
                )()
            )

        control_widget.rospy.ServiceProxy = service_proxy
        try:
            widget._run_set_streaming(True)
            widget._run_set_streaming(False)
        finally:
            control_widget.rospy.wait_for_service = original_wait
            control_widget.rospy.ServiceProxy = original_proxy

        self.assertEqual(request_calls, [True, False])
        self.assertEqual(stop_grasp_calls, [])
        self.assertEqual([item[:2] for item in results], [(True, True), (False, True)])
        self.assertTrue(widget._readiness.state(now_sec=10.0).fresh)

    def test_failed_stream_request_keeps_last_confirmed_button_state(self):
        widget = control_widget.Grasp6DControlWidget.__new__(
            control_widget.Grasp6DControlWidget
        )
        widget._requesting_plan = True
        widget._streaming_enabled = False
        widget._remote_status = ''
        widget._refresh_view = lambda: None

        widget._finish_stream_request(True, False, 'start failed')
        self.assertFalse(widget._streaming_enabled)
        self.assertFalse(widget._requesting_plan)

        widget._requesting_plan = True
        widget._finish_stream_request(True, True, 'started')
        self.assertTrue(widget._streaming_enabled)
        self.assertFalse(widget._requesting_plan)

        widget._requesting_plan = True
        widget._finish_stream_request(False, False, 'stop failed')
        self.assertTrue(widget._streaming_enabled)
        self.assertFalse(widget._requesting_plan)

    def test_preview_updates_diagnostics_without_mutating_execution_readiness(self):
        widget = control_widget.Grasp6DControlWidget.__new__(
            control_widget.Grasp6DControlWidget
        )
        widget._readiness = control_widget.Grasp6DReadinessTracker(
            validity_sec=5.0
        )
        widget._preview_readiness = control_widget.Grasp6DReadinessTracker(
            validity_sec=5.0
        )
        widget._refresh_view = lambda: None

        preview = self._rich_plan(stamp_sec=9.0, plan_id='preview-A')
        execution = self._rich_plan(stamp_sec=9.5, plan_id='execution-A')
        original_now = control_widget._ros_now_seconds
        control_widget._ros_now_seconds = lambda: 10.0
        try:
            widget._update_preview_plan(preview)
            self.assertEqual(widget._preview_readiness.plan_id, preview.plan_id)
            self.assertFalse(widget._readiness.state(now_sec=10.0).fresh)
            widget._update_plan(execution)
        finally:
            control_widget._ros_now_seconds = original_now

        self.assertEqual(widget._readiness.plan_id, execution.plan_id)
        self.assertEqual(widget._preview_readiness.plan_id, preview.plan_id)

    def test_execute_enablement_uses_execution_only_even_while_stream_toggle_is_pending(self):
        class FixedTracker:
            def __init__(self, state):
                self.value = state
                self.validity_sec = 0.0

            def state(self):
                return self.value

        class Control:
            def __init__(self):
                self.enabled = None
                self.text = ''

            def setEnabled(self, enabled):
                self.enabled = bool(enabled)

            def setText(self, text):
                self.text = str(text)

        fresh = control_widget.Grasp6DPlanState(True, 0.1, 'fresh')
        waiting = control_widget.Grasp6DPlanState(
            False, float('inf'), 'waiting'
        )
        widget = control_widget.Grasp6DControlWidget.__new__(
            control_widget.Grasp6DControlWidget
        )
        widget._plan_validity_sec = 5.0
        widget._readiness = FixedTracker(fresh)
        widget._preview_readiness = FixedTracker(fresh)
        widget._use_remote_grasp6d = True
        widget._legacy_only_status = False
        widget._remote_status = 'stream transition'
        widget._grasp_state = 'IDLE'
        widget._grasp_message = ''
        widget._last_plan_pose_count = 0
        widget._requesting_plan = True
        widget._streaming_enabled = False
        widget._command_active = False
        widget.summary = Control()
        widget.remote_chip = None
        widget.preview_chip = None
        widget.plan_chip = None
        widget.pose_chip = None
        widget.grasp_chip = None
        widget.request_plan_btn = Control()
        widget.execute_btn = Control()
        original_get_param = control_widget.rospy.get_param
        control_widget.rospy.get_param = (
            lambda _name, default=None: default
        )
        try:
            widget._refresh_view()
            self.assertTrue(widget.execute_btn.enabled)

            widget._readiness.value = waiting
            widget._preview_readiness.value = fresh
            widget._refresh_view()
            self.assertFalse(widget.execute_btn.enabled)
        finally:
            control_widget.rospy.get_param = original_get_param

    def test_state_summary_labels_preview_separately_from_execution(self):
        preview_state = format_grasp6d_plan_state(
            99.5, now_sec=100.0, max_age_sec=5.0
        )
        execution_state = format_grasp6d_plan_state(
            None, now_sec=100.0, max_age_sec=5.0
        )
        state = Grasp6DGuiState(
            remote_status='streaming',
            preview_state=preview_state,
            plan_state=execution_state,
            grasp_state='IDLE',
            grasp_message='waiting',
        )

        summary = state.summary()

        self.assertIn('Preview：稳定', summary)
        self.assertIn('Execution：不可执行', summary)

    def test_initial_continuous_pipeline_configuration_is_exact_and_physical_width_is_unchanged(self):
        with (ROOT / 'config' / 'grasp_params.yaml').open(
            'r', encoding='utf-8'
        ) as stream:
            config = yaml.safe_load(stream)

        remote = config['grasp_6d']['remote']
        self.assertEqual(config['grasp_6d']['plan_validity_sec'], 5.0)
        self.assertEqual(
            {
                key: remote[key]
                for key in (
                    'request_hz',
                    'result_max_age_sec',
                    'stability_window_size',
                    'stability_min_hits',
                    'tracking_position_threshold_m',
                    'tracking_orientation_threshold_deg',
                    'tracking_approach_threshold_deg',
                    'tracking_width_threshold_m',
                    'target_instance_association_threshold_m',
                    'target_absolute_sanity_distance_m',
                    'moveit_top_n',
                    'replan_position_delta_m',
                    'replan_orientation_delta_deg',
                    'replan_target_drift_m',
                    'replan_cooldown_sec',
                    'selection_hysteresis_ratio',
                    'candidate_consecutive_invalidations',
                    'performance_window_size',
                )
            },
            {
                'request_hz': 1.5,
                'result_max_age_sec': 5.0,
                'stability_window_size': 5,
                'stability_min_hits': 3,
                'tracking_position_threshold_m': 0.025,
                'tracking_orientation_threshold_deg': 25.0,
                'tracking_approach_threshold_deg': 20.0,
                'tracking_width_threshold_m': 0.008,
                'target_instance_association_threshold_m': 0.08,
                'target_absolute_sanity_distance_m': 0.15,
                'moveit_top_n': 5,
                'replan_position_delta_m': 0.012,
                'replan_orientation_delta_deg': 12.0,
                'replan_target_drift_m': 0.025,
                'replan_cooldown_sec': 1.0,
                'selection_hysteresis_ratio': 0.12,
                'candidate_consecutive_invalidations': 2,
                'performance_window_size': 100,
            },
        )
        self.assertEqual(config['gripper']['open_position_m'], 0.05)
        self.assertEqual(remote['max_gripper_width_m'], 0.05)
        self.assertNotIn('grasp6d_preview_enriched_plan_topic', config['grasp'])

    def test_launch_uses_one_shared_wsl_endpoint_without_fake_topic_overrides(self):
        root = ET.parse(ROOT / 'launch' / 'grasp_system.launch').getroot()
        args = {item.attrib['name']: item.attrib for item in root.findall('arg')}
        params = {
            item.attrib['name']: item.attrib['value']
            for item in root.findall('param')
        }

        self.assertIn('remote_grasp6d_url', args)
        self.assertEqual(
            params['/grasp_6d/remote/server_url'],
            '$(arg remote_grasp6d_url)',
        )
        self.assertEqual(
            params['/mujoco_digital_twin/server_url'],
            '$(arg remote_grasp6d_url)',
        )
        self.assertNotIn('grasp6d_preview_enriched_plan_topic', args)
        self.assertNotIn('grasp6d_execution_enriched_plan_topic', args)
        self.assertNotIn(
            '/grasp/grasp6d_preview_enriched_plan_topic', params
        )
        self.assertNotIn('/grasp/grasp6d_enriched_plan_topic', params)

        signature = inspect.signature(control_widget.Grasp6DControlWidget.__init__)
        self.assertEqual(
            signature.parameters['preview_enriched_plan_topic'].default,
            '/grasp_6d/preview_plan_enriched',
        )

    def test_state_summary_combines_remote_status_plan_and_grasp_state(self):
        state = Grasp6DGuiState(
            remote_status='remote 6D plan ready score=0.900 width=0.050',
            preview_state=format_grasp6d_plan_state(
                99.5, now_sec=100.0, max_age_sec=2.0
            ),
            plan_state=format_grasp6d_plan_state(99.0, now_sec=100.0, max_age_sec=2.0),
            grasp_state='IDLE',
            grasp_message='waiting',
        )

        summary = state.summary()

        self.assertIn('远程推理：remote 6D plan ready', summary)
        self.assertIn('Preview：稳定', summary)
        self.assertIn('Execution：可执行', summary)
        self.assertIn('抓取状态：IDLE', summary)


if __name__ == '__main__':
    unittest.main()
