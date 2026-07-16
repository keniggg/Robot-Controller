#!/usr/bin/env python3
import pathlib
import sys
import unittest

import rospy
from geometry_msgs.msg import Pose, PoseArray
from alicia_flexible_grasp_supervisor.msg import Grasp6DPlan


ROOT = pathlib.Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / 'src'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from gui.widgets import grasp6d_control_widget as control_widget
from gui.widgets.grasp6d_control_widget import (
    Grasp6DGuiState,
    format_grasp6d_plan_state,
    grasp6d_button_labels,
)


class Grasp6DControlWidgetTest(unittest.TestCase):
    @staticmethod
    def _rich_plan(stamp_sec=9.0, plan_id='plan-a'):
        plan = Grasp6DPlan()
        plan.header.frame_id = 'base_link'
        plan.header.stamp = rospy.Time.from_sec(float(stamp_sec))
        plan.valid = True
        plan.plan_id = str(plan_id)
        plan.model_choice = 'carton_segment'
        plan.score = 0.9
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
        geometry.header = plan.header
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
        return plan

    def test_enriched_source_stamp_controls_readiness_and_legacy_never_enables(self):
        tracker = control_widget.Grasp6DReadinessTracker(validity_sec=2.0)

        legacy = PoseArray()
        legacy.poses = [Pose() for _ in range(4)]
        tracker.update_legacy(legacy)

        self.assertFalse(tracker.state(now_sec=10.0).fresh)
        self.assertEqual(tracker.legacy_pose_count, 4)

        tracker.update_enriched(self._rich_plan(stamp_sec=9.0), now_sec=10.0)
        state = tracker.state(now_sec=10.0)
        self.assertTrue(state.fresh)
        self.assertEqual(tracker.plan_id, 'plan-a')
        self.assertIn('plan-a', state.text)

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

    def test_replacement_changes_ready_plan_identity_without_aliasing(self):
        tracker = control_widget.Grasp6DReadinessTracker(validity_sec=2.0)
        first = self._rich_plan(stamp_sec=9.0, plan_id='first')
        tracker.update_enriched(first, now_sec=10.0)
        first.plan_id = 'mutated-after-callback'
        self.assertEqual(tracker.plan_id, 'first')

        tracker.update_enriched(
            self._rich_plan(stamp_sec=9.5, plan_id='second'),
            now_sec=10.0,
        )
        self.assertEqual(tracker.plan_id, 'second')
        self.assertFalse(tracker.matches_current('first', now_sec=10.0))
        self.assertTrue(tracker.matches_current('second', now_sec=10.0))

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
        self.assertEqual(labels.execute_grasp, '执行 6D 抓取流程')
        self.assertEqual(labels.stop, '停止抓取')

    def test_state_summary_combines_remote_status_plan_and_grasp_state(self):
        state = Grasp6DGuiState(
            remote_status='remote 6D plan ready score=0.900 width=0.050',
            plan_state=format_grasp6d_plan_state(99.0, now_sec=100.0, max_age_sec=2.0),
            grasp_state='IDLE',
            grasp_message='waiting',
        )

        summary = state.summary()

        self.assertIn('远程推理：remote 6D plan ready', summary)
        self.assertIn('候选计划：可执行', summary)
        self.assertIn('抓取状态：IDLE', summary)


if __name__ == '__main__':
    unittest.main()
