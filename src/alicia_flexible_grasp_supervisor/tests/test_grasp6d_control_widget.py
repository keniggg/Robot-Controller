#!/usr/bin/env python3
import pathlib
import sys
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / 'src'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from gui.widgets.grasp6d_control_widget import (
    Grasp6DGuiState,
    format_grasp6d_plan_state,
    grasp6d_button_labels,
)


class Grasp6DControlWidgetTest(unittest.TestCase):
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
