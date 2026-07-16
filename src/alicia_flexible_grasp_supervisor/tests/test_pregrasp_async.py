#!/usr/bin/env python3
import pathlib
import sys
import time
import types
import unittest

from PyQt5 import QtCore
from geometry_msgs.msg import PoseStamped
import rospy


ROOT = pathlib.Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / 'src'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from gui.widgets.perception_widget import PerceptionWidget
from alicia_flexible_grasp.grasp.grasp_pose_generator import make_pregrasp_pose

APP = QtCore.QCoreApplication.instance() or QtCore.QCoreApplication([])


class FakeLabel:
    def __init__(self):
        self.text = ''

    def setText(self, text):
        self.text = text


class FakeButton:
    def __init__(self):
        self.enabled = True

    def setEnabled(self, enabled):
        self.enabled = bool(enabled)


class FakeSpin:
    def __init__(self, value):
        self._value = value

    def value(self):
        return self._value


class FakeCamera:
    def __init__(self):
        self.overlay = 'initial'

    def set_detection_overlay(self, bbox=None, label='', color=(80, 255, 120), contour_xy=None):
        self.overlay = {
            'bbox': bbox,
            'label': label,
            'color': color,
            'contour_xy': contour_xy,
        }


class FakeText:
    def __init__(self, text):
        self._text = text

    def text(self):
        return self._text


class PregraspAsyncTest(unittest.TestCase):
    def _pose(self, x):
        pose = PoseStamped()
        pose.pose.position.x = float(x)
        pose.pose.orientation.w = 1.0
        return pose

    def _object_msg(self, label='mouse', u=320, v=240, depth=0.5, base_xyz=(0.1, 0.2, 0.3)):
        pose_base = PoseStamped()
        pose_base.pose.position.x = float(base_xyz[0])
        pose_base.pose.position.y = float(base_xyz[1])
        pose_base.pose.position.z = float(base_xyz[2])
        pose_base.pose.orientation.w = 1.0
        return types.SimpleNamespace(
            detected=True,
            label=label,
            u=int(u),
            v=int(v),
            depth_m=float(depth),
            bbox_x=int(u) - 20,
            bbox_y=int(v) - 20,
            bbox_width=40,
            bbox_height=40,
            pose_base=pose_base,
        )

    def _worker_widget(self, execute):
        widget = PerceptionWidget.__new__(PerceptionWidget)
        widget.pregrasp_pose = self._pose(2.0)
        widget._planned_pregrasp_pose = self._pose(1.0) if execute else None
        widget._planned_pregrasp_executable = bool(execute)
        widget._planned_pregrasp_time = time.monotonic() if execute else 0.0
        widget._planned_target_base_xyz = (0.1, 0.2, 0.3) if execute else None
        widget._locked_grasp_target_base_xyz = None
        widget._locked_grasp_target_time = 0.0
        widget._pending_plan_pose = None
        widget._pending_plan_token = None
        widget._last_object_receive_time = time.monotonic()
        widget._planning_active = False
        widget._grasp_active = False
        widget._plan_token = 0
        widget._plan_timeout_sec = 60.0
        widget._alive = True
        widget._localization_ok = True
        widget._localization_error_m = None
        widget._object_stable_count = 3
        widget._last_object_base_xyz = (0.1, 0.2, 0.3)
        widget._last_target_signature = {'label': 'carton'}
        widget._pregrasp_pose_is_stale = lambda: False
        widget._planned_pregrasp_is_stale = lambda: False
        widget._pregrasp_target_is_stable = lambda: True
        widget.status = FakeLabel()
        widget.model_status_chip = FakeLabel()
        widget.model_profiles = {'carton': {'display_name': 'Carton 模型'}}
        widget.plan_pregrasp_btn = FakeButton()
        widget.execute_pregrasp_btn = FakeButton()
        widget.start_grasp_btn = FakeButton()
        started = []
        widget._start_pregrasp_worker = lambda pose, do_execute, token: started.append(
            (pose, bool(do_execute), token)
        )
        return widget, started

    def test_switch_during_plan_keeps_worker_busy_then_releases_without_stale_result(self):
        widget, started = self._worker_widget(execute=False)

        PerceptionWidget.plan_pregrasp(widget, False)
        canceled_token = started[0][2]
        PerceptionWidget._update_detector_status(widget, 'loading:carton')

        self.assertEqual(widget.__dict__.get('_pregrasp_worker_token'), canceled_token)
        self.assertFalse(widget.plan_pregrasp_btn.enabled)
        self.assertFalse(widget.start_grasp_btn.enabled)

        widget.pregrasp_pose = self._pose(3.0)
        PerceptionWidget.plan_pregrasp(widget, False)

        self.assertEqual(len(started), 1)
        status_before_completion = widget.status.text
        PerceptionWidget._finish_pregrasp_worker(
            widget,
            canceled_token,
            False,
            True,
            '规划成功：stale canceled plan',
        )

        self.assertIsNone(widget.__dict__.get('_pregrasp_worker_token'))
        self.assertTrue(widget.plan_pregrasp_btn.enabled)
        self.assertTrue(widget.start_grasp_btn.enabled)
        self.assertFalse(widget.execute_pregrasp_btn.enabled)
        self.assertIsNone(widget._planned_pregrasp_pose)
        self.assertEqual(widget.status.text, status_before_completion)
        self.assertNotIn('stale canceled plan', widget.status.text)

    def test_switch_during_execute_timeout_waits_for_actual_worker_return(self):
        widget, started = self._worker_widget(execute=True)

        PerceptionWidget.plan_pregrasp(widget, True)
        canceled_token = started[0][2]
        PerceptionWidget._update_detector_status(widget, 'loading:carton')

        self.assertEqual(widget.__dict__.get('_pregrasp_worker_token'), canceled_token)
        self.assertFalse(widget.plan_pregrasp_btn.enabled)
        self.assertFalse(widget.start_grasp_btn.enabled)

        widget.pregrasp_pose = self._pose(3.0)
        PerceptionWidget.plan_pregrasp(widget, False)

        self.assertEqual(len(started), 1)
        status_before_timeout = widget.status.text
        PerceptionWidget._timeout_pregrasp_worker(widget, canceled_token)

        self.assertEqual(widget.__dict__.get('_pregrasp_worker_token'), canceled_token)
        self.assertFalse(widget.plan_pregrasp_btn.enabled)
        self.assertFalse(widget.start_grasp_btn.enabled)
        self.assertFalse(widget.execute_pregrasp_btn.enabled)
        self.assertEqual(widget.status.text, status_before_timeout)
        self.assertNotIn('超时', widget.status.text)

        PerceptionWidget._finish_pregrasp_worker(
            widget,
            canceled_token,
            True,
            True,
            '执行成功：late canceled execute',
        )

        self.assertIsNone(widget.__dict__.get('_pregrasp_worker_token'))
        self.assertTrue(widget.plan_pregrasp_btn.enabled)
        self.assertTrue(widget.start_grasp_btn.enabled)
        self.assertFalse(widget.execute_pregrasp_btn.enabled)
        self.assertEqual(widget.status.text, status_before_timeout)
        self.assertNotIn('late canceled execute', widget.status.text)

    def test_timeout_blocks_second_dispatch_until_late_result_releases_owner(self):
        widget, started = self._worker_widget(execute=True)
        original_plan = widget._planned_pregrasp_pose

        PerceptionWidget.plan_pregrasp(widget, True)
        timed_out_token = started[0][2]
        PerceptionWidget._timeout_pregrasp_worker(widget, timed_out_token)

        self.assertEqual(widget.__dict__.get('_pregrasp_worker_token'), timed_out_token)
        self.assertFalse(widget.plan_pregrasp_btn.enabled)
        self.assertFalse(widget.start_grasp_btn.enabled)
        self.assertFalse(widget.execute_pregrasp_btn.enabled)
        self.assertIn('超时', widget.status.text)

        PerceptionWidget.plan_pregrasp(widget, True)

        self.assertEqual(len(started), 1)
        status_before_late_result = widget.status.text
        PerceptionWidget._finish_pregrasp_worker(
            widget,
            timed_out_token,
            True,
            True,
            '执行成功：late timed-out execute',
        )

        self.assertIsNone(widget.__dict__.get('_pregrasp_worker_token'))
        self.assertTrue(widget.plan_pregrasp_btn.enabled)
        self.assertTrue(widget.start_grasp_btn.enabled)
        self.assertTrue(widget.execute_pregrasp_btn.enabled)
        self.assertIs(widget._planned_pregrasp_pose, original_plan)
        self.assertIsNone(widget.__dict__.get('_locked_grasp_target_base_xyz'))
        self.assertEqual(widget.status.text, status_before_late_result)
        self.assertNotIn('late timed-out execute', widget.status.text)

        PerceptionWidget.plan_pregrasp(widget, True)

        self.assertEqual(len(started), 2)

    def test_dispatch_message_explains_timeout_keeps_controls_locked(self):
        widget, _ = self._worker_widget(execute=False)

        PerceptionWidget.plan_pregrasp(widget, False)

        self.assertIn('超时后继续锁定', widget.status.text)
        self.assertIn('直到后台请求结束', widget.status.text)
        self.assertNotIn('自动释放按钮', widget.status.text)

    def test_timeout_status_survives_refresh_until_worker_returns(self):
        widget, started = self._worker_widget(execute=True)

        PerceptionWidget.plan_pregrasp(widget, True)
        timed_out_token = started[0][2]
        PerceptionWidget._timeout_pregrasp_worker(widget, timed_out_token)
        timeout_status = widget.status.text

        widget._status_hold_until = 0.0
        PerceptionWidget._set_perception_status(widget, '目标识别稳定，已更新目标坐标和预抓取位姿')
        PerceptionWidget._update_detector_status(widget, 'error:carton:metadata unavailable')

        self.assertEqual(widget.__dict__.get('_pregrasp_worker_token'), timed_out_token)
        self.assertEqual(widget.status.text, timeout_status)
        self.assertIn('后台请求仍未结束', widget.status.text)

        PerceptionWidget._finish_pregrasp_worker(
            widget,
            timed_out_token,
            True,
            True,
            '执行成功：late timed-out execute',
        )
        PerceptionWidget._set_perception_status(widget, '目标识别稳定，已恢复正常状态更新')

        self.assertIsNone(widget.__dict__.get('_pregrasp_worker_token'))
        self.assertEqual(widget.status.text, '目标识别稳定，已恢复正常状态更新')

    def test_plan_pregrasp_starts_worker_instead_of_sync_service_call(self):
        widget = PerceptionWidget.__new__(PerceptionWidget)
        widget.pregrasp_pose = types.SimpleNamespace(value=1)
        widget.status = FakeLabel()
        widget.plan_pregrasp_btn = FakeButton()
        widget.execute_pregrasp_btn = FakeButton()
        widget._planning_active = False
        widget._plan_token = 0
        widget._pregrasp_target_is_stable = lambda: True
        started = []

        widget._start_pregrasp_worker = lambda pose, execute, token: started.append((pose, execute, token))

        PerceptionWidget.plan_pregrasp(widget, False)

        self.assertEqual(len(started), 1)
        self.assertFalse(started[0][1])
        self.assertTrue(widget._planning_active)
        self.assertFalse(widget.plan_pregrasp_btn.enabled)
        self.assertIn('后台规划', widget.status.text)

    def test_execute_pregrasp_uses_last_successful_plan_pose(self):
        widget = PerceptionWidget.__new__(PerceptionWidget)
        widget.pregrasp_pose = self._pose(2.0)
        widget._planned_pregrasp_pose = self._pose(1.0)
        widget.status = FakeLabel()
        widget.plan_pregrasp_btn = FakeButton()
        widget.execute_pregrasp_btn = FakeButton()
        widget._planning_active = False
        widget._plan_token = 0
        widget._localization_ok = True
        widget._planned_pregrasp_executable = True
        started = []

        widget._start_pregrasp_worker = lambda pose, execute, token: started.append((pose, execute, token))

        PerceptionWidget.plan_pregrasp(widget, True)

        self.assertEqual(len(started), 1)
        self.assertTrue(started[0][1])
        self.assertAlmostEqual(started[0][0].pose.position.x, 1.0)
        self.assertIn('后台执行', widget.status.text)

    def test_execute_pregrasp_requires_a_successful_plan_first(self):
        widget = PerceptionWidget.__new__(PerceptionWidget)
        widget.pregrasp_pose = self._pose(2.0)
        widget._planned_pregrasp_pose = None
        widget.status = FakeLabel()
        widget.plan_pregrasp_btn = FakeButton()
        widget.execute_pregrasp_btn = FakeButton()
        widget._planning_active = False
        widget._plan_token = 0
        widget._localization_ok = True
        started = []

        widget._start_pregrasp_worker = lambda pose, execute, token: started.append((pose, execute, token))

        PerceptionWidget.plan_pregrasp(widget, True)

        self.assertEqual(started, [])
        self.assertIn('先点击规划预抓取', widget.status.text)

    def test_stale_live_pose_blocks_non_executing_plan(self):
        widget = PerceptionWidget.__new__(PerceptionWidget)
        widget.pregrasp_pose = self._pose(2.0)
        widget.status = FakeLabel()
        widget.plan_pregrasp_btn = FakeButton()
        widget.execute_pregrasp_btn = FakeButton()
        widget._planning_active = False
        widget._plan_token = 0
        widget._pregrasp_pose_is_stale = lambda: True
        widget._pregrasp_target_is_stable = lambda: True
        started = []

        widget._start_pregrasp_worker = lambda pose, execute, token: started.append((pose, execute, token))

        PerceptionWidget.plan_pregrasp(widget, False)

        self.assertEqual(started, [])
        self.assertFalse(widget._planning_active)
        self.assertIn('过期', widget.status.text)

    def test_pregrasp_pose_is_stale_uses_pose_stamp(self):
        widget = PerceptionWidget.__new__(PerceptionWidget)
        widget.pregrasp_pose = self._pose(2.0)
        widget.pregrasp_pose.header.stamp = rospy.Time.from_sec(98.0)
        widget._max_object_age_sec = 1.5
        widget._last_object_receive_time = None
        original_now = rospy.Time.now
        rospy.Time.now = staticmethod(lambda: rospy.Time.from_sec(100.0))
        try:
            self.assertTrue(PerceptionWidget._pregrasp_pose_is_stale(widget))
        finally:
            rospy.Time.now = original_now

    def test_pregrasp_pose_uses_receive_time_for_slow_yolo_stamp(self):
        widget = PerceptionWidget.__new__(PerceptionWidget)
        widget.pregrasp_pose = self._pose(2.0)
        widget.pregrasp_pose.header.stamp = rospy.Time.from_sec(98.0)
        widget._max_object_age_sec = 1.5
        original_now = rospy.Time.now
        original_monotonic = time.monotonic
        rospy.Time.now = staticmethod(lambda: rospy.Time.from_sec(101.0))
        time.monotonic = lambda: 50.0
        widget._last_object_receive_time = 49.7
        try:
            self.assertFalse(PerceptionWidget._pregrasp_pose_is_stale(widget))
        finally:
            rospy.Time.now = original_now
            time.monotonic = original_monotonic

    def test_unstable_live_pose_blocks_non_executing_plan(self):
        widget = PerceptionWidget.__new__(PerceptionWidget)
        widget.pregrasp_pose = self._pose(2.0)
        widget.status = FakeLabel()
        widget.plan_pregrasp_btn = FakeButton()
        widget.execute_pregrasp_btn = FakeButton()
        widget._planning_active = False
        widget._plan_token = 0
        widget._pregrasp_pose_is_stale = lambda: False
        widget._pregrasp_target_is_stable = lambda: False
        started = []

        widget._start_pregrasp_worker = lambda pose, execute, token: started.append((pose, execute, token))

        PerceptionWidget.plan_pregrasp(widget, False)

        self.assertEqual(started, [])
        self.assertFalse(widget._planning_active)
        self.assertIn('不稳定', widget.status.text)

    def test_target_stability_accepts_depth_noise_when_pixel_target_stays_locked(self):
        widget = PerceptionWidget.__new__(PerceptionWidget)
        widget.pregrasp_pose = self._pose(2.0)
        widget._object_stable_count = 0
        widget._last_object_base_xyz = None
        widget._last_target_signature = None
        widget._required_stable_detections = 3
        widget._object_stability_radius_m = 0.03
        widget._object_stability_pixel_radius_px = 45.0
        widget._object_stability_depth_radius_m = 0.15

        for depth, z in ((0.54, 0.30), (0.62, 0.38), (0.50, 0.26)):
            PerceptionWidget._update_target_stability(
                widget,
                self._object_msg(u=318, v=242, depth=depth, base_xyz=(0.1, 0.2, z)),
            )

        self.assertTrue(PerceptionWidget._pregrasp_target_is_stable(widget))
        self.assertEqual(widget._object_stable_count, 3)

    def test_target_stability_resets_on_large_pixel_jump(self):
        widget = PerceptionWidget.__new__(PerceptionWidget)
        widget.pregrasp_pose = self._pose(2.0)
        widget._object_stable_count = 0
        widget._last_object_base_xyz = None
        widget._last_target_signature = None
        widget._required_stable_detections = 3
        widget._object_stability_radius_m = 0.08
        widget._object_stability_pixel_radius_px = 45.0
        widget._object_stability_depth_radius_m = 0.15

        PerceptionWidget._update_target_stability(widget, self._object_msg(u=318, v=242, depth=0.54))
        PerceptionWidget._update_target_stability(widget, self._object_msg(u=430, v=242, depth=0.55))

        self.assertEqual(widget._object_stable_count, 1)
        self.assertFalse(PerceptionWidget._pregrasp_target_is_stable(widget))

    def test_successful_plan_result_saves_pose_for_execution(self):
        widget = PerceptionWidget.__new__(PerceptionWidget)
        widget._pending_plan_pose = self._pose(1.0)
        widget._pending_plan_token = 4
        widget._planned_pregrasp_pose = None
        widget.status = FakeLabel()
        widget.plan_pregrasp_btn = FakeButton()
        widget.execute_pregrasp_btn = FakeButton()
        widget._alive = True
        widget._planning_active = True
        widget._plan_token = 4

        PerceptionWidget._finish_pregrasp_worker(
            widget,
            4,
            False,
            True,
            '规划成功：planned with candidate orientation current',
        )

        self.assertIsNotNone(widget._planned_pregrasp_pose)
        self.assertAlmostEqual(widget._planned_pregrasp_pose.pose.position.x, 1.0)
        self.assertTrue(widget._planned_pregrasp_executable)
        self.assertTrue(widget.execute_pregrasp_btn.enabled)
        self.assertIn('执行已规划预抓取', widget.status.text)

    def test_detection_loss_does_not_clear_recent_planned_pregrasp(self):
        widget = PerceptionWidget.__new__(PerceptionWidget)
        widget._alive = True
        widget.pregrasp_pose = self._pose(1.0)
        widget._planned_pregrasp_pose = self._pose(1.0)
        widget._planned_pregrasp_executable = True
        widget._planned_pregrasp_time = time.monotonic()
        widget._planned_target_base_xyz = (0.1, 0.2, 0.3)
        widget._object_stable_count = 3
        widget._last_object_base_xyz = (0.1, 0.2, 0.3)
        widget._last_target_signature = {'label': 'mouse'}
        widget._localization_ok = True
        widget._localization_error_m = None
        widget._planning_active = False
        widget._status_hold_until = 0.0
        widget.status = FakeLabel()
        widget.detected_chip = FakeLabel()
        widget.label_edit = FakeText('mouse')
        widget.camera_preview = FakeCamera()
        widget.execute_pregrasp_btn = FakeButton()
        widget.execute_pregrasp_btn.setEnabled(True)

        PerceptionWidget.update_object(widget, types.SimpleNamespace(detected=False, label='mouse'))

        self.assertIsNotNone(widget._planned_pregrasp_pose)
        self.assertTrue(widget._planned_pregrasp_executable)
        self.assertTrue(widget.execute_pregrasp_btn.enabled)
        self.assertIsNone(widget.pregrasp_pose)
        self.assertIn('未检测到', widget.detected_chip.text)

    def test_executed_pregrasp_success_locks_target_for_grasp_flow(self):
        widget = PerceptionWidget.__new__(PerceptionWidget)
        widget._pending_plan_pose = None
        widget._pending_plan_token = None
        widget._planned_pregrasp_pose = self._pose(1.0)
        widget._planned_pregrasp_executable = True
        widget._planned_pregrasp_time = time.monotonic()
        widget._planned_target_base_xyz = (0.1, 0.2, 0.3)
        widget.status = FakeLabel()
        widget.plan_pregrasp_btn = FakeButton()
        widget.execute_pregrasp_btn = FakeButton()
        widget.start_grasp_btn = FakeButton()
        widget._alive = True
        widget._planning_active = True
        widget._grasp_active = False
        widget._plan_token = 4
        widget._grasp_flow_lock_max_age_sec = 60.0

        PerceptionWidget._finish_pregrasp_worker(
            widget,
            4,
            True,
            True,
            '执行成功：executed cached plan',
        )

        self.assertEqual(widget.__dict__.get('_locked_grasp_target_base_xyz'), (0.1, 0.2, 0.3))
        self.assertGreater(widget.__dict__.get('_locked_grasp_target_time', 0.0), 0.0)
        self.assertIn('执行抓取流程', widget.status.text)

    def test_position_only_plan_result_does_not_enable_execution(self):
        widget = PerceptionWidget.__new__(PerceptionWidget)
        widget._pending_plan_pose = self._pose(1.0)
        widget._pending_plan_token = 4
        widget._planned_pregrasp_pose = None
        widget._planned_pregrasp_executable = False
        widget.status = FakeLabel()
        widget.plan_pregrasp_btn = FakeButton()
        widget.execute_pregrasp_btn = FakeButton()
        widget._alive = True
        widget._planning_active = True
        widget._plan_token = 4

        PerceptionWidget._finish_pregrasp_worker(
            widget,
            4,
            False,
            True,
            '规划成功：planned with position-only fallback: target xyz=(0.1, 0.2, 0.3)',
        )

        self.assertIsNone(widget._planned_pregrasp_pose)
        self.assertFalse(widget._planned_pregrasp_executable)
        self.assertFalse(widget.execute_pregrasp_btn.enabled)
        self.assertIn('仅位置', widget.status.text)
        self.assertIn('禁止执行', widget.status.text)

    def test_execute_pregrasp_blocks_non_executable_plan(self):
        widget = PerceptionWidget.__new__(PerceptionWidget)
        widget.pregrasp_pose = self._pose(2.0)
        widget._planned_pregrasp_pose = self._pose(1.0)
        widget._planned_pregrasp_executable = False
        widget.status = FakeLabel()
        widget.plan_pregrasp_btn = FakeButton()
        widget.execute_pregrasp_btn = FakeButton()
        widget._planning_active = False
        widget._plan_token = 0
        widget._localization_ok = True
        started = []

        widget._start_pregrasp_worker = lambda pose, execute, token: started.append((pose, execute, token))

        PerceptionWidget.plan_pregrasp(widget, True)

        self.assertEqual(started, [])
        self.assertIn('仅位置', widget.status.text)

    def test_detection_refresh_does_not_immediately_hide_successful_plan_status(self):
        widget = PerceptionWidget.__new__(PerceptionWidget)
        widget._pending_plan_pose = self._pose(1.0)
        widget._pending_plan_token = 4
        widget._planned_pregrasp_pose = None
        widget.status = FakeLabel()
        widget.plan_pregrasp_btn = FakeButton()
        widget.execute_pregrasp_btn = FakeButton()
        widget._alive = True
        widget._planning_active = True
        widget._plan_token = 4
        widget.pregrasp = FakeSpin(0.08)
        widget._pregrasp_mode = 'camera_ray'
        widget._localization_ok = True
        widget._localization_error_m = None
        widget._planned_pregrasp_executable = False
        widget._object_stable_count = 3
        widget._required_stable_detections = 3
        widget._last_object_base_xyz = (0.3, 0.0, 0.0)
        widget._object_stability_radius_m = 0.03
        widget._pregrasp_pose_is_stale = lambda: False
        widget._lookup_camera_pose_base = lambda: None
        widget.detected_chip = FakeLabel()
        widget.pixel_chip = FakeLabel()
        widget.depth_chip = FakeLabel()
        widget.conf_chip = FakeLabel()
        widget.camera_chip = FakeLabel()
        widget.base_chip = FakeLabel()
        widget.pregrasp_chip = FakeLabel()

        PerceptionWidget._finish_pregrasp_worker(widget, 4, False, True, '规划成功：planned')
        msg = types.SimpleNamespace(
            detected=True,
            label='mouse',
            u=320,
            v=240,
            depth_m=0.2,
            confidence=0.9,
            pose_camera=self._pose(0.2),
            pose_base=self._pose(0.3),
        )
        PerceptionWidget.update_object(widget, msg)

        self.assertIn('规划成功', widget.status.text)
        self.assertIn('执行已规划预抓取', widget.status.text)

    def test_pregrasp_status_text_names_failed_plan_and_execute(self):
        plan_text = PerceptionWidget._pregrasp_status_text(False, False, 'plan failed')
        execute_text = PerceptionWidget._pregrasp_status_text(True, False, 'failed')

        self.assertIn('规划失败', plan_text)
        self.assertIn('plan failed', plan_text)
        self.assertIn('执行失败', execute_text)
        self.assertIn('failed', execute_text)

    def test_pregrasp_status_text_explains_unreachable_detected_target(self):
        text = PerceptionWidget._pregrasp_status_text(
            False,
            False,
            'plan failed: target unreachable or pose orientation invalid',
        )

        self.assertIn('识别成功但目标不可达/姿态不可达', text)
        self.assertIn('target unreachable', text)

    def test_pregrasp_status_text_names_successful_plan_and_execute(self):
        plan_text = PerceptionWidget._pregrasp_status_text(False, True, 'planned')
        execute_text = PerceptionWidget._pregrasp_status_text(True, True, 'executed')

        self.assertIn('规划成功', plan_text)
        self.assertIn('planned', plan_text)
        self.assertIn('执行成功', execute_text)
        self.assertIn('executed', execute_text)

    def test_pregrasp_pose_uses_camera_ray_standoff(self):
        obj = PoseStamped()
        obj.pose.position.x = 0.40
        obj.pose.position.y = 0.00
        obj.pose.position.z = 0.20
        camera = PoseStamped()
        camera.pose.position.x = 0.10
        camera.pose.position.y = 0.00
        camera.pose.position.z = 0.20

        pre = make_pregrasp_pose(obj, 0.08, camera_pose=camera, mode='camera_ray')

        self.assertAlmostEqual(pre.pose.position.x, 0.32)
        self.assertAlmostEqual(pre.pose.position.y, 0.00)
        self.assertAlmostEqual(pre.pose.position.z, 0.20)

    def test_execute_pregrasp_is_blocked_when_localization_untrusted(self):
        widget = PerceptionWidget.__new__(PerceptionWidget)
        widget.pregrasp_pose = PoseStamped()
        widget.status = FakeLabel()
        widget.plan_pregrasp_btn = FakeButton()
        widget.execute_pregrasp_btn = FakeButton()
        widget._planning_active = False
        widget._plan_token = 0
        widget._localization_ok = False
        widget._planned_pregrasp_pose = PoseStamped()
        widget._planned_pregrasp_executable = True
        started = []

        widget._start_pregrasp_worker = lambda pose, execute, token: started.append((pose, execute, token))

        PerceptionWidget.plan_pregrasp(widget, True)

        self.assertEqual(started, [])
        self.assertIn('定位不可信', widget.status.text)

    def test_grasp_flow_requires_detected_object(self):
        widget = PerceptionWidget.__new__(PerceptionWidget)
        widget.status = FakeLabel()
        widget.last_object = None
        widget._grasp_active = False

        PerceptionWidget.start_grasp_flow(widget)

        self.assertFalse(widget._grasp_active)
        self.assertIn('没有可用目标', widget.status.text)

    def test_grasp_flow_allows_recent_locked_target_when_current_detection_is_lost(self):
        widget = PerceptionWidget.__new__(PerceptionWidget)
        widget.status = FakeLabel()
        widget.last_object = types.SimpleNamespace(detected=False, label='mouse')
        widget._grasp_active = False
        widget._locked_grasp_target_base_xyz = (0.1, 0.2, 0.3)
        widget._locked_grasp_target_time = time.monotonic()
        widget._grasp_flow_lock_max_age_sec = 60.0
        widget._pregrasp_target_is_stable = lambda: False
        widget._localization_ok = False
        widget.plan_pregrasp_btn = FakeButton()
        widget.execute_pregrasp_btn = FakeButton()
        widget.start_grasp_btn = FakeButton()
        started = []
        widget._start_grasp_flow_worker = lambda: started.append(True)

        PerceptionWidget.start_grasp_flow(widget)

        self.assertTrue(widget._grasp_active)
        self.assertEqual(started, [True])
        self.assertIn('锁定目标', widget.status.text)

    def test_active_grasp_keeps_locked_target_when_detection_temporarily_lost(self):
        widget = PerceptionWidget.__new__(PerceptionWidget)
        previous = self._object_msg(label='mouse')
        widget.last_object = previous
        widget.pregrasp_pose = self._pose(1.0)
        widget.status = FakeLabel()
        widget.detected_chip = FakeLabel()
        widget.label_edit = FakeText('mouse')
        widget.camera_preview = FakeCamera()
        widget._grasp_active = True
        widget._locked_grasp_target_base_xyz = (0.1, 0.2, 0.3)
        widget._locked_grasp_target_time = time.monotonic()
        widget._grasp_flow_lock_max_age_sec = 60.0
        widget._alive = True
        widget._reset_target_stability = lambda: None
        widget._set_perception_status = lambda text: widget.status.setText(text)

        PerceptionWidget.update_object(widget, types.SimpleNamespace(detected=False, label='mouse'))

        self.assertIs(widget.last_object, previous)
        self.assertIsNotNone(widget.pregrasp_pose)
        self.assertEqual(widget.camera_preview.overlay['bbox'], (300, 220, 40, 40))
        self.assertEqual(widget.camera_preview.overlay['label'], 'mouse')
        self.assertIsNone(widget.camera_preview.overlay['contour_xy'])
        self.assertEqual(widget._mask_status, 'mask stale')
        self.assertIn('目标已锁定', widget.status.text)


if __name__ == '__main__':
    unittest.main()
