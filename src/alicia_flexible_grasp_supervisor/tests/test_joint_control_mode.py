#!/usr/bin/env python3
import pathlib
import sys
import unittest

import rospy


ROOT = pathlib.Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / 'src'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from gui.widgets.joint_control_widget import (
    DirectControlRecovery,
    JointControlWidget,
)


class JointControlModeTest(unittest.TestCase):
    def test_default_joint_mode_keeps_trajectory_controllers_available(self):
        original_get_param = rospy.get_param
        rospy.get_param = lambda name, default=None: default
        try:
            self.assertFalse(JointControlWidget._default_direct_control_enabled())
        finally:
            rospy.get_param = original_get_param

    def test_default_joint_mode_can_be_overridden_for_direct_control(self):
        original_get_param = rospy.get_param
        rospy.get_param = lambda name, default=None: True if name == '/gui/default_joint_direct_control' else default
        try:
            self.assertTrue(JointControlWidget._default_direct_control_enabled())
        finally:
            rospy.get_param = original_get_param

    def test_private_runtime_direct_mode_override_wins(self):
        original_get_param = rospy.get_param

        def get_param(name, default=None):
            if name == '/gui/default_joint_direct_control':
                return False
            if name == '~default_joint_direct_control':
                return True
            return default

        rospy.get_param = get_param
        try:
            self.assertTrue(JointControlWidget._default_direct_control_enabled())
        finally:
            rospy.get_param = original_get_param

    def test_blocked_slider_target_requires_measured_probe_before_replay(self):
        recovery = DirectControlRecovery(timeout_sec=2.0)
        recovery.update_actuation_status(
            'OVERHEAT_BLOCKED:SUSTAINED_SAME_CHANNEL_TEMPERATURE'
        )
        recovery.update_feedback_ready(True)

        action, target, _ = recovery.submit([0.1, 0.2], 10.0)
        self.assertEqual(action, 'enable')
        self.assertIsNone(target)

        # The positive-enable reset intentionally invalidates retained
        # feedback. The GUI must not publish the target during that gap.
        recovery.update_actuation_status('PENDING:POSITIVE_ENABLE_REQUESTED')
        recovery.update_feedback_ready(False)
        self.assertEqual(recovery.poll(10.1)[0], 'wait')

        recovery.update_feedback_ready(True)
        self.assertEqual(recovery.poll(10.15)[0], 'wait')
        measured = [0.0, 0.0]
        recovery.note_joint_feedback(measured, 10.16)
        action, target, _ = recovery.poll(10.2)
        self.assertEqual(action, 'sync')
        self.assertEqual(target, measured)

        recovery.update_actuation_status('PENDING:COMMAND_SYNCHRONIZED')
        action, target, _ = recovery.poll(10.25)
        self.assertEqual(action, 'probe')
        self.assertEqual(target, [0.0, 0.025])

        recovery.update_actuation_status('PENDING:AWAITING_ENCODER_RESPONSE')
        self.assertEqual(recovery.poll(10.3)[0], 'wait')

        recovery.update_actuation_status(
            'CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE'
        )
        action, target, _ = recovery.poll(10.35)
        self.assertEqual(action, 'publish')
        self.assertEqual(target, [0.1, 0.2])
        self.assertEqual(recovery.poll(10.4)[0], 'none')

    def test_recovery_sync_allows_user_target_beyond_reconnect_tolerance(self):
        recovery = DirectControlRecovery(timeout_sec=2.0)
        recovery.update_actuation_status(
            'OVERHEAT_BLOCKED:SUSTAINED_SAME_CHANNEL_TEMPERATURE'
        )
        recovery.update_feedback_ready(True)
        user_target = [-2.138, 0.607, 0.778]
        self.assertEqual(recovery.submit(user_target, 15.0)[0], 'enable')

        measured = [-2.138, -0.431, 0.778]
        recovery.update_actuation_status('PENDING:POSITIVE_ENABLE_REQUESTED')
        recovery.note_joint_feedback(measured, 15.1)
        action, sync_target, _ = recovery.poll(15.2)
        self.assertEqual(action, 'sync')
        self.assertEqual(sync_target, measured)

        recovery.update_actuation_status('PENDING:COMMAND_SYNCHRONIZED')
        action, target, _ = recovery.poll(15.3)
        self.assertEqual(action, 'probe')
        self.assertEqual(target[0], -2.138)
        self.assertAlmostEqual(target[1], -0.406)
        self.assertEqual(target[2], 0.778)

        recovery.update_actuation_status(
            'CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE'
        )
        action, target, _ = recovery.poll(15.4)
        self.assertEqual(action, 'publish')
        self.assertEqual(target, user_target)

    def test_rejected_positive_enable_is_not_retried_in_background(self):
        recovery = DirectControlRecovery(
            timeout_sec=2.0,
            enable_retry_sec=0.5,
        )
        recovery.update_actuation_status(
            'OVERHEAT_BLOCKED:SUSTAINED_SAME_CHANNEL_TEMPERATURE'
        )
        self.assertEqual(recovery.submit([0.1], 20.0)[0], 'enable')
        recovery.update_feedback_ready(True)
        recovery.note_joint_feedback([0.0], 20.1)
        self.assertEqual(recovery.poll(20.8)[0], 'wait')

        # A later explicit slider action may retry; the timer alone may not.
        self.assertEqual(recovery.submit([0.2], 20.8)[0], 'enable')

    def test_confirmed_direct_target_publishes_without_enable_request(self):
        recovery = DirectControlRecovery(timeout_sec=2.0)
        recovery.update_actuation_status(
            'CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE'
        )
        recovery.update_feedback_ready(True)

        action, target, _ = recovery.submit([0.3], 30.0)
        self.assertEqual(action, 'publish')
        self.assertEqual(target, [0.3])

    def test_external_pending_enable_never_publishes_full_slider_target(self):
        recovery = DirectControlRecovery(timeout_sec=2.0)
        recovery.update_actuation_status('PENDING:POSITIVE_ENABLE_REQUESTED')
        recovery.update_feedback_ready(True)
        recovery.note_joint_feedback([0.0, 0.0], 29.9)

        action, target, _ = recovery.submit([0.0, 0.4], 30.0)
        self.assertEqual(action, 'wait')
        self.assertIsNone(target)

        recovery.note_joint_feedback([0.0, 0.0], 30.1)
        action, target, _ = recovery.poll(30.2)
        self.assertEqual(action, 'sync')
        self.assertEqual(target, [0.0, 0.0])

    def test_startup_target_waits_for_latched_driver_state(self):
        recovery = DirectControlRecovery(timeout_sec=2.0)
        self.assertEqual(recovery.submit([0.4], 40.0)[0], 'wait')

        recovery.update_actuation_status('UNCONFIRMED:CONFIRMED_FEEDBACK_STALE')
        self.assertEqual(recovery.poll(40.1)[0], 'enable')

    def test_recovery_timeout_clears_bounded_probe_and_drops_user_target(self):
        recovery = DirectControlRecovery(timeout_sec=0.2)
        recovery.submit([0.5], 50.0)
        action, target, message = recovery.poll(50.3)
        self.assertEqual(action, 'clear')
        self.assertIsNone(target)
        self.assertIn('恢复超时', message)
        self.assertEqual(recovery.poll(50.4)[0], 'none')

    def test_encoder_timeout_never_replays_unconfirmed_user_target(self):
        recovery = DirectControlRecovery(timeout_sec=2.0)
        recovery.update_actuation_status('UNCONFIRMED:ENCODER_RESPONSE_TIMEOUT')
        recovery.update_feedback_ready(True)

        self.assertEqual(recovery.submit([0.4, 0.0], 60.0)[0], 'enable')
        recovery.update_actuation_status('PENDING:POSITIVE_ENABLE_REQUESTED')
        recovery.note_joint_feedback([0.0, 0.0], 60.1)
        self.assertEqual(recovery.poll(60.2)[0], 'sync')
        recovery.update_actuation_status('PENDING:COMMAND_SYNCHRONIZED')
        action, probe, _ = recovery.poll(60.3)
        self.assertEqual(action, 'probe')
        self.assertEqual(probe, [0.025, 0.0])

        recovery.update_actuation_status('UNCONFIRMED:ENCODER_RESPONSE_TIMEOUT')
        action, target, message = recovery.poll(61.4)
        self.assertEqual(action, 'clear')
        self.assertIsNone(target)
        self.assertIn('未检测到真实关节运动', message)
        self.assertEqual(recovery.poll(61.5)[0], 'none')

    def test_gui_recovery_only_uses_positive_enable_value(self):
        source = (
            ROOT / 'gui' / 'widgets' / 'joint_control_widget.py'
        ).read_text()
        self.assertIn('enable_msg.data = False', source)
        self.assertNotIn('enable_msg.data = True', source)


if __name__ == '__main__':
    unittest.main()
