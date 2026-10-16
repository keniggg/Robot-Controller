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
    DirectCommandGesture,
    DirectControlRecovery,
    JointControlWidget,
)


class JointControlModeTest(unittest.TestCase):
    def test_direct_gesture_holds_every_unedited_joint(self):
        gesture = DirectCommandGesture()
        self.assertTrue(gesture.begin([-1.8, 0.52, -0.20, 0.11], 0))

        first = gesture.compose([-1.7, 0.51, -0.21, 0.10])
        later = gesture.compose([-1.4, 0.47, -0.26, 0.09])

        self.assertEqual(first, [-1.7, 0.52, -0.20, 0.11])
        self.assertEqual(later, [-1.4, 0.52, -0.20, 0.11])

    def test_direct_gesture_clear_allows_a_fresh_feedback_snapshot(self):
        gesture = DirectCommandGesture()
        gesture.begin([1.0, 2.0, 3.0], 0)
        gesture.clear()
        gesture.begin([1.1, 2.1, 3.1], 1)

        self.assertEqual(gesture.compose([9.0, 1.5, 9.0]), [1.1, 1.5, 3.1])

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

    def test_task_moved_arm_rebases_every_unedited_slider_to_fresh_feedback(self):
        recovery = DirectControlRecovery(timeout_sec=2.0)
        recovery.update_actuation_status(
            'OVERHEAT_BLOCKED:SUSTAINED_SAME_CHANNEL_TEMPERATURE'
        )
        recovery.update_feedback_ready(True)
        recovery.note_joint_feedback([1.0, 2.0, 3.0], 9.9)

        # The GUI still displays a pre-task all-zero target, but the operator
        # explicitly edits only Joint2 to 1.5 rad.
        action, _, _ = recovery.submit(
            [0.0, 1.5, 0.0],
            10.0,
            edited_index=1,
        )
        self.assertEqual(action, 'enable')

        recovery.update_actuation_status('PENDING:POSITIVE_ENABLE_REQUESTED')
        recovery.note_joint_feedback([1.1, 2.1, 3.1], 10.1)
        action, sync_target, _ = recovery.poll(10.2)
        self.assertEqual(action, 'sync')
        self.assertEqual(sync_target, [1.1, 2.1, 3.1])

        recovery.update_actuation_status('PENDING:COMMAND_SYNCHRONIZED')
        action, probe, _ = recovery.poll(10.3)
        self.assertEqual(action, 'probe')
        self.assertEqual(probe[0], 1.1)
        self.assertAlmostEqual(probe[1], 2.075)
        self.assertEqual(probe[2], 3.1)

        recovery.update_actuation_status(
            'CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE'
        )
        action, target, _ = recovery.poll(10.4)
        self.assertEqual(action, 'publish')
        self.assertEqual(target, [1.1, 1.5, 3.1])

    def test_locked_gesture_target_is_not_rebased_by_later_feedback(self):
        recovery = DirectControlRecovery(timeout_sec=2.0)
        recovery.update_actuation_status(
            'OVERHEAT_BLOCKED:SUSTAINED_SAME_CHANNEL_TEMPERATURE'
        )
        recovery.update_feedback_ready(True)
        recovery.note_joint_feedback([1.0, 2.0, 3.0], 9.9)

        held_target = [1.0, 1.5, 3.0]
        action, _, _ = recovery.submit(
            held_target,
            10.0,
            edited_index=1,
            rebase_unedited=False,
        )
        self.assertEqual(action, 'enable')

        recovery.update_actuation_status('PENDING:POSITIVE_ENABLE_REQUESTED')
        recovery.note_joint_feedback([1.1, 2.1, 3.1], 10.1)
        self.assertEqual(recovery.poll(10.2)[0], 'sync')
        recovery.update_actuation_status('PENDING:COMMAND_SYNCHRONIZED')
        self.assertEqual(recovery.poll(10.3)[0], 'probe')

        # Probe motion/settling changes the live feedback, but must not become
        # a new target for either unedited joint.
        recovery.note_joint_feedback([1.2, 2.075, 3.2], 10.35)
        recovery.update_actuation_status(
            'CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE'
        )
        action, target, _ = recovery.poll(10.4)
        self.assertEqual(action, 'publish')
        self.assertEqual(target, held_target)

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
        self.assertEqual(recovery.last_action_edited_index, None)

    def test_direct_action_preserves_explicit_edited_joint_metadata(self):
        recovery = DirectControlRecovery(timeout_sec=2.0)
        recovery.update_actuation_status(
            'CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE'
        )
        recovery.update_feedback_ready(True)

        action, target, _ = recovery.submit(
            [0.1, -0.4, 0.2],
            30.0,
            edited_index=1,
        )

        self.assertEqual(action, 'publish')
        self.assertEqual(target, [0.1, -0.4, 0.2])
        self.assertEqual(recovery.last_action_edited_index, 1)

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

    def test_new_slider_reuses_existing_command_synchronization(self):
        recovery = DirectControlRecovery(timeout_sec=2.0)
        recovery.update_actuation_status('PENDING:COMMAND_SYNCHRONIZED')
        recovery.update_feedback_ready(True)
        recovery.note_joint_feedback([0.4, -0.2, 0.1], 69.9)

        action, target, _ = recovery.submit(
            [0.4, -0.5, 0.1],
            70.0,
            edited_index=1,
        )
        self.assertEqual(action, 'wait')
        self.assertIsNone(target)

        # Do not probe from feedback older than this explicit slider action.
        self.assertEqual(recovery.poll(70.05)[0], 'wait')
        recovery.note_joint_feedback([0.41, -0.21, 0.11], 70.1)
        action, probe, _ = recovery.poll(70.2)
        self.assertEqual(action, 'probe')
        self.assertEqual(probe[0], 0.41)
        self.assertAlmostEqual(probe[1], -0.235)
        self.assertEqual(probe[2], 0.11)

        recovery.update_actuation_status(
            'CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE'
        )
        action, target, _ = recovery.poll(70.3)
        self.assertEqual(action, 'publish')
        self.assertEqual(target, [0.41, -0.5, 0.11])

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

    def test_gui_direct_message_marks_exactly_one_edited_channel(self):
        source = (
            ROOT / 'gui' / 'widgets' / 'joint_control_widget.py'
        ).read_text()
        self.assertIn("msg.header.frame_id = (", source)
        self.assertIn("'gui_direct_sync'", source)
        self.assertIn("else 'gui_direct'", source)
        self.assertIn('msg.effort = [0.0] * len(msg.name)', source)
        self.assertIn('msg.effort[index] = 1.0', source)

    def test_failed_recovery_clear_does_not_create_orphan_enable_handshake(self):
        source = (
            ROOT / 'gui' / 'widgets' / 'joint_control_widget.py'
        ).read_text()
        clear_branch = source.split("if action == 'clear':", 1)[1].split(
            "if action == 'timeout':",
            1,
        )[0]
        self.assertNotIn('enable_pub.publish', clear_branch)
        self.assertNotIn('enable_msg.data', clear_branch)


if __name__ == '__main__':
    unittest.main()
