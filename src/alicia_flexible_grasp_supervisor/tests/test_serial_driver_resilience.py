#!/usr/bin/env python3
import pathlib
import re
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[2]
DRIVER_SRC = ROOT / 'real-arm' / 'alicia_d_driver' / 'src'
DRIVER_HEADER = (
    ROOT / 'real-arm' / 'alicia_d_driver' / 'include'
    / 'alicia_d_driver' / 'alicia_d_driver_node.hpp'
)


def _function_body(source, signature):
    start = source.index(signature)
    brace = source.index('{', start)
    depth = 0
    for index in range(brace, len(source)):
        char = source[index]
        if char == '{':
            depth += 1
        elif char == '}':
            depth -= 1
            if depth == 0:
                return source[brace + 1:index]
    raise AssertionError('function body not found: %s' % signature)


class SerialDriverResilienceTest(unittest.TestCase):
    def test_read_thread_does_not_call_full_disconnect_from_inside_itself(self):
        source = (DRIVER_SRC / 'serial_communicator.cpp').read_text()
        body = _function_body(source, 'void SerialCommunicator::read_thread_loop()')

        self.assertNotRegex(
            body,
            re.compile(r'\bdisconnect\s*\('),
            'read_thread_loop must not call disconnect(); self-disconnect races with writers and can crash',
        )
        self.assertIn('handle_read_error_disconnect()', body)

    def test_disconnect_closes_serial_port_under_serial_mutex(self):
        source = (DRIVER_SRC / 'serial_communicator.cpp').read_text()
        body = _function_body(source, 'void SerialCommunicator::disconnect()')

        self.assertIn('std::lock_guard<std::mutex> lock(serial_mutex_)', body)
        self.assertIn('serial_port_.close()', body)

    def test_driver_keeps_reconnect_timer_alive_after_successful_initial_connect(self):
        source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
        ctor_body = _function_body(source, 'AliciaDDriverNode::AliciaDDriverNode()')
        reconnect_body = _function_body(source, 'void AliciaDDriverNode::reconnect_callback')

        self.assertIn('reconnect_timer_ = nh_.createTimer', ctor_body)
        self.assertNotIn('reconnect_timer_.stop()', ctor_body)
        self.assertNotIn('reconnect_timer_.stop()', reconnect_body)

    def test_over_temperature_confirmation_is_consecutive_per_channel(self):
        source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
        header = DRIVER_HEADER.read_text()
        body = _function_body(
            source,
            'void AliciaDDriverNode::parse_sdk_temperature_frame',
        )

        self.assertIn(
            'std::vector<int> consecutive_high_temperature_samples_by_channel_;',
            header,
        )
        self.assertIn(
            'consecutive_high_temperature_samples_by_channel_[i]',
            body,
        )
        self.assertIn(
            'msg.data[i] >=',
            body,
        )
        self.assertIn(
            'channel_streak + 1',
            body,
        )
        self.assertIn(
            'channel_streak = 0;',
            body,
        )
        self.assertIn(
            'consecutive_high_temperature_samples_ =\n'
            '            high_temperature_sample_count;',
            body,
        )
        self.assertNotIn(
            '++consecutive_high_temperature_samples_',
            body,
        )

        # The retained real failure sequence changed the hot channel between
        # frames. A global max streak gives 3/3; a per-channel streak does not.
        threshold = 60.0
        frames = [
            [33, 34, 36, 33, 36, 36, 34, 34, 33, 163],
            [33, 34, 36, 33, 100, 36, 43, 34, 33, 34],
            [33, 34, 36, 33, 100, 36, 43, 34, 33, 34],
            [33, 34, 36, 33, 36, 36, 34, 34, 33, 34],
        ]
        streaks = [0] * len(frames[0])
        maxima = []
        for frame in frames:
            streaks = [
                old + 1 if value >= threshold else 0
                for old, value in zip(streaks, frame)
            ]
            maxima.append(max(streaks))

        self.assertEqual(maxima, [1, 1, 2, 0])
        self.assertNotIn(3, maxima)

    def test_temperature_telemetry_cannot_autonomously_remove_torque(self):
        source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
        joint_body = _function_body(
            source,
            'void AliciaDDriverNode::parse_sdk_joint_state_frame',
        )
        demo_body = _function_body(
            source,
            'void AliciaDDriverNode::demonstration_mode_callback',
        )

        self.assertIn(
            'autonomous torque_off is disabled and motion enable is unchanged',
            joint_body,
        )
        self.assertNotIn('trigger_torque_off', joint_body)
        self.assertNotIn('motion_commands_enabled_ = false', joint_body)
        self.assertNotIn('torque_off_frame', joint_body)

        # The explicit operator zero-torque command remains available; the
        # prohibition applies to autonomous temperature/status handling.
        self.assertIn('if (msg->data)', demo_body)
        self.assertIn('motion_commands_enabled_ = false', demo_body)
        self.assertIn('torque_off_frame', demo_body)

    def test_single_implausible_joint_feedback_frame_requires_confirmation(self):
        source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
        header = DRIVER_HEADER.read_text()
        body = _function_body(
            source,
            'void AliciaDDriverNode::parse_sdk_joint_state_frame',
        )

        self.assertIn('reject_implausible_joint_feedback_', header)
        self.assertIn('feedback_max_velocity_rad_s_', header)
        self.assertIn('pending_joint_feedback_', header)
        self.assertIn(
            'reject_command_inconsistent_joint_feedback_',
            header,
        )
        self.assertIn('last_streamed_joint_positions_', header)
        self.assertIn(
            'Rejected implausible one-frame SDK joint feedback',
            body,
        )
        self.assertIn(
            'pending_joint_feedback_count_ >=',
            body,
        )
        self.assertIn(
            'current_joint_positions_ = candidate_joint_positions;',
            body,
        )
        self.assertIn(
            'Rejected SDK joint feedback discontinuity inconsistent with fresh streamed command',
            body,
        )
        self.assertIn(
            'candidate_command_error_rad >',
            body,
        )
        self.assertIn(
            'previous_command_error_rad +',
            body,
        )

    def test_driver_restart_seeds_command_interpolator_from_real_feedback(self):
        source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
        header = DRIVER_HEADER.read_text()
        send_body = _function_body(
            source,
            'void AliciaDDriverNode::send_command_timer_callback',
        )

        self.assertIn('command_state_seeded_from_feedback_', header)
        self.assertIn(
            'cmd_joint_angles_ = feedback_joint_angles;',
            send_body,
        )
        self.assertIn(
            'cmd_gripper_rad_ = feedback_gripper_rad;',
            send_body,
        )
        self.assertLess(
            send_body.index('cmd_joint_angles_ = feedback_joint_angles;'),
            send_body.index('// Interpolate toward latest command'),
        )

        # Retained failure: commanded/accepted Joint6 stayed near +8.3 deg,
        # while the controller's 0.621719 rad error implies one actual sample
        # near -27.3 deg. At the deployed 10 Hz feedback cadence this exceeds
        # the same generic temporal bound used for every joint.
        base_tolerance = 0.0123
        max_velocity = 1.2
        frame_dt = 0.1
        allowed = base_tolerance + max_velocity * frame_dt
        implied_jump = 0.621719
        ordinary_motion = 0.026
        self.assertLess(ordinary_motion, allowed)
        self.assertGreater(implied_jump, allowed)

        # A real discontinuity is delayed for one sample, then accepted only
        # when a second sample independently confirms the same new state.
        confirmation_tolerance = 0.05
        pending = -0.477
        next_confirming_sample = -0.451
        self.assertLessEqual(
            abs(next_confirming_sample - pending),
            confirmation_tolerance,
        )


if __name__ == '__main__':
    unittest.main()
