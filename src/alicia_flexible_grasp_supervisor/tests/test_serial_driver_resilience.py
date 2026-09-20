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
DRIVER_PACKAGE = ROOT / 'real-arm' / 'alicia_d_driver'


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
    def test_sdk_feedback_clock_and_snapshot_are_captured_under_the_same_lock(self):
        source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
        body = _function_body(source, 'void AliciaDDriverNode::send_command_timer_callback')
        lock = body.index('std::lock_guard<std::mutex> lock(data_mutex_);')
        self.assertNotIn('ros::Time::now()', body[:lock])
        snapshot = _function_body(body, 'ros::Time feedback_sample_time;')
        self.assertLess(snapshot.index('lock(data_mutex_)'),
                        snapshot.index('now = ros::Time::now();'))
        self.assertLess(snapshot.index('now = ros::Time::now();'),
                        snapshot.index('feedback_sample_time = last_accepted_joint_feedback_time_;'))
        self.assertIn('feedback_age_sec = (now - feedback_sample_time).toSec();', snapshot)
        self.assertIn('feedback_joint_angles = current_joint_positions_;', snapshot)

        # Deterministic interleaving that caused the false future-stamp gate:
        # callback enters, parser wins data_mutex and timestamps its sample,
        # then callback acquires data_mutex. No real clock or ROS is involved.
        callback_entry, accepted_stamp, locked_now = 100.0, 100.005, 100.006
        self.assertLess(callback_entry - accepted_stamp, 0.0)
        self.assertGreaterEqual(locked_now - accepted_stamp, 0.0)
        self.assertLess(locked_now - accepted_stamp, .002)

    def test_sdk_feedback_age_gate_preserves_real_future_expired_and_missing_rejection(self):
        source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
        body = _function_body(source, 'void AliciaDDriverNode::send_command_timer_callback')
        snapshot = _function_body(body, 'ros::Time feedback_sample_time;')
        predicate = re.search(r'feedback_stale\s*=\s*(.*?);', snapshot, re.S).group(1)
        self.assertIn('feedback_age_sec < 0.0', predicate)
        self.assertIn('feedback_age_sec >', predicate)
        # Evaluate the actual, restricted C++ Boolean predicate rather than
        # a second hand-written implementation of its boundary conditions.
        predicate = predicate.replace('feedback_sample_time.isZero()', 'stamp_is_zero')
        predicate = predicate.replace('||', ' or ').replace('&&', ' and ')
        predicate = re.sub(r'!(?!=)', 'not ', predicate)
        predicate = ' '.join(predicate.split())
        cases = [
            (True, False, .001, False),
            (True, False, 0.0, False),
            (True, False, 1.0, False),
            (True, False, 1.000001, True),
            (True, False, -1e-9, True),
            (False, False, .001, True),
            (True, True, .001, True),
        ]
        for ready, zero, age, expected in cases:
            with self.subTest(ready=ready, zero=zero, age=age):
                self.assertEqual(eval(predicate, {'__builtins__': {}}, {
                    'feedback_ready': ready,
                    'has_real_feedback_': ready,
                    'stamp_is_zero': zero,
                    'feedback_age_sec': age,
                    'feedback_stale_timeout_sec_': 1.0,
                }), expected)

    def test_sdk_feedback_pause_log_distinguishes_future_from_expired_timestamp(self):
        source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
        body = _function_body(source, 'void AliciaDDriverNode::send_command_timer_callback')
        diagnostic = body[body.index('if (pause_commands_when_feedback_stale_ &&'):]
        diagnostic = diagnostic[:diagnostic.index('// A driver-only restart')]
        for reason in ('not_ready', 'zero_stamp', 'future_stamp', 'expired'):
            self.assertIn('"' + reason + '"', diagnostic)
        self.assertIn('reason=%s age=%.6fs timeout=%.2fs', diagnostic)
        self.assertIn('feedback_age_sec', diagnostic)

    def test_heartbeat_uses_locked_accepted_arm_feedback_not_generic_traffic(self):
        source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
        body = _function_body(source, 'void AliciaDDriverNode::heartbeat_publish_callback')
        self.assertLess(body.index('lock2(data_mutex_)'), body.index('ros::Time::now()'))
        self.assertEqual(body.count('lock2(data_mutex_)'), 1)
        self.assertIn('feedback_sample_time = last_accepted_joint_feedback_time_;', body)
        self.assertIn('feedback_age = (now - feedback_sample_time).toSec();', body)
        self.assertNotIn('feedback_age = (now - last_feedback_time_).toSec();', body)
        for reason in ('not_ready', 'zero_stamp', 'future_stamp', 'expired'):
            self.assertIn('"' + reason + '"', body)
        self.assertIn('reason=%s age=%.6fs timeout=%.2fs', body)

    def test_heartbeat_status_only_activity_does_not_renew_arm_freshness(self):
        source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
        body = _function_body(source, 'void AliciaDDriverNode::heartbeat_publish_callback')
        self.assertIn('feedback_sample_time = last_accepted_joint_feedback_time_;', body)
        predicate = re.search(r'const bool feedback_ready\s*=\s*(.*?);', body, re.S).group(1)
        predicate = predicate.replace('feedback_sample_time.isZero()', 'stamp_is_zero')
        predicate = predicate.replace('||', ' or ').replace('&&', ' and ')
        predicate = re.sub(r'!(?!=)', 'not ', predicate)
        predicate = ' '.join(predicate.split())
        # Generic/status traffic can advance last_feedback_time_ indefinitely;
        # only an accepted complete arm sample may renew this Boolean gate.
        cases = [
            (True, 100.0, 100.1, 100.1, True),
            (True, 100.0, 101.0, 101.0, True),
            (True, 100.0, 101.000001, 101.000001, False),
            (True, 100.0, 120.0, 120.0, False),
            (True, 100.0, 99.999999, 99.999999, False),
            (True, 0.0, .1, .1, False),
            (False, 100.0, 100.1, 100.1, False),
        ]
        for has_arm, accepted, now, generic_stamp, expected in cases:
            with self.subTest(has_arm=has_arm, accepted=accepted, now=now):
                self.assertEqual(eval(predicate, {'__builtins__': {}}, {
                    'has_real_feedback_': has_arm,
                    'stamp_is_zero': accepted == 0.0,
                    'feedback_age': now - accepted,
                    'feedback_stale_timeout_sec_': 1.0,
                    'last_feedback_time_': generic_stamp,
                }), expected)

    def test_accepted_arm_diagnostic_is_a_separate_unlatched_joint_state_topic(self):
        source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
        header = DRIVER_HEADER.read_text()
        setup = _function_body(source, 'void AliciaDDriverNode::setup_ros_communications')
        self.assertIn('ros::Publisher accepted_joint_state_pub_;', header)
        self.assertIn('void publish_joint_state(bool accepted_sdk_sample = false);', header)
        self.assertIn('accepted_joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/alicia_d/accepted_joint_states", 20);', setup)

    def test_only_complete_accepted_sdk_frame_can_publish_arm_sample_evidence(self):
        source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
        parser = _function_body(source, 'void AliciaDDriverNode::parse_sdk_joint_state_frame')
        self.assertEqual(source.count('publish_joint_state(true);'), 1)
        self.assertIn('if (accept_joint_positions) {\n        publish_joint_state(true);', parser)
        for prerequisite in ('current_joint_positions_ = candidate_joint_positions;',
                             'last_accepted_joint_feedback_time_ = feedback_time;',
                             'current_gripper_position_ =', 'last_run_status_ = data_payload[14];'):
            self.assertLess(parser.index(prerequisite), parser.index('publish_joint_state(true);'))
        for signature in ('void AliciaDDriverNode::heartbeat_publish_callback',
                          'void AliciaDDriverNode::parse_servo_states_frame',
                          'void AliciaDDriverNode::parse_gripper_state_frame',
                          'void AliciaDDriverNode::parse_sdk_temperature_frame',
                          'void AliciaDDriverNode::parse_sdk_self_check_frame',
                          'void AliciaDDriverNode::parse_error_frame'):
            self.assertNotIn('publish_joint_state(true)', _function_body(source, signature))

    def test_accepted_arm_message_has_original_sample_time_and_complete_gripper(self):
        source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
        body = _function_body(source, 'void AliciaDDriverNode::publish_joint_state')
        self.assertIn('js_msg.header.stamp = now;', body)  # Existing TF compatibility unchanged.
        self.assertIn('sensor_msgs::JointState accepted_msg = js_msg;', body)
        self.assertLess(body.index('js_msg.position.push_back(gripper_m);'),
                        body.index('sensor_msgs::JointState accepted_msg = js_msg;'))
        self.assertIn('accepted_msg.header.stamp = last_accepted_joint_feedback_time_;', body)
        self.assertIn('accepted_msg.header.frame_id = "sdk_measured";', body)
        self.assertIn('if (accepted_sdk_sample && has_real_feedback_', body)
        self.assertIn('!last_accepted_joint_feedback_time_.isZero()', body)
        self.assertIn('js_msg.name.size() == 7 && js_msg.position.size() == 7', body)
        self.assertIn('accepted_joint_state_pub_.publish(accepted_msg);', body)

    def test_ownership_transition_preserves_wire_hold_without_new_motion_authority(self):
        source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
        body = _function_body(source, 'void AliciaDDriverNode::gui_control_mode_callback')
        self.assertIn('decode_retained_sdk_hold(', body)
        self.assertIn('has_latest_command_ = retained_hold;', body)
        self.assertIn('command_state_seeded_from_feedback_ = retained_hold;', body)
        self.assertIn('cmd_joint_angles_ = hold;', body)
        self.assertIn('cmd_joint_velocities_.assign(6, 0.0)', body)
        self.assertIn('control_mode_needs_sync_ = !gui_control_mode_', body)
        self.assertIn('control_mode_hold_only_ = true;', body)
        self.assertIn('reset_motion_observation()', body)
        self.assertNotIn('last_sent_sdk_command_frame_.clear()', body)
        self.assertNotIn('reset_command_synchronization()', body)
        self.assertNotIn('write_raw_frame(', body)
        self.assertNotIn('request_positive_enable(', body)

    def test_direct_command_accumulates_only_admitted_targets_with_accepted_feedback(self):
        source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
        body = _function_body(source, 'void AliciaDDriverNode::joint_command_callback')
        self.assertIn('select_gui_direct_command_reference(', body)
        self.assertNotIn('const bool continue_gesture', body)
        self.assertNotIn('streamed_hold_max_age_sec', body)
        self.assertIn('command_time - last_accepted_joint_feedback_time_', body)
        self.assertNotIn('command_time - last_feedback_time_', body)
        self.assertLess(body.index('actuation_confirmation_.admit_command('),
                        body.index('gui_direct_hold_joint_angles_ = joint_angles;'))
        self.assertLess(body.index('last_observation_accepted()'),
                        body.index('gui_direct_hold_joint_angles_ = joint_angles;'))
        self.assertLess(body.index('has_latest_command_ = true;'),
                        body.index('control_mode_needs_sync_ = false;'))
        self.assertIn('static_cast<int>(i) != gui_direct_edited_index', body)

    def test_auto_handoff_uses_only_frozen_successfully_written_sdk_words(self):
        source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
        body = _function_body(source, 'void AliciaDDriverNode::joint_command_callback')
        handoff = _function_body(body, 'if (automatic_command_needs_sync)')
        self.assertIn('controller_handoff_matches_frozen_sdk_or_initial(', handoff)
        self.assertIn('last_sent_sdk_command_frame_', handoff)
        self.assertIn('control_mode_needs_sync_ && control_mode_hold_only_', handoff)
        self.assertIn('retained_sdk_age_sec >= 0.0', handoff)
        self.assertIn('retained_sdk_age_sec <= feedback_stale_timeout_sec_', handoff)
        self.assertNotIn('latest_joint_angles_', handoff)
        self.assertNotIn('last_streamed_joint_positions_', handoff)
        self.assertLess(body.index('mode_lock(control_mode_mutex_)'),
                        body.index('controller_handoff_matches_frozen_sdk_or_initial('))
        self.assertLess(body.index('controller_handoff_matches_frozen_sdk_or_initial('),
                        body.index('actuation_confirmation_.admit_command('))

    def test_auto_handoff_samples_fresh_feedback_clock_under_its_data_lock(self):
        source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
        body = _function_body(source, 'void AliciaDDriverNode::joint_command_callback')
        handoff = _function_body(body, 'if (automatic_command_needs_sync)')
        self.assertLess(handoff.index('data_lock(data_mutex_)'),
                        handoff.index('handoff_now = ros::Time::now()'))
        self.assertIn('handoff_now - last_accepted_joint_feedback_time_', handoff)
        self.assertIn('age >= 0.0 && age <= feedback_stale_timeout_sec_', handoff)
        self.assertIn('frozen SDK position words', handoff)

    def test_auto_handoff_invalid_or_expired_wire_never_falls_back_to_feedback(self):
        source = (DRIVER_PACKAGE / 'include' / 'alicia_d_driver' /
                  'gui_direct_hold.hpp').read_text()
        body = _function_body(source, 'inline bool controller_handoff_matches_frozen_sdk_or_initial')
        self.assertIn('if (last_written_frame.empty())', body)
        self.assertIn('return controller_handoff_matches_feedback(', body)
        self.assertIn('if (!retained_frozen_and_fresh)', body)
        self.assertIn('decode_retained_sdk_hold(', body)
        self.assertIn('sdk_joint_position_encode(command[i])', body)
        self.assertIn('sdk_gripper_position_encode(command_gripper_rad)', body)
        self.assertEqual(body.count('controller_handoff_matches_feedback('), 1)

    def test_dedicated_reference_sync_checks_reset_epoch_under_the_mode_lock(self):
        source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
        reset = _function_body(source, 'void AliciaDDriverNode::clear_retained_command_state')
        self.assertLess(reset.index('mode_lock(control_mode_mutex_)'),
                        reset.index('control_reference_reset_time_ = ros::Time::now();'))
        body = _function_body(source, 'void AliciaDDriverNode::joint_command_callback')
        self.assertIn('"motion_gateway_reference_sync"', body)
        self.assertLess(body.index('mode_lock(control_mode_mutex_)'),
                        body.index('reference_sync_stamp_in_current_epoch('))
        self.assertIn('msg->header.stamp.toNSec()', body)
        self.assertIn('control_reference_reset_time_.toNSec()', body)
        self.assertIn('control_mode_changed_time_.toNSec()', body)
        self.assertIn('command_time.toNSec()', body)

    def test_dedicated_reference_sync_is_arm_only_and_tracking_noop(self):
        source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
        body = _function_body(source, 'void AliciaDDriverNode::joint_command_callback')
        self.assertIn('msg->name.size() != hardware_joint_names.size()', body)
        self.assertIn('msg->position.size() != hardware_joint_names.size()', body)
        self.assertIn('joint_map.size() != hardware_joint_names.size()', body)
        self.assertIn('joint_map.count(name) != 1', body)
        proof = body.index('reference_sync_matches_successful_sdk_or_initial(')
        noop = body.index('if (!automatic_command_needs_sync)', proof)
        self.assertIn('return;', _function_body(body[noop:], 'if (!automatic_command_needs_sync)'))
        self.assertLess(noop, body.index('actuation_confirmation_.admit_command('))
        self.assertLess(noop, body.index('latest_joint_angles_ = joint_angles;'))
        self.assertIn('last_sent_sdk_command_frame_, sdk_is_fresh, actuation_status', body)
        self.assertNotIn('write_raw_frame(', body)

    def test_dedicated_reference_sync_samples_accepted_clock_and_status_without_widening_init(self):
        source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
        body = _function_body(source, 'void AliciaDDriverNode::joint_command_callback')
        proof = _function_body(body, '// Dedicated reference-only admission')
        self.assertLess(proof.index('data_lock(data_mutex_)'),
                        proof.index('reference_now = ros::Time::now()'))
        self.assertIn('reference_now - last_accepted_joint_feedback_time_', proof)
        self.assertIn('feedback_age_sec >= 0.0', proof)
        self.assertIn('feedback_age_sec <= feedback_stale_timeout_sec_', proof)
        self.assertIn('actuation_confirmation_.status_text()', proof)
        self.assertIn('sdk_age_sec >= 0.0', proof)
        self.assertIn('sdk_age_sec <= feedback_stale_timeout_sec_', proof)
        helper = (DRIVER_PACKAGE / 'include' / 'alicia_d_driver' / 'gui_direct_hold.hpp').read_text()
        sync = _function_body(helper, 'inline bool reference_sync_matches_successful_sdk_or_initial')
        self.assertIn('"PENDING:POSITIVE_ENABLE_REQUESTED"', sync)
        self.assertIn('"PENDING:COMMAND_SYNCHRONIZED"', sync)
        self.assertIn('controller_handoff_matches_frozen_sdk_or_initial(', sync)

    def test_reference_epoch_is_latched_reset_only_telemetry_not_pending_heartbeat(self):
        source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
        setup = _function_body(source, 'void AliciaDDriverNode::setup_ros_communications')
        self.assertIn('control_reference_epoch_pub_', DRIVER_HEADER.read_text())
        self.assertIn('"/alicia_d/control_reference_epoch", 1, true)', setup)
        reset = _function_body(source, 'void AliciaDDriverNode::clear_retained_command_state')
        self.assertIn('reference_epoch_next_stamp_ns(', reset)
        self.assertIn('epoch_msg.stamp = control_reference_reset_time_;', reset)
        self.assertIn('epoch_msg.frame_id = "sdk_reference_epoch";', reset)
        self.assertIn('if (control_reference_epoch_pub_)', reset)
        self.assertIn('control_reference_epoch_pub_.publish(epoch_msg);', reset)
        self.assertLess(reset.index('last_sent_sdk_command_frame_.clear();'),
                        reset.index('control_reference_epoch_pub_.publish(epoch_msg);'))
        self.assertEqual(source.count('control_reference_epoch_pub_.publish('), 1)
        self.assertNotIn('control_reference_epoch_pub_',
                         _function_body(source, 'void AliciaDDriverNode::publish_actuation_status'))

    def test_successful_sdk_write_reports_unlatched_control_reference_state(self):
        source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
        setup = _function_body(source, 'void AliciaDDriverNode::setup_ros_communications')
        self.assertIn('control_reference_pub_', DRIVER_HEADER.read_text())
        self.assertIn('"/alicia_d/control_reference", 20)', setup)
        body = _function_body(source, 'void AliciaDDriverNode::send_command_timer_callback')
        written = _function_body(body, 'if (wrote)')
        self.assertIn('control_reference_msg.header.stamp = now;', written)
        self.assertIn('"sdk_manual"', written)
        self.assertIn('"sdk_handoff_required"', written)
        self.assertIn('"sdk_tracking"', written)
        self.assertIn('"sdk_reference_unconfirmed"', written)
        self.assertIn('control_mode_needs_sync_ && control_mode_hold_only_', written)
        self.assertIn('control_reference_msg.name.push_back("right_finger");', written)
        self.assertIn('static_cast<double>(gripper_hw_val) / 1000.0 * gripper_stroke_m', written)
        self.assertIn('control_reference_pub_.publish(control_reference_msg);', written)
        self.assertIn('wire_msg.header.frame_id = "sdk_transmitted";', written)
        self.assertEqual(source.count('control_reference_pub_.publish('), 1)

    def test_successful_sdk_write_publishes_quantized_diagnostic_and_manual_has_no_trim(self):
        source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
        body = _function_body(source, 'void AliciaDDriverNode::send_command_timer_callback')
        self.assertIn('!gui_control_mode_ && (endpoint_feedback_trim_enabled_', body)
        self.assertIn('last_streamed_joint_positions_ = wire_joint_angles;', body)
        self.assertIn('last_streamed_gripper_rad_ = wire_gripper_rad;', body)
        self.assertIn('if (!control_mode_hold_only_)', body)
        self.assertLess(body.index('if (wrote)'), body.index('sdk_command_pub_.publish(wire_msg)'))
        self.assertIn('wire_msg.position = wire_joint_angles;', body)
        reset = _function_body(source, 'void AliciaDDriverNode::clear_retained_command_state')
        self.assertIn('std::lock_guard<std::mutex> mode_lock(control_mode_mutex_)', reset)
        for cache in ('gui_direct_hold_joint_angles_', 'last_streamed_joint_positions_', 'last_sent_sdk_command_frame_'):
            self.assertIn(cache + '.clear()', reset)

    def test_full_bringup_uses_one_shot_controller_loader_without_shutdown_stop(self):
        launch = (
            DRIVER_PACKAGE / 'launch' / 'alicia_d_bringup.launch'
        ).read_text()
        loader = (
            DRIVER_PACKAGE / 'scripts' / 'controller_loader_once.py'
        ).read_text()

        self.assertIn('type="controller_loader_once.py"', launch)
        self.assertNotIn('type="spawner"', launch)
        self.assertNotIn('rospy.on_shutdown', loader)
        self.assertNotIn('UnloadController', loader)
        self.assertIn('switch_controller(\n            to_start,\n            [],', loader)

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

    def test_physically_impossible_temperature_channel_is_excluded_without_hiding_valid_channels(self):
        source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
        header = DRIVER_HEADER.read_text()
        body = _function_body(
            source,
            'void AliciaDDriverNode::parse_sdk_temperature_frame',
        )

        self.assertIn('max_plausible_temperature_c_', header)
        self.assertIn(
            'std::numeric_limits<float>::quiet_NaN()',
            body,
        )
        rejection = body.index('Rejected SDK temperature telemetry')
        telemetry_refresh = body.index('latest_temperatures_c_ = msg.data;')
        conversion_complete = body.index('msg.data.push_back(value);')
        self.assertGreater(rejection, telemetry_refresh)
        self.assertIn(
            'if (std::isfinite(value))',
            body,
        )
        self.assertNotIn(
            'return;',
            body[conversion_complete:telemetry_refresh],
        )
        self.assertIn(
            'consecutive_high_temperature_samples_by_channel_[i]',
            body,
        )

    def test_temperature_slew_filter_rejects_short_uart_like_spike_but_not_real_rise(self):
        source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
        header = DRIVER_HEADER.read_text()
        body = _function_body(
            source,
            'void AliciaDDriverNode::parse_sdk_temperature_frame',
        )

        self.assertIn('max_temperature_slew_c_per_sec_', header)
        self.assertIn('temperature_slew_tolerance_c_', header)
        self.assertIn('last_plausible_temperatures_c_', header)
        self.assertIn('last_plausible_temperature_times_', header)
        self.assertIn('allowed_delta_c', body)
        self.assertIn('slew_rejected_temperature_channels', body)

        max_slew_c_per_sec = 8.0
        tolerance_c = 5.0
        previous = 36.0
        previous_time = 0.0

        def admit(value, now):
            nonlocal previous, previous_time
            allowed = tolerance_c + max_slew_c_per_sec * (now - previous_time)
            if abs(value - previous) > allowed:
                return False
            previous = value
            previous_time = now
            return True

        # The retained 36 -> 98 C corruption lasted five seconds. Comparing
        # every frame with the last plausible sample keeps all five invalid.
        self.assertEqual(
            [admit(98.0, second) for second in range(1, 6)],
            [False, False, False, False, False],
        )
        self.assertTrue(admit(36.0, 6.0))

        # A physically progressive rise remains accepted and still supplies
        # three consecutive >=60 C samples to the unchanged protection gate.
        accepted = [
            value
            for second, value in enumerate((48.0, 59.0, 60.0, 61.0, 62.0), 7)
            if admit(value, float(second))
        ]
        self.assertEqual(accepted[-3:], [60.0, 61.0, 62.0])

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

    def test_endpoint_trim_admission_is_owned_by_one_serialized_coordinator(self):
        source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
        header = DRIVER_HEADER.read_text()
        timer = _function_body(
            source,
            'void AliciaDDriverNode::send_command_timer_callback',
        )
        joint_command = _function_body(
            source,
            'void AliciaDDriverNode::joint_command_callback',
        )
        self.assertIn(
            '#include "alicia_d_driver/endpoint_trim_continuity.hpp"',
            header,
        )
        self.assertEqual(
            header.count('EndpointTrimContinuity endpoint_trim_continuity_;'),
            1,
        )
        self.assertEqual(
            source.count('endpoint_trim_continuity_.note_feedback('),
            1,
        )
        self.assertIn(
            'feedback_sample_time = last_accepted_joint_feedback_time_;',
            timer,
        )
        self.assertNotIn(
            'feedback_sample_time = last_feedback_time_;',
            timer,
        )
        self.assertIn('accepted_feedback_sample_is_new', timer)
        self.assertNotIn(
            'endpoint_trim_last_feedback_sample_time_ = ros::Time(0);',
            joint_command,
        )
        self.assertIn(
            'endpoint_trim_decision.phase == EndpointTrimPhase::ACTIVE_READY',
            timer,
        )
        self.assertIn(
            'endpoint_trim_continuity_.request_correction(',
            timer,
        )
        self.assertNotIn('retry_stalled_endpoint_feedback_trim', timer)
        self.assertIn(
            'const std::vector<double> sdk_joint_angles = '
            'endpoint_trim_stream_target(',
            timer,
        )

    def test_endpoint_trim_release_paths_do_not_mutate_offsets_directly(self):
        source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
        timer = _function_body(
            source,
            'void AliciaDDriverNode::send_command_timer_callback',
        )
        service = _function_body(
            source,
            'bool AliciaDDriverNode::set_task_endpoint_precision_callback',
        )

        self.assertNotIn('endpoint_feedback_trim_offsets_', service)
        self.assertNotIn('expired_lease_measured_hold_latched', timer)
        self.assertIn('endpoint_trim_continuity_.request_release(', service)
        expiry_start = timer.index(
            'endpoint_feedback_trim_task_lease_expired = true;'
        )
        expiry_end = timer.index(
            '// Lease-expiry release request end',
            expiry_start,
        )
        expiry_path = timer[expiry_start:expiry_end]
        self.assertIn('endpoint_trim_continuity_.request_release(', expiry_path)
        self.assertNotIn('endpoint_feedback_trim_offsets_', expiry_path)

        trim_start = timer.index('// Endpoint trim continuity begin')
        trim_end = timer.index('// Endpoint trim continuity end', trim_start)
        endpoint_trim_path = service + timer[trim_start:trim_end]
        for forbidden in (
            'torque_off_frame',
            'CMD_DEMO_CONTROL',
            'motion_commands_enabled_ = false',
            'generate_simple_frame(',
            'ros::ServiceClient',
        ):
            self.assertNotIn(forbidden, endpoint_trim_path)

    def test_endpoint_trim_orders_upstream_commands_and_release_by_generation(self):
        source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
        header = DRIVER_HEADER.read_text()
        callback = _function_body(
            source,
            'void AliciaDDriverNode::joint_command_callback',
        )
        timer = _function_body(
            source,
            'void AliciaDDriverNode::send_command_timer_callback',
        )
        service = _function_body(
            source,
            'bool AliciaDDriverNode::set_task_endpoint_precision_callback',
        )
        admission = (
            DRIVER_SRC.parent / 'include' / 'alicia_d_driver'
            / 'endpoint_trim_driver_admission.hpp'
        ).read_text()

        self.assertIn(
            '#include "alicia_d_driver/endpoint_trim_driver_admission.hpp"',
            header,
        )
        self.assertIn('EndpointTrimCommandOrder endpoint_trim_command_order_;', header)
        self.assertNotIn('endpoint_trim_release_requires_timer_install_', header)
        self.assertNotIn('endpoint_trim_release_requires_timer_install_', source)
        self.assertIn('endpoint_trim_command_order_.observe_upstream_command(', callback)
        self.assertIn('endpoint_trim_command_order_.record_release(', service)
        self.assertIn('endpoint_trim_command_order_.record_release(', timer)
        self.assertIn('newer_task_command_requires_handoff()', timer)
        self.assertNotIn('explicit_gui_handoff(', timer)
        self.assertIn('endpoint_trim_command_order_.mark_command_applied();', timer)
        self.assertIn('upstream_target_', admission)
        self.assertIn('command_generation_', admission)
        self.assertIn('release_generation_', admission)
        self.assertNotIn(
            'endpoint_trim_reference_joint_angles_.size() != joint_angles.size()',
            callback,
        )
        self.assertNotIn('endpoint_trim_reference_joint_angles_ = joint_angles;', callback)
        self.assertNotIn('endpoint_feedback_trim_offsets_.assign(', callback)

    def test_endpoint_trim_release_and_expiry_share_pending_handoff_contract(self):
        source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
        service = _function_body(
            source,
            'bool AliciaDDriverNode::set_task_endpoint_precision_callback',
        )
        timer = _function_body(
            source,
            'void AliciaDDriverNode::send_command_timer_callback',
        )

        self.assertIn('endpoint_trim_continuity_.request_release(', service)
        expiry_start = timer.index(
            'endpoint_feedback_trim_task_lease_expired = true;'
        )
        expiry_end = timer.index(
            '// Lease-expiry release request end',
            expiry_start,
        )
        expiry = timer[expiry_start:expiry_end]
        self.assertIn('endpoint_trim_continuity_.request_release(', expiry)
        self.assertIn('EndpointTrimReleaseStatus::PENDING', service)
        self.assertIn('EndpointTrimReleaseStatus::COMPLETED', service)
        self.assertNotIn('release queued', service)
        for path in (service, expiry):
            self.assertNotIn('endpoint_feedback_trim_offsets_', path)
            self.assertNotIn('endpoint_trim_response_wait_since_', path)
            self.assertNotIn('endpoint_trim_feedback_anchor_joint_angles_', path)

    def test_explicit_gui_handoff_is_installed_at_the_accepted_boundary(self):
        source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
        callback = _function_body(
            source,
            'void AliciaDDriverNode::joint_command_callback',
        )

        admission = callback.index('actuation_confirmation_.admit_command(')
        handoff = callback.index('endpoint_trim_continuity_.explicit_gui_handoff(')
        self.assertGreater(handoff, admission)
        self.assertIn('if (endpoint_trim_explicit_gui_command)', callback)
        self.assertIn(
            'endpoint_trim_reference_joint_angles_ =\n'
            '                gui_handoff.reference;',
            callback,
        )
        self.assertIn(
            'endpoint_feedback_trim_offsets_ = gui_handoff.offsets;',
            callback,
        )
        self.assertIn('endpoint_trim_command_order_.mark_command_applied();', callback)
        self.assertIn('endpoint_feedback_trim_task_lease_active_ = false;', callback)
        self.assertIn('endpoint_trim_response_wait_since_ = ros::Time(0);', callback)
        self.assertIn('EndpointTrimCommandSource::GUI_DIRECT_EDIT', callback)
        self.assertIn('EndpointTrimCommandSource::GUI_DIRECT_SYNC', callback)

    def test_endpoint_trim_release_logs_exact_terminal_codes_and_fresh_age(self):
        source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
        timer = _function_body(
            source,
            'void AliciaDDriverNode::send_command_timer_callback',
        )

        self.assertIn('"ENDPOINT_TRIM_RELEASE_SETTLED"', timer)
        self.assertIn('"ENDPOINT_TRIM_RESPONSE_TIMEOUT"', timer)
        self.assertIn('"ENDPOINT_TRIM_CONTINUITY_VIOLATION"', timer)
        self.assertIn('endpoint_trim_terminal_release_code', timer)
        self.assertIn('endpoint_trim_response_generation_started', timer)
        generation = timer.index('endpoint_trim_response_generation_started')
        age = timer.index('endpoint_trim_response_age_sec =', generation)
        self.assertLess(generation, age)

    def test_endpoint_trim_command_order_tracks_source_authority(self):
        admission = (
            DRIVER_SRC.parent / 'include' / 'alicia_d_driver'
            / 'endpoint_trim_driver_admission.hpp'
        ).read_text()

        self.assertIn('enum class EndpointTrimCommandSource', admission)
        self.assertIn('TASK_CONTROLLER', admission)
        self.assertIn('GUI_DIRECT_EDIT', admission)
        self.assertIn('GUI_DIRECT_SYNC', admission)
        self.assertIn('authoritative_source_', admission)
        self.assertIn('last_observation_accepted_', admission)
        self.assertIn(
            'newer_explicit_gui_command_requires_handoff',
            admission,
        )
        self.assertIn('newer_task_command_requires_handoff', admission)
        self.assertIn('gui_task_holdoff_sec_', admission)
        self.assertIn('gui_task_holdoff_until_sec_', admission)
        self.assertIn('now_sec <= gui_task_holdoff_until_sec_', admission)

    def test_task_authority_is_classified_and_timed_in_committed_source(self):
        source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
        callback = _function_body(
            source,
            'void AliciaDDriverNode::joint_command_callback',
        )

        self.assertIn(
            'EndpointTrimCommandSource endpoint_trim_command_source',
            callback,
        )
        self.assertIn('EndpointTrimCommandSource::GUI_DIRECT_EDIT', callback)
        self.assertIn('EndpointTrimCommandSource::GUI_DIRECT_SYNC', callback)
        self.assertIn('EndpointTrimCommandSource::TASK_CONTROLLER', callback)
        observe = callback.index(
            'endpoint_trim_command_order_.observe_upstream_command('
        )
        handoff = callback.index(
            'endpoint_trim_continuity_.explicit_gui_handoff('
        )
        self.assertLess(observe, handoff)
        self.assertIn(
            'endpoint_trim_command_source',
            callback[observe:handoff],
        )
        self.assertIn('command_time.toSec()', callback[observe:handoff])
        self.assertNotIn('task_controller_handoff_permitted', callback)

    def test_optional_gui_direct_guard_uses_the_same_default_boundary(self):
        source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
        callback = _function_body(
            source,
            'void AliciaDDriverNode::joint_command_callback',
        )
        admission = (
            DRIVER_SRC.parent / 'include' / 'alicia_d_driver'
            / 'endpoint_trim_driver_admission.hpp'
        ).read_text()

        # The committed arbiter remains self-contained. If the separate
        # operator GUI-hold implementation is present, its default inclusive
        # 0.25 s boundary must agree instead of creating a wider second gate.
        self.assertIn('double gui_task_holdoff_sec = 0.25', admission)
        self.assertIn('now_sec <= gui_task_holdoff_until_sec_', admission)
        if 'gui_direct_gesture_timeout_sec_' not in callback:
            return
        self.assertIn(
            'direct_age_sec <= gui_direct_gesture_timeout_sec_',
            callback,
        )
        self.assertIn(
            'gui_direct_gesture_timeout_sec_,\n        0.25',
            source,
        )

    def test_release_service_is_release_only_and_reports_actual_outcome(self):
        source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
        service = _function_body(
            source,
            'bool AliciaDDriverNode::set_task_endpoint_precision_callback',
        )

        self.assertNotIn('explicit_gui_handoff(', service)
        self.assertNotIn('task_controller_handoff(', service)
        self.assertIn('EndpointTrimReleaseStatus::PENDING', service)
        self.assertIn('EndpointTrimReleaseStatus::COMPLETED', service)
        self.assertIn('EndpointTrimReleaseStatus::REJECTED', service)
        self.assertIn('release pending serialized encoder response', service)
        self.assertIn('release completed; serialized target install pending', service)
        self.assertIn('release rejected:', service)
        self.assertIn('no release handoff required', service)

    def test_terminal_release_event_is_latched_before_same_tick_feedback(self):
        source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
        timer = _function_body(
            source,
            'void AliciaDDriverNode::send_command_timer_callback',
        )

        record = timer.index('endpoint_trim_command_order_.record_release(')
        feedback = timer.index('endpoint_trim_continuity_.note_feedback(')
        consume = timer.index(
            'endpoint_trim_command_order_.consume_terminal_release_code()'
        )
        self.assertLess(record, feedback)
        self.assertLess(feedback, consume)
        self.assertIn('endpoint_trim_terminal_release_code', timer)

    def test_only_repeated_user_enable_may_preserve_fresh_confirmation(self):
        source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
        enable = _function_body(
            source, 'bool AliciaDDriverNode::request_positive_enable',
        )
        self.assertIn('preserve_fresh_confirmation &&', enable)
        self.assertLess(
            enable.index('can_preserve_positive_enable('),
            enable.index('clear_retained_command_state();'),
        )
        self.assertLess(
            enable.index('if (sustained_temperature_protection)'),
            enable.index('can_preserve_positive_enable('),
        )
        self.assertIn('request_positive_enable("startup");', source)
        self.assertIn('request_positive_enable("serial_reconnect");', source)
        self.assertIn('request_positive_enable("demonstration_false", true);', source)
        self.assertIn('bool preserve_fresh_confirmation = false', DRIVER_HEADER.read_text())

    def test_retained_command_clear_resets_coordinator_at_every_call_site(self):
        source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
        clear = _function_body(
            source,
            'void AliciaDDriverNode::clear_retained_command_state',
        )
        self.assertIn(
            'endpoint_trim_continuity_ =\n'
            '            EndpointTrimContinuity(endpoint_trim_config_);',
            clear,
        )
        self.assertIn('endpoint_trim_command_order_.reset();', clear)

        callers = (
            'bool AliciaDDriverNode::request_positive_enable',
            'void AliciaDDriverNode::reconnect_callback',
            'void AliciaDDriverNode::parse_sdk_joint_state_frame',
            'void AliciaDDriverNode::demonstration_mode_callback',
        )
        for caller in callers:
            with self.subTest(caller=caller):
                self.assertIn(
                    'clear_retained_command_state();',
                    _function_body(source, caller),
                )

    def test_endpoint_trim_correction_is_after_every_transmission_gate(self):
        source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
        timer = _function_body(
            source,
            'void AliciaDDriverNode::send_command_timer_callback',
        )
        correction = timer.index('endpoint_trim_continuity_.request_correction(')

        self.assertIn('EndpointTrimTransmissionGate endpoint_trim_gate', timer)
        self.assertIn('endpoint_trim_gate.allows_correction()', timer)
        self.assertLess(timer.index('motion_enabled = motion_commands_enabled_;'), correction)
        self.assertLess(timer.index('ActuationState::OVERHEAT_BLOCKED'), correction)
        self.assertLess(timer.index('protection_latched = protection_fault_latched_;'), correction)
        self.assertLess(timer.index('feedback_stale ='), correction)
        self.assertIn(
            'endpoint_trim_gate.allows_correction() &&\n'
            '            endpoint_trim_decision.phase == EndpointTrimPhase::ACTIVE_READY',
            timer,
        )

    def test_endpoint_trim_continuity_parameters_are_bounded_and_forwarded(self):
        source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
        header = DRIVER_HEADER.read_text()
        driver_launch = (
            DRIVER_SRC.parent / 'launch' / 'alicia_d_driver.launch'
        ).read_text()
        bringup_launch = (
            DRIVER_SRC.parent / 'launch' / 'alicia_d_bringup.launch'
        ).read_text()

        for member in (
            'endpoint_feedback_trim_max_step_quantums_',
            'endpoint_feedback_trim_response_min_quantums_',
            'endpoint_feedback_trim_response_deadline_sec_',
        ):
            self.assertIn(member, header)
            self.assertIn(member, source)

        self.assertIn('endpoint_trim_parameters_valid', source)
        self.assertIn('endpoint_feedback_trim_max_step_quantums_ >= 1', source)
        self.assertIn('endpoint_feedback_trim_max_step_quantums_ <= 16', source)
        self.assertIn('endpoint_feedback_trim_response_min_quantums_ >= 1', source)
        self.assertIn(
            'endpoint_feedback_trim_response_min_quantums_ <=\n'
            '            endpoint_feedback_trim_max_step_quantums_',
            source,
        )
        self.assertIn('endpoint_feedback_trim_response_deadline_sec_ >= 0.30', source)
        self.assertIn('endpoint_feedback_trim_response_deadline_sec_ <= 3.0', source)

        launch_defaults = {
            'endpoint_feedback_trim_max_step_quantums': '4',
            'endpoint_feedback_trim_response_min_quantums': '2',
            'endpoint_feedback_trim_response_deadline_sec': '1.0',
        }
        for launch in (driver_launch, bringup_launch):
            for name, default in launch_defaults.items():
                self.assertIn(
                    f'<arg name="{name}" default="{default}"/>',
                    launch,
                )
                self.assertIn(
                    f'<param name="{name}" value="$(arg {name})"/>',
                    launch,
                )

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
        self.assertNotIn(
            'streamed_command_age_sec <= feedback_stale_timeout_sec_',
            body,
        )

        acceptance = body.index('if (accept_joint_positions) {')
        feedback_refresh = body.index(
            'last_feedback_time_ = feedback_time;'
        )
        self.assertLess(acceptance, feedback_refresh)
        self.assertNotIn(
            'last_feedback_time_ = feedback_time;',
            body[:acceptance],
        )
        self.assertIn(
            'if (accept_joint_positions) {\n'
            '        publish_joint_state(true);',
            body,
        )

    def test_repeated_feedback_discontinuity_requires_command_consistent_recovery(self):
        source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
        body = _function_body(
            source,
            'void AliciaDDriverNode::parse_sdk_joint_state_frame',
        )

        self.assertIn('has_streamed_command_reference', body)
        self.assertIn('command_consistent_recovery', body)
        self.assertIn(
            'pending_joint_feedback_count_ >=\n'
            '                    feedback_jump_confirm_samples_ &&\n'
            '                command_consistent_recovery;',
            body,
        )
        self.assertIn(
            'Rejected repeated SDK joint feedback discontinuity without command-consistent recovery',
            body,
        )

    def test_encoder_not_ready_frame_clears_stale_feedback_bootstrap(self):
        source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
        body = _function_body(
            source,
            'void AliciaDDriverNode::parse_sdk_joint_state_frame',
        )

        not_ready_start = body.index('if (all_zero || all_full_scale) {')
        conversion_start = body.index(
            'std::vector<double> candidate_joint_positions',
            not_ready_start,
        )
        not_ready_branch = body[not_ready_start:conversion_start]

        self.assertIn('clear_retained_command_state();', not_ready_branch)
        self.assertIn(
            'actuation_confirmation_.mark_unconfirmed(\n'
            '                "ENCODER_FEEDBACK_NOT_READY",',
            not_ready_branch,
        )
        self.assertIn('publish_actuation_status();', not_ready_branch)
        self.assertLess(
            not_ready_branch.index('clear_retained_command_state();'),
            not_ready_branch.index('return;'),
        )
        self.assertNotIn('motion_commands_enabled_ = false', not_ready_branch)
        self.assertNotIn('torque_off', not_ready_branch)

    def test_feedback_gap_cannot_expand_jump_window_or_confirm_actuation(self):
        source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
        body = _function_body(
            source,
            'void AliciaDDriverNode::parse_sdk_joint_state_frame',
        )

        self.assertIn('maximum_jump_interval_sec', body)
        self.assertIn('bounded_frame_interval_sec', body)
        self.assertIn(
            'feedback_max_velocity_rad_s_ * bounded_frame_interval_sec;',
            body,
        )
        self.assertIn('accepted_joint_feedback_discontinuity', body)
        self.assertIn(
            'actuation_confirmation_.mark_unconfirmed(\n'
            '                    "DISCONTINUOUS_FEEDBACK_RECOVERY",',
            body,
        )
        discontinuity_reset = body.index(
            '"DISCONTINUOUS_FEEDBACK_RECOVERY"'
        )
        feedback_note = body.index(
            'actuation_confirmation_.note_feedback('
        )
        self.assertLess(discontinuity_reset, feedback_note)

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
