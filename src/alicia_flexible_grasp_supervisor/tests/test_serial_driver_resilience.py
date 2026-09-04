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
        rejection = body.index('Rejected %zu implausible SDK temperature')
        telemetry_refresh = body.index('latest_temperatures_c_ = msg.data;')
        self.assertLess(rejection, telemetry_refresh)
        self.assertIn(
            'if (std::isfinite(value))',
            body,
        )
        self.assertNotIn('return;', body[rejection:telemetry_refresh])
        self.assertIn(
            'consecutive_high_temperature_samples_by_channel_[i]',
            body[telemetry_refresh:],
        )

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
        self.assertIn(
            'msg->header.frame_id == "gui_direct" ||',
            callback,
        )

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
        self.assertIn('EXPLICIT_GUI', admission)
        self.assertIn('authoritative_source_', admission)
        self.assertIn('last_observation_accepted_', admission)
        self.assertIn(
            'newer_explicit_gui_command_requires_handoff',
            admission,
        )
        self.assertIn('newer_task_command_requires_handoff', admission)
        self.assertIn('task_controller_handoff_permitted', admission)
        self.assertIn(
            '!task_controller_handoff_permitted',
            admission,
        )

    def test_task_authority_resumes_at_the_existing_gui_gesture_boundary(self):
        source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
        callback = _function_body(
            source,
            'void AliciaDDriverNode::joint_command_callback',
        )

        # The pre-existing direct-GUI gesture owns the short timeout window.
        # Once that guard has allowed an untagged command through, it is
        # admitted as a TASK_CONTROLLER handoff rather than a GUI handoff.
        self.assertIn('gui_direct_gesture_timeout_sec_', callback)
        self.assertIn(
            'Ignored %s /joint_commands source during active gui_direct gesture',
            callback,
        )
        observe = callback.index(
            'endpoint_trim_command_order_.observe_upstream_command('
        )
        handoff = callback.index(
            'endpoint_trim_continuity_.explicit_gui_handoff('
        )
        self.assertLess(observe, handoff)
        self.assertIn(
            '!endpoint_trim_explicit_gui_command',
            callback[observe:handoff],
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
            '        publish_joint_state();',
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
