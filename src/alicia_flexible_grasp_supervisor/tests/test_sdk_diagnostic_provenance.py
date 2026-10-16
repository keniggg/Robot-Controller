"""Diagnostic paths must never become an actuation or fake-feedback path."""
from test_serial_driver_resilience import DRIVER_SRC, _function_body


def test_raw_evidence_precedes_filters_and_is_not_heartbeat_republished():
    source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
    dispatch = _function_body(source, 'void AliciaDDriverNode::process_serial_data()')
    assert dispatch.index('publish_sdk_raw_diagnostic(') < dispatch.index('switch (command_id)')
    heartbeat = _function_body(source, 'void AliciaDDriverNode::heartbeat_publish_callback')
    assert 'sdk_diagnostic_pub_' not in heartbeat
    assert 'device_info_pub_' not in heartbeat


def test_version_request_only_queues_and_never_sends_arbitrary_commands():
    source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
    body = _function_body(source, 'bool AliciaDDriverNode::query_device_info_callback')
    assert 'device_info_query_pending_.exchange(true)' in body
    assert 'NOT a device response' in body
    for forbidden in ('write_raw_frame', 'request_positive_enable', 'connect()',
                      'torque_off', 'motion_commands_enabled_', 'joint_command'):
        assert forbidden not in body
    polling = _function_body(source, 'void AliciaDDriverNode::state_poll_timer_callback')
    assert '!diagnostic_queries_suppressed && device_info_query_pending_.load()' in polling
    assert '(now - last_device_info_query_time_).toSec() >= 5.0' in polling
    assert 'write_raw_frame(sdk_readonly_version_query())' in polling


def test_raw_status_does_not_confirm_actuation_or_renew_joint_feedback():
    source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
    body = _function_body(source, 'void AliciaDDriverNode::publish_sdk_raw_diagnostic')
    for forbidden in ('note_feedback(', 'note_streamed_target(', 'write_raw_frame(',
                      'last_accepted_joint_feedback_time_', 'motion_commands_enabled_',
                      'current_joint_positions_', 'request_positive_enable('):
        assert forbidden not in body
    assert 'payload[14] == 0xE1 || payload[14] == 0xE2' in body
    assert 'host_parse_time_not_device_time' in body
    assert 'crc_valid_serial_response_not_motion_ack' in body


def test_raw_temperature_is_published_with_quality_without_changing_protection():
    source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
    body = _function_body(source, 'void AliciaDDriverNode::parse_sdk_temperature_frame')
    assert 'sdk_temperature_evidence(data_payload, msg.data, max_enable_temperature_c_)' in body
    assert 'diagnostic.header.stamp = temperature_time;' in body
    assert 'unknown' in body.lower()
    assert 'unverified_servo_order_not_joint_indices' in body
    assert 'write_raw_frame' not in body


def test_motion_diagnostic_request_only_queues_a_bounded_readonly_batch():
    source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
    body = _function_body(source, 'bool AliciaDDriverNode::query_motion_diagnostics_callback')
    assert 'readonly_motion_query_batch_.request(ros::SteadyTime::now().toSec())' in body
    assert 'NOT a device response' in body
    for forbidden in ('write_raw_frame', 'request_positive_enable', 'connect()',
                      'torque_off', 'motion_commands_enabled_', 'joint_command'):
        assert forbidden not in body
    # No startup or periodic request: service callback is the sole admission.
    assert source.count('readonly_motion_query_batch_.request(') == 1
    polling = _function_body(source, 'void AliciaDDriverNode::state_poll_timer_callback')
    assert 'ros::SteadyTime::now().toSec(), !diagnostic_queries_suppressed' in polling
    assert 'write_raw_frame(sdk_readonly_motion_query(readonly_query))' in polling
    assert 'publish_readonly_query_write(readonly_query, written)' in polling
    assert polling.index('if (temperature_due)') < polling.index('SdkReadonlyMotionQuery readonly_query')
    assert 'readonly_motion_query_batch_.cancel()' in polling


def test_velocity_and_self_check_remain_raw_diagnostics_not_joint_feedback():
    source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
    body = _function_body(source, 'void AliciaDDriverNode::publish_sdk_raw_diagnostic')
    assert 'sdk_decode_follower_velocity_words(payload, words)' in body
    assert 'sdk_decode_follower_self_check(payload, evidence)' in body
    assert 'raw_u16_not_joint_rad_per_second' in body
    assert 'unverified_servo_order_not_joint_indices' in body
    for forbidden in ('joint_state_pub_', 'accepted_joint_state_pub_', 'current_joint_positions_',
                      'velocity.push_back', 'hardware_value_to_rad', 'motion_commands_enabled_'):
        assert forbidden not in body


def test_readonly_tx_evidence_distinguishes_host_write_from_device_reply():
    source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
    body = _function_body(source, 'void AliciaDDriverNode::publish_readonly_query_write')
    assert 'sdk_serial_tx_readonly' in body
    assert 'NOT device ACK' in body
    assert 'host_write_time_not_device_time' in body
    assert 'no automatic retry' in body
    for forbidden in ('write_raw_frame', 'note_feedback', 'motion_commands_enabled_',
                      'last_accepted_joint_feedback_time_', 'sdk_command_pub_'):
        assert forbidden not in body
