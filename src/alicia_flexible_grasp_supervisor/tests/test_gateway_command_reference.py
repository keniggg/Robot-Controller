"""Offline regression for feedback-rebase ratchet and reference admission."""
import importlib.util
from copy import deepcopy
import math
from pathlib import Path
import threading
from types import SimpleNamespace as NS

import pytest
import rospy
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, String

SPEC = importlib.util.spec_from_file_location(
    'gateway_reference_test_module', Path(__file__).resolve().parents[1] /
    'scripts/motion_gateway_node.py')
module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(module)
Gateway = module.MotionGateway
COUNT = 2 * math.pi / 4096
NAMES = ['Joint%d' % i for i in range(1, 7)]


def packet(frame, values, stamp=10., names=None):
    result = JointState()
    result.header.stamp = rospy.Time.from_sec(stamp)
    result.header.frame_id = frame
    result.name = list(names or (NAMES + ['right_finger'] if len(values) == 7 else NAMES))
    result.position = list(values)
    return result


@pytest.fixture
def node(monkeypatch):
    gateway = Gateway.__new__(Gateway)
    gateway.joint_names = NAMES + ['right_finger']
    gateway._command_reference_lock = threading.RLock()
    gateway._driver_reference_sub = object()
    gateway._gui_direct_mode = False
    gateway._actuation_status = 'CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE'
    gateway._actuation_received_sec = 10.
    gateway._positive_enable_reference_epoch_sec = None
    gateway._control_reference_epoch_ns = 9000000000
    # Recorded 2026-09-11 residual: actual differs from the successful goal.
    desired = [COUNT * n for n in (-1251, -84, 135, -39, -153, 47)]
    actual = list(desired)
    actual[1] -= 8 * COUNT
    actual[2] -= 11 * COUNT
    gateway._sdk_command_feedback = packet('sdk_transmitted', desired)
    gateway._driver_control_reference = packet('sdk_tracking', desired + [.05])
    gateway._observation_joint_feedback = packet('sdk_measured', actual + [.04975])
    state = JointTrajectoryControllerState()
    state.header.stamp = rospy.Time.from_sec(10.)
    state.joint_names = NAMES
    state.desired.positions = list(desired)
    state.desired.velocities = [0.] * 6
    state.desired.accelerations = [0.] * 6
    state.actual.positions = list(actual)
    gateway._trajectory_controller_state = state
    gateway._completed_command_reference = {
        'names': tuple(NAMES), 'positions': tuple(desired),
        'end_sec': 9.9, 'kind': 'completed_gateway_trajectory'}
    gateway.joint_cmd = NS(last_state_time_sec=10., last_positions=actual + [.04975])
    gateway.sent = []
    gateway.trajectory_command_pub = NS(publish=gateway.sent.append)
    gateway.gripper = NS(gripper_index=6, set_position=lambda v, arm_positions:
                         gateway.sent.append((v, arm_positions)))
    gateway._test_params = {}
    monkeypatch.setattr(module.rospy, 'get_time', lambda: 10.1)
    monkeypatch.setattr(module.rospy, 'get_param',
                        lambda k, default=None: gateway._test_params.get(k, default))
    return gateway


def test_recorded_residual_does_not_generate_a_second_feedback_hold(node):
    actual_before = list(node.joint_cmd.last_positions)
    ok, reason = node._synchronize_trajectory_controller_to_feedback()
    assert ok and 'preserved' in reason
    assert node.sent == []
    assert node.joint_cmd.last_positions == actual_before


def test_multiple_automatic_stages_and_multiple_joint_changes_remain_allowed(node):
    for _ in range(4):
        assert node._synchronize_trajectory_controller_to_feedback()[0]
        for i in (0, 2, 4):
            node._sdk_command_feedback.position[i] += 3 * COUNT
            node._driver_control_reference.position[i] += 3 * COUNT
            node._trajectory_controller_state.desired.positions[i] += 3 * COUNT
        node._completed_command_reference['positions'] = tuple(node._trajectory_controller_state.desired.positions)
    assert node.sent == []


def test_provider_preserves_actual_controller_desired_not_measured_or_rounded(node):
    node._trajectory_controller_state.desired.positions[0] += COUNT * .2
    node._completed_command_reference['positions'] = tuple(node._trajectory_controller_state.desired.positions)
    desired = node._controller_reference_for_execution(list(reversed(NAMES)))
    assert desired.positions == list(reversed(node._trajectory_controller_state.desired.positions))
    assert desired.positions != list(reversed(node._sdk_command_feedback.position))


@pytest.mark.parametrize('source', ['sdk', 'reference', 'accepted', 'controller'])
def test_stale_sources_reject_before_any_publish(node, source):
    item = {'sdk': node._sdk_command_feedback, 'reference': node._driver_control_reference,
            'accepted': node._observation_joint_feedback,
            'controller': node._trajectory_controller_state}[source]
    item.header.stamp = rospy.Time.from_sec(9.)
    ok, reason = node._synchronize_trajectory_controller_to_feedback()
    assert not ok and 'CONTROLLER_REFERENCE_INVALID' in reason
    assert node.sent == []


@pytest.mark.parametrize('bad', ['word', 'names', 'future', 'mode', 'moving', 'missing', 'manual'])
def test_invalid_reference_cannot_fall_back_to_feedback(node, bad):
    if bad == 'word': node._trajectory_controller_state.desired.positions[1] += COUNT
    if bad == 'names': node._sdk_command_feedback.name[1] = 'Joint1'
    if bad == 'future':
        node._sdk_command_feedback.header.stamp = rospy.Time.from_sec(10.2)
        node._driver_control_reference.header.stamp = rospy.Time.from_sec(10.2)
    if bad == 'mode': node._driver_control_reference.header.frame_id = 'sdk_reference_unconfirmed'
    if bad == 'moving': node._trajectory_controller_state.desired.velocities[1] = .001
    if bad == 'missing':
        node._sdk_command_feedback = None
        node._driver_control_reference = None
    if bad == 'manual': node._gui_direct_mode = True
    ok, reason = node._synchronize_trajectory_controller_to_feedback()
    assert not ok and 'CONTROLLER_REFERENCE_INVALID' in reason
    assert node.sent == []


def test_explicit_driver_handoff_selects_frozen_wire_never_actual(node):
    node._driver_control_reference.header.frame_id = 'sdk_handoff_required'
    node._trajectory_controller_state.desired.positions = [0.] * 6
    positions, mode = node._select_controller_sync_reference()
    assert mode == 'frozen_sdk_handoff'
    assert positions == node._sdk_command_feedback.position
    assert positions != node.joint_cmd.last_positions[:-1]
    assert node.sent == []


def test_handoff_refuses_hardware_stream_that_could_overwrite_frozen_gripper(node):
    node._driver_control_reference.header.frame_id = 'sdk_handoff_required'
    node._test_params['/bessica_d_hw_interface/publish_gripper_command'] = True
    assert not node._synchronize_trajectory_controller_to_feedback()[0]
    assert node.sent == []


def test_only_initial_positive_enable_without_any_wire_reference_selects_actual(node):
    node._sdk_command_feedback = node._driver_control_reference = None
    node._actuation_status = 'PENDING:POSITIVE_ENABLE_REQUESTED'
    positions, mode = node._select_controller_sync_reference()
    assert mode == 'initial_positive_enable'
    assert positions == node.joint_cmd.last_positions[:-1]
    node._actuation_status = 'PENDING:AWAITING_ENCODER_RESPONSE'
    with pytest.raises(module.ObservationPathError): node._select_controller_sync_reference()


def test_repeated_status_heartbeat_does_not_invent_a_new_command_epoch(node):
    node._actuation_status = 'PENDING:POSITIVE_ENABLE_REQUESTED'
    node._actuation_cb(String(data='PENDING:POSITIVE_ENABLE_REQUESTED'))
    assert node._sdk_command_feedback is not None
    assert node._control_reference_epoch_ns == 9000000000


def test_new_driver_reset_epoch_revokes_prior_reference_even_with_same_status(node):
    node._actuation_status = 'PENDING:POSITIVE_ENABLE_REQUESTED'
    node._control_reference_epoch_cb(Header(
        stamp=rospy.Time.from_sec(10.05), frame_id='sdk_reference_epoch'))
    assert node._sdk_command_feedback is None and node._driver_control_reference is None
    assert node._completed_command_reference is None
    assert node._positive_enable_reference_epoch_sec == pytest.approx(10.05)


def test_old_or_repeated_driver_epoch_cannot_revoke_a_new_reference(node):
    for stamp in (8., 9.):
        node._control_reference_epoch_cb(Header(
            stamp=rospy.Time.from_sec(stamp), frame_id='sdk_reference_epoch'))
    assert node._sdk_command_feedback is not None
    assert node._completed_command_reference is not None


def test_missing_driver_epoch_cannot_initialize_or_continue(node):
    node._control_reference_epoch_ns = None
    assert not node._synchronize_trajectory_controller_to_feedback()[0]
    node._sdk_command_feedback = node._driver_control_reference = None
    node._actuation_status = 'PENDING:POSITIVE_ENABLE_REQUESTED'
    assert not node._synchronize_trajectory_controller_to_feedback()[0]
    assert node.sent == []


def test_future_reset_revokes_old_reference_without_authorizing_initialization(node):
    node._control_reference_epoch_cb(Header(
        stamp=rospy.Time.from_sec(11.), frame_id='sdk_reference_epoch'))
    assert node._sdk_command_feedback is None
    node._actuation_status = 'PENDING:POSITIVE_ENABLE_REQUESTED'
    assert not node._synchronize_trajectory_controller_to_feedback()[0]
    assert node.sent == []


@pytest.mark.parametrize('change', ['epoch', 'manual'])
def test_admission_rechecks_ownership_after_paired_topic_wait(node, monkeypatch, change):
    original = node._command_reference_snapshot
    epoch = node._control_reference_epoch_ns

    def callback_during_snapshot():
        result = original()
        if change == 'manual':
            node._gui_direct_mode = True
        else:
            node._control_reference_epoch_cb(Header(
                stamp=rospy.Time.from_sec(10.05), frame_id='sdk_reference_epoch'))
        return result

    monkeypatch.setattr(node, '_command_reference_snapshot', callback_during_snapshot)
    with pytest.raises(module.ObservationPathError, match='ownership or epoch changed'):
        node._controller_reference_admission_state(
            NAMES, list(node._sdk_command_feedback.position), 'frozen_sdk_handoff', epoch, True)
    assert node.sent == []


def test_gripper_commands_keep_successful_arm_baseline_despite_feedback_residual(node):
    desired = list(node._sdk_command_feedback.position)
    for opening in (.04, .03, .02):
        response = node.handle_gripper(NS(value=opening))
        assert response.success
        node.joint_cmd.last_positions[1] -= COUNT
    assert [command[1] for command in node.sent] == [desired] * 3


def test_gripper_cannot_close_during_unacknowledged_handoff(node):
    node._driver_control_reference.header.frame_id = 'sdk_handoff_required'
    assert not node.handle_gripper(NS(value=.02)).success
    assert node.sent == []


def test_sdk_codec_round_trip_all_words():
    for word in range(4096):
        q = (-180. + word / 4096. * 360.) * math.pi / 180.
        assert Gateway._sdk_arm_words([q]) == (word,)


def test_zero_velocity_snapshot_alone_cannot_claim_command_completion(node):
    node._completed_command_reference = None
    with pytest.raises(module.ObservationPathError, match='completed gateway'):
        node._controller_reference_for_execution(NAMES)
    # Selection may propose a NEW, zero-displacement acknowledged transaction,
    # but must not claim that the unowned prior trajectory completed.
    _, mode = node._select_controller_sync_reference()
    assert mode == 'stationary_sdk_reacquisition'
    assert node._completed_command_reference is None
    assert node.sent == []


def test_old_topic_packets_cannot_replace_newer_reference(node):
    old_sdk = packet('sdk_transmitted', [0.] * 6, stamp=9.9)
    old_reference = packet('sdk_handoff_required', [0.] * 6 + [.05], stamp=9.9)
    node._sdk_command_feedback_cb(old_sdk)
    node._driver_control_reference_cb(old_reference)
    assert node._driver_control_reference.header.frame_id == 'sdk_tracking'
    assert node._synchronize_trajectory_controller_to_feedback()[0]


def test_completion_callback_before_controller_frame_waits_without_publishing(node, monkeypatch):
    node._completed_command_reference['end_sec'] = 10.05
    completed = deepcopy(node._completed_command_reference)
    sleeps = []

    def next_frame(duration):
        sleeps.append(duration)
        node._trajectory_controller_state.header.stamp = rospy.Time.from_sec(10.1)

    monkeypatch.setattr(module.time, 'sleep', next_frame)
    desired = node._controller_reference_for_execution(NAMES)
    assert desired.controller_stamp_sec == pytest.approx(10.1, abs=1e-8)
    assert sleeps and max(sleeps) <= .002
    assert node._completed_command_reference == completed
    assert node.sent == []


def test_old_snapshot_cannot_borrow_new_live_controller_timestamp(node, monkeypatch):
    node._completed_command_reference['end_sec'] = 10.05
    original = node._stationary_controller_reference

    def callback_after_snapshot(names):
        snapshot = original(names)
        node._trajectory_controller_state = deepcopy(node._trajectory_controller_state)
        node._trajectory_controller_state.header.stamp = rospy.Time.from_sec(10.1)
        node._trajectory_controller_state.desired.positions[0] += .01
        return snapshot

    monkeypatch.setattr(node, '_stationary_controller_reference', callback_after_snapshot)
    monkeypatch.setattr(module.time, 'sleep', lambda duration: None)
    with pytest.raises(module.ObservationPathError, match='differs from successful SDK'):
        node._controller_reference_for_execution(NAMES)
    assert node.sent == []


def test_post_completion_frame_wait_has_monotonic_deadline(node, monkeypatch):
    node._completed_command_reference['end_sec'] = 10.05
    elapsed = [0.]
    monkeypatch.setattr(module.time, 'monotonic', lambda: elapsed[0])
    monkeypatch.setattr(module.time, 'sleep', lambda duration: elapsed.__setitem__(0, elapsed[0] + duration))
    with pytest.raises(module.ObservationPathError, match='snapshot timed out'):
        node._controller_reference_for_execution(NAMES)
    assert .1 <= elapsed[0] <= .100001
    assert node.sent == []


@pytest.mark.parametrize('change', ['manual', 'completion', 'stale', 'endpoint'])
def test_post_completion_wait_revalidates_all_evidence(node, monkeypatch, change):
    node._completed_command_reference['end_sec'] = 10.05

    def next_frame(duration):
        node._trajectory_controller_state.header.stamp = rospy.Time.from_sec(10.1)
        if change == 'manual': node._gui_direct_mode = True
        if change == 'completion': node._completed_command_reference = None
        if change == 'stale': node._observation_joint_feedback.header.stamp = rospy.Time.from_sec(9.)
        if change == 'endpoint': node._trajectory_controller_state.desired.positions[0] += COUNT * .2

    monkeypatch.setattr(module.time, 'sleep', next_frame)
    with pytest.raises(module.ObservationPathError):
        node._controller_reference_for_execution(NAMES)
    assert node.sent == []
