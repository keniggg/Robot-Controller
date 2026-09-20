"""Offline fake-clock initial admission and frozen-SDK handoff acknowledgments.

No ROS publishers/services are created: both outgoing channels are recorders.
Fresh feedback advances independently from the one selected command reference.
"""
from copy import deepcopy
from types import SimpleNamespace as NS

import pytest
import rospy
from std_msgs.msg import Header, String

from test_gateway_command_reference import COUNT, NAMES, module, node, packet


class HandoffClock:
    def __init__(self, gateway, monkeypatch, mode):
        self.gateway = gateway
        self.start = 10.1
        self.elapsed = 0.
        self.steps = 0
        self.mode = mode
        self.holds = []
        self.explicit = []
        self.on_tick = lambda _clock: None
        self.on_explicit = lambda _clock, _message: None
        self.ack_stamp = None
        self.stale_source = None
        self.gateway._completed_command_reference = None
        self.gateway._control_reference_epoch_ns = 10_000_000_000
        if mode in ('handoff', 'reacquire'):
            gateway._driver_control_reference.header.frame_id = (
                'sdk_handoff_required' if mode == 'handoff' else 'sdk_tracking')
            self.selected = tuple(gateway._sdk_command_feedback.position)
        elif mode == 'initial':
            gateway._sdk_command_feedback = None
            gateway._driver_control_reference = None
            gateway._actuation_status = 'PENDING:POSITIVE_ENABLE_REQUESTED'
            gateway._positive_enable_reference_epoch_sec = 10.
            self.selected = tuple(gateway._observation_joint_feedback.position[:-1])
        else:
            raise AssertionError(mode)
        gateway._test_params['/robot'] = {
            'strict_execution_controller_sync_duration_sec': .06,
            'strict_execution_controller_sync_timeout_sec': .40,
            'strict_execution_controller_sync_bridge_duration_sec': .02,
            'strict_execution_controller_sync_max_feedback_age_sec': .5,
            'strict_execution_controller_sync_tolerance_rad': .03,
        }
        gateway.joint_cmd.pub = NS(publish=self.publish_explicit)
        gateway.trajectory_command_pub = NS(publish=self.publish_hold)
        monkeypatch.setattr(module.rospy, 'get_time', lambda: self.now)
        monkeypatch.setattr(module.rospy, 'sleep', self.sleep)
        monkeypatch.setattr(module.rospy, 'is_shutdown', lambda: False)
        monkeypatch.setattr(module.rospy.Time, 'now', staticmethod(
            lambda: rospy.Time.from_sec(self.now)))
        self.refresh()

    @property
    def now(self):
        return self.start + self.elapsed

    def refresh(self):
        gateway = self.gateway
        gateway.joint_cmd.last_state_time_sec = self.now
        gateway._actuation_received_sec = self.now
        gateway._trajectory_controller_state.header.stamp = rospy.Time.from_sec(
            self.now - 1. if self.stale_source == 'controller' else self.now)
        gateway._observation_joint_feedback.header.stamp = rospy.Time.from_sec(
            self.now - 1. if self.stale_source == 'accepted' else self.now)
        if gateway._sdk_command_feedback is not None:
            stamp = self.now if self.ack_stamp is None else self.ack_stamp
            if self.stale_source == 'sdk':
                stamp = self.now - 1.
            gateway._sdk_command_feedback.header.stamp = rospy.Time.from_sec(stamp)
            gateway._driver_control_reference.header.stamp = rospy.Time.from_sec(stamp)

    def sleep(self, seconds):
        self.elapsed = round(self.elapsed + float(seconds), 9)
        self.steps += 1
        assert self.steps <= 100, 'handoff must retain its original bounded deadline'
        # This is new measured feedback, not permission to reselect its q.
        # Keep it inside the existing hold tolerance throughout the fixture.
        self.gateway._observation_joint_feedback.position[4] += COUNT / 100.
        self.gateway.joint_cmd.last_positions = list(self.gateway._observation_joint_feedback.position)
        self.gateway._trajectory_controller_state.actual.positions = list(
            self.gateway._observation_joint_feedback.position[:-1])
        self.refresh()
        self.on_tick(self)

    def publish_hold(self, message):
        self.holds.append((self.now, deepcopy(message)))
        assert message.joint_names == NAMES
        assert list(message.points[-1].positions) == list(self.selected)
        self.gateway._trajectory_controller_state.desired.positions = list(self.selected)
        self.gateway._trajectory_controller_state.desired.velocities = [0.] * 6
        self.gateway._trajectory_controller_state.desired.accelerations = [0.] * 6
        self.refresh()

    def publish_explicit(self, message):
        self.explicit.append((self.now, deepcopy(message)))
        assert len(self.explicit) == 1, 'same selected command must be sent at most once'
        assert message.header.frame_id == 'motion_gateway_reference_sync'
        assert message.header.stamp.to_sec() == pytest.approx(self.now, abs=1e-8)
        assert message.name == NAMES and len(message.position) == 6
        assert list(message.position) == list(self.selected)
        assert 'right_finger' not in message.name
        assert self.gateway._manual_control_active() is False
        self.on_explicit(self, message)

    def acknowledge(self, *, stamp=None, wrong_word=False):
        values = list(self.selected)
        if wrong_word:
            values[1] += COUNT
        self.ack_stamp = self.now if stamp is None else float(stamp)
        self.gateway._sdk_command_feedback = packet('sdk_transmitted', values, self.ack_stamp)
        self.gateway._driver_control_reference = packet('sdk_tracking', values + [.05], self.ack_stamp)

    def change_epoch(self):
        message = Header()
        message.frame_id = 'sdk_reference_epoch'
        message.stamp = rospy.Time.from_sec(self.now)
        self.gateway._control_reference_epoch_cb(message)


@pytest.mark.parametrize('mode', ['handoff', 'initial'])
def test_natural_ack_before_settle_needs_no_explicit_sdk_resend(node, monkeypatch, mode):
    clock = HandoffClock(node, monkeypatch, mode)
    def tick(current):
        if current.elapsed >= .04 and node._driver_control_reference is None:
            current.acknowledge()
        elif current.elapsed >= .04 and node._driver_control_reference.header.frame_id != 'sdk_tracking':
            current.acknowledge()
    clock.on_tick = tick
    ok, reason = node._synchronize_trajectory_controller_to_feedback()
    assert ok, reason
    assert len(clock.holds) == 1 and not clock.explicit
    assert node._completed_command_reference['positions'] == clock.selected
    assert clock.elapsed <= .40


@pytest.mark.parametrize('mode', ['handoff', 'initial'])
def test_same_word_swallowed_hardware_command_gets_one_arm_only_resend_then_delayed_ack(node, monkeypatch, mode):
    clock = HandoffClock(node, monkeypatch, mode)
    def tick(current):
        if current.explicit and current.now >= current.explicit[0][0] + .04 - 1e-9:
            current.acknowledge()
    clock.on_tick = tick
    ok, reason = node._synchronize_trajectory_controller_to_feedback()
    assert ok, reason
    assert len(clock.holds) == 1 and len(clock.explicit) == 1
    assert clock.ack_stamp >= clock.explicit[0][0]
    assert node._completed_command_reference['positions'] == clock.selected
    assert tuple(node.joint_cmd.last_positions[:-1]) != clock.selected
    assert clock.elapsed <= .40


@pytest.mark.parametrize('mode', ['handoff', 'initial'])
def test_pending_ack_waits_only_original_deadline_and_never_republishes_or_rebases(node, monkeypatch, mode):
    clock = HandoffClock(node, monkeypatch, mode)
    ok, reason = node._synchronize_trajectory_controller_to_feedback()
    assert not ok, reason
    assert .40 <= clock.elapsed <= .42 + 1e-9
    assert len(clock.holds) == 1 and len(clock.explicit) == 1
    assert node._completed_command_reference is None


@pytest.mark.parametrize('mode', ['handoff', 'initial'])
def test_ack_older_than_explicit_resend_cannot_authorize_completion(node, monkeypatch, mode):
    clock = HandoffClock(node, monkeypatch, mode)
    clock.on_explicit = lambda current, _message: current.acknowledge(stamp=current.now - .01)
    ok, reason = node._synchronize_trajectory_controller_to_feedback()
    assert not ok, reason
    assert len(clock.explicit) == 1
    assert clock.ack_stamp < clock.explicit[0][0]
    assert node._completed_command_reference is None


@pytest.mark.parametrize('mode', ['handoff', 'initial'])
@pytest.mark.parametrize('failure', ['manual', 'epoch', 'accepted', 'controller', 'wrong_word', 'moving'])
def test_changed_or_stale_waiting_evidence_rejects_without_explicit_command(node, monkeypatch, mode, failure):
    clock = HandoffClock(node, monkeypatch, mode)
    changed_at = []
    def tick(current):
        if changed_at or current.elapsed < .04:
            return
        changed_at.append(current.now)
        if failure == 'manual':
            node._gui_direct_mode = True
        elif failure == 'epoch':
            current.change_epoch()
        elif failure in ('accepted', 'controller'):
            current.stale_source = failure
            current.refresh()
        elif failure == 'moving':
            node._trajectory_controller_state.desired.velocities[0] = .001
        else:
            current.acknowledge(wrong_word=True)
    clock.on_tick = tick
    ok, reason = node._synchronize_trajectory_controller_to_feedback()
    assert not ok, reason
    assert not clock.explicit
    assert len(clock.holds) == 1 and node._completed_command_reference is None
    assert clock.now - changed_at[0] <= .04 + 1e-9


@pytest.mark.parametrize('mode', ['handoff', 'initial'])
def test_mismatched_tracking_ack_after_resend_fails_without_second_command(node, monkeypatch, mode):
    clock = HandoffClock(node, monkeypatch, mode)
    clock.on_explicit = lambda current, _message: current.acknowledge(wrong_word=True)
    ok, reason = node._synchronize_trajectory_controller_to_feedback()
    assert not ok, reason
    assert len(clock.holds) == 1 and len(clock.explicit) == 1
    assert node._completed_command_reference is None
    assert clock.now - clock.explicit[0][0] <= .04 + 1e-9


@pytest.mark.parametrize('mode', ['handoff', 'initial'])
@pytest.mark.parametrize('failure', ['manual', 'epoch', 'accepted'])
def test_evidence_change_after_explicit_publish_still_revokes_admission(node, monkeypatch, mode, failure):
    clock = HandoffClock(node, monkeypatch, mode)
    changed_at = []
    def tick(current):
        if not current.explicit or changed_at:
            return
        changed_at.append(current.now)
        if failure == 'manual':
            node._gui_direct_mode = True
        elif failure == 'epoch':
            current.change_epoch()
        else:
            current.stale_source = 'accepted'
            current.refresh()
    clock.on_tick = tick
    ok, reason = node._synchronize_trajectory_controller_to_feedback()
    assert not ok, reason
    assert len(clock.holds) == 1 and len(clock.explicit) == 1
    assert node._completed_command_reference is None
    assert clock.now - changed_at[0] <= .04 + 1e-9


@pytest.mark.parametrize('mode', ['handoff', 'initial'])
def test_repeated_pending_heartbeats_do_not_create_new_epoch_or_destroy_ack(node, monkeypatch, mode):
    clock = HandoffClock(node, monkeypatch, mode)
    epoch = node._control_reference_epoch_ns
    pending = String(data='PENDING:POSITIVE_ENABLE_REQUESTED')
    def tick(current):
        # Driver publishes this status repeatedly on accepted feedback. It
        # is not a second enable command or a new hardware reference epoch.
        node._actuation_cb(pending)
        assert node._control_reference_epoch_ns == epoch
        if current.explicit and current.now >= current.explicit[0][0] + .04 - 1e-9:
            current.acknowledge()
            node._actuation_cb(pending)
            assert node._sdk_command_feedback is not None
            assert node._driver_control_reference.header.frame_id == 'sdk_tracking'
    clock.on_tick = tick
    ok, reason = node._synchronize_trajectory_controller_to_feedback()
    assert ok, reason
    assert len(clock.holds) == 1 and len(clock.explicit) == 1
    assert node._completed_command_reference['positions'] == clock.selected
    assert node._control_reference_epoch_ns == epoch


@pytest.mark.parametrize('mode', ['handoff', 'initial'])
def test_controller_old_pre_bridge_frame_waits_for_delayed_new_frame_without_rejection(node, monkeypatch, mode):
    clock = HandoffClock(node, monkeypatch, mode)
    old_state = deepcopy(node._trajectory_controller_state)
    old_state.desired.positions[0] += 2 * COUNT
    original_publish = clock.publish_hold
    def publish(message):
        original_publish(message)
        node._trajectory_controller_state = deepcopy(old_state)
    node.trajectory_command_pub.publish = publish
    def tick(current):
        if current.elapsed < .04:
            # New hold reaches ros_control after 20ms, but its first state
            # callback reaches the gateway only at 40ms. The old message is
            # still fresh; its different q cannot represent a failed hold.
            node._trajectory_controller_state = deepcopy(old_state)
        else:
            state = node._trajectory_controller_state
            state.header.stamp = rospy.Time.from_sec(current.now)
            state.desired.positions = list(current.selected)
            state.desired.velocities = [0.] * 6
            state.desired.accelerations = [0.] * 6
            current.acknowledge()
    clock.on_tick = tick
    ok, reason = node._synchronize_trajectory_controller_to_feedback()
    assert ok, reason
    assert clock.elapsed >= .04
    assert len(clock.holds) == 1 and not clock.explicit
    assert node._completed_command_reference['positions'] == clock.selected


@pytest.mark.parametrize('outcome', ['delayed_completion', 'never_consumed',
                                    'different_stationary_target', 'gate_released'])
def test_newly_stamped_old_hold_waits_only_while_original_sdk_gate_is_pending(
        node, monkeypatch, outcome):
    clock = HandoffClock(node, monkeypatch, 'handoff')
    origin = list(clock.selected)
    origin[1] -= .57
    node._trajectory_controller_state.desired.positions = list(origin)
    original_publish = clock.publish_hold
    def publish(message):
        original_publish(message)
        node._trajectory_controller_state.desired.positions = list(origin)
    node.trajectory_command_pub.publish = publish
    def tick(current):
        # Unlike a stale callback, this is a new source-stamped controller
        # frame still holding the original reference before command receipt.
        state = node._trajectory_controller_state
        state.desired.positions = list(origin)
        if current.elapsed >= .08:
            if outcome == 'delayed_completion':
                state.desired.positions = list(current.selected)
                current.acknowledge()
            elif outcome == 'different_stationary_target':
                state.desired.positions[0] += COUNT
            elif outcome == 'gate_released':
                current.acknowledge()
    clock.on_tick = tick
    ok, reason = node._synchronize_trajectory_controller_to_feedback()
    assert len(clock.holds) == 1 and not clock.explicit
    if outcome == 'delayed_completion':
        assert ok, reason
        assert clock.elapsed >= .08
        assert node._completed_command_reference['positions'] == clock.selected
    else:
        assert not ok
        assert node._completed_command_reference is None
        if outcome == 'never_consumed':
            assert 'timed out' in reason
            assert .40 <= clock.elapsed <= .42 + 1e-9
        else:
            assert 'has not reached' in reason
            assert clock.elapsed <= .10 + 1e-9


def test_ordinary_completed_tracking_preserves_both_channels_without_any_publish(node):
    explicit = []
    node.joint_cmd.pub = NS(publish=explicit.append)
    ok, reason = node._synchronize_trajectory_controller_to_feedback()
    assert ok and 'preserved' in reason
    assert not explicit and not node.sent


@pytest.mark.parametrize('delay', [.02, .06, .12])
def test_frozen_handoff_waits_for_its_delayed_interpolation_not_publish_clock(node, monkeypatch, delay):
    clock = HandoffClock(node, monkeypatch, 'handoff')
    origin = list(clock.selected)
    origin[1] -= .57
    origin[2] += .37
    node._trajectory_controller_state.desired.positions = list(origin)
    original_publish = clock.publish_hold
    def publish(message):
        original_publish(message)
        node._trajectory_controller_state.desired.positions = list(origin)
    node.trajectory_command_pub.publish = publish
    def tick(current):
        state = node._trajectory_controller_state
        if current.elapsed <= delay + 1e-9:
            # Real 19:37 frame: a valid bridge can start after publish+20ms.
            delta = [b-a for a, b in zip(origin, current.selected)]
            state.desired.positions = [a+.1*d for a, d in zip(origin, delta)]
            state.desired.velocities = [2.*d for d in delta]
            state.desired.accelerations = [100.*d for d in delta]
        else:
            state.desired.positions = list(current.selected)
            state.desired.velocities = [0.] * 6
            state.desired.accelerations = [0.] * 6
            current.acknowledge()
    clock.on_tick = tick
    ok, reason = node._synchronize_trajectory_controller_to_feedback()
    assert ok, reason
    assert delay < clock.elapsed <= .40
    assert len(clock.holds) == 1 and not clock.explicit
    assert node._completed_command_reference['positions'] == clock.selected


def test_interpolation_wait_does_not_accept_an_unrelated_controller_path(node, monkeypatch):
    clock = HandoffClock(node, monkeypatch, 'handoff')
    origin = list(clock.selected)
    origin[1] -= .57
    node._trajectory_controller_state.desired.positions = origin
    def tick(current):
        node._trajectory_controller_state.desired.positions[0] += .01
        node._trajectory_controller_state.desired.velocities[0] = .1
    clock.on_tick = tick
    ok, reason = node._synchronize_trajectory_controller_to_feedback()
    assert not ok and 'not stationary' in reason
    assert len(clock.holds) == 1 and not clock.explicit


def test_real_completed_handoff_roundoff_is_not_an_infinite_transition():
    # 22:49:38.3926368 controller frame from probe_exact_1.bag. SDK ACK
    # arrived, but floating-point interpolation ended one ULP before qSDK.
    origin = [-1.8205083195724625, -.1758752247817451, .08891763908254686,
              -.4372861944387775, .14230186959932573, .4687821468625767]
    endpoint = [-1.8668546188568254, .48013598660820567, -.3957670432744954,
                .0030679615757712823, .2500388684253595, .006135923151542565]
    rounded = [-1.8668546188568258, .48013598660820556, -.39576704327449563,
               .003067961575771494, .2500388684253595, .006135923151542655]
    desired = NS(positions=rounded, velocities=[0.]*6, accelerations=[0.]*6)
    assert not module.MotionGateway._is_own_frozen_reference_transition(desired, origin, endpoint)


@pytest.mark.parametrize('direction', [-1., 1.])
def test_stationary_endpoint_roundoff_completes_only_with_real_sdk_ack(node, monkeypatch, direction):
    clock = HandoffClock(node, monkeypatch, 'handoff')
    origin = list(clock.selected)
    origin[1] -= direction*.57
    node._trajectory_controller_state.desired.positions = list(origin)
    rounded = list(clock.selected)
    rounded[1] -= direction*1e-16
    def tick(current):
        state = node._trajectory_controller_state
        state.desired.positions = list(rounded)
        state.desired.velocities = [0.]*6
        state.desired.accelerations = [0.]*6
        current.acknowledge()
    clock.on_tick = tick
    ok, reason = node._synchronize_trajectory_controller_to_feedback()
    assert ok, reason
    assert len(clock.holds) == 1 and not clock.explicit
    assert node._completed_command_reference['positions'] == tuple(rounded)


@pytest.mark.parametrize('invalid', ['no_ack', 'wrong_word', 'velocity', 'acceleration'])
def test_stationary_roundoff_fix_does_not_admit_invalid_evidence(node, monkeypatch, invalid):
    clock = HandoffClock(node, monkeypatch, 'handoff')
    origin = list(clock.selected)
    origin[1] -= .57
    node._trajectory_controller_state.desired.positions = list(origin)
    def tick(current):
        state = node._trajectory_controller_state
        state.desired.positions = list(current.selected)
        state.desired.positions[1] -= 1e-16
        state.desired.velocities = [0.]*6
        state.desired.accelerations = [0.]*6
        if invalid == 'velocity': state.desired.velocities[0] = 1e-12
        elif invalid == 'acceleration': state.desired.accelerations[0] = 1e-12
        if invalid != 'no_ack': current.acknowledge(wrong_word=invalid == 'wrong_word')
    clock.on_tick = tick
    ok, reason = node._synchronize_trajectory_controller_to_feedback()
    assert not ok, reason
    assert len(clock.holds) == 1
    assert node._completed_command_reference is None


class ReferenceAction:
    def __init__(self, clock, final_status=3, acknowledge=True):
        self.clock, self.final_status, self.acknowledge = clock, final_status, acknowledge
        self.goals = []

    def wait_for_server(self, timeout):
        return True

    def send_goal(self, goal):
        self.goals.append((self.clock.now, deepcopy(goal)))
        assert len(self.goals) == 1
        assert goal.trajectory.joint_names == NAMES
        assert len(goal.trajectory.points) == 1
        point = goal.trajectory.points[0]
        assert list(point.positions) == list(self.clock.selected)
        assert list(point.velocities) == list(point.accelerations) == [0.] * 6

    def get_state(self):
        if (self.acknowledge and self.goals
                and self.clock.now-self.goals[0][0] >= .10-1e-9):
            return self.final_status
        return 1  # ACTIVE; never infer completion from time or telemetry alone.

    def get_result(self):
        return NS(error_code=0)


def setup_reference_action(node, monkeypatch, **kwargs):
    clock = HandoffClock(node, monkeypatch, 'reacquire')
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock.elapsed)
    monkeypatch.setattr(module.time, 'sleep', clock.sleep)
    client = ReferenceAction(clock, **kwargs)
    node._reference_action_client = client
    return clock, client


def test_reload_reacquires_only_after_new_same_raw_controller_q_action_success(node, monkeypatch):
    clock, client = setup_reference_action(node, monkeypatch)
    # Preserve raw controller q, not quantized SDK q, despite measured error.
    node._trajectory_controller_state.desired.positions[0] += COUNT * .1
    clock.selected = tuple(node._trajectory_controller_state.desired.positions)
    before = deepcopy(node._sdk_command_feedback.position)
    ok, reason = node._synchronize_trajectory_controller_to_feedback()
    assert ok, reason
    assert len(client.goals) == 1 and not clock.holds and not clock.explicit
    assert node._completed_command_reference['kind'] == 'completed_stationary_sdk_reacquisition_action'
    assert node._completed_command_reference['positions'] == clock.selected
    assert node._sdk_command_feedback.position == before


@pytest.mark.parametrize('failure', ['manual', 'epoch', 'accepted', 'sdk_word', 'controller_q', 'velocity', 'acceleration'])
def test_reacquisition_revokes_on_changed_evidence_without_cancel_or_second_goal(node, monkeypatch, failure):
    clock, client = setup_reference_action(node, monkeypatch)
    def tick(current):
        if current.elapsed < .04:
            return
        if failure == 'manual': node._gui_direct_mode = True
        elif failure == 'epoch': current.change_epoch()
        elif failure == 'accepted':
            current.stale_source = 'accepted'
            current.refresh()
        elif failure == 'sdk_word': current.acknowledge(wrong_word=True)
        elif failure == 'controller_q': node._trajectory_controller_state.desired.positions[0] += COUNT * .1
        elif failure == 'velocity': node._trajectory_controller_state.desired.velocities[0] = .001
        else: node._trajectory_controller_state.desired.accelerations[0] = .001
    clock.on_tick = tick
    ok, reason = node._synchronize_trajectory_controller_to_feedback()
    assert not ok, reason
    assert len(client.goals) == 1 and not clock.holds and not clock.explicit
    assert node._completed_command_reference is None
    assert clock.elapsed <= .06


@pytest.mark.parametrize('status', [2, 4, 5, 8, 9])
def test_unsuccessful_reference_action_does_not_claim_completion(node, monkeypatch, status):
    clock, client = setup_reference_action(node, monkeypatch, final_status=status)
    ok, reason = node._synchronize_trajectory_controller_to_feedback()
    assert not ok and 'failed with status' in reason
    assert len(client.goals) == 1 and node._completed_command_reference is None


def test_unacknowledged_stationary_action_times_out_without_fake_completion(node, monkeypatch):
    clock, client = setup_reference_action(node, monkeypatch, acknowledge=False)
    ok, reason = node._synchronize_trajectory_controller_to_feedback()
    assert not ok and 'timed out' in reason
    assert .4 <= clock.elapsed <= .42
    assert len(client.goals) == 1 and not clock.holds and not clock.explicit
    assert node._completed_command_reference is None


def test_late_ack_from_failed_transaction_is_not_used_as_new_completion(node, monkeypatch):
    clock, client = setup_reference_action(node, monkeypatch, acknowledge=False)
    assert not node._synchronize_trajectory_controller_to_feedback()[0]
    client.acknowledge = True  # The old goal succeeds after its deadline.
    assert client.get_state() == 3
    assert node._completed_command_reference is None
    # A subsequent call needs a new uniquely acknowledged zero-displacement
    # goal; observing old SUCCEEDED must never skip send_goal.
    client.goals = []
    ok, reason = node._synchronize_trajectory_controller_to_feedback()
    assert ok, reason
    assert len(client.goals) == 1
    assert clock.now-client.goals[0][0] >= .10-1e-9
