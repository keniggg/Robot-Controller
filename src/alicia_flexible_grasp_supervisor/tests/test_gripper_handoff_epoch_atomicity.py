"""No gripper entry may turn a changed reference into an arm bootstrap.

Real gateway methods and epoch callbacks run with inert transports. These
tests exercise callback boundaries; they do not claim historical arm motion.
"""
from types import SimpleNamespace as NS
import threading

import pytest
from std_msgs.msg import String

from test_gateway_command_reference import COUNT, module, node
from test_gateway_reference_handoff import HandoffClock


def test_epoch_change_after_ack_does_not_reinstall_old_completion(node, monkeypatch):
    clock = HandoffClock(node, monkeypatch, 'handoff')
    clock.on_tick = lambda c: c.acknowledge() if c.elapsed >= .04 else None
    original = node._validated_controller_sdk_reference

    def reset_after_validation(names):
        result = original(names)
        clock.change_epoch()
        return result

    monkeypatch.setattr(node, '_validated_controller_sdk_reference', reset_after_validation)
    result = node.handle_gripper(NS(value=.05))
    assert not result.success and not node.sent
    assert node._completed_command_reference is None
    assert node._observation_sync_hold_evidence is None


def test_manual_takeover_before_publish_without_arm_hold_is_rejected(node, monkeypatch):
    node._test_params['/gripper'] = {'hold_arm_during_gripper_commands': False}
    original = node._manual_control_active
    checks = []

    def takeover_after_unlocked_check():
        result = original()
        checks.append(result)
        if len(checks) == 2:
            node._gui_mode_cb(NS(data=True))
        return result

    monkeypatch.setattr(node, '_manual_control_active', takeover_after_unlocked_check)
    result = node.handle_gripper(NS(value=.05))
    assert not result.success and not node.sent


def _change_reference(clock, kind):
    gateway = clock.gateway
    if kind == 'epoch':
        clock.change_epoch()
        gateway._actuation_status = 'PENDING:POSITIVE_ENABLE_REQUESTED'
        clock.mode = 'initial'
        clock.selected = tuple(gateway._observation_joint_feedback.position[:-1])
    elif kind == 'word':
        gateway._sdk_command_feedback.position[1] += COUNT
        gateway._driver_control_reference.position[1] += COUNT
        clock.selected = tuple(gateway._sdk_command_feedback.position)
    elif kind == 'opening':
        gateway._driver_control_reference.position[-1] = .045
    elif kind == 'manual':
        gateway._gui_mode_cb(NS(data=True))
    else:
        raise AssertionError(kind)


@pytest.mark.parametrize('change', ['epoch', 'word', 'opening'])
def test_changed_reference_before_helper_cannot_publish_arm_hold(node, monkeypatch, change):
    clock = HandoffClock(node, monkeypatch, 'handoff')
    node._driver_control_reference.position[-1] = .0288
    expected = dict(epoch_ns=node._control_reference_epoch_ns,
                    sdk_words=node._sdk_arm_words(clock.selected), opening=.0288,
                    max_feedback_age_sec=.5)
    original = node._synchronize_trajectory_controller_to_feedback
    bound = []

    def callback_before_helper(expected_handoff=None):
        bound.append(expected_handoff)
        _change_reference(clock, change)
        return original(expected_handoff=expected_handoff)

    clock.on_tick = lambda c: c.acknowledge() if c.elapsed >= .04 else None
    monkeypatch.setattr(node, '_synchronize_trajectory_controller_to_feedback', callback_before_helper)
    result = node.handle_gripper(NS(value=.04))
    assert bound == [expected]
    assert not result.success, result.message
    assert not clock.holds and not clock.explicit and not node.sent


@pytest.mark.parametrize('change', ['epoch', 'word', 'opening'])
def test_changed_reference_after_selection_is_rejected_before_publish(node, monkeypatch, change):
    clock = HandoffClock(node, monkeypatch, 'handoff')
    node._driver_control_reference.position[-1] = .0288
    original = node._select_controller_sync_reference

    def callback_after_selection():
        selected = original()
        _change_reference(clock, change)
        return selected

    monkeypatch.setattr(node, '_select_controller_sync_reference', callback_after_selection)
    result = node.handle_gripper(NS(value=.04))
    assert not result.success, result.message
    assert not clock.holds and not clock.explicit and not node.sent


def test_epoch_callback_between_admission_and_publish_cannot_publish_in_new_epoch(node, monkeypatch):
    clock = HandoffClock(node, monkeypatch, 'handoff')
    old_epoch = node._control_reference_epoch_ns
    original_admission = node._controller_reference_admission_state
    original_publish = node.trajectory_command_pub.publish
    entered = threading.Event()
    finished = threading.Event()
    callback_errors = []
    workers = []
    publication_epochs = []

    def reset_epoch():
        entered.set()
        try:
            clock.change_epoch()
        except BaseException as error:
            callback_errors.append(error)
        finally:
            finished.set()

    def admission(*args, **kwargs):
        result = original_admission(*args, **kwargs)
        stationary = args[4] if len(args) > 4 else kwargs['require_stationary']
        if not stationary and not workers:
            worker = threading.Thread(target=reset_epoch, daemon=True)
            workers.append(worker)
            worker.start()
            assert entered.wait(.5), 'epoch callback did not start'
            # With no commit lock, deliver the real callback in this gap.
            # If the gate already holds that lock, the callback must wait
            # until after its old-epoch publication instead.
            if not node._command_reference_lock._is_owned():
                assert finished.wait(.5), 'unlocked epoch callback did not finish'
        return result

    def publish(message):
        publication_epochs.append(node._control_reference_epoch_ns)
        original_publish(message)

    monkeypatch.setattr(node, '_controller_reference_admission_state', admission)
    monkeypatch.setattr(node.trajectory_command_pub, 'publish', publish)
    clock.on_tick = lambda _c: finished.wait(.5)
    result = node.handle_gripper(NS(value=.05))
    for worker in workers:
        worker.join(.5)
        assert not worker.is_alive(), 'reference lock held across the acknowledgment wait'
    assert workers and not callback_errors
    assert not result.success and not node.sent
    assert all(epoch == old_epoch for epoch in publication_epochs), publication_epochs
    assert not clock.explicit


def test_handoff_ack_cannot_turn_requested_opening_into_closing(node, monkeypatch):
    clock = HandoffClock(node, monkeypatch, 'handoff')
    node._driver_control_reference.position[-1] = .0288

    def changed_opening_after_ack(current):
        if current.elapsed >= .04:
            current.acknowledge()
            node._driver_control_reference.position[-1] = .045

    clock.on_tick = changed_opening_after_ack
    result = node.handle_gripper(NS(value=.04))
    assert not result.success, result.message
    assert len(clock.holds) <= 1 and not node.sent


def test_closing_request_cannot_enter_reference_handoff(node, monkeypatch):
    clock = HandoffClock(node, monkeypatch, 'handoff')
    node._driver_control_reference.position[-1] = .04
    result = node.handle_gripper(NS(value=.035))
    assert not result.success and not clock.holds and not clock.explicit and not node.sent


def test_nonclosing_handoff_keeps_partial_opening_until_final_gripper_command(node, monkeypatch):
    clock = HandoffClock(node, monkeypatch, 'handoff')
    node._driver_control_reference.position[-1] = .0288

    def preserve_opening_after_ack(current):
        if current.elapsed >= .04:
            current.acknowledge()
            node._driver_control_reference.position[-1] = .0288

    clock.on_tick = preserve_opening_after_ack
    result = node.handle_gripper(NS(value=.04))
    assert result.success, result.message
    assert len(clock.holds) == 1
    assert node.sent == [(.04, list(clock.selected))]


@pytest.mark.parametrize('mode', ['handoff', 'reacquire'])
@pytest.mark.parametrize('change', ['epoch', 'word', 'opening', 'manual'])
def test_reference_change_before_final_gripper_publish_is_rejected(node, monkeypatch, mode, change):
    clock = HandoffClock(node, monkeypatch, mode)
    node._driver_control_reference.position[-1] = .0288

    def preserve_opening_after_ack(current):
        if current.elapsed >= .04:
            current.acknowledge()
            node._driver_control_reference.position[-1] = .0288

    clock.on_tick = preserve_opening_after_ack
    original = node._gripper_arm_positions_for_command

    def callback_before_submission(*args, **kwargs):
        prepared = original(*args, **kwargs)
        _change_reference(clock, change)
        return prepared

    monkeypatch.setattr(node, '_gripper_arm_positions_for_command', callback_before_submission)
    result = node.handle_gripper(NS(value=.04))
    assert not result.success, result.message
    assert not node.sent
    assert len(clock.holds) <= (1 if mode == 'handoff' else 0)


def test_manual_callback_and_final_gripper_commit_share_one_reference_lock(node, monkeypatch):
    clock = HandoffClock(node, monkeypatch, 'reacquire')
    entered = threading.Event()
    finished = threading.Event()
    errors = []
    workers = []
    original = node.gripper.set_position

    def takeover():
        entered.set()
        try:
            node._gui_mode_cb(NS(data=True))
        except BaseException as error:
            errors.append(error)
        finally:
            finished.set()

    def publish(*args, **kwargs):
        worker = threading.Thread(target=takeover, daemon=True)
        workers.append(worker)
        worker.start()
        assert entered.wait(.5)
        assert not finished.wait(.03), 'manual owner changed within the gripper commit'
        assert node._gui_direct_mode is False
        return original(*args, **kwargs)

    monkeypatch.setattr(node.gripper, 'set_position', publish)
    result = node.handle_gripper(NS(value=.04))
    for worker in workers:
        worker.join(.5)
        assert not worker.is_alive()
    assert result.success and workers and not errors, result.message
    assert node._gui_direct_mode and node.sent == [(.04, list(clock.selected))]


@pytest.mark.parametrize('mode', ['handoff', 'reacquire'])
@pytest.mark.parametrize('change', [
    'elapsed', 'future_sdk', 'future_accepted',
    'actuation_revoked', 'stale_actuation', 'future_actuation',
])
def test_final_gripper_commit_requires_current_feedback_and_actuation(node, monkeypatch, mode, change):
    clock = HandoffClock(node, monkeypatch, mode)
    node._driver_control_reference.position[-1] = .0288

    def preserve_opening_after_ack(current):
        if current.elapsed >= .04:
            current.acknowledge()
            node._driver_control_reference.position[-1] = .0288

    clock.on_tick = preserve_opening_after_ack
    original = node._gripper_arm_positions_for_command

    def callback_before_submission(*args, **kwargs):
        prepared = original(*args, **kwargs)
        if change == 'elapsed':
            # Scheduler/RPC time passes without another accepted observation.
            clock.elapsed += .6
        elif change == 'future_sdk':
            for msg in (node._sdk_command_feedback, node._driver_control_reference):
                msg.header.stamp = module.rospy.Time.from_sec(clock.now+.01)
        elif change == 'future_accepted':
            node._observation_joint_feedback.header.stamp = module.rospy.Time.from_sec(clock.now+.01)
        elif change == 'actuation_revoked':
            node._actuation_cb(String(data='UNCONFIRMED:CONFIRMED_FEEDBACK_STALE'))
        elif change == 'stale_actuation':
            node._actuation_received_sec = clock.now-2.01
        elif change == 'future_actuation':
            node._actuation_received_sec = clock.now+.01
        else:
            raise AssertionError(change)
        return prepared

    monkeypatch.setattr(node, '_gripper_arm_positions_for_command', callback_before_submission)
    result = node.handle_gripper(NS(value=.04))
    assert not result.success, result.message
    assert not node.sent
    assert len(clock.holds) <= (1 if mode == 'handoff' else 0)
