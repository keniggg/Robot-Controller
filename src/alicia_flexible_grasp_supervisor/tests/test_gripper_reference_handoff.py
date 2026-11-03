"""Gripper entry uses the same frozen-word admission as arm execution."""
from types import SimpleNamespace as NS

import pytest

from test_gateway_command_reference import node, module
from test_gateway_reference_handoff import HandoffClock


def test_open_after_manual_handoff_admits_original_words_before_gripper(node, monkeypatch):
    clock = HandoffClock(node, monkeypatch, 'handoff')
    clock.on_tick = lambda c: c.acknowledge() if c.elapsed >= .04 else None
    result = node.handle_gripper(NS(value=.05))
    assert result.success, result.message
    assert len(clock.holds) == 1
    assert node.sent == [(.05, list(clock.selected))]
    assert tuple(node.joint_cmd.last_positions[:-1]) != clock.selected
    assert clock.elapsed <= .4


@pytest.mark.parametrize('failure', ['no_ack', 'manual', 'epoch', 'wrong_word'])
def test_failed_gripper_handoff_never_publishes_gripper_or_rebases(node, monkeypatch, failure):
    clock = HandoffClock(node, monkeypatch, 'handoff')
    def tick(c):
        if c.elapsed < .04:
            return
        if failure == 'manual':
            node._gui_direct_mode = True
        elif failure == 'epoch':
            c.change_epoch()
        elif failure == 'wrong_word':
            c.acknowledge(wrong_word=True)
    clock.on_tick = tick
    result = node.handle_gripper(NS(value=.05))
    assert not result.success
    assert node.sent == []
    assert len(clock.holds) <= 1 and len(clock.explicit) <= 1
    assert clock.elapsed <= .42 + 1e-9


def test_missing_reference_does_not_bootstrap_arm_for_a_gripper_request(node):
    node._sdk_command_feedback = node._driver_control_reference = None
    node._actuation_status = 'PENDING:POSITIVE_ENABLE_REQUESTED'
    result = node.handle_gripper(NS(value=.05))
    assert not result.success and node.sent == []
