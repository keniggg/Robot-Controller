"""Real trailing feedback may arrive after RGB; never extrapolate a pose."""
import pytest
from std_msgs.msg import Header

from test_unknown_stream import module, rgb_interest_node
from test_rgb_handoff_stream import frozen_node, settled


def test_late_real_feedback_completes_original_source_bracket(monkeypatch):
    n, clock, _, joint, _ = rgb_interest_node(monkeypatch)
    joint(9.85)
    waits = []
    def deliver(delay):
        waits.append(delay)
        clock[0] += delay
        joint(9.95)
    monkeypatch.setattr(module.time, 'sleep', deliver)
    records, (_, before, after) = n._rgb_bracketed_feedback(9_900_000_000, 1_000_000_000)
    assert len(waits) == 1 and waits[0] <= .02
    assert before[0] == module.rospy.Time.from_sec(9.85).to_nsec()
    assert after[0] == module.rospy.Time.from_sec(9.95).to_nsec()
    assert tuple(records) == tuple(n._rgb_interest_feedback)


def test_missing_feedback_wait_is_bounded_and_never_synthesizes_after(monkeypatch):
    n, clock, _, joint, _ = rgb_interest_node(monkeypatch)
    joint(9.85)
    waits = []
    def wait(delay):
        waits.append(delay); clock[0] += delay
    monkeypatch.setattr(module.time, 'sleep', wait)
    with pytest.raises(ValueError, match='does_not_bracket_source'):
        n._rgb_bracketed_feedback(9_900_000_000, 1_000_000_000)
    assert 1 <= len(waits) <= 6 and sum(waits) <= .1200001
    assert len(n._rgb_interest_feedback) == 1


def test_feedback_wait_does_not_renew_source_age(monkeypatch):
    n, clock, _, joint, _ = rgb_interest_node(monkeypatch)
    joint(9.20)
    def deliver(delay):
        clock[0] += delay; joint(9.23)
    monkeypatch.setattr(module.time, 'sleep', deliver)
    with pytest.raises(ValueError, match='source_exceeds_existing_frame_limit'):
        n._rgb_bracketed_feedback(9_210_000_000, 1_000_000_000)


def test_epoch_change_while_waiting_rejects_new_feedback(monkeypatch):
    n, _, _, joint, _ = rgb_interest_node(monkeypatch)
    joint(9.85)
    def change(_):
        n._rgb_interest_epoch_cb(Header(stamp=module.rospy.Time(2), frame_id='sdk_reference_epoch'))
        joint(9.95)
    monkeypatch.setattr(module.time, 'sleep', change)
    with pytest.raises(ValueError, match='epoch_changed'):
        n._rgb_bracketed_feedback(9_900_000_000, 1_000_000_000)


def test_existing_excessive_bracket_is_not_repaired_by_waiting(monkeypatch):
    n, _, _, joint, _ = rgb_interest_node(monkeypatch)
    joint(9.60); joint(10.)
    monkeypatch.setattr(module.time, 'sleep', lambda _:pytest.fail('invalid bracket must fail immediately'))
    with pytest.raises(ValueError):
        n._rgb_bracketed_feedback(9_900_000_000, 1_000_000_000)


def test_initial_set_can_confirm_three_frames_with_delayed_feedback(monkeypatch):
    n, clock, feed, joint, _ = rgb_interest_node(monkeypatch)
    monkeypatch.setattr(module.time, 'sleep', lambda _:joint(clock[0]-.05))
    for index in range(3):
        clock[0] = 10.+index*.6
        feed(bracket=False)
    assert n._rgb_observation_interest['observation_interest_ready']
    assert n.tracker.anchor is None


def test_handoff_can_confirm_real_depth_after_delayed_feedback(monkeypatch):
    n, clock, feed, joint, _, _ = frozen_node(monkeypatch)
    settled(n, clock)
    monkeypatch.setattr(module.time, 'sleep', lambda _:joint(clock[0]-.05))
    for index in range(3):
        clock[0] += .2
        feed(bracket=False, target_depth=.38)
        assert n.pubs['object'].publish.call_args[0][0].detected is (index == 2)
