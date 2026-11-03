"""Arrival-qualified RGB must survive bounded support and metric recovery."""
import numpy as np
import pytest

from test_observation_arrival_station import completed, capture_refinement
from test_locked_rgb_hold import custom_frame, latest_status, module
from test_metric_ambiguity_hold import frame, assert_false_empty
from test_support_measurement_hold import support_gap_depth


@pytest.mark.parametrize('kind', ['support', 'ambiguity'])
def test_qualified_station_gap_then_unique_recovery_keeps_fixed_marker(monkeypatch, kind):
    n, clock, _, joint, _, marker = completed(monkeypatch)
    anchor, last = n.tracker.anchor, n.tracker.last
    plane = n.tracker.plane_base.copy()
    reference = n._locked_rgb_hold_reference
    stamp = n.last_good_source_stamp_ns
    deadline = marker['session_deadline_monotonic']
    calls = capture_refinement(monkeypatch)
    clock[0] += .1
    if kind == 'support':
        custom_frame(n, clock, joint, depth_image=support_gap_depth())
    else:
        frame(n, clock, joint)
        assert calls == [True, True]
    assert_false_empty(n)
    assert not n.tracker.lost, latest_status(n)
    assert n.tracker.last is last and n._locked_rgb_hold_reference is reference
    assert n.last_good_source_stamp_ns == stamp
    clock[0] += .2
    frame(n, clock, joint, extra=False)
    assert n.pubs['object'].publish.call_args[0][0].detected, latest_status(n)
    assert calls and all(calls)
    assert n._locked_source_pending is None and n._metric_ambiguity_pending is None
    assert n.tracker.anchor is anchor
    np.testing.assert_array_equal(n.tracker.plane_base, plane)
    assert n._locked_rgb_hold_reference['observation_arrival_retry'] is marker
    assert marker['session_deadline_monotonic'] == deadline
    np.testing.assert_array_equal(marker['arrival_q'], np.zeros(6))
    clock[0] += .2
    frame(n, clock, joint, extra=False)
    assert n.pubs['object'].publish.call_args[0][0].detected, latest_status(n)
    assert calls[-1] is True


@pytest.mark.parametrize('kind', ['support', 'ambiguity_hold', 'ambiguity_recovery'])
@pytest.mark.parametrize('change', ['manual', 'motion', 'camera', 'session_expired'])
def test_station_invalidation_inside_recovery_rgb_cannot_hold_or_publish(monkeypatch, kind, change):
    n, clock, _, joint, params, marker = completed(monkeypatch)
    clock[0] += .1
    if kind == 'support':
        custom_frame(n, clock, joint, depth_image=support_gap_depth())
    elif kind == 'ambiguity_recovery':
        frame(n, clock, joint)
    old = n.last_good_source_stamp_ns
    last = n.tracker.last
    original = module.refine_candidate
    calls = []
    def interrupted(*args, **kwargs):
        calls.append(kwargs.get('observation_arrival_retry', False))
        result = original(*args, **kwargs)
        if change == 'manual':
            params['/gui/joint_direct_mode'] = True
        elif change == 'motion':
            joint(clock[0], [.005, 0., 0., 0., 0., 0.])
            joint(clock[0]+.01, [0.]*6)
        elif change == 'camera':
            params['/camera']['fx'] += 1
        else:
            marker['session_deadline_ns'] = module.rospy.Time.now().to_nsec()
        return result
    monkeypatch.setattr(module, 'refine_candidate', interrupted)
    clock[0] += .1
    frame(n, clock, joint, extra=kind == 'ambiguity_hold')
    assert calls and all(calls)
    assert_false_empty(n)
    assert n.last_good_source_stamp_ns == old and n.tracker.last is last
    assert n._metric_ambiguity_pending is None
    assert 'observation_arrival_retry' not in (n._locked_rgb_hold_reference or {})


def test_arrival_ambiguity_still_rejects_same_colour_alternative(monkeypatch):
    n, clock, _, joint, _, _ = completed(monkeypatch)
    old = n.last_good_source_stamp_ns
    clock[0] += .1
    frame(n, clock, joint, same_colour=True)
    assert_false_empty(n)
    assert n.tracker.lost and n.last_good_source_stamp_ns == old
    assert n._metric_ambiguity_pending is None
