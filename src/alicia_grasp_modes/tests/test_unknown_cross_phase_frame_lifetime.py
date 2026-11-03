"""Actual producer paths across the same bounded station-frame clock edges.

Gen24/gen27 captured timing differences are applied to existing synthetic RGB-D
transport fixtures. Segmentation, refinement, identity and publication remain
the production process, not a replay claim about the recorded physical pixels.
"""
from copy import deepcopy

import numpy as np
import pytest

from test_locked_rgb_hold import locked, custom_frame, latest_status, module
from test_observation_arrival_station import completed
from test_observation_arrival_frame_completion_guards import pending_source


CLOCKS = {
    'gen24': (.494414091, .530324221, .828624487),
    'gen27': (.446166277, .492824078, .815897703),
}


def station(monkeypatch, phase):
    if phase == 'initial':
        node, clock, feed, joint, params = locked(monkeypatch, initial_station=True)
        marker = node._locked_rgb_hold_reference['initial_station_current_support']
    else:
        node, clock, feed, joint, params, marker = completed(monkeypatch)
    return node, clock, feed, joint, params, marker


def install_wait(monkeypatch, node, source, kind, pause):
    """Only transport/context wall time changes; never substitute a gate."""
    calls = []
    if kind == 'decode':
        original = module.rospy.get_param

        def parameter_read(key, *args, **kwargs):
            result = original(key, *args, **kwargs)
            if key == '/camera' and not calls:
                calls.append(kind)
                pause()
            return result
        monkeypatch.setattr(module.rospy, 'get_param', parameter_read)
    elif kind == 'tf':
        original = node.buffer.lookup_transform

        def lookup(*args, **kwargs):
            result = original(*args, **kwargs)
            if (args[0] == node.base_frame and args[1] == node.camera_frame
                    and args[2] == source[0].header.stamp and not calls):
                calls.append(kind)
                pause()
            return result
        monkeypatch.setattr(node.buffer, 'lookup_transform', lookup)
    else:
        owner = node if kind == 'context' else module
        name = '_rgb_interest_source_context' if kind == 'context' else 'refine_candidate'
        original = getattr(owner, name)

        def delayed(*args, **kwargs):
            result = original(*args, **kwargs)
            if not calls:
                calls.append(kind)
                pause()
            return result
        monkeypatch.setattr(owner, name, delayed)
    return calls


@pytest.mark.parametrize('phase', ['initial', 'arrival'])
@pytest.mark.parametrize('timing_case,wait_at', [
    (case, stage) for case in ('gen24', 'gen27') for stage in ('tf', 'context', 'rgb')
] + [('gen27', 'decode')])
def test_timely_normal_frame_finishes_across_old_reference_cutoff_in_both_phases(
        monkeypatch, phase, timing_case, wait_at):
    node, clock, _, joint, _, marker = station(monkeypatch, phase)
    capture_gap, start_age, completion_age = CLOCKS[timing_case]
    pair, reference = pending_source(node, clock, joint,
        capture_gap=capture_gap, start_age=start_age)
    source_ns = pair[0].header.stamp.to_nsec()
    old_ns = reference['source_stamp_ns']
    last, anchor, plane = node.tracker.last, node.tracker.anchor, node.tracker.plane_base.copy()
    initial_count = node.pubs['object'].publish.call_count

    def pause():
        assert node.pubs['object'].publish.call_count == initial_count
        assert node.tracker.last is last and node.last_good_source_stamp_ns == old_ns
        clock[0] = old_ns / 1e9 + completion_age
    calls = install_wait(monkeypatch, node, pair, wait_at, pause)
    node.process(None)
    assert calls == [wait_at]
    status = latest_status(node)
    assert status['state'] == 'ready', status
    assert node.pubs['object'].publish.call_count == initial_count + 1
    assert node.last_good_source_stamp_ns == source_ns
    assert node._locked_rgb_hold_reference is not reference
    assert reference['source_stamp_ns'] == old_ns
    assert node.tracker.anchor is anchor
    np.testing.assert_array_equal(node.tracker.plane_base, plane)
    marker_key = 'initial_station_current_support' if phase == 'initial' else 'observation_arrival_retry'
    assert node._locked_rgb_hold_reference[marker_key] is marker
    timing_key = 'station_frame_completion' if phase == 'initial' else 'arrival_frame_completion'
    evidence = status['frame_timing'][timing_key]
    assert evidence['used_past_geometry_deadline'] is True
    assert evidence['geometry_deadline_ns'] == old_ns + 800_000_000
    assert evidence['metric_geometry_authority'] is False
    assert evidence['motion_authorized'] is False
    assert getattr(node, '_arrival_frame_completion', None) is None
    # An ordinary following frame must use its own new source, preserving the
    # original station instead of rolling the fixed marker/deadline forward.
    clock[0] += .15
    next_pair = custom_frame(node, clock, joint, target_depth=.35)
    assert latest_status(node)['state'] == 'ready'
    assert node.last_good_source_stamp_ns == next_pair[0].header.stamp.to_nsec()
    assert node._locked_rgb_hold_reference[marker_key] is marker


@pytest.mark.parametrize('phase', ['initial', 'arrival'])
@pytest.mark.parametrize('change', ['current_expired', 'manual', 'out_and_back',
                                   'camera', 'reference_replaced'])
def test_context_wait_cannot_authorize_changed_or_expired_frame(monkeypatch, phase, change):
    node, clock, _, joint, params, _ = station(monkeypatch, phase)
    pair, reference = pending_source(node, clock, joint,
        capture_gap=CLOCKS['gen27'][0], start_age=CLOCKS['gen27'][1])
    old_ns = reference['source_stamp_ns']
    last, anchor = node.tracker.last, node.tracker.anchor

    def pause():
        clock[0] = old_ns / 1e9 + CLOCKS['gen27'][2]
        if change == 'current_expired':
            clock[0] = pair[0].header.stamp.to_sec() + .801
        elif change == 'manual':
            params['/gui/joint_direct_mode'] = True
        elif change == 'camera':
            params['/camera']['fx'] += 1
        elif change == 'reference_replaced':
            node._locked_rgb_hold_reference = dict(reference)
        else:
            joint(clock[0], [.006, 0., 0., 0., 0., 0.])
            joint(clock[0] + .001, [0.] * 6)
    calls = install_wait(monkeypatch, node, pair, 'context', pause)
    node.process(None)
    assert calls == ['context']
    assert not node.pubs['object'].publish.call_args[0][0].detected, latest_status(node)
    assert node.last_good_source_stamp_ns == old_ns
    assert node.tracker.last is last and node.tracker.anchor is anchor
    assert getattr(node, '_arrival_frame_completion', None) is None
    assert not np.any(node.bridge.imgmsg_to_cv2(
        node.pubs['object_mask'].publish.call_args[0][0], 'mono8'))


@pytest.mark.parametrize('phase', ['initial', 'arrival'])
def test_current_source_expiring_during_main_tf_is_not_renewed(monkeypatch, phase):
    node, clock, _, joint, _, _ = station(monkeypatch, phase)
    pair, reference = pending_source(node, clock, joint,
        capture_gap=CLOCKS['gen27'][0], start_age=CLOCKS['gen27'][1])
    old_ns, last = reference['source_stamp_ns'], node.tracker.last
    install_wait(monkeypatch, node, pair, 'tf',
        lambda: clock.__setitem__(0, pair[0].header.stamp.to_sec() + .801))
    node.process(None)
    assert not node.pubs['object'].publish.call_args[0][0].detected
    assert node.last_good_source_stamp_ns == old_ns and node.tracker.last is last
    assert node.tracker.lost and getattr(node, '_arrival_frame_completion', None) is None
    # A later perfectly good frame cannot revive the retired identity.
    first_loss = deepcopy(node.first_loss)
    clock[0] += .15
    custom_frame(node, clock, joint, target_depth=.35)
    assert not node.pubs['object'].publish.call_args[0][0].detected
    assert node.first_loss == first_loss and node.last_good_source_stamp_ns == old_ns


@pytest.mark.parametrize('phase', ['initial', 'arrival'])
@pytest.mark.parametrize('failure', ['invalid_camera', 'current_expired'])
def test_decode_wait_must_still_pass_current_source_and_camera_gates(monkeypatch, phase, failure):
    node, clock, _, joint, params, _ = station(monkeypatch, phase)
    pair, reference = pending_source(node, clock, joint,
        capture_gap=CLOCKS['gen27'][0], start_age=CLOCKS['gen27'][1])
    old_ns, last = reference['source_stamp_ns'], node.tracker.last

    def pause():
        clock[0] = old_ns / 1e9 + CLOCKS['gen27'][2]
        if failure == 'invalid_camera':
            params['/camera']['align_depth_to_color'] = False
        else:
            clock[0] = pair[0].header.stamp.to_sec() + .801
    calls = install_wait(monkeypatch, node, pair, 'decode', pause)
    node.process(None)
    assert calls == ['decode']
    assert not node.pubs['object'].publish.call_args[0][0].detected
    assert node.last_good_source_stamp_ns == old_ns and node.tracker.last is last
    assert getattr(node, '_arrival_frame_completion', None) is None


def test_source_captured_timely_but_processing_started_late_cannot_retroactively_enroll(monkeypatch):
    node, clock, _, joint, _, _ = station(monkeypatch, 'initial')
    _, reference = pending_source(node, clock, joint, capture_gap=.70, start_age=.801)
    old_ns, last = reference['source_stamp_ns'], node.tracker.last
    node.process(None)
    assert not node.pubs['object'].publish.call_args[0][0].detected
    assert node.last_good_source_stamp_ns == old_ns and node.tracker.last is last
    assert 'station_frame_completion' not in latest_status(node)['frame_timing']
    assert getattr(node, '_arrival_frame_completion', None) is None
