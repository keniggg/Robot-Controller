"""Acknowledged arrival crop qualification survives only its measured station."""
from copy import deepcopy

import numpy as np
import pytest
from std_msgs.msg import Header

from test_observation_measurement_unknown import frozen, arrival, next_frame, send, ack
from test_locked_rgb_hold import locked, latest_status, module


def completed(monkeypatch):
    node, clock, feed, joint, params, request = frozen(monkeypatch)
    original_deadline = node._observation_measurement['deadline_monotonic']
    request = arrival(node, clock, joint, request)
    for _ in range(3):
        next_frame(node, clock, joint)
    stamps = ack(node)['source_stamps_ns']
    clock[0] += .01
    request = dict(request, state='qualified', request_id='completed',
        stamp_ns=module.rospy.Time.now().to_nsec(), source_stamps_ns=stamps)
    send(node, request)
    assert ack(node)['accepted'] and node._observation_measurement is None
    marker = node._locked_rgb_hold_reference['observation_arrival_retry']
    assert marker['session_deadline_monotonic'] == original_deadline
    return node, clock, feed, joint, params, marker


def capture_refinement(monkeypatch):
    calls = []
    original = module.refine_candidate
    def refine(*args, **kwargs):
        calls.append(kwargs.get('observation_arrival_retry', False))
        return original(*args, **kwargs)
    monkeypatch.setattr(module, 'refine_candidate', refine)
    return calls


def test_three_frame_arrival_then_normal_frames_keep_fixed_qualification(monkeypatch):
    node, clock, _, joint, _, marker = completed(monkeypatch)
    anchor = node.tracker.anchor
    plane = node.tracker.plane_base.copy()
    original_q = marker['arrival_q'].copy()
    deadline = marker['session_deadline_monotonic']
    stamps = []
    calls = capture_refinement(monkeypatch)
    # Continue beyond the original capture window without extending the
    # session or using an old RGB-D frame to bridge an age violation.
    for _ in range(28):
        pair = next_frame(node, clock, joint)
        stamps.append(pair[0].header.stamp.to_nsec())
        assert latest_status(node)['state'] == 'ready', latest_status(node)
        assert node._locked_rgb_hold_reference['observation_arrival_retry'] is marker
        assert marker['session_deadline_monotonic'] == deadline
    assert calls == [True] * 28
    assert stamps == sorted(set(stamps))
    assert np.array_equal(marker['arrival_q'], original_q)
    assert node.tracker.anchor is anchor and np.array_equal(node.tracker.plane_base, plane)
    assert not node.tracker.lost


def test_initial_and_ordinary_tracking_never_get_arrival_retry(monkeypatch):
    calls = capture_refinement(monkeypatch)
    node, clock, feed, _, _ = locked(monkeypatch)
    clock[0] += .2
    feed(target_depth=.35)
    assert calls and not any(calls)
    assert 'observation_arrival_retry' not in node._locked_rgb_hold_reference


@pytest.mark.parametrize('change', ['manual', 'epoch', 'camera', 'source', 'model',
    'selection', 'config', 'anchor', 'plane', 'lost', 'first_loss',
    'old_reference', 'session_ros_deadline', 'session_monotonic_deadline', 'out_and_back'])
def test_station_change_permanently_removes_only_extra_crop_qualification(monkeypatch, change):
    node, clock, _, joint, params, marker = completed(monkeypatch)
    calls = capture_refinement(monkeypatch)
    if change == 'manual': params['/gui/joint_direct_mode'] = True
    elif change == 'epoch': node._rgb_interest_epoch_cb(Header(stamp=module.rospy.Time(2), frame_id='sdk_reference_epoch'))
    elif change == 'camera': params['/camera']['fx'] += 1
    elif change == 'source': params['/handeye']['changed'] = True
    elif change == 'model': params['/robot_description'] = '<robot name="changed"/>'
    elif change == 'selection': params['/grasp_mode/selection']['generation'] += 1
    elif change == 'config': node.config.support_threshold_m = .0019
    elif change == 'anchor': node.tracker.anchor.position_base[0] += .001
    elif change == 'plane': node.tracker.plane_base[3] += .001
    elif change == 'lost': node.tracker.lost = True
    elif change == 'first_loss': node.first_loss = dict(reason='prior_loss')
    elif change == 'old_reference': clock[0] += .81
    elif change == 'session_ros_deadline': marker['session_deadline_ns'] = module.rospy.Time.now().to_nsec()
    elif change == 'session_monotonic_deadline': marker['session_deadline_monotonic'] = module.time.monotonic() - .01
    elif change == 'out_and_back':
        joint(clock[0], [.005, 0., 0., 0., 0., 0.])
        joint(clock[0]+.01, [0.] * 6)
    next_frame(node, clock, joint)
    assert not any(calls)
    assert 'observation_arrival_retry' not in (node._locked_rgb_hold_reference or {})


def test_small_steps_cannot_slide_the_fixed_arrival_joint_reference(monkeypatch):
    node, clock, _, joint, _, marker = completed(monkeypatch)
    calls = capture_refinement(monkeypatch)
    next_frame(node, clock, joint, q=[.003, 0., 0., 0., 0., 0.])
    assert calls[-1] is True
    next_frame(node, clock, joint, q=[.006, 0., 0., 0., 0., 0.])
    assert calls[-1] is False
    next_frame(node, clock, joint, q=[0.] * 6)
    assert calls[-1] is False
    assert np.array_equal(marker['arrival_q'], np.zeros(6))


@pytest.mark.parametrize('change', ['manual', 'motion', 'camera'])
def test_invalidation_during_rgb_work_cannot_publish_or_inherit(monkeypatch, change):
    node, clock, _, joint, params, _ = completed(monkeypatch)
    original = module.refine_candidate
    before = node.last_good_source_stamp_ns
    def interrupted(*args, **kwargs):
        assert kwargs['observation_arrival_retry'] is True
        result = original(*args, **kwargs)
        if change == 'manual': params['/gui/joint_direct_mode'] = True
        elif change == 'motion': joint(clock[0], [.005, 0., 0., 0., 0., 0.])
        else: params['/camera']['fx'] += 1
        return result
    monkeypatch.setattr(module, 'refine_candidate', interrupted)
    next_frame(node, clock, joint)
    assert node.last_good_source_stamp_ns == before
    assert not node.pubs['object'].publish.call_args[0][0].detected
    assert 'observation_arrival_retry' not in (node._locked_rgb_hold_reference or {})


def test_new_observation_execution_clears_prior_station_marker(monkeypatch):
    node, clock, _, joint, params, _ = completed(monkeypatch)
    request = deepcopy(node._observation_measurement_status['request'])
    for key in ('source_stamps_ns', 'settle'):
        request.pop(key, None)
    request.update(state='executing', lease_id='second_station', request_id='second_executing',
        stamp_ns=module.rospy.Time.now().to_nsec(), arrival_stamp_ns=0, capture_deadline_ns=0,
        view_source_stamp_ns=node.last_good_source_stamp_ns)
    send(node, request)
    assert ack(node)['accepted'], ack(node)
    assert node._locked_rgb_hold_reference is None
