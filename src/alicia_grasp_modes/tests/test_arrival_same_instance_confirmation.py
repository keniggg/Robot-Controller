"""All competing raw fragments must qualify as one measured arrival instance."""
from copy import deepcopy
import numpy as np
import pytest

from test_observation_measurement_unknown import frozen, arrival, send, ack
from test_locked_rgb_hold import custom_frame, latest_status, module
from alicia_grasp_modes import observation_arrival_confirmation as arrival_helper
from alicia_grasp_modes.tabletop import _matching_candidates


def diagonal_depth():
    yy, xx = np.indices((28, 25))
    depth = np.full((28, 25), .35, np.float32)
    depth[np.abs(xx + (25./28.)*yy - 24.) < 1.2] = 0.
    return depth


def matching_frame(node, clock, joint):
    clock[0] += .2
    joint(clock[0]-.2); joint(clock[0]-.1)
    return custom_frame(node, clock, joint, target_depth=diagonal_depth())


def completed_matching_arrival(monkeypatch):
    node, clock, feed, joint, params, request = frozen(monkeypatch)
    anchor = node.tracker.anchor
    plane = node.tracker.plane_base.copy()
    publication_count = node.pubs['object'].publish.call_count
    request = arrival(node, clock, joint, request)
    original = module.segment_with_support_fallback
    original_recovery = arrival_helper.recover_arrival_matching_fragments
    raw_matches = []; calls = []
    def measured(*args, **kwargs):
        result = original(*args, **kwargs)
        raw_matches.append(len(_matching_candidates(result.candidates,
            anchor, node.tracker.last, node.config)))
        return result
    def recovery(*args, **kwargs):
        result, pair = original_recovery(*args, **kwargs)
        assert pair is not None
        calls.append((result, pair))
        return result, pair
    stamps = []
    with monkeypatch.context() as inspect:
        inspect.setattr(module, 'segment_with_support_fallback', measured)
        inspect.setattr(arrival_helper, 'recover_arrival_matching_fragments', recovery)
        for index in range(3):
            pair = matching_frame(node, clock, joint)
            stamps.append(pair[0].header.stamp.to_nsec())
            assert node.pubs['object'].publish.call_count == publication_count+(index == 2), latest_status(node)
    assert raw_matches == [2, 2, 2]
    assert len(calls) == 3
    assert ack(node)['qualified'] and ack(node)['source_stamps_ns'] == stamps, latest_status(node)
    assert node.tracker.anchor is anchor
    np.testing.assert_array_equal(node.tracker.plane_base, plane)
    terminal = deepcopy(request)
    terminal.update(state='qualified', request_id='matching-completed',
        stamp_ns=module.rospy.Time.now().to_nsec(), source_stamps_ns=stamps)
    send(node, terminal)
    assert ack(node)['qualified'] and node._observation_measurement is None
    return node, clock, feed, joint, params, node._locked_rgb_hold_reference['observation_arrival_retry']


def test_actual_three_matching_fragment_frames_only_publish_third_and_keep_anchor(monkeypatch):
    node, clock, feed, joint, params, marker = completed_matching_arrival(monkeypatch)
    assert latest_status(node)['target_lost'] is False
    raw = node.tracker.last
    assert np.count_nonzero(raw.mask) == raw.metrics['valid_points']
    assert _matching_candidates([raw], node.tracker.anchor, raw, node.config)


def test_original_single_match_does_not_request_multi_instance_recovery(monkeypatch):
    from test_observation_measurement_unknown import next_frame
    node, clock, feed, joint, params, request = frozen(monkeypatch)
    arrival(node, clock, joint, request)
    def forbidden(*args, **kwargs):
        pytest.fail('single raw match must retain the original path')
    monkeypatch.setattr(arrival_helper, 'recover_arrival_matching_fragments', forbidden)
    for _ in range(3):
        next_frame(node, clock, joint)
    assert ack(node)['qualified'], latest_status(node)


def test_new_multi_proposal_failure_resets_confirmation_without_publishing(monkeypatch):
    from test_observation_measurement_unknown import next_frame
    node, clock, feed, joint, params, request = frozen(monkeypatch)
    anchor, last = node.tracker.anchor, node.tracker.last
    arrival(node, clock, joint, request)
    next_frame(node, clock, joint)
    assert len(node._observation_measurement['confirmation'].source_stamps_ns) == 1
    count = node.pubs['object'].publish.call_count
    def rejected(*args, **kwargs):
        raise ValueError('arrival_matching_fragments_independent_instances')
    monkeypatch.setattr(arrival_helper, 'recover_arrival_matching_fragments', rejected)
    matching_frame(node, clock, joint)
    assert node._observation_measurement['confirmation'].source_stamps_ns == []
    assert node.tracker.anchor is anchor and node.tracker.last is last
    assert node.pubs['object'].publish.call_count == count
    assert not node.tracker.lost
