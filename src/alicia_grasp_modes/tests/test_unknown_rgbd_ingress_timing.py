"""Locate RGB-D transport/pairing delay without changing exact-frame authority."""
import json

import pytest
from sensor_msgs.msg import Image

from test_unknown_stream import module, node, diagnostic_node


def image(stamp, seq):
    msg = Image()
    msg.header.stamp = module.rospy.Time.from_sec(stamp)
    msg.header.seq = seq
    msg.header.frame_id = 'camera_link'
    return msg


def ingress_node(monkeypatch, capacity=32):
    clock = [10.01]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0]+100.)
    monkeypatch.setattr(module.rospy.Time, 'now', staticmethod(lambda: module.rospy.Time.from_sec(clock[0])))
    n = node()
    n._frame_arrivals = module.RgbdIngressTiming(capacity)
    color, depth = module.message_filters.SimpleFilter(), module.message_filters.SimpleFilter()
    # Use the real Noetic synchronizer and production raw/frame callbacks.
    color.registerCallback(n._raw_frame_arrival, 'color')
    depth.registerCallback(n._raw_frame_arrival, 'depth')
    sync = module.message_filters.TimeSynchronizer([color, depth], 8)
    sync.registerCallback(n.frame)
    return n, clock, color, depth, sync


@pytest.mark.parametrize('first', ['color', 'depth'])
def test_actual_exact_synchronizer_separates_one_stream_delay_from_processing(monkeypatch, first):
    n, clock, color, depth, _ = ingress_node(monkeypatch)
    streams = {'color': color, 'depth': depth}
    messages = {'color': image(10., 51), 'depth': image(10., 73)}
    second = 'depth' if first == 'color' else 'color'
    streams[first].signalMessage(messages[first])
    assert n.latest is None
    clock[0] = 10.56
    streams[second].signalMessage(messages[second])
    timing = n._latest_frame_timing
    assert n.latest == (messages['color'], messages['depth'])
    assert timing['raw_arrivals'][first]['source_age_at_arrival_sec'] == pytest.approx(.01)
    assert timing['raw_arrivals'][second]['source_age_at_arrival_sec'] == pytest.approx(.56)
    assert timing['raw_arrivals'][first]['until_exact_pair_callback_sec'] == pytest.approx(.55)
    assert timing['raw_arrivals'][second]['until_exact_pair_callback_sec'] == pytest.approx(0.)
    assert timing['raw_arrival_difference_sec'] == pytest.approx(.55)
    assert timing['after_latest_raw_arrival_sec'] == pytest.approx(0.)
    assert timing['input_lock_wait_sec'] == pytest.approx(0.)
    assert timing['exact_pair_source_age_sec'] == pytest.approx(.56)
    assert timing['color_seq'] == 51 and timing['depth_seq'] == 73
    assert sync_queue_sizes(_) == [0, 0]
    json.dumps(timing, allow_nan=False)


def sync_queue_sizes(sync):
    return [len(queue) for queue in sync.queues]


def test_mismatched_and_out_of_order_arrivals_do_not_change_exact_matching_or_latest_pair(monkeypatch):
    n, clock, color, depth, sync = ingress_node(monkeypatch)
    older, newer = image(10., 10), image(10.1, 11)
    color.signalMessage(older)
    clock[0] = 10.2
    newer_depth = image(10.1, 12)
    depth.signalMessage(newer_depth)
    assert n.latest is None
    clock[0] = 10.7
    color.signalMessage(newer)
    assert n.latest == (newer, newer_depth)
    assert n._latest_frame_timing['raw_arrivals']['depth']['until_exact_pair_callback_sec'] == pytest.approx(.5)
    accepted_wall = n.last_receive_wall
    clock[0] = 10.8
    depth.signalMessage(image(10., 13))
    # The original latest-only/generation guard still discards older pairs.
    assert n.latest == (newer, newer_depth)
    assert n.last_receive_wall == accepted_wall
    assert sync.queue_size == 8


def test_same_stamp_different_message_identity_uses_the_actually_matched_arrival(monkeypatch):
    n, clock, color, depth, _ = ingress_node(monkeypatch)
    original, replacement = image(10., 20), image(10., 21)
    color.signalMessage(original)
    clock[0] = 10.2
    color.signalMessage(replacement)
    clock[0] = 10.3
    depth.signalMessage(image(10., 30))
    timing = n._latest_frame_timing
    assert n.latest[0] is replacement
    assert timing['color_seq'] == 21
    assert timing['raw_arrivals']['color']['source_age_at_arrival_sec'] == pytest.approx(.2)
    assert timing['raw_arrivals']['color']['until_exact_pair_callback_sec'] == pytest.approx(.1)


def test_latest_raw_delivery_is_frozen_before_later_shared_header_mutation(monkeypatch):
    n, clock, _, _, _ = ingress_node(monkeypatch)
    rgb, depth = image(10., 81), image(10., 82)
    n._raw_frame_arrival(rgb, 'color')
    rgb.header.seq = 999
    # The newest raw delivery is the one passed to TimeSynchronizer.add.
    clock[0] = 10.1
    n._raw_frame_arrival(rgb, 'color')
    n._raw_frame_arrival(depth, 'depth')
    n.frame(rgb, depth)
    assert rgb.header.seq == 999  # diagnostics do not repair/mutate messages
    assert n._latest_frame_timing['color_seq'] == 999
    assert n._latest_frame_timing['raw_arrivals']['color']['arrival_ros_ns'] == module.rospy.Time.from_sec(10.1).to_nsec()
    frozen = n._latest_frame_timing['raw_arrivals']['color'].copy()
    rgb.header.seq = 1234
    assert n._latest_frame_timing['raw_arrivals']['color'] == frozen


def test_mutated_source_stamp_does_not_attach_unrelated_arrival(monkeypatch):
    n, clock, _, _, _ = ingress_node(monkeypatch)
    rgb, depth = image(10., 90), image(10.1, 91)
    n._raw_frame_arrival(rgb, 'color')
    rgb.header.stamp = module.rospy.Time.from_sec(10.1)
    clock[0] = 10.2
    n._raw_frame_arrival(depth, 'depth')
    n.frame(rgb, depth)
    assert n._latest_frame_timing['raw_arrivals']['color'] is None
    assert n._latest_frame_timing['raw_arrival_difference_sec'] is None
    assert n._latest_frame_timing['source_stamp_ns'] == rgb.header.stamp.to_nsec()


def test_bounded_numeric_cache_eviction_never_fabricates_missing_arrival(monkeypatch):
    n, clock, _, _, _ = ingress_node(monkeypatch, capacity=3)
    held = [image(10.+index*.01, index) for index in range(100)]
    for msg in held:
        n._raw_frame_arrival(msg, 'color')
    cache = n._frame_arrivals
    assert len(cache.records['color']) == 3
    assert len(cache.records['depth']) == 0
    assert all(isinstance(value, (int, float)) for row in cache.records['color'].values() for value in row.values())
    missing = cache.pair(held[0], image(10., 200), 11_000_000_000, 111.)
    assert missing['raw_arrivals'] == {'color': None, 'depth': None}
    snapshot = cache.snapshot(11_000_000_000, 111.)
    assert snapshot['cached_arrivals'] == {'color': 3, 'depth': 0}
    assert snapshot['latest_raw_arrivals']['color']['source_seq'] == 99
    snapshot['latest_raw_arrivals']['color']['source_seq'] = 10000
    assert cache.latest['color']['source_seq'] == 99


def test_exact_pair_callback_to_input_lock_wait_is_separate(monkeypatch):
    n, clock, _, _, _ = ingress_node(monkeypatch)
    rgb, depth = image(10., 1), image(10., 2)
    n._raw_frame_arrival(rgb, 'color')
    n._raw_frame_arrival(depth, 'depth')
    class DelayedLock:
        def __enter__(self): clock[0] += .25
        def __exit__(self, *_): pass
    n.input_lock = DelayedLock()
    n.frame(rgb, depth)
    t = n._latest_frame_timing
    assert t['exact_pair_source_age_sec'] == pytest.approx(.01)
    assert t['input_lock_wait_sec'] == pytest.approx(.25)
    assert t['exact_pair_to_input_slot_sec'] == pytest.approx(.25)
    assert t['ingress_source_age_sec'] == pytest.approx(.26)
    assert t['raw_arrival_difference_sec'] == pytest.approx(0.)


def test_delay_after_both_raw_callbacks_is_visible_as_pairing_residence(monkeypatch):
    n, clock, _, _, _ = ingress_node(monkeypatch)
    rgb, depth = image(10., 1), image(10., 2)
    n._raw_frame_arrival(rgb, 'color')
    clock[0] = 10.03
    n._raw_frame_arrival(depth, 'depth')
    clock[0] = 10.73
    n.frame(rgb, depth)
    t = n._latest_frame_timing
    assert t['raw_arrival_difference_sec'] == pytest.approx(.02)
    assert t['after_latest_raw_arrival_sec'] == pytest.approx(.7)
    assert t['input_lock_wait_sec'] == pytest.approx(0.)


def test_watchdog_without_source_keeps_recent_raw_arrivals_and_original_loss(monkeypatch):
    n = diagnostic_node(monkeypatch)
    n._frame_arrivals = module.RgbdIngressTiming()
    n._frame_arrivals.record('color', image(9., 51), 9_100_000_000, 101.)
    n.unavailable('exact_rgbd_stream_stale')
    result = json.loads(n.pubs['detector_status'].publish.call_args[0][0].data)
    assert result['first_loss']['reason'] == 'exact_rgbd_stream_stale'
    assert result['first_loss']['source_pair_available'] is False
    assert 'frame_timing' not in result
    timing = result['rgbd_ingress_timing']
    assert timing['latest_raw_arrivals']['color']['source_seq'] == 51
    assert timing['latest_raw_arrivals']['color']['source_age_at_status_sec'] == pytest.approx(1.)
    assert timing['latest_raw_arrivals']['depth'] is None
    json.dumps(result, allow_nan=False)


def test_recycled_message_identity_and_equal_stamp_refresh_latest_delivery(monkeypatch):
    # Deterministically emulate Python reusing an id after the old Image is
    # released, instead of relying on allocator behavior in a particular run.
    monkeypatch.setattr(module, 'id', lambda _message: 101, raising=False)
    cache = module.RgbdIngressTiming()
    old = image(10., 11)
    cache.record('color', old, 10_100_000_000, 110.1)
    del old
    replacement = image(10., 22)
    cache.record('color', replacement, 10_700_000_000, 110.7)
    paired = cache.pair(replacement, image(10., 33), 10_800_000_000, 110.8)
    raw = paired['raw_arrivals']['color']
    assert raw['source_seq'] == 22
    assert raw['arrival_ros_ns'] == 10_700_000_000
    assert raw['until_exact_pair_callback_sec'] == pytest.approx(.1)
    assert len(cache.records['color']) == 1
    assert all(isinstance(value, (int, float)) for value in cache.records['color'][(101, 10_000_000_000)].values())


def test_repeated_delivery_updates_bounded_cache_recency():
    cache = module.RgbdIngressTiming(capacity=2)
    first, second, third = (image(10.+index, index) for index in range(3))
    cache.record('color', first, 10_000_000_000, 110.)
    cache.record('color', second, 11_000_000_000, 111.)
    cache.record('color', first, 12_000_000_000, 112.)
    cache.record('color', third, 13_000_000_000, 113.)
    assert len(cache.records['color']) == 2
    assert cache.pair(first, first, 14_000_000_000, 114.)['raw_arrivals']['color']['arrival_ros_ns'] == 12_000_000_000
    assert cache.pair(second, second, 14_000_000_000, 114.)['raw_arrivals']['color'] is None
