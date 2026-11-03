"""Timely captured recovery may finish after old geometry loses authority."""
from copy import deepcopy
import itertools
from types import SimpleNamespace as NS

import numpy as np
import pytest
from std_msgs.msg import Header

from alicia_grasp_modes.initial_station_identity_completion import (
    completion_contract, station_from_marker, validate_completion)
from alicia_grasp_modes.unknown_pending_identity import validate_support_receipt
from test_locked_rgb_hold import locked, custom_frame, latest_status, module
from test_support_measurement_hold import support_gap_depth
from test_unknown_pending_identity import consumer_for, publication, deliver


@pytest.fixture
def gap(monkeypatch):
    p, clock, feed, joint, params = locked(monkeypatch, initial_station=True)
    n = consumer_for(p, clock, params)
    ready = publication(p)
    deliver(n, ready)
    ref = p._locked_rgb_hold_reference
    custom_frame(p, clock, joint, depth_image=support_gap_depth())
    row = publication(p)
    assert row['status']['source_qualification']['identity_completion']['version'] == 1
    return NS(p=p, n=n, clock=clock, feed=feed, joint=joint, params=params,
              ref=ref, ready=ready, gap=row)


def frame_after_gap(s, monkeypatch, *, capture_gap=.698523521,
                    final_old_age=.835039377, change=None):
    ref_ns = s.ref['source_stamp_ns']
    source_ns = ref_ns+round(capture_gap*1e9)
    # Actual producer process; only transport/source-clock placement is synthetic.
    # Use the actual acquired depth/RGB, not the failed support-gap image.
    source_pair = s.p.bridge
    rgb = s.ref['bgr'].copy()
    depth = np.full(rgb.shape[:2], .4, np.float32)
    depth[106:134,148:173] = .35
    pair = (source_pair.cv2_to_imgmsg(rgb, 'bgr8'), source_pair.cv2_to_imgmsg(depth, '32FC1'))
    stamp = module.rospy.Time(source_ns//10**9, source_ns%10**9)
    for msg in pair:
        msg.header.stamp = stamp
        msg.header.frame_id = s.p.camera_frame
    s.clock[0] = source_ns/1e9+.018
    s.joint(source_ns/1e9-.01); s.joint(source_ns/1e9+.01)
    original = module.segment_with_support_fallback
    def segment(*a, **k):
        result = original(*a, **k)
        s.clock[0] = ref_ns/1e9+final_old_age
        if change == 'manual': s.params['/gui/joint_direct_mode'] = True
        elif change == 'config': s.p.config.support_threshold_m = .0019
        elif change == 'camera': s.params['/camera']['fx'] += 1
        elif change == 'selection': s.params['/grasp_mode/selection']['generation'] += 1
        elif change == 'epoch':
            s.p._rgb_interest_epoch_cb(Header(stamp=module.rospy.Time(2), frame_id='sdk_reference_epoch'))
        elif change == 'q':
            s.joint(s.clock[0], [.006, 0., 0., 0., 0., 0.])
            s.joint(s.clock[0]+.001, [0.]*6)
        return result
    monkeypatch.setattr(module, 'segment_with_support_fallback', segment)
    s.completed_source = pair
    s.p.latest = pair
    s.p.process(None)
    return publication(s.p)


@pytest.mark.parametrize('order', list(itertools.permutations(('mask', 'object', 'status'))))
def test_actual_pipeline_and_consumer_complete_gen20_timing_without_refreshing_reference(gap, monkeypatch, order):
    s = gap
    deliver(s.n, s.gap, order)
    identity = s.n._current_stream_target_identity()
    pending = s.n._unknown_identity_pending
    geometry_deadline = pending['deadline_ns']
    completion = deepcopy(pending['identity_completion'])
    assert geometry_deadline == s.ref['source_stamp_ns']+800_000_000
    assert completion['identity_completion_deadline_ns'] == s.ref['source_stamp_ns']+1_600_000_000
    recovered = frame_after_gap(s, monkeypatch)
    assert recovered['object'].detected, recovered['status']
    receipt = recovered['status']['source_qualification']
    assert receipt['identity_completion'] == completion
    assert receipt['reference_source_stamp_ns'] == s.ref['source_stamp_ns']
    assert s.clock[0]*1e9 > geometry_deadline
    assert s.n._unknown_identity_pending['deadline_ns'] == geometry_deadline
    assert s.n._unknown_identity_poll()
    assert not s.n.latest_object.detected and not s.n.geometry_pub.messages[-1].valid
    deliver(s.n, recovered, order)
    assert s.n.latest_object.detected and s.n._unknown_identity_pending is None
    assert s.n._current_stream_target_identity() == identity
    ns = recovered['status']['source_stamp_ns']
    assert s.n._unknown_identity_reference['source_ns'] == ns
    assert s.n._unknown_identity_reference['deadline_ns'] == ns+800_000_000
    assert 'identity_completion' not in s.n._unknown_identity_reference
    assert not s.n.rich_plan_pub.messages[-1].valid


def test_real_integer_gen20_timestamps_and_configured_shorter_maximum(gap):
    marker = gap.ref['initial_station_current_support']
    station = station_from_marker(marker)
    template = marker['identity_template']['provenance']
    ref = 1791535531442678928; source = 1791535532141202449; now = 1791535532277718305
    contract = completion_contract(ref, .8, template, station)
    assert validate_completion(contract, ref, .8, template, station, now, source) == ref+1_600_000_000
    assert source-ref == 698523521 and now-source == 136515856 and now-ref == 835039377
    shorter = completion_contract(ref, .4, template, station)
    assert shorter['capture_deadline_ns'] == ref+400_000_000
    assert shorter['identity_completion_deadline_ns'] == ref+800_000_000
    validate_completion(shorter, ref, .4, template, station, ref+500_000_000, ref+350_000_000)


@pytest.mark.parametrize('which', ['capture', 'current', 'completion', 'future', 'reference', 'permission_int'])
def test_three_independent_clock_bounds_and_typed_authority(gap, which):
    c = deepcopy(gap.gap['status']['source_qualification']['identity_completion'])
    ref = gap.ref['source_stamp_ns']; source = ref+700_000_000; now = ref+850_000_000
    if which == 'capture': source = ref+800_000_000
    elif which == 'current': now = source+800_000_000
    elif which == 'completion': now = ref+1_600_000_000; source = None
    elif which == 'future': source = ref+750_000_000; now = ref+700_000_000
    elif which == 'reference': source = ref
    else: c['motion_authorized'] = 0
    marker = gap.ref['initial_station_current_support']
    with pytest.raises(ValueError):
        validate_completion(c, ref, .8, marker['identity_template']['provenance'],
                            station_from_marker(marker), now, source)


@pytest.mark.parametrize('change', ['manual', 'config', 'camera', 'selection', 'epoch', 'q'])
def test_completion_does_not_survive_station_context_or_epoch_change(gap, monkeypatch, change):
    recovered = frame_after_gap(gap, monkeypatch, change=change)
    assert not recovered['object'].detected
    assert gap.p.last_good_source_stamp_ns == gap.ref['source_stamp_ns']


@pytest.mark.parametrize('capture,complete', [(.8,.9), (.698523521,1.51), (.698523521,1.61)])
def test_producer_cannot_publish_late_captured_or_late_completed_measurement(gap, monkeypatch, capture, complete):
    recovered = frame_after_gap(gap, monkeypatch, capture_gap=capture, final_old_age=complete)
    assert not recovered['object'].detected
    assert gap.p.last_good_source_stamp_ns == gap.ref['source_stamp_ns']


def test_repeated_pending_and_no_frame_never_roll_completion_deadline(gap):
    s = gap
    deliver(s.n, s.gap)
    original = deepcopy(s.n._unknown_identity_pending['identity_completion'])
    s.clock[0] += .15
    custom_frame(s.p, s.clock, s.joint, depth_image=support_gap_depth())
    later = publication(s.p)
    deliver(s.n, later)
    assert s.n._unknown_identity_pending['identity_completion'] == original
    assert later['status']['source_qualification']['identity_completion'] == original
    s.clock[0] = original['identity_completion_deadline_ns']/1e9
    s.n._unknown_identity_poll()
    assert s.n._unknown_identity_pending is None and not s.n.latest_object.detected
    s.p.latest = None
    s.p.process(None)
    assert s.p.tracker.lost


def test_missing_receipt_keeps_original_strict_geometry_deadline(gap):
    row = deepcopy(gap.gap)
    row['status']['source_qualification'].pop('identity_completion')
    deliver(gap.n, row)
    assert 'identity_completion' not in gap.n._unknown_identity_pending
    gap.clock[0] = gap.n._unknown_identity_pending['deadline_ns']/1e9+.001
    gap.n._unknown_identity_poll()
    assert gap.n._unknown_identity_pending is None and not gap.n.latest_object.detected


def test_no_pending_normal_tracking_cannot_use_completion(monkeypatch):
    p, clock, feed, _, _ = locked(monkeypatch, initial_station=True)
    assert p._locked_source_pending is None
    clock[0] += .6
    feed(target_depth=.35)
    assert not p.pubs['object'].publish.call_args[0][0].detected
    assert p.tracker.lost


@pytest.mark.parametrize('change', ['missing_station', 'changed_template', 'changed_deadline', 'future_capture'])
def test_consumer_rejects_unbound_or_rewritten_completion_receipt(gap, change):
    row = deepcopy(gap.gap)
    q = row['status']['source_qualification']
    if change == 'missing_station':
        gap.n._unknown_identity_reference.pop('initial_station_measurement')
        gap.n._unknown_identity_status[gap.ref['source_stamp_ns']].pop('initial_station_measurement')
    elif change == 'changed_template': q['identity_completion']['identity_template_source']['source_rgb_sha256'] = 'f'*64
    elif change == 'changed_deadline': q['identity_completion']['identity_completion_deadline_ns'] += 1
    else: q['identity_completion']['capture_deadline_ns'] += 1
    deliver(gap.n, row)
    assert gap.n._unknown_identity_pending is None and not gap.n.latest_object.detected


@pytest.mark.parametrize('object_before_deadline', [False, True])
def test_late_first_completion_receipt_cannot_enroll_or_revive_retired_identity(gap, monkeypatch, object_before_deadline):
    s = gap
    if object_before_deadline:
        deliver(s.n, s.gap, ('object', 'mask'))
        assert s.n._unknown_identity_pending is not None
        assert 'identity_completion' not in s.n._unknown_identity_pending
    recovered = frame_after_gap(s, monkeypatch)
    assert recovered['object'].detected
    assert s.clock[0]*1e9 > s.ref['source_stamp_ns']+800_000_000
    deliver(s.n, s.gap, ('status',) if object_before_deadline else ('mask', 'object', 'status'))
    assert not s.n.latest_object.detected
    deliver(s.n, recovered)
    assert not s.n.latest_object.detected
    assert s.n._unknown_identity_reference is None
    assert not s.n.rich_plan_pub.messages[-1].valid


from test_observation_recovery_history import recovered as previously_recovered, fuse


def test_new_geometry_deadline_allows_existing_history_admission_after_extended_completion(previously_recovered, monkeypatch):
    old = previously_recovered
    n, p = old.node, old.producer
    ref = p._locked_rgb_hold_reference
    old.clock[0] += .15
    custom_frame(p, old.clock, old.joint, depth_image=support_gap_depth())
    deliver(n, publication(p))
    assert n._observation_history_gap is not None
    s = NS(p=p, n=n, clock=old.clock, joint=old.joint, params=old.params, ref=ref)
    recovered = frame_after_gap(s, monkeypatch)
    assert recovered['object'].detected
    # The actual new source image/mask, never the previous geometry, enters
    # the original buffer/fuser after the full typed recovery has completed.
    colour, depth = s.completed_source
    n.frames.update_color(p.bridge.imgmsg_to_cv2(colour, 'bgr8'), colour.header.stamp, colour.header.frame_id)
    n.frames.update_depth(p.bridge.imgmsg_to_cv2(depth, 'passthrough'), depth.header.stamp, depth.header.frame_id)
    deliver(n, recovered)
    ns = recovered['status']['source_stamp_ns']
    assert n._observation_history_recovery['source_ns'] == ns
    assert n._unknown_identity_reference['deadline_ns'] == ns+800_000_000
    samples = n.frames.wait_for_samples(5, 0., require_mask=True,
        max_age_sec=.35, collection_span_sec=12., max_inference_latency_sec=3.5,
        target_identity=n._current_stream_target_identity())
    snapshot = fuse(samples)
    assert snapshot.ok and n.submit_stream_snapshot(snapshot)
    assert n._stream_worker_ticket.request_id != old.old_ticket.request_id
    assert n._stream_ticket_cancellation_code_locked(old.old_ticket) == 'TARGET_MEASUREMENT_PENDING'
    assert not n.rich_plan_pub.messages[-1].valid
