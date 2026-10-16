"""Actual producer receipts + unchanged five-frame fusion, without ROS services."""
from copy import deepcopy
from dataclasses import replace
from types import SimpleNamespace as NS

import numpy as np
import pytest

from alicia_grasp_modes.observation_recovery_history import station_binding
from alicia_flexible_grasp.vision.rgbd_snapshot import fuse_stable_samples
from test_locked_rgb_hold import custom_frame
from test_support_measurement_hold import support_gap_depth
from test_unknown_pending_identity import consumer_for, publication, deliver, base
from test_unknown_stream import rgb_interest_node


def fuse(samples):
    return fuse_stable_samples(samples, require_mask=True, min_mask_iou=.85,
        max_centroid_shift_px=5., max_joint_delta_rad=.01, erosion_px=0,
        depth_scale=1., depth_min_m=.03, depth_max_m=2., mad_scale=3.5,
        mad_absolute_floor_m=.002, internal_hole_max_area_px=0)


@pytest.fixture
def recovered(monkeypatch):
    producer, clock, feed, joint, params = rgb_interest_node(monkeypatch)
    for _ in range(3):
        pair = feed(target_depth=.35)
        clock[0] += .2
    node = consumer_for(producer, clock, params)
    node._observe_return_phase = 'observation_prefix'
    node._observe_return_trigger = 'BILATERAL_SURFACE_EVIDENCE_MISSING'
    node.near_field_planning_active = False
    node.planning_snapshot_frames = 5
    node.planning_snapshot_max_span_sec = 12.
    node.frames.update_joints(np.zeros(6))

    def capture(pair):
        colour, depth = pair
        node.frames.update_color(producer.bridge.imgmsg_to_cv2(colour, 'bgr8'),
                                 colour.header.stamp, colour.header.frame_id)
        node.frames.update_depth(producer.bridge.imgmsg_to_cv2(depth, 'passthrough'),
                                 depth.header.stamp, depth.header.frame_id)
        deliver(node, publication(producer))
        return node.frames.wait_for_samples(5, 0., require_mask=True,
            max_age_sec=.35, collection_span_sec=12., max_inference_latency_sec=3.5,
            target_identity=node._current_stream_target_identity())

    capture(pair)
    for _ in range(4):
        clock[0] += .15
        pair = feed(target_depth=.35)
        samples = capture(pair)
    before = fuse(samples)
    assert before.ok and len(node._observation_history_ready) == 5
    assert node.submit_stream_snapshot(before)
    old_ticket = node._stream_worker_ticket
    old_revision = node._capture_geometry_generation()
    node.inference_coordinator.complete(old_ticket, now_sec=clock[0],
                                       target_epoch=node.target_instance_epoch)
    node._stream_worker_ticket = None
    clock[0] += .15
    pair = custom_frame(producer, clock, joint, depth_image=support_gap_depth())
    gap_ns = pair[0].header.stamp.to_nsec()
    capture(pair)
    assert node._unknown_identity_pending is not None
    clock[0] += .15
    pair = feed(target_depth=.35)
    samples = capture(pair)
    snapshot = fuse(samples)
    assert snapshot.ok and node._unknown_identity_pending is None
    return NS(node=node, producer=producer, clock=clock, feed=feed, joint=joint,
              params=params, snapshot=snapshot, samples=samples, before=before,
              old_ticket=old_ticket, old_revision=old_revision, gap_ns=gap_ns, capture=capture)


def test_real_recovery_reuses_five_qualified_samples_but_creates_new_ticket(recovered):
    s, n = recovered, recovered.node
    assert len(s.snapshot.sample_stamp_ns) == 5
    assert s.gap_ns not in s.snapshot.sample_stamp_ns
    assert any(ns <= n._unknown_identity_cancelled_through_ns for ns in s.snapshot.sample_stamp_ns)
    assert n._ros_source_clock_ns()-s.snapshot.sample_stamp_ns[0] > 800_000_000
    assert n._observation_history_recovery is not None
    assert n.submit_stream_snapshot(s.snapshot)
    new = n._stream_worker_ticket
    assert new.request_id != s.old_ticket.request_id
    assert new.payload[0] is s.snapshot
    assert n._capture_geometry_generation() > s.old_revision
    assert n._stream_ticket_cancellation_code_locked(s.old_ticket) == 'TARGET_MEASUREMENT_PENDING'
    assert n._stream_ticket_cancellation_code_locked(new) == ''
    assert not n.rich_plan_pub.messages[-1].valid


@pytest.mark.parametrize('change', ['contact', 'direct', 'diagnostic', 'no_receipt', 'missing_ready',
    'model', 'config', 'camera', 'manual', 'wrong_station', 'late_admission',
    'old_latest', 'expired_latest', 'not_fused', 'false', 'new_pending', 'epoch', 'wrong_mask',
    'history_false', 'history_empty_mask', 'history_not_ready', 'history_mask_changed', 'not_entered',
    'fusion_history_hash', 'missing_fusion_hash'])
def test_history_never_bypasses_scope_current_source_or_provenance(recovered, change):
    s, n = recovered, recovered.node
    snapshot = s.snapshot
    records = n._observation_history_ready
    if change == 'contact': n._observe_return_phase = 'initial_contact'
    elif change == 'direct': n._mode_selection['strategy'] = 'direct'
    elif change == 'diagnostic': n._observe_return_diagnostic_observation_only = True
    elif change == 'no_receipt': n._observation_history_recovery = None
    elif change == 'missing_ready': records.pop(snapshot.sample_stamp_ns[0])
    elif change == 'model': s.params['/robot_description'] += ' '
    elif change == 'config': s.params['/unknown_perception']['support_min_ratio'] = .71
    elif change == 'camera': s.params['/camera']['fx'] += 1.
    elif change == 'manual': s.params['/gui/joint_direct_mode'] = True
    elif change == 'wrong_station': records[snapshot.sample_stamp_ns[0]]['station'] += 'x'
    elif change == 'late_admission':
        row = records[snapshot.sample_stamp_ns[0]]
        row['admitted_ns'] = row['freshness_deadline_ns']
    elif change == 'old_latest': snapshot = s.before
    elif change == 'expired_latest': s.clock[0] += .81
    elif change == 'not_fused': snapshot = replace(snapshot, quality=replace(snapshot.quality, fused_frames=1))
    elif change == 'false':
        obj = deepcopy(snapshot.object_msg); obj.detected = False
        snapshot = replace(snapshot, object_msg=obj)
    elif change == 'new_pending': n._unknown_identity_pending = {'blocked': True}
    elif change == 'epoch': n.target_instance_epoch += 1
    elif change == 'wrong_mask':
        snapshot = replace(snapshot, original_sample=replace(snapshot.original_sample,
            object_mask=np.zeros_like(snapshot.original_sample.object_mask)))
    elif change == 'history_false': n._unknown_identity_objects[snapshot.sample_stamp_ns[0]].detected = False
    elif change == 'history_empty_mask':
        ns = snapshot.sample_stamp_ns[0]
        row = n._unknown_identity_masks[ns]
        n._unknown_identity_masks[ns] = (False,) + row[1:]
    elif change == 'history_not_ready': n._unknown_identity_status[snapshot.sample_stamp_ns[0]]['state'] = 'unavailable'
    elif change == 'history_mask_changed':
        ns = snapshot.sample_stamp_ns[0]
        row = n._unknown_identity_masks[ns]
        n._unknown_identity_masks[ns] = row[:2] + ('f'*64,) + row[3:]
    elif change == 'not_entered': n._observation_history_entered = None
    elif change == 'fusion_history_hash':
        snapshot = replace(snapshot, sample_mask_sha256=('f'*64,) + snapshot.sample_mask_sha256[1:])
    elif change == 'missing_fusion_hash': snapshot = replace(snapshot, sample_mask_sha256=())
    assert not n.submit_stream_snapshot(snapshot)
    assert not n.rich_plan_pub.messages[-1].valid


def test_history_admission_keeps_original_fusion_rejection(recovered):
    samples = deepcopy(recovered.samples)
    samples[-1] = replace(samples[-1], object_mask=np.roll(samples[-1].object_mask, 30, axis=1))
    bad = fuse(samples)
    assert not bad.ok
    assert not recovered.node.submit_stream_snapshot(bad)


@pytest.mark.parametrize('field', ['station_source_stamp_ns', 'epoch_ns', 'source_id',
                                 'refined_mask_sha256', 'source_stamp_ns', 'authority'])
def test_station_receipt_must_match_pinned_acquisition_and_real_current_mask(recovered, field):
    n = recovered.node
    ns = recovered.snapshot.stamp_ns
    status = deepcopy(n._unknown_identity_status[ns])
    receipt = status['initial_station_measurement']
    if field in ('source_id', 'refined_mask_sha256'): receipt[field] = 'f'*64
    elif field == 'authority': receipt['motion_authorized'] = True
    else: receipt[field] += 1
    with pytest.raises(ValueError):
        station_binding(status, n._unknown_identity_reference,
                        n._unknown_identity_masks[ns], n._ros_source_clock_ns())


def test_repeated_ready_callback_cannot_rewrite_historical_admission_time(recovered):
    n = recovered.node
    ns = recovered.snapshot.stamp_ns
    original = deepcopy(n._observation_history_ready[ns])
    recovered.clock[0] += .1
    n._capture_observation_history_ready(n._unknown_identity_status[ns],
        n._unknown_identity_reference, n._unknown_identity_masks[ns], n._ros_source_clock_ns())
    assert n._observation_history_ready[ns] == original
