"""Actual remote callbacks after arrival release; no ROS or robot transport.

The producer owns the .8-second current-frame gate.  This file proves that
ordinary released-session consumption does not independently retire the old
identity because its preceding ready source aged during current processing.
"""
from copy import deepcopy

import numpy as np
import pytest

import test_observation_measurement_remote as measurement
from test_remote_modes import direct_observation
from test_unknown_pending_identity import deliver
from alicia_flexible_grasp.vision.multiview_surface import (
    fused_surface_from_view, surface_view_from_observation,
)


@pytest.fixture
def released(monkeypatch):
    c = measurement.context.__wrapped__(monkeypatch)
    req = measurement.arrival(c)
    row, sources = measurement.fresh(c, req)
    for key in ('object', 'mask', 'status', 'ack'):
        measurement.deliver_one(c, row, key)
    assert measurement.qualified_ack(c)[-1]['source_stamps_ns'] == sources
    done = dict(req, state='qualified', request_id='done',
                stamp_ns=c.node._ros_source_clock_ns(), source_stamps_ns=sources)
    assert measurement.request(c.node, done)['accepted']
    assert not c.node._measurement_remote_active()
    # A real measured-surface value, not a fake authority or replacement
    # registration.  Saved phase evidence must survive normal source renewal.
    view = surface_view_from_observation(direct_observation(c.node, sources[-1]))
    surface = fused_surface_from_view(view)
    c.node._observe_return_saved = (surface, view)
    c.saved = c.node._observe_return_saved
    c.row = row
    c.reference_ns = sources[-1]
    c.now_ns = [c.node._ros_source_clock_ns()]
    c.node._ros_source_clock_ns = lambda: c.now_ns[0]
    c.node.frames._source_clock_ns = lambda: c.now_ns[0]
    monkeypatch.setattr(measurement.module.rospy.Time, 'now',
                        lambda: measurement.at(c.now_ns[0]))
    return c


def normal_frame(c):
    # Exact recorded gen24 differences, translated to this isolated clock.
    reference = 1791543687167207002
    source = 1791543687661621093
    observed = 1791543687995831489
    ns = c.reference_ns + source - reference
    c.now_ns[0] = c.reference_ns + observed - reference
    c.clock[0] = c.now_ns[0] / 1e9
    row = deepcopy(c.row)
    for key in ('object', 'mask'):
        row[key].header.stamp = measurement.at(ns)
    for key in ('source_stamp_ns', 'color_stamp_ns', 'depth_stamp_ns', 'tf_lookup_stamp_ns'):
        row['status'][key] = ns
    row['status'].update(state='ready', detail='tracked')
    row['status']['observation_measurement']['state'] = 'completed'
    row['status']['observation_measurement']['request'] = deepcopy(
        c.node._measurement_remote_request)
    assert c.now_ns[0] - c.reference_ns > 800_000_000
    assert 0 < c.now_ns[0] - ns < 800_000_000
    return row, ns


def fill_pixels(c, row, ns):
    shape = c.node.bridge.imgmsg_to_cv2(row['mask'], desired_encoding='mono8').shape
    # Pixel content is a deterministic fixture, not a gen24 RGB reconstruction.
    c.node.frames.update_color(np.full(shape + (3,), 30, np.uint8),
                              measurement.at(ns), row['mask'].header.frame_id)
    c.node.frames.update_depth(np.full(shape, .35, np.float32),
                              measurement.at(ns), row['mask'].header.frame_id)


def exact_sample(c, ns):
    return c.node.frames.wait_for_exact_sample(ns, c.identity, 0., True,
        c.node.planning_snapshot_max_age_sec,
        c.node.planning_snapshot_max_inference_latency_sec)


def test_fresh_normal_source_after_old_ready_expiry_preserves_identity_and_surface(released):
    c = released
    row, ns = normal_frame(c)
    previous_time = c.node.latest_object_time
    assert c.node._unknown_identity_pending is None
    assert not c.node._unknown_identity_scope()  # Post-arrival session, not initial gap.
    assert c.node._poll_stream_snapshot() is False  # Original paused phase.
    assert c.node._current_stream_target_identity() == c.identity
    assert c.node.latest_object.header.stamp.to_nsec() == c.reference_ns
    assert c.node.latest_object_time == previous_time  # No fake freshness renewal.
    fill_pixels(c, row, ns)
    deliver(c.node, row)
    assert c.node._current_stream_target_identity() == c.identity
    assert c.node._observe_return_saved is c.saved
    assert c.saved[0].view_stamps_ns == (c.reference_ns,)
    assert c.node.latest_object.header.stamp.to_nsec() == ns
    assert c.node.latest_object.detected
    sample = exact_sample(c, ns)
    assert sample is not None and sample.stamp_ns == ns
    assert sample.target_identity == c.identity
    assert c.node._observe_return_paused and c.node._poll_stream_snapshot() is False
    assert c.node._unknown_identity_pending is None


def test_current_source_still_faces_original_exact_buffer_latency_gate(released):
    c = released
    row, ns = normal_frame(c)
    c.now_ns[0] = ns + round(c.node.planning_snapshot_max_inference_latency_sec * 1e9) + 1
    fill_pixels(c, row, ns)
    deliver(c.node, row)
    # Object association does not certify input age or open planning authority.
    assert c.node._current_stream_target_identity() == c.identity
    assert exact_sample(c, ns) is None
    assert c.node._observe_return_paused and c.node._poll_stream_snapshot() is False


def test_real_false_after_qualified_retires_session_and_late_true_cannot_revive_it(released):
    c = released
    row, ns = normal_frame(c)
    absent = deepcopy(row['object'])
    absent.detected = False
    c.node.object_cb(absent)
    retired = c.node._current_stream_target_identity()
    assert retired != c.identity
    assert c.node._observe_return_phase == 'identity_lost'
    assert c.node._observe_return_session is None
    assert c.node._observe_return_saved is None
    assert c.node._active_multiview_surface() == (None, None)
    assert c.node._observe_return_paused
    fill_pixels(c, row, ns)
    deliver(c.node, row)
    assert c.node._current_stream_target_identity() != c.identity
    assert c.node._observe_return_phase == 'identity_lost'
    assert c.node._observe_return_session is None
    assert c.node._observe_return_saved is None
    assert c.node._active_multiview_surface() == (None, None)
    assert exact_sample(c, ns) is None  # The retired identity cannot borrow it.
    assert c.node._poll_stream_snapshot() is False
