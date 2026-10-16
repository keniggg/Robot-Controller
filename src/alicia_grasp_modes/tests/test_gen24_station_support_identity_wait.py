"""Real three-owner support recovery across the old geometry lifetime.

RGB-D and transport are synthetic; producer segmentation, identity proof,
consumer fan-in and all control/clock gates are the production methods.
"""
from copy import deepcopy

import pytest

from test_station_requalification_three_owners import owners
from test_station_support_gap_three_owners import wire
from test_unknown_pending_identity import publication
from test_support_measurement_hold import support_gap_depth
from test_locked_rgb_hold import custom_frame


def pending(c):
    c.clock[0] += .15
    custom_frame(c.producer, c.clock, c.joint, depth_image=support_gap_depth())
    row = publication(c.producer)
    assert row['status']['detail'] == 'support_measurement_pending_revalidation'
    wire(c, row, ('status', 'object', 'mask'))
    return row


def assert_waiting(c, parent, clocks):
    p, r, t = c.producer, c.node, c.task
    assert p.first_loss is None and not p.tracker.lost
    assert r._station_compute_pending and not r._station_compute_retired
    assert t._station_task_record['pending'] and not t._station_task_retired
    assert t._unknown_target_loss is None
    assert p.last_good_source_stamp_ns == parent['source_ns']
    assert p._station_compute_registration['contract'] == parent
    assert r._station_compute_reference['contract'] == parent
    assert t._station_task_record['contract'] == parent
    assert clocks == (p._station_compute_registration['monotonic_deadline'],
        r._station_compute_reference['identity_monotonic'], t._station_task_record['identity_mono'])
    assert t.latest_obj is None
    assert not getattr(r.latest_object_geometry, 'valid', False)
    assert not getattr(r.latest_preview_rich_plan, 'valid', False)


@pytest.mark.parametrize('order', [('status', 'object', 'mask'), ('object', 'mask', 'status')])
def test_timely_frame_crosses_old_geometry_deadline_during_tf_then_real_join_recovers(monkeypatch, order):
    c = owners(monkeypatch)
    p, r, t = c.producer, c.node, c.task
    parent = deepcopy(c.ready['status']['station_compute_wait'])
    original_deadline = parent['geometry_deadline_ns']
    delayed = pending(c)
    old_identity = r._current_stream_target_identity()
    c.clock[0] = original_deadline / 1e9 - .057
    lookup = p.buffer.lookup_transform
    waits = []
    def cross_old_deadline(*args, **kwargs):
        result = lookup(*args, **kwargs)
        if not waits:
            c.clock[0] += .058
            c.joint(c.clock[0] - .001)
            waits.append(True)
        return result
    with monkeypatch.context() as changed:
        changed.setattr(p.buffer, 'lookup_transform', cross_old_deadline)
        pair = custom_frame(p, c.clock, c.joint, target_depth=.35)
    ready = publication(p)
    assert waits
    assert ready['status']['state'] == 'ready', ready['status']
    assert p.first_loss is None and p.last_good_source_stamp_ns == pair[0].header.stamp.to_nsec()
    assert pair[0].header.stamp.to_nsec() < original_deadline < round(c.clock[0] * 1e9)
    assert round(c.clock[0] * 1e9) - pair[0].header.stamp.to_nsec() < 800_000_000
    assert ready['status']['station_compute_recovery'] == parent
    assert ready['status']['station_support_gap_recovery'] == delayed['status']['station_support_gap']
    for key in order[:-1]:
        wire(c, ready, (key,))
        assert r._station_compute_pending and t._station_task_record['pending']
    wire(c, ready, order[-1:])
    assert not r._station_compute_retired and not t._station_task_retired
    assert not r._station_compute_pending and not t._station_task_record['pending']
    assert r._current_stream_target_identity() == old_identity
    assert r.latest_object.header.stamp == pair[0].header.stamp
    assert t.latest_obj.header.stamp == pair[0].header.stamp
    assert not getattr(r.latest_preview_rich_plan, 'valid', False)
    wire(c, delayed, order)
    assert not r._station_compute_retired and not t._station_task_retired
    assert not r._station_compute_pending and not t._station_task_record['pending']


def test_repeated_fresh_missing_support_after_geometry_expiry_does_not_slide_identity(monkeypatch):
    c = owners(monkeypatch)
    p, r, t = c.producer, c.node, c.task
    parent = deepcopy(c.ready['status']['station_compute_wait'])
    clocks = (p._station_compute_registration['monotonic_deadline'],
        r._station_compute_reference['identity_monotonic'], t._station_task_record['identity_mono'])
    first = pending(c)['status']['station_support_gap']
    for offset in (1.1, 2.1, 3.2):
        c.clock[0] = parent['source_ns'] / 1e9 + offset
        custom_frame(p, c.clock, c.joint, depth_image=support_gap_depth())
        row = publication(p)
        assert row['status']['detail'] == 'support_measurement_pending_revalidation', row['status']
        receipt = row['status']['station_support_gap']
        assert receipt['deadline_ns'] == first['deadline_ns'] == parent['identity_deadline_ns']
        assert receipt['first_source_ns'] == first['first_source_ns']
        wire(c, row, ('object', 'mask', 'status'))
        assert_waiting(c, parent, clocks)
    c.clock[0] += .2
    pair = custom_frame(p, c.clock, c.joint, target_depth=.35)
    row = publication(p)
    assert row['status']['state'] == 'ready', row['status']
    wire(c, row, ('object', 'mask', 'status'))
    assert r.latest_object.header.stamp == pair[0].header.stamp
    assert t.latest_obj.header.stamp == pair[0].header.stamp
    assert not r._station_compute_retired and not t._station_task_retired


@pytest.mark.parametrize('fault', ['manual', 'epoch', 'out_and_back', 'rgb'])
def test_identity_wait_does_not_accept_real_owner_motion_or_appearance_failure(monkeypatch, fault):
    c = owners(monkeypatch)
    p = c.producer
    parent = deepcopy(c.ready['status']['station_compute_wait'])
    pending(c)
    c.clock[0] = parent['source_ns'] / 1e9 + 1.1
    if fault == 'manual':
        c.params['/gui/joint_direct_mode'] = True
    elif fault == 'epoch':
        p._rgb_interest_epoch += 1
    elif fault == 'out_and_back':
        c.joint(c.clock[0] - .02, [.005, 0., 0., 0., 0., 0.])
        c.joint(c.clock[0] - .01)
    custom_frame(p, c.clock, c.joint, target_depth=.35, rgb_missing=fault == 'rgb')
    row = publication(p)
    assert row['status']['state'] != 'ready'
    assert p.last_good_source_stamp_ns == parent['source_ns']
    assert not row['object'].detected
