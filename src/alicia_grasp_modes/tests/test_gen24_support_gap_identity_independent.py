"""Independent GEN23 regression: fresh support recovery after old geometry expires.

Only clocks, ROS transport and camera pixels are synthetic in owner tests.
Producer segmentation, support validation, publication and both consumer joins
are real. The wire fixture preserves the failed experiment's exact integers.
"""
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
import numpy as np

from alicia_grasp_modes.station_compute_reacquisition import (
    build_station_support_gap, validate_contract, validate_registration,
    validate_station_support_gap,
)
from test_station_requalification_three_owners import owners, task_deliver
from test_unknown_pending_identity import publication, deliver
from test_support_measurement_hold import support_gap_depth
from test_locked_rgb_hold import custom_frame, latest_status


def wire(c, row, order=('status', 'object', 'mask')):
    deliver(c.node, row, order)
    task_deliver(NS(node=c.task), row, order)


def held(monkeypatch):
    c = owners(monkeypatch)
    c.parent = deepcopy(c.ready['status']['station_compute_wait'])
    c.clock[0] += .15
    custom_frame(c.producer, c.clock, c.joint, depth_image=support_gap_depth())
    c.pending = publication(c.producer)
    assert c.pending['status']['detail'] == 'support_measurement_pending_revalidation'
    wire(c, c.pending)
    return c


def assert_pending_without_geometry(c):
    assert not c.producer.tracker.lost and c.producer.first_loss is None
    assert c.node._station_compute_pending and not c.node._station_compute_retired
    assert c.task._station_task_record['pending'] and not c.task._station_task_retired
    assert c.task._unknown_target_loss is None
    assert not getattr(c.node.latest_object_geometry, 'valid', False)
    assert not getattr(c.node.latest_preview_rich_plan, 'valid', False)
    assert c.task.latest_obj is None
    ticket = NS(payload=(NS(stamp_ns=c.parent['source_ns'],
                            sample_stamp_ns=(c.parent['source_ns'],)),))
    assert c.node._station_compute_ticket_code(ticket) == 'TARGET_PROCESSING_PENDING'
    old_plan = deepcopy(c.plan)
    old_plan.diagnostic = 'CONTACT_EXECUTION_PLAN'
    old_plan.header.stamp = type(old_plan.header.stamp).from_sec(c.parent['source_ns'] / 1e9)
    assert c.task._station_task_plan_failure(old_plan).code == 'TARGET_PROCESSING_PENDING'


def test_recorded_boundary_retains_identity_but_never_authorizes_old_geometry():
    recorded = json.loads((Path(__file__).parent / 'fixtures/gen24_support_gap_expiry.json').read_text())
    parent = recorded['parent_contract']
    original = recorded['last_recorded_gap']
    now = recorded['producer_failure_ns']
    assert original['deadline_ns'] == parent['geometry_deadline_ns'] < now
    assert recorded['failing_source_ns'] < now < parent['identity_deadline_ns']
    gap = build_station_support_gap(parent, original['source_ns'], original['reason'],
                                   original['first_source_ns'])
    assert gap['deadline_ns'] == parent['identity_deadline_ns']
    assert validate_station_support_gap(gap, parent, now) == gap
    assert validate_contract(parent, binding=parent['binding'], now_ns=now) == 'PENDING'
    with pytest.raises(ValueError):
        validate_registration(parent, binding=parent['binding'], now_ns=now)
    assert all(gap[k] is False for k in
               ('metric_geometry_authority', 'motion_authorized', 'reference_refreshed'))


@pytest.mark.parametrize('empty_or_stale', ['empty', 'stale'])
def test_real_three_owner_early_returns_wait_without_reissuing_old_geometry(monkeypatch, empty_or_stale):
    c = held(monkeypatch)
    old = c.producer._station_compute_registration
    caps = (old['support_gap_monotonic'],
            c.node._station_compute_reference['support_gap']['monotonic_deadline'],
            c.task._station_task_record['support_gap']['monotonic_deadline'])
    c.clock[0] = c.parent['source_ns'] / 1e9 + 1.25
    c.joint(c.clock[0] - .01)
    if empty_or_stale == 'empty':
        c.producer.latest = None
        c.producer.process(None)
        expected_source = c.parent['source_ns']
    else:
        pair = custom_frame(c.producer, c.clock, c.joint, target_depth=.35, old_stamp=True)
        expected_source = pair[0].header.stamp.to_nsec()
    row = publication(c.producer)
    # These early returns describe a different source from the measured gap.
    # Preserve their real source instead of rewriting timestamps to satisfy a
    # consumer. The already admitted gap still retains its original clocks.
    assert row['status']['source_stamp_ns'] == expected_source
    assert row['status']['detail'] == 'processing_deadline_pending'
    assert 'station_support_gap' not in row['status']
    wire(c, row)
    c.node._station_compute_poll()
    c.task._station_task_check()
    assert_pending_without_geometry(c)
    assert old['support_gap']['deadline_ns'] == c.parent['identity_deadline_ns']
    assert c.producer._station_compute_registration is old
    assert c.producer.last_good_source_stamp_ns == c.parent['source_ns']
    assert caps == (old['support_gap_monotonic'],
                    c.node._station_compute_reference['support_gap']['monotonic_deadline'],
                    c.task._station_task_record['support_gap']['monotonic_deadline'])


@pytest.mark.parametrize('order', [('status', 'object', 'mask'), ('object', 'mask', 'status')])
def test_real_fresh_support_recovery_past_parent_geometry_requires_complete_join(monkeypatch, order):
    c = held(monkeypatch)
    identity = c.node._current_stream_target_identity()
    c.clock[0] = c.parent['source_ns'] / 1e9 + 1.35
    custom_frame(c.producer, c.clock, c.joint, target_depth=.35)
    ready = publication(c.producer)
    assert ready['status']['state'] == 'ready', ready['status'].get('detail')
    assert ready['status']['station_compute_recovery'] == c.parent
    assert ready['status']['station_support_gap_recovery'] == c.pending['status']['station_support_gap']
    assert ready['object'].header.stamp.to_nsec() > c.parent['geometry_deadline_ns']
    for key in order[:-1]:
        wire(c, ready, (key,))
        assert c.node._station_compute_pending and c.task._station_task_record['pending']
    wire(c, ready, order[-1:])
    assert not c.node._station_compute_pending and not c.task._station_task_record['pending']
    assert c.node._current_stream_target_identity() == identity
    assert c.node._station_compute_reference['contract'] == ready['status']['station_compute_wait']
    assert c.task._station_task_record['contract'] == ready['status']['station_compute_wait']
    assert c.producer.last_good_source_stamp_ns == ready['object'].header.stamp.to_nsec()
    assert c.node.latest_object.detected and c.task.latest_obj.detected
    assert not getattr(c.node.latest_preview_rich_plan, 'valid', False)
    wire(c, c.pending, order)
    assert not c.node._station_compute_pending and not c.task._station_task_record['pending']
    assert c.task.latest_obj.header.stamp == ready['object'].header.stamp


def test_later_repeated_real_gaps_cannot_slide_any_owner_deadline(monkeypatch):
    c = held(monkeypatch)
    registration = c.producer._station_compute_registration
    gap = deepcopy(c.pending['status']['station_support_gap'])
    caps = (registration['support_gap_monotonic'],
            c.node._station_compute_reference['support_gap']['monotonic_deadline'],
            c.task._station_task_record['support_gap']['monotonic_deadline'])
    for offset in (1.2, 2.5, 4.7):
        c.clock[0] = c.parent['source_ns'] / 1e9 + offset
        custom_frame(c.producer, c.clock, c.joint, depth_image=support_gap_depth())
        row = publication(c.producer)
        assert row['status']['state'] == 'processing_deadline_pending', row['status'].get('detail')
        wire(c, row)
        assert_pending_without_geometry(c)
        current = row['status']['station_support_gap']
        assert current['source_ns'] > gap['source_ns']
        assert all(current[k] == gap[k] for k in ('first_source_ns', 'parent_source_ns', 'deadline_ns'))
        assert caps == (registration['support_gap_monotonic'],
                        c.node._station_compute_reference['support_gap']['monotonic_deadline'],
                        c.task._station_task_record['support_gap']['monotonic_deadline'])
    assert c.producer.last_good_source_stamp_ns == c.parent['source_ns']


@pytest.mark.parametrize('clock_kind', ['ros', 'monotonic'])
def test_fixed_identity_deadline_cannot_be_reopened_by_new_support_or_ready(monkeypatch, clock_kind):
    c = held(monkeypatch)
    old_good = c.producer.last_good_source_stamp_ns
    if clock_kind == 'ros':
        c.clock[0] = c.parent['identity_deadline_ns'] / 1e9 + .001
    else:
        from alicia_grasp_modes import station_compute_wait_producer as impl
        cap = c.producer._station_compute_registration['support_gap_monotonic']
        c.clock[0] = c.parent['source_ns'] / 1e9 + .5
        monkeypatch.setattr(impl.time, 'monotonic', lambda: cap + .001)
    custom_frame(c.producer, c.clock, c.joint, target_depth=.35)
    wire(c, publication(c.producer))
    c.node._station_compute_poll()
    c.task._station_task_check()
    assert latest_status(c.producer)['state'] != 'ready'
    assert c.producer.last_good_source_stamp_ns == old_good
    assert c.node._station_compute_retired and c.task._station_task_retired
    assert not getattr(c.node.latest_preview_rich_plan, 'valid', False)


@pytest.mark.parametrize('fault', ['manual', 'epoch', 'camera', 'support_conflict'])
def test_identity_wait_never_waives_current_measurement_or_owner_checks(monkeypatch, fault):
    c = held(monkeypatch)
    old_good = c.producer.last_good_source_stamp_ns
    c.clock[0] = c.parent['source_ns'] / 1e9 + 1.2
    kwargs = dict(target_depth=.35)
    if fault == 'manual':
        c.params['/gui/joint_direct_mode'] = True
    elif fault == 'epoch':
        c.producer._rgb_interest_epoch += 1
    elif fault == 'camera':
        c.params['/camera']['fx'] += 1
    else:
        # Real depth disagrees with the immutable support by 8 mm. Let the
        # actual fitter and original 7 mm consistency gate reject it; changing
        # only a result summary would not exercise that geometric boundary.
        depth = np.full((240, 320), .408, dtype=np.float32)
        depth[106:134, 148:173] = .358
        kwargs['depth_image'] = depth
    custom_frame(c.producer, c.clock, c.joint, **kwargs)
    wire(c, publication(c.producer))
    assert latest_status(c.producer)['state'] != 'ready'
    assert c.producer.last_good_source_stamp_ns == old_good
    assert not getattr(c.node.latest_preview_rich_plan, 'valid', False)
    assert c.task.latest_obj is None
