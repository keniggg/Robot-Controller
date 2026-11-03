"""Actual callback fan-in before first READY registration; no ROS or movement."""
from copy import deepcopy
from itertools import permutations
from types import SimpleNamespace as NS

import pytest

from test_station_compute_wait_remote import station, context, clock, row_at, feedback
from test_unknown_pending_identity import deliver
from alicia_grasp_modes.selection import SELECTION_PARAM


def rejected(c):
    parent = c.ready['status']['station_compute_wait']
    ns = parent['source_ns'] + 200_000_000
    clock(c, ns + 10_000_000)
    row = row_at(c, ns)
    row['object'].detected = False
    row['mask'].data = bytes(len(row['mask'].data))
    row['status'].update(state='station_measurement_pending',
        detail='station_seed_measurement_insufficient', station_compute_wait=deepcopy(parent),
        discarded_source_stamp_ns=ns, station_seed_insufficiency=dict(
            version=1, parent_source_ns=parent['source_ns'], source_ns=ns,
            stage='raw_only_extra_core', region_index=0, region_mask_sha256='d'*64,
            region_shape=[row['mask'].height, row['mask'].width],
            measured_pixels=80, eroded_pixels=70, reliable_eroded_pixels=60,
            original_top_pixels=20, trusted_top_pixels=10, trusted_side_pixels=8,
            trusted_core_pixels=18, minimum_core_pixels=25, minimum_measured_points=60,
            upper_surface_band_used=False))
    return row


def no_authority(c):
    n = c.node
    assert n._station_compute_pending
    assert not getattr(n.latest_object_geometry, 'valid', False)
    assert not getattr(n.latest_preview_rich_plan, 'valid', False)
    ticket = NS(payload=(NS(stamp_ns=c.ready['status']['source_stamp_ns']),))
    assert n._station_compute_ticket_code(ticket) == 'TARGET_PROCESSING_PENDING'
    assert not n._station_compute_work


@pytest.mark.parametrize('order', list(permutations(('object', 'mask', 'status'))))
@pytest.mark.parametrize('ready_last', ('object', 'mask', 'status'))
@pytest.mark.parametrize('ready_arrival', ('after_first', 'after_all'))
def test_first_negative_waits_for_original_fresh_ready_join(station, order, ready_last, ready_arrival):
    c = station; n = c.node
    failed = rejected(c)
    deliver(n, c.ready, tuple(k for k in ('object', 'mask', 'status') if k != ready_last))
    deliver(n, failed, order[:1])
    assert n._station_compute_reference is None
    assert n._current_stream_target_identity() == c.identity
    no_authority(c)
    if ready_arrival == 'after_all':
        deliver(n, failed, order[1:])
        assert n._station_compute_reference is None
        no_authority(c)
    deliver(n, c.ready, (ready_last,))
    assert n._station_compute_reference['contract'] == c.ready['status']['station_compute_wait']
    no_authority(c)
    if ready_arrival == 'after_first':
        deliver(n, failed, order[1:])
    assert not n._station_compute_retired
    assert n._station_initial_pending is None
    assert n._station_private_pending_sources[failed['status']['source_stamp_ns']]['kind'] == 'seed_measurement_insufficient'
    assert n._current_stream_target_identity() == c.identity
    no_authority(c)


@pytest.mark.parametrize('bad', ('sources_absent', 'sources_differ', 'mask_absent'))
def test_producer_qualified_marker_cannot_create_initial_hold(station, bad):
    c = station; n = c.node; failed = rejected(c)
    if bad == 'sources_absent': n._measurement_remote_sources = None
    elif bad == 'sources_differ': n._measurement_remote_sources = [1, 2, 3]
    else: n._measurement_remote_masks.clear()
    deliver(n, failed, ('object',))
    assert n._station_initial_pending is None
    assert n._station_compute_reference is None
    assert n._current_stream_target_identity().epoch > c.identity.epoch


@pytest.mark.parametrize('bad', ('epoch', 'manual', 'selection', 'phase', 'true_loss',
                                'session', 'measurement', 'mask_frame', 'mask_shape',
                                'untyped', 'config', 'source', 'object_became_true', 'mask_became_nonempty'))
def test_real_contradictions_are_not_hidden_by_classification_wait(station, bad):
    c = station; n = c.node; failed = rejected(c)
    deliver(n, failed, ('object',))
    if bad == 'epoch': n._measurement_remote_epoch_ns += 1
    elif bad == 'manual': c.params['/gui/joint_direct_mode'] = True
    elif bad == 'selection': c.params[SELECTION_PARAM] = {}
    elif bad == 'phase': c.phase.phase_id += 1
    elif bad == 'session': n._observe_return_session['session_id'] = 'different'
    elif bad == 'measurement': n._measurement_remote_reference['binding'] = 'f'*64
    elif bad in ('mask_frame', 'mask_shape'):
        if bad == 'mask_frame': failed['mask'].header.frame_id = 'different'
        else:
            failed['mask'].height += 1
            failed['mask'].data += bytes(failed['mask'].step)
        deliver(n, failed, ('mask',))
    elif bad == 'true_loss': failed['status']['target_lost'] = True
    elif bad == 'untyped': failed['status']['state'] = 'unavailable'
    elif bad == 'config': failed['status']['station_compute_wait']['binding']['reference_binding'] = 'f'*64
    elif bad == 'source': failed['status']['station_seed_insufficiency']['source_ns'] += 1
    elif bad == 'object_became_true':
        changed = deepcopy(failed); changed['object'].detected = True
        deliver(n, changed, ('object',))
    elif bad == 'mask_became_nonempty':
        changed = deepcopy(failed); changed['mask'].data = c.ready['mask'].data
        deliver(n, changed, ('mask',))
    if bad in ('true_loss', 'untyped', 'config', 'source'):
        deliver(n, failed, ('status',))
    n._station_compute_poll()
    assert n._station_compute_retired
    assert n._station_compute_reference is None


@pytest.mark.parametrize('clock_kind', ('ros', 'monotonic'))
def test_missing_status_and_repeated_frames_do_not_extend_classification_deadline(station, monkeypatch, clock_kind):
    c = station; n = c.node; failed = rejected(c)
    deliver(n, failed, ('object',))
    hold = deepcopy(n._station_initial_pending)
    c.clock[0] += .1
    deliver(n, failed, ('object', 'mask'))
    assert n._station_initial_pending['deadline_ns'] == hold['deadline_ns']
    assert n._station_initial_pending['monotonic_deadline'] == hold['monotonic_deadline']
    if clock_kind == 'ros':
        monkeypatch.setattr(n, '_ros_source_clock_ns', lambda: hold['deadline_ns'])
    else:
        from alicia_grasp_modes import station_compute_wait_remote as impl
        monkeypatch.setattr(impl.time, 'monotonic', lambda: hold['monotonic_deadline'])
    n._station_compute_poll()
    assert n._station_compute_retired


def test_stale_parent_is_not_first_registered_even_with_valid_pending(station, monkeypatch):
    c = station; n = c.node; failed = rejected(c)
    deliver(n, failed)
    hold = deepcopy(n._station_initial_pending)
    clock(c, c.ready['status']['station_compute_wait']['geometry_deadline_ns'] + 1)
    deliver(n, c.ready)
    assert n._station_compute_reference is None
    no_authority(c)
    monkeypatch.setattr(n, '_ros_source_clock_ns', lambda: hold['deadline_ns'])
    assert n._station_compute_poll()
    assert n._station_compute_retired


@pytest.mark.parametrize('field', ('source_id', 'camera_configuration_sha256', 'config_sha256'))
def test_pending_cannot_bind_a_different_parent_source_or_configuration(station, field):
    c = station; n = c.node; failed = rejected(c)
    failed['status']['station_compute_wait']['binding'][field] = 'f'*64
    deliver(n, failed)
    deliver(n, c.ready)
    n._station_compute_poll()
    assert n._station_compute_retired
    assert n._station_compute_reference is None


def test_unexplained_initial_input_is_not_silently_evicted(station):
    c = station; n = c.node; failed = rejected(c)
    source = failed['object'].header.stamp.to_nsec()
    for offset in range(65):
        row = deepcopy(failed)
        from test_observation_measurement_remote import at
        row['object'].header.stamp = at(source + offset)
        deliver(n, row, ('object',))
        if n._station_compute_retired:
            break
    assert n._station_compute_retired
    assert n._station_compute_reference is None
