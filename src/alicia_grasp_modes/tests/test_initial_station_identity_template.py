"""Fixed acquisition appearance and fresh geometry remain separate contracts."""
from copy import deepcopy
import hashlib
import itertools

import numpy as np
import pytest

from test_locked_rgb_hold import locked, custom_frame, latest_status, module
from test_support_measurement_hold import support_gap_depth
from test_unknown_pending_identity import consumer_for, publication, deliver


def initial(monkeypatch):
    return locked(monkeypatch, initial_station=True)


def sha(mask):
    return hashlib.sha256(np.ascontiguousarray(mask != 0, dtype=np.uint8).tobytes()).hexdigest()


def test_acquisition_template_is_immutable_and_not_the_rolling_geometry_source(monkeypatch):
    n, clock, feed, _, _ = initial(monkeypatch)
    ref = n._locked_rgb_hold_reference
    marker = ref['initial_station_current_support']
    template = marker['identity_template']
    provenance = deepcopy(template['provenance'])
    assert provenance['source_stamp_ns'] == n.anchor_source_stamp_ns == ref['source_stamp_ns']
    assert provenance['refined_mask_sha256'] == sha(template['mask'])
    assert not template['bgr'].flags.writeable and not template['mask'].flags.writeable
    for _ in range(5):
        clock[0] += .15
        feed(target_depth=.35)
        current = n._locked_rgb_hold_reference
        assert current['initial_station_current_support']['identity_template'] is template
        assert latest_status(n)['initial_station_identity_template'] == provenance
        assert current['source_stamp_ns'] > provenance['source_stamp_ns']
        assert latest_status(n)['initial_station_measurement']['source_stamp_ns'] == current['source_stamp_ns']
    assert clock[0]-provenance['source_stamp_ns']/1e9 > .8


@pytest.mark.parametrize('order', list(itertools.permutations(('object', 'mask', 'status'))))
def test_first_acquired_three_topic_join_pins_immediately_in_any_order(monkeypatch, order):
    p, clock, _, _, params = initial(monkeypatch)
    n = consumer_for(p, clock, params)
    row = publication(p)
    deliver(n, row, order[:2])
    assert n._unknown_identity_template is None
    deliver(n, row, order[2:])
    assert n._unknown_identity_template['provenance'] == row['status']['initial_station_identity_template']
    assert n._unknown_identity_reference['source_ns'] == row['status']['anchor_source_stamp_ns']
    assert n._unknown_identity_reference['identity_template'] == n._unknown_identity_template['provenance']


def test_old_template_survives_only_fresh_rolling_geometry_and_real_typed_recovery(monkeypatch):
    p, clock, feed, joint, params = initial(monkeypatch)
    n = consumer_for(p, clock, params)
    deliver(n, publication(p))
    pinned = deepcopy(n._unknown_identity_template)
    identity = n._current_stream_target_identity()
    for _ in range(5):
        clock[0] += .15
        feed(target_depth=.35)
        deliver(n, publication(p))
    ref = p._locked_rgb_hold_reference
    old_geometry_ns = ref['source_stamp_ns']
    assert clock[0]-pinned['provenance']['source_stamp_ns']/1e9 > .8
    # Pollute only the rolling appearance cache. The fixed acquisition template
    # must supply identity; this never replaces current depth/mask geometry.
    ref['mask'] = np.zeros_like(ref['mask'])
    clock[0] += .1
    custom_frame(p, clock, joint, depth_image=support_gap_depth())
    gap = publication(p)
    deliver(n, gap)
    assert n._unknown_identity_pending is not None and not n.latest_object.detected
    original_deadline = n._unknown_identity_pending['deadline_ns']
    clock[0] += .15
    feed(target_depth=.35)
    recovered = publication(p)
    assert recovered['object'].detected, recovered['status']
    q = recovered['status']['source_qualification']
    assert q['reference_source_stamp_ns'] == old_geometry_ns
    assert q['rgb_identity']['identity_template_source'] == pinned['provenance']
    assert q['source_stamp_ns'] > old_geometry_ns > pinned['provenance']['source_stamp_ns']
    assert clock[0]*1e9 < original_deadline
    deliver(n, recovered)
    assert n.latest_object.detected and n._unknown_identity_pending is None
    assert n._current_stream_target_identity() == identity
    assert n._unknown_identity_template == pinned
    assert n._unknown_identity_reference['source_ns'] == q['source_stamp_ns']
    assert not n.geometry_pub.messages[-1].valid


@pytest.mark.parametrize('change', ['source', 'mask_sha', 'rgb_sha', 'epoch', 'selection',
                                   'generation', 'source_id', 'shape', 'role', 'missing'])
def test_consumer_never_replaces_pinned_template_from_later_ready(monkeypatch, change):
    p, clock, feed, _, params = initial(monkeypatch)
    n = consumer_for(p, clock, params)
    deliver(n, publication(p))
    identity = n._current_stream_target_identity()
    feed(target_depth=.35)
    row = publication(p)
    value = row['status']['initial_station_identity_template']
    if change == 'source': value['source_stamp_ns'] += 1
    elif change == 'mask_sha': value['refined_mask_sha256'] = 'f'*64
    elif change == 'rgb_sha': value['source_rgb_sha256'] = 'f'*64
    elif change == 'epoch': value['epoch_ns'] += 1
    elif change == 'selection': value['selection_stamp_ns'] += 1
    elif change == 'generation': value['generation'] += 1
    elif change == 'source_id': value['source_id'] = 'f'*64
    elif change == 'shape': value['image_shape'][0] += 1
    elif change == 'role': value['role'] = 'latest_available'
    else: row['status'].pop('initial_station_identity_template')
    deliver(n, row)
    assert not n.latest_object.detected
    assert n._current_stream_target_identity() != identity


def test_template_pin_verifies_actual_received_mask_pixels(monkeypatch):
    p, clock, _, _, params = initial(monkeypatch)
    n = consumer_for(p, clock, params)
    row = publication(p)
    row['status']['initial_station_identity_template']['refined_mask_sha256'] = 'f'*64
    deliver(n, row)
    assert n._unknown_identity_template is None and not n.latest_object.detected


def test_unobserved_acquisition_cannot_be_self_declared_by_later_ready(monkeypatch):
    p, clock, feed, _, params = initial(monkeypatch)
    n = consumer_for(p, clock, params)
    acquired = publication(p)
    feed(target_depth=.35)
    later = publication(p)
    deliver(n, later)
    assert n._unknown_identity_template is None and n._unknown_identity_reference is None
    # The missing exact initial trio may arrive within its original .8 seconds;
    # its actual pixels and immutable reference are then verified, without any
    # five-frame collector having run.
    deliver(n, acquired)
    assert n._unknown_identity_template['provenance'] == acquired['status']['initial_station_identity_template']


@pytest.mark.parametrize('change', ['omit', 'swap', 'geometry_label'])
def test_recovery_receipt_cannot_relabel_historical_template_as_fresh_geometry(monkeypatch, change):
    p, clock, feed, joint, params = initial(monkeypatch)
    n = consumer_for(p, clock, params)
    deliver(n, publication(p))
    custom_frame(p, clock, joint, depth_image=support_gap_depth())
    deliver(n, publication(p))
    clock[0] += .15
    feed(target_depth=.35)
    row = publication(p)
    q = row['status']['source_qualification']
    if change == 'omit': q['rgb_identity'].pop('identity_template_source')
    elif change == 'swap': q['rgb_identity']['identity_template_source']['source_rgb_sha256'] = 'f'*64
    else: q['rgb_identity']['identity_template_source']['source_stamp_ns'] = q['source_stamp_ns']
    deliver(n, row)
    assert not n.latest_object.detected and n._unknown_identity_pending is None


@pytest.mark.parametrize('change', ['source', 'mask_pixels', 'writable', 'epoch'])
def test_producer_rejects_template_mutation_without_publishing_recovery(monkeypatch, change):
    p, clock, feed, joint, _ = initial(monkeypatch)
    template = p._locked_rgb_hold_reference['initial_station_current_support']['identity_template']
    custom_frame(p, clock, joint, depth_image=support_gap_depth())
    if change == 'source': template['provenance']['source_stamp_ns'] += 1
    elif change == 'epoch': template['provenance']['epoch_ns'] += 1
    elif change == 'writable': template['mask'] = template['mask'].copy()
    else:
        value = np.zeros_like(template['mask'])
        template['mask'] = np.frombuffer(value.tobytes(), np.uint8).reshape(value.shape)
    clock[0] += .15
    feed(target_depth=.35)
    assert not p.pubs['object'].publish.call_args[0][0].detected
    assert p.tracker.lost
