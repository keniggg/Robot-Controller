"""Current source conflicts remain rejected while one bounded station waits.

The measured finite-prism fixture produces the original numerical overlap
exception; only its delivery at a registered producer measurement hook is
controlled. This is a transport/ownership contract test, not real-frame replay.
"""
from copy import deepcopy
from dataclasses import replace
from itertools import permutations
from types import SimpleNamespace as NS

import numpy as np
import pytest

from alicia_grasp_modes import source_connected_owner as owner
from alicia_grasp_modes.source_sampling_topology import SourceSamplingTopologyCache
from alicia_grasp_modes.station_compute_reacquisition import (
    measurement_rejection_receipt, validate_measurement_rejection)
import test_locked_rgb_hold as hold_fixture
from test_gen30_source_sampling_topology import identity_camera
from test_gen57_source_component_measurement import measured_scene, recover, _foreign
from test_gen61_station_seed_wait import (
    owners as original_owners, wire, fail_frame as seed_frame,
    fresh_frame, assert_nonmetric_wait, seed_error)
from test_locked_rgb_hold import custom_frame, module
from test_unknown_pending_identity import publication


def owners(monkeypatch):
    original = hold_fixture.rgb_interest_node
    def calibrated(patch):
        node, clock, feed, joint, params = original(patch)
        camera = params['/camera']
        camera.update(identity_camera((240, 320), (420., 420., 160., 120.)))
        cache = node._source_sampling_topology = SourceSamplingTopologyCache()
        cache.offer(camera)
        worker = cache._thread
        if worker is not None:
            worker.join(2.)
            assert not worker.is_alive()
        cache.require(camera, (240, 320), (420., 420., 160., 120.), 100)
        return node, clock, feed, joint, params
    monkeypatch.setattr(hold_fixture, 'rgb_interest_node', calibrated)
    c = original_owners(monkeypatch)
    return c


def numerical_conflict(monkeypatch, source_ns, *, overlap=1, native=False):
    scene = measured_scene()
    scene['arguments']['source_stamp_ns'] = source_ns
    valid = recover(scene)
    foreign, candidate = _foreign(scene, 20, 20)
    if native:
        scene['arguments']['result'].candidates.append(candidate)
    proposed = valid.refined.mask.copy()
    proposed.ravel()[np.flatnonzero(foreign)[:overlap]] = 255
    original_depth = scene['arguments']['depth'].copy()
    originals = [x.mask.copy() for x in scene['arguments']['result'].candidates]
    with monkeypatch.context() as patch:
        patch.setattr(owner._FrameRefinement, 'refine',
            lambda *a, **kw: replace(valid.refined, mask=proposed))
        with pytest.raises(owner.CurrentSourceOwnershipConflict) as caught:
            recover(scene)
    error = caught.value
    assert type(error) is owner.CurrentSourceOwnershipConflict
    assert isinstance(error, ValueError)
    assert str(error) == ('current_fragment_other_measured_component_overlap' if native
                         else 'current_fragment_foreign_raw_source_component_overlap')
    e = error.evidence
    assert e['component_basis'] == ('native_measured' if native else 'raw_source')
    assert e['component_pixels'] == 144 and e['overlap_pixels'] == overlap
    assert e['source_ns'] == source_ns and e['source_qualified'] is False
    assert e['metric_geometry_authority'] is False and e['motion_authorized'] is False
    np.testing.assert_array_equal(scene['arguments']['depth'], original_depth)
    for candidate, before in zip(scene['arguments']['result'].candidates, originals):
        np.testing.assert_array_equal(candidate.mask, before)
    return error


def overlap_frame(c, monkeypatch, *, path='single', overlap=1, native=False,
                  mutate=None, error_kind='typed'):
    reached = []
    def reject(source, *args, **kwargs):
        assert c.producer._station_compute_frame['source'] is source
        e = numerical_conflict(monkeypatch, source[0].header.stamp.to_nsec(),
                               overlap=overlap, native=native)
        if mutate is not None:
            mutate(e.evidence)
        if error_kind == 'plain':
            e = ValueError(str(e))
        elif error_kind == 'subclass':
            class UnprovenSubclass(owner.CurrentSourceOwnershipConflict):
                pass
            e = UnprovenSubclass(str(e), e.evidence)
        reached.append(e)
        raise e
    with monkeypatch.context() as patch:
        patch.setattr(c.producer, '_recover_station_current_fragment_owner' if path == 'single'
                      else '_recover_station_source_component_measurement', reject)
        c.clock[0] += .15
        source = custom_frame(c.producer, c.clock, c.joint, target_depth=.35)
    assert len(reached) == 1
    return publication(c.producer), source, reached[0]


def clocks(c):
    return (c.producer._station_compute_registration['monotonic_deadline'],
            c.node._station_compute_reference['identity_monotonic'],
            c.task._station_task_record['identity_mono'])


def assert_no_plan(c):
    assert not getattr(c.node.latest_preview_rich_plan, 'valid', False)
    # The original FAR_FIELD cache remains for metadata rebind. Its valid bit
    # is not contact authority; exercise the actual independent admission gate.
    assert not getattr(getattr(c.task, 'latest_grasp6d_preview_plan', None), 'valid', False)
    assert not c.task._station_contact_is_admitted(c.plan)
    assert not c.task._station_contact_provenance(c.plan).ok
    cached = c.task.latest_grasp6d_plan
    if cached is not None:
        assert cached.diagnostic == 'FAR_FIELD_OBSERVATION_PLAN'
        assert cached.plan_id == c.plan.plan_id
        assert c.task._execution_plan_digest(cached) == c.task._execution_plan_digest(c.plan)


ORDERS = list(permutations(('status', 'object', 'mask')))
@pytest.mark.parametrize('order,path,overlap,native', [
    (order, 'single' if i % 2 == 0 else 'multi', 1 if i < 3 else 144, i % 3 == 0)
    for i, order in enumerate(ORDERS)])
def test_all_topic_orders_withhold_conflict_and_require_original_fresh_join(
        monkeypatch, order, path, overlap, native):
    c = owners(monkeypatch)
    parent = deepcopy(c.ready['status']['station_compute_wait'])
    previous = (c.producer.tracker.anchor, c.producer.tracker.last,
                c.producer._locked_rgb_hold_reference, c.producer.last_good_source_stamp_ns)
    deadlines = clocks(c)
    acknowledgments = (c.producer._observation_measurement_ack_pub.publish.call_count,
                       len(c.node._measurement_remote_ack_pub.messages))
    original_publish = c.node.geometry_pub.publish
    def geometry(message):
        original_publish(message)
        c.task.target_geometry_cb(deepcopy(message))
    monkeypatch.setattr(c.node.geometry_pub, 'publish', geometry)
    row, source, error = overlap_frame(c, monkeypatch, path=path, overlap=overlap, native=native)
    status = row['status'];ns = source[0].header.stamp.to_nsec()
    receipt = dict(error.evidence, version=1, parent_source_ns=parent['source_ns'])
    assert status['state'] == 'station_measurement_pending'
    assert status['detail'] == 'station_current_measurement_rejected'
    assert status['station_frame_rejection'] == receipt
    assert status.get('station_seed_insufficiency') is None
    assert status['station_compute_wait'] == parent
    assert status['source_stamp_ns'] == row['object'].header.stamp.to_nsec() == row['mask'].header.stamp.to_nsec() == ns
    assert not row['object'].detected and not np.any(c.producer.bridge.imgmsg_to_cv2(row['mask']))
    assert not c.producer.tracker.lost and c.producer.first_loss is None
    assert c.producer.tracker.anchor is previous[0] and c.producer.tracker.last is previous[1]
    assert c.producer._locked_rgb_hold_reference is previous[2]
    assert c.producer.last_good_source_stamp_ns == previous[3]
    assert validate_measurement_rejection(receipt, parent,
        module.rospy.Time.now().to_nsec()) == receipt
    wire(c, row, order)
    assert_nonmetric_wait(c)
    assert clocks(c) == deadlines
    assert_no_plan(c)
    ready = fresh_frame(c)
    assert ready['status']['station_frame_rejection_recovery'] == receipt
    assert ready['status'].get('station_seed_insufficiency_recovery') is None
    assert ready['status']['station_compute_recovery'] == parent
    for key in order[:-1]:
        wire(c, ready, (key,))
        assert_nonmetric_wait(c)
        assert clocks(c)[1:] == deadlines[1:]
    wire(c, ready, order[-1:])
    assert not c.node._station_compute_retired and not c.task._station_task_retired
    assert not c.node._station_compute_pending and not c.task._station_task_record['pending']
    assert c.task.latest_obj.header.stamp == ready['object'].header.stamp
    assert c.node._current_stream_target_identity() == c.identity
    assert acknowledgments == (c.producer._observation_measurement_ack_pub.publish.call_count,
                              len(c.node._measurement_remote_ack_pub.messages))
    assert_no_plan(c)
    wire(c, row, order)
    assert not c.node._station_compute_retired and not c.task._station_task_retired
    assert c.task.latest_obj.header.stamp == ready['object'].header.stamp


@pytest.mark.parametrize('first_kind', ['seed', 'overlap'])
@pytest.mark.parametrize('missing', ['status', 'object', 'mask'])
def test_alternating_causes_do_not_erase_earlier_unjoined_bad_source(monkeypatch, first_kind, missing):
    c = owners(monkeypatch)
    first = (seed_frame if first_kind == 'seed' else overlap_frame)(c, monkeypatch)[0]
    wire(c, first, tuple(k for k in ('status', 'object', 'mask') if k != missing))
    second = (overlap_frame if first_kind == 'seed' else seed_frame)(c, monkeypatch)[0]
    wire(c, second)
    assert_nonmetric_wait(c)
    ready = fresh_frame(c)
    wire(c, ready)
    assert_nonmetric_wait(c)
    assert_no_plan(c)
    wire(c, first, (missing,))
    later = fresh_frame(c)
    wire(c, later)
    assert not c.node._station_compute_retired and not c.task._station_task_retired
    assert not c.node._station_compute_pending and not c.task._station_task_record['pending']
    assert c.task.latest_obj.header.stamp == later['object'].header.stamp


@pytest.mark.parametrize('missing', ['status', 'object', 'mask'])
def test_missing_fresh_topic_cannot_grant_geometry_or_renew_wait(monkeypatch, missing):
    c = owners(monkeypatch)
    row, _, _ = overlap_frame(c, monkeypatch)
    wire(c, row)
    deadlines = clocks(c)
    ready = fresh_frame(c)
    wire(c, ready, tuple(k for k in ('status', 'object', 'mask') if k != missing))
    assert_nonmetric_wait(c)
    assert clocks(c)[1:] == deadlines[1:]
    assert_no_plan(c)


@pytest.mark.parametrize('expired', ['ros', 'monotonic'])
def test_repeated_conflicts_share_original_absolute_and_monotonic_limit(monkeypatch, expired):
    c = owners(monkeypatch);parent = deepcopy(c.ready['status']['station_compute_wait'])
    deadlines = clocks(c);last_good = c.producer.last_good_source_stamp_ns
    for maker in (overlap_frame, seed_frame, overlap_frame):
        row, _, _ = maker(c, monkeypatch);wire(c, row)
        assert_nonmetric_wait(c)
        assert row['status']['station_compute_wait'] == parent
        assert clocks(c) == deadlines
        assert c.producer.last_good_source_stamp_ns == last_good
    if expired == 'ros':
        c.clock[0] = parent['identity_deadline_ns']/1e9 + .001
        c.joint(c.clock[0]-.001)
    else:
        monkeypatch.setattr('time.monotonic', lambda: max(deadlines)+.001)
    c.producer.latest = None;c.producer.process(None)
    c.node._station_compute_poll();c.task._station_task_check()
    assert c.producer.first_loss is not None
    assert c.node._station_compute_retired and c.task._station_task_retired
    assert_no_plan(c)


@pytest.mark.parametrize('fault', ['source', 'topology', 'shape', 'camera'])
def test_typed_evidence_must_match_actual_source_and_cached_topology(monkeypatch, fault):
    c = owners(monkeypatch)
    def corrupt(e):
        if fault == 'source':e['source_ns'] -= 1
        elif fault == 'topology':e['source_topology_contract_digest'] = '0'*64
        elif fault == 'camera':c.params['/camera']['fx'] += 1.
        else:e['source_shape'][0] += 1
    row, _, _ = overlap_frame(c, monkeypatch, mutate=corrupt)
    assert row['status']['state'] != 'station_measurement_pending'
    assert c.producer.first_loss is not None
    wire(c, row)
    assert c.node._station_compute_retired and c.task._station_task_retired
    assert_no_plan(c)


@pytest.mark.parametrize('fault', ['epoch', 'selection'])
def test_wait_does_not_hide_owner_generation_revocation(monkeypatch, fault):
    c = owners(monkeypatch);row, _, _ = overlap_frame(c, monkeypatch);wire(c, row)
    assert_nonmetric_wait(c)
    if fault == 'epoch':
        c.producer._rgb_interest_epoch += 1
        c.node._measurement_remote_epoch_ns += 1
        c.task._following_epoch_ns += 1
    else:
        c.params['/grasp_mode/selection']['generation'] += 1
    c.producer.latest = None;c.producer.process(None)
    c.node._station_compute_poll();c.task._station_task_check()
    assert c.producer.first_loss is not None or c.producer._station_compute_registration is None
    assert c.node._station_compute_retired and c.task._station_task_retired
    assert_no_plan(c)


@pytest.mark.parametrize('kind', ['plain', 'subclass'])
def test_same_reason_text_or_unproven_subclass_cannot_request_wait(monkeypatch, kind):
    c = owners(monkeypatch);row, _, _ = overlap_frame(c, monkeypatch, error_kind=kind)
    assert row['status']['state'] != 'station_measurement_pending'
    assert c.producer.first_loss is not None
    wire(c, row)
    assert c.node._station_compute_retired and c.task._station_task_retired
    assert_no_plan(c)


def test_one_status_cannot_claim_seed_and_overlap_receipts_together(monkeypatch):
    c = owners(monkeypatch);first, _, _ = seed_frame(c, monkeypatch)
    second, _, _ = overlap_frame(c, monkeypatch)
    mixed = deepcopy(second['status'])
    mixed['station_seed_insufficiency'] = first['status']['station_seed_insufficiency']
    with pytest.raises(ValueError, match='mixed_receipts'):
        measurement_rejection_receipt(mixed)
    forged = dict(second, status=mixed)
    wire(c, forged)
    assert c.node._station_compute_retired and c.task._station_task_retired
    assert_no_plan(c)


def test_completed_same_source_cannot_change_failure_kind(monkeypatch):
    c = owners(monkeypatch);row, source, _ = overlap_frame(c, monkeypatch)
    wire(c, row);assert_nonmetric_wait(c)
    changed = deepcopy(row)
    status = changed['status'];parent = status['station_compute_wait']
    del status['station_frame_rejection']
    status['station_seed_insufficiency'] = dict(
        seed_error(source[0].header.stamp.to_nsec(), (240, 320)).evidence,
        version=1, parent_source_ns=parent['source_ns'])
    status['detail'] = 'station_seed_measurement_insufficient'
    wire(c, changed, ('status',))
    assert c.node._station_compute_retired and c.task._station_task_retired
    assert_no_plan(c)


def test_overlap_cannot_bootstrap_without_registered_parent(monkeypatch):
    c = owners(monkeypatch)
    def remove_parent(_):
        c.producer._station_compute_registration = None
    row, _, _ = overlap_frame(c, monkeypatch, mutate=remove_parent)
    assert row['status']['state'] != 'station_measurement_pending'
    assert row['status'].get('station_frame_rejection') is None
    assert c.producer.first_loss is not None
    wire(c, row)
    assert c.node._station_compute_retired and c.task._station_task_retired
    assert_no_plan(c)
