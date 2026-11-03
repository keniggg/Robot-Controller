"""Availability receipts retain provenance, never current motion authority."""
from copy import deepcopy
from itertools import permutations
import hashlib
import json

import pytest

from test_station_compute_wait_task import (
    station, configured, frame, deliver, advance, feedback, base, at_ns)
from test_arrival_feedback_gap_late_consumers import gap_receipt, rejected
from alicia_grasp_modes.station_contact_admission import source_witness


ORDERS = list(permutations(('object', 'mask', 'status')))


def begin(c):
    original = frame(c)
    deliver(c, original)
    parent = deepcopy(c.node._station_task_record['contract'])
    bad_ns = parent['source_ns'] + 250_000_000
    receipt = gap_receipt(parent, bad_ns)
    gap = rejected(frame(c, bad_ns), parent, receipt)
    gap['status']['discarded_source_stamp_ns'] = bad_ns
    # Both rejected SDK endpoints must already exist; the late endpoint is
    # bad_ns+196932289ns in this fixed original gap fixture.
    advance(c, bad_ns + 250_000_000)
    return original, parent, gap, receipt


def invalid_geometry(c, parent, notice):
    msg = base.ObjectGeometry()
    msg.header.stamp = at_ns(c.now[0]); msg.header.frame_id = 'base_link'
    msg.target_track_id = parent['binding']['target_track_id']; msg.valid = False
    msg.failure_reason = 'TARGET_PROCESSING_PENDING: ' + json.dumps(dict(
        station_compute_wait=parent,
        station_compute_recovery_chain=notice.get('station_compute_recovery_chain'),
        clock_only=False, station_pending_notice=deepcopy(notice)))
    return msg


def fresh(c, parent, receipt):
    ns = parent['source_ns'] + 600_000_000
    advance(c, ns + 50_000_000)
    feedback(c, ns - 10_000_000); feedback(c, ns + 10_000_000)
    row = frame(c, ns, parent)
    row['status']['station_feedback_gap_recovery'] = deepcopy(receipt)
    return row


@pytest.mark.parametrize('order', ORDERS)
def test_typed_gap_public_geometry_preserves_original_provenance_only(station, order):
    c = station; original, parent, gap, receipt = begin(c); n = c.node
    source = deepcopy(n._station_contact_sources[parent['source_ns']])
    assert n._station_compute_task_geometry(invalid_geometry(c, parent, gap['status']))
    assert n._station_contact_hard_cutoff == 0
    assert n._station_task_record['pending'] and n.latest_obj is None
    deliver(c, gap, order)
    assert n._station_contact_hard_cutoff == 0
    assert n._station_contact_sources[parent['source_ns']] == source
    assert n._station_task_record['pending']
    row = fresh(c, parent, receipt); deliver(c, row, order)
    assert not n._station_task_record['pending']
    assert n._station_task_record['contract'] == row['status']['station_compute_wait']
    assert n._station_contact_hard_cutoff == 0
    assert n._station_contact_sources[parent['source_ns']] == source
    assert n._station_contact_admitted is None


@pytest.mark.parametrize('order', ORDERS)
def test_complete_recovery_proof_does_not_wait_for_old_empty_topics(station, order):
    c = station; _, parent, gap, receipt = begin(c); n = c.node
    row = fresh(c, parent, receipt); deliver(c, row, order)
    assert not n._station_task_retired and not n._station_task_record['pending']
    assert n._station_task_record['contract']['source_ns'] == row['status']['source_stamp_ns']
    frozen = deepcopy(n._station_task_record)
    # Late failed topics and duplicate current READY cannot refresh clocks or
    # resurrect failed input. Every current source was independently joined.
    deliver(c, gap, order); deliver(c, row, order)
    assert n._station_task_record == frozen
    assert not n._station_task_retired and n._station_contact_hard_cutoff == 0
    assert n._station_contact_admitted is None


@pytest.mark.parametrize('fault', [
    'notice_owner', 'notice_support', 'true_object', 'nonempty_mask',
    'changed_original_mask', 'recovery_proof', 'control_epoch', 'manual'])
def test_real_input_or_owner_change_cannot_be_recovered_as_availability(station, fault):
    c = station; original, parent, gap, receipt = begin(c); n = c.node
    if fault == 'notice_owner':
        gap['status']['station_compute_wait']['binding']['lease_id'] = 'changed'
        n._station_compute_task_geometry(invalid_geometry(c, parent, gap['status']))
    elif fault == 'notice_support':
        gap['status']['reference_support_plane_base'][3] += .01
        n._station_compute_task_geometry(invalid_geometry(c, parent, gap['status']))
    elif fault == 'changed_original_mask':
        original['mask'].data = bytes([0,255,255,255,255,255])
        deliver(c, original, ('mask',))
    elif fault in ('control_epoch', 'manual'):
        if fault == 'control_epoch': n._following_epoch_ns += 1
        else: c.params['/gui/joint_direct_mode'] = True
        n._station_compute_task_geometry(invalid_geometry(c, parent, gap['status']))
    else:
        n._station_compute_task_geometry(invalid_geometry(c, parent, gap['status']))
        if fault == 'true_object':
            gap['object'].detected = True; deliver(c, gap, ('object',))
        elif fault == 'nonempty_mask':
            gap['mask'].data = bytes([255]*6); deliver(c, gap, ('mask',))
        else:
            receipt['discarded_source_evidence'][0]['feedback_gap_evidence']['span_ns'] -= 1
            row = fresh(c, parent, receipt); deliver(c, row)
    assert n._station_task_retired
    assert n._station_contact_hard_cutoff > 0 and n._station_contact_admitted is None
    # Good later input does not undo a real hard cutoff or a retired owner.
    good = fresh(c, parent, gap_receipt(parent, gap['status']['source_stamp_ns']))
    deliver(c, good)
    assert n._station_task_retired and n._station_contact_admitted is None


@pytest.mark.parametrize('fragment', ['object', 'mask'])
def test_unexplained_partial_input_wait_is_bounded_by_original_identity(station, fragment):
    c = station; _, parent, gap, _ = begin(c); n = c.node
    deliver(c, gap, (fragment,))
    assert n._station_task_record['pending'] and n.latest_obj is None
    assert n._station_contact_hard_cutoff == 0
    advance(c, parent['identity_deadline_ns'])
    n._station_compute_task_tick()
    assert n._station_task_retired and n._station_contact_hard_cutoff > 0
    assert n._station_contact_admitted is None


def test_legacy_clock_false_without_typed_notice_keeps_hard_cutoff(station):
    c = station; _, parent, gap, _ = begin(c); n = c.node
    msg = invalid_geometry(c, parent, gap['status'])
    payload = json.loads(msg.failure_reason.partition(':')[2])
    payload.pop('station_pending_notice')
    msg.failure_reason = 'TARGET_PROCESSING_PENDING: ' + json.dumps(payload)
    assert n._station_compute_task_geometry(msg)
    assert n._station_task_record['pending']
    assert n._station_contact_hard_cutoff == c.now[0]


def test_ready_notice_geometry_is_only_causal_proof_not_registration(station):
    c = station; _, parent, _, receipt = begin(c); n = c.node
    row = fresh(c, parent, receipt)
    msg = invalid_geometry(c, parent, row['status'])
    assert n._station_compute_task_geometry(msg)
    assert n._station_task_record['contract'] == parent
    assert n._station_task_record['pending'] and n.latest_obj is None
    assert n._station_contact_hard_cutoff == 0
    deliver(c, row)
    assert not n._station_task_record['pending']
    assert n._station_contact_admitted is None


@pytest.mark.parametrize('field', ['generation', 'selection_stamp_ns'])
def test_unrelated_status_does_not_rewrite_or_retire_registered_owner(station, field):
    c = station; _, parent, gap, _ = begin(c); n = c.node
    before = deepcopy(n._station_task_record)
    gap['status'][field] += 1
    deliver(c, gap, ('status',))
    assert not n._station_task_retired and n._station_task_record == before
    assert n._station_contact_hard_cutoff == 0


def partial_geometry(c, parent, gap, kind):
    msg = invalid_geometry(c, parent, gap['status'])
    payload = json.loads(msg.failure_reason.partition(':')[2])
    payload.pop('station_pending_notice')
    marker = dict(version=1, kind='unqualified_station_input',
        source_ns=gap['status']['source_stamp_ns'], input_kind=kind,
        parent_contract=deepcopy(parent), source_qualified=False,
        identity_verified=False, metric_geometry_authority=False,
        motion_authorized=False, reference_refreshed=False)
    if kind == 'object':
        marker['object_sha256'] = source_witness(parent, gap['object'], {})['object_sha256']
    else:
        marker['mask'] = dict(sha256=hashlib.sha256(bytes(6)).hexdigest(),
                             shape=[2,3], frame_id='camera')
    payload['station_pending_input'] = marker
    msg.failure_reason = 'TARGET_PROCESSING_PENDING: ' + json.dumps(payload)
    return msg


@pytest.mark.parametrize('kind', ['object', 'mask'])
@pytest.mark.parametrize('local_first', [False, True])
def test_partial_marker_only_holds_until_complete_causal_fresh_recovery(station, kind, local_first):
    c = station; _, parent, gap, receipt = begin(c); n = c.node
    if local_first: deliver(c, gap, (kind,))
    assert n._station_compute_task_geometry(partial_geometry(c, parent, gap, kind))
    if not local_first: deliver(c, gap, (kind,))
    assert n._station_task_record['contract'] == parent
    assert n._station_task_record['pending'] and n.latest_obj is None
    assert n._station_contact_hard_cutoff == 0
    # Complete fresh causal proof is sufficient even if other obsolete empty
    # gap topics are still absent; it does not replace this new source's SDK.
    row = fresh(c, parent, receipt); deliver(c, row)
    assert not n._station_task_retired and not n._station_task_record['pending']
    assert n._station_contact_hard_cutoff == 0 and n._station_contact_admitted is None


@pytest.mark.parametrize('kind', ['object', 'mask'])
def test_partial_marker_hash_must_match_actual_local_input(station, kind):
    c = station; _, parent, gap, _ = begin(c); n = c.node
    msg = partial_geometry(c, parent, gap, kind)
    if kind == 'object':
        assert n._station_compute_task_geometry(msg)
        gap['object'].confidence -= .01
        deliver(c, gap, ('object',))
    else:
        payload = json.loads(msg.failure_reason.partition(':')[2])
        payload['station_pending_input']['mask']['sha256'] = 'f'*64
        msg.failure_reason = 'TARGET_PROCESSING_PENDING: ' + json.dumps(payload)
        assert n._station_compute_task_geometry(msg)
    assert n._station_task_retired and n._station_contact_hard_cutoff > 0
    assert n._station_contact_admitted is None


@pytest.mark.parametrize('kind', ['object', 'mask'])
def test_partial_marker_without_causal_proof_cannot_outlive_original_identity(station, kind):
    c = station; _, parent, gap, _ = begin(c); n = c.node
    assert n._station_compute_task_geometry(partial_geometry(c, parent, gap, kind))
    deliver(c, gap, (kind,))
    advance(c, parent['identity_deadline_ns']); n._station_compute_task_tick()
    assert n._station_task_retired and n._station_contact_hard_cutoff > 0
    assert n._station_contact_admitted is None


def test_partial_marker_never_undoes_a_real_preexisting_hard_cutoff(station):
    c = station; _, parent, gap, _ = begin(c); n = c.node
    n._station_contact_invalidate(c.now[0]); cutoff = n._station_contact_hard_cutoff
    assert n._station_compute_task_geometry(partial_geometry(c, parent, gap, 'object'))
    assert n._station_contact_hard_cutoff == cutoff
    assert n._station_contact_admitted is None


@pytest.mark.parametrize('marker_first', [False, True])
def test_complete_pending_proof_does_not_require_obsolete_empty_topics(station, marker_first):
    c = station; _, parent, gap, receipt = begin(c); n = c.node
    if marker_first:
        assert n._station_compute_task_geometry(partial_geometry(c, parent, gap, 'object'))
    deliver(c, gap, ('status',))
    assert n._station_task_record['pending'] and n.latest_obj is None
    assert not n._station_task_availability_pending
    row = fresh(c, parent, receipt)
    # The gap was already fully explained by its original pending status.
    # Repeating it in READY is optional; the fresh join's original parent and
    # SDK evidence are still mandatory.
    row['status'].pop('station_feedback_gap_recovery')
    deliver(c, row)
    assert not n._station_task_retired and not n._station_task_record['pending']
    deliver(c, gap, ('object', 'mask'))
    assert not n._station_task_retired and n._station_contact_hard_cutoff == 0
    assert n._station_contact_admitted is None


def ordinary_overrun(c):
    """A rejected newer frame, distinct from age of the registered READY."""
    original = frame(c); deliver(c, original)
    parent = deepcopy(c.node._station_task_record['contract'])
    bad_ns = parent['source_ns'] + 250_000_000
    bad = frame(c, bad_ns)
    bad['object'].detected = False; bad['mask'].data = bytes(6)
    bad['status'].update(state='processing_deadline_pending',
        detail='processing_deadline_pending', station_compute_wait=deepcopy(parent),
        station_compute_recovery_chain=[deepcopy(parent)],
        discarded_source_stamp_ns=bad_ns)
    advance(c, parent['source_ns'] + 500_000_000)
    return parent, bad


def ordinary_recovery(c, parent):
    ns = max(parent['source_ns'] + 600_000_000, c.now[0] + 100_000_000)
    advance(c, ns + 50_000_000)
    feedback(c, ns - 10_000_000); feedback(c, ns + 10_000_000)
    return frame(c, ns, parent)


@pytest.mark.parametrize('order', ORDERS)
def test_new_frame_overrun_keeps_legacy_hard_cutoff_and_fresh_recovery(station, order):
    c = station; parent, bad = ordinary_overrun(c); n = c.node
    deliver(c, bad, order)
    assert not n._station_task_retired and n._station_task_record['pending']
    assert n._station_task_record['contract'] == parent and n.latest_obj is None
    cutoff = n._station_contact_hard_cutoff
    assert cutoff >= bad['status']['source_stamp_ns']
    row = ordinary_recovery(c, parent); deliver(c, row, order)
    assert not n._station_task_retired and not n._station_task_record['pending']
    assert n._station_task_record['contract'] == row['status']['station_compute_wait']
    assert n.latest_obj.header.stamp == row['object'].header.stamp
    # A new metric source can restore observation, never the invalidated old
    # calculation's execution authority or its preexisting hard cutoff.
    assert n._station_contact_hard_cutoff == cutoff and n._station_contact_admitted is None


@pytest.mark.parametrize('fault', ['changed_parent', 'original_identity_deadline'])
def test_new_frame_overrun_scope_never_recovers_bad_parent_or_expiry(station, fault):
    c = station; parent, bad = ordinary_overrun(c); n = c.node
    if fault == 'changed_parent':
        bad['status']['station_compute_wait']['binding']['lease_id'] = 'changed'
        deliver(c, bad, ('status', 'object', 'mask'))
    else:
        deliver(c, bad)
        advance(c, parent['identity_deadline_ns']); n._station_compute_task_tick()
    assert n._station_task_retired and n._station_contact_hard_cutoff > 0
    row = ordinary_recovery(c, parent); deliver(c, row)
    assert n._station_task_retired and n._station_contact_admitted is None


def _fresh_after_original_parent_deadline(c, parent, receipt):
    row = fresh(c, parent, receipt); deliver(c, row)
    current = deepcopy(c.node._station_task_record['contract'])
    ns = parent['identity_deadline_ns'] + 100_000_000
    assert ns + 50_000_000 < current['identity_deadline_ns']
    # Advance the original fixture clock before each real SDK callback.
    advance(c, ns - 10_000_000); advance(c, ns + 10_000_000)
    advance(c, ns + 50_000_000)
    newest = frame(c, ns, current); deliver(c, newest)
    assert c.now[0] > parent['identity_deadline_ns']
    assert not c.node._station_task_record['pending']
    assert c.node._station_task_record['contract'] == newest['status']['station_compute_wait']
    return newest


def test_late_proven_gap_past_old_identity_cannot_clear_fresh_current(station):
    c = station; _, parent, gap, receipt = begin(c); n = c.node
    newest = _fresh_after_original_parent_deadline(c, parent, receipt)
    frozen = deepcopy(n._station_task_record)
    source = deepcopy(n._station_contact_sources[parent['source_ns']])
    deliver(c, gap, ('status',))
    assert not n._station_task_retired and n._station_task_record == frozen
    assert n.latest_obj.header.stamp == newest['object'].header.stamp
    assert n._station_contact_sources[parent['source_ns']] == source
    assert n._station_contact_hard_cutoff == 0 and n._station_contact_admitted is None


def test_changed_late_gap_past_old_identity_is_still_permanent_hard_loss(station):
    c = station; _, parent, gap, receipt = begin(c); n = c.node
    _fresh_after_original_parent_deadline(c, parent, receipt)
    bad = deepcopy(gap)
    bad['status']['station_feedback_gap']['feedback_gap_evidence']['span_ns'] += 1
    bad['status']['station_feedback_gap']['discarded_source_evidence'][0]['feedback_gap_evidence']['span_ns'] += 1
    deliver(c, bad, ('status',))
    assert n._station_task_retired and n._station_task_record is None
    assert n._station_contact_hard_cutoff > 0 and n._station_contact_admitted is None
    assert n._execution_authority_revoked and n.latest_grasp6d_plan is None
