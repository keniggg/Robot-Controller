"""Independent supplemental callback race and rejection tests; no motion or ROS I/O."""
from copy import deepcopy
import hashlib
import json
from types import SimpleNamespace as NS

import pytest

from test_station_compute_wait_task import (
    station, configured, frame, deliver, advance, feedback, base, at_ns, module)
from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id
from alicia_flexible_grasp.robot.observation_path_guard import committed_plan_digest
from alicia_grasp_modes.station_contact_admission import source_witness

KEY = 'station_supplemental_observation_computation'


def proposal(c):
    witnesses = []
    parent = None
    for stamp in (100_000_000_000, 100_400_000_000, 100_800_000_000,
                  101_200_000_000, 101_600_000_000):
        advance(c, stamp + 50_000_000)
        feedback(c, stamp - 20_000_000)
        feedback(c, stamp + 20_000_000)
        row = frame(c, stamp, parent)
        deliver(c, row)
        parent = deepcopy(row['status']['station_compute_wait'])
        witnesses.append(deepcopy(c.node._station_contact_sources[stamp]))
    plan = deepcopy(c.plan)
    plan.header.stamp = at_ns(101_600_000_000)
    plan.object_geometry.header = deepcopy(plan.header)
    plan.plan_id = compute_plan_id(plan)
    receipt = dict(version=1, scope='registered_station_supplemental_observation_provenance',
        motion_authorized=False, metric_geometry_authority=False,
        plan_id=plan.plan_id, plan_sha256=committed_plan_digest(plan),
        plan_source_ns=plan.header.stamp.to_nsec(), issued_ns=c.now[0],
        ticket=dict(request_id=23, generation=9, target_epoch=4),
        sources=witnesses, current=deepcopy(witnesses[-1]),
        snapshot_policy=dict(purpose='supplementary_observation', phase='observation_prefix',
                             frame_count=5, maximum_span_sec=12.))
    report = dict(plan_id=plan.plan_id, observe_return=dict(phase='supplemental_observation'))
    report[KEY] = receipt
    c.node._observe_return_session['max_observations'] = 3
    return plan, report, parent


def audit(c, report, tmp_path, *, corrupt_bytes=False):
    """Deliver the real audit reference callback, including its file hash check."""
    payload = json.dumps(report, sort_keys=True).encode()
    path = tmp_path / ('audit_%d.json' % len(list(tmp_path.iterdir())))
    path.write_bytes(payload + (b' ' if corrupt_bytes else b''))
    ref = dict(report_path=str(path), report_sha256=hashlib.sha256(payload).hexdigest())
    c.node._observe_return_audit_cb(NS(data=json.dumps(ref)))


def pending_preview(c, parent):
    msg = base.Grasp6DPlan()
    msg.header.stamp = at_ns(c.now[0])
    msg.header.frame_id = 'base_link'
    msg.valid = False
    msg.candidate_source = 'near_field_pending'
    msg.target_track_id = c.plan.target_track_id
    msg.diagnostic = 'TARGET_PROCESSING_PENDING: ' + json.dumps(dict(
        station_compute_wait=parent, station_compute_recovery_chain=None, clock_only=True))
    c.node.grasp6d_preview_plan_cb(msg)
    assert c.node.latest_grasp6d_preview_plan.valid is False


def gap(c, parent):
    # The recorded race was roughly 8.2 ms across the original geometry TTL.
    advance(c, parent['geometry_deadline_ns'] + 8_198_766)
    pending_preview(c, parent)
    assert c.node._station_task_record['pending']


def ready(c, parent, order=('object', 'mask', 'status')):
    stamp = 102_500_000_000
    advance(c, stamp + 50_000_000)
    feedback(c, stamp - 20_000_000)
    feedback(c, stamp + 20_000_000)
    row = frame(c, stamp, parent)
    deliver(c, row, order)
    return row


def admission(c, monkeypatch):
    """Use the real consumer; its first original admission gate is our stop."""
    calls = []
    monkeypatch.setattr(c.node, '_diagnostic_observation_checkpoint',
                        lambda plan, cfg, label: calls.append(label) or False)
    def forbidden(*args, **kwargs):
        pytest.fail('An offline provenance test reached a motion callback')
    result = c.node._before_near_field_terminal_recovery(None, c.plan, {}, forbidden, forbidden)
    assert c.node._bound_execution_plan == c.plan
    return result, calls


@pytest.mark.parametrize('when', ['before_preview', 'during_gap', 'after_ready'])
@pytest.mark.parametrize('order', [('object', 'mask', 'status'), ('status', 'mask', 'object')])
def test_real_async_callbacks_preserve_proposal_only_until_original_gate(
        station, monkeypatch, tmp_path, when, order):
    c = station
    plan, report, parent = proposal(c)
    original_digest = committed_plan_digest(plan)
    if when == 'before_preview':
        audit(c, report, tmp_path)
    c.node.grasp6d_preview_plan_cb(plan)
    gap(c, parent)
    if when == 'during_gap':
        audit(c, report, tmp_path)
    result, calls = admission(c, monkeypatch)
    assert result == {'waiting': True} and calls == []
    ready(c, parent, order)
    if when == 'after_ready':
        audit(c, report, tmp_path)
    result, calls = admission(c, monkeypatch)
    assert calls == ['supplemental observation admission']
    assert result == {'failed': True}  # Original gate deliberately stops here.
    assert c.node.latest_grasp6d_preview_plan.valid is False
    assert committed_plan_digest(plan) == original_digest
    assert plan.header.stamp.to_nsec() == 101_600_000_000


@pytest.mark.parametrize('fault', ['session', 'lease', 'selection', 'control_epoch', 'manual'])
def test_owner_change_cannot_reuse_private_proposal(station, monkeypatch, tmp_path, fault):
    c = station
    plan, report, parent = proposal(c)
    c.node.grasp6d_preview_plan_cb(plan)
    gap(c, parent)
    ready(c, parent)
    audit(c, report, tmp_path)
    if fault == 'session': c.node._observe_return_session['session_id'] = 'foreign'
    elif fault == 'lease': c.node._observation_measurement_request['lease_id'] = 'foreign'
    elif fault == 'selection':
        c.params[module.SELECTION_PARAM] = dict(c.selection, generation=c.selection['generation'] + 1)
    elif fault == 'control_epoch': c.node._following_epoch_ns += 1
    else: c.params['/gui/joint_direct_mode'] = True
    _, calls = admission(c, monkeypatch)
    assert calls == []


@pytest.mark.parametrize('fault', ['file_sha', 'plan_digest', 'source_mask', 'source_object',
                                   'authority', 'policy', 'receipt_owner'])
def test_audit_bytes_and_semantic_tampering_cannot_admit(station, monkeypatch, tmp_path, fault):
    c = station
    plan, report, parent = proposal(c)
    c.node.grasp6d_preview_plan_cb(plan)
    gap(c, parent)
    ready(c, parent)
    receipt = report[KEY]
    if fault == 'plan_digest': receipt['plan_sha256'] = '0' * 64
    elif fault == 'source_mask': receipt['sources'][0]['mask']['sha256'] = '0' * 64
    elif fault == 'source_object': receipt['sources'][0]['object_sha256'] = '0' * 64
    elif fault == 'authority': receipt['motion_authorized'] = True
    elif fault == 'policy': receipt['snapshot_policy']['frame_count'] = 4
    elif fault == 'receipt_owner': receipt['current']['contract']['binding']['lease_id'] = 'foreign'
    audit(c, report, tmp_path, corrupt_bytes=(fault == 'file_sha'))
    _, calls = admission(c, monkeypatch)
    assert calls == []


@pytest.mark.parametrize('delivered', [(), ('object',), ('object', 'mask'), ('status', 'mask')])
def test_recovered_rgbd_requires_complete_fresh_ready_join(station, monkeypatch, tmp_path, delivered):
    c = station
    plan, report, parent = proposal(c)
    c.node.grasp6d_preview_plan_cb(plan)
    gap(c, parent)
    audit(c, report, tmp_path)
    ready(c, parent, delivered)
    result, calls = admission(c, monkeypatch)
    assert calls == [] and result == {'waiting': True}


@pytest.mark.parametrize('newer', ['contact', 'foreign_view', 'hard_invalid'])
def test_new_preview_never_resurrects_older_supplement(station, monkeypatch, tmp_path, newer):
    c = station
    plan, report, parent = proposal(c)
    c.node.grasp6d_preview_plan_cb(plan)
    gap(c, parent)
    ready(c, parent)
    audit(c, report, tmp_path)
    replacement = deepcopy(plan)
    if newer == 'hard_invalid':
        replacement.valid = False
        replacement.diagnostic = 'TARGET_LOST: real target loss'
    elif newer == 'contact': replacement.diagnostic = 'CONTACT_EXECUTION_PLAN'
    else:
        replacement.target_track_id += '_foreign'
        replacement.object_geometry.target_track_id = replacement.target_track_id
        replacement.plan_id = compute_plan_id(replacement)
    c.node.grasp6d_preview_plan_cb(replacement)
    _, calls = admission(c, monkeypatch)
    assert calls == []
    assert c.node.latest_grasp6d_preview_plan == replacement


def test_actual_hard_loss_callback_and_later_ready_cannot_resurrect(station, monkeypatch, tmp_path):
    c = station
    plan, report, parent = proposal(c)
    c.node.grasp6d_preview_plan_cb(plan)
    gap(c, parent)
    row = ready(c, parent)
    audit(c, report, tmp_path)
    lost = deepcopy(row['status'])
    lost.update(target_lost=True, state='unavailable')
    c.node._unknown_support_status_cb(NS(data=json.dumps(lost)))
    deliver(c, row)
    _, calls = admission(c, monkeypatch)
    assert calls == []
    assert c.node._station_task_retired and c.node._station_task_record is None


def test_newer_preview_then_older_clock_never_revives_previous_plan(station, monkeypatch, tmp_path):
    c = station
    plan, report, parent = proposal(c)
    c.node.grasp6d_preview_plan_cb(plan)
    gap(c, parent)
    old_notice = deepcopy(c.node.latest_grasp6d_preview_plan)
    ready(c, parent)
    audit(c, report, tmp_path)  # Late audit arrives after new READY.
    newer = deepcopy(plan)
    newer.header.stamp = at_ns(102_500_000_000)
    newer.object_geometry.header = deepcopy(newer.header)
    newer.plan_id = compute_plan_id(newer)
    assert newer.plan_id != plan.plan_id
    c.node.grasp6d_preview_plan_cb(newer)
    assert old_notice.header.stamp.to_nsec() < c.now[0]
    c.node.grasp6d_preview_plan_cb(old_notice)
    _, calls = admission(c, monkeypatch)
    assert calls == []
    assert c.node.latest_grasp6d_preview_plan.valid is False
    # A stale notice need not preserve newer work, but must never select the
    # audited old plan or treat either computation as execution permission.
    assert c.node._bound_execution_plan == c.plan


def test_same_plan_id_different_geometry_cannot_borrow_original_audit(station, monkeypatch, tmp_path):
    c = station
    plan, report, parent = proposal(c)
    altered = deepcopy(plan)
    altered.object_geometry.pose_base.position.x += .01
    assert altered.plan_id == plan.plan_id
    assert committed_plan_digest(altered) != committed_plan_digest(plan)
    c.node.grasp6d_preview_plan_cb(altered)
    gap(c, parent)
    ready(c, parent)
    audit(c, report, tmp_path)
    _, calls = admission(c, monkeypatch)
    assert calls == []


@pytest.mark.parametrize('clock', ['phase_deadline', 'session_deadline', 'identity_wall', 'missing_current_feedback'])
def test_retention_does_not_extend_original_clocks(station, monkeypatch, tmp_path, clock):
    c = station
    plan, report, parent = proposal(c)
    c.node.grasp6d_preview_plan_cb(plan)
    gap(c, parent)
    ready(c, parent)
    audit(c, report, tmp_path)
    if clock == 'phase_deadline': advance(c, parent['phase_deadline_ns'])
    elif clock == 'session_deadline': advance(c, parent['session_deadline_ns'])
    elif clock == 'identity_wall': c.mono[0] += 5.
    else: c.now[0] += 800_000_000
    _, calls = admission(c, monkeypatch)
    assert calls == []


@pytest.mark.parametrize('fault', ['not_clock', 'foreign_parent', 'missing_witness', 'missing_audit'])
def test_clock_notice_and_original_evidence_are_mandatory(station, monkeypatch, tmp_path, fault):
    c = station
    plan, report, parent = proposal(c)
    c.node.grasp6d_preview_plan_cb(plan)
    gap(c, parent)
    notice = deepcopy(c.node.latest_grasp6d_preview_plan)
    if fault in ('not_clock', 'foreign_parent'):
        payload = json.loads(notice.diagnostic.partition(':')[2])
        if fault == 'not_clock': payload['clock_only'] = False
        else: payload['station_compute_wait']['binding']['lease_id'] = 'foreign'
        notice.diagnostic = 'TARGET_PROCESSING_PENDING: ' + json.dumps(payload)
        c.node.grasp6d_preview_plan_cb(notice)
    ready(c, parent)
    if fault != 'missing_audit': audit(c, report, tmp_path)
    if fault == 'missing_witness':
        del c.node._station_contact_sources[report[KEY]['sources'][0]['contract']['source_ns']]
    _, calls = admission(c, monkeypatch)
    assert calls == []


@pytest.mark.parametrize('order', [('object', 'mask', 'status'), ('status', 'mask', 'object')])
def test_audit_current_arrives_before_local_join_waits_for_exact_source(
        station, monkeypatch, tmp_path, order):
    c = station
    plan, report, parent = proposal(c)
    c.node.grasp6d_preview_plan_cb(plan)
    gap(c, parent)
    local_ready = ready(c, parent)
    stamp = 102_650_000_000
    advance(c, stamp + 50_000_000)
    feedback(c, stamp - 20_000_000)
    feedback(c, stamp + 20_000_000)
    incoming = frame(c, stamp, local_ready['status']['station_compute_wait'])
    report[KEY]['current'] = source_witness(incoming['status']['station_compute_wait'],
        incoming['object'], incoming['status']['station_compute_mask'])
    report[KEY]['issued_ns'] = c.now[0]
    audit(c, report, tmp_path)
    assert stamp not in c.node._station_contact_sources
    _, calls = admission(c, monkeypatch)
    assert calls == []
    # A valid report is not proof that this consumer received its RGB-D join.
    deliver(c, incoming, order[:-1])
    _, calls = admission(c, monkeypatch)
    assert calls == []
    deliver(c, incoming, order[-1:])
    assert c.node._station_contact_sources[stamp] == report[KEY]['current']
    result, calls = admission(c, monkeypatch)
    assert result == {'failed': True}
    assert calls == ['supplemental observation admission']
    assert c.node.latest_grasp6d_preview_plan.valid is False
