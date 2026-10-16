"""Five-frame remote fuser, actual MRO publications and atomic audit transport."""
from copy import deepcopy
from types import SimpleNamespace as NS
import hashlib
import json
import threading

import pytest

from test_observation_measurement_remote import context, at
from test_station_compute_wait_remote import station, clock, feedback, row_at
from test_observation_station_snapshot_scope import submitted_observation
from test_station_compute_work_handoff import prepared_for, rgbd
from test_gen11_contact_remote_receipt import contact_plan
from test_remote_modes import base
from test_unknown_pending_identity import deliver
from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id
from alicia_grasp_modes.station_supplemental_admission import KEY, validate_receipt
from alicia_grasp_modes.observe_return_task import ObserveReturnTask


def published(c):
    n = c.node; t, row, ticket = submitted_observation(c)
    assert n._activate_prepared_geometry(prepared_for(n, ticket))
    plan = contact_plan(n, ticket)
    plan.diagnostic = 'FAR_FIELD_OBSERVATION_PLAN'; plan.fused_view_count = 5
    plan.plan_id = compute_plan_id(plan)
    legacy = base.PoseArray(); legacy.header = deepcopy(plan.header)
    n._publish_preview_plan(plan, legacy, ticket=ticket)
    assert n.preview_rich_plan_pub.messages[-1] == plan
    assert not n._station_compute_publication_started
    assert not n._station_compute_work_published
    return row, ticket, plan


def expire(c, row):
    parent = row['status']['station_compute_wait']
    clock(c, parent['geometry_deadline_ns'] + 20_000_000)
    c.node._station_compute_poll()
    return parent


def test_full_remote_publication_clock_handoff_and_delayed_atomic_audit(station, tmp_path):
    c = station; n = c.node; row, ticket, plan = published(c)
    old = deepcopy(plan); parent = expire(c, row)
    assert n._station_compute_pending and n._station_compute_soft_wait
    notice = n.preview_rich_plan_pub.messages[-1]
    assert notice.valid is False
    payload = json.loads(notice.diagnostic.partition(':')[2])
    assert payload['clock_only'] is True and payload['station_compute_wait'] == parent
    assert not n.latest_object_geometry.valid
    assert n._station_compute_work_certificate(ticket)
    # The actual atomic writer can commit a finished non-executable proposal
    # after a clock handoff; no timestamp, current geometry or deadline changes.
    path = tmp_path / 'supplement.json'
    n.gate_audit_output_path = str(path)
    n.mujoco_audit_output_path = str(tmp_path / 'mujoco.json')
    report = dict(plan_id=plan.plan_id, request_id=ticket.request_id,
                  planning_origin='observe_return_supplemental_observation', rows=[])
    n._write_gate_audit_report(report, ticket=ticket)
    payload_bytes = path.read_bytes(); report = json.loads(payload_bytes)
    assert validate_receipt(report[KEY], plan, now_ns=n._ros_source_clock_ns()) == c.binding
    assert len(report[KEY]['sources']) == 5
    assert report[KEY]['plan_source_ns'] == ticket.payload[0].stamp_ns
    assert report[KEY]['current']['contract'] == parent and plan == old
    sink = NS(_observe_return_condition=threading.Condition(), _observe_return_reports={},
              _observe_return_report_references={})
    ObserveReturnTask._observe_return_audit_cb(sink, NS(data=json.dumps(dict(
        report_path=str(path), report_sha256=hashlib.sha256(payload_bytes).hexdigest()))))
    assert sink._observe_return_reports[plan.plan_id][KEY] == report[KEY]
    # A complete measured successor remains required; the notice alone never
    # restores geometry or becomes an executable plan.
    ns = parent['source_ns'] + 900_000_000
    feedback(c, ns-10_000_000); feedback(c, ns+10_000_000); clock(c, ns+100_000_000)
    successor = row_at(c, ns, recovery=parent); rgbd(c, successor); deliver(n, successor)
    assert not n._station_compute_pending
    assert n._station_supplement_proposal['plan'] == old
    assert not n._station_compute_publication_started


@pytest.mark.parametrize('flag', ['_station_compute_publication_started', '_station_compute_work_published'])
def test_actual_authority_flag_never_permits_soft_supplement_retention(station, flag):
    c = station; n = c.node; row, ticket, plan = published(c)
    setattr(n, flag, True)
    expire(c, row)
    assert n._station_compute_pending and not n._station_compute_soft_wait
    assert n._station_supplement_proposal is None
    assert not n._station_compute_work_certificate(ticket)


@pytest.mark.parametrize('fault', ['manual', 'epoch', 'false', 'mask', 'deadline'])
def test_remote_supplement_does_not_survive_real_invalidation(station, fault):
    c = station; n = c.node; row, ticket, plan = published(c)
    if fault == 'manual': c.params['/gui/joint_direct_mode'] = True
    elif fault == 'epoch': n._measurement_remote_epoch_ns += 1
    elif fault == 'false':
        obj = deepcopy(row['object']); obj.detected = False; n.object_cb(obj)
    elif fault == 'mask':
        mask = deepcopy(row['mask']); mask.data = bytes(len(mask.data)); n.mask_cb(mask)
    else: clock(c, c.phase.deadline.to_nsec() + 1_000_000)
    n._station_compute_poll()
    assert n._station_supplement_proposal is None
    assert not n._station_compute_work_certificate(ticket)


def test_nonpromoting_token_keeps_private_view_and_sends_nothing(station, monkeypatch):
    c = station; n = c.node; row, ticket, plan = published(c)
    held = n._station_supplement_proposal
    monkeypatch.setattr(n, '_execution_promotion_audit_ready', lambda p: False)
    count = len(n.rich_plan_pub.messages)
    result = n._publish_execution_with_token(plan, None, 1., c.clock[0], ticket, {})
    assert not result.promote and result.code == 'PLAN_AUDIT_NOT_READY'
    assert n._station_supplement_proposal is held
    assert not n._station_compute_publication_started
    assert len(n.rich_plan_pub.messages) == count


@pytest.mark.parametrize('entry', ['cached_pair', 'worker_token'])
def test_execution_entry_closes_private_exception_before_first_send(station, monkeypatch, entry):
    c = station; n = c.node; row, ticket, plan = published(c)
    callbacks = []
    def first_send(message):
        callbacks.append(True)
        assert n._station_compute_publication_started
        assert n._station_supplement_proposal is None
        expire(c, row)
        assert not n._station_compute_soft_wait
        raise RuntimeError('inert publisher ends execution transaction')
    monkeypatch.setattr(n.plan_pub, 'publish', first_send)
    if entry == 'cached_pair':
        with pytest.raises(RuntimeError, match='inert publisher'):
            n._publish_plan_pair_if_current(plan, n._capture_geometry_generation())
    else:
        # Only the authority-token preconditions are inert; the real publication
        # MRO must close the exception before the first publisher callback.
        monkeypatch.setattr(n, '_execution_promotion_audit_ready', lambda p: True)
        monkeypatch.setattr(n, '_promotion_token_state', lambda *a: (True, 'CURRENT'))
        monkeypatch.setattr(n, '_promotion_token_current', lambda *a: (True, 'CURRENT'))
        monkeypatch.setattr(n, '_recover_aborted_execution_publish', lambda *a: None)
        result = n._publish_execution_with_token(plan, None, 1., c.clock[0], ticket, {})
        assert not result.promote
    assert callbacks == [True]
    assert not n._station_compute_work_certificate(ticket)


@pytest.mark.parametrize('first_reject', ['audit', 'token'])
def test_rejected_first_authority_check_cannot_become_a_send_on_second_check(station, monkeypatch, first_reject):
    c = station; n = c.node; row, ticket, plan = published(c)
    held = n._station_supplement_proposal; sends = []; audit_calls = []; token_calls = []
    def audit(p):
        audit_calls.append(True)
        return first_reject != 'audit' or len(audit_calls) > 1
    def token(*a):
        token_calls.append(True)
        return (first_reject != 'token' or len(token_calls) > 1, 'EXECUTION_FROZEN')
    monkeypatch.setattr(n, '_execution_promotion_audit_ready', audit)
    monkeypatch.setattr(n, '_promotion_token_state', token)
    monkeypatch.setattr(n, '_promotion_token_current', lambda *a: (True, 'CURRENT'))
    monkeypatch.setattr(n, '_recover_aborted_execution_publish', lambda *a: None)
    def send(m):
        sends.append(m)
        raise RuntimeError('unexpected inert send')
    monkeypatch.setattr(n.plan_pub, 'publish', send)
    result = n._publish_execution_with_token(plan, None, 1., c.clock[0], ticket, {})
    assert not result.promote and not sends
    assert n._station_supplement_proposal is held
    assert not n._station_compute_publication_started
