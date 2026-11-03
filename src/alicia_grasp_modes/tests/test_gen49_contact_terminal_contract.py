"""Recorded G48 negative-result schema plus actual registered-work publication.

The recorded fixture is a counterfactual protocol carrier, not a historical
typed receipt. Remote integration uses original synthetic RGB-D/SDK providers;
no MoveIt trajectory or physical motion is reconstructed by this file.
"""
from copy import deepcopy
import hashlib
import json
from pathlib import Path

import pytest

from alicia_grasp_modes.observation_search_failure import (
    CONTACT_CODE, CONTACT_SOURCE, validate_contact_search_failure)
from test_observation_measurement_remote import context
from test_station_compute_wait_remote import station, clock, feedback
from test_station_compute_work_handoff import submitted, prepared_for


FIXTURE = Path(__file__).parent/'fixtures/gen49_g48_terminal_inputs.json'
FIXTURE_SHA = 'f924807ea25c78d6bb36dc15c1c1433428ec615c2b0b3fd391f9ba694db60119'


def recorded():
    raw = FIXTURE.read_bytes()
    assert hashlib.sha256(raw).hexdigest() == FIXTURE_SHA
    return json.loads(raw)


def test_exact_g48_completed_search_is_only_a_bound_negative_result(monkeypatch):
    # Exact historical wire contracts retain their original 5-second policy.
    # Current 10-second coverage is separate; the fixture bytes stay unchanged.
    from alicia_grasp_modes import station_compute_reacquisition as historical_policy
    monkeypatch.setattr(historical_policy, 'IDENTITY_WAIT_NS', 5_000_000_000)
    data = recorded(); receipt = data['counterfactual_typed_receipt']
    now = data['original_terminal_wire']['received_ns']
    contract = receipt['current']['contract']
    assert contract['geometry_deadline_ns'] < now < contract['identity_deadline_ns']
    checked = validate_contact_search_failure(receipt, contract,
        now_ns=now, header_ns=receipt['emitted_ns'])
    assert checked == receipt and checked is not receipt
    assert receipt['request']['request_id'] == 22
    assert receipt['request']['generation'] == 7
    assert receipt['binding']['selection']['generation'] == 8  # Separate namespaces.
    assert [r['contract']['source_ns'] for r in receipt['sources']] == [
        1792317082624930858, 1792317083490865230, 1792317083889718770]
    assert receipt['execution_authority'] is False
    assert receipt['metric_geometry_authority'] is False
    assert data['counterfactual_receipt_was_recorded'] is False
    assert data['next_original_clock_wire']['received_ns']-now == 256203890


@pytest.mark.parametrize('fault', ['policy', 'selection', 'phase', 'session_deadline',
    'source_order', 'source_count', 'snapshot', 'incomplete_search', 'reachable',
    'early', 'execution', 'metric', 'audit', 'deadline', 'future'])
def test_recorded_failure_schema_cannot_change_owner_sources_result_or_time(fault,monkeypatch):
    # Exact historical wire contracts retain their original 5-second policy.
    # Current 10-second coverage is separate; the fixture bytes stay unchanged.
    from alicia_grasp_modes import station_compute_reacquisition as historical_policy
    monkeypatch.setattr(historical_policy, 'IDENTITY_WAIT_NS', 5_000_000_000)
    receipt = recorded()['counterfactual_typed_receipt']
    contract = deepcopy(receipt['current']['contract'])
    now = receipt['emitted_ns']+1_000_000; header = receipt['emitted_ns']
    if fault == 'policy': receipt['policy'] = 'unknown'
    elif fault == 'selection': receipt['binding']['selection']['generation'] += 1
    elif fault == 'phase': receipt['phase_deadline_ns'] += 1
    elif fault == 'session_deadline': receipt['session_deadline_ns'] += 1
    elif fault == 'source_order': receipt['sources'].reverse()
    elif fault == 'source_count': receipt['sources'].pop()
    elif fault == 'snapshot': receipt['request']['snapshot_stamp_ns'] += 1
    elif fault == 'incomplete_search': receipt['search']['shortlist_count'] += 1
    elif fault == 'reachable': receipt['search']['reachable_count'] = 1
    elif fault == 'early': receipt['search']['terminated_early'] = True
    elif fault == 'execution': receipt['execution_authority'] = True
    elif fault == 'metric': receipt['metric_geometry_authority'] = True
    elif fault == 'audit': receipt['audit_sha256'] = 'z'*64
    elif fault == 'deadline': now = contract['phase_deadline_ns']
    elif fault == 'future': now = receipt['emitted_ns']-1
    with pytest.raises(ValueError):
        validate_contact_search_failure(receipt, contract, now_ns=now, header_ns=header)


def commit(n, prepared, *, complete=True):
    ticket = prepared.ticket
    selection = dict(checked_count=3, shortlist_count=3 if complete else 4,
        reachable_count=0, terminated_early=False)
    report = dict(request_id=ticket.request_id, generation=ticket.generation,
        target_epoch=ticket.target_epoch, snapshot_stamp_ns=prepared.snapshot.stamp_ns,
        selected=None, outcome=dict(code=CONTACT_CODE, valid_plan=False, preview_valid=False),
        moveit_selection=selection)
    n._station_contact_terminal_audit(report, dict(report_sha256='a'*64), ticket)
    return report


@pytest.mark.parametrize('pending', [False, True])
def test_real_three_source_work_emits_failure_without_refresh_or_authority(station, pending):
    c = station; n = c.node
    _, _, ticket = submitted(c); prepared = prepared_for(n, ticket)
    original_stamps = tuple(prepared.snapshot.sample_stamp_ns)
    original_snapshot = prepared.snapshot
    commit(n, prepared)
    reference = n._station_compute_reference
    original_contract = deepcopy(reference['contract'])
    deadlines = (reference['geometry_monotonic'], reference['identity_monotonic'])
    if pending:
        now = original_contract['geometry_deadline_ns']+10_000_000
        # clock() already feeds original SDK ingress at the actual ROS clock.
        # Its float-to-Time conversion can truncate1ns; feeding the unconverted
        # integer again would create a real future-feedback hard failure.
        clock(c, now)
        assert original_contract['geometry_deadline_ns'] <= n._ros_source_clock_ns()
        assert n._ros_source_clock_ns() < original_contract['identity_deadline_ns']
        assert n._station_compute_poll()
        assert n._station_compute_pending and not n._station_compute_retired
    assert n._publish_station_contact_terminal(prepared, CONTACT_CODE, 'original strict search exhausted')
    terminal = n.preview_rich_plan_pub.messages[-1]
    receipt = json.loads(terminal.diagnostic.partition(':')[2])
    validate_contact_search_failure(receipt, original_contract,
        now_ns=n._ros_source_clock_ns(), header_ns=terminal.header.stamp.to_nsec())
    assert terminal.candidate_source == CONTACT_SOURCE
    assert terminal.valid is False and not terminal.poses and not terminal.plan_id
    assert terminal.target_track_id == c.binding['target_track_id']
    assert [r['contract']['source_ns'] for r in receipt['sources']] == list(original_stamps)
    assert prepared.snapshot is original_snapshot
    assert n._station_compute_reference is reference
    assert reference['contract'] == original_contract
    assert (reference['geometry_monotonic'], reference['identity_monotonic']) == deadlines
    assert not n._station_compute_publication_started
    assert not n._station_compute_work_published


def test_incomplete_or_uncommitted_search_does_not_acquire_bound_failure(station):
    c = station; n = c.node
    _, _, ticket = submitted(c); prepared = prepared_for(n, ticket)
    assert not n._publish_station_contact_terminal(prepared, CONTACT_CODE, 'not committed')
    report = commit(n, prepared, complete=False)
    assert not n._publish_station_contact_terminal(prepared, CONTACT_CODE, 'unchecked tail')
    report['moveit_selection']['shortlist_count'] = 3
    report['selected'] = {'source_index': 1}
    n._station_contact_terminal_audit(report, dict(report_sha256='a'*64), ticket)
    assert not n._publish_station_contact_terminal(prepared, CONTACT_CODE, 'selected candidate')
    report['selected'] = None; report['outcome']['preview_valid'] = True
    n._station_contact_terminal_audit(report, dict(report_sha256='a'*64), ticket)
    assert not n._publish_station_contact_terminal(prepared, CONTACT_CODE, 'valid preview')
    assert not any(m.candidate_source == CONTACT_SOURCE
        for m in n.preview_rich_plan_pub.messages)
