"""Recorded GEN7 scheduling inputs; transport after that prefix is synthetic.

The sealed experiment failed. These tests neither replay its trajectories nor
claim the later counterfactual second-view/grasp succeeded on hardware.
"""
from copy import deepcopy
import json
from pathlib import Path

import pytest

from test_remote_modes import configured
from test_contact_search_yield_remote import ready, search
import test_contact_search_yield_remote as search_module
from alicia_grasp_modes import observe_return_remote
from alicia_grasp_modes.contact_search_yield import CODE, contact_search_yield_proof_valid


def recorded():
    return json.loads((Path(__file__).parent / 'fixtures/gen7_contact_yield_completed_budget.json').read_text())


def test_gen7_actual_prefix_has_two_attempts_but_only_one_completed_family():
    data = recorded()
    state = data['recorded_contact_search_yield_attempt']
    assert data['request_id'] == 28 and data['selection']['generation'] == 7
    assert data['recorded_final_outcome']['code'] == 'NEAR_FIELD_DIRECT_TIMEOUT'
    assert [(f['track_id'], f['variant_index']) for f in state['attempted_families']] == [(24, 1), (4, 0)]
    assert [(f['track_id'], f['variant_index']) for f in state['original_families']] == [(4, 0)]
    assert len(state['collision_receipts']) == 1 and state['fallback'] is True
    by_track = {r['track_id']: r for r in data['first_pass_rejections']}
    assert by_track[24]['rejection']['code'] == 'CONTACT_HEIGHT_REGISTRATION_MARGIN'
    assert by_track[24]['had_bound_precheck_failure'] is False
    assert by_track[4]['rejection']['code'] == 'CONTACT_SEQUENCE_PRECHECK_FAILED'
    assert by_track[4]['had_bound_precheck_failure'] is True


def test_recorded_margin_then_collision_keeps_next_complete_direction_opportunity(configured, monkeypatch):
    node, p, clock, candidates = ready(configured, monkeypatch)
    data = recorded()
    first, second = data['recorded_final_rejection_times'][:2]
    started = data['recorded_contact_search_yield_attempt']['started_ns'] / 1e9
    # Exact recorded rejection-to-rejection intervals, replayed as transport
    # delays only; synthetic third receipt is deliberately not a recorded fact.
    durations = [float(first['logged_unix_seconds']) - started,
                 float(second['logged_unix_seconds']) - float(first['logged_unix_seconds']), 2.]
    phase_deadline = p.near_field_phase_deadline_sec
    selection, calls, retries = search(node, p, clock, candidates, monkeypatch,
        results=['margin', 'collision', 'collision'], seconds=durations)
    state = node._contact_yield_search
    assert calls == [1, 2, 3, 4] and len(retries) == 4
    assert len(state['attempted_families']) == 4 and len(state['original_families']) == 3
    assert [f['track_id'] for f in state['original_families']] == [2, 3, 4]
    assert not state['fallback'] and selection.termination_reason != CODE
    assert not selection.terminated_early and len(selection.checked) == 4
    assert node._observe_return_completed_contact_failure(p, CODE) is None
    assert state['budget_deadline_ns'] - state['started_ns'] == 30_000_000_000
    assert p.near_field_phase_deadline_sec == phase_deadline



@pytest.mark.parametrize('early', [['margin'], ['ik', 'transport'], ['margin', 'ik', 'wrong_plan']])
def test_multiple_early_rejections_are_not_completed_collision_evidence(configured, monkeypatch, early):
    node, p, clock, candidates = ready(configured, monkeypatch, candidate_count=len(early)+3)
    kinds = early + ['collision', 'collision', 'timeout']
    selection, calls, retries = search(node, p, clock, candidates, monkeypatch, results=kinds)
    state = node._contact_yield_search
    assert calls == list(range(1, len(kinds)+1)) and len(retries) == len(kinds)
    assert [f['track_id'] for f in state['original_families']] == [len(early)+1, len(early)+2]
    assert len(state['collision_receipts']) == 2 and not state['fallback']
    assert selection.termination_reason != CODE and not selection.terminated_early
    assert node._observe_return_completed_contact_failure(p, CODE) is None


def test_duplicate_geometry_family_cannot_supply_two_completed_directions(configured, monkeypatch):
    node, p, clock, candidates = ready(configured, monkeypatch)
    node._stable_variant_runtime[(2, 0)]['sequence'] = deepcopy(node._stable_variant_runtime[(1, 0)]['sequence'])
    selection, calls, retries = search(node, p, clock, candidates, monkeypatch, seconds=[10.,10.,10.])
    assert calls == [1, 2, 3] and len(retries) == 3
    state = node._contact_yield_search
    assert len(state['attempted_families']) == len(state['original_families']) == 2
    assert [f['track_id'] for f in state['original_families']] == [1, 3]
    assert len({f['family_sha256'] for f in state['original_families']}) == 2
    assert selection.termination_reason == CODE


def test_all_early_rejections_exhaust_finite_candidates_without_a_certificate(configured, monkeypatch):
    node, p, clock, candidates = ready(configured, monkeypatch, candidate_count=7)
    phase = p.near_field_phase_deadline_sec
    selection, calls, retries = search(node, p, clock, candidates, monkeypatch, results=['margin']*14)
    assert calls == list(range(1, 8))
    assert not node._contact_yield_search['fallback'] and len(retries) == 7
    assert not node._contact_yield_search['original_families']
    assert not node._contact_yield_search['collision_receipts']
    assert selection.termination_reason != CODE
    assert not node._observe_return_prepare_supplement(p, CODE)
    assert p.near_field_phase_deadline_sec == phase


def test_early_rejections_still_hit_unchanged_child_time_budget(configured, monkeypatch):
    node, p, clock, candidates = ready(configured, monkeypatch)
    selection, calls, retries = search(node, p, clock, candidates, monkeypatch,
        results=['margin']*8, seconds=[15., 15.])
    state = node._contact_yield_search
    assert state['first_pass_checked'] == 2
    assert state['budget_deadline_ns'] - state['started_ns'] == 30_000_000_000
    assert state['fallback'] and calls == [1, 2, 3, 4, 1, 2]
    assert not state['original_families'] and not state['collision_receipts']
    assert selection.termination_reason != CODE and retries


def test_true_incumbent_after_early_failure_keeps_original_selection(configured, monkeypatch):
    node, p, clock, candidates = ready(configured, monkeypatch)
    selection, calls, retries = search(node, p, clock, candidates, monkeypatch,
        results=['margin', 'success', 'collision', 'collision'])
    assert selection.selected is not None and selection.selected.track_id == 2
    assert node._contact_yield_search['incumbent_found']
    assert not node._contact_yield_search['fallback']
    assert selection.termination_reason != CODE
    assert not node._observe_return_prepare_supplement(p, CODE)


def test_parent_publication_reserve_still_prevents_boundary_yield(configured, monkeypatch):
    node, p, clock, candidates = ready(configured, monkeypatch)
    p.near_field_phase_deadline_sec = clock[0]*1e-9 + 6.
    node._near_field_phase_deadline_sec = p.near_field_phase_deadline_sec
    selection, calls, retries = search(node, p, clock, candidates, monkeypatch,
        results=['collision']*8, seconds=[2., 2.])
    state = node._contact_yield_search
    assert state['budget_deadline_ns'] <= state['original_phase_deadline_ns'] - 2_000_000_000
    assert not state['stopped']
    assert selection.termination_reason != CODE
    assert not node._observe_return_prepare_supplement(p, CODE)


def test_second_complete_receipt_at_child_deadline_keeps_original_certificate_order(configured, monkeypatch):
    node, p, clock, candidates = ready(configured, monkeypatch)
    selection, calls, retries = search(node, p, clock, candidates, monkeypatch,
        results=['margin', 'collision', 'collision'], seconds=[1., 14., 15.])
    state = node._contact_yield_search
    assert clock[0] == state['budget_deadline_ns']
    assert calls == [1, 2, 3] and len(retries) == 3
    assert not state['fallback'] and selection.termination_reason == CODE
    assert contact_search_yield_proof_valid(node._observe_return_completed_contact_failure(p, CODE))


def test_monotonic_child_deadline_still_stops_without_two_receipts(configured, monkeypatch):
    node, p, clock, candidates = ready(configured, monkeypatch)
    wall = [100.]
    monkeypatch.setattr(observe_return_remote.time, 'monotonic', lambda: wall[0])
    original = search_module.receipt
    def consume_wall(*args):
        value = original(*args)
        wall[0] += 31.
        return value
    monkeypatch.setattr(search_module, 'receipt', consume_wall)
    selection, calls, retries = search(node, p, clock, candidates, monkeypatch,
        results=['collision'] + ['margin']*6)
    state = node._contact_yield_search
    assert state['first_pass_checked'] == 1 and state['fallback']
    assert clock[0] < state['budget_deadline_ns']
    assert len(state['original_families']) == 1
    assert selection.termination_reason != CODE and retries


def test_two_completed_non_grasp_rejections_do_not_create_collision_proof(configured, monkeypatch):
    node, p, clock, candidates = ready(configured, monkeypatch)
    original = search_module.receipt
    def no_grasp_collision(*args):
        value = original(*args)
        value.update(failed_stage='approach', reason='strict approach collision')
        return value
    monkeypatch.setattr(search_module, 'receipt', no_grasp_collision)
    selection, calls, retries = search(node, p, clock, candidates, monkeypatch)
    state = node._contact_yield_search
    assert len(state['original_families']) == 4 and not state['collision_receipts']
    assert not state['fallback'] and len(retries) == 4
    assert selection.termination_reason != CODE
    assert not node._observe_return_prepare_supplement(p, CODE)
