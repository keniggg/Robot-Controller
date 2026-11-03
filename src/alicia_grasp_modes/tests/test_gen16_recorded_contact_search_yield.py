"""GEN16 recorded failures and limited wire regression, entirely offline.

The real run timed out. It did not yield, and its failed trajectories and
private registered-gain object were not recorded. The positive typed proof
below uses a clearly counterfactual schedule with original receipt contents;
it is not a replay of an unrecorded successful supplement or grasp. Actual
producer/registration/task callback chains are covered by the separate
generic integration tests rather than invented GEN16 fused point votes.
"""
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from alicia_grasp_modes.active_observation_policy import evidence_digest, information_gain_valid
from alicia_grasp_modes.contact_search_yield import CODE, contact_search_yield_proof_valid
from alicia_grasp_modes.observe_return_remote import ObserveReturnRemoteMixin
from alicia_grasp_modes.observe_return_visual import bounded_bilateral_view_gaps


FIXTURE = Path(__file__).parent / 'fixtures/gen16_recorded_contact_search_yield.json'


def recorded():
    return json.loads(FIXTURE.read_text())


def geometry(data):
    return NS(**{key: np.asarray(value) if isinstance(value, list) else value
                 for key, value in data['geometry'].items()})


def context(data):
    """Real mixin proof readers, with recorded request and no motion interface."""
    node = ObserveReturnRemoteMixin()
    node._mode_selection = deepcopy(data['request']['grasp_mode_selection'])
    node._observe_return_phase = 'observing_contact'
    node._observe_return_session = dict(data['session'], version=2, phase='observing')
    node._observe_return_diagnostic_observation_only = False
    node._observe_return_step_gain = None
    node._observe_return_pending_supplement = None
    node.tabletop_geometry_config = NS(**data['tabletop_geometry_config'])
    source = data['request']['snapshot_stamp_ns']
    node._ros_source_clock_ns = lambda: source + 24_000_000_000
    node._active_gate_audit_report = dict(deepcopy(data['request']),
        moveit_selection=deepcopy(data['original_moveit_selection']),
        selected=data['original_selected'], outcome=deepcopy(data['original_outcome']),
        pipeline_funnel=dict(stage_counts=dict(locally_valid=data['recorded_locally_valid'])))
    prepared = NS(ticket=NS(**{key: data['request'][key] for key in
                              ('request_id', 'generation', 'target_epoch')}),
        snapshot=NS(stamp_ns=source), geometry=geometry(data),
        remote_diagnostics=deepcopy(data['remote_diagnostics']), remote_failure_code='',
        near_field_phase_id=data['original_near_field_context']['registration_phase_id'])
    return node, prepared


def counterfactual_typed_proof(data):
    """Unexecuted two-family schedule; original first collision, not fake times.

    Only the first actual collision receipt is consumed. The second family is
    an original candidate whose *hypothetical earlier check* is independent of
    its later recorded collision. No later SDK receipt is moved back in time.
    """
    source = data['request']['snapshot_stamp_ns']
    original = data['recorded_contact_receipts'][0]
    p = original['precheck']
    receipt = {key: data['request'][key] for key in
               ('request_id', 'generation', 'target_epoch', 'snapshot_stamp_ns')}
    receipt.update({key: p[key] for key in ('plan_id', 'scope', 'success',
        'execution_authorized', 'failed_stage', 'failure_code', 'reason')})
    receipt.update(evidence_sha256=original['precheck_sha256'],
        sdk_stamp_ns=p['source_sample']['sdk_stamp_ns'],
        accepted_stamp_ns=p['source_sample']['accepted_stamp_ns'])
    started = source + 1
    yielded = original['ros_stamp_ns'] + 1_000_000_000
    phase_deadline = data['original_near_field_context']['deadline_ns']
    proof = {key: data['request'][key] for key in
             ('request_id', 'generation', 'target_epoch', 'snapshot_stamp_ns')}
    proof.update(contact_search_code=CODE, selected=False, terminated_early=True,
        checked_count=2, shortlist_count=data['original_moveit_selection']['shortlist_count'],
        reachable_count=0, locally_valid=deepcopy(data['recorded_locally_valid']),
        contact_or_motion_authorized=False)
    proof['yield_evidence'] = dict(policy='bounded_contact_search_for_measured_view',
        phase_id=data['original_near_field_context']['registration_phase_id'],
        original_phase_deadline_ns=phase_deadline, started_ns=started, yielded_ns=yielded,
        budget_deadline_ns=min(started+30_000_000_000, phase_deadline-2_000_000_000),
        maximum_duration_sec=30., maximum_original_families=2,
        exhausted_by='original_family_limit', incumbent_found=False,
        original_families=[{key: family[key] for key in
            ('track_id', 'variant_index', 'family_sha256')}
            for family in data['original_family_indices'][:2]],
        collision_receipts=[receipt])
    return proof


def test_gen16_eight_actual_collisions_are_distinct_from_timeout_and_no_contact_execution():
    d = recorded()
    assert d['request']['request_id'] == 13
    assert d['request']['snapshot_stamp_ns'] == 1791718687000939369
    assert d['original_moveit_selection']['checked_count'] == 3
    assert d['original_moveit_selection']['shortlist_count'] == 30
    assert d['original_moveit_selection']['terminated_early'] is True
    assert d['original_outcome']['code'] == 'NEAR_FIELD_DIRECT_TIMEOUT'
    rows = d['recorded_contact_receipts']
    assert len(rows) == len({row['plan_id'] for row in rows}) == 8
    assert rows[0]['ros_stamp_ns'] == 1791718710026530000
    assert d['session']['deadline_ns'] - rows[0]['ros_stamp_ns'] > 500_000_000_000
    for row in rows:
        p = row['precheck']
        assert row['source_stamp_ns'] == d['request']['snapshot_stamp_ns']
        assert p['failed_stage'] == 'grasp'
        assert p['reason'].startswith('predicted_command continuous geometry: target OBB collision at ')
        assert p['checked_stage_names'] == ['pregrasp', 'approach', 'grasp']
        assert row['contact_height_bounds_m'][1] - row['contact_height_bounds_m'][0] >= .002
        assert p['success'] is False and p['execution_authorized'] is False
        assert p['hardware_response_verified'] is False
        assert row['original_failed_trajectory_recorded'] is False
    # The actual second original-family failure occurred after the hypothetical
    # 30 s sub-budget; it is never mislabeled as this regression's early yield.
    assert rows[3]['ros_stamp_ns'] - d['request']['snapshot_stamp_ns'] > 30_000_000_000


def test_gen16_current_bounded_gaps_include_five_real_thin_directions():
    d = recorded()
    before = deepcopy(d)
    cfg = NS(**d['tabletop_geometry_config'])
    ordinary = bounded_bilateral_view_gaps(d['remote_diagnostics'], geometry(d), cfg)
    gaps = bounded_bilateral_view_gaps(d['remote_diagnostics'], geometry(d), cfg, include_thin=True)
    thin = [row for row in gaps if row.get('observation_deficit') == 'insufficient_measured_common_height']
    assert len(ordinary) == 2 and len(gaps) == 7 and len(thin) == 5
    heights = sorted(row['contact_height_bounds_m'][1] - row['contact_height_bounds_m'][0] for row in thin)
    assert heights[0] == pytest.approx(.000699483240986431, rel=0., abs=1e-15)
    assert heights[-1] == pytest.approx(.00091226976304831, rel=0., abs=1e-15)
    assert all(row['required_open_width_m'] <= .05 for row in gaps)
    assert all(row['contact_or_motion_authorized'] is False for row in gaps)
    assert d == before


@pytest.mark.parametrize('code', ['NEAR_FIELD_DIRECT_TIMEOUT', 'NEAR_FIELD_NO_REACHABLE_CANDIDATE'])
def test_gen16_ordinary_timeout_or_incomplete_search_never_becomes_completed_supplement(code):
    node, prepared = context(recorded())
    assert node._observe_return_completed_contact_failure(prepared, code) is None
    assert node._observe_return_mixed_visual_failure(prepared, code) is None
    assert not node._observe_return_prepare_supplement(prepared, code)
    assert node._observe_return_pending_supplement is None


def test_counterfactual_typed_schedule_binds_original_first_collision_before_unchanged_deadline():
    d = recorded()
    proof = counterfactual_typed_proof(d)
    before = deepcopy(proof)
    assert contact_search_yield_proof_valid(proof)
    e = proof['yield_evidence']
    assert e['started_ns'] < e['yielded_ns'] < e['budget_deadline_ns'] < e['original_phase_deadline_ns']
    assert e['original_phase_deadline_ns'] == d['original_near_field_context']['deadline_ns']
    assert e['collision_receipts'][0]['evidence_sha256'] == d['recorded_contact_receipts'][0]['precheck_sha256']
    for original, family in zip(d['original_family_indices'], e['original_families']):
        assert family['family_sha256'] == evidence_digest(original['sequence'])
    assert proof == before


@pytest.mark.parametrize('fault', ['absent', 'cross_source', 'old_receipt', 'timeout_only', 'expired'])
def test_gen16_recorded_collision_cannot_be_relabelled_with_missing_or_wrong_binding(fault):
    proof = counterfactual_typed_proof(recorded())
    evidence = proof['yield_evidence']
    if fault == 'absent': evidence['collision_receipts'] = []
    elif fault == 'cross_source': evidence['collision_receipts'][0]['snapshot_stamp_ns'] -= 1
    elif fault == 'old_receipt': evidence['collision_receipts'][0]['sdk_stamp_ns'] = proof['snapshot_stamp_ns'] - 1
    elif fault == 'timeout_only': evidence['collision_receipts'][0]['reason'] = 'near-field deadline consumed'
    elif fault == 'expired': evidence['yielded_ns'] = evidence['original_phase_deadline_ns']
    assert not contact_search_yield_proof_valid(proof)


def test_gen16_unrecorded_actual_gain_is_not_fabricated_from_counts_or_initial_hash():
    d = recorded()
    assert d['recorded_actual_gain'] is None
    assert not information_gain_valid(d['recorded_actual_gain'])
    node, prepared = context(d)
    # Even a separately valid typed wire proof does not create a private
    # registered-gain object or authorize the live producer to open a new view.
    assert contact_search_yield_proof_valid(counterfactual_typed_proof(d))
    assert not node._observe_return_prepare_supplement(prepared, CODE)
    assert node._observe_return_pending_supplement is None
