"""Synthetic stage coverage; this is not a replay of the unavailable R6 bag."""
from copy import deepcopy

import pytest

from alicia_flexible_grasp.grasp.contact_height_placement import place_contact_height
from alicia_flexible_grasp.grasp.stable_recheck_contract import (
    CODE, STAGE, REJECTION_CODE, completed_stable_recheck_proof_valid)
from test_contact_height_placement import case


def completed_proof(**binding):
    # Use the actual interval evaluator on the existing geometry fixture.
    transform, placement = place_contact_height(
        translation_evidence_limit_m=.00460616416406, **case())
    assert transform is None and placement['before']['evaluated']
    source = dict(request_id=13, generation=7, target_epoch=4, snapshot_stamp_ns=9000)
    source.update(binding)
    rows = [dict(stable_index=i, track_id=i + 1,
                 source_request_id=source['request_id'], candidate_source='tabletop_geometry',
                 variant_index=v, status='rejected', code=REJECTION_CODE,
                 reason=placement['reason'], contact_height_placement=deepcopy(placement))
            for i in range(6) for v in (0, 1)]
    ledger = dict(source, schema_version=1, stage=STAGE, completed=True,
                  stable_candidate_count=6, expected_variant_count=12,
                  completed_variant_count=12, generated_count=0, rejected_count=12,
                  incomplete_count=0, variants=rows, contact_or_motion_authorized=False)
    return dict(source, contact_search_code=CODE, selected=False, terminated_early=False,
                checked_count=0, shortlist_count=0, reachable_count=0,
                locally_valid=dict(entered=8, passed=6, rejected=2),
                stable_recheck=ledger, contact_or_motion_authorized=False)


def test_measured_complete_geometry_rejections_are_not_path_failures():
    proof = completed_proof()
    assert completed_stable_recheck_proof_valid(proof)
    assert proof['locally_valid']['passed'] == 6
    assert proof['checked_count'] == 0


@pytest.mark.parametrize('fault', [
    'stale_request', 'stale_generation', 'stale_epoch', 'stale_source',
    'bool_binding', 'missing_tail', 'duplicate_variant', 'duplicate_track',
    'mixed_track', 'future_source', 'bool_variant', 'unknown_error',
    'incomplete', 'generated', 'count_mismatch', 'missing_evidence',
    'not_evaluated', 'placement_succeeded', 'changed_limit', 'nan_limit',
    'bool_limit', 'motion_authorized', 'local_zero', 'local_incomplete',
    'bool_count', 'moveit_checked', 'selected', 'timeout', 'wrong_code',
])
def test_incomplete_unbound_or_nonphysical_rejections_never_prove_exhaustion(fault):
    proof = completed_proof()
    ledger = proof['stable_recheck']
    row = ledger['variants'][0]
    placement = row['contact_height_placement']
    if fault.startswith('stale_'):
        name = {'stale_request': 'request_id', 'stale_generation': 'generation',
                'stale_epoch': 'target_epoch', 'stale_source': 'snapshot_stamp_ns'}[fault]
        ledger[name] += 1
    elif fault == 'bool_binding': ledger['generation'] = True
    elif fault == 'missing_tail': ledger['variants'].pop()
    elif fault == 'duplicate_variant': ledger['variants'][1] = deepcopy(row)
    elif fault == 'duplicate_track':
        for item in ledger['variants'][2:4]: item['track_id'] = row['track_id']
    elif fault == 'mixed_track': row['track_id'] += 100
    elif fault == 'future_source': row['source_request_id'] += 1
    elif fault == 'bool_variant': row['variant_index'] = False
    elif fault == 'unknown_error': row['code'] = 'ValueError'
    elif fault == 'incomplete': row['status'] = 'incomplete'
    elif fault == 'generated': row['status'] = 'generated'
    elif fault == 'count_mismatch': ledger['expected_variant_count'] += 1
    elif fault == 'missing_evidence': row.pop('contact_height_placement')
    elif fault == 'not_evaluated': placement['before']['evaluated'] = False
    elif fault == 'placement_succeeded': placement['before']['ok'] = True
    elif fault == 'changed_limit':
        placement['after'] = dict(placement['before'], translation_evidence_limit_m=.00001)
    elif fault == 'nan_limit': placement['before']['translation_evidence_limit_m'] = float('nan')
    elif fault == 'bool_limit': placement['before']['translation_evidence_limit_m'] = True
    elif fault == 'motion_authorized': proof['contact_or_motion_authorized'] = True
    elif fault == 'local_zero': proof['locally_valid'] = dict(entered=8, passed=0, rejected=8)
    elif fault == 'local_incomplete': proof['locally_valid']['entered'] += 1
    elif fault == 'bool_count': proof['checked_count'] = False
    elif fault == 'moveit_checked': proof['checked_count'] = proof['shortlist_count'] = 1
    elif fault == 'selected': proof['selected'] = True
    elif fault == 'timeout': proof['terminated_early'] = True
    elif fault == 'wrong_code': proof['contact_search_code'] = 'NEAR_FIELD_NO_REACHABLE_CANDIDATE'
    assert not completed_stable_recheck_proof_valid(proof)
