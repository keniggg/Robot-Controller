#!/usr/bin/env python3
from dataclasses import replace
import pathlib
import sys

import numpy as np
import pytest


ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT / 'src') not in sys.path:
    sys.path.insert(0, str(ROOT / 'src'))

from alicia_flexible_grasp.grasp.gripper_geometry import CandidateGateResult
from alicia_flexible_grasp.grasp.hybrid_grasp_candidates import (
    MergeConfig,
    NormalizedPlanningCandidate,
    merge_hybrid_candidates,
)


def successful_gate(support_clearance_m=0.010):
    return CandidateGateResult(
        ok=True,
        failure_code='',
        failure_reason='',
        required_open_width_m=0.040,
        center_distance_m=0.002,
        support_clearance_m=support_clearance_m,
        jaw_alignment=1.0,
        motion_cost=0.0,
        geometry_cost=0.1,
        passed_gate_count=6,
    )


def normalized_candidate(
    source,
    *,
    center=(0.0, 0.0, 0.01),
    insertion=(0.0, 0.0, -1.0),
    jaw=(0.0, 1.0, 0.0),
    source_index=0,
    variant_index=0,
    common_cost=0.20,
    required_width_m=0.040,
    support_clearance_m=0.010,
):
    transform = np.eye(4)
    transform[:3, 3] = center
    is_geometry = source == 'tabletop_geometry'
    return NormalizedPlanningCandidate(
        candidate_source=source,
        source_index=source_index,
        variant_index=variant_index,
        source_lineage=(source,),
        contact_center_base=np.asarray(center, dtype=float),
        T_base_tool0=transform,
        insertion_axis_base=np.asarray(insertion, dtype=float),
        jaw_axis_base=np.asarray(jaw, dtype=float),
        required_open_width_m=required_width_m,
        model_width_m=None if is_geometry else 0.038,
        model_score=None if is_geometry else 0.80,
        source_local_score=0.1,
        common_physical_cost=common_cost,
        geometry_gate=successful_gate(support_clearance_m),
        grasp_sequence={'sequence': source},
        payload={'source': source},
        audit={'source': source},
    )


def test_cross_source_duplicate_keeps_physical_winner_and_both_lineages():
    graspnet = normalized_candidate(
        'graspnet', center=(0.0, 0.0, 0.01), common_cost=0.20
    )
    geometry = normalized_candidate(
        'tabletop_geometry', center=(0.002, 0.0, 0.01), common_cost=0.10
    )

    merged = merge_hybrid_candidates(
        (graspnet, geometry),
        MergeConfig(0.005, 10.0, 10.0),
    )

    assert len(merged) == 1
    assert merged[0].candidate_source == 'tabletop_geometry'
    assert merged[0].common_physical_cost == pytest.approx(0.10)
    assert merged[0].source_lineage == ('graspnet', 'tabletop_geometry')


def test_geometry_candidate_has_no_model_width_or_confidence():
    candidate = normalized_candidate('tabletop_geometry')

    assert candidate.model_width_m is None
    assert candidate.model_score is None
    assert candidate.candidate_source == 'tabletop_geometry'


def test_jaw_axis_is_undirected_but_insertion_axis_is_directed_for_deduplication():
    graspnet = normalized_candidate('graspnet')
    opposite_jaw = normalized_candidate(
        'tabletop_geometry', jaw=(0.0, -1.0, 0.0), common_cost=0.10
    )
    opposite_insertion = replace(
        opposite_jaw,
        insertion_axis_base=np.array((0.0, 0.0, 1.0)),
    )

    jaw_merged = merge_hybrid_candidates(
        (graspnet, opposite_jaw), MergeConfig()
    )
    insertion_not_merged = merge_hybrid_candidates(
        (graspnet, opposite_insertion), MergeConfig()
    )

    assert len(jaw_merged) == 1
    assert len(insertion_not_merged) == 2


def test_same_source_and_different_variant_candidates_are_not_collapsed():
    direct = normalized_candidate('tabletop_geometry', variant_index=0)
    wrist_flip = normalized_candidate(
        'tabletop_geometry',
        variant_index=1,
        jaw=(0.0, -1.0, 0.0),
    )
    cross_source_other_variant = normalized_candidate(
        'graspnet', variant_index=1
    )

    same_source = merge_hybrid_candidates((direct, wrist_flip), MergeConfig())
    different_variants = merge_hybrid_candidates(
        (direct, cross_source_other_variant), MergeConfig()
    )

    assert [item.variant_index for item in same_source] == [0, 1]
    assert len(different_variants) == 2


def test_reversing_tied_input_preserves_physical_result_and_sorted_lineage():
    graspnet = normalized_candidate('graspnet')
    geometry = normalized_candidate('tabletop_geometry')

    forward = merge_hybrid_candidates((graspnet, geometry), MergeConfig())[0]
    reverse = merge_hybrid_candidates((geometry, graspnet), MergeConfig())[0]

    assert forward.common_physical_cost == reverse.common_physical_cost
    assert forward.required_open_width_m == reverse.required_open_width_m
    assert np.array_equal(forward.contact_center_base, reverse.contact_center_base)
    assert np.array_equal(forward.T_base_tool0, reverse.T_base_tool0)
    assert forward.source_lineage == reverse.source_lineage == (
        'graspnet',
        'tabletop_geometry',
    )


def test_normalized_candidate_defensively_copies_and_freezes_arrays_and_audit():
    center = np.array((0.0, 0.0, 0.01))
    audit = {'quality': 1}
    candidate = normalized_candidate('graspnet', center=center)
    candidate = replace(candidate, audit=audit)
    center[0] = 9.0
    audit['quality'] = 9

    assert candidate.contact_center_base.tolist() == [0.0, 0.0, 0.01]
    assert candidate.audit['quality'] == 1
    for array in (
        candidate.contact_center_base,
        candidate.T_base_tool0,
        candidate.insertion_axis_base,
        candidate.jaw_axis_base,
    ):
        assert array.flags.writeable is False


@pytest.mark.parametrize(
    'changes',
    [
        {'candidate_source': 'unknown'},
        {'source_lineage': ('tabletop_geometry', 'graspnet')},
        {'source_lineage': ('graspnet', 'graspnet')},
        {'model_width_m': -0.001},
        {'model_score': float('nan')},
    ],
)
def test_normalized_candidate_rejects_invalid_provenance_and_model_values(changes):
    with pytest.raises(ValueError):
        replace(normalized_candidate('graspnet'), **changes)
