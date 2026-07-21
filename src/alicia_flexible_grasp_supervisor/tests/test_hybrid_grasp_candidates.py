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
from alicia_flexible_grasp.grasp.grasp6d_pipeline import (
    SoftCandidateFeatures,
    SoftScoreWeights,
    source_neutral_candidate_cost,
)
from alicia_flexible_grasp.grasp.hybrid_grasp_candidates import (
    MergeConfig,
    NormalizedPlanningCandidate,
    bilateral_contact_balance,
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


def projection_band_cloud(negative_count, positive_count):
    negative = [(-1.0, float(index) * 0.001, 0.0) for index in range(negative_count)]
    positive = [(1.0, float(index) * 0.001, 0.0) for index in range(positive_count)]
    middle = [(0.0, -0.01, 0.0), (0.0, 0.01, 0.0)]
    return np.asarray(negative + middle + positive, dtype=float)


def test_bilateral_contact_balance_is_one_for_balanced_projection_bands():
    points = projection_band_cloud(6, 6)

    balance = bilateral_contact_balance(points, (1.0, 0.0, 0.0))

    assert balance == pytest.approx(1.0)
    assert 0.0 <= balance <= 1.0


def test_bilateral_contact_balance_reports_one_sided_band_imbalance():
    points = projection_band_cloud(1, 5)

    balance = bilateral_contact_balance(points, (1.0, 0.0, 0.0))

    assert balance == pytest.approx(0.2)
    assert 0.0 <= balance <= 1.0


@pytest.mark.parametrize(
    ('points', 'jaw_axis'),
    [
        (np.zeros((1, 3)), (1.0, 0.0, 0.0)),
        (np.zeros((4, 3)), (1.0, 0.0, 0.0)),
        (np.array(((0.0, 0.0, 0.0), (float('nan'), 0.0, 0.0))), (1.0, 0.0, 0.0)),
        (np.zeros((4, 2)), (1.0, 0.0, 0.0)),
        (projection_band_cloud(2, 2), (0.0, 0.0, 0.0)),
    ],
)
def test_bilateral_contact_balance_rejects_invalid_or_sparse_clouds(
    points, jaw_axis
):
    with pytest.raises(ValueError):
        bilateral_contact_balance(points, jaw_axis)


def test_bilateral_contact_balance_is_source_independent():
    points = projection_band_cloud(2, 5)
    graspnet = normalized_candidate('graspnet', jaw=(1.0, 0.0, 0.0))
    geometry = normalized_candidate(
        'tabletop_geometry', jaw=(-1.0, 0.0, 0.0)
    )

    graspnet_balance = bilateral_contact_balance(
        points, graspnet.jaw_axis_base
    )
    geometry_balance = bilateral_contact_balance(
        points, geometry.jaw_axis_base
    )

    assert graspnet_balance == geometry_balance == pytest.approx(0.4)


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
    balance = bilateral_contact_balance(
        projection_band_cloud(4, 4), candidate.jaw_axis_base
    )
    features = SoftCandidateFeatures(
        model_score=candidate.model_score,
        cloud_distance_m=0.0,
        center_distance_m=0.0,
        downward_approach_cos=1.0,
        visibility_center_cost=0.0,
        support_margin_m=0.0,
        jaw_tilt_cos=1.0,
        geometry_margin_m=0.0,
        joint_path_cost=0.0,
        joint_max_delta_rad=0.0,
        stability_hit_ratio=0.0,
        position_dispersion_m=0.0,
        orientation_dispersion_rad=0.0,
        contact_balance=balance,
    )

    score = source_neutral_candidate_cost(features, SoftScoreWeights())

    assert candidate.model_width_m is None
    assert candidate.model_score is None
    assert candidate.candidate_source == 'tabletop_geometry'
    assert score.components['model_score'] == 0.0
    assert score.components['contact_balance'] < 0.0


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


def scaled_insertion_axis(angle_deg, scale=1.0000005):
    angle = np.radians(angle_deg)
    return scale * np.array((np.sin(angle), 0.0, -np.cos(angle)))


def scaled_opposite_jaw_axis(angle_deg, scale=1.0000005):
    angle = np.radians(angle_deg)
    return scale * np.array((np.sin(angle), -np.cos(angle), 0.0))


def test_normalized_candidate_normalizes_accepted_near_unit_axes():
    candidate = normalized_candidate(
        'tabletop_geometry',
        insertion=scaled_insertion_axis(5.0),
        jaw=scaled_opposite_jaw_axis(5.0),
    )

    assert np.linalg.norm(candidate.insertion_axis_base) == pytest.approx(
        1.0, rel=0.0, abs=1e-12
    )
    assert np.linalg.norm(candidate.jaw_axis_base) == pytest.approx(
        1.0, rel=0.0, abs=1e-12
    )


@pytest.mark.parametrize(
    ('angle_deg', 'expected_count'),
    [(9.9999, 1), (10.0, 1), (10.0001, 2)],
)
def test_directed_insertion_merge_threshold_boundaries(angle_deg, expected_count):
    graspnet = normalized_candidate('graspnet')
    geometry = normalized_candidate(
        'tabletop_geometry',
        insertion=scaled_insertion_axis(angle_deg),
        common_cost=0.1,
    )

    merged = merge_hybrid_candidates(
        (graspnet, geometry),
        MergeConfig(insertion_angle_deg=10.0),
    )

    assert len(merged) == expected_count


@pytest.mark.parametrize(
    ('angle_deg', 'expected_count'),
    [(9.9999, 1), (10.0, 1), (10.0001, 2)],
)
def test_undirected_jaw_merge_threshold_boundaries(angle_deg, expected_count):
    graspnet = normalized_candidate('graspnet')
    geometry = normalized_candidate(
        'tabletop_geometry',
        jaw=scaled_opposite_jaw_axis(angle_deg),
        common_cost=0.1,
    )

    merged = merge_hybrid_candidates(
        (graspnet, geometry),
        MergeConfig(jaw_angle_deg=10.0),
    )

    assert len(merged) == expected_count


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
