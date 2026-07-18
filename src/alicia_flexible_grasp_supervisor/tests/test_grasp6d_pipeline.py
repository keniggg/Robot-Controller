#!/usr/bin/env python3
from dataclasses import replace
import math
import pathlib
import sys

import numpy as np
import pytest


ROOT = pathlib.Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / 'src'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from alicia_flexible_grasp.grasp.grasp6d_pipeline import (  # noqa: E402
    CandidateStageFunnel,
    MoveItResult,
    SafetyGateInput,
    ScoredStableCandidate,
    SoftCandidateFeatures,
    SoftScoreWeights,
    bounded_moveit_select,
    mandatory_safety_gate,
    soft_candidate_cost,
)
from alicia_flexible_grasp.grasp.grasp6d_stability import (  # noqa: E402
    StableCandidate,
)


def valid_safety_input(**overrides):
    values = {
        'depth_valid': True,
        'transform_valid': True,
        'target_present': True,
        'same_target_instance': True,
        'target_absolute_distance_m': 0.020,
        'target_absolute_limit_m': 0.150,
        'required_open_width_m': 0.040,
        'physical_open_width_m': 0.050,
        'geometry_valid': True,
        'collision_free': True,
    }
    values.update(overrides)
    return SafetyGateInput(**values)


def soft_features(**overrides):
    values = {
        'model_score': 0.80,
        'cloud_distance_m': 0.005,
        'center_distance_m': 0.010,
        'downward_approach_cos': 0.90,
        'visibility_center_cost': 0.10,
        'support_margin_m': 0.010,
        'jaw_tilt_cos': 0.95,
        'geometry_margin_m': 0.008,
        'joint_path_cost': 0.10,
        'joint_max_delta_rad': 0.10,
        'stability_hit_ratio': 0.80,
        'position_dispersion_m': 0.002,
        'orientation_dispersion_rad': 0.03,
    }
    values.update(overrides)
    return SoftCandidateFeatures(**values)


def weights(**overrides):
    values = {
        'model_score_weight': 1.0,
        'cloud_distance_weight': 0.4,
        'center_distance_weight': 0.8,
        'downward_approach_weight': 0.8,
        'visibility_center_weight': 0.4,
        'support_margin_weight': 0.3,
        'jaw_tilt_weight': 0.3,
        'geometry_margin_weight': 0.6,
        'joint_path_weight': 0.5,
        'joint_max_delta_weight': 0.5,
        'stability_hit_ratio_weight': 0.8,
        'position_dispersion_weight': 0.4,
        'orientation_dispersion_weight': 0.4,
        'cloud_distance_knee_m': 0.020,
        'center_distance_knee_m': 0.040,
        'downward_approach_cos_knee': 0.75,
        'visibility_center_cost_knee': 0.40,
        'support_margin_knee_m': 0.010,
        'jaw_tilt_cos_knee': 0.90,
        'geometry_margin_knee_m': 0.010,
        'joint_path_cost_knee': 1.0,
        'joint_max_delta_knee_rad': 1.0,
        'position_dispersion_knee_m': 0.010,
        'orientation_dispersion_knee_rad': 0.20,
    }
    values.update(overrides)
    return SoftScoreWeights(**values)


def stable_candidate(track_id, pre_moveit_score=0.0):
    return StableCandidate(
        track_id=track_id,
        hit_count=3,
        window_count=5,
        hit_request_ids=(1, 2, 3),
        request_id=3,
        snapshot_stamp_sec=10.0,
        target_epoch=7,
        target_label='carton',
        model_choice='carton_segmentation',
        center_base_xyz=np.array([0.1, 0.0, 0.2]),
        tool0_position_xyz=np.array([0.1, 0.0, 0.2]),
        quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
        approach_base_xyz=np.array([0.0, 0.0, -1.0]),
        required_open_width_m=0.040,
        model_width_m=0.040,
        model_score=0.8,
        geometry_margin_m=0.008,
        pre_moveit_score=pre_moveit_score,
        payload={'track_id': track_id},
    )


def scored_candidate(
    track_id,
    score,
    variant_index=0,
    safety=None,
    features=None,
    score_weights=None,
):
    return ScoredStableCandidate(
        stable_candidate=stable_candidate(track_id, pre_moveit_score=score),
        variant_index=variant_index,
        latest_safety=valid_safety_input() if safety is None else safety,
        soft_features=(
            soft_features(joint_path_cost=0.0, joint_max_delta_rad=0.0)
            if features is None
            else features
        ),
        score_weights=weights() if score_weights is None else score_weights,
        pre_moveit_score=score,
    )


@pytest.mark.parametrize(
    ('field', 'value', 'code'),
    [
        ('depth_valid', False, 'DEPTH_INVALID'),
        ('transform_valid', False, 'TRANSFORM_INVALID'),
        ('target_present', False, 'TARGET_LOST'),
        ('same_target_instance', False, 'TARGET_INSTANCE_MISMATCH'),
        ('required_open_width_m', 0.051, 'GRIPPER_TOO_NARROW'),
        ('collision_free', False, 'COLLISION'),
    ],
)
def test_mandatory_safety_conditions_remain_hard(field, value, code):
    gate = replace(valid_safety_input(), **{field: value})

    result = mandatory_safety_gate(gate)

    assert result.ok is False
    assert result.code == code


def test_preference_thresholds_do_not_become_hard_gates():
    result = mandatory_safety_gate(valid_safety_input())

    assert result.ok is True
    assert result.code == 'OK'


@pytest.mark.parametrize(
    ('overrides', 'code'),
    [
        ({'depth_valid': None}, 'DEPTH_INVALID'),
        ({'transform_valid': 1}, 'TRANSFORM_INVALID'),
        ({'target_absolute_distance_m': float('nan')}, 'TRANSFORM_INVALID'),
        ({'target_absolute_limit_m': float('inf')}, 'TRANSFORM_INVALID'),
        ({'target_absolute_distance_m': 0.151}, 'TARGET_ABSOLUTE_DISTANCE'),
        ({'required_open_width_m': 0.0}, 'GRIPPER_WIDTH_INVALID'),
        ({'required_open_width_m': float('nan')}, 'GRIPPER_WIDTH_INVALID'),
        ({'physical_open_width_m': 0.0}, 'GRIPPER_CONTRACT_INVALID'),
        ({'geometry_valid': None}, 'GEOMETRY_INVALID'),
        ({'collision_free': 1}, 'COLLISION'),
    ],
)
def test_mandatory_safety_gate_fails_closed_on_malformed_physical_facts(
    overrides, code
):
    result = mandatory_safety_gate(valid_safety_input(**overrides))

    assert result.ok is False
    assert result.code == code
    assert result.reason


def test_physical_open_width_cannot_raise_the_fifty_millimetre_limit():
    result = mandatory_safety_gate(
        valid_safety_input(
            required_open_width_m=0.0501,
            physical_open_width_m=0.060,
        )
    )

    assert result.ok is False
    assert result.code == 'GRIPPER_TOO_NARROW'


@pytest.mark.parametrize(
    ('required_width_m', 'physical_width_m'),
    [(0.030, 0.040), (0.040, 0.060)],
)
def test_physical_open_width_outside_contract_tolerance_fails_closed(
    required_width_m, physical_width_m
):
    result = mandatory_safety_gate(
        valid_safety_input(
            required_open_width_m=required_width_m,
            physical_open_width_m=physical_width_m,
        )
    )

    assert result.ok is False
    assert result.code == 'GRIPPER_CONTRACT_INVALID'


def test_stage_funnel_records_entered_passed_rejected_and_reason_ratios():
    funnel = CandidateStageFunnel(input_count=10)
    funnel.record_stage('hard_recheck', entered=10, passed=7, rejected=3)
    funnel.record_rejection('COLLISION', count=2, stage='hard_recheck')
    funnel.record_rejection(
        'GRIPPER_TOO_NARROW', count=1, stage='hard_recheck'
    )
    funnel.record_stage('moveit_checked', entered=5, passed=5, rejected=0)
    funnel.record_stage('moveit_reachable', entered=5, passed=2, rejected=3)
    funnel.record_rejection('MOVEIT_IK_FAILED', count=3, stage='moveit_reachable')

    metrics = funnel.to_dict()

    assert metrics['remaining_by_stage'] == {
        'hard_recheck': 7,
        'moveit_checked': 5,
        'moveit_reachable': 2,
    }
    assert metrics['stage_counts']['hard_recheck'] == {
        'entered': 10,
        'passed': 7,
        'rejected': 3,
    }
    assert metrics['rejection_counts'] == {
        'COLLISION': 2,
        'GRIPPER_TOO_NARROW': 1,
        'MOVEIT_IK_FAILED': 3,
    }
    assert metrics['rejection_ratios']['MOVEIT_IK_FAILED'] == pytest.approx(0.3)
    assert metrics['primary_failure'] == 'MOVEIT_IK_FAILED'
    assert metrics['dominant_hard_failure'] == 'MOVEIT_IK_FAILED'


def test_stage_funnel_primary_failure_uses_lexical_tie_break():
    funnel = CandidateStageFunnel(input_count=4)
    funnel.record_rejection('TARGET_LOST', count=2)
    funnel.record_rejection('COLLISION', count=2)

    metrics = funnel.to_dict()

    assert metrics['primary_failure'] == 'COLLISION'
    assert metrics['rejection_ratios'] == {
        'COLLISION': 0.5,
        'TARGET_LOST': 0.5,
    }


def test_stage_funnel_remaining_count_form_infers_stage_transitions():
    funnel = CandidateStageFunnel(input_count=8)

    funnel.record_stage('hard_recheck', 6)
    funnel.record_stage('stable', 3)

    assert funnel.to_dict()['stage_counts'] == {
        'hard_recheck': {'entered': 8, 'passed': 6, 'rejected': 2},
        'stable': {'entered': 6, 'passed': 3, 'rejected': 3},
    }


def test_approach_center_visibility_and_joint_motion_are_soft_costs():
    preferred = soft_features()
    degraded = replace(
        preferred,
        center_distance_m=0.060,
        downward_approach_cos=0.30,
        visibility_center_cost=0.80,
        joint_path_cost=2.0,
        joint_max_delta_rad=2.1,
    )

    preferred_score = soft_candidate_cost(preferred, weights())
    degraded_score = soft_candidate_cost(degraded, weights())

    assert degraded_score.total > preferred_score.total
    assert math.isfinite(degraded_score.total)
    assert all(math.isfinite(value) for value in degraded_score.components.values())


def test_model_score_and_geometry_margin_improve_rank_without_bypass():
    low = soft_features(model_score=0.2, geometry_margin_m=0.003)
    high = soft_features(model_score=0.9, geometry_margin_m=0.010)

    low_score = soft_candidate_cost(low, weights())
    high_score = soft_candidate_cost(high, weights())

    assert high_score.total < low_score.total
    assert high_score.components['model_score'] < low_score.components['model_score']
    assert (
        high_score.components['geometry_margin']
        < low_score.components['geometry_margin']
    )


def test_support_jaw_and_stability_are_bounded_soft_rewards():
    poor = soft_features(
        support_margin_m=-0.001,
        jaw_tilt_cos=-0.5,
        stability_hit_ratio=0.2,
    )
    strong = replace(
        poor,
        support_margin_m=10.0,
        jaw_tilt_cos=10.0,
        stability_hit_ratio=10.0,
    )

    poor_score = soft_candidate_cost(poor, weights())
    strong_score = soft_candidate_cost(strong, weights())

    assert strong_score.total < poor_score.total
    for component in ('support_margin', 'jaw_tilt', 'stability_hit_ratio'):
        assert -weights().__dict__[component + '_weight'] <= (
            strong_score.components[component]
        ) <= 0.0


def test_soft_penalties_saturate_for_large_finite_inputs():
    huge = soft_features(
        cloud_distance_m=1e300,
        center_distance_m=1e300,
        visibility_center_cost=1e300,
        joint_path_cost=1e300,
        joint_max_delta_rad=1e300,
        position_dispersion_m=1e300,
        orientation_dispersion_rad=1e300,
    )

    score = soft_candidate_cost(huge, weights())

    assert math.isfinite(score.total)
    assert all(math.isfinite(value) for value in score.components.values())


@pytest.mark.parametrize(
    'overrides',
    [
        {'model_score': float('nan')},
        {'center_distance_m': float('inf')},
        {'joint_path_cost': None},
    ],
)
def test_soft_features_reject_non_finite_values(overrides):
    with pytest.raises(ValueError, match='finite'):
        soft_features(**overrides)


@pytest.mark.parametrize(
    'overrides',
    [
        {'model_score_weight': -0.1},
        {'center_distance_knee_m': 0.0},
        {'joint_path_cost_knee': float('nan')},
    ],
)
def test_soft_weights_reject_negative_weights_and_invalid_knees(overrides):
    with pytest.raises(ValueError):
        weights(**overrides)


def test_soft_scoring_does_not_create_hard_rejections():
    funnel = CandidateStageFunnel(input_count=1)
    soft_candidate_cost(
        soft_features(
            center_distance_m=0.060,
            downward_approach_cos=0.30,
            visibility_center_cost=0.80,
        ),
        weights(),
    )
    funnel.record_stage('soft_ranked', entered=1, passed=1, rejected=0)

    assert funnel.rejection_counts == {}
    assert funnel.to_dict()['stage_counts']['soft_ranked']['rejected'] == 0


def test_moveit_checks_only_top_n_stable_candidates():
    calls = []

    def check(candidate):
        calls.append(candidate.track_id)
        return MoveItResult(
            reachable=candidate.track_id == 3,
            joint_path_cost=float(candidate.track_id),
            joint_max_delta_rad=0.1,
            reason='',
        )

    candidates = [
        scored_candidate(track_id=i, score=float(i)) for i in range(20)
    ]

    result = bounded_moveit_select(candidates, check, top_n=5)

    assert calls == [0, 1, 2, 3, 4]
    assert result.selected.track_id == 3
    assert result.funnel.remaining_by_stage['moveit_checked'] == 5
    assert result.funnel.remaining_by_stage['moveit_reachable'] == 1


@pytest.mark.parametrize(('configured', 'expected'), [(1, 3), (99, 10)])
def test_moveit_top_n_is_clamped_between_three_and_ten(configured, expected):
    calls = []

    def check(candidate):
        calls.append((candidate.track_id, candidate.variant_index))
        return MoveItResult(False, 0.0, 0.0, 'IK failed')

    candidates = [
        scored_candidate(track_id=i, score=float(i)) for i in range(20)
    ]

    result = bounded_moveit_select(candidates, check, top_n=configured)

    assert len(calls) == expected
    assert result.configured_top_n == expected
    assert result.funnel.remaining_by_stage['moveit_checked'] == expected


def test_latest_hard_recheck_precedes_soft_ranking_and_moveit_calls():
    calls = []

    def check(candidate):
        calls.append(candidate.track_id)
        return MoveItResult(True, 0.1, 0.1, '')

    candidates = [
        scored_candidate(
            0,
            -100.0,
            safety=valid_safety_input(collision_free=False),
        ),
        scored_candidate(1, 1.0),
        scored_candidate(2, 2.0),
        scored_candidate(3, 3.0),
        scored_candidate(4, 4.0),
    ]

    result = bounded_moveit_select(candidates, check, top_n=3)

    assert calls == [1, 2, 3]
    assert result.selected.track_id == 1
    assert result.funnel.rejection_counts == {'COLLISION': 1}
    assert result.funnel.to_dict()['stage_counts']['hard_recheck'] == {
        'entered': 5,
        'passed': 4,
        'rejected': 1,
    }


def test_stability_never_bypasses_latest_geometry_hard_failure():
    calls = []
    candidate = scored_candidate(
        4,
        0.0,
        safety=valid_safety_input(geometry_valid=False),
    )

    result = bounded_moveit_select(
        [candidate],
        lambda item: calls.append(item.track_id),
    )

    assert calls == []
    assert result.selected is None
    assert result.funnel.rejection_counts == {'GEOMETRY_INVALID': 1}
    metrics = result.funnel.to_dict()
    assert metrics['dominant_hard_failure'] == 'GEOMETRY_INVALID'
    assert metrics['remaining_by_stage']['moveit_checked'] == 0


def test_non_finite_fused_coordinates_fail_closed_before_moveit():
    calls = []
    malformed_stable = replace(
        stable_candidate(9),
        tool0_position_xyz=np.array([float('nan'), 0.0, 0.2]),
    )
    candidate = replace(
        scored_candidate(9, 0.0),
        stable_candidate=malformed_stable,
    )

    result = bounded_moveit_select(
        [candidate],
        lambda item: calls.append(item.track_id),
    )

    assert calls == []
    assert result.selected is None
    assert result.funnel.rejection_counts == {'TRANSFORM_INVALID': 1}


def test_symmetric_variants_each_consume_one_top_n_check_slot():
    calls = []

    def check(candidate):
        calls.append((candidate.track_id, candidate.variant_index))
        return MoveItResult(True, 0.0, 0.0, '')

    candidates = [
        scored_candidate(1, 0.0, variant_index=1),
        scored_candidate(2, 0.1, variant_index=0),
        scored_candidate(1, 0.0, variant_index=0),
        scored_candidate(2, 0.1, variant_index=1),
    ]

    result = bounded_moveit_select(candidates, check, top_n=3)

    assert calls == [(1, 0), (1, 1), (2, 0)]
    assert result.selected.track_id == 1
    assert result.selected.variant_index == 0


def test_strict_moveit_failures_do_not_stop_the_rest_of_the_shortlist():
    calls = []

    def check(candidate):
        calls.append(candidate.track_id)
        if candidate.track_id == 0:
            return MoveItResult(False, 0.0, 0.0, 'IK failed')
        if candidate.track_id == 1:
            return MoveItResult(False, 0.0, 0.0, 'joint limit')
        return MoveItResult(True, 0.2, 0.1, '')

    result = bounded_moveit_select(
        [scored_candidate(i, float(i)) for i in range(3)],
        check,
        top_n=3,
    )

    assert calls == [0, 1, 2]
    assert result.selected.track_id == 2
    assert result.funnel.rejection_counts == {
        'MOVEIT_IK_FAILED': 1,
        'MOVEIT_JOINT_LIMIT': 1,
    }


def test_moveit_motion_cost_can_reorder_reachable_candidates():
    def check(candidate):
        if candidate.track_id == 0:
            return MoveItResult(True, 10.0, 10.0, '')
        return MoveItResult(True, 0.0, 0.0, '')

    result = bounded_moveit_select(
        [scored_candidate(0, 0.0), scored_candidate(1, 0.1)],
        check,
        top_n=3,
    )

    assert result.selected.track_id == 1
    assert result.selected.final_score == pytest.approx(0.1)
    assert result.reachable[0].final_score > result.reachable[1].final_score


def test_malformed_moveit_result_fails_closed_and_records_dominant_failure():
    calls = []

    def check(candidate):
        calls.append(candidate.track_id)
        if candidate.track_id == 0:
            return MoveItResult(True, float('nan'), 0.0, '')
        raise RuntimeError('service unavailable')

    result = bounded_moveit_select(
        [scored_candidate(0, 0.0), scored_candidate(1, 1.0)],
        check,
        top_n=3,
    )

    assert calls == [0, 1]
    assert result.selected is None
    assert result.funnel.rejection_counts == {
        'MOVEIT_CHECK_ERROR': 1,
        'MOVEIT_RESULT_INVALID': 1,
    }
    assert (
        result.funnel.to_dict()['dominant_hard_failure']
        == 'MOVEIT_CHECK_ERROR'
    )


def test_candidates_outside_top_n_are_not_mislabeled_as_hard_rejections():
    candidates = [
        scored_candidate(track_id=i, score=float(i)) for i in range(8)
    ]

    result = bounded_moveit_select(
        candidates,
        lambda _candidate: MoveItResult(True, 0.0, 0.0, ''),
        top_n=3,
    )

    assert len(result.checked) == 3
    assert result.funnel.rejection_counts == {}
    assert result.funnel.to_dict()['stage_counts']['moveit_shortlist'] == {
        'entered': 8,
        'passed': 3,
        'rejected': 5,
    }
