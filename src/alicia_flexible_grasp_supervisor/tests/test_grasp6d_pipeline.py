#!/usr/bin/env python3
from dataclasses import fields as dataclass_fields, replace
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
    CandidateObservation,
    CandidateTracker,
    StableCandidate,
    TrackingConfig,
)


def test_safety_interfaces_carry_exact_variant_and_snapshot_evidence():
    safety_fields = {item.name for item in dataclass_fields(SafetyGateInput)}
    scored_fields = {item.name for item in dataclass_fields(ScoredStableCandidate)}

    assert {
        'request_id',
        'snapshot_stamp_sec',
        'target_epoch',
        'target_label',
        'model_choice',
        'track_id',
        'variant_index',
        'center_base_xyz',
        'tool0_position_xyz',
        'quaternion_xyzw',
        'approach_base_xyz',
        'snapshot_context_revision',
    } <= safety_fields
    assert {
        'evaluation_request_id',
        'evaluation_snapshot_stamp_sec',
        'evaluation_context_revision',
    } <= scored_fields
    assert {
        'variant_quaternion_xyzw',
        'variant_approach_base_xyz',
    }.isdisjoint(scored_fields)


def test_scored_candidate_has_no_caller_supplied_pre_moveit_score_field():
    scored_fields = {item.name for item in dataclass_fields(ScoredStableCandidate)}

    assert 'pre_moveit_score' not in scored_fields


def test_moveit_result_exposes_structured_strict_hard_states():
    result_fields = {item.name for item in dataclass_fields(MoveItResult)}

    assert {
        'collision_free',
        'within_joint_limits',
        'ik_valid',
        'planning_success',
    } <= result_fields


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
        'request_id': 3,
        'snapshot_stamp_sec': 10.0,
        'target_epoch': 7,
        'target_label': 'carton',
        'model_choice': 'carton_segmentation',
        'track_id': 1,
        'variant_index': 0,
        'center_base_xyz': (0.1, 0.0, 0.2),
        'tool0_position_xyz': (0.1, 0.0, 0.2),
        'quaternion_xyzw': (0.0, 0.0, 0.0, 1.0),
        'approach_base_xyz': (0.0, 0.0, -1.0),
        'snapshot_context_revision': 'ctx-3',
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


def ranking_score_weights():
    base = weights()
    zero_weights = {
        item.name: 0.0
        for item in dataclass_fields(SoftScoreWeights)
        if item.name.endswith('_weight')
    }
    zero_weights.update(
        center_distance_weight=1.0,
        model_score_weight=1.0,
        joint_path_weight=0.5,
        joint_max_delta_weight=0.5,
        center_distance_knee_m=1000.0,
    )
    return replace(base, **zero_weights)


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


def tracker_observation(request_id):
    return CandidateObservation(
        request_id=request_id,
        snapshot_stamp_sec=100.0 + request_id,
        target_epoch=7,
        target_label='carton',
        model_choice='carton_segmentation',
        center_base_xyz=(0.1, 0.0, 0.2),
        tool0_position_xyz=(0.1, 0.0, 0.2),
        quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
        approach_base_xyz=(0.0, 0.0, -1.0),
        required_open_width_m=0.040,
        model_width_m=0.038,
        model_score=0.8,
        geometry_margin_m=0.008,
        pre_moveit_score=0.0,
        payload={'request_id': request_id},
    )


def multiply_xyzw(first, second):
    x1, y1, z1, w1 = first
    x2, y2, z2, w2 = second
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def expected_variant_quaternion(stable, variant_index):
    direct = tuple(stable.quaternion_xyzw)
    if variant_index == 1:
        return multiply_xyzw(direct, (0.0, 0.0, 1.0, 0.0))
    return direct


def safety_for_candidate(
    stable,
    variant_index=0,
    quaternion=None,
    approach=None,
    evaluation_request_id=None,
    evaluation_snapshot_stamp_sec=None,
    evaluation_context_revision='ctx-3',
    **overrides
):
    if evaluation_request_id is None:
        evaluation_request_id = stable.request_id
    if evaluation_snapshot_stamp_sec is None:
        evaluation_snapshot_stamp_sec = stable.snapshot_stamp_sec
    return valid_safety_input(
        request_id=evaluation_request_id,
        snapshot_stamp_sec=evaluation_snapshot_stamp_sec,
        target_epoch=stable.target_epoch,
        target_label=stable.target_label,
        model_choice=stable.model_choice,
        track_id=stable.track_id,
        variant_index=variant_index,
        center_base_xyz=stable.center_base_xyz,
        tool0_position_xyz=stable.tool0_position_xyz,
        quaternion_xyzw=(
            stable.quaternion_xyzw if quaternion is None else quaternion
        ),
        approach_base_xyz=(
            stable.approach_base_xyz if approach is None else approach
        ),
        required_open_width_m=stable.required_open_width_m,
        snapshot_context_revision=evaluation_context_revision,
        **overrides
    )


def scored_candidate(
    track_id,
    score,
    variant_index=0,
    safety=None,
    features=None,
    score_weights=None,
    context_revision='ctx-3',
    stable=None,
    evaluation_request_id=None,
    evaluation_snapshot_stamp_sec=None,
):
    stable = (
        stable_candidate(track_id, pre_moveit_score=score)
        if stable is None
        else stable
    )
    if evaluation_request_id is None:
        evaluation_request_id = stable.request_id
    if evaluation_snapshot_stamp_sec is None:
        evaluation_snapshot_stamp_sec = stable.snapshot_stamp_sec
    variant_quaternion = expected_variant_quaternion(stable, variant_index)
    variant_approach = stable.approach_base_xyz
    candidate_features = (
        soft_features(
            model_score=1.0 if score < 0.0 else 0.0,
            center_distance_m=max(0.0, float(score)),
            joint_path_cost=0.0,
            joint_max_delta_rad=0.0,
        )
        if features is None
        else features
    )
    return ScoredStableCandidate(
        stable_candidate=stable,
        variant_index=variant_index,
        latest_safety=(
            safety_for_candidate(
                stable,
                variant_index=variant_index,
                quaternion=variant_quaternion,
                approach=variant_approach,
                evaluation_request_id=evaluation_request_id,
                evaluation_snapshot_stamp_sec=evaluation_snapshot_stamp_sec,
                evaluation_context_revision=context_revision,
            )
            if safety is None
            else safety
        ),
        soft_features=candidate_features,
        score_weights=(
            ranking_score_weights()
            if score_weights is None
            else score_weights
        ),
        evaluation_request_id=evaluation_request_id,
        evaluation_snapshot_stamp_sec=evaluation_snapshot_stamp_sec,
        evaluation_context_revision=context_revision,
    )


def reachable_moveit_result(joint_path_cost=0.0, joint_max_delta_rad=0.0):
    return MoveItResult(
        reachable=True,
        joint_path_cost=joint_path_cost,
        joint_max_delta_rad=joint_max_delta_rad,
        reason='',
        collision_free=True,
        within_joint_limits=True,
        ik_valid=True,
        planning_success=True,
    )


def failed_moveit_result(code):
    states = {
        'collision_free': True,
        'within_joint_limits': True,
        'ik_valid': True,
        'planning_success': True,
    }
    states[code] = False
    return MoveItResult(
        reachable=False,
        joint_path_cost=0.0,
        joint_max_delta_rad=0.0,
        reason='ignored diagnostic text',
        **states
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


@pytest.mark.parametrize('boolean_value', [True, np.bool_(True)])
@pytest.mark.parametrize(
    ('field', 'code'),
    [
        ('snapshot_stamp_sec', 'SAFETY_EVIDENCE_STALE'),
        ('target_absolute_distance_m', 'TRANSFORM_INVALID'),
        ('target_absolute_limit_m', 'TRANSFORM_INVALID'),
        ('required_open_width_m', 'GRIPPER_WIDTH_INVALID'),
        ('physical_open_width_m', 'GRIPPER_CONTRACT_INVALID'),
    ],
)
def test_hard_numeric_fields_reject_python_and_numpy_bool(
    field, code, boolean_value
):
    result = mandatory_safety_gate(
        replace(valid_safety_input(), **{field: boolean_value})
    )

    assert result.ok is False
    assert result.code == code


@pytest.mark.parametrize('boolean_value', [True, np.bool_(True)])
def test_hard_pose_coordinates_reject_python_and_numpy_bool(boolean_value):
    result = mandatory_safety_gate(
        replace(
            valid_safety_input(),
            center_base_xyz=(boolean_value, 0.0, 0.2),
        )
    )

    assert result.ok is False
    assert result.code == 'TRANSFORM_INVALID'


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
        return (
            reachable_moveit_result(float(candidate.track_id), 0.1)
            if candidate.track_id == 3
            else failed_moveit_result('ik_valid')
        )

    candidates = [
        scored_candidate(track_id=i, score=float(i)) for i in range(20)
    ]

    result = bounded_moveit_select(candidates, check, top_n=5)

    assert calls == [0, 1, 2, 3, 4]
    assert result.selected.track_id == 3
    assert result.funnel.remaining_by_stage['moveit_checked'] == 5
    assert result.funnel.remaining_by_stage['moveit_reachable'] == 1


def test_stable_metadata_score_cannot_override_soft_ranking():
    soft_best = scored_candidate(1, 0.0)
    soft_best = replace(
        soft_best,
        stable_candidate=replace(
            soft_best.stable_candidate, pre_moveit_score=999.0
        ),
    )
    soft_worse = scored_candidate(2, 10.0)
    soft_worse = replace(
        soft_worse,
        stable_candidate=replace(
            soft_worse.stable_candidate, pre_moveit_score=-999.0
        ),
    )
    calls = []

    bounded_moveit_select(
        [soft_worse, soft_best],
        lambda candidate: calls.append(candidate.track_id),
        top_n=3,
    )

    assert calls == [1, 2]
    assert soft_best.pre_moveit_score == pytest.approx(
        soft_candidate_cost(
            soft_best.soft_features, soft_best.score_weights
        ).total
    )


def test_final_score_recomputes_full_soft_cost_with_actual_moveit_motion():
    candidate = scored_candidate(
        1,
        0.0,
        features=soft_features(
            joint_path_cost=0.2,
            joint_max_delta_rad=0.3,
        ),
        score_weights=weights(),
    )
    candidate = replace(
        candidate,
        stable_candidate=replace(
            candidate.stable_candidate, pre_moveit_score=999.0
        ),
    )
    actual_path_cost = 0.7
    actual_max_delta = 0.8

    result = bounded_moveit_select(
        [candidate],
        lambda _candidate: reachable_moveit_result(
            actual_path_cost, actual_max_delta
        ),
    )
    expected = soft_candidate_cost(
        replace(
            candidate.soft_features,
            joint_path_cost=actual_path_cost,
            joint_max_delta_rad=actual_max_delta,
        ),
        candidate.score_weights,
    ).total

    assert result.selected.final_score == pytest.approx(expected)


@pytest.mark.parametrize(('configured', 'expected'), [(1, 3), (99, 10)])
def test_moveit_top_n_is_clamped_between_three_and_ten(configured, expected):
    calls = []

    def check(candidate):
        calls.append((candidate.track_id, candidate.variant_index))
        return failed_moveit_result('ik_valid')

    candidates = [
        scored_candidate(track_id=i, score=float(i)) for i in range(20)
    ]

    result = bounded_moveit_select(candidates, check, top_n=configured)

    assert len(calls) == expected
    assert result.configured_top_n == expected
    assert result.funnel.remaining_by_stage['moveit_checked'] == expected


@pytest.mark.parametrize('configured', [True, np.bool_(True)])
def test_moveit_top_n_rejects_boolean_values(configured):
    with pytest.raises(ValueError, match='top_n'):
        bounded_moveit_select([], lambda _candidate: None, top_n=configured)


def test_latest_hard_recheck_precedes_soft_ranking_and_moveit_calls():
    calls = []

    def check(candidate):
        calls.append(candidate.track_id)
        return reachable_moveit_result(0.1, 0.1)

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


def test_evaluation_after_empty_frame_can_recheck_older_stable_source():
    tracker = CandidateTracker(TrackingConfig(window_size=5, min_hits=3))
    for request_id in (1, 2, 3):
        tracker.update(request_id, [tracker_observation(request_id)])
    stable = tracker.update(4, [])[0]
    assert stable.hit_request_ids == (1, 2, 3)
    assert stable.request_id == 3

    candidate = scored_candidate(
        stable.track_id,
        0.0,
        stable=stable,
        evaluation_request_id=4,
        evaluation_snapshot_stamp_sec=104.0,
        context_revision='ctx-4',
    )
    calls = []

    result = bounded_moveit_select(
        [candidate],
        lambda item: calls.append(item) or reachable_moveit_result(),
    )

    assert calls == [candidate]
    assert result.selected is not None


@pytest.mark.parametrize(
    ('evaluation_request_id', 'evaluation_stamp'),
    [
        (2, 10.0),
        (3, 9.0),
    ],
)
def test_evaluation_older_than_stable_source_fails_closed(
    evaluation_request_id, evaluation_stamp
):
    calls = []
    candidate = scored_candidate(
        6,
        0.0,
        evaluation_request_id=evaluation_request_id,
        evaluation_snapshot_stamp_sec=evaluation_stamp,
        context_revision='ctx-evaluation',
    )

    result = bounded_moveit_select(
        [candidate], lambda item: calls.append(item)
    )

    assert calls == []
    assert result.funnel.rejection_counts == {'SAFETY_EVIDENCE_STALE': 1}


def test_safety_request_stamp_and_context_bind_to_evaluation_identity():
    candidate = scored_candidate(
        6,
        0.0,
        evaluation_request_id=4,
        evaluation_snapshot_stamp_sec=11.0,
        context_revision='ctx-4',
    )
    candidate = replace(
        candidate,
        latest_safety=replace(
            candidate.latest_safety,
            request_id=3,
            snapshot_stamp_sec=10.0,
            snapshot_context_revision='ctx-3',
        ),
    )

    result = bounded_moveit_select([candidate], lambda _item: None)

    assert result.funnel.rejection_counts == {'SAFETY_EVIDENCE_STALE': 1}


@pytest.mark.parametrize(
    ('field', 'value'),
    [
        ('evaluation_request_id', True),
        ('evaluation_request_id', np.bool_(True)),
        ('evaluation_request_id', 0),
        ('evaluation_request_id', 1.5),
        ('evaluation_snapshot_stamp_sec', True),
        ('evaluation_snapshot_stamp_sec', np.bool_(True)),
        ('evaluation_snapshot_stamp_sec', 0.0),
        ('evaluation_snapshot_stamp_sec', float('nan')),
        ('evaluation_context_revision', []),
        ('evaluation_context_revision', {}),
        ('evaluation_context_revision', -1),
        ('evaluation_context_revision', ''),
    ],
)
def test_evaluation_identity_fields_are_strictly_validated(field, value):
    with pytest.raises(ValueError):
        replace(scored_candidate(6, 0.0), **{field: value})


@pytest.mark.parametrize(
    ('evidence_change', 'expected_code'),
    [
        ({'request_id': 2}, 'SAFETY_EVIDENCE_STALE'),
        ({'snapshot_stamp_sec': 9.5}, 'SAFETY_EVIDENCE_STALE'),
        ({'snapshot_context_revision': 'ctx-old'}, 'SAFETY_EVIDENCE_STALE'),
        ({'target_epoch': 8}, 'SAFETY_BINDING_MISMATCH'),
        ({'target_label': 'other'}, 'SAFETY_BINDING_MISMATCH'),
        ({'model_choice': 'other-model'}, 'SAFETY_BINDING_MISMATCH'),
        ({'track_id': 99}, 'SAFETY_BINDING_MISMATCH'),
        ({'variant_index': 1}, 'SAFETY_BINDING_MISMATCH'),
        ({'center_base_xyz': (0.1001, 0.0, 0.2)}, 'SAFETY_BINDING_MISMATCH'),
        ({'tool0_position_xyz': (0.1, 0.0001, 0.2)}, 'SAFETY_BINDING_MISMATCH'),
        ({'quaternion_xyzw': (0.0, 0.0, 0.1, 0.994987)}, 'SAFETY_BINDING_MISMATCH'),
        ({'approach_base_xyz': (0.01, 0.0, -0.99995)}, 'SAFETY_BINDING_MISMATCH'),
        ({'required_open_width_m': 0.039}, 'SAFETY_BINDING_MISMATCH'),
    ],
)
def test_stale_or_mismatched_safety_evidence_never_reaches_moveit(
    evidence_change, expected_code
):
    candidate = scored_candidate(6, 0.0)
    candidate = replace(
        candidate,
        latest_safety=replace(candidate.latest_safety, **evidence_change),
    )
    calls = []

    result = bounded_moveit_select(
        [candidate], lambda item: calls.append(item.track_id)
    )

    assert calls == []
    assert result.funnel.rejection_counts == {expected_code: 1}


def test_fused_width_cannot_be_replaced_by_narrower_safety_evidence():
    candidate = scored_candidate(6, 0.0)
    wider_stable = replace(
        candidate.stable_candidate,
        required_open_width_m=0.060,
    )
    candidate = replace(candidate, stable_candidate=wider_stable)
    calls = []

    result = bounded_moveit_select(
        [candidate], lambda item: calls.append(item.track_id)
    )

    assert calls == []
    assert result.funnel.rejection_counts == {'SAFETY_BINDING_MISMATCH': 1}


@pytest.mark.parametrize(
    ('field', 'value'),
    [
        ('quaternion_xyzw', (0.0, 0.0, 0.0, 2.0)),
        ('approach_base_xyz', (0.0, 0.0, -2.0)),
    ],
)
def test_non_unit_safety_directions_fail_before_moveit(field, value):
    candidate = scored_candidate(6, 0.0)
    candidate = replace(
        candidate,
        latest_safety=replace(
            candidate.latest_safety,
            **{field: value}
        ),
    )
    calls = []

    result = bounded_moveit_select(
        [candidate], lambda item: calls.append(item.track_id)
    )

    assert calls == []
    assert result.funnel.rejection_counts == {'TRANSFORM_INVALID': 1}


def test_only_direct_and_local_rz_pi_variants_reach_moveit():
    half_sqrt = math.sqrt(0.5)
    stable = replace(
        stable_candidate(6),
        quaternion_xyzw=np.array([half_sqrt, 0.0, 0.0, half_sqrt]),
    )
    direct = scored_candidate(6, 0.0, variant_index=0, stable=stable)
    symmetric = scored_candidate(6, 0.0, variant_index=1, stable=stable)
    calls = []

    result = bounded_moveit_select(
        [symmetric, direct],
        lambda item: calls.append(item) or reachable_moveit_result(),
        top_n=3,
    )

    expected_direct = tuple(stable.quaternion_xyzw)
    expected_symmetric = multiply_xyzw(
        expected_direct, (0.0, 0.0, 1.0, 0.0)
    )
    assert [item.variant_index for item in calls] == [0, 1]
    assert tuple(calls[0].variant_quaternion_xyzw) == pytest.approx(
        expected_direct
    )
    assert tuple(calls[1].variant_quaternion_xyzw) == pytest.approx(
        expected_symmetric
    )
    assert tuple(calls[1].variant_approach_base_xyz) == pytest.approx(
        stable.approach_base_xyz
    )
    assert len(result.checked) == 2


def test_arbitrary_variant_pose_cannot_be_supplied_by_caller():
    candidate = scored_candidate(6, 0.0)

    with pytest.raises(TypeError):
        replace(
            candidate,
            variant_quaternion_xyzw=(0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)),
        )


def test_rz_90_safety_pose_never_reaches_moveit():
    candidate = scored_candidate(6, 0.0)
    rz_90 = (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5))
    candidate = replace(
        candidate,
        latest_safety=replace(
            candidate.latest_safety,
            quaternion_xyzw=rz_90,
        ),
    )
    calls = []

    result = bounded_moveit_select(
        [candidate], lambda item: calls.append(item)
    )

    assert calls == []
    assert result.funnel.rejection_counts == {'SAFETY_BINDING_MISMATCH': 1}


def test_variant_index_outside_direct_and_rz_pi_is_rejected():
    with pytest.raises(ValueError, match='variant_index'):
        scored_candidate(6, 0.0, variant_index=99)


def test_safety_evidence_defensively_copies_pose_vectors():
    center = np.array([0.1, 0.0, 0.2])
    safety = valid_safety_input(center_base_xyz=center)

    center[0] = 99.0

    assert tuple(safety.center_base_xyz) == pytest.approx((0.1, 0.0, 0.2))


@pytest.mark.parametrize('revision', [[], {}, -1, ''])
def test_safety_context_revision_must_be_an_immutable_scalar(revision):
    result = mandatory_safety_gate(
        replace(valid_safety_input(), snapshot_context_revision=revision)
    )

    assert result.ok is False
    assert result.code == 'SAFETY_EVIDENCE_STALE'


def test_symmetric_variants_each_consume_one_top_n_check_slot():
    calls = []

    def check(candidate):
        calls.append((candidate.track_id, candidate.variant_index))
        return reachable_moveit_result()

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
            return failed_moveit_result('ik_valid')
        if candidate.track_id == 1:
            return failed_moveit_result('within_joint_limits')
        return reachable_moveit_result(0.2, 0.1)

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


@pytest.mark.parametrize(
    ('failed_state', 'expected_code'),
    [
        ('collision_free', 'MOVEIT_COLLISION'),
        ('within_joint_limits', 'MOVEIT_JOINT_LIMIT'),
        ('ik_valid', 'MOVEIT_IK_FAILED'),
        ('planning_success', 'MOVEIT_PLANNING_FAILED'),
    ],
)
def test_structured_moveit_hard_failure_codes_continue_shortlist(
    failed_state, expected_code
):
    calls = []

    def check(candidate):
        calls.append(candidate.track_id)
        if candidate.track_id == 0:
            return failed_moveit_result(failed_state)
        return reachable_moveit_result()

    result = bounded_moveit_select(
        [scored_candidate(0, 0.0), scored_candidate(1, 1.0)],
        check,
        top_n=3,
    )

    assert calls == [0, 1]
    assert result.selected.track_id == 1
    assert result.funnel.rejection_counts == {expected_code: 1}


@pytest.mark.parametrize(
    'contradictory',
    [
        MoveItResult(
            reachable=True,
            joint_path_cost=0.0,
            joint_max_delta_rad=0.0,
            reason='claims success',
            collision_free=False,
            within_joint_limits=True,
            ik_valid=True,
            planning_success=True,
        ),
        MoveItResult(
            reachable=False,
            joint_path_cost=0.0,
            joint_max_delta_rad=0.0,
            reason='claims failure',
            collision_free=True,
            within_joint_limits=True,
            ik_valid=True,
            planning_success=True,
        ),
        MoveItResult(
            reachable=True,
            joint_path_cost=0.0,
            joint_max_delta_rad=0.0,
            reason='missing structured states',
        ),
        MoveItResult(
            reachable=True,
            joint_path_cost=0.0,
            joint_max_delta_rad=0.0,
            reason='numpy bool state',
            collision_free=np.bool_(True),
            within_joint_limits=True,
            ik_valid=True,
            planning_success=True,
        ),
    ],
)
def test_contradictory_or_missing_moveit_hard_states_fail_closed(contradictory):
    result = bounded_moveit_select(
        [scored_candidate(0, 0.0)],
        lambda _candidate: contradictory,
    )

    assert result.selected is None
    assert result.funnel.rejection_counts == {'MOVEIT_RESULT_INVALID': 1}


def test_moveit_motion_cost_can_reorder_reachable_candidates():
    def check(candidate):
        if candidate.track_id == 0:
            return reachable_moveit_result(10.0, 10.0)
        return reachable_moveit_result()

    result = bounded_moveit_select(
        [scored_candidate(0, 0.0), scored_candidate(1, 0.1)],
        check,
        top_n=3,
    )

    assert result.selected.track_id == 1
    assert result.selected.final_score == pytest.approx(
        result.selected.pre_moveit_score
    )
    assert result.reachable[0].final_score > result.reachable[1].final_score


def test_malformed_moveit_result_fails_closed_and_records_dominant_failure():
    calls = []

    def check(candidate):
        calls.append(candidate.track_id)
        if candidate.track_id == 0:
            return reachable_moveit_result(float('nan'), 0.0)
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


@pytest.mark.parametrize('boolean_value', [True, np.bool_(True)])
def test_moveit_motion_metrics_reject_python_and_numpy_bool(boolean_value):
    result = bounded_moveit_select(
        [scored_candidate(0, 0.0)],
        lambda _candidate: reachable_moveit_result(boolean_value, 0.0),
        top_n=3,
    )

    assert result.selected is None
    assert result.funnel.rejection_counts == {'MOVEIT_RESULT_INVALID': 1}


def test_candidates_outside_top_n_are_not_mislabeled_as_hard_rejections():
    candidates = [
        scored_candidate(track_id=i, score=float(i)) for i in range(8)
    ]

    result = bounded_moveit_select(
        candidates,
        lambda _candidate: reachable_moveit_result(),
        top_n=3,
    )

    assert len(result.checked) == 3
    assert result.funnel.rejection_counts == {}
    assert result.funnel.to_dict()['stage_counts']['moveit_shortlist'] == {
        'entered': 8,
        'passed': 3,
        'rejected': 5,
    }
