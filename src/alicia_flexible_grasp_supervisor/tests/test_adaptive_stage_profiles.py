import math

import pytest

from alicia_flexible_grasp.grasp.adaptive_stage_profiles import (
    AdaptiveStageLimits,
    adaptive_stage_profile_for_tilt,
    derive_adaptive_stage_profiles,
)


def profiles(**overrides):
    values = {
        'object_height_m': 0.018,
        'depth_repeatability_m': 0.003,
        'finger_length_m': 0.060,
        'support_clearance_m': 0.003,
        'limits': AdaptiveStageLimits(),
    }
    values.update(overrides)
    return derive_adaptive_stage_profiles(**values)


def test_stage_profiles_are_deterministic_and_cover_admissible_tilt_interval():
    first = profiles()
    second = profiles()

    assert first == second
    assert len(first) == 5
    assert first[0].tilt_deg == 0.0
    assert [item.tilt_deg for item in first] == sorted(
        item.tilt_deg for item in first
    )
    assert first[-1].tilt_deg > 15.0
    assert first[-1].tilt_deg < 45.0


def test_every_profile_preserves_physical_sweep_and_stage_order():
    limits = AdaptiveStageLimits(max_lateral_sweep_m=0.010)

    for profile in profiles(limits=limits):
        assert profile.lateral_sweep_m <= limits.max_lateral_sweep_m + 1e-12
        assert (
            profile.pregrasp_distance_m
            >= profile.approach_offset_m
            + limits.pregrasp_approach_gap_min_m
            - 1e-12
        )
        assert (
            math.cos(math.radians(profile.tilt_deg))
            >= limits.min_downward_cos - 1e-12
        )


def test_lateral_sweep_tracks_final_approach_not_normal_pregrasp():
    sampled = profiles(
        execution_position_error_m=0.0228,
        limits=AdaptiveStageLimits(max_lateral_sweep_m=0.020),
    )
    tilted = sampled[-1]

    assert tilted.lateral_sweep_m == pytest.approx(
        tilted.approach_offset_m
        * math.sin(math.radians(tilted.tilt_deg))
    )
    assert tilted.pregrasp_distance_m > tilted.approach_offset_m
    assert tilted.lateral_sweep_m == pytest.approx(0.020)


def test_object_geometry_and_depth_repeatability_change_stage_distances():
    compact = profiles(
        object_height_m=0.010,
        depth_repeatability_m=0.001,
    )
    tall_noisy = profiles(
        object_height_m=0.050,
        depth_repeatability_m=0.006,
    )

    assert (
        tall_noisy[0].approach_offset_m
        > compact[0].approach_offset_m
    )
    assert tall_noisy[0].lift_height_m > compact[0].lift_height_m
    assert (
        tall_noisy[0].depth_uncertainty_m
        > compact[0].depth_uncertainty_m
    )


def test_contact_overlap_uses_general_floor_and_live_depth_repeatability():
    quiet = profiles(depth_repeatability_m=0.0005)
    noisy = profiles(depth_repeatability_m=0.003)
    measured = profiles(
        depth_repeatability_m=0.0005,
        execution_position_error_m=0.0132,
    )

    assert quiet[0].contact_overlap_requirement_m == pytest.approx(0.002)
    assert noisy[0].contact_overlap_requirement_m == pytest.approx(0.006)
    assert measured[0].contact_overlap_requirement_m == pytest.approx(0.002)
    assert measured[0].depth_uncertainty_m > quiet[0].depth_uncertainty_m


def test_live_execution_error_expands_clearance_and_reduces_tilt():
    baseline = profiles()
    measured = profiles(execution_position_error_m=0.0228)

    assert measured[0].execution_position_error_m == pytest.approx(0.0228)
    assert measured[0].approach_offset_m > baseline[0].approach_offset_m
    assert measured[0].pregrasp_distance_m > baseline[0].pregrasp_distance_m
    assert measured[-1].tilt_deg < baseline[-1].tilt_deg
    for profile in measured:
        assert profile.pregrasp_distance_m >= (
            profile.approach_offset_m
            + profile.execution_position_error_m
            + AdaptiveStageLimits().pregrasp_approach_gap_min_m
            - 1e-12
        )


def test_execution_error_that_cannot_fit_stage_bounds_fails_closed():
    with pytest.raises(ValueError, match='execution uncertainty'):
        profiles(
            execution_position_error_m=0.061,
            limits=AdaptiveStageLimits(
                approach_max_m=0.060,
                pregrasp_max_m=0.095,
            ),
        )


def test_runtime_tilt_uses_same_geometry_driven_distance_contract():
    limits = AdaptiveStageLimits()
    sampled = profiles(limits=limits)
    requested = adaptive_stage_profile_for_tilt(
        tilt_deg=sampled[-1].tilt_deg,
        object_height_m=0.018,
        depth_repeatability_m=0.003,
        finger_length_m=0.060,
        support_clearance_m=0.003,
        limits=limits,
    )

    assert requested == sampled[-1]


def test_tilt_outside_hard_bounds_has_no_profile():
    with pytest.raises(ValueError, match='hard bounds'):
        adaptive_stage_profile_for_tilt(
            tilt_deg=40.0,
            object_height_m=0.018,
            depth_repeatability_m=0.003,
            finger_length_m=0.060,
            support_clearance_m=0.003,
            limits=AdaptiveStageLimits(max_lateral_sweep_m=0.010),
        )


@pytest.mark.parametrize(
    'kwargs',
    (
        {'tilt_sample_count': 0},
        {'max_lateral_sweep_m': 0.0},
        {'approach_min_m': 0.04, 'approach_max_m': 0.03},
        {'pregrasp_min_m': 0.07, 'pregrasp_max_m': 0.06},
        {
            'approach_max_m': 0.05,
            'pregrasp_max_m': 0.05,
            'pregrasp_approach_gap_min_m': 0.005,
        },
        {'lift_min_m': 0.09, 'lift_max_m': 0.08},
    ),
)
def test_invalid_physical_search_contract_fails_closed(kwargs):
    with pytest.raises(ValueError):
        AdaptiveStageLimits(**kwargs)
