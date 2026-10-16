"""Independent cross-boundary regressions for bounded observation comparison."""
import json
import math
from types import SimpleNamespace as NS

import pytest

from test_remote_modes import configured
from test_observation_information_comparison import context
from alicia_grasp_modes.observation_view_utility import predicted_information_utility


@pytest.mark.parametrize('utility', [0., 1.e-320])
def test_checked_zero_or_tiny_utility_keeps_audit_finite(configured, utility):
    node, _ = configured
    node._information_driven_observation_comparison = lambda: True
    variant = dict(observation_view=dict(information_utility=dict(utility=utility)))
    result = NS(reason='observation_screened_duration_sec=12.0', joint_max_delta_rad=.1,
                joint_path_cost=.2)
    rank = node._checked_observation_variant_rank(result, variant)
    assert rank  # Ranking may retain an infinity sentinel internally.
    json.dumps(variant['observation_view'], allow_nan=False)
    assert variant['observation_view']['checked_information_motion_rank']['claims_global_optimum'] is False


@pytest.mark.parametrize('arguments', [
    (1.e308, .003, .3, 400.), (.01, 1.e308, .3, 400.), (.01, .003, 1.e-320, 400.),
])
def test_finite_inputs_cannot_produce_nonfinite_information_audit(arguments):
    try:
        report = predicted_information_utility(*arguments)
    except (ValueError, OverflowError):
        return  # Explicitly rejecting unrepresentable geometry is valid.
    assert math.isfinite(report['utility']) and 0. <= report['utility'] <= 1.
    json.dumps(report, allow_nan=False)


@pytest.mark.parametrize('source_now,tilt', [(1000., 0.), (1000., 10.), (40., 0.), (40., 10.)])
def test_expired_source_cannot_start_roll_checks_outside_tilt_budget(source_now, tilt):
    node, _elapsed, _runtime, variants, checked = context()
    try:
        node._stream_source_clock = lambda: source_now  # Frozen source=49.8, lifetime=120.
        for variant in variants:
            variant['camera_tilt_offset_deg'] = tilt
        result = node._check_moveit_stable_candidate(NS(track_id=3, variant_index=1))
        assert not result.reachable
        assert checked == []
    finally:
        node.shutdown_streaming_worker()
