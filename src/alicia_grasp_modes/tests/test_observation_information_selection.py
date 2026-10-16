"""Real ModeAware information selection hooks; no ROS planning or motion."""
from copy import deepcopy
import json
import math
from types import SimpleNamespace as NS

import numpy as np
import pytest

from test_remote_modes import configured, base
from test_observe_return_remote import setup_observe_return
from test_observe_return_visual import measured_gap_fixture, camera_rotation_toward
from alicia_grasp_modes.observation_policy import WORKING_VOLUME_POLICY
from alicia_grasp_modes.observation_view_utility import predicted_information_utility


def information_context(configured, monkeypatch):
    node, _, _ = setup_observe_return(configured)
    _, center, _, initial, gaps = measured_gap_fixture()
    identity = node._current_stream_target_identity()
    node._observe_return_phase = 'observation_prefix'
    node._observe_return_diagnostic_observation_only = False
    node._observe_return_view_gaps = tuple(dict(gap, target_epoch=identity.epoch,
        target_track_id=identity.track_id) for gap in gaps[:1])
    node._observe_return_directed_views = ()
    node.grasp_config = dict(observation_acceptance_policy=WORKING_VOLUME_POLICY)
    camera = NS(fx=400., fy=380., cx=320., cy=240., width=640, height=480)
    monkeypatch.setattr(node, '_camera_intrinsics', lambda: camera)
    monkeypatch.setattr(node, '_tool_from_camera_matrix', lambda: np.eye(4))
    monkeypatch.setattr(node, '_observation_view_limit_configuration', lambda: {
        'effective_visibility_limits': dict(roi_margin_px=36., minimum_depth_m=.07, maximum_depth_m=.5)})
    geometry = NS(center_base=center, axes_base=np.eye(3), size_xyz_m=np.array([.045, .03, .02]))
    frozen = np.eye(4)
    frozen[:3, :3] = camera_rotation_toward(center, initial)
    frozen[:3, 3] = initial
    prepared = NS(pose_estimator=NS(T_base_camera_link=frozen))
    node._stable_variant_runtime = {}
    return node, geometry, prepared, camera


def candidate_view(geometry, gain_m, distance=.3, uncertainty=.003):
    # The measured fixture has a previously unseen +Y face. Height times
    # the +Y bearing is exactly the existing physical missing-side exposure.
    bearing_y = gain_m / geometry.size_xyz_m[2]
    origin = geometry.center_base + distance * np.array([0., bearing_y, math.sqrt(1.-bearing_y**2)])
    rotation = camera_rotation_toward(geometry.center_base, origin)
    pose = base.make_pose_stamped('base_link', origin, base.quaternion_from_matrix(
        np.block([[rotation, np.zeros((3, 1))], [np.zeros((1, 3)), np.ones((1, 1))]])),
        stamp=base.rospy.Time(10))
    sequence = NS(pregrasp=pose, adaptive_stage_profile=NS(depth_uncertainty_m=uncertainty),
        observation_view_audit={})
    return dict(sequence=sequence, observation_view={}, observation_translation_delta_m=.1)


def qualify_variants(node, geometry, variants, monkeypatch):
    # Reuse already generated poses; execute the actual mode layer's eight-
    # corner working-volume check, live missing-side filter and utility builder.
    monkeypatch.setattr(base.RemoteGrasp6DNode, '_far_field_observation_variants',
        lambda *_args, **_kwargs: (tuple(variants), ()))
    return node._far_field_observation_variants(geometry=geometry, snapshot=NS(stamp_ns=10_000_000_000))


def checked_result(duration, max_delta=.2, path_cost=.3):
    return base.MoveItResult(True, path_cost, max_delta,
        'observation_screened_duration_sec=%.9f execution_duration_lower_bound_sec=1.0' % duration,
        True, True, True, True)


def family(node, prepared, variant, index, duration, max_delta=.2, path_cost=.3):
    candidate = NS(track_id=index, variant_index=0, moveit_result=checked_result(duration, max_delta, path_cost))
    node._stable_variant_runtime[(index, 0)] = dict(prepared=prepared,
        observation_sequence=variant['sequence'], observation_view=variant['observation_view'])
    return candidate


def test_real_checked_and_cross_family_rank_prefer_useful_15_seconds_over_barely_useful_12(configured, monkeypatch):
    node, geometry, prepared, _ = information_context(configured, monkeypatch)
    variants, rejected = qualify_variants(node, geometry,
        [candidate_view(geometry, .003012), candidate_view(geometry, .012)], monkeypatch)
    assert len(variants) == 2 and not rejected
    weak, useful = variants
    a = node._checked_observation_variant_rank(checked_result(12.), weak)
    b = node._checked_observation_variant_rank(checked_result(15., .4, .5), useful)
    assert b < a
    fast = family(node, prepared, weak, 1, 12.)
    informative = family(node, prepared, useful, 2, 15., .4, .5)
    assert node._select_far_field_observation([fast, informative]) is informative
    assert node._select_far_field_observation([informative, fast]) is informative
    audit = useful['observation_view']['checked_information_motion_rank']
    assert audit['screened_duration_sec'] == 15.
    assert audit['prediction_only'] and not audit['claims_global_optimum']


def test_saturated_benefit_prefers_five_second_small_motion_over_sixteen_second_large_turn(configured, monkeypatch):
    node, geometry, prepared, _ = information_context(configured, monkeypatch)
    variants, rejected = qualify_variants(node, geometry,
        [candidate_view(geometry, .016), candidate_view(geometry, .019)], monkeypatch)
    assert len(variants) == 2 and not rejected
    small, large = variants
    first = small['observation_view']['information_utility']['utility']
    second = large['observation_view']['information_utility']['utility']
    assert 0. < first < second < 1. and second / first < 1.2
    small_result, large_result = checked_result(5., .25, .4), checked_result(16., 2.8, 3.2)
    assert node._checked_observation_variant_rank(small_result, small) < node._checked_observation_variant_rank(large_result, large)
    fast = family(node, prepared, small, 1, 5., .25, .4)
    turn = family(node, prepared, large, 2, 16., 2.8, 3.2)
    assert node._select_far_field_observation([turn, fast]) is fast


@pytest.mark.parametrize('fault', ['no_measured_gap', 'wrong_target', 'same_side', 'no_baseline'])
def test_high_utility_cannot_bypass_live_missing_side_or_distinct_view_filter(configured, monkeypatch, fault):
    node, geometry, prepared, _ = information_context(configured, monkeypatch)
    variants, _ = qualify_variants(node, geometry, [candidate_view(geometry, .012)], monkeypatch)
    candidate = family(node, prepared, variants[0], 1, 1.)
    assert node._select_far_field_observation([candidate]) is candidate
    if fault == 'no_measured_gap':
        node._observe_return_view_gaps = ()
    elif fault == 'wrong_target':
        node._observe_return_view_gaps = tuple(dict(gap, target_track_id='other') for gap in node._observe_return_view_gaps)
    elif fault == 'same_side':
        variants[0]['sequence'].pregrasp.pose.position.y *= -1.
    else:
        prepared.pose_estimator.T_base_camera_link = base.pose_matrix(variants[0]['sequence'].pregrasp)
    assert node._select_far_field_observation([candidate]) is None


def test_zero_utility_has_infinite_rank_but_finite_json_audit(configured, monkeypatch):
    node, geometry, _, _ = information_context(configured, monkeypatch)
    variants, _ = qualify_variants(node, geometry, [candidate_view(geometry, .012)], monkeypatch)
    variant = deepcopy(variants[0])
    variant['observation_view']['information_utility'] = predicted_information_utility(.003, .003, .3, 380.)
    rank = node._checked_observation_variant_rank(checked_result(5.), variant)
    assert math.isinf(rank[0])
    audit = variant['observation_view']['checked_information_motion_rank']
    assert audit['cost'] is None and audit['utility'] == 0.
    json.dumps(variant['observation_view'], allow_nan=False)


def test_actual_variant_utilities_use_farthest_corner_depth_and_weaker_focal_axis(configured, monkeypatch):
    node, geometry, _, camera = information_context(configured, monkeypatch)
    inputs = [candidate_view(geometry, .012, .2), candidate_view(geometry, .012, .4)]
    variants, rejected = qualify_variants(node, geometry, inputs, monkeypatch)
    assert len(variants) == 2 and not rejected
    first_values = []
    for variant in variants:
        audit = variant['observation_view']
        gain = audit['observe_return_missing_side_gain']
        depth = max(audit['working_volume']['target_corner_depths_m'])
        benefit = audit['information_utility']
        assert benefit == predicted_information_utility(gain['predicted_new_side_gain_m'],
            gain['depth_uncertainty_m'], depth, min(camera.fx, camera.fy))
        assert benefit['optical_depth_m'] > np.linalg.norm(base.pose_matrix(variant['sequence'].pregrasp)[:3, 3] - geometry.center_base)
        first_values.append(benefit['utility'])
    assert first_values[0] > first_values[1]
    # Doubling only the stronger focal axis must not overstate resolution.
    camera.fx *= 2.
    same, _ = qualify_variants(node, geometry, deepcopy(inputs), monkeypatch)
    assert [v['observation_view']['information_utility']['utility'] for v in same] == pytest.approx(first_values)
    # Reducing the weaker axis reduces the actual mode-generated utility.
    camera.fy *= .5
    reduced, _ = qualify_variants(node, geometry, deepcopy(inputs), monkeypatch)
    assert all(v['observation_view']['information_utility']['utility'] < old for v, old in zip(reduced, first_values))
