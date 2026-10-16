"""Offline geometric properties and orientation-specific observation jobs."""
from copy import deepcopy
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest

from alicia_grasp_modes.observation_policy import observation_distance_preferences, WORKING_VOLUME_POLICY
from alicia_flexible_grasp.grasp.observation_working_volume import (
    observation_obb_corners, validate_observation_working_volume,
)

K = (438.6, 438.4, 317.7, 245.3)


def rotation(rng):
    result, _ = np.linalg.qr(rng.normal(size=(3, 3)))
    if np.linalg.det(result) < 0.:
        result[:, 0] *= -1.
    return result


def test_every_proposed_distance_fits_all_corners_over_random_rotated_scenes():
    rng = np.random.RandomState(43129)
    for _ in range(150):
        center = rng.uniform(-.5, .5, 3)
        camera_rotation, object_rotation = rotation(rng), rotation(rng)
        corners = observation_obb_corners(center, object_rotation, rng.uniform(.005, .12, 3))
        current_position = center + rng.uniform(-.6, .6, 3)
        values = observation_distance_preferences(corners, center, camera_rotation, current_position, K, 640, 480)
        assert 1 <= len(values) <= 5
        assert len(values) == len(set(values))
        transform = np.eye(4)
        transform[:3, :3] = camera_rotation
        for value in values:
            transform[:3, 3] = center - value * camera_rotation[:, 0]
            assert validate_observation_working_volume(corners, transform, K, 640, 480,
                source_stamp_ns=1, camera_transform_stamp_ns=1)['ok']


def test_distance_preferences_are_invariant_under_common_rigid_frame_change():
    rng = np.random.RandomState(9871)
    center = np.array([.31, -.22, .19])
    object_rotation, camera_rotation, frame_rotation = rotation(rng), rotation(rng), rotation(rng)
    corners = observation_obb_corners(center, object_rotation, [.06, .03, .05])
    position = np.array([.1, .2, .3])
    values = observation_distance_preferences(corners, center, camera_rotation, position, K, 640, 480)
    offset = np.array([.02, .05, -.08])
    moved = observation_distance_preferences(corners @ frame_rotation.T + offset,
        frame_rotation @ center + offset, frame_rotation @ camera_rotation,
        frame_rotation @ position + offset, K, 640, 480)
    assert moved == pytest.approx(values, abs=1e-12)


def test_infeasible_field_of_view_has_no_distance_proposals():
    center = np.zeros(3)
    corners = observation_obb_corners(center, np.eye(3), [.02, 2., 2.])
    assert observation_distance_preferences(corners, center, np.eye(3), [.25, 0., 0.], K, 640, 480) == ()


def _remote_modules():
    modes = Path(__file__).resolve().parents[1]
    supervisor_tests = modes.parent / 'alicia_flexible_grasp_supervisor' / 'tests'
    if str(supervisor_tests) not in sys.path:
        sys.path.insert(0, str(supervisor_tests))
    import test_observation_distance_search as scene
    spec = importlib.util.spec_from_file_location('distance_preferences_mode_remote',
                                                modes / 'scripts' / 'mode_aware_remote_grasp6d.py')
    mode = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mode)
    return scene, mode


def test_production_intrinsics_and_cached_orientation_specific_views(monkeypatch):
    scene, mode = _remote_modules()
    legacy, geometry, prepared, reference = scene.recorded_scene((0., 90.))
    node = mode.ModeAwareRemoteGrasp6DNode.__new__(mode.ModeAwareRemoteGrasp6DNode)
    node.__dict__.update(legacy.__dict__)
    node._observe_return_selected = lambda: False
    node._mode_selection = {'mode': 'unknown', 'strategy': 'two_stage'}
    node.grasp_config = dict(node.grasp_config,
        observation_acceptance_policy=WORKING_VOLUME_POLICY,
        observation_camera_target_min_distance_m=.070,
        observation_camera_target_nominal_distance_m=.26,
        observation_camera_target_max_distance_m=.500)
    camera = mode.original.CameraIntrinsics(640, 480, *K, .0001)
    node._camera_intrinsics = lambda: camera
    node._observation_view_limit_configuration = lambda: {'effective_visibility_limits': {
        'roi_margin_px': 24., 'minimum_depth_m': .070, 'maximum_depth_m': .500}}
    snapshot = NS(stamp_ns=prepared.stamp.to_nsec())
    # The profile builder uses depth statistics only when a full snapshot is
    # provided. Retain the same existing profile in this camera-only regression.
    profile = legacy._contact_stage_profile(geometry, -geometry.support_normal_base)
    node._contact_stage_profile = lambda *args, **kwargs: deepcopy(profile)
    cache = {}
    first, _ = node._far_field_observation_variants(reference, geometry,
        -geometry.support_normal_base, snapshot, reference, prepared, view_cache=cache)
    changed = deepcopy(reference)
    changed.pose.position.x += .012
    expected, expected_rejected = node._far_field_observation_variants(changed, geometry,
        -geometry.support_normal_base, snapshot, reference, prepared)
    monkeypatch.setattr(node, '_observation_envelope_gate',
        lambda *args: pytest.fail('same frozen request must reuse cached endpoint CAD'))
    actual, actual_rejected = node._far_field_observation_variants(changed, geometry,
        -geometry.support_normal_base, snapshot, reference, prepared, view_cache=cache)
    assert len(first) == len(actual) == len(expected) > 0
    assert actual_rejected == expected_rejected
    for got, want in zip(actual, expected):
        assert got['observation_view'] == want['observation_view']
        for stage in ('pregrasp', 'approach', 'grasp', 'lift'):
            np.testing.assert_array_equal(mode.original.pose_matrix(getattr(got['sequence'], stage)),
                                          mode.original.pose_matrix(getattr(want['sequence'], stage)))
        assert got['observation_view']['working_volume']['actual_arrival_measurement_required']


def test_unequal_orientation_distance_counts_keep_global_preference_indices_unique():
    scene, _ = _remote_modules()
    node, geometry, prepared, reference = scene.recorded_scene()
    node.observation_envelope_gate_enabled = False
    # A valid new policy may return no feasible distance for one roll and five
    # for another. One separate tilt still needs unique ranking/audit identity.
    tilt = deepcopy(reference)
    roll_empty, roll_valid = deepcopy(reference), deepcopy(reference)
    node._observation_reference_roll_variants = lambda _: (
        dict(preference_index=0, camera_roll_offset_deg=0., reference_pose=roll_empty),
        dict(preference_index=1, camera_roll_offset_deg=90., reference_pose=roll_valid))
    node._observation_reference_tilt_variants = lambda *_: (
        dict(preference_index=0, camera_roll_offset_deg=0., camera_tilt_offset_deg=15., reference_pose=tilt),)
    node._observation_distance_preferences = lambda geometry, snapshot, pose, prepared: (
        () if pose is roll_empty else ((.2,) if pose is tilt else (.18, .19, .20, .21, .22)))
    node._observation_side_evidence = lambda *args: {'observation_side_evidence_deficit_m': 0.}
    passing, rejected = scene.variants(node, geometry, prepared, reference)
    assert len(passing) == 6 and not rejected
    indices = [row['preference_index'] for row in passing]
    assert len(indices) == len(set(indices)), indices
