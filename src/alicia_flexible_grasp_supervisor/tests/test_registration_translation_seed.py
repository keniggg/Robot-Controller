"""A third measured initialization must not change registration authority."""
from dataclasses import replace
from pathlib import Path
import hashlib

import numpy as np
import pytest

from alicia_flexible_grasp.vision import multiview_surface as registration
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity


RECORD = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/'
              'full_latest_ros_20261009T003407Z/gen7_registration_analysis/'
              'registration_inputs.npz')
RECORD_SHA256 = '1d374fcb76efea70190365a9f74fbc68e7bbf6626b307d2a10a11bbd2c3afbb0'


@pytest.fixture
def recorded_pair():
    if not RECORD.exists():
        pytest.skip('sealed gen7 measured point clouds are retained on SanStand')
    assert hashlib.sha256(RECORD.read_bytes()).hexdigest() == RECORD_SHA256
    with np.load(RECORD, allow_pickle=False) as data:
        identity = TargetTrackIdentity.from_stream(1, 4)
        return tuple(registration.SurfaceView(
            identity=identity, stamp_ns=int(data[name + '_stamp_ns']),
            points_base=data[name + '_points'],
            support_normal_base=data[name + '_normal'],
            support_offset_m=float(data[name + '_offset']),
            edge_clearance_px=int(data[name + '_clearance']))
            for name in ('reference', 'moving'))


def cost(reference, moving, transform, config):
    _, _, distances, inliers = registration._registration_correspondences(
        registration._sorted_points(reference.points_base),
        registration._sorted_points(moving.points_base), transform, config)
    return float(np.mean(np.where(inliers, distances ** 2,
                                  config.correspondence_max_m ** 2)))


def test_recorded_third_seed_wins_original_initial_objective_and_one_icp_budget(
        recorded_pair, monkeypatch):
    reference, moving = recorded_pair
    before = tuple(v.points_base.tobytes() for v in recorded_pair)
    config = registration.RegistrationConfig()
    basis = registration._support_basis(reference.support_normal_base)
    coarse = registration._initial_support_transform(
        reference.points_base, moving.points_base, basis)
    translation = registration._initial_support_transform(
        reference.points_base, moving.points_base, basis, align_yaw=False)
    np.testing.assert_allclose(translation[:3, :3], np.eye(3), rtol=0, atol=1e-15)
    assert cost(reference, moving, translation, config) == pytest.approx(1.3362887050324241e-5)
    assert cost(reference, moving, translation, config) < cost(reference, moving, coarse, config)
    assert cost(reference, moving, coarse, config) < cost(reference, moving, np.eye(4), config)
    updates = []
    original = registration._rigid_update_about_support

    def counted(source, target, basis):
        updates.append(source.copy())
        return original(source, target, basis)

    monkeypatch.setattr(registration, '_rigid_update_about_support', counted)
    result = registration.register_surface_view(reference, moving, config)
    assert result.ok, result.code
    assert len(updates) == 5 <= config.maximum_iterations
    assert registration._yaw_degrees(result.transform_base[:3, :3], basis) == pytest.approx(.0858007675)
    assert result.inlier_count == 679
    assert result.overlap_fraction == pytest.approx(679 / 829)
    assert result.rmse_m == pytest.approx(.0011471377813921592)
    assert result.maximum_point_displacement_m == pytest.approx(.005789527812614619)
    assert tuple(v.points_base.tobytes() for v in recorded_pair) == before
    assert all(not v.points_base.flags.writeable for v in recorded_pair)


def test_recorded_original_two_seed_policy_still_reproduces_exact_failure(
        recorded_pair, monkeypatch):
    original = registration._initial_support_transform
    # Offering PCA again as the extra seed cannot strictly improve its cost;
    # this recovers the original two-seed choice and unchanged ICP loop.
    monkeypatch.setattr(registration, '_initial_support_transform',
        lambda reference, moving, basis, **kwargs: original(reference, moving, basis))
    result = registration.register_surface_view(*recorded_pair)
    basis = registration._support_basis(recorded_pair[0].support_normal_base)
    assert not result.ok
    assert result.code == 'YAW_BOUND_EXCEEDED'
    assert registration._yaw_degrees(result.transform_base[:3, :3], basis) == pytest.approx(13.533172911647148)


def test_previous_recording_also_selects_translation_by_cost_before_any_bounds():
    from test_multiview_surface import recorded_observation_recovery_pair
    reference, moving, config = recorded_observation_recovery_pair()
    basis = registration._support_basis(reference.support_normal_base)
    coarse = registration._initial_support_transform(
        reference.points_base, moving.points_base, basis)
    translation = registration._initial_support_transform(
        reference.points_base, moving.points_base, basis, align_yaw=False)
    assert cost(reference, moving, translation, config) < min(
        cost(reference, moving, coarse, config),
        cost(reference, moving, np.eye(4), config))
    result = registration.register_surface_view(reference, moving, config)
    assert result.ok, result.code
    assert registration._yaw_degrees(result.transform_base[:3, :3], basis) == pytest.approx(.09038503819)
    assert result.maximum_point_displacement_m < config.maximum_translation_m
    assert result.inlier_count >= config.minimum_inliers
    assert result.overlap_fraction >= config.minimum_overlap_fraction
    assert result.rmse_m <= config.maximum_rmse_m


@pytest.mark.parametrize('change, code', [
    ({'maximum_yaw_deg': .01}, 'YAW_BOUND_EXCEEDED'),
    ({'maximum_translation_m': .001}, 'TRANSLATION_BOUND_EXCEEDED'),
    ({'minimum_inliers': 700}, 'INLIERS_INSUFFICIENT'),
    ({'maximum_rmse_m': .001}, 'RMSE_EXCEEDED'),
])
def test_recorded_new_seed_does_not_select_a_different_fit_to_pass_bounds(
        recorded_pair, change, code):
    normal = registration.register_surface_view(*recorded_pair)
    rejected = registration.register_surface_view(
        *recorded_pair, replace(registration.RegistrationConfig(), **change))
    assert not rejected.ok
    assert rejected.code == code
    np.testing.assert_array_equal(rejected.transform_base, normal.transform_base)


@pytest.mark.parametrize('case', ['no_overlap', 'narrow_fragment'])
def test_third_seed_keeps_original_overlap_and_span_admission(monkeypatch, case):
    points = np.asarray([(x, y, .02) for x in np.linspace(-.025, .025, 21)
                         for y in np.linspace(-.022, .022, 19)])
    moving = points + [.20, 0., 0.] if case == 'no_overlap' else points[np.abs(points[:, 0]) < .003]
    identity = TargetTrackIdentity.from_stream(1, 4)
    reference = registration.SurfaceView(identity, 100, points, [0., 0., 1.], 0., 12)
    other = registration.SurfaceView(identity, 200, moving, [0., 0., 1.], 0., 12)

    def forbidden(*args, **kwargs):
        pytest.fail('coarse initialization must retain original admission')

    monkeypatch.setattr(registration, '_initial_support_transform', forbidden)
    result = registration.register_surface_view(reference, other)
    assert not result.ok
