"""Exact source and measured feedback binding for weak RGB observation evidence."""
import numpy as np
import pytest

from alicia_grasp_modes.rgb_observation_source import (
    bracket_feedback, camera_geometry_snapshot, digest, empty_observation_interest,
    measured_support_coverage, require_fresh_source)
from alicia_grasp_modes.tabletop import Config, _project, _support_quadrants


def test_feedback_is_interpolated_by_source_time_not_arrival_order():
    records = [(1_200_000_000, np.ones(6)), (1_000_000_000, np.zeros(6))]
    q, lo, hi = bracket_feedback(records, 1_050_000_000, 900_000_000)
    np.testing.assert_allclose(q, .25)
    assert lo == (1_000_000_000, [0.] * 6)
    assert hi == (1_200_000_000, [1.] * 6)


@pytest.mark.parametrize('records', [
    [(1_000_000_000, np.zeros(6))], [(1_200_000_000, np.zeros(6))],
    [(999_999_999, np.zeros(6)), (1_200_000_000, np.zeros(6))],
    [(800_000_000, np.zeros(6)), (1_200_000_000, np.zeros(6))],
    [(1_000_000_000, np.full(6, np.nan)), (1_200_000_000, np.zeros(6))],
])
def test_feedback_never_extrapolates_or_widens_200ms_bracket(records):
    with pytest.raises(ValueError, match='does_not_bracket'):
        bracket_feedback(records, 1_100_000_000, 900_000_000)


def test_conflicting_duplicate_feedback_cannot_be_hidden_by_arrival_order():
    with pytest.raises(ValueError, match='conflicting'):
        bracket_feedback([(1_000_000_000, np.zeros(6)), (1_000_000_000, np.ones(6)),
                          (1_200_000_000, np.zeros(6))], 1_100_000_000, 900_000_000)


@pytest.mark.parametrize('stamp,epoch', [(True, 1), (100, True), (100., 1), (100, 100), (99, 100)])
def test_exact_source_epoch_requires_positive_integers(stamp, epoch):
    with pytest.raises(ValueError):
        bracket_feedback([], stamp, epoch)


def test_geometry_digest_ignores_filter_telemetry_but_not_intrinsics_or_device():
    camera = dict(fx=400., fy=400., runtime_profile=dict(
        device={'serial_number': 'camera_a'}, geometry_intrinsics={'fx': 400.},
        filter_telemetry={'frames': 10}))
    old = camera_geometry_snapshot(camera)
    camera['runtime_profile']['filter_telemetry']['frames'] += 1
    assert digest(old) == digest(camera_geometry_snapshot(camera))
    camera['runtime_profile']['device']['serial_number'] = 'camera_b'
    assert digest(old) != digest(camera_geometry_snapshot(camera))
    assert old['runtime_profile_geometry']['device']['serial_number'] == 'camera_a'
    camera['fx'] = 401.
    assert digest(old) != digest(camera_geometry_snapshot(camera))


@pytest.mark.parametrize('now,max_age', [(1_800_000_001, .8), (999_999_999, .8),
                                       (1_300_000_001, .3), (1_000_000_000, .80001)])
def test_source_age_is_never_renewed_or_relaxed(now, max_age):
    with pytest.raises(ValueError):
        require_fresh_source(1_000_000_000, now, max_age)
    assert require_fresh_source(1_000_000_000, 1_800_000_000) == pytest.approx(.8)


def test_sparse_support_coverage_matches_existing_projected_pixel_equation():
    config = Config()
    depth = np.full((240, 320), .4, np.float32)
    depth[30:100, 100:160] = 0.
    depth[150:180, 140:240] = .5
    intrinsics = (400., 405., 160., 120.)
    transform = np.array([[0., -1., 0., .1], [0., 0., 1., -.2],
                          [-1., 0., 0., .4], [0., 0., 0., 1.]])
    plane = np.array((0., 0., 1., 0.))
    outer = 85.
    actual = measured_support_coverage(depth, intrinsics, transform, plane, config, outer)
    yy, xx = np.indices(depth.shape)
    center = (160., 120.)
    radius = np.hypot(xx-center[0], yy-center[1])
    plane_camera = transform.T @ plane
    distance = np.einsum('ijk,k->ij', _project(depth, intrinsics), plane_camera[:3]) + plane_camera[3]
    supporting = ((depth >= config.depth_min_m) & (depth <= config.depth_max_m)
                  & (radius >= 35.) & (radius <= outer)
                  & (np.abs(distance) <= config.support_threshold_m))
    assert actual == dict(spatial_quadrants=_support_quadrants(supporting, xx, yy, center),
                          measured_support_pixel_count=int(np.count_nonzero(supporting)))


def test_tombstone_never_carries_previous_volume_or_authority():
    result = empty_observation_interest('changed')
    assert result['observation_only'] is True
    assert not any(result[key] for key in ('observation_interest_ready',
        'metric_geometry_authority', 'object_identity_confirmed',
        'motion_authorized', 'contact_geometry_authorized'))
    assert 'interest_volume' not in result
