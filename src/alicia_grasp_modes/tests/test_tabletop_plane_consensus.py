"""Original support quality gates survive a worse least-squares refinement."""
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from alicia_grasp_modes.tabletop import Config, _fit_plane


def asymmetric_measured_cloud():
    rng = np.random.RandomState(11)
    xy = rng.uniform(-.1, .1, (1800, 2))
    z = np.r_[np.full(900, -.0019), np.full(378, .0019),
              np.zeros(180), rng.uniform(.02, .1, 342)]
    points = np.column_stack((xy, z))
    rng.shuffle(points)
    return points


def test_svd_cannot_discard_an_already_qualified_measured_consensus():
    points = asymmetric_measured_cloud()
    original_inliers = np.abs(points[:, 2]) <= .002
    assert original_inliers.mean() == .81
    # The asymmetric measured noise makes ordinary squared-error refinement
    # move away from the middle plane and lose a complete measured stratum.
    center = points[original_inliers].mean(axis=0)
    _, _, vh = np.linalg.svd(points[original_inliers] - center, full_matrices=False)
    assert np.mean(np.abs((points - center) @ vh[-1]) <= .002) == .60
    plane, ratio, residual, count = _fit_plane(points, Config())
    assert count == 1800 and ratio == .81
    assert residual < .002
    np.testing.assert_allclose(np.abs(plane[:3]), [0., 0., 1.], atol=1e-12)
    np.testing.assert_allclose(plane[3], 0., atol=1e-12)
    assert np.mean(np.abs(points @ plane[:3] + plane[3]) <= .002) == ratio


@pytest.mark.parametrize('translation', [-.4, .4])
def test_retained_plane_is_finite_normalized_and_rigid_frame_consistent(translation):
    points = asymmetric_measured_cloud()
    angle = .71
    rotation = np.array([[np.cos(angle), 0., np.sin(angle)],
                         [0., 1., 0.],
                         [-np.sin(angle), 0., np.cos(angle)]])
    shift = np.array([.12, -.06, translation])
    measured = points @ rotation.T + shift
    plane, ratio, residual, _ = _fit_plane(measured, Config())
    assert np.all(np.isfinite(plane)) and plane[3] >= 0.
    assert np.linalg.norm(plane[:3]) == pytest.approx(1., abs=1e-12)
    expected = np.r_[rotation[:, 2], -rotation[:, 2] @ shift]
    if expected[3] < 0.:
        expected = -expected
    np.testing.assert_allclose(plane, expected, atol=1e-12)
    assert ratio == .81 and residual < .002


def test_clean_support_still_uses_measured_refinement_and_original_sample_budget():
    rng = np.random.RandomState(5)
    points = np.column_stack((rng.uniform(-.15, .15, (6000, 2)),
                              rng.normal(.30, .0001, 6000)))
    plane, ratio, residual, count = _fit_plane(points, Config())
    assert count == 1800 and ratio == 1.
    assert residual < .00012
    assert plane[3] == pytest.approx(.30, abs=.00002)
    assert np.dot(plane[:3], [0., 0., -1.]) > .999999


def test_original_ratio_and_minimum_point_gates_are_not_relaxed():
    rng = np.random.RandomState(8)
    points = rng.uniform(-.1, .1, (1800, 3))
    with pytest.raises(ValueError, match='support_plane_not_found|support_plane_inlier_ratio_low'):
        _fit_plane(points, Config())
    with pytest.raises(ValueError, match='insufficient_support_depth'):
        _fit_plane(np.zeros((119, 3)), Config())


@pytest.mark.parametrize('invalid', ['nan', 'infinite', 'wrong_shape', 'collinear'])
def test_invalid_or_degenerate_measurements_never_create_a_support_plane(invalid):
    points = asymmetric_measured_cloud()
    if invalid == 'nan':
        points[5, 1] = np.nan
    elif invalid == 'infinite':
        points[5, 1] = np.inf
    elif invalid == 'wrong_shape':
        points = points[:, :2]
    else:
        points = np.column_stack((np.linspace(0., 1., 1800), np.zeros((1800, 2))))
    with pytest.raises(ValueError, match='invalid_support_plane_points|support_plane_not_found'):
        _fit_plane(points, Config())
