"""The unknown-only box representation must remain a measured enclosure."""
from dataclasses import dataclass
import hashlib

import numpy as np
import pytest

from alicia_grasp_modes.measured_enclosure import minimum_area_measured_enclosure


@dataclass(frozen=True)
class Geometry:
    ok: bool
    center_base: np.ndarray
    axes_base: np.ndarray
    size_xyz_m: np.ndarray
    support_normal_base: np.ndarray
    support_offset_m: float
    object_points_base: np.ndarray
    source_mode: str = 'instance_mask'


def geometry(points, normal=(0., 0., 1.), offset=0.):
    return Geometry(True, np.zeros(3), np.eye(3), np.ones(3) * .05,
                    np.asarray(normal, dtype=float), offset, np.asarray(points, dtype=float))


def cloud(width=.04, length=.06, height=.02, yaw=.37):
    xy = np.asarray([(x, y) for x in (-length / 2, length / 2)
                     for y in (-width / 2, width / 2)])
    rotation = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    xy = xy @ rotation.T
    return np.asarray([(x, y, z) for x, y in xy for z in (0., height)])


def assert_encloses(result):
    local = (result.object_points_base - result.center_base) @ result.axes_base
    assert np.all(np.abs(local) <= result.size_xyz_m / 2 + 1e-12)
    np.testing.assert_allclose(result.axes_base.T @ result.axes_base, np.eye(3), atol=1e-12)
    assert np.linalg.det(result.axes_base) == pytest.approx(1.)


def test_full_point_rectangle_recovers_rotated_short_edge_and_keeps_raw_points():
    source = geometry(cloud())
    before = source.object_points_base.copy()
    result, audit = minimum_area_measured_enclosure(source)
    np.testing.assert_allclose(result.size_xyz_m, [.06, .04, .02], atol=1e-12)
    assert_encloses(result)
    for value in (result.center_base, result.axes_base, result.size_xyz_m):
        assert not value.flags.writeable
    assert result.object_points_base is source.object_points_base
    np.testing.assert_array_equal(source.object_points_base, before)
    assert audit['all_measured_points_enclosed']
    assert audit['points_removed'] == 0
    assert audit['point_count'] == len(before)
    assert audit['object_points_sha256'] == hashlib.sha256(
        np.ascontiguousarray(before, dtype='<f8').tobytes()).hexdigest()
    assert source.size_xyz_m.tolist() == [.05] * 3


def test_extreme_measured_point_is_never_trimmed():
    points = np.vstack((np.repeat(cloud(), 50, axis=0), [0.065, 0.012, .029]))
    result, audit = minimum_area_measured_enclosure(geometry(points))
    assert_encloses(result)
    assert audit['point_count'] == 401
    assert result.size_xyz_m[2] == pytest.approx(.029)
    assert max(result.size_xyz_m[:2]) > .065


@pytest.mark.parametrize('width,length,yaw', [(.04, .06, .37), (.04, .04, 0.),
                                             (.04, .04, np.pi / 4)])
def test_rectangle_symmetries_and_duplicate_permutations_are_deterministic(width, length, yaw):
    points = np.repeat(cloud(width, length, yaw=yaw), 4, axis=0)
    expected, _ = minimum_area_measured_enclosure(geometry(points))
    rng = np.random.RandomState(18)
    for _ in range(12):
        actual, _ = minimum_area_measured_enclosure(geometry(points[rng.permutation(len(points))]))
        np.testing.assert_array_equal(actual.axes_base, expected.axes_base)
        np.testing.assert_array_equal(actual.size_xyz_m, expected.size_xyz_m)
        np.testing.assert_array_equal(actual.center_base, expected.center_base)


def test_tilted_support_plane_and_negative_measured_bottom_remain_enclosed():
    angle = .24
    rotation = np.array([[1., 0., 0.], [0., np.cos(angle), -np.sin(angle)],
                         [0., np.sin(angle), np.cos(angle)]])
    normal = rotation[:, 2]
    offset = -.23
    local = cloud()
    local[:4, 2] -= .001
    points = local @ rotation.T - offset * normal
    source = geometry(points, normal, offset)
    result, audit = minimum_area_measured_enclosure(source)
    assert_encloses(result)
    np.testing.assert_array_equal(result.support_normal_base, normal)
    assert result.support_offset_m == offset
    assert audit['support_height_bounds_m'] == pytest.approx([-.001, .020])
    assert result.size_xyz_m[2] == pytest.approx(.021)


def test_top_face_keeps_original_support_resting_geometry_semantics():
    points = cloud()[1::2]
    result, audit = minimum_area_measured_enclosure(geometry(points))
    assert_encloses(result)
    assert audit['support_height_bounds_m'] == pytest.approx([0., .02])
    assert result.center_base[2] == pytest.approx(.01)
    # This helper supplies no contact authorization, registration or fabricated points.
    assert len(result.object_points_base) == 4


@pytest.mark.parametrize('points', [np.zeros((4, 3)),
    np.array([[0., 0., .01], [.01, 0., .02], [.02, 0., .03]]),
    np.array([[0., 0., 0.], [.01, 0., 0.], [0., .01, 0.]]),
    np.array([[0., 0., .01], [.01, 0., .01], [0., np.nan, .01]])])
def test_degenerate_plane_hull_zero_physical_height_and_nonfinite_points_rejected(points):
    with pytest.raises(ValueError):
        minimum_area_measured_enclosure(geometry(points))


@pytest.mark.parametrize('kwargs', [{'max_size_m': .03}, {'min_size_m': .03},
                                   {'max_height_m': .01}, {'min_size_m': -1.},
                                   {'max_size_m': float('nan')}])
def test_existing_size_limits_apply_to_full_enclosure(kwargs):
    with pytest.raises(ValueError):
        minimum_area_measured_enclosure(geometry(cloud()), **kwargs)


def test_invalid_support_normal_is_rejected_without_silent_normalization():
    with pytest.raises(ValueError, match='unit length'):
        minimum_area_measured_enclosure(geometry(cloud(), normal=(0., 0., 2.)))
