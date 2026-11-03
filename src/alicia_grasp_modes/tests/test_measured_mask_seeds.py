"""Small measured surfaces need interior seeds, not only a noisy high tail."""
import cv2
import numpy as np
import pytest

from alicia_grasp_modes.measured_mask_seeds import confidence_labels, local_surface_normals
from alicia_grasp_modes.tabletop import Config


def surface(*, width=12, height=12, elevation=.0048, noise=.00015, slope=0.):
    """A measured surface and surrounding support in camera-link coordinates."""
    shape = (height + 16, width + 16)
    yy, xx = np.indices(shape)
    seed = np.zeros(shape, bool)
    seed[8:8+height, 8:8+width] = True
    z = np.zeros(shape)
    rng = np.random.RandomState(112)
    z[seed] = (elevation + slope * (xx[seed]-8) * .001
               + rng.uniform(-noise, noise, int(seed.sum())))
    depth = .3-z
    points = np.stack((depth, xx*.001, yy*.001), axis=-1)
    return seed, points, depth, z, np.array([-1., 0., 0.]), Config()


def labels(data, **kwargs):
    return confidence_labels(*data, **kwargs)


@pytest.mark.parametrize('colour', [(5, 5, 5), (240, 240, 240), (15, 180, 40), (190, 50, 20)])
@pytest.mark.parametrize('shape', [(10, 10), (12, 12), (16, 10)])
@pytest.mark.parametrize('slope', [0., .025])
def test_small_noisy_planar_interior_is_recovered_without_colour_or_depth_changes(colour, shape, slope):
    data = surface(width=shape[0], height=shape[1], slope=slope)
    before = [v.copy() if isinstance(v, np.ndarray) else v for v in data]
    bgr = np.full(data[0].shape+(3,), colour, np.uint8)
    fg, bg, evidence = labels(data, bgr=bgr)
    assert evidence['upper_surface_band_used']
    assert evidence['original_top_pixels'] < 25 <= np.count_nonzero(fg)
    assert np.all(data[0][fg]) and np.all(data[3][fg] >= data[-1].foreground_min_m)
    assert np.count_nonzero(fg) < np.count_nonzero(data[0])
    assert not np.any(fg & bg)
    _, reliable = local_surface_normals(data[1], data[2], np.ones(data[0].shape, bool))
    assert np.all(reliable[fg])
    for old, new in zip(before, data):
        if isinstance(new, np.ndarray):
            np.testing.assert_array_equal(new, old)


def test_original_required_measured_point_count_is_not_lowered_for_small_top():
    data = surface()
    data[-1].min_object_points = 150
    with pytest.raises(ValueError, match='insufficient_measured_foreground'):
        labels(data)


def test_top_band_is_not_a_permission_to_cross_four_mm_foreground_threshold():
    data = surface(elevation=.00405, noise=.0002)
    try:
        fg, _, _ = labels(data)
    except ValueError as exc:
        assert str(exc) in ('rgb_seed_has_insufficient_measured_foreground', 'rgb_seed_core_too_small')
    else:
        assert np.all(data[3][fg] >= .004)


def test_shallow_measured_ramp_without_sufficient_upper_interior_is_not_expanded():
    data = surface(elevation=.0042, noise=0., slope=.6)
    with pytest.raises(ValueError, match='rgb_seed_core_too_small'):
        labels(data)


def test_reliable_steep_face_keeps_existing_side_policy():
    data = surface(elevation=.0042, noise=0., slope=1.2)
    fg, _, evidence = labels(data)
    assert np.count_nonzero(fg) >= 25
    assert evidence['trusted_side_pixels'] >= 25
    assert not evidence['upper_surface_band_used']


@pytest.mark.parametrize('colour', [(0, 0, 0), (255, 255, 255), (0, 0, 255)])
def test_table_print_or_shadow_without_measured_foreground_cannot_seed(colour):
    data = surface(elevation=0., noise=.0002)
    bgr = np.full(data[0].shape+(3,), (90, 90, 90), np.uint8)
    bgr[data[0]] = colour
    with pytest.raises(ValueError, match='insufficient_measured_foreground'):
        labels(data, bgr=bgr)


def test_absent_reliable_normal_does_not_promote_the_entire_component():
    data = list(surface())
    # Collinear point coordinates cannot establish a 2-D local surface normal.
    data[1][..., 1:] = 0.
    with pytest.raises(ValueError, match='rgb_seed_core_too_small'):
        labels(data)


@pytest.mark.parametrize('corruption', ['rough', 'holes', 'depth_discontinuity'])
def test_unreliable_depth_does_not_gain_hard_foreground_seeds(corruption):
    data = list(surface(noise=.00015))
    seed, points, depth, heights = data[:4]
    yy, xx = np.indices(seed.shape)
    if corruption == 'rough':
        heights[seed] += np.random.RandomState(2).uniform(-.0008, .0008, int(seed.sum()))
        depth[:] = .3-heights
        points[..., 0] = depth
    elif corruption == 'holes':
        invalid = seed & ((xx+yy) % 3 == 0)
        depth[invalid] = 0.
        points[invalid] = np.nan
    else:
        depth[seed & (xx % 2 == 0)] -= .012
        points[..., 0] = depth
        heights[:] = .3-depth
    valid = (np.isfinite(depth) & (depth >= .03) & (depth <= 2.)
             & np.all(np.isfinite(points), axis=-1))
    _, reliable = local_surface_normals(points, depth, valid)
    assert np.count_nonzero(seed & reliable) < 25
    with pytest.raises(ValueError, match='rgb_seed_core_too_small|insufficient_measured_foreground'):
        labels(data)


def test_existing_large_top_labels_are_bitwise_unchanged():
    data = surface(width=40, height=40, elevation=.015)
    seed, points, depth, heights, normal, config = data
    core = cv2.erode(seed.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
    expected = core & (heights >= np.percentile(heights[seed], 90.))
    fg, _, evidence = labels(data)
    assert np.count_nonzero(expected) >= 25
    np.testing.assert_array_equal(fg, expected)
    assert not evidence['upper_surface_band_used']
