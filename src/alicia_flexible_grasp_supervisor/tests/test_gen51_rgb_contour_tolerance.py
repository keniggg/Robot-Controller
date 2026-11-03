"""Four-pixel contour policy with unchanged numerical and raw-source proofs.

These are offline synthetic-scene and evidence-contract checks. A numerical
correspondence is deliberately not treated as authentic source-image evidence,
and no test grants a grasp, collision, path or robot-execution approval.
"""
from copy import deepcopy

import cv2
import numpy as np
import pytest

from test_gen31_outer_rgb_box import visual_edge_gap_scene, qualify_visual_scene
from test_gen38_cuboid_rgb_interval import scene
from alicia_flexible_grasp.vision import cuboid_rgb_interval as cuboid
from alicia_flexible_grasp.vision import outer_rgb_rectangle as outer


@pytest.fixture(scope='module')
def outer_model():
    result = qualify_visual_scene(visual_edge_gap_scene())
    assert result.ok, (result.code, result.audit)
    assert result.model.schema_version == 2
    return result.model


@pytest.fixture(scope='module')
def complete_cuboid():
    args = scene()
    before = [np.array(args[i], copy=True) for i in (0, 1, 2)]
    result = cuboid.infer_cuboid_rgb_interval(*args)
    assert result['ok'], result
    for old, actual in zip(before, args[:3]):
        np.testing.assert_array_equal(actual, old)
    cuboid.validate_cuboid_rgb_interval_source(result['audit'], *args[:6])
    return args, result


@pytest.mark.parametrize('distance', (3.23, 4.0), ids=('between_old_and_new', 'exactly_four'))
def test_outer_real_interval_validator_accepts_four_pixel_boundary(outer_model, distance):
    assert outer.RGB_CONTOUR_MAX_DISTANCE_PX == cuboid.RGB_CONTOUR_MAX_DISTANCE_PX == 4.0
    audit = deepcopy(outer_model.qualification['outer_rgb_interval'])
    audit['edge_evidence'][0]['maximum_rgb_distance_px'] = distance
    # This parser checks numerical evidence, not authenticity of an image.
    # Keep every metric position/strip/yaw budget and exterior proof unchanged.
    outer.validate_outer_rgb_interval(audit, outer_model.size_xyz_m[:2],
        outer_model.empirical_edge_uncertainty_m,
        outer_model.empirical_yaw_uncertainty_rad)


def test_outer_real_interval_validator_rejects_more_than_four_pixels(outer_model):
    audit = deepcopy(outer_model.qualification['outer_rgb_interval'])
    audit['edge_evidence'][0]['maximum_rgb_distance_px'] = 4.000001
    with pytest.raises(ValueError, match='outside limits'):
        outer.validate_outer_rgb_interval(audit, outer_model.size_xyz_m[:2],
            outer_model.empirical_edge_uncertainty_m,
            outer_model.empirical_yaw_uncertainty_rad)


def _validate_cuboid(audit, result):
    size = np.r_[result['upper']-result['lower'], result['nominal_height_m']]
    return cuboid.validate_cuboid_rgb_interval(audit, size,
        result['edge_uncertainty'], result['yaw_uncertainty'],
        result['height_uncertainty'])


def _different_correspondence(args, result, lower_distance, upper_distance):
    """Change one integer pixel, recomputing its actual geometric distance.

    No production gate or renderer is patched. The new pixel is explicitly
    absent from the original Canny raster, so only the numerical validator may
    accept it; the independent raw-source validator must still reject it.
    """
    audit = deepcopy(result['audit'])
    model = cuboid._model(np.asarray(audit['nominal_axes_base']),
        np.asarray(audit['nominal_lower']), np.asarray(audit['nominal_upper']),
        audit['nominal_height_m'], audit['support_offset_m'], args[4], args[5])
    edge = model['edges'][0]
    start, end = np.asarray(edge['start_pixel']), np.asarray(edge['end_pixel'])
    samples = start+np.outer(np.linspace(0., 1., 101), end-start)
    canny = cv2.Canny(cv2.cvtColor(args[0], cv2.COLOR_BGR2GRAY), 50., 100.)
    anchor = np.floor(samples[0]).astype(int)
    choices = []
    for dx in range(-6, 7):
        for dy in range(-6, 7):
            pixel = anchor+np.array([dx, dy])
            if not (0 <= pixel[0] < canny.shape[1] and 0 <= pixel[1] < canny.shape[0]):
                continue
            distance = float(np.linalg.norm(samples[0]-pixel))
            if lower_distance <= distance <= upper_distance and not canny[pixel[1], pixel[0]]:
                choices.append((distance, int(pixel[0]), int(pixel[1])))
    assert choices, 'fixture must provide a genuine integer-pixel boundary witness'
    distance, x, y = min(choices)
    pixels = np.asarray(audit['silhouette_raster_evidence'][0]['corresponding_canny_pixels'])
    pixels[0] = [x, y]
    audit['silhouette_raster_evidence'][0]['corresponding_canny_pixels'] = pixels.tolist()
    actual = np.linalg.norm(samples-pixels, axis=1)
    audit['full_silhouette_rgb_distances_px'][0] = float(actual.max())
    audit['corner_rgb_distances_px'][0] = float(actual[0])
    assert actual[0] == pytest.approx(distance, abs=1e-12)
    return audit


def test_cuboid_numeric_and_raw_source_proofs_remain_independent(complete_cuboid):
    args, result = complete_cuboid
    audit = _different_correspondence(args, result, 3.0, 4.0)
    assert 3.0 <= audit['corner_rgb_distances_px'][0] <= 4.0
    _validate_cuboid(audit, result)
    with pytest.raises(ValueError, match='not an actual source Canny pixel'):
        cuboid.validate_cuboid_rgb_interval_source(audit, *args[:6])


def test_cuboid_real_validator_rejects_consistent_distance_above_four(complete_cuboid):
    args, result = complete_cuboid
    audit = _different_correspondence(args, result, 4.1, 4.9)
    assert audit['corner_rgb_distances_px'][0] > 4.0
    with pytest.raises(ValueError, match='qualification budget violated'):
        _validate_cuboid(audit, result)


@pytest.mark.parametrize('source_index', (0, 1, 2), ids=('rgb', 'mask', 'depth'))
def test_four_pixel_policy_never_rebinds_changed_original_source(complete_cuboid, source_index):
    args, result = complete_cuboid
    changed = list(args[:6])
    changed[source_index] = np.array(changed[source_index], copy=True)
    changed[source_index].flat[0] ^= 1
    with pytest.raises(ValueError, match='pixel source'):
        cuboid.validate_cuboid_rgb_interval_source(result['audit'], *changed)
