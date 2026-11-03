"""Independent synthetic pinhole/box-ray checks for mixed-plane intervals."""
from copy import deepcopy
import math

import cv2
import numpy as np
import pytest

from alicia_flexible_grasp.vision.cuboid_rgb_interval import (
    infer_cuboid_rgb_interval, validate_cuboid_rgb_interval,
    validate_cuboid_rgb_interval_source)
from alicia_flexible_grasp.vision.cuboid_silhouette import project_cuboid_silhouette


def scene():
    size = np.array([.058, .040, .020]); center = np.array([0., 0., .010])
    T = np.eye(4); T[:3, 3] = [0., -.10, .27]
    z = (np.array([0., 0., .020])-T[:3, 3]); z /= np.linalg.norm(z)
    x = np.array([1., 0., 0.]); y = np.cross(z, x)
    T[:3, :3] = np.column_stack((x, y, z)); K = np.array([500., 500., 200., 180.])
    yy, xx = np.mgrid[:360, :400]
    optical = np.column_stack(((xx.ravel()-K[2])/K[0], (yy.ravel()-K[3])/K[1], np.ones(xx.size)))
    rays = optical @ T[:3, :3].T
    # Independent slab ray-box intersection, not the production projector.
    lower, upper = center-size/2., center+size/2.
    with np.errstate(divide='ignore', invalid='ignore'):
        times = np.stack(((lower-T[:3, 3])/rays, (upper-T[:3, 3])/rays))
    entry = np.max(np.min(times, axis=0), axis=1); exit = np.min(np.max(times, axis=0), axis=1)
    hit = (exit >= entry) & (entry > 0.)
    support = -T[2, 3]/rays[:, 2]; zdepth = np.where(hit, entry, support)
    scale = .0001; depth = np.rint(zdepth/scale).astype(np.uint16).reshape(xx.shape)
    mask = hit.reshape(xx.shape).astype(np.uint8)*255
    color = np.full(xx.shape+(3,), 25, np.uint8); color[mask > 0] = 240
    return color, mask, depth, scale, T, K, np.array([0., 0., .020]), np.array([0., 0., 1.]), np.eye(3), .0006, .0001, 0., .020


def test_complete_visible_cuboid_has_mixed_plane_interval():
    args = scene(); result = infer_cuboid_rgb_interval(*args)
    assert result['ok'], result
    audit = result['audit']; size = np.r_[result['upper']-result['lower'], result['nominal_height_m']]
    assert {r['plane'] for r in audit['groups']} == {'top', 'support'}
    assert all(len(r['line_pixels']) >= 12 for r in audit['groups'])
    assert max(audit['full_silhouette_rgb_distances_px']) <= math.sqrt(8.)
    assert audit['silhouette_mask_coverage'] >= .88
    validate_cuboid_rgb_interval(audit, size, result['edge_uncertainty'], result['yaw_uncertainty'], result['height_uncertainty'])


def test_missing_outer_rgb_cannot_be_replaced_by_printed_inner_box():
    args = list(scene()); args[0][:] = 25
    # A smaller internal printed rectangle cannot explain the measured cube.
    cv2.rectangle(args[0], (175, 165), (225, 195), (240, 240, 240), 1)
    result = infer_cuboid_rgb_interval(*args)
    assert not result['ok']


def test_validator_rejects_truncated_height_or_raster_budget():
    result = infer_cuboid_rgb_interval(*scene()); assert result['ok'], result
    size = np.r_[result['upper']-result['lower'], result['nominal_height_m']]
    for mutate in ('height', 'sampling'):
        audit = deepcopy(result['audit'])
        if mutate == 'height':
            audit['empirical_height_uncertainty_m'] *= .5
        else:
            audit['groups'][0]['sampling_bound_m'] = 0.
            audit['groups'][0]['strip_radius_m'] = audit['groups'][0]['maximum_line_residual_m']
        with pytest.raises(ValueError):
            validate_cuboid_rgb_interval(audit, size, result['edge_uncertainty'], result['yaw_uncertainty'], result['height_uncertainty'])


def test_renderer_no_ambiguity_is_required_for_plane_backprojection():
    args = scene(); model = project_cuboid_silhouette([0., 0., .01], np.eye(3), [.058, .040, .020], args[4], args[5])
    assert all(row['kind'] in ('top', 'support', 'vertical') for row in model['edges'])


def test_source_validation_preserves_original_mask_dtype_and_pixel_values():
    args = scene(); result = infer_cuboid_rgb_interval(*args); assert result['ok'], result
    validate_cuboid_rgb_interval_source(result['audit'], args[0], args[1], args[2], args[3], args[4], args[5])
    with pytest.raises(ValueError, match='pixel source'):
        validate_cuboid_rgb_interval_source(result['audit'], args[0], args[1] > 0, args[2], args[3], args[4], args[5])
    changed = args[0].copy(); changed[180, 200, 0] -= 1
    with pytest.raises(ValueError, match='pixel source'):
        validate_cuboid_rgb_interval_source(result['audit'], changed, args[1], args[2], args[3], args[4], args[5])


def test_scalar_compatibility_cannot_hide_changed_directional_family():
    result = infer_cuboid_rgb_interval(*scene()); assert result['ok'], result
    assert result['edge_uncertainty'] == max(result['axis_uncertainty'])
    size = np.r_[result['upper']-result['lower'], result['nominal_height_m']]
    audit = deepcopy(result['audit']); audit['empirical_axis_uncertainty_m'][0] *= .9
    with pytest.raises(ValueError, match='directional'):
        validate_cuboid_rgb_interval(audit, size, result['edge_uncertainty'], result['yaw_uncertainty'], result['height_uncertainty'])
