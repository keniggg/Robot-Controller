"""Top evidence is independent of side segmentation; raw geometry stays binding.

The source scene uses an independent slab ray renderer. Positive cases preserve
its real RGB silhouette and depth, and remove only side mask membership. No
fitted model, Canny response, uncertainty or downstream permission is mocked.
"""
from copy import deepcopy
import hashlib
import math

import cv2
import numpy as np
import pytest

from alicia_flexible_grasp.vision.cuboid_rgb_interval import (
    infer_cuboid_rgb_interval, validate_cuboid_rgb_interval,
    validate_cuboid_rgb_interval_source)
from test_gen38_cuboid_rgb_interval import scene


POLICY = 'measured_top_mask_with_complete_cuboid_rgb_v1'
NEW_FIELDS = (
    'silhouette_mask_intersection_pixels', 'silhouette_mask_eroded_polygon_pixels',
    'top_mask_intersection_pixels', 'top_mask_eroded_polygon_pixels',
    'top_mask_coverage',
)


def _physical_faces(args):
    """Independent first-hit slab classification, using the renderer's box."""
    T, K = args[4], args[5]
    yy, xx = np.mgrid[:args[1].shape[0], :args[1].shape[1]]
    optical = np.stack(((xx-K[2])/K[0], (yy-K[3])/K[1], np.ones_like(xx)), axis=-1)
    rays = optical @ T[:3, :3].T
    lower, upper = np.array([-.029, -.020, 0.]), np.array([.029, .020, .020])
    with np.errstate(divide='ignore', invalid='ignore'):
        near = np.minimum((lower-T[:3, 3])/rays, (upper-T[:3, 3])/rays)
    full = args[1] > 0
    top = full & (np.argmax(near, axis=2) == 2)
    return top, full & ~top, rays


def _top_only_args(side_depth_missing=False):
    args = list(scene())
    top, side, _rays = _physical_faces(args)
    assert np.any(top) and np.any(side)
    args[1] = top.astype(np.uint8)*255
    if side_depth_missing:
        args[2][side] = 0
    return args


def _numeric(result, audit=None):
    audit = result['audit'] if audit is None else audit
    size = np.r_[result['upper']-result['lower'], result['nominal_height_m']]
    return validate_cuboid_rgb_interval(audit, size, result['edge_uncertainty'],
        result['yaw_uncertainty'], result['height_uncertainty'])


def _source(args, audit):
    return validate_cuboid_rgb_interval_source(audit, *args[:6])


def _legacy(audit):
    value = deepcopy(audit)
    for key in ('coverage_policy', *NEW_FIELDS):
        value.pop(key)
    return value


@pytest.fixture(scope='module')
def complete_case():
    args = scene()
    result = infer_cuboid_rgb_interval(*args)
    assert result['ok'], result
    return args, result


@pytest.fixture(scope='module')
def top_only_case():
    args = _top_only_args()
    result = infer_cuboid_rgb_interval(*args)
    assert result['ok'], result
    return args, result


@pytest.mark.parametrize('side_depth_missing', [False, True])
def test_real_top_with_missing_side_mask_retains_all_rgb_and_numeric_checks(side_depth_missing):
    args = _top_only_args(side_depth_missing)
    before = [value.tobytes() for value in args[:3]]
    result = infer_cuboid_rgb_interval(*args)
    assert result['ok'], result
    audit = result['audit']
    assert audit['coverage_policy'] == POLICY
    assert audit['top_mask_coverage'] >= .88
    assert audit['silhouette_mask_coverage'] < .88
    assert max(audit['full_silhouette_rgb_distances_px']) <= math.sqrt(8.)
    assert max(audit['corner_rgb_distances_px']) <= math.sqrt(8.)
    assert audit['complete_raster_responses_used'] is True
    assert audit['unexplained_unknown_mask_pixel_count'] == 0
    assert audit['certified_error_bound'] is False
    _numeric(result)
    _source(args, audit)
    assert before == [value.tobytes() for value in args[:3]]


def test_missing_top_mask_stays_rejected_even_with_unchanged_real_rgb_and_depth():
    args = list(scene())
    top, _side, _rays = _physical_faces(args)
    yy, xx = np.where(top)
    hole = ((np.abs(np.indices(top.shape)[1]-(xx.min()+xx.max())/2.) < .3*np.ptp(xx)) &
            (np.abs(np.indices(top.shape)[0]-(yy.min()+yy.max())/2.) < .3*np.ptp(yy)))
    args[1][hole & top] = 0
    before = [value.tobytes() for value in (args[0], args[2])]
    result = infer_cuboid_rgb_interval(*args)
    assert not result['ok']
    assert result['audit'].get('top_mask_coverage', 0.) < .88
    assert before == [value.tobytes() for value in (args[0], args[2])]


@pytest.mark.parametrize('fault', ['top_continuation', 'external_occluder', 'missing_outer_rgb', 'internal_print'])
def test_top_mask_policy_does_not_bypass_real_outer_or_depth_contradictions(fault):
    args = _top_only_args()
    original = list(scene())
    _top, _side, rays = _physical_faces(original)
    if fault in ('top_continuation', 'external_occluder'):
        plane_height = .020 if fault == 'top_continuation' else .050
        optical_depth = (plane_height-args[4][2, 3])/rays[:, :, 2]
        outside = original[1] == 0
        args[2][outside] = np.rint(optical_depth[outside]/args[3]).astype(np.uint16)
    else:
        args[0][:] = 25
        if fault == 'internal_print':
            cv2.rectangle(args[0], (175, 165), (225, 195), (240, 240, 240), 1)
    before = [value.tobytes() for value in args[:3]]
    result = infer_cuboid_rgb_interval(*args)
    assert not result['ok'], result
    assert before == [value.tobytes() for value in args[:3]]


@pytest.mark.parametrize('field', NEW_FIELDS + ('silhouette_mask_coverage',))
def test_changed_counts_or_ratios_fail_numeric_validation(complete_case, field):
    _args, result = complete_case
    audit = deepcopy(result['audit'])
    audit[field] = .5 if field.endswith('coverage') else audit[field]+1
    with pytest.raises(ValueError):
        _numeric(result, audit)


def test_self_consistent_forged_numerator_cannot_pass_real_source_validation(complete_case):
    args, result = complete_case
    audit = deepcopy(result['audit'])
    audit['top_mask_intersection_pixels'] -= 1
    audit['top_mask_coverage'] = (audit['top_mask_intersection_pixels']/
                                  audit['top_mask_eroded_polygon_pixels'])
    # JSON alone cannot authenticate mask intersection pixels; the exact source can.
    _numeric(result, audit)
    with pytest.raises(ValueError, match='coverage'):
        _source(args, audit)


def test_rehashed_changed_source_mask_cannot_reuse_old_intersection_counts(complete_case):
    args, result = complete_case
    changed = list(args)
    changed[1] = args[1].copy()
    top, _side, _rays = _physical_faces(args)
    yy, xx = np.where(top)
    index = int(np.argmin((xx-xx.mean())**2+(yy-yy.mean())**2))
    changed[1][yy[index], xx[index]] = 0
    audit = deepcopy(result['audit'])
    audit['source_mask_sha256'] = hashlib.sha256(changed[1].tobytes()).hexdigest()
    _numeric(result, audit)
    with pytest.raises(ValueError, match='coverage'):
        _source(changed, audit)


@pytest.mark.parametrize('policy', [None, '', 'arbitrary_policy'])
def test_unknown_or_null_policy_cannot_fall_back_to_legacy(complete_case, policy):
    _args, result = complete_case
    audit = deepcopy(result['audit'])
    audit['coverage_policy'] = policy
    with pytest.raises(ValueError, match='policy'):
        _numeric(result, audit)


@pytest.mark.parametrize('field', NEW_FIELDS)
def test_new_policy_missing_any_count_or_top_ratio_is_rejected(complete_case, field):
    _args, result = complete_case
    audit = deepcopy(result['audit'])
    audit.pop(field)
    with pytest.raises(ValueError, match='incomplete'):
        _numeric(result, audit)


def test_legacy_full_mask_audit_keeps_original_whole_hull_rule(complete_case):
    args, result = complete_case
    audit = _legacy(result['audit'])
    assert audit['silhouette_mask_coverage'] >= .88
    _numeric(result, audit)
    _source(args, audit)


def test_legacy_cannot_relabel_low_whole_coverage_as_top_policy(top_only_case):
    _args, result = top_only_case
    audit = _legacy(result['audit'])
    assert audit['silhouette_mask_coverage'] < .88
    with pytest.raises(ValueError, match='original qualification'):
        _numeric(result, audit)


@pytest.mark.parametrize('field', NEW_FIELDS)
def test_partial_new_audit_without_policy_is_not_legacy(complete_case, field):
    _args, result = complete_case
    audit = _legacy(result['audit'])
    audit[field] = result['audit'][field]
    with pytest.raises(ValueError, match='policy'):
        _numeric(result, audit)


@pytest.mark.parametrize('coverage', [float('nan'), float('inf'), -float('inf'), -.01, 1.01, True, '0.99'])
def test_legacy_nonfinite_or_non_numeric_coverage_cannot_pass(complete_case, coverage):
    _args, result = complete_case
    audit = _legacy(result['audit'])
    audit['silhouette_mask_coverage'] = coverage
    with pytest.raises(ValueError):
        _numeric(result, audit)
