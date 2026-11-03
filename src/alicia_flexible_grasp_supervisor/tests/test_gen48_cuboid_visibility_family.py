"""Public qualification of independent ray-box scenes near a face transition.

RGB, mask and depth come from analytic slab intersections, never the
production silhouette projector. The new family remains an empirical model
enclosure; these tests provide no robot/path/physical-contact evidence.
"""
from copy import deepcopy
import hashlib
import math

import cv2
import numpy as np
import pytest

from alicia_flexible_grasp.vision.cuboid_rgb_interval import (
    infer_cuboid_rgb_interval,
    validate_cuboid_rgb_interval,
    validate_cuboid_rgb_interval_source,
)


FAMILY_POLICY = 'complete_rgb_nominal_anchor_and_all_plane_assignment_enclosures_v1'


def ray_box_scene(camera_x):
    """Render a known rectangular solid and its support in optical depth."""
    size = np.array([.058, .040, .020])
    center = np.array([0., 0., size[2]/2.])
    T = np.eye(4)
    T[:3, 3] = [float(camera_x), -.10, .27]
    forward = np.array([0., 0., .020])-T[:3, 3]
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, [0., 0., 1.])
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)
    T[:3, :3] = np.column_stack((right, down, forward))
    K = np.array([500., 500., 200., 180.])
    yy, xx = np.mgrid[:360, :400]
    optical = np.column_stack(((xx.ravel()-K[2])/K[0],
        (yy.ravel()-K[3])/K[1], np.ones(xx.size)))
    rays = optical @ T[:3, :3].T
    lower, upper = center-size/2., center+size/2.
    with np.errstate(divide='ignore', invalid='ignore'):
        times = np.stack(((lower-T[:3, 3])/rays, (upper-T[:3, 3])/rays))
    entry = np.max(np.min(times, axis=0), axis=1)
    exit = np.min(np.max(times, axis=0), axis=1)
    support = -T[2, 3]/rays[:, 2]
    hit = np.isfinite(entry) & (exit >= entry) & (entry > 0.) & (entry <= support)
    scale = .0001
    distance = np.where(hit, entry, support)
    assert np.all(np.isfinite(distance)) and np.all(distance > 0.)
    depth = np.rint(distance/scale).astype(np.uint16).reshape(xx.shape)
    mask = hit.reshape(xx.shape).astype(np.uint8)*255
    color = np.full(xx.shape+(3,), 25, np.uint8)
    color[mask > 0] = 240
    measured = T[:3, 3]+rays*(depth.ravel()*scale)[:, None]
    top = hit & (np.abs(measured[:, 2]-.020) < .0002)
    assert np.count_nonzero(top) > 1000
    args = (color, mask, depth, scale, T, K, np.array([0., 0., .020]),
        np.array([0., 0., 1.]), np.eye(3), .0006, .0001, 0., .020)
    return args, dict(size=size, center=center,
        independent_top_count=int(np.count_nonzero(top)),
        independent_side_count=int(np.count_nonzero(hit & ~top)))


def validate_json(audit):
    size = np.r_[np.asarray(audit['nominal_upper'])-audit['nominal_lower'],
        audit['nominal_height_m']]
    return validate_cuboid_rgb_interval(audit, size,
        audit['empirical_edge_uncertainty_m'], audit['empirical_yaw_uncertainty_rad'],
        audit['empirical_height_uncertainty_m'])


def validate_source(audit, args):
    return validate_cuboid_rgb_interval_source(audit, *args[:6])


@pytest.fixture(scope='module', params=(-1., 1.), ids=('near_lower_x', 'near_upper_x'))
def transition(request):
    # The camera is physically inside the true x span by only0.10mm.
    # Raster/height/yaw uncertainty still admits the adjacent side assignment.
    args, truth = ray_box_scene(request.param*(.058/2.-.0001))
    result = infer_cuboid_rgb_interval(*args)
    assert result['ok'], result
    return args, truth, result


@pytest.fixture(scope='module')
def lower_transition():
    args, truth = ray_box_scene(-(.058/2.-.0001))
    result = infer_cuboid_rgb_interval(*args)
    assert result['ok'], result
    return args, truth, result


def test_real_near_transition_keeps_all_global_interpretations_and_source(transition):
    args, truth, result = transition
    audit = result['audit']
    family = audit['visibility_family']
    assert family['policy'] == FAMILY_POLICY
    assert family['nominal_anchor_membership_claimed'] is False
    assert len(family['branches']) == 9
    assert len({tuple(row['states']) for row in family['branches']}) == 9
    retained = [row for row in family['branches'] if row['retained']]
    assert len(retained) == family['retained_branch_count'] >= 2
    assert len({tuple(row['planes']) for row in retained}) >= 2
    assert len(family['projection_groups']) == 4
    for original, alternatives in zip(audit['groups'], family['projection_groups']):
        assert set(alternatives) == {'top', 'support'}
        for plane in ('top', 'support'):
            value = alternatives[plane]
            assert value['plane'] == plane
            assert len(value['metric_points']) == len(original['line_pixels'])
            assert value['strip_radius_m'] >= value['sampling_bound_m'] > 0.
    assert audit['certified_error_bound'] is False
    assert audit['raster_response_policy'] == 'complete_seed_and_final_fixed_strip_union'
    assert max(audit['full_silhouette_rgb_distances_px']) <= math.sqrt(8.)
    assert max(audit['corner_rgb_distances_px']) <= math.sqrt(8.)
    assert audit['top_mask_coverage'] >= .88
    assert audit['complete_raster_responses_used'] is True
    assert audit['source_color_sha256'] == hashlib.sha256(args[0].tobytes()).hexdigest()
    assert audit['source_mask_sha256'] == hashlib.sha256(args[1].tobytes()).hexdigest()
    assert audit['source_depth_sha256'] == hashlib.sha256(args[2].tobytes()).hexdigest()
    validate_json(audit)
    validate_source(audit, args)
    # Independent true footprint must remain inside the reported enclosure.
    A = np.asarray(audit['nominal_axes_base'])
    lo, hi = np.asarray(audit['nominal_lower']), np.asarray(audit['nominal_upper'])
    size, mid = hi-lo, (hi+lo)/2.
    yaw = audit['empirical_yaw_uncertainty_rad']
    axis = np.asarray(audit['empirical_axis_uncertainty_m'])
    half = []
    for j in (0, 1):
        angle = min(yaw, math.atan2(size[1-j], size[j]))
        half.append((size[j]*math.cos(angle)+size[1-j]*math.sin(angle))/2.+axis[j])
    true_corners = np.array([[x, y, .020] for x in (-.029, .029) for y in (-.020, .020)])
    assert np.all(np.abs(true_corners @ A[:, :2]-mid) <= np.asarray(half)+1e-9)
    assert abs(result['nominal_height_m']-truth['size'][2]) <= result['height_uncertainty']
    assert np.all(axis >= .001) and axis.max() <= .01
    assert math.radians(.25) <= yaw <= math.radians(12.)


def test_existing_stable_scene_retains_original_assignment_policy():
    args, _ = ray_box_scene(0.)
    result = infer_cuboid_rgb_interval(*args)
    assert result['ok'], result
    audit = result['audit']
    assert 'visibility_family' not in audit
    assert audit['silhouette_plane_assignment_stability'] == ['inside', 'outside_negative']
    assert audit['top_support_assignment_policy'] == 'actual_shared_eight_corner_convex_hull_no_ambiguous_edges'
    assert audit['raster_response_policy'] == 'complete_seed_and_final_fixed_strip_union'
    validate_json(audit)
    validate_source(audit, args)


def test_missing_outer_rgb_is_still_rejected_near_transition():
    args, _ = ray_box_scene(-(.058/2.-.0001))
    args[0][:] = 25
    cv2.rectangle(args[0], (175, 165), (225, 195), (240, 240, 240), 1)
    result = infer_cuboid_rgb_interval(*args)
    assert not result['ok']


@pytest.mark.parametrize('fault', ('camera', 'intrinsics', 'color', 'mask_dtype'))
def test_source_or_calibration_change_cannot_reuse_transition_family(lower_transition, fault):
    args, _, result = lower_transition
    altered = list(args[:6])
    if fault == 'camera':
        altered[4] = args[4].copy()
        altered[4][0, 3] += .001
    elif fault == 'intrinsics':
        altered[5] = args[5].copy()
        altered[5][0] += 1.
    elif fault == 'color':
        altered[0] = args[0].copy()
        altered[0][0, 0, 0] += 1
    else:
        altered[1] = args[1] > 0
    with pytest.raises(ValueError):
        validate_cuboid_rgb_interval_source(result['audit'], *altered)


@pytest.mark.parametrize('fault', ('drop_branch', 'count', 'projection', 'anchor_claim'))
def test_tampered_alternative_evidence_is_recomputed(lower_transition, fault):
    args, _, result = lower_transition
    audit = deepcopy(result['audit'])
    family = audit['visibility_family']
    if fault == 'drop_branch':
        index = next(i for i, row in enumerate(family['branches']) if row['retained'])
        del family['branches'][index]
        family['retained_branch_count'] -= 1
    elif fault == 'count':
        family['retained_branch_count'] += 1
    elif fault == 'projection':
        family['projection_groups'][3]['support']['metric_points'][0][0] += .0001
    else:
        family['nominal_anchor_membership_claimed'] = True
    with pytest.raises(ValueError):
        validate_json(audit)
    with pytest.raises(ValueError):
        validate_source(audit, args)


def test_deleting_one_original_raster_response_cannot_qualify_a_smaller_family(lower_transition):
    args, _, result = lower_transition
    audit = deepcopy(result['audit'])
    group = audit['groups'][3]
    del group['line_pixels'][0]
    del group['metric_points'][0]
    for plane in ('top', 'support'):
        del audit['visibility_family']['projection_groups'][3][plane]['metric_points'][0]
    with pytest.raises(ValueError):
        validate_source(audit, args)


@pytest.mark.parametrize('budget', ('axis', 'yaw', 'height'))
def test_family_cannot_publish_a_smaller_consistently_labelled_budget(lower_transition, budget):
    args, _, result = lower_transition
    audit = deepcopy(result['audit'])
    family = audit['visibility_family']
    if budget == 'axis':
        audit['empirical_axis_uncertainty_m'] = [v*.9 for v in audit['empirical_axis_uncertainty_m']]
        audit['empirical_edge_uncertainty_m'] = max(audit['empirical_axis_uncertainty_m'])
        family['axis_uncertainty'] = list(audit['empirical_axis_uncertainty_m'])
    elif budget == 'yaw':
        audit['empirical_yaw_uncertainty_rad'] *= .9
        family['yaw_uncertainty'] = audit['empirical_yaw_uncertainty_rad']
    else:
        audit['empirical_height_uncertainty_m'] *= .5
        h, hu = audit['nominal_height_m'], audit['empirical_height_uncertainty_m']
        audit['common_height_interval_m'] = [h-hu, h+hu]
    with pytest.raises(ValueError):
        validate_json(audit)
    with pytest.raises(ValueError):
        validate_source(audit, args)
