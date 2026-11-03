"""Independent ray/box scenes for complete cuboid projection diagnostics.

These tests preserve missing depth, actual side pixels, internal printing and
segmentation omissions. They never replace source pixels with inferred depth.
"""
import copy
import hashlib
import json
import math
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from alicia_flexible_grasp.vision.cuboid_silhouette import (
    diagnose_cuboid_silhouette, project_cuboid_silhouette,
    validate_cuboid_silhouette_diagnostic)


def scene():
    width, height = 320, 240
    K = np.array([500., 500., (width-1)/2., (height-1)/2.])
    yaw = math.radians(21.)
    A = np.array([[math.cos(yaw), -math.sin(yaw), 0.],
                  [math.sin(yaw), math.cos(yaw), 0.], [0., 0., 1.]])
    size = np.array([.065, .035, .025])
    center = np.array([0., 0., size[2]/2.])
    camera = A @ np.array([.19, -.16, .32])
    forward = center-camera
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, [0., 0., 1.])
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = np.column_stack((right, down, forward)), camera
    yy, xx = np.mgrid[:height, :width]
    rays = np.stack(((xx-K[2])/K[0], (yy-K[3])/K[1], np.ones_like(xx)), axis=-1) @ T[:3, :3].T
    local_rays, local_camera = rays @ A, camera @ A
    lower, upper = np.r_[-size[:2]/2., 0.], np.r_[size[:2]/2., size[2]]
    with np.errstate(divide='ignore', invalid='ignore'):
        near = np.minimum((lower-local_camera)/local_rays, (upper-local_camera)/local_rays)
        far = np.maximum((lower-local_camera)/local_rays, (upper-local_camera)/local_rays)
    entry, exit_ = near.max(axis=2), far.min(axis=2)
    table = -camera[2]/rays[:, :, 2]
    mask = (entry > 0.) & (entry <= exit_) & (entry < table)
    top = mask & (np.argmax(near, axis=2) == 2)
    side = mask & ~top
    optical_z = np.where(mask, entry, table)
    color = np.full((height, width, 3), 24, np.uint8)
    color[top], color[side] = [70, 140, 220], [35, 75, 120]
    scale = .0001
    depth = np.rint(optical_z/scale).astype(np.uint16)
    return SimpleNamespace(K=K, T=T, center=center, axes=A, size=size,
        color=color, mask=mask.astype(np.uint8)*255, depth=depth, scale=scale,
        top=top, side=side, true_points=camera+rays*optical_z[:, :, None])


def diagnose(s, **kwargs):
    args = dict(color_bgr=s.color, object_mask=s.mask, depth_raw=s.depth,
        intrinsics=s.K, depth_scale_m=s.scale, T_base_optical=s.T,
        center_base=s.center, axes_base=s.axes, size_xyz_m=s.size,
        support_normal_base=np.array([0., 0., 1.]), support_offset_m=0.,
        source_stamp_ns=1792207340733134031,
        target_identity=dict(epoch=4, track_id='g1-t4'),
        plane_slab_m=.001, pixel_footprint_m=.0004)
    args.update(kwargs)
    return diagnose_cuboid_silhouette(**args)


def test_complete_eight_corner_projection_has_real_top_support_vertical_edges():
    s = scene()
    result = project_cuboid_silhouette(s.center, s.axes, s.size, s.T, s.K)
    assert result['vertices_base'].shape == (8, 3)
    assert result['vertices_pixel'].shape == (8, 2)
    assert len(result['hull_vertex_indices']) == 6
    kinds = [edge['kind'] for edge in result['edges']]
    assert {kind:kinds.count(kind) for kind in set(kinds)} == dict(top=2, support=2, vertical=2)
    for edge in result['edges']:
        endpoints = result['vertices_base'][[edge['start_vertex'], edge['end_vertex']]]
        if edge['kind'] == 'top':
            np.testing.assert_allclose(endpoints[:, 2], s.size[2], atol=1e-15)
        elif edge['kind'] == 'support':
            np.testing.assert_allclose(endpoints[:, 2], 0., atol=1e-15)
        else:
            np.testing.assert_allclose(endpoints[0, :2], endpoints[1, :2], atol=1e-15)


def test_exact_scene_supports_complete_current_edges_and_unchanged_depth():
    s = scene()
    before = [a.tobytes() for a in (s.color, s.mask, s.depth)]
    result = diagnose(s)
    assert result['raster']['complete_outer_edges_supported']
    assert result['raster']['physical_edge_assignment_complete']
    assert result['raster']['projection_inside_original_4px_margin']
    assert result['coverage']['original_top_interior_88pct_supported']
    assert result['measured_depth']['contradiction_count'] == 0
    assert before == [a.tobytes() for a in (s.color, s.mask, s.depth)]
    assert result['qualified_model'] is None and result['motion_authorized'] is False
    assert validate_cuboid_silhouette_diagnostic(result, result['source']) == result
    json.dumps(result, allow_nan=False)


def test_barcode_side_can_extend_past_mask_without_inventing_boundary_equality_gate():
    s = scene()
    s.mask[s.side] = 0
    result = diagnose(s)
    assert result['raster']['complete_outer_edges_supported']
    assert result['coverage']['original_top_interior_88pct_supported']
    assert result['coverage']['missing_mask_pixels_inside_model'] > 0
    assert result['coverage']['model_silhouette_mask_coverage_fraction'] < 1.
    assert result['coverage']['strict_boundary_equality_required'] is False
    assert result['qualified_model'] is None


def test_missing_edge_is_not_replaced_with_internal_print_or_nearest_infinite_line():
    s = scene()
    s.color[:] = 24
    # A visible internal stripe cannot support all six real external edges.
    s.color[110:113, 150:170] = 0
    result = diagnose(s)
    assert not result['raster']['complete_outer_edges_supported']
    assert any(not edge['complete_supported'] for edge in result['projection']['edges'])


def test_unknown_pixels_keep_top_side_ambiguous_and_outside_explanations():
    s = scene()
    s.depth[s.mask > 0] = 0
    result = diagnose(s)
    report = result['unknown_pixels']
    assert len(report['pixels_xy']) == np.count_nonzero(s.mask)
    assert sum(report['counts'].values()) == np.count_nonzero(s.mask)
    assert report['counts']['top'] > 0 and report['counts']['side'] > 0
    assert report['counts']['ambiguous'] > 0
    assert 'resolved_sides' not in report
    assert result['measured_side_surfaces'] is False
    assert result['measured_depth']['valid_mask_pixels'] == 0


def test_observed_depth_contradiction_is_retained_without_filtering_source():
    s = scene()
    y, x = np.argwhere(s.top)[len(np.argwhere(s.top))//2]
    s.depth[y, x] += 100
    before = s.depth.copy()
    result = diagnose(s)
    assert result['measured_depth']['contradiction_count'] >= 1
    assert any(row['pixel_xy'] == [int(x), int(y)] for row in result['measured_depth']['contradictions'])
    np.testing.assert_array_equal(s.depth, before)
    assert result['source']['depth_sha256'] == hashlib.sha256(s.depth.tobytes()).hexdigest()


def test_no_canny_evidence_stays_finite_json_and_is_explicitly_unsupported():
    s = scene()
    s.color[:] = 24
    result = diagnose(s)
    assert not result['raster']['complete_outer_edges_supported']
    assert all(row['complete_max_distance_px'] is None for row in result['projection']['edges'])
    json.dumps(result, allow_nan=False)


def test_complete_rotated_true_mask_does_not_validate_its_internal_print_cube():
    # Same analytic GEN37 counterexample, now through the new diagnostic API.
    height, width = 960, 1280
    yaw = math.radians(10.)
    turn = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    true_corners = np.array([[-1., -1.], [1., -1.], [1., 1.], [-1., 1.]])*[.026, .0175] @ turn.T
    px = np.column_stack((true_corners[:, 0]*10000+640, -true_corners[:, 1]*10000+480))
    mask = np.zeros((height, width), np.uint8)
    cv2.fillConvexPoly(mask, np.rint(px).astype(np.int32), 255)
    color = np.full((height, width, 3), 128, np.uint8)
    color[mask > 0] = [0, 150, 134]
    assert np.all(cv2.cvtColor(color, cv2.COLOR_BGR2GRAY) == 128)
    cv2.rectangle(color, (420, 345), (860, 615), (0, 0, 0), 1)
    depth = np.full((height, width), 5200, np.uint16)
    depth[mask > 0] = 0
    depth[345:616, 420:861] = 5000
    T = np.diag([1., -1., -1., 1.])
    T[:3, 3] = [0., 0., .52]
    s = SimpleNamespace(color=color, mask=mask, depth=depth, K=np.array([5000., 5000., 640., 480.]),
        T=T, center=np.array([0., 0., .01]), axes=np.eye(3), size=np.array([.044, .027, .020]), scale=.0001)
    result = diagnose(s, pixel_footprint_m=.0001)
    assert result['raster']['complete_outer_edges_supported']  # Print alone is misleading.
    assert not result['raster']['source_mask_boundary_canny_supported']
    assert not result['coverage']['source_explained_88pct_report_only']
    assert result['coverage']['source_mask_explained_by_model_fraction'] < .70
    assert result['unknown_pixels']['counts']['outside'] > 0
    assert result['qualified_model'] is None and result['motion_authorized'] is False


def test_projection_accepts_existing_namespace_intrinsics_without_changing_geometry():
    s = scene()
    K = SimpleNamespace(fx=s.K[0], fy=s.K[1], cx=s.K[2], cy=s.K[3])
    a = project_cuboid_silhouette(s.center, s.axes, s.size, s.T, K)
    b = project_cuboid_silhouette(s.center, s.axes, s.size, s.T, s.K)
    np.testing.assert_array_equal(a['vertices_pixel'], b['vertices_pixel'])


@pytest.mark.parametrize('fault', ['source', 'candidate', 'coverage', 'motion', 'qualified'])
def test_tampered_diagnostic_cannot_be_promoted_or_rebound(fault):
    result = diagnose(scene())
    changed = copy.deepcopy(result)
    if fault == 'source':
        changed['source']['stamp_ns'] += 1
    elif fault == 'candidate':
        changed['candidate']['size_xyz_m'][0] -= .001
    elif fault == 'coverage':
        changed['coverage']['source_mask_explained_by_model_fraction'] *= .5
    elif fault == 'motion':
        changed['motion_authorized'] = True
    else:
        changed['qualified_model'] = {'ok': True}
    with pytest.raises(ValueError):
        validate_cuboid_silhouette_diagnostic(changed)


def test_expected_source_must_match_even_when_proof_digest_is_valid():
    result = diagnose(scene())
    source = dict(result['source'], stamp_ns=result['source']['stamp_ns']+1)
    with pytest.raises(ValueError, match='source'):
        validate_cuboid_silhouette_diagnostic(result, source)


@pytest.mark.parametrize('fault', ['reflection', 'zero_size', 'bad_support', 'floating',
                                   'bad_camera', 'bad_focal', 'zero_scale', 'boolean_stamp'])
def test_invalid_geometry_or_source_is_explicitly_rejected(fault):
    s = scene()
    kwargs = {}
    if fault == 'reflection':
        s.axes[:, 0] *= -1.
    elif fault == 'zero_size':
        s.size[1] = 0.
    elif fault == 'bad_support':
        kwargs['support_normal_base'] = [0., 0., 2.]
    elif fault == 'floating':
        s.center[2] += .001
    elif fault == 'bad_camera':
        s.T[3, 0] = 1.
    elif fault == 'bad_focal':
        s.K[0] = 0.
    elif fault == 'zero_scale':
        s.scale = 0.
    else:
        kwargs['source_stamp_ns'] = True
    with pytest.raises(ValueError):
        diagnose(s, **kwargs)
