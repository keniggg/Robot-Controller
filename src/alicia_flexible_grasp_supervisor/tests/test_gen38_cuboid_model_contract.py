"""Rehashed schema3 mutations start from a real synthetic RGB-D producer.

The scene has independently rendered top/side/support depth and uses the real
geometry estimator. No model, contact permission or source proof is fabricated
to make the positive fixture pass. The cuboid branch is selected explicitly so
legacy schema1/2 success preservation is checked separately.
"""
import copy
from dataclasses import replace
import hashlib
import json
import math

import numpy as np
import cv2
import pytest

from alicia_flexible_grasp.vision import regular_box_inference as box
from test_gen31_oblique_box_independent import oblique_box_scene, update_geometry
from test_gen31_outer_rgb_box import visual_edge_gap_scene


def _produce(scene):
    return box._qualify_regular_box_top(scene.depth, scene.mask, scene.K,
        scene.scale, scene.T, scene.geometry, scene.identity, scene.stamp,
        'instance_mask', color_bgr=scene.color, cuboid_fallback=True)


def _rehash(value):
    body = {key:item for key, item in value.items() if key != 'model_sha256'}
    value['model_sha256'] = hashlib.sha256(json.dumps(body, sort_keys=True,
        separators=(',', ':'), ensure_ascii=True, allow_nan=False).encode()).hexdigest()
    return value


@pytest.fixture(scope='module')
def qualified_model():
    result = _produce(oblique_box_scene())
    assert result.ok, (result.code, result.audit.get('reason'))
    assert result.model.schema_version == 3
    return result.model.to_evidence_dict()


def test_real_cuboid_producer_keeps_full_source_and_height_model(qualified_model):
    parsed = box.RegularBoxInference.from_evidence_dict(qualified_model)
    q = parsed.qualification
    interval = q['cuboid_rgb_interval']
    assert q['contour_source'] == 'cuboid_rgb_joint_interval_on_measured_support'
    assert interval['source_color_sha256'] == parsed.source['color_sha256']
    assert interval['source_depth_sha256'] == parsed.source['depth_sha256']
    assert interval['source_mask_sha256'] == parsed.source['mask_sha256']
    assert interval['T_base_camera_sha256'] == parsed.source['T_base_camera_sha256']
    assert interval['intrinsics_sha256'] == parsed.source['intrinsics_sha256']
    assert interval['common_height_interval_m'][0] <= interval['original_common_height_interval_m'][0]+1e-9
    assert interval['common_height_interval_m'][1] >= interval['original_common_height_interval_m'][1]-1e-9
    assert interval['unknown_mask_pixel_count'] > 0
    axis_budget = np.asarray(interval['empirical_axis_uncertainty_m'])
    assert axis_budget.shape == (2,) and np.all(axis_budget >= .001)
    assert math.isclose(float(axis_budget.max()), parsed.empirical_edge_uncertainty_m, rel_tol=0., abs_tol=1e-9)


def test_existing_complete_oblique_branch_preserves_schema1():
    scene = oblique_box_scene()
    result = box.qualify_regular_box_top(scene.depth, scene.mask, scene.K,
        scene.scale, scene.T, scene.geometry, scene.identity, scene.stamp,
        'instance_mask', color_bgr=scene.color)
    assert result.ok and result.model.schema_version == 1


def test_existing_rgb_edge_gap_branch_preserves_schema2():
    scene = visual_edge_gap_scene()
    result = box.qualify_regular_box_top(scene.depth, scene.mask, scene.K,
        .0001, scene.T, scene.geometry, scene.identity, scene.stamp,
        'instance_mask', color_bgr=scene.color)
    assert result.ok and result.model.schema_version == 2


@pytest.mark.parametrize('field', ['color_sha256', 'depth_sha256', 'mask_sha256',
                                  'intrinsics_sha256', 'T_base_camera_sha256'])
def test_rehashed_model_cannot_rebind_interval_to_a_different_source(qualified_model, field):
    changed = copy.deepcopy(qualified_model)
    changed['source'][field] = '0'*64
    with pytest.raises(ValueError):
        box.RegularBoxInference.from_evidence_dict(_rehash(changed))


@pytest.mark.parametrize('mutation', ['translate_source', 'rotate_source'])
def test_rehashed_source_geometry_must_equal_interval_geometry(qualified_model, mutation):
    changed = copy.deepcopy(qualified_model)
    if mutation == 'translate_source':
        delta = np.asarray(changed['source_axes_base'])[:, 0]*.003
        for field in ('source_center_base', 'center_base'):
            changed[field] = (np.asarray(changed[field])+delta).tolist()
    else:
        angle = math.radians(4.)
        normal = np.asarray(changed['source_support_normal_base'])
        x, y, z = normal
        skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
        turn = np.eye(3)*math.cos(angle)+(1.-math.cos(angle))*np.outer(normal, normal)+math.sin(angle)*skew
        for field in ('source_axes_base', 'axes_base'):
            changed[field] = (turn @ np.asarray(changed[field])).tolist()
    with pytest.raises(ValueError):
        box.RegularBoxInference.from_evidence_dict(_rehash(changed))


@pytest.mark.parametrize('mutation', [
    'drop_full_edges', 'drop_raster_edge', 'false_top_support', 'nonunit_top_normal',
    'invalid_camera', 'group_outside_strip', 'false_nearest_canny',
    'reduced_height', 'inflate_scalar_edge', 'inflate_axis_budget', 'zero_unknown_count',
])
def test_rehashed_interval_cannot_replace_physical_or_numeric_proof(qualified_model, mutation):
    changed = copy.deepcopy(qualified_model)
    interval = changed['qualification']['cuboid_rgb_interval']
    if mutation == 'drop_full_edges':
        interval['full_silhouette_rgb_distances_px'] = [0.]
        interval['corner_rgb_distances_px'] = [0.]
    elif mutation == 'drop_raster_edge':
        interval['silhouette_raster_evidence'].pop()
    elif mutation == 'false_top_support':
        row = interval['groups'][0]
        row['plane'] = 'support' if row['plane'] == 'top' else 'top'
        row['plane_height_m'] = 0. if row['plane'] == 'support' else interval['nominal_height_m']
    elif mutation == 'nonunit_top_normal':
        interval['measured_top_normal_base'] = (np.asarray(interval['measured_top_normal_base'])*2.).tolist()
    elif mutation == 'invalid_camera':
        interval['T_base_optical'][3][0] = 1.
    elif mutation == 'group_outside_strip':
        interval['groups'][0]['line_pixels'][0] = [0, 0]
    elif mutation == 'false_nearest_canny':
        interval['silhouette_raster_evidence'][0]['corresponding_canny_pixels'][0] = [0, 0]
    elif mutation == 'reduced_height':
        changed['empirical_height_uncertainty_m'] *= .5
        interval['empirical_height_uncertainty_m'] = changed['empirical_height_uncertainty_m']
    elif mutation == 'inflate_scalar_edge':
        changed['empirical_edge_uncertainty_m'] += .0005
        interval['empirical_edge_uncertainty_m'] = changed['empirical_edge_uncertainty_m']
    elif mutation == 'inflate_axis_budget':
        interval['empirical_axis_uncertainty_m'][1] += .0005
        edge = max(interval['empirical_axis_uncertainty_m'])
        interval['empirical_edge_uncertainty_m'] = edge
        changed['empirical_edge_uncertainty_m'] = edge
    else:
        interval['unknown_mask_pixel_count'] = 0
    with pytest.raises(ValueError):
        box.RegularBoxInference.from_evidence_dict(_rehash(changed))


def test_source_registered_model_preserves_immutable_interval_frame(qualified_model):
    model = box.RegularBoxInference.from_evidence_dict(qualified_model)
    T = np.eye(4)
    angle = math.radians(3.)
    T[:3, :3] = [[math.cos(angle), -math.sin(angle), 0.],
                 [math.sin(angle), math.cos(angle), 0.], [0., 0., 1.]]
    T[:3, 3] = [.003, -.002, .001]
    updated = box.transform_inferred_box(model, T)
    assert updated.qualification['cuboid_rgb_interval'] == model.qualification['cuboid_rgb_interval']
    np.testing.assert_allclose(updated.center_base, T[:3, :3] @ model.center_base+T[:3, 3])
    assert updated.source == model.source


def test_observed_geometry_outside_cuboid_family_stays_rejected():
    scene = oblique_box_scene()
    points = scene.geometry.object_points_base.copy()
    points[0] += scene.true_axes[:, 0]*.08
    scene.geometry = replace(scene.geometry, object_points_base=points)
    result = _produce(scene)
    assert not result.ok and result.code == 'BOX_MEASURED_GEOMETRY_CONFLICT'
    assert result.model is None


def test_large_real_interior_depth_hole_does_not_become_qualified_cuboid():
    scene = oblique_box_scene()
    local = scene.local
    hole = scene.top & (np.abs(local[:, :, 0]) < .010) & (np.abs(local[:, :, 1]) < .008)
    scene.depth[hole] = 0
    update_geometry(scene)
    result = _produce(scene)
    assert not result.ok
    assert result.model is None


def _source_validator(interval, scene):
    from alicia_flexible_grasp.vision.cuboid_rgb_interval import validate_cuboid_rgb_interval_source
    return validate_cuboid_rgb_interval_source(interval, scene.color, scene.mask,
        scene.depth, scene.scale, scene.T,
        np.asarray([scene.K.fx, scene.K.fy, scene.K.cx, scene.K.cy]))


def _numeric_validator(model):
    from alicia_flexible_grasp.vision.cuboid_rgb_interval import validate_cuboid_rgb_interval
    return validate_cuboid_rgb_interval(model['qualification']['cuboid_rgb_interval'],
        model['size_xyz_m'], model['empirical_edge_uncertainty_m'],
        model['empirical_yaw_uncertainty_rad'], model['empirical_height_uncertainty_m'])


def test_real_source_replay_validator_is_required_and_accepts_original_frame(qualified_model):
    interval = qualified_model['qualification']['cuboid_rgb_interval']
    _source_validator(interval, oblique_box_scene())


@pytest.mark.parametrize('mode', ['empty', 'proper_subset'])
def test_rehashed_unknown_subset_cannot_pass_actual_source_validation(qualified_model, mode):
    changed = copy.deepcopy(qualified_model)
    interval = changed['qualification']['cuboid_rgb_interval']
    pixels = np.asarray(interval['unknown_mask_pixels_yx'], dtype=np.int64)
    subset = np.empty((0, 2), dtype=np.int64) if mode == 'empty' else pixels[::2].copy()
    assert len(subset) < len(pixels)
    interval['unknown_mask_pixels_yx'] = subset.tolist()
    interval['unknown_mask_pixel_count'] = len(subset)
    interval['unknown_mask_pixels_sha256'] = hashlib.sha256(subset.tobytes()).hexdigest()
    interval['unexplained_unknown_mask_pixel_count'] = 0
    # A self-consistent subset can still satisfy the pure numeric contract.
    # Only replay against the real unchanged mask/depth establishes completeness.
    _numeric_validator(_rehash(changed))
    with pytest.raises(ValueError):
        _source_validator(interval, oblique_box_scene())


def test_fabricated_nearby_non_canny_response_fails_actual_source_validation(qualified_model):
    changed = copy.deepcopy(qualified_model)
    interval = changed['qualification']['cuboid_rgb_interval']
    scene = oblique_box_scene()
    from alicia_flexible_grasp.vision.cuboid_silhouette import project_cuboid_silhouette
    axes = np.asarray(interval['nominal_axes_base'])
    lo, hi = np.asarray(interval['nominal_lower']), np.asarray(interval['nominal_upper'])
    normal = np.asarray(interval['support_normal_base'])
    center = axes[:, :2] @ ((lo+hi)/2.)+normal*(interval['nominal_height_m']/2.-interval['support_offset_m'])
    projection = project_cuboid_silhouette(center, axes, changed['size_xyz_m'],
        np.asarray(interval['T_base_optical']), interval['intrinsics'])
    edge = projection['edges'][0]
    start, end = np.asarray(edge['start_pixel']), np.asarray(edge['end_pixel'])
    canny = cv2.Canny(cv2.cvtColor(scene.color, cv2.COLOR_BGR2GRAY), 50., 100.)
    candidates = []
    for x in range(int(math.floor(start[0]))-2, int(math.ceil(start[0]))+3):
        for y in range(int(math.floor(start[1]))-2, int(math.ceil(start[1]))+3):
            if 0 <= x < canny.shape[1] and 0 <= y < canny.shape[0] and not canny[y, x] and np.linalg.norm(start-[x, y]) <= math.sqrt(8.):
                candidates.append([x, y])
    assert candidates
    points = interval['silhouette_raster_evidence'][0]['corresponding_canny_pixels']
    points[0] = candidates[0]
    samples = start+np.outer(np.linspace(0., 1., 101), end-start)
    distances = np.linalg.norm(samples-np.asarray(points), axis=1)
    interval['full_silhouette_rgb_distances_px'][0] = float(distances.max())
    interval['corner_rgb_distances_px'][0] = float(distances[0])
    _numeric_validator(_rehash(changed))
    with pytest.raises(ValueError):
        _source_validator(interval, scene)


def test_diagnostic_digest_cannot_replace_interval_qualification(qualified_model):
    from alicia_flexible_grasp.vision.cuboid_silhouette import diagnose_cuboid_silhouette
    scene = oblique_box_scene()
    changed = copy.deepcopy(qualified_model)
    diagnostic = diagnose_cuboid_silhouette(scene.color, scene.mask, scene.depth,
        scene.K, scene.scale, scene.T, scene.true_axes @ [0., 0., scene.true_size[2]/2.],
        scene.true_axes, scene.true_size, [0., 0., 1.], 0., source_stamp_ns=scene.stamp,
        target_identity=scene.identity, plane_slab_m=.001, pixel_footprint_m=.0004)
    changed['qualification']['cuboid_rgb_interval'] = diagnostic
    with pytest.raises(ValueError):
        box.RegularBoxInference.from_evidence_dict(_rehash(changed))


def test_declared_retained_raster_proposal_cannot_be_silently_removed(qualified_model):
    changed = copy.deepcopy(qualified_model)
    interval = changed['qualification']['cuboid_rgb_interval']
    assert len(interval['groups'][0]['raster_proposals']) == 2
    interval['groups'][0]['raster_proposals'].pop()
    with pytest.raises(ValueError):
        box.RegularBoxInference.from_evidence_dict(_rehash(changed))


def test_actual_source_replay_rejects_a_removed_competing_raster_response(qualified_model):
    changed = copy.deepcopy(qualified_model)
    interval = changed['qualification']['cuboid_rgb_interval']
    group = interval['groups'][0]
    assert len(group['line_pixels']) > 12
    group['line_pixels'].pop()
    group['metric_points'].pop()
    with pytest.raises(ValueError):
        _source_validator(interval, oblique_box_scene())


@pytest.mark.parametrize('principal', [0, 1])
@pytest.mark.parametrize('registered', [False, True])
def test_opening_and_collision_use_the_same_axis_family(qualified_model, principal, registered):
    from alicia_flexible_grasp.grasp.gripper_geometry import (
        ANALYTICAL_FINGER_SIZE_XYZ_M, ANALYTICAL_PALM_SIZE_XYZ_M,
        GripperGeometry, evaluate_inferred_box_contact)
    model = box.RegularBoxInference.from_evidence_dict(qualified_model)
    if registered:
        T = np.eye(4)
        angle = math.radians(13.)
        T[:3, :3] = [[math.cos(angle), -math.sin(angle), 0.],
                     [math.sin(angle), math.cos(angle), 0.], [0., 0., 1.]]
        T[:3, 3] = [.003, -.002, .001]
        model = box.transform_inferred_box(model, T)
    jaw, insertion = model.axes_base[:, principal], model.axes_base[:, 2]
    gripper = GripperGeometry(.05, .002, ANALYTICAL_FINGER_SIZE_XYZ_M,
                              ANALYTICAL_PALM_SIZE_XYZ_M, .003)
    evidence = evaluate_inferred_box_contact(model, model.center_base, jaw,
        insertion, gripper, opening_fit_clearance_each_side_m=.0005)
    enclosure = box.inferred_box_enclosure_vertices(model)
    collision_projected_width = float(np.ptp(enclosure @ jaw))
    assert math.isclose(evidence.inferred_width_m+evidence.width_uncertainty_m,
                        collision_projected_width, rel_tol=0., abs_tol=1e-9)
    np.testing.assert_allclose(box.inferred_box_axis_uncertainty(model),
        model.qualification['cuboid_rgb_interval']['empirical_axis_uncertainty_m'])


@pytest.mark.parametrize('version', [1, 2])
def test_legacy_models_keep_the_original_common_scalar_axis_budget(version):
    scene = oblique_box_scene() if version == 1 else visual_edge_gap_scene()
    result = box.qualify_regular_box_top(scene.depth, scene.mask, scene.K,
        scene.scale if version == 1 else .0001, scene.T, scene.geometry, scene.identity, scene.stamp,
        'instance_mask', color_bgr=scene.color)
    assert result.ok and result.model.schema_version == version
    np.testing.assert_array_equal(box.inferred_box_axis_uncertainty(result.model),
                                  [result.model.empirical_edge_uncertainty_m]*2)


def test_unknown_rays_cannot_be_explained_by_volume_below_verified_support():
    from alicia_flexible_grasp.vision.cuboid_rgb_interval import _unknown_mask_rays
    camera = np.array([0., -.2, .2])
    forward = np.array([0., 0., .01])-camera
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, [0., 0., 1.])
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = np.column_stack((right, down, forward)), camera
    K = np.array([600., 600., 159.5, 119.5])
    below = np.array([0., -.016, -.004])
    p = (below-camera) @ T[:3, :3]
    uv = np.rint(p[:2]/p[2]*K[:2]+K[2:]).astype(int)
    compatible = _unknown_mask_rays(np.array([[uv[1], uv[0]]]), T, K,
        np.eye(3), np.array([-.03, -.015]), np.array([.03, .015]),
        .02, 0., np.array([.001, .001]), math.radians(.25), .005)
    assert not bool(compatible[0])
