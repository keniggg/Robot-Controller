"""Current RGB outer contour replaces absent depth-edge support explicitly."""
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from test_gen26_regular_box_inference_independent import ray_scene, qualify
from alicia_flexible_grasp.vision.regular_box_inference import qualify_regular_box_top


def visual_edge_gap_scene():
    scene = ray_scene(width=.035, length=.065)
    actual = scene.mask > 0
    scene.color = np.full(scene.depth.shape+(3,), 20, np.uint8)
    scene.color[actual] = [70, 140, 220]
    # Actual RGB remains complete. Segmentation is clipped at its corners,
    # and one depth edge is absent; neither creates artificial measured data.
    x, y = scene.local[:, :, 0], scene.local[:, :, 1]
    corners = (np.abs(x) > .029) & (np.abs(y) > .014)
    scene.mask[corners] = 0
    missing = actual & (y > .012)
    scene.depth[missing] = 0
    points = scene.geometry.object_points_base
    keep = (points@scene.geometry.axes_base)[:, 1] <= .012
    scene.geometry = replace(scene.geometry, object_points_base=points[keep])
    return scene


def qualify_visual_scene(scene, **changes):
    return qualify(scene, color_bgr=scene.color, **changes)


def precision_interior_scene():
    """Real current-source unknown interior, with complete RGB outer evidence."""
    scene = visual_edge_gap_scene()
    x, y = scene.local[:, :, 0], scene.local[:, :, 1]
    unknown = (np.abs(x) < .006) & (np.abs(y) < .005)
    scene.depth[unknown] = 0
    local = scene.geometry.object_points_base@scene.geometry.axes_base
    keep = ~((np.abs(local[:, 0]) < .006) & (np.abs(local[:, 1]) < .005))
    scene.geometry = replace(scene.geometry, object_points_base=scene.geometry.object_points_base[keep])
    return scene


def precision_scene_snapshot(scene):
    from alicia_flexible_grasp.vision.rgbd_snapshot import RgbdSample, fuse_stable_samples
    yy, xx = np.where(scene.mask)
    sample = RgbdSample(scene.color, scene.depth, scene.mask,
        (int(xx.min()), int(yy.min()), int(np.ptp(xx))+1, int(np.ptp(yy))+1),
        None, scene.stamp*1e-9, 'camera_optical', np.zeros(6), scene.stamp,
        scene.identity.epoch, scene.identity)
    result = fuse_stable_samples([sample], require_mask=True, min_mask_iou=.85,
        max_centroid_shift_px=5., max_joint_delta_rad=.01, erosion_px=0,
        depth_scale=.0001, depth_min_m=.03, depth_max_m=2., mad_scale=3.,
        mad_absolute_floor_m=.002, internal_hole_max_area_px=0)
    assert result.ok, result.failure_reason
    return result


def test_full_rgb_outer_edges_qualify_explicit_v2_with_depth_gaps_audited():
    scene = visual_edge_gap_scene()
    original = scene.geometry.object_points_base.copy()
    result = qualify_visual_scene(scene)
    assert result.ok, (result.code, result.audit)
    assert result.model.schema_version == 2
    assert result.model.qualification['contour_source'] == 'rgb_outer_joint_interval_on_measured_top_plane'
    assert min(result.model.qualification['corner_coverage']) < .5 or min(result.model.qualification['edge_coverage']) < .7
    np.testing.assert_array_equal(scene.geometry.object_points_base, original)
    np.testing.assert_allclose(sorted(result.model.size_xyz_m[:2]), [.035, .065], atol=.0015)


@pytest.mark.parametrize('fault', ['missing_rgb_edge', 'large_chamfer', 'crop', 'internal_hole', 'external_top'])
def test_incomplete_or_contradictory_rgb_cannot_supply_box_qualification(fault):
    scene = visual_edge_gap_scene()
    x, y = scene.local[:, :, 0], scene.local[:, :, 1]
    if fault == 'missing_rgb_edge':
        scene.color[y > .01] = 20
    elif fault == 'large_chamfer':
        cut = (x > .018) & (y > .003) & (x+y > .033)
        scene.color[cut] = 20
        scene.mask[cut] = 0
    elif fault == 'crop':
        scene.mask[0, 120:140] = 255
    elif fault == 'internal_hole':
        scene.depth[(abs(x)<.009)&(abs(y)<.008)] = 0
    else:
        # Extend the measured top beyond the outer color boundary while the
        # original object mask remains narrow: this is an internal print edge.
        same = (abs(x)<.04)&(abs(y)<.026)
        scene.depth[same] = 3250
    result = qualify_visual_scene(scene)
    assert not result.ok, (result.code, result.audit)


def test_unchanged_complete_depth_path_keeps_schema_v1():
    result = qualify(ray_scene())
    assert result.ok
    assert result.model.schema_version == 1


def test_v2_measured_yaw_extreme_uses_same_envelope_as_opening_and_collision():
    scene = visual_edge_gap_scene()
    qualified = qualify_visual_scene(scene)
    assert qualified.ok
    model = qualified.model
    angle = model.empirical_yaw_uncertainty_rad
    rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    local = np.r_[rotation@(model.size_xyz_m[:2]/2.), 0.]
    # Existing measured-conflict tolerance remains 1mm. The endpoint lies
    # 0.75mm beyond the yaw+edge family, within that unchanged allowance.
    local[1] += model.empirical_edge_uncertainty_m + .00075
    point = model.center_base+model.axes_base@local
    geometry = replace(scene.geometry, object_points_base=np.vstack((scene.geometry.object_points_base, point)))
    result = qualify_visual_scene(scene, geometry=geometry)
    assert result.ok, (result.code, result.audit)
    assert result.audit['maximum_observed_model_excess_m'] == pytest.approx(.00075, abs=1e-10)


def test_unknown_interior_returns_only_source_bound_precision_target_without_filling_depth():
    from alicia_flexible_grasp.vision.regular_box_inference import (
        RegularBoxInference, validate_box_precision_candidate_source)
    scene = precision_interior_scene()
    original_depth = scene.depth.copy()
    original_points = scene.geometry.object_points_base.copy()
    result = qualify_visual_scene(scene)
    assert not result.ok and result.model is None
    assert result.code == 'BOX_INFERENCE_PRECISION_VIEW_REQUIRED'
    candidate = result.audit['precision_view_candidate']
    assert candidate['motion_authorized'] is False
    assert candidate['qualification'] == 'observation_only_not_grasp_qualified'
    assert result.audit['unknown_interior_is_not_measured_hole'] is True
    assert result.audit['unknown_depth_filled'] is False
    unknown = np.asarray(candidate['targeted_view']['missing_region_pixels_yx'])
    assert len(unknown) > 0 and np.all(scene.depth[unknown[:, 0], unknown[:, 1]] == 0)
    bound = validate_box_precision_candidate_source(candidate, precision_scene_snapshot(scene),
        scene.K, .0001, scene.T, scene.geometry, scene.identity, scene.stamp)
    assert bound == candidate and bound is not candidate
    with pytest.raises(ValueError):
        RegularBoxInference.from_evidence_dict(candidate)
    np.testing.assert_array_equal(scene.depth, original_depth)
    np.testing.assert_array_equal(scene.geometry.object_points_base, original_points)


def test_measured_deep_interior_recess_is_not_classified_as_unknown_depth():
    scene = precision_interior_scene()
    x, y = scene.local[:, :, 0], scene.local[:, :, 1]
    region = (np.abs(x) < .006) & (np.abs(y) < .005)
    scene.depth[region] = 3400  # actual measured layer 15 mm below the top
    result = qualify_visual_scene(scene)
    assert not result.ok and result.model is None
    assert 'precision_view_candidate' not in result.audit
    assert result.code == 'BOX_TOP_CONTOUR_INCOMPLETE'
    assert any(row['measured_recess_or_occluder_pixels_yx'] for row in result.audit['interior_depth_regions'])


@pytest.mark.parametrize('fault', ['authority', 'unknown_partition', 'measured_distance', 'target_pixels', 'jaw', 'hash'])
def test_precision_target_rejects_rehashed_inconsistent_evidence(fault):
    from copy import deepcopy
    from alicia_flexible_grasp.vision import regular_box_inference as box
    candidate = deepcopy(qualify_visual_scene(precision_interior_scene()).audit['precision_view_candidate'])
    if fault == 'authority':
        candidate['motion_authorized'] = True
    elif fault == 'unknown_partition':
        candidate['interior_regions'][0]['raw_unknown_pixels_yx'].pop()
    elif fault == 'measured_distance':
        candidate['interior_regions'][0]['measured_plane_distance_m'].append(.1)
    elif fault == 'target_pixels':
        candidate['targeted_view']['missing_region_pixels_yx'].pop()
    elif fault == 'jaw':
        candidate['targeted_view']['jaw_axis_base'] = candidate['axes_base'][2]
    if fault != 'hash':
        candidate['candidate_sha256'] = box._hash({k:v for k,v in candidate.items() if k != 'candidate_sha256'})
    else:
        candidate['candidate_sha256'] = '0'*64
    with pytest.raises(ValueError):
        box.validate_box_precision_candidate(candidate)


@pytest.mark.parametrize('fault', ['rgb', 'depth', 'mask', 'intrinsics', 'scale', 'transform', 'geometry', 'stamp', 'identity', 'admission'])
def test_precision_source_rejects_stale_or_changed_inputs_without_refitting(fault):
    from alicia_flexible_grasp.vision.regular_box_inference import validate_box_precision_candidate_source
    from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
    scene = precision_interior_scene()
    candidate = qualify_visual_scene(scene).audit['precision_view_candidate']
    snapshot = precision_scene_snapshot(scene)
    args = dict(snapshot=snapshot, camera_intrinsics=scene.K, depth_scale_m=.0001,
        T_base_camera=scene.T, geometry=scene.geometry, source_identity=scene.identity,
        source_stamp_ns=scene.stamp)
    if fault in ('rgb', 'depth', 'mask'):
        field = {'rgb': 'color_bgr', 'depth': 'depth_raw', 'mask': 'object_mask'}[fault]
        value = np.array(getattr(snapshot.original_sample, field), copy=True)
        value[0, 0] = 1 if fault != 'mask' else 255
        original = replace(snapshot.original_sample, **{field:value})
        changed = dict(original_sample=original)
        if fault == 'rgb': changed['color_bgr'] = value
        args['snapshot'] = replace(snapshot, **changed)
    elif fault == 'intrinsics':
        args['camera_intrinsics'] = SimpleNamespace(**dict(vars(scene.K), fx=scene.K.fx+1.))
    elif fault == 'scale':
        args['depth_scale_m'] = .00011
    elif fault == 'transform':
        T = scene.T.copy(); T[0, 3] += .001; args['T_base_camera'] = T
    elif fault == 'geometry':
        args['geometry'] = replace(scene.geometry, object_points_base=scene.geometry.object_points_base[:-1])
    elif fault == 'stamp':
        args['source_stamp_ns'] += 1
    elif fault == 'identity':
        args['source_identity'] = TargetTrackIdentity(2, 'g4-t2')
    else:
        args['snapshot'] = replace(snapshot, ok=False)
    with pytest.raises(ValueError):
        validate_box_precision_candidate_source(candidate, **args)
