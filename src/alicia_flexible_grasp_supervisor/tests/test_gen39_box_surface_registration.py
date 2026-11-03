"""Source-closed choice with real box qualification and original fixed gates.

Only selected tests inject the optimizer's proposed transform. Their proposal
is first accepted by the real registration evaluator; neither fixed-transform
gates, measured-family checks nor fusion are replaced. Actual ICP/recorded
producer/task chain replay is covered separately, not claimed by this unit.
"""
from dataclasses import replace
import hashlib
import json
import math
from types import SimpleNamespace as NS

import numpy as np
import pytest

from alicia_flexible_grasp.vision import box_surface_registration as choice
from alicia_flexible_grasp.vision import multiview_surface as surface
from alicia_flexible_grasp.vision import regular_box_inference as box
from test_gen26_regular_box_inference_independent import ray_scene, qualify
from test_gen31_oblique_box_independent import oblique_box_scene
from test_gen31_outer_rgb_box import visual_edge_gap_scene


@pytest.fixture(scope='module')
def raw():
    scene = ray_scene()
    # Keep real deprojected samples; reduce only the unit's correspondence cost.
    scene.geometry = replace(scene.geometry,
        object_points_base=scene.geometry.object_points_base[::5])
    qualified = qualify(scene)
    assert qualified.ok, (qualified.code, qualified.audit)
    moving = surface.SurfaceView(scene.identity, scene.stamp,
        scene.geometry.object_points_base, scene.geometry.support_normal_base,
        scene.geometry.support_offset_m, 30)
    reference = replace(moving, stamp_ns=scene.stamp-2_000_000_000)
    before = surface.fused_surface_from_view(reference, voxel_size_m=.0001)
    return NS(scene=scene, geometry=scene.geometry, model=qualified.model,
              moving=moving, reference=reference, before=before)


def proposed(monkeypatch, raw, distance=.004, *, config=None, reference=None):
    reference = raw.reference if reference is None else reference
    T = np.eye(4)
    T[:3, 3] = raw.model.axes_base[:, 0]*distance
    checked = surface.evaluate_registration_transform(reference, raw.moving, T, config)
    assert checked.ok, (checked.code, checked.rmse_m, checked.overlap_fraction)
    # Optimization alone is injected; append still invokes the original fixed
    # evaluator for the identity choice, and all scene/source checks are real.
    monkeypatch.setattr(surface, 'register_surface_view', lambda *_: checked)
    return checked


def run(raw, **changes):
    args = dict(surface=raw.before, reference=raw.reference, moving=raw.moving,
                raw_geometry=raw.geometry, raw_box_model=raw.model, voxel_size_m=.0001)
    args.update(changes)
    return choice.append_box_consistent_registered_view(**args)


def assert_same_result(actual, expected):
    assert actual.ok == expected.ok and actual.code == expected.code
    np.testing.assert_array_equal(actual.transform_base, expected.transform_base)
    assert (actual.inlier_count, actual.overlap_fraction, actual.rmse_m,
            actual.maximum_point_displacement_m) == (
        expected.inlier_count, expected.overlap_fraction, expected.rmse_m,
        expected.maximum_point_displacement_m)


def test_ordinary_real_icp_keeps_original_result_and_fusion(raw):
    baseline, expected = surface.append_registered_view(raw.before, raw.reference,
        raw.moving, voxel_size_m=.0001)
    result, fused, audit = run(raw)
    assert baseline.ok
    assert_same_result(result, baseline)
    np.testing.assert_array_equal(fused.points_base, expected.points_base)
    assert fused.view_sources == expected.view_sources
    assert audit['selected_hypothesis'] == 'original_icp'
    assert audit['selection_reason'] == 'ORIGINAL_ICP_BOX_CONSISTENT'
    assert 'identity' not in audit['hypotheses']
    assert audit['final_fused_model_authorized'] is False
    assert audit['execution_authorized'] is False


def test_only_original_gate_accepted_identity_can_resolve_complete_history_conflict(raw, monkeypatch):
    original = proposed(monkeypatch, raw)
    original_points = raw.before.points_base.copy()
    original_sources = raw.before.view_sources
    result, fused, audit = run(raw)
    assert audit['hypotheses']['original_icp']['registration']['ok']
    assert not audit['hypotheses']['original_icp']['consistency']['ok']
    assert audit['hypotheses']['identity']['registration']['ok']
    assert audit['hypotheses']['identity']['consistency']['ok']
    assert audit['selected_hypothesis'] == 'identity'
    np.testing.assert_array_equal(result.transform_base, np.eye(4))
    assert np.linalg.norm(original.transform_base[:3, 3]) > .003
    np.testing.assert_array_equal(raw.before.points_base, original_points)
    assert raw.before.view_sources == original_sources
    assert fused.view_stamps_ns == raw.before.view_stamps_ns+(raw.moving.stamp_ns,)
    # Exact old representatives survive unchanged; source sets may gain a view.
    assert set(map(tuple, original_points)).issubset(set(map(tuple, fused.points_base)))


def test_two_consistent_explanations_preserve_original_icp_without_aperture_score(raw, monkeypatch):
    original = proposed(monkeypatch, raw, .0002)
    result, _fused, audit = run(raw)
    assert_same_result(result, original)
    assert audit['selected_hypothesis'] == 'original_icp'
    assert audit['selection_uses_opening_or_path_score'] is False
    assert 'identity' not in audit['hypotheses']


def test_complete_history_outside_both_boxes_keeps_original_measured_fallback(raw, monkeypatch):
    original = proposed(monkeypatch, raw)
    # This occupied point belongs to a different accepted historical view,
    # not the reference used by ICP. It must still enter both exact unions.
    extra = raw.model.center_base+raw.model.axes_base[:, 0]*.100
    before = surface.FusedTargetSurface(raw.before.identity,
        np.vstack((raw.before.points_base, extra)),
        np.r_[raw.before.view_indices, 1],
        raw.before.view_stamps_ns+(raw.moving.stamp_ns-1_000_000_000,),
        raw.before.view_sources+((1,),))
    result, fused, audit = run(raw, surface=before)
    assert_same_result(result, original)
    assert audit['selected_box_explanation'] is False
    assert audit['selection_reason'] == 'NO_BOX_CONSISTENT_HYPOTHESIS_KEEP_MEASURED'
    assert all(not h['consistency']['ok'] for h in audit['hypotheses'].values())
    assert all(h['consistency']['before_history_point_count'] == len(before.points_base)
               for h in audit['hypotheses'].values())
    assert any(np.array_equal(p, extra) for p in fused.points_base)


def test_failed_original_registration_is_never_replaced_by_identity(raw, monkeypatch):
    T = np.eye(4); T[:3, 3] = raw.model.axes_base[:, 0]*.004
    config = surface.RegistrationConfig(maximum_translation_m=.001)
    failed = surface.evaluate_registration_transform(raw.reference, raw.moving, T, config)
    assert not failed.ok and failed.code == 'TRANSLATION_BOUND_EXCEEDED'
    assert surface.evaluate_registration_transform(raw.reference, raw.moving, np.eye(4), config).ok
    monkeypatch.setattr(surface, 'register_surface_view', lambda *_: failed)
    result, fused, audit = run(raw, config=config)
    assert_same_result(result, failed)
    assert fused is raw.before
    assert audit['selection_reason'] == 'ORIGINAL_REGISTRATION_FAILED'
    assert 'identity' not in audit['hypotheses']


def test_original_success_does_not_override_identity_rmse_gate(raw, monkeypatch):
    points = raw.moving.points_base+np.array([0., 0., .004])
    reference = replace(raw.reference, points_base=points)
    before = surface.fused_surface_from_view(reference, voxel_size_m=.0001)
    bottom = raw.model.center_base-raw.model.support_normal_base*raw.model.size_xyz_m[2]/2.
    before = surface.FusedTargetSurface(before.identity, np.vstack((before.points_base, bottom)),
        np.r_[before.view_indices, 1], before.view_stamps_ns+(raw.moving.stamp_ns-1_000_000_000,),
        before.view_sources+((1,),))
    T = np.eye(4); T[2, 3] = .004
    config = surface.RegistrationConfig(maximum_rmse_m=.003)
    original = surface.evaluate_registration_transform(reference, raw.moving, T, config)
    assert original.ok
    identity = surface.evaluate_registration_transform(reference, raw.moving, np.eye(4), config)
    assert not identity.ok and identity.code == 'RMSE_EXCEEDED'
    monkeypatch.setattr(surface, 'register_surface_view', lambda *_: original)
    result, _fused, audit = run(raw, surface=before, reference=reference, config=config)
    assert_same_result(result, original)
    assert audit['selection_reason'] == 'IDENTITY_REGISTRATION_REJECTED_KEEP_MEASURED'
    assert audit['hypotheses']['identity']['consistency'] is None


def rehash_model(model, change):
    value = model.to_evidence_dict(); change(value)
    value['model_sha256'] = hashlib.sha256(json.dumps(
        {k:v for k,v in value.items() if k != 'model_sha256'},
        sort_keys=True, separators=(',', ':'), allow_nan=False, ensure_ascii=True).encode()).hexdigest()
    return box.RegularBoxInference.from_evidence_dict(value)


@pytest.mark.parametrize('fault', ['none', 'dict', 'target', 'stamp', 'geometry_digest',
    'registered_model', 'raw_points', 'moving_points', 'moving_support', 'raw_support'])
def test_invalid_raw_binding_keeps_original_icp_and_original_measured_fusion(raw, monkeypatch, fault):
    original = proposed(monkeypatch, raw)
    changes = {}
    if fault == 'none': changes['raw_box_model'] = None
    elif fault == 'dict': changes['raw_box_model'] = raw.model.to_evidence_dict()
    elif fault == 'target': changes['raw_box_model'] = rehash_model(raw.model,
        lambda d: d['source']['target_identity'].update(track_id='different'))
    elif fault == 'stamp': changes['raw_box_model'] = rehash_model(raw.model,
        lambda d: d['source'].update(stamp_ns=d['source']['stamp_ns']+1))
    elif fault == 'geometry_digest': changes['raw_box_model'] = rehash_model(raw.model,
        lambda d: d['source'].update(geometry_sha256='0'*64))
    elif fault == 'registered_model':
        T = np.eye(4); T[0, 3] = .001
        changes['raw_box_model'] = box.transform_inferred_box(raw.model, T)
    elif fault == 'raw_points': changes['raw_geometry'] = replace(raw.geometry,
        object_points_base=raw.geometry.object_points_base[::-1])
    elif fault == 'moving_points': changes['moving'] = replace(raw.moving,
        points_base=raw.moving.points_base[::-1])
    elif fault == 'moving_support': changes['moving'] = replace(raw.moving, support_offset_m=.001)
    else: changes['raw_geometry'] = replace(raw.geometry, support_offset_m=.001)
    expected, expected_surface = surface.append_registered_view(raw.before, raw.reference,
        changes.get('moving', raw.moving), voxel_size_m=.0001)
    assert expected.ok
    result, fused, audit = run(raw, **changes)
    assert_same_result(result, expected)
    np.testing.assert_array_equal(fused.points_base, expected_surface.points_base)
    assert audit['selection_reason'] == 'RAW_MODEL_BINDING_UNAVAILABLE'
    assert audit['selected_box_explanation'] is False
    assert 'identity' not in audit['hypotheses']


@pytest.fixture(scope='module', params=[1, 2, 3])
def qualified_family(request):
    if request.param == 1:
        scene = ray_scene(); result = qualify(scene)
    elif request.param == 2:
        scene = visual_edge_gap_scene()
        result = box.qualify_regular_box_top(scene.depth, scene.mask, scene.K,
            .0001, scene.T, scene.geometry, scene.identity, scene.stamp,
            'instance_mask', color_bgr=scene.color)
    else:
        scene = oblique_box_scene()
        result = box._qualify_regular_box_top(scene.depth, scene.mask, scene.K,
            scene.scale, scene.T, scene.geometry, scene.identity, scene.stamp,
            'instance_mask', color_bgr=scene.color, cuboid_fallback=True)
    assert result.ok, (result.code, result.audit)
    assert result.model.schema_version == request.param
    return scene, result.model


def test_original_qualifier_geometry_digest_is_shared_exactly(qualified_family):
    scene, model = qualified_family
    assert box.regular_box_geometry_sha256(scene.geometry) == model.source['geometry_sha256']
    checked = box.evaluate_box_measured_consistency(model, scene.geometry.object_points_base)
    assert checked.ok
    assert checked.maximum_observed_model_excess_m == model.qualification['maximum_observed_model_excess_m']


@pytest.mark.parametrize('axis', [0, 1, 2])
def test_same_original_family_threshold_and_conflict_witness_for_all_schemas(qualified_family, axis):
    scene, model = qualified_family
    size = model.size_xyz_m; half = size/2.
    if model.schema_version in (2, 3):
        for j in range(2):
            turn = min(model.empirical_yaw_uncertainty_rad, math.atan2(size[1-j], size[j]))
            half[j] = (size[j]*math.cos(turn)+size[1-j]*math.sin(turn))/2.
    half += np.r_[box.inferred_box_axis_uncertainty(model), model.empirical_height_uncertainty_m]
    local = np.zeros(3); local[axis] = half[axis]+.0015
    extra = model.center_base+model.axes_base@local
    points = np.vstack((scene.geometry.object_points_base, extra))
    original = points.copy(); original_model = model.to_evidence_dict()
    result = box.evaluate_box_measured_consistency(model, points)
    assert not result.ok and result.code == 'BOX_MEASURED_GEOMETRY_CONFLICT'
    assert result.maximum_observed_model_excess_m == pytest.approx(.0015, abs=1e-15)
    conflict = result.conflict
    assert conflict['point_index'] == len(points)-1 and conflict['axis_index'] == axis
    assert conflict['permitted_observed_excess_m'] == .001
    assert conflict['measured_points_sha256'] == hashlib.sha256(points.tobytes()).hexdigest()
    np.testing.assert_array_equal(points, original)
    assert model.to_evidence_dict() == original_model
    conflict['point_base_m'][0] += 1.
    assert result.conflict['point_base_m'] != conflict['point_base_m']


@pytest.mark.parametrize('points', [[], [[0., 0.]], [[float('nan'), 0., 0.]],
    [[float('inf'), 0., 0.]], [[True, False, True]], [['x', 'y', 'z']]])
def test_public_consistency_rejects_malformed_points_without_model_or_geometry_change(raw, points):
    with pytest.raises(ValueError):
        box.evaluate_box_measured_consistency(raw.model, points)
