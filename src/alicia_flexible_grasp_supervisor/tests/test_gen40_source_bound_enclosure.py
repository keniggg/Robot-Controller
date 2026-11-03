"""Qualified planning bounds retain the complete family and all source points.

Analytical RGB-D fixtures run the real qualifiers. Explicit upstream geometry
doubles exercise a measured extension and unsupported old OBB corners; no
qualified model or pixel proof is fabricated. This is offline software evidence.
"""
from dataclasses import replace
from itertools import product
import hashlib
import json

import numpy as np
import pytest

from test_gen26_regular_box_inference_independent import ray_scene, qualify
from test_gen31_outer_rgb_box import visual_edge_gap_scene, qualify_visual_scene
from test_gen31_oblique_box_independent import oblique_box_scene
from alicia_flexible_grasp.vision import regular_box_inference as box
from alicia_flexible_grasp.vision.fused_scene_geometry import enclose_registered_surface
from alicia_flexible_grasp.vision.multiview_surface import FusedTargetSurface


POLICY = 'source_bound_measured_points_and_inferred_box_uncertainty'


def inside(points, geometry):
    return np.all(np.abs((points-geometry.center_base) @ geometry.axes_base)
                  <= geometry.size_xyz_m/2.+1e-12)


def corners(geometry):
    signs = np.asarray(list(product((-1., 1.), repeat=3)))
    return geometry.center_base+(signs*geometry.size_xyz_m/2.) @ geometry.axes_base.T


@pytest.fixture(scope='module')
def qualified_pair():
    scene = visual_edge_gap_scene()
    result = qualify_visual_scene(scene)
    assert result.ok and result.model.schema_version == 2, (result.code, result.audit)
    return scene, result.model


@pytest.mark.parametrize('schema', [1, 2, 3])
def test_real_qualified_schemas_enclose_every_point_and_full_family(schema):
    if schema == 1:
        scene = ray_scene()
        result = qualify(scene)
    elif schema == 2:
        scene = visual_edge_gap_scene()
        result = qualify_visual_scene(scene)
    else:
        scene = oblique_box_scene()
        result = box._qualify_regular_box_top(scene.depth, scene.mask, scene.K,
            scene.scale, scene.T, scene.geometry, scene.identity, scene.stamp,
            'instance_mask', color_bgr=scene.color, cuboid_fallback=True)
    assert result.ok and result.model.schema_version == schema, (result.code, result.audit)
    geometry, model = scene.geometry, result.model
    original_digest = box.regular_box_geometry_sha256(geometry)
    original_model = model.to_evidence_dict()
    output, audit = box.enclose_inferred_box_geometry(geometry, model)
    points = np.vstack((geometry.object_points_base, box.inferred_box_enclosure_vertices(model)))
    assert inside(points, output)
    # Exact extrema in the qualified axes, rather than a favorable subset.
    local = (points-model.center_base) @ model.axes_base
    np.testing.assert_allclose(output.size_xyz_m, np.ptp(local, axis=0), rtol=0., atol=1e-15)
    np.testing.assert_array_equal(output.object_points_base, geometry.object_points_base)
    assert box.regular_box_geometry_sha256(geometry) == original_digest == model.source['geometry_sha256']
    assert model.to_evidence_dict() == original_model
    assert audit['policy'] == POLICY and audit['schema_version'] == 1
    assert audit['source_geometry_sha256'] == original_digest
    assert audit['source_stamp_ns'] == model.source['stamp_ns']
    assert audit['source_target_identity'] == model.source['target_identity']
    assert audit['measured_points_count'] == len(geometry.object_points_base)
    assert audit['measured_points_sha256'] == hashlib.sha256(geometry.object_points_base.tobytes()).hexdigest()
    assert audit['producer_verified_all_points'] and audit['all_measured_points_contained']
    assert audit['full_family_contained'] and audit['full_family_vertex_count'] == 8
    assert not audit['original_obb_used_as_occupancy']
    assert not audit['consumer_independent_measured_point_verification']
    assert not audit['measured_side_surfaces'] and not audit['contact_or_motion_authorized']
    assert audit['synthetic_measured_points_added'] == 0
    np.testing.assert_array_equal(audit['original_center_base'], geometry.center_base)
    np.testing.assert_array_equal(audit['original_axes_base'], geometry.axes_base)
    np.testing.assert_array_equal(audit['original_size_xyz_m'], geometry.size_xyz_m)
    json.dumps(audit, allow_nan=False)


def extended_geometry(scene, model, extension):
    family = box.inferred_box_enclosure_vertices(model)
    local = (family-model.center_base) @ model.axes_base
    # An explicit admitted side sample in the upstream GeometryEstimate double.
    # Its original conflict allowance is tested by the real qualifier below.
    point = model.center_base+model.axes_base @ [local[:, 0].max()+extension, 0., 0.]
    points = np.vstack((scene.geometry.object_points_base, point))
    old_local = (points-scene.geometry.center_base) @ scene.geometry.axes_base
    size = np.maximum(scene.geometry.size_xyz_m, 2.*np.abs(old_local).max(axis=0))
    return replace(scene.geometry, object_points_base=points, size_xyz_m=size), point


def test_real_point_point_nine_mm_outside_family_is_retained(qualified_pair):
    scene, seed = qualified_pair
    geometry, witness = extended_geometry(scene, seed, .0009)
    result = qualify_visual_scene(scene, geometry=geometry)
    assert result.ok, (result.code, result.audit)
    model = result.model
    assert model.source['geometry_sha256'] == box.regular_box_geometry_sha256(geometry)
    assert result.audit['maximum_observed_model_excess_m'] == pytest.approx(.0009, abs=1e-10)
    family = box.inferred_box_enclosure_vertices(model)
    half = np.max(np.abs((family-model.center_base) @ model.axes_base), axis=0)
    assert ((witness-model.center_base) @ model.axes_base)[0] > half[0]+.00089
    output, audit = box.enclose_inferred_box_geometry(geometry, model)
    assert inside(np.vstack((geometry.object_points_base, family)), output)
    np.testing.assert_array_equal(output.object_points_base[-1], witness)
    np.testing.assert_array_equal(output.object_points_base, geometry.object_points_base)
    assert audit['maximum_observed_model_excess_m'] == pytest.approx(.0009, abs=1e-10)
    assert output.size_xyz_m[0] > np.ptp((family-model.center_base) @ model.axes_base, axis=0)[0]+.00089


def test_real_full_point_conflict_is_still_unqualified(qualified_pair):
    scene, seed = qualified_pair
    geometry, _ = extended_geometry(scene, seed, .0011)
    result = qualify_visual_scene(scene, geometry=geometry)
    assert not result.ok and result.model is None
    assert result.code == 'BOX_MEASURED_GEOMETRY_CONFLICT'
    with pytest.raises(ValueError):
        box.enclose_inferred_box_geometry(geometry, result.model)


def test_old_unmeasured_obb_corners_are_trace_only_for_qualified_path():
    scene = ray_scene()
    angle = .34
    R = np.array([[np.cos(angle), -np.sin(angle), 0.],
                  [np.sin(angle), np.cos(angle), 0.], [0., 0., 1.]])
    # Explicit loose upstream OBB; the real qualifier still uses the unchanged
    # RGB-D source/support/all points and binds this original geometry digest.
    geometry = replace(scene.geometry, axes_base=scene.geometry.axes_base @ R,
                       size_xyz_m=np.array([.13, .10, .025]))
    result = qualify(scene, geometry=geometry)
    assert result.ok, (result.code, result.audit)
    output, audit = box.enclose_inferred_box_geometry(geometry, result.model)
    assert not inside(corners(geometry), output)
    assert inside(geometry.object_points_base, output)
    assert inside(box.inferred_box_enclosure_vertices(result.model), output)
    assert audit['original_obb_used_as_occupancy'] is False
    np.testing.assert_array_equal(audit['original_size_xyz_m'], geometry.size_xyz_m)


@pytest.mark.parametrize('fault', [
    'center', 'axes', 'size', 'support_normal', 'support_offset',
    'support_ratio', 'point_value', 'point_delete', 'point_reorder',
])
def test_replaced_original_geometry_or_points_rejected(qualified_pair, fault):
    scene, model = qualified_pair
    geometry = scene.geometry
    if fault == 'center':
        geometry = replace(geometry, center_base=geometry.center_base+[.0001, 0., 0.])
    elif fault == 'axes':
        geometry = replace(geometry, axes_base=geometry.axes_base @ np.diag([-1., -1., 1.]))
    elif fault == 'size':
        geometry = replace(geometry, size_xyz_m=geometry.size_xyz_m+[.0001, 0., 0.])
    elif fault == 'support_normal':
        geometry = replace(geometry, support_normal_base=np.array([.001, 0., np.sqrt(1.-.001**2)]))
    elif fault == 'support_offset':
        geometry = replace(geometry, support_offset_m=.0001)
    elif fault == 'support_ratio':
        geometry = replace(geometry, support_inlier_ratio=.9)
    elif fault == 'point_value':
        points = geometry.object_points_base.copy(); points[0, 0] += 1e-9
        geometry = replace(geometry, object_points_base=points)
    elif fault == 'point_delete':
        geometry = replace(geometry, object_points_base=geometry.object_points_base[:-1])
    else:
        geometry = replace(geometry, object_points_base=geometry.object_points_base[::-1])
    with pytest.raises(ValueError, match='source geometry digest differs'):
        box.enclose_inferred_box_geometry(geometry, model)


@pytest.mark.parametrize('fault', ['failed', 'truthy_ok', 'failure_evidence', 'wrong_mode'])
def test_nonadmitted_geometry_cannot_enter_qualified_path(qualified_pair, fault):
    scene, model = qualified_pair
    changes = {'failed': dict(ok=False), 'truthy_ok': dict(ok=1),
        'failure_evidence': dict(failure_code='BAD'),
        'wrong_mode': dict(source_mode='bbox_fallback')}[fault]
    with pytest.raises(ValueError):
        box.enclose_inferred_box_geometry(replace(scene.geometry, **changes), model)


@pytest.mark.parametrize('fault', ['translation', 'rotation'])
def test_registered_model_cannot_be_assumed_to_share_original_coordinates(qualified_pair, fault):
    scene, model = qualified_pair
    T = np.eye(4)
    if fault == 'translation':
        T[0, 3] = .001
    else:
        angle = .01
        T[:2, :2] = [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]
    transformed = box.transform_inferred_box(model, T)
    with pytest.raises(ValueError, match='original source-frame model'):
        box.enclose_inferred_box_geometry(scene.geometry, transformed)


@pytest.mark.parametrize('kind', ['none', 'plain_dict', 'failed_result'])
def test_unqualified_inputs_cannot_opt_into_smaller_enclosure(qualified_pair, kind):
    scene, model = qualified_pair
    value = {'none': None, 'plain_dict': model.to_evidence_dict(),
        'failed_result': box.BoxInferenceResult(False, 'BAD', None, {})}[kind]
    with pytest.raises(ValueError):
        box.enclose_inferred_box_geometry(scene.geometry, value)


def test_typed_object_with_bypassed_immutable_validation_is_rechecked(qualified_pair):
    scene, model = qualified_pair
    tampered = object.__new__(box.RegularBoxInference)
    evidence = model.to_evidence_dict(); evidence['model_sha256'] = '0'*64
    object.__setattr__(tampered, '_serialized', json.dumps(evidence))
    with pytest.raises(ValueError, match='digest differs'):
        box.enclose_inferred_box_geometry(scene.geometry, tampered)


def test_producer_rechecks_complete_point_set_and_honors_conflict(qualified_pair, monkeypatch):
    scene, model = qualified_pair
    real_check = box.evaluate_box_measured_consistency
    calls = []

    def checked_all_points(actual_model, points):
        calls.append(np.asarray(points).copy())
        return real_check(actual_model, points)

    monkeypatch.setattr(box, 'evaluate_box_measured_consistency', checked_all_points)
    box.enclose_inferred_box_geometry(scene.geometry, model)
    assert len(calls) == 1
    np.testing.assert_array_equal(calls[0], scene.geometry.object_points_base)

    def conflict(actual_model, points):
        # Explicit failed-check injection verifies the helper cannot silently
        # fall back to a family-only box; it creates no qualified model.
        checked_all_points(actual_model, points)
        return box.BoxMeasuredConsistency(False, 'BOX_MEASURED_GEOMETRY_CONFLICT',
                                          .0011, 'null')

    monkeypatch.setattr(box, 'evaluate_box_measured_consistency', conflict)
    with pytest.raises(ValueError, match='measured geometry conflicts'):
        box.enclose_inferred_box_geometry(scene.geometry, model)
    np.testing.assert_array_equal(calls[-1], scene.geometry.object_points_base)


def test_generic_measured_fusion_retains_old_obb_contract(qualified_pair):
    scene, _ = qualified_pair
    geometry = replace(scene.geometry, size_xyz_m=np.array([.13, .10, .025]))
    points = geometry.object_points_base[:3]
    surface = FusedTargetSurface(scene.identity, points, np.zeros(len(points), dtype=int),
                                 (scene.stamp,))
    output, audit = enclose_registered_surface(geometry, surface, expected_identity=scene.identity)
    assert inside(corners(geometry), output) and inside(geometry.object_points_base, output)
    assert audit['schema_version'] == 2
    assert audit['policy'] == 'registered_measured_surface_tangent_enclosure_v2'
    assert audit['original_obb_preserved'] is True
