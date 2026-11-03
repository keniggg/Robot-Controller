"""Qualified RGB intervals survive aperture, collision and execution consumers.

Every model comes from calibrated pixels through the production qualifier.
The configured extra clearance is a preference. Complete width uncertainty,
physical opening and actual center error remain mandatory.
"""
from dataclasses import replace
from itertools import product

import numpy as np
import pytest

from test_gen31_outer_rgb_box import visual_edge_gap_scene, qualify_visual_scene
from test_gen26_regular_box_inference_independent import ray_scene, qualify
import test_gen26_contact_provenance_independent as downstream
from alicia_flexible_grasp.grasp.gripper_geometry import (
    evaluate_inferred_box_contact, inferred_box_precision_reserve)
from alicia_flexible_grasp.grasp.tabletop_geometry_candidates import (
    TabletopGeometryConfig, generate_tabletop_proposals)
from alicia_flexible_grasp.vision.regular_box_inference import (
    RegularBoxInference, enclose_inferred_box_geometry, inferred_box_enclosure_vertices)


@pytest.fixture(scope='module')
def rgb_box():
    scene = visual_edge_gap_scene()
    result = qualify_visual_scene(scene)
    assert result.ok, (result.code, result.audit)
    assert result.model.schema_version == 2
    return scene, result.model


def contact(model, *, gap=.05, center=None, jaw=None):
    return evaluate_inferred_box_contact(model,
        model.center_base if center is None else center,
        model.axes_base[:, 1] if jaw is None else jaw,
        -model.support_normal_base, downstream.physical_gripper(gap),
        opening_fit_clearance_each_side_m=.002)


def proposals(scene, model, configured_gap):
    geometry, audit = enclose_inferred_box_geometry(scene.geometry, model)
    result = generate_tabletop_proposals(geometry.object_points_base,
        geometry.center_base, geometry.axes_base, geometry.size_xyz_m,
        np.zeros(3), model.support_normal_base,
        TabletopGeometryConfig(max_inner_gap_m=configured_gap),
        finger_geometry=downstream.physical_gripper(), inferred_box=model)
    return result, geometry, audit


@pytest.mark.parametrize('remaining,accepted', [(.0021, True), (.0019, True), (.0005, True), (-.0001, False)])
def test_rgb_candidate_and_final_contact_agree_at_physical_opening_boundary(rgb_box, remaining, accepted):
    scene, model = rgb_box
    full = contact(model)
    assert full.ok, full
    required = full.targeted_view['required_open_upper_m']
    gap = required + remaining
    assert 0. < gap < .05
    assert full.width_uncertainty_m > .0021
    final = contact(model, gap=gap)
    generated, _, _ = proposals(scene, model, gap)
    assert final.ok is accepted, final
    assert generated.ok is accepted, generated
    assert final.targeted_view['precision_reserve_m'] == pytest.approx(.002)
    assert final.targeted_view['precision_reserve_is_hard_requirement'] is False
    assert final.targeted_view['precision_preference_met'] is (remaining >= .002)
    if accepted:
        # Deliberately less margin than the full interval: the extra copy
        # previously rejected this physically fitting, source-qualified box.
        assert final.targeted_view['aperture_margin_m'] < final.width_uncertainty_m
        assert generated.proposals[0].required_open_width_m == pytest.approx(required)
    else:
        assert final.code == generated.failure_code == 'BOX_INFERENCE_PRECISION_VIEW_REQUIRED'


@pytest.mark.parametrize('delta,accepted', [(-.0001, False), (.0001, True)])
def test_complete_depth_v1_keeps_configured_preference_and_physical_fit(delta, accepted):
    scene = ray_scene(width=.030)
    model = qualify(scene).model
    assert model.schema_version == 1
    full = contact(model)
    assert full.ok
    reserve = .002
    gap = full.targeted_view['required_open_upper_m'] + delta
    assert gap < .05
    result = contact(model, gap=gap)
    assert result.ok is accepted
    assert result.targeted_view['precision_reserve_m'] == pytest.approx(reserve)
    assert result.targeted_view['precision_reserve_source'] == 'configured_jaw_clearance_after_width_uncertainty_once'


def test_actual_center_displacement_still_consumes_both_sides_of_opening(rgb_box):
    _, model = rgb_box
    aligned = contact(model)
    gap = aligned.targeted_view['required_open_upper_m'] + .0003
    centered = contact(model, gap=gap)
    shifted = contact(model, gap=gap, center=model.center_base + model.axes_base[:, 1]*.0002)
    assert centered.ok
    assert not shifted.ok and shifted.code == 'BOX_INFERENCE_PRECISION_VIEW_REQUIRED'
    assert shifted.width_uncertainty_m - centered.width_uncertainty_m == pytest.approx(.0004)
    assert shifted.targeted_view['commanded_center_jaw_offset_m'] == pytest.approx(.0002)
    assert shifted.targeted_view['precision_reserve_m'] == pytest.approx(.002)


def test_actual_yaw_and_hardware_cap_remain_active(rgb_box):
    _, model = rgb_box
    angle = np.radians(7.)
    jaw = model.axes_base @ np.array([np.sin(angle), np.cos(angle), 0.])
    aligned, rotated = contact(model), contact(model, jaw=jaw)
    assert rotated.targeted_view['required_open_upper_m'] > aligned.targeted_view['required_open_upper_m']
    wide = contact(model, gap=.08, jaw=model.axes_base[:, 0])
    assert not wide.ok and wide.code == 'NO_FIT_DIRECTION'
    assert wide.targeted_view['physical_opening_m'] == .05


def test_joint_uncertainty_and_all_measured_points_are_inside_collision_enclosure(rgb_box):
    scene, model = rgb_box
    _, envelope, audit = proposals(scene, model, .05)
    np.testing.assert_array_equal(envelope.object_points_base, scene.geometry.object_points_base)
    assert audit['policy'] == 'source_bound_measured_points_and_inferred_box_uncertainty'
    assert audit['measured_points_unchanged'] is True
    assert audit['source_geometry_sha256'] == model.source['geometry_sha256']
    assert audit['producer_verified_all_points'] is True
    assert audit['all_measured_points_contained'] is True
    assert audit['full_family_contained'] is True
    assert audit['original_obb_used_as_occupancy'] is False
    assert audit['consumer_independent_measured_point_verification'] is False
    points = list(scene.geometry.object_points_base)
    points.extend(inferred_box_enclosure_vertices(model))
    for yaw in np.linspace(-model.empirical_yaw_uncertainty_rad, model.empirical_yaw_uncertainty_rad, 31):
        R = np.array([[np.cos(yaw), -np.sin(yaw), 0.], [np.sin(yaw), np.cos(yaw), 0.], [0., 0., 1.]])
        for signs in product((-1., 1.), repeat=3):
            corner = R @ (np.asarray(signs)*model.size_xyz_m/2.)
            corner[:2] += np.asarray(signs[:2])*model.empirical_edge_uncertainty_m
            if signs[2] > 0:
                corner[2] += model.empirical_height_uncertainty_m
            points.append(model.center_base + model.axes_base @ corner)
    local = (np.asarray(points)-envelope.center_base) @ envelope.axes_base
    assert np.all(np.abs(local) <= envelope.size_xyz_m/2. + 1e-10)


@pytest.mark.parametrize('fault', ['drop_interval', 'change_policy', 'fake_v2_from_v1'])
def test_rehashed_claim_cannot_enable_reduced_reserve_without_valid_interval(rgb_box, fault):
    _, model = rgb_box
    payload = model.to_evidence_dict()
    if fault == 'drop_interval':
        del payload['qualification']['outer_rgb_interval']
    elif fault == 'change_policy':
        payload['qualification']['outer_rgb_interval']['policy'] = 'unqualified_rectangle'
    else:
        payload = qualify(ray_scene()).model.to_evidence_dict()
        payload['schema_version'] = 2
        payload['qualification']['contour_source'] = 'rgb_outer_joint_interval_on_measured_top_plane'
    downstream.rehash_model(payload)
    with pytest.raises(ValueError):
        RegularBoxInference.from_evidence_dict(payload)
    with pytest.raises(ValueError):
        inferred_box_precision_reserve(payload, .004, downstream.physical_gripper())


@pytest.fixture
def rgb_execution_fixture(rgb_box, monkeypatch):
    scene, model = rgb_box
    rotation = np.column_stack((model.axes_base[:, 0], -model.axes_base[:, 1], -model.axes_base[:, 2]))
    evidence = contact(model, jaw=rotation[:, 1])
    assert evidence.ok
    monkeypatch.setattr(downstream, 'contact_fixture', lambda: (scene, model, rotation, evidence))


def test_rgb_interval_survives_ros_transport_and_final_execution_gate(rgb_execution_fixture):
    downstream.test_inferred_sides_remain_explicit_across_ros_transport_and_execution_report()


def test_rgb_interval_preserves_nominal_fixed_close_contract(rgb_execution_fixture):
    downstream.test_fixed_close_uses_nominal_width_not_inferred_upper_opening_as_contact_proof()


@pytest.mark.parametrize('fault', ['shrink', 'shift', 'rotate'])
def test_rgb_interval_final_collision_geometry_cannot_drop_uncertainty(rgb_execution_fixture, fault):
    downstream.test_rehashed_final_collision_enclosure_cannot_drop_inferred_box_uncertainty(fault)
