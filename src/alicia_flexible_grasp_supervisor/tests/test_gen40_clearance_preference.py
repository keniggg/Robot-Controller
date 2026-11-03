"""Clearance preference crosses the real contact consumers, never real motion.

All models are produced from calibrated synthetic RGB-D by the production
qualifiers. Aperture positives keep the complete model family, fit and source;
insufficient physical opening remains a negative with either legacy flag.
"""
import json
from types import SimpleNamespace as NS

import numpy as np
import pytest
from tf.transformations import quaternion_from_matrix

from test_gen26_regular_box_inference_independent import ray_scene, qualify
from test_gen26_contact_provenance_independent import physical_gripper
from test_gen31_outer_rgb_box import visual_edge_gap_scene, qualify_visual_scene
from test_gen31_oblique_box_independent import oblique_box_scene
from test_gen38_cuboid_model_contract import _produce
from alicia_flexible_grasp.grasp.contact_evidence import encode_inferred_contact_evidence
from alicia_flexible_grasp.grasp.contact_evidence_wire import decode_contact_evidence_wire
from alicia_flexible_grasp.grasp.gripper_geometry import (
    evaluate_inferred_box_contact, _inferred_gate_values)
from alicia_flexible_grasp.grasp.measured_contact import evaluate_measured_aperture
from alicia_flexible_grasp.grasp.tabletop_geometry_candidates import (
    TabletopGeometryConfig, generate_tabletop_proposals,
    solve_tool0_translation_for_support_clearance)
from alicia_flexible_grasp.vision.regular_box_inference import enclose_inferred_box_geometry


@pytest.fixture(scope='module', params=(1, 2, 3))
def qualified_source(request):
    if request.param == 1:
        scene = ray_scene(width=.030)
        result = qualify(scene)
    elif request.param == 2:
        scene = visual_edge_gap_scene()
        result = qualify_visual_scene(scene)
    else:
        scene = oblique_box_scene()
        result = _produce(scene)
    assert result.ok, (result.code, result.audit)
    assert result.model.schema_version == request.param
    return scene, result.model


def projected(model, gap=.05, enforce=True):
    return evaluate_inferred_box_contact(model, model.center_base,
        model.axes_base[:, 1], -model.support_normal_base,
        physical_gripper(gap), opening_fit_clearance_each_side_m=.0005,
        enforce_precision_margin=enforce)


def pose_for_rotation(rotation):
    transform = np.eye(4)
    transform[:3, :3] = rotation
    q = quaternion_from_matrix(transform)
    return NS(position=NS(x=0., y=0., z=.08),
              orientation=NS(x=q[0], y=q[1], z=q[2], w=q[3]))


@pytest.mark.parametrize('remaining,accepted', [
    (-.0001, False), (0., True), (.0003, True), (.0021, True)])
@pytest.mark.parametrize('legacy_enforce', [False, True])
def test_all_schemas_use_full_physical_budget_and_report_preference(
        qualified_source, remaining, accepted, legacy_enforce):
    scene, model = qualified_source
    frozen_model = model.to_evidence_dict()
    original_points = scene.geometry.object_points_base.copy()
    full = projected(model)
    assert full.ok, full
    required = full.targeted_view['required_open_upper_m']
    gap = required + remaining
    assert 0. < gap < .05
    evidence = projected(model, gap, legacy_enforce)
    assert evidence.ok is accepted, evidence
    assert evidence.inferred_width_m == full.inferred_width_m
    assert evidence.width_uncertainty_m == full.width_uncertainty_m
    assert evidence.targeted_view['required_open_upper_m'] == required
    assert evidence.targeted_view['precision_reserve_m'] == .002
    assert evidence.targeted_view['precision_reserve_is_hard_requirement'] is False
    assert evidence.targeted_view['precision_preference_met'] is (remaining >= .002)
    generation = generate_tabletop_proposals(scene.geometry.object_points_base,
        scene.geometry.center_base, scene.geometry.axes_base, scene.geometry.size_xyz_m,
        np.zeros(3), model.support_normal_base,
        TabletopGeometryConfig(max_inner_gap_m=gap,
            opening_fit_clearance_each_side_m=.0005),
        finger_geometry=physical_gripper(), inferred_box=model)
    assert generation.ok is accepted, generation
    if accepted:
        proposal = generation.proposals[0]
        assert proposal.required_open_width_m == pytest.approx(required, abs=1e-12)
        assert proposal.audit['targeted_view']['precision_preference_met'] is (remaining >= .002)
        # The typed-candidate check and canonical contact encoder use their
        # normal default policy; neither can reintroduce the old margin veto.
        typed, _ = _inferred_gate_values(evidence, model.center_base,
            model.axes_base[:, 1], -model.support_normal_base,
            physical_gripper(), model.source_support_normal_base,
            .0005, evidence.contact_height_bounds_m)
        assert typed == pytest.approx(required, abs=1e-12)
        rotation = np.column_stack((-model.axes_base[:, 0], model.axes_base[:, 1],
                                    -model.support_normal_base))
        encoded = encode_inferred_contact_evidence(
            contact_center_base=model.center_base,
            contact_height_axis_base=evidence.contact_height_axis_base,
            contact_height_bounds_m=evidence.contact_height_bounds_m,
            minimum_contact_overlap_m=.002, grasp_pose=pose_for_rotation(rotation),
            inferred_contact_width_m=evidence.inferred_width_m,
            width_uncertainty_m=evidence.width_uncertainty_m,
            opening_fit_clearance_each_side_m=.0005, required_open_width_m=required,
            box_model_evidence=evidence.model_evidence)
        assert json.loads(decode_contact_evidence_wire(encoded))['width_uncertainty_m'] == evidence.width_uncertainty_m
    else:
        assert evidence.code == generation.failure_code == 'BOX_INFERENCE_PRECISION_VIEW_REQUIRED'
    assert model.to_evidence_dict() == frozen_model
    np.testing.assert_array_equal(scene.geometry.object_points_base, original_points)


@pytest.mark.parametrize('invalid', [None, 1, 'true'])
def test_legacy_flag_still_requires_a_real_boolean(qualified_source, invalid):
    _, model = qualified_source
    evidence = projected(model, enforce=invalid)
    assert not evidence.ok and evidence.code == 'BOX_INFERENCE_INPUT_INVALID'


def measured_arguments(index):
    scene = ray_scene(width=.030)
    result = qualify(scene)
    assert result.ok, result.audit
    model = result.model
    union, _ = enclose_inferred_box_geometry(scene.geometry, model)
    gripper = physical_gripper()
    rotation = np.column_stack((-model.axes_base[:, 0], model.axes_base[:, 1],
                                -model.support_normal_base))
    contact = np.eye(4)
    contact[:3, :3] = rotation
    contact[:3, 3] = solve_tool0_translation_for_support_clearance(rotation,
        model.center_base, np.zeros(3), model.support_normal_base, .003, gripper)
    stage = contact.copy()
    stage[:3, 3] += model.support_normal_base * {0: .08, 1: .02, 2: 0.}[index]
    return dict(gripper=gripper, actual_T_base_tool0=stage.copy(),
        planned_stage_T_base_tool0=stage, planned_contact_T_base_tool0=contact,
        contact_center_base=model.center_base, obb_center_base=union.center_base,
        R_base_obb=union.axes_base, obb_size_xyz_m=union.size_xyz_m,
        support_normal_base=model.support_normal_base, support_offset_m=model.support_offset_m,
        opening_width_m=.05, opening_fit_clearance_each_side_m=.0005,
        stage_reference_pose_index=index, inferred_model=model)


@pytest.mark.parametrize('index', [0, 1, 2])
@pytest.mark.parametrize('remaining,accepted', [(-.0001, False), (.0003, True)])
def test_measured_stages_require_actual_union_and_model_fit_without_extra_veto(
        index, remaining, accepted):
    kw = measured_arguments(index)
    original = evaluate_measured_aperture(**kw)
    assert original['ok'], original
    required = max(original['union_required_open_width_m'],
                   original['inferred_required_open_width_m'])
    kw['opening_width_m'] = required + remaining
    assert 0. < kw['opening_width_m'] < .05
    result = evaluate_measured_aperture(**kw)
    assert result['ok'] is accepted, result
    assert result['precision_reserve_m'] == .002
    assert result['precision_reserve_is_hard_requirement'] is False
    assert result['precision_preference_met'] is False
    assert result['effective_required_open_width_m'] == pytest.approx(required, abs=1e-12)
    assert result['required_open_width_m'] == pytest.approx(required, abs=1e-12)
    assert result['continuous_tracking_tube_checked'] is False
    if not accepted:
        assert result['code'] == 'MEASURED_APERTURE_INSUFFICIENT'


def test_model_fit_does_not_hide_an_asymmetric_union_that_exceeds_physical_gap():
    kw = measured_arguments(2)
    kw['obb_center_base'] = np.asarray(kw['obb_center_base']) + kw['R_base_obb'][:, 1]*.009
    result = evaluate_measured_aperture(**kw)
    assert result['inferred_required_open_width_m'] < .05
    assert result['union_required_open_width_m'] > .05
    assert not result['ok'] and result['code'] == 'MEASURED_APERTURE_INSUFFICIENT'


def test_preference_change_keeps_support_collision_rejection():
    kw = measured_arguments(2)
    kw['actual_T_base_tool0'] = kw['actual_T_base_tool0'].copy()
    kw['actual_T_base_tool0'][:3, 3] -= kw['support_normal_base']*.004
    result = evaluate_measured_aperture(**kw)
    assert not result['ok'] and result['code'] == 'MEASURED_APERTURE_CAD_COLLISION'
    assert result['actual_current_minimum_support_clearance_m'] < .003
