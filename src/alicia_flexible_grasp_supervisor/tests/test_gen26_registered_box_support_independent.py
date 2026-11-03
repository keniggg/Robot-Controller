"""Accepted registration preserves the distinction between model and clearance planes."""
from copy import deepcopy
from types import SimpleNamespace as NS

import numpy as np
import pytest

from test_gen26_inferred_box_remote_pipeline import pipeline, remote, fixtures
from alicia_flexible_grasp.grasp.contact_evidence import decode_contact_evidence
from alicia_flexible_grasp.vision.multiview_surface import (
    SurfaceView, append_registered_view, fused_surface_from_view)


def registered_pipeline(monkeypatch, *, normal_angle_deg=0., height_shift_m=0., yaw_deg=0.):
    node, scene, prepared, params, _ = pipeline(monkeypatch)
    points = scene.geometry.object_points_base
    angle = np.radians(normal_angle_deg)
    normal = np.asarray([np.sin(angle), 0., np.cos(angle)])
    yaw = np.radians(yaw_deg)
    cross = np.array([[0., -normal[2], normal[1]],
                      [normal[2], 0., -normal[0]], [-normal[1], normal[0], 0.]])
    rotation = np.eye(3)+np.sin(yaw)*cross+(1.-np.cos(yaw))*(cross@cross)
    reference = SurfaceView(scene.identity, scene.stamp-1,
        points@rotation.T+np.asarray([0., 0., height_shift_m]), normal,
        -height_shift_m*normal[2], 12)
    moving = SurfaceView(scene.identity, scene.stamp, points,
        scene.geometry.support_normal_base, scene.geometry.support_offset_m, 12)
    result, surface = append_registered_view(fused_surface_from_view(reference), reference, moving)
    assert result.ok, result
    node._near_field_reference_view = reference
    node._latest_registration_evidence = remote.RegistrationEvidence(
        identity=scene.identity, stamp_ns=scene.stamp, result=result, source_clipped=False)
    node._active_multiview_surface = lambda: (surface, reference)
    return node, scene, prepared, params, result


@pytest.mark.parametrize('angle', [.1, .5])
def test_accepted_reference_plane_variation_does_not_replace_current_box_plane(monkeypatch, angle):
    node, _, _, _, registration = registered_pipeline(monkeypatch, normal_angle_deg=angle)
    assert registration.support_normal_angle_deg == pytest.approx(angle)
    model = node._current_inferred_box()
    rotation = np.column_stack((model.axes_base[:, 0], -model.axes_base[:, 1],
                               -model.axes_base[:, 2]))
    _, contact, bounds, _ = node._current_bilateral_surface_measurement(
        model.center_base, rotation)
    assert contact.ok, contact
    assert bounds is not None


def final_registered_plan(monkeypatch, *, normal_angle_deg=0., yaw_deg=0.):
    node, scene, prepared, _, registration = registered_pipeline(monkeypatch,
        height_shift_m=.0005, normal_angle_deg=normal_angle_deg, yaw_deg=yaw_deg)
    assert registration.transform_base[2, 3] == pytest.approx(.0005, abs=1e-9)
    candidates, audit = node._generate_tabletop_candidates(prepared.geometry, prepared.snapshot)
    assert candidates, audit
    normalized = None
    for candidate in candidates:
        try:
            normalized = node._normalize_tabletop_candidate(prepared, candidate)
            break
        except remote.CandidateContractError:
            pass
    assert normalized is not None, audit
    sequence = normalized.grasp_sequence
    _, profile = node._make_contact_sequence(sequence.grasp, prepared.geometry,
        insertion_axis_base=normalized.insertion_axis_base, snapshot=prepared.snapshot)
    runtime = dict(prepared=prepared, grasp_pose=sequence.grasp, adaptive_stage_profile=profile,
        scored_candidate=NS(payload=normalized, stable_candidate=NS(
            center_base_xyz=normalized.contact_center_base,
            required_open_width_m=normalized.required_open_width_m)))
    plan = fixtures.bound_surface_plan(prepared.target_observation)
    plan.header.frame_id = 'base_link'
    plan.object_geometry = remote.geometry_estimate_to_message(prepared.geometry,
        snapshot=prepared.snapshot, stamp=prepared.stamp, target_identity=scene.identity)
    plan.candidate_source, plan.model_choice, plan.diagnostic = (
        'tabletop_geometry', 'unknown_tabletop', 'CONTACT_EXECUTION_PLAN')
    plan.required_open_width_m = normalized.required_open_width_m
    plan.poses = [deepcopy(getattr(sequence, stage).pose)
                  for stage in ('pregrasp', 'approach', 'grasp', 'lift')]
    original_support = (deepcopy(plan.object_geometry.support_normal_base),
                        plan.object_geometry.support_offset_m)
    assert node._apply_registration_evidence_to_plan(plan, prepared.target_observation)
    assert plan.refinement_status == 'VALID_3D'
    assert plan.object_geometry.support_normal_base == original_support[0]
    assert plan.object_geometry.support_offset_m == original_support[1]
    corrected = node._rich_plan_execution_sequence(plan)
    gate = node._registered_sequence_geometry_gate(plan, prepared, runtime, corrected)
    assert gate.ok, gate
    plan.contact_geometry_evidence = runtime['final_contact_geometry_evidence']
    evidence = decode_contact_evidence(plan, required=True)
    model = evidence['box_model_evidence']
    assert model['support_offset_m'] == pytest.approx(-.0005, abs=1e-9)
    assert model['source_support_offset_m'] == original_support[1]
    np.testing.assert_allclose(model['registration_transform'], registration.transform_base)
    return plan, evidence


@pytest.mark.parametrize('normal_angle_deg,yaw_deg', [(0., 0.), (.5, 0.), (.5, 1.)])
def test_accepted_registration_keeps_original_clearance_plane_in_final_plan(
        monkeypatch, normal_angle_deg, yaw_deg):
    final_registered_plan(monkeypatch, normal_angle_deg=normal_angle_deg, yaw_deg=yaw_deg)


@pytest.mark.parametrize('fault', ['normal', 'offset'])
def test_registered_model_still_rejects_contradictory_source_clearance_plane(monkeypatch, fault):
    plan, _ = final_registered_plan(monkeypatch, normal_angle_deg=.5, yaw_deg=1.)
    if fault == 'offset':
        plan.object_geometry.support_offset_m += .001
    else:
        normal = plan.object_geometry.support_normal_base
        normal.x, normal.z = np.sin(.001), np.cos(.001)
    with pytest.raises(ValueError, match='source support differs'):
        decode_contact_evidence(plan, required=True)


def test_physical_gate_binds_source_plane_even_when_model_plane_is_registered(monkeypatch):
    from alicia_flexible_grasp.grasp.gripper_geometry import (
        evaluate_inferred_box_contact, _inferred_gate_values)
    from alicia_flexible_grasp.vision.regular_box_inference import RegularBoxInference
    from tf.transformations import quaternion_matrix
    plan, frozen = final_registered_plan(monkeypatch, normal_angle_deg=.5, yaw_deg=1.)
    model = RegularBoxInference.from_evidence_dict(frozen['box_model_evidence'])
    rotation = quaternion_matrix(frozen['grasp_tool_pose_base'][3:])[:3, :3]
    gripper = fixtures.tabletop_gripper()
    center, jaw, insertion = frozen['contact_center_base'], rotation[:, 1], rotation[:, 2]
    fit = frozen['opening_fit_clearance_each_side_m']
    contact = evaluate_inferred_box_contact(model, center, jaw, insertion, gripper,
        opening_fit_clearance_each_side_m=fit)
    assert contact.ok
    assert not np.allclose(model.support_normal_base, model.source_support_normal_base,
                           atol=1e-6, rtol=0.)
    result = _inferred_gate_values(contact, center, jaw, insertion, gripper,
        model.source_support_normal_base, fit, contact.contact_height_bounds_m)
    assert result[0] == pytest.approx(frozen['required_open_width_m'])
    with pytest.raises(ValueError, match='source and physical clearance support differ'):
        _inferred_gate_values(contact, center, jaw, insertion, gripper,
            model.support_normal_base, fit, contact.contact_height_bounds_m)
