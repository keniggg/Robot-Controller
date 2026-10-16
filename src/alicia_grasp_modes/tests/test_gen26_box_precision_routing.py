"""Source-bound precision routing; supplementary views never authorize contact."""
from copy import deepcopy

import numpy as np
import pytest

from test_remote_modes import configured
from test_measured_observation_planning import preflight_context
from test_gen26_regular_box_inference_independent import ray_scene, qualify
from alicia_flexible_grasp.vision.multiview_surface import SurfaceView, fused_surface_from_view
from alicia_flexible_grasp.vision.regular_box_inference import enclose_inferred_box_geometry
from alicia_flexible_grasp.grasp.tabletop_geometry_candidates import generate_tabletop_proposals
from alicia_grasp_modes.observe_return_visual import inferred_contact_precision_view_gaps


def precision_inputs(configured, monkeypatch, width=.046):
    node, ticket, snapshot, _, transform, _, _, guards = preflight_context(configured, monkeypatch)
    scene = ray_scene(width=width)
    scene.identity, scene.stamp = snapshot.target_identity, snapshot.stamp_ns
    qualified = qualify(scene)
    assert qualified.ok, qualified.audit
    geometry, _ = enclose_inferred_box_geometry(scene.geometry, qualified.model)
    view = SurfaceView(scene.identity, scene.stamp, scene.geometry.object_points_base,
        geometry.support_normal_base, geometry.support_offset_m, 12)
    surface = fused_surface_from_view(view)
    node._near_field_fused_surface, node._near_field_reference_view = surface, view
    monkeypatch.setattr(node, '_current_inferred_box', lambda *a: qualified.model)
    generated = generate_tabletop_proposals(geometry.object_points_base, geometry.center_base,
        geometry.axes_base, geometry.size_xyz_m, np.zeros(3), geometry.support_normal_base,
        node.tabletop_geometry_config, surface, node.gripper_geometry, inferred_box=qualified.model)
    diagnostics = dict(materialized_count=0, plan_phase='CONTACT_EXECUTION_PLAN',
        contact_execution_gate_deferred=False, failure_code=generated.failure_code,
        surface_evidence_rejections=deepcopy(generated.surface_evidence_rejections))
    return node, ticket, snapshot, geometry, transform, diagnostics, surface, qualified.model, generated


def test_uncertain_narrow_face_requests_targeted_view_without_claiming_missing_measurements(configured, monkeypatch):
    node, ticket, snapshot, geometry, transform, diagnostics, surface, model, generated = precision_inputs(configured, monkeypatch)
    assert not generated.proposals, diagnostics
    assert generated.failure_code == 'BOX_INFERENCE_PRECISION_VIEW_REQUIRED'
    assert node._maybe_prepare_observation_from_contact_geometry(ticket, snapshot, geometry,
        transform, (), diagnostics)
    assert node._observe_return_phase == 'observation_prefix'
    assert node._observe_return_trigger == 'BOX_INFERENCE_PRECISION_VIEW_REQUIRED'
    assert node._unknown_observation_planning_active()
    assert node._joint_space_observation_active()
    proof = node._observe_return_trigger_evidence
    assert proof['policy'] == 'current_request_box_precision_view_preflight'
    assert 'measured_missing_sides' not in proof
    gap = proof['targeted_precision_views'][0]
    assert gap['measured_side_surfaces'] is False
    assert gap['model_sha256'] == model.model_sha256
    assert gap['source_view_stamps_ns'] == [snapshot.stamp_ns]
    assert gap['targeted_view']['target_edges_or_corners_base']
    assert gap['contact_or_motion_authorized'] is False
    assert proof['contact_search_completed'] is False


@pytest.mark.parametrize('fault', ['model_hash', 'wrong_source', 'wrong_track', 'oversize'])
def test_precision_view_request_requires_current_feasible_nominal_direction(configured, monkeypatch, fault):
    _, _, snapshot, _, transform, diagnostics, surface, model, _ = precision_inputs(configured, monkeypatch)
    if fault == 'model_hash':
        for row in diagnostics['surface_evidence_rejections']: row['model_sha256'] = '0'*64
    elif fault == 'wrong_source':
        from dataclasses import replace
        surface = replace(surface, view_stamps_ns=(snapshot.stamp_ns+1,))
    elif fault == 'wrong_track':
        from dataclasses import replace
        from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
        surface = replace(surface, identity=TargetTrackIdentity(snapshot.target_identity.epoch, 'other'))
    else:
        for row in diagnostics['surface_evidence_rejections']:
            row['failure_code'] = 'NO_FIT_DIRECTION'
    assert not inferred_contact_precision_view_gaps(diagnostics, model, surface, transform[:3, 3])


def test_ample_width_margin_keeps_qualified_initial_contact_route(configured, monkeypatch):
    node, ticket, snapshot, geometry, transform, diagnostics, _, _, generated = precision_inputs(
        configured, monkeypatch, width=.030)
    assert generated.proposals
    # The real qualified model generation supplies proposals; no special
    # strategy flag or fictitious observation session is needed to retain it.
    assert not node._maybe_prepare_observation_from_contact_geometry(ticket, snapshot, geometry,
        transform, generated.proposals, dict(diagnostics, materialized_count=len(generated.proposals)))
    assert node._observe_return_phase == 'initial_contact'
    assert node._observe_return_session is None
