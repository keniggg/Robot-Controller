"""Unresolved current RGB box hypotheses can guide views, never close the jaws."""
from copy import deepcopy
from dataclasses import replace
import hashlib

import numpy as np
import pytest

from test_remote_modes import configured, base
from test_measured_observation_planning import preflight_context
from test_gen31_outer_rgb_box import precision_interior_scene
from alicia_flexible_grasp.vision.target_observation import TargetObservation
from alicia_flexible_grasp.vision.rgbd_snapshot import FrozenRgbdSample
from alicia_flexible_grasp.vision.multiview_surface import SurfaceView, fused_surface_from_view
from alicia_flexible_grasp.grasp.tabletop_geometry_candidates import generate_tabletop_proposals
from alicia_grasp_modes.observe_return_visual import unqualified_box_precision_view_gaps


def current_candidate(configured, monkeypatch):
    node, ticket, snapshot, _, _, _, _, _ = preflight_context(configured, monkeypatch)
    monkeypatch.setattr(base.rospy, 'get_param_cached', base.rospy.get_param)
    scene = precision_interior_scene()
    scene.identity, scene.stamp = snapshot.target_identity, snapshot.stamp_ns
    scene.K.depth_scale = .0001
    ys, xs = np.nonzero(scene.mask)
    bbox = (int(xs.min()), int(ys.min()), int(xs.max()-xs.min()+1), int(ys.max()-ys.min()+1))
    edge = min(bbox[0], bbox[1], scene.depth.shape[1]-bbox[0]-bbox[2], scene.depth.shape[0]-bbox[1]-bbox[3])
    observation = TargetObservation(scene.identity, scene.stamp, 'base_link',
        scene.geometry.object_points_base, scene.geometry.support_normal_base,
        scene.geometry.support_offset_m, bbox, scene.depth.shape, edge, 'instance_mask', 'unknown_small')
    view = SurfaceView(scene.identity, scene.stamp, observation.points_base,
        observation.support_normal_base, observation.support_offset_m, edge)
    surface = fused_surface_from_view(view)
    node._latest_target_observation = observation
    node._near_field_fused_surface, node._near_field_reference_view = surface, view
    node._latest_registration_evidence = None
    monkeypatch.setattr(node, '_camera_intrinsics', lambda: scene.K)
    snapshot.ok, snapshot.source_mode = True, 'instance_mask'
    snapshot.stamp_sec, snapshot.frame_id = scene.stamp*1e-9, 'camera_optical'
    snapshot.target_epoch = scene.identity.epoch
    snapshot.color_bgr, snapshot.depth_raw, snapshot.object_mask = scene.color, scene.depth, scene.mask
    snapshot.sample_stamp_ns = (scene.stamp,)
    snapshot.sample_mask_sha256 = (hashlib.sha256(np.asarray(scene.mask > 0, dtype=np.uint8).tobytes()).hexdigest(),)
    snapshot.original_sample = FrozenRgbdSample(scene.color, scene.depth, scene.mask,
        bbox, None, snapshot.stamp_sec, snapshot.frame_id, np.zeros(6), scene.stamp,
        scene.identity.epoch, scene.identity)
    geometry = node._prepare_regular_box_snapshot(snapshot, scene.geometry, scene.T)
    result = node._regular_box_snapshot['result']
    assert not result.ok and result.model is None and result.code == 'BOX_INFERENCE_PRECISION_VIEW_REQUIRED', result.audit
    assert geometry is scene.geometry and node._regular_box_snapshot['enclosure'] is None
    assert node._current_inferred_box() is None
    generated = generate_tabletop_proposals(geometry.object_points_base, geometry.center_base,
        geometry.axes_base, geometry.size_xyz_m, np.zeros(3), geometry.support_normal_base,
        node.tabletop_geometry_config, surface, node.gripper_geometry, inferred_box=None)
    assert not generated.proposals
    diagnostics = dict(materialized_count=0, plan_phase='CONTACT_EXECUTION_PLAN',
        contact_execution_gate_deferred=False, failure_code=generated.failure_code,
        surface_evidence_rejections=deepcopy(generated.surface_evidence_rejections))
    return node, ticket, snapshot, geometry, scene.T, diagnostics, surface, scene


def test_unknown_interior_routes_to_source_bound_precision_view_without_grasp_model(configured, monkeypatch):
    node, ticket, snapshot, geometry, transform, diagnostics, surface, _ = current_candidate(configured, monkeypatch)
    measured = surface.points_base.copy()
    assert node._maybe_prepare_observation_from_contact_geometry(ticket, snapshot, geometry, transform, (), diagnostics)
    proof = node._observe_return_trigger_evidence
    assert proof['policy'] == 'current_request_box_precision_view_preflight'
    assert proof['contact_or_motion_authorized'] is False
    assert proof['contact_search_completed'] is False
    assert 'measured_missing_sides' not in proof
    gap = proof['targeted_precision_views'][0]
    assert gap['view_deficit_kind'] == 'unqualified_box_depth_and_edge_precision'
    assert 'model_sha256' not in gap
    assert gap['precision_view_candidate']['motion_authorized'] is False
    assert gap['targeted_view']['missing_region_pixels_yx']
    assert gap['source_view_stamps_ns'] == [snapshot.stamp_ns]
    assert gap['required_open_upper_m'] > gap['required_open_width_m']
    assert node._unknown_observation_planning_active()
    assert node._joint_space_observation_active()
    assert node._current_inferred_box() is None
    assert not any(plan.valid for plan in node.rich_plan_pub.messages)
    np.testing.assert_array_equal(surface.points_base, measured)


@pytest.mark.parametrize('fault', ['depth', 'color', 'mask', 'intrinsics', 'transform',
    'geometry', 'original_stamp', 'generation', 'phase', 'observation', 'strict'])
def test_candidate_accessor_requires_exact_current_source(configured, monkeypatch, fault):
    node, _, snapshot, geometry, transform, _, _, scene = current_candidate(configured, monkeypatch)
    assert node._current_box_precision_candidate(snapshot, geometry, transform)
    if fault in ('depth', 'color', 'mask'):
        field = dict(depth='depth_raw', color='color_bgr', mask='object_mask')[fault]
        data = np.array(getattr(snapshot.original_sample, field), copy=True)
        data.flat[0] = 255 if fault == 'mask' else data.flat[0]+1
        snapshot.original_sample = replace(snapshot.original_sample, **{field: data})
    elif fault == 'intrinsics': scene.K.fx += 1.
    elif fault == 'transform': transform = transform.copy(); transform[0, 3] += .001
    elif fault == 'geometry': geometry = replace(geometry, object_points_base=geometry.object_points_base[:-1])
    elif fault == 'original_stamp': snapshot.original_sample = replace(snapshot.original_sample, stamp_ns=snapshot.stamp_ns+1)
    elif fault == 'generation': node._stream_generation += 1
    elif fault == 'phase': node._near_field_phase_id += 1
    elif fault == 'observation': node._latest_target_observation = deepcopy(node._latest_target_observation)
    else:
        import test_remote_grasp6d_streaming as fixtures
        monkeypatch.setattr(fixtures.remote_node.rospy, 'get_param_cached', lambda key, default=None: True)
    assert node._current_box_precision_candidate(snapshot, geometry, transform) is None


@pytest.mark.parametrize('fault', ['wrong_surface_stamp', 'wrong_surface_identity', 'no_nominal_fit', 'no_hardware_fit', 'bad_camera'])
def test_precision_observation_still_needs_feasible_current_target(configured, monkeypatch, fault):
    node, _, snapshot, geometry, transform, _, surface, _ = current_candidate(configured, monkeypatch)
    candidate = node._current_box_precision_candidate(snapshot, geometry, transform)
    config, gripper, camera = node.tabletop_geometry_config, node.gripper_geometry, transform[:3, 3]
    if fault == 'wrong_surface_stamp': surface = replace(surface, view_stamps_ns=(snapshot.stamp_ns+1,))
    elif fault == 'wrong_surface_identity': surface = replace(surface, identity=replace(surface.identity, track_id='other'))
    elif fault == 'no_nominal_fit': config = replace(config, max_inner_gap_m=.02)
    elif fault == 'no_hardware_fit': gripper = replace(gripper, max_inner_gap_m=.02)
    else: camera = np.full(3, np.nan)
    assert not unqualified_box_precision_view_gaps(candidate, surface, camera, config, gripper)


def test_existing_contact_proposal_precedes_precision_candidate(configured, monkeypatch):
    node, ticket, snapshot, geometry, transform, diagnostics, _, _ = current_candidate(configured, monkeypatch)
    assert not node._maybe_prepare_observation_from_contact_geometry(ticket, snapshot, geometry,
        transform, (object(),), dict(diagnostics, materialized_count=1))
    assert node._observe_return_phase == 'initial_contact'
    assert node._observe_return_session is None
