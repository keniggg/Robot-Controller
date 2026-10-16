"""Actual remote geometry path from calibrated top pixels to frozen evidence."""
from copy import deepcopy
from types import SimpleNamespace as NS
import hashlib

import numpy as np
import pytest

import test_remote_grasp6d_streaming as fixtures
from test_gen26_regular_box_inference_independent import ray_scene
from alicia_flexible_grasp.vision.target_observation import TargetObservation
from alicia_flexible_grasp.vision.rgbd_snapshot import FrozenRgbdSample
from alicia_flexible_grasp.vision.multiview_surface import SurfaceView, fused_surface_from_view
from alicia_flexible_grasp.grasp.contact_evidence import decode_contact_evidence

remote = fixtures.remote_node


def pipeline(monkeypatch, *, strict=False, width=.030):
    scene = ray_scene(width=width)
    scene.K.depth_scale = .0001
    params = {'/grasp/unknown_require_measured_bilateral_contact': strict}
    monkeypatch.setattr(remote.rospy, 'get_param', lambda name, default=None: params.get(name, default))
    monkeypatch.setattr(remote.rospy, 'get_param_cached', lambda name, default=None: params.get(name, default))
    node = remote.RemoteGrasp6DNode.__new__(remote.RemoteGrasp6DNode)
    node.tabletop_geometry_enabled = True
    node.tabletop_geometry_config = fixtures.TabletopGeometryConfig(max_candidates=16)
    node.gripper_geometry = fixtures.tabletop_gripper()
    node.gripper_tool_jaw_axis, node.gripper_tool_finger_length_axis = 'y', 'z'
    node.gripper_physical_open_width_m = .05
    node.grasp_config = dict(tool_approach_axis='z', pregrasp_distance_m=.08,
                            final_approach_offset_m=.015, lift_height_m=.05)
    node.candidate_min_downward_approach_cos = .65
    node.candidate_max_final_approach_lateral_m = .010
    node.soft_score_weights = fixtures.SoftScoreWeights()
    node.target_absolute_sanity_distance_m = .15
    node.camera_visibility_gate_enabled = False
    node.camera_visibility_diagnostic_enabled = False
    fixtures.configure_identity_handeye(node)
    node.near_field_planning_active = True
    node._stream_generation, node._near_field_phase_id = 4, 1
    node._current_stream_target_identity = lambda: scene.identity
    ys, xs = np.nonzero(scene.mask)
    bbox = (int(xs.min()), int(ys.min()), int(xs.max()-xs.min()+1), int(ys.max()-ys.min()+1))
    edge = min(bbox[0], bbox[1], scene.depth.shape[1]-bbox[0]-bbox[2],
               scene.depth.shape[0]-bbox[1]-bbox[3])
    observed = TargetObservation(scene.identity, scene.stamp, 'base_link',
        scene.geometry.object_points_base, scene.geometry.support_normal_base,
        scene.geometry.support_offset_m, bbox, scene.depth.shape, edge, 'instance_mask', 'unknown_small')
    view = SurfaceView(scene.identity, scene.stamp, observed.points_base,
        observed.support_normal_base, observed.support_offset_m, edge)
    surface = fused_surface_from_view(view)
    node._latest_target_observation = observed
    node._near_field_reference_view = view
    node._latest_registration_evidence = None
    node._active_multiview_surface = lambda: (surface, view)
    node._camera_intrinsics = lambda: scene.K
    snapshot = NS(ok=True, stamp_ns=scene.stamp, stamp_sec=scene.stamp*1e-9,
        color_bgr=np.zeros(scene.depth.shape+(3,), dtype=np.uint8),
        depth_raw=scene.depth, object_mask=scene.mask, source_mode='instance_mask',
        quality=NS(depth_repeatability_m=0.), target_identity=scene.identity,
        target_epoch=scene.identity.epoch, frame_id='camera_optical',
        sample_stamp_ns=(scene.stamp,), sample_mask_sha256=(hashlib.sha256(
            np.asarray(scene.mask > 0, dtype=np.uint8).tobytes()).hexdigest(),))
    snapshot.original_sample = FrozenRgbdSample(snapshot.color_bgr, scene.depth,
        scene.mask, bbox, None, snapshot.stamp_sec, snapshot.frame_id,
        np.zeros(6), scene.stamp, scene.identity.epoch, scene.identity)
    planning = node._prepare_regular_box_snapshot(snapshot, scene.geometry, scene.T)
    prepared = NS(geometry=planning, stamp=remote.rospy.Time(*divmod(scene.stamp, 10**9)),
                  near_field=True, snapshot=snapshot, target_observation=observed)
    return node, scene, prepared, params, surface


def test_complete_top_without_measured_sides_reaches_final_typed_contact_evidence(monkeypatch):
    node, scene, prepared, _, surface = pipeline(monkeypatch)
    original = surface.points_base.copy()
    assert node._current_inferred_box() is not None
    candidates, audit = node._generate_tabletop_candidates(prepared.geometry, prepared.snapshot)
    assert candidates, audit
    assert audit['regular_box_inference']['ok'] is True
    assert audit['regular_box_planning_enclosure']
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
    plan.candidate_source, plan.model_choice, plan.diagnostic = 'tabletop_geometry', 'unknown_tabletop', 'CONTACT_EXECUTION_PLAN'
    plan.required_open_width_m = normalized.required_open_width_m
    plan.poses = [deepcopy(getattr(sequence, stage).pose) for stage in ('pregrasp','approach','grasp','lift')]
    gate = node._registered_sequence_geometry_gate(plan, prepared, runtime, sequence)
    assert gate.ok, gate
    plan.contact_geometry_evidence = runtime['final_contact_geometry_evidence']
    evidence = decode_contact_evidence(plan, required=True)
    assert evidence['schema_version'] == 3
    assert evidence['measurement_source'] == 'regular_box_model_inference'
    assert 'measured_contact_width_m' not in evidence
    assert evidence['required_open_width_m'] == pytest.approx(
        evidence['inferred_contact_width_m']+evidence['width_uncertainty_m']+
        2*evidence['opening_fit_clearance_each_side_m'])
    np.testing.assert_array_equal(surface.points_base, original)
    np.testing.assert_array_equal(prepared.geometry.object_points_base, scene.geometry.object_points_base)


def test_reenabling_measured_requirement_restores_top_only_rejection(monkeypatch):
    node, _, prepared, _, _ = pipeline(monkeypatch, strict=True)
    assert node._current_inferred_box() is None
    candidates, audit = node._generate_tabletop_candidates(prepared.geometry, prepared.snapshot)
    assert not candidates
    assert audit['failure_code'] == 'BILATERAL_SURFACE_EVIDENCE_MISSING'


@pytest.mark.parametrize('fault', ['generation', 'phase', 'observation', 'policy'])
def test_inferred_model_never_survives_changed_source_or_policy(monkeypatch, fault):
    node, _, _, params, _ = pipeline(monkeypatch)
    assert node._current_inferred_box() is not None
    if fault == 'generation': node._stream_generation += 1
    elif fault == 'phase': node._near_field_phase_id += 1
    elif fault == 'observation': node._latest_target_observation = deepcopy(node._latest_target_observation)
    else: params['/grasp/unknown_require_measured_bilateral_contact'] = True
    assert node._current_inferred_box() is None
