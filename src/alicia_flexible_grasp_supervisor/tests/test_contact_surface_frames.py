"""Registration must not mix snapshot poses with reference-frame points."""
from dataclasses import replace
from types import SimpleNamespace as NS

import numpy as np
import pytest

import test_remote_grasp6d_streaming as f

remote = f.remote_node


def node_with_correction(roll_deg=0., z=0.):
    node = remote.RemoteGrasp6DNode.__new__(remote.RemoteGrasp6DNode)
    node.gripper_geometry = f.tabletop_gripper()
    node.gripper_tool_jaw_axis = 'y'
    node.gripper_tool_finger_length_axis = 'z'
    surface = f.attach_bilateral_surface(node, height_m=.021)
    _, reference = node._active_multiview_surface()
    correction = remote.quaternion_matrix(remote.quaternion_from_euler(np.deg2rad(roll_deg), 0., np.deg2rad(3.)))
    correction[:3, 3] = [.0086, -.00465, z]
    result = remote.RegistrationResult(True, 'OK', correction, 200, .9, .001, abs(roll_deg), abs(z))
    f.bind_surface_snapshot(node, surface, reference, result)
    return node, surface, reference, correction


@pytest.mark.parametrize('roll_deg,z', [(0., 0.), (1., .002)])
def test_measurement_is_rigidly_equivariant_and_does_not_rewrite_reference(roll_deg, z):
    node, stored, reference, correction = node_with_correction(roll_deg, z)
    center = np.array([0., 0., .0105])
    rotation = np.diag([-1., 1., -1.])
    inverse = np.linalg.inv(correction)
    raw_center = inverse[:3, :3].dot(center) + inverse[:3, 3]
    raw_rotation = inverse[:3, :3].dot(rotation)
    raw, evidence, bounds, axis = node._current_bilateral_surface_measurement(raw_center, raw_rotation)
    _, wanted, wanted_bounds, _ = node._current_bilateral_surface_measurement(center, rotation, surface_frame='reference')
    assert evidence.ok and wanted.ok
    assert evidence.measured_width_m == pytest.approx(wanted.measured_width_m, abs=1e-12)
    np.testing.assert_allclose(bounds, wanted_bounds, atol=1e-12)
    np.testing.assert_allclose(raw.points_base.dot(correction[:3, :3].T)+correction[:3, 3], stored.points_base, atol=1e-12)
    assert node._active_multiview_surface()[0] is stored
    np.testing.assert_array_equal(raw.view_indices, stored.view_indices)
    assert raw.view_stamps_ns == stored.view_stamps_ns
    _, raw_reference = node._snapshot_multiview_surface()
    np.testing.assert_allclose(raw_reference.points_base.dot(raw_reference.support_normal_base) + raw_reference.support_offset_m,
                               reference.points_base.dot(reference.support_normal_base) + reference.support_offset_m, atol=1e-12)


@pytest.mark.parametrize('invalid', ['stamp', 'identity', 'failed', 'missing'])
def test_surface_correction_requires_exact_current_registration(invalid):
    node, _, _, _ = node_with_correction()
    evidence = node._latest_registration_evidence
    if invalid == 'stamp':
        evidence = replace(evidence, stamp_ns=evidence.stamp_ns-1)
    elif invalid == 'identity':
        evidence = replace(evidence, identity=f.TargetTrackIdentity(99, 'other'))
    elif invalid == 'failed':
        evidence = replace(evidence, result=replace(evidence.result, ok=False))
    else:
        evidence = None
    node._latest_registration_evidence = evidence
    assert node._snapshot_multiview_surface() == (None, None)
    assert node._current_bilateral_surface_measurement([0., 0., .01], np.eye(3)) == (None, None, None, None)


def test_candidates_and_final_corrected_plan_use_one_frame_at_each_stage():
    node, surface, reference, correction = node_with_correction()
    node.tabletop_geometry_enabled = True
    node.tabletop_geometry_config = f.TabletopGeometryConfig(max_candidates=32)
    node.gripper_physical_open_width_m = .05
    node.grasp_config = dict(tool_approach_axis='z', pregrasp_distance_m=.08,
                            final_approach_offset_m=.015, lift_height_m=.05)
    node.candidate_min_downward_approach_cos = .65
    node.candidate_max_final_approach_lateral_m = .01
    node.soft_score_weights = f.SoftScoreWeights()
    node.target_absolute_sanity_distance_m = .15
    node.camera_visibility_gate_enabled = False
    node.camera_visibility_diagnostic_enabled = False
    f.configure_identity_handeye(node)
    inverse = np.linalg.inv(correction)
    geometry = f.tabletop_geometry((.040, .035, .021))
    geometry.center_base = inverse[:3, :3].dot(geometry.center_base)+inverse[:3, 3]
    geometry.axes_base = inverse[:3, :3]
    geometry.object_points_base = geometry.object_points_base.dot(inverse[:3, :3].T)+inverse[:3, 3]
    prepared = NS(geometry=geometry, stamp=remote.rospy.Time(1, 100000000), near_field=True,
        snapshot=NS(quality=NS(depth_repeatability_m=0.)), target_observation=node._latest_target_observation)
    candidates, diagnostics = node._generate_tabletop_candidates(geometry, snapshot=prepared.snapshot)
    assert candidates, diagnostics
    normalized = node._normalize_tabletop_candidate(prepared, candidates[0])
    sequence = normalized.grasp_sequence
    _, profile = node._make_contact_sequence(sequence.grasp, geometry,
        insertion_axis_base=normalized.insertion_axis_base, snapshot=prepared.snapshot)
    runtime = dict(prepared=prepared, grasp_pose=sequence.grasp, adaptive_stage_profile=profile,
        scored_candidate=NS(payload=normalized, stable_candidate=NS(center_base_xyz=normalized.contact_center_base,
            required_open_width_m=normalized.required_open_width_m)))
    plan = f.bound_surface_plan(prepared.target_observation)
    plan.candidate_source = 'tabletop_geometry'
    plan.poses = [remote.deepcopy(getattr(sequence, stage).pose) for stage in ('pregrasp', 'approach', 'grasp', 'lift')]
    expected = [correction.dot(remote.pose_matrix(getattr(sequence, stage))) for stage in ('pregrasp', 'approach', 'grasp', 'lift')]
    assert node._apply_registration_evidence_to_plan(plan, prepared.target_observation)
    assert plan.refinement_status == 'VALID_3D'
    node._prepare_final_registered_contact_plan(plan, prepared, runtime)
    assert runtime['final_registered_geometry_gate']['ok']
    # No second correction, and the reference cloud and raw OBB stay frozen.
    for pose, transform in zip(plan.poses, expected):
        np.testing.assert_allclose(remote.pose_matrix(NS(pose=pose)), transform, atol=1e-10)
    np.testing.assert_allclose(geometry.center_base, inverse[:3, :3].dot([0., 0., .0105])+inverse[:3, 3])
