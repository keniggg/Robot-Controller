"""Measured contact evidence survives transport and binds the final family."""
from copy import deepcopy
import io
import json
from types import SimpleNamespace as NS

import numpy as np
import pytest

from alicia_flexible_grasp.grasp.contact_evidence import decode_contact_evidence, encode_contact_evidence
from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id, plan_id_matches_content
from alicia_flexible_grasp.robot.observation_path_guard import committed_plan_digest
from alicia_flexible_grasp_supervisor.msg import Grasp6DPlan
from test_rich_plan_candidate_source import valid_refined_ros_plan
from test_remote_grasp6d_node import make_selected_candidate, make_geometry_message
from test_remote_grasp6d_streaming import (
    remote_node as remote, tabletop_geometry, tabletop_gripper,
    configure_identity_handeye, attach_bilateral_surface, TabletopGeometryConfig,
    SoftScoreWeights,
)


def bound_plan():
    plan = valid_refined_ros_plan()
    plan.model_choice = 'unknown_tabletop'
    plan.diagnostic = 'CONTACT_EXECUTION_PLAN'
    plan.contact_geometry_evidence = encode_contact_evidence(
        contact_center_base=[.30, -.02, .21], contact_height_axis_base=[0., 0., 1.],
        contact_height_bounds_m=[-.008, .006], minimum_contact_overlap_m=.002,
        grasp_pose=plan.poses[2])
    plan.plan_id = compute_plan_id(plan)
    return plan


def test_frozen_measurement_survives_ros_transport_and_full_commit_digest():
    plan = bound_plan()
    original = decode_contact_evidence(plan)
    output = io.BytesIO()
    plan.serialize(output)
    received = Grasp6DPlan().deserialize(output.getvalue())
    assert decode_contact_evidence(received) == original
    assert received.contact_geometry_evidence == plan.contact_geometry_evidence
    assert compute_plan_id(received) == plan.plan_id
    assert committed_plan_digest(received) == committed_plan_digest(plan)
    received.header.seq = 239
    assert committed_plan_digest(received) == committed_plan_digest(plan)


@pytest.mark.parametrize('field,value', [
    ('contact_center_base', [.31, -.02, .21]),
    ('contact_height_bounds_m', [-.007, .006]),
    ('minimum_contact_overlap_m', .001),
    ('gripper_model', 'other'),
])
def test_changed_measurement_revokes_both_execution_signatures(field, value):
    plan = bound_plan()
    original_id, original_digest = plan.plan_id, committed_plan_digest(plan)
    evidence = json.loads(plan.contact_geometry_evidence)
    evidence[field] = value
    plan.contact_geometry_evidence = json.dumps(evidence, sort_keys=True, separators=(',', ':'))
    assert compute_plan_id(plan) != original_id
    assert not plan_id_matches_content(plan)
    assert committed_plan_digest(plan) != original_digest


@pytest.mark.parametrize('field,value', [
    ('schema_version', True), ('schema_version', 2),
    ('gripper_model', 'other'), ('tool_jaw_axis', '+X'),
    ('measurement_source', 'obb'), ('contact_height_axis_base', [0., 0., -1.]),
    ('contact_height_bounds_m', [.01, -.01]),
    ('contact_height_bounds_m', [0., float('nan')]),
    ('minimum_contact_overlap_m', 0.), ('minimum_contact_overlap_m', True),
    ('contact_center_base', [10**400, 0., 0.]),
    ('minimum_contact_overlap_m', 10**400),
])
def test_invalid_evidence_cannot_be_authorized_by_rehashing(field, value):
    plan = bound_plan()
    evidence = json.loads(plan.contact_geometry_evidence)
    evidence[field] = value
    plan.contact_geometry_evidence = json.dumps(evidence, sort_keys=True, separators=(',', ':'))
    plan.plan_id = compute_plan_id(plan)
    with pytest.raises(ValueError):
        decode_contact_evidence(plan)


def test_final_pose_change_requires_new_measurement_even_with_new_plan_id():
    plan = bound_plan()
    plan.poses[2].position.z += .001
    plan.plan_id = compute_plan_id(plan)
    assert plan_id_matches_content(plan)
    with pytest.raises(ValueError, match='final grasp pose'):
        decode_contact_evidence(plan)


def test_duplicate_or_noncanonical_json_is_not_valid_evidence():
    plan = bound_plan()
    for value in ('{ "schema_version":1,"schema_version":1}',
                  json.dumps(json.loads(plan.contact_geometry_evidence), indent=2)):
        plan.contact_geometry_evidence = value
        with pytest.raises(ValueError):
            decode_contact_evidence(plan)


@pytest.mark.parametrize('model,phase,required', [
    ('unknown_tabletop', 'CONTACT_EXECUTION_PLAN', True),
    ('unknown_tabletop', 'FAR_FIELD_OBSERVATION_PLAN', False),
    ('carton_segment', 'CONTACT_EXECUTION_PLAN', False),
])
def test_missing_measurement_is_rejected_only_where_contact_authority_needs_it(model, phase, required):
    plan = bound_plan()
    plan.contact_geometry_evidence = ''
    plan.model_choice, plan.diagnostic = model, phase
    if required:
        with pytest.raises(ValueError, match='missing'):
            decode_contact_evidence(plan)
    else:
        assert decode_contact_evidence(plan) is None
    with pytest.raises(ValueError, match='missing'):
        decode_contact_evidence(plan, required=True)
    old = NS(**{name: getattr(plan, name) for name in plan.__slots__ if name != 'contact_geometry_evidence'})
    assert compute_plan_id(old) == compute_plan_id(plan)


def test_build_rich_plan_copies_explicit_evidence_and_rejects_seed_pose_evidence():
    selected, geometry = make_selected_candidate(), make_geometry_message()
    seed = remote.build_rich_plan(selected, geometry, geometry.header, 'carton_segment')
    evidence = encode_contact_evidence(contact_center_base=[.4, 0., .1],
        contact_height_axis_base=remote.pose_matrix(NS(pose=seed.poses[2]))[:3, 2],
        contact_height_bounds_m=[-.005, .005], minimum_contact_overlap_m=.002,
        grasp_pose=seed.poses[2])
    plan = remote.build_rich_plan(selected, geometry, geometry.header, 'unknown_tabletop',
                                  contact_geometry_evidence=evidence)
    assert plan.contact_geometry_evidence == evidence
    assert plan.plan_id != seed.plan_id
    stale = json.loads(evidence)
    stale['grasp_tool_pose_base'][2] += .003
    with pytest.raises(ValueError, match='final grasp pose'):
        remote.build_rich_plan(selected, geometry, geometry.header, 'unknown_tabletop',
            contact_geometry_evidence=json.dumps(stale, sort_keys=True, separators=(',', ':')))


def final_family_fixture():
    node = remote.RemoteGrasp6DNode.__new__(remote.RemoteGrasp6DNode)
    node.tabletop_geometry_enabled = True
    node.tabletop_geometry_config = TabletopGeometryConfig(max_candidates=32)
    node.gripper_geometry = tabletop_gripper()
    node.gripper_tool_jaw_axis, node.gripper_tool_finger_length_axis = 'y', 'z'
    node.gripper_physical_open_width_m = .05
    node.grasp_config = {'tool_approach_axis': 'z', 'pregrasp_distance_m': .08,
                         'final_approach_offset_m': .015, 'lift_height_m': .05}
    node.candidate_min_downward_approach_cos = .65
    node.candidate_max_final_approach_lateral_m = .010
    node.soft_score_weights = SoftScoreWeights()
    node.target_absolute_sanity_distance_m = .15
    node.camera_visibility_gate_enabled = False
    node.camera_visibility_diagnostic_enabled = False
    configure_identity_handeye(node)
    attach_bilateral_surface(node, height_m=.021)
    geometry = tabletop_geometry((.040, .035, .021))
    prepared = NS(geometry=geometry, stamp=remote.rospy.Time(1),
        snapshot=NS(quality=NS(depth_repeatability_m=0.)), near_field=True)
    candidates, _ = node._generate_tabletop_candidates(geometry, snapshot=prepared.snapshot)
    candidate = candidates[0]
    normalized = node._normalize_tabletop_candidate(prepared, candidate)
    sequence = normalized.grasp_sequence
    _, profile = node._make_contact_sequence(sequence.grasp, geometry,
        insertion_axis_base=candidate.insertion_axis_base, snapshot=prepared.snapshot)
    runtime = dict(prepared=prepared, grasp_pose=sequence.grasp,
        adaptive_stage_profile=profile,
        scored_candidate=NS(payload=normalized,
            stable_candidate=NS(center_base_xyz=candidate.contact_center_base,
                required_open_width_m=candidate.required_open_width_m)))
    plan = Grasp6DPlan()
    plan.candidate_source = 'tabletop_geometry'
    plan.model_choice = 'unknown_tabletop'
    plan.diagnostic = 'CONTACT_EXECUTION_PLAN'
    plan.required_open_width_m = candidate.required_open_width_m
    plan.poses = [deepcopy(getattr(sequence, s).pose) for s in ('pregrasp', 'approach', 'grasp', 'lift')]
    return node, prepared, runtime, plan, candidate


def test_final_clearance_materialization_overwrites_stale_seed_geometry():
    node, prepared, runtime, plan, candidate = final_family_fixture()
    for pose in plan.poses:
        pose.position.z -= .001
    plan.refinement_translation_m = .001
    initial_grasp_z = plan.poses[2].position.z
    runtime['final_contact_geometry_evidence'] = 'stale seed measurement'
    node._prepare_final_registered_contact_plan(plan, prepared, runtime)
    frozen = decode_contact_evidence(plan)
    assert plan.poses[2].position.z > initial_grasp_z
    assert frozen['grasp_tool_pose_base'][2] == plan.poses[2].position.z
    transform = remote.pose_matrix(node._rich_plan_execution_sequence(plan).grasp)
    _, _, bounds, axis = node._current_bilateral_surface_measurement(
        candidate.contact_center_base, transform[:3, :3])
    np.testing.assert_allclose(frozen['contact_height_bounds_m'], bounds)
    np.testing.assert_allclose(frozen['contact_height_axis_base'], axis)
    assert frozen['minimum_contact_overlap_m'] == node._phase_contact_overlap_requirement_m(
        True, runtime['adaptive_stage_profile'])


def test_registered_runtime_measurement_uses_transformed_center_and_final_pose():
    node, prepared, runtime, plan, candidate = final_family_fixture()
    correction = np.eye(4)
    correction[:3, 3] = [.0002, 0., -.0002]
    runtime['contact_registration_transform'] = correction
    for pose in plan.poses:
        pose.position.x += correction[0, 3]
        # Final materialization has already restored the support clearance.
        pose.position.z += correction[2, 3] + .0002
    sequence = node._rich_plan_execution_sequence(plan)
    runtime['grasp_pose'] = sequence.grasp
    gate = node._resolved_sequence_geometry_gate(runtime, sequence)
    assert gate.ok, gate
    node._freeze_final_contact_evidence(plan, runtime)
    frozen = decode_contact_evidence(plan)
    center = candidate.contact_center_base + correction[:3, 3]
    np.testing.assert_allclose(frozen['contact_center_base'], center)
    _, _, bounds, _ = node._current_bilateral_surface_measurement(
        center, remote.pose_matrix(sequence.grasp)[:3, :3], surface_frame='reference')
    np.testing.assert_allclose(frozen['contact_height_bounds_m'], bounds)
    assert frozen['grasp_tool_pose_base'][2] == plan.poses[2].position.z


def test_missing_final_measurement_cannot_reuse_prior_candidate_evidence():
    node, prepared, runtime, plan, _ = final_family_fixture()
    runtime['final_contact_geometry_evidence'] = 'stale seed measurement'
    node._active_multiview_surface = lambda: (None, None)
    with pytest.raises(remote.CandidateContractError):
        node._prepare_final_registered_contact_plan(plan, prepared, runtime)
    assert 'final_contact_geometry_evidence' not in runtime
    assert not plan.contact_geometry_evidence


def test_unknown_contact_cannot_be_published_without_final_measured_evidence():
    from contextlib import nullcontext
    node = remote.RemoteGrasp6DNode.__new__(remote.RemoteGrasp6DNode)
    node._geometry_state_guard = nullcontext
    node._geometry_invalidation_generation = 3
    plan = bound_plan()
    plan.contact_geometry_evidence = ''
    plan.plan_id = compute_plan_id(plan)
    # No publishers are installed: rejection must occur before either topic.
    assert node._publish_plan_pair_if_current(plan, 3) == (
        False, 'CONTACT_GEOMETRY_EVIDENCE_INVALID')
