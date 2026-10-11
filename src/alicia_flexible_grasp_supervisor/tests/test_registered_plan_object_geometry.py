"""Registered contact poses and the committed target OBB share one correction."""
from copy import deepcopy
from dataclasses import replace

import numpy as np
import pytest

from test_remote_grasp6d_streaming import (
    remote_node as remote, streaming_node, seed_measured_reference_surface,
    measured_observation, measured_surface_points, bound_surface_plan,
)
from alicia_flexible_grasp.robot.observation_path_guard import FrozenObservationScene


def matrix(pose):
    return remote.transform_matrix(
        [pose.position.x, pose.position.y, pose.position.z],
        [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])


@pytest.fixture
def registered():
    node = streaming_node(start_worker=False)
    reference = seed_measured_reference_surface(node)
    angle = np.deg2rad(3.)
    rotation = np.array([[np.cos(angle), -np.sin(angle), 0.],
                         [np.sin(angle), np.cos(angle), 0.], [0., 0., 1.]])
    translation = np.array([.003, -.002, .001])
    observation = measured_observation(node, stamp_sec=20.1,
        points=(measured_surface_points()-translation).dot(rotation))
    result = node._register_near_field_surface(observation)
    assert result.ok and node._registration_rotation_deg(result.transform_base) > 1.
    plan = bound_surface_plan(observation)
    plan.header.frame_id = plan.object_geometry.header.frame_id = 'base_link'
    plan.valid = plan.object_geometry.valid = True
    plan.model_choice = 'unknown_tabletop'
    plan.candidate_source = 'tabletop_geometry'
    plan.candidate_source_lineage = ['tabletop_geometry']
    plan.diagnostic = 'CONTACT_EXECUTION_PLAN'
    plan.object_geometry.support_normal_base.z = 1.
    plan.object_geometry.support_offset_m = .002
    plan.object_geometry.size_xyz_m.x = .042
    plan.object_geometry.size_xyz_m.y = .027
    plan.object_geometry.size_xyz_m.z = .018
    plan.object_geometry.source_mode = 'instance_mask'
    plan.object_geometry.object_point_count = len(observation.points_base)
    plan.object_geometry.pose_base = remote.make_pose_stamped(
        'base_link', [.005, -.004, .033], remote.quaternion_from_euler(0., 0., -.2)).pose
    plan.poses = [remote.make_pose_stamped('base_link', [.004, -.003, height],
                  remote.quaternion_from_euler(.05, -.1, .3)).pose
                  for height in (.15, .10, .075, .13)]
    try:
        yield node, reference, observation, result, plan
    finally:
        node.shutdown_streaming_worker()


def test_registered_committed_scene_and_all_poses_share_exact_transform(registered):
    node, reference, observation, result, plan = registered
    before = deepcopy(plan)
    surface = node._near_field_fused_surface
    surface_points = surface.points_base.copy()
    observation_points = observation.points_base.copy()
    reference_points = reference.points_base.copy()
    assert node._apply_registration_evidence_to_plan(plan, observation)
    correction = np.asarray(result.transform_base)
    for old, new in zip(before.poses, plan.poses):
        np.testing.assert_allclose(matrix(new), correction @ matrix(old), atol=1e-14)
    np.testing.assert_allclose(matrix(plan.object_geometry.pose_base),
                               correction @ matrix(before.object_geometry.pose_base), atol=1e-14)
    # Only the plan's OBB pose changes. Measured support, source and identity
    # retain the current-snapshot contract, rather than relabelling old data.
    unchanged_geometry = deepcopy(plan.object_geometry)
    unchanged_geometry.pose_base = deepcopy(before.object_geometry.pose_base)
    assert unchanged_geometry == before.object_geometry
    assert plan.header == before.header
    assert plan.target_track_id == before.target_track_id
    assert plan.refinement_status == 'VALID_3D'
    assert node._near_field_fused_surface is surface
    np.testing.assert_array_equal(surface.points_base, surface_points)
    np.testing.assert_array_equal(observation.points_base, observation_points)
    np.testing.assert_array_equal(reference.points_base, reference_points)
    plan.plan_id = remote.compute_plan_id(plan)
    scene = FrozenObservationScene.from_contact_plan(plan)
    np.testing.assert_allclose(scene.center, matrix(plan.object_geometry.pose_base)[:3, 3])
    np.testing.assert_allclose(scene.rotation, matrix(plan.object_geometry.pose_base)[:3, :3])
    np.testing.assert_array_equal(scene.normal, [0., 0., 1.])
    assert scene.offset == before.object_geometry.support_offset_m
    assert scene.source_ns == observation.stamp_ns
    stale_geometry = deepcopy(plan)
    stale_geometry.object_geometry = deepcopy(before.object_geometry)
    assert remote.compute_plan_id(stale_geometry) != plan.plan_id


@pytest.mark.parametrize('collision', [False, True])
def test_original_necessary_collision_gate_is_rigid_transform_invariant(registered, collision):
    node, _, observation, _, plan = registered
    gripper = remote.GripperGeometry(.05, .002, np.array([.0434, .0286, .06]),
                                     np.array([.1175, .155, .0774]), .003)
    _, center, _ = remote._stage_boxes(matrix(plan.poses[2]), gripper, .0495, 'y', 'z')[0]
    if not collision:
        center = center + np.array([.3, .2, .1])
    plan.object_geometry.pose_base = remote.make_pose_stamped(
        'base_link', center, [0., 0., 0., 1.]).pose
    size = plan.object_geometry.size_xyz_m
    size.x = size.y = size.z = .008
    before = node._contact_goal_inevitable_collision(plan, gripper, .05, .001)
    assert before['checked'] and before['rejected'] is collision
    assert node._apply_registration_evidence_to_plan(plan, observation)
    after = node._contact_goal_inevitable_collision(plan, gripper, .05, .001)
    assert after['rejected'] is collision
    assert after['opening_interval_m'] == before['opening_interval_m'] == [.049, .05]
    assert after['total_goal_position_bound_m'] == before['total_goal_position_bound_m']
    assert after['total_goal_orientation_bound_rad'] == before['total_goal_orientation_bound_rad']
    assert not after['execution_authorized'] and not after['path_verified']


@pytest.mark.parametrize('fault', ['no_observation', 'no_evidence', 'stale_stamp', 'wrong_track'])
def test_unbound_or_direct_plan_geometry_and_poses_remain_unchanged(registered, fault):
    node, _, observation, _, plan = registered
    if fault == 'no_observation':
        observation = None
    elif fault == 'no_evidence':
        node._latest_registration_evidence = None
    elif fault == 'stale_stamp':
        observation = replace(observation, stamp_ns=observation.stamp_ns+1)
    else:
        plan.object_geometry.target_track_id = 'other-track'
    before = deepcopy(plan)
    assert node._apply_registration_evidence_to_plan(plan, observation) is False
    assert plan.poses == before.poses
    assert plan.object_geometry == before.object_geometry


def test_rejected_registration_cannot_move_target_geometry(registered):
    node, _, observation, result, plan = registered
    node._latest_registration_evidence = replace(node._latest_registration_evidence,
        result=replace(result, ok=False, code='YAW_BOUND_EXCEEDED'))
    before = deepcopy(plan)
    assert node._apply_registration_evidence_to_plan(plan, observation)
    assert plan.refinement_status == 'INVALID_3D'
    assert plan.poses == before.poses
    assert plan.object_geometry == before.object_geometry


def test_identity_registration_preserves_valid_target_pose(registered):
    node, _, observation, result, plan = registered
    node._latest_registration_evidence = replace(node._latest_registration_evidence,
        result=replace(result, transform_base=np.eye(4), maximum_point_displacement_m=0.))
    before = deepcopy(plan)
    assert node._apply_registration_evidence_to_plan(plan, observation)
    np.testing.assert_allclose(matrix(plan.object_geometry.pose_base),
                               matrix(before.object_geometry.pose_base), atol=1e-15)
    assert plan.object_geometry.header == before.object_geometry.header
    assert plan.refinement_rotation_deg == 0.
