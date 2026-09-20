"""Closed-scene regression: useful tilts must precede budget exhaustion."""
import json
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from test_observation_distance_search import recorded_scene, remote
from test_remote_grasp6d_streaming import streaming_node, MutableClock, snapshot
from alicia_flexible_grasp.grasp.grasp6d_pipeline import MoveItResult
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_flexible_grasp.robot.observation_preview import candidate_scene, qualify_candidate_path
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


def recorded_views(fixture='observation_tilt_20260919.json'):
    data = json.loads((Path(__file__).parent / 'fixtures' / fixture).read_text())
    node, geometry, prepared, _ = recorded_scene(remote.DEFAULT_OBSERVATION_CAMERA_ROLL_OFFSETS_DEG)
    node.observation_camera_tilt_offsets_deg = remote.DEFAULT_OBSERVATION_CAMERA_TILT_OFFSETS_DEG
    g = data['geometry']
    geometry = NS(center_base=np.array(g['obb_center_base_m']), axes_base=np.array(g['R_base_obb']),
                  size_xyz_m=np.array(g['obb_size_xyz_m']), support_normal_base=np.array(g['support_normal_base']),
                  support_offset_m=g['support_offset_m'])
    prepared.pose_estimator.T_base_camera_link = np.array(data['T_base_camera_link'])
    prepared.stamp = remote.rospy.Time(data['source_stamp_ns']//10**9, data['source_stamp_ns']%10**9)
    # Measured uncertainty from this request, no robot/parameter access.
    node._contact_stage_profile = lambda *a, **kw: NS(
        pregrasp_distance_m=.04, approach_offset_m=.025, lift_height_m=.033,
        depth_uncertainty_m=data.get('depth_uncertainty_m', .002))
    reference = node._frozen_observation_reference_pose(prepared)
    passing, _ = node._far_field_observation_variants(reference, geometry, -geometry.support_normal_base,
                                                   None, reference, prepared)
    return data, node, geometry, node._order_observation_checks(passing)


def test_recorded_useful_tilt_is_admitted_and_checked_before_shallow_views():
    data, node, geometry, ordered = recorded_views()
    first = ordered[0]
    assert first['camera_tilt_offset_deg'] == 30.
    assert first['camera_target_distance_m'] == .22
    assert first['camera_roll_offset_deg'] == 0.
    assert first['observation_side_evidence']['observation_side_evidence_deficit_m'] == 0.
    xml = (Path(__file__).resolve().parents[3] / 'src/real-arm/alicia_d_descriptions/urdf/alicia_duo_with_gripper.urdf').read_text()
    fk = SerialUrdfFk(xml, ['Joint%d'%i for i in range(1, 7)])
    np.testing.assert_allclose(fk(data['alternative_q']), remote.pose_matrix(first['sequence'].pregrasp), atol=1e-8)
    assert max(abs(np.array(data['alternative_q'])-data['start_q'])) < .81
    path = RobotTrajectory()
    path.joint_trajectory.joint_names = list(fk.names)
    path.joint_trajectory.points = [JointTrajectoryPoint(positions=q, velocities=[0.]*6,
        time_from_start=remote.rospy.Duration.from_sec(t)) for q, t in
        [(data['start_q'], 0.), (data['alternative_q'], data['duration_sec'])]]
    report = qualify_candidate_path(path, NS(positions=data['start_q'], velocities=[0.]*6, accelerations=[0.]*6),
        candidate_scene(geometry, data['source_stamp_ns']), fk, node.gripper_geometry,
        data['opening_m'], np.full(6, .12+np.pi/4096.))
    assert report['ok'], report
    # A model path proof is not a full-arm collision check or a grasp trial.


def test_expensive_first_proof_cannot_starve_useful_tilt_and_roll_rank_is_retained():
    node = streaming_node(clock=MutableClock(50.), start_worker=False)
    node.execution_plan_validity_sec = 120.
    node.mujoco_selection_snapshot_reserve_sec = 30.
    elapsed = MutableClock(0.)
    node._observation_tilt_clock = elapsed
    try:
        node.start_streaming()
        node.submit_stream_snapshot(snapshot(49.8))
        variants = []
        for i, (tilt, roll, clearance) in enumerate([(10., 0., .008), (30., 0., .073), (0., 180., .095)]):
            pose = remote.PoseStamped()
            pose.pose.position.x = i
            variants.append(dict(sequence=NS(pregrasp=pose), camera_tilt_offset_deg=tilt,
                camera_roll_offset_deg=roll, preference_index=i,
                observation_envelope=remote.ObservationEnvelopeResult(True, '', '', clearance)))
        runtime = dict(prepared=NS(ticket=node._stream_worker_ticket), grasp_pose=variants[0]['sequence'].pregrasp,
            observation_sequence=variants[0]['sequence'], observation_variants=variants, soft_evidence={})
        node._stable_variant_runtime = {(3, 1): runtime}
        checked = []
        def strict(pose, observation=False):
            i = int(pose.pose.position.x)
            checked.append(i)
            elapsed.value += 4.  # One proof consumes the whole supplemental budget.
            return MoveItResult(reachable=i > 0, joint_path_cost=1. if i == 1 else 3.3,
                joint_max_delta_rad=.81 if i == 1 else 3.06,
                reason='observation_screened_duration_sec=%s' % (24.1 if i == 1 else 61.2)), {}, ''
        node._strict_moveit_evaluation = strict
        result = node._check_moveit_stable_candidate(NS(track_id=3, variant_index=1))
        assert result.reachable
        assert checked == [1, 2]
        audit = runtime['observation_orientation_search']
        assert audit['selected_camera_tilt_offset_deg'] == 30.
        assert audit['selected_camera_roll_offset_deg'] == 0.
        assert audit['search_complete'] is False
        assert audit['ranking_scope'] == 'qualified_checked_variants_only'
    finally:
        node.shutdown_streaming_worker()


def test_intermediate_tilt_fills_recorded_side_evidence_and_support_gap():
    data, node, geometry, ordered = recorded_views('observation_tilt_gap_20260919.json')
    first = ordered[0]
    assert first['camera_tilt_offset_deg'] == 25.
    assert first['camera_roll_offset_deg'] == 0.
    assert first['camera_target_distance_m'] == .22
    assert not any(v['camera_tilt_offset_deg'] == 30. for v in ordered)
    xml = (Path(__file__).resolve().parents[3] / 'src/real-arm/alicia_d_descriptions/urdf/alicia_duo_with_gripper.urdf').read_text()
    fk = SerialUrdfFk(xml, ['Joint%d'%i for i in range(1, 7)])
    np.testing.assert_allclose(fk(data['alternative_q']), remote.pose_matrix(first['sequence'].pregrasp), atol=1e-8)
    assert max(abs(np.array(data['alternative_q'])-data['start_q'])) < .80
    path = RobotTrajectory()
    path.joint_trajectory.joint_names = list(fk.names)
    path.joint_trajectory.points = [JointTrajectoryPoint(positions=q, velocities=[0.]*6,
        time_from_start=remote.rospy.Duration.from_sec(t)) for q, t in
        [(data['start_q'], 0.), (data['alternative_q'], data['duration_sec'])]]
    report = qualify_candidate_path(path, NS(positions=data['sdk_q'], velocities=[0.]*6,
        accelerations=[0.]*6), candidate_scene(geometry, data['source_stamp_ns']), fk,
        node.gripper_geometry, data['opening_m'], np.full(6, .12+np.pi/4096.))
    assert report['ok'], report
    # Collision planning and a new exact source binding remain required live.


def test_comparison_budget_starts_only_after_qualified_incumbent():
    node = streaming_node(clock=MutableClock(50.), start_worker=False)
    elapsed = MutableClock(0.)
    node._observation_comparison_clock = elapsed
    try:
        node.start_streaming()
        node.submit_stream_snapshot(snapshot(49.8))
        variants = []
        for i in range(6):
            pose = remote.PoseStamped()
            pose.pose.position.x = i
            variants.append(dict(sequence=NS(pregrasp=pose),
                camera_roll_offset_deg=i * 15., preference_index=i))
        runtime = dict(prepared=NS(ticket=node._stream_worker_ticket),
            grasp_pose=variants[0]['sequence'].pregrasp,
            observation_sequence=variants[0]['sequence'],
            observation_variants=variants, soft_evidence={})
        node._stable_variant_runtime = {(3, 1): runtime}
        checked = []
        def strict(pose, observation=False):
            i = int(pose.pose.position.x)
            checked.append(i)
            elapsed.value += 4.
            return MoveItResult(reachable=i >= 2, joint_path_cost=6.-i,
                joint_max_delta_rad=(6.-i)/10.,
                reason='observation_screened_duration_sec=%s' % (6.-i)), {}, ''
        node._strict_moveit_evaluation = strict
        result = node._check_moveit_stable_candidate(NS(track_id=3, variant_index=1))
        assert result.reachable
        # Initial failures do not consume comparison time. Two further views
        # finish, then the unchecked tail is explicitly recorded and omitted.
        assert checked == [0, 1, 2, 3, 4]
        assert runtime['observation_sequence'] is variants[4]['sequence']
        audit = runtime['observation_orientation_search']
        assert audit['comparison_budget_reached'] is True
        assert audit['search_complete'] is False
        assert audit['reachable_variant_count'] == 3
        assert audit['variant_results'][-1]['checked'] is False
        assert audit['variant_results'][-1]['reason'] == 'OBSERVATION_COMPARISON_BUDGET_REACHED'
    finally:
        node.shutdown_streaming_worker()
