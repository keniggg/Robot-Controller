"""Necessary endpoint failures can skip IK; passing endpoints prove no path."""
from copy import deepcopy
from types import SimpleNamespace as NS

import numpy as np
import pytest

from test_remote_grasp6d_streaming import remote_node as remote
import test_grasp_task_sequence as task_fixtures


def fixture():
    node = remote.RemoteGrasp6DNode.__new__(remote.RemoteGrasp6DNode)
    g = remote.GripperGeometry(.05, .002, np.array([.0434, .0286, .06]),
                               np.array([.1175, .155, .0774]), .003)
    plan = task_fixtures.GraspTaskSequenceTest()._rich_plan()
    plan.model_choice = 'unknown_tabletop'
    plan.refinement_translation_m = plan.refinement_rotation_deg = 0.
    plan.poses = [remote.make_pose_stamped('base_link', [0., 0., .1], [0., 0., 0., 1.]).pose
                  for _ in range(4)]
    T = remote.pose_matrix(node._rich_plan_execution_sequence(plan).grasp)
    name, center, size = remote._stage_boxes(T, g, .05, 'y', 'z')[0]
    obj = plan.object_geometry
    obj.pose_base.position.x, obj.pose_base.position.y, obj.pose_base.position.z = center
    obj.pose_base.orientation.x = obj.pose_base.orientation.y = obj.pose_base.orientation.z = 0.
    obj.pose_base.orientation.w = 1.
    obj.size_xyz_m.x = obj.size_xyz_m.y = obj.size_xyz_m.z = .008
    plan.plan_id = remote.compute_plan_id(plan)
    node.gripper_geometry = g
    return node, plan, g, T, center, size


def test_inevitable_overlap_is_a_rejection_only_and_preserves_inputs():
    node, plan, g, *_ = fixture()
    original = deepcopy(plan)
    report = node._contact_goal_inevitable_collision(plan, g, .05, .001)
    assert report['checked'] and report['rejected']
    assert not report['execution_authorized'] and not report['path_verified']
    assert report['opening_interval_m'] == [.049, .05]
    assert plan == original


def test_disjoint_or_boundary_overlap_never_claims_safe_path_or_inevitable_failure():
    node, plan, g, T, center, size = fixture()
    for offset in (1., .5*size[0]+.004-.0001):
        plan.object_geometry.pose_base.position.x = center[0]+offset
        report = node._contact_goal_inevitable_collision(plan, g, .05, .001)
        assert report['checked'] and not report['rejected']
        assert not report['execution_authorized'] and not report['path_verified']


@pytest.mark.parametrize('opening,tolerance', [(float('nan'), .001), (.05, -.001), (.06, .001)])
def test_invalid_opening_has_no_early_verdict(opening, tolerance):
    node, plan, g, *_ = fixture()
    report = node._contact_goal_inevitable_collision(plan, g, opening, tolerance)
    assert not report['checked'] and not report['rejected']


def test_core_collision_survives_allowed_pose_and_opening_perturbations():
    node, plan, g, T, center, size = fixture()
    report = node._contact_goal_inevitable_collision(plan, g, .05, .001)
    assert report['rejected']
    rng = np.random.default_rng(27)
    for _ in range(120):
        axis = rng.normal(size=3); axis /= np.linalg.norm(axis)
        theta = .005
        q = np.r_[axis*np.sin(theta/2.), np.cos(theta/2.)]
        moved = remote.quaternion_matrix(q)
        direction = rng.normal(size=3); direction /= np.linalg.norm(direction)
        moved[:3, 3] = T[:3, 3]+.0005*direction
        for opening in (.049, .05):
            body, actual_center, actual_size = remote._stage_boxes(moved, g, opening, 'y', 'z')[0]
            assert remote._obb_overlap(actual_center, moved[:3, :3], actual_size,
                                       center, np.eye(3), np.full(3, .008))


def test_global_rigid_rotation_and_translation_preserve_necessary_result():
    node, plan, g, *_ = fixture()
    reference = node._contact_goal_inevitable_collision(plan, g, .05, .001)
    transform = remote.quaternion_matrix(remote.quaternion_from_euler(.3, -.7, 1.2))
    transform[:3, 3] = [.18, -.21, .07]
    for pose in plan.poses+[plan.object_geometry.pose_base]:
        p = remote.make_pose_stamped('base_link', [pose.position.x, pose.position.y, pose.position.z],
            [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        result = transform @ remote.pose_matrix(p)
        pose.position.x, pose.position.y, pose.position.z = result[:3, 3]
        q = remote.quaternion_from_matrix(result)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
    actual = node._contact_goal_inevitable_collision(plan, g, .05, .001)
    assert actual['rejected'] == reference['rejected']
    assert [b['inevitable_overlap'] for b in actual['bodies']] == [b['inevitable_overlap'] for b in reference['bodies']]


def setup_early(node, plan, monkeypatch):
    node.contact_sequence_precheck_enabled = True
    runtime = dict(prepared=NS(model_choice='unknown_tabletop'), registration_support_lift_m=0.)
    calls = []
    def build(candidate, geometry_only=False):
        assert geometry_only is True
        calls.append('final_geometry')
        return dict(rich_plan=plan)
    node._build_selected_preview_bundle = build
    monkeypatch.setattr(remote.rospy, 'get_param', lambda name, default=None: default)
    return runtime, calls


@pytest.mark.parametrize('field', ['refinement_translation_m', 'refinement_rotation_deg', 'registration_support_lift_m'])
def test_registration_or_support_recovery_retains_resolver_opportunity(monkeypatch, field):
    node, plan, *_ = fixture()
    runtime, calls = setup_early(node, plan, monkeypatch)
    if hasattr(plan, field):
        setattr(plan, field, .001)
    else:
        runtime[field] = .001
    assert node._early_contact_goal_rejection(NS(), runtime) is None
    assert not runtime['contact_goal_necessary_check']['checked']
    assert calls == ['final_geometry']


def test_unresolved_free_pose_geometry_does_not_become_early_rejection(monkeypatch):
    node, plan, *_ = fixture()
    runtime, _ = setup_early(node, plan, monkeypatch)
    def unresolved(*a, **kw):
        raise remote.CandidateContractError('GRIPPER_SWEEP_COLLISION', 'pregrasp pose needs resolution')
    node._build_selected_preview_bundle = unresolved
    assert node._early_contact_goal_rejection(NS(), runtime) is None
    assert not runtime['contact_goal_necessary_check']['rejected']


def test_inevitable_contact_failure_skips_all_strict_and_resolver_calls(monkeypatch):
    node, plan, *_ = fixture()
    runtime, calls = setup_early(node, plan, monkeypatch)
    node._stable_variant_runtime = {(1, 0): runtime}
    node._check_moveit_stable_candidate = lambda _: pytest.fail('irreparable contact needs no IK')
    result = node._check_direct_registered_candidate(NS(track_id=1, variant_index=0))
    assert result.reachable is False
    assert runtime['final_contact_rejection']['code'] == 'CONTACT_SEQUENCE_PRECHECK_FAILED'
    assert runtime['contact_goal_necessary_check']['rejected']
    assert calls == ['final_geometry']


def test_two_resolver_roundoff_allowances_prevent_false_boundary_rejection():
    node, plan, g, T, _, _ = fixture()
    name, center, size = remote._stage_boxes(T, g, .0495, 'y', 'z')[0]
    radius = np.linalg.norm(center-T[:3, 3])+.5*np.linalg.norm(size)
    old_erosion = .0005+2*radius*np.sin(.005/2)+.00025+1e-7
    cumulative_angle = 2*2*np.arccos(1.-1e-9)
    true_erosion = (.0005+2*np.sqrt(3)*1e-12
        +2*radius*np.sin((.005+cumulative_angle)/2)+.00025+1e-7)
    obj=plan.object_geometry
    obj.pose_base.position.x = center[0]+.5*size[0]-.5*(old_erosion+true_erosion)
    obj.pose_base.position.y, obj.pose_base.position.z = center[1:]
    obj.size_xyz_m.x = obj.size_xyz_m.y = obj.size_xyz_m.z = 1e-7
    assert remote._obb_overlap(center, T[:3, :3], size-2*old_erosion,
        np.array([obj.pose_base.position.x, obj.pose_base.position.y, obj.pose_base.position.z]),
        np.eye(3), np.full(3, 1e-7))
    report = node._contact_goal_inevitable_collision(plan, g, .05, .001)
    assert report['rejected'] is False
    assert report['maximum_contact_orientation_resolutions'] == 2
    assert report['total_goal_orientation_bound_rad'] >= .005+cumulative_angle
    assert report['total_goal_position_bound_m'] >= .0005+2*np.sqrt(3)*1e-12


@pytest.mark.parametrize('quaternion', [[.1, 0., 0., 1.], [float('nan'), 0., 0., 1.], [0., 0., 0., 0.], [0., 0., 0., 2.]])
def test_resolver_raw_dot_cannot_hide_changed_or_invalid_contact_rotation(monkeypatch, quaternion):
    node, plan, *_ = fixture()
    sequence = node._rich_plan_execution_sequence(plan)
    stages = [(name, getattr(sequence, name)) for name in ('pregrasp','approach','grasp','lift')]
    response = [deepcopy(pose) for _,pose in stages]
    q = response[2].pose.orientation
    q.x, q.y, q.z, q.w = quaternion
    monkeypatch.setattr(remote.rospy, 'wait_for_service', lambda *a, **kw: None)
    monkeypatch.setattr(remote.rospy, 'get_param', lambda name, default=None: default)
    monkeypatch.setattr(remote.rospy, 'ServiceProxy', lambda *a: lambda request: NS(
        success=True, resolved_targets=response, max_position_error=0.,
        message='policy=deterministic_geodesic_collision_ik'))
    resolved, audit, code, reason = node._resolve_free_space_sequence(stages)
    assert resolved is None and code == 'MOVEIT_RESOLVE_ERROR'


@pytest.mark.parametrize('field', ['refinement_translation_m', 'refinement_rotation_deg', 'registration_support_lift_m'])
def test_final_registered_core_collision_rejects_before_precheck(monkeypatch, field):
    node, plan, *_ = fixture()
    runtime, _ = setup_early(node, plan, monkeypatch)
    if hasattr(plan, field):
        setattr(plan, field, .001)
    else:
        runtime[field] = .001
    # The early unresolved family must keep its resolver opportunity.
    assert node._early_contact_goal_rejection(NS(), runtime) is None
    assert not runtime['contact_goal_necessary_check']['checked']
    node._final_contact_sequence_evaluation = lambda *a, **kw: pytest.fail('known collision needs no service')
    with pytest.raises(remote.CandidateContractError, match='unavoidable target OBB collision'):
        node._strict_check_final_near_field_plan(plan, NS(), runtime)
    report = runtime['contact_goal_necessary_check']
    assert report['checked'] and report['rejected']
    assert report['proof_stage'] == 'final_registered_after_original_resolution'
    assert report['plan_id'] == plan.plan_id
    assert report['execution_authorized'] is False


def test_inconclusive_final_core_still_requires_the_complete_checker(monkeypatch):
    node, plan, _, _, center, size = fixture()
    runtime, _ = setup_early(node, plan, monkeypatch)
    plan.refinement_translation_m = .001
    # Only a thin outer-shell overlap: the unchanged eroded-core proof cannot reject.
    plan.object_geometry.pose_base.position.x = center[0] + .5*size[0]+.004-.0001
    node._joint_state_audit = lambda _: {}
    node._direct_near_field_active = lambda _: False
    def full_check(*a, **kw):
        raise RuntimeError('complete contact precheck still required')
    node._final_contact_sequence_evaluation = full_check
    with pytest.raises(RuntimeError, match='complete contact precheck still required'):
        node._strict_check_final_near_field_plan(plan, NS(), runtime)
    assert runtime['contact_goal_necessary_check']['checked']
    assert not runtime['contact_goal_necessary_check']['rejected']
    assert not runtime['contact_goal_necessary_check']['path_verified']


def _recorded_request47():
    import json
    from pathlib import Path
    return json.loads((Path(__file__).parent / 'fixtures' / 'request47_contact_goal_20261003.json').read_text())


def _reconstructed_registered_plan(row, record):
    node, plan, gripper, *_ = fixture()
    geometry = record['geometry']
    obj = plan.object_geometry
    obj.pose_base.position.x, obj.pose_base.position.y, obj.pose_base.position.z = geometry['obb_center_base_m']
    frame = np.eye(4); frame[:3, :3] = geometry['R_base_obb']
    q = remote.quaternion_from_matrix(frame)
    obj.pose_base.orientation.x, obj.pose_base.orientation.y, obj.pose_base.orientation.z, obj.pose_base.orientation.w = q
    obj.size_xyz_m.x, obj.size_xyz_m.y, obj.size_xyz_m.z = geometry['obb_size_xyz_m']
    obj.support_normal_base.x, obj.support_normal_base.y, obj.support_normal_base.z = geometry['support_normal_base']
    obj.support_offset_m = geometry['support_offset_m']
    correction = np.asarray(record['registration']['snapshot_to_reference_transform'])
    plan.poses = []
    # This reconstructs requested endpoints, not unrecorded MoveIt trajectories.
    # Each registered endpoint is already above the existing 3.5 mm placement
    # floor, so the original support-restoration step adds zero translation.
    for stage in row['original_stages']:
        pose = remote.make_pose_stamped('base_link', stage['position_m'], stage['quaternion_xyzw'])
        transform = correction @ remote.pose_matrix(pose)
        for _, center, size in remote._stage_boxes(transform, gripper, .05, 'y', 'z'):
            n = np.asarray(geometry['support_normal_base'])
            clearance = center @ n + geometry['support_offset_m'] - .5*np.asarray(size) @ np.abs(transform[:3, :3].T @ n)
            assert clearance >= .0035
        q = remote.quaternion_from_matrix(transform)
        plan.poses.append(remote.make_pose_stamped('base_link', transform[:3, 3], q).pose)
    registration = record['registration']
    plan.refinement_status = 'VALID_3D'
    plan.refinement_inlier_count = registration['inlier_count']
    plan.refinement_overlap_fraction = registration['overlap_fraction']
    plan.refinement_rmse_m = registration['rmse_m']
    plan.refinement_translation_m = registration['translation_m']
    plan.refinement_rotation_deg = registration['rotation_deg']
    plan.refinement_source_clipped = registration['source_clipped']
    plan.fused_view_count = 3
    plan.plan_id = remote.compute_plan_id(plan)
    return node, plan, gripper


def test_request47_all_fourteen_recorded_nominal_collisions_reject_before_precheck(monkeypatch):
    record = _recorded_request47()
    rows = [row for row in record['candidates'] if row['recorded_failure'].startswith('nominal continuous geometry:')]
    assert record['request_id'] == 47 and len(rows) == 14
    for row in rows:
        node, plan, _ = _reconstructed_registered_plan(row, record)
        runtime, _ = setup_early(node, plan, monkeypatch)
        node._final_contact_sequence_evaluation = lambda *a, **kw: pytest.fail('no service for inevitable overlap')
        original = deepcopy(plan)
        with pytest.raises(remote.CandidateContractError, match='unavoidable target OBB collision'):
            node._strict_check_final_near_field_plan(plan, NS(), runtime)
        assert runtime['contact_goal_necessary_check']['proof_stage'] == 'final_registered_after_original_resolution'
        assert runtime['contact_goal_necessary_check']['rejected']
        assert plan == original


def test_request47_short_axis_is_not_rejected_or_marked_reachable_by_long_axis_failure():
    record = _recorded_request47()
    node, plan, gripper = _reconstructed_registered_plan(record['candidates'][0], record)
    frame = np.eye(4)
    frame[:3, :3] = np.asarray(record['geometry']['R_base_obb']) @ np.diag([-1., 1., -1.])
    frame[:3, 3] = record['geometry']['obb_center_base_m']
    q = remote.quaternion_from_matrix(frame)
    plan.poses = [remote.make_pose_stamped('base_link', frame[:3, 3], q).pose for _ in range(4)]
    # A diagnostic orientation only: no bilateral surface, path or reachability
    # is invented for it. This test proves the rejection remains pose-specific.
    report = node._contact_goal_inevitable_collision(plan, gripper, .05, .001)
    assert report['checked'] and not report['rejected']
    assert report['path_verified'] is False and report['execution_authorized'] is False
