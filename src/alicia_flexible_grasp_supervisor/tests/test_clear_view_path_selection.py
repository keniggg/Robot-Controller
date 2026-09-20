"""Finite search must prefer qualified timed paths and retain the winning cache."""
from unittest.mock import patch
from types import SimpleNamespace as NS

import pytest

import test_grasp_task_sequence as sequence
from test_grasp_task_sequence import FakeServiceResponse, grasp_task_node as task


def setup_node():
    helper = sequence.GraspTaskSequenceTest()
    node = task.GraspTaskNode.__new__(task.GraspTaskNode)
    plan = helper._rich_plan(stamp_sec=9.)
    pose = helper._pose(.4, 0., .32)
    node._current_tool_pose_base = lambda: pose
    node._current_camera_pose_base = lambda: pose
    node._clear_view_reacquisition_attempts = 0
    node.set_state = lambda *args: None
    node._execution_checkpoint = lambda *args: True
    node._invoke_plan_bound_action = lambda plan, cfg, label, action: (task.PlanValidationResult(True), action())
    node._wait_for_motion_settle = lambda *args: True
    node._validate_measured_observation_envelope = lambda *args: True
    node._request_near_field_preview_stream = lambda *args: True
    return node, plan, [helper._pose(.4+i*.01, 0., .32) for i in range(4)]


@pytest.mark.parametrize('case', ['timed_rank', 'bounded', 'replan_rejected'])
def test_path_selection_uses_final_timing_and_never_executes_an_unqualified_replan(case):
    node, plan, poses = setup_node()
    preflights, executed = [], []
    clock = [0.]
    def planner(pose, execute):
        assert not execute
        preflights.append(pose)
        return FakeServiceResponse(True, 'joint_duration_lower_bound_sec=1.0 joint_path_cost=0.2 joint_max_delta=0.1')
    def qualify(response, pose, *args):
        if case == 'replan_rejected' and len(preflights) > len(poses):
            raise ValueError('changed path fails proof')
        if case == 'bounded':
            clock[0] += 9.
        # The first candidate has the shortest lower bound but a slower actual path.
        i = next(i for i, item in enumerate(poses) if item is pose)
        return dict(joint_duration_lower_bound_sec=1.+i,
                    timed_execution_duration_sec=[10., 2., 5., 6.][i],
                    joint_path_cost=.2, joint_max_delta=.1)
    node._qualify_clear_view_candidate = qualify
    with patch.object(task, 'make_clear_view_reacquisition_poses', return_value=poses), \
            patch.object(task.time, 'monotonic', side_effect=lambda: clock[0]), \
            patch.object(task.rospy.Time, 'now', return_value=task.rospy.Time.from_sec(10.)):
        result = node._execute_clear_view_reacquisition(plan,
            dict(clear_view_reacquisition_camera_body_radius_m=.025), planner,
            lambda pose, execute: executed.append(pose) or FakeServiceResponse(True))
    if case == 'replan_rejected':
        assert not result.ok
        assert 'no longer qualified' in result.reason
        assert executed == []
    else:
        assert result.ok, result.reason
        assert len(executed) == 1 and executed[0] is poses[1]
        assert preflights[-1] is executed[0]
    if case == 'bounded':
        # A qualified second candidate exhausts comparison time; the two
        # unsearched candidates do not trigger replanning of the cache owner.
        assert len(preflights) == 2


def test_required_path_evidence_cannot_fall_back_to_scalar_metrics():
    node, plan, poses = setup_node()
    response = FakeServiceResponse(True,
        'joint_duration_lower_bound_sec=1.0 joint_path_cost=0.2 joint_max_delta=0.1')
    with pytest.raises(ValueError, match='timed path evidence'):
        node._qualify_clear_view_candidate(response, poses[0], plan,
            dict(clear_view_reacquisition_path_evidence_required=True), None)


def test_path_evidence_uses_frozen_wire_header_without_mutating_measured_pose():
    node, plan, poses = setup_node()
    pose = poses[0]
    pose.header.stamp = task.rospy.Time.from_sec(20.)
    response = FakeServiceResponse(True, 'joint_duration_lower_bound_sec=1.0 '
        'joint_path_cost=0.2 joint_max_delta=0.1 observation_path_evidence=test')
    def decode(token, wire_target, fk):
        assert token == 'test'
        assert wire_target.header == plan.header
        assert wire_target.pose == pose.pose
        return dict(opening=.05, trajectory_sha256='digest'), NS(joint_trajectory=NS(
            points=[NS(time_from_start=task.rospy.Duration(10.))])), object()
    with patch.object(task.rospy, 'get_param', return_value=True), \
            patch.object(task, 'SerialUrdfFk', return_value=object()), \
            patch('alicia_flexible_grasp.robot.observation_preview.decode_path_evidence', side_effect=decode), \
            patch('alicia_flexible_grasp.robot.observation_path_guard.FrozenObservationScene.from_plan'), \
            patch('alicia_flexible_grasp.robot.observation_tracking_contract.qualify_contract_path', return_value={'ok': True}):
        metrics = node._qualify_clear_view_candidate(response, pose, plan,
            dict(clear_view_reacquisition_path_evidence_required=True), object())
    assert metrics['timed_execution_duration_sec'] == 10.
    assert pose.header.stamp.to_sec() == 20.
