"""A registered lift crossing an IK boundary may re-resolve free poses only."""
from copy import deepcopy
from dataclasses import replace
from types import SimpleNamespace as NS

import numpy as np
import pytest

from test_remote_grasp6d_streaming import remote_node as remote, CandidateGateResult, MoveItResult
import test_grasp_task_sequence as task_fixtures


def fixture():
    node = remote.RemoteGrasp6DNode.__new__(remote.RemoteGrasp6DNode)
    node._direct_near_field_active = lambda _p: True
    node._direct_near_field_candidate_deadline_sec = lambda _p, _r: 123.
    plan = task_fixtures.GraspTaskSequenceTest()._rich_plan()
    plan.header.frame_id = 'base_link'
    plan.header.stamp = remote.rospy.Time(10)
    plan.required_open_width_m = .04
    plan.poses = [remote.make_pose_stamped('base_link', [.1, -.3, z], [0., 0., 0., 1.],
        stamp=plan.header.stamp).pose for z in [.17, .13, .11, .16]]
    plan.plan_id = remote.compute_plan_id(plan)
    resolved = node._rich_plan_execution_sequence(plan)
    q = remote.quaternion_from_euler(.1, 0., 0.)
    for name in ('pregrasp', 'lift'):
        p = getattr(resolved, name).pose.orientation
        p.x, p.y, p.z, p.w = q
    good = MoveItResult(True, 1., .4, 'strict final resolved sequence',
        collision_free=True, within_joint_limits=True, ik_valid=True, planning_success=True)
    bad = MoveItResult(False, 1., .4, 'strict sequence lift unreachable: fraction 0.938 < 0.980',
        failure_code='MOVEIT_UNREACHABLE')
    gate = CandidateGateResult(True, '', '', .04, 0., .003, 1., 0., 0., '', 6)
    prepared, runtime = NS(), dict(scored_candidate=object())
    calls = []
    def strict(stages, _prepared, _runtime, operation, deadline_sec):
        calls.append((operation, deadline_sec))
        return (bad if len(calls) == 1 else good), {}
    def resolve(stages, _prepared, _runtime, deadline_sec):
        assert deadline_sec == 123.
        for (name, pose), original in zip(stages, plan.poses):
            assert pose.pose == original
        return resolved, dict(available=True, policy_attested=True), '', ''
    node._cached_strict_moveit_sequence_evaluation = strict
    node._cached_free_space_sequence_resolution = resolve
    node._registered_sequence_geometry_gate = lambda *a: gate
    return node, plan, prepared, runtime, resolved, good, bad, gate, calls


def test_registered_free_pose_resolution_is_rechecked_and_rehashed_before_publication():
    node, plan, prepared, runtime, resolved, *_rest, calls = fixture()
    original = deepcopy(plan)
    sequence, result, _ = node._strict_check_final_near_field_plan(plan, prepared, runtime)
    assert result.reachable
    assert calls == [('final_registered_strict_sequence', 123.), ('final_resolved_strict_sequence', 123.)]
    for index, name in enumerate(('pregrasp', 'approach', 'grasp', 'lift')):
        assert plan.poses[index] == getattr(sequence, name).pose
        assert plan.poses[index].position == original.poses[index].position
    assert plan.poses[1:3] == original.poses[1:3]
    assert plan.plan_id != original.plan_id
    assert plan.plan_id == remote.compute_plan_id(plan) == runtime['final_strict_plan_id']
    assert runtime['final_registered_geometry_gate']['ok']


@pytest.mark.parametrize('failure', ['geometry', 'strict', 'incomplete_evidence', 'resolver'])
def test_failed_final_resolution_cannot_mutate_the_hashed_plan(failure):
    node, plan, prepared, runtime, resolved, good, bad, gate, calls = fixture()
    original = deepcopy(plan)
    if failure == 'geometry':
        node._registered_sequence_geometry_gate = lambda *a: replace(gate, ok=False,
            failure_code='GRIPPER_SWEEP_COLLISION', failure_reason='palm enters support',
            failed_gate='static_envelope', passed_gate_count=4)
    elif failure == 'resolver':
        node._cached_free_space_sequence_resolution = lambda *a, **kw: (
            None, dict(available=False), 'MOVEIT_UNREACHABLE', 'no admissible free-space rotation')
    else:
        def strict(*args, **kw):
            calls.append(args[3])
            if len(calls) == 1 or failure == 'strict':
                return bad, {}
            return replace(good, collision_free=None), {}
        node._cached_strict_moveit_sequence_evaluation = strict
    with pytest.raises(remote.CandidateContractError):
        node._strict_check_final_near_field_plan(plan, prepared, runtime)
    assert plan == original
    assert 'final_execution_sequence' not in runtime
    assert 'final_strict_plan_id' not in runtime


def test_final_timeout_does_not_start_additional_resolution():
    node, plan, prepared, runtime, _, _, bad, *_ = fixture()
    node._cached_strict_moveit_sequence_evaluation = lambda *a, **kw: (
        replace(bad, failure_code='MOVEIT_TIMEOUT'), {})
    node._cached_free_space_sequence_resolution = lambda *a, **kw: pytest.fail('deadline already consumed')
    with pytest.raises(remote.CandidateContractError, match='final registration-bound'):
        node._strict_check_final_near_field_plan(plan, prepared, runtime)
