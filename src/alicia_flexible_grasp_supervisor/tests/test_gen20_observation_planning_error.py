"""Real observation planner/error transport with inert MoveGroup results.

No ROS node, service, action or hardware is started. The recorded R2 failures
were TIMED_OUT; these fixtures reproduce error propagation, not physical IK.
"""
from copy import deepcopy
from types import SimpleNamespace as NS

import pytest
from moveit_msgs.msg import MoveItErrorCodes

from test_observation_branch_hint import (
    check, fixture, offline_robot_description,
)
from test_moveit_planner_pose_feedback import JointPlan, make_pose


def noetic_result(manipulator, *, success=False, code=MoveItErrorCodes.TIMED_OUT,
                  empty=True):
    """Control the MoveGroup boundary, keeping planner methods under test real."""
    def plan():
        manipulator.plan_calls += 1
        trajectory = (NS(joint_trajectory=NS(joint_names=[], points=[])) if empty else
                      JointPlan([[0.] * 6, list(manipulator.new_goal)], duration_sec=30.))
        return success, trajectory, .8, NS(val=code)
    manipulator.plan = plan


@pytest.mark.parametrize('hinted', [False, True])
def test_noetic_timeout_preserves_code_and_never_renews_or_executes(hinted):
    planner, manipulator, clock, _ = fixture()
    planner.strict_pose_planning_time = .8
    if hinted:
        assert check(planner)[0]
        clock[0] += 1.
    previous_calls = manipulator.plan_calls
    previous_time = manipulator.get_planning_time()
    previous_clock = clock[0]
    noetic_result(manipulator)

    ok, reason = check(planner)

    assert not ok and reason.startswith('OBSERVATION_PLANNING_TIMEOUT: ')
    assert 'moveit_error_code=-6' in reason
    assert 'planning_kind=' + ('branch' if hinted else 'pose') in reason
    assert 'unreachable' not in reason.lower()
    assert manipulator.plan_calls == previous_calls + 1
    assert manipulator.get_planning_time() == previous_time
    assert manipulator.planning_time_updates[-2:] == [.8, previous_time]
    assert clock[0] == previous_clock
    assert planner._last_pose_plan is None and not planner._observation_branch_hints
    assert len(manipulator.joint_targets) == int(hinted)
    assert manipulator.goal_joint_tolerance == .0001
    assert not manipulator.executed_plans and manipulator.go_calls == 0


@pytest.mark.parametrize('code', [MoveItErrorCodes.PLANNING_FAILED,
    MoveItErrorCodes.INVALID_MOTION_PLAN, MoveItErrorCodes.CONTROL_FAILED,
    MoveItErrorCodes.START_STATE_IN_COLLISION, MoveItErrorCodes.GOAL_IN_COLLISION,
    MoveItErrorCodes.NO_IK_SOLUTION])
def test_other_noetic_failures_keep_their_code_and_are_not_timeouts(code):
    planner, manipulator, _, _ = fixture()
    noetic_result(manipulator, code=code)
    ok, reason = check(planner)
    assert not ok and 'OBSERVATION_POSE_PLAN_FAILED' in reason
    assert 'moveit_error_code=%d' % code in reason
    assert 'OBSERVATION_PLANNING_TIMEOUT' not in reason
    assert planner._last_pose_plan is None and not planner._observation_branch_hints
    assert manipulator.plan_calls == 1 and not manipulator.executed_plans


@pytest.mark.parametrize('success,code', [(True, MoveItErrorCodes.TIMED_OUT),
    (True, MoveItErrorCodes.PLANNING_FAILED), (False, MoveItErrorCodes.SUCCESS),
    (False, '-6'), (False, True), (False, None)])
def test_inconsistent_or_untyped_noetic_result_cannot_become_timeout_or_success(success, code):
    planner, manipulator, _, _ = fixture()
    noetic_result(manipulator, success=success, code=code, empty=False)
    ok, reason = check(planner)
    assert not ok and 'OBSERVATION_PLANNING_RESULT_INVALID' in reason
    assert 'OBSERVATION_PLANNING_TIMEOUT' not in reason
    assert planner._last_pose_plan is None and not planner._observation_branch_hints
    assert manipulator.plan_calls == 1 and not manipulator.executed_plans


@pytest.mark.parametrize('empty', [False, True])
def test_real_success_code_still_requires_an_executable_trajectory(empty):
    planner, manipulator, _, _ = fixture()
    noetic_result(manipulator, success=True, code=MoveItErrorCodes.SUCCESS, empty=empty)
    ok, reason = check(planner)
    assert ok is (not empty)
    if empty:
        assert 'OBSERVATION_BRANCH_TRAJECTORY_INVALID' in reason
        assert planner._last_pose_plan is None and not planner._observation_branch_hints
    else:
        assert planner._last_pose_plan is not None
    assert 'OBSERVATION_PLANNING_TIMEOUT' not in reason
    assert manipulator.plan_calls == 1 and not manipulator.executed_plans


@pytest.mark.parametrize('flag', [1, 'true', None, 'trajectory'])
def test_four_item_result_requires_a_real_bool_success_flag(flag):
    planner, manipulator, _, _ = fixture()
    if flag == 'trajectory':
        # The legacy tuple contract regards its first item as a trajectory.
        # A malformed Noetic flag must never escape into that compatibility path.
        flag = JointPlan([[0.] * 6, list(manipulator.new_goal)], duration_sec=30.)
    noetic_result(manipulator, success=flag, empty=False)

    ok, reason = check(planner)

    assert not ok and 'OBSERVATION_PLANNING_RESULT_INVALID' in reason
    assert 'OBSERVATION_PLANNING_TIMEOUT' not in reason
    assert planner._last_pose_plan is None and not planner._observation_branch_hints
    assert manipulator.plan_calls == 1 and not manipulator.executed_plans


@pytest.mark.parametrize('legacy', ['bare_empty', 'two_item_false', 'transport_exception'])
def test_failure_without_a_noetic_timeout_code_is_not_invented_as_timeout(legacy):
    planner, manipulator, _, _ = fixture()
    empty = NS(joint_trajectory=NS(points=[]))
    def plan():
        manipulator.plan_calls += 1
        if legacy == 'transport_exception':
            raise TimeoutError('fixture transport wait; no MoveIt error code')
        return (False, empty) if legacy == 'two_item_false' else empty
    manipulator.plan = plan
    ok, reason = check(planner)
    assert not ok and 'OBSERVATION_PLANNING_TIMEOUT' not in reason
    assert planner._last_pose_plan is None and not planner._observation_branch_hints
    assert manipulator.plan_calls == 1 and not manipulator.executed_plans


def test_expired_branch_cannot_be_used_when_new_pose_planning_times_out():
    planner, manipulator, clock, _ = fixture()
    assert check(planner)[0]
    clock[0] += planner.OBSERVATION_BRANCH_HINT_MAX_AGE_SEC + .1
    noetic_result(manipulator)
    ok, reason = check(planner)
    assert not ok and reason.startswith('OBSERVATION_PLANNING_TIMEOUT: ')
    assert 'planning_kind=pose' in reason
    assert len(manipulator.pose_targets) == 2 and not manipulator.joint_targets
    assert planner._last_pose_plan is None and not planner._observation_branch_hints
    assert not manipulator.executed_plans


def test_failed_joint_tolerance_restore_remains_a_failure_even_after_timeout():
    planner, manipulator, _, _ = fixture()
    assert check(planner)[0]
    noetic_result(manipulator)
    setter = manipulator.set_goal_joint_tolerance
    def cannot_restore(value):
        if value != 0.:
            raise RuntimeError('fixture restore failure')
        setter(value)
    manipulator.set_goal_joint_tolerance = cannot_restore
    ok, reason = check(planner)
    assert not ok and 'OBSERVATION_BRANCH_JOINT_TOLERANCE_RESTORE_FAILED' in reason
    assert 'OBSERVATION_PLANNING_TIMEOUT' not in reason
    assert planner._last_pose_plan is None and not planner._observation_branch_hints


def test_real_observation_gateway_preserves_timeout_in_bool_reason_contract(monkeypatch):
    from test_motion_gateway_controller_start import MotionGateway, motion_gateway_node
    planner, manipulator, _, _ = fixture()
    noetic_result(manipulator)
    planner.strict_execution_max_joint_velocity_rad_s = .08
    planner.strict_execution_velocity_scaling = .3
    planner.strict_execution_acceleration_scaling = .2
    node = MotionGateway.__new__(MotionGateway)
    old_planner = node.planner = object()
    node._observation_planner = planner
    node._configure_observation_timing = lambda p: ('offline-timing',)
    node._configure_controller_reference_guard = lambda p: None
    monkeypatch.setattr(motion_gateway_node.rospy, 'loginfo', lambda *a, **k: None)
    monkeypatch.setattr(motion_gateway_node.rospy, 'logwarn', lambda *a, **k: None)
    target = make_pose()
    planner.expected_pose = deepcopy(target)

    response = node.handle_observation_pose_strict(NS(target=target, execute=False))

    assert response.success is False
    assert response.message.startswith('OBSERVATION_PLANNING_TIMEOUT: ')
    assert 'moveit_error_code=-6' in response.message
    assert 'speed_profile=observation' in response.message
    assert node.planner is old_planner
    assert manipulator.plan_calls == 1 and not manipulator.executed_plans
    assert planner._last_pose_plan is None and not planner._observation_branch_hints
