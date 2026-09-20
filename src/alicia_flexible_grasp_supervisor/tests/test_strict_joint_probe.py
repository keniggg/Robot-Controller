from copy import deepcopy

import pytest

from test_observation_branch_hint import fixture


def test_exact_joint_probe_never_goes_through_pose_ik():
    planner, arm, _, _ = fixture()
    goal = [0., .006, .006, 0., .006, 0.]
    ok, reason = planner.plan_and_execute_joint_probe(goal, execute=False)
    assert ok, reason
    assert arm.joint_targets[-1] == dict(zip(arm.get_active_joints(), goal))
    assert arm.pose_targets == []
    assert arm.goal_joint_tolerance == .0001
    assert arm.joint_tolerance_updates == [0., .0001]
    assert not arm.executed_plans
    assert planner._last_pose_plan is None


def test_exact_joint_probe_rejects_planner_endpoint_sampling_and_leaves_no_cache():
    planner, arm, _, _ = fixture()
    arm.wrong_endpoint = True
    ok, reason = planner.plan_and_execute_joint_probe([.005]*6, execute=False)
    assert not ok and 'JOINT_PROBE_GOAL_CHANGED' in reason
    assert planner._last_pose_plan is None
    assert not arm.executed_plans
    assert arm.goal_joint_tolerance == .0001


def test_exact_joint_probe_uses_common_retiming_reference_and_final_path_guard():
    planner, arm, _, _ = fixture()
    events = []
    planner._retime_strict_execution_plan = lambda p: (events.append('retime') or deepcopy(p), '')
    planner._prepare_controller_reference_timing = lambda p: (p, None, events.append('reference') or '')
    planner._observation_path_guard_error = lambda p: events.append('cad') or 'probe rejection'
    ok, reason = planner.plan_and_execute_joint_probe([.005]*6, execute=True)
    assert not ok and 'probe rejection' in reason
    assert events == ['retime', 'reference', 'cad']
    assert not arm.executed_plans
    assert planner._last_pose_plan is None


@pytest.mark.parametrize('goal', [[0.]*5, [0.]*7, [float('nan')]*6, [float('inf')]*6])
def test_invalid_joint_probe_shape_rejects_without_planning(goal):
    planner, arm, _, _ = fixture()
    ok, _ = planner.plan_and_execute_joint_probe(goal, execute=False)
    assert not ok
    assert not arm.plan_calls


def test_transmitted_reference_removes_artificial_measured_start_backtrack():
    planner, arm, _, _ = fixture()
    arm.current_joint_values = [0., -.007, -.014, 0., -.011, 0.]
    ok, reason = planner.plan_and_execute_joint_probe(
        [0., .0107378655, 0., 0., 0., 0.], execute=False,
        start_joint_positions=[0.]*6)
    assert ok, reason
    assert arm.start_states[-1].joint_state.position == [0.]*6
    assert arm.current_joint_values == [0., -.007, -.014, 0., -.011, 0.]
    assert not arm.executed_plans


def test_reference_cannot_hide_feedback_outside_original_following_contract():
    planner, arm, _, _ = fixture()
    arm.current_joint_values = [0., -.036, 0., 0., 0., 0.]
    ok, reason = planner.plan_and_execute_joint_probe([.005]*6, execute=True,
                                                   start_joint_positions=[0.]*6)
    assert not ok and 'REFERENCE_FOLLOWING_EXCEEDED' in reason
    assert not arm.plan_calls and not arm.executed_plans


def test_submission_hook_is_after_geometry_check_and_cannot_run_on_rejection():
    planner, arm, _, _ = fixture()
    events = []
    planner._observation_path_guard_error = lambda p: 'geometry rejected'
    ok, reason = planner.plan_and_execute_joint_probe([.005]*6, execute=True,
        before_execute=lambda: events.append('consumed'))
    assert not ok and 'geometry rejected' in reason
    assert events == [] and not arm.executed_plans


def test_submission_hook_failure_prevents_physical_submission():
    planner, arm, _, _ = fixture()
    def blocked():
        raise ValueError('consumption persistence failed')
    ok, reason = planner.plan_and_execute_joint_probe([.005]*6, execute=True,
                                                     before_execute=blocked)
    assert not ok and 'consumption persistence failed' in reason
    assert not arm.executed_plans
