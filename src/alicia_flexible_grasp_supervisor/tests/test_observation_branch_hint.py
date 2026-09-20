"""Offline branch preservation; these tests never initialize a ROS node."""
from copy import deepcopy
from types import SimpleNamespace as NS

import pytest

from test_moveit_planner_pose_feedback import (
    FakeManipulator, JointPlan, make_pose,
)
import test_moveit_planner_pose_feedback as shared


@pytest.fixture(autouse=True)
def offline_robot_description(monkeypatch):
    """Parameter reads must not silently depend on the real arm's ROS master.

    FK and robot state are already fakes in this module. This description is
    only the immutable identity checked before/after synthetic planning.
    """
    params = {'/robot_description': 'offline-test-description',
              '/robot_description_semantic': 'offline-test-semantic'}
    monkeypatch.setattr('alicia_flexible_grasp.robot.moveit_planner.rospy.get_param',
                        lambda name, default=None: params.get(name, default))
    return params


class BranchManipulator(FakeManipulator):
    def __init__(self):
        super().__init__(current_joint_values=[0.] * 6)
        self.joint_targets = []
        self.fixed_goal = None
        self.new_goal = [0.2, 0.3, 0.1, 1.8, 0.1, 0.4]
        self.fail_plan = False
        self.wrong_endpoint = False
        self.goal_joint_tolerance = .0001
        self.joint_tolerance_updates = []

    def get_active_joints(self):
        return ['Joint%d' % i for i in range(1, 7)]

    def get_planning_frame(self):
        return 'base_link'

    def get_goal_position_tolerance(self):
        return .0001  # production kinematics.yaml

    def get_goal_orientation_tolerance(self):
        return .001

    def get_goal_joint_tolerance(self):
        return self.goal_joint_tolerance

    def set_goal_joint_tolerance(self, value):
        self.goal_joint_tolerance = float(value)
        self.joint_tolerance_updates.append(float(value))

    def set_pose_target(self, pose):
        self.fixed_goal = None
        super().set_pose_target(pose)

    def set_joint_value_target(self, goal):
        self.fixed_goal = dict(goal)
        self.joint_targets.append(dict(goal))

    def plan(self):
        self.plan_calls += 1
        if self.fail_plan:
            return NS(joint_trajectory=NS(points=[]))
        start = self.start_states[-1].joint_state.position
        goal = (list(self.new_goal) if self.fixed_goal is None else
                [self.fixed_goal[name] for name in self.get_active_joints()])
        if self.wrong_endpoint:
            goal[5] += .2
        return JointPlan([list(start), goal], duration_sec=30.)


def fixture():
    manipulator = BranchManipulator()
    planner = shared.MoveItPlannerPoseFeedbackTest().make_planner(manipulator)
    planner.observation_branch_hints_enabled = True
    planner.manipulator_group = 'alicia'
    clock = [10.]
    signature = ['model-one']
    planner._observation_branch_clock = lambda: clock[0]
    planner._observation_branch_signature = lambda: (
        signature[0], tuple(manipulator.get_active_joints()))
    planner.robot = NS(get_current_state=lambda:
        shared.MoveItPlannerPoseFeedbackTest.robot_state(*manipulator.current_joint_values))
    planner.expected_pose = make_pose()
    planner._forward_kinematics_from_state = lambda *_a, **_kw: (
        deepcopy(planner.expected_pose), 'offline FK')
    return planner, manipulator, clock, signature


def check(planner, pose=None):
    pose = pose or make_pose()
    planner.expected_pose = deepcopy(pose)
    return planner.move_to_pose(pose, execute=False, allow_fallbacks=False)


def test_other_candidate_cannot_overwrite_exact_target_joint_branch():
    planner, manipulator, _, _ = fixture()
    assert check(planner)[0]
    original = deepcopy(planner._last_pose_plan['observation_branch'])
    manipulator.new_goal = [0., 0., 0., 0., 0., 2.8]
    assert check(planner, make_pose(x=.4))[0]
    assert check(planner)[0]
    assert len(manipulator.pose_targets) == 2
    assert len(manipulator.joint_targets) == 1
    assert list(manipulator.joint_targets[-1].values()) == original['goal_joint_positions']
    assert planner._last_pose_plan['observation_branch']['goal_sha256'] == original['goal_sha256']
    assert planner._last_pose_plan['observation_branch']['mode'] == 'fresh_start_fixed_joint_goal'
    assert not manipulator.executed_plans


def test_description_changed_during_planning_rejects_without_ros_master(offline_robot_description):
    planner, manipulator, _, _ = fixture()
    original = manipulator.plan
    def changed_model():
        result = original()
        offline_robot_description['/robot_description'] = 'changed-during-plan'
        return result
    manipulator.plan = changed_model
    ok, reason = check(planner)
    assert not ok
    assert 'OBSERVATION_BRANCH_MODEL_CHANGED' in reason
    assert planner._last_pose_plan is None
    assert not manipulator.executed_plans


def test_prefix_consumes_old_trajectory_but_keeps_branch_for_fresh_start():
    planner, manipulator, _, _ = fixture()
    assert check(planner)[0]
    original_goal = planner._last_pose_plan['observation_branch']['goal_joint_positions']
    planner.strict_execution_retime_enabled = True
    planner._retime_strict_execution_plan = lambda p: (deepcopy(p), 'offline retime')
    ok, _ = planner.execute_cached_observation_prefix(make_pose())
    assert ok
    prefix = manipulator.executed_plans[-1]
    assert max(abs(v) for v in prefix.joint_trajectory.points[-1].positions) == pytest.approx(.025)
    assert planner._last_pose_plan is None
    assert len(planner._observation_branch_hints) == 1
    manipulator.current_joint_values = list(prefix.joint_trajectory.points[-1].positions)
    assert check(planner)[0]
    assert manipulator.start_states[-1].joint_state.position == manipulator.current_joint_values
    assert planner._last_pose_plan['plan'].joint_trajectory.points[0].positions == manipulator.current_joint_values
    assert planner._last_pose_plan['plan'].joint_trajectory.points[-1].positions == original_goal
    assert len(manipulator.executed_plans) == 1  # replan itself never executes.


@pytest.mark.parametrize('invalidate', ['expired', 'clock_backwards', 'model_changed'])
def test_stale_or_different_model_hint_is_not_used(invalidate):
    planner, manipulator, clock, signature = fixture()
    assert check(planner)[0]
    if invalidate == 'expired':
        clock[0] += planner.OBSERVATION_BRANCH_HINT_MAX_AGE_SEC + .1
    elif invalidate == 'clock_backwards':
        clock[0] -= 1.
    else:
        signature[0] = 'new-model'
    assert check(planner)[0]
    assert len(manipulator.pose_targets) == 2
    assert not manipulator.joint_targets
    assert len(planner._observation_branch_hints) == 1


def test_hint_reuse_never_renews_original_lifetime_and_capacity_is_bounded():
    planner, _, clock, _ = fixture()
    assert check(planner)[0]
    born = next(iter(planner._observation_branch_hints.values()))['created_sec']
    clock[0] += 10.
    assert check(planner)[0]
    assert next(iter(planner._observation_branch_hints.values()))['created_sec'] == born
    planner.OBSERVATION_BRANCH_HINT_CAPACITY = 2
    assert check(planner, make_pose(x=.2))[0]
    assert check(planner, make_pose(x=.3))[0]
    assert len(planner._observation_branch_hints) == 2
    assert all(key[1][0] != make_pose().position.x for key in planner._observation_branch_hints)


def test_nearby_pose_cannot_borrow_hint_through_old_cache_tolerance():
    planner, manipulator, _, _ = fixture()
    assert check(planner)[0]
    assert check(planner, make_pose(x=make_pose().position.x + 1e-7))[0]
    assert len(manipulator.pose_targets) == 2
    assert not manipulator.joint_targets


@pytest.mark.parametrize('failure', ['planning', 'fk_position', 'fk_orientation', 'joint_goal'])
def test_failed_hint_revalidation_never_falls_back_to_random_ik(failure):
    planner, manipulator, _, _ = fixture()
    assert check(planner)[0]
    if failure == 'planning':
        manipulator.fail_plan = True
    elif failure == 'joint_goal':
        manipulator.wrong_endpoint = True
    else:
        wrong = make_pose(x=.7) if failure == 'fk_position' else make_pose(q=(0., 0., 1., 0.))
        planner._forward_kinematics_from_state = lambda *_a, **_kw: (wrong, 'offline FK mismatch')
    ok, message = check(planner)
    assert not ok
    assert 'OBSERVATION_BRANCH_' in message
    assert len(manipulator.pose_targets) == 1
    assert planner._last_pose_plan is None
    assert not planner._observation_branch_hints
    assert not manipulator.executed_plans
    assert manipulator.goal_joint_tolerance == .0001


def test_fixed_goal_sampling_cannot_spend_original_near_boundary_fk_margin():
    planner, manipulator, _, _ = fixture()
    # Captured production branch from 2026-09-11 20:49:55.365. FK below is a
    # synthetic near-boundary model, not a claimed replay of the real service.
    goal = [-1.5672166919174038, -.19451014207462197, .285882434544522,
            1.2545809774539374, -.22562771694338063, -2.8033464389877953]
    manipulator.new_goal = list(goal)
    plan = manipulator.plan
    sampling_tolerances = []
    def sampling_plan():
        result = plan()
        if manipulator.fixed_goal is not None:
            sampling_tolerances.append(manipulator.goal_joint_tolerance)
            result.joint_trajectory.points[-1].positions[0] += manipulator.goal_joint_tolerance / 2.
        return result
    manipulator.plan = sampling_plan
    def near_boundary_fk(state, *_args):
        by_name = dict(zip(state.joint_state.name, state.joint_state.position))
        return make_pose(x=make_pose().position.x + .000099 + by_name['Joint1'] - goal[0]), 'synthetic near-boundary FK'
    planner._forward_kinematics_from_state = near_boundary_fk
    assert check(planner)[0]
    assert manipulator.joint_tolerance_updates == []
    assert check(planner)[0]
    assert sampling_tolerances == [0.]
    assert manipulator.joint_tolerance_updates == [0., .0001]
    assert manipulator.goal_joint_tolerance == .0001
    assert planner._last_pose_plan['plan'].joint_trajectory.points[-1].positions == goal


def test_fixed_goal_tolerance_is_restored_on_planner_exception():
    planner, manipulator, _, _ = fixture()
    assert check(planner)[0]
    def raises():
        assert manipulator.goal_joint_tolerance == 0.
        raise RuntimeError('offline planner transport failure')
    manipulator.plan = raises
    assert not check(planner)[0]
    assert manipulator.joint_tolerance_updates == [0., .0001]
    assert manipulator.goal_joint_tolerance == .0001
    assert planner._last_pose_plan is None


def test_fixed_goal_rejects_a_planner_that_ignores_zero_sampling_tolerance():
    planner, manipulator, _, _ = fixture()
    assert check(planner)[0]
    original_plan = manipulator.plan
    def ignores_constraint():
        result = original_plan()
        result.joint_trajectory.points[-1].positions[0] += .000001
        return result
    manipulator.plan = ignores_constraint
    ok, message = check(planner)
    assert not ok
    assert 'OBSERVATION_BRANCH_JOINT_GOAL_CHANGED' in message
    assert manipulator.goal_joint_tolerance == .0001


def test_failed_tolerance_restore_cannot_leave_an_executable_cache():
    planner, manipulator, _, _ = fixture()
    assert check(planner)[0]
    setter = manipulator.set_goal_joint_tolerance
    def cannot_restore(value):
        if value != 0.:
            raise RuntimeError('offline restore failure')
        setter(value)
    manipulator.set_goal_joint_tolerance = cannot_restore
    ok, message = check(planner)
    assert not ok
    assert 'TOLERANCE_RESTORE_FAILED' in message
    assert planner._last_pose_plan is None
    assert not planner._observation_branch_hints


def test_invalid_old_joint_tolerance_is_not_written_back_to_moveit():
    planner, manipulator, _, _ = fixture()
    assert check(planner)[0]
    manipulator.get_goal_joint_tolerance = lambda: float('nan')
    ok, message = check(planner)
    assert not ok
    assert 'TOLERANCE_INVALID' in message
    assert manipulator.joint_tolerance_updates == []


def test_original_orientation_is_never_replaced_by_fk_result():
    planner, _, _, _ = fixture()
    pose = make_pose(q=(.1, .2, .3, .9))
    original = deepcopy(vars(pose.orientation))
    assert check(planner, pose)[0]
    assert check(planner, pose)[0]
    assert vars(pose.orientation) == original


def test_fk_request_is_bound_to_planning_frame_and_rejects_another_frame():
    planner, _, _, _ = fixture()
    headers = []
    def fk(_state, header):
        headers.append(header)
        return NS(header=NS(frame_id='camera_link'), pose=make_pose()), 'wrong frame'
    planner._forward_kinematics_from_state = fk
    ok, message = check(planner)
    assert not ok
    assert headers[0].frame_id == 'base_link'
    assert 'FK_FRAME_MISMATCH' in message
    assert planner._last_pose_plan is None


def test_orientation_uses_existing_per_axis_bounds_not_a_new_so3_bound():
    from tf.transformations import quaternion_from_euler
    planner, _, _, _ = fixture()
    within_axes = make_pose(q=tuple(quaternion_from_euler(.0009999, .0009999, .0009999, axes='rxyz')))
    planner._forward_kinematics_from_state = lambda *_a, **_kw: (within_axes, 'offline FK')
    assert check(planner)[0]  # total angle > .001; intrinsic axes all pass.
    too_far = make_pose(q=tuple(quaternion_from_euler(.0011, 0., 0.)))
    planner._forward_kinematics_from_state = lambda *_a, **_kw: (too_far, 'offline FK')
    assert not check(planner)[0]


@pytest.mark.parametrize('distance, allowed', [(.00009, True), (.00011, False)])
def test_fk_preserves_production_position_tolerance(distance, allowed):
    planner, _, _, _ = fixture()
    shifted = make_pose(x=make_pose().position.x + distance)
    planner._forward_kinematics_from_state = lambda *_a, **_kw: (shifted, 'offline FK')
    assert check(planner)[0] is allowed


def test_contact_planner_does_not_enter_observation_hint_path():
    planner, manipulator, _, _ = fixture()
    planner.observation_branch_hints_enabled = False
    planner._observation_branch_signature = lambda: pytest.fail('contact must not access hints')
    # Ordinary planner uses its original set_start_state_to_current_state API.
    manipulator.start_states = [shared.MoveItPlannerPoseFeedbackTest.robot_state(*([0.] * 6))]
    assert check(planner)[0]
    assert check(planner)[0]
    assert len(manipulator.pose_targets) == 2
    assert not manipulator.joint_targets


def test_model_signature_includes_actual_descriptions_and_tool(monkeypatch):
    planner, manipulator, _, _ = fixture()
    del planner._observation_branch_signature
    params = {'/robot_description': 'urdf-one', '/robot_description_semantic': 'srdf-one'}
    monkeypatch.setattr('alicia_flexible_grasp.robot.moveit_planner.rospy.get_param',
                        lambda name, default=None: params.get(name, default))
    first = planner._observation_branch_signature()[0]
    params['/robot_description'] = 'urdf-two'
    second = planner._observation_branch_signature()[0]
    assert first != second
    manipulator.get_end_effector_link = lambda: 'different_tool'
    assert planner._observation_branch_signature()[0] != second
    params.clear()
    with pytest.raises(ValueError, match='MODEL_UNAVAILABLE'):
        planner._observation_branch_signature()
