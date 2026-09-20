#!/usr/bin/env python3
import pathlib
import sys
import types
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / 'src'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from alicia_flexible_grasp.robot.moveit_planner import MoveItPlanner


def make_pose(x=0.123, y=-0.456, z=2.5, q=None):
    position = types.SimpleNamespace(x=x, y=y, z=z)
    q = q or (0.0, 0.0, 0.0, 1.0)
    orientation = types.SimpleNamespace(x=q[0], y=q[1], z=q[2], w=q[3])
    return types.SimpleNamespace(position=position, orientation=orientation)


class EmptyPlan:
    joint_trajectory = types.SimpleNamespace(points=[])


class SuccessfulPlan:
    joint_trajectory = types.SimpleNamespace(points=[object()])


class JointPlan:
    def __init__(self, positions, duration_sec=None):
        points = []
        for index, values in enumerate(positions):
            point = types.SimpleNamespace(positions=list(values))
            if duration_sec is not None:
                denominator = max(1, len(positions) - 1)
                point.time_from_start = types.SimpleNamespace(
                    to_sec=lambda value=float(duration_sec) * index / denominator: value
                )
            points.append(point)
        self.joint_trajectory = types.SimpleNamespace(
            joint_names=['Joint%d' % (index + 1) for index in range(len(positions[0]))],
            points=points,
        )


class FakeManipulator:
    def __init__(
        self,
        go_result=False,
        plan_result=None,
        current_pose=None,
        execute_result=True,
        cartesian_result=None,
        current_joint_values=None,
    ):
        self.go_results = list(go_result) if isinstance(go_result, (list, tuple)) else [go_result]
        if isinstance(plan_result, (list, tuple)):
            self.plan_results = list(plan_result)
        else:
            self.plan_results = [plan_result if plan_result is not None else EmptyPlan()]
        self.current_pose = current_pose
        self.execute_result = execute_result
        self.cartesian_result = cartesian_result
        self.current_joint_values = current_joint_values
        self.target = None
        self.pose_targets = []
        self.position_targets = []
        self.executed_plans = []
        self.go_calls = 0
        self.plan_calls = 0
        self.stopped = False
        self.cleared = False
        self.planning_time = 2.0
        self.planning_time_updates = []
        self.cartesian_calls = []
        self.start_state_to_current_calls = 0
        self.start_states = []

    def set_pose_target(self, pose):
        self.target = pose
        self.pose_targets.append(pose)

    def set_position_target(self, position):
        self.position_targets.append(list(position))

    def go(self, wait=True):
        self.go_calls += 1
        if len(self.go_results) > 1:
            return self.go_results.pop(0)
        return self.go_results[0]

    def plan(self):
        self.plan_calls += 1
        if len(self.plan_results) > 1:
            return self.plan_results.pop(0)
        return self.plan_results[0]

    def execute(self, plan, wait=True):
        self.executed_plans.append(plan)
        return self.execute_result

    def compute_cartesian_path(self, waypoints, eef_step, jump_threshold, avoid_collisions=True):
        self.cartesian_calls.append((list(waypoints), eef_step, jump_threshold, avoid_collisions))
        if self.cartesian_result is not None:
            return self.cartesian_result
        return SuccessfulPlan(), 1.0

    def stop(self):
        self.stopped = True

    def clear_pose_targets(self):
        self.cleared = True

    def get_current_pose(self):
        return self.current_pose

    def get_current_joint_values(self):
        if self.current_joint_values is None:
            raise RuntimeError('joint feedback unavailable')
        return list(self.current_joint_values)

    def get_planning_time(self):
        return self.planning_time

    def set_planning_time(self, value):
        self.planning_time = float(value)
        self.planning_time_updates.append(float(value))

    def set_start_state_to_current_state(self):
        self.start_state_to_current_calls += 1

    def set_start_state(self, state):
        self.start_states.append(state)

    def get_end_effector_link(self):
        return 'tool0'


class FakeNoeticManipulator(FakeManipulator):
    def compute_cartesian_path(
        self,
        waypoints,
        eef_step,
        avoid_collisions=True,
        path_constraints=None,
    ):
        self.cartesian_calls.append(
            (list(waypoints), eef_step, avoid_collisions, path_constraints)
        )
        if self.cartesian_result is not None:
            return self.cartesian_result
        return SuccessfulPlan(), 1.0


class MoveItPlannerPoseFeedbackTest(unittest.TestCase):
    def make_planner(self, manipulator):
        planner = MoveItPlanner.__new__(MoveItPlanner)
        planner.ready = True
        planner.error = None
        planner.manipulator = manipulator
        planner.strict_pose_planning_time = 0.25
        planner.orientation_resolution_planning_time = 2.0
        planner.orientation_resolution_step_rad = 0.4
        planner.orientation_resolution_max_candidates = 8
        planner.orientation_resolution_ik_timeout_sec = 0.05
        planner.orientation_resolution_repeatability_tolerance_rad = 1e-9
        planner.cached_plan_position_tolerance_m = 0.002
        planner.cached_plan_orientation_tolerance_rad = 0.02
        planner.execution_goal_tolerance_rad = 0.03
        planner.execution_goal_tolerance_slack_rad = 0.005
        planner.strict_execution_retime_enabled = False
        planner.cartesian_eef_step_m = 0.003
        planner.cartesian_jump_threshold = 0.0
        planner.cartesian_min_fraction = 0.98
        planner.cartesian_max_segment_m = 0.08
        return planner

    @staticmethod
    def robot_state(*positions):
        return types.SimpleNamespace(
            joint_state=types.SimpleNamespace(
                name=['Joint%d' % (index + 1) for index in range(len(positions))],
                position=list(positions),
                velocity=[],
                effort=[],
            )
        )

    def test_plan_metrics_expose_per_joint_hardware_duration_lower_bound(self):
        planner = self.make_planner(FakeManipulator())
        planner.strict_execution_max_joint_velocity_rad_s = 0.08
        planner.strict_execution_joint_velocity_limits_rad_s = {
            'Joint3': 0.02,
            'Joint4': 0.02,
            'Joint6': 0.02,
        }
        plan = JointPlan(
            [
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.8, 0.0, 0.3, 0.0, 0.0, 0.5],
            ]
        )

        metrics = planner._plan_joint_path_metrics(plan)

        self.assertAlmostEqual(
            metrics['execution_duration_lower_bound_sec'],
            25.0,
        )
        self.assertEqual(metrics['hardware_limiting_joint'], 'Joint6')
        self.assertAlmostEqual(metrics['joint_duration_lower_bound_sec'], 25.0)

    def test_strict_sequence_sums_stage_duration_lower_bounds(self):
        planner = self.make_planner(FakeManipulator())
        planner.strict_execution_max_joint_velocity_rad_s = 0.1
        planner.strict_execution_joint_velocity_limits_rad_s = {}
        planner.robot = types.SimpleNamespace(
            get_current_state=lambda: self.robot_state(0.0, 0.0)
        )
        plans = iter([
            JointPlan([[0.0, 0.0], [0.2, 0.0]]),
            JointPlan([[0.2, 0.0], [0.2, 0.3]]),
        ])
        planner._plan_pose_from_start_state = (
            lambda _state, _target, **_kwargs: (next(plans), 'planned')
        )

        ok, code, failed_stage, metrics, message = planner.check_pose_sequence(
            [make_pose(x=0.1), make_pose(x=0.2)],
            ['pregrasp', 'lift'],
            [False, False],
        )

        self.assertTrue(ok, message)
        self.assertEqual(code, '')
        self.assertEqual(failed_stage, '')
        # 0.2 / 0.1 + 0.3 / 0.1, summed because stages execute serially.
        self.assertAlmostEqual(metrics['joint_duration_lower_bound_sec'], 5.0)
        self.assertAlmostEqual(metrics['execution_duration_lower_bound_sec'], 5.0)
        self.assertEqual(metrics['hardware_limiting_joint'], 'Joint2')
        self.assertIn('joint_duration_lower_bound_sec=5.000', message)

    def test_strict_sequence_rejects_nonfinite_trajectory_fail_closed(self):
        planner = self.make_planner(FakeManipulator())
        planner.robot = types.SimpleNamespace(
            get_current_state=lambda: self.robot_state(0.0, 0.0)
        )
        bad_plan = JointPlan([[0.0, 0.0], [float('nan'), 0.1]])
        planner._plan_pose_from_start_state = (
            lambda _state, _target, **_kwargs: (bad_plan, 'planned')
        )

        ok, code, failed_stage, metrics, _message = planner.check_pose_sequence(
            [make_pose()], ['pregrasp'], [False]
        )

        self.assertFalse(ok)
        self.assertEqual(code, 'MOVEIT_CHECK_ERROR')
        self.assertEqual(failed_stage, '')
        self.assertEqual(metrics['joint_duration_lower_bound_sec'], 0.0)

    def test_strict_sequence_rejects_empty_trajectory_fail_closed(self):
        planner = self.make_planner(FakeManipulator())
        planner.robot = types.SimpleNamespace(
            get_current_state=lambda: self.robot_state(0.0, 0.0)
        )
        planner._plan_pose_from_start_state = (
            lambda _state, _target, **_kwargs: (EmptyPlan(), 'empty')
        )

        ok, code, failed_stage, _metrics, _message = planner.check_pose_sequence(
            [make_pose()], ['pregrasp'], [False]
        )

        self.assertFalse(ok)
        self.assertEqual(code, 'MOVEIT_UNREACHABLE')
        self.assertEqual(failed_stage, 'pregrasp')

    def test_strict_sequence_uses_each_planned_terminal_state(self):
        pregrasp_plan = JointPlan([[0.0, 0.0], [0.2, 0.3]])
        approach_plan = JointPlan([[0.2, 0.3], [0.25, 0.35]])
        grasp_plan = JointPlan([[0.25, 0.35], [0.3, 0.4]])
        manipulator = FakeManipulator(plan_result=pregrasp_plan)
        planner = self.make_planner(manipulator)
        planner.manipulator_group = 'alicia'
        planner.robot = types.SimpleNamespace(
            get_current_state=lambda: self.robot_state(0.0, 0.0)
        )
        cartesian_states = []
        cartesian_plans = iter([approach_plan, grasp_plan])

        def cartesian(start_state, _target):
            cartesian_states.append(list(start_state.joint_state.position))
            return next(cartesian_plans), 'planned'

        planner._plan_cartesian_from_start_state = cartesian
        targets = [make_pose(x=0.1), make_pose(x=0.2), make_pose(x=0.3)]

        ok, code, failed_stage, metrics, message = (
            planner.check_pose_sequence(
                targets,
                ['pregrasp', 'approach', 'grasp'],
                [False, True, True],
            )
        )

        self.assertTrue(ok, message)
        self.assertEqual(code, '')
        self.assertEqual(failed_stage, '')
        self.assertEqual(
            cartesian_states,
            [[0.2, 0.3], [0.25, 0.35]],
        )
        self.assertAlmostEqual(metrics['path_cost'], 0.5019764838)
        self.assertAlmostEqual(metrics['max_delta'], 0.3)
        self.assertIn('stages=pregrasp,approach,grasp', message)

    def test_strict_sequence_reports_failed_virtual_stage(self):
        pregrasp_plan = JointPlan([[0.0, 0.0], [0.2, 0.3]])
        manipulator = FakeManipulator(plan_result=pregrasp_plan)
        planner = self.make_planner(manipulator)
        planner.manipulator_group = 'alicia'
        planner.robot = types.SimpleNamespace(
            get_current_state=lambda: self.robot_state(0.0, 0.0)
        )
        planner._plan_cartesian_from_start_state = (
            lambda _state, _target: (None, 'fraction 0.50')
        )

        ok, code, failed_stage, _metrics, message = (
            planner.check_pose_sequence(
                [make_pose(x=0.1), make_pose(x=0.2)],
                ['pregrasp', 'approach'],
                [False, True],
            )
        )

        self.assertFalse(ok)
        self.assertEqual(code, 'MOVEIT_UNREACHABLE')
        self.assertEqual(failed_stage, 'approach')
        self.assertIn('fraction 0.50', message)

    def test_strict_sequence_stops_before_stage_after_shared_deadline(self):
        planner = self.make_planner(FakeManipulator())
        planner.robot = types.SimpleNamespace(
            get_current_state=lambda: self.robot_state(0.0, 0.0)
        )
        clock = iter([10.0, 10.5, 11.0])
        planner._planning_now_sec = lambda: next(clock)
        calls = []
        planner._plan_pose_from_start_state = (
            lambda _state, _target, **_kwargs: (
                calls.append('pregrasp')
                or JointPlan([[0.0, 0.0], [0.1, 0.1]]),
                'planned',
            )
        )

        ok, code, failed_stage, _metrics, message = (
            planner.check_pose_sequence(
                [make_pose(x=0.1), make_pose(x=0.2)],
                ['pregrasp', 'lift'],
                [False, False],
                deadline_sec=11.0,
            )
        )

        self.assertFalse(ok)
        self.assertEqual(code, 'MOVEIT_TIMEOUT')
        self.assertEqual(failed_stage, 'lift')
        self.assertEqual(calls, ['pregrasp'])
        self.assertIn('deadline', message)

    def test_strict_sequence_accepts_full_five_stage_execution_order(self):
        observation_plan = JointPlan([[0.0, 0.0], [0.1, 0.1]])
        pregrasp_plan = JointPlan([[0.1, 0.1], [0.2, 0.2]])
        approach_plan = JointPlan([[0.2, 0.2], [0.25, 0.25]])
        grasp_plan = JointPlan([[0.25, 0.25], [0.3, 0.3]])
        lift_plan = JointPlan([[0.3, 0.3], [0.4, 0.4]])
        planner = self.make_planner(FakeManipulator())
        planner.manipulator_group = 'alicia'
        planner.robot = types.SimpleNamespace(
            get_current_state=lambda: self.robot_state(0.0, 0.0)
        )
        pose_plans = iter(
            [observation_plan, pregrasp_plan, lift_plan]
        )
        cartesian_plans = iter([approach_plan, grasp_plan])
        planner._plan_pose_from_start_state = (
            lambda _state, _target: (next(pose_plans), 'planned')
        )
        planner._plan_cartesian_from_start_state = (
            lambda _state, _target: (next(cartesian_plans), 'planned')
        )

        ok, code, failed_stage, _metrics, message = (
            planner.check_pose_sequence(
                [make_pose(x=0.1 * index) for index in range(1, 6)],
                [
                    'observation',
                    'pregrasp',
                    'approach',
                    'grasp',
                    'lift',
                ],
                [False, False, True, True, False],
            )
        )

        self.assertTrue(ok, message)
        self.assertEqual(code, '')
        self.assertEqual(failed_stage, '')
        self.assertIn(
            'stages=observation,pregrasp,approach,grasp,lift',
            message,
        )

    def test_free_space_resolver_uses_repeatable_geodesic_ik_and_preserves_xyz(self):
        manipulator = FakeManipulator()
        planner = self.make_planner(manipulator)
        planner.manipulator_group = 'alicia'
        planner.robot = types.SimpleNamespace(
            get_current_state=lambda: self.robot_state(0.0, 0.0)
        )
        target = make_pose(
            x=0.10,
            y=-0.20,
            z=0.30,
            q=(0.0, 0.0, 0.0, 1.0),
        )
        fk_states = []

        def forward_kinematics(state, _header):
            positions = list(state.joint_state.position)
            fk_states.append(positions)
            if positions == [0.0, 0.0]:
                return (
                    make_pose(
                        x=0.0,
                        y=0.0,
                        z=0.0,
                        q=(0.0, 0.0, 0.70710678, 0.70710678),
                    ),
                    'initial anchor',
                )
            return (
                make_pose(
                    x=0.101,
                    y=-0.20,
                    z=0.30,
                    q=(0.0, 0.0, 0.3, 0.953939201),
                ),
                'resolved',
            )

        planner._forward_kinematics_from_state = forward_kinematics
        ik_quaternions = []

        def inverse_kinematics(_state, candidate):
            quaternion = candidate.orientation
            ik_quaternions.append(
                (
                    quaternion.x,
                    quaternion.y,
                    quaternion.z,
                    quaternion.w,
                )
            )
            magnitude = abs(float(quaternion.z))
            if magnitude < 0.1:
                return None, 'unreachable'
            return self.robot_state(magnitude, -0.5 * magnitude), 'resolved'

        planner._inverse_kinematics_from_state = inverse_kinematics

        ok, code, failed_stage, resolved, metrics, message = (
            planner.resolve_free_space_orientations(
                [target],
                ['pregrasp'],
                [False],
                [True],
            )
        )

        self.assertTrue(ok, message)
        self.assertEqual(code, '')
        self.assertEqual(failed_stage, '')
        self.assertEqual(fk_states[0], [0.0, 0.0])
        self.assertEqual(len(fk_states), 2)
        self.assertGreaterEqual(len(ik_quaternions), 3)
        self.assertIn(ik_quaternions[-1], ik_quaternions[:-1])
        self.assertEqual(ik_quaternions.count(ik_quaternions[-1]), 2)
        self.assertEqual(manipulator.position_targets, [])
        self.assertEqual(len(manipulator.pose_targets), 0)
        self.assertEqual(manipulator.planning_time_updates, [])
        self.assertAlmostEqual(resolved[0].position.x, 0.10)
        self.assertAlmostEqual(resolved[0].position.y, -0.20)
        self.assertAlmostEqual(resolved[0].position.z, 0.30)
        self.assertAlmostEqual(resolved[0].orientation.x, 0.0)
        self.assertAlmostEqual(resolved[0].orientation.y, 0.0)
        self.assertAlmostEqual(resolved[0].orientation.z, 0.3)
        self.assertAlmostEqual(resolved[0].orientation.w, 0.953939201)
        self.assertAlmostEqual(metrics['max_position_error'], 0.001)
        self.assertIsNone(planner._last_pose_plan)
        self.assertIn('stages=pregrasp', message)
        self.assertIn('policy=deterministic_geodesic_collision_ik', message)
        self.assertIn('max_repeatability_error=0.000000000', message)

    def test_free_space_resolver_rejects_nonrepeatable_selected_ik_seed(self):
        planner = self.make_planner(FakeManipulator())
        planner.orientation_resolution_max_candidates = 2
        planner.orientation_resolution_repeatability_tolerance_rad = 1e-6
        calls = []

        def inverse_kinematics(_state, _candidate):
            calls.append(len(calls) + 1)
            if len(calls) == 1:
                return self.robot_state(0.1, -0.1), 'first sample'
            if len(calls) == 2:
                return self.robot_state(0.2, -0.2), 'second sample'
            return self.robot_state(0.1001, -0.1), 'repeat changed'

        planner._inverse_kinematics_from_state = inverse_kinematics
        start = self.robot_state(0.0, 0.0)
        target = make_pose(q=(0.0, 0.0, 0.0, 1.0))
        anchor = make_pose(q=(0.0, 0.0, 0.70710678, 0.70710678))

        seed, code, reason = planner._resolve_orientation_from_state(
            start,
            target,
            anchor,
        )

        self.assertIsNone(seed)
        self.assertEqual(code, 'MOVEIT_RESOLVE_NONDETERMINISTIC')
        self.assertIn('repeatability error', reason)
        self.assertEqual(len(calls), 4)

    def test_orientation_resolver_does_not_start_ik_after_shared_deadline(self):
        planner = self.make_planner(FakeManipulator())
        planner.orientation_resolution_max_candidates = 8
        clock = iter([20.0, 20.1, 21.0])
        planner._planning_now_sec = lambda: next(clock)
        calls = []
        planner._inverse_kinematics_from_state = (
            lambda *_args, **_kwargs: (
                calls.append(True) or self.robot_state(0.1, -0.1),
                'resolved',
            )
        )

        seed, code, reason = planner._resolve_orientation_from_state(
            self.robot_state(0.0, 0.0),
            make_pose(q=(0.0, 0.0, 0.0, 1.0)),
            make_pose(q=(0.0, 0.0, 0.70710678, 0.70710678)),
            deadline_sec=21.0,
        )

        self.assertIsNone(seed)
        self.assertEqual(code, 'MOVEIT_TIMEOUT')
        self.assertEqual(len(calls), 1)
        self.assertIn('deadline', reason)

    def test_orientation_resolver_checks_next_sample_after_nonrepeatable_best(self):
        planner = self.make_planner(FakeManipulator())
        planner.orientation_resolution_max_candidates = 2
        calls = []

        def ik(_state, target):
            key = round(target.orientation.z, 3)
            calls.append(key)
            count = calls.count(key)
            value = (0.1 if count == 1 else 0.11) if key == 0.0 else 0.2
            return self.robot_state(value, -value), 'ik'

        target = make_pose(x=0.1, q=(0, 0, 0, 1))
        planner._inverse_kinematics_from_state = ik
        planner._forward_kinematics_from_state = lambda *_: (
            make_pose(x=0.1, q=(0, 0, 0.70710678, 0.70710678)), 'fk')
        seed, code, reason = planner._resolve_orientation_from_state(
            self.robot_state(0, 0), target,
            make_pose(q=(0, 0, 0.70710678, 0.70710678)))
        self.assertIsNotNone(seed, (code, reason))
        self.assertEqual(seed['terminal_state'].joint_state.position, [0.2, -0.2])
        self.assertEqual(calls, [0.0, 0.707, 0.0, 0.707])
        self.assertEqual(planner.manipulator.executed_plans, [])

    def test_free_space_resolver_backtracks_when_low_cost_pregrasp_blocks_approach(self):
        planner = self.make_planner(FakeManipulator())
        planner.orientation_resolution_max_candidates = 2
        planner.robot = types.SimpleNamespace(get_current_state=lambda: self.robot_state(0, 0))
        planner._inverse_kinematics_from_state = lambda state, target: (
            self.robot_state(0.1 if abs(target.orientation.z) < 0.1 else 0.2, 0), 'ik')
        planner._forward_kinematics_from_state = lambda state, header: (
            make_pose(x=0.1, q=(0, 0, 0 if state.joint_state.position[0] == 0.1 else 0.70710678,
                               1 if state.joint_state.position[0] == 0.1 else 0.70710678)), 'fk')
        attempts = []

        def cartesian(state, target):
            attempts.append(state.joint_state.position[0])
            if state.joint_state.position[0] < 0.15:
                return None, 'Cartesian fraction 0.689 < 0.980'
            return JointPlan([[0.2, 0], [0.3, 0]]), 'complete'

        planner._plan_cartesian_from_start_state = cartesian
        targets = [make_pose(x=0.1), make_pose(x=0.12)]
        ok, code, stage, resolved, metrics, reason = planner.resolve_free_space_orientations(
            targets, ['pregrasp', 'approach'], [False, True], [True, False])
        self.assertTrue(ok, (code, stage, reason))
        self.assertEqual(attempts, [0.1, 0.2])
        self.assertEqual(resolved[1].orientation, targets[1].orientation)
        self.assertEqual(resolved[0].position, targets[0].position)
        self.assertEqual(planner.manipulator.executed_plans, [])

    def test_orientation_resolver_does_not_try_next_branch_after_terminal_suffix(self):
        for terminal in ('MOVEIT_TIMEOUT', 'MOVEIT_SEARCH_EXHAUSTED'):
            with self.subTest(terminal=terminal):
                planner = self.make_planner(FakeManipulator())
                planner.orientation_resolution_max_candidates = 2
                ik_calls, suffix_calls = [], []

                def ik(state, target):
                    ik_calls.append(True)
                    value = 0.1 + abs(target.orientation.z)
                    return self.robot_state(value, 0), 'repeatable'

                def suffix(seed):
                    suffix_calls.append(True)
                    return False, terminal, 'shared bound reached'

                planner._inverse_kinematics_from_state = ik
                planner._forward_kinematics_from_state = lambda *args: (make_pose(), 'fk')
                seed, code, reason = planner._resolve_orientation_from_state(
                    self.robot_state(0, 0), make_pose(),
                    make_pose(q=(0, 0, 0.70710678, 0.70710678)), accept_seed=suffix)
                self.assertIsNone(seed)
                self.assertEqual(code, terminal)
                self.assertEqual(len(ik_calls), 3)  # Two proposals, one repeat.
                self.assertEqual(suffix_calls, [True])
                self.assertEqual(planner.manipulator.executed_plans, [])

    def test_free_space_lift_seed_uses_virtual_grasp_terminal_state(self):
        approach_plan = JointPlan([[0.1, 0.2], [0.2, 0.3]])
        grasp_plan = JointPlan([[0.2, 0.3], [0.3, 0.4]])
        planner = self.make_planner(FakeManipulator())
        planner.robot = types.SimpleNamespace(
            get_current_state=lambda: self.robot_state(0.0, 0.0)
        )
        cartesian_plans = iter([approach_plan, grasp_plan])
        planner._plan_cartesian_from_start_state = (
            lambda _state, _target: (next(cartesian_plans), 'planned')
        )
        planner._forward_kinematics_from_state = (
            lambda _state, _header: (
                make_pose(q=(0.0, 0.0, 0.0, 1.0)),
                'initial anchor',
            )
        )
        resolver_calls = []

        def resolve_orientation(start_state, target, anchor, accept_seed=None):
            resolver_calls.append(
                (
                    list(start_state.joint_state.position),
                    (
                        anchor.orientation.x,
                        anchor.orientation.y,
                        anchor.orientation.z,
                        anchor.orientation.w,
                    ),
                )
            )
            index = len(resolver_calls)
            if index == 1:
                resolved = make_pose(
                    x=target.position.x,
                    y=target.position.y,
                    z=target.position.z,
                    q=(0.1, 0.2, 0.3, 0.9),
                )
                terminal = self.robot_state(0.1, 0.2)
            else:
                resolved = make_pose(
                    x=target.position.x,
                    y=target.position.y,
                    z=target.position.z,
                    q=(0.4, 0.3, 0.2, 0.8),
                )
                terminal = self.robot_state(0.6, 0.7)
            seed = {
                    'resolved_target': resolved,
                    'terminal_state': terminal,
                    'path_cost': 0.5,
                    'max_delta': 0.3,
                    'position_error': 0.0,
                    'candidates_tested': 4,
                    'repeatability_error': 0.0,
                }
            if accept_seed is not None:
                accepted, code, reason = accept_seed(seed)
                if not accepted:
                    return None, code, reason
            return seed, '', 'resolved'

        planner._resolve_orientation_from_state = resolve_orientation
        targets = [
            make_pose(z=0.05),
            make_pose(z=0.08),
            make_pose(z=0.10),
            make_pose(z=0.15),
        ]

        ok, _code, _failed_stage, resolved, _metrics, message = (
            planner.resolve_free_space_orientations(
                targets,
                ['pregrasp', 'approach', 'grasp', 'lift'],
                [False, True, True, False],
                [True, False, False, True],
            )
        )

        self.assertTrue(ok, message)
        self.assertEqual(resolver_calls[0][0], [0.0, 0.0])
        self.assertEqual(resolver_calls[1][0], [0.3, 0.4])
        self.assertEqual(
            resolver_calls[1][1],
            (0.1, 0.2, 0.3, 0.9),
        )
        self.assertEqual(
            (
                resolved[3].orientation.x,
                resolved[3].orientation.y,
                resolved[3].orientation.z,
                resolved[3].orientation.w,
            ),
            (0.4, 0.3, 0.2, 0.8),
        )

    def test_free_space_resolver_rejects_cartesian_position_only_stage(self):
        planner = self.make_planner(FakeManipulator())
        planner.robot = types.SimpleNamespace(
            get_current_state=lambda: self.robot_state(0.0, 0.0)
        )

        ok, code, failed_stage, resolved, _metrics, message = (
            planner.resolve_free_space_orientations(
                [make_pose()],
                ['lift'],
                [True],
                [True],
            )
        )

        self.assertFalse(ok)
        self.assertEqual(code, 'MOVEIT_RESOLVE_ERROR')
        self.assertEqual(failed_stage, 'lift')
        self.assertEqual(resolved, ())
        self.assertIn(
            'cannot be both free-space orientation resolution and Cartesian',
            message,
        )
        self.assertEqual(planner.manipulator.plan_calls, 0)

    def test_execute_failure_message_includes_target_xyz(self):
        manipulator = FakeManipulator(go_result=False)
        planner = self.make_planner(manipulator)

        ok, message = planner.move_to_pose(make_pose(), execute=True)

        self.assertFalse(ok)
        self.assertIn('execute planning failed before motion', message)
        self.assertIn('target xyz=(0.123, -0.456, 2.500)', message)
        self.assertEqual(manipulator.go_calls, 0)
        self.assertTrue(manipulator.cleared)

    def test_plan_failure_message_includes_target_xyz(self):
        manipulator = FakeManipulator(plan_result=EmptyPlan())
        planner = self.make_planner(manipulator)

        ok, message = planner.move_to_pose(make_pose(), execute=False)

        self.assertFalse(ok)
        self.assertIn('plan failed', message)
        self.assertIn('target xyz=(0.123, -0.456, 2.500)', message)
        self.assertTrue(manipulator.cleared)

    def test_strict_plan_does_not_try_orientation_or_position_fallbacks(self):
        manipulator = FakeManipulator(plan_result=[EmptyPlan(), SuccessfulPlan()])
        planner = self.make_planner(manipulator)
        planner.orientation_fallback_enabled = True
        planner.position_only_fallback_enabled = True

        ok, message = planner.move_to_pose(
            make_pose(q=(0.0, 0.7071, 0.0, 0.7071)),
            execute=False,
            allow_fallbacks=False,
        )

        self.assertFalse(ok)
        self.assertIn('strict pose', message)
        self.assertEqual(len(manipulator.pose_targets), 1)
        self.assertEqual(manipulator.position_targets, [])
        self.assertEqual(manipulator.start_state_to_current_calls, 1)
        self.assertEqual(manipulator.planning_time_updates, [0.25, 2.0])

    def test_strict_plan_then_cached_only_execute_uses_exact_planned_trajectory(self):
        planned = SuccessfulPlan()
        manipulator = FakeManipulator(
            plan_result=planned,
            execute_result=True,
            current_joint_values=[0.0, 0.0],
        )
        planner = self.make_planner(manipulator)
        target = make_pose(q=(0.0, 0.7071, 0.0, 0.7071))

        plan_ok, plan_message = planner.move_to_pose(
            target,
            execute=False,
            allow_fallbacks=False,
        )
        execute_ok, execute_message = planner.execute_cached_strict_pose(target)

        self.assertTrue(plan_ok, plan_message)
        self.assertTrue(execute_ok, execute_message)
        self.assertIn('strict pose', execute_message)
        self.assertEqual(manipulator.executed_plans, [planned])
        self.assertEqual(manipulator.plan_calls, 1)
        self.assertEqual(manipulator.go_calls, 0)
        self.assertIsNone(planner._last_pose_plan)

    def test_failed_cached_execute_succeeds_when_joint_feedback_reached_hardware_tolerance(self):
        planned = JointPlan([[0.0, 0.0], [0.10, 0.20]])
        manipulator = FakeManipulator(
            plan_result=planned,
            execute_result=False,
            current_joint_values=[0.099, 0.226],
        )
        planner = self.make_planner(manipulator)
        target = make_pose(q=(0.0, 0.7071, 0.0, 0.7071))

        plan_ok, plan_message = planner.move_to_pose(
            target,
            execute=False,
            allow_fallbacks=False,
        )
        execute_ok, execute_message = planner.execute_cached_strict_pose(target)

        self.assertTrue(plan_ok, plan_message)
        self.assertTrue(execute_ok, execute_message)
        self.assertIn('within hardware goal tolerance', execute_message)
        self.assertEqual(manipulator.executed_plans, [planned])
        self.assertIsNone(planner._last_pose_plan)

    def test_failed_cached_execute_accepts_real_quantized_goal_boundary(self):
        planned = JointPlan([[0.0, 0.0], [0.10, 0.20]])
        manipulator = FakeManipulator(
            plan_result=planned,
            execute_result=False,
            current_joint_values=[0.099, 0.2300047],
        )
        planner = self.make_planner(manipulator)
        target = make_pose(q=(0.0, 0.7071, 0.0, 0.7071))

        plan_ok, plan_message = planner.move_to_pose(
            target,
            execute=False,
            allow_fallbacks=False,
        )
        execute_ok, execute_message = planner.execute_cached_strict_pose(target)

        self.assertTrue(plan_ok, plan_message)
        self.assertTrue(execute_ok, execute_message)
        self.assertIn('including slack', execute_message)
        self.assertIn('max_error=0.030005rad', execute_message)
        self.assertEqual(manipulator.executed_plans, [planned])
        self.assertIsNone(planner._last_pose_plan)

    def test_failed_cached_execute_stays_failed_when_joint_feedback_misses_hardware_tolerance_slack(self):
        planned = JointPlan([[0.0, 0.0], [0.10, 0.20]])
        manipulator = FakeManipulator(
            plan_result=planned,
            execute_result=False,
            current_joint_values=[0.099, 0.236],
        )
        planner = self.make_planner(manipulator)
        target = make_pose(q=(0.0, 0.7071, 0.0, 0.7071))

        plan_ok, plan_message = planner.move_to_pose(
            target,
            execute=False,
            allow_fallbacks=False,
        )
        execute_ok, execute_message = planner.execute_cached_strict_pose(target)

        self.assertTrue(plan_ok, plan_message)
        self.assertFalse(execute_ok)
        self.assertIn('execute failed from cached plan', execute_message)
        self.assertEqual(manipulator.executed_plans, [planned])
        self.assertIsNone(planner._last_pose_plan)

    def test_cached_only_strict_execute_reports_missing_cache_without_planning(self):
        manipulator = FakeManipulator(plan_result=SuccessfulPlan())
        planner = self.make_planner(manipulator)

        ok, message = planner.execute_cached_strict_pose(make_pose())

        self.assertFalse(ok)
        self.assertIn('no cached pose plan', message)
        self.assertEqual(manipulator.plan_calls, 0)
        self.assertEqual(manipulator.go_calls, 0)
        self.assertEqual(manipulator.executed_plans, [])

    def test_cached_only_strict_execute_rejects_position_and_orientation_mismatch(self):
        planned = SuccessfulPlan()
        manipulator = FakeManipulator(plan_result=planned)
        planner = self.make_planner(manipulator)
        target = make_pose(q=(0.0, 0.0, 0.0, 1.0))
        plan_ok, plan_message = planner.move_to_pose(
            target,
            execute=False,
            allow_fallbacks=False,
        )
        self.assertTrue(plan_ok, plan_message)

        position_ok, position_message = planner.execute_cached_strict_pose(
            make_pose(x=0.130, q=(0.0, 0.0, 0.0, 1.0))
        )
        orientation_ok, orientation_message = planner.execute_cached_strict_pose(
            make_pose(q=(0.0, 0.7071, 0.0, 0.7071))
        )

        self.assertFalse(position_ok)
        self.assertIn('position mismatch', position_message)
        self.assertFalse(orientation_ok)
        self.assertIn('orientation mismatch', orientation_message)
        self.assertEqual(manipulator.plan_calls, 1)
        self.assertEqual(manipulator.go_calls, 0)
        self.assertEqual(manipulator.executed_plans, [])

    def test_cached_only_strict_execute_rejects_every_non_strict_cache_kind(self):
        manipulator = FakeManipulator()
        planner = self.make_planner(manipulator)
        target = make_pose()

        for kind in ('position-only', 'candidate orientation current', 'cartesian'):
            with self.subTest(kind=kind):
                planner._remember_pose_plan(target, SuccessfulPlan(), kind)

                ok, message = planner.execute_cached_strict_pose(target)

                self.assertFalse(ok)
                self.assertIn("is not 'strict pose'", message)
                self.assertIn(kind, message)
        self.assertEqual(manipulator.plan_calls, 0)
        self.assertEqual(manipulator.go_calls, 0)
        self.assertEqual(manipulator.executed_plans, [])

    def test_position_only_plan_overwrites_and_blocks_previous_strict_cache(self):
        strict_plan = SuccessfulPlan()
        position_plan = SuccessfulPlan()
        manipulator = FakeManipulator(
            plan_result=[strict_plan, EmptyPlan(), position_plan],
        )
        planner = self.make_planner(manipulator)
        planner.orientation_fallback_enabled = False
        target = make_pose()

        strict_ok, strict_message = planner.move_to_pose(
            target,
            execute=False,
            allow_fallbacks=False,
        )
        replacement_ok, replacement_message = planner.move_to_pose(
            target,
            execute=False,
            allow_fallbacks=True,
        )
        execute_ok, execute_message = planner.execute_cached_strict_pose(target)

        self.assertTrue(strict_ok, strict_message)
        self.assertTrue(replacement_ok, replacement_message)
        self.assertIn('position-only', replacement_message)
        self.assertFalse(execute_ok)
        self.assertIn('position-only', execute_message)
        self.assertEqual(manipulator.executed_plans, [])
        self.assertEqual(manipulator.go_calls, 0)

    def test_failed_new_strict_plan_invalidates_previous_strict_cache(self):
        manipulator = FakeManipulator(
            plan_result=[SuccessfulPlan(), EmptyPlan()],
        )
        planner = self.make_planner(manipulator)
        target = make_pose()

        first_ok, first_message = planner.move_to_pose(
            target,
            execute=False,
            allow_fallbacks=False,
        )
        second_ok, second_message = planner.move_to_pose(
            target,
            execute=False,
            allow_fallbacks=False,
        )
        execute_ok, execute_message = planner.execute_cached_strict_pose(target)

        self.assertTrue(first_ok, first_message)
        self.assertFalse(second_ok, second_message)
        self.assertFalse(execute_ok)
        self.assertIn('no cached pose plan', execute_message)
        self.assertEqual(manipulator.executed_plans, [])
        self.assertEqual(manipulator.go_calls, 0)

    def test_plan_falls_back_to_position_only_when_pose_orientations_are_unreachable(self):
        manipulator = FakeManipulator(plan_result=[EmptyPlan(), SuccessfulPlan()])
        planner = self.make_planner(manipulator)
        planner.orientation_fallback_enabled = False

        ok, message = planner.move_to_pose(make_pose(), execute=False)

        self.assertTrue(ok)
        self.assertIn('position-only', message)
        self.assertEqual(len(manipulator.pose_targets), 1)
        self.assertEqual(manipulator.position_targets, [[0.123, -0.456, 2.5]])
        self.assertTrue(manipulator.cleared)

    def test_execute_does_not_use_position_only_fallback_by_default(self):
        manipulator = FakeManipulator(go_result=[False, True], plan_result=[EmptyPlan(), SuccessfulPlan()])
        planner = self.make_planner(manipulator)
        planner.orientation_fallback_enabled = False
        planner.position_only_fallback_enabled = True
        planner.position_only_execute_enabled = False

        ok, message = planner.move_to_pose(make_pose(), execute=True)

        self.assertFalse(ok)
        self.assertIn('execute blocked for position-only cached plan', message)
        self.assertEqual(manipulator.position_targets, [[0.123, -0.456, 2.5]])
        self.assertEqual(manipulator.go_calls, 0)

    def test_execute_plans_then_executes_cached_plan_without_live_go_fallbacks(self):
        planned = SuccessfulPlan()
        manipulator = FakeManipulator(
            go_result=[False, True],
            plan_result=[planned],
            execute_result=True,
        )
        planner = self.make_planner(manipulator)
        planner.candidate_orientations = []

        ok, message = planner.move_to_pose(make_pose(), execute=True)

        self.assertTrue(ok, message)
        self.assertIn('executed cached plan', message)
        self.assertEqual(manipulator.executed_plans, [planned])
        self.assertEqual(manipulator.go_calls, 0)

    def test_plan_tries_current_orientation_before_position_only_fallback(self):
        current = types.SimpleNamespace(pose=make_pose(q=(0.0, 0.0, 0.0, 1.0)))
        manipulator = FakeManipulator(
            plan_result=[EmptyPlan(), SuccessfulPlan()],
            current_pose=current,
        )
        planner = self.make_planner(manipulator)
        planner.candidate_orientations = []

        ok, message = planner.move_to_pose(
            make_pose(q=(0.0, 0.7071, 0.0, 0.7071)),
            execute=False,
        )

        self.assertTrue(ok)
        self.assertIn('candidate orientation', message)
        self.assertEqual(len(manipulator.pose_targets), 2)
        self.assertEqual(manipulator.pose_targets[1].orientation.w, 1.0)
        self.assertEqual(manipulator.position_targets, [])

    def test_execute_uses_cached_successful_plan_instead_of_replanning(self):
        planned = SuccessfulPlan()
        manipulator = FakeManipulator(
            go_result=False,
            plan_result=[EmptyPlan(), planned, EmptyPlan()],
            execute_result=True,
        )
        planner = self.make_planner(manipulator)

        plan_ok, plan_message = planner.move_to_pose(make_pose(), execute=False)
        execute_ok, execute_message = planner.move_to_pose(make_pose(), execute=True)

        self.assertTrue(plan_ok, plan_message)
        self.assertTrue(execute_ok, execute_message)
        self.assertIn('cached plan', execute_message)
        self.assertEqual(manipulator.executed_plans, [planned])

    def test_strict_cached_execute_retimes_same_geometry_before_motion(self):
        planned = JointPlan([[0.0, 0.0], [1.0, -0.2]])
        retimed = JointPlan([[0.0, 0.0], [1.0, -0.2]], duration_sec=20.0)
        manipulator = FakeManipulator(
            plan_result=planned,
            execute_result=True,
            current_joint_values=[0.0, 0.0],
        )
        manipulator.retime_trajectory = (
            lambda _state, _plan, **_kwargs: retimed
        )
        planner = self.make_planner(manipulator)
        planner.robot = types.SimpleNamespace(get_current_state=lambda: object())
        planner.strict_execution_retime_enabled = True
        planner.strict_execution_velocity_scaling = 0.20
        planner.strict_execution_acceleration_scaling = 0.30
        planner.strict_execution_max_joint_velocity_rad_s = 0.08
        planner.strict_execution_start_tolerance_rad = 0.08

        plan_ok, plan_message = planner.move_to_pose(
            make_pose(),
            execute=False,
            allow_fallbacks=False,
        )
        execute_ok, execute_message = planner.execute_cached_strict_pose(
            make_pose()
        )

        self.assertTrue(plan_ok, plan_message)
        self.assertTrue(execute_ok, execute_message)
        self.assertEqual(manipulator.executed_plans, [retimed])
        self.assertIn('strict trajectory retimed duration=20.000s', execute_message)

    def test_strict_cached_execute_stretches_under_timed_retimed_path(self):
        planned = JointPlan([[0.0, 0.0], [1.0, -0.2]])
        retimed = JointPlan([[0.0, 0.0], [1.0, -0.2]], duration_sec=1.0)
        manipulator = FakeManipulator(
            plan_result=planned,
            execute_result=True,
            current_joint_values=[0.0, 0.0],
        )
        manipulator.retime_trajectory = (
            lambda _state, _plan, **_kwargs: retimed
        )
        planner = self.make_planner(manipulator)
        planner.robot = types.SimpleNamespace(get_current_state=lambda: object())
        planner.strict_execution_retime_enabled = True
        planner.strict_execution_velocity_scaling = 0.20
        planner.strict_execution_acceleration_scaling = 0.30
        planner.strict_execution_max_joint_velocity_rad_s = 0.08
        planner.strict_execution_start_tolerance_rad = 0.08

        plan_ok, plan_message = planner.move_to_pose(
            make_pose(),
            execute=False,
            allow_fallbacks=False,
        )
        execute_ok, execute_message = planner.execute_cached_strict_pose(
            make_pose()
        )

        self.assertTrue(plan_ok, plan_message)
        self.assertTrue(execute_ok, execute_message)
        self.assertEqual(len(manipulator.executed_plans), 1)
        executed = manipulator.executed_plans[0]
        self.assertAlmostEqual(
            planner._trajectory_duration_sec(executed),
            12.5,
        )
        self.assertEqual(
            [
                list(point.positions)
                for point in executed.joint_trajectory.points
            ],
            [[0.0, 0.0], [1.0, -0.2]],
        )
        self.assertIn('velocity_time_scale=4.167', execute_message)
        self.assertIn('time_scale=12.500', execute_message)
        self.assertIn('peak_velocity=0.080rad/s', execute_message)

    def test_strict_cached_execute_stretches_local_peak_missed_by_endpoint_average(self):
        positions = [
            [0.0, 0.0],
            [0.04, -0.01],
            [0.08, -0.02],
            [1.0, -0.2],
        ]
        planned = JointPlan(positions)
        retimed = JointPlan(positions, duration_sec=20.0)
        manipulator = FakeManipulator(
            plan_result=planned,
            execute_result=True,
            current_joint_values=[0.0, 0.0],
        )
        manipulator.retime_trajectory = (
            lambda _state, _plan, **_kwargs: retimed
        )
        planner = self.make_planner(manipulator)
        planner.robot = types.SimpleNamespace(get_current_state=lambda: object())
        planner.strict_execution_retime_enabled = True
        planner.strict_execution_velocity_scaling = 0.20
        planner.strict_execution_acceleration_scaling = 0.30
        planner.strict_execution_max_joint_velocity_rad_s = 0.08
        planner.strict_execution_min_duration_sec = 3.0
        planner.strict_execution_start_tolerance_rad = 0.08

        plan_ok, plan_message = planner.move_to_pose(
            make_pose(),
            execute=False,
            allow_fallbacks=False,
        )
        execute_ok, execute_message = planner.execute_cached_strict_pose(
            make_pose()
        )

        self.assertTrue(plan_ok, plan_message)
        self.assertTrue(execute_ok, execute_message)
        executed = manipulator.executed_plans[0]
        self.assertGreater(
            planner._trajectory_duration_sec(executed),
            20.0,
        )
        peaks, peak_error = planner._trajectory_peak_joint_velocities_rad_s(
            executed
        )
        self.assertEqual(peak_error, '')
        self.assertLessEqual(max(peaks), 0.08 + 1e-9)
        self.assertEqual(
            [
                list(point.positions)
                for point in executed.joint_trajectory.points
            ],
            positions,
        )

    def test_strict_cached_execute_honors_generic_per_joint_velocity_limit(self):
        positions = [[0.0, 0.0], [0.4, 0.4]]
        planned = JointPlan(positions)
        retimed = JointPlan(positions, duration_sec=8.0)
        manipulator = FakeManipulator(
            plan_result=planned,
            execute_result=True,
            current_joint_values=[0.0, 0.0],
        )
        manipulator.retime_trajectory = (
            lambda _state, _plan, **_kwargs: retimed
        )
        planner = self.make_planner(manipulator)
        planner.robot = types.SimpleNamespace(get_current_state=lambda: object())
        planner.strict_execution_retime_enabled = True
        planner.strict_execution_velocity_scaling = 0.20
        planner.strict_execution_acceleration_scaling = 0.30
        planner.strict_execution_max_joint_velocity_rad_s = 0.08
        planner.strict_execution_joint_velocity_limits_rad_s = {
            'Joint2': 0.02,
        }
        planner.strict_execution_min_duration_sec = 3.0
        planner.strict_execution_start_tolerance_rad = 0.08

        plan_ok, plan_message = planner.move_to_pose(
            make_pose(),
            execute=False,
            allow_fallbacks=False,
        )
        execute_ok, execute_message = planner.execute_cached_strict_pose(
            make_pose()
        )

        self.assertTrue(plan_ok, plan_message)
        self.assertTrue(execute_ok, execute_message)
        executed = manipulator.executed_plans[0]
        self.assertAlmostEqual(
            planner._trajectory_duration_sec(executed),
            20.0,
        )
        peaks, peak_error = planner._trajectory_peak_joint_velocities_rad_s(
            executed
        )
        self.assertEqual(peak_error, '')
        self.assertLessEqual(peaks[1], 0.02 + 1e-9)
        self.assertIn('limiting_joint=Joint2', execute_message)
        self.assertIn('limiting_velocity=0.020rad/s', execute_message)

    def test_short_strict_cached_execute_stretches_timing_without_changing_path(self):
        planned = JointPlan([[0.0, 0.0], [0.06, -0.02]])
        retimed = JointPlan(
            [[0.0, 0.0], [0.06, -0.02]],
            duration_sec=1.2,
        )
        for point in retimed.joint_trajectory.points:
            point.velocities = [0.05, -0.02]
            point.accelerations = [0.10, -0.04]
        manipulator = FakeManipulator(
            plan_result=planned,
            execute_result=True,
            current_joint_values=[0.0, 0.0],
        )
        manipulator.retime_trajectory = (
            lambda _state, _plan, **_kwargs: retimed
        )
        planner = self.make_planner(manipulator)
        planner.robot = types.SimpleNamespace(get_current_state=lambda: object())
        planner.strict_execution_retime_enabled = True
        planner.strict_execution_velocity_scaling = 0.20
        planner.strict_execution_acceleration_scaling = 0.30
        planner.strict_execution_max_joint_velocity_rad_s = 0.08
        planner.strict_execution_min_duration_sec = 3.0
        planner.strict_execution_start_tolerance_rad = 0.08

        plan_ok, plan_message = planner.move_to_pose(
            make_pose(),
            execute=False,
            allow_fallbacks=False,
        )
        execute_ok, execute_message = planner.execute_cached_strict_pose(
            make_pose()
        )

        self.assertTrue(plan_ok, plan_message)
        self.assertTrue(execute_ok, execute_message)
        self.assertEqual(len(manipulator.executed_plans), 1)
        executed = manipulator.executed_plans[0]
        self.assertIsNot(executed, retimed)
        self.assertAlmostEqual(
            planner._trajectory_duration_sec(executed),
            3.0,
        )
        self.assertEqual(
            [
                list(point.positions)
                for point in executed.joint_trajectory.points
            ],
            [[0.0, 0.0], [0.06, -0.02]],
        )
        self.assertAlmostEqual(
            executed.joint_trajectory.points[-1].velocities[0],
            0.02,
        )
        self.assertAlmostEqual(
            executed.joint_trajectory.points[-1].accelerations[0],
            0.016,
        )
        self.assertIn('minimum_duration=3.000s', execute_message)
        self.assertIn('time_scale=2.500', execute_message)

    def test_strict_cached_execute_blocks_stale_trajectory_start_state(self):
        planned = JointPlan([[0.0, 0.0], [0.2, -0.1]])
        retimed = JointPlan([[0.0, 0.0], [0.2, -0.1]], duration_sec=5.0)
        manipulator = FakeManipulator(
            plan_result=planned,
            execute_result=True,
            current_joint_values=[0.0, 0.2],
        )
        manipulator.retime_trajectory = (
            lambda _state, _plan, **_kwargs: retimed
        )
        planner = self.make_planner(manipulator)
        planner.robot = types.SimpleNamespace(get_current_state=lambda: object())
        planner.strict_execution_retime_enabled = True
        planner.strict_execution_velocity_scaling = 0.20
        planner.strict_execution_acceleration_scaling = 0.30
        planner.strict_execution_max_joint_velocity_rad_s = 0.08
        planner.strict_execution_start_tolerance_rad = 0.08

        plan_ok, plan_message = planner.move_to_pose(
            make_pose(),
            execute=False,
            allow_fallbacks=False,
        )
        execute_ok, execute_message = planner.execute_cached_strict_pose(
            make_pose()
        )

        self.assertTrue(plan_ok, plan_message)
        self.assertFalse(execute_ok)
        self.assertIn('trajectory start mismatch', execute_message)
        self.assertEqual(manipulator.executed_plans, [])

    def test_cached_noetic_tuple_plan_executes_inner_trajectory(self):
        planned = SuccessfulPlan()
        manipulator = FakeManipulator(
            go_result=False,
            plan_result=[EmptyPlan(), (True, planned, 0.1, None), EmptyPlan()],
            execute_result=True,
        )
        planner = self.make_planner(manipulator)

        plan_ok, plan_message = planner.move_to_pose(make_pose(), execute=False)
        execute_ok, execute_message = planner.move_to_pose(make_pose(), execute=True)

        self.assertTrue(plan_ok, plan_message)
        self.assertTrue(execute_ok, execute_message)
        self.assertEqual(manipulator.executed_plans, [planned])

    def test_cartesian_line_plans_then_executes_same_cached_trajectory(self):
        current = types.SimpleNamespace(pose=make_pose(x=0.0, y=0.0, z=0.20))
        planned = JointPlan([[0.0, 0.0], [0.1, -0.2]])
        manipulator = FakeManipulator(
            current_pose=current,
            cartesian_result=(planned, 1.0),
            execute_result=True,
        )
        planner = self.make_planner(manipulator)
        target = make_pose(x=0.04, y=0.0, z=0.20)

        plan_ok, plan_message = planner.move_to_pose_linear(target, execute=False)
        execute_ok, execute_message = planner.move_to_pose_linear(target, execute=True)

        self.assertTrue(plan_ok, plan_message)
        self.assertIn('fraction=1.000', plan_message)
        self.assertIn('joint_path_cost=', plan_message)
        self.assertTrue(execute_ok, execute_message)
        self.assertEqual(manipulator.executed_plans, [planned])
        self.assertEqual(len(manipulator.cartesian_calls), 1)

    def test_cartesian_line_uses_noetic_signature_without_jump_threshold(self):
        current = types.SimpleNamespace(pose=make_pose(x=0.0, y=0.0, z=0.20))
        manipulator = FakeNoeticManipulator(current_pose=current)
        planner = self.make_planner(manipulator)

        ok, message = planner.move_to_pose_linear(
            make_pose(x=0.04, y=0.0, z=0.20),
            execute=False,
        )

        self.assertTrue(ok, message)
        self.assertEqual(len(manipulator.cartesian_calls), 1)
        self.assertTrue(manipulator.cartesian_calls[0][2])
        self.assertIsNone(manipulator.cartesian_calls[0][3])

    def test_failed_cartesian_execution_invalidates_cached_trajectory(self):
        current = types.SimpleNamespace(pose=make_pose(x=0.0, y=0.0, z=0.20))
        planned = JointPlan([[0.0, 0.0], [0.1, -0.2]])
        manipulator = FakeManipulator(
            current_pose=current,
            cartesian_result=(planned, 1.0),
            execute_result=False,
        )
        planner = self.make_planner(manipulator)
        target = make_pose(x=0.04, y=0.0, z=0.20)

        plan_ok, _plan_message = planner.move_to_pose_linear(target, execute=False)
        first_ok, first_message = planner.move_to_pose_linear(target, execute=True)
        second_ok, second_message = planner.move_to_pose_linear(target, execute=True)

        self.assertTrue(plan_ok)
        self.assertFalse(first_ok)
        self.assertIn('execute failed from cached plan', first_message)
        self.assertFalse(second_ok)
        self.assertIn('no matching Cartesian plan', second_message)
        self.assertEqual(manipulator.executed_plans, [planned])

    def test_cartesian_line_rejects_incomplete_path(self):
        current = types.SimpleNamespace(pose=make_pose(x=0.0, y=0.0, z=0.20))
        manipulator = FakeManipulator(
            current_pose=current,
            cartesian_result=(SuccessfulPlan(), 0.75),
        )
        planner = self.make_planner(manipulator)

        ok, message = planner.move_to_pose_linear(make_pose(x=0.04, y=0.0, z=0.20), execute=False)

        self.assertFalse(ok)
        self.assertIn('fraction=0.750 < 0.980', message)

    def test_cartesian_line_rejects_segment_over_limit(self):
        current = types.SimpleNamespace(pose=make_pose(x=0.0, y=0.0, z=0.20))
        manipulator = FakeManipulator(current_pose=current)
        planner = self.make_planner(manipulator)

        ok, message = planner.move_to_pose_linear(make_pose(x=0.09, y=0.0, z=0.20), execute=False)

        self.assertFalse(ok)
        self.assertIn('exceeds limit', message)
        self.assertEqual(manipulator.cartesian_calls, [])

    def test_cartesian_cache_rejects_same_position_with_different_orientation(self):
        current = types.SimpleNamespace(pose=make_pose(x=0.0, y=0.0, z=0.20))
        manipulator = FakeManipulator(current_pose=current)
        planner = self.make_planner(manipulator)
        planned_pose = make_pose(x=0.04, y=0.0, z=0.20, q=(0.0, 0.0, 0.0, 1.0))
        changed_pose = make_pose(x=0.04, y=0.0, z=0.20, q=(0.0, 0.7071, 0.0, 0.7071))

        plan_ok, _message = planner.move_to_pose_linear(planned_pose, execute=False)
        execute_ok, execute_message = planner.move_to_pose_linear(changed_pose, execute=True)

        self.assertTrue(plan_ok)
        self.assertFalse(execute_ok)
        self.assertIn('no matching Cartesian plan', execute_message)


if __name__ == '__main__':
    unittest.main()
