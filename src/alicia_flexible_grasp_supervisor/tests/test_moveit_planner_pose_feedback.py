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
    def __init__(self, positions):
        self.joint_trajectory = types.SimpleNamespace(
            points=[types.SimpleNamespace(positions=list(values)) for values in positions]
        )


class FakeManipulator:
    def __init__(
        self,
        go_result=False,
        plan_result=None,
        current_pose=None,
        execute_result=True,
        cartesian_result=None,
    ):
        self.go_results = list(go_result) if isinstance(go_result, (list, tuple)) else [go_result]
        if isinstance(plan_result, (list, tuple)):
            self.plan_results = list(plan_result)
        else:
            self.plan_results = [plan_result if plan_result is not None else EmptyPlan()]
        self.current_pose = current_pose
        self.execute_result = execute_result
        self.cartesian_result = cartesian_result
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

    def get_planning_time(self):
        return self.planning_time

    def set_planning_time(self, value):
        self.planning_time = float(value)
        self.planning_time_updates.append(float(value))


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
        planner.cached_plan_position_tolerance_m = 0.002
        planner.cached_plan_orientation_tolerance_rad = 0.02
        planner.cartesian_eef_step_m = 0.003
        planner.cartesian_jump_threshold = 0.0
        planner.cartesian_min_fraction = 0.98
        planner.cartesian_max_segment_m = 0.08
        return planner

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
        self.assertEqual(manipulator.planning_time_updates, [0.25, 2.0])

    def test_strict_plan_then_cached_only_execute_uses_exact_planned_trajectory(self):
        planned = SuccessfulPlan()
        manipulator = FakeManipulator(plan_result=planned, execute_result=True)
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
