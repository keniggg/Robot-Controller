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


class FakeManipulator:
    def __init__(self, go_result=False, plan_result=None, current_pose=None, execute_result=True):
        self.go_results = list(go_result) if isinstance(go_result, (list, tuple)) else [go_result]
        if isinstance(plan_result, (list, tuple)):
            self.plan_results = list(plan_result)
        else:
            self.plan_results = [plan_result if plan_result is not None else EmptyPlan()]
        self.current_pose = current_pose
        self.execute_result = execute_result
        self.target = None
        self.pose_targets = []
        self.position_targets = []
        self.executed_plans = []
        self.stopped = False
        self.cleared = False

    def set_pose_target(self, pose):
        self.target = pose
        self.pose_targets.append(pose)

    def set_position_target(self, position):
        self.position_targets.append(list(position))

    def go(self, wait=True):
        if len(self.go_results) > 1:
            return self.go_results.pop(0)
        return self.go_results[0]

    def plan(self):
        if len(self.plan_results) > 1:
            return self.plan_results.pop(0)
        return self.plan_results[0]

    def execute(self, plan, wait=True):
        self.executed_plans.append(plan)
        return self.execute_result

    def stop(self):
        self.stopped = True

    def clear_pose_targets(self):
        self.cleared = True

    def get_current_pose(self):
        return self.current_pose


class MoveItPlannerPoseFeedbackTest(unittest.TestCase):
    def make_planner(self, manipulator):
        planner = MoveItPlanner.__new__(MoveItPlanner)
        planner.ready = True
        planner.error = None
        planner.manipulator = manipulator
        return planner

    def test_execute_failure_message_includes_target_xyz(self):
        manipulator = FakeManipulator(go_result=False)
        planner = self.make_planner(manipulator)

        ok, message = planner.move_to_pose(make_pose(), execute=True)

        self.assertFalse(ok)
        self.assertIn('execute failed', message)
        self.assertIn('target xyz=(0.123, -0.456, 2.500)', message)
        self.assertTrue(manipulator.stopped)
        self.assertTrue(manipulator.cleared)

    def test_plan_failure_message_includes_target_xyz(self):
        manipulator = FakeManipulator(plan_result=EmptyPlan())
        planner = self.make_planner(manipulator)

        ok, message = planner.move_to_pose(make_pose(), execute=False)

        self.assertFalse(ok)
        self.assertIn('plan failed', message)
        self.assertIn('target xyz=(0.123, -0.456, 2.500)', message)
        self.assertTrue(manipulator.cleared)

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


if __name__ == '__main__':
    unittest.main()
