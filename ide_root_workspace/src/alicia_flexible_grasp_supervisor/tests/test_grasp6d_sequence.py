#!/usr/bin/env python3
import importlib.util
import math
import pathlib
import sys
import unittest

from geometry_msgs.msg import PoseStamped
from alicia_flexible_grasp_supervisor.msg import Grasp6DPlan
from tf.transformations import quaternion_from_euler


ROOT = pathlib.Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / 'src'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from alicia_flexible_grasp.grasp.grasp6d_sequence import make_grasp_sequence_from_grasp_pose

TASK_SCRIPT = ROOT / 'scripts' / 'grasp_task_node.py'
task_spec = importlib.util.spec_from_file_location(
    'grasp_task_node_for_sequence', str(TASK_SCRIPT)
)
grasp_task_node = importlib.util.module_from_spec(task_spec)
task_spec.loader.exec_module(grasp_task_node)


class Grasp6DSequenceTest(unittest.TestCase):
    def _pose(self, x=0.4, y=0.1, z=0.2, quat=(0.0, 0.0, 0.0, 1.0)):
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.x = float(quat[0])
        pose.pose.orientation.y = float(quat[1])
        pose.pose.orientation.z = float(quat[2])
        pose.pose.orientation.w = float(quat[3])
        return pose

    def test_sequence_offsets_along_grasp_approach_axis(self):
        grasp_pose = self._pose(quat=quaternion_from_euler(0.0, 0.0, 0.0))

        plan = make_grasp_sequence_from_grasp_pose(
            grasp_pose,
            pregrasp_distance_m=0.08,
            approach_offset_m=0.015,
            lift_height_m=0.05,
        )

        self.assertAlmostEqual(plan.pregrasp.pose.position.x, 0.32)
        self.assertAlmostEqual(plan.approach.pose.position.x, 0.385)
        self.assertAlmostEqual(plan.grasp.pose.position.x, 0.4)
        self.assertAlmostEqual(plan.lift.pose.position.z, 0.25)
        self.assertAlmostEqual(plan.grasp.pose.orientation.w, 1.0)

    def test_sequence_uses_rotated_grasp_axis_not_fixed_base_axis(self):
        quat = quaternion_from_euler(0.0, 0.0, math.pi * 0.5)
        grasp_pose = self._pose(quat=quat)

        plan = make_grasp_sequence_from_grasp_pose(
            grasp_pose,
            pregrasp_distance_m=0.08,
            approach_offset_m=0.015,
            lift_height_m=0.05,
        )

        self.assertAlmostEqual(plan.pregrasp.pose.position.x, 0.4, places=6)
        self.assertAlmostEqual(plan.pregrasp.pose.position.y, 0.02, places=6)
        self.assertAlmostEqual(plan.approach.pose.position.x, 0.4, places=6)
        self.assertAlmostEqual(plan.approach.pose.position.y, 0.085, places=6)

    def test_sequence_can_use_physical_tool_z_as_approach_axis(self):
        grasp_pose = self._pose()

        plan = make_grasp_sequence_from_grasp_pose(
            grasp_pose,
            pregrasp_distance_m=0.08,
            approach_offset_m=0.015,
            lift_height_m=0.05,
            tool_approach_axis='z',
        )

        self.assertAlmostEqual(plan.pregrasp.pose.position.x, 0.4)
        self.assertAlmostEqual(plan.pregrasp.pose.position.z, 0.12)
        self.assertAlmostEqual(plan.approach.pose.position.z, 0.185)

    def test_rich_plan_fixed_order_is_split_into_independent_stamped_poses(self):
        plan = Grasp6DPlan()
        plan.header.frame_id = 'base_link'
        plan.valid = True
        plan.plan_id = 'fixed-order'
        plan.poses = [self._pose(x=float(index)).pose for index in range(4)]

        pregrasp, approach, grasp, lift = grasp_task_node.split_rich_plan_poses(plan)

        self.assertEqual(
            [
                pregrasp.pose.position.x,
                approach.pose.position.x,
                grasp.pose.position.x,
                lift.pose.position.x,
            ],
            [0.0, 1.0, 2.0, 3.0],
        )
        pregrasp.pose.position.x = 99.0
        self.assertEqual(plan.poses[0].position.x, 0.0)


if __name__ == '__main__':
    unittest.main()
