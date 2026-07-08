#!/usr/bin/env python3
import pathlib
import sys
import types
import unittest

from geometry_msgs.msg import PoseArray, PoseStamped
from sensor_msgs.msg import JointState


ROOT = pathlib.Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / 'src'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from alicia_flexible_grasp.vision.mujoco_digital_twin_client import (  # noqa: E402
    build_simulation_payload,
    validate_mujoco_digital_twin_url,
)


class MujocoDigitalTwinClientTest(unittest.TestCase):
    def _pose(self, x, y=0.0, z=0.2):
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.w = 1.0
        return pose

    def test_rejects_placeholder_url(self):
        with self.assertRaises(ValueError):
            validate_mujoco_digital_twin_url('http://<WSL_IP>:8000')

    def test_builds_simulation_payload_from_ros_messages(self):
        joints = JointState()
        joints.name = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6', 'right_finger']
        joints.position = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.05]
        plan = PoseArray()
        plan.header.frame_id = 'base_link'
        for x in (0.10, 0.20, 0.30, 0.30):
            plan.poses.append(self._pose(x).pose)
        obj = types.SimpleNamespace(
            detected=True,
            pose_base=self._pose(0.30, -0.04, 0.12),
            label='mouse',
            confidence=0.91,
        )

        payload = build_simulation_payload(
            joint_state=joints,
            object_pose=obj,
            grasp_plan=plan,
            gripper_width_m=0.05,
            object_model={'type': 'mouse_compound', 'size_xyz_m': [0.10, 0.06, 0.035]},
        )

        self.assertEqual(payload['joint_names'][:2], ['Joint1', 'Joint2'])
        self.assertEqual(payload['gripper_width_m'], 0.05)
        self.assertEqual(payload['object_pose_base']['label'], 'mouse')
        self.assertEqual(payload['object_pose_base']['size_xyz_m'], [0.10, 0.06, 0.035])
        self.assertEqual([item['name'] for item in payload['grasp_sequence_base']], ['pregrasp', 'approach', 'grasp', 'lift'])
        self.assertEqual(payload['grasp_sequence_base'][2]['position'], [0.30, 0.0, 0.2])


if __name__ == '__main__':
    unittest.main()
