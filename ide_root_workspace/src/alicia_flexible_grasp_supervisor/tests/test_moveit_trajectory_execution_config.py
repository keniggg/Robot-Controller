#!/usr/bin/env python3
import pathlib
import unittest
import xml.etree.ElementTree as ET

import yaml


ROOT = pathlib.Path(__file__).resolve().parents[2]
TRAJECTORY_EXECUTION = ROOT / 'real-arm' / 'alicia_d_moveit' / 'launch' / 'trajectory_execution.launch.xml'
CONTROLLERS = ROOT / 'real-arm' / 'alicia_d_driver' / 'config' / 'controllers.yaml'


class MoveItTrajectoryExecutionConfigTest(unittest.TestCase):
    def test_allowed_start_tolerance_accepts_real_arm_feedback_lag(self):
        tree = ET.parse(str(TRAJECTORY_EXECUTION))
        values = [
            node.attrib.get('value')
            for node in tree.findall('.//param')
            if node.attrib.get('name') == 'trajectory_execution/allowed_start_tolerance'
        ]

        self.assertTrue(values)
        self.assertGreaterEqual(float(values[0]), 0.05)

    def test_goal_duration_margin_matches_slow_hardware_goal_window(self):
        tree = ET.parse(str(TRAJECTORY_EXECUTION))
        values = [
            node.attrib.get('value')
            for node in tree.findall('.//param')
            if node.attrib.get('name') == 'trajectory_execution/allowed_goal_duration_margin'
        ]

        self.assertTrue(values)
        self.assertGreaterEqual(float(values[0]), 8.0)

    def test_real_arm_goal_tolerance_allows_measured_pregrasp_settle_error(self):
        data = yaml.safe_load(CONTROLLERS.read_text())
        constraints = data['alicia_controller']['constraints']

        for joint_name in ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']:
            with self.subTest(joint_name=joint_name):
                self.assertGreaterEqual(float(constraints[joint_name]['goal']), 0.03)


if __name__ == '__main__':
    unittest.main()
