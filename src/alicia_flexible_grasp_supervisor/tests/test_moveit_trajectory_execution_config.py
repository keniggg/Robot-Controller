#!/usr/bin/env python3
import pathlib
import unittest
import xml.etree.ElementTree as ET


ROOT = pathlib.Path(__file__).resolve().parents[2]
TRAJECTORY_EXECUTION = ROOT / 'real-arm' / 'alicia_d_moveit' / 'launch' / 'trajectory_execution.launch.xml'


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


if __name__ == '__main__':
    unittest.main()
