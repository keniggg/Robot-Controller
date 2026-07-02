#!/usr/bin/env python3
import pathlib
import sys
import unittest

import rospy


ROOT = pathlib.Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / 'src'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from gui.widgets.joint_control_widget import JointControlWidget


class JointControlModeTest(unittest.TestCase):
    def test_default_joint_mode_keeps_trajectory_controllers_available(self):
        original_get_param = rospy.get_param
        rospy.get_param = lambda name, default=None: default
        try:
            self.assertFalse(JointControlWidget._default_direct_control_enabled())
        finally:
            rospy.get_param = original_get_param

    def test_default_joint_mode_can_be_overridden_for_direct_control(self):
        original_get_param = rospy.get_param
        rospy.get_param = lambda name, default=None: True if name == '/gui/default_joint_direct_control' else default
        try:
            self.assertTrue(JointControlWidget._default_direct_control_enabled())
        finally:
            rospy.get_param = original_get_param


if __name__ == '__main__':
    unittest.main()
