#!/usr/bin/env python3
import pathlib
import sys
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / 'src'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from gui.widgets.perception_widget import perception_grasp_action_mode


class GuiGraspModeHelpersTest(unittest.TestCase):
    def test_perception_actions_are_observation_only_in_6d_mode(self):
        mode = perception_grasp_action_mode(True)

        self.assertTrue(mode['observation_only'])
        self.assertFalse(mode['show_legacy_pregrasp'])
        self.assertIn('6D 抓取', mode['note'])

    def test_perception_actions_keep_legacy_buttons_when_6d_disabled(self):
        mode = perception_grasp_action_mode(False)

        self.assertFalse(mode['observation_only'])
        self.assertTrue(mode['show_legacy_pregrasp'])
        self.assertIn('规划预抓取', mode['note'])


if __name__ == '__main__':
    unittest.main()
