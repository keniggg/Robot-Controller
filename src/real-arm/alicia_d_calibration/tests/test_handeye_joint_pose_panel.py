#!/usr/bin/env python3

import importlib.util
import math
from pathlib import Path
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "handeye_joint_pose_panel.py"
SPEC = importlib.util.spec_from_file_location("handeye_joint_pose_panel", str(SCRIPT))
PANEL = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PANEL)


class HandeyeJointPosePanelHelpersTest(unittest.TestCase):
    def test_extracts_named_arm_and_gripper_without_reordering_error(self):
        names = ["right_finger", "Joint3", "Joint1", "Joint6", "Joint2", "Joint5", "Joint4"]
        positions = [0.04, 0.3, 0.1, 0.6, 0.2, 0.5, 0.4]
        arm, gripper = PANEL.extract_joint_snapshot(names, positions)
        self.assertEqual(arm, [0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
        self.assertEqual(gripper, 0.04)

    def test_rejects_incomplete_or_nonfinite_feedback(self):
        arm, gripper = PANEL.extract_joint_snapshot(["Joint1"], [0.1])
        self.assertIsNone(arm)
        self.assertIsNone(gripper)

    def test_does_not_guess_a_missing_named_joint_by_position(self):
        names = ["Joint1", "Joint2", "Joint3", "not_Joint4", "Joint5", "Joint6"]
        arm, gripper = PANEL.extract_joint_snapshot(names, [0.0] * 6)
        self.assertIsNone(arm)
        self.assertIsNone(gripper)
        arm, gripper = PANEL.extract_joint_snapshot(
            PANEL.ARM_JOINT_NAMES, [0.0, 0.0, math.nan, 0.0, 0.0, 0.0]
        )
        self.assertIsNone(arm)
        self.assertIsNone(gripper)

    def test_jog_changes_only_selected_joint(self):
        target, clamped = PANEL.make_jog_target(
            [0.0] * 6, 2, math.radians(2.0), [-1.0] * 6, [1.0] * 6
        )
        self.assertFalse(clamped)
        self.assertEqual(target[:2], [0.0, 0.0])
        self.assertAlmostEqual(target[2], math.radians(2.0))
        self.assertEqual(target[3:], [0.0, 0.0, 0.0])

    def test_jog_clamps_to_protocol_joint_limit(self):
        target, clamped = PANEL.make_jog_target(
            [0.0, 0.95, 0.0, 0.0, 0.0, 0.0],
            1,
            0.1,
            [-1.0] * 6,
            [1.0] * 6,
        )
        self.assertTrue(clamped)
        self.assertEqual(target[1], 1.0)

    def test_actuation_status_sync_gate_tracks_reconnect_states(self):
        self.assertTrue(
            PANEL.actuation_status_needs_driver_sync(
                "PENDING:POSITIVE_ENABLE_REQUESTED"
            )
        )
        self.assertTrue(
            PANEL.actuation_status_needs_driver_sync(
                "UNCONFIRMED:ENCODER_RESPONSE_TIMEOUT"
            )
        )
        self.assertFalse(
            PANEL.actuation_status_needs_driver_sync(
                "PENDING:COMMAND_SYNCHRONIZED"
            )
        )
        self.assertFalse(
            PANEL.actuation_status_needs_driver_sync(
                "CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE"
            )
        )


if __name__ == "__main__":
    unittest.main()
