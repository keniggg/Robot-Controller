"""A qualified current plane may recover visibility of a frozen plane.

Synthetic depth establishes the branch contract, not a cause of past hardware
failures. No ROS or robot access is used.
"""
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from alicia_grasp_modes.tabletop import (
    TargetTracker, _audit_current_support, segment, segment_with_support_fallback)


class SupportVisibilityRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.intrinsics = (240., 240., 160., 120.)
        self.transform = np.eye(4)
        self.depth = np.full((240, 320), .300, dtype=np.float32)
        self.depth[112:128, 146:174] = .288
        self.tracker = TargetTracker()
        self.anchor, _ = self.tracker.choose(
            segment(self.depth, self.intrinsics, self.transform), self.depth.shape)
        self.assertIsNotNone(self.anchor)
        self.plane = self.tracker.plane_base.copy()

    def recover(self, depth):
        return segment_with_support_fallback(
            depth, self.intrinsics, self.transform, self.tracker.config,
            self.anchor, self.anchor, self.plane)

    def test_qualified_three_mm_shift_recovers_without_changing_identity(self):
        depth = self.depth + .003
        audit = _audit_current_support(
            depth, self.intrinsics, self.transform, self.tracker.config,
            self.anchor, self.plane)
        self.assertTrue(audit['support_current_audit_available'])
        self.assertEqual(audit['support_current_audit_inlier_ratio'], 1.)
        self.assertEqual(audit['support_current_audit_quadrants'], 4)
        self.assertLess(audit['support_reference_max_distance_m'], .004)
        with self.assertRaisesRegex(ValueError, 'locked_support_plane_no_longer_visible'):
            segment(depth, self.intrinsics, self.transform, self.tracker.config,
                    target_position_base=self.anchor.position_base,
                    plane_base=self.plane, fit_current_support=False,
                    target_footprint_base=self.anchor.support_footprint_base)
        result = self.recover(depth)
        self.assertEqual(result.metrics['support_reference_segmentation_error'],
                         'locked_support_plane_no_longer_visible')
        self.assertTrue(result.metrics['support_refit_attempted'])
        self.assertIs(self.tracker.last, self.anchor)
        selected, reason = self.tracker.choose(result, depth.shape)
        self.assertIsNotNone(selected)
        self.assertEqual(reason, 'tracked')
        self.assertIs(self.tracker.anchor, self.anchor)
        np.testing.assert_array_equal(self.tracker.plane_base, self.plane)
        self.tracker.unavailable()
        self.assertIsNone(self.tracker.choose(result, depth.shape)[0])

    def test_current_plane_outside_hard_tolerance_still_rejected(self):
        with self.assertRaisesRegex(ValueError, 'support_plane_local_distance_inconsistent'):
            self.recover(self.depth + .008)
        self.assertIs(self.tracker.last, self.anchor)
        np.testing.assert_array_equal(self.tracker.plane_base, self.plane)

    def test_unqualified_current_plane_does_not_admit_visibility_recovery(self):
        # All current support fit attempts fail their original ratio gate.
        with patch('alicia_grasp_modes.tabletop._fit_plane',
                   side_effect=ValueError('support_plane_inlier_ratio_low')):
            with self.assertRaisesRegex(ValueError, 'locked_support_plane_no_longer_visible'):
                self.recover(self.depth + .003)
        self.assertIs(self.tracker.last, self.anchor)
        self.assertFalse(self.tracker.lost)

    def test_recovered_support_cannot_choose_between_two_matching_objects(self):
        depth = np.full(self.depth.shape, .303, dtype=np.float32)
        depth[112:128, 136:152] = .291
        depth[112:128, 169:185] = .291
        result = self.recover(depth)
        selected, reason = self.tracker.choose(result, depth.shape)
        self.assertIsNone(selected)
        self.assertEqual(reason, 'ambiguous_locked_instances')
        self.assertTrue(self.tracker.lost)
        np.testing.assert_array_equal(self.tracker.plane_base, self.plane)


if __name__ == '__main__':
    unittest.main()
