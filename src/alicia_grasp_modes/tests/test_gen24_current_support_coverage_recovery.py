"""Use a qualified measured table when a frozen plane sees only a narrow strip.

The synthetic scene exercises the real fitter and original identity gates.
The separately retained hardware replay establishes the observed failure cause.
"""
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from alicia_grasp_modes.tabletop import (
    TargetTracker, _audit_current_support, segment, segment_with_support_fallback)


class CurrentSupportCoverageRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.intrinsics = (240., 240., 160., 120.)
        self.transform = np.eye(4)
        original = np.full((240, 320), .300, dtype=np.float32)
        original[112:128, 146:174] = .288
        self.tracker = TargetTracker()
        self.anchor, _ = self.tracker.choose(
            segment(original, self.intrinsics, self.transform), original.shape)
        self.assertIsNotNone(self.anchor)
        self.reference = self.tracker.plane_base.copy()

    def scene(self, shift=.003):
        depth = np.full((240, 320), .300 + shift, dtype=np.float32)
        depth[112:128, 146:174] = .288 + shift
        # A real measured strip still intersects the old plane, but only in
        # its two upper quadrants. This is outside the independent fit ring.
        depth[20:30, :] = .300
        return depth

    def reference_segment(self, depth):
        return segment(depth, self.intrinsics, self.transform, self.tracker.config,
            target_position_base=self.anchor.position_base, plane_base=self.reference,
            target_footprint_base=self.anchor.support_footprint_base, fit_current_support=False)

    def recover(self, depth):
        return segment_with_support_fallback(depth, self.intrinsics, self.transform,
            self.tracker.config, self.anchor, self.anchor, self.reference,
            prefer_current_support=True)

    def test_current_four_quadrants_recover_old_two_quadrant_strip(self):
        depth = self.scene()
        audit = _audit_current_support(depth, self.intrinsics, self.transform,
            self.tracker.config, self.anchor, self.reference)
        self.assertTrue(audit['support_current_audit_available'])
        self.assertEqual(audit['support_current_audit_quadrants'], 4)
        self.assertLess(audit['support_reference_max_distance_m'], .004)
        with self.assertRaisesRegex(ValueError, '^insufficient_support_spatial_coverage$'):
            self.reference_segment(depth)
        result = self.recover(depth)
        self.assertEqual(result.metrics['support_reference_segmentation_error'],
                         'insufficient_support_spatial_coverage')
        self.assertEqual(result.metrics['support_source'], 'current_ring_fit')
        self.assertTrue(result.metrics['support_current_audit_available'])
        self.assertTrue(result.metrics['support_refit_attempted'])
        self.assertIs(self.tracker.last, self.anchor)
        selected, reason = self.tracker.choose(result, depth.shape)
        self.assertIsNotNone(selected)
        self.assertEqual(reason, 'tracked')
        self.assertIs(self.tracker.anchor, self.anchor)
        np.testing.assert_array_equal(self.tracker.plane_base, self.reference)
        self.tracker.unavailable()
        self.assertIsNone(self.tracker.choose(result, depth.shape)[0])

    def test_missing_current_ring_cannot_recover_old_coverage(self):
        depth = self.scene()
        depth[30:, :] = 0.
        audit = _audit_current_support(depth, self.intrinsics, self.transform,
            self.tracker.config, self.anchor, self.reference)
        self.assertFalse(audit['support_current_audit_available'])
        with self.assertRaisesRegex(ValueError, '^insufficient_support_spatial_coverage$'):
            self.recover(depth)
        self.assertFalse(self.tracker.lost)

    def test_current_two_quadrants_remain_insufficient(self):
        depth = self.scene()
        depth[120:, :] = 0.
        audit = _audit_current_support(depth, self.intrinsics, self.transform,
            self.tracker.config, self.anchor, self.reference)
        self.assertFalse(audit['support_current_audit_available'])
        self.assertEqual(audit['support_current_audit_quadrants'], 2)
        with self.assertRaisesRegex(ValueError, '^insufficient_support_spatial_coverage$'):
            self.recover(depth)

    def test_positive_current_shift_conflict_still_rejected(self):
        with self.assertRaisesRegex(ValueError, 'support_plane_local_distance_inconsistent'):
            self.recover(self.scene(shift=.008))
        self.assertIs(self.tracker.last, self.anchor)
        np.testing.assert_array_equal(self.tracker.plane_base, self.reference)

    def test_current_refit_cannot_choose_between_two_measured_objects(self):
        depth = self.scene()
        depth[112:128, 146:174] = .303
        depth[112:128, 136:152] = .291
        depth[112:128, 169:185] = .291
        result = self.recover(depth)
        selected, reason = self.tracker.choose(result, depth.shape)
        self.assertIsNone(selected)
        self.assertEqual(reason, 'ambiguous_locked_instances')
        self.assertTrue(self.tracker.lost)
        np.testing.assert_array_equal(self.tracker.plane_base, self.reference)


if __name__ == '__main__':
    unittest.main()
