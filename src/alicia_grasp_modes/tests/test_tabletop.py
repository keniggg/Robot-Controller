"""Offline geometry/identity regression tests; no ROS or arm connection."""
import json
import sys
import unittest
from unittest.mock import patch
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from alicia_grasp_modes.tabletop import (Config, TargetTracker, Segmentation, segment,
                                        segment_with_support_fallback,
                                        _matching_candidates)


class TabletopTests(unittest.TestCase):
    intrinsics = (240.0, 240.0, 160.0, 120.0)

    def scene(self, objects=(), translation=(0.0, 0.0, 0.0)):
        transform = np.eye(4)
        transform[:3, 3] = translation
        depth = np.full((240, 320), 0.30 - translation[0], np.float32)
        for position in objects:
            x, y, z = np.asarray(position) - np.asarray(translation)
            u, v = 160 - 240*y/x, 120 - 240*z/x
            half_w, half_h = 240*0.017/x, 240*0.009/x
            left, right = max(0, int(round(u-half_w))), min(320, int(round(u+half_w)))
            top, bottom = max(0, int(round(v-half_h))), min(240, int(round(v+half_h)))
            depth[top:bottom, left:right] = x
        return depth, transform

    def test_empty_table_is_not_an_object(self):
        depth, transform = self.scene()
        result = segment(depth, self.intrinsics, transform)
        selected, reason = TargetTracker().choose(result, depth.shape)
        self.assertEqual(result.candidates, [])
        self.assertIsNone(selected)
        self.assertEqual(reason, 'no_center_instance')

    def test_two_equally_centered_objects_are_ambiguous(self):
        # Separate instances have equally good center distance.
        depth, transform = self.scene([(0.288, 0.029, 0), (0.288, -0.029, 0)])
        selected, reason = TargetTracker().choose(segment(depth, self.intrinsics, transform), depth.shape)
        self.assertIsNone(selected)
        self.assertEqual(reason, 'ambiguous_center_instances')

    def test_depth_holes_never_become_measured_mask(self):
        depth, transform = self.scene([(0.288, 0, 0)])
        depth[116:123, 157:164] = 0.0
        selected, _ = TargetTracker().choose(segment(depth, self.intrinsics, transform), depth.shape)
        self.assertIsNotNone(selected)
        self.assertFalse(np.any(selected.mask[depth == 0]))
        self.assertGreater(selected.quality, 0.5)
        self.assertAlmostEqual(selected.depth_m, 0.288, places=5)

    def test_no_valid_depth_is_explicit(self):
        with self.assertRaisesRegex(ValueError, 'insufficient_valid_depth'):
            segment(np.zeros((240, 320)), self.intrinsics, np.eye(4))

    def test_target_loss_does_not_jump_to_new_object(self):
        tracker = TargetTracker()
        depth, transform = self.scene([(0.288, 0, 0)])
        first, _ = tracker.choose(segment(depth, self.intrinsics, transform), depth.shape)
        self.assertIsNotNone(first)
        empty, _ = self.scene()
        selected, reason = tracker.choose(segment(empty, self.intrinsics, transform), empty.shape)
        self.assertIsNone(selected)
        self.assertTrue(tracker.lost)
        # Even indistinguishable replacement at the exact same location is not
        # a new authorization after observed loss.
        selected, reason = tracker.choose(segment(depth, self.intrinsics, transform), depth.shape)
        self.assertIsNone(selected)
        self.assertEqual(reason, 'target_lost_requires_new_generation')
        tracker.reset()
        selected, _ = tracker.choose(segment(depth, self.intrinsics, transform), depth.shape)
        self.assertIsNotNone(selected)

    def test_camera_approach_tracks_base_position_not_screen_center(self):
        tracker = TargetTracker()
        depth, transform = self.scene([(0.288, 0, 0)])
        selected, _ = tracker.choose(segment(depth, self.intrinsics, transform), depth.shape)
        initial_base = selected.position_base.copy()
        depth, transform = self.scene([(0.288, 0, 0)], translation=(0.06, 0.032, 0))
        result = segment(depth, self.intrinsics, transform,
                         target_position_base=tracker.anchor.position_base,
                         plane_base=tracker.plane_base,
                         target_footprint_base=tracker.anchor.support_footprint_base)
        selected, _ = tracker.choose(result, depth.shape)
        self.assertIsNotNone(selected)
        self.assertGreater(selected.center_uv[0], 190)
        self.assertLess(np.linalg.norm(selected.position_base-initial_base), 0.002)
        self.assertEqual(selected.metrics['support_source'], 'current_ring_fit')

    def test_missing_evidence_latches_previously_selected_target(self):
        tracker = TargetTracker()
        depth, transform = self.scene([(0.288, 0, 0)])
        result = segment(depth, self.intrinsics, transform)
        tracker.choose(result, depth.shape)
        tracker.unavailable()
        selected, reason = tracker.choose(result, depth.shape)
        self.assertIsNone(selected)
        self.assertEqual(reason, 'target_lost_requires_new_generation')

    def test_camera_rotation_retains_plane_and_target_in_base(self):
        tracker = TargetTracker()
        depth, transform = self.scene([(0.288, 0, 0)])
        first, _ = tracker.choose(segment(depth, self.intrinsics, transform), depth.shape)
        angle = np.deg2rad(9)
        transform[:3, :3] = [[np.cos(angle), -np.sin(angle), 0],
                             [np.sin(angle), np.cos(angle), 0], [0, 0, 1]]
        transform[:3, 3] = [0.035, -0.01, 0]
        v, u = np.indices(depth.shape)
        rays = np.stack((np.ones(depth.shape), -(u-160)/240, -(v-120)/240), axis=-1)
        base_rays = rays @ transform[:3, :3].T
        depth = ((0.30-transform[0, 3]) / base_rays[:, :, 0]).astype(np.float32)
        object_depth = (0.288-transform[0, 3]) / base_rays[:, :, 0]
        object_points = object_depth[:, :, None]*base_rays + transform[:3, 3]
        object_pixels = (np.abs(object_points[:, :, 1]) < 0.017) & (np.abs(object_points[:, :, 2]) < 0.009)
        depth[object_pixels] = object_depth[object_pixels]
        result = segment(depth, self.intrinsics, transform,
                         target_position_base=first.position_base,
                         plane_base=tracker.plane_base,
                         target_footprint_base=first.support_footprint_base)
        selected, _ = tracker.choose(result, depth.shape)
        self.assertIsNotNone(selected)
        self.assertLess(np.linalg.norm(selected.position_base-first.position_base), 0.003)
        self.assertGreater(selected.center_uv[0], 180)

    def test_far_center_replacement_cannot_take_lock(self):
        tracker = TargetTracker()
        depth, transform = self.scene([(0.288, 0, 0)])
        tracker.choose(segment(depth, self.intrinsics, transform), depth.shape)
        # The camera moves and a different object becomes centered.
        depth, transform = self.scene([(0.288, 0.055, 0)], translation=(0, 0.055, 0))
        selected, _ = tracker.choose(segment(depth, self.intrinsics, transform), depth.shape)
        self.assertIsNone(selected)
        self.assertTrue(tracker.lost)

    def test_touching_depth_discontinuity_is_split(self):
        depth, transform = self.scene()
        depth[113:128, 143:160] = 0.286
        depth[113:128, 160:177] = 0.267
        result = segment(depth, self.intrinsics, transform)
        self.assertEqual(len(result.candidates), 2)

    def test_actual_saved_rgbd_snapshot(self):
        directory = Path('/home/zhuyupei/alicia_wa_full/.ros_log/ros_live_20260921_session/unknown_sample')
        if not (directory / 'source.json').exists():
            self.skipTest('local measured RGB-D fixture is not distributed')
        source = json.loads((directory / 'source.json').read_text())
        self.assertEqual(source['color_stamp_ns'], source['depth_stamp_ns'])
        camera = source['camera']
        depth = np.load(directory / 'depth.npy') * camera['depth_scale']
        intrinsics = [camera[key] for key in ('fx', 'fy', 'cx', 'cy')]
        result = segment(depth, intrinsics, np.eye(4))
        chosen, _ = TargetTracker().choose(result, depth.shape)
        self.assertIsNotNone(chosen)
        self.assertTrue(295 <= chosen.bbox[0] <= 310)
        self.assertGreater(chosen.metrics['valid_points'], 1000)
        self.assertLess(chosen.extent_m[0], 0.055)
        self.assertGreater(chosen.extent_m[0], 0.040)
        self.assertLess(chosen.metrics['support_sample_count'], 2000)
        self.assertGreater(chosen.quality, 0.5)

    def test_measured_roll_frame_repairs_merged_foreground_before_single_commit(self):
        fixture = np.load(Path(__file__).parent / 'fixtures' / 'unknown_support_merge.npz')
        scale = float(fixture['depth_scale'])
        intrinsics = fixture['intrinsics']
        tracker = TargetTracker()
        initial_depth = fixture['initial_depth'].astype(np.float32) * scale
        initial = segment_with_support_fallback(initial_depth, intrinsics,
                                               fixture['initial_transform'])
        anchor, _ = tracker.choose(initial, initial_depth.shape)
        original_plane = tracker.plane_base.copy()
        depth = fixture['failure_depth'].astype(np.float32) * scale
        transform = fixture['failure_transform']
        contaminated = segment(depth, intrinsics, transform,
                               target_position_base=anchor.position_base,
                               plane_base=original_plane,
                               fit_current_support=False)
        self.assertEqual(_matching_candidates(contaminated.candidates, anchor,
                                              anchor, tracker.config), [])
        repaired = segment_with_support_fallback(
            depth, intrinsics, transform, tracker.config, anchor, anchor, original_plane)
        self.assertTrue(repaired.metrics['support_refit_attempted'])
        self.assertLess(repaired.metrics['support_reference_angle_deg'], 4.)
        self.assertLess(repaired.metrics['support_reference_max_distance_m'], .004)
        # Segmentation, including the rejected primary candidate, has not
        # committed or reset target identity. Only the final choose commits.
        self.assertIs(tracker.anchor, anchor)
        self.assertIs(tracker.last, anchor)
        self.assertFalse(tracker.lost)
        selected, _ = tracker.choose(repaired, depth.shape)
        self.assertIsNotNone(selected)
        self.assertGreater(selected.metrics['valid_points'], 1500)
        self.assertLess(selected.metrics['valid_points'], 2500)
        self.assertLess(selected.extent_m[1], .025)
        np.testing.assert_array_equal(tracker.plane_base, original_plane)
        # Re-running after an explicit target loss must never clear that loss.
        tracker.unavailable()
        self.assertIsNone(tracker.choose(repaired, depth.shape)[0])

    def test_valid_reference_segmentation_does_not_refit(self):
        tracker = TargetTracker()
        depth, transform = self.scene([(0.288, 0, 0)])
        initial = segment_with_support_fallback(depth, self.intrinsics, transform)
        anchor, _ = tracker.choose(initial, depth.shape)
        result = segment_with_support_fallback(depth, self.intrinsics, transform,
                                              tracker.config, anchor, anchor,
                                              tracker.plane_base)
        self.assertEqual(result.metrics['support_source'], 'locked_base_plane')
        self.assertFalse(result.metrics.get('support_refit_attempted', False))

    def test_reference_component_cap_uses_validated_same_frame_refit(self):
        depth, transform = self.scene([(0.288, 0, 0)])
        for reason in ('too_many_foreground_instances', 'too_many_depth_components'):
            tracker = TargetTracker()
            anchor, _ = tracker.choose(segment(depth, self.intrinsics, transform), depth.shape)
            plane = tracker.plane_base.copy()
            def capped(*args, **kwargs):
                if kwargs.get('fit_current_support') is False:
                    raise ValueError(reason)
                return segment(*args, **kwargs)
            with patch('alicia_grasp_modes.tabletop.segment', side_effect=capped) as measured:
                result = segment_with_support_fallback(depth, self.intrinsics, transform,
                    tracker.config, anchor, anchor, plane)
            self.assertEqual(measured.call_count, 2)
            self.assertEqual(result.metrics['support_reference_segmentation_error'], reason)
            self.assertLess(result.metrics['support_reference_max_distance_m'], .004)
            self.assertIs(tracker.last, anchor)
            self.assertIsNotNone(tracker.choose(result, depth.shape)[0])
            np.testing.assert_array_equal(tracker.plane_base, plane)
            tracker.unavailable()
            self.assertIsNone(tracker.choose(result, depth.shape)[0])

    def test_component_cap_refit_does_not_admit_changed_support_plane(self):
        depth, transform = self.scene([(0.288, 0, 0)])
        tracker = TargetTracker()
        anchor, _ = tracker.choose(segment(depth, self.intrinsics, transform), depth.shape)
        def capped(*args, **kwargs):
            if kwargs.get('fit_current_support') is False:
                raise ValueError('too_many_foreground_instances')
            return segment(*args, **kwargs)
        with patch('alicia_grasp_modes.tabletop.segment', side_effect=capped):
            with self.assertRaisesRegex(ValueError, 'support_plane_local_distance_inconsistent'):
                segment_with_support_fallback(depth + .005, self.intrinsics, transform,
                    tracker.config, anchor, anchor, tracker.plane_base)
        self.assertIs(tracker.last, anchor)

    def test_actual_reference_plane_texture_overflow_can_refit_before_loss(self):
        depth, transform = self.scene([(0.288, 0, 0)])
        tracker = TargetTracker()
        anchor, _ = tracker.choose(segment(depth, self.intrinsics, transform), depth.shape)
        # A 3 mm transported-plane difference makes shallow table texture
        # foreground under the old fit. The same frame's measured table is
        # still within the original 4 mm agreement limit.
        depth -= .003
        # Remote patches still support the transported plane, as can happen
        # under a slightly rotated fit. They are outside the current fit ring.
        depth[:10,:10] = depth[:10,-10:] = .30
        depth[-10:,:10] = depth[-10:,-10:] = .30
        for y in (20,50,80,155,185,215):
            for x in (15,60,105,200,245,290):
                depth[y:y+8,x:x+8] = .2955
        with self.assertRaisesRegex(ValueError, 'too_many_foreground_instances'):
            segment(depth, self.intrinsics, transform, tracker.config,
                    target_position_base=anchor.position_base, plane_base=tracker.plane_base,
                    fit_current_support=False)
        result = segment_with_support_fallback(depth, self.intrinsics, transform,
            tracker.config, anchor, anchor, tracker.plane_base)
        self.assertEqual(result.metrics['support_reference_segmentation_error'], 'too_many_foreground_instances')
        self.assertLess(result.metrics['support_reference_max_distance_m'], .004)
        selected, _ = tracker.choose(result, depth.shape)
        self.assertIsNotNone(selected)
        self.assertLess(np.linalg.norm(selected.position_base-anchor.position_base), .004)

    def test_refit_does_not_hide_unrelated_segmentation_failure(self):
        depth, transform = self.scene([(0.288, 0, 0)])
        tracker = TargetTracker()
        anchor, _ = tracker.choose(segment(depth, self.intrinsics, transform), depth.shape)
        with patch('alicia_grasp_modes.tabletop.segment', side_effect=ValueError('invalid_exact_transform')) as measured:
            with self.assertRaisesRegex(ValueError, 'invalid_exact_transform'):
                segment_with_support_fallback(depth, self.intrinsics, transform,
                    tracker.config, anchor, anchor, tracker.plane_base)
        self.assertEqual(measured.call_count, 1)

    def test_support_refit_cannot_disambiguate_two_existing_eligible_instances(self):
        tracker = TargetTracker()
        depth, transform = self.scene([(0.288, 0, 0)])
        initial = segment(depth, self.intrinsics, transform)
        anchor, _ = tracker.choose(initial, depth.shape)
        from copy import deepcopy
        neighbor = deepcopy(anchor)
        neighbor.position_base = anchor.position_base + np.array([0, .015, 0])
        ambiguous = Segmentation([anchor, neighbor], tracker.plane_base, {})
        # Supplying the alternate segment result would allow a plane refit to
        # hide the second valid identity. That refit must never be requested.
        with patch('alicia_grasp_modes.tabletop.segment', return_value=ambiguous) as measured:
            result = segment_with_support_fallback(
                depth, self.intrinsics, transform, tracker.config,
                anchor, anchor, tracker.plane_base)
        self.assertEqual(measured.call_count, 1)
        selected, reason = tracker.choose(result, depth.shape)
        self.assertIsNone(selected)
        self.assertEqual(reason, 'ambiguous_locked_instances')
        self.assertTrue(tracker.lost)


if __name__ == '__main__':
    unittest.main()
