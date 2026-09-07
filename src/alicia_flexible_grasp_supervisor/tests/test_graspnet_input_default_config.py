#!/usr/bin/env python3
import pathlib
import unittest

import yaml


ROOT = pathlib.Path(__file__).resolve().parents[1]


class GraspNetInputDefaultConfigTest(unittest.TestCase):
    def test_runtime_defaults_keep_ros_and_wsl_execution_age_aligned(self):
        with (ROOT / 'config' / 'grasp_params.yaml').open(
            'r', encoding='utf-8'
        ) as stream:
            config = yaml.safe_load(stream)
        start_script = (ROOT.parents[1] / 'tools' / 'start_mujoco_digital_twin_wsl.sh').read_text(
            encoding='utf-8'
        )

        self.assertEqual(config['grasp_6d']['plan_validity_sec'], 120.0)
        self.assertEqual(
            config['mujoco_digital_twin'][
                'selection_snapshot_reserve_sec'
            ],
            30.0,
        )
        self.assertEqual(
            config['grasp']['near_field_replan_timeout_sec'],
            60.0,
        )
        self.assertEqual(
            config['grasp']['near_field_strategy'],
            'single_snapshot_direct',
        )
        self.assertTrue(
            config['grasp']['post_lift_visual_verification_enabled']
        )
        self.assertTrue(config['grasp']['final_visual_refine_enabled'])
        self.assertTrue(config['grasp']['final_visual_refine_required'])
        self.assertFalse(
            config['grasp']['final_visual_refine_center_fallback_enabled']
        )
        self.assertNotIn(
            'post_lift_visual_verification_enabled',
            config['grasp_6d'],
        )
        self.assertEqual(
            config['gripper'][
                'plan_bound_opening_clearance_each_side_m'
            ],
            0.002,
        )
        self.assertIn('--max-snapshot-age-sec', start_script)
        self.assertIn('MUJOCO_MAX_SNAPSHOT_AGE_SEC:-120.0', start_script)

    def test_runtime_default_allows_measured_segmentation_latency(self):
        with (ROOT / 'config' / 'grasp_params.yaml').open(
            'r', encoding='utf-8'
        ) as stream:
            grasp6d = yaml.safe_load(stream)['grasp_6d']

        self.assertEqual(
            grasp6d['target_observation_validity_sec'],
            8.0,
        )

    def test_production_default_keeps_target_with_local_support_context(self):
        with (ROOT / 'config' / 'grasp_params.yaml').open(
            'r', encoding='utf-8'
        ) as stream:
            remote = yaml.safe_load(stream)['grasp_6d']['remote']

        self.assertEqual(remote['graspnet_input_mode'], 'context_roi')
        self.assertTrue(remote['candidate_target_gate_enabled'])
        self.assertTrue(remote['target_cloud_support_plane_enabled'])

    def test_production_default_uses_exact_bounded_multiview_registration(self):
        with (ROOT / 'config' / 'grasp_params.yaml').open(
            'r', encoding='utf-8'
        ) as stream:
            remote = yaml.safe_load(stream)['grasp_6d']['remote']

        self.assertEqual(
            remote['multiview'],
            {
                'correspondence_max_m': 0.008,
                'minimum_inliers': 80,
                'minimum_overlap_fraction': 0.30,
                'maximum_rmse_m': 0.004,
                'maximum_translation_m': 0.025,
                'maximum_yaw_deg': 10.0,
                'maximum_support_normal_angle_deg': 4.0,
                'maximum_support_offset_delta_m': 0.004,
                'maximum_iterations': 12,
            },
        )

    def test_production_default_keeps_joint_flip_gate_tightly_bounded(self):
        with (ROOT / 'config' / 'grasp_params.yaml').open(
            'r', encoding='utf-8'
        ) as stream:
            remote = yaml.safe_load(stream)['grasp_6d']['remote']

        self.assertEqual(remote['candidate_max_joint_delta_rad'], 0.0)
        self.assertFalse(remote['camera_visibility_gate_enabled'])
        self.assertTrue(remote['camera_visibility_diagnostic_enabled'])

    def test_production_default_uses_adaptive_contact_stages(self):
        with (ROOT / 'config' / 'grasp_params.yaml').open(
            'r', encoding='utf-8'
        ) as stream:
            config = yaml.safe_load(stream)
        grasp = config['grasp']
        remote = config['grasp_6d']['remote']

        self.assertEqual(
            grasp['observation_camera_target_nominal_distance_m'],
            0.200,
        )
        self.assertEqual(
            grasp['observation_camera_target_min_distance_m'],
            0.180,
        )
        self.assertEqual(
            grasp['observation_camera_target_max_distance_m'],
            0.220,
        )
        self.assertTrue(
            grasp['observation_camera_target_range_check_enabled']
        )
        self.assertTrue(
            grasp[
                'observation_camera_target_retreat_correction_enabled'
            ]
        )
        adaptive = remote['adaptive_stage_generation']
        self.assertTrue(adaptive['enabled'])
        self.assertEqual(adaptive['tilt_sample_count'], 4)
        self.assertEqual(adaptive['max_tilt_deg'], 45.0)
        self.assertLess(adaptive['approach_min_m'], adaptive['approach_max_m'])
        self.assertLess(adaptive['pregrasp_min_m'], adaptive['pregrasp_max_m'])
        self.assertLess(adaptive['lift_min_m'], adaptive['lift_max_m'])
        self.assertGreaterEqual(adaptive['approach_max_m'], 0.060)
        self.assertGreaterEqual(adaptive['pregrasp_max_m'], 0.095)
        self.assertTrue(grasp['measured_endpoint_check_enabled'])
        self.assertTrue(grasp['require_actuation_confirmation'])
        self.assertEqual(
            grasp['actuation_confirmation_freshness_sec'],
            2.0,
        )
        self.assertLessEqual(
            grasp['measured_endpoint_position_tolerance_m'],
            0.006,
        )
        self.assertEqual(
            grasp['observation_endpoint_correction_attempts'],
            0,
        )
        self.assertTrue(grasp['final_visual_refine_enabled'])
        self.assertTrue(grasp['final_visual_refine_required'])
        self.assertEqual(
            grasp['final_visual_refine_max_translation_m'],
            0.025,
        )
        self.assertEqual(grasp['final_visual_refine_max_yaw_deg'], 10.0)
        self.assertEqual(
            grasp['final_visual_refine_max_roll_pitch_change_deg'],
            4.0,
        )
        self.assertFalse(
            grasp['final_visual_refine_center_fallback_enabled']
        )
        self.assertEqual(
            grasp['final_visual_refine_center_fallback_delay_sec'],
            0.0,
        )
        self.assertEqual(
            grasp['final_visual_refine_center_fallback_required_samples'],
            5,
        )
        self.assertEqual(
            grasp['final_visual_refine_center_fallback_max_jitter_m'],
            0.006,
        )
        self.assertEqual(
            grasp['final_visual_refine_center_fallback_edge_margin_px'],
            4,
        )
        self.assertTrue(
            grasp['final_visual_refine_post_move_confirm_enabled']
        )
        self.assertEqual(
            grasp['final_visual_refine_post_move_confirm_timeout_sec'],
            12.0,
        )
        self.assertEqual(
            grasp[
                'final_visual_refine_post_move_confirm_required_samples'
            ],
            5,
        )
        self.assertEqual(
            grasp['final_visual_refine_post_move_confirm_max_jitter_m'],
            0.006,
        )
        self.assertEqual(
            grasp['final_visual_refine_post_move_confirm_max_residual_m'],
            0.006,
        )
        self.assertEqual(remote['planning_snapshot_frames'], 5)
        self.assertEqual(remote['near_field_planning_snapshot_frames'], 3)
        self.assertEqual(remote['planning_snapshot_timeout_sec'], 8.0)
        self.assertEqual(
            remote['planning_snapshot_max_inference_latency_sec'],
            5.0,
        )
        self.assertEqual(remote['planning_snapshot_max_span_sec'], 12.0)
        self.assertEqual(remote['max_candidates'], 300)
        self.assertEqual(remote['near_field_max_candidates'], 12)

    def test_production_default_enables_bounded_tabletop_geometry_candidates(self):
        with (ROOT / 'config' / 'grasp_params.yaml').open(
            'r', encoding='utf-8'
        ) as stream:
            config = yaml.safe_load(stream)

        geometry = config['grasp_6d']['remote']['tabletop_geometry_candidates']
        self.assertEqual(
            geometry,
            {
                'enabled': True,
                'angle_step_deg': 5.0,
                'angle_dedup_deg': 1.0,
                'jaw_clearance_each_side_m': 0.002,
                'width_projection_trim_fraction': 0.01,
                'min_contact_band_points': 6,
                'contact_band_fraction': 0.12,
                'min_finger_support_clearance_m': 0.003,
                'max_candidates': 32,
                'approach_tilt_degrees': [],
                'merge_center_distance_m': 0.005,
                'merge_insertion_angle_deg': 10.0,
                'merge_jaw_angle_deg': 10.0,
            },
        )
        self.assertEqual(
            config['grasp_6d']['remote']['candidate_min_downward_approach_cos'],
            0.65,
        )
        self.assertEqual(
            config['grasp_6d']['remote'][
                'candidate_max_final_approach_lateral_m'
            ],
            0.020,
        )
        self.assertEqual(
            config['mujoco_digital_twin']['object_model']['type'],
            'obb_box',
        )


if __name__ == '__main__':
    unittest.main()
