#!/usr/bin/env python3
import pathlib
import unittest

import yaml


ROOT = pathlib.Path(__file__).resolve().parents[1]


class GraspNetInputDefaultConfigTest(unittest.TestCase):
    def test_production_default_keeps_target_with_local_support_context(self):
        with (ROOT / 'config' / 'grasp_params.yaml').open(
            'r', encoding='utf-8'
        ) as stream:
            remote = yaml.safe_load(stream)['grasp_6d']['remote']

        self.assertEqual(remote['graspnet_input_mode'], 'context_roi')
        self.assertTrue(remote['candidate_target_gate_enabled'])
        self.assertTrue(remote['target_cloud_support_plane_enabled'])

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
                'angle_step_deg': 15.0,
                'angle_dedup_deg': 2.0,
                'jaw_clearance_each_side_m': 0.002,
                'min_contact_band_points': 6,
                'contact_band_fraction': 0.12,
                'min_finger_support_clearance_m': 0.003,
                'max_candidates': 8,
                'merge_center_distance_m': 0.005,
                'merge_insertion_angle_deg': 10.0,
                'merge_jaw_angle_deg': 10.0,
            },
        )
        self.assertEqual(
            config['mujoco_digital_twin']['object_model']['type'],
            'obb_box',
        )


if __name__ == '__main__':
    unittest.main()
