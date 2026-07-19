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


if __name__ == '__main__':
    unittest.main()
