#!/usr/bin/env python3
import importlib.util
import pathlib
import tempfile
import textwrap
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'scripts' / 'handeye_transform_node.py'


def load_module():
    spec = importlib.util.spec_from_file_location('handeye_transform_node', str(SCRIPT))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class HandeyeTransformNodeTest(unittest.TestCase):
    def test_resolve_transform_config_loads_easy_handeye_eye_on_hand_file(self):
        module = load_module()
        with tempfile.NamedTemporaryFile('w', suffix='.yaml') as handle:
            handle.write(textwrap.dedent('''
                parameters:
                  eye_on_hand: true
                  robot_base_frame: base_link
                  robot_effector_frame: tool0
                  tracking_base_frame: camera_link
                transformation:
                  x: -0.084
                  y: 0.010
                  z: -0.123
                  qx: 0.004
                  qy: -0.685
                  qz: 0.000
                  qw: 0.728
            '''))
            handle.flush()

            resolved = module.resolve_transform_config({'calibration_file': handle.name})

        self.assertEqual(resolved['parent_frame'], 'tool0')
        self.assertEqual(resolved['child_frame'], 'camera_link')
        self.assertEqual(resolved['translation_xyz'], [-0.084, 0.010, -0.123])
        self.assertEqual(resolved['rotation_xyzw'], [0.004, -0.685, 0.000, 0.728])


if __name__ == '__main__':
    unittest.main()
