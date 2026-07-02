#!/usr/bin/env python3
import importlib.util
import pathlib
import types
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
PERCEPTION_NODE = ROOT / 'scripts' / 'perception_node.py'


def load_perception_node():
    spec = importlib.util.spec_from_file_location('perception_node_under_test', str(PERCEPTION_NODE))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def fake_object(detected, label='target'):
    return types.SimpleNamespace(
        detected=detected,
        label=label,
        u=100,
        v=120,
        bbox_x=80,
        bbox_y=100,
        bbox_width=40,
        bbox_height=40,
        header=types.SimpleNamespace(stamp=None, frame_id='base_link'),
    )


class DetectionStabilityTest(unittest.TestCase):
    def test_recent_detection_is_held_during_short_miss(self):
        module = load_perception_node()
        stabilizer = module.DetectionStabilizer(hold_seconds=0.8)

        detected = stabilizer.update(fake_object(True, '绿色圆形'), now=10.0)
        held = stabilizer.update(fake_object(False, '绿色圆形'), now=10.4)

        self.assertTrue(detected.detected)
        self.assertTrue(held.detected)
        self.assertEqual(held.label, '绿色圆形')

    def test_detection_hold_expires_after_timeout(self):
        module = load_perception_node()
        stabilizer = module.DetectionStabilizer(hold_seconds=0.8)

        stabilizer.update(fake_object(True, '绿色圆形'), now=10.0)
        missed = stabilizer.update(fake_object(False, '绿色圆形'), now=11.0)

        self.assertFalse(missed.detected)

    def test_far_false_candidate_does_not_replace_locked_detection_immediately(self):
        module = load_perception_node()
        stabilizer = module.DetectionStabilizer(
            hold_seconds=0.8,
            max_jump_px=80.0,
            switch_confirmations=2,
        )

        target = fake_object(True, '绿色圆形')
        held_target = stabilizer.update(target, now=10.0)
        distractor = fake_object(True, '绿色圆形')
        distractor.u = 340
        distractor.v = 110
        distractor.bbox_x = 315
        distractor.bbox_y = 85
        distractor.bbox_width = 50
        distractor.bbox_height = 50
        stable = stabilizer.update(distractor, now=10.1)

        self.assertTrue(held_target.detected)
        self.assertTrue(stable.detected)
        self.assertEqual(stable.u, target.u)
        self.assertEqual(stable.bbox_x, target.bbox_x)


if __name__ == '__main__':
    unittest.main()
