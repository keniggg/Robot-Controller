#!/usr/bin/env python3
import pathlib
import sys
import unittest
from types import SimpleNamespace

import rospy


ROOT = pathlib.Path(__file__).resolve().parents[1]
SRC = ROOT / 'src'
for path in (ROOT, SRC):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from alicia_flexible_grasp.vision.pose_estimator import PoseEstimator


def make_transform(xyz=(0.0, 0.0, 0.0), xyzw=(0.0, 0.0, 0.0, 1.0)):
    return SimpleNamespace(
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=xyz[0], y=xyz[1], z=xyz[2]),
            rotation=SimpleNamespace(x=xyzw[0], y=xyzw[1], z=xyzw[2], w=xyzw[3]),
        )
    )


class StampThenLatestBuffer:
    def __init__(self):
        self.calls = []

    def lookup_transform(self, target, source, stamp, timeout):
        self.calls.append(float(stamp.to_sec()))
        if stamp != rospy.Time(0):
            raise RuntimeError('missing stamped transform')
        return make_transform((1.0, 2.0, 3.0))


class AlwaysFailBuffer:
    def lookup_transform(self, target, source, stamp, timeout):
        raise RuntimeError('no tf')


class PoseEstimatorTfFallbackTest(unittest.TestCase):
    def test_stamped_lookup_falls_back_to_latest_tf_before_static(self):
        buffer = StampThenLatestBuffer()
        estimator = PoseEstimator(
            'camera_link',
            'base_link',
            [9.0, 9.0, 9.0],
            [0.0, 0.0, 0.0, 1.0],
            [0.0, 0.0, 0.0, 1.0],
            tf_buffer=buffer,
            tf_lookup_latest=False,
            allow_static_fallback=False,
        )

        base = estimator._camera_point_to_base([0.1, 0.2, 0.3], 'camera_link', rospy.Time(123.0))

        self.assertEqual([round(v, 3) for v in base], [1.1, 2.2, 3.3])
        self.assertEqual(buffer.calls, [123.0, 0.0])
        self.assertEqual(estimator.last_transform_source, 'tf_latest_fallback')

    def test_static_fallback_can_be_disabled_for_base_pose_safety(self):
        estimator = PoseEstimator(
            'camera_link',
            'base_link',
            [9.0, 9.0, 9.0],
            [0.0, 0.0, 0.0, 1.0],
            [0.0, 0.0, 0.0, 1.0],
            tf_buffer=AlwaysFailBuffer(),
            allow_static_fallback=False,
        )

        with self.assertRaises(RuntimeError):
            estimator._camera_point_to_base([0.1, 0.2, 0.3], 'camera_link', rospy.Time(123.0))


if __name__ == '__main__':
    unittest.main()
