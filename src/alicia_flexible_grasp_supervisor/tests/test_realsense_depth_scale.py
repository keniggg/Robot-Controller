#!/usr/bin/env python3
import pathlib
import sys
import types
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
SRC_DIR = ROOT / 'src'
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from alicia_flexible_grasp.vision.realsense_manager import RealSenseManager


class FakeDepthSensor:
    def get_depth_scale(self):
        return 0.0001


class FakeDevice:
    def first_depth_sensor(self):
        return FakeDepthSensor()


class FakeProfile:
    def get_device(self):
        return FakeDevice()


class FakePipeline:
    def start(self, config):
        return FakeProfile()


class FakeConfig:
    def enable_stream(self, *args):
        pass


class FakeAlign:
    def __init__(self, stream):
        self.stream = stream


class RealSenseDepthScaleTest(unittest.TestCase):
    def test_start_reads_hardware_depth_scale(self):
        fake_rs = types.SimpleNamespace(
            pipeline=lambda: FakePipeline(),
            config=lambda: FakeConfig(),
            stream=types.SimpleNamespace(color='color', depth='depth'),
            format=types.SimpleNamespace(bgr8='bgr8', z16='z16'),
            align=FakeAlign,
        )
        original = sys.modules.get('pyrealsense2')
        sys.modules['pyrealsense2'] = fake_rs
        try:
            manager = RealSenseManager()
            manager.start()
        finally:
            if original is None:
                sys.modules.pop('pyrealsense2', None)
            else:
                sys.modules['pyrealsense2'] = original

        self.assertEqual(manager.depth_scale, 0.0001)


if __name__ == '__main__':
    unittest.main()
