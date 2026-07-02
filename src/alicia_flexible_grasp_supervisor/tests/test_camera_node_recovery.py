#!/usr/bin/env python3
import importlib.util
import pathlib
import sys
import unittest

import numpy as np


ROOT = pathlib.Path(__file__).resolve().parents[1]
CAMERA_NODE = ROOT / 'scripts' / 'camera_node.py'
SRC_DIR = ROOT / 'src'
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))


def load_camera_node():
    spec = importlib.util.spec_from_file_location('camera_node_under_test', str(CAMERA_NODE))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class FakeBridge:
    def cv2_to_imgmsg(self, cv_img, encoding):
        return type('FakeImage', (), {
            'image': cv_img,
            'encoding': encoding,
            'header': type('FakeHeader', (), {'stamp': None, 'frame_id': ''})(),
        })()


class FakeRate:
    def __init__(self, rospy):
        self.rospy = rospy

    def sleep(self):
        self.rospy.sleep_count += 1
        if self.rospy.sleep_count >= 1:
            self.rospy.shutdown = True


class FakeRospy:
    def __init__(self, camera_cfg=None):
        self.shutdown = False
        self.sleep_count = 0
        self.publishers = []
        self.params = {}
        self.errors = []
        self.warnings = []
        self.infos = []
        self.Time = type('FakeTime', (), {'now': staticmethod(lambda: 'now')})
        self.camera_cfg = {
            'width': 4,
            'height': 3,
            'fps': 30,
            'simulate': False,
            'fallback_to_simulation': True,
            'read_failure_limit': 1,
        }
        if camera_cfg:
            self.camera_cfg.update(camera_cfg)

    def get_param(self, name, default=None):
        return {'/camera': self.camera_cfg}.get(name, default)

    def set_param(self, name, value):
        self.params[name] = value

    def Publisher(self, *args, **kwargs):
        pub = FakePublisher()
        self.publishers.append(pub)
        return pub

    def Rate(self, hz):
        return FakeRate(self)

    def is_shutdown(self):
        return self.shutdown

    def loginfo(self, *args):
        self.infos.append(args)

    def logwarn(self, *args):
        self.warnings.append(args)

    def logwarn_throttle(self, *args):
        self.warnings.append(args)

    def logerr(self, *args):
        self.errors.append(args)


class FailingThenSimulatedCamera:
    created = []

    def __init__(self, width, height, fps, align_depth_to_color, simulate=False):
        self.width = int(width)
        self.height = int(height)
        self.simulate = bool(simulate)
        self.started = False
        self.stopped = False
        FailingThenSimulatedCamera.created.append(self)

    def start(self):
        self.started = True
        return True

    def stop(self):
        self.stopped = True

    def read(self):
        if not self.simulate:
            raise RuntimeError('Frame did not arrive within 5000')
        color = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        depth = np.ones((self.height, self.width), dtype=np.uint16)
        return color, depth


class FailingThenRestartedCamera:
    created = []

    def __init__(self, width, height, fps, align_depth_to_color, simulate=False):
        self.width = int(width)
        self.height = int(height)
        self.simulate = bool(simulate)
        self.index = len(FailingThenRestartedCamera.created)
        self.started = False
        self.stopped = False
        FailingThenRestartedCamera.created.append(self)

    def start(self):
        self.started = True
        return True

    def stop(self):
        self.stopped = True

    def read(self):
        if self.index == 0:
            raise RuntimeError('Frame did not arrive within 5000')
        if self.simulate:
            raise AssertionError('unexpected simulated camera')
        color = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        depth = np.ones((self.height, self.width), dtype=np.uint16)
        return color, depth


class StartupFailThenRestartedCamera:
    created = []

    def __init__(self, width, height, fps, align_depth_to_color, simulate=False):
        self.width = int(width)
        self.height = int(height)
        self.simulate = bool(simulate)
        self.index = len(StartupFailThenRestartedCamera.created)
        self.started = False
        self.stopped = False
        StartupFailThenRestartedCamera.created.append(self)

    def start(self):
        self.started = True
        if self.index == 0:
            raise RuntimeError('Device or resource busy')
        return True

    def stop(self):
        self.stopped = True

    def read(self):
        color = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        depth = np.ones((self.height, self.width), dtype=np.uint16)
        return color, depth


class CameraWithDepthScale:
    created = []

    def __init__(self, width, height, fps, align_depth_to_color, simulate=False):
        self.width = int(width)
        self.height = int(height)
        self.simulate = bool(simulate)
        self.depth_scale = 0.0001
        self.started = False
        CameraWithDepthScale.created.append(self)

    def start(self):
        self.started = True
        return True

    def stop(self):
        pass


class CameraNodeRecoveryTest(unittest.TestCase):
    def test_camera_node_publishes_runtime_depth_scale_param(self):
        module = load_camera_node()
        fake_rospy = FakeRospy({'fallback_to_simulation': False})
        CameraWithDepthScale.created = []
        module.rospy = fake_rospy
        module.CvBridge = FakeBridge
        module.RealSenseManager = CameraWithDepthScale

        module.CameraNode()

        self.assertEqual(fake_rospy.params['/camera/depth_scale'], 0.0001)

    def test_read_timeout_falls_back_without_exiting(self):
        module = load_camera_node()
        fake_rospy = FakeRospy()
        FailingThenSimulatedCamera.created = []
        module.rospy = fake_rospy
        module.CvBridge = FakeBridge
        module.RealSenseManager = FailingThenSimulatedCamera

        node = module.CameraNode()
        node.spin()

        self.assertGreaterEqual(len(FailingThenSimulatedCamera.created), 2)
        self.assertTrue(FailingThenSimulatedCamera.created[0].stopped)
        self.assertTrue(FailingThenSimulatedCamera.created[-1].simulate)
        self.assertEqual(len(fake_rospy.publishers[0].messages), 1)
        self.assertEqual(len(fake_rospy.publishers[1].messages), 1)

    def test_read_timeout_restarts_real_camera_when_fallback_disabled(self):
        module = load_camera_node()
        fake_rospy = FakeRospy({'fallback_to_simulation': False})
        FailingThenRestartedCamera.created = []
        module.rospy = fake_rospy
        module.CvBridge = FakeBridge
        module.RealSenseManager = FailingThenRestartedCamera

        node = module.CameraNode()
        node.spin()

        self.assertGreaterEqual(len(FailingThenRestartedCamera.created), 2)
        self.assertTrue(FailingThenRestartedCamera.created[0].stopped)
        self.assertFalse(any(cam.simulate for cam in FailingThenRestartedCamera.created))
        self.assertEqual(len(fake_rospy.publishers[0].messages), 1)
        self.assertEqual(len(fake_rospy.publishers[1].messages), 1)

    def test_startup_device_busy_keeps_node_alive_and_retries_real_camera(self):
        module = load_camera_node()
        fake_rospy = FakeRospy({'fallback_to_simulation': False})
        StartupFailThenRestartedCamera.created = []
        module.rospy = fake_rospy
        module.CvBridge = FakeBridge
        module.RealSenseManager = StartupFailThenRestartedCamera

        node = module.CameraNode()
        node.spin()

        self.assertGreaterEqual(len(StartupFailThenRestartedCamera.created), 2)
        self.assertTrue(StartupFailThenRestartedCamera.created[0].started)
        self.assertTrue(StartupFailThenRestartedCamera.created[0].stopped)
        self.assertEqual(len(fake_rospy.publishers[0].messages), 1)
        self.assertEqual(len(fake_rospy.publishers[1].messages), 1)


if __name__ == '__main__':
    unittest.main()
