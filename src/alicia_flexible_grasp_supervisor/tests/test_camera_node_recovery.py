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
    def __init__(self, camera_cfg=None, perception_cfg=None):
        self.shutdown = False
        self.shutdown_after_stamp = False
        self.sleep_count = 0
        self.publishers = []
        self.params = {}
        self.errors = []
        self.warnings = []
        self.infos = []
        self.time_values = ['now']
        self.time_calls = 0

        def now():
            index = min(self.time_calls, len(self.time_values) - 1)
            self.time_calls += 1
            if self.shutdown_after_stamp:
                self.shutdown = True
            return self.time_values[index]

        self.Time = type('FakeTime', (), {'now': staticmethod(now)})
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
        self.perception_cfg = {
            'depth_min_m': 0.03,
            'depth_max_m': 2.0,
        }
        if perception_cfg:
            self.perception_cfg.update(perception_cfg)

    def get_param(self, name, default=None):
        return {
            '/camera': self.camera_cfg,
            '/perception': self.perception_cfg,
        }.get(name, default)

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

    def __init__(
        self,
        width,
        height,
        fps,
        align_depth_to_color,
        simulate=False,
        depth_filter_cfg=None,
    ):
        self.width = int(width)
        self.height = int(height)
        self.simulate = bool(simulate)
        self.depth_filter_cfg = dict(depth_filter_cfg or {})
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

    def __init__(
        self,
        width,
        height,
        fps,
        align_depth_to_color,
        simulate=False,
        depth_filter_cfg=None,
    ):
        self.width = int(width)
        self.height = int(height)
        self.simulate = bool(simulate)
        self.depth_filter_cfg = dict(depth_filter_cfg or {})
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

    def __init__(
        self,
        width,
        height,
        fps,
        align_depth_to_color,
        simulate=False,
        depth_filter_cfg=None,
    ):
        self.width = int(width)
        self.height = int(height)
        self.simulate = bool(simulate)
        self.depth_filter_cfg = dict(depth_filter_cfg or {})
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

    def __init__(
        self,
        width,
        height,
        fps,
        align_depth_to_color,
        simulate=False,
        depth_filter_cfg=None,
    ):
        self.width = int(width)
        self.height = int(height)
        self.simulate = bool(simulate)
        self.depth_filter_cfg = dict(depth_filter_cfg or {})
        self.depth_scale = 0.0001
        self.started = False
        CameraWithDepthScale.created.append(self)

    def start(self):
        self.started = True
        return True

    def stop(self):
        pass


class SinglePairCamera(CameraWithDepthScale):
    created = []

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        SinglePairCamera.created.append(self)

    def read(self):
        color = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        depth = np.ones((self.height, self.width), dtype=np.uint16)
        return color, depth


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

    def test_camera_node_passes_filter_and_perception_depth_limits(self):
        module = load_camera_node()
        fake_rospy = FakeRospy(
            camera_cfg={'depth_filter': {
                'spatial_magnitude': 1,
                'depth_min_m': 0.05,
            }},
            perception_cfg={'depth_min_m': 0.04, 'depth_max_m': 1.5},
        )
        CameraWithDepthScale.created = []
        module.rospy = fake_rospy
        module.CvBridge = FakeBridge
        module.RealSenseManager = CameraWithDepthScale

        module.CameraNode()

        self.assertEqual(CameraWithDepthScale.created[-1].depth_filter_cfg, {
            'spatial_magnitude': 1,
            'depth_min_m': 0.05,
            'depth_max_m': 1.5,
        })

    def test_published_color_and_depth_share_one_acquisition_stamp(self):
        module = load_camera_node()
        fake_rospy = FakeRospy({'fallback_to_simulation': False})
        fake_rospy.shutdown_after_stamp = True
        fake_rospy.time_values = ['pair-stamp', 'separate-stamp']
        SinglePairCamera.created = []
        CameraWithDepthScale.created = []
        module.rospy = fake_rospy
        module.CvBridge = FakeBridge
        module.RealSenseManager = SinglePairCamera

        node = module.CameraNode()
        node.spin()

        self.assertEqual(node.pub_color.messages[-1].header.stamp, 'pair-stamp')
        self.assertEqual(node.pub_depth.messages[-1].header.stamp, 'pair-stamp')
        self.assertEqual(fake_rospy.time_calls, 1)

    def test_real_camera_does_not_sleep_after_blocking_frame_acquisition(self):
        module = load_camera_node()
        fake_rospy = FakeRospy({'fallback_to_simulation': False})
        SinglePairCamera.created = []
        CameraWithDepthScale.created = []
        module.rospy = fake_rospy
        module.CvBridge = FakeBridge
        module.RealSenseManager = SinglePairCamera

        node = module.CameraNode()
        blocking_read = node.cam.read

        def read_one_frame_then_shutdown():
            frame_pair = blocking_read()
            fake_rospy.shutdown = True
            return frame_pair

        node.cam.read = read_one_frame_then_shutdown
        node.spin()

        self.assertEqual(fake_rospy.sleep_count, 0)

    def test_real_camera_limits_ros_publications_without_sleeping(self):
        module = load_camera_node()
        fake_rospy = FakeRospy({
            'fallback_to_simulation': False,
            'fps': 30,
            'publish_fps': 15,
        })
        SinglePairCamera.created = []
        CameraWithDepthScale.created = []
        module.rospy = fake_rospy
        module.CvBridge = FakeBridge
        module.RealSenseManager = SinglePairCamera

        node = module.CameraNode()
        frame_pair = node.cam.read()
        read_count = [0]
        monotonic_times = iter((0.0, 0.01, 0.07))

        def read_three_hardware_frames():
            read_count[0] += 1
            if read_count[0] >= 3:
                fake_rospy.shutdown = True
            return frame_pair

        node.cam.read = read_three_hardware_frames
        node._monotonic = lambda: next(monotonic_times)
        node.spin()

        self.assertEqual(len(node.pub_color.messages), 2)
        self.assertEqual(len(node.pub_depth.messages), 2)
        self.assertEqual(fake_rospy.sleep_count, 0)

    def test_simulated_camera_keeps_rate_sleep(self):
        module = load_camera_node()
        fake_rospy = FakeRospy({
            'simulate': True,
            'fallback_to_simulation': False,
        })
        SinglePairCamera.created = []
        CameraWithDepthScale.created = []
        module.rospy = fake_rospy
        module.CvBridge = FakeBridge
        module.RealSenseManager = SinglePairCamera

        node = module.CameraNode()
        fake_rospy.shutdown_after_stamp = True
        node.spin()

        self.assertEqual(fake_rospy.sleep_count, 1)

    def test_read_timeout_falls_back_without_exiting(self):
        module = load_camera_node()
        fake_rospy = FakeRospy()
        fake_rospy.shutdown_after_stamp = True
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
        fake_rospy.shutdown_after_stamp = True
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
        fake_rospy.shutdown_after_stamp = True
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
