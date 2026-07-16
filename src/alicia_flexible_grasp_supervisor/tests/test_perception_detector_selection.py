#!/usr/bin/env python3
import importlib.util
import pathlib
import sys
import types
import unittest
from unittest import mock

import numpy as np


ROOT = pathlib.Path(__file__).resolve().parents[1]
PERCEPTION_NODE = ROOT / 'scripts' / 'perception_node.py'
SRC_DIR = ROOT / 'src'
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))


def load_perception_node():
    spec = importlib.util.spec_from_file_location('perception_node_under_test', str(PERCEPTION_NODE))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakeRospy:
    def __init__(self, perception_cfg):
        self.perception_cfg = perception_cfg
        self.warnings = []
        self.infos = []

    def get_param(self, name, default=None):
        if name == '/perception':
            return self.perception_cfg
        return default

    def loginfo(self, *args):
        self.infos.append(args)

    def logwarn_throttle(self, *args):
        self.warnings.append(args)

    def loginfo_throttle(self, *args):
        self.infos.append(args)

    @staticmethod
    def get_time():
        return 10.0


class FakeHSVDetector:
    created = []

    def __init__(self, *args):
        FakeHSVDetector.created.append(args)


class FakeYOLODetector:
    created = []

    def __init__(self, **kwargs):
        FakeYOLODetector.created.append(kwargs)


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(str(getattr(msg, 'data', msg)))


class FakeMessagePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class FakeBridge:
    def cv2_to_imgmsg(self, image, encoding):
        return types.SimpleNamespace(
            data=np.asarray(image).copy(),
            encoding=encoding,
            header=types.SimpleNamespace(stamp=None, frame_id=''),
        )


class FakeSegmentDetector:
    def detect(self, _color, **_kwargs):
        return {
            'u': 10,
            'v': 10,
            'bbox': (5, 5, 10, 10),
            'mask_centroid': (10, 10),
            'label': 'carton',
            'confidence': 0.9,
        }, None


class FakeValidSegmentDetector:
    def detect(self, color, **_kwargs):
        mask = np.zeros(np.asarray(color).shape[:2], dtype=np.uint8)
        mask[5:15, 5:15] = 255
        return {
            'u': 10,
            'v': 10,
            'bbox': (5, 5, 10, 10),
            'mask_centroid': (10, 10),
            'label': 'carton',
            'confidence': 0.9,
        }, mask


class NoFrameBuffer:
    def __init__(self):
        self.polls = 0

    def take_latest(self):
        self.polls += 1
        return None


class PerceptionDetectorSelectionTest(unittest.TestCase):
    def test_yolov8_config_creates_yolo_detector(self):
        module = load_perception_node()
        module.rospy = FakeRospy({
            'enabled': True,
            'object_label': 'bottle',
            'detector': 'yolov8',
            'yolo_model_choice': 'carton',
            'yolo_model': 'models/custom.pt',
            'yolo_target_class': 'bottle',
            'yolo_conf': 0.42,
            'yolo_iou': 0.5,
            'yolo_device': 'cpu',
            'yolo_imgsz': 416,
        })
        status_pub = FakePublisher()
        node = types.SimpleNamespace(
            detector=None,
            detector_signature=None,
            detector_status_pub=status_pub,
            stabilizer=module.DetectionStabilizer(),
        )
        FakeYOLODetector.created = []
        module.HSVObjectDetector = FakeHSVDetector
        module.YOLOv8ObjectDetector = FakeYOLODetector

        with mock.patch.object(module, 'resolve_yolo_model_path', return_value='/resolved/models/custom.pt'):
            module.PerceptionNode.refresh_detector(node, force=True)

        self.assertEqual(node.detector_kind, 'yolov8')
        self.assertEqual(FakeYOLODetector.created[0]['model_path'], '/resolved/models/custom.pt')
        self.assertEqual(FakeYOLODetector.created[0]['target_class'], 'carton')
        self.assertAlmostEqual(FakeYOLODetector.created[0]['conf'], 0.42)
        self.assertEqual(FakeYOLODetector.created[0]['imgsz'], 416)
        self.assertEqual(FakeYOLODetector.created[0]['expected_task'], 'detect')
        self.assertFalse(FakeYOLODetector.created[0]['require_instance_mask'])
        self.assertEqual(status_pub.messages, [
            'loading:carton',
            'ready:carton:/resolved/models/custom.pt',
        ])

    def test_segment_task_without_mask_fails_closed_even_if_profile_flag_is_false(self):
        module = load_perception_node()
        module.rospy = FakeRospy({})
        node = module.PerceptionNode.__new__(module.PerceptionNode)
        node.color = np.zeros((20, 20, 3), dtype=np.uint8)
        node.depth = np.full((20, 20), 2200, dtype=np.uint16)
        node.color_frame_id = 'camera_link'
        node.enabled = True
        node.detector = FakeSegmentDetector()
        node.detector_error = ''
        node.detector_task = 'segment'
        node.require_instance_mask = False
        node.label = 'carton'
        node.bridge = FakeBridge()
        node.pub_mask = FakeMessagePublisher()
        node.pub_obj = FakeMessagePublisher()
        node.pub_detected = FakeMessagePublisher()
        node.pub_raw_detected = FakeMessagePublisher()
        node.stabilizer = module.DetectionStabilizer(hold_seconds=0.8)
        held = module.ObjectPose()
        held.detected = True
        held.u = 2
        held.v = 3
        node.stabilizer.update(held, 9.8)
        node._refresh_camera_params = lambda: None
        node.refresh_detector = lambda: None
        stamp = types.SimpleNamespace(secs=12, nsecs=34)

        module.PerceptionNode.try_detect(node, stamp=stamp, camera_frame='camera_link')

        self.assertEqual(len(node.pub_mask.messages), 1)
        mask_message = node.pub_mask.messages[-1]
        self.assertEqual(mask_message.encoding, 'mono8')
        self.assertIs(mask_message.header.stamp, stamp)
        self.assertEqual(mask_message.header.frame_id, 'camera_link')
        self.assertEqual(np.count_nonzero(mask_message.data), 0)
        self.assertFalse(node.pub_obj.messages[-1].detected)
        self.assertIs(node.pub_obj.messages[-1].header.stamp, stamp)
        self.assertFalse(node.pub_detected.messages[-1].data)
        self.assertFalse(node.pub_raw_detected.messages[-1].data)

    def test_checkpoint_task_mismatch_uses_stable_error_prefix_and_clears_segment_state(self):
        module = load_perception_node()
        module.rospy = FakeRospy({
            'enabled': True,
            'object_label': 'carton',
            'detector': 'yolov8',
            'yolo_model_choice': 'carton_segment',
            'yolo_model': 'carton_segment_model/best.pt',
            'yolo_models': {
                'carton_segment': {
                    'display_name': 'Carton segment',
                    'model_path': 'carton_segment_model/best.pt',
                    'task': 'segment',
                    'target_class_mode': 'fixed',
                    'target_class': 'carton',
                    'require_instance_mask': True,
                },
            },
        })
        node = module.PerceptionNode.__new__(module.PerceptionNode)
        node.color = np.zeros((20, 20, 3), dtype=np.uint8)
        node.color_frame_id = 'camera_link'
        node.bridge = FakeBridge()
        node.pub_mask = FakeMessagePublisher()
        node.pub_obj = FakeMessagePublisher()
        node.pub_detected = FakeMessagePublisher()
        node.pub_raw_detected = FakeMessagePublisher()
        node.detector_status_pub = FakePublisher()
        node.detector = object()
        node.detector_signature = None
        node.stabilizer = module.DetectionStabilizer()
        old = module.ObjectPose()
        old.detected = True
        node.stabilizer.update(old, 9.8)

        class MismatchedYOLODetector:
            def __init__(self, **kwargs):
                self.kwargs = kwargs
                raise RuntimeError('YOLO checkpoint task mismatch: expected segment, got detect')

        module.YOLOv8ObjectDetector = MismatchedYOLODetector
        with mock.patch.object(
            module,
            'resolve_yolo_model_path',
            return_value='/workspace/carton_segment_model/best.pt',
        ):
            module.PerceptionNode.refresh_detector(node, force=True)

        self.assertIsNone(node.detector)
        self.assertEqual(node.detector_task, 'segment')
        self.assertTrue(node.require_instance_mask)
        self.assertTrue(node.detector_error.startswith('MODEL_TASK_MISMATCH'))
        self.assertTrue(
            node.detector_status_pub.messages[-1].startswith(
                'error:carton_segment:MODEL_TASK_MISMATCH'
            )
        )
        self.assertFalse(node.pub_obj.messages[-1].detected)
        self.assertEqual(np.count_nonzero(node.pub_mask.messages[-1].data), 0)
        self.assertIsNone(node.stabilizer.last_detection)

    def test_segment_depth_failure_overwrites_valid_inference_mask_with_zero(self):
        module = load_perception_node()
        module.rospy = FakeRospy({})
        node = module.PerceptionNode.__new__(module.PerceptionNode)
        node.color = np.zeros((20, 20, 3), dtype=np.uint8)
        node.depth = np.zeros((20, 20), dtype=np.uint16)
        node.color_frame_id = 'camera_link'
        node.enabled = True
        node.detector = FakeValidSegmentDetector()
        node.detector_error = ''
        node.detector_task = 'segment'
        node.require_instance_mask = True
        node.label = 'carton'
        node.bridge = FakeBridge()
        node.pub_mask = FakeMessagePublisher()
        node.pub_obj = FakeMessagePublisher()
        node.pub_detected = FakeMessagePublisher()
        node.pub_raw_detected = FakeMessagePublisher()
        node.stabilizer = module.DetectionStabilizer(hold_seconds=0.8)
        node.depth_scale = 0.0001
        node.depth_min_m = 0.03
        node.depth_max_m = 2.0
        node.depth_min_valid_px = 24
        node._last_depth_source = 'none'
        node._last_depth_valid_count = 0
        node._refresh_camera_params = lambda: None
        node.refresh_detector = lambda: None
        stamp = types.SimpleNamespace(secs=56, nsecs=78)

        module.PerceptionNode.try_detect(node, stamp=stamp, camera_frame='camera_link')

        self.assertFalse(node.pub_obj.messages[-1].detected)
        mask_message = node.pub_mask.messages[-1]
        self.assertIs(mask_message.header.stamp, stamp)
        self.assertEqual(mask_message.encoding, 'mono8')
        self.assertEqual(np.count_nonzero(mask_message.data), 0)

    def test_failed_model_switch_clears_old_detector_and_publishes_error(self):
        module = load_perception_node()
        module.rospy = FakeRospy({
            'enabled': True,
            'detector': 'yolov8',
            'yolo_model_choice': 'carton',
            'yolo_model': 'carton_model/best.pt',
            'yolo_target_class': 'carton',
        })
        stabilizer = module.DetectionStabilizer()
        stabilizer.last_detection = object()
        status_pub = FakePublisher()
        node = types.SimpleNamespace(
            detector=object(),
            detector_signature=None,
            detector_status_pub=status_pub,
            stabilizer=stabilizer,
        )

        class BrokenYOLODetector:
            def __init__(self, **kwargs):
                raise RuntimeError('broken weights')

        module.YOLOv8ObjectDetector = BrokenYOLODetector
        with mock.patch.object(module, 'resolve_yolo_model_path', return_value='/workspace/carton_model/best.pt'):
            module.PerceptionNode.refresh_detector(node, force=True)

        self.assertIsNone(node.detector)
        self.assertIsNone(node.stabilizer.last_detection)
        self.assertEqual(status_pub.messages[0], 'loading:carton')
        self.assertEqual(status_pub.messages[1], 'error:carton:broken weights')

    def test_reload_generation_retries_same_selection_after_failed_construction(self):
        module = load_perception_node()
        perception_cfg = {
            'enabled': True,
            'detector': 'yolov8',
            'yolo_model_choice': 'carton',
            'yolo_model': 'carton_model/best.pt',
            'yolo_target_class': 'carton',
            'yolo_reload_generation': 1,
        }
        module.rospy = FakeRospy(perception_cfg)
        status_pub = FakePublisher()
        node = types.SimpleNamespace(
            detector=None,
            detector_signature=None,
            detector_status_pub=status_pub,
            pub_obj=FakeMessagePublisher(),
            pub_detected=FakeMessagePublisher(),
            pub_raw_detected=FakeMessagePublisher(),
            stabilizer=module.DetectionStabilizer(),
        )

        class RetryYOLODetector:
            attempts = 0

            def __init__(self, **kwargs):
                RetryYOLODetector.attempts += 1
                if RetryYOLODetector.attempts == 1:
                    raise RuntimeError('temporary load failure')

        module.YOLOv8ObjectDetector = RetryYOLODetector
        with mock.patch.object(module, 'resolve_yolo_model_path', return_value='/workspace/carton_model/best.pt'):
            module.PerceptionNode.refresh_detector(node, force=True)
            module.PerceptionNode.refresh_detector(node)
            perception_cfg['yolo_reload_generation'] = 2
            module.PerceptionNode.refresh_detector(node)

        self.assertEqual(RetryYOLODetector.attempts, 2)
        self.assertIsInstance(node.detector, RetryYOLODetector)
        self.assertEqual(
            [message for message in status_pub.messages if message.startswith('loading:')],
            ['loading:carton', 'loading:carton'],
        )

    def test_no_frame_poll_observes_reload_and_invalidates_old_downstream_target(self):
        module = load_perception_node()
        perception_cfg = {
            'enabled': True,
            'detector': 'yolov8',
            'object_label': 'carton',
            'yolo_model_choice': 'carton',
            'yolo_model': 'carton_model/best.pt',
            'yolo_target_class': 'carton',
            'yolo_reload_generation': 1,
        }
        module.rospy = FakeRospy(perception_cfg)
        node = module.PerceptionNode.__new__(module.PerceptionNode)
        node.detector = None
        node.detector_signature = None
        node.detector_status_pub = FakePublisher()
        node.pub_obj = FakeMessagePublisher()
        node.pub_detected = FakeMessagePublisher()
        node.pub_raw_detected = FakeMessagePublisher()
        node.stabilizer = module.DetectionStabilizer()
        node.frames = NoFrameBuffer()
        FakeYOLODetector.created = []
        module.YOLOv8ObjectDetector = FakeYOLODetector

        with mock.patch.object(module, 'resolve_yolo_model_path', return_value='/workspace/carton_model/best.pt'):
            module.PerceptionNode.refresh_detector(node, force=True)
            node.detector_status_pub.messages = []
            node.pub_obj.messages = [types.SimpleNamespace(detected=True)]
            node.pub_detected.messages = [types.SimpleNamespace(data=True)]
            node.pub_raw_detected.messages = [types.SimpleNamespace(data=True)]
            node.stabilizer.last_detection = types.SimpleNamespace(detected=True)
            perception_cfg['yolo_reload_generation'] = 2

            processed_frame = module.PerceptionNode._poll_detector_and_process_latest_frame(node)

        self.assertFalse(processed_frame)
        self.assertEqual(node.frames.polls, 1)
        self.assertEqual(len(FakeYOLODetector.created), 2)
        self.assertEqual(node.detector_status_pub.messages, [
            'loading:carton',
            'ready:carton:/workspace/carton_model/best.pt',
        ])
        self.assertFalse(node.pub_obj.messages[-1].detected)
        self.assertFalse(node.pub_detected.messages[-1].data)
        self.assertFalse(node.pub_raw_detected.messages[-1].data)
        self.assertIsNone(node.stabilizer.last_detection)

    def test_detector_reload_publishes_negative_object_and_visibility_before_loading(self):
        module = load_perception_node()
        module.rospy = FakeRospy({
            'enabled': True,
            'detector': 'yolov8',
            'yolo_model_choice': 'carton',
            'yolo_model': 'carton_model/best.pt',
            'yolo_target_class': 'carton',
            'yolo_reload_generation': 1,
        })
        stabilizer = module.DetectionStabilizer()
        stabilizer.last_detection = types.SimpleNamespace(detected=True)
        node = types.SimpleNamespace(
            detector=object(),
            detector_signature=None,
            detector_status_pub=FakePublisher(),
            pub_obj=FakeMessagePublisher(),
            pub_detected=FakeMessagePublisher(),
            pub_raw_detected=FakeMessagePublisher(),
            stabilizer=stabilizer,
        )
        module.YOLOv8ObjectDetector = FakeYOLODetector

        with mock.patch.object(module, 'resolve_yolo_model_path', return_value='/workspace/carton_model/best.pt'):
            module.PerceptionNode.refresh_detector(node, force=True)

        self.assertFalse(node.pub_obj.messages[-1].detected)
        self.assertFalse(node.pub_detected.messages[-1].data)
        self.assertFalse(node.pub_raw_detected.messages[-1].data)
        self.assertIsNone(node.stabilizer.last_detection)

    def test_unavailable_detector_republishes_complete_negative_state(self):
        module = load_perception_node()
        module.rospy = FakeRospy({})
        node = types.SimpleNamespace(
            color=module.np.zeros((2, 2, 3), dtype=module.np.uint8),
            depth=module.np.ones((2, 2), dtype=module.np.uint16),
            enabled=True,
            detector=None,
            detector_error='load failed',
            label='carton',
            pub_obj=FakeMessagePublisher(),
            pub_detected=FakeMessagePublisher(),
            pub_raw_detected=FakeMessagePublisher(),
            stabilizer=module.DetectionStabilizer(),
            _refresh_camera_params=lambda: None,
            refresh_detector=lambda: None,
        )

        module.PerceptionNode.try_detect(node, stamp=None)

        self.assertFalse(node.pub_obj.messages[-1].detected)
        self.assertFalse(node.pub_detected.messages[-1].data)
        self.assertFalse(node.pub_raw_detected.messages[-1].data)

    def test_resolver_exception_directly_publishes_error_without_constructing_detector(self):
        module = load_perception_node()
        module.rospy = FakeRospy({
            'enabled': True,
            'detector': 'yolov8',
            'yolo_model_choice': 'carton',
            'yolo_model': 'carton_model/missing.pt',
            'yolo_target_class': 'carton',
            'yolo_reload_generation': 1,
        })
        status_pub = FakePublisher()
        node = types.SimpleNamespace(
            detector=object(),
            detector_signature=None,
            detector_status_pub=status_pub,
            pub_obj=FakeMessagePublisher(),
            pub_detected=FakeMessagePublisher(),
            pub_raw_detected=FakeMessagePublisher(),
            stabilizer=module.DetectionStabilizer(),
        )
        FakeYOLODetector.created = []
        module.YOLOv8ObjectDetector = FakeYOLODetector

        with mock.patch.object(
            module,
            'resolve_yolo_model_path',
            side_effect=FileNotFoundError('carton_model/missing.pt'),
        ):
            module.PerceptionNode.refresh_detector(node, force=True)

        self.assertIsNone(node.detector)
        self.assertEqual(FakeYOLODetector.created, [])
        self.assertEqual(status_pub.messages[-1], 'error:carton:carton_model/missing.pt')

    def test_hsv_config_still_creates_hsv_detector(self):
        module = load_perception_node()
        module.rospy = FakeRospy({
            'enabled': True,
            'object_label': 'target',
            'detector': 'simple_hsv',
            'hsv_lower': [28, 35, 35],
            'hsv_upper': [95, 255, 255],
            'shape': 'circle',
            'min_area': 300,
        })
        node = types.SimpleNamespace(
            detector_signature=None,
            stabilizer=module.DetectionStabilizer(),
        )
        FakeHSVDetector.created = []
        module.HSVObjectDetector = FakeHSVDetector
        module.YOLOv8ObjectDetector = FakeYOLODetector

        module.PerceptionNode.refresh_detector(node, force=True)

        self.assertEqual(node.detector_kind, 'simple_hsv')
        self.assertEqual(len(FakeHSVDetector.created), 1)


if __name__ == '__main__':
    unittest.main()
