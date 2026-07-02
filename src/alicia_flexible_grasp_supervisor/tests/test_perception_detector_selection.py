#!/usr/bin/env python3
import importlib.util
import pathlib
import sys
import types
import unittest


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


class FakeHSVDetector:
    created = []

    def __init__(self, *args):
        FakeHSVDetector.created.append(args)


class FakeYOLODetector:
    created = []

    def __init__(self, **kwargs):
        FakeYOLODetector.created.append(kwargs)


class PerceptionDetectorSelectionTest(unittest.TestCase):
    def test_yolov8_config_creates_yolo_detector(self):
        module = load_perception_node()
        module.rospy = FakeRospy({
            'enabled': True,
            'object_label': 'bottle',
            'detector': 'yolov8',
            'yolo_model': 'models/custom.pt',
            'yolo_target_class': 'bottle',
            'yolo_conf': 0.42,
            'yolo_iou': 0.5,
            'yolo_device': 'cpu',
            'yolo_imgsz': 416,
        })
        node = types.SimpleNamespace(
            detector_signature=None,
            stabilizer=module.DetectionStabilizer(),
        )
        FakeYOLODetector.created = []
        module.HSVObjectDetector = FakeHSVDetector
        module.YOLOv8ObjectDetector = FakeYOLODetector

        module.PerceptionNode.refresh_detector(node, force=True)

        self.assertEqual(node.detector_kind, 'yolov8')
        self.assertEqual(FakeYOLODetector.created[0]['model_path'], 'models/custom.pt')
        self.assertEqual(FakeYOLODetector.created[0]['target_class'], 'bottle')
        self.assertAlmostEqual(FakeYOLODetector.created[0]['conf'], 0.42)
        self.assertEqual(FakeYOLODetector.created[0]['imgsz'], 416)

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
