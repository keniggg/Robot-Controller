#!/usr/bin/env python3
import pathlib
import sys
import types
import unittest

import numpy as np


ROOT = pathlib.Path(__file__).resolve().parents[1]
SRC_DIR = ROOT / 'src'
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from alicia_flexible_grasp.vision.yolov8_detector import YOLOv8ObjectDetector


class FakeTensor:
    def __init__(self, data):
        self.data = data

    def cpu(self):
        return self

    def numpy(self):
        return np.asarray(self.data)

    def tolist(self):
        return list(self.data)


class FakeBoxes:
    def __init__(self, xyxy, conf, cls):
        self.xyxy = FakeTensor(xyxy)
        self.conf = FakeTensor(conf)
        self.cls = FakeTensor(cls)


class FakeResult:
    def __init__(self, names, boxes):
        self.names = names
        self.boxes = boxes


class FakeBackend:
    def __init__(self):
        self.names = {0: 'cup', 1: 'bottle'}
        self.calls = []

    def predict(self, image, conf=0.25, iou=0.45, device='cpu', verbose=False, imgsz=None):
        self.calls.append({
            'shape': image.shape,
            'conf': conf,
            'iou': iou,
            'device': device,
            'verbose': verbose,
            'imgsz': imgsz,
        })
        boxes = FakeBoxes(
            xyxy=[
                [10, 20, 70, 100],
                [140, 30, 230, 130],
            ],
            conf=[0.60, 0.82],
            cls=[0, 1],
        )
        return [FakeResult(self.names, boxes)]


class YOLOv8DetectorTest(unittest.TestCase):
    def test_filters_by_target_class_and_returns_bbox_center(self):
        detector = YOLOv8ObjectDetector(
            model_backend=FakeBackend(),
            target_class='bottle',
            conf=0.35,
            iou=0.40,
            device='cpu',
        )

        det, mask = detector.detect(np.zeros((160, 260, 3), dtype=np.uint8))

        self.assertIsNone(mask)
        self.assertEqual(det['label'], 'bottle')
        self.assertEqual(det['bbox'], (140, 30, 90, 100))
        self.assertEqual((det['u'], det['v']), (185, 80))
        self.assertAlmostEqual(det['confidence'], 0.82)

    def test_predict_receives_configured_image_size(self):
        backend = FakeBackend()
        detector = YOLOv8ObjectDetector(
            model_backend=backend,
            target_class='bottle',
            imgsz=416,
        )

        detector.detect(np.zeros((480, 640, 3), dtype=np.uint8))

        self.assertEqual(backend.calls[0]['imgsz'], 416)

    def test_preferred_point_selects_near_candidate_over_higher_confidence(self):
        detector = YOLOv8ObjectDetector(model_backend=FakeBackend(), target_class='')

        det, _ = detector.detect(
            np.zeros((160, 260, 3), dtype=np.uint8),
            preferred_uv=(40, 60),
            max_preferred_distance_px=80,
        )

        self.assertEqual(det['label'], 'cup')
        self.assertEqual((det['u'], det['v']), (40, 60))

    def test_missing_ultralytics_reports_actionable_error(self):
        with self.assertRaises(RuntimeError) as ctx:
            YOLOv8ObjectDetector(model_loader=lambda _: (_ for _ in ()).throw(ImportError('no ultralytics')))

        self.assertIn('ultralytics', str(ctx.exception))
        self.assertIn('pip3 install', str(ctx.exception))


if __name__ == '__main__':
    unittest.main()
