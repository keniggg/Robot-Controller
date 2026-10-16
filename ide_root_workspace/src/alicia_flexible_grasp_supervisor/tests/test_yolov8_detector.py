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


class FakeMasks:
    def __init__(self, data):
        self.data = FakeTensor(data)


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

    def test_selected_bbox_uses_mask_from_same_instance(self):
        backend = FakeBackend()
        first = np.zeros((8, 13), dtype=np.float32)
        second = np.zeros((8, 13), dtype=np.float32)
        first[1:4, 1:4] = 1.0
        second[2:7, 7:12] = 1.0
        result = backend.predict(np.zeros((16, 26, 3), dtype=np.uint8))[0]
        result.boxes = FakeBoxes(
            xyxy=[[2, 2, 8, 8], [14, 4, 24, 14]],
            conf=[0.60, 0.82],
            cls=[0, 1],
        )
        result.masks = FakeMasks([first, second])
        backend.predict = lambda image, **kwargs: [result]
        detector = YOLOv8ObjectDetector(
            model_backend=backend,
            target_class='bottle',
            expected_task='segment',
            require_instance_mask=True,
        )

        detection, mask = detector.detect(np.zeros((16, 26, 3), dtype=np.uint8))

        self.assertEqual(detection['class_id'], 1)
        self.assertEqual(mask.shape, (16, 26))
        self.assertEqual(mask.dtype, np.uint8)
        self.assertGreater(np.count_nonzero(mask[:, 14:]), 0)
        self.assertEqual(np.count_nonzero(mask[:, :6]), 0)
        self.assertIs(detection['mask'], mask)
        self.assertEqual(detection['mask_area'], int(np.count_nonzero(mask)))
        ys, xs = np.nonzero(mask)
        self.assertEqual(
            detection['mask_centroid'],
            (int(round(float(np.mean(xs)))), int(round(float(np.mean(ys))))),
        )

    def test_detect_task_ignores_masks_exposed_by_backend(self):
        backend = FakeBackend()
        backend.task = 'detect'
        first = np.zeros((80, 130), dtype=np.float32)
        second = np.zeros((80, 130), dtype=np.float32)
        first[10:50, 5:35] = 1.0
        second[15:65, 70:115] = 1.0
        result = backend.predict(np.zeros((160, 260, 3), dtype=np.uint8))[0]
        result.masks = FakeMasks([first, second])
        backend.predict = lambda image, **kwargs: [result]
        detector = YOLOv8ObjectDetector(
            model_backend=backend,
            target_class='bottle',
            expected_task='detect',
        )

        detection, mask = detector.detect(np.zeros((160, 260, 3), dtype=np.uint8))

        self.assertEqual(detection['label'], 'bottle')
        self.assertIsNone(mask)
        self.assertIsNone(detection['mask'])
        self.assertEqual(detection['mask_area'], 0)
        self.assertIsNone(detection['mask_centroid'])

    def test_required_instance_mask_rejects_missing_mask(self):
        detector = YOLOv8ObjectDetector(
            model_backend=FakeBackend(),
            target_class='bottle',
            expected_task='segment',
            require_instance_mask=True,
        )

        detection, mask = detector.detect(np.zeros((160, 260, 3), dtype=np.uint8))

        self.assertIsNone(detection)
        self.assertIsNone(mask)

    def test_required_instance_mask_rejects_zero_sized_mask_dimensions(self):
        for mask_shape in ((2, 0, 13), (2, 8, 0)):
            with self.subTest(mask_shape=mask_shape):
                backend = FakeBackend()
                result = backend.predict(np.zeros((16, 26, 3), dtype=np.uint8))[0]
                result.masks = FakeMasks(np.zeros(mask_shape, dtype=np.float32))
                backend.predict = lambda image, **kwargs: [result]
                detector = YOLOv8ObjectDetector(
                    model_backend=backend,
                    target_class='bottle',
                    expected_task='segment',
                    require_instance_mask=True,
                )

                detection, mask = detector.detect(np.zeros((16, 26, 3), dtype=np.uint8))

                self.assertIsNone(detection)
                self.assertIsNone(mask)

    def test_required_instance_mask_rejects_non_numeric_mask_data(self):
        backend = FakeBackend()
        result = backend.predict(np.zeros((16, 26, 3), dtype=np.uint8))[0]
        result.masks = FakeMasks(np.full((2, 8, 13), 'invalid'))
        backend.predict = lambda image, **kwargs: [result]
        detector = YOLOv8ObjectDetector(
            model_backend=backend,
            target_class='bottle',
            expected_task='segment',
            require_instance_mask=True,
        )

        detection, mask = detector.detect(np.zeros((16, 26, 3), dtype=np.uint8))

        self.assertIsNone(detection)
        self.assertIsNone(mask)

    def test_required_instance_mask_rejects_non_finite_mask_data(self):
        for label, non_finite in (('nan', np.nan), ('positive_inf', np.inf)):
            with self.subTest(non_finite=label):
                backend = FakeBackend()
                backend.task = 'segment'
                first = np.zeros((80, 130), dtype=np.float32)
                second = np.zeros((80, 130), dtype=np.float32)
                second[15:65, 70:115] = 1.0
                second[20, 75] = non_finite
                result = backend.predict(np.zeros((160, 260, 3), dtype=np.uint8))[0]
                result.masks = FakeMasks([first, second])
                backend.predict = lambda image, **kwargs: [result]
                detector = YOLOv8ObjectDetector(
                    model_backend=backend,
                    target_class='bottle',
                    expected_task='segment',
                    require_instance_mask=True,
                )

                detection, mask = detector.detect(np.zeros((160, 260, 3), dtype=np.uint8))

                self.assertIsNone(detection)
                self.assertIsNone(mask)

    def test_required_instance_mask_rejects_mask_inconsistent_with_bbox(self):
        backend = FakeBackend()
        first = np.zeros((80, 130), dtype=np.float32)
        second = np.zeros((80, 130), dtype=np.float32)
        first[10:50, 5:35] = 1.0
        second[10:50, 5:35] = 1.0
        result = backend.predict(np.zeros((160, 260, 3), dtype=np.uint8))[0]
        result.masks = FakeMasks([first, second])
        backend.predict = lambda image, **kwargs: [result]
        detector = YOLOv8ObjectDetector(
            model_backend=backend,
            target_class='bottle',
            expected_task='segment',
            require_instance_mask=True,
        )

        detection, mask = detector.detect(np.zeros((160, 260, 3), dtype=np.uint8))

        self.assertIsNone(detection)
        self.assertIsNone(mask)

    def test_segment_task_rejects_detect_checkpoint(self):
        backend = FakeBackend()
        backend.task = 'detect'
        with self.assertRaisesRegex(RuntimeError, 'task mismatch'):
            YOLOv8ObjectDetector(
                model_backend=backend,
                expected_task='segment',
                require_instance_mask=True,
            )

    def test_letterboxed_mask_restores_to_original_image_coordinates(self):
        original_shape = (360, 640)
        padded_mask = np.zeros((640, 640), dtype=np.float32)
        padded_mask[240:400, 160:480] = 1.0
        restored = YOLOv8ObjectDetector._restore_mask(padded_mask, original_shape)
        expected = np.zeros(original_shape, dtype=np.float32)
        expected[100:260, 160:480] = 1.0
        np.testing.assert_array_equal(restored, expected)


if __name__ == '__main__':
    unittest.main()
