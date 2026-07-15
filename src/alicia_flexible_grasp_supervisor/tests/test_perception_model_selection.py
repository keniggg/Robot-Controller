#!/usr/bin/env python3
import pathlib
import sys
import tempfile
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
SRC_DIR = ROOT / 'src'
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from alicia_flexible_grasp.vision.model_selection import (
    normalize_model_profiles,
    resolve_yolo_model_path,
    select_yolo_model,
)


MODEL_CONFIG = {
    'yolo_models': {
        'original': {
            'display_name': 'YOLOv8 原模型',
            'model_path': 'yolov8n.pt',
            'target_class_mode': 'description',
        },
        'carton': {
            'display_name': 'Carton 模型',
            'model_path': 'carton_model/best.pt',
            'target_class_mode': 'fixed',
            'target_class': 'carton',
        },
    },
}


class PerceptionModelSelectionTest(unittest.TestCase):
    def test_original_model_uses_description_target_class(self):
        selected = select_yolo_model(MODEL_CONFIG, 'original', 'bottle')

        self.assertEqual(selected['model_path'], 'yolov8n.pt')
        self.assertEqual(selected['target_class'], 'bottle')
        self.assertEqual(selected['target_class_mode'], 'description')

    def test_carton_model_always_uses_fixed_carton_class(self):
        selected = select_yolo_model(MODEL_CONFIG, 'carton', 'mouse')

        self.assertEqual(selected['model_path'], 'carton_model/best.pt')
        self.assertEqual(selected['target_class'], 'carton')
        self.assertEqual(selected['target_class_mode'], 'fixed')

    def test_default_profiles_exist_when_yaml_profiles_are_absent(self):
        profiles = normalize_model_profiles({})

        self.assertEqual(list(profiles), ['original', 'carton'])
        self.assertEqual(profiles['carton']['target_class'], 'carton')

    def test_resolves_carton_path_from_catkin_workspace_root(self):
        with tempfile.TemporaryDirectory() as tmp:
            workspace = pathlib.Path(tmp) / 'catkin_ws'
            package = workspace / 'src' / 'alicia_flexible_grasp_supervisor'
            model = workspace / 'carton_model' / 'best.pt'
            package.mkdir(parents=True)
            model.parent.mkdir(parents=True)
            model.write_bytes(b'weights')

            resolved = resolve_yolo_model_path(
                'carton_model/best.pt',
                package_path=str(package),
                cwd=str(pathlib.Path(tmp) / 'other'),
            )

            self.assertEqual(resolved, str(model.resolve()))

    def test_catkin_workspace_model_wins_over_cwd_and_package_decoys(self):
        with tempfile.TemporaryDirectory() as tmp:
            workspace = pathlib.Path(tmp) / 'catkin_ws'
            package = workspace / 'src' / 'alicia_flexible_grasp_supervisor'
            cwd = pathlib.Path(tmp) / 'other'
            workspace_model = workspace / 'carton_model' / 'best.pt'
            package_decoy = package / 'carton_model' / 'best.pt'
            cwd_decoy = cwd / 'carton_model' / 'best.pt'
            for model, contents in (
                (workspace_model, b'workspace weights'),
                (package_decoy, b'package decoy'),
                (cwd_decoy, b'cwd decoy'),
            ):
                model.parent.mkdir(parents=True, exist_ok=True)
                model.write_bytes(contents)

            resolved = resolve_yolo_model_path(
                'carton_model/best.pt',
                package_path=str(package),
                cwd=str(cwd),
            )

            self.assertEqual(resolved, str(workspace_model.resolve()))

    def test_standard_ultralytics_weight_name_remains_unmodified(self):
        self.assertEqual(resolve_yolo_model_path('yolov8n.pt'), 'yolov8n.pt')

    def test_invalid_target_class_mode_is_rejected(self):
        invalid = {
            'yolo_models': {
                'broken': {
                    'display_name': 'Broken',
                    'model_path': 'broken.pt',
                    'target_class_mode': 'automatic',
                },
            },
        }

        with self.assertRaises(ValueError) as ctx:
            normalize_model_profiles(invalid)

        self.assertIn('target_class_mode', str(ctx.exception))

    def test_missing_custom_weight_path_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            package = pathlib.Path(tmp) / 'catkin_ws' / 'src' / 'alicia_flexible_grasp_supervisor'
            package.mkdir(parents=True)

            with self.assertRaises(FileNotFoundError) as ctx:
                resolve_yolo_model_path(
                    'carton_model/best.pt',
                    package_path=str(package),
                    cwd=tmp,
                )

        self.assertIn('carton_model/best.pt', str(ctx.exception))


if __name__ == '__main__':
    unittest.main()
