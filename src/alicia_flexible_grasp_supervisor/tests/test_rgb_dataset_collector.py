#!/usr/bin/env python3
import importlib.util
import tempfile
import unittest
from pathlib import Path


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / 'scripts'
    / 'rgb_dataset_collector_gui.py'
)
LAUNCH_PATH = (
    Path(__file__).resolve().parents[1]
    / 'launch'
    / 'rgb_dataset_collector.launch'
)


def load_collector_module():
    spec = importlib.util.spec_from_file_location(
        'rgb_dataset_collector_gui',
        str(SCRIPT_PATH),
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class RgbDatasetCollectorTest(unittest.TestCase):
    def test_ensure_category_dirs_uses_expected_default_labels(self):
        collector = load_collector_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            paths = collector.ensure_category_dirs(Path(tmpdir))

            self.assertEqual(
                set(paths.keys()),
                {'positive', 'negative', 'low_sample'},
            )
            self.assertEqual(paths['positive'], Path(tmpdir) / 'positive')
            self.assertEqual(paths['negative'], Path(tmpdir) / 'negative')
            self.assertEqual(paths['low_sample'], Path(tmpdir) / 'low_sample')
            for directory in paths.values():
                self.assertTrue(directory.is_dir())

    def test_next_sequence_path_continues_from_largest_numeric_png(self):
        collector = load_collector_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            directory = Path(tmpdir)
            (directory / '000001.png').write_bytes(b'')
            (directory / '000003.png').write_bytes(b'')
            (directory / 'note.png').write_bytes(b'')
            (directory / '000009.jpg').write_bytes(b'')

            self.assertEqual(
                collector.next_sequence_path(directory),
                directory / '000004.png',
            )

    def test_next_sequence_path_starts_at_one_for_empty_directory(self):
        collector = load_collector_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            self.assertEqual(
                collector.next_sequence_path(Path(tmpdir)),
                Path(tmpdir) / '000001.png',
            )

    def test_launch_uses_camera_only_config(self):
        text = LAUNCH_PATH.read_text(encoding='utf-8')

        self.assertIn('rgb_dataset_camera.yaml', text)
        self.assertNotIn('config/camera.yaml', text)


if __name__ == '__main__':
    unittest.main()
