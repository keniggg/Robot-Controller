#!/usr/bin/env python3
import pathlib
import sys
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / 'src'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from gui.widgets.perception_widget import PerceptionWidget


class PerceptionVisualAlignmentTest(unittest.TestCase):
    def test_visual_jog_values_map_axes_and_execute_flag(self):
        self.assertEqual(
            PerceptionWidget._visual_jog_values('X+', 0.005, True),
            (0.005, 0.0, 0.0, 0.0, 0.0, 0.0, True),
        )
        self.assertEqual(
            PerceptionWidget._visual_jog_values('Z-', 0.01, False),
            (0.0, 0.0, -0.01, 0.0, 0.0, 0.0, False),
        )

    def test_visual_jog_values_reject_unknown_axis(self):
        with self.assertRaises(ValueError):
            PerceptionWidget._visual_jog_values('Q+', 0.005, True)

    def test_parse_description_maps_common_chinese_objects_to_yolo_class(self):
        parsed = PerceptionWidget._parse_description(PerceptionWidget, '绿色瓶子')

        self.assertEqual(parsed['yolo_target_class'], 'bottle')
        self.assertEqual(parsed['hsv_ranges'][0], ([28, 35, 35], [95, 255, 255]))

    def test_parse_description_leaves_unknown_shape_only_target_to_all_yolo_classes(self):
        parsed = PerceptionWidget._parse_description(PerceptionWidget, '绿色圆形')

        self.assertEqual(parsed['shape'], 'circle')
        self.assertEqual(parsed['yolo_target_class'], '')


if __name__ == '__main__':
    unittest.main()
