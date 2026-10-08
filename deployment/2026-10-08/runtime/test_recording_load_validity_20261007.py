import importlib.util
from pathlib import Path
import unittest

s = importlib.util.spec_from_file_location('load_validity', Path(__file__).with_name('compare_recording_load_20261007.py'))
m = importlib.util.module_from_spec(s)
s.loader.exec_module(m)


class RecordingValidityTests(unittest.TestCase):
    def outcome(self, **changes):
        v = dict(bag_closed=True, interrupted=False, stop_reason='duration_elapsed', keyframe_count=90,
                 started_at='2026-10-07T00:00:00+00:00', finished_at='2026-10-07T00:00:45+00:00')
        v.update(changes)
        return v

    def test_closed_nonempty_full_window_is_valid_only_for_measured_load(self):
        r = m.validate_recording_load(0, self.outcome(), 900000)
        self.assertTrue(r['valid'])
        self.assertEqual(r['duration_seconds'], 45)
        self.assertIn('not proof of full-rate', r['scope'])

    def test_failed_child_or_unclosed_bag_is_invalid(self):
        for rc, outcome in [(1,self.outcome()), (None,self.outcome()), (0,self.outcome(bag_closed=False))]:
            with self.subTest(rc=rc, outcome=outcome):
                self.assertFalse(m.validate_recording_load(rc,outcome,900)['valid'])

    def test_zero_images_metadata_only_is_not_full_recording_load(self):
        self.assertIn('no_actual_keyframe_image_load',m.validate_recording_load(0,self.outcome(keyframe_count=0),9000)['reasons'])

    def test_early_stop_missing_duration_and_signal_do_not_qualify(self):
        for changes in [dict(finished_at='2026-10-07T00:00:04+00:00'),dict(finished_at=None),dict(interrupted=True),dict(stop_reason='signal_SIGINT')]:
            with self.subTest(changes=changes):
                self.assertFalse(m.validate_recording_load(0,self.outcome(**changes),900)['valid'])


if __name__ == '__main__':
    unittest.main()
