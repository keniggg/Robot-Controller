#!/usr/bin/env python3
import json
import pathlib
import sys
import types
import unittest
import warnings
from unittest.mock import patch

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
SRC_DIR = ROOT / 'src'
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from alicia_flexible_grasp.vision.realsense_manager import RealSenseManager


class FakeDepthSensor:
    def get_depth_scale(self):
        return 0.0001


class FakeDevice:
    def get_info(self, key):
        return {'serial_number': 'runtime-test-camera', 'firmware_version': '5.test'}[key]

    def first_depth_sensor(self):
        return FakeDepthSensor()


class FakeVideoStream:
    def __init__(self, stream):
        self.stream = stream

    def as_video_stream_profile(self):
        return self

    def get_intrinsics(self):
        return types.SimpleNamespace(width=4, height=2,
                                     fx=4. if self.stream == 'color' else 3.5,
                                     fy=4. if self.stream == 'color' else 3.6,
                                     ppx=1.5, ppy=.5, model='brown_conrady', coeffs=[0.] * 5)

    def get_extrinsics_to(self, target):
        return types.SimpleNamespace(rotation=[1., 0., 0., 0., 1., 0., 0., 0., 1.],
                                     translation=[.0001, .0002, .0003])


class FakeProfile:
    def get_device(self):
        return FakeDevice()

    def get_stream(self, stream):
        return FakeVideoStream(stream)


class FakeFrame:
    def __init__(self, data):
        self.data = data

    def get_data(self):
        return self.data


class FakeFrames:
    def __init__(self, color, depth):
        self.color = FakeFrame(color)
        self.depth = FakeFrame(depth)

    def get_color_frame(self):
        return self.color

    def get_depth_frame(self):
        return self.depth


class FakePipeline:
    def __init__(self, frames=None):
        self.frames = frames

    def start(self, config):
        return FakeProfile()

    def wait_for_frames(self):
        return self.frames


class FakeConfig:
    def enable_stream(self, *args):
        pass


class FakeAlign:
    def __init__(self, stream, events=None):
        self.stream = stream
        self.events = events

    def process(self, frames):
        if self.events is not None:
            self.events.append('align')
        return frames


class FakeDepthFilter:
    def __init__(self, name='spatial', events=None, fail_option=None):
        self.name = name
        self.events = events
        self.fail_option = fail_option
        self.options = []
        self.process_count = 0

    def set_option(self, option, value):
        if option == self.fail_option:
            raise RuntimeError('%s unsupported' % option)
        self.options.append((option, value))

    def process(self, depth_frame):
        self.process_count += 1
        if self.events is not None:
            self.events.append(self.name)
        return depth_frame


def make_fake_rs(depth=None, spatial=None, events=None):
    color = np.zeros((2, 4, 3), dtype=np.uint8)
    if depth is None:
        depth = np.full((2, 4), 6000, dtype=np.uint16)
    pipeline = FakePipeline(FakeFrames(color, depth))
    if spatial is None:
        spatial = FakeDepthFilter(events=events)
    return types.SimpleNamespace(
        __version__='2.test',
        camera_info=types.SimpleNamespace(serial_number='serial_number', firmware_version='firmware_version'),
        pipeline=lambda: pipeline,
        config=lambda: FakeConfig(),
        stream=types.SimpleNamespace(color='color', depth='depth'),
        format=types.SimpleNamespace(bgr8='bgr8', z16='z16'),
        align=lambda stream: FakeAlign(stream, events=events),
        option=types.SimpleNamespace(
            filter_magnitude='filter_magnitude',
            filter_smooth_alpha='filter_smooth_alpha',
            filter_smooth_delta='filter_smooth_delta',
        ),
        spatial_filter=lambda: spatial,
        temporal_filter=lambda: FakeDepthFilter(name='temporal', events=events),
        hole_filling_filter=lambda: FakeDepthFilter(name='hole_filling', events=events),
        decimation_filter=lambda: (_ for _ in ()).throw(
            AssertionError('decimation must remain disabled')
        ),
    ), spatial


class RealSenseDepthScaleTest(unittest.TestCase):
    @staticmethod
    def _projection_setup():
        sdk = dict(width=4, height=2, fx=4., fy=4., cx=1.5, cy=.5,
                   model='brown_conrady', coeffs=[0., 0., 0., 0., 0.])
        cfg = dict(enabled=True, device_serial='projection-test-camera',
                   sdk_intrinsics=sdk, measured_intrinsics=dict(sdk, fx=3., fy=3.))
        fake_rs, _ = make_fake_rs()
        intrinsic = types.SimpleNamespace(width=4, height=2, fx=4., fy=4., ppx=1.5, ppy=.5,
                                          model='brown_conrady', coeffs=[0.]*5)
        video = types.SimpleNamespace(get_intrinsics=lambda: intrinsic)
        profile = types.SimpleNamespace(
            get_stream=lambda _: types.SimpleNamespace(as_video_stream_profile=lambda: video,
                get_extrinsics_to=lambda _: types.SimpleNamespace(rotation=[1.,0.,0.,0.,1.,0.,0.,0.,1.],
                                                                  translation=[0.,0.,0.])),
            get_device=lambda: types.SimpleNamespace(
                get_info=lambda _: 'projection-test-camera', first_depth_sensor=lambda: FakeDepthSensor()))
        fake_rs.pipeline().start = lambda _: profile
        fake_rs.camera_info = types.SimpleNamespace(serial_number='serial_number', firmware_version='firmware_version')
        fake_rs.pipeline().frames.color.data = np.arange(24,dtype=np.uint8).reshape(2,4,3)*10
        return fake_rs, cfg

    def test_opt_in_projection_corrects_rgb_and_keeps_matching_aligned_depth(self):
        fake_rs, cfg = self._projection_setup()
        raw = fake_rs.pipeline().frames.color.data.copy()
        raw_depth = fake_rs.pipeline().frames.depth.data.copy()
        with patch.dict(sys.modules, {'pyrealsense2': fake_rs}):
            manager = RealSenseManager(width=4,height=2,color_projection_cfg=cfg)
            manager.start()
            color,depth = manager.read()
        self.assertFalse(np.array_equal(color,raw))
        np.testing.assert_array_equal(depth,raw_depth)
        np.testing.assert_array_equal(fake_rs.pipeline().frames.color.data,raw)

    def test_projection_profile_mismatch_does_not_activate_a_partial_mapping(self):
        fake_rs, cfg = self._projection_setup()
        cfg['sdk_intrinsics'] = dict(cfg['sdk_intrinsics'],fx=4.5)
        with patch.dict(sys.modules, {'pyrealsense2': fake_rs}):
            manager = RealSenseManager(width=4,height=2,color_projection_cfg=cfg)
            with self.assertRaisesRegex(RuntimeError,'SDK profile changed: fx'):
                manager.start()
        self.assertIsNone(manager.color_projection_correction)

    def test_projection_cannot_be_applied_to_unaligned_depth(self):
        fake_rs, cfg = self._projection_setup()
        with patch.dict(sys.modules, {'pyrealsense2': fake_rs}):
            manager = RealSenseManager(width=4,height=2,align_depth_to_color=False,color_projection_cfg=cfg)
            with self.assertRaisesRegex(RuntimeError,'requires SDK depth-to-color alignment'):
                manager.start()

    def test_runtime_profile_describes_active_alignment_and_json_safe_device_geometry(self):
        fake_rs, _ = make_fake_rs()
        with patch.dict(sys.modules, {'pyrealsense2': fake_rs}):
            manager = RealSenseManager()
            manager.start()
        profile = json.loads(json.dumps(manager.runtime_profile, allow_nan=False))
        self.assertEqual(profile['device'], dict(serial_number='runtime-test-camera', firmware_version='5.test'))
        self.assertEqual(profile['sdk_version'], '2.test')
        self.assertEqual(profile['depth_scale_source'], 'sdk_depth_sensor')
        self.assertEqual(profile['geometry_intrinsics']['fx'], 4.)
        self.assertEqual(profile['native_depth_intrinsics']['fx'], 3.5)
        self.assertEqual(profile['geometry_intrinsics_source'], 'sdk_active_color_profile')
        self.assertEqual(profile['depth_to_color_extrinsics']['translation_m'], [.0001, .0002, .0003])
        self.assertEqual(profile['depth_filters']['applied'][0]['name'], 'spatial')
        self.assertFalse(profile['color_projection_correction']['enabled'])

    def test_unaligned_profile_explicitly_uses_native_depth_geometry(self):
        fake_rs, _ = make_fake_rs()
        with patch.dict(sys.modules, {'pyrealsense2': fake_rs}):
            manager = RealSenseManager(align_depth_to_color=False)
            manager.start()
        self.assertEqual(manager.runtime_profile['geometry_intrinsics']['fx'], 3.5)
        self.assertEqual(manager.runtime_profile['geometry_intrinsics_source'], 'sdk_active_native_depth_profile')
        self.assertFalse(manager.runtime_profile['alignment']['enabled'])

    def test_failed_filter_is_not_reported_as_applied(self):
        fake_rs, _ = make_fake_rs(spatial=FakeDepthFilter(fail_option='filter_smooth_delta'))
        with patch.dict(sys.modules, {'pyrealsense2': fake_rs}), warnings.catch_warnings():
            warnings.simplefilter('ignore')
            manager = RealSenseManager()
            manager.start()
        self.assertEqual(manager.runtime_profile['depth_filters']['applied'], [])

    def test_corrected_rgb_still_reports_sdk_geometry_and_keeps_measured_candidate_separate(self):
        fake_rs, cfg = self._projection_setup()
        with patch.dict(sys.modules, {'pyrealsense2': fake_rs}):
            manager = RealSenseManager(width=4, height=2, color_projection_cfg=cfg)
            manager.start()
        self.assertEqual(manager.runtime_profile['geometry_intrinsics']['fx'], 4.)
        self.assertEqual(manager.runtime_profile['color_projection_correction']['measured_intrinsics']['fx'], 3.)
        cfg['measured_intrinsics']['fx'] = 99.
        self.assertEqual(manager.runtime_profile['color_projection_correction']['measured_intrinsics']['fx'], 3.)

    def test_depth_scale_read_failure_is_identified_as_fallback_in_runtime_record(self):
        fake_rs, _ = make_fake_rs()
        with patch.dict(sys.modules, {'pyrealsense2': fake_rs}), patch.object(
                FakeDepthSensor, 'get_depth_scale', side_effect=RuntimeError('sensor unavailable')):
            manager = RealSenseManager()
            manager.start()
        self.assertEqual(manager.depth_scale, .0001)
        self.assertEqual(manager.runtime_profile['depth_scale_source'], 'previous_value_fallback')

    def test_start_reads_hardware_depth_scale(self):
        fake_rs, _ = make_fake_rs()
        original = sys.modules.get('pyrealsense2')
        sys.modules['pyrealsense2'] = fake_rs
        try:
            manager = RealSenseManager()
            manager.start()
        finally:
            if original is None:
                sys.modules.pop('pyrealsense2', None)
            else:
                sys.modules['pyrealsense2'] = original

        self.assertEqual(manager.depth_scale, 0.0001)

    def test_default_spatial_filter_options_and_processes_depth_once(self):
        events = []
        fake_rs, spatial = make_fake_rs(events=events)
        original = sys.modules.get('pyrealsense2')
        sys.modules['pyrealsense2'] = fake_rs
        try:
            manager = RealSenseManager()
            manager.start()
            manager.read()
        finally:
            if original is None:
                sys.modules.pop('pyrealsense2', None)
            else:
                sys.modules['pyrealsense2'] = original

        self.assertEqual(spatial.options, [
            ('filter_magnitude', 2),
            ('filter_smooth_alpha', 0.5),
            ('filter_smooth_delta', 20.0),
        ])
        self.assertEqual(spatial.process_count, 1)

    def test_read_aligns_before_applying_optional_depth_filters(self):
        events = []
        fake_rs, _ = make_fake_rs(events=events)
        original = sys.modules.get('pyrealsense2')
        sys.modules['pyrealsense2'] = fake_rs
        try:
            manager = RealSenseManager(depth_filter_cfg={
                'temporal_enabled': True,
                'hole_filling_enabled': True,
            })
            manager.start()
            manager.read()
        finally:
            if original is None:
                sys.modules.pop('pyrealsense2', None)
            else:
                sys.modules['pyrealsense2'] = original

        self.assertEqual(events, ['align', 'spatial', 'temporal', 'hole_filling'])

    def test_unsupported_spatial_option_warns_and_omits_whole_filter(self):
        spatial = FakeDepthFilter(fail_option='filter_smooth_delta')
        fake_rs, _ = make_fake_rs(spatial=spatial)
        original = sys.modules.get('pyrealsense2')
        sys.modules['pyrealsense2'] = fake_rs
        try:
            with warnings.catch_warnings(record=True) as caught:
                warnings.simplefilter('always')
                manager = RealSenseManager()
                manager.start()
                manager.read()
        finally:
            if original is None:
                sys.modules.pop('pyrealsense2', None)
            else:
                sys.modules['pyrealsense2'] = original

        self.assertEqual(manager.depth_filters, [])
        self.assertEqual(spatial.process_count, 0)
        self.assertTrue(any(
            'RealSense depth filter disabled: spatial/filter_smooth_delta: '
            'filter_smooth_delta unsupported' in str(item.message)
            for item in caught
        ))

    def test_unsupported_optional_filter_constructor_warns_and_is_omitted(self):
        fake_rs, _ = make_fake_rs()
        del fake_rs.temporal_filter
        original = sys.modules.get('pyrealsense2')
        sys.modules['pyrealsense2'] = fake_rs
        try:
            with warnings.catch_warnings(record=True) as caught:
                warnings.simplefilter('always')
                manager = RealSenseManager(depth_filter_cfg={
                    'spatial_enabled': False,
                    'temporal_enabled': True,
                })
                manager.start()
        finally:
            if original is None:
                sys.modules.pop('pyrealsense2', None)
            else:
                sys.modules['pyrealsense2'] = original

        self.assertEqual(manager.depth_filters, [])
        self.assertTrue(any(
            'RealSense depth filter disabled: temporal/constructor:' in str(item.message)
            for item in caught
        ))

    def test_depth_range_clipping_preserves_uint16_shape(self):
        depth = np.array([
            [0, 299, 300, 301],
            [19999, 20000, 20001, 65535],
        ], dtype=np.uint16)
        fake_rs, _ = make_fake_rs(depth=depth)
        original = sys.modules.get('pyrealsense2')
        sys.modules['pyrealsense2'] = fake_rs
        try:
            manager = RealSenseManager(depth_filter_cfg={
                'spatial_enabled': False,
                'depth_min_m': 0.03,
                'depth_max_m': 2.0,
            })
            manager.start()
            _, filtered = manager.read()
        finally:
            if original is None:
                sys.modules.pop('pyrealsense2', None)
            else:
                sys.modules['pyrealsense2'] = original

        expected = np.array([
            [0, 0, 300, 301],
            [19999, 20000, 0, 0],
        ], dtype=np.uint16)
        np.testing.assert_array_equal(filtered, expected)
        self.assertEqual(filtered.shape, depth.shape)
        self.assertEqual(filtered.dtype, depth.dtype)

    def test_simulated_depth_uses_d405_scale_without_changing_scene_distance(self):
        manager = RealSenseManager(width=4, height=3, simulate=True)
        manager.start()

        _, depth = manager.read()

        self.assertEqual(manager.depth_scale, 0.0001)
        self.assertAlmostEqual(float(np.median(depth)) * manager.depth_scale, 0.6)

    def test_simulated_depth_is_zeroed_when_outside_configured_range(self):
        manager = RealSenseManager(
            width=4,
            height=3,
            simulate=True,
            depth_filter_cfg={
                'depth_min_m': 0.7,
                'depth_max_m': 1.0,
            },
        )
        manager.start()

        _, depth = manager.read()

        np.testing.assert_array_equal(depth, np.zeros((3, 4), dtype=np.uint16))
        self.assertEqual(depth.shape, (3, 4))
        self.assertEqual(depth.dtype, np.dtype(np.uint16))


if __name__ == '__main__':
    unittest.main()
