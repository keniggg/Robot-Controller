"""Fake-SDK timing and unchanged-pixel checks; no devices, ROS, or network."""
from types import SimpleNamespace

import numpy as np
import pytest

from alicia_flexible_grasp.vision.realsense_manager import RealSenseManager


def fake_manager(slow=None, failure=None):
    manager = RealSenseManager(mode_depth_preset_cfg={'stationary_temporal_enabled': True})
    now = [100.]
    manager._diagnostic_clock = lambda: now[0]
    calls = []
    color = np.arange(18, dtype=np.uint8).reshape(1, 6, 3)
    depth = np.array([[0, 299, 300, 1000, 20000, 20001]], dtype=np.uint16)
    def step(name, value):
        calls.append(name)
        if name == slow:
            now[0] += .35
        if failure and name == failure[0]:
            raise failure[1]
        return value
    color_frame = SimpleNamespace(get_data=lambda: step('color_array', color))
    depth_frame = SimpleNamespace(get_data=lambda: depth)
    frames = SimpleNamespace(get_color_frame=lambda: step('color_frame', color_frame),
                             get_depth_frame=lambda: step('depth_frame', depth_frame))
    manager.pipeline = SimpleNamespace(wait_for_frames=lambda: step('wait_for_frames', frames))
    manager.align = SimpleNamespace(process=lambda f: step('align', f))
    manager.applied_depth_filters = [{'name': 'spatial'}, {'name': 'temporal'}]
    manager.depth_filters = [SimpleNamespace(process=lambda f: step('depth_filter_0_spatial', f)),
                             SimpleNamespace(process=lambda f: step('depth_filter_1_temporal', f))]
    manager.mode_depth_preset = SimpleNamespace(mode='unknown')
    manager.stationary_depth_provider = lambda: step('stationary_evidence', True)
    manager._stationary_temporal = SimpleNamespace(process=lambda f: step('stationary_temporal_process', f))
    manager.color_projection_correction = SimpleNamespace(apply=lambda c: step('color_projection', c.copy()))
    original_clip = manager._clip_depth_range
    manager._clip_depth_range = lambda d: original_clip(step('depth_array_and_clip', d))
    return manager, calls, color, depth, now


@pytest.mark.parametrize('stage', ['wait_for_frames', 'align', 'depth_filter_0_spatial',
                                  'depth_filter_1_temporal', 'stationary_evidence',
                                  'stationary_temporal_process', 'color_array',
                                  'color_projection', 'depth_array_and_clip'])
def test_slow_stage_is_measured_once_with_unchanged_sdk_order_and_pixels(stage):
    manager, calls, original_color, original_depth, now = fake_manager(slow=stage)
    color, depth = manager.read()
    assert calls == ['wait_for_frames', 'color_frame', 'depth_frame',
                     'align', 'color_frame', 'depth_frame',
                     'depth_filter_0_spatial', 'depth_filter_1_temporal',
                     'stationary_evidence', 'stationary_temporal_process',
                     'color_array', 'color_projection', 'depth_array_and_clip']
    np.testing.assert_array_equal(color, original_color)
    np.testing.assert_array_equal(depth, [[0, 0, 300, 1000, 20000, 0]])
    np.testing.assert_array_equal(original_depth, [[0, 299, 300, 1000, 20000, 20001]])
    report = manager.last_read_diagnostics
    assert report['result'] == 'complete' and report['failed_stage'] is None
    assert report['total_sec'] == pytest.approx(.35)
    assert report['stages_sec'][stage] == pytest.approx(.35)
    assert sum(report['stages_sec'].values()) == pytest.approx(.35)
    assert set(manager.runtime_profile) == {'stationary_temporal'}
    assert not any('diagnostic' in key for key in manager.runtime_profile)


@pytest.mark.parametrize('stage', ['wait_for_frames', 'align', 'depth_filter_0_spatial',
                                  'stationary_evidence', 'stationary_temporal_process',
                                  'color_projection', 'depth_array_and_clip'])
def test_exception_identity_and_failed_stage_are_preserved(stage):
    error = RuntimeError('fake backend exception')
    manager, calls, _, _, _ = fake_manager(slow=stage, failure=(stage, error))
    with pytest.raises(RuntimeError) as caught:
        manager.read()
    assert caught.value is error
    assert calls[-1] == stage
    report = manager.last_read_diagnostics
    assert report['result'] == 'error' and report['failed_stage'] == stage
    assert report['stages_sec'][stage] == pytest.approx(.35)
    assert report['total_sec'] == pytest.approx(.35)


def test_each_read_replaces_previous_diagnostics():
    manager, calls, _, _, now = fake_manager()
    manager.read()
    previous = manager.last_read_diagnostics
    manager.read()
    assert manager.last_read_diagnostics is not previous
    assert manager.last_read_diagnostics['total_sec'] == 0.
    assert calls.count('wait_for_frames') == 2


def test_missing_frame_returns_original_none_pair_without_filtering():
    manager, calls, _, _, _ = fake_manager()
    manager.pipeline = SimpleNamespace(wait_for_frames=lambda: SimpleNamespace(
        get_color_frame=lambda: None, get_depth_frame=lambda: object()))
    assert manager.read() == (None, None)
    assert calls == ['align']
    assert manager.last_read_diagnostics['result'] == 'missing_frame'
    assert manager.last_read_diagnostics['failed_stage'] is None


def test_simulation_does_not_touch_real_pipeline():
    manager = RealSenseManager(width=8, height=4, simulate=True)
    color, depth = manager.read()
    assert color.shape == (4, 8, 3) and depth.shape == (4, 8)
    assert set(manager.last_read_diagnostics['stages_sec']) == {'simulate', 'depth_array_and_clip'}
    assert manager.runtime_profile == {}


def telemetry_manager():
    """Raw metadata differs from aligned metadata; no SDK device constructed."""
    manager = RealSenseManager()
    clocks = dict(monotonic=100., wall=1700000000., cpu=2.)
    manager._diagnostic_clock = lambda: clocks['monotonic']
    manager._diagnostic_wall_clock = lambda: clocks['wall']
    manager._diagnostic_thread_clock = lambda: clocks['cpu']
    calls = []
    color_pixels = np.arange(12, dtype=np.uint8).reshape(2, 2, 3)
    depth_pixels = np.full((2, 2), 1000, dtype=np.uint16)
    manager.rs = SimpleNamespace(frame_metadata_value=SimpleNamespace(
        sensor_timestamp='sensor_timestamp', backend_timestamp='backend_timestamp',
        time_of_arrival='time_of_arrival', actual_fps='actual_fps'))
    counter = [10]
    class Frame:
        def __init__(self, stream):
            self.stream = stream
        def get_frame_number(self):
            calls.append(self.stream + '_number')
            return counter[0]
        def get_timestamp(self):
            return 3000. + counter[0] * (1000. / 30.)
        def get_frame_timestamp_domain(self):
            return 'hardware_clock'
        def supports_frame_metadata(self, field):
            calls.append(self.stream + '_supports_' + field)
            return True
        def get_frame_metadata(self, field):
            calls.append(self.stream + '_metadata_' + field)
            return dict(sensor_timestamp=3333333, backend_timestamp=1700000000123,
                        time_of_arrival=1700000000124, actual_fps=30000)[field]
    raw_color, raw_depth = Frame('color'), Frame('depth')
    def raw_frame(stream, frame):
        calls.append('raw_' + stream)
        return frame
    raw = SimpleNamespace(get_color_frame=lambda: raw_frame('color', raw_color),
                          get_depth_frame=lambda: raw_frame('depth', raw_depth))
    def pixel_frame(stream, pixels):
        calls.append('aligned_' + stream)
        return SimpleNamespace(get_data=lambda: pixels)
    aligned = SimpleNamespace(get_color_frame=lambda: pixel_frame('color', color_pixels),
                              get_depth_frame=lambda: pixel_frame('depth', depth_pixels))
    def wait():
        calls.append('wait')
        clocks['monotonic'] += 1.5
        clocks['wall'] += 1.5
        clocks['cpu'] += .003
        return raw
    def align(frames):
        assert frames is raw
        calls.append('align')
        return aligned
    manager.pipeline = SimpleNamespace(wait_for_frames=wait)
    manager.align = SimpleNamespace(process=align)
    return manager, calls, clocks, counter, raw_color, raw_depth, color_pixels, depth_pixels


def test_raw_frame_telemetry_precedes_align_and_captures_wait_not_processing():
    manager, calls, clocks, _, _, _, expected_color, expected_depth = telemetry_manager()
    color, depth = manager.read()
    np.testing.assert_array_equal(color, expected_color)
    np.testing.assert_array_equal(depth, expected_depth)
    report = manager.last_read_diagnostics
    assert report['schema'] == 2
    assert report['host_wait']['enter'] == dict(monotonic_sec=100.,
        wall_time_sec=1700000000., thread_cpu_sec=2.)
    assert report['host_wait']['exit'] == dict(monotonic_sec=101.5,
        wall_time_sec=1700000001.5, thread_cpu_sec=2.003)
    assert report['host_wait']['elapsed_sec'] == pytest.approx(1.5)
    assert report['host_wait']['thread_cpu_elapsed_sec'] == pytest.approx(.003)
    assert report['stages_sec']['wait_for_frames'] == pytest.approx(1.5)
    assert report['stages_sec']['raw_frame_diagnostics'] == 0.
    for stream in ('color', 'depth'):
        raw = report['raw_frames'][stream]
        assert raw['present'] and raw['frame_number'] == 10
        assert raw['device_timestamp_ms'] == pytest.approx(3333.3333333333)
        assert raw['timestamp_domain'] == 'hardware_clock'
        assert raw['metadata_raw'] == dict(sensor_timestamp=3333333,
            backend_timestamp=1700000000123, time_of_arrival=1700000000124, actual_fps=30000)
        assert raw['unavailable'] == {}
    assert calls[0] == 'wait'
    assert calls.index('depth_metadata_actual_fps') < calls.index('align')
    assert calls[-3:] == ['align', 'aligned_color', 'aligned_depth']
    assert manager.runtime_profile == {}


def test_wait_timeout_retains_previous_complete_raw_evidence_and_original_exception():
    import json
    manager, _, clocks, counter, _, _, _, _ = telemetry_manager()
    manager.read()
    first = manager.last_read_diagnostics
    counter[0] += 1
    manager.read()
    second = manager.last_read_diagnostics
    assert second['raw_frames']['color']['frame_number'] == 11
    assert second['previous_completed_raw_frames']['raw_frames']['color']['frame_number'] == 10
    assert 'previous_completed_raw_frames' not in second['previous_completed_raw_frames']
    original = RuntimeError('original wait timeout')
    def fail():
        clocks['monotonic'] += 5.
        clocks['wall'] += 5.
        clocks['cpu'] += .001
        raise original
    manager.pipeline.wait_for_frames = fail
    with pytest.raises(RuntimeError) as caught:
        manager.read()
    assert caught.value is original
    report = manager.last_read_diagnostics
    assert report['result'] == 'error' and report['failed_stage'] == 'wait_for_frames'
    assert report['host_wait']['elapsed_sec'] == 5.
    assert report['host_wait']['thread_cpu_elapsed_sec'] == pytest.approx(.001)
    assert 'raw_frames' not in report
    assert report['previous_completed_raw_frames']['raw_frames']['color']['frame_number'] == 11
    assert first['raw_frames']['color']['frame_number'] == 10
    json.dumps(report, allow_nan=False)


@pytest.mark.parametrize('method', ['get_frame_number', 'get_timestamp',
    'get_frame_timestamp_domain', 'supports_frame_metadata', 'get_frame_metadata'])
def test_optional_frame_method_error_cannot_change_pixels_or_processing(method):
    manager, calls, _, _, raw_color, _, expected_color, expected_depth = telemetry_manager()
    def fail(*args):
        raise RuntimeError('unsupported metadata')
    setattr(raw_color, method, fail)
    color, depth = manager.read()
    np.testing.assert_array_equal(color, expected_color)
    np.testing.assert_array_equal(depth, expected_depth)
    assert manager.last_read_diagnostics['result'] == 'complete'
    assert 'RuntimeError' in manager.last_read_diagnostics['raw_frames']['color']['unavailable'].values()
    assert calls[-3:] == ['align', 'aligned_color', 'aligned_depth']


def test_missing_methods_and_metadata_enums_are_optional_and_json_serializable():
    import json
    manager, _, _, _, _ = fake_manager()
    manager.rs = SimpleNamespace()
    manager.read()
    report = manager.last_read_diagnostics
    assert report['result'] == 'complete'
    assert report['raw_frames']['color']['present'] is True
    assert report['raw_frames']['color']['metadata_raw'] == {}
    assert set(report['raw_frames']['color']['unavailable'].values()) == {'AttributeError'}
    json.dumps(report, allow_nan=False)


def test_unsupported_metadata_is_never_queried():
    manager, calls, _, _, raw_color, _, _, _ = telemetry_manager()
    raw_color.supports_frame_metadata = lambda _: False
    manager.read()
    assert not any(call.startswith('color_metadata_') for call in calls)
    report = manager.last_read_diagnostics['raw_frames']['color']
    assert set(report['unavailable'].values()) == {'unsupported'}
    assert report['metadata_raw'] == {}


def test_nonexistent_raw_frames_do_not_change_aligned_pixel_result():
    manager, calls, _, _, _, _, expected_color, expected_depth = telemetry_manager()
    raw = manager.pipeline.wait_for_frames()
    raw.get_color_frame = lambda: None
    raw.get_depth_frame = lambda: None
    color, depth = manager.read()
    np.testing.assert_array_equal(color, expected_color)
    np.testing.assert_array_equal(depth, expected_depth)
    assert not manager.last_read_diagnostics['raw_frames']['color']['present']
    assert not manager.last_read_diagnostics['raw_frames']['depth']['present']
    assert calls[-3:] == ['align', 'aligned_color', 'aligned_depth']


def test_raw_frame_getter_exception_does_not_replace_processing_exception():
    manager, _, _, _, _, _, expected_color, expected_depth = telemetry_manager()
    raw = manager.pipeline.wait_for_frames()
    def fail():
        raise KeyError('diagnostic extraction failure')
    raw.get_color_frame = fail
    color, depth = manager.read()
    np.testing.assert_array_equal(color, expected_color)
    assert manager.last_read_diagnostics['raw_frames']['color']['unavailable']['frame'] == 'KeyError'
    original = ValueError('align failure')
    def fail_align(_):
        raise original
    manager.align.process = fail_align
    with pytest.raises(ValueError) as caught:
        manager.read()
    assert caught.value is original
    assert manager.last_read_diagnostics['failed_stage'] == 'align'


@pytest.mark.parametrize('clock', ['_diagnostic_wall_clock', '_diagnostic_thread_clock'])
def test_optional_host_clock_failure_does_not_change_read(clock):
    manager, _, _, _, _, _, expected_color, expected_depth = telemetry_manager()
    def fail():
        raise OSError('clock unavailable')
    setattr(manager, clock, fail)
    color, depth = manager.read()
    np.testing.assert_array_equal(color, expected_color)
    np.testing.assert_array_equal(depth, expected_depth)
    assert manager.last_read_diagnostics['result'] == 'complete'
    if clock == '_diagnostic_thread_clock':
        assert manager.last_read_diagnostics['host_wait']['thread_cpu_elapsed_sec'] is None
    else:
        assert manager.last_read_diagnostics['host_wait']['enter']['wall_time_sec'] is None


def test_metadata_collection_fault_is_diagnostic_only():
    manager, _, _, _, _, _, expected_color, expected_depth = telemetry_manager()
    def fail(_):
        raise RuntimeError('unexpected introspection failure')
    manager._raw_frame_diagnostics = fail
    color, depth = manager.read()
    np.testing.assert_array_equal(color, expected_color)
    np.testing.assert_array_equal(depth, expected_depth)
    assert manager.last_read_diagnostics['result'] == 'complete'
    assert manager.last_read_diagnostics['raw_frames_unavailable'] == 'RuntimeError'
    assert manager.last_read_diagnostics['failed_stage'] is None


def test_missing_frame_read_does_not_replace_previous_completed_evidence():
    manager, _, _, _, _, _, _, _ = telemetry_manager()
    manager.read()
    previous = manager._previous_completed_raw_frames
    manager.align.process = lambda f: SimpleNamespace(get_color_frame=lambda: None,
                                                     get_depth_frame=lambda: None)
    assert manager.read() == (None, None)
    assert manager._previous_completed_raw_frames is previous
    assert manager.last_read_diagnostics['result'] == 'missing_frame'


def test_nonfinite_device_timestamp_does_not_escape_into_json():
    import json
    manager, _, _, _, raw_color, _, _, _ = telemetry_manager()
    raw_color.get_timestamp = lambda: float('nan')
    manager.read()
    assert manager.last_read_diagnostics['raw_frames']['color']['unavailable']['device_timestamp_ms'] == 'ValueError'
    json.dumps(manager.last_read_diagnostics, allow_nan=False)
