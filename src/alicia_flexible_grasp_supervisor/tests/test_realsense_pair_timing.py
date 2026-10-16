"""RGB-D temporal integrity using real stall values; no camera/ROS/network."""
import json
from types import SimpleNamespace

import numpy as np
import pytest

from test_realsense_read_diagnostics import telemetry_manager


def timed_pair(color_ms, depth_ms, color_fps=30, depth_fps=30, domain='global_time'):
    manager, calls, clocks, counter, color, depth, expected_color, expected_depth = telemetry_manager()
    for frame, stamp, fps in ((color, color_ms, color_fps), (depth, depth_ms, depth_fps)):
        frame.get_timestamp = lambda stamp=stamp: stamp
        frame.get_frame_timestamp_domain = lambda: 'timestamp_domain.' + domain
        frame.get_profile = lambda fps=fps: SimpleNamespace(fps=lambda: fps)
    return manager, calls, color, depth, expected_color, expected_depth


def test_real_433ms_arrival_mismatch_rejected_before_align_or_pixels():
    # camera_transport_20260929T140917Z, wait exit 1790690988.1581905.
    # Raw arrival times differ 433ms; SDK global timestamps differ 436.794ms.
    manager, calls, color, depth, _, _ = timed_pair(
        1790690987815.4678, 1790690988252.2615)
    for frame, arrival, number in ((color, 1790690987724, 627615),
                                    (depth, 1790690988157, 628036)):
        get_metadata = frame.get_frame_metadata
        frame.get_frame_metadata = lambda field, arrival=arrival, original=get_metadata: (
            arrival if field == 'time_of_arrival' else original(field))
        frame.get_frame_number = lambda number=number: number
    assert manager.read() == (None, None)
    report = manager.last_read_diagnostics
    timing = report['pair_timing']
    assert report['result'] == 'rejected_pair' and report['failed_stage'] is None
    assert timing['reason'] == 'timestamp_skew_exceeded'
    assert timing['skew_ms'] == pytest.approx(436.7937, abs=.001)
    assert timing['max_skew_ms'] == pytest.approx(1000. / 30. / 2.)
    assert report['raw_frames']['color']['frame_number'] == 627615
    assert report['raw_frames']['depth']['metadata_raw']['time_of_arrival'] == 1790690988157
    assert 'align' not in calls and 'aligned_color' not in calls
    assert manager._previous_completed_raw_frames is None
    json.dumps(report, allow_nan=False)


def test_real_normal_pair_preserves_pixels_despite_common_future_clock_offset():
    # The 2.225s stall recovery pair is valid: only 0.3064ms inter-stream skew.
    # Both SDK global timestamps are ~72ms ahead of host receipt, so no absolute
    # source-age test or ROS exposure-time conversion is justified here.
    manager, calls, _, _, expected_color, expected_depth = timed_pair(
        1790690986230.4019, 1790690986230.0955)
    manager._diagnostic_wall_clock = lambda: 1790690986.1579337
    actual_color, actual_depth = manager.read()
    np.testing.assert_array_equal(actual_color, expected_color)
    np.testing.assert_array_equal(actual_depth, expected_depth)
    timing = manager.last_read_diagnostics['pair_timing']
    assert timing['accepted'] and timing['skew_ms'] == pytest.approx(.3064, abs=.001)
    assert 'align' in calls


@pytest.mark.parametrize('color_fps,depth_fps,skew_ms,accepted', [
    (30, 30, 16., True), (30, 30, 17., False),
    (60, 60, 8., True), (60, 60, 9., False),
    (30, 15, 33., True), (30, 15, 34., False),
    (15, 30, 33., True), (15, 30, 34., False),
    (6, 6, 83., True), (6, 6, 84., False),
])
def test_tolerance_uses_actual_frame_profiles_not_requested_manager_fps(
        color_fps, depth_fps, skew_ms, accepted):
    manager, _, _, _, _, _ = timed_pair(1000., 1000. + skew_ms, color_fps, depth_fps)
    manager.fps = 120  # Requested/configured FPS is deliberately different.
    result = manager.read()
    timing = manager.last_read_diagnostics['pair_timing']
    assert timing['accepted'] is accepted
    assert timing['max_skew_ms'] == pytest.approx(500. / min(color_fps, depth_fps))
    if not accepted:
        assert result == (None, None)


@pytest.mark.parametrize('domain', ['hardware_clock', 'global_time'])
def test_both_known_measurement_clock_domains_allow_a_close_pair(domain):
    manager, _, _, _, _, _ = timed_pair(1000., 1000.3, domain=domain)
    manager.read()
    assert manager.last_read_diagnostics['pair_timing']['accepted']


@pytest.mark.parametrize('domain', ['system_time', 'unknown', '', 'None'])
def test_close_host_fallback_or_unknown_clock_is_not_measurement_sync(domain):
    manager, calls, _, _, _, _ = timed_pair(1000., 1000., domain=domain)
    assert manager.read() == (None, None)
    assert manager.last_read_diagnostics['pair_timing']['reason'] == 'color_timestamp_domain_unusable'
    assert 'align' not in calls


def test_mixed_timestamp_domains_reject_without_comparing_numeric_values():
    manager, _, _, depth, _, _ = timed_pair(1000., 1000.)
    depth.get_frame_timestamp_domain = lambda: 'timestamp_domain.hardware_clock'
    assert manager.read() == (None, None)
    timing = manager.last_read_diagnostics['pair_timing']
    assert timing['reason'] == 'timestamp_domain_mismatch'
    assert 'skew_ms' not in timing


@pytest.mark.parametrize('stream', ['color', 'depth'])
@pytest.mark.parametrize('method,reason', [
    ('get_timestamp', 'timestamp_unavailable'),
    ('get_frame_timestamp_domain', 'timestamp_domain_unusable'),
    ('get_profile', 'stream_fps_unavailable'),
])
def test_unavailable_required_timing_rejects_with_exact_reason(stream, method, reason):
    manager, calls, color, depth, _, _ = timed_pair(1000., 1000.)
    def fail():
        raise RuntimeError('unavailable timing')
    setattr(color if stream == 'color' else depth, method, fail)
    assert manager.read() == (None, None)
    report = manager.last_read_diagnostics
    assert report['pair_timing']['reason'] == stream + '_' + reason
    assert report['failed_stage'] is None
    assert 'align' not in calls
    json.dumps(report, allow_nan=False)


@pytest.mark.parametrize('value', [float('nan'), float('inf'), -1.])
def test_nonfinite_or_negative_timestamp_is_rejected_and_json_is_finite(value):
    manager, _, _, _, _, _ = timed_pair(value, 1000.)
    assert manager.read() == (None, None)
    assert manager.last_read_diagnostics['pair_timing']['reason'] == 'color_timestamp_unavailable'
    json.dumps(manager.last_read_diagnostics, allow_nan=False)


@pytest.mark.parametrize('fps', [0., -30., float('nan'), float('inf')])
def test_invalid_active_fps_never_uses_requested_fps_as_a_fallback(fps):
    manager, _, _, _, _, _ = timed_pair(1000., 1000., depth_fps=fps)
    assert manager.read() == (None, None)
    assert manager.last_read_diagnostics['pair_timing']['reason'] == 'depth_stream_fps_unavailable'
    json.dumps(manager.last_read_diagnostics, allow_nan=False)


def test_rejected_pair_does_not_replace_last_good_evidence_and_next_good_pair_recovers():
    manager, _, _, depth, expected_color, expected_depth = timed_pair(1000., 1000.3)
    manager.read()
    previous = manager._previous_completed_raw_frames
    depth.get_timestamp = lambda: 1433.
    assert manager.read() == (None, None)
    assert manager._previous_completed_raw_frames is previous
    assert manager.last_read_diagnostics['previous_completed_raw_frames'] is previous
    depth.get_timestamp = lambda: 1000.3
    color_pixels, depth_pixels = manager.read()
    np.testing.assert_array_equal(color_pixels, expected_color)
    np.testing.assert_array_equal(depth_pixels, expected_depth)
    assert manager.last_read_diagnostics['result'] == 'complete'
