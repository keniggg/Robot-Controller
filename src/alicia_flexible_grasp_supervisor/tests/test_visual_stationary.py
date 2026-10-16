"""Real image evidence, no ROS/SDK/devices and no gate bypass mocks."""
import json

import cv2
import numpy as np
import pytest

from alicia_flexible_grasp.vision.visual_stationary import VisualStationaryWindow


def texture(width=640, height=480):
    image = np.random.RandomState(2718).randint(0, 256, (height, width), dtype=np.uint8)
    return cv2.GaussianBlur(image, (5, 5), 1.)


def source(index):
    return dict(frame_number=100 + index, device_timestamp_ms=1000. + index * 1000. / 30.,
                timestamp_domain='timestamp_domain.hardware_clock', stream_fps=30.)


def observe(window, image, index, **kwargs):
    return window.observe(image, kwargs.get('source', source(index)), kwargs.get('host', 10. + index / 30.))


def shifted(image, x=0., y=0.):
    return cv2.warpAffine(image, np.float32([[1, 0, x], [0, 1, y]]),
                          (image.shape[1], image.shape[0]), borderMode=cv2.BORDER_REFLECT)


def warm(window, image, start=0):
    reports = [observe(window, image, i) for i in range(start, start + 12)]
    assert not any(r['qualified'] for r in reports[:-1])
    assert reports[-1]['qualified']
    return reports[-1]


def test_real_static_texture_qualifies_after_source_window_with_bounded_state():
    window = VisualStationaryWindow()
    report = warm(window, texture())
    assert report['quiet_span_sec'] >= .35
    assert report['motion_p90_px'] < .01
    assert report['covered_quadrants'] == 4
    assert len(window._points) <= 64 and max(window._anchor.shape) <= 320
    json.dumps(report, allow_nan=False)


@pytest.mark.parametrize('amplitude', [.5, 1., 2.])
def test_7_5hz_camera_vibration_never_qualifies_even_when_cycle_endpoints_match(amplitude):
    window = VisualStationaryWindow(); image = texture()
    reports = [observe(window, shifted(image, y=amplitude * np.sin(np.pi * i / 2)), i)
               for i in range(40)]
    assert not any(r['qualified'] for r in reports)
    assert any(r['reason'] == 'visual_motion' for r in reports)


def test_small_incremental_drift_does_not_accumulate_into_stationary_permission():
    window = VisualStationaryWindow(); image = texture()
    assert not any(observe(window, shifted(image, y=.08 * i), i)['qualified'] for i in range(40))


@pytest.mark.parametrize('width,height', [(320, 240), (640, 480), (1280, 960)])
def test_threshold_is_full_resolution_pixels(width, height):
    window = VisualStationaryWindow(); image = texture(width, height)
    warm(window, image)
    report = observe(window, shifted(image, y=.7), 12)
    assert not report['qualified']
    assert report['reason'] == 'visual_motion'
    assert report['motion_p90_px'] > .25


def test_motion_discards_window_and_static_reacquisition_requires_whole_new_window():
    window = VisualStationaryWindow(); image = texture()
    warm(window, image)
    moved = shifted(image, y=1.)
    assert observe(window, moved, 12)['reason'] == 'visual_motion'
    assert not any(observe(window, moved, i)['qualified'] for i in range(13, 23))
    assert observe(window, moved, 23)['qualified']


@pytest.mark.parametrize('mutation', [
    lambda s: s.update(frame_number=200),
    lambda s: s.update(frame_number=111),
    lambda s: s.update(device_timestamp_ms=1000.),
    lambda s: s.update(device_timestamp_ms=2000.),
    lambda s: s.update(timestamp_domain='global_time'),
    lambda s: s.update(stream_fps=60.),
])
def test_source_discontinuities_reset_even_when_pixels_identical(mutation):
    window = VisualStationaryWindow(); image = texture(); warm(window, image)
    info = source(12); mutation(info)
    report = observe(window, image, 12, source=info)
    assert not report['qualified'] and report['reason'] == 'source_discontinuity'


@pytest.mark.parametrize('host', [10.36, 11., float('nan')])
def test_host_pause_or_reversal_resets_with_continuous_device_metadata(host):
    window = VisualStationaryWindow(); image = texture(); warm(window, image)
    assert not observe(window, image, 12, host=host)['qualified']
    assert not observe(window, image, 13)['qualified']


@pytest.mark.parametrize('field,value', [
    ('frame_number', None), ('frame_number', 1.5), ('frame_number', True), ('frame_number', -1),
    ('device_timestamp_ms', float('nan')), ('device_timestamp_ms', float('inf')),
    ('device_timestamp_ms', True), ('stream_fps', 0.), ('stream_fps', 10000.),
    ('timestamp_domain', 'system_time'), ('timestamp_domain', None),
])
def test_invalid_source_revokes_prior_permission(field, value):
    window = VisualStationaryWindow(); image = texture(); warm(window, image)
    info = source(12); info[field] = value
    report = observe(window, image, 12, source=info)
    assert not report['qualified']
    assert window._anchor is None and window._previous is None
    json.dumps(report, allow_nan=False)


@pytest.mark.parametrize('image', [None, np.zeros((0, 0), np.uint8),
    np.zeros((24, 24), np.uint8), np.zeros((50, 50, 4), np.uint8),
    np.zeros((48, 9000), np.uint8), np.full((64, 64), np.nan),
    np.zeros((64, 64), np.float32)])
def test_invalid_image_is_unqualified_without_throwing(image):
    report = observe(VisualStationaryWindow(), image, 0)
    assert not report['qualified'] and report['reason'] == 'invalid_evidence'


def test_textureless_or_one_patch_only_cannot_authorize_depth_history():
    image = np.full((480, 640), 128, np.uint8)
    patch = texture()
    for kind in ('blank', 'patch'):
        if kind == 'patch': image[:200, :260] = patch[:200, :260]
        window = VisualStationaryWindow()
        reports = [observe(window, image, i) for i in range(15)]
        assert not any(r['qualified'] for r in reports)
        assert reports[-1]['reason'] == 'insufficient_texture'


def test_opencv_failure_revokes_permission_without_rejecting_image(monkeypatch):
    window = VisualStationaryWindow(); image = texture(); warm(window, image)
    def fail(*args, **kwargs): raise cv2.error('tracking unavailable')
    monkeypatch.setattr(cv2, 'calcOpticalFlowPyrLK', fail)
    assert not observe(window, image, 12)['qualified']
    assert window._anchor is None


def test_fast_buffered_frames_do_not_substitute_source_duration_for_fresh_wall_window():
    window = VisualStationaryWindow(); image = texture()
    reports = [observe(window, image, i, host=10. + i * .001) for i in range(40)]
    assert not any(r['qualified'] for r in reports)
    assert reports[-1]['quiet_span_sec'] > 1.
    assert reports[-1]['host_quiet_span_sec'] < .04
