"""Regional history reset and real RGB evidence without ROS or camera access."""
import json
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from alicia_flexible_grasp.vision.regional_stationary_depth import RegionalStationaryDepth


class Frame:
    def __init__(self, pixels, *, profile_id=7, fx=400., declared_shape=None):
        self.pixels = pixels
        height, width = pixels.shape if declared_shape is None else declared_shape
        intrinsics = SimpleNamespace(width=width, height=height, fx=fx, fy=400.,
            ppx=width/2., ppy=height/2., coeffs=[0.]*5, model='none')
        video = SimpleNamespace(get_intrinsics=lambda: intrinsics)
        self.profile = SimpleNamespace(stream_type=lambda: 'depth', stream_index=lambda: 0,
            format=lambda: 'z16', unique_id=lambda: profile_id, fps=lambda: 30,
            as_video_stream_profile=lambda: video)

    def get_data(self):
        return self.pixels

    def get_profile(self):
        return self.profile


class Factory:
    def __init__(self, fault=None):
        self.filters = []
        self.fault = fault

    def __call__(self, **kwargs):
        assert kwargs == dict(alpha=.4, delta=100, persistence=0)
        factory = self

        class Filter:
            def __init__(self):
                self.calls = 0

            def process(self, frame):
                self.calls += 1
                pixels = np.asarray(frame.get_data())
                output = pixels.copy()
                output[output > 0] += 1
                if factory.fault == 'fill_hole':
                    output[pixels == 0] = 123
                elif factory.fault == 'dtype':
                    output = output.astype(np.float32)
                elif factory.fault == 'shape':
                    output = output[:-1]
                elif factory.fault == 'mutate_input':
                    pixels[0, 0] += 1
                return Frame(output)

        created = Filter()
        self.filters.append(created)
        return created


def source(index):
    return dict(frame_number=100+index, device_timestamp_ms=1000.+index*1000./30.,
        timestamp_domain='hardware_clock', stream_fps=30.)


def texture(width=640, height=480):
    gray = np.random.RandomState(2718).randint(0, 256, (height, width), np.uint8)
    gray = cv2.GaussianBlur(gray, (5, 5), 1.)
    return np.repeat(gray[..., None], 3, axis=-1)


def run(engine, rgb, index, *, frame=None, **kwargs):
    frame = Frame(np.full(rgb.shape[:2], 1000, np.uint16)) if frame is None else frame
    return engine.process(frame, rgb, kwargs.get('source', source(index)),
        kwargs.get('host', 10.+index/30.), kwargs.get('encoder_active', True),
        aligned_to_color=kwargs.get('aligned_to_color', True))


def warm(engine, rgb):
    for index in range(12):
        output, report = run(engine, rgb, index)
        assert report['fully_active'] is (index == 11)
        if index < 11:
            assert np.all(output == 1000)
    return output, report


@pytest.fixture(autouse=True)
def bounded_opencv_threads():
    previous = cv2.getNumThreads()
    cv2.setNumThreads(1)
    yield
    cv2.setNumThreads(previous)


def test_real_static_rgb_warms_private_history_before_any_filtered_output():
    factory = Factory(); engine = RegionalStationaryDepth(factory)
    output, report = warm(engine, texture())
    assert len(factory.filters) == 5
    assert [f.calls for f in factory.filters] == [12]*5
    assert np.all(output == 1001)
    assert report['filtered_pixel_count'] == output.size
    assert report['filtered_fraction'] == 1.
    assert not report['whole_frame_stationarity_qualified']
    assert all(r['visual']['quiet_span_sec'] >= .35 for r in report['regions'])
    json.dumps(report, allow_nan=False)


@pytest.mark.parametrize('width,height', [(320, 240), (640, 480), (1280, 960)])
def test_actual_crop_displacement_is_original_pixels_at_every_resolution(width, height):
    factory = Factory(); engine = RegionalStationaryDepth(factory)
    rgb = texture(width, height); warm(engine, rgb)
    moved = cv2.warpAffine(rgb, np.float32([[1, 0, 0], [0, 1, .8]]),
        (width, height), borderMode=cv2.BORDER_REFLECT)
    output, report = run(engine, moved, 12)
    assert not report['active'] and np.all(output == 1000)
    assert all(r['visual']['reason'] == 'visual_motion' for r in report['regions'])
    assert all(r['visual']['motion_p90_px'] > .25 for r in report['regions'])


def test_local_peripheral_motion_resets_its_regions_without_resetting_stationary_center():
    factory = Factory(); engine = RegionalStationaryDepth(factory)
    rgb = texture(); warm(engine, rgb)
    original_filters = [r['filter'] for r in engine._regions]
    moved = rgb.copy()
    moved[:, :128] = np.roll(rgb[:, :128], 6, axis=0)
    output, report = run(engine, moved, 12)
    regions = {r['name']: r for r in report['regions']}
    assert regions['center']['qualified']
    assert not regions['top_left']['qualified'] and not regions['bottom_left']['qualified']
    assert regions['top_right']['qualified'] and regions['bottom_right']['qualified']
    assert np.all(output[48:432, 128:512] == 1001)
    assert np.all(output[:, :128] == 1000)
    assert engine._regions[0]['filter'] is original_filters[0]
    assert engine._regions[2]['filter'] is original_filters[2]
    assert engine._regions[4]['filter'] is original_filters[4]
    assert engine._regions[1]['filter'] is None and engine._regions[3]['filter'] is None
    # Reacquisition has to accumulate the original new visual window.
    for index in range(13, 23):
        output, report = run(engine, moved, index)
        assert np.all(output[:, :128] == 1000)
    _, report = run(engine, moved, 23)
    assert report['fully_active']
    assert len(factory.filters) == 7


@pytest.mark.parametrize('change', ['frame_number', 'timestamp', 'domain', 'fps', 'host_pause', 'host_reverse'])
def test_global_source_discontinuity_resets_all_region_histories(change):
    factory = Factory(); engine = RegionalStationaryDepth(factory); rgb = texture(); warm(engine, rgb)
    old = [r['filter'] for r in engine._regions]
    info = source(12); host = 10.4
    if change == 'frame_number': info['frame_number'] += 2
    elif change == 'timestamp': info['device_timestamp_ms'] += 100.
    elif change == 'domain': info['timestamp_domain'] = 'global_time'
    elif change == 'fps': info['stream_fps'] = 60.
    elif change == 'host_pause': host = 11.
    elif change == 'host_reverse': host = 10.3
    output, report = run(engine, rgb, 12, source=info, host=host)
    assert report['global_history_reset_reason'] == 'sdk_source_discontinuity'
    assert not report['active'] and np.all(output == 1000)
    assert all(r['filter'] is not previous for r, previous in zip(engine._regions, old))


@pytest.mark.parametrize('kind', ['encoder', 'alignment', 'invalid_source', 'invalid_color'])
def test_global_missing_evidence_outputs_current_and_revokes_all_history(kind):
    factory = Factory(); engine = RegionalStationaryDepth(factory); rgb = texture(); warm(engine, rgb)
    kwargs = {}
    if kind == 'encoder': kwargs['encoder_active'] = False
    elif kind == 'alignment': kwargs['aligned_to_color'] = False
    elif kind == 'invalid_source': kwargs['source'] = dict(source(12), device_timestamp_ms=float('nan'))
    elif kind == 'invalid_color': rgb = rgb.astype(np.float32)
    output, report = run(engine, rgb, 12, **kwargs)
    assert not report['active'] and np.all(output == 1000)
    assert all(r['filter'] is None and r['visual']._anchor is None for r in engine._regions)
    json.dumps(report, allow_nan=False)


@pytest.mark.parametrize('change', ['id', 'intrinsics', 'size'])
def test_profile_or_image_geometry_change_starts_new_histories(change):
    factory = Factory(); engine = RegionalStationaryDepth(factory); rgb = texture(); warm(engine, rgb)
    if change == 'size': rgb = texture(320, 240)
    frame = Frame(np.full(rgb.shape[:2], 1000, np.uint16),
        profile_id=8 if change == 'id' else 7, fx=401. if change == 'intrinsics' else 400.)
    output, report = run(engine, rgb, 12, frame=frame)
    assert not report['active'] and np.all(output == 1000)
    assert report['global_history_reset_reason'] in ('sdk_profile_changed', 'image_geometry_changed')
    assert len(factory.filters) == 10


def test_declared_profile_dimensions_must_match_actual_aligned_depth():
    engine = RegionalStationaryDepth(Factory()); rgb = texture(); warm(engine, rgb)
    frame = Frame(np.full(rgb.shape[:2], 1000, np.uint16), declared_shape=(240, 320))
    output, report = run(engine, rgb, 12, frame=frame)
    assert report['reason'] == 'source_or_frame_profile_invalid'
    assert not report['active'] and np.all(output == 1000)
    assert all(r['filter'] is None for r in engine._regions)


def test_alignment_permission_is_never_implied_by_matching_image_shapes():
    engine = RegionalStationaryDepth(Factory()); rgb = texture()
    frame = Frame(np.full(rgb.shape[:2], 1000, np.uint16))
    output, report = engine.process(frame, rgb, source(0), 10., True)
    assert report['reason'] == 'sdk_color_alignment_unproven'
    assert not report['active'] and np.all(output == 1000)


def test_current_holes_remain_zero_and_result_does_not_alias_sdk_input():
    engine = RegionalStationaryDepth(Factory()); rgb = texture(); warm(engine, rgb)
    pixels = np.full(rgb.shape[:2], 1000, np.uint16); pixels[::7, ::9] = 0
    frame = Frame(pixels)
    output, report = run(engine, rgb, 12, frame=frame)
    assert report['fully_active']
    assert np.all(output[pixels == 0] == 0)
    assert not np.shares_memory(output, pixels)
    output[:] = 900
    assert np.all(pixels[pixels != 0] == 1000)


@pytest.mark.parametrize('fault,reason', [('fill_hole','filled_current_depth_holes'),
    ('dtype','output_geometry_changed'),('shape','output_geometry_changed'),('mutate_input','mutated_current_input')])
def test_bad_filter_result_fails_without_leaving_usable_history(fault, reason):
    factory = Factory(fault); engine = RegionalStationaryDepth(factory); rgb = texture()
    pixels = np.full(rgb.shape[:2], 1000, np.uint16); pixels[10, 10] = 0
    with pytest.raises(ValueError, match=reason):
        run(engine, rgb, 0, frame=Frame(pixels))
    assert all(r['filter'] is None and r['visual']._anchor is None for r in engine._regions)


def test_featureless_regions_cannot_gain_permission_from_neighbors():
    engine = RegionalStationaryDepth(Factory()); rgb = texture()
    rgb[48:432, 128:512] = 100
    for index in range(15):
        output, report = run(engine, rgb, index)
        assert np.all(output[48:432, 128:512] == 1000)
        assert not report['regions'][0]['qualified']


def test_external_pair_rejection_reset_requires_whole_new_window():
    factory = Factory(); engine = RegionalStationaryDepth(factory); rgb = texture(); warm(engine, rgb)
    engine.reset()
    for index in range(12, 23):
        output, report = run(engine, rgb, index)
        assert not report['active'] and np.all(output == 1000)
    _, report = run(engine, rgb, 23)
    assert report['fully_active'] and len(factory.filters) == 10
