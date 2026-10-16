"""Independent compositor permission checks; no devices or ROS are involved.

Visual decisions are controlled here to exercise otherwise hard-to-produce
overlap contradictions. The separate regional tests exercise real RGB motion.
"""
from types import SimpleNamespace

import numpy as np
import pytest

from alicia_flexible_grasp.vision import regional_stationary_depth as regional


class Frame:
    def __init__(self, pixels):
        self.pixels = pixels

    def get_data(self):
        return self.pixels

    def get_profile(self):
        height, width = self.pixels.shape
        intrinsic = SimpleNamespace(width=width, height=height, fx=100., fy=100.,
                                    ppx=width / 2., ppy=height / 2., coeffs=[0.] * 5,
                                    model='none')
        video = SimpleNamespace(get_intrinsics=lambda: intrinsic)
        return SimpleNamespace(stream_type=lambda: 'depth', stream_index=lambda: 0,
                               format=lambda: 'z16', unique_id=lambda: 1, fps=lambda: 30,
                               as_video_stream_profile=lambda: video)


def source(index):
    return dict(frame_number=100 + index,
                device_timestamp_ms=1000. + index * 1000. / 30.,
                timestamp_domain='hardware_clock', stream_fps=30.)


@pytest.fixture
def compositor(monkeypatch):
    decisions = {}
    calls = []
    filters = []

    class VisualDecisions:
        def __init__(self):
            self._anchor = None

        def reset(self):
            self._anchor = None

        def observe(self, color, evidence, host):
            # Unique original-pixel coordinate at each crop's top-left corner.
            crop = tuple(int(x) for x in color[0, 0, :2])
            reason = decisions.get(crop, 'stationary')
            if self._anchor is None or reason == 'visual_motion':
                self._anchor = object()
            return dict(qualified=reason == 'stationary', reason=reason)

    class IdentifiableFilter:
        def __init__(self, serial):
            self.serial = serial
            self.calls = 0

        def process(self, frame):
            self.calls += 1
            original = np.asanyarray(frame.get_data())
            calls.append((self.serial, frame, original.copy()))
            result = original.copy()
            result[result != 0] += 10 * self.serial
            return Frame(result)

    def factory(**options):
        assert options == dict(alpha=.4, delta=100, persistence=0)
        created = IdentifiableFilter(len(filters) + 1)
        filters.append(created)
        return created

    monkeypatch.setattr(regional, 'VisualStationaryWindow', VisualDecisions)
    height, width = 160, 200
    y, x = np.indices((height, width))
    color = np.stack((x, y, np.zeros_like(x)), axis=-1).astype(np.uint8)
    pixels = (1000 + x + 2 * y).astype(np.uint16)
    pixels[::13, ::17] = 0
    center = (x >= 40) & (x < 160) & (y >= 16) & (y < 144)
    return SimpleNamespace(engine=regional.RegionalStationaryDepth(factory),
                           decisions=decisions, calls=calls, filters=filters,
                           color=color, pixels=pixels, center=center)


def process(c, index=0, **kwargs):
    frame = Frame(c.pixels.copy())
    output, report = c.engine.process(frame, c.color, source(index),
                                     10. + index / 30.,
                                     kwargs.get('encoder_active', True),
                                     aligned_to_color=kwargs.get('aligned_to_color', True))
    # No region, including an active one, may modify the source SDK frame.
    np.testing.assert_array_equal(frame.get_data(), c.pixels)
    for _, input_frame, observed in c.calls:
        if input_frame is frame:
            np.testing.assert_array_equal(observed, c.pixels)
    return output, report, frame


def test_center_negative_evidence_overrules_every_qualified_quadrant(compositor):
    c = compositor
    c.decisions[(40, 16)] = 'visual_motion'
    output, _, frame = process(c)
    np.testing.assert_array_equal(output[c.center], c.pixels[c.center])
    valid_outer = ~c.center & (c.pixels != 0)
    assert np.all(output[valid_outer] > c.pixels[valid_outer])
    assert len(c.calls) == 4
    assert all(call[1] is frame for call in c.calls)
    assert not np.any(output[c.pixels == 0])


def test_center_can_filter_without_reusing_failed_peripheral_histories(compositor):
    c = compositor
    for corner in ((0, 0), (100, 0), (0, 80), (100, 80)):
        c.decisions[corner] = 'visual_motion'
    output, _, frame = process(c)
    np.testing.assert_array_equal(output[~c.center], c.pixels[~c.center])
    valid_center = c.center & (c.pixels != 0)
    assert np.all(output[valid_center] > c.pixels[valid_center])
    assert len(c.calls) == 1 and c.calls[0][1] is frame
    assert not np.any(output[c.pixels == 0])


def test_qualified_overlap_uses_one_filter_and_never_a_filter_chain(compositor):
    c = compositor
    output, _, frame = process(c)
    assert len(c.calls) == 5
    assert all(call[1] is frame for call in c.calls)
    # A fixed owner selects one independently filtered result at every pixel.
    # Chaining or averaging histories produces values outside this contract.
    difference = output.astype(np.int32) - c.pixels.astype(np.int32)
    assert set(np.unique(difference[c.pixels != 0])) <= {10, 20, 30, 40, 50}
    assert len(np.unique(difference[c.center & (c.pixels != 0)])) == 1
    assert not np.any(output[c.pixels == 0])


def test_warming_owner_never_inherits_already_qualified_overlap(compositor):
    c = compositor
    c.decisions[(40, 16)] = 'warming_up'
    output, _, _ = process(c)
    np.testing.assert_array_equal(output[c.center], c.pixels[c.center])
    assert len(c.calls) == 5  # Private warmup is permitted, output is not.
