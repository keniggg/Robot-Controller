from types import SimpleNamespace
import math

import numpy as np
import pytest

from alicia_flexible_grasp.vision.stationary_depth import StationaryDepthWindow
from alicia_flexible_grasp.vision.realsense_manager import RealSenseManager
from test_visual_stationary import texture, source, shifted


def message(kind, sec, positions=None):
    names = StationaryDepthWindow.ARM + (('right_finger',) if kind == 'accepted' else ())
    return SimpleNamespace(header=SimpleNamespace(
        stamp=SimpleNamespace(to_nsec=lambda: int(sec*1e9)),
        frame_id=StationaryDepthWindow.FRAMES[kind]), name=names,
        position=list(positions) if positions is not None else [0.]*len(names))


def window():
    result = StationaryDepthWindow()
    for kind in ('sdk', 'accepted'):
        for sec in (9.5, 9.6, 9.7, 9.8, 9.9, 10.):
            result.observe(kind, message(kind, sec))
    return result


def test_both_real_sources_required_and_fresh():
    w = window()
    assert w.stationary(10.01)
    assert not w.stationary(10.3)
    assert not w.stationary(9.99)
    assert not w.stationary(float('nan'))
    w.history['accepted'].clear()
    assert not w.stationary(10.01)


@pytest.mark.parametrize('kind,values', [
    ('sdk', [.000001]+[0.]*5),
    ('accepted', [.004]+[0.]*6),
    ('accepted', [0.]*6+[.001]),
])
def test_command_arm_or_gripper_motion_resets_stationarity(kind, values):
    w = window()
    w.observe(kind, message(kind, 10.05, values))
    assert not w.stationary(10.06)


def test_encoder_quantization_only_is_allowed():
    w = window()
    w.observe('accepted', message('accepted', 10.05, [2*math.pi/4096]+[0.]*6))
    assert w.stationary(10.06)


@pytest.mark.parametrize('mutation', [
    lambda m: setattr(m.header, 'frame_id', 'heartbeat'),
    lambda m: setattr(m, 'name', ['Joint1']*7),
    lambda m: setattr(m, 'position', [float('nan')]*7),
    lambda m: setattr(m.header.stamp, 'to_nsec', lambda: 9_000_000_000),
])
def test_invalid_callback_revokes_permission(mutation):
    w = window();m = message('accepted', 10.05);mutation(m)
    w.observe('accepted', m)
    assert not w.stationary(10.06)


def test_missing_middle_feedback_is_not_a_stationary_window():
    w = window();w.history['sdk'].clear()
    for sec in (9.5, 9.6, 10.):w.observe('sdk', message('sdk', sec))
    assert not w.stationary(10.01)


class Frame:
    def __init__(self, array):self.array = array
    def get_data(self):return self.array


class Filter:
    def __init__(self):self.options = {};self.fill_holes = False
    def set_option(self, key, value):self.options[key] = value
    def process(self, frame):
        if self.fill_holes:return Frame(np.ones_like(frame.array))
        return Frame(frame.array.copy())


def manager():
    m = RealSenseManager(mode_depth_preset_cfg={'stationary_temporal_enabled': True})
    m.mode_depth_preset = SimpleNamespace(mode='unknown')
    made = []
    def create():
        f = Filter();made.append(f);return f
    m.rs = SimpleNamespace(temporal_filter=create, option=SimpleNamespace(
        filter_smooth_alpha='alpha',filter_smooth_delta='delta',holes_fill='persistence'))
    m.stationary_depth_provider = lambda: True
    m.test_index = 0
    m.test_color = texture()
    return m, made


def feed(m, frame, color=None):
    index = m.test_index
    m.test_index += 1
    m._diagnostic_clock = lambda: 10. + index / 30.
    return m._filter_stationary_depth(frame, color=m.test_color if color is None else color,
                                      source=source(index))


def settle(m, frame):
    for _ in range(12):
        result = feed(m, frame)
    return result


def test_motion_discards_temporal_history_and_reacquisition_starts_new_filter():
    m, made = manager();f = Frame(np.array([[0, 2000]], dtype=np.uint16))
    settle(m, f)
    assert made[0].options == {'alpha': .4, 'delta': 100., 'persistence': 0.}
    m.stationary_depth_provider = lambda: False
    assert feed(m, f) is f
    assert m._stationary_temporal is None
    m.stationary_depth_provider = lambda: True
    settle(m, f)
    assert len(made) == 2


def test_carton_and_missing_motion_evidence_use_current_frame():
    m, made = manager();f = Frame(np.ones((2,2),dtype=np.uint16))
    m.mode_depth_preset.mode = 'carton'
    assert m._filter_stationary_depth(f) is f
    m.mode_depth_preset.mode = 'unknown';m.stationary_depth_provider = None
    assert m._filter_stationary_depth(f) is f
    assert made == []


def test_current_holes_cannot_be_filled_from_previous_frames():
    m, made = manager();f = Frame(np.array([[0, 2000]],dtype=np.uint16))
    settle(m, f);made[0].fill_holes = True
    with pytest.raises(ValueError,match='filled current depth holes'):
        feed(m, f)
    assert m._stationary_temporal is None


def test_missing_rgb_or_frame_identity_bypasses_depth_and_resets_history():
    m, made = manager(); f = Frame(np.array([[0, 2000]], dtype=np.uint16))
    settle(m, f)
    assert m._stationary_temporal is not None
    assert m._filter_stationary_depth(f) is f
    assert m._stationary_temporal is None
    assert feed(m, f) is f
    assert len(made) == 2  # New private warmup; output remains current depth.


def test_encoder_stationary_but_camera_vibrating_uses_unmodified_current_depth():
    m, made = manager(); f = Frame(np.array([[0, 2000]], dtype=np.uint16))
    settle(m, f)
    original = m._stationary_temporal
    for i in range(24):
        image = shifted(m.test_color, y=(1. if i % 2 == 0 else -1.))
        assert feed(m, f, image) is f
        assert m._stationary_temporal is None
    assert len(made) == 1 and original is not None
    settle(m, f)
    assert len(made) == 2


def test_manager_full_read_uses_current_rgb_and_raw_sdk_identity_before_temporal():
    m, made = manager()
    index = [-1]
    m._diagnostic_clock = lambda: 10. + index[0] / 30.
    color = m.test_color
    pixels = np.array([[0, 2000]], dtype=np.uint16)
    class SDKFrame:
        def __init__(self, data): self.data = data
        def get_data(self): return self.data
        def get_frame_number(self): return 100 + index[0]
        def get_timestamp(self): return 1000. + index[0] * 1000. / 30.
        def get_frame_timestamp_domain(self): return 'hardware_clock'
        def get_profile(self): return SimpleNamespace(fps=lambda: 30)
    rgb = SDKFrame(color); depth = SDKFrame(pixels)
    # Fake SDK temporal implementation keeps the real current-depth contract.
    depth.array = pixels
    frames = SimpleNamespace(get_color_frame=lambda: rgb, get_depth_frame=lambda: depth)
    def wait():
        index[0] += 1
        return frames
    m.pipeline = SimpleNamespace(wait_for_frames=wait)
    for i in range(12):
        returned_rgb, returned_depth = m.read()
        np.testing.assert_array_equal(returned_rgb, color)
        np.testing.assert_array_equal(returned_depth, pixels)
        assert m.runtime_profile['stationary_temporal']['active'] == (i == 11)
    assert len(made) == 1
    assert m.last_read_diagnostics['stationary_rgb_evidence']['frame_number'] == 111
    rgb.data = np.zeros_like(color)
    m.read()
    assert m._stationary_temporal is None
    assert not m.runtime_profile['stationary_temporal']['active']
    assert m.last_read_diagnostics['result'] == 'complete'


def test_stop_and_mode_change_reset_both_histories():
    m, made = manager(); f = Frame(np.ones((2, 2), dtype=np.uint16))
    settle(m, f)
    m._stop_pipeline()
    assert m._stationary_temporal is None and m._visual_stationary._anchor is None
    assert not m.runtime_profile['stationary_temporal']['active']
    settle(m, f)
    m.mode_depth_preset.apply = lambda mode: True
    m.mode_depth_preset.evidence = lambda: dict(mode='carton')
    assert m.set_depth_preset_mode('carton')
    assert m._stationary_temporal is None and m._visual_stationary._anchor is None
    assert not m.runtime_profile['stationary_temporal']['active']


def test_saved_baseline_unknown_retains_same_stationary_rules_and_mode_boundaries(tmp_path):
    from test_mode_depth_preset import setup_policy
    m, made = manager()
    policy, sensor, advanced = setup_policy(tmp_path, preset='saved_baseline')
    m.mode_depth_preset = policy
    assert m.set_depth_preset_mode('unknown')
    assert m.runtime_profile['mode_depth_preset']['mode'] == 'unknown'
    assert m.runtime_profile['mode_depth_preset']['preset'] == 'saved_baseline'
    f = Frame(np.array([[0, 2000]], dtype=np.uint16))
    settle(m, f)
    assert m.runtime_profile['stationary_temporal']['active']
    assert made[0].options == {'alpha': .4, 'delta': 100., 'persistence': 0.}
    assert not sensor.writes  # No visual_preset=3, no fake carton task mode.
    m.stationary_depth_provider = lambda: False
    assert feed(m, f) is f and m._stationary_temporal is None
    m.stationary_depth_provider = lambda: True
    settle(m, f)
    assert len(made) == 2
    assert m.set_depth_preset_mode('carton')
    assert m._stationary_temporal is None and m._visual_stationary._anchor is None
    assert feed(m, f) is f
    assert m.set_depth_preset_mode('unknown')
    settle(m, f)
    assert len(made) == 3 and m.runtime_profile['stationary_temporal']['active']
    # The original current-hole prohibition is unchanged in the new preset.
    made[-1].fill_holes = True
    with pytest.raises(ValueError, match='filled current depth holes'):
        feed(m, f)
    assert m._stationary_temporal is None
