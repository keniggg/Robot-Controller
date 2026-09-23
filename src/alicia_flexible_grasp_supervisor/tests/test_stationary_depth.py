from types import SimpleNamespace
import math

import numpy as np
import pytest

from alicia_flexible_grasp.vision.stationary_depth import StationaryDepthWindow
from alicia_flexible_grasp.vision.realsense_manager import RealSenseManager


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
    return m, made


def test_motion_discards_temporal_history_and_reacquisition_starts_new_filter():
    m, made = manager();f = Frame(np.array([[0, 2000]], dtype=np.uint16))
    m._filter_stationary_depth(f)
    assert made[0].options == {'alpha': .4, 'delta': 100., 'persistence': 0.}
    m.stationary_depth_provider = lambda: False
    assert m._filter_stationary_depth(f) is f
    assert m._stationary_temporal is None
    m.stationary_depth_provider = lambda: True
    m._filter_stationary_depth(f)
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
    m._filter_stationary_depth(f);made[0].fill_holes = True
    with pytest.raises(ValueError,match='filled current depth holes'):
        m._filter_stationary_depth(f)
    assert m._stationary_temporal is None
