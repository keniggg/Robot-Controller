"""Offline source-admission tests; importing the recorder never imports ROS."""
import ast
from copy import deepcopy
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest


WORK = Path(__file__).resolve().parents[2]
for package in ('alicia_grasp_modes', 'alicia_flexible_grasp_supervisor'):
    sys.path.insert(0, str(WORK/'src'/package/'src'))
SCRIPT = WORK/'tools/diagnostics/record_rgb_observation.py'
spec = importlib.util.spec_from_file_location('readonly_rgb_record_test', SCRIPT)
recorder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(recorder)


def test_source_rgb_proposals_excludes_padding_without_changing_recorded_depth(monkeypatch):
    rgb = np.full((240, 320, 3), 180, np.uint8); rgb[106:134, 148:173] = 0
    depth_valid = np.ones((240, 320), bool)
    valid = depth_valid.copy(); valid[106:134, 148:173] = False
    monkeypatch.setattr(recorder, 'rectification_rgb_valid_mask', lambda camera, shape: valid)
    original = rgb.copy()
    result = recorder.source_rgb_proposals(rgb, {}, 10, recorder.CenterSearchPolicy((.5, .5)), depth_valid)
    assert result.selected is None and not result.candidates
    assert depth_valid.all()
    np.testing.assert_array_equal(rgb, original)


def test_source_rgb_proposals_rejects_pre_activation_before_image_analysis(monkeypatch):
    from alicia_flexible_grasp.vision.rgbd_rectification import PairedPinholeRectification
    mapper = PairedPinholeRectification(dict(width=32, height=24, fx=40., fy=40.,
        cx=16., cy=12., model='none', coeffs=[0.]*5))
    camera = dict(runtime_profile_geometry=dict(geometry_intrinsics=mapper.output_intrinsics,
        rectification=mapper.contract, geometry_activation_stamp_ns=100))
    monkeypatch.setattr(recorder, 'propose_rgb_observation_regions', lambda *_a, **_k: pytest.fail('old source reached RGB analysis'))
    with pytest.raises(ValueError, match='source_predates_geometry_activation'):
        recorder.source_rgb_proposals(np.zeros((24, 32, 3), np.uint8), camera, 100,
                                      recorder.CenterSearchPolicy((.5, .5)))


def test_filter_telemetry_does_not_reset_source_but_calibration_does():
    camera = dict(fx=400., fy=400., cx=320., cy=240., width=640, height=480,
        depth_scale=.0001, frame_id='camera_link', align_depth_to_color=True,
        temporal_filter_active=True, diagnostics=dict(frame_count=1),
        runtime_profile=dict(geometry_intrinsics=dict(fx=400., coeffs=[.01]),
                             status_stamp_ns=1))
    changed = deepcopy(camera)
    changed['temporal_filter_active'] = False
    changed['diagnostics']['frame_count'] = 99
    changed['runtime_profile']['status_stamp_ns'] = 2
    assert recorder.camera_geometry_snapshot(camera) == recorder.camera_geometry_snapshot(changed)
    changed['runtime_profile']['geometry_intrinsics']['coeffs'] = [.02]
    assert recorder.camera_geometry_snapshot(camera) != recorder.camera_geometry_snapshot(changed)
    frozen = recorder.camera_geometry_snapshot(camera)
    camera['runtime_profile']['geometry_intrinsics']['coeffs'][0] = 100.
    assert frozen['runtime_profile_geometry']['geometry_intrinsics']['coeffs'] == [.01]


def test_feedback_bracket_uses_source_order_not_arrival_order():
    records = [(210_000_000, [2.]*6), (180_000_000, [0.]*6),
               (190_000_000, [1.]*6), (220_000_000, [9.]*6)]
    q, before, after = recorder.bracket_feedback(records, 200_000_000, 100_000_000)
    assert before[0] == 190_000_000 and after[0] == 210_000_000
    np.testing.assert_array_equal(q, [1.5]*6)


@pytest.mark.parametrize('records,stamp,epoch', [
    ([(90, [0.]*6), (110, [1.]*6)], 105, 100),
    ([(10, [0.]*6), (200_000_011, [1.]*6)], 100, 1),
    ([(10, [0.]*6), (10, [1.]*6), (20, [2.]*6)], 15, 1),
    ([(10, [np.nan]*6), (20, [2.]*6)], 15, 1),
    ([(10, [0.]*5), (20, [2.]*6)], 15, 1),
])
def test_invalid_cross_epoch_or_unbound_feedback_rejected(records, stamp, epoch):
    with pytest.raises(ValueError):
        recorder.bracket_feedback(records, stamp, epoch)


def test_exact_feedback_is_its_own_bracket():
    q, before, after = recorder.bracket_feedback([(15, [3.]*6)], 15, 1)
    assert before == after
    np.testing.assert_array_equal(q, [3.]*6)


def test_latest_complete_feedback_pair_wins_without_extrapolation():
    options = [(150_000_000, 'old'), (250_000_000, 'complete'),
               (350_000_000, 'waiting_for_future_feedback')]
    result = recorder.latest_bracketed_pair(options,
        [(100_000_000, [0.]*6), (300_000_000, [2.]*6)], 1)
    assert result[:2] == (250_000_000, 'complete')
    np.testing.assert_array_equal(result[2][0], [1.5]*6)


def test_no_complete_pair_waits_without_reusing_preselection_or_old_feedback():
    assert recorder.latest_bracketed_pair([(150, 'stale')],
        [(100, [0.]*6), (200, [2.]*6)], 160) is None
    assert recorder.latest_bracketed_pair([(150_000_000, 'wide_gap')],
        [(100, [0.]*6), (300_000_000, [2.]*6)], 1) is None


def test_conflicting_feedback_cannot_fall_back_to_different_frame():
    with pytest.raises(ValueError, match='conflicting_accepted_feedback_source'):
        recorder.latest_bracketed_pair([(150, 'one'), (180, 'two')],
            [(100, [0.]*6), (100, [1.]*6), (200, [2.]*6)], 1)


@pytest.mark.parametrize('elapsed,accepted', [(0, True), (800_000_000, True),
    (800_000_001, False), (-1, False)])
def test_existing_source_age_limit_including_nanosecond_boundary(elapsed, accepted):
    if accepted:
        assert recorder.require_fresh_source(1_000_000_000, 1_000_000_000+elapsed) <= .8
    else:
        with pytest.raises(ValueError):
            recorder.require_fresh_source(1_000_000_000, 1_000_000_000+elapsed)


def test_support_quadrants_count_actual_current_ring_pixels():
    depth = np.full((480, 640), .5, dtype=np.float32)
    transform = np.array([[0., -1., 0., 0.], [0., 0., 1., 0.],
                          [-1., 0., 0., .5], [0., 0., 0., 1.]])
    kwargs = dict(intrinsics=[400.,400.,320.,240.], transform=transform,
                  plane=np.array([0.,0.,1.,0.]), config=recorder.Config(), outer=170.)
    full = recorder.measured_support_coverage(depth, **kwargs)
    assert full['spatial_quadrants'] == 4
    depth[:240, :320] = 0.
    three = recorder.measured_support_coverage(depth, **kwargs)
    assert three['spatial_quadrants'] == 3
    assert three['measured_support_pixel_count'] < full['measured_support_pixel_count']
    depth[:240, 320:] = 0.
    assert recorder.measured_support_coverage(depth, **kwargs)['spatial_quadrants'] == 2
    depth[:] = .45  # A measured object in the ring cannot be counted as table.
    assert recorder.measured_support_coverage(depth, **kwargs)['measured_support_pixel_count'] == 0


@pytest.mark.parametrize('field', ['epoch_ns', 'selection', 'camera', 'handeye',
    'tool_camera', 'current_tool_camera', 'model_sha256', 'observation_view_parameters'])
def test_context_change_cannot_reuse_source_identity(field):
    original = {key:'before' for key in ('epoch_ns', 'selection', 'camera', 'handeye',
        'tool_camera', 'current_tool_camera', 'model_sha256', 'observation_view_parameters')}
    changed = dict(original, **{field:'after'})
    assert recorder.source_context_matches(original, deepcopy(original))
    assert not recorder.source_context_matches(original, changed)


def test_reproducibility_metadata_identifies_four_files():
    metadata = recorder.implementation_metadata()
    assert metadata['opencv_version'] and metadata['numpy_version'] and metadata['python_version']
    assert len(metadata['implementation_sha256']) == 4
    assert all(len(v) == 64 for v in metadata['implementation_sha256'].values())


def test_tool_has_no_robot_command_interface():
    tree = ast.parse(SCRIPT.read_text())
    calls = {node.func.attr for node in ast.walk(tree)
             if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)}
    assert not calls.intersection({'Publisher', 'Service', 'ServiceProxy', 'set_param', 'delete_param'})
