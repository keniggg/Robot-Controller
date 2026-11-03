"""Offline same-observation board evidence contracts; no ROS/hardware calls."""
import copy
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest


HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE / 'gen4_root_cause')]
SPEC = importlib.util.spec_from_file_location(
    'board_source_capture_under_test', HERE / 'gen4_root_cause' / 'board_source_capture.py')
SOURCE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SOURCE)


class Stamp:
    def __init__(self, ns):
        self.ns = ns

    def to_nsec(self):
        return self.ns


def observation():
    ns = 1791033828489520788
    quality = dict(
        source_stamp_ns=ns, pose_ok=True, camera_model_error=None,
        camera_model_change_latched=False, camera_frame='camera_link',
        board_frame=SOURCE.BOARD_FRAME, intrinsics_source='realsense_runtime',
        board_size=[12, 9], dictionary_id='DICT_5X5_100', square_length_m=.015,
        marker_length_m=.01125, charuco_count=47, edge_clearance_px=140.9,
        reprojection_rms_px=.1062, camera_model_fingerprint='a' * 64)
    transform = SimpleNamespace(
        header=SimpleNamespace(stamp=Stamp(ns), frame_id='camera_link'),
        child_frame_id=SOURCE.BOARD_FRAME)
    return ns, quality, transform


def test_valid_same_source_current_camera_observation_does_not_mutate_inputs():
    ns, quality, transform = observation()
    original = copy.deepcopy(quality)
    SOURCE.require_exact_binding(ns, quality, transform)
    assert quality == original
    assert transform.header.stamp.to_nsec() == ns


@pytest.mark.parametrize('component', ['quality', 'tf'])
@pytest.mark.parametrize('difference_ns', [-1, 1])
def test_nearby_observations_are_not_substituted_for_the_same_source(component, difference_ns):
    ns, quality, transform = observation()
    if component == 'quality':
        quality['source_stamp_ns'] += difference_ns
    else:
        transform.header.stamp = Stamp(ns + difference_ns)
    with pytest.raises(RuntimeError):
        SOURCE.require_exact_binding(ns, quality, transform)


@pytest.mark.parametrize('component', ['tf_camera', 'tf_board', 'quality_camera', 'quality_board'])
def test_matching_timestamp_cannot_hide_frame_provenance_mismatch(component):
    ns, quality, transform = observation()
    if component == 'tf_camera':
        transform.header.frame_id = 'camera_color_optical_frame'
    elif component == 'tf_board':
        transform.child_frame_id = 'previous_board'
    elif component == 'quality_camera':
        quality['camera_frame'] = 'camera_color_optical_frame'
    else:
        quality['board_frame'] = 'previous_board'
    with pytest.raises(RuntimeError):
        SOURCE.require_exact_binding(ns, quality, transform)


@pytest.mark.parametrize('key,value', [
    ('pose_ok', False), ('camera_model_error', 'RUNTIME_CAMERA_MODEL_CHANGED'),
    ('camera_model_change_latched', True), ('intrinsics_source', 'legacy'),
    ('board_size', [11, 8]), ('dictionary_id', 'DICT_4X4_100'),
    ('square_length_m', .0125), ('marker_length_m', .011),
])
def test_same_source_pose_cannot_override_invalid_model_or_wrong_board_contract(key, value):
    ns, quality, transform = observation()
    quality[key] = value
    with pytest.raises(RuntimeError):
        SOURCE.require_exact_binding(ns, quality, transform)


@pytest.mark.parametrize('key,value', [
    ('charuco_count', 34), ('reprojection_rms_px', 1.500001),
    ('reprojection_rms_px', float('nan')), ('reprojection_rms_px', float('inf')),
    ('reprojection_rms_px', None), ('edge_clearance_px', 19.999),
    ('edge_clearance_px', float('nan')), ('edge_clearance_px', float('inf')),
    ('edge_clearance_px', None),
])
def test_insufficient_or_nonfinite_board_quality_is_rejected(key, value):
    ns, quality, transform = observation()
    quality[key] = value
    with pytest.raises(RuntimeError):
        SOURCE.require_exact_binding(ns, quality, transform)


def test_quality_boundary_matches_declared_corner_residual_and_margin_limits():
    ns, quality, transform = observation()
    quality.update(charuco_count=35, reprojection_rms_px=1.5, edge_clearance_px=20.)
    SOURCE.require_exact_binding(ns, quality, transform)


def test_legacy_residual_field_is_not_accepted_as_current_tracker_evidence():
    ns, quality, transform = observation()
    quality['reprojection_error_px'] = quality.pop('reprojection_rms_px')
    with pytest.raises(RuntimeError):
        SOURCE.require_exact_binding(ns, quality, transform)
