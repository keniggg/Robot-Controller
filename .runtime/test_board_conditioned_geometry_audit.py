"""Synthetic geometry/provenance regressions, not physical grasp validation."""
from copy import deepcopy
import importlib.util
from pathlib import Path

import numpy as np
import pytest

spec = importlib.util.spec_from_file_location('board_audit',
    Path(__file__).with_name('board_conditioned_geometry_audit.py'))
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


def transform(x=0, y=0, z=0, angle=0):
    c, s = np.cos(angle), np.sin(angle)
    result = np.eye(4)
    result[:3, :3] = [[c, 0, s], [0, 1, 0], [-s, 0, c]]
    result[:3, 3] = [x, y, z]
    return result


def sample(stamp, actual_camera, modeled_camera=None, support=None):
    modeled_camera = actual_camera if modeled_camera is None else modeled_camera
    board = transform(.4, .1, .07)
    camera_from_base = np.linalg.inv(actual_camera)
    result = dict(board_reference_id='new-fixed-board-session', target_track_id='motor-1',
        plane_label='observed-table', camera_model_fingerprint='current-sdk-rays',
        handeye_sha256='same-verified-handeye', driver_epoch_ns=1,
        source_stamp_ns=stamp, received_stamp_ns=stamp+100_000_000,
        accepted_stamp_ns=stamp, independent_stationary_window_verified=True,
        camera_frame_convention='ros_camera_link',
        T_camera_board=camera_from_base @ board, T_base_camera_fk=modeled_camera,
        support_plane_camera=audit.transform_plane(camera_from_base,
            [0, 0, 1, -.065] if support is None else support),
        anchor_camera=audit.transform_point(camera_from_base, [.44, .12, .065]))
    result.update({name: stamp for name in audit.STAMP_FIELDS})
    return result


def pair():
    return sample(1_000_000_000, transform(.2, -.1, .4, .3)), sample(
        2_000_000_000, transform(.25, -.05, .3, .45))


def test_rigid_scene_is_invariant_in_both_frames():
    result = audit.audit_pair(*pair())
    for comparison in result['plane_comparisons'].values():
        assert comparison['absolute_local_plane_separation_m'] < 1e-12
    assert result['apparent_base_board_displacement_m'] < 1e-12
    assert result['motion_authorized'] is False


def test_pose_dependent_11mm_fk_error_does_not_move_board_local_plane():
    first, second = pair()
    second['T_base_camera_fk'] = transform(z=.011) @ second['T_base_camera_fk']
    result = audit.audit_pair(first, second)
    assert result['plane_comparisons']['board']['absolute_local_plane_separation_m'] < 1e-12
    assert result['plane_comparisons']['base_fk']['absolute_local_plane_separation_m'] == pytest.approx(.011)
    assert result['apparent_base_board_translation_m'] == pytest.approx([0, 0, .011])


def test_real_plane_movement_remains_visible_in_board_coordinates():
    first, second = pair()
    second = sample(second['source_stamp_ns'], second['T_base_camera_fk'], support=[0, 0, 1, -.076])
    result = audit.audit_pair(first, second)
    for comparison in result['plane_comparisons'].values():
        assert comparison['absolute_local_plane_separation_m'] == pytest.approx(.011)
    assert result['apparent_base_board_displacement_m'] < 1e-12


def test_plane_transform_preserves_signed_distance():
    matrix = transform(.2, -.1, .4, .3)
    original = np.array([0., 0., 1., -.07])
    moved = audit.transform_plane(matrix, original)
    for value in ([0, 0, 0], [.4, .2, .07], [.3, .1, .08]):
        assert moved[:3] @ audit.transform_point(matrix, value)+moved[3] == pytest.approx(
            original[:3] @ value+original[3])


@pytest.mark.parametrize('name', audit.PAIR_FIELDS)
def test_mixed_board_target_model_or_epoch_rejected(name):
    first, second = pair()
    second[name] = 2 if name == 'driver_epoch_ns' else 'changed'
    with pytest.raises(ValueError, match='changed'):
        audit.audit_pair(first, second)


@pytest.mark.parametrize('name', audit.STAMP_FIELDS)
def test_one_nanosecond_source_mismatch_rejected(name):
    first, second = pair()
    second[name] += 1
    with pytest.raises(ValueError, match='exactly'):
        audit.audit_pair(first, second)


@pytest.mark.parametrize('age', [350_000_000, 0])
def test_source_age_boundary_inclusive(age):
    first, second = pair()
    second['received_stamp_ns'] = second['source_stamp_ns']+age
    assert audit.audit_pair(first, second)['motion_authorized'] is False


@pytest.mark.parametrize('age', [350_000_001, -1])
def test_source_age_outside_boundary_rejected(age):
    first, second = pair()
    second['received_stamp_ns'] = second['source_stamp_ns']+age
    with pytest.raises(ValueError, match='reception age'):
        audit.audit_pair(first, second)


@pytest.mark.parametrize('field,value', [
    ('camera_frame_convention', 'optical'),
    ('independent_stationary_window_verified', False),
    ('driver_epoch_ns', True),
    ('support_plane_camera', [0, 0, 2, .1]),
    ('anchor_camera', [0, 0, np.nan]),
    ('T_camera_board', np.diag([1, 1, -1, 1])),
])
def test_malformed_or_wrong_frame_rejected(field, value):
    first, second = pair()
    second[field] = value
    with pytest.raises(ValueError):
        audit.audit_pair(first, second)


def test_reversed_support_normal_not_silently_fixed():
    first, second = pair()
    second['support_plane_camera'] *= -1
    with pytest.raises(ValueError, match='signed orientation'):
        audit.audit_pair(first, second)


def test_input_data_unchanged():
    first, second = pair()
    original = deepcopy(second)
    audit.audit_pair(first, second)
    for name, value in second.items():
        if isinstance(value, np.ndarray):
            np.testing.assert_array_equal(value, original[name])
        else:
            assert value == original[name]
