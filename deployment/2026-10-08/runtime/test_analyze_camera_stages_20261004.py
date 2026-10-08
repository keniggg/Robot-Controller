"""Offline coordinate and measurement-boundary checks, no device interfaces."""
import numpy as np
import pytest

import analyze_camera_stages_20261004 as analysis


def model():
    return dict(width=8, height=8, fx=20., fy=20., ppx=3.5, ppy=3.5,
                model='distortion.brown_conrady', coeffs=[0., 0., 0., 0., 0.])


def test_native_footprint_uses_extrinsic_inverse_not_color_indices():
    info = model()
    mask = np.zeros((8, 8), bool); mask[3:5, 4:6] = True
    translation = np.array([.01, 0., 0.])
    result, uv = analysis.projected_native_footprint(mask, np.repeat(.2, 4),
        analysis.rays(info), analysis.intrinsics(info), np.eye(3), translation)
    expected = np.zeros((8, 8), bool); expected[3:5, 3:5] = True
    assert np.array_equal(result, expected)
    # One-pixel movement follows t/z; changing the assumed depth moves the
    # footprint, without changing or inventing any raw measurements.
    _, far_uv = analysis.projected_native_footprint(mask, np.repeat(.4, 4),
        analysis.rays(info), analysis.intrinsics(info), np.eye(3), translation)
    assert np.allclose(far_uv[:, 0]-uv[:, 0], .5)


def test_native_rotation_roundtrip_recovers_measured_point():
    info = model(); intr = analysis.intrinsics(info)
    angle = .12
    rotation = np.array([[np.cos(angle), 0., np.sin(angle)], [0., 1., 0.],
                         [-np.sin(angle), 0., np.cos(angle)]])
    translation = np.array([.001, -.002, .0004])
    point = np.array([.01, -.005, .3])
    color = point@rotation.T+translation
    reconstructed = (color-translation)@rotation
    assert np.allclose(point, reconstructed)
    assert np.allclose(analysis.project(point[None], intr), analysis.project(reconstructed[None], intr))


def test_scalar_alignment_cannot_create_measurements_without_raw_input():
    info = model(); raw = np.zeros((8, 8), np.uint16)
    selected = np.ones_like(raw, bool)
    assert not np.any(analysis.raw_point_raster(raw, .0001, info,
        analysis.intrinsics(info), np.eye(3), np.zeros(3), selected))
    raw[3, 3] = 2000
    result = analysis.raw_point_raster(raw, .0001, info,
        analysis.intrinsics(info), np.eye(3), np.zeros(3), selected)
    assert np.all(result[result > 0] == 2000)
    assert np.count_nonzero(result) <= 4


def test_depth_delta_distinguishes_missing_and_changed_samples():
    before = np.array([[0, 1000], [1200, 1300]], np.uint16)
    after = np.array([[0, 1005], [0, 1300]], np.uint16)
    result = analysis.delta(before, after, np.ones((2, 2), bool), .0001)
    assert result['lost_positive_to_zero'] == 1
    assert result['filled_zero_to_positive'] == 0
    assert result['changed_positive'] == 1
    assert result['signed_change_mm']['max'] == pytest.approx(.5)
