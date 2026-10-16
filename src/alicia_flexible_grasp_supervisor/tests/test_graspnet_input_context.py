#!/usr/bin/env python3
import pathlib
import sys
from types import SimpleNamespace

import numpy as np
import pytest


ROOT = pathlib.Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / 'src'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from alicia_flexible_grasp.vision.graspnet_input_context import (
    CONTEXT_ROI,
    FULL_SCENE,
    MASKED_TARGET,
    GraspNetInputContextError,
    build_graspnet_input_context,
)


def _normalized(vector):
    vector = np.asarray(vector, dtype=float)
    return vector / np.linalg.norm(vector)


def _render_scene(
    height=120,
    width=160,
    target_box=(60, 43, 40, 30),
):
    scale = 0.001
    fx = 205.0
    fy = 198.0
    cx = 79.0
    cy = 58.0
    intrinsics = SimpleNamespace(
        width=width,
        height=height,
        fx=fx,
        fy=fy,
        cx=cx,
        cy=cy,
        depth_scale=scale,
    )

    # The plane is deliberately tilted in camera_link coordinates.  This
    # catches implementations that compare an optical-frame point directly to
    # the camera_link plane coefficients.
    plane_normal = _normalized([-0.58, 0.17, 0.80])
    plane_point = np.asarray([0.80, 0.0, 0.0], dtype=float)
    vv, uu = np.indices((height, width), dtype=np.float64)
    optical_ray_x = (uu - cx) / fx
    optical_ray_y = (vv - cy) / fy
    camera_link_rays = np.stack(
        [
            np.ones_like(optical_ray_x),
            -optical_ray_x,
            -optical_ray_y,
        ],
        axis=-1,
    )
    numerator = float(np.dot(plane_normal, plane_point))
    denominator = camera_link_rays @ plane_normal
    plane_z_m = numerator / denominator
    assert np.all(np.isfinite(plane_z_m))
    assert np.all(plane_z_m > 0.0)
    full_depth = np.rint(plane_z_m / scale).astype(np.uint16)

    x, y, box_width, box_height = target_box
    mask = np.zeros((height, width), dtype=np.uint8)
    mask[y:y + box_height, x:x + box_width] = 255
    target_depth = np.zeros_like(full_depth)
    target_depth_m = np.maximum(plane_z_m - 0.060, 0.05)
    target_depth[mask > 0] = np.rint(target_depth_m[mask > 0] / scale).astype(
        np.uint16
    )
    full_depth[mask > 0] = target_depth[mask > 0]

    color = np.zeros((height, width, 3), dtype=np.uint8)
    color[:, :, 0] = 11
    color[:, :, 1] = 97
    color[:, :, 2] = 203
    return SimpleNamespace(
        target_depth=target_depth,
        full_depth=full_depth,
        mask=mask,
        color=color,
        intrinsics=intrinsics,
        plane_point=plane_point,
        plane_normal=plane_normal,
        target_box=target_box,
    )


@pytest.fixture
def scene():
    return _render_scene()


def _build(scene, mode=CONTEXT_ROI, **overrides):
    values = {
        'mode': mode,
        'target_depth_raw': scene.target_depth,
        'object_mask': scene.mask,
        'full_depth_raw': scene.full_depth,
        'color_bgr': scene.color,
        'intrinsics': scene.intrinsics,
        'support_plane_point_camera': scene.plane_point,
        'support_plane_normal_camera': scene.plane_normal,
    }
    values.update(overrides)
    return build_graspnet_input_context(**values)


def _assert_code(expected_code, call):
    with pytest.raises(GraspNetInputContextError) as error:
        call()
    assert error.value.code == expected_code
    assert str(error.value).startswith(expected_code + ':')
    return error.value


def test_masked_target_depth_is_byte_identical_and_outputs_are_read_only(scene):
    result = _build(scene, MASKED_TARGET)

    assert result.mode == MASKED_TARGET
    assert not result.diagnostic_only
    assert result.depth_raw.dtype == scene.target_depth.dtype
    assert result.depth_raw.tobytes() == scene.target_depth.tobytes()
    assert not result.depth_raw.flags.writeable
    assert not result.color_bgr.flags.writeable
    assert result.audit.target_points == int(np.count_nonzero(scene.target_depth))
    assert result.audit.support_points == 0
    assert result.audit.target_fraction == pytest.approx(1.0)


def test_context_roi_keeps_target_and_plane_but_excludes_raised_clutter(scene):
    clutter = (slice(24, 32), slice(39, 47))
    assert not np.any(scene.mask[clutter])
    full_depth = scene.full_depth.copy()
    full_depth[clutter] = np.maximum(full_depth[clutter] - 55, 1)

    result = _build(scene, full_depth_raw=full_depth)

    np.testing.assert_array_equal(
        result.depth_raw[scene.mask > 0],
        scene.target_depth[scene.mask > 0],
    )
    assert not np.any(result.depth_raw[clutter])
    assert result.depth_raw[34, 48] == full_depth[34, 48]
    assert result.depth_raw[0, 0] == 0
    assert result.audit.support_points >= 200
    assert result.audit.excluded_off_plane_points >= 64
    assert result.audit.padding_px == 24
    assert result.audit.context_margin_px == pytest.approx(24.0)
    assert result.audit.context_expand_ratio == pytest.approx(0.30)
    assert result.audit.context_max_margin_px == pytest.approx(64.0)
    assert result.audit.guard_px == 2
    assert result.audit.plane_distance_threshold_m == pytest.approx(0.006)


def test_fused_target_hole_may_be_absent_from_full_depth_and_is_audited(scene):
    full_depth = scene.full_depth.copy()
    pixel = tuple(np.argwhere(scene.target_depth > 0)[0])
    full_depth[pixel] = 0

    result = _build(scene, full_depth_raw=full_depth)

    assert result.depth_raw[pixel] == scene.target_depth[pixel]
    assert result.audit.target_hole_filled_points == 1
    assert result.audit.target_points == int(np.count_nonzero(scene.target_depth))


def test_context_plane_distance_gate_keeps_five_mm_and_rejects_seven_mm(scene):
    kept_pixel = (30, 45)
    rejected_pixel = (31, 45)
    assert scene.mask[kept_pixel] == 0
    assert scene.mask[rejected_pixel] == 0
    full_depth = scene.full_depth.copy()

    # Find raw-depth perturbations whose true perpendicular distances land on
    # opposite sides of the 6 mm gate for this inclined plane.
    def perpendicular_distance(pixel, raw_value):
        v, u = pixel
        z = float(raw_value) * scene.intrinsics.depth_scale
        optical_x = (float(u) - scene.intrinsics.cx) * z / scene.intrinsics.fx
        optical_y = (float(v) - scene.intrinsics.cy) * z / scene.intrinsics.fy
        point = np.asarray([z, -optical_x, -optical_y])
        return abs(float(np.dot(point - scene.plane_point, scene.plane_normal)))

    for pixel, lower, upper in (
        (kept_pixel, 0.0040, 0.0058),
        (rejected_pixel, 0.0062, 0.0085),
    ):
        original = int(full_depth[pixel])
        choices = range(max(31, original - 40), original + 41)
        selected = next(
            value
            for value in choices
            if lower <= perpendicular_distance(pixel, value) <= upper
        )
        full_depth[pixel] = selected

    result = _build(scene, full_depth_raw=full_depth)

    assert result.depth_raw[kept_pixel] == full_depth[kept_pixel]
    assert result.depth_raw[rejected_pixel] == 0


def test_two_pixel_guard_never_consumes_target_and_removes_nearby_plane(scene):
    result = _build(scene)
    x, y, width, height = scene.target_box

    assert np.all(result.depth_raw[y:y + height, x:x + width] > 0)
    assert not np.any(result.depth_raw[y - 2:y, x:x + width])
    assert not np.any(result.depth_raw[y + height:y + height + 2, x:x + width])
    assert not np.any(result.depth_raw[y:y + height, x - 2:x])
    assert not np.any(result.depth_raw[y:y + height, x + width:x + width + 2])
    assert result.audit.excluded_guard_points > 0


def test_context_roi_is_clamped_to_image_boundary_and_uses_minimum_pad():
    edge = _render_scene(target_box=(0, 1, 20, 18))
    result = _build(
        edge,
        min_target_points=100,
        min_support_points=50,
        min_total_points=150,
        min_target_fraction=0.10,
    )

    assert result.audit.bbox_xyxy == (0, 1, 20, 19)
    assert result.audit.padding_px == 24
    assert result.audit.roi_xyxy == (0, 0, 44, 43)


def test_context_padding_uses_ceiling_and_is_clamped_to_64():
    ceiling = _render_scene(
        height=260,
        width=320,
        target_box=(70, 80, 101, 80),
    )
    ceiling_result = _build(ceiling)
    assert ceiling_result.audit.padding_px == 31

    maximum = _render_scene(
        height=360,
        width=420,
        target_box=(90, 100, 240, 100),
    )
    maximum_result = _build(maximum)
    assert maximum_result.audit.padding_px == 64


def test_context_padding_parameters_are_explicit_and_override_defaults(scene):
    result = _build(
        scene,
        context_margin_px=0.0,
        context_expand_ratio=0.301,
        context_max_margin_px=20.0,
    )

    assert result.audit.padding_px == 13
    assert result.audit.roi_xyxy == (47, 30, 113, 86)
    assert result.audit.context_margin_px == pytest.approx(0.0)
    assert result.audit.context_expand_ratio == pytest.approx(0.301)
    assert result.audit.context_max_margin_px == pytest.approx(20.0)


def test_zero_expand_ratio_and_zero_guard_are_valid_explicit_choices(scene):
    result = _build(
        scene,
        context_margin_px=4.0,
        context_expand_ratio=0.0,
        context_max_margin_px=4.0,
        target_guard_px=0,
        min_support_points=1,
    )

    assert result.audit.padding_px == 4
    assert result.audit.context_expand_ratio == pytest.approx(0.0)
    assert result.audit.guard_px == 0
    assert result.audit.support_points > 0


def test_zero_context_extent_cannot_bypass_positive_support_gate(scene):
    error = _assert_code(
        'SUPPORT_POINTS_INSUFFICIENT',
        lambda: _build(
            scene,
            context_margin_px=0.0,
            context_expand_ratio=0.0,
            context_max_margin_px=0.0,
            target_guard_px=0,
            min_support_points=1,
        ),
    )

    assert error.audit['support_points'] == 0
    assert error.audit['min_support_points'] == 1


def test_rgb_is_nonzero_at_exactly_the_valid_depth_pixels(scene):
    result = _build(scene)
    valid = result.depth_raw > 0

    np.testing.assert_array_equal(result.color_bgr[valid], scene.color[valid])
    assert np.all(result.color_bgr[~valid] == 0)
    assert np.array_equal(np.any(result.color_bgr != 0, axis=2), valid)


def test_full_scene_is_explicitly_diagnostic_only(scene):
    result = _build(scene, FULL_SCENE)

    assert result.mode == FULL_SCENE
    assert result.diagnostic_only
    assert result.audit.diagnostic_only
    assert result.depth_raw.tobytes() == scene.full_depth.tobytes()
    assert result.audit.total_points == int(np.count_nonzero(scene.full_depth))
    assert result.audit.valid_full_scene_points == result.audit.total_points
    assert result.audit.target_fraction < result.audit.min_target_fraction


def test_detected_bbox_consistent_with_mask_is_recorded(scene):
    detected = (55, 38, 50, 40)
    result = _build(scene, detected_bbox_xywh=detected)

    assert result.audit.detected_bbox_xywh == detected
    assert result.audit.detected_bbox_mask_iou == pytest.approx(0.60)


def test_detected_bbox_mask_mismatch_fails_closed(scene):
    error = _assert_code(
        'BBOX_MISMATCH',
        lambda: _build(scene, detected_bbox_xywh=(0, 0, 20, 20)),
    )

    assert error.audit['detected_bbox_xywh'] == (0, 0, 20, 20)
    assert error.audit['detected_bbox_mask_iou'] == pytest.approx(0.0)


def test_unknown_mode_never_falls_back(scene):
    _assert_code('MODE_INVALID', lambda: _build(scene, 'automatic'))


def test_empty_mask_fails_closed(scene):
    _assert_code(
        'MASK_EMPTY',
        lambda: _build(
            scene,
            object_mask=np.zeros_like(scene.mask),
            target_depth_raw=np.zeros_like(scene.target_depth),
        ),
    )


@pytest.mark.parametrize(
    'overrides',
    [
        {'object_mask': np.zeros((4, 5), dtype=np.uint8)},
        {'full_depth_raw': np.zeros((4, 5), dtype=np.uint16)},
        {'color_bgr': np.zeros((120, 160, 4), dtype=np.uint8)},
        {
            'intrinsics': SimpleNamespace(
                width=159,
                height=120,
                fx=205.0,
                fy=198.0,
                cx=79.0,
                cy=58.0,
                depth_scale=0.001,
            )
        },
    ],
)
def test_shape_contract_failures_are_structured(scene, overrides):
    expected = (
        'INTRINSICS_INVALID'
        if 'intrinsics' in overrides
        else 'SHAPE_INVALID'
    )
    _assert_code(expected, lambda: _build(scene, **overrides))


@pytest.mark.parametrize(
    'overrides',
    [
        {'target_depth_raw': np.zeros((120, 160), dtype=np.float32)},
        {'object_mask': np.zeros((120, 160), dtype=np.int16)},
        {'full_depth_raw': np.zeros((120, 160), dtype=np.uint32)},
        {'color_bgr': np.zeros((120, 160, 3), dtype=np.float32)},
    ],
)
def test_dtype_contract_failures_are_structured(scene, overrides):
    _assert_code('DTYPE_INVALID', lambda: _build(scene, **overrides))


@pytest.mark.parametrize(
    'point,normal',
    [
        (None, None),
        ([0.8, 0.0], [0.0, 0.0, 1.0]),
        ([0.8, 0.0, 0.0], [0.0, 0.0, 0.0]),
        ([0.8, 0.0, 0.0], [0.0, np.nan, 1.0]),
    ],
)
def test_invalid_support_plane_fails_without_masked_fallback(scene, point, normal):
    _assert_code(
        'SUPPORT_PLANE_INVALID',
        lambda: _build(
            scene,
            support_plane_point_camera=point,
            support_plane_normal_camera=normal,
        ),
    )


def test_out_of_range_nonzero_depth_fails_instead_of_being_silently_filtered(scene):
    target = scene.target_depth.copy()
    full = scene.full_depth.copy()
    y, x = np.argwhere(target > 0)[0]
    target[y, x] = 1
    full[y, x] = 1

    _assert_code(
        'DEPTH_RANGE_INVALID',
        lambda: _build(scene, target_depth_raw=target, full_depth_raw=full),
    )


def test_snapshot_depth_disagreement_fails_closed(scene):
    full = scene.full_depth.copy()
    y, x = np.argwhere(scene.target_depth > 0)[0]
    full[y, x] += 1

    _assert_code(
        'SNAPSHOT_INCONSISTENT',
        lambda: _build(scene, full_depth_raw=full),
    )


@pytest.mark.parametrize(
    'overrides',
    [
        {'context_margin_px': -1.0},
        {'context_expand_ratio': -0.01},
        {'context_max_margin_px': -1.0},
        {'context_margin_px': 65.0, 'context_max_margin_px': 64.0},
        {'context_margin_px': float('nan')},
        {'context_expand_ratio': float('inf')},
        {'context_max_margin_px': True},
        {'detected_bbox_min_iou': True},
        {'depth_min_m': np.bool_(True)},
    ],
)
def test_invalid_or_boolean_numeric_configuration_is_rejected(scene, overrides):
    _assert_code('CONFIG_INVALID', lambda: _build(scene, **overrides))


@pytest.mark.parametrize(
    'parameter',
    ('min_target_points', 'min_support_points', 'min_total_points'),
)
@pytest.mark.parametrize(
    'invalid_value',
    (0, -1, 1.5, float('nan'), float('inf'), float('-inf'), True, np.bool_(False)),
)
def test_quality_gate_counts_must_be_positive_finite_integers(
    scene,
    parameter,
    invalid_value,
):
    error = _assert_code(
        'CONFIG_INVALID',
        lambda: _build(scene, **{parameter: invalid_value}),
    )

    assert parameter in error.reason


@pytest.mark.parametrize(
    'parameter',
    ('min_target_fraction', 'detected_bbox_min_iou'),
)
@pytest.mark.parametrize(
    'invalid_value',
    (
        0.0,
        -0.01,
        1.000001,
        float('nan'),
        float('inf'),
        float('-inf'),
        True,
        np.bool_(False),
    ),
)
def test_fraction_and_bbox_iou_thresholds_must_be_in_open_closed_unit_interval(
    scene,
    parameter,
    invalid_value,
):
    error = _assert_code(
        'CONFIG_INVALID',
        lambda: _build(scene, **{parameter: invalid_value}),
    )

    assert parameter in error.reason


def test_quality_gate_lower_and_upper_boundaries_are_accepted(scene):
    result = _build(
        scene,
        MASKED_TARGET,
        detected_bbox_xywh=scene.target_box,
        detected_bbox_min_iou=1.0,
        min_target_points=1,
        min_support_points=1,
        min_total_points=1,
        min_target_fraction=1.0,
    )

    assert result.audit.detected_bbox_mask_iou == pytest.approx(1.0)
    assert result.audit.min_target_points == 1
    assert result.audit.min_support_points == 1
    assert result.audit.min_total_points == 1
    assert result.audit.min_target_fraction == pytest.approx(1.0)


@pytest.mark.parametrize(
    'overrides',
    [
        {'context_plane_distance_m': 0.0},
        {'context_plane_distance_m': -0.001},
        {'context_plane_distance_m': float('nan')},
        {'context_plane_distance_m': float('inf')},
        {'context_plane_distance_m': True},
        {'target_guard_px': -1},
        {'target_guard_px': 0.5},
        {'target_guard_px': float('nan')},
        {'target_guard_px': float('inf')},
        {'target_guard_px': True},
    ],
)
def test_plane_band_and_guard_configuration_fail_closed(scene, overrides):
    _assert_code('CONFIG_INVALID', lambda: _build(scene, **overrides))


@pytest.mark.parametrize(
    'bbox',
    [
        (1, 2, 3),
        (1, 2, 3, -1),
        (1, 2, 3, 4.5),
        (1, 2, np.nan, 4),
        (1, 2, True, 4),
    ],
)
def test_invalid_detected_bbox_is_rejected(scene, bbox):
    _assert_code(
        'BBOX_INVALID',
        lambda: _build(scene, detected_bbox_xywh=bbox),
    )


@pytest.mark.parametrize(
    'expected_code,overrides',
    [
        ('TARGET_POINTS_INSUFFICIENT', {'min_target_points': 1201}),
        ('SUPPORT_POINTS_INSUFFICIENT', {'min_support_points': 100000}),
        ('TOTAL_POINTS_INSUFFICIENT', {'min_total_points': 100000}),
        ('TARGET_FRACTION_INSUFFICIENT', {'min_target_fraction': 0.99}),
    ],
)
def test_point_count_and_target_fraction_gates_fail_closed(
    scene,
    expected_code,
    overrides,
):
    error = _assert_code(expected_code, lambda: _build(scene, **overrides))
    assert error.audit['target_points'] == 1200
    assert error.audit['total_points'] >= error.audit['target_points']
    assert 0.0 < error.audit['target_fraction'] <= 1.0


def test_default_production_thresholds_are_recorded_in_audit(scene):
    result = _build(scene)

    assert result.audit.min_target_points == 120
    assert result.audit.min_support_points == 200
    assert result.audit.min_total_points == 320
    assert result.audit.min_target_fraction == pytest.approx(0.15)
    assert result.audit.target_hole_filled_points == 0
    assert result.audit.detected_bbox_xywh is None
    assert result.audit.detected_bbox_mask_iou is None
