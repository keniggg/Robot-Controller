#!/usr/bin/env python3
import pathlib
import sys
from types import SimpleNamespace

import numpy as np
import pytest
from tf.transformations import quaternion_matrix


ROOT = pathlib.Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / 'src'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from alicia_flexible_grasp.vision.object_geometry import (
    GeometryEstimate,
    deproject_depth,
    estimate_object_geometry,
)


def _normalize(vector):
    vector = np.asarray(vector, dtype=float)
    return vector / np.linalg.norm(vector)


def _render_scene():
    height, width = 300, 420
    fx = 520.0
    fy = 515.0
    cx = 205.0
    cy = 148.0
    depth_scale = 0.0001
    intrinsics = SimpleNamespace(
        width=width,
        height=height,
        fx=fx,
        fy=fy,
        cx=cx,
        cy=cy,
        depth_scale=depth_scale,
    )

    support_normal_camera = _normalize([0.52, -0.34, -1.0])
    support_center_camera = np.asarray([0.045, 0.025, 0.82], dtype=float)
    support_offset_camera = -float(np.dot(support_normal_camera, support_center_camera))
    long_hint = _normalize([np.cos(np.deg2rad(27.0)), np.sin(np.deg2rad(27.0)), 0.0])
    long_axis_camera = _normalize(
        long_hint - np.dot(long_hint, support_normal_camera) * support_normal_camera
    )
    short_axis_camera = _normalize(np.cross(support_normal_camera, long_axis_camera))
    box_axes_camera = np.column_stack(
        [long_axis_camera, short_axis_camera, support_normal_camera]
    )
    box_size = np.asarray([0.24, 0.16, 0.10], dtype=float)
    box_center_camera = support_center_camera + support_normal_camera * (0.5 * box_size[2])

    vv, uu = np.indices((height, width), dtype=np.float64)
    rays = np.stack(
        [
            (uu - cx) / fx,
            (vv - cy) / fy,
            np.ones_like(uu),
        ],
        axis=-1,
    )
    plane_denom = rays @ support_normal_camera
    plane_depth = np.where(
        np.abs(plane_denom) > 1e-10,
        -support_offset_camera / plane_denom,
        np.inf,
    )

    origin_local = box_axes_camera.T @ (-box_center_camera)
    rays_local = rays @ box_axes_camera
    half = box_size * 0.5
    with np.errstate(divide='ignore', invalid='ignore'):
        t0 = (-half - origin_local) / rays_local
        t1 = (half - origin_local) / rays_local
    slab_near = np.minimum(t0, t1)
    slab_far = np.maximum(t0, t1)
    box_near = np.max(slab_near, axis=-1)
    box_far = np.min(slab_far, axis=-1)
    box_hit = (box_far >= np.maximum(box_near, 0.0)) & np.isfinite(box_near)
    box_depth = np.where(box_hit, box_near, np.inf)

    valid_plane = np.isfinite(plane_depth) & (plane_depth > 0.0)
    target = box_hit & (box_depth > 0.0) & (box_depth < plane_depth)
    full_depth_m = np.where(target, box_depth, np.where(valid_plane, plane_depth, 0.0))
    target_depth_m = np.where(target, box_depth, 0.0)
    full_depth_raw = np.rint(full_depth_m / depth_scale).astype(np.uint16)
    target_depth_raw = np.rint(target_depth_m / depth_scale).astype(np.uint16)
    mask = np.where(target, 255, 0).astype(np.uint8)
    ys, xs = np.nonzero(mask)
    bbox = (
        int(np.min(xs)),
        int(np.min(ys)),
        int(np.max(xs) - np.min(xs) + 1),
        int(np.max(ys) - np.min(ys) + 1),
    )

    yaw = np.deg2rad(19.0)
    base_from_camera = quaternion_matrix([0.0, 0.0, np.sin(yaw / 2.0), np.cos(yaw / 2.0)])
    base_from_camera[:3, 3] = [0.32, -0.18, 0.27]
    support_normal_base = base_from_camera[:3, :3] @ support_normal_camera
    return SimpleNamespace(
        full_depth_raw=full_depth_raw,
        masked_box_depth_raw=target_depth_raw,
        mask=mask,
        bbox=bbox,
        intrinsics=intrinsics,
        T_base_camera=base_from_camera,
        support_normal_base=support_normal_base,
    )


@pytest.fixture
def scene():
    return _render_scene()


def geometry_kwargs(scene):
    return {
        'depth_raw': scene.full_depth_raw,
        'target_depth_raw': scene.masked_box_depth_raw,
        'object_mask': scene.mask,
        'bbox': scene.bbox,
        'intrinsics': scene.intrinsics,
        'depth_scale': 0.0001,
        'T_base_camera': scene.T_base_camera,
        'source_mode': 'instance_mask',
        'support_bbox_expand_ratio': 0.30,
        'support_distance_threshold_m': 0.004,
        'voxel_size_m': 0.0025,
        'min_support_points': 200,
        'min_object_points': 120,
        'min_size_m': 0.005,
        'max_size_m': 0.600,
        'max_height_m': 0.500,
        'previous_axes_base': None,
    }


def one_pixel_wide_target(scene):
    output = np.zeros_like(scene.masked_box_depth_raw)
    columns = np.flatnonzero(np.any(scene.masked_box_depth_raw > 0, axis=0))
    column = int(columns[len(columns) // 2])
    output[:, column] = scene.masked_box_depth_raw[:, column]
    return output


def test_mask_geometry_recovers_box_and_excludes_support_points(scene):
    estimate = estimate_object_geometry(**geometry_kwargs(scene))

    assert estimate.ok, estimate.failure_reason
    np.testing.assert_allclose(
        np.sort(estimate.size_xyz_m),
        [0.10, 0.16, 0.24],
        atol=0.018,
    )
    assert estimate.support_inlier_ratio >= 0.70
    signed = estimate.object_points_base @ estimate.support_normal_base + estimate.support_offset_m
    assert float(np.min(signed)) > -0.005
    assert np.linalg.det(estimate.axes_base) > 0.999


def test_support_plane_never_uses_mask_pixels(scene):
    corrupted = scene.full_depth_raw.copy()
    corrupted[scene.mask > 0] = 1
    args = geometry_kwargs(scene)
    args['depth_raw'] = corrupted

    estimate = estimate_object_geometry(**args)

    assert estimate.ok, estimate.failure_reason
    np.testing.assert_allclose(
        estimate.support_normal_base,
        scene.support_normal_base,
        atol=0.04,
    )


def test_too_few_context_points_reports_support_plane_invalid(scene):
    args = geometry_kwargs(scene)
    args['depth_raw'] = np.where(scene.mask > 0, scene.full_depth_raw, 0)

    result = estimate_object_geometry(**args)

    assert not result.ok
    assert result.failure_code == 'SUPPORT_PLANE_INVALID'


def test_degenerate_object_reports_obb_invalid(scene):
    args = geometry_kwargs(scene)
    args['target_depth_raw'] = one_pixel_wide_target(scene)

    result = estimate_object_geometry(**args)

    assert not result.ok
    assert result.failure_code == 'OBB_INVALID'


def test_nan_transform_fails_closed_with_obb_invalid(scene):
    args = geometry_kwargs(scene)
    args['T_base_camera'] = scene.T_base_camera.copy()
    args['T_base_camera'][0, 3] = np.nan

    result = estimate_object_geometry(**args)

    assert not result.ok
    assert result.failure_code == 'OBB_INVALID'
    assert 'transform' in result.failure_reason


def test_out_of_range_dimension_reports_obb_invalid(scene):
    args = geometry_kwargs(scene)
    args['max_size_m'] = 0.20

    result = estimate_object_geometry(**args)

    assert not result.ok
    assert result.failure_code == 'OBB_INVALID'
    assert 'size' in result.failure_reason


def test_axis_sign_stabilizes_against_previous_axes(scene):
    first = estimate_object_geometry(**geometry_kwargs(scene))
    assert first.ok, first.failure_reason
    previous = first.axes_base.copy()
    previous[:, :2] *= -1.0
    args = geometry_kwargs(scene)
    args['previous_axes_base'] = previous

    stabilized = estimate_object_geometry(**args)

    assert stabilized.ok, stabilized.failure_reason
    assert float(np.dot(stabilized.axes_base[:, 0], previous[:, 0])) > 0.99
    assert float(np.dot(stabilized.axes_base[:, 1], previous[:, 1])) > 0.99
    assert np.linalg.det(stabilized.axes_base) > 0.999


def test_nonrigid_previous_axes_reports_obb_invalid(scene):
    args = geometry_kwargs(scene)
    args['previous_axes_base'] = np.zeros((3, 3), dtype=float)

    result = estimate_object_geometry(**args)

    assert not result.ok
    assert result.failure_code == 'OBB_INVALID'
    assert 'previous axes' in result.failure_reason


@pytest.mark.parametrize(
    'mutate, reason_text',
    [
        (lambda args: setattr(args['intrinsics'], 'fx', 0.0), 'intrinsics'),
        (lambda args: args.update(object_mask=np.zeros((2, 3), dtype=np.uint8)), 'shape'),
        (lambda args: args.update(source_mode='unknown'), 'source_mode'),
        (lambda args: args.update(support_distance_threshold_m=np.nan), 'threshold'),
        (lambda args: args.update(min_size_m=0.7, max_size_m=0.6), 'dimension'),
    ],
)
def test_invalid_geometry_inputs_fail_closed(scene, mutate, reason_text):
    args = geometry_kwargs(scene)
    args['intrinsics'] = SimpleNamespace(**vars(scene.intrinsics))
    mutate(args)

    result = estimate_object_geometry(**args)

    assert not result.ok
    assert result.failure_code in ('SUPPORT_PLANE_INVALID', 'OBB_INVALID')
    assert reason_text in result.failure_reason


def test_deproject_depth_returns_points_and_matching_uv():
    depth = np.asarray([[0, 1000], [2000, 0]], dtype=np.uint16)
    mask = np.ones((2, 2), dtype=np.uint8)
    intrinsics = SimpleNamespace(width=2, height=2, fx=100.0, fy=200.0, cx=0.0, cy=0.0)

    points, uv = deproject_depth(depth, mask, intrinsics, 0.001)

    np.testing.assert_allclose(points, [[0.01, 0.0, 1.0], [0.0, 0.01, 2.0]])
    np.testing.assert_array_equal(uv, [[1, 0], [0, 1]])


def test_geometry_estimate_arrays_are_defensive_read_only_copies():
    center = np.asarray([1.0, 2.0, 3.0])
    axes = np.eye(3)
    size = np.asarray([0.2, 0.1, 0.05])
    normal = np.asarray([0.0, 0.0, 1.0])
    points = np.asarray([[1.0, 2.0, 3.0]])

    estimate = GeometryEstimate(
        ok=True,
        failure_code='',
        failure_reason='',
        center_base=center,
        axes_base=axes,
        size_xyz_m=size,
        support_normal_base=normal,
        support_offset_m=0.0,
        support_inlier_ratio=1.0,
        object_points_base=points,
        source_mode='instance_mask',
    )
    center[0] = 99.0
    axes[0, 0] = 99.0
    size[0] = 99.0
    normal[2] = 99.0
    points[0, 0] = 99.0

    np.testing.assert_allclose(estimate.center_base, [1.0, 2.0, 3.0])
    np.testing.assert_allclose(estimate.axes_base, np.eye(3))
    np.testing.assert_allclose(estimate.size_xyz_m, [0.2, 0.1, 0.05])
    np.testing.assert_allclose(estimate.support_normal_base, [0.0, 0.0, 1.0])
    np.testing.assert_allclose(estimate.object_points_base, [[1.0, 2.0, 3.0]])
    for value in (
        estimate.center_base,
        estimate.axes_base,
        estimate.size_xyz_m,
        estimate.support_normal_base,
        estimate.object_points_base,
    ):
        assert not value.flags.writeable
        with pytest.raises(ValueError):
            value.flat[0] = 0.0
