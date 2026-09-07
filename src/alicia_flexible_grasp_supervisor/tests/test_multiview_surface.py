#!/usr/bin/env python3
import dataclasses
import pathlib
import subprocess
import sys

import numpy as np
import pytest


ROOT = pathlib.Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / 'src'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))


from alicia_flexible_grasp.vision.multiview_surface import (  # noqa: E402
    FusedTargetSurface,
    RegistrationConfig,
    SurfaceView,
    append_registered_view,
    fused_surface_from_view,
    register_surface_view,
    surface_view_from_observation,
)
from alicia_flexible_grasp.vision.target_observation import (  # noqa: E402
    TargetObservation,
    TargetTrackIdentity,
)


IDENTITY = TargetTrackIdentity.from_stream(3, 7)


def test_registration_runs_with_documented_system_numpy():
    script = '''
import runpy
import numpy as np
tests = runpy.run_path(%r)
reference, moving = tests['registered_pair']()
result = tests['register_surface_view'](reference, moving)
assert result.ok, result.code
assert result.inlier_count >= 80
assert result.rmse_m <= 0.004
print(np.__version__, result.code)
''' % str(pathlib.Path(__file__).resolve())
    result = subprocess.run(
        ['/usr/bin/python3', '-s', '-c', script],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert 'REGISTERED' in result.stdout


def rectangular_prism_views():
    """Return generic top/front and top/side measured surface samples."""

    xs = np.linspace(-0.040, 0.040, 17)
    ys = np.linspace(-0.025, 0.025, 13)
    zs = np.linspace(0.004, 0.034, 9)
    top = np.asarray([(x, y, 0.034) for x in xs for y in ys])
    front = np.asarray([(x, -0.025, z) for x in xs for z in zs[:-1]])
    side = np.asarray([(0.040, y, z) for y in ys for z in zs[:-1]])
    return np.vstack((top, front)), np.vstack((top, side))


def opposing_side_views(extra_moving_points=()):
    """Return a shared top plus measured negative/positive opposing faces."""

    xs = np.linspace(-0.040, 0.040, 17)
    ys = np.linspace(-0.025, 0.025, 15)
    zs = np.linspace(0.004, 0.034, 15)
    top = np.asarray([(x, y, 0.034) for x in xs for y in ys])
    negative = np.asarray([(x, -0.025, z) for x in xs for z in zs[:-1]])
    positive = np.asarray([(x, 0.025, z) for x in xs for z in zs[:-1]])
    moving = np.vstack((top, positive, np.asarray(extra_moving_points).reshape(-1, 3)))
    return view(np.vstack((top, negative)), 1_000_000_000), view(
        moving, 1_100_000_000
    )


def transform_points_inverse(points, yaw_deg=3.0, translation=(0.003, -0.002, 0.001)):
    yaw = np.deg2rad(float(yaw_deg))
    rotation = np.asarray(
        [
            [np.cos(yaw), -np.sin(yaw), 0.0],
            [np.sin(yaw), np.cos(yaw), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    translation = np.asarray(translation, dtype=float)
    return (np.asarray(points) - translation).dot(rotation)


def view(points, stamp_ns, identity=IDENTITY, normal=(0.0, 0.0, 1.0), offset=0.0, edge=12):
    return SurfaceView(
        identity=identity,
        stamp_ns=stamp_ns,
        points_base=points,
        support_normal_base=normal,
        support_offset_m=offset,
        edge_clearance_px=edge,
    )


def registered_pair(**moving_kwargs):
    reference_points, moving_aligned = rectangular_prism_views()
    reference = view(reference_points, 1_000_000_000)
    moving = view(
        transform_points_inverse(moving_aligned),
        1_100_000_000,
        **moving_kwargs
    )
    return reference, moving


def test_registers_partial_measured_views_deterministically_with_provenance():
    reference, moving = registered_pair()

    first_result = register_surface_view(reference, moving)
    second_result = register_surface_view(reference, moving)
    first_result, first_surface = append_registered_view(
        fused_surface_from_view(reference), reference, moving
    )
    second_result, second_surface = append_registered_view(
        fused_surface_from_view(reference), reference, moving
    )

    assert first_result.ok is True
    assert first_result.code == 'REGISTERED'
    assert first_result.inlier_count >= 80
    assert first_result.overlap_fraction >= 0.30
    assert first_result.rmse_m <= 0.004
    assert np.linalg.norm(first_result.transform_base[:3, 3]) <= 0.025
    assert np.array_equal(first_result.transform_base, second_result.transform_base)
    assert np.array_equal(first_surface.points_base, second_surface.points_base)
    assert np.array_equal(first_surface.view_indices, second_surface.view_indices)
    assert first_surface.view_stamps_ns == (1_000_000_000, 1_100_000_000)
    assert set(first_surface.view_indices.tolist()) == {0, 1}
    assert len(first_surface.points_base) < len(reference.points_base) + first_result.inlier_count
    for array in (
        reference.points_base,
        reference.support_normal_base,
        first_result.transform_base,
        first_surface.points_base,
        first_surface.view_indices,
    ):
        assert array.flags.writeable is False
        with pytest.raises(ValueError):
            array.setflags(write=True)


@pytest.mark.parametrize(
    ('mutate', 'expected_code'),
    [
        (
            lambda reference, moving: dataclasses.replace(
                moving, identity=TargetTrackIdentity.from_stream(3, 8)
            ),
            'IDENTITY_MISMATCH',
        ),
        (
            lambda reference, moving: dataclasses.replace(
                moving, stamp_ns=reference.stamp_ns
            ),
            'STAMP_NOT_MONOTONIC',
        ),
        (
            lambda reference, moving: dataclasses.replace(
                moving,
                support_normal_base=(0.0, np.sin(np.deg2rad(5.0)), np.cos(np.deg2rad(5.0))),
            ),
            'SUPPORT_NORMAL_MISMATCH',
        ),
        (
            lambda reference, moving: dataclasses.replace(
                moving, support_offset_m=0.0041
            ),
            'SUPPORT_OFFSET_MISMATCH',
        ),
    ],
)
def test_registration_rejects_mixed_identity_stamp_and_support(mutate, expected_code):
    reference, moving = registered_pair()

    result = register_surface_view(reference, mutate(reference, moving))

    assert result.ok is False
    assert result.code == expected_code


def test_registration_rejects_fewer_than_eighty_inliers():
    reference_points, _moving_points = rectangular_prism_views()
    reference = view(reference_points, 1_000_000_000)
    moving = view(
        transform_points_inverse(reference_points[:79]),
        1_100_000_000,
    )

    result = register_surface_view(reference, moving)

    assert result.ok is False
    assert result.code == 'INLIERS_INSUFFICIENT'
    assert result.inlier_count < 80


def test_registration_rejects_overlap_below_thirty_percent():
    reference_points, _moving_points = rectangular_prism_views()
    outliers = np.column_stack(
        (
            np.linspace(0.20, 0.28, 81),
            np.linspace(-0.20, 0.20, 81),
            np.linspace(0.08, 0.16, 81),
        )
    )
    candidate = np.vstack((reference_points[:29], outliers))
    reference = view(reference_points, 1_000_000_000)
    moving = view(candidate, 1_100_000_000)

    result = register_surface_view(
        reference,
        moving,
        RegistrationConfig(minimum_inliers=20),
    )

    assert result.ok is False
    assert result.code == 'OVERLAP_INSUFFICIENT'
    assert result.overlap_fraction < 0.30


def test_registration_rejects_rmse_above_four_millimeters():
    reference_points, _moving_points = rectangular_prism_views()
    candidate = reference_points[:120].copy()
    candidate[::2, 2] += 0.005
    candidate[1::2, 2] -= 0.005
    reference = view(reference_points, 1_000_000_000)
    moving = view(candidate, 1_100_000_000)

    result = register_surface_view(reference, moving)

    assert result.ok is False
    assert result.code == 'RMSE_EXCEEDED'
    assert result.rmse_m > 0.004


@pytest.mark.parametrize(
    ('yaw_deg', 'translation', 'expected_code'),
    [
        (0.0, (0.0, 0.0, 0.026), 'TRANSLATION_BOUND_EXCEEDED'),
        (10.5, (0.0, 0.0, 0.0), 'YAW_BOUND_EXCEEDED'),
    ],
)
def test_registration_rejects_corrections_outside_bounds(
    yaw_deg, translation, expected_code
):
    reference_points, _moving_points = rectangular_prism_views()
    reference = view(reference_points, 1_000_000_000)
    moving = view(
        transform_points_inverse(
            reference_points,
            yaw_deg=yaw_deg,
            translation=translation,
        ),
        1_100_000_000,
    )

    result = register_surface_view(
        reference,
        moving,
        RegistrationConfig(correspondence_max_m=0.040),
    )

    assert result.ok is False
    assert result.code == expected_code


def target_observation(points, bbox):
    x, y, width, height = bbox
    return TargetObservation(
        identity=IDENTITY,
        stamp_ns=1_100_000_000,
        frame_id='base_link',
        points_base=points,
        support_normal_base=(0.0, 0.0, 1.0),
        support_offset_m=0.0,
        bbox_xywh=bbox,
        image_shape_hw=(480, 640),
        edge_clearance_px=min(x, y, 640 - x - width, 480 - y - height),
        source_kind='instance_mask',
        source_label='diagnostic-only',
    )


def test_bottom_clipped_bbox_is_diagnostic_and_registration_uses_only_3d_evidence():
    reference_points, moving_aligned = rectangular_prism_views()
    measured = transform_points_inverse(moving_aligned)
    reference = view(reference_points, 1_000_000_000)
    clipped = surface_view_from_observation(
        target_observation(measured, (232, 364, 115, 116))
    )
    other_bbox = surface_view_from_observation(
        target_observation(measured, (12, 12, 115, 116))
    )

    clipped_result = register_surface_view(reference, clipped)
    other_result = register_surface_view(reference, other_bbox)

    assert clipped.is_clipped is True
    assert other_bbox.is_clipped is False
    assert clipped_result.ok is True
    assert np.array_equal(clipped_result.transform_base, other_result.transform_base)


@pytest.mark.parametrize(
    ('field', 'bad_value'),
    [
        ('correspondence_max_m', True),
        ('correspondence_max_m', float('nan')),
        ('minimum_inliers', 80.0),
        ('minimum_overlap_fraction', 0.0),
        ('maximum_rmse_m', -0.001),
        ('maximum_translation_m', float('inf')),
        ('maximum_yaw_deg', 181.0),
        ('maximum_support_normal_angle_deg', -1.0),
        ('maximum_support_offset_delta_m', '0.004'),
        ('maximum_iterations', 0),
    ],
)
def test_registration_config_fails_closed_without_coercion(field, bad_value):
    with pytest.raises(ValueError):
        RegistrationConfig(**{field: bad_value})


def test_fused_surface_rejects_mixed_or_nonmonotonic_provenance():
    reference, moving = registered_pair()
    surface = fused_surface_from_view(reference)

    for invalid in (
        dataclasses.replace(moving, identity=TargetTrackIdentity.from_stream(4, 7)),
        dataclasses.replace(moving, stamp_ns=reference.stamp_ns),
    ):
        result, unchanged = append_registered_view(surface, reference, invalid)
        assert result.ok is False
        assert unchanged is surface

    with pytest.raises(ValueError):
        FusedTargetSurface(
            identity=IDENTITY,
            points_base=np.zeros((2, 3)),
            view_indices=np.asarray([0, 2]),
            view_stamps_ns=(1_000_000_000, 1_100_000_000),
        )


def test_fusion_retains_only_real_connected_measurements_and_rejection_is_atomic():
    reference, moving = registered_pair()
    original = fused_surface_from_view(reference)
    result, fused = append_registered_view(original, reference, moving)
    assert result.ok
    transformed = moving.points_base.dot(result.transform_base[:3, :3].T) + result.transform_base[:3, 3]
    distances = np.linalg.norm(transformed[:, None] - reference.points_base[None, :], axis=2).min(axis=1)
    assert np.count_nonzero(distances > RegistrationConfig().correspondence_max_m) > 0
    for point, index in zip(fused.points_base, fused.view_indices):
        measured = reference.points_base if index == 0 else transformed
        assert np.any(np.all(measured == point, axis=1))
    retained_moving = fused.points_base[fused.view_indices == 1]
    retained_distances = np.linalg.norm(
        retained_moving[:, None] - reference.points_base[None, :], axis=2
    ).min(axis=1)
    assert np.count_nonzero(
        retained_distances > RegistrationConfig().correspondence_max_m
    ) > 0
    rejected = dataclasses.replace(moving, stamp_ns=moving.stamp_ns + 1, support_offset_m=0.005)
    failure, unchanged = append_registered_view(fused, reference, rejected)
    assert failure.code == 'SUPPORT_OFFSET_MISMATCH'
    assert unchanged is fused
    assert original.view_stamps_ns == (reference.stamp_ns,)


def test_fusion_keeps_newly_measured_opposing_face_from_real_append_boundary():
    reference, moving = opposing_side_views()

    result, fused = append_registered_view(
        fused_surface_from_view(reference), reference, moving
    )

    assert result.ok
    assert result.inlier_count == 255
    assert result.overlap_fraction == pytest.approx(255.0 / 493.0)
    moving_points = fused.points_base[fused.view_indices == 1]
    positive_face = moving_points[np.isclose(moving_points[:, 1], 0.025)]
    assert len(positive_face) > 0
    assert np.ptp(positive_face[:, 2]) > 0.0
    transformed = (
        moving.points_base.dot(result.transform_base[:3, :3].T)
        + result.transform_base[:3, 3]
    )
    for point in positive_face:
        assert np.any(np.all(transformed == point, axis=1))


def test_fusion_excludes_disconnected_cluster_and_is_order_and_repeat_invariant():
    disconnected = np.asarray([
        (-0.002, 0.000, 0.010),
        (0.000, 0.000, 0.012),
        (0.002, 0.000, 0.014),
    ])
    reference, moving = opposing_side_views(disconnected)
    rng = np.random.RandomState(17)
    shuffled = dataclasses.replace(
        moving, points_base=moving.points_base[rng.permutation(len(moving.points_base))]
    )
    repeated = dataclasses.replace(
        moving, points_base=np.repeat(shuffled.points_base, 3, axis=0)
    )

    surfaces = []
    results = []
    for candidate in (moving, shuffled, repeated):
        result, fused = append_registered_view(
            fused_surface_from_view(reference), reference, candidate
        )
        assert result.ok
        assert result.inlier_count == 255
        assert result.overlap_fraction == pytest.approx(255.0 / 496.0)
        results.append(result)
        surfaces.append(fused)

    for result in results[1:]:
        assert result.inlier_count == results[0].inlier_count
        assert result.overlap_fraction == results[0].overlap_fraction
        np.testing.assert_array_equal(result.transform_base, results[0].transform_base)
    for fused in surfaces[1:]:
        np.testing.assert_array_equal(fused.points_base, surfaces[0].points_base)
        np.testing.assert_array_equal(fused.view_indices, surfaces[0].view_indices)
    retained = surfaces[0].points_base[surfaces[0].view_indices == 1]
    for point in disconnected:
        assert not np.any(np.all(retained == point, axis=1))


@pytest.mark.parametrize('outside', [False, True])
def test_fusion_connectivity_bound_is_exact(outside):
    from alicia_flexible_grasp.vision import multiview_surface as module
    distance = np.nextafter(0.008, np.inf) if outside else 0.008
    points = np.asarray([[0.0, 0.0, 0.0], [distance, 0.0, 0.0]])

    connected = module._connected_measurement_mask(
        points, np.asarray([True, False]), RegistrationConfig()
    )

    np.testing.assert_array_equal(connected, [True, not outside])


@pytest.mark.parametrize('axis', [0, 1, 2])
@pytest.mark.parametrize('outside', [False, True])
def test_fusion_connectivity_bound_is_exact_across_signed_zero(axis, outside):
    from alicia_flexible_grasp.vision import multiview_surface as module
    radius = RegistrationConfig().correspondence_max_m
    candidate = np.nextafter(radius, np.inf) if outside else radius
    points = np.zeros((2, 3))
    points[0, axis] = np.nextafter(0.0, -np.inf)
    points[1, axis] = candidate
    distance = np.linalg.norm(points[1] - points[0])

    connected = module._connected_measurement_mask(
        points, np.asarray([True, False]), RegistrationConfig()
    )

    assert distance == candidate
    assert bool(distance <= radius) is (not outside)
    np.testing.assert_array_equal(connected, [True, not outside])


@pytest.mark.parametrize('origin', [-0.016, -0.008, 0.0, 0.008, 0.016])
@pytest.mark.parametrize('direction', [-1.0, 1.0])
def test_fusion_connectivity_matches_exact_distance_across_shifted_bins(
    origin, direction
):
    from alicia_flexible_grasp.vision import multiview_surface as module
    config = RegistrationConfig()
    points = np.asarray([
        [origin, 0.0, 0.0],
        [origin + direction * config.correspondence_max_m, 0.0, 0.0],
    ])
    expected = np.linalg.norm(points[1] - points[0]) <= config.correspondence_max_m

    connected = module._connected_measurement_mask(
        points, np.asarray([True, False]), config
    )

    np.testing.assert_array_equal(connected, [True, expected])


def test_registration_is_independent_of_point_order_and_support_plane_orientation():
    reference, moving = registered_pair()
    baseline = register_surface_view(reference, moving)
    shuffled = register_surface_view(
        dataclasses.replace(reference, points_base=reference.points_base[::-1]),
        dataclasses.replace(moving, points_base=moving.points_base[::-1]),
    )
    assert np.array_equal(baseline.transform_base, shuffled.transform_base)
    angle = np.deg2rad(25.0)
    tilt = np.array([[1., 0., 0.], [0., np.cos(angle), -np.sin(angle)], [0., np.sin(angle), np.cos(angle)]])
    normal = tilt[:, 2]
    tilted = register_surface_view(
        dataclasses.replace(reference, points_base=reference.points_base.dot(tilt.T), support_normal_base=normal),
        dataclasses.replace(moving, points_base=moving.points_base.dot(tilt.T), support_normal_base=normal),
    )
    assert tilted.ok
    np.testing.assert_allclose(tilted.transform_base[:3, :3].dot(normal), normal, atol=1e-12)
    np.testing.assert_allclose(tilted.transform_base[:3, :3], tilt.dot(baseline.transform_base[:3, :3]).dot(tilt.T), atol=1e-8)
    np.testing.assert_allclose(tilted.transform_base[:3, 3], tilt.dot(baseline.transform_base[:3, 3]), atol=1e-8)


def test_chunked_nearest_neighbors_match_full_distances_including_ties(monkeypatch):
    from alicia_flexible_grasp.vision import multiview_surface as module
    rng = np.random.default_rng(42)
    source = rng.normal(size=(19, 3))
    target = rng.normal(size=(23, 3))
    target[0] = source[0]
    target[8] = source[0]
    monkeypatch.setattr(module, '_NEAREST_NEIGHBOR_CHUNK', 4)
    monkeypatch.setattr(module, '_NEAREST_NEIGHBOR_TARGET_CHUNK', 5)
    indices, distances = module._nearest_neighbors(source, target)
    full = np.linalg.norm(source[:, None] - target[None, :], axis=2)
    np.testing.assert_array_equal(indices, np.argmin(full, axis=1))
    np.testing.assert_allclose(distances, np.min(full, axis=1))


@pytest.mark.parametrize('outside', [False, True])
def test_correspondence_bound_is_exact(outside):
    from alicia_flexible_grasp.vision.multiview_surface import _registration_correspondences
    distance = np.nextafter(0.008, np.inf) if outside else 0.008
    _points, _indices, distances, inliers = _registration_correspondences(
        np.zeros((1, 3)), np.array([[distance, 0., 0.]]), np.eye(4), RegistrationConfig())
    assert distances[0] == distance
    assert bool(inliers[0]) is (not outside)


@pytest.mark.parametrize('outside', [False, True])
def test_support_offset_bound_is_exact(outside):
    reference, moving = registered_pair()
    delta = np.nextafter(0.004, np.inf) if outside else 0.004
    result = register_surface_view(reference, dataclasses.replace(moving, support_offset_m=delta))
    assert result.ok is (not outside)
    if outside:
        assert result.code == 'SUPPORT_OFFSET_MISMATCH'


def test_support_plane_gate_uses_target_local_separation_for_tilted_fit():
    points, _other_view = rectangular_prism_views()
    points = points + np.asarray([-0.12, -0.38, 0.06])
    anchor = np.median(points, axis=0)
    reference_normal = np.asarray([0.0, 0.0, 1.0])
    angle = np.deg2rad(0.9)
    moving_normal = np.asarray([0.0, np.sin(angle), np.cos(angle)])
    reference_offset = -float(np.dot(reference_normal, anchor))
    moving_offset = -float(np.dot(moving_normal, anchor))
    assert abs(reference_offset - moving_offset) > 0.004
    reference = view(
        points,
        1_000_000_000,
        normal=reference_normal,
        offset=reference_offset,
    )
    moving = view(
        points,
        1_100_000_000,
        normal=moving_normal,
        offset=moving_offset,
    )

    result = register_surface_view(reference, moving)

    assert result.ok
    assert result.code == 'REGISTERED'
    assert result.support_plane_separation_m == pytest.approx(0.0, abs=1e-12)


def test_support_plane_gate_rejects_target_local_separation_over_bound():
    points, _other_view = rectangular_prism_views()
    points = points + np.asarray([-0.12, -0.38, 0.06])
    anchor = np.median(points, axis=0)
    reference_normal = np.asarray([0.0, 0.0, 1.0])
    angle = np.deg2rad(0.9)
    moving_normal = np.asarray([0.0, np.sin(angle), np.cos(angle)])
    reference_offset = -float(np.dot(reference_normal, anchor))
    moving_offset = -float(np.dot(moving_normal, anchor)) + 0.0041
    reference = view(
        points,
        1_000_000_000,
        normal=reference_normal,
        offset=reference_offset,
    )
    moving = view(
        points,
        1_100_000_000,
        normal=moving_normal,
        offset=moving_offset,
    )

    result = register_surface_view(reference, moving)

    assert not result.ok
    assert result.code == 'SUPPORT_OFFSET_MISMATCH'
    assert result.support_plane_separation_m == pytest.approx(0.0041)


@pytest.mark.parametrize('outside', [False, True])
def test_support_angle_bound_is_exact(outside):
    reference, moving = registered_pair()
    moving = dataclasses.replace(moving, support_normal_base=(0., 0.05, np.sqrt(1. - 0.05 ** 2)))
    angle = float(np.degrees(np.arccos(moving.support_normal_base[2])))
    # The reported finite angle is the boundary; tightening by one ULP puts
    # that same physical measurement outside, without trigonometric rounding.
    bound = np.nextafter(angle, 0.) if outside else angle
    result = register_surface_view(reference, moving, RegistrationConfig(maximum_support_normal_angle_deg=bound))
    assert result.ok is (not outside)
    if outside:
        assert result.code == 'SUPPORT_NORMAL_MISMATCH'


@pytest.mark.parametrize('field,code', [
    ('maximum_translation_m', 'TRANSLATION_BOUND_EXCEEDED'),
    ('maximum_yaw_deg', 'YAW_BOUND_EXCEEDED'),
    ('maximum_rmse_m', 'RMSE_EXCEEDED'),
    ('minimum_overlap_fraction', 'OVERLAP_INSUFFICIENT'),
    ('minimum_inliers', 'INLIERS_INSUFFICIENT'),
])
@pytest.mark.parametrize('outside', [False, True])
def test_reported_registration_metric_bound_is_exact(field, code, outside):
    reference, moving = registered_pair()
    config = RegistrationConfig()
    if field in ('maximum_translation_m', 'maximum_yaw_deg'):
        # One ICP update on a narrow fragment: the reported correction is
        # also the largest visited correction (no earlier coarse overshoot).
        moving = dataclasses.replace(moving, points_base=transform_points_inverse(reference.points_base[:100]))
        config = dataclasses.replace(config, maximum_iterations=1)
    if field == 'minimum_overlap_fraction':
        # Narrow measured fragment disables coarse footprint initialization,
        # so changing the acceptance threshold cannot change the optimizer.
        points = reference.points_base[:100]
        moving = dataclasses.replace(moving, points_base=np.vstack((
            points, points[:20] + [0., 0., 0.1])))
    baseline = register_surface_view(reference, moving, config)
    assert baseline.ok
    values = {
        'maximum_translation_m': float(np.linalg.norm(baseline.transform_base[:3, 3])),
        # For normal +Z the support basis is (-Y, +X, +Z).
        'maximum_yaw_deg': abs(float(np.degrees(np.arctan2(-baseline.transform_base[0, 1], baseline.transform_base[1, 1])))),
        'maximum_rmse_m': baseline.rmse_m,
        'minimum_overlap_fraction': baseline.overlap_fraction,
        'minimum_inliers': baseline.inlier_count,
    }
    bound = values[field]
    if outside:
        bound = bound + 1 if field == 'minimum_inliers' else np.nextafter(
            bound, np.inf if field.startswith('minimum') else 0.)
    result = register_surface_view(reference, moving, dataclasses.replace(config, **{field: bound}))
    assert result.ok is (not outside), result.code
    if outside:
        assert result.code == code


@pytest.mark.parametrize('kind', ['duplicates', 'distinct_many_to_one', 'line'])
def test_registration_rejects_collapsed_or_repeated_correspondences(kind):
    reference, moving = registered_pair()
    if kind == 'duplicates':
        points = np.repeat(reference.points_base[:1], 80, axis=0)
    elif kind == 'distinct_many_to_one':
        offsets = np.array([(x, y, 0.) for x in np.linspace(-0.00001, 0.00001, 5)
                            for y in np.linspace(-0.00001, 0.00001, 4)])
        points = np.vstack([offsets + [x, y, 0.034]
                            for x in (-0.04, 0.04) for y in (-0.025, 0.025)])
    else:
        points = np.column_stack((np.linspace(-0.04, 0.04, 80), np.zeros(80), np.full(80, 0.034)))
        reference = dataclasses.replace(reference, points_base=points)
    moving = dataclasses.replace(moving, points_base=points)
    original = fused_surface_from_view(reference)
    result, fused = append_registered_view(original, reference, moving)
    assert not result.ok
    assert result.inlier_count < 80
    assert fused is original


def test_repeated_input_samples_cannot_inflate_authoritative_metrics():
    reference, moving = registered_pair()
    baseline, surface = append_registered_view(fused_surface_from_view(reference), reference, moving)
    repeated = dataclasses.replace(moving, points_base=np.repeat(moving.points_base, 3, axis=0))
    result, repeated_surface = append_registered_view(fused_surface_from_view(reference), reference, repeated)
    assert baseline.ok and result.ok
    assert result.inlier_count == baseline.inlier_count
    assert result.overlap_fraction == baseline.overlap_fraction
    np.testing.assert_array_equal(result.transform_base, baseline.transform_base)
    np.testing.assert_array_equal(repeated_surface.points_base, surface.points_base)
    np.testing.assert_array_equal(repeated_surface.view_indices, surface.view_indices)


@pytest.mark.parametrize('iterations', [1, 2])
def test_iteration_budget_is_an_exact_integer_limit(monkeypatch, iterations):
    from alicia_flexible_grasp.vision import multiview_surface as module
    reference, moving = registered_pair()
    moving = dataclasses.replace(moving, points_base=transform_points_inverse(reference.points_base[:100]))
    calls = []
    rigid_update = module._rigid_update_about_support

    def counted_update(*args):
        calls.append(1)
        return rigid_update(*args)

    monkeypatch.setattr(module, '_rigid_update_about_support', counted_update)
    result = register_surface_view(reference, moving, RegistrationConfig(maximum_iterations=iterations))
    assert result.ok
    assert len(calls) == iterations
    with pytest.raises(ValueError):
        RegistrationConfig(maximum_iterations=np.nextafter(float(iterations), np.inf))
