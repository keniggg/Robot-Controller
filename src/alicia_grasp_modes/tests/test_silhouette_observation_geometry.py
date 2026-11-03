"""Model-union containment never manufactures measured contact evidence."""
from dataclasses import replace
import hashlib
import itertools
import json

import numpy as np
import pytest

from alicia_flexible_grasp.vision.object_geometry import GeometryEstimate
from alicia_grasp_modes.silhouette_observation_geometry import conservative_silhouette_observation_geometry
from alicia_grasp_modes.tabletop import Config


def geometry(*, center=(0., 0., .012), size=(.06, .042, .024), yaw=0., tilt=0., height=0., points=None):
    a, b = np.deg2rad([yaw, tilt])
    rz = np.array([[np.cos(a), -np.sin(a), 0.], [np.sin(a), np.cos(a), 0.], [0., 0., 1.]])
    rx = np.array([[1., 0., 0.], [0., np.cos(b), -np.sin(b)], [0., np.sin(b), np.cos(b)]])
    axes = rz @ rx
    normal = axes[:, 2]
    if points is None:
        points = np.array([[.001, -.001, .021], [.004, .002, .021], [-.003, .005, .022]])
    return GeometryEstimate(True, '', '', np.array(center), axes, np.array(size), normal,
        -float(normal @ [0., 0., height]), .96, points, 'instance_mask')


def combine(original, fresh, **kwargs):
    args = dict(original_source_stamp_ns=10000000001, fresh_source_stamp_ns=10100000002,
        max_size_m=.6, max_height_m=.5)
    args.update(kwargs)
    return conservative_silhouette_observation_geometry(original, fresh, **args)


def corners(estimate):
    signs = np.array(tuple(itertools.product((-1., 1.), repeat=3)))
    return estimate.center_base + (signs * .5 * estimate.size_xyz_m) @ estimate.axes_base.T


def assert_contains(box, points):
    coordinates = (points - box.center_base) @ box.axes_base
    assert np.max(np.abs(coordinates) - .5 * box.size_xyz_m) <= 1e-12


def test_rotated_and_shifted_fresh_box_retains_both_full_boxes_without_point_fabrication():
    old = geometry(size=(.07, .051, .024))
    fresh = geometry(center=(.006, -.002, .013), size=(.059, .042, .022), yaw=31.,
        points=np.array([[.005, .002, .021], [.004, -.006, .021]]))
    before_old, before_fresh = corners(old).copy(), corners(fresh).copy()
    assert np.max(np.abs((before_old-fresh.center_base) @ fresh.axes_base)-fresh.size_xyz_m*.5) > .005
    result, audit = combine(old, fresh)
    assert result is not fresh and result is not old
    assert_contains(result, before_old)
    assert_contains(result, before_fresh)
    np.testing.assert_array_equal(result.axes_base, fresh.axes_base)
    np.testing.assert_array_equal(result.support_normal_base, fresh.support_normal_base)
    np.testing.assert_array_equal(result.object_points_base, fresh.object_points_base)
    assert len(result.object_points_base) == 2
    assert not np.shares_memory(result.object_points_base, fresh.object_points_base)
    assert result.support_offset_m == fresh.support_offset_m
    assert result.support_inlier_ratio == fresh.support_inlier_ratio
    assert result.source_mode == fresh.source_mode
    np.testing.assert_array_equal(corners(old), before_old)
    np.testing.assert_array_equal(corners(fresh), before_fresh)
    for values in (result.center_base, result.axes_base, result.size_xyz_m, result.object_points_base):
        assert not values.flags.writeable
    assert audit['containment']['original_corners_preserved'] is True
    assert audit['containment']['fresh_corners_preserved'] is True
    assert audit['measured_points'] == dict(source='fresh_measurement_only', count=2,
        sha256=hashlib.sha256(fresh.object_points_base.astype('<f8').tobytes()).hexdigest(),
        synthetic_points_added=0)
    assert audit['diagnostic_observation_only'] is True
    assert audit['executable_contact_geometry'] is audit['motion_authorized'] is False
    assert audit['support_consistency']['perception_fixed_anchor_gate_replaced'] is False
    assert audit['support_consistency']['footprint_kind'] == 'projected_expanded_collision_envelope_not_measured_support'
    json.dumps(audit, allow_nan=False)


@pytest.mark.parametrize('yaw', [-180., -90., -45., -17., 0., 12., 60., 180.])
@pytest.mark.parametrize('shift', [(0., 0., 0.), (.012, -.003, .001)])
def test_all_corners_and_interior_points_survive_axes_changes(yaw, shift):
    old = geometry()
    fresh = geometry(center=np.array(old.center_base)+shift, yaw=yaw)
    result, _ = combine(old, fresh)
    for box in (old, fresh):
        grid = np.array(tuple(itertools.product(np.linspace(-.5, .5, 5), repeat=3)))
        points = box.center_base + (grid * box.size_xyz_m) @ box.axes_base.T
        assert_contains(result, points)


def test_same_box_retains_dimensions_and_audit_hashes_are_deterministic():
    old, fresh = geometry(), geometry()
    result, audit = combine(old, fresh)
    again, repeated = combine(old, fresh)
    np.testing.assert_allclose(result.size_xyz_m, fresh.size_xyz_m, atol=1e-15, rtol=0.)
    np.testing.assert_array_equal(result.center_base, fresh.center_base)
    assert audit == repeated
    assert audit['original']['geometry_sha256'] == audit['fresh']['geometry_sha256']
    moved = geometry(center=(.001, 0., .012))
    _, changed = combine(old, moved)
    assert changed['fresh']['geometry_sha256'] != audit['fresh']['geometry_sha256']
    assert changed['envelope']['geometry_sha256'] != audit['envelope']['geometry_sha256']


@pytest.mark.parametrize('side', ['original', 'fresh', 'both'])
def test_real_points_outside_input_obbs_expand_collision_bounds_without_fabricating_fresh_points(side):
    original, fresh = geometry(), geometry(yaw=23.)
    old_tail = [[-.065, .012, .031], [.045, -.018, -.002]]
    fresh_tail = [[.095, 0., .03], [.001, -.052, .019]]
    if side in ('original', 'both'):
        original = geometry(points=np.vstack((original.object_points_base, old_tail)))
    if side in ('fresh', 'both'):
        fresh = geometry(yaw=23., points=np.vstack((fresh.object_points_base, fresh_tail)))
    before = fresh.object_points_base.copy()
    result, audit = combine(original, fresh)
    assert_contains(result, original.object_points_base)
    assert_contains(result, fresh.object_points_base)
    assert_contains(result, corners(original))
    assert_contains(result, corners(fresh))
    np.testing.assert_array_equal(result.object_points_base, before)
    assert len(result.object_points_base) == len(fresh.object_points_base)
    assert audit['containment']['original_measured_points_preserved'] is True
    assert audit['containment']['fresh_measured_points_preserved'] is True
    assert audit['containment']['maximum_measured_point_excess_m'] <= 1e-12
    assert audit['collision_bound_sources'] == ['original_obb', 'fresh_obb',
        'original_measured_points', 'fresh_measured_points']
    assert audit['measured_points']['synthetic_points_added'] == 0
    assert audit['original']['object_points_sha256'] == hashlib.sha256(
        original.object_points_base.astype('<f8').tobytes()).hexdigest()


@pytest.mark.parametrize('side', ['original', 'fresh'])
def test_measured_tail_outside_trimmed_obb_cannot_bypass_combined_size_limit(side):
    estimates = dict(original=geometry(), fresh=geometry())
    estimates[side] = geometry(points=np.array([[.8, 0., .02]]))
    with pytest.raises(ValueError, match='COMBINED_SIZE_LIMIT_EXCEEDED'):
        combine(**estimates)


def test_support_plane_shift_is_checked_at_measured_tail_outside_original_obb():
    old = geometry(points=np.array([[0., .22, .015]]))
    fresh = geometry(tilt=2.)
    assert np.max(np.abs(corners(old) @ fresh.support_normal_base - corners(old)[:, 2])) < .004
    with pytest.raises(ValueError, match='support_plane_local_distance_inconsistent'):
        combine(old, fresh)


@pytest.mark.parametrize('height,accepted', [(.004, True), (.004001, True), (.006999, True), (.007, True), (.007001, False)])
def test_seven_mm_hard_support_translation_threshold(height, accepted):
    old, fresh = geometry(), geometry(height=height)
    if accepted:
        _, audit = combine(old, fresh)
        assert audit['support_consistency']['support_reference_max_distance_m'] == pytest.approx(height)
        assert audit['support_consistency']['support_reference_distance_warning'] is (height > .004)
    else:
        with pytest.raises(ValueError, match='support_plane_local_distance_inconsistent'):
            combine(old, fresh)


@pytest.mark.parametrize('tilt,accepted', [(3.99, True), (4., True), (4.01, False)])
def test_original_four_degree_normal_threshold_is_preserved(tilt, accepted):
    old, fresh = geometry(), geometry(tilt=tilt)
    if accepted:
        _, audit = combine(old, fresh)
        assert audit['support_consistency']['support_reference_angle_deg'] == pytest.approx(tilt)
    else:
        with pytest.raises(ValueError, match='support_plane_normal_inconsistent'):
            combine(old, fresh)


def test_plane_shift_is_checked_over_occupied_footprint_not_just_center():
    old = geometry(size=(.30, .30, .024))
    fresh = geometry(size=(.30, .30, .024), tilt=3.)
    assert abs(old.center_base @ (fresh.support_normal_base-old.support_normal_base)) < .004
    with pytest.raises(ValueError, match='support_plane_local_distance_inconsistent'):
        combine(old, fresh)


@pytest.mark.parametrize('config', [Config(support_max_local_shift_m=.0071),
    Config(support_max_normal_change_deg=4.1), Config(support_max_local_shift_m=0.),
    Config(support_max_normal_change_deg=float('nan')), {}])
def test_support_contract_cannot_be_relaxed_or_disabled(config):
    with pytest.raises(ValueError, match='SUPPORT_LIMIT_INVALID'):
        combine(geometry(), geometry(), support_config=config)


def test_caller_may_supply_stricter_existing_support_limit():
    with pytest.raises(ValueError, match='support_plane_local_distance_inconsistent'):
        combine(geometry(), geometry(height=.0025), support_config=Config(support_max_local_shift_m=.002))


@pytest.mark.parametrize('args', [dict(max_size_m=.07), dict(max_height_m=.025)])
def test_combined_extent_must_obey_original_limits_even_when_each_input_passes(args):
    old = geometry()
    fresh = geometry(center=(.02, 0., .015))
    with pytest.raises(ValueError, match='COMBINED_SIZE_LIMIT_EXCEEDED'):
        combine(old, fresh, **args)


def test_oversized_original_box_is_not_discarded_or_clipped_to_current_limit():
    with pytest.raises(ValueError, match='INPUT_SIZE_LIMIT_EXCEEDED'):
        combine(geometry(size=(.7, .04, .02)), geometry())


@pytest.mark.parametrize('args', [dict(original_source_stamp_ns=0), dict(original_source_stamp_ns=True),
    dict(fresh_source_stamp_ns='10100000002'), dict(fresh_source_stamp_ns=10000000001),
    dict(fresh_source_stamp_ns=9999999999)])
def test_exact_source_timestamps_require_a_later_fresh_observation(args):
    with pytest.raises(ValueError, match='SOURCE_ORDER_INVALID'):
        combine(geometry(), geometry(), **args)


@pytest.mark.parametrize('changes', [dict(ok=False), dict(size_xyz_m=[.06, 0., .02]),
    dict(center_base=[0., np.nan, .01]), dict(axes_base=np.diag([1., 1., -1.])),
    dict(axes_base=np.diag([1., .8, 1.])), dict(object_points_base=[]),
    dict(object_points_base=[[np.nan, 0., .01]]), dict(support_inlier_ratio=0.),
    dict(source_mode='synthetic')])
@pytest.mark.parametrize('side', ['original', 'fresh'])
def test_invalid_geometry_cannot_produce_executable_envelope(changes, side):
    estimates = dict(original=geometry(), fresh=geometry())
    estimates[side] = replace(estimates[side], **changes)
    with pytest.raises(ValueError, match='GEOMETRY_INVALID'):
        combine(**estimates)


@pytest.mark.parametrize('name', ['max_size_m', 'max_height_m'])
@pytest.mark.parametrize('value', [0., -1., float('nan'), float('inf'), True, '.6', None])
def test_invalid_size_limits_fail_closed(name, value):
    with pytest.raises(ValueError, match='SIZE_LIMIT_INVALID'):
        combine(geometry(), geometry(), **{name: value})
