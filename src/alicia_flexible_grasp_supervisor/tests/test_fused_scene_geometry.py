"""Pure measured-surface enclosure: no ROS services or synthesized geometry."""
from dataclasses import FrozenInstanceError, replace
import itertools
import json
from pathlib import Path
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / 'src'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from alicia_flexible_grasp.vision.fused_scene_geometry import enclose_registered_surface
from alicia_flexible_grasp.vision.object_geometry import GeometryEstimate
from alicia_flexible_grasp.vision.multiview_surface import FusedTargetSurface
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity

IDENTITY = TargetTrackIdentity(7, 'g4-t7')


def geometry(center=(.2, -.3, .04), axes=None, size=(.06, .04, .03), points=None):
    axes = np.eye(3) if axes is None else np.asarray(axes)
    center = np.asarray(center)
    if points is None:
        points = np.array([[0., 0., 0.], [.02, .01, .005]]) @ axes.T + center
    return GeometryEstimate(True, '', '', center, axes, np.array(size),
        axes[:, 2], -.025, .9, points, 'instance_mask')


def surface(points, indices=None, stamps=(10, 20), identity=IDENTITY):
    points = np.asarray(points, dtype=float)
    if indices is None:
        indices = np.arange(len(points), dtype=int) % len(stamps)
    return FusedTargetSurface(identity, points, indices, stamps)


def corners(g):
    signs = np.asarray(list(itertools.product((-1., 1.), repeat=3)))
    return signs * (g.size_xyz_m / 2.) @ g.axes_base.T + g.center_base


def inside(points, g):
    return np.all(np.abs((points - g.center_base) @ g.axes_base)
                  <= g.size_xyz_m / 2. + 1e-12)


def test_historical_surface_encloses_original_box_with_tangent_midpoint():
    g = geometry()
    history = np.array([[.15, -.3, .04], [.2, -.265, .04], [.2, -.3, .068]])
    s = surface(history, [0, 0, 1])
    output, audit = enclose_registered_surface(g, s, expected_identity=IDENTITY)
    np.testing.assert_allclose(output.size_xyz_m, [.08, .055, .056])
    np.testing.assert_allclose(output.center_base, [.19, -.2925, .04])
    for key in ('axes_base', 'support_normal_base'):
        np.testing.assert_array_equal(getattr(output, key), getattr(g, key))
    assert output.support_offset_m == g.support_offset_m
    assert inside(corners(g), output) and inside(history, output)
    assert inside(g.object_points_base, output)
    assert audit['expanded'] and audit['expected_identity_checked']
    assert audit['target_epoch'] == 7 and audit['target_track_id'] == 'g4-t7'
    assert audit['view_stamps_ns'] == [10, 20] and audit['per_view_point_counts'] == [2, 1]
    assert audit['synthetic_points_added'] == 0 and not audit['contact_or_motion_authorized']
    assert len(output.object_points_base) == len(g.object_points_base) + len(history)
    json.dumps(audit, allow_nan=False)


def test_same_points_do_not_expand_or_duplicate_existing_measurements():
    g = geometry()
    output, audit = enclose_registered_surface(g, surface(g.object_points_base))
    np.testing.assert_array_equal(output.size_xyz_m, g.size_xyz_m)
    np.testing.assert_array_equal(output.object_points_base, g.object_points_base)
    assert not audit['expanded'] and audit['exact_duplicates_removed'] == len(g.object_points_base)
    assert not audit['expected_identity_checked']


@pytest.mark.parametrize('center', [(0., 0., 0.), (.4, -.2, .3), (-.7, 1.4, -.6)])
@pytest.mark.parametrize('angle', [0., .37, -1.2])
def test_enclosure_uses_original_rotated_axes_and_local_center(center, angle):
    c, s = np.cos(angle), np.sin(angle)
    axes = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
    g = geometry(center=center, axes=axes)
    local = np.array([[-.04, .015, .012], [.03, -.026, .004]])
    fused = surface(local @ axes.T + g.center_base)
    output, _ = enclose_registered_surface(g, fused)
    np.testing.assert_allclose(output.size_xyz_m, [.07, .046, .03])
    np.testing.assert_allclose(output.center_base, g.center_base + axes @ [-.005, -.003, 0.], atol=1e-15)
    np.testing.assert_array_equal(output.axes_base, axes)
    assert inside(corners(g), output) and inside(fused.points_base, output)


def test_raw_admitted_extrema_are_not_dropped_even_if_original_obb_was_percentile_fit():
    raw = np.array([[.2, -.3, .04], [.235, -.3, .04]])
    g = geometry(points=raw)
    output, _ = enclose_registered_surface(g, surface([[.2, -.3, .04]]))
    assert output.size_xyz_m[0] == pytest.approx(.065)
    assert inside(raw, output)
    np.testing.assert_array_equal(output.object_points_base, raw)


def test_no_corner_or_unobserved_surface_is_added_as_a_measured_point():
    g = geometry()
    s = surface([[.2, -.3, .04], [.19, -.31, .04]])
    output, _ = enclose_registered_surface(g, s)
    expected = {tuple(p) for p in np.vstack((g.object_points_base, s.points_base))}
    assert {tuple(p) for p in output.object_points_base} == expected
    assert not any(tuple(p) in expected for p in corners(g))


def test_inputs_and_returned_arrays_are_independent_and_readonly():
    g = geometry(); s = surface([[.15, -.3, .04]])
    before = (g.size_xyz_m.copy(), g.object_points_base.copy(), s.points_base.copy())
    output, audit = enclose_registered_surface(g, s)
    for array in (output.size_xyz_m, output.object_points_base, output.center_base):
        with pytest.raises(ValueError): array.flat[0] = 999.
    with pytest.raises(FrozenInstanceError): output.size_xyz_m = np.ones(3)
    audit['old_size_xyz_m'][0] = 999.
    np.testing.assert_array_equal(g.size_xyz_m, before[0])
    np.testing.assert_array_equal(g.object_points_base, before[1])
    np.testing.assert_array_equal(s.points_base, before[2])
    assert not np.shares_memory(output.object_points_base, g.object_points_base)
    assert not np.shares_memory(output.object_points_base, s.points_base)


@pytest.mark.parametrize('key,value', [
    ('ok', False), ('ok', 1), ('center_base', [np.nan, 0., 0.]),
    ('size_xyz_m', [.1, np.inf, .1]), ('size_xyz_m', [.1, -.01, .1]),
    ('axes_base', np.diag([1., 1., -1.])), ('axes_base', np.diag([1., 1., 1.001])),
    ('support_normal_base', [0., 0., 2.]), ('support_normal_base', [0., 1., 0.]),
    ('support_offset_m', np.nan),
    ('support_inlier_ratio', 1.01), ('object_points_base', [[0., np.inf, 0.]]),
    ('object_points_base', []), ('failure_code', 'BAD'), ('source_mode', ''),
])
def test_invalid_geometry_is_rejected_instead_of_repaired(key, value):
    g = replace(geometry(), **{key: value})
    with pytest.raises(ValueError): enclose_registered_surface(g, surface([[.2, -.3, .04]]))


@pytest.mark.parametrize('key,value', [
    ('identity', None), ('points_base', [[0., np.nan, 0.]]),
    ('view_indices', np.array([2])), ('view_indices', np.array([True])),
    ('view_stamps_ns', (20, 10)), ('view_stamps_ns', (0, 10)),
    ('view_sources', [()]), ('view_sources', [(0, 2)]), ('view_sources', [(1,)]),
])
def test_surface_fields_are_revalidated_even_if_dataclass_constructor_was_bypassed(key, value):
    s = surface([[.2, -.3, .04]])
    object.__setattr__(s, key, value)
    with pytest.raises(ValueError): enclose_registered_surface(geometry(), s)


@pytest.mark.parametrize('epoch,track_id', [(True, 'g1-t1'), (-1, 'g1-t1'), (3, 'not canonical')])
def test_identity_contents_are_revalidated(epoch, track_id):
    identity = TargetTrackIdentity(1, 'g1-t1')
    object.__setattr__(identity, 'epoch', epoch)
    object.__setattr__(identity, 'track_id', track_id)
    s = surface([[.2, -.3, .04]], identity=identity)
    with pytest.raises(ValueError): enclose_registered_surface(geometry(), s)


def test_foreign_valid_identity_cannot_borrow_snapshot_binding():
    with pytest.raises(ValueError, match='identity mismatch'):
        enclose_registered_surface(geometry(), surface([[.2, -.3, .04]]),
            expected_identity=TargetTrackIdentity(8, 'g4-t8'))


@pytest.mark.parametrize('point,kwargs', [
    ([.801, -.3, .04], {}), ([.2, -.3, .291], {}),
    ([.401, -.3, .04], {'max_size_m': .2}),
    ([.2, -.3, .141], {'max_height_m': .2}),
])
def test_output_exceeding_existing_dimension_policy_is_rejected(point, kwargs):
    with pytest.raises(ValueError, match='OVERSIZE'):
        enclose_registered_surface(geometry(), surface([point]), **kwargs)


def test_already_oversize_original_box_cannot_be_shrunk_into_compliance():
    with pytest.raises(ValueError, match='OVERSIZE'):
        enclose_registered_surface(geometry(size=(.61, .04, .03)), surface([[.2, -.3, .04]]))


@pytest.mark.parametrize('value', [True, '0.6', 0., -.1, np.nan, np.inf])
@pytest.mark.parametrize('key', ['max_size_m', 'max_height_m'])
def test_invalid_caller_bounds_fail_closed(key, value):
    with pytest.raises(ValueError):
        enclose_registered_surface(geometry(), surface([[.2, -.3, .04]]), **{key: value})


def test_audit_hashes_bind_coordinates_view_assignment_and_ordered_source_stamps():
    g = geometry(); s = surface([[.15, -.3, .04], [.2, -.29, .04]])
    _, a = enclose_registered_surface(g, s)
    _, b = enclose_registered_surface(g, s)
    assert a == b
    _, c = enclose_registered_surface(g, replace(s, view_indices=[1, 0], view_sources=((1,), (0,))))
    assert c['fused_points_sha256'] == a['fused_points_sha256']
    assert c['fused_view_indices_sha256'] != a['fused_view_indices_sha256']
    shifted = np.array(s.points_base); shifted[0, 0] += .001
    _, d = enclose_registered_surface(g, replace(s, points_base=shifted))
    assert d['fused_points_sha256'] != a['fused_points_sha256']
    assert d['enclosed_points_sha256'] != a['enclosed_points_sha256']


def test_shared_voxel_sources_are_retained_and_audited_without_synthetic_votes():
    g = geometry(); s = surface([[.15, -.3, .04], [.2, -.29, .04]])
    shared = replace(s, view_sources=((0, 1), (1,)))
    output, audit = enclose_registered_surface(g, shared)
    single, earlier = enclose_registered_surface(g, s)
    np.testing.assert_array_equal(output.object_points_base, single.object_points_base)
    assert audit['per_view_point_counts'] == [1, 2]
    assert audit['per_view_representative_point_counts'] == [1, 1]
    assert audit['fused_view_sources_sha256'] != earlier['fused_view_sources_sha256']
    assert audit['fused_points_sha256'] == earlier['fused_points_sha256']
    assert shared.view_sources == ((0, 1), (1,))


def test_full_3d_rotation_preserves_support_frame_and_original_corner_enclosure():
    yaw, pitch = .8, -.4
    rz = np.array([[np.cos(yaw), -np.sin(yaw), 0.],
                   [np.sin(yaw), np.cos(yaw), 0.], [0., 0., 1.]])
    ry = np.array([[np.cos(pitch), 0., np.sin(pitch)], [0., 1., 0.],
                   [-np.sin(pitch), 0., np.cos(pitch)]])
    axes = rz @ ry
    g = geometry(axes=axes)
    local = np.array([[.055, -.035, .022], [-.045, .015, -.028]])
    s = surface(local @ axes.T + g.center_base)
    output, audit = enclose_registered_surface(g, s)
    np.testing.assert_allclose(output.size_xyz_m, [.10, .055, .056])
    np.testing.assert_array_equal(output.support_normal_base, g.support_normal_base)
    assert output.support_offset_m == g.support_offset_m
    assert inside(corners(g), output) and inside(s.points_base, output)
    assert not audit['center_preserved'] and audit['support_plane_preserved']
    assert abs(float((output.center_base-g.center_base) @ g.support_normal_base)) < 1e-15


def test_finite_inputs_whose_coordinate_difference_overflows_are_rejected():
    g = geometry(center=(-1e308, -.3, .04), points=[[-1e308, -.3, .04]])
    s = surface([[1e308, -.3, .04]])
    with pytest.raises(ValueError, match='nonfinite enclosure'):
        enclose_registered_surface(g, s)


@pytest.mark.parametrize('which', ['geometry', 'surface'])
def test_untyped_partial_inputs_are_not_silently_accepted(which):
    from types import SimpleNamespace
    g = geometry(); s = surface([[.2, -.3, .04]])
    if which == 'geometry': g = SimpleNamespace(**g.__dict__)
    else: s = SimpleNamespace(**s.__dict__)
    with pytest.raises(ValueError): enclose_registered_surface(g, s)
