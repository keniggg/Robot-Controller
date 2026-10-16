"""A small fitted axis is diagnostic evidence, never contact or motion authority."""
from dataclasses import FrozenInstanceError, replace
from copy import deepcopy
from types import SimpleNamespace

import numpy as np
import pytest

from test_object_geometry import geometry_module, estimate_object_geometry


def thin_scene():
    # Exact rendered plane and narrower visible top. No mocked fit or gates.
    # Dense pixels make the original point minimum independent of this axis.
    depth = np.full((120, 160), .400, dtype=np.float64)
    mask = np.zeros(depth.shape, np.uint8)
    mask[44:78, 30:130] = 255
    depth[mask > 0] = .370
    return dict(depth_raw=depth, target_depth_raw=np.where(mask, depth, 0.),
        object_mask=mask, bbox=(30, 44, 100, 34),
        intrinsics=SimpleNamespace(width=160, height=120, fx=3000., fy=3000., cx=80., cy=60.),
        depth_scale=1., T_base_camera=np.eye(4), source_mode='instance_mask',
        support_bbox_expand_ratio=.30, support_distance_threshold_m=.004,
        voxel_size_m=.0003, min_support_points=200, min_object_points=120,
        min_size_m=.005, max_size_m=.600, max_height_m=.500,
        previous_axes_base=None)


@pytest.fixture
def failed():
    estimate = estimate_object_geometry(**thin_scene())
    assert not estimate.ok, estimate
    assert estimate.observation_deficit is not None, estimate.failure_reason
    return estimate


def test_only_minimum_extent_failure_keeps_original_contact_failure(failed):
    evidence = failed.observation_deficit
    assert failed.failure_code == 'OBB_INVALID'
    assert 'below minimum 0.005000 m' in failed.failure_reason
    assert evidence.kind == 'contact_obb_minimum_extent_not_met'
    assert evidence.min_size_m == .005
    assert evidence.failed_axis_indices == (1,)
    assert evidence.axis_roles[1] == 'support_plane_short'
    assert evidence.size_xyz_m[1] < .005
    assert evidence.size_xyz_m[2] == pytest.approx(.03)
    assert not evidence.contact_geometry_authorized
    assert not evidence.observation_motion_authorized
    assert not evidence.complete_object_geometry
    # Existing failed-geometry consumers still see no usable contact box.
    assert np.count_nonzero(failed.size_xyz_m) == 0
    assert failed.object_points_base.shape == (0, 3)
    with pytest.raises(ValueError, match='cannot authorize contact'):
        replace(failed, ok=True)
    with pytest.raises(ValueError, match='valid geometry'):
        geometry_module.anchor_geometry_center_in_support_plane(failed, np.zeros(3), .01)


def test_evidence_preserves_measured_points_and_original_fit_without_padding(failed):
    evidence = failed.observation_deficit
    args = thin_scene()
    target, _uv = geometry_module.deproject_depth(args['target_depth_raw'],
        args['target_depth_raw'] > 0, args['intrinsics'], args['depth_scale'])
    np.testing.assert_array_equal(evidence.target_points_base, target)
    mask = geometry_module._support_context_mask(args['depth_raw'], args['object_mask'] > 0,
        args['bbox'], args['intrinsics'], args['depth_scale'], .30, 0.)
    support, _uv = geometry_module.deproject_depth(args['depth_raw'], mask,
        args['intrinsics'], args['depth_scale'])
    np.testing.assert_array_equal(evidence.support_points_base, support)
    # Test-only reference: a lower acceptance minimum cannot affect the fit.
    # Production retains the original .005 threshold and failed result.
    reference = estimate_object_geometry(**dict(args, min_size_m=.001))
    assert reference.ok and reference.observation_deficit is None
    for name in ('center_base', 'axes_base', 'size_xyz_m', 'object_points_base',
                 'support_normal_base', 'support_offset_m', 'support_inlier_ratio'):
        np.testing.assert_array_equal(getattr(evidence, name), getattr(reference, name))
    assert len(evidence.object_points_base) < len(evidence.target_points_base)


@pytest.mark.parametrize('name', ['center_base', 'axes_base', 'size_xyz_m',
    'support_normal_base', 'target_points_base', 'object_points_base',
    'support_points_base', 'T_base_camera'])
def test_evidence_arrays_cannot_be_written_or_reenabled(failed, name):
    values = getattr(failed.observation_deficit, name)
    with pytest.raises(ValueError):
        values.flat[0] = 20.
    with pytest.raises(ValueError):
        values.setflags(write=True)


def test_evidence_defensively_copies_inputs_and_fixed_rights(failed):
    original = failed.observation_deficit
    points = original.object_points_base.copy()
    copy = replace(original, object_points_base=points)
    points[:] = 999.
    np.testing.assert_array_equal(copy.object_points_base, original.object_points_base)
    with pytest.raises(FrozenInstanceError):
        copy.contact_geometry_authorized = True
    with pytest.raises(ValueError):
        replace(copy, observation_motion_authorized=True)
    frozen = deepcopy(copy)
    assert frozen is copy
    with pytest.raises(ValueError):
        frozen.object_points_base.setflags(write=True)


@pytest.mark.parametrize('fault', ['support_missing', 'support_nonfinite', 'target_missing',
    'raw_point_minimum', 'cleaned_point_minimum', 'degenerate', 'nonfinite_transform',
    'nonrigid_transform', 'nonrigid_previous_axes', 'invalid_intrinsics',
    'dimension_configuration', 'maximum_size', 'maximum_height', 'source_mode'])
def test_other_failures_never_receive_minimum_extent_evidence(fault):
    args = thin_scene()
    if fault == 'support_missing':
        args['depth_raw'] = np.where(args['object_mask'], args['depth_raw'], 0.)
    elif fault == 'support_nonfinite':
        args['depth_raw'] = np.where(args['object_mask'], args['depth_raw'], np.nan)
    elif fault == 'target_missing':
        args['target_depth_raw'][:] = 0.
    elif fault == 'raw_point_minimum':
        args['min_object_points'] = 4000
    elif fault == 'cleaned_point_minimum':
        args['voxel_size_m'] = .003
    elif fault == 'degenerate':
        args['target_depth_raw'][:, :79] = 0.
        args['target_depth_raw'][:, 80:] = 0.
        args['min_object_points'] = 3
    elif fault == 'nonfinite_transform':
        args['T_base_camera'][0, 3] = np.nan
    elif fault == 'nonrigid_transform':
        args['T_base_camera'][0, 0] = 2.
    elif fault == 'nonrigid_previous_axes':
        args['previous_axes_base'] = np.zeros((3, 3))
    elif fault == 'invalid_intrinsics':
        args['intrinsics'].fx = 0.
    elif fault == 'dimension_configuration':
        args['min_size_m'] = .7
    elif fault == 'maximum_size':
        args['max_size_m'] = .02
    elif fault == 'maximum_height':
        args['max_height_m'] = .02
    elif fault == 'source_mode':
        args['source_mode'] = 'unbound_rgb'
    result = estimate_object_geometry(**args)
    assert not result.ok
    assert result.observation_deficit is None


@pytest.mark.parametrize('mutation', [
    dict(center_base=[np.nan, 0., 0.]), dict(size_xyz_m=[.012, np.inf, .03]),
    dict(object_points_base=[[np.nan, 0., 0.]] * 120),
    dict(support_points_base=[[0., 0., np.inf]] * 200),
    dict(size_xyz_m=[.012, 0., .03]), dict(size_xyz_m=[.012, .006, .03]),
    dict(size_xyz_m=[.7, .004, .03]), dict(size_xyz_m=[.012, .004, .51]),
    dict(support_normal_base=[0., 0., 0.]), dict(axes_base=np.zeros((3, 3))),
    dict(support_inlier_ratio=.2), dict(min_object_points=True),
    dict(T_base_camera=np.zeros((4, 4))), dict(source_mode='unknown'),
    dict(min_size_m='0.005'), dict(support_distance_threshold_m=False),
])
def test_malformed_or_multiple_failure_evidence_rejected(failed, mutation):
    with pytest.raises(ValueError):
        replace(failed.observation_deficit, **mutation)


def test_successful_and_legacy_geometry_construction_have_no_deficit():
    args = thin_scene()
    args['intrinsics'].fy = 1500.
    result = estimate_object_geometry(**args)
    assert result.ok and result.observation_deficit is None
    positional = geometry_module.GeometryEstimate(True, '', '', np.zeros(3), np.eye(3),
        np.ones(3), np.array([0., 0., 1.]), 0., 1., np.zeros((3, 3)), 'instance_mask')
    assert positional.observation_deficit is None
