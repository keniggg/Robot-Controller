"""Fixed hypotheses retain the original registration and fusion contract.

The GEN38 comparison imports its verified, unmodified local baseline module.
No live registration, ROS, image acquisition or motion is used.
"""
from dataclasses import asdict, fields, replace
import hashlib
import importlib.util
import math
from pathlib import Path
import sys

import numpy as np
import pytest

from alicia_flexible_grasp.vision import multiview_surface as surface
from test_multiview_surface import (registered_pair, rectangular_prism_views,
    transform_points_inverse, view)


@pytest.fixture(scope='module')
def gen38():
    work = Path(__file__).resolve().parents[3]
    relative = 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/multiview_surface.py'
    path = work/'.runtime/gen38_baseline_source'/relative
    assert hashlib.sha256(path.read_bytes()).hexdigest() == (
        '369a2947114c6c3953c439a7937cf26971e337c015e65abf96825720bc5c1ba8')
    name = 'alicia_flexible_grasp.vision._gen38_registration_test_baseline'
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def _legacy_view(module, value):
    return module.SurfaceView(identity=value.identity, stamp_ns=value.stamp_ns,
        points_base=value.points_base, support_normal_base=value.support_normal_base,
        support_offset_m=value.support_offset_m, edge_clearance_px=value.edge_clearance_px)


def _assert_result_exact(actual, expected):
    for field in fields(actual):
        left, right = getattr(actual, field.name), getattr(expected, field.name)
        if isinstance(left, np.ndarray):
            assert np.array_equal(left, right), field.name
        elif field.name == 'support_consistency_audit':
            assert (None if left is None else asdict(left)) == (
                None if right is None else asdict(right))
        else:
            assert left == right, field.name


def _assert_surface_exact(actual, expected):
    assert actual.identity == expected.identity
    assert actual.view_stamps_ns == expected.view_stamps_ns
    assert actual.view_sources == expected.view_sources
    np.testing.assert_array_equal(actual.points_base, expected.points_base)
    np.testing.assert_array_equal(actual.view_indices, expected.view_indices)


def _rigid(yaw_deg=0., translation=(0., 0., 0.)):
    angle = math.radians(yaw_deg)
    T = np.eye(4)
    T[:3, :3] = [[math.cos(angle), -math.sin(angle), 0.],
                 [math.sin(angle), math.cos(angle), 0.], [0., 0., 1.]]
    T[:3, 3] = translation
    return T


def _case(name):
    reference, moving = registered_pair()
    config = surface.RegistrationConfig()
    if name == 'identity':
        moving = replace(moving, points_base=reference.points_base)
    elif name == 'identity_mismatch':
        moving = replace(moving, identity=type(moving.identity).from_stream(3, 88))
    elif name == 'stamp':
        moving = replace(moving, stamp_ns=reference.stamp_ns)
    elif name == 'support_normal':
        angle = math.radians(5.)
        moving = replace(moving, support_normal_base=[0., math.sin(angle), math.cos(angle)])
    elif name == 'support_offset':
        moving = replace(moving, support_offset_m=.0071)
    elif name == 'warning_support_offset':
        moving = replace(moving, support_offset_m=.0055)
    elif name == 'inliers':
        moving = replace(moving, points_base=transform_points_inverse(reference.points_base[:79]))
    elif name == 'rmse':
        points = reference.points_base[:120].copy()
        points[::2, 2] += .005
        points[1::2, 2] -= .005
        moving = replace(moving, points_base=points)
    elif name == 'degenerate':
        line = np.column_stack((np.linspace(-.04, .04, 120), np.zeros(120), np.full(120, .034)))
        reference, moving = replace(reference, points_base=line), replace(moving, points_base=line)
    elif name == 'strict_yaw':
        config = replace(config, maximum_yaw_deg=1.)
    elif name == 'strict_displacement':
        config = replace(config, maximum_translation_m=.001)
    elif name == 'duplicates':
        reference = replace(reference, points_base=np.r_[reference.points_base[::-1], reference.points_base[:40]])
        moving = replace(moving, points_base=np.r_[moving.points_base[::-1], moving.points_base[:40]])
    elif name == 'changed_base_origin':
        z = _rigid(37.)[:3, :3]; angle = math.radians(11.)
        y = np.asarray([[math.cos(angle), 0., math.sin(angle)], [0., 1., 0.],
                        [-math.sin(angle), 0., math.cos(angle)]])
        R, t = z @ y, np.asarray([.7, -.4, .2])
        def transformed(value):
            normal = R @ value.support_normal_base
            return replace(value, points_base=value.points_base @ R.T+t,
                support_normal_base=normal, support_offset_m=value.support_offset_m-float(normal @ t))
        reference, moving = transformed(reference), transformed(moving)
    return reference, moving, config


@pytest.mark.parametrize('case', ['registered', 'identity', 'identity_mismatch', 'stamp',
    'support_normal', 'support_offset', 'warning_support_offset', 'inliers', 'rmse',
    'degenerate', 'strict_yaw', 'strict_displacement', 'duplicates', 'changed_base_origin'])
def test_default_icp_result_matches_gen38_every_field_exactly(gen38, case):
    reference, moving, config = _case(case)
    expected = gen38.register_surface_view(_legacy_view(gen38, reference),
        _legacy_view(gen38, moving), gen38.RegistrationConfig(**asdict(config)))
    actual = surface.register_surface_view(reference, moving, config)
    _assert_result_exact(actual, expected)


@pytest.mark.parametrize('case', ['registered', 'duplicates', 'support_offset'])
def test_default_append_and_explicit_none_keep_gen38_fusion_exact(gen38, case):
    reference, moving, config = _case(case)
    old_reference, old_moving = _legacy_view(gen38, reference), _legacy_view(gen38, moving)
    expected_result, expected_surface = gen38.append_registered_view(
        gen38.fused_surface_from_view(old_reference), old_reference, old_moving,
        gen38.RegistrationConfig(**asdict(config)))
    current = surface.fused_surface_from_view(reference)
    for kwargs in ({}, {'transform_base': None}):
        actual_result, actual_surface = surface.append_registered_view(
            current, reference, moving, config, **kwargs)
        _assert_result_exact(actual_result, expected_result)
        _assert_surface_exact(actual_surface, expected_surface)


def test_optimizer_final_acceptance_uses_shared_evaluator(monkeypatch):
    reference, moving = registered_pair()
    calls = []
    evaluate = surface.evaluate_registration_transform
    def watched(*args, **kwargs):
        calls.append(np.asarray(args[2]).copy())
        return evaluate(*args, **kwargs)
    monkeypatch.setattr(surface, 'evaluate_registration_transform', watched)
    result = surface.register_surface_view(reference, moving)
    assert result.ok and len(calls) == 1
    np.testing.assert_array_equal(calls[0], result.transform_base)


def test_optimizer_intermediate_degenerate_rejection_is_preserved(monkeypatch):
    reference, moving, config = _case('degenerate')
    def forbidden(*args, **kwargs):
        raise AssertionError('intermediate degenerate input reached final evaluator')
    monkeypatch.setattr(surface, 'evaluate_registration_transform', forbidden)
    result = surface.register_surface_view(reference, moving, config)
    assert not result.ok and result.code == 'DEGENERATE_SUPPORT'


@pytest.mark.parametrize('case,code', [('identity_mismatch', 'IDENTITY_MISMATCH'),
    ('stamp', 'STAMP_NOT_MONOTONIC'), ('support_normal', 'SUPPORT_NORMAL_MISMATCH'),
    ('support_offset', 'SUPPORT_OFFSET_MISMATCH')])
def test_fixed_metadata_and_support_failures_keep_original_semantics(gen38, case, code):
    reference, moving, config = _case(case)
    expected = gen38.register_surface_view(_legacy_view(gen38, reference),
        _legacy_view(gen38, moving), gen38.RegistrationConfig(**asdict(config)))
    result = surface.evaluate_registration_transform(reference, moving, np.eye(4), config)
    assert not result.ok and result.code == code
    _assert_result_exact(result, expected)


@pytest.mark.parametrize('case,code', [('inliers', 'INLIERS_INSUFFICIENT'),
    ('overlap', 'OVERLAP_INSUFFICIENT'), ('rmse', 'RMSE_EXCEEDED'),
    ('yaw', 'YAW_BOUND_EXCEEDED'), ('displacement', 'TRANSLATION_BOUND_EXCEEDED'),
    ('degenerate', 'DEGENERATE_SUPPORT')])
def test_fixed_transform_must_pass_each_original_geometric_gate(case, code):
    points, _ = rectangular_prism_views()
    reference, moving = view(points, 1_000_000_000), view(points, 1_100_000_000)
    config, T = surface.RegistrationConfig(), np.eye(4)
    if case == 'inliers':
        moving = replace(moving, points_base=points[:79])
    elif case == 'overlap':
        outliers = np.column_stack((np.linspace(.2, .3, 110), np.linspace(.2, .4, 110), np.full(110, .034)))
        moving = replace(moving, points_base=np.r_[points[:29], outliers])
        config = replace(config, minimum_inliers=20)
    elif case == 'rmse':
        top = points[np.isclose(points[:, 2], .034)]
        reference = replace(reference, points_base=top)
        moving = replace(moving, points_base=top+np.asarray([0., 0., .0045]))
    elif case == 'yaw':
        T = _rigid(10.5)
        moving = replace(moving, points_base=(points-T[:3, 3]) @ T[:3, :3])
    elif case == 'displacement':
        T = _rigid(translation=(0., 0., .026))
        moving = replace(moving, points_base=points-T[:3, 3])
    else:
        reference, moving, config = _case('degenerate')
    result = surface.evaluate_registration_transform(reference, moving, T, config)
    assert not result.ok and result.code == code


def test_many_nearby_samples_cannot_vote_as_unique_correspondences():
    points, _ = rectangular_prism_views()
    anchors = points[[0, 12, 208]]
    jitter = np.asarray([[.00001*i, .000001*i, 0.] for i in range(40)])
    moving_points = np.concatenate([point+jitter for point in anchors])
    result = surface.evaluate_registration_transform(view(points, 1_000_000_000),
        view(moving_points, 1_100_000_000), np.eye(4))
    assert not result.ok and result.code == 'INLIERS_INSUFFICIENT'
    assert result.inlier_count == 3


@pytest.mark.parametrize('bad', [np.eye(3), np.diag([2., 1., 1., 1.]),
    np.diag([-1., 1., 1., 1.]), np.full((4, 4), np.nan),
    np.asarray([[1., 0., 0., 0.], [0., 1., 0., 0.], [0., 0., 1., 0.], [0., 0., 1., 1.]])])
def test_invalid_fixed_rigid_transform_cannot_reach_fusion(bad):
    reference, moving = registered_pair()
    before = surface.fused_surface_from_view(reference)
    with pytest.raises(ValueError):
        surface.evaluate_registration_transform(reference, moving, bad)
    with pytest.raises(ValueError):
        surface.append_registered_view(before, reference, moving, transform_base=bad)
    assert before.view_stamps_ns == (reference.stamp_ns,)


def test_fixed_append_revalidates_and_never_runs_optimizer(monkeypatch):
    reference, moving = registered_pair()
    before = surface.fused_surface_from_view(reference)
    def forbidden(*args, **kwargs):
        raise AssertionError('fixed hypothesis must not be refitted')
    monkeypatch.setattr(surface, 'register_surface_view', forbidden)
    T = _rigid(3., (.003, -.002, .001))
    result, fused = surface.append_registered_view(before, reference, moving, transform_base=T)
    assert result.ok and fused is not before
    np.testing.assert_array_equal(result.transform_base, T)
    assert fused.view_stamps_ns == (reference.stamp_ns, moving.stamp_ns)
    assert all(set(row) <= {0, 1} for row in fused.view_sources)


def test_fixed_append_does_not_trust_a_previously_successful_result(monkeypatch):
    reference, moving = registered_pair()
    previous = surface.register_surface_view(reference, moving)
    assert previous.ok
    before = surface.fused_surface_from_view(reference)
    with pytest.raises((TypeError, ValueError)):
        surface.append_registered_view(before, reference, moving, transform_base=previous)
    def no_correspondences(reference_points, moving_points, T, config):
        return moving_points.copy(), np.zeros(len(moving_points), dtype=int), np.ones(len(moving_points)), np.zeros(len(moving_points), bool)
    monkeypatch.setattr(surface, '_registration_correspondences', no_correspondences)
    result, after = surface.append_registered_view(before, reference, moving,
        transform_base=previous.transform_base)
    assert not result.ok and result.code == 'DEGENERATE_SUPPORT'
    assert after is before


def test_fixed_identity_remains_identity_and_clipping_adds_no_new_gate():
    points, _ = rectangular_prism_views()
    reference, moving = view(points, 1_000_000_000), view(points, 1_100_000_000, edge=0)
    result = surface.evaluate_registration_transform(reference, moving, np.eye(4))
    assert result.ok and moving.is_clipped
    assert result.inlier_count == len(np.unique(points, axis=0)) and result.rmse_m == 0.
    np.testing.assert_array_equal(result.transform_base, np.eye(4))
