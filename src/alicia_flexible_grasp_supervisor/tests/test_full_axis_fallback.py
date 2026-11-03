"""Isolated candidate checks: original assets and explicit all-vertex oracles."""
from dataclasses import replace
import gzip
import hashlib
from itertools import product
import json
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from alicia_flexible_grasp.robot import finger_cad_separation as cad


FIXTURE = Path(__file__).parent / 'fixtures/rec3_full_axis_fallback.json.gz'
RAW_FIXTURE = FIXTURE.read_bytes()
assert hashlib.sha256(RAW_FIXTURE).hexdigest() == '356980622e9c3c4b862b6ae8fdd1c9369150a50613400f0222920cd01fa43d9b'
DATA = json.loads(gzip.decompress(RAW_FIXTURE))
ROWS = DATA['rows']
POSITIVES = [row for row in ROWS if row['best_full_axis_certificate']['separated']]
UNKNOWN = [row for row in ROWS if not row['best_full_axis_certificate']['separated']]
assert len(ROWS) == 14 and len(POSITIVES) == 3 and len(UNKNOWN) == 11
XML = DATA['robot_description_xml']
assert hashlib.sha256(XML.encode()).hexdigest() == DATA['robot_description_sha256']


@pytest.fixture
def model():
    result = cad.load_finger_cad_model(XML)
    assert result is not None
    return result


def args(row):
    scene = row['target_scene']
    return dict(body=row['sample']['body'], opening=.04980000000000001,
                T_base_tool=np.array(row['T_base_tool0']),
                target_center=np.array(scene['center']),
                target_rotation=np.array(scene['rotation']), target_size=np.array(scene['size']))


def assert_all_vertex_certificate(model, kwargs, result):
    """Independent explicit Minkowski points and eight OBB corners, in base."""
    assert result['separated'] and result['gap_m'] > 1e-9
    transform = np.array(kwargs['T_base_tool'])
    axis = np.array(result['axis_base'])
    assert np.linalg.norm(axis) == pytest.approx(1., abs=2e-14)
    padding = .0006 + kwargs.get('displacement_m', 0.)
    corners = padding * np.array(list(product((-1., 1.), repeat=3)))
    points = np.concatenate([
        part.vertices_at_zero + kwargs['opening'] * part.velocity_per_opening
        for part in model._body_components(kwargs['body'])])
    explicit = (points[:, None, :] + corners[None, :, :]).reshape(-1, 3)
    finger_world = explicit @ transform[:3, :3].T + transform[:3, 3]
    target_local = np.array(list(product((-1., 1.), repeat=3))) * kwargs['target_size'] / 2.
    target_world = target_local @ kwargs['target_rotation'].T + kwargs['target_center']
    finger_projection = (finger_world - transform[:3, 3]) @ axis
    target_projection = (target_world - transform[:3, 3]) @ axis
    fi = [finger_projection.min(), finger_projection.max()]
    ti = [target_projection.min(), target_projection.max()]
    assert result['finger_projection_m'] == pytest.approx(fi, abs=3e-13)
    assert result['target_projection_m'] == pytest.approx(ti, abs=3e-13)
    gap = max(ti[0] - fi[1], fi[0] - ti[1]) - result['numerical_gap_reserve_m']
    assert result['gap_m'] == pytest.approx(gap, abs=3e-13)
    assert result['padding_each_tool_axis_m'] == .0006
    assert result['displacement_each_tool_axis_m'] == kwargs.get('displacement_m', 0.)
    assert result['execution_authorized'] is False


@pytest.mark.parametrize('index', range(3))
def test_rec3_lateral_missing_axis_is_now_proved_by_all_original_vertices(model, index):
    row = POSITIVES[index]
    kwargs = args(row)
    result = model.separation(**kwargs)
    assert result['separated'], result
    assert result['axis_policy'] == 'padded_hull_axes_full_original_vertex_recheck'
    assert_all_vertex_certificate(model, kwargs, result)
    assert result['gap_m'] == pytest.approx(row['best_full_axis_certificate']['gap_m'], abs=2e-11)


@pytest.mark.parametrize('index', range(11))
def test_other_fixed_failures_remain_unknown_not_physical_collision(model, index):
    result = model.separation(**args(UNKNOWN[index]))
    assert result['separated'] is False and result['gap_m'] == 0.
    assert result['status'] == 'no_sufficient_axis'


@pytest.mark.parametrize('rotation_vector,translation', [([0., 0., 0.], [0., 0., 0.]),
    ([.4, -.2, .7], [.2, -.3, .1]), ([-.7, .3, -.8], [-.3, .2, -.2])])
@pytest.mark.parametrize('size_factor', [.6, .85, 1.])
@pytest.mark.parametrize('index', range(3))
def test_positive_proof_generalizes_rigid_world_frame_and_object_size(model, index, size_factor,
                                                                    rotation_vector, translation):
    kwargs = args(POSITIVES[index])
    rr = Rotation.from_rotvec(rotation_vector)
    rr = (getattr(rr, 'as_matrix', None) or rr.as_dcm)()
    tt = np.array(translation)
    transform = kwargs['T_base_tool'].copy()
    transform[:3, :3] = rr @ transform[:3, :3]
    transform[:3, 3] = rr @ transform[:3, 3] + tt
    kwargs.update(T_base_tool=transform, target_center=rr @ kwargs['target_center'] + tt,
                  target_rotation=rr @ kwargs['target_rotation'], target_size=kwargs['target_size'] * size_factor)
    result = model.separation(**kwargs)
    assert_all_vertex_certificate(model, kwargs, result)


@pytest.mark.parametrize('opening', [0., .01, .023, .049, .0498, .05])
@pytest.mark.parametrize('body', ['left_finger', 'right_finger'])
def test_target_contains_an_actual_vertex_cannot_become_separated(model, opening, body):
    part = model._body_components(body)[0]
    point = part.vertices_at_zero[321] + opening * part.velocity_per_opening
    result = model.separation(body, opening, np.eye(4), point, np.eye(3), [.0004] * 3)
    assert result['separated'] is False and result['gap_m'] == 0.


@pytest.mark.parametrize('opening', [0., .01, .023, .049, .0498, .05])
@pytest.mark.parametrize('index', range(3))
def test_opening_mapping_and_all_positive_axes_are_reverified(model, opening, index):
    kwargs = args(POSITIVES[index]); kwargs['opening'] = opening
    result = model.separation(**kwargs)
    if result['separated']:
        assert_all_vertex_certificate(model, kwargs, result)
    else:
        assert result['status'] == 'no_sufficient_axis'


@pytest.mark.parametrize('index', range(3))
def test_interval_displacement_never_shrinks_padding_or_reuses_a_verdict(model, index):
    kwargs = args(POSITIVES[index]); previous = float('inf')
    for displacement in (0., .0001, .0006, .001, .003, .01):
        kwargs['displacement_m'] = displacement
        result = model.separation(**kwargs)
        assert result['gap_m'] <= previous + 2e-12
        previous = result['gap_m']
        if result['separated']:
            assert_all_vertex_certificate(model, kwargs, result)
    assert previous == 0.


def test_old_fifteen_axis_positive_never_calls_hull(model, monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError('fast path invoked full-axis hull')
    monkeypatch.setattr(cad, '_full_axis_features', forbidden)
    result = model.separation('left_finger', .0498, np.eye(4), [1., 0., 0.], np.eye(3), [.02] * 3)
    assert result['separated'] and 'axis_policy' not in result


@pytest.mark.parametrize('error', [ImportError, RuntimeError, ValueError, MemoryError])
def test_missing_or_broken_optional_hull_retains_original_unknown(model, monkeypatch, error):
    def broken(*args):
        raise error('synthetic unavailable optional hull')
    monkeypatch.setattr(cad, '_full_axis_features', broken)
    result = model.separation(**args(POSITIVES[0]))
    assert not result['separated'] and result['status'] == 'no_sufficient_axis'
    assert result['full_axis_fallback_unavailable'] == error.__name__


def test_unverified_constructed_model_cannot_use_new_full_axis_fallback(model):
    unverified = replace(model)
    assert not cad.validated_model_matches(unverified, model._robot_description_sha256)
    result = unverified.separation(**args(POSITIVES[0]))
    assert not result['separated'] and result['status'] == 'no_sufficient_axis'


def test_false_hull_screen_never_substitutes_for_all_original_vertices(model, monkeypatch):
    part = model._body_components('left_finger')[0]
    point = part.vertices_at_zero[321] + .0498 * part.velocity_per_opening
    monkeypatch.setattr(cad, '_full_axis_features', lambda *unused: (
        np.array([[2., 2., 2.]]), np.eye(3), np.eye(3)))
    result = model.separation('left_finger', .0498, np.eye(4), point, np.eye(3), [.0004] * 3)
    assert result['separated'] is False and result['gap_m'] == 0.


def test_shape_cache_is_bounded_immutable_and_uses_exact_opening(model):
    with cad._FULL_AXIS_CACHE_LOCK:
        cad._FULL_AXIS_CACHE.clear()
    a = cad._full_axis_features(model, 'left_finger', .0498)
    assert cad._full_axis_features(model, 'left_finger', .0498) is a
    b = cad._full_axis_features(model, 'left_finger', float(np.nextafter(.0498, 0.)))
    assert b is not a
    other_body = cad._full_axis_features(model, 'right_finger', .0498)
    assert other_body is not a
    assert not np.array_equal(other_body[0], a[0])
    for value in a:
        assert value.flags.writeable is False
        with pytest.raises(ValueError): value.setflags(write=True)
    for index in range(20):
        cad._full_axis_features(model, 'left_finger', .04 + index * .0001)
    assert len(cad._FULL_AXIS_CACHE) == cad._FULL_AXIS_CACHE_MAXSIZE == 16


def test_same_cached_shape_rechecks_changed_pose_and_target(model):
    kwargs = args(POSITIVES[0]); first = model.separation(**kwargs)
    assert first['separated']
    part = model._body_components(kwargs['body'])[0]
    point = part.vertices_at_zero[321] + kwargs['opening'] * part.velocity_per_opening
    changed = kwargs['T_base_tool'].copy()
    changed[:3, 3] = kwargs['target_center'] - changed[:3, :3] @ point
    kwargs['T_base_tool'] = changed
    second = model.separation(**kwargs)
    assert second['separated'] is False
