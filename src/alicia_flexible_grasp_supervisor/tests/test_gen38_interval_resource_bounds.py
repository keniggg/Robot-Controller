"""Exact bounded-pair arithmetic against the retained dense specification.

The dense reference is restricted to small controlled inputs. The real-record
fixture exercises arithmetic on its complete four stored response groups; it
does not qualify that recorded box or reinterpret its occupancy failure.
"""
import copy
import json
import math
import os
from pathlib import Path

import numpy as np
import pytest

from alicia_flexible_grasp.vision import cuboid_rgb_interval as interval
from alicia_flexible_grasp.vision import regular_box_inference as box
from test_gen31_oblique_box_independent import oblique_box_scene


def _dense_trig_range(x, y, lower, upper):
    candidates = [lower, upper]
    phase = math.atan2(y, x)
    for index in range(-3, 4):
        angle = phase+index*math.pi
        if lower <= angle <= upper:
            candidates.append(angle)
    values = [x*math.cos(angle)+y*math.sin(angle) for angle in candidates]
    return min(values), max(values)


def _dense_joint(groups, fixed_theta=None):
    """Pre-resource-fix dense all-pairs specification, not production code."""
    cov = []
    for row in groups:
        p = np.asarray(row['metric_points']); centered = p-p.mean(0)
        cov.append(centered.T @ centered/len(p))
    J = np.asarray([[0., -1.], [1., 0.]])
    _, eig = np.linalg.eigh(cov[1]+cov[3]+J.T @ (cov[0]+cov[2]) @ J)
    u = eig[:, 0]
    if u[0] < 0.:
        u = -u
    theta = math.atan2(u[1], u[0]) if fixed_theta is None else float(fixed_theta)
    rotation = np.asarray([[math.cos(theta), -math.sin(theta)],
                           [math.sin(theta), math.cos(theta)]])
    lower_angle, upper_angle = -math.radians(8.), math.radians(8.)
    for index, row in enumerate(groups):
        p = np.asarray(row['metric_points']); delta = row['strip_radius_m']
        differences = p[:, None, :]-p[None, :, :]
        length = np.linalg.norm(differences, axis=2)
        active = length > 2.*delta+1e-15
        dif, lengths = differences[active], length[active]
        if not len(lengths):
            raise ValueError('outer response family cannot constrain a common orientation')
        angle = np.arctan2(dif[:, 1], dif[:, 0])-(math.pi/2. if index % 2 else 0.)
        angle = (angle-theta+math.pi/2.) % math.pi-math.pi/2.+theta
        arc = np.arcsin(2.*delta/lengths)
        lower_angle = max(lower_angle, float(np.max(angle-arc)))
        upper_angle = min(upper_angle, float(np.min(angle+arc)))
    if not lower_angle <= theta <= upper_angle:
        raise ValueError('four complete mixed-plane lines have no common orthogonal interval')
    offsets = []
    for index, row in enumerate(groups):
        p = np.asarray(row['metric_points']) @ rotation[:, 1 if index % 2 == 0 else 0]
        offsets.append(float((p.max()+p.min())/2.))
    lo = np.asarray([offsets[3], offsets[0]])
    hi = np.asarray([offsets[1], offsets[2]])
    if np.any(hi-lo < .005) or np.any(hi-lo > .6):
        raise ValueError('mixed-plane rectangle extent invalid')
    mid = (lo+hi)/2.; center = rotation @ mid
    intervals, error = [], 0.
    for index, row in enumerate(groups):
        p = np.asarray(row['metric_points'])-center
        ac = 1 if index % 2 == 0 else 0
        extrema = [_dense_trig_range(v[0], v[1], lower_angle, upper_angle) if ac == 0 else
                   _dense_trig_range(v[1], -v[0], lower_angle, upper_angle) for v in p]
        low = max(v[0] for v in extrema)-row['strip_radius_m']
        high = min(v[1] for v in extrema)+row['strip_radius_m']
        nominal = offsets[index]-mid[ac]
        err = max(abs(low-nominal), abs(high-nominal))
        if low > high+1e-12:
            raise ValueError('current edge response strips have empty position intersection')
        error = max(error, err)
        intervals.append(dict(minimum_m=low, maximum_m=high, nominal_m=nominal, error_m=err))
    yaw = max(math.radians(.25), theta-lower_angle, upper_angle-theta)
    edge = max(.001, error*(math.cos(yaw)+math.sin(yaw)))
    return dict(rotation=rotation, lower=lo, upper=hi, theta=theta,
        common_yaw_interval_rad=[lower_angle, upper_angle], position_intervals=intervals,
        position_family_error_m=error, edge_uncertainty=edge, yaw_uncertainty=yaw)


def _dense_reference_joint(groups, lo, hi):
    result = _dense_joint(groups, fixed_theta=0.)
    errors = np.asarray([max(result['position_intervals'][i]['error_m'] for i in (1, 3)),
                         max(result['position_intervals'][i]['error_m'] for i in (0, 2))])
    shift = np.abs((result['lower']+result['upper']-lo-hi)/2.)
    size = np.abs(result['upper']-result['lower']-(hi-lo))/2.
    yaw = result['yaw_uncertainty']
    def support(x, y):
        angle = min(yaw, math.atan2(y, x))
        return x*math.cos(angle)+y*math.sin(angle)
    axis = np.asarray([max(.001, support(errors[i], errors[1-i])+shift[i]+
        support(size[i], size[1-i])) for i in (0, 1)])
    result.update(axis_uncertainty=axis, edge_uncertainty=float(axis.max()),
        axis_position_error_m=errors, nominal_reference_center_offset_m=shift,
        nominal_reference_half_size_offset_m=size)
    return result


def _assert_equivalent(actual, expected):
    assert set(actual) == set(expected)
    for key, value in expected.items():
        if isinstance(value, list) and value and isinstance(value[0], dict):
            assert len(actual[key]) == len(value)
            for row, ref in zip(actual[key], value):
                _assert_equivalent(row, ref)
        else:
            np.testing.assert_allclose(actual[key], value, rtol=0., atol=1e-14, err_msg=key)


def _random_groups(seed, large=False):
    rng = np.random.default_rng(seed)
    theta = math.radians(float(rng.uniform(-.4, .4)))
    R = np.asarray([[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]])
    size = rng.uniform([.045, .025], [.075, .045]); center = rng.uniform(-.15, .15, 2)
    rows = []
    for side in range(4):
        count = 513 if large and side == 0 else int(rng.integers(19, 83))
        ac = 1 if side % 2 == 0 else 0; al = 1-ac
        local = np.empty((count, 2))
        local[:, al] = np.linspace(-size[al]/2., size[al]/2., count)
        local[:, ac] = (-1. if side in (0, 3) else 1.)*size[ac]/2.
        local[:, ac] += rng.uniform(-.00003, .00003, count)
        rows.append(dict(metric_points=local @ R.T+center, strip_radius_m=.00045))
    return rows, center-size/2., center+size/2.


@pytest.mark.parametrize('seed', [0, 1, 7, 19, 101])
@pytest.mark.parametrize('large', [False, True])
def test_bounded_pairs_match_every_dense_joint_and_axis_budget_field(seed, large):
    groups, lo, hi = _random_groups(seed, large)
    _assert_equivalent(interval._joint(groups), _dense_joint(groups))
    _assert_equivalent(interval._joint(groups, fixed_theta=0.), _dense_joint(groups, fixed_theta=0.))
    _assert_equivalent(interval._reference_joint(groups, lo, hi), _dense_reference_joint(groups, lo, hi))


def test_complete_recorded_response_groups_keep_dense_result_without_requalifying_record():
    default = ('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/'
        'full_latest_gen36_restart_20261017T012037Z/verification/gen38_recorded_box_contact_r2/result.json')
    path = Path(os.environ.get('GEN38_RECORDED_BOX_RESULT', default))
    if not path.is_file():
        pytest.skip('recorded interval fixture unavailable; no source substitute generated')
    recorded = json.loads(path.read_text())
    assert recorded['box_qualified'] is False
    assert recorded['actual_production_code'] == 'BOX_TOP_CONTOUR_INCOMPLETE'
    value = recorded['audit']['cuboid_rgb_interval']
    groups = value['groups']; lo, hi = np.asarray(value['nominal_lower']), np.asarray(value['nominal_upper'])
    _assert_equivalent(interval._joint(groups), _dense_joint(groups))
    _assert_equivalent(interval._reference_joint(groups, lo, hi), _dense_reference_joint(groups, lo, hi))


@pytest.mark.parametrize('case', ['no_constraining_pairs', 'incompatible_orientation'])
def test_dense_and_bounded_reducers_reject_same_empty_families(case):
    groups, _, _ = _random_groups(37)
    if case == 'no_constraining_pairs':
        groups[0]['metric_points'] = np.zeros((25, 2))
    else:
        p = groups[1]['metric_points']; center = p.mean(0); angle = math.radians(20.)
        R = np.asarray([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
        groups[1]['metric_points'] = (p-center) @ R.T+center
    with pytest.raises(ValueError):
        _dense_joint(groups, fixed_theta=0.)
    with pytest.raises(ValueError):
        interval._joint(groups, fixed_theta=0.)


@pytest.fixture(scope='module')
def actual_synthetic_interval():
    scene = oblique_box_scene()
    result = box._qualify_regular_box_top(scene.depth, scene.mask, scene.K, scene.scale,
        scene.T, scene.geometry, scene.identity, scene.stamp, 'instance_mask',
        color_bgr=scene.color, cuboid_fallback=True)
    assert result.ok, (result.code, result.audit.get('reason'))
    model = result.model.to_evidence_dict()
    return model['qualification']['cuboid_rgb_interval'], model


@pytest.mark.parametrize('mutation', ['duplicate', 'reverse_unique'])
def test_noncanonical_raster_rows_are_rejected_before_any_pair_reduction(
        monkeypatch, actual_synthetic_interval, mutation):
    value, model = actual_synthetic_interval
    changed = copy.deepcopy(value)
    row = changed['groups'][0]
    for key in ('line_pixels', 'metric_points'):
        data = np.asarray(row[key])
        row[key] = (np.repeat(data, 2, axis=0) if mutation == 'duplicate' else data[::-1]).tolist()
    calls = []
    def forbidden_pair_reduction(*args, **kwargs):
        calls.append(True)
        raise AssertionError('noncanonical untrusted raster reached pair reduction')
    monkeypatch.setattr(interval, '_joint', forbidden_pair_reduction)
    with pytest.raises(ValueError):
        interval.validate_cuboid_rgb_interval(changed, model['size_xyz_m'],
            model['empirical_edge_uncertainty_m'], model['empirical_yaw_uncertainty_rad'],
            model['empirical_height_uncertainty_m'])
    assert not calls
