"""Offline endpoint tracking-error support bounds; no execution authority."""
import itertools

import numpy as np
import pytest

from alicia_flexible_grasp.robot.observation_path_guard import (
    SerialUrdfFk, ObservationPathError, observation_endpoint_support_bound,
)
from alicia_flexible_grasp.grasp.gripper_geometry import _stage_boxes, _box_corners
from test_observation_path_guard import model_xml, gripper


def minimum(fk, q, n, d):
    t = fk(q)
    return min(float(np.min(_box_corners(c, t[:3, :3], s) @ n+d))
               for _, c, s in _stage_boxes(t, gripper(), .05, 'y', 'z'))


@pytest.mark.parametrize('names', [('J1', 'J2'), ('J2', 'J1')])
@pytest.mark.parametrize('epsilon', [.035, .12, .5])
def test_analytic_bound_contains_error_box_corners_and_random_interior(names, epsilon):
    fk = SerialUrdfFk(model_xml(True), names)
    rng = np.random.RandomState(903)
    for _ in range(10):
        goal = rng.uniform(-1., 1., 2)
        normal = rng.normal(size=3)
        normal /= np.linalg.norm(normal)
        errors = np.array([epsilon, .7*epsilon])
        report = observation_endpoint_support_bound(fk, goal, errors, normal, .5, gripper(), .05)
        samples = list(itertools.product([-1., 1.], repeat=2)) + list(rng.uniform(-1., 1., (100, 2)))
        for sample in samples:
            actual = minimum(fk, goal+np.asarray(sample)*errors, normal, .5)
            assert actual >= report['minimum_support_clearance_lower_bound_m']-1e-12
        assert report['certifies_transient_tracking_or_calibration'] is False


def test_zero_error_recovers_nominal_clearance_without_modifying_physical_gate():
    fk = SerialUrdfFk(model_xml(), ['J1'])
    report = observation_endpoint_support_bound(fk, [.2], [0.], [0., 0., 1.], 0., gripper(), .05)
    assert report['minimum_support_clearance_lower_bound_m'] == pytest.approx(
        minimum(fk, [.2], np.array([0., 0., 1.]), 0.), abs=2e-12)
    assert report['required_support_clearance_m'] == .003


def test_nominal_margin_is_not_enough_when_allowed_endpoint_error_can_cross_table():
    fk = SerialUrdfFk(model_xml(), ['J1'])
    n = np.array([0., 0., 1.])
    d = .0037-minimum(fk, [1.], n, 0.)
    report = observation_endpoint_support_bound(fk, [1.], [.035], n, d, gripper(), .05)
    assert report['nominal_minimum_support_clearance_m'] > .003
    assert not report['ok']
    assert minimum(fk, [1.035], n, d) < .003


@pytest.mark.parametrize('errors', [[-.01], [float('nan')], [float('inf')], [], [4.]])
def test_invalid_error_contract_fails_closed(errors):
    fk = SerialUrdfFk(model_xml(), ['J1'])
    with pytest.raises(ObservationPathError):
        observation_endpoint_support_bound(fk, [0.], errors, [0., 0., 1.], 0., gripper(), .05)
