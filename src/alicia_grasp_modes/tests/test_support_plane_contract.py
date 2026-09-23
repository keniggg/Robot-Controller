"""Independent limits for measured support refits; no ROS or arm connection."""
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'src'))
from alicia_grasp_modes import tabletop


def fixture_geometry(half_x=.020, half_y=.010):
    center = np.array([.4, -.2, .1])
    reference = np.array([0., 0., 1., -.1])
    footprint = np.array([center + [x, y, 0.]
                          for x, y in ((-half_x, -half_y), (half_x, -half_y),
                                       (half_x, half_y), (-half_x, half_y))])
    return center, reference, footprint


def tilted_plane(center, angle_deg, center_distance=0.):
    angle = np.deg2rad(angle_deg)
    normal = np.array([np.sin(angle), 0., np.cos(angle)])
    return np.r_[normal, center_distance - normal @ center]


def validate(reference, current, center, footprint, config=None):
    return tabletop._validate_support_plane_update(
        reference, current, center, config or tabletop.Config(), footprint)


def test_equivalent_plane_sign_and_scale_do_not_change_geometric_decision():
    center, reference, footprint = fixture_geometry()
    current = tilted_plane(center, 2.)
    validate(reference, current, center, footprint)
    validate(reference * 3.7, current * -2.1, center, footprint)


def test_changing_world_origin_does_not_change_local_plane_consistency():
    center, reference, footprint = fixture_geometry()
    current = tilted_plane(center, 2., .001)
    validate(reference, current, center, footprint)
    offset = np.array([50., -25., 12.])
    translated_reference = reference.copy()
    translated_current = current.copy()
    translated_reference[3] -= reference[:3] @ offset
    translated_current[3] -= current[:3] @ offset
    # Origin-plane offsets now differ by metres; local geometry is unchanged.
    assert abs(translated_current[3] - translated_reference[3]) > 1.
    validate(translated_reference, translated_current, center + offset, footprint + offset)


def test_entire_initial_measured_footprint_is_checked_not_only_its_center():
    center, reference, footprint = fixture_geometry(half_x=.05)
    current = tilted_plane(center, 3.9, .0015)
    assert abs(current[:3] @ center + current[3]) < .004
    assert max(abs(footprint @ current[:3] + current[3])) > .004
    with pytest.raises(ValueError):
        validate(reference, current, center, footprint)


@pytest.mark.parametrize('angle,distance', [(4.01, 0.), (0., .00401), (0., -.00401)])
def test_original_angular_and_local_shift_limits_remain_enforced(angle, distance):
    center, reference, footprint = fixture_geometry()
    with pytest.raises(ValueError):
        validate(reference, tilted_plane(center, angle, distance), center, footprint)


@pytest.mark.parametrize('footprint', [None, [], np.zeros((3, 2)),
    np.array([[0., 0., 0.], [1., 0., 0.], [np.nan, 1., 0.]]),
    np.zeros((3, 3))])
def test_missing_nonfinite_or_degenerate_footprint_is_not_replaced_by_a_point(footprint):
    center, reference, _ = fixture_geometry()
    with pytest.raises(ValueError):
        validate(reference, reference.copy(), center, footprint)


@pytest.mark.parametrize('bad_plane', [[0., 0., 0., .1], [0., 0., np.nan, 0.],
                                     [0., np.inf, 1., 0.]])
def test_invalid_plane_coefficients_fail_closed(bad_plane):
    center, reference, footprint = fixture_geometry()
    with pytest.raises(ValueError):
        validate(reference, np.array(bad_plane), center, footprint)
    with pytest.raises(ValueError):
        validate(np.array(bad_plane), reference, center, footprint)


def test_nonfinite_anchor_cannot_bypass_local_consistency():
    center, reference, footprint = fixture_geometry()
    center[0] = np.nan
    with pytest.raises(ValueError):
        validate(reference, reference.copy(), center, footprint)
