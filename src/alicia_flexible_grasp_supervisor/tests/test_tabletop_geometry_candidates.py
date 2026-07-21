#!/usr/bin/env python3
import pathlib
import sys

import numpy as np
import pytest


ROOT = pathlib.Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / 'src'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from alicia_flexible_grasp.grasp.tabletop_geometry_candidates import (  # noqa: E402
    TabletopGeometryConfig,
    generate_tabletop_proposals,
)


def rotation_about_z(angle_rad):
    cosine = float(np.cos(angle_rad))
    sine = float(np.sin(angle_rad))
    return np.array(
        [[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]],
        dtype=float,
    )


def box_cloud(size_xyz, yaw_rad=0.0):
    xs = np.linspace(-size_xyz[0] / 2.0, size_xyz[0] / 2.0, 21)
    ys = np.linspace(-size_xyz[1] / 2.0, size_xyz[1] / 2.0, 17)
    zs = np.linspace(0.0, size_xyz[2], 7)
    points = []
    for x in xs:
        for z in zs:
            points.extend(((x, -size_xyz[1] / 2.0, z), (x, size_xyz[1] / 2.0, z)))
    for y in ys:
        for z in zs:
            points.extend(((-size_xyz[0] / 2.0, y, z), (size_xyz[0] / 2.0, y, z)))
    return np.asarray(points, dtype=float).dot(rotation_about_z(yaw_rad).T)


def carton_result(**overrides):
    values = {
        'object_points_base': box_cloud((0.051, 0.035, 0.011)),
        'obb_center_base': np.array([0.0, 0.0, 0.0055]),
        'R_base_obb': np.eye(3),
        'obb_size_xyz_m': np.array([0.051, 0.035, 0.011]),
        'support_point_base': np.zeros(3),
        'support_normal_base': np.array([0.0, 0.0, 1.0]),
        'config': TabletopGeometryConfig(),
    }
    values.update(overrides)
    return generate_tabletop_proposals(**values)


def test_real_carton_prefers_35mm_side_and_requires_39mm():
    result = carton_result()

    assert result.ok
    assert len(result.proposals) <= 8
    best = result.proposals[0]
    assert best.required_open_width_m == pytest.approx(0.039, abs=5e-4)
    assert abs(np.dot(best.jaw_axis_base, [0.0, 1.0, 0.0])) > 0.999
    assert np.dot(best.insertion_axis_base, [0.0, 0.0, 1.0]) < -0.999


@pytest.mark.parametrize('yaw_deg', (0.0, 17.0, 63.0, 121.0))
def test_rotated_unknown_instance_preserves_short_side_solution(yaw_deg):
    yaw = np.deg2rad(yaw_deg)
    result = carton_result(
        object_points_base=box_cloud((0.051, 0.035, 0.011), yaw),
        R_base_obb=rotation_about_z(yaw),
    )

    assert result.ok
    assert result.proposals[0].required_open_width_m == pytest.approx(0.039, abs=8e-4)


def test_nonfinite_cloud_has_stable_target_cloud_failure():
    points = box_cloud((0.030, 0.020, 0.010))
    points[0, 0] = np.nan

    result = carton_result(object_points_base=points)

    assert not result.ok
    assert result.failure_code == 'TARGET_CLOUD_INVALID'


def test_zero_support_normal_has_stable_support_failure():
    result = carton_result(support_normal_base=np.zeros(3))

    assert not result.ok
    assert result.failure_code == 'SUPPORT_PLANE_INVALID'


def test_obb_misaligned_with_support_has_stable_support_failure():
    result = carton_result(
        R_base_obb=np.array([[1.0, 0.0, 0.0],
                             [0.0, 0.0, -1.0],
                             [0.0, 1.0, 0.0]])
    )

    assert not result.ok
    assert result.failure_code == 'SUPPORT_PLANE_INVALID'


def test_oversized_object_has_no_fit_direction_failure():
    result = carton_result(
        object_points_base=box_cloud((0.060, 0.055, 0.011)),
        obb_size_xyz_m=np.array([0.060, 0.055, 0.011]),
    )

    assert not result.ok
    assert result.failure_code == 'NO_FIT_DIRECTION'


def test_one_sided_contact_bands_have_contact_support_failure():
    angles = np.linspace(0.0, 2.0 * np.pi, 12, endpoint=False)
    points = np.column_stack((0.017 * np.cos(angles), 0.015 * np.sin(angles),
                              np.full(12, 0.005)))

    result = carton_result(
        object_points_base=points,
        obb_size_xyz_m=np.array([0.034, 0.030, 0.010]),
    )

    assert not result.ok
    assert result.failure_code == 'CONTACT_SUPPORT_INVALID'


def test_output_respects_configured_candidate_bound():
    result = carton_result(
        object_points_base=box_cloud((0.030, 0.030, 0.011)),
        obb_size_xyz_m=np.array([0.030, 0.030, 0.011]),
        config=TabletopGeometryConfig(max_candidates=3),
    )

    assert result.ok
    assert len(result.proposals) == 3
    assert [proposal.source_index for proposal in result.proposals] == list(
        range(len(result.proposals))
    )


def test_returned_proposal_data_is_defensively_immutable():
    proposal = carton_result().proposals[0]

    with pytest.raises(ValueError):
        proposal.jaw_axis_base[0] = 0.0
    with pytest.raises(TypeError):
        proposal.audit['projection_min_m'] = 0.0
