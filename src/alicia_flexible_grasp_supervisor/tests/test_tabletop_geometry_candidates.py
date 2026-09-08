#!/usr/bin/env python3
import pathlib
import sys
from dataclasses import replace

import numpy as np
import pytest


ROOT = pathlib.Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / 'src'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from alicia_flexible_grasp.grasp.tabletop_geometry_candidates import (  # noqa: E402
    TabletopCandidateContractError,
    TabletopGeometryConfig,
    generate_tabletop_proposals,
    materialize_tabletop_candidates,
)
from alicia_flexible_grasp.grasp.gripper_geometry import (  # noqa: E402
    GripperGeometry,
)
from alicia_flexible_grasp.vision.multiview_surface import (  # noqa: E402
    SurfaceView,
    append_registered_view,
    fused_surface_from_view,
)
from alicia_flexible_grasp.vision.target_observation import (  # noqa: E402
    TargetTrackIdentity,
)


GRIPPER = GripperGeometry(
    max_inner_gap_m=0.050,
    jaw_clearance_each_side_m=0.002,
    finger_size_xyz_m=np.array([0.0434, 0.0286, 0.0600]),
    palm_size_xyz_m=np.array([0.1175, 0.1550, 0.0774]),
    support_clearance_m=0.003,
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


def measured_fused_surface(two_sides=True):
    identity = TargetTrackIdentity.from_stream(3, 9)
    xs = np.linspace(-0.020, 0.020, 17)
    ys = np.linspace(-0.0175, 0.0175, 15)
    zs = np.linspace(0.0, 0.021, 15)
    top = np.asarray([(x, y, 0.021) for x in xs for y in ys])
    negative = np.asarray(
        [(x, -0.0175, z) for x in xs for z in zs[:-1]]
    )
    reference = SurfaceView(
        identity, 1_000_000_000, np.vstack((top, negative)),
        np.array([0.0, 0.0, 1.0]), 0.0, 12,
    )
    surface = fused_surface_from_view(reference)
    if not two_sides:
        top_view = SurfaceView(
            identity, 1_000_000_000, top,
            np.array([0.0, 0.0, 1.0]), 0.0, 12,
        )
        return fused_surface_from_view(top_view)
    positive = np.asarray(
        [(x, 0.0175, z) for x in xs for z in zs[:-1]]
    )
    moving = SurfaceView(
        identity, 1_100_000_000, np.vstack((top, positive)),
        np.array([0.0, 0.0, 1.0]), 0.0, 12,
    )
    registration, surface = append_registered_view(surface, reference, moving)
    assert registration.ok
    return surface


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


def test_top_only_fused_surface_cannot_gain_contact_from_complete_obb():
    result = carton_result(
        fused_surface=measured_fused_surface(two_sides=False),
        finger_geometry=GRIPPER,
        obb_center_base=np.array([0.0, 0.0, 0.0105]),
        obb_size_xyz_m=np.array([0.051, 0.035, 0.021]),
    )

    assert not result.ok
    assert result.failure_code == 'BILATERAL_SURFACE_EVIDENCE_MISSING'


def test_registered_opposing_views_supply_proposal_width_height_and_provenance():
    result = carton_result(
        fused_surface=measured_fused_surface(),
        finger_geometry=GRIPPER,
        obb_center_base=np.array([0.0, 0.0, 0.0105]),
        obb_size_xyz_m=np.array([0.051, 0.035, 0.021]),
    )

    assert result.ok
    best = result.proposals[0]
    assert best.required_open_width_m == pytest.approx(0.039, abs=0.0025)
    assert best.negative_contact_count >= 12
    assert best.positive_contact_count >= 12
    assert best.audit['negative_jaw_unique_view_count'] >= 1
    assert best.audit['positive_jaw_unique_view_count'] >= 1
    assert best.audit['measured_width_m'] == pytest.approx(0.035, abs=0.0025)
    assert (
        best.audit['bilateral_contact_height_max_m']
        > best.audit['bilateral_contact_height_min_m']
    )
    assert best.audit['contact_height_bounds_source'] == (
        'fused_measured_bilateral_surface'
    )


def test_obb_height_cannot_change_fused_measured_proposals():
    surface = measured_fused_surface()
    common = {
        'fused_surface': surface,
        'finger_geometry': GRIPPER,
        'obb_center_base': np.array([0.0, 0.0, 0.0105]),
    }
    short = carton_result(
        **common,
        obb_size_xyz_m=np.array([0.051, 0.035, 0.021]),
    )
    tall = carton_result(
        **common,
        obb_size_xyz_m=np.array([0.051, 0.035, 0.081]),
    )

    assert short.ok and tall.ok
    assert [proposal.audit for proposal in short.proposals] == [
        proposal.audit for proposal in tall.proposals
    ]
    for first, second in zip(short.proposals, tall.proposals):
        np.testing.assert_array_equal(
            first.contact_center_base, second.contact_center_base
        )
        np.testing.assert_array_equal(first.jaw_axis_base, second.jaw_axis_base)
        assert first.required_open_width_m == second.required_open_width_m


def test_real_carton_prefers_35mm_side_and_requires_39mm():
    result = carton_result()

    assert result.ok
    assert len(result.proposals) <= 8
    best = result.proposals[0]
    assert best.required_open_width_m == pytest.approx(0.039, abs=5e-4)
    assert abs(np.dot(best.jaw_axis_base, [0.0, 1.0, 0.0])) > 0.999
    assert np.dot(best.insertion_axis_base, [0.0, 0.0, 1.0]) < -0.999


def test_width_projection_trim_ignores_sparse_mask_edge_outliers():
    points = box_cloud((0.051, 0.035, 0.011))
    points = np.vstack(
        (
            points,
            np.array(
                [
                    [0.0, -0.021, 0.006],
                    [0.0, 0.021, 0.006],
                ],
                dtype=float,
            ),
        )
    )

    result = carton_result(
        object_points_base=points,
        config=TabletopGeometryConfig(
            opening_fit_clearance_each_side_m=0.0005,
            width_projection_trim_fraction=0.01,
        ),
    )

    assert result.ok
    best = result.proposals[0]
    assert best.required_open_width_m == pytest.approx(0.036, abs=5e-4)
    assert best.audit['width_projection_trim_fraction'] == pytest.approx(0.01)
    assert (
        best.audit['projection_raw_max_m']
        - best.audit['projection_raw_min_m']
    ) == pytest.approx(0.042)


def test_visible_cloud_bias_does_not_move_tabletop_contact_center():
    base_points = box_cloud((0.051, 0.035, 0.011))
    visible_side_and_top = base_points[
        (base_points[:, 1] > 0.0) & (base_points[:, 2] > 0.0055)
    ]
    biased_points = np.vstack((base_points, np.repeat(visible_side_and_top, 6, axis=0)))
    obb_center = np.array([0.0, 0.0, 0.0055])

    assert np.linalg.norm(np.median(biased_points, axis=0) - obb_center) > 0.003

    result = carton_result(object_points_base=biased_points)

    assert result.ok
    for proposal in result.proposals:
        np.testing.assert_allclose(proposal.contact_center_base, obb_center)


def test_opening_fit_clearance_allows_near_limit_tabletop_short_side():
    size = (0.060, 0.049, 0.011)
    strict = carton_result(
        object_points_base=box_cloud(size),
        obb_size_xyz_m=np.array(size),
    )
    relaxed_fit = carton_result(
        object_points_base=box_cloud(size),
        obb_size_xyz_m=np.array(size),
        config=TabletopGeometryConfig(
            jaw_clearance_each_side_m=0.002,
            opening_fit_clearance_each_side_m=0.0005,
        ),
    )

    assert not strict.ok
    assert strict.failure_code == 'NO_FIT_DIRECTION'
    assert relaxed_fit.ok
    assert relaxed_fit.proposals[0].required_open_width_m == pytest.approx(
        0.050,
        abs=5e-4,
    )


@pytest.mark.parametrize('jaw_variant', (0, 1))
@pytest.mark.parametrize('polarity', (-1.0, 0.0, 1.0))
def test_single_branch_matches_complete_materialization(jaw_variant, polarity, monkeypatch):
    import alicia_flexible_grasp.grasp.tabletop_geometry_candidates as module
    kwargs = dict(
        proposal=carton_result().proposals[0],
        support_point_base=np.zeros(3),
        support_normal_base=np.array([0.0, 0.0, 1.0]),
        gripper=GRIPPER, approach_tilt_degrees=(10.0, 15.0),
    )
    expected = [item for item in materialize_tabletop_candidates(**kwargs)
                if item.variant_index == jaw_variant
                and item.audit['approach_tilt_polarity'] == polarity]
    solved = []
    original = module.solve_tool0_translation_for_support_clearance
    def record_solve(**values):
        solved.append(values['rotation'])
        return original(**values)
    monkeypatch.setattr(module, 'solve_tool0_translation_for_support_clearance', record_solve)
    actual = materialize_tabletop_candidates(
        **kwargs, branch_key=(jaw_variant, polarity))
    assert len(solved) == len(expected) == len(actual)
    for wanted, got in zip(expected, actual):
        np.testing.assert_array_equal(got.T_base_tool0, wanted.T_base_tool0)
        assert got.source_index == wanted.source_index
        assert got.variant_index == wanted.variant_index
        assert got.audit == wanted.audit
        assert got.required_open_width_m == wanted.required_open_width_m


@pytest.mark.parametrize('branch', [(2, 1), (0, 2), (0,), 'invalid', (False, 1)])
def test_invalid_materialization_branch_is_rejected(branch):
    with pytest.raises(TabletopCandidateContractError):
        materialize_tabletop_candidates(
            proposal=carton_result().proposals[0], support_point_base=np.zeros(3),
            support_normal_base=(0, 0, 1), gripper=GRIPPER, branch_key=branch,
        )


def test_materialized_carton_candidate_places_fingers_above_table():
    candidates = materialize_tabletop_candidates(
        proposal=carton_result().proposals[0],
        support_point_base=np.zeros(3),
        support_normal_base=np.array([0.0, 0.0, 1.0]),
        gripper=GRIPPER,
        tool_jaw_axis='y',
        tool_finger_length_axis='z',
    )

    assert len(candidates) == 2
    candidate = candidates[0]
    np.testing.assert_allclose(
        candidate.T_base_tool0[:3, 1], candidate.jaw_axis_base
    )
    np.testing.assert_allclose(
        candidate.T_base_tool0[:3, 2], candidate.insertion_axis_base
    )
    assert candidate.minimum_finger_support_clearance_m >= 0.003 - 1e-9
    assert candidate.required_open_width_m == pytest.approx(0.039, abs=5e-4)
    relative = (
        candidates[1].T_base_tool0[:3, :3].T
        @ candidate.T_base_tool0[:3, :3]
    )
    np.testing.assert_allclose(
        relative,
        np.diag([-1.0, -1.0, 1.0]),
        atol=1e-8,
    )


def test_materialized_jaw_center_stays_on_contact_center_normal():
    proposal = carton_result().proposals[0]
    assert (
        proposal.audit['bilateral_contact_height_max_m']
        > proposal.audit['bilateral_contact_height_min_m']
    )
    candidates = materialize_tabletop_candidates(
        proposal=proposal,
        support_point_base=np.zeros(3),
        support_normal_base=np.array([0.0, 0.0, 1.0]),
        gripper=GRIPPER,
        tool_jaw_axis='y',
        tool_finger_length_axis='z',
        approach_tilt_degrees=(10.0,),
    )

    assert candidates
    for candidate in candidates:
        assert candidate.audit['finger_pair_center_lateral_error_m'] < 1e-9
        finger_center = np.asarray(
            candidate.audit['finger_pair_center_base'],
            dtype=float,
        )
        delta = finger_center - proposal.contact_center_base
        np.testing.assert_allclose(delta[:2], np.zeros(2), atol=1e-9)


def test_materialized_candidate_adds_finger_length_tilt_variants():
    proposal = carton_result().proposals[0]
    candidates = materialize_tabletop_candidates(
        proposal=proposal,
        support_point_base=np.zeros(3),
        support_normal_base=np.array([0.0, 0.0, 1.0]),
        gripper=GRIPPER,
        tool_jaw_axis='y',
        tool_finger_length_axis='z',
        approach_tilt_degrees=(10.0, 15.0),
    )

    assert len(candidates) == 10
    assert {item.variant_index for item in candidates} == {0, 1}
    assert len({(item.source_index, item.variant_index) for item in candidates}) == 10
    normals = [
        -float(np.dot(item.insertion_axis_base, [0.0, 0.0, 1.0]))
        for item in candidates
    ]
    assert normals.count(pytest.approx(1.0)) == 2
    assert any(value == pytest.approx(np.cos(np.deg2rad(10.0))) for value in normals)
    assert any(value == pytest.approx(np.cos(np.deg2rad(15.0))) for value in normals)
    tilted = [item for item in candidates if item.audit['approach_tilt_deg'] > 0.0]
    assert tilted
    for candidate in tilted:
        assert abs(float(np.dot(
            candidate.insertion_axis_base,
            candidate.jaw_axis_base,
        ))) < 1e-8
        assert candidate.minimum_finger_support_clearance_m >= 0.003 - 1e-9
        assert candidate.required_open_width_m == pytest.approx(
            proposal.required_open_width_m
        )


def test_materialization_rejects_non_top_down_approach_with_stable_code():
    proposal = carton_result().proposals[0]
    invalid = replace(
        proposal,
        insertion_axis_base=np.array([1.0, 0.0, 0.0]),
    )

    with pytest.raises(TabletopCandidateContractError) as caught:
        materialize_tabletop_candidates(
            proposal=invalid,
            support_point_base=np.zeros(3),
            support_normal_base=np.array([0.0, 0.0, 1.0]),
            gripper=GRIPPER,
        )

    assert caught.value.code == 'TABLETOP_APPROACH_INVALID'


def test_materialization_rejects_gripper_clearance_contract_mismatch():
    mismatched_gripper = GripperGeometry(
        max_inner_gap_m=0.050,
        jaw_clearance_each_side_m=0.003,
        finger_size_xyz_m=np.array([0.0434, 0.0286, 0.0600]),
        palm_size_xyz_m=np.array([0.1175, 0.1550, 0.0774]),
        support_clearance_m=0.003,
    )

    with pytest.raises(TabletopCandidateContractError) as caught:
        materialize_tabletop_candidates(
            proposal=carton_result().proposals[0],
            support_point_base=np.zeros(3),
            support_normal_base=np.array([0.0, 0.0, 1.0]),
            gripper=mismatched_gripper,
        )

    assert caught.value.code == 'TOOL0_GEOMETRY_INVALID'


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


def test_nonunit_support_normal_has_stable_support_failure():
    result = carton_result(support_normal_base=np.array([0.0, 0.0, 2.0]))

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


def test_aperture_above_fixed_50mm_contract_has_input_failure():
    result = carton_result(config=TabletopGeometryConfig(max_inner_gap_m=0.060))

    assert not result.ok
    assert result.failure_code == 'TABLETOP_GEOMETRY_INPUT_INVALID'


@pytest.mark.parametrize('clearance_m', (0.001, 0.003))
def test_jaw_clearance_must_match_fixed_two_mm_contract(clearance_m):
    result = carton_result(
        config=TabletopGeometryConfig(
            jaw_clearance_each_side_m=clearance_m,
        )
    )

    assert not result.ok
    assert result.failure_code == 'TABLETOP_GEOMETRY_INPUT_INVALID'


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


def test_production_tabletop_candidate_bound_accepts_twenty_four():
    result = carton_result(
        object_points_base=box_cloud((0.030, 0.030, 0.011)),
        obb_size_xyz_m=np.array([0.030, 0.030, 0.011]),
        config=TabletopGeometryConfig(max_candidates=24),
    )

    assert result.ok
    assert len(result.proposals) <= 24


def test_returned_proposal_data_is_defensively_immutable():
    proposal = carton_result().proposals[0]

    with pytest.raises(ValueError):
        proposal.jaw_axis_base[0] = 0.0
    with pytest.raises(TypeError):
        proposal.audit['projection_min_m'] = 0.0
