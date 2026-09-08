"""Width authority follows the final gate on registered measured surfaces."""

import dataclasses
import types

import numpy as np
import pytest

import test_remote_grasp6d_streaming as fixtures


@pytest.fixture(scope='module')
def tilted_measured_candidates():
    remote = fixtures.remote_node
    node = remote.RemoteGrasp6DNode.__new__(remote.RemoteGrasp6DNode)
    node.tabletop_geometry_enabled = True
    node.tabletop_geometry_config = fixtures.TabletopGeometryConfig(max_candidates=32)
    node.gripper_geometry = fixtures.tabletop_gripper()
    node.gripper_tool_jaw_axis = 'y'
    node.gripper_tool_finger_length_axis = 'z'
    node.gripper_physical_open_width_m = 0.05
    node.grasp_config = {
        'tool_approach_axis': 'z',
        'pregrasp_distance_m': 0.08,
        'final_approach_offset_m': 0.015,
        'lift_height_m': 0.05,
    }
    node.candidate_min_downward_approach_cos = 0.65
    node.candidate_max_final_approach_lateral_m = 0.02
    node.soft_score_weights = fixtures.SoftScoreWeights()
    node.target_absolute_sanity_distance_m = 0.15
    node.camera_visibility_gate_enabled = False
    node.camera_visibility_diagnostic_enabled = False
    fixtures.configure_identity_handeye(node)

    # Actual partial measurements: a shared top and opposing sides. The
    # second view/support fit tilts by 0.9 degrees; ordinary bounded ICP,
    # correspondence selection and fusion construct the contact evidence.
    xs = np.linspace(-0.020, 0.020, 17)
    ys = np.linspace(-0.0175, 0.0175, 15)
    zs = np.linspace(0.0, 0.021, 15)
    top = np.asarray([(x, y, 0.021) for x in xs for y in ys])
    negative = np.asarray([(x, -0.0175, z) for x in xs for z in zs[:-1]])
    positive = np.asarray([(x, 0.0175, z) for x in xs for z in zs[:-1]])
    angle = np.deg2rad(0.9)
    rotation = np.asarray([
        [1.0, 0.0, 0.0],
        [0.0, np.cos(angle), -np.sin(angle)],
        [0.0, np.sin(angle), np.cos(angle)],
    ])
    identity = fixtures.TargetTrackIdentity.from_stream(5, 13)
    reference = fixtures.SurfaceView(
        identity, 1_000_000_000, np.vstack((top, negative)),
        np.asarray([0.0, 0.0, 1.0]), 0.0, 12,
    )
    moving = fixtures.SurfaceView(
        identity, 1_100_000_000, np.vstack((top, positive)).dot(rotation.T),
        rotation[:, 2], 0.0, 12,
    )
    registration, surface = fixtures.append_registered_view(
        fixtures.fused_surface_from_view(reference), reference, moving,
    )
    assert registration.ok, registration.code
    assert len(surface.view_stamps_ns) == 2
    node._active_multiview_surface = lambda: (surface, reference)
    geometry = fixtures.tabletop_geometry((0.040, 0.035, 0.021))
    geometry.center_base = rotation.dot(geometry.center_base)
    geometry.axes_base = rotation
    geometry.support_normal_base = rotation[:, 2]
    geometry.object_points_base = moving.points_base
    prepared = types.SimpleNamespace(
        geometry=geometry,
        stamp=remote.rospy.Time(1),
        snapshot=types.SimpleNamespace(
            quality=types.SimpleNamespace(depth_repeatability_m=0.0),
        ),
        near_field=True,
    )
    candidates, _diagnostics = node._generate_tabletop_candidates(
        geometry, snapshot=prepared.snapshot, contact_execution_phase=True,
    )
    assert candidates
    return node, prepared, candidates


def test_tabletop_normalization_uses_final_measured_width(tilted_measured_candidates):
    node, prepared, candidates = tilted_measured_candidates
    corrected = []
    for candidate in candidates:
        try:
            normalized = node._normalize_tabletop_candidate(prepared, candidate)
        except fixtures.remote_node.CandidateContractError:
            continue
        gate = normalized.geometry_gate
        assert normalized.required_open_width_m == gate.required_open_width_m
        delta = abs(candidate.required_open_width_m - gate.required_open_width_m)
        if delta > 1e-9:
            corrected.append(normalized)
            assert delta <= 0.0005
            assert gate.ok
            assert gate.support_clearance_m >= 0.003 - 1e-12
    # Tilted candidates remeasure a different aperture on the fused surface.
    # Serializing their original proposal width used to raise ValueError even
    # though the measured-width consistency and physical gates had passed.
    assert corrected


def test_tabletop_normalization_keeps_width_consistency_gate(tilted_measured_candidates):
    node, prepared, candidates = tilted_measured_candidates
    candidate = candidates[0]
    inconsistent = dataclasses.replace(
        candidate, required_open_width_m=candidate.required_open_width_m + 0.001,
    )
    with pytest.raises(fixtures.remote_node.CandidateContractError) as failure:
        node._normalize_tabletop_candidate(prepared, inconsistent)
    assert failure.value.code == 'GRIPPER_WIDTH_INVALID'
