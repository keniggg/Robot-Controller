"""Frozen gen3/gen4 geometry: small feasible tilts survive contact sampling."""
import json
from pathlib import Path
from types import SimpleNamespace, MethodType

import numpy as np
import pytest

from test_remote_modes import configured, module, base
from alicia_flexible_grasp.vision.multiview_surface import FusedTargetSurface, SurfaceView
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry
from alicia_flexible_grasp.grasp.tabletop_geometry_candidates import (
    TabletopGeometryConfig, generate_tabletop_proposals, materialize_tabletop_candidates)
from alicia_flexible_grasp.grasp.adaptive_stage_profiles import AdaptiveStageLimits


def configured_replay(configured, fixture_name):
    node, _ = configured
    fixture = json.loads((Path(__file__).parent / 'fixtures' / fixture_name).read_text())
    g = fixture['geometry']
    geometry = SimpleNamespace(
        object_points_base=np.array(g['object_points_base_m']),
        center_base=np.array(g['obb_center_base_m']),
        axes_base=np.array(g['R_base_obb']), size_xyz_m=np.array(g['obb_size_xyz_m']),
        support_normal_base=np.array(g['support_normal_base']),
        support_offset_m=g['support_offset_m'])
    identity = TargetTrackIdentity(fixture['surface']['target_epoch'],
                                   fixture['surface']['target_track_id'])
    stamp = fixture['snapshot_stamp_ns']
    surface = FusedTargetSurface(identity, geometry.object_points_base,
        np.zeros(len(geometry.object_points_base), dtype=int), (stamp,))
    reference = SurfaceView(identity, stamp, geometry.object_points_base,
        geometry.support_normal_base, geometry.support_offset_m, 1)
    snapshot = SimpleNamespace(quality=SimpleNamespace(
        depth_mad_m=fixture['depth_spatial_mad_m'],
        depth_repeatability_m=fixture['depth_repeatability_m']))
    node._mode_selection = {'mode': 'unknown', 'strategy': 'direct'}
    node.near_field_planning_active = True
    node.tabletop_geometry_enabled = True
    node.tabletop_geometry_config = TabletopGeometryConfig(angle_step_deg=5.,
        angle_dedup_deg=1., opening_fit_clearance_each_side_m=.0005,
        width_projection_trim_fraction=.01, max_candidates=32)
    node.gripper_geometry = GripperGeometry(.05, .002,
        np.array([.0434, .0286, .06]), np.array([.1175, .155, .0774]), .003)
    node.gripper_tool_jaw_axis, node.gripper_tool_finger_length_axis = 'y', 'z'
    node.candidate_min_downward_approach_cos = .65
    node.candidate_max_final_approach_lateral_m = .020
    node.adaptive_stage_limits = AdaptiveStageLimits(approach_max_m=.060,
                                                    pregrasp_max_m=.095)
    frozen = surface, reference
    node._snapshot_multiview_surface = lambda: frozen
    node._runtime_execution_position_error_m = lambda: 0.
    return node, geometry, snapshot, frozen, fixture


@pytest.fixture
def replay(configured):
    return configured_replay(configured, 'gen3_contact_tilt_20260926.json')


def test_frozen_gen3_small_feasible_tilts_survive_final_batch(replay):
    node, geometry, snapshot, frozen, fixture = replay
    observed_snapshots = []
    original = node._current_bilateral_surface_measurement
    def measure(*args, **kwargs):
        observed_snapshots.append(kwargs.get('surface_snapshot'))
        return original(*args, **kwargs)
    node._current_bilateral_surface_measurement = measure
    candidates, audit = node._generate_tabletop_candidates(geometry, snapshot)
    assert candidates and len(candidates) <= 32
    assert audit['required_contact_patch_overlap_m'] == pytest.approx(.002)
    assert all(x == frozen for x in observed_snapshots)
    tilts = [float(c.audit['approach_tilt_deg']) for c in candidates]
    assert any(6. < a < 11.25 for a in tilts)
    assert not any(abs(a) < 1e-9 for a in tilts)  # 0 degrees remains unsupported.
    assert any(p['boundary_tilts_deg'] for p in audit['contact_boundary_profiles'])
    assert all(len(p['materialization_tilts_deg']) <= 8
               for p in audit['contact_boundary_profiles'])
    for c in candidates:
        _, evidence, bounds, axis = original(c.contact_center_base,
            c.T_base_tool0[:3, :3], surface_snapshot=frozen)
        assert evidence.ok and bounds is not None
        overlap = node._tabletop_candidate_contact_overlap_m(None, c,
            geometry.support_normal_base, contact_height_bounds_m=bounds,
            contact_height_axis_base_override=axis)
        assert overlap + 1e-9 >= .002
        assert c.audit['contact_patch_overlap_m'] == pytest.approx(overlap)
    # Production capture had no continuous boundary despite 11.25-degree
    # final candidates passing. The old static-axis bounds cap is <2 mm.
    assert all(not p['boundary_tilts_deg'] for p in
               fixture['provenance']['original_boundary_profiles'])


def test_missing_measured_surface_does_not_fall_back_to_proposal_bounds(replay):
    node, geometry, _, frozen, _ = replay
    normal = geometry.support_normal_base
    point = -geometry.support_offset_m * normal
    proposals = generate_tabletop_proposals(geometry.object_points_base,
        geometry.center_base, geometry.axes_base, geometry.size_xyz_m,
        point, normal, node.tabletop_geometry_config, fused_surface=frozen[0],
        finger_geometry=node.gripper_geometry)
    proposal = proposals.proposals[0]
    probes = materialize_tabletop_candidates(proposal, point, normal,
        node.gripper_geometry, 'y', 'z', approach_tilt_degrees=(11.25, 22.5, 33.75, 45.))
    for invalid in (None, (None, None)):
        angles, audit = node._contact_boundary_tilts(proposal, point, normal,
            probes, 45., .002, surface_snapshot=invalid)
        assert not angles
        assert all(row['maximum_safe_tilt_deg'] is None for row in audit)


def test_legacy_boundary_replay_exposes_only_coarse_tilts(replay):
    node, geometry, snapshot, _, _ = replay
    node._contact_boundary_tilts = MethodType(base.RemoteGrasp6DNode._contact_boundary_tilts, node)
    node._tabletop_placement_gripper = MethodType(base.RemoteGrasp6DNode._tabletop_placement_gripper, node)
    candidates, audit = node._generate_tabletop_candidates(geometry, snapshot)
    assert len(candidates) == 22
    assert all(c.audit['approach_tilt_deg'] == pytest.approx(11.25) for c in candidates)
    assert all(not row['boundary_tilts_deg'] for row in audit['contact_boundary_profiles'])


def test_boundary_never_exceeds_requested_maximum_with_wider_probe_grid(replay):
    node, geometry, _, frozen, _ = replay
    normal = geometry.support_normal_base
    point = -geometry.support_offset_m * normal
    proposal = generate_tabletop_proposals(geometry.object_points_base,
        geometry.center_base, geometry.axes_base, geometry.size_xyz_m,
        point, normal, node.tabletop_geometry_config, fused_surface=frozen[0],
        finger_geometry=node.gripper_geometry).proposals[0]
    probes = materialize_tabletop_candidates(proposal, point, normal,
        node.gripper_geometry, 'y', 'z', approach_tilt_degrees=(11.25, 22.5, 33.75, 45.))
    angles, _ = node._contact_boundary_tilts(proposal, point, normal,
        probes, 6., .002, surface_snapshot=frozen)
    assert all(0. < angle <= 6. for angle in angles)


def test_gen4_interior_island_is_measured_and_keeps_original_gates(configured):
    node, geometry, snapshot, frozen, fixture = configured_replay(
        configured, 'gen4_contact_tilt_20260926.json')
    assert fixture['provenance']['original_materialized_count'] == 0
    assert all(not profile['boundary_tilts_deg'] for profile in
               fixture['provenance']['original_boundary_profiles'])
    original = node._current_bilateral_surface_measurement
    def measure(*args, **kwargs):
        assert kwargs['surface_snapshot'][0] is frozen[0]
        assert kwargs['surface_snapshot'][1] is frozen[1]
        return original(*args, **kwargs)
    node._current_bilateral_surface_measurement = measure
    candidates, audit = node._generate_tabletop_candidates(geometry, snapshot)
    assert 0 < len(candidates) <= 32
    assert audit['required_contact_patch_overlap_m'] == pytest.approx(.002)
    assert any(0. < c.audit['approach_tilt_deg'] < 11.25 for c in candidates)
    assert all(c.audit['approach_tilt_deg'] > 1e-9 for c in candidates)
    for profile in audit['contact_boundary_profiles']:
        assert len(profile['boundary_tilts_deg']) <= 4
        assert len(profile['materialization_tilts_deg']) <= 8
        assert all(0 <= row['interior_probe_count'] <= 4 for row in profile['branches'])
    assert any(row['interior_probe_count'] > 0 and row['maximum_safe_tilt_deg'] is not None
               for profile in audit['contact_boundary_profiles'] for row in profile['branches'])
    for candidate in candidates:
        _, evidence, bounds, axis = original(candidate.contact_center_base,
            candidate.T_base_tool0[:3, :3], surface_snapshot=frozen)
        assert evidence.ok and bounds is not None
        overlap = node._tabletop_candidate_contact_overlap_m(None, candidate,
            geometry.support_normal_base, contact_height_bounds_m=bounds,
            contact_height_axis_base_override=axis)
        assert overlap + 1e-9 >= .002
        assert candidate.audit['contact_patch_overlap_m'] == pytest.approx(overlap)


def test_gen4_interior_search_is_opt_in(configured):
    node, geometry, snapshot, _, _ = configured_replay(
        configured, 'gen4_contact_tilt_20260926.json')
    node._contact_boundary_tilts = MethodType(base.RemoteGrasp6DNode._contact_boundary_tilts, node)
    candidates, audit = node._generate_tabletop_candidates(geometry, snapshot)
    assert not candidates
    assert all(row['interior_probe_count'] == 0
               for profile in audit['contact_boundary_profiles'] for row in profile['branches'])
