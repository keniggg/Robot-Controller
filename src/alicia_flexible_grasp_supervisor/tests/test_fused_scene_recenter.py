"""Exact conservative box recentering; no perception votes or motion authority."""
from dataclasses import replace
import json
from pathlib import Path

import numpy as np
import pytest

from test_fused_scene_geometry import geometry, surface, corners, inside
from alicia_flexible_grasp.vision.fused_scene_geometry import enclose_registered_surface
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry, _obb_line_interval
from alicia_flexible_grasp.grasp.tabletop_geometry_candidates import (
    TabletopGeometryConfig, generate_tabletop_proposals, materialize_tabletop_candidates,
    _plane_basis_x)


def synthetic_asymmetric_surface():
    """Measured-side counterexample, explicitly not a recorded GEN12 replay."""
    center = np.array([.2, -.3, .0115])
    points = np.array([(x, y, z) for y in (-.025, .020)
        for x in np.linspace(-.026, .026, 41)
        for z in np.linspace(.006, .022, 21)]) + [.2, -.3, 0.]
    g = geometry(center=center, size=(.060, .040, .023), points=points[861:])
    g = replace(g, support_offset_m=0.)
    return g, surface(points)


def test_conservative_recenter_restores_real_candidate_with_original_hard_gates():
    raw, fused = synthetic_asymmetric_surface()
    output, audit = enclose_registered_surface(raw, fused)
    np.testing.assert_allclose(output.center_base - raw.center_base, [0., -.0025, 0.], atol=1e-15)
    np.testing.assert_allclose(output.size_xyz_m, [.060, .045, .023], atol=1e-15)
    assert inside(corners(raw), output)
    assert inside(raw.object_points_base, output) and inside(fused.points_base, output)
    assert len(output.object_points_base) == len(fused.points_base)
    finger = GripperGeometry(.05, .002, np.array([.0434, .0286, .060]),
                            np.array([.1175, .155, .0774]), .003)
    config = TabletopGeometryConfig(angle_step_deg=5., max_candidates=8,
                                   opening_fit_clearance_each_side_m=.0005)
    generated = generate_tabletop_proposals(output.object_points_base, output.center_base,
        output.axes_base, output.size_xyz_m, np.zeros(3), output.support_normal_base,
        config, fused_surface=fused, finger_geometry=finger)
    qualified = []
    for proposal in generated.proposals:
        interval = _obb_line_interval(proposal.contact_center_base, proposal.jaw_axis_base,
            output.center_base, output.axes_base, output.size_xyz_m)
        reaches = (-interval[0] + .0005, interval[1] + .0005)
        if max(reaches) > .025 + 1.001e-9:
            continue
        variants = materialize_tabletop_candidates(proposal, np.zeros(3),
            output.support_normal_base, finger)
        qualified.append((proposal, variants))
    assert qualified
    proposal, variants = min(qualified, key=lambda row: abs(row[0].angle_deg - 90.))
    assert proposal.angle_deg == pytest.approx(90.)
    assert proposal.required_open_width_m == pytest.approx(.046)
    assert len(variants) == 2
    assert all(v.minimum_finger_support_clearance_m >= .003 - 1e-12 for v in variants)
    assert not audit['contact_or_motion_authorized']


@pytest.mark.parametrize('pitch', [0., .3, -.6])
def test_only_support_tangent_center_changes_preserving_height_and_all_old_corners(pitch):
    c, s = np.cos(pitch), np.sin(pitch)
    axes = np.array([[c, 0., s], [0., 1., 0.], [-s, 0., c]])
    g = geometry(axes=axes)
    local = np.array([[-.055, -.027, .024], [.016, .004, -.006]])
    fused = surface(local @ axes.T + g.center_base)
    output, audit = enclose_registered_surface(g, fused)
    shift = output.center_base - g.center_base
    assert float(shift @ g.support_normal_base) == pytest.approx(0., abs=1e-15)
    np.testing.assert_allclose(shift @ axes, [-.0125, -.0035, 0.], atol=1e-15)
    np.testing.assert_allclose(output.size_xyz_m, [.085, .047, .048], atol=1e-15)
    assert inside(corners(g), output) and inside(fused.points_base, output)
    assert output.support_offset_m == g.support_offset_m
    np.testing.assert_array_equal(output.axes_base, g.axes_base)
    np.testing.assert_array_equal(output.support_normal_base, g.support_normal_base)
    assert audit['original_obb_preserved'] and audit['axes_preserved']
    assert audit['support_plane_preserved'] and not audit['center_preserved']


def test_recorded_gen12_raw_box_and_complete_union_recenter_without_removing_measurements():
    fixtures = Path(__file__).parent / 'fixtures'
    metadata = json.loads((fixtures / 'gen12_enclosure_geometry.json').read_text())
    with np.load(fixtures / 'gen12_enclosure_geometry.npz') as data:
        raw = geometry(center=data['raw_center'], axes=data['raw_axes'],
                       size=data['raw_size'], points=data['raw_points'])
        raw = replace(raw, support_offset_m=float(data['support_offset']))
        union = data['recorded_union_points'].copy()
    np.testing.assert_array_equal(raw.object_points_base, union[:860])
    # The retained appended unique measurements are sufficient to reproduce
    # exact collision bounds. A unit-test surface container carries those
    # coordinates only; its synthetic vote/stamp assignment is NOT recovered
    # GEN12 fused contact evidence and never enters a contact evaluator here.
    bounds_only_container = surface(union[860:], np.zeros(len(union)-860, dtype=int), stamps=(1,))
    output, audit = enclose_registered_surface(raw, bounds_only_container)
    np.testing.assert_array_equal(output.object_points_base, union)
    assert len(output.object_points_base) == metadata['recorded_union_point_count']
    assert inside(corners(raw), output) and inside(union, output)
    old_size = np.maximum(raw.size_xyz_m, 2.*np.max(np.abs((union-raw.center_base) @ raw.axes_base), axis=0))
    np.testing.assert_array_equal(old_size, metadata['recorded_obb_size_xyz_m'])
    np.testing.assert_allclose(output.size_xyz_m,
        [.058305512475865476, .043980925584341135, .022716436783437412], rtol=0., atol=1e-12)
    assert output.size_xyz_m[2] == old_size[2]
    assert abs(float((output.center_base-raw.center_base) @ raw.support_normal_base)) < 1e-15
    assert audit['center_base_m'] == output.center_base.tolist()
    assert audit['old_center_base_m'] == raw.center_base.tolist()
    basis_x = _plane_basis_x(raw.support_normal_base)
    basis_y = np.cross(raw.support_normal_base, basis_x)
    passed = []
    for row in metadata['recorded_reach_rejections']:
        angle = np.deg2rad(row['angle_deg'])
        jaw = basis_x*np.cos(angle) + basis_y*np.sin(angle)
        old_interval = _obb_line_interval(raw.center_base, jaw, raw.center_base, raw.axes_base, old_size)
        old_reach = max(-old_interval[0], old_interval[1]) + .0005
        assert old_reach == pytest.approx(max(row['negative_reach_m'], row['positive_reach_m']), abs=1e-15)
        interval = _obb_line_interval(output.center_base, jaw, output.center_base, output.axes_base, output.size_xyz_m)
        if max(-interval[0], interval[1])+.0005 <= row['maximum_side_reach_m']+1.001e-9:
            passed.append(row['angle_deg'])
    assert sorted(passed) == [40., 45.]
    assert not metadata['contact_authorized'] and not metadata['path_authorized']
