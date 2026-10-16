"""Positive thin support-normal evidence proposes observation, never contact.

GEN15 geometry/rejections/heights are recorded. Associations between old profile
indices and angles, missing from that old audit, are explicitly constructed.
Synthetic current points exercise real measurement and the real handoff gates.
"""
from copy import deepcopy
from dataclasses import replace
import json
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from alicia_grasp_modes.observe_return_visual import (bounded_bilateral_view_gaps,
    measured_bilateral_view_gaps)
from alicia_flexible_grasp.grasp.gripper_geometry import (
    insufficient_measured_common_height, _obb_line_interval)
from test_observe_return_supplement_remote import continuation, configured
from test_tabletop_geometry_candidates import (GRIPPER, TabletopGeometryConfig,
    SurfaceView, TargetTrackIdentity, fused_surface_from_view)


def recorded_summary():
    f = json.loads((Path(__file__).parent / 'fixtures/gen15_thin_common_height_summary.json').read_text())
    r = f['geometry']
    g = NS(center_base=np.array(r['obb_center_base_m']), axes_base=np.array(r['R_base_obb']),
        size_xyz_m=np.array(r['obb_size_xyz_m']), support_normal_base=np.array(r['support_normal_base']),
        support_offset_m=r['support_offset_m'])
    cfg = TabletopGeometryConfig(opening_fit_clearance_each_side_m=.0005)
    t = deepcopy(f['recorded_tabletop'])
    profiles = []
    n = g.support_normal_base
    basis_x = np.array([1., 0., 0.]); basis_x -= n*(basis_x@n); basis_x /= np.linalg.norm(basis_x)
    basis_y = np.cross(n, basis_x)
    for association in f['constructed_associations']:
        p = deepcopy(t['contact_boundary_profiles'][association['height_profile_index']])
        angle = association['angle_deg']
        jaw = basis_x*np.cos(np.deg2rad(angle)) + basis_y*np.sin(np.deg2rad(angle))
        interval = _obb_line_interval(g.center_base, jaw, g.center_base, g.axes_base, g.size_xyz_m)
        p.update(angle_deg=angle, negative_reach_m=-interval[0]+.0005,
            positive_reach_m=interval[1]+.0005, maximum_side_reach_m=.025)
        profiles.append(p)
    t['contact_boundary_profiles'] = profiles
    return f, {'tabletop_geometry': t}, g, cfg


def synthetic_surface(g, height=.00027, identity=None, stamp=1_000_000_000):
    """Explicit synthetic two-side depth samples, not GEN15 fused points."""
    identity = identity or TargetTrackIdentity.from_stream(4, 2)
    local = np.array([(x, sign*g.size_xyz_m[1]/2, z) for sign in (-1, 1)
        for x in np.linspace(-.020, .020, 41) for z in np.linspace(.00928, .00928+height, 33)])
    points = local @ g.axes_base.T + g.center_base
    view = SurfaceView(identity, stamp, points, g.support_normal_base, g.support_offset_m, 12)
    return fused_surface_from_view(view), view


@pytest.mark.parametrize('bounds,required,expected', [
    ((0., .000267), .002, True), ((0., .001999), .002, True),
    ((0., .002), .002, False), ((0., .003), .002, False),
    ((0., 0.), .002, False), ((.003, .001), .002, False),
    ((0., float('nan')), .002, False), ((0., float('inf')), .002, False),
    ((False, .001), .002, False), (('0', .001), .002, False),
    ((0., .001), True, False), ((0., .001), float('nan'), False),
    ((0., .001), 0., False), (None, .002, False),
    (np.array([[0., .001]]), .002, False)])
def test_positive_thin_predicate_has_no_invalid_or_contact_authority(bounds, required, expected):
    assert insufficient_measured_common_height(bounds, required) is expected


def test_gen15_summary_aperture_rejects_old_rows_and_retains_two_constructed_thin_directions():
    f, d, g, cfg = recorded_summary()
    assert f['recorded_search']['request_id'] == 17
    assert f['recorded_search']['moveit_selection']['checked_count'] == 0
    assert not bounded_bilateral_view_gaps(d, g, cfg)  # Initial preflight unchanged.
    gaps = bounded_bilateral_view_gaps(d, g, cfg, include_thin=True)
    assert [x['angle_deg'] for x in gaps] == pytest.approx([68.01385047876752, 66.50692523938376])
    assert [x['required_open_width_m'] for x in gaps] == pytest.approx([.04764009291698533, .049470235], abs=1e-9)
    assert all(x['contact_or_motion_authorized'] is False for x in gaps)
    assert all(x['observation_deficit'] == 'insufficient_measured_common_height' for x in gaps)
    assert all(x['required_open_width_m'] < .04975 for x in gaps)
    assert all(x['angle_deg'] != 70. for x in gaps)


@pytest.mark.parametrize('fault', ['missing_angle', 'missing_reach', 'source', 'sufficient',
    'zero', 'reversed', 'nan', 'oversize', 'nonrigid', 'deferred'])
def test_incomplete_or_invalid_profile_never_manufactures_observation(fault):
    _, d, g, cfg = recorded_summary(); t = d['tabletop_geometry']
    for p in t['contact_boundary_profiles']:
        if fault == 'missing_angle': p.pop('angle_deg')
        elif fault == 'missing_reach': p.pop('negative_reach_m')
        elif fault == 'source': p['contact_height_bounds_source'] = 'obb_assumed'
        elif fault in ('sufficient', 'zero', 'reversed', 'nan'):
            p['contact_height_bounds_m'] = {'sufficient':[0., .002], 'zero':[0., 0.],
                'reversed':[.003, .001], 'nan':[0., float('nan')]}[fault]
    if fault == 'oversize': g.size_xyz_m[:2] = [.08, .07]
    elif fault == 'nonrigid': g.axes_base[0, 0] += .01
    elif fault == 'deferred': t['contact_execution_gate_deferred'] = True
    assert not bounded_bilateral_view_gaps(d, g, cfg, include_thin=True)


@pytest.mark.parametrize('height,expected', [(.00027, True), (.004, False), (0., False)])
def test_current_surface_remeasures_and_retires_historical_thin_claim(height, expected):
    _, d, g, cfg = recorded_summary(); surface, _ = synthetic_surface(g, height)
    before = surface.points_base.tobytes()
    camera = g.center_base + g.axes_base @ np.array([0., -.15, .15])
    assert not measured_bilateral_view_gaps(d, g, cfg, surface, GRIPPER, camera)
    gaps = measured_bilateral_view_gaps(d, g, cfg, surface, GRIPPER, camera, include_thin=True)
    assert bool(gaps) is expected
    assert surface.points_base.tobytes() == before
    for gap in gaps:
        assert 0. < gap['measured_common_height_m'] < .002
        assert gap['source_view_stamps_ns'] == list(surface.view_stamps_ns)
        assert gap['target_epoch'] == surface.identity.epoch
        assert not gap['contact_or_motion_authorized']


def test_completed_zero_contact_search_can_prepare_second_view_from_thin_evidence(configured, monkeypatch):
    node, p = continuation(configured, monkeypatch)
    f, diagnostics, geometry, cfg = recorded_summary()
    diagnostics['tabletop_geometry']['adaptive_stage_profiles'] = [
        dict(depth_uncertainty_m=.001, execution_position_error_m=0.)]
    p.remote_diagnostics, p.geometry = diagnostics, geometry
    node.tabletop_geometry_config = cfg
    identity = node._current_stream_target_identity()
    surface, view = synthetic_surface(geometry, identity=identity, stamp=p.snapshot.stamp_ns)
    node._near_field_fused_surface, node._near_field_reference_view = surface, view
    node._latest_target_observation = replace(node._latest_target_observation,
        points_base=view.points_base, support_normal_base=view.support_normal_base,
        support_offset_m=view.support_offset_m)
    report = node._active_gate_audit_report
    report['outcome'] = {'code': 'NEAR_FIELD_SURFACE_VIEW_REQUIRED'}
    report['moveit_selection'] = deepcopy(f['recorded_search']['moveit_selection'])
    report['pipeline_funnel']['stage_counts']['locally_valid'] = dict(entered=9, passed=0, rejected=9)
    deadline = node._observe_return_session['deadline_ns']
    assert node._observe_return_prepare_supplement(p, 'NEAR_FIELD_SURFACE_VIEW_REQUIRED')
    request = node._observe_return_pending_supplement
    assert request['view_index'] == 2 and request['max_observations'] == 3
    assert request['deadline_ns'] == deadline
    assert request['failed_search']['locally_valid'] == dict(entered=9, passed=0, rejected=9)
    assert request['failed_search']['checked_count'] == 0
    assert not request['contact_or_motion_authorized']
    assert node._observe_return_phase == 'observation_prefix'
    assert node._observe_return_saved[0] is surface
    assert node._observe_return_view_gaps


def test_thin_only_initial_preflight_does_not_skip_learned_or_tilt_contact_search(configured, monkeypatch):
    from test_measured_observation_planning import preflight_context
    node, ticket, snapshot, _, transform, candidates, _, guards = preflight_context(configured, monkeypatch)
    _, d, g, cfg = recorded_summary(); g.ok = True
    d['tabletop_geometry']['materialized_count'] = 0
    surface, view = synthetic_surface(g, identity=snapshot.target_identity, stamp=snapshot.stamp_ns)
    node.tabletop_geometry_config = cfg
    node._near_field_fused_surface, node._near_field_reference_view = surface, view
    assert measured_bilateral_view_gaps(d, g, cfg, surface, GRIPPER, transform[:3, 3], include_thin=True)
    assert not node._maybe_prepare_observation_from_contact_geometry(
        ticket, snapshot, g, transform, candidates, d['tabletop_geometry'])
    assert guards == [ticket]
    assert node._observe_return_phase == 'initial_contact'
    assert node._observe_return_trigger_evidence is None


def test_actual_proposal_loop_records_matching_angle_reach_and_still_materializes_tilt_contact(configured, monkeypatch):
    from test_contact_tilt_replay import configured_replay
    from test_remote_modes import base
    node, geometry, snapshot, _, _ = configured_replay(configured, 'gen3_contact_tilt_20260926.json')
    original = base.generate_tabletop_proposals
    captured = []
    def generate(*args, **kwargs):
        result = original(*args, **kwargs); captured.extend(result.proposals); return result
    monkeypatch.setattr(base, 'generate_tabletop_proposals', generate)
    candidates, audit = node._generate_tabletop_candidates(geometry, snapshot)
    assert candidates  # The new observation predicate never truncates this contact search.
    assert any(6. < c.audit['approach_tilt_deg'] < 11.25 for c in candidates)
    by_index = {p.source_index: p for p in captured}
    assert audit['contact_boundary_profiles']
    for row in audit['contact_boundary_profiles']:
        p = by_index[row['proposal_source_index']]
        assert row['angle_deg'] == p.angle_deg
        interval = _obb_line_interval(p.contact_center_base, p.jaw_axis_base,
            geometry.center_base, geometry.axes_base, geometry.size_xyz_m)
        fit = getattr(node, 'opening_fit_clearance_each_side_m', None)
        if fit is None: fit = node.gripper_geometry.jaw_clearance_each_side_m
        assert row['negative_reach_m'] == pytest.approx(-interval[0]+fit)
        assert row['positive_reach_m'] == pytest.approx(interval[1]+fit)
        assert row['maximum_side_reach_m'] == .025
