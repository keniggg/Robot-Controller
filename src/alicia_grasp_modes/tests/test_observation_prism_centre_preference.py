"""GEN20 sealed-source proposal regression; no ROS/hardware access."""
from copy import deepcopy
import json
from pathlib import Path

import numpy as np
import pytest

from alicia_grasp_modes import reachable_observation_views as module
from alicia_grasp_modes.observation_arrival_projection import (
    capture_arrival_projection, validate_arrival_projection)
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_flexible_grasp.robot.observation_view_limits import MINIMUM_OBSERVATION_IMAGE_MARGIN_PX


@pytest.fixture(scope='module')
def gen20_source():
    out = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/'
        'full_latest_restart_20261010T002347Z/gen20_analysis/arrival_projection')
    if not (out/'view_source.json').is_file():
        pytest.skip('sealed local GEN20 source is not distributed with code')
    data = json.loads((out/'view_source.json').read_text())
    source = data['source']
    with np.load(str(out/'view_source.npz')) as arrays:
        source.update({key: arrays[key].copy() for key in arrays.files})
    source['arrival_projection'] = capture_arrival_projection(data['source_status'],
        data['selection'], source['binding']['snapshot_stamp_ns'], source['camera'])
    work = Path(__file__).resolve().parents[3]
    fk = SerialUrdfFk((work/'deployment/2026-10-08/runtime/candidate_models/'
        'expanded_metric_20260929/alicia_expanded_metric_candidate.urdf').read_text(),
        ['Joint%d' % i for i in range(1, 7)])
    assert fk.model_sha256 == data['model_sha256']
    args = dict(source=source, current_joint_positions=data['current_joint_positions'], fk=fk,
        gripper=GripperGeometry(.05, .002, np.array([.0434, .0286, .06]),
            np.array([.1175, .155, .0774]), .003), opening_width_m=.05,
        gaps=data['gaps'], depth_uncertainty_m=data['depth_uncertainty_m'],
        side_depth_uncertainty_m=data['side_depth_uncertainty_m'],
        joint_error_bounds_rad=data['joint_error_bounds_rad'], max_seconds=2., max_directions=2)
    diagnostic = json.loads((out/'planned_actual_projection.json').read_text())
    error = np.linalg.inv(np.asarray(diagnostic['planned_transform'])) @ np.asarray(
        diagnostic['frames'][0]['T_base_camera'])
    return args, error


@pytest.fixture(scope='module')
def centered_result(gen20_source):
    args, _ = gen20_source
    # Complete semantic comparison; real deadline behavior is covered by the
    # budget/early-qualified tests, not CPU contention in this replay.
    rows, report = module.generate_reachable_observation_views(**args, clock=lambda: 0.)
    assert rows, report
    assert report['status'] == 'complete', report
    return rows, report


def test_gen20_centering_improves_same_direction_baseline_reserve(
        gen20_source, centered_result, monkeypatch):
    args, _ = gen20_source
    original = module._arrival_center_preference
    monkeypatch.setattr(module, '_arrival_center_preference',
        lambda *a, **kw: dict(original(*a, **kw), cost=0.))
    rows, report = module.generate_reachable_observation_views(**args, clock=lambda: 0.)
    assert rows, report
    # Compare corresponding baseline directions, not sorted positions or a
    # historical optimizer endpoint. Refinement tiers optimize face exposure.
    centered = {row['direction_gap_index']: row for row in centered_result[0]
                if row['exposure_target_noise_multiplier'] == 1.}
    joint_only = {row['direction_gap_index']: row for row in rows
                  if row['exposure_target_noise_multiplier'] == 1.}
    assert centered and centered.keys() == joint_only.keys()
    source = args['source']; camera = source['camera']
    intrinsics = np.asarray([camera[k] for k in ('fx', 'fy', 'cx', 'cy')])
    margin = source['view_limits']['effective_visibility_limits']['roi_margin_px']
    for direction, centered_row in centered.items():
        plain_row = joint_only[direction]
        centered_check = centered_row['endpoint_checks']['arrival_projection_endpoint']
        plain_check = plain_row['endpoint_checks']['arrival_projection_endpoint']
        for check in (centered_check, plain_check):
            assert check['ok'] and check['required_image_margin_px'] == margin
            assert check['minimum_image_edge_distance_px'] >= margin + report['proposal_image_reserve_px']
        assert centered_check['minimum_image_edge_distance_px'] > plain_check['minimum_image_edge_distance_px']
        costs = [original(source['arrival_projection']['prism_base'],
            np.asarray(row['T_base_camera_link']), intrinsics,
            (camera['width'], camera['height']))['cost'] for row in (centered_row, plain_row)]
        assert costs[0] < costs[1]


def test_gen20_centering_improves_selected_reserve_without_new_gates(centered_result):
    rows, report = centered_result
    assert report['objective'] == 'joint_motion_plus_frozen_prism_image_centre'
    assert report['original_candidates_removed'] == 0
    assert rows[0]['endpoint_checks']['arrival_projection_endpoint'][
        'minimum_image_edge_distance_px'] > 44.
    for row in rows:
        checks = row['endpoint_checks']
        assert not row['execution_authority'] and row['actual_arrival_measurement_required']
        for key in ('working_volume', 'observation_envelope', 'conditional_endpoint_support',
                    'measured_support_projection', 'optical_side_evidence',
                    'arrival_projection_endpoint'):
            assert checks[key]['ok'], checks[key]
        assert checks['arrival_projection_endpoint']['required_image_margin_px'] == 36.
        assert checks['arrival_centre_preference']['proposal_preference_only']
        assert not checks['arrival_centre_preference']['certifies_arrival_or_following']


def test_recorded_relative_error_remains_a_diagnostic_with_strict_counterexamples(
        gen20_source, centered_result, tmp_path):
    args, error = gen20_source
    source = args['source']; camera = source['camera']
    assert 'endpoint_error_bounds_rad' not in args
    assert centered_result[1]['arrival_error_box_required'] is False
    assert all('arrival_error_box_visibility' not in row['endpoint_checks'] for row in centered_result[0])
    evidence = dict(certifies_arrival_error_box=False, historical_transform_is_future_bound=False,
        probes={}, synthetic_margin_controls={})
    # Preserve the old default-margin probe, and also check this frozen source's
    # actual configured limits. Neither is the live goal-error-box contract.
    for name, limits in (
            ('legacy_default_margin', dict(roi_margin_px=MINIMUM_OBSERVATION_IMAGE_MARGIN_PX)),
            ('frozen_source_limits', source['view_limits']['effective_visibility_limits'])):
        accepted, rejected, probes = [], [], []
        for row in centered_result[0]:
            predicted = np.asarray(row['T_base_camera_link']) @ error
            points = (source['arrival_projection']['prism_base']-predicted[:3, 3]) @ predicted[:3, :3]
            u = camera['cx']-camera['fx']*points[:, 1]/points[:, 0]
            v = camera['cy']-camera['fy']*points[:, 2]/points[:, 0]
            edge = float(np.min(np.r_[u, camera['width']-1-u, v, camera['height']-1-v]))
            probe = dict(gap_index=row['direction_gap_index'], tier=row['exposure_target_noise_multiplier'],
                minimum_image_edge_distance_px=edge, required_image_margin_px=limits['roi_margin_px'])
            call_args = (source['arrival_projection'], predicted,
                [camera[k] for k in ('fx', 'fy', 'cx', 'cy')], camera['width'], camera['height'])
            if edge < limits['roi_margin_px']:
                with pytest.raises(ValueError, match='arrival prism leaves required camera image margin'):
                    validate_arrival_projection(*call_args, **limits)
                probe.update(ok=False, reason='arrival prism leaves required camera image margin')
                rejected.append(probe)
            else:
                check = validate_arrival_projection(*call_args, **limits)
                assert check['ok'] and check['certifies_arrival_or_tracking'] is False
                probe.update(ok=True)
                accepted.append(probe)
            probes.append(probe)
        # A changed bounded search may legitimately return only historical
        # transforms in one class. Preserve every diagnostic classification;
        # gate rejection is independently exercised by a synthetic control.
        assert len(probes) == len(centered_result[0]) and probes
        evidence['probes'][name] = probes
        nominal_row = max(centered_result[0], key=lambda row:
            row['endpoint_checks']['arrival_projection_endpoint']['minimum_image_edge_distance_px'])
        nominal = np.asarray(nominal_row['T_base_camera_link'])
        positive = validate_arrival_projection(source['arrival_projection'], nominal,
            [camera[k] for k in ('fx', 'fy', 'cx', 'cy')], camera['width'], camera['height'], **limits)
        assert positive['ok'] and positive['certifies_arrival_or_tracking'] is False
        assert positive['minimum_image_edge_distance_px'] >= limits['roi_margin_px']
        nominal_points = (source['arrival_projection']['prism_base']-nominal[:3, 3]) @ nominal[:3, :3]
        nominal_u = camera['cx']-camera['fx']*nominal_points[:, 1]/nominal_points[:, 0]
        index = int(np.argmin(nominal_u))
        target_u = limits['roi_margin_px']-1.
        # Local camera-Y translation preserves forward depth and shifts this
        # point's u by fx*dy/depth. This is not a recorded physical transform.
        local_y_shift = (target_u-nominal_u[index])*nominal_points[index, 0]/camera['fx']
        outside = nominal.copy()
        outside[:3, 3] += nominal[:3, 1]*local_y_shift
        outside_points = (source['arrival_projection']['prism_base']-outside[:3, 3]) @ outside[:3, :3]
        outside_u = camera['cx']-camera['fx']*outside_points[:, 1]/outside_points[:, 0]
        outside_v = camera['cy']-camera['fy']*outside_points[:, 2]/outside_points[:, 0]
        outside_edge = float(np.min(np.r_[outside_u, camera['width']-1-outside_u,
                                         outside_v, camera['height']-1-outside_v]))
        np.testing.assert_allclose(outside_points[:, 0], nominal_points[:, 0], rtol=0., atol=1e-12)
        assert outside_u[index] == pytest.approx(target_u, abs=1e-9, rel=0.)
        assert outside_edge < limits['roi_margin_px']
        with pytest.raises(ValueError, match='arrival prism leaves required camera image margin'):
            validate_arrival_projection(source['arrival_projection'], outside,
                [camera[k] for k in ('fx', 'fy', 'cx', 'cy')], camera['width'], camera['height'], **limits)
        evidence['synthetic_margin_controls'][name] = dict(
            synthetic=True, historical_measurement=False, execution_authority=False,
            positive=dict(ok=True, minimum_image_edge_distance_px=positive['minimum_image_edge_distance_px'],
                required_image_margin_px=limits['roi_margin_px'],
                certifies_arrival_or_tracking=positive['certifies_arrival_or_tracking']),
            negative=dict(ok=False, reason='arrival prism leaves required camera image margin',
                required_image_margin_px=limits['roi_margin_px'],
                computed_minimum_image_edge_distance_px=outside_edge,
                target_u_px=target_u, computed_u_px=float(outside_u[index]), prism_point_index=index,
                camera_local_y_shift_m=float(local_y_shift),
                T_base_camera=outside.tolist(), forward_depth_unchanged=True))
    (tmp_path/'historical-probe.json').write_text(json.dumps(evidence, sort_keys=True))


def test_no_arrival_prism_keeps_measured_motion_baseline_and_exposure_refinement(gen20_source, monkeypatch):
    args, _ = gen20_source
    args = deepcopy(args); args['source'].pop('arrival_projection')
    backend = module._backend(); checked = []
    def solver(fun, numerical_seed, *a, **kw):
        probe = numerical_seed+np.array([.001, -.002, .003, -.001, .002, -.003])
        checked.append((probe.copy(), fun(probe)))
        return backend(fun, numerical_seed, *a, **kw)
    monkeypatch.setattr(module, '_backend', lambda: solver)
    monkeypatch.setattr(module, '_arrival_center_preference',
        lambda *a, **kw: pytest.fail('prism preference used without frozen prism'))
    rows, report = module.generate_reachable_observation_views(**args, clock=lambda: 0.)
    assert checked and rows, report
    assert report['objective'] == 'joint_motion_only'
    assert len(checked) == len(report['directions'])
    seen_tiers = set(); finite_side_differs_from_center = False
    for (probe, cost), direction in zip(checked, report['directions']):
        tier = direction['exposure_target_noise_multiplier']; seen_tiers.add(tier)
        if tier == 1.:
            assert direction['objective'] == 'joint_motion_only'
            assert cost == float(np.sum((probe-np.asarray(args['current_joint_positions']))**2))
        else:
            assert direction['objective'] == 'bounded_missing_side_exposure'
            gap = args['gaps'][direction['gap_index']]
            # This sealed GEN20 input has the documented common-height schema.
            assert 'side_measurement_basis' not in gap
            center = np.asarray(gap['target_center_base_m'])
            axis = np.asarray(gap['jaw_axis_base']); sign = direction['side_sign']
            outward = sign*axis
            side_point = center+.5*gap['whole_obb_width_m']*outward
            original_camera = np.asarray(gap['initial_camera_origin_base_m'])
            # Independently project a side-plane surface element onto the
            # viewing plane. Its incidence cosine is measured from the actual
            # side point, whose finite half-width is absent in a centre ray.
            def incidence(point, camera):
                ray = camera-point
                return float(outward @ ray)/float(np.linalg.norm(ray))
            previous_side = max(0.,incidence(side_point,original_camera))
            previous_center = max(0.,incidence(center,original_camera))
            requested = previous_side+direction['requested_new_side_gain_m']/gap['object_height_m']
            camera_pose = args['fk'](np.clip(probe, args['fk'].limits[:, 0],
                args['fk'].limits[:, 1])) @ args['source']['tool_from_camera']
            camera = camera_pose[:3,3]
            face_projection = incidence(side_point,camera)
            # Radial motion must not buy directional gain. Independently cap
            # the side incidence by the change of the target-centre bearing.
            directional_increment = incidence(center,camera)-previous_center
            expected = -min(requested,face_projection,previous_side+directional_increment)
            assert cost == pytest.approx(expected, abs=1e-14, rel=0.)
            old_center_cost = -min(previous_center+direction['requested_new_side_gain_m']/gap['object_height_m'],
                                   incidence(center,camera))
            finite_side_differs_from_center |= abs(expected-old_center_cost)>1e-6
    assert seen_tiers == set(report['exposure_target_noise_multipliers'])
    assert finite_side_differs_from_center, 'the sealed source must exercise finite side distance'


def test_changed_immutable_rgb_domain_never_falls_back_to_smaller_obb(gen20_source):
    args, _ = gen20_source
    args = deepcopy(args)
    contract = args['source']['arrival_projection']
    contract['rgb_valid'] = np.logical_not(contract['rgb_valid'])
    rows, report = module.generate_reachable_observation_views(**args)
    assert not rows
    assert any('immutable inputs changed' in x.get('reason', '') for x in report['directions'])


def test_soft_preference_cannot_be_mistaken_for_acceptance():
    prism = np.array([[1., 0., 0.], [1., 0., 0.]])
    report = module._arrival_center_preference(prism, np.eye(4),
        np.array([400., 400., 326., 244.]), (653, 489))
    assert report['cost'] == 0.
    assert report['proposal_preference_only'] and not report['certifies_arrival_or_following']
    assert 'ok' not in report


def test_expired_objective_stops_before_extra_fk_projection(gen20_source, monkeypatch):
    args, _ = gen20_source
    args = dict(args); clock = [0.]
    args['clock'] = lambda: clock[0]
    def solver(fun, q0, *a, **kw):
        clock[0] = 2.01
        fun(q0)
        pytest.fail('expired objective did not stop')
    monkeypatch.setattr(module, '_backend', lambda: solver)
    monkeypatch.setattr(module, '_arrival_center_preference',
        lambda *a, **kw: pytest.fail('expired objective computed extra projection'))
    rows, report = module.generate_reachable_observation_views(**args)
    assert not rows and report['status'] == 'budget_exhausted'
