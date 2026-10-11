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
    rows, report = module.generate_reachable_observation_views(**args)
    assert rows, report
    assert report['elapsed_sec'] <= 2.01, report
    return rows, report


def test_gen20_old_joint_only_objective_hugs_boundary(gen20_source, monkeypatch):
    args, _ = gen20_source
    original = module._arrival_center_preference
    monkeypatch.setattr(module, '_arrival_center_preference',
        lambda *a, **kw: dict(original(*a, **kw), cost=0.))
    rows, report = module.generate_reachable_observation_views(**args)
    assert rows, report
    assert rows[0]['endpoint_checks']['arrival_projection_endpoint'][
        'minimum_image_edge_distance_px'] == pytest.approx(40., abs=.01)


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


def test_recorded_relative_error_is_a_regression_probe_not_future_evidence(
        gen20_source, centered_result):
    args, error = gen20_source
    source = args['source']; camera = source['camera']
    for row in centered_result[0]:
        predicted = np.asarray(row['T_base_camera_link']) @ error
        check = validate_arrival_projection(source['arrival_projection'], predicted,
            [camera[k] for k in ('fx', 'fy', 'cx', 'cy')], camera['width'], camera['height'])
        assert check['ok']
        assert check['certifies_arrival_or_tracking'] is False


def test_no_arrival_prism_keeps_exact_original_objective(gen20_source, monkeypatch):
    args, _ = gen20_source
    args = deepcopy(args); args['source'].pop('arrival_projection')
    backend = module._backend(); checked = []
    def solver(fun, q0, *a, **kw):
        probe = q0+np.array([.001, -.002, .003, -.001, .002, -.003])
        assert fun(probe) == float(np.sum((probe-q0)**2))
        checked.append(True)
        return backend(fun, q0, *a, **kw)
    monkeypatch.setattr(module, '_backend', lambda: solver)
    monkeypatch.setattr(module, '_arrival_center_preference',
        lambda *a, **kw: pytest.fail('prism preference used without frozen prism'))
    rows, report = module.generate_reachable_observation_views(**args)
    assert checked and rows, report
    assert report['objective'] == 'joint_motion_only'


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
