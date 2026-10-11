"""GEN14 exact measured source; real solver and endpoint checks, offline only."""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import hashlib
import json

import numpy as np
import pytest
from scipy.optimize import minimize

from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
import alicia_grasp_modes.reachable_observation_views as module

ROOT = Path(__file__).resolve().parents[3]
FIXTURE = Path(__file__).with_name('fixtures')
# Exact feasible finite-difference probe recorded by the unmodified solver on
# these inputs. This is a test candidate, never a saved motion authorization.
FEASIBLE_PROBE = np.array([-1.9662889215842203, -.541789220533962,
    1.1962718646211747, -.012362442272550192, -1.4279141953957244, .13385894906599669])


@pytest.fixture
def recorded():
    metadata = json.loads((FIXTURE / 'gen14_fk_source.json').read_text())
    arrays_path = FIXTURE / 'gen14_fk_source.npz'
    assert hashlib.sha256(arrays_path.read_bytes()).hexdigest() == metadata['arrays_sha256']
    with np.load(str(arrays_path)) as arrays:
        source = deepcopy(metadata['source'])
        source.update({key: arrays[key].copy() for key in
            ('points_base', 'corners_base', 'center_base', 'tool_from_camera')})
        source['arrival_projection'] = {key: arrays[key].copy() for key in
            ('prism_base', 'rgb_valid', 'invalid_integral')}
    source['arrival_projection']['audit'] = metadata['arrival_audit']
    for key, expected in [('points_base', 'support_points_sha256'),
                          ('corners_base', 'conservative_target_corners_sha256'),
                          ('tool_from_camera', 'tool_camera_sha256')]:
        assert hashlib.sha256(source[key].tobytes()).hexdigest() == source['audit'][expected]
    fk = SerialUrdfFk((ROOT / metadata['model_relative_path']).read_text(),
                      ['Joint%d' % i for i in range(1, 7)])
    assert fk.model_sha256 == metadata['model_sha256']
    return dict(source=source, current_joint_positions=metadata['current_joint_positions'],
        fk=fk, gripper=GripperGeometry(.05, .002, np.array([.0434, .0286, .06]),
            np.array([.1175, .155, .0774]), .003), opening_width_m=.05,
        gaps=metadata['gaps'], depth_uncertainty_m=.003, side_depth_uncertainty_m=.003,
        joint_error_bounds_rad=metadata['joint_error_bounds_rad'])


@pytest.fixture
def measured_work_clock(monkeypatch):
    """Charge each real inequality computation; preserve the original 2s cap.

    This deterministic work model reproduces the live failure to complete all
    optimization before the cap without claiming a wall-clock benchmark.
    """
    evidence = dict(now=0., calls=0, solver_entries=0)
    def backend(fun, seed, **kwargs):
        evidence['solver_entries'] += 1
        original = kwargs['constraints'][0]['fun']
        def measured(q):
            evidence['calls'] += 1
            evidence['now'] += .002
            return original(q)
        kwargs['constraints'] = [dict(type='ineq', fun=measured)]
        return minimize(fun, seed, **kwargs)
    monkeypatch.setattr(module, '_backend', lambda: backend)
    return evidence


def assert_real_endpoint(row, args):
    assert row['execution_authority'] is False
    assert row['whole_arm_collision_checked'] is False
    assert row['complete_path_checked'] is False
    assert row['actual_arrival_measurement_required'] is True
    np.testing.assert_allclose(args['fk'](row['q']), row['T_base_tool0'], rtol=0., atol=1e-13)
    for key in ('missing_side_view_gain', 'working_volume', 'observation_envelope',
                'conditional_endpoint_support', 'measured_support_projection',
                'optical_side_evidence', 'arrival_projection_endpoint'):
        assert row['endpoint_checks'][key]['ok'], (key, row['endpoint_checks'][key])
    assert row['endpoint_checks']['tool_translation_delta_m'] >= .015
    assert row['binding'] == args['source']['binding']


def test_gen14_first_qualified_probe_survives_original_computation_budget(recorded, measured_work_clock):
    args = deepcopy(recorded)
    rows, report = module.generate_reachable_observation_views(
        **args, clock=lambda: measured_work_clock['now'])
    assert rows, report
    assert report['max_seconds'] == 2. and report['max_iterations'] == 96
    assert any(r.get('early_endpoint_qualified') for r in report['directions'])
    for row in rows:
        assert_real_endpoint(row, args)
    assert args['gaps'] == recorded['gaps']
    for name in ('points_base', 'corners_base', 'tool_from_camera'):
        assert np.array_equal(args['source'][name], recorded['source'][name])


@pytest.mark.parametrize('fault', ['support_false', 'arrival_exception', 'arrival_false'])
def test_early_endpoint_failure_continues_same_solver(recorded, measured_work_clock, monkeypatch, fault):
    name = ('validate_observation_support_projection' if fault == 'support_false'
            else 'validate_arrival_projection')
    original = getattr(module, name)
    seen = []
    def reject_first(*args, **kwargs):
        result = original(*args, **kwargs)
        seen.append(measured_work_clock['solver_entries'])
        if len(seen) == 1 and fault == 'arrival_exception':
            raise ValueError('first arrival probe has unobserved RGB')
        return dict(result, ok=False) if len(seen) == 1 else result
    monkeypatch.setattr(module, name, reject_first)
    def controlled_solver(_objective, seed, **kwargs):
        measured_work_clock['solver_entries'] += 1
        if measured_work_clock['solver_entries'] > 1:
            return SimpleNamespace(x=seed, nit=1, success=False, message='no later test seed')
        check = kwargs['constraints'][0]['fun']
        assert np.all(check(FEASIBLE_PROBE) >= 0.)
        assert np.all(check(FEASIBLE_PROBE) >= 0.)  # Exact duplicate: no second audit.
        check(np.nextafter(FEASIBLE_PROBE, np.zeros(6)))
        pytest.fail('second fully qualified probe did not leave the same solver')
    monkeypatch.setattr(module, '_backend', lambda: controlled_solver)
    rows, report = module.generate_reachable_observation_views(
        **recorded, clock=lambda: measured_work_clock['now'])
    assert rows, report
    assert seen[:2] == [1, 1], 'the first rejected early endpoint must not abandon this solver'
    assert report['directions'][0]['early_endpoint_checks'][0]['ok'] is False
    assert_real_endpoint(rows[0], recorded)


def test_four_rejected_early_checks_do_not_remove_original_final_audit(recorded, monkeypatch):
    original = module.validate_observation_support_projection
    calls = []
    def reject_four(*args, **kwargs):
        result = original(*args, **kwargs)
        calls.append(True)
        return dict(result, ok=False) if len(calls) <= 4 else result
    monkeypatch.setattr(module, 'validate_observation_support_projection', reject_four)
    entries = []
    def controlled_solver(_objective, seed, **kwargs):
        entries.append(True)
        if len(entries) > 1:
            return SimpleNamespace(x=seed, nit=1, success=False, message='no later test seed')
        check = kwargs['constraints'][0]['fun']
        q = FEASIBLE_PROBE.copy()
        for _ in range(6):
            assert np.all(check(q) >= 0.)
            q = np.nextafter(q, np.zeros(6))
        assert len(calls) == 4
        return SimpleNamespace(x=FEASIBLE_PROBE, nit=6, success=False, message='final test proposal')
    monkeypatch.setattr(module, '_backend', lambda: controlled_solver)
    rows, report = module.generate_reachable_observation_views(**recorded)
    assert rows and report['directions'][0]['accepted']
    assert not report['directions'][0].get('early_endpoint_qualified')
    assert len(report['directions'][0]['early_endpoint_checks']) == 4
    assert len(calls) >= 5
    assert_real_endpoint(rows[0], recorded)


@pytest.mark.parametrize('kind', ['false_result', 'validation_exception'])
def test_unqualified_feasible_probes_never_become_proposals(recorded, measured_work_clock, monkeypatch, kind):
    original = module.validate_observation_support_projection
    calls = []
    def reject(*args, **kwargs):
        calls.append(measured_work_clock['calls'])
        if kind == 'validation_exception':
            raise ValueError('recorded support is not valid at this candidate')
        return dict(original(*args, **kwargs), ok=False)
    monkeypatch.setattr(module, 'validate_observation_support_projection', reject)
    rows, report = module.generate_reachable_observation_views(
        **recorded, clock=lambda: measured_work_clock['now'])
    assert not rows
    assert report['directions'][0]['accepted'] is False
    assert 1 <= len(calls) <= 4, 'bounded early validations, then original solver continues'
    assert measured_work_clock['calls'] > calls[-1]


@pytest.mark.parametrize('validation', ['projection', 'arrival'])
def test_deadline_during_full_validation_cannot_publish_candidate(recorded, measured_work_clock, monkeypatch, validation):
    name = ('validate_observation_support_projection' if validation == 'projection'
            else 'validate_arrival_projection')
    original = getattr(module, name)
    calls = []
    def late(*args, **kwargs):
        calls.append(True)
        result = original(*args, **kwargs)
        measured_work_clock['now'] = 2.001
        return result
    monkeypatch.setattr(module, name, late)
    rows, report = module.generate_reachable_observation_views(
        **recorded, clock=lambda: measured_work_clock['now'])
    assert calls
    assert rows == () and report['status'] == 'budget_exhausted'
    assert report['candidate_count'] == 0


def test_expired_budget_still_cannot_begin_solver(recorded, monkeypatch):
    times = iter([0., 2., 2.])
    monkeypatch.setattr(module, '_backend', lambda: pytest.fail('late solver entry'))
    rows, report = module.generate_reachable_observation_views(
        **recorded, clock=lambda: next(times))
    assert rows == () and report['status'] == 'budget_exhausted'
