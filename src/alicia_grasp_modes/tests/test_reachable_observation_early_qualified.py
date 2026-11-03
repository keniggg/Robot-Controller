"""GEN58 exact measured source; real solver and endpoint checks, offline only."""
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
HISTORICAL_PROBE = np.array([-2.1047748267723776, -0.29429469199616265, 0.7883154661851629, -0.17722527308387753, -1.037025450323302, -0.06858081976901832])
# Same frozen source, recomputed with the unchanged contract's joint reserve.
# Provenance: view_joint_reserve_r1/timed_replay_r1/result.json.
FEASIBLE_PROBE = np.array([-2.0924490288171715, -0.7284824952037167, 1.5172604694577312, -0.6132201900278033, -1.5024365819178174, -0.5225764871502988])


@pytest.fixture
def recorded():
    # This mechanism suite retains its historical test identities. GEN14's
    # original probe now fails the new evidence contract and has a separate
    # rejection regression; these exact GEN58 inputs have a real qualified probe.
    metadata_path = FIXTURE / 'gen59_fk_source.json'
    assert hashlib.sha256(metadata_path.read_bytes()).hexdigest() == '682027c8e00af2fb0fdd85628836728576f89612dee23cd39fd4d7ae30622ac8'
    metadata = json.loads(metadata_path.read_text())
    assert np.array_equal(HISTORICAL_PROBE, metadata['first_real_qualified_probe'])
    arrays_path = FIXTURE / 'gen59_fk_source.npz'
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
        joint_error_bounds_rad=metadata['joint_error_bounds_rad'], max_seconds=2.)


@pytest.fixture
def measured_work_clock(monkeypatch):
    """Charge each real inequality computation; preserve the original 2s cap.

    This deterministic work model exercises the optimization budget without
    claiming a wall-clock benchmark or requiring every solver to use the cap.
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
                'endpoint_joint_reserve',
                'conditional_endpoint_support', 'measured_support_projection',
                'optical_side_evidence', 'arrival_projection_endpoint'):
        assert row['endpoint_checks'][key]['ok'], (key, row['endpoint_checks'][key])
    assert row['endpoint_checks']['tool_translation_delta_m'] >= .015
    assert row['binding'] == args['source']['binding']


def test_gen14_first_qualified_probe_survives_original_computation_budget(recorded, measured_work_clock, monkeypatch):
    """A genuinely qualified early probe survives a later solver's deadline."""
    def qualified_probe_then_expiry(_objective, seed, **kwargs):
        measured_work_clock['solver_entries'] += 1
        check = kwargs['constraints'][0]['fun']
        if measured_work_clock['solver_entries'] > 1:
            measured_work_clock['now'] = 2.001
            check(seed)
            pytest.fail('expired original budget allowed another solver probe')
        measured_work_clock['calls'] += 1
        measured_work_clock['now'] += .002
        check(FEASIBLE_PROBE)
        pytest.fail('fully qualified probe did not leave the solver early')
    monkeypatch.setattr(module, '_backend', lambda: qualified_probe_then_expiry)
    args = deepcopy(recorded)
    rows, report = module.generate_reachable_observation_views(
        **args, clock=lambda: measured_work_clock['now'])
    assert rows, report
    assert report['max_seconds'] == 2. and report['max_iterations'] == 96
    assert report['status'] == 'budget_exhausted'
    assert report['candidate_count'] == len(rows) == 1
    assert report['directions'][0]['early_endpoint_qualified'] is True
    assert report['directions'][0]['early_endpoint_checks'] == [dict(ok=True)]
    assert measured_work_clock['solver_entries'] == 2
    np.testing.assert_array_equal(rows[0]['q'], FEASIBLE_PROBE)
    for row in rows:
        assert_real_endpoint(row, args)
    assert args['gaps'] == recorded['gaps']
    for name in ('points_base', 'corners_base', 'tool_from_camera'):
        assert np.array_equal(args['source'][name], recorded['source'][name])


def test_recorded_real_slsqp_keeps_qualified_endpoint_within_original_work_budget(recorded, measured_work_clock):
    """Keep the frozen-source real solver without assuming its iteration route.

    SLSQP may reach the independent final audit before producing a probe that
    satisfies every stricter proposal inequality. Both routes retain the same
    full endpoint gates; the deterministic test above requires the early route.
    """
    args = deepcopy(recorded)
    rows, report = module.generate_reachable_observation_views(
        **args, clock=lambda: measured_work_clock['now'])
    assert rows, report
    assert report['max_seconds'] == 2. and report['max_iterations'] == 96
    assert report['status'] in ('complete', 'budget_exhausted')
    assert measured_work_clock['solver_entries'] > 0
    assert measured_work_clock['calls'] > 0
    # The work clock can cross the cap by one charged inequality computation.
    assert report['elapsed_sec'] <= report['max_seconds'] + .002 + 1e-12
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
    # Exercise the four-probe/final-audit mechanism independently of host load.
    rows, report = module.generate_reachable_observation_views(**recorded, clock=lambda: 0.)
    assert rows and report['directions'][0]['accepted']
    assert not report['directions'][0].get('early_endpoint_qualified')
    assert len(report['directions'][0]['early_endpoint_checks']) == 4
    assert len(calls) >= 5
    assert_real_endpoint(rows[0], recorded)


def test_direction_restarts_share_four_early_checks_and_keep_each_final_audit(recorded, monkeypatch):
    args = deepcopy(recorded); args.update(max_directions=1, max_search_directions=1)
    original = module.validate_observation_support_projection
    calls = []
    def reject(*a, **kw):
        calls.append(True)
        return dict(original(*a, **kw), ok=False)
    monkeypatch.setattr(module, 'validate_observation_support_projection', reject)
    def solver(_objective, _seed, **kwargs):
        check = kwargs['constraints'][0]['fun']; q = FEASIBLE_PROBE.copy()
        for _ in range(5):
            check(q)
            q = np.nextafter(q, np.zeros(6))
        return SimpleNamespace(x=FEASIBLE_PROBE, nit=1, success=False, message='final rejected endpoint')
    monkeypatch.setattr(module, '_backend', lambda: solver)
    rows, report = module.generate_reachable_observation_views(**args, clock=lambda: 0.)
    assert not rows and len(report['directions']) > 2
    assert sum(len(r['early_endpoint_checks']) for r in report['directions']) == 4
    assert len(calls) == 4+len(report['directions'])
    assert all(r['endpoint_checks']['measured_support_projection']['ok'] is False for r in report['directions'])


@pytest.mark.parametrize('kind', ['false_result', 'validation_exception'])
def test_unqualified_feasible_probes_never_become_proposals(recorded, measured_work_clock, monkeypatch, kind):
    original = module.validate_observation_support_projection
    calls = []
    def reject(*args, **kwargs):
        calls.append((measured_work_clock['solver_entries'], measured_work_clock['calls']))
        if kind == 'validation_exception':
            raise ValueError('recorded support is not valid at this candidate')
        return dict(original(*args, **kwargs), ok=False)
    monkeypatch.setattr(module, 'validate_observation_support_projection', reject)
    def controlled_solver(_objective, seed, **kwargs):
        measured_work_clock['solver_entries'] += 1
        if measured_work_clock['solver_entries'] > 1:
            return SimpleNamespace(x=seed, nit=1, success=False, message='no later test seed')
        check = kwargs['constraints'][0]['fun']
        q = FEASIBLE_PROBE.copy()
        # Six distinct feasible probes exercise the four-audit limit. The real
        # endpoint validator rejects each one; solver feasibility cannot admit it.
        for _ in range(6):
            measured_work_clock['calls'] += 1
            measured_work_clock['now'] += .002
            assert np.all(check(q) >= 0.)
            q = np.nextafter(q, np.zeros(6))
        assert len(calls) == 4
        return SimpleNamespace(x=FEASIBLE_PROBE, nit=6, success=False, message='final test proposal')
    monkeypatch.setattr(module, '_backend', lambda: controlled_solver)
    rows, report = module.generate_reachable_observation_views(
        **recorded, clock=lambda: measured_work_clock['now'])
    assert not rows
    assert report['directions'][0]['accepted'] is False
    assert calls
    # The cap is four early audits per direction, followed by its original final
    # audit. Later directions still retain their independent final validation.
    early_counts = []
    final_audits = 0
    for entry, direction in enumerate(report['directions'], 1):
        early = direction['early_endpoint_checks']
        seen = [count for owner, count in calls if owner == entry]
        assert len(early) <= 4
        assert all(not check['ok'] for check in early)
        assert len(seen) in (len(early), len(early) + 1)
        if early and len(seen) > len(early):
            # A converged solver may return the same last probe for the
            # mandatory final audit, without a further inequality evaluation.
            assert seen[-1] >= seen[len(early)-1]
        early_counts.append(len(early))
        final_audits += len(seen) - len(early)
    assert early_counts[0] == max(early_counts) == 4
    assert final_audits >= 1, 'original final validation remains after bounded early audits'
    # A completed search can end at its final rejection without another
    # inequality call. Faster refinement must not require exhausting the cap.
    assert measured_work_clock['calls'] >= calls[-1][1]
    if measured_work_clock['calls'] == calls[-1][1]:
        assert report['status'] == 'complete'
        assert all(not direction['accepted'] for direction in report['directions'])


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
