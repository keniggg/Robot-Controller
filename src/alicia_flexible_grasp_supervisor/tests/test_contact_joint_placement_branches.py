"""Frozen r18 failures and generic disjunctive CAD search contracts."""
from functools import lru_cache
import hashlib
import json
from pathlib import Path
import time
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

import alicia_flexible_grasp.grasp.contact_joint_placement as placement
from test_contact_joint_placement import (
    assert_hint, assert_fresh_endpoint_contract, exact_same_branch_solution,
    fixture_case, fresh_endpoint_contract, fresh_contract_passes,
    geometry_only_clock, geometry_proposal, no_authority, rotation_matrix, serializable,
)


FIXTURE = Path(__file__).parent/'fixtures/contact_joint_terminal_r18_20261027.json'


@lru_cache(maxsize=1)
def saved():
    data = json.loads(FIXTURE.read_text())
    assert data['schema_version'] == 1 and len(data['rows']) == 4
    assert data['execution_authorized'] is False
    assert hashlib.sha256(data['model_xml'].encode()).hexdigest() == data['model_sha256']
    assert len({row['scene']['source_ns'] for row in data['rows']}) == 1
    return data


def case(index=0):
    return fixture_case(saved(), index)


def test_original_single_cell_reproduces_negative_frozen_endpoint_margin(monkeypatch, tmp_path):
    monkeypatch.setattr(placement, 'MAXIMUM_SOLVES', 1)
    proposed, report = geometry_proposal(case())
    (tmp_path/'single_cell_geometry_contract.json').write_text(json.dumps(report, indent=2))
    assert proposed is None and report['input_valid'] and report['fallback_allowed']
    assert len(report['optimizer_attempts']) == 1
    assert report['optimizer_attempts'][0]['minimum_margin_m'] < -.0006
    no_authority(report)


@pytest.fixture(scope='module', params=range(4), ids=lambda i: 'live_r18_family_'+str(i))
def solved(request):
    args = case(request.param)
    before = serializable(args)
    proposed, report = geometry_proposal(args)
    assert serializable(args) == before
    return request.param, args, proposed, report


def test_frozen_family_uses_original_endpoint_contracts(solved, tmp_path):
    index, args, proposed, report = solved
    (tmp_path/'bounded_search.json').write_text(json.dumps(report, indent=2))
    if index == 3:
        # The nearly vertical family has no admissible hint in this bounded
        # search. That is not a claim of global geometric infeasibility.
        assert proposed is None and report['input_valid'] and report['fallback_allowed']
        assert 'original endpoint/evidence gates' in report['reason']
        no_authority(report)
        return
    assert_hint(args, proposed, report)
    assert len(report['optimizer_attempts']) <= placement.MAXIMUM_SOLVES
    assert sum(row['iterations'] for row in report['optimizer_attempts']) <= placement.MAXIMUM_ITERATIONS
    joints = exact_same_branch_solution(args, proposed)
    outcomes = fresh_endpoint_contract(args, proposed, joints)
    (tmp_path/'fresh_endpoints.json').write_text(json.dumps(outcomes, indent=2))
    if index == 2:
        # A rigid separation hint cannot authorize contact after SDK count
        # quantization reduces the exact original normal-height interval.
        assert all(not row['collision'] for outcome in outcomes.values() for row in outcome['endpoints'])
        assert outcomes['nominal']['height']['ok']
        assert not outcomes['quantized_actual']['height']['ok']
        assert not fresh_contract_passes(args, outcomes)
        no_authority(report)
    else:
        assert fresh_contract_passes(args, outcomes), outcomes


def test_changed_sat_cell_is_geometry_derived_and_shared_budget_is_not_reset(solved):
    index, args, proposed, report = solved
    if index != 3:
        assert_hint(args, proposed, report)
    attempts = report['optimizer_attempts']
    assert attempts[0]['minimum_margin_m'] < 0.
    assert len(attempts) > 1
    if index != 3:
        assert attempts[-1]['minimum_margin_m'] > 0.
    else:
        assert attempts[-1]['minimum_margin_m'] <= 0.
    assert all(a['evaluation_count'] < b['evaluation_count'] for a, b in zip(attempts, attempts[1:]))
    assert report['parameters']['maximum_seconds'] == 2.
    assert report['parameters']['maximum_evaluations'] == 256
    assert report['parameters']['maximum_iterations'] == 48
    seed = attempts[1]['seed']
    assert seed['policy'] == 'nearest_separating_halfspace_intersection'
    assert np.dot(seed['changed_axis_base'], seed['previous_active_axis_base']) < .95


def test_original_wall_budget_is_shared_across_real_sat_branches(tmp_path):
    from scipy.optimize import minimize as original_minimize
    original_projection = placement._separating_branch_seed
    clock = dict(now=0., solver_calls=0)
    events = [dict(stage='request_start', seconds=0.)]

    def projection(*a, **k):
        seed, audit = original_projection(*a, **k)
        assert seed is not None
        assert audit['policy'] == 'nearest_separating_halfspace_intersection'
        clock['now'] = 1.5
        events.append(dict(stage='original_branch_seed_completed', seconds=clock['now']))
        return seed, audit

    def constrained_solver(fun, initial, **kwargs):
        clock['solver_calls'] += 1
        if clock['solver_calls'] == 1:
            # The original SciPy solver and original constraints must reach
            # the negative first cell; no fabricated OptimizeResult/endpoint.
            result = original_minimize(fun, initial, **kwargs)
            assert result.success
            events.append(dict(stage='original_first_solver_completed', seconds=clock['now']))
            return result
        assert clock['solver_calls'] == 2 and clock['now'] == 1.5
        clock['now'] = 2.001
        events.append(dict(stage='second_solver_entry', seconds=clock['now']))
        # Invoke the unchanged production constraint at the real second seed.
        # A restarted deadline would incorrectly leave 0.501 s elapsed here.
        try:
            kwargs['constraints'][0]['fun'](initial)
        except TimeoutError as error:
            assert str(error) == 'joint placement computation budget exhausted'
            events.append(dict(stage='original_shared_budget_rejected', seconds=clock['now']))
            raise
        pytest.fail('original constraint allowed a renewed computation deadline')

    started = time.monotonic()
    with patch.object(placement, 'time', SimpleNamespace(monotonic=lambda: clock['now'])), \
            patch.object(placement, '_separating_branch_seed', projection), \
            patch('scipy.optimize.minimize', constrained_solver):
        proposal, report = placement.contact_joint_placement(**case())
    (tmp_path/'shared_branch_deadline_contract.json').write_text(json.dumps(dict(
        clock_mode='controlled_shared_deadline', actual_wall_sec=time.monotonic()-started,
        production_budget_or_performance_pass_claimed=False, events=events, report=report), indent=2))
    assert proposal is None and not report['proposed']
    assert report['input_valid'] and report['fallback_allowed']
    assert report['reason'] == 'joint placement computation budget exhausted'
    assert report['elapsed_sec'] == 2.001
    assert clock['solver_calls'] == 2
    assert [row['stage'] for row in events] == [
        'request_start', 'original_first_solver_completed',
        'original_branch_seed_completed', 'second_solver_entry', 'original_shared_budget_rejected']
    attempts = report['optimizer_attempts']
    assert len(attempts) == 1 and attempts[0]['minimum_margin_m'] < 0.
    assert 0 < attempts[0]['iterations'] <= placement.MAXIMUM_ITERATIONS
    assert 0 < attempts[0]['evaluation_count'] <= report['evaluation_count'] <= placement.MAXIMUM_EVALUATIONS
    assert report['parameters']['maximum_seconds'] == 2.
    assert report['parameters']['maximum_iterations'] == 48
    assert report['parameters']['maximum_evaluations'] == 256
    assert report['parameters']['maximum_solves'] == 3
    no_authority(report)


@pytest.mark.parametrize('angles', ((0., 0., 0.), (.23, -.42, 1.17), (-.7, .9, -.31)))
def test_halfspace_projection_is_independent_of_world_axes(angles):
    rotation = rotation_matrix(Rotation.from_euler('xyz', angles))
    # The two jaw faces x <= -0.2 and x >= 0.2 cannot both separate.
    # The second body's alternative y >= 0.5 is a different SAT cell.
    affine = [
        (np.array([-.2]), np.array([[-1., 0., 0.]]) @ rotation.T),
        (np.array([-.2, -.5]), np.array([[1., 0., 0.], [0., 1., 0.]]) @ rotation.T),
    ]
    point, audit = placement._separating_branch_seed(affine, np.zeros(3), 1.,
        [.5], rotation @ np.array([0., 0., 1.]), set(), lambda: None)
    np.testing.assert_allclose(point, rotation @ np.array([-.2, .5, 0.]), atol=1e-12, rtol=0.)
    assert audit['changed_body_index'] == 1 and audit['retained_body_index'] == 0
    assert all(np.max(b+a @ point) >= -1e-12 for b, a in affine)
    rejected, _ = placement._separating_branch_seed(affine, np.zeros(3), .4,
        [.5], rotation @ np.array([0., 0., 1.]), set(), lambda: None)
    assert rejected is None


@pytest.mark.parametrize('exception_type', (ValueError, RuntimeError, TimeoutError))
def test_original_callback_exception_in_branch_projection_propagates(exception_type, monkeypatch, tmp_path):
    args = case()
    entered = []
    original = placement._separating_branch_seed
    error = exception_type('cancel during new CAD branch')
    def projection(*a, **k):
        entered.append(True)
        return original(*a, **k)
    def available():
        if entered:
            raise error
        return True
    monkeypatch.setattr(placement, '_separating_branch_seed', projection)
    args['deadline_check'] = available
    with geometry_only_clock() as clock_evidence:
        with pytest.raises(exception_type) as caught:
            placement.contact_joint_placement(**args)
    (tmp_path/'branch_cancellation_clock.json').write_text(json.dumps(clock_evidence, indent=2))
    assert caught.value is error


def test_evaluation_budget_counts_original_and_branch_cells(monkeypatch):
    monkeypatch.setattr(placement, 'MAXIMUM_EVALUATIONS', 12)
    proposal, report = placement.contact_joint_placement(**case())
    assert proposal is None and report['evaluation_count'] == 12
    assert report['fallback_allowed'] and 'evaluation limit' in report['reason']
    assert len(report['optimizer_attempts']) == 1
    no_authority(report)


def test_missing_midpoint_does_not_skip_geometric_search(monkeypatch, tmp_path):
    args = case()
    original = placement._failure_seed
    def no_midpoint(*a, **k):
        fk, command, _ = original(*a, **k)
        return fk, command, np.zeros(3)
    monkeypatch.setattr(placement, '_failure_seed', no_midpoint)
    proposal, report = geometry_proposal(args)
    (tmp_path/'missing_midpoint_geometry_contract.json').write_text(json.dumps(report, indent=2))
    assert report['seed_available'] is False and report['evaluation_count'] > 0
    assert_hint(args, proposal, report)
    assert_fresh_endpoint_contract(args, proposal, exact_same_branch_solution(args, proposal))
