"""The user-selected FK budget keeps the original endpoint admission checks."""
from types import SimpleNamespace

import pytest

import alicia_grasp_modes.reachable_observation_views as module
from test_reachable_observation_early_qualified import (
    FEASIBLE_PROBE, assert_real_endpoint, recorded)


def test_default_budget_admits_an_originally_valid_endpoint_after_two_seconds(recorded, monkeypatch):
    now = [0.]

    def solver(_objective, _seed, **_kwargs):
        now[0] = 2.1
        return SimpleNamespace(x=FEASIBLE_PROBE.copy(), nit=1, success=True,
            message='diagnostic candidate; all original endpoint gates must still pass')

    monkeypatch.setattr(module, '_backend', lambda: solver)
    arguments = dict(recorded, max_directions=1, max_search_directions=1)
    arguments.pop('max_seconds')  # Exercise the new production default.
    rows, report = module.generate_reachable_observation_views(**arguments, clock=lambda: now[0])
    assert rows and report['max_seconds'] == report['maximum_shared_budget_sec'] == 5.
    assert 2. < report['elapsed_sec'] < 5.
    for row in rows:
        assert_real_endpoint(row, arguments)


def test_valid_solver_endpoint_after_five_seconds_is_still_discarded(recorded, monkeypatch):
    now = [0.]

    def solver(_objective, _seed, **_kwargs):
        now[0] = 5.001
        return SimpleNamespace(x=FEASIBLE_PROBE.copy(), nit=1, success=True, message='late')

    monkeypatch.setattr(module, '_backend', lambda: solver)
    arguments = dict(recorded, max_seconds=5., max_directions=1, max_search_directions=1)
    rows, report = module.generate_reachable_observation_views(**arguments, clock=lambda: now[0])
    assert rows == () and report['status'] == 'budget_exhausted'
    assert report['elapsed_sec'] == pytest.approx(5.001)
    assert report['execution_authority'] is False


@pytest.mark.parametrize('budget', [5.000001, 6., float('inf'), float('nan'), True, 0., -1.])
def test_oversized_or_invalid_budget_cannot_enter_solver(recorded, monkeypatch, budget):
    monkeypatch.setattr(module, '_backend', lambda: pytest.fail('invalid budget reached backend'))
    rows, report = module.generate_reachable_observation_views(**dict(recorded, max_seconds=budget))
    assert rows == () and report['status'] == 'invalid_input'
    assert report['execution_authority'] is False and report['original_candidates_removed'] == 0


def test_a_smaller_explicit_budget_still_stops_before_start(recorded, monkeypatch):
    times = iter([0., 2.001, 2.001])
    monkeypatch.setattr(module, '_backend', lambda: pytest.fail('late backend start'))
    rows, report = module.generate_reachable_observation_views(
        **dict(recorded, max_seconds=2.), clock=lambda: next(times))
    assert rows == () and report['status'] == 'budget_exhausted'
    assert report['max_seconds'] == 2. and report['maximum_shared_budget_sec'] == 5.
