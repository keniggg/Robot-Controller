"""Exercise the real selection loop; only strict service replies are synthetic."""
from dataclasses import replace
from types import SimpleNamespace as NS

import pytest

from test_remote_grasp6d_streaming import remote_node, MutableClock, MoveItResult
from test_grasp6d_pipeline import scored_candidate
from alicia_flexible_grasp.grasp.grasp6d_pipeline import bounded_moveit_select


def context():
    node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
    node.near_field_planning_active = True
    node._near_field_phase_id = 1
    node._near_field_phase_deadline_sec = 100.
    node.candidate_max_joint_delta_rad = 1.
    clock = MutableClock(21.)
    node._execution_plan_validity_now_sec = clock
    prepared = NS(near_field=True, near_field_phase_id=1, near_field_phase_deadline_sec=100.)
    candidates = [scored_candidate(i, float(i)) for i in (1, 2, 3)]
    node._stable_variant_runtime = {(c.track_id, c.variant_index): {} for c in candidates}
    result = MoveItResult(True, .2, .1, collision_free=True, within_joint_limits=True,
                         ik_valid=True, planning_success=True, reason='synthetic strict reply')
    return node, clock, prepared, candidates, result


def select(node, prepared, candidates):
    checker, gate = node._direct_near_field_comparison_search(prepared, candidates)
    return bounded_moveit_select(candidates, checker, exhaustive=True,
        ranking_key=lambda c: (c.track_id, c.variant_index),
        max_joint_delta_rad=node.candidate_max_joint_delta_rad,
        continue_checking=gate, continuation_stop_reason='NEAR_FIELD_DIRECT_TIMEOUT')


def test_real_driver_stops_before_another_strict_call_and_preserves_unchecked_tail():
    node, clock, prepared, candidates, result = context()
    calls = []
    def strict(candidate):
        calls.append(candidate.track_id)
        if len(calls) > 1:
            pytest.fail('additional comparison started after a complete incumbent')
        clock.value += 1.
        return result
    node._check_direct_registered_candidate = strict
    chosen = select(node, prepared, candidates)
    assert calls == [1]
    assert chosen.selected.track_id == 1
    assert len(chosen.checked) == len(chosen.reachable) == 1
    assert chosen.shortlist_count == 3 and chosen.terminated_early
    assert node._near_field_phase_deadline_sec == prepared.near_field_phase_deadline_sec == 100.
    assert node._direct_near_field_deadline_gate(prepared)() is True
    assert all('selection_deadline_sec' not in runtime for runtime in node._stable_variant_runtime.values())


@pytest.mark.parametrize('change', [
    {'reachable': False}, {'collision_free': False}, {'within_joint_limits': False},
    {'ik_valid': False}, {'planning_success': False}, {'collision_free': None},
    {'joint_path_cost': float('nan')}, {'joint_path_cost': -1.},
    {'joint_max_delta_rad': float('inf')}, {'joint_max_delta_rad': 1.1},
])
def test_incomplete_or_failed_first_result_continues_to_actual_next_candidate(change):
    node, clock, prepared, candidates, result = context()
    calls = []
    def strict(candidate):
        calls.append(candidate.track_id)
        clock.value += .1
        return replace(result, **change) if candidate.track_id == 1 else result
    node._check_direct_registered_candidate = strict
    chosen = select(node, prepared, candidates)
    assert calls == [1, 2]
    assert chosen.selected.track_id == 2
    assert len(chosen.checked) == 2 and chosen.shortlist_count == 3


def test_phase_expiry_prevents_starting_any_candidate():
    node, clock, prepared, candidates, result = context()
    clock.value = 100.
    node._check_direct_registered_candidate = lambda _: pytest.fail('expired phase ran strict call')
    chosen = select(node, prepared, candidates)
    assert chosen.selected is None and not chosen.checked


def test_no_solution_never_creates_an_incumbent():
    node, clock, prepared, candidates, result = context()
    calls = []
    def strict(candidate):
        calls.append(candidate.track_id)
        clock.value += .1
        return replace(result, reachable=False, collision_free=False)
    node._check_direct_registered_candidate = strict
    chosen = select(node, prepared, candidates)
    assert calls == [1, 2, 3] and chosen.selected is None and not chosen.reachable



@pytest.mark.parametrize('state', ['live', 'expired', 'inactive'])
def test_actual_postselection_ast_keeps_live_incumbent_and_retires_invalid_phase(state):
    """Execute both unchanged production post-selection branches, not a copy."""
    import ast
    from pathlib import Path
    tree = ast.parse(Path(remote_node.__file__).read_text())
    method = next(n for n in ast.walk(tree)
                  if isinstance(n, ast.FunctionDef) and n.name == '_accept_prediction')
    branches = []
    for statement in method.body:
        if not isinstance(statement, ast.If):
            continue
        reasons = [n.value for n in ast.walk(statement)
                   if isinstance(n, ast.Constant) and isinstance(n.value, str)]
        if ('NEAR_FIELD_COMPARISON_BUDGET_REACHED' in reasons
                or 'NEAR_FIELD_DIRECT_TIMEOUT' in reasons):
            if len(statement.body) == 1 and isinstance(statement.body[0], ast.Assign):
                branches.append(statement)
    assert len(branches) == 2
    node, clock, prepared, candidates, result = context()
    node._check_direct_registered_candidate = lambda _: result
    selection = select(node, prepared, candidates)
    assert selection.selected is not None
    if state == 'expired':
        clock.value = 100.
    elif state == 'inactive':
        node.near_field_planning_active = False
    namespace = dict(self=node, prepared=prepared, selection=selection,
                     direct_near_field=True, replace=replace)
    exec(compile(ast.Module(body=branches, type_ignores=[]), remote_node.__file__, 'exec'), namespace)
    final = namespace['selection']
    if state == 'live':
        assert final.selected is selection.selected
        assert final.termination_reason == 'NEAR_FIELD_COMPARISON_BUDGET_REACHED'
    else:
        assert final.selected is None
        assert final.termination_reason == 'NEAR_FIELD_DIRECT_TIMEOUT'
