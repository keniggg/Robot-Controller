"""Recorded driver-reference failures terminate shared observation work.

Service transport is controlled; original strict adapter, Mode acceptance,
search accounting and terminal publication are exercised without robot I/O.
"""
from copy import deepcopy
from pathlib import Path
from types import MethodType, SimpleNamespace as NS
import hashlib
import json

import pytest

from test_remote_modes import configured, base
from test_silhouette_observation_planning import context


FIXTURE = Path(__file__).parent / 'fixtures/gen10_shared_reference_failure.json'
DATA = json.loads(FIXTURE.read_text())
CODE = 'OBSERVATION_REFERENCE_UNAVAILABLE'
assert hashlib.sha256(FIXTURE.read_bytes()).hexdigest() == '2d5cf50b64ee427fe8372b0eee2e6b5453912267b7fb0d5a578a44733f28a52c'


def service_responses(monkeypatch, responses):
    calls = []
    monkeypatch.setattr(base.rospy, 'wait_for_service', lambda name, timeout: None)
    def proxy(name, service_type):
        assert name == '/supervisor/check_observation_pose_strict'
        def call(pose, execute):
            assert execute is False
            calls.append(pose)
            return responses[len(calls)-1]
        return call
    monkeypatch.setattr(base.rospy, 'ServiceProxy', proxy)
    return calls


@pytest.mark.parametrize('index', range(6))
def test_all_six_recorded_failures_keep_exact_cause_as_shared_reference(monkeypatch, index):
    row = DATA['checked_results'][index]
    assert row['checked'] and row['failure_code'] == 'MOVEIT_UNREACHABLE'
    node = base.RemoteGrasp6DNode.__new__(base.RemoteGrasp6DNode)
    calls = service_responses(monkeypatch, [NS(success=False, message=row['reason'])])
    result, _, _ = node._strict_moveit_evaluation(object(), observation=True)
    assert len(calls) == 1 and not result.reachable
    assert result.failure_code == CODE and result.reason == row['reason']
    assert result.ik_valid is None and result.collision_free is None


@pytest.mark.parametrize('ordinary', [False, True])
@pytest.mark.parametrize('earlier_success', [False, True])
def test_real_mode_shared_failure_stops_all_remaining_views_and_cannot_promote(
        configured, monkeypatch, tmp_path, ordinary, earlier_success):
    node, prepared, _, variant = context(configured, monkeypatch, tmp_path, ordinary=ordinary)
    variants = tuple(deepcopy(variant) for _ in range(6))
    for index, current in enumerate(variants):
        current.update(preference_index=index, camera_tilt_offset_deg=45.)
    monkeypatch.setattr(node, '_far_field_observation_variants', lambda *_: (variants, ()))
    node._strict_moveit_evaluation = MethodType(base.RemoteGrasp6DNode._strict_moveit_evaluation, node)
    good = NS(success=True, message='controlled planned observation_screened_duration_sec=2.0')
    bad = NS(success=False, message=DATA['checked_results'][0]['reason'])
    calls = service_responses(monkeypatch, ([good] if earlier_success else []) + [bad])
    outcome = node._accept_prediction(prepared)
    assert outcome['status'] == CODE
    expected = 2 if earlier_success else 1
    assert len(calls) == expected
    runtime = node._stable_variant_runtime[(0, 0)]
    search = runtime['observation_orientation_search']
    assert search['shared_reference_rejected'] and search['stop_reason'] == CODE
    assert search['checked_variant_count'] == expected and search['unchecked_variant_count'] == 6-expected
    assert not search['search_complete']
    assert runtime['_observation_qualified_incumbent_ticket'] is None
    assert not any(p.valid for p in node.rich_plan_pub.messages)
    assert not (tmp_path/'planning.json.execution').exists()
    terminal = node.latest_preview_rich_plan
    assert not terminal.valid and terminal.diagnostic.startswith(CODE + ': ')
    report = json.loads((tmp_path/'planning.json').read_text())
    assert report['selected'] is None
    assert report['observation_failure_diagnostics']['observation_orientation_search']['stop_reason'] == CODE


def test_shared_failure_also_stops_outer_family_search(configured, monkeypatch):
    node, _ = configured
    failure = base.MoveItResult(False, 0., 0., DATA['checked_results'][0]['reason'], failure_code=CODE)
    monkeypatch.setattr(node, '_check_moveit_stable_candidate', lambda _: failure)
    checker, continuing = node._far_field_observation_search()
    assert continuing()
    assert checker(object()) is failure
    assert not continuing()
    # Each fresh search has its own failure state; no previous request can
    # donate a result, reset a ticket timestamp or authorize a new operation.
    _, new_continuing = node._far_field_observation_search()
    assert new_continuing()


@pytest.mark.parametrize('code', [CODE, 'MOVEIT_UNREACHABLE'])
def test_real_outer_selector_retires_earlier_family_only_for_shared_reference(
        configured, monkeypatch, code):
    from test_grasp6d_pipeline import scored_candidate, reachable_moveit_result
    node, _ = configured
    checked = []
    def strict(candidate):
        checked.append(candidate.track_id)
        return (reachable_moveit_result(.1, .1) if candidate.track_id == 0 else
            base.MoveItResult(False, 0., 0., 'controlled family failure', failure_code=code))
    monkeypatch.setattr(node, '_check_moveit_stable_candidate', strict)
    checker, continuing = node._far_field_observation_search()
    candidates = tuple(scored_candidate(i, float(i)) for i in range(3))
    selection = base.bounded_moveit_select(candidates, checker,
        top_n=3, continue_checking=continuing)
    assert selection.selected is not None  # Original selector retains prior successes.
    outcome = node._retire_observation_selection_on_shared_failure(selection)
    if code == CODE:
        assert checked == [0, 1]
        assert outcome.selected is None and outcome.reachable == ()
        assert outcome.terminated_early and outcome.termination_reason == CODE
        assert outcome.checked == selection.checked
    else:
        assert checked == [0, 1, 2] and outcome is selection
        assert outcome.selected.track_id == 0


@pytest.mark.parametrize('message', [
    'move_to_pose exception: OBSERVATION_POSE_PLAN_FAILED',
    'OBSERVATION_PREVIEW_PATH_UNAVAILABLE: original path FOV qualification failed',
    'unrelated log prefix ' + DATA['checked_results'][0]['reason'],
])
def test_unrelated_path_failures_keep_original_classification(monkeypatch, message):
    node = base.RemoteGrasp6DNode.__new__(base.RemoteGrasp6DNode)
    service_responses(monkeypatch, [NS(success=False, message=message)])
    result, _, _ = node._strict_moveit_evaluation(object(), observation=True)
    assert result.failure_code == 'MOVEIT_UNREACHABLE' and result.reason == message


def test_contact_service_does_not_acquire_observation_reference_classification(monkeypatch):
    node = base.RemoteGrasp6DNode.__new__(base.RemoteGrasp6DNode)
    monkeypatch.setattr(base.rospy, 'wait_for_service', lambda name, timeout: None)
    def proxy(name, service_type):
        assert name == '/supervisor/check_pose_strict'
        return lambda pose, execute: NS(success=False, message=DATA['checked_results'][0]['reason'])
    monkeypatch.setattr(base.rospy, 'ServiceProxy', proxy)
    result, _, _ = node._strict_moveit_evaluation(object(), observation=False)
    assert result.failure_code == 'MOVEIT_UNREACHABLE'
