"""Rebuilding a selected placement still needs a new, exact full precheck."""
from copy import deepcopy
from dataclasses import replace
import json
from types import SimpleNamespace as NS
from unittest import mock

import pytest

from test_contact_height_window_proposals import fixture, remote


def bound_fixture():
    values = fixture()
    node, source, prepared, runtime, good, bad, calls, clock, _ = values
    prepared.ticket = NS(request_id=40, generation=9, target_epoch=1791800107306397756)
    prepared.request_invalidation_generation = 3
    prepared.near_field_phase_id = 2
    prepared.near_field_phase_deadline_sec = 123.
    node._stream_condition = object()
    node._require_stream_ticket_current = mock.Mock()
    state = {'reject_all': False, 'reject_plan_ids': set(), 'reply_after_deadline': False}

    def strict(candidate, stages, p, r, operation, deadline_sec):
        assert p is prepared and deadline_sec == 123.
        if clock[0] >= deadline_sec:
            return replace(bad, failure_code='MOVEIT_TIMEOUT'), {}
        calls.append((deepcopy(candidate), operation, deadline_sec))
        passed = (operation == 'final_registered_height_window_strict_sequence'
                  and not state['reject_all'] and candidate.plan_id not in state['reject_plan_ids'])
        r['contact_sequence_precheck'] = dict(success=passed, failed_stage='grasp',
            reason=bad.reason, plan_id=candidate.plan_id, fresh_call=len(calls))
        if state['reply_after_deadline']:
            clock[0] = 124.
        return (good if passed else bad), {'fresh_call': len(calls)}

    node._final_contact_sequence_evaluation = strict
    return values, state


def select(values):
    node, source, prepared, runtime, *_ = values
    selected = deepcopy(source)
    node._strict_check_final_near_field_plan(selected, prepared, runtime)
    return selected


def test_selection_then_two_publication_rebuilds_preserve_height_and_recheck():
    values, _ = bound_fixture()
    node, source, prepared, runtime, _, _, calls, _, _ = values
    original_runtime_sequence = deepcopy(runtime.get('sequence'))
    original = deepcopy(source)
    selected = select(values)
    assert len(calls) == 2
    for expected_count in (3, 4):
        publication = deepcopy(source)
        sequence, _, metrics = node._strict_check_final_near_field_plan(publication, prepared, runtime)
        assert publication == selected and publication.plan_id != source.plan_id
        assert sequence.grasp.pose == publication.poses[2]
        assert len(calls) == expected_count and metrics['fresh_call'] == expected_count
        assert runtime['contact_sequence_precheck']['fresh_call'] == expected_count
        audit = runtime['contact_height_window_preferred_revalidation']
        assert audit['selected_geometry_revalidation'] is True
        assert audit['strict_result_reused'] is False and audit['deadline_extended'] is False
        assert audit['evaluated_proposal_indices'] == [0]
    assert source == original and runtime.get('sequence') == original_runtime_sequence
    assert node._registered_sequence_geometry_gate.call_count == 3
    assert [item[0].plan_id for item in calls] == [source.plan_id] + [selected.plan_id] * 3
    assert all(item[2] == 123. for item in calls)


@pytest.mark.parametrize('change', [
    'request', 'generation', 'epoch', 'invalidation', 'phase', 'deadline',
    'source_stamp', 'position', 'registration', 'contact_evidence',
    'opening', 'height_limit', 'gripper', 'prepared_object',
])
def test_changed_binding_does_not_prefer_previous_recipe(change):
    values, _ = bound_fixture()
    node, source, prepared, runtime, _, _, calls, _, _ = values
    select(values)
    recipe = runtime['_selected_contact_height_recipe']
    if change in ('request', 'generation', 'epoch'):
        field = {'request': 'request_id', 'generation': 'generation', 'epoch': 'target_epoch'}[change]
        setattr(prepared.ticket, field, getattr(prepared.ticket, field) + 1)
    elif change == 'invalidation': prepared.request_invalidation_generation += 1
    elif change == 'phase': prepared.near_field_phase_id += 1
    elif change == 'deadline': prepared.near_field_phase_deadline_sec += .001
    elif change == 'source_stamp': source.header.stamp = remote.rospy.Time(11)
    elif change == 'position': source.poses[0].position.x += 1e-12
    elif change == 'registration': source.refinement_translation_m += .00001
    elif change == 'contact_evidence': source.contact_geometry_evidence += ' '
    elif change == 'opening': node.gripper_physical_open_width_m -= .00001
    elif change == 'height_limit': node._contact_height_evidence_limit = lambda: .0009
    elif change == 'gripper': node.gripper_geometry = replace(node.gripper_geometry, support_clearance_m=.0031)
    elif change == 'prepared_object': prepared = deepcopy(prepared)
    assert not node._contact_height_recipe_matches(recipe, source, prepared)
    # A changed input uses the normal search; no cached strict result is used.
    node._contact_height_window_retry = mock.Mock(return_value=None)
    node._final_contact_sequence_evaluation = mock.Mock(return_value=(values[4], {}))
    node._strict_check_final_near_field_plan(deepcopy(source), prepared, runtime)
    node._contact_height_window_retry.assert_not_called()
    assert node._final_contact_sequence_evaluation.call_count == 1
    assert '_selected_contact_height_recipe' not in runtime


def test_rejected_preferred_recipe_falls_back_without_fifth_check():
    values, state = bound_fixture()
    node, source, prepared, runtime, _, _, calls, _, _ = values
    selected = select(values)
    state['reject_all'] = True
    rebuilt = deepcopy(source)
    with pytest.raises(remote.CandidateContractError):
        node._strict_check_final_near_field_plan(rebuilt, prepared, runtime)
    retry_calls = calls[2:]
    assert len(retry_calls) == 4
    ids = [item[0].plan_id for item in retry_calls]
    assert ids[0] == selected.plan_id and ids[1] == source.plan_id
    assert len(set(ids)) == 4
    assert rebuilt == source
    assert '_selected_contact_height_recipe' not in runtime
    assert 'final_execution_sequence' not in runtime


def test_rejected_preferred_recipe_can_select_original_remaining_proposal():
    values, state = bound_fixture()
    node, source, prepared, runtime, _, _, calls, _, _ = values
    selected = select(values)
    state['reject_plan_ids'].add(selected.plan_id)
    rebuilt = deepcopy(source)
    node._strict_check_final_near_field_plan(rebuilt, prepared, runtime)
    assert len(calls[2:]) == 3  # Preferred rejected, original rejected, next bounded proposal accepted.
    assert len({item[0].plan_id for item in calls[2:]}) == 3
    assert rebuilt.plan_id not in (source.plan_id, selected.plan_id)
    assert runtime['_selected_contact_height_recipe']['proposal_index'] == 1


@pytest.mark.parametrize('boundary', ['before', 'after'])
def test_preferred_revalidation_cannot_extend_original_deadline(boundary):
    values, state = bound_fixture()
    node, source, prepared, runtime, _, _, calls, clock, _ = values
    select(values)
    if boundary == 'before': clock[0] = 123.
    else: state['reply_after_deadline'] = True
    rebuilt = deepcopy(source)
    with pytest.raises(remote.CandidateContractError):
        node._strict_check_final_near_field_plan(rebuilt, prepared, runtime)
    assert rebuilt == source
    assert len(calls) == (2 if boundary == 'before' else 3)
    assert 'final_execution_sequence' not in runtime
    assert '_selected_contact_height_recipe' not in runtime


def test_preferred_recipe_still_calls_real_fresh_contact_service_adapter(monkeypatch):
    values, _ = bound_fixture()
    node, source, prepared, runtime, _, _, calls, _, _ = values
    selected = select(values)
    node.contact_sequence_precheck_enabled = True
    node._final_contact_sequence_evaluation = remote.RemoteGrasp6DNode._final_contact_sequence_evaluation.__get__(node)
    phase_checks = []
    node._require_current_contact_precheck_phase = lambda p, d: phase_checks.append((p.ticket.request_id, d))
    requests = []
    def service(request):
        requests.append(deepcopy(request))
        evidence = dict(plan_id=request.plan.plan_id,
            scope='read_only_candidate_constant_bias_prediction', execution_authorized=False,
            success=True, checked_stage_names=['pregrasp', 'approach', 'grasp'],
            contact_prediction=dict(scope='constant_current_bias_prediction',
                future_following_verified=False, execution_authorized=False,
                predicted_command_geometry={'fresh': len(requests)}, nominal_geometry={'fresh': len(requests)}))
        return NS(success=True, evidence_json=json.dumps(evidence), joint_path_cost=1., joint_max_delta=.5,
                  message='strict pose sequence planned stages=pregrasp,approach,grasp')
    monkeypatch.setattr(remote.rospy, 'wait_for_service', lambda *a, **kw: None)
    monkeypatch.setattr(remote.rospy, 'ServiceProxy', lambda *a, **kw: service)
    for _ in range(2):
        rebuilt = deepcopy(source)
        node._strict_check_final_near_field_plan(rebuilt, prepared, runtime)
        assert rebuilt == selected
    assert len(requests) == 2 and len(phase_checks) == 6
    assert all(r.plan.plan_id == selected.plan_id and r.deadline.to_sec() == 123. for r in requests)
    assert runtime['contact_sequence_precheck']['contact_prediction']['nominal_geometry']['fresh'] == 2


def test_cancellation_during_preferred_geometry_propagates_without_fallback():
    values, _ = bound_fixture()
    node, source, prepared, runtime, _, _, calls, _, _ = values
    select(values)
    node._require_stream_ticket_current.side_effect = remote.StreamResultCancelled('request revoked')
    with pytest.raises(remote.StreamResultCancelled):
        node._strict_check_final_near_field_plan(deepcopy(source), prepared, runtime)
    assert len(calls) == 2
    assert '_selected_contact_height_recipe' not in runtime


@pytest.mark.parametrize('options', ['preferred', 'skip', 'both', 'ordinary'])
@pytest.mark.parametrize('measured_opportunity', [False, True])
def test_observe_return_override_keeps_height_search_and_forwards_options(options, measured_opportunity):
    from alicia_grasp_modes.observe_return_remote import ObserveReturnRemoteMixin
    forwarded = []
    class Parent:
        def _contact_height_window_retry(self, *args, **kwargs):
            forwarded.append((args, kwargs))
            return 'fresh-super-check'
    class Node(ObserveReturnRemoteMixin, Parent):
        pass
    node = Node()
    node._contact_yield_search = dict(key='exact-request', incumbent_found=False,
        fallback=False, deferred_height_families=[])
    node._contact_yield_request_key = lambda p: 'exact-request'
    node._contact_yield_measured_opportunity = lambda p: measured_opportunity
    recipe = {'geometry_only': True}
    kwargs = {}
    if options in ('preferred', 'both'): kwargs['preferred_recipe'] = recipe
    if options in ('skip', 'both'): kwargs['skip_proposal_indices'] = (1,)
    plan, prepared, state, result = NS(plan_id='a' * 24), NS(), {}, NS()
    returned = node._contact_height_window_retry(plan, prepared, state, result, 123., **kwargs)
    assert node._contact_yield_search['incumbent_found'] is False
    # The measured-view opportunity does not suppress a same-station height
    # retry. The original candidate deadline and option identities are retained.
    assert returned == 'fresh-super-check' and len(forwarded) == 1
    args, passed = forwarded[0]
    assert args == (plan, prepared, state, result, 123.)
    if options == 'ordinary': assert passed == {}
    else:
        assert passed['preferred_recipe'] is kwargs.get('preferred_recipe')
        assert passed['skip_proposal_indices'] == kwargs.get('skip_proposal_indices', ())
    assert 'contact_height_window_search' not in state
    assert node._contact_yield_search['deferred_height_families'] == []


@pytest.mark.parametrize('measured_opportunity', [False, True])
def test_observe_return_override_runs_real_height_search_and_fresh_publication_check(measured_opportunity):
    from alicia_grasp_modes.observe_return_remote import ObserveReturnRemoteMixin
    class Node(ObserveReturnRemoteMixin, remote.RemoteGrasp6DNode):
        pass
    values, _ = bound_fixture()
    node, source, prepared, runtime, _, _, calls, _, _ = values
    # Retain the existing real geometry/height-search fixture and its synthetic
    # strict transport; add the production mode override to its method chain.
    node.__class__ = Node
    node._contact_yield_search = dict(key='exact-request', incumbent_found=False,
        fallback=False, deferred_height_families=[])
    node._contact_yield_request_key = lambda p: 'exact-request'
    node._contact_yield_measured_opportunity = lambda p: measured_opportunity
    selected = select(values)
    assert selected.plan_id != source.plan_id
    assert len(calls) == 2
    assert [operation for _,operation,_ in calls] == [
        'final_registered_strict_sequence', 'final_registered_height_window_strict_sequence']
    rebuilt = deepcopy(source)
    node._strict_check_final_near_field_plan(rebuilt, prepared, runtime)
    assert rebuilt == selected and len(calls) == 3
    assert all(deadline == 123. for _,_,deadline in calls)
    assert runtime['contact_height_window_preferred_revalidation']['strict_result_reused'] is False
    assert node._contact_yield_search['deferred_height_families'] == []
