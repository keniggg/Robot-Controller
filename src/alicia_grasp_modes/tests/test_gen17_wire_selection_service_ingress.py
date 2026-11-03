"""Wire-format failure passes actual precheck parsing before yield collection.

The GEN17 decimal-string timestamp form is applied to a coherent synthetic
callback timeline. This does not replay GEN17 geometry or failed trajectories.
Exact original selection values are separately retained in the recorded fixture.
"""
from copy import deepcopy
import json
from types import SimpleNamespace as NS

import pytest

from test_remote_modes import configured, base
from test_contact_search_yield_reached_gain import reached_gain_context
from alicia_flexible_grasp.grasp.grasp6d_pipeline import bounded_moveit_select


@pytest.mark.parametrize('fault', ['none', 'wrong_plan', 'foreign_stamp', 'float_stamp'])
def test_wire_failure_is_parsed_and_bound_before_actual_collector(configured, monkeypatch, fault):
    node, prepared, clock, _, candidates = reached_gain_context(configured, monkeypatch)
    node.contact_sequence_precheck_enabled = True
    prepared.ticket.snapshot_stamp_sec = prepared.snapshot.stamp_ns * 1e-9
    # A newly prepared request captures the remote's current internal phase,
    # not the incoming task message's distinct phase id used by the gain fixture.
    prepared.near_field_phase_id = node._near_field_phase_id
    prepared.near_field_phase_deadline_sec = node._near_field_phase_deadline_sec
    calls = []
    results = []

    def service(request):
        calls.append(deepcopy(request))
        selection = deepcopy(node._mode_selection)
        selection['stamp_ns'] = str(selection['stamp_ns'])
        if fault == 'foreign_stamp': selection['stamp_ns'] = str(int(selection['stamp_ns']) + 1)
        elif fault == 'float_stamp': selection['stamp_ns'] = float(selection['stamp_ns'])
        evidence = dict(plan_id=request.plan.plan_id,
            scope='read_only_candidate_constant_bias_prediction',
            execution_authorized=False, success=False, selection=selection,
            failed_stage='grasp', failure_code='CONTACT_SEQUENCE_PRECHECK_FAILED',
            reason='predicted_command continuous geometry: target OBB collision at 12-end/right_finger',
            source_sample=dict(sdk_stamp_ns=clock[0]-100_000_000,
                               accepted_stamp_ns=clock[0]-90_000_000))
        if fault == 'wrong_plan': evidence['plan_id'] = 'f' * 24
        return NS(success=False, failure_code=evidence['failure_code'],
            failed_stage=evidence['failed_stage'], message=evidence['reason'],
            evidence_json=json.dumps(evidence), joint_path_cost=0., joint_max_delta=0.)

    monkeypatch.setattr(base.rospy, 'wait_for_service', lambda *args, **kwargs: None)
    monkeypatch.setattr(base.rospy, 'ServiceProxy', lambda *args, **kwargs: service)

    def strict(candidate):
        clock[0] += 2_000_000_000
        runtime = node._stable_variant_runtime[(candidate.track_id, candidate.variant_index)]
        plan = base.Grasp6DPlan()
        plan.model_choice = 'unknown_tabletop'
        plan.plan_id = '%024x' % candidate.track_id
        result, _ = node._final_contact_sequence_evaluation(plan, (), prepared, runtime,
            'final_registered_strict_sequence',
            deadline_sec=node._direct_near_field_candidate_deadline_sec(prepared, runtime))
        results.append((result.failure_code, result.reason))
        return result

    monkeypatch.setattr(node, '_check_direct_registered_candidate', strict)
    checker, gate = node._direct_near_field_comparison_search(prepared, candidates)
    selection = bounded_moveit_select(candidates, checker, exhaustive=True,
        ranking_key=lambda c: (c.track_id, c.variant_index), continue_checking=gate,
        continuation_stop_reason='NEAR_FIELD_DIRECT_TIMEOUT')
    state = node._contact_yield_search
    assert len(calls) == len(selection.checked) == 4, results
    assert all(request.deadline.to_sec() <= prepared.near_field_phase_deadline_sec for request in calls)
    expected = 4 if fault == 'none' else 0
    assert len(state['original_families']) == len(state['collision_receipts']) == expected, {
        'results': results,
        'marker_keys': [sorted(k for k in runtime if k.startswith('validated'))
                        for runtime in node._stable_variant_runtime.values()],
    }
    assert state['stopped'] is False  # Completed families do not end a fast search.
    for receipt in state['collision_receipts']:
        assert receipt['plan_id'] in {request.plan.plan_id for request in calls}
        assert receipt['snapshot_stamp_ns'] == prepared.snapshot.stamp_ns
        assert receipt['execution_authorized'] is False
    if fault == 'wrong_plan':
        assert all('validated_contact_precheck_failure' not in runtime
                   for runtime in node._stable_variant_runtime.values())
    assert node._observe_return_pending_supplement is None
    assert not any(plan.valid for plan in node.rich_plan_pub.messages)
