"""Actual read-only precheck ingress must validate new failure provenance.

Transport is stubbed, but the real request, phase checks, JSON parsing and
result handling run. These tests do not certify a trajectory or a robot move.
"""
from copy import deepcopy
import json

import pytest

from test_contact_precheck_phase_cancellation import context, check


FAILURE = 'validated_contact_precheck_failure'
PLAN = 'validated_contact_precheck_plan_id'
COLLISION = 'predicted_command continuous geometry: target OBB collision at 12-end/left_finger'


def failed_response(response, plan):
    evidence = dict(scope='read_only_candidate_constant_bias_prediction',
        plan_id=plan.plan_id, execution_authorized=False, success=False,
        failed_stage='grasp', failure_code='CONTACT_SEQUENCE_PRECHECK_FAILED',
        reason=COLLISION, source_sample=dict(sdk_stamp_ns=100_000_000_000,
                                         accepted_stamp_ns=100_000_000_000))
    response.success = False
    response.failure_code = evidence['failure_code']
    response.failed_stage = evidence['failed_stage']
    response.message = evidence['reason']
    response.evidence_json = json.dumps(evidence)
    return evidence


def stale_runtime():
    return {FAILURE: dict(plan_id='old', success=False), PLAN: 'old'}


def test_real_precheck_failure_is_bound_to_sent_plan_and_frozen(monkeypatch):
    node, prepared, plan, _, calls, _, _, response = context(monkeypatch)
    evidence = failed_response(response, plan)
    runtime = stale_runtime()
    result, _ = check(node, prepared, plan, 190., runtime)
    assert not result.reachable
    assert result.failure_code == 'CONTACT_SEQUENCE_PRECHECK_FAILED'
    assert len(calls) == 1 and calls[0].plan == plan
    assert runtime[PLAN] == calls[0].plan.plan_id
    assert runtime[FAILURE] == evidence
    assert runtime[FAILURE] is not runtime['contact_sequence_precheck']
    runtime['contact_sequence_precheck']['source_sample']['sdk_stamp_ns'] = 1
    assert runtime[FAILURE]['source_sample']['sdk_stamp_ns'] == 100_000_000_000
    assert runtime[FAILURE]['execution_authorized'] is False


@pytest.mark.parametrize('fault', [
    'wrong_plan', 'wrong_scope', 'claims_execution', 'success_disagrees',
    'non_boolean_success', 'failure_code_disagrees', 'stage_disagrees',
    'reason_disagrees', 'invalid_json', 'transport_failure', 'late_response',
])
def test_unbound_or_late_response_cannot_relabel_old_failure(monkeypatch, fault):
    node, prepared, plan, clock, calls, _, proxy, response = context(monkeypatch)
    evidence = failed_response(response, plan)
    if fault == 'wrong_plan': evidence['plan_id'] = 'b' * 24
    elif fault == 'wrong_scope': evidence['scope'] = 'nominal_only'
    elif fault == 'claims_execution': evidence['execution_authorized'] = True
    elif fault == 'success_disagrees': evidence['success'] = True
    elif fault == 'non_boolean_success': response.success = 0
    elif fault == 'failure_code_disagrees': evidence['failure_code'] = 'MOVEIT_TIMEOUT'
    elif fault == 'stage_disagrees': evidence['failed_stage'] = 'approach'
    elif fault == 'reason_disagrees': evidence['reason'] = 'old collision'
    response.evidence_json = json.dumps(evidence)
    if fault == 'invalid_json': response.evidence_json = '{'
    if fault in ('transport_failure', 'late_response'):
        def service(request):
            calls.append(deepcopy(request))
            if fault == 'transport_failure': raise RuntimeError('read-only service failed')
            clock.value = 190.
            return response
        proxy.return_value = service
    runtime = stale_runtime()
    result, _ = check(node, prepared, plan, 190., runtime)
    assert not result.reachable and len(calls) == 1
    assert FAILURE not in runtime and PLAN not in runtime


@pytest.mark.parametrize('boundary', ['expired_before_call', 'success'])
def test_no_current_collision_clears_preexisting_failure_marker(monkeypatch, boundary):
    node, prepared, plan, clock, calls, _, _, _ = context(monkeypatch)
    if boundary == 'expired_before_call': clock.value = 190.
    runtime = stale_runtime()
    result, _ = check(node, prepared, plan, 190., runtime)
    assert result.reachable is (boundary == 'success')
    assert len(calls) == (1 if boundary == 'success' else 0)
    assert FAILURE not in runtime and PLAN not in runtime
