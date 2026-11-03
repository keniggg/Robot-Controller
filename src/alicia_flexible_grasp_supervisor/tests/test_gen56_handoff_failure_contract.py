"""GEN56 handoff failures and the original execution transaction contract.

The legacy promotion fixture supplies real controller, audit writer, token
checks and compensation. Only transport/clock failure boundaries are injected.
The first three cases explain the old swallowed-False behavior: the old
finalizer returned normally after losing the exact audit or current owner.
They are behavior checks, not a wall-time performance benchmark.
"""
import json
from pathlib import Path

import pytest

from test_remote_grasp6d_streaming import (
    promotion_transaction_node, remote_node, FailingPublisher)
from test_gen56_contact_bundle_independent import (
    station, context, station_transaction, base as mode_base)
from test_remote_modes import configured as observation_configured
from test_supplement_preview_audit_handoff import continuation as supplement_continuation


def finalize(setup, **kwargs):
    node, prepared, selection, proposal, local, moveit = setup
    return node._finalize_promotion_transaction(
        prepared, selection, proposal, local, moveit, 1, 'PREVIEW_READY',
        {'summary': {}, 'rows': []}, **kwargs)


@pytest.mark.parametrize('failure', ['phase_expired', 'owner_cancelled'])
def test_exact_preview_audit_failure_is_reported_instead_of_false_ready(
        tmp_path, monkeypatch, failure):
    setup = promotion_transaction_node(tmp_path)
    node = setup[0]
    node.robot_execution_active = True  # Real controller selects preview-only.
    error = (remote_node.CandidateContractError(
                'TARGET_LOST', 'station_compute_wait_expired_or_future')
             if failure == 'phase_expired' else
             remote_node.StreamResultCancelled('GENERATION_STALE'))
    attempts = []

    def audit_boundary(*args, **kwargs):
        attempts.append(kwargs['execution_authority'])
        raise error

    monkeypatch.setattr(node, '_finalize_streaming_gate_audit', audit_boundary)
    try:
        with pytest.raises(type(error)) as caught:
            finalize(setup)
        assert caught.value is error
        assert attempts == [False]
        assert not node.plan_pub.messages and not node.rich_plan_pub.messages
        assert not node.execution_plan_controller.has_execution
    finally:
        node.shutdown_streaming_worker()


def test_owner_revoked_after_decision_does_not_return_preview_ready(tmp_path, monkeypatch):
    setup = promotion_transaction_node(tmp_path)
    node = setup[0]
    node.robot_execution_active = True
    original_capture = node._capture_promotion_token_locked

    def capture_then_cancel(*args):
        answer = original_capture(*args)
        node.stop_streaming()
        return answer

    monkeypatch.setattr(node, '_capture_promotion_token_locked', capture_then_cancel)
    try:
        with pytest.raises(remote_node.StreamResultCancelled):
            finalize(setup)
        assert not node.execution_plan_controller.has_execution
        assert not any(getattr(message, 'valid', False)
                       for message in node.rich_plan_pub.messages)
    finally:
        node.shutdown_streaming_worker()


def test_compensated_execution_rejection_carries_exact_preview_audit_failure(
        tmp_path, monkeypatch):
    setup = promotion_transaction_node(tmp_path)
    node, prepared = setup[:2]
    node.plan_pub = FailingPublisher()
    real_finalize = node._finalize_streaming_gate_audit

    def fail_preview_after_compensation(*args, **kwargs):
        if kwargs.get('execution_authority') is False:
            assert node._active_execution_audit_report['outcome']['valid_plan'] is False
            raise remote_node.CandidateContractError(
                'PLANNING_AUDIT_FAILED', 'current owner lost its exact preview audit')
        return real_finalize(*args, **kwargs)

    monkeypatch.setattr(node, '_finalize_streaming_gate_audit', fail_preview_after_compensation)
    try:
        funnel, actual = finalize(setup)
        node._require_stream_ticket_current(prepared.ticket)
        assert actual.promote is False and actual.code == 'PLAN_PUBLICATION_FAILED'
        assert funnel['preview_audit_failure'] == {
            'code': 'PLANNING_AUDIT_FAILED',
            'reason': 'current owner lost its exact preview audit'}
        assert not node.execution_plan_controller.has_execution
        assert not any(message.valid for message in node.rich_plan_pub.messages)
    finally:
        node.shutdown_streaming_worker()


@pytest.mark.parametrize('supersede_during_audit', [False, True])
def test_single_observed_context_retains_execution_audit_and_original_token(
        tmp_path, monkeypatch, supersede_during_audit):
    setup = promotion_transaction_node(tmp_path)
    node, prepared, _selection, proposal, _local, _moveit = setup
    decisions, writes, sends = [], [], []
    original_decide = node._decide_preview_promotion
    original_finalize = node._finalize_streaming_gate_audit
    original_send = node.rich_plan_pub.publish

    def decide(*args):
        decisions.append(args)
        return original_decide(*args)

    def write(*args, **kwargs):
        result = original_finalize(*args, **kwargs)
        if kwargs.get('execution_authority'):
            report, reference = result
            on_disk = json.loads(Path(reference['report_path']).read_text())
            assert on_disk['plan_id'] == report['plan_id']
            assert on_disk['promotion'] == report['promotion']
            assert report['plan_id'] == proposal['rich_plan'].plan_id
            assert report['promotion']['promote'] is True
            writes.append(reference)
            if supersede_during_audit:
                node.execution_plan_controller.invalid_streak += 1
        return result

    def send(message):
        if message.valid:
            assert len(writes) == 1
            sends.append(message.plan_id)
        return original_send(message)

    monkeypatch.setattr(node, '_decide_preview_promotion', decide)
    monkeypatch.setattr(node, '_finalize_streaming_gate_audit', write)
    monkeypatch.setattr(node.rich_plan_pub, 'publish', send)
    try:
        with node._stream_condition, node._geometry_state_guard():
            decision, now = node._decide_preview_promotion(
                proposal['rich_plan'], proposal['signature'], proposal['score'])
            token = node._capture_promotion_token_locked(
                prepared.ticket, proposal['expected_generation'])
        assert decision.promote is True
        funnel, actual = finalize(setup, promotion_context=(decision, now, token))
        assert len(decisions) == 1 and len(writes) == 1
        assert actual.promote is (not supersede_during_audit)
        assert funnel['stage_counts']['promoted']['passed'] == int(actual.promote)
        if supersede_during_audit:
            assert actual.code == 'PROMOTION_SUPERSEDED'
            assert sends == [] and not node.execution_plan_controller.has_execution
            assert node._active_execution_audit_report['promotion']['promote'] is False
        else:
            assert sends == [proposal['rich_plan'].plan_id]
            assert node.execution_plan_controller.execution_plan_id == sends[0]
    finally:
        node.shutdown_streaming_worker()


@pytest.mark.parametrize('fault', [
    'missing_audit_current_owner', 'cancelled_owner', 'published_wire',
    'report_owner', 'atomic_reference', 'none'])
def test_audited_acceptance_requires_current_owner_and_committed_publication(
        station, monkeypatch, tmp_path, fault):
    """Real early-audit/publication path consumes a typed rejection boundary.

    The old and preceding tests cover the real execution compensation. Here
    its explicit negative result is the boundary input so the actual caller's
    handling is exercised without granting this station fixture motion rights.
    """
    env = station_transaction(station, monkeypatch, tmp_path)
    node = env.node
    aborts = []
    original_abort = node._abort_station_contact_preview_audit

    def abort(ticket):
        aborts.append(ticket)
        return original_abort(ticket)

    monkeypatch.setattr(node, '_abort_station_contact_preview_audit', abort)
    monkeypatch.setattr(node._promotion_controller(), 'observe_preview',
        lambda *args, **kwargs: mode_base.PromotionDecision(
            True, 'BRANCH_PROVIDER', 'explicit promotable boundary input'))
    executions = []

    def completed_rejection(*args, **kwargs):
        node._require_stream_ticket_current(env.ticket)
        assert node.latest_preview_rich_plan == env.bundle['rich_plan']
        assert node._latest_gate_audit_reference['atomic_committed'] is True
        executions.append(kwargs['promotion_context'])
        if fault == 'cancelled_owner':
            node.stop_streaming()
            return {}, mode_base.PromotionDecision(False, 'GENERATION_STALE', 'cancelled')
        if fault == 'missing_audit_current_owner':
            funnel = {'preview_audit_failure': {
                'code': 'PLANNING_AUDIT_FAILED', 'reason': 'post-compensation preview audit failed'}}
        else:
            funnel = {}
            if fault == 'published_wire':
                node.latest_preview_rich_plan.poses[0].position.x += .01
            elif fault == 'report_owner':
                node._active_gate_audit_report['request_id'] += 1
            elif fault == 'atomic_reference':
                node._latest_gate_audit_reference['atomic_committed'] = False
            else:
                assert fault == 'none'
        return funnel, mode_base.PromotionDecision(False, 'PLAN_PUBLICATION_FAILED', 'compensated')

    monkeypatch.setattr(node, '_finalize_promotion_transaction', completed_rejection)
    if fault == 'none':
        assert env.run()['status'] == 'PREVIEW_READY'
    else:
        error = (mode_base.StreamResultCancelled if fault == 'cancelled_owner'
                 else mode_base.CandidateContractError)
        with pytest.raises(error):
            env.run()
    assert len(executions) == 1
    # These failures happen after a completely committed preview. A later
    # execution decision must not roll back that earlier delivered fact.
    assert aborts == []
    if fault == 'missing_audit_current_owner':
        # Revocation is a consequence of the failure, not the reason the
        # required marker was noticed. The provider verified owner validity.
        assert executions[0][0].promote is True
    assert sum(bool(m.valid) for m in node.rich_plan_pub.messages) == env.valid_execution_before


def test_task_prepare_before_publisher_returns_preserves_completed_handoff(
        station, monkeypatch, tmp_path):
    """Force takeover after atomic send/fact commit, before caller bookkeeping."""
    from types import SimpleNamespace
    from test_gen56_contact_bundle_independent import contact_prepare_request, Publisher, String

    env = station_transaction(station, monkeypatch, tmp_path)
    node = env.node
    node._contact_station_ack_pub = Publisher()
    real_publish = node._publish_preview_plan
    consumed = []

    def consume_before_return(*args, **kwargs):
        result = real_publish(*args, **kwargs)
        request = contact_prepare_request(station, env.bundle['rich_plan'])
        node.grasp_state_cb(SimpleNamespace(state='MOVE_PREGRASP', active=True))
        node._contact_station_request_cb(String(data=json.dumps(request)))
        response = json.loads(node._contact_station_ack_pub.messages[-1].data)
        assert response['accepted'] is True, response
        consumed.append(request)
        return result

    monkeypatch.setattr(node, '_publish_preview_plan', consume_before_return)
    assert env.run()['status'] == 'PREVIEW_READY'
    assert len(consumed) == 1 and node._contact_station_prepared == consumed[0]
    proof = node._station_compute_work[env.ticket.request_id]
    assert proof['contact_published'] is True and proof['contact_audit']
    assert node._station_contact_audited_proposals(consumed[0])
    assert node._latest_preview_promotion[:3] == (
        env.ticket.request_id, env.ticket.generation, env.ticket.target_epoch)
    assert node._latest_preview_promotion[3].promote is False
    assert not node._station_compute_retired
    assert sum(bool(m.valid) for m in node.rich_plan_pub.messages) == env.valid_execution_before


@pytest.mark.parametrize('promotable', [False, True])
def test_invalid_supplement_audit_returns_exact_refusal_without_ready(
        observation_configured, monkeypatch, tmp_path, promotable):
    """Use the original failing continuation and actual audit/finalizer."""
    node, prepared, _, _ = supplement_continuation(
        observation_configured, monkeypatch, tmp_path)
    node.robot_execution_active = not promotable
    node._observe_return_pending_supplement['view_request_id'] = '0' * 64
    decisions = []
    decide = node._decide_preview_promotion

    def capture_decision(*args, **kwargs):
        result = decide(*args, **kwargs)
        decisions.append(result[0])
        return result

    monkeypatch.setattr(node, '_decide_preview_promotion', capture_decision)
    result = node._accept_prediction(prepared)
    assert decisions and decisions[0].promote is promotable
    assert result['status'] == 'UNKNOWN_OBSERVATION_AUDIT_INVALID'
    assert 'pending request changed' in result['reason']
    assert result['funnel']['preview_audit_failure']['code'] == result['status']
    node._require_stream_ticket_current(prepared.ticket)
    assert not (tmp_path/'planning.json').exists()
    assert not (tmp_path/'planning.json.execution').exists()
    assert not node.gate_audit_pub.messages
    assert not any(plan.valid for plan in node.rich_plan_pub.messages)
    assert node._execution_authority_ticket is None


@pytest.mark.parametrize('fault', [
    'owner_cancelled', 'source_expired', 'camera_contract', 'audit_io'])
def test_observation_audit_refusal_does_not_hide_owner_source_or_io_failure(
        observation_configured, monkeypatch, tmp_path, fault):
    node, prepared, _, _ = supplement_continuation(
        observation_configured, monkeypatch, tmp_path)
    expected = {
        'owner_cancelled': 'GENERATION_STALE',
        'source_expired': 'OBSERVATION_SOURCE_RESERVE_REACHED',
        'camera_contract': 'UNKNOWN_OBSERVATION_VIEW_SOURCE_INVALID',
        'audit_io': 'PLANNING_AUDIT_FAILED',
    }[fault]
    if fault == 'audit_io':
        def disk_failure(*args, **kwargs):
            raise mode_base.CandidateContractError(
                'PLANNING_AUDIT_FAILED', 'injected audit writer failure')
        monkeypatch.setattr(node, '_write_gate_audit_report', disk_failure)
    else:
        node._observe_return_pending_supplement['view_request_id'] = '0' * 64
        build = node._build_diagnostic_observation_gate_audit

        def change_during_audit(*args, **kwargs):
            try:
                return build(*args, **kwargs)
            except mode_base.CandidateContractError as exc:
                assert exc.code == 'UNKNOWN_OBSERVATION_AUDIT_INVALID'
                if fault == 'owner_cancelled':
                    node.stop_streaming()
                elif fault == 'source_expired':
                    expiry = (prepared.ticket.snapshot_stamp_sec +
                              node._configured_execution_plan_validity_sec())
                    monkeypatch.setattr(node, '_stream_source_clock', lambda: expiry)
                else:
                    # The existing fixture substitutes the camera transport.
                    # Fail that source boundary after the real audit rejects,
                    # then exercise the original solution revalidation caller.
                    def camera_changed(_prepared):
                        raise ValueError('camera intrinsics changed during audit')
                    monkeypatch.setattr(node, '_require_diagnostic_observation_view_source',
                                        camera_changed)
                raise

        monkeypatch.setattr(node, '_build_diagnostic_observation_gate_audit',
                            change_during_audit)
    with pytest.raises((mode_base.CandidateContractError,
                        mode_base.StreamResultCancelled)) as caught:
        node._accept_prediction(prepared)
    assert caught.value.code == expected
    assert not (tmp_path/'planning.json').exists()
    assert not any(plan.valid for plan in node.rich_plan_pub.messages)
    assert node._execution_authority_ticket is None


def test_compensated_observation_refusal_marker_cannot_become_ready(
        observation_configured, monkeypatch, tmp_path):
    node, prepared, _, _ = supplement_continuation(
        observation_configured, monkeypatch, tmp_path)
    node.robot_execution_active = False
    publish = node.plan_pub.publish
    attempted = []

    def fail_first_execution(message):
        if not attempted:
            attempted.append(message)
            node._observe_return_pending_supplement['view_request_id'] = '0' * 64
            raise RuntimeError('injected execution transport failure')
        return publish(message)

    monkeypatch.setattr(node.plan_pub, 'publish', fail_first_execution)
    result = node._accept_prediction(prepared)
    assert attempted
    assert result['status'] == 'UNKNOWN_OBSERVATION_AUDIT_INVALID'
    assert result['funnel']['preview_audit_failure']['code'] == result['status']
    assert not (tmp_path/'planning.json').exists()
    compensated = json.loads((tmp_path/'planning.json.execution').read_text())
    assert compensated['outcome']['valid_plan'] is False
    assert not any(plan.valid for plan in node.rich_plan_pub.messages)
    assert node._execution_authority_ticket is None


def test_observation_audit_error_after_execution_is_not_an_unexecuted_refusal(
        observation_configured, monkeypatch, tmp_path):
    """Inert publishers record an actual authority commit before late refusal."""
    node, prepared, _, _ = supplement_continuation(
        observation_configured, monkeypatch, tmp_path)
    node.robot_execution_active = False
    build = node._build_diagnostic_observation_gate_audit
    late = []

    def reject_after_commit(*args, **kwargs):
        if kwargs.get('execution_authority') is False:
            assert node._execution_authority_ticket == node._execution_authority_record(
                prepared.ticket)
            late.append(True)
            node._observe_return_pending_supplement['view_request_id'] = '0' * 64
        return build(*args, **kwargs)

    monkeypatch.setattr(node, '_build_diagnostic_observation_gate_audit', reject_after_commit)
    with pytest.raises(mode_base.CandidateContractError) as caught:
        node._accept_prediction(prepared)
    assert late and caught.value.code == 'UNKNOWN_OBSERVATION_AUDIT_INVALID'
    assert any(plan.valid for plan in node.rich_plan_pub.messages)
    assert node._request_promotion_count(prepared.ticket) == 1
