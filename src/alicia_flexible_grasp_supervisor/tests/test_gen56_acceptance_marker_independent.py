"""A compensated execution refusal must not conceal missing preview evidence.

The original acceptance entrypoint, promotion controller, execution transaction,
atomic audit writer and compensation execute. Upstream perception/selection and
preview transport are explicit providers; this is not another geometry replay.
The live-ticket checks are never replaced. No ROS or device transport is used.
"""
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from test_remote_grasp6d_streaming import (
    BoundedMoveItSelection,
    FailingPublisher,
    TargetTrackIdentity,
    promotion_transaction_node,
    remote_node,
)


@pytest.mark.parametrize('audit_failure_code', [
    None, 'PLANNING_AUDIT_FAILED', 'TARGET_LOST',
], ids=['committed_audit_control', 'audit_failed_current_owner',
        'phase_failed_current_owner'])
def test_real_acceptance_checks_preview_audit_after_compensated_execution(
        tmp_path, monkeypatch, audit_failure_code):
    node, previous, selected, proposal, local, moveit = promotion_transaction_node(tmp_path)
    ticket = previous.ticket
    candidate = selected.selected
    prepared = remote_node.PreparedPrediction(
        ticket=ticket,
        snapshot=SimpleNamespace(
            object_msg=SimpleNamespace(detected=True, label='carton'),
            target_identity=TargetTrackIdentity.from_stream(0, ticket.target_epoch)),
        stamp=remote_node.rospy.Time.from_sec(ticket.snapshot_stamp_sec),
        geometry=object(), pose_estimator=object(), graspnet_input=object(),
        candidates=(), remote_diagnostics={}, remote_performance={},
        model_choice='carton_segment', near_field=False)

    # These providers stop before the transaction under test; they do not
    # replace its owner check, final audit, publication failure or compensation.
    node.moveit_top_n = 1
    node.tracker = SimpleNamespace(config=SimpleNamespace(min_hits=3))
    node._activate_prepared_geometry = lambda _prepared: True
    node._run_candidate_gate_audit = lambda *args, **kwargs: {'summary': {}, 'rows': []}
    node._append_normalized_geometry_audit_rows = lambda report, *args: report
    node._evaluate_local_candidates = lambda _prepared: ((candidate.stable_candidate,), local)
    node._update_candidate_tracker = lambda **kwargs: (candidate.stable_candidate,)
    node._recheck_and_score_stable = lambda _prepared, _stable: (candidate,)
    node._select_far_field_observation = lambda reachable: reachable[0]
    node._stable_variant_runtime[(candidate.track_id, candidate.variant_index)]['prepared'] = prepared

    def selection_provider(candidates, checker, **kwargs):
        assert tuple(candidates) == (candidate,)
        return BoundedMoveItSelection(
            selected=candidate, checked=(candidate,), reachable=(candidate,),
            configured_top_n=1, terminated_early=False, termination_reason='',
            funnel=SimpleNamespace(to_dict=lambda: moveit))

    monkeypatch.setattr(remote_node, 'bounded_moveit_select', selection_provider)
    preview_requests = []

    def preview_provider(chosen, ticket=None):
        assert chosen is candidate and ticket is prepared.ticket
        preview_requests.append(chosen)
        return proposal

    node._publish_selected_preview = preview_provider
    # The original execution transaction handles a real transport exception,
    # invalidates publication and writes the corrected execution report.
    node.plan_pub = FailingPublisher()
    original_audit = node._finalize_streaming_gate_audit
    audit_attempts = []

    def audit_boundary(*args, **kwargs):
        execution = kwargs['execution_authority']
        audit_attempts.append(execution)
        assert node._stream_ticket_still_current(ticket)
        if not execution and audit_failure_code is not None:
            raise remote_node.CandidateContractError(
                audit_failure_code, 'independent current-owner preview audit failure')
        return original_audit(*args, **kwargs)

    monkeypatch.setattr(node, '_finalize_streaming_gate_audit', audit_boundary)
    original_transaction = node._finalize_promotion_transaction
    transactions = []

    def transaction(*args, **kwargs):
        answer = original_transaction(*args, **kwargs)
        transactions.append(answer)
        assert node._stream_ticket_still_current(ticket)
        return answer

    monkeypatch.setattr(node, '_finalize_promotion_transaction', transaction)
    try:
        assert node._stream_ticket_still_current(ticket)
        if audit_failure_code is None:
            result = remote_node.RemoteGrasp6DNode._accept_prediction(node, prepared)
            assert result['status'] == 'PREVIEW_READY'
            assert 'preview_audit_failure' not in result['funnel']
            assert node._latest_gate_audit_reference['atomic_committed'] is True
        else:
            with pytest.raises(remote_node.CandidateContractError) as caught:
                remote_node.RemoteGrasp6DNode._accept_prediction(node, prepared)
            assert caught.value.code == audit_failure_code
            assert 'independent current-owner preview audit failure' in str(caught.value)

        assert node._stream_ticket_still_current(ticket)
        assert len(preview_requests) == len(transactions) == 1
        funnel, decision = transactions[0]
        assert decision.promote is False and decision.code == 'PLAN_PUBLICATION_FAILED'
        assert funnel['stage_counts']['promoted']['passed'] == 0
        assert audit_attempts == [True, False]
        if audit_failure_code is not None:
            assert funnel['preview_audit_failure']['code'] == audit_failure_code
        execution = json.loads(Path(node.gate_audit_output_path + '.execution').read_text())
        assert execution['outcome']['valid_plan'] is False
        assert execution['outcome']['code'] == 'PLAN_PUBLICATION_FAILED'
        assert node.execution_plan_controller.execution_plan_id is None
        assert node.rich_plan_pub.messages[-1].valid is False
        assert not any(message.valid for message in node.rich_plan_pub.messages)
    finally:
        node.shutdown_streaming_worker()
