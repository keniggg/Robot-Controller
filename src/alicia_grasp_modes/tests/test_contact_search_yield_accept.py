"""Actual failure audit commit and ModeAware terminal-to-view handoff.

Registration and bounded selection use their real callbacks. Only physical
strict transport is simulated. This is not an original GEN16 path replay.
"""
from copy import deepcopy
import json

import pytest

from test_remote_modes import configured, Publisher
from test_contact_search_yield_reached_gain import reached_gain_context
from test_contact_search_yield_remote import search
from alicia_grasp_modes.contact_search_yield import CODE, contact_search_yield_proof_valid


@pytest.mark.parametrize('expire_after_commit', [False, True])
def test_actual_failure_audit_commit_precedes_mode_terminal_handoff(
        configured, monkeypatch, tmp_path, expire_after_commit):
    node, prepared, clock, _, candidates = reached_gain_context(configured, monkeypatch)
    prepared.ticket.snapshot_stamp_sec = prepared.snapshot.stamp_ns * 1e-9
    base_report = deepcopy(node._active_gate_audit_report)
    selection, calls, retries = search(node, prepared, clock, candidates, monkeypatch)
    assert calls == [1, 2] and retries == []
    assert selection.termination_reason == CODE
    # The prior report lacks a typed proof. Only the actual builder/writer's
    # commit below may establish the report consumed by the terminal override.
    node._active_gate_audit_report = deepcopy(base_report)
    assert node._observe_return_completed_contact_failure(prepared, CODE) is None
    node.gate_audit_enabled = True
    node.gate_audit_output_path = str(tmp_path / 'planning.json')
    node.mujoco_audit_output_path = str(tmp_path / 'mujoco.json')
    node.gate_audit_pub = Publisher()
    lifecycle = []
    original_identity = node._current_stream_target_identity()
    original_generation = node._stream_generation
    original_deadline = node._observe_return_session['deadline_ns']

    def commit_invalid():
        lifecycle.append(deepcopy(node._active_gate_audit_report))
        # This is the original pre-terminal lifecycle callback, not a stub.
        node._observe_execution_candidate_invalid(ticket=prepared.ticket)

    report, reference = node._finalize_streaming_gate_audit(
        prepared, selection, base_report['pipeline_funnel'], CODE,
        base_report=base_report, lifecycle_commit_callback=commit_invalid)
    persisted = json.loads((tmp_path / 'planning.json').read_text())
    assert persisted == report == lifecycle[0] == node._active_gate_audit_report
    assert reference['atomic_committed']
    assert node.gate_audit_pub.messages
    proof = report['moveit_selection']['contact_search_yield']
    assert contact_search_yield_proof_valid(proof)
    assert proof['checked_count'] == 2 and proof['shortlist_count'] == 4
    assert report['outcome']['code'] == CODE
    assert not report['outcome']['valid_plan']
    assert node._current_stream_target_identity() == original_identity
    assert node._stream_generation == original_generation
    assert node._observe_return_pending_supplement is None
    if expire_after_commit:
        clock[0] = proof['yield_evidence']['original_phase_deadline_ns']
    node._publish_direct_near_field_terminal(prepared, CODE, 'bounded contact search yielded')
    if expire_after_commit:
        assert node._observe_return_pending_supplement is None
        assert node._observe_return_phase == 'observing_contact'
    else:
        pending = node._observe_return_pending_supplement
        assert pending['failed_search']['contact_search_code'] == CODE
        assert pending['failed_search']['yield_evidence']['transition_ns'] == clock[0]
        assert node._observe_return_phase == 'observation_prefix'
        assert pending['deadline_ns'] == original_deadline
        assert not pending['contact_or_motion_authorized']
    assert not any(plan.valid for plan in node.rich_plan_pub.messages)
