"""Completed geometry rejection can request evidence, never contact execution.

The height evidence is computed by the real placement function on a local
unit fixture. This is not a replay or physical success claim for attempt 6.
"""
from copy import deepcopy
from dataclasses import replace
from pathlib import PurePosixPath
from types import SimpleNamespace as NS

import pytest

from alicia_flexible_grasp.grasp.stable_recheck_contract import (
    CODE, completed_stable_recheck_proof_valid)
from alicia_grasp_modes.active_observation_policy import evidence_digest
from alicia_grasp_modes.runner_contract import observe_return_audit_error
from test_stable_recheck_contract import completed_proof
from test_observe_return_supplement_remote import continuation, configured
from test_observe_return_supplement_task import context, begin, supplement


BINDING = ('request_id', 'generation', 'target_epoch', 'snapshot_stamp_ns')
REQUEST_FIELDS = ('session_id', 'view_index', 'previous_view_index', 'origin_plan_id',
    'origin_snapshot_stamp_ns', 'target_epoch', 'target_track_id', 'deadline_ns',
    'max_observations', 'failed_search', 'visual_opportunity', 'previous_gain',
    'previous_gain_sha256', 'contact_or_motion_authorized')


def proof_for(source):
    proof = completed_proof(**{key: source[key] for key in BINDING})
    assert completed_stable_recheck_proof_valid(proof)
    return proof


def current_request(configured, monkeypatch):
    node, prepared = continuation(configured, monkeypatch)
    report = node._active_gate_audit_report
    proof = proof_for(report)
    report['outcome'] = {'code': CODE}
    report['stable_recheck'] = deepcopy(proof['stable_recheck'])
    report['pipeline_funnel']['stage_counts']['locally_valid'] = deepcopy(proof['locally_valid'])
    return node, prepared, proof


def test_completed_recheck_preserves_local_success_and_zero_path_checks(configured, monkeypatch):
    node, prepared, proof = current_request(configured, monkeypatch)
    source = deepcopy(node._active_gate_audit_report)
    deadline = node._observe_return_session['deadline_ns']
    assert node._observe_return_completed_contact_failure(prepared, CODE) == proof
    assert node._observe_return_prepare_supplement(prepared, CODE)
    request = node._observe_return_pending_supplement
    assert request['failed_search'] == proof
    assert request['visual_opportunity']['completed_contact_search'] == proof
    assert request['failed_search']['locally_valid']['passed'] == 6
    assert request['failed_search']['checked_count'] == 0
    assert request['failed_search']['stable_recheck']['rejected_count'] == 12
    assert not request['contact_or_motion_authorized']
    assert request['deadline_ns'] == deadline
    assert node._observe_return_phase == 'observation_prefix'
    # The proof copied out of the finished request must not change its input.
    assert node._active_gate_audit_report == source


@pytest.mark.parametrize('fault', ['outcome', 'audit_source', 'missing_ledger',
    'ledger_source', 'ledger_incomplete', 'unknown_failure', 'checked_path',
    'local_count', 'no_gap', 'no_gain', 'expired', 'registration', 'max_views', 'transport'])
def test_new_classification_needs_complete_current_proof_and_independent_view_evidence(
        configured, monkeypatch, fault):
    node, prepared, _ = current_request(configured, monkeypatch)
    report = node._active_gate_audit_report
    ledger = report['stable_recheck']
    if fault == 'outcome': report['outcome']['code'] = 'NEAR_FIELD_NO_REACHABLE_CANDIDATE'
    elif fault == 'audit_source': report['snapshot_stamp_ns'] -= 1
    elif fault == 'missing_ledger': del report['stable_recheck']
    elif fault == 'ledger_source': ledger['snapshot_stamp_ns'] -= 1
    elif fault == 'ledger_incomplete': ledger['completed'] = False
    elif fault == 'unknown_failure': ledger['variants'][0]['code'] = 'ValueError'
    elif fault == 'checked_path': report['moveit_selection'].update(checked_count=1, shortlist_count=1)
    elif fault == 'local_count':
        report['pipeline_funnel']['stage_counts']['locally_valid'].update(entered=7, passed=7, rejected=0)
    elif fault == 'no_gap': prepared.remote_diagnostics['tabletop_geometry']['surface_evidence_rejections'] = []
    elif fault == 'no_gain': node._observe_return_step_gain['ok'] = False
    elif fault == 'expired': node._observe_return_session['deadline_ns'] = prepared.snapshot.stamp_ns
    elif fault == 'registration':
        evidence = node._latest_registration_evidence
        node._latest_registration_evidence = replace(evidence, stamp_ns=evidence.stamp_ns-1)
    elif fault == 'max_views': node._observe_return_session['view_index'] = 3
    elif fault == 'transport': prepared.remote_failure_code = 'WSL_UNAVAILABLE'
    assert not node._observe_return_prepare_supplement(prepared, CODE)
    assert node._observe_return_pending_supplement is None
    assert node._observe_return_phase == 'observing_contact'


def test_old_classifications_cannot_launder_the_new_zero_path_shape(configured, monkeypatch):
    node, prepared, _ = current_request(configured, monkeypatch)
    for code in ('NEAR_FIELD_NO_REACHABLE_CANDIDATE', 'NEAR_FIELD_NO_HARD_SAFE_CANDIDATE'):
        assert node._observe_return_completed_contact_failure(prepared, code) is None
        assert node._observe_return_mixed_visual_failure(prepared, code) is None


@pytest.mark.parametrize('code', ['NEAR_FIELD_NO_HARD_SAFE_CANDIDATE',
                                'NEAR_FIELD_NO_REACHABLE_CANDIDATE'])
@pytest.mark.parametrize('ledger', [None, {'incomplete_count': 0},
                                  {'incomplete_count': 1}, {'incomplete_count': False}])
def test_legacy_terminal_cannot_hide_an_incomplete_stable_variant(
        configured, monkeypatch, code, ledger):
    node, prepared = continuation(configured, monkeypatch)
    report = node._active_gate_audit_report
    if code == 'NEAR_FIELD_NO_REACHABLE_CANDIDATE':
        report['moveit_selection'].update(checked_count=1, shortlist_count=1)
        report['pipeline_funnel']['stage_counts']['locally_valid'] = dict(entered=2, passed=2, rejected=0)
    if ledger is not None:
        report['stable_recheck'] = deepcopy(ledger)
    rejected = ledger is not None and (type(ledger['incomplete_count']) is not int
                                       or ledger['incomplete_count'] > 0)
    assert (node._observe_return_completed_contact_failure(prepared, code) is None) is rejected
    assert (node._observe_return_mixed_visual_failure(prepared, code) is None) is rejected
    if code == 'NEAR_FIELD_NO_HARD_SAFE_CANDIDATE':
        node._observe_return_phase = 'initial_contact'
        assert node._observe_return_precheck_visual_failure(prepared, code) is not rejected


def test_height_rejection_does_not_expand_silhouette_diagnostic_authority(configured, monkeypatch):
    node, prepared, _ = current_request(configured, monkeypatch)
    node._observe_return_phase = 'initial_contact'
    monkeypatch.setattr(node, '_observe_return_completed_contact_failure',
        lambda *_: pytest.fail('silhouette must reject this classification before proof dispatch'))
    assert node._observe_return_silhouette_failure(prepared, CODE) is None


def mutate_proof(proof, fault):
    if fault == 'missing_ledger': del proof['stable_recheck']
    elif fault == 'ledger_source': proof['stable_recheck']['snapshot_stamp_ns'] -= 1
    elif fault == 'incomplete': proof['stable_recheck']['completed'] = False
    elif fault == 'unknown_failure': proof['stable_recheck']['variants'][0]['code'] = 'ValueError'
    elif fault == 'checked_path': proof.update(checked_count=1, shortlist_count=1)
    elif fault == 'authority': proof['contact_or_motion_authorized'] = True


def install_proof(metadata, proof, fault=''):
    mutate_proof(proof, fault)
    metadata['failed_search'] = proof
    opportunity = metadata['visual_opportunity']
    opportunity.update({key: proof[key] for key in BINDING + ('contact_search_code', 'checked_count')})
    opportunity['completed_contact_search'] = deepcopy(proof)
    if fault == 'missing_copy': del opportunity['completed_contact_search']
    elif fault == 'changed_copy': opportunity['completed_contact_search']['stable_recheck']['variants'][0]['reason'] += ' changed'
    elif fault == 'opportunity_count': opportunity['checked_count'] = 1
    elif fault == 'bool_opportunity_count': opportunity['checked_count'] = False
    metadata['view_request_id'] = evidence_digest({key: metadata[key] for key in REQUEST_FIELDS})


@pytest.mark.parametrize('fault', ['', 'missing_ledger', 'ledger_source', 'incomplete',
    'unknown_failure', 'checked_path', 'authority', 'missing_copy', 'changed_copy',
    'opportunity_count', 'bool_opportunity_count'])
def test_task_validates_same_complete_rejection_after_request_digest_is_recomputed(context, fault):
    c = context
    begin(c)
    c.now[0] += 2_000_000_000
    plan, report = supplement(c, 2)
    metadata = report['observe_return']
    install_proof(metadata, proof_for(metadata['failed_search']), fault)
    if fault:
        with pytest.raises(ValueError, match='SUPPLEMENTAL_EVIDENCE_INVALID'):
            c.node._observe_return_supplement_report(plan, report)
    else:
        assert c.node._observe_return_supplement_report(plan, report) is metadata


@pytest.mark.parametrize('policy,fault', [
    ('mixed', ''), ('completed', ''), ('mixed', 'missing'), ('completed', 'missing'),
    ('mixed', 'ledger_source'), ('completed', 'ledger_source'),
    ('mixed', 'checked_path'), ('completed', 'incomplete'), ('completed', 'changed_copy')])
def test_measured_runner_requires_bound_complete_recheck_for_both_trigger_shapes(
        configured, monkeypatch, tmp_path, policy, fault):
    from test_measured_observation_planning import independent_context
    node, prepared, _, _ = independent_context(configured, monkeypatch, tmp_path)
    assert node._accept_prediction(prepared)['status'] == 'PREVIEW_READY'
    report = deepcopy(node._active_execution_audit_report)
    trigger = report['observe_return']['trigger_evidence']
    proof = proof_for(trigger)
    mutate_proof(proof, fault)
    trigger['completed_contact_search'] = deepcopy(proof)
    if fault == 'missing': del trigger['completed_contact_search']
    elif fault == 'changed_copy': trigger['completed_contact_search']['stable_recheck']['variants'][0]['reason'] += ' changed'
    if policy == 'mixed':
        trigger.update(policy='current_request_independent_bilateral_view_gap',
            contact_search_code=CODE, checked_count=proof['checked_count'])
    else:
        trigger.update(policy='current_request_completed_contact_view_deficit',
            contact_search_completed=True, initial_contact_search=proof)
        trigger.pop('evidence_sha256')
        trigger['evidence_sha256'] = evidence_digest(trigger)
    assert bool(observe_return_audit_error(report)) is bool(fault)


@pytest.mark.parametrize('fault', ['', 'missing_ledger', 'ledger_source', 'incomplete',
    'checked_path', 'missing_copy', 'changed_copy'])
def test_supplemental_runner_checks_ledger_after_exact_pending_request_binding(
        configured, monkeypatch, tmp_path, fault):
    from test_supplement_preview_audit_handoff import continuation as planner, check
    node, prepared, _, _ = planner(configured, monkeypatch, tmp_path)
    pending = node._observe_return_pending_supplement
    install_proof(pending, proof_for(pending['failed_search']))
    node._observe_return_trigger_evidence = deepcopy(pending['visual_opportunity'])
    assert node._accept_prediction(prepared)['status'] == 'PREVIEW_READY'
    report = deepcopy(node._active_gate_audit_report)
    assert check(node, report) == ''
    if fault:
        metadata = report['observe_return']
        install_proof(metadata, deepcopy(metadata['failed_search']), fault)
        metadata['trigger_evidence'] = deepcopy(metadata['visual_opportunity'])
        node._observe_return_pending_supplement = {
            key: deepcopy(metadata[key]) for key in REQUEST_FIELDS + ('view_request_id',)}
        assert check(node, report)


@pytest.fixture
def pure_task_context(monkeypatch):
    """Only the task audit consumer; no file callback, service or session send."""
    from test_observe_return_task import evidence, at_ns, module
    selection, report, plan = evidence('observation_prefix')
    node = module.ModeAwareGraspTaskNode.__new__(module.ModeAwareGraspTaskNode)
    node._initialize_mode_boundary()
    node._mode_bound_selection = deepcopy(selection)
    now = [24_000_000_000]
    monkeypatch.setattr(module.rospy.Time, 'now', staticmethod(lambda: at_ns(now[0])))
    node._observe_return_session = dict(version=2, phase='observing',
        session_id='f'*64, selection=deepcopy(selection),
        origin_plan_id=plan.plan_id, origin_snapshot_stamp_ns=report['snapshot_stamp_ns'],
        target_epoch=report['target_epoch'], target_track_id=plan.target_track_id,
        deadline_ns=200_000_000_000, max_observations=3, view_index=1,
        view_plan_id=plan.plan_id, view_source_stamp_ns=report['snapshot_stamp_ns'])
    return NS(node=node, now=now)


@pytest.mark.parametrize('fault', ['', 'missing_ledger', 'changed_copy', 'bool_opportunity_count'])
def test_pure_task_consumer_without_audit_file_or_gateway(pure_task_context, fault):
    c = pure_task_context
    plan, report = supplement(c, 2)
    metadata = report['observe_return']
    install_proof(metadata, proof_for(metadata['failed_search']), fault)
    if fault:
        with pytest.raises(ValueError, match='SUPPLEMENTAL_EVIDENCE_INVALID'):
            c.node._observe_return_supplement_report(plan, report)
    else:
        assert c.node._observe_return_supplement_report(plan, report) is metadata


def capture_uncommitted_audit(node, monkeypatch):
    """Keep the real report builder; stop before persistence and promotion.

    No file reference or atomic-commit receipt is invented. This intentionally
    does not test the production writer, its hash binding or motion authority.
    The disk-backed tests above retain those existing integration boundaries.
    """
    captured = []

    def build(prepared, selection, proposal, local, moveit, stable_count, status,
              base_report, **kwargs):
        funnel = node._merge_pipeline_funnel(local, moveit,
            stable_count=stable_count, preview_count=1,
            stable_candidates=kwargs.get('stable_candidates', ()),
            preview_candidate=selection.selected)
        report, _, _ = node._build_streaming_gate_audit_report(
            prepared, selection, funnel, status, base_report=base_report)
        captured.append(deepcopy(report))
        return funnel, None

    monkeypatch.setattr(node, '_finalize_promotion_transaction', build)
    return captured


@pytest.mark.parametrize('policy,fault', [('mixed', ''), ('completed', ''),
    ('mixed', 'ledger_source'), ('completed', 'changed_copy')])
def test_pure_measured_consumer_uses_real_report_builder_without_persistence(
        configured, monkeypatch, policy, fault):
    from test_measured_observation_planning import independent_context
    node, prepared, _, _ = independent_context(configured, monkeypatch,
        PurePosixPath('/offline-test-not-persisted'))
    reports = capture_uncommitted_audit(node, monkeypatch)
    assert node._accept_prediction(prepared)['status'] == 'PREVIEW_READY'
    assert len(reports) == 1
    report = reports[0]
    trigger = report['observe_return']['trigger_evidence']
    proof = proof_for(trigger)
    mutate_proof(proof, fault)
    trigger['completed_contact_search'] = deepcopy(proof)
    if fault == 'changed_copy':
        trigger['completed_contact_search']['stable_recheck']['variants'][0]['reason'] += ' changed'
    if policy == 'mixed':
        trigger.update(policy='current_request_independent_bilateral_view_gap',
            contact_search_code=CODE, checked_count=0)
    else:
        trigger.update(policy='current_request_completed_contact_view_deficit',
            contact_search_completed=True, initial_contact_search=proof)
        trigger.pop('evidence_sha256')
        trigger['evidence_sha256'] = evidence_digest(trigger)
    assert bool(observe_return_audit_error(report)) is bool(fault)


@pytest.mark.parametrize('fault', ['', 'missing_ledger', 'ledger_source', 'changed_copy'])
def test_pure_supplemental_consumer_uses_real_report_builder_without_persistence(
        configured, monkeypatch, fault):
    from test_supplement_preview_audit_handoff import continuation as planner, check
    node, prepared, _, _ = planner(configured, monkeypatch,
        PurePosixPath('/offline-test-not-persisted'))
    pending = node._observe_return_pending_supplement
    install_proof(pending, proof_for(pending['failed_search']))
    node._observe_return_trigger_evidence = deepcopy(pending['visual_opportunity'])
    reports = capture_uncommitted_audit(node, monkeypatch)
    assert node._accept_prediction(prepared)['status'] == 'PREVIEW_READY'
    assert len(reports) == 1
    report = reports[0]
    assert check(node, report) == ''
    if fault:
        metadata = report['observe_return']
        install_proof(metadata, deepcopy(metadata['failed_search']), fault)
        metadata['trigger_evidence'] = deepcopy(metadata['visual_opportunity'])
        node._observe_return_pending_supplement = {
            key: deepcopy(metadata[key]) for key in REQUEST_FIELDS + ('view_request_id',)}
        assert check(node, report)
