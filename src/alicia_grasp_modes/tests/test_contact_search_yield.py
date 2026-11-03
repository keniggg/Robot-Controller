"""Typed incomplete-search proof and real preview/audit consumer boundaries."""
from copy import deepcopy
import json

import pytest

from alicia_grasp_modes.contact_search_yield import CODE, contact_search_yield_proof_valid
from alicia_grasp_modes.active_observation_policy import evidence_digest
from alicia_grasp_modes.runner_contract import supplemental_observation_audit_error, observe_return_audit_error
from alicia_grasp_modes import observe_return_task as task
from test_remote_modes import configured
from test_observe_return_task import context as task_context
from test_supplement_preview_audit_handoff import continuation


def typed_proof(proof):
    """Synthetic bounded schedule receipt; not a recorded GEN16 execution."""
    proof = deepcopy(proof)
    source = proof['snapshot_stamp_ns']
    proof.update(contact_search_code=CODE, selected=False, terminated_early=True,
        checked_count=2, shortlist_count=3, reachable_count=0,
        locally_valid=dict(entered=4, passed=3, rejected=1), contact_or_motion_authorized=False)
    receipt = {key: proof[key] for key in ('request_id', 'generation', 'target_epoch', 'snapshot_stamp_ns')}
    receipt.update(plan_id='c'*24, scope='read_only_candidate_constant_bias_prediction',
        success=False, execution_authorized=False, failed_stage='grasp',
        failure_code='CONTACT_SEQUENCE_PRECHECK_FAILED',
        reason='predicted_command continuous geometry: target OBB collision at 12-end/left_finger',
        evidence_sha256='a'*64, sdk_stamp_ns=source+1, accepted_stamp_ns=source+1)
    proof['yield_evidence'] = dict(policy='bounded_contact_search_for_measured_view',
        phase_id=1, original_phase_deadline_ns=source+60_000_000_000,
        started_ns=source+1, yielded_ns=source+10, transition_ns=source+11,
        budget_deadline_ns=source+30_000_000_001,
        maximum_duration_sec=30., maximum_original_families=2, exhausted_by='original_family_limit',
        original_families=[dict(track_id=1, variant_index=0, family_sha256='d'*64),
                           dict(track_id=2, variant_index=0, family_sha256='e'*64)],
        collision_receipts=[receipt], incumbent_found=False)
    return proof


def replace_pending_with_typed(node):
    pending=node._observe_return_pending_supplement
    proof=typed_proof(pending['failed_search'])
    opportunity=deepcopy(pending['visual_opportunity'])
    opportunity.update(contact_search_code=CODE, checked_count=proof['checked_count'],
                       yielded_contact_search=deepcopy(proof))
    pending.update(failed_search=proof, visual_opportunity=opportunity)
    pending.pop('view_request_id',None)
    pending['view_request_id']=evidence_digest(pending)
    node._observe_return_trigger_evidence=deepcopy(opportunity)
    return proof


def fixture_proof():
    return typed_proof(dict(request_id=13,generation=13,target_epoch=1,snapshot_stamp_ns=1791718687000939369))


def test_typed_proof_preserves_incomplete_counts_and_no_authority():
    proof=fixture_proof()
    assert contact_search_yield_proof_valid(proof)
    assert proof['terminated_early'] is True
    assert proof['checked_count'] < proof['shortlist_count']
    assert proof['contact_or_motion_authorized'] is False
    proof['yield_evidence']['exhausted_by']='shared_time_budget'
    assert not contact_search_yield_proof_valid(proof)
    proof['yield_evidence']['yielded_ns']=proof['yield_evidence']['budget_deadline_ns']
    proof['yield_evidence']['transition_ns']=proof['yield_evidence']['yielded_ns']
    assert contact_search_yield_proof_valid(proof)


@pytest.mark.parametrize('fault',[
    'ordinary_timeout','missing_typed_evidence','claimed_completed','claimed_selected','winner',
    'checked_zero','checked_all','bool_count','local_counts','too_few_local_passes','family_missing','family_duplicate',
    'same_original_family','over_family_budget','no_collision','old_request','old_generation',
    'old_source','foreign_target','nominal_collision','different_stage','success','execution',
    'old_feedback','future_feedback','unhashed','source_after_start','expired_phase',
    'extended_duration','extended_budget','no_phase_reserve','late_transition','bool_transition','unknown_exhaustion','bool_phase','incumbent'])
def test_untyped_or_foreign_or_expired_proof_cannot_be_used(fault):
    p=fixture_proof();e=p['yield_evidence'];r=e['collision_receipts'][0]
    if fault=='ordinary_timeout':p['contact_search_code']='NEAR_FIELD_DIRECT_TIMEOUT'
    elif fault=='missing_typed_evidence':del p['yield_evidence']
    elif fault=='claimed_completed':p['terminated_early']=False
    elif fault=='claimed_selected':p['selected']=True
    elif fault=='winner':p['reachable_count']=1
    elif fault=='checked_zero':p['checked_count']=0
    elif fault=='checked_all':p['checked_count']=p['shortlist_count']
    elif fault=='bool_count':p['checked_count']=True
    elif fault=='local_counts':p['locally_valid']['entered']+=1
    elif fault=='too_few_local_passes':p['locally_valid'].update(entered=4,passed=1,rejected=3)
    elif fault=='family_missing':e['original_families'].pop()
    elif fault=='family_duplicate':e['original_families'][1]['family_sha256']=e['original_families'][0]['family_sha256']
    elif fault=='same_original_family':e['original_families'][1]['track_id']=1
    elif fault=='over_family_budget':e['maximum_original_families']=3
    elif fault=='no_collision':e['collision_receipts']=[]
    elif fault=='old_request':r['request_id']-=1
    elif fault=='old_generation':r['generation']-=1
    elif fault=='old_source':r['snapshot_stamp_ns']-=1
    elif fault=='foreign_target':r['target_epoch']+=1
    elif fault=='nominal_collision':r['reason']=r['reason'].replace('predicted_command','nominal')
    elif fault=='different_stage':r['failed_stage']='approach'
    elif fault=='success':r['success']=True
    elif fault=='execution':r['execution_authorized']=True
    elif fault=='old_feedback':r['sdk_stamp_ns']=p['snapshot_stamp_ns']-1
    elif fault=='future_feedback':r['accepted_stamp_ns']=e['yielded_ns']+1
    elif fault=='unhashed':r['evidence_sha256']=''
    elif fault=='source_after_start':e['started_ns']=p['snapshot_stamp_ns']-1
    elif fault=='expired_phase':e['yielded_ns']=e['original_phase_deadline_ns']
    elif fault=='extended_duration':e['maximum_duration_sec']=31.
    elif fault=='extended_budget':e['budget_deadline_ns']=e['started_ns']+30_000_000_001
    elif fault=='no_phase_reserve':e['original_phase_deadline_ns']=e['budget_deadline_ns']+1
    elif fault=='late_transition':e['transition_ns']=e['original_phase_deadline_ns']
    elif fault=='bool_transition':e['transition_ns']=True
    elif fault=='unknown_exhaustion':e['exhausted_by']='controller_timeout'
    elif fault=='bool_phase':e['phase_id']=True
    elif fault=='incumbent':e['incumbent_found']=True
    assert not contact_search_yield_proof_valid(p)


def build_typed(configured,monkeypatch,tmp_path):
    node,prepared,checks,_=continuation(configured,monkeypatch,tmp_path)
    proof=replace_pending_with_typed(node)
    result=node._accept_prediction(prepared)
    assert result['status']=='PREVIEW_READY'
    assert checks and all(observation for _,observation in checks)
    report=json.loads((tmp_path/'planning.json').read_text())
    return node,prepared,report,proof


def test_real_selected_preview_report_and_task_accept_only_typed_continuation(
        configured,task_context,monkeypatch,tmp_path):
    node,prepared,report,proof=build_typed(configured,monkeypatch,tmp_path)
    assert supplemental_observation_audit_error(report,session=node._observe_return_session,
        pending_request=node._observe_return_pending_supplement,
        now_ns=int(node._ros_source_clock_ns()))==''
    assert observe_return_audit_error(report)  # Still not an external initial attempt.
    assert not any(plan.valid for plan in node.rich_plan_pub.messages)
    consumer=task_context.node
    consumer._mode_bound_selection=deepcopy(node._mode_selection)
    consumer._observe_return_session=deepcopy(node._observe_return_session)
    consumer._observe_return_audit_cb(node.gate_audit_pub.messages[-1])
    plan=node.latest_preview_rich_plan
    loaded=consumer._observe_return_reports[plan.plan_id]
    assert loaded==report
    assert consumer._observe_return_supplement_report(plan,loaded)['failed_search']==proof
    assert not task_context.gateways  # Audit consumption itself submits nothing.


@pytest.mark.parametrize('fault',['missing_proof','mismatch_proof','timeout_only','no_gain',
    'wrong_request','old_receipt','missing_gap','expired_original_phase','missing_transition','initial_entry'])
def test_real_audit_and_task_reject_typed_yield_without_original_gates(
        configured,task_context,monkeypatch,tmp_path,fault):
    node,_,report,_=build_typed(configured,monkeypatch,tmp_path)
    meta=report['observe_return'];failure=meta['failed_search'];opportunity=meta['visual_opportunity']
    if fault=='missing_proof':opportunity.pop('yielded_contact_search')
    elif fault=='mismatch_proof':opportunity['yielded_contact_search']['request_id']+=1
    elif fault=='timeout_only':failure['contact_search_code']='NEAR_FIELD_DIRECT_TIMEOUT'
    elif fault=='no_gain':meta['previous_gain']['registration_ok']=False
    elif fault=='wrong_request':opportunity['request_id']+=1
    elif fault=='old_receipt':failure['yield_evidence']['collision_receipts'][0]['snapshot_stamp_ns']-=1
    elif fault=='missing_gap':meta['measured_missing_sides']=[]
    elif fault=='expired_original_phase':
        failure['yield_evidence']['original_phase_deadline_ns']=failure['yield_evidence']['yielded_ns']
    elif fault=='missing_transition':failure['yield_evidence'].pop('transition_ns')
    elif fault=='initial_entry':meta['phase']='observation_prefix'
    # Rehash the wrapper: invalid inner evidence cannot be rescued by a fresh wrapper digest.
    fields=('session_id','view_index','previous_view_index','origin_plan_id','origin_snapshot_stamp_ns',
        'target_epoch','target_track_id','deadline_ns','max_observations','failed_search',
        'visual_opportunity','previous_gain','previous_gain_sha256','contact_or_motion_authorized')
    meta['view_request_id']=evidence_digest({key:meta[key] for key in fields})
    assert supplemental_observation_audit_error(report,session=node._observe_return_session,
        pending_request=node._observe_return_pending_supplement,now_ns=int(node._ros_source_clock_ns()))
    consumer=task_context.node;consumer._mode_bound_selection=deepcopy(node._mode_selection)
    consumer._observe_return_session=deepcopy(node._observe_return_session)
    with pytest.raises(ValueError):consumer._observe_return_supplement_report(node.latest_preview_rich_plan,report)
    assert not task_context.gateways


def test_completed_yield_handoff_does_not_reapply_expired_contact_deadline_to_new_view(
        configured,task_context,monkeypatch,tmp_path):
    node,_,report,proof=build_typed(configured,monkeypatch,tmp_path)
    old_deadline=proof['yield_evidence']['original_phase_deadline_ns']
    now=old_deadline+1
    assert now < node._observe_return_session['deadline_ns']
    assert supplemental_observation_audit_error(report,session=node._observe_return_session,
        pending_request=node._observe_return_pending_supplement,now_ns=now)==''
    consumer=task_context.node;consumer._mode_bound_selection=deepcopy(node._mode_selection)
    consumer._observe_return_session=deepcopy(node._observe_return_session)
    task_context.now[0]=now
    assert consumer._observe_return_supplement_report(node.latest_preview_rich_plan,report)
    assert not task_context.gateways
