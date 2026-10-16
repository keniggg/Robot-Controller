"""Success-first scheduling, retained v1 wire history, and exact v2 evidence."""
from copy import deepcopy
from types import SimpleNamespace as NS
import pytest

from test_contact_search_yield import fixture_proof
from test_contact_search_yield_remote import ready, search, base
from test_remote_modes import configured
from alicia_grasp_modes.contact_search_yield import CODE, contact_search_yield_proof_valid


def v2_proof():
    p=fixture_proof();p['proof_version']=2
    p.update(checked_count=4,shortlist_count=5,locally_valid=dict(entered=5,passed=5,rejected=0))
    e=p['yield_evidence'];e.pop('maximum_original_families')
    e['original_families'].append(dict(track_id=3,variant_index=0,family_sha256='f'*64))
    e.update(policy='contact_search_budget_checkpoint_v2',minimum_completed_families=1,
             exhausted_by='budget_checkpoint',yielded_ns=e['budget_deadline_ns'],
             transition_ns=e['budget_deadline_ns']+1,completed_family_count=3,
             attempted_family_count=4,unchecked_count=1,
             family_evidence_scope='at_least_one_bound_failed_precheck_not_exhaustive',
             unchecked_count_scope='original_shortlist_candidates',height_search_exhaustiveness_claimed=False)
    e['attempted_families']=deepcopy(e['original_families'])+[dict(track_id=4,variant_index=0,family_sha256='1'*64)]
    return p


def test_v2_preserves_more_than_two_completed_families_and_incomplete_counts():
    p=v2_proof()
    assert contact_search_yield_proof_valid(p)
    assert len(p['yield_evidence']['original_families'])==3
    assert p['yield_evidence']['unchecked_count']==1
    assert p['contact_or_motion_authorized'] is False


@pytest.mark.parametrize('version',[None,True,False,0,3,'2',2.,-1])
def test_unknown_or_ambiguous_version_cannot_reinterpret_legacy_proof(version):
    p=fixture_proof();p['proof_version']=version
    assert not contact_search_yield_proof_valid(p)


def test_legacy_absent_or_explicit_version_one_keeps_original_wire_rules():
    p=fixture_proof();assert contact_search_yield_proof_valid(p)
    p['proof_version']=1;assert contact_search_yield_proof_valid(p)
    p['yield_evidence']['original_families'].append(dict(track_id=3,variant_index=0,family_sha256='f'*64))
    assert not contact_search_yield_proof_valid(p)


@pytest.mark.parametrize('fault',[
 'early','forged_early_budget','wrong_exhaustion','legacy_limit','wrong_completed_count','wrong_attempted_count',
 'wrong_unchecked','bool_count','unattempted_completed','duplicate_attempt','zero_completed',
 'wrong_scope','claims_height_exhaustive','selected','no_collision','foreign_receipt',
 'missing_feedback','expired','duration_extended','no_reserve','v1_disguised_as_v2'])
def test_v2_rejects_unproven_checkpoint_or_ownership_or_counts(fault):
    p=v2_proof();e=p['yield_evidence'];r=e['collision_receipts'][0]
    if fault=='early':e['yielded_ns']=e['budget_deadline_ns']-1
    elif fault=='forged_early_budget':e['budget_deadline_ns']-=1;e['yielded_ns']=e['budget_deadline_ns']
    elif fault=='wrong_exhaustion':e['exhausted_by']='original_family_limit'
    elif fault=='legacy_limit':e['maximum_original_families']=2
    elif fault=='wrong_completed_count':e['completed_family_count']-=1
    elif fault=='wrong_attempted_count':e['attempted_family_count']-=1
    elif fault=='wrong_unchecked':e['unchecked_count']+=1
    elif fault=='bool_count':e['completed_family_count']=True
    elif fault=='unattempted_completed':e['attempted_families'][2]['family_sha256']='2'*64
    elif fault=='duplicate_attempt':e['attempted_families'][3]=deepcopy(e['attempted_families'][0])
    elif fault=='zero_completed':e['original_families']=[];e['completed_family_count']=0
    elif fault=='wrong_scope':e['unchecked_count_scope']='all_height_alternatives'
    elif fault=='claims_height_exhaustive':e['height_search_exhaustiveness_claimed']=True
    elif fault=='selected':p['selected']=True
    elif fault=='no_collision':e['collision_receipts']=[]
    elif fault=='foreign_receipt':r['request_id']+=1
    elif fault=='missing_feedback':r.pop('accepted_stamp_ns')
    elif fault=='expired':e['yielded_ns']=e['original_phase_deadline_ns']
    elif fault=='duration_extended':e['maximum_duration_sec']=31.
    elif fault=='no_reserve':e['original_phase_deadline_ns']=e['budget_deadline_ns']+1
    elif fault=='v1_disguised_as_v2':p=fixture_proof();p['proof_version']=2
    assert not contact_search_yield_proof_valid(p)


def test_fast_two_families_are_only_milestone_and_all_height_calls_remain(configured,monkeypatch):
    node,p,clock,candidates=ready(configured,monkeypatch,candidate_count=5)
    selection,calls,retries=search(node,p,clock,candidates,monkeypatch)
    assert calls==[1,2,3,4,5] and len(retries)==5
    assert not selection.terminated_early and selection.termination_reason!=CODE
    assert not node._contact_yield_search['fallback']
    assert node._contact_yield_search['deferred_height_families']==[]


def test_checkpoint_keeps_all_three_families_and_real_two_unchecked_then_prepares_view(configured,monkeypatch):
    node,p,clock,candidates=ready(configured,monkeypatch,candidate_count=5)
    selection,calls,retries=search(node,p,clock,candidates,monkeypatch,seconds=[10.,10.,10.])
    assert calls==[1,2,3] and len(retries)==3
    assert selection.termination_reason==CODE
    proof=node._observe_return_completed_contact_failure(p,CODE)
    assert contact_search_yield_proof_valid(proof)
    e=proof['yield_evidence']
    assert e['completed_family_count']==e['attempted_family_count']==3
    assert e['unchecked_count']==2 and len(e['original_families'])==3
    assert e['yielded_ns']==e['budget_deadline_ns']
    assert node._observe_return_prepare_supplement(p,CODE)
    assert node._observe_return_pending_supplement['failed_search']['proof_version']==2
    assert node._observe_return_pending_supplement['contact_or_motion_authorized'] is False


def test_normal_height_retry_forwards_recipe_and_skip_indices_without_success_stub(configured,monkeypatch):
    node,p,clock,candidates=ready(configured,monkeypatch)
    node._contact_yield_begin(p,candidates)
    calls=[];sentinel=object()
    def original(self,*args,**kwargs):
        calls.append((args,kwargs));return sentinel
    monkeypatch.setattr(base.RemoteGrasp6DNode,'_contact_height_window_retry',original)
    plan=NS(plan_id='a'*24);runtime=node._stable_variant_runtime[(1,0)]
    result=node._contact_height_window_retry(plan,p,runtime,None,31.,preferred_recipe={'i':2},skip_proposal_indices=(0,))
    assert result is sentinel and len(calls)==1
    assert calls[0][1]==dict(preferred_recipe={'i':2},skip_proposal_indices=(0,))
    assert 'contact_height_window_search' not in runtime


def test_height_trial_clearing_receipt_cannot_be_counted_as_completed(configured,monkeypatch):
    node,p,clock,candidates=ready(configured,monkeypatch)
    def clear_actual_runtime(runtime):
        runtime.pop('validated_contact_precheck_failure',None)
        runtime.pop('validated_contact_precheck_plan_id',None)
    selection,calls,retries=search(node,p,clock,candidates,monkeypatch,
        seconds=[15.,15.],height_hook=clear_actual_runtime)
    state=node._contact_yield_search
    assert state['fallback'] and state['first_pass_checked']==2
    assert not state['original_families'] and not state['collision_receipts']
    assert selection.termination_reason!=CODE and len(retries)==len(calls)


def test_multiple_variants_of_one_family_can_yield_at_checkpoint(configured,monkeypatch):
    node,p,clock,candidates=ready(configured,monkeypatch,candidate_count=4)
    original=deepcopy(node._stable_variant_runtime[(1,0)]['sequence'])
    for runtime in node._stable_variant_runtime.values():
        runtime['sequence']=deepcopy(original)
    selection,calls,retries=search(node,p,clock,candidates,monkeypatch,seconds=[30.])
    assert calls==[1] and len(retries)==1 and selection.termination_reason==CODE
    proof=node._observe_return_completed_contact_failure(p,CODE)
    assert contact_search_yield_proof_valid(proof)
    assert len(proof['yield_evidence']['original_families'])==1
    assert proof['yield_evidence']['unchecked_count']==3


def test_one_family_before_checkpoint_keeps_searching_and_complete_shortlist_is_not_yield(configured,monkeypatch):
    node,p,clock,candidates=ready(configured,monkeypatch,candidate_count=2)
    selection,calls,retries=search(node,p,clock,candidates,monkeypatch,seconds=[15.,15.])
    assert calls==[1,2] and len(retries)==2
    assert not selection.terminated_early and selection.termination_reason!=CODE
    assert node._observe_return_completed_contact_failure(p,CODE) is None


def test_zero_bound_failure_receipts_at_checkpoint_preserve_original_phase_fallback(configured,monkeypatch):
    node,p,clock,candidates=ready(configured,monkeypatch)
    selection,calls,retries=search(node,p,clock,candidates,monkeypatch,results=['timeout']*6,seconds=[30.])
    assert node._contact_yield_search['fallback']
    assert not node._contact_yield_search['original_families']
    assert not node._contact_yield_search['collision_receipts']
    assert selection.termination_reason!=CODE
    assert p.near_field_phase_deadline_sec>clock[0]*1e-9


from test_observe_return_task import context as task_context


@pytest.mark.parametrize('family_count',[1,3])
def test_real_preview_builder_runner_and_task_consume_version_two(
        configured,task_context,monkeypatch,tmp_path,family_count):
    """Consumer transaction uses a phase-reserve-limited synthetic checkpoint.

    The real producer's full 30-second schedule is tested above. This separate
    builder fixture's next RGBD sample is 100ms later, so keep its old phase
    reserve boundary before that new source; never move a checkpoint after its
    supplemental preview. Geometry and strict transport are existing unit inputs.
    """
    import test_contact_search_yield as transaction
    from alicia_grasp_modes.runner_contract import supplemental_observation_audit_error
    old_typed=transaction.typed_proof
    def typed(proof):
        p=old_typed(proof);p['proof_version']=2
        p.update(checked_count=family_count,shortlist_count=family_count+1,
                 locally_valid=dict(entered=family_count+1,passed=family_count+1,rejected=0))
        e=p['yield_evidence'];e.pop('maximum_original_families')
        e['original_families']=[dict(track_id=i+1,variant_index=0,family_sha256=('%064x'%(i+1))) for i in range(family_count)]
        checkpoint=p['snapshot_stamp_ns']+10_000_000
        e.update(policy='contact_search_budget_checkpoint_v2',minimum_completed_families=1,
                 family_evidence_scope='at_least_one_bound_failed_precheck_not_exhaustive',
                 unchecked_count_scope='original_shortlist_candidates',height_search_exhaustiveness_claimed=False,
                 completed_family_count=family_count,attempted_family_count=family_count,
                 attempted_families=deepcopy(e['original_families']),unchecked_count=1,
                 exhausted_by='budget_checkpoint',budget_deadline_ns=checkpoint,
                 original_phase_deadline_ns=checkpoint+2_000_000_000,
                 yielded_ns=checkpoint,transition_ns=checkpoint+1)
        assert contact_search_yield_proof_valid(p)
        return p
    monkeypatch.setattr(transaction,'typed_proof',typed)
    node,prepared,report,proof=transaction.build_typed(configured,monkeypatch,tmp_path)
    assert proof['yield_evidence']['transition_ns']<report['observe_return']['view_source_stamp_ns']
    assert supplemental_observation_audit_error(report,session=node._observe_return_session,
        pending_request=node._observe_return_pending_supplement,now_ns=int(node._ros_source_clock_ns()))==''
    consumer=task_context.node
    consumer._mode_bound_selection=deepcopy(node._mode_selection)
    consumer._observe_return_session=deepcopy(node._observe_return_session)
    consumer._observe_return_audit_cb(node.gate_audit_pub.messages[-1])
    plan=node.latest_preview_rich_plan
    loaded=consumer._observe_return_reports[plan.plan_id]
    assert loaded==report
    assert consumer._observe_return_supplement_report(plan,loaded)['failed_search']==proof
    assert not task_context.gateways
    assert not any(plan.valid for plan in node.rich_plan_pub.messages)


def test_same_geometry_later_variant_receipt_belongs_to_attempted_family(configured,monkeypatch):
    node,p,clock,candidates=ready(configured,monkeypatch,candidate_count=4)
    original=deepcopy(node._stable_variant_runtime[(1,0)]['sequence'])
    for runtime in node._stable_variant_runtime.values():
        runtime['sequence']=deepcopy(original)
    selection,calls,retries=search(node,p,clock,candidates,monkeypatch,
        results=['margin','collision'],seconds=[15.,15.])
    assert calls==[1,2] and len(retries)==2 and selection.termination_reason==CODE
    proof=node._observe_return_completed_contact_failure(p,CODE)
    assert contact_search_yield_proof_valid(proof)
    e=proof['yield_evidence']
    assert e['attempted_family_count']==e['completed_family_count']==1
    assert e['attempted_families'][0]['track_id']==1
    assert e['original_families'][0]['track_id']==2
    assert e['attempted_families'][0]['family_sha256']==e['original_families'][0]['family_sha256']
    assert proof['checked_count']==e['unchecked_count']==2
