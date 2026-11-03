"""Real bounded-selection and remote handoff, simulated strict transport only."""
from copy import deepcopy
from dataclasses import replace
from types import SimpleNamespace as NS

import pytest

from test_observe_return_supplement_remote import continuation, configured, base
from test_grasp6d_pipeline import scored_candidate
from alicia_flexible_grasp.grasp.grasp6d_pipeline import MoveItResult, bounded_moveit_select
from alicia_grasp_modes.contact_search_yield import CODE, contact_search_yield_proof_valid


def ready(configured, monkeypatch, candidate_count=4):
    node,p=continuation(configured,monkeypatch)
    clock=[p.snapshot.stamp_ns+1_000_000_000]
    node._ros_source_clock_ns=lambda:clock[0]
    node._execution_plan_validity_now_sec=lambda:clock[0]*1e-9
    p.near_field=True;p.near_field_phase_id=4
    p.near_field_phase_deadline_sec=clock[0]*1e-9+90.
    node._near_field_phase_id=4
    # This fixture starts with the already registered phase, unlike the
    # reached-callback test which produces these bindings through its callback.
    node._near_field_surface_phase_id=4
    node._near_field_phase_deadline_sec=p.near_field_phase_deadline_sec
    node.near_field_planning_active=True
    monkeypatch.setattr(node,'_direct_near_field_active',lambda prepared:True)
    node.candidate_max_joint_delta_rad=0.
    candidates=[];runtimes={}
    for i in range(1,candidate_count+1):
        c=scored_candidate(i,.001*i)
        source=dict(request_id=p.ticket.request_id,snapshot_stamp_sec=p.snapshot.stamp_ns*1e-9,
            target_epoch=p.ticket.target_epoch,target_track_id=p.snapshot.target_identity.track_id,
            model_choice='unknown_tabletop')
        c=replace(c,stable_candidate=replace(c.stable_candidate,**source),
            latest_safety=replace(c.latest_safety,**source),
            evaluation_request_id=p.ticket.request_id,
            evaluation_snapshot_stamp_sec=p.snapshot.stamp_ns*1e-9)
        candidates.append(c)
        poses={}
        for j,name in enumerate(('pregrasp','approach','grasp','lift')):
            pose=base.make_pose_stamped('base_link',[.1+i*.001,0.,.2-j*.01],[0.,0.,0.,1.])
            pose.header.stamp=base.rospy.Time.from_sec(p.snapshot.stamp_ns*1e-9)
            poses[name]=pose
        runtimes[(i,0)]=dict(prepared=p,sequence=NS(**poses))
    node._stable_variant_runtime=runtimes
    node._active_gate_audit_report["pipeline_funnel"]["stage_counts"]["locally_valid"]=dict(entered=candidate_count,passed=candidate_count,rejected=0)
    node._direct_near_field_execution_rank_key=lambda c:(c.track_id,c.variant_index)
    return node,p,clock,candidates


def receipt(node,p,clock,plan_id):
    return dict(plan_id=plan_id,scope='read_only_candidate_constant_bias_prediction',
        success=False,execution_authorized=False,failed_stage='grasp',
        failure_code='CONTACT_SEQUENCE_PRECHECK_FAILED',
        reason='predicted_command continuous geometry: target OBB collision at left_finger',
        selection=deepcopy(node._mode_selection),
        source_sample=dict(sdk_stamp_ns=clock[0]-100_000_000,
                           accepted_stamp_ns=clock[0]-90_000_000))


def search(node,p,clock,candidates,monkeypatch, *, results=None, seconds=None, gap_after=None, height_hook=None):
    calls=[];retry_calls=[]
    failure=MoveItResult(False,0.,0.,reason='contact rejected',failure_code='MOVEIT_CHECK_ERROR')
    success=MoveItResult(True,.1,.1,reason="strict test success",collision_free=True,within_joint_limits=True,
                        ik_valid=True,planning_success=True)
    def height(self,plan,*args):
        retry_calls.append(plan.plan_id)
        if height_hook is not None:
            height_hook(args[1])
        return None
    monkeypatch.setattr(base.RemoteGrasp6DNode,'_contact_height_window_retry',height)
    def strict(c):
        index=len(calls);calls.append(c.track_id)
        runtime=node._stable_variant_runtime[(c.track_id,c.variant_index)]
        clock[0]+=int((seconds[index] if seconds and index<len(seconds) else 2.)*1e9)
        kind=results[index] if results and index<len(results) else 'collision'
        plan=NS(plan_id=('%024x'%c.track_id))
        if kind=='collision':
            runtime['validated_contact_precheck_failure']=receipt(node,p,clock,plan.plan_id)
            runtime['validated_contact_precheck_plan_id']=plan.plan_id
        elif kind=='wrong_plan':
            runtime['validated_contact_precheck_failure']=receipt(node,p,clock,'f'*24)
            runtime['validated_contact_precheck_plan_id']=plan.plan_id
        if kind!='success':
            node._contact_height_window_retry(plan,p,runtime,failure,
                node._direct_near_field_candidate_deadline_sec(p,runtime))
        if gap_after is not None and len(calls)==gap_after:
            node._observe_return_step_gain=dict(ok=False)
        return success if kind=='success' else failure
    monkeypatch.setattr(node,'_check_direct_registered_candidate',strict)
    checker,gate=node._direct_near_field_comparison_search(p,candidates=candidates)
    rank=lambda c:(c.track_id,c.variant_index)
    selection=bounded_moveit_select(candidates,checker,exhaustive=True,ranking_key=rank,
        continue_checking=gate,continuation_stop_reason='NEAR_FIELD_DIRECT_TIMEOUT')
    local=deepcopy(node._active_gate_audit_report['pipeline_funnel'])
    selection=node._contact_search_yield_selection(p,selection,candidates=candidates,
        ranking_key=rank,local_funnel=local)
    report=deepcopy(node._active_gate_audit_report)
    report['selected']=None if selection.selected is None else {'winner':True}
    report['moveit_selection']=dict(checked_count=len(selection.checked),
        shortlist_count=selection.shortlist_count,reachable_count=len(selection.reachable),
        terminated_early=selection.terminated_early,termination_reason=selection.termination_reason)
    node._active_gate_audit_report=report
    return selection,calls,retry_calls


def test_two_completed_families_at_checkpoint_yield_to_real_measured_supplement(configured,monkeypatch):
    node,p,clock,candidates=ready(configured,monkeypatch)
    original_deadline=p.near_field_phase_deadline_sec
    selection,calls,retries=search(node,p,clock,candidates,monkeypatch,seconds=[15.,15.])
    assert calls==[1,2] and len(retries)==2
    assert selection.termination_reason==CODE and selection.terminated_early
    proof=node._observe_return_completed_contact_failure(p,CODE)
    assert contact_search_yield_proof_valid(proof)
    assert proof['proof_version']==2
    assert proof['yield_evidence']['exhausted_by']=='budget_checkpoint'
    assert proof['checked_count']==2 and proof['shortlist_count']==4
    assert len(proof['yield_evidence']['original_families'])==2
    assert node._observe_return_prepare_supplement(p,CODE)
    request=node._observe_return_pending_supplement
    assert request['failed_search']['yield_evidence']['transition_ns']==clock[0]
    assert request['visual_opportunity']['yielded_contact_search']==request['failed_search']
    assert node._observe_return_phase=='observation_prefix'
    assert not request['contact_or_motion_authorized']
    assert request['deadline_ns']==node._observe_return_session['deadline_ns']
    assert p.near_field_phase_deadline_sec==original_deadline


@pytest.mark.parametrize('kinds',[
    ['collision','timeout'],['collision','wrong_plan'],['timeout','timeout'],
    ['ik','collision'],['transport','collision']])
def test_incomplete_second_family_or_unbound_receipt_exhausts_only_finite_shortlist(configured,monkeypatch,kinds):
    node,p,clock,candidates=ready(configured,monkeypatch)
    # No later candidate supplies the missing second completed receipt.
    # Early local failure alone no longer exhausts the completed-family budget.
    selection,calls,retries=search(node,p,clock,candidates,monkeypatch,results=kinds+['timeout']*6)
    assert not node._contact_yield_search['fallback']
    assert calls==[1,2,3,4]  # No two-family stop or redundant completed-shortlist fallback.
    assert retries and set(calls)=={1,2,3,4}
    assert selection.termination_reason!=CODE
    assert not node._observe_return_prepare_supplement(p,CODE)
    assert node._direct_near_field_candidate_deadline_sec(p,{})<=p.near_field_phase_deadline_sec


def test_one_bound_failed_family_at_child_checkpoint_can_request_measured_view(configured,monkeypatch):
    node,p,clock,candidates=ready(configured,monkeypatch)
    selection,calls,retries=search(node,p,clock,candidates,monkeypatch,seconds=[31.])
    assert not node._contact_yield_search['fallback']
    assert calls==[1] and len(retries)==1
    assert selection.termination_reason==CODE and selection.terminated_early
    proof=node._observe_return_completed_contact_failure(p,CODE)
    assert contact_search_yield_proof_valid(proof)
    assert proof['yield_evidence']['completed_family_count']==1
    assert proof['yield_evidence']['minimum_completed_families']==1
    assert proof['yield_evidence']['unchecked_count']==3
    assert node._observe_return_prepare_supplement(p,CODE)
    assert clock[0]*1e-9 < p.near_field_phase_deadline_sec



def test_actual_phase_expiry_never_yields_or_restarts(configured,monkeypatch):
    node,p,clock,candidates=ready(configured,monkeypatch)
    selection,calls,retries=search(node,p,clock,candidates,monkeypatch,seconds=[91.])
    assert calls==[1] and selection.termination_reason=='NEAR_FIELD_DIRECT_TIMEOUT'
    assert not node._contact_yield_search['fallback']
    assert not node._observe_return_prepare_supplement(p,CODE)


def test_winner_retains_original_authority_path_and_never_yields(configured,monkeypatch):
    node,p,clock,candidates=ready(configured,monkeypatch)
    selection,calls,retries=search(node,p,clock,candidates,monkeypatch,results=['collision','success'])
    assert selection.selected is not None
    assert node._contact_yield_search['incumbent_found']
    assert not node._contact_yield_search['fallback']
    assert not node._observe_return_prepare_supplement(p,CODE)


def test_loss_of_same_source_gain_restores_height_opportunities(configured,monkeypatch):
    node,p,clock,candidates=ready(configured,monkeypatch)
    selection,calls,retries=search(node,p,clock,candidates,monkeypatch,gap_after=1)
    assert not node._contact_yield_search['fallback'] and len(retries)==4
    assert selection.termination_reason!=CODE


@pytest.mark.parametrize('fault',['initial','one_family','no_gain','expired_session','views_full'])
def test_noneligible_scope_keeps_old_height_search(configured,monkeypatch,fault):
    node,p,clock,candidates=ready(configured,monkeypatch)
    if fault=='initial':node._observe_return_phase='initial_contact'
    elif fault=='one_family':candidates=candidates[:1]
    elif fault=='no_gain':node._observe_return_step_gain=dict(ok=False)
    elif fault=='expired_session':node._observe_return_session['deadline_ns']=clock[0]
    elif fault=='views_full':node._observe_return_session['view_index']=node._observe_return_session['max_observations']
    selection,calls,retries=search(node,p,clock,candidates,monkeypatch)
    assert getattr(node,'_contact_yield_search',None) is None
    assert retries and selection.termination_reason!=CODE


def test_completed_audit_cannot_be_relabelled_incomplete(configured,monkeypatch):
    node,p,clock,candidates=ready(configured,monkeypatch)
    search(node,p,clock,candidates,monkeypatch)
    node._active_gate_audit_report['moveit_selection']['terminated_early']=False
    assert node._observe_return_completed_contact_failure(p,CODE) is None


def test_checker_short_circuit_clears_previous_validated_receipts(configured,monkeypatch):
    node,p,clock,candidates=ready(configured,monkeypatch)
    for c in candidates:
        runtime=node._stable_variant_runtime[(c.track_id,c.variant_index)]
        plan_id='%024x'%c.track_id
        runtime['validated_contact_precheck_failure']=receipt(node,p,clock,plan_id)
        runtime['validated_contact_precheck_plan_id']=plan_id
    selection,calls,retries=search(node,p,clock,candidates,monkeypatch,
                                  results=['timeout']*8)
    assert not node._contact_yield_search['fallback'] and len(retries)==4
    assert not node._contact_yield_search['original_families']
    assert not node._contact_yield_search['collision_receipts']
    assert selection.termination_reason!=CODE


def test_fallback_same_request_reentry_retains_original_budget_and_height_search(configured,monkeypatch):
    node,p,clock,candidates=ready(configured,monkeypatch)
    search(node,p,clock,candidates,monkeypatch,results=['timeout']*8,seconds=[31.])
    state=node._contact_yield_search
    started,deadline=state['started_ns'],state['budget_deadline_ns']
    selection,calls,retries=search(node,p,clock,candidates,monkeypatch)
    assert calls==[1,2,3,4] and len(retries)==4
    assert node._contact_yield_search is state and state['fallback']
    assert (state['started_ns'],state['budget_deadline_ns'])==(started,deadline)
    assert selection.termination_reason!=CODE and not selection.terminated_early
