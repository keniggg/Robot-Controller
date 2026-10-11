"""Real supplemental task/session lifecycle with inert physical interfaces."""
from copy import deepcopy
from types import SimpleNamespace as NS
from unittest import mock

import numpy as np
import pytest

from alicia_grasp_modes.active_observation_policy import registered_information_gain, evidence_digest
from alicia_grasp_modes import observe_return_task as task
from test_observe_return_task import (context, begin, evidence, audit_reference, at_ns, module, base)
from test_mode_runner import working_volume_view


def supplement(c, index):
    session=c.node._observe_return_session
    _, report, plan=evidence('observation_prefix')
    stamp=c.now[0]-1_000_000
    plan.plan_id=report['plan_id']=str(index)*24
    plan.header.stamp=at_ns(stamp)
    plan.poses[0].position.x += .02*index
    report['selected']['execution_sequence']['stages'][0]['position_m'][0]=plan.poses[0].position.x
    report['selected']['observation_view']=working_volume_view(stamp)
    report['snapshot_stamp_ns']=stamp
    report['selected']['final_strict_plan_id']=plan.plan_id
    identity=NS(epoch=session['target_epoch'],track_id=session['target_track_id'])
    source=int(session['view_source_stamp_ns'])+100_000
    old=np.array([[0.,y,.02] for y in (.001,.011,.021)])
    new=np.array([[.01,y,.02] for y in (.001,.011,.021)])
    before=NS(identity=identity,points_base=old,view_stamps_ns=(int(session['view_source_stamp_ns']),))
    after=NS(identity=identity,points_base=np.vstack((old,new)),view_stamps_ns=before.view_stamps_ns+(source,),
        view_indices=np.array([0,0,0,1,1,1]))
    gain=registered_information_gain(before,after,NS(ok=True),depth_uncertainty_m=.001)
    failure=dict(request_id=index+100,generation=index+10,target_epoch=identity.epoch,
        snapshot_stamp_ns=source,contact_search_code='NEAR_FIELD_NO_REACHABLE_CANDIDATE',
        selected=False,terminated_early=False,checked_count=2,shortlist_count=2,reachable_count=0,
        locally_valid=dict(entered=3,passed=2,rejected=1),contact_or_motion_authorized=False)
    opportunity={k:failure[k] for k in ('request_id','generation','target_epoch','snapshot_stamp_ns','contact_search_code')}
    opportunity.update(policy='current_request_independent_bilateral_view_gap',contact_or_motion_authorized=False)
    request=dict(session_id=session['session_id'],view_index=index,previous_view_index=session['view_index'],
        origin_plan_id=session['origin_plan_id'],origin_snapshot_stamp_ns=int(session['origin_snapshot_stamp_ns']),
        target_epoch=identity.epoch,target_track_id=identity.track_id,deadline_ns=int(session['deadline_ns']),
        max_observations=session['max_observations'],failed_search=failure,visual_opportunity=opportunity,
        previous_gain=gain,previous_gain_sha256=evidence_digest(gain),contact_or_motion_authorized=False)
    request['view_request_id']=evidence_digest(request)
    report['observe_return'].update(request,phase='supplemental_observation',version=2,
        view_source_stamp_ns=stamp,view_plan_id=plan.plan_id)
    report.update(planning_origin='observe_return_supplemental_observation',observation_motion_performed=True)
    return plan,report


def prepare(c,monkeypatch):
    begin(c)
    calls=[]
    bound=[c.plan]
    monkeypatch.setattr(task, 'committed_plan_digest',
        lambda plan: __import__('hashlib').sha256(plan.plan_id.encode()).hexdigest())
    c.node._validate_bound_plan_locked=lambda plan,cfg:NS(ok=plan.plan_id==bound[0].plan_id,code='BOUND',reason='')
    def freeze(plan):
        calls.append(('freeze',plan.plan_id));bound[0]=deepcopy(plan);return bound[0]
    c.node._freeze_execution_plan=freeze
    c.node._execution_checkpoint=lambda plan,cfg,label:plan.plan_id==bound[0].plan_id
    def execute(*args,**kwargs):
        calls.append(('move',kwargs['execution_plan_id']))
        assert kwargs['allow_post_failure_observation_validation'] is False
        c.node._last_motion_settle_sample=dict(stationary_window_start_ns=c.now[0]+10_000_000,
            stationary_window_end_ns=c.now[0]+20_000_000)
        c.now[0]+=30_000_000
        return True
    c.node._plan_and_execute_pose=execute
    c.node._validate_measured_observation_envelope=lambda *a:True
    def measure(plan,cfg,cutoff):
        c.node._last_observation_camera_range_evidence=dict(image_source_stamp_ns=cutoff+1,
            observation_stamp_ns=cutoff+1)
        c.now[0]=cutoff+2
        return NS(ok=True)
    c.node._wait_for_fresh_observation_camera_target_range=measure
    def proxy(name,_service):
        def invoke(plan,execute):
            c.gateways.append((name,plan.plan_id,execute))
            import json
            if name=='/supervisor/admit_observe_return_view':
                next_session=c.publications[-1]
                proof=dict(code='OBSERVE_RETURN_VIEW_ADMITTED',physical_motion_submitted=False,
                    plan_id=plan.plan_id,target_track_id=plan.target_track_id,
                    session_id=next_session['session_id'],origin_plan_id=c.plan.plan_id,
                    origin_plan_digest=c.proof['plan_digest'],origin_goal_sha256=c.proof['origin_goal_sha256'],
                    epoch_ns=c.proof['epoch_ns'],previous_view_plan_id=c.node._observe_return_session['view_plan_id'],
                    plan_digest=task.committed_plan_digest(plan),
                    **{key:next_session[key] for key in ('view_index','view_source_stamp_ns',
                        'view_request_id','previous_gain_sha256','deadline_ns')})
                return NS(success=True,message=json.dumps(proof))
            assert name=='/supervisor/return_observe_return_origin'
            current=c.now[0];c.now[0]+=2_000_000_000
            proof=dict(c.proof,code='OBSERVE_RETURN_MEASURED_ARRIVAL',post_command_feedback_valid=True,
                controller_action_completed=True,command_submission_attempted=True,
                completion_stamp_ns=current+500_000_000,accepted_stamp_ns=current+1_500_000_000)
            import json
            return NS(success=True,message=json.dumps(proof))
        return invoke
    monkeypatch.setattr(module.rospy,'ServiceProxy',proxy)
    return calls,bound


def install(c,plan,report):
    c.node.latest_grasp6d_preview_plan=plan
    c.node._observe_return_audit_cb(audit_reference(c.tmp_path,report))


def invoke(c,current):
    return c.node._before_near_field_terminal_recovery(NS(code='NEAR_FIELD_PLAN_WAITING'),
        current,{},mock.Mock(),mock.Mock())


def test_two_supplements_preserve_original_return_authority_and_deadline(context,monkeypatch):
    c=context;calls,bound=prepare(c,monkeypatch)
    original_deadline=c.node._observe_return_session['deadline_ns']
    current=c.plan
    for index in (2,3):
        c.now[0]+=2_000_000_000
        plan,report=supplement(c,index);install(c,plan,report)
        result=invoke(c,current)
        assert result.get('retry') is True,c.states
        current=result['current_plan']
        assert current.plan_id==plan.plan_id and c.node._observe_return_session['view_index']==index
        assert c.node._observe_return_session['deadline_ns']==original_deadline
        assert c.node._observe_return_origin_plan.plan_id==c.plan.plan_id
        assert result['minimum_stamp_ns']>int(c.node._observe_return_session['view_source_stamp_ns'])
        assert c.node._coordinated_observation_views()
    assert [kind for kind,_ in calls]==['freeze','move','freeze','move']
    c.now[0]+=1_000_000_000
    _,_,contact=evidence('initial_contact')
    contact.header.stamp=at_ns(c.now[0]-1)
    handoff=c.node._before_near_field_contact_freeze(contact,current,{'unknown_observe_return_return_to_origin': True})
    assert handoff.get('retry') is True,c.states
    assert c.gateways[-1]==('/supervisor/return_observe_return_origin',c.plan.plan_id,True)
    assert c.node._observe_return_session['deadline_ns']==original_deadline
    assert [p['phase'] for p in c.publications]==['observing','observing','observing','returning','returned']
    assert [p['view_index'] for p in c.publications]==[1,2,3,3,3]
    # A genuinely new contact after the measured return may freeze, even
    # though the active observation plan differs from the origin plan.
    _,report,fresh=evidence('returned_contact')
    fresh.plan_id=report['plan_id']='f'*24
    report['selected']['final_strict_plan_id']=fresh.plan_id
    report['snapshot_stamp_ns']=c.now[0]+1
    fresh.header.stamp=at_ns(report['snapshot_stamp_ns'])
    for key in ('session_id','origin_plan_id','origin_snapshot_stamp_ns',
                'return_verified_stamp_ns','contact_deadline_ns','deadline_ns'):
        value=c.node._observe_return_session[key]
        report['observe_return'][key]=int(value) if key.endswith('_ns') else value
    report['observe_return']['fused_source_stamps_ns']=[19500000000,24000000000,report['snapshot_stamp_ns']]
    c.now[0]+=2;install(c,fresh,report)
    assert c.node._before_near_field_contact_freeze(fresh,current,{'unknown_observe_return_return_to_origin': True}) is None,c.states


@pytest.mark.parametrize('fault', ['manual','lost','bound','fourth_view','deadline','no_gain',
    'gain_digest','wrong_origin','wrong_target','wrong_view','request_digest','physical_gate','future_source'])
def test_invalid_context_or_evidence_never_submits_supplement_motion(context,monkeypatch,fault):
    c=context;calls,bound=prepare(c,monkeypatch);c.now[0]+=2_000_000_000
    plan,report=supplement(c,2);meta=report['observe_return']
    if fault=='manual':c.params['/gui/joint_direct_mode']=True
    elif fault=='lost':c.node._near_field_wait_failure=lambda *a:NS(code='TARGET_LOST',reason='latched')
    elif fault=='bound':c.node._validate_bound_plan_locked=lambda *a:NS(ok=False,code='REVOKED',reason='')
    elif fault=='fourth_view':c.node._observe_return_session['view_index']=3
    elif fault=='deadline':c.now[0]=int(c.node._observe_return_session['deadline_ns'])
    elif fault=='no_gain':meta['previous_gain']['ok']=False
    elif fault=='gain_digest':meta['previous_gain_sha256']='0'*64
    elif fault=='wrong_origin':meta['origin_plan_id']='different'
    elif fault=='wrong_target':plan.target_track_id='different'
    elif fault=='wrong_view':meta['view_index']=3
    elif fault=='request_digest':meta['view_request_id']='0'*64
    elif fault=='physical_gate':report['selected']['moveit']['collision_free']=False
    elif fault=='future_source':
        plan.header.stamp=at_ns(c.now[0]+1);report['snapshot_stamp_ns']=c.now[0]+1;meta['view_source_stamp_ns']=c.now[0]+1
        report['selected']['observation_view']=working_volume_view(c.now[0]+1)
    install(c,plan,report)
    assert invoke(c,c.plan)=={'failed':True},c.states
    assert not [x for x in calls if x[0]=='move']


@pytest.mark.parametrize('fault',['execution','settle','envelope','fresh_source','range','post_motion_loss'])
def test_post_submission_failure_never_retries_or_enters_contact(context,monkeypatch,fault):
    c=context;calls,bound=prepare(c,monkeypatch);c.now[0]+=2_000_000_000
    plan,report=supplement(c,2);install(c,plan,report)
    original=c.node._plan_and_execute_pose
    def execute(*a,**kw):
        result=original(*a,**kw)
        if fault=='execution':return False
        if fault=='settle':c.node._last_motion_settle_sample=None
        if fault=='post_motion_loss':c.node._near_field_wait_failure=lambda *a:NS(code='TARGET_LOST',reason='latched')
        return result
    c.node._plan_and_execute_pose=execute
    if fault=='envelope':c.node._validate_measured_observation_envelope=lambda *a:False
    if fault=='range':c.node._wait_for_fresh_observation_camera_target_range=lambda *a:NS(ok=False,code='RGBD_INVALID',reason='')
    if fault=='fresh_source':
        def old(*a):c.node._last_observation_camera_range_evidence=dict(image_source_stamp_ns=1,observation_stamp_ns=1);return NS(ok=True)
        c.node._wait_for_fresh_observation_camera_target_range=old
    assert invoke(c,c.plan)=={'failed':True}
    assert len([x for x in calls if x[0]=='move'])==1
    assert not c.replans
    assert [p['phase'] for p in c.publications]==['observing','observing']


def test_v1_session_keeps_single_view_and_never_consumes_supplement(context):
    c=context
    c.node._begin_observe_return(c.plan,{'unknown_observe_return_session_version':1})
    assert c.node._coordinated_observation_views()  # No uncounted legacy clear-view.
    assert invoke(c,c.plan) is None


@pytest.mark.parametrize('field',['code','physical_motion_submitted','plan_id','target_track_id',
    'session_id','origin_plan_id','origin_plan_digest','origin_goal_sha256','epoch_ns',
    'previous_view_plan_id','view_index','view_source_stamp_ns','view_request_id',
    'previous_gain_sha256','deadline_ns','plan_digest'])
def test_gateway_admission_proof_is_fully_bound_before_freeze_or_motion(context,monkeypatch,field):
    import json
    c=context;calls,bound=prepare(c,monkeypatch);c.now[0]+=2_000_000_000
    plan,report=supplement(c,2);install(c,plan,report)
    original=module.rospy.ServiceProxy
    def proxy(name,kind):
        call=original(name,kind)
        def invoke(*a,**kw):
            response=call(*a,**kw);proof=json.loads(response.message)
            if field=='physical_motion_submitted':proof[field]=True
            elif isinstance(proof[field],int):proof[field]+=1
            else:proof[field]='different'
            response.message=json.dumps(proof);return response
        return invoke
    monkeypatch.setattr(module.rospy,'ServiceProxy',proxy)
    assert invoke(c,c.plan)=={'failed':True}
    assert not calls  # No freeze and no physical interface call.


def test_revocation_during_remote_or_gateway_wait_is_not_erased_by_new_freeze(context,monkeypatch):
    c=context;calls,bound=prepare(c,monkeypatch);c.now[0]+=2_000_000_000
    plan,report=supplement(c,2);install(c,plan,report)
    c.node._execution_authority_revoked=False
    original=module.rospy.ServiceProxy
    def proxy(name,kind):
        call=original(name,kind)
        def invoke(*a,**kw):
            response=call(*a,**kw)
            # Models the real callback acquiring the SAME plan RLock during
            # an asynchronous admission wait, before the commit boundary.
            with c.node._grasp6d_plan_guard():
                c.node._execution_authority_revoked=True
            return response
        return invoke
    monkeypatch.setattr(module.rospy,'ServiceProxy',proxy)
    c.node._validate_bound_plan_locked=lambda *a:NS(ok=not c.node._execution_authority_revoked,
        code='EXECUTION_AUTHORITY_REVOKED',reason='callback')
    assert invoke(c,c.plan)=={'failed':True}
    assert not calls and c.node._execution_authority_revoked
    assert c.node._observe_return_session['view_index']==1


@pytest.mark.parametrize('field',['max_observations','view_index','view_plan_id',
    'view_source_stamp_ns','view_request_id','previous_gain_sha256'])
def test_initial_v2_ack_cannot_change_any_view_authority_field(context,field):
    c=context
    def mutate(ack):
        ack[field]=ack[field]+1 if type(ack[field]) is int else 'different'
    c.transport['mutate_ack']=mutate
    with pytest.raises(ValueError,match='ACK_BINDING_INVALID'):begin(c)


@pytest.mark.parametrize('fault',['already_stale','expires_during_admission','controller_failure'])
def test_fresh_admission_and_no_failed_execution_retry_are_preserved(context,monkeypatch,fault):
    c=context;calls,bound=prepare(c,monkeypatch);c.now[0]+=2_000_000_000
    plan,report=supplement(c,2);install(c,plan,report)
    if fault=='already_stale':c.now[0]+=3_000_000_000
    elif fault=='controller_failure':c.node._last_observation_execution_failure=dict(plan_id=c.plan.plan_id)
    else:
        original=module.rospy.ServiceProxy
        def proxy(name,kind):
            call=original(name,kind)
            def invoke(*a,**kw):
                response=call(*a,**kw);c.now[0]+=3_000_000_000;return response
            return invoke
        monkeypatch.setattr(module.rospy,'ServiceProxy',proxy)
    assert invoke(c,c.plan)=={'failed':True}
    assert not calls


@pytest.mark.parametrize('boundary',['return_ack','service_wait'])
@pytest.mark.parametrize('fault',['revoked','target_lost','expired'])
def test_return_submission_revalidates_active_view_after_asynchronous_wait(context,monkeypatch,boundary,fault):
    c=context;calls,bound=prepare(c,monkeypatch)
    c.now[0]+=2_000_000_000
    _,_,contact=evidence('initial_contact');contact.plan_id='contact'
    contact.header.stamp=at_ns(c.now[0]-1)
    invalid=[False]
    def revoke(*unused, **unused_kw):
        invalid[0]=True
        if fault=='expired':c.now[0]=int(c.node._observe_return_session['deadline_ns'])
    if boundary=='return_ack':
        def ack(message):
            if message['phase']=='returning':revoke()
        c.transport['mutate_ack']=ack
    else:
        monkeypatch.setattr(module.rospy,'wait_for_service',revoke)
    validations=[]
    def validate(plan,cfg):
        validations.append(plan.plan_id)
        return NS(ok=not invalid[0],code='OBSERVE_RETURN_'+fault.upper(),reason='callback during wait')
    c.node._validate_bound_plan_locked=validate
    assert c.node._before_near_field_contact_freeze(contact,c.plan,{'unknown_observe_return_return_to_origin': True})=={'failed':True}
    assert not any(name=='/supervisor/return_observe_return_origin' for name,_,_ in c.gateways)
    assert not calls
    if not (boundary=='return_ack' and fault=='expired'):
        assert validations==[c.plan.plan_id]
