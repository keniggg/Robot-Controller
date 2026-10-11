"""Real producer messages and task callbacks; inert physical service boundary."""
from collections import deque
from copy import deepcopy
import itertools
import json
import threading
import time
from types import SimpleNamespace as NS

import pytest
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from test_initial_static_identity_completion import gap, frame_after_gap
from test_task_modes import module, base
from test_contact_linear_compensation import recorded
from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id
from alicia_grasp_modes.initial_preflight_task import _StillPending
from alicia_grasp_modes.observe_return_task import ObserveReturnTask


def feed(c,row,order=('object','mask','status')):
    for key in order:
        if key=='object':c.node.obj_cb(deepcopy(row[key]))
        elif key=='mask':c.node._initial_preflight_mask(deepcopy(row[key]))
        else:c.node._unknown_support_status_cb(String(data=json.dumps(row[key])))


@pytest.fixture
def prepared(gap,monkeypatch):
    s=gap;n=module.ModeAwareGraspTaskNode.__new__(module.ModeAwareGraspTaskNode)
    n._initialize_mode_boundary();n.active=False;n.set_state=lambda *a,**k:None
    assert n.reset_mode_cb(None).success
    n._mode_bound_selection=deepcopy(n._mode_selection)
    n._following_evidence_lock=threading.RLock()
    n._following_epoch_ns=s.ready['status']['initial_station_measurement']['epoch_ns']
    joint=JointState();joint.header.stamp=module.rospy.Time.from_sec(s.clock[0]-.01)
    joint.header.frame_id='sdk_measured';joint.name=['Joint%d'%i for i in range(1,7)]+['right_finger']
    joint.position=s.ready['status']['initial_station_measurement']['q_station_rad']+[.05]
    n._following_accepted_history=deque([joint],maxlen=128);n._following_sdk_history=deque(maxlen=128)
    n._initial_preflight_request_pub=NS(publish=lambda msg:None)
    c=NS(node=n,source=s,clock=s.clock,params=s.params)
    # The producer's actual acquired template can precede its latest READY.
    publications={}
    for key in ('object','object_mask','detector_status'):
        for call in s.p.pubs[key].publish.call_args_list:
            msg=deepcopy(call[0][0])
            if key=='detector_status':
                value=json.loads(msg.data);ns=value.get('source_stamp_ns')
                if ns is not None:publications.setdefault(ns,{})['status']=value
            else:
                ns=msg.header.stamp.to_nsec();publications.setdefault(ns,{})['mask' if key=='object_mask' else 'object']=msg
    saved=s.clock[0]
    for ns,row in sorted(publications.items()):
        if len(row)==3 and ns<=s.ready['object'].header.stamp.to_nsec():
            s.clock[0]=ns/1e9+.1;feed(c,row)
    s.clock[0]=saved
    assert n._initial_preflight_collector.template is not None,n._initial_preflight_collector.fatal
    assert not n._initial_preflight_collector.fatal
    _,_,plan,_,_=recorded(2)
    plan.diagnostic='FAR_FIELD_OBSERVATION_PLAN';plan.model_choice='unknown_tabletop'
    plan.header.stamp=deepcopy(s.ready['object'].header.stamp)
    plan.object_geometry.pose_base=deepcopy(s.ready['object'].pose_base.pose)
    plan.plan_id=compute_plan_id(plan)
    n._initial_preflight_begin(NS(plan_id=plan.plan_id));n._freeze_execution_plan(plan)
    n.active=True;n.stage=3
    reference=n._initial_preflight_collector.references[s.ready['object'].header.stamp.to_nsec()]
    req=dict(version=1,token_id=n._initial_preflight_attempt['attempt_id'],selection=deepcopy(n._mode_bound_selection),
        plan_id=plan.plan_id,target_track_id=plan.target_track_id,target_epoch=0,
        control_epoch_ns=n._following_epoch_ns,q_station_rad=list(joint.position[:6]),
        anchor_binding=reference['binding'],deadline_ns=round((s.clock[0]+30)*1e9),physical_action_submitted=False)
    n._initial_preflight_token=dict(request=req,state='armed',monotonic_deadline=time.monotonic()+30)
    c.plan=plan
    return c


def notice(c,row=None):
    row=row or c.source.gap;ns=row['object'].header.stamp.to_nsec()
    m=deepcopy(c.plan.object_geometry);m.valid=False;m.header.stamp=deepcopy(row['object'].header.stamp)
    m.header.frame_id='base_link';m.target_track_id=c.plan.target_track_id
    m.failure_reason='TARGET_MEASUREMENT_PENDING: '+json.dumps(dict(initial_preflight=dict(
        token_id=c.node._initial_preflight_token['request']['token_id'],
        reference_source_ns=row['status']['source_qualification']['reference_source_stamp_ns'],pending_source_ns=ns)))
    return m


@pytest.mark.parametrize('order',list(itertools.permutations(('object','mask','status'))))
@pytest.mark.parametrize('notice_first',[False,True])
def test_three_topic_pending_then_new_measurement_preserves_no_action_history(prepared,monkeypatch,order,notice_first):
    c=prepared;n=c.node;before=n._execution_plan_digest(n._bound_execution_plan)
    if notice_first:n.target_geometry_cb(notice(c))
    feed(c,c.source.gap,order)
    if not notice_first:n.target_geometry_cb(notice(c))
    assert n._initial_preflight_pending and not n._execution_authority_revoked
    assert n.latest_obj is None and n.latest_target_geometry is None
    recovery=frame_after_gap(c.source,monkeypatch)
    feed(c,recovery,order)
    assert not n._initial_preflight_pending
    assert not n._execution_authority_revoked
    assert n.latest_obj.header.stamp==recovery['object'].header.stamp
    assert n._initial_preflight_replan
    assert n._execution_plan_digest(n._bound_execution_plan)==before
    assert not n._initial_preflight_attempt['physical_action_submitted']


def test_typed_notice_before_arm_ack_is_quarantine_not_authority(prepared):
    c=prepared;n=c.node;n._initial_preflight_token['state']='registering'
    n.target_geometry_cb(notice(c));feed(c,c.source.gap)
    assert not n._execution_authority_revoked and n._initial_preflight_pending
    called=[]
    result,_=n._invoke_plan_bound_action(c.plan,{},'opening',lambda:called.append(True))
    assert not result.ok and result.code=='TARGET_MEASUREMENT_PENDING' and not called


@pytest.mark.parametrize('fault',['plain','track','frame','token','future','true_loss','hard_revoked','epoch','manual','pose_out_back'])
def test_unbound_or_hard_failures_never_become_initial_wait(prepared,fault):
    c=prepared;n=c.node;m=notice(c)
    if fault=='plain':m.failure_reason='TARGET_MEASUREMENT_PENDING: old unbound text'
    elif fault=='track':m.target_track_id='different'
    elif fault=='frame':m.header.frame_id='camera'
    elif fault=='token':m.failure_reason=m.failure_reason.replace(n._initial_preflight_token['request']['token_id'],'foreign')
    elif fault=='future':m.header.stamp=module.rospy.Time.from_sec(c.clock[0]+1)
    elif fault=='true_loss':m.failure_reason='TARGET_LOST: actual target changed'
    elif fault=='hard_revoked':n._execution_authority_revoked=True
    elif fault=='epoch':n._following_epoch_ns+=1
    elif fault=='manual':c.params['/gui/joint_direct_mode']=True
    else:
        q=deepcopy(n._following_accepted_history[-1]);q.position[0]+=.01
        n._initial_preflight_feedback(q);q.position[0]-=.01;n._initial_preflight_feedback(q)
    n.target_geometry_cb(m)
    assert n._execution_authority_revoked


def test_preflight_gap_discards_success_and_replans_after_actual_recovery(prepared,monkeypatch):
    c=prepared;n=c.node;calls=[]
    def preflight(pose,execute):
        assert execute is False;calls.append(pose)
        if len(calls)==1:
            feed(c,c.source.gap);n.target_geometry_cb(notice(c))
            feed(c,frame_after_gap(c.source,monkeypatch))
        return NS(success=True,message='planned')
    result=n._initial_preflight_wrap_move(preflight)(object(),False)
    assert result.success and len(calls)==2
    assert not n._initial_preflight_replan and not n._execution_authority_revoked


def test_already_open_has_no_command_or_commit(prepared,monkeypatch):
    c=prepared;n=c.node;calls=[]
    monkeypatch.setattr(n,'_gripper_position_reached',lambda *a:True)
    monkeypatch.setattr(n,'_execution_checkpoint',lambda *a:True)
    monkeypatch.setattr(n,'_initial_preflight_rpc',lambda *a:calls.append('rpc'))
    assert n._command_gripper_position(lambda *a:calls.append('command'),.05,'open',.5,
        execution_plan=c.plan,gcfg={},skip_if_reached=True,reached_tolerance=.001)
    assert calls==[] and n._initial_preflight_token['state']=='armed'
    assert not n._initial_preflight_attempt['physical_action_submitted']


def test_late_registered_pending_cannot_revoke_recovered_or_committed(prepared,monkeypatch):
    c=prepared;n=c.node;m=notice(c)
    feed(c,c.source.gap);n.target_geometry_cb(m)
    feed(c,frame_after_gap(c.source,monkeypatch))
    obj=n.latest_obj
    n._initial_preflight_token['state']='committed'
    n.target_geometry_cb(m)
    assert not n._execution_authority_revoked and n.latest_obj is obj


@pytest.mark.parametrize('key',['object','mask','status'])
def test_duplicate_source_keeps_first_receive_without_renewal(prepared,key):
    c=prepared;collector=c.node._initial_preflight_collector
    row=c.source.ready;ns=row['object'].header.stamp.to_nsec()
    caches=dict(object=collector.objects,mask=collector.masks,status=collector.statuses)
    before=deepcopy(caches[key][ns])
    c.clock[0]+=.05;feed(c,row,(key,))
    assert not collector.fatal
    assert caches[key][ns]==before


@pytest.mark.parametrize('kind',['bootstrap','service_exception'])
def test_actual_invoke_closes_future_preparation_even_if_service_raises(prepared,monkeypatch,kind):
    c=prepared;n=c.node;n._initial_preflight_token=None
    monkeypatch.setattr(base.GraspTaskNode,'_validate_bound_plan_locked',lambda *a:base.PlanValidationResult(True))
    called=[]
    def physical():
        called.append(True)
        assert n._initial_preflight_attempt['physical_action_submitted']
        if kind=='service_exception':raise RuntimeError('service failed after submission')
        return NS(success=True)
    if kind=='service_exception':
        with pytest.raises(RuntimeError):n._invoke_plan_bound_action(c.plan,{},'actuation observation prefix',physical)
    else:n._invoke_plan_bound_action(c.plan,{},'actuation observation prefix',physical)
    assert called==[True] and n._initial_preflight_attempt['physical_action_submitted']
    assert n._initial_preflight_arm(c.plan,{}) is False
    # Ordinary cleanup cannot erase the independent commit record.
    n._clear_bound_execution_plan()
    assert n._initial_preflight_attempt['physical_action_submitted']


def test_original_rejection_does_not_claim_action_submission(prepared,monkeypatch):
    c=prepared;n=c.node;n._initial_preflight_token=None
    monkeypatch.setattr(base.GraspTaskNode,'_validate_bound_plan_locked',
        lambda *a:base.PlanValidationResult(False,'TEST_REJECTION','inert test gate'))
    called=[]
    result,_=n._invoke_plan_bound_action(c.plan,{},'motion',lambda:called.append(True))
    assert not result.ok and not called
    assert not n._initial_preflight_attempt['physical_action_submitted']


def test_actual_opening_revokes_preparation_and_keeps_original_command_path(prepared,monkeypatch):
    c=prepared;n=c.node;calls=[]
    monkeypatch.setattr(n,'_gripper_position_reached',lambda *a:False)
    monkeypatch.setattr(n,'_initial_preflight_rpc',lambda action:(calls.append(action) or dict(state='revoked')))
    def original(self,set_gripper,*a,**kw):
        assert self._initial_preflight_token['state']=='retired_for_gripper'
        set_gripper(.05)
        return True
    monkeypatch.setattr(base.GraspTaskNode,'_command_gripper_position',original)
    assert n._command_gripper_position(lambda *a:calls.append('command'),.05,'open',.5,
        execution_plan=c.plan,gcfg={},skip_if_reached=True,reached_tolerance=.001)
    assert calls==['revoke','command']
    n._initial_preflight_wrap_strict(lambda *a:calls.append('original_motion'))(None,True)
    assert calls==['revoke','command','original_motion']


def test_recovery_after_preflight_returns_forces_fresh_preflight_at_checkpoint(prepared,monkeypatch):
    c=prepared;n=c.node;calls=[]
    move=lambda p,e:(calls.append(e) or NS(success=True))
    assert n._initial_preflight_wrap_move(move)(object(),False).success
    feed(c,c.source.gap);n.target_geometry_cb(notice(c))
    feed(c,frame_after_gap(c.source,monkeypatch))
    monkeypatch.setattr(base.GraspTaskNode,'_execution_checkpoint',lambda *a:True)
    assert n._execution_checkpoint(c.plan,{},'open after preflight')
    assert calls==[False,False] and not n._initial_preflight_replan


def test_expired_or_partial_recovery_does_not_authorize(prepared,monkeypatch):
    c=prepared;n=c.node;feed(c,c.source.gap);n.target_geometry_cb(notice(c))
    recovery=frame_after_gap(c.source,monkeypatch)
    feed(c,recovery,('object','status'))
    assert n._initial_preflight_pending and n.latest_obj is None
    c.clock[0]=n._initial_preflight_pending['deadline_ns']/1e9+.001
    feed(c,recovery,('mask',))
    assert n._execution_authority_revoked
    assert not n._initial_preflight_wait()


@pytest.mark.parametrize('phase',['before_begin_rejected','after_begin_pending','commit_accepted'])
def test_motion_lease_handoff_failure_never_retries_or_submits(prepared,monkeypatch,phase):
    c=prepared;n=c.node;events=[]
    def begin(self,*a):
        events.append('lease_begin')
        if phase=='before_begin_rejected':raise ValueError('ordinary measurement rejection')
        return dict(state='executing')
    def commit():
        events.append('commit')
        if phase=='after_begin_pending':raise _StillPending()
        n._initial_preflight_token['state']='committed'
    monkeypatch.setattr(ObserveReturnTask,'_observation_measurement_begin',begin)
    monkeypatch.setattr(n,'_initial_preflight_commit',commit)
    if phase=='commit_accepted':
        n._observation_measurement_begin(c.plan)
        assert not n._initial_preflight_attempt['physical_action_submitted']
        n._initial_preflight_wrap_strict(lambda *a:events.append('physical'))(None,True)
        assert n._initial_preflight_attempt['physical_action_submitted']
        assert events==['lease_begin','commit','physical']
    else:
        with pytest.raises(ValueError):n._observation_measurement_begin(c.plan)
        assert not n._initial_preflight_attempt['physical_action_submitted']
        assert events==(['lease_begin'] if phase=='before_begin_rejected' else ['lease_begin','commit'])


@pytest.mark.parametrize('fault',['manual','pose_out_back','epoch','hard_revoke','selection'])
@pytest.mark.parametrize('moment',['during_ack','after_ack'])
def test_commit_and_submit_check_irreversible_history(prepared,monkeypatch,fault,moment):
    c=prepared;n=c.node;calls=[]
    def change():
        if fault=='manual':
            n._initial_preflight_manual(NS(data=True));n._initial_preflight_manual(NS(data=False))
        elif fault=='pose_out_back':
            q=deepcopy(n._following_accepted_history[-1]);q.position[0]+=.01
            n._initial_preflight_feedback(q);q.position[0]-=.01;n._initial_preflight_feedback(q)
        elif fault=='epoch':n._following_epoch_ns+=1
        elif fault=='hard_revoke':n._execution_authority_revoked=True
        else:n._mode_bound_selection=dict(n._mode_bound_selection,generation=999)
    def rpc(action):
        if moment=='during_ack':change()
        return dict(state='committed')
    monkeypatch.setattr(n,'_initial_preflight_rpc',rpc)
    if moment=='during_ack':
        with pytest.raises(ValueError,match='SUBMISSION_CONTEXT_CHANGED'):n._initial_preflight_commit()
    else:
        n._initial_preflight_commit();change()
        with pytest.raises(ValueError,match='SUBMISSION_CONTEXT_CHANGED'):
            n._initial_preflight_wrap_strict(lambda *a:calls.append(True))(None,True)
    assert not calls and not n._initial_preflight_attempt['physical_action_submitted']
    assert n._execution_authority_revoked


def test_strict_wrapper_closes_after_first_submit_and_delegates_later_views(prepared,monkeypatch):
    c=prepared;n=c.node;calls=[]
    monkeypatch.setattr(n,'_initial_preflight_rpc',lambda action:dict(state='committed'))
    n._initial_preflight_commit()
    strict=n._initial_preflight_wrap_strict(lambda pose,execute:calls.append(pose))
    strict('first_observation',True)
    assert n._initial_preflight_attempt['physical_action_submitted']
    # The original controller may now move, rebind a CONTACT plan and enter
    # later views. This no-action wrapper must not claim their authorization.
    q=deepcopy(n._following_accepted_history[-1]);q.position[0]+=.1
    n._initial_preflight_feedback(q)
    n._bound_execution_plan=deepcopy(c.plan);n._bound_execution_plan.plan_id='contact_new_plan'
    strict('supplementary_observation',True);strict('contact',True)
    assert calls==['first_observation','supplementary_observation','contact']
    assert len(n._initial_preflight_attempt['submissions'])==1


@pytest.mark.parametrize('changed',[False,True])
def test_only_exact_proven_old_false_is_ignored_after_commit(prepared,monkeypatch,changed):
    c=prepared;n=c.node;feed(c,c.source.gap);n.target_geometry_cb(notice(c))
    feed(c,frame_after_gap(c.source,monkeypatch));current=n.latest_obj
    n._initial_preflight_token['state']='committed'
    late=deepcopy(c.source.gap['object'])
    if changed:late.header.frame_id='foreign_frame'
    n.obj_cb(late)
    if changed:assert n._execution_authority_revoked
    else:assert n.latest_obj is current and not n._execution_authority_revoked


def test_missing_epoch_is_disqualified_without_crashing_attempt_cleanup(prepared):
    c=prepared;n=c.node
    del n._following_epoch_ns
    n._initial_preflight_epoch()
    assert 'control_epoch_changed_or_missing' in n._initial_preflight_attempt['disqualifications']
    assert n._initial_preflight_arm(c.plan,{}) is False
    n.active=False;n._initial_preflight_token=None
    receipt=n._initial_preflight_finish(NS(success=False,message='missing feedback'))
    assert receipt['released'] and receipt['final_result_success'] is False
    assert not receipt['physical_action_submitted']
