"""Provenance boundaries using real task source joins, never motion services."""
from copy import deepcopy
from types import SimpleNamespace as NS
import json

import pytest

from test_station_compute_wait_task import station, configured, frame, deliver, advance, feedback, base, at_ns
from alicia_grasp_modes.station_contact_admission import (
    KEY, build_receipt, validate_receipt, pending_payload_schema)
from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id


def computed(c):
    def measured(ns):
        c.mono[0]+=(ns+50_000_000-c.now[0])/1e9;c.now[0]=ns+50_000_000
        feedback(c,ns-20_000_000);feedback(c,ns+20_000_000)
    n=c.node; rows=[]; previous=None
    for ns in (100_000_000_000,100_400_000_000,100_800_000_000):
        measured(ns)
        row=frame(c, ns, previous);deliver(c,row)
        previous=row['status']['station_compute_wait'];rows.append(deepcopy(n._station_contact_sources[ns]))
    advance(c,101_650_000_000)
    assert n._station_task_record['pending']
    for i in range(25):
        ns=101_800_000_000+i*400_000_000
        measured(ns)
        row=frame(c,ns,previous);deliver(c,row)
        previous=row['status']['station_compute_wait']
    plan=deepcopy(c.plan);plan.diagnostic='CONTACT_EXECUTION_PLAN'
    plan.refinement_status='VALID_3D';plan.header.stamp=at_ns(100_800_000_000)
    plan.object_geometry.header=deepcopy(plan.header)
    plan.refinement_inlier_count=100;plan.fused_view_count=3
    plan.refinement_overlap_fraction=.9;plan.refinement_rmse_m=.001
    plan.refinement_translation_m=.001;plan.refinement_rotation_deg=.1
    plan.refinement_source_clipped=False;plan.plan_id=compute_plan_id(plan)
    receipt=build_receipt(plan,ticket=dict(request_id=43,generation=13,target_epoch=4),
        source_witnesses=rows,current_witness=n._station_contact_sources[ns],
        issued_ns=c.now[0],snapshot_policy=dict(purpose='direct_contact',phase='observing_contact',
            frame_count=3,maximum_span_sec=12.))
    n._observe_return_reports[plan.plan_id]={KEY:receipt}
    return plan,receipt,previous


def test_long_compute_keeps_original_sources_and_requires_independent_current_join(station):
    c=station;plan,receipt,previous=computed(c)
    assert c.node._station_task_cutoff > plan.header.stamp.to_nsec()
    assert len(c.node._station_contact_sources)>16
    assert c.node._station_contact_provenance(plan).ok
    assert plan.header.stamp.to_nsec()==100_800_000_000
    assert receipt['motion_authorized'] is False
    # A newer independently joined source is valid; equality to the publisher's
    # last current source is not required, and no receipt clock is renewed.
    ns=previous['source_ns']+400_000_000
    c.mono[0]+=(ns+50_000_000-c.now[0])/1e9;c.now[0]=ns+50_000_000
    feedback(c,ns-20_000_000);feedback(c,ns+20_000_000);deliver(c,frame(c,ns,previous))
    assert c.node._station_contact_provenance(plan).ok
    assert receipt['current']['contract']==previous


@pytest.mark.parametrize('fault',['object_digest','missing_old_source','missing_current_source',
    'hard_invalidation','epoch','manual','camera','phase_deadline','geometry_deadline',
    'plan_pose','source','ticket_epoch','snapshot_count'])
def test_long_compute_provenance_never_substitutes_a_failed_original_gate(station,fault):
    c=station;plan,receipt,previous=computed(c);n=c.node
    if fault=='object_digest':receipt['sources'][0]['object_sha256']='f'*64
    elif fault=='missing_old_source':n._station_contact_sources.pop(100_000_000_000)
    elif fault=='missing_current_source':n._station_contact_sources.pop(previous['source_ns'])
    elif fault=='hard_invalidation':n._station_contact_invalidate(previous['source_ns'])
    elif fault=='epoch':n._following_epoch_ns+=1
    elif fault=='manual':c.params['/gui/joint_direct_mode']=True
    elif fault=='camera':c.params['/camera']['fx']+=1
    elif fault=='phase_deadline':advance(c,190_000_000_000)
    elif fault=='geometry_deadline':advance(c,previous['geometry_deadline_ns'])
    elif fault=='plan_pose':plan.poses[2].position.z+=.001
    elif fault=='source':plan.header.stamp=at_ns(100_800_000_001)
    elif fault=='ticket_epoch':receipt['ticket']['target_epoch']+=1
    elif fault=='snapshot_count':receipt['snapshot_policy']['frame_count']=5
    result=n._station_contact_provenance(plan)
    assert not result.ok
    assert n._station_contact_admitted is None


@pytest.mark.parametrize('kind',['terminal','ordinary_pending','old_hint','wrong_binding','false_clock'])
def test_invalid_preview_cannot_resurrect_old_computational_candidate(station,kind):
    c=station;plan,receipt,contract=computed(c);n=c.node
    n.grasp6d_preview_plan_cb(plan)
    msg=base.Grasp6DPlan();msg.header=deepcopy(plan.header)
    msg.header.stamp=at_ns(c.now[0]);msg.valid=False;msg.candidate_source='near_field_pending'
    msg.diagnostic='TARGET_PROCESSING_PENDING: current geometry expired; identity only'
    if kind=='terminal':
        msg.candidate_source='near_field_terminal';msg.diagnostic='NEAR_FIELD_NO_REACHABLE_CANDIDATE: exhausted'
    elif kind in ('old_hint','wrong_binding','false_clock'):
        parent=deepcopy(contract)
        if kind=='wrong_binding':parent['binding']['lease_id']='changed'
        msg.diagnostic='TARGET_PROCESSING_PENDING: '+json.dumps(dict(
            station_compute_wait=parent,station_compute_recovery_chain=[parent],clock_only=kind!='false_clock'))
        if kind=='old_hint':msg.header.stamp=at_ns(c.now[0]-1)
    n.grasp6d_preview_plan_cb(msg)
    assert n._station_contact_proposal is None
    assert n.latest_grasp6d_preview_plan==msg


@pytest.mark.parametrize('value',[None,0,1,'true',[],{}])
def test_clock_schema_never_treats_truthy_values_as_clock_proof(value):
    assert not pending_payload_schema(dict(station_compute_wait={},station_compute_recovery_chain=[],clock_only=value))


@pytest.mark.parametrize('flag',[False,True])
def test_old_owner_payload_schema_accepts_typed_extension_without_changing_owner_validation(flag):
    assert pending_payload_schema(dict(station_compute_wait={},station_compute_recovery_chain=[],clock_only=flag))
    assert pending_payload_schema(dict(station_compute_wait={},station_compute_recovery_chain=[]))


@pytest.mark.parametrize('epoch',[0,-1])
def test_receipt_preserves_existing_station_contract_epoch_restriction(station,epoch):
    c=station;plan,receipt,_=computed(c)
    receipt['ticket']['target_epoch']=epoch
    for row in receipt['sources']+[receipt['current']]:
        row['contract']['binding']['target_epoch']=epoch
    # Generic TargetIdentity accepts zero, but this existing station contract
    # requires strictly positive target_epoch. The new receipt cannot relax it.
    with pytest.raises(ValueError,match='station_compute_wait_invalid_integer'):
        validate_receipt(receipt,plan,now_ns=c.now[0])


@pytest.mark.parametrize('kind',['object','mask','ready_contract','ready_mask','ready_reference'])
def test_original_registered_source_conflict_is_checked_after_sixteen_cache_eviction(station,kind):
    c=station;plan,receipt,_=computed(c);n=c.node;old=frame(c)
    assert 100_000_000_000 not in n._station_task_objects
    if kind=='object':old['object'].pose_base.pose.position.x+=.001;deliver(c,old,('object',))
    elif kind=='mask':old['mask'].data=bytes([0,255,255,255,255,255]);deliver(c,old,('mask',))
    else:
        if kind=='ready_contract':old['status']['station_compute_wait']['binding']['source_id']='f'*64
        elif kind=='ready_mask':old['status']['station_compute_mask']['sha256']='f'*64
        else:old['status']['anchor_position_base'][0]+=.001
        deliver(c,old,('status',))
    assert n._station_task_retired and n._execution_authority_revoked
    assert not n._station_contact_provenance(plan).ok


@pytest.mark.parametrize('order',[('object','status'),('status','object')])
def test_historical_false_only_closes_with_exact_typed_owner(station,order):
    c=station;plan,receipt,_=computed(c);n=c.node;old=frame(c)
    old['object'].detected=False
    old['status'].update(state='processing_deadline_pending',detail='processing_deadline_pending')
    saved=deepcopy(n._station_task_record)
    deliver(c,old,order)
    assert not n._station_contact_unpaired_false
    assert n._station_task_record==saved
    assert n._station_contact_provenance(plan).ok


@pytest.mark.parametrize('fault',['missing','wrong_owner','support_gap','real_loss','untyped'])
def test_unproven_historical_false_never_authorizes_old_computation(station,fault):
    c=station;plan,receipt,_=computed(c);n=c.node;old=frame(c)
    old['object'].detected=False
    old['status'].update(state='processing_deadline_pending',detail='processing_deadline_pending')
    if fault=='wrong_owner':old['status']['station_compute_wait']['binding']['source_id']='f'*64
    elif fault=='support_gap':old['status']['detail']='support_measurement_pending_revalidation'
    elif fault=='real_loss':old['status']['target_lost']=True
    elif fault=='untyped':old['status'].pop('station_compute_wait')
    deliver(c,old,('object',))
    if fault!='missing':deliver(c,old,('status',))
    result=n._station_contact_provenance(plan)
    assert not result.ok and result.terminal_for_request


def test_different_valid_preview_revokes_only_uncommitted_rebind_candidate(station):
    c=station;plan,receipt,_=computed(c);n=c.node
    n.grasp6d_preview_plan_cb(plan);n._station_contact_rebind_candidate=deepcopy(plan)
    n.grasp6d_preview_plan_cb(deepcopy(plan))
    assert n._station_contact_rebind_candidate==plan
    other=deepcopy(plan);other.poses[2].position.x+=.001;other.plan_id=compute_plan_id(other)
    n.grasp6d_preview_plan_cb(other)
    assert n._station_contact_rebind_candidate is None
    assert n._station_contact_proposal==other


@pytest.mark.parametrize('clock_only',[False,True])
def test_gen10_real_handoff_callbacks_accept_new_typed_clock_schema_without_revocation(station,monkeypatch,clock_only):
    from test_gen10_handoff_authority_independent import published, receive
    c,_,_,messages=published(station,monkeypatch)
    g=messages['geometry'];payload=json.loads(g.failure_reason.partition(':')[2])
    payload['clock_only']=clock_only
    g.failure_reason='TARGET_PROCESSING_PENDING: '+json.dumps(payload)
    for channel in ('geometry','object','status'):
        receive(c,channel,messages[channel])
        assert not c.node._execution_authority_revoked
    result,value=c.node._invoke_plan_bound_action(c.plan,{},'supplemental observation rebind',lambda:'metadata')
    assert result.ok and value=='metadata'


@pytest.mark.parametrize('channel',['object','mask'])
def test_callback_rechecks_trusted_owner_after_plan_lock_acquisition(station,monkeypatch,channel):
    import threading
    from contextlib import contextmanager
    c=station;plan,receipt,_=computed(c);n=c.node;old=frame(c)
    if channel=='object':old['object'].confidence=.5
    else:old['mask'].data=bytes([0,255,255,255,255,255])
    entered=threading.Event();resume=threading.Event();errors=[]
    original=n._grasp6d_plan_guard
    @contextmanager
    def gate():
        entered.set()
        assert resume.wait(1.)
        with original():yield
    monkeypatch.setattr(n,'_grasp6d_plan_guard',gate)
    def callback():
        try:
            if channel=='object':n._station_compute_task_object(old['object'])
            else:n._station_compute_task_mask_cb(old['mask'])
        except BaseException as exc:errors.append(exc)
    worker=threading.Thread(target=callback,daemon=True);worker.start()
    assert entered.wait(1.)
    # Change only the trusted local owner while this callback is waiting.
    # Its incoming payload cannot choose this owner or clear any ledger.
    with original():
        n._station_task_record=None
        n._near_field_phase_id+=1
        n._station_task_disarmed=n._station_task_owner_key()
    resume.set();worker.join(1.)
    assert not worker.is_alive() and not errors
    assert not n._station_task_retired and not n._execution_authority_revoked
    assert receipt['sources'][0]==n._station_contact_sources[100_000_000_000]
