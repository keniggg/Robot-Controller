"""The real ObjectGeometry callback must preserve only proven station waits."""
from copy import deepcopy
import json

import pytest
from test_task_modes import configured
from test_station_compute_wait_task import station, frame, deliver, advance, feedback, at_ns, base
from alicia_grasp_modes.station_compute_reacquisition import build_contract


def invalid(c, contract=None, chain=None, stamp=None):
    m=deepcopy(c.plan.object_geometry)
    m.header.stamp=at_ns(c.now[0] if stamp is None else stamp)
    m.header.frame_id='base_link';m.target_track_id=c.plan.target_track_id;m.valid=False
    m.failure_reason='TARGET_PROCESSING_PENDING: '+json.dumps(dict(
        station_compute_wait=deepcopy(contract or c.node._station_task_record['contract']),
        station_compute_recovery_chain=deepcopy(chain)))
    return m


def test_real_invalid_geometry_quarantines_without_revoking_or_renewing(station):
    c=station;deliver(c,frame(c));saved=deepcopy(c.node._station_task_record)
    c.node.target_geometry_cb(invalid(c))
    assert c.node._station_task_record['pending']
    assert c.node.latest_obj is None and c.node.latest_target_geometry is None
    assert c.node._station_task_record['contract']==saved['contract']
    assert c.node._station_task_record['identity_mono']==saved['identity_mono']
    assert c.node._station_task_record['geometry_mono']==saved['geometry_mono']
    assert not c.node._execution_authority_revoked and not c.node._station_task_retired
    called=[]
    result,_=c.node._invoke_plan_bound_action(c.plan,{},'motion',lambda:called.append(True))
    assert not result.ok and not called
    assert c.node._bound_execution_plan.plan_id==c.plan.plan_id
    c.node._station_compute_task_tick()
    assert not c.node._station_task_retired


@pytest.mark.parametrize('fault', ['reason_only','bad_json','extra_payload','missing_contract',
    'track','frame','future','contract_binding','deadline','unbound_new_contract',
    'target_lost','contact','direct','no_registered','preexisting_revocation','manual','expired',
    'epoch','pose_out_back','moving_stage'])
def test_nonmatching_geometry_keeps_original_revocation(station,fault):
    c=station;deliver(c,frame(c));m=invalid(c)
    if fault=='reason_only':m.failure_reason='TARGET_PROCESSING_PENDING: waiting'
    elif fault=='bad_json':m.failure_reason='TARGET_PROCESSING_PENDING: {'
    elif fault=='target_lost':m.failure_reason='TARGET_LOST: disappeared'
    elif fault=='track':m.target_track_id='other-track'
    elif fault=='frame':m.header.frame_id='camera'
    elif fault=='future':m.header.stamp=at_ns(c.now[0]+1)
    elif fault=='contact':c.node._bound_execution_plan.diagnostic='CONTACT_EXECUTION_PLAN'
    elif fault=='direct':c.node._mode_bound_selection['strategy']='direct'
    elif fault=='no_registered':c.node._station_task_record=None
    elif fault=='preexisting_revocation':c.node._execution_authority_revoked=True
    elif fault=='manual':c.params['/gui/joint_direct_mode']=True
    elif fault=='expired':advance(c,105_000_000_000)
    elif fault=='epoch':c.node._following_epoch_ns+=1
    elif fault=='pose_out_back':feedback(c,100_030_000_000,.01);feedback(c,100_040_000_000,0.)
    elif fault=='moving_stage':c.node.stage=3
    else:
        p=json.loads(m.failure_reason.partition(':')[2])
        if fault=='extra_payload':p['permission']=True
        elif fault=='missing_contract':p.pop('station_compute_wait')
        elif fault=='contract_binding':p['station_compute_wait']['binding']['lease_id']='different'
        elif fault=='deadline':p['station_compute_wait']['identity_deadline_ns']+=1
        else:
            old=p['station_compute_wait']
            p['station_compute_wait']=build_contract(source_ns=100_010_000_000,maximum_frame_age_sec=.8,
                session_deadline_ns=old['session_deadline_ns'],phase_deadline_ns=old['phase_deadline_ns'],binding=old['binding'])
        m.failure_reason='TARGET_PROCESSING_PENDING: '+json.dumps(p)
    c.node.target_geometry_cb(m)
    assert c.node._execution_authority_revoked
    c.node._station_compute_task_tick()
    assert c.node._station_task_record is None or c.node._station_task_retired


def test_delayed_old_invalid_geometry_does_not_quarantine_new_measured_recovery(station):
    c=station;deliver(c,frame(c));old=deepcopy(c.node._station_task_record['contract'])
    m=invalid(c);c.node.target_geometry_cb(m)
    c.now[0]=101_050_000_000;c.mono[0]=11.
    feedback(c,100_980_000_000);feedback(c,101_020_000_000)
    deliver(c,frame(c,101_000_000_000,old));saved=deepcopy(c.node._station_task_record)
    current_geometry=deepcopy(c.plan.object_geometry)
    current_geometry.header.stamp=at_ns(101_000_000_000)
    c.node.latest_target_geometry=current_geometry
    # Delay delivery while retaining the original remote publication timestamp.
    c.node.target_geometry_cb(m)
    assert c.node._station_task_record==saved and not saved['pending']
    assert c.node.latest_target_geometry is current_geometry
    assert not c.node._execution_authority_revoked
    # A plain similarly named notification is never a stale-contract exception.
    m.failure_reason='TARGET_PROCESSING_PENDING: unbound stale message'
    m.header.stamp=at_ns(c.now[0])
    c.node.target_geometry_cb(m)
    assert c.node._execution_authority_revoked


def test_foreign_old_contract_cannot_claim_superseded_notification(station):
    c=station;deliver(c,frame(c));old=deepcopy(c.node._station_task_record['contract'])
    c.node.target_geometry_cb(invalid(c));c.now[0]=101_050_000_000;c.mono[0]=11.
    feedback(c,100_980_000_000);feedback(c,101_020_000_000)
    deliver(c,frame(c,101_000_000_000,old))
    changed=deepcopy(old);changed['binding']['source_id']='d'*64
    c.node.target_geometry_cb(invalid(c,changed))
    assert c.node._execution_authority_revoked


def test_remote_ahead_of_task_requires_exact_registered_parent_chain(station):
    c=station;deliver(c,frame(c));old=deepcopy(c.node._station_task_record)
    new=frame(c,100_010_000_000)['status']['station_compute_wait']
    c.node.target_geometry_cb(invalid(c,new,[old['contract'],new]))
    assert c.node._station_task_record['pending'] and not c.node._execution_authority_revoked
    assert c.node._station_task_record['contract']==old['contract']
    assert c.node._station_task_record['identity_mono']==old['identity_mono']


def test_disarm_clears_registered_notification_history(station):
    c=station;deliver(c,frame(c));assert c.node._station_task_proven_contracts
    c.node._station_task_disarm()
    assert not c.node._station_task_proven_contracts


def test_valid_geometry_keeps_original_callback(station,monkeypatch):
    c=station;deliver(c,frame(c));m=invalid(c);m.valid=True;called=[]
    monkeypatch.setattr(base.GraspTaskNode,'target_geometry_cb',lambda self,msg:called.append(msg))
    c.node.target_geometry_cb(m)
    assert called==[m] and not c.node._station_task_record['pending']


def test_delayed_remote_notification_of_missed_ready_proven_by_recovery_chain(station):
    c=station;deliver(c,frame(c));a=deepcopy(c.node._station_task_record['contract'])
    advance(c,100_900_000_000)
    b=frame(c,100_500_000_000)['status']['station_compute_wait']
    notice=invalid(c,b,[a,b])
    c.now[0]=101_050_000_000;c.mono[0]=11.
    feedback(c,100_980_000_000);feedback(c,101_020_000_000)
    new=frame(c,101_000_000_000,b)
    new['status']['station_compute_recovery_chain']=[a,b]
    deliver(c,new)
    saved=deepcopy(c.node._station_task_record)
    assert not saved['pending']
    assert c.node._station_task_proven_contracts[b['source_ns']]==b
    c.node.target_geometry_cb(notice)
    assert c.node._station_task_record==saved
    assert not c.node._execution_authority_revoked and not c.node._station_task_retired


@pytest.mark.parametrize('missed_middle',[False,True])
def test_late_pending_cannot_revoke_fresh_task_that_never_waited(station,missed_middle,monkeypatch):
    c=station;deliver(c,frame(c));a=deepcopy(c.node._station_task_record['contract'])
    b=frame(c,100_100_000_000)['status']['station_compute_wait']
    c.now[0]=100_250_000_000;c.mono[0]=10.2
    feedback(c,100_180_000_000);feedback(c,100_220_000_000)
    new=frame(c,100_200_000_000,b if missed_middle else None)
    if missed_middle:new['status']['station_compute_recovery_chain']=[a,b]
    deliver(c,new);saved=deepcopy(c.node._station_task_record)
    assert not saved['pending'] and c.node._station_task_cutoff==0
    # Use the actual remote publisher, including its current wall-time header.
    from test_remote_modes import configured as remote_configured, base as remote_base
    from alicia_grasp_modes import station_compute_wait_remote as remote_wait
    remote,_=remote_configured.__wrapped__(monkeypatch)
    monkeypatch.setattr(remote_base.rospy,'get_param',lambda name,default=None:c.params.get(name,default))
    offered=b if missed_middle else a
    remote._station_compute_reference=dict(contract=deepcopy(offered))
    remote._station_compute_pending=False
    remote._current_stream_target_identity=lambda:remote_base.TargetTrackIdentity(
        offered['binding']['target_epoch'],offered['binding']['target_track_id'])
    with remote._stream_condition:
        remote._station_compute_quarantine()
    notice=remote.geometry_pub.messages[-1]
    assert notice.header.stamp.to_nsec()==c.now[0]
    assert json.loads(notice.failure_reason.partition(':')[2])['station_compute_wait']==offered
    c.node.target_geometry_cb(notice)
    assert c.node._station_task_record==saved and c.node._station_task_cutoff==0
    assert not c.node._execution_authority_revoked and not c.node._station_task_retired
