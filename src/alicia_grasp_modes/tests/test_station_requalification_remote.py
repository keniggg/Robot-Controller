"""Actual ModeAware callback/MRO admission, using only mocked ROS I/O."""
from copy import deepcopy
import itertools
import json
from types import SimpleNamespace as NS

import pytest
from std_msgs.msg import String

from test_observation_measurement_remote import context, request, at, fresh, deliver_one
from test_station_compute_wait_remote import station, arm, feedback, row_at
from test_unknown_pending_identity import deliver
from alicia_grasp_modes.observe_return_measurement import (
    STATION_REQUALIFICATION, station_requalification_witness)


def move(c, q=None, *, deliver_sdk=True):
    q = q or [0., 0., 0., 0., 0., .006135923]
    c.clock[0] += .1
    ns = c.node._ros_source_clock_ns()
    if deliver_sdk:
        feedback(c, ns, q)
    return ns, q


def executing(c, parent, ns, q):
    old = c.node._measurement_remote_request
    witness = station_requalification_witness(parent, ns, q, c.node._ros_source_clock_ns())
    c.clock[0] += .001
    r = dict(old, lease_id='requalified-station', request_id='requal-start',
        state='executing', stamp_ns=c.node._ros_source_clock_ns(), arrival_stamp_ns=0,
        capture_deadline_ns=0, **{STATION_REQUALIFICATION: witness})
    for k in ('settle', 'source_stamps_ns'):
        r.pop(k, None)
    return r


def pending_status(c, parent, ns, q):
    status = deepcopy(c.ready['status'])
    status.update(source_stamp_ns=ns, state='pending', detail=STATION_REQUALIFICATION,
        target_lost=False, first_loss=None,
        **{STATION_REQUALIFICATION: station_requalification_witness(parent, ns, q, c.node._ros_source_clock_ns())})
    return status


def status(c, value):
    c.node._silhouette_source_status_cb(String(data=json.dumps(value)))


def send(c, value):
    c.node._measurement_remote_request_cb(String(data=json.dumps(value)))


def acks(c):
    return [json.loads(m.data) for m in c.node._measurement_remote_ack_pub.messages]


def arrived(c, start):
    c.clock[0] += .4
    now=c.node._ros_source_clock_ns()
    r=dict(start, state='requalifying', request_id='requal-arrived', stamp_ns=now,
        arrival_stamp_ns=now, capture_deadline_ns=min(now+2_000_000_000, start[STATION_REQUALIFICATION]['deadline_ns']),
        settle=dict(epoch_ns=start['control_epoch_ns'],stationary_window_start_ns=now-300_000_000,
            stationary_window_end_ns=now-1))
    send(c,r)
    assert acks(c)[-1]['accepted'], acks(c)[-1]
    return r


def test_own_sdk_drift_cancels_old_metric_tickets_without_retiring_identity(station):
    c=station; parent=arm(c); n=c.node
    ns,q=move(c)
    assert n._station_requalification_hold is not None
    assert n._station_compute_reference is None
    assert n._station_compute_pending and not n._station_compute_retired
    assert n._observe_return_paused
    assert n._current_stream_target_identity()==c.identity
    ticket=NS(payload=(NS(stamp_ns=parent['source_ns'],sample_stamp_ns=(parent['source_ns'],)),))
    assert n._station_compute_ticket_code(ticket)=='TARGET_PROCESSING_PENDING'
    assert not n.geometry_pub.messages[-1].valid
    # Encoder return cannot revive the old geometry or tickets.
    feedback(c,ns+1,[0.]*6)
    assert n._station_compute_reference is None
    assert n._station_compute_pending


@pytest.mark.parametrize('order',list(itertools.permutations(('status','request','sdk'))))
def test_real_status_fanout_and_request_sdk_connections_can_arrive_in_any_order(station,order):
    c=station; parent=arm(c); n=c.node
    ns,q=move(c,deliver_sdk=False)
    row=pending_status(c,parent,ns,q); r=executing(c,parent,ns,q)
    old=len(acks(c))
    for item in order:
        if item=='status': status(c,row)
        elif item=='request': send(c,r)
        else: feedback(c,ns,q)
        n._observation_measurement_poll()
        if 'sdk' not in order[:order.index(item)+1]:
            assert len(acks(c))==old
    assert n._measurement_remote_request==r
    assert n._measurement_remote_admission is None
    assert acks(c)[-1]['accepted'],acks(c)[-1]
    assert n._station_compute_reference is None
    assert n._station_compute_pending and n._observe_return_paused
    assert n._unknown_identity_retired_selection != n._mode_selection


@pytest.mark.parametrize('bad',['manual','epoch','deadline','phase','identity','false_object','target_lost'])
def test_hold_cannot_swallow_manual_epoch_real_loss_or_original_deadline(station,bad):
    c=station; p=arm(c); n=c.node; ns,q=move(c)
    if bad=='manual': c.params['/gui/joint_direct_mode']=True
    elif bad=='epoch': n._measurement_remote_epoch_ns+=1
    elif bad=='deadline': c.clock[0]=p['identity_deadline_ns']/1e9+.001
    elif bad=='phase': c.phase.deadline=at(c.phase.deadline.to_nsec()+1)
    elif bad=='identity': n.target_instance_epoch+=1
    elif bad=='false_object':
        obj=deepcopy(c.ready['object']);obj.detected=False;obj.header.stamp=at(ns+1);n.object_cb(obj)
    else:
        s=deepcopy(c.ready['status']);s.update(target_lost=True,state='target_lost',source_stamp_ns=ns+1);status(c,s)
    n._station_compute_poll()
    assert n._station_compute_retired
    assert n._station_requalification_hold is None


@pytest.mark.parametrize('bad',['unregistered','wrong_q','late_sdk'])
def test_payload_never_proves_local_drift(station,bad):
    c=station;p=arm(c);n=c.node;ns,q=move(c,deliver_sdk=False)
    r=executing(c,p,ns,q)
    if bad=='unregistered':
        r[STATION_REQUALIFICATION]['parent_contract']['binding']['source_id']='d'*64
    if bad=='wrong_q': feedback(c,ns,[0.,0.,0.,0.,0.,.0062])
    send(c,r)
    if bad in ('late_sdk','wrong_q'):
        assert n._measurement_remote_admission is not None
        c.clock[0]=p['identity_deadline_ns']/1e9+.001
        n._observation_measurement_poll()
    assert not acks(c)[-1]['accepted']
    assert n._measurement_remote_request.get('lease_id') != r['lease_id']


def test_historical_locally_registered_parent_is_allowed_without_deadline_extension(station):
    c=station; a=arm(c);n=c.node
    c.clock[0]+=.1;ns=n._ros_source_clock_ns()-1
    feedback(c,ns-1,[0.]*6);feedback(c,ns+1,[0.]*6)
    row=row_at(c,ns);deliver(n,row)
    assert n._station_compute_reference['contract']['source_ns']==ns
    drift,q=move(c)
    r=executing(c,a,drift,q);send(c,r)
    assert acks(c)[-1]['accepted'],acks(c)[-1]
    assert n._station_requalification_hold['deadline_ns']==a['identity_deadline_ns']


@pytest.mark.parametrize('order',list(itertools.permutations(('object','mask','status','ack'))))
def test_requalification_still_requires_three_frame_owner_ack_and_actual_final_join(station,order):
    c=station;p=arm(c);n=c.node;ns,q=move(c)
    r=executing(c,p,ns,q);send(c,r);assert acks(c)[-1]['accepted']
    r=arrived(c,r)
    before=len([a for a in acks(c) if a['qualified']])
    row,sources=fresh(c,r)
    for key in order[:-1]:
        deliver_one(c,row,key)
        assert len([a for a in acks(c) if a['qualified']])==before
    deliver_one(c,row,order[-1])
    assert acks(c)[-1]['qualified'],acks(c)[-1]
    assert acks(c)[-1]['source_stamps_ns']==sources
    assert n._station_compute_reference is None and n._station_compute_pending
    assert n._observe_return_paused
    c.clock[0]+=.001
    done=dict(r,state='qualified',request_id='requalified',stamp_ns=n._ros_source_clock_ns(),source_stamps_ns=sources)
    send(c,done)
    assert acks(c)[-1]['accepted'],acks(c)[-1]
    assert n._station_requalification_hold is not None # New phase still must register.


def test_expiry_between_original_gate_and_qualified_ack_cannot_release(station,monkeypatch):
    c=station;p=arm(c);n=c.node;ns,q=move(c)
    r=executing(c,p,ns,q);send(c,r);r=arrived(c,r)
    row,sources=fresh(c,r)
    original=n._measurement_remote_check_source
    calls=[]
    def gate(req,source):
        result=original(req,source);calls.append(source)
        if len(calls)==1:c.clock[0]=p['identity_deadline_ns']/1e9+.001
        return result
    monkeypatch.setattr(n,'_measurement_remote_check_source',gate)
    for key in ('object','mask','status','ack'):deliver_one(c,row,key)
    assert not acks(c)[-1]['accepted']
    assert n._unknown_identity_retired_selection == n._mode_selection


def qualified_recovery(c):
    p=arm(c);n=c.node;ns,q=move(c)
    r=executing(c,p,ns,q);send(c,r);assert acks(c)[-1]['accepted']
    r=arrived(c,r);row,sources=fresh(c,r)
    # Actual SDK brackets for the new final measurement, independent from ACK.
    feedback(c,sources[-1]-1,q);feedback(c,sources[-1]+1,q)
    for key in ('object','mask','status','ack'):deliver_one(c,row,key)
    assert acks(c)[-1]['qualified']
    c.clock[0]+=.001
    done=dict(r,state='qualified',request_id='done',stamp_ns=n._ros_source_clock_ns(),source_stamps_ns=sources)
    send(c,done);assert acks(c)[-1]['accepted']
    return p,done,row,q


def new_phase_registration(c,done,row,q):
    n=c.node
    # Model the completed reached-view registration's new phase owner here;
    # its reconstruction/registration gates have their separate actual tests.
    c.clock[0]+=.1;start=n._ros_source_clock_ns()
    c.phase=deepcopy(c.phase);c.phase.phase_id+=1;c.phase.header.stamp=at(start)
    c.phase.reference_source_stamp=at(done['source_stamps_ns'][-1])
    n._station_compute_phase=c.phase;n._observe_return_paused=False
    c.binding=deepcopy(c.binding)
    c.binding.update(lease_id=done['lease_id'],phase_id=c.phase.phase_id,
        phase_started_ns=start,phase_reference_source_ns=done['source_stamps_ns'][-1],station_q_rad=q)
    c.ready=row
    c.clock[0]+=.1;ns=n._ros_source_clock_ns()-1
    feedback(c,ns-1,q);feedback(c,ns+1,q)
    row=row_at(c,ns)
    return row


def test_new_phase_first_full_source_registration_ends_hold_and_second_drift_is_hard(station):
    c=station;n=c.node;p,done,row,q=qualified_recovery(c)
    new=new_phase_registration(c,done,row,q)
    for key in ('status','object'):
        deliver(n,new,(key,));assert n._station_requalification_hold is not None
        assert n._station_compute_pending
    deliver(n,new,('mask',))
    assert n._station_requalification_hold is None
    assert n._station_compute_reference['contract']==new['status']['station_compute_wait']
    assert not n._station_compute_pending and not n._station_compute_retired
    old=NS(payload=(NS(stamp_ns=p['source_ns'],sample_stamp_ns=(p['source_ns'],)),))
    assert n._station_compute_ticket_code(old)=='TARGET_PROCESSING_PENDING'
    move(c,[0.,0.,0.,0.,0.,q[-1]+.006])
    assert n._station_compute_retired
    assert n._station_requalification_hold is None


def test_new_phase_first_source_cannot_complete_after_original_recovery_deadline(station):
    c=station;n=c.node;p,done,row,q=qualified_recovery(c)
    new=new_phase_registration(c,done,row,q)
    c.clock[0]=p['identity_deadline_ns']/1e9+.001
    deliver(n,new)
    assert n._station_compute_retired
    assert n._station_compute_reference is None


def test_request_before_sdk_cancels_old_ticket_and_new_old_station_rows_cannot_restore(station):
    c=station;n=c.node;p=arm(c);ns,q=move(c,deliver_sdk=False)
    r=executing(c,p,ns,q);send(c,r)
    assert n._measurement_remote_admission==r
    ticket=NS(payload=(NS(stamp_ns=p['source_ns'],sample_stamp_ns=(p['source_ns'],)),))
    assert n._station_compute_ticket_code(ticket)=='TARGET_PROCESSING_PENDING'
    new=row_at(c,n._ros_source_clock_ns()-1,recovery=p)
    deliver(n,new)
    assert n._station_compute_pending
    assert n._station_compute_reference['contract']==p
    assert n._measurement_remote_request['state']=='qualified'
    assert n._measurement_remote_request['lease_id']!=r['lease_id']


def test_accepted_drift_proof_survives_normal_sdk_history_rollover(station):
    c=station;p=arm(c);n=c.node;ns,q=move(c)
    r=executing(c,p,ns,q)
    # Each callback validates new entries; the locally proven old sample can
    # leave the bounded buffer without becoming an uninspected history gap.
    for i in range(140):
        c.clock[0]+=.002
        feedback(c,n._ros_source_clock_ns(),q)
    assert all(stamp!=ns for stamp,_ in n._station_compute_feedback)
    assert n._station_requalification_hold is not None
    send(c,r)
    assert acks(c)[-1]['accepted'],acks(c)[-1]
    assert n._measurement_remote_request==r
    assert n._station_requalification_hold['deadline_ns']==p['identity_deadline_ns']


def test_uninspected_sdk_overrun_remains_hard_failure(station):
    c=station;p=arm(c);n=c.node;ns,q=move(c)
    msg=deepcopy(n.latest_joint_state);msg.header.frame_id='sdk_measured';msg.position=q
    for i in range(140):
        c.clock[0]+=.001;msg.header.stamp=at(n._ros_source_clock_ns())
        n._station_compute_feedback_ingress_cb(msg)
    n._station_compute_accepted_cb(None)
    assert n._station_compute_retired
