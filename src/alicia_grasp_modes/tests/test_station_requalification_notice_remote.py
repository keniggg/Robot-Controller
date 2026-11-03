"""A producer notice withdraws old authority before SDK/task fan-in, without ACK."""
from copy import deepcopy
from types import SimpleNamespace as NS

import pytest

from test_observation_measurement_remote import context
from test_station_compute_wait_remote import station, arm, feedback, row_at
from test_station_requalification_remote import move, executing, pending_status as old_pending_status, status, send, acks
from test_unknown_pending_identity import deliver
from alicia_grasp_modes.observe_return_measurement import STATION_REQUALIFICATION


def pending_status(c, parent, ns, q):
    value = old_pending_status(c, parent, ns, q)
    for key in ('color_stamp_ns', 'depth_stamp_ns', 'tf_lookup_stamp_ns'):
        value[key] = ns
    return value


def old_ticket(parent):
    return NS(payload=(NS(stamp_ns=parent['source_ns'], sample_stamp_ns=(parent['source_ns'],)),))


def test_typed_status_first_revokes_old_ticket_without_ack_or_local_drift_authority(station):
    c=station;n=c.node;p=arm(c);ns,q=move(c,deliver_sdk=False)
    count=len(acks(c));previous=deepcopy(n._measurement_remote_request)
    status(c,pending_status(c,p,ns,q))
    assert n._station_compute_pending and not n._station_compute_soft_wait
    assert not n.geometry_pub.messages[-1].valid
    assert n._station_compute_ticket_code(old_ticket(p))=='TARGET_PROCESSING_PENDING'
    assert len(acks(c))==count
    assert n._station_requalification_hold is None
    assert not n._station_requalification_used
    assert n._measurement_remote_request==previous
    assert n._measurement_remote_admission is None
    assert n._current_stream_target_identity()==c.identity
    # A new READY from the canceled station cannot undo this pending notice.
    candidate=row_at(c,ns,recovery=p);deliver(n,candidate)
    assert n._station_compute_pending
    assert n._station_compute_reference['contract']==p
    # The ordinary local SDK proof and task lease admission still run.
    feedback(c,ns,q)
    assert n._station_requalification_hold is not None
    r=executing(c,p,ns,q);send(c,r)
    assert acks(c)[-1]['accepted']
    assert n._measurement_remote_request==r
    assert n._station_compute_pending


@pytest.mark.parametrize('bad',['unregistered_parent','foreign_selection','foreign_session','foreign_view',
    'foreign_epoch','wrong_state','wrong_detail','wrong_mode','future_source','old_source','identity_binding','malformed_witness'])
def test_nonmatching_notice_cannot_enroll_or_quarantine_registered_station(station,bad):
    c=station;n=c.node;p=arm(c);ns,q=move(c,deliver_sdk=False)
    message=pending_status(c,p,ns,q)
    b=message[STATION_REQUALIFICATION]['parent_contract']['binding']
    if bad=='unregistered_parent':b['source_id']='d'*64
    elif bad=='foreign_selection':message['generation']+=1
    elif bad=='foreign_session':b['session_id']='other'
    elif bad=='foreign_view':b['view_index']+=1
    elif bad=='foreign_epoch':b['control_epoch_ns']+=1
    elif bad=='wrong_state':message['state']='waiting'
    elif bad=='wrong_detail':message['detail']='other'
    elif bad=='wrong_mode':message['mode']='carton'
    elif bad=='future_source':message['source_stamp_ns']=n._ros_source_clock_ns()+1
    elif bad=='old_source':message['source_stamp_ns']=p['source_ns']
    elif bad=='identity_binding':message['anchor_position_base'][0]+=.01
    else:message[STATION_REQUALIFICATION]['feedback_q_rad']=[0.]*6
    count=len(acks(c));status(c,message)
    assert not n._station_compute_pending
    assert not n._station_compute_retired
    assert n._station_compute_reference['contract']==p
    assert n._station_requalification_hold is None and not n._station_requalification_used
    assert len(acks(c))==count


@pytest.mark.parametrize('loss',['target_lost','first_loss'])
def test_notice_does_not_swallow_real_producer_loss(station,loss):
    c=station;n=c.node;p=arm(c);ns,q=move(c,deliver_sdk=False)
    message=pending_status(c,p,ns,q)
    if loss=='target_lost':message['target_lost']=True
    else:message['first_loss']={'reason':'measured_target_mismatch'}
    status(c,message)
    assert n._station_compute_retired
    assert n._station_requalification_hold is None


def test_notice_without_local_drift_expires_at_original_registered_deadline(station):
    c=station;n=c.node;p=arm(c);ns,q=move(c,deliver_sdk=False)
    status(c,pending_status(c,p,ns,q))
    assert n._station_compute_pending
    c.clock[0]=p['identity_deadline_ns']/1e9+.001
    feedback(c,n._ros_source_clock_ns(),[0.]*6)
    n._station_compute_poll()
    assert n._station_compute_retired
    assert n._station_requalification_hold is None
    assert n._measurement_remote_request is None
