"""Actual producer process(None) notice crosses the real remote status fanout."""
from copy import deepcopy
import json
from types import SimpleNamespace as NS

import pytest
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from test_station_requalification_three_owners import owners
from test_locked_rgb_hold import module, latest_status
from test_station_requalification_remote import executing, send, acks
from alicia_grasp_modes.observe_return_measurement import STATION_REQUALIFICATION


def producer_notice(monkeypatch):
    c=owners(monkeypatch);p=c.producer;r=c.node
    parent=deepcopy(r._station_compute_reference['contract'])
    q=[0.,0.,0.,0.,0.,-.006135923151542567]
    c.clock[0]+=.1
    msg=JointState();msg.header.stamp=module.rospy.Time.now();msg.header.frame_id='sdk_measured'
    msg.name=list(module.JOINT_NAMES);msg.position=q
    # Only the producer receives this SDK packet. Remote's connection is late.
    p._rgb_interest_joint_cb(msg)
    assert p._station_requalification_hold is not None
    assert r._station_requalification_hold is None
    p.process(None)
    value=deepcopy(latest_status(p))
    assert value['state']=='pending' and value['detail']==STATION_REQUALIFICATION
    assert value.get('source_stamp_ns') is None
    assert 'exact_rgbd' not in value and 'tf_lookup_stamp_ns' not in value
    assert value[STATION_REQUALIFICATION]['parent_contract']==parent
    return c,parent,msg,value


def wire(c,value):
    c.node._silhouette_source_status_cb(String(data=json.dumps(value)))


def test_real_producer_no_frame_notice_only_revokes_until_local_sdk_and_task_arrive(monkeypatch):
    c,p,msg,value=producer_notice(monkeypatch);n=c.node
    before=len(acks(c));old_lease=deepcopy(n._measurement_remote_request)
    wire(c,value)
    assert n._station_compute_pending and not n._station_compute_soft_wait
    assert not n.geometry_pub.messages[-1].valid
    ticket=NS(payload=(NS(stamp_ns=p['source_ns'],sample_stamp_ns=(p['source_ns'],)),))
    assert n._station_compute_ticket_code(ticket)=='TARGET_PROCESSING_PENDING'
    assert n._station_requalification_hold is None and not n._station_requalification_used
    assert len(acks(c))==before and n._measurement_remote_admission is None
    assert n._measurement_remote_request==old_lease
    # No image timestamps, geometry or local SDK evidence were fabricated.
    assert all(ns!=msg.header.stamp.to_nsec() for ns,_ in n._station_compute_feedback)
    assert n._station_compute_reference['contract']==p
    assert n._current_stream_target_identity()==c.identity
    n._station_compute_accepted_cb(msg)
    assert n._station_requalification_hold is not None
    req=executing(c,p,msg.header.stamp.to_nsec(),list(msg.position));send(c,req)
    assert acks(c)[-1]['accepted'],acks(c)[-1]
    assert n._measurement_remote_request==req
    assert n._station_compute_pending


@pytest.mark.parametrize('fault',['parent','generation','detail','state','deadline','epoch','partial_frame'])
def test_actual_source_less_notice_still_requires_exact_live_registered_owner(monkeypatch,fault):
    c,p,msg,value=producer_notice(monkeypatch);n=c.node
    if fault=='parent':value[STATION_REQUALIFICATION]['parent_contract']['binding']['source_id']='d'*64
    elif fault=='generation':value['generation']+=1
    elif fault=='detail':value['detail']='another_pending_state'
    elif fault=='state':value['state']='ready'
    elif fault=='deadline':c.clock[0]=p['identity_deadline_ns']/1e9+.001
    elif fault=='epoch':n._measurement_remote_epoch_ns+=1
    else:value['exact_rgbd']=True  # A partial frame envelope is not a state-only notice.
    before=len(acks(c));wire(c,value)
    assert not n._station_requalification_notice
    assert n._station_requalification_hold is None and not n._station_requalification_used
    assert len(acks(c))==before
    assert n._measurement_remote_request['lease_id']==p['binding']['lease_id']


@pytest.mark.parametrize('loss',['target_lost','first_loss'])
def test_no_frame_notice_does_not_hide_explicit_real_target_loss(monkeypatch,loss):
    c,p,msg,value=producer_notice(monkeypatch);n=c.node
    if loss=='target_lost':value['target_lost']=True
    else:value['first_loss']={'reason':'identity_mismatch'}
    wire(c,value)
    assert n._station_compute_retired
    assert n._station_requalification_hold is None
