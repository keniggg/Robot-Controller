"""Unsubmitted RGB retries keep the original pixels and never acquire geometry."""
from copy import deepcopy
import json

import numpy as np
import pytest
from std_msgs.msg import String

from test_rgb_handoff_stream import frozen_node, settled
from test_rgb_set_stream import freeze_set
from alicia_grasp_modes.rgb_prefix_preflight import validate_prefix_preflight_receipt


def receipt(n, clock, attempts=1):
    active = n._rgb_acquisition_active
    created = active['deadline_ns']-101_000_000_000
    owner = active['request_id']
    reservations = [owner]+['reservation-%d'%i for i in range(2,attempts+1)]
    ledger = dict(phase='preflight_rejected', request_id=owner, request_ids=reservations,
        reservation_request_id=reservations[-1], reference_id=active['reference_id'],
        source_id=active['source_id'], selection=deepcopy(active['selection']),
        created_ns=str(created), deadline_ns=str(active['deadline_ns']),
        created_monotonic=created*1e-9, deadline_monotonic=created*1e-9+101.,
        duration_sec=101., max_observations=3, attempts=attempts, observation_index=1,
        contact_authorized=False, actuation_prefix_attempted=True,
        actuation_prefix_submission_started=False)
    fields = ('request_id','reference_id','source_id','selection','created_ns','deadline_ns',
        'created_monotonic','deadline_monotonic','duration_sec','max_observations','attempts','observation_index')
    proof = {key:deepcopy(ledger[key]) for key in fields}
    proof.update(kind='rgb_prefix_preflight_no_submission_v1', submission_started=False,
        original_reference_preserved=True, epoch_ns=active['epoch_ns'],
        q_source=[float(value) for value in active['hold_q']], failed_reservation_request_id=reservations[-1],
        last_capture_source_stamp_ns=active['reference'].source_stamp_ns)
    ledger['actuation_prefix_preflight'] = proof
    return ledger


def publish(n, ledger):
    n._rgb_acquisition_status_cb(String(data=json.dumps(ledger)))


@pytest.mark.parametrize('member_set',[False,True])
def test_preflight_retains_original_reference_and_fresh_association_without_geometry(monkeypatch, member_set):
    if member_set:
        n,clock,feed,_,_ = freeze_set(monkeypatch)
    else:
        n,clock,feed,_,_,_ = frozen_node(monkeypatch)
    active=n._rgb_acquisition_active
    reference=active['reference'];hold_q=tuple(active['hold_q'])
    identity=(active['reference_id'],active['deadline_ns'],active['deadline_monotonic'])
    event=receipt(n,clock);publish(n,event)
    assert active['phase']=='preflight_rejected'
    for _ in range(3):clock[0]+=.2;feed()
    assert n._rgb_acquisition_active is active and active['reference'] is reference
    assert tuple(active['hold_q'])==hold_q
    assert (active['reference_id'],active['deadline_ns'],active['deadline_monotonic'])==identity
    interest=n._rgb_observation_interest
    assert interest['reference_id']==identity[0] and interest['association_sha256']
    assert interest['observation_interest_ready'] and not interest['metric_geometry_authority']
    assert not interest['motion_authorized'] and not n.pubs['object'].publish.call_args[0][0].detected
    assert n.tracker.anchor is None and active['completed_observations']==0
    assert len(interest['interest_volumes'])==(2 if member_set else 1)
    settled(n,clock,'reserved',1)
    assert active['phase']=='reserved' and active['reference'] is reference


@pytest.mark.parametrize('change', ['missing_proof','missing_submission','submitted','wrong_kind','proof_submitted',
    'new_reference','new_source','epoch','selection','q','duration','deadline','max_budget','attempts',
    'old_capture','reservation','history','observations','authority','nan','future','expired'])
def test_unproved_preflight_cannot_retain_or_revive_reference(monkeypatch,change):
    n,clock,_,_,_,_=frozen_node(monkeypatch);event=receipt(n,clock);proof=event['actuation_prefix_preflight']
    if change=='missing_proof':event.pop('actuation_prefix_preflight')
    elif change=='missing_submission':event.pop('actuation_prefix_submission_started')
    elif change=='submitted':event['actuation_prefix_submission_started']=True
    elif change=='wrong_kind':proof['kind']='other'
    elif change=='proof_submitted':proof['submission_started']=True
    elif change=='new_reference':proof['reference_id']=event['reference_id']='f'*64
    elif change=='new_source':proof['source_id']=event['source_id']='f'*64
    elif change=='epoch':proof['epoch_ns']+=1
    elif change=='selection':proof['selection']['generation']+=1
    elif change=='q':proof['q_source'][0]+=.001
    elif change=='duration':proof['duration_sec']=102.
    elif change=='deadline':proof['deadline_ns']=str(int(proof['deadline_ns'])+1)
    elif change=='max_budget':proof['max_observations']=event['max_observations']=2
    elif change=='attempts':proof['attempts']=True
    elif change=='old_capture':proof['last_capture_source_stamp_ns']-=1
    elif change=='reservation':proof['failed_reservation_request_id']='changed'
    elif change=='history':event['request_ids']=['other']
    elif change=='observations':proof['observation_index']=event['observation_index']=2
    elif change=='authority':event['contact_authorized']=True
    elif change=='nan':proof['q_source'][0]=float('nan')
    elif change=='future':proof['last_capture_source_stamp_ns']=int((clock[0]+1)*1e9)
    elif change=='expired':clock[0]+=101.
    publish(n,event)
    assert n._rgb_acquisition_active['phase']=='revoked'
    settled(n,clock,'reserved',1)
    assert n._rgb_acquisition_active['phase']=='revoked'


@pytest.mark.parametrize('phase',['revoked','executing','awaiting_measured_target','planning_rejected'])
def test_receipt_cannot_reopen_executed_or_retired_reference(monkeypatch,phase):
    n,clock,_,_,_,_=frozen_node(monkeypatch);event=receipt(n,clock)
    settled(n,clock,phase);publish(n,event)
    assert n._rgb_acquisition_active['phase']=='revoked'


def test_repeated_receipt_keeps_budget_and_original_source_binding(monkeypatch):
    n,clock,_,_,_,_=frozen_node(monkeypatch)
    original=n._rgb_acquisition_active['reference'];event=receipt(n,clock)
    publish(n,event);publish(n,event)
    assert n._rgb_acquisition_active['phase']=='preflight_rejected'
    next_event=receipt(n,clock,2);publish(n,next_event)
    assert n._rgb_acquisition_active['phase']=='preflight_rejected'
    assert n._rgb_acquisition_active['reference'] is original
    publish(n,event)  # Delayed old reservation is not a new retry permission.
    assert n._rgb_acquisition_active['phase']=='revoked'


def test_pose_change_still_fails_original_four_milliradian_gate(monkeypatch):
    n,clock,feed,_,_,_=frozen_node(monkeypatch);publish(n,receipt(n,clock))
    clock[0]+=.2;feed(q=np.array([.005,0.,0.,0.,0.,0.]))
    assert n._rgb_acquisition_active is None
    assert not n._rgb_observation_interest['observation_interest_ready']


def test_shared_receipt_validation_returns_owned_copy(monkeypatch):
    n,clock,_,_,_,_=frozen_node(monkeypatch);event=receipt(n,clock)
    proof=validate_prefix_preflight_receipt(event,now_ns=int(clock[0]*1e9),monotonic_now=clock[0])
    proof['q_source'][0]=.1
    assert event['actuation_prefix_preflight']['q_source'][0]==0.


@pytest.mark.parametrize('field',['attempts','max_observations','observation_index'])
@pytest.mark.parametrize('convert',[bool,float,str])
def test_shared_receipt_rejects_coerced_outer_budget_types(monkeypatch,field,convert):
    n,clock,_,_,_,_=frozen_node(monkeypatch);event=receipt(n,clock)
    event[field]=convert(event[field])
    with pytest.raises(ValueError):validate_prefix_preflight_receipt(event)
