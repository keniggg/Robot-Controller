"""Independent unchanged deadlines, owner rejection and ACK taxonomy edges."""
from copy import deepcopy
from types import MethodType, SimpleNamespace as NS
import threading

import pytest
from test_measurement_timeout_reason import exchange, task, contract
from test_observe_return_measurement import request


def pair(c, qualified=False):
    for participant in contract.PARTICIPANTS: c.ack(participant, qualified)


def test_ros_clock_stall_cannot_renew_original_monotonic_capture_budget(exchange, monkeypatch):
    c=exchange;pair(c)
    monkeypatch.setattr(task.rospy.Time,'now',staticmethod(lambda:NS(to_nsec=lambda:10_400_000_000)))
    before=deepcopy(c.req)
    with pytest.raises(ValueError,match='^OBSERVATION_MEASUREMENT_QUALIFICATION_TIMEOUT: pending=unknown,remote$'):
        c.node._observation_measurement_publish(c.req,qualified=True)
    assert 13.0<=c.now[0]<13.021 and c.sent==[before] and c.req==before


@pytest.mark.parametrize('cutoff,reason',[('capture_deadline_ns','NOT_QUALIFIED'),('deadline_ns','EXPIRED')])
def test_ros_expiry_before_monotonic_cutoff_keeps_original_rejection(exchange,monkeypatch,cutoff,reason):
    c=exchange;c.ack('unknown',True);c.ack('remote')
    def jump():
        monkeypatch.setattr(task.rospy.Time,'now',staticmethod(lambda:NS(to_nsec=lambda:c.req[cutoff])))
    c.callback[0]=jump
    with pytest.raises(ValueError,match='^OBSERVATION_MEASUREMENT_'+reason+'$'):
        c.node._observation_measurement_publish(c.req,qualified=True)
    assert c.now[0]<10.5


@pytest.mark.parametrize('owner',['epoch','session','manual','stop'])
def test_live_owner_fault_still_exits_before_timeout(exchange,owner):
    c=exchange;pair(c)
    c.node._following_evidence_lock=threading.RLock()
    c.node._following_epoch_ns=c.req['control_epoch_ns']
    c.node._observe_return_session={key:c.req[key] for key in ('session_id','view_plan_id','target_epoch','target_track_id')}
    c.node._observe_return_live=lambda:None
    c.node._observation_measurement_live=MethodType(task.ObserveReturnTask._observation_measurement_live,c.node)
    expected={'epoch':'OBSERVATION_MEASUREMENT_CONTROL_EPOCH_CHANGED',
              'session':'OBSERVATION_MEASUREMENT_SESSION_CHANGED',
              'manual':'independent_manual_revocation','stop':'independent_stop_revocation'}[owner]
    def revoked(): raise ValueError(expected)
    def change():
        if owner=='epoch':c.node._following_epoch_ns+=1
        elif owner=='session':c.node._observe_return_session['view_plan_id']='changed'
        else:c.node._observe_return_live=revoked
    c.callback[0]=change
    with pytest.raises(ValueError,match='^'+expected+'$'):
        c.node._observation_measurement_publish(c.req,qualified=True)
    assert c.now[0]<10.5


@pytest.mark.parametrize('fault',['short','duplicate','future','bool'])
def test_false_qualified_source_proof_remains_immediate_failure(exchange,fault):
    c=exchange
    def mutate(ack):
        if fault=='short':ack['source_stamps_ns'].pop()
        elif fault=='duplicate':ack['source_stamps_ns'][1]=ack['source_stamps_ns'][0]
        elif fault=='future':ack['source_stamps_ns'][-1]=11_000_000_000
        else:ack['source_stamps_ns'][0]=True
    c.ack('unknown',True,mutate);c.ack('remote',True)
    with pytest.raises(ValueError,match='^OBSERVATION_MEASUREMENT_FRESH_FRAMES_REQUIRED$'):
        c.node._observation_measurement_publish(c.req,qualified=True)
    assert c.now[0]==10.4


def test_distinct_individually_valid_qualified_sources_never_timeout_or_succeed(exchange):
    c=exchange;c.ack('unknown',True)
    c.ack('remote',True,lambda a:a['source_stamps_ns'].__setitem__(1,10_250_000_000))
    with pytest.raises(ValueError,match='^OBSERVATION_MEASUREMENT_SOURCE_ACK_MISMATCH$'):
        c.node._observation_measurement_publish(c.req,qualified=True)
    assert c.now[0]==10.4


@pytest.mark.parametrize('fault',['authority','qualified_type','identity_binding'])
def test_later_invalid_ack_cannot_hide_behind_earlier_accepted_pair(exchange,fault):
    c=exchange;pair(c)
    change={'authority':{'metric_geometry_authority':True},
            'qualified_type':{'qualified':1},'identity_binding':{'request_sha256':'changed'}}[fault]
    c.callback[0]=lambda:c.ack('remote',mutate=lambda a:a.update(change))
    with pytest.raises(ValueError,match='^OBSERVATION_MEASUREMENT_ACK_REJECTED:'):
        c.node._observation_measurement_publish(c.req,qualified=True)
    assert c.now[0]<10.5


def test_ordinary_executing_missing_ack_keeps_five_second_limit(exchange):
    c=exchange;c.req.clear();c.req.update(request())
    before=deepcopy(c.req)
    with pytest.raises(ValueError,match='^OBSERVATION_MEASUREMENT_ACK_TIMEOUT$'):
        c.node._observation_measurement_publish(c.req,qualified=False)
    assert 15.4<=c.now[0]<15.421 and c.req==before and c.sent==[before]


def test_ordinary_executing_acceptance_needs_no_qualification(exchange):
    c=exchange;c.req.clear();c.req.update(request());pair(c)
    rows=c.node._observation_measurement_publish(c.req,qualified=False)
    assert c.now[0]==10.4 and all(a['accepted'] and not a['qualified'] for a in rows)


def test_no_time_to_validate_cannot_claim_prior_transport_ack(exchange):
    c=exchange;pair(c);c.now[0]=13.
    with pytest.raises(ValueError,match='^OBSERVATION_MEASUREMENT_ACK_TIMEOUT$'):
        c.node._observation_measurement_publish(c.req,qualified=True)
    assert c.now[0]==13.


def test_previous_request_accepted_pair_cannot_qualify_new_request(exchange):
    c=exchange;pair(c);c.req['request_id']='lease:next-requalifying'
    with pytest.raises(ValueError,match='^OBSERVATION_MEASUREMENT_ACK_TIMEOUT$'):
        c.node._observation_measurement_publish(c.req,qualified=True)
    assert 13.0<=c.now[0]<13.021
