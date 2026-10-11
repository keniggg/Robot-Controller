"""Real SDK ingress and measurement callbacks; synthetic RGB-D, no ROS calls."""
from copy import deepcopy
import json

import pytest
import numpy as np
from std_msgs.msg import String

from alicia_grasp_modes.observe_return_measurement import STATION_REQUALIFICATION, station_requalification_witness
from test_station_compute_wait_producer import armed
from test_observation_measurement_unknown import ack, send
from test_locked_rgb_hold import module, latest_status, custom_frame


def shifted(monkeypatch):
    n, clock, joint, params, marker, source = armed(monkeypatch)
    parent = deepcopy(n._station_compute_registration['contract'])
    q = marker['arrival_q'].copy(); q[5] -= 0.006135923151542567
    clock[0] += .1
    joint(clock[0], q.tolist())
    assert n._station_requalification_hold is not None
    assert 'observation_arrival_retry' not in n._locked_rgb_hold_reference
    n.process(None)
    assert latest_status(n)['detail'] == STATION_REQUALIFICATION
    assert n.first_loss is None and not n.tracker.lost
    return n, clock, joint, params, marker, parent, q


def renewal(n, clock, marker):
    # Full qualified receipt is kept in the producer's completed diagnostic.
    previous = deepcopy(n._observation_measurement_status['request'])
    clock[0] += .01
    request = dict(previous, state='executing', lease_id='new-station', request_id='new-station:executing',
        stamp_ns=module.rospy.Time.now().to_nsec(), arrival_stamp_ns=0, capture_deadline_ns=0,
        **{STATION_REQUALIFICATION:deepcopy(n._station_requalification_hold['witness'])})
    request.pop('source_stamps_ns', None); request.pop('settle', None)
    send(n, request)
    assert ack(n)['accepted'], ack(n)
    return request


@pytest.mark.parametrize('finalize', ['success', 'late'])
def test_real_pose_callback_then_new_three_frames_preserve_identity_and_replace_station(monkeypatch, finalize):
    n, clock, joint, params, marker, parent, q = shifted(monkeypatch)
    anchor = n.tracker.anchor; plane = n.tracker.plane_base.copy()
    previous_source = n.last_good_source_stamp_ns
    request = renewal(n, clock, marker)
    assert n._locked_rgb_hold_reference is None
    start = clock[0]
    for _ in range(10):
        clock[0] += .1; joint(clock[0], q.tolist())
    now = module.rospy.Time.now().to_nsec()
    request = dict(request, state='requalifying', request_id='new-station:requalifying',
        stamp_ns=now, arrival_stamp_ns=now,
        capture_deadline_ns=min(now+3_000_000_000, request[STATION_REQUALIFICATION]['deadline_ns']),
        settle=dict(epoch_ns=n._rgb_interest_epoch,
            stationary_window_start_ns=module.rospy.Time.from_sec(start+.1).to_nsec(),
            stationary_window_end_ns=now))
    send(n, request); assert ack(n)['accepted'], ack(n)
    def station_joint(stamp, values=None):
        joint(stamp, q.tolist())
    count=n.pubs['object'].publish.call_count
    for i in range(3):
        clock[0]+=.2
        station_joint(clock[0]-.1)
        custom_frame(n, clock, station_joint, target_depth=.35)
        assert n.pubs['object'].publish.call_count == count+(i==2), latest_status(n)
    assert ack(n)['qualified'], latest_status(n)
    sources = ack(n)['source_stamps_ns']
    assert sources[0] > request['arrival_stamp_ns'] > previous_source
    clock[0]+=.01
    done=dict(request,state='qualified',request_id='new-station:qualified',
        stamp_ns=module.rospy.Time.now().to_nsec(),source_stamps_ns=sources)
    send(n,done); assert ack(n)['accepted'],ack(n)
    assert n._station_requalification_hold['finalizing']
    assert np.array_equal(n._locked_rgb_hold_reference['observation_arrival_retry']['arrival_q'], q)
    assert n.tracker.anchor is anchor and np.array_equal(n.tracker.plane_base,plane)
    assert not n.tracker.lost and n.first_loss is None
    phase=deepcopy(n._station_compute_phase);phase.phase_id+=1
    phase.header.stamp=module.rospy.Time.now()
    phase.reference_source_stamp=module.rospy.Time.from_sec(sources[-1]*1e-9)
    n._station_compute_phase_cb(phase)
    if finalize=='late':
        clock[0]=request[STATION_REQUALIFICATION]['deadline_ns']*1e-9+.01
    clock[0]+=.2
    station_joint(clock[0]-.1)
    custom_frame(n,clock,station_joint,target_depth=.35)
    if finalize=='late':
        assert n.first_loss is not None and n._station_compute_registration is None
    else:
        fresh=n._station_compute_registration
        assert fresh is not None, latest_status(n)
        assert fresh['contract']['binding']['phase_id']==phase.phase_id
        assert fresh['contract']['binding']['lease_id']==request['lease_id']
        assert fresh['contract']['source_ns']>sources[-1]
        assert n._station_requalification_hold is None
        assert n.first_loss is None


@pytest.mark.parametrize('fault',['manual','epoch','prior_loss','expired','fake_q'])
def test_new_lease_cannot_revive_other_faults(monkeypatch,fault):
    n,clock,joint,params,marker,parent,q=shifted(monkeypatch)
    previous=deepcopy(n._observation_measurement_status['request']); clock[0]+=.01
    request=dict(previous,state='executing',lease_id='new-station',request_id='new-station:executing',
        stamp_ns=module.rospy.Time.now().to_nsec(),arrival_stamp_ns=0,capture_deadline_ns=0,
        **{STATION_REQUALIFICATION:deepcopy(n._station_requalification_hold['witness'])})
    request.pop('source_stamps_ns',None);request.pop('settle',None)
    if fault=='manual':params['/gui/joint_direct_mode']=True
    elif fault=='epoch':n._rgb_interest_epoch+=1
    elif fault=='prior_loss':n.first_loss={'reason':'true identity loss'}
    elif fault=='expired':clock[0]=request[STATION_REQUALIFICATION]['deadline_ns']*1e-9+.01
    else:request[STATION_REQUALIFICATION]['feedback_q_rad'][5]*=2
    old_ack=deepcopy(ack(n))
    send(n,request)
    assert n._observation_measurement is None
    if fault=='expired':
        assert ack(n)==old_ack  # Parser rejects an expired envelope before ACK.
    else:
        assert not ack(n)['accepted']


def test_local_sdk_witness_survives_more_than_128_later_samples(monkeypatch):
    n,clock,joint,params,marker,parent,q=shifted(monkeypatch)
    witness=deepcopy(n._station_requalification_hold['witness'])
    for _ in range(160):
        clock[0]+=.002; joint(clock[0],q.tolist())
    assert all(ns!=witness['feedback_stamp_ns'] for ns,_ in n._rgb_interest_feedback)
    renewal(n,clock,marker)
    assert n.first_loss is None


@pytest.mark.parametrize('sdk', [True,False,'late_revoked'])
def test_request_before_local_sdk_never_acks_claimed_feedback(monkeypatch,sdk):
    n,clock,joint,params,marker,source=armed(monkeypatch)
    parent=deepcopy(n._station_compute_registration['contract'])
    q=marker['arrival_q'].copy(); q[5]-=.006135923151542567
    previous=deepcopy(n._observation_measurement_status['request'])
    clock[0]+=.1; stamp=module.rospy.Time.now().to_nsec()
    witness=station_requalification_witness(parent,stamp,q.tolist(),stamp)
    clock[0]+=.01
    request=dict(previous,state='executing',lease_id='new-station',request_id='new-station:executing',
        stamp_ns=module.rospy.Time.now().to_nsec(),arrival_stamp_ns=0,capture_deadline_ns=0,
        **{STATION_REQUALIFICATION:witness})
    request.pop('source_stamps_ns',None);request.pop('settle',None)
    old=deepcopy(ack(n)); send(n,request)
    assert ack(n)==old and n._observation_measurement is None
    assert n._station_requalification_admission is not None
    if sdk=='late_revoked':
        clock[0]=witness['deadline_ns']*1e-9+.01
        revoked=dict(request,state='revoked',request_id='new-station:revoked',
            stamp_ns=module.rospy.Time.now().to_nsec(),reason='task_cancelled')
        send(n,revoked)
        assert n._station_requalification_admission is None
        assert n._observation_measurement is None and not ack(n)['accepted']
        assert ack(n)['request_id']==revoked['request_id']
        return
    if sdk:
        joint(stamp*1e-9,q.tolist())
    else:
        clock[0]=witness['deadline_ns']*1e-9+.01
    n.process(None)
    assert ack(n)['request_id']==request['request_id']
    assert ack(n)['accepted'] is sdk
    assert (n._observation_measurement is not None) is sdk
