"""Current READY joins at the actual initial commit boundary; no ROS/motion."""
from copy import deepcopy
import itertools
import threading
import time
from types import SimpleNamespace as NS

import pytest

from test_initial_preflight_remote import (initial, request, feedback,
    admit_measurement, publication, deliver, module)


def handoff(s):
    assert request(s)['accepted']
    lease = admit_measurement(s)
    original = deepcopy(s.n._measurement_remote_reference)
    # A producer frame already in flight can complete delivery after the remote
    # ACK. The active remote's real callbacks route it to measurement caches.
    s.clock[0] += .2
    s.feed(target_depth=.35)
    row = publication(s.p)
    feedback(s)
    return lease, original, row


def commit_time(s, original, age=.837253809):
    s.clock[0] = original['source_ns']/1e9 + age
    feedback(s)


@pytest.mark.parametrize('order', list(itertools.permutations(('object', 'mask', 'status'))))
def test_fresh_actual_join_commits_once_without_replacing_admission(initial, order):
    s=initial;n=s.n
    lease, original, row = handoff(s)
    old_object = deepcopy(n.latest_object)
    old_identity = n._current_stream_target_identity()
    geometry_count = len(n.geometry_pub.messages)
    publications = len(n.rich_plan_pub.messages)
    deliver(n, row, order)
    commit_time(s, original)
    assert s.n._ros_source_clock_ns() >= original['source_ns'] + original['maximum_ns']
    assert s.n._ros_source_clock_ns() < row['object'].header.stamp.to_nsec() + original['maximum_ns']
    reply = request(s, 'commit')
    assert reply['accepted'], reply
    assert reply['state'] == 'committed'
    assert reply['commit_reference']['source_ns'] == row['object'].header.stamp.to_nsec()
    assert reply['commit_reference']['admission_source_ns'] == original['source_ns']
    assert reply['commit_reference']['refreshed']
    assert 0 <= reply['commit_reference']['source_age_sec'] < .8
    assert reply['commit_reference']['lease_id'] == lease['lease_id']
    assert n._measurement_remote_reference == original
    assert n._measurement_remote_request == lease
    assert n.latest_object == old_object
    assert n._current_stream_target_identity() == old_identity
    assert len(n.geometry_pub.messages) == geometry_count
    assert len(n.rich_plan_pub.messages) == publications
    # Initial commit stays irreversible. No second begin or physical call is
    # involved in either the join or this final validation.
    assert request(s, 'commit')['accepted'] is False


@pytest.mark.parametrize('missing', ('object','mask','status'))
def test_expired_admission_and_incomplete_current_join_reject(initial, missing):
    s=initial;lease,original,row=handoff(s)
    deliver(s.n,row,tuple(k for k in ('object','mask','status') if k != missing))
    commit_time(s,original)
    reply=request(s,'commit')
    assert not reply['accepted'] and reply['state']=='armed'
    assert reply['reason']=='TARGET_MEASUREMENT_PENDING'
    assert s.n._measurement_remote_request==lease
    assert s.n._measurement_remote_reference==original


@pytest.mark.parametrize('fault', ('confidence','nan_confidence','jump','association',
    'pose_stamp','pose_frame','outer_stamp','outer_frame','position','nan_position',
    'mask_hash','mask_shape','template','station_epoch','station_q','depth_stamp',
    'anchor','observation_receipt','unavailable'))
def test_active_cache_cannot_bypass_original_identity_and_quality(initial, fault):
    s=initial;lease,original,row=handoff(s)
    obj,status,mask=row['object'],row['status'],row['mask']
    if fault=='confidence':obj.confidence=.01
    elif fault=='nan_confidence':obj.confidence=float('nan')
    elif fault in ('jump','association'):
        delta=.13 if fault=='jump' else .09
        obj.pose_base.pose.position.x+=delta;status['position_base'][0]+=delta
    elif fault=='pose_stamp':obj.pose_base.header.stamp=module.rospy.Time(1)
    elif fault=='pose_frame':obj.pose_base.header.frame_id='camera_link'
    elif fault=='outer_stamp':obj.header.stamp=module.rospy.Time(1)
    elif fault=='outer_frame':obj.header.frame_id='camera_link'
    elif fault=='position':obj.pose_base.pose.position.x+=.01
    elif fault=='nan_position':obj.pose_base.pose.position.x=float('nan')
    elif fault=='mask_hash':mask.data=bytes(len(mask.data))
    elif fault=='mask_shape':mask.width-=1
    elif fault=='template':status['initial_station_identity_template']['source_id']='a'*64
    elif fault=='station_epoch':status['initial_station_measurement']['epoch_ns']+=1
    elif fault=='station_q':status['initial_station_measurement']['q_station_rad'][0]+=.01
    elif fault=='depth_stamp':status['depth_stamp_ns']+=1
    elif fault=='anchor':status['anchor_position_base'][0]+=.01
    elif fault=='observation_receipt':status['observation_measurement']={'state':'executing'}
    else:status['state']='unavailable'
    deliver(s.n,row)
    commit_time(s,original)
    assert request(s,'commit')['accepted'] is False
    assert s.n._initial_preflight_remote['state']=='armed'
    assert s.n._measurement_remote_request==lease
    assert s.n._measurement_remote_reference==original


@pytest.mark.parametrize('cache', ('object','mask','status'))
def test_same_source_conflict_across_active_and_ordinary_caches_reject(initial, cache):
    s=initial;_,original,row=handoff(s);deliver(s.n,row)
    ns=row['object'].header.stamp.to_nsec()
    if cache=='object':
        value=deepcopy(row['object']);value.confidence-=.01
        s.n._unknown_identity_objects[ns]=value
    elif cache=='mask':
        value=s.n._initial_preflight_mask_evidence(row['mask'])
        s.n._unknown_identity_masks[ns]=(value[0],value[1],'f'*64,value[3])
    else:
        value=deepcopy(row['status']);value['position_base'][0]+=.01
        s.n._unknown_identity_status[ns]=value
    commit_time(s,original)
    reply=request(s,'commit')
    assert not reply['accepted']
    assert reply['reason']=='initial_preflight_current_source_conflict'


@pytest.mark.parametrize('fault', ('deadline','backward_clock','lease','lease_content','admission','token','state'))
def test_final_context_cannot_expire_or_replace_evidence_before_commit(initial, monkeypatch, fault):
    s=initial;_,original,row=handoff(s);deliver(s.n,row);commit_time(s,original)
    context=s.n._initial_preflight_context
    def changed(*args,**kwargs):
        result=context(*args,**kwargs)
        if kwargs.get('source_ns')==row['object'].header.stamp.to_nsec():
            if fault=='deadline':s.clock[0]=(row['object'].header.stamp.to_nsec()+original['maximum_ns'])/1e9
            elif fault=='backward_clock':s.clock[0]-=.001
            elif fault=='lease':s.n._measurement_remote_request=deepcopy(s.n._measurement_remote_request)
            elif fault=='lease_content':s.n._measurement_remote_request['view_index']=2
            elif fault=='admission':s.n._measurement_remote_reference['source_ns']+=1
            elif fault=='token':s.n._initial_preflight_remote=deepcopy(s.n._initial_preflight_remote)
            else:s.n._initial_preflight_remote['state']='revoked'
        return result
    monkeypatch.setattr(s.n,'_initial_preflight_context',changed)
    reply=request(s,'commit')
    assert not reply['accepted']
    assert reply['reason']=='initial_preflight_current_commit_changed_or_expired'
    assert s.n._initial_preflight_remote['state']!='committed'


def test_fresh_original_admission_retains_existing_commit_path(initial):
    s=initial;assert request(s)['accepted'];admit_measurement(s)
    original=deepcopy(s.n._measurement_remote_reference)
    reply=request(s,'commit')
    assert reply['accepted']
    assert not reply['commit_reference']['refreshed']
    assert reply['commit_reference']['source_ns']==original['source_ns']


def test_actual_task_rpc_consumes_one_commit_before_inert_submission(initial):
    from alicia_grasp_modes.initial_preflight_task import InitialPreflightTaskMixin
    s=initial;lease,original,row=handoff(s);deliver(s.n,row);commit_time(s,original)
    # Execute the real task RPC/ACK/commit/submission methods, using its armed
    # token and the actual remote callbacks. Only transport and actuator are
    # inert; no status, acceptance or context validator is mocked to succeed.
    task=InitialPreflightTaskMixin.__new__(InitialPreflightTaskMixin)
    task._initial_preflight_condition=threading.Condition(threading.RLock())
    task._initial_preflight_lock=threading.RLock()
    task._initial_preflight_acks={}
    task._initial_preflight_token=dict(request=deepcopy(s.token),state='armed',
        monotonic_deadline=time.monotonic()+10)
    task._initial_preflight_attempt=dict(ended_ns=None,disqualifications=[],
        physical_action_submitted=False,submissions=[])
    task.active=True;task._following_epoch_ns=s.token['control_epoch_ns']
    task._mode_bound_selection=deepcopy(s.token['selection'])
    task._bound_execution_plan=s.plan
    task._initial_preflight_pending=None;task._initial_preflight_replan=False
    task._initial_preflight_request_pub=NS(publish=s.n._initial_preflight_request_cb)
    s.n._initial_preflight_ack_pub=NS(publish=task._initial_preflight_ack)
    task._initial_preflight_commit()
    assert task._initial_preflight_token['state']=='committed'
    assert not task._initial_preflight_attempt['physical_action_submitted']
    assert task._initial_preflight_attempt['submissions']==[]
    assert s.n._measurement_remote_request==lease
    assert s.n._measurement_remote_reference==original
    calls=[]
    execute=task._initial_preflight_wrap_strict(lambda:calls.append('inert-action'))
    execute()
    assert calls==['inert-action']
    assert len(task._initial_preflight_attempt['submissions'])==1
