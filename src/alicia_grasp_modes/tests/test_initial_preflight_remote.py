"""Actual producer/remote callbacks; no ROS master or hardware commands."""
from copy import deepcopy
import itertools
import json
from types import SimpleNamespace as NS

import numpy as np
import pytest
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from test_locked_rgb_hold import locked, custom_frame
from test_support_measurement_hold import support_gap_depth
from test_unknown_pending_identity import consumer_for, publication, deliver, module, base, Publisher


def request(scene, action='arm', **changes):
    token = dict(scene.token, action=action, request_id='req-'+action,
                 stamp_ns=scene.n._ros_source_clock_ns())
    token.update(changes)
    scene.n._initial_preflight_request_cb(String(data=json.dumps(token)))
    return json.loads(scene.n._initial_preflight_ack_pub.messages[-1].data)


def feedback(scene, q=None):
    scene.n._station_compute_feedback.clear()
    for ns, values in scene.p._rgb_interest_feedback:
        scene.n._station_compute_feedback.append((ns, np.array(values)))
    msg = JointState()
    msg.header.frame_id = 'sdk_measured'
    msg.header.stamp = module.rospy.Time.from_sec(scene.clock[0])
    msg.name = ['Joint%d' % i for i in range(1, 7)]
    msg.position = [0.]*6 if q is None else q
    scene.n._station_compute_accepted_cb(msg)


@pytest.fixture
def initial(monkeypatch):
    p, clock, feed, joint, params = locked(monkeypatch, initial_station=True)
    n = consumer_for(p, clock, params)
    ready = publication(p)
    deliver(n, ready)
    assert n._unknown_identity_reference is not None
    plan = base.Grasp6DPlan()
    plan.valid = True
    plan.header.stamp = ready['object'].header.stamp
    plan.header.frame_id = 'base_link'
    plan.plan_id = 'initial-observation'
    plan.target_track_id = n._current_stream_target_identity().track_id
    plan.model_choice = 'unknown_tabletop'
    plan.diagnostic = 'FAR_FIELD_OBSERVATION_PLAN'
    n._observe_return_phase = 'observation_prefix'
    n._observe_return_diagnostic_observation_only = False
    n._observe_return_prefix = deepcopy(plan)
    n._observe_return_origin_prefix = deepcopy(plan)
    n.latest_preview_rich_plan = deepcopy(plan)
    n._unknown_identity_publication_started = True
    n._measurement_remote_epoch_ns = 1000000000
    n._initial_preflight_ack_pub = Publisher()
    n.execution_plan_validity_sec = 30.
    token = dict(version=1, token_id='precommit-one', selection=deepcopy(n._mode_selection),
        plan_id=plan.plan_id, target_track_id=plan.target_track_id,
        target_epoch=n._current_stream_target_identity().epoch, control_epoch_ns=1000000000,
        q_station_rad=[0.]*6, anchor_binding=n._unknown_identity_reference['binding'],
        deadline_ns=round(clock[0]*1e9)+10000000000, physical_action_submitted=False)
    s = NS(p=p,n=n,clock=clock,feed=feed,joint=joint,params=params,ready=ready,plan=plan,token=token)
    feedback(s)
    return s


def gap(s):
    custom_frame(s.p,s.clock,s.joint,depth_image=support_gap_depth())
    feedback(s)
    return publication(s.p)


def recover(s):
    s.clock[0]+=.2
    s.feed(target_depth=.35)
    feedback(s)
    return publication(s.p)


def admit_measurement(s):
    n=s.n
    from test_observe_return_measurement import request as measurement_request
    value=measurement_request()
    value.update(selection=deepcopy(n._mode_selection), session_id='initial-session',
        target_epoch=s.token['target_epoch'], target_track_id=s.token['target_track_id'],
        origin_plan_id=s.plan.plan_id, origin_snapshot_stamp_ns=s.plan.header.stamp.to_nsec(),
        view_plan_id=s.plan.plan_id, view_source_stamp_ns=s.plan.header.stamp.to_nsec(),
        plan_sha256=n._execution_prefix_digest(s.plan), control_epoch_ns=s.token['control_epoch_ns'],
        deadline_ns=s.token['deadline_ns'], stamp_ns=n._ros_source_clock_ns())
    n._observe_return_session=dict(value,version=2,phase='observing')
    n._observe_return_phase='observing';n._observe_return_paused=True
    n._measurement_remote_ack_pub=Publisher()
    n._measurement_remote_request_cb(String(data=json.dumps(value)))
    assert n._measurement_remote_request is not None, [m.data for m in n._measurement_remote_ack_pub.messages]
    assert n._measurement_remote_request['state']=='executing'
    assert json.loads(n._measurement_remote_ack_pub.messages[-1].data)['accepted']
    return value


@pytest.mark.parametrize('order',list(itertools.permutations(('object','mask','status'))))
def test_published_initial_plan_holds_only_identity_with_exact_recovery(initial,order):
    s=initial;n=s.n
    assert request(s)['accepted']
    original_identity=n._current_stream_target_identity()
    execution_publications=len(n.rich_plan_pub.messages)
    pending=gap(s);deliver(n,pending,order)
    assert n._unknown_identity_pending is not None
    assert n._current_stream_target_identity()==original_identity
    geometry=n.geometry_pub.messages[-1]
    assert geometry.header.stamp.to_nsec()==pending['status']['source_stamp_ns']
    payload=json.loads(geometry.failure_reason.partition(':')[2])['initial_preflight']
    assert payload==dict(token_id=s.token['token_id'],reference_source_ns=s.ready['status']['source_stamp_ns'],
                        pending_source_ns=pending['status']['source_stamp_ns'])
    assert not geometry.valid and not n.latest_object.detected
    # In contrast to normal unpublished pending, no execution-plan tombstone.
    assert len(n.rich_plan_pub.messages)==execution_publications
    assert n.preview_rich_plan_pub.messages[-1].candidate_source=='initial_preflight_pending'
    assert request(s,'commit')['accepted'] is False
    recovered=recover(s);deliver(n,recovered,order)
    assert n._unknown_identity_pending is None
    assert n.latest_object.header.stamp==recovered['object'].header.stamp
    assert n._current_stream_target_identity()==original_identity
    admit_measurement(s)
    assert request(s,'commit')['state']=='committed'
    assert request(s)['accepted'] is False


@pytest.mark.parametrize('missing',('object','mask','status'))
def test_missing_recovery_component_never_commits(initial,missing):
    s=initial;assert request(s)['accepted'];deliver(s.n,gap(s))
    restored=recover(s);deliver(s.n,restored,tuple(k for k in ('object','mask','status') if k!=missing))
    reply=request(s,'commit')
    assert reply['accepted'] is False
    assert s.n._initial_preflight_remote['state']!='committed'


def test_unpaired_false_object_blocks_commit_before_status(initial):
    s=initial;assert request(s)['accepted'];row=gap(s)
    deliver(s.n,row,('object',))
    assert s.n._unknown_identity_pending['unpaired']
    assert request(s,'commit')['accepted'] is False


def test_arm_rejoins_new_source_without_renewing_stale_reference(initial):
    s=initial;old=s.n._unknown_identity_reference['source_ns']
    # The producer continues normal fresh measurements while published-prefix
    # scope prevents the remote from registering newer references.
    for _ in range(4):
        s.clock[0]+=.2
        s.feed(target_depth=.35);row=publication(s.p);feedback(s);deliver(s.n,row)
    assert s.n._unknown_identity_reference['source_ns']==old
    reply=request(s)
    assert reply['accepted'],reply
    assert reply['reference_source_ns']==row['status']['source_stamp_ns']
    assert s.n._unknown_identity_reference['deadline_ns']==row['status']['source_stamp_ns']+800000000


@pytest.mark.parametrize('change',('manual','epoch','q_out_and_back','selection','prefix','contact','supplement','deadline'))
def test_context_change_cannot_arm_or_keep_initial_exception(initial,change):
    s=initial;assert request(s)['accepted']
    if change=='manual':s.params['/gui/joint_direct_mode']=True
    elif change=='epoch':s.n._measurement_remote_epoch_ns+=1
    elif change=='q_out_and_back':feedback(s,[.01,0,0,0,0,0]);feedback(s)
    elif change=='selection':s.params[module.SELECTION_PARAM]=dict(s.n._mode_selection,generation=9)
    elif change=='prefix':s.n._observe_return_origin_prefix.plan_id='other'
    elif change=='contact':s.n._observe_return_phase='observing_contact'
    elif change=='supplement':s.n._observe_return_pending_supplement={'view_index':2}
    else:s.clock[0]=s.token['deadline_ns']/1e9+.0001
    assert request(s,'commit')['accepted'] is False
    assert s.n._initial_preflight_remote['state']!='committed'


def test_pending_keeps_original_source_and_completion_deadline(initial):
    s=initial;assert request(s)['accepted'];deliver(s.n,gap(s))
    pending=deepcopy(s.n._unknown_identity_pending)
    fixed=pending['identity_completion']['identity_completion_deadline_ns']
    s.clock[0]=fixed/1e9+.00001
    feedback(s)
    s.n._unknown_identity_poll()
    assert s.n._unknown_identity_retired_selection==s.n._mode_selection
    assert request(s,'commit')['accepted'] is False
    assert s.n.geometry_pub.messages[-1].failure_reason.startswith('TARGET_LOST:')


def test_duplicate_arm_does_not_extend_token_or_source_deadline(initial):
    s=initial;assert request(s)['accepted']
    old=deepcopy(s.n._initial_preflight_remote)
    source_deadline=s.n._unknown_identity_reference['deadline_ns']
    assert request(s)['accepted']
    assert s.n._initial_preflight_remote==old
    assert s.n._unknown_identity_reference['deadline_ns']==source_deadline
    assert request(s,deadline_ns=s.token['deadline_ns']+1)['accepted'] is False


def test_published_prefix_without_explicit_token_keeps_original_fail_closed_behavior(initial):
    s=initial;old=s.n._current_stream_target_identity();deliver(s.n,gap(s))
    assert s.n._unknown_identity_pending is None
    assert s.n._current_stream_target_identity()!=old
    assert s.n.geometry_pub.messages[-1].failure_reason.startswith('TARGET_LOST:')


@pytest.mark.parametrize('after_commit',(False,True))
@pytest.mark.parametrize('order',list(itertools.permutations(('object','mask','status'))))
def test_proven_old_gap_replay_between_original_lease_ack_and_commit(initial,after_commit,order):
    s=initial;assert request(s)['accepted']
    pending=gap(s);deliver(s.n,pending)
    pending_ack=request(s,'commit')
    assert pending_ack['pending_source_ns']==pending['status']['source_stamp_ns']
    restored=recover(s);deliver(s.n,restored)
    assert pending['status']['source_stamp_ns'] in s.n._initial_preflight_remote['proven_gap_statuses']
    admit_measurement(s)
    identity=s.n._current_stream_target_identity()
    if after_commit:assert request(s,'commit')['accepted']
    deliver(s.n,pending,order)
    feedback(s)
    s.n._unknown_identity_poll()
    assert s.n._current_stream_target_identity()==identity
    assert s.n._measurement_remote_request['state']=='executing'
    if not after_commit:assert request(s,'commit')['accepted']


@pytest.mark.parametrize('fault',('unproven','new_source','changed_status','other_session'))
def test_handoff_does_not_ignore_unproven_or_changed_false(initial,fault):
    s=initial;assert request(s)['accepted'];pending=gap(s);deliver(s.n,pending)
    deliver(s.n,recover(s));admit_measurement(s)
    identity=s.n._current_stream_target_identity()
    if fault=='unproven':s.n._initial_preflight_remote['proven_gap_statuses'].clear()
    elif fault=='new_source':pending['object'].header.stamp=module.rospy.Time.from_sec(s.clock[0])
    elif fault=='changed_status':s.n._unknown_identity_status[pending['status']['source_stamp_ns']]['state']='target_lost'
    else:s.n._observe_return_session['session_id']='other'
    s.n.object_cb(pending['object'])
    assert s.n._current_stream_target_identity()!=identity
    assert s.n._measurement_remote_request is None


def test_commit_without_original_measurement_lease_is_forbidden(initial):
    s=initial;assert request(s)['accepted']
    response=request(s,'commit')
    assert not response['accepted']
    assert response['reason']=='initial_preflight_original_measurement_handoff_missing'


def test_feedback_handoff_context_does_not_repeat_rpc_or_plan_serialization(initial,monkeypatch):
    s=initial;assert request(s)['accepted'];admit_measurement(s)
    calls=[]
    get_param=module.rospy.get_param
    digest=s.n._execution_prefix_digest
    monkeypatch.setattr(module.rospy,'get_param',lambda *a,**k:(calls.append('rpc') or get_param(*a,**k)))
    monkeypatch.setattr(module.rospy,'get_param_cached',lambda name,default=None:s.params.get(name,default))
    monkeypatch.setattr(s.n,'_execution_prefix_digest',lambda p:(calls.append('digest') or digest(p)))
    s.n._initial_preflight_context(allow_handoff=True)
    assert calls==[]
    assert request(s,'commit')['accepted']
    assert 'rpc' in calls and 'digest' in calls
