"""GEN16 exact temporal evidence with synthetic RGB-D through real callbacks.

This is a conditional offline recovery, not a replay of recorded image pixels
or evidence that the failed live generation recovered.
"""
from copy import deepcopy
import itertools
import json
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from alicia_grasp_modes.feedback_bracket_gap import KIND, validate_gap_evidence
from alicia_grasp_modes.rgb_observation_source import bracket_feedback
from test_locked_rgb_hold import locked, latest_status, module
from test_unknown_pending_identity import consumer_for, publication, deliver


RECORD = json.loads((Path(__file__).parent/'fixtures/gen16_feedback_bracket_gap.json').read_text())


def evidence():
    r = RECORD['actual_nearest_feedback']
    return dict(accepted_before=[r['before']['stamp_ns'], r['before']['q']],
        accepted_after=[r['after']['stamp_ns'], r['after']['q']], span_ns=r['span_ns'],
        maximum_span_ns=200_000_000, bracket_qualified=False)


def test_exact_recorded_bad_and_next_good_brackets_keep_original_200ms_gate():
    e = evidence(); epoch = RECORD['selection_stamp_ns']
    with pytest.raises(ValueError, match='accepted_feedback_does_not_bracket_source'):
        bracket_feedback((e['accepted_before'], e['accepted_after']), RECORD['source_stamp_ns'], epoch)
    assert validate_gap_evidence(e, RECORD['source_stamp_ns'], RECORD['last_good_source_stamp_ns'],
        epoch, e['accepted_before'][1], RECORD['first_failure_observed_ns'], .8) == e
    row = RECORD['next_frames'][0]; b = row['offline_bracket']
    q, before, after = bracket_feedback([(b['before_stamp_ns'], b['q']), (b['after_stamp_ns'], b['q'])], row['source_stamp_ns'], epoch)
    assert after[0]-before[0] == 113602517
    np.testing.assert_allclose(q, b['q'], atol=1e-15)


@pytest.fixture
def scene(monkeypatch):
    p, clock, feed, joint, params = locked(monkeypatch, initial_station=True)
    ready = publication(p)
    n = consumer_for(p, clock, params); deliver(n, ready)
    return NS(p=p, node=n, clock=clock, feed=feed, joint=joint, params=params,
        ready=ready, identity=n._current_stream_target_identity(),
        shift=RECORD['last_good_source_stamp_ns']-p.last_good_source_stamp_ns,
        lookup=p.buffer.lookup_transform)


def frame(s, *, good=False, fault=''):
    p = s.p
    if good:
        recorded = RECORD['next_frames'][0]; b = recorded['offline_bracket']
        ns = recorded['source_stamp_ns']-s.shift
        lo, hi = b['before_stamp_ns']-s.shift, b['after_stamp_ns']-s.shift
        finish_ns = recorded['received_ros_ns']-s.shift
    else:
        e = evidence(); ns = RECORD['source_stamp_ns']-s.shift
        lo, hi = e['accepted_before'][0]-s.shift, e['accepted_after'][0]-s.shift
        finish_ns = RECORD['first_failure_observed_ns']-s.shift
    if fault == 'expired': finish_ns = p.last_good_source_stamp_ns+800000001
    s.clock[0] = (ns+18000000)/1e9
    def accepted(stamp, q=None):
        m=JointState();m.header.stamp=module.rospy.Time(stamp//10**9,stamp%10**9)
        m.header.frame_id='sdk_measured';m.name=list(module.JOINT_NAMES);m.position=[0.]*6 if q is None else q
        p._rgb_interest_joint_cb(m)
    if fault.startswith('missing'):p._rgb_interest_feedback.clear()
    if fault not in ('missing_before','missing_both'): accepted(lo)
    fired=[]
    def lookup(*args):
        if args[0]=='base_link' and not fired:
            fired.append(True);s.clock[0]=finish_ns/1e9
            if fault not in ('missing_after','missing_both'):
                accepted(hi, [.005,0.,0.,0.,0.,0.] if fault=='pose' else None)
            if fault=='out_back':
                accepted(hi-1000,[.006,0.,0.,0.,0.,0.]);accepted(hi)
            if fault=='manual':s.params['/gui/joint_direct_mode']=True
            if fault=='epoch':p._rgb_interest_epoch_cb(Header(stamp=module.rospy.Time(2),frame_id='sdk_reference_epoch'))
            if fault=='camera':s.params['/camera']['fx']+=1
        return s.lookup(*args)
    p.buffer.lookup_transform=lookup
    rgb=np.full((240,320,3),(60,90,190),np.uint8);rgb[106:134,148:173]=20
    depth=np.full((240,320),.4,np.float32);depth[106:134,148:173]=.35
    if fault=='bad_support':depth[:]=0.
    pair=(p.bridge.cv2_to_imgmsg(rgb,encoding='bgr8'),p.bridge.cv2_to_imgmsg(depth,encoding='32FC1'))
    for msg in pair:msg.header.stamp=module.rospy.Time(ns//10**9,ns%10**9);msg.header.frame_id=p.camera_frame
    p.latest=pair;p.process(None)
    return publication(p)


@pytest.mark.parametrize('order',list(itertools.permutations(('object','mask','status'))))
def test_real_process_false_pending_then_next_exact_join_recovers_same_identity(scene,order):
    s=scene;p=s.p;n=s.node;ref=p._locked_rgb_hold_reference;marker=ref['initial_station_current_support']
    anchor,last,good_stamp=p.tracker.anchor,p.tracker.last,p.last_good_source_stamp_ns
    deadline=n._unknown_identity_reference['deadline_ns']
    bad=frame(s);q=bad['status']['source_qualification']
    assert q['kind']==KIND and q['source_qualified'] is False
    assert q['feedback_gap_evidence']['span_ns']==200008830
    assert not bad['object'].detected and not np.any(p.bridge.imgmsg_to_cv2(bad['mask'],'mono8'))
    assert p.tracker.anchor is anchor and p.tracker.last is last and not p.tracker.lost
    assert p.first_loss is None and p.last_good_source_stamp_ns==good_stamp
    assert p._locked_rgb_hold_reference is ref and not marker['invalidated_reason']
    deliver(n,bad,order)
    assert n._current_stream_target_identity()==s.identity
    assert n._unknown_identity_pending['deadline_ns']==deadline
    assert not n.latest_object.detected and not n.geometry_pub.messages[-1].valid
    assert not n.submit_stream_snapshot(NS(stamp_ns=bad['object'].header.stamp.to_nsec()))
    new=frame(s,good=True)
    assert new['object'].detected, new['status']
    assert new['status']['source_qualification']['kind']==KIND
    deliver(n,new,order)
    assert n._unknown_identity_pending is None
    assert n._current_stream_target_identity()==s.identity
    assert n.latest_object.header.stamp==new['object'].header.stamp
    assert n.latest_object.detected and not n.rich_plan_pub.messages[-1].valid
    # A duplicate, already validated older false message must not erase recovery.
    deliver(n,bad,order)
    assert n._current_stream_target_identity()==s.identity and n.latest_object.detected
    assert n.latest_object.header.stamp==new['object'].header.stamp


@pytest.mark.parametrize('fault',['missing_before','missing_after','missing_both','pose',
    'out_back','manual','epoch','camera','expired'])
def test_no_new_pending_for_real_station_changes_or_incomplete_feedback(scene,fault):
    row=frame(scene,fault=fault)
    assert not row['object'].detected
    assert scene.p._locked_source_pending is None
    assert scene.p.tracker.lost or fault=='epoch'
    assert (row['status'].get('source_qualification') or {}).get('state')!='pending_feedback_bracket'


@pytest.mark.parametrize('fault',['support','current_bracket','authority','qualified','extended','kind'])
def test_unproved_recovery_or_receipt_does_not_restore_consumer_geometry(scene,fault):
    s=scene;bad=frame(s)
    if fault in ('authority','qualified','extended','kind'):
        q=bad['status']['source_qualification']
        if fault=='authority':q['motion_authorized']=True
        elif fault=='qualified':q['source_qualified']=True
        elif fault=='extended':q['identity_completion']={}
        else:q['kind']='support_measurement_gap'
        deliver(s.node,bad)
    else:
        deliver(s.node,bad)
        new=frame(s,good=True,fault='bad_support' if fault=='support' else '')
        if fault=='current_bracket':new['status']['source_qualification']['requalified_accepted_feedback']=[]
        deliver(s.node,new)
    assert not s.node.latest_object.detected
    assert s.node._current_stream_target_identity()!=s.identity


def test_real_task_preflight_collector_receives_new_kind_and_completes_three_topic_recovery(scene,monkeypatch):
    from test_initial_preflight_task import prepared,feed as task_feed,notice
    s=scene;s.gap=frame(s)
    c=prepared.__wrapped__(s,monkeypatch)
    c.node.target_geometry_cb(notice(c,s.gap));task_feed(c,s.gap)
    assert c.node._initial_preflight_pending and not c.node._execution_authority_revoked
    new=frame(s,good=True);task_feed(c,new)
    assert not c.node._initial_preflight_collector.fatal
    assert not c.node._initial_preflight_pending
    assert not c.node._execution_authority_revoked
    assert c.node._initial_preflight_replan
    assert c.node.latest_obj.header.stamp==new['object'].header.stamp
    assert not c.node._initial_preflight_attempt['physical_action_submitted']
    # Both an older typed geometry notice and all original false-source
    # callbacks remain tombstones after the current complete recovery.
    task_feed(c,s.gap,('status','object','mask'))
    c.node.target_geometry_cb(notice(c,s.gap))
    assert not c.node._initial_preflight_pending
    assert not c.node._execution_authority_revoked
    assert c.node.latest_obj.header.stamp==new['object'].header.stamp


def test_repeated_bad_spans_do_not_extend_original_reference_deadline(scene):
    s=scene;n=s.node;reference=s.p._locked_rgb_hold_reference
    deadline=n._unknown_identity_reference['deadline_ns']
    deliver(n,frame(s))
    first=s.p._locked_source_pending['first_unqualified_source_stamp_ns']
    # A later, genuinely separate 200.008830ms interval, with all recorded
    # prior samples retained and no sample inserted inside this new interval.
    s.shift-=350000000
    deliver(n,frame(s))
    assert s.p._locked_rgb_hold_reference is reference
    assert s.p._locked_source_pending['first_unqualified_source_stamp_ns']==first
    assert n._unknown_identity_pending['deadline_ns']==deadline
    assert 'identity_completion' not in s.p._locked_source_pending
    s.clock[0]=(deadline+1000000)/1e9
    n._unknown_identity_poll();s.p.process(None)
    assert n._current_stream_target_identity()!=s.identity
    assert not n.latest_object.detected
    assert s.p.tracker.lost


def test_station_changes_after_first_validation_cannot_publish_pending(scene,monkeypatch):
    s=scene;original=s.p._publish_initial_feedback_gap
    def changed(source,error):
        s.params['/gui/joint_direct_mode']=True
        return original(source,error)
    monkeypatch.setattr(s.p,'_publish_initial_feedback_gap',changed)
    row=frame(s)
    assert not row['object'].detected and s.p.tracker.lost
    assert s.p._locked_source_pending is None
