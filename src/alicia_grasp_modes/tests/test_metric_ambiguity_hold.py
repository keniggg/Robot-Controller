"""Real segmentation/refinement with mocked ROS transport and measured callbacks."""
import json
import numpy as np
import pytest
from std_msgs.msg import Header, String

from test_locked_rgb_hold import locked, latest_status
from test_unknown_stream import module


def frame(n,clock,joint,*,extra=True,same_colour=False,q=None,bracket=True,source_ns=None,target_colour=20):
    source_ns=int((clock[0]-.1)*1e9) if source_ns is None else source_ns
    rgb=np.full((240,320,3),(60,90,190),np.uint8)
    rgb[106:134,148:173]=target_colour
    depth=np.full((240,320),.4,np.float32);depth[106:134,148:173]=.35
    if extra:
        rgb[138:155,151:167]=20 if same_colour else (230,210,30)
        depth[138:155,151:167]=.35
    pair=(n.bridge.cv2_to_imgmsg(rgb,'bgr8'),n.bridge.cv2_to_imgmsg(depth,'32FC1'))
    for m in pair:m.header.stamp=module.rospy.Time(source_ns//10**9,source_ns%10**9);m.header.frame_id=n.camera_frame
    if bracket:joint(source_ns*1e-9-.05,q);joint(source_ns*1e-9+.05,q)
    else:n._rgb_interest_feedback.clear()
    n.latest=pair;n.process(None)
    return pair


def assert_false_empty(n):
    assert not n.pubs['object'].publish.call_args[0][0].detected
    assert not n.pubs['object_detected'].publish.call_args[0][0].data
    assert not np.any(n.bridge.imgmsg_to_cv2(n.pubs['object_mask'].publish.call_args[0][0]))


def test_transient_multi_metric_holds_false_and_recovers_only_new_unique_frame(monkeypatch):
    n,clock,_,joint,_=locked(monkeypatch)
    old_reference=n._locked_rgb_hold_reference;anchor,last=n.tracker.anchor,n.tracker.last
    old_stamp=n.last_good_source_stamp_ns
    frame(n,clock,joint)
    status=latest_status(n)
    assert status['detail']=='measured_geometry_temporarily_ambiguous_rgb_identity_held',status
    assert_false_empty(n)
    assert n.tracker.anchor is anchor and n.tracker.last is last and not n.tracker.lost
    assert n._locked_rgb_hold_reference is old_reference and n.last_good_source_stamp_ns==old_stamp
    assert n._metric_ambiguity_pending is not None and n.first_loss is None
    assert status['metric_ambiguity_hold']['candidate_count']==2
    clock[0]+=.2;frame(n,clock,joint,extra=False)
    assert n.pubs['object'].publish.call_args[0][0].detected
    assert n.tracker.anchor is anchor and n.tracker.last is not last
    assert n._metric_ambiguity_pending is None and not n.tracker.lost
    assert n._locked_rgb_hold_reference is not old_reference
    assert latest_status(n)['metric_ambiguity_hold']['state']=='recovered_unique_measured_frame'


@pytest.mark.parametrize('change',['same_colour','pose','pose_return','epoch','camera','handeye',
    'config','bracket','support','budget','slow_rgb','bad_refine','already_lost','no_reference'])
def test_unqualified_ambiguity_does_not_hold_or_change_live_tracker(monkeypatch,change):
    n,clock,_,joint,params=locked(monkeypatch);last=n.tracker.last
    kwargs={}
    if change=='same_colour':kwargs['same_colour']=True
    elif change=='pose':kwargs['q']=[.005,0.,0.,0.,0.,0.]
    elif change=='pose_return':joint(clock[0]-.18,[.005,0.,0.,0.,0.,0.]);joint(clock[0]-.16,[0.]*6)
    elif change=='epoch':n._rgb_interest_epoch_cb(Header(stamp=module.rospy.Time(2),frame_id='sdk_reference_epoch'))
    elif change=='camera':params['/camera']['fx']+=1
    elif change=='handeye':params['/handeye']['mount']='changed'
    elif change=='config':n.config.support_threshold_m=.0019
    elif change=='bracket':kwargs['bracket']=False
    elif change in ('support','budget'):
        original=module.segment_with_support_fallback
        def change_result(*args,**kwargs):
            result=original(*args,**kwargs)
            if change=='support':result.metrics['support_reference_max_distance_m']=.00701
            else:result.candidates=result.candidates*9
            return result
        monkeypatch.setattr(module,'segment_with_support_fallback',change_result)
    elif change in ('slow_rgb','bad_refine'):
        original=module.refine_candidate
        def changed(*args,**kwargs):
            if change=='bad_refine':raise ValueError('missing_complete_rgb')
            result=original(*args,**kwargs);clock[0]+=.81;return result
        monkeypatch.setattr(module,'refine_candidate',changed)
    elif change=='already_lost':n.tracker.lost=True
    elif change=='no_reference':n._locked_rgb_hold_reference=None
    frame(n,clock,joint,**kwargs)
    assert_false_empty(n)
    assert n.tracker.lost and n.tracker.last is last
    assert n._metric_ambiguity_pending is None


@pytest.mark.parametrize('stage',['hold','recovery'])
@pytest.mark.parametrize('change',['pose_return','epoch','context','expire'])
def test_changes_during_refinement_prevent_hold_or_recovery_commit(monkeypatch,stage,change):
    n,clock,_,joint,params=locked(monkeypatch)
    if stage=='recovery':frame(n,clock,joint);clock[0]+=.2
    last=n.tracker.last;good=n.last_good_source_stamp_ns;original=module.refine_candidate
    def changed(*args,**kwargs):
        result=original(*args,**kwargs)
        if change=='pose_return':joint(clock[0]-.03,[.005,0.,0.,0.,0.,0.]);joint(clock[0]-.01,[0.]*6)
        elif change=='epoch':n._rgb_interest_epoch_cb(Header(stamp=module.rospy.Time(2),frame_id='sdk_reference_epoch'))
        elif change=='context':params['/handeye']['mount']='changed'
        else:clock[0]+=.81
        return result
    monkeypatch.setattr(module,'refine_candidate',changed)
    frame(n,clock,joint,extra=stage=='hold')
    assert_false_empty(n)
    assert n.tracker.lost and n.tracker.last is last and n.last_good_source_stamp_ns==good


@pytest.mark.parametrize('change',['expired','pose_return','context','epoch'])
def test_no_new_image_retires_pending_when_original_reference_fails(monkeypatch,change):
    n,clock,_,joint,params=locked(monkeypatch);frame(n,clock,joint)
    assert n._metric_ambiguity_pending is not None
    if change=='expired':clock[0]+=.6
    elif change=='pose_return':joint(clock[0]-.02,[.005,0.,0.,0.,0.,0.]);joint(clock[0]-.01,[0.]*6)
    elif change=='context':params['/handeye']['mount']='changed'
    else:n._rgb_interest_epoch_cb(Header(stamp=module.rospy.Time(2),frame_id='sdk_reference_epoch'))
    n.last_receive_wall=clock[0];n.process(None)
    assert n.tracker.lost and n._metric_ambiguity_pending is None
    assert_false_empty(n)


def test_repeated_ambiguity_cannot_slide_deadline_or_revive_loss(monkeypatch):
    n,clock,_,joint,_=locked(monkeypatch);good=n.last_good_source_stamp_ns
    frame(n,clock,joint);clock[0]+=.2;frame(n,clock,joint)
    assert not n.tracker.lost and n.last_good_source_stamp_ns==good
    clock[0]+=.4;frame(n,clock,joint)
    assert n.tracker.lost
    clock[0]+=.1;frame(n,clock,joint,extra=False)
    assert_false_empty(n);assert n.tracker.lost and n.last_good_source_stamp_ns==good


def test_new_selection_clears_the_separate_ambiguity_state(monkeypatch):
    n,clock,_,joint,_=locked(monkeypatch);frame(n,clock,joint)
    n.selection(String(data=json.dumps(dict(mode='unknown',generation=n.generation+1,stamp_ns=int(clock[0]*1e9)))))
    assert n._metric_ambiguity_pending is None and n._metric_ambiguity_status is None
    assert n.tracker.anchor is None


def test_unique_recovery_of_different_appearance_is_rejected(monkeypatch):
    n,clock,_,joint,_=locked(monkeypatch);frame(n,clock,joint)
    old=n.tracker.last;good=n.last_good_source_stamp_ns
    clock[0]+=.2;frame(n,clock,joint,extra=False,target_colour=(230,210,30))
    assert_false_empty(n)
    assert n.tracker.lost and n.tracker.last is old and n.last_good_source_stamp_ns==good


@pytest.mark.parametrize('offset',[0,-1])
def test_pending_requires_strictly_new_source_even_with_unique_geometry(monkeypatch,offset):
    n,clock,_,joint,_=locked(monkeypatch);pair=frame(n,clock,joint)
    old=n.tracker.last;clock[0]+=.2
    frame(n,clock,joint,extra=False,source_ns=pair[0].header.stamp.to_nsec()+offset)
    assert_false_empty(n)
    assert n.tracker.lost and n.tracker.last is old


@pytest.mark.parametrize('change',['context','pose_return','epoch','expire','missing_new_cache'])
def test_changes_after_new_reference_preparation_cannot_commit_recovery(monkeypatch,change):
    n,clock,_,joint,params=locked(monkeypatch);frame(n,clock,joint)
    old=n.tracker.last;good=n.last_good_source_stamp_ns;clock[0]+=.2
    prepare=n._prepare_locked_rgb_hold_reference
    def changed(*args,**kwargs):
        reference=prepare(*args,**kwargs)
        if change=='context':params['/handeye']['mount']='changed'
        elif change=='pose_return':joint(clock[0]-.03,[.005,0.,0.,0.,0.,0.]);joint(clock[0]-.01,[0.]*6)
        elif change=='epoch':n._rgb_interest_epoch_cb(Header(stamp=module.rospy.Time(2),frame_id='sdk_reference_epoch'))
        elif change=='expire':clock[0]+=.81
        else:return None
        return reference
    monkeypatch.setattr(n,'_prepare_locked_rgb_hold_reference',changed)
    frame(n,clock,joint,extra=False)
    assert_false_empty(n)
    assert n.tracker.lost and n.tracker.last is old and n.last_good_source_stamp_ns==good


@pytest.mark.parametrize('key',['support_current_audit_quadrants','support_current_audit_inlier_ratio',
    'support_current_audit_residual_m','support_reference_max_distance_m','support_reference_angle_deg'])
def test_nonfinite_support_evidence_never_allows_hold(monkeypatch,key):
    n,clock,_,joint,_=locked(monkeypatch);original=module.segment_with_support_fallback
    def invalid(*args,**kwargs):
        result=original(*args,**kwargs);result.metrics[key]=float('nan');return result
    monkeypatch.setattr(module,'segment_with_support_fallback',invalid)
    frame(n,clock,joint)
    assert n.tracker.lost and n._metric_ambiguity_pending is None
    assert_false_empty(n)


@pytest.mark.parametrize('which',['anchor','last'])
def test_reference_mutation_during_refinement_retires_identity(monkeypatch,which):
    n,clock,_,joint,_=locked(monkeypatch);original=module.refine_candidate
    def invalid(*args,**kwargs):
        result=original(*args,**kwargs)
        getattr(n.tracker,which).position_base[0]+=.001
        return result
    monkeypatch.setattr(module,'refine_candidate',invalid)
    frame(n,clock,joint)
    assert n.tracker.lost and n._metric_ambiguity_pending is None
    assert_false_empty(n)
