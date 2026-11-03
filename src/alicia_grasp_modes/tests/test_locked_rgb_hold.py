"""Same-station identity survives only a short, positively observed depth gap."""
from copy import deepcopy
from dataclasses import replace
import json

import numpy as np
import pytest
from std_msgs.msg import Header, String

from test_unknown_stream import module, rgb_interest_node


def locked(monkeypatch, *, initial_station=False):
    n,clock,feed,joint,params=rgb_interest_node(monkeypatch)
    for _ in range(3):
        feed(target_depth=.35)
        clock[0]+=.2
    assert n.tracker.anchor is not None and not n.tracker.lost
    assert n.pubs['object'].publish.call_args[0][0].detected
    assert n._locked_rgb_hold_reference is not None
    if not initial_station:
        # This fixture exercises the original RGB hold/pending contract.
        # The initial-station opt-in has its own process-level test suite.
        n._locked_rgb_hold_reference.pop('initial_station_current_support', None)
        # Opt out its published provenance too, so consumer fixtures represent
        # the same legacy producer rather than a later template disappearance.
        status_message = n.pubs['detector_status'].publish.call_args[0][0]
        status = json.loads(status_message.data)
        status.pop('initial_station_identity_template', None)
        status.pop('initial_station_measurement', None)
        status_message.data = json.dumps(status)
    return n,clock,feed,joint,params


def custom_frame(n,clock,joint,*,rgb_missing=False,shift=0,target_depth=0.,q=None,bracket=True,old_stamp=False,depth_image=None):
    stamp=clock[0]-(1. if old_stamp else .1)
    rgb=np.full((240,320,3),(60,90,190),np.uint8)
    if not rgb_missing:rgb[106:134,148+shift:173+shift]=20
    depth=np.full((240,320),.4,np.float32)
    depth[106:134,148+shift:173+shift]=target_depth
    if depth_image is not None:depth=np.asarray(depth_image,dtype=np.float32).copy()
    pair=(n.bridge.cv2_to_imgmsg(rgb,encoding='bgr8'),n.bridge.cv2_to_imgmsg(depth,encoding='32FC1'))
    for msg in pair:
        msg.header.stamp=module.rospy.Time.from_sec(stamp);msg.header.frame_id=n.camera_frame
    if bracket:
        joint(stamp-.05,q);joint(stamp+.05,q)
    else:n._rgb_interest_feedback.clear()
    n.latest=pair;n.process(None)
    return pair


def latest_status(n):
    return json.loads(n.pubs['detector_status'].publish.call_args[0][0].data)


def fragment_depth():
    """An actual measured subcomponent inside the unchanged full RGB outline."""
    depth=np.zeros((28,25),np.float32)
    depth[7:20,7:18]=.35
    return depth


def multiple_fragment_depth():
    depth=np.zeros((28,25),np.float32)
    depth[2:12,7:18]=.35
    depth[16:26,7:18]=.35
    return depth


def gap_depth(kind):
    return {'zero':lambda:0.,'fragment':fragment_depth,'fragments':multiple_fragment_depth}[kind]()


def fragment_evidence(monkeypatch):
    n,clock,feed,joint,params=locked(monkeypatch)
    captured=[]
    original=module.segment_with_support_fallback
    def capture(*args,**kwargs):
        result=original(*args,**kwargs)
        captured.append(result)
        return result
    monkeypatch.setattr(module,'segment_with_support_fallback',capture)
    source=custom_frame(n,clock,joint,target_depth=fragment_depth())
    assert len(captured)==1 and len(captured[0].candidates)==1
    assert latest_status(n)['rgb_identity_hold']['state']=='holding_identity_only'
    return n,clock,feed,joint,params,source,captured[0]


def test_zero_candidate_gap_holds_identity_only_with_current_real_rgb_evidence(monkeypatch):
    n,clock,feed,_,_=locked(monkeypatch)
    anchor,last=n.tracker.anchor,n.tracker.last
    ref=n._locked_rgb_hold_reference;stamp=n.last_good_source_stamp_ns
    feed(target_depth=0.)
    status=latest_status(n)
    assert status['detail']=='measured_depth_temporarily_unavailable_rgb_identity_held'
    assert status['rgb_identity_hold']['mask_iou']>=.85
    assert status['rgb_identity_hold']['state']=='holding_identity_only'
    assert not status['rgb_identity_hold']['metric_geometry_authority']
    assert not status['rgb_identity_hold']['motion_authorized']
    assert not status['observation_interest']['observation_interest_ready']
    assert not n.pubs['object'].publish.call_args[0][0].detected
    assert not n.pubs['object_detected'].publish.call_args[0][0].data
    assert not n.pubs['raw_object_detected'].publish.call_args[0][0].data
    assert not np.any(n.bridge.imgmsg_to_cv2(n.pubs['object_mask'].publish.call_args[0][0]))
    assert n.tracker.anchor is anchor and n.tracker.last is last and not n.tracker.lost
    assert n._locked_rgb_hold_reference is ref and n.last_good_source_stamp_ns==stamp
    assert n.first_loss is None and n._rgb_acquisition_active is None
    assert not ref['bgr'].flags.writeable and not ref['mask'].flags.writeable and not ref['histogram'].flags.writeable


def test_valid_current_geometry_still_passes_original_path_and_refreshes_real_reference(monkeypatch):
    n,clock,feed,_,_=locked(monkeypatch)
    ref=n._locked_rgb_hold_reference;anchor=n.tracker.anchor
    feed(target_depth=0.)
    clock[0]+=.2;feed(target_depth=.35)
    assert n.pubs['object'].publish.call_args[0][0].detected
    assert n.tracker.anchor is anchor and not n.tracker.lost
    assert n._locked_rgb_hold_reference['source_stamp_ns']>ref['source_stamp_ns']
    assert latest_status(n)['detail']=='tracked'


def test_rgb_only_frames_cannot_slide_the_metric_window_or_revive_loss(monkeypatch):
    n,clock,feed,_,_=locked(monkeypatch);last_good=n.last_good_source_stamp_ns
    feed(target_depth=0.)
    clock[0]+=.2;feed(target_depth=0.)
    assert not n.tracker.lost and n.last_good_source_stamp_ns==last_good
    clock[0]+=.5;feed(target_depth=0.)
    assert n.tracker.lost and n._locked_rgb_hold_reference is None
    clock[0]+=.1;feed(target_depth=.35)
    assert n.tracker.lost and not n.pubs['object'].publish.call_args[0][0].detected
    assert n.last_good_source_stamp_ns==last_good


@pytest.mark.parametrize('gap_kind',['zero','fragment','fragments'])
@pytest.mark.parametrize('change',['missing_rgb','object_shift','pose','pose_out_and_back','epoch',
    'camera','handeye','selection_context','config','bracket','stale','support_audit_missing',
    'support_shift','support_tilt','rgb_ambiguous','rgb_budget','anchor_changed','last_changed'])
def test_bad_identity_or_source_evidence_keeps_original_loss_behavior(monkeypatch,change,gap_kind):
    n,clock,feed,joint,params=locked(monkeypatch)
    kwargs=dict(target_depth=gap_depth(gap_kind))
    if change=='missing_rgb':kwargs['rgb_missing']=True
    elif change=='object_shift':kwargs['shift']=15
    elif change=='pose':kwargs['q']=[.005,0.,0.,0.,0.,0.]
    elif change=='pose_out_and_back':
        joint(clock[0]-.15,[.005,0.,0.,0.,0.,0.]);joint(clock[0]-.13,[0.]*6)
    elif change=='epoch':n._rgb_interest_epoch_cb(Header(stamp=module.rospy.Time(2),frame_id='sdk_reference_epoch'))
    elif change=='camera':params['/camera']['fx']+=1
    elif change=='handeye':params['/handeye']['mount']='changed'
    elif change=='selection_context':params['/grasp_mode/selection']['generation']+=1
    elif change=='config':n.config.support_threshold_m=.0019
    elif change=='bracket':kwargs['bracket']=False
    elif change=='stale':kwargs['old_stamp']=True
    elif change.startswith('support_'):
        original=module.segment_with_support_fallback
        def broken(*a,**kw):
            result=original(*a,**kw)
            if change=='support_audit_missing':result.metrics['support_current_audit_available']=False
            elif change=='support_shift':result.plane_base[3]+=.0071
            else:result.plane_base[:3]=[0.,np.sin(.08),np.cos(.08)]
            return result
        monkeypatch.setattr(module,'segment_with_support_fallback',broken)
    elif change.startswith('rgb_'):
        original=module.propose_rgb_observation_regions
        def broken(*a,**kw):
            result=original(*a,**kw)
            if change=='rgb_budget':return replace(result,reason='rgb_candidate_budget_exceeded')
            region=result.candidates[0]
            return replace(result,candidates=(region,region),total_candidate_count=2)
        monkeypatch.setattr(module,'propose_rgb_observation_regions',broken)
    elif change=='anchor_changed':n.tracker.anchor.position_base[0]+=.001
    elif change=='last_changed':n.tracker.last=replace(n.tracker.last,position_base=n.tracker.last.position_base+[.001,0.,0.])
    custom_frame(n,clock,joint,**kwargs)
    assert n.tracker.lost
    assert n._locked_rgb_hold_reference is None
    assert not n.pubs['object'].publish.call_args[0][0].detected
    assert n.first_loss is not None


def test_new_selection_clears_old_same_station_cache(monkeypatch):
    n,clock,_,_,_=locked(monkeypatch)
    n.selection(String(data=json.dumps(dict(mode='unknown',generation=n.generation+1,stamp_ns=int(clock[0]*1e9)))))
    assert n._locked_rgb_hold_reference is None and n.tracker.anchor is None


def test_fragment_candidate_mismatch_is_not_zero_candidate_hold(monkeypatch):
    n,clock,_,joint,_=locked(monkeypatch)
    original=module.segment_with_support_fallback
    def fragmented(*args,**kwargs):
        result=original(*args,**kwargs)
        small=replace(n.tracker.anchor,extent_m=n.tracker.anchor.extent_m*.1)
        result.candidates=[small]
        return result
    monkeypatch.setattr(module,'segment_with_support_fallback',fragmented)
    custom_frame(n,clock,joint)
    assert n.tracker.lost and not n.pubs['object'].publish.call_args[0][0].detected


@pytest.mark.parametrize('gap_kind',['zero','fragment','fragments'])
def test_source_expiring_during_rgb_matching_cannot_hold(monkeypatch,gap_kind):
    n,clock,_,joint,_=locked(monkeypatch);original=module.propose_rgb_observation_regions
    def slow(*args,**kwargs):
        result=original(*args,**kwargs);clock[0]+=1.
        return result
    monkeypatch.setattr(module,'propose_rgb_observation_regions',slow)
    custom_frame(n,clock,joint,target_depth=gap_depth(gap_kind))
    assert n.tracker.lost and not n.pubs['object'].publish.call_args[0][0].detected


@pytest.mark.parametrize('gap_kind',['zero','fragment','fragments'])
def test_source_expiring_during_final_context_validation_cannot_hold(monkeypatch,gap_kind):
    n,clock,_,joint,_=locked(monkeypatch)
    original=n._rgb_interest_source_context
    calls=[]
    def slow_final_context(*args,**kwargs):
        context=original(*args,**kwargs)
        calls.append(context)
        if len(calls)==2:
            clock[0]+=1.
        return context
    monkeypatch.setattr(n,'_rgb_interest_source_context',slow_final_context)
    custom_frame(n,clock,joint,target_depth=gap_depth(gap_kind))
    assert len(calls)==2
    assert n.tracker.lost and n._locked_rgb_hold_reference is None
    assert latest_status(n)['rgb_identity_hold']['state']=='rejected'
    assert not n.pubs['object'].publish.call_args[0][0].detected
    assert not np.any(n.bridge.imgmsg_to_cv2(n.pubs['object_mask'].publish.call_args[0][0]))


@pytest.mark.parametrize('change',['pose_out_and_back','epoch','context'])
@pytest.mark.parametrize('gap_kind',['zero','fragment','fragments'])
def test_change_during_rgb_work_is_revalidated_before_hold_publication(monkeypatch,change,gap_kind):
    n,clock,_,joint,params=locked(monkeypatch);original=module.propose_rgb_observation_regions
    def changed(*args,**kwargs):
        result=original(*args,**kwargs)
        if change=='pose_out_and_back':
            joint(clock[0]-.03,[.005,0.,0.,0.,0.,0.]);joint(clock[0]-.01,[0.]*6)
        elif change=='epoch':n._rgb_interest_epoch_cb(Header(stamp=module.rospy.Time(2),frame_id='sdk_reference_epoch'))
        else:params['/handeye']['changed_during_rgb']=True
        return result
    monkeypatch.setattr(module,'propose_rgb_observation_regions',changed)
    custom_frame(n,clock,joint,target_depth=gap_depth(gap_kind))
    assert n.tracker.lost and not n.pubs['object'].publish.call_args[0][0].detected


@pytest.mark.parametrize('gap_kind',['zero','fragment','fragments'])
def test_already_lost_tracker_cannot_reuse_even_an_intact_previous_reference(monkeypatch,gap_kind):
    n,clock,_,joint,_=locked(monkeypatch)
    n.tracker.lost=True
    custom_frame(n,clock,joint,target_depth=gap_depth(gap_kind))
    assert n.tracker.lost and n._locked_rgb_hold_reference is None
    assert not n.pubs['object'].publish.call_args[0][0].detected


def test_measured_fragment_preserves_identity_without_metric_publication_or_refresh(monkeypatch):
    n,clock,_,joint,_=locked(monkeypatch)
    anchor,last=n.tracker.anchor,n.tracker.last
    reference=n._locked_rgb_hold_reference
    stamp=n.last_good_source_stamp_ns
    original_choose=n.tracker.choose
    from unittest.mock import Mock
    n.tracker.choose=Mock(side_effect=original_choose)
    before_pose_counts={key:n.pubs[key].publish.call_count for key in
                        ('object_pose_camera','object_pose_base')}
    source=custom_frame(n,clock,joint,target_depth=fragment_depth())
    status=latest_status(n)
    hold=status['rgb_identity_hold'];evidence=hold['fragment_evidence']
    assert hold['state']=='holding_identity_only'
    assert hold['depth_gap_kind']=='unique_contained_shrinking_fragment'
    assert evidence['measured_pixel_count']==143
    assert min(evidence['extent_ratios_xy'])<.5
    assert max(evidence['extent_ratios_xy'])<=1.
    assert evidence['last_measured_rgb_containment_fraction']==1.
    assert evidence['current_rgb_containment_fraction']==1.
    assert hold['mask_iou']>=.85 and hold['maximum_hold_seconds']==.8
    assert not hold['reference_refreshed']
    assert not hold['metric_geometry_authority'] and not hold['motion_authorized']
    assert not status['observation_interest']['observation_interest_ready']
    assert n.tracker.anchor is anchor and n.tracker.last is last and not n.tracker.lost
    assert n._locked_rgb_hold_reference is reference and n.last_good_source_stamp_ns==stamp
    assert n.first_loss is None and n._rgb_acquisition_active is None
    n.tracker.choose.assert_not_called()
    for key,count in before_pose_counts.items():
        assert n.pubs[key].publish.call_count==count
    obj=n.pubs['object'].publish.call_args[0][0]
    mask=n.pubs['object_mask'].publish.call_args[0][0]
    assert not obj.detected and obj.header.stamp==source[0].header.stamp
    assert mask.header.stamp==source[0].header.stamp
    assert not np.any(n.bridge.imgmsg_to_cv2(mask))
    assert not n.pubs['object_detected'].publish.call_args[0][0].data
    assert not n.pubs['raw_object_detected'].publish.call_args[0][0].data


@pytest.mark.parametrize('reported_ratios',[None,(.48445,.51236)])
def test_fragment_does_not_renew_window_and_true_geometry_can_refresh_before_expiry(monkeypatch,reported_ratios):
    n,clock,feed,joint,_=locked(monkeypatch)
    stamp=n.last_good_source_stamp_ns;reference=n._locked_rgb_hold_reference
    if reported_ratios is not None:
        # Exercise the reported failure ratios on synthetic native RGB-D.
        # The historical loss source was not recorded; this is not its replay.
        original=module.segment_with_support_fallback
        def reported(*args,**kwargs):
            result=original(*args,**kwargs)
            if len(result.candidates)==1:
                candidate=result.candidates[0]
                if min(candidate.extent_m[:2]/n.tracker.anchor.extent_m[:2])<.5:
                    extent=candidate.extent_m.copy()
                    extent[:2]=n.tracker.anchor.extent_m[:2]*reported_ratios
                    result.candidates=[replace(candidate,extent_m=extent)]
            return result
        monkeypatch.setattr(module,'segment_with_support_fallback',reported)
    custom_frame(n,clock,joint,target_depth=fragment_depth())
    assert latest_status(n)['rgb_identity_hold']['state']=='holding_identity_only'
    assert not n.pubs['object'].publish.call_args[0][0].detected
    if reported_ratios is not None:
        assert latest_status(n)['rgb_identity_hold']['fragment_evidence']['extent_ratios_xy']==pytest.approx(reported_ratios)
    clock[0]+=.2
    custom_frame(n,clock,joint,target_depth=fragment_depth())
    assert n.last_good_source_stamp_ns==stamp and n._locked_rgb_hold_reference is reference
    clock[0]+=.1;feed(target_depth=.35)
    assert latest_status(n)['detail']=='tracked'
    assert n.pubs['object'].publish.call_args[0][0].detected
    assert n.last_good_source_stamp_ns>stamp
    assert n._locked_rgb_hold_reference['source_stamp_ns']==n.last_good_source_stamp_ns


@pytest.mark.parametrize('gap_kind',['fragment','fragments'])
def test_fragment_window_expires_from_last_true_frame_and_cannot_revive_loss(monkeypatch,gap_kind):
    n,clock,_,joint,_=locked(monkeypatch)
    reference=n._locked_rgb_hold_reference;stamp=n.last_good_source_stamp_ns
    for advance in (0.,.2,.2):
        clock[0]+=advance
        custom_frame(n,clock,joint,target_depth=gap_depth(gap_kind))
        assert not n.tracker.lost and n._locked_rgb_hold_reference is reference
        assert n.last_good_source_stamp_ns==stamp
    clock[0]+=.100001
    custom_frame(n,clock,joint,target_depth=gap_depth(gap_kind))
    assert n.tracker.lost and n._locked_rgb_hold_reference is None
    # Even an intact cache cannot undo an already latched loss.
    n._locked_rgb_hold_reference=reference
    clock[0]+=.01
    custom_frame(n,clock,joint,target_depth=gap_depth(gap_kind))
    assert n.tracker.lost and n._locked_rgb_hold_reference is None
    assert not n.pubs['object'].publish.call_args[0][0].detected
    assert n.last_good_source_stamp_ns==stamp


@pytest.mark.parametrize('change',['multiple','growth','outside_last_rgb','outside_current_rgb',
                                    'anchor_distance'])
def test_fragment_rejection_does_not_bypass_original_tracking_loss(monkeypatch,change):
    n,clock,_,joint,_=locked(monkeypatch)
    original=module.segment_with_support_fallback
    def changed(*args,**kwargs):
        result=original(*args,**kwargs)
        assert len(result.candidates)==1
        candidate=result.candidates[0]
        if change=='multiple':result.candidates.append(deepcopy(candidate))
        elif change=='growth':
            extent=candidate.extent_m.copy();extent[1]=n.tracker.anchor.extent_m[1]*1.001
            result.candidates=[replace(candidate,extent_m=extent)]
        elif change=='outside_last_rgb':
            mask=candidate.mask.copy();mask[113,165:175]=255
            result.candidates=[replace(candidate,mask=mask,bbox=module.cv2.boundingRect(mask),
                metrics=dict(candidate.metrics,valid_points=int(np.count_nonzero(mask))))]
        elif change=='anchor_distance':
            result.candidates=[replace(candidate,position_base=candidate.position_base+[.03,0.,0.])]
        return result
    monkeypatch.setattr(module,'segment_with_support_fallback',changed)
    if change=='outside_current_rgb':
        propose=module.propose_rgb_observation_regions
        def hole(*args,**kwargs):
            result=propose(*args,**kwargs)
            assert len(result.candidates)==1
            region=result.candidates[0];mask=region.mask.copy();mask[115,158]=0
            return replace(result,candidates=(replace(region,mask=mask),))
        monkeypatch.setattr(module,'propose_rgb_observation_regions',hole)
    custom_frame(n,clock,joint,target_depth=fragment_depth())
    assert n.tracker.lost and n.first_loss is not None
    assert n._locked_rgb_hold_reference is None
    assert not n.pubs['object'].publish.call_args[0][0].detected
    assert not np.any(n.bridge.imgmsg_to_cv2(n.pubs['object_mask'].publish.call_args[0][0]))


@pytest.mark.parametrize('field,value',[
    ('candidate',None),('candidate',{}),('candidate',[]),
    ('extent_m',[.01,.01]),('extent_m',[.01,.01,float('nan')]),
    ('extent_m',['.01','.01','0']),('extent_m',[True,True,False]),
    ('extent_m',[0.,.01,0.]),('extent_m',[-.01,.01,0.]),('extent_m',[.01,.01,-.001]),
    ('position_base',[0.,float('nan'),0.]),('position_base',[[0.,0.,0.]]),
    ('position_camera',[0.,0.]),('center_uv',[160.,float('inf')]),
    ('depth_m',float('nan')),('depth_m',[.35]),('depth_m','.35'),('depth_m',0.),
    ('quality',float('inf')),('quality',True),('quality',-1.),('quality',1.01),
    ('mask',None),('mask',np.zeros((240,320),np.uint8)),
    ('mask',np.zeros((240,320,1),np.uint8)),('mask',np.full((240,320),float('nan'))),
    ('mask',np.full((240,320),2,np.uint8)),('mask',np.full((240,320),'1')),
    ('bbox',(155,113,11)),('bbox',(155,113,11,float('nan'))),('bbox',(0,0,1,1)),
    ('support_footprint_base',None),('support_footprint_base',np.zeros((2,3))),
    ('support_footprint_base',np.full((3,3),float('nan'))),
    ('metrics',None),('metrics',{}),('metrics',{'valid_points':True}),
    ('metrics',{'valid_points':143.}),('metrics',{'valid_points':142})])
def test_malformed_fragment_never_supplies_identity_evidence(monkeypatch,field,value):
    n,_,_,_,params,source,result=fragment_evidence(monkeypatch)
    candidate=value if field=='candidate' else replace(result.candidates[0],**{field:value})
    with pytest.raises((ValueError,TypeError)):
        n._locked_rgb_shrinking_fragment(candidate,n._locked_rgb_hold_reference,source,params['/camera'])


@pytest.mark.parametrize('bad_depth',[0.,float('nan'),float('inf'),.029,2.001])
def test_every_fragment_pixel_requires_valid_native_depth(monkeypatch,bad_depth):
    n,_,_,_,params,source,result=fragment_evidence(monkeypatch)
    depth=n.bridge.imgmsg_to_cv2(source[1]).copy();depth[115,158]=bad_depth
    message=n.bridge.cv2_to_imgmsg(depth,encoding='32FC1');message.header=source[1].header
    with pytest.raises(ValueError,match='has_unmeasured_pixels'):
        n._locked_rgb_shrinking_fragment(result.candidates[0],n._locked_rgb_hold_reference,
                                        (source[0],message),params['/camera'])


@pytest.mark.parametrize('encoding',['16UC1','mono16'])
def test_native_integer_depth_uses_runtime_scale(monkeypatch,encoding):
    n,_,_,_,params,source,result=fragment_evidence(monkeypatch)
    scale=params['/camera']['depth_scale']
    depth=np.rint(n.bridge.imgmsg_to_cv2(source[1])/scale).astype(np.uint16)
    message=n.bridge.cv2_to_imgmsg(depth,encoding=encoding);message.header=source[1].header
    fragment=n._locked_rgb_shrinking_fragment(result.candidates[0],n._locked_rgb_hold_reference,
                                             (source[0],message),params['/camera'])
    assert fragment['evidence']['measured_pixel_count']==143
    with pytest.raises(ValueError,match='depth_scale_invalid'):
        n._locked_rgb_shrinking_fragment(result.candidates[0],n._locked_rgb_hold_reference,
            (source[0],message),dict(params['/camera'],depth_scale=float('nan')))


@pytest.mark.parametrize('change',['shape','encoding'])
def test_fragment_rejects_incompatible_native_depth(monkeypatch,change):
    n,_,_,_,params,source,result=fragment_evidence(monkeypatch)
    depth=(np.zeros((2,2),np.float32) if change=='shape' else
           np.full((240,320),35,np.uint8))
    message=n.bridge.cv2_to_imgmsg(depth,encoding='32FC1' if change=='shape' else 'mono8')
    message.header=source[1].header
    with pytest.raises(ValueError,match='depth_'+('shape' if change=='shape' else 'encoding')+'_invalid'):
        n._locked_rgb_shrinking_fragment(result.candidates[0],n._locked_rgb_hold_reference,
                                         (source[0],message),params['/camera'])


@pytest.mark.parametrize('ratios,expected',[
    ((.48445,.51236),True),((.5,.49999),True),((.49999,.5),True),
    ((.5,.5),False),((.9,1.),False)])
def test_fragment_requires_at_least_one_axis_below_original_half_ratio(monkeypatch,ratios,expected):
    n,_,_,_,params,source,result=fragment_evidence(monkeypatch)
    candidate=result.candidates[0]
    extent=candidate.extent_m.copy();extent[:2]=n.tracker.anchor.extent_m[:2]*ratios
    candidate=replace(candidate,extent_m=extent)
    fragment=n._locked_rgb_shrinking_fragment(candidate,n._locked_rgb_hold_reference,source,params['/camera'])
    assert (fragment is not None)==expected


def test_fragment_must_remain_within_last_position_as_well_as_anchor(monkeypatch):
    n,_,_,_,params,source,result=fragment_evidence(monkeypatch)
    n.tracker.last=replace(n.tracker.last,position_base=n.tracker.last.position_base+[.03,0.,0.])
    with pytest.raises(ValueError,match='not_contained_contraction'):
        n._locked_rgb_shrinking_fragment(result.candidates[0],n._locked_rgb_hold_reference,source,params['/camera'])


@pytest.mark.parametrize('gap,accepted',[(.2,True),(.200002,False)])
def test_fragment_does_not_widen_feedback_bracket(monkeypatch,gap,accepted):
    n,clock,_,joint,_=locked(monkeypatch)
    source=custom_frame(n,clock,joint,target_depth=fragment_depth())
    n._rgb_interest_feedback.clear()
    stamp=source[0].header.stamp.to_sec()
    joint(stamp-gap/2);joint(stamp+gap/2)
    original=module.segment_with_support_fallback
    camera=module.rospy.get_param('/camera')
    transform=module.transform_matrix(('base_link',))
    depth=n.bridge.imgmsg_to_cv2(source[1])
    result=original(depth,[camera[key] for key in ('fx','fy','cx','cy')],transform,n.config,
        anchor=n.tracker.anchor,last=n.tracker.last,reference_plane_base=n.tracker.plane_base)
    assert n._same_station_zero_candidate_rgb_hold(source,result,camera) is accepted


@pytest.mark.parametrize('reported_ratios',[None,((.4954439,.5413911),(.4378781,.50472635))])
def test_disjoint_fragments_hold_only_identity_then_restore_on_real_complete_frame(monkeypatch,reported_ratios):
    from unittest.mock import Mock
    n,clock,feed,joint,_=locked(monkeypatch)
    anchor,last=n.tracker.anchor,n.tracker.last
    reference=n._locked_rgb_hold_reference;stamp=n.last_good_source_stamp_ns
    choose=Mock(wraps=n.tracker.choose);n.tracker.choose=choose
    pose_count=n.pubs['object_pose_base'].publish.call_count
    if reported_ratios is not None:
        original=module.segment_with_support_fallback
        def reported(*args,**kwargs):
            result=original(*args,**kwargs)
            if len(result.candidates)==2:
                # Synthetic ratio regression, not the historical loss pixels.
                for index,candidate in enumerate(result.candidates):
                    extent=candidate.extent_m.copy()
                    extent[:2]=anchor.extent_m[:2]*reported_ratios[index]
                    result.candidates[index]=replace(candidate,extent_m=extent)
            return result
        monkeypatch.setattr(module,'segment_with_support_fallback',reported)
    for advance in (0.,.2):
        clock[0]+=advance
        custom_frame(n,clock,joint,target_depth=multiple_fragment_depth())
        hold=latest_status(n)['rgb_identity_hold']
        assert hold['state']=='holding_identity_only'
        assert hold['depth_gap_kind']=='disjoint_contained_shrinking_fragments'
        assert hold['fragment_count']==2 and hold['fragment_masks_disjoint']
        assert hold['maximum_fragment_candidates']==module.HandoffPolicy().maximum_candidates
        assert hold['fragment_evidence'] is None
        assert len(hold['fragment_evidence_list'])==2
        for index,evidence in enumerate(hold['fragment_evidence_list']):
            assert evidence['measured_pixel_count']==110
            assert min(evidence['extent_ratios_xy'])<.5
            assert evidence['current_rgb_containment_fraction']==1.
            assert evidence['last_measured_rgb_containment_fraction']==1.
            if reported_ratios is not None:
                assert evidence['extent_ratios_xy']==pytest.approx(reported_ratios[index])
        assert not hold['reference_refreshed'] and not hold['metric_geometry_authority']
        assert not hold['motion_authorized']
        assert not latest_status(n)['observation_interest']['observation_interest_ready']
        assert n.tracker.anchor is anchor and n.tracker.last is last and not n.tracker.lost
        assert n.last_good_source_stamp_ns==stamp and n._locked_rgb_hold_reference is reference
        assert not n.pubs['object'].publish.call_args[0][0].detected
        assert not np.any(n.bridge.imgmsg_to_cv2(n.pubs['object_mask'].publish.call_args[0][0]))
        assert n.pubs['object_pose_base'].publish.call_count==pose_count
        choose.assert_not_called()
    clock[0]+=.1;feed(target_depth=.35)
    assert choose.call_count==1
    assert n.pubs['object'].publish.call_args[0][0].detected
    assert n.tracker.anchor is anchor and not n.tracker.lost
    assert n.last_good_source_stamp_ns>stamp


@pytest.mark.parametrize('index',[0,1])
@pytest.mark.parametrize('change',['outside_last','outside_current','growth','missing_depth',
                                  'invalid_type','bad_mask_shape','bad_mask_nan','bad_extent'])
def test_every_member_of_fragment_set_must_pass_independently(monkeypatch,index,change):
    n,clock,_,joint,_=locked(monkeypatch)
    original=module.segment_with_support_fallback
    captured=[]
    def changed(*args,**kwargs):
        result=original(*args,**kwargs);assert len(result.candidates)==2
        candidate=result.candidates[index]
        mask=candidate.mask.copy();ys,xs=np.nonzero(mask)
        captured.append((int(np.median(ys)),int(np.median(xs))))
        if change=='outside_last':
            mask[ys[0],xs.min():175]=255
            candidate=replace(candidate,mask=mask,bbox=module.cv2.boundingRect(mask),
                metrics=dict(candidate.metrics,valid_points=int(np.count_nonzero(mask))))
        elif change=='missing_depth':
            mask[ys[0],xs.min()-1]=255
            candidate=replace(candidate,mask=mask,bbox=module.cv2.boundingRect(mask),
                metrics=dict(candidate.metrics,valid_points=int(np.count_nonzero(mask))))
        elif change=='growth':
            extent=candidate.extent_m.copy();extent[1]=n.tracker.anchor.extent_m[1]*1.01
            candidate=replace(candidate,extent_m=extent)
        elif change=='invalid_type':candidate=None
        elif change=='bad_mask_shape':candidate=replace(candidate,mask=mask[:,:,None])
        elif change=='bad_mask_nan':
            mask=mask.astype(float);mask[ys[0],xs[0]]=np.nan
            candidate=replace(candidate,mask=mask)
        elif change=='bad_extent':candidate=replace(candidate,extent_m=np.full(3,np.nan))
        result.candidates[index]=candidate
        return result
    monkeypatch.setattr(module,'segment_with_support_fallback',changed)
    if change=='outside_current':
        propose=module.propose_rgb_observation_regions
        def hole(*args,**kwargs):
            result=propose(*args,**kwargs);region=result.candidates[0]
            mask=region.mask.copy();mask[captured[0]]=0
            return replace(result,candidates=(replace(region,mask=mask),))
        monkeypatch.setattr(module,'propose_rgb_observation_regions',hole)
    custom_frame(n,clock,joint,target_depth=multiple_fragment_depth())
    assert n.tracker.lost and n._locked_rgb_hold_reference is None
    assert not n.pubs['object'].publish.call_args[0][0].detected
    assert not np.any(n.bridge.imgmsg_to_cv2(n.pubs['object_mask'].publish.call_args[0][0]))
    assert latest_status(n)['rgb_identity_hold']['state']=='rejected'


@pytest.mark.parametrize('metric_index',[0,1])
def test_metric_eligible_member_keeps_original_tracking_path(monkeypatch,metric_index):
    from unittest.mock import Mock
    n,clock,_,joint,_=locked(monkeypatch)
    original=module.segment_with_support_fallback
    selected=[]
    def changed(*args,**kwargs):
        result=original(*args,**kwargs);assert len(result.candidates)==2
        candidate=replace(result.candidates[metric_index],extent_m=n.tracker.anchor.extent_m*.8)
        result.candidates[metric_index]=candidate;selected.append(candidate)
        return result
    monkeypatch.setattr(module,'segment_with_support_fallback',changed)
    choose=Mock(wraps=n.tracker.choose);n.tracker.choose=choose
    custom_frame(n,clock,joint,target_depth=multiple_fragment_depth())
    assert choose.call_count==1 and n.tracker.last is selected[0]
    assert n._locked_rgb_hold_status is None


def test_fragment_budget_rejects_before_decoding_any_candidate(monkeypatch):
    from unittest.mock import Mock
    n,clock,_,joint,_=locked(monkeypatch)
    original=module.segment_with_support_fallback
    def crowded(*args,**kwargs):
        result=original(*args,**kwargs)
        result.candidates=[deepcopy(result.candidates[0]) for _ in
                           range(module.HandoffPolicy().maximum_candidates+1)]
        return result
    monkeypatch.setattr(module,'segment_with_support_fallback',crowded)
    fragment=Mock(wraps=n._locked_rgb_shrinking_fragment)
    monkeypatch.setattr(n,'_locked_rgb_shrinking_fragment',fragment)
    custom_frame(n,clock,joint,target_depth=multiple_fragment_depth())
    fragment.assert_not_called()
    assert n.tracker.lost
    assert latest_status(n)['rgb_identity_hold']['reason']=='locked_rgb_hold_fragment_candidate_budget'


def test_one_shared_pixel_between_fragments_is_already_overlap(monkeypatch):
    n,clock,_,joint,_=locked(monkeypatch)
    original=module.segment_with_support_fallback
    def overlapping(*args,**kwargs):
        result=original(*args,**kwargs);assert len(result.candidates)==2
        first,second=result.candidates
        mask=first.mask.copy()
        mask[117:123,155]=255
        assert np.count_nonzero((mask!=0)&(second.mask!=0))==1
        result.candidates[0]=replace(first,mask=mask,bbox=module.cv2.boundingRect(mask),
            metrics=dict(first.metrics,valid_points=int(np.count_nonzero(mask))))
        return result
    monkeypatch.setattr(module,'segment_with_support_fallback',overlapping)
    depth=multiple_fragment_depth()
    # Native measured table pixels between the two components keep the
    # deliberately malformed candidate valid-depth, isolating the overlap gate.
    depth[12:16,7]=.4
    custom_frame(n,clock,joint,target_depth=depth)
    assert n.tracker.lost and not n.pubs['object'].publish.call_args[0][0].detected
    assert latest_status(n)['rgb_identity_hold']['reason']=='locked_rgb_hold_fragment_masks_overlap'


def test_disjoint_native_fragment_set_accepts_existing_sixteen_candidate_limit(monkeypatch):
    n,clock,_,joint,_=rgb_interest_node(monkeypatch)
    def feed(patch):
        stamp=clock[0]-.1
        rgb=np.full((240,320,3),(60,90,190),np.uint8);rgb[90:150,130:190]=20
        depth=np.full((240,320),.4,np.float32);depth[90:150,130:190]=patch
        source=(n.bridge.cv2_to_imgmsg(rgb,encoding='bgr8'),n.bridge.cv2_to_imgmsg(depth,encoding='32FC1'))
        for message in source:
            message.header.stamp=module.rospy.Time.from_sec(stamp)
            message.header.frame_id=n.camera_frame
        joint(stamp-.05);joint(stamp+.05)
        n.latest=source;n.process(None)
    for _ in range(3):
        feed(.35);clock[0]+=.2
    assert n.pubs['object'].publish.call_args[0][0].detected
    reference=n._locked_rgb_hold_reference;last=n.tracker.last
    patches=np.zeros((60,60),np.float32)
    for y in (8,20,32,44):
        for x in (8,20,32,44):patches[y:y+8,x:x+8]=.35
    feed(patches)
    hold=latest_status(n)['rgb_identity_hold']
    assert hold['state']=='holding_identity_only'
    assert hold['fragment_count']==module.HandoffPolicy().maximum_candidates==16
    assert [row['measured_pixel_count'] for row in hold['fragment_evidence_list']]==[64]*16
    assert n._locked_rgb_hold_reference is reference and n.tracker.last is last
    assert not n.tracker.lost and not n.pubs['object'].publish.call_args[0][0].detected


def ownership(fragment,old,current):
    node=module.UnknownTabletopPerception.__new__(module.UnknownTabletopPerception)
    return node._locked_rgb_fragment_ownership(fragment,old,current)


def square_mask(start=20,stop=30):
    mask=np.zeros((100,100),np.uint8)
    mask[start:stop,start:stop]=1
    return mask


@pytest.mark.parametrize('removed',[1,10,15])
def test_boundary_ownership_reports_real_retention_and_never_changes_masks(removed):
    fragment=square_mask();old=fragment.copy();current=fragment.copy()
    current[20,20:20+min(10,removed)]=0
    if removed>10:current[21:21+removed-10,20]=0
    snapshots=[value.copy() for value in (fragment,old,current)]
    report=ownership(fragment,old,current)
    assert report['boundary_tolerance_used']
    assert report['ownership_policy']=='bounded_rgb_depth_boundary_identity_only'
    assert report['last_measured_rgb_containment_fraction']==1.
    assert report['current_rgb_containment_fraction']==pytest.approx(1-removed/100.)
    assert report['joint_rgb_containment_fraction']==pytest.approx(1-removed/100.)
    assert report['joint_rgb_outside_pixels']==removed
    assert report['eroded_interior_pixels']==64
    assert report['eroded_interior_outside_joint_rgb_pixels']==0
    assert report['last_measured_rgb_boundary_margin_px']==0
    assert report['current_rgb_boundary_margin_px']==1
    assert report['maximum_rgb_boundary_margin_px']==1
    assert not report['candidate_mask_modified']
    for actual,before in zip((fragment,old,current),snapshots):assert np.array_equal(actual,before)


def test_independent_eighty_five_percent_masks_do_not_replace_joint_retention():
    fragment=square_mask();old=fragment.copy();current=fragment.copy()
    old[20,20:30]=0;old[21:26,20]=0
    current[29,20:30]=0;current[24:29,29]=0
    assert np.count_nonzero(old&fragment)==85
    assert np.count_nonzero(current&fragment)==85
    assert np.count_nonzero(old&current&fragment)==70
    with pytest.raises(ValueError,match='boundary_retention_insufficient'):
        ownership(fragment,old,current)


@pytest.mark.parametrize('extension',[2,10,25])
def test_thin_protrusions_must_stay_within_rgb_one_pixel_band(extension):
    old=square_mask(20,50);fragment=old.copy()
    fragment[35,50:50+extension]=1
    assert np.count_nonzero(fragment&old)/np.count_nonzero(fragment)>.85
    with pytest.raises(ValueError,match='outside_one_pixel_rgb_boundary'):
        ownership(fragment,old,old)


def test_boundary_fraction_cannot_hide_conflicting_interior_pixel():
    fragment=square_mask();old=fragment.copy();current=fragment.copy()
    current[24,24]=0
    with pytest.raises(ValueError,match='boundary_core_conflict'):
        ownership(fragment,old,current)


@pytest.mark.parametrize('shape,expected_core',[(('thin'),0),(('small'),22),(('exact'),25)])
def test_boundary_exception_requires_twenty_five_interior_pixels(shape,expected_core):
    fragment=np.zeros((100,100),np.uint8)
    if shape=='thin':fragment[20:22,20:50]=1
    elif shape=='small':fragment[20:23,20:44]=1
    else:
        fragment[20:27,20:27]=1
        fragment[20,5:20]=1
    assert np.count_nonzero(fragment)>=60
    assert np.count_nonzero(module.cv2.erode(fragment,np.ones((3,3),np.uint8)))==expected_core
    old=fragment.copy();current=fragment.copy();current[20,20]=0
    if expected_core<25:
        with pytest.raises(ValueError,match='boundary_core_too_small'):ownership(fragment,old,current)
    else:
        report=ownership(fragment,old,current)
        assert report['eroded_interior_pixels']==25 and report['boundary_tolerance_used']


def test_original_exact_containment_does_not_gain_an_interior_size_gate():
    fragment=np.zeros((100,100),np.uint8);fragment[20:22,20:50]=1
    report=ownership(fragment,fragment,fragment)
    assert report['eroded_interior_pixels']==0
    assert not report['boundary_tolerance_used']
    assert report['ownership_policy']=='exact_containment'
    assert report['joint_rgb_containment_fraction']==1.


def boundary_depth(count=1):
    depth=np.full((240,320),.4,np.float32)
    depth[106:134,148:173]=0.
    if count==1:depth[109:122,163:174]=.35
    else:
        depth[108:118,147:158]=.35
        depth[122:132,163:174]=.35
    return depth


@pytest.mark.parametrize('count',[1,2])
def test_native_boundary_fragments_hold_false_without_clipping_and_restore_true(monkeypatch,count):
    from unittest.mock import Mock
    n,clock,feed,joint,_=locked(monkeypatch)
    anchor,last=n.tracker.anchor,n.tracker.last
    reference=n._locked_rgb_hold_reference;stamp=n.last_good_source_stamp_ns
    choose=Mock(wraps=n.tracker.choose);n.tracker.choose=choose
    captured=[];original=module.segment_with_support_fallback
    def capture(*args,**kwargs):
        result=original(*args,**kwargs)
        captured.append((result,[deepcopy(candidate) for candidate in result.candidates]))
        return result
    monkeypatch.setattr(module,'segment_with_support_fallback',capture)
    custom_frame(n,clock,joint,depth_image=boundary_depth(count))
    hold=latest_status(n)['rgb_identity_hold']
    assert hold['state']=='holding_identity_only' and hold['fragment_count']==count
    assert hold['depth_gap_kind']=='rgb_boundary_consistent_shrinking_fragments'
    assert not hold['reference_refreshed'] and not hold['metric_geometry_authority']
    assert not hold['motion_authorized']
    for report in hold['fragment_evidence_list']:
        assert report['boundary_tolerance_used'] and not report['candidate_mask_modified']
        assert report['joint_rgb_containment_fraction']==pytest.approx(10/11)
        assert report['last_measured_rgb_containment_fraction']==pytest.approx(10/11)
        assert report['current_rgb_containment_fraction']==pytest.approx(10/11)
        assert report['last_measured_rgb_boundary_margin_px']==1
        assert report['current_rgb_boundary_margin_px']==1
    assert n.tracker.anchor is anchor and n.tracker.last is last and not n.tracker.lost
    assert n.last_good_source_stamp_ns==stamp and n._locked_rgb_hold_reference is reference
    choose.assert_not_called()
    assert not n.pubs['object'].publish.call_args[0][0].detected
    assert not np.any(n.bridge.imgmsg_to_cv2(n.pubs['object_mask'].publish.call_args[0][0]))
    for candidate,before in zip(captured[0][0].candidates,captured[0][1]):
        for field in ('mask','extent_m','position_base','position_camera'):
            assert np.array_equal(getattr(candidate,field),getattr(before,field))
    clock[0]+=.2;feed(target_depth=.35)
    assert n.pubs['object'].publish.call_args[0][0].detected
    assert n.tracker.anchor is anchor and n.last_good_source_stamp_ns>stamp


@pytest.mark.parametrize('change',['expired','final_context_expired','moved','ambiguous','outside_component'])
def test_boundary_ownership_never_overrides_other_identity_failures(monkeypatch,change):
    n,clock,_,joint,_=locked(monkeypatch)
    kwargs={};depth=boundary_depth(2)
    if change=='expired':clock[0]+=.6
    elif change=='moved':kwargs['q']=[.005,0.,0.,0.,0.,0.]
    elif change=='outside_component':depth[115:125,182:192]=.35
    elif change=='ambiguous':
        original=module.propose_rgb_observation_regions
        def duplicate(*args,**kwargs):
            result=original(*args,**kwargs)
            return replace(result,candidates=(result.candidates[0],result.candidates[0]))
        monkeypatch.setattr(module,'propose_rgb_observation_regions',duplicate)
    else:
        original=n._rgb_interest_source_context;calls=[]
        def delayed(*args,**kwargs):
            result=original(*args,**kwargs);calls.append(result)
            if len(calls)==2:clock[0]+=1.
            return result
        monkeypatch.setattr(n,'_rgb_interest_source_context',delayed)
    custom_frame(n,clock,joint,depth_image=depth,**kwargs)
    assert n.tracker.lost and n._locked_rgb_hold_reference is None
    assert not n.pubs['object'].publish.call_args[0][0].detected
    assert not np.any(n.bridge.imgmsg_to_cv2(n.pubs['object_mask'].publish.call_args[0][0]))


def pending_frame(n,clock,*,target_depth=None,depth_image=None,gap_ns=200043250,q=None):
    """Two real samples with the exact observed unqualified 200.04325 ms gap."""
    from sensor_msgs.msg import JointState
    source_ns=module.rospy.Time.from_sec(clock[0]-.1).to_nsec()
    before_ns=source_ns-140279338
    n._rgb_interest_feedback.clear()
    injected=[]
    def feedback(_stamp,_q=None):
        if injected:return
        injected.append(True)
        for ns in (before_ns,before_ns+gap_ns):
            message=JointState()
            message.header.stamp=module.rospy.Time(ns//10**9,ns%10**9)
            message.header.frame_id='sdk_measured'
            message.name=list(module.JOINT_NAMES)
            message.position=[0.]*6 if q is None else q
            n._rgb_interest_joint_cb(message)
    return custom_frame(n,clock,feedback,
        target_depth=fragment_depth() if target_depth is None else target_depth,depth_image=depth_image)


@pytest.mark.parametrize('kind',['zero','fragment','fragments','boundary'])
def test_unqualified_frame_does_not_mutate_identity_or_claim_positive_hold(monkeypatch,kind):
    from unittest.mock import Mock
    n,clock,_,_,_=locked(monkeypatch)
    anchor,last=n.tracker.anchor,n.tracker.last
    reference=n._locked_rgb_hold_reference;stamp=n.last_good_source_stamp_ns
    choose=Mock(wraps=n.tracker.choose);n.tracker.choose=choose
    pose_count=n.pubs['object_pose_base'].publish.call_count
    source=pending_frame(n,clock,**(dict(depth_image=boundary_depth(2)) if kind=='boundary' else
                                   dict(target_depth=gap_depth(kind))))
    status=latest_status(n);qualification=status['source_qualification']
    assert status['detail']=='source_qualification_pending'
    assert qualification['state']=='pending_source_qualification'
    assert qualification['source_stamp_ns']==source[0].header.stamp.to_nsec()
    assert qualification['reference_source_stamp_ns']==stamp
    assert qualification['feedback_rejection']['span_ns']==200043250
    assert qualification['feedback_rejection']['maximum_span_ns']==200000000
    assert not qualification['feedback_rejection']['bracket_qualified']
    for field in ('source_qualified','identity_verified','metric_geometry_authority',
                  'motion_authorized','reference_refreshed'):assert qualification[field] is False
    assert 'rgb_identity_hold' not in status
    assert not status['observation_interest']['observation_interest_ready']
    assert n.tracker.anchor is anchor and n.tracker.last is last and not n.tracker.lost
    assert n._locked_rgb_hold_reference is reference and n.last_good_source_stamp_ns==stamp
    assert n._locked_source_pending is not None and n.first_loss is None
    choose.assert_not_called()
    assert not n.pubs['object'].publish.call_args[0][0].detected
    assert not np.any(n.bridge.imgmsg_to_cv2(n.pubs['object_mask'].publish.call_args[0][0]))
    assert n.pubs['object_pose_base'].publish.call_count==pose_count


def test_pending_source_requalifies_full_metric_frame_before_original_tracking(monkeypatch):
    from unittest.mock import Mock
    n,clock,feed,_,_=locked(monkeypatch)
    anchor=n.tracker.anchor;old_stamp=n.last_good_source_stamp_ns
    pending_frame(n,clock)
    assert n._locked_source_pending is not None
    choose=Mock(wraps=n.tracker.choose);n.tracker.choose=choose
    clock[0]+=.1;feed(target_depth=.35)
    # The original TargetTracker algorithm selects on a staged tracker; the
    # live tracker is committed only after refinement and final qualification.
    choose.assert_not_called()
    assert n._locked_source_pending is None and n.tracker.anchor is anchor and not n.tracker.lost
    assert n.pubs['object'].publish.call_args[0][0].detected
    assert n.last_good_source_stamp_ns>old_stamp
    qualification=latest_status(n)['source_qualification']
    assert qualification['state']=='qualified_new_frame' and qualification['source_qualified']
    assert qualification['support_audit']['source']=='independent_current_support_audit'
    assert latest_status(n)['detail']=='tracked'
    assert n._locked_rgb_hold_reference['last_fingerprint']==module._candidate_fingerprint(n.tracker.last)


@pytest.mark.parametrize('next_kind',['zero','fragment','fragments'])
def test_qualified_depth_gap_after_pending_still_has_original_identity_only_behavior(monkeypatch,next_kind):
    n,clock,_,joint,_=locked(monkeypatch)
    reference=n._locked_rgb_hold_reference;stamp=n.last_good_source_stamp_ns
    pending_frame(n,clock)
    clock[0]+=.1;custom_frame(n,clock,joint,target_depth=gap_depth(next_kind))
    assert n._locked_source_pending is None and not n.tracker.lost
    assert latest_status(n)['rgb_identity_hold']['state']=='holding_identity_only'
    assert n._locked_rgb_hold_reference is reference and n.last_good_source_stamp_ns==stamp
    assert not n.pubs['object'].publish.call_args[0][0].detected


@pytest.mark.parametrize('change',['rgb_missing','rgb_shift','context','config','epoch','pose',
    'pose_out_and_back','support_missing','support_shift','support_nan','expired','future',
    'reference_removed','already_lost'])
def test_metric_looking_frame_cannot_bypass_pending_requalification(monkeypatch,change):
    from unittest.mock import Mock
    n,clock,_,joint,params=locked(monkeypatch)
    pending_frame(n,clock)
    choose=Mock(wraps=n.tracker.choose);n.tracker.choose=choose
    kwargs=dict(target_depth=.35)
    if change=='rgb_missing':kwargs['rgb_missing']=True
    elif change=='rgb_shift':kwargs['shift']=15
    elif change=='context':params['/handeye']['source_changed']=True
    elif change=='config':n.config.support_threshold_m=.0019
    elif change=='epoch':n._rgb_interest_epoch_cb(Header(stamp=module.rospy.Time(2),frame_id='sdk_reference_epoch'))
    elif change=='pose':kwargs['q']=[.005,0.,0.,0.,0.,0.]
    elif change=='pose_out_and_back':
        joint(clock[0]-.02,[.005,0.,0.,0.,0.,0.]);joint(clock[0]-.01,[0.]*6)
    elif change=='expired':clock[0]+=.6
    elif change=='future':clock[0]-=.4
    elif change=='reference_removed':n._locked_rgb_hold_reference=None
    elif change=='already_lost':n.tracker.lost=True
    else:
        original=module.segment_with_support_fallback
        def bad_audit(*args,**kw):
            result=original(*args,**kw)
            assert result.metrics['support_source']=='locked_base_plane'
            if change=='support_missing':result.metrics['support_current_audit_available']=False
            elif change=='support_shift':result.metrics['support_reference_max_distance_m']=.0071
            else:result.metrics['support_reference_max_distance_m']=float('nan')
            return result
        monkeypatch.setattr(module,'segment_with_support_fallback',bad_audit)
    clock[0]+=.1;custom_frame(n,clock,joint,**kwargs)
    choose.assert_not_called()
    assert n.tracker.lost and n._locked_source_pending is None
    assert not n.pubs['object'].publish.call_args[0][0].detected
    assert not np.any(n.bridge.imgmsg_to_cv2(n.pubs['object_mask'].publish.call_args[0][0]))


def test_repeated_unqualified_metric_frames_never_renew_pending_deadline(monkeypatch):
    from unittest.mock import Mock
    n,clock,_,_,_=locked(monkeypatch)
    reference=n._locked_rgb_hold_reference;stamp=n.last_good_source_stamp_ns
    pending_frame(n,clock)
    first=n._locked_source_pending['first_unqualified_source_stamp_ns']
    choose=Mock(wraps=n.tracker.choose);n.tracker.choose=choose
    for step in (.1,.1):
        clock[0]+=step;pending_frame(n,clock,target_depth=.35)
        assert not n.tracker.lost and n._locked_source_pending is not None
        assert n._locked_source_pending['first_unqualified_source_stamp_ns']==first
        assert latest_status(n)['source_qualification']['state']=='pending_source_qualification'
        assert n.last_good_source_stamp_ns==stamp and n._locked_rgb_hold_reference is reference
    clock[0]+=.31;pending_frame(n,clock,target_depth=.35)
    assert n.tracker.lost and n._locked_source_pending is None
    assert n.last_good_source_stamp_ns==stamp
    choose.assert_not_called()


@pytest.mark.parametrize('invalidate',['expiry','pose','epoch'])
def test_no_image_timer_retires_pending_without_waiting_for_stream_timeout(monkeypatch,invalidate):
    n,clock,_,joint,_=locked(monkeypatch)
    pending_frame(n,clock)
    if invalidate=='expiry':clock[0]+=.51
    elif invalidate=='pose':joint(clock[0]-.01,[.005,0.,0.,0.,0.,0.])
    else:n._rgb_interest_epoch_cb(Header(stamp=module.rospy.Time(2),frame_id='sdk_reference_epoch'))
    n.latest=None;n.last_receive_wall=clock[0]
    n.process(None)
    assert n.tracker.lost and n._locked_source_pending is None
    assert latest_status(n)['detail'].startswith('source_qualification_rejected: ')
    assert not n.pubs['object'].publish.call_args[0][0].detected


def test_new_selection_clears_pending_reference_and_status(monkeypatch):
    n,clock,_,_,_=locked(monkeypatch)
    pending_frame(n,clock)
    assert n._locked_source_pending is not None
    n.selection(String(data=json.dumps(dict(mode='unknown',generation=n.generation+1,stamp_ns=int(clock[0]*1e9)))))
    assert n._locked_source_pending is None and n._locked_source_status is None
    assert n._locked_rgb_hold_reference is None and n.tracker.anchor is None


@pytest.mark.parametrize('reason',['conflicting_accepted_feedback_source',
    'source_predates_selection_or_epoch','source_exceeds_existing_frame_limit'])
def test_only_missing_feedback_bracket_can_enter_pending(monkeypatch,reason):
    n,clock,_,joint,_=locked(monkeypatch)
    def rejected(*_args):raise ValueError(reason)
    monkeypatch.setattr(n,'_rgb_bracketed_feedback',rejected)
    custom_frame(n,clock,joint,target_depth=fragment_depth())
    assert n.tracker.lost and n._locked_source_pending is None
    assert not n.pubs['object'].publish.call_args[0][0].detected


@pytest.mark.parametrize('change',['pose','pose_out_and_back','epoch','context','old_ttl','new_bracket_gap','future_feedback'])
def test_pending_recovery_rechecks_source_after_refinement_before_live_commit(monkeypatch,change):
    n,clock,feed,joint,params=locked(monkeypatch)
    pending_frame(n,clock)
    anchor,last=n.tracker.anchor,n.tracker.last
    old_good=n.last_good_source_stamp_ns
    original=module.refine_candidate
    def invalidated(*args,**kwargs):
        candidate=original(*args,**kwargs)
        assert n.tracker.anchor is anchor and n.tracker.last is last
        assert n._locked_source_pending is not None
        if change in ('pose','pose_out_and_back'):
            joint(clock[0]-.01,[.005,0.,0.,0.,0.,0.])
            if change=='pose_out_and_back':joint(clock[0],[0.]*6)
        elif change=='epoch':n._rgb_interest_epoch_cb(Header(stamp=module.rospy.Time(2),frame_id='sdk_reference_epoch'))
        elif change=='context':params['/handeye']['changed_during_refinement']=True
        elif change=='old_ttl':clock[0]+=.41
        elif change=='future_feedback':joint(clock[0]+.2,[0.]*6)
        else:
            n._rgb_interest_feedback.clear()
            source_stamp=clock[0]-.1
            joint(source_stamp-.14);joint(source_stamp+.06004325)
        return candidate
    monkeypatch.setattr(module,'refine_candidate',invalidated)
    clock[0]+=.1;feed(target_depth=.35)
    assert n.tracker.lost and n._locked_source_pending is None
    assert n.tracker.anchor is anchor and n.tracker.last is last
    assert n.last_good_source_stamp_ns==old_good
    assert not n.pubs['object'].publish.call_args[0][0].detected
    assert not np.any(n.bridge.imgmsg_to_cv2(n.pubs['object_mask'].publish.call_args[0][0]))
    qualification=latest_status(n)['source_qualification']
    assert qualification['state']=='rejected' and not qualification['source_qualified']
