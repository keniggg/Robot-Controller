"""Support gaps withhold geometry and wait for identity revalidation.

Synthetic native RGB-D process fixtures, not replay of the unsaved bad hardware
frame. All support and tracking quality gates retain their production values.
"""
from dataclasses import replace

import numpy as np
import pytest
from std_msgs.msg import Header

from test_locked_rgb_hold import locked, custom_frame, latest_status, module
from test_unknown_stream import rgb_interest_node


def support_gap_depth(kind='missing_ring', target=True):
    depth = np.full((240, 320), .4, np.float32)
    v, u = np.indices(depth.shape)
    radius = np.hypot(u-160, v-120)
    ring = (radius >= 30) & (radius <= 95)
    if kind == 'missing_ring':
        depth[ring] = 0.
    elif kind == 'noise':
        depth[ring] = np.random.RandomState(42).uniform(.30, .50, np.count_nonzero(ring))
    elif kind == 'missing_all':
        depth[:] = 0.
    else:
        raise AssertionError(kind)
    depth[106:134, 148:173] = .35 if target else 0.
    return depth


def false_output(n):
    assert not n.pubs['object'].publish.call_args[0][0].detected
    assert not n.pubs['object_detected'].publish.call_args[0][0].data
    assert not n.pubs['raw_object_detected'].publish.call_args[0][0].data
    assert not np.any(n.bridge.imgmsg_to_cv2(n.pubs['object_mask'].publish.call_args[0][0]))


@pytest.mark.parametrize('kind,target', [('missing_ring',True),('missing_ring',False),
                                         ('noise',False),('missing_all',False)])
def test_real_support_failure_holds_only_identity_and_recovers_original_geometry(monkeypatch,kind,target):
    n,clock,feed,joint,_=locked(monkeypatch)
    reference=n._locked_rgb_hold_reference;last=n.tracker.last;anchor=n.tracker.anchor
    stamp=n.last_good_source_stamp_ns;plane=n.tracker.plane_base.copy()
    original=module.segment_with_support_fallback
    observed=[]
    def captured(*args,**kwargs):
        try:
            result=original(*args,**kwargs)
        except ValueError as exc:
            observed.append(str(exc));raise
        observed.append(result)
        return result
    monkeypatch.setattr(module,'segment_with_support_fallback',captured)
    custom_frame(n,clock,joint,depth_image=support_gap_depth(kind,target))
    false_output(n)
    assert not n.tracker.lost,latest_status(n)
    status=latest_status(n)['source_qualification']
    assert status['state']=='pending_current_support'
    assert status['identity_verified'] is False and status['source_qualified']
    assert not status['support_measurement_available']
    assert not status['metric_geometry_authority'] and not status['motion_authorized']
    assert not status['reference_refreshed'] and status['maximum_hold_seconds']==.8
    assert n._locked_rgb_hold_reference is reference and n.last_good_source_stamp_ns==stamp
    assert n.tracker.last is last and n.tracker.anchor is anchor
    np.testing.assert_array_equal(n.tracker.plane_base,plane)
    if kind=='missing_ring' and target:
        assert len(observed[0].candidates)==1
        assert observed[0].metrics['support_current_audit_available'] is False
    else:
        assert isinstance(observed[0],str)
    clock[0]+=.2;feed(target_depth=.35)
    assert n.pubs['object'].publish.call_args[0][0].detected
    assert n._locked_source_pending is None and n.last_good_source_stamp_ns>stamp
    assert n.tracker.anchor is anchor and not n.tracker.lost
    assert latest_status(n)['detail']=='tracked'


def test_tilted_acquired_plane_roundoff_keeps_support_gap_and_recovery_contract(monkeypatch):
    n,clock,feed,joint,_=rgb_interest_node(monkeypatch)
    transform=np.eye(4)
    transform[:3,:3],_=np.linalg.qr(np.random.RandomState(27).normal(size=(3,3)))
    monkeypatch.setattr(module,'transform_matrix',lambda _:transform.copy())
    for _ in range(3):
        feed(target_depth=.35);clock[0]+=.2
    assert n.tracker.anchor is not None and not n.tracker.lost
    reference=n._locked_rgb_hold_reference;plane=n.tracker.plane_base.copy()
    assert reference is not None
    # A real synthetic fit at a tilted camera pose has the same one-ULP
    # normalization issue as the recorded generation 27 acquisition plane.
    assert not np.array_equal(plane,plane/np.linalg.norm(plane[:3]))
    anchor,last=n.tracker.anchor,n.tracker.last;stamp=n.last_good_source_stamp_ns
    custom_frame(n,clock,joint,depth_image=support_gap_depth())
    false_output(n)
    assert not n.tracker.lost,latest_status(n)
    status=latest_status(n)['source_qualification']
    assert status['state']=='pending_current_support'
    assert not status['identity_verified'] and not status['support_measurement_available']
    assert not status['metric_geometry_authority'] and not status['motion_authorized']
    assert not status['reference_refreshed']
    assert n._locked_rgb_hold_reference is reference and n.last_good_source_stamp_ns==stamp
    assert n.tracker.anchor is anchor and n.tracker.last is last
    np.testing.assert_array_equal(n.tracker.plane_base,plane)
    clock[0]+=.2;feed(target_depth=.35)
    assert n.pubs['object'].publish.call_args[0][0].detected
    assert not n.tracker.lost and n._locked_source_pending is None
    assert n.tracker.anchor is anchor and n.last_good_source_stamp_ns>stamp
    np.testing.assert_array_equal(n.tracker.plane_base,plane)
    status=latest_status(n)['source_qualification']
    assert status['state']=='recovered_unique_measured_frame'
    assert status['identity_verified'] and status['measurement_republished']


@pytest.mark.parametrize('change',['bracket','pose','out_and_back','epoch',
    'camera','handeye','selection','generation','config','stale','lost'])
def test_support_gap_does_not_rescue_invalid_identity_or_source(monkeypatch,change):
    n,clock,_,joint,params=locked(monkeypatch)
    kwargs=dict(depth_image=support_gap_depth())
    if change=='bracket':kwargs['bracket']=False
    elif change=='pose':kwargs['q']=[.005,0.,0.,0.,0.,0.]
    elif change=='out_and_back':
        joint(clock[0]-.15,[.005,0.,0.,0.,0.,0.]);joint(clock[0]-.13,[0.]*6)
    elif change=='epoch':n._rgb_interest_epoch_cb(Header(stamp=module.rospy.Time(2),frame_id='sdk_reference_epoch'))
    elif change=='camera':params['/camera']['fx']+=1
    elif change=='handeye':params['/handeye']['mount']='changed'
    elif change=='selection':params['/grasp_mode/selection']['generation']+=1
    elif change=='generation':n.generation+=1
    elif change=='config':n.config.support_threshold_m=.0019
    elif change=='stale':kwargs['old_stamp']=True
    elif change=='lost':n.tracker.lost=True
    custom_frame(n,clock,joint,**kwargs)
    false_output(n)
    assert n.tracker.lost and n._locked_source_pending is None


@pytest.mark.parametrize('fault',['positive_shift','positive_angle','nan_angle','wrong_plane',
    'unknown_reason','invalid_transform','invalid_depth_shape','outside_segmentation'])
def test_false_audit_never_hides_positive_contradiction_or_unrelated_failures(monkeypatch,fault):
    n,clock,_,joint,_=locked(monkeypatch)
    original=module.segment_with_support_fallback
    def changed(*args,**kwargs):
        if fault in ('invalid_transform','invalid_depth_shape'):
            raise ValueError(fault)
        result=original(*args,**kwargs)
        assert result.metrics['support_current_audit_available'] is False
        if fault=='positive_shift':result.metrics['support_reference_max_distance_m']=.0041
        elif fault=='positive_angle':result.metrics['support_reference_angle_deg']=4.1
        elif fault=='nan_angle':result.metrics['support_reference_angle_deg']=float('nan')
        elif fault=='wrong_plane':result.plane_base[3]+=.0041
        elif fault=='unknown_reason':result.metrics['support_current_audit_reason']='unclassified_fault'
        return result
    monkeypatch.setattr(module,'segment_with_support_fallback',changed)
    if fault=='outside_segmentation':
        monkeypatch.setattr(n,'_rgb_interest_source_context',
            lambda *args:(_ for _ in ()).throw(ValueError('support_plane_inlier_ratio_low')))
    custom_frame(n,clock,joint,depth_image=support_gap_depth())
    false_output(n)
    assert n.tracker.lost and n._locked_source_pending is None


def test_repeated_gap_cannot_renew_deadline_and_complete_frame_cannot_revive_loss(monkeypatch):
    n,clock,feed,joint,_=locked(monkeypatch)
    stamp=n.last_good_source_stamp_ns
    for advance in (0.,.2,.2):
        clock[0]+=advance;custom_frame(n,clock,joint,depth_image=support_gap_depth())
        assert not n.tracker.lost and n.last_good_source_stamp_ns==stamp
    clock[0]+=.100001;n.latest=None;n.last_receive_wall=module.time.monotonic();n.process(None)
    assert n.tracker.lost and n._locked_source_pending is None
    clock[0]+=.01;feed(target_depth=.35)
    false_output(n)
    assert n.tracker.lost and n.last_good_source_stamp_ns==stamp


@pytest.mark.parametrize('change',['camera','handeye','config','epoch','pose','wait_past_deadline'])
def test_pending_no_frame_detects_context_loss_without_waiting_for_new_images(monkeypatch,change):
    n,clock,_,joint,params=locked(monkeypatch)
    custom_frame(n,clock,joint,depth_image=support_gap_depth())
    assert not n.tracker.lost
    if change=='camera':params['/camera']['fx']+=1
    elif change=='handeye':params['/handeye']['mount']='changed'
    elif change=='config':n.config.support_threshold_m=.0019
    elif change=='epoch':n._rgb_interest_epoch_cb(Header(stamp=module.rospy.Time(2),frame_id='sdk_reference_epoch'))
    elif change=='pose':joint(clock[0],[.005,0.,0.,0.,0.,0.])
    else:
        original=n._rgb_interest_source_context
        def waited(*args):
            result=original(*args);clock[0]+=.6;return result
        monkeypatch.setattr(n,'_rgb_interest_source_context',waited)
    n.latest=None;n.last_receive_wall=module.time.monotonic();n.process(None)
    false_output(n)
    assert n.tracker.lost and n._locked_source_pending is None


def test_replayed_source_is_not_new_support_gap_evidence(monkeypatch):
    n,clock,_,joint,_=locked(monkeypatch)
    source=custom_frame(n,clock,joint,depth_image=support_gap_depth())
    n.latest=source;n.process(None)
    false_output(n)
    assert n.tracker.lost


def test_support_recovery_without_metric_object_remains_pending_until_real_recovery(monkeypatch):
    n,clock,feed,joint,_=locked(monkeypatch)
    custom_frame(n,clock,joint,depth_image=support_gap_depth())
    pending=dict(n._locked_source_pending);reference=n._locked_rgb_hold_reference
    clock[0]+=.2;feed(target_depth=0.)
    false_output(n)
    assert not n.tracker.lost and n._locked_rgb_hold_reference is reference
    assert n._locked_source_pending['first_unqualified_source_stamp_ns']==pending['first_unqualified_source_stamp_ns']
    assert latest_status(n)['source_qualification']['state']=='pending_unique_measured_geometry'
    assert latest_status(n)['source_qualification']['identity_verified'] is False
    clock[0]+=.1;feed(target_depth=.35)
    assert n.pubs['object'].publish.call_args[0][0].detected
    assert n._locked_source_pending is None


@pytest.mark.parametrize('change',['epoch','context','pose','bracket','ambiguous_metric','expired'])
def test_recovery_cannot_commit_after_refinement_or_unique_geometry_failure(monkeypatch,change):
    n,clock,feed,joint,params=locked(monkeypatch)
    custom_frame(n,clock,joint,depth_image=support_gap_depth())
    original_refine=module.refine_candidate
    anchor,last=n.tracker.anchor,n.tracker.last;stamp=n.last_good_source_stamp_ns
    if change=='ambiguous_metric':
        segment=module.segment_with_support_fallback
        def ambiguous(*args,**kwargs):
            result=segment(*args,**kwargs);result.candidates.append(result.candidates[0]);return result
        monkeypatch.setattr(module,'segment_with_support_fallback',ambiguous)
    else:
        def changed(*args,**kwargs):
            result=original_refine(*args,**kwargs)
            if change=='epoch':n._rgb_interest_epoch_cb(Header(stamp=module.rospy.Time(2),frame_id='sdk_reference_epoch'))
            elif change=='context':params['/camera']['fx']+=1
            elif change=='pose':joint(clock[0],[.005,0.,0.,0.,0.,0.])
            elif change=='bracket':n._rgb_interest_feedback.clear()
            elif change=='expired':clock[0]+=.5
            return result
        monkeypatch.setattr(module,'refine_candidate',changed)
    clock[0]+=.2;feed(target_depth=.35)
    false_output(n)
    assert n.tracker.lost and n.tracker.anchor is anchor and n.tracker.last is last
    assert n.last_good_source_stamp_ns==stamp


def test_support_pending_cannot_downgrade_real_brackets_to_weaker_source_wait(monkeypatch):
    n,clock,_,joint,_=locked(monkeypatch)
    custom_frame(n,clock,joint,depth_image=support_gap_depth())
    clock[0]+=.2
    custom_frame(n,clock,joint,target_depth=.35,bracket=False)
    false_output(n)
    assert n.tracker.lost and n._locked_source_pending is None


@pytest.mark.parametrize('rgb_missing,shift',[(True,0),(False,15),(False,0)])
def test_missing_support_wait_never_claims_current_identity_even_with_missing_or_shifted_rgb(monkeypatch,rgb_missing,shift):
    n,clock,_,joint,_=locked(monkeypatch)
    reference=n._locked_rgb_hold_reference
    monkeypatch.setattr(module,'propose_rgb_observation_regions',
        lambda *a,**k:pytest.fail('unverified waiting must not claim an independent RGB match'))
    custom_frame(n,clock,joint,depth_image=support_gap_depth(),rgb_missing=rgb_missing,shift=shift)
    false_output(n)
    assert not n.tracker.lost and n._locked_rgb_hold_reference is reference
    status=latest_status(n)['source_qualification']
    assert status['identity_verified'] is False and status['support_measurement_available'] is False
    assert not status['metric_geometry_authority'] and not status['motion_authorized']
    assert 'mask_iou' not in status and 'rgb_identity' not in status


def test_recovery_compares_actual_refined_masks_without_independent_mser(monkeypatch):
    n,clock,feed,joint,_=locked(monkeypatch)
    monkeypatch.setattr(module,'propose_rgb_observation_regions',
        lambda *a,**k:pytest.fail('support recovery must use current measured RGB-D refinement'))
    custom_frame(n,clock,joint,depth_image=support_gap_depth())
    clock[0]+=.2;feed(target_depth=.35)
    assert n.pubs['object'].publish.call_args[0][0].detected
    status=latest_status(n)['source_qualification']
    assert status['state']=='recovered_unique_measured_frame'
    assert status['identity_verified'] and status['measurement_republished']
    identity=status['rgb_identity']
    assert identity['evidence_type']=='metric_seeded_full_outline_rgb_identity'
    assert identity['full_outline_iou_minimum']==.85
    assert identity['max_colour_distance']==module.HandoffPolicy().max_colour_distance
    assert identity['max_contour_distance']==module.HandoffPolicy().max_contour_distance
    assert identity['candidates'][0]['mask_iou']>=.85
    assert not identity['independent_mser_contract_claimed']


@pytest.mark.parametrize('fault',['shifted_outline','colour','support_shift','support_ratio',
    'support_quadrants','support_residual','support_audit_missing','rgb_refinement_failed'])
def test_support_recovery_rejects_new_visual_or_support_contradiction(monkeypatch,fault):
    n,clock,_,joint,_=locked(monkeypatch)
    custom_frame(n,clock,joint,depth_image=support_gap_depth())
    reference=n._locked_rgb_hold_reference;last=n.tracker.last;stamp=n.last_good_source_stamp_ns
    kwargs=dict(target_depth=.35)
    if fault=='shifted_outline':kwargs['shift']=5
    elif fault=='colour':
        decode=n.bridge.imgmsg_to_cv2
        def changed(message,desired_encoding='passthrough'):
            result=decode(message,desired_encoding=desired_encoding)
            if desired_encoding=='bgr8':
                result=result.copy();result[106:134,148:173]=(220,220,220)
            return result
        monkeypatch.setattr(n.bridge,'imgmsg_to_cv2',changed)
    elif fault=='rgb_refinement_failed':
        monkeypatch.setattr(module,'refine_candidate',lambda *a,**k:
            (_ for _ in ()).throw(ValueError('support_plane_inlier_ratio_low')))
    else:
        segment=module.segment_with_support_fallback
        def changed(*args,**kwargs):
            result=segment(*args,**kwargs)
            field,value=dict(support_shift=('support_reference_max_distance_m',.0041),
                support_ratio=('support_current_audit_inlier_ratio',n.config.support_min_ratio-.01),
                support_quadrants=('support_current_audit_quadrants',2),
                support_residual=('support_current_audit_residual_m',n.config.support_threshold_m+.0001),
                support_audit_missing=('support_current_audit_available',False))[fault]
            result.metrics[field]=value
            return result
        monkeypatch.setattr(module,'segment_with_support_fallback',changed)
    clock[0]+=.2;custom_frame(n,clock,joint,**kwargs)
    false_output(n)
    assert n.tracker.lost and n.tracker.last is last and n.last_good_source_stamp_ns==stamp
    assert n._locked_rgb_hold_reference is None and n._locked_source_pending is None
