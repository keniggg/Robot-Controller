"""Real three-frame arrival and later process calls recover measured fragments."""
from copy import deepcopy
import numpy as np
import pytest
from test_observation_measurement_unknown import frozen,arrival,send,ack
from test_locked_rgb_hold import custom_frame,latest_status,module
from alicia_grasp_modes.tabletop import _matching_candidates,_project,_measured_support_footprint


def split_depth():
    depth=np.zeros((28,25),np.float32)
    depth[:,0:7]=.35;depth[:,10:17]=.35;depth[:,20:25]=.35
    return depth


def fragment_frame(n,clock,joint):
    clock[0]+=.2
    joint(clock[0]-.2);joint(clock[0]-.1)
    return custom_frame(n,clock,joint,target_depth=split_depth())


def completed_fragment_arrival(monkeypatch):
    n,clock,feed,joint,params,request=frozen(monkeypatch)
    anchor=n.tracker.anchor;plane=n.tracker.plane_base.copy();count=n.pubs['object'].publish.call_count
    request=arrival(n,clock,joint,request);stamps=[];raw_counts=[];original=module.segment_with_support_fallback
    def inspect(*args,**kwargs):
        result=original(*args,**kwargs)
        raw_counts.append(len(_matching_candidates(result.candidates,anchor,n.tracker.last,n.config)))
        return result
    monkeypatch.setattr(module,'segment_with_support_fallback',inspect)
    for i in range(3):
        pair=fragment_frame(n,clock,joint);stamps.append(pair[0].header.stamp.to_nsec())
        assert n.pubs['object'].publish.call_count==count+(i==2),latest_status(n)
    assert raw_counts==[0,0,0]
    assert ack(n)['qualified'] and ack(n)['source_stamps_ns']==stamps,latest_status(n)
    assert n.tracker.anchor is anchor;np.testing.assert_array_equal(n.tracker.plane_base,plane)
    terminal=deepcopy(request);terminal.update(state='qualified',request_id='completed',
        stamp_ns=module.rospy.Time.now().to_nsec(),source_stamps_ns=stamps)
    send(n,terminal);assert ack(n)['qualified'] and n._observation_measurement is None
    return n,clock,feed,joint,params,n._locked_rgb_hold_reference['observation_arrival_retry']


def test_actual_arrival_three_frames_then_normal_group_geometry_and_frozen_identity(monkeypatch):
    n,clock,feed,joint,params,marker=completed_fragment_arrival(monkeypatch)
    fixed=(marker['spatial_reference'],marker['fragment_histogram'],marker['fragment_contour'])
    for _ in range(3):
        pair=fragment_frame(n,clock,joint)
        assert latest_status(n)['state']=='ready',latest_status(n)
        assert n._locked_rgb_hold_reference['observation_arrival_retry'] is marker
        assert all(marker[k] is v for k,v in zip(('spatial_reference','fragment_histogram','fragment_contour'),fixed))
        raw=n.tracker.last;mask=raw.mask>0
        depth=n.bridge.imgmsg_to_cv2(pair[1],desired_encoding='passthrough')
        assert np.all(np.isfinite(depth[mask])) and np.all(depth[mask]>.03)
        assert raw.metrics['valid_points']==int(mask.sum())
        yy,xx=np.nonzero(mask);assert raw.bbox==(int(xx.min()),int(yy.min()),int(xx.max()-xx.min()+1),int(yy.max()-yy.min()+1))
        camera=params['/camera'];K=[camera[k] for k in ('fx','fy','cx','cy')]
        xyz=_project(depth,K)[mask].astype(float)
        np.testing.assert_allclose(raw.position_camera,np.median(xyz,axis=0),rtol=0,atol=1e-12)
        centered=xyz-xyz.mean(0);_,_,vh=np.linalg.svd(centered,full_matrices=False)
        extent=np.sort(np.percentile(centered@vh.T,98,axis=0)-np.percentile(centered@vh.T,2,axis=0))[::-1]
        np.testing.assert_allclose(raw.extent_m,extent,rtol=0,atol=1e-12)
        T=module.transform_matrix(n.buffer.lookup_transform(n.base_frame,n.camera_frame,pair[0].header.stamp,module.rospy.Duration(0)))
        base=xyz@T[:3,:3].T+T[:3,3];plane=np.asarray(raw.metrics['support_plane_base'])
        np.testing.assert_allclose(raw.support_footprint_base,_measured_support_footprint(base,plane),rtol=0,atol=1e-12)
        assert raw.metrics['median_height_m']==pytest.approx(float(np.median(base@plane[:3]+plane[3])),abs=1e-8)
        expected=.65*max(0.,1.-raw.metrics['support_residual_m']/(2*n.config.support_threshold_m))+.35*min(1.,mask.sum()/400.)
        assert raw.quality==pytest.approx(expected)
        assert raw.metrics['observation_fragment_group']['final_measured_points']==mask.sum()


@pytest.mark.parametrize('kind',['support','metric'])
def test_current_fragment_group_reuses_single_rgb_through_existing_recovery(monkeypatch,kind):
    from test_support_measurement_hold import support_gap_depth
    from test_metric_ambiguity_hold import frame
    import alicia_grasp_modes.arrival_fragment_group as recovery
    n,clock,feed,joint,params,marker=completed_fragment_arrival(monkeypatch)
    clock[0]+=.1
    if kind=='support':custom_frame(n,clock,joint,depth_image=support_gap_depth())
    else:frame(n,clock,joint)
    assert not n.tracker.lost and (n._locked_source_pending is not None or n._metric_ambiguity_pending is not None),latest_status(n)
    calls=[];original=recovery._FrameRefinement.refine
    def once(*args,**kwargs):calls.append(1);return original(*args,**kwargs)
    monkeypatch.setattr(recovery._FrameRefinement,'refine',once)
    fragment_frame(n,clock,joint)
    assert latest_status(n)['state']=='ready',latest_status(n)
    assert calls==[1]
    assert n._locked_source_pending is None and n._metric_ambiguity_pending is None
    assert n._locked_rgb_hold_reference['observation_arrival_retry'] is marker


@pytest.mark.parametrize('fault',['after_rgb_stale','after_rgb_motion','after_rgb_session','commit_lock_stale'])
def test_group_normal_commit_failure_cannot_change_live_last_or_reference(monkeypatch,fault):
    import inspect
    n,clock,feed,joint,params,marker=completed_fragment_arrival(monkeypatch)
    old_last=n.tracker.last;old_hold=n._locked_rgb_hold_reference;old_source=n.last_good_source_stamp_ns
    old_publications=n.pubs['object'].publish.call_count
    original=module.recover_arrival_fragment_group
    def interrupted(*args,**kwargs):
        result=original(*args,**kwargs)
        assert result[1] is not None
        if fault=='after_rgb_stale':clock[0]+=.81
        elif fault=='after_rgb_motion':
            joint(clock[0],[.005,0,0,0,0,0]);joint(clock[0]+.01,[0.]*6)
        elif fault=='after_rgb_session':marker['session_deadline_ns']=module.rospy.Time.now().to_nsec()
        return result
    monkeypatch.setattr(module,'recover_arrival_fragment_group',interrupted)
    if fault=='commit_lock_stale':
        lock=n.input_lock;at_commit=[False];stage=n._frame_timing_stage
        def timing(name,*args,**kwargs):
            if name=='post_refine':at_commit[0]=True
            return stage(name,*args,**kwargs)
        monkeypatch.setattr(n,'_frame_timing_stage',timing)
        class DelayedCommit:
            def __enter__(self):
                if at_commit[0] and inspect.currentframe().f_back.f_code.co_name=='process':
                    clock[0]+=.81;at_commit[0]=False
                return lock.__enter__()
            def __exit__(self,*args):return lock.__exit__(*args)
        n.input_lock=DelayedCommit()
    fragment_frame(n,clock,joint)
    assert n.tracker.last is old_last
    # The existing loss latch may revoke the old reference; it must never
    # replace it with a reference made from the rejected frame.
    assert n._locked_rgb_hold_reference is None or n._locked_rgb_hold_reference is old_hold
    assert n.last_good_source_stamp_ns==old_source
    assert all(not call[0][0].detected for call in n.pubs['object'].publish.call_args_list[old_publications:])
    detail=latest_status(n)['detail']
    if fault=='commit_lock_stale':
        assert 'observation_arrival_station_invalidated_at_commit' in detail
        timing=latest_status(n)['frame_timing']
        assert timing['arrival_station_invalidated_reason']=='arrival_frame_completion_binding_changed_or_expired'
        assert timing['failure_source_age_sec']>=.8
    else:
        assert 'observation_arrival_station_invalidated_during_recovery' in detail


@pytest.mark.parametrize('fault',['different_pair_object','absent_station','metadata_without_pair'])
def test_recovery_does_not_infer_current_fit_permission_from_group_metadata(monkeypatch,fault):
    from dataclasses import replace
    from test_metric_ambiguity_hold import frame
    n,clock,feed,joint,params,marker=completed_fragment_arrival(monkeypatch)
    clock[0]+=.1;frame(n,clock,joint)
    assert n._metric_ambiguity_pending is not None
    old_last=n.tracker.last;old_source=n.last_good_source_stamp_ns
    original=n._metric_ambiguity_transition
    def invalid(*args,**kwargs):
        pair=kwargs['pre_refined'];assert pair is not None
        if fault=='different_pair_object':kwargs['pre_refined']=(replace(pair[0]),pair[1])
        elif fault=='absent_station':kwargs['arrival_retry_reference']=None
        else:kwargs['pre_refined']=None
        return original(*args,**kwargs)
    monkeypatch.setattr(n,'_metric_ambiguity_transition',invalid)
    fragment_frame(n,clock,joint)
    assert n.tracker.last is old_last and n.last_good_source_stamp_ns==old_source
    assert not n.pubs['object'].publish.call_args[0][0].detected
    assert 'observation_fragment_recovery_pair_invalid' in latest_status(n)['detail'] or 'metric_ambiguity_current_support_unavailable' in latest_status(n)['detail']
