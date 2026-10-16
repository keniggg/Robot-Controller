"""A diagonal explanation may select only one independently complete owner."""
from dataclasses import replace
from pathlib import Path
import json
import cv2
import numpy as np
import pytest
from alicia_grasp_modes import tabletop as t, arrival_fragment_group as group
from alicia_grasp_modes.observation_arrival_confirmation import ObservationArrivalConfirmation
from alicia_grasp_modes.rgb_observation_handoff import _appearance
from test_arrival_matching_fragments import scene


def comps(depth,connectivity):
    return t._depth_components((depth>0).astype('uint8'),depth,4,.008,connectivity=connectivity)


def test_diagonal_measurements_connect_without_filling_missing_corner_pixels():
    d=np.zeros((16,16),np.float32);d[2:7,2:7]=.3;d[7:12,7:12]=.3001
    assert len(comps(d,4))==2
    c=comps(d,8);assert len(c)==1
    np.testing.assert_array_equal(c[0],d>0)
    assert not c[0][6,7] and not c[0][7,6]


@pytest.mark.parametrize('kind',['invalid_stripe','depth_jump'])
def test_disconnected_or_depth_discontinuous_current_pieces_remain_separate(kind):
    d=np.zeros((16,16),np.float32);d[2:7,2:7]=.3
    if kind=='invalid_stripe':d[8:13,8:13]=.3
    else:d[7:12,7:12]=.31
    assert len(comps(d,8))==2


@pytest.mark.parametrize('bad',[True,False,0,6,8.,'8',None])
def test_invalid_connectivity_is_not_coerced(bad):
    with pytest.raises(ValueError,match='invalid_depth_component_connectivity'):
        comps(np.full((10,10),.3,np.float32),bad)


def test_existing_all_owner_success_never_runs_diagonal_recovery(monkeypatch):
    inputs=scene()
    monkeypatch.setattr(group,'_depth_components',lambda *a,**k:pytest.fail('unnecessary new branch'))
    _,pair=group.recover_arrival_matching_fragments(**inputs)
    audit=pair[1].metrics['observation_fragment_group']
    assert audit['consensus_policy']=='intersection_of_all_independent_current_rgb_instances'
    assert audit['diagonal_recovery'] is None


@pytest.fixture
def recorded_inputs():
    root=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z/gen24_analysis/replay_inputs')
    if not (root/'manifest.json').exists():pytest.skip('exact sealed GEN24 replay inputs require SanStand')
    m=json.loads((root/'manifest.json').read_text())
    def frame(role):
        row=next(r for r in m['files']if r['role']==role);return dict(np.load(row['path'])),row
    def arrays(f):
        k=f['K'];k=[k[0,0],k[1,1],k[0,2],k[1,2]]if k.shape==(3,3)else k
        return f['depth_raw'].astype(np.float32)*float(f['depth_scale']),k,f['T_base_camera']
    cfg=t.Config();tracker=t.TargetTracker(cfg)
    a,_=frame('anchor');raw,_=tracker.choose(t.segment_with_support_fallback(*arrays(a),cfg),a['depth_raw'].shape)
    status=json.loads((root/'anchor_status.json').read_text())['message']
    np.testing.assert_allclose(raw.position_base,status['anchor_position_base'],atol=1e-10,rtol=0)
    ref,rr=frame('last_pre_motion')
    r=t.segment_with_support_fallback(*arrays(ref),cfg,anchor=tracker.anchor,last=tracker.last,reference_plane_base=tracker.plane_base,prefer_current_support=True)
    matches=t._matching_candidates(r.candidates,tracker.anchor,tracker.last,cfg);assert len(matches)==1
    tracker.last=matches[0][1];hist,contour=_appearance(ref['color'],ref['recorded_refined_mask'])
    f,row=frame('first_failure');depth,k,T=arrays(f)
    result=t.segment_with_support_fallback(depth,k,T,cfg,anchor=tracker.anchor,last=tracker.last,reference_plane_base=tracker.plane_base,prefer_current_support=True)
    assert len(result.candidates)==3 and len(t._matching_candidates(result.candidates,tracker.anchor,tracker.last,cfg))==2
    assert result.metrics['support_current_audit_available'] is True
    helper=ObservationArrivalConfirmation(cfg,tracker.anchor,tracker.last,tracker.plane_base,hist,contour,.5)
    return helper,result,f,depth,k,T,row['source_stamp_ns']


def observe_recorded(inputs):
    h,r,f,d,k,T,ns=inputs
    return h.observe(r,f['color'],d,k,T,ns,rgb_valid=f['rgb_valid'])


def test_exact_gen24_failure_reaches_one_confirmation_with_all_original_cores(recorded_inputs):
    h,r,f,d,k,T,ns=recorded_inputs;anchor=h.anchor;last=h.last;plane=h.plane.copy();before=d.copy()
    confirmed,candidate,staged,evidence=observe_recorded(recorded_inputs)
    assert not confirmed and evidence['reason']=='initial_instance_confirming_1_of_3'
    audit=candidate.metrics['observation_fragment_group'];recovery=audit['diagonal_recovery']
    assert recovery['selected_independent_index']==1
    assert recovery['matched_native_pixels']==recovery['connected_current_pixels']==10837
    assert recovery['nonmatching_candidates_used_as_bridge']==recovery['added_depth_pixels']==0
    assert not recovery['union_hard_seed_used'] and recovery['additional_rgb_refinements']==0
    assert audit['independent_refinements'][0]['retained_native_cores']==[510,560]
    assert audit['independent_refinements'][1]['retained_native_cores']==[510,1235]
    assert h.source_stamps_ns==[ns] and h.anchor is anchor and h.last is last
    np.testing.assert_array_equal(h.plane,plane);np.testing.assert_array_equal(d,before)
    # No current-plane refit is introduced by this recovery.
    np.testing.assert_array_equal(candidate.metrics['support_plane_base'],r.candidates[0].metrics['support_plane_base'])


@pytest.mark.parametrize('fault',['disconnected','discarded_small_piece','no_complete_owner','low_outline_overlap'])
def test_recorded_recovery_refuses_missing_independent_proof(monkeypatch,recorded_inputs,fault):
    if fault in ('disconnected','discarded_small_piece'):
        original=group._depth_components
        def broken(fg,depth,*a,**kw):
            assert kw['connectivity']==8
            if fault=='disconnected':
                h,r,*_=recorded_inputs
                return [c.mask>0 for _,c in t._matching_candidates(r.candidates,h.anchor,h.last,h.config)]
            result=original(fg,depth,*a,**kw);result[0]=result[0].copy()
            y,x=np.argwhere(result[0])[0];result[0][y,x]=False
            return result
        monkeypatch.setattr(group,'_depth_components',broken)
    else:
        original=group._FrameRefinement.refine;first=[]
        def altered(self,*a,**kw):
            current=original(self,*a,**kw)
            if not first:
                first.append(current)
                if fault=='low_outline_overlap':
                    mask=current.mask.copy();mask[:,:310]=0
                    return replace(current,mask=mask)
            return first[0] if fault=='no_complete_owner' else current
        monkeypatch.setattr(group._FrameRefinement,'refine',altered)
    with pytest.raises(ValueError,match='^matching_fragment_competitor_not_owned$'):
        observe_recorded(recorded_inputs)
    assert recorded_inputs[0].source_stamps_ns==[]


@pytest.mark.parametrize('fault',['manual_before','manual_during_recovery','source_expires_during_recovery'])
def test_actual_producer_three_frames_then_qualified_recovery_calls_same_helper(monkeypatch,fault):
    # Real producer/frame/confirmation/marker chain. Only the RGB backend's
    # first independent outline is controlled to model a partial side; all
    # depth segmentation, ownership, final appearance and publication run.
    from test_observation_measurement_unknown import frozen,arrival,send,ack
    from test_locked_rgb_hold import custom_frame,latest_status,module
    n,clock,feed,joint,params,req=frozen(monkeypatch);anchor=n.tracker.anchor;plane=n.tracker.plane_base.copy()
    req=arrival(n,clock,joint,req)
    yy,xx=np.indices((28,25));patch=np.full((28,25),.35,np.float32)
    patch[np.abs(xx+(25./28.)*yy-24.)<.75]=0.
    original=group._FrameRefinement.refine;counter=[0]
    def partial(self,rgb,candidate,**kw):
        output=original(self,rgb,candidate,**kw);counter[0]+=1
        if counter[0]%2:
            pieces=t._depth_components(((self.depth>.3)&(self.depth<.38)).astype('uint8'),self.depth,60,.008)
            other=next(p for p in pieces if not np.any(p&(candidate.mask>0)))
            core=cv2.erode(other.astype('uint8'),np.ones((3,3),np.uint8))>0
            coords=np.argwhere(core);mask=output.mask.copy()
            for y,x in coords[:int(np.ceil(len(coords)*.2))]:mask[y,x]=0
            output=replace(output,mask=mask)
        return output
    monkeypatch.setattr(group._FrameRefinement,'refine',partial)
    baseline=n.pubs['object'].publish.call_count;stamps=[]
    for index in range(3):
        clock[0]+=.2;joint(clock[0]-.2);joint(clock[0]-.1)
        pair=custom_frame(n,clock,joint,target_depth=patch);stamps.append(pair[0].header.stamp.to_nsec())
        assert n.pubs['object'].publish.call_count==baseline+(index==2),latest_status(n)
    assert ack(n)['qualified'] and ack(n)['source_stamps_ns']==stamps,latest_status(n)
    audit=latest_status(n)['geometry_metrics']['observation_fragment_group']
    assert audit['diagonal_recovery'] is not None
    terminal=dict(req,state='qualified',request_id='diagonal-complete',stamp_ns=module.rospy.Time.now().to_nsec(),source_stamps_ns=stamps)
    send(n,terminal);marker=n._locked_rgb_hold_reference['observation_arrival_retry']
    clock[0]+=.2;custom_frame(n,clock,joint,target_depth=patch)
    assert latest_status(n)['state']=='ready',latest_status(n)
    assert latest_status(n)['geometry_metrics']['observation_fragment_group']['diagonal_recovery'] is not None
    assert n.tracker.anchor is anchor;np.testing.assert_array_equal(n.tracker.plane_base,plane)
    before=n.last_good_source_stamp_ns
    if fault=='manual_before':params['/gui/joint_direct_mode']=True
    else:
        original_components=group._depth_components;entered=[]
        def invalidated(*a,**kw):
            result=original_components(*a,**kw)
            assert kw['connectivity']==8;entered.append(True)
            if fault=='manual_during_recovery':params['/gui/joint_direct_mode']=True
            else:clock[0]+=.81;joint(clock[0]-.01)
            return result
        monkeypatch.setattr(group,'_depth_components',invalidated)
    clock[0]+=.2;custom_frame(n,clock,joint,target_depth=patch)
    if fault!='manual_before':assert entered
    assert n.last_good_source_stamp_ns==before
    assert not n.pubs['object'].publish.call_args[0][0].detected
    assert 'observation_arrival_retry' not in (n._locked_rgb_hold_reference or {})


@pytest.mark.parametrize('colour',[(220,220,220),(220,40,30),(20,220,20)])
def test_real_two_metric_corner_objects_rejected_by_independent_rgb_identity(colour):
    from test_arrival_fragment_recovery_guards import fragment_scene,K,PLANE
    inputs=fragment_scene();rgb=inputs['rgb'].copy();rgb[:]=(60,90,190)
    depth=np.full(rgb.shape[:2],.3,np.float32)
    depth[210:238,285:315]=.28;rgb[210:238,285:315]=20
    depth[238:266,315:345]=.28;rgb[238:266,315:345]=colour
    result=t.segment_with_support_fallback(depth,K,np.eye(4),inputs['config'],
        anchor=inputs['anchor'],last=inputs['last'],reference_plane_base=PLANE,
        prefer_current_support=True)
    assert len(result.candidates)==2
    assert len(t._matching_candidates(result.candidates,inputs['anchor'],inputs['last'],inputs['config']))==2
    assert result.metrics['support_current_audit_available'] is True
    helper=ObservationArrivalConfirmation(inputs['config'],inputs['anchor'],inputs['last'],PLANE,
        inputs['histogram'],inputs['contour'],.5)
    with pytest.raises(ValueError,match='^matching_fragment_independent_rgb_identity_required$'):
        helper.observe(result,rgb,depth,K,np.eye(4),123)
    assert helper.source_stamps_ns==[]
