#!/usr/bin/env python3
"""Sealed RGB-D diagnostic: observe original GrabCut inputs, never change gates."""
import hashlib
import json
import inspect
import time
from types import FunctionType, MethodType
from pathlib import Path
from dataclasses import asdict
import cv2
import numpy as np
import rosbag
import rospy
import tf2_py
from cv_bridge import CvBridge
from tf.transformations import quaternion_matrix
from alicia_grasp_modes import tabletop as table
from alicia_grasp_modes.rgb_instance import refine_candidate
from alicia_grasp_modes import rgb_instance
from alicia_grasp_modes.observation_arrival_confirmation import ObservationArrivalConfirmation
from alicia_grasp_modes.rgb_observation_handoff import _appearance, _mask
from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask

ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
RECORD=ROOT/'grasp_records/20261009T015130.939636Z_053c5f63c0a5405da2a53aa19e654e4c'
OUTPUT=ROOT/'ros_log/full_latest_ros_20261009T003407Z/gen6_arrival_rgb_boundary'
EXECUTING=1791510719194801330
ARRIVAL=1791510743796234130
DEADLINE=1791510746796234130

def matrix(msg):
    t=msg.transform;q=t.rotation
    m=quaternion_matrix([q.x,q.y,q.z,q.w]);m[:3,3]=[t.translation.x,t.translation.y,t.translation.z]
    return m

def main():
    cv2.setNumThreads(1)
    assert json.loads((RECORD/'manifest.json').read_text())['recorder_closed'] is True
    static=[];dynamic=[];statuses=[];cameras=[]
    with rosbag.Bag(str(RECORD/'keyframes.bag')) as bag:
        for topic,msg,received in bag.read_messages(topics=['/tf','/tf_static','/perception/unknown/detector_status','/grasp_record/camera_runtime_metadata']):
            if topic=='/tf':dynamic.extend(msg.transforms)
            elif topic=='/tf_static':static.extend(msg.transforms)
            elif topic.endswith('detector_status'):
                row=json.loads(msg.data)
                if row.get('generation')==6:statuses.append((received.to_nsec(),row))
            else:
                row=json.loads(msg.data)
                if row.get('parameters'):cameras.append(row['parameters'])
    acquired=next(row for _,row in statuses if row.get('detail')=='acquired')
    anchor_ns=acquired['source_stamp_ns']
    last_ns=max(row['source_stamp_ns'] for received,row in statuses if received<EXECUTING and row.get('state')=='ready')
    failures=[row for _,row in statuses if row.get('observation_measurement',{}).get('reason')=='rgb_instance_reaches_search_boundary']
    needed={anchor_ns,last_ns,*(row['source_stamp_ns'] for row in failures)};images={};masks={};bridge=CvBridge()
    with rosbag.Bag(str(RECORD/'keyframes.bag')) as bag:
        for topic,msg,_ in bag.read_messages(topics=['/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw','/perception/object_mask','/experiment/observation_rgbd/color','/experiment/observation_rgbd/depth']):
            ns=msg.header.stamp.to_nsec()
            if ns not in needed and not ARRIVAL<ns<DEADLINE:continue
            needed.add(ns)
            if topic.endswith('object_mask'):masks[ns]=bridge.imgmsg_to_cv2(msg,'mono8')
            else:images.setdefault(ns,{})['rgb' if '/color/' in topic or topic.endswith('/color') else 'depth']=msg
    available=sorted(ns for ns in needed if ns not in (anchor_ns,last_ns) and set(images.get(ns,{}))=={'rgb','depth'})
    assert available and all(set(images[ns])=={'rgb','depth'} for ns in (anchor_ns,last_ns))
    selected=available
    OUTPUT.mkdir(parents=True,exist_ok=True)
    for ns in selected:
        cv2.imwrite(str(OUTPUT/f'{ns}_rgb.png'),bridge.imgmsg_to_cv2(images[ns]['rgb'],'bgr8'))
    print('RAW_RGB',str(OUTPUT/f'{selected[0]}_rgb.png'),flush=True)
    buffer=tf2_py.BufferCore(rospy.Duration(600.))
    for t in static:buffer.set_transform_static(t,'sealed_rgb_diagnostic')
    for t in sorted(dynamic,key=lambda t:t.header.stamp.to_nsec()):buffer.set_transform(t,'sealed_rgb_diagnostic')
    camera=cameras[0];keys=('fx','fy','cx','cy','width','height','depth_scale')
    assert all(all(c[k]==camera[k] for k in keys) for c in cameras)
    k=[camera[n] for n in ('fx','fy','cx','cy')];config=table.Config()
    def frame(ns):
        rgb=bridge.imgmsg_to_cv2(images[ns]['rgb'],'bgr8');raw=bridge.imgmsg_to_cv2(images[ns]['depth'],'passthrough')
        depth=np.asarray(raw,dtype=np.float32)*camera['depth_scale']
        transform=matrix(buffer.lookup_transform_core('base_link','camera_link',rospy.Time(ns//10**9,ns%10**9)))
        valid=rectification_rgb_valid_mask(camera,depth.shape)
        return rgb,depth,transform,({} if valid is None else {'rgb_valid':valid})
    rgb,depth,T,kw=frame(anchor_ns)
    seg=table.segment_with_support_fallback(depth,k,T,config)
    tracker=table.TargetTracker(config);anchor,reason=tracker.choose(seg,depth.shape);assert anchor is not None,reason
    assert np.allclose(anchor.position_base,acquired['anchor_position_base'],atol=1e-10,rtol=0)
    assert np.allclose(anchor.extent_m,acquired['anchor_extent_m'],atol=1e-10,rtol=0)
    plane=np.asarray(acquired['reference_support_plane_base']);assert np.allclose(seg.plane_base,plane,atol=1e-10,rtol=0)
    rgb,depth,T,kw=frame(last_ns)
    seg=table.segment_with_support_fallback(depth,k,T,config,anchor,anchor,plane)
    matches=table._matching_candidates(seg.candidates,anchor,anchor,config);assert len(matches)==1
    last=matches[0][1];prior=refine_candidate(rgb,depth,k,T,last,config,**kw)
    assert np.array_equal(prior.mask,masks[last_ns]),'pre-motion candidate reconstruction differs'
    histogram,contour=_appearance(rgb,_mask(masks[last_ns],rgb.shape[:2]))
    helper=ObservationArrivalConfirmation(config,anchor,last,plane,histogram,contour,.5)
    original_code=inspect.getsource(rgb_instance.instance_mask_from_seed)
    old='        result, evidence = attempt(2 * margin)\n        attempts = 2'
    new='''        try:
            result, evidence = attempt(2 * margin)
            attempts = 2
        except ValueError as exc:
            if str(exc) != 'rgb_instance_reaches_search_boundary':
                raise
            result, evidence = attempt(3 * margin)
            attempts = 3'''
    assert original_code.count(old)==1
    namespace=dict(rgb_instance.__dict__);exec(original_code.replace(old,new),namespace)
    trial_refine=FunctionType(refine_candidate.__code__,dict(refine_candidate.__globals__,instance_mask_from_seed=namespace['instance_mask_from_seed']),argdefs=refine_candidate.__defaults__)
    trial_refine.__kwdefaults__=refine_candidate.__kwdefaults__
    trial_observe=FunctionType(ObservationArrivalConfirmation.observe.__code__,dict(ObservationArrivalConfirmation.observe.__globals__,refine_candidate=trial_refine),argdefs=ObservationArrivalConfirmation.observe.__defaults__)
    helper.observe=MethodType(trial_observe,helper)
    OUTPUT.mkdir(parents=True,exist_ok=True);rows=[]
    for ns in selected:
        rgb,depth,T,kw=frame(ns)
        seg=table.segment_with_support_fallback(depth,k,T,config,anchor,last,plane)
        matches=table._matching_candidates(seg.candidates,anchor,last,config);assert len(matches)==1
        candidate=matches[0][1];ys,xs=np.nonzero(candidate.mask);attempts=[];original=cv2.grabCut
        def observed(patch,labels,*args):
            core=labels==cv2.GC_FGD
            original(patch,labels,*args)
            count,components=cv2.connectedComponents(np.isin(labels,(cv2.GC_FGD,cv2.GC_PR_FGD)).astype(np.uint8),connectivity=8)
            matching=[i for i in range(1,count) if np.count_nonzero((components==i)&core)>=.85*np.count_nonzero(core)]
            margin=16*(len(attempts)+1)  # Current 653x489 image rounds to16.
            x0=max(0,int(xs.min())-margin);x1=min(rgb.shape[1],int(xs.max())+margin+1)
            y0=max(0,int(ys.min())-margin);y1=min(rgb.shape[0],int(ys.max())+margin+1)
            assert patch.shape[:2]==(y1-y0,x1-x0)
            evidence=dict(margin_px=margin,box_xyxy=[x0,y0,x1,y1],matching_components=len(matching),trusted_core_pixels=int(np.count_nonzero(core)),roi_clipped_by_image=dict(left=x0==0,top=y0==0,right=x1==rgb.shape[1],bottom=y1==rgb.shape[0]))
            if len(matching)==1:
                chosen=components==matching[0];cy,cx=np.nonzero(chosen)
                boundary=dict(top=int(np.count_nonzero(chosen[3])),bottom=int(np.count_nonzero(chosen[-4])),left=int(np.count_nonzero(chosen[:,3])),right=int(np.count_nonzero(chosen[:,-4])))
                evidence.update(boundary_pixels=boundary,chosen_pixels=int(chosen.sum()),chosen_bbox_xyxy=[int(cx.min()+x0),int(cy.min()+y0),int(cx.max()+x0+1),int(cy.max()+y0+1)],image_edge_distance_px=[int(cx.min()+x0),int(cy.min()+y0),int(rgb.shape[1]-1-cx.max()-x0),int(rgb.shape[0]-1-cy.max()-y0)])
                full=np.zeros(rgb.shape[:2],np.uint8);full[y0:y1,x0:x1]=chosen.astype(np.uint8)*255
                evidence['selected_mask_sha256']=hashlib.sha256(full.tobytes()).hexdigest()
                cv2.imwrite(str(OUTPUT/f'{ns}_attempt{len(attempts)+1}_mask.png'),full)
            attempts.append(evidence)
        cv2.grabCut=observed
        try:
            refined=refine_candidate(rgb,depth,k,T,candidate,config,**kw);error=None
        except ValueError as exc:error=str(exc)
        finally:cv2.grabCut=original
        row=dict(source_stamp_ns=ns,recorded_error=('rgb_instance_reaches_search_boundary' if any(f['source_stamp_ns']==ns for f in failures) else 'not_a_published_evaluation_source'),reproduced_error=error,raw_candidate_bbox=list(candidate.bbox),raw_candidate_pixels=int(np.count_nonzero(candidate.mask)),raw_candidate_position_base=candidate.position_base.tolist(),raw_candidate_quality=float(candidate.quality),support_audit=seg.metrics,matching_original_identity_count=len(matches),attempts=attempts,T_base_camera_link=T.tolist())
        attempts=[];cv2.grabCut=observed;started=time.monotonic()
        try:
            confirmed,refined,staged,evidence=helper.observe(seg,rgb,depth,k,T,ns,**kw)
            row['third_bounded_search']=dict(passed=True,confirmed=confirmed,evidence=evidence,refined_metrics=refined.metrics,refined_bbox=list(refined.bbox),source_span_sec=(helper.source_stamps_ns[-1]-helper.source_stamps_ns[0])*1e-9)
            cv2.imwrite(str(OUTPUT/f'{ns}_48px_refined_mask.png'),refined.mask)
            if confirmed:helper.clear()
        except ValueError as exc:row['third_bounded_search']=dict(passed=False,reason=str(exc));helper.clear()
        finally:cv2.grabCut=original
        row['third_bounded_search'].update(attempts=attempts,wall_sec=time.monotonic()-started)
        try:
            current=table.segment(depth,k,T,config,target_position_base=anchor.position_base,plane_base=plane,target_footprint_base=anchor.support_footprint_base,fit_current_support=True)
            matches_current=table._matching_candidates(current.candidates,anchor,last,config)
            row['current_fit_original_rgb']=dict(matching_count=len(matches_current),metrics=current.metrics)
            if len(matches_current)==1:
                current_refined=refine_candidate(rgb,depth,k,T,matches_current[0][1],config,**kw)
                row['current_fit_original_rgb'].update(passed=True,bbox=list(current_refined.bbox),rgb_metrics=current_refined.metrics)
        except ValueError as exc:row.setdefault('current_fit_original_rgb',{}).update(passed=False,reason=str(exc))
        row['new_acquisition_center_distance_px']=float(np.linalg.norm(candidate.center_uv-np.array([rgb.shape[1]/2,rgb.shape[0]/2])))
        row['new_acquisition_center_limit_px']=min(rgb.shape[:2])*config.center_radius_fraction
        rows.append(row)
        cv2.imwrite(str(OUTPUT/f'{ns}_rgb.png'),rgb)
        cv2.imwrite(str(OUTPUT/f'{ns}_seed_mask.png'),candidate.mask)
        np.savez_compressed(OUTPUT/f'{ns}_source.npz',rgb=rgb,depth=depth,T_base_camera_link=T,seed_mask=candidate.mask)
        canvas=rgb.copy();contours,_=cv2.findContours(candidate.mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE);cv2.drawContours(canvas,contours,-1,(0,255,0),1)
        for i,attempt in enumerate(attempts):
            x0,y0,x1,y1=attempt['box_xyxy'];cv2.rectangle(canvas,(x0,y0),(x1-1,y1-1),(0,255,255) if i==0 else (0,0,255),1)
            cv2.putText(canvas,f'ROI{i+1}',(x0,max(15,y0-3)),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,255,255) if i==0 else (0,0,255),1)
        cv2.imwrite(str(OUTPUT/f'{ns}_diagnostic_overlay.png'),canvas)
        print(json.dumps(dict(source=ns,original_error=error,trial48_passed=row['third_bounded_search']['passed'],trial48_error=row['third_bounded_search'].get('reason'),current_fit_passed=row['current_fit_original_rgb'].get('passed'),current_fit_error=row['current_fit_original_rgb'].get('reason')),ensure_ascii=False),flush=True)
    report=dict(scope='offline same-frame diagnostic; original segmentation and RGB guards unchanged',execution_authority=False,record=str(RECORD),anchor_source_ns=anchor_ns,pre_motion_source_ns=last_ns,anchor_position_extent_plane_reproduced=True,last_ready_refined_mask_bit_equal=True,camera={k:camera[k] for k in keys},config=asdict(config),config_scope='original pure Config defaults; reconstructed anchor, plane and last-ready RGB mask independently agree with sealed record',reported_boundary_failures=len(failures),selected_frames=rows,overlay_legend='green: raw measured seed; yellow: original 16px search; red: same-source32px retry',limitations=['No source is republished; this does not revive a lost target or grant execution authority.'])
    (OUTPUT/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print('OUTPUT',OUTPUT,flush=True)

if __name__=='__main__':main()
