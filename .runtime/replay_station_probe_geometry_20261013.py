#!/usr/bin/env python3
"""Full-rate new start-pose frames; original offline geometry, no old lease."""
import argparse
from collections import Counter
from dataclasses import asdict
import importlib.util
import json
import time
from pathlib import Path
import cv2
import numpy as np
import rosbag
import rospy
import tf2_py
from cv_bridge import CvBridge

W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
spec=importlib.util.spec_from_file_location('sealed_phase_inputs',W/'.runtime/replay_attempt1_first_phase_sources_20261013.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
from alicia_grasp_modes.metric_ambiguity_identity import metric_seeded_identity_evidence
from alicia_grasp_modes.acquisition import InitialAcquisition

def main():
    p=argparse.ArgumentParser();p.add_argument('--probe',required=True);p.add_argument('--fixture',required=True);p.add_argument('--output',required=True);a=p.parse_args()
    source=Path(a.probe);old=Path(a.fixture);out=Path(a.output);out.mkdir(parents=True,exist_ok=False)
    cv2.setNumThreads(1)
    receipt=json.loads((source/'result.json').read_text());assert receipt['closed'] is True
    assert m.sha(source/'full_frames.bag')==receipt['bag_sha256']
    context=json.loads((source/'context.json').read_text());camera=context['parameters']['/camera']
    fixture=json.loads((old/'fixture.json').read_text());config=m.table.Config(**fixture['config'])
    for key in ('fx','fy','cx','cy','depth_scale','width','height'):assert camera[key]==fixture['camera'][key]
    intrinsics=[camera[key] for key in ('fx','fy','cx','cy')]
    def oldframe(ns):
        f=np.load(old/f'{ns}_source.npz');return f['rgb'],f['depth'],f['T_base_camera'],{'rgb_valid':f['rgb_valid']} if 'rgb_valid' in f else {},f
    anchor_ns=fixture['frames'][0]['source_stamp_ns'];rgb,depth,T,kw,data=oldframe(anchor_ns)
    seg=m.table.segment_with_support_fallback(depth,intrinsics,T,config);tracker=m.table.TargetTracker(config)
    anchor,reason=tracker.choose(seg,depth.shape);assert anchor is not None,reason
    plane=seg.plane_base.copy();spatial=m.freeze_observation_spatial_reference(anchor,plane)
    rgb,depth,T,kw,data=oldframe(m.PRE)
    premask=data['recorded_mask'];hist,contour=m._appearance(rgb,m._mask(premask,rgb.shape[:2]))
    frozen=m.FrozenObservationAppearance(rgb,premask,kw.get('rgb_valid'),m.PRE,hist,contour)
    seg=m.table.segment_with_support_fallback(depth,intrinsics,T,config,anchor,anchor,plane)
    matches=m.table._matching_candidates(seg.candidates,anchor,anchor,config);assert len(matches)==1
    last=matches[0][1]
    rgb,depth,T,kw,data=oldframe(m.GOOD)
    seg=m.table.segment_with_support_fallback(depth,intrinsics,T,config,anchor,last,plane,prefer_current_support=True)
    seg,pair=m.recover_arrival_matching_fragments(seg,rgb,depth,intrinsics,T,config,anchor,last,spatial,hist,contour,.5,appearance_context=dict(reference=frozen,source_ns=m.GOOD,deadline_monotonic=time.monotonic()+5,check_current=lambda:None),**kw)
    assert pair is not None and np.array_equal(pair[1].mask,data['recorded_mask'])
    last,good=pair;good_rgb=rgb.copy()
    images={};static=[];dynamic=[];sdk=[]
    topics=['/tf','/tf_static','/alicia_d/accepted_joint_states','/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw']
    with rosbag.Bag(str(source/'full_frames.bag')) as bag:
        for topic,msg,received in bag.read_messages(topics=topics):
            if topic=='/tf':dynamic.extend(msg.transforms)
            elif topic=='/tf_static':static.extend(msg.transforms)
            elif topic.endswith('accepted_joint_states'):
                sdk.append(dict(received_ns=received.to_nsec(),stamp_ns=msg.header.stamp.to_nsec(),name=list(msg.name),position=list(msg.position),frame_id=msg.header.frame_id))
            else:images.setdefault(msg.header.stamp.to_nsec(),{})['rgb' if '/color/' in topic else 'depth']=msg
    buf=tf2_py.BufferCore(rospy.Duration(60))
    for msg in static:buf.set_transform_static(msg,'new_stationary_probe')
    for msg in sorted(dynamic,key=lambda t:t.header.stamp.to_nsec()):buf.set_transform(msg,'new_stationary_probe')
    bridge=CvBridge();rows=[];fresh=InitialAcquisition(config);fresh_confirmations=0
    valid=m.rectification_rgb_valid_mask(camera,(camera['height'],camera['width']));kwargs={} if valid is None else {'rgb_valid':valid}
    for ns,pair in sorted(images.items()):
        row=dict(source_ns=ns,record='new_probe',historical_pixels_used_as_current=False,metric_geometry_authority=False,motion_authorized=False)
        started=time.perf_counter()
        try:
            assert set(pair)=={'rgb','depth'}
            rgb=bridge.imgmsg_to_cv2(pair['rgb'],'bgr8');raw=bridge.imgmsg_to_cv2(pair['depth'],'passthrough');depth=np.asarray(raw,np.float32)*camera['depth_scale']
            transform=buf.lookup_transform_core('base_link','camera_link',rospy.Time(ns//10**9,ns%10**9));T=m.transform_matrix(transform)
            before=max((x for x in sdk if x['stamp_ns']<=ns),key=lambda x:x['stamp_ns'],default=None)
            after=min((x for x in sdk if x['stamp_ns']>=ns),key=lambda x:x['stamp_ns'],default=None)
            row['sdk_bracket']=dict(before_ns=None if before is None else before['stamp_ns'],after_ns=None if after is None else after['stamp_ns'],span_ns=None if before is None or after is None else after['stamp_ns']-before['stamp_ns'])
            # A fresh generation must reacquire using its normal center rule;
            # this independent pipeline never borrows the historic target lock.
            try:
                initial_seg=m.table.segment_with_support_fallback(depth,intrinsics,T,config)
                fresh_tracker=m.table.TargetTracker(config);initial,reason=fresh_tracker.choose(initial_seg,depth.shape)
                row['fresh_acquisition']=dict(candidate_count=len(initial_seg.candidates),reason=reason)
                if initial is not None and initial.quality>=.5:
                    initial_refined=m.refine_candidate(rgb,depth,intrinsics,T,initial,config,**kwargs)
                    confirmed,detail=fresh.observe(initial,initial_refined,ns)
                    row['fresh_acquisition'].update(confirmed=confirmed,detail=detail,quality=initial_refined.quality,bbox=list(initial_refined.bbox),position_base=initial_refined.position_base)
                    fresh_confirmations+=int(confirmed)
                else:fresh.reset()
            except ValueError as exc:row['fresh_acquisition']=dict(error=str(exc));fresh.reset()
            # The historical anchor/appearance are diagnostic references only.
            # Its expired lease is never constructed or reactivated here.
            seg=m.table.segment_with_support_fallback(depth,intrinsics,T,config,anchor,last,plane,prefer_current_support=True)
            matching=m.table._matching_candidates(seg.candidates,anchor,last,config)
            row.update(candidate_count=len(seg.candidates),original_metric_matches=len(matching),support_source=seg.metrics.get('support_source'))
            audit=[];pair=None
            if len(matching)!=1:
                recovery=m.recover_arrival_matching_fragments if len(matching)>1 else m.recover_arrival_fragment_group
                try:
                    seg,pair=recovery(seg,rgb,depth,intrinsics,T,config,anchor,last,spatial,hist,contour,.5,appearance_context=dict(reference=frozen,source_ns=ns,deadline_monotonic=time.monotonic()+5,check_current=lambda:None,record=audit.append),**kwargs)
                    row['fragment_accepted']=pair is not None
                except ValueError as exc:row.update(fragment_accepted=False,fragment_reason=str(exc))
            else:
                candidate=matching[0][1];refined=m.refine_candidate(rgb,depth,intrinsics,T,candidate,config,observation_arrival_retry=True,observation_spatial_reference=spatial,**kwargs);pair=(candidate,refined);row['unique_original_metric']=True
            row['current_geometry_candidate_available']=pair is not None
            row['appearance_audits']=audit
            if pair is not None:
                row.update(refined_position_base=pair[1].position_base,refined_mask_sha256=__import__('hashlib').sha256(np.ascontiguousarray(pair[1].mask>0,np.uint8).tobytes()).hexdigest())
            elif len(matching)>1:
                row['independent_metric_identity']=m.capture(lambda:metric_seeded_identity_evidence(rgb,[x[1] for x in matching],good_rgb,good.mask,minimum_quality=.5,rgb_valid=valid,refine=lambda candidate:m.refine_candidate(rgb,depth,intrinsics,T,candidate,config,observation_arrival_retry=True,**kwargs))[1])
        except Exception as exc:row.update(error=str(exc),exception=type(exc).__name__)
        row['wall_sec']=time.perf_counter()-started;rows.append(row)
        if len(rows)%10==0:print(json.dumps(dict(evaluated=len(rows),geometry_available=sum(x.get('current_geometry_candidate_available',False) for x in rows),fresh_confirmations=fresh_confirmations)),flush=True)
    summary=dict(total=len(rows),geometry_available=sum(x.get('current_geometry_candidate_available',False) for x in rows),fragment_rejections=dict(Counter(x['fragment_reason'] for x in rows if 'fragment_reason'in x)),independent_identity_rejections=dict(Counter(x['independent_metric_identity'].get('reason','pass') for x in rows if 'independent_metric_identity'in x)),errors=dict(Counter(x['error'] for x in rows if 'error'in x)),fresh_acquisition_reasons=dict(Counter(x.get('fresh_acquisition',{}).get('error',x.get('fresh_acquisition',{}).get('reason','none')) for x in rows)),fresh_confirmations=fresh_confirmations)
    pose_comparison_path=source/'pose_comparison.json'
    result=dict(scope='new start-pose full-rate measured geometry diagnostic, not historical missing pixels or revived authority',probe=str(source),bag_sha256=receipt['bag_sha256'],old_fixture=str(old),config=asdict(config),summary=summary,rows=rows,historical_reference_source_ns=m.GOOD,pose_comparison=(json.loads(pose_comparison_path.read_text()) if pose_comparison_path.exists() else {'historical_same_pose_not_proven':True}),expired_lease_reactivated=False,last_good_renewed=False,original_predicates_unchanged=True,temperature_queried=False,motion_commands_sent=False,limitations=['The user repositioned the arm before this probe. Only fresh_acquisition describes current new-start acquisition; historic-reference columns cannot estimate the failed station failure rate.','Normal fresh-generation center acquisition and fixed historic-target geometry are evaluated separately.','Historic anchor and appearance are references only; no old lease, old READY, or receipt is refreshed.','CPU timings are offline diagnostic wall times, not substituted source or processing timestamps.','This geometry-only diagnostic cannot certify producer/remote/task registration or automatic motion readiness.'])
    (out/'result.json').write_text(json.dumps(m.plain(result),ensure_ascii=False,indent=2)+'\n');print('RESULT '+str(out/'result.json'));print(json.dumps(summary))
if __name__=='__main__':main()
