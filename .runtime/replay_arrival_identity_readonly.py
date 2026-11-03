#!/usr/bin/env python3
"""Offline original identity/RGB arrival replay; grants no runtime authority."""
import importlib.util,json,hashlib,time
from pathlib import Path
import numpy as np
import rosbag,rospy
from cv_bridge import CvBridge
from alicia_grasp_modes import tabletop as table
from alicia_grasp_modes.rgb_instance import refine_candidate
from alicia_grasp_modes.rgb_observation_handoff import _appearance,_mask
from alicia_grasp_modes.observation_arrival_confirmation import ObservationArrivalConfirmation
from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask
spec=importlib.util.spec_from_file_location('support_replay',Path(__file__).with_name('analyze_arrival_support_readonly.py'))
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
PLAN_SOURCE=1791507388408942222
SUBMIT=1791507410505713000

def main():
    old=a.read(a.RECORD/'keyframes.bag',True);new=a.read(a.OUTPUT/'arrival.bag')
    ready=[]
    with rosbag.Bag(str(a.RECORD/'keyframes.bag')) as bag:
        for _,msg,stamp in bag.read_messages(topics=['/perception/unknown/detector_status']):
            row=json.loads(msg.data)
            if row.get('state')=='ready' and row.get('generation')==3 and stamp.to_nsec()<SUBMIT:
                ready.append(row['source_stamp_ns'])
    reference_source=max(ready);print('Last ready before submission:',reference_source,flush=True)
    acquired=old[4];initial=acquired['source_stamp_ns'];camera=old[2][0]
    images={};masks={};bridge=CvBridge();cfg=table.Config();k=[camera[z] for z in ('fx','fy','cx','cy')]
    for path,needed in [(a.RECORD/'keyframes.bag',{initial,reference_source}),(a.OUTPUT/'arrival.bag',None)]:
        with rosbag.Bag(str(path)) as bag:
            for topic,msg,_ in bag.read_messages(topics=['/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw','/perception/object_mask']):
                ns=msg.header.stamp.to_nsec()
                if needed is not None and ns not in needed:continue
                if topic.endswith('object_mask'):masks[ns]=bridge.imgmsg_to_cv2(msg,'mono8');continue
                images.setdefault(ns,{})['rgb' if '/color/' in topic else 'depth']=msg
    def frame(ns,data):
        pair=images[ns];rgb=bridge.imgmsg_to_cv2(pair['rgb'],'bgr8');raw=bridge.imgmsg_to_cv2(pair['depth'],'passthrough')
        depth=np.asarray(raw,dtype=np.float32)*camera['depth_scale']
        T=a.matrix(data[3].lookup_transform_core('base_link','camera_link',rospy.Time(ns//10**9,ns%10**9)))
        valid=rectification_rgb_valid_mask(camera,depth.shape)
        return rgb,depth,T,({} if valid is None else {'rgb_valid':valid})
    rgb,depth,T,kw=frame(initial,old)
    first=table.segment_with_support_fallback(depth,k,T,cfg)
    tracker=table.TargetTracker(cfg);anchor,reason=tracker.choose(first,depth.shape)
    assert anchor is not None,reason
    assert np.allclose(anchor.position_base,acquired['anchor_position_base'],atol=1e-12,rtol=0),anchor.position_base
    assert np.allclose(anchor.extent_m,acquired['anchor_extent_m'],atol=1e-12,rtol=0)
    assert np.allclose(first.plane_base,acquired['reference_support_plane_base'],atol=1e-12,rtol=0)
    # Exact saved immutable reference; the reproducible fit is checked above.
    tracker.plane_base=np.asarray(acquired['reference_support_plane_base']);plane=tracker.plane_base.copy()
    rgb,depth,T,kw=frame(reference_source,old)
    before=table.segment_with_support_fallback(depth,k,T,cfg,anchor,anchor,plane)
    matches=table._matching_candidates(before.candidates,anchor,anchor,cfg);assert len(matches)==1
    last=matches[0][1];refined=refine_candidate(rgb,depth,k,T,last,cfg,**kw)
    mask=masks[reference_source]
    hist,contour=_appearance(rgb,_mask(mask,rgb.shape[:2]))
    helper=ObservationArrivalConfirmation(cfg,anchor,last,plane,hist,contour,.5)
    result=[]
    for ns in sorted(new[0]):
        started=time.monotonic()
        row=dict(source_stamp_ns=ns)
        try:
            rgb,depth,T,kw=frame(ns,new)
            seg=table.segment_with_support_fallback(depth,k,T,cfg,anchor,last,plane)
            matches=table._matching_candidates(seg.candidates,anchor,last,cfg)
            row.update(candidate_count=len(seg.candidates),original_matching_count=len(matches),support_audit=seg.metrics)
            if matches:
                c=matches[0][1];row.update(anchor_distance_m=float(np.linalg.norm(c.position_base-anchor.position_base)),last_distance_m=float(np.linalg.norm(c.position_base-last.position_base)),extent_ratios=(c.extent_m[:2]/anchor.extent_m[:2]).tolist())
            qualified,candidate,staged,evidence=helper.observe(seg,rgb,depth,k,T,ns,**kw)
            row.update(qualified=qualified,evidence=evidence,refined_quality=float(candidate.quality))
            # This is a diagnostic series: make successive disjoint 3-frame confirmations.
            if qualified:helper.clear()
        except Exception as exc:helper.clear();row.update(qualified=False,error=str(exc))
        row['processing_wall_sec']=time.monotonic()-started;result.append(row)
        print(json.dumps({key:row[key] for key in ('source_stamp_ns','original_matching_count','qualified','error') if key in row}),flush=True)
    report=dict(scope='offline counterfactual of arrival identity requalification; no lost latch or authority is restored',source_bag=str(a.RECORD/'keyframes.bag'),arrival_bag=str(a.OUTPUT/'arrival.bag'),immutable_anchor_source_ns=initial,pre_motion_rgb_source_ns=reference_source,action_submit_stamp_ns=SUBMIT,pre_motion_reference_basis='last recorded ready source before actual action submission; not a captured runtime private cache',pre_motion_mask_recorded=reference_source in masks,recorded_mask_equals_replayed_refined=bool(np.array_equal(mask,refined.mask)),anchor_position_reproduced=True,extent_reproduced=True,plane_reproduced=True,reference_plane=plane.tolist(),reference_anchor=anchor.position_base.tolist(),helper_source_sha256=a.sha(Path(__import__('alicia_grasp_modes.observation_arrival_confirmation',fromlist=['']).__file__)),tabletop_source_sha256=a.sha(Path(table.__file__)),source_bag_sha256=a.sha(a.RECORD/'keyframes.bag'),capture_bag_sha256=a.sha(a.OUTPUT/'arrival.bag'),frames=result,summary=dict(frames=len(result),unique_association_frames=sum(r.get('original_matching_count')==1 for r in result),original_rgb_and_geometry_pass_frames=sum('evidence' in r for r in result),three_frame_confirmations=sum(r['qualified'] for r in result)))
    output=a.OUTPUT/'arrival_identity_last_ready_replay.json'
    with output.open('x') as f:json.dump(report,f,indent=2,ensure_ascii=False);f.write('\n')
    print(json.dumps(dict(output=str(output),summary=report['summary'])),flush=True)
if __name__=='__main__':main()
