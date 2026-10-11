#!/usr/bin/env python3
"""Sealed gen12 actual spatial arrival helper replay; no ROS node or authority."""
from pathlib import Path
from dataclasses import asdict
import argparse, json, hashlib, time
import cv2, numpy as np, rosbag, rospy, tf2_py
from cv_bridge import CvBridge
from tf.transformations import quaternion_matrix
from alicia_grasp_modes import tabletop as table
from alicia_grasp_modes.rgb_instance import refine_candidate, freeze_observation_spatial_reference
from alicia_grasp_modes.observation_arrival_confirmation import ObservationArrivalConfirmation
from alicia_grasp_modes.rgb_observation_handoff import _appearance, _mask
from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask
ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
RECORD=ROOT/'grasp_records/20261009T042814.711798Z_02d7a2117e114e0ea476744c19fa64c4'
OUT=ROOT/'ros_log/full_latest_ros_20261009T003407Z/gen12_arrival_analysis/spatial_measured_support_replay'
def matrix(msg):
 t=msg.transform;q=t.rotation;m=quaternion_matrix([q.x,q.y,q.z,q.w]);m[:3,3]=[t.translation.x,t.translation.y,t.translation.z];return m
def main():
 global OUT
 parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path)
 args=parser.parse_args()
 analysis_path=OUT.parent/'analysis.json'
 if args.output is not None:OUT=args.output
 cv2.setNumThreads(1)
 assert json.loads((RECORD/'manifest.json').read_text())['recorder_closed'] is True
 analysis=json.loads(analysis_path.read_text())
 requests={r['message']['state']:r['message'] for r in analysis['requests']}
 executing=requests['executing']['stamp_ns'];arrival=analysis['arrival_ns'];deadline=analysis['capture_deadline_ns']
 images={};masks={};statuses=[];static=[];dynamic=[];cameras=[]
 topics=['/tf','/tf_static','/perception/unknown/detector_status','/grasp_record/camera_runtime_metadata','/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw','/perception/object_mask','/experiment/observation_rgbd/color','/experiment/observation_rgbd/depth']
 with rosbag.Bag(str(RECORD/'keyframes.bag')) as bag:
  for topic,msg,received in bag.read_messages(topics=topics):
   if topic=='/tf':dynamic.extend(msg.transforms)
   elif topic=='/tf_static':static.extend(msg.transforms)
   elif topic.endswith('camera_runtime_metadata'):
    data=json.loads(msg.data)
    if data.get('parameters'):cameras.append(data['parameters'])
   elif topic.endswith('detector_status'):
    data=json.loads(msg.data)
    if data.get('generation')==12:statuses.append((received.to_nsec(),data))
   else:
    ns=msg.header.stamp.to_nsec()
    if topic.endswith('object_mask'):masks[ns]=msg
    elif ns<executing or arrival<ns<deadline:images.setdefault(ns,{})['rgb' if '/color/' in topic or topic.endswith('/color') else 'depth']=msg
 acquired=next(r for _,r in statuses if r.get('detail')=='acquired');anchor_ns=acquired['anchor_source_stamp_ns']
 ready=[r['source_stamp_ns'] for received,r in statuses if received<executing and r.get('state')=='ready']
 last_ns=max(ns for ns in ready if ns in masks and set(images.get(ns,{}))=={'rgb','depth'})
 selected=sorted(ns for ns,pair in images.items() if arrival<ns<deadline and set(pair)=={'rgb','depth'})
 recorded={r['source_stamp_ns']:r.get('observation_measurement',{}).get('reason') for _,r in statuses if r.get('source_stamp_ns') and arrival<r['source_stamp_ns']<deadline}
 assert selected and set(images[anchor_ns])=={'rgb','depth'}
 buf=tf2_py.BufferCore(rospy.Duration(600.))
 for t in static:buf.set_transform_static(t,'sealed_gen12')
 for t in sorted(dynamic,key=lambda t:t.header.stamp.to_nsec()):buf.set_transform(t,'sealed_gen12')
 camera=cameras[0];keys=('fx','fy','cx','cy','width','height','depth_scale')
 assert all(all(c[k]==camera[k] for k in keys) for c in cameras)
 k=[camera[name] for name in ('fx','fy','cx','cy')];bridge=CvBridge();config=table.Config()
 def frame(ns):
  rgb=bridge.imgmsg_to_cv2(images[ns]['rgb'],'bgr8');raw=bridge.imgmsg_to_cv2(images[ns]['depth'],'passthrough')
  depth=np.asarray(raw,dtype=np.float32)*camera['depth_scale'];T=matrix(buf.lookup_transform_core('base_link','camera_link',rospy.Time(ns//10**9,ns%10**9)))
  valid=rectification_rgb_valid_mask(camera,depth.shape)
  return rgb,depth,T,({} if valid is None else {'rgb_valid':valid})
 rgb,depth,T,kw=frame(anchor_ns);seg=table.segment_with_support_fallback(depth,k,T,config)
 tracker=table.TargetTracker(config);anchor,reason=tracker.choose(seg,depth.shape);assert anchor is not None,reason
 assert np.allclose(anchor.position_base,acquired['anchor_position_base'],atol=1e-10,rtol=0)
 assert np.allclose(anchor.extent_m,acquired['anchor_extent_m'],atol=1e-10,rtol=0)
 plane=np.asarray(acquired['reference_support_plane_base']);assert np.allclose(seg.plane_base,plane,atol=1e-10,rtol=0)
 rgb,depth,T,kw=frame(last_ns);seg=table.segment_with_support_fallback(depth,k,T,config,anchor,anchor,plane)
 matching=table._matching_candidates(seg.candidates,anchor,anchor,config);assert len(matching)==1
 last=matching[0][1];refined=refine_candidate(rgb,depth,k,T,last,config,**kw)
 mask=bridge.imgmsg_to_cv2(masks[last_ns],'mono8');assert np.array_equal(refined.mask,mask)
 hist,contour=_appearance(rgb,_mask(mask,rgb.shape[:2]));helper=ObservationArrivalConfirmation(config,anchor,last,plane,hist,contour,.5)
 OUT.mkdir(parents=True,exist_ok=True)
 cv2.imwrite(str(OUT/'reference_rgb.png'),rgb);cv2.imwrite(str(OUT/'reference_mask.png'),mask)
 reference_rgb=rgb.copy();reference_mask=mask.copy()
 np.savez_compressed(OUT/'reference.npz',rgb=rgb,mask=mask,histogram=hist,contour=contour,plane=plane,T_base_camera=T)
 rows=[]
 for ns in selected:
  rgb,depth,T,kw=frame(ns);started=time.perf_counter()
  row=dict(source_stamp_ns=ns,recorded_reason=recorded.get(ns,'source_not_evaluated_online'))
  try:
   seg=table.segment_with_support_fallback(depth,k,T,config,anchor,last,plane,prefer_current_support=True)
   matches=table._matching_candidates(seg.candidates,anchor,last,config)
   row.update(segmentation_candidates=len(seg.candidates),original_identity_matches=len(matches),support_audit=seg.metrics)
   row['raw_candidates']=[dict(bbox=list(c.bbox),quality=c.quality,position_base=c.position_base.tolist(),extent_m=c.extent_m.tolist()) for c in seg.candidates]
   if len(matches)==1:
    raw=matches[0][1];candidate=refine_candidate(rgb,depth,k,T,raw,config,observation_arrival_retry=True,observation_spatial_reference=helper.spatial_reference,**kw)
    h,c=_appearance(rgb,_mask(candidate.mask,rgb.shape[:2]));colour=float(np.sqrt(max(0.,1.-np.sum(np.sqrt(h*hist)))));shape=float(cv2.matchShapes(contour,c,cv2.CONTOURS_MATCH_I1,0.))
    row.update(colour_distance=colour,contour_distance=shape,colour_pass=colour<=helper.policy.max_colour_distance,contour_pass=shape<=helper.policy.max_contour_distance,refined_bbox=list(candidate.bbox),refined_pixels=int(np.count_nonzero(candidate.mask)),refined_metrics=candidate.metrics)
    if helper.confirmation.samples:
     first,previous=helper.confirmation.samples[0],helper.confirmation.samples[-1]
     a,b=first[1].mask>0,candidate.mask>0
     continuity=dict(anchor_source_ns=first[2],previous_source_ns=previous[2],mask_iou=float(np.count_nonzero(a&b)/max(1,np.count_nonzero(a|b))),matching=bool(table._matching_candidates([raw],first[0],previous[0],config)))
     try:table._validate_support_plane_update(first[0].metrics['support_plane_base'],raw.metrics['support_plane_base'],first[0].position_base,config,first[0].support_footprint_base);continuity['support_update']='passed'
     except ValueError as exc:continuity['support_update']=str(exc)
     row['confirmation_continuity']=continuity
    cv2.imwrite(str(OUT/f'{ns}_refined_mask.png'),candidate.mask)
    overlay=rgb.copy();cv2.drawContours(overlay,cv2.findContours(candidate.mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2],-1,(0,255,0),2)
    cv2.imwrite(str(OUT/f'{ns}_overlay.png'),overlay)
    np.savez_compressed(OUT/f'{ns}_source.npz',rgb=rgb,depth=depth,T_base_camera=T,raw_mask=raw.mask,refined_mask=candidate.mask,histogram=h,contour=c)
   try:
    confirmed,_,_,evidence=helper.observe(seg,rgb,depth,k,T,ns,**kw)
    row.update(actual_helper_passed=True,confirmed=confirmed,evidence=evidence)
   except ValueError as exc:row.update(actual_helper_passed=False,helper_reason=str(exc))
  except ValueError as exc:
   row.update(actual_helper_passed=False,helper_reason=str(exc));helper.clear()
  row['wall_sec']=time.perf_counter()-started;rows.append(row)
  cv2.imwrite(str(OUT/f'{ns}_rgb.png'),rgb)
  print(json.dumps({key:row.get(key) for key in ('source_stamp_ns','recorded_reason','helper_reason','colour_distance','contour_distance','original_identity_matches','confirmation_continuity')}),flush=True)
 sources=[Path(__file__),Path(table.__file__),Path('src/alicia_grasp_modes/src/alicia_grasp_modes/rgb_instance.py'),Path('src/alicia_grasp_modes/src/alicia_grasp_modes/observation_arrival_confirmation.py')]
 report=dict(record=str(RECORD),scope='sealed measured source offline diagnostic; no execution authority',no_ros_calls=True,anchor_ns=anchor_ns,last_ready_ns=max(ready),reference_ns=last_ns,reference_is_last_ready=last_ns==max(ready),reference_mask_bit_equal=True,original_anchor_plane_reproduced=True,policy=asdict(helper.policy),config=asdict(config),available_rgbd_count=len(selected),evaluated_online_source_count=len(recorded),online_sources_missing_rgbd=sorted(set(recorded)-set(selected)),rows=rows,source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},limitations=['Only sealed available exact RGB-D sources were replayed; sources absent from keyframes are listed.','Anchor/plane reconstructed with original Config defaults and verified against recorded anchor/plane; last-ready reconstructed RGB mask is bit equal.','No target is reactivated, no input source is republished, and no execution authority is obtained.','Actual helper and current-support segmentation are used. This is a content/identity replay without live callback timing, deadline, joint feedback, or execution authority.'])
 (OUT/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
 print('RESULT '+str(OUT/'result.json'),flush=True)
if __name__=='__main__':main()
