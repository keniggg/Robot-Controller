#!/usr/bin/env python3
"""Exact sealed pixels/TF; compare existing default versus arrival RGB search only."""
from pathlib import Path
from dataclasses import asdict
import json,hashlib,time
import numpy as np
import cv2,rosbag,rospy,tf2_py
from cv_bridge import CvBridge
from tf.transformations import quaternion_matrix
from alicia_grasp_modes.tabletop import Config,TargetTracker,segment_with_support_fallback,_matching_candidates,_project
from alicia_grasp_modes.measured_mask_seeds import confidence_labels
from alicia_grasp_modes.rgb_instance import refine_candidate
from alicia_grasp_modes.metric_ambiguity_identity import metric_seeded_identity_evidence
from alicia_grasp_modes.rgb_observation_handoff import _appearance,HandoffPolicy
from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask
BASE=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller');NEW=BASE/'ros_log/full_latest_ros_20261009T003407Z';REC=BASE/'grasp_records/20261009T052127.805554Z_6b8770587c5c4cd8abc679178eaa1c70';OUT=NEW/'gen15_handoff_analysis/support_gap_rgb_replay_details'
summary=json.loads((NEW/'gen15_handoff_analysis/arrivals_and_terminal.json').read_text());lost=summary['first_loss']['message'];initial=lost['anchor_source_stamp_ns'];good=lost['first_loss']['last_good_source_stamp_ns'];bad=lost['source_stamp_ns'];gap=1791523444091633558;needed={initial,good,bad,gap};images={};masks={};static=[];dynamic=[];cameras=[];statuses={};feedback=[]
with rosbag.Bag(str(REC/'keyframes.bag')) as b:
 for topic,m,t in b.read_messages(topics=['/tf','/tf_static','/grasp_record/camera_runtime_metadata','/perception/unknown/detector_status','/alicia_d/accepted_joint_states','/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw','/perception/object_mask','/experiment/observation_rgbd/color','/experiment/observation_rgbd/depth']):
  if topic=='/tf':dynamic.extend(m.transforms)
  elif topic=='/tf_static':static.extend(m.transforms)
  elif topic.endswith('camera_runtime_metadata'):
   x=json.loads(m.data)
   if x.get('parameters'):cameras.append((t.to_nsec(),x['parameters']))
  elif topic.endswith('detector_status'):
   x=json.loads(m.data)
   if x.get('source_stamp_ns') in needed:statuses[x['source_stamp_ns']]=x
  elif topic.endswith('accepted_joint_states'):
   if m.header.frame_id=='sdk_measured' and good-800_000_000<=m.header.stamp.to_nsec()<=bad+500_000_000:feedback.append(dict(stamp_ns=m.header.stamp.to_nsec(),names=list(m.name),positions=list(m.position)))
  else:
   ns=m.header.stamp.to_nsec()
   if ns not in needed:continue
   if topic.endswith('object_mask'):masks[ns]=m
   else:images.setdefault(ns,{})['rgb' if '/color/' in topic or topic.endswith('/color') else 'depth']=m
assert all(set(images.get(ns,{}))=={'rgb','depth'} for ns in needed),{ns:list(x) for ns,x in images.items()}
bridge=CvBridge();buffer=tf2_py.BufferCore(rospy.Duration(600.));cv2.setNumThreads(1)
for t in static:buffer.set_transform_static(t,'sealed_gen15')
for t in sorted(dynamic,key=lambda x:x.header.stamp.to_nsec()):buffer.set_transform(t,'sealed_gen15')
def stamp(ns):return rospy.Time(ns//10**9,ns%10**9)
def matrix(msg):
 t=msg.transform;q=t.rotation;a=quaternion_matrix([q.x,q.y,q.z,q.w]);a[:3,3]=[t.translation.x,t.translation.y,t.translation.z];return a
camera=min(cameras,key=lambda x:abs(x[0]-bad))[1];K=[camera[k] for k in ('fx','fy','cx','cy')];cfg=Config();plane=np.array(lost['reference_support_plane_base'])
def decoded(ns):
 a=images[ns];rgb=bridge.imgmsg_to_cv2(a['rgb'],'bgr8');raw=bridge.imgmsg_to_cv2(a['depth'],'passthrough');depth=np.asarray(raw,np.float32)*camera['depth_scale'];T=matrix(buffer.lookup_transform_core('base_link','camera_link',stamp(ns)));return rgb,raw,depth,T
rgb,raw,dep,T=decoded(initial);seg=segment_with_support_fallback(dep,K,T,cfg);tracker=TargetTracker(cfg);anchor,reason=tracker.choose(seg,dep.shape);assert anchor is not None,reason
assert np.allclose(anchor.position_base,lost['anchor_position_base'],rtol=0,atol=1e-10),(anchor.position_base,lost['anchor_position_base'])
assert np.allclose(anchor.extent_m,lost['anchor_extent_m'],rtol=0,atol=1e-10)
recorded_footprint=np.asarray(lost['reference_support_footprint_base']); reconstructed_footprint=np.asarray(anchor.support_footprint_base)
print(json.dumps(dict(anchor_footprint_shapes=[list(reconstructed_footprint.shape),list(recorded_footprint.shape)],anchor_footprint_max_difference=float(np.max(np.abs(reconstructed_footprint-recorded_footprint))) if reconstructed_footprint.shape==recorded_footprint.shape else None)),flush=True)
assert np.allclose(reconstructed_footprint,recorded_footprint,rtol=0,atol=1e-10),'reconstructed anchor footprint must match recorded immutable footprint to 1e-10 m'
anchor.support_footprint_base=recorded_footprint.copy()
tracker.plane_base=plane.copy();rgb_ref,raw,dep,T=decoded(good);seg=segment_with_support_fallback(dep,K,T,cfg,anchor,tracker.last,plane);candidate,reason=tracker.choose(seg,dep.shape);assert candidate is not None,reason
refined_ref=refine_candidate(rgb_ref,dep,K,T,candidate,cfg,observation_arrival_retry=True,rgb_valid=rectification_rgb_valid_mask(camera,dep.shape));refmask=bridge.imgmsg_to_cv2(masks[good],'mono8');assert np.array_equal(refined_ref.mask,refmask),'last-good reconstructed mask must match bitwise'
good_candidate=candidate; good_depth=dep.copy();good_T=T.copy();good_raw=raw.copy();good_segmentation_metrics=seg.metrics.copy()
last=tracker.last;rowgap={}
try:
 rgb,raw,dep,T=decoded(gap);sg=segment_with_support_fallback(dep,K,T,cfg,anchor,last,plane);rowgap=dict(result='segmented',metrics=sg.metrics)
except ValueError as e:rowgap=dict(result='rejected',reason=str(e))
rgb,raw,dep,T=decoded(bad);sg=segment_with_support_fallback(dep,K,T,cfg,anchor,last,plane);matches=_matching_candidates(sg.candidates,anchor,last,cfg);staged=TargetTracker(cfg);staged.anchor,staged.last,staged.plane_base=anchor,last,plane.copy();candidate,reason=staged.choose(sg,dep.shape);assert candidate is not None and reason=='tracked',reason
OUT.mkdir(parents=True,exist_ok=True);cv2.imwrite(str(OUT/'last_good_rgb.png'),rgb_ref);cv2.imwrite(str(OUT/'last_good_mask.png'),refmask);cv2.imwrite(str(OUT/'failed_source_rgb.png'),rgb);cv2.imwrite(str(OUT/'failed_source_depth_seed.png'),candidate.mask)
np.savez_compressed(OUT/'inputs.npz',reference_rgb=rgb_ref,reference_mask=refmask,reference_depth_raw=good_raw,reference_T_base_camera=good_T,reference_candidate_mask=good_candidate.mask,current_rgb=rgb,current_depth_raw=raw,current_T_base_camera=T,K=K,plane=plane,anchor_position=anchor.position_base,anchor_extent=anchor.extent_m,last_position=last.position_base,current_candidate_mask=candidate.mask)
rows=[];valid=rectification_rgb_valid_mask(camera,dep.shape);policy=HandoffPolicy();rh,rc=_appearance(rgb_ref,refmask)
for flag in [False,True]:
 began=time.perf_counter();refined=refine_candidate(rgb,dep,K,T,candidate,cfg,observation_arrival_retry=flag,rgb_valid=valid);mask=refined.mask;h,c=_appearance(rgb,mask);colour=float(np.sqrt(max(0.,1.-np.sum(np.sqrt(h*rh)))));shape=float(cv2.matchShapes(rc,c,cv2.CONTOURS_MATCH_I1,0.));iou=float(np.count_nonzero((refmask>0)&(mask>0))/np.count_nonzero((refmask>0)|(mask>0)))
 try:
  rr,ev=metric_seeded_identity_evidence(rgb,[candidate],rgb_ref,refmask,minimum_quality=.5,rgb_valid=valid,refine=lambda x:refine_candidate(rgb,dep,K,T,x,cfg,observation_arrival_retry=flag,rgb_valid=valid));accepted=True;error=None
 except ValueError as e:accepted=False;error=str(e);ev=None
 cv2.imwrite(str(OUT/('refined_arrival.png' if flag else 'refined_default.png')),mask)
 rows.append(dict(arrival_retry=flag,accepted=accepted,error=error,identity=ev,mask_pixels=int(np.count_nonzero(mask)),mask_iou=iou,colour_distance=colour,contour_distance=shape,quality=refined.quality,metrics=refined.metrics,wall_sec=time.perf_counter()-began))
def box(mask):
 y,x=np.nonzero(mask);return [int(x.min()),int(y.min()),int(x.max())+1,int(y.max())+1]
def geom(colour,depth,tf,c):
 points=_project(depth,K);p=tf.T@np.asarray(c.metrics['support_plane_base']);h=np.einsum('ijk,k->ij',points,p[:3])+p[3]
 y,x=np.nonzero(c.mask);x0=max(0,int(x.min())-16);y0=max(0,int(y.min())-16);x1=min(depth.shape[1],int(x.max())+17);y1=min(depth.shape[0],int(y.max())+17);sl=np.s_[y0:y1,x0:x1]
 core,support,evidence=confidence_labels(c.mask[sl]>0,points[sl],depth[sl],h[sl],p[:3],cfg,bgr=colour[sl]);fullcore=np.zeros(depth.shape,bool);fullcore[sl]=core
 return dict(points=points,heights=h,core=fullcore,evidence=evidence,seed_bbox=box(c.mask),first_box=[x0,y0,x1,y1])
goodgeom=geom(rgb_ref,good_depth,good_T,good_candidate);badgeom=geom(rgb,dep,T,candidate)
removed=(refmask>0)&~(mask>0);added=(mask>0)&~(refmask>0)
def quantify(region,depth,geo):
 valid=np.isfinite(depth)&(depth>=cfg.depth_min_m)&(depth<=cfg.depth_max_m);v=geo['heights'][region&valid]
 return dict(pixels=int(np.count_nonzero(region)),valid_depth=int(np.count_nonzero(region&valid)),height_mm_quantiles=np.percentile(v,[0,5,25,50,75,95,100]).tolist() if len(v) else [],height_quantiles_units='m despite legacy key; corrected below',hard_core_pixels=int(np.count_nonzero(region&geo['core'])),support_pixels=int(np.count_nonzero(region&valid&(np.abs(geo['heights'])<=cfg.support_threshold_m))),foreground_height_pixels=int(np.count_nonzero(region&valid&(geo['heights']>=cfg.foreground_min_m)&(geo['heights']<=cfg.foreground_max_m))))
differences={}
for label,region in [('reference_only',removed),('current_only',added)]:
 differences[label]={}
 for name,depth,geo in [('last_good',good_depth,goodgeom),('first_loss',dep,badgeom)]:
  q=quantify(region,depth,geo);q['height_mm_quantiles']=[1000*v for v in q['height_mm_quantiles']];q.pop('height_quantiles_units');differences[label][name]=q
for name,colour,c,ref,geo in [('last_good',rgb_ref,good_candidate,refmask,goodgeom),('first_loss',rgb,candidate,mask,badgeom)]:
 overlay=colour.copy()
 for m,col in [(c.mask,(255,0,0)),(ref,(0,255,0)),(geo['core'].astype(np.uint8)*255,(255,0,255))]:
  contours,_=cv2.findContours(m.astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE);cv2.drawContours(overlay,contours,-1,col,1)
 cv2.imwrite(str(OUT/(name+'_overlay.png')),overlay);cv2.imwrite(str(OUT/(name+'_overlay_crop.png')),overlay[160:325,245:400])
 cv2.imwrite(str(OUT/(name+'_hard_core.png')),geo['core'].astype(np.uint8)*255)
 cv2.imwrite(str(OUT/(name+'_depth_seed.png')),c.mask)
cv2.imwrite(str(OUT/'reference_only_pixels.png'),removed.astype(np.uint8)*255)
report=dict(record=str(REC),scope='exact failed source and original immutable anchor/reference plane with reconstructed last raw candidate; original RGB identity function on unchanged single candidate, varying only established arrival RGB flag',source_stamps=dict(anchor=initial,last_good=good,support_gap=gap,first_loss=bad),anchor_reconstruction=dict(position_extent_allclose_atol_m=1e-10,footprint_shape=list(recorded_footprint.shape),footprint_max_difference_m=float(np.max(np.abs(reconstructed_footprint-recorded_footprint))),footprint_used='exact recorded immutable footprint'),last_good_refined_mask_bitexact=True,last_good_refine_metrics=refined_ref.metrics,last_good_segmentation_metrics=good_segmentation_metrics,mask_bboxes=dict(reference=box(refmask),current=box(mask),reference_seed=goodgeom['seed_bbox'],current_seed=badgeom['seed_bbox']),mask_difference= differences,overlay_colours_bgr=dict(depth_seed=[255,0,0],refined_outline=[0,255,0],hard_core=[255,0,255]),gap_replay=rowgap,failure_frame_support_metrics=sg.metrics,failure_candidate_count=len(sg.candidates),failure_matching_count=len(matches),original_tracker_choice=reason,failed_source_status=lost,last_good_source_status=statuses[good],rows=rows,camera=camera,accepted_feedback=feedback,configuration=asdict(cfg),production_modified=False,no_ros_calls=True,source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path('src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py'),Path('src/alicia_grasp_modes/src/alicia_grasp_modes/rgb_instance.py'),Path('src/alicia_grasp_modes/src/alicia_grasp_modes/metric_ambiguity_identity.py')]},limitations=['Pure original segmentation/refinement/identity replay; full node pending/marker/source lease guards not yet executed by this script.','The flag counterfactual alone grants no recovery or motion authority; production must independently validate the immutable same-station marker and all original source/support/identity gates.','No offline mocked clock is used for this pure function comparison; timing is processing wall time, not live readiness.','Anchor position/extent reconstructed to 1e-10 m; recorded footprint explicitly used after 6.6e-17 m floating roundoff difference, not claimed bitwise reconstructed.'])
p=OUT/'result.json';p.write_text(json.dumps(report,ensure_ascii=False,indent=2,default=lambda a:a.tolist() if isinstance(a,np.ndarray) else a)+'\n');print(json.dumps(dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),gap_replay=rowgap,segmentation_candidates=len(sg.candidates),matching_candidates=len(matches),rows=[{k:r[k] for k in ['arrival_retry','accepted','error','mask_pixels','mask_iou','colour_distance','contour_distance','quality','wall_sec']} for r in rows]),ensure_ascii=False,indent=2))
