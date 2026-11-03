"""Closed-bag exact-source segmentation diagnostic. No ROS master/control."""
import inspect,json,hashlib,shutil
from pathlib import Path
from dataclasses import replace
from types import SimpleNamespace
import numpy as np,cv2,rosbag,rospy,tf2_ros
from cv_bridge import CvBridge
from tf.transformations import quaternion_matrix
from alicia_grasp_modes import rgb_instance
from alicia_grasp_modes.tabletop import Config,segment_with_support_fallback,_matching_candidates
ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log')
SESSION=ROOT/'feedforward_speed_grasp_20260929T065233Z'
BAG=SESSION/'scene_diagnostic.bag'
FORMAL=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20260929T065234.384593Z_e51bc4ad93b94214b68131188cc923df/keyframes.bag')
CONTEXT=ROOT/'fresh_target_20260929T065152Z/context.json'
OUT=SESSION/'offline_rgb_boundary_audit'

def tfmat(m):
 q=m.transform.rotation;t=m.transform.translation;T=quaternion_matrix([q.x,q.y,q.z,q.w]);T[:3,3]=[t.x,t.y,t.z];return T

def main():
 cv2.setNumThreads(1);OUT.mkdir(exist_ok=False)
 camera_rows=[]
 with rosbag.Bag(str(FORMAL)) as b:
  for topic,m,t in b.read_messages(topics=['/grasp_record/camera_runtime_metadata']):camera_rows.append(dict(receipt_ns=t.to_nsec(),metadata=json.loads(m.data)))
 camera=camera_rows[0]['metadata']['parameters'];keys=['fx','fy','cx','cy','depth_scale','width','height','align_depth_to_color']
 assert all(all(r['metadata']['parameters'][k]==camera[k] for k in keys) for r in camera_rows)
 (OUT/'same_attempt_camera_metadata.json').write_text(json.dumps(camera_rows,ensure_ascii=False,indent=2)+'\n')
 cfg=replace(Config(),center_radius_fraction=.20);K=[camera[k] for k in ['fx','fy','cx','cy']]
 c=json.loads(CONTEXT.read_text());s=c['current_status'];anchor=SimpleNamespace(position_base=np.array(s['anchor_position_base']),extent_m=np.array(s['anchor_extent_m']),support_footprint_base=np.array(s['reference_support_footprint_base']));plane=np.array(s['reference_support_plane_base'])
 buffer=tf2_ros.Buffer(cache_time=rospy.Duration(300),debug=False);statuses={};allstatus=[];rgbs={};depths={}
 with rosbag.Bag(str(BAG)) as b:
  for topic,m,t in b.read_messages(topics=['/tf','/tf_static','/perception/unknown/detector_status']):
   if topic in ['/tf','/tf_static']:
    for x in m.transforms:(buffer.set_transform_static if topic=='/tf_static' else buffer.set_transform)(x,'closed_bag')
   else:
    d=json.loads(m.data);allstatus.append(d);ns=d.get('source_stamp_ns')
    if ns and 1790664804000000000<=ns<=1790664810000000000:statuses[ns]=d
 with rosbag.Bag(str(BAG)) as b:
  for topic,m,t in b.read_messages(topics=['/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw']):
   ns=m.header.stamp.to_nsec()
   if ns in statuses:(rgbs if 'color' in topic else depths)[ns]=m
 bridge=CvBridge();rows=[]
 original_source=inspect.getsource(rgb_instance.instance_mask_from_seed);refine_source=inspect.getsource(rgb_instance.refine_candidate)
 for ns in sorted(set(rgbs)&set(depths)):
  row=dict(source_stamp_ns=ns,recorded_detail=statuses[ns]['detail'],counterfactual_only=True)
  image=bridge.imgmsg_to_cv2(rgbs[ns],'bgr8');depth=bridge.imgmsg_to_cv2(depths[ns],'passthrough').astype(np.float32)*camera['depth_scale'];F=tfmat(buffer.lookup_transform_core('base_link','camera_link',rgbs[ns].header.stamp))
  # Original last raw depth candidate is not archived; use immutable anchor for
  # association. The depth/plane algorithms and unique candidate itself are exact.
  result=segment_with_support_fallback(depth,K,F,cfg,anchor=anchor,last=anchor,reference_plane_base=plane)
  matching=_matching_candidates(result.candidates,anchor,anchor,cfg);row.update(candidates=len(result.candidates),unique_anchor_matches=len(matching),support_metrics=result.metrics)
  if len(matching)!=1:row['error']='no_unique_anchor_match';rows.append(row);continue
  chosen=matching[0][1];row.update(seed_bbox=list(chosen.bbox),seed_pixels=int(np.count_nonzero(chosen.mask)),seed_extent_mm=(chosen.extent_m*1000).tolist(),anchor_distance_mm=matching[0][0]*1000)
  cv2.imwrite(str(OUT/(str(ns)+'_raw.png')),image);cv2.imwrite(str(OUT/(str(ns)+'_depth_seed.png')),chosen.mask)
  trials=[]
  for margin in [16,24,32,48]:
   diag={};scope=rgb_instance.__dict__.copy();scope['diagnostic']=diag
   source=original_source.replace('16 * min(seed.shape[1] / 640., seed.shape[0] / 480.)',str(margin)+' * min(seed.shape[1] / 640., seed.shape[0] / 480.)')
   source=source.replace('    chosen = components == matching[0]','    chosen = components == matching[0]\n    diagnostic.update(roi=[x0,y0,x1,y1], chosen=chosen.copy(), core=core.copy(), support=support.copy(), labels=labels.copy())')
   exec(compile(source,'<offline_counterfactual_roi>','exec'),scope);exec(compile(refine_source,'<unchanged_refine_candidate>','exec'),scope)
   trial=dict(search_margin_px=margin,all_other_refinement_checks_unchanged=True)
   try:
    refined=scope['refine_candidate'](image,depth,K,F,chosen,cfg);trial.update(passed=True,refined_bbox=list(refined.bbox),refined_extent_mm=(refined.extent_m*1000).tolist(),metrics=refined.metrics);cv2.imwrite(str(OUT/(str(ns)+'_margin%d_mask.png'%margin)),refined.mask)
   except ValueError as e:trial.update(passed=False,error=str(e))
   if diag:
    x0,y0,x1,y1=diag['roi'];mask=diag['chosen'];sl=depth[y0:y1,x0:x1];boundary=np.zeros(mask.shape,bool);boundary[3]=mask[3];boundary[-4]=mask[-4];boundary[:,3]|=mask[:,3];boundary[:,-4]|=mask[:,-4]
    trial.update(search_box_xyxy=diag['roi'],selected_rgb_pixels=int(mask.sum()),trusted_core_pixels=int(diag['core'].sum()),touch_sides=dict(top=int(mask[3].sum()),bottom=int(mask[-4].sum()),left=int(mask[:,3].sum()),right=int(mask[:,-4].sum())),boundary_selected_pixels=int(boundary.sum()),boundary_invalid_depth_pixels=int(np.count_nonzero(boundary&((sl<cfg.depth_min_m)|~np.isfinite(sl)))))
    overlay=image.copy();cv2.rectangle(overlay,(x0,y0),(x1-1,y1-1),(255,150,0),1);contours,_=cv2.findContours(mask.astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE);shifted=[p+np.array([[[x0,y0]]]) for p in contours];cv2.drawContours(overlay,shifted,-1,(255,0,255),1);core_ys,core_xs=np.nonzero(diag['core']);overlay[core_ys+y0,core_xs+x0]=[0,255,255];cv2.imwrite(str(OUT/(str(ns)+'_margin%d_overlay.png'%margin)),overlay)
   trials.append(trial)
  row['trials']=trials;rows.append(row);print(json.dumps({k:row[k] for k in ['source_stamp_ns','recorded_detail','seed_bbox','seed_pixels','seed_extent_mm','unique_anchor_matches','trials']},ensure_ascii=False),flush=True)
 report=dict(scope='exact_RGBD_TF_closed_bag_diagnostic_only',production_changed=False,motion_authorized=False,ros_master_used=False,
  camera_metadata_source=str(FORMAL),camera_parameter_reads=len(camera_rows),camera_relevant_fields_unchanged=True,
  source_bag=str(BAG),context=str(CONTEXT),window_detector_sources=len(statuses),same_source_archived_rgbd=len(rows),rows=rows,
  source_implementation_sha256=hashlib.sha256(Path(rgb_instance.__file__).read_bytes()).hexdigest(),replay_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
  limitations=['Only four of 26 detector sources have same-source RGB-D archived by this recorder; no substituted nearest images.',
   'Original last raw-depth candidate is not archived; anchor used for association only, with unique eligible candidate required.',
   'Counterfactual margins change search crop only; they do not authorize masks or alter identity/contact/support gates.',
   'Camera metadata is same-attempt parameter evidence, not itself an exact image-time camera liveness claim.'])
 (OUT/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');shutil.copy2(__file__,OUT/Path(__file__).name)
if __name__=='__main__':main()
