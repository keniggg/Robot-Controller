#!/usr/bin/env python3
"""Offline counterfactual acquisition radius; never creates a production lock."""
import argparse,json,hashlib,os,sys
from pathlib import Path
from dataclasses import asdict,replace
import numpy as np
import cv2,rosbag,rospy,tf2_ros
from cv_bridge import CvBridge
HERE=Path(__file__).resolve().parent;sys.path[:0]=[str(HERE),str(HERE.parent/'src/alicia_grasp_modes/src')]
from alicia_grasp_modes.tabletop import Config,TargetTracker,segment_with_support_fallback
from alicia_grasp_modes.rgb_instance import refine_candidate
from alicia_grasp_modes.acquisition import InitialAcquisition
from audit_production_lock_support import transform_matrix,audit_frame

def candidate(c,shape):
 return dict(pixel=c.center_uv.tolist(),center_distance_px=float(np.linalg.norm(c.center_uv-[shape[1]/2,shape[0]/2])),bbox=list(c.bbox),
             extent_mm=(c.extent_m*1000).tolist(),quality=c.quality,position_camera_m=c.position_camera.tolist(),position_base_m=c.position_base.tolist(),metrics=c.metrics)

def main():
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('session',type=Path);ap.add_argument('--output',required=True,type=Path);a=ap.parse_args()
 mount=Path('/media/zhuyupei/SanStand')
 if not os.path.ismount(mount) or mount not in a.output.resolve().parents:raise RuntimeError('external archive required')
 a.output.mkdir(parents=True,exist_ok=False)
 cam=json.loads((a.session/'camera_parameters.json').read_text());archive=json.loads((a.session/'report.json').read_text())
 images={};depths={};qualities={};boards={};statuses={};buffer=tf2_ros.Buffer(cache_time=rospy.Duration(120),debug=False)
 with rosbag.Bag(str(a.session/'evidence.bag')) as bag:
  for topic,m,t in bag.read_messages():
   if topic in ('/tf','/tf_static'):
    for tr in m.transforms:
     (buffer.set_transform_static if topic=='/tf_static' else buffer.set_transform)(tr,'offline_archive')
     if tr.header.frame_id=='camera_link' and tr.child_frame_id=='charuco_board':boards[tr.header.stamp.to_nsec()]=tr
   elif topic=='/charuco/quality':q=json.loads(m.data);qualities[q['source_stamp_ns']]=q
   elif topic=='/perception/unknown/detector_status':s=json.loads(m.data);statuses[s.get('source_stamp_ns')]=s
   elif topic=='/production_lock/unfiltered_diagnostic_rgb':images[m.header.stamp.to_nsec()]=m
   elif topic=='/production_lock/unfiltered_diagnostic_depth':depths[m.header.stamp.to_nsec()]=m
 bridge=CvBridge();cfg=Config();wide=replace(cfg,center_radius_fraction=.20);acquisition=InitialAcquisition(wide);rows=[]
 intrinsics=[cam[k] for k in ('fx','fy','cx','cy')]
 for ns in sorted(set(images)&set(depths)):
  row=dict(source_stamp_ns=ns,production_locked=False,counterfactual_only=True,observed_production_status=statuses.get(ns))
  try:
   rgbmsg=images[ns];depthmsg=depths[ns];image=bridge.imgmsg_to_cv2(rgbmsg,'bgr8');z16=bridge.imgmsg_to_cv2(depthmsg,'passthrough')
   if rgbmsg.header.frame_id!='camera_link' or depthmsg.header.frame_id!='camera_link' or depthmsg.encoding not in ('16UC1','mono16'):raise ValueError('exact aligned RGB-D schema differs')
   depth=np.asarray(z16,dtype=np.float32)*cam['depth_scale']
   F=transform_matrix(buffer.lookup_transform_core('base_link','camera_link',rgbmsg.header.stamp))
   result=segment_with_support_fallback(depth,intrinsics,F,cfg)
   row.update(plane_base=result.plane_base.tolist(),plane_metrics=result.metrics,candidates=[candidate(c,depth.shape) for c in result.candidates],T_base_camera=F.tolist())
   default,reason=TargetTracker(cfg).choose(result,depth.shape);selected,wreason=TargetTracker(wide).choose(result,depth.shape)
   row.update(default_selection=dict(reason=reason,chosen=None if default is None else candidate(default,depth.shape)),radius_020_selection=dict(reason=wreason,chosen=None if selected is None else candidate(selected,depth.shape)),inside_60px=int(sum(np.linalg.norm(c.center_uv-[320,240])<=60 for c in result.candidates)),inside_96px=int(sum(np.linalg.norm(c.center_uv-[320,240])<=96 for c in result.candidates)))
   overlay=image.copy();cv2.circle(overlay,(320,240),60,(0,180,255),1);cv2.circle(overlay,(320,240),96,(255,180,0),1)
   for i,c in enumerate(result.candidates):
    x,y,w,h=c.bbox;cv2.rectangle(overlay,(x,y),(x+w,y+h),(0,255,0),1);cv2.putText(overlay,str(i),(x,y-4),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,0),1)
   cv2.imwrite(str(a.output/(str(ns)+'_raw.png')),image);cv2.imwrite(str(a.output/(str(ns)+'_candidates.png')),overlay)
   if selected is not None:
    refined=refine_candidate(image,depth,intrinsics,F,selected,wide)
    confirmed,detail=acquisition.observe(selected,refined,ns)
    row.update(refined_candidate=candidate(refined,depth.shape),offline_acquisition_confirmed=confirmed,offline_acquisition_detail=detail)
    cv2.imwrite(str(a.output/(str(ns)+'_offline_mask.png')),refined.mask)
    contours,_=cv2.findContours(refined.mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE);cv2.drawContours(overlay,contours,-1,(255,0,255),1);cv2.imwrite(str(a.output/(str(ns)+'_refined.png')),overlay)
    row['offline_adjacent_support']='pending'
    if ns in qualities and ns in boards:
     try:
      from types import SimpleNamespace
      q=qualities[ns];maskmsg=bridge.cv2_to_imgmsg(refined.mask,'mono8');maskmsg.header=rgbmsg.header
      # Explicit offline hypothetical status, not passed to production or to
      # the production-lock evidence-completeness checker.
      hypothetical=dict(state='offline_counterfactual_candidate',target_locked=False,geometry_metrics=refined.metrics)
      m=dict(rgb=rgbmsg,depth=depthmsg,mask=maskmsg,quality=q,board=boards[ns],status=hypothetical,object=SimpleNamespace(label='offline_unknown_candidate'))
      row['offline_adjacent_support']=audit_frame(m,cam,F,a.output,ns)
     except Exception as error:row['offline_adjacent_support']={'error':str(error)}
    else:row['offline_adjacent_support']={'error':'same-source board quality/TF unavailable','quality_present':ns in qualities,'board_present':ns in boards}
   print(json.dumps({k:row.get(k) for k in ('source_stamp_ns','inside_60px','inside_96px','default_selection','radius_020_selection','offline_acquisition_confirmed','offline_acquisition_detail')},ensure_ascii=False),flush=True)
  except Exception as error:row['error']=str(error);print('REPLAY_ERROR',ns,error,flush=True)
  rows.append(row)
 report=dict(session=str(a.session),offline_only=True,production_locked=False,parameters_modified=False,motion_authorized=False,
             config_default=asdict(cfg),config_proposal=asdict(wide),only_config_change='center_radius_fraction: 0.125 -> 0.20; 60 -> 96px; ambiguity unchanged at 21.6px',
             archived_writer_complete=archive['writer']['archive_complete'],robot_description_sha256=hashlib.sha256((a.session/'robot_description.urdf').read_bytes()).hexdigest(),
             rows=rows,source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
             limits=['Sparse archived sources replay computation only; not continuous live tracking or production lock.','Counterfactual masks/status are labeled offline and never published.','Motor identity still requires reviewing saved actual RGB overlays.'])
 (a.output/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
if __name__=='__main__':main()
