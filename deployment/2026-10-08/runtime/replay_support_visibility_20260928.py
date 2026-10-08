#!/usr/bin/env python3
"""Offline source-depth replay; no ROS node, publisher, service or motion."""
import hashlib,json,time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import rosbag
from cv_bridge import CvBridge
from alicia_grasp_modes import tabletop as tt
p=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/grasp_diagnostic_20260928T084245Z')
out=p/'support_visibility_replay';out.mkdir(exist_ok=True)
context=json.loads((p/'initial_context.json').read_text());g=json.loads((p/'geometry_forensics/geometry_report.json').read_text())
camera=context['camera'];K=[camera[k] for k in ('fx','fy','cx','cy')];cfg=tt.Config()
frames=g['frames'];wanted={f['source_stamp_ns']:f for f in frames};depths={};anchor_status=None
with rosbag.Bag(str(p/'scene_diagnostic.bag')) as bag:
 for topic,m,t in bag.read_messages(topics=['/supervisor/camera/depth/image_raw','/perception/unknown/detector_status']):
  if topic.endswith('detector_status'):
   d=json.loads(m.data)
   if anchor_status is None and d.get('generation')==2 and d.get('target_locked') is True:
    anchor_status=d
  elif m.header.stamp.to_nsec() in wanted:
   depths[m.header.stamp.to_nsec()]=CvBridge().imgmsg_to_cv2(m,'passthrough').copy()
assert anchor_status is not None and len(depths)==len(frames)
anchor=SimpleNamespace(position_base=np.array(anchor_status['anchor_position_base']),extent_m=np.array(anchor_status['anchor_extent_m']),support_footprint_base=np.array(anchor_status['reference_support_footprint_base']))
reference=np.array(anchor_status['reference_support_plane_base']);O=np.eye(4);O[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]]
rows=[]
for frame in frames:
 ns=frame['source_stamp_ns'];depth=depths[ns].astype(np.float32)*camera['depth_scale'];T=np.array(frame['T_base_optical'])@O.T
 row=dict(label=frame['label'],source_stamp_ns=ns,stationary=frame['stationary_around_image'])
 np.savez_compressed(out/(frame['label']+'.npz'),depth_raw=depths[ns],depth_scale=camera['depth_scale'],intrinsics=K,transform=T,anchor_position=anchor.position_base,anchor_extent=anchor.extent_m,reference_plane=reference,reference_footprint=anchor.support_footprint_base)
 for name,audit_enabled in [('without_independent_audit',False),('with_independent_audit',True)]:
  started=time.monotonic()
  try:
   with patch.object(tt,'_audit_current_support',tt._audit_current_support if audit_enabled else lambda *a,**k:{}):
    result=tt.segment_with_support_fallback(depth,K,T,cfg,anchor,anchor,reference)
   row[name]=dict(result='segmentation',matching_candidates=len(tt._matching_candidates(result.candidates,anchor,anchor,cfg)),metrics=result.metrics)
  except ValueError as e:row[name]=dict(result='rejected',reason=str(e))
  row[name]['elapsed_sec']=time.monotonic()-started
 points=tt._project(depth,K);v,u=np.indices(depth.shape);center=tt.project_base_to_pixel(anchor.position_base,T,K);radius=np.hypot(u-center[0],v-center[1]);ring=np.isfinite(depth)&(depth>=cfg.depth_min_m)&(depth<=cfg.depth_max_m)&(radius>=70)&(radius<=170)
 try:
  plane,ratio,residual,count=tt._fit_plane(points[ring],cfg);supporting=ring&(np.abs(np.einsum('ijk,k->ij',points,plane[:3])+plane[3])<=cfg.support_threshold_m);covered=tt._support_quadrants(supporting,u,v,center);current=tt._normalized_plane(np.linalg.solve(T.T,plane))
  if current[:3]@reference[:3]<0:current=-current
  delta=current-reference;distance=max(abs(delta[:3]@anchor.position_base+delta[3]),float(np.max(abs(anchor.support_footprint_base@delta[:3]+delta[3]))))
  row['raw_ring_fit']=dict(inlier_ratio=ratio,residual_mm=residual*1000,sample_count=count,quadrants=covered,plane_base=current.tolist(),distance_over_frozen_footprint_mm=distance*1000,quality_eligible_for_comparison=bool(residual<=cfg.support_threshold_m and covered>=3))
 except ValueError as e:row['raw_ring_fit']=dict(reason=str(e))
 rows.append(row);print(json.dumps(row,ensure_ascii=False),flush=True)
source=Path(tt.__file__)
report=dict(scope='offline_only; frozen exact source depth; accepted-FK transform from geometry_forensics; no calibration or target identity update',source=str(source),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),config=vars(cfg),anchor_status=anchor_status,frames=rows,limitations=['Replay uses production pinhole tabletop projection, not a replacement camera model.','Transforms reuse saved source-time accepted FK/hand-eye interpolation from geometry report.','A positive plane inconsistency is coordinate-chain evidence; it does not identify which calibration or arm component is wrong.','Weak-coverage frames are unavailable support, not a positive contradiction.'])
(out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print('REPORT '+str(out/'report.json'),flush=True)
