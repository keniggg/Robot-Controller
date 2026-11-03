#!/usr/bin/env python3
"""Read-only exact-frame tracing of the deployed RGB validity rejection."""
from pathlib import Path
import sys,json,hashlib,time
import cv2,numpy as np
from alicia_grasp_modes import tabletop as table,rgb_instance as rgbmod
from alicia_grasp_modes.arrival_fragment_group import recover_arrival_fragment_group
from alicia_grasp_modes.observation_arrival_confirmation import ObservationArrivalConfirmation
from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask

ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_ros_20261009T003407Z/gen25_handoff_analysis')
IN=ROOT/'source_images';OUT=ROOT/'rgb_required_region_trace';OUT.mkdir(exist_ok=True)
meta=json.loads((IN/'manifest.json').read_text());camera=meta['camera'];cfg=table.Config()
K=[camera[k] for k in ('fx','fy','cx','cy')];cv2.setNumThreads(1)
def role(name):return next(r for r in meta['files'] if name in r['roles'])
def frame(row):
 a=np.load(IN/row['npz']);dep=np.asarray(a['depth_raw'],dtype=np.float32)*camera['depth_scale']
 return a,a['rgb'],dep,a['T_base_camera'],rectification_rgb_valid_mask(camera,dep.shape)
def bbox(mask):
 yy,xx=np.nonzero(mask);return [int(xx.min()),int(yy.min()),int(xx.max()+1),int(yy.max()+1)] if len(xx) else None
a,rgb,dep,T,valid=frame(role('anchor'));seg=table.segment_with_support_fallback(dep,K,T,cfg)
tracker=table.TargetTracker(cfg);anchor,reason=tracker.choose(seg,dep.shape);assert anchor is not None,reason
expected=meta['anchor_status'];plane=np.asarray(expected['reference_support_plane_base'])
assert np.allclose(anchor.position_base,expected['anchor_position_base'],atol=1e-10,rtol=0)
assert np.allclose(anchor.extent_m,expected['anchor_extent_m'],atol=1e-10,rtol=0)
assert np.allclose(seg.plane_base,plane,atol=1e-10,rtol=0)
spatial=rgbmod.freeze_observation_spatial_reference(anchor,plane)
first,_,_,_,_=frame(role('view2_last_pre_motion'))
a,rgb,dep,T,valid=frame(role('view3_last_pre_motion'))
seg=table.segment_with_support_fallback(dep,K,T,cfg,anchor,anchor,plane,prefer_current_support=True)
seg,pair=recover_arrival_fragment_group(seg,rgb,dep,K,T,cfg,anchor,anchor,spatial,
 first['histogram'],first['contour'],.5,rgb_valid=valid)
matches=table._matching_candidates(seg.candidates,anchor,anchor,cfg);assert len(matches)==1
last=matches[0][1]
refined=(pair[1] if pair is not None and last is pair[0] else rgbmod.refine_candidate(
 rgb,dep,K,T,last,cfg,rgb_valid=valid,observation_arrival_retry=True,observation_spatial_reference=spatial))
reference_equal=bool(np.array_equal(refined.mask,a['refined_mask']));assert reference_equal
helper=ObservationArrivalConfirmation(cfg,anchor,last,plane,a['histogram'],a['contour'],.5)
rows=[]
for row in sorted([r for r in meta['files'] if 'view3_arrival' in r['roles']],key=lambda r:not r['online_evaluated']):
 a,rgb,dep,T,valid=frame(row);trace=[];ns=row['source_stamp_ns'];began=time.perf_counter()
 output=dict(source_stamp_ns=ns,online_evaluated=row['online_evaluated'],online_reason=row['online_reason'])
 def region(bounds):
  xx0,yy0,xx1,yy1=map(int,bounds); inside=0<=xx0<xx1<=valid.shape[1] and 0<=yy0<yy1<=valid.shape[0]
  mask=np.zeros(valid.shape,bool);mask[max(0,yy0):min(valid.shape[0],yy1),max(0,xx0):min(valid.shape[1],xx1)]=True
  bad=mask&~valid
  return dict(box_xyxy=list(map(int,bounds)),inside_canvas=inside,invalid_count=int(bad.sum()),invalid_bbox_xyxy=bbox(bad))
 def capture(f,event,arg):
  if f.f_code.co_filename!=rgbmod.__file__ or event!='exception':return capture
  if f.f_code.co_name!='_observed_spatial_search_bounds':return capture
  v=f.f_locals
  if str(arg[1])!='rgb_required_spatial_region_unobserved':return capture
  required=[v[k] for k in ('rx0','ry0','rx1','ry1')]
  t={name:region(v[k]) for name,k in [('desired','desired_bounds'),('first16','first_bounds'),('proposal','proposal_bounds')]}
  t['required']=region(required)
  trace.append(t)
  overlay=rgb.copy();overlay[~valid]=(0,0,255)
  for name,color in [('first16',(0,255,0)),('proposal',(255,0,0)),('required',(0,255,255))]:
   x0,y0,x1,y1=t[name]['box_xyxy'];cv2.rectangle(overlay,(x0,y0),(x1-1,y1-1),color,1)
  cv2.imwrite(str(OUT/(str(ns)+'_overlay.png')),overlay)
  return capture
 try:
  seg=table.segment_with_support_fallback(dep,K,T,cfg,anchor,last,plane,prefer_current_support=True)
  output.update(raw_candidate_count=len(seg.candidates),raw_matching_count=len(table._matching_candidates(seg.candidates,anchor,last,cfg)))
  sys.settrace(capture)
  try:
   confirmed,candidate,staged,evidence=helper.observe(seg,rgb,dep,K,T,ns,rgb_valid=valid)
   output.update(passed=True,confirmed=confirmed,evidence=evidence)
  finally:sys.settrace(None)
 except (ValueError,AssertionError) as exc:output.update(passed=False,reason=str(exc))
 output.update(trace=trace,offline_wall_sec=time.perf_counter()-began);rows.append(output)
 print(json.dumps(output),flush=True)
report=dict(scope='ORIGINAL_FUNCTION_TRACE_NO_ROS_NO_AUTHORITY',record=meta['record'],
 anchor_source_stamp_ns=role('anchor')['source_stamp_ns'],reference_source_stamp_ns=role('view3_last_pre_motion')['source_stamp_ns'],
 reference_mask_bitexact=reference_equal,source_sha256={p:hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in [rgbmod.__file__,table.__file__]},rows=rows,
 limitations=['13 recorded view3 frames include only one exact online evaluated failure; earliest online failure has no saved RGB-D.',
 'Exact recorded reference mask reproduced; reconstruction verifies numeric original anchor and plane, not live authority.',
 'sys.settrace observes original function locals; seeds, labels, crops and rejection rules are unchanged.'])
(OUT/'result.json').write_text(json.dumps(report,indent=2)+'\n');print('RESULT',OUT/'result.json')
