#!/usr/bin/env python3
"""Read-only exact-frame tracing of the deployed RGB validity rejection."""
from pathlib import Path
import sys,json,hashlib,time
import cv2,numpy as np
from alicia_grasp_modes import tabletop as table,rgb_instance as rgbmod
from alicia_grasp_modes.arrival_fragment_group import recover_arrival_fragment_group
from alicia_grasp_modes.observation_arrival_confirmation import ObservationArrivalConfirmation
from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask

ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_ros_20261009T003407Z/gen23_handoff_analysis')
IN=ROOT/'source_images';OUT=ROOT/'rgb_valid_crop_trace';OUT.mkdir(exist_ok=True)
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
first,_,_,_,_=frame(role('view1_last_pre_motion'))
a,rgb,dep,T,valid=frame(role('view2_last_pre_motion'))
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
for row in [r for r in meta['files'] if 'view2_arrival' in r['roles']]:
 a,rgb,dep,T,valid=frame(row);trace=[];ns=row['source_stamp_ns'];began=time.perf_counter()
 output=dict(source_stamp_ns=ns,online_evaluated=row['online_evaluated'],online_reason=row['online_reason'])
 def capture(f,event,arg):
  if f.f_code.co_filename!=rgbmod.__file__ or event!='exception':return capture
  if f.f_code.co_name not in ('instance_mask_from_seed','attempt') or str(arg[1])!='rgb_search_contains_unobserved_rgb':return capture
  v=f.f_locals
  keys=('ax0','ay0','ax1','ay1') if f.f_code.co_name=='attempt' else ('x0','y0','x1','y1')
  if not all(k in v for k in keys):return capture
  bounds=[int(v[k]) for k in keys];x0,y0,x1,y1=bounds
  invalid=np.zeros(dep.shape,bool);invalid[y0:y1,x0:x1]=~valid[y0:y1,x0:x1]
  t=dict(function=f.f_code.co_name,line=f.f_lineno,search_box_xyxy=bounds,
   invalid_rgb_pixels=int(invalid.sum()),invalid_bbox_xyxy=bbox(invalid),
   seed_box_xyxy=bbox(v['seed']),seed_invalid_pixels=int(np.count_nonzero(v['seed']&~valid)))
  if 'box' in v:t['spatial_box_xyxy']=np.asarray(v['box']).tolist()
  if 'core' in v:t['trusted_core_pixels']=int(np.count_nonzero(v['core']))
  trace.append(t)
  if len(trace)==1:
   overlay=rgb.copy();overlay[invalid]=(0,0,255);cv2.rectangle(overlay,(x0,y0),(x1-1,y1-1),(0,255,255),1)
   cv2.drawContours(overlay,cv2.findContours(v['seed'].astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2],-1,(0,255,0),1)
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
 anchor_source_stamp_ns=role('anchor')['source_stamp_ns'],reference_source_stamp_ns=role('view2_last_pre_motion')['source_stamp_ns'],
 reference_mask_bitexact=reference_equal,source_sha256={p:hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in [rgbmod.__file__,table.__file__]},rows=rows,
 limitations=['12 recorded view2 frames include only two exact online evaluated failures; earliest online failure has no saved RGB-D.',
 'Exact recorded reference mask reproduced; reconstruction verifies numeric original anchor and plane, not live authority.',
 'sys.settrace observes original function locals; seeds, labels, crops and rejection rules are unchanged.'])
(OUT/'result.json').write_text(json.dumps(report,indent=2)+'\n');print('RESULT',OUT/'result.json')
