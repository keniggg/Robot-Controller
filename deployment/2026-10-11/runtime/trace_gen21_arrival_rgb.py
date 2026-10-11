#!/usr/bin/env python3
"""Trace original RGB attempt locals without changing its labels or decisions."""
from pathlib import Path
from dataclasses import replace,asdict
import sys,json,hashlib,time
import cv2,numpy as np
from alicia_grasp_modes import tabletop as table
from alicia_grasp_modes import rgb_instance as rgbmod
from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask
ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_ros_20261009T003407Z/gen21_handoff_analysis')
IN=ROOT/'source_images';OUT=ROOT/'rgb_attempt_trace';OUT.mkdir(exist_ok=True)
meta=json.loads((IN/'manifest.json').read_text());camera=meta['camera'];cfg=table.Config()
K=[camera[k] for k in ('fx','fy','cx','cy')];cv2.setNumThreads(1)
def frame(row):
 a=np.load(IN/row['npz']);rgb=a['rgb'];raw=a['depth_raw'];dep=np.asarray(raw,dtype=np.float32)*camera['depth_scale'];T=a['T_base_camera'];valid=rectification_rgb_valid_mask(camera,dep.shape)
 return a,rgb,dep,T,valid
ar=next(r for r in meta['files'] if r['role']=='anchor');la=next(r for r in meta['files'] if r['role']=='last_pre_motion')
a,col,dep,T,valid=frame(ar);seg=table.segment_with_support_fallback(dep,K,T,cfg)
tracker=table.TargetTracker(cfg);anchor,why=tracker.choose(seg,dep.shape);assert anchor is not None,why
actual=meta['anchor_status'];plane=np.asarray(actual['reference_support_plane_base'])
assert np.allclose(anchor.position_base,actual['anchor_position_base'],atol=1e-10,rtol=0)
assert np.allclose(anchor.extent_m,actual['anchor_extent_m'],atol=1e-10,rtol=0)
assert np.allclose(seg.plane_base,plane,atol=1e-10,rtol=0)
a,col,dep,T,valid=frame(la);seg=table.segment_with_support_fallback(dep,K,T,cfg,anchor,anchor,plane,prefer_current_support=True)
matches=table._matching_candidates(seg.candidates,anchor,anchor,cfg);assert len(matches)==1
last=matches[0][1];refined=rgbmod.refine_candidate(col,dep,K,T,last,cfg,rgb_valid=valid)
reference_equal=bool(np.array_equal(refined.mask,a['refined_mask']));assert reference_equal
rows=[]
def bbox(mask):
 y,x=np.nonzero(mask);return [int(x.min()),int(y.min()),int(x.max()+1),int(y.max()+1)] if len(x) else None
for row in [r for r in meta['files'] if r['role']=='arrival']:
 a,col,dep,T,valid=frame(row);ns=row['source_stamp_ns'];attempts=[];began=time.perf_counter()
 result=dict(source_stamp_ns=ns,online_reason=row['online_reason'],online_evaluated=row['online_evaluated'])
 try:
  seg=table.segment_with_support_fallback(dep,K,T,cfg,anchor,last,plane,prefer_current_support=True)
  matches=table._matching_candidates(seg.candidates,anchor,last,cfg)
  result.update(candidate_count=len(seg.candidates),matching_count=len(matches),support_metrics=seg.metrics)
  assert len(matches)==1
  seed=matches[0][1];result.update(raw_seed_bbox_xyxy=bbox(seed.mask),raw_seed_pixels=int(np.count_nonzero(seed.mask)),raw_seed_metrics=seed.metrics)
  cv2.imwrite(str(OUT/(str(ns)+'_seed.png')),seed.mask)
  def capture(frame,event,arg):
   if frame.f_code.co_filename!=rgbmod.__file__ or frame.f_code.co_name!='attempt':return capture
   if event!='return':return capture
   v=frame.f_locals
   if 'chosen' not in v:return capture
   chosen=v['chosen'];core=v['core_now'];measured=v['measured_now']>0
   bounds=[int(v[k]) for k in ('ax0','ay0','ax1','ay1')]
   full=np.zeros(dep.shape,np.uint8);full[bounds[1]:bounds[3],bounds[0]:bounds[2]]=chosen.astype(np.uint8)*255
   fullcore=np.zeros(dep.shape,np.uint8);fullcore[bounds[1]:bounds[3],bounds[0]:bounds[2]]=core.astype(np.uint8)*255
   i=len(attempts);prefix=str(ns)+'_attempt'+str(i)
   cv2.imwrite(str(OUT/(prefix+'_chosen.png')),full)
   overlay=col.copy();cv2.drawContours(overlay,cv2.findContours(full,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2],-1,(0,255,0),1)
   cv2.drawContours(overlay,cv2.findContours(fullcore,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2],-1,(255,0,0),1)
   cv2.rectangle(overlay,tuple(bounds[:2]),tuple(bounds[2:]),(0,255,255),1);cv2.imwrite(str(OUT/(prefix+'_overlay.png')),overlay)
   attempts.append(dict(search_box_xyxy=bounds,search_margins=list(v['margins']),chosen_bbox_xyxy=bbox(full),chosen_pixels=int(chosen.sum()),
    raw_seed_pixels=int(measured.sum()),trusted_core_pixels=int(core.sum()),chosen_core_fraction=float(np.count_nonzero(chosen&core)/core.sum()),
    chosen_seed_fraction=float(np.count_nonzero(chosen&measured)/measured.sum()),chosen_to_seed_ratio=float(chosen.sum()/measured.sum()),
    boundary_sides=[name for name,touch in zip(('left','top','right','bottom'),(np.any(chosen[:,3]),np.any(chosen[3]),np.any(chosen[:,-4]),np.any(chosen[-4]))) if touch],
    trusted_core_bbox_xyxy=bbox(fullcore),overlay=str(OUT/(prefix+'_overlay.png')),mask=str(OUT/(prefix+'_chosen.png'))))
   return capture
  sys.settrace(capture)
  try:
   refined=rgbmod.refine_candidate(col,dep,K,T,seed,cfg,rgb_valid=valid,observation_arrival_retry=True)
   result.update(refine_pass=True,refined_metrics=refined.metrics)
  finally:sys.settrace(None)
 except (ValueError,AssertionError) as exc:result.update(refine_pass=False,reason=str(exc))
 result.update(attempts=attempts,offline_wall_sec=time.perf_counter()-began);rows.append(result)
 print(json.dumps({k:result.get(k) for k in ('source_stamp_ns','online_reason','matching_count','raw_seed_bbox_xyxy','raw_seed_pixels','reason','attempts')},ensure_ascii=False),flush=True)
report=dict(record=meta['record'],original_source_sha256={rgbmod.__file__:hashlib.sha256(Path(rgbmod.__file__).read_bytes()).hexdigest(),table.__file__:hashlib.sha256(Path(table.__file__).read_bytes()).hexdigest()},
 config=asdict(cfg),anchor_source_stamp_ns=ar['source_stamp_ns'],last_pre_motion_source_stamp_ns=la['source_stamp_ns'],last_reference_mask_bitexact=reference_equal,
 original_anchor_position_extent_plane_verified=True,rows=rows,
 limitations=['Pure original functions with reconstructed exact TF; no live age/commit authority.','Only four saved arrivals were actually evaluated online, identified per row; other frames are diagnostic only.',
 'Python tracing observes original attempt locals and does not modify seeds, labels, RNG, crop bounds or decisions.','No arrival masks are published or reused for robot authority.'])
(OUT/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print('RESULT',OUT/'result.json',flush=True)
