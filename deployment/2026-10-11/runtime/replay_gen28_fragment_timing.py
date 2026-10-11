#!/usr/bin/env python3
"""Exact sealed RGB-D replay with scoped offline-only timing wrappers."""
import argparse, hashlib, json, time, cProfile, pstats
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import cv2
from alicia_grasp_modes import tabletop as table, rgb_instance as rgbmod, arrival_fragment_group as group, measured_mask_seeds as seeds
from alicia_grasp_modes.rgb_observation_handoff import _appearance
from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask

p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--repeat',type=int,default=5);args=p.parse_args()
root=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_ros_20261009T003407Z/gen28_handoff_analysis/exact_source_images')
meta=json.loads((root/'manifest.json').read_text())
assert not (args.output/'result.json').exists()
args.output.mkdir(parents=True,exist_ok=True)
cv2.setNumThreads(1)
files=[Path(m.__file__) for m in (table,rgbmod,group,seeds)]+[Path(__file__)]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
before={str(p):sha(p) for p in files}
stages=[]
def timed(name,f):
 def wrapper(*a,**kw):
  t=(time.perf_counter(),time.thread_time(),time.process_time())
  try:return f(*a,**kw)
  finally:stages.append(dict(stage=name,wall_sec=time.perf_counter()-t[0],thread_cpu_sec=time.thread_time()-t[1],process_cpu_sec=time.process_time()-t[2]))
 return wrapper
for module,names in ((group,('_project','_observation_spatial_support','confidence_labels','refine_candidate','_appearance')),
                     (rgbmod,('_project','_observation_spatial_support','confidence_labels','instance_mask_from_seed')),
                     (seeds,('local_surface_normals','_side_seed_colour_evidence'))):
 for name in names:
  if hasattr(module,name):setattr(module,name,timed(module.__name__.split('.')[-1]+'.'+name,getattr(module,name)))
if hasattr(rgbmod,'_FrameRefinement'):
 for name in ('refine','_local_fit','confidence_labels'):
  setattr(rgbmod._FrameRefinement,name,timed('FrameRefinement.'+name,getattr(rgbmod._FrameRefinement,name)))
 rgbmod.local_surface_normals=timed('frame.local_surface_normals',rgbmod.local_surface_normals)
cv2.grabCut=timed('opencv.grabCut',cv2.grabCut)
cfg=table.Config()
s=meta['sources'][0]['detector_status']['message']
anchor=SimpleNamespace(position_base=np.array(s['anchor_position_base']),extent_m=np.array(s['anchor_extent_m']),support_footprint_base=np.array(s['reference_support_footprint_base']),metrics={'median_height_m':s['reference_object_height_m']})
plane=np.array(s['reference_support_plane_base']);spatial=rgbmod.freeze_observation_spatial_reference(anchor,plane)
def frame(row):
 a=np.load(row['npz']);cam=row['camera_metadata']['message']['parameters'];K=[cam[k] for k in ('fx','fy','cx','cy')];dep=a['depth_raw'].astype(np.float32)*cam['depth_scale'];valid=rectification_rgb_valid_mask(cam,dep.shape)
 return a,dep,K,valid
prem=meta['sources'][1];pa,pd,pk,pv=frame(prem)
hist,contour=_appearance(pa['rgb'],pa['refined_mask'])
# Anchor geometry is published independently of RGB refinement. Reconstruct the
# last reference from its exact source before processing the two target frames.
last=SimpleNamespace(position_base=np.array(prem['detector_status']['message']['position_base']))
rows=[]
for row in meta['sources'][2:]:
 a,depth,K,valid=frame(row);T=a['T_base_camera'];stages.clear()
 seg=timed('segment',table.segment_with_support_fallback)(depth,K,T,cfg,anchor,last,plane,prefer_current_support=True)
 assert not table._matching_candidates(seg.candidates,anchor,last,cfg)
 segment_stages=list(stages);runs=[]
 for i in range(args.repeat):
  stages.clear()
  out,pair=timed('group_total',group.recover_arrival_fragment_group)(seg,a['rgb'],depth,K,T,cfg,anchor,last,spatial,hist,contour,.5,rgb_valid=valid)
  assert pair is not None
  raw,refined=pair
  runs.append(dict(repeat=i,stages=list(stages),mask_sha256=hashlib.sha256(refined.mask.tobytes()).hexdigest()))
 assert len({r['mask_sha256'] for r in runs})==1
 eq=bool(np.array_equal(refined.mask,a['refined_mask']))
 if row['object_message']['detected']:assert eq
 np.savez_compressed(args.output/(str(row['source_stamp_ns'])+'_result.npz'),mask=refined.mask,raw_mask=raw.mask,position=refined.position_base,extent=refined.extent_m,footprint=refined.support_footprint_base)
 result=dict(source_stamp_ns=row['source_stamp_ns'],role=row['role'],published_mask_bitexact=eq if row['object_message']['detected'] else None,mask_pixels=int(np.count_nonzero(refined.mask)),native_candidates=len(seg.candidates),segmentation=segment_stages,runs=runs,metrics=refined.metrics)
 rows.append(result)
 print(json.dumps(dict(source=row['source_stamp_ns'],bitexact=result['published_mask_bitexact'],pixels=result['mask_pixels'],timings={name:[sum(z['wall_sec'] for z in r['stages'] if z['stage']==name) for r in runs] for name in sorted(set(z['stage'] for z in runs[0]['stages']))})),flush=True)
 profile=cProfile.Profile();profile.enable();group.recover_arrival_fragment_group(seg,a['rgb'],depth,K,T,cfg,anchor,last,spatial,hist,contour,.5,rgb_valid=valid);profile.disable()
 with (args.output/(str(row['source_stamp_ns'])+'_profile.txt')).open('w') as f:pstats.Stats(profile,stream=f).sort_stats('cumulative').print_stats(40)
 last=raw
after={str(p):sha(p) for p in files};assert before==after
report=dict(scope='OFFLINE_EXACT_PIXELS_NO_ROS_NO_AUTHORITY',input_manifest=str(root/'manifest.json'),input_manifest_sha256=sha(root/'manifest.json'),source_before=before,source_after=after,rows=rows,limitations=['Published original anchor geometry reconstructed from exact status fields, not tracker object memory.','Fixed appearance recomputed from exact premotion ready RGB and bitexact published mask; in-memory histogram/contour was not separately recorded.','Lastgood output must equal actual published mask bit for bit. Failed published empty mask is not an internal refinement result.','No callbacks, processing deadline, hold state, GIL or historical scheduling state replayed; wrappers overlap and timings must not be summed.','Original Config defaults; no threshold or source values modified.'])
(args.output/'result.json').write_text(json.dumps(report,indent=2)+'\n')
