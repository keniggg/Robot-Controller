#!/usr/bin/env python3
"""Offline same-frame RGB support ablation; never a production authority."""
from pathlib import Path
import inspect,json,hashlib
import cv2,numpy as np
from alicia_grasp_modes import rgb_instance as rgbmod
from alicia_grasp_modes.observation_arrival_confirmation import ObservationArrivalConfirmation
from alicia_grasp_modes.rgb_observation_handoff import _appearance,_mask
# Reuse only the existing verified immutable input reconstruction, not its loop.
loader=Path(__file__).with_name('trace_gen21_arrival_rgb.py')
g={};exec(compile(loader.read_text().split('rows=[]')[0],str(loader),'exec'),g)
ROOT=g['ROOT'];OUT=ROOT/'geometry_rgb_counterfactual';OUT.mkdir(exist_ok=True)
meta,anchor,last,plane,cfg,K,frame=g['meta'],g['anchor'],g['last'],g['plane'],g['cfg'],g['K'],g['frame']
table=g['table'];ref_rgb=g['col'];ref_mask=g['refined'].mask
hist,contour=_appearance(ref_rgb,_mask(ref_mask,ref_rgb.shape[:2]))
footprint=np.asarray(anchor.support_footprint_base).copy();normal=plane[:3]/np.linalg.norm(plane[:3])
# Measured height from the original measured anchor, used only for a search box.
height=float(anchor.metrics['median_height_m']);prism=np.vstack((footprint,footprint+normal*height))
current={};namespace=dict(rgbmod.__dict__)
def diagnostic(chosen,core,measured,bounds,foreground,points):
 x0,y0,x1,y1=bounds
 support=chosen & foreground
 count,labels,stats,_=cv2.connectedComponentsWithStats(support.astype(np.uint8),connectivity=4)
 components=[]
 for i in range(1,count):
  n=int(stats[i,cv2.CC_STAT_AREA])
  if n<cfg.min_object_points:continue
  xyz=points[labels==i];base=xyz@current['T'][:3,:3].T+current['T'][:3,3]
  components.append(dict(points=n,centroid_base=np.median(base,axis=0).tolist(),
    centroid_anchor_distance_m=float(np.linalg.norm(np.median(base,axis=0)-anchor.position_base)),
    overlaps_original_seed=bool(np.any((labels==i)&(measured>0)))))
 yy,xx=np.nonzero(chosen)
 row=dict(search_box_xyxy=bounds,chosen_bbox_xyxy=[int(xx.min()+x0),int(yy.min()+y0),int(xx.max()+x0+1),int(yy.max()+y0+1)],
  chosen_pixels=int(chosen.sum()),original_seed_pixels=int((measured>0).sum()),current_measured_fg_pixels=int(support.sum()),
  original_area_ratio=float(chosen.sum()/(measured>0).sum()),current_measured_area_ratio=float(chosen.sum()/max(1,support.sum())),
  core_retained=float(np.count_nonzero(chosen&core)/core.sum()),components=components,
  boundary_sides=[s for s,t in zip(('left','top','right','bottom'),(np.any(chosen[:,3]),np.any(chosen[3]),np.any(chosen[:,-4]),np.any(chosen[-4]))) if t])
 current['attempts'].append(row)
 full=np.zeros(current['rgb'].shape[:2],np.uint8);full[y0:y1,x0:x1]=chosen.astype(np.uint8)*255
 row['mask_path']=str(OUT/(str(current['ns'])+'_mask.png'));cv2.imwrite(row['mask_path'],full)
 return int(support.sum())
namespace['diagnostic']=diagnostic;namespace['CURRENT']=current
source=inspect.getsource(rgbmod.instance_mask_from_seed)
needle='        # Unobserved black padding must not train'
source=source.replace(needle,"        if arrival_geometry:\n            ax0, ay0, ax1, ay1 = CURRENT['bounds']\n"+needle)
needle='        boundary_sides = tuple(name for name, touches in zip(side_names, ('
insert="""        if arrival_geometry:
            sl = np.s_[ay0:ay1, ax0:ax1]
            actual_fg = (np.isfinite(depth[sl]) & (depth[sl] >= config.depth_min_m)
                & (depth[sl] <= config.depth_max_m) & np.all(np.isfinite(points[sl]), axis=-1)
                & (heights[sl] >= config.foreground_min_m) & (heights[sl] <= config.foreground_max_m))
            actual_count = diagnostic(chosen,core_now,measured_now,[ax0,ay0,ax1,ay1],actual_fg,points[sl])
"""
assert source.count(needle)==1;source=source.replace(needle,insert+needle)
needle='        expansion = float(np.count_nonzero(chosen)) / float(np.count_nonzero(measured_now))'
source=source.replace(needle,needle+"\n        if arrival_geometry:\n            expansion = float(np.count_nonzero(chosen))/float(max(1,actual_count))")
exec(compile(source,'<offline_current_measured_rgb_support>','exec'),namespace)
exec(compile(inspect.getsource(rgbmod.refine_candidate),'<offline_refine>','exec'),namespace)
refine=namespace['refine_candidate']
import alicia_grasp_modes.observation_arrival_confirmation as arrivalmod
original_refine=arrivalmod.refine_candidate;arrivalmod.refine_candidate=refine
helper=ObservationArrivalConfirmation(cfg,anchor,last,plane,hist,contour,.5)
rows=[]
for row in [r for r in meta['files'] if r['role']=='arrival']:
 _,rgb,depth,T,valid=frame(row);ns=row['source_stamp_ns'];projected=np.array([table.project_base_to_pixel(p,T,K) for p in prism]);margin=max(8,int(round(16*min(depth.shape[1]/640.,depth.shape[0]/480.))))
 result=dict(source_stamp_ns=ns,online_evaluated=row['online_evaluated'],online_reason=row['online_reason'])
 try:
  seg=table.segment_with_support_fallback(depth,K,T,cfg,anchor,last,plane,prefer_current_support=True)
  matches=table._matching_candidates(seg.candidates,anchor,last,cfg);assert len(matches)==1
  seed=matches[0][1];yy,xx=np.nonzero(seed.mask)
  lower=np.minimum(np.floor(projected.min(0)).astype(int),[xx.min(),yy.min()])-margin
  upper=np.maximum(np.ceil(projected.max(0)).astype(int)+1,[xx.max()+1,yy.max()+1])+margin
  bounds=[max(0,int(lower[0])),max(0,int(lower[1])),min(depth.shape[1],int(upper[0])),min(depth.shape[0],int(upper[1]))]
  current.clear();current.update(ns=ns,rgb=rgb,T=T,bounds=bounds,attempts=[])
  result.update(proposal_projected_bounds_xyxy=[*projected.min(0).tolist(),*projected.max(0).tolist()],search_bounds=bounds,raw_seed_pixels=int(np.count_nonzero(seed.mask)))
  confirmed,candidate,_,evidence=helper.observe(seg,rgb,depth,K,T,ns,rgb_valid=valid)
  result.update(passed=True,confirmed=confirmed,evidence=evidence,metrics=candidate.metrics)
 except (ValueError,AssertionError) as exc:
  result.update(passed=False,reason=str(exc));helper.clear()
 result['attempts']=list(current.get('attempts',[]));rows.append(result)
 print(json.dumps({k:result.get(k) for k in ['source_stamp_ns','online_evaluated','passed','confirmed','reason','search_bounds','attempts']},ensure_ascii=False),flush=True)
arrivalmod.refine_candidate=original_refine
report=dict(scope='OFFLINE_COUNTERFACTUAL_NOT_PRODUCTION_NOT_AUTHORITY',record=meta['record'],source_sha256=hashlib.sha256(Path(rgbmod.__file__).read_bytes()).hexdigest(),frozen_anchor_footprint_sha256=hashlib.sha256(footprint.tobytes()).hexdigest(),measured_anchor_height_m=height,rows=rows,
 limitations=['Historical footprint/height only propose crop; never supplies foreground/depth/area denominator.','Area denominator changed diagnostically from single raw component to all CURRENT valid-height measured pixels in the chosen core-connected RGB component; not yet a qualified multi-object identity rule.','Current hard core, support background, RGB validity, 4x numeric limit, original refinement/appearance/three-frame gates unchanged.','No live freshness or execution authority; frames absent from recording not reconstructed.'])
(OUT/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print('RESULT',OUT/'result.json',flush=True)
