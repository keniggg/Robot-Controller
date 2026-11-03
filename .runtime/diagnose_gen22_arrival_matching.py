#!/usr/bin/env python3
"""Read-only current measurement association diagnostic, never execution authority."""
from pathlib import Path
import json,hashlib,sys
import cv2,numpy as np
p=Path(__file__).with_name('trace_gen21_arrival_rgb.py');code=p.read_text().split('rows=[]')[0].replace('gen21_handoff_analysis','gen22_handoff_analysis');g={};exec(compile(code,str(p),'exec'),g)
OUT=g['ROOT']/'raw_matching';OUT.mkdir(exist_ok=True)
anchor,last,cfg=g['anchor'],g['last'],g['cfg'];rows=[]
for row in (r for r in g['meta']['files'] if r['role']=='arrival'):
 _,rgb,depth,T,valid=g['frame'](row);ns=row['source_stamp_ns'];r=dict(source_stamp_ns=ns,online_evaluated=row['online_evaluated'],online_reason=row['online_reason'])
 try:
  result=g['table'].segment_with_support_fallback(depth,g['K'],T,cfg,anchor,last,g['plane'],prefer_current_support=True)
  matches=g['table']._matching_candidates(result.candidates,anchor,last,cfg)
  r.update(candidate_count=len(result.candidates),matching_count=len(matches),support=result.metrics,candidates=[])
  overlay=rgb.copy()
  for i,c in enumerate(result.candidates):
   dist=float(np.linalg.norm(c.position_base-anchor.position_base));step=float(np.linalg.norm(c.position_base-last.position_base));ratios=c.extent_m[:2]/np.maximum(anchor.extent_m[:2],.003)
   r['candidates'].append(dict(index=i,pixels=int(np.count_nonzero(c.mask)),bbox=c.bbox,position_base=c.position_base.tolist(),extent_m=c.extent_m.tolist(),distance_anchor_m=dist,distance_last_m=step,size_ratios=ratios.tolist(),gates=dict(anchor_distance=dist<=cfg.association_distance_m,last_distance=step<=cfg.association_distance_m,long_ratio=.5<=ratios[0]<=2,short_ratio=.5<=ratios[1]<=2),quality=c.quality,median_height_m=c.metrics['median_height_m']))
   cv2.drawContours(overlay,cv2.findContours(c.mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2],-1,(0,255,0),1);cv2.putText(overlay,str(i),(c.bbox[0],c.bbox[1]),cv2.FONT_HERSHEY_SIMPLEX,.4,(0,0,255),1)
  cv2.imwrite(str(OUT/(str(ns)+'_candidates.png')),overlay)
 except Exception as e:r.update(error=type(e).__name__+': '+str(e))
 rows.append(r);print(json.dumps(r,default=lambda v:v.item()),flush=True)
report=dict(record=g['meta']['record'],anchor_source_stamp_ns=g['meta']['anchor_source_stamp_ns'],last_pre_motion_source_stamp_ns=g['meta']['last_pre_motion_source_stamp_ns'],last_mask_bitexact=g['reference_equal'],anchor_position=anchor.position_base.tolist(),anchor_extent=anchor.extent_m.tolist(),last_position=last.position_base.tolist(),rows=rows,limitations=['Exact recorded sources only; five arrivals match online evaluated sources.','TF reconstruction and original segmentation, no ROS calls or online authority.'])
(OUT/'result.json').write_text(json.dumps(report,indent=2,default=lambda v:v.item())+'\n');print('RESULT',OUT/'result.json')
