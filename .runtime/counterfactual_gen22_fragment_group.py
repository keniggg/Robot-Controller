#!/usr/bin/env python3
"""Measured fragment grouping counterfactual; no production edits or authority."""
from pathlib import Path
from dataclasses import replace
import json,time
import numpy as np,cv2
from alicia_grasp_modes import rgb_instance as rgbmod
from alicia_grasp_modes.observation_arrival_confirmation import ObservationArrivalConfirmation
from alicia_grasp_modes.rgb_observation_handoff import _appearance,_mask
p=Path(__file__).with_name('trace_gen21_arrival_rgb.py');g={};exec(compile(p.read_text().split('rows=[]')[0].replace('gen21_handoff_analysis','gen22_handoff_analysis'),str(p),'exec'),g)
OUT=g['ROOT']/'fragment_group_counterfactual';OUT.mkdir(exist_ok=True)
anchor,last,cfg,table=g['anchor'],g['last'],g['cfg'],g['table'];hist,contour=_appearance(g['col'],_mask(g['refined'].mask,g['col'].shape[:2]));helper=ObservationArrivalConfirmation(cfg,anchor,last,g['plane'],hist,contour,.5);rows=[]
for row in (r for r in g['meta']['files'] if r['role']=='arrival'):
 _,rgb,depth,T,valid=g['frame'](row);ns=row['source_stamp_ns'];r=dict(source_stamp_ns=ns,online_evaluated=row['online_evaluated']);started=time.perf_counter()
 try:
  seg=table.segment_with_support_fallback(depth,g['K'],T,cfg,anchor,last,g['plane'],prefer_current_support=True)
  assert not table._matching_candidates(seg.candidates,anchor,last,cfg)
  camera=table._project(depth,g['K']);spatial=rgbmod._observation_spatial_support(camera,T,g['K'],helper.spatial_reference,cfg)
  binary=np.zeros(depth.shape,bool);r['fragment_pixels_in_domain']=[]
  for c in seg.candidates:
   current=(c.mask>0)&spatial['domain_mask'];r['fragment_pixels_in_domain'].append(int(current.sum()));binary |= current
  assert np.all(np.isfinite(depth[binary])) and np.all(depth[binary]>0)
  xyz=camera[binary].astype(float);base=xyz@T[:3,:3].T+T[:3,3];pcamera=np.median(xyz,axis=0);pbase=(T@np.r_[pcamera,1.])[:3]
  centered=xyz-np.mean(xyz,axis=0);_,_,vh=np.linalg.svd(centered,full_matrices=False);oriented=centered@vh.T;extent=np.sort(np.percentile(oriented,98,axis=0)-np.percentile(oriented,2,axis=0))[::-1]
  yy,xx=np.nonzero(binary);plane=np.asarray(seg.candidates[0].metrics['support_plane_base']);height=camera@(T.T@plane)[:3]+(T.T@plane)[3]
  metrics=dict(seg.candidates[0].metrics,valid_points=int(binary.sum()),median_height_m=float(np.median(height[binary])),observed_surface_extent_m=extent.tolist(),group_proposal_only=True)
  group=table.Candidate(binary.astype(np.uint8)*255,np.array([xx.mean(),yy.mean()]),(int(xx.min()),int(yy.min()),int(xx.max()-xx.min()+1),int(yy.max()-yy.min()+1)),pcamera,pbase,extent,float(pcamera[0]),seg.candidates[0].quality,metrics,table._measured_support_footprint(base,plane))
  r.update(group_pixels=int(binary.sum()),group_extent=extent.tolist(),ratios=(extent[:2]/anchor.extent_m[:2]).tolist(),anchor_distance_m=float(np.linalg.norm(pbase-anchor.position_base)),last_distance_m=float(np.linalg.norm(pbase-last.position_base)))
  assert len(table._matching_candidates([group],anchor,last,cfg))==1,'group fails original identity'
  altered=replace(seg,candidates=[group]);confirmed,candidate,staged,evidence=helper.observe(altered,rgb,depth,g['K'],T,ns,rgb_valid=valid)
  margin=max(8,int(round(16*min(depth.shape[1]/640.,depth.shape[0]/480.))))
  x0=max(0,int(xx.min())-margin);x1=min(depth.shape[1],int(xx.max())+margin+1);y0=max(0,int(yy.min())-margin);y1=min(depth.shape[0],int(yy.max())+margin+1);sl=np.s_[y0:y1,x0:x1]
  core,_,_=rgbmod.confidence_labels(binary[sl],camera[sl],depth[sl],height[sl],(T.T@plane)[:3],cfg,bgr=rgb[sl]);fullcore=np.zeros(depth.shape,bool);fullcore[sl]=core
  r['member_trusted_core']=[dict(count=int(np.count_nonzero((c.mask>0)&fullcore)),retained=int(np.count_nonzero((c.mask>0)&fullcore&(candidate.mask>0)))) for c in seg.candidates]
  r.update(passed=True,confirmed=confirmed,evidence=evidence,metrics=candidate.metrics,
   member_retained_fractions=[float(np.count_nonzero((c.mask>0)&spatial['domain_mask']&(candidate.mask>0))/max(1,np.count_nonzero((c.mask>0)&spatial['domain_mask']))) for c in seg.candidates])
  assert np.array_equal(staged.anchor.support_footprint_base,anchor.support_footprint_base)
  cv2.imwrite(str(OUT/(str(ns)+'_group.png')),group.mask);cv2.imwrite(str(OUT/(str(ns)+'_mask.png')),candidate.mask)
 except Exception as exc:r.update(passed=False,reason=str(exc));helper.clear()
 r['offline_wall_sec']=time.perf_counter()-started;rows.append(r);print(json.dumps({k:v for k,v in r.items() if k not in ('metrics','evidence')}),flush=True)
report=dict(scope='OFFLINE_COUNTERFACTUAL_NOT_AUTHORITY',record=g['meta']['record'],last_reference_mask_bitexact=g['reference_equal'],rows=rows,limits=['Replaces raw candidate list diagnostically with a same-frame measured group confined to frozen zero-expanded footprint; does not establish new grouping authorization or arbitrary touching-object ownership.','No fabricated points/holes or threshold changes; original group geometry gates, actual RGB function/appearance/3-frame checker are used.','Five recorded frames coincide with online evaluated sources; absent sources not recreated.'])
(OUT/'result.json').write_text(json.dumps(report,indent=2)+'\n');print('RESULT',OUT/'result.json')
