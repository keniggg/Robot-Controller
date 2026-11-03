from pathlib import Path
from dataclasses import fields
import json,hashlib
import numpy as np,cv2
from alicia_grasp_modes import tabletop as t,rgb_instance as rgb
BASE=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z/gen21_analysis');p=BASE/'replay_inputs';m=json.loads((p/'manifest.json').read_text());rows={r['role']:r for r in m['files']};allrows={r['source_stamp_ns']:r for r in m['files']}
cv=json.loads((p/'unknown_perception_configuration.json').read_text())['parameters'];cfg=t.Config(**{k:v for k,v in cv.items() if k in {f.name for f in fields(t.Config)}})
def arr(a):return a['depth_raw'].astype(np.float32)*float(a['depth_scale']),a['K'],a['T_base_camera']
def load(r):return dict(np.load(r['path']))
a=load(rows['anchor']);tracker=t.TargetTracker(cfg);anchor,_=tracker.choose(t.segment_with_support_fallback(*arr(a),cfg),a['depth_raw'].shape);plane=tracker.plane_base
l=load(rows['last_pre_motion']);last,_=tracker.choose(t.segment_with_support_fallback(*arr(l),cfg,anchor=anchor,last=anchor,reference_plane_base=plane,prefer_current_support=True),l['depth_raw'].shape)
prev=load(rows['previous_ready_reference_probe']);out=[]
for ns in [int(l['source_stamp_ns']),1791636584089448213,1791636585362278461,1791636585622787237]:
 f=load(allrows[ns]);depth,K,T=arr(f);points=t._project(depth,K);valid=np.isfinite(depth)&(depth>=cfg.depth_min_m)&(depth<=cfg.depth_max_m)
 center=t.project_base_to_pixel(anchor.position_base,T,K);v,u=np.indices(depth.shape);rad=np.hypot(u-center[0],v-center[1]);scale=min(depth.shape[1]/640.,depth.shape[0]/480.)
 cp,ratio,rms,count,ring,rpoints,outer=t._fit_support_ring_sequence(depth,K,valid,rad,scale,cfg)
 current=t._normalized_plane(np.linalg.solve(T.T,cp));audit=t._audit_current_support(depth,K,T,cfg,anchor,plane);assert audit['support_current_audit_available']
 height=points@cp[:3]+cp[3];frozen_height=points@(T.T@plane)[:3]+(T.T@plane)[3]
 seg=t.segment_with_support_fallback(depth,K,T,cfg,anchor=anchor,last=last,reference_plane_base=plane,prefer_current_support=True);matches=t._matching_candidates(seg.candidates,anchor,last,cfg);assert len(matches)==1;raw=matches[0][1]
 domain=rgb._observation_spatial_support(points,T,K,rgb.freeze_observation_spatial_reference(anchor,plane),cfg)['domain_mask']
 rawdomain=rgb._observation_spatial_support(points,T,K,rgb.freeze_observation_spatial_reference(raw,seg.plane_base),cfg)['domain_mask']
 masks={}
 if ns==int(l['source_stamp_ns']):
  masks['recorded_latest']=f['recorded_refined_mask']!=0;masks['latest_minus_previous_same_pixels']=masks['recorded_latest']&~(prev['recorded_refined_mask']!=0)
 else:
  for variant in ('production','union48_counterfactual'):
   masks[variant]=cv2.imread(str(BASE/('crop_ab_'+variant+'_'+str(ns)+'_mask.png')),cv2.IMREAD_GRAYSCALE)!=0
 summaries={}
 for name,mask in masks.items():
  hv=height[mask&valid];zv=frozen_height[mask&valid];inside=mask&valid&domain
  summaries[name]=dict(pixels=int(mask.sum()),valid_depth_pixels=int((mask&valid).sum()),invalid_depth_pixels=int((mask&~valid).sum()),current_plane_within_2mm=int((mask&valid&(abs(height)<=cfg.support_threshold_m)).sum()),current_plane_between_2_and_4mm=int((mask&valid&(height>cfg.support_threshold_m)&(height<cfg.foreground_min_m)).sum()),current_plane_above_4mm=int((mask&valid&(height>=cfg.foreground_min_m)).sum()),current_plane_negative_beyond_2mm=int((mask&valid&(height< -cfg.support_threshold_m)).sum()),current_plane_height_mm_percentiles=np.percentile(hv*1000,[0,10,50,90,100]).tolist(),frozen_plane_height_mm_percentiles=np.percentile(zv*1000,[0,10,50,90,100]).tolist(),valid_depth_outside_frozen_anchor_footprint=int((mask&valid&~domain).sum()),valid_depth_outside_current_raw_footprint=int((mask&valid&~rawdomain).sum()),current_geometric_mask_points=int((mask&valid&(frozen_height>=cfg.foreground_min_m)&(frozen_height<=cfg.foreground_max_m)).sum()))
 rawsum=dict(candidate_count=len(seg.candidates),unique_matches=len(matches),bbox_xywh=list(raw.bbox),position_base=raw.position_base.tolist(),extent_m=raw.extent_m.tolist(),quality=raw.quality,raw_pixels=int(np.count_nonzero(raw.mask)),metrics=raw.metrics)
 out.append(dict(source_ns=ns,raw_candidate=rawsum,current_support_plane_base=current.tolist(),frozen_support_plane_base=plane.tolist(),current_support_audit=audit,mask_summaries=summaries))
report=dict(record=m['record'],analysis=out,implementation_reason='refine_candidate geometry uses valid depth and foreground_min_m..foreground_max_m; _appearance intentionally uses all owned RGB mask pixels, including depth holes and between-support-and-foreground pixels. Current depth alone therefore does not constrain all appearance histogram contributions.',limitations=['Current support fit remains a local fitted measurement with recorded exact TF; no physical metrology claim.','Footprint containment is computed from current points at the recorded TF. Outside a historical footprint is not by itself wrong identity, and boundary/depth noise may change current raw footprint.','No red-color-only filter, threshold relaxation, replacement reference or production mutation is applied.'])
def default(v):
 if isinstance(v,np.ndarray):return v.tolist()
 if isinstance(v,np.generic):return v.item()
 raise TypeError(type(v).__name__)
path=BASE/'identity_support_domain_diagnostic.json';path.write_text(json.dumps(report,default=default,indent=2)+'\n');print(json.dumps(dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),frames=[dict(source_ns=r['source_ns'],masks=r['mask_summaries']) for r in out]),default=default))
