#!/usr/bin/env python3
"""Sealed exact frames: existing current-support segmentation, unchanged gates."""
from pathlib import Path
import hashlib,json
source=Path('.runtime/replay_gen15_support_gap_rgb_details.py').read_text()
prefix=source.split('rows=[];valid=')[0]
prefix=prefix.replace("OUT=NEW/'gen15_handoff_analysis/support_gap_rgb_replay_details'", "OUT=NEW/'gen15_handoff_analysis/current_support_rgb_counterfactual'")
prefix=prefix.replace('needed={initial,good,bad,gap}', 'needed={initial,good,bad,gap,1791523443295139074,1791523443657088041}')
exec(compile(prefix,'sealed_gen15_original_loader','exec'))
from alicia_grasp_modes.tabletop import segment

def bbox(m):
 y,x=np.nonzero(m);return [int(x.min()),int(y.min()),int(x.max())+1,int(y.max())+1]
def iou(a,b):return float(np.count_nonzero((a>0)&(b>0))/np.count_nonzero((a>0)|(b>0)))
def native(segm,previous):
 matches=_matching_candidates(segm.candidates,anchor,previous,cfg)
 assert len(matches)==1,dict(reason='original_unique_identity',count=len(matches))
 return matches[0][1]
def refine(colour,d,t,c):return refine_candidate(colour,d,K,t,c,cfg,observation_arrival_retry=True,rgb_valid=rectification_rgb_valid_mask(camera,d.shape))
def hmap(d,t,p):
 points=_project(d,K);pc=t.T@np.asarray(p);return np.einsum('ijk,k->ij',points,pc[:3])+pc[3]
def stats(region,d,h,seed=None,core=None):
 valid=np.isfinite(d)&(d>=cfg.depth_min_m)&(d<=cfg.depth_max_m);v=h[valid&region];ah=np.abs(h)
 return dict(pixels=int(np.count_nonzero(region)),valid_depth=int(np.count_nonzero(valid&region)),invalid_depth=int(np.count_nonzero(region&~valid)),height_mm_quantiles=(np.percentile(v,[0,5,25,50,75,95,100])*1000).tolist() if len(v) else [],abs_le_2mm=int(np.count_nonzero(region&valid&(ah<=.002))),abs_2_to_4mm=int(np.count_nonzero(region&valid&(ah>.002)&(ah<=.004))),abs_gt_4mm=int(np.count_nonzero(region&valid&(ah>.004))),positive_ge_original_foreground=int(np.count_nonzero(region&valid&(h>=cfg.foreground_min_m)&(h<=cfg.foreground_max_m))),native_seed_pixels=None if seed is None else int(np.count_nonzero(region&(seed>0))),hard_core_pixels=None if core is None else int(np.count_nonzero(region&core)))
def coremask(colour,d,t,c):
 y,x=np.nonzero(c.mask);x0=max(0,int(x.min())-16);x1=min(d.shape[1],int(x.max())+17);y0=max(0,int(y.min())-16);y1=min(d.shape[0],int(y.max())+17);sl=np.s_[y0:y1,x0:x1]
 points=_project(d,K);p=t.T@np.asarray(c.metrics['support_plane_base']);heights=np.einsum('ijk,k->ij',points,p[:3])+p[3]
 core,support,evidence=confidence_labels(c.mask[sl]>0,points[sl],d[sl],heights[sl],p[:3],cfg,bgr=colour[sl]);result=np.zeros(d.shape,bool);result[sl]=core
 return result,evidence

frames={};rows=[];prior_original=anchor;prior_current=anchor
for ns in [1791523443295139074,1791523443657088041,good,bad]:
 rgb,raw,d,T=decoded(ns);valid=rectification_rgb_valid_mask(camera,d.shape);began=time.perf_counter()
 original=segment_with_support_fallback(d,K,T,cfg,anchor,prior_original,plane)
 old=native(original,prior_original);oldref=refine(rgb,d,T,old)
 current=segment(d,K,T,cfg,target_position_base=anchor.position_base,plane_base=plane,target_footprint_base=anchor.support_footprint_base,fit_current_support=True)
 cur=native(current,prior_current);curref=refine(rgb,d,T,cur)
 recorded=None if ns not in masks else bridge.imgmsg_to_cv2(masks[ns],'mono8')
 bitexact=None if recorded is None or not np.any(recorded) else bool(np.array_equal(oldref.mask,recorded))
 if ns!=bad:assert bitexact,'actual published mask must match original full replay'
 oc,oe=coremask(rgb,d,T,old);nc,ne=coremask(rgb,d,T,cur)
 frames[ns]=dict(rgb=rgb,depth=d,T=T,original=old,oldref=oldref,current=cur,newref=curref,oldcore=oc,newcore=nc,raw=raw)
 original_only=(oldref.mask>0)&~(curref.mask>0)
 rr=dict(source_stamp_ns=ns,original_source=original.metrics['support_source'],original_seed_pixels=int(np.count_nonzero(old.mask)),original_refined_pixels=int(np.count_nonzero(oldref.mask)),original_seed_bbox=bbox(old.mask),original_refined_bbox=bbox(oldref.mask),original_published_mask_bitexact=bitexact,current_seed_pixels=int(np.count_nonzero(cur.mask)),current_refined_pixels=int(np.count_nonzero(curref.mask)),current_seed_bbox=bbox(cur.mask),current_refined_bbox=bbox(curref.mask),current_vs_original_iou=iou(curref.mask,oldref.mask),current_support_metrics=current.metrics,original_support_metrics=original.metrics,original_quality=oldref.quality,current_quality=curref.quality,current_pose_delta_m=float(np.linalg.norm(curref.position_base-oldref.position_base)),current_original_only_stats=dict(under_original_geometry=stats(original_only,d,hmap(d,T,old.metrics['support_plane_base']),old.mask,oc),under_current_geometry=stats(original_only,d,hmap(d,T,cur.metrics['support_plane_base']),cur.mask,nc)),processing_wall_sec=time.perf_counter()-began)
 rows.append(rr);prior_original=old;prior_current=cur
 for name,mask_,cc in [('original',oldref.mask,oc),('current_support',curref.mask,nc)]:
  overlay=rgb.copy();contours,_=cv2.findContours(mask_,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE);cv2.drawContours(overlay,contours,-1,(0,255,0),1);contours,_=cv2.findContours(cc.astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE);cv2.drawContours(overlay,contours,-1,(255,0,255),1)
  cv2.imwrite(str(OUT/(str(ns)+'_'+name+'_mask.png')),mask_);cv2.imwrite(str(OUT/(str(ns)+'_'+name+'_overlay.png')),overlay);cv2.imwrite(str(OUT/(str(ns)+'_'+name+'_crop.png')),overlay[160:325,245:410])

comparisons=[]
for mode in ['oldref','newref']:
 previous=frames[good];now=frames[bad];rh,rc=_appearance(previous['rgb'],previous[mode].mask);h,c=_appearance(now['rgb'],now[mode].mask)
 colour=float(np.sqrt(max(0.,1.-np.sum(np.sqrt(h*rh)))));shape=float(cv2.matchShapes(rc,c,cv2.CONTOURS_MATCH_I1,0.))
 try:
  refined,evidence=metric_seeded_identity_evidence(now['rgb'],[now['original' if mode=='oldref' else 'current']],previous['rgb'],previous[mode].mask,minimum_quality=.5,rgb_valid=rectification_rgb_valid_mask(camera,now['depth'].shape),refine=lambda cand:refine(now['rgb'],now['depth'],now['T'],cand))
  accepted=True;error=None
 except ValueError as e:accepted=False;error=str(e);evidence=None
 comparisons.append(dict(mode=mode,accepted=accepted,error=error,evidence=evidence,mask_iou=iou(previous[mode].mask,now[mode].mask),colour_distance=colour,contour_distance=shape))

oldonly=(frames[good]['oldref'].mask>0)&~(frames[bad]['oldref'].mask>0)
oldonly_stats={}
for ns in [good,bad]:
 f=frames[ns];oldonly_stats[str(ns)]={}
 for geometry,plane_ in [('immutable_plane',plane),('current_independently_fitted_plane',f['current'].metrics['support_plane_base'])]:
  oldonly_stats[str(ns)][geometry]=stats(oldonly,f['depth'],hmap(f['depth'],f['T'],plane_),f['original'].mask,f['oldcore'])
joint_names=feedback[0]['names'];joints=np.array([f['positions'] for f in feedback]);joint_range=np.ptp(joints,axis=0).tolist()
report=dict(record=str(REC),scope='Exact original RGB-D/TF, same immutable anchor/plane/footprint, original independent current-support fit and original unique/RGB identity gates; counterfactual changes segmentation choice only.',source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path('src/alicia_grasp_modes/src/alicia_grasp_modes/tabletop.py'),Path('src/alicia_grasp_modes/src/alicia_grasp_modes/rgb_instance.py'),Path('src/alicia_grasp_modes/src/alicia_grasp_modes/metric_ambiguity_identity.py')]},rows=rows,last_good_to_failed_identity=comparisons,oldonly_height_decomposition=oldonly_stats,original_qualified_arrival_sources=[1791523442892243862,1791523443057922601,1791523443295139074],exact_arrival_sources_saved=[1791523443295139074],missing_original_arrival_rgbd=[1791523442892243862,1791523443057922601],joint_names=joint_names,accepted_feedback_joint_range_rad=joint_range,anchor_reconstruction=dict(position_extent_tolerance_m=1e-10,footprint_max_reconstruction_difference_m=float(np.max(np.abs(reconstructed_footprint-recorded_footprint))),footprint_used='exact recorded fixed footprint'),limitations=['The first two online qualified arrival frames are absent from saved 5Hz RGB-D, so this report cannot certify all three exact arrival frames or full requalification.','The third qualified frame and subsequent two normal published masks reproduce bitwise; terminal source is exact saved RGB-D.','Pure offline fit/refinement/identity replay does not create live source freshness, marker/session authority or physical execution permission.','No production changes, ROS calls, threshold relaxation or replacement of the immutable reference plane.'])
p=OUT/'result.json';p.write_text(json.dumps(report,ensure_ascii=False,indent=2,default=lambda a:a.tolist() if isinstance(a,np.ndarray) else a)+'\n');print(json.dumps(dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),rows=[{k:r[k] for k in ['source_stamp_ns','original_source','original_refined_pixels','current_refined_pixels','original_published_mask_bitexact']} for r in rows],identity=comparisons),ensure_ascii=False,indent=2))
