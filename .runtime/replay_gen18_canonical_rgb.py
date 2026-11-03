#!/usr/bin/env python3
"""Compare exact canonical/rolling masks without changing any image or gate."""
from pathlib import Path
import json,hashlib
s=Path('.runtime/replay_gen15_support_gap_rgb_details.py').read_text();prefix=s.split('rows=[];valid=')[0]
prefix=prefix.replace('20261009T052127.805554Z_6b8770587c5c4cd8abc679178eaa1c70','20261009T061104.522156Z_f598aaa2d91444a99bdf1a87af163419').replace("OUT=NEW/'gen15_handoff_analysis/support_gap_rgb_replay_details'","OUT=NEW/'gen18_handoff_analysis/canonical_rgb_replay'").replace("NEW/'gen15_handoff_analysis/arrivals_and_terminal.json'","NEW/'gen18_handoff_analysis/first_loss_analysis.json'").replace('gap=1791523444091633558','gap=1791526278703832387').replace('observation_arrival_retry=True','observation_arrival_retry=False')
prefix=prefix.replace('needed={initial,good,bad,gap}', "needed={initial,good,bad,gap,*[row['message']['source_stamp_ns'] for row in summary['all_gen18_status_before_first_loss'] if row['message'].get('source_stamp_ns') and (row['message'].get('detail','').startswith('initial_instance_confirming') or (row['message'].get('state')=='ready' and row['message']['source_stamp_ns']>=good-1_500_000_000))]}")
prefix=prefix.replace('for ns in needed),{ns:list(x) for ns,x in images.items()}','for ns in {initial,good,bad,gap}),{ns:list(x) for ns,x in images.items()}')
prefix=prefix.replace('cfg,anchor,tracker.last,plane)','cfg,anchor,tracker.last,plane,prefer_current_support=True)').replace('cfg,anchor,last,plane)','cfg,anchor,last,plane,prefer_current_support=True)')
exec(compile(prefix,'sealed_gen18_original_loader','exec'))
frames={};rows=[];missing=[]
def iou(a,b):return float(np.count_nonzero((a>0)&(b>0))/np.count_nonzero((a>0)|(b>0)))
for ns in sorted(needed):
 if ns==gap:continue
 if set(images.get(ns,{}))!={'rgb','depth'}:missing.append(ns);continue
 rgb,raw,d,T=decoded(ns);began=time.perf_counter()
 try:
  if ns<=initial:
   seg=segment_with_support_fallback(d,K,T,cfg);candidate,reason=TargetTracker(cfg).choose(seg,d.shape);assert candidate is not None,reason
  else:
   seg=segment_with_support_fallback(d,K,T,cfg,anchor,last,plane,prefer_current_support=True);matches=_matching_candidates(seg.candidates,anchor,last,cfg);assert len(matches)==1;candidate=matches[0][1]
  valid=rectification_rgb_valid_mask(camera,d.shape);refined=refine_candidate(rgb,d,K,T,candidate,cfg,rgb_valid=valid)
  original_mask=bridge.imgmsg_to_cv2(masks[ns],'mono8') if ns in masks else None
  verified=(None if original_mask is None or not np.any(original_mask) else bool(np.array_equal(original_mask,refined.mask)))
  if ns>=initial and ns!=bad:assert verified,'normal actual source mask must match bitwise'
  frames[ns]=dict(rgb=rgb,raw=raw,depth=d,T=T,candidate=candidate,refined=refined,valid=valid)
  row=dict(source_stamp_ns=ns,online_detail=statuses.get(ns,{}).get('detail'),mask_pixels=int(np.count_nonzero(refined.mask)),actual_published_mask_bitexact=verified,candidate_count=len(seg.candidates),matching_count=(None if ns<=initial else len(matches)),support=seg.metrics,refine_metrics=refined.metrics,quality=refined.quality,processing_wall_sec=time.perf_counter()-began)
  cv2.imwrite(str(OUT/(str(ns)+'_rgb.png')),rgb);cv2.imwrite(str(OUT/(str(ns)+'_mask.png')),refined.mask);overlay=rgb.copy();contours,_=cv2.findContours(refined.mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE);cv2.drawContours(overlay,contours,-1,(0,255,0),1);cv2.imwrite(str(OUT/(str(ns)+'_crop.png')),overlay[145:330,235:435])
  rows.append(row)
 except Exception as exc:rows.append(dict(source_stamp_ns=ns,error=str(exc),processing_wall_sec=time.perf_counter()-began))
assert initial in frames and good in frames and bad in frames
comparisons=[];current=frames[bad]
for ns,ref in frames.items():
 if ns==bad:continue
 rh,rc=_appearance(ref['rgb'],ref['refined'].mask);h,c=_appearance(current['rgb'],current['refined'].mask);colour=float(np.sqrt(max(0.,1.-np.sum(np.sqrt(h*rh)))));shape=float(cv2.matchShapes(rc,c,cv2.CONTOURS_MATCH_I1,0.))
 try:
  rr,proof=metric_seeded_identity_evidence(current['rgb'],[current['candidate']],ref['rgb'],ref['refined'].mask,minimum_quality=.5,rgb_valid=current['valid'],refine=lambda c:refine_candidate(current['rgb'],current['depth'],K,current['T'],c,cfg,rgb_valid=current['valid']));accepted=True;error=None
 except Exception as exc:accepted=False;error=str(exc);proof=None
 comparisons.append(dict(reference_source_ns=ns,current_source_ns=bad,reference_role=('immutable_acquired_frame' if ns==initial else 'actual_last_good' if ns==good else 'initial_confirmation_frame' if ns<initial else 'earlier_measured_frame'),reference_mask_pixels=int(np.count_nonzero(ref['refined'].mask)),current_mask_pixels=int(np.count_nonzero(current['refined'].mask)),mask_iou=iou(ref['refined'].mask,current['refined'].mask),colour_distance=colour,contour_distance=shape,accepted=accepted,error=error,identity=proof))
canonical_pairs=[]
for ns,f in frames.items():
 if ns>initial:canonical_pairs.append(dict(source_stamp_ns=ns,mask_iou_to_acquired=iou(frames[initial]['refined'].mask,f['refined'].mask)))
report=dict(record=str(REC),source_stamps=dict(anchor=initial,lastgood=good,gap=gap,firstloss=bad),actual_current_support_preference=True,actual_default16_rgb_unchanged=True,rows=rows,reference_comparisons_to_failed_frame=comparisons,canonical_iou_for_each_measured_frame=canonical_pairs,missing_exact_source_stamps=missing,source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path('src/alicia_grasp_modes/src/alicia_grasp_modes/tabletop.py'),Path('src/alicia_grasp_modes/src/alicia_grasp_modes/rgb_instance.py'),Path('src/alicia_grasp_modes/src/alicia_grasp_modes/metric_ambiguity_identity.py')]},anchor_footprint_roundoff_max_m=float(np.max(np.abs(recorded_footprint-reconstructed_footprint))),footprint_used='recorded fixed footprint',config=asdict(cfg),ros_calls=False,production_modified=False,limitations=['Initial confirming frames are reconstructed from their exact RGB-D; only acquired and subsequent published masks can be compared bitwise to archived ObjectMask.','No canonical-template acceptance policy has been added. Matrix results are diagnostic uses of the original identity function and do not supersede last-good source age, unique geometry, source/epoch/pose binding or runtime lifecycle.','This content replay is not a node publication or timing simulation.'])
p=OUT/'result.json';p.write_text(json.dumps(report,ensure_ascii=False,indent=2,default=lambda a:a.tolist() if isinstance(a,np.ndarray) else a)+'\n');print(json.dumps(dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),rows=[{k:r.get(k) for k in ['source_stamp_ns','mask_pixels','actual_published_mask_bitexact','matching_count','error']} for r in rows],comparisons=[{k:r[k] for k in ['reference_source_ns','reference_role','reference_mask_pixels','current_mask_pixels','mask_iou','colour_distance','contour_distance','accepted']} for r in comparisons],missing=missing),ensure_ascii=False,indent=2))
