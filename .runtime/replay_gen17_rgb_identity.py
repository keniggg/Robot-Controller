#!/usr/bin/env python3
"""Exact sealed gen17 identity: measured plane and existing RGB crop comparisons."""
from pathlib import Path
import json,hashlib,itertools
text=Path('.runtime/replay_gen15_support_gap_rgb_details.py').read_text();prefix=text.split('rows=[];valid=')[0]
prefix=prefix.replace('20261009T052127.805554Z_6b8770587c5c4cd8abc679178eaa1c70','20261009T055520.443822Z_b937f5b63a8b4e3cb90f741b2eb36871').replace("OUT=NEW/'gen15_handoff_analysis/support_gap_rgb_replay_details'","OUT=NEW/'gen17_handoff_analysis/rgb_identity_replay'").replace("NEW/'gen15_handoff_analysis/arrivals_and_terminal.json'","NEW/'gen17_handoff_analysis/first_loss_analysis.json'").replace('gap=1791523444091633558','gap=1791525329088324069').replace('needed={initial,good,bad,gap}','needed={initial,good,bad,gap,1791525329292920351}').replace('observation_arrival_retry=True','observation_arrival_retry=False')
exec(compile(prefix,'sealed_gen17_original_loader','exec'))
from alicia_grasp_modes.tabletop import segment
frames={};rows=[]
def iou(a,b):return float(np.count_nonzero((a>0)&(b>0))/np.count_nonzero((a>0)|(b>0)))
def box(m):
 yy,xx=np.nonzero(m);return [int(xx.min()),int(yy.min()),int(xx.max())+1,int(yy.max())+1]
for ns in [good,bad]:
 rgb,raw,d,T=decoded(ns);valid=rectification_rgb_valid_mask(camera,d.shape);variants={};rawrows={}
 for geometry in ['original_wrapper','current_segment']:
  result=(segment_with_support_fallback(d,K,T,cfg,anchor,last,plane) if geometry=='original_wrapper' else segment(d,K,T,cfg,target_position_base=anchor.position_base,plane_base=plane,target_footprint_base=anchor.support_footprint_base,fit_current_support=True))
  matching=_matching_candidates(result.candidates,anchor,last,cfg);assert len(matching)==1,dict(source=ns,geometry=geometry,matching=len(matching));candidate=matching[0][1]
  rawrows[geometry]=dict(support=result.metrics,metric_candidates=len(result.candidates),matching_count=len(matching),native_pixels=int(np.count_nonzero(candidate.mask)),native_bbox=box(candidate.mask),candidate_metrics=candidate.metrics)
  for arrival in [False,True]:
   key=geometry+('_arrival48' if arrival else '_default16');began=time.perf_counter()
   try:refined=refine_candidate(rgb,d,K,T,candidate,cfg,observation_arrival_retry=arrival,rgb_valid=valid)
   except Exception as exc:variants[key]=dict(error=str(exc),raw=candidate,arrival=arrival);continue
   variants[key]=dict(candidate=refined,raw=candidate,arrival=arrival)
   cv2.imwrite(str(OUT/(str(ns)+'_'+key+'_mask.png')),refined.mask)
   overlay=rgb.copy();contours,_=cv2.findContours(refined.mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE);cv2.drawContours(overlay,contours,-1,(0,255,0),1)
   cv2.imwrite(str(OUT/(str(ns)+'_'+key+'_overlay.png')),overlay);cv2.imwrite(str(OUT/(str(ns)+'_'+key+'_crop.png')),overlay[145:330,235:435])
   rows.append(dict(source_stamp_ns=ns,profile=key,mask_pixels=int(np.count_nonzero(refined.mask)),bbox=box(refined.mask),quality=refined.quality,metrics=refined.metrics,processing_wall_sec=time.perf_counter()-began,recorded_last_good_mask_bitexact=(bool(np.array_equal(refined.mask,refmask)) if ns==good and key=='original_wrapper_default16' else None)))
 frames[ns]=dict(rgb=rgb,depth=d,T=T,valid=valid,variants=variants,raw=rawrows)
assert np.array_equal(frames[good]['variants']['original_wrapper_default16']['candidate'].mask,refmask)
comparisons=[]
for refname,newname in itertools.product(frames[good]['variants'],frames[bad]['variants']):
 ref=frames[good]['variants'][refname];new=frames[bad]['variants'][newname]
 if 'candidate' not in ref or 'candidate' not in new:comparisons.append(dict(reference=refname,current=newname,accepted=False,error='refinement_failed'));continue
 rm=ref['candidate'].mask;cm=new['candidate'].mask;rh,rc=_appearance(frames[good]['rgb'],rm);h,c=_appearance(frames[bad]['rgb'],cm);colour=float(np.sqrt(max(0.,1.-np.sum(np.sqrt(h*rh)))));shape=float(cv2.matchShapes(rc,c,cv2.CONTOURS_MATCH_I1,0.))
 try:
  rr,proof=metric_seeded_identity_evidence(frames[bad]['rgb'],[new['raw']],frames[good]['rgb'],rm,minimum_quality=.5,rgb_valid=frames[bad]['valid'],refine=lambda cand:refine_candidate(frames[bad]['rgb'],frames[bad]['depth'],K,frames[bad]['T'],cand,cfg,observation_arrival_retry=new['arrival'],rgb_valid=frames[bad]['valid']));accepted=True;error=None
 except Exception as exc:accepted=False;error=str(exc);proof=None
 comparisons.append(dict(reference=refname,current=newname,accepted=accepted,error=error,mask_iou=iou(rm,cm),colour_distance=colour,contour_distance=shape,identity=proof))
result=dict(record=str(REC),source_stamps_ns=dict(anchor=initial,last_good=good,first_gap=gap,second_gap=1791525329292920351,first_loss=bad),original_last_good_mask_bitexact=True,raw_segmentation={str(ns):f['raw'] for ns,f in frames.items()},refinement_rows=rows,identity_comparisons=comparisons,config=asdict(cfg),source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path('src/alicia_grasp_modes/src/alicia_grasp_modes/tabletop.py'),Path('src/alicia_grasp_modes/src/alicia_grasp_modes/rgb_instance.py'),Path('src/alicia_grasp_modes/src/alicia_grasp_modes/metric_ambiguity_identity.py')]},footprint_roundoff_max_m=float(np.max(np.abs(recorded_footprint-reconstructed_footprint))),footprint_used='exact recorded immutable footprint',ros_calls=False,production_modified=False,limits_changed=False,limitations=['Only original_wrapper_default16/default16 is the real initial-station production configuration. The other plane/crop combinations are explicit offline counterfactuals, not an authorized initial-station policy change.','Each variant uses exact same recorded pixels, depth, TF and immutable target reference. No new pixels, old-mask seeds, merged candidate or relaxed identity threshold is supplied.','Pure fit/refinement/identity analysis does not replace node freshness, lifecycle, handoff or publication checks.'])
p=OUT/'result.json';p.write_text(json.dumps(result,ensure_ascii=False,indent=2,default=lambda a:a.tolist() if isinstance(a,np.ndarray) else a)+'\n');print(json.dumps(dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),raw=result['raw_segmentation'],rows=[{k:r[k] for k in ['source_stamp_ns','profile','mask_pixels','bbox','quality','recorded_last_good_mask_bitexact']} for r in rows],comparisons=[{k:r.get(k) for k in ['reference','current','accepted','mask_iou','colour_distance','contour_distance','error']} for r in comparisons]),ensure_ascii=False,indent=2))
