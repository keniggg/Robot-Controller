#!/usr/bin/env python3
"""Exact gen16 original fit, native RGB and same-source identity diagnosis."""
from pathlib import Path
import json,hashlib,sys
source=Path('.runtime/replay_gen15_support_gap_rgb_details.py').read_text()
prefix=source.split('rows=[];valid=')[0]
prefix=prefix.replace('20261009T052127.805554Z_6b8770587c5c4cd8abc679178eaa1c70','20261009T054827.257779Z_e15d068b793c43ada6713a78f421f2bb')
prefix=prefix.replace("OUT=NEW/'gen15_handoff_analysis/support_gap_rgb_replay_details'", "OUT=NEW/'gen16_handoff_analysis/support_gap_replay'")
prefix=prefix.replace("NEW/'gen15_handoff_analysis/arrivals_and_terminal.json'", "NEW/'gen16_handoff_analysis/first_loss_analysis.json'")
prefix=prefix.replace('gap=1791523444091633558','gap=1791524920594934463')
prefix=prefix.replace('needed={initial,good,bad,gap}','needed={initial,good,bad,gap,1791524920799158334}')
prefix=prefix.replace('observation_arrival_retry=True','observation_arrival_retry=False')
exec(compile(prefix,'sealed_gen16_original_loader','exec'))
from alicia_grasp_modes import tabletop as table

fitrows=[]
def trace(frame,event,arg):
 if frame.f_code is table._fit_plane.__code__ and event in ('exception','return'):
  loc=frame.f_locals
  if 'ratio' in loc:
   reason=(str(arg[1]) if event=='exception' else None)
   if event=='return' and arg is None:return trace
   fitrows.append(dict(event=event,reason=reason,outer_ring_unscaled_px=frame.f_back.f_locals.get('outer'),inlier_count=int(loc['best_count']),sample_count=len(loc['points']),ratio=float(loc['ratio']),residual_m=float(loc['best_residual'])))
 return trace
rows=[];identity=None
for ns in [good,gap,1791524920799158334,bad]:
 rgb,raw,d,T=decoded(ns);fitrows=[];started=time.perf_counter()
 try:
  sys.settrace(trace)
  result=segment_with_support_fallback(d,K,T,cfg,anchor,last,plane)
 finally:sys.settrace(None)
 row=dict(source_stamp_ns=ns,current_support_audit=result.metrics,original_fit_trace=list(fitrows),segmentation_wall_sec_with_trace=time.perf_counter()-started)
 if result.metrics.get('support_current_audit_available') is True:
  matches=_matching_candidates(result.candidates,anchor,last,cfg);row['matching_count']=len(matches);assert len(matches)==1
  c=matches[0][1];valid=rectification_rgb_valid_mask(camera,d.shape);refined=refine_candidate(rgb,d,K,T,c,cfg,rgb_valid=valid)
  row.update(mask_pixels=int(np.count_nonzero(refined.mask)),quality=refined.quality,refine_metrics=refined.metrics)
  if ns==good:assert np.array_equal(refined.mask,refmask);row['recorded_mask_bitexact']=True
  if ns==bad:
   rr,identity=metric_seeded_identity_evidence(rgb,[c],rgb_ref,refmask,minimum_quality=.5,rgb_valid=valid,refine=lambda cand:refine_candidate(rgb,d,K,T,cand,cfg,rgb_valid=valid))
   row['original_rgb_recovery_identity']=identity
  cv2.imwrite(str(OUT/(str(ns)+'_rgb.png')),rgb);cv2.imwrite(str(OUT/(str(ns)+'_mask.png')),refined.mask)
 rows.append(row)
report=dict(record=str(REC),scope='Original initial-station algorithm only: no observation-arrival marker or current-support preference introduced',rows=rows,current_recovery_identity=identity,original_last_good_mask_bitexact=True,config_unchanged=True,immutable_anchor_and_plane_preserved=True,source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path('src/alicia_grasp_modes/src/alicia_grasp_modes/tabletop.py'),Path('src/alicia_grasp_modes/src/alicia_grasp_modes/rgb_instance.py'),Path('src/alicia_grasp_modes/src/alicia_grasp_modes/metric_ambiguity_identity.py')]},anchor_footprint_reconstruction_max_difference_m=float(np.max(np.abs(recorded_footprint-reconstructed_footprint))),footprint_used='exact recorded immutable footprint',limitations=['sys.settrace observes original _fit_plane locals immediately before rejection/return. No fitted model, sample, random seed or acceptance threshold is changed.','Fit timing includes tracing and is not live processing latency.','This replay confirms support and RGB content only; the real terminal failure was later reference age enforcement, which is not bypassed or simulated here.'],ros_calls=False,production_modified=False)
p=OUT/'result.json';p.write_text(json.dumps(report,ensure_ascii=False,indent=2,default=lambda a:a.tolist() if isinstance(a,np.ndarray) else a)+'\n');print(json.dumps(dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),rows=[{k:r.get(k) for k in ['source_stamp_ns','original_fit_trace','matching_count','mask_pixels','original_rgb_recovery_identity']} for r in rows]),ensure_ascii=False,indent=2))
