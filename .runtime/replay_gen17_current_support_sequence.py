#!/usr/bin/env python3
"""Serial original wrapper + default16 RGB on available sealed initial-station frames."""
from pathlib import Path
import json,hashlib
files=[Path(p) for p in ['src/alicia_grasp_modes/src/alicia_grasp_modes/tabletop.py','src/alicia_grasp_modes/src/alicia_grasp_modes/rgb_instance.py','src/alicia_grasp_modes/src/alicia_grasp_modes/metric_ambiguity_identity.py']];sha_before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
text=Path('.runtime/replay_gen15_support_gap_rgb_details.py').read_text();prefix=text.split('rows=[];valid=')[0]
prefix=prefix.replace('20261009T052127.805554Z_6b8770587c5c4cd8abc679178eaa1c70','20261009T055520.443822Z_b937f5b63a8b4e3cb90f741b2eb36871').replace("OUT=NEW/'gen15_handoff_analysis/support_gap_rgb_replay_details'","OUT=NEW/'gen17_handoff_analysis/current_support_sequence'").replace("NEW/'gen15_handoff_analysis/arrivals_and_terminal.json'","NEW/'gen17_handoff_analysis/first_loss_analysis.json'").replace('gap=1791523444091633558','gap=1791525329088324069').replace('needed={initial,good,bad,gap}',"needed={initial,good,bad,gap,*[row['message'].get('source_stamp_ns',0) for row in summary['all_gen17_status_before_first_loss'] if row['message'].get('source_stamp_ns',0)>=initial]}").replace('observation_arrival_retry=True','observation_arrival_retry=False')
prefix=prefix.replace('for ns in needed),{ns:list(x) for ns,x in images.items()}','for ns in {initial,good,bad,gap}),{ns:list(x) for ns,x in images.items()}')
exec(compile(prefix,'sealed_gen17_sequence_loader','exec'))
from alicia_grasp_modes.tabletop import _validate_support_plane_update
available=sorted(ns for ns,values in images.items() if ns>=initial and set(values)=={'rgb','depth'});missing=sorted(needed-set(available));missing=[ns for ns in missing if ns>=initial]
rgb0,raw0,d0,T0=decoded(initial);firstref=refine_candidate(rgb0,d0,K,T0,anchor,cfg,rgb_valid=rectification_rgb_valid_mask(camera,d0.shape));assert np.array_equal(firstref.mask,bridge.imgmsg_to_cv2(masks[initial],'mono8')),'acquired frame must match original mask bitwise'
state=TargetTracker(cfg);state.anchor=anchor;state.last=anchor;state.plane_base=plane.copy();frozen_plane=state.plane_base.copy();frozen_footprint=anchor.support_footprint_base.copy();frozen_position=anchor.position_base.copy();ref=dict(source=initial,rgb=rgb0,refined=firstref);pending=False;rows=[];first_failure=None
for ns in [ns for ns in available if ns>initial]:
 started=time.perf_counter();rgb,raw,d,T=decoded(ns);r=dict(source_stamp_ns=ns,original_online_detail=statuses.get(ns,{}).get('detail'),reference_source_ns=ref['source'],reference_to_current_source_sec=(ns-ref['source'])*1e-9)
 try:
  result=segment_with_support_fallback(d,K,T,cfg,anchor,state.last,plane,prefer_current_support=True);r.update(support=result.metrics,candidate_count=len(result.candidates))
  if result.metrics.get('support_current_audit_available') is not True:
   pending=True;r.update(state='pending_without_geometry',reason=result.metrics.get('support_current_audit_reason'));rows.append(r);r['processing_wall_sec']=time.perf_counter()-started;continue
  m=result.metrics
  assert cfg.support_min_ratio<=m['support_current_audit_inlier_ratio']<=1.
  assert 0<=m['support_current_audit_residual_m']<=cfg.support_threshold_m
  assert m['support_current_audit_quadrants'] in (3,4)
  _validate_support_plane_update(plane,result.plane_base,anchor.position_base,cfg,anchor.support_footprint_base)
  staged=TargetTracker(cfg);staged.anchor,staged.last,staged.plane_base=state.anchor,state.last,state.plane_base.copy();candidate,reason=staged.choose(result,d.shape);assert candidate is not None and reason=='tracked',reason
  r['matching_count']=len(_matching_candidates(result.candidates,anchor,state.last,cfg));assert r['matching_count']==1
  valid=rectification_rgb_valid_mask(camera,d.shape);candidate_rgb=refine_candidate(rgb,d,K,T,candidate,cfg,rgb_valid=valid);assert candidate.quality>=.5 and candidate_rgb.quality>=.5
  r.update(mask_pixels=int(np.count_nonzero(candidate_rgb.mask)),refine_metrics=candidate_rgb.metrics,quality=candidate_rgb.quality)
  if pending:
   rr,proof=metric_seeded_identity_evidence(rgb,[candidate],ref['rgb'],ref['refined'].mask,minimum_quality=.5,rgb_valid=valid,refine=lambda c:refine_candidate(rgb,d,K,T,c,cfg,rgb_valid=valid))
   r.update(state='qualified_unique_measured_recovery',identity=proof)
  else:r['state']='ready'
  if ns in [good,bad]:
   baseline=cv2.imread(str(NEW/'gen17_handoff_analysis/rgb_identity_replay'/(str(ns)+'_current_segment_default16_mask.png')),cv2.IMREAD_GRAYSCALE);r['same_as_prior_same_source_current16_counterfactual']=bool(np.array_equal(candidate_rgb.mask,baseline));assert r['same_as_prior_same_source_current16_counterfactual']
  state=staged;ref=dict(source=ns,rgb=rgb,refined=candidate_rgb);pending=False
  cv2.imwrite(str(OUT/(str(ns)+'_mask.png')),candidate_rgb.mask)
 except Exception as exc:
  r.update(state='rejected',reason=str(exc),error_type=type(exc).__name__);first_failure=r;rows.append(r);r['processing_wall_sec']=time.perf_counter()-started;break
 r['processing_wall_sec']=time.perf_counter()-started;rows.append(r)
 assert state.anchor is anchor and np.array_equal(state.plane_base,frozen_plane) and np.array_equal(anchor.support_footprint_base,frozen_footprint) and np.array_equal(anchor.position_base,frozen_position)
after={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files};assert sha_before==after
result=dict(status='passed' if first_failure is None else 'rejected',record=str(REC),source_sha256=after,source_unchanged=True,initial_acquired_source_ns=initial,initial_acquired_mask_bitexact=True,requested_online_source_count=len([ns for ns in needed if ns>=initial]),available_exact_source_count=len(available),missing_online_exact_source_stamps=missing,frames=rows,first_failure=first_failure,ready_count=sum(r['state']=='ready' for r in rows),gap_count=sum(r['state']=='pending_without_geometry' for r in rows),recovery_count=sum(r['state']=='qualified_unique_measured_recovery' for r in rows),original_anchor_position_plane_footprint_unchanged=True,original_default16_rgb_unchanged=True,production_modified=False,ros_calls=False,limitations=['Pure actual patched wrapper, original tracker uniqueness and default16 RGB identity functions on exact available saved sources; no initial-station qualification or ROS publication/lease/source-age authority is fabricated.','Missing source frames are listed explicitly. This is the available-source serial counterfactual, not a claim to replay all original online evaluations.','The first acquired recorded mask is bitwise reproduced and used only as the original reference; it is never supplied as a new RGB segmentation seed.','No refit is allowed to discard a second original eligible metric identity. A true ambiguous/error frame stops this counterfactual.','Processing time is offline wall time; it does not prove live frame freshness.'])
p=OUT/'result.json';p.write_text(json.dumps(result,ensure_ascii=False,indent=2,default=lambda a:a.tolist() if isinstance(a,np.ndarray) else a)+'\n');print(json.dumps(dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),status=result['status'],available=len(available),missing=missing,ready=result['ready_count'],gap=result['gap_count'],recovery=result['recovery_count'],failure=first_failure,recoveries=[{k:r[k] for k in ['source_stamp_ns','reference_source_ns','identity']} for r in rows if r['state']=='qualified_unique_measured_recovery']),ensure_ascii=False,indent=2))
