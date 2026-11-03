#!/usr/bin/env python3
"""Actual patched wrapper on every saved source; no transport or control calls."""
from pathlib import Path
import json,hashlib
files=[Path(p) for p in ['src/alicia_grasp_modes/src/alicia_grasp_modes/tabletop.py','src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py','src/alicia_grasp_modes/src/alicia_grasp_modes/rgb_instance.py','src/alicia_grasp_modes/src/alicia_grasp_modes/metric_ambiguity_identity.py']]
before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
source=Path('.runtime/replay_gen15_support_gap_rgb_details.py').read_text();prefix=source.split('rows=[];valid=')[0]
prefix=prefix.replace("OUT=NEW/'gen15_handoff_analysis/support_gap_rgb_replay_details'", "OUT=NEW/'gen15_handoff_analysis/patched_current_support_wrapper'")
prefix=prefix.replace('needed={initial,good,bad,gap}', "needed={initial,good,bad,gap,*[row['source_stamp_ns'] for row in summary['saved_third_arrival_images'] if 1791523443295139074<=row['source_stamp_ns']<=1791523447459038972]}")
prefix=prefix.replace('<=bad+500_000_000','<=1791523447459038972+500_000_000')
exec(compile(prefix,'sealed_gen15_original_loader','exec'))
from alicia_grasp_modes.tabletop import segment,_validate_support_plane_update

def mask_iou(a,b):return float(np.count_nonzero((a>0)&(b>0))/np.count_nonzero((a>0)|(b>0)))
frames=[ns for ns in sorted(needed) if ns>=1791523443295139074];state_last=anchor;reference=None;pending=False;rows=[];failure=None
original_plane=plane.copy();frozen_anchor_position=anchor.position_base.copy();frozen_anchor_footprint=anchor.support_footprint_base.copy();first_gap_recovery=None
for ns in frames:
 began=time.perf_counter();rgb,raw,d,T=decoded(ns);row=dict(source_stamp_ns=ns,original_online_detail=statuses.get(ns,{}).get('detail'),original_online_frame_timing=statuses.get(ns,{}).get('frame_timing'),processing_wall_sec=None)
 try:
  result=segment_with_support_fallback(d,K,T,cfg,anchor,state_last,plane,prefer_current_support=True)
  row.update(support=result.metrics,raw_candidate_count=len(result.candidates))
  if result.metrics.get('support_current_audit_available') is not True:
   pending=True;row.update(state='pending_without_geometry',reason=result.metrics.get('support_current_audit_reason'),retained_reference_source_ns=None if reference is None else reference['source']);rows.append(row);row['processing_wall_sec']=time.perf_counter()-began;continue
  metrics=result.metrics
  assert cfg.support_min_ratio<=metrics['support_current_audit_inlier_ratio']<=1.
  assert 0<=metrics['support_current_audit_residual_m']<=cfg.support_threshold_m
  assert metrics['support_current_audit_quadrants'] in (3,4)
  assert 0<=metrics['support_reference_angle_deg']<=cfg.support_max_normal_change_deg
  assert 0<=metrics['support_reference_max_distance_m']<=cfg.support_max_local_shift_m
  assert metrics['support_reference_footprint_vertices']==len(anchor.support_footprint_base)
  _validate_support_plane_update(plane,result.plane_base,anchor.position_base,cfg,anchor.support_footprint_base)
  matches=_matching_candidates(result.candidates,anchor,state_last,cfg);row['matching_count']=len(matches)
  assert len(matches)==1,dict(reason='original_unique_identity',matching_count=len(matches))
  cand=matches[0][1];valid=rectification_rgb_valid_mask(camera,d.shape)
  refined=refine_candidate(rgb,d,K,T,cand,cfg,observation_arrival_retry=True,rgb_valid=valid)
  assert np.isfinite(cand.quality) and cand.quality>=.5 and refined.quality>=.5
  row.update(mask_pixels=int(np.count_nonzero(refined.mask)),quality=refined.quality,refined_metrics=refined.metrics,previous_mask_iou=None if reference is None else mask_iou(reference['refined'].mask,refined.mask))
  direct=segment(d,K,T,cfg,target_position_base=anchor.position_base,plane_base=plane,target_footprint_base=anchor.support_footprint_base,fit_current_support=True)
  direct_matches=_matching_candidates(direct.candidates,anchor,state_last,cfg)
  assert len(direct_matches)==1
  direct_refined=refine_candidate(rgb,d,K,T,direct_matches[0][1],cfg,observation_arrival_retry=True,rgb_valid=valid)
  row['raw_current_segment_mask_bitexact']=bool(np.array_equal(refined.mask,direct_refined.mask));assert row['raw_current_segment_mask_bitexact']
  if pending:
   assert reference is not None,'missing real last good source'
   recovered,identity=metric_seeded_identity_evidence(rgb,[cand],reference['rgb'],reference['refined'].mask,minimum_quality=.5,rgb_valid=valid,refine=lambda c:refine_candidate(rgb,d,K,T,c,cfg,observation_arrival_retry=True,rgb_valid=valid))
   row.update(state='qualified_unique_measured_recovery',identity=identity,retained_reference_source_ns=reference['source'],reference_to_current_source_delta_sec=(ns-reference['source'])*1e-9)
   if first_gap_recovery is None:first_gap_recovery=dict(row)
  else:row['state']='ready'
  state_last=cand;reference=dict(source=ns,rgb=rgb,refined=refined);pending=False
  cv2.imwrite(str(OUT/(str(ns)+'_patched_mask.png')),refined.mask)
  if ns in [good,bad,1791523443295139074,1791523443657088041]:
   old_path=NEW/'gen15_handoff_analysis/current_support_rgb_counterfactual'/(str(ns)+'_current_support_mask.png');baseline=cv2.imread(str(old_path),cv2.IMREAD_GRAYSCALE);row['previous_raw_segment_counterfactual_mask_bitexact']=bool(np.array_equal(refined.mask,baseline));assert row['previous_raw_segment_counterfactual_mask_bitexact']
 except Exception as exc:
  row.update(state='rejected',reason=str(exc),error_type=type(exc).__name__);failure=dict(row);rows.append(row);row['processing_wall_sec']=time.perf_counter()-began;break
 row['processing_wall_sec']=time.perf_counter()-began;rows.append(row)
 assert np.array_equal(plane,original_plane) and np.array_equal(anchor.position_base,frozen_anchor_position) and np.array_equal(anchor.support_footprint_base,frozen_anchor_footprint)
after={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files};assert before==after,'production changed during replay'
result=dict(record=str(REC),status='passed' if failure is None else 'rejected',source_sha256=after,source_unchanged=True,frames=rows,saved_frame_count=len(frames),evaluated_frame_count=len(rows),ready_count=sum(r['state']=='ready' for r in rows),gap_count=sum(r['state']=='pending_without_geometry' for r in rows),recovery_count=sum(r['state']=='qualified_unique_measured_recovery' for r in rows),first_gap_recovery=first_gap_recovery,first_failure=failure,anchor_plane_footprint_unchanged=True,production_modified=False,no_ros_calls=True,limitations=['Actual patched segmentation wrapper, native unique matching, original RGB refinement and original identity function are executed. Unknown node transport, source/lease/marker guards and object publication are not simulated here.','The first two online qualified arrival exact RGB-D sources were not saved. Start is the third qualified real source; some saved 5Hz frames did not undergo original online evaluation.','After the real terminal frame these are original archived RGB-D counterfactuals, never a resurrection of the retired live target.','Processing wall time includes duplicated raw-segment comparison and is not a real node latency measurement; no mocked timing grants freshness.'])
p=OUT/'result.json';p.write_text(json.dumps(result,ensure_ascii=False,indent=2,default=lambda a:a.tolist() if isinstance(a,np.ndarray) else a)+'\n');print(json.dumps(dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),status=result['status'],saved_frame_count=len(frames),evaluated_frame_count=len(rows),ready_count=result['ready_count'],gap_count=result['gap_count'],recovery_count=result['recovery_count'],failure=failure,first_gap_recovery=None if first_gap_recovery is None else {k:first_gap_recovery[k] for k in ['source_stamp_ns','identity','reference_to_current_source_delta_sec']},wall_sec=[r['processing_wall_sec'] for r in rows]),ensure_ascii=False,indent=2))
