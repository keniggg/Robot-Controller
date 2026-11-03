#!/usr/bin/env python3
"""One specified timely-captured source through unchanged pure measurement gates."""
from pathlib import Path
import json,hashlib,time
s=Path('.runtime/replay_gen15_support_gap_rgb_details.py').read_text()
prefix=s.split('rgb,raw,dep,T=decoded(bad);sg=')[0]
prefix=prefix.replace('20261009T052127.805554Z_6b8770587c5c4cd8abc679178eaa1c70','20261009T084503.785647Z_98a11257317947ada4721dad54fdb028')
prefix=prefix.replace("OUT=NEW/'gen15_handoff_analysis/support_gap_rgb_replay_details'","OUT=NEW/'gen20_handoff_analysis/timely_completion_replay'")
prefix=prefix.replace("NEW/'gen15_handoff_analysis/arrivals_and_terminal.json'","NEW/'gen20_handoff_analysis/first_loss_analysis.json'")
prefix=prefix.replace('gap=1791523444091633558','gap=1791535531941278457')
prefix=prefix.replace('observation_arrival_retry=True','observation_arrival_retry=False')
prefix=prefix.replace('cfg,anchor,tracker.last,plane)','cfg,anchor,tracker.last,plane,prefer_current_support=True)').replace('cfg,anchor,last,plane)','cfg,anchor,last,plane,prefer_current_support=True)')
exec(compile(prefix,'gen20_exact_sealed_loader','exec'))
from alicia_grasp_modes.tabletop import _validate_support_plane_update
from alicia_grasp_modes.rgb_observation_handoff import _mask
assert good==1791535531442678928 and bad==1791535532141202449
OUT.mkdir(parents=True,exist_ok=True)
original=summary['first_loss']['message']; observed=original['first_loss']['observed_stamp_ns']
template_provenance=statuses[initial]['initial_station_identity_template']
reference_rgb,_,_,_=decoded(initial)
reference_mask=_mask(bridge.imgmsg_to_cv2(masks[initial],'mono8'),reference_rgb.shape[:2])
assert template_provenance['source_stamp_ns']==initial
assert hashlib.sha256(reference_rgb.tobytes()).hexdigest()==template_provenance['source_rgb_sha256']
assert hashlib.sha256(reference_mask.tobytes()).hexdigest()==template_provenance['refined_mask_sha256']
report=dict(record=str(REC),source_stamps=dict(acquired_fixed_template=initial,last_good_metric_reference=good,chosen_current=bad,actual_loss_observed=observed),capture_gap_sec=(bad-good)*1e-9,current_age_at_actual_loss_sec=(observed-bad)*1e-9,last_good_age_at_actual_loss_sec=(observed-good)*1e-9,original_max_age_sec=.8,actual_typed_pending=original['source_qualification'],fixed_template_provenance=template_provenance,template_rgb_and_mask_match_published_hashes=True,last_good_replay_mask_bitexact=True,immutable_anchor_footprint_max_roundoff_m=float(np.max(np.abs(reconstructed_footprint-recorded_footprint))),reference_footprint_used='exact recorded immutable footprint',config=asdict(cfg),camera=camera,source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path('src/alicia_grasp_modes/src/alicia_grasp_modes/tabletop.py'),Path('src/alicia_grasp_modes/src/alicia_grasp_modes/rgb_instance.py'),Path('src/alicia_grasp_modes/src/alicia_grasp_modes/metric_ambiguity_identity.py')]},ros_calls=False,production_modified=False,geometry_or_motion_authorized=False,limitations=['Only the specified original current frame and immutable acquired template are evaluated; no template or later-frame selection.','Pure original support segmentation, tracking and RGB identity functions; this does not simulate the node lease/commit protocol or bypass production time checks.','Recorded source, reference and actual observation timestamps are preserved. No mocked ROS now or falsely fresh source is supplied.','Recorded runtime failed before RGB revalidation when last-good age exceeded .8 s. Pixel-gate success would support a separate bounded timely-capture contract but is not proof of runtime recovery or grasp success.'])
began=time.perf_counter();stage='current_support_segmentation'
try:
 rgb,raw,dep,T=decoded(bad)
 current=segment_with_support_fallback(dep,K,T,cfg,anchor,last,plane,prefer_current_support=True)
 m=current.metrics;report['segmentation_metrics']=m
 assert m.get('support_current_audit_available') is True
 assert m['support_current_audit_quadrants'] in (3,4)
 assert cfg.support_min_ratio<=m['support_current_audit_inlier_ratio']<=1.
 assert 0.<=m['support_current_audit_residual_m']<=cfg.support_threshold_m
 assert 0.<=m['support_reference_angle_deg']<=cfg.support_max_normal_change_deg
 assert 0.<=m['support_reference_max_distance_m']<=cfg.support_max_local_shift_m
 assert m['support_reference_footprint_vertices']==len(recorded_footprint)
 assert m['support_source']=='current_ring_fit'
 _validate_support_plane_update(plane,current.plane_base,anchor.position_base,cfg,recorded_footprint)
 report['original_recovery_support_predicates_pass']=True
 stage='unique_metric_tracking';matches=_matching_candidates(current.candidates,anchor,last,cfg)
 staged=TargetTracker(cfg);staged.anchor,staged.last,staged.plane_base=anchor,last,plane.copy()
 candidate,reason=staged.choose(current,dep.shape)
 report.update(candidate_count=len(current.candidates),matching_count=len(matches),tracker_choice=reason)
 assert candidate is not None and reason=='tracked' and len(matches)==1
 stage='original_rgb_identity';valid=rectification_rgb_valid_mask(camera,dep.shape)
 refined,identity=metric_seeded_identity_evidence(rgb,[candidate],reference_rgb,reference_mask,minimum_quality=.5,rgb_valid=valid,refine=lambda c:refine_candidate(rgb,dep,K,T,c,cfg,rgb_valid=valid))
 identity['identity_template_source']=template_provenance
 report.update(passed=True,identity=identity,refine_metrics=refined[0].metrics,refined_quality=refined[0].quality,current_geometry_source_stamp_ns=bad,current_metric_geometry_derived_from_reference=False,reference_plane_unchanged=bool(np.array_equal(staged.plane_base,plane)))
 for name,col,mask in [('fixed_template',reference_rgb,reference_mask),('current',rgb,refined[0].mask)]:
  cv2.imwrite(str(OUT/(name+'_rgb.png')),col);cv2.imwrite(str(OUT/(name+'_mask.png')),mask)
  overlay=col.copy();contours,_=cv2.findContours((mask>0).astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE);cv2.drawContours(overlay,contours,-1,(0,255,0),1);cv2.imwrite(str(OUT/(name+'_overlay.png')),overlay)
except Exception as exc:
 report.update(passed=False,failed_stage=stage,error_type=type(exc).__name__,error=str(exc))
report['offline_processing_wall_sec']=time.perf_counter()-began
p=OUT/'result.json';p.write_text(json.dumps(report,ensure_ascii=False,indent=2,default=lambda v:v.tolist() if isinstance(v,np.ndarray) else v)+'\n')
print(json.dumps(dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),**{k:report.get(k) for k in ['passed','failed_stage','error','capture_gap_sec','current_age_at_actual_loss_sec','last_good_age_at_actual_loss_sec','candidate_count','matching_count','segmentation_metrics','identity','offline_processing_wall_sec']}),ensure_ascii=False,indent=2))
