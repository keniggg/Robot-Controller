"""Read-only-result summary; writes only external diagnostic evidence."""
import json,hashlib
from pathlib import Path
root=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_ros_20261009T003407Z/gen26_contact_analysis')
p=root/'contact_endpoint_counterfactual.json';d=json.loads(p.read_text());rows=d['rows'];ts=[t for r in rows for t in r['trials']]
def ext(key):return [min(t[key] for t in ts),max(t[key] for t in ts)]
r=rows[0];t=r['trials'][0]
rec=json.loads((root/'fused_reconstruction_final/result.json').read_text());disp=rec['registrations'][-1]['registration']['maximum_point_displacement_m']
summary=dict(scope='bounded original factory and constant-current-bias endpoint counterfactual; not exact canceled MoveIt path replay or execution authority',
 production_changed=False,factory_candidates=32,original_height_qualified_candidates=len(rows),bounded_endpoint_trials=len(ts),full_original_explicit_geometry_pass=sum(x['full_geometry_gate']['ok'] for x in ts),
 both_endpoint_paths_clear=sum(x['endpoint_pair_clear'] for x in ts),nominal_obb_collision=sum(any(b['overlap'] for b in x['nominal']['bodies']) for x in ts),command_obb_collision=sum(any(b['overlap'] for b in x['predicted_command']['bodies']) for x in ts),command_with_measured_points_inside=sum(any(b['measured_points_inside'] for b in x['predicted_command']['bodies']) for x in ts),
 command_q_within_original_joint_limits=sum(x['command_in_joint_limits'] for x in ts),max_weighted_ik_residual=max(x['weighted_ik_residual'] for x in ts),
 bias_sdk_minus_accepted_rad=d['sample']['bias'],opening_m=d['sample']['opening_m'],
 position_shift_projection_m={k:ext(k) for k in ('shift_normal_m','shift_jaw_m','shift_lateral_m','shift_insertion_m')},
 reconstructed_registration_max_point_displacement_m=disp,
 max_registration_plus_support_lift_plus_height_proposal_m=max(disp+r['support_lift_m']+abs(t['delta_m']) for r in rows for t in r['trials']),
 representative=dict(reconstructed_source_index=r['source_index'],variant_index=r['variant_index'],actual_runtime_candidate_identity_known=False,nominal=t['nominal'],command=t['predicted_command'],allowed_original_proposals_m=[x['delta_m'] for x in r['trials']],remaining_original_height_evidence_radius_m=r['registered_height']['translation_evidence_limit_m']),
 interpretation=[
 'The representative has 40.365mm target projection within 49.75mm open jaws; nominal fingers clear by4.093mm. Opening width alone is not the failure.',
 'Frozen target points are in registered base coordinates. Finger bodies use FK(q+current SDK-minus-accepted). Nineteen intersections concern measured points under that prediction, not observed physical collisions.',
 'Current observation-station bias projects upward about10.2mm and along the jaw about8.4mm in opposite physical variants. Upward motion can remove one collision while nominal tilted fingers remain blocked; no tested height produced both-safe endpoints.',
 'Original full explicit geometry, bilateral height bands and unchanged2.676768mm registration-height margin were used. This is not full MoveIt path or all-arm validation.',
 'Existing execution already remeasures SDK/accepted after true pregrasp settling and compensation; the pre-publication whole-contact precheck still extrapolates observation-station bias. No existing partial-contact authority allows skipping this failed admission.',
 'Do not change collision gates from this bounded counterfactual. Capture immediate bound failure audit in the next authorized initial pose; a separate pregrasp-only transaction would require explicit design and verification before contact qualification.'],
 limitations=d['limitations']+['No claim that the current measured bias persists after a different physical pregrasp.','Nine candidate poses and their two existing proposals are bounded coverage, not a mathematical nonexistence proof.'],
 evidence={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [p,root/'factory_replay.json',root/'fused_reconstruction_final/enclosed_geometry_manifest.json',Path(__file__)]})
out=root/'contact_endpoint_summary.json';out.write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n');print(json.dumps(dict(result=str(out),sha256=hashlib.sha256(out.read_bytes()).hexdigest(),max_total_displacement=summary['max_registration_plus_support_lift_plus_height_proposal_m'])))
