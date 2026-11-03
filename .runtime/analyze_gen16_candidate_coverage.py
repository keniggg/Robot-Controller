"""Offline recorded audit only; no ROS, no production writes."""
import hashlib
import json
import math
from pathlib import Path
import numpy as np

EXT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
NEW = EXT / 'ros_log/full_latest_no_skin_20261011T000625Z'
AUDIT = EXT / 'grasp_records/20261011T113712.131215Z_6bfa27d92a2944fbb1b445f2c8ebdd6f/keyframes_audits/87c4f588d028eb67065a14ea82b7f71ee63ffe5056db5aa7d32798dd0009b10a.json'
INPUT = NEW / 'gen16_analysis/contact_path_replay_inputs/req13_rejected_contact_inputs.json'
WORK = Path(__file__).resolve().parents[1]

def rotation(q):
    x,y,z,w = np.asarray(q,dtype=float)/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()

d = json.loads(AUDIT.read_text())
r = json.loads(INPUT.read_text())
scene = d['multiview_surface']['fused_scene_geometry']
normal = np.asarray(scene['support_normal_base'])
bx = np.array([1.,0.,0.]);bx -= normal*np.dot(bx,normal);bx /= np.linalg.norm(bx)
by = np.cross(normal,bx)
rows=[]
for i,e in enumerate(d['stable_evaluations']):
    poses={s['stage']:s for s in e['execution_sequence']['stages']}
    rot=rotation(poses['grasp']['quaternion_xyzw']); jaw=rot[:,1]
    after=(e.get('contact_height_placement') or {}).get('after') or {}
    nominal=after.get('nominal') or {}
    final=e.get('final_contact_height_margin') or {}
    rows.append(dict(eval_index=i,source_index=e['source_index'],track_id=e['tracking']['track_id'],
        variant_index=e['variant_index'],jaw_azimuth_signed_deg=math.degrees(math.atan2(np.dot(jaw,by),np.dot(jaw,bx)))%360.,
        jaw_azimuth_mod180_deg=math.degrees(math.atan2(np.dot(jaw,by),np.dot(jaw,bx)))%180.,
        insertion_tilt_deg=math.degrees(math.acos(np.clip(np.dot(-rot[:,2],normal),-1.,1.))),
        planned_tilt_deg=e['adaptive_stage_profile']['tilt_deg'],
        pre_registration_placed_contact_margin_ok=after.get('ok'),
        pre_registration_overlap_m=nominal.get('actual_overlap_m'),
        pre_registration_symmetric_normal_margin_m=after.get('maximum_symmetric_normal_translation_m'),
        original_normal_evidence_limit_m=after.get('translation_evidence_limit_m'),
        contact_height_placement_m=(e.get('contact_height_placement') or {}).get('normal_translation_m'),
        final_registered_contact_margin_ok=final.get('ok'),
        endpoint_probe_status=(e.get('endpoint_ik_search_probe') or {}).get('status'),
        strict_attempted=e.get('moveit') is not None,
        final_rejection=e.get('final_contact_rejection'),
        execution_pose_frame='current_snapshot_before_final_registration',
        grasp_pose_xyz_xyzw=poses['grasp']['position_m']+poses['grasp']['quaternion_xyzw']))
height=[]
for item in r['height_retry_reports']:
    e=item['evidence'];height.append(dict(log_line=item['log_line'],ros_stamp_sec=item['ros_stamp_sec'],
        evidence_keys=list(e),evidence=e))
tabletop=d['remote_diagnostics']['tabletop_geometry']
report=dict(schema='gen16_recorded_candidate_coverage_v1',audit_path=str(AUDIT),audit_sha256=sha(AUDIT),
    receipt_input_path=str(INPUT),receipt_input_sha256=sha(INPUT),
    candidate_rows=rows,selection=d['moveit_selection'],
    generation={k:tabletop[k] for k in ('sampled_angles_deg','proposal_count','bounded_proposal_count',
        'materialized_proposal_count','materialized_count','materialized_total_count','contact_boundary_profiles',
        'surface_evidence_rejections','proposal_reach_rejections','rejection_counts')},
    deduplication=d['pipeline_funnel']['acceptance_diagnostics'],
    exact_fused_source_metadata=scene,
    limitations=['Stable rows are pre-final-registration poses; use receipt poses for the 8 actual strict failures.',
        'All 34 pass recorded initial placement normal margin; this is not complete path or final registration proof.',
        'The replay cloud is raw+fused union, not independently recoverable fused XYZ with per-point view votes.',
        'No new IK, collision or motion claim is made by this read-only extraction.'],
    production_changed=False,ros_called=False)
out=NEW/'gen16_analysis/candidate_coverage_and_contact_evidence.json'
out.write_text(json.dumps(report,ensure_ascii=False,sort_keys=True,indent=2)+'\n')
print(json.dumps(dict(path=str(out),sha256=sha(out),rows=len(rows),strict_evals=[x['eval_index'] for x in rows if x['strict_attempted']],
    all_initial_normal_margin_ok=all(x['pre_registration_placed_contact_margin_ok'] is True for x in rows),
    azimuths=sorted(set(round(x['jaw_azimuth_mod180_deg'],6) for x in rows)),
    upright_rows=sum(x['insertion_tilt_deg']<1e-6 for x in rows)),indent=2))
