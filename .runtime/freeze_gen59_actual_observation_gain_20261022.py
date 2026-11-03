"""Freeze GEN58 view-gain inputs and compare finite geometry-only proposals."""
import os
os.environ.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
from pathlib import Path
import json,hashlib,math,time
import numpy as np
W=Path(__file__).resolve().parents[1]
B=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N=B/'ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1'
O=N/'observation_pose_fix/actual_gain_counterfactual_r1'
C=W/'.runtime/gen59_observation_pose_candidate_r1'
assert Path('/media/zhuyupei/SanStand').is_mount() and C.is_dir()
O.mkdir(parents=True,exist_ok=True)
sha=lambda b:hashlib.sha256(b).hexdigest()
save=lambda p,d:p.write_text(json.dumps(d,sort_keys=True,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
mp=N/'actual_attempt_r1_handoff/viewplan_fusion_metadata.json';m=json.loads(mp.read_text())
sources=[]
for row in m['source_audits']:
    p=Path(row['path']);assert sha(p.read_bytes())==row['sha256'];sources.append(row)
initial=json.loads(Path(sources[0]['path']).read_text());final=json.loads(Path(sources[1]['path']).read_text())
gaps=initial['observe_return']['measured_missing_sides'];view=initial['selected']['observation_view']
assert gaps==m['initial']['observe_return']['measured_missing_sides']
assert view['observe_return_missing_side_gain']==m['initial']['selected_view']['observe_return_missing_side_gain']
planned=np.asarray(view['proposal']['T_base_camera_link']);actual=np.asarray(final['snapshot_transform']['T_base_camera_link'])
selected=next(g for g in gaps if g['angle_deg']==view['observe_return_missing_side_gain']['angle_deg'])
# The gap's exact initial support normal was not serialized. Reconstruct the
# normal to its shared jaw plane solely to build explicit hypothetical origins.
j0=np.asarray(gaps[0]['jaw_axis_base']);j1=np.asarray(gaps[1]['jaw_axis_base']);normal=np.cross(j0,j1);normal/=np.linalg.norm(normal);normal*=np.sign(normal[2])
assert all(abs(normal@np.asarray(g['jaw_axis_base']))<1e-12 for g in gaps)
fixture=dict(schema=1,kind='frozen_actual_view_gain_counterfactual_not_motion_fixture',source_audits=sources,
    aggregation_source=dict(path=str(mp),sha256=sha(mp.read_bytes())),
    original_gaps=gaps,selected_original_gain=view['observe_return_missing_side_gain'],
    planned_T_base_camera_link=planned.tolist(),planned_target_base_xyz=view['target_base_xyz'],planned_camera_target_distance_m=view['camera_target_distance_m'],
    actual_later_T_base_camera_link=actual.tolist(),actual_later_source_stamp_ns=final['snapshot_stamp_ns'],initial_request_source_stamp_ns=initial['snapshot_stamp_ns'],
    initial_surface_source_stamp_ns=initial['observe_return']['initial_surface_stamp_ns'],
    current_depth_uncertainty_m=view['observe_return_missing_side_gain']['depth_uncertainty_m'],
    allowed_camera_target_distance_m=[.18,.22],derived_gap_jaw_plane_normal_base=normal.tolist(),hypothetical_origin_range_center='planned_target_base_xyz (retained gap center is unchanged for gain evaluation)',
    unavailable_in_legacy_gaps=['absolute_independent_support_height_bounds','required_contact_patch_overlap_m'],
    limitations=['Gap heights/counts and transforms are exact recorded values. No side-band bounds or overlap requirement were invented.',
        'Actual later camera pose is a geometric proxy against the retained initial target, not synchronized endpoint error or registered new-view gain.',
        'Alternative origins are hypothetical; no IK, target visibility, collision, path, timing or physical motion feasibility has been checked.',
        'Positive projected opposite-side height is an information estimate; it is not measured common contact support.'],execution_authorized=False)

def evaluate(camera):
    rows=[]
    for gap in gaps:
        center=np.asarray(gap['target_center_base_m']);jaw=np.asarray(gap['jaw_axis_base'])
        delta=np.asarray(camera)-center;initialdelta=np.asarray(gap['initial_camera_origin_base_m'])-center
        projection=float(jaw@delta/np.linalg.norm(delta));before=float(jaw@initialdelta/np.linalg.norm(initialdelta))
        for sign in gap['missing_side_signs']:
            opposite='positive' if sign<0 else 'negative'
            reliable=gap[opposite+'_measured_points']>=gap['minimum_points_per_side']
            height=gap['object_height_m'];cap=min(height,gap[opposite+'_measured_height_m']) if reliable else 0.
            factor=max(0.,sign*projection);prior=max(0.,sign*before)
            travel=sign*float(jaw@delta)>.5*gap['whole_obb_width_m']+1e-9
            old=height*(factor-prior);new=cap*(factor-prior);noise=fixture['current_depth_uncertainty_m']
            rows.append(dict(angle_deg=gap['angle_deg'],missing_side_sign=sign,distance_to_retained_gap_center_m=float(np.linalg.norm(delta)),
                missing_side_cosine=factor,initial_missing_side_cosine=prior,old_object_height_m=height,opposite_measured_height_m=gap[opposite+'_measured_height_m'],opposite_measured_points=gap[opposite+'_measured_points'],opposite_count_sufficient=reliable,
                proposed_height_cap_m=cap,old_gain_m=old,opposite_height_capped_gain_m=new,
                beyond_missing_side_extent=travel,old_formula_pass=bool(travel and height*factor>noise+1e-9 and old>noise+1e-9),
                opposite_height_formula_pass=bool(travel and cap*factor>noise+1e-9 and new>noise+1e-9)))
    return dict(per_gap=rows,old_formula_accepts=any(r['old_formula_pass'] for r in rows),opposite_height_formula_accepts=any(r['opposite_height_formula_pass'] for r in rows),
        maximum_old_gain_m=max(r['old_gain_m'] for r in rows),maximum_opposite_height_gain_m=max(r['opposite_height_capped_gain_m'] for r in rows))
planned_eval=evaluate(planned[:3,3]);actual_eval=evaluate(actual[:3,3]);assert abs(planned_eval['maximum_old_gain_m']-fixture['selected_original_gain']['predicted_new_side_gain_m'])<1e-15
assert planned_eval['old_formula_accepts'] and not planned_eval['opposite_height_formula_accepts']
assert not actual_eval['opposite_height_formula_accepts']
alternatives=[];center=np.asarray(view['target_base_xyz']);missing_axis=-np.asarray(selected['jaw_axis_base'])
for angle in (30.,45.):
    direction=np.cos(np.deg2rad(angle))*normal+np.sin(np.deg2rad(angle))*missing_axis
    assert abs(np.linalg.norm(direction)-1.)<1e-12
    for distance in (.18,.20,.22):
        origin=center+distance*direction
        alternatives.append(dict(tilt_from_derived_gap_normal_toward_missing_side_deg=angle,distance_m=distance,camera_origin_base_m=origin.tolist(),
            geometry=evaluate(origin),hypothetical_only=True,motion_feasibility_checked=False))
fixture['hypothetical_origins']=[{k:v for k,v in r.items() if k!='geometry'} for r in alternatives]
fixture_path=O/'gen58_observation_shared_band_gain.json';save(fixture_path,fixture)
target=C/'src/alicia_grasp_modes/tests/fixtures/gen58_observation_shared_band_gain.json';target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(fixture_path.read_bytes())
report=dict(fixture_path=str(fixture_path),candidate_fixture_path=str(target),fixture_sha256=sha(fixture_path.read_bytes()),source_formula='old: object_height * delta(max(0, missing_sign * jaw.dot(unit_camera_direction))); proposed diagnostic: replace height with min(object_height, reliable opposite measured side height)',
    production_formula_changed=False,planned=planned_eval,actual_later_pose_proxy=actual_eval,alternatives=alternatives,
    selected_gap_angle_semantics='35 degrees is a jaw-direction identifier in the support plane, not camera tilt or orbit.',
    outcome='Recorded selected pose passes old scalar gain and fails opposite-band gain; finite more lateral origins can pass the information estimate at unchanged range. Their physical visibility and motion feasibility remain unknown.',
    script_sha256=sha(Path(__file__).read_bytes()),baseline_source=dict(path=str(W/'src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_visual.py'),sha256=sha((W/'src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_visual.py').read_bytes())),ROS_initialized=False,network_accessed=False,motion_requested=False)
save(O/'result.json',report);(O/'helper_source.py').write_bytes(Path(__file__).read_bytes())
print(json.dumps(dict(result=str(O/'result.json'),fixture_sha256=report['fixture_sha256'],planned_old_mm=planned_eval['maximum_old_gain_m']*1000,planned_capped_mm=planned_eval['maximum_opposite_height_gain_m']*1000,actual_capped_mm=actual_eval['maximum_opposite_height_gain_m']*1000,alternative_selected_gap_mm=[next(r['opposite_height_capped_gain_m']*1000 for r in x['geometry']['per_gap'] if r['angle_deg']==selected['angle_deg']) for x in alternatives]),indent=2))
