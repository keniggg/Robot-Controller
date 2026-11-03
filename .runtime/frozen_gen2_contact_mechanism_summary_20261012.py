from pathlib import Path
import json,hashlib,math
import numpy as np
from tf.transformations import quaternion_matrix
BASE=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_frozen_no_skin_20261012T021050Z/gen2_analysis')
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
rows=[]
for name,event in (('0ec46c52c6eaa86ebae4d872',12),('ffad12d49afa8f64c04b9a4d',6)):
 p=BASE/'mujoco_contact_trace'/(name+'.json');d=json.loads(p.read_text());states=d['steps'];first=states[0];R0=quaternion_matrix(np.roll(first['object_quaternion_wxyz'],-1))[:3,:3];events=[]
 for n in (event,len(states)-1):
  s=states[n];R=quaternion_matrix(np.roll(s['object_quaternion_wxyz'],-1))[:3,:3];delta=np.array(s['object_position_base_m'])-first['object_position_base_m'];events.append(dict(index=n,time_sec=s['time'],object_translation_m=delta.tolist(),object_translation_norm_m=float(np.linalg.norm(delta)),object_rotation_deg=math.degrees(math.acos(np.clip((np.trace(R0.T@R)-1)/2,-1,1))),classification=s['classification'],contacts=[dict(c,abs_normal_object_top_cos=float(abs(np.array(c['normal_base'])@R[:,2])))for c in s['gripper']['finger_object_contacts']]))
 a={k:v for k,v in d['recorded_response'].items()if k not in ('ok','used_joint_state_source','candidate_source','candidate_source_lineage')};b={k:v for k,v in d['replayed_response'].items()if k not in ('ok','used_joint_state_source','candidate_source','candidate_source_lineage')}
 rows.append(dict(plan_id=name,trace_path=str(p),trace_sha256=sha(p),full_physics_response_exact_equal=a==b,excluded_response_fields='HTTP transport/provenance additions only; every retained IK, dynamics and failure field exact',grasp_ik=next(r for r in d['recorded_response']['ik_results']if r['name']=='grasp'),events=events))
summaryfiles=[BASE/'height_intersection_scan/summary.json',BASE/'height_intersection_consistent_close/summary.json',BASE/'height_intersection_half_window/summary.json'];tables=[]
for f in summaryfiles:
 r=json.loads(f.read_text());tables.append(dict(path=str(f),sha256=sha(f),cases=[{k:v for k,v in x.items()if k!='contact_curve'}for x in r['rows']]))
best=BASE/'height_intersection_half_window/0ec46c52c6eaa86ebae4d872_1p7208355344308164mm.json';b=json.loads(best.read_text())
report=dict(scope='GEN2_frozen_two_candidates_offline_cause_and_existing_height_window',motion_authorized=False,production_changed=False,online_service_changed=False,ros_called=False,hardware_grasp_verified=False,original_replay=rows,scan_tables=tables,best_diagnostic_candidate=dict(path=str(best),sha256=sha(best),original_plan_id=b['source_plan_id'],new_committable_plan_id=None,warning='Original ID retained solely as offline provenance; altered poses/close must receive a newly rebuilt valid bound plan ID before any live check.',height_above_original_m=b['height_offset_from_original_m'],height_policy='0.5 * (existing_allowed_upper - unchanged_empirical_translation_limit)',close_target_m=b['simulated_payload']['gripper']['close_target_inner_gap_m'],thresholds_relaxed=False,actual_original_no_lift_client_gate=b['current_no_lift_gate'],preload_contact=b['response']['lift_evidence']['settled_gripper_state'],full_dynamic_lift_success=b['response']['lift_success']),conclusions=[
'Original failed geometry has no issue of mistaking intended closed contact: collision occurs on still-open predicted actuator curve. Current existing height search can clear it, so additional camera observation is unnecessary for these candidates.',
'First original raised proposal forms an opposed pair at 18 ms, loses it at 26 ms before any lift: object has descended 1.505 mm and rotated 0.191 degrees, right normal becomes corner/top dominated. Final right normal aligns with object top; both fingers may touch without lateral opposing hold.',
'Second raised proposal never develops the required opposed pair, finishing with both normals aligned with object top. This is a genuine dynamic closure rejection, not just missing lift evidence.',
'Original grasp IK residuals are only 0.189 mm and 0.045 degrees. Offline exact replay supports a shallow contact/height and object-settling mechanism rather than large IK reach error; it does not identify actual hardware calibration or compliance.',
'Static finite-pad projection area is nonzero, but does not guarantee which normals and loads the dynamic collision/contact solver will create at narrow upper-edge contact.',
'Changing only the close target by +3 mm does not repair the original +2.581/+2.779 mm height candidates. Consistent close together with the first direction at a smaller permitted height gives a contact-path/preload intersection.',
'At exactly 50% of the original feasible upper window (+1.720835534 mm), corrected close 32.728938878 mm, nominal and predicted reconstructed contact splines pass; both loaded opposed contacts survive preload; unchanged current require_lift_success=false validator accepts.',
'Lift fails in that successful closure case. Full lifting is still unverified and must remain a separate later stage; raw simulation_ok=false is retained.'],limitations=[
'All new height cases are offline counterfactuals. No live observation, movement, closure, grasp, or full hardware success occurred.',
'Only approach-to-grasp CAD curves were independently reconstructed with local IK and artificial timing. Full current MoveIt pregrasp/held-approach policies, real retiming/whole-arm collisions, real new joint starts and following bias remain untested.',
'Changing height shifts all four requested simulated Cartesian poses; offline contact geometry uses the recorded accepted proposal conditional approach joint start, so it cannot authorize actual execution of changed stages.',
'An offline modification reuses source plan_id as a provenance label only, which must never be interpreted as a valid immutable execution plan for changed payload.',
'First-direction success does not imply second-direction success or repeatability under new measurement noise.',
'Earlier static_contact_patch_summary noted absence of mujoco in the default interpreter; subsequent replay explicitly found and used the pinned runtime .runtime/mujoco-3.2.3 with unchanged actual loaded model/source hashes.'])
out=BASE/'contact_mechanism_and_window_intersection.json';out.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n');print(json.dumps(dict(path=str(out),sha256=sha(out),exact_physics_replay=[r['full_physics_response_exact_equal']for r in rows])))
