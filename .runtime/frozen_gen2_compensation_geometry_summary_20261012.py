from pathlib import Path
import json,hashlib
BASE=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_frozen_no_skin_20261012T021050Z')
lines=BASE.joinpath('core.log').read_text().splitlines()
rows=[]
for n in (2936,3001,3089,3154,3244):
 p=BASE/'gen2_analysis/compensation_collision'/str(n)/'offline_reconstruction.json'
 d=json.loads(p.read_text()); raw=json.loads(lines[n-1][lines[n-1].index('{'):])
 row=dict(source_log_line=n,plan_id=d['plan_id'],recorded_failure=d['recorded_first_failure'],offset_counts=d['sdk_minus_encoder_counts'],goal=d['registered_goal_pose'],command_displacement=d['command_relative_to_fixed_goal_translation'],terminal={k:v['terminal']for k,v in d['summary'].items()},reconstructed_continuous_spline=d['reconstructed_complete_spline'],path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest())
 if 'search' in raw:
  p0=raw['search']['proposals'][0]; pc=p0['contact_sequence_precheck']
  row.update(recorded_retry_accepted=p0['accepted'],recorded_strict_success=pc['success'],recorded_result=p0['result'],normal_translation_m=p0['placement']['normal_translation_m'],contact_overlap_m=p0['final_contact_height_margin']['nominal']['actual_overlap_m'],recorded_complete_curve_prediction=pc['contact_prediction'])
 rows.append(row)
report=dict(schema=1,scope='offline_recorded_collision_and_existing_height_retry_diagnosis',motion_authorized=False,production_changed=False,ros_called=False,hardware_motion=False,source_log=str(BASE/'core.log'),recorded_original_failed_curve_available=False,rows=rows,conclusions=[
'Contact geometry and original nominal paths passed; current measured SDK-minus-encoder compensation consumes the open-finger target clearance. The collision is before closure with 49.8 mm opening, not intended closed-finger contact.',
'Constant measured following bias is [0,+14,+12,0,-1,-3] SDK counts, a virtual conditional prediction not a future encoder guarantee.',
'The first direction predicted command shifts -8.356 mm along jaw axis; reconstructed nominal left/right target SAT gaps 3.275 mm become left-finger 1.040 mm penetration. Wrist reversal changes colliding finger but not this world displacement.',
'Existing bounded contact-height retry already solves full path geometry in actual recorded runtime: +2.581253 mm (plan 0ec46c52c6eaa86ebae4d872) and +2.778816 mm (plan ffad12d49afa8f64c04b9a4d), both accepted and full nominal plus predicted-command curves pass.',
'Independent local IK reconstruction also passes each raised candidate and reproduces collision label 13-end left/right for the original candidates.',
'After those successes, MuJoCo closed-contact/preload rejection is the remaining blocker; no evidence supports further observation for these candidates.'],limitations=[
'Original failing MoveIt waypoint arrays/timing and exact first-collision joint state were not saved. Reconstructed scipy local IK paths start from exact logged conditional approach joints and exact logged fixed goal using identical hashed URDF, but are not bitwise replays of original MoveIt.',
'Continuous proofs on reconstructed paths use artificial one-second zero-derivative knots, not original timing. Endpoint/SAT geometry and same failure segment labels are evidence of mechanism, not execution authorization.',
'SAT best separating gap is an oriented-box separation/penetration diagnostic, not measured clearance nor universal Euclidean margin.',
'Raised candidates are recorded complete-path successes, not physical closure or holding successes. Measurement contact overlap barely exceeds 2 mm; actual MuJoCo failure must remain checked.',
'No fresh live measurement, future tracking proof, environmental whole-arm collision analysis, contact dynamics replay, or physical grasp occurred in this offline analysis.'])
out=BASE/'gen2_analysis/compensation_geometry_summary.json';out.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
print(json.dumps(dict(path=str(out),sha256=hashlib.sha256(out.read_bytes()).hexdigest(),rows=len(rows))))
