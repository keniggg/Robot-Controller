"""Frozen goal-error visibility enclosure and bounded view-search replay."""
from pathlib import Path
import os,json,time,math,hashlib
W=Path(__file__).resolve().parents[1]
R=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1/contact_feedback_fix_r1/actual_gen63_baseline_validation_20261023T041149Z/high_accuracy_grasp_r1/arrival_margin_repair_r1')
D=R/'view_arrival_reserve_r1/probe_r2';D.mkdir()
os.environ['OFFLINE_BOUNDARY_LOG']=str(D/'boundary.jsonl');exec((R/'guard.py').read_bytes())
import numpy as np
from alicia_flexible_grasp.robot.observation_arrival_reserve import PreparedObservationArrivalReserve,observation_goal_error_bounds
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_flexible_grasp.grasp.observation_working_volume import observation_obb_corners
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES,SDK_QUANTUM_RAD as Q
from alicia_grasp_modes.reachable_observation_views import generate_reachable_observation_views
from test_reachable_observation_seed_replay import load_recorded_scene
out={};errors=observation_goal_error_bounds({n:dict(goal=.035) for n in ARM_NAMES},ARM_NAMES)
fk=SerialUrdfFk((W/'deployment/2026-10-08/runtime/candidate_models/expanded_metric_20260929/alicia_expanded_metric_candidate.urdf').read_text(),ARM_NAMES)
for a,f in [('supplemental_view_r9','endpoint_replay.json'),('supplemental_view_r8','held_axes_probe.json')]:
 x=json.loads((R/a/'feedback_diagnosis'/f).read_text());s=x['scene'];cfg=x['configuration'];points=observation_obb_corners(np.array(s['center']),np.array(s['rotation']),np.array(s['size']))
 bound=PreparedObservationArrivalReserve(fk,errors,cfg['tool_from_camera'],points,cfg['camera'],cfg['limits'])
 summary=json.loads((R/a/'actual_validation/task_resources_and_result.json').read_text());q=summary['goals'][0]['endpoint'];audit=bound.audit(q)
 out[a]=audit;print(json.dumps(dict(attempt=a,audit=audit)),flush=True)
for stem in ['supplement_view_joint_reserve_20261023','supplement_view_feasible_gain_20261023','supplement_view_seed_20261023']:
 args=load_recorded_scene(stem);args['endpoint_error_bounds_rad']=errors;args['max_seconds']=2.
 started=time.monotonic();rows,report=generate_reachable_observation_views(**args)
 out[stem]=dict(elapsed_sec=time.monotonic()-started,candidates=rows,report=report)
 print(json.dumps(dict(stem=stem,elapsed_sec=out[stem]['elapsed_sec'],status=report['status'],candidate_count=len(rows),nominal_edges=[r['endpoint_checks']['arrival_error_box_visibility']['nominal_minimum_image_edge_distance_px'] for r in rows])),flush=True)
sources=['src/alicia_grasp_modes/src/alicia_grasp_modes/reachable_observation_views.py','src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py','src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/observation_arrival_reserve.py']
result=dict(passed=all(v['candidates'] and all(r['endpoint_checks']['arrival_error_box_visibility']['ok'] for r in v['candidates']) for k,v in out.items() if k.startswith('supplement_view_')),cases=out,source_sha256={p:hashlib.sha256((W/p).read_bytes()).hexdigest() for p in sources})
(D/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,default=lambda v:v.tolist() if hasattr(v,'tolist') else str(v))+'\n')
