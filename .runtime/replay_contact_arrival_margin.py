"""Replay the executed selected endpoint using only its pre-motion bias."""
from pathlib import Path
import hashlib, json, math, os, time
W=Path(__file__).resolve().parents[1]
R=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1/contact_feedback_fix_r1/actual_gen63_baseline_validation_20261023T041149Z/high_accuracy_grasp_r1/arrival_margin_repair_r1')
os.environ['OFFLINE_BOUNDARY_LOG']=str(R/'replay_boundary.jsonl')
exec((R/'guard.py').read_text())
import numpy as np
from alicia_flexible_grasp.robot.held_pregrasp_margin import refine_held_pregrasp_margin
from alicia_flexible_grasp.robot.held_axes_prediction import predict_held_axes_endpoint, HeldAxesPredictionError
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES, SerialUrdfFk, SDK_QUANTUM_RAD as Q
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
d=json.loads((R/'frozen_arrival.json').read_text())
fk=SerialUrdfFk((W/'deployment/2026-10-08/runtime/candidate_models/expanded_metric_20260929/alicia_expanded_metric_candidate.urdf').read_text(),ARM_NAMES)
old=(np.array(d['command_endpoint_counts'])-2048)*Q
def predict(c,offset):
    rows=[]
    try:
        actual=c-offset
        for goal,tol in zip(d['poses'][:2],[.01,.006]):
            _,proof=predict_held_axes_endpoint(fk,c,actual,goal,position_tolerance_m=tol,
                orientation_tolerance_rad=math.radians(5),allow_already_converged=True)
            rows.append(proof);c=np.asarray(proof['predicted_command_positions_rad']);actual=np.asarray(proof['predicted_positions_rad'])
        return dict(error=None,proofs=rows)
    except HeldAxesPredictionError as exc:return dict(error=str(exc),proofs=rows,failure=exc.evidence)
started=time.monotonic()
command,margin=refine_held_pregrasp_margin(fk,d['sample'],*d['poses'][:2],old,
    position_tolerance_m=.01,orientation_tolerance_rad=math.radians(5))
offset=np.array(d['sample']['sdk_positions_rad'])-d['sample']['accepted_positions_rad']
arrival=np.array(d['arrival_sdk_minus_actual_counts'])*Q
results=[predict(command,offset),predict(command,arrival)]
baseline=predict(old,arrival)
assert baseline['error']=='PREGRASP_HELD_AXES_GUIDE_DOES_NOT_REACH_FIXED_GOAL'
assert margin['refined'] and all(row['error'] is None for row in results)
sources=[W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot'/n for n in
    ['contact_pregrasp_gateway.py','held_pregrasp_margin.py','held_pregrasp_selection.py','held_axes_prediction.py','pregrasp_compensation.py']]
out=dict(scope='conditional endpoint replay; observed pregoal sample is not a claimed private planner snapshot; no complete trajectory or real hardware success',
    source_sha256={str(p):sha(p) for p in sources},input_sha256=sha(R/'frozen_arrival.json'),baseline=baseline,
    results=results,margin=margin,wall_sec=time.monotonic()-started,motion_commands_sent=False)
(R/'frozen_replay.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(baseline=baseline['error'],refined=margin['refined'],elapsed=out['wall_sec'],
    pregoal_pregrasp_mm=results[0]['proofs'][0]['position_error_m']*1000,
    arrival_corrected_mm=results[1]['proofs'][0]['predicted_position_error_m']*1000)))
