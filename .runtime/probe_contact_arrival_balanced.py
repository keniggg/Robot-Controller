"""Offline finite endpoint experiment; no ROS or physical path authorization."""
from pathlib import Path
import json, os, math, time

W = Path(__file__).resolve().parents[1]
R = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1/contact_feedback_fix_r1/actual_gen63_baseline_validation_20261023T041149Z/high_accuracy_grasp_r1/arrival_margin_repair_r1')
os.environ['OFFLINE_BOUNDARY_LOG'] = str(R/'probe_boundary.jsonl')
exec((R/'guard.py').read_text())
import numpy as np
from scipy.optimize import minimize
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES, SDK_QUANTUM_RAD as Q, SerialUrdfFk
from alicia_flexible_grasp.robot.pregrasp_compensation import pose_matrix
from alicia_flexible_grasp.robot.held_axes_prediction import predict_held_axes_endpoint, HeldAxesPredictionError
from alicia_flexible_grasp.robot.endpoint_correction import sdk_counts

d=json.loads((R/'frozen_arrival.json').read_text())
fk=SerialUrdfFk((W/'deployment/2026-10-08/runtime/candidate_models/expanded_metric_20260929/alicia_expanded_metric_candidate.urdf').read_text(),ARM_NAMES)
offset=np.array(d['sample']['sdk_positions_rad'])-d['sample']['accepted_positions_rad']
command=(np.array(d['command_endpoint_counts'])-2048)*Q
pg,ag=map(pose_matrix,d['poses'][:2]); free=[0,2,3,5]
def error(q,g):
    t=fk(q);return np.linalg.norm(t[:3,3]-g[:3,3]),math.acos(float(np.clip((np.trace(t[:3,:3].T@g[:3,:3])-1)/2,-1,1)))
def after(x):
    q=x[:6].copy();q[free]+=x[6:];return q
def obj(x):
    p,a=error(x[:6],pg);return (p/.01)**2 + (a/math.radians(5))**2
def con(x):
    p,a=error(after(x),ag);_,pa=error(x[:6],pg)
    return [1-p/.0055,1-a/math.radians(4.5),1-pa/math.radians(4.5)]
start=time.monotonic();rows=[]
for name,delta in [('zero',[0]*4),('lower',[-32]*4),('upper',[32]*4)]:
    z=minimize(obj,np.r_[command-offset,np.array(delta)*Q],method='SLSQP',
        bounds=list(zip(command-offset-.25,command-offset+.25))+[(-32*Q,32*Q)]*4,
        constraints=[dict(type='ineq',fun=con)],options=dict(maxiter=150,ftol=1e-12))
    c=(sdk_counts(z.x[:6]+offset)-2048)*Q
    row=dict(seed=name,solver_success=bool(z.success),message=str(z.message),pre_mm=error(c-offset,pg)[0]*1000,command_counts=sdk_counts(c).tolist(),prediction=[])
    for label,off in [('prior',offset),('arrival',np.array(d['arrival_sdk_minus_actual_counts'])*Q)]:
        try:
            q,p=predict_held_axes_endpoint(fk,c,c-off,d['poses'][0],position_tolerance_m=.01,orientation_tolerance_rad=math.radians(5),allow_already_converged=True)
            _,a=predict_held_axes_endpoint(fk,p['predicted_command_positions_rad'],p['predicted_positions_rad'],d['poses'][1],position_tolerance_m=.006,orientation_tolerance_rad=math.radians(5),allow_already_converged=True)
            row['prediction'].append(dict(label=label,passed=True,pre=p,approach=a))
        except HeldAxesPredictionError as e:row['prediction'].append(dict(label=label,passed=False,error=str(e),evidence=e.evidence))
    rows.append(row)
(R/'margin_probe_balanced.json').write_text(json.dumps(dict(rows=rows,wall_sec=time.monotonic()-start),indent=2))
for r in rows:print(json.dumps({k:( [dict(label=i['label'],passed=i['passed'],error=i.get('error')) for i in v] if k=='prediction' else v) for k,v in r.items()}))
