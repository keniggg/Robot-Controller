"""Offline current-record bounded endpoint diagnostic. No ROS I/O."""
import sys, json, math, time, hashlib
from pathlib import Path
from itertools import product
import numpy as np
from scipy.optimize import minimize
WORK=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(WORK/'src/alicia_flexible_grasp_supervisor/src'))
from alicia_flexible_grasp.robot.stationary_following import SerialUrdfFk, ARM_NAMES, SDK_QUANTUM_RAD as Q
from alicia_flexible_grasp.robot.pregrasp_compensation import pose_matrix
EXT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
assert Path('/media/zhuyupei/SanStand').is_mount()
NEW=EXT/'ros_log/full_latest_restart_20261010T002347Z'
OUT=NEW/'gen20_analysis'; OUT.mkdir(exist_ok=True)
records=[]
for line in (NEW/'gen19_direct_slot_compute_deployment/consumers.log').read_text().splitlines():
    if 'remote 6D rejected contact evidence:' in line:
        try:r=json.JSONDecoder().raw_decode(line[line.index('{'):])[0]
        except ValueError:continue
        h=r.get('precheck',{}).get('held_approach_prediction',{})
        if h.get('predicted_approach',{}).get('predicted_position_error_m') is not None:records.append(r)
records.sort(key=lambda r:r['precheck']['held_approach_prediction']['predicted_approach']['predicted_position_error_m'])
model=WORK/'deployment/2026-10-08/runtime/candidate_models/expanded_metric_20260929/alicia_expanded_metric_candidate.urdf'
fk=SerialUrdfFk(model.read_text(),ARM_NAMES)
free=np.array([0,2,3,5]); report=[]
for r in records[:3]:
    h=r['precheck']['held_approach_prediction']; e=h['predicted_approach']
    assert fk.model_sha256==r['precheck']['model_sha256']
    q=np.asarray(e['virtual_start_positions_rad']); counts=np.asarray(e['initial_sdk_counts']); goal=pose_matrix(e['fixed_goal_pose'])
    def residual(j):
        t=fk(j)
        return float(np.linalg.norm(t[:3,3]-goal[:3,3])), math.acos(float(np.clip((np.trace(t[:3,:3].T@goal[:3,:3])-1)/2,-1,1)))
    def state(x):
        j=q.copy();j[free]+=np.asarray(x)*Q;return j
    continuous=[]; lattice=[]; seen=set()
    for seed in [np.array(e['proposed_delta_counts'])[free],np.zeros(4)]+list(np.random.RandomState(20).uniform(-32,32,(4,4))):
        result=minimize(lambda x:residual(state(x))[0]**2,seed,method='SLSQP',bounds=[(-32,32)]*4,
            constraints=[dict(type='ineq',fun=lambda x:math.radians(5)-residual(state(x))[1])],options={'maxiter':100,'ftol':1e-14})
        continuous.append(dict(x=result.x.tolist(),error=list(residual(state(result.x))),success=bool(result.success),iterations=int(result.nit)))
        for point in product(*[sorted(set(max(-32,min(32,v)) for v in (math.floor(x),math.ceil(x)))) for x in result.x]):
            if point in seen:continue
            seen.add(point);delta=np.zeros(6,dtype=int);delta[free]=point
            command=(counts+delta-2048)*Q
            try:fk(command);error=residual(state(point))
            except ValueError:continue
            lattice.append(dict(delta=delta.tolist(),error=list(error),feasible=bool(error[0]<=.006 and error[1]<=math.radians(5))))
    row=dict(plan_id=r['plan_id'],record=r,continuous=continuous,best=min(lattice,key=lambda v:v['error'][0]),feasible=[x for x in lattice if x['feasible']])
    report.append(row)
    print(r['plan_id'],json.dumps(row['best']),flush=True)
(OUT/'held_same_start_search.json').write_text(json.dumps(dict(scope='offline_same_start_bounded_search_not_global_proof',model_sha256=fk.model_sha256,records=report),indent=2)+'\n')
