"""Frozen endpoint search only; no trajectory or hardware authorization."""
from pathlib import Path
import concurrent.futures
import itertools
import json
import os
import resource
import time

R=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1/contact_feedback_fix_r1/actual_gen63_baseline_validation_20261023T041149Z/high_accuracy_grasp_r1')
OUT=R/'handoff_repair_r2/actual_validation'
os.environ['OFFLINE_BOUNDARY_LOG']=str(OUT/'bounded_search_boundary.jsonl')
exec((R/'handoff_repair_r2/guard.py').read_text())
import numpy as np
from scipy.optimize import minimize
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES, SDK_QUANTUM_RAD as Q
from alicia_flexible_grasp.robot.pregrasp_compensation import pose_matrix

model=json.loads((R/'pregrasp_replay_r1/frozen_handoff.json').read_text())['parameters']['/robot_description']
fk=SerialUrdfFk(model,ARM_NAMES)
analysis=json.loads((OUT/'endpoint_analysis.json').read_text())
evidence=analysis['failure']['held_axes_prediction']
actual=np.array(evidence['virtual_start_positions_rad'])
goal=pose_matrix(evidence['fixed_goal_pose'])
free=[0,2,3,5]
bound=32*Q
tolerance=np.deg2rad(5.)


def residual(x):
    q=actual.copy();q[free]+=x;m=fk(q)
    return float(np.linalg.norm(goal[:3,3]-m[:3,3])),float(np.arccos(np.clip((np.trace(goal[:3,:3].T@m[:3,:3])-1)/2,-1,1)))


def solve(seed):
    z=minimize(lambda x:(residual(x)[0]/.01)**2,np.array(seed)*Q,method='SLSQP',
        bounds=[(-bound,bound)]*4,constraints=[dict(type='ineq',fun=lambda x:1-residual(x)[1]/tolerance)],
        options=dict(maxiter=100,ftol=1e-12))
    centre=np.rint(z.x/Q).astype(int);best=None
    for shift in itertools.product((-1,0,1),repeat=4):
        d=centre+shift
        if max(abs(d))>32:continue
        p,angle=residual(d*Q)
        if angle>tolerance:continue
        if best is None or p<best['position_mm']/1000:
            delta=np.zeros(6,dtype=int);delta[free]=d
            best=dict(delta_counts=delta.tolist(),position_mm=p*1000,orientation_deg=float(np.rad2deg(angle)))
    return dict(seed_counts=[int(x) for x in seed],solver_success=bool(z.success),evaluations=int(z.nfev),quantized_best=best)


if __name__=='__main__':
    assert Path('/media/zhuyupei/SanStand').is_mount()
    seeds=list(itertools.product((-32,32),repeat=4))+[(0,0,0,0),tuple(np.array(evidence['proposed_delta_counts'])[free])]
    started=time.monotonic()
    with concurrent.futures.ProcessPoolExecutor(max_workers=2) as pool:rows=list(pool.map(solve,seeds))
    best=min((r['quantized_best'] for r in rows if r['quantized_best']),key=lambda r:r['position_mm'])
    usage=resource.getrusage(resource.RUSAGE_CHILDREN)
    result=dict(scope='18 independent starts in unchanged 32-count box; Joint2/Joint5 held; original 5-degree gate. Numerical endpoint search, not a global infeasibility proof, collision check or hardware authorization.',
        workers=2,wall_sec=time.monotonic()-started,child_cpu_sec=usage.ru_utime+usage.ru_stime,
        child_max_rss_kib=usage.ru_maxrss,best=best,within_10mm=best['position_mm']<=10.,rows=rows,
        production_changed=False,motion_commands_sent=False,
        superseded_diagnostic='Initial inline search completed but JSON serialization of a numpy seed failed; no saved result from that invocation.')
    (OUT/'bounded_search.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='rows'},ensure_ascii=False))
