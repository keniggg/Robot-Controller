"""Finite frozen endpoint feasibility diagnosis; never a motion command."""
from pathlib import Path
import os,json,math,time
W=Path(__file__).resolve().parents[1]
R=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1/contact_feedback_fix_r1/actual_gen63_baseline_validation_20261023T041149Z/high_accuracy_grasp_r1/arrival_margin_repair_r1')
D=R/'supplemental_view_r9/feedback_diagnosis';os.environ['OFFLINE_BOUNDARY_LOG']=str(D/'feasibility_boundary.jsonl');exec((R/'guard.py').read_bytes())
import numpy as np
from scipy.optimize import minimize
from itertools import product
from types import SimpleNamespace as NS
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES,SDK_QUANTUM_RAD as Q
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_flexible_grasp.robot.pregrasp_compensation import pose_matrix
from alicia_flexible_grasp.robot.observation_endpoint_compensation import observation_endpoint_view
from alicia_flexible_grasp.grasp.observation_working_volume import observation_obb_corners
out=[]
for attempt,file in [('supplemental_view_r9','endpoint_replay.json'),('supplemental_view_r8','held_axes_probe.json')]:
 x=json.loads((R/attempt/'feedback_diagnosis'/file).read_text());fk=SerialUrdfFk((W/'deployment/2026-10-08/runtime/candidate_models/expanded_metric_20260929/alicia_expanded_metric_candidate.urdf').read_text(),ARM_NAMES)
 measured=np.array(x['initial_measured']);sdk=np.array(x['initial_sdk']);goal=pose_matrix(x['goal']);cfg=x['configuration'];cam=cfg['camera'];mount=np.array(cfg['tool_from_camera']);scene=NS(**{k:np.array(v) if k in ('center','size','rotation') else v for k,v in x['scene'].items()});corners=observation_obb_corners(scene.center,scene.rotation,scene.size)
 with np.load(R/'supplemental_view_r7/collision_diagnosis/view_search_source.npz') as z:valid=z['rgb_valid'].copy()
 n=[0];free=[0,2,3,5];tol=(.010,math.radians(5))
 def components(q):
  t=fk(q);p=float(np.linalg.norm(goal[:3,3]-t[:3,3]));a=math.acos(float(np.clip((np.trace(goal[:3,:3].T@t[:3,:3])-1)/2,-1,1)))
  camera=t@mount;pts=(corners-camera[:3,3])@camera[:3,:3];uv=np.array([cam['cx'],cam['cy']])-np.array([cam['fx'],cam['fy']])*pts[:,1:]/pts[:,:1]
  edge=float(np.min(np.column_stack((uv[:,0],cam['width']-1-uv[:,0],uv[:,1],cam['height']-1-uv[:,1]))))
  return np.array([p/tol[0],a/tol[1],1+(cfg['limits']['roi_margin_px']-edge)/cfg['limits']['roi_margin_px']]),edge
 def delta(v):
  d=np.zeros(6);d[free]=v;return d
 def merit(v):
  n[0]+=1
  try:
   d=delta(v);fk(sdk+d);ratios,_=components(measured+d);return max(ratios)
  except ValueError:return 1.e6
 seed=np.array(x.get('proposal_before_view_gate',{}).get('proposed_delta_counts',[0,0,10,4,0,22]))*Q
 started=time.monotonic();result=minimize(merit,seed[free],method='SLSQP',bounds=[(-32*Q,32*Q)]*4,options=dict(maxiter=64,ftol=1.e-10))
 centre=np.rint(delta(result.x)/Q).astype(int);best=None;valid_count=0
 for offsets in product((-1,0,1),repeat=4):
  dc=centre.copy();dc[free]+=offsets
  if np.max(abs(dc))>32:continue
  try:
   fk(sdk+dc*Q);ratios,edge=components(measured+dc*Q)
   if np.max(ratios)>1.:continue
   view=observation_endpoint_view(scene,cfg,fk(measured+dc*Q),valid)
   valid_count+=1;key=(float(np.max(ratios)),float(np.sum(ratios)))
   if best is None or key<best['key']:best=dict(key=key,delta_counts=dc.tolist(),ratios=ratios.tolist(),edge_px=edge,view=view)
  except ValueError:pass
 row=dict(attempt=attempt,elapsed_sec=time.monotonic()-started,evaluations=n[0],solver_success=bool(result.success),solver_message=str(result.message),solver_merit=float(result.fun),quantized_feasible_count=valid_count,best=best,initial=components(measured)[0].tolist(),original_guide=components(measured+seed)[0].tolist(),scope='endpoint diagnosis only; no continuous path or hardware response proof')
 out.append(row);print(json.dumps(row,ensure_ascii=False),flush=True)
(D/'bounded_feasibility_probe.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n')
