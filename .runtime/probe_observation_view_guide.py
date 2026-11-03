"""Existing bounded held-axis guide with observation feasibility as its merit."""
from pathlib import Path
from types import SimpleNamespace as NS
import json,os,math,time
W=Path(__file__).resolve().parents[1]
R=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1/contact_feedback_fix_r1/actual_gen63_baseline_validation_20261023T041149Z/high_accuracy_grasp_r1/arrival_margin_repair_r1')
D=R/'supplemental_view_r8/feedback_diagnosis';os.environ['OFFLINE_BOUNDARY_LOG']=str(D/'view_guide_boundary.jsonl');exec((R/'guard.py').read_bytes())
import numpy as np
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES,SDK_QUANTUM_RAD as Q
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_flexible_grasp.robot.held_axes_prediction import predict_held_axes_endpoint,HeldAxesPredictionError
from alicia_flexible_grasp.robot.pregrasp_compensation import remaining_budget_guide,pose_matrix,PregraspCompensation
from alicia_flexible_grasp.robot.endpoint_correction import sdk_counts
from alicia_flexible_grasp.robot.observation_endpoint_compensation import observation_endpoint_view
from alicia_flexible_grasp.grasp.observation_working_volume import observation_obb_corners
x=json.loads((D/'held_axes_probe.json').read_text());fk=SerialUrdfFk((W/'deployment/2026-10-08/runtime/candidate_models/expanded_metric_20260929/alicia_expanded_metric_candidate.urdf').read_text(),ARM_NAMES)
sdk,actual=np.array(x['initial_sdk']),np.array(x['initial_measured']);goal=pose_matrix(x['goal']);cfg=x['configuration'];scene=NS(**{k:np.array(v) if k in ('center','rotation','size') else v for k,v in x['scene'].items()});camera=cfg['camera'];mount=np.array(cfg['tool_from_camera']);corners=observation_obb_corners(scene.center,scene.rotation,scene.size)
with np.load(R/'supplemental_view_r7/collision_diagnosis/view_search_source.npz') as a:valid=a['rgb_valid'].copy()
limits=cfg['limits'];tol=(.010,math.radians(5));n=[0]
def residual(q):
 t=fk(q);return float(np.linalg.norm(goal[:3,3]-t[:3,3])),math.acos(float(np.clip((np.trace(goal[:3,:3].T@t[:3,:3])-1)/2,-1,1)))
def merit(q):
 n[0]+=1;t=fk(q)@mount;p=(corners-t[:3,3])@t[:3,:3];depth=p[:,0]
 if np.any(depth<=0):raise ValueError('proposal outside camera half space')
 uv=np.array([camera['cx'],camera['cy']])-np.array([camera['fx'],camera['fy']])*p[:,1:]/depth[:,None]
 edge=float(np.min(np.column_stack((uv[:,0],camera['width']-1-uv[:,0],uv[:,1],camera['height']-1-uv[:,1]))));position,angle=residual(q)
 return max(position/tol[0],angle/tol[1],1+(limits['roi_margin_px']-edge)/limits['roi_margin_px'])
counts=sdk_counts(sdk)
def guide(q):return remaining_budget_guide(fk,residual,q,counts,counts,maximum_steps=PregraspCompensation.MAX_STEPS,steps=0,maximum_step_counts=PregraspCompensation.MAX_STEP_COUNTS,maximum_total_counts=PregraspCompensation.MAX_TOTAL_COUNTS,held_axes=PregraspCompensation.HELD_AXES,response_gains=np.ones(6),tolerances=tol,cost=merit)
started=time.monotonic();out=dict(scope='conditional offline endpoint only; no path or motion proof')
try:
 target,evidence=predict_held_axes_endpoint(fk,sdk,actual,x['goal'],position_tolerance_m=tol[0],orientation_tolerance_rad=tol[1],guide_provider=guide)
 out.update(held_endpoint=evidence,target_counts=target.tolist())
 try:out['predicted_view']=observation_endpoint_view(scene,cfg,fk(evidence['predicted_positions_rad']),valid)
 except Exception as exc:out['view_error']=str(exc)
except HeldAxesPredictionError as exc:out.update(error=str(exc),held_endpoint=exc.evidence)
out.update(elapsed_sec=time.monotonic()-started,merit_calls=n[0]);(D/'view_guide_probe.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n');print(json.dumps(out,ensure_ascii=False))
