from pathlib import Path
import json,time
from copy import deepcopy
import numpy as np
from scipy.optimize import minimize
from alicia_grasp_modes.observation_arrival_projection import capture_arrival_projection,validate_arrival_projection
from alicia_grasp_modes import reachable_observation_views as mod
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry
OUT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z/gen20_analysis/arrival_projection');d=json.loads((OUT/'view_source.json').read_text());source=d['source'];source.update(dict(np.load(str(OUT/'view_source.npz'))));source['arrival_projection']=capture_arrival_projection(d['source_status'],d['selection'],source['binding']['snapshot_stamp_ns'],source['camera'])
WORK=Path(__file__).resolve().parents[1];fk=SerialUrdfFk((WORK/'deployment/2026-10-08/runtime/candidate_models/expanded_metric_20260929/alicia_expanded_metric_candidate.urdf').read_text(),['Joint%d'%i for i in range(1,7)]);assert fk.model_sha256==d['model_sha256'];mount=np.asarray(source['tool_from_camera']);prism=source['arrival_projection']['prism_base'];camera=source['camera'];k=np.asarray([camera[x] for x in ('fx','fy','cx','cy')]);q0=np.asarray(d['current_joint_positions']);eps=np.asarray(d['joint_error_bounds_rad'])
args=dict(source=source,current_joint_positions=q0,fk=fk,gripper=GripperGeometry(.05,.002,np.array([.0434,.0286,.06]),np.array([.1175,.155,.0774]),.003),opening_width_m=.05,gaps=d['gaps'],depth_uncertainty_m=d['depth_uncertainty_m'],side_depth_uncertainty_m=d['side_depth_uncertainty_m'],joint_error_bounds_rad=eps,max_seconds=2.,max_directions=2)
diag=json.loads((OUT/'planned_actual_projection.json').read_text());Terror=np.linalg.inv(np.asarray(diag['planned_transform']))@np.asarray(diag['frames'][0]['T_base_camera']);rows=[]
for weight in (0.,1.,3.):
 def solver(fun,*a,**kw):
  def cost(q):
   cam=fk(q)@mount;p=(prism-cam[:3,3])@cam[:3,:3];uv=np.column_stack((k[2]-k[0]*p[:,1]/p[:,0],k[3]-k[1]*p[:,2]/p[:,0]));mid=(uv.max(0)+uv.min(0))/2.;target=np.array([(camera['width']-1)/2.,(camera['height']-1)/2.]);return fun(q)+weight*np.sum(((mid-target)/k[:2])**2)
  return minimize(cost,*a,**kw)
 mod._backend=lambda:solver
 candidates,report=mod.generate_reachable_observation_views(**args);record=dict(weight=weight,report=report,candidates=[])
 for c in candidates:
  T=np.asarray(c['T_base_camera_link']);bounds=c['endpoint_checks']['arrival_projection_endpoint'];actual=T@Terror
  try:pred=validate_arrival_projection(source['arrival_projection'],actual,k,camera['width'],camera['height'])
  except ValueError as e:pred={'ok':False,'reason':str(e)}
  record['candidates'].append(dict(q=c['q'],delta=c['joint_max_delta_rad'],nominal=bounds,reapplied_recorded_rigid_error=pred))
 rows.append(record);print('weight',weight,'status',report['status'],'seconds',report['elapsed_sec'],'n',len(candidates),[(r['nominal']['minimum_image_edge_distance_px'],r['reapplied_recorded_rigid_error']['ok']) for r in record['candidates']])
(OUT/'centered_objective_probe.json').write_text(json.dumps(rows,indent=2)+'\n')
