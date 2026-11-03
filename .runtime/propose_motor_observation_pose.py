#!/usr/bin/env python3
"""Offline image-centering proposal only. No motion permit or ROS publisher."""
import json,os,sys,importlib.util,hashlib
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
HERE=Path(__file__).resolve().parent;ROOT=HERE.parent
sys.path.insert(0,str(ROOT/'src/alicia_flexible_grasp_supervisor/src'))
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry,_stage_boxes,_box_corners
sp=importlib.util.spec_from_file_location('camera',ROOT/'src/real-arm/alicia_d_calibration/scripts/charuco_tracker.py');CAM=importlib.util.module_from_spec(sp);sp.loader.exec_module(CAM)
sp=importlib.util.spec_from_file_location('geom',HERE/'analyze_fixed_handeye_translation.py');G=importlib.util.module_from_spec(sp);sp.loader.exec_module(G)
M=Path('/media/zhuyupei/SanStand');A=M/'实验记录/Robot-Controller/ros_log';R=A/'grasp_repair_resume_20260928T120606Z'
if not os.path.ismount(M):raise RuntimeError('SanStand absent')
s=A/'bounded_joint23_counterrotation_20260928T131913Z_197737'
g=json.loads((s/'positive_post_hold_geometry.json').read_text())
r=json.loads((R/'motor_adjacent_table_normal_oriented'/s.name/'report_with_provenance.json').read_text())
point=np.median([row['motor_board_surface_median_m'] for row in r['frames'] if row['phase']=='positive'],axis=0)
fk=SerialUrdfFk((s/'robot_description.urdf').read_text(),['Joint%d'%i for i in range(1,7)])
model=CAM.RuntimeColorModel(g['quality']['camera_model']);q0=np.array(g['accepted_positions_rad'][:6]);X=np.array(g['T_tool_camera']);B=np.array(g['T_camera_board']);Y=fk(q0)@X@B
world=Y[:3,:3]@point+Y[:3,3]
corners=np.array([[x,y,0,1.] for x in [0.,.18] for y in [0.,.135]])
normal=Y[:3,2].copy()
if normal[2]<0:normal=-normal
offset=-normal@Y[:3,3]
gr=GripperGeometry(.05,.002,np.array([.0434,.0286,.06]),np.array([.1175,.155,.0774]),.003)

def project_link(pc):
 return np.asarray([model.sdk.rs2_project_point_to_pixel(model.intrinsics,[-p[1],-p[2],p[0]]) for p in np.atleast_2d(pc)])
def metrics(q):
 T=fk(q);C=G.inv(T@X);pc=C[:3,:3]@world+C[:3,3];uv=project_link(pc)[0]
 bp=(C@Y@corners.T).T[:,:3];pixels=project_link(bp)
 edge=float(min(pixels[:,0].min(),639-pixels[:,0].max(),pixels[:,1].min(),479-pixels[:,1].max()))
 clearance=min(float(np.min(_box_corners(c,T[:3,:3],sz)@normal+offset)) for _,c,sz in _stage_boxes(T,gr,.04975,'y','z'))
 return uv,edge,clearance,pc[0]

proposals=[]
for preserve in [False,True]:
 def residual(delta):
  q=q0+delta;uv,edge,clearance,depth=metrics(q)
  values=list((uv-[295.,210.])/10.)+list(delta/np.radians(5)*.12)
  if preserve:values.extend(G.log_so3(fk(q0)[:3,:3].T@fk(q)[:3,:3])/np.radians(2))
  values.extend([max(0.,25-edge)/3.,max(0.,.060-clearance)/.005])
  return np.asarray(values)
 def jac(delta):
  h=1e-4
  return np.column_stack([(residual(delta+np.eye(6)[i]*h)-residual(delta-np.eye(6)[i]*h))/(2*h) for i in range(6)])
 fit=least_squares(residual,np.zeros(6),jac=jac,bounds=(-np.radians(10),np.radians(10)),max_nfev=160,ftol=1e-9,xtol=1e-9,gtol=1e-9)
 q=q0+fit.x;values=[metrics(q0+a*fit.x) for a in np.linspace(0,1,51)]
 proposals.append(dict(preserve_tool_orientation=preserve,actual_q_rad=q.tolist(),delta_deg=np.degrees(fit.x).tolist(),
  predicted_motor_pixel=values[-1][0].tolist(),predicted_pixel_radius=float(np.linalg.norm(values[-1][0]-[320,240])),
  nominal_sample_min_board_edge_px=min(v[1] for v in values),nominal_sample_min_cad_board_clearance_m=min(v[2] for v in values),
  endpoint_camera_object_depth_m=values[-1][3],cost=fit.cost,optimizer_success=fit.success))
out=R/'motor_observation_proposal';out.mkdir(exist_ok=True)
report=dict(diagnostic_only=True,motion_authorized=False,source_session=str(s),source_stamp_ns=g['source_stamp_ns'],
 current_actual_q_rad=q0.tolist(),current_projection=metrics(q0)[0].tolist(),motor_point_board=point.tolist(),proposals=proposals,
 limitations=['Archived evidence only, not live authority.','Nominal samples do not bound actual tracking or collision.','No MoveIt, SDK start bridge, fresh scene, original target identity or motor occlusion proof.','A fresh observation must replace this offline source before planning execution.'])
(out/'report.json').write_text(json.dumps(report,indent=2)+'\n');(out/'source.py').write_bytes(Path(__file__).read_bytes())
print(json.dumps(report,indent=2))
