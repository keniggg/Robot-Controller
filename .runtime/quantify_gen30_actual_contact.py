"""Actual rejected plan/evidence; independent IK is diagnostic, not recorded trajectory."""
from pathlib import Path
import json,hashlib,math
import numpy as np
import yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from tf.transformations import quaternion_matrix
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry,_stage_boxes,_box_corners,_obb_overlap
from alicia_flexible_grasp.grasp.contact_height_margin import evaluate_contact_height_margin
from alicia_flexible_grasp.grasp.contact_height_placement import contact_height_window_proposals
B=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_ros_20261009T003407Z/gen30_contact_analysis');report=json.loads((B/'actual_contact_rejections.json').read_text());data=report['rows'][0]['message'];sample=data['precheck']['source_sample'];ev=json.loads(data['contact_geometry_evidence'])
conf=Path('src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml');cfg=yaml.safe_load(conf.read_text());gg=cfg['grasp_6d']['remote']['gripper_geometry'];g=GripperGeometry(gg['max_inner_gap_m'],gg['width_safety_margin_per_side_m'],np.array(gg['finger_box_xyz_m']),np.array(gg['palm_box_xyz_m']),gg.get('support_clearance_m',.003));limit=cfg['grasp']['handeye_translation_max_error_m']
model=Path('.runtime/candidate_models/expanded_metric_20260929/alicia_expanded_metric_candidate.urdf');fk=SerialUrdfFk(model.read_text(),tuple(sample['joint_names']));assert fk.model_sha256==sample['model_sha256'];seed=np.array(sample['accepted_positions_rad']);bias=np.array(sample['sdk_positions_rad'])-seed;opening=data['precheck']['opening_m']
def matrix(v):
 T=quaternion_matrix(v[3:]);T[:3,3]=v[:3];return T
Ts=[matrix(v) for v in data['poses_xyz_xyzw']];G=matrix(data['object_geometry']['pose_xyz_xyzw']);center=G[:3,3];axes=G[:3,:3];size=np.array(data['object_geometry']['size_xyz_m']);normal=np.array(data['object_geometry']['support_normal_base']);offset=data['object_geometry']['support_offset_m']
def convert(v):
 if isinstance(v,np.ndarray):return v.tolist()
 if isinstance(v,np.generic):return v.item()
 if isinstance(v,dict):return {str(k):convert(x) for k,x in v.items()}
 if isinstance(v,(tuple,list)):return [convert(x) for x in v]
 return v
def sat(ca,ra,sa,cb,rb,sb):
 gaps=[]
 for a in [*ra.T,*rb.T]+[np.cross(a,b) for a in ra.T for b in rb.T]:
  if np.linalg.norm(a)<1e-10:continue
  a=a/np.linalg.norm(a);gaps.append(abs((ca-cb)@a)-abs(ra.T@a)@(sa/2)-abs(rb.T@a)@(sb/2))
 return float(max(gaps))
def geom(T):
 return dict(jaw_projection_width_m=float(abs(axes.T@T[:3,1])@size),target_center_in_tool=(center-T[:3,3])@T[:3,:3],bodies=[dict(body=name,obb_overlap=bool(_obb_overlap(c,T[:3,:3],s,center,axes,size)),sat_separation_m=sat(c,T[:3,:3],s,center,axes,size),minimum_support_clearance_m=float(np.min(_box_corners(c,T[:3,:3],s)@normal+offset))) for name,c,s in _stage_boxes(T,g,opening,'y','z')])
def solve(T,start):
 def residual(q):
  actual=fk(q);return np.r_[actual[:3,3]-T[:3,3],.2*Rotation.from_dcm(T[:3,:3].T@actual[:3,:3]).as_rotvec()]
 fit=least_squares(residual,start,bounds=(fk.limits[:,0],fk.limits[:,1]),max_nfev=200,ftol=1e-12,xtol=1e-12,gtol=1e-12)
 err=residual(fit.x);return fit.x,dict(position_error_m=float(np.linalg.norm(err[:3])),orientation_error_rad=float(np.linalg.norm(err[3:]/.2)),iterations=fit.nfev,success=bool(fit.success))
hgeometry=dict(gripper=g,actual_T_base_tool0=Ts[2],planned_T_base_tool0=Ts[2],contact_center_base=np.array(ev['contact_center_base']),contact_height_axis_base=np.array(ev['contact_height_axis_base']),contact_height_bounds_m=tuple(ev['contact_height_bounds_m']),minimum_contact_overlap_m=ev['minimum_contact_overlap_m'],support_normal_base=normal,support_offset_m=offset,opening_width_m=opening)
height=evaluate_contact_height_margin(translation_evidence_limit_m=limit,**hgeometry);options=[(Ts[2],dict(normal_translation_m=0.))]+list(contact_height_window_proposals(translation_evidence_limit_m=limit,**hgeometry));trials=[]
for goal,placement in options:
 q,ik=solve(goal,seed);nom=fk(q);command=fk(q+bias);ng=geom(nom);cg=geom(command);shift=command[:3,3]-nom[:3,3]
 trials.append(dict(delta_m=placement['normal_translation_m'],height_proposal=placement,ik=ik,q=q,bias=bias,nominal_T=nom,predicted_command_T=command,nominal=ng,predicted_command=cg,command_in_joint_limits=bool(np.all(q+bias>=fk.limits[:,0]) and np.all(q+bias<=fk.limits[:,1])),command_minus_nominal_position_m=shift,command_minus_nominal_components_m=dict(support_normal=float(shift@normal),tool_jaw=float(shift@nom[:3,1]),tool_cross=float(shift@nom[:3,0])),endpoint_pair_clear=not any(x['obb_overlap'] or x['minimum_support_clearance_m']<g.support_clearance_m for geo in (ng,cg) for x in geo['bodies'])))
 # Generic original geometry path test requires original failed MoveIt joints; not generated here.
out=dict(scope='ACTUAL_REJECTED_PLAN_WITH_INDEPENDENT_OFFLINE_ENDPOINT_IK',actual_records=2,same_plan_both_attempts=data['plan_id']==report['rows'][1]['message']['plan_id'],actual_failure=data['precheck']['reason'],source_stamp_ns=data['source_stamp_ns'],plan_id=data['plan_id'],sample=sample,actual_pose_nominal_geometry=[dict(stage=name,geometry=geom(T)) for name,T in zip(('pregrasp','approach','grasp','lift'),Ts)],contact_height_limit_m=limit,height_margin=height,trials=trials,sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [B/'actual_contact_rejections.json',model,conf,Path(__file__)]},limitations=['The two actual service rejections are exact. No failed MoveIt trajectory or exact IK branch was logged.','Offline FK/IK uses the logged final four poses, frozen OBB, actual opening and actual sampled SDK-minus-encoder offset. It is not actual future movement.','Contact-height options are the unchanged production helper and do not grant planning/execution authority.','No fused cloud or full-arm collision service is replaced with an invented result; sampled endpoint clearance does not establish a full continuous safe path.'])
p=B/'actual_contact_endpoint_counterfactual.json';p.write_text(json.dumps(convert(out),indent=2)+'\n');print(json.dumps(convert(dict(result=str(p),height=height,trials=[{k:v for k,v in t.items() if k not in ('height_proposal','q','bias','nominal_T','predicted_command_T')} for t in trials])),indent=2))
