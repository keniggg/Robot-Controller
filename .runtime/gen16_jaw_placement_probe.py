"""Offline endpoint hypothesis; no original MoveIt trajectory is available."""
from pathlib import Path
import hashlib,json,time,math
from dataclasses import asdict
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry,_stage_boxes,_box_corners,_obb_overlap
from alicia_flexible_grasp.grasp.measured_contact import evaluate_measured_contact
from alicia_flexible_grasp.grasp.contact_height_margin import evaluate_contact_height_margin

ROOT=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
NEW=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_no_skin_20261011T000625Z')
INPUT=NEW/'gen16_analysis/contact_path_replay_inputs/req13_rejected_contact_inputs.json'
MODEL=ROOT/'deployment/2026-10-08/runtime/candidate_models/expanded_metric_20260929/alicia_expanded_metric_candidate.urdf'
SHA=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
fk=SerialUrdfFk(MODEL.read_text(),ARM_NAMES)
g=GripperGeometry(.05,.002,np.array([.0434,.0286,.06]),np.array([.1175,.155,.0774]),.003)
def matrix(v):
 t=np.eye(4);t[:3,:3]=Rotation.from_quat(v[3:]).as_dcm();t[:3,3]=v[:3];return t

def endpoint(q,scene,opening):
 t=fk(q);out=[]
 for name,c,size in _stage_boxes(t,g,opening,'y','z'):
  minimum=float(np.min(_box_corners(c,t[:3,:3],size)@np.array(scene['support_normal_base'])+scene['support_offset_m']))
  out.append(dict(name=name,support_clearance_m=minimum,target_collision=bool(_obb_overlap(c,t[:3,:3],size,np.array(scene['pose_xyz_xyzw'][:3]),matrix(scene['pose_xyz_xyzw'])[:3,:3],np.array(scene['size_xyz_m'])))))
 return dict(ok=all(not x['target_collision'] and x['support_clearance_m']>=.003-1e-9 for x in out),boxes=out)

def solve(target,seed):
 def residual(q):
  t=fk(q);return np.r_[(t[:3,3]-target[:3,3])/.006,Rotation.from_dcm(target[:3,:3].T@t[:3,:3]).as_rotvec()/math.radians(5)]
 sol=least_squares(residual,np.clip(seed,fk.limits[:,0]+1e-10,fk.limits[:,1]-1e-10),bounds=(fk.limits[:,0],fk.limits[:,1]),max_nfev=120,ftol=1e-12,xtol=1e-12,gtol=1e-12)
 t=fk(sol.x);return sol.x,dict(position_error_m=float(np.linalg.norm(t[:3,3]-target[:3,3])),orientation_error_rad=float(np.linalg.norm(Rotation.from_dcm(target[:3,:3].T@t[:3,:3]).as_rotvec())),nfev=sol.nfev)

data=json.loads(INPUT.read_text());cases=[];start=time.monotonic()
for index,record in enumerate(data['rejections']):
 r=record['evidence'];sample=r['precheck']['source_sample'];bias=np.array(sample['sdk_positions_rad'])-sample['accepted_positions_rad'];scene=r['object_geometry'];goal=matrix(r['poses_xyz_xyzw'][2]);jaw=goal[:3,1];opening=r['precheck']['opening_m'];seed=np.array(r['precheck']['held_approach_prediction']['predicted_approach']['predicted_positions_rad']);contact=json.loads(r['contact_geometry_evidence']);trials=[]
 for delta in np.r_[0.,np.arange(-.010,.01001,.0005)]:
  if delta !=0 and abs(delta)<1e-10:continue
  target=goal.copy();target[:3,3]+=float(delta)*jaw
  q,sol=solve(target,seed);command=q+bias
  row=dict(jaw_translation_m=float(delta),nominal_joint_positions_rad=q.tolist(),predicted_command_joint_positions_rad=command.tolist(),ik=sol)
  if sol['position_error_m']<.000001 and sol['orientation_error_rad']<.00001 and np.all(command>=fk.limits[:,0]) and np.all(command<=fk.limits[:,1]):
   row['nominal']=endpoint(q,scene,opening);row['predicted_command']=endpoint(command,scene,opening)
   t=fk(q);cmd=fk(command);error=cmd[:3,3]-t[:3,3];row['command_minus_nominal_translation_m']=error.tolist();row['command_jaw_shift_m']=float(error@jaw);row['command_orientation_delta_rad']=float(np.linalg.norm(Rotation.from_dcm(t[:3,:3].T@cmd[:3,:3]).as_rotvec()))
   args=dict(gripper=g,planned_T_base_tool0=target,contact_center_base=contact['contact_center_base'],contact_height_axis_base=contact['contact_height_axis_base'],contact_height_bounds_m=contact['contact_height_bounds_m'],minimum_contact_overlap_m=contact['minimum_contact_overlap_m'],support_normal_base=scene['support_normal_base'],support_offset_m=scene['support_offset_m'],opening_width_m=opening)
   row['contact']=asdict(evaluate_measured_contact(actual_T_base_tool0=t,**args));row['height_margin']=evaluate_contact_height_margin(translation_evidence_limit_m=.0026767676132463574,actual_T_base_tool0=t,**args)
   row['endpoint_and_frozen_contact_pass']=bool(row['nominal']['ok'] and row['predicted_command']['ok'] and row['contact']['ok'] and row['height_margin']['ok'])
  else:row['endpoint_and_frozen_contact_pass']=False
  trials.append(row)
 passing=[v for v in trials if v['endpoint_and_frozen_contact_pass']]
 case=dict(receipt_index=index,plan_id=r['plan_id'],operation=r['operation'],source_stamp_ns=r['source_stamp_ns'],object_geometry=scene,original_goal=r['poses_xyz_xyzw'][2],sdk_minus_encoder_rad=bias.tolist(),opening_m=opening,contact_geometry=contact,trials=trials,passing_offsets_m=[v['jaw_translation_m'] for v in passing])
 cases.append(case)
 zero=trials[0];print(index,r['plan_id'],'zero',zero['ik'],'commandjawmm',1000*zero.get('command_jaw_shift_m',float('nan')),'nom/cmd',zero.get('nominal',{}).get('ok'),zero.get('predicted_command',{}).get('ok'),'pass offsets mm',[round(v['jaw_translation_m']*1000,3) for v in passing],flush=True)
report=dict(scope='offline_reconstructed_endpoint_jaw_placement_hypothesis',execution_authorized=False,input_path=str(INPUT),input_sha256=SHA(INPUT),model_sha256=SHA(MODEL),elapsed_sec=time.monotonic()-start,cases=cases,limitations=['Original failed grasp trajectory and colliding q were not recorded. These are newly solved kinematic endpoints seeded from recorded predicted approach, not exact original MoveIt endpoints.','Frozen contact-band overlap is tested but no new measured bilateral association or four-stage/full-arm collision path is certified.','Goals move along existing tool jaw axis; object OBB, every original point, support plane and numerical hard gates stay unchanged.'])
out=NEW/'gen16_analysis/jaw_placement_endpoint_hypothesis.json';assert not out.exists();out.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n');print(out,SHA(out),flush=True)
