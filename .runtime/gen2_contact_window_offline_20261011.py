"""Bounded offline candidate-position diagnostic, no production/ROS writes."""
from pathlib import Path
import hashlib,json,sys,math
import numpy as np
from scipy.optimize import least_squares
from tf.transformations import quaternion_matrix
WORK=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(WORK/'src/alicia_flexible_grasp_supervisor/src'))
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES
from alicia_flexible_grasp.robot.contact_sequence_precheck import contact_precheck_gripper
from alicia_flexible_grasp.grasp.contact_height_margin import evaluate_contact_height_margin
from alicia_flexible_grasp.grasp.gripper_geometry import _stage_boxes,_box_corners,_obb_overlap
assert Path('/media/zhuyupei/SanStand').is_mount()
OUT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_no_skin_20261011T000625Z/gen2_analysis/contact_collision')
source=OUT/'offline_reconstruction.json';record=json.loads(source.read_text());candidate=json.loads((OUT/'recorded_candidates.json').read_text())[0]['evidence']
fk=SerialUrdfFk(Path(record['model_path']).read_text(),ARM_NAMES)
g=contact_precheck_gripper(record['gripper_config']);opening=record['opening_m']
goal=quaternion_matrix(record['registered_goal_pose'][3:]);goal[:3,3]=record['registered_goal_pose'][:3]
e=json.loads(candidate['contact_geometry_evidence']);obj=record['scene'];n=np.array(obj['support_normal_base']);center=np.array(obj['pose_xyz_xyzw'][:3]);rotation=quaternion_matrix(obj['pose_xyz_xyzw'][3:])[:3,:3];size=np.array(obj['size_xyz_m']);offset=np.array(record['sdk_minus_encoder_rad']);q0=np.array(record['paths'][-1]['predicted_actual']['q_rad'])
args=dict(gripper=g,planned_T_base_tool0=goal,contact_center_base=e['contact_center_base'],contact_height_axis_base=e['contact_height_axis_base'],contact_height_bounds_m=e['contact_height_bounds_m'],minimum_contact_overlap_m=e['minimum_contact_overlap_m'],support_normal_base=n,support_offset_m=obj['support_offset_m'],opening_width_m=opening)
limit=.0026767676132463574
margin=evaluate_contact_height_margin(actual_T_base_tool0=goal,translation_evidence_limit_m=limit,**args)
interval=next((lo+limit,hi-limit)for lo,hi in margin['allowed_normal_translation_intervals_m']if lo<=-limit and hi>=limit)
def gaps(T):
 rows=[]
 for name,c,s in _stage_boxes(T,g,opening,'y','z'):
  axes=[*T[:3,:3].T,*rotation.T]+[np.cross(a,b)for a in T[:3,:3].T for b in rotation.T]
  sep=max(abs(float((c-center)@(a/np.linalg.norm(a))))-float(abs(T[:3,:3].T@(a/np.linalg.norm(a)))@(s/2))-float(abs(rotation.T@(a/np.linalg.norm(a)))@(size/2))for a in axes if np.linalg.norm(a)>1e-10)
  rows.append(dict(body=name,sat_gap_m=sep,overlap=bool(_obb_overlap(c,T[:3,:3],s,center,rotation,size)),support_clearance_m=float(np.min(_box_corners(c,T[:3,:3],s)@n+obj['support_offset_m']))))
 return rows
def evaluate(delta):
 target=goal.copy();target[:3,3]+=delta
 def res(q):
  T=fk(q);return np.r_[20*(T[:3,3]-target[:3,3]),(T[:3,:3]-target[:3,:3]).ravel()]
 sol=least_squares(res,q0,bounds=(fk.limits[:,0],fk.limits[:,1]),max_nfev=80,xtol=1e-12,ftol=1e-12,gtol=1e-12)
 assert np.linalg.norm(fk(sol.x)[:3,3]-target[:3,3])<1e-6
 actual=fk(sol.x);command=fk(sol.x+offset);a,b=gaps(actual),gaps(command)
 hg=evaluate_contact_height_margin(actual_T_base_tool0=actual,translation_evidence_limit_m=limit,**dict(args,planned_T_base_tool0=target))
 return dict(delta_base_m=delta.tolist(),nominal_q=sol.x.tolist(),actual_boxes=a,command_boxes=b,endpoint_both_no_collision=not any(x['overlap'] for x in a+b),height_margin_ok=hg['ok'],height_margin_m=hg['maximum_symmetric_normal_translation_m'],minimum_sat_gap_m=min(x['sat_gap_m']for x in a+b))
heights=[]
for h in np.linspace(*interval,9):heights.append(dict(normal_delta_m=float(h),**evaluate(h*n)))
jaw=goal[:3,1];laterals=[]
for j in np.linspace(-.00455,0,40):laterals.append(dict(jaw_delta_m=float(j),production_materializer_lateral_center_contract_satisfied=bool(abs(j)<=1e-8),**evaluate(j*jaw)))
best=max(laterals,key=lambda x:x['minimum_sat_gap_m'])
report=dict(scope='offline_existing_height_window_and_unimplemented_lateral_counterfactual',source_report=str(source),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),model_sha256=fk.model_sha256,original_margin=margin,original_translation_evidence_limit_m=limit,existing_height_window_m=list(interval),height_samples=heights,lateral_samples=laterals,best_lateral_endpoint=best,limitations=['Lateral offsets are NOT current materialized candidates: frozen measured contact center and finger pair lateral alignment are unchanged production constraints.','Lateral results only endpoint checks; no complete motion, MoveIt, closing or bilateral remeasurement was done.','Nine height samples do not prove every point in the continuous interval fails, although original two bounded height retries also failed.','Current stationary bias is a conditional future prediction, never a verified future encoder response.'],production_or_hardware_changed=False)
(OUT/'existing_window_and_lateral_counterfactual.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(dict(output=str(OUT/'existing_window_and_lateral_counterfactual.json'),height_window_m=interval,height_endpoint_passes=sum(x['endpoint_both_no_collision']for x in heights),best_lateral=best),indent=2))
