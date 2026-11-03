"""Offline screen of already recorded candidates; not full MoveIt validation."""
from pathlib import Path
import hashlib,json,sys,math,time
import numpy as np
from scipy.optimize import least_squares
from tf.transformations import quaternion_matrix,quaternion_from_matrix
WORK=Path(__file__).resolve().parents[1];sys.path.insert(0,str(WORK/'src/alicia_flexible_grasp_supervisor/src'))
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES
from alicia_flexible_grasp.robot.contact_sequence_precheck import contact_precheck_gripper
from alicia_flexible_grasp.grasp.gripper_geometry import _stage_boxes,_box_corners,_obb_overlap
assert Path('/media/zhuyupei/SanStand').is_mount()
EXT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
OUT=EXT/'ros_log/full_latest_no_skin_20261011T000625Z/gen2_analysis/contact_collision'
source=OUT/'offline_reconstruction.json';record=json.loads(source.read_text());rows=json.loads((OUT/'recorded_candidates.json').read_text())
audit_path=EXT/'grasp_records/20261011T003714.176499Z_8cd314e96edb4eba8d04c92b93166af5/keyframes_audits/9e3458796c4d2cce6f5414ef1f1ed1beecf2eb35ed3697d5924b84f78637ca26.json'
audit=json.loads(audit_path.read_text());C=np.array(audit['multiview_surface']['latest_registration']['snapshot_to_reference_transform'])
fk=SerialUrdfFk(Path(record['model_path']).read_text(),ARM_NAMES);g=contact_precheck_gripper(record['gripper_config']);opening=record['opening_m'];offset=np.array(record['sdk_minus_encoder_rad'])
obj=record['scene'];n=np.array(obj['support_normal_base']);center=np.array(obj['pose_xyz_xyzw'][:3]);rotation=quaternion_matrix(obj['pose_xyz_xyzw'][3:])[:3,:3];size=np.array(obj['size_xyz_m'])
seeds=[np.array(x['evidence']['precheck']['stage_start_prediction']['approach']['predicted_actual_joint_positions_rad'])for x in rows if x['evidence']['operation']=='final_registered_strict_sequence']
def boxes(T):
 result=[]
 for name,c,s in _stage_boxes(T,g,opening,'y','z'):
  axes=[*T[:3,:3].T,*rotation.T]+[np.cross(a,b)for a in T[:3,:3].T for b in rotation.T]
  sep=max(abs(float((c-center)@(a/np.linalg.norm(a))))-float(abs(T[:3,:3].T@(a/np.linalg.norm(a)))@(s/2))-float(abs(rotation.T@(a/np.linalg.norm(a)))@(size/2))for a in axes if np.linalg.norm(a)>1e-10)
  result.append(dict(body=name,sat_gap_m=sep,overlap=bool(_obb_overlap(c,T[:3,:3],s,center,rotation,size)),support_clearance_m=float(np.min(_box_corners(c,T[:3,:3],s)@n+obj['support_offset_m']))))
 return result
out=[];began=time.monotonic()
for index,r in enumerate(audit['stable_evaluations']):
 stages=r['execution_sequence']['stages'];pose=next(x for x in stages if x['stage']=='grasp');T=quaternion_matrix(pose['quaternion_xyzw']);T[:3,3]=pose['position_m'];target=C@T
 seed=min(seeds,key=lambda q:np.linalg.norm(fk(q)[:3,:3]-target[:3,:3]))
 def res(q):
  A=fk(q);return np.r_[20*(A[:3,3]-target[:3,3]),(A[:3,:3]-target[:3,:3]).ravel()]
 sol=least_squares(res,seed,bounds=(fk.limits[:,0],fk.limits[:,1]),max_nfev=100,xtol=1e-11,ftol=1e-11,gtol=1e-11)
 actual=fk(sol.x);command=fk(sol.x+offset)
 a,b=boxes(actual),boxes(command);err=float(np.linalg.norm(actual[:3,3]-target[:3,3]));angle=float(math.acos(np.clip((np.trace(actual[:3,:3].T@target[:3,:3])-1)/2,-1,1)))
 out.append(dict(index=index,track=r['tracking']['track_id'],variant=r['variant_index'],source_index=r['source_index'],recorded_moveit=r['moveit'],registered_proposal_pose=np.r_[target[:3,3],quaternion_from_matrix(target)].tolist(),q=sol.x.tolist(),ik_position_error_m=err,ik_orientation_error_rad=angle,actual=a,command=b,endpoint_both_collision_free=bool(err<.0005 and angle<.005 and not any(x['overlap']for x in a+b) and min(x['support_clearance_m']for x in a+b)>=.003),minimum_sat_gap_m=min(x['sat_gap_m']for x in a+b)))
report=dict(scope='offline_recorded_48_stable_proposal_endpoint_screen_not_full_registered_recheck',request_id=40,audit_path=str(audit_path),audit_sha256=hashlib.sha256(audit_path.read_bytes()).hexdigest(),source_reconstruction_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),model_sha256=fk.model_sha256,registration=audit['multiview_surface']['latest_registration'],recorded_stable_candidates=len(out),elapsed_sec=time.monotonic()-began,candidates=out,endpoint_pass_count=sum(x['endpoint_both_collision_free']for x in out),limitations=['Exact recorded pre-registration stable proposal poses transformed once by exact recorded registration; later final orientation/height resolvers were NOT synthesized.','One local IK branch per recorded candidate, seeded from nearest recorded checked approach orientation; not global IK infeasibility proof.','No ROS, motion, MoveIt, full path, new candidate generation or new bilateral-contact measurement.','Endpoint pass, if any, would still require original complete registered geometry/contact/owner/path gates.'],production_changed=False)
(OUT/'recorded_48_pose_endpoint_screen.json').write_text(json.dumps(report,indent=2)+'\n')
best=sorted(out,key=lambda x:x['minimum_sat_gap_m'],reverse=True)[:3]
print(json.dumps(dict(output=str(OUT/'recorded_48_pose_endpoint_screen.json'),count=len(out),passes=report['endpoint_pass_count'],elapsed=report['elapsed_sec'],best=[{k:x[k]for k in ['index','track','variant','minimum_sat_gap_m','ik_position_error_m','ik_orientation_error_rad','endpoint_both_collision_free']}for x in best]),indent=2))
