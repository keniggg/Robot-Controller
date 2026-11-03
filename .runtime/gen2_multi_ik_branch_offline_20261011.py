"""At most 115 wall seconds, deterministic offline IK branch diagnostic."""
from pathlib import Path
import hashlib,json,sys,math,time,itertools
import numpy as np
from scipy.optimize import least_squares
from tf.transformations import quaternion_matrix,quaternion_from_matrix
WORK=Path(__file__).resolve().parents[1];sys.path.insert(0,str(WORK/'src/alicia_flexible_grasp_supervisor/src'))
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES
from alicia_flexible_grasp.robot.contact_sequence_precheck import contact_precheck_gripper
from alicia_flexible_grasp.grasp.gripper_geometry import _stage_boxes,_box_corners,_obb_overlap
assert Path('/media/zhuyupei/SanStand').is_mount()
OUT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_no_skin_202611T000625Z/gen2_analysis/contact_collision')
# Date path is explicit to avoid any fallback to the system disk.
OUT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_no_skin_20261011T000625Z/gen2_analysis/contact_collision')
src=OUT/'offline_reconstruction.json';r=json.loads(src.read_text());screen_src=OUT/'recorded_48_pose_endpoint_screen.json';screen=json.loads(screen_src.read_text())
fk=SerialUrdfFk(Path(r['model_path']).read_text(),ARM_NAMES);gripper=contact_precheck_gripper(r['gripper_config']);opening=r['opening_m'];offset=np.array(r['sdk_minus_encoder_rad']);obj=r['scene']
normal=np.array(obj['support_normal_base']);center=np.array(obj['pose_xyz_xyzw'][:3]);rotation=quaternion_matrix(obj['pose_xyz_xyzw'][3:])[:3,:3];size=np.array(obj['size_xyz_m'])
best=min(screen['candidates'],key=lambda x:-x['minimum_sat_gap_m'])
goals=[dict(name='first_recorded_final_contact',pose=r['registered_goal_pose'],q=r['paths'][-1]['predicted_actual']['q_rad'],provenance='exact logged final registered goal'),dict(name='recorded_track6_variant0',pose=best['registered_proposal_pose'],q=best['q'],provenance='recorded stable proposal transformed by recorded registration; later final resolver not executed')]
def boxes(T):
 rows=[]
 for name,c,s in _stage_boxes(T,gripper,opening,'y','z'):
  axes=[*T[:3,:3].T,*rotation.T]+[np.cross(a,b)for a in T[:3,:3].T for b in rotation.T]
  sep=max(abs(float((c-center)@(a/np.linalg.norm(a))))-float(abs(T[:3,:3].T@(a/np.linalg.norm(a)))@(s/2))-float(abs(rotation.T@(a/np.linalg.norm(a)))@(size/2))for a in axes if np.linalg.norm(a)>1e-10)
  rows.append(dict(body=name,sat_gap_m=sep,overlap=bool(_obb_overlap(c,T[:3,:3],s,center,rotation,size)),support_clearance_m=float(np.min(_box_corners(c,T[:3,:3],s)@normal+obj['support_offset_m']))))
 return rows
began=time.monotonic();reports=[];rng=np.random.RandomState(2402)
for goal in goals:
 Tgoal=quaternion_matrix(goal['pose'][3:]);Tgoal[:3,3]=goal['pose'][:3];qbase=np.array(goal['q']);seeds=[qbase,np.mean(fk.limits,axis=1)]
 for shoulder,elbow,wrist in itertools.product((0.,math.pi),(-1.,1.),(0.,math.pi)):
  s=qbase.copy();s[0]+=shoulder;s[2]*=elbow;s[3]+=wrist;s[4]*=(-1 if wrist else 1);s[5]+=wrist
  s=(s+math.pi)%(2*math.pi)-math.pi;seeds.append(np.clip(s,fk.limits[:,0]+1e-7,fk.limits[:,1]-1e-7))
 seeds.extend(rng.uniform(fk.limits[:,0]+1e-7,fk.limits[:,1]-1e-7,(48,6)))
 branches=[];attempts=[]
 for i,seed in enumerate(seeds):
  if time.monotonic()-began>=115.:break
  def residual(q):
   if time.monotonic()-began>=115.:raise TimeoutError('bounded diagnostic 115s budget')
   T=fk(q);return np.r_[20*(T[:3,3]-Tgoal[:3,3]),(T[:3,:3]-Tgoal[:3,:3]).ravel()]
  try:sol=least_squares(residual,seed,bounds=(fk.limits[:,0]+1e-9,fk.limits[:,1]-1e-9),max_nfev=160,xtol=1e-11,ftol=1e-11,gtol=1e-11)
  except TimeoutError:break
  q=sol.x;T=fk(q);position=float(np.linalg.norm(T[:3,3]-Tgoal[:3,3]));angle=float(math.acos(np.clip((np.trace(T[:3,:3].T@Tgoal[:3,:3])-1)/2,-1,1)))
  matched=position<1e-6 and angle<1e-5;branch=None
  if matched:
   branch=next((j for j,b in enumerate(branches)if np.max(np.abs(np.array(b['q_actual'])-q))<1e-4),None)
   if branch is None:
    a=boxes(T)
    try:command=fk(q+offset);b=boxes(command);failure=None
    except ValueError as exc:b=[];failure=str(exc)
    branch=len(branches);branches.append(dict(branch=branch,seed_index=i,seed_rad=np.asarray(seed).tolist(),q_actual=q.tolist(),q_command=(q+offset).tolist(),position_error_m=position,orientation_error_rad=angle,actual_boxes=a,command_boxes=b,command_failure=failure,endpoint_pass=bool(b and not any(x['overlap']for x in a+b)and min(x['support_clearance_m']for x in a+b)>=.003),min_sat_gap_m=min(x['sat_gap_m']for x in a+b)))
  attempts.append(dict(seed_index=i,seed_rad=np.asarray(seed).tolist(),position_error_m=position,orientation_error_rad=angle,matched=matched,branch=branch,nfev=int(sol.nfev)))
 reports.append(dict(goal=goal,seeds_total=len(seeds),attempts=attempts,branches=branches))
 print(goal['name'],'attempts',len(attempts),'branches',len(branches),'passes',sum(b['endpoint_pass']for b in branches),flush=True)
report=dict(scope='bounded_deterministic_multiseed_offline_IK_branch_screen',model_sha256=fk.model_sha256,limits_rad=fk.limits.tolist(),original_sdk_minus_encoder_rad=offset.tolist(),source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest()for p in [src,screen_src,Path(__file__)]},maximum_wall_seconds=115.,elapsed_sec=time.monotonic()-began,reports=reports,globally_exhaustive=False,full_path_checked=False,moveit_or_hardware_called=False,production_changed=False,interpretation='Only original candidate goal poses and recorded current-bias conditional model; distinct branches are not authorized paths. Endpoint candidates still require fresh full geometry, path, ownership and hardware evidence.')
out=OUT/'multi_ik_branch_screen.json';out.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(dict(output=str(out),elapsed=report['elapsed_sec'],summary=[dict(name=x['goal']['name'],attempts=len(x['attempts']),branches=len(x['branches']),passing=[b for b in x['branches']if b['endpoint_pass']])for x in reports]),indent=2))
