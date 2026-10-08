import sys,json,pathlib,numpy as np,rospy
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_from_matrix
from alicia_flexible_grasp.grasp.adaptive_stage_profiles import AdaptiveStageLimits,adaptive_stage_profile_for_tilt
sys.path.insert(0,str(pathlib.Path.cwd()/'tools'))
from replay_grasp_ik_readonly import replay
p=pathlib.Path(json.load(open('.runtime/current_handeye_deployment.json'))['directory']);rows=json.load(open(p/'wider_support_candidate_replay.json'));rospy.init_node('wider_support_path_replay_readonly',anonymous=True,disable_signals=True)
s=rospy.wait_for_message('/alicia_d/accepted_joint_states',JointState,timeout=3);seed=dict(available=True,name=list(s.name),position_rad=list(s.position),service_start_state_exact=False)
out=[]
for row in rows:
 g=row['geometry'];n=np.array(g['normal']);candidates=sorted(enumerate(row['results']),key=lambda r:(abs(r[1]['tilt']-6),r[1]['required_open_width_m']))
 for i,c in candidates[:12]:
  T=np.array(c['T']);rot=quaternion_from_matrix(T);tool=T[:3,3];prof=adaptive_stage_profile_for_tilt(tilt_deg=c['tilt'],object_height_m=g['size'][2],depth_repeatability_m=0.,finger_length_m=.06,support_clearance_m=.003,limits=AdaptiveStageLimits());poses=[tool+prof.pregrasp_distance_m*n,tool-prof.approach_offset_m*T[:3,2],tool,tool+prof.lift_height_m*n]
  ev=dict(moveit_input_joint_state=seed,execution_sequence=dict(stages=[dict(stage=name,frame_id='base_link',stamp_ns=0,position_m=pos.tolist(),quaternion_xyzw=rot.tolist()) for name,pos in zip(('pregrasp','approach','grasp','lift'),poses)]))
  result=replay(ev,3);item=dict(source_audit=row['source_audit'],candidate_index=i,margin_m=c['margin_m'],result=result,sequence=ev)
  out.append(item);(p/'wider_support_path_replay.json').write_text(json.dumps(out,indent=2)+'\n');print(item['source_audit'],i,c['tilt'],c['polarity'],result['success'],result['reason'][:120],flush=True)
  if result['success']:break
