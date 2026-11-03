import json,copy,time
from pathlib import Path
from datetime import datetime,timezone
import rospy,numpy as np
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK,GetPositionIKRequest
from moveit_msgs.msg import RobotState
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
ROOT=Path.cwd();dep=Path(json.loads((ROOT/'.runtime/current_handeye_deployment.json').read_text())['directory']);d=json.loads((dep/'first_grasp_planning_audit.json').read_text());rospy.init_node('fixed_target_contact_ik_readonly',anonymous=True,disable_signals=True)
s=rospy.wait_for_message('/alicia_d/accepted_joint_states',JointState,timeout=3);fk=SerialUrdfFk(rospy.get_param('/robot_description'),['Joint%d'%i for i in range(1,7)]);service=rospy.ServiceProxy('/compute_ik',GetPositionIK);rows=[]
for i,e in enumerate(d['stable_evaluations']):
 st=e['execution_sequence']['stages'][2];req=GetPositionIKRequest();req.ik_request.group_name='alicia';req.ik_request.ik_link_name='tool0';req.ik_request.avoid_collisions=True;req.ik_request.timeout=rospy.Duration(.12);req.ik_request.pose_stamped.header.frame_id='base_link'
 for a,v in zip('xyz',st['position_m']):setattr(req.ik_request.pose_stamped.pose.position,a,v)
 for a,v in zip('xyzw',st['quaternion_xyzw']):setattr(req.ik_request.pose_stamped.pose.orientation,a,v)
 success=None;codes=[]
 for seed_name in ['current','joint_midpoints']:
  req.ik_request.robot_state=RobotState();req.ik_request.robot_state.joint_state=copy.deepcopy(s)
  if seed_name=='joint_midpoints':req.ik_request.robot_state.joint_state.position=list(np.mean(fk.limits,axis=1))+[s.position[6]]
  res=service(req);codes.append(res.error_code.val)
  if res.error_code.val==1:
   success={'seed':seed_name,'joint_names':list(res.solution.joint_state.name),'positions':list(res.solution.joint_state.position)};break
 row={'index':i,'tilt_deg':e['adaptive_stage_profile']['tilt_deg'],'target':st,'codes':codes,'solution':success};rows.append(row)
 print('CONTACT_IK',i,'tilt',round(row['tilt_deg'],2),'success',bool(success),flush=True)
(dep/'fixed_target_contact_ik.json').write_text(json.dumps({'time':datetime.now(timezone.utc).isoformat(),'source':str(dep/'first_grasp_planning_audit.json'),'fixed_target':True,'planning_only':True,'collision_aware':True,'rows':rows,'successes':sum(r['solution'] is not None for r in rows)},ensure_ascii=False,indent=2)+'\n')
print('FINAL_CONTACT_IK_SUCCESS',sum(r['solution'] is not None for r in rows),'OF',len(rows),flush=True)
