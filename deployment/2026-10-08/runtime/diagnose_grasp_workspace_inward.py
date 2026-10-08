import json,sys,copy
from pathlib import Path
import rospy,numpy as np
from sensor_msgs.msg import JointState
ROOT=Path.cwd();sys.path.insert(0,str(ROOT/'tools'))
from replay_grasp_ik_readonly import replay
p=Path(json.loads((ROOT/'.runtime/current_handeye_deployment.json').read_text())['directory']);d=json.loads((p/'first_grasp_planning_audit.json').read_text());rospy.init_node('hypothetical_inward_grasp_readonly',anonymous=True,disable_signals=True)
s=rospy.wait_for_message('/alicia_d/accepted_joint_states',JointState,timeout=3);seed={'available':True,'name':list(s.name),'position_rad':list(s.position),'service_start_state_exact':False}
n=np.array(d['replay_geometry']['support_normal_base']);center=np.array(d['target_cloud_center_base']);h=-center[:2]/np.linalg.norm(center[:2]);results=[]
for horizontal in [.04,.06]:
 delta=np.r_[h*horizontal,0.];delta[2]=-np.dot(n[:2],delta[:2])/n[2];e=copy.deepcopy(d['stable_evaluations'][0]);e['moveit_input_joint_state']=seed
 for st in e['execution_sequence']['stages']:st['position_m']=(np.array(st['position_m'])+delta).tolist()
 try:r=replay(e,15)
 except Exception as ex:r={'error':str(ex)}
 r.update(hypothetical_only=True,execution_authorized=False,displacement_m=delta.tolist(),horizontal_inward_distance_m=horizontal,all_stages_shifted_together=True);results.append(r);print(json.dumps(r,ensure_ascii=False),flush=True)
 if r.get('success'):break
(p/'hypothetical_inward_workspace.json').write_text(json.dumps(results,ensure_ascii=False,indent=2)+'\n')
