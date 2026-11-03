import json,sys,copy
from pathlib import Path
import rospy
from sensor_msgs.msg import JointState
ROOT=Path.cwd();sys.path.insert(0,str(ROOT/'tools'))
from replay_grasp_ik_readonly import replay
p=Path(json.loads((ROOT/'.runtime/current_handeye_deployment.json').read_text())['directory']);d=json.loads((p/'first_grasp_planning_audit.json').read_text());rospy.init_node('replay_unchecked_current_grasp',anonymous=True,disable_signals=True)
s=rospy.wait_for_message('/alicia_d/accepted_joint_states',JointState,timeout=3);seed={'available':True,'name':list(s.name),'position_rad':list(s.position),'service_start_state_exact':False,'source':'fresh accepted_joint_states for planning-only diagnostic','stamp_ns':s.header.stamp.to_nsec()}
with (p/'unchecked_alternative_replay.jsonl').open('w') as f:
 for i in [46,58]:
  e=copy.deepcopy(d['stable_evaluations'][i]);e['moveit_input_joint_state']=seed
  try:r=replay(e,15)
  except Exception as ex:r={'error':str(ex)}
  r.update(index=i,live_seed_used=True,execution_authorized=False);line=json.dumps(r,ensure_ascii=False);f.write(line+'\n');f.flush();print(line,flush=True)
