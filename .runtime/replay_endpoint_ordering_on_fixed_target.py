import json,importlib.util,sys
from pathlib import Path
from types import SimpleNamespace as NS
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
root=Path.cwd();sys.path.insert(0,str(root/'src/alicia_flexible_grasp_supervisor'));p=root/'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py';spec=importlib.util.spec_from_file_location('fixed_target_remote_replay',p);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
dep=Path(json.loads((root/'.runtime/current_handeye_deployment.json').read_text())['directory']);d=json.loads((dep/'first_grasp_planning_audit.json').read_text());rospy.init_node('endpoint_hint_fixed_target_replay',anonymous=True,disable_signals=True)
n=m.RemoteGrasp6DNode.__new__(m.RemoteGrasp6DNode);n.latest_joint_state=rospy.wait_for_message('/alicia_d/accepted_joint_states',JointState,timeout=3);n._direct_near_field_deadline_gate=lambda _:lambda:True;n._stable_variant_runtime={};cs=[]
for i,e in enumerate(d['stable_evaluations']):
 seq={}
 for st in e['execution_sequence']['stages']:
  v=PoseStamped();v.header.frame_id='base_link';v.header.stamp=rospy.Time(st['stamp_ns']//10**9,st['stamp_ns']%10**9)
  for a,x in zip('xyz',st['position_m']):setattr(v.pose.position,a,x)
  for a,x in zip('xyzw',st['quaternion_xyzw']):setattr(v.pose.orientation,a,x)
  seq[st['stage']]=v
 c=NS(track_id=i+1,variant_index=0,pre_moveit_score=e['pre_moveit_score'],required_open_width_m=.02);cs.append(c)
 n._stable_variant_runtime[(i+1,0)]={'sequence':NS(**seq),'soft_evidence':e['latest_soft_evidence']}
report={};h=n._direct_contact_endpoint_priorities(cs,None,report);indices=[k[0]-1 for k in h];out={'fixed_target':True,'full_path_checks_still_required':True,'hinted_original_indices':indices,'diagnostics':report,'probes':[dict(original_index=i,probe=n._stable_variant_runtime[(i+1,0)].get('endpoint_ik_search_probe')) for i in range(len(cs))]};(dep/'endpoint_ordering_frozen_replay.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n');print(json.dumps({'hints':indices,'report':report},ensure_ascii=False))
