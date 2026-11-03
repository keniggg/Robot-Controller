import json,time,numpy as np,rospy
from std_msgs.msg import String
from handeye_auto_motion import AutoMotion
from plan_task_pose_sweep import out,segment
rospy.init_node('task_pose_return_transition',anonymous=True,disable_signals=True)
m=AutoMotion(out);m.task_pose_directional_reservation=True
def guard(q):
 ok,data=segment(np.array(m.q[:6]),np.array(q[:6]))
 if not ok:raise RuntimeError('return transition outside reserved view/clearance: '+json.dumps(data))
m.command_guard=guard
m.emit('return_transition_begin',reason='local shoulder/wrist ordering dead-end; two bounded wrist4 steps reserve space for return',source_pose_id=20,ordered_joint_steps_deg=[[4,-4.],[4,-4.],[2,-3.],[5,4.]],absolute_response_bound_deg=4.8)
for joint,delta in [(3,-4.),(3,-4.),(1,-3.),(4,4.)]:
 deadline=time.monotonic()+5;valid=False
 while time.monotonic()<deadline:
  m.guard();q=json.loads(rospy.wait_for_message('/charuco/quality',String,timeout=3).data)
  if q.get('pose_ok') and q.get('charuco_count',0)>=35 and q.get('edge_clearance_px',0)>=12:valid=True;break
 if not valid:raise RuntimeError('board observation unavailable after settling')
 m.move(joint,delta)
m.emit('return_transition_complete',q_deg=np.degrees(m.q[:6]).tolist())
