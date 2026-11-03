"""Read-only concise terminal monitor for the current recorded grasp."""
import os,json,time
from pathlib import Path
p=Path(json.loads(Path('.runtime/latest_candidate_real_grasp.json').read_text())['directory']);os.environ['ROS_LOG_DIR']=str(p/'ros')
import rospy
from rosgraph_msgs.msg import Log
from alicia_flexible_grasp_supervisor.msg import GraspState
rospy.init_node('speed_grasp_terminal_supervision',anonymous=True,disable_signals=True)
seen={'active':False,'ended':False}
def state(m):
 if m.active:seen['active']=True
 elif seen['active']:seen['ended']=True
 print('TASK',m.header.stamp.to_sec(),m.state,m.stage,m.active,m.message[:900],flush=True)
def log(m):
 if any(k in m.msg for k in ('Pregrasp feedforward','strict trajectory retimed','Measured endpoint','motion settled','Motion settled','exact joint probe','LIFT_LINEAR_REJECTED','Strict execution trajectory','strict execution trajectory')):
  text=m.msg
  if 'Pregrasp feedforward' in text:
   try:
    d=json.loads(text[text.index('{'):]);text=json.dumps({k:d[k] for k in ('fixed_goal_pose_index','feedforward_timing','code','position_error_m','orientation_error_rad','steps_completed','hardware_response_verified','last_step_response') if k in d})
   except Exception:pass
  print('LOG',m.header.stamp.to_sec(),m.name,text[:1800],flush=True)
rospy.Subscriber('/grasp/state',GraspState,state,queue_size=20)
rospy.Subscriber('/rosout_agg',Log,log,queue_size=100)
end=time.monotonic()+300
while time.monotonic()<end and not rospy.is_shutdown() and not seen['ended']:time.sleep(.1)
print('MONITOR_DONE',seen,flush=True)
