"""Reload idle task only; no joint command or enable-state mutation."""
import datetime, hashlib, json, subprocess, time, xmlrpc.client
from pathlib import Path
import rospy, rosgraph
from std_srvs.srv import Trigger
from std_msgs.msg import String, Header
from alicia_flexible_grasp_supervisor.msg import GraspState
from alicia_flexible_grasp_supervisor.srv import TriggerZero
ROOT=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
OUT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/resumed_observe_return_grasp_20260930T040740Z')
rospy.init_node('reload_bound_observation_task_only',anonymous=True,disable_signals=True)
master=rosgraph.Master(rospy.get_name())
names=('/alicia_d_driver_node','/motion_gateway','/grasp_task_node','/remote_grasp6d_node','/supervisor_camera')
def pids():
 return {n:xmlrpc.client.ServerProxy(master.lookupNode(n)).getPid(rospy.get_name())[2] for n in names}
s=rospy.wait_for_message('/grasp/state',GraspState,timeout=4)
if s.active: raise RuntimeError('task active; will not reload')
before=pids()
selection=rospy.get_param('/grasp_mode/selection')
r=dict(started_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),before_pids=before,selection=selection,task_active_before=False,robot_motion_command_sent=False,torque_command_sent=False,files={})
for name in ('src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py','src/alicia_grasp_modes/scripts/mode_aware_grasp_task.py','src/alicia_grasp_modes/scripts/mode_aware_grasp_runner.py'):
 r['files'][name]=hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
try:
 stop=rospy.ServiceProxy('/grasp_6d/request_plan',TriggerZero)(False)
 if not stop.success: raise RuntimeError(stop.message)
 node=xmlrpc.client.ServerProxy(master.lookupNode('/grasp_task_node'))
 try:
  response=node.shutdown(rospy.get_name(),'load tested observation distance correction while idle')
  if response[0]!=1: raise RuntimeError(str(response))
 except (ConnectionError, OSError) as exc:
  # A rospy node may exit before XML-RPC can return its shutdown reply.
  # The exact old PID exit is independently required below.
  r['shutdown_transport_result']=repr(exc)
 deadline=time.monotonic()+12.
 while time.monotonic()<deadline and Path('/proc/'+str(before['/grasp_task_node'])).exists(): time.sleep(.1)
 if Path('/proc/'+str(before['/grasp_task_node'])).exists(): raise RuntimeError('old task did not exit')
 subprocess.run(['systemd-run','--user','--unit=alicia-plan-bound-observation-task-20260930','--collect','/bin/bash',str(ROOT/'.runtime/start_plan_bound_observation_task_20260930.sh')],check=True,timeout=10)
 deadline=time.monotonic()+25.
 while time.monotonic()<deadline:
  try:
   after=pids()
   if after['/grasp_task_node']!=before['/grasp_task_node']:
    rospy.wait_for_service('/grasp_mode/reset_task',timeout=1.);break
  except Exception: pass
  time.sleep(.2)
 else: raise RuntimeError('new task service unavailable')
 reset=rospy.ServiceProxy('/grasp_mode/reset_task',Trigger)()
 if not reset.success: raise RuntimeError(reset.message)
 r['reset_ack']=json.loads(reset.message)
 r['after_pids']=after
 r['unchanged_other_processes']=all(after[n]==before[n] for n in names if n!='/grasp_task_node')
 assert r['unchanged_other_processes']
 assert rospy.get_param('/grasp_mode/selection')==selection
 s=rospy.wait_for_message('/grasp/state',GraspState,timeout=4)
 assert not s.active
 r['task_active_after']=False
 r['actuation_after']=rospy.wait_for_message('/alicia_d/actuation_status',String,timeout=4).data
 r['result']='task_loaded_idle_ready_for_fresh_selection'
except Exception as exc:
 r['error']=repr(exc);raise
finally:
 r['completed_at_utc']=datetime.datetime.now(datetime.timezone.utc).isoformat()
 (OUT/'plan_bound_observation_task_deployment.json').write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n')
 print(json.dumps(r,ensure_ascii=False),flush=True)
