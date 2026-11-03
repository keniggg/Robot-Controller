"""Reload only idle perception/task software; retain all hardware/control units."""
import os,json,time,subprocess,xmlrpc.client,socket,hashlib
from pathlib import Path
from datetime import datetime,timezone
base=Path('/media/zhuyupei/SanStand');assert base.is_mount()
out=base/'实验记录/Robot-Controller/ros_log'/('exact_tf_wait_load_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'));out.mkdir();(out/'ros').mkdir();os.environ['ROS_LOG_DIR']=str(out/'ros');socket.setdefaulttimeout(4)
import rospy,rosgraph
from alicia_flexible_grasp_supervisor.msg import GraspState
from alicia_flexible_grasp_supervisor.srv import TriggerZero
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import Header
rospy.init_node('load_exact_tf_wait_repair',anonymous=True,disable_signals=True)
master=rosgraph.Master('/load_exact_tf_wait_repair')
def pid(n):return xmlrpc.client.ServerProxy(master.lookupNode(n)).getPid('/load_exact_tf_wait_repair')[2]
protected=['/alicia_d_driver_node','/bessica_d_hw_interface','/motion_gateway','/accepted_robot_state_publisher','/supervisor_camera']
before={n:pid(n) for n in protected};changing={n:pid(n) for n in ['/grasp_task_node','/unknown_tabletop_perception']}
s=rospy.wait_for_message('/grasp/state',GraspState,timeout=4);assert not s.active
for t in ['/alicia_controller/follow_joint_trajectory/status','/hand_controller/follow_joint_trajectory/status']:
 a=rospy.wait_for_message(t,GoalStatusArray,timeout=4);assert not any(v.status in [0,1,6,7] for v in a.status_list)
e=rospy.wait_for_message('/alicia_d/control_reference_epoch',Header,timeout=4).stamp.to_nsec()
files=['src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py','src/alicia_grasp_modes/scripts/mode_aware_grasp_task.py','src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py']
r=dict(before=before,old_consumer_pids=changing,epoch_ns=e,old_selection=rospy.get_param('/grasp_mode/selection'),old_unknown_parameters=rospy.get_param('/unknown_perception',{}),sources={p:hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in files},tests_passed=433,hardware_commands_sent=0,started_ns=time.time_ns())
(out/'before.json').write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n')
stop=rospy.ServiceProxy('/grasp_6d/request_plan',TriggerZero)(False);assert stop.success
r['old_inference_retired']=stop.message
# The user explicitly completed the manual handoff. This GUI flag only chooses
# command ownership; no joint enable/disable or controller service is involved.
r['manual_before']=rospy.get_param('/gui/joint_direct_mode',False)
if r['manual_before']:rospy.set_param('/gui/joint_direct_mode',False)
rospy.set_param('/unknown_perception/tf_wait_sec',.4)
for unit in ['alicia-task-postmove-20260929.service','alicia-unknown-rgb-search-fix-20260929.service']:
 subprocess.run(['systemctl','--user','restart',unit],check=True,timeout=20)
 deadline=time.monotonic()+12
 node='/grasp_task_node' if 'task-postmove' in unit else '/unknown_tabletop_perception'
 while time.monotonic()<deadline:
  try:
   if pid(node)!=changing[node]:break
  except Exception:pass
  time.sleep(.1)
 else:raise RuntimeError('consumer did not return: '+node)
r['after']={n:pid(n) for n in protected};assert r['after']==before
r['new_consumer_pids']={n:pid(n) for n in changing}
r['epoch_after_ns']=rospy.wait_for_message('/alicia_d/control_reference_epoch',Header,timeout=4).stamp.to_nsec();assert r['epoch_after_ns']==e
assert not rospy.wait_for_message('/grasp/state',GraspState,timeout=4).active
r.update(status='loaded_pending_fresh_selection',tf_wait_sec=rospy.get_param('/unknown_perception/tf_wait_sec'),source_max_age_sec=rospy.get_param('/unknown_perception/max_frame_age_sec',.8),completed_ns=time.time_ns())
(out/'result.json').write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n');Path('.runtime/latest_exact_tf_wait_load.json').write_text(json.dumps({'directory':str(out)},ensure_ascii=False)+'\n');print(json.dumps(dict(directory=str(out),**r),ensure_ascii=False,indent=2))
