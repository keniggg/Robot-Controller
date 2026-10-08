import os,json,time,subprocess,hashlib,xmlrpc.client
from pathlib import Path
import rospy,rosgraph
from alicia_flexible_grasp_supervisor.msg import GraspState
from std_msgs.msg import Header
root=Path.cwd()
out=Path(json.loads((root/'.runtime/current_grasp_repair_resume.json').read_text())['directory'])
assert os.path.ismount('/media/zhuyupei/SanStand')
rospy.init_node('load_regression_verified_grasp_repairs',anonymous=True)
master=rosgraph.Master(rospy.get_name())
def pid(name):
 try:
  uri=master.lookupNode(name)
  return xmlrpc.client.ServerProxy(uri).getPid(rospy.get_name())[2]
 except (ConnectionRefusedError,rosgraph.MasterError):return None
if pid('/grasp_task_node') is not None:
 state=rospy.wait_for_message('/grasp/state',GraspState,timeout=5)
 if state.active:raise RuntimeError('Active task cannot be reloaded')
driver_pid=pid('/alicia_d_driver_node')
epoch=rospy.wait_for_message('/alicia_d/control_reference_epoch',Header,timeout=4).stamp.to_nsec()
files=['src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py','src/alicia_grasp_modes/src/alicia_grasp_modes/tabletop.py']
record=dict(started=time.time(),driver_pid_before=driver_pid,epoch_before_ns=epoch,disable_command_sent=False,motion_command_sent=False,nodes={})
record['source_sha256']={f:hashlib.sha256((root/f).read_bytes()).hexdigest() for f in files}
commands=[('/grasp_task_node',['/usr/bin/python3','-u',str(root/'src/alicia_grasp_modes/scripts/mode_aware_grasp_task.py'),'__name:=grasp_task_node','/grasp/start:=/grasp/internal_start']),('/unknown_tabletop_perception',['/usr/bin/python3','-u',str(root/'src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py'),'__name:=unknown_tabletop_perception']),('/remote_grasp6d_node',['/usr/bin/python3','-u','/home/zhuyupei/alicia_wa_full/.ros_log/unknown_grasp_20260923/remote_with_simulation_audit.py','__name:=remote_grasp6d_node'])]
for name,command in commands:
 uri=master.lookupNode(name); old=pid(name)
 print('RELOAD',name,old,flush=True)
 if old is not None:
  xmlrpc.client.ServerProxy(uri).shutdown(rospy.get_name(),'load offline-regression-verified grasp fixes and preserve request audit')
 for _ in range(100):
  try:
   if pid(name) is None:break
  except Exception:break
  time.sleep(.05)
 else:raise RuntimeError(name+' did not unregister')
 unit='alicia-repaired-'+name.strip('/').replace('_','-')+'-20260928'
 log=out/(name.strip('/')+'.log')
 # A transient unit keeps the reloaded process supervised outside this helper.
 result=subprocess.run(['systemd-run','--user','--unit='+unit,'--collect','--property=KillMode=control-group','--property=WorkingDirectory='+str(root),'--property=StandardOutput=append:'+str(log),'--property=StandardError=append:'+str(log)]+['--setenv='+k+'='+v for k,v in os.environ.items() if k in ('PATH','PYTHONPATH','LD_LIBRARY_PATH','CMAKE_PREFIX_PATH','ROS_PACKAGE_PATH','ROS_MASTER_URI','ROS_ROOT','ROS_DISTRO','ROS_VERSION','ROS_PYTHON_VERSION','ALICIA_EXPERIMENT_STORAGE_BASE','ALICIA_SESSION_LOG_DIR','ALICIA_UNKNOWN_LOG_DIR','GRASP_RECORD_ROOT')]+['--setenv=ROS_LOG_DIR='+str(out/'ros'),'--setenv=PYTHONUNBUFFERED=1','--setenv=OPENBLAS_NUM_THREADS=1']+command,text=True,capture_output=True)
 if result.returncode:raise RuntimeError(result.stderr)
 for _ in range(160):
  try:
   new=pid(name)
   if new is not None and new!=old:break
  except Exception:pass
  time.sleep(.1)
 else:raise RuntimeError(name+' new node unavailable')
 record['nodes'][name]=dict(old_pid=old,new_pid=new,unit=unit,log=str(log))
 print('READY',name,new,flush=True)
record['driver_pid_after']=pid('/alicia_d_driver_node')
record['epoch_after_ns']=rospy.wait_for_message('/alicia_d/control_reference_epoch',Header,timeout=4).stamp.to_nsec()
assert record['driver_pid_after']==driver_pid and record['epoch_after_ns']==epoch
record['done']=time.time()
record['policy']={'unknown_mass_bound':rospy.get_param('/mujoco_digital_twin/unknown_object_mass_bound'),'lift_simulation_gate':rospy.get_param('/mujoco_digital_twin/unknown_lift_gate_enabled'),'physical_lift':rospy.get_param('/grasp/unknown_lift_after_close_enabled'),'handeye':rospy.get_param('/handeye')}
(out/'repair_deployment.json').write_text(json.dumps(record,ensure_ascii=False,indent=2))
print(json.dumps(record,ensure_ascii=False),flush=True)
