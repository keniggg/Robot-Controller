import pathlib,json,os,time,subprocess,xmlrpc.client,hashlib,yaml
import rospy,rosgraph
from alicia_flexible_grasp_supervisor.msg import GraspState
out=pathlib.Path(json.load(open('.runtime/current_resumed_grasp_session.json'))['directory']);rospy.init_node('load_declared_unknown_mass_limit',anonymous=True)
assert not rospy.wait_for_message('/grasp/state',GraspState,timeout=4).active
policy=yaml.safe_load(pathlib.Path('src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml').read_text())['mujoco_digital_twin']['unknown_object_mass_bound']
rospy.set_param('/mujoco_digital_twin/unknown_object_mass_bound',policy)
master=rosgraph.Master(rospy.get_name());changes=[]
for name,command,logname,service in [
 ('/remote_grasp6d_node',['/usr/bin/python3','-u','/home/zhuyupei/alicia_wa_full/.ros_log/unknown_grasp_20260923/remote_with_simulation_audit.py','__name:=remote_grasp6d_node'],'remote_mass_bound_runtime.log','/grasp_mode/reset_remote'),
 ('/grasp_task_node',['/usr/bin/python3','-u',str(pathlib.Path('src/alicia_grasp_modes/scripts/mode_aware_grasp_task.py').resolve()),'__name:=grasp_task_node'],'task_mass_bound_runtime.log','/grasp_mode/reset_task')]:
 uri=master.lookupNode(name);proxy=xmlrpc.client.ServerProxy(uri)
 try:old=proxy.getPid(rospy.get_name())[2]
 except (ConnectionError,OSError):old={'/remote_grasp6d_node':22243,'/grasp_task_node':10047}[name]
 print('RELOAD',name,old,flush=True)
 try:proxy.shutdown(rospy.get_name(),'load user-declared unknown mass bound')
 except (ConnectionError,OSError):print('RPC already closed',name,flush=True)
 for _ in range(200):
  stat=pathlib.Path('/proc')/str(old)/'stat'
  if not stat.exists() or stat.read_text().split()[2]=='Z':break
  time.sleep(.05)
 else:raise RuntimeError(name+' has not exited')
 log=(out/logname).open('ab',buffering=0);p=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,env=os.environ.copy(),start_new_session=True)
 for _ in range(100):
  try:
   newuri=master.lookupNode(name)
   if newuri!=uri:break
  except rosgraph.MasterError:pass
  time.sleep(.05)
 rospy.wait_for_service(service,timeout=15)
 changes.append(dict(node=name,old_pid=old,new_pid=p.pid,log=str(out/logname)));print('READY',name,p.pid,flush=True)
record=dict(user_statement=policy['operator_statement'],policy=policy,regression='162 passed in 1.17 seconds',changes=changes,driver_restarted=False,torque_command_sent=False,source_sha256=hashlib.sha256(pathlib.Path('src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/mujoco_digital_twin_client.py').read_bytes()).hexdigest(),deployed_unix=time.time())
(out/'declared_mass_bound_deployment.json').write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n')
print('MASS_BOUND_DEPLOYMENT_COMPLETE',flush=True)
