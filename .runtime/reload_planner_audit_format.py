import pathlib,json,os,time,subprocess,xmlrpc.client,hashlib
import rospy,rosgraph
from alicia_flexible_grasp_supervisor.msg import GraspState
out=pathlib.Path(json.load(open('.runtime/current_resumed_grasp_session.json'))['directory']);rospy.init_node('load_height_audit_format_fix',anonymous=True)
assert not rospy.wait_for_message('/grasp/state',GraspState,timeout=4).active
master=rosgraph.Master(rospy.get_name());uri=master.lookupNode('/remote_grasp6d_node');proxy=xmlrpc.client.ServerProxy(uri);old=proxy.getPid(rospy.get_name())[2]
try:proxy.shutdown(rospy.get_name(),'load JSON-stable height audit evidence')
except (ConnectionError,OSError):pass
for _ in range(200):
 stat=pathlib.Path('/proc')/str(old)/'stat'
 if not stat.exists() or stat.read_text().split()[2]=='Z':break
 time.sleep(.05)
else:raise RuntimeError('Planner process still active')
log=(out/'remote_audit_format_runtime.log').open('ab',buffering=0)
p=subprocess.Popen(['/usr/bin/python3','-u','/home/zhuyupei/alicia_wa_full/.ros_log/unknown_grasp_20260923/remote_with_simulation_audit.py','__name:=remote_grasp6d_node'],stdout=log,stderr=subprocess.STDOUT,env=os.environ.copy(),start_new_session=True)
for _ in range(100):
 try:
  if master.lookupNode('/remote_grasp6d_node')!=uri:break
 except rosgraph.MasterError:pass
 time.sleep(.05)
rospy.wait_for_service('/grasp_mode/reset_remote',timeout=15)
record=dict(old_pid=old,new_pid=p.pid,driver_restarted=False,torque_command_sent=False,changed_only='height audit named clearance pairs normalized to JSON lists',source_sha256=hashlib.sha256(pathlib.Path('src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/contact_height_margin.py').read_bytes()).hexdigest(),policy={k:rospy.get_param(k) for k in ('/mujoco_digital_twin/unknown_lift_gate_enabled','/grasp/unknown_lift_after_close_enabled','/mujoco_digital_twin/unknown_object_mass_bound')})
(out/'height_audit_format_deployment.json').write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n');print('PLANNER_READY',p.pid,flush=True)
