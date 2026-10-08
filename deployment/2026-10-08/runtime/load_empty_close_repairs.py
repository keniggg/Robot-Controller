import hashlib,json,os,pathlib,subprocess,time,xmlrpc.client
import rospy,rosgraph
from alicia_flexible_grasp_supervisor.msg import GraspState
out=pathlib.Path(json.load(open('.runtime/current_resumed_grasp_session.json'))['directory'])
rospy.init_node('load_empty_close_repairs',anonymous=True)
assert not rospy.wait_for_message('/grasp/state',GraspState,timeout=5).active, 'Task active'
master=rosgraph.Master(rospy.get_name())
def pid(name): return xmlrpc.client.ServerProxy(master.lookupNode(name)).getPid(rospy.get_name())[2]
def shutdown(name):
    old=pid(name)
    try:xmlrpc.client.ServerProxy(master.lookupNode(name)).shutdown(rospy.get_name(),'load tested empty-close and lift-start repairs')
    except (ConnectionError,OSError):pass
    return old
before={n:pid(n) for n in ['/alicia_d_driver_node','/motion_gateway','/grasp_task_node','/remote_grasp6d_node']}
manual=rospy.get_param('/gui/joint_direct_mode',None)
old=shutdown('/motion_gateway')
# The existing roslaunch owns this node's respawn. Never start a duplicate.
for _ in range(400):
    try:
        new=pid('/motion_gateway')
        if new!=old:break
    except (rosgraph.MasterError,ConnectionError,OSError):pass
    time.sleep(.05)
else:raise RuntimeError('Motion gateway did not respawn')
rospy.wait_for_service('/supervisor/move_to_pose_linear',timeout=10)
print('GATEWAY_READY',old,new,flush=True)
old=shutdown('/grasp_task_node')
for _ in range(200):
    stat=pathlib.Path('/proc')/str(old)/'stat'
    if not stat.exists() or stat.read_text().split()[2]=='Z':break
    time.sleep(.05)
else:raise RuntimeError('Task has not exited')
log=out/'task_empty_close_repair_runtime.log'
with log.open('ab',buffering=0) as handle:
    child=subprocess.Popen(['/usr/bin/python3','-u',str(pathlib.Path('src/alicia_grasp_modes/scripts/mode_aware_grasp_task.py').resolve()),'__name:=grasp_task_node'],stdout=handle,stderr=subprocess.STDOUT,env=os.environ.copy(),start_new_session=True)
for _ in range(200):
    if child.poll() is not None:raise RuntimeError('New task exited')
    try:
        if pid('/grasp_task_node')==child.pid:break
    except (rosgraph.MasterError,ConnectionError,OSError):pass
    time.sleep(.05)
else:raise RuntimeError('Task did not register')
rospy.wait_for_service('/grasp_mode/reset_task',timeout=10)
print('TASK_READY',old,child.pid,flush=True)
after={n:pid(n) for n in before}
assert after['/alicia_d_driver_node']==before['/alicia_d_driver_node']
assert rospy.get_param('/gui/joint_direct_mode',None)==manual
sources=['src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/moveit_planner.py','src/alicia_grasp_modes/scripts/mode_aware_grasp_task.py','src/alicia_grasp_modes/src/alicia_grasp_modes/support_observation.py']
r=dict(deployed_unix=time.time(),before=before,after=after,manual_control_preserved=manual,regressions={'lift_current_start':132,'support_contradiction':112},source_sha256={s:hashlib.sha256(pathlib.Path(s).read_bytes()).hexdigest() for s in sources},driver_restarted=False,torque_command_sent=False,grasp_requested=False,physical_grasp_fixed=False,remaining_issue='recorded support-plane reconstruction shifts 12-13 mm with pose; requires wider task-pose validation')
(out/'empty_close_repairs_deployment.json').write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n')
print('REPAIRS_DEPLOYED',json.dumps(r,ensure_ascii=False),flush=True)
