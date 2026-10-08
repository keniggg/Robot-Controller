import hashlib,json,os,pathlib,subprocess,time,xmlrpc.client
import rospy,rosgraph
from alicia_flexible_grasp_supervisor.msg import GraspState
out=pathlib.Path(json.load(open('.runtime/current_resumed_grasp_session.json'))['directory'])
rospy.init_node('load_checked_duration_policy_compatibility',anonymous=True)
assert not rospy.wait_for_message('/grasp/state',GraspState,timeout=5).active, 'Task is active'
master=rosgraph.Master(rospy.get_name())
name='/grasp_task_node'; uri=master.lookupNode(name); proxy=xmlrpc.client.ServerProxy(uri)
old=proxy.getPid(rospy.get_name())[2]
print('TASK_RELOAD_START',old,flush=True)
try: proxy.shutdown(rospy.get_name(),'load checked hardware duration audit compatibility')
except (ConnectionError,OSError): pass
for _ in range(200):
    stat=pathlib.Path('/proc')/str(old)/'stat'
    if not stat.exists() or stat.read_text().split()[2]=='Z': break
    time.sleep(.05)
else: raise RuntimeError('Task process has not exited')
log=out/'task_speed_contract_runtime.log'
with log.open('ab',buffering=0) as handle:
    child=subprocess.Popen(['/usr/bin/python3','-u',str(pathlib.Path('src/alicia_grasp_modes/scripts/mode_aware_grasp_task.py').resolve()),'__name:=grasp_task_node'],stdout=handle,stderr=subprocess.STDOUT,env=os.environ.copy(),start_new_session=True)
for _ in range(200):
    if child.poll() is not None: raise RuntimeError('New task exited')
    try:
        active_pid=xmlrpc.client.ServerProxy(master.lookupNode(name)).getPid(rospy.get_name())[2]
        if active_pid==child.pid: break
    except (rosgraph.MasterError,ConnectionError,OSError): pass
    time.sleep(.05)
else: raise RuntimeError('New task failed to register')
rospy.wait_for_service('/grasp_mode/reset_task',timeout=15)
source=pathlib.Path('src/alicia_grasp_modes/src/alicia_grasp_modes/runner_contract.py')
record=dict(deployed_unix=time.time(),old_task_pid=old,new_task_pid=child.pid,log=str(log),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),regressions=['105 passed: runner and duration selection','52 passed: observe_return task'],driver_restarted=False,torque_command_sent=False,physical_gates_changed=False)
(out/'runner_duration_policy_deployment.json').write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n')
print('TASK_RELOAD_COMPLETE',json.dumps(record),flush=True)
