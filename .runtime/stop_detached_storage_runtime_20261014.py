"""Stop the exact idle, broken-log runtime before one verified cold restart."""
import json
import os
from pathlib import Path
import subprocess
import time
import xmlrpc.client
from reload_gen20_compute_gateway_20261014 import Transport, sha
from continuous_grasp_authorization_20261012 import atomic_json
import start_gen18_full_restart_20261013 as start

N=start.LOG_ROOT/'full_latest_gen18_restart_20261014T012216Z'
O=N/'gen20_full_storage_recovery'
baseline=json.loads((N/'gen20_camera_recovery/runtime_baseline.json').read_text())
expected=dict(baseline['after'],**baseline['preserved_after'])
startup=json.loads((N/'startup.json').read_text())
camera=json.loads((N/'gen20_camera_recovery/result.json').read_text())
compute=json.loads((N/'gen19_deployment/result.json').read_text())
units=[compute['unit'],camera['unit'],startup['core']['unit'],startup['simulation']['unit']]
assert len(units)==len(set(units))==4
start.storage_check(O)
start.verify_release(N/'verification/gen20_release_manifest.json')
start.parameter_source_check();start.serial_check()
assert not (O/'stop_runtime.json').exists()
os.environ['ROS_LOG_DIR']=str(O/'stop_observer_ros')
import rospy
import rosgraph
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from actionlib_msgs.msg import GoalStatusArray
from alicia_flexible_grasp_supervisor.msg import GraspState
rospy.init_node('detached_storage_runtime_recovery',anonymous=True,disable_rosout=True)
master=rosgraph.Master(rospy.get_name())
def identity(name):
    code,_,pid=xmlrpc.client.ServerProxy(master.lookupNode(name),transport=Transport()).getPid(rospy.get_name())
    assert code==1
    f=(Path('/proc')/str(pid)/'stat').read_text().rsplit(')',1)[1].split()
    assert f[0]!='Z'
    return dict(pid=pid,start_ticks=f[19],parent_pid=int(f[1]))
def props(unit):
    raw=subprocess.check_output(['systemctl','--user','show',unit,'--property=ActiveState','--property=MainPID','--property=InvocationID','--property=ControlGroup'],text=True)
    return dict(l.split('=',1) for l in raw.splitlines() if '=' in l)
def members(group):
    result=[]
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():continue
        try:
            groups=[l.rsplit(':',1)[-1] for l in (p/'cgroup').read_text().splitlines()]
            if group in groups or any(s.startswith(group+'/') for s in groups):
                f=(p/'stat').read_text().rsplit(')',1)[1].split()
                if f[0]!='Z': result.append(dict(pid=int(p.name),start_ticks=f[19],parent_pid=int(f[1])))
        except (OSError,ProcessLookupError):continue
    return result
assert {n:identity(n) for n in expected}==expected
assert rospy.get_param('/gui/joint_direct_mode') is True
assert not rospy.wait_for_message('/grasp/state',GraspState,timeout=5).active
mode=json.loads(rospy.wait_for_message('/grasp_mode/status',String,timeout=5).data)
assert not any(mode.get(k) for k in ('active','start_reserved','pending'))
for topic in ('/alicia_controller/follow_joint_trajectory/status','/hand_controller/follow_joint_trajectory/status'):
    assert not any(g.status in (0,1,6,7) for g in rospy.wait_for_message(topic,GoalStatusArray,timeout=5).status_list)
q=rospy.wait_for_message('/alicia_d/accepted_joint_states',JointState,timeout=5)
assert q.header.frame_id=='sdk_measured' and 0<=rospy.Time.now().to_nsec()-q.header.stamp.to_nsec()<200_000_000
before={u:props(u) for u in units}
assert all(v['ActiveState']=='active' and int(v['MainPID'])>1 and len(v['InvocationID'])==32 for v in before.values())
assert all(v['ControlGroup'].startswith('/') and v['ControlGroup']!='/' for v in before.values())
assert len({v['ControlGroup'] for v in before.values()})==4
assert int(before[compute['unit']]['MainPID'])==expected['/grasp_task_node']['parent_pid']
assert int(before[startup['core']['unit']]['MainPID'])==expected['/alicia_d_driver_node']['parent_pid']
assert int(before[camera['unit']]['MainPID'])==expected['/supervisor_camera']['pid']
inventory={u:members(v['ControlGroup']) for u,v in before.items()}
assert all(int(v['MainPID']) in {r['pid'] for r in inventory[u]} for u,v in before.items())
assert set(row['pid'] for row in expected.values())<=set(row['pid'] for rs in inventory.values() for row in rs)
result=dict(passed=False,reason='Detached SanStand log descriptors produce EIO before ROS shutdown; functional runtime recovery.',
    user_request='现在机械臂已经对准了目标物，现在进行目标物的抓取任务',
    source_release=str(N/'verification/gen20_release_manifest.json'),script_sha256=sha(__file__),
    before=before,members_before=inventory,node_identities_before=expected,
    measured_before=dict(names=list(q.name),positions=list(q.position),stamp_ns=q.header.stamp.to_nsec()),
    gui_manual=True,task_idle=True,no_active_trajectories=True,
    motion_commands_sent=False,temperature_queried=False,torque_disable_commands_sent=False,
    controller_stop_services_called=False,shutdown_destructor_review='Driver closes serial and joins threads only; GUI closes timers/subscriptions; no torque-off or automatic goal.',
    stop_requested_ns=time.time_ns(),force_killed_units=[])
def save():
    start.storage_check(O);atomic_json(O/'stop_runtime.json',result)
save()
try:
    assert {u:props(u) for u in units}==before
    assert {n:identity(n) for n in expected}==expected
    assert rospy.get_param('/gui/joint_direct_mode') is True
    assert not rospy.wait_for_message('/grasp/state',GraspState,timeout=5).active
    current_mode=json.loads(rospy.wait_for_message('/grasp_mode/status',String,timeout=5).data)
    assert not any(current_mode.get(k) for k in ('active','start_reserved','pending'))
    for topic in ('/alicia_controller/follow_joint_trajectory/status','/hand_controller/follow_joint_trajectory/status'):
        assert not any(g.status in (0,1,6,7) for g in rospy.wait_for_message(topic,GoalStatusArray,timeout=5).status_list)
    subprocess.run(['systemctl','--user','stop','--no-block',*units],check=True)
    end=time.monotonic()+35
    while time.monotonic()<end:
        remaining={u:members(v['ControlGroup']) for u,v in before.items()}
        if not any(remaining.values()):break
        time.sleep(.2)
    for u,rows in remaining.items():
        if not rows:continue
        now=props(u)
        assert now['InvocationID']==before[u]['InvocationID'] and now['ControlGroup']==before[u]['ControlGroup']
        result['force_killed_units'].append(dict(unit=u,remaining=rows,reason='Normal service stop did not exit broken-log process group within 35 seconds.'))
        save()
        subprocess.run(['systemctl','--user','kill','--kill-who=all','--signal=SIGKILL',u],check=True)
    end=time.monotonic()+10
    while time.monotonic()<end:
        if not any(members(v['ControlGroup']) for v in before.values()):break
        time.sleep(.2)
    assert not any(members(v['ControlGroup']) for v in before.values())
    start.idle_process_check()
    result.update(passed=True,stopped_ns=time.time_ns(),old_units_empty=True,ports_11311_8002_closed=True)
    save();print(json.dumps(dict(passed=True,receipt=str(O/'stop_runtime.json'),force_killed_units=len(result['force_killed_units'])),ensure_ascii=False))
except BaseException as exc:
    result['error']=repr(exc);save();raise
