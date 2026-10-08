"""Load tested speed/ranking fixes only after the operator's alignment confirmation.

Reloads idle planning/gateway nodes, never the driver; does not start a grasp.
"""
import hashlib,json,os,pathlib,subprocess,time,xmlrpc.client
import rospy,rosgraph,yaml
from alicia_flexible_grasp_supervisor.msg import GraspState

ROOT=pathlib.Path(__file__).resolve().parents[1]
OUT=pathlib.Path(json.loads((ROOT/'.runtime/current_resumed_grasp_session.json').read_text())['directory'])
rospy.init_node('load_unknown_pregrasp_speed_fix',anonymous=True)
assert not rospy.wait_for_message('/grasp/state',GraspState,timeout=5).active,'Task is active'
master=rosgraph.Master(rospy.get_name())
def pid_of(name):
    uri=master.lookupNode(name)
    return uri,xmlrpc.client.ServerProxy(uri).getPid(rospy.get_name())[2]
driver_before=pid_of('/alicia_d_driver_node')[1]
profile=yaml.safe_load((ROOT/'src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml').read_text())['robot']['unknown_pregrasp_timing']
assert profile=={'enabled':True,'joint_velocity_rad_s':.04}
record=dict(started_unix=time.time(),driver_pid_before=driver_before,
    prior_speed_profile=rospy.get_param('/robot/unknown_pregrasp_timing',None),new_speed_profile=profile,
    commands_sent=False,grasp_started=False,nodes=[])
(OUT/'unknown_speed_deployment_started.json').write_text(json.dumps(record,indent=2)+'\n')
rospy.set_param('/robot/unknown_pregrasp_timing',profile)
for name,script,service,logfile in (
    ('/motion_gateway',str(ROOT/'src/alicia_flexible_grasp_supervisor/scripts/motion_gateway_node.py'),'/supervisor/check_pose_sequence_strict','gateway_speed_runtime.log'),
    ('/remote_grasp6d_node','/home/zhuyupei/alicia_wa_full/.ros_log/unknown_grasp_20260923/remote_with_simulation_audit.py','/grasp_mode/reset_remote','remote_speed_runtime.log')):
    assert not rospy.wait_for_message('/grasp/state',GraspState,timeout=4).active
    old_uri,old_pid=pid_of(name)
    try:xmlrpc.client.ServerProxy(old_uri).shutdown(rospy.get_name(),'load tested unknown pregrasp speed and motion ranking')
    except (ConnectionError,OSError):pass
    for _ in range(200):
        stat=pathlib.Path('/proc')/str(old_pid)/'stat'
        if not stat.exists() or stat.read_text().split()[2]=='Z':break
        time.sleep(.05)
    else:raise RuntimeError('Old node still running: '+name)
    with (OUT/logfile).open('ab',buffering=0) as log:
        process=subprocess.Popen(['/usr/bin/python3','-u',script,'__name:='+name.lstrip('/')],
            cwd=str(ROOT),stdout=log,stderr=subprocess.STDOUT,env=os.environ.copy(),start_new_session=True)
    for _ in range(150):
        try:
            new_uri,new_pid=pid_of(name)
            if new_pid==process.pid and new_uri!=old_uri:break
        except (rosgraph.MasterError,ConnectionError,OSError):pass
        if process.poll() is not None:raise RuntimeError('Replacement exited: '+name)
        time.sleep(.1)
    else:raise RuntimeError('Replacement did not register: '+name)
    rospy.wait_for_service(service,timeout=10)
    record['nodes'].append(dict(name=name,old_pid=old_pid,new_pid=new_pid,log=str(OUT/logfile)))
    print('READY',name,new_pid,flush=True)
record['driver_pid_after']=pid_of('/alicia_d_driver_node')[1]
assert record['driver_pid_after']==driver_before
record['policy']={k:rospy.get_param(k) for k in (
    '/mujoco_digital_twin/unknown_lift_gate_enabled','/grasp/unknown_lift_after_close_enabled',
    '/mujoco_digital_twin/unknown_object_mass_bound','/robot/unknown_pregrasp_timing')}
assert record['policy']['/mujoco_digital_twin/unknown_lift_gate_enabled'] is False
assert record['policy']['/grasp/unknown_lift_after_close_enabled'] is True
record['source_sha256']={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in (
    ROOT/'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py',
    ROOT/'src/alicia_flexible_grasp_supervisor/scripts/motion_gateway_node.py',
    ROOT/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/pregrasp_timing.py',
    ROOT/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/pregrasp_feedforward.py',
    ROOT/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/moveit_planner.py')}
record.update(finished_unix=time.time(),deployed=True,driver_restarted=False,torque_command_sent=False)
(OUT/'unknown_speed_deployment.json').write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n')
print('SPEED_FIX_READY_NO_MOTION',flush=True)
