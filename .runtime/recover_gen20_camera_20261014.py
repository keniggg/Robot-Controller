"""Restore only a confirmed absent camera and attest the unchanged remaining nodes."""
import ast
import json
import os
from pathlib import Path
import subprocess
import time
import xmlrpc.client
from reload_gen20_compute_gateway_20261014 import Transport, sha

W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
N = M/'实验记录/Robot-Controller/ros_log/full_latest_gen18_restart_20261014T012216Z'
out = N/'gen20_camera_recovery'
assert M.is_mount() and not os.statvfs(M).f_flag & os.ST_RDONLY
assert out.parent.stat().st_dev == M.stat().st_dev
out.mkdir(exist_ok=False)
os.environ['ROS_LOG_DIR'] = str(out/'observer_ros')
import rospy
import rosgraph
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatusArray
from alicia_flexible_grasp_supervisor.msg import GraspState
rospy.init_node('camera_only_recovery', anonymous=True, disable_rosout=True)
master = rosgraph.Master(rospy.get_name())
prior_path = N/'gen19_deployment/result.json'
prior = json.loads(prior_path.read_text())
expected = dict(prior['after'], **prior['preserved_after'])
camera = '/supervisor_camera'
def identity(name):
    rpc = xmlrpc.client.ServerProxy(master.lookupNode(name), transport=Transport())
    code, _, pid = rpc.getPid(rospy.get_name())
    assert code == 1
    fields = (Path('/proc')/str(pid)/'stat').read_text().rsplit(')',1)[1].split()
    assert fields[0] != 'Z'
    return dict(pid=pid,start_ticks=fields[19],parent_pid=int(fields[1]))
def idle():
    assert rospy.get_param('/gui/joint_direct_mode') is True
    assert not rospy.wait_for_message('/grasp/state',GraspState,timeout=4).active
    mode=json.loads(rospy.wait_for_message('/grasp_mode/status',String,timeout=4).data)
    assert not any(mode.get(k) for k in ('active','start_reserved','pending'))
    for topic in ('/alicia_controller/follow_joint_trajectory/status','/hand_controller/follow_joint_trajectory/status'):
        assert not any(g.status in (0,1,6,7) for g in rospy.wait_for_message(topic,GoalStatusArray,timeout=4).status_list)
def write(name,value):
    assert M.is_mount() and out.stat().st_dev==M.stat().st_dev
    (out/name).write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')
result=dict(passed=False,motion_commands_sent=False,temperature_queried=False,
            previous_deployment=str(prior_path),previous_deployment_sha256=sha(prior_path),
            script_sha256=sha(__file__),camera_before=expected[camera])
try:
    idle()
    others={n:identity(n) for n in expected if n!=camera}
    assert all(row==expected[n] for n,row in others.items())
    assert not (Path('/proc')/str(expected[camera]['pid'])).exists()
    try:
        live=identity(camera)
    except (ConnectionRefusedError, OSError, rosgraph.MasterError):
        live=None
    assert live is None, 'camera already running'
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit(): continue
        try: argv=(proc/'cmdline').read_bytes().split(b'\0')
        except OSError: continue
        assert not any(a.endswith(b'/camera_node.py') for a in argv), 'another camera process exists'
    log=N/'core_ros/fd1d800e-c76e-11f1-8bba-d31cd6c63278/roslaunch-ubuntu-11379.log'
    lines=[l for l in log.read_text().splitlines() if 'process[supervisor_camera-8]: args[' in l]
    assert len(lines)==1
    argv=ast.literal_eval(lines[0].split('args',1)[1])[0]
    assert len(argv)==10 and argv[-1]=='__name:=supervisor_camera'
    assert Path(argv[0]).resolve()==W/'tools/with_local_realsense_sdk.sh'
    assert Path(argv[1]).resolve()==W/'devel/lib/alicia_flexible_grasp_supervisor/camera_node.py'
    for name in ('src/alicia_flexible_grasp_supervisor/scripts/camera_node.py','tools/with_local_realsense_sdk.sh'):
        if name in prior['source_sha256']: assert sha(W/name)==prior['source_sha256'][name]
    unit='alicia-camera-recovery-'+str(time.time_ns())
    command=['systemd-run','--user','--collect','--unit='+unit,'--property=Type=exec','--property=Restart=no',
        '--property=WorkingDirectory='+str(W),'--property=StandardOutput=append:'+str(out/'camera.log'),
        '--property=StandardError=append:'+str(out/'camera.log'),'--setenv=ROS_LOG_DIR='+str(out/'camera_ros')]
    for key in ('PATH','PYTHONPATH','LD_LIBRARY_PATH','CMAKE_PREFIX_PATH','ROS_MASTER_URI','ROS_PACKAGE_PATH','ROS_ROOT','ROS_DISTRO','ROS_VERSION','ROS_PYTHON_VERSION'):
        if key in os.environ: command.append('--setenv='+key+'='+os.environ[key])
    command+=argv
    result.update(preserved_before=others,unit=unit,command=command,launch_attempted=True)
    write('result.json',result)
    subprocess.run(command,check=True)
    end=time.monotonic()+35
    while time.monotonic()<end:
        try:
            current=identity(camera)
            break
        except (OSError,rosgraph.MasterError): time.sleep(.2)
    else: raise RuntimeError('camera did not register')
    env=dict(os.environ,ROS_LOG_DIR=str(out/'check_ros'),PYTHONDONTWRITEBYTECODE='1')
    check=out/'idle_rgbd'
    with (out/'verification.log').open('x') as stream:
        subprocess.run(['/usr/bin/python3',str(W/'.runtime/verify_gen18_idle_gui_camera_20261013.py'),'--output',str(check)],env=env,stdout=stream,stderr=subprocess.STDOUT,check=True)
    idle()
    assert {n:identity(n) for n in others}==others
    assert identity(camera)==current
    result.update(passed=True,camera_after=current,preserved_after=others,recorded_ns=time.time_ns(),verification=str(check/'result.json'),verification_sha256=sha(check/'result.json'))
    write('result.json',result)
    baseline=dict(prior)
    baseline['preserved_after']=dict(prior['preserved_after'],**{camera:current})
    baseline.update(previous_deployment=str(prior_path),previous_deployment_sha256=sha(prior_path),
        deployment_kind='gen19_runtime_with_camera_only_recovery',camera_recovery=str(out/'result.json'),
        camera_recovery_sha256=sha(out/'result.json'),runtime_source_unchanged=True,
        disk_contains_unloaded_gen20=True,recorded_ns=time.time_ns())
    write('runtime_baseline.json',baseline)
    print(json.dumps(dict(passed=True,camera=current,baseline=str(out/'runtime_baseline.json')),ensure_ascii=False))
except BaseException as exc:
    result['error']=repr(exc);write('result.json',result);raise
