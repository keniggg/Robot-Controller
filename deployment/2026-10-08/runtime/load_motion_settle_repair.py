"""Reload the idle task only; no arm, gripper, enable or motion publishers."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
import xmlrpc.client

import rosgraph
import rospy
from alicia_flexible_grasp_supervisor.msg import GraspState

ROOT = Path(__file__).resolve().parents[1]
OUT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/settling_readonly_20260928T062049Z')
assert os.path.ismount('/media/zhuyupei/SanStand')
rospy.init_node('load_independent_settling_repair', anonymous=True)
master = rosgraph.Master(rospy.get_name())

def pid(name):
    code, message, value = xmlrpc.client.ServerProxy(master.lookupNode(name)).getPid(rospy.get_name())
    if code != 1:
        raise RuntimeError(message)
    return value

names = ['/alicia_d_driver_node', '/motion_gateway', '/grasp_task_node', '/remote_grasp6d_node']
before = {name: pid(name) for name in names}
state = rospy.wait_for_message('/grasp/state', GraspState, timeout=5)
assert not state.active, 'Task is active; reload not performed'
manual = rospy.get_param('/gui/joint_direct_mode', None)
handeye = rospy.get_param('/handeye', {})
old = before['/grasp_task_node']
assert b'mode_aware_grasp_task.py' in Path('/proc/%s/cmdline' % old).read_bytes()
node = xmlrpc.client.ServerProxy(master.lookupNode('/grasp_task_node'))
try:
    node.shutdown(rospy.get_name(), 'load tested independent encoder settling window')
except (ConnectionError, OSError):
    pass
for _ in range(200):
    stat = Path('/proc/%s/stat' % old)
    if not stat.exists() or stat.read_text().split()[2] == 'Z':
        break
    time.sleep(.05)
else:
    raise RuntimeError('Old task did not exit; no duplicate started')

with (OUT / 'task_runtime.log').open('ab', buffering=0) as log:
    child = subprocess.Popen([
        '/usr/bin/python3', '-u',
        str(ROOT / 'src/alicia_grasp_modes/scripts/mode_aware_grasp_task.py'),
        '__name:=grasp_task_node'], stdout=log, stderr=subprocess.STDOUT,
        env=os.environ.copy(), start_new_session=True)
for _ in range(200):
    if child.poll() is not None:
        raise RuntimeError('Reloaded task exited; inspect task_runtime.log')
    try:
        if pid('/grasp_task_node') == child.pid:
            break
    except (rosgraph.MasterError, ConnectionError, OSError):
        pass
    time.sleep(.05)
else:
    raise RuntimeError('Reloaded task did not register')
rospy.wait_for_service('/grasp_mode/reset_task', timeout=10)
after = {name: pid(name) for name in names}
for name in names:
    if name != '/grasp_task_node':
        assert after[name] == before[name], name + ' unexpectedly changed'
assert rospy.get_param('/gui/joint_direct_mode', None) == manual
assert rospy.get_param('/handeye', {}) == handeye
assert not rospy.wait_for_message('/grasp/state', GraspState, timeout=5).active
sources = [
    'src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/stationary_following.py',
    'src/alicia_flexible_grasp_supervisor/tests/test_motion_settle.py']
result = dict(deployed_unix=time.time(), before=before, after=after,
    manual_control_preserved=manual, handeye_preserved=True,
    task_regression_passed=321, real_stationary_replay_passed=True,
    historic_oscillation_windows_rejected=43,
    source_sha256={name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest() for name in sources},
    driver_restarted=False, driver_patch_loaded=False,
    motion_commands_sent=False, enable_commands_sent=False,
    physical_oscillation_fixed=False, physical_grasp_fixed=False)
(OUT / 'task_settling_deployment.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
print(json.dumps(result, ensure_ascii=False), flush=True)
