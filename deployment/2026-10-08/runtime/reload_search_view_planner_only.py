"""Load saved planner fixes while the task is idle; never publish robot commands."""
import datetime
import hashlib
import json
from pathlib import Path
import subprocess
import time
import xmlrpc.client

import rosgraph
import rospy
from alicia_flexible_grasp_supervisor.msg import GraspState

ROOT = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
OUT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/reoriented_observe_return_grasp_20260930T030005Z')
rospy.init_node('reload_saved_search_view_planner_only', anonymous=True, disable_signals=True)
master = rosgraph.Master(rospy.get_name())
names = ('/alicia_d_driver_node', '/motion_gateway', '/grasp_task_node', '/remote_grasp6d_node')
def pids():
    result = {}
    for name in names:
        result[name] = xmlrpc.client.ServerProxy(master.lookupNode(name)).getPid(rospy.get_name())[2]
    return result

state = rospy.wait_for_message('/grasp/state', GraspState, timeout=5)
if state.active:
    raise RuntimeError('Task became active; leave planner unchanged')
manual = rospy.get_param('/gui/joint_direct_mode', False)
selection = rospy.get_param('/grasp_mode/selection')
before = pids()
report = dict(started_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
              user_instruction='finish_fixes_then_pause_until_wakeup',
              task_active_before=False, manual_control_before=manual,
              selection_before=selection, before_pids=before,
              robot_motion_command_sent=False, torque_command_sent=False,
              files={})
for relative in (
        'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py',
        'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py',
        'src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_remote.py',
        'src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_visual.py'):
    report['files'][relative] = hashlib.sha256((ROOT / relative).read_bytes()).hexdigest()
try:
    subprocess.run(['systemctl', '--user', 'restart', 'alicia-contact-precheck-remote-20260930.service'],
                   check=True, timeout=15)
    deadline = time.monotonic() + 25.
    while time.monotonic() < deadline:
        try:
            after = pids()
            if after['/remote_grasp6d_node'] != before['/remote_grasp6d_node']:
                rospy.wait_for_service('/grasp_mode/reset_remote', timeout=1.)
                break
        except Exception:
            pass
        time.sleep(.2)
    else:
        raise RuntimeError('Planner did not return ready within 25 seconds')
    report['after_pids'] = after
    report['unchanged_motion_processes'] = all(after[n] == before[n] for n in names[:-1])
    report['selection_after'] = rospy.get_param('/grasp_mode/selection')
    report['manual_control_after'] = rospy.get_param('/gui/joint_direct_mode', False)
    state = rospy.wait_for_message('/grasp/state', GraspState, timeout=5)
    report['task_active_after'] = bool(state.active)
    assert report['unchanged_motion_processes']
    assert report['selection_after'] == selection
    assert not state.active
    report['result'] = 'planner_loaded_task_idle_waiting_for_user_wakeup'
except Exception as exc:
    report['error'] = repr(exc)
    raise
finally:
    report['completed_at_utc'] = datetime.datetime.now(datetime.timezone.utc).isoformat()
    OUT.joinpath('search_view_planner_deployment.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
