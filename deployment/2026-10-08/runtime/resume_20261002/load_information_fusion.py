"""Load the tested planner only; preserve all hardware and motion owners."""
import datetime
import hashlib
import json
from pathlib import Path
import subprocess
import time
import xmlrpc.client
import rosgraph
import rospy
from std_msgs.msg import Bool
from alicia_flexible_grasp_supervisor.msg import GraspState

ROOT = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
OUT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_ros_latest_start_20261002')
SOURCES = [
    'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py',
    'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/observation_view_utility.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/silhouette_observation_planning.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/multiview_surface.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/fused_scene_geometry.py',
]
PRESERVE = ('/alicia_d_driver_node', '/supervisor_camera', '/motion_gateway', '/grasp_task_node')
REMOTE = '/remote_grasp6d_node'

def hashes():
    return {name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in SOURCES}

def node(name):
    uri = rosgraph.Master('/information_fusion_reload').lookupNode(name)
    code, message, pid = xmlrpc.client.ServerProxy(uri).getPid('/information_fusion_reload')
    if code != 1:
        raise RuntimeError(message)
    return dict(uri=uri, pid=pid)

def main():
    if not Path('/media/zhuyupei/SanStand').is_mount():
        raise RuntimeError('SanStand unavailable')
    verified = json.loads((OUT/'information_fusion_validation.json').read_text())
    if hashes() != verified['source_sha256']:
        raise RuntimeError('source differs from regression-validated version')
    rospy.init_node('information_fusion_reload', anonymous=True, disable_signals=True)
    state = rospy.wait_for_message('/grasp/state', GraspState, timeout=4.)
    if state.active:
        raise RuntimeError('active task must retain its running planner')
    before = {name:node(name) for name in PRESERVE+(REMOTE,)}
    report = dict(recorded_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        source_sha256=hashes(), before=before, hardware_commands_sent=False,
        joint_disable_commands_sent=False, real_motion_verified=False,
        support_reference_failure_retained=True)
    (OUT/'information_fusion_load_started.json').write_text(json.dumps(report,indent=2)+'\n')
    subprocess.run(['systemctl','--user','restart',
        'alicia-information-observation-remote-20261002.service'], check=True, timeout=15.)
    deadline = time.monotonic()+25.
    while time.monotonic()<deadline:
        try:
            after = {name:node(name) for name in PRESERVE+(REMOTE,)}
            if after[REMOTE]['pid'] != before[REMOTE]['pid']:
                break
        except Exception:
            pass
        time.sleep(.3)
    else:
        raise RuntimeError('new planner PID unavailable')
    for name in PRESERVE:
        if after[name]['pid'] != before[name]['pid']:
            raise RuntimeError('preserved node PID changed: '+name)
    if hashes() != report['source_sha256']:
        raise RuntimeError('source changed during load')
    report.update(after=after, preserved_nodes_unchanged=True,
        motion_enabled=rospy.wait_for_message('/alicia_d/motion_enabled',Bool,timeout=4.).data,
        feedback_ready=rospy.wait_for_message('/alicia_d/feedback_ready',Bool,timeout=4.).data,
        loaded=True)
    (OUT/'information_fusion_deployment.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({key:value for key,value in report.items() if key!='source_sha256'},indent=2))

if __name__ == '__main__':
    main()
