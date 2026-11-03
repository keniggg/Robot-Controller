"""Load tested diagnostic-only planner while retaining all motion owners."""
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
OUT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_ros_resume_20261004T002930Z')
EXPECTED = {
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/multiview_surface.py': 'bbf3e3a863e9a5c104e682ebe33820a0e84f42c27425ac1f8d34a4ee41327e2c',
    'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py': '51dfb4287cc5e78b11d2ece0840e7d2cace38417fb792e663906ead65e6e38f4',
}
PRESERVE = ('/alicia_d_driver_node', '/supervisor_camera', '/motion_gateway',
            '/grasp_task_node', '/bessica_d_hw_interface')
REMOTE = '/remote_grasp6d_node'

def node(name):
    uri = rosgraph.Master('/load_support_audit').lookupNode(name)
    code, message, pid = xmlrpc.client.ServerProxy(uri).getPid('/load_support_audit')
    if code != 1:
        raise RuntimeError(message)
    return dict(uri=uri, pid=pid)

def verify_sources():
    for name, expected in EXPECTED.items():
        if hashlib.sha256((ROOT/name).read_bytes()).hexdigest() != expected:
            raise RuntimeError('tested source changed: '+name)

if not Path('/media/zhuyupei/SanStand').is_mount():
    raise RuntimeError('SanStand unavailable')
verify_sources()
rospy.init_node('load_support_audit', anonymous=True, disable_signals=True)
if rospy.wait_for_message('/grasp/state', GraspState, timeout=4).active:
    raise RuntimeError('active task retains current planner')
before = {name:node(name) for name in PRESERVE+(REMOTE,)}
report = dict(recorded_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
              before=before, source_sha256=EXPECTED, regression_passed=197,
              diagnostic_only=True, motion_commands_sent=False,
              torque_commands_sent=False, loaded=False)
(OUT/'support_audit_load_started.json').write_text(json.dumps(report, indent=2)+'\n')
xmlrpc.client.ServerProxy(before[REMOTE]['uri']).shutdown(
    '/load_support_audit', 'load tested diagnostic-only support consistency audit')
deadline = time.monotonic()+10
while time.monotonic() < deadline and Path('/proc', str(before[REMOTE]['pid'])).exists():
    time.sleep(.2)
if Path('/proc', str(before[REMOTE]['pid'])).exists():
    raise RuntimeError('previous planner has not exited')
subprocess.run(['systemd-run','--user','--collect','--unit=alicia-support-audit-remote-20261004',
    '--property=PartOf=alicia-ros-live-20260928.service','--property=Restart=no',
    '--property=WorkingDirectory='+str(ROOT),
    '--property=StandardOutput=append:'+str(OUT/'support_audit_remote.log'),
    '--property=StandardError=append:'+str(OUT/'support_audit_remote.log'),
    '/bin/bash',str(ROOT/'.runtime/start_support_audit_remote_20261004.sh')], check=True, timeout=15)
deadline = time.monotonic()+25
while time.monotonic() < deadline:
    try:
        after = {name:node(name) for name in PRESERVE+(REMOTE,)}
        if after[REMOTE]['pid'] != before[REMOTE]['pid']:
            rospy.wait_for_service('/grasp_mode/reset_remote', timeout=2)
            break
    except Exception:
        pass
    time.sleep(.3)
else:
    raise RuntimeError('replacement planner unavailable')
for name in PRESERVE:
    if after[name]['pid'] != before[name]['pid']:
        raise RuntimeError('preserved owner changed: '+name)
verify_sources()
report.update(after=after, loaded=True, preserved_owners_unchanged=True,
    motion_enabled=rospy.wait_for_message('/alicia_d/motion_enabled',Bool,timeout=4).data,
    feedback_ready=rospy.wait_for_message('/alicia_d/feedback_ready',Bool,timeout=4).data)
(OUT/'support_audit_deployment.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
