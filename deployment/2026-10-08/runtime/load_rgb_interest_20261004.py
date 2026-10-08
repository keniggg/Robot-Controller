"""Reload only tested unknown perception; no motion/controller/torque APIs."""
import datetime
import hashlib
import json
from pathlib import Path
import subprocess
import time
import xmlrpc.client

import rosgraph
import rospy
from std_msgs.msg import Bool, String
from alicia_flexible_grasp_supervisor.msg import GraspState

ROOT = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
OUT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_ros_resume_20261004T002930Z')
RUN_STAMP = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
TARGET = '/unknown_tabletop_perception'
PRESERVE = ('/alicia_d_driver_node', '/supervisor_camera', '/motion_gateway',
            '/grasp_task_node', '/bessica_d_hw_interface', '/remote_grasp6d_node')
LOAD_SOURCES = ('src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py',) + tuple(
    'src/alicia_grasp_modes/src/alicia_grasp_modes/'+name+'.py' for name in
    ('rgb_observation_region', 'rgb_observation_volume', 'rgb_observation_source'))


def node(name):
    uri = rosgraph.Master('/load_rgb_interest').lookupNode(name)
    code, message, pid = xmlrpc.client.ServerProxy(uri).getPid('/load_rgb_interest')
    if code != 1:
        raise RuntimeError(message)
    return dict(uri=uri, pid=pid)


def verify_sources(manifest):
    if manifest['test_exit_code'] != 0 or not manifest['test_result']:
        raise RuntimeError('missing successful regression')
    for name in LOAD_SOURCES:
        expected = manifest['source_sha256'][name]
        if hashlib.sha256((ROOT/name).read_bytes()).hexdigest() != expected:
            raise RuntimeError('tested source changed: '+name)


if not Path('/media/zhuyupei/SanStand').is_mount():
    raise RuntimeError('SanStand unavailable')
manifest = json.loads((ROOT/'.runtime/rgb_observation_validation_20261004.json').read_text())
verify_sources(manifest)
rospy.init_node('load_rgb_interest', anonymous=True, disable_signals=True)
if rospy.wait_for_message('/grasp/state', GraspState, timeout=4).active:
    raise RuntimeError('active grasp task retains current perception')
before = {name: node(name) for name in PRESERVE+(TARGET,)}
report = dict(recorded_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
    before=before, source_sha256={name:manifest['source_sha256'][name] for name in LOAD_SOURCES},
    regression=manifest['test_result'],
    loaded=False, motion_commands_sent=False, torque_commands_sent=False,
    scope='RGB observation interest only; original detected and contact gates unchanged')
(OUT/('rgb_interest_load_started_'+RUN_STAMP+'.json')).write_text(json.dumps(report, indent=2)+'\n')
xmlrpc.client.ServerProxy(before[TARGET]['uri']).shutdown(
    '/load_rgb_interest', 'load tested independent RGB observation interest')
deadline = time.monotonic()+10
while time.monotonic() < deadline and Path('/proc', str(before[TARGET]['pid'])).exists():
    time.sleep(.2)
if Path('/proc', str(before[TARGET]['pid'])).exists():
    raise RuntimeError('old perception did not exit')
subprocess.run(['systemd-run', '--user', '--collect', '--unit=alicia-rgb-interest-perception-'+RUN_STAMP.lower(),
    '--property=PartOf=alicia-ros-live-20260928.service', '--property=Restart=no',
    '--property=WorkingDirectory='+str(ROOT),
    '--property=StandardOutput=append:'+str(OUT/'rgb_interest_perception.log'),
    '--property=StandardError=append:'+str(OUT/'rgb_interest_perception.log'),
    '/bin/bash', str(ROOT/'.runtime/start_rgb_interest_perception_20261004.sh')], check=True, timeout=15)
deadline = time.monotonic()+25
while time.monotonic() < deadline:
    try:
        after = {name: node(name) for name in PRESERVE+(TARGET,)}
        status = json.loads(rospy.wait_for_message('/perception/unknown/detector_status', String, timeout=2).data)
        if after[TARGET]['pid'] != before[TARGET]['pid'] and 'observation_interest' in status:
            break
    except Exception:
        pass
    time.sleep(.3)
else:
    raise RuntimeError('updated unknown perception not available')
for name in PRESERVE:
    if after[name]['pid'] != before[name]['pid']:
        raise RuntimeError('preserved node changed: '+name)
verify_sources(manifest)
report.update(after=after, loaded=True, preserved_nodes_unchanged=True,
    motion_enabled=rospy.wait_for_message('/alicia_d/motion_enabled', Bool, timeout=4).data,
    feedback_ready=rospy.wait_for_message('/alicia_d/feedback_ready', Bool, timeout=4).data,
    first_status=status)
(OUT/('rgb_interest_deployment_'+RUN_STAMP+'.json')).write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps({key:value for key,value in report.items() if key!='first_status'}, indent=2))
