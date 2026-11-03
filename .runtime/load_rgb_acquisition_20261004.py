"""Load tested perception/gateway adapters while preserving hardware nodes."""
from datetime import datetime, timezone
import argparse
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
STAMP = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
PRESERVE = ('/alicia_d_driver_node','/bessica_d_hw_interface','/supervisor_camera',
            '/grasp_task_node','/remote_grasp6d_node')
TARGETS = ('/unknown_tabletop_perception','/motion_gateway')


def node(name):
    uri=rosgraph.Master('/load_rgb_acquisition').lookupNode(name)
    code,message,pid=xmlrpc.client.ServerProxy(uri).getPid('/load_rgb_acquisition')
    if code != 1:
        raise RuntimeError(message)
    return dict(uri=uri,pid=pid)


def verify(manifest):
    if manifest['test_exit_code'] != 0 or not manifest['test_result']:
        raise RuntimeError('missing successful regression')
    for name, expected in manifest['source_sha256'].items():
        if hashlib.sha256((ROOT/name).read_bytes()).hexdigest() != expected:
            raise RuntimeError('tested source changed: '+name)


if not Path('/media/zhuyupei/SanStand').is_mount():
    raise RuntimeError('SanStand unavailable; no fallback')
parser = argparse.ArgumentParser()
parser.add_argument('--resume-report', type=Path)
parser.add_argument('--carry-forward-result', type=Path)
parser.add_argument('--unreserved-transition', type=Path, action='append', default=[])
parser.add_argument('--maintenance-receipt', type=Path)
parser.add_argument('--aligned-observation-record', type=Path)
parser.add_argument('--aligned-source-stamp-ns', type=int)
scope = parser.add_mutually_exclusive_group()
scope.add_argument('--perception-only', action='store_true')
scope.add_argument('--gateway-only', action='store_true')
args = parser.parse_args()
if args.unreserved_transition and args.carry_forward_result is None:
    raise RuntimeError('unreserved transitions require original aligned source')
if args.maintenance_receipt is not None and args.carry_forward_result is None:
    raise RuntimeError('maintenance receipt requires original aligned source')
retry_fields = (args.carry_forward_result, args.aligned_observation_record, args.aligned_source_stamp_ns)
if any(value is not None for value in retry_fields):
    if (not all(value is not None for value in retry_fields) or not args.perception_only
            or args.resume_report is not None):
        raise RuntimeError('bound retry requires all source fields and perception-only fresh deployment')
if args.perception_only:
    PRESERVE = PRESERVE+('/motion_gateway',)
    TARGETS = ('/unknown_tabletop_perception',)
elif args.gateway_only:
    PRESERVE = PRESERVE+('/unknown_tabletop_perception',)
    TARGETS = ('/motion_gateway',)
manifest=json.loads((ROOT/'.runtime/rgb_acquisition_validation_20261004.json').read_text())
verify(manifest)
rospy.init_node('load_rgb_acquisition',anonymous=True,disable_signals=True)
if rospy.wait_for_message('/grasp/state',GraspState,timeout=4).active:
    raise RuntimeError('active grasp task owns current gateway')
ledger=rospy.get_param('/rgb_acquisition/ledger',{})
if ledger.get('phase') in ('reserved','executing'):
    raise RuntimeError('active observation owns current gateway')
if args.resume_report is not None:
    path = args.resume_report
    if path.parent != OUT:
        raise RuntimeError('resume report must be original external deployment record')
    report = json.loads(path.read_text())
    if report['loaded'] or report['source_sha256'] != manifest['source_sha256']:
        raise RuntimeError('resume source or deployment changed')
    before = report['before']
    for name in PRESERVE:
        if node(name)['pid'] != before[name]['pid']:
            raise RuntimeError('preserved process changed before resume: '+name)
    report['resumed_at_utc'] = datetime.now(timezone.utc).isoformat()
else:
    before={name:node(name) for name in PRESERVE+TARGETS}
    report=dict(started_at_utc=datetime.now(timezone.utc).isoformat(),before=before,
        source_sha256=manifest['source_sha256'],regression=manifest['test_result'],
        loaded=False,motion_commands_sent=False,torque_commands_sent=False)
    path=OUT/('rgb_acquisition_deployment_'+STAMP+'.json')
path.write_text(json.dumps(report,indent=2)+'\n')
# Gateway's original roslaunch respawns it. Perception's replacement service
# intentionally does not respawn; recreate only that service explicitly.
for name in TARGETS:
    if not Path('/proc',str(before[name]['pid'])).exists():
        continue
    try:
        xmlrpc.client.ServerProxy(before[name]['uri']).shutdown('/load_rgb_acquisition',
            'load tested independent RGB acquisition and identity handoff')
    except OSError as exc:
        # rospy may close its XMLRPC socket before the shutdown reply arrives.
        # The process-exit check below remains authoritative; a broken reply
        # alone neither proves exit nor starts another instance.
        report.setdefault('shutdown_reply_errors', []).append(dict(node=name, error=str(exc)))
deadline=time.monotonic()+12
while time.monotonic()<deadline and any(Path('/proc',str(before[n]['pid'])).exists() for n in TARGETS):
    time.sleep(.2)
if any(Path('/proc',str(before[n]['pid'])).exists() for n in TARGETS):
    raise RuntimeError('old adapter did not exit; hardware left running')
if args.carry_forward_result is not None:
    # The old producer is permanently lost. Retire that generation while it is
    # stopped, so restarting cannot erase loss and publish under its old identity.
    # The guarded tool reads raw camera/SDK sources independently of perception;
    # it calls only the ordinary selection service, never a motion interface.
    command = ['/usr/bin/python3', str(ROOT/'tools/diagnostics/prepare_rgb_acquisition_retry.py'),
        '--carry-forward-result', str(args.carry_forward_result),
        '--aligned-observation-record', str(args.aligned_observation_record),
        '--aligned-source-stamp-ns', str(args.aligned_source_stamp_ns),
        '--reason', 'Load tested thin-surface and bounded depth-gap fixes after the original '
            'user alignment; verify original intent on fresh RGB and retire unreserved generation.',
        '--wait-sec', '8', '--apply']
    for transition in args.unreserved_transition:
        command.extend(('--unreserved-transition', str(transition)))
    if args.maintenance_receipt is not None:
        command.extend(('--maintenance-receipt', str(args.maintenance_receipt)))
    retry_log = OUT/('rgb_acquisition_deployment_retry_'+STAMP+'.log')
    with retry_log.open('w') as stream:
        result = subprocess.run(command, cwd=str(ROOT), stdout=stream, stderr=subprocess.STDOUT,
                                timeout=25)
    report['prestart_retry'] = dict(command=command, returncode=result.returncode,
        terminal_log=str(retry_log), selection=rospy.get_param('/grasp_mode/selection'))
    path.write_text(json.dumps(report,indent=2)+'\n')
    if result.returncode != 0:
        raise RuntimeError('fresh alignment retry failed; lost producer remains stopped, hardware untouched: '+str(retry_log))
    verify(manifest)
if '/unknown_tabletop_perception' in TARGETS:
    subprocess.run(['systemd-run','--user','--collect','--unit=alicia-rgb-handoff-perception-'+STAMP.lower(),
        '--property=PartOf=alicia-ros-live-20260928.service','--property=Restart=no',
        '--property=WorkingDirectory='+str(ROOT),
        '--property=StandardOutput=append:'+str(OUT/'rgb_handoff_perception.log'),
        '--property=StandardError=append:'+str(OUT/'rgb_handoff_perception.log'),
        '/bin/bash',str(ROOT/'.runtime/start_rgb_interest_perception_20261004.sh')],check=True,timeout=15)
rospy.wait_for_service('/supervisor/plan_and_execute_rgb_acquisition_pose_strict',timeout=30)
deadline=time.monotonic()+20
while time.monotonic()<deadline:
    publishers,subscribers,_=rosgraph.Master('/load_rgb_acquisition').getSystemState()
    if ('/perception/rgb_acquisition/reference_status' in dict(publishers)
            and '/supervisor/rgb_acquisition/reference_request' in dict(subscribers)):
        break
    time.sleep(.2)
else:
    raise RuntimeError('perception reference handshake unavailable')
after={name:node(name) for name in PRESERVE+TARGETS}
for name in PRESERVE:
    if before[name]['pid'] != after[name]['pid']:
        raise RuntimeError('preserved node changed: '+name)
for name in TARGETS:
    if before[name]['pid'] == after[name]['pid']:
        raise RuntimeError('adapter did not reload: '+name)
verify(manifest)
report.update(loaded=True,after=after,preserved_nodes_unchanged=True,
    motion_enabled=rospy.wait_for_message('/alicia_d/motion_enabled',Bool,timeout=4).data,
    feedback_ready=rospy.wait_for_message('/alicia_d/feedback_ready',Bool,timeout=4).data,
    finished_at_utc=datetime.now(timezone.utc).isoformat())
path.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2),flush=True)
