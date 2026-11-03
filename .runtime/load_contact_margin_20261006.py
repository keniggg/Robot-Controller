#!/usr/bin/env python3
"""Review by default; --execute reloads idle gateway via its owner, then the task.

No target selection, mode handoff, plan request, robot command, grasp, torque,
temperature access or controller restart. A failed deployment is only recorded;
it never triggers an automatic retry or grasp. Manual GUI mode may stay true,
but any state change during the bounded operation invalidates the receipt.
"""
import argparse
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import subprocess
import threading
import time
import xmlrpc.client

W = Path(__file__).resolve().parents[1]
MOUNT = Path('/media/zhuyupei/SanStand')
EXT = MOUNT/'实验记录/Robot-Controller'
ARCHIVE = EXT/'ros_log'
TARGET = '/grasp_task_node'
PRESERVE = ('/remote_grasp6d_node', '/supervisor_camera', '/alicia_d_driver_node',
    '/bessica_d_hw_interface', '/move_group',
    '/robot_state_publisher', '/handeye_transform', '/unknown_tabletop_perception',
    '/grasp_mode_router', '/compliant_gripper_controller')
REQUIRED_PRODUCTION = (
    'src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/contact_execution.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/contact_height_margin.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/pregrasp_compensation.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/pregrasp_gateway.py')
ARTIFACTS = ('.runtime/load_contact_margin_20261006.py',
    '.runtime/start_support_source_task_20261006.sh',
    '.runtime/support_source_task_20261006.launch')
NAMES = ['Joint%d' % i for i in range(1, 7)]


def require(condition, reason):
    if not condition:
        raise RuntimeError(reason)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text())


def verified_manifest(path, root=W):
    manifest = read_json(path)
    require(manifest.get('tests_passed') is True and manifest.get('reviewed') is True,
            'reviewed passing validation manifest required')
    files = manifest.get('files')
    production = manifest.get('production_files')
    require(isinstance(files, dict) and bool(files), 'nonempty files SHA manifest required')
    require(isinstance(production, list) and bool(production)
            and all(isinstance(p, str) for p in production)
            and len(set(production)) == len(production), 'explicit changed production_files required')
    require(set(REQUIRED_PRODUCTION).issubset(production), 'support fix production manifest incomplete')
    require(set(production).issubset(files), 'changed production source missing file SHA')
    for relative, expected in files.items():
        require(isinstance(relative, str) and not Path(relative).is_absolute(), 'manifest paths must be relative')
        target = (root/relative).resolve()
        require(root.resolve() in target.parents, 'manifest source outside workspace')
        require(isinstance(expected, str) and re.fullmatch('[0-9a-f]{64}', expected), 'invalid source SHA')
        require(sha(target) == expected, 'tested source changed: '+relative)
    return manifest


def process_identity(pid, proc=Path('/proc')):
    require(type(pid) is int and pid > 0, 'invalid node process ID')
    directory = proc/str(pid)
    raw = (directory/'stat').read_text()
    fields = raw.rsplit(')', 1)[1].split()
    return dict(pid=pid, ppid=int(fields[1]), start_ticks=fields[19], state=fields[0],
        command=(directory/'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace'),
        boot_id=(proc/'sys/kernel/random/boot_id').read_text().strip())


def same_process(a, b):
    return all(a.get(k) == b.get(k) for k in ('pid', 'start_ticks', 'boot_id'))


class BoundedTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 3.
        return connection


def proxy(uri):
    require(isinstance(uri, str) and uri.startswith('http://'), 'unexpected XMLRPC URI')
    return xmlrpc.client.ServerProxy(uri, transport=BoundedTransport())


def rpc_value(response, label):
    require(isinstance(response, (list, tuple)) and len(response) == 3
            and response[0] == 1, 'XMLRPC failed: '+label+' '+repr(response))
    return response[2]


def load_guard():
    spec = importlib.util.spec_from_file_location('support_reload_alignment_guard',
        W/'tools/diagnostics/prepare_rgb_acquisition_retry.py')
    guard = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(guard)
    return guard


def read_receipt(path, external=EXT):
    path = path.resolve()
    require(external.resolve() in path.parents, 'alignment receipt must be on SanStand')
    receipt = read_json(path)
    event_path = Path(receipt['event_file']).resolve()
    require(external.resolve() in event_path.parents, 'alignment event must be on SanStand')
    require(sha(event_path) == receipt.get('event_file_sha256'), 'alignment event SHA mismatch')
    event = read_json(event_path)
    require(receipt.get('kind') == 'explicit_user_target_alignment'
            and receipt.get('target_aligned') is True and event.get('target_aligned') is True,
            'explicit user alignment required')
    source = receipt.get('confirmation_source', {})
    require(source.get('kind') == 'user_message' and bool(source.get('message_id'))
            and isinstance(source.get('text'), str) and source['text'].strip()
            and isinstance(receipt.get('confirmation_id'), str) and receipt['confirmation_id'],
            'actual confirmation provenance required')
    for key in ('confirmation_source', 'confirmed_ns', 'bound_feedback'):
        require(receipt.get(key) == event.get(key), 'alignment event/receipt mismatch: '+key)
    context_digest = hashlib.sha256(json.dumps(event['source_context'],
        sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()
    require(context_digest == receipt.get('source_context_sha256'), 'alignment source context SHA mismatch')
    require(event['source_context'].get('selection') == receipt.get('selection')
            and int(event['source_context'].get('epoch_ns', 0)) == int(receipt['epoch_ns']),
            'alignment context selection/epoch mismatch')
    feedback = receipt['bound_feedback']
    require(feedback.get('frame_id') == 'sdk_measured'
        and len(feedback.get('positions', [])) == 6
        and all(type(v) in (float, int) and math.isfinite(v) for v in feedback['positions']),
        'six real measured alignment joints required')
    require(0 < int(receipt['epoch_ns']) <= int(receipt['confirmed_ns'])
            <= int(feedback['stamp_ns']), 'alignment timestamp order invalid')
    return receipt


def bounded_call(function, timeout):
    """Bound even a stalled ROS service response; daemon makes no retries."""
    result, done = {}, threading.Event()
    def run():
        try:
            result['value'] = function()
        except BaseException as exc:
            result['error'] = exc
        finally:
            done.set()
    threading.Thread(target=run, daemon=True).start()
    require(done.wait(timeout), 'bounded operation timed out; no retry issued')
    if 'error' in result:
        raise result['error']
    return result['value']


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--validation-manifest', type=Path, required=True)
    parser.add_argument('--alignment-receipt', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    frozen = verified_manifest(args.validation_manifest)
    receipt = read_receipt(args.alignment_receipt)
    frozen_manifest_sha = sha(args.validation_manifest)
    receipt_sha = sha(args.alignment_receipt)
    artifacts = {name: sha(W/name) for name in ARTIFACTS}
    out = args.output.resolve()
    require(MOUNT.is_mount() and ARCHIVE.resolve() in out.parents,
            'mounted SanStand ros_log output required; no fallback')
    require(not out.exists(), 'output must be a new archive directory')
    review = dict(execute=args.execute, target=TARGET, gateway_reload="original_roslaunch_respawn_only", preserved_nodes=list(PRESERVE),
        validation_manifest=str(args.validation_manifest.resolve()),
        validation_manifest_sha256=frozen_manifest_sha,
        production_files=frozen['production_files'], alignment_receipt=str(args.alignment_receipt.resolve()),
        alignment_receipt_sha256=receipt_sha, artifacts_sha256=artifacts,
        output=str(out), manual_gui_allowed_if_unchanged=True, no_automatic_grasp=True)
    if not args.execute:
        print(json.dumps(review, ensure_ascii=False, indent=2))
        return 0
    out.mkdir(parents=False, exist_ok=False)
    (out/'ros').mkdir()
    require(out.stat().st_dev == MOUNT.stat().st_dev, 'output escaped SanStand')
    # This is deliberately before rospy import/init and before roslaunch.
    os.environ['ROS_LOG_DIR'] = str(out/'ros')
    os.environ['PYTHONDONTWRITEBYTECODE'] = '1'
    report = dict(review, started_utc=datetime.now(timezone.utc).isoformat(),
        deployment_passed=False, robot_commands_sent=False, torque_commands_sent=False,
        selection_changed=False, confirmation_reissued=False, grasp_started=False)
    (out/'deployment_started.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    try:
        import rospy
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool, Header, String
        from std_srvs.srv import Trigger
        from alicia_flexible_grasp_supervisor.msg import GraspState
        guard = load_guard()
    except BaseException as exc:
        report.update(error=repr(exc), completed_utc=datetime.now(timezone.utc).isoformat())
        (out/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        raise
    lock = threading.RLock()
    latest, faults, subscriptions = {}, [], []
    pose_monitor = guard.AlignmentPoseMonitor(receipt)
    baseline_sdk = baseline_manual = None
    def fault(reason):
        with lock:
            if reason not in faults:
                faults.append(reason)
    def remember(key, value):
        with lock:
            latest[key] = (value, time.monotonic())
    def accepted_cb(message):
        try:
            require(message.header.frame_id == 'sdk_measured', 'wrong accepted source frame')
            values = dict(zip(message.name, message.position))
            q = [values[n] for n in NAMES]
            stamp = message.header.stamp.to_nsec()
            with lock:
                pose_monitor.observe(stamp, q)
                pose_monitor.require_stationary()
                latest['accepted'] = (dict(stamp_ns=stamp, positions=q), time.monotonic())
        except Exception as exc:
            fault('accepted feedback invalidated: '+str(exc))
    def sdk_cb(message):
        try:
            require(message.header.frame_id == 'sdk_transmitted', 'wrong SDK source frame')
            values = dict(zip(message.name, message.position))
            q = [values[n] for n in NAMES]
            require(all(math.isfinite(v) for v in q), 'invalid SDK target')
            with lock:
                if baseline_sdk is not None and q != baseline_sdk:
                    fault('SDK target changed during task reload')
                remember('sdk', q)
        except Exception as exc:
            fault('SDK context invalid: '+str(exc))
    def manual_cb(message):
        with lock:
            if baseline_manual is not None and message.data != baseline_manual:
                fault('GUI manual state changed during task reload')
            remember('manual', bool(message.data))
    def task_cb(message):
        if message.active:
            fault('task became active')
        remember('task_active', bool(message.active))
    def epoch_cb(message):
        if message.frame_id != 'sdk_reference_epoch' or message.stamp.to_nsec() != int(receipt['epoch_ns']):
            fault('control reference epoch changed')
        remember('epoch', message.stamp.to_nsec())
    def router_cb(message):
        try:
            value = json.loads(message.data)
            require(value.get('active') is False and value.get('start_reserved') is False,
                    'router became active or reserved')
            require(guard.parse_selection(value) == guard.parse_selection(receipt['selection']),
                    'router selection changed')
            remember('router', value)
        except Exception as exc:
            fault('router context invalid: '+str(exc))
    try:
        rospy.init_node('load_support_source_task_only', anonymous=True, disable_signals=True)
        caller = rospy.get_name()
        master_uri = os.environ.get('ROS_MASTER_URI', 'http://localhost:11311')
        master = proxy(master_uri)
        def parameter(key):
            return rpc_value(master.getParam(caller, key), 'getParam '+key)
        def node_identity(name):
            uri = rpc_value(master.lookupNode(caller, name), 'lookup '+name)
            pid = rpc_value(proxy(uri).getPid(caller), 'getPid '+name)
            identity = process_identity(pid)
            require(identity['state'] != 'Z', 'node is a zombie: '+name)
            identity['uri'] = uri
            return identity
        baseline_manual = parameter('/gui/joint_direct_mode')
        require(type(baseline_manual) is bool, 'GUI state must be boolean')
        selection = guard.parse_selection(parameter('/grasp_mode/selection'))
        require(selection == guard.parse_selection(receipt['selection']), 'receipt selection no longer current')
        before = {name: node_identity(name) for name in (TARGET,'/motion_gateway')+PRESERVE}
        require('mode_aware_grasp_task.py' in before[TARGET]['command'], 'unexpected task process entry')
        subscriptions = [rospy.Subscriber('/alicia_d/accepted_joint_states', JointState, accepted_cb, queue_size=100),
            rospy.Subscriber('/alicia_d/sdk_command', JointState, sdk_cb, queue_size=100),
            rospy.Subscriber('/gui/joint_direct_mode', Bool, manual_cb, queue_size=20),
            rospy.Subscriber('/grasp/state', GraspState, task_cb, queue_size=20),
            rospy.Subscriber('/alicia_d/control_reference_epoch', Header, epoch_cb, queue_size=20),
            rospy.Subscriber('/grasp_mode/status', String, router_cb, queue_size=20)]
        deadline = time.monotonic()+5.
        while not all(k in latest for k in ('accepted','sdk','manual','task_active','epoch','router')):
            require(time.monotonic()<deadline and not faults, 'maintenance source unavailable: '+str(faults))
            time.sleep(.05)
        with lock:
            baseline_sdk = list(latest['sdk'][0])
            accepted_before = deepcopy(latest['accepted'][0])
        report.update(before_identities=before, selection=selection, manual_before=baseline_manual,
            sdk_before=baseline_sdk, accepted_before=accepted_before, epoch_before=int(receipt['epoch_ns']))

        def files_unchanged():
            require(sha(args.validation_manifest) == frozen_manifest_sha, 'validation manifest changed')
            require(verified_manifest(args.validation_manifest) == frozen, 'validation changed')
            require(sha(args.alignment_receipt) == receipt_sha, 'alignment receipt changed')
            read_receipt(args.alignment_receipt)
            require(all(sha(W/p)==v for p,v in artifacts.items()), 'loader artifact changed')

        def context_unchanged(check_processes=True):
            with lock:
                require(not faults, 'maintenance context changed: '+str(faults))
                pose_monitor.require_stationary()
                for key in ('accepted','sdk'):
                    require(time.monotonic()-latest[key][1] <= .5, 'stale maintenance source: '+key)
                sample = latest['accepted'][0]
                age = (rospy.Time.now().to_nsec()-sample['stamp_ns'])/1e9
                require(0 <= age <= .5, 'accepted feedback timestamp stale')
                require(latest['task_active'][0] is False, 'task active')
                require(latest['manual'][0] == baseline_manual, 'manual topic changed')
            require(parameter('/gui/joint_direct_mode') == baseline_manual, 'manual parameter changed')
            require(guard.parse_selection(parameter('/grasp_mode/selection')) == selection, 'selection parameter changed')
            context = guard.retry_source_context(selection, latest['epoch'][0], parameter('/camera'),
                parameter('/handeye'), parameter('/robot_description'))
            require(guard.digest(context) == receipt['source_context_sha256'], 'camera/handeye/model source context changed')
            if check_processes:
                for name in PRESERVE:
                    require(same_process(before[name], node_identity(name)), 'preserved node identity changed: '+name)

        files_unchanged(); context_unchanged()
        gateway = '/motion_gateway'
        require('motion_gateway_node.py' in before[gateway]['command'], 'unexpected gateway entry')
        require(same_process(before[gateway], node_identity(gateway)), 'gateway owner changed')
        gateway_parent = process_identity(before[gateway]['ppid'])
        require('roslaunch' in gateway_parent['command'], 'gateway parent is not roslaunch')
        report['gateway_original_parent'] = gateway_parent
        # Original bringup owns respawn. Never start a duplicate gateway here.
        report['gateway_shutdown_reply'] = proxy(before[gateway]['uri']).shutdown(
            caller, 'load reviewed final contact margin while task is idle')
        require(report['gateway_shutdown_reply'][0] == 1, 'gateway shutdown not acknowledged')
        deadline = time.monotonic()+12.
        while True:
            try:
                prior = process_identity(before[gateway]['pid'])
                require(same_process(prior,before[gateway]), 'gateway old PID reused')
                if prior['state'] == 'Z':
                    break
            except FileNotFoundError:
                break
            require(time.monotonic()<deadline, 'old gateway did not exit')
            context_unchanged(check_processes=False)
            time.sleep(.1)
        deadline = time.monotonic()+30.
        new_gateway = None
        while time.monotonic()<deadline:
            context_unchanged(check_processes=False)
            try:
                candidate = node_identity(gateway)
                if not same_process(before[gateway],candidate):
                    require('motion_gateway_node.py' in candidate['command'], 'unexpected new gateway')
                    require(candidate['ppid'] == gateway_parent['pid'] and same_process(
                        gateway_parent, process_identity(candidate['ppid'])), 'gateway respawn owner changed')
                    new_gateway = candidate
                    break
            except (OSError, xmlrpc.client.Error, RuntimeError):
                pass
            time.sleep(.1)
        require(new_gateway is not None, 'original gateway owner did not respawn; no duplicate launched')
        rospy.wait_for_service('/supervisor/compensate_grasp_pose', timeout=15.)
        report['gateway_replacement'] = new_gateway
        files_unchanged(); context_unchanged()
        require(same_process(new_gateway, node_identity(gateway)), 'gateway changed while awaiting service')
        require(same_process(gateway_parent, process_identity(new_gateway['ppid'])), 'gateway parent changed')
        require(same_process(before[TARGET], node_identity(TARGET)), 'old task identity changed')
        report['shutdown_requested_at_utc'] = datetime.now(timezone.utc).isoformat()
        reply = proxy(before[TARGET]['uri']).shutdown(caller,
            'load reviewed source-qualified support checks in idle task only')
        require(isinstance(reply, (list,tuple)) and reply[0] == 1, 'task shutdown not acknowledged')
        report['shutdown_reply'] = reply
        deadline = time.monotonic()+12.
        while True:
            try:
                current = process_identity(before[TARGET]['pid'])
                require(same_process(current, before[TARGET]), 'old task PID reused before exit proof')
                if current['state'] == 'Z':
                    break
            except FileNotFoundError:
                break
            require(time.monotonic()<deadline, 'old task did not exit; no second task launched')
            context_unchanged(check_processes=False)
            time.sleep(.1)
        files_unchanged(); context_unchanged()
        unit = 'alicia-contact-margin-task-'+datetime.now(timezone.utc).strftime('%Y%m%dt%H%M%Sz').lower()
        command = ['systemd-run','--user','--collect','--unit='+unit,
            '--property=Type=exec','--property=Restart=no','--property=WorkingDirectory='+str(W),
            '--property=StandardOutput=append:'+str(out/'node.log'),
            '--property=StandardError=append:'+str(out/'node.log'),
            '--setenv=ROS_LOG_DIR='+str(out/'ros'),'--setenv=ROS_MASTER_URI='+master_uri,
            '/bin/bash',str(W/'.runtime/start_support_source_task_20261006.sh')]
        report.update(unit=unit, command=command)
        subprocess.run(command, check=True, timeout=15.)
        deadline = time.monotonic()+30.
        replacement = None
        while time.monotonic()<deadline:
            context_unchanged(check_processes=False)
            try:
                candidate = node_identity(TARGET)
                if not same_process(before[TARGET], candidate):
                    require('mode_aware_grasp_task.py' in candidate['command'], 'unexpected replacement entry')
                    replacement = candidate
                    break
            except (OSError, xmlrpc.client.Error, RuntimeError):
                pass
            time.sleep(.1)
        require(replacement is not None, 'replacement task unavailable; no automatic retry')
        rospy.wait_for_service('/grasp_mode/reset_task', timeout=20.)
        context_unchanged()
        reset = bounded_call(lambda: rospy.ServiceProxy('/grasp_mode/reset_task', Trigger)(), 8.)
        report['reset_ack'] = dict(success=bool(reset.success),message=reset.message)
        require(reset.success is True, 'new task reset failed: '+reset.message)
        ack = json.loads(reset.message)
        require(ack.get('ready') is True and guard.parse_selection(ack) == selection,
                'new task did not acknowledge existing selection')
        # Observe a short fresh-feedback interval after initialization; this is
        # not an arrival test or permission to execute any retained plan.
        deadline = time.monotonic()+1.
        while time.monotonic()<deadline:
            context_unchanged(check_processes=False)
            time.sleep(.1)
        files_unchanged(); context_unchanged()
        after = {name: node_identity(name) for name in (TARGET,'/motion_gateway')+PRESERVE}
        require(same_process(replacement,after[TARGET]), 'replacement task changed during acknowledgment')
        require(same_process(new_gateway,after['/motion_gateway']), 'replacement gateway changed during acknowledgment')
        require(all(same_process(before[n],after[n]) for n in PRESERVE), 'unexpected preserved process replacement')
        with lock:
            report.update(after_identities=after, accepted_after=deepcopy(latest['accepted'][0]),
                sdk_after=deepcopy(latest['sdk'][0]),manual_after=latest['manual'][0],
                max_arm_delta_rad=max(abs(a-b) for a,b in zip(accepted_before['positions'],latest['accepted'][0]['positions'])))
        report['deployment_passed'] = True
    except BaseException as exc:
        report['error'] = repr(exc)
        raise
    finally:
        for subscription in subscriptions:
            subscription.unregister()
        final_error = None
        with lock:
            try:
                pose_monitor.require_stationary()
                require(not faults, 'maintenance context changed at completion: '+str(faults))
            except Exception as exc:
                final_error = exc
                report['deployment_passed'] = False
                report['completion_error'] = repr(exc)
            report['context_faults'] = list(faults)
        report['completed_utc'] = datetime.now(timezone.utc).isoformat()
        (out/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
        print(json.dumps(report,ensure_ascii=False,indent=2),flush=True)
        if final_error is not None:
            raise final_error
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
