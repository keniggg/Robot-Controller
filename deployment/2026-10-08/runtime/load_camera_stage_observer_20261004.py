#!/usr/bin/env python3
"""Reload only the existing camera for a finite pixel-stage diagnostic."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import xmlrpc.client
from datetime import datetime, timezone

WORK = Path(__file__).resolve().parents[1]
OBSERVER = WORK / '.runtime/camera_stage_observer.py'
OBSERVER_SHA = '7e4d9e2c8f00b9f948d93a27528e6ac9e5fb068565b51f894770457c6c173676'
MOUNT = Path('/media/zhuyupei/SanStand')
PRESERVE = ('/alicia_d_driver_node', '/bessica_d_hw_interface', '/grasp_task_node',
            '/remote_grasp6d_node', '/motion_gateway', '/unknown_tabletop_perception')


def digest(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def load_helpers():
    p = WORK / '.runtime/gen4_root_cause/reload_camera_visual_gate.py'
    spec = importlib.util.spec_from_file_location('camera_reload_helpers', p)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main():
    if not MOUNT.is_mount():
        raise RuntimeError('SanStand unavailable; no fallback')
    assert digest(OBSERVER) == OBSERVER_SHA, 'observer changed after tests/review'
    helper = load_helpers()
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    out = MOUNT / '实验记录/Robot-Controller/ros_log' / ('camera_stage_capture_' + stamp)
    out.mkdir()
    (out / 'ros').mkdir()
    os.environ['ROS_LOG_DIR'] = str(out / 'ros')
    import rospy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Header, Bool, String
    from alicia_flexible_grasp_supervisor.msg import GraspState
    rospy.init_node('camera_stage_loader', anonymous=True, disable_signals=True)
    report = dict(output=str(out), camera_loaded=False, capture_complete=False,
                  robot_commands_sent=False, sensor_options_changed=False,
                  temperature_queried=False, observer_sha256=OBSERVER_SHA)
    current = dict(sdk=None, epoch=None)
    frozen = dict(sdk=None, epoch=None)
    faults, subscriptions = [], []

    def on_sdk(msg):
        if set(msg.name) != {'Joint%d' % i for i in range(1, 7)}:
            faults.append('invalid SDK command reference'); return
        values = dict(zip(msg.name, msg.position))
        q = tuple(values['Joint%d' % i] for i in range(1, 7))
        if frozen['sdk'] is not None and q != frozen['sdk']:
            faults.append('SDK hold changed during camera-only diagnostic')
        current['sdk'] = q

    def on_epoch(msg):
        ns = msg.stamp.to_nsec()
        if frozen['epoch'] is not None and ns != frozen['epoch']:
            faults.append('control epoch changed during camera-only diagnostic')
        current['epoch'] = ns

    def guard():
        if faults: raise RuntimeError(faults[0])
        if rospy.is_shutdown(): raise RuntimeError('loader ROS shutdown')
        if not MOUNT.is_mount(): raise RuntimeError('SanStand disappeared')

    try:
        task = rospy.wait_for_message('/grasp/state', GraspState, timeout=5)
        if task.active:
            raise RuntimeError('active grasp task; camera diagnostic not started')
        before = {name: helper.identity(helper.node_pid(rospy, name)) for name in PRESERVE}
        cfg = rospy.get_param('/camera')
        selection = rospy.get_param('/grasp_mode/selection')
        handeye = rospy.get_param('/handeye', None)
        old = helper.identity(helper.node_pid(rospy, '/supervisor_camera'))
        entry, remaps = helper.camera_entry_and_ros_tail(old['argv'])
        environment = helper.camera_environment(old['pid'])
        subscriptions.append(rospy.Subscriber('/alicia_d/sdk_command', JointState, on_sdk, queue_size=20))
        subscriptions.append(rospy.Subscriber('/alicia_d/control_reference_epoch', Header, on_epoch, queue_size=1))
        helper.wait(lambda: current['sdk'] is not None and current['epoch'] is not None,
                    5, 'current SDK reference', guard)
        frozen.update(current)
        unit = 'alicia-camera-stage-' + stamp.lower()
        command = ['systemd-run', '--user', '--collect', '--unit=' + unit,
                   '--property=Type=exec', '--property=Restart=no',
                   '--property=WorkingDirectory=' + old['cwd'],
                   '--property=StandardOutput=append:' + str(out / 'camera.stdout.log'),
                   '--property=StandardError=append:' + str(out / 'camera.stderr.log')]
        command += ['--setenv=' + key + '=' + value for key, value in environment.items()]
        command += ['--setenv=ROS_LOG_DIR=' + str(out / 'ros'), '--setenv=PYTHONUNBUFFERED=1']
        remaps = [arg for arg in remaps if not arg.startswith('__log:=')]
        remaps.append('__log:=' + str(out / 'camera.ros.log'))
        command += [old['argv'][0], str(OBSERVER), '--camera-script', entry,
                    '--output', str(out / 'stages'), '--seconds', '30', '--samples', '12',
                    '--interval', '2', '--warmup', '3', '--'] + remaps
        inputs = dict(old_camera=old, preserved_nodes=before, camera_parameters=cfg,
                      selection=selection, handeye=handeye, held_reference=frozen,
                      retained_environment=environment, systemd_argv=command)
        (out / 'load_inputs.json').write_text(json.dumps(inputs, ensure_ascii=False, indent=2) + '\n')
        for name in ('camera_stage_observer.py', 'test_camera_stage_observer.py',
                     'camera_stage_observer_tests_20261004.log',
                     'gen13_camera_chain_diagnostic_20261004.json'):
            source = WORK / '.runtime' / name
            if source.exists(): (out / name).write_bytes(source.read_bytes())
        guard()
        if not helper.alive(old) or helper.node_pid(rospy, '/supervisor_camera') != old['pid']:
            raise RuntimeError('camera identity changed')
        os.kill(old['pid'], signal.SIGINT)
        report['old_camera_sigint_pid'] = old['pid']
        helper.wait(lambda: not helper.alive(old), 12, 'old camera exit', guard)
        started = subprocess.run(command, capture_output=True, text=True, timeout=10)
        (out / 'systemd_start.txt').write_text(started.stdout + started.stderr)
        started.check_returncode()
        report['systemd_unit'] = unit
        print(json.dumps({'phase': 'camera_only_started', 'output': str(out), 'unit': unit}), flush=True)

        def new_camera():
            try:
                pid = int(subprocess.check_output(['systemctl', '--user', 'show', unit,
                          '--property=MainPID', '--value'], text=True, timeout=2).strip())
                if pid <= 0 or pid == old['pid'] or helper.node_pid(rospy, '/supervisor_camera') != pid:
                    return False
                info = helper.identity(pid)
                if info['exe_sha256'] != old['exe_sha256'] or str(OBSERVER) not in info['argv']:
                    raise RuntimeError('unexpected replacement camera executable')
                return info
            except (helper.CameraRegistrationPending, OSError, subprocess.SubprocessError,
                    xmlrpc.client.Error):
                return False

        report['new_camera'] = helper.wait(new_camera, 15, 'replacement camera', guard)
        report['camera_loaded'] = True
        summary_file = out / 'stages/summary.json'

        def completed():
            if not summary_file.exists(): return False
            try: return json.loads(summary_file.read_text())
            except json.JSONDecodeError: return False

        summary = helper.wait(completed, 42, 'finite stage capture', guard)
        report['stage_summary'] = summary
        for item in summary['files']:
            p = out / 'stages' / item['name']
            assert p.stat().st_size == item['bytes'] and digest(p) == item['sha256']
        after_cfg = rospy.get_param('/camera')
        assert helper.stable_camera_configuration(cfg) == helper.stable_camera_configuration(after_cfg)
        assert rospy.get_param('/grasp_mode/selection') == selection
        assert rospy.get_param('/handeye', None) == handeye
        for name, previous in before.items():
            actual = helper.identity(helper.node_pid(rospy, name))
            assert (actual['pid'], actual['start_ticks'], actual['exe_sha256']) == (
                previous['pid'], previous['start_ticks'], previous['exe_sha256']), name
        report.update(preserved_nodes_unchanged=True, geometry_configuration_unchanged=True,
                      unchanged_sdk_reference=current, capture_complete=summary['capture_complete'],
                      motion_enabled=rospy.wait_for_message('/alicia_d/motion_enabled', Bool, timeout=5).data,
                      feedback_ready=rospy.wait_for_message('/alicia_d/feedback_ready', Bool, timeout=5).data,
                      actuation_status=rospy.wait_for_message('/alicia_d/actuation_status', String, timeout=5).data)
        guard()
        (out / 'camera_parameters_after.json').write_text(json.dumps(after_cfg, indent=2) + '\n')
        if not summary['capture_complete']:
            raise RuntimeError('stage capture incomplete; camera remains running, inspect preserved data')
    except BaseException as exc:
        report['error'] = str(exc)
        raise
    finally:
        for sub in subscriptions: sub.unregister()
        report['finished_at_utc'] = datetime.now(timezone.utc).isoformat()
        (out / 'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
        print(json.dumps({'output': str(out), 'camera_loaded': report['camera_loaded'],
                         'capture_complete': report['capture_complete'], 'error': report.get('error')},
                         ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
