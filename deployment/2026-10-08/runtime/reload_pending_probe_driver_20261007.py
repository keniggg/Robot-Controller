"""One deployment: reviewed driver plus a fresh GUI, never sends joint targets."""
from datetime import datetime, timezone
import argparse
import hashlib
import http.client
import json
import os
from pathlib import Path
import subprocess
import time
import xmlrpc.client

WORK = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
EXT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
RUN = EXT / 'ros_log/pending_actuation_drift_20261007T104410.504268Z'
DRIVER = '/alicia_d_driver_node'
GUI = '/alicia_supervisor_gui'
PRESERVED = ('/bessica_d_hw_interface', '/motion_gateway', '/move_group',
             '/grasp_task_node', '/supervisor_camera', '/unknown_tabletop_perception',
             '/grasp_mode_router', '/remote_grasp6d_node')


class TimeoutTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        return http.client.HTTPConnection(host, timeout=3.)


def same_process(item):
    try:
        fields = (Path('/proc') / str(item['pid']) / 'stat').read_text().split(') ', 1)[1].split()
        return fields[19] == item['start_ticks']
    except FileNotFoundError:
        return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--resume-gui-closed', action='store_true')
    args = parser.parse_args()
    if not Path('/media/zhuyupei/SanStand').is_mount():
        raise RuntimeError('SanStand required; no fallback')
    build = json.loads((RUN / 'driver_build.json').read_text())
    assert build['returncode'] == 0
    binary = WORK / 'devel/lib/alicia_d_driver/alicia_d_driver_node'
    assert hashlib.sha256(binary.read_bytes()).hexdigest() == build['binary_sha256']
    for name, sha in build['source_sha256'].items():
        assert hashlib.sha256((WORK / name).read_bytes()).hexdigest() == sha, name
    previous = None
    if args.resume_gui_closed:
        previous = json.loads((RUN / 'deployment/result.json').read_text())
        assert previous.get('error') == "RuntimeError('Old GUI still registered; driver untouched')"
        assert not previous['completed'] and 'shutdown_reply' not in previous
        assert same_process(previous['before'][DRIVER])
        assert not same_process(previous['before'][GUI])
    out = RUN / ('deployment_resume' if previous else 'deployment')
    out.mkdir()  # Never silently rerun a partially completed transaction.
    (out / 'ros').mkdir()
    os.environ['ROS_LOG_DIR'] = str(out / 'ros')
    import rosgraph
    import rospy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, Header, String
    from alicia_flexible_grasp_supervisor.msg import GraspState
    rospy.init_node('pending_probe_driver_deploy', anonymous=True, disable_signals=True)
    master = rosgraph.Master(rospy.get_name())

    def identity(name):
        uri = master.lookupNode(name)
        code, detail, pid = xmlrpc.client.ServerProxy(uri, transport=TimeoutTransport()).getPid(rospy.get_name())
        if code != 1:
            raise RuntimeError(detail)
        proc = Path('/proc') / str(pid)
        fields = (proc / 'stat').read_text().split(') ', 1)[1].split()
        return dict(pid=pid, start_ticks=fields[19], uri=uri,
                    binary_sha256=hashlib.sha256((proc / 'exe').read_bytes()).hexdigest())

    def state():
        feedback = rospy.wait_for_message('/alicia_d/accepted_joint_states', JointState, timeout=8)
        epoch = rospy.wait_for_message('/alicia_d/control_reference_epoch', Header, timeout=5)
        if feedback.header.frame_id != 'sdk_measured':
            raise RuntimeError('Real feedback not available')
        return dict(epoch_ns=epoch.stamp.to_nsec(),
                    feedback=dict(stamp_ns=feedback.header.stamp.to_nsec(), names=feedback.name,
                                  positions=feedback.position, frame_id=feedback.header.frame_id),
                    actuation=rospy.wait_for_message('/alicia_d/actuation_status', String, timeout=5).data,
                    feedback_ready=rospy.wait_for_message('/alicia_d/feedback_ready', Bool, timeout=5).data,
                    motion_enabled=rospy.wait_for_message('/alicia_d/motion_enabled', Bool, timeout=5).data,
                    selection=rospy.get_param('/grasp_mode/selection'),
                    manual=rospy.get_param('/gui/joint_direct_mode'))

    result = dict(started_utc=datetime.now(timezone.utc).isoformat(), completed=False,
                  joint_targets_sent=False, torque_off_sent=False,
                  positive_enable='Driver startup only; no extra publish',
                  gui_restart_reason='Discard all queued pre-reload user inputs',
                  awaiting_new_user_slider_validation=True)
    try:
        if rospy.wait_for_message('/grasp/state', GraspState, timeout=5).active:
            raise RuntimeError('An active task owns the arm')
        if rospy.get_param('/gui/joint_direct_mode') is not True:
            raise RuntimeError('Manual direct mode expected')
        if previous:
            result['resume_of'] = str(RUN / 'deployment/result.json')
            result['before'] = {name: identity(name) for name in (DRIVER,) + PRESERVED}
            assert result['before'][DRIVER] == previous['before'][DRIVER]
            result['before'][GUI] = previous['before'][GUI]
        else:
            result['before'] = {name: identity(name) for name in (DRIVER, GUI) + PRESERVED}
        result['before_state'] = state()
        old = result['before'][DRIVER]
        # Close only the GUI service: all queued targets disappear with it.
        if not previous:
            subprocess.run(['systemctl', '--user', 'stop',
                            'alicia-gui-resume-20261007t102730z.service'], check=True, timeout=15)
        deadline = time.monotonic() + 12.
        while same_process(result['before'][GUI]) and time.monotonic() < deadline:
            time.sleep(.1)
        if same_process(result['before'][GUI]):
            raise RuntimeError('Old GUI still alive; driver untouched')
        # ROS master can retain a dead GUI registration. The bound process,
        # not that stale name, determines whether its input queue still exists.
        # Destructor closes the serial port and sends no torque-off frame.
        try:
            result['shutdown_reply'] = xmlrpc.client.ServerProxy(
                old['uri'], transport=TimeoutTransport()).shutdown(
                rospy.get_name(), 'Load tested pending probe response fix; preserve torque')
        except (OSError, EOFError, xmlrpc.client.ProtocolError) as exc:
            result['shutdown_reply_error'] = repr(exc)
        deadline = time.monotonic() + 12.
        while same_process(old) and time.monotonic() < deadline:
            time.sleep(.1)
        if same_process(old):
            raise RuntimeError('Old driver still owns serial; no replacement started')
        suffix = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ').lower()
        unit = 'alicia-pending-probe-driver-' + suffix
        command = ['systemd-run', '--user', '--collect', '--unit=' + unit,
                   '--property=Type=exec', '--property=Restart=no',
                   '--property=WorkingDirectory=' + str(WORK),
                   '--property=StandardOutput=append:' + str(out / 'driver.log'),
                   '--property=StandardError=append:' + str(out / 'driver.log'),
                   '/bin/bash', str(WORK / '.runtime/start_pending_probe_driver_20261007.sh'),
                   str(out / 'ros_driver')]
        result['driver_unit'] = unit
        result['driver_start_command'] = command
        subprocess.run(command, check=True, timeout=15)
        deadline = time.monotonic() + 15.
        while time.monotonic() < deadline:
            try:
                current = identity(DRIVER)
                if current['pid'] != old['pid']:
                    break
            except (rosgraph.masterapi.Error, OSError):
                pass
            time.sleep(.15)
        else:
            raise RuntimeError('Replacement driver registration timeout')
        assert current['binary_sha256'] == build['binary_sha256']
        time.sleep(2)
        result['after_driver_state'] = state()
        assert result['after_driver_state']['epoch_ns'] != result['before_state']['epoch_ns']
        gui_unit = 'alicia-slider-gui-' + suffix
        result['gui_unit'] = gui_unit
        subprocess.run(['systemd-run', '--user', '--collect', '--unit=' + gui_unit,
                        '--property=Type=exec', '--property=Restart=no',
                        '--property=WorkingDirectory=' + str(WORK),
                        '--property=StandardOutput=append:' + str(out / 'gui.log'),
                        '--property=StandardError=append:' + str(out / 'gui.log'),
                        '/bin/bash', str(WORK / '.runtime/start_existing_stack_gui_20261006.sh'),
                        str(out / 'ros_gui')], check=True, timeout=15)
        deadline = time.monotonic() + 15.
        while time.monotonic() < deadline:
            try:
                if identity(GUI)['pid'] != result['before'][GUI]['pid']:
                    break
            except (rosgraph.masterapi.Error, OSError):
                pass
            time.sleep(.2)
        else:
            raise RuntimeError('GUI registration timeout')
        result['after'] = {name: identity(name) for name in (DRIVER, GUI) + PRESERVED}
        for name in PRESERVED:
            assert result['before'][name] == result['after'][name], name
        result['after_state'] = state()
        assert result['after_state']['manual'] is True
        assert result['after_state']['selection'] == result['before_state']['selection']
        result['completed'] = True
    except Exception as exc:
        result['error'] = repr(exc)
        raise
    finally:
        result['finished_utc'] = datetime.now(timezone.utc).isoformat()
        (out / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
        print(json.dumps(result, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
