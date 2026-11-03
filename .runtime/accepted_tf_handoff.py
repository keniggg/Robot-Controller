"""TF-only handoff while the original respawn-owning launch stays up.

Suspend exactly the old RSP (not its parent or group), run one RSP on accepted
SDK samples, restore the old process only after the replacement has exited.
Full startup uses the bringup remap instead. No robot command interface is used.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import xmlrpc.client

RSP = '/opt/ros/noetic/lib/robot_state_publisher/robot_state_publisher'
MOUNT = Path('/media/zhuyupei/SanStand')


def identity(pid):
    root = Path('/proc') / str(pid)
    fields = (root / 'stat').read_text().rsplit(')', 1)[1].split()
    return dict(pid=pid, state=fields[0], ppid=int(fields[1]), start_ticks=int(fields[19]),
                argv=(root / 'cmdline').read_bytes().decode().strip('\0').split('\0'))


def same_process(saved):
    try:
        current = identity(saved['pid'])
    except (FileNotFoundError, ProcessLookupError):
        return False
    return all(current[k] == saved[k] for k in ('pid', 'ppid', 'start_ticks', 'argv'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--legacy-pid', type=int, required=True)
    parser.add_argument('--parent-pid', type=int, required=True)
    parser.add_argument('--model-sha256', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    out = args.output.resolve()
    if not MOUNT.is_mount() or MOUNT not in out.parents:
        raise RuntimeError('mounted SanStand is required')
    out.mkdir(parents=True, exist_ok=True)
    (out / 'ros').mkdir(exist_ok=True)
    os.environ['ROS_LOG_DIR'] = str(out / 'ros')
    socket.setdefaulttimeout(4)
    import rospy
    import rosgraph
    from alicia_flexible_grasp_supervisor.msg import GraspState
    from sensor_msgs.msg import JointState
    rospy.init_node('accepted_tf_handoff', anonymous=False, disable_signals=True)
    master = rosgraph.Master('/accepted_tf_handoff')
    legacy = identity(args.legacy_pid)
    parent = identity(args.parent_pid)
    if (legacy['argv'][0] != RSP or '__name:=robot_state_publisher' not in legacy['argv']
            or legacy['ppid'] != args.parent_pid or legacy['state'] in ('T', 't', 'Z')):
        raise RuntimeError('legacy RSP PID/parent/argv is not the expected live process')
    api = xmlrpc.client.ServerProxy(master.lookupNode('/robot_state_publisher'))
    if api.getPid('/accepted_tf_handoff')[2] != args.legacy_pid:
        raise RuntimeError('legacy ROS node PID mismatch')
    subs = api.getSubscriptions('/accepted_tf_handoff')[2]
    if ['/joint_states', 'sensor_msgs/JointState'] not in subs:
        raise RuntimeError('legacy RSP is not the heartbeat-input instance')
    if any('/accepted_robot_state_publisher' in nodes for section in master.getSystemState() for _, nodes in section):
        raise RuntimeError('replacement node already exists')
    state = rospy.wait_for_message('/grasp/state', GraspState, timeout=4)
    if state.active:
        raise RuntimeError('TF handoff requires the grasp task to be idle')
    accepted = rospy.wait_for_message('/alicia_d/accepted_joint_states', JointState, timeout=4)
    if not 0 <= (rospy.Time.now() - accepted.header.stamp).to_sec() < 0.5:
        raise RuntimeError('accepted SDK feedback is not fresh')
    model_sha = hashlib.sha256(rospy.get_param('/robot_description').encode()).hexdigest()
    if model_sha != args.model_sha256:
        raise RuntimeError('live model changed')
    record = dict(legacy=legacy, parent=parent, model_sha256=model_sha,
                  input_topic='/alicia_d/accepted_joint_states', hardware_commands_sent=0,
                  started_ns=time.time_ns(), status='prepared')

    def event(status, **values):
        record.update(status=status, updated_ns=time.time_ns(), **values)
        (out / 'handoff.json').write_text(json.dumps(record, indent=2) + '\n')
        print(json.dumps(dict(status=status, **values)), flush=True)

    stop = [False]
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda *_: stop.__setitem__(0, True))
    child = None
    frozen = False
    try:
        if not same_process(legacy) or not same_process(parent):
            raise RuntimeError('process identity changed before handoff')
        os.kill(legacy['pid'], signal.SIGSTOP)
        frozen = True
        deadline = time.monotonic() + 2
        while identity(legacy['pid'])['state'] not in ('T', 't'):
            if time.monotonic() > deadline:
                raise RuntimeError('legacy RSP did not suspend')
            time.sleep(0.01)
        event('legacy_rsp_suspended')
        with (out / 'replacement_rsp.log').open('a') as log:
            child = subprocess.Popen([RSP, 'joint_states:=/alicia_d/accepted_joint_states',
                                      '__name:=accepted_robot_state_publisher'],
                                     stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            event('replacement_started', replacement_pid=child.pid)
            while not stop[0] and not rospy.is_shutdown():
                if child.poll() is not None:
                    raise RuntimeError('replacement RSP exited: %s' % child.returncode)
                if not same_process(parent) or not same_process(legacy):
                    raise RuntimeError('original launch/RSP identity changed')
                if identity(legacy['pid'])['state'] not in ('T', 't'):
                    raise RuntimeError('legacy RSP resumed unexpectedly')
                time.sleep(0.1)
    except Exception as exc:
        event('handoff_error', error=str(exc))
        raise
    finally:
        if child is not None and child.poll() is None:
            # This group belongs only to the RSP created above.
            os.killpg(child.pid, signal.SIGINT)
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGTERM)
                child.wait(timeout=5)
        if frozen and same_process(legacy):
            os.kill(legacy['pid'], signal.SIGCONT)
            event('replacement_exited_legacy_restored')
        rospy.signal_shutdown('TF-only handoff ended')


if __name__ == '__main__':
    main()
