"""Load tested perception/planning/task consumers while GUI owns the idle task.

No robot, gripper, camera or controller lifecycle changes; no motion, torque or
temperature interfaces. User manual movement may continue during this reload.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
import xmlrpc.client


class Transport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 3.
        return connection


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    work = Path(__file__).resolve().parents[1]
    mount = Path('/media/zhuyupei/SanStand')
    assert mount.is_mount() and mount in args.output.resolve().parents
    manifest = json.loads(args.manifest.read_text())
    assert manifest['passed'] is True and manifest['reviewed'] is True

    def sources():
        for relative, digest in manifest['source_sha256'].items():
            assert hashlib.sha256((work / relative).read_bytes()).hexdigest() == digest, relative

    sources()
    args.output.mkdir(exist_ok=False)
    os.environ['ROS_LOG_DIR'] = str(args.output / 'observer_ros')
    import rosgraph
    import rospy
    from alicia_flexible_grasp_supervisor.msg import GraspState
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    rospy.init_node('load_observation_measurement_consumers', anonymous=True, disable_rosout=True)
    master = rosgraph.Master(rospy.get_name())
    targets = ('/grasp_task_node', '/remote_grasp6d_node', '/unknown_tabletop_perception')
    preserve = ('/alicia_d_driver_node', '/bessica_d_hw_interface', '/move_group',
                '/motion_gateway', '/supervisor_camera', '/compliant_gripper_controller',
                '/grasp_mode_router', '/alicia_supervisor_gui')

    def rpc(name):
        return xmlrpc.client.ServerProxy(master.lookupNode(name), transport=Transport())

    def identity(name):
        code, _, pid = rpc(name).getPid(rospy.get_name())
        assert code == 1
        stat = (Path('/proc') / str(pid) / 'stat').read_text().rsplit(')', 1)[1].split()
        assert stat[0] != 'Z'
        return dict(pid=pid, start_ticks=stat[19], parent_pid=int(stat[1]))

    def manual_idle(require_task=True):
        assert rospy.get_param('/gui/joint_direct_mode') is True
        status = json.loads(rospy.wait_for_message('/grasp_mode/status', String, timeout=5).data)
        assert not (status.get('active') or status.get('start_reserved') or status.get('pending'))
        if require_task:
            assert not rospy.wait_for_message('/grasp/state', GraspState, timeout=5).active

    result = dict(deployment_passed=False, source_sha256=manifest['source_sha256'],
                  motion_commands_sent=False, torque_commands_sent=False, temperature_queried=False,
                  manual_motion_allowed=True, new_selection_required_before_automatic_grasp=True)

    def save():
        (args.output / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')

    try:
        manual_idle()
        result.update(before={name: identity(name) for name in targets},
                      preserved_before={name: identity(name) for name in preserve})
        save()
        for name in targets:
            assert identity(name) == result['before'][name]
            code, message, _ = rpc(name).shutdown(rospy.get_name(),
                'load tested observation measurement consumers while GUI owns control')
            assert code == 1, message
            result.setdefault('shutdown_requests_sent', []).append(name)
            save()
        deadline = time.monotonic() + 25.
        while time.monotonic() < deadline:
            remaining = []
            for name, previous in result['before'].items():
                path = Path('/proc') / str(previous['pid']) / 'stat'
                if path.exists():
                    stat = path.read_text().rsplit(')', 1)[1].split()
                    if stat[0] != 'Z' and stat[19] == previous['start_ticks']:
                        remaining.append(name)
            if not remaining:
                break
            time.sleep(.2)
        else:
            raise RuntimeError('old consumers did not exit: ' + repr(remaining))
        manual_idle(require_task=False)
        sources()
        unit = 'alicia-observation-measurement-' + str(time.time_ns())
        command = ['systemd-run', '--user', '--collect', '--unit=' + unit,
            '--property=Type=exec', '--property=Restart=no', '--property=WorkingDirectory=' + str(work),
            '--property=StandardOutput=append:' + str(args.output / 'consumers.log'),
            '--property=StandardError=append:' + str(args.output / 'consumers.log'),
            '--setenv=ROS_LOG_DIR=' + str(args.output / 'consumers_ros'),
            '/bin/bash', str(work / '.runtime/start_observation_measurement_consumers_20261008.sh')]
        result.update(unit=unit, command=command)
        save()
        subprocess.run(command, check=True, timeout=15.)
        for service in ('/grasp_mode/reset_remote', '/grasp_mode/reset_task'):
            rospy.wait_for_service(service, timeout=40.)
            reply = rospy.ServiceProxy(service, Trigger)()
            assert reply.success, reply.message
            result.setdefault('reset_ack', {})[service] = reply.message
        manual_idle()
        result.update(after={name: identity(name) for name in targets},
                      preserved_after={name: identity(name) for name in preserve})
        assert result['preserved_before'] == result['preserved_after']
        assert all(result['after'][name] != result['before'][name] for name in targets)
        sources()
        result['deployment_passed'] = True
    except Exception as exc:
        result['error'] = repr(exc)
        raise
    finally:
        save()
        print(json.dumps(dict(deployment_passed=result['deployment_passed'],
            output=str(args.output), error=result.get('error')), ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
