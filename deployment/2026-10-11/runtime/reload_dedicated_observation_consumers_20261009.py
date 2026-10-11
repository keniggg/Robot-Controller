"""Reload the previously recorded three-consumer unit; preserve hardware nodes."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
import xmlrpc.client


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--previous-deployment', type=Path, required=True)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    work = Path(__file__).resolve().parents[1]
    mount = Path('/media/zhuyupei/SanStand')
    assert mount.is_mount() and mount in args.output.resolve().parents
    previous = json.loads(args.previous_deployment.read_text())
    manifest = json.loads(args.manifest.read_text())
    assert previous['deployment_passed'] is True
    assert manifest['passed'] is True and manifest['reviewed'] is True
    unit = previous['unit']
    if unit.endswith('.service'):
        unit = unit[:-8]
    assert unit.startswith('alicia-observation-measurement-')
    assert unit[len('alicia-observation-measurement-'):].isdigit()

    def sources():
        for relative, digest in manifest['source_sha256'].items():
            assert hashlib.sha256((work / relative).read_bytes()).hexdigest() == digest, relative

    sources()
    args.output.mkdir(exist_ok=False)
    os.environ['ROS_LOG_DIR'] = str(args.output / 'observer_ros')
    import rosgraph
    import rospy
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger
    from alicia_flexible_grasp_supervisor.msg import GraspState
    rospy.init_node('reload_dedicated_observation_consumers', anonymous=True, disable_rosout=True)
    master = rosgraph.Master(rospy.get_name())

    class Transport(xmlrpc.client.Transport):
        def make_connection(self, host):
            connection = super().make_connection(host)
            connection.timeout = 3.
            return connection

    def identity(name):
        server = xmlrpc.client.ServerProxy(master.lookupNode(name), transport=Transport())
        code, _, pid = server.getPid(rospy.get_name())
        assert code == 1
        fields = (Path('/proc') / str(pid) / 'stat').read_text().rsplit(')', 1)[1].split()
        assert fields[0] != 'Z'
        return dict(pid=pid, start_ticks=fields[19], parent_pid=int(fields[1]))

    def idle():
        state = rospy.wait_for_message('/grasp/state', GraspState, timeout=5)
        status = json.loads(rospy.wait_for_message('/grasp_mode/status', String, timeout=5).data)
        assert not state.active
        assert not any(status.get(key) for key in ('active', 'start_reserved', 'pending'))

    targets = ('/grasp_task_node', '/remote_grasp6d_node', '/unknown_tabletop_perception')
    preserve = tuple(previous['preserved_after'])
    assert set(preserve) == {'/alicia_d_driver_node', '/bessica_d_hw_interface', '/move_group',
        '/motion_gateway', '/supervisor_camera', '/compliant_gripper_controller',
        '/grasp_mode_router', '/alicia_supervisor_gui'}
    result = dict(deployment_passed=False, previous_deployment=str(args.previous_deployment),
        source_sha256=manifest['source_sha256'], motion_commands_sent=False,
        torque_commands_sent=False, temperature_queried=False, stopped_unit=unit,
        new_selection_required_before_automatic_grasp=True)

    def save():
        (args.output / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')

    try:
        idle()
        result['before'] = {name: identity(name) for name in targets}
        result['preserved_before'] = {name: identity(name) for name in preserve}
        assert result['before'] == previous['after']
        assert result['preserved_before'] == previous['preserved_after']
        parent = int(subprocess.check_output(['systemctl', '--user', 'show', unit,
            '--property=MainPID', '--value'], text=True).strip())
        assert parent > 1 and all(row['parent_pid'] == parent for row in result['before'].values())
        command = (Path('/proc') / str(parent) / 'cmdline').read_bytes().rstrip(b'\0').split(b'\0')
        launchers = [index for index, value in enumerate(command)
                     if value.rsplit(b'/', 1)[-1] == b'roslaunch']
        assert len(launchers) == 1 and launchers[0] in (0, 1)
        assert command[launchers[0] + 1:] == [b'--skip-log-check',
            b'.runtime/observation_measurement_consumers_20261008.launch']
        result['old_parent_pid'] = parent
        control = rospy.Publisher('/gui/joint_direct_mode', Bool, queue_size=1, latch=True)
        deadline = time.monotonic() + 5.
        while not control.get_num_connections() and time.monotonic() < deadline:
            time.sleep(.05)
        assert control.get_num_connections()
        rospy.set_param('/gui/joint_direct_mode', True)
        control.publish(Bool(data=True))
        idle()
        save()
        subprocess.run(['systemctl', '--user', 'stop', unit], check=True, timeout=35.)
        for row in result['before'].values():
            path = Path('/proc') / str(row['pid']) / 'stat'
            if path.exists():
                fields = path.read_text().rsplit(')', 1)[1].split()
                assert fields[0] == 'Z' or fields[19] != row['start_ticks']
        assert {name: identity(name) for name in preserve} == result['preserved_before']
        sources()
        new_unit = 'alicia-observation-measurement-' + str(time.time_ns())
        command = ['systemd-run', '--user', '--collect', '--unit=' + new_unit,
            '--property=Type=exec', '--property=Restart=no', '--property=WorkingDirectory=' + str(work),
            '--property=StandardOutput=append:' + str(args.output / 'consumers.log'),
            '--property=StandardError=append:' + str(args.output / 'consumers.log'),
            '--setenv=ROS_LOG_DIR=' + str(args.output / 'consumers_ros'),
            '/bin/bash', str(work / '.runtime/start_observation_measurement_consumers_20261008.sh')]
        result.update(unit=new_unit, command=command)
        save()
        subprocess.run(command, check=True, timeout=15.)
        for service in ('/grasp_mode/reset_remote', '/grasp_mode/reset_task'):
            rospy.wait_for_service(service, timeout=40.)
            reply = rospy.ServiceProxy(service, Trigger)()
            assert reply.success, reply.message
            result.setdefault('reset_ack', {})[service] = reply.message
        idle()
        result['after'] = {name: identity(name) for name in targets}
        result['preserved_after'] = {name: identity(name) for name in preserve}
        assert result['preserved_before'] == result['preserved_after']
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
