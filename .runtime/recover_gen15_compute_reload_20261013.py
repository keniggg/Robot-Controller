"""Complete the recorded partial GEN15 compute reload; never manage hardware."""
import argparse
import hashlib
import http.client
import json
import os
from pathlib import Path
import subprocess
import time
import xmlrpc.client


TARGETS = ('/grasp_task_node', '/remote_grasp6d_node', '/unknown_tabletop_perception')
PRESERVE = ('/alicia_d_driver_node', '/bessica_d_hw_interface', '/move_group',
            '/motion_gateway', '/supervisor_camera', '/compliant_gripper_controller',
            '/grasp_mode_router', '/alicia_supervisor_gui')
CONNECTION_ERRORS = (OSError, http.client.RemoteDisconnected)


class Transport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 3.
        return connection


def process_state(expected):
    """PID reuse is an unknown identity, never evidence that shutdown succeeded."""
    try:
        fields = (Path('/proc') / str(expected['pid']) / 'stat').read_text().rsplit(')', 1)[1].split()
    except FileNotFoundError:
        return 'exited'
    assert fields[19] == expected['start_ticks'], 'Recorded PID was reused: ' + str(expected)
    assert int(fields[1]) == expected['parent_pid'], 'Recorded process parent changed: ' + str(expected)
    return 'exited' if fields[0] == 'Z' else 'running'


def main():
    parser = argparse.ArgumentParser()
    for name in ('manifest', 'previous-deployment', 'partial-reload',
                 'initial-state', 'manual-receipt', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    args = parser.parse_args()
    work = Path(__file__).resolve().parents[1]
    mount = Path('/media/zhuyupei/SanStand')
    assert mount.is_mount() and mount in args.output.resolve().parents
    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    inputs = {name: getattr(args, name) for name in (
        'manifest', 'previous_deployment', 'partial_reload', 'initial_state', 'manual_receipt')}
    pins = {str(path.resolve()): sha(path) for path in inputs.values()}
    manifest, previous, partial, initial, manual = (
        json.loads(inputs[name].read_text()) for name in (
            'manifest', 'previous_deployment', 'partial_reload', 'initial_state', 'manual_receipt'))
    assert manifest['passed'] and manifest['reviewed'] and manifest['installed']
    assert previous['deployment_passed'] and previous['whole_patch_deployed']
    assert partial['deployment_passed'] is False and 'after' not in partial and 'command' not in partial
    assert set(partial['before']) == set(TARGETS)
    assert set(partial['preserved_before']) == set(PRESERVE)
    assert partial['before'] == previous['after']
    assert partial['preserved_before'] == previous['preserved_after']
    assert partial['source_sha256'] == manifest['source_sha256']
    assert not initial['task']['active']
    assert not any(initial['mode'].get(k) for k in ('active', 'start_reserved', 'pending'))
    assert manual['manual_restored'] and manual['gui_own_publisher_acknowledged'] and manual['verified_inactive']
    assert manual['generation'] == initial['selection']['generation']
    launcher = '.runtime/start_observation_measurement_consumers_20261008.sh'
    assert launcher in manifest['source_sha256']

    def sources():
        assert all(sha(Path(path)) == value for path, value in pins.items()), 'Recovery input changed'
        for relative, value in manifest['source_sha256'].items():
            assert sha(work / relative) == value, relative

    sources()
    args.output.mkdir(exist_ok=False)
    os.environ['ROS_LOG_DIR'] = str(args.output / 'observer_ros')
    import rosgraph
    import rospy
    from actionlib_msgs.msg import GoalStatusArray
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from alicia_flexible_grasp_supervisor.msg import GraspState
    from alicia_grasp_modes.selection import parse_selection
    rospy.init_node('recover_gen15_compute_reload', anonymous=True, disable_rosout=True)
    master = rosgraph.Master(rospy.get_name())
    selection = parse_selection(initial['selection'])
    result = dict(deployment_passed=False, previous_deployment=str(args.previous_deployment),
        partial_reload=str(args.partial_reload), evidence_sha256=pins,
        recovery_script_sha256=sha(Path(__file__)), before=partial['before'],
        preserved_before=partial['preserved_before'], source_sha256=manifest['source_sha256'],
        selection=selection, shutdown_requests_sent=[], shutdown_observations={},
        new_launch_started=False, motion_commands_sent=False, torque_commands_sent=False,
        temperature_queried=False, manual_motion_allowed=True,
        new_selection_required_before_automatic_grasp=True)

    def save():
        temporary = args.output / 'result.json.tmp'
        temporary.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
        temporary.replace(args.output / 'result.json')

    def rpc(name):
        return xmlrpc.client.ServerProxy(master.lookupNode(name), transport=Transport())

    def identity(name):
        code, message, pid = rpc(name).getPid(rospy.get_name())
        assert code == 1, message
        fields = (Path('/proc') / str(pid) / 'stat').read_text().rsplit(')', 1)[1].split()
        assert fields[0] != 'Z', name
        return dict(pid=pid, start_ticks=fields[19], parent_pid=int(fields[1]))

    def manual_idle(require_task=False):
        assert rospy.get_param('/gui/joint_direct_mode') is True
        assert parse_selection(rospy.get_param('/grasp_mode/selection')) == selection
        status = json.loads(rospy.wait_for_message('/grasp_mode/status', String, timeout=5).data)
        assert parse_selection(status) == selection
        assert not any(status.get(k) for k in ('active', 'start_reserved', 'pending'))
        for topic in ('/alicia_controller/follow_joint_trajectory/status',
                      '/hand_controller/follow_joint_trajectory/status'):
            state = rospy.wait_for_message(topic, GoalStatusArray, timeout=5)
            assert not any(g.status in (0, 1, 6, 7) for g in state.status_list), topic
        if require_task:
            assert not rospy.wait_for_message('/grasp/state', GraspState, timeout=5).active
        assert {name: identity(name) for name in PRESERVE} == partial['preserved_before']

    def wait_exited(name):
        deadline = time.monotonic() + 25.
        while process_state(partial['before'][name]) != 'exited':
            assert time.monotonic() < deadline, 'Recorded compute process did not exit: ' + name
            time.sleep(.1)

    def confirm_no_replacement(name):
        assert process_state(partial['before'][name]) == 'exited', name
        try:
            endpoint = master.lookupNode(name)
        except rosgraph.MasterError:
            return 'unregistered'
        try:
            reply = xmlrpc.client.ServerProxy(endpoint, transport=Transport()).getPid(rospy.get_name())
        except CONNECTION_ERRORS as exc:
            # A stale ROS registration is acceptable only with independent
            # recorded-process exit evidence. A responding replacement is not.
            assert process_state(partial['before'][name]) == 'exited', name
            return repr(exc)
        raise AssertionError('Compute name still has an endpoint: %s %r' % (name, reply))

    try:
        manual_idle()
        save()
        for name in TARGETS:
            expected = partial['before'][name]
            entry = result['shutdown_observations'][name] = dict(expected=expected)
            if process_state(expected) == 'running':
                manual_idle()
                try:
                    server = rpc(name)
                    code, message, pid = server.getPid(rospy.get_name())
                except (rosgraph.MasterError,) + CONNECTION_ERRORS as exc:
                    entry['identity_rpc_error'] = repr(exc)
                    save()
                    wait_exited(name)
                else:
                    assert code == 1 and pid == expected['pid'], (name, code, message, pid)
                    if process_state(expected) == 'running':
                        # Only this original, still-live process may receive
                        # shutdown. Never signal a replacement by node name.
                        entry['shutdown_attempted'] = True
                        save()
                        try:
                            code, message, _ = server.shutdown(rospy.get_name(),
                                'complete recorded GEN15 compute-only reload while GUI owns control')
                        except CONNECTION_ERRORS as exc:
                            entry['shutdown_rpc_error'] = repr(exc)
                            save()
                        else:
                            assert code == 1, message
                            result['shutdown_requests_sent'].append(name)
                            entry['shutdown_acknowledged'] = True
                            save()
                        wait_exited(name)
            entry['recorded_process_exited'] = True
            entry['master_endpoint_after_exit'] = confirm_no_replacement(name)
            save()
        # No new launcher before all original compute processes have exited.
        for name in TARGETS:
            confirm_no_replacement(name)
        manual_idle()
        sources()
        unit = 'alicia-observation-measurement-' + str(time.time_ns())
        command = ['systemd-run', '--user', '--collect', '--unit=' + unit,
            '--property=Type=exec', '--property=Restart=no', '--property=WorkingDirectory=' + str(work),
            '--property=StandardOutput=append:' + str(args.output / 'consumers.log'),
            '--property=StandardError=append:' + str(args.output / 'consumers.log'),
            '--setenv=ROS_LOG_DIR=' + str(args.output / 'consumers_ros'),
            '/bin/bash', str(work / launcher)]
        result.update(unit=unit, command=command, launch_attempted=True)
        save()
        subprocess.run(command, check=True, timeout=15.)
        result['new_launch_started'] = True
        save()
        for service in ('/grasp_mode/reset_remote', '/grasp_mode/reset_task'):
            rospy.wait_for_service(service, timeout=40.)
            reply = rospy.ServiceProxy(service, Trigger)()
            assert reply.success, reply.message
            ack = json.loads(reply.message)
            assert ack.get('ready') is True and parse_selection(ack) == selection, ack
            result.setdefault('reset_ack', {})[service] = reply.message
            save()
        manual_idle(require_task=True)
        result['after'] = {name: identity(name) for name in TARGETS}
        result['preserved_after'] = {name: identity(name) for name in PRESERVE}
        assert result['preserved_after'] == result['preserved_before']
        assert all((result['after'][n]['pid'], result['after'][n]['start_ticks']) !=
                   (result['before'][n]['pid'], result['before'][n]['start_ticks']) for n in TARGETS)
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
