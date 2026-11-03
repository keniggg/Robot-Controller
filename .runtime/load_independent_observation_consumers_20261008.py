"""Explicit idle-only four-node reload; never starts a second motion gateway.

Requires the root agent's passed source manifest and --execute. Gateway is
restarted by its verified existing roslaunch parent. The other three nodes
have non-respawning owners and are started by a separate parameter-free launch.
No trajectory, joint, gripper, torque, camera, or selection command is issued.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
import xmlrpc.client

import rosgraph
import rospy
from actionlib_msgs.msg import GoalStatusArray
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
from alicia_flexible_grasp_supervisor.msg import GraspState
from alicia_flexible_grasp_supervisor.srv import TriggerZero

WORK = Path(__file__).resolve().parents[1]
SESSION = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/supplement_grasp_20261008T103117Z')
DEFAULT_OUT = SESSION / 'independent_observation_deployment_retry1'
GATEWAY = '/motion_gateway'
NEW_LAUNCH_NODES = ('/remote_grasp6d_node', '/grasp_task_node', '/unknown_tabletop_perception')
TARGETS = NEW_LAUNCH_NODES + (GATEWAY,)
REQUIRED_PRESERVE = ('/alicia_d_driver_node', '/supervisor_camera',
    '/bessica_d_hw_interface', '/move_group', '/grasp_mode_router',
    '/compliant_gripper_controller', '/robot_state_publisher', '/handeye_transform')
REQUIRED_SOURCES = (
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/rich_plan_integrity.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/observation_path_guard.py',
    'src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py',
    'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/runner_contract.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_remote.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/silhouette_observation_planning.py',
    'src/alicia_grasp_modes/scripts/mode_aware_grasp_runner.py',
    'src/alicia_grasp_modes/scripts/mode_aware_grasp_task.py',
    'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py',
    'src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/tabletop.py')


class Transport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 3.
        return connection


def rpc(uri):
    return xmlrpc.client.ServerProxy(uri, transport=Transport())


def process_identity(pid):
    fields = (Path('/proc') / str(pid) / 'stat').read_text().rsplit(')', 1)[1].split()
    if fields[0] == 'Z':
        raise RuntimeError('process is a zombie: %s' % pid)
    return dict(pid=int(pid), start_ticks=fields[19], parent_pid=int(fields[1]))


def wait_until(check, timeout, reason):
    deadline = time.monotonic() + timeout
    while True:
        value = check()
        if value:
            return value
        if time.monotonic() >= deadline:
            raise RuntimeError(reason)
        time.sleep(.2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path,
        default=SESSION / 'verification/independent_observation_combined_final/validated_sources.json')
    parser.add_argument('--output-dir', type=Path, default=DEFAULT_OUT)
    operation = parser.add_mutually_exclusive_group(required=True)
    operation.add_argument('--execute', action='store_true')
    operation.add_argument('--prepare-only', action='store_true',
        help='read-only live preflight; never calls request_plan, shutdown, reset or systemd-run')
    args = parser.parse_args()
    out = args.output_dir.resolve()
    assert SESSION.resolve() in out.parents, 'output must be a new directory under this external session'
    assert Path('/media/zhuyupei/SanStand').is_mount(), 'SanStand unavailable'
    validation_bytes = args.manifest.read_bytes()
    validation = json.loads(validation_bytes)
    assert validation['passed'] is True
    assert set(REQUIRED_SOURCES).issubset(validation['source_sha256'])

    def source_check():
        assert args.manifest.read_bytes() == validation_bytes, 'validation manifest changed'
        for relative, digest in validation['source_sha256'].items():
            path = (WORK / relative).resolve()
            assert WORK == path or WORK in path.parents, 'source outside workspace'
            assert hashlib.sha256(path.read_bytes()).hexdigest() == digest, relative

    source_check()
    out.mkdir(exist_ok=True)
    result_path = out / ('prepare_result.json' if args.prepare_only else 'result.json')
    assert not (out / 'result.json').exists(), 'previous result exists; do not repeat deployment'
    assert not result_path.exists(), 'previous preflight result exists; use a new output directory'
    os.environ['ROS_LOG_DIR'] = str(out / ('prepare_ros' if args.prepare_only else 'observer_ros'))
    rospy.init_node('load_independent_observation_consumers', anonymous=True, disable_rosout=True)
    master = rosgraph.Master(rospy.get_name())

    def proxy(name):
        return rpc(master.lookupNode(name))

    def identity(name):
        response = proxy(name).getPid(rospy.get_name())
        assert response[0] == 1
        return process_identity(response[2])

    def idle():
        state = rospy.wait_for_message('/grasp/state', GraspState, timeout=5.)
        assert not state.active, 'task active'
        status = json.loads(rospy.wait_for_message('/grasp_mode/status', String, timeout=5.).data)
        assert status.get('active') is False and status.get('start_reserved') is False and not status.get('pending')
        assert rospy.get_param('/gui/joint_direct_mode') is False
        assert not rospy.get_param('/rgb_acquisition/ledger', {}), 'RGB acquisition ledger not empty'

    def no_active_goals():
        published = dict(master.getPublishedTopics('/'))
        required = ('/alicia_controller/follow_joint_trajectory/status',
                    '/hand_controller/follow_joint_trajectory/status')
        assert all(topic in published for topic in required), 'controller action status unavailable'
        topics = sorted(topic for topic, kind in published.items()
                        if kind == 'actionlib_msgs/GoalStatusArray')
        result = {}
        for topic in topics:
            message = rospy.wait_for_message(topic, GoalStatusArray, timeout=4.)
            assert 0. <= (rospy.Time.now() - message.header.stamp).to_sec() < 2., topic
            states = [row.status for row in message.status_list]
            assert not any(value in (0, 1, 6, 7) for value in states), (topic, states)
            result[topic] = states
        return result

    def measured():
        message = rospy.wait_for_message('/alicia_d/accepted_joint_states', JointState, timeout=5.)
        assert message.header.frame_id == 'sdk_measured'
        assert 0. <= (rospy.Time.now() - message.header.stamp).to_sec() < .2
        return dict(names=list(message.name), positions=list(message.position))

    def epoch():
        message = rospy.wait_for_message('/alicia_d/control_reference_epoch', Header, timeout=5.)
        assert message.frame_id == 'sdk_reference_epoch'
        return message.stamp.to_nsec()

    def owners(before):
        verified = {}
        diagnostics = []
        result['owner_diagnostics'] = diagnostics
        uris = rospy.get_param('/roslaunch/uris', {})
        parents = {row['parent_pid'] for row in before.values()}
        prefixes = tuple(node.lstrip('/') + '-' for node in TARGETS)
        for uri in uris.values():
            diagnostic = dict(uri=uri)
            diagnostics.append(diagnostic)
            try:
                owner = rpc(uri)
                reply = owner.get_pid()
                assert reply[0] == 1, reply[:2]
                parent = int(reply[2])
                diagnostic['parent_pid'] = parent
                if parent not in parents:
                    diagnostic['not_a_target_parent'] = True
                    continue
                reply = owner.list_processes()
                assert reply[0] == 1, reply[:2]
                candidates = [name for name, _count in reply[2][0]
                              if isinstance(name, str) and name.startswith(prefixes)]
                diagnostic['target_process_names'] = candidates
                # ROS master process_info includes a builtin method that its
                # XML-RPC server cannot marshal. Unrelated processes must not
                # prevent verification of the gateway in the same parent.
                for name in candidates:
                    try:
                        reply = owner.process_info(name)
                        assert reply[0] == 1, reply[:2]
                        info = reply[2]
                    except (OSError, xmlrpc.client.Error, AssertionError) as exc:
                        diagnostic.setdefault('process_errors', {})[name] = repr(exc)
                        continue
                    for node, child in before.items():
                        if (int(info.get('pid', -1)) == child['pid']
                                and parent == child['parent_pid']):
                            # Never persist the full pmon environment.
                            verified[node] = dict(uri=uri, parent=process_identity(parent),
                                process={key: info[key] for key in (
                                    'pid', 'name', 'alive', 'respawn', 'respawn_delay', 'spawn_count', 'args')})
            except (OSError, xmlrpc.client.Error, ValueError, TypeError, AssertionError) as exc:
                diagnostic['error'] = repr(exc)
                continue
        missing = sorted(set(TARGETS) - set(verified))
        assert not missing, 'unverified node owners: %s; diagnostics=%s' % (
            missing, json.dumps(diagnostics, ensure_ascii=False))
        assert verified[GATEWAY]['process']['respawn'] is True
        assert all(verified[node]['process']['respawn'] is False for node in NEW_LAUNCH_NODES)
        assert all(row['process']['alive'] is True for row in verified.values())
        return verified

    result = dict(deployment_passed=False, restarted_nodes=list(TARGETS),
        prepare_only=args.prepare_only, prepare_passed=False,
        shutdown_requests_sent=[], new_launch_started=False,
        motion_commands_sent=False, joint_disable_commands_sent=False, temperature_queried=False,
        validation_manifest=str(args.manifest), validated_source_sha256=validation['source_sha256'])
    try:
        idle()
        result['action_status_before'] = no_active_goals()
        registered = set()
        for group in master.getSystemState():
            for _topic, names in group:
                registered.update(names)
        preserve = {}
        for name in sorted(registered - set(TARGETS) - {rospy.get_name()}):
            try:
                preserve[name] = identity(name)
            except Exception:
                if name in REQUIRED_PRESERVE:
                    raise
        assert set(REQUIRED_PRESERVE).issubset(preserve)
        before = {name: identity(name) for name in TARGETS}
        result.update(before=before, preserved_before=preserve)
        ownership = owners(before)
        selection = rospy.get_param('/grasp_mode/selection')
        q0, e0 = measured(), epoch()
        result.update(before=before, preserved_before=preserve, ownership=ownership,
            selection=selection, measured_before=q0, epoch_before=e0)
        if args.prepare_only:
            source_check(); idle(); no_active_goals()
            assert {name: identity(name) for name in TARGETS} == before
            assert {name: identity(name) for name in preserve} == preserve
            assert rospy.get_param('/grasp_mode/selection') == selection
            assert epoch() == e0 and measured() == q0
            result.update(prepare_passed=True, after=before,
                measured_after=q0, epoch_after=e0,
                mutation_services_called=False, shutdown_requests_sent=[], new_launch_started=False)
            return
        (out / 'before.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
        response = rospy.ServiceProxy('/grasp_6d/request_plan', TriggerZero)(False)
        assert response.success, response.message
        idle(); no_active_goals(); source_check()

        def invariant_check():
            assert rospy.get_param('/grasp_mode/selection') == selection
            assert epoch() == e0 and measured() == q0, 'reference or posture changed'
            for name, previous in preserve.items():
                assert identity(name) == previous, 'preserved node changed: ' + name

        def shutdown(name):
            invariant_check(); no_active_goals(); source_check()
            assert identity(name) == before[name], 'shutdown identity changed'
            try:
                result['shutdown_requests_sent'].append(name)
                answer = proxy(name).shutdown(rospy.get_name(), 'load validated independent observation while idle')
                assert answer[0] == 1, answer
            except (OSError, xmlrpc.client.ProtocolError) as exc:
                result.setdefault('shutdown_transport', {})[name] = repr(exc)
            def exited():
                try:
                    return process_identity(before[name]['pid']) != before[name]
                except (OSError, RuntimeError):
                    return True
            wait_until(exited, 15., 'old process did not exit: ' + name)

        for name in NEW_LAUNCH_NODES:
            shutdown(name)
        shutdown(GATEWAY)
        owner = ownership[GATEWAY]
        def gateway_respawned():
            try:
                assert process_identity(owner['parent']['pid']) == owner['parent']
                current = identity(GATEWAY)
                info = rpc(owner['uri']).process_info(owner['process']['name'])[2]
                if (current != before[GATEWAY] and current['parent_pid'] == owner['parent']['pid']
                        and info['pid'] == current['pid'] and info['alive'] is True
                        and info['respawn'] is True
                        and info['spawn_count'] == owner['process']['spawn_count'] + 1):
                    return current
            except (OSError, xmlrpc.client.Error, rosgraph.MasterError, RuntimeError):
                return False
            return False
        result['respawned_gateway'] = wait_until(gateway_respawned, 30., 'gateway did not respawn exactly once')
        invariant_check(); no_active_goals(); source_check()
        command = ['systemd-run', '--user', '--collect', '--unit=alicia-independent-observation-consumers-20261008',
            '--property=Type=exec', '--property=Restart=no', '--property=WorkingDirectory=' + str(WORK),
            '--property=StandardOutput=append:' + str(out / 'consumers.log'),
            '--property=StandardError=append:' + str(out / 'consumers.log'),
            '--setenv=ROS_LOG_DIR=' + str(out / 'consumers_ros'), '/bin/bash',
            str(WORK / '.runtime/start_independent_observation_consumers_20261008.sh')]
        result['start_command'] = command
        subprocess.run(command, check=True, timeout=15.)
        result['new_launch_started'] = True
        def available():
            try:
                return all(identity(name) != before[name] for name in NEW_LAUNCH_NODES)
            except Exception:
                return False
        wait_until(available, 45., 'new consumers unavailable')
        for service in ('/grasp_mode/reset_task', '/grasp_mode/reset_remote'):
            rospy.wait_for_service(service, timeout=20.)
            response = rospy.ServiceProxy(service, Trigger)()
            assert response.success, (service, response.message)
            result.setdefault('reset_ack', {})[service] = response.message
        idle(); invariant_check(); source_check()
        result['action_status_after'] = no_active_goals()
        result.update(deployment_passed=True, after={name: identity(name) for name in TARGETS},
            preserved_after={name: identity(name) for name in preserve},
            measured_after=measured(), epoch_after=epoch())
    except Exception as exc:
        result['error'] = repr(exc)
        raise
    finally:
        result_path.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
        print(json.dumps(result, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
