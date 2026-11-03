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
    parser.add_argument('--failed', type=Path, required=True)
    parser.add_argument('--preflight', type=Path, required=True)
    args = parser.parse_args()
    work = Path(__file__).resolve().parents[1]
    mount = Path('/media/zhuyupei/SanStand')
    assert mount.is_mount() and mount in args.output.resolve().parents
    manifest_bytes = args.manifest.read_bytes()
    manifest = json.loads(manifest_bytes)
    assert manifest['passed'] is True and manifest['reviewed'] is True

    def sources():
        assert args.manifest.read_bytes() == manifest_bytes
        for relative, digest in manifest['source_sha256'].items():
            assert hashlib.sha256((work / relative).read_bytes()).hexdigest() == digest, relative
        for name, digest in manifest['evidence_sha256'].items():
            assert hashlib.sha256(Path(name).read_bytes()).hexdigest() == digest, name

    sources()
    failed_bytes, preflight_bytes = args.failed.read_bytes(), args.preflight.read_bytes()
    failed, preflight = json.loads(failed_bytes), json.loads(preflight_bytes)
    assert failed['deployment_passed'] is False and 'unit' not in failed
    assert failed['source_sha256'] == manifest['source_sha256']
    assert preflight['passed'] is True
    assert preflight['source_manifest_sha256'] == hashlib.sha256(manifest_bytes).hexdigest()
    assert {**failed['before'], **failed['preserved_before']} == preflight['node_identities']
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

    def identity(name, endpoint=None):
        code, _, pid = (endpoint if endpoint is not None else rpc(name)).getPid(rospy.get_name())
        assert code == 1
        stat = (Path('/proc') / str(pid) / 'stat').read_text().rsplit(')', 1)[1].split()
        assert stat[0] != 'Z'
        return dict(pid=pid, start_ticks=stat[19], parent_pid=int(stat[1]))

    def old_process_alive(previous):
        path = Path('/proc') / str(previous['pid']) / 'stat'
        try:
            stat = path.read_text().rsplit(')', 1)[1].split()
        except FileNotFoundError:
            return False
        return stat[0] != 'Z' and stat[19] == previous['start_ticks']

    def pinned_inputs():
        sources()
        assert args.failed.read_bytes() == failed_bytes
        assert args.preflight.read_bytes() == preflight_bytes
        assert rospy.get_param('/grasp_mode/selection') == preflight['selection']

    def manual_idle(require_task=True):
        assert rospy.get_param('/gui/joint_direct_mode') is True
        status = json.loads(rospy.wait_for_message('/grasp_mode/status', String, timeout=5).data)
        assert not (status.get('active') or status.get('start_reserved') or status.get('pending'))
        if require_task:
            assert not rospy.wait_for_message('/grasp/state', GraspState, timeout=5).active

    result = dict(deployment_passed=False, source_sha256=manifest['source_sha256'],
                  motion_commands_sent=False, torque_commands_sent=False, temperature_queried=False,
                  manual_motion_allowed=True, new_selection_required_before_automatic_grasp=True)

    result.update(recovery_from=str(args.failed.resolve()),
        recovery_from_sha256=hashlib.sha256(failed_bytes).hexdigest(),
        recovery_preflight=str(args.preflight.resolve()),
        recovery_helper=str(Path(__file__).resolve()),
        recovery_helper_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        shutdown_reply_errors={})

    def save():
        (args.output / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')

    try:
        pinned_inputs()
        manual_idle(require_task=False)
        assert set(failed['before']) == set(targets)
        result.update(before=failed['before'],
                      preserved_before={name: identity(name) for name in preserve})
        assert result['preserved_before'] == failed['preserved_before']
        assert rospy.get_param('/grasp_mode/selection') == preflight['selection']
        save()
        for name in targets:
            previous = result['before'][name]
            if not old_process_alive(previous):
                # A vanished old process is not sent another shutdown. A
                # responsive replacement must not be overwritten silently.
                try:
                    current = identity(name)
                except (OSError, xmlrpc.client.Error, rosgraph.MasterError):
                    current = None
                assert current is None or current == previous, name
                result.setdefault('already_exited', []).append(name)
                save()
                continue
            endpoint = rpc(name)
            assert identity(name, endpoint=endpoint) == previous
            try:
                code, message, _ = endpoint.shutdown(rospy.get_name(),
                    'resume tested compute reload while GUI owns control')
            except (OSError, xmlrpc.client.Error) as exc:
                # Never infer success from a failed reply. The bounded old
                # PID/start-tick wait below must prove all writers exited.
                result['shutdown_reply_errors'][name] = repr(exc)
            else:
                assert code == 1, message
                result.setdefault('shutdown_requests_sent', []).append(name)
            save()
        deadline = time.monotonic() + 25.
        while time.monotonic() < deadline:
            remaining = []
            for name, previous in result['before'].items():
                if old_process_alive(previous):
                    remaining.append(name)
            if not remaining:
                break
            time.sleep(.2)
        else:
            raise RuntimeError('old consumers did not exit: ' + repr(remaining))
        manual_idle(require_task=False)
        assert {name: identity(name) for name in preserve} == result['preserved_before']
        pinned_inputs()
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
        pinned_inputs()
        assert rospy.get_param('/grasp_mode/selection') == preflight['selection']
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
