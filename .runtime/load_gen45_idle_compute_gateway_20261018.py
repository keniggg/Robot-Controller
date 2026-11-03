"""Reload GEN45's four idle geometry consumers, preserving seven hardware/GUI nodes.

The existing core launcher alone respawns the gateway. The new compute launch
explicitly disables its optional gateway, preventing a duplicate instance.
Only idle consumer shutdown and the existing mode-reset services are invoked.
This helper sends no motion, controller-mode, torque or temperature commands.
"""
import argparse
import os
import subprocess
import time
import xmlrpc.client
from gen45_idle_reload_common_20261018 import *


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--preflight', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve(); storage(output)
    require(not output.exists(), 'output already exists')
    release = validate_release(args.manifest)
    preflight = read(args.preflight)
    require(preflight['passed'] is True and preflight['source_manifest_sha256'] == sha(args.manifest), 'preflight is not accepted for this release')
    require(preflight['previous_deployment_sha256'] == PREVIOUS_SHA, 'preflight previous GEN44 identity mismatch')
    _, expected = validate_previous(Path(preflight['previous_deployment']))
    require(preflight['node_identities'] == expected, 'preflight node set differs from accepted GEN44')
    control_path = Path(preflight['control_path'])
    require(validate_control(control_path) == preflight['control_snapshot'] and sha(control_path) == preflight['control_sha256'], 'control differs from preflight')
    pinned = {str(path.resolve()): path.read_bytes() for path in (args.manifest, args.preflight, control_path, PREVIOUS)+HELPERS}
    require(preflight['helper_sha256'] == {str(path): sha(path) for path in HELPERS}, 'helpers differ from preflight')
    output.mkdir()
    os.environ['ROS_LOG_DIR'] = str(output/'observer_ros')
    import rosgraph
    import rospy
    from actionlib_msgs.msg import GoalStatusArray
    from std_msgs.msg import Header, String
    from std_srvs.srv import Trigger
    from alicia_flexible_grasp_supervisor.msg import GraspState
    from alicia_grasp_modes.selection import parse_selection
    rospy.init_node('load_gen45_idle_compute_gateway', anonymous=True, disable_rosout=True)
    master = rosgraph.Master(rospy.get_name())
    def rpc(name):
        return xmlrpc.client.ServerProxy(master.lookupNode(name), transport=Transport())
    def identity(name, endpoint=None):
        code, _, pid = (endpoint if endpoint is not None else rpc(name)).getPid(rospy.get_name())
        require(code == 1, 'node PID unavailable: '+name)
        return process(pid)
    def pinned_inputs():
        validate_sources(release)
        require(all(Path(path).read_bytes() == raw for path, raw in pinned.items()), 'input/helper bytes changed')
        require(rospy.get_param('/grasp_mode/selection') == preflight['selection'], 'mode selection/generation changed')
        require(rospy.wait_for_message('/alicia_d/control_reference_epoch', Header, timeout=5.).stamp.to_nsec() == preflight['epoch_ns'], 'control epoch changed')
        require({name: rospy.get_param(name) for name in PARAMETERS} == preflight['runtime_parameters'], 'runtime parameters changed')
    def idle(require_task=True):
        require(rospy.get_param('/gui/joint_direct_mode') is True, 'GUI is not manual')
        mode = json.loads(rospy.wait_for_message('/grasp_mode/status', String, timeout=5.).data)
        require(not any(mode.get(k) for k in ('active', 'start_reserved', 'pending')), 'router busy')
        if require_task:
            require(not rospy.wait_for_message('/grasp/state', GraspState, timeout=5.).active, 'task active')
        for topic in ('/alicia_controller/follow_joint_trajectory/status', '/hand_controller/follow_joint_trajectory/status'):
            msg = rospy.wait_for_message(topic, GoalStatusArray, timeout=5.)
            require(not any(goal.status in (0, 1, 6, 7) for goal in msg.status_list), 'active action')
    def preserve():
        require({name: identity(name) for name in PRESERVE} == {name: expected[name] for name in PRESERVE}, 'preserved hardware/GUI process changed')
    def bounded_call(service):
        # Run only the existing two idle reset callbacks. No execution services.
        import threading
        done, values = threading.Event(), []
        def invoke():
            try:
                values.append(rospy.ServiceProxy(service, Trigger)())
            except Exception as exc:
                values.append(exc)
            finally:
                done.set()
        thread = threading.Thread(target=invoke, daemon=True)
        thread.start()
        require(done.wait(45.), 'bounded reset acknowledgment timed out: '+service)
        require(len(values) == 1 and not isinstance(values[0], Exception), 'reset failed: '+repr(values))
        reply = values[0]
        require(reply.success is True, service+': '+reply.message)
        ack = json.loads(reply.message)
        require(ack.get('ready') is True and parse_selection(ack) == parse_selection(preflight['selection']), 'reset ACK selection/generation differs: '+service)
        return dict(success=True, ready=True, selection=parse_selection(ack), raw_selection=ack, message=reply.message)
    result = dict(deployment_passed=False, whole_patch_deployed=False,
        deployment_kind='idle_three_compute_plus_original_parent_gateway_respawn_seven_preserved',
        source_manifest=str(args.manifest.resolve()), source_manifest_sha256=sha(args.manifest),
        source_sha256=release['source_sha256'], test_summary=release['test_summary'],
        previous_deployment=str(PREVIOUS), previous_deployment_sha256=PREVIOUS_SHA,
        deployment_preflight=str(args.preflight.resolve()), deployment_preflight_sha256=sha(args.preflight),
        input_sha256={path: hashlib.sha256(raw).hexdigest() for path, raw in pinned.items()},
        helper_sha256={str(path): sha(path) for path in HELPERS},
        control_path=str(control_path), control_snapshot=preflight['control_snapshot'], control_sha256=preflight['control_sha256'],
        selection=preflight['selection'], epoch_ns=preflight['epoch_ns'],
        motion_commands_sent=False, torque_disable_commands_sent=False, temperature_queried=False,
        hardware_processes_preserved=False, scene_permission=False,
        new_selection_required_before_automatic_grasp=True, shutdown_requests=[])
    def save():
        write(output/'result.json', result)
    try:
        pinned_inputs(); idle()
        require({name: identity(name) for name in expected} == expected, 'GEN44 identities changed since preflight')
        result.update(before={name: expected[name] for name in TARGETS}, preserved_before={name: expected[name] for name in PRESERVE}, feedback_before=feedback(rospy))
        require(result['feedback_before']['epoch_ns'] == preflight['epoch_ns'], 'feedback epoch changed')
        gateway_parent = process(expected['/motion_gateway']['parent_pid'])
        require(gateway_parent == preflight['gateway_original_parent'], 'gateway launcher identity changed')
        result['gateway_respawn_parent'] = gateway_parent
        save()
        for name in TARGETS:
            pinned_inputs(); idle(require_task=False); preserve()
            endpoint = rpc(name)
            require(identity(name, endpoint) == expected[name], 'target changed before shutdown: '+name)
            row = dict(name=name, requested_ns=time.time_ns(), attempted=True)
            result['shutdown_requests'].append(row); save()
            try:
                code, message, _ = endpoint.shutdown(rospy.get_name(), 'load reviewed GEN45 top-mask contract while manual/idle')
            except (OSError, xmlrpc.client.Error) as exc:
                # A transport error does not prove exit; exact PID/start ticks
                # must disappear before any replacement launch is attempted.
                row['reply_error'] = repr(exc)
            else:
                require(code == 1, 'shutdown rejected: '+str(message))
                row.update(return_code=code, message=message)
            save()
        deadline = time.monotonic()+25.
        while time.monotonic() < deadline and not all(exited(row) for row in result['before'].values()):
            time.sleep(.1)
        require(all(exited(row) for row in result['before'].values()), 'old consumers remain; no retry or duplicate launch')
        result['old_processes_exited'] = True
        pinned_inputs(); idle(require_task=False); preserve()
        # Original bringup owns respawn=true. Never launch another gateway.
        rospy.wait_for_service('/supervisor/check_contact_sequence', timeout=40.)
        gateway = identity('/motion_gateway')
        require(gateway != expected['/motion_gateway'] and gateway['parent_pid'] == gateway_parent['pid'], 'gateway was not respawned by original parent')
        require(process(gateway_parent['pid']) == gateway_parent, 'original gateway launcher changed')
        result['gateway_after_respawn'] = gateway
        pinned_inputs(); idle(require_task=False); preserve(); save()
        unit = 'alicia-gen45-compute-' + str(time.time_ns())
        command = ['systemd-run', '--user', '--collect', '--unit='+unit,
            '--property=Type=exec', '--property=Restart=no', '--property=WorkingDirectory='+str(W),
            '--property=StandardOutput=append:'+str(output/'consumers.log'),
            '--property=StandardError=append:'+str(output/'consumers.log'),
            '--setenv=ROS_LOG_DIR='+str(output/'consumers_ros'), '/bin/bash',
            str(W/'.runtime/start_gen45_observation_measurement_consumers_20261018.sh')]
        result.update(unit=unit, command=command, new_launch_attempted=True, gateway_started_by_compute_launch=False); save()
        subprocess.run(command, check=True, timeout=15.)
        for service in ('/grasp_mode/reset_remote', '/grasp_mode/reset_task'):
            rospy.wait_for_service(service, timeout=45.)
            pinned_inputs(); idle(require_task=False); preserve()
            result.setdefault('reset_ack', {})[service] = bounded_call(service)
            save()
        for service in GATEWAY_SERVICES:
            rospy.wait_for_service(service, timeout=15.)
        idle(); pinned_inputs(); preserve()
        after = {name: identity(name) for name in TARGETS}
        require(all(after[name] != expected[name] for name in TARGETS), 'old consumer process reused')
        parent = int(subprocess.check_output(['systemctl', '--user', 'show', unit, '--property=MainPID', '--value'], text=True, timeout=5.).strip())
        require(parent > 1 and all(after[name]['parent_pid'] == parent for name in COMPUTE), 'new compute parent mismatch')
        require(after['/motion_gateway'] == gateway and process(gateway_parent['pid']) == gateway_parent, 'accepted gateway/parent changed')
        providers = {name: list(nodes) for name, nodes in master.getSystemState()[2]}
        require(all(providers.get(service) == ['/motion_gateway'] for service in GATEWAY_SERVICES), 'gateway service owner mismatch')
        require(providers.get('/grasp_mode/reset_remote') == ['/remote_grasp6d_node'] and providers.get('/grasp_mode/reset_task') == ['/grasp_task_node'], 'reset service owner mismatch')
        sdk = feedback(rospy)
        require(sdk['epoch_ns'] == preflight['epoch_ns'], 'SDK epoch changed after reload')
        require(sdk['names'] == result['feedback_before']['names'], 'SDK joint order changed')
        require(max(abs(a-b) for a, b in zip(sdk['positions'], result['feedback_before']['positions'])) <= .004, 'robot moved during idle reload')
        idle(); pinned_inputs(); preserve()
        require({name: identity(name) for name in TARGETS} == after, 'new consumers changed during acceptance')
        result.update(after=after, preserved_after={name: identity(name) for name in PRESERVE},
            feedback_after=sdk, gateway_service_owners={name: providers[name] for name in GATEWAY_SERVICES},
            replaced_count=4, preserved_count=7, manual_mode=True, task_active=False, actions_active=False,
            control_unchanged=True, selection_unchanged=True, epoch_unchanged=True,
            gateway_original_parent_preserved=True, gateway_reloaded=True, compute_reloaded=True,
            hardware_processes_preserved=True, ready_ack_scope='remote/task explicit ready JSON; gateway registered services owned by new process; perception new process under compute unit',
            deployment_passed=True, whole_patch_deployed=True, recorded_ns=time.time_ns())
    except Exception as exc:
        result['error'] = repr(exc)
        raise
    finally:
        save()
        print(json.dumps(dict(deployment_passed=result['deployment_passed'], output=str(output), error=result.get('error')), ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
