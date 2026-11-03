#!/usr/bin/env python3
"""Read-only installed-release one-view cold-start acceptance; evidence stays on SanStand.

Never starts/stops a service, publishes control, resets a node, changes parameters,
or subscribes to temperature/actuation diagnostics. Run only after root startup.
"""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import threading
import time
import xmlrpc.client

W = Path(__file__).resolve().parents[1]
MOUNT = Path('/media/zhuyupei/SanStand')
EXT = MOUNT / '实验记录/Robot-Controller'
COMPUTE = ('/grasp_task_node', '/remote_grasp6d_node', '/unknown_tabletop_perception')
HARDWARE = ('/alicia_d_driver_node', '/bessica_d_hw_interface', '/move_group',
            '/motion_gateway', '/supervisor_camera', '/compliant_gripper_controller',
            '/grasp_mode_router', '/alicia_supervisor_gui')
NODES = COMPUTE + HARDWARE
NODE_ENTRY = {
    '/grasp_task_node': ('alicia_grasp_modes', 'mode_aware_grasp_task.py'),
    '/remote_grasp6d_node': ('alicia_grasp_modes', 'mode_aware_remote_grasp6d.py'),
    '/unknown_tabletop_perception': ('alicia_grasp_modes', 'unknown_tabletop_perception.py'),
    '/alicia_d_driver_node': ('alicia_d_driver', 'alicia_d_driver_node'),
    '/bessica_d_hw_interface': ('alicia_d_driver', 'alicia_d_hw_interface_node'),
    '/motion_gateway': ('alicia_flexible_grasp_supervisor', 'motion_gateway_node.py'),
    '/supervisor_camera': ('alicia_flexible_grasp_supervisor', 'camera_node.py'),
    '/compliant_gripper_controller': ('alicia_flexible_grasp_supervisor', 'compliant_gripper_controller.py'),
    '/grasp_mode_router': ('alicia_grasp_modes', 'mode_router.py'),
    '/alicia_supervisor_gui': ('alicia_grasp_modes', 'mode_aware_gui.py'),
}
DRIVER_BUILD = EXT / 'ros_log/pending_actuation_drift_20261007T104410.504268Z/driver_build.json'


def require(value, reason):
    if not value:
        raise RuntimeError(reason)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def source_check(source):
    require(bool(source), 'release source map empty')
    for relative, expected in source.items():
        p = Path(relative)
        require(not p.is_absolute() and '..' not in p.parts, 'invalid source path: ' + relative)
        require(sha(W / p) == expected, 'source hash changed: ' + relative)


def process(pid):
    p = Path('/proc') / str(pid)
    fields = (p / 'stat').read_text().rsplit(')', 1)[1].split()
    require(fields[0] != 'Z', 'zombie PID: ' + str(pid))
    return dict(pid=int(pid), start_ticks=fields[19], parent_pid=int(fields[1]))


def properties(unit):
    names = ('ActiveState', 'SubState', 'MainPID', 'InvocationID', 'ControlGroup')
    raw = subprocess.check_output(['systemctl', '--user', 'show', unit,
        *('--property=' + n for n in names)], text=True, timeout=3.)
    result = dict(line.split('=', 1) for line in raw.splitlines() if '=' in line)
    require(result['ActiveState'] == 'active' and result['SubState'] == 'running',
            'service is not running: ' + unit)
    require(int(result['MainPID']) > 1 and len(result['InvocationID']) == 32,
            'service identity missing: ' + unit)
    return result


def in_group(pid, group):
    return any(line.rsplit(':', 1)[-1] == group or
               line.rsplit(':', 1)[-1].startswith(group + '/')
               for line in (Path('/proc') / str(pid) / 'cgroup').read_text().splitlines())


def gui_windows(pid):
    env = dict(os.environ, DISPLAY=':0', XAUTHORITY='/run/user/1000/gdm/Xauthority')
    raw = subprocess.check_output(['xprop', '-root', '_NET_CLIENT_LIST'],
                                  env=env, text=True, timeout=3.)
    found = []
    for window in re.findall(r'0x[0-9a-fA-F]+', raw):
        details = subprocess.check_output(['xprop', '-id', window, '_NET_WM_PID'],
                                         env=env, text=True, timeout=3.)
        match = re.search(r'=\s*(\d+)\s*$', details)
        if match and int(match.group(1)) == pid:
            found.append(window)
    require(found, 'GUI has no mapped X11 client window')
    return found


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--release', required=True, type=Path)
    parser.add_argument('--release-sha256', required=True)
    parser.add_argument('--expected-generation', required=True)
    parser.add_argument('--expected-control-revision', required=True, type=int)
    parser.add_argument('--request-sha256', required=True)
    parser.add_argument('--previous-deployment', required=True, type=Path)
    parser.add_argument('--previous-deployment-sha256', required=True)
    parser.add_argument('--core-unit', required=True)
    parser.add_argument('--simulation-unit', required=True)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    args.output = args.output.resolve()
    require(MOUNT.is_mount() and (EXT / 'ros_log') in args.output.parents,
            'SanStand mounted ros_log output required; no fallback')
    require(args.output.parent.exists() and args.output.parent.stat().st_dev == MOUNT.stat().st_dev,
            'output parent is not on SanStand')
    args.output.mkdir(exist_ok=False)
    os.environ['ROS_LOG_DIR'] = str(args.output / 'observer_ros')
    sys.dont_write_bytecode = True
    result = dict(deployment_passed=False, whole_patch_deployed=False,
        full_restart=True, all_processes_replaced=False,
        deployment_kind='full_cold_restart_sealed_release_after_actual_user_request',
        preserved_after={}, preserved_before={},
        process_groups_semantics='after contains all newly started nodes; no old process preservation claimed',
        dedicated_compute_reloader_compatible=False,
        motion_commands_sent=False, torque_commands_sent=False, temperature_queried=False,
        parameters_changed=False, services_started_by_verifier=False,
        new_selection_required_before_automatic_grasp=True, physical_grasp_verified=False,
        verifier=str(Path(__file__).resolve()), verifier_sha256=sha(__file__))
    subscriptions = []
    try:
        require(sha(args.release) == args.release_sha256, 'release digest changed')
        require(sha(args.previous_deployment) == args.previous_deployment_sha256, 'previous deployment digest changed')
        startup_path = args.output.parent/'startup.json'
        startup_receipt_sha256 = sha(startup_path)
        startup_receipt = read(startup_path)
        request_path = args.output.parent/'operator_restart_instruction.json'
        require(sha(request_path) == args.request_sha256 == startup_receipt['user_instruction_sha256'],
                'actual restart request digest differs from startup')
        request = read(request_path)
        require(request.get('source') == 'actual_user_message' and request.get('full_restart_authorized') is True
                and isinstance(request.get('user_message'), str) and bool(request['user_message'].strip()),
                'actual full restart request missing')
        require(startup_receipt['source_manifest_sha256'] == args.release_sha256
                and startup_receipt.get('generation') == args.expected_generation
                and startup_receipt.get('full_restart') is True, 'startup release/scope differs')
        require(startup_receipt['core']['unit'].replace('.service', '') == args.core_unit.replace('.service', '')
                and startup_receipt['simulation']['unit'].replace('.service', '') == args.simulation_unit.replace('.service', ''),
                'requested services differ from startup receipt')
        require(all(sha(path) == digest for path, digest in startup_receipt['operational_source_sha256'].items()),
                'operational startup/verifier source changed')
        release, previous = read(args.release), read(args.previous_deployment)
        require(release.get('generation') == args.expected_generation
                and release.get('source_count') == len(release['source_sha256']), 'release generation/source count differs')
        require(all(release.get(k) is True for k in ('passed', 'reviewed', 'installed')),
                'release is not sealed/reviewed/installed')
        summary = release['test_summary']
        require(summary['tests'] > 0 and all(summary[k] == 0 for k in ('failures', 'errors', 'skipped')),
                'release regression failed or incomplete')
        source_check(release['source_sha256'])
        for path, expected in release['evidence_sha256'].items():
            require(sha(path) == expected, 'release evidence hash changed: ' + path)
        old = dict(previous['after'], **previous['preserved_after'])
        require(set(old) == set(NODES), 'previous deployment node scope differs')
        require(previous.get('deployment_passed') is True, 'previous deployment failed')
        result.update(source_manifest=str(args.release.resolve()), source_manifest_sha256=sha(args.release),
            source_sha256=release['source_sha256'], test_summary=summary,
            previous_deployment=str(args.previous_deployment.resolve()),
            previous_deployment_sha256=sha(args.previous_deployment), previous_nodes=old,
            unit=args.core_unit, simulation_unit=args.simulation_unit)
        core = properties(args.core_unit)
        result['core_service'] = core
        core_proc = Path('/proc') / core['MainPID']
        core_argv = (core_proc / 'cmdline').read_bytes().rstrip(b'\0').decode().split('\0')
        roslaunch = [i for i, value in enumerate(core_argv) if Path(value).name == 'roslaunch']
        require(len(roslaunch) == 1 and
                core_argv[roslaunch[0]+1:] == ['--skip-log-check', '.runtime/full_latest_gen36_single_observation_20261016.launch'] and
                (core_proc / 'cwd').resolve() == W, 'core is not the reviewed frozen startup entry')
        result['core_argv'] = core_argv
        import rosgraph
        import rospy
        import yaml
        from actionlib_msgs.msg import GoalStatusArray
        from controller_manager_msgs.srv import ListControllers
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool, Header, String
        from alicia_flexible_grasp_supervisor.msg import GraspState
        rospy.init_node('full_contact_restart_readonly', anonymous=True, disable_rosout=True)
        master = rosgraph.Master(rospy.get_name())
        master_pid = master.getPid()
        master_process = process(master_pid)
        require(master_process['parent_pid'] == int(core['MainPID'])
                and in_group(master_pid, core['ControlGroup']), 'master is not owned by the new core service')
        master_argv = (Path('/proc')/str(master_pid)/'cmdline').read_bytes().rstrip(b'\0').decode().split('\0')
        require(any(Path(arg).name == 'rosmaster' for arg in master_argv), 'unexpected master process entry')
        result.update(master_process=master_process, master_argv=master_argv,
                      generation=args.expected_generation, user_instruction_sha256=args.request_sha256)

        class Transport(xmlrpc.client.Transport):
            def make_connection(self, host):
                connection = super().make_connection(host)
                connection.timeout = 3.
                return connection

        def identities():
            rows = {}
            for name in NODES:
                uri = master.lookupNode(name)
                code, _, pid = xmlrpc.client.ServerProxy(uri, transport=Transport()).getPid(rospy.get_name())
                require(code == 1 and type(pid) is int and pid > 1, 'invalid registered node PID: ' + name)
                row = process(pid)
                require(row['parent_pid'] == int(core['MainPID']) and in_group(pid, core['ControlGroup']),
                        'node not owned by requested core: ' + name)
                require(row != old[name], 'old node identity reused: ' + name)
                argv = (Path('/proc') / str(pid) / 'cmdline').read_bytes().rstrip(b'\0').decode().split('\0')
                if name in NODE_ENTRY:
                    package, entry = NODE_ENTRY[name]
                    allowed = {str(W / 'devel/lib' / package / entry),
                               str(W / 'src' / package / 'scripts' / entry)}
                    require(len([arg for arg in argv if arg in allowed]) == 1,
                            'node entry not in actual worktree: ' + name)
                else:
                    require('/opt/ros/noetic/lib/moveit_ros_move_group/move_group' in argv,
                            'unexpected MoveIt node entry')
                rows[name] = row
            require(len({r['pid'] for r in rows.values()}) == len(NODES), 'nonunique node PID')
            matches = {name: [] for name in NODES}
            for proc in Path('/proc').iterdir():
                if not proc.name.isdigit():
                    continue
                try:
                    argv = (proc / 'cmdline').read_bytes().split(b'\0')
                    for name in NODES:
                        if ('__name:=' + name[1:]).encode() in argv or ('__name:=' + name).encode() in argv:
                            matches[name].append(int(proc.name))
                except (FileNotFoundError, ProcessLookupError):
                    continue
                except PermissionError:
                    # Other users' processes cannot own this user's ROS service cgroup.
                    continue
            for name, row in rows.items():
                require(matches[name] == [row['pid']], 'duplicate or mismatched ROS process: ' + name)
            return rows

        nodes = identities()
        result.update(after=nodes, compute_after={n:nodes[n] for n in COMPUTE},
                      hardware_after={n:nodes[n] for n in HARDWARE})
        registered = set()
        # Master state lists contain node names but no diagnostic payloads.
        for group in master.getSystemState():
            for _, names in group:
                registered.update(names)
        require(not any('tactile_skin' in n for n in registered), 'unexpected tactile node')
        registered_processes = {}
        for name in sorted(registered - {rospy.get_name()}):
            uri = master.lookupNode(name)
            code, _, pid = xmlrpc.client.ServerProxy(uri, transport=Transport()).getPid(rospy.get_name())
            require(code == 1 and type(pid) is int and pid > 1, 'registered node PID unavailable: '+name)
            require(in_group(pid, core['ControlGroup']), 'registered node belongs to another stack: '+name)
            registered_processes[name] = process(pid)
        require(set(NODES) <= set(registered_processes), 'critical nodes missing from master graph')
        result['all_registered_nodes'] = registered_processes
        result['registered_nodes_scope'] = 'master publisher/subscriber/service graph, excluding this read-only verifier; every process belongs to the newly created core service group'
        expected = {'/gui/joint_direct_mode': True,
            '/grasp/unknown_observe_return_max_observations': 1,
            '/grasp/unknown_observe_return_return_to_origin': False,
            '/grasp/unknown_lift_after_close_enabled': False,
            '/alicia_d_driver_node/software_temperature_monitoring_enabled': False,
            '/alicia_d_driver_node/temperature_poll_rate_hz': 0.0,
            '/alicia_d_driver_node/self_check_poll_rate_hz': 0.0,
            '/alicia_d_driver_node/auto_torque_on_startup': True,
            '/camera/fps': 15, '/camera/publish_fps': 15.0,
            '/unknown_perception/detect_hz': 6.0,
            '/unknown_perception/support_max_local_shift_m': 0.007,
            '/unknown_perception/support_local_shift_warning_m': 0.004,
            '/grasp_6d/remote/multiview/maximum_support_offset_delta_m': 0.007,
            '/grasp_6d/remote/server_url': 'http://172.23.132.97:8000',
            '/mujoco_digital_twin/server_url': 'http://127.0.0.1:8002'}
        actual = {key:rospy.get_param(key) for key in expected}
        for key, value in expected.items():
            require(actual[key] == value and (type(value) is not bool or type(actual[key]) is bool),
                    'startup parameter differs: ' + key)
        result['parameters'] = actual
        lock, values, history = threading.Lock(), {}, []
        def receive(key, message):
            with lock:
                values[key] = message
                if key == 'joint':
                    history.append(message)
        topic_rows = [('joint', '/alicia_d/accepted_joint_states', JointState),
            ('epoch', '/alicia_d/control_reference_epoch', Header),
            ('task', '/grasp/state', GraspState), ('mode', '/grasp_mode/status', String),
            ('manual', '/gui/joint_direct_mode', Bool),
            ('arm', '/alicia_controller/follow_joint_trajectory/status', GoalStatusArray),
            ('hand', '/hand_controller/follow_joint_trajectory/status', GoalStatusArray)]
        subscriptions.extend(rospy.Subscriber(topic, kind, lambda msg,k=key: receive(k,msg), queue_size=30)
                             for key, topic, kind in topic_rows)
        deadline = time.monotonic() + 10.
        while time.monotonic() < deadline:
            with lock:
                ready = len(values) == len(topic_rows) and len(history) >= 20
            if ready:
                break
            time.sleep(.05)
        with lock:
            state, samples = dict(values), list(history)
        require(len(state) == len(topic_rows) and len(samples) >= 20, 'missing startup telemetry')
        now, q = rospy.Time.now().to_nsec(), state['joint']
        require(q.header.frame_id == 'sdk_measured' and 0 <= now - q.header.stamp.to_nsec() < 200_000_000,
                'SDK measured feedback stale')
        epoch = state['epoch'].stamp.to_nsec()
        require(epoch > 0 and epoch != previous['epoch_ns']
                and all(s.header.stamp.to_nsec() >= epoch for s in samples),
                'feedback does not belong to a new positive-enable epoch')
        require(len({s.header.stamp.to_nsec() for s in samples}) >= 20, 'fewer than 20 unique SDK feedback samples')
        require(state['manual'].data is True
                and state['manual']._connection_header.get('callerid') == '/alicia_supervisor_gui',
                'GUI own manual-mode publication missing')
        require(not state['task'].active, 'task active during restart acceptance')
        mode = json.loads(state['mode'].data)
        require(not any(mode.get(k) for k in ('active', 'start_reserved', 'pending')), 'router busy')
        require(all(not any(g.status in (0,1,6,7) for g in state[k].status_list) for k in ('arm','hand')),
                'active trajectory during restart acceptance')
        selection = rospy.get_param('/grasp_mode/selection')
        require(selection['mode'] == 'unknown' and selection['strategy'] == 'observe_return', 'wrong startup mode')
        result['feedback'] = dict(epoch_ns=epoch, stamp_ns=q.header.stamp.to_nsec(), sample_count=len(samples),
            names=list(q.name), positions=list(q.position), age_sec=(now-q.header.stamp.to_nsec())/1e9)
        result.update(selection=selection, task_state=state['task'].state, task_active=False, mode=mode,
            gui_manual=True, previous_epoch_ns=previous['epoch_ns'],
            command_flags_scope='verifier only; startup has the separately reviewed positive-enable request')
        rospy.wait_for_service('/controller_manager/list_controllers', timeout=5.)
        controllers = rospy.ServiceProxy('/controller_manager/list_controllers', ListControllers)().controller
        states = {c.name:c.state for c in controllers}
        require(all(states.get(n) == 'running' for n in ('alicia_controller', 'hand_controller')),
                'required controller not running')
        result['controllers'] = states
        build = read(DRIVER_BUILD)
        require(build['returncode'] == 0, 'driver build failed')
        source_check(build['source_sha256'])
        binary = W / 'devel/lib/alicia_d_driver/alicia_d_driver_node'
        require(sha(binary) == build['binary_sha256'] == sha(Path('/proc') / str(nodes['/alicia_d_driver_node']['pid']) / 'exe'),
                'driver binary does not match reviewed build')
        result['driver_build'] = dict(path=str(DRIVER_BUILD), sha256=sha(DRIVER_BUILD), binary_sha256=sha(binary),
                                      source_count=len(build['source_sha256']))
        result['gui_windows'] = gui_windows(nodes['/alicia_supervisor_gui']['pid'])
        sim_path = W / '.runtime/full_startup_candidate/candidate_simulation_startup.py'
        spec = importlib.util.spec_from_file_location('cold_start_sim_verifier', sim_path)
        sim = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(sim)
        sim.UNIT = args.simulation_unit if args.simulation_unit.endswith('.service') else args.simulation_unit + '.service'
        sim.wait_ready(timeout=40.)
        result['simulation_service'] = sim.service_snapshot()
        result['simulation_manifest'] = sim.frozen_manifest()
        result['robot_description_sha256'] = hashlib.sha256(rospy.get_param('/robot_description').encode()).hexdigest()
        require(result['robot_description_sha256'] == sim.IDENTITY['candidate_model_sha256'], 'live URDF mismatch')
        handeye_path = W / '.runtime/candidate_models/expanded_metric_20260929/handeye_candidate_config.yaml'
        expected_handeye = yaml.safe_load(handeye_path.read_text())['handeye']
        require(rospy.get_param('/handeye') == expected_handeye, 'live handeye differs from paired configuration')
        result['handeye_configuration_sha256'] = sha(handeye_path)
        config, control = read(EXT / 'grasp_records/archive_config.json'), read(EXT / 'maintenance/storage_control.json')
        require(config['upload_enabled'] is False and config['delete_enabled'] is False and
                control['archive_policy'] == 'local_only', 'storage policy changed')
        import shutil
        result['free_bytes'] = shutil.disk_usage(EXT).free
        require(result['free_bytes'] >= config['min_free_bytes'], 'recording free space inadequate')
        require(properties(args.core_unit) == core and identities() == nodes, 'process identity changed during acceptance')
        require(master.getPid() == master_pid and process(master_pid) == master_process,
                'master process identity changed during acceptance')
        final_registered = set()
        for group in master.getSystemState():
            for _, names in group:
                final_registered.update(names)
        final_processes = {}
        for name in sorted(final_registered - {rospy.get_name()}):
            uri = master.lookupNode(name)
            code, _, pid = xmlrpc.client.ServerProxy(uri, transport=Transport()).getPid(rospy.get_name())
            require(code == 1 and type(pid) is int and pid > 1, 'final registered node PID unavailable: '+name)
            require(in_group(pid, core['ControlGroup']), 'final registered node belongs to another stack: '+name)
            final_processes[name] = process(pid)
        require(set(NODES) <= set(final_processes), 'critical nodes missing from final graph')
        result['final_all_registered_nodes'] = final_processes
        with lock:
            final_state = dict(values)
        final_now = rospy.Time.now().to_nsec()
        final_q = final_state['joint']
        require(final_q.header.frame_id == 'sdk_measured' and
                0 <= final_now-final_q.header.stamp.to_nsec() < 200_000_000,
                'SDK feedback became stale during acceptance')
        require(final_state['epoch'].stamp.to_nsec() == epoch and
                rospy.get_param('/grasp_mode/selection') == selection,
                'epoch or selection changed during acceptance')
        require(final_state['manual'].data is True
                and final_state['manual']._connection_header.get('callerid') == '/alicia_supervisor_gui',
                'GUI manual publication changed during acceptance')
        require(not final_state['task'].active and
                not any(json.loads(final_state['mode'].data).get(k) for k in ('active', 'start_reserved', 'pending')) and
                all(not any(g.status in (0,1,6,7) for g in final_state[k].status_list) for k in ('arm','hand')),
                'task or action became active during acceptance')
        require(all(rospy.get_param(key) == value for key, value in expected.items()),
                'startup parameter changed during acceptance')
        source_check(release['source_sha256'])
        control_path = EXT/'ros_log/full_latest_frozen_no_skin_20261012T021050Z/continuous_authorization_20261012/control.json'
        continuation = read(control_path)
        require(continuation['state'] == 'active', 'continuation control changed after restart')
        require(continuation['revision'] == args.expected_control_revision, 'continuation control revision changed during restart')
        require(continuation == startup_receipt['control'], 'continuation control content changed during restart')
        require(sha(args.release) == args.release_sha256 and sha(request_path) == args.request_sha256,
                'release/request changed during acceptance')
        require(all(sha(path) == digest for path, digest in startup_receipt['operational_source_sha256'].items()),
                'operational source changed during acceptance')
        require(sha(startup_path) == startup_receipt_sha256, 'startup receipt changed during acceptance')
        require(startup_receipt['continuation_control_state'] == continuation['state']
                and startup_receipt['continuation_control_revision'] == continuation['revision'],
                'grasp control differs from startup receipt')
        result.update(deployment_passed=True, whole_patch_deployed=True, all_processes_replaced=True,
                      epoch_ns=epoch, source_count=release['source_count'],
                      continuation_control_state=continuation['state'],
                      continuation_control_revision=continuation['revision'],
                      recorded_ns=time.time_ns())
    except Exception as exc:
        result['error'] = repr(exc)
        raise
    finally:
        for subscription in subscriptions:
            subscription.unregister()
        require(MOUNT.is_mount(), 'SanStand disappeared; cannot save restart receipt')
        with (args.output / 'result.json').open('x') as stream:
            json.dump(result, stream, ensure_ascii=False, indent=2)
            stream.write('\n')
        print(json.dumps(dict(deployment_passed=result['deployment_passed'], output=str(args.output),
                              error=result.get('error')), ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
