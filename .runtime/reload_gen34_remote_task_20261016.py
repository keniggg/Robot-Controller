"""Load sealed GEN34 remote and task; preserve nine nodes and the original producer launcher."""
import argparse
import ast
import hashlib
import json
import os
import re
from pathlib import Path
import shutil
import subprocess
import time
import xmlrpc.client

from continuous_grasp_authorization_20261012 import atomic_json

W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
E = M / '实验记录/Robot-Controller'
CONTROL = E / 'ros_log/full_latest_frozen_no_skin_20261012T021050Z/continuous_authorization_20261012/control.json'
REMOTE = '/remote_grasp6d_node'
TASK = '/grasp_task_node'
TARGETS = (REMOTE, TASK)
PRESERVE = ('/unknown_tabletop_perception', '/motion_gateway',
            '/alicia_d_driver_node', '/bessica_d_hw_interface', '/move_group',
            '/supervisor_camera', '/compliant_gripper_controller',
            '/grasp_mode_router', '/alicia_supervisor_gui')
HELPERS = ('.runtime/start_gen34_remote_task_20261016.sh',
           '.runtime/gen34_remote_task_consumers_20261016.launch',
           '.runtime/reload_gen34_remote_task_20261016.py',
           '.runtime/finalize_gen34_remote_task_deployment_20261016.py',
           '.runtime/read_gen7_yield_readiness_20261012.py',
           '.runtime/continuous_grasp_authorization_20261012.py')
EXPECTED_CONTROL_REVISION = 39
EXPECTED_GENERATION = 6
SESSION = E / 'ros_log/full_latest_gen29_restart_20261016T000728Z'
PRIOR_DEPLOYMENT = SESSION / 'gen33_runtime_deployment.json'
PRIOR_DEPLOYMENT_SHA = '77567fad4b3f9db4ca0eea97bba3387428ddaf0e2a2a39aac209e6421028425c'
BASELINE_RELEASE_SHA = 'f71189761d831981ff648677ebc8c3d73e32fcc27824eda90ccd675314ba555a'
EXPECTED_OLD_REMOTE = dict(pid=124593, start_ticks='3791264', parent_pid=1707)
EXPECTED_OLD_TASK = dict(pid=71345, start_ticks='2599534', parent_pid=71289)
ORIGINAL_PARENT = dict(pid=71289, start_ticks='2599473', parent_pid=1707)
ORIGINAL_LAUNCH = W / '.runtime/observation_measurement_consumers_gen30_20261016.launch'
ORIGINAL_LAUNCH_SHA = '63457b65ea95b40534746eeeed2f1e3d2e91aa9a160c2219efe6f2a2553f6921'
ORIGINAL_PARENT_ARGV = ['/usr/bin/python3', '/opt/ros/noetic/bin/roslaunch', '--skip-log-check',
                        '.runtime/observation_measurement_consumers_gen30_20261016.launch']
TASK_CACHED_KEYS = ('/grasp', '/gui/joint_direct_mode', '/rosout_disable_topics_generation')
CACHED_KEYS = ('/grasp/unknown_require_measured_bilateral_contact',
    '/grasp_mode/selection', '/gui/joint_direct_mode', '/robot_description',
    '/camera', '/unknown_perception', '/handeye', '/rosout_disable_topics_generation')
CACHED_KEY_SOURCE_FILES = (
    'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_remote.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/initial_preflight_remote.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/observation_recovery_history.py',
    'src/alicia_grasp_modes/scripts/mode_aware_grasp_task.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/initial_preflight_task.py')
ROSOUT_PROOF = SESSION / 'verification/gen32_rosout_subscription_r1/result.json'
ROSOUT_PROOF_SHA = 'f7737cb9695485a853c4f23b7cdfad00e9a7e0df76e279304db6d4bf7b1121f4'
ROSOUT_SOURCE = Path('/opt/ros/noetic/lib/python3/dist-packages/rospy/impl/rosout.py')
ROSOUT_SOURCE_SHA = '6ecad7a514b3e8a570d8c416e7047d65ca55ad83f4e1fcf092e3d34e824f4c66'


def read(path):
    return json.loads(Path(path).read_text())


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def require(value, reason):
    if not value:
        raise RuntimeError(reason)


def storage(path):
    path = Path(path).resolve()
    require(M.is_mount() and not os.statvfs(M).f_flag & os.ST_RDONLY,
            'mounted writable SanStand required')
    require(E / 'ros_log' in path.parents and path.parent.stat().st_dev == M.stat().st_dev,
            'external session output required')
    config = read(E / 'grasp_records/archive_config.json')
    require(config['upload_enabled'] is False and config['delete_enabled'] is False,
            'archive policy changed')
    require(read(E / 'maintenance/storage_control.json')['archive_policy'] == 'local_only',
            'external local-only policy changed')
    require(shutil.disk_usage(E).free >= config['min_free_bytes'], 'insufficient external space')


def control():
    value = read(CONTROL)
    require(value['state'] == 'active' and value['revision'] == EXPECTED_CONTROL_REVISION,
            'continuation authorization changed or paused')
    return value


def sources(release):
    require(release['source_count'] == len(release['source_sha256']), 'source count differs')
    for name, digest in release['source_sha256'].items():
        path = Path(name)
        require(not path.is_absolute() and '..' not in path.parts and sha(W / path) == digest,
                'source differs: ' + name)
    for name, digest in release['evidence_sha256'].items():
        require(sha(name) == digest, 'release evidence differs: ' + name)
    for name in HELPERS:
        require(release['evidence_sha256'].get(str(W / name)) == sha(W / name),
                'runtime helper not bound by release: ' + name)


def proc_identity(pid):
    fields = (Path('/proc') / str(pid) / 'stat').read_text().rsplit(')', 1)[1].split()
    require(fields[0] != 'Z', 'zombie process: ' + str(pid))
    return dict(pid=int(pid), start_ticks=fields[19], parent_pid=int(fields[1]))


def exited(row):
    try:
        return proc_identity(row['pid'])['start_ticks'] != row['start_ticks']
    except (FileNotFoundError, ProcessLookupError):
        return True
    except RuntimeError as exc:
        if str(exc).startswith('zombie process:'):
            return True
        raise


def target_processes():
    """Reject an unregistered live replacement of either selected entrypoint."""
    found = []
    names = {'mode_aware_remote_grasp6d.py', 'remote_grasp6d_node.py',
             'mode_aware_grasp_task.py', 'grasp_task_node.py'}
    for process in Path('/proc').iterdir():
        if not process.name.isdigit():
            continue
        try:
            argv = (process / 'cmdline').read_bytes().split(b'\0')
        except (FileNotFoundError, ProcessLookupError):
            continue
        if any(Path(os.fsdecode(arg)).name in names for arg in argv if arg):
            found.append(proc_identity(int(process.name)))
    return found


def original_parent():
    require(proc_identity(ORIGINAL_PARENT['pid']) == ORIGINAL_PARENT, 'original producer launcher changed')
    proc = Path('/proc') / str(ORIGINAL_PARENT['pid'])
    argv = [os.fsdecode(v) for v in (proc/'cmdline').read_bytes().split(b'\0') if v]
    require(argv == ORIGINAL_PARENT_ARGV and Path(os.readlink(proc/'cwd')) == W,
            'original producer launcher command changed')
    require(sha(ORIGINAL_LAUNCH) == ORIGINAL_LAUNCH_SHA, 'original launch file changed')
    return dict(ORIGINAL_PARENT)


class Transport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 3.
        return connection


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ('previous-deployment', 'release', 'output'):
        parser.add_argument('--' + key, type=Path, required=True)
    args = parser.parse_args()
    out = args.output.resolve()
    prior_path, release_path = args.previous_deployment.resolve(strict=True), args.release.resolve(strict=True)
    deployment_path = out.parent / 'gen34_runtime_deployment.json'
    storage(out)
    require(prior_path.parent == out.parent and E in release_path.parents, 'deployment input scope differs')
    require(not deployment_path.exists(), 'GEN34 deployment already exists')
    require(out == SESSION / 'gen34_remote_task_reload_r1', 'fixed single-use reload output required')
    require(prior_path == PRIOR_DEPLOYMENT and sha(prior_path) == PRIOR_DEPLOYMENT_SHA,
            'expected exact GEN33 deployed baseline required')
    previous, release = read(prior_path), read(release_path)
    require(previous['deployment_passed'] is True and previous['whole_patch_deployed'] is True,
            'previous deployment not accepted')
    require(previous['source_manifest_sha256'] == release['baseline_release_sha256'] == BASELINE_RELEASE_SHA,
            'previous deployment differs from reviewed baseline')
    require(Path(release['previous_deployment']).resolve() == prior_path,
            'release was sealed against another deployment')
    require(release['kind'] == 'gen34_post_observation_box_precision_routing'
            and all(release.get(k) is True for k in ('passed', 'reviewed', 'installed')),
            'GEN34 release not sealed')
    summary = release['test_summary']
    require(summary['tests'] > 7483 and all(summary[k] == 0 for k in ('failures', 'errors', 'skipped')),
            'release regression incomplete')
    require(set(previous['after']).isdisjoint(previous['preserved_after']), 'duplicate prior node')
    expected = dict(previous['after'], **previous['preserved_after'])
    require(set(expected) == set(TARGETS + PRESERVE), 'prior eleven-node scope differs')
    require(expected[REMOTE] == EXPECTED_OLD_REMOTE and expected[TASK] == EXPECTED_OLD_TASK,
            'unexpected old remote/task identity')
    original_parent()
    require(sha(ROSOUT_PROOF) == ROSOUT_PROOF_SHA and sha(ROSOUT_SOURCE) == ROSOUT_SOURCE_SHA,
            'source-proven ROS rosout subscription evidence changed')
    proof = read(ROSOUT_PROOF)
    require(proof.get('passed') is True and proof.get('live_ros_contacted') is False
            and proof.get('remaining_after_seven') == [CACHED_KEYS[-1] + '/'],
            'unexpected rosout subscription proof')
    cleanup_sources = {str(ROSOUT_SOURCE): ROSOUT_SOURCE_SHA}
    for path, digest in proof['source_sha256'].items():
        require(sha(path) == digest, 'installed subscription API source changed: ' + path)
        cleanup_sources[path] = digest
    for name in CACHED_KEY_SOURCE_FILES:
        backup = W / '.runtime/gen34_baseline_source' / name
        digest = previous['source_sha256'][name]
        require(sha(backup) == digest, 'prior cached-read source changed: ' + name)
        cleanup_sources[str(backup)] = digest
    prior_sha, release_sha = sha(prior_path), sha(release_path)
    input_bindings = {str(path):sha(path) for path in
        (Path(__file__), prior_path, release_path, ROSOUT_PROOF, ORIGINAL_LAUNCH)}
    input_bindings.update(cleanup_sources)

    def bound_inputs():
        require(all(sha(path) == digest for path,digest in input_bindings.items()),
                'reload input or subscription source changed')
    sources(release); control()
    out.mkdir(exist_ok=False)
    (out / 'consumers_ros').mkdir()
    os.environ['ROS_LOG_DIR'] = str(out / 'observer_ros')
    import rosgraph
    import rospy
    from std_msgs.msg import Header, String
    from std_srvs.srv import Trigger
    from actionlib_msgs.msg import GoalStatusArray
    from alicia_flexible_grasp_supervisor.msg import GraspState
    from alicia_grasp_modes.selection import parse_selection
    rospy.init_node('gen34_remote_task_reload', anonymous=True, disable_rosout=True)
    master = rosgraph.Master(rospy.get_name())

    def rpc(name):
        return xmlrpc.client.ServerProxy(master.lookupNode(name), transport=Transport())

    def identity(name):
        code, _, pid = rpc(name).getPid(rospy.get_name())
        require(code == 1, 'node PID unavailable: ' + name)
        return proc_identity(pid)

    selected = parse_selection(master.getParam('/grasp_mode/selection'))
    require(selected['generation'] == EXPECTED_GENERATION and selected['mode'] == 'unknown'
            and selected['strategy'] == 'observe_return', 'unexpected current selection')
    epoch_ns = rospy.wait_for_message('/alicia_d/control_reference_epoch', Header, timeout=4).stamp.to_nsec()
    require(epoch_ns > 0, 'control reference epoch missing')

    def idle(require_task=True):
        storage(out); control(); bound_inputs(); original_parent()
        require(master.getParam('/gui/joint_direct_mode') is True, 'GUI is not manual')
        require(parse_selection(master.getParam('/grasp_mode/selection')) == selected, 'selection changed')
        require(rospy.wait_for_message('/alicia_d/control_reference_epoch', Header, timeout=4).stamp.to_nsec()
                == epoch_ns, 'control reference epoch changed')
        status = json.loads(rospy.wait_for_message('/grasp_mode/status', String, timeout=4).data)
        require(not any(status.get(k) for k in ('active', 'start_reserved', 'pending')), 'router busy')
        if require_task:
            task = rospy.wait_for_message('/grasp/state', GraspState, timeout=4)
            require(not task.active, 'task active')
            if result.get('new_launch_attempted'):
                require(task.header.stamp.to_nsec() >= result['launch_ros_stamp_ns'],
                        'task state predates this new launch')
            task_state = task.state
        else:
            # A dead task has no latched publisher to query. Its pre-stop idle
            # evidence remains recorded; require exact process absence instead.
            require(exited(result['before'][TASK]) and not target_processes(),
                    'task gap is not an exact inactive-process gap')
            task_state = 'stopped_for_reviewed_reload'
        for topic in ('/alicia_controller/follow_joint_trajectory/status',
                      '/hand_controller/follow_joint_trajectory/status'):
            statuses = rospy.wait_for_message(topic, GoalStatusArray, timeout=4)
            require(not any(g.status in (0, 1, 6, 7) for g in statuses.status_list), 'active action')
        require(master.getParam('/alicia_d_driver_node/software_temperature_monitoring_enabled') is False
                and master.getParam('/alicia_d_driver_node/temperature_poll_rate_hz') == 0.,
                'temperature monitoring policy changed')
        control()
        require(master.getParam('/gui/joint_direct_mode') is True, 'manual mode changed during idle check')
        return dict(recorded_ns=time.time_ns(), task_state=task_state, task_active=False,
                    task_state_live=bool(require_task),
                    manual_mode=True, selection=selected, epoch_ns=epoch_ns, router_inactive=True,
                    action_inactive=True)

    result = dict(deployment_passed=False, whole_patch_deployed=False, remote_reloaded=False, task_reloaded=False,
        source_manifest=str(release_path), source_manifest_sha256=release_sha,
        source_sha256=release['source_sha256'], test_summary=summary,
        previous_deployment=str(prior_path), previous_deployment_sha256=prior_sha,
        motion_commands_sent=False, torque_commands_sent=False, temperature_queried=False,
        new_selection_required_before_automatic_grasp=True, selection=selected, epoch_ns=epoch_ns,
        control_revision=EXPECTED_CONTROL_REVISION, deployment_kind='remote_and_task_nine_processes_preserved',
        reload_script_sha256=sha(__file__), startup_dependencies_sha256={name:sha(W/name) for name in HELPERS},
        shutdown_requests=[], physical_grasp_verified=False, new_launch_attempted=False,
        parameter_subscription_keys={REMOTE:list(CACHED_KEYS), TASK:list(TASK_CACHED_KEYS)}, unsubscribe_requests=[],
        parameter_subscription_source_sha256=cleanup_sources,
        parameter_subscription_proof=str(ROSOUT_PROOF), parameter_subscription_proof_sha256=ROSOUT_PROOF_SHA,
        reload_input_evidence_sha256=input_bindings)

    def absent_or_stale(name):
        # Only an exact refused old URI is removable. Timeout/DNS failure is not
        # evidence of process death, and an unexpected live replacement is a stop.
        control(); bound_inputs(); original_parent()
        require(all(exited(result['before'][n]) for n in TARGETS), 'an old target process remains')
        require(not target_processes(), 'a live target process exists; no cleanup or duplicate launch')
        require({name:identity(name) for name in PRESERVE} == result['preserved_before'],
                'one of the nine preserved processes changed')
        state = master.getSystemState()
        require(not any(name in owners for rows in state for _, owners in rows),
                'unexpected remote topic/service registration; bounded parameter cleanup only')
        try:
            uri = master.lookupNode(name)
        except rosgraph.MasterError:
            return False
        require(uri == result['old_target_uris'][name], 'remote URI changed')
        refused = []
        for _ in range(2):
            require(master.lookupNode(name) == result['old_target_uris'][name], 'remote URI changed before refusal check')
            try:
                with xmlrpc.client.ServerProxy(result['old_target_uris'][name], transport=Transport()) as old_rpc:
                    old_rpc.getPid(rospy.get_name())
            except ConnectionRefusedError as exc:
                refused.append(dict(error=type(exc).__name__, errno=exc.errno, checked_ns=time.time_ns()))
            else:
                raise RuntimeError('old remote URI responded; cleanup prohibited')
            time.sleep(.05)
        result.setdefault('stale_refusal_checks', []).append(dict(name=name, refusals=refused))
        require(exited(result['before'][name]) and not target_processes(),
                'remote appeared during stale-node verification')
        require(master.lookupNode(name) == result['old_target_uris'][name], 'remote URI changed during refusal checks')
        return True

    def parameter_value_fingerprints():
        values = {}
        for name in dict.fromkeys(('/grasp',) + CACHED_KEYS + TASK_CACHED_KEYS):
            try:
                value = master.getParam(name)
            except rosgraph.MasterError:
                values[name] = dict(present=False)
            else:
                encoded = json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
                comparison = json.loads(encoded)
                # The existing camera timer updates only this diagnostic tree.
                # Preserve its raw digest; all other camera keys remain exact.
                if name == '/camera' and isinstance(comparison, dict):
                    profile = comparison.get('runtime_profile')
                    if isinstance(profile, dict):
                        profile.pop('stationary_temporal', None)
                compared = json.dumps(comparison, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
                values[name] = dict(present=True, value_sha256=hashlib.sha256(encoded).hexdigest(),
                                    comparison_sha256=hashlib.sha256(compared).hexdigest())
        return values


    def save():
        storage(out)
        atomic_json(out / 'result.json', result)

    try:
        result['idle_before'] = idle()
        result['before'] = {name:identity(name) for name in TARGETS}
        result['preserved_before'] = {name:identity(name) for name in PRESERVE}
        require(dict(result['before'], **result['preserved_before']) == expected,
                'prior node identity changed')
        result['original_parent_before'] = original_parent()
        result['old_target_uris'] = {name:master.lookupNode(name) for name in TARGETS}
        values_before = parameter_value_fingerprints()
        result['parameter_values_before'] = values_before
        sources(release); save(); idle()
        for name in TARGETS:
            idle()
            require(identity(name) == result['before'][name], 'target identity changed before shutdown')
            require(master.lookupNode(name) == result['old_target_uris'][name], 'target URI changed before shutdown')
            row = dict(name=name, uri=result['old_target_uris'][name], requested_ns=time.time_ns(), attempted=True)
            result['shutdown_requests'].append(row); save(); control(); original_parent()
            try:
                with xmlrpc.client.ServerProxy(result['old_target_uris'][name], transport=Transport()) as old_rpc:
                    code, message, _ = old_rpc.shutdown(rospy.get_name(), 'load reviewed GEN34 post-observation box precision routing repair')
                row.update(return_code=code, message=message)
            except Exception as exc:
                row['rpc_error'] = repr(exc)
            save()
            deadline = time.monotonic() + 25.
            while time.monotonic() < deadline and not exited(result['before'][name]):
                control(); original_parent(); time.sleep(.1)
            require(exited(result['before'][name]), 'old target remains; no duplicate launch')
        result['old_processes_exited'] = True
        # Only source-proven parameter NodeRefs may remain after graceful exit.
        # A successful RPC is not evidence that a key existed; lookup absence is.
        for name in TARGETS:
            stale_master = rosgraph.Master(name)
            keys = CACHED_KEYS if name == REMOTE else TASK_CACHED_KEYS
            for key in keys:
                if not absent_or_stale(name):
                    break
                result['idle_before_cleanup_step'] = idle(require_task=False)
                if not absent_or_stale(name):
                    break
                row = dict(caller_id=name, caller_api=result['old_target_uris'][name], key=key,
                           requested_ns=time.time_ns(), attempted=False)
                result['unsubscribe_requests'].append(row); save(); control(); bound_inputs(); original_parent()
                require(master.lookupNode(name) == result['old_target_uris'][name], 'target URI changed at cleanup commit')
                require(all(exited(result['before'][n]) for n in TARGETS) and not target_processes(),
                        'target appeared at cleanup commit')
                require(master.getParam('/gui/joint_direct_mode') is True, 'manual ownership changed')
                row['attempted'] = True; save()
                row['rpc_return'] = stale_master.unsubscribeParam(result['old_target_uris'][name], key)
                row['returned_ns'] = time.time_ns(); save()
            require(not absent_or_stale(name), 'unknown stale registration remains; no expanded cleanup or launch')
        result['parameter_values_after_cleanup'] = parameter_value_fingerprints()
        comparison = lambda rows: {key: (row['present'], row.get('comparison_sha256'))
                                   for key,row in rows.items()}
        result['parameter_value_comparison_excludes'] = ['/camera/runtime_profile/stationary_temporal']
        result['parameter_raw_digest_changes'] = [key for key in values_before
            if values_before[key] != result['parameter_values_after_cleanup'][key]]
        require(comparison(values_before) == comparison(result['parameter_values_after_cleanup']),
                'parameter configuration changed during shutdown or cleanup')
        result['parameter_configuration_unchanged'] = True
        result['stale_registration_cleanup_complete'] = True
        sources(release); result['idle_before_launch'] = idle(require_task=False); save(); control()
        require(not any(absent_or_stale(n) for n in TARGETS), 'target registration reappeared before launch')
        grasp_before = master.getParam('/grasp')
        require(isinstance(grasp_before, dict) and len(grasp_before) == 127,
                'expected complete 127-key grasp mapping unavailable')
        grasp_bytes = json.dumps(grasp_before, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
        grasp_digest = hashlib.sha256(grasp_bytes).hexdigest()
        require(grasp_digest == values_before['/grasp']['value_sha256'],
                'grasp configuration changed before new process launch')
        unit = 'alicia-gen34-remote-task-' + str(time.time_ns())
        command = ['systemd-run', '--user', '--collect', '--unit=' + unit,
            '--property=Type=exec', '--property=Restart=no', '--property=WorkingDirectory=' + str(W),
            '--property=StandardOutput=append:' + str(out/'consumers.log'),
            '--property=StandardError=append:' + str(out/'consumers.log'),
            '--setenv=ROS_LOG_DIR=' + str(out/'consumers_ros'),
            '/bin/bash', str(W/'.runtime/start_gen34_remote_task_20261016.sh')]
        result.update(unit=unit, command=command, new_launch_attempted=True,
                      launch_ros_stamp_ns=rospy.Time.now().to_nsec()); save(); control()
        require(not any(absent_or_stale(n) for n in TARGETS), 'target appeared at launch commit')
        bound_inputs()
        subprocess.run(command, check=True, timeout=15.)
        for service in ('/grasp_mode/reset_remote', '/grasp_mode/reset_task'):
            rospy.wait_for_service(service, timeout=40.)
        main_pid = int(subprocess.check_output(
            ['systemctl', '--user', 'show', unit, '--property=MainPID', '--value'], text=True).strip())
        loaded_targets = {name:identity(name) for name in TARGETS}
        require(main_pid > 1 and all(row['parent_pid'] == main_pid for row in loaded_targets.values()),
                'new targets do not share the reviewed roslaunch unit parent')
        require(all(loaded_targets[n] != result['before'][n] for n in TARGETS), 'old target process reused')
        idle()
        result['reset_ack'] = {}
        for service in ('/grasp_mode/reset_remote', '/grasp_mode/reset_task'):
            response = rospy.ServiceProxy(service, Trigger)()
            require(response.success, service + ' reset rejected: ' + response.message)
            ack = json.loads(response.message)
            require(ack.get('ready') is True and parse_selection(ack) == selected,
                    service + ' reset acknowledged another selection')
            result['reset_ack'][service] = ack
        grasp_after = master.getParam('/grasp')
        require(grasp_after == grasp_before, 'master grasp configuration changed during reload')
        marker = '[RemoteGrasp6D] authoritative /grasp loaded '
        log_path = out / 'remote.log'
        deadline = time.monotonic() + 3.
        prefix = b''
        lines = []
        while time.monotonic() < deadline:
            control()
            prefix = log_path.read_bytes()
            lines = [line for line in prefix.decode(errors='replace').splitlines() if marker in line]
            if lines:
                break
            time.sleep(.05)
        require(len(lines) == 1, 'expected exactly one authoritative startup summary')
        match = re.search(r'authoritative /grasp loaded key_count=(\d+) '
                          r'contact_height_registration_margin_enabled=(\S+) '
                          r'handeye_translation_max_error_m=(\S+)', lines[0])
        require(match is not None, 'startup summary format differs')
        loaded = dict(key_count=int(match.group(1)),
            contact_height_registration_margin_enabled=ast.literal_eval(match.group(2)),
            handeye_translation_max_error_m=ast.literal_eval(match.group(3)))
        expected_loaded = dict(key_count=len(grasp_after),
            contact_height_registration_margin_enabled=grasp_after.get('contact_height_registration_margin_enabled'),
            handeye_translation_max_error_m=grasp_after.get('handeye_translation_max_error_m'))
        require(loaded == expected_loaded, 'startup did not load the complete authoritative grasp mapping')
        prefix_path = out / 'remote_startup_prefix.log'
        with prefix_path.open('xb') as stream:
            stream.write(prefix); stream.flush(); os.fsync(stream.fileno())
        require(log_path.read_bytes()[:len(prefix)] == prefix and prefix_path.read_bytes() == prefix,
                'startup log prefix changed')
        result['startup_configuration'] = dict(loaded=loaded, master_grasp_sha256=grasp_digest,
            master_mapping_unchanged=True, current_master_matches=True,
            live_log_path=str(log_path), live_log_prefix_bytes=len(prefix),
            frozen_log_prefix=str(prefix_path), frozen_log_prefix_sha256=sha(prefix_path))
        result['idle_after'] = idle()
        result['after'] = {name:identity(name) for name in TARGETS}
        result['preserved_after'] = {name:identity(name) for name in PRESERVE}
        require(result['after'] == loaded_targets, 'target changed after reset')
        require(result['preserved_after'] == result['preserved_before'], 'preserved process changed')
        require(len({(r['pid'],r['start_ticks']) for r in
                     dict(result['after'], **result['preserved_after']).values()}) == 11,
                'deployment identities overlap')
        main_proc = Path('/proc') / str(main_pid)
        main_argv = [os.fsdecode(v) for v in (main_proc/'cmdline').read_bytes().split(b'\0') if v]
        require('/opt/ros/noetic/bin/roslaunch' in main_argv
                and str(W/'.runtime/gen34_remote_task_consumers_20261016.launch') in main_argv,
                'unexpected new roslaunch command')
        expected_entry = {REMOTE:'mode_aware_remote_grasp6d.py', TASK:'mode_aware_grasp_task.py'}
        for name,row in loaded_targets.items():
            proc = Path('/proc') / str(row['pid'])
            env = dict(x.split(b'=',1) for x in (proc/'environ').read_bytes().split(b'\0') if b'=' in x)
            require(Path(os.fsdecode(env[b'ROS_LOG_DIR'])).resolve() == out/'consumers_ros', 'target logs not external')
            require(env.get(b'PYTHONNOUSERSITE') == b'1' and env.get(b'PYTHONDONTWRITEBYTECODE') == b'1',
                    'target environment differs from reviewed startup')
            argv = [os.fsdecode(v) for v in (proc/'cmdline').read_bytes().split(b'\0') if v]
            require(any(Path(v).name == expected_entry[name] for v in argv), 'unexpected target entrypoint')
            if name == TASK:
                require('/grasp/start:=/grasp/internal_start' in argv, 'task internal start remap missing')
        result['original_parent_after'] = original_parent()
        require(result['original_parent_before'] == result['original_parent_after'], 'original parent changed')
        sources(release); bound_inputs()
        require(sha(prior_path) == prior_sha and sha(release_path) == release_sha, 'deployment inputs changed')
        result['idle_after'] = idle()
        require({name:identity(name) for name in TARGETS + PRESERVE}
                == dict(result['after'], **result['preserved_after']),
                'process changed before deployment publication')
        result.update(deployment_passed=True, whole_patch_deployed=True, remote_reloaded=True, task_reloaded=True,
                      compute_reloaded=True, hardware_processes_preserved=True,
                      other_nine_processes_preserved=True, original_parent_preserved=True, unit_main_pid=main_pid,
                      unit_identity=proc_identity(main_pid), recorded_ns=time.time_ns())
        save()
        deployment = dict(result, reload_evidence=str(out/'result.json'),
                          reload_sha256=sha(out/'result.json'))
        require(not deployment_path.exists(), 'deployment output appeared during reload')
        atomic_json(deployment_path, deployment)
    except Exception as exc:
        result['error'] = repr(exc)
        result['deployment_passed'] = False
        result['whole_patch_deployed'] = False
        save()
        raise
    print(json.dumps(dict(deployment_passed=True, result=str(out/'result.json'),
                          deployment=str(deployment_path), sha256=sha(deployment_path)), ensure_ascii=False))


if __name__ == '__main__':
    main()
