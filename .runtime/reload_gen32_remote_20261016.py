"""Load sealed GEN32 in remote only; preserve the other ten ROS process identities."""
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
TARGETS = (REMOTE,)
PRESERVE = ('/grasp_task_node', '/unknown_tabletop_perception', '/motion_gateway',
            '/alicia_d_driver_node', '/bessica_d_hw_interface', '/move_group',
            '/supervisor_camera', '/compliant_gripper_controller',
            '/grasp_mode_router', '/alicia_supervisor_gui')
HELPERS = ('.runtime/start_gen32_remote_20261016.sh',
           '.runtime/reload_gen32_remote_20261016.py',
           '.runtime/finalize_gen32_remote_deployment_20261016.py',
           '.runtime/read_gen7_yield_readiness_20261012.py',
           '.runtime/continuous_grasp_authorization_20261012.py')
EXPECTED_CONTROL_REVISION = 35
EXPECTED_GENERATION = 4


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
    deployment_path = out.parent / 'gen32_runtime_deployment.json'
    storage(out)
    require(prior_path.parent == out.parent and E in release_path.parents, 'deployment input scope differs')
    require(not deployment_path.exists(), 'GEN32 deployment already exists')
    previous, release = read(prior_path), read(release_path)
    require(previous['deployment_passed'] is True and previous['whole_patch_deployed'] is True,
            'previous deployment not accepted')
    require(previous['source_manifest_sha256'] == release['baseline_release_sha256'],
            'previous deployment differs from reviewed baseline')
    require(Path(release['previous_deployment']).resolve() == prior_path,
            'release was sealed against another deployment')
    require(release['kind'] == 'gen32_authoritative_grasp_configuration'
            and all(release.get(k) is True for k in ('passed', 'reviewed', 'installed')),
            'GEN32 release not sealed')
    summary = release['test_summary']
    require(summary['tests'] > 7329 and all(summary[k] == 0 for k in ('failures', 'errors', 'skipped')),
            'release regression incomplete')
    require(set(previous['after']).isdisjoint(previous['preserved_after']), 'duplicate prior node')
    expected = dict(previous['after'], **previous['preserved_after'])
    require(set(expected) == set(TARGETS + PRESERVE), 'prior eleven-node scope differs')
    prior_sha, release_sha = sha(prior_path), sha(release_path)
    sources(release); control()
    out.mkdir(exist_ok=False)
    (out / 'remote_ros').mkdir()
    os.environ['ROS_LOG_DIR'] = str(out / 'observer_ros')
    import rosgraph
    import rospy
    from std_msgs.msg import Header, String
    from std_srvs.srv import Trigger
    from actionlib_msgs.msg import GoalStatusArray
    from alicia_flexible_grasp_supervisor.msg import GraspState
    from alicia_grasp_modes.selection import parse_selection
    rospy.init_node('gen32_remote_reload', anonymous=True, disable_rosout=True)
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

    def idle():
        storage(out); control()
        require(master.getParam('/gui/joint_direct_mode') is True, 'GUI is not manual')
        require(parse_selection(master.getParam('/grasp_mode/selection')) == selected, 'selection changed')
        require(rospy.wait_for_message('/alicia_d/control_reference_epoch', Header, timeout=4).stamp.to_nsec()
                == epoch_ns, 'control reference epoch changed')
        status = json.loads(rospy.wait_for_message('/grasp_mode/status', String, timeout=4).data)
        require(not any(status.get(k) for k in ('active', 'start_reserved', 'pending')), 'router busy')
        task = rospy.wait_for_message('/grasp/state', GraspState, timeout=4)
        require(not task.active, 'task active')
        for topic in ('/alicia_controller/follow_joint_trajectory/status',
                      '/hand_controller/follow_joint_trajectory/status'):
            statuses = rospy.wait_for_message(topic, GoalStatusArray, timeout=4)
            require(not any(g.status in (0, 1, 6, 7) for g in statuses.status_list), 'active action')
        require(master.getParam('/alicia_d_driver_node/software_temperature_monitoring_enabled') is False
                and master.getParam('/alicia_d_driver_node/temperature_poll_rate_hz') == 0.,
                'temperature monitoring policy changed')
        control()
        require(master.getParam('/gui/joint_direct_mode') is True, 'manual mode changed during idle check')
        return dict(recorded_ns=time.time_ns(), task_state=task.state, task_active=False,
                    manual_mode=True, selection=selected, epoch_ns=epoch_ns, router_inactive=True,
                    action_inactive=True)

    result = dict(deployment_passed=False, whole_patch_deployed=False, remote_reloaded=False,
        source_manifest=str(release_path), source_manifest_sha256=release_sha,
        source_sha256=release['source_sha256'], test_summary=summary,
        previous_deployment=str(prior_path), previous_deployment_sha256=prior_sha,
        motion_commands_sent=False, torque_commands_sent=False, temperature_queried=False,
        new_selection_required_before_automatic_grasp=True, selection=selected, epoch_ns=epoch_ns,
        control_revision=EXPECTED_CONTROL_REVISION, deployment_kind='remote_only_ten_processes_preserved',
        reload_script_sha256=sha(__file__), startup_dependencies_sha256={name:sha(W/name) for name in HELPERS},
        shutdown_requests=[], physical_grasp_verified=False)

    def save():
        storage(out)
        atomic_json(out / 'result.json', result)

    try:
        result['idle_before'] = idle()
        result['before'] = {REMOTE:identity(REMOTE)}
        result['preserved_before'] = {name:identity(name) for name in PRESERVE}
        require(dict(result['before'], **result['preserved_before']) == expected,
                'prior node identity changed')
        sources(release); save(); idle()
        require(identity(REMOTE) == result['before'][REMOTE], 'remote identity changed before shutdown')
        row = dict(name=REMOTE, requested_ns=time.time_ns(), attempted=True)
        result['shutdown_requests'].append(row); save(); control()
        try:
            code, message, _ = rpc(REMOTE).shutdown(rospy.get_name(), 'load reviewed GEN32 complete grasp configuration repair')
            row.update(return_code=code, message=message)
        except Exception as exc:
            row['rpc_error'] = repr(exc)
        save()
        deadline = time.monotonic() + 25.
        while time.monotonic() < deadline and not exited(result['before'][REMOTE]):
            control(); time.sleep(.1)
        require(exited(result['before'][REMOTE]), 'old remote remains; no duplicate launch')
        result['old_processes_exited'] = True
        # No respawn is configured for remote; require its old registration to go away.
        deadline = time.monotonic() + 10.
        absent = False
        while time.monotonic() < deadline:
            control()
            try:
                master.lookupNode(REMOTE)
            except rosgraph.MasterError:
                absent = True
                break
            time.sleep(.1)
        require(absent, 'remote registration still present; no duplicate launch')
        require({name:identity(name) for name in PRESERVE} == result['preserved_before'],
                'one of the ten preserved processes changed')
        sources(release); result['idle_before_launch'] = idle(); save(); control()
        grasp_before = master.getParam('/grasp')
        require(isinstance(grasp_before, dict) and grasp_before, 'complete grasp mapping unavailable')
        grasp_bytes = json.dumps(grasp_before, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
        grasp_digest = hashlib.sha256(grasp_bytes).hexdigest()
        unit = 'alicia-gen32-remote-' + str(time.time_ns())
        command = ['systemd-run', '--user', '--collect', '--unit=' + unit,
            '--property=Type=exec', '--property=Restart=no', '--property=WorkingDirectory=' + str(W),
            '--property=StandardOutput=append:' + str(out/'remote.log'),
            '--property=StandardError=append:' + str(out/'remote.log'),
            '--setenv=ROS_LOG_DIR=' + str(out/'remote_ros'),
            '/bin/bash', str(W/'.runtime/start_gen32_remote_20261016.sh')]
        result.update(unit=unit, command=command, new_launch_attempted=True); save(); control()
        subprocess.run(command, check=True, timeout=15.)
        rospy.wait_for_service('/grasp_mode/reset_remote', timeout=40.)
        main_pid = int(subprocess.check_output(
            ['systemctl', '--user', 'show', unit, '--property=MainPID', '--value'], text=True).strip())
        remote = identity(REMOTE)
        require(main_pid > 1 and remote['pid'] == main_pid, 'remote is not the direct unit MainPID')
        require(remote != result['before'][REMOTE], 'old remote process reused')
        idle()
        response = rospy.ServiceProxy('/grasp_mode/reset_remote', Trigger)()
        require(response.success, 'remote reset rejected: ' + response.message)
        ack = json.loads(response.message)
        require(ack.get('ready') is True and parse_selection(ack) == selected,
                'remote reset acknowledged another selection')
        result['reset_ack'] = {'/grasp_mode/reset_remote':ack}
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
        result['after'] = {REMOTE:identity(REMOTE)}
        result['preserved_after'] = {name:identity(name) for name in PRESERVE}
        require(result['after'][REMOTE] == remote, 'remote changed after reset')
        require(result['preserved_after'] == result['preserved_before'], 'preserved process changed')
        require(len({(r['pid'],r['start_ticks']) for r in
                     dict(result['after'], **result['preserved_after']).values()}) == 11,
                'deployment identities overlap')
        env = dict(x.split(b'=',1) for x in (Path('/proc')/str(main_pid)/'environ').read_bytes().split(b'\0') if b'=' in x)
        require(Path(os.fsdecode(env[b'ROS_LOG_DIR'])).resolve() == out/'remote_ros', 'remote logs not external')
        require(env.get(b'PYTHONNOUSERSITE') == b'1' and env.get(b'PYTHONDONTWRITEBYTECODE') == b'1',
                'remote environment differs from reviewed startup')
        cmdline = (Path('/proc')/str(main_pid)/'cmdline').read_bytes().split(b'\0')
        require(b'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py' in cmdline,
                'unexpected remote entrypoint')
        sources(release)
        require(sha(prior_path) == prior_sha and sha(release_path) == release_sha, 'deployment inputs changed')
        result['idle_after'] = idle()
        require({name:identity(name) for name in TARGETS + PRESERVE}
                == dict(result['after'], **result['preserved_after']),
                'process changed before deployment publication')
        result.update(deployment_passed=True, whole_patch_deployed=True, remote_reloaded=True,
                      compute_reloaded=True, hardware_processes_preserved=True,
                      other_ten_processes_preserved=True, unit_main_pid=main_pid, recorded_ns=time.time_ns())
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
