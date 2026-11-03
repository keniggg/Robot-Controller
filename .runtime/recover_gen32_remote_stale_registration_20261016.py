"""Recover one exact dead GEN31 remote registration, then load sealed GEN32 once.

Only the seven parameter-subscription keys used by the prior remote are removed.
No parameter values, topic/service registrations, other nodes or hardware are changed.
The original five startup dependencies and postlaunch acceptance remain unchanged.
"""
import argparse
import ast
import hashlib
import json
import os
import re
from pathlib import Path
import subprocess
import time
import xmlrpc.client

from continuous_grasp_authorization_20261012 import atomic_json
from reload_gen32_remote_20261016 import (
    W, M, E, CONTROL, REMOTE, TARGETS, PRESERVE, HELPERS,
    EXPECTED_CONTROL_REVISION, EXPECTED_GENERATION,
    read, sha, require, storage, control, sources, proc_identity, exited, Transport)

SESSION = E / 'ros_log/full_latest_gen29_restart_20261016T000728Z'
RELEASE = E / 'ros_log/full_latest_gen22_restart_20261015T003700Z/verification/gen32_release_manifest.json'
RELEASE_SHA = 'a4cd78e241e3c232cd858661a2c2c4e4eedc3ced85fc5ac79f5f488d07db043a'
FAILED = SESSION / 'gen32_remote_reload_r1/result.json'
FAILED_SHA = 'b9f76ad1690c1b73bd41ee864a5c6541af647bda7d9e41f69f6477a865c088ae'
STALE_READONLY = SESSION / 'gen32_remote_reload_r1/stale_registration_readonly.json'
STALE_READONLY_SHA = 'bdf23da076c314e521790cf350930b63ce548882a56c3c7d3c67f23566c94acf'
STALE_URI = 'http://ubuntu:38761/'
CACHED_KEYS = (
    '/grasp/unknown_require_measured_bilateral_contact',
    '/grasp_mode/selection', '/gui/joint_direct_mode',
    '/robot_description', '/camera', '/unknown_perception', '/handeye')
CACHED_KEY_SOURCE_FILES = (
    'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_remote.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/initial_preflight_remote.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/observation_recovery_history.py')


def remote_processes():
    """Read-only bounded entrypoint scan, including an unregistered replacement."""
    found = []
    names = {'mode_aware_remote_grasp6d.py', 'remote_grasp6d_node.py'}
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ('previous-deployment', 'release', 'failed-reload', 'recovery-review', 'output'):
        parser.add_argument('--' + key, type=Path, required=True)
    parser.add_argument('--recovery-review-sha256', required=True)
    args = parser.parse_args()
    out = args.output.resolve()
    prior_path, release_path = args.previous_deployment.resolve(strict=True), args.release.resolve(strict=True)
    deployment_path = out.parent / 'gen32_runtime_deployment.json'
    storage(out)
    require(prior_path.parent == out.parent and E in release_path.parents, 'deployment input scope differs')
    require(not deployment_path.exists(), 'GEN32 deployment already exists')
    require(out == SESSION / 'gen32_remote_recovery_r1', 'fixed single-use recovery output required')
    require(release_path == RELEASE and sha(release_path) == RELEASE_SHA,
            'recovery requires the exact immutable GEN32 release')
    failed_path = args.failed_reload.resolve(strict=True)
    review_path = args.recovery_review.resolve(strict=True)
    require(failed_path == FAILED and sha(failed_path) == FAILED_SHA,
            'recovery requires the exact failed reload receipt')
    require(SESSION in review_path.parents and sha(review_path) == args.recovery_review_sha256,
            'recovery review path or digest differs')
    review = read(review_path)
    require(review.get('passed') is True
            and review.get('recovery_helper_sha256') == sha(__file__)
            and review.get('failed_reload_sha256') == FAILED_SHA
            and review.get('release_sha256') == RELEASE_SHA,
            'recovery helper has not passed its exact independent review')
    require(sha(STALE_READONLY) == STALE_READONLY_SHA, 'readonly stale-node receipt changed')
    readonly = read(STALE_READONLY)
    require(readonly['system_state_remote'] == dict(publishers=[], subscribers=[], services=[])
            and readonly['lookup'] == [1, 'node api', STALE_URI], 'unexpected recorded stale registration')
    failed = read(failed_path)
    require(failed.get('deployment_passed') is False
            and failed.get('whole_patch_deployed') is False
            and failed.get('remote_reloaded') is False
            and failed.get('old_processes_exited') is True
            and not failed.get('new_launch_attempted')
            and failed.get('error') == "RuntimeError('remote registration still present; no duplicate launch')"
            and failed['source_manifest_sha256'] == RELEASE_SHA
            and Path(failed['source_manifest']).resolve() == release_path
            and Path(failed['previous_deployment']).resolve() == prior_path
            and failed['previous_deployment_sha256'] == sha(prior_path),
            'failed reload was not the reviewed prelaunch stale-registration failure')
    recovery_inputs = {str(path): sha(path) for path in
        (Path(__file__), failed_path, review_path, STALE_READONLY)}

    def recovery_bindings():
        require(all(sha(path) == digest for path, digest in recovery_inputs.items()),
                'recovery input or helper changed')

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
    require(dict(failed['before'], **failed['preserved_before']) == expected,
            'failed reload process identities differ from baseline')
    require(failed['startup_dependencies_sha256'] == {name:sha(W/name) for name in HELPERS},
            'original five startup helpers changed')
    historical_sources = {}
    for name in CACHED_KEY_SOURCE_FILES:
        backup = W / '.runtime/gen32_baseline_source' / name
        digest = previous['source_sha256'][name]
        require(sha(backup) == digest, 'historical cached-read source changed: '+name)
        historical_sources[str(backup)] = digest
    recovery_inputs.update(historical_sources)
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
    rospy.init_node('gen32_remote_stale_registration_recovery', anonymous=True, disable_rosout=True)
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
    require(selected == failed['selection'] and epoch_ns == failed['epoch_ns'],
            'selection or epoch changed since the interrupted reload')

    def idle():
        storage(out); control(); recovery_bindings()
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
        shutdown_requests=[], physical_grasp_verified=False,
        recovery_kind='exact_dead_remote_parameter_subscriptions_only',
        recovery_helper_sha256=sha(__file__), recovery_review=str(review_path),
        recovery_review_sha256=args.recovery_review_sha256,
        recovery_input_evidence_sha256=recovery_inputs,
        failed_reload=str(failed_path), failed_reload_sha256=FAILED_SHA,
        failed_reload_shutdown_requests=failed['shutdown_requests'],
        original_reload_script_sha256=failed['reload_script_sha256'],
        stale_registration_readonly=str(STALE_READONLY),
        stale_registration_readonly_sha256=STALE_READONLY_SHA,
        stale_node_uri=STALE_URI, parameter_subscription_keys=list(CACHED_KEYS),
        parameter_subscription_source_provenance=historical_sources,
        unsubscribe_requests=[], new_launch_attempted=False)

    def absent_or_stale():
        # Only an exact refused old URI is removable. Timeout/DNS failure is not
        # evidence of process death, and an unexpected live replacement is a stop.
        control(); recovery_bindings()
        require(exited(failed['before'][REMOTE]), 'old remote process remains')
        require(not remote_processes(), 'a live remote process exists; no cleanup or duplicate launch')
        require({name:identity(name) for name in PRESERVE} == failed['preserved_before'],
                'one of the ten preserved processes changed')
        state = master.getSystemState()
        require(not any(REMOTE in owners for rows in state for _, owners in rows),
                'unexpected remote topic/service registration; bounded parameter cleanup only')
        try:
            uri = master.lookupNode(REMOTE)
        except rosgraph.MasterError:
            return False
        require(uri == STALE_URI, 'remote URI changed')
        refused = []
        for _ in range(2):
            require(master.lookupNode(REMOTE) == STALE_URI, 'remote URI changed before refusal check')
            try:
                with xmlrpc.client.ServerProxy(STALE_URI, transport=Transport()) as old_rpc:
                    old_rpc.getPid(rospy.get_name())
            except ConnectionRefusedError as exc:
                refused.append(dict(error=type(exc).__name__, errno=exc.errno, checked_ns=time.time_ns()))
            else:
                raise RuntimeError('old remote URI responded; cleanup prohibited')
            time.sleep(.05)
        result.setdefault('stale_refusal_checks', []).append(refused)
        require(exited(failed['before'][REMOTE]) and not remote_processes(),
                'remote appeared during stale-node verification')
        require(master.lookupNode(REMOTE) == STALE_URI, 'remote URI changed during refusal checks')
        return True

    def parameter_value_fingerprints():
        values = {}
        for name in ('/grasp',) + CACHED_KEYS:
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
        result['before'] = failed['before']
        result['preserved_before'] = {name:identity(name) for name in PRESERVE}
        require(result['preserved_before'] == failed['preserved_before'], 'preserved process identity changed')
        require(absent_or_stale(), 'expected exact stale remote registration is no longer present')
        result['old_processes_exited'] = True
        values_before = parameter_value_fingerprints()
        result['parameter_values_before'] = values_before
        sources(release); save(); idle()
        # Noetic master_api.unsubscribeParam returns 1 even for an absent key.
        # Record raw RPC responses only; final lookup absence is the proof that
        # the dead NodeRef is gone. This never deletes a parameter value.
        stale_master = rosgraph.Master(REMOTE)
        for key in CACHED_KEYS:
            if not absent_or_stale():
                break
            row = dict(caller_id=REMOTE, caller_api=STALE_URI, key=key,
                       attempted=False, requested_ns=time.time_ns())
            result['unsubscribe_requests'].append(row); save()
            result['idle_before_cleanup_step'] = idle()
            control(); recovery_bindings()
            require(master.lookupNode(REMOTE) == STALE_URI, 'remote URI changed at cleanup commit')
            require(exited(failed['before'][REMOTE]) and not remote_processes(),
                    'remote appeared at cleanup commit')
            require(master.getParam('/gui/joint_direct_mode') is True, 'manual ownership changed')
            row['attempted'] = True; save()
            row['rpc_return'] = stale_master.unsubscribeParam(STALE_URI, key)
            row['returned_ns'] = time.time_ns()
            save()
        require(not absent_or_stale(), 'unknown stale registration remains; no expanded cleanup or launch')
        result['parameter_values_after'] = parameter_value_fingerprints()
        comparison = lambda rows: {key: (row['present'], row.get('comparison_sha256'))
                                   for key, row in rows.items()}
        result['parameter_value_comparison_excludes'] = ['/camera/runtime_profile/stationary_temporal']
        result['parameter_raw_digest_changes'] = [key for key in values_before
            if values_before[key] != result['parameter_values_after'][key]]
        require(comparison(result['parameter_values_after']) == comparison(values_before),
                'parameter configuration changed during cleanup')
        result['stale_registration_cleared'] = True
        result['cleanup_authority'] = 'seven source-derived exact caller/URI subscriptions; no parameter deletion'
        sources(release); result['idle_before_launch'] = idle(); save(); control()
        require(not absent_or_stale(), 'remote registration reappeared before launch')
        grasp_before = master.getParam('/grasp')
        require(isinstance(grasp_before, dict) and grasp_before, 'complete grasp mapping unavailable')
        grasp_bytes = json.dumps(grasp_before, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
        grasp_digest = hashlib.sha256(grasp_bytes).hexdigest()
        unit = 'alicia-gen32-recovered-remote-' + str(time.time_ns())
        command = ['systemd-run', '--user', '--collect', '--unit=' + unit,
            '--property=Type=exec', '--property=Restart=no', '--property=WorkingDirectory=' + str(W),
            '--property=StandardOutput=append:' + str(out/'remote.log'),
            '--property=StandardError=append:' + str(out/'remote.log'),
            '--setenv=ROS_LOG_DIR=' + str(out/'remote_ros'),
            '/bin/bash', str(W/'.runtime/start_gen32_remote_20261016.sh')]
        result.update(unit=unit, command=command, new_launch_attempted=True); save(); control()
        require(not absent_or_stale(), 'remote appeared at launch commit')
        recovery_bindings()
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
        sources(release); recovery_bindings()
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
