"""Finalize a verified remote-and-task GEN34 deployment using fresh idle evidence; no ROS calls."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import time

from continuous_grasp_authorization_20261012 import atomic_json
from alicia_grasp_modes.selection import parse_selection
from reload_gen34_remote_task_20261016 import (
    W, M, E, CONTROL, REMOTE, TASK, TARGETS, PRESERVE, HELPERS, ORIGINAL_PARENT, original_parent, EXPECTED_CONTROL_REVISION,
    EXPECTED_GENERATION, read, sha, require, storage, control, sources, proc_identity)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ('session', 'release', 'deployment', 'idle-check'):
        parser.add_argument('--' + key, required=True, type=Path)
    args = parser.parse_args()
    session, release_path, deployment_path, idle_path = (
        path.resolve(strict=True) for path in
        (args.session, args.release, args.deployment, args.idle_check))
    storage(session / 'gen34_deployment_finalization.json')
    require(deployment_path == session/'gen34_runtime_deployment.json'
            and session in idle_path.parents and E in release_path.parents, 'input paths differ')
    release, deployment, idle = map(read, (release_path, deployment_path, idle_path))
    require(release['kind'] == 'gen34_post_observation_box_precision_routing'
            and all(release[k] is True for k in ('passed', 'reviewed', 'installed')), 'release not sealed')
    require(release['test_summary']['tests'] > 7483
            and all(release['test_summary'][k] == 0 for k in ('failures', 'errors', 'skipped')),
            'complete GEN34 regression not passed')
    require(all(deployment[k] is True for k in ('deployment_passed', 'whole_patch_deployed',
            'remote_reloaded', 'task_reloaded', 'compute_reloaded', 'hardware_processes_preserved',
            'other_nine_processes_preserved', 'old_processes_exited', 'original_parent_preserved',
            'stale_registration_cleanup_complete', 'parameter_configuration_unchanged')), 'remote deployment incomplete')
    require(Path(deployment['source_manifest']).resolve() == release_path
            and deployment['source_manifest_sha256'] == sha(release_path)
            and deployment['source_sha256'] == release['source_sha256'], 'release binding differs')
    require(deployment['control_revision'] == EXPECTED_CONTROL_REVISION, 'reload authority differs')
    reload_path = Path(deployment['reload_evidence']).resolve(strict=True)
    require(session in reload_path.parents and sha(reload_path) == deployment['reload_sha256'],
            'reload evidence changed')
    reloaded = read(reload_path)
    require({k:v for k,v in deployment.items() if k not in ('reload_evidence', 'reload_sha256')} == reloaded,
            'deployment differs from exact reload result')
    previous_path = Path(deployment['previous_deployment']).resolve(strict=True)
    require(previous_path.parent == session and sha(previous_path) == deployment['previous_deployment_sha256'],
            'previous deployment changed')
    previous = read(previous_path)
    require(previous['deployment_passed'] and previous['whole_patch_deployed']
            and previous['source_manifest_sha256'] == release['baseline_release_sha256']
            and Path(release['previous_deployment']).resolve() == previous_path, 'wrong baseline deployment')
    require(set(deployment['before']) == set(deployment['after']) == set(TARGETS)
            and set(deployment['preserved_before']) == set(deployment['preserved_after']) == set(PRESERVE),
            'remote/task node scope differs')
    require(deployment['preserved_before'] == deployment['preserved_after'], 'preserved process changed')
    require(dict(deployment['before'], **deployment['preserved_before']) == dict(
            previous['after'], **previous['preserved_after']), 'before identities differ from baseline')
    key = lambda row: (row['pid'], row['start_ticks'])
    require(all(key(deployment['before'][n]) != key(deployment['after'][n]) for n in TARGETS), 'old target reused')
    require(all(deployment['after'][n]['parent_pid'] == deployment['unit_main_pid'] for n in TARGETS),
            'new target unit parent differs')
    require(deployment['original_parent_before'] == deployment['original_parent_after'] == ORIGINAL_PARENT,
            'original producer launcher identity differs')
    identities = dict(deployment['after'], **deployment['preserved_after'])
    require(len({key(row) for row in identities.values()}) == 11, 'duplicate node process')
    selection = parse_selection(deployment['selection'])
    require(selection['generation'] == EXPECTED_GENERATION and selection['mode'] == 'unknown'
            and selection['strategy'] == 'observe_return', 'unexpected deployment selection')
    for service in ('/grasp_mode/reset_remote', '/grasp_mode/reset_task'):
        ack = deployment['reset_ack'][service]
        require(ack['ready'] is True and parse_selection(ack) == selection, 'reset acknowledgement differs')
    require(deployment['startup_dependencies_sha256'] == {name:sha(W/name) for name in HELPERS},
            'startup helper changed')
    startup = deployment['startup_configuration']
    prefix_path = Path(startup['frozen_log_prefix']).resolve(strict=True)
    require(prefix_path.parent == reload_path.parent
            and sha(prefix_path) == startup['frozen_log_prefix_sha256']
            and prefix_path.stat().st_size == startup['live_log_prefix_bytes']
            and startup['master_mapping_unchanged'] is True and startup['current_master_matches'] is True,
            'startup configuration evidence invalid')
    require(idle['passed'] is True and idle['manual_mode'] is True and idle['task_active'] is False,
            'fresh idle check not accepted')
    require(parse_selection(idle['selection']) == selection
            and idle['epoch_ns'] == deployment['epoch_ns'], 'selection or reference epoch changed')
    require(not any(idle['mode'].get(k) for k in ('active', 'start_reserved', 'pending')), 'router active')
    require(idle['source_matches_operational_release'] is True
            and idle['deployment_processes_unchanged'] is True and idle['sample_count'] >= 20,
            'idle source/process/measured feedback validation missing')
    require(Path(idle['release']).resolve() == release_path
            and Path(idle['deployment']).resolve() == deployment_path
            and idle['source_count'] == release['source_count']
            and idle['operational_validation'] == release['test_summary'], 'idle evidence source scope differs')
    require(idle['motion_commands_sent'] is False and idle['temperature_queried'] is False,
            'idle helper was not read-only')

    def current_processes():
        original_parent()
        require(proc_identity(deployment['unit_main_pid']) == deployment['unit_identity'],
                'new launcher identity changed')
        require(0 <= time.time_ns() - idle['recorded_ns'] < 120_000_000_000, 'idle evidence expired')
        for name, identity in identities.items():
            require(proc_identity(identity['pid']) == identity, 'process identity changed: ' + name)
            proc = Path('/proc')/str(identity['pid'])
            env = dict(row.split(b'=',1) for row in (proc/'environ').read_bytes().split(b'\0') if b'=' in row)
            require(session in Path(os.fsdecode(env[b'ROS_LOG_DIR'])).resolve().parents,
                    'node logs outside external session: ' + name)

    current_processes(); sources(release); control()
    progress_path = session / 'current_progress.json'
    progress = read(progress_path)
    require(Path(progress['runtime_session']).resolve() == session
            and Path(progress['latest_deployment']).resolve() == previous_path,
            'current progress no longer points to the prior deployment')
    pointers = ('latest_full_ros_start.json', 'current_resumed_grasp_session.json', 'latest_work_pause.json')
    pointer_paths = [W/'.runtime'/name for name in pointers]
    backup = session / 'before_gen34_remote_task_progress.json'
    receipt = session / 'gen34_deployment_finalization.json'
    require(not backup.exists() and not receipt.exists(), 'GEN34 finalization already started')
    bound = {str(path):sha(path) for path in
             (release_path, deployment_path, reload_path, previous_path, idle_path,
              prefix_path, progress_path, Path(__file__), *pointer_paths)}
    with (CONTROL.parent / 'control.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        authorized = control()
        require(all(sha(path) == digest for path,digest in bound.items()), 'bound evidence or pointer changed')
        sources(release); current_processes(); storage(receipt)
        require(all(sha(path) == digest for path,digest in bound.items()),
                'bound evidence or pointer changed before finalization')
        atomic_json(backup, dict(progress=progress,
            pointers={name:read(W/'.runtime'/name) for name in pointers}))
        progress.update(updated_ns=time.time_ns(),
            active_work='gen34_post_observation_box_precision_routing_deployed_fresh_scene_pending',
            status='gen34_deployed_fresh_grasp_checks_pending',
            source_manifest=str(release_path), source_manifest_sha256=sha(release_path),
            test_summary=release['test_summary'], source_count=release['source_count'],
            test_scope_files=release['test_scope_files'], current_test_result=release['actual_regression'],
            latest_deployment=str(deployment_path), deployment=str(deployment_path),
            deployment_passed=True, whole_patch_deployed=True, runtime_reload_pending=False,
            all_processes_replaced=False, compute_reloaded=True, remote_reloaded=True, task_reloaded=True,
            hardware_processes_preserved=True, other_nine_processes_preserved=True,
            other_ten_processes_preserved=False, original_parent_preserved=True,
            deployment_kind='remote_and_task_nine_processes_preserved',
            deployed_node_identities=identities, gen30_deployed=False, gen31_deployed=False, gen32_deployed=False, gen33_deployed=False, gen34_deployed=True,
            pending_work='Fresh scene and measured readiness, new selection, then the authorized recorded grasp.',
            idle_evidence=str(idle_path), latest_idle_evidence=str(idle_path),
            manual_mode=True, task_active=False, task_state=idle['task_state'], selection=idle['selection'],
            automatic_grasp_started=False, automatic_grasp_running=False,
            physical_grasp_success=False, physical_grasp_verified=False,
            fresh_grasp_scene_required=True, continuation_control_state='active',
            continuation_control_revision=authorized['revision'],
            control_state='active', control_revision=authorized['revision'], user_requested_pause=False,
            temperature_queries_sent=False, torque_disable_commands_sent=False)
        atomic_json(progress_path, progress)
        atomic_json(W/'.runtime/latest_full_ros_start.json', progress)
        atomic_json(W/'.runtime/current_resumed_grasp_session.json', dict(
            directory=str(session), runtime_session=str(session), updated_ns=time.time_ns()))
        pause = read(W/'.runtime/latest_work_pause.json')
        pause.update(status='resumed_by_user', user_requested_pause=False, recorded_ns=time.time_ns(),
            control_state='active', control_revision=authorized['revision'],
            gen30_deployed=False, gen31_deployed=False, gen32_deployed=False, gen33_deployed=False, gen34_deployed=True,
            physical_grasp_verified=False, automatic_grasp_running=False, restart_progress=str(progress_path))
        atomic_json(W/'.runtime/latest_work_pause.json', pause)
        atomic_json(receipt, dict(passed=True, recorded_ns=time.time_ns(),
            input_evidence_sha256=bound, previous_progress_archived=str(backup),
            backup_sha256=sha(backup), current_progress_sha256=sha(progress_path),
            pointer_sha256={name:sha(W/'.runtime'/name) for name in pointers},
            gen34_deployed=True, remote_reloaded=True, task_reloaded=True, other_nine_processes_preserved=True,
            motion_commands_sent=False, temperature_queried=False,
            physical_grasp_verified=False, new_grasp_authorized=False))
    print(json.dumps(dict(passed=True, receipt=str(receipt), sha256=sha(receipt)), ensure_ascii=False))


if __name__ == '__main__':
    main()
