"""Save accepted manual GEN36 restart without resuming grasp authorization."""
import argparse
import json
import os
from pathlib import Path
import time

from continuous_grasp_authorization_20261012 import atomic_json
from verify_gen36_offline_tests_20261016 import W, E, M, sha


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--release', type=Path, required=True)
    args = parser.parse_args()
    N, release_path = args.session.resolve(), args.release.resolve(strict=True)
    assert M.is_mount() and N.stat().st_dev == M.stat().st_dev
    N.relative_to(E/'ros_log')
    startup = json.loads((N/'startup.json').read_text())
    release = json.loads(release_path.read_text())
    accepted_path = N/'cold_restart_acceptance/result.json'
    accepted = json.loads(accepted_path.read_text())
    request = json.loads((N/'operator_restart_instruction.json').read_text())
    assert request['full_restart_authorized'] and not request['automatic_grasp_authorized_by_this_request']
    assert all(accepted[k] for k in ('deployment_passed', 'whole_patch_deployed', 'all_processes_replaced'))
    assert accepted['source_manifest_sha256'] == startup['source_manifest_sha256'] == sha(release_path)
    assert accepted['unit'] == startup['core']['unit'] and accepted['simulation_unit'] == startup['simulation']['unit']
    assert accepted['parameters']['/gui/joint_direct_mode'] is True and not accepted['task_active']
    assert accepted['parameters']['/grasp/unknown_observe_return_max_observations'] == 1
    assert 0 <= time.time_ns()-accepted['recorded_ns'] < 120_000_000_000
    assert all(sha(W/name) == digest for name, digest in release['source_sha256'].items())
    assert all(sha(path) == digest for path, digest in release['evidence_sha256'].items())
    for name, identity in accepted['after'].items():
        proc = Path('/proc')/str(identity['pid'])
        fields = (proc/'stat').read_text().rsplit(')', 1)[1].split()
        assert fields[0] != 'Z' and fields[19] == identity['start_ticks'] and int(fields[1]) == identity['parent_pid'], name
        env = dict(row.split(b'=', 1) for row in (proc/'environ').read_bytes().split(b'\0') if b'=' in row)
        assert N in Path(os.fsdecode(env[b'ROS_LOG_DIR'])).resolve().parents, name
    control_path = E/'ros_log/full_latest_frozen_no_skin_20261012T021050Z/continuous_authorization_20261012/control.json'
    control = json.loads(control_path.read_text())
    assert control['state'] == accepted['continuation_control_state'] == 'paused'
    assert control['revision'] == accepted['continuation_control_revision']
    pointer_names = ('latest_full_ros_start.json', 'current_resumed_grasp_session.json', 'latest_work_pause.json')
    backup = N/'previous_runtime_pointers.json'
    assert not backup.exists()
    atomic_json(backup, {name:json.loads((W/'.runtime'/name).read_text()) for name in pointer_names})
    progress = dict(startup, updated_ns=time.time_ns(), runtime_session=str(N),
        status='gen36_full_restart_accepted_manual_grasp_paused', active_work='manual_idle_after_requested_full_restart',
        latest_deployment=str(accepted_path), deployment=str(accepted_path),
        deployment_passed=True, whole_patch_deployed=True, all_processes_replaced=True,
        runtime_reload_pending=False, gen35_deployed=False, gen36_deployed=True,
        deployed_node_identities=accepted['after'], source_count=release['source_count'],
        test_scope_files=release['test_scope_files'], current_test_result=release['actual_regression'],
        manual_mode=True, task_active=False, actions_active=False, task_state=accepted['task_state'],
        selection=accepted['selection'], idle_evidence=str(accepted_path),
        automatic_grasp_started=False, automatic_grasp_running=False, physical_grasp_verified=False,
        control_state=control['state'], control_revision=control['revision'], hardware_motion_paused=True,
        new_grasp_authorized_in_current_scope=False, fresh_grasp_scene_required=True,
        pending_work='Full restart completed; automatic grasp remains paused. Offline SDK-wait recovery and actual jaw-aperture verification remain separate unresolved work.',
        previous_runtime_pointers=str(backup))
    atomic_json(N/'current_progress.json', progress)
    atomic_json(W/'.runtime/latest_full_ros_start.json', progress)
    atomic_json(W/'.runtime/current_resumed_grasp_session.json', dict(directory=str(N), runtime_session=str(N),
        updated_ns=time.time_ns(), control_state='paused', control_revision=control['revision']))
    atomic_json(W/'.runtime/latest_work_pause.json', dict(status='hardware_grasp_paused_after_full_restart',
        user_requested_pause=False, software_work_active=False, control_state='paused',
        control_revision=control['revision'], user_request=request['user_message'],
        gen36_deployed=True, automatic_grasp_running=False, physical_grasp_verified=False,
        restart_progress=str(N/'current_progress.json'), pending_work=progress['pending_work']))
    atomic_json(N/'finalization.json', dict(passed=True, recorded_ns=time.time_ns(),
        evidence_sha256={str(p):sha(p) for p in (release_path, accepted_path, backup, Path(__file__))},
        current_progress_sha256=sha(N/'current_progress.json'), gen36_deployed=True,
        automatic_grasp_started=False, physical_grasp_verified=False,
        motion_commands_sent=False, temperature_queried=False))
    print(json.dumps(dict(passed=True, progress=str(N/'current_progress.json'))))


if __name__ == '__main__':
    main()
