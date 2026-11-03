"""Record a verified compute reload and idle state; never call ROS or authorize motion."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import time

from continuous_grasp_authorization_20261012 import atomic_json
from alicia_grasp_modes.selection import parse_selection

W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
E = M / '实验记录/Robot-Controller'
CONTROL = E / 'ros_log/full_latest_frozen_no_skin_20261012T021050Z/continuous_authorization_20261012/control.json'


def read(path):
    return json.loads(Path(path).read_text())


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ('session', 'release', 'deployment', 'idle-check'):
        parser.add_argument('--' + key, required=True, type=Path)
    args = parser.parse_args()
    session, release_path, deployment_path, idle_path = (
        path.resolve(strict=True) for path in
        (args.session, args.release, args.deployment, args.idle_check))
    assert M.is_mount() and not os.statvfs(M).f_flag & os.ST_RDONLY
    assert E / 'ros_log' in session.parents and session.stat().st_dev == M.stat().st_dev
    assert session in deployment_path.parents and session in idle_path.parents
    assert E in release_path.parents
    release, deployment, idle = map(read, (release_path, deployment_path, idle_path))
    assert all(release[key] is True for key in ('passed', 'reviewed', 'installed'))
    assert release['test_summary']['tests'] > 7125
    assert all(release['test_summary'][key] == 0 for key in ('failures', 'errors', 'skipped'))
    assert all(deployment[key] is True for key in
               ('deployment_passed', 'whole_patch_deployed', 'compute_reloaded', 'hardware_processes_preserved'))
    assert Path(deployment['source_manifest']).resolve() == release_path
    assert deployment['source_manifest_sha256'] == sha(release_path)
    assert deployment['source_sha256'] == release['source_sha256']
    reload_path = Path(deployment['reload_evidence']).resolve(strict=True)
    assert session in reload_path.parents and sha(reload_path) == deployment['reload_sha256']
    reloaded = read(reload_path)
    for key in ('deployment_passed', 'compute_reloaded', 'hardware_processes_preserved',
                'source_manifest', 'source_manifest_sha256', 'source_sha256',
                'before', 'after', 'preserved_before', 'preserved_after', 'selection',
                'previous_deployment', 'previous_deployment_sha256'):
        assert reloaded[key] == deployment[key], key
    compute = {'/grasp_task_node', '/remote_grasp6d_node', '/unknown_tabletop_perception'}
    hardware = {'/alicia_d_driver_node', '/bessica_d_hw_interface', '/move_group',
                '/motion_gateway', '/supervisor_camera', '/compliant_gripper_controller',
                '/grasp_mode_router', '/alicia_supervisor_gui'}
    assert set(deployment['after']) == set(deployment['before']) == compute
    assert set(deployment['preserved_after']) == hardware
    assert deployment['preserved_after'] == deployment['preserved_before']
    previous_path = Path(deployment['previous_deployment']).resolve(strict=True)
    assert E / 'ros_log' in previous_path.parents
    assert sha(previous_path) == deployment['previous_deployment_sha256']
    previous = read(previous_path)
    assert previous['deployment_passed'] and previous['whole_patch_deployed']
    assert previous['source_manifest_sha256'] == release['baseline_release_sha256']
    assert dict(deployment['before'], **deployment['preserved_before']) == dict(
        previous['after'], **previous.get('preserved_after', {}))
    process_key = lambda row: (row['pid'], row['start_ticks'])
    assert all(process_key(deployment['before'][key]) != process_key(deployment['after'][key])
               for key in compute)
    assert idle['passed'] is True and idle['manual_mode'] is True and idle['task_active'] is False
    assert parse_selection(idle['selection']) == parse_selection(deployment['selection'])
    assert idle['source_matches_operational_release'] is True
    assert idle['deployment_processes_unchanged'] is True and idle['sample_count'] >= 20
    assert Path(idle['release']).resolve() == release_path
    assert Path(idle['deployment']).resolve() == deployment_path
    assert idle['source_count'] == release['source_count']
    assert idle['operational_validation'] == release['test_summary']
    assert idle['motion_commands_sent'] is False and idle['temperature_queried'] is False
    identities = dict(deployment['after'], **deployment['preserved_after'])
    assert len({process_key(row) for row in identities.values()}) == 11

    def require_current_idle_processes():
        assert 0 <= time.time_ns() - idle['recorded_ns'] < 120_000_000_000
        for name, identity in identities.items():
            proc = Path('/proc') / str(identity['pid'])
            fields = (proc / 'stat').read_text().rsplit(')', 1)[1].split()
            assert fields[0] != 'Z' and fields[19] == identity['start_ticks'], name
            assert int(fields[1]) == identity['parent_pid'], name
            env = dict(row.split(b'=', 1) for row in (proc / 'environ').read_bytes().split(b'\0') if b'=' in row)
            assert session in Path(os.fsdecode(env[b'ROS_LOG_DIR'])).resolve().parents, name

    require_current_idle_processes()
    for name, digest in release['source_sha256'].items():
        path = Path(name)
        assert not path.is_absolute() and '..' not in path.parts and sha(W / path) == digest
    for name, digest in release['evidence_sha256'].items():
        assert sha(name) == digest, name
    config = read(E / 'grasp_records/archive_config.json')
    assert config['upload_enabled'] is False and config['delete_enabled'] is False
    assert read(E / 'maintenance/storage_control.json')['archive_policy'] == 'local_only'
    pointers = ('latest_full_ros_start.json', 'current_resumed_grasp_session.json', 'latest_work_pause.json')
    progress_path = session / 'current_progress.json'
    progress = read(progress_path)
    assert Path(progress['runtime_session']).resolve() == session
    assert Path(progress['latest_deployment']).resolve() == previous_path
    backup = session / 'before_gen30_compute_progress.json'
    receipt = session / 'gen30_deployment_finalization.json'
    assert not backup.exists() and not receipt.exists()
    bound = {str(path): sha(path) for path in
             (release_path, deployment_path, reload_path, previous_path, idle_path, progress_path, Path(__file__))}
    with (CONTROL.parent / 'control.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        control = read(CONTROL)
        assert control['state'] == 'active'
        assert all(sha(path) == digest for path, digest in bound.items())
        require_current_idle_processes()
        assert M.is_mount() and session.stat().st_dev == M.stat().st_dev
        atomic_json(backup, dict(progress=progress,
            pointers={name: read(W / '.runtime' / name) for name in pointers}))
        progress.update(updated_ns=time.time_ns(),
            active_work='gen30_source_topology_deployed_fresh_grasp_scene_pending',
            source_manifest=str(release_path), source_manifest_sha256=sha(release_path),
            test_summary=release['test_summary'], source_count=release['source_count'],
            test_scope_files=release['test_scope_files'], current_test_result=release['actual_regression'],
            latest_deployment=str(deployment_path), deployment=str(deployment_path),
            deployment_passed=True, whole_patch_deployed=True, runtime_reload_pending=False,
            all_processes_replaced=False, compute_reloaded=True, hardware_processes_preserved=True,
            deployed_node_identities=identities, gen29_deployed=False, gen30_deployed=True,
            pending_work='Fresh scene and measured readiness, new selection, then the authorized recorded grasp.',
            idle_evidence=str(idle_path), manual_mode=True, task_active=False, selection=idle['selection'],
            automatic_grasp_started=False, automatic_grasp_running=False,
            physical_grasp_success=False, physical_grasp_verified=False,
            fresh_grasp_scene_required=True, continuation_control_state='active',
            continuation_control_revision=control['revision'],
            temperature_queries_sent=False, torque_disable_commands_sent=False)
        atomic_json(progress_path, progress)
        atomic_json(W / '.runtime/latest_full_ros_start.json', progress)
        atomic_json(W / '.runtime/current_resumed_grasp_session.json', dict(
            directory=str(session), runtime_session=str(session), updated_ns=time.time_ns()))
        pause = read(W / '.runtime/latest_work_pause.json')
        pause.update(status='resumed_by_user', user_requested_pause=False, recorded_ns=time.time_ns(),
            control_state='active', control_revision=control['revision'], gen29_deployed=False,
            gen30_deployed=True, physical_grasp_verified=False, automatic_grasp_running=False,
            restart_progress=str(progress_path))
        atomic_json(W / '.runtime/latest_work_pause.json', pause)
        atomic_json(receipt, dict(passed=True, recorded_ns=time.time_ns(),
            input_evidence_sha256=bound, previous_progress_archived=str(backup),
            backup_sha256=sha(backup), current_progress_sha256=sha(progress_path),
            gen30_deployed=True, motion_commands_sent=False, temperature_queried=False,
            physical_grasp_verified=False, new_grasp_authorized=False))
    print(json.dumps(dict(passed=True, receipt=str(receipt), sha256=sha(receipt)), ensure_ascii=False))


if __name__ == '__main__':
    main()
