"""Persist GEN29 cold-start acceptance; GEN30 remains undeployed. Never send ROS control."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import time
from continuous_grasp_authorization_20261012 import atomic_json

W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
E = M / '实验记录/Robot-Controller'

def read(path):
    return json.loads(Path(path).read_text())

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--session', type=Path, required=True)
    ap.add_argument('--release', type=Path, required=True)
    ap.add_argument('--idle-check', type=Path, required=True)
    a = ap.parse_args()
    n, release_path = a.session.resolve(), a.release.resolve(strict=True)
    assert M.is_mount() and not os.statvfs(M).f_flag & os.ST_RDONLY
    assert E / 'ros_log' in n.parents and n.stat().st_dev == M.stat().st_dev
    deployment_path, idle_path = n/'cold_restart_acceptance/result.json', a.idle_check.resolve(strict=True)
    assert n in idle_path.parents
    startup, release, deployment, idle = map(read, (n/'startup.json', release_path, deployment_path, idle_path))
    assert all(deployment[k] for k in ('deployment_passed', 'whole_patch_deployed', 'all_processes_replaced'))
    assert deployment['source_manifest_sha256'] == startup['source_manifest_sha256'] == sha(release_path)
    assert deployment['unit'] == startup['core']['unit']
    assert deployment['simulation_unit'] == startup['simulation']['unit']
    assert idle['passed'] and idle['manual_mode'] and not idle['task_active']
    assert 0 <= time.time_ns() - idle['recorded_ns'] < 120_000_000_000
    for name, digest in release['source_sha256'].items():
        assert sha(W/name) == digest, name
    for name, identity in deployment['after'].items():
        proc = Path('/proc')/str(identity['pid'])
        fields = (proc/'stat').read_text().rsplit(')', 1)[1].split()
        assert fields[0] != 'Z' and fields[19] == identity['start_ticks'], name
        assert int(fields[1]) == identity['parent_pid'], name
        env = dict(row.split(b'=', 1) for row in (proc/'environ').read_bytes().split(b'\0') if b'=' in row)
        assert n in Path(os.fsdecode(env[b'ROS_LOG_DIR'])).resolve().parents, name
    control_path = E/'ros_log/full_latest_frozen_no_skin_20261012T021050Z/continuous_authorization_20261012/control.json'
    control = read(control_path)
    assert control['state'] == 'active'
    assert read(n/'operator_resume_instruction.json')['full_restart_authorized'] is True
    config = read(E/'grasp_records/archive_config.json')
    assert config['upload_enabled'] is False and config['delete_enabled'] is False
    assert read(E/'maintenance/storage_control.json')['archive_policy'] == 'local_only'
    backup = n/'previous_runtime_pointers.json'
    assert not backup.exists()
    pointer_names = ('latest_full_ros_start.json', 'current_resumed_grasp_session.json', 'latest_work_pause.json')
    atomic_json(backup, {name: read(W/'.runtime'/name) for name in pointer_names})
    progress = dict(startup, runtime_session=str(n), updated_ns=time.time_ns(),
        active_work='gen29_full_restart_accepted_gen30_contour_repair_pending',
        latest_deployment=str(deployment_path), deployment=str(deployment_path),
        deployment_passed=True, whole_patch_deployed=True, runtime_reload_pending=False,
        all_processes_replaced=True, deployed_node_identities=deployment['after'],
        source_count=release['source_count'], test_scope_files=release['test_scope_files'],
        current_test_result=release['actual_regression'],
        gen29_deployed=True, gen30_deployed=False,
        pending_work='GEN30 complete-instance contour consistency repair, regression and deployment remain pending.',
        manual_mode=True, task_active=False, selection=idle['selection'],
        idle_evidence=str(idle_path), automatic_grasp_started=False, automatic_grasp_running=False,
        physical_grasp_success=False, physical_grasp_verified=False,
        continuation_control_state=control['state'], continuation_control_revision=control['revision'],
        fresh_grasp_scene_required=True, previous_runtime_pointers=str(backup),
        temperature_queries_sent=False, torque_disable_commands_sent=False)
    atomic_json(n/'current_progress.json', progress)
    atomic_json(W/'.runtime/latest_full_ros_start.json', progress)
    atomic_json(W/'.runtime/current_resumed_grasp_session.json', dict(directory=str(n), runtime_session=str(n), updated_ns=time.time_ns()))
    atomic_json(W/'.runtime/latest_work_pause.json', dict(status='resumed_by_user',
        user_requested_pause=False, user_request='你重新启动各节点和上位机，然后继续处理任务',
        recorded_ns=time.time_ns(), previous_pause=str(backup),
        control_state=control['state'], control_revision=control['revision'],
        gen29_deployed=True, gen30_deployed=False, physical_grasp_verified=False,
        automatic_grasp_running=False, restart_progress=str(n/'current_progress.json')))
    atomic_json(n/'finalization.json', dict(passed=True, recorded_ns=time.time_ns(),
        current_progress_sha256=sha(n/'current_progress.json'),
        evidence_sha256={str(p):sha(p) for p in (release_path, deployment_path, idle_path, backup, Path(__file__))},
        gen29_deployed=True, gen30_deployed=False, physical_grasp_verified=False,
        motion_commands_sent=False, temperature_queried=False, automatic_grasp_started=False))
    print(json.dumps(dict(passed=True, progress=str(n/'current_progress.json')), ensure_ascii=False))

if __name__ == '__main__':
    main()
