"""Record completed node/GUI restoration without changing ROS or grasp control."""
import hashlib
import json
import os
from pathlib import Path
import time

W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
E = M / '实验记录/Robot-Controller'
N = E / 'ros_log/full_latest_gen18_restart_20261014T012216Z'

def read(path):
    return json.loads(Path(path).read_text())

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def write(path, value, replace=False):
    assert M.is_mount() and not os.statvfs(M).f_flag & os.ST_RDONLY
    if not replace:
        assert not path.exists(), str(path)
    temporary = path.with_name(path.name + '.finalize.' + str(os.getpid()) + '.tmp')
    with temporary.open('x') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
        stream.write('\n'); stream.flush(); os.fsync(stream.fileno())
    temporary.replace(path)

assert M.is_mount() and N.stat().st_dev == M.stat().st_dev
release_path = N / 'verification/gen18_release_manifest.json'
release = read(release_path)
deployment_path = N / 'cold_restart_acceptance/result.json'
deployment = read(deployment_path)
idle_path = N / 'final_idle_gui_camera/result.json'
idle = read(idle_path)
assert deployment['deployment_passed'] and deployment['whole_patch_deployed']
assert deployment['all_processes_replaced']
assert deployment['source_manifest_sha256'] == sha(release_path)
assert idle['passed'] and idle['manual_mode'] and not idle['task_active']
assert not idle['automatic_grasp_started']
assert time.time_ns() - idle['recorded_ns'] < 600 * 10**9
for name, digest in release['source_sha256'].items():
    assert sha(W / name) == digest, name
for name, identity in deployment['after'].items():
    fields = (Path('/proc') / str(identity['pid']) / 'stat').read_text().rsplit(')', 1)[1].split()
    assert fields[0] != 'Z' and fields[19] == identity['start_ticks'], name
    assert int(fields[1]) == identity['parent_pid'], name
control_path = E / 'ros_log/full_latest_frozen_no_skin_20261012T021050Z/continuous_authorization_20261012/control.json'
control = read(control_path)
assert control['state'] == 'paused'
instruction_path = N / 'operator_restart_only_instruction.json'
assert read(instruction_path)['automatic_grasp_authorized_now'] is False

local = W / '.runtime/latest_full_ros_start.json'
previous = read(local)
backup = N / 'previous_progress_before_gen18_finalization.json'
write(backup, previous)
startup_path = N / 'startup.json'
startup = read(startup_path)
startup_backup = N / 'startup_before_acceptance_finalization.json'
write(startup_backup, startup)
doc = W / 'docs/operations/2026-10-13-gen18-restart-manual-idle.md'
mirror = E / '文档' / doc.name
assert not mirror.exists()
with mirror.open('xb') as stream:
    stream.write(doc.read_bytes()); stream.flush(); os.fsync(stream.fileno())
assert sha(doc) == sha(mirror)

now = time.time_ns()
progress = dict(startup)
progress.update(updated_ns=now, recorded_ns=now,
    active_work='gen18_nodes_and_gui_restored_manual_idle_grasp_paused',
    requested_by_user='先恢复节点与上位机，暂不开始抓取',
    latest_scope_receipt=str(instruction_path),
    previous_session=previous['directory'], previous_progress_snapshot=str(backup),
    current_progress=str(N / 'current_progress.json'),
    running=True, paused=True, paused_by_user=True, pause_scope='automatic_grasp_only',
    automatic_grasp_started=False, automatic_grasp_running=False, start_reserved=False,
    active_record=None, current_record=None, physical_grasp_success=False,
    physical_grasp_verified=False, approach_executed=False, gripper_closed=False,
    last_record=previous.get('last_record'), last_result=previous.get('last_result'),
    historical_failure_details=str(backup), last_failure_evidence_doc=str(E / '文档/2026-10-13-r6-recorded-recheck-evidence.md'),
    continuing_user_authorization_state=control['state'],
    continuation_authorization=str(control_path.with_name('authorization.json')),
    continuous_control_path=str(control_path), continuous_control_revision=control['revision'],
    current_deployment=str(deployment_path), latest_deployment=str(deployment_path),
    deployment=str(deployment_path), deployment_sha256=sha(deployment_path),
    runtime_receipt=str(deployment_path), deployment_passed=True, whole_patch_deployed=True,
    runtime_reload_pending=False, all_processes_replaced=True, hardware_processes_preserved=False,
    deployed_node_identities=deployment['after'],
    current_test_result=str(N / 'verification/gen18_combined/result.json'),
    current_test_result_sha256=sha(N / 'verification/gen18_combined/result.json'),
    current_regression=str(N / 'verification/gen18_combined'),
    current_regression_summary=release['test_summary'], current_test_scope_files=184,
    current_source_count=924, source_count=924, disk_source_count=924, runtime_source_count=924,
    operational_source_sha256=release['source_sha256'], disk_source_sha256=release['source_sha256'],
    runtime_source_manifest=str(release_path), runtime_source_manifest_sha256=sha(release_path),
    source_count_scope='Release bundle count includes tests and resources; not a loaded-module count.',
    runtime_matches_disk=True, startup_runtime_verified=True,
    deployment_scope='All 11 core node identities belong to the new full cold start.',
    installed_but_not_loaded_patch=None, pending_combined_test_scope_files=0,
    gui_manual=True, manual_mode=True, manual_evidence=str(idle_path),
    verified_manual_state_evidence=str(idle_path), manual_state_is_live_snapshot=False,
    current_selection=idle['selection'], selection=idle['selection'],
    current_generation=idle['selection']['generation'],
    current_router_status=idle['mode'], task_active=False, task_state=idle['task_state'],
    grasp_ready=False, new_selection_required_before_automatic_grasp=True,
    operator_clearance_claimed=False, current_alignment_pose_is_current=False,
    final_camera_evidence=str(idle_path),
    final_scene_observation='Tabletop and small box visible; a forearm is visible at left. No clearance or alignment is certified.',
    max_supplemental_observations=2, return_to_origin=False, automatic_lift=False,
    software_temperature_monitoring_enabled=False, temperature_poll_rate_hz=0.0,
    temperature_queries_sent=False, torque_disable_commands_sent=False,
    startup_positive_enable_configured=True, local_only=True,
    free_bytes=os.statvfs(M).f_bavail * os.statvfs(M).f_frsize,
    operation_document=str(doc), operation_document_mirror=str(mirror),
    next_action='Keep nodes and GUI running in manual idle. Do not start grasp under restart-only scope. On explicit execution resume, use a new target selection and current pose/site checks.',
    pending_work=['Physical grasp validation remains unperformed for GEN18.'])
write(N / 'current_progress.json', progress)
startup.update(active_work=progress['active_work'], accepted_ns=now,
    runtime_receipt=str(deployment_path), deployment_passed=True, whole_patch_deployed=True,
    final_manual_evidence=str(idle_path), automatic_grasp_started=False,
    automatic_grasp_running=False, current_progress=str(N / 'current_progress.json'),
    operator_restart_only_instruction=str(instruction_path))
write(startup_path, startup, replace=True)
write(local, progress, replace=True)
evidence_paths = [release_path, deployment_path, idle_path, instruction_path,
    N / 'restored_status.json', N / 'current_progress.json', startup_path, backup,
    startup_backup, doc, mirror, Path(__file__).resolve()]
audit = dict(passed=True, recorded_ns=now, kind='gen18_manual_restart_finalization',
    automatic_grasp_started=False, control_state_unchanged=control['state'],
    control_revision_unchanged=control['revision'], control_file_sha256=sha(control_path),
    current_progress_sha256=sha(local), source_count=924,
    test_summary=release['test_summary'], evidence_sha256={str(p):sha(p) for p in evidence_paths})
write(N / 'finalization.json', audit)
assert sha(local) == sha(N / 'current_progress.json')
print(json.dumps(dict(passed=True, current_progress=str(N/'current_progress.json'),
    document=str(doc), automatic_grasp_started=False,
    control_state=control['state'], free_gib=progress['free_bytes']/2**30),ensure_ascii=False))
