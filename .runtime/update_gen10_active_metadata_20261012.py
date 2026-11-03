#!/usr/bin/env python3
"""Bind GEN10 running metadata and add its recording to storage protection."""
from pathlib import Path
import fcntl
import hashlib
import json
import os
import time

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
S = E / 'ros_log/full_latest_frozen_no_skin_20261012T021050Z'
R = E / 'grasp_records/20261012T131015.007765Z_ca31b008d8ab4f9aa749a6c9ef9032a5'
OUT = S / 'verification/gen10_finished_progress_update'
DOC = W / 'docs/operations/2026-10-11-frozen-grasp-acceptance.md'
PROTECTED = E / 'maintenance/failed_record_space_guard_protected.json'


def sha(data):
    return hashlib.sha256(data).hexdigest()


def atomic(path, data):
    tmp = path.with_name(path.name + '.active-metadata-tmp')
    tmp.write_bytes(data)
    os.replace(tmp, path)


assert Path('/media/zhuyupei/SanStand').is_mount()
clear_path = S / 'gen10_continuation_scene/scene_clear_receipt.json'
clear_bytes = clear_path.read_bytes()
clear = json.loads(clear_bytes)
assert clear['message_id'] == 'call_8e456d3fb70548a2a82141f0100a4059:0'
assert clear['user_message'] == '双手已移到活动范围之外'
assert clear['purpose'] == 'current_scene_clearance_only' and clear['new_motion_authorization_claimed'] is False
control_path = S / 'continuous_authorization_20261012/control.json'
control_bytes = control_path.read_bytes()
assert json.loads(control_bytes)['state'] == 'active'
manifest_bytes = (R / 'manifest.json').read_bytes()
manifest = json.loads(manifest_bytes)
assert manifest['state'] == 'complete' and manifest['recorder_closed'] and manifest['result'] == 'failure'
verified_files = []
for item in manifest['files']:
    path = R / item['path']
    assert path.resolve().parent == R.resolve() or R.resolve() in path.resolve().parents
    assert path.is_file() and not path.is_symlink()
    before_stat = path.stat()
    digest = hashlib.sha256()
    with path.open('rb') as source:
        for chunk in iter(lambda: source.read(4 * 1024 * 1024), b''):
            digest.update(chunk)
    after_stat = path.stat()
    assert (before_stat.st_ino, before_stat.st_size, before_stat.st_mtime_ns) == (after_stat.st_ino, after_stat.st_size, after_stat.st_mtime_ns)
    assert after_stat.st_size == item['size_bytes'] and digest.hexdigest() == item['sha256']
    verified_files.append(dict(path=item['path'], size_bytes=after_stat.st_size, sha256=digest.hexdigest()))
assert (R / 'manifest.json').read_bytes() == manifest_bytes
recorder = json.loads((R / 'recorder_outcome.json').read_bytes())
assert recorder['bag_closed'] and recorder['active_seen']
manual_path = S / 'gen10_finished_manual/result.json'
idle_path = S / 'gen10_finished_idle_verified/result.json'
manual, idle = json.loads(manual_path.read_bytes()), json.loads(idle_path.read_bytes())
assert manual['manual_restored'] and manual['gui_own_publisher_acknowledged'] and manual['verified_inactive']
assert idle['passed'] and idle['manual_mode'] and not idle['task_active']
active_path = S / 'gen10_continuation_scene/active_record.json'
active = json.loads(active_path.read_bytes())
assert active['record_dir'] == str(R)
release = json.loads((S / 'verification/gen9_ready_for_gen10_release_manifest.json').read_bytes())
assert len(release['source_sha256']) == 873
current_path = S / 'current_progress.json'
latest_path = W / '.runtime/latest_full_ros_start.json'
mirrors = [E / '文档' / DOC.name, S / DOC.name]
targets = [DOC, current_path, latest_path, PROTECTED] + mirrors
for path in targets:
    try:
        rel = path.relative_to(W)
    except ValueError:
        continue
    assert str(rel) not in release['source_sha256']

with (E / 'maintenance/failed_record_space_guard_lock').open('a') as guard_lock:
    fcntl.flock(guard_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    before = {p: p.read_bytes() for p in targets}
    now = time.time_ns()
    current = json.loads(before[current_path])
    assert current.get('current_generation') in (9, 10)
    if current.get('current_generation') == 10:
        assert current.get('current_record') == str(R)
    patch = dict(updated_ns=now,
        active_work='investigating_gen10_authority_revocation',
        current_generation=10,
        current_selection=dict(mode='unknown', strategy='observe_return', generation=10, stamp_ns=1791810616887247323),
        current_record=str(R), active_record=None,
        last_record=str(R), last_result='failure', last_failure_reason=manifest['failure_reason'],
        previous_completed_record=current.get('last_record'), previous_completed_result=current.get('last_result'),
        automatic_grasp_started=True, automatic_grasp_running=False,
        running=False, recording_started=False, next_trial_started=False,
        next_generation_expected=11, current_trial_result='failure',
        awaiting_scene_clear_confirmation=False, scene_clear_confirmed=True,
        scene_clear_confirmation_receipt=str(clear_path),
        scene_clear_confirmation_sha256=sha(clear_bytes), scene_clear_confirmation_message_id=clear['message_id'],
        continuing_user_authorization_state='active', awaiting_user_authorization=False,
        hand_clear_confirmation=str(clear_path),
        scene_clear_confirmation_reason='Resolved by the actual user reply and fresh scene/feedback review before GEN10.',
        gen10_watch_directory=str(S / 'gen10_watch'),
        gen10_resume_readiness=str(S / 'gen10_continuation_readiness'),
        gen10_active_record_receipt=str(active_path),
        gen10_launcher_pid=active['launcher_pid'], gen10_selection_pid=active['selection_pid'],
        initial_plan_id='1000e0a8cf268b26c6dd6fda',
        gen10_finished_snapshot=dict(recorded_ns=now, initial_task_state_reported_by_root='MOVE_PREGRASP',
                                 final_result='failure', physical_grasp_verified=False,
                                 recorder_closed=True, manifest_file_count=len(verified_files)),
        gen10_finished_manual=str(manual_path), gen10_finished_idle=str(idle_path),
        gen10_record_verification=str(OUT / 'manifest_verification.json'),
        gen10_observer_stop_receipt=str(S / 'gen10_watch/stop_receipt.json'),
        gui_manual=True, gui_manual_restored=True,
        physical_grasp_success=False, physical_grasp_verified=False,
        next_action='Investigate GEN10 frozen execution authority revocation at supplemental admission; preserve all sealed recording and prior evidence under the existing active continuous authorization.',
        resume_note='GEN10 failed during supplemental admission after actual scene-clear reply and fresh image/20-feedback review. Recording is sealed; GUI manual and idle verified. Continuous authorization remains active; no physical grasp success is claimed.',
        active_progress_update_receipt=str(OUT / 'receipt.json'))
    updates = {}
    for path in [current_path, latest_path]:
        value = json.loads(before[path])
        value.update(patch)
        updates[path] = (json.dumps(value, ensure_ascii=False, indent=2) + '\n').encode()
    protection = json.loads(before[PROTECTED])
    old_entries = list(protection['protected_paths'])
    gen9 = 'grasp_records/20261012T122920.925646Z_9a08734d03f247ac895e5b762b04c615'
    assert any(row['path'] == gen9 for row in old_entries)
    relative_record = str(R.relative_to(E))
    if not any(row['path'] == relative_record for row in old_entries):
        protection['protected_paths'].append(dict(path=relative_record,
            reason='GEN10 supplemental execution-authority revocation investigation; preserve all sealed raw recording and current selected-height validation evidence.'))
    assert protection['protected_paths'][:len(old_entries)] == old_entries
    updates[PROTECTED] = (json.dumps(protection, ensure_ascii=False, indent=2) + '\n').encode()
    paragraph = f'''

### 第10轮：补视准入时执行权限被撤销

用户实际回复 `call_8e456d3fb70548a2a82141f0100a4059:0`“双手已移到活动范围之外”，回执为 `gen10_continuation_scene/scene_clear_receipt.json`；这是当前现场清空确认，既有持续抓取授权仍为active。主代理重新核验现场图及20条实际关节反馈后启动第10轮，selection10/stamp1791810616887247323，记录 `20261012T131015.007765Z_ca31b008d8ab4f9aa749a6c9ef9032a5`，初始plan `1000e0a8cf268b26c6dd6fda`。启动时任务为MOVE_PREGRASP，原现场清空等待已解除。

第一观察位之后，本轮在补视准入时记录 `OBSERVE_RETURN_SUPPLEMENTAL_FAILED: EXECUTION_AUTHORITY_REVOKED: frozen execution authority was revoked by a hard safety event`，未进入闭爪；最终manifest于2026-10-12T13:11:24.714083Z封口为complete／failure，recorder_closed=true。这条通用撤销信息本身不能定位具体触发原因，正在核查交接证据与回调次序；没有写成本修复实物验收或抓取成功。

录制自然停止，实际{recorder['keyframe_count']}组关键帧。按manifest实际清单逐项实读核验 **{len(verified_files)}份文件** 的大小及SHA256全部匹配，包含328,577,143字节bag，SHA256 `4bb04f17f5741c9e1d9df9fe380e25c0bf72575e4bcf9742c625c3b76e69dd2c`；核验清单为 `verification/gen10_finished_progress_update/manifest_verification.json`。GUI自身手动回传、任务及两动作空闲核验通过，见 `gen10_finished_manual/result.json`、`gen10_finished_idle_verified/result.json`；873份来源及进程身份保持。观察器由主代理封口，记录 `gen10_watch/stop_receipt.json`。

本轮记录已加入 `maintenance/failed_record_space_guard_protected.json`；第9轮及全部既有保护条目保留，没有删除录制或更改归档策略。当前进度为failure sealed、active_record=null、automatic_grasp_running=false，工作状态为调查GEN10权限撤销；第9轮失败历史保持，持续授权仍为active。
'''
    assert '### 第10轮：补视准入时执行权限被撤销' not in before[DOC].decode()
    doc_bytes = before[DOC] + paragraph.encode()
    updates[DOC] = doc_bytes
    for path in mirrors:
        updates[path] = doc_bytes
    for path, data in before.items():
        assert path.read_bytes() == data, 'Concurrent metadata update: ' + str(path)
    live = json.loads((R / 'manifest.json').read_bytes())
    assert live['state'] == 'complete' and live['recorder_closed'] and live['result'] == 'failure'
    assert control_path.read_bytes() == control_bytes
    OUT.mkdir(exist_ok=False)
    (OUT / 'before').mkdir()
    backups = []
    for index, (path, data) in enumerate(before.items()):
        backup = OUT / 'before' / ('%02d_' % index + path.name)
        backup.write_bytes(data)
        backups.append(dict(path=str(path), backup=str(backup), sha256=sha(data)))
    (OUT / 'record_manifest_finished_snapshot.json').write_bytes(manifest_bytes)
    (OUT / 'manifest_verification.json').write_text(json.dumps(dict(
        passed=True, record_path=str(R), manifest_sha256=sha(manifest_bytes),
        recorded_ns=now, file_count=len(verified_files), files=verified_files,
        keyframe_count=recorder['keyframe_count'], recorder_closed=True,
        terminal_result='failure', physical_grasp_success_claimed=False), ensure_ascii=False, indent=2) + '\n')
    for path, data in updates.items():
        atomic(path, data)
        assert path.read_bytes() == data
    assert clear_path.read_bytes() == clear_bytes and control_path.read_bytes() == control_bytes
    receipt = dict(passed=True, recorded_ns=now, scope='documentation_and_finished_metadata_only',
        current_record=str(R), generation=10, selection_stamp_ns=1791810616887247323,
        initial_plan_id='1000e0a8cf268b26c6dd6fda', terminal_result='failure', manifest_file_count=len(verified_files),
        scene_clear_receipt_sha256=sha(clear_bytes), awaiting_scene_clear_confirmation=False,
        continuous_authorization_state='active', document_sha256=sha(doc_bytes),
        document_paths=[str(DOC)] + [str(p) for p in mirrors],
        protected_path_added=relative_record, previous_protected_entry_count=len(old_entries),
        protected_entry_count=len(protection['protected_paths']), all_prior_protections_preserved=True,
        updated_sha256={str(p): sha(data) for p, data in updates.items()}, before=backups,
        release_source_files_written=[], files_deleted=[], ros_commands_sent=False, temperature_queried=False)
    (OUT / 'receipt.json').write_text(json.dumps(receipt, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps({'passed': True, 'document_sha256': sha(doc_bytes),
                      'protected_count': len(protection['protected_paths']), 'receipt': str(OUT / 'receipt.json')}, ensure_ascii=False))
