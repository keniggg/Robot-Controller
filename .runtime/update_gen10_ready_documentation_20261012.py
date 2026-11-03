#!/usr/bin/env python3
"""Documentation/status update only, after the GEN9 fix deployment."""
from pathlib import Path
import hashlib
import json
import os
import re
import time

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
S = E / 'ros_log/full_latest_frozen_no_skin_20261012T021050Z'
DOC = W / 'docs/operations/2026-10-11-frozen-grasp-acceptance.md'
OUT = S / 'verification/gen10_deployed_documentation_update'
RELEASE = S / 'verification/gen9_ready_for_gen10_release_manifest.json'
DEPLOY = S / 'gen9_ready_for_gen10_runtime_deployment.json'
TEST = S / 'verification/gen9_selected_height_actual_integrated/result.json'


def sha(data):
    return hashlib.sha256(data).hexdigest()


def atomic(path, data):
    temp = path.with_name(path.name + '.documentation-tmp')
    temp.write_bytes(data)
    os.replace(temp, path)


assert Path('/media/zhuyupei/SanStand').is_mount()
release_bytes, deploy_bytes, test_bytes = (p.read_bytes() for p in (RELEASE, DEPLOY, TEST))
release, deploy, test = (json.loads(data) for data in (release_bytes, deploy_bytes, test_bytes))
assert sha(release_bytes) == 'c8676926c0de6816f4fb3b3ab867caf0ea6a2753d88042f1369c47ffd64a0548'
assert sha(deploy_bytes) == 'c1aaded78be35f6aeb1351f04aea15f1c342f3445808eb69a14b330da2c5ca35'
assert release['passed'] and test['passed'] and test['source_unchanged'] and test['production_worktree_tested']
assert release['test_summary'] == {'tests': 2030, 'failures': 0, 'errors': 0, 'skipped': 0}
assert release['source_sha256'] == deploy['source_sha256'] == test['source_sha256']
assert len(release['source_sha256']) == 873
assert deploy['whole_patch_deployed'] and deploy['compute_reloaded'] and deploy['deployment_passed']
assert deploy['preserved_before'] == deploy['preserved_after'] and len(deploy['preserved_after']) == 8
assert deploy['source_manifest_sha256'] == sha(release_bytes)
auth_path = S / 'continuous_authorization_20261012/authorization.json'
control_path = S / 'continuous_authorization_20261012/control.json'
auth_bytes, control_bytes = auth_path.read_bytes(), control_path.read_bytes()
auth, control = json.loads(auth_bytes), json.loads(control_bytes)
assert control['state'] == 'active' and control['authorization_id'] == auth['authorization_id']
idle_path = S / 'gen9_fix_post_reload_idle_verified/result.json'
idle_bytes = idle_path.read_bytes()
idle = json.loads(idle_bytes)
assert idle['passed'] and idle['source_count'] == 873 and idle['deployment_processes_unchanged']

current_path = S / 'current_progress.json'
latest_path = W / '.runtime/latest_full_ros_start.json'
legacy_paths = [W / '.runtime/current_full_latest_ros_20261008.json', W / '.runtime/codex_reload_handoff_20261009.json']
mirrors = [E / '文档' / DOC.name, S / DOC.name]
targets = [DOC, current_path, latest_path] + legacy_paths + mirrors
for path in targets:
    try:
        relative = path.relative_to(W)
    except ValueError:
        continue
    assert str(relative) not in release['source_sha256']
before = {path: path.read_bytes() for path in targets}
current = json.loads(before[current_path])
assert current['current_generation'] == 9 and not current['automatic_grasp_running']
assert current['last_result'] == 'failure'
now = time.time_ns()
draft = (W / '.runtime/gen9_contact_selected_height_20261012/OPERATIONS_APPEND_DRAFT.md').read_text()
section = re.sub(r'<!--.*?-->', '', draft, flags=re.S).strip()
assert '### 第9轮后：保留已选高度几何' not in before[DOC].decode()
deployed = '''

实际工作树已安装这5份交付文件并保存原文件备份，74个文件的实际组合 **2030项通过、0失败、0错误、0跳过**，873份来源前后SHA256一致。`verification/gen9_selected_height_actual_integrated/result.json` 的SHA256为 `8c5304e374d215bedfdaa3c2a30670b6f5383b415c2edf6c6a7b57b8ec108035`；统一发布 `verification/gen9_ready_for_gen10_release_manifest.json` 的SHA256为 `c8676926c0de6816f4fb3b3ab867caf0ea6a2753d88042f1369c47ffd64a0548`。历史1710通过／1失败、GEN9实际失败及本次所有隔离失败证据均保留，不改写为成功。

仅感知、remote、任务三个计算节点已重载，运行单元 `alicia-observation-measurement-1791810339050654828`；八个硬件相关进程身份保持。重载回执 `gen9_fix_compute_deployment/result.json`，统一部署 `gen9_ready_for_gen10_runtime_deployment.json`（SHA256 `c1aaded78be35f6aeb1351f04aea15f1c342f3445808eb69a14b330da2c5ca35`）记录 `whole_patch_deployed=true`、`compute_reloaded=true`。重载后的空闲及来源核验为 `gen9_fix_post_reload_idle_verified/result.json`。修复已部署，但尚未有本修复后的实物抓取验收。

准备第10轮时，重载前的只读反馈与GEN9结束姿态对比不一致，故未沿用旧末端对准记录；新现场图桌边可见手指。原失败对比、新反馈空闲核验与场景图分别保存在 `gen9_fix_before_reload_idle/`、`gen9_fix_before_reload_current_idle/`、`gen9_fix_changed_pose_scene/`。主代理已请求现场清空确认，当前 `awaiting_scene_clear_confirmation=true`，第10轮尚未启动；该确认核实现场清空，既有持续授权保持active。当前没有发起新的自动抓取，也没有实物夹持成功断言。
'''
doc_bytes = before[DOC] + ('\n\n' + section + deployed + '\n').encode()
patch = {
    'updated_ns': now, 'active_work': 'gen10_deployment_ready_awaiting_scene_clear_confirmation',
    'current_deployment': str(DEPLOY), 'latest_deployment': str(DEPLOY),
    'current_deployment_sha256': sha(deploy_bytes),
    'current_compute_deployment': str(S / 'gen9_fix_compute_deployment/result.json'),
    'current_operational_release': str(RELEASE), 'validation_manifest': str(RELEASE),
    'validation_manifest_sha256': sha(release_bytes),
    'source_sha256': release['source_sha256'], 'operational_source_sha256': release['source_sha256'],
    'current_source_count': 873, 'source_count': 873,
    'current_test_result': str(TEST), 'current_test_result_sha256': sha(test_bytes),
    'regression_pass_count': 2030, 'regression_fail_count': 0,
    'regression_error_count': 0, 'regression_skip_count': 0, 'tests_passed': 2030,
    'whole_patch_deployed': True, 'compute_reloaded': True,
    'deployed_compute_processes': deploy['after'], 'preserved_hardware_processes': deploy['preserved_after'],
    'deployment_idle_after': str(idle_path),
    'continuing_user_authorization': str(auth_path), 'continuing_user_authorization_sha256': sha(auth_bytes),
    'continuing_user_authorization_state': 'active', 'continuous_until_physical_grasp_success_requested': True,
    'awaiting_scene_clear_confirmation': True, 'awaiting_user_authorization': False,
    'new_user_confirmation_claimed': False,
    'current_alignment_pose_is_current': False,
    'scene_clear_confirmation_reason': 'Fresh feedback differs from historical GEN9 endpoint and a finger is visible near the table edge in the new scene image.',
    'scene_clear_confirmation_scope': 'Current scene clear confirmation; existing continuous grant remains active.',
    'new_scene_evidence': str(S / 'gen9_fix_changed_pose_scene'),
    'next_generation_expected': 10, 'next_trial_started': False,
    'automatic_grasp_running': False, 'running': False, 'recording_started': False, 'active_record': None,
    'physical_grasp_success': False, 'physical_grasp_verified': False,
    'next_action': 'Wait for the pending current-scene clear confirmation, then perform fresh scene/feedback/readiness checks before GEN10 under the existing continuous authorization and unchanged gates.',
    'resume_note': 'Selected-height geometry recipe repair deployed after actual 2030 passing tests/873 sources; only three compute nodes reloaded and eight hardware process identities preserved. GEN9 remains failure; GEN10 has not started and current-scene clearance confirmation is pending.',
    'documentation_update_receipt': str(OUT / 'receipt.json'),
}
updates = {DOC: doc_bytes}
for path in [current_path, latest_path]:
    value = json.loads(before[path])
    value.update(patch)
    updates[path] = (json.dumps(value, ensure_ascii=False, indent=2) + '\n').encode()
for path in legacy_paths:
    value = json.loads(before[path])
    value.update(snapshot_is_historical=True, authoritative_current_progress=str(current_path),
                 latest_release_pointer=str(RELEASE), latest_deployment_pointer=str(DEPLOY), pointer_updated_ns=now)
    updates[path] = (json.dumps(value, ensure_ascii=False, indent=2) + '\n').encode()
for path in mirrors:
    updates[path] = doc_bytes
for path, data in before.items():
    assert path.read_bytes() == data, 'Concurrent status/document change: ' + str(path)
assert control_path.read_bytes() == control_bytes
OUT.mkdir(exist_ok=False)
(OUT / 'before').mkdir()
backups = []
for index, (path, data) in enumerate(before.items()):
    backup = OUT / 'before' / ('%02d_' % index + path.name)
    backup.write_bytes(data)
    backups.append(dict(path=str(path), backup=str(backup), sha256=sha(data)))
for path, data in updates.items():
    atomic(path, data)
    assert path.read_bytes() == data
for path, expected in [(RELEASE, release_bytes), (DEPLOY, deploy_bytes), (TEST, test_bytes),
                       (auth_path, auth_bytes), (control_path, control_bytes), (idle_path, idle_bytes)]:
    assert path.read_bytes() == expected
receipt = dict(passed=True, recorded_ns=now, scope='documentation_and_progress_only',
    document_sha256=sha(doc_bytes), document_paths=[str(DOC)] + [str(p) for p in mirrors],
    updated_sha256={str(p): sha(data) for p, data in updates.items()}, before=backups,
    release_sha256=sha(release_bytes), deployment_sha256=sha(deploy_bytes),
    actual_test_result_sha256=sha(test_bytes), source_count=873, test_summary=release['test_summary'],
    release_source_files_written=[], historical_failures_preserved=True,
    awaiting_scene_clear_confirmation=True, continuous_authorization_state='active',
    gen10_started=False, physical_grasp_success_claimed=False, ros_commands_sent=False,
    temperature_queried=False)
(OUT / 'receipt.json').write_text(json.dumps(receipt, ensure_ascii=False, indent=2) + '\n')
print(json.dumps({'passed': True, 'document_sha256': sha(doc_bytes), 'receipt': str(OUT / 'receipt.json')}, ensure_ascii=False))
