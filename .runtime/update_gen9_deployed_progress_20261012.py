#!/usr/bin/env python3
"""Update documentation and status only; never invoke ROS or write release sources."""
import hashlib
import json
import os
import time
from pathlib import Path

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
S = E / 'ros_log/full_latest_frozen_no_skin_20261012T021050Z'
R = E / 'grasp_records/20261012T122920.925646Z_9a08734d03f247ac895e5b762b04c615'
OUT = S / 'verification/gen9_deployed_documentation_update'
DOC = W / 'docs/operations/2026-10-11-frozen-grasp-acceptance.md'
RELEASE = S / 'verification/gen8_ready_for_gen9_release_manifest.json'
DEPLOY = S / 'gen8_ready_for_gen9_runtime_deployment.json'
TEST = S / 'verification/gen8_fk_actual_integrated/result.json'


def sha(data):
    return hashlib.sha256(data).hexdigest()


def write_atomic(path, data):
    temp = path.with_name(path.name + '.doc-update-tmp')
    temp.write_bytes(data)
    os.replace(temp, path)


release_bytes = RELEASE.read_bytes()
assert sha(release_bytes) == '6c8d924ebee71ea3b405b9be5f6d07ebbbffb9047fcb41921e51e017503bb15d'
release = json.loads(release_bytes)
deploy_bytes = DEPLOY.read_bytes()
deploy = json.loads(deploy_bytes)
test_bytes = TEST.read_bytes()
test = json.loads(test_bytes)
assert release['passed'] and release['reviewed'] and release['installed']
assert test['passed'] and test['source_unchanged'] and test['production_worktree_tested']
assert release['test_summary'] == {'tests': 1873, 'failures': 0, 'errors': 0, 'skipped': 0}
assert test['source_sha256'] == release['source_sha256'] == deploy['source_sha256']
assert len(release['source_sha256']) == 871
assert deploy['whole_patch_deployed'] and deploy['compute_reloaded'] and deploy['deployment_passed']
assert deploy['source_manifest_sha256'] == sha(release_bytes)
assert deploy['preserved_before'] == deploy['preserved_after']
assert len(deploy['preserved_after']) == 8 and len(deploy['after']) == 3
auth_path = S / 'continuous_authorization_20261012/authorization.json'
auth_bytes = auth_path.read_bytes()
auth = json.loads(auth_bytes)
control_path = S / 'continuous_authorization_20261012/control.json'
control_bytes = control_path.read_bytes()
control = json.loads(control_bytes)
assert control['state'] == 'active'
assert control['authorization_id'] == auth['authorization_id']
record_bytes = (R / 'manifest.json').read_bytes()
record = json.loads(record_bytes)
assert record['state'] == 'complete' and record['recorder_closed'] and record['result'] == 'failure'
manual_path = S / 'gen9_finished_manual/result.json'
idle_path = S / 'gen9_finished_idle_verified/result.json'
manual = json.loads(manual_path.read_bytes())
idle = json.loads(idle_path.read_bytes())
assert manual['manual_restored'] and manual['gui_own_publisher_acknowledged'] and manual['verified_inactive']
assert idle['passed'] and idle['manual_mode'] and not idle['task_active']
assert idle['source_count'] == 871 and idle['deployment_processes_unchanged'] and idle['source_matches_operational_release']

now = time.time_ns()
current_path = S / 'current_progress.json'
latest_path = W / '.runtime/latest_full_ros_start.json'
legacy_paths = [W / '.runtime/current_full_latest_ros_20261008.json', W / '.runtime/codex_reload_handoff_20261009.json']
mirrors = [E / '文档/2026-10-11-frozen-grasp-acceptance.md', S / DOC.name]
targets = [DOC, current_path, latest_path] + legacy_paths + mirrors
for path in targets:
    try:
        relative = path.relative_to(W)
    except ValueError:
        continue
    assert str(relative) not in release['source_sha256']
before = {path: path.read_bytes() for path in targets}
current = json.loads(before[current_path])
assert current.get('current_generation') in (8, 9)
if current.get('current_generation') == 9:
    assert current.get('current_record') == str(R)

section = '''

### 第9轮：补视执行链已通过，补视后接触阶段到期失败（2026-10-12）

上述1710通过／1失败是首次组合回归的历史结果，原XML及失败说明继续保留。测试隔离问题修正、补视中心一致性修复、请求内FK约束预计算及持续授权包装器现已合入实际工作树，最终实际组合为 **1873通过、0失败、0错误、0跳过**，871份来源前后SHA256一致。结果为 `verification/gen8_fk_actual_integrated/result.json`（SHA256 `39348066a5ef1b48d13c199138b675ff4f3f35f163e50209e551803498595e9a`），最终发布清单 `verification/gen8_ready_for_gen9_release_manifest.json` 的SHA256为 `6c8d924ebee71ea3b405b9be5f6d07ebbbffb9047fcb41921e51e017503bb15d`。各隔离／独立范围与本组合重叠，不累加测试数；旧GEN20测试的两项历史基线失败另行留证，没有写成本组合通过项。

FK优化只把同一请求内不变的CAD角点、归一化支撑法向及误差盒Taylor项预计算；逐关节状态的原式及最终原始端点门仍执行，预计算计入原2秒预算。独立数值等价、边界及请求隔离检查通过；离线实录3组对照在原预算中从0／0／1个候选变为1／1／1个候选，不保证所有线上负载下必有解，也不证明实物夹持。原期限、迭代上限、碰撞采样、最多两次观察位、双ACK、限速及不自动抬升规则保持。

三计算节点 `/unknown_tabletop_perception`、`/remote_grasp6d_node`、`/grasp_task_node` 已完成重载，统一部署 `gen8_ready_for_gen9_runtime_deployment.json`（SHA256 `094097e05a085bbdabadb1bcdf2bfd4222872250e89dc7acbff3e12e79edeb5d`）记录 `whole_patch_deployed=true`、`compute_reloaded=true`，运行单元 `alicia-observation-measurement-1791808026847264978`。驱动、硬件接口、MoveIt、运动网关、相机、夹爪、路由和GUI八个进程的PID／启动标识前后保持；重载本身未发送运动、失能或温度查询。此前“在线消费者尚未重载”仅为该阶段的历史状态。

第9轮按已有 `continuous_authorization_20261012/authorization.json` 的真实持续授权启动，未伪造新的用户手清确认。selection9/stamp1791808162970761537，记录 `20261012T122920.925646Z_9a08734d03f247ac895e5b762b04c615`；新场景核验、准备回执和实时观察器分别为 `gen9_continuation_scene/`、`gen9_continuation_readiness/`、`gen9_watch/`。本轮第二补视已通过严格准入、双ACK、运动及到位采样，已经越过第8轮补视路径未产生的阻断；这不等于整个抓取完成。

补视后接触阶段的phase_started为1791808264957453250，原90秒期限为1791808354957432985；到期后记录 `TARGET_LOST: station_compute_wait_expired_or_future`，未进入接触或闭爪。`command_result.json`、`result_summary.json`及manifest均记录failure，manifest为complete且recorder_closed=true；录制自然停止，553组关键帧。request40报告end_to_end约91.152秒，目前继续核查接触计算预算／重试及其与父阶段期限的关系，不把到期报错推断为现场目标实际消失，也不把补视通过写为实物夹持成功。

任务结束后GUI自身手动回传及空闲核验通过，见 `gen9_finished_manual/result.json`、`gen9_finished_idle_verified/result.json`：任务及两动作空闲，20条实际关节反馈稳定，871份来源／1873项组合回归及11个进程身份保持。两观察器按精确PID／启动标识停止，见 `gen9_watch/stop_receipt.json`。持续授权控制文件仍为active；本次失败没有暂停或撤回该授权。继续定位、验证后按原门禁推进，当前尚无实物抓取成功证据。
'''
assert '### 第9轮：补视执行链已通过' not in before[DOC].decode()
doc_bytes = before[DOC] + section.encode()
patch = {
    'updated_ns': now,
    'active_work': 'gen9_failed_after_completed_supplemental_view_contact_compute_budget_analysis',
    'current_deployment': str(DEPLOY), 'latest_deployment': str(DEPLOY),
    'current_deployment_sha256': sha(deploy_bytes),
    'current_compute_deployment': str(S / 'gen8_fix_compute_deployment/result.json'),
    'current_operational_release': str(RELEASE), 'validation_manifest': str(RELEASE),
    'validation_manifest_sha256': sha(release_bytes),
    'source_sha256': release['source_sha256'],
    'operational_source_sha256': release['source_sha256'],
    'current_source_count': 871, 'source_count': 871,
    'current_test_result': str(TEST), 'current_test_result_sha256': sha(test_bytes),
    'regression_pass_count': 1873, 'regression_fail_count': 0,
    'regression_error_count': 0, 'regression_skip_count': 0, 'tests_passed': 1873,
    'whole_patch_deployed': True, 'compute_reloaded': True,
    'deployed_compute_processes': deploy['after'],
    'preserved_hardware_processes': deploy['preserved_after'],
    'continuing_user_authorization': str(auth_path),
    'continuing_user_authorization_sha256': sha(auth_bytes),
    'continuing_user_authorization_state': control['state'],
    'continuous_until_physical_grasp_success_requested': True,
    'new_user_confirmation_claimed': False, 'awaiting_user_authorization': False,
    'automatic_grasp_started': True, 'automatic_grasp_running': False,
    'running': False, 'recording_started': False, 'next_trial_started': False,
    'active_record': None, 'current_record': str(R),
    'last_record': str(R), 'last_result': 'failure', 'last_failure_reason': record['failure_reason'],
    'current_generation': 9,
    'current_selection': {'mode': 'unknown', 'strategy': 'observe_return', 'generation': 9, 'stamp_ns': 1791808162970761537},
    'next_generation_expected': 10,
    'gen9_watch_directory': str(S / 'gen9_watch'),
    'gen9_resume_readiness': str(S / 'gen9_continuation_readiness'),
    'gui_manual': True, 'gui_manual_restored': True,
    'gen9_finished_manual': str(manual_path), 'gen9_finished_idle': str(idle_path),
    'gen9_observers_stop_receipt': str(S / 'gen9_watch/stop_receipt.json'),
    'physical_grasp_success': False, 'physical_grasp_verified': False,
    'next_action': 'Diagnose GEN9 contact compute budget and bounded retry after successful supplemental-view handoff, then validate fixes under the existing active continuing authorization and unchanged execution gates.',
    'resume_note': 'Final release deployed with 1873 passing tests and 871 bound sources. GEN9 failed at the original 90-second post-supplement phase deadline; recorder closed and GUI manual/idle verified. Continuing user authorization remains active.',
    'gen9_finished_snapshot': {
        'recorded_ns': now, 'result': 'failure', 'record_state': 'complete',
        'supplemental_view_2_motion_and_arrival_completed': True,
        'phase_started_ns': 1791808264957453250, 'phase_deadline_ns': 1791808354957432985,
        'physical_grasp_verified': False, 'recorder_closed': True,
    },
    'documentation_update_receipt': str(OUT / 'receipt.json'),
}
current.update(patch)
updates = {DOC: doc_bytes, current_path: (json.dumps(current, ensure_ascii=False, indent=2) + '\n').encode()}
latest = json.loads(before[latest_path])
latest.update(patch)
updates[latest_path] = (json.dumps(latest, ensure_ascii=False, indent=2) + '\n').encode()
for path in legacy_paths:
    legacy = json.loads(before[path])
    legacy.update({'snapshot_is_historical': True, 'authoritative_current_progress': str(current_path),
                   'latest_release_pointer': str(RELEASE), 'latest_deployment_pointer': str(DEPLOY),
                   'pointer_updated_ns': now})
    updates[path] = (json.dumps(legacy, ensure_ascii=False, indent=2) + '\n').encode()
for path in mirrors:
    updates[path] = doc_bytes

# Reject concurrent status updates instead of overwriting a later attempt state.
for path, data in before.items():
    assert path.read_bytes() == data, 'Concurrent update: ' + str(path)
latest_record = json.loads((R / 'manifest.json').read_bytes())
assert latest_record['state'] == 'complete' and latest_record['recorder_closed'] and latest_record['result'] == 'failure'
OUT.mkdir(parents=True, exist_ok=False)
(OUT / 'before').mkdir()
backup_receipts = []
for i, (path, data) in enumerate(before.items()):
    backup = OUT / 'before' / ('%02d_' % i + path.name)
    backup.write_bytes(data)
    backup_receipts.append({'path': str(path), 'backup': str(backup), 'sha256': sha(data)})
(OUT / 'gen9_record_manifest_finished_snapshot.json').write_bytes(record_bytes)
for path, data in updates.items():
    write_atomic(path, data)
for path, data in updates.items():
    assert path.read_bytes() == data
assert RELEASE.read_bytes() == release_bytes and DEPLOY.read_bytes() == deploy_bytes and TEST.read_bytes() == test_bytes
assert auth_path.read_bytes() == auth_bytes and control_path.read_bytes() == control_bytes
receipt = {
    'passed': True, 'recorded_ns': now, 'scope': 'documentation_and_status_only',
    'document_sha256': sha(doc_bytes), 'document_paths': [str(DOC)] + [str(p) for p in mirrors],
    'updated_sha256': {str(p): sha(data) for p, data in updates.items()},
    'before': backup_receipts,
    'release_path': str(RELEASE), 'release_sha256': sha(release_bytes),
    'deployment_path': str(DEPLOY), 'deployment_sha256': sha(deploy_bytes),
    'actual_test_result_path': str(TEST), 'actual_test_result_sha256': sha(test_bytes),
    'release_source_count': len(release['source_sha256']),
    'source_files_written': [], 'source_rehash_performed_by_this_document_update': False,
    'test_summary': release['test_summary'], 'historical_1710_pass_1_fail_preserved': True,
    'gen9_finished_record': str(R), 'gen9_terminal_result': 'failure',
    'physical_grasp_success_claimed': False, 'ros_commands_sent': False, 'temperature_queried': False,
}
(OUT / 'receipt.json').write_text(json.dumps(receipt, ensure_ascii=False, indent=2) + '\n')
print(json.dumps({'passed': True, 'document_sha256': sha(doc_bytes), 'receipt': str(OUT / 'receipt.json'), 'updated_files': len(updates)}, ensure_ascii=False))
