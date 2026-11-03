from pathlib import Path
import hashlib
import json
import time

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
EXT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N = EXT / 'ros_log/full_latest_contact_admission_20261013T011103Z'
A = N / 'verification/gen14_reporting_verified_document_update'

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')

assert EXT.is_dir() and not A.exists()
test = N / 'verification/gen14_reporting_actual_integrated/result.json'
release = N / 'verification/gen14_timeout_reporting_release_manifest.json'
small = N / 'verification/gen14_small_support_features_r2/result.json'
t = json.loads(test.read_text())
r = json.loads(release.read_text())
assert t['passed'] and t['source_unchanged'] and t['production_worktree_tested']
assert t['summary'] == {'tests': 3653, 'failures': 0, 'errors': 0, 'skipped': 0}
assert len(t['source_sha256']) == 892 and t['source_sha256'] == r['source_sha256']
assert sha(release) == 'cb13b6cf22ca73d3f67d80aaabb092b0654d64e844583d67a27921f4ae1290f2'
assert all(sha(W / p) == h for p, h in t['source_sha256'].items())
doc = W / 'docs/operations/2026-10-11-frozen-grasp-acceptance.md'
mirrors = [EXT / '文档' / doc.name, N / doc.name]
before = doc.read_bytes()
assert all(p.read_bytes() == before for p in mirrors)
assert sha(doc) == 'f0d1b0298dbe16fa2cabd98b2bde51d5881edc2d1b131d24625aa476197f3ca0'
A.mkdir()
(A / 'document_before.md').write_bytes(before)
fragment = f'''

### 2026-10-12 GEN14 错误分类实际联合验证完成，仍未在线加载

上述139文件实际联合现已完成：**3653项通过、0失败、0错误、0跳过**，用时341.05秒，892份来源前后SHA256一致。结果 `verification/gen14_reporting_actual_integrated/result.json` SHA256为 `{sha(test)}`；统一发布 `verification/gen14_timeout_reporting_release_manifest.json` SHA256为 `cb13b6cf22ca73d3f67d80aaabb092b0654d64e844583d67a27921f4ae1290f2`。隔离81项已包含于该实际组合，不重复相加。此前“运行中”的记录按当时状态保留。

该完成状态只覆盖错误分类修复及原范围联合验证。**GEN14仍未加载，在线保持GEN13的890份运行来源；磁盘为验证后的892份来源，不能写成二者已经一致。** 完整图像的外观／几何提案原因仍在调查。另行的小支持区域ORB提案诊断 `verification/gen14_small_support_features_r2/result.json` 未产生可接受方案，不作为资格通过、算法修复完成或性能保证；它使用明确标注的宽裕诊断时钟，并非原source期限回放。

本次没有部署或新抓取，当前第2轮仍为封口失败，实物抓取仍未成功。更新前文档与进度元数据、892份来源再次只读核验及三文档一致性回执保存在 `verification/gen14_reporting_verified_document_update/`。后续实际算法候选须继续通过原身份、完整mask、独立稠密验证、当前3D、来源期限和三帧资格检查；温度与local_only约定保持。
'''
(A / 'document_fragment.md').write_text(fragment)
doc.write_bytes(before + fragment.encode())
for p in mirrors:
    p.write_bytes(doc.read_bytes())
paths = [N / 'current_progress.json', W / '.runtime/latest_full_ros_start.json']
now = time.time_ns()
for p in paths:
    old = p.read_bytes()
    (A / (p.stem + '_before.json')).write_bytes(old)
    d = json.loads(old)
    assert len(d['operational_source_sha256']) == 890
    assert d['runtime_matches_disk'] is False
    d.update({
        'updated_ns': now, 'recorded_ns': now,
        'active_work': 'gen14_arrival_identity_geometry_diagnosis_reporting_verified_not_loaded',
        'current_regression_state': 'passed',
        'current_test_result': str(test),
        'current_test_result_sha256': sha(test),
        'current_regression_summary': t['summary'],
        'current_regression_elapsed_sec': t['elapsed_sec'],
        'pending_combined_test_scope_files': 0,
        'verified_disk_release_manifest': str(release),
        'verified_disk_release_manifest_sha256': sha(release),
        'runtime_reload_pending': True,
        'whole_patch_deployed': False,
        'runtime_matches_disk': False,
        'automatic_grasp_running': False,
        'active_record': None,
        'root_cause_confirmed': False,
        'physical_grasp_verified': False,
        'physical_grasp_success': False,
        'algorithm_root_cause_status': 'Photometric and geometric proposal investigation pending; small-support ORB diagnostic produced no accepted proposal; GEN13 qualified template was not reached in attempt 2.',
        'proposal_diagnostic': str(small),
        'proposal_diagnostic_sha256': sha(small),
        'next_action': 'Continue original-frame identity and geometry investigation. Reporting-only actual regression passed; runtime reload remains pending and no new grasp is recorded.',
        'metadata_scope': 'GEN14 reporting disk release has 3653 passing actual tests but is not loaded. Verified runtime remains GEN13 with 890 sources; disk contains 892 sources. Attempt 2 remains sealed failure.',
        'metadata_update_audit': str(A / 'result.json'),
    })
    write_json(p, d)
assert all(sha(W / p) == h for p, h in t['source_sha256'].items())
assert all(p.read_bytes() == doc.read_bytes() for p in mirrors)
write_json(A / 'result.json', {
    'passed': True, 'recorded_ns': time.time_ns(),
    'source_unchanged': True, 'disk_source_count': 892, 'runtime_source_count': 890,
    'test_summary': t['summary'], 'runtime_loaded': False,
    'physical_grasp_success': False, 'algorithm_repair_pending': True,
    'evidence_sha256': {str(p): sha(p) for p in [test, release, small]},
    'written_sha256': {str(p): sha(p) for p in [doc, *mirrors, *paths]},
    'motion_commands_sent': False, 'temperature_queried': False,
    'robot_nodes_restarted': False, 'production_sources_modified': False,
})
print(json.dumps({'passed': True, 'audit': str(A), 'document_sha256': sha(doc)}, ensure_ascii=False))
