from pathlib import Path
import hashlib
import json
import time

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
EXT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N = EXT / 'ros_log/full_latest_contact_admission_20261013T011103Z'
A = N / 'verification/gen14_reporting_installed_document_update'

def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def write_json(p, value):
    p.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')

assert EXT.is_dir()
assert not A.exists(), 'Do not overwrite the previous audit'
install_path = N / 'verification/gen14_reporting_integrated_install/result.json'
install = json.loads(install_path.read_text())
assert install['passed'] and len(install['source_sha256']) == 892
assert len(install['test_scope']) == 139
assert all(sha(W / p) == h for p, h in install['source_sha256'].items())
actual = N / 'verification/gen14_reporting_actual_integrated'
assert not (actual / 'result.json').exists(), 'Final regression arrived; refresh the requested status first'
diagnosis_path = N / 'verification/gen14_arrival_independent/result.json'
diagnosis = json.loads(diagnosis_path.read_text())
assert diagnosis['passed'] and len(diagnosis['rows']) == 10
wire_path = N / 'verification/gen14_arrival_diagnosis/wire_diagnosis.json'
wire = json.loads(wire_path.read_text())
assert wire['completed_arrival_frames'] == 9 and wire['qualified_sources'] == []
assert wire['ack_delay_from_request_ns'] == {'remote': 3030777, 'unknown': 4281997}
candidate_path = N / 'verification/gen14_reporting_independent_candidate_r1/result.json'
candidate = json.loads(candidate_path.read_text())
assert candidate['passed'] and candidate['counts'] == {'tests': 81, 'failures': 0, 'errors': 0, 'skipped': 0}
delivery = W / '.runtime/gen14_measurement_timeout_reason_20261013/delivery_manifest.json'
review = N / 'verification/gen14_timeout_reporting_delivery_seal/independent_review.json'
assert sha(delivery) == '617785775106e726153a8fb674341ee5ad3749e929cf5bc1078361db65eab627'
assert sha(review) == 'b2db356bc26c2ca78b9d5ecfd2e5447d5b61ce836dd89877f502ca9defcc5ca1'
doc = W / 'docs/operations/2026-10-11-frozen-grasp-acceptance.md'
mirrors = [EXT / '文档' / doc.name, N / doc.name]
before_doc = doc.read_bytes()
assert all(p.read_bytes() == before_doc for p in mirrors)
assert sha(doc) == '02c10d262ba564cda788e457273f610e88bf858f824352224e4d1d8cb5822517'
A.mkdir()
(A / 'document_before.md').write_bytes(before_doc)
fragment = '''

### 2026-10-12 第2轮确认失败阶段，GEN14 错误分类修复已安装但尚未加载

继续复核第2轮原始请求、ACK及完整图像后，已能区分传输确认和测量资格：requalifying请求的远端ACK在3,030,777纳秒、感知ACK在4,281,997纳秒内到达，均accepted=true；随后九张完成计算的到位帧均被 `arrival_fragment_group_original_rgb_identity_required` 拒绝，qualified来源列表一直为空，第十张到达原3秒capture deadline。独立结果为 `verification/gen14_arrival_independent/result.json`，SHA256 `eb8f0aa0e75a5fb348f2e6963104325b5034fc387caefc42ff589e956db76e92`；原wire依据为 `verification/gen14_arrival_diagnosis/wire_diagnosis.json`。因此原 `OBSERVATION_MEASUREMENT_ACK_TIMEOUT` 文案混淆了“缺少传输确认”和“已确认请求但尚未取得合格测量”两种状态。

九帧对运动前外观的颜色距离约0.402–0.430，超过原0.4门；原ORB单应提案的完整轮廓双向IoU约0.56–0.73，未通过原完整mask门限。这些证据确认失败阶段及具体拒绝门，不足以单独认定光照、分割或几何提案的最终算法原因。原像素与匹配几何仍在独立调查，原颜色、完整mask、稠密独立验证、当前3D和三帧资格要求均未放宽。本轮未到qualified callback，GEN13固定第三合格帧模板没有建立或参与该失败，不能据此写成该模板修复回退。

原动作证据仅有一次已成功结束的初始机械臂轨迹；没有夹爪动作goal、没有接触phase，未执行接近或闭爪。第2轮仍保持原sealed failure和原错误全文，不以新分类回写历史实验。收尾手动及空闲核验、两份已关闭bag的精确SHA和107条保护清单仍见 `attempt_2_seal/`；没有新实物成功证据。

独立交付的GEN14 reporting-only修复只修改等待循环的终止错误：在同一请求的双方原ACK校验均已通过、但qualified条件仍未满足时，到原截止报告 `OBSERVATION_MEASUREMENT_QUALIFICATION_TIMEOUT` 并列出仍待qualified的一端或双方；没有完整合法双ACK时继续原 `OBSERVATION_MEASUREMENT_ACK_TIMEOUT`。它记录最后一对已完整校验的ACK，不重新授予权限。所有原capture／lease／monotonic预算、校验次序、等待间隔、返回条件及真实过期／拒绝异常保持；普通executing路径保持原行为，不续期或重试动作。

隔离验证包含作者13项、独立19项及原49项，共81项通过、0失败／错误／跳过，见 `verification/gen14_reporting_independent_candidate_r1/result.json`。同组在实际GEN13基线为77通过、4个预期的新分类断言失败，见 `gen14_reporting_independent_baseline_r1/`；原拒绝、manual／stop、owner变化、错请求、错三帧和真实ROS时钟越过capture／lease截止仍按原规则拒绝。初次作者11通过／2失败报告保留：浮点测试时钟与ROS纳秒舍入不一致碰到了原截止拒绝，后只将夹具步长按9位小数对齐，生产逻辑未因此改变。

交付共3文件（一份生产源码、两份新增测试），delivery SHA256为 `617785775106e726153a8fb674341ee5ad3749e929cf5bc1078361db65eab627`；独立评审 `verification/gen14_timeout_reporting_delivery_seal/independent_review.json` SHA256为 `b2db356bc26c2ca78b9d5ecfd2e5447d5b61ce836dd89877f502ca9defcc5ca1`。主代理已安装到实际工作树，原备份与回执为 `verification/gen14_reporting_integrated_install/result.json`。磁盘来源增至892份，139个测试文件的最终实际联合在 `verification/gen14_reporting_actual_integrated/` 运行，本段没有提前声明其通过。

**GEN14错误分类修复仅已安装到磁盘，尚未加载；在线仍是已部署的GEN13、890份运行来源。** `gen13_deployment.json` 与其source manifest继续指向那次已验证运行版本，不被磁盘安装清单替换。本次未部署、未发起新抓取；算法修复仍待调查及验证，实物抓取仍未成功。本文档及两处SanStand镜像、进度元数据的更新前副本和核验保存在 `verification/gen14_reporting_installed_document_update/`，local_only、失败录制保护及温度约定保持。
'''
(A / 'document_fragment.md').write_text(fragment)
doc.write_bytes(before_doc + fragment.encode())
for p in mirrors:
    p.write_bytes(doc.read_bytes())
metadata_paths = [N / 'current_progress.json', W / '.runtime/latest_full_ros_start.json']
updated_ns = time.time_ns()
for p in metadata_paths:
    before = p.read_bytes()
    (A / (p.stem + '_before.json')).write_bytes(before)
    d = json.loads(before)
    runtime_map = dict(d['operational_source_sha256'])
    assert len(runtime_map) == 890
    d['last_completed_regression'] = {k: d.get(k) for k in ['current_regression', 'current_test_result', 'current_test_result_sha256', 'current_regression_summary', 'current_test_scope_files']}
    d.update({
        'updated_ns': updated_ns,
        'recorded_ns': updated_ns,
        'active_work': 'gen14_reporting_actual_regression_and_arrival_identity_geometry_diagnosis',
        'installed_repair': str(install_path),
        'installed_repair_sha256': sha(install_path),
        'installed_but_not_loaded_patch': str(delivery),
        'installed_but_not_loaded_patch_sha256': sha(delivery),
        'disk_source_count': 892,
        'disk_source_sha256': install['source_sha256'],
        'disk_source_evidence': str(install_path),
        'runtime_source_count': 890,
        'runtime_source_manifest': d['source_manifest'],
        'runtime_source_manifest_sha256': d['source_manifest_sha256'],
        'source_manifest_scope': 'Verified GEN13 runtime only; installed GEN14 disk source differs.',
        'source_count_scope': 'current_source_count and source_count describe installed disk; operational_source_sha256 describes verified GEN13 deployment.',
        'current_source_count': 892,
        'source_count': 892,
        'runtime_matches_disk': False,
        'runtime_reload_pending': True,
        'whole_patch_deployed': False,
        'deployment_scope': 'deployment_passed and compute_reloaded refer to prior GEN13 deployment only; GEN14 is installed but not loaded.',
        'current_regression': str(actual),
        'current_regression_state': 'running',
        'current_test_result': None,
        'current_test_result_sha256': None,
        'current_regression_summary': None,
        'current_test_scope_files': 139,
        'pending_combined_test_scope_files': 139,
        'automatic_grasp_running': False,
        'active_record': None,
        'physical_grasp_verified': False,
        'physical_grasp_success': False,
        'root_cause_confirmed': False,
        'failure_stage_confirmed': 'Both accepted ACKs timely; nine original premotion appearance rejections; zero qualified frames; tenth capture-deadline expiry.',
        'failure_stage_evidence': str(diagnosis_path),
        'failure_stage_evidence_sha256': sha(diagnosis_path),
        'wire_diagnosis_evidence': str(wire_path),
        'wire_diagnosis_evidence_sha256': sha(wire_path),
        'algorithm_root_cause_status': 'Photometric and geometric proposal investigation pending; GEN13 qualified template was not reached.',
        'next_action': 'Finish actual reporting-only regression; continue original-frame identity and geometry investigation. No new deployment or grasp recorded by this update.',
        'metadata_scope': 'Installed disk GEN14 reporting-only patch differs from verified GEN13 runtime. Attempt 2 remains sealed failure; manual evidence is historical, not a live-state sample.',
        'metadata_update_audit': str(A / 'result.json'),
    })
    assert d['operational_source_sha256'] == runtime_map
    write_json(p, d)
assert all(sha(W / p) == h for p, h in install['source_sha256'].items())
assert all(p.read_bytes() == doc.read_bytes() for p in mirrors)
result = {
    'passed': True, 'recorded_ns': time.time_ns(),
    'status': 'GEN14 reporting installed / actual regression running / GEN13 runtime retained',
    'source_unchanged': True, 'disk_source_count': 892, 'runtime_source_count': 890,
    'test_scope_files': 139, 'physical_grasp_success': False,
    'evidence_sha256': {str(p): sha(p) for p in [install_path, diagnosis_path, wire_path, candidate_path, delivery, review]},
    'written_sha256': {str(p): sha(p) for p in [doc, *mirrors, *metadata_paths]},
    'motion_commands_sent': False, 'temperature_queried': False,
    'robot_nodes_restarted': False, 'production_sources_modified': False,
}
write_json(A / 'result.json', result)
print(json.dumps({'passed': True, 'audit': str(A), 'document_sha256': sha(doc)}, ensure_ascii=False))
