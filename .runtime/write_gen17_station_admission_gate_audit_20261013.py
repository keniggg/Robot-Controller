#!/usr/bin/env python3
"""Read-only source/wire audit; writes only the new SanStand evidence directory."""
import hashlib
import json
from pathlib import Path

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
EXT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N = EXT/'ros_log/full_latest_contact_admission_20261013T011103Z'
R = EXT/'grasp_records/20261013T112006.120233Z_d2b04edd648643b4b57d8e9febc4d88c'
O = N/'verification/gen17_station_admission_gate_audit'

def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def main():
    original = N/'verification/attempt5_station_timeline_r1/result.json'
    assert sha(original) == 'd130fcdb4cd5f5f8a0a291111354f6a19c322d9f8014d82d8dba82d6e42be6e5'
    extracted = json.loads(original.read_text())
    wire_path = Path(extracted['wire'])
    assert sha(wire_path) == extracted['wire_sha256']
    rows = json.loads(wire_path.read_text())
    ref = rows[1412]['message']
    report_path = R/'keyframes_audits'/(ref['report_sha256']+'.json')
    assert sha(report_path) == ref['report_sha256']
    report = json.loads(report_path.read_text())
    assert report['request_id'] == 27
    selected = report['selected']
    receipt = report['station_contact_computation']
    failure = json.loads(rows[1417]['message']['msg'].split(' ', 1)[1])
    current_ns = failure['current_source_ns']
    ready = [r for r in rows if r['topic']=='/perception/unknown/detector_status'
             and r['message'].get('state')=='ready'
             and 1791890535_000000000 < r['received_ns'] < rows[1418]['received_ns']]
    local_contract = next(r['message']['station_compute_wait'] for r in ready
                          if r['message']['source_stamp_ns']==current_ns)
    failure_header = rows[1417]['message']['header']['stamp']
    source_files = [
        'src/alicia_grasp_modes/src/alicia_grasp_modes/'+name+'.py' for name in (
            'station_compute_reacquisition', 'station_contact_admission',
            'station_compute_wait_task', 'station_compute_wait_remote',
            'station_compute_wait_producer')]
    source_files += ['src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py',
        'src/alicia_grasp_modes/tests/test_station_contact_admission.py',
        'src/alicia_grasp_modes/tests/test_gen11_contact_remote_receipt.py']
    locations = {
        'validator': 'station_compute_reacquisition.py:90-124',
        'receipt_history': 'station_contact_admission.py:36-131',
        'task_current': 'station_compute_wait_task.py:153-208',
        'task_context': 'station_compute_wait_task.py:544-610',
        'task_join': 'station_compute_wait_task.py:969-1087',
        'task_soft_wait': 'station_compute_wait_task.py:854-869,933-961',
        'task_retained_candidate': 'station_compute_wait_task.py:1320-1371',
        'task_freeze': 'station_compute_wait_task.py:1405-1453',
        'task_commit': 'station_compute_wait_task.py:1504-1587',
        'base_commit_consumer': 'grasp_task_node.py:4198-4239,7204-7216',
        'remote_join': 'station_compute_wait_remote.py:845-940',
        'remote_receipt': 'station_compute_wait_remote.py:1253-1283',
        'producer_registration': 'station_compute_wait_producer.py:223-245,615-699',
    }
    findings = [
        dict(id='F1', status='R5 supported; precise Python call timing inferred',
             location=locations['task_current'],
             problem='Already registered current geometry can expire between _station_task_check and the second context/registration validation. ROS expiry becomes terminal provenance invalid, while the equivalent monotonic expiry is PENDING.',
             fix='Keep immutable receipt validation strict; after live owner/SDK/identity validation, classify current geometry expiry as typed pending, quarantine only geometry and retain the exact private candidate. Use successor independent READY and rerun original admission gates.'),
        dict(id='F2', status='static reachable follow-on; not observed as R5 terminal site',
             location=locations['task_freeze']+'; '+locations['task_commit']+'; '+locations['base_commit_consumer'],
             problem='Freeze checks provenance again and converts even PENDING to ValueError. Returning PENDING to the base near-field rebind also sets FAILED. First-site correction alone leaves a second terminal boundary.',
             fix='Represent pre-freeze metadata-only pending distinctly and catch it inside the station rebind loop. Release plan lock before waiting and preserve candidate/digest. Retry only when base freeze/action has not run; any post-commit exception must not retry a physical action.'),
        dict(id='F3', status='static analogous race; R5 remote issuance succeeded',
             location=locations['remote_receipt'],
             problem='Remote wait/tick/context can pass, then current source crosses its geometry deadline before validate_registration. The existing receipt loop has no typed retry for this crossing.',
             fix='For unchanged live owner and private work proof, release stream/geometry locks and wait for a genuinely newer joined source. Preserve original three input stamps, plan digest, ticket and absolute phase/session/identity limits; do not change a historically invalid issued receipt to pending.'),
        dict(id='F4', status='static clock-pairing defect risk; not established R5 cause',
             location=locations['task_join']+'; '+locations['remote_join']+'; '+locations['producer_registration'],
             problem='New registration uses an earlier ROS now with a later monotonic read after intervening CPU/locks. Task also begins the multi-source loop with one ROS now. This can give monotonic deadlines extra elapsed CPU time and briefly commit a source that aged during validation. Producer final marker check checks ownership, not source freshness.',
             fix='Capture a paired monotonic-before/ROS commit observation after expensive validation, revalidate new source at that commit, and derive monotonic deadlines from this pair. Same-source receipts must retain old deadlines. Recheck existing identity independently; discard a late offered source without retiring a still-valid older owner.'),
        dict(id='F5', status='static progress/lock concern, recorder evidence consistent but insufficient to prove lock contention',
             location=locations['task_context']+'; '+locations['task_retained_candidate']+'; '+locations['task_commit'],
             problem='Repeated context and immutable receipt/plan checks run while task plan lock excludes READY joins. R5 recorder had already received a newer READY before task failed, but task memory at that point is unavailable. Merely returning pending must not create a permanent lock-holding retry loop or repeatedly rehash the same proof until every frame expires.',
             fix='Verify concurrent ingress can acquire its original locks during metadata waits. If measured check cost still consumes each window, prepare immutable receipt/digest evidence outside the short commit section and recheck exact version/owner/current source/SDK at commit; invalidate preparation on every relevant change. Do not cache a successful live gate indefinitely.'),
    ]
    boundaries = [
        dict(boundary='new registration', rule='Unregistered source already at/after geometry deadline', outcome='reject this source; cannot manufacture registration or refresh source timestamp', hard_motion_block=True),
        dict(boundary='historical receipt issuance', rule='Original source witness missing/changed, issued_ns outside its own geometry window, wrong plan/ticket/digest/target', outcome='terminal provenance invalid; never downgrade by matching an error string', hard_motion_block=True),
        dict(boundary='current registered station', rule='Only current geometry crossed .8s, immutable witness valid and same owner/identity still live', outcome='PENDING; preserve private computation, wait without motion for independently registered successor', hard_motion_block=True),
        dict(boundary='identity/session/phase', rule='Absolute identity/session/phase deadline expired, clock rewind, retired owner or unknown ancestry', outcome='reject/retire; repeat receipt does not renew deadline', hard_motion_block=True),
        dict(boundary='physical validity', rule='Owner/control epoch/manual change, missing/stale SDK, measured movement, target or camera configuration changed, geometry/collision evidence failure', outcome='retain existing reject or separately evidenced recovery path; not a warning permitting motion', hard_motion_block=True),
        dict(boundary='metadata commit', rule='Current source expires before any freeze side effect', outcome='typed bounded retry inside station mixin, release lock, exact candidate retained', hard_motion_block=True),
        dict(boundary='action commit', rule='Freeze already completed or any physical action submitted', outcome='never retry the callback on a metadata pending result; preserve original physical action and result rules', hard_motion_block=True),
    ]
    counterexamples = [
        'Task: enter with source age 799ms, advance ROS and monotonic during second real context call to 801ms, expect PENDING/non-terminal/no action; deliver actual next object+mask+READY, same candidate later binds exactly once.',
        'Task freeze: first provenance succeeds, advance during original atomic bound-plan gate, second provenance pending; wrapper releases plan lock, ingress completes and same digest commits once. A post-super-freeze exception must not retry.',
        'Remote: enter wait/tick with fresh reference, advance during context, next joined source recovers receipt; original three stamps and receipt plan digest remain unchanged; no ticket/session renewal.',
        'Historical invalid receipt: issued_ns == its current geometry deadline remains terminal even if task has a newer healthy source. Missing/changed historical sources and wrong ticket/plan remain terminal.',
        'Current expiry exactly at deadline and ROS-only/monotonic-only expiry agree on pending; identity expiry, clock rewind, manual/control epoch/camera/target/SDK faults do not become pending-success.',
        'Registration: add deterministic CPU time between initial check and commit; source aging before commit never enrolls. ROS/monotonic deadline pair must not gain the injected time. Repeat same-source receipt never renews.',
        'No new READY: bounded pending ends at original identity/phase limit, candidate is not executed, no infinite waiting or lock starvation.',
        'Concurrent successor: READY delivered while metadata check holds lock is accepted after release; no lucky phase/source selection and all original receipts retained.',
    ]
    result = dict(schema=1, scope='offline R5 wire and source gate audit; no candidate implementation or live verification',
        wire_evidence=dict(result=str(original), result_sha256=sha(original), wire=str(wire_path), wire_sha256=sha(wire_path)),
        source_sha256={str(W/p):sha(W/p) for p in source_files}, locations=locations,
        actual_r5=dict(plan_id=receipt['plan_id'], plan_source_ns=receipt['plan_source_ns'],
            receipt_current_ns=receipt['current']['contract']['source_ns'], issued_ns=receipt['issued_ns'],
            receipt_current_geometry_deadline_ns=receipt['current']['contract']['geometry_deadline_ns'],
            receipt_current_remaining_at_issue_ns=receipt['current']['contract']['geometry_deadline_ns']-receipt['issued_ns'],
            task_failure=failure, task_failure_log_header_ns=failure_header,
            task_current_geometry_deadline_ns=local_contract['geometry_deadline_ns'],
            log_header_current_age_ns=failure_header-current_ns,
            log_header_past_geometry_deadline_ns=failure_header-local_contract['geometry_deadline_ns'],
            phase_remaining_at_failure_log_ns=local_contract['phase_deadline_ns']-failure_header,
            current_record_exact_source_proven_by_admission_diagnostic=True,
            ready_before_failure=[dict(index=r['index'], received_ns=r['received_ns'],
                source_ns=r['message']['source_stamp_ns'], geometry_deadline_ns=r['message']['station_compute_wait']['geometry_deadline_ns']) for r in ready],
            terminal_state=rows[1418], subsequent_first_loss_log=rows[1422],
            internal_callback_entry_and_lock_acquisition_unobserved=True),
        request27=dict(report=str(report_path), sha256=sha(report_path), raw_gate_audit_row=1412,
            outcome=report['outcome'], promotion=report['promotion'],
            moveit_selection=report['moveit_selection'],
            selected={k:selected[k] for k in ('moveit','final_geometry_gate','final_strict_moveit_metrics',
                'final_strict_plan_id','moveit_execution_stage_policy','final_execution_sequence_checked',
                'final_contact_rejection','final_contact_height_margin')},
            real_checked_contact_preview=True, task_admitted=False, physical_grasp_success=False,
            limits=['Strict sequence covers pregrasp/approach/grasp, close-hold policy; lift path not verified.',
                    'Stationary SDK following evidence and predicted future path do not prove future following or real grip.',
                    'Report outcome.valid_plan=false and promotion.promote=false; rich preview.valid=true is proposal validity, not action authority.',
                    'Pipeline metrics cite a different later audit digest; this audit binds the actual raw gate_audit row and its SHA-exact archived report, not the unarchived metrics digest.']),
        findings=findings, boundary_table=boundaries, proposed_counterexamples=counterexamples,
        tests_run=False, source_modified=False, ros_master_accessed=False,
        fullframes_accessed=False, large_bag_hashed=False, temperature_queried=False,
        limitations=['Static reachable races are not asserted to have all occurred in R5.',
                    'Recorder receipt order does not establish subscription callback entry, internal lock acquisition or action line execution.',
                    'This audit is an implementation/test design, not evidence of a repaired or successful grasp.'])
    lines = ['# R5 注册与接触提交边界独立审计', '',
        'R5 已得到真实严格检查通过的接触预览；任务尚未接纳，未发送夹手动作，未证明抓取成功。', '',
        '当前失败不是“计算旧源必须重新变新”。三帧原始计算来源应保持不变；远端在出具凭据时另绑当时鲜活源，任务又已滚动到更新的独立注册源。错误是当前源在重复检查中刚跨过几何有效期后，被通用异常分支转成了永久凭据错误。', '',
        '原始消息与接收时刻：', '',
        '- 凭据 current 源 %d；issued %d；出具时剩 %.3f ms。' % (receipt['current']['contract']['source_ns'], receipt['issued_ns'], result['actual_r5']['receipt_current_remaining_at_issue_ns']/1e6),
        '- 任务失败诊断 current 源 %d，几何截止 %d；诊断消息 header 在截止后 %.3f ms。该 header 是记录时点，不是 Python 校验语句的精确入口。' % (current_ns, local_contract['geometry_deadline_ns'], result['actual_r5']['log_header_past_geometry_deadline_ns']/1e6),
        '- 下一源 1791890537992609739 的 READY 录包接收 1791890538319964170，早于 FAILED；不能仅由此断言任务回调已经取得锁。',
        '- 原 phase 在失败诊断时仍余 %.3f s，未过 phase 期限。' % (result['actual_r5']['phase_remaining_at_failure_log_ns']/1e9), '',
        '| 边界 | 应有处理 |', '|---|---|']
    for b in boundaries:
        lines.append('| %s：%s | %s |' % (b['boundary'],b['rule'],b['outcome']))
    lines += ['', '系统修复应同时覆盖：', '']
    for f in findings:
        lines.append('- %s [%s]：%s 建议：%s' % (f['id'],f['status'],f['problem'],f['fix']))
    lines += ['', '新增反例应在真实控制流内推进时钟，不能只在函数调用前设置一个静止时刻：', '']
    lines += ['- '+c for c in counterexamples]
    lines += ['', 'request27 的 strict MoveIt：pregrasp、approach、grasp 通过；最终几何 6 gate 通过；final_execution_sequence_checked=true；close_hold_only=true、lift_path_verified=false。promotion 为 EXECUTION_FROZEN/promote=false，outcome 为 PREVIEW_READY/valid_plan=false。正确结论是“已有经过严格规划检查的接触候选，接纳竞态阻断后续”，不是“已经可无条件抓取”或“抓取成功”。', '',
        '本审计没有改生产、运行测试、访问 ROS master 或 fullframes，也没有重新计算大 bag 哈希。源码 SHA、原始消息索引、候选范围和限制见 result.json。']
    O.mkdir(exist_ok=False)
    (O/'audit.md').write_text('\n'.join(lines)+'\n')
    result['audit_sha256']=sha(O/'audit.md')
    result['script_sha256']=sha(__file__)
    (O/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(result=str(O/'result.json'),sha256=sha(O/'result.json'),audit=str(O/'audit.md')),ensure_ascii=False))

if __name__ == '__main__':
    main()
