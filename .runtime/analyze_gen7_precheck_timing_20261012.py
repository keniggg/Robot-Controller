#!/usr/bin/env python3
"""Read-only GEN7 timing reconstruction; no ROS imports, clients or motion."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import re


def digest(data):
    return hashlib.sha256(data).hexdigest()


def callable_name(node):
    if isinstance(node, ast.Name):
        return node.id
    if isinstance(node, ast.Attribute):
        return callable_name(node.value) + '.' + node.attr
    raise ValueError('unexpected callable node: %s' % type(node).__name__)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-dir', required=True)
    args = parser.parse_args()
    output = Path(args.output_dir)
    session = output.parent
    workspace = Path(__file__).resolve().parent.parent
    filenames = ['near_field_rejection_summary.json',
                 'gate_audit_request28_30f81a03f46091da.json',
                 'request_metrics_from_consumers.json']
    inputs = {}
    decoded = {}
    for name in filenames:
        data = (output / name).read_bytes()
        inputs[name] = {'path': str(output / name), 'sha256': digest(data),
                        'size_bytes': len(data)}
        decoded[name] = json.loads(data)
    summary, audit, metrics = [decoded[name] for name in filenames]
    assert inputs[filenames[1]]['sha256'] == '30f81a03f46091daef49413f2bc01a11ab2da4497b9091d373d32fbf3dd1d4b8'
    with Path(metrics['source']).open('rb') as handle:
        log = handle.read(metrics['source_prefix_bytes'])
    assert digest(log) == metrics['source_prefix_sha256']
    inputs['consumers_log_prefix'] = {'path': metrics['source'],
        'size_bytes': len(log), 'sha256': digest(log), 'scope': 'immutable verified prefix only'}

    events = {}
    for line_number, line in enumerate(log.decode().splitlines(), 1):
        marker = 'remote 6D rejected contact evidence: '
        if marker not in line:
            continue
        evidence = json.loads(line.split(marker, 1)[1])
        if evidence.get('request', {}).get('request_id') != 28:
            continue
        stamp = re.search(r'\[(\d+\.\d+)\]', line).group(1)
        events[evidence['plan_id']] = {'line': line_number, 'stamp_sec': float(stamp),
            'logged_unix_seconds': stamp, 'operation': evidence['operation'],
            'precheck': evidence['precheck']}

    checks = []
    seen = set()
    for index, row in enumerate(audit['stable_evaluations']):
        base = row.get('contact_sequence_precheck')
        if not base:
            continue
        retry = row.get('contact_height_window_search') or {}
        cases = [('original', base, 'contact_sequence_precheck')]
        cases += [('height_proposal_%s' % (i+1), proposal['contact_sequence_precheck'],
                   'contact_height_window_search.proposals[%s].contact_sequence_precheck' % i)
                  for i, proposal in enumerate(retry.get('proposals', []))
                  if proposal.get('contact_sequence_precheck')]
        for label, precheck, subpath in cases:
            plan_id = precheck['plan_id']
            assert plan_id not in seen
            seen.add(plan_id)
            event = events[plan_id]
            assert precheck == event['precheck'], (plan_id, 'audit/log evidence mismatch')
            assert precheck['success'] is False and precheck['execution_authorized'] is False
            curves = []
            groups = [('held_pregrasp', precheck['held_pregrasp_selection']['full_path']),
                      ('held_approach', precheck['held_approach_prediction']['guide_path_prediction']['curves'])]
            for group_name, group in groups:
                for name in ('command', 'predicted_actual'):
                    curve = group[name]
                    curves.append({'stage': group_name, 'curve': name,
                        'geometry_elapsed_sec': curve['geometry']['elapsed_sec'],
                        'geometry_fk_checks': curve['geometry']['fk_checks'],
                        'collision_state_checks': curve['collision_state_checks'],
                        'trajectory_sha256': curve['trajectory_sha256']})
            sample = precheck['source_sample']['stamp_sec']
            checks.append({'track_id': row['tracking']['track_id'],
                'variant_index': row['variant_index'], 'case': label, 'plan_id': plan_id,
                'audit_path': 'stable_evaluations[%s].%s' % (index, subpath),
                'evidence_sha256': digest(json.dumps(precheck, sort_keys=True, separators=(',', ':')).encode()),
                'audit_equals_logged_evidence': True,
                'source_sample_stamp_sec': sample,
                'rejection_diagnostic_log_line': event['line'],
                'rejection_diagnostic_stamp_sec': event['stamp_sec'],
                'operation': event['operation'],
                'sample_to_rejection_diagnostic_wall_sec': event['stamp_sec'] - sample,
                'wall_window_scope': 'sample acquisition inside service through client diagnostic after response; excludes entry/lock/setup and includes client decode/log overhead; not exclusive service duration',
                'held_endpoint_elapsed_sec': precheck['held_pregrasp_selection']['elapsed_sec'],
                'completed_geometry_elapsed_sec': sum(c['geometry_elapsed_sec'] for c in curves),
                'completed_collision_state_checks': sum(c['collision_state_checks'] for c in curves),
                'following_sample_acquisition_elapsed_sec': sum(x['elapsed_sec'] for x in precheck['following_sample_acquisition']),
                'following_sample_waited': any(x['waited'] for x in precheck['following_sample_acquisition']),
                'failed_stage': precheck['failed_stage'], 'reason': precheck['reason'],
                'completed_curves': curves})
    assert len(checks) == 5 and len(events) == 5
    assert len([c for c in checks if c['track_id'] == 16]) == 4

    robot = 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/'
    sources = {}
    for name in ('contact_sequence_precheck.py', 'contact_linear_compensation.py',
                 'held_pregrasp_selection.py', 'contact_pregrasp_gateway.py', 'moveit_planner.py'):
        path = workspace / robot / name
        sources[str(path.relative_to(workspace))] = digest(path.read_bytes())
    precheck_source = (workspace / robot / 'contact_sequence_precheck.py').read_text()
    tree = ast.parse(precheck_source)
    handler = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)
                   and n.name == 'handle_contact_sequence_precheck')
    remaining = next(n for n in ast.walk(handler) if isinstance(n, ast.FunctionDef) and n.name == 'remaining')
    remaining_calls = sorted(set(callable_name(n.func) for n in ast.walk(remaining) if isinstance(n, ast.Call)))
    assert remaining_calls == ['ContactPrecheckError', 'math.isfinite', 'min', 'rospy.get_time', 'time.monotonic']
    assert sum(isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id == 'SerialUrdfFk'
               for n in ast.walk(handler)) == 1

    request_rows = []
    for event in metrics['request_completed_events']:
        item = event['metrics']
        request_rows.append({'request_id': item['request_id'], 'generation': item['generation'],
            'status': item['status'], 'log_line': event['log_line'],
            'completion_logged_sec': float(event['logged_unix_seconds']),
            'end_to_end_sec': item['end_to_end_ms']/1000.,
            'prepare_sec': item['ros_prepare_ms']/1000.,
            'prepare_stage_sec': {k: v/1000. for k,v in item['ros_prepare_stages_ms'].items()},
            'transport_sec': item['transport_ms']/1000., 'server_in_transport_sec': item['wsl_total_ms']/1000.,
            'scope': 'recorded metrics; stage timers do not completely partition end-to-end time; completion log timestamp follows measurement'})
    totals = {}
    for name, subset in [('all_five_prechecks', checks), ('track16_original_and_three_proposals', [c for c in checks if c['track_id'] == 16])]:
        totals[name] = {key: sum(c[key] for c in subset) for key in
            ('held_endpoint_elapsed_sec', 'completed_geometry_elapsed_sec',
             'completed_collision_state_checks', 'following_sample_acquisition_elapsed_sec')}
        totals[name]['service_evidence_count'] = len(subset)

    report = {
        'schema': 1, 'analysis_kind': 'offline_reconstruction_and_source_inspection',
        'validation_passed': True, 'performance_fix_implemented': False,
        'production_modified': False, 'ros_imports_or_service_calls': 0,
        'motion_commands_sent': 0, 'temperature_queried': False,
        'record': summary['record'], 'manifest_sha256': summary['manifest_sha256'],
        'inputs': inputs, 'working_tree_source_sha256': sources,
        'source_hash_scope': 'analysis-time worktree source snapshots; not a new deployed-source attestation',
        'analysis_script_sha256': digest(Path(__file__).read_bytes()),
        'requests': request_rows, 'phase_timing': summary['timing'],
        'prechecks': checks, 'totals': totals,
        'deduplication': 'Only original contact_sequence_precheck and each proposal contact_sequence_precheck counted. original_failure is a duplicate and is excluded. Five distinct plan IDs match five raw diagnostic events exactly.',
        'findings': [
            'Track26-to-track16 final rejection gap is 28.017817 seconds and includes the original track16 precheck plus three bounded height proposals. It is not one precheck taking 28 seconds.',
            'All five observed service evidences failed actual predicted-command continuous geometry at grasp. The failed grasp geometry duration and preceding contact compensation/retiming durations were not retained.',
            'GEN4 compute guard fix is in contact_pregrasp_gateway, which GEN7 did not reach. In contact_sequence_precheck, remaining is already a local dual-clock check; model, FK and frozen context are constructed once per service call, not per SLSQP evaluation or collision sample.',
            'Repeated setup across original and proposal calls is visible in source, but each binds its own plan and fresh feedback. There is no timing evidence that setup is dominant or safely reusable without all original binding checks.',
            'The completed pregrasp and guide curves contain 8444 state-validity checks across five calls. The helper constructs one nonpersistent ROS proxy per curve and calls it once per unchanged sample. Per-RPC and curve collision elapsed times are absent, so transport/server cost is a hypothesis, not an attributed 28-second cause.',
            'Held endpoint solving totals 11.262326 seconds across all five calls; lowering iterations, changing curve coverage, reducing samples, caching validity or extending deadlines is not supported by this analysis.'
        ],
        'remaining_local_guard_calls': remaining_calls,
        'unknown_timing': [
            'gateway service queue and planner-lock waiting; parameter reads and FK/context construction',
            'nominal pregrasp planning, selected joint-path replanning and approach/cartesian planning',
            'collision service discovery, transport reconnection, server computation and response waiting',
            'grasp compensation solver, retiming, and rejected grasp continuous-geometry runtime',
            'request27 canceled work and request28 local ranking/registration/serialization outside existing timers'
        ],
        'minimal_observability_proposal': {
            'implementation_status': 'design_only_not_implemented',
            'files': {
                'contact_sequence_precheck.py': 'Add bounded per-call monotonic audit timings for lock wait, setup, sample, nominal pregrasp plan, endpoint solve, selected joint plan, each completed/failing geometry/retiming stage and total. Persist in finally on success and every failure; use existing plan_id.',
                'contact_linear_compensation.py': 'Optional opt-in timing collector for _contact_command_collision_check: discovery time, proxy construction time, RPC count, aggregate RPC elapsed, maximum RPC elapsed, total curve elapsed and whether it exited by invalid state/exception/deadline. Retain integer return, exact sample order and existing pre/post deadline calls. No per-sample log IO.',
                'remote_grasp6d_node.py': 'Optional client-side service wall timer and request/track/variant/proposal identity. Keep this transport metadata separate from bound safety evidence; record even exceptions without overriding original rejection.'
            },
            'invariants': 'No deadline/iteration/curve/sampling/timeout/gate/cache/retry changes; timings do not decide validity. No new ROS calls or movement. Bounded aggregate counters only.',
            'offline_tests': [
                'Fake clock and fake service validate identical state request sequence and counts with collector disabled/enabled, including invalid state, service exception and deadline failure.',
                'Every existing geometry and compensation failure path retains original code/reason plus bounded partial timing; no success fabricated by instrumentation.',
                'Slow proxy/discovery/solver mocks distinguish lock/setup/plan/geometry/RPC timers without double-counting nested intervals; original deadlines still reject.',
                'Five-call original/proposal identity fixture keeps original_failure duplicates out of totals and preserves fresh sample/plan bindings.',
                'Run contact sequence, held pregrasp, linear compensation, final contact selection and full-curve/deadline regressions; no actual ROS services needed.'
            ]
        },
        'optimization_recommendation': 'Deploy only separately proven scheduling/count correction for this iteration. Measure RPC/setup/solver costs first. Persistent connection reuse is a possible later experiment only after transport cost attribution and exact-request/error/restart tests; this report does not recommend deploying it now.',
        'limits': [
            'No complete trajectory or service execution trace was replayed and no live planning services were used. Audit timings are real recorded timers; missing timers cannot be reconstructed as exclusive durations.',
            'Changing scheduling/counting alone does not accelerate five service calls. The unchanged total deadline may still permit only one complete original-plus-three-proposal family after earlier canceled work; completion of multiple families is not guaranteed.',
            'Unchecked candidates and untested families remain unknown; no claim of globally optimal grasp, no physical-success or collision-free execution claim.'
        ]
    }
    total = totals['all_five_prechecks']
    track16 = totals['track16_original_and_three_proposals']
    lines = ['# GEN7 contact precheck timing review (read-only)', '',
        '原始审计恢复出 **5 次**不同 plan ID 的预检：track4 原始方案 1 次；track16 原始方案及 3 个有界高度方案共 4 次。五份审计与消费者日志中的原始响应逐字段一致。`original_failure` 副本未重复计数。', '',
        'track26→track16 的 **28.017817 秒**为两个最终拒绝日志之间的墙钟区间，包含上述 track16 四次预检及周边处理，不能当作一次预检耗时。', '',
        '| Track / 方案 | 样本→响应诊断窗口 (s) | endpoint 求解 (s) | 已完成连续几何 (s) | 已完成状态碰撞检查数 |',
        '|---|---:|---:|---:|---:|']
    for c in checks:
        lines.append('| %s / %s | %.6f | %.6f | %.6f | %s |' %
                     (c['track_id'], c['case'], c['sample_to_rejection_diagnostic_wall_sec'],
                      c['held_endpoint_elapsed_sec'], c['completed_geometry_elapsed_sec'], c['completed_collision_state_checks']))
    lines += ['', '样本→响应诊断窗口从服务内部采样时刻到客户端收到响应后记录日志，不含入站/锁等待和此前初始化，也含客户端解码/日志开销；不是服务独占计时。已完成连续几何列仅含 pregrasp/guide 双曲线，不含最终失败 grasp 曲线。', '',
        '五次预检的 endpoint 求解合计 **%.6f 秒**，已完成连续几何合计 **%.6f 秒**，状态检查 **%s 次**；track16 四次 endpoint 合计 **%.6f 秒**。采样均无需等待。' %
        (total['held_endpoint_elapsed_sec'], total['completed_geometry_elapsed_sec'], total['completed_collision_state_checks'], track16['held_endpoint_elapsed_sec']), '',
        '请求 28 的端到端时间为 61.449909 秒，准备为 2.072191 秒，传输为 0.403636 秒（已包含服务器 0.396632 秒）。严格比较起点到任务失败为 43.689301 秒。请求 27 先前耗费 25.155317 秒后被 TARGET_PROCESSING_PENDING 取消，完整候选审计不可恢复；不能据空计数断言它没有执行检查。近场站位原阶段预算为 90 秒；不能把它与单次预检的最多 15 秒或另一阶段 60 秒混同。', '',
        'GEN4 的优化已在 contact_pregrasp_gateway 部署，但 GEN7 尚未进入该执行入口。当前 contact_sequence_precheck 的 remaining() 只有本地 ROS/monotonic 时钟检查；URDF/FK 和冻结上下文每个调用构造一次，并未随优化器/每个采样重建。原始方案与不同高度方案各自绑定新反馈与 plan ID；不能无证据跨调用复用。', '',
        '状态碰撞 helper 使用非 persistent ServiceProxy，逐原样本调用 /check_state_validity。8444 次服务检查是可证实工作量，网络往返、服务计算和连接成本均无计时，尚不能归因为主要瓶颈。缺失项还包括规划、grasp 补偿/retiming、失败曲线校验、锁等待和初始化。', '',
        '本轮建议先部署独立证实的调度计数修复，不做未归因性能修改。最小后续观测补丁是在 contact_sequence_precheck 按阶段保留 monotonic 时间，在碰撞 helper 增加可选汇总计时（次数、总 RPC 时间、最大 RPC 时间及失败退出），在客户端绑定 request/track/variant/proposal 记录传输时间；所有失败也保留部分计时。具体范围、测试和输入 SHA256 见同名 JSON。所有全曲线、采样、deadline、迭代次数与拒绝规则保持。', '',
        '限制：计数/调度修复本身不会使每次预检变快，原期限内仍可能只容纳一个完整“原始＋三个高度方案”family；不能保证完成更多 family，也不能将未检查候选算成拒绝。本报告未实现生产修改，未调用 ROS 或规划服务，未运动、未查询温度。', '']
    json_path = output / 'precheck_timing_independent_review.json'
    md_path = output / 'precheck_timing_independent_review.md'
    json_path.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    md_path.write_text('\n'.join(lines))
    print(json.dumps({'validation_passed': True, 'distinct_prechecks': len(checks),
        'totals': totals, 'outputs': {str(p): digest(p.read_bytes()) for p in (json_path, md_path)}}, ensure_ascii=False))


if __name__ == '__main__':
    main()
