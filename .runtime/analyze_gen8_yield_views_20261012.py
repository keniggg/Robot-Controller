#!/usr/bin/env python3
"""Offline sealed GEN8 JSON/text review. No ROS imports or calls."""
import hashlib
import json
from pathlib import Path
import re

ROOT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
SESSION = ROOT / 'ros_log/full_latest_frozen_no_skin_20261012T021050Z'
RECORD = ROOT / 'grasp_records/20261012T112501.434384Z_ec184365e63145858bb80bea45a03b94'
WORK = Path(__file__).resolve().parent.parent
OUT = SESSION / 'gen8_analysis'


def sha(data):
    return hashlib.sha256(data).hexdigest()


def main():
    inputs = {}

    def read(path, expected=None):
        raw = path.read_bytes()
        actual = sha(raw)
        if expected is not None:
            assert actual == expected, str(path)
        inputs[str(path)] = dict(sha256=actual, size_bytes=len(raw))
        return json.loads(raw)

    manifest = read(RECORD / 'manifest.json')
    files = {row['path']: row for row in manifest['files']}
    assert manifest['state'] == 'complete' and manifest['recorder_closed'] is True
    assert manifest['result'] == 'failure'
    command = read(RECORD / 'command_result.json', files['command_result.json']['sha256'])
    recorder = read(RECORD / 'recorder_outcome.json', files['recorder_outcome.json']['sha256'])
    assert command['runner_finished'] and recorder['bag_closed']
    audits = {}
    for rel in files:
        if rel.startswith('keyframes_audits/'):
            d = read(RECORD / rel, files[rel]['sha256'])
            if d['request_id'] in (24, 25):
                audits[d['request_id']] = (d, rel)
    # Select the decisive yield audit, not any later canceled copy.
    yield_rel = 'keyframes_audits/0a9a8a8ceb69dadfbaf3e5ab4e9cc855cdfdbd16b4dcb49773b0b5e789cad953.json'
    contact = read(RECORD / yield_rel, files[yield_rel]['sha256'])
    view, view_rel = audits[25]
    assert contact['outcome']['code'] == 'CONTACT_SEARCH_YIELDED_FOR_MEASURED_VIEW'
    assert view['outcome']['code'] == 'UNKNOWN_OBSERVATION_NO_SAFE_VIEW'
    assert contact['grasp_mode_selection']['generation'] == view['grasp_mode_selection']['generation'] == 8
    state = contact['contact_search_yield_attempt']
    proof = view['observe_return']['failed_search']
    yield_evidence = proof['yield_evidence']
    assert len(state['attempted_families']) == len(state['original_families']) == 2
    assert len({r['family_sha256'] for r in state['original_families']}) == 2
    assert state['original_families'] == yield_evidence['original_families']
    assert state['fallback'] is False and state['incumbent_found'] is False
    assert yield_evidence['exhausted_by'] == 'original_family_limit'
    assert state['started_ns'] < yield_evidence['yielded_ns'] < state['budget_deadline_ns'] < state['original_phase_deadline_ns']
    assert view['observe_return']['view_index'] == 2 and not view['observe_return']['view_plan_id']

    log_path = SESSION / 'gen7_yield_compute_deployment/consumers.log'
    raw_log = log_path.read_bytes()
    inputs[str(log_path)] = dict(sha256=sha(raw_log), prefix_bytes=len(raw_log), scope='live log immutable prefix')
    diag = {}
    metrics = []
    task_events = []
    for number, line in enumerate(raw_log.decode().splitlines(), 1):
        match = re.search(r'\[(\d+\.\d+)\]', line)
        if not match:
            continue
        stamp = float(match.group(1))
        if stamp < 1791804301:
            continue
        if 'remote 6D rejected contact evidence: ' in line:
            d = json.loads(line.split('remote 6D rejected contact evidence: ', 1)[1])
            if d['request']['request_id'] == 24:
                diag[d['plan_id']] = dict(stamp=stamp, line=number, evidence=d)
        if 'remote 6D pipeline metrics: ' in line:
            d = json.loads(line.split('remote 6D pipeline metrics: ', 1)[1])
            if d.get('event') == 'request_completed':
                metrics.append(dict(log_line=number, logged_sec=stamp, **{k:d.get(k) for k in
                    ('request_id','generation','status','drop_reason','end_to_end_ms','ros_prepare_ms',
                     'ros_prepare_stages_ms','transport_ms','wsl_total_ms','stage_counts')}))
        if '[Grasp]' in line:
            task_events.append(dict(log_line=number, logged_sec=stamp, line=line))
    prechecks = []
    for row in contact['stable_evaluations']:
        c = row.get('contact_sequence_precheck')
        if not c:
            continue
        event = diag[c['plan_id']]
        assert event['evidence']['precheck'] == c
        curves = [group[name] for group in
            (c['held_pregrasp_selection']['full_path'], c['held_approach_prediction']['guide_path_prediction']['curves'])
            for name in ('command','predicted_actual')]
        prechecks.append(dict(track_id=row['tracking']['track_id'], variant_index=row['variant_index'],
            plan_id=c['plan_id'], failed_stage=c['failed_stage'], reason=c['reason'],
            sample_stamp_sec=c['source_sample']['stamp_sec'], diagnostic_stamp_sec=event['stamp'],
            diagnostic_log_line=event['line'],
            sample_to_diagnostic_wall_sec=event['stamp']-c['source_sample']['stamp_sec'],
            held_endpoint_elapsed_sec=c['held_pregrasp_selection']['elapsed_sec'],
            completed_geometry_elapsed_sec=sum(x['geometry']['elapsed_sec'] for x in curves),
            completed_collision_state_checks=sum(x['collision_state_checks'] for x in curves),
            height_retry=row['contact_height_window_search'],
            sample_to_diagnostic_is_exact_service_duration=False))
    prechecks.sort(key=lambda x:x['sample_stamp_sec'])
    assert len(prechecks) == 2
    orientation = view['observation_failure_diagnostics']['observation_orientation_search']
    checked = [x for x in orientation['variant_results'] if x.get('checked')]
    unchecked = [x for x in orientation['variant_results'] if not x.get('checked')]
    assert len(checked) == 4 and len(unchecked) == 102
    assert all(x['reason'] == 'OBSERVATION_TILT_CHECK_BUDGET_REACHED' for x in unchecked)
    assert all(x['failure_code'] == 'MOVEIT_UNREACHABLE' for x in checked)
    assert all('OBSERVATION_POSE_PLAN_FAILED' in x['reason'] for x in checked)

    core = SESSION / 'core.log'
    with core.open('rb') as handle:
        handle.seek(0,2)
        end = handle.tell()
        start = max(0, end-2000000)
        handle.seek(start)
        raw_core = handle.read(end-start)
    inputs[str(core)] = dict(byte_offset=start, size_bytes=len(raw_core), sha256=sha(raw_core),
                            scope='fixed byte window; source file remains append-only')
    core_lines = []
    requests = []
    results = []
    for line in raw_core.decode(errors='replace').splitlines():
        match = re.search(r'\[(17918044\d+\.\d+)\]',line)
        if not match or not 1791804408.8 <= float(match.group(1)) <= 1791804412.4:
            continue
        if any(x in line for x in ('check_pose_strict request','check_pose_strict result',
                                  'Unable to sample any valid states','Timed out','ABORTED: TIMED_OUT')):
            core_lines.append(line)
        if 'check_pose_strict request' in line:
            assert 'execute=False' in line
            requests.append((float(match.group(1)), line))
        if 'check_pose_strict result' in line:
            results.append((float(match.group(1)), line))
    assert len(requests) == len(results) == 4
    for variant, request, result in zip(checked, requests, results):
        variant['strict_request_log_sec'] = request[0]
        variant['strict_result_log_sec'] = result[0]
        variant['strict_gateway_log_interval_sec'] = result[0]-request[0]
        variant['moveit_goal_sampling_outcome'] = 'Unable to sample any valid states for goal tree; ABORTED: TIMED_OUT'
        variant['endpoint_is_proven_unreachable'] = False

    release = read(SESSION / 'verification/gen7_yield_release_manifest.json')
    deployment = read(SESSION / 'gen7_yield_runtime_deployment.json')
    assert len(release['source_sha256']) == 854
    source_bindings = {}
    for rel in ('src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_remote.py',
                'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py',
                'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/moveit_planner.py'):
        actual = sha((WORK/rel).read_bytes())
        assert actual == release['source_sha256'][rel] == deployment['source_sha256'][rel]
        source_bindings[rel] = actual
    search = view['observation_failure_diagnostics']
    scheduling = search['unknown_observation_scheduling']
    report = dict(schema=1, validation_passed=True, record=str(RECORD), inputs=inputs,
        analysis_script_sha256=sha(Path(__file__).read_bytes()), relevant_source_sha256=source_bindings,
        bag_identity=files['keyframes.bag'], bag_rehashed_by_this_analysis=False,
        bag_integrity_scope='Uses sealed manifest identity; root independently verified bag hash. This review read JSON/text only.',
        result=command['result'], failure_reason=command['failure_reason'], recorder_closed=recorder['bag_closed'],
        keyframe_count=recorder['keyframe_count'], selected_user_generation=8,
        contact_request_id=24, contact_internal_generation=7, second_view_request_id=25, second_view_internal_generation=9,
        original_family_completion=dict(attempted_count=2, completed_count=2, collision_receipt_count=1,
            fallback=False, completion_records=state['original_families'], deferred_height_plan_ids=state['deferred_height_families'],
            yielded_code=contact['outcome']['code'], bound_proof=proof,
            completed_counter_code_executed=True, yielded_second_observation_search=True,
            gen7_early_local_rejection_regression_trigger_reproduced=False,
            qualification='Both attempts completed; this validates the deployed yield/second-view chain, not the old counter bug discriminating early-local-rejection prefix.'),
        prechecks=prechecks, request_metrics=metrics, task_events=task_events,
        second_observation=dict(phase=view['observe_return']['phase'], view_index=2, selected_plan_id='',
            candidates_generated=106, strict_checked=4, unchecked=102,
            four_checked_views=checked, unchecked_reason='OBSERVATION_TILT_CHECK_BUDGET_REACHED',
            full_path_incumbent_found=False, strict_max_count=6, strict_start_budget_sec=3.0,
            checked_gateway_log_intervals_total_sec=sum(x['strict_gateway_log_interval_sec'] for x in checked),
            post_incumbent_comparison_window_started=False,
            stop_inference='Four intervals alone sum above the original 3-second cumulative start-new-check budget, while count remains below six; source audit marks remaining 102 as tilt-budget skipped. No qualified incumbent exists to enable the post-incumbent window.',
            joint_space_proposal_search={k:search['joint_space_fk_search']['audit'].get(k) for k in
                ('status','elapsed_sec','max_seconds','max_iterations','candidate_count','tier_attempt_count','available_directions')},
            endpoint_ik_hint={k:scheduling['endpoint_ik_search_hint'].get(k) for k in
                ('status','checked_count','hinted_count','maximum_queries','maximum_start_budget_sec','ik_timeout_sec','used_sec')},
            measured_support_schedule=scheduling['measured_support_schedule']),
        timing_limits=['Sample-to-diagnostic intervals are mixed wall-clock windows, not exclusive precheck service duration.',
            'The four strict log intervals exclude request transport before handler and response transport afterward; they are sufficient to show at least 3.313293 seconds of checked work, not exact client budget.used_sec.',
            'MoveIt timeout and failure to sample goal states do not prove mathematical unreachability; 102 untested variants remain unknown.',
            'No per-state RPC or rejected contact curve timer was added. No planning service was called for this review.'],
        production_modified=False, ros_calls=0, motion_commands_sent=0, temperature_queried=False)
    OUT.mkdir(parents=True,exist_ok=True)
    output = OUT / 'trial_yield_and_view_review.json'
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    (OUT/'second_view_core_trace.txt').write_text('\n'.join(core_lines)+'\n')
    lines=['# GEN8 independent yield and second-view review','',
        'GEN8 已封口失败：UNKNOWN_OBSERVATION_NO_SAFE_VIEW / OBSERVATION_SEARCH_INCOMPLETE。第一补视已到位；第二补视进入规划但未选出有效计划，未执行第二补视运动。', '',
        'request24 的 track20 与 track19 均完成原始方向预检，计数为 attempted=2 / completed=2，fallback=false；高度重试均延后。系统按 original_family_limit 生成真实 CONTACT_SEARCH_YIELDED_FOR_MEASURED_VIEW 证据并切换到 request25/view_index2。track20 是预测命令曲线左指碰撞，track19 是名义曲线右指碰撞；因此符合特定 predicted-command 类别的 collision_receipts 只有一条，completed family 有两条。', '',
        '这验证了部署后的完成计数和第二补视链实际运行。本轮两个尝试都完整结束，未复现 GEN7“先局部拒绝却耗掉 family 名额”的区分前缀，不能声称该分支也由实测覆盖。', '',
        '| 严格检查顺序 | 相机距离 m | tilt / roll ° | 网关请求→结果 s |',
        '|---|---:|---:|---:|']
    for i,v in enumerate(checked,1):
        lines.append('| %d | %.6f | %.0f / %.0f | %.6f |' % (i,v['camera_target_distance_m'],
            v['camera_tilt_offset_deg'],v['camera_roll_offset_deg'],v['strict_gateway_log_interval_sec']))
    lines += ['', '四次均为 execute=False 的严格规划，MoveIt 报无法生成有效目标采样并 TIMED_OUT，最终 OBSERVATION_POSE_PLAN_FAILED。四个网关日志区间合计 3.313293 秒，超过原 3 秒累计启动预算；最多 6 次的数量上限尚未到。没有首个合格方案，因此首个合格方案之后的 8 秒比较窗未启动。剩余 **102 个视角均未严格检查**，其可达性未知。', '',
        '此前关节空间补充方案搜索耗时约 2.000 秒，未产出候选；端点 IK 提示在约 3.016 秒内查了 91/96 次，未获得正提示。这些有界搜索失败均不等于目标数学上不可达。', '',
        '两次接触预检样本→客户端诊断墙钟窗口分别约 %.6f / %.6f 秒；endpoint 求解 %.6f / %.6f 秒，已完成状态碰撞采样 %d / %d 次。窗口不是精确服务独占耗时，失败曲线和每次 RPC 仍缺计时。' %
        tuple([prechecks[0]['sample_to_diagnostic_wall_sec'],prechecks[1]['sample_to_diagnostic_wall_sec'],
               prechecks[0]['held_endpoint_elapsed_sec'],prechecks[1]['held_endpoint_elapsed_sec'],
               prechecks[0]['completed_collision_state_checks'],prechecks[1]['completed_collision_state_checks']]), '',
        '报告只读核验 JSON 和文本，854 项 release 中涉及判断的三份生产文件 SHA256 与当前及部署清单相同。未改生产、未调用 ROS 服务或动作、未查温度。原记录失败；不能写成抓取成功。', '']
    (OUT/'trial_yield_and_view_review.md').write_text('\n'.join(lines))
    print(json.dumps(dict(validation_passed=True, output=str(output), sha256=sha(output.read_bytes()),
                         prechecks=2, checked_views=4, unchecked_views=102),ensure_ascii=False))


if __name__ == '__main__':
    main()
