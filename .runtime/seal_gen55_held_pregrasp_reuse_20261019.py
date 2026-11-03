"""Summarize scoped offline evidence, with no deployment or hardware authority."""
import hashlib
import json
from pathlib import Path

W = Path(__file__).resolve().parents[1]
D = W/'.runtime/gen55_candidate_r1'
I = W/'.runtime/gen55_fk_isolated_r1'
V = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification')
O = V/'gen55_held_pregrasp_reuse_r1'
SOURCE = 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/held_pregrasp_selection.py'
TEST = 'src/alicia_flexible_grasp_supervisor/tests/test_gen55_held_pregrasp_reuse.py'
FIXTURE = 'src/alicia_flexible_grasp_supervisor/tests/fixtures/gen55_g54_held_pregrasp_inputs.json.gz'
PINS = {SOURCE:'6017a740b15e020bfaf51a83c835dce3216016414c5424330fb0d9218c36e38a',
        TEST:'cd086522888db7581e428581324d03f7328902cf812eecf2720b2dd587c2af9e',
        FIXTURE:'34f65601583d1ed385ffc9492e8018153c55dad647b67ebfc53176b5d68712dd'}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(path, value):
    with Path(path).open('x') as f:
        json.dump(value, f, sort_keys=True, indent=2, allow_nan=False); f.write('\n')


def stable(value):
    if isinstance(value, dict):
        return {key:stable(item) for key,item in value.items() if key != 'elapsed_sec'}
    if isinstance(value, list): return [stable(item) for item in value]
    return value


def main():
    dependencies = {}
    def read(path):
        path = Path(path); dependencies[str(path)] = sha(path)
        return json.loads(path.read_bytes())
    assert Path('/media/zhuyupei/SanStand').is_mount()
    for path, expected in PINS.items():
        assert sha(D/path) == sha(I/path) == expected
        dependencies[str(D/path)] = dependencies[str(I/path)] = expected
    baseline = read(V/'gen55_baseline_r1/result.json')
    assert dependencies[str(V/'gen55_baseline_r1/result.json')] == 'c74a26f5bb9de1356141dc8665fa32d5261af3357d434a184ab930c38a55212f'
    assert sha(W/SOURCE) == baseline['source_sha256'][SOURCE] == 'a55fcdc0c1c086cc822d3dd2991d2e0374b29942eae3b0374ad51661ee51fafc'
    preparation = read(O/'isolated_candidate_preparation.json')
    freeze = read(O/'input_freeze.json')
    assert preparation['passed'] and freeze['passed'] and freeze['cases'] == 8
    for path, digest in freeze['dependencies'].items():
        assert sha(path) == digest
        dependencies[path] = digest
    source_review = read(O/'source_review.json')
    assert source_review['passed'] and source_review['candidate_sha256'] == PINS[SOURCE]
    assert sha(O/'source.diff') == source_review['source_diff_sha256']
    dependencies[str(O/'source.diff')] = sha(O/'source.diff')
    runs, replays = {}, {}
    for name, expected_count in [('new_r2', 19), ('legacy_r1', 105), ('replay_reverse_r1', 8)]:
        result = read(O/name/'result.json')
        assert result['passed'] and result['returncode'] == 0 and result['direct_child_reaped']
        assert result['counts'] == dict(tests=expected_count, failures=0, errors=0, skipped=0)
        assert result['source_sha256'] == PINS[SOURCE]
        assert result['boundary_attempts'] == [] and result['invalid_imports'] == {}
        assert result['candidate_source_imported'] and result['candidate_source_unchanged']
        assert result['old_tests_unchanged'] and result['live_source_unchanged']
        assert result['final_integration_test'] is False and result['full_regression'] is False
        for path, digest in result['files'].items():
            assert sha(O/name/path) == digest
            dependencies[str(O/name/path)] = digest
        runs[name] = {key:result[key] for key in ('counts','elapsed_sec','child_cpu_sec','child_maxrss_kib','replay_count')}
        if result['replay_count']:
            rows = read(O/name/'replays.json')
            assert len(rows) == 8 and sorted(row['index'] for row in rows) == list(range(8))
            replays[name] = {row['index']:row for row in rows}
    prior = read(O/'new_r1/result.json')
    assert prior['passed'] is False and prior['counts']['errors'] == 1
    assert prior['boundary_attempts'] == []
    summaries, per_request = {}, []
    for name, rows in replays.items():
        aggregate = {}
        for version in ('baseline','candidate'):
            aggregate[version] = dict(wall_sec=sum(r['outputs'][version]['wall_sec'] for r in rows.values()),
                cpu_sec=sum(r['outputs'][version]['cpu_sec'] for r in rows.values()),
                optimizer_fk_calls=sum(r['outputs'][version]['fk_calls']['optimizer'] for r in rows.values()),
                verification_fk_calls=sum(r['outputs'][version]['fk_calls']['verification'] for r in rows.values()),
                deadline_callbacks=sum(r['outputs'][version]['deadline_callbacks'] for r in rows.values()))
        old,new=aggregate['baseline'],aggregate['candidate']
        aggregate.update(wall_reduction_fraction=1-new['wall_sec']/old['wall_sec'],
            cpu_reduction_fraction=1-new['cpu_sec']/old['cpu_sec'],
            optimizer_fk_reduction_fraction=1-new['optimizer_fk_calls']/old['optimizer_fk_calls'])
        summaries[name] = aggregate
    for index in range(8):
        one,two=(replays[name][index] for name in ('new_r2','replay_reverse_r1'))
        assert one['order'] == ['baseline','candidate'] and two['order'] == ['candidate','baseline']
        compared=[r['outputs'][version] for r in (one,two) for version in ('baseline','candidate')]
        for row in compared:
            assert stable(row['report']) == stable(compared[0]['report'])
            assert row['command'] == compared[0]['command']
            assert row['deadline_callbacks'] == compared[0]['deadline_callbacks']
            assert row['fk_calls']['verification'] == compared[0]['fk_calls']['verification']
        assert one['exact_recorded_report_equal'] and two['exact_recorded_report_equal']
        report = compared[0]['report']
        per_request.append(dict(index=index, diagnostic_call_id=one['diagnostic_call_id'],
            exact_numeric_reports_equal_in_both_orders_and_recorded_history=True,
            selected_command_rad=compared[0]['command'], solver_iterations=[r['iterations'] for r in report['trials']],
            deadline_callbacks=compared[0]['deadline_callbacks'],
            baseline_fk_calls=one['outputs']['baseline']['fk_calls'],
            candidate_fk_calls=one['outputs']['candidate']['fk_calls'],
            timing={name:{version:{key:r['outputs'][version][key] for key in ('wall_sec','cpu_sec')}
                          for version in ('baseline','candidate')}
                    for name,r in [('baseline_first',one),('candidate_first',two)]}))
    dependencies[str(Path(__file__))] = sha(__file__)
    resource_result = V/'gen54_actual_resources_analysis_r1/analysis.json'
    assert sha(resource_result) == '5c0a909fa1a137e522718e9d8366c44e481a5c1f0ceb74b4b7a6b7e9ce055794'
    dependencies[str(resource_result)] = sha(resource_result)
    result = dict(passed=True, scope='isolated_GEN54_baseline_plus_exact_function_local_FK_reuse',
        source_files=PINS, baseline_source_sha256=baseline['source_sha256'][SOURCE],
        new_test_cases=19, existing_related_test_cases=105, distinct_passed_cases=124,
        additional_order_reversed_replay_executions=8, total_test_executions=132,
        runs=runs, benchmark=summaries, per_request=per_request,
        exact_actual_recorded_reports_equal=True, unchanged_verification_fk_counts=True,
        unchanged_deadline_callback_counts=True, failure_and_cancellation_tests_passed=True,
        bounded_per_invocation_cache=True, shared_candidate_owned_files_match_tested_isolation=True,
        final_integration_test=False, full_regression=False, deployment=False,
        source_review=source_review, dependencies=dependencies,
        prior_attempts=[dict(path=str(O/'new_r1/result.json'), passed=False,
            cause='Shared candidate imported finger_cad_separation before that parallel-owned module existed; no test bodies ran.')],
        limitations=[
            'Timings are guarded offline endpoint-function measurements, not a new full strict RPC or hardware task.',
            'The eight actual endpoint selections succeed but their later contact prechecks historically failed; no historical failure is reclassified.',
            'Actual-task held selection total was 17.737750216 seconds; offline timing is not substituted for it.',
            'Pure fixed-model FK transforms only are reused. No feedback, collision result, target qualification or success is cached.',
            'The shared candidate still requires its integrated focused checks and the root-owned complete release workflow.'],
        production_modified=False, ROS_initialized=False, hardware_access=False, temperature_queried=False)
    save(O/'result.json', result)
    summary = ('GEN55 函数内精确 FK 复用：隔离验证通过。\n'
        '19 个新用例 + 105 个原有相关用例 = 124 个不同用例通过，另 8 次互换顺序回放通过；0 fail/error/skip，guard 无越界。\n'
        '8 份实际 GEN54 输入的命令、量化、求解器迭代、完整数值报告与原版及历史记录逐项相同（只排除 elapsed）。\n'
        '原版先运行：8 次选择函数合计 %.6f → %.6f s，减少 %.2f%%。\n'
        '候选先运行：8 次选择函数合计 %.6f → %.6f s，减少 %.2f%%。\n'
        '优化器 FK 总数 112460 → 48948；量化后/初始验证 FK 总数 24420 完全保留，每次截止回调数量逐案不变。\n'
        '仅一次函数调用内最多 64 个精确关节字节键；无跨请求状态、成功缓存或几何/执行检查变更。\n'
        '这是冻结输入函数耗时，不能当作线上 RPC 总耗时或新任务成功。最终集成与完整回归由 root 继续。\n'
        % (summaries['new_r2']['baseline']['wall_sec'],summaries['new_r2']['candidate']['wall_sec'],100*summaries['new_r2']['wall_reduction_fraction'],
           summaries['replay_reverse_r1']['baseline']['wall_sec'],summaries['replay_reverse_r1']['candidate']['wall_sec'],100*summaries['replay_reverse_r1']['wall_reduction_fraction']))
    with (O/'summary.txt').open('x') as f:f.write(summary)
    save(O/'receipt.json',dict(passed=True,result_sha256=sha(O/'result.json'),summary_sha256=sha(O/'summary.txt'),
        helper_sha256=sha(__file__), source_files=PINS, final_integration_test=False, full_regression=False))
    print(json.dumps(dict(passed=True,result_sha256=sha(O/'result.json'),receipt_sha256=sha(O/'receipt.json'),benchmark=summaries)))


if __name__ == '__main__':
    main()
