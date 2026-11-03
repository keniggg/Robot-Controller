#!/usr/bin/env python3
"""Seal source/runner/root-helper reviews and actual focus evidence only."""
import ast
from collections import Counter
import hashlib
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
V = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification')
OUT = V / 'gen54_combined_independent_review_r1'
PINS = {
    'gen54_idle_helpers_root_review_r1/result.json': '56f8ed3d0c437bf6e4c8751f66d7e344ccd208d80d6acc1a54f76c48901d46c2',
    'gen54_idle_helpers_preparation_r5/result.json': '047a3362895d551b6bb260eb61c475d36d3fcdf838df1139815a0acc0dfb0837',
    'gen54_geometry_candidate_independent_review_r1/result.json': 'd847d08bf3fc651c441114875cab72e008d1bd1b8ebf1abb5447042352af7a59',
    'gen54_parallel_runner_final_independent_review_r1/result.json': '74888dbd732764f0097e37a07ab9ac48d206d7a9b1d195aaff102a437ca4f034',
    'gen54_final_collected_r1/candidate_manifest.json': '78e39ce6fb2d56c95d50d672003d54f018994d48eeb222ee7c2ffd92eb3881d3',
    'gen54_final_collect_r1/result.json': '1b62b02b8b3bc2bd1791c236925dbab4d176adfb6bfc21a2a561e24cd3384093',
    'gen54_final_focus_r1/result.json': '949fcb61186171c55d532d283d417a6c7115ddc8fd34a1526f832a0336d7fb32',
    'gen54_source_installation_r1/result.json': 'ab7998e337444e51ace20c0292432a6e2639017d22224a259902fb0937cb1139',
    'gen54_installed_candidate_r1/candidate_manifest.json': 'aebaa01c523bfb2f8d8351f05b4e060ed6091424f4847cfaf0c76023729a6cfe',
    'gen54_old_geometry_collect_r1/result.json': '8658aaa84ba970a2b93fda982a4527740e2aedef722df7739563b50365fee29e',
}


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def read(p):
    return json.loads(Path(p).read_bytes())


def inventory(p):
    root = ET.parse(p).getroot()
    result = []
    for case in root.iter('testcase'):
        assert not any(case.findall(tag) for tag in ('failure', 'error', 'skipped')), 'non-passing XML outcome'
        result.append((case.get('file', ''), case.get('classname', ''), case.get('name', '')))
    assert len(result) == len(set(result))
    return result


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount() and not OUT.exists()
    dependencies = {str(V / p): h for p, h in PINS.items()}
    assert all(sha(p) == h for p, h in dependencies.items())
    data = {p: read(V / p) for p in PINS}
    root_review = data['gen54_idle_helpers_root_review_r1/result.json']
    prep = data['gen54_idle_helpers_preparation_r5/result.json']
    geometry = data['gen54_geometry_candidate_independent_review_r1/result.json']
    runner_review = data['gen54_parallel_runner_final_independent_review_r1/result.json']
    manifest = data['gen54_final_collected_r1/candidate_manifest.json']
    installed = data['gen54_installed_candidate_r1/candidate_manifest.json']
    installation = data['gen54_source_installation_r1/result.json']
    collection = data['gen54_final_collect_r1/result.json']
    focus = data['gen54_final_focus_r1/result.json']
    original_extra = data['gen54_old_geometry_collect_r1/result.json']
    sources = manifest['source_sha256']
    assert len(sources) == 1075 and manifest['full_scope_ready']
    assert installed['workspace_root'] == str(W)
    ignored = {'workspace_root', 'installation_evidence', 'isolated_manifest', 'stage', 'created_ns'}
    assert {k: v for k, v in installed.items() if k not in ignored} == {
        k: v for k, v in manifest.items() if k not in ignored}
    assert installed['installation_evidence'] == dict(path=str(V / 'gen54_source_installation_r1/result.json'),
        sha256=PINS['gen54_source_installation_r1/result.json'])
    assert installed['isolated_manifest'] == dict(path=str(V / 'gen54_final_collected_r1/candidate_manifest.json'),
        sha256=PINS['gen54_final_collected_r1/candidate_manifest.json'])
    assert type(installed['created_ns']) is int and installed['created_ns'] >= manifest['created_ns']
    assert installation['passed'] and installation['source_sha256'] == sources
    assert installation['source_count'] == 1075 and len(installation['changed_files']) == 13
    assert not installation['runtime_reloaded'] and not installation['full_regression_passed']
    assert all(sha(W / p) == h for p, h in sources.items())
    assert all(sha(Path(manifest['workspace_root']) / p) == h for p, h in sources.items())
    assert geometry['review_passed_for_pinned_candidate'] and runner_review['passed']
    production = geometry['production_sha256']
    assert len(production) == 4 and all(sources[p] == h for p, h in production.items())
    assert set(production) == set(manifest['required_changed_sources'])
    assert runner_review['candidate_source_sha256'] == sources
    assert root_review['passed'] and root_review['reviewer'] == 'root'
    assert root_review['review_scope'] == 'root_independent_nine_idle_helpers'
    files = root_review['reviewed_files_sha256']
    assert len(files) == 9 and files == {row['path']: row['sha256'] for row in prep['files']}
    assert all(sha(p) == h for p, h in files.items())
    dependencies.update(root_review['dependency_sha256'])
    for p, h in prep['SCOPE_EVIDENCE'].items():
        assert dependencies[str(V / p)] == h
    for row in manifest['approved_old_test_changes']:
        p, h = row['review_evidence']['path'], row['review_evidence']['sha256']
        assert sha(p) == h
        dependencies[p] = h
    runner = W / '.runtime/verify_gen54_parallel_offline_20261018.py'
    assert sha(runner) == runner_review['runner']['sha256'] == '2b9023c285ce3aadaaad37c0f734b96a567963dc384838ed3319c3858d24b5f5'
    reviewed_files = dict(files)
    reviewed_files[str(runner)] = sha(runner)
    reviewed_files.update({str(W / p): h for p, h in production.items()})
    # The published review retains separate authorship: root independently
    # reviews this agent's helpers; this agent independently reviews sources/runner.
    dependencies[str(runner)] = sha(runner)
    baseline_xml = V / 'gen53_full_parallel_r1/pytest.xml'
    assert sha(baseline_xml) == '3ce8c6f4e6de0be68745042e113f2e727f7b8f3cdc862e4728dca0523b8ebb78'
    dependencies[str(baseline_xml)] = sha(baseline_xml)
    old_cases = inventory(baseline_xml)
    extra_cases = [tuple(c) for row in manifest['new_tests'] + manifest['supplemental_existing_tests'] for c in row['case_identities']]
    assert len(old_cases) == 9010 and len(extra_cases) == 245
    assert Counter(map(tuple, original_extra['collected_case_identities'])) == Counter(
        tuple(c) for row in manifest['supplemental_existing_tests'] for c in row['case_identities'])
    assert collection['passed'] and collection['mode'] == 'collect' and collection['source_unchanged']
    assert collection['source_sha256'] == sources and not collection['test_bodies_executed']
    assert Counter(map(tuple, collection['collected_case_identities'])) == Counter(extra_cases)
    assert not collection['external_boundary_attempts'] and not collection['foreign_or_inconsistent_imports']
    full_cases = old_cases + extra_cases
    assert len(full_cases) == len(set(full_cases)) == 9255 and len(set(c[0] for c in full_cases)) == 329
    assert focus['passed'] and focus['mode'] == 'focus' and focus['test_bodies_executed']
    assert focus['source_sha256'] == sources and focus['source_unchanged'] and focus['scope_complete']
    assert focus['summary'] == dict(tests=836, failures=0, errors=0, skipped=0)
    assert not focus['full_regression_passed'] and not focus['full_regression_scope']
    assert not focus['external_boundary_attempts'] and not focus['foreign_or_inconsistent_imports']
    assert set(focus['loaded_changed_sources_global']) == set(production) and all(focus['loaded_changed_sources_global'].values())
    focus_folder = V / 'gen54_final_focus_r1'
    focus_plan = read(focus_folder / 'plan.json')
    assert sha(focus_folder / 'plan.json') == focus['plan_sha256']
    assert sha(focus_folder / 'pytest.xml') == focus['xml_sha256']
    focus_cases = inventory(focus_folder / 'pytest.xml')
    expected_focus_cases = [c for c in full_cases if c[0] in manifest['focus_scope']]
    assert Counter(focus_cases) == Counter(expected_focus_cases) == Counter(map(tuple, focus_plan['expected_case_identities']))
    assert len(focus_cases) == 836 and len(focus['worker_result_sha256']) == focus['worker_count'] == 7
    for result, folder in ((focus, focus_folder), (collection, V / 'gen54_final_collect_r1')):
        for relative, digest in [('plan.json', result['plan_sha256']), ('pytest.xml', result['xml_sha256'])]:
            p = folder / relative; assert sha(p) == digest; dependencies[str(p)] = digest
        for name, digest in result['worker_result_sha256'].items():
            path = Path(name); assert sha(path) == digest; dependencies[name] = digest
            worker = read(path)
            assert worker['passed'] and worker['source_unchanged'] and worker['scope_complete']
            assert not worker['external_boundary_attempts'] and not worker['foreign_or_inconsistent_imports']
            for filename, key in [('pytest.xml', 'xml_sha256'), ('pytest.log', 'log_sha256'), ('module_provenance.json', 'module_provenance_sha256')]:
                p = path.parent / filename; assert sha(p) == worker[key]; dependencies[str(p)] = worker[key]
            for relative, h in worker['generated_bootstrap_sha256'].items():
                p = path.parent / relative; assert sha(p) == h; dependencies[str(p)] = h
            boundary = path.parent / 'boundary_attempts.jsonl'
            assert not boundary.exists() or not boundary.read_text().splitlines()
            if boundary.exists(): dependencies[str(boundary)] = sha(boundary)
            if result['mode'] == 'collect':
                p = path.parent / 'collected_items.json'
                assert sha(p) == worker['collection_sha256']; dependencies[str(p)] = sha(p)
    assert all(sha(p) == h for p, h in dependencies.items())
    assert all(sha(p) == h for p, h in reviewed_files.items())
    result = dict(passed=True, review_scope='static_production_runner_and_idle_loader',
        source_sha256=sources, reviewed_production_sources=production,
        reviewed_files_sha256=reviewed_files, dependency_sha256=dependencies,
        original_failure_replay_reviewed=True, physical_success_claimed=False,
        source_count=1075, test_scope_files=329, expected_complete_test_count=9255,
        historical_tests=9010, supplemental_existing_tests=206, new_tests=39,
        collected_scope_cases=245, focused_executed_cases=836, focused_elapsed_sec=focus['elapsed_sec'],
        frozen_helper_count=9, approved_old_test_change_count=3,
        authorship=dict(production_and_runner_independent_reviewer='gen54_runtime_efficiency',
            idle_helper_author='gen54_runtime_efficiency', idle_helper_independent_reviewer='root',
            root_review_path=str(V / 'gen54_idle_helpers_root_review_r1/result.json')),
        checks=[
            'Four final production changes retain current strict gates, the original three height alternatives, unchanged evidence limit/deadline/cancellation and revalidation of any fourth proposal.',
            'Final runner adds exactly three reviewed old-test exceptions, preserves 9010 original and 206 supplemental identities, and pins eight CAD inputs without code/import fallback expansion.',
            'Root independently reviewed all nine final operational helpers; r5 changes only the six exact scope-evidence bindings.',
            'Installed workspace has the exact frozen 1075-source map. Installed manifest changes only workspace and installation lineage metadata.',
            'Actual 245-case collection and 836-case focus evidence, worker XML/log/import/guard artifacts, exact identities and four production imports were rechecked without rerunning tests.',
        ],
        limitations=[
            'Complete 9255-case installed-workspace regression remains independently required by the sealer; this review does not mark that ongoing run passed.',
            'Dual nominal/raw-command checks are preserved reference-curve checks, not a complete measured following-error tube or proof of physical grip. Padded-model residual rejection is not evidence of actual solid collision.',
            'No operational helper or ROS/hardware API was invoked by this aggregate; root owns install, full regression and any subsequent authorized reload.',
        ],
        full_regression_passed=False, runtime_reloaded=False, hardware_contacted=False,
        motion_commands_sent=False, temperature_queried=False, recorded_ns=time.time_ns(),
        aggregation_helper=dict(path=str(Path(__file__).resolve()), sha256=sha(__file__)))
    OUT.mkdir()
    with (OUT / 'result.json').open('x') as f:
        json.dump(result, f, ensure_ascii=False, indent=2, allow_nan=False); f.write('\n')
    (OUT / 'summary.txt').write_text('综合静态审查封口：4 个精确生产源码、最终 runner、root 独立复核的 9 个运维 helper 均已绑定；实际 collect 245=206+39 与重点回归 836/836 的 XML/来源/隔离证据再次核对。完整范围 1075 源码、329 文件、9255 用例，已安装字节精确一致。此回执可供 sealer 消费，但不把仍在运行的完整回归记作通过；最终封口仍必须核验实际全量结果。原 strict、控制所有权和 4 替换/7 保留边界不变，实物夹持成功未验证。\n')
    print(json.dumps(dict(result=str(OUT / 'result.json'), sha256=sha(OUT / 'result.json'),
                          dependencies=len(dependencies), reviewed_files=len(reviewed_files)), ensure_ascii=False))


if __name__ == '__main__':
    main()
