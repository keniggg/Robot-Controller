"""Recover only the completed GEN54 r1 workers after loss of their launcher.

No tests or process control are run. Wrapper return codes remain unknown.
The launcher interruption remains explicit; a separate review is required
before the exact recovered result can be used by the release sealer.
"""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import time
import xml.etree.ElementTree as ET
from collections import Counter
from copy import deepcopy

W = Path(__file__).resolve().parents[1]
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
V = E/'ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
OUT = V/'gen54_full_parallel_r1'
RECEIPT = V/'gen54_full_recovery_r1'
RUNNER = W/'.runtime/verify_gen54_parallel_offline_20261018.py'
RUNNER_SHA = '2b9023c285ce3aadaaad37c0f734b96a567963dc384838ed3319c3858d24b5f5'
MANIFEST = V/'gen54_installed_candidate_r1/candidate_manifest.json'
MANIFEST_SHA = 'aebaa01c523bfb2f8d8351f05b4e060ed6091424f4847cfaf0c76023729a6cfe'
PLAN_SHA = '478deefff62d468da9136918ee709c6198491c83bab51909dc1060327b9f20e2'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_bytes())


def save(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())


def main():
    assert E.parents[1].is_mount()
    assert not RECEIPT.exists() and not (OUT/'result.json').exists()
    assert not (OUT/'pytest.xml').exists()
    assert sha(RUNNER) == RUNNER_SHA and sha(MANIFEST) == MANIFEST_SHA
    plan_path = OUT/'plan.json'
    plan, plan_sha = read(plan_path), sha(plan_path)
    assert plan_sha == PLAN_SHA
    assert plan['output'] == str(OUT) and plan['runner_sha256'] == RUNNER_SHA
    assert plan['candidate_manifest_sha256'] == MANIFEST_SHA
    assert plan['mode'] == plan['scope_kind'] == 'full'
    assert plan['worker_count'] == 7 and plan['expected_case_count'] == 9255
    assert plan['expected_file_count'] == 329 and plan['full_regression_scope']
    spec = importlib.util.spec_from_file_location('gen54_recovery_contract', RUNNER)
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)
    runner.bind_manifest(MANIFEST, MANIFEST_SHA)
    serial = runner.load_serial()
    context = runner.current_context(serial)
    assert plan['context'] == context and runner.stable(serial, plan)
    assert len(context['sources']) == 1075
    dependencies = {str(plan_path): plan_sha, str(RUNNER): RUNNER_SHA,
                    str(MANIFEST): MANIFEST_SHA}
    dependencies.update(context['compatibility_dependencies'])
    results, cases, bindings = [], [], {}
    summary = dict(tests=0, failures=0, errors=0, skipped=0)
    combined = ET.Element('testsuites')
    for index, shard in enumerate(plan['shards']):
        folder = OUT/('worker_%d' % index)
        path = folder/'result.json'
        row = read(path)
        assert row['worker_index'] == shard['index'] == index
        assert row['plan_sha256'] == plan_sha
        assert row['test_scope'] == shard['scope']
        assert row['expected_count'] == shard['expected_count']
        assert row['mode'] == row['scope_kind'] == 'full'
        assert all(row[key] is True for key in ('passed', 'test_bodies_executed',
            'full_regression_scope', 'scope_complete', 'source_unchanged',
            'direct_child_reaped'))
        assert row['interrupted'] is False and row['returncode'] == 0
        assert not row['direct_child_cleanup_signals']
        assert not row['foreign_or_inconsistent_imports']
        assert not row['external_boundary_attempts']
        assert row['required_changed_sources'] == shard['required_changed_sources']
        assert all(row['loaded_changed_sources'][p] for p in shard['required_changed_sources'])
        assert not runner.group_exists(row['worker_pgid'])
        assert row['worker_pgid'] == 32759 + index
        for name, key in [('pytest.xml', 'xml_sha256'), ('pytest.log', 'log_sha256'),
                          ('module_provenance.json', 'module_provenance_sha256')]:
            assert sha(folder/name) == row[key]
            dependencies[str(folder/name)] = row[key]
        for name, digest in row['generated_bootstrap_sha256'].items():
            assert not Path(name).is_absolute() and '..' not in Path(name).parts
            assert sha(folder/name) == digest
            dependencies[str(folder/name)] = digest
        boundary = folder/'boundary_attempts.jsonl'
        assert not boundary.exists() or not boundary.read_text().splitlines()
        if boundary.exists():
            dependencies[str(boundary)] = sha(boundary)
        child_summary, child_cases, _ = runner.inventory(folder/'pytest.xml')
        assert child_summary == row['summary']
        assert child_cases == row['case_identities']
        assert Counter(map(tuple, child_cases)) == Counter(map(tuple, shard['case_identities']))
        assert child_summary['tests'] == shard['expected_count']
        assert all(child_summary[k] == 0 for k in ('failures', 'errors', 'skipped'))
        for key in summary:
            summary[key] += child_summary[key]
        cases.extend(child_cases)
        for suite in ET.parse(folder/'pytest.xml').getroot().iter('testsuite'):
            combined.append(deepcopy(suite))
        bindings[str(path)] = sha(path)
        dependencies[str(path)] = sha(path)
        results.append(row)
    assert not Path('/proc/32756').exists()
    assert summary == dict(tests=9255, failures=0, errors=0, skipped=0)
    assert len(cases) == len(set(map(tuple, cases))) == 9255
    assert Counter(map(tuple, cases)) == Counter(map(tuple, context['case_identities']))
    counts = dict(Counter(c[0] for c in cases))
    assert counts == plan['expected_counts_by_file'] and len(counts) == 329
    assert runner.stable(serial, plan) and sha(plan_path) == plan_sha
    assert all(sha(p) == h for p, h in dependencies.items())
    launcher = dict(observed_tool_session_id=34939, observed_exit_code=143,
        original_pid=32756, cause='undetermined',
        original_launcher_returned_no_summary=True,
        wrapper_exit_codes='unknown; not reconstructed from pytest exit codes')
    RECEIPT.mkdir()
    receipt = dict(kind='GEN54_exact_completed_worker_recovery', passed=True,
        scope='completed_pytest_evidence_only_no_rerun_no_process_control',
        original_output=str(OUT), original_plan_sha256=plan_sha,
        runner_sha256=RUNNER_SHA, source_sha256=context['sources'],
        original_launcher=launcher, worker_wrappers_exit_codes=[None]*7,
        pytest_direct_child_exit_codes=[r['returncode'] for r in results],
        all_pytest_children_reaped=True, all_pytest_uninterrupted=True,
        all_recorded_worker_groups_absent=True, worker_result_sha256=bindings,
        exact_test_summary=summary, dependency_sha256=dependencies,
        helper_sha256=sha(__file__), recorded_ns=time.time_ns(),
        tests_reexecuted=False, motion_commands_sent=False,
        temperature_queried=False, runtime_reloaded=False,
        requires_independent_release_review=True)
    receipt_path = RECEIPT/'recovery_receipt.json'
    save(receipt_path, receipt)
    ET.ElementTree(combined).write(OUT/'pytest.xml', encoding='utf-8', xml_declaration=True)
    actual_summary, actual_cases, _ = runner.inventory(OUT/'pytest.xml')
    assert actual_summary == summary and Counter(map(tuple, actual_cases)) == Counter(map(tuple, cases))
    globally_loaded = {p: any(r['loaded_changed_sources'].get(p, False) for r in results)
                       for p in context['required_changed_sources']}
    assert all(globally_loaded.values())
    result = dict(passed=True, summary=summary, scope_complete=True,
        source_unchanged=True, kind='GEN54_parallel_full_offline_result',
        scope_kind='full', mode='full', candidate_manifest_path=str(MANIFEST),
        candidate_manifest_sha256=MANIFEST_SHA, full_regression_scope=True,
        full_regression_passed=True, test_bodies_executed=True,
        collected_case_identities=[], collected_nodeids=[], collected_case_count=0,
        external_boundary_attempts=[], foreign_or_inconsistent_imports={},
        elapsed_sec=None, elapsed_sec_unavailable_reason='original launcher summary was lost',
        longest_worker_elapsed_sec=max(r['elapsed_sec'] for r in results),
        worker_count=7, resource_budget=plan['resource_budget'],
        launch_resource_budget=read(OUT/'launch_resource_budget.json'),
        launch_resource_budget_sha256=sha(OUT/'launch_resource_budget.json'),
        loaded_changed_sources_global=globally_loaded, test_scope_files=329,
        source_count=1075, test_case_counts_by_file=counts,
        duplicate_case_identities=[], missing_case_identities=[],
        unexpected_case_identities=[], missing_worker_results=[],
        worker_result_sha256=bindings, worker_returncodes=[None]*7,
        pytest_returncodes=[r['returncode'] for r in results],
        interrupted=True, launcher_error=launcher,
        offline_child_cleanup_required=None, offline_child_cleanup_signals=None,
        original_parent_cleanup_status='unknown; all pytest children completed without cleanup',
        residual_offline_process_groups=[], plan_sha256=plan_sha,
        combined_xml_matches_worker_results=True, xml_sha256=sha(OUT/'pytest.xml'),
        runner_sha256=RUNNER_SHA, source_sha256=context['sources'],
        source_compatibility=context['compatibility'], accepted_for_current_release=False,
        live_ROS_contacted=False, hardware_contacted=False, motion_commands_sent=False,
        temperature_queried=False, deployed=False, physical_grasp_verified=False,
        summary_recovered_from_completed_workers=True,
        recovery_receipt=dict(path=str(receipt_path), sha256=sha(receipt_path)),
        recovery_helper_sha256=sha(__file__))
    assert runner.stable(serial, plan)
    assert all(sha(p) == h for p, h in dependencies.items())
    save(OUT/'result.json', result)
    print(json.dumps(dict(summary_recovered=True, summary=summary,
        receipt_sha256=sha(receipt_path), result_sha256=sha(OUT/'result.json'))))


if __name__ == '__main__':
    main()
