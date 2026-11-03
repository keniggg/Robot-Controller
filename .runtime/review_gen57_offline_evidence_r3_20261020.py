"""Independent read-only GEN57 regression evidence review; writes only external reports.

Never executes tests, imports candidate modules, changes ROS, or reads control state.
"""
import argparse
from collections import Counter
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import time

W = Path(__file__).resolve().parents[1]
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
V = E/'ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
D = W/'.runtime/gen57_candidate_r1'
B = W/'.runtime/gen57_baseline_source'
M = V/'gen57_candidate_full_ready_r1/candidate_manifest.json'
M_SHA = '42929a2e0ba6f1984ba29af284239eefaf179e6c51edcde4c4a9e6cd8a912409'
R = W/'.runtime/verify_gen57_parallel_offline_r2_20261020.py'
TOOLS = {
    'verify_gen57_parallel_offline_r2_20261020.py': '0e217bc784b24c603afa1f4bca7c977f35f9036c03b6b263c9801f749d1c2727',
    'prepare_gen57_offline_manifest_r2_20261020.py': 'c648c8decc911f221da074cb193d93a98a2a3c19319bd02270094f58b44781f8',
    'gen57_offline_result_verifier_r2_20261020.py': '8fb10512bffef2f34d922fd6baac6747a3df3f6d8c7688d13485697aff2b3ed9',
    'start_gen57_offline_unit_r2_20261020.py': '4562ff72a1135d17d90b5b18a9c2d2d895693f18878386bb5d426f4ca755744a',
}


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def read(p):
    return json.loads(Path(p).read_bytes())


def inside(path, root):
    try:
        path.relative_to(root)
        return True
    except ValueError:
        return False


def load(name):
    p = W/'.runtime'/name
    assert sha(p) == TOOLS[name]
    spec = importlib.util.spec_from_file_location(name[:-3], p)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def save(p, value):
    with p.open('x') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--full-result', type=Path)
    a = parser.parse_args()
    out = a.output.resolve()
    assert E.parent.parent.is_mount() and V in out.parents and not out.exists()
    assert out.parent.stat().st_dev == E.stat().st_dev
    started = time.monotonic()
    assert sha(M) == M_SHA
    for filename, digest in TOOLS.items():
        assert sha(W/'.runtime'/filename) == digest
    verifier = load('gen57_offline_result_verifier_r2_20261020.py')
    manifest, context, runner = verifier.candidate_context(M, M_SHA, R, TOOLS[R.name])
    runner.load_serial().storage(out)
    assert manifest['workspace_root'] == str(D) and manifest['full_scope_ready'] is True
    assert len(context['sources']) == 1088 and len(context['scope']) == 342
    baseline = read(B/'runner_manifest.json')['source_sha256']
    old_cases = list(map(tuple, context['old_case_identities']))
    all_cases = list(map(tuple, context['case_identities']))
    assert len(baseline) == 1085 and len(old_cases) == len(set(old_cases)) == 9658
    assert len(all_cases) == len(set(all_cases)) == 9766
    new_cases = set(all_cases) - set(old_cases)
    assert len(new_cases) == 108 and set(old_cases) <= set(all_cases)
    changed = {p for p in baseline if context['sources'][p] != baseline[p]}
    added = set(context['sources']) - set(baseline)
    assert changed == set(manifest['allowed_production_paths']) and len(changed) == 4
    assert added == {r['path'] for r in manifest['new_tests']} and len(added) == 3
    old_tests = set(context['old_expected_counts'])
    assert len(old_tests) == 339 and not old_tests & changed
    source_rows = []
    for path, digest in sorted(context['sources'].items()):
        actual = D/path
        assert sha(actual) == digest
        if path in baseline:
            assert sha(B/path) == baseline[path]
        source_rows.append(dict(relative_path=path, candidate_path=str(actual),
            candidate_realpath=str(actual.resolve(strict=True)), candidate_sha256=digest,
            baseline_path=str(B/path) if path in baseline else None,
            baseline_sha256=baseline.get(path),
            classification='new_test' if path in added else 'changed_production' if path in changed else 'unchanged_GEN56',
            candidate_size=actual.stat().st_size))
    assert len(source_rows) == len({row['relative_path'] for row in source_rows}) == 1088
    focus_path = V/'gen57_focus_r1/result.json'
    assert sha(focus_path) == 'a61a83cf4da43272de7180445f44d353480ab103a4bc2c74ad83d5613aae4b85'
    focus = read(focus_path)
    _, _, _, focus_evidence = verifier.validate_result(focus_path,
        focus['candidate_manifest_path'], focus['candidate_manifest_sha256'], R, TOOLS[R.name], 'focus')
    assert focus['summary'] == dict(tests=623, failures=0, errors=0, skipped=0)
    assert focus['source_sha256'] == context['sources']
    collect_path = V/'gen57_collect_r2/result.json'
    assert sha(collect_path) == 'd28826cfbd3609f4d903769428f0c90dbd71009d7525722cfde90fa8ee9ff5e2'
    collect = read(collect_path)
    freeze = load('prepare_gen57_offline_manifest_r2_20261020.py')
    collect_manifest_path = Path(collect['candidate_manifest_path'])
    cm = runner.bind_manifest(collect_manifest_path, collect['candidate_manifest_sha256'])
    cc = runner.current_context(runner.load_serial())
    collected = freeze.validate_collection(collect_path,
        dict(path=str(collect_manifest_path), sha256=collect['candidate_manifest_sha256']), cm, cc, runner)
    assert set(map(tuple, collected)) == new_cases and len(collected) == 108
    # Independently read actual collect provenance, not only the child JSON flags.
    collect_provenance_count = 0
    for filename, digest in collect['worker_result_sha256'].items():
        child_path = Path(filename)
        assert sha(child_path) == digest
        provenance = read(child_path.parent/'module_provenance.json')
        assert provenance
        for label, row in provenance.items():
            path = Path(row['path'])
            if inside(path, D/'src'):
                expected = context['sources'].get(str(path.relative_to(D)))
            elif inside(path, W/'devel'):
                expected = context['generated_imports'].get(str(path))
            else:
                raise AssertionError(('unexpected collect import', label, row))
            assert sha(path) == row['sha256'] == expected
            collect_provenance_count += 1
    runner.bind_manifest(M, M_SHA)
    assert runner.current_context(runner.load_serial()) == context
    full = None
    if a.full_result:
        fp = a.full_result.resolve(strict=True)
        _, _, result, evidence = verifier.validate_result(fp, M, M_SHA, R, TOOLS[R.name], 'full')
        assert result['summary'] == dict(tests=9766, failures=0, errors=0, skipped=0)
        assert result['worker_count'] <= 7 and result['resource_budget']['reserved_cpu_count'] >= 5
        launch = read(fp.parent/'durable_launch_request.json')
        submitted = read(fp.parent/'durable_launch_submitted.json')
        identity = read(fp.parent/'coordinator_identity.json')
        assert submitted['submitted'] is True and submitted['returncode'] == 0
        assert submitted['request_sha256'] == sha(fp.parent/'durable_launch_request.json')
        assert launch['plan_sha256'] == submitted['plan_sha256'] == result['plan_sha256'] == identity['plan_sha256']
        unit = launch['unit']
        assert unit == identity['durable_unit'] == submitted['unit']
        assert unit.startswith('alicia-gen57-offline-') and unit.endswith('.service')
        probe = subprocess.run(['systemctl', '--user', 'show', unit,
            '--property=Id,LoadState,ActiveState,SubState,MainPID,ControlGroup,InvocationID,Result,ExecMainCode,ExecMainStatus,ExecMainStartTimestamp,ExecMainExitTimestamp'],
            text=True, capture_output=True, timeout=10)
        assert probe.returncode == 0
        props = dict(line.split('=', 1) for line in probe.stdout.splitlines())
        assert props['Id'] == unit and props['Result'] == 'success'
        assert props['ExecMainCode'] == '1' and props['ExecMainStatus'] == '0'
        assert props['ActiveState'] == 'active' and props['SubState'] == 'exited' and props['MainPID'] == '0'
        assert props['InvocationID'] == identity['systemd_invocation_id']
        work = result['measured_workload']; wp = Path(work['path'])
        assert wp == fp.parent/'workload_samples.jsonl' and sha(wp) == work['sha256']
        samples = [json.loads(line) for line in wp.read_text().splitlines()]
        assert len(samples) == work['sample_count'] and len(samples) > 1
        cpu = result['pytest_cpu_user_sec'] + result['pytest_cpu_system_sec']
        full = dict(path=str(fp), sha256=sha(fp), summary=result['summary'],
            elapsed_sec=result['elapsed_sec'], worker_count=result['worker_count'],
            pytest_cpu_sec=cpu, mean_cpu_core_equivalents=cpu/result['elapsed_sec'],
            measured_workload=work, unit_properties=props,
            coordinator_identity=identity, loaded_changed_sources=result['loaded_changed_sources_global'],
            worker_returncodes=result['worker_returncodes'], worker_sha256=result['worker_result_sha256'],
            xml_sha256=result['xml_sha256'], evidence_sha256=evidence)
    assert all(sha(D/p) == h for p, h in context['sources'].items())
    out.mkdir()
    save(out/'source_mapping_1088.json', dict(candidate_manifest_path=str(M), candidate_manifest_sha256=M_SHA,
        candidate_root=str(D), baseline_root=str(B), source_count=1088, sources=source_rows))
    save(out/'case_inventory_9766.json', dict(baseline_case_count=9658, new_case_count=108,
        cases=context['case_identities'], case_counts_by_file=context['expected_counts'],
        full_execution_claimed=full is not None))
    report = dict(kind='GEN57_independent_regression_evidence_review', passed=True,
        candidate_manifest_path=str(M), candidate_manifest_sha256=M_SHA, source_count=1088,
        changed_production_count=4, new_test_files=3, old_test_files_preserved=339,
        baseline_case_count=9658, collected_new_case_count=108, expected_full_case_count=9766,
        case_identities_unique=True, all_baseline_cases_retained=True,
        focus_result_sha256=sha(focus_path), focus_summary=focus['summary'], focus_not_rerun=True,
        focus_evidence_sha256=focus_evidence,
        collect_result_sha256=sha(collect_path), collect_provenance_rows_checked=collect_provenance_count,
        collect_test_bodies_executed=False, collect_not_rerun=True,
        mapping_sha256=sha(out/'source_mapping_1088.json'), inventory_sha256=sha(out/'case_inventory_9766.json'),
        tools_sha256={str(W/'.runtime'/p):h for p,h in TOOLS.items()}, reviewer_sha256=sha(__file__),
        guard_sha256=context['guard_sha256'], audit_sha256=context['audit_sha256'],
        offline_execution_method='Exact hash-bound plan through a unique durable systemd user unit; isolated whole-file pytest shards.',
        controls_revision_dependency=False,
        controls_revision_basis='Reviewed GEN57 runner/current_context/stable and launcher have no current runtime-control read or revision binding; serial helper supplies storage and bootstrap definitions only.',
        storage=dict(mount=str(E.parent.parent), mounted=True, free_bytes=shutil.disk_usage(E).free),
        full_result=full, full_regression_passed=full is not None,
        reviewed_elapsed_sec=time.monotonic()-started, at_ns=time.time_ns(),
        tests_executed_by_reviewer=False, candidate_modified=False, ROS_or_hardware_used=False,
        deployed=False, physical_grasp_verified=False,
        supplemental_cases_not_in_9766=['test_support_source_qualification:28', 'late_consumers:26', 'integration:12'])
    save(out/'result.json', report)
    print(json.dumps(dict(passed=True, report=str(out/'result.json'), sha256=sha(out/'result.json'),
        mapping_sha256=report['mapping_sha256'], full_regression_passed=report['full_regression_passed'],
        full_result=None if full is None else {k:full[k] for k in ('sha256','summary','elapsed_sec','worker_count','pytest_cpu_sec','mean_cpu_core_equivalents','xml_sha256')}), ensure_ascii=False))


if __name__ == '__main__':
    main()
