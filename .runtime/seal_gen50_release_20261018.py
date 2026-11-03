"""Seal one explicitly pinned, complete GEN50 offline regression.

All input paths, SHA256 values and exact counts are supplied after the run.
This writes release metadata only; it imports no ROS and grants no motion.
"""
import argparse
from collections import Counter
import importlib.util
import json
from pathlib import Path
import time

from gen50_idle_reload_common_20261018 import (
    W, N, BASE_SHA, PREVIOUS, PREVIOUS_SHA, HELPERS,
    sha, read, require, storage, validate_sources, write, inside,
)

V = N/'verification'
RELEASE = V/'gen50_release_manifest.json'
RUNNER = W/'.runtime/verify_gen50_parallel_offline_20261018.py'
FINALIZER = W/'.runtime/finalize_gen50_idle_deployment_20261018.py'
SUPPLEMENTAL_EXISTING_TESTS = frozenset((
    'src/alicia_flexible_grasp_supervisor/tests/test_motion_settle.py',
    'src/alicia_grasp_modes/tests/test_observation_compensation_handoff.py',
))
STATIONARITY_EVIDENCE = {
    'gen49_arrival_motion_audit_r1/analysis_r1.json':
        '41fe7f171a9d5720bd25d636a474801c8074aec5ce468b2589403aa918f39121',
    'gen50_original_failure_policy_review_r1/review.json':
        'ef6dc07fac2903bd326b996052da42c0d8cbb7b4fb42b277b6d1996a347a4111',
    'gen50_settle_source_independent_review_r4/result.json':
        '7aa1c278e9fd942f346eacfb9952da94445aa7299f2a6578a33d1d0e19eab738',
}


def digest_argument(value):
    if len(value) != 64 or any(c not in '0123456789abcdef' for c in value):
        raise argparse.ArgumentTypeError('a lowercase SHA256 is required')
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('manifest', 'runner', 'full', 'review'):
        parser.add_argument('--'+name, type=Path, required=True)
        parser.add_argument('--'+name+'-sha256', type=digest_argument, required=True)
    for name in ('source-count', 'test-files', 'tests', 'workers'):
        parser.add_argument('--expected-'+name, type=int, required=True)
    args = parser.parse_args()
    storage(V)
    require(not RELEASE.exists(), 'GEN50 release already sealed')
    require(args.expected_source_count == 1058 and args.expected_test_files == 312
        and args.expected_tests == 8818 and 1 <= args.expected_workers <= 8,
        'GEN50 stationarity sources and exact tests must extend GEN49')
    paths = {name: getattr(args, name).resolve(strict=True)
        for name in ('manifest', 'runner', 'full', 'review')}
    require(paths['runner'] == RUNNER.resolve(strict=True), 'unexpected GEN50 runner')
    require(all(inside(paths[name], V) for name in ('manifest', 'full', 'review')),
        'manifest, full result and review must stay on external verification storage')
    input_sha = {str(path): getattr(args, name+'_sha256') for name, path in paths.items()}
    require(all(sha(path) == digest for path, digest in input_sha.items()),
        'pinned manifest, runner, full result or review changed')
    manifest, full, review = (read(paths[name]) for name in ('manifest', 'full', 'review'))
    require(Path(manifest['workspace_root']).resolve(strict=True) == W,
        'release must verify the installed project workspace')
    require(manifest.get('variant', 'candidate') == 'candidate'
        and manifest.get('full_scope_ready') is True, 'candidate full scope is not frozen')
    supplemental_rows = manifest.get('supplemental_existing_tests', [])
    new_rows = manifest.get('new_tests', [])
    require(len(supplemental_rows) == 2
        and {row['path'] for row in supplemental_rows} == SUPPLEMENTAL_EXISTING_TESTS
        and len(new_rows) == 1
        and sum(row['expected_count'] for row in supplemental_rows) == 40
        and sum(row['expected_count'] for row in new_rows) == 31,
        'GEN50 keeps historical scope and adds two exact supplements and one stationarity test file')
    require(review.get('passed') is True and review.get('review_scope') ==
        'static_production_runner_and_idle_loader', 'independent static review is incomplete')
    reviewed = review['reviewed_files_sha256']
    reviewed_required = {str(path.resolve()) for path in (RUNNER, FINALIZER, Path(__file__), *HELPERS)}
    require(reviewed_required <= set(reviewed), 'review did not bind runner and idle helpers')
    require(all(sha(path) == digest for path, digest in reviewed.items()),
        'independently reviewed dependency changed')

    spec = importlib.util.spec_from_file_location('gen50_seal_offline_contract', paths['runner'])
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)
    runner.bind_manifest(paths['manifest'], args.manifest_sha256)
    context = runner.current_context(runner.load_serial())
    require(len(context['sources']) == args.expected_source_count and
        review['source_sha256'] == context['sources'], 'reviewed source differs from frozen candidate')
    require(context['compatibility']['variant'] == 'candidate' and
        not context['compatibility']['approved_old_test_changes'], 'historical tests changed')
    require(len(context['old_expected_counts']) == 309 and
        sum(context['old_expected_counts'].values()) == 8747,
        'original GEN49 exact case inventory is missing')
    require(len(context['scope']) == args.expected_test_files and
        len(context['case_identities']) == args.expected_tests,
        'frozen candidate does not match the explicit exact scope')
    # These pinned receipts establish the original source/feedback evidence and
    # the limited static/pure-policy review. They do not claim an original
    # threaded G49 replay, new sensor messages, or physical task success.
    dependencies = review['dependency_sha256']
    require(isinstance(dependencies, dict) and dependencies,
        'independent review dependencies are missing')
    require(all(Path(name).is_absolute() and sha(name) == digest
        for name, digest in dependencies.items()), 'review dependency changed')
    for name, digest in STATIONARITY_EVIDENCE.items():
        require(dependencies.get(str((V/name).resolve(strict=True))) == digest,
            'review did not bind GEN50 stationarity evidence: '+name)
    causal = read(V/'gen50_original_failure_policy_review_r1/review.json')
    closure = read(V/'gen50_settle_source_independent_review_r4/result.json')
    changed = {path: context['sources'][path]
        for path in context['required_changed_sources']}
    require(len(changed) == 3 and causal['passed'] is True
        and closure['static_passed'] is True
        and closure['production_source_sha256'] == changed,
        'limited policy/closure review differs from the three production changes')
    overlay = W/'.runtime/gen50_settle_candidate'
    require(causal['reviewed_candidate_source_sha256'] ==
        {str(overlay/path): digest for path, digest in changed.items()},
        'causal review production source binding differs')
    require(causal['ROS_initialized'] is False and causal['hardware_contacted'] is False
        and causal['full_regression_executed'] is False,
        'limited policy evidence must not claim physical or full-regression validation')
    focuses = []
    for name in dependencies:
        if Path(name).suffix != '.json':
            continue
        value = read(name)
        if (isinstance(value, dict)
                and value.get('kind') == 'GEN50_parallel_focus_offline_result'
                and all(value.get(key) is True for key in
                    ('passed', 'scope_complete', 'source_unchanged', 'test_bodies_executed'))
                and value.get('mode') == 'focus'
                and value.get('source_sha256') == context['sources']
                and not value.get('external_boundary_attempts')
                and not value.get('foreign_or_inconsistent_imports')
                and value['summary']['tests'] > 0
                and all(value['summary'][key] == 0 for key in ('failures', 'errors', 'skipped'))):
            focuses.append(name)
    require(focuses, 'final candidate focused regression is not bound by the review')
    require(all(full.get(key) is True for key in ('passed', 'full_regression_scope',
        'full_regression_passed', 'scope_complete', 'source_unchanged',
        'combined_xml_matches_worker_results', 'test_bodies_executed')),
        'complete GEN50 regression has not passed')
    expected_summary = dict(tests=args.expected_tests, failures=0, errors=0, skipped=0)
    require(full['summary'] == expected_summary and
        full['test_scope_files'] == args.expected_test_files and
        full['source_count'] == args.expected_source_count and
        full['worker_count'] == args.expected_workers and full['mode'] == 'full' and
        full['scope_kind'] == 'full', 'full regression scope differs')
    require(full['source_sha256'] == context['sources'] and
        full['candidate_manifest_sha256'] == args.manifest_sha256 and
        Path(full['candidate_manifest_path']).resolve(strict=True) == paths['manifest'] and
        full['runner_sha256'] == args.runner_sha256, 'full regression source binding differs')
    require(not full['external_boundary_attempts'] and not full['foreign_or_inconsistent_imports']
        and not full['residual_offline_process_groups'] and not full['offline_child_cleanup_required']
        and not full['offline_child_cleanup_signals'] and not full['interrupted']
        and full['launcher_error'] is None and not full['missing_worker_results']
        and not full['duplicate_case_identities'] and not full['missing_case_identities']
        and not full['unexpected_case_identities'], 'offline regression evidence is incomplete')
    require(all(full['loaded_changed_sources_global'].get(path) is True
        for path in context['required_changed_sources']), 'changed production source was not loaded')
    folder = paths['full'].parent
    summary, cases, _ = runner.inventory(folder/'pytest.xml')
    require(summary == expected_summary and Counter(map(tuple, cases)) ==
        Counter(map(tuple, context['case_identities'])), 'full exact testcase inventory differs')
    require(sha(folder/'pytest.xml') == full['xml_sha256'], 'full XML changed')
    plan_path = folder/'plan.json'
    require(sha(plan_path) == full['plan_sha256'], 'full execution plan changed')
    plan = read(plan_path)
    require(plan['context'] == context and plan['mode'] == 'full' and
        plan['full_regression_scope'] is True and plan['worker_count'] == args.expected_workers and
        plan['candidate_manifest_sha256'] == args.manifest_sha256 and
        Path(plan['candidate_manifest_path']).resolve(strict=True) == paths['manifest'] and
        plan['runner_sha256'] == args.runner_sha256 and
        Path(plan['runner_path']).resolve(strict=True) == paths['runner'] and
        Path(plan['output']).resolve(strict=True) == folder,
        'full execution plan binding differs')
    require(plan['test_scope'] == context['scope'] and
        plan['expected_file_count'] == args.expected_test_files and
        plan['expected_case_count'] == args.expected_tests and
        plan['expected_counts_by_file'] == context['expected_counts'] and
        Counter(map(tuple, plan['expected_case_identities'])) ==
            Counter(map(tuple, context['case_identities'])), 'plan exact inventory differs')
    require(len(plan['shards']) == args.expected_workers, 'shard count differs')
    shard_scope = [name for row in plan['shards'] for name in row['scope']]
    require(len(shard_scope) == len(set(shard_scope)) == args.expected_test_files and
        set(shard_scope) == set(context['scope']), 'shards omit or duplicate test files')
    worker_paths = {str(folder/('worker_%d' % index)/'result.json')
        for index in range(args.expected_workers)}
    require(set(full['worker_result_sha256']) == worker_paths and
        full['worker_returncodes'] == [0]*args.expected_workers, 'worker set differs')
    budget_path = folder/'launch_resource_budget.json'
    require(sha(budget_path) == full['launch_resource_budget_sha256'], 'launch resource evidence changed')
    evidence = dict(context['compatibility_dependencies'])
    evidence.update(input_sha)
    evidence.update(dependencies)
    for path in (plan_path, folder/'pytest.xml', budget_path, PREVIOUS,
            V/'gen49_release_manifest.json', FINALIZER, Path(__file__), *HELPERS):
        evidence[str(path.resolve())] = sha(path)
    evidence.update(reviewed)
    require(sha(PREVIOUS) == PREVIOUS_SHA and sha(V/'gen49_release_manifest.json') == BASE_SHA,
        'previous deployment or release changed')
    worker_cases = []
    worker_summary = dict.fromkeys(expected_summary, 0)
    for name, digest in full['worker_result_sha256'].items():
        path = Path(name)
        require(sha(path) == digest, 'worker result changed')
        worker = read(path)
        require(all(worker.get(key) is True for key in ('passed', 'source_unchanged',
            'scope_complete', 'direct_child_reaped', 'test_bodies_executed', 'full_regression_scope'))
            and worker['returncode'] == 0 and not worker['direct_child_cleanup_signals'],
            'worker did not complete successfully')
        index = worker['worker_index']
        require(type(index) is int and 0 <= index < args.expected_workers and
            path == folder/('worker_%d' % index)/'result.json', 'worker index/path differs')
        shard = plan['shards'][index]
        require(shard['index'] == index and worker['plan_sha256'] == full['plan_sha256'] and
            worker['test_scope'] == shard['scope'] and worker['mode'] == 'full' and
            worker['scope_kind'] == 'full' and worker['expected_count'] == shard['expected_count'] and
            worker['required_changed_sources'] == shard['required_changed_sources'] and
            all(worker['loaded_changed_sources'].get(p) is True for p in shard['required_changed_sources']),
            'worker does not belong to the executed plan')
        boundary = path.parent/'boundary_attempts.jsonl'
        require(not worker['external_boundary_attempts'] and not worker['foreign_or_inconsistent_imports']
            and (not boundary.exists() or not boundary.read_text().splitlines()),
            'worker boundary/provenance changed')
        if boundary.exists():
            evidence[str(boundary)] = sha(boundary)
        child_summary, child_cases, _ = runner.inventory(path.parent/'pytest.xml')
        require(child_summary == worker['summary'] and
            child_summary['tests'] == shard['expected_count'] and
            all(child_summary[k] == 0 for k in ('failures', 'errors', 'skipped')) and
            Counter(map(tuple, child_cases)) == Counter(map(tuple, shard['case_identities'])) and
            child_cases == worker['case_identities'], 'worker exact testcase inventory differs')
        worker_cases.extend(child_cases)
        for key in worker_summary:
            worker_summary[key] += child_summary[key]
        evidence[name] = digest
        for filename, key in (('pytest.xml', 'xml_sha256'), ('pytest.log', 'log_sha256'),
                ('module_provenance.json', 'module_provenance_sha256')):
            item = path.parent/filename
            require(sha(item) == worker[key], 'worker artifact changed: '+str(item))
            evidence[str(item)] = worker[key]
        for relative, digest in worker['generated_bootstrap_sha256'].items():
            item = path.parent/relative
            require(not Path(relative).is_absolute() and '..' not in Path(relative).parts and
                sha(item) == digest, 'worker bootstrap changed')
            evidence[str(item)] = digest
    require(worker_summary == expected_summary and Counter(map(tuple, worker_cases)) ==
        Counter(map(tuple, cases)), 'combined XML does not equal the worker union')
    release = dict(generation='GEN50', passed=True, reviewed=True, installed=True,
        runtime_deployed=False, physical_grasp_verified=False,
        baseline_release_sha256=BASE_SHA, previous_deployment_sha256=PREVIOUS_SHA,
        source_sha256=context['sources'], source_count=args.expected_source_count,
        test_scope_files=args.expected_test_files, test_summary=expected_summary,
        supplemental_existing_test_files=2, historical_test_files=309, historical_tests=8747,
        new_test_files=1,
        supplemental_existing_test_cases=sum(row['expected_count'] for row in supplemental_rows),
        new_test_cases=sum(row['expected_count'] for row in new_rows),
        actual_regression=str(paths['full']), actual_regression_sha256=args.full_sha256,
        candidate_manifest=str(paths['manifest']), candidate_manifest_sha256=args.manifest_sha256,
        independent_review=str(paths['review']), independent_review_sha256=args.review_sha256,
        runner=str(paths['runner']), runner_sha256=args.runner_sha256,
        worker_count=args.expected_workers, evidence_sha256=evidence, motion_commands_sent=False,
        new_fresh_scene_robot_and_path_acceptance_required=True, recorded_ns=time.time_ns())
    validate_sources(release)
    write(RELEASE, release)
    require(read(RELEASE) == release, 'sealed release read-back differs')
    print(json.dumps(dict(sealed=True, manifest=str(RELEASE), sha256=sha(RELEASE),
        test_summary=expected_summary), ensure_ascii=False))


if __name__ == '__main__':
    main()
