"""Seal one explicitly pinned, complete GEN54 offline regression.

All input paths, SHA256 values and exact counts are supplied after the run.
This writes release metadata only; it imports no ROS and grants no motion.
"""
import argparse
from collections import Counter
import importlib.util
import json
from pathlib import Path
import time

from gen54_idle_reload_common_20261018 import (
    W, N, BASE_SHA, PREVIOUS, PREVIOUS_SHA, HELPERS, SUPPLEMENTAL_TESTS,
    REVIEWED_OLD_TEST_CHANGES, SUPPLEMENTAL_CAD_INPUTS,
    SUPPLEMENTAL_ORIGINAL_COLLECTION, SUPPLEMENTAL_ORIGINAL_COLLECTION_SHA,
    sha, read, require, storage, validate_sources, write, inside,
)

V = N/'verification'
RELEASE = V/'gen54_release_manifest.json'
RUNNER = W/'.runtime/verify_gen54_parallel_offline_20261018.py'
FINALIZER = W/'.runtime/finalize_gen54_idle_deployment_20261018.py'
SCOPE_EVIDENCE = {'gen54_collision_geometry_r1/original_cases_geometry.json': '157ccff23a02a4875a49b7ad622e63e583755b26b7f2cd7fc2c1ce1642a4e2a6', 'gen54_collision_geometry_r1/independent_review_final_r1.json': '4a05ba17dc55de306a56162d91817d19df85029f49ef15ded72de9ea2251782f', 'gen54_collision_geometry_r1/release_candidate_binding_r1.json': '77e8a16231742b0c816efd2348f6c11978b052ce17173bdc8c18d26f20084054', 'gen54_geometry_candidate_independent_review_r1/result.json': 'd847d08bf3fc651c441114875cab72e008d1bd1b8ebf1abb5447042352af7a59', 'gen54_command_semantics_independent_review_r1/result.json': '3844e284f7d711dce743f89ea97f0b3e09b21a7eefd0f39acb7eb4c36c9040d6', 'gen54_command_semantics_independent_review_r1/model_layer_clarification.json': '7f54ae73d18edd2e8c534ecc1c8f45340788bd7936892c76e046e5fcf2f1c632'}
# File paths are frozen. SCOPE_EVIDENCE and exact collected case inventory remain required.
EXPECTED_NEW_TEST_FILES = 3
EXPECTED_NEW_TEST_PATHS = frozenset(('src/alicia_flexible_grasp_supervisor/tests/test_gen54_collision_contract.py', 'src/alicia_flexible_grasp_supervisor/tests/test_gen54_height_window.py', 'src/alicia_flexible_grasp_supervisor/tests/test_gen54_open_aperture_refinement.py'))
PRODUCTION_SOURCES = frozenset(('src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py', 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/contact_height_placement.py', 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/gripper_geometry.py', 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/tabletop_geometry_candidates.py'))
RECOVERED_RESULT_SHA = '6963ef9c30e17a4b94c0fc0aee62476bce29af4641c3966ad561c0ec05728adb'
RECOVERY_RECEIPT_SHA = 'b650aeb6eb2345f47c31eb3e93bfb03b68b24ce26590f543c50d0b734dba72f3'
RECOVERY_HELPER_SHA = '159a232e039f26c93211b59ab41d6fcbb38e47aaf78dc9f2fcae0ed6c0b20521'


def reviewed_completed_worker_recovery(full, full_path, dependencies):
    """One exact interrupted launcher; every test still needs the checks below."""
    if not full.get('summary_recovered_from_completed_workers'):
        return False
    receipt_path = V/'gen54_full_recovery_r1/recovery_receipt.json'
    review_path = V/'gen54_full_recovery_independent_review_r1/result.json'
    helper_path = W/'.runtime/recover_gen54_completed_full_20261019.py'
    require(full_path == V/'gen54_full_parallel_r1/result.json' and
        sha(full_path) == RECOVERED_RESULT_SHA and
        full['plan_sha256'] == '478deefff62d468da9136918ee709c6198491c83bab51909dc1060327b9f20e2',
        'only the exact completed GEN54 r1 run is recoverable here')
    for path, digest in ((receipt_path, RECOVERY_RECEIPT_SHA),
                         (helper_path, RECOVERY_HELPER_SHA)):
        require(sha(path) == digest and dependencies.get(str(path)) == digest,
            'recovery evidence was not independently bound: '+str(path))
    require(dependencies.get(str(review_path)) == sha(review_path),
        'independent completed-worker recovery review is missing')
    receipt, review = read(receipt_path), read(review_path)
    require(review.get('passed') is True and
        review.get('review_scope') == 'exact_completed_worker_recovery' and
        review.get('full_result_sha256') == RECOVERED_RESULT_SHA and
        review.get('recovery_receipt_sha256') == RECOVERY_RECEIPT_SHA and
        review.get('recovery_helper_sha256') == RECOVERY_HELPER_SHA,
        'recovery review does not bind the exact interrupted run')
    require(receipt['passed'] is True and
        receipt['kind'] == 'GEN54_exact_completed_worker_recovery' and
        receipt['source_sha256'] == full['source_sha256'] and
        receipt['worker_result_sha256'] == full['worker_result_sha256'] and
        receipt['all_pytest_children_reaped'] is True and
        receipt['all_pytest_uninterrupted'] is True and
        receipt['all_recorded_worker_groups_absent'] is True and
        receipt['pytest_direct_child_exit_codes'] == full['pytest_returncodes'] == [0]*7 and
        receipt['worker_wrappers_exit_codes'] == full['worker_returncodes'] == [None]*7,
        'completed tests and unknown wrapper exits must remain distinct')
    require(full['interrupted'] is True and full['elapsed_sec'] is None and
        full['offline_child_cleanup_required'] is None and
        full['offline_child_cleanup_signals'] is None and
        full['launcher_error'] == receipt['original_launcher'] and
        full['launcher_error']['observed_exit_code'] == 143 and
        full['recovery_receipt'] == dict(path=str(receipt_path), sha256=RECOVERY_RECEIPT_SHA),
        'original launcher interruption was hidden or reinterpreted')
    require(all(sha(path) == digest for path, digest in receipt['dependency_sha256'].items()),
        'original completed-worker evidence changed after recovery')
    return True


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
    require(isinstance(SCOPE_EVIDENCE, dict) and SCOPE_EVIDENCE, 'GEN54 original-failure evidence is not yet bound')
    require(type(EXPECTED_NEW_TEST_FILES) is int and EXPECTED_NEW_TEST_FILES > 0, 'GEN54 candidate test-file inventory is not yet bound')
    require(isinstance(REVIEWED_OLD_TEST_CHANGES, list) and REVIEWED_OLD_TEST_CHANGES,
        'GEN54 reviewed old-test exception inventory is not yet bound')
    storage(V)
    require(not RELEASE.exists(), 'GEN54 release already sealed')
    require(args.expected_source_count > 1069 and args.expected_test_files > 322
        and args.expected_tests > 9010 and 1 <= args.expected_workers <= 8,
        'GEN54 must extend the exact GEN53 baseline')
    paths = {name: getattr(args, name).resolve(strict=True)
        for name in ('manifest', 'runner', 'full', 'review')}
    require(paths['runner'] == RUNNER.resolve(strict=True), 'unexpected GEN54 runner')
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
    require(manifest.get('approved_old_test_changes', []) == REVIEWED_OLD_TEST_CHANGES,
        'manifest old-test exceptions differ from the exact reviewed rows')
    supplemental_rows = manifest.get('supplemental_existing_tests', [])
    new_rows = manifest.get('new_tests', [])
    extra_rows = supplemental_rows + new_rows
    require(len(supplemental_rows) == len(SUPPLEMENTAL_TESTS)
        and {row['path'] for row in supplemental_rows} == SUPPLEMENTAL_TESTS
        and len(new_rows) == EXPECTED_NEW_TEST_FILES
        and {row['path'] for row in new_rows} == EXPECTED_NEW_TEST_PATHS
        and len({row['path'] for row in extra_rows}) == len(extra_rows)
        and all(type(row['expected_count']) is int and row['expected_count'] > 0
                for row in extra_rows)
        and args.expected_test_files == 322 + len(extra_rows)
        and args.expected_tests == 9010 + sum(row['expected_count'] for row in extra_rows),
        'GEN54 requires all 9010 original tests plus four existing geometry files and collected new tests')
    require(review.get('passed') is True and review.get('review_scope') ==
        'static_production_runner_and_idle_loader', 'independent static review is incomplete')
    reviewed = review['reviewed_files_sha256']
    reviewed_required = {str(path.resolve()) for path in (RUNNER, FINALIZER, Path(__file__), *HELPERS)}
    require(reviewed_required <= set(reviewed), 'review did not bind runner and idle helpers')
    require(all(sha(path) == digest for path, digest in reviewed.items()),
        'independently reviewed dependency changed')

    spec = importlib.util.spec_from_file_location('gen54_seal_offline_contract', paths['runner'])
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)
    runner.bind_manifest(paths['manifest'], args.manifest_sha256)
    context = runner.current_context(runner.load_serial())
    require(len(context['sources']) == args.expected_source_count and
        review['source_sha256'] == context['sources'], 'reviewed source differs from frozen candidate')
    require(context['compatibility']['variant'] == 'candidate' and
        context['compatibility']['approved_old_test_changes'] ==
            {row['path']: row for row in REVIEWED_OLD_TEST_CHANGES},
        'historical tests differ outside the exact reviewed exceptions')
    require(sha(SUPPLEMENTAL_ORIGINAL_COLLECTION) == SUPPLEMENTAL_ORIGINAL_COLLECTION_SHA,
        'original supplemental geometry inventory changed')
    original_supplement = read(SUPPLEMENTAL_ORIGINAL_COLLECTION)
    original_supplement_cases = [tuple(case) for case in original_supplement['collected_case_identities']]
    require(original_supplement['mode'] == 'collect' and original_supplement['passed'] is True and
        original_supplement['test_bodies_executed'] is False and
        len(original_supplement_cases) == len(set(original_supplement_cases)) == 206 and
        sorted(tuple(case) for row in supplemental_rows for case in row['case_identities']) ==
            sorted(original_supplement_cases), 'original supplemental testcase identities changed')
    for relative, digest in SUPPLEMENTAL_CAD_INPUTS.items():
        path = W/relative
        require(relative not in context['sources'] and path.resolve(strict=True) == path and
            sha(path) == digest and context['compatibility_dependencies'].get(str(path)) == digest,
            'readonly CAD dependency is not pinned: '+relative)
    require(len(context['old_expected_counts']) == 322 and
        sum(context['old_expected_counts'].values()) == 9010,
        'original GEN53 exact case inventory is missing')
    require(len(context['scope']) == args.expected_test_files and
        len(context['case_identities']) == args.expected_tests,
        'frozen candidate does not match the explicit exact scope')
    dependencies = review['dependency_sha256']
    require(isinstance(dependencies, dict) and dependencies,
        'independent review dependencies are missing')
    require(all(Path(name).is_absolute() and sha(name) == digest
        for name, digest in dependencies.items()), 'review dependency changed')
    recovered = reviewed_completed_worker_recovery(full, paths['full'], dependencies)
    for row in REVIEWED_OLD_TEST_CHANGES:
        evidence = Path(row['review_evidence']['path']).resolve(strict=True)
        require(inside(evidence, V) and sha(evidence) == row['review_evidence']['sha256'] and
            dependencies.get(str(evidence)) == row['review_evidence']['sha256'],
            'independent review did not bind an old-test correction')
    for name, digest in SCOPE_EVIDENCE.items():
        require(dependencies.get(str((V/name).resolve(strict=True))) == digest,
            'review did not bind GEN54 original failure evidence: '+name)
    changed = {path: context['sources'][path]
        for path in context['required_changed_sources']}
    require(set(changed) == PRODUCTION_SOURCES,
        'GEN54 production scope differs from reviewed gripper/contact geometry and candidate materialization')
    require(review.get('reviewed_production_sources') == changed,
        'independent review does not bind every final production byte')
    require(review.get('original_failure_replay_reviewed') is True
        and review.get('physical_success_claimed') is False,
        'original failure replay review is required and is not physical success')
    focuses = []
    for name in dependencies:
        if Path(name).suffix != '.json':
            continue
        value = read(name)
        if (isinstance(value, dict)
                and value.get('kind') == 'GEN54_parallel_focus_offline_result'
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
        'complete GEN54 regression has not passed')
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
        and not full['offline_child_cleanup_signals']
        and (recovered or (not full['interrupted'] and full['launcher_error'] is None))
        and not full['missing_worker_results']
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
        (full['worker_returncodes'] == [0]*args.expected_workers or recovered), 'worker set differs')
    budget_path = folder/'launch_resource_budget.json'
    require(sha(budget_path) == full['launch_resource_budget_sha256'], 'launch resource evidence changed')
    evidence = dict(context['compatibility_dependencies'])
    evidence[str(SUPPLEMENTAL_ORIGINAL_COLLECTION)] = SUPPLEMENTAL_ORIGINAL_COLLECTION_SHA
    evidence.update(input_sha)
    evidence.update(dependencies)
    for path in (plan_path, folder/'pytest.xml', budget_path, PREVIOUS,
            V/'gen53_release_manifest.json', FINALIZER, Path(__file__), *HELPERS):
        evidence[str(path.resolve())] = sha(path)
    evidence.update(reviewed)
    require(sha(PREVIOUS) == PREVIOUS_SHA and sha(V/'gen53_release_manifest.json') == BASE_SHA,
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
        if recovered:
            require(worker['interrupted'] is False and
                worker['worker_pgid'] == 32759 + index and
                not runner.group_exists(worker['worker_pgid']),
                'recovered worker was interrupted or its original group still exists')
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
    release = dict(generation='GEN54', passed=True, reviewed=True, installed=True,
        runtime_deployed=False, physical_grasp_verified=False,
        baseline_release_sha256=BASE_SHA, previous_deployment_sha256=PREVIOUS_SHA,
        source_sha256=context['sources'], source_count=args.expected_source_count,
        test_scope_files=args.expected_test_files, test_summary=expected_summary,
        supplemental_existing_test_files=len(supplemental_rows), historical_test_files=322, historical_tests=9010,
        new_test_files=len(new_rows),
        supplemental_existing_test_cases=sum(row['expected_count'] for row in supplemental_rows),
        new_test_cases=sum(row['expected_count'] for row in new_rows),
        actual_regression=str(paths['full']), actual_regression_sha256=args.full_sha256,
        candidate_manifest=str(paths['manifest']), candidate_manifest_sha256=args.manifest_sha256,
        independent_review=str(paths['review']), independent_review_sha256=args.review_sha256,
        runner=str(paths['runner']), runner_sha256=args.runner_sha256,
        worker_count=args.expected_workers, evidence_sha256=evidence, motion_commands_sent=False,
        regression_launcher_interrupted=full['interrupted'],
        regression_summary_recovered=recovered,
        recovery_receipt=full.get('recovery_receipt'),
        new_fresh_scene_robot_and_path_acceptance_required=True, recorded_ns=time.time_ns())
    validate_sources(release)
    write(RELEASE, release)
    require(read(RELEASE) == release, 'sealed release read-back differs')
    print(json.dumps(dict(sealed=True, manifest=str(RELEASE), sha256=sha(RELEASE),
        test_summary=expected_summary), ensure_ascii=False))


if __name__ == '__main__':
    main()
