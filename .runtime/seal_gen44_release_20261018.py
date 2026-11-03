"""Seal a complete GEN44 offline regression and independent static review.

This writes release metadata only. It imports no ROS and grants no motion.
"""
import argparse
from collections import Counter
import importlib.util
import json
from pathlib import Path
import time

from gen44_idle_reload_common_20261018 import (
    W, N, BASE_SHA, PREVIOUS, PREVIOUS_SHA, HELPERS,
    sha, read, require, storage, validate_sources, write,
)

V = N/'verification'
MANIFEST = V/'gen44_full_preparation_r9/candidate_manifest.json'
MANIFEST_SHA = '48aa802a09bd81f126d76dbb7dc11520b8cbb387024edd91150e5f447c1016a4'
RUNNER = W/'.runtime/verify_gen44_parallel_offline_20261018.py'
RUNNER_SHA = 'bc58605c50de3008a8c5c555da0907696d15676b9612c224e66fd896213a0b6d'
FULL = V/'gen44_full_parallel_r2/result.json'
RELEASE = V/'gen44_release_manifest.json'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--review', type=Path, required=True)
    parser.add_argument('--review-sha256', required=True)
    args = parser.parse_args()
    storage(V)
    require(not RELEASE.exists(), 'GEN44 release already sealed')
    require(sha(MANIFEST) == MANIFEST_SHA and sha(RUNNER) == RUNNER_SHA,
        'candidate manifest or runner changed')
    require(sha(args.review) == args.review_sha256, 'independent review changed')
    review = read(args.review)
    require(review.get('passed') is True and review.get('review_scope') ==
        'static_production_runner_and_idle_loader', 'independent static review is incomplete')
    spec = importlib.util.spec_from_file_location('gen44_seal_offline_contract', RUNNER)
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)
    runner.bind_manifest(MANIFEST, MANIFEST_SHA)
    context = runner.current_context(runner.load_serial())
    require(len(context['sources']) == 1040 and review['source_sha256'] == context['sources'],
        'reviewed source differs from frozen candidate')
    require(not context['compatibility']['approved_old_test_changes'],
        'historical test changes are outside this release')
    full = read(FULL)
    require(all(full.get(key) is True for key in ('passed', 'full_regression_scope',
        'full_regression_passed', 'scope_complete', 'source_unchanged',
        'combined_xml_matches_worker_results')), 'full regression has not passed')
    require(full['summary'] == dict(tests=8289, failures=0, errors=0, skipped=0)
        and full['test_scope_files'] == 290 and full['source_count'] == 1040,
        'full regression scope differs')
    require(full['source_sha256'] == context['sources'] and
        full['candidate_manifest_sha256'] == MANIFEST_SHA and
        full['runner_sha256'] == RUNNER_SHA, 'full regression source binding differs')
    require(not full['external_boundary_attempts'] and not full['foreign_or_inconsistent_imports']
        and not full['residual_offline_process_groups'] and not full['offline_child_cleanup_required'],
        'offline boundary, provenance or cleanup is incomplete')
    require(all(full['loaded_changed_sources_global'].get(path) is True
        for path in context['required_changed_sources']), 'changed production source was not loaded')
    summary, cases, _ = runner.inventory(FULL.parent/'pytest.xml')
    require(summary == full['summary'] and Counter(map(tuple, cases)) ==
        Counter(map(tuple, context['case_identities'])), 'exact testcase inventory differs')
    require(sha(FULL.parent/'pytest.xml') == full['xml_sha256'], 'full XML changed')
    plan_path = FULL.parent/'plan.json'
    require(sha(plan_path) == full['plan_sha256'], 'full execution plan changed')
    plan = read(plan_path)
    require(plan['context'] == context and plan['mode'] == 'full' and
        plan['full_regression_scope'] is True and plan['worker_count'] == 8 and
        plan['candidate_manifest_sha256'] == MANIFEST_SHA and
        plan['runner_sha256'] == RUNNER_SHA, 'full execution plan binding differs')
    worker_paths = {str(FULL.parent/('worker_%d' % index)/'result.json')
        for index in range(plan['worker_count'])}
    require(set(full['worker_result_sha256']) == worker_paths and
        full['worker_returncodes'] == [0]*plan['worker_count'], 'worker set differs')
    evidence = dict(context['compatibility_dependencies'])
    for path in (MANIFEST, RUNNER, FULL, FULL.parent/'plan.json', FULL.parent/'pytest.xml',
            args.review, PREVIOUS, N/'verification/gen43_release_manifest.json',
            W/'.runtime/finalize_gen44_idle_deployment_20261018.py', Path(__file__), *HELPERS):
        evidence[str(path.resolve())] = sha(path)
    for name, digest in review['reviewed_files_sha256'].items():
        require(sha(name) == digest, 'reviewed dependency changed: '+name)
        evidence[name] = digest
    require(sha(PREVIOUS) == PREVIOUS_SHA and
        sha(N/'verification/gen43_release_manifest.json') == BASE_SHA,
        'previous deployment or release changed')
    for name, digest in full['worker_result_sha256'].items():
        path = Path(name)
        require(sha(path) == digest, 'worker result changed')
        worker = read(path)
        require(worker['passed'] and worker['source_unchanged'] and worker['scope_complete'],
            'worker did not complete successfully')
        index = worker['worker_index']
        require(type(index) is int and 0 <= index < plan['worker_count'] and
            path == FULL.parent/('worker_%d' % index)/'result.json', 'worker index/path differs')
        shard = plan['shards'][index]
        require(worker['plan_sha256'] == full['plan_sha256'] and
            worker['test_scope'] == shard['scope'] and worker['mode'] == 'full',
            'worker does not belong to the executed plan')
        boundary = path.parent/'boundary_attempts.jsonl'
        require(not worker['external_boundary_attempts'] and
            not worker['foreign_or_inconsistent_imports'] and
            (not boundary.exists() or not boundary.read_text().splitlines()),
            'worker boundary/provenance changed')
        if boundary.exists():
            evidence[str(boundary)] = sha(boundary)
        child_summary, child_cases, _ = runner.inventory(path.parent/'pytest.xml')
        require(child_summary == worker['summary'] and
            Counter(map(tuple, child_cases)) == Counter(map(tuple, shard['case_identities'])),
            'worker exact testcase inventory differs')
        evidence[name] = digest
        for filename, key in (('pytest.xml', 'xml_sha256'), ('pytest.log', 'log_sha256'),
                ('module_provenance.json', 'module_provenance_sha256')):
            item = path.parent/filename
            require(sha(item) == worker[key], 'worker artifact changed: '+str(item))
            evidence[str(item)] = worker[key]
        for name, digest in worker['generated_bootstrap_sha256'].items():
            item = path.parent/name
            require(sha(item) == digest, 'worker bootstrap changed')
            evidence[str(item)] = digest
    release = dict(generation='GEN44', passed=True, reviewed=True, installed=True,
        runtime_deployed=False, physical_grasp_verified=False,
        baseline_release_sha256=BASE_SHA, previous_deployment_sha256=PREVIOUS_SHA,
        source_sha256=context['sources'], source_count=1040,
        test_scope_files=290, test_summary=full['summary'],
        actual_regression=str(FULL), actual_regression_sha256=sha(FULL),
        candidate_manifest=str(MANIFEST), candidate_manifest_sha256=MANIFEST_SHA,
        independent_review=str(args.review.resolve()), independent_review_sha256=args.review_sha256,
        evidence_sha256=evidence, motion_commands_sent=False,
        new_fresh_scene_robot_and_path_acceptance_required=True, recorded_ns=time.time_ns())
    validate_sources(release)
    write(RELEASE, release)
    require(read(RELEASE) == release, 'sealed release read-back differs')
    print(json.dumps(dict(sealed=True, manifest=str(RELEASE), sha256=sha(RELEASE),
        test_summary=full['summary']), ensure_ascii=False))


if __name__ == '__main__':
    main()
