"""Seal the GEN23 contact handoff delta separately from the historical GEN22 suite."""
import argparse
import json
from pathlib import Path
import time
from verify_gen23_contact_station_20261014 import (
    W, M, N, BASE, BASE_SHA256, PRODUCTION, NEW_TESTS, SCOPE,
    MINIMUM_CASES_BY_FILE, MINIMUM_DELTA_CASES, baseline_release,
    current_source_map, read, save, sha, storage_guard, xml_report)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--delta', '--combined', dest='delta', type=Path, required=True)
    ap.add_argument('--review', type=Path, required=True)
    ap.add_argument('--review-sha256', required=True)
    ap.add_argument('--output', type=Path, required=True)
    a = ap.parse_args()
    output = a.output.resolve()
    storage_guard(output)
    assert output.parent.is_dir() and not output.exists()
    evidence = {}

    def bind(path, expected=None):
        path = Path(path).resolve(strict=True)
        digest = sha(path)
        assert expected is None or digest == expected, str(path)
        assert str(path) not in evidence or evidence[str(path)] == digest, str(path)
        evidence[str(path)] = digest
        return path

    baseline = baseline_release()
    bind(BASE, BASE_SHA256)
    for path, digest in baseline['evidence_sha256'].items():
        bind(path, digest)
    delta_path = bind(a.delta)
    assert N / 'verification' in delta_path.parents
    delta = read(delta_path)
    assert delta['passed'] is True and delta['source_unchanged'] is True and delta['returncode'] == 0
    assert delta['production_worktree_tested'] is True and delta['live_ros_contacted'] is False
    assert delta['regression_scope'] == 'delta_only' and delta['baseline_tests_rerun'] is False
    assert delta['baseline'] == str(BASE) and delta['baseline_sha256'] == BASE_SHA256
    assert delta['baseline_test_summary'] == baseline['test_summary']
    assert delta['baseline_test_scope'] == baseline['test_scope']
    assert delta['test_scope'] == list(SCOPE) and delta['test_scope_files'] == len(SCOPE)
    assert delta['source_count'] == len(delta['source_sha256']) == 942
    assert current_source_map(baseline) == delta['source_sha256']
    summary = delta['summary']
    assert summary['tests'] >= MINIMUM_DELTA_CASES
    assert all(summary[key] == 0 for key in ('failures', 'errors', 'skipped'))
    for filename, key in (('pytest.log', 'log_sha256'), ('pytest.xml', 'xml_sha256')):
        bind(delta_path.parent / filename, delta[key])
    xml_summary, counts, unassigned = xml_report(delta_path.parent / 'pytest.xml')
    assert xml_summary == summary and counts == delta['test_case_counts_by_file']
    assert not unassigned and sum(counts.values()) == summary['tests']
    assert all(counts[name] >= minimum for name, minimum in MINIMUM_CASES_BY_FILE.items())
    patch_path = bind(delta['patch'], delta['patch_sha256'])
    patch = read(patch_path)
    assert patch['baseline'] == str(BASE) and patch['baseline_sha256'] == BASE_SHA256
    assert set(patch['files']) == set(PRODUCTION + NEW_TESTS)
    for name, value in patch['files'].items():
        assert value == dict(baseline_sha256=baseline['source_sha256'].get(name),
                             sha256=delta['source_sha256'][name])
    bind(W / '.runtime/verify_gen23_contact_station_20261014.py', delta['runner_sha256'])
    for name in SCOPE + PRODUCTION:
        bind(W / name, delta['source_sha256'][name])
    review_path = bind(a.review, a.review_sha256)
    review = read(review_path)
    assert review['reviewed'] is True and not review['blocking_findings']
    assert set(PRODUCTION + NEW_TESTS) <= set(review['reviewed_source_sha256'])
    for name, digest in review['reviewed_source_sha256'].items():
        assert not Path(name).is_absolute() and '..' not in Path(name).parts
        assert sha(W / name) == digest, name
        if name in delta['source_sha256']:
            assert delta['source_sha256'][name] == digest, name
    for path, digest in review['evidence_sha256'].items():
        bind(path, digest)
    reload_review = review['reload_review']
    assert not reload_review['blockers']
    assert reload_review['path'] == '.runtime/reload_gen19_compute_20261014.py'
    bind(W / reload_review['path'], reload_review['sha256'])
    for name in ('start_observation_measurement_consumers_20261008.sh',
                 'observation_measurement_consumers_20261008.launch',
                 'prepare_gen20_continuation_receipt_20261014.py',
                 'continuous_grasp_authorization_20261012.py',
                 'run_reboot_realigned_grasp_20261010.py',
                 'capture_continuation_scene_20261012.py',
                 'read_gen7_yield_readiness_20261012.py'):
        bind(W / '.runtime' / name)
    previous_path = bind(N / 'cold_restart_acceptance/result.json')
    previous = read(previous_path)
    assert previous['deployment_passed'] is True and previous['whole_patch_deployed'] is True
    assert previous['source_manifest_sha256'] == BASE_SHA256
    assert len(dict(previous['after'], **previous['preserved_after'])) == 11
    bind(__file__)
    release = dict(schema_version=1, kind='gen23_contact_station_ros_parameter_types',
        sealed_ns=time.time_ns(), passed=True, reviewed=True, installed=True,
        installed_semantics='One production parser change and one new test installed; only the six-file delta was rerun. Three compute consumer reload pending; eight hardware/GUI processes remain preserved.',
        source_sha256=delta['source_sha256'], source_count=delta['source_count'],
        test_scope=delta['test_scope'], test_scope_files=delta['test_scope_files'],
        test_summary=summary, regression_scope='delta_only',
        delta_test_summary=summary, test_case_counts_by_file=counts,
        baseline_release=str(BASE), baseline_release_sha256=BASE_SHA256,
        baseline_test_summary=baseline['test_summary'],
        baseline_test_scope=baseline['test_scope'],
        baseline_test_scope_files=baseline['test_scope_files'], baseline_tests_rerun=False,
        validation_semantics='GEN22 historical 6341-case suite plus this separately counted contact delta; no claim that the full suite was rerun.',
        actual_regression=str(delta_path), combined_review=str(review_path),
        evidence_sha256=evidence, previous_deployment=str(previous_path),
        runtime_deployment_pending=True, physical_grasp_verified=False,
        physical_grasp_success=False, motion_commands_sent=False, temperature_queried=False)
    save(output, release)
    print(json.dumps(dict(release=str(output), sha256=sha(output), delta_summary=summary,
        historical_baseline_summary=baseline['test_summary']), ensure_ascii=False))


if __name__ == '__main__':
    main()
