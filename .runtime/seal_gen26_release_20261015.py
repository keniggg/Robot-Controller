"""Seal exact reviewed GEN26 sources and complete offline regression."""
import argparse
import time
from pathlib import Path
from verify_gen26_combined_20261015 import W, N, BASE, BASE_SHA, sha, read, guard, save, inventory, xml_summary


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    for key in ('combined', 'review', 'output'):
        ap.add_argument('--' + key, type=Path, required=True)
    ap.add_argument('--review-sha256', required=True)
    a = ap.parse_args()
    guard(a.output)
    assert not a.output.exists()
    evidence = {}

    def bind(path, digest=None):
        path = Path(path).resolve(strict=True)
        actual = sha(path)
        assert digest is None or digest == actual, str(path)
        assert str(path) not in evidence or evidence[str(path)] == actual
        evidence[str(path)] = actual
        return path

    bind(BASE, BASE_SHA)
    combined = bind(a.combined)
    assert N / 'verification' in combined.parents
    c = read(combined)
    assert c['passed'] is True and c['returncode'] == 0
    assert c['production_worktree_tested'] is True and c['live_ros_contacted'] is False
    assert c['source_unchanged'] is True and c['scope_complete'] is True
    assert c['regression_scope'] == 'full_combined'
    assert c['baseline'] == str(BASE) and c['baseline_sha256'] == BASE_SHA
    summary = c['summary']
    assert summary['tests'] >= read(BASE)['test_summary']['tests']
    assert all(summary[k] == 0 for k in ('failures', 'errors', 'skipped'))
    patch = read(bind(c['patch'], c['patch_sha256']))
    spec = patch['spec']
    bind(patch['spec_path'], patch['spec_sha256'])
    sources, scope = inventory(spec)
    assert sources == c['source_sha256'] and len(sources) == c['source_count']
    assert scope == c['test_scope'] and len(scope) == c['test_scope_files']
    modified = spec['production'] + spec['changed_tests'] + spec['new_tests'] + spec.get('new_resources', [])
    assert set(patch['files']) == set(modified)
    for name, row in patch['files'].items():
        assert row == dict(baseline_sha256=read(BASE)['source_sha256'].get(name), sha256=sources[name])
    bind(combined.parent / 'pytest.log', c['log_sha256'])
    bind(combined.parent / 'pytest.xml', c['xml_sha256'])
    assert xml_summary(combined.parent / 'pytest.xml') == summary
    assert set(c['test_case_counts_by_file']) == set(scope)
    assert all(n > 0 for n in c['test_case_counts_by_file'].values())
    assert sum(c['test_case_counts_by_file'].values()) == summary['tests']
    assert not c['unassigned_testcases']
    bind(W / '.runtime/verify_gen26_combined_20261015.py', c['runner_sha256'])
    review_path = bind(a.review, a.review_sha256)
    review = read(review_path)
    assert review['reviewed'] is True and not review['blocking_findings']
    assert set(modified) <= set(review['reviewed_source_sha256'])
    for name, digest in review['reviewed_source_sha256'].items():
        assert not Path(name).is_absolute() and '..' not in Path(name).parts
        assert sha(W / name) == digest, name
        if name in sources:
            assert sources[name] == digest
    for name, digest in review['evidence_sha256'].items():
        bind(name, digest)
    for name in ('reload_gen20_compute_gateway_20261014.py',
                 'start_observation_measurement_consumers_20261008.sh',
                 'observation_measurement_consumers_20261008.launch',
                 'prepare_gen20_continuation_receipt_20261014.py',
                 'prepare_gen26_continuation_receipt_20261015.py',
                 'resume_gen26_control_20261015.py',
                 'continuous_grasp_authorization_20261012.py',
                 'run_reboot_realigned_grasp_20261010.py',
                 'capture_continuation_scene_20261012.py',
                 'read_gen7_yield_readiness_20261012.py'):
        bind(W / '.runtime' / name)
    previous = bind(N / 'gen25_deployment/result.json')
    dep = read(previous)
    assert dep['deployment_passed'] is True and dep['whole_patch_deployed'] is True
    assert dep['source_manifest_sha256'] == BASE_SHA
    assert len(dict(dep['after'], **dep['preserved_after'])) == 11
    bind(__file__)
    release = dict(schema_version=1, kind='gen26_explicit_box_inference_and_optional_measured_bilateral_contact',
        sealed_ns=time.time_ns(), passed=True, reviewed=True, installed=True,
        installed_semantics='Exact source tested offline; three compute consumers and gateway reload pending.',
        source_sha256=sources, source_count=len(sources), test_scope=scope, test_scope_files=len(scope),
        test_summary=summary, test_case_counts_by_file=c['test_case_counts_by_file'],
        regression_scope='full_combined', baseline_release=str(BASE), baseline_release_sha256=BASE_SHA,
        actual_regression=str(combined), combined_review=str(review_path), evidence_sha256=evidence,
        previous_deployment=str(previous), runtime_deployment_pending=True,
        physical_grasp_verified=False, motion_commands_sent=False, temperature_queried=False)
    save(a.output, release)
    print(dict(release=str(a.output), sha256=sha(a.output), summary=summary), flush=True)


if __name__ == '__main__':
    main()
