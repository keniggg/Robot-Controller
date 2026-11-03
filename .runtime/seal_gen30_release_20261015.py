"""Seal exact reviewed GEN30 sources; historical GEN29 evidence stays historical."""
import argparse
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

from verify_gen30_combined_20261015 import (
    W, E, BASE, BASE_SHA, BACKUP, BACKUP_SHA, sha, read, write, guard, new_tests)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    for name in ('combined', 'review', 'diagnosis', 'previous-deployment', 'output'):
        ap.add_argument('--'+name, required=True, type=Path)
    ap.add_argument('--review-sha256', required=True)
    args = ap.parse_args()
    guard(args.output)
    assert not args.output.exists()
    assert sha(BASE) == BASE_SHA and sha(BACKUP/'manifest.json') == BACKUP_SHA
    baseline = read(BASE)
    evidence, historical = {}, {}

    def bind(path, expected=None):
        path = Path(path).resolve(strict=True)
        digest = sha(path)
        assert expected is None or digest == expected, str(path)
        assert str(path) not in evidence or evidence[str(path)] == digest
        evidence[str(path)] = digest
        return path

    bind(BASE, BASE_SHA)
    bind(BACKUP/'manifest.json', BACKUP_SHA)
    # Changed source evidence is preserved at its independently verified GEN29
    # backup path. It is never relabelled as a check of the new source.
    for name, digest in baseline['evidence_sha256'].items():
        path = Path(name).resolve(strict=True)
        if sha(path) == digest:
            bind(path, digest)
            continue
        relative = str(path.relative_to(W))
        assert baseline['source_sha256'][relative] == digest
        archived = bind(BACKUP/relative, digest)
        historical[name] = dict(path=str(archived), sha256=digest,
            role='immutable_historical_GEN29_source_not_GEN30_validation')
    combined = read(bind(args.combined))
    assert all(combined[k] for k in ('passed', 'source_unchanged', 'scope_complete',
                                    'baseline_file_test_counts_retained', 'production_worktree_tested'))
    assert combined['returncode'] == 0 and combined['regression_scope'] == 'full_combined'
    assert combined['live_ros_contacted'] is False and combined['motion_commands_sent'] is False
    assert combined['baseline'] == str(BASE) and combined['baseline_sha256'] == BASE_SHA
    sources, scope, summary = combined['source_sha256'], combined['test_scope'], combined['summary']
    assert len(sources) == combined['source_count'] and len(scope) == combined['test_scope_files']
    plan = read(bind(combined['scope_plan'], combined['scope_plan_sha256']))
    assert plan['baseline_sha256'] == BASE_SHA
    changed_tests = set(plan['changed_tests'])
    assert len(changed_tests) == len(plan['changed_tests'])
    assert changed_tests <= set(baseline['source_sha256'])
    assert all('/tests/' in name and Path(name).name.startswith('test_')
               for name in changed_tests)
    assert sorted(plan['new_tests']) == new_tests()
    assert set(scope) == set(baseline['test_scope']) | changed_tests | set(new_tests())
    assert summary['tests'] > baseline['test_summary']['tests']
    assert all(summary[k] == 0 for k in ('failures', 'errors', 'skipped'))
    assert all(sha(W/name) == digest for name, digest in sources.items())
    assert set(combined['test_case_counts_by_file']) == set(scope)
    assert sum(combined['test_case_counts_by_file'].values()) == summary['tests']
    assert all(combined['test_case_counts_by_file'][name] >= count
               for name, count in baseline['test_case_counts_by_file'].items())
    patch = read(bind(combined['patch'], combined['patch_sha256']))
    modified = set().union(*(set(plan[k]) for k in
        ('production', 'changed_tests', 'new_resources', 'new_tests')))
    assert set(patch['files']) == modified
    assert set(sources) == set(baseline['source_sha256']) | set(plan['new_resources']) | set(plan['new_tests'])
    for name, row in patch['files'].items():
        assert row == dict(baseline_sha256=baseline['source_sha256'].get(name), sha256=sources[name])
    for name in baseline['source_sha256']:
        assert (sources[name] != baseline['source_sha256'][name]) == (name in modified)
    for path, digest in patch['evidence_sha256'].items():
        bind(path, digest)
    bind(args.combined.parent/'pytest.log', combined['log_sha256'])
    xml = bind(args.combined.parent/'pytest.xml', combined['xml_sha256'])
    actual = {key: sum(int(s.get(key, 0)) for s in ET.parse(xml).getroot().iter('testsuite'))
              for key in ('tests', 'failures', 'errors', 'skipped')}
    assert actual == summary
    bind(W/'.runtime/verify_gen30_combined_20261015.py', combined['runner_sha256'])
    review_path = bind(args.review, args.review_sha256)
    review = read(review_path)
    assert review['reviewed'] is True and not review['blocking_findings']
    required = modified | {'.runtime/verify_gen30_combined_20261015.py',
                           '.runtime/seal_gen30_release_20261015.py'}
    assert required <= set(review['reviewed_source_sha256'])
    for name, digest in review['reviewed_source_sha256'].items():
        assert not Path(name).is_absolute() and '..' not in Path(name).parts
        bind(W/name, digest)
        if name in sources:
            assert sources[name] == digest
    for path, digest in review['evidence_sha256'].items():
        bind(path, digest)
    assert review['evidence_sha256'].get(str(args.combined.resolve())) == sha(args.combined)
    diagnosis = bind(args.diagnosis)
    assert review['evidence_sha256'].get(str(diagnosis)) == sha(diagnosis)
    previous = bind(args.previous_deployment)
    assert E/'ros_log' in previous.parents
    deployment = read(previous)
    assert deployment['deployment_passed'] and deployment['whole_patch_deployed']
    assert deployment['source_manifest_sha256'] == BASE_SHA
    assert len(dict(deployment['after'], **deployment.get('preserved_after', {}))) == 11
    bind(__file__)
    # Recheck every retained byte immediately before publishing the seal.
    assert all(sha(path) == digest for path, digest in evidence.items())
    assert all(sha(W/name) == digest for name, digest in sources.items())
    release = dict(schema_version=1, kind='gen30_exact_source_sampling_topology',
        sealed_ns=time.time_ns(), passed=True, reviewed=True, installed=True,
        installed_semantics='Exact local source passed offline regression and independent review; runtime deployment is pending.',
        source_sha256=sources, source_count=len(sources), test_scope=scope,
        test_scope_files=len(scope), test_summary=summary,
        test_case_counts_by_file=combined['test_case_counts_by_file'],
        regression_scope='full_combined', baseline_release=str(BASE), baseline_release_sha256=BASE_SHA,
        actual_regression=str(args.combined.resolve()), combined_review=str(review_path),
        recorded_diagnosis=str(diagnosis), evidence_sha256=evidence,
        historical_source_evidence=historical,
        previous_deployment=str(previous), runtime_deployment_pending=True,
        physical_grasp_verified=False, motion_commands_sent=False, temperature_queried=False)
    write(args.output, release)
    print(json.dumps(dict(release=str(args.output), sha256=sha(args.output), summary=summary)), flush=True)


if __name__ == '__main__':
    main()
