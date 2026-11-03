"""Seal exact GEN41 sources after full regression and independent review."""
import argparse
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

from verify_gen41_offline_tests_20261017 import W, C, BASE, BASE_SHA, BACKUP, sha, write, guard

P = 'src/alicia_grasp_modes/'
CHANGED = {
    P+'src/alicia_grasp_modes/station_compute_wait_remote.py',
    P+'src/alicia_grasp_modes/station_compute_wait_task.py',
    P+'src/alicia_grasp_modes/station_contact_admission.py',
    P+'src/alicia_grasp_modes/station_compute_reacquisition.py',
}
MODIFIED_TESTS = set()
NEW_TESTS = {
    P+'tests/test_gen41_station_private_pending.py',
    P+'tests/test_gen41_pending_handoff_chain.py',
    P+'tests/test_gen41_task_availability_wait.py',
}
MODULES = set()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    for name in ('combined', 'review', 'output'):
        ap.add_argument('--'+name, type=Path, required=True)
    a = ap.parse_args()
    guard(a.output)
    assert not a.output.exists()
    evidence = {}

    def bind(path, expected=None):
        path = Path(path).resolve(strict=True)
        digest = sha(path)
        assert expected is None or digest == expected, str(path)
        evidence[str(path)] = digest
        return path

    baseline = json.loads(bind(BASE, BASE_SHA).read_text())
    assert baseline['source_count'] == 1029 and baseline['test_scope_files'] == 279
    assert baseline['test_summary']['tests'] == 8043
    backup = json.loads(bind(BACKUP/'manifest.json').read_text())
    assert backup['source_sha256'] == baseline['source_sha256']
    assert all(sha(BACKUP/n) == d for n, d in backup['source_sha256'].items())
    combined = json.loads(bind(a.combined).read_text())
    assert all(combined[k] for k in ('passed', 'source_unchanged', 'scope_complete',
                                    'baseline_file_test_counts_retained'))
    assert combined['returncode'] == 0 and combined['regression_scope'] == 'full_combined'
    assert combined['baseline'] == str(BASE) and combined['baseline_sha256'] == BASE_SHA
    assert not combined['external_boundary_attempts']
    assert combined['live_ros_contacted'] is False and combined['motion_commands_sent'] is False
    new_tests = {str(p.relative_to(W)) for root in (
        W/'src/alicia_grasp_modes/tests', W/'src/alicia_flexible_grasp_supervisor/tests')
        for p in root.glob('test_gen41*.py')}
    assert new_tests == NEW_TESTS
    sources, scope = combined['source_sha256'], combined['test_scope']
    assert set(scope) == set(baseline['test_scope']) | new_tests
    assert set(sources) == set(baseline['source_sha256']) | new_tests | MODULES
    assert len(sources) == combined['source_count'] == 1032
    assert len(scope) == combined['test_scope_files'] == 282
    assert all(sha(W/n) == d for n, d in sources.items())
    changed_baseline = {n for n, d in baseline['source_sha256'].items() if sources[n] != d}
    assert changed_baseline == CHANGED | MODIFIED_TESTS
    summary = combined['summary']
    assert summary['tests'] > baseline['test_summary']['tests']
    assert all(summary[k] == 0 for k in ('failures', 'errors', 'skipped'))
    counts = combined['test_case_counts_by_file']
    assert set(counts) == set(scope) and sum(counts.values()) == summary['tests']
    assert all(counts[n] >= c for n, c in baseline['test_case_counts_by_file'].items())
    xml = bind(a.combined.parent/'pytest.xml', combined['xml_sha256'])
    xml_root = ET.parse(xml).getroot()
    actual = {k: sum(int(s.get(k, 0)) for s in xml_root.iter('testsuite'))
              for k in ('tests', 'failures', 'errors', 'skipped')}
    assert actual == summary
    xml_counts = dict.fromkeys(scope, 0)
    for case in xml_root.iter('testcase'):
        name = case.get('file', '')
        assert name in xml_counts, (name, case.get('name'))
        xml_counts[name] += 1
    assert xml_counts == counts and all(xml_counts.values())
    assert not combined['unassigned_testcases']
    bind(a.combined.parent/'pytest.log', combined['log_sha256'])
    declared = json.loads(bind(a.combined.parent/'scope.json').read_text())
    assert declared['full'] is True and declared['tests'] == scope
    assert declared['source_sha256'] == sources
    assert declared['baseline'] == str(BASE) and declared['baseline_sha256'] == BASE_SHA
    bind(a.combined.parent/'bootstrap/sitecustomize.py', combined['bootstrap_sha256'])
    bind(W/'.runtime/verify_gen41_offline_tests_20261017.py', combined['runner_sha256'])
    review = json.loads(bind(a.review).read_text())
    assert review['reviewed'] is True and review['release_review_complete'] is True
    assert not review['blocking_findings']
    reviewed = review['reviewed_source_sha256']
    assert CHANGED | MODIFIED_TESTS | NEW_TESTS | MODULES <= set(reviewed)
    for n, d in reviewed.items():
        assert not Path(n).is_absolute() and '..' not in Path(n).parts
        bind(W/n, d)
    for path, digest in review['evidence_sha256'].items():
        bind(path, digest)
    assert review['evidence_sha256'].get(str(a.combined.resolve())) == sha(a.combined)
    bind(__file__)
    assert all(sha(p) == d for p, d in evidence.items())
    assert all(sha(W/n) == d for n, d in sources.items())
    release = dict(schema_version=1,
        kind='gen41_private_station_compute_reacquisition_handoff',
        sealed_ns=time.time_ns(), passed=True, reviewed=True, installed=True,
        installed_semantics='Exact disk sources passed offline regression/review; runtime restart pending.',
        source_sha256=sources, source_count=len(sources), test_scope=scope,
        test_scope_files=len(scope), test_summary=summary, test_case_counts_by_file=counts,
        changed_production_sources=sorted(CHANGED),
        modified_existing_tests=sorted(MODIFIED_TESTS), new_tests=sorted(NEW_TESTS),
        regression_scope='full_combined', baseline_release=str(BASE), baseline_release_sha256=BASE_SHA,
        actual_regression=str(a.combined.resolve()), combined_review=str(a.review.resolve()),
        evidence_sha256=evidence, runtime_deployment_pending=True,
        baseline_evidence_role='Unchanged GEN40 history; not GEN41 physical validation.',
        pending_validation_boundary='Nonmetric availability proofs retain private computation only. Public geometry and motion remain blocked until complete fresh measurement, robot state, geometry and path checks pass. Genuine input/identity/pose/support/control changes remain irreversible.',
        physical_grasp_verified=False, motion_commands_sent=False, temperature_queried=False)
    write(a.output, release)
    print(json.dumps(dict(release=str(a.output), sha256=sha(a.output), summary=summary)))


if __name__ == '__main__':
    main()
