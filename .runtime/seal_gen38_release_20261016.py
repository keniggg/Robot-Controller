"""Seal exact GEN38 sources after complete regression and independent review."""
import argparse
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

from verify_gen38_offline_tests_20261016 import W, C, BASE, BASE_SHA, BACKUP, sha, write, guard

P = 'src/alicia_flexible_grasp_supervisor/'
CHANGED = {
    P+'scripts/remote_grasp6d_node.py',
    P+'src/alicia_flexible_grasp/vision/regular_box_inference.py',
    P+'src/alicia_flexible_grasp/grasp/gripper_geometry.py',
    P+'src/alicia_flexible_grasp/grasp/contact_evidence.py',
    P+'src/alicia_flexible_grasp/robot/contact_sequence_precheck.py',
    P+'src/alicia_flexible_grasp/robot/observation_path_guard.py',
}
MODULES = {
    P+'src/alicia_flexible_grasp/vision/cuboid_silhouette.py',
    P+'src/alicia_flexible_grasp/vision/cuboid_rgb_interval.py',
    P+'src/alicia_flexible_grasp/robot/contact_replay_receipt.py',
    P+'src/alicia_flexible_grasp/grasp/contact_evidence_wire.py',
}


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
        W/'src/alicia_grasp_modes/tests', W/(P+'tests'))
        for p in root.glob('test_gen38*.py')}
    sources, scope = combined['source_sha256'], combined['test_scope']
    assert set(scope) == set(baseline['test_scope']) | new_tests
    assert set(sources) == set(baseline['source_sha256']) | new_tests | MODULES
    assert len(sources) == combined['source_count'] and len(scope) == combined['test_scope_files']
    assert all(sha(W/n) == d for n, d in sources.items())
    assert {n for n, d in baseline['source_sha256'].items() if sources[n] != d} == CHANGED
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
    bind(W/'.runtime/verify_gen38_offline_tests_20261016.py', combined['runner_sha256'])
    review = json.loads(bind(a.review).read_text())
    assert review['reviewed'] is True and review['release_review_complete'] is True
    assert not review['blocking_findings']
    reviewed = review['reviewed_source_sha256']
    assert CHANGED | MODULES <= set(reviewed)
    for n, d in reviewed.items():
        assert not Path(n).is_absolute() and '..' not in Path(n).parts
        bind(W/n, d)
    for path, digest in review['evidence_sha256'].items():
        bind(path, digest)
    assert review['evidence_sha256'].get(str(a.combined.resolve())) == sha(a.combined)
    bind(__file__)
    assert all(sha(p) == d for p, d in evidence.items())
    assert all(sha(W/n) == d for n, d in sources.items())
    release = dict(schema_version=1, kind='gen38_source_bound_cuboid_and_contact_handoff',
        sealed_ns=time.time_ns(), passed=True, reviewed=True, installed=True,
        installed_semantics='Exact disk sources passed offline regression/review; runtime restart pending.',
        source_sha256=sources, source_count=len(sources), test_scope=scope,
        test_scope_files=len(scope), test_summary=summary, test_case_counts_by_file=counts,
        regression_scope='full_combined', baseline_release=str(BASE), baseline_release_sha256=BASE_SHA,
        actual_regression=str(a.combined.resolve()), combined_review=str(a.review.resolve()),
        evidence_sha256=evidence, runtime_deployment_pending=True,
        baseline_evidence_role='Unchanged GEN37 history; not GEN38 physical validation.',
        physical_grasp_verified=False, motion_commands_sent=False, temperature_queried=False)
    write(a.output, release)
    print(json.dumps(dict(release=str(a.output), sha256=sha(a.output), summary=summary)))


if __name__ == '__main__':
    main()
