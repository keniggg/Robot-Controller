"""Seal independently reviewed GEN36 bytes after the complete offline suite."""
import argparse
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

from verify_gen36_offline_tests_20261016 import W, C, BASE, BASE_SHA, BACKUP, sha, write, guard

HELPERS = (
    '.runtime/verify_gen36_offline_tests_20261016.py',
    '.runtime/seal_gen36_release_20261016.py',
    '.runtime/start_gen36_full_restart_20261016.py',
    '.runtime/start_full_latest_gen36_20261016.sh',
    '.runtime/full_latest_gen36_single_observation_20261016.launch',
    '.runtime/verify_gen36_full_restart_20261016.py',
    '.runtime/finalize_gen36_full_restart_20261016.py',
    '.runtime/start_gen18_full_restart_20261013.py',
    'src/alicia_grasp_modes/config/minimal_observation_chain.yaml')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('combined', 'review', 'previous-deployment', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    args = parser.parse_args()
    guard(args.output)
    assert not args.output.exists()
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
    assert all(sha(BACKUP/name) == digest for name, digest in backup['source_sha256'].items())
    combined = json.loads(bind(args.combined).read_text())
    assert all(combined[k] for k in ('passed', 'source_unchanged', 'scope_complete',
                                    'baseline_file_test_counts_retained'))
    assert combined['returncode'] == 0 and combined['regression_scope'] == 'full_combined'
    assert not combined['external_boundary_attempts']
    assert combined['live_ros_contacted'] is False and combined['motion_commands_sent'] is False
    assert combined['baseline'] == str(BASE) and combined['baseline_sha256'] == BASE_SHA
    sources, scope = combined['source_sha256'], combined['test_scope']
    new_tests = sorted(str(p.relative_to(W)) for root in (
        W/'src/alicia_grasp_modes/tests', W/'src/alicia_flexible_grasp_supervisor/tests')
        for p in root.glob('test_gen36*.py'))
    assert set(scope) == set(baseline['test_scope']) | set(new_tests)
    assert set(sources) == set(baseline['source_sha256']) | set(new_tests)
    assert len(sources) == combined['source_count'] and len(scope) == combined['test_scope_files']
    assert all(sha(W/name) == digest for name, digest in sources.items())
    changed = {name for name, digest in baseline['source_sha256'].items()
               if sources[name] != digest}
    assert changed == {
        'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/gripper_geometry.py',
        'src/alicia_flexible_grasp_supervisor/tests/test_gen31_box_precision_budget.py',
        'src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py',
        'src/alicia_grasp_modes/scripts/mode_aware_grasp_task.py',
        'src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_task.py'}
    assert combined['summary']['tests'] > 7570
    assert all(combined['summary'][k] == 0 for k in ('failures', 'errors', 'skipped'))
    counts = combined['test_case_counts_by_file']
    assert set(counts) == set(scope) and sum(counts.values()) == combined['summary']['tests']
    assert all(counts[name] >= count for name, count in baseline['test_case_counts_by_file'].items())
    xml = bind(args.combined.parent/'pytest.xml', combined['xml_sha256'])
    actual = {k: sum(int(s.get(k, 0)) for s in ET.parse(xml).getroot().iter('testsuite'))
              for k in ('tests', 'failures', 'errors', 'skipped')}
    assert actual == combined['summary']
    bind(args.combined.parent/'pytest.log', combined['log_sha256'])
    bind(args.combined.parent/'scope.json')
    bind(args.combined.parent/'bootstrap/sitecustomize.py', combined['bootstrap_sha256'])
    bind(W/'.runtime/verify_gen36_offline_tests_20261016.py', combined['runner_sha256'])
    review = json.loads(bind(args.review).read_text())
    assert review['reviewed'] is True and review['release_review_complete'] is True
    assert not review['blocking_findings']
    required = changed | set(new_tests) | set(HELPERS)
    assert required <= set(review['reviewed_source_sha256'])
    for name, digest in review['reviewed_source_sha256'].items():
        assert not Path(name).is_absolute() and '..' not in Path(name).parts
        bind(W/name, digest)
    for path, digest in review['evidence_sha256'].items():
        bind(path, digest)
    assert review['evidence_sha256'].get(str(args.combined.resolve())) == sha(args.combined)
    previous = json.loads(bind(args.previous_deployment).read_text())
    assert previous['deployment_passed'] and previous['whole_patch_deployed']
    assert previous['source_manifest_sha256'] == BASE_SHA
    assert all(sha(path) == digest for path, digest in evidence.items())
    assert all(sha(W/name) == digest for name, digest in sources.items())
    release = dict(schema_version=1, kind='gen36_single_count_aperture_and_arrival_source_retention',
        sealed_ns=time.time_ns(), passed=True, reviewed=True, installed=True,
        installed_semantics='Exact local sources passed offline regression/review; runtime restart pending.',
        source_sha256=sources, source_count=len(sources), test_scope=scope,
        test_scope_files=len(scope), test_summary=combined['summary'], test_case_counts_by_file=counts,
        regression_scope='full_combined', baseline_release=str(BASE), baseline_release_sha256=BASE_SHA,
        actual_regression=str(args.combined.resolve()), combined_review=str(args.review.resolve()),
        evidence_sha256=evidence, previous_deployment=str(args.previous_deployment.resolve()),
        baseline_evidence_role='GEN35 historical baseline remains referenced by its unchanged manifest, not relabelled as GEN36 validation.',
        runtime_deployment_pending=True, physical_grasp_verified=False,
        motion_commands_sent=False, temperature_queried=False)
    write(args.output, release)
    print(json.dumps(dict(release=str(args.output), sha256=sha(args.output), summary=combined['summary'])))


if __name__ == '__main__':
    main()
