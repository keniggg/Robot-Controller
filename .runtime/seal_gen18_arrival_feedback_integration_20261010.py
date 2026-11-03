"""Bind the final arrival-feedback patch to its tests and independent review."""
import argparse
import collections
import json
from pathlib import Path
import runpy
import time

WORK = Path(__file__).resolve().parents[1]
NEW = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z')
OUT = NEW/'verification/gen18_arrival_feedback_gap_combined'
HELPERS = runpy.run_path(str(WORK/'.runtime/seal_gen16_feedback_pending_integration_20261010.py'),
                         run_name='offline_helpers_only')
sha, read, artifact = (HELPERS[key] for key in ('sha', 'read', 'artifact'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--delivery', type=Path, required=True)
    ap.add_argument('--review', type=Path, required=True)
    ap.add_argument('--suite-manifest', type=Path, required=True)
    ap.add_argument('--expected-passed', type=int, required=True)
    args = ap.parse_args()
    assert Path('/media/zhuyupei/SanStand').is_mount()
    delivery, review = read(args.delivery), read(args.review)
    assert delivery['delivery_ready'] is True and delivery['all_delivery_files_frozen'] is True
    assert review['reviewed'] is True and not review['blocking_findings']
    changed = HELPERS['source_map'](delivery)
    for path, digest in changed.items():
        assert sha(WORK/path) == digest, ('delivery_drift', path)
        assert review['source_sha256'].get(path) == digest, ('review_binding', path)

    suite = read(args.suite_manifest)
    assert suite['source_unchanged'] is True
    tested = dict(suite['source_sha256'])
    for path, digest in tested.items():
        assert sha(WORK/path) == digest, ('tested_source_drift', path)
    assert args.suite_manifest.name.endswith('_manifest.json')
    xml = args.suite_manifest.with_name(args.suite_manifest.name[:-len('_manifest.json')]+'.xml')
    cases = HELPERS['testcase_results'](xml, suite['test_scope'])
    counts = collections.Counter(cases.values())
    assert counts['passed'] == args.expected_passed and counts['failed'] == counts['error'] == 0

    baseline_manifest_path = NEW/'verification/gen17_feedback_ingress_lock/deployment_sources.json'
    baseline_path = NEW/'gen17_feedback_ingress_compute_deployment/result.json'
    baseline_manifest, baseline = read(baseline_manifest_path), read(baseline_path)
    assert baseline['deployment_passed'] is True
    assert baseline['source_sha256'] == baseline_manifest['source_sha256']
    assert len(baseline['source_sha256']) == 198
    sources = dict(baseline['source_sha256'])
    sources.update(changed)
    sources.update(tested)
    for path, digest in sources.items():
        assert sha(WORK/path) == digest, ('unaccounted_drift', path)
    assert not any('/gui/' in p.lower() or 'widget' in p.lower() for p in sources)
    assert not any(p.startswith('docs/operations/') for p in sources)
    verification = {
        'unique_passed': counts['passed'], 'unique_failed': 0, 'unique_skipped': counts['skipped'],
        'test_scope': suite['test_scope'], 'unique_test_ids': sorted(cases),
        'manifest': artifact(args.suite_manifest), 'xml': artifact(xml), 'log': artifact(xml.with_suffix('.log')),
        'scope': 'Final current patch relevant suite only; historical 547 and earlier targeted runs are not accumulated.',
    }
    evidence = [artifact(NEW/'gen18_analysis'/name) for name in (
        'arrival_feedback_bracket_first_failure.json', 'arrival_contract_bag_evidence.json',
        'arrival_gap_recovery_contract_review.json')]
    OUT.mkdir(exist_ok=True, parents=True)
    def save(name, data):
        p = OUT/name
        p.write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n')
        return artifact(p)
    manifest = save('deployment_sources.json', {
        'passed': True, 'reviewed': True, 'deployment_performed': False,
        'source_count': len(sources), 'source_sha256': sources,
        'previous_deployment': str(baseline_path), 'previous_deployment_sha256': sha(baseline_path),
        'previous_verified_manifest': artifact(baseline_manifest_path),
        'delivery': artifact(args.delivery), 'review_evidence': artifact(args.review),
        'verification_scope': verification,
        'gui_loaded_by_this_change': False, 'deployment_order': ['three compute nodes'],
        'gateway_reload_required_by_this_change': False,
    })
    result = save('integration_result.json', {
        'passed': True, 'reviewed': True, 'recorded_ns': time.time_ns(),
        'offline_only': True, 'deployment_performed': False,
        'prepared_deployment_manifest': manifest, 'source_count': len(sources), 'source_sha256': sources,
        'changed_source_sha256': changed, 'delivery': artifact(args.delivery),
        'independent_review': artifact(args.review), 'tests': verification,
        'previous_deployment': artifact(baseline_path), 'recorded_gen18_evidence': evidence,
        'limitations': [
            'Only an existing registered arrival-station identity contract can retain identity while geometry is unavailable.',
            'Original 200 ms feedback interpolation and 0.8 s current measurement gates are not relaxed.',
            'Offline tests and recorded timing do not prove completed full perception recovery or a physical grasp.',
            'No hardware, ROS calls, temperature access, source changes or deployment occur in this integration step.',
        ],
    })
    print(json.dumps({'deployment_sources': manifest, 'integration_result': result,
                      'source_count': len(sources), 'unique_passed': counts['passed'],
                      'unique_skipped': counts['skipped']}, ensure_ascii=False))


if __name__ == '__main__':
    main()
