"""Seal frozen feedback-ingress fix evidence without ROS or device access."""
import argparse
import ast
import collections
import hashlib
import json
from pathlib import Path
import runpy
import time

WORK = Path(__file__).resolve().parents[1]
NEW = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z')
OUT = NEW/'verification/gen17_feedback_ingress_lock'
RUNNER = '.runtime/run_reboot_realigned_grasp_20261010.py'
RUNNER_APPROVED_SHA = '00dbc88804cd4877e962363a8114e37e6d795f327ad48354407b3b6f13db2e9b'
READY_TEXT = '盒子已在画面中央，可以继续'
HELPERS = runpy.run_path(str(WORK/'.runtime/seal_gen16_feedback_pending_integration_20261010.py'),
                         run_name='offline_seal_helpers_only')
sha, read, artifact = (HELPERS[key] for key in ('sha', 'read', 'artifact'))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--delivery', type=Path, required=True)
    parser.add_argument('--review', type=Path, required=True)
    parser.add_argument('--suite-manifest', type=Path, required=True, action='append')
    parser.add_argument('--expected-passed', type=int, required=True)
    args = parser.parse_args()
    assert Path('/media/zhuyupei/SanStand').is_mount()
    delivery, review = read(args.delivery), read(args.review)
    assert delivery['delivery_ready'] is True and delivery['all_delivery_files_frozen'] is True
    assert review['reviewed'] is True and not review['blocking_findings']
    assert sha(delivery['report']['path']) == delivery['report']['sha256']
    changed = HELPERS['source_map'](delivery)
    for path, digest in changed.items():
        assert sha(WORK/path) == digest, ('delivery_drift', path)
        assert review['source_sha256'].get(path) == digest, ('review_binding', path)

    tested, merged, suites = {}, {}, []
    for path in args.suite_manifest:
        manifest = read(path)
        assert manifest['source_unchanged'] is True
        for filename, digest in manifest['source_sha256'].items():
            assert sha(WORK/filename) == digest, ('tested_source_drift', filename)
            assert filename not in tested or tested[filename] == digest, ('inconsistent_suites', filename)
            tested[filename] = digest
        assert path.name.endswith('_manifest.json')
        xml = path.with_name(path.name[:-len('_manifest.json')]+'.xml')
        cases = HELPERS['testcase_results'](xml, manifest['test_scope'])
        merged.update(cases)
        suites.append({'manifest': artifact(path), 'xml': artifact(xml),
                       'log': artifact(xml.with_suffix('.log')),
                       'results': dict(collections.Counter(cases.values()))})
    counts = collections.Counter(merged.values())
    assert counts['passed'] == args.expected_passed and counts['failed'] == counts['error'] == 0
    baseline_path = NEW/'gen16_feedback_gap_compute_deployment/result.json'
    baseline = read(baseline_path)
    assert baseline['deployment_passed'] is True and len(baseline['source_sha256']) == 196
    runner = WORK/RUNNER
    assert sha(runner) == RUNNER_APPROVED_SHA
    parsed = ast.parse(runner.read_text(), str(runner))
    assert any(isinstance(node, ast.Constant) and node.value == READY_TEXT for node in ast.walk(parsed))
    runner_note = {
        'path': RUNNER, 'previous_sha256': baseline['source_sha256'][RUNNER],
        'sha256': RUNNER_APPROVED_SHA, 'ready_text': READY_TEXT,
        'change': 'Add the actual user readiness message as one allowed literal.',
        'assessment_source': 'Parent /root explicitly confirmed only the single literal was appended; authorization, pose checks, parameters and actions are unchanged.',
        'independent_local_validation': 'AST syntax and exact readiness literal checked without executing the runner.',
    }
    sources = dict(baseline['source_sha256'])
    sources.update(changed)
    sources.update(tested)
    sources[RUNNER] = RUNNER_APPROVED_SHA
    for path, digest in sources.items():
        assert sha(WORK/path) == digest, ('unaccounted_drift', path)
    assert not any('/gui/' in p.lower() or 'widget' in p.lower() for p in sources)
    assert not any(p.startswith('docs/operations/') for p in sources)
    verification = {
        'unique_passed': counts['passed'], 'unique_failed': 0, 'unique_skipped': counts['skipped'],
        'suite_artifacts': suites, 'unique_test_ids': sorted(merged),
        'scope': 'Only new-fix relevant suites; previous 300 tests are not rerun or added.',
    }
    OUT.mkdir(exist_ok=True, parents=True)
    def save(name, data):
        path = OUT/name
        path.write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n')
        return artifact(path)
    manifest = save('deployment_sources.json', {
        'passed': True, 'reviewed': True, 'deployment_performed': False,
        'source_count': len(sources), 'source_sha256': sources,
        'previous_deployment': str(baseline_path), 'previous_deployment_sha256': sha(baseline_path),
        'delivery': artifact(args.delivery), 'review_evidence': artifact(args.review),
        'root_reviewed_runtime_change': runner_note, 'verification_scope': verification,
        'gui_loaded_by_this_change': False, 'deployment_order': ['three compute nodes'],
        'gateway_reload_required_by_this_change': False,
    })
    result = save('integration_result.json', {
        'passed': True, 'reviewed': True, 'recorded_ns': time.time_ns(),
        'offline_only': True, 'deployment_performed': False,
        'prepared_deployment_manifest': manifest, 'source_count': len(sources), 'source_sha256': sources,
        'changed_source_sha256': changed, 'root_reviewed_runtime_change': runner_note,
        'delivery': artifact(args.delivery), 'independent_review': artifact(args.review),
        'implementation_verification': delivery['report'],
        'tests': verification, 'previous_deployment': artifact(baseline_path),
        'gen17_recorded_evidence': artifact(NEW/'gen17_analysis/owner_invalidation_timeline.json'),
        'limitations': [
            'Offline concurrency verification is not proof of a successful physical grasp.',
            'Recorder receipt timestamps do not directly observe the remote callback cache.',
            'No source mutation, ROS calls, hardware commands, temperature access, deployment or GUI reload occurs here.',
            'Historical unrelated tests are not represented as newly rerun or newly passing.',
        ],
    })
    print(json.dumps({'deployment_sources': manifest, 'integration_result': result,
                      'source_count': len(sources), 'unique_passed': counts['passed'],
                      'unique_skipped': counts['skipped']}, ensure_ascii=False))


if __name__ == '__main__':
    main()
