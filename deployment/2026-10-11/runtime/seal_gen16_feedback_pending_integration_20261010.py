"""Bind frozen, tested GEN16 source to its independent review; no ROS calls."""
import argparse
import collections
import hashlib
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

WORK = Path(__file__).resolve().parents[1]
NEW = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z')
OUT = NEW / 'verification/gen16_initial_feedback_span_pending'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def artifact(path):
    return {'path': str(path), 'sha256': sha(path)}


def source_map(delivery):
    if 'source_sha256' in delivery:
        return dict(delivery['source_sha256'])
    if isinstance(delivery.get('files'), dict):
        return dict(delivery['files'])
    return {row['path']: row['sha256']
            for row in delivery['files'] + delivery.get('test_files', [])}


def testcase_results(xml, scope):
    modules = collections.defaultdict(set)
    for item in scope:
        filename = item.split('::', 1)[0]
        modules[Path(filename).stem].add(filename)
    results = {}
    for case in ET.parse(xml).getroot().findall('.//testcase'):
        parts = case.attrib['classname'].split('.')
        matches = [(i, modules[part]) for i, part in enumerate(parts) if part in modules]
        assert len(matches) == 1 and len(matches[0][1]) == 1
        i, paths = matches[0]
        suffix = ('::' + '::'.join(parts[i+1:])) if len(parts) > i+1 else ''
        node = next(iter(paths)) + suffix + '::' + case.attrib['name']
        assert node not in results
        results[node] = ('error' if case.find('error') is not None else
                         'failed' if case.find('failure') is not None else
                         'skipped' if case.find('skipped') is not None else 'passed')
    return results


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--delivery', type=Path, required=True)
    parser.add_argument('--review', type=Path, required=True)
    parser.add_argument('--suite-manifest', type=Path, action='append', required=True)
    args = parser.parse_args()
    assert Path('/media/zhuyupei/SanStand').is_mount()
    delivery, review = read(args.delivery), read(args.review)
    assert delivery['delivery_ready'] is True and delivery['all_delivery_files_frozen'] is True
    assert review['reviewed'] is True and not review['blocking_findings']
    changed = source_map(delivery)
    for path, digest in changed.items():
        assert sha(WORK/path) == digest, ('delivery_drift', path)
        assert review['source_sha256'].get(path) == digest, ('review_binding', path)

    tested, merged, suites = {}, {}, []
    replaced_results, revised_tests = [], []
    for path in args.suite_manifest:
        manifest = read(path)
        assert manifest['source_unchanged'] is True
        assert path.name.endswith('_manifest.json')
        xml = path.with_name(path.name[:-len('_manifest.json')] + '.xml')
        results = testcase_results(xml, manifest['test_scope'])
        for filename, digest in manifest['source_sha256'].items():
            if filename in tested and tested[filename] != digest:
                # One reviewed assertion extension may be covered by a later
                # exact-node rerun; never permit this for production or data.
                assert '/tests/' in filename and filename.endswith('.py'), ('production_drift', filename)
                repeated = [node for node in results if node in merged and node.startswith(filename+'::')]
                assert repeated, ('changed_test_not_rerun', filename)
                revised_tests.append({'path': filename, 'previous_sha256': tested[filename],
                                      'final_sha256': digest, 'rerun_nodeids': repeated})
            tested[filename] = digest
        replaced_results.extend({'nodeid': node, 'previous': merged[node], 'final': value}
                                for node, value in results.items() if node in merged)
        merged.update(results)
        log = xml.with_suffix('.log')
        suites.append({'manifest': artifact(path), 'xml': artifact(xml),
                       'log': artifact(log), 'results': dict(collections.Counter(results.values()))})
    counts = collections.Counter(merged.values())
    assert counts['failed'] == 0 and counts['error'] == 0 and counts['passed'] > 0
    assert counts['passed'] == delivery['tests']['unique_passed'] == 300
    assert len(replaced_results) == len(revised_tests) == 1
    assert delivery['tests']['post_combined_test_edit']
    assert sha(delivery['report']['path']) == delivery['report']['sha256']
    for filename, digest in tested.items():
        assert sha(WORK/filename) == digest, ('final_test_source_drift', filename)
    baseline_path = NEW/'gen15_thin_gap_combined_deployment/result.json'
    baseline = read(baseline_path)
    assert baseline['deployment_passed'] is True and len(baseline['source_sha256']) == 192
    sources = dict(baseline['source_sha256'])
    sources.update(changed)
    sources.update(tested)
    for path, digest in sources.items():
        assert sha(WORK/path) == digest, ('unaccounted_drift', path)
    assert not any('/gui/' in p.lower() or 'widget' in p.lower() for p in sources)
    assert not any(p.startswith('docs/operations/') for p in sources)
    verification = {'unique_passed': counts['passed'], 'unique_failed': 0,
                    'unique_skipped': counts['skipped'], 'suite_artifacts': suites,
                    'scope': 'Actual current targeted suites only; previous 351 tests are not rerun or added.',
                    'deduplication': 'Exact manifest-bound test file, class and full parametrized test name.',
                    'replaced_same_node_results': replaced_results,
                    'reviewed_test_assertion_updates': revised_tests}
    evidence = [artifact(NEW/'gen16_analysis'/name) for name in (
        'accepted_feedback_bracket_timeline.json', 'accepted_feedback_interval_distribution.json')]
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
        'verification_scope': verification, 'gui_loaded_by_this_change': False,
        'deployment_order': ['three compute nodes'],
        'gateway_reload_required_by_this_change': False,
    })
    result = save('integration_result.json', {
        'passed': True, 'reviewed': True, 'recorded_ns': time.time_ns(),
        'offline_only': True, 'deployment_performed': False,
        'prepared_deployment_manifest': manifest,
        'source_count': len(sources), 'source_sha256': sources, 'changed_source_sha256': changed,
        'delivery': artifact(args.delivery), 'independent_review': artifact(args.review),
        'implementation_verification': delivery['report'],
        'unchanged_limits': delivery['unchanged_limits'],
        'tests': verification, 'unique_test_ids': sorted(merged),
        'previous_deployment': artifact(baseline_path), 'gen16_recorded_evidence': evidence,
        'limitations': [
            'Offline verification does not demonstrate real grasp success or deployment.',
            'No ROS calls, hardware commands, temperature access or source mutation occur here.',
            'The prior unrelated GEN14 assertion failure remains historical evidence, not a passing test.',
            'Recorder callback receipt timestamps do not prove producer subscriber callback ordering.',
        ],
    })
    print(json.dumps({'deployment_sources': manifest, 'integration_result': result,
                      'source_count': len(sources), 'unique_passed': counts['passed'],
                      'unique_skipped': counts['skipped']}, ensure_ascii=False))


if __name__ == '__main__':
    main()
