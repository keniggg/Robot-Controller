"""Seal completed, independently reviewed GEN15 offline results; no ROS calls."""
import argparse
import collections
import hashlib
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

WORK = Path(__file__).resolve().parents[1]
NEW = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z')
OUT = NEW / 'verification/gen15_thin_contact_view_gap'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def artifact(path):
    return {'path': str(path), 'sha256': sha(path)}


def save(name, content):
    path = OUT / name
    path.write_text(json.dumps(content, indent=2, ensure_ascii=False) + '\n')
    return artifact(path)


def cases(path, test_scope):
    modules = collections.defaultdict(set)
    for nodeid in test_scope:
        filename = nodeid.split('::', 1)[0]
        modules[Path(filename).stem].add(filename)
    result = {}
    for case in ET.parse(path).getroot().findall('.//testcase'):
        parts = case.attrib['classname'].split('.')
        matches = [(i, modules[part]) for i, part in enumerate(parts) if part in modules]
        assert len(matches) == 1 and len(matches[0][1]) == 1
        index, paths = matches[0]
        filename = next(iter(paths))
        suffix = '::' + '::'.join(parts[index + 1:]) if len(parts) > index + 1 else ''
        key = filename + suffix + '::' + case.attrib['name']
        assert key not in result
        result[key] = ('error' if case.find('error') is not None else
                       'failed' if case.find('failure') is not None else
                       'skipped' if case.find('skipped') is not None else 'passed')
    return result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--delivery', type=Path, required=True)
    ap.add_argument('--review', type=Path, required=True)
    args = ap.parse_args()
    assert Path('/media/zhuyupei/SanStand').is_mount()
    delivery, review = read(args.delivery), read(args.review)
    assert delivery['delivery_ready'] is True and delivery['all_delivery_files_frozen'] is True
    assert review['reviewed'] is True and not review['blocking_findings']
    if 'source_sha256' in delivery:
        changed = dict(delivery['source_sha256'])
    elif isinstance(delivery['files'], dict):
        changed = dict(delivery['files'])
    else:
        changed = {entry['path']: entry['sha256'] for entry in delivery['files'] + delivery.get('test_files', [])}
    for path, digest in changed.items():
        assert sha(WORK / path) == digest, ('delivery_drift', path)
        assert review['source_sha256'].get(path) == digest, ('review_binding', path)

    suite_manifests = {name: read(OUT / (name + '_manifest.json'))
                       for name in ('combined_r1', 'backend_environment_r2')}
    all_tested = {}
    for name, manifest in suite_manifests.items():
        assert manifest['source_unchanged'] is True
        for path, digest in manifest['source_sha256'].items():
            assert sha(WORK / path) == digest, ('tested_source_drift', name, path)
            assert path not in all_tested or all_tested[path] == digest
            all_tested[path] = digest
    original = cases(OUT / 'combined_r1.xml', suite_manifests['combined_r1']['test_scope'])
    rerun = cases(OUT / 'backend_environment_r2.xml', suite_manifests['backend_environment_r2']['test_scope'])
    assert collections.Counter(original.values()) == {'passed': 350, 'error': 1}
    assert collections.Counter(rerun.values()) == {'passed': 1}
    assert set(rerun) == {key for key, value in original.items() if value != 'passed'}
    merged = {**original, **rerun}
    assert collections.Counter(merged.values()) == {'passed': 351}

    baseline_path = NEW / 'gen14_contact_combined_deployment/result.json'
    baseline = read(baseline_path)
    assert baseline['deployment_passed'] is True and len(baseline['source_sha256']) == 185
    sources = dict(baseline['source_sha256'])
    sources.update(changed)
    sources.update(all_tested)
    for path, digest in sources.items():
        assert sha(WORK / path) == digest, ('unaccounted_drift', path)
    assert not any('/gui/' in path.lower() or 'widget' in path.lower() for path in sources)
    assert not any(path.startswith('docs/operations/') for path in sources)
    suite_artifacts = {name: artifact(OUT / name) for name in (
        'combined_r1.xml', 'combined_r1.log', 'combined_r1_manifest.json',
        'backend_environment_r2.xml', 'backend_environment_r2.log', 'backend_environment_r2_manifest.json')}
    verification = {
        'unique_passed': 351, 'unique_failed': 0, 'unique_skipped': 0,
        'original_combined': {'passed': 350, 'fixture_setup_error': 1},
        'single_node_environment_rerun': {'passed': 1},
        'deduplication': 'Different pytest versions emit different classname prefixes. Both are bound to the same unique file in the recorded test scope, retaining the complete test name and parameters. The one rerun replaces that exact errored nodeid; no duplicate count or additional test execution.',
        'resolved_setup_nodeid': next(iter(rerun)),
        'environment_correction': suite_manifests['backend_environment_r2']['environment'],
        'preserved_original_setup_error': suite_artifacts['combined_r1.log'],
        'suite_artifacts': suite_artifacts,
    }
    manifest = save('deployment_sources.json', {
        'passed': True, 'reviewed': True, 'deployment_performed': False,
        'source_count': len(sources), 'source_sha256': sources,
        'previous_deployment': str(baseline_path), 'previous_deployment_sha256': sha(baseline_path),
        'delivery': artifact(args.delivery), 'review_evidence': artifact(args.review),
        'verification_scope': verification, 'gui_loaded_by_this_change': False,
        'deployment_order': ['three compute nodes', 'motion gateway'],
    })
    report = save('integration_result.json', {
        'passed': True, 'reviewed': True, 'recorded_ns': time.time_ns(),
        'offline_only': True, 'deployment_performed': False,
        'prepared_deployment_manifest': manifest,
        'source_count': len(sources), 'source_sha256': sources,
        'changed_source_sha256': changed,
        'delivery': artifact(args.delivery), 'independent_review': artifact(args.review),
        'tests': verification, 'unique_test_ids': sorted(merged),
        'previous_deployment': artifact(baseline_path),
        'gen15_first_blocker': artifact(NEW / 'gen15_analysis/first_planning_blocker.json'),
        'gen14_gen15_evidence_comparison': artifact(NEW / 'gen15_analysis/bilateral_evidence_vs_gen14.json'),
        'prior_unrelated_baseline_test_failure': artifact(NEW / 'gen14_analysis/contact_station_retirement/baseline_spatial_old_assertion/result.json'),
        'limitations': [
            'These are the actual targeted tests for this change, not a new run of the previous 1117 tests or the whole repository.',
            'The old unrelated assertion failure is retained as prior evidence; it was not rerun or recategorized as passing.',
            'The historical GEN15 audit lacks exact proposal-angle/side-count binding. Constructed callback fixtures and recorded scalar fields do not claim exact fused-cloud replay.',
            'This observation-only repair does not authorize a contact motion or establish real grasp success.',
            'No deployment, ROS, GUI, hardware, temperature or source mutation is performed by this script.',
        ],
    })
    print(json.dumps({'integration_result': report, 'deployment_sources': manifest,
                      'source_count': len(sources), 'unique_passed': 351}, ensure_ascii=False))


if __name__ == '__main__':
    main()
