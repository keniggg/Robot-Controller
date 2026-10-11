"""Seal GEN18 same-station registration reacquisition fix for compute-only reload; no ROS."""
import argparse
from collections import Counter
import json
from pathlib import Path
import runpy
import time

WORK = Path(__file__).resolve().parents[1]
NEW = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_no_skin_20261011T000625Z')
H = runpy.run_path(str(WORK/'.runtime/seal_gen16_feedback_pending_integration_20261010.py'), run_name='helpers_only')
sha, read, artifact = (H[k] for k in ('sha', 'read', 'artifact'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--delivery', type=Path, action='append', required=True)
    ap.add_argument('--review', type=Path, action='append', required=True)
    ap.add_argument('--suite-manifest', type=Path, action='append', required=True)
    ap.add_argument('--previous-deployment', type=Path, required=True)
    ap.add_argument('--expected-passed', type=int, required=True)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    assert Path('/media/zhuyupei/SanStand').is_mount()
    assert args.output.resolve().is_relative_to(NEW.resolve()) if hasattr(Path, 'is_relative_to') else str(args.output.resolve()).startswith(str(NEW.resolve())+'/')
    changed, reviewed, tested, merged, suites = {}, {}, {}, {}, []
    for path in args.delivery:
        delivery = read(path)
        assert delivery.get('delivery_ready', delivery.get('ready')) is True
        delivery_sources = H['source_map'](delivery)
        if isinstance(delivery.get('test_files'), dict):
            delivery_sources.update(delivery['test_files'])
        for filename, digest in delivery_sources.items():
            assert sha(WORK/filename) == digest, ('delivery_drift', filename)
            assert filename not in changed or changed[filename] == digest
            changed[filename] = digest
    production = {name for name in changed if '/tests/' not in name}
    assert production == {
        'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py',
        'src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_remote.py',
    }, ('unexpected_production_scope', production)
    for path in args.review:
        review = read(path)
        assert review['reviewed'] is True and review['blocking_findings'] == []
        for filename, digest in review['source_sha256'].items():
            assert sha(WORK/filename) == digest, ('review_drift', filename)
            assert filename not in reviewed or reviewed[filename] == digest
            reviewed[filename] = digest
    for filename, digest in changed.items():
        assert reviewed.get(filename) == digest, ('unreviewed_change', filename)
    for path in args.suite_manifest:
        suite = read(path)
        assert suite['source_unchanged'] is True
        assert path.name.endswith('_manifest.json')
        xml = path.with_name(path.name[:-len('_manifest.json')]+'.xml')
        scope = suite.get('test_scope') or [arg for arg in suite['command']
            if arg.startswith('src/') and '.py' in arg and (WORK/arg.split('::')[0]).is_file()]
        assert scope
        cases = H['testcase_results'](xml, scope)
        assert cases and not any(v in ('failed', 'error') for v in cases.values())
        for filename, digest in suite['source_sha256'].items():
            assert sha(WORK/filename) == digest, ('tested_drift', filename)
            assert filename not in tested or tested[filename] == digest
            tested[filename] = digest
        merged.update(cases)
        suites.append(dict(manifest=artifact(path), xml=artifact(xml),
            log=artifact(xml.with_suffix('.log')), results=dict(Counter(cases.values()))))
    counts = Counter(merged.values())
    assert counts['passed'] == args.expected_passed > 0
    assert counts['failed'] == counts['error'] == 0
    for filename, digest in changed.items():
        assert tested.get(filename) == digest, ('changed_source_not_test_bound', filename)
    baseline_path = args.previous_deployment
    baseline = read(baseline_path)
    assert baseline['deployment_passed'] is True
    sources = dict(baseline['source_sha256'])
    sources.update(changed)
    sources.update(tested)
    for filename, digest in sources.items():
        assert sha(WORK/filename) == digest, ('unaccounted_drift', filename)
    verification = dict(unique_passed=counts['passed'], unique_skipped=counts['skipped'],
        unique_failed=0, suites=suites, unique_test_ids=sorted(merged),
        scope='Only final follow-up tests; prior suite totals are not accumulated.')
    args.output.mkdir(parents=True, exist_ok=False)
    def save(name, value):
        path = args.output/name
        path.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')
        return artifact(path)
    manifest = save('deployment_sources.json', dict(passed=True, reviewed=True,
        deployment_performed=False, source_count=len(sources), source_sha256=sources,
        previous_deployment=str(baseline_path), previous_deployment_sha256=sha(baseline_path),
        delivery=[artifact(p) for p in args.delivery], review_evidence=[artifact(p) for p in args.review],
        verification_scope=verification, gui_loaded_by_this_change=False,
        gateway_reload_required_by_this_change=False,
        deployment_order=['three compute nodes']))
    result = save('integration_result.json', dict(passed=True, reviewed=True, offline_only=True,
        recorded_ns=time.time_ns(), deployment_performed=False,
        prepared_deployment_manifest=manifest, source_count=len(sources), source_sha256=sources,
        changed_source_sha256=changed, tests=verification,
        delivery=[artifact(p) for p in args.delivery], independent_reviews=[artifact(p) for p in args.review],
        previous_deployment=artifact(baseline_path),
        limitations=['Offline tests do not establish a real grasp or deployment.',
                    'Current measured start, complete path and arrival checks remain required.']))
    print(json.dumps(dict(deployment_sources=manifest, integration_result=result,
        source_count=len(sources), unique_passed=counts['passed'], unique_skipped=counts['skipped']), ensure_ascii=False))


if __name__ == '__main__':
    main()
