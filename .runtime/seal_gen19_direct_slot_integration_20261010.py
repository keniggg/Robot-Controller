"""Seal the reviewed direct-slot patch and actual tests; do not deploy."""
import argparse
from collections import Counter
import json
from pathlib import Path
import runpy
import time

WORK = Path(__file__).resolve().parents[1]
NEW = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z')
OUT = NEW/'verification/gen19_direct_slot_retry'
H = runpy.run_path(str(WORK/'.runtime/seal_gen16_feedback_pending_integration_20261010.py'), run_name='helpers_only')
sha, read, artifact = (H[k] for k in ('sha', 'read', 'artifact'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--delivery', type=Path, required=True)
    ap.add_argument('--review', type=Path, required=True)
    ap.add_argument('--suite-manifest', type=Path, action='append', required=True)
    ap.add_argument('--expected-passed', type=int, required=True)
    args = ap.parse_args()
    assert Path('/media/zhuyupei/SanStand').is_mount()
    delivery, review = read(args.delivery), read(args.review)
    assert delivery.get('delivery_ready', delivery.get('ready')) is True
    assert review['reviewed'] is True and review['blocking_findings'] == []
    changed = H['source_map'](delivery)
    for f, digest in changed.items():
        assert sha(WORK/f) == digest, ('delivery_drift', f)
        assert review['source_sha256'][f] == digest, ('review_drift', f)
    merged, tested, suites = {}, {}, []
    for manifest_path in args.suite_manifest:
        suite = read(manifest_path)
        assert suite['source_unchanged'] is True
        assert manifest_path.name.endswith('_manifest.json')
        xml = manifest_path.with_name(manifest_path.name[:-len('_manifest.json')]+'.xml')
        scope = suite.get('test_scope') or [arg for arg in suite['command']
            if arg.startswith('src/') and '.py' in arg and (WORK/arg.split('::')[0]).is_file()]
        assert scope
        cases = H['testcase_results'](xml, scope)
        assert not any(value in ('failed', 'error') for value in cases.values())
        for f, digest in suite['source_sha256'].items():
            assert sha(WORK/f) == digest, ('tested_drift', f)
            assert f not in tested or tested[f] == digest, ('conflicting_test_source', f)
            tested[f] = digest
        merged.update(cases)
        suites.append(dict(manifest=artifact(manifest_path), xml=artifact(xml),
            log=artifact(xml.with_suffix('.log')), results=dict(Counter(cases.values()))))
    counts = Counter(merged.values())
    assert counts['passed'] == args.expected_passed and counts['failed'] == counts['error'] == 0
    for f, digest in changed.items():
        assert tested.get(f) == digest, ('changed_source_not_test_bound', f)
    baseline_path = NEW/'gen18_arrival_feedback_gap_compute_deployment/result.json'
    baseline = read(baseline_path)
    assert baseline['deployment_passed'] is True and len(baseline['source_sha256']) == 202
    sources = dict(baseline['source_sha256'])
    sources.update(changed)
    sources.update(tested)
    for f, digest in sources.items():
        assert sha(WORK/f) == digest, ('unaccounted_drift', f)
    assert not any('/gui/' in f.lower() or 'widget' in f.lower() or f.startswith('docs/operations/') for f in sources)
    verification = dict(unique_passed=counts['passed'], unique_skipped=counts['skipped'], unique_failed=0,
        suites=suites, unique_test_ids=sorted(merged),
        scope='Only current frozen-patch lifecycle and necessary regression results. Earlier 334 are not accumulated.')
    OUT.mkdir(parents=True, exist_ok=True)
    def save(name, value):
        path = OUT/name
        path.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')
        return artifact(path)
    manifest = save('deployment_sources.json', dict(passed=True, reviewed=True, deployment_performed=False,
        source_count=len(sources), source_sha256=sources,
        previous_deployment=str(baseline_path), previous_deployment_sha256=sha(baseline_path),
        delivery=artifact(args.delivery), review_evidence=artifact(args.review), verification_scope=verification,
        gui_loaded_by_this_change=False, gateway_reload_required_by_this_change=False,
        deployment_order=['three compute nodes']))
    result = save('integration_result.json', dict(passed=True, reviewed=True, offline_only=True,
        recorded_ns=time.time_ns(), deployment_performed=False, prepared_deployment_manifest=manifest,
        source_count=len(sources), source_sha256=sources, changed_source_sha256=changed,
        delivery=artifact(args.delivery), independent_review=artifact(args.review), tests=verification,
        previous_deployment=artifact(baseline_path),
        recorded_gen19_evidence=artifact(NEW/'gen19_analysis/direct_slot_stall_recorded_evidence.json'),
        limitations=['Offline lifecycle tests do not prove a completed grasp.',
            'Fresh source identity, epoch, pose, phase deadline and original geometry/authority gates remain required.',
            'No ROS call, hardware operation, temperature access or deployment occurs in this sealing step.']))
    print(json.dumps(dict(deployment_sources=manifest, integration_result=result,
        source_count=len(sources), unique_passed=counts['passed'], unique_skipped=counts['skipped']), ensure_ascii=False))


if __name__ == '__main__':
    main()
