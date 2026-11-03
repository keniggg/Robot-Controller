#!/usr/bin/env python3
"""Summarize completed GEN14 offline verification without running tests or ROS."""
import argparse
import collections
import hashlib
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

WORK = Path(__file__).resolve().parents[1]
NEW = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z')
ANALYSIS = NEW / 'gen14_analysis'
OUT = NEW / 'verification/gen14_contact_station_and_held_precheck'
HELD_TEST_FILES = [
    'test_gen14_held_approach_precheck.py', 'test_pregrasp_compensation.py',
    'test_pregrasp_feedforward.py', 'test_held_approach_replay.py',
    'test_held_approach_gateway.py', 'test_contact_sequence_precheck.py',
    'test_contact_sample_collection.py', 'test_contact_precheck_phase_cancellation.py',
    'test_held_approach_task.py',
]
RUNTIME = [
    '.runtime/observation_measurement_consumers_20261008.launch',
    '.runtime/reload_idle_contact_gateway_gen14_20261010.py',
    '.runtime/run_reboot_realigned_grasp_20261010.py',
    'tools/record_grasp_keyframes_readonly.py',
]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def artifact(path):
    return {'path': str(path), 'sha256': sha(path)}


def write(path, content):
    path.write_text(json.dumps(content, ensure_ascii=False, indent=2) + '\n')
    return artifact(path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--suite', default='combined_r3')
    ap.add_argument('--delivery', type=Path, default=WORK / '.runtime/gen14_contact_station_retirement_20261010/delivery.json')
    ap.add_argument('--supplement-suite', action='append', default=[])
    ap.add_argument('--additional-review', type=Path, action='append', default=[])
    ap.add_argument('--replay-review', type=Path)
    ap.add_argument('--baseline-failure-report', type=Path)
    ap.add_argument('--replay', type=Path, default=ANALYSIS / 'contact_station_retirement/recorded_continuous_frame_replay.json')
    ap.add_argument('--seal', action='store_true', help='Require complete final evidence before writing deployment_sources.json')
    args = ap.parse_args()
    assert Path('/media/zhuyupei/SanStand').is_mount(), 'external storage unavailable'
    OUT.mkdir(exist_ok=True, parents=True)
    baseline_path = NEW / 'support_offset_policy_deployment/result.json'
    baseline = read(baseline_path)
    assert baseline['deployment_passed'] is True
    delivery_path = args.delivery
    delivery = read(delivery_path)
    assert delivery['delivery_ready'] and delivery['all_delivery_files_frozen']
    held_path = ANALYSIS / 'held_precheck_delivery.json'
    held = read(held_path)
    selected = {entry['path']: entry['sha256'] for entry in delivery['files'] + delivery['test_files']}
    selected.update(held['source_sha256'])
    for path, digest in selected.items():
        assert sha(WORK / path) == digest, ('delivery_source_drift', path)
    sources = dict(baseline['source_sha256'])
    sources.update(selected)
    sources.update({path: sha(WORK / path) for path in RUNTIME})
    for path, digest in sources.items():
        assert sha(WORK / path) == digest, ('unaccounted_source_drift', path)
    assert not any('/gui/' in p.lower() or 'widget' in p.lower() for p in sources)

    review_paths = [ANALYSIS / 'contact_station_independent_review.json',
                    ANALYSIS / 'held_precheck_independent_review.json'] + args.additional_review
    reviewed_sources = {}
    reviews = []
    for path in review_paths:
        review = read(path)
        assert review['reviewed'] is True and not review['blocking_findings'], path
        reviewed_sources.update(review['source_sha256'])
        reviews.append(artifact(path))
    uncovered = [path for path, digest in selected.items() if reviewed_sources.get(path) != digest]

    suite_dir = ANALYSIS / 'contact_station_retirement'
    cases = {}
    suite_manifest = {}
    handoff_evidence = []
    case_provenance = collections.defaultdict(list)
    for name in [args.suite] + args.supplement_suite:
        xml_path = suite_dir / (name + '.xml')
        suite_manifest_path = suite_dir / (name + '_manifest.json')
        suite_manifest.update(read(suite_manifest_path))
        current_cases = {}
        for case in ET.parse(xml_path).getroot().findall('.//testcase'):
            identity = 'alicia_grasp_modes/' + case.attrib['classname'] + '::' + case.attrib['name']
            assert identity not in current_cases
            status = ('failed' if case.find('failure') is not None else
                      'error' if case.find('error') is not None else
                      'skipped' if case.find('skipped') is not None else 'passed')
            current_cases[identity] = status
            case_provenance[identity].append({'suite': name, 'status': status})
        cases.update(current_cases)
        handoff_evidence.append({'name': name, 'counts': dict(collections.Counter(current_cases.values())),
                                'xml': artifact(xml_path), 'file_manifest': artifact(suite_manifest_path)})
    for path, digest in suite_manifest.items():
        assert sha(WORK / path) == digest, ('tested_source_drift', path)
    counts = dict(collections.Counter(cases.values()))
    known_failure = None
    if set(counts) != {'passed'}:
        assert args.baseline_failure_report is not None, counts
        known_failure = read(args.baseline_failure_report)
        assert known_failure['baseline_failure_confirmed'] is True
        assert known_failure['same_nodeid_and_exact_failure_message'] is True
        relative, case_name = known_failure['nodeid'].split('::', 1)
        assert relative.startswith('src/alicia_grasp_modes/')
        classname = relative[len('src/alicia_grasp_modes/'):-3].replace('/', '.')
        known_identity = 'alicia_grasp_modes/' + classname + '::' + case_name
        assert {key: value for key, value in cases.items() if value != 'passed'} == {known_identity: 'failed'}
        assert any(item['xml']['sha256'] == known_failure['current_suite']['sha256'] for item in handoff_evidence)
        assert sha(known_failure['unchanged_test']['path']) == known_failure['unchanged_test']['sha256']
    assert held['passed'] == 284 and held['failed'] == 0
    assert sha(held['test_log']) == held['test_log_sha256']
    assert '284 passed' in Path(held['test_log']).read_text()
    held_scope = ['src/alicia_flexible_grasp_supervisor/tests/' + p for p in HELD_TEST_FILES]
    assert not set(held_scope) & set(suite_manifest)
    replay = read(args.replay)
    for path, digest in replay.get('source_sha256', {}).items():
        assert sha(WORK / path) == digest, ('replay_source_drift', path)
    all_replayed_frames_ready = replay.get('all_replayed_fresh_frames_ready') is True
    replay_passed = False
    replay_review_evidence = None
    if args.replay_review is not None:
        replay_review = read(args.replay_review)
        assert replay_review['reviewed'] is True and not replay_review['blocking_findings']
        assert replay_review['replay']['sha256'] == sha(args.replay)
        assert Path(replay_review['replay']['path']).resolve() == args.replay.resolve()
        replay_passed = replay_review['replay_passed'] is True
        replay_review_evidence = artifact(args.replay_review)
    final_ready = not uncovered and replay_passed
    report = {
        'passed': final_ready, 'reviewed': not uncovered,
        'validation_passed': final_ready,
        'deployment_performed': False, 'recorded_ns': time.time_ns(),
        'offline_only': True,
        'state': 'ready_for_parent_deployment' if final_ready else 'pending_continuous_tracking_resolution',
        'source_count': len(sources), 'source_sha256': sources,
        'deliveries': [artifact(delivery_path), artifact(held_path)],
        'review_evidence': reviews, 'unreviewed_final_source_paths': uncovered,
        'previous_deployment': artifact(baseline_path),
        'tests': {
            'handoff': {'counts': counts, 'suites': handoff_evidence},
            'held_precheck': {'passed': 284, 'failed': 0, 'log': artifact(held['test_log']), 'file_scope': held_scope},
            'unique_passed': counts['passed'] + 284,
            'unique_failed': counts.get('failed', 0),
            'known_preexisting_failed': counts.get('failed', 0),
            'all_collected_cases_passed': not counts.get('failed', 0),
            'baseline_failure_comparison': artifact(args.baseline_failure_report) if known_failure else None,
            'baseline_failure_note': known_failure['conclusion'] if known_failure else None,
            'overlap': 0,
            'deduplication_basis': 'The handoff and held-precheck file scopes belong to different packages and are disjoint; repeated prior runs are excluded.',
        },
        'handoff_case_provenance': dict(case_provenance),
        'continuous_replay': artifact(args.replay),
        'continuous_replay_all_ready': all_replayed_frames_ready,
        'continuous_replay_reviewed_passed': replay_passed,
        'continuous_replay_review': replay_review_evidence,
        'replay_acceptance_note': 'READY on every frame is not an absolute admission rule: expected close-contact visibility loss is assessed within the existing frozen contact execution scope. The separate exact-artifact review must distinguish that condition from artificial ROI failure, support/identity contradiction and fabricated freshness.',
        'runtime_paths': RUNTIME,
        'deployment_order': ['three compute nodes', 'motion gateway'],
        'gui_loaded_by_this_change': False,
        'limitations': [
            'Targeted completed offline suites, not every repository test.',
            'No new real-arm action, deployment, or physical grasp success by this integration script.',
            'Offline replay uses recorded RGB-D, TF and accepted joint samples; it does not reproduce full runtime load or all original callback scheduling.',
            'Replay admission does not establish continuous visual tracking, prove the cause of every unavailable measurement, or waive the existing frozen-contact execution checks.',
            'GUI source and reload are excluded.',
        ],
    }
    if not args.seal:
        print(json.dumps(write(OUT / 'integration_pending.json', report), ensure_ascii=False))
        return
    assert final_ready, 'Final source review and explicit exact-artifact replay review are required'
    manifest = write(OUT / 'deployment_sources.json', {
        'passed': True, 'reviewed': True, 'source_sha256': sources,
        'source_count': len(sources), 'review_evidence': reviews,
        'previous_deployment': str(baseline_path), 'previous_deployment_sha256': sha(baseline_path),
        'deployment_performed': False, 'gui_loaded_by_this_change': False,
        'verification_scope': report['tests'], 'continuous_replay': artifact(args.replay),
        'continuous_replay_review': replay_review_evidence,
        'runtime_paths': RUNTIME, 'deployment_order': report['deployment_order'],
    })
    report['prepared_deployment_manifest'] = manifest
    print(json.dumps({'integration_result': write(OUT / 'integration_result.json', report),
                      'deployment_sources': manifest}, ensure_ascii=False))


if __name__ == '__main__':
    main()
