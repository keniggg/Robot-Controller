"""Write GEN27 final review only from completed source review and exact passing runs."""
import argparse
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

from verify_gen27_combined_20261015 import (
    W, N, BASE, BASE_SHA, SPEC, sha, read, guard, save, inventory, xml_summary)

CHECKLIST = W / '.runtime/gen27_independent_review_checklist.json'
INDEPENDENT_TEST = 'src/alicia_grasp_modes/tests/test_gen27_feedback_gap_completion_independent.py'
RED = N / 'verification/gen27_feedback_completion_independent_red/receipt.json'
EXCLUDED_RED_CASE = 'test_completion_cannot_admit_late_or_changed_current_measurement[late_entry]'
REQUIRED_CHECKS = (
    'minimal_approved_patch', 'one_source_completion_identity',
    'notice_received_before_original_deadline', 'fixed_ros_and_monotonic_deadlines',
    'fresh_measured_feedback_and_stationary_pose', 'consumer_orders_and_poll_boundary',
    'duplicate_missing_late_notice_and_source_switch_rejected',
    'manual_epoch_selection_phase_change_revokes', 'no_pending_geometry_or_motion_authority',
    'unchanged_existing_observation_routes', 'complete_scope_and_runtime_provenance',
    'red_float_boundary_exclusion_recorded',
)
RUNTIME = (
    'reload_gen20_compute_gateway_20261014.py',
    'start_observation_measurement_consumers_20261008.sh',
    'observation_measurement_consumers_20261008.launch',
    'prepare_gen20_continuation_receipt_20261014.py',
    'prepare_gen26_continuation_receipt_20261015.py',
    'resume_gen26_control_20261015.py',
    'continuous_grasp_authorization_20261012.py',
    'run_reboot_realigned_grasp_20261010.py',
    'capture_continuation_scene_20261012.py',
    'read_gen7_yield_readiness_20261012.py',
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--combined', type=Path, required=True)
    parser.add_argument('--focused', type=Path, action='append', required=True,
                        help='Final exact-source passing focused receipt; repeat as needed.')
    parser.add_argument('--checklist', type=Path, default=CHECKLIST)
    parser.add_argument('--checklist-sha256', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    assert output.parent.parent == N / 'verification'
    assert not output.exists()
    # Reject unfinished code review before any output directory or file is made.
    checklist_path = args.checklist.resolve(strict=True)
    assert checklist_path == CHECKLIST
    assert sha(checklist_path) == args.checklist_sha256
    checklist = read(checklist_path)
    assert checklist['code_review_completed'] is True
    assert not checklist['blocking_findings']
    assert all(checklist['checks'].get(name) is True for name in REQUIRED_CHECKS)
    assert checklist['review_notes'] and checklist['resolved_findings']
    assert checklist['evidence_qualifications']
    guard(output.parent)

    spec_sha = sha(SPEC)
    assert checklist['spec_sha256'] == spec_sha
    spec = read(SPEC)
    sources, scope = inventory(spec)
    base = read(BASE)
    modified = spec['production'] + spec['changed_tests'] + spec['new_tests'] + spec['new_resources']
    required_review = set(modified) | {
        '.runtime/gen27_verification_spec.json',
        '.runtime/verify_gen27_combined_20261015.py',
        '.runtime/seal_gen27_release_20261015.py',
        '.runtime/write_gen27_final_independent_review_20261015.py',
        '.runtime/run_gen27_independent_feedback_review.py',
        '.runtime/analyze_gen26_recorded_feedback_gap.py',
    }
    reviewed = checklist['reviewed_source_sha256']
    assert required_review <= set(reviewed)
    for name, digest in reviewed.items():
        assert not Path(name).is_absolute() and '..' not in Path(name).parts
        assert sha(W / name) == digest, name
        if name in sources:
            assert sources[name] == digest

    evidence = {}

    def bind(path, expected=None):
        path = Path(path).resolve(strict=True)
        digest = sha(path)
        assert expected is None or expected == digest, str(path)
        assert str(path) not in evidence or evidence[str(path)] == digest
        evidence[str(path)] = digest
        return path

    bind(BASE, BASE_SHA)
    bind(SPEC, spec_sha)
    bind(checklist_path, args.checklist_sha256)
    for path, digest in spec['evidence_sha256'].items():
        bind(path, digest)
    for path, digest in checklist.get('evidence_sha256', {}).items():
        bind(path, digest)
    combined_path = bind(args.combined)
    assert N / 'verification' in combined_path.parents
    combined = read(combined_path)
    assert combined['passed'] is True and combined['returncode'] == 0
    assert combined['source_unchanged'] is True and combined['scope_complete'] is True
    assert combined['regression_scope'] == 'full_combined'
    assert combined['production_worktree_tested'] is True and combined['live_ros_contacted'] is False
    assert combined['motion_commands_sent'] is False and combined['temperature_queried'] is False
    assert combined['baseline'] == str(BASE) and combined['baseline_sha256'] == BASE_SHA
    assert combined['source_sha256'] == sources and combined['test_scope'] == scope
    assert combined['source_count'] == len(sources) and combined['test_scope_files'] == len(scope)
    summary = combined['summary']
    assert summary['tests'] >= base['test_summary']['tests']
    assert all(summary[key] == 0 for key in ('failures', 'errors', 'skipped'))
    patch = read(bind(combined['patch'], combined['patch_sha256']))
    assert patch['spec'] == spec and patch['spec_sha256'] == spec_sha
    assert Path(patch['spec_path']).resolve() == SPEC
    assert set(patch['files']) == set(modified)
    for name, row in patch['files'].items():
        assert row == dict(baseline_sha256=base['source_sha256'].get(name), sha256=sources[name])
    bind(combined_path.parent / 'pytest.log', combined['log_sha256'])
    xml = bind(combined_path.parent / 'pytest.xml', combined['xml_sha256'])
    assert xml_summary(xml) == summary
    counts = dict.fromkeys(scope, 0)
    for case in ET.parse(xml).getroot().iter('testcase'):
        assert case.get('file') in counts
        counts[case.get('file')] += 1
    assert counts == combined['test_case_counts_by_file']
    assert all(value > 0 for value in counts.values()) and sum(counts.values()) == summary['tests']
    assert not combined['unassigned_testcases']
    assert reviewed['.runtime/verify_gen27_combined_20261015.py'] == combined['runner_sha256']

    def receipt(path):
        path = bind(path)
        assert N / 'verification' in path.parents
        data = read(path)
        bind(path.parent / 'pytest.log')
        test_xml = bind(path.parent / 'pytest.xml')
        assert xml_summary(test_xml) == data['summary']
        return path, data, test_xml

    _, red, red_xml = receipt(RED)
    assert red['returncode'] != 0 and red['source_unchanged'] is True
    assert red['summary'] == dict(tests=17, failures=9, errors=0, skipped=0)
    for name, digest in red['source_sha256'].items():
        if name != INDEPENDENT_TEST:
            assert base['source_sha256'][name] == digest, name
    red_failures = [case.get('name') for case in ET.parse(red_xml).getroot().iter('testcase')
                    if case.find('failure') is not None]
    assert len(red_failures) == 9 and red_failures.count(EXCLUDED_RED_CASE) == 1
    focused = {}
    focused_sources = set()
    focused_tests = set()
    for path in args.focused:
        path, data, _ = receipt(path)
        assert path != RED
        assert data['returncode'] == 0 and data['source_unchanged'] is True
        assert data['summary']['tests'] > 0
        assert all(data['summary'][key] == 0 for key in ('failures', 'errors', 'skipped'))
        assert data['live_ros_contacted'] is False and data['motion_commands_sent'] is False
        assert data['temperature_queried'] is False
        for name, digest in data['source_sha256'].items():
            assert not Path(name).is_absolute() and '..' not in Path(name).parts
            assert sha(W / name) == digest, name
            focused_sources.add(name)
        focused_tests.update(name for name in data['command'] if name in scope)
        focused[str(path)] = dict(summary=data['summary'], source_unchanged=True)
    assert set(spec['production'] + spec['new_resources'] + spec['new_tests']) <= focused_sources
    assert set(spec['new_tests']) <= focused_tests
    assert INDEPENDENT_TEST in focused_sources and INDEPENDENT_TEST in focused_tests

    # Keep unsuccessful and superseded completed runs as history, not as passes.
    focused_history = {}
    for path in sorted((N / 'verification').glob('gen27*/receipt.json')):
        data = read(path)
        if not {'summary', 'source_unchanged', 'source_sha256'} <= set(data):
            continue
        path, data, _ = receipt(path)
        focused_history[path.parent.name] = dict(summary=data['summary'],
            returncode=data['returncode'], source_unchanged=data['source_unchanged'])
    combined_history = {}
    for path in sorted((N / 'verification').glob('gen27_combined_*/result.json')):
        bind(path)
        data = read(path)
        for name, key in (('patch_manifest.json', 'patch_sha256'),
                          ('pytest.log', 'log_sha256'), ('pytest.xml', 'xml_sha256')):
            bind(path.parent / name, data[key])
        combined_history[path.parent.name] = dict(passed=data['passed'], summary=data['summary'],
            source_unchanged=data['source_unchanged'], scope_complete=data['scope_complete'])
    for name in RUNTIME:
        path = (W / '.runtime' / name).resolve(strict=True)
        bind(path, base['evidence_sha256'][str(path)])
    for name, digest in reviewed.items():
        bind(W / name, digest)

    qualifications = list(checklist['evidence_qualifications']) + [
        'Original independent red was 17 cases: 9 failures, 8 passes. Eight failures exposed genuine producer/consumer rejection paths. The late_entry +1ns floating-point fixture rounded at the boundary; it was corrected to +0.801s and is excluded from causal production-failure count.',
        'Red test version differs from final expanded tests; its original source receipt, log and XML remain immutable. Directory names such as green do not prove passing results; actual counts above are authoritative.',
        'First-admission red_r2 has three failures but only the remote and collector failures establish causal original-clock gaps. Its producer fixture replaced the monotonic clock during scene setup, so that failure is diagnostic only. The initial first_admission_red also had a missing thread_time fixture error. independent_final had one failure from the same producer clock-domain setup; final_r2 separates the clock proxy after acquisition and validates the actual fixed producer reference budget. These historical runs are not reported as passing or as three causal production gaps.',
        'Focused counts overlap the full suite and are not summed. All 226 GEN26 files and at least 6993 baseline cases are retained in the complete run.',
        'Recorded timing uses failure_status_build_ns: old-reference age 0.826196909s, recovery-source age 0.171872139s. This later status build must not be conflated with the earlier internal failure time.',
        'Static 16-second prefix evidence shows 107 accepted samples with no joint change and no active task or arm/hand trajectory goal. It is a bounded observation, not a proof about every instant outside that window.',
    ]
    review = dict(schema_version=1, kind='gen27_feedback_gap_timely_frame_completion_review',
        reviewed=True, reviewed_ns=time.time_ns(), blocking_findings=[],
        reviewed_source_sha256=reviewed, evidence_sha256=evidence,
        verdict='Exact reviewed GEN27 patch passed all focused checks and the inherited complete offline scope; new deployment and physical execution remain separate evidence.',
        review_method='Validation-agent independent source and provenance review recorded in the SHA-pinned checklist, plus independent adversarial tests and a complete exact-source regression.',
        review_checklist=str(checklist_path), code_review_notes=checklist['review_notes'],
        reviewed_checks=checklist['checks'], resolved_findings=checklist['resolved_findings'],
        validation=dict(combined=str(combined_path), summary=summary,
            source_count=len(sources), test_scope_files=len(scope), source_unchanged=True,
            scope_complete=True, all_cases_single_full_run=True, focused=focused,
            focused_history=focused_history, combined_history=combined_history,
            causal_red_failure_count=8, excluded_red_cases=[EXCLUDED_RED_CASE],
            focused_counts_not_summed=True),
        evidence_qualifications=qualifications,
        source_delta={name:dict(baseline_sha256=base['source_sha256'].get(name), sha256=sources[name]) for name in modified},
        prior_attempt_record=spec['prior_attempt_record'],
        prior_attempt_bag_metadata=spec['prior_attempt_bag_metadata'],
        prior_attempt_evidence_note=spec['prior_attempt_evidence_note'],
        limitations=['Offline tests establish software contracts, not real-world clearance or physical grip.',
                    'GEN26 first attempt ended before motion; this repair does not change that historical result.',
                    'No GEN27 deployment or successful grasp is claimed. A no-lift close without confirmed holding remains unknown.'],
        remaining=['Seal and deploy the exact reviewed source while idle.',
                   'Capture fresh scene/readiness under active continuing authorization.',
                   'Record actual execution and holding result without inferring success from command completion.'],
        runtime_deployment_pending=True, physical_grasp_verified=False,
        physical_grasp_success=False, live_ros_contacted=False, motion_commands_sent=False,
        temperature_queried=False)
    assert sha(SPEC) == spec_sha and sha(checklist_path) == args.checklist_sha256
    assert all(sha(W / name) == digest for name, digest in sources.items())
    assert all(sha(path) == digest for path, digest in evidence.items())
    output.parent.mkdir(exist_ok=True)
    save(output, review)
    print(json.dumps(dict(review=str(output), sha256=sha(output), summary=summary,
                          reviewed_source_files=len(reviewed), evidence_files=len(evidence))))


if __name__ == '__main__':
    main()
