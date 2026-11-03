"""Materialize the completed GEN26 source/evidence review only after full regression passes."""
import argparse
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

from verify_gen26_combined_20261015 import W, N, BASE, BASE_SHA, sha, read, guard, save, inventory, xml_summary

SPEC = W / '.runtime/gen26_verification_spec.json'
SPEC_SHA = '1ab05b09308ebb10ed6d1ff71d8bd68d5d191a06bab4a27bc27fa75a2737f998'
DIRECT_TEST = 'src/alicia_flexible_grasp_supervisor/tests/test_gen26_direct_lifetime_downstream_independent.py'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--combined', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    assert output.parent.parent == N / 'verification'
    guard(output.parent)
    assert not output.exists()
    assert sha(SPEC) == SPEC_SHA
    spec = read(SPEC)
    sources, scope = inventory(spec)
    assert len(sources) == 960 and len(scope) == 226
    combined_path = args.combined.resolve(strict=True)
    assert N / 'verification' in combined_path.parents
    combined = read(combined_path)
    assert combined['passed'] is True and combined['returncode'] == 0
    assert combined['source_unchanged'] is True and combined['scope_complete'] is True
    assert combined['regression_scope'] == 'full_combined'
    assert combined['production_worktree_tested'] is True and combined['live_ros_contacted'] is False
    assert combined['motion_commands_sent'] is False and combined['temperature_queried'] is False
    assert combined['baseline'] == str(BASE) and combined['baseline_sha256'] == BASE_SHA
    assert combined['source_sha256'] == sources and combined['test_scope'] == scope
    assert combined['source_count'] == 960 and combined['test_scope_files'] == 226
    summary = combined['summary']
    assert summary['tests'] >= read(BASE)['test_summary']['tests']
    assert all(summary[key] == 0 for key in ('failures', 'errors', 'skipped'))

    evidence = {}

    def bind(path, expected=None):
        path = Path(path).resolve(strict=True)
        digest = sha(path)
        assert expected is None or digest == expected, str(path)
        assert str(path) not in evidence or evidence[str(path)] == digest
        evidence[str(path)] = digest
        return path

    bind(BASE, BASE_SHA)
    bind(SPEC, SPEC_SHA)
    bind(combined_path)
    patch_path = bind(combined['patch'], combined['patch_sha256'])
    patch = read(patch_path)
    assert patch['spec'] == spec and patch['spec_sha256'] == SPEC_SHA
    assert Path(patch['spec_path']).resolve() == SPEC
    modified = spec['production'] + spec['changed_tests'] + spec['new_tests'] + spec['new_resources']
    assert set(patch['files']) == set(modified)
    assert all(row['sha256'] == sources[name] for name, row in patch['files'].items())
    bind(combined_path.parent / 'pytest.log', combined['log_sha256'])
    xml_path = bind(combined_path.parent / 'pytest.xml', combined['xml_sha256'])
    assert xml_summary(xml_path) == summary
    counts = dict.fromkeys(scope, 0)
    for case in ET.parse(xml_path).getroot().iter('testcase'):
        assert case.get('file') in counts
        counts[case.get('file')] += 1
    assert counts == combined['test_case_counts_by_file']
    assert all(value > 0 for value in counts.values()) and sum(counts.values()) == summary['tests']
    assert not combined['unassigned_testcases']

    checkpoint_path = bind(N / 'gen26_user_pause_checkpoint/checkpoint.json')
    checkpoint = read(checkpoint_path)
    assert checkpoint['source_deployed'] == 'GEN25' and checkpoint['pending_repair'] == 'GEN26'
    for path, digest in checkpoint['completed_evidence'].items():
        bind(path, digest)
    focused = {}
    for directory in sorted((N / 'verification').glob('gen26*')):
        receipt = directory / 'receipt.json'
        if not receipt.is_file():
            continue
        bind(receipt)
        data = read(receipt)
        for name in ('pytest.log', 'pytest.xml'):
            if (directory / name).is_file():
                bind(directory / name)
        focused[directory.name] = dict(returncode=data.get('returncode'),
            summary=data.get('summary', xml_summary(directory / 'pytest.xml')
                             if (directory / 'pytest.xml').is_file() else None),
            source_unchanged=data.get('source_unchanged'))
    combined_history = {}
    for path in sorted((N / 'verification').glob('gen26_combined_*/result.json')):
        bind(path)
        data = read(path)
        for name, key in (('patch_manifest.json', 'patch_sha256'),
                          ('pytest.log', 'log_sha256'), ('pytest.xml', 'xml_sha256')):
            bind(path.parent / name, data[key])
        combined_history[path.parent.name] = dict(passed=data['passed'],
            summary=data['summary'], source_unchanged=data['source_unchanged'],
            scope_complete=data['scope_complete'], spec_sha256=read(data['patch'])['spec_sha256'])
    for name, count in (('gen26_registered_inference_independent_green_r1', 8),
                        ('gen26_direct_lifetime_scope_final', 47)):
        data = read(N / 'verification' / name / 'receipt.json')
        assert data['returncode'] == 0 and data['source_unchanged'] is True
        assert data['summary'] == dict(tests=count, failures=0, errors=0, skipped=0)
        assert all(sha(W / path) == digest for path, digest in data['source_sha256'].items())
    broad = read(N / 'verification/gen26_direct_lifetime_downstream_green_final/receipt.json')
    assert broad['returncode'] == 0 and broad['source_unchanged'] is True
    assert broad['summary'] == dict(tests=420, failures=0, errors=0, skipped=0)
    assert all(sha(W / path) == digest for path, digest in broad['source_sha256'].items()
               if path != DIRECT_TEST)
    for name in ('source_manifest.json', 'qualified_initial_model_v1.json'):
        bind(N / 'verification/gen26_box_inference_design' / name)
    bind(N / 'gen25_deployment/result.json')

    runtime = ['.runtime/gen26_verification_spec.json',
        '.runtime/verify_gen26_combined_20261015.py', '.runtime/seal_gen26_release_20261015.py',
        '.runtime/prepare_gen26_continuation_receipt_20261015.py', '.runtime/resume_gen26_control_20261015.py',
        '.runtime/prepare_gen20_continuation_receipt_20261014.py',
        '.runtime/reload_gen20_compute_gateway_20261014.py',
        '.runtime/start_observation_measurement_consumers_20261008.sh',
        '.runtime/observation_measurement_consumers_20261008.launch',
        '.runtime/continuous_grasp_authorization_20261012.py',
        '.runtime/run_reboot_realigned_grasp_20261010.py',
        '.runtime/capture_continuation_scene_20261012.py', '.runtime/read_gen7_yield_readiness_20261012.py',
        '.runtime/run_gen26_registered_review.py', '.runtime/run_gen26_direct_lifetime_review.py',
        '.runtime/write_gen26_final_independent_review_20261015.py']
    reviewed_paths = sorted(set(modified + runtime + ['src/alicia_grasp_modes/scripts/mode_aware_grasp_task.py']))
    reviewed_sources = {name: sha(W / name) for name in reviewed_paths}
    for name in runtime:
        bind(W / name)
    assert reviewed_sources['.runtime/verify_gen26_combined_20261015.py'] == combined['runner_sha256']
    assert set(checkpoint['source_sha256']) <= set(reviewed_sources)
    pause_delta = {name: dict(paused_sha256=digest, reviewed_sha256=reviewed_sources[name])
                  for name, digest in checkpoint['source_sha256'].items()
                  if digest != reviewed_sources[name]}

    review = dict(schema_version=1, kind='gen26_box_inference_and_complete_direct_task_review',
        reviewed=True, reviewed_ns=time.time_ns(), blocking_findings=[],
        reviewed_source_sha256=reviewed_sources, evidence_sha256=evidence,
        verdict='Exact current GEN26 source has passed the expanded full offline scope; runtime deployment and physical grasp remain separate evidence.',
        review_method='Coordinator review plus independent registration and direct-lifetime adversarial reviews; validation agent separately inspected both fixes, provenance, scope, runtime helpers and final artifacts.',
        validation=dict(combined=str(combined_path), summary=summary, source_count=len(sources),
            test_scope_files=len(scope), source_unchanged=True, scope_complete=True,
            all_cases_single_full_run=True, focused_summaries=focused,
            combined_history=combined_history,
            expanded_baseline_scope='All 216 GEN25 test files retained, nine new GEN26 test files added, and the previously hashed-but-not-executed test_grasp_task_sequence.py fully executed.',
            focused_counts_not_summed=True),
        resolved_findings=[
            dict(id='GEN26_REGISTERED_SUPPORT_FRAME', status='resolved',
                problem='Accepted registration can leave its reference/model plane different from the current source plane; comparing them as identical rejected valid inferred contact downstream.',
                fix='Model projection uses its registered plane; final contact evidence and physical clearance remain bound to immutable source support. Source-to-registered transform, contact projection and uncertainty enclosure checks remain.',
                evidence='Independent original red 3/3 failed; expanded green 8/8 passed, including source-normal/offset contradictions.'),
            dict(id='GEN26_DIRECT_TASK_LIFETIME_DOWNSTREAM', status='resolved',
                problem='The direct-task lease originally protected nominal pregrasp but later local segments could create new 60-second episodes and task-side close lacked the same fixed lifetime.',
                fix='Capture original direct CONTACT task start under start/plan locks. First active publication uses that exact event stamp; later states cannot renew it. Validate fixed ROS/monotonic deadlines and immutable selection/model/config/epoch before motion and close. Manual/epoch changes revoke. Gateway local contexts retain the original lease and persistent required-budget flag. Matching observed-session origin keeps its established route; other generations cannot hide direct expiry.',
                evidence='Original red had 20 failures: 18 causal gaps and 2 pre-existing manual refusals with different diagnostics. Final same-production broad scope passed 420; expanded downstream file passed 47 and is included in the full combined run.'),
            dict(id='GEN26_OLD_SEQUENCE_FIXTURE', status='resolved_test_fixture',
                problem='Previously unexecuted sequence coverage changed a diagnostic after plan identity was built and omitted the real frozen execution authority; hardlink fixture also required inode support unavailable on SanStand.',
                fix='Recompute identity after the diagnostic, freeze through the production API, stub only unrelated target drift. Use explicit /dev/shm RAM for synthetic inode fixtures; all verification logs/results stay on SanStand.',
                evidence='Mixed runs preserved; entire corrected test file is now in the complete scope.'),
            dict(id='GEN26_REMOTE_PARAMETER_FIXTURE', status='resolved_test_fixture',
                problem='Four legacy remote preparation fixtures did not stub the newly read Boolean contact-policy ROS parameter; the isolated run reached the intentionally unreachable test master instead of its intended assertion.',
                fix='Supply the actual parameter default at those fixture boundaries while retaining production inference and original assertions; rerun the complete remote file and GEN26 pipeline, then the full combined scope.',
                evidence='Combined r1 preserved as 6989 pass/4 fail, zero errors/skips; only its exact tested patch and failure output are evidence for that run.')],
        reviewed_behavior=[
            'Boolean unknown_require_measured_bilateral_contact=false enables typed inference; true restores measured bilateral evidence. Frozen/live mismatch or invalid parameter type prevents submission.',
            'Only complete qualified regular-box tops produce an explicit orthogonal-prism prior. Source depth/color/mask/intrinsics/transform/geometry hashes remain bound; no side samples are fabricated.',
            'Opening uses nominal projected width plus full empirical position/yaw/height uncertainty and fit clearance. Collision enclosure also contains the inferred body and uncertainty.',
            'Near the physical aperture limit, precision-view evidence is tied to the exact model, target and source and identifies narrow-face edges/corners. It is not an execution permit.',
            'ROS transport, final contact decoding, frozen task validation and simulated/real fixed-close share one plan and nominal inferred contact width; inferred upper opening is not claimed as measured contact.',
            'GEN25 long nominal trajectory timing remains within its original task/session; local computation/compensation cap, physical width, speed limits, collision and real feedback checks remain.'],
        evidence_qualifications=[
            'Directory labels containing green or red are historical names; receipt/XML counts above are authoritative. direct green_r1 had 34 pass/4 fail from boundary-fixture float-to-nanosecond rounding; green_r2 had 411 pass/2 fixture failures. Neither is reported as passing.',
            'The original direct red_r1 receipt mistakenly inventoried registration production files rather than affected direct files. It is retained as observed failure evidence, not a complete direct-source-pinned release proof. Earlier pause hash audit and final exact-source combined verification are separate evidence.',
            'The 420-case direct green used the same final production bytes but preceded two added scope cases in its new test file; 47-case final focused and the full combined result cover that final test version.',
            'Full combined r1 was not a pass: 6993 cases with four missing parameter-stub failures. The final full result above is separate and must pass all 226 files without exclusions.',
            'Focused counts overlap each other and the full suite; they are never added to claim a larger independent total.'],
        source_changes_after_pause=pause_delta,
        runtime_review=dict(
            reload='Existing reload validates eleven prior identities, replaces three compute consumers and lets the preserved core respawn its gateway; seven hardware/GUI processes remain. No duplicate gateway or driver rebuild.',
            configuration='Explicitly set only /grasp/unknown_require_measured_bilateral_contact to Boolean false while idle; verify existing 600s session, max_observations=2, no-lift and other operational settings. Do not load whole YAML (its max_observations is 3).',
            continuation='New helper uses current GEN22 storage scope, original authorization/hands hashes, actual clearance args, operator-supplied image assessment, fresh same-epoch/selection readiness and control locking. Resume records exact user request before paused23->active24 and only allows reviewed checkpoint changes.',
            readiness='Derive SDK joint reference from the fresh capture, then use read_gen7 --compare; no fabricated alignment/user receipt. Refresh after deployment and before grasp.',
            old_helper_unchanged=True),
        limitations=[
            'Regular-box geometry is an explicitly qualified modeling assumption, and empirical uncertainty is not a certified worst-case error bound.',
            'The original recorded initial frame can infer the box but its depth-edge uncertainty still requires a targeted precision view; this review does not guarantee a direct grasp from every view.',
            'Offline planner/controller/service doubles establish software boundaries, not physical collision clearance, execution success or actual bilateral holding.',
            'No new GEN26 physical attempt has been verified. With lift disabled and tactile evidence unavailable, closed gripper may remain HOLDING_UNVERIFIED.'],
        remaining=['Seal and idle-deploy these exact reviewed bytes.',
                   'Obtain fresh scene/readiness and follow existing recorded continuation guard.',
                   'Record actual execution/holding result without treating command completion as physical success.'],
        runtime_deployment_pending=True, physical_grasp_verified=False,
        physical_grasp_success=False, live_ros_contacted=False, motion_commands_sent=False,
        temperature_queried=False)
    # Prevent a review from certifying bytes changed while its evidence was read.
    assert sha(SPEC) == SPEC_SHA
    assert all(sha(W / name) == digest for name, digest in sources.items())
    assert all(sha(W / name) == digest for name, digest in reviewed_sources.items())
    assert all(sha(path) == digest for path, digest in evidence.items())
    output.parent.mkdir(exist_ok=True)
    save(output, review)
    print(json.dumps(dict(review=str(output), sha256=sha(output), summary=summary,
                          reviewed_source_files=len(reviewed_sources), evidence_files=len(evidence))))


if __name__ == '__main__':
    main()
