"""Seal reviewed shared-reference classification against combined evidence."""
from datetime import datetime, timezone
from pathlib import Path
import hashlib
import json

WORK = Path(__file__).resolve().parents[1]
NEW = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z')
STAGE = WORK / '.runtime/gen10_shared_reference_staging_20261010'
assert Path('/media/zhuyupei/SanStand').is_mount()
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
read = lambda p: json.loads(p.read_text())
delivery = read(STAGE / 'delivery.json')
combined_path = NEW / 'verification/gen10_shared_reference_combined_final/result.json'
combined = read(combined_path)
assert combined['passed'] and combined['source_unchanged']
assert combined['baseline150']['matched_before'] and combined['baseline150']['matched_after']
expected = {
    'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py': 'a5be2369b1a1661e3bbe69f4d014d44f0ea737c7fd8cc06b9896708a936099af',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/silhouette_observation_planning.py': '2973d78a8b62c7fbefd82a94c59250e0ea2dd9d9c937a0979dccb47bf87b2966',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/grasp6d_pipeline.py': '2b2acd64d8cf18cbdeacb2ea6da08df65f91f1873c5c4cbef4cf466ee3b7ed98',
    'src/alicia_grasp_modes/tests/test_observation_shared_reference_failure.py': '895d514fabcd940e9885c9fce7e7024d23a3fd707b61f2b0bbd60706b10c8c06',
    'src/alicia_grasp_modes/tests/fixtures/gen10_shared_reference_failure.json': '2d5cf50b64ee427fe8372b0eee2e6b5453912267b7fb0d5a578a44733f28a52c',
}
assert expected == combined['source_sha256']
assert expected == {r['path']: r['sha256'] for r in delivery['files']}
assert all(sha(STAGE / p) == digest for p, digest in expected.items())
fixture = read(STAGE / 'src/alicia_grasp_modes/tests/fixtures/gen10_shared_reference_failure.json')
audit_path = Path(fixture['source_audit'])
assert sha(audit_path) == fixture['source_audit_sha256'] == audit_path.stem
actual_rows = read(audit_path)['observation_failure_diagnostics']['observation_orientation_search']['variant_results']
assert fixture['checked_results'] == [r for r in actual_rows if r['checked']]
assessment_path = NEW / 'gen10_handoff_analysis/strict_actuation_failure_assessment.json'
baseline_comparison_path = NEW / 'verification/gen10_baseline_variant_fixture/comparison.json'
assert sha(baseline_comparison_path) == '9f6b9d3e51d92ee557170d8fe5407bd681776a1d0ca48c833f9ce1da5b225d49'
report = dict(
    reviewer='/root/gen9_review', reviewed_at_utc=datetime.now(timezone.utc).isoformat(),
    baseline=150, stage=str(STAGE), passed=True,
    disposition='no_blocking_findings_in_frozen_patch', findings=[],
    scope='Shared observation driver-reference failure classification and first-failure termination; three production files, one regression file and exact six-row GEN10 recorded fixture.',
    files=[dict(path=p, sha256=digest) for p, digest in sorted(expected.items())],
    physical_actions=False, production_edited_by_reviewer=False, deployed_by_reviewer=False,
    evidence=[
        dict(kind='independent_recorded_cause_assessment', path=str(assessment_path), sha256=sha(assessment_path)),
        dict(kind='original_recorded_gate_audit', path=str(audit_path), sha256=sha(audit_path)),
        dict(kind='combined_regression', path=str(combined_path), sha256=sha(combined_path), tests=combined['unique_tests']),
        dict(kind='preexisting_baseline_test_failure_comparison', path=str(baseline_comparison_path), sha256=sha(baseline_comparison_path)),
    ],
    resolved_review_findings=[
        'Stopping the inner view loop alone could leave an earlier reachable outer-family selection. The patch now clears selected/reachable at the non-near-field acceptance boundary if any checked candidate reports the shared reference failure, while retaining checked evidence. A real bounded-selector regression covers prior success followed by common failure and a normal IK-failure control.',
        'The real bounded-selector regression found that the existing result validator dropped the new rejection label as an unknown generic failure. The rejection-only whitelist now accepts it when reachable is False, every hard state is None and no success evidence is present; original assertions were retained.',
    ],
    verified_invariants=[
        'Classification requires observation=True, the failed SetTargetPose response path and the exact leading OBSERVATION_PREVIEW_PATH_UNAVAILABLE: driver reference has no fresh actuation epoch: gateway contract. Generic IK rejection, other preview/FOV failure text, prefixed unrelated text and the contact service retain their old classification.',
        'The six fixture rows exactly match the original GEN10 audit including their reason strings. Classification preserves each original reason and does not claim IK, collision, joint-limit or planning success.',
        'The pipeline whitelist change only preserves a typed generic rejection through result validation. Success still requires the original STRICT_SERVICE_SUCCESS contract or consistent complete hard-state booleans; mixed/partial evidence and a success carrying this failure label remain invalid.',
        'After current-ticket validation and strict result commit, the first shared reference failure returns immediately from the shared view loop. No remaining variant consumes another strict check, and unchecked endpoints are not declared unreachable.',
        'An earlier same-candidate incumbent is cleared. The outer comparison continuation also stops on the typed reason; any earlier outer-family selected/reachable result is retired before the common non-near-field acceptance path can promote it.',
        'Both ordinary measured-observation and silhouette independent-observation acceptance preserve OBSERVATION_REFERENCE_UNAVAILABLE in terminal diagnostics and the gate audit. Existing runner handling accepts these invalid near_field_terminal messages under the same source/selection/freshness checks.',
        'Private stop state contains only strings and booleans; public failure audit is built from scalar counters, original row dictionaries and existing dataclass serialization. No new non-serializable object or execution authority is introduced.',
        'Ordinary IK failures continue searching and may retain an earlier qualified candidate. Near-field/direct contact acceptance does not call the new selection retirement helper. Source, epoch, feedback freshness, strict path, following, collision and actuation-confirmation gates are unchanged.',
        'No driver, gateway, positive-enable implementation, ROS wire protocol, timeout, candidate generator, FK solver or physical confirmation behavior is changed by this patch. This patch identifies and stops a shared infrastructure failure; it does not itself repair the actuator state.',
    ],
    validation_limits=[
        'The reviewer inspected the exact production patch, fixture and test code and independently compared fixture rows with the recorded audit; implementation/combined agents executed the pytest suite.',
        'The ordinary/silhouette regression invokes real Mode acceptance, adapter, shared loop, audit and terminal publication using controlled service responses and established test source context; it does not run a physical planner or move the robot.',
        'Outer-family regression invokes the real bounded selector, checker/continuation and retirement helper. The helper integration into the full base acceptance entry was statically verified; that regression does not independently rerun the entire base geometry-to-promotion chain.',
        'The initial expanded suite had 27 failures in the 29-case old observation_variant_local_reuse file because its prepared fixture lacks a ticket for the existing arrival-projection validation. Baseline150 independently reproduces all 27 identical test names and exception evidence, with the other two passing. That unchanged file is excluded from the final focused suite, and its failures remain documented rather than being claimed fixed.',
        'Fresh positive-enable, new control epoch, fresh source evidence and real measured directional confirmation remain runtime requirements. No recovered confirmation, reachable endpoint, completed observation or physical grasp is claimed by these tests.',
    ])
output = NEW / 'verification/gen10_shared_reference_independent_review.json'
assert not output.exists()
output.write_text(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + '\n')
print(json.dumps(dict(path=str(output), sha256=sha(output), disposition=report['disposition'],
    files=len(expected), combined_tests=combined['unique_tests']), ensure_ascii=False))
