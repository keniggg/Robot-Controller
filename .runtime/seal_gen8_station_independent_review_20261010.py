"""Seal the independently reviewed frozen GEN8 patch and external evidence."""
from datetime import datetime, timezone
from pathlib import Path
import hashlib
import json

WORK = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
STAGE = WORK / '.runtime/gen8_station_compute_staging_20261010'
NEW = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z')
EXPECTED = {
    'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py': '24f9453c7bc85b89267d9ed3ebbfb81eb29af8744e0e3f8c7189ca486c32cb0f',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_remote.py': 'a75f0e6e1f93b15be980590a9d11f40d06732497e8dfbb3db5b425b35b509008',
    'src/alicia_grasp_modes/tests/test_station_compute_work_handoff.py': 'e9fb357508da0b022afebdbb0169f8cd94968157498234396ef1ad6aeb3d7074',
    'src/alicia_grasp_modes/tests/test_gen8_recorded_station_admission.py': '58cbb355ad649510bf5b2be559886106a650d9945b2fbd4348caefb1fa68e59c',
    'src/alicia_grasp_modes/tests/fixtures/gen8_recorded_station_admission.json': '6a434361e2258001ecb1335284e9c7c3e8e267aa06cd8788231dfb46286ebeb2',
}

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

delivery = json.loads((STAGE / 'delivery.json').read_text())
assert {r['path']: r['sha256'] for r in delivery['files']} == EXPECTED
assert all(sha(STAGE / path) == digest for path, digest in EXPECTED.items())
combined_path = NEW / 'verification/gen8_station_admission_combined/result.json'
combined = json.loads(combined_path.read_text())
assert combined.get('passed') is True
assert combined['source_sha256'] == EXPECTED
assert combined['source_unchanged'] is True
targeted_path = NEW / 'verification/gen8_station_compute_work/result.json'
assert sha(targeted_path) == '2cac8dd78c799f0240727ebcff00215aa333e511a3730bdb2881075a30fc96ac'
recorded_log = NEW / 'verification/gen8_recorded_station_admission/attempt4.log'
assert '8 passed' in recorded_log.read_text()
cutoff_path = NEW / 'verification/gen8_station_compute_cutoff_regression/result.json'
assert sha(cutoff_path) == '2ad0f6a222e9f6dced3be15802622b80c2a87de0974a10616eb84312cb1f05b8'
report = dict(
    disposition='no_blocking_findings_in_frozen_patch', passed=True,
    reviewer='/root/support_residual_audit', reviewed_at_utc=datetime.now(timezone.utc).isoformat(),
    stage=str(STAGE), baseline=144,
    files=[dict(path=p, sha256=s) for p, s in sorted(EXPECTED.items())],
    scope='GEN8 station computation admission and publication lifecycle; two production files, two regression files and original-wire fixture.',
    findings=[],
    verified_invariants=[
        'Pure clock handoff is limited to the existing qualified same-station unknown observe_return planning scope, three-frame single_snapshot_direct, before publication starts. Public geometry and preview become invalid immediately; a certificate preserves private computation only, never motion or contact authority.',
        'New snapshots retain original source stamps and unchanged fusion checks. Each historical source was independently admitted before its original <=0.8 s geometry deadline with exact binding, object and mask digest. Latest source is <0.8 s; original retained mask bytes, object, target, increasing stamps and span are verified. Generation, target epoch and exact snapshot identity bind the private ticket.',
        'False objects, explicit processing pending, changed historical object/mask, source identity/binding changes, ownership, epoch, station motion and feedback failure preserve original hard cancellation. Hard cancellation clears private work/history, resets the coordinator and advances geometry invalidation. Old cancelled tickets are not renewed.',
        'The original ROS and monotonic deadlines, source freshness, fixed phase/session deadlines and recovery-parent validation remain. A normal receipt lacking required recovery proof cannot renew an expired reference. Aging inside original object/mask gates discards the inadequate source instead of manufacturing a new receipt.',
        'Ticket joins retain stream-to-geometry lock order. Waiting releases the condition lock and holds neither geometry nor control lock; cached owner checks and change/deadline signatures avoid unconditional 100 Hz Master RPC. Authoritative original gates run again before successful return and activation/publication.',
        'Publication-start and publication-success are separate. Starting a send irreversibly closes the soft-wait exception; successful publication does not self-cancel the current fresh ticket. A failed/partial preview publication hard-cancels private work. Existing geometry, contact, collision, path and final execution gates are unchanged.',
        'Initial/direct paths without a registered qualified station keep original behavior. Existing station scopes outside the three-frame direct configuration retain hard expiry semantics.',
    ],
    resolved_review_findings=[
        'Premature publication flag invalidated its own ticket before the base publisher, and could also cancel the worker after success; separated admission certificate from publication lifecycle.',
        'Soft cutoff early returns could hide a historical source becoming false or changing its mask; kept original hard-cutoff filtering and hard-cancelled changed audited content.',
        'Required recovery proof could disappear when original gates crossed the old pin deadline; source is now discarded without extending or permanently invalidating an otherwise live identity.',
        'Wait polling repeatedly invoked full RPC and did not independently notice silent feedback expiry/scope/clock changes; changed-input checks and original local deadline signatures cover these cases.',
        'Added exact retained mask validation and restored the original geometry lock around the newly introduced worker join entry.',
        'Combined regression found the aged-candidate source argument was dropped when introducing clock_only. Restoring quarantine(ns, clock_only=True) retains the original hard-cutoff progression outside the new scope; the original assertions are unchanged.',
    ],
    evidence=[
        dict(kind='pre_final_compatibility_fix_targeted_regression', path=str(targeted_path), sha256=sha(targeted_path), tests=86),
        dict(kind='independent_recorded_wire_regression_log', path=str(recorded_log), sha256=sha(recorded_log), tests=8),
        dict(kind='final_compatibility_fix_targeted_regression', path=str(cutoff_path), sha256=sha(cutoff_path), tests=38),
        dict(kind='combined_regression', path=str(combined_path), sha256=sha(combined_path), tests=combined['unique_tests']),
    ],
    validation_limits=[
        'The reviewer statically inspected frozen sources, tests and reported outputs; regression execution was performed by the implementation and combined-test agents, not independently repeated by this reviewer.',
        'The worker regression runs actual Mode entry, worker/coordinator, base geometry activation, publishers and terminal metrics with controlled CPU prediction and selected-candidate output. It does not validate IK, contact sufficiency or a physical grasp.',
        'The recorded fixture preserves 58 original ROS-wire messages, hashes and recorder receipt times. Qualified prior context is seeded using actual receipts and production session parsing; omitted sensor history is not reconstructed. Six callback permutations are explicit simulated scheduling, not claims about actual callback ingress.',
        'The actual recorded terminal false occurs after the original phase deadline; unexpired false rejection is isolated in synthetic negative tests. Live scheduling/performance and physical success require subsequent deployment verification.',
    ],
    production_edited_by_reviewer=False, deployed_by_reviewer=False, physical_actions=False,
)
destination = NEW / 'verification/gen8_station_admission_independent_review.json'
assert not destination.exists(), 'Do not overwrite an existing sealed review'
destination.write_text(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + '\n')
print(json.dumps(dict(path=str(destination), sha256=sha(destination), disposition=report['disposition']), ensure_ascii=False))
