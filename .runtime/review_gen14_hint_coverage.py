"""Seal an independent offline source/fixture/test review. No ROS imports."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

WORK = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
STAGE = WORK / '.runtime/gen14_endpoint_hint_coverage_20261011'
NEW = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_no_skin_20261011T000625Z')
assert Path('/media/zhuyupei/SanStand').is_mount()
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
delivery_path = STAGE / 'delivery.json'
delivery = json.loads(delivery_path.read_text())
for path, digest in delivery['source_sha256'].items():
    assert sha(STAGE / path) == digest
work_source_state = {}
for path, digest in delivery['base_sha256'].items():
    if digest is not None:
        current = sha(WORK / path)
        assert current in (digest, delivery['source_sha256'][path])
        work_source_state[path] = dict(sha256=current,
            state='root_integrated_frozen_patch' if current == delivery['source_sha256'][path] else 'baseline')

fixture_path = STAGE / 'src/alicia_grasp_modes/tests/fixtures/gen14_endpoint_hint_coverage.json'
fixture = json.loads(fixture_path.read_text())
audit_path = Path(fixture['source_audit_path'])
assert sha(audit_path) == fixture['source_audit_sha256']
audit = json.loads(audit_path.read_text())['observation_failure_diagnostics']
original = audit['observation_candidates']
assert len(original) == len(fixture['rows']) == 220
direct_keys = ('candidate_index', 'base_from_tool', 'camera_target_distance_m',
               'camera_roll_offset_deg', 'camera_tilt_offset_deg', 'preference_index',
               'observation_envelope')
for expected, frozen in zip(original, fixture['rows']):
    assert all(frozen[k] == expected[k] for k in direct_keys)
    assert frozen['information_utility'] == expected['observation_view']['information_utility']
    assert frozen['metrics'] == expected['observation_view']['check_scheduling_metrics']
hint_report = audit['unknown_observation_scheduling']['endpoint_ik_search_hint']
assert fixture['scheduler_report'] == hint_report
assert fixture['joint_seed'] == hint_report['joint_seed']
mount_sha = hashlib.sha256(np.asarray(fixture['tool_from_camera'], dtype=float).tobytes()).hexdigest()
assert mount_sha == fixture['tool_from_camera_sha256'] == audit['source']['captured_audit']['tool_camera_sha256']

verification = NEW / 'verification/gen14_endpoint_hint_coverage'
manifest_path = verification / 'independent_r2_manifest.json'
manifest = json.loads(manifest_path.read_text())
assert manifest['returncode'] == 0 and manifest['source_unchanged']
for path, digest in delivery['source_sha256'].items():
    assert manifest['source_sha256'][path] == digest
    assert manifest['source_after_sha256'][path] == digest
xml = ET.parse(verification / 'independent_r2.xml').getroot()
suite = xml if xml.tag == 'testsuite' else xml.find('testsuite')
assert int(suite.attrib['tests']) == 94 and int(suite.attrib['failures']) == 0 and int(suite.attrib['errors']) == 0

summary_path = verification / 'replay_summary.json'
summary = json.loads(summary_path.read_text())
report = dict(
    reviewed=True, blocking_findings=[], reviewer='gen6_contact_search',
    scope='Advisory endpoint IK hint ordering only; no execution/geometry/budget changes',
    stage=str(STAGE), delivery=str(delivery_path), delivery_sha256=sha(delivery_path),
    source_sha256=delivery['source_sha256'], base_sha256=delivery['base_sha256'],
    work_source_state_at_seal=work_source_state,
    source_fixture_verification=dict(
        source_audit=str(audit_path), source_audit_sha256=sha(audit_path),
        exact_recorded_rows_verified=220, direct_keys_verified=list(direct_keys),
        all_information_metrics_and_support_witnesses_exact=True,
        joint_seed_and_scheduler_report_exact=True, mount_array_sha256=mount_sha),
    independent_test_manifest=str(manifest_path), independent_test_manifest_sha256=sha(manifest_path),
    independent_tests_passed=94, independent_tests_failed=0,
    first_attempt=dict(path=str(verification / 'independent_r1_manifest.json'),
        outcome='Collection failed: existing generated ROS message package was absent from PYTHONPATH; no source edits; corrected existing devel import path in r2'),
    behavior_review=[
        'Original support strata retained; zero-gain views remain behind positive-gain views within their stratum.',
        'Best information-ranked useful view remains first, then existing distance/optical-direction coverage distributes advisory queries.',
        'All 220 exact target transforms retained; no additional poses or dropped candidates.',
        'Existing shared 3 s start budget, maximum 96 queries and 0.02 s service timeout are unchanged; cache does not renew on reorder.',
        'Positive endpoint hints remain advisory. Actual strict-loop regression rejects a controlled low-tilt positive hint on collision; no full-path incumbent or valid rich plan is published.',
        'Current prepared-request validation, model binding, source identity, collision-aware service request and later strict/following gates are unchanged.'
    ],
    replay_summary=str(summary_path), replay_summary_sha256=sha(summary_path),
    recorded_query_summary=summary['recorded_query_summary'],
    new_actual_callback_query_summary=summary['new_actual_callback_query_summary'],
    limitations=[
        'Mock service replies test dispatch/authorization boundaries, not physical IK success.',
        'Current-scene independent ROS queries found all 220 recorded poses without IK; this fix closes query starvation but does not resolve that geometric limitation.',
        'Offline review only; no ROS master, robot motion, temperature query or production source edit.'
    ])
out = NEW / 'gen14_analysis/hint_coverage_review.json'
assert not out.exists()
out.write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps(dict(path=str(out), sha256=sha(out), reviewed=True, blocking_findings=[])))
