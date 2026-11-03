"""Seal the independently reviewed GEN29 sources against completed offline evidence."""
import hashlib
import json
import os
from pathlib import Path
import time
import xml.etree.ElementTree as ET

W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
N = M / '实验记录/Robot-Controller/ros_log/full_latest_gen22_restart_20261015T003700Z'
V = N / 'verification'
O = V / 'gen29_final_independent_review'
BASE = V / 'gen28_release_manifest.json'
BASE_SHA = '2ae560cd671ae402366eaa41a2235905f51bd3b5cc6f43afd379df1f842bddfd'
COMBINED = V / 'gen29_combined_r1/result.json'
DIAGNOSIS = V / 'gen29_recorded_handoff_diagnosis/result.json'
DIAGNOSIS_SHA = '5abf72749df3e7812a7cc1a6c5d26dd8f5e3e473fe83886001b4a1e2d3877999'
REVIEWED = {
    'src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_remote.py': '1501f521abd1e70cd361d3d4018632e2323648ec4558ccb34a59f5899b9a9c73',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_task.py': '1cfa3bffa8162ee7c6f40c3fcd3eca378122f315d379f218021b11299f453d3f',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_task.py': 'd1f44dd685e035ae44b665b9524fbc7698110fe965912422883be67558c5f370',
    'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py': 'fa866437580ac9f94f967c78faf80f2a108333f8a09a4c6644142787eebba141',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/station_supplemental_admission.py': '3967098c136a9d98270f335c39f54df8427eef32b2627de653ec1c0cf5f9c870',
    'src/alicia_grasp_modes/tests/test_gen29_supplemental_preview_retention.py': 'd4e1d225ed047228930d15e41e6e91d0bb6a2e1f70dccbfe4ef112a44c5f6ba5',
    'src/alicia_grasp_modes/tests/test_gen29_supplemental_receipt_clock_independent.py': '2c33f36d9416f17b71c8af16ad38cf231cd4dde927eda2449a7f06235a1b6a54',
    'src/alicia_grasp_modes/tests/test_gen29_supplemental_remote_publication.py': '4c102b33f4ef0526b92bb5eb94544c09db6ab886fa2f559b767b0e8e026afdaa',
    'src/alicia_grasp_modes/tests/test_gen29_supplemental_retention_independent.py': '4f75877bc66d2eb2d0cfe22a69fdeb65e5bdf7fcf42679580c5b6c31a15473e5',
    '.runtime/verify_gen29_combined_20261015.py': 'e9b750a0a9d0599649ef96944802897ab075a2685649e8cef9d1e954201a313e',
    '.runtime/seal_gen29_release_20261015.py': 'a9b9775ee05dbdb93f9d0d6d0c8d8c958f073e09f8cd958f8140a29c9100e9b3',
    '.runtime/diagnose_gen29_recorded_handoff.py': '43f56e6c0dfcb002ad38cc31647e2e4d84f54c082a19324f96e10b6ff192d6ed',
    '.runtime/run_gen29_supplemental_independent.py': '0e0a8852a95910ee21ee2384e37dc97a39294cb60c5b51ee17bf128a2a3f2a88',
    '.runtime/run_gen29_supplemental_retention.py': '79f4741cfc0cd934b430f8e3fc36c560fff59caae21ca293a128d9872672c5b9',
}
HISTORY = {
    'gen29_supplemental_retention_red': 'Three causal failures through the original task entry before the repair.',
    'gen29_supplemental_retention_green_r1': 'Earlier 222-pass revision; predates final MRO and authority corrections, not final proof.',
    'gen29_supplemental_independent_red_r1': 'Exact saved GEN28 task-module overlay: six causal callback races and one fixture error (foreign plan track differed from embedded geometry track). Preserve all seven failures; only six are causal.',
    'gen29_supplemental_independent_green_r1': 'Earlier 36-pass consumer revision; not final whole-patch proof.',
    'gen29_supplemental_remote_green_r1': 'One collection error from the test import name; no executed passing-test claim.',
    'gen29_supplemental_remote_green_r2': 'Earlier 110-pass actual MRO subset; superseded by final focused run.',
    'gen29_authority_first_check_red': 'Two causal failures in 13 cases: first audit/token rejection changed to approval on the repeated base check and reached a publisher without the barrier.',
    'gen29_supplemental_context_counterfactual_red': 'Four failures using a minimal in-memory context-then-build counterfactual. This is not a complete historical GEN28 source snapshot.',
    'gen29_supplemental_independent_green_r2': '39 pass and one fixture failure: floating seconds converted to one nanosecond before the intended deadline.',
    'gen29_supplemental_clock_fixture_check': 'Diagnostic fixture check proves actual now 23012525638 is less than original deadline 23012525639. This is not a production expiry bug.',
    'gen29_supplemental_retention_final': 'Final frozen production, 343 passing focused and inherited tests; no failures, errors or skips.',
    'gen29_supplemental_independent_final': 'Final 38 independent task and four actual remote clock tests, all passing. Includes two audit-current-before-local-join orders. Expiry fixture advances one millisecond and asserts the original deadline is actually crossed.',
}


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for data in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(data)
    return h.hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def xml_summary(path):
    root = ET.parse(path).getroot()
    return {key: sum(int(row.get(key, 0)) for row in root.iter('testsuite'))
            for key in ('tests', 'failures', 'errors', 'skipped')}


def main():
    assert M.is_mount() and not os.statvfs(M).f_flag & os.ST_RDONLY
    assert V.stat().st_dev == M.stat().st_dev and not O.exists()
    evidence = {}

    def bind(path, expected=None):
        path = Path(path).resolve(strict=True)
        digest = sha(path)
        assert expected is None or digest == expected, str(path)
        assert str(path) not in evidence or evidence[str(path)] == digest
        evidence[str(path)] = digest
        return path

    for name, digest in REVIEWED.items():
        assert sha(W / name) == digest, name
    baseline = read(bind(BASE, BASE_SHA))
    assert baseline['source_count'] == 965 and baseline['test_scope_files'] == 230
    assert baseline['test_summary'] == dict(tests=7067, failures=0, errors=0, skipped=0)
    combined = read(bind(COMBINED))
    assert combined['passed'] and combined['returncode'] == 0
    assert combined['source_unchanged'] and combined['scope_complete']
    assert combined['baseline_file_test_counts_retained'] and not combined['unassigned_testcases']
    assert combined['baseline'] == str(BASE) and combined['baseline_sha256'] == BASE_SHA
    assert combined['source_count'] == 970 and combined['test_scope_files'] == 234
    assert combined['regression_scope'] == 'full_combined' and combined['production_worktree_tested']
    assert combined['live_ros_contacted'] is False and combined['motion_commands_sent'] is False
    assert combined['summary']['tests'] > 7067
    assert all(combined['summary'][key] == 0 for key in ('failures', 'errors', 'skipped'))
    for name, digest in combined['source_sha256'].items():
        assert sha(W / name) == digest, name
        if name in REVIEWED:
            assert REVIEWED[name] == digest, name
    assert set(baseline['test_scope']) <= set(combined['test_scope'])
    for name, count in baseline['test_case_counts_by_file'].items():
        assert combined['test_case_counts_by_file'][name] >= count, name
    for short, count in (('preview_retention', 3), ('receipt_clock_independent', 4),
                         ('remote_publication', 13), ('retention_independent', 38)):
        name = 'src/alicia_grasp_modes/tests/test_gen29_supplemental_' + short + '.py'
        assert combined['test_case_counts_by_file'][name] == count
    bind(COMBINED.parent / 'pytest.log', combined['log_sha256'])
    bind(COMBINED.parent / 'pytest.xml', combined['xml_sha256'])
    assert xml_summary(COMBINED.parent / 'pytest.xml') == combined['summary']
    patch = read(bind(combined['patch'], combined['patch_sha256']))
    for name in patch['files']:
        assert name in REVIEWED and patch['files'][name]['sha256'] == REVIEWED[name]
    for path, digest in patch['evidence_sha256'].items():
        bind(path, digest)
    runs = []
    for name, interpretation in HISTORY.items():
        folder = V / name
        receipt = read(bind(folder / 'receipt.json'))
        bind(folder / 'pytest.log'); bind(folder / 'pytest.xml')
        assert xml_summary(folder / 'pytest.xml') == receipt['summary']
        assert receipt['source_unchanged'] is True
        if name.endswith('_final'):
            assert receipt['returncode'] == 0
            assert all(receipt['summary'][key] == 0 for key in ('failures', 'errors', 'skipped'))
            for source, digest in receipt['source_sha256'].items():
                assert sha(W / source) == digest, source
        runs.append(dict(path=str(folder / 'receipt.json'), summary=receipt['summary'],
                         interpretation=interpretation))
    diagnosis = read(bind(DIAGNOSIS, DIAGNOSIS_SHA))
    assert diagnosis['record_closed_verified'] and diagnosis['recorded_supplemental_preview_published']
    assert diagnosis['recorded_pending_overwrote_preview']
    assert diagnosis['live_ros_contacted'] is False and diagnosis['physical_grasp_verified'] is False
    # The reviewed extractor actually verified the complete bag hash. Rebind
    # its evidence now as well; no live graph or temperature topics are used.
    for path, digest in diagnosis['evidence_sha256'].items():
        bind(path, digest)
    timeline = read(diagnosis['timeline'])
    assert timeline['preview_public_window_ns'] == 349705458
    assert timeline['audit_after_preceding_geometry_expiry_ns'] == 8199167
    assert timeline['observation_audit_promotion']['promote'] is False
    assert timeline['observation_audit_promotion']['code'] == 'EXECUTION_FROZEN'
    geometry = V / 'gen28_attempt1_local_rgb_edges_prototype_r1/measured_conflict_analysis.json'
    bind(__file__)
    review = dict(schema_version=1, reviewed=True, reviewed_ns=time.time_ns(),
        blocking_findings=[], scope='GEN29 supplemental preview delivery and original authority boundaries',
        reviewed_source_sha256=REVIEWED, evidence_sha256=evidence,
        full_regression=dict(summary=combined['summary'], source_count=970, test_scope_files=234,
            baseline_tests=7067, baseline_test_files=230, inherited_per_file_counts_retained=True,
            result=str(COMBINED), unique_case_count_not_added_to_focused_runs=True),
        run_history=runs,
        source_review=[
            'The real ModeAware Remote wrapper classifies the exact five-source supplemental computation as private before station mixin publication; the original soft-wait predicate never overrides execution publication flags.',
            'The actual atomic audit writer adds an immutable plan/source witness receipt despite the diagnostic audit-builder early return; the real consumer checks file bytes, exact plan digest and its own registered source witnesses.',
            'Context work crossing geometry TTL releases input locks and waits for a complete measured successor within the original identity, phase, session, support-gap, ROS and monotonic limits. No source or deadline is restamped.',
            'A receipt current witness arriving before the local join waits only while fresh; already registered contradictions, hard loss, ownership changes, manual takeover and expired clocks cannot revive private work.',
            'Cached computation is only a candidate. The unchanged observation freshness, full supplemental audit, checkpoint, admission, checked path and atomic handoff remain downstream.',
            'Actual execution token/pair entries close private retention before the first publisher callback. First audit/token rejection returns without a second chance to send; partial-send recovery remains in the original implementation.',
            'The full runner keeps all 7067 GEN28 cases with per-file counts, pins all sources/evidence before and after, and isolates ROS at an unreachable local address. All artifacts remain on SanStand.',
        ],
        recorded_failure=dict(preview_public_window_ns=349705458, audit_after_preceding_expiry_ns=8199167,
            strict_path_previously_passed=True, promotion='EXECUTION_FROZEN', later_valid_previews=0,
            limitation='Bag receipts do not expose exact task callback/thread execution times; causal offline entry-point tests provide the implementation evidence.'),
        unresolved_geometry_followup=dict(context_only_not_release_evidence=True,
            retained_diagnostic=str(geometry),
            retained_diagnostic_sha256='3ca4ca57f54d2c5958a5f722029bd206ccb331e8c9937caa83bd9c46f77d0788',
            correction=str(V / 'gen29_registered_box_conflict_review_r2/findings.json'),
            correction_sha256='fb5b7b373f778a21ac7eb3de949b212418d3b1acaeba008203e0d797952c70a6',
            production_geometry_changed=False,
            current_points='993 points from the current three-frame fused snapshot before registered fusion enclosure, not 993 single-frame raw pixels',
            additional_registered_points=1754, current_conflicting_points=0, registered_tail_conflicting_points=6,
            maximum_registered_tail_excess_m=0.0025914177636608736,
            terminology_correction='The preserved diagnostic called the entire tail historical. The tail is additional registered fused geometry and may include representatives from the current view. Tail membership alone does not assign each conflict to an older view; no such per-point source claim is required for this release.',
            uncertainty_preserved='Prototype retains edge 3.571 mm, height 1.248 mm and yaw 0.06085 rad uncertainty; no measurement points were removed.',
            qualification='The RGB prototype still fails measured-model consistency. No qualified model, opening feasibility, complete grasp path or physical success is established.',
            user_preference='Prefer a direct grasp from the initial view when evidence and all original gates support it; investigate precise localization before supplemental motion.'),
        limits=['Offline receipt and callback validation does not prove a new physical grasp or supplemental motion.',
            'GEN29 changes delivery/authority handling only; RGB edge extraction, registered-fusion model consistency, opening and collision geometry remain unchanged.',
                'Older green revisions, collection errors and fixture failures remain preserved and are not counted as final passing evidence.'],
        runtime_deployment_pending=True, physical_grasp_verified=False,
        live_ros_contacted=False, motion_commands_sent=False, temperature_queried=False)
    assert all(sha(W / name) == digest for name, digest in REVIEWED.items())
    O.mkdir()
    path = O / 'review.json'
    with path.open('x') as stream:
        json.dump(review, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write('\n'); stream.flush(); os.fsync(stream.fileno())
    assert read(path) == review
    print(json.dumps(dict(review=str(path), sha256=sha(path), summary=combined['summary'])))


if __name__ == '__main__':
    main()
