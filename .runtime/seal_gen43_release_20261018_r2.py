"""Seal reviewed GEN43 offline evidence; never install, deploy or contact ROS.

Preparation is not a release. Frozen source/test and actual replay/focus/full
receipts must all agree before this helper can create the external manifest.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import time
import xml.etree.ElementTree as ET


W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
E = M/'实验记录/Robot-Controller'
V = E/'ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
BASE = V/'gen42_release_manifest.json'
BASE_SHA = '4ce3b5142b18e9968deff461d39a202f0fe6f91572394744abad67d29c4810dc'
BACKUP = W/'.runtime/gen42_source_before_gen43'
CANDIDATE = W/'.runtime/gen43_mask_candidate'
RUNNER = W/'.runtime/verify_gen43_offline_tests_20261018_r2.py'
RUNNER_SHA = 'de1b3727a78644ade82690dbfd748745c4e03969eda18556bd1192b54c55698f'
CHANGE = 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/cuboid_rgb_interval.py'
NEW = (
    'src/alicia_flexible_grasp_supervisor/tests/test_gen43_top_mask_contract.py',
    'src/alicia_grasp_modes/tests/test_gen43_top_mask_handoff.py',
)
COMPAT = 'src/alicia_grasp_modes/tests/test_gen40_clearance_enclosure_chain.py'
COMPAT_BEFORE_SHA = 'a2d73319cfc7bd5a0fba2ef53a3046c7cf2fd53dec0794c354bf6a4e7636edf1'
COMPAT_AFTER_SHA = 'e766140daef39331cfb6cfb797eabf6e6a477d2c0db228d71f9e99e626dd6084'
COMPAT_BACKUP = W/'.runtime/gen43_gen40_hash_fixture_a2d_before_r1.py.txt'
COMPAT_PROOF = V/'gen43_historical_model_hash_comparison_r1/result.json'
COMPAT_PROOF_SHA = '9505bd2732141e897d512d77f355386a494769068dd7f12929769c063a61b6ca'
COMPAT_REVISION = V/'gen43_gen40_model_hash_fixture_revision_r1/result.json'
COMPAT_REVISION_SHA = '14f5905f8035d7e97561d8cdd031d00ef5177a8be4237923523ac0f58bbcd770'
PRIOR_FULL = V/'gen43_combined_r1/result.json'
PRIOR_FULL_SHA = '255736d7c8c66a5161b3bf4eb5b8008752ee5eb4f011ee831a665e277114854c'
ORIGINAL_RUNNER = W/'.runtime/verify_gen43_offline_tests_20261018.py'
ORIGINAL_RUNNER_SHA = '86e4a35b7e122ba81e18d02f8875c4a1db26769c45c35c6274f16dad0e55617c'
ORIGINAL_SEALER = W/'.runtime/seal_gen43_release_20261018.py'
ORIGINAL_SEALER_SHA = 'afe2a301f60d4a55e6668a08f432e4dbe6017d95f5f4fc72b56dc8f55f54d049'
EXPECTED_SOURCE = {
    CHANGE: '50e00a5b751ba0470f98c4215922bb6f05471784a364181c297ad5f617751d7f',
    NEW[0]: '3b6722a0514f280aa7aaba28450a18a7e2dd8503359460c8b0e8d692abdb4cf6',
    NEW[1]: '6b9d31328431558be8f9b79ca987febf9d33f2dde4381feb73be542d2cd0c4cb',
    COMPAT: COMPAT_AFTER_SHA,
}
FOCUS_SCOPE = (*NEW, COMPAT)
EXPECTED_NEW_COUNTS = {NEW[0]: 37, NEW[1]: 5}
ORACLE_LABEL = 'alicia_flexible_grasp.vision._gen38_registration_test_baseline'
ORACLE = W/'.runtime/gen38_baseline_source/src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/multiview_surface.py'
ORACLE_SHA = '369a2947114c6c3953c439a7937cf26971e337c015e65abf96825720bc5c1ba8'
ORACLE_TEST = 'src/alicia_flexible_grasp_supervisor/tests/test_gen39_registration_transform.py'
ORACLE_TEST_SHA = 'fcd9fe03951fc4761f1faaf486b1ccf979583cb1cd07be39f9912845a5bd7c71'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def inside(path, root):
    try:
        Path(path).relative_to(root)
        return True
    except ValueError:
        return False


def storage(output):
    assert M.is_mount(), 'SanStand unavailable; no system-disk fallback'
    assert inside(output, V.resolve(strict=True)) and output != V.resolve(strict=True)
    assert output.parent.stat().st_dev == M.stat().st_dev
    config = json.loads((E/'grasp_records/archive_config.json').read_text())
    policy = json.loads((E/'maintenance/storage_control.json').read_text())
    assert config['upload_enabled'] is False and config['delete_enabled'] is False
    assert policy['archive_policy'] == 'local_only'
    assert shutil.disk_usage(E).free >= config['min_free_bytes']


def compatibility_inputs(original):
    """Bind the exact, reviewed exception without rewriting historical receipts.

    The proof ran before the old test was amended: its mutable W path recorded
    a2d at that time. That entry is now resolved through both immutable backups,
    rather than falsely requiring the present W test to retain the old bytes.
    """
    dependencies = {}
    def keep(path, digest):
        assert sha(path) == digest, str(path)
        dependencies[str(path)] = digest
    for path, digest in ((COMPAT_PROOF, COMPAT_PROOF_SHA),
            (COMPAT_REVISION, COMPAT_REVISION_SHA), (PRIOR_FULL, PRIOR_FULL_SHA),
            (ORIGINAL_RUNNER, ORIGINAL_RUNNER_SHA), (ORIGINAL_SEALER, ORIGINAL_SEALER_SHA),
            (COMPAT_BACKUP, COMPAT_BEFORE_SHA), (BACKUP/COMPAT, COMPAT_BEFORE_SHA)):
        keep(path, digest)
    assert original[COMPAT] == COMPAT_BEFORE_SHA
    proof = json.loads(COMPAT_PROOF.read_text())
    revision = json.loads(COMPAT_REVISION.read_text())
    assert proof['passed'] is True and proof['sources_unchanged'] is True
    assert proof['external_boundary_attempts'] == []
    assert all(proof[k] is False for k in ('ROS_initialized', 'network_access',
        'hardware_access', 'temperature_queried', 'production_or_tests_changed', 'pytest_suite_run'))
    fields = ('coverage_policy', 'silhouette_mask_intersection_pixels',
        'silhouette_mask_eroded_polygon_pixels', 'top_mask_intersection_pixels',
        'top_mask_eroded_polygon_pixels', 'top_mask_coverage')
    assert proof['exact_added_fields'] == list(fields)
    assert all(proof[k] is True for k in ('no_old_fields_or_values_changed',
        'legacy_projection_canonical_equal', 'new_source_and_numeric_validator_passed',
        'original_points_bytes_unchanged',
        'geometry_and_all_old_uncertainty_source_qualification_fields_bit_exact'))
    assert proof['original_points_count'] == 2610
    assert proof['source_sha256'][str(W/COMPAT)] == COMPAT_BEFORE_SHA
    for path, digest in proof['source_sha256'].items():
        if path != str(W/COMPAT): keep(Path(path), digest)
    for path, digest in proof['output_sha256'].items(): keep(Path(path), digest)
    legacy = json.loads((COMPAT_PROOF.parent/'legacy_model.json').read_text())
    current = json.loads((COMPAT_PROOF.parent/'current_model.json').read_text())
    projected = json.loads(json.dumps(current))
    projected.pop('model_sha256')
    for name in fields: projected['qualification']['cuboid_rgb_interval'].pop(name)
    legacy_body = dict(legacy); legacy_body.pop('model_sha256')
    assert projected == legacy_body
    canonical = lambda value: json.dumps(value, sort_keys=True, separators=(',', ':'),
        allow_nan=False, ensure_ascii=True).encode('utf8')
    assert hashlib.sha256(canonical(projected)).hexdigest() == proof['legacy_model_sha256']
    current_body = dict(current); current_body.pop('model_sha256')
    assert hashlib.sha256(canonical(current_body)).hexdigest() == proof['current_model_sha256']
    assert legacy['model_sha256'] == proof['legacy_projection_digest'] == proof['legacy_model_sha256']
    assert current['model_sha256'] == proof['current_model_sha256']
    assert revision['fixture_relative'] == COMPAT
    assert revision['before_sha256'] == revision['backup_sha256'] == COMPAT_BEFORE_SHA
    assert revision['after_sha256'] == COMPAT_AFTER_SHA
    assert revision['same_source_model_proof'] == str(COMPAT_PROOF)
    assert revision['same_source_model_proof_sha256'] == COMPAT_PROOF_SHA
    assert revision['old_file_collected_case_count'] == 4
    assert all(revision[k] is True for k in ('all_existing_test_assert_AST_preserved',
        'all_existing_test_parameter_AST_preserved', 'all_other_old_285_test_files_bytes_preserved',
        'projection_never_used_as_execution_model', 'upstream_failed_and_interrupted_full_report_preserved'))
    keep(Path(revision['diff']), revision['diff_sha256'])
    prior = json.loads(PRIOR_FULL.read_text())
    assert prior['passed'] is False and prior['scope_complete'] is False
    assert prior['summary'] == dict(tests=8056, failures=1, errors=0, skipped=0)
    descriptor = dict(path=COMPAT, before_sha256=COMPAT_BEFORE_SHA,
        after_sha256=COMPAT_AFTER_SHA, collected_case_count=4,
        compatibility_proof=str(COMPAT_PROOF), compatibility_proof_sha256=COMPAT_PROOF_SHA,
        fixture_revision=str(COMPAT_REVISION), fixture_revision_sha256=COMPAT_REVISION_SHA,
        historical_before_source_resolved_to=[str(BACKUP/COMPAT), str(COMPAT_BACKUP)],
        historical_model_sha256=proof['legacy_model_sha256'],
        current_model_sha256=proof['current_model_sha256'], exact_added_fields=list(fields),
        projection_used_for_execution=False,
        prior_failed_incomplete_full=str(PRIOR_FULL), prior_failed_incomplete_full_sha256=PRIOR_FULL_SHA)
    return descriptor, dependencies


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('combined', 'focus', 'baseline-replay', 'candidate-replay', 'review', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    storage(output)
    assert output == (V/'gen43_release_manifest.json').resolve() and not output.exists()
    assert all(len(h) == 64 and set(h) <= set('0123456789abcdef')
               for h in [RUNNER_SHA, *EXPECTED_SOURCE.values()]), 'Preparation is not frozen'
    evidence = {}

    def bind(path, expected=None):
        path = Path(path).resolve(strict=True)
        digest = sha(path)
        assert expected is None or digest == expected, str(path)
        if str(path) in evidence:
            assert evidence[str(path)] == digest, str(path)
        evidence[str(path)] = digest
        return path

    baseline = json.loads(bind(BASE, BASE_SHA).read_text())
    original, old_scope = baseline['source_sha256'], baseline['test_scope']
    old_counts = baseline['test_case_counts_by_file']
    assert all(baseline[k] is True for k in ('passed', 'reviewed', 'installed'))
    assert len(original) == baseline['source_count'] == 1034
    assert len(old_scope) == len(set(old_scope)) == len(old_counts) == 286
    assert set(old_scope) == set(old_counts) and sum(old_counts.values()) == 8216
    assert baseline['test_summary'] == dict(tests=8216, failures=0, errors=0, skipped=0)
    backup_manifest = json.loads(bind(BACKUP/'manifest.json').read_text())
    assert backup_manifest['source_sha256'] == original
    assert all(sha(BACKUP/p) == h for p, h in original.items())
    bind(RUNNER, RUNNER_SHA)
    compatibility, compatibility_dependencies = compatibility_inputs(original)
    assert old_counts[COMPAT] == 4
    for path, digest in compatibility_dependencies.items(): bind(path, digest)

    def completed_run(path, *, full):
        path = bind(path)
        result = json.loads(path.read_text())
        assert result['passed'] is True and result['returncode'] == 0
        assert result['full'] is full and result['baseline_sha256'] == BASE_SHA
        assert result['source_unchanged'] is True and result['scope_complete'] is True
        assert result['changed_source_loaded'] is True and result['runner_sha256'] == RUNNER_SHA
        for key in ('baseline_count_mismatches', 'unassigned_testcases',
                    'foreign_or_inconsistent_imports', 'external_boundary_attempts', 'changed_during_run'):
            assert not result[key], key
        for key in ('live_ros_contacted', 'hardware_contacted', 'motion_commands_sent',
                    'temperature_queried', 'deployed', 'physical_grasp_verified'):
            assert result[key] is False, key
        assert result['source_count'] == len(result['source_sha256']) == 1036
        assert set(result['source_sha256']) == set(original) | set(NEW)
        assert set(result['changed_from_gen42']) == {CHANGE, COMPAT}
        assert len(result['changed_from_gen42']) == 2
        assert all(result['source_sha256'][p] == original[p]
                   for p in original if p not in (CHANGE, COMPAT))
        assert result['changed_production_sources'] == [CHANGE]
        assert result['modified_existing_tests'] == [COMPAT]
        assert result['original285_test_files_bytes_unchanged'] is True
        assert result['original286_exact_counts_preserved'] is True
        assert result['historical_fixture_compatibility'] == compatibility
        assert result['compatibility_proof_sha256'] == COMPAT_PROOF_SHA
        assert result['compatibility_evidence_sha256'] == compatibility_dependencies
        assert all(result['source_sha256'][p] == h for p, h in EXPECTED_SOURCE.items())
        root = W
        assert Path(result['workspace']).resolve(strict=True) == root.resolve(strict=True)
        scope = result['test_scope']
        assert len(scope) == len(set(scope)) == result['test_scope_files']
        assert set(scope) <= set(old_scope) | set(NEW) and set(NEW) <= set(scope)
        if full:
            assert scope == old_scope + list(NEW) and len(scope) == 288
        else:
            assert tuple(scope) == FOCUS_SCOPE and len(scope) == 3
        summary = result['summary']
        assert summary['tests'] == (8258 if full else 46)
        assert summary['tests'] > 0 and all(summary[k] == 0 for k in ('failures', 'errors', 'skipped'))
        xml = bind(path.parent/'pytest.xml', result['xml_sha256'])
        tree = ET.parse(xml).getroot()
        actual = {k: sum(int(s.get(k, 0)) for s in tree.iter('testsuite')) for k in summary}
        assert actual == summary
        counts = dict.fromkeys(scope, 0)
        for case in tree.iter('testcase'):
            name = case.get('file', '')
            assert name in counts and all(case.find(k) is None for k in ('failure', 'error', 'skipped'))
            counts[name] += 1
        assert counts == result['test_case_counts_by_file'] and all(counts.values())
        assert all(counts[p] == old_counts[p] for p in scope if p in old_counts)
        expected_counts = dict(old_counts) if full else {COMPAT: 4}
        expected_counts.update(EXPECTED_NEW_COUNTS)
        assert counts == expected_counts
        bind(path.parent/'pytest.log', result['log_sha256'])
        declaration = json.loads(bind(path.parent/'scope.json').read_text())
        assert declaration['full'] is full and declaration['tests'] == scope
        assert declaration['source_sha256'] == result['source_sha256']
        assert declaration['baseline_sha256'] == BASE_SHA and declaration['runner_sha256'] == RUNNER_SHA
        assert declaration['original_test_counts'] == old_counts
        assert declaration['changed_production_sources'] == [CHANGE]
        assert declaration['modified_existing_tests'] == [COMPAT]
        assert declaration['original_sources_unchanged'] is False
        assert declaration['original285_test_files_bytes_unchanged'] is True
        assert declaration['original286_exact_counts_preserved'] is True
        assert declaration['historical_fixture_compatibility'] == compatibility
        assert declaration['compatibility_proof_sha256'] == COMPAT_PROOF_SHA
        assert declaration['compatibility_evidence_sha256'] == compatibility_dependencies
        for name, digest in result['generated_bootstrap_sha256'].items():
            assert not Path(name).is_absolute() and '..' not in Path(name).parts
            bind(path.parent/name, digest)
        provenance = json.loads(bind(path.parent/'module_provenance.json',
                                    result['module_provenance_sha256']).read_text())
        assert any(row['path'] == str(root/CHANGE) and row['sha256'] == EXPECTED_SOURCE[CHANGE]
                   for row in provenance.values())
        oracle_rows = {}
        for label, row in provenance.items():
            module = Path(row['path']).resolve(strict=True)
            if inside(module, root/'src'):
                relative = str(module.relative_to(root))
                assert relative in result['source_sha256']
                assert row['sha256'] == result['source_sha256'][relative]
            elif not inside(module, W/'devel'):
                assert label == ORACLE_LABEL and module == ORACLE.resolve(strict=True)
                assert row['sha256'] == ORACLE_SHA and ORACLE_TEST in scope
                assert result['source_sha256'][ORACLE_TEST] == ORACLE_TEST_SHA
                oracle_rows[label] = row
            bind(module, row['sha256'])
        assert oracle_rows == result['accepted_exact_historical_oracle']
        if full:
            assert oracle_rows == {ORACLE_LABEL: dict(path=str(ORACLE), sha256=ORACLE_SHA)}
        boundary = path.parent/'boundary_attempts.jsonl'
        if boundary.exists():
            assert not bind(boundary).read_text().splitlines()
        return path, result

    combined_path, combined = completed_run(args.combined, full=True)
    focus_path, focus = completed_run(args.focus, full=False)
    sources = combined['source_sha256']
    assert sources == focus['source_sha256']
    expected_counts = dict(old_counts)
    expected_counts.update(EXPECTED_NEW_COUNTS)
    assert combined['test_case_counts_by_file'] == expected_counts
    assert combined['summary'] == dict(tests=sum(expected_counts.values()), failures=0, errors=0, skipped=0)
    assert all(sha(W/p) == h for p, h in sources.items())
    found_new = {str(p.relative_to(W)) for root in (
        W/'src/alicia_flexible_grasp_supervisor/tests', W/'src/alicia_grasp_modes/tests')
        for p in root.glob('test_gen43*.py')}
    assert found_new == set(NEW)

    baseline_replay = bind(args.baseline_replay)
    candidate_replay = bind(args.candidate_replay)
    # Replay schemas are reviewed evidence, not authority inferred from an
    # arbitrary `ok` field. The completed independent review binds exact files
    # and states its original-source/computation scope explicitly.
    review_path = bind(args.review)
    review = json.loads(review_path.read_text())
    assert review['reviewed'] is True and review['release_review_complete'] is True
    assert not review['blocking_findings']
    reviewed = review['reviewed_source_sha256']
    required = set(EXPECTED_SOURCE) | {str(RUNNER.relative_to(W)),
                                     str(Path(__file__).resolve().relative_to(W))}
    assert required <= set(reviewed)
    for name, digest in reviewed.items():
        assert not Path(name).is_absolute() and '..' not in Path(name).parts
        bind(W/name, digest)
    for path in (combined_path, focus_path, baseline_replay, candidate_replay,
                 COMPAT_PROOF, COMPAT_REVISION, PRIOR_FULL):
        assert review['evidence_sha256'].get(str(path)) == sha(path)
    replay_review = review['replay_review']
    assert replay_review['baseline_source_sha256'] == original[CHANGE]
    assert replay_review['candidate_source_sha256'] == EXPECTED_SOURCE[CHANGE]
    assert replay_review['same_archived_input_sources'] is True
    assert replay_review['original_failure_preserved'] is True
    assert replay_review['candidate_original_source_qualification_verified'] is True
    assert replay_review['no_hardware_or_motion'] is True
    for path, digest in review['evidence_sha256'].items():
        bind(path, digest)
    bind(__file__)
    assert all(sha(p) == h for p, h in evidence.items())
    assert all(sha(W/p) == h for p, h in sources.items())
    assert all(sha(BACKUP/p) == h for p, h in original.items())
    storage(output)
    release = dict(schema_version=1, kind='gen43_same_source_cuboid_top_mask_coverage',
        sealed_ns=time.time_ns(), passed=True, reviewed=True, installed=True,
        installed_semantics='Exact disk sources passed full offline regression and completed review; this helper installs nothing and starts no node.',
        source_sha256=sources, source_count=1036, test_scope=combined['test_scope'], test_scope_files=288,
        test_summary=combined['summary'], test_case_counts_by_file=combined['test_case_counts_by_file'],
        changed_production_sources=[CHANGE], modified_existing_tests=[COMPAT], new_tests=list(NEW),
        original285_test_files_bytes_unchanged=True,
        historical_fixture_compatibility=compatibility,
        compatibility_proof_sha256=COMPAT_PROOF_SHA,
        compatibility_evidence_sha256=compatibility_dependencies,
        original286_exact_counts_preserved=True, original286_case_count=8216,
        regression_scope='full_combined', baseline_release=str(BASE), baseline_release_sha256=BASE_SHA,
        actual_regression=str(combined_path), focused_regression=str(focus_path),
        baseline_source_replay=str(baseline_replay), candidate_source_replay=str(candidate_replay),
        combined_review=str(review_path), evidence_sha256=evidence,
        accepted_exact_historical_oracle=combined['accepted_exact_historical_oracle'],
        baseline_history_unchanged=True,
        prior_incomplete_failed_full_regression_preserved=str(PRIOR_FULL),
        prior_incomplete_failed_full_regression_sha256=PRIOR_FULL_SHA,
        historical_fixture_semantics='One existing four-case GEN40 fixture explicitly accepts the current full model digest and separately proves its six-field audit-only projection reproduces the historical digest; the production digest and consumer binding remain unchanged. The other 285 original test files remain byte-identical; all 286 original per-file case counts remain exact.',
        baseline_rawfalse_semantics='GEN42 original raw metadata rejection and its exact oracle addendum remain unchanged history; GEN43 full runner verifies the exact oracle directly.',
        runtime_deployment_pending=True, deployed=False,
        required_runtime_replaced_consumers=['/grasp_task_node', '/remote_grasp6d_node',
            '/unknown_tabletop_perception', '/motion_gateway'],
        required_runtime_preserved_consumers=['/alicia_d_driver_node', '/bessica_d_hw_interface',
            '/move_group', '/supervisor_camera', '/compliant_gripper_controller',
            '/grasp_mode_router', '/alicia_supervisor_gui'],
        runtime_shared_validator_requirement='GEN42 motion_gateway caches the old cuboid validator through ContactSequencePrecheckGateway and gripper_geometry. GEN43 idle deployment must reload four consumers including the gateway, preserve the other seven, and verify their source/process bindings; the GEN42 three-plus-eight finalizer is insufficient.',
        pending_validation_boundary='Only reported source qualification, offline common-chain fixtures and regression are verified. Runtime deployment, fresh measured target/robot/path acceptance and physical approach/closure remain separate.',
        live_ros_contacted=False, hardware_contacted=False, motion_commands_sent=False,
        temperature_queried=False, physical_grasp_verified=False)
    with output.open('x') as stream:
        json.dump(release, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n'); stream.flush(); os.fsync(stream.fileno())
    assert json.loads(output.read_text()) == release
    print(json.dumps(dict(release=str(output), sha256=sha(output), summary=release['test_summary'])))


if __name__ == '__main__':
    main()
