"""Seal a completed, reviewed GEN42 full offline regression; never install.

Preparation of this helper is not a release. It requires actual final XML,
source provenance, zero guarded boundary attempts, and a completed review.
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
OUTPUT_ROOT = E/'ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
BASE = E/'ros_log/full_latest_gen36_restart_20261017T012037Z/verification/gen41_release_manifest.json'
BASE_SHA = 'b98c25da01b77582bff4567a791afed8b8c908fcfe391b3b84601276c9ed706d'
BACKUP = W/'.runtime/gen41_baseline_source'
BACKUP_MANIFEST_SHA = '61c2533a61070079fc58e18fade39eb4fa5ff673828ca1b3663d587b4c6994d2'
RUNNER = W/'.runtime/verify_gen42_offline_tests_20261017.py'
GUARD_SOURCE = W/'.runtime/verify_gen41_offline_tests_20261017.py'
GUARD_SOURCE_SHA = '0c7c13321f4eed998d54f3ff49456179c7c4c28deb193e9ab646113554de21c9'
PRODUCTION = 'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py'
FIXTURE = 'src/alicia_flexible_grasp_supervisor/tests/test_fused_scene_request_audit.py'
NEW_TEST = 'src/alicia_grasp_modes/tests/test_gen42_geometry_lock_handoff.py'
SUPPLEMENTAL = {
    'src/alicia_flexible_grasp_supervisor/tests/test_fused_scene_pipeline.py': 11,
    'src/alicia_flexible_grasp_supervisor/tests/test_fused_scene_recenter.py': 5,
    FIXTURE: 11,
}
EXTRA = {'tools/run_fresh_grasp6d_after_alignment.py':
    '8d750485b6166823486c2933579ef3faef0640a4a323c6b6c2554a42a7a0c45b'}
EXPECTED_PREPARED_SOURCE_SHA256 = {
    PRODUCTION: 'fa011168dbf945f518934ffd6e0b0705d78a45285c8eabde5e5afafbcdc36818',
    NEW_TEST: '9af9d9c0dab6fddd50c51f107aaa63e4a1e6ae77750f6dd39bee08ac9432edae',
    FIXTURE: 'a3f05463fe04dfc3c2c75f29123261b65f758b68bd8468d15421c706696bcee3',
}
EXPECTED_RUNNER_SHA = '226b7f0cb282be162ea3ccd5c6e236e4127a8e6b3d9f0769772169962e383342'
HISTORICAL_LABEL = 'alicia_flexible_grasp.vision._gen38_registration_test_baseline'
HISTORICAL_PATH = W/'.runtime/gen38_baseline_source/src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/multiview_surface.py'
HISTORICAL_SHA = '369a2947114c6c3953c439a7937cf26971e337c015e65abf96825720bc5c1ba8'
COMPARISON_TEST = 'src/alicia_flexible_grasp_supervisor/tests/test_gen39_registration_transform.py'
COMPARISON_TEST_SHA = 'fcd9fe03951fc4761f1faaf486b1ccf979583cb1cd07be39f9912845a5bd7c71'
ADDENDUM_RAW_RESULT = OUTPUT_ROOT/'gen42_combined_r1/result.json'
ADDENDUM_RAW_SHA = 'bcb025ac7b4826f165af21dfaa62f0b67b13feba25345340917a03186417ad18'
ORACLE_TEST_FUNCTIONS = {
    'test_default_icp_result_matches_gen38_every_field_exactly': 14,
    'test_default_append_and_explicit_none_keep_gen38_fusion_exact': 3,
    'test_fixed_metadata_and_support_failures_keep_original_semantics': 4,
}


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def inside(path, root):
    try:
        Path(path).relative_to(root)
        return True
    except ValueError:
        return False


def storage(output):
    assert M.is_mount(), 'SanStand unavailable; no system-disk fallback'
    root = OUTPUT_ROOT.resolve(strict=True)
    assert inside(output, root) and output != root
    assert output.parent.stat().st_dev == M.stat().st_dev
    config = json.loads((E/'grasp_records/archive_config.json').read_text())
    policy = json.loads((E/'maintenance/storage_control.json').read_text())
    assert config['upload_enabled'] is False and config['delete_enabled'] is False
    assert policy['archive_policy'] == 'local_only'
    assert shutil.disk_usage(E).free >= config['min_free_bytes']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('combined', 'review', 'output'):
        parser.add_argument('--'+name, required=True, type=Path)
    parser.add_argument('--acceptance-addendum', type=Path,
        help='Explicit exact historical-comparator classification; raw result stays immutable')
    args = parser.parse_args()
    output = args.output.resolve()
    storage(output)
    assert not output.exists()
    assert all(len(value) == 64 and set(value) <= set('0123456789abcdef')
               for value in [EXPECTED_RUNNER_SHA, *EXPECTED_PREPARED_SOURCE_SHA256.values()]), \
        'Preparation hashes have not been frozen'
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
    original = baseline['source_sha256']
    original_scope = baseline['test_scope']
    original_counts = baseline['test_case_counts_by_file']
    assert baseline['passed'] is True and baseline['reviewed'] is True
    assert len(original) == baseline['source_count'] == 1032
    assert len(original_scope) == len(set(original_scope)) == baseline['test_scope_files'] == 282
    assert set(original_scope) == set(original_counts)
    assert baseline['test_summary'] == dict(tests=8168, failures=0, errors=0, skipped=0)
    assert sum(original_counts.values()) == 8168
    assert set(SUPPLEMENTAL) <= set(original) and not set(SUPPLEMENTAL) & set(original_scope)
    assert NEW_TEST not in original
    backup = json.loads(bind(BACKUP/'manifest.json', BACKUP_MANIFEST_SHA).read_text())
    assert backup['release_sha256'] == BASE_SHA and backup['source_sha256'] == original
    assert all(sha(BACKUP/name) == digest for name, digest in original.items())
    bind(RUNNER, EXPECTED_RUNNER_SHA)
    bind(GUARD_SOURCE, GUARD_SOURCE_SHA)

    combined_path = bind(args.combined)
    combined = json.loads(combined_path.read_text())
    addendum_path, addendum = None, None
    expected_historical = {HISTORICAL_LABEL: dict(
        path=str(HISTORICAL_PATH.resolve(strict=True)), sha256=HISTORICAL_SHA)}
    if args.acceptance_addendum is not None:
        # This one immutable full run failed only the metadata classification of
        # its intentionally imported, hash-checked historical comparison oracle.
        assert combined_path == ADDENDUM_RAW_RESULT.resolve(strict=True)
        assert sha(combined_path) == ADDENDUM_RAW_SHA and combined['passed'] is False
        assert combined['foreign_imports'] == expected_historical
        addendum_path = bind(args.acceptance_addendum)
        addendum = json.loads(addendum_path.read_text())
        assert addendum['kind'] == 'gen42_exact_historical_oracle_acceptance_addendum'
        assert addendum['passed'] is True and addendum['reviewed'] is True
        assert Path(addendum['raw_result']).resolve(strict=True) == combined_path
        assert addendum['raw_result_sha256'] == ADDENDUM_RAW_SHA
        assert addendum['raw_result_passed'] is False
        assert addendum['only_failed_raw_gate'] == 'foreign_imports'
        assert addendum['raw_foreign_imports'] == expected_historical
        assert addendum['unrecognized_foreign_imports'] == {}
        assert addendum['accepted_historical_oracle'] == dict(
            label=HISTORICAL_LABEL, **expected_historical[HISTORICAL_LABEL])
        comparison = addendum['unchanged_comparison_test']
        assert Path(comparison['path']).resolve(strict=True) == (W/COMPARISON_TEST).resolve(strict=True)
        assert comparison['sha256'] == comparison['baseline_sha256'] == COMPARISON_TEST_SHA
        assert comparison['AST_unchanged'] is True
        assert comparison['test_file_case_count'] == 38 and comparison['oracle_case_count'] == 21
        assert original[COMPARISON_TEST] == COMPARISON_TEST_SHA
        bind(W/COMPARISON_TEST, COMPARISON_TEST_SHA)
        bind(HISTORICAL_PATH, HISTORICAL_SHA)
        assert addendum['all_other_raw_pass_gates'] is True
        assert addendum['source_count'] == 1034 and addendum['test_scope_files'] == 286
        assert addendum['test_summary'] == dict(tests=8216, failures=0, errors=0, skipped=0)
        assert addendum['source_sha256'] == combined['source_sha256']
        assert addendum['evidence_sha256'].get(str(combined_path)) == ADDENDUM_RAW_SHA
        for path, digest in addendum['evidence_sha256'].items():
            bind(path, digest)
    else:
        assert combined['passed'] is True and not combined['foreign_imports']
    assert all(combined[key] is True for key in (
        'full', 'source_unchanged', 'scope_complete',
        'baseline_file_test_counts_exact', 'baseline_backup_sources_unchanged',
        'generated_bootstrap_unchanged', 'supplemental_reference_unchanged',
        'supplemental_file_test_counts_exact', 'original_base_loaded_from_actual_workspace'))
    assert combined['returncode'] == 0 and combined['regression_scope'] == 'full_combined'
    assert combined['baseline'] == str(BASE) and combined['baseline_sha256'] == BASE_SHA
    assert combined['baseline_source_backup_manifest_sha256'] == BACKUP_MANIFEST_SHA
    assert combined['runner_sha256'] == EXPECTED_RUNNER_SHA
    assert combined['guard_source_sha256'] == GUARD_SOURCE_SHA
    assert combined['baseline_count_comparison'] == 'exact_per_file'
    assert combined['baseline_scope_files_compared'] == 282
    for key in ('baseline_count_mismatches', 'supplemental_count_mismatches',
                'unassigned_testcases', 'external_boundary_attempts',
                'changed_during_run'):
        assert not combined[key], key
    for key in ('live_ros_contacted', 'motion_commands_sent', 'temperature_queried',
                'deployed', 'physical_grasp_verified'):
        assert combined[key] is False, key

    scope, sources = combined['test_scope'], combined['source_sha256']
    assert len(scope) == len(set(scope)) == combined['test_scope_files'] == 286
    assert scope == original_scope + list(SUPPLEMENTAL) + [NEW_TEST]
    assert set(sources) == set(original) | set(EXTRA) | {NEW_TEST}
    assert len(sources) == combined['source_count'] == 1034
    assert combined['new_tests'] == [NEW_TEST]
    assert combined['extra_dependencies_sha256'] == EXTRA
    assert combined['supplemental_existing_tests'] == list(SUPPLEMENTAL)
    assert combined['supplemental_expected_test_counts'] == SUPPLEMENTAL
    changes = {name for name, digest in original.items() if sources[name] != digest}
    assert changes == {PRODUCTION, FIXTURE}
    assert set(combined['changed_from_gen41']) == changes
    assert all(sources[name] == digest for name, digest in EXPECTED_PREPARED_SOURCE_SHA256.items())
    assert all(sources[name] == digest for name, digest in EXTRA.items())
    assert all(sources[name] == original[name] for name in original_scope)
    assert all(sha(W/name) == digest for name, digest in sources.items())
    new_on_disk = {str(path.relative_to(W)) for root in (
        W/'src/alicia_grasp_modes/tests', W/'src/alicia_flexible_grasp_supervisor/tests')
        for path in root.glob('test_gen42*.py')}
    assert new_on_disk == {NEW_TEST}

    expected_counts = dict(original_counts, **SUPPLEMENTAL)
    expected_counts[NEW_TEST] = 21
    summary = combined['summary']
    assert summary == dict(tests=8216, failures=0, errors=0, skipped=0)
    counts = combined['test_case_counts_by_file']
    assert counts == expected_counts and set(counts) == set(scope)
    xml = bind(combined_path.parent/'pytest.xml', combined['xml_sha256'])
    xml_root = ET.parse(xml).getroot()
    actual = {key: sum(int(suite.get(key, 0)) for suite in xml_root.iter('testsuite'))
              for key in ('tests', 'failures', 'errors', 'skipped')}
    assert actual == summary
    xml_counts = dict.fromkeys(scope, 0)
    for case in xml_root.iter('testcase'):
        name = case.get('file', '')
        assert name in xml_counts, (name, case.get('name'))
        assert case.find('failure') is None and case.find('error') is None
        assert case.find('skipped') is None
        xml_counts[name] += 1
    assert xml_counts == counts and all(xml_counts.values())
    if addendum is not None:
        comparison_cases = [case for case in xml_root.iter('testcase')
            if case.get('file') == COMPARISON_TEST]
        assert len(comparison_cases) == original_counts[COMPARISON_TEST] == 38
        oracle_counts = {name: sum(case.get('name', '').split('[')[0] == name
            for case in comparison_cases) for name in ORACLE_TEST_FUNCTIONS}
        assert oracle_counts == ORACLE_TEST_FUNCTIONS and sum(oracle_counts.values()) == 21
    bind(combined_path.parent/'pytest.log', combined['log_sha256'])

    declaration = json.loads(bind(combined_path.parent/'scope.json').read_text())
    assert declaration['full'] is True and declaration['tests'] == scope
    assert declaration['source_sha256'] == sources and declaration['source_count'] == 1034
    assert declaration['baseline'] == str(BASE) and declaration['baseline_sha256'] == BASE_SHA
    assert declaration['runner_sha256'] == EXPECTED_RUNNER_SHA
    assert declaration['original_file_test_counts'] == original_counts
    assert declaration['inherited_pytest_deselection_disabled'] is True
    assert declaration['extra_dependencies_sha256'] == EXTRA
    assert declaration['new_tests'] == [NEW_TEST]
    assert declaration['supplemental_count_reference'] == combined['supplemental_count_reference']
    reference = combined['supplemental_count_reference']
    assert reference['expected_test_counts_by_file'] == SUPPLEMENTAL
    assert reference['original_GEN41_282_scope_included_these_files'] is False
    for path, digest in reference['input_sha256'].items():
        bind(path, digest)
    assert declaration['generated_bootstrap_sha256'] == combined['generated_bootstrap_sha256']
    for path, digest in combined['generated_bootstrap_sha256'].items():
        assert not Path(path).is_absolute() and '..' not in Path(path).parts
        bind(combined_path.parent/path, digest)

    provenance_path = bind(combined_path.parent/'module_provenance.json',
                           combined['module_provenance_sha256'])
    provenance = json.loads(provenance_path.read_text())
    expected_base = str((W/PRODUCTION).resolve(strict=True))
    assert any(row['path'] == expected_base and row['sha256'] == sources[PRODUCTION]
               for row in provenance.values())
    for label, row in provenance.items():
        path = Path(row['path']).resolve(strict=True)
        if addendum is not None and label == HISTORICAL_LABEL:
            assert row == expected_historical[HISTORICAL_LABEL]
        else:
            assert inside(path, W/'src') or inside(path, W/'devel'), str(path)
        bind(path, row['sha256'])
    if addendum is not None:
        assert provenance[HISTORICAL_LABEL] == expected_historical[HISTORICAL_LABEL]
        current_surface = 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/multiview_surface.py'
        assert provenance['alicia_flexible_grasp.vision.multiview_surface'] == dict(
            path=str((W/current_surface).resolve(strict=True)), sha256=sources[current_surface])
    boundary_path = combined_path.parent/'boundary_attempts.jsonl'
    if boundary_path.exists():
        assert not bind(boundary_path).read_text().splitlines()

    review = json.loads(bind(args.review).read_text())
    assert review['reviewed'] is True and review['release_review_complete'] is True
    assert not review['blocking_findings']
    reviewed = review['reviewed_source_sha256']
    required_review_sources = set(EXPECTED_PREPARED_SOURCE_SHA256) | {
        str(RUNNER.relative_to(W)), str(Path(__file__).resolve().relative_to(W))}
    assert required_review_sources <= set(reviewed)
    for name, digest in reviewed.items():
        path = Path(name)
        assert not path.is_absolute() and '..' not in path.parts
        bind(W/name, digest)
    assert review['evidence_sha256'].get(str(combined_path)) == sha(combined_path)
    if addendum_path is not None:
        assert review['evidence_sha256'].get(str(addendum_path)) == sha(addendum_path)
    for path, digest in review['evidence_sha256'].items():
        bind(path, digest)
    bind(__file__)
    assert all(sha(path) == digest for path, digest in evidence.items())
    assert all(sha(W/name) == digest for name, digest in sources.items())
    assert all(sha(BACKUP/name) == digest for name, digest in original.items())
    storage(output)
    release = dict(schema_version=1,
        kind='gen42_geometry_compute_lock_handoff', sealed_ns=time.time_ns(),
        passed=True, reviewed=True, installed=True,
        installed_semantics='Exact disk sources passed full offline regression and completed review; this helper installs nothing and starts no node.',
        source_sha256=sources, source_count=1034, test_scope=scope, test_scope_files=286,
        test_summary=summary, test_case_counts_by_file=counts,
        changed_production_sources=[PRODUCTION], modified_existing_tests=[FIXTURE],
        modified_original282_tests=[], new_tests=[NEW_TEST], extra_dependencies_sha256=EXTRA,
        supplemental_existing_tests=list(SUPPLEMENTAL), supplemental_test_case_counts=SUPPLEMENTAL,
        original282_exact_counts_preserved=True, original282_case_count=8168,
        regression_scope='full_combined', baseline_release=str(BASE), baseline_release_sha256=BASE_SHA,
        actual_regression=str(combined_path), combined_review=str(Path(args.review).resolve()),
        raw_regression_passed=combined['passed'],
        raw_regression_foreign_imports=combined['foreign_imports'],
        acceptance_addendum=str(addendum_path) if addendum_path is not None else None,
        acceptance_addendum_sha256=sha(addendum_path) if addendum_path is not None else None,
        historical_comparator_metadata_accepted=(addendum is not None),
        evidence_sha256=evidence, runtime_deployment_pending=True,
        pending_validation_boundary='Offline callbacks, coherent cache CAS, common preparation and simulated task acceptance are verified only within the reported fixtures. Real robot movement, fresh execution checks, collision checks and physical grasp still require runtime validation.',
        physical_grasp_verified=False, deployed=False, live_ros_contacted=False,
        motion_commands_sent=False, temperature_queried=False)
    with output.open('x') as stream:
        json.dump(release, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    print(json.dumps(dict(release=str(output), sha256=sha(output), summary=summary)))


if __name__ == '__main__':
    main()
