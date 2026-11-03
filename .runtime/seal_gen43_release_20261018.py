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
RUNNER = W/'.runtime/verify_gen43_offline_tests_20261018.py'
RUNNER_SHA = '86e4a35b7e122ba81e18d02f8875c4a1db26769c45c35c6274f16dad0e55617c'
CHANGE = 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/cuboid_rgb_interval.py'
NEW = (
    'src/alicia_flexible_grasp_supervisor/tests/test_gen43_top_mask_contract.py',
    'src/alicia_grasp_modes/tests/test_gen43_top_mask_handoff.py',
)
EXPECTED_SOURCE = {
    CHANGE: '50e00a5b751ba0470f98c4215922bb6f05471784a364181c297ad5f617751d7f',
    NEW[0]: '3b6722a0514f280aa7aaba28450a18a7e2dd8503359460c8b0e8d692abdb4cf6',
    NEW[1]: '6b9d31328431558be8f9b79ca987febf9d33f2dde4381feb73be542d2cd0c4cb',
}
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
        assert result['changed_from_gen42'] == [CHANGE]
        assert all(result['source_sha256'][p] == original[p] for p in original if p != CHANGE)
        assert all(result['source_sha256'][p] == h for p, h in EXPECTED_SOURCE.items())
        root = W if full else CANDIDATE
        assert Path(result['workspace']).resolve(strict=True) == root.resolve(strict=True)
        scope = result['test_scope']
        assert len(scope) == len(set(scope)) == result['test_scope_files']
        assert set(scope) <= set(old_scope) | set(NEW) and set(NEW) <= set(scope)
        if full:
            assert scope == old_scope + list(NEW) and len(scope) == 288
        summary = result['summary']
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
        bind(path.parent/'pytest.log', result['log_sha256'])
        declaration = json.loads(bind(path.parent/'scope.json').read_text())
        assert declaration['full'] is full and declaration['tests'] == scope
        assert declaration['source_sha256'] == result['source_sha256']
        assert declaration['baseline_sha256'] == BASE_SHA and declaration['runner_sha256'] == RUNNER_SHA
        assert declaration['original_test_counts'] == old_counts
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
    expected_counts.update({p: focus['test_case_counts_by_file'][p] for p in NEW})
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
    for path in (combined_path, focus_path, baseline_replay, candidate_replay):
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
        changed_production_sources=[CHANGE], modified_existing_tests=[], new_tests=list(NEW),
        original286_exact_counts_preserved=True, original286_case_count=8216,
        regression_scope='full_combined', baseline_release=str(BASE), baseline_release_sha256=BASE_SHA,
        actual_regression=str(combined_path), focused_regression=str(focus_path),
        baseline_source_replay=str(baseline_replay), candidate_source_replay=str(candidate_replay),
        combined_review=str(review_path), evidence_sha256=evidence,
        accepted_exact_historical_oracle=combined['accepted_exact_historical_oracle'],
        baseline_history_unchanged=True,
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
