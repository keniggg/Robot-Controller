"""Draft: install reviewed GEN54 bytes while the accepted GEN53 runtime is idle.

No ROS/motion API is called. Installed-W full regression is required before
any later reload. This file is prepared for root execution, not run on import.
"""
import argparse
import hashlib
import json
import os
import time
from pathlib import Path

W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
E = M/'实验记录/Robot-Controller'
N = E/'ros_log/full_latest_gen41_restart_20261018T011222Z'
V = N/'verification'
CONTROL = E/'ros_log/full_latest_frozen_no_skin_20261012T021050Z/continuous_authorization_20261012/control.json'
CONTROL_EXPECTED_SHA = '135af0f40ab4dbfc30b4d75be7c44c963263c2498484da271f056d8104966833'
BASE = W/'.runtime/gen54_baseline_source/runner_manifest.json'
BASE_SHA = 'e0ea9529ff6afdff8a6794578f1e64d996cdb124ee1adf3e6732e38e44984748'
RELEASE = V/'gen53_release_manifest.json'
RELEASE_SHA = 'e9f07af55bfac1ae9fd7562d93f04748b6b2e68139f7bab3e0f645cc46ef255f'
DEPLOYMENT = N/'gen53_runtime/gen53_runtime_deployment.json'
DEPLOYMENT_SHA = 'd3e8c419d724e7eb941403c2bf6d47e0bc5f4cb77dbb549f93158d6c0d16202f'
READINESS_HELPER = W/'.runtime/read_gen54_preinstall_readiness_20261018.py'
PRODUCTION = {'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py', 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/contact_height_placement.py', 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/gripper_geometry.py', 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/tabletop_geometry_candidates.py'}
NEW_PRODUCTION = set()
# Exact three new file paths are frozen; case counts still require actual collection evidence.
EXPECTED_NEW_TEST_FILES = 3
EXPECTED_NEW_TEST_PATHS = frozenset(('src/alicia_flexible_grasp_supervisor/tests/test_gen54_collision_contract.py', 'src/alicia_flexible_grasp_supervisor/tests/test_gen54_height_window.py', 'src/alicia_flexible_grasp_supervisor/tests/test_gen54_open_aperture_refinement.py'))
SUPPLEMENTAL_TESTS = frozenset((
    'src/alicia_flexible_grasp_supervisor/tests/test_gripper_geometry.py',
    'src/alicia_flexible_grasp_supervisor/tests/test_tabletop_geometry_candidates.py',
    'src/alicia_grasp_modes/tests/test_contact_tilt_replay.py',
    'src/alicia_grasp_modes/tests/test_contact_placement_clearance.py',
))
# Exact three reviewed old-test updates; all other original test bytes stay fixed.
REVIEWED_OLD_TEST_CHANGES = [{'path': 'src/alicia_flexible_grasp_supervisor/tests/test_gripper_geometry.py', 'baseline_sha256': '0b2b44da8962476a81a4cdfcd9655b5e94cd20160eb6a334cd0766d9ddd3d45f', 'candidate_sha256': '4fe13a1336b611b4417ea2944ad1020d052b7586d994ed829f562415116dd06b', 'review_evidence': {'path': '/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification/gen54_collision_geometry_r1/near_limit_versioned_expectation_review.json', 'sha256': '431c590a5d7304a29a2a9c80316a7122529ac6c7381d41297c275b28e19b8a2a'}}, {'path': 'src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py', 'baseline_sha256': '6957b2d808da254dface00bd025791362a44c98e93765980de25a74bb1e68638', 'candidate_sha256': 'b6b4c8004caadc7826969568bd81a5964d65b87299d5836898cd72ea4f1d9c77', 'review_evidence': {'path': '/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification/gen54_old_expectation_review_r1/recorded_expectation_review.json', 'sha256': 'f1884d032e508c43e268d3a2ba2929b9127199a2643ac1bf7661b5066b52a1de'}}, {'path': 'src/alicia_flexible_grasp_supervisor/tests/test_contact_height_window_proposals.py', 'baseline_sha256': '331388555feb1323fdb8013e4745113826662d4013796d0462e2c0393be87a11', 'candidate_sha256': 'd8ed14b42ccb3a1a8681e7791490097143f7a14d97ca7bb392fbf3f4e4b52201', 'review_evidence': {'path': '/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification/gen54_geometry_pipeline_r1/height_cardinality_only_old_test_review_r1.json', 'sha256': '55df9ba96aed22b77be4db213859b41c59cdea3d90b0ffc853a85471109cbf1b'}}]
SUPPLEMENTAL_CAD_INPUTS = {'src/arm-mujoco/synriard/urdf/Alicia_D_v5_6/Alicia_D_v5_6_gripper_50mm.urdf': '7e08a01f1a6cf3b45ed78a26d3a9f5939e2bb6bc33f6bc01032db097882edaeb', 'src/arm-mujoco/synriard/meshes/Alicia_D_v5_6/gripper_50mm/Link6.STL': 'cff46c5522d1a869b0397965adb885a7f4c62c5b137b409d8df605aab3cc3d5d', 'src/arm-mujoco/synriard/meshes/Alicia_D_v5_6/gripper_50mm/Link7.STL': 'd546b7ab908c74281a41261412089a2f83f7519c2444739afe8ffff46450d19d', 'src/arm-mujoco/synriard/meshes/Alicia_D_v5_6/gripper_50mm/Link8.STL': '5e5b548734269b731c1d60502c742e3d70d5a97a1e8316db523e2232b4c5043b', 'src/real-arm/alicia_d_descriptions/urdf/alicia_duo_with_gripper.urdf': '890592a24b8ad818d995e69beb670ea4254bf84f589615bfac3c8562f62449b7', 'src/real-arm/alicia_d_descriptions/meshes/Link7.STL': 'ff006151147d25c431d868fc6a0f46b96275690d30526d13c741ffe51ac6fcea', 'src/real-arm/alicia_d_descriptions/meshes/Link8.STL': 'fd49b1a4c9ecf39854d0de1ee3645239aae2427cca6ce7a24b195bf041c6098e', 'src/real-arm/alicia_d_moveit/config/alicia_d_descriptions.srdf': 'ac14d9a4de083d30444b3c8e762bc166a61382ec334ac455a93ac792ae6a42c2'}
SUPPLEMENTAL_ORIGINAL_COLLECTION = N/'verification/gen54_old_geometry_collect_r1/result.json'
SUPPLEMENTAL_ORIGINAL_COLLECTION_SHA = '8658aaa84ba970a2b93fda982a4527740e2aedef722df7739563b50365fee29e'



def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_bytes())


def write(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())


def process_matches(row):
    fields = (Path('/proc')/str(row['pid'])/'stat').read_text().rsplit(')', 1)[1].split()
    return (fields[0] != 'Z' and fields[19] == row['start_ticks']
            and int(fields[1]) == row['parent_pid'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('manifest', 'focus', 'review', 'readiness'):
        parser.add_argument('--'+name, type=Path, required=True)
        parser.add_argument('--'+name+'-sha256', required=True)
    parser.add_argument('--expected-control-revision', type=int, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    assert type(EXPECTED_NEW_TEST_FILES) is int and EXPECTED_NEW_TEST_FILES > 0, 'GEN54 candidate test-file inventory is not yet bound'
    assert isinstance(REVIEWED_OLD_TEST_CHANGES, list) and REVIEWED_OLD_TEST_CHANGES, 'GEN54 reviewed old-test exception inventory is not yet bound'
    assert M.is_mount() and args.expected_control_revision == 56
    output = args.output.resolve()
    assert V in output.parents and not output.exists()
    refs = {}
    for name in ('manifest', 'focus', 'review', 'readiness'):
        path = getattr(args, name).resolve(strict=True)
        digest = getattr(args, name+'_sha256')
        assert E in path.parents and sha(path) == digest
        refs[str(path)] = digest
    assert sha(BASE) == BASE_SHA and sha(RELEASE) == RELEASE_SHA
    assert sha(DEPLOYMENT) == DEPLOYMENT_SHA
    baseline = read(BASE)['source_sha256']
    assert len(baseline) == 1069 and read(RELEASE)['source_sha256'] == baseline
    manifest, focus, review, ready = [read(getattr(args, name))
        for name in ('manifest', 'focus', 'review', 'readiness')]
    candidate = Path(manifest['workspace_root']).resolve(strict=True)
    assert candidate == W/'.runtime/gen54_release_candidate_r1'
    assert manifest['kind'] == 'GEN54_frozen_offline_candidate'
    assert manifest.get('variant', 'candidate') == 'candidate'
    assert manifest.get('full_scope_ready') is True
    source = manifest['source_sha256']
    approved_rows = manifest.get('approved_old_test_changes', [])
    assert approved_rows == REVIEWED_OLD_TEST_CHANGES
    approved = {row['path']: row for row in approved_rows}
    assert len(approved) == len(approved_rows) and not set(approved) & PRODUCTION
    for relative, row in approved.items():
        assert relative in baseline and '/tests/test_' in relative
        assert row['baseline_sha256'] == baseline[relative]
        assert row['candidate_sha256'] == source[relative] != baseline[relative]
        evidence = Path(row['review_evidence']['path']).resolve(strict=True)
        assert V in evidence.parents and sha(evidence) == row['review_evidence']['sha256']
        assert review['dependency_sha256'].get(str(evidence)) == row['review_evidence']['sha256']
        refs[str(evidence)] = row['review_evidence']['sha256']
    assert sha(SUPPLEMENTAL_ORIGINAL_COLLECTION) == SUPPLEMENTAL_ORIGINAL_COLLECTION_SHA
    original_supplement = read(SUPPLEMENTAL_ORIGINAL_COLLECTION)
    original_supplement_cases = [tuple(case) for case in original_supplement['collected_case_identities']]
    assert original_supplement['mode'] == 'collect' and original_supplement['passed'] is True
    assert original_supplement['test_bodies_executed'] is False
    assert len(original_supplement_cases) == len(set(original_supplement_cases)) == 206
    refs[str(SUPPLEMENTAL_ORIGINAL_COLLECTION)] = SUPPLEMENTAL_ORIGINAL_COLLECTION_SHA
    for relative, digest in SUPPLEMENTAL_CAD_INPUTS.items():
        assert relative not in source
        for root in (W, candidate):
            path = root/relative
            assert path.resolve(strict=True) == path and sha(path) == digest
            refs[str(path)] = digest
    ADDED = {row['path'] for row in manifest['new_tests'] + manifest.get('new_fixture_sources', []) + manifest.get('new_production_sources', [])}
    assert len(manifest['new_tests']) == EXPECTED_NEW_TEST_FILES
    assert {row['path'] for row in manifest['new_tests']} == EXPECTED_NEW_TEST_PATHS
    supplemental = manifest.get('supplemental_existing_tests', [])
    assert len(supplemental) == len(SUPPLEMENTAL_TESTS)
    assert {row['path'] for row in supplemental} == SUPPLEMENTAL_TESTS
    assert all(row['path'] in baseline and row['sha256'] == (
        approved[row['path']]['candidate_sha256'] if row['path'] in approved else baseline[row['path']])
        and row['path'] not in read(BASE)['test_scope'] for row in supplemental)
    assert sorted(tuple(case) for row in supplemental for case in row['case_identities']) == sorted(original_supplement_cases)
    extra_rows = supplemental + manifest['new_tests']
    assert len({row['path'] for row in extra_rows}) == len(extra_rows)
    for row in extra_rows:
        cases = [tuple(case) for case in row['case_identities']]
        assert type(row['expected_count']) is int and row['expected_count'] > 0
        assert len(cases) == len(set(cases)) == row['expected_count']
        assert all(len(case) == 3 and case[0] == row['path'] for case in cases)
        evidence = Path(row['collection_evidence']['path']).resolve(strict=True)
        assert E in evidence.parents and sha(evidence) == row['collection_evidence']['sha256']
        collection = read(evidence)
        assert collection['mode'] == 'collect' and collection['passed'] is True
        assert collection['source_unchanged'] is True and collection['test_bodies_executed'] is False
        assert collection['source_sha256'] == source and collection['full_regression_passed'] is False
        assert not collection['external_boundary_attempts'] and not collection['foreign_or_inconsistent_imports']
        assert sorted(cases) == sorted(tuple(case) for case in collection['collected_case_identities']
            if case[0] == row['path'])
        refs[str(evidence)] = sha(evidence)
    assert {row['path'] for row in manifest.get('new_production_sources', [])} == NEW_PRODUCTION
    for row in manifest.get('new_production_sources', []):
        evidence = Path(row['review_evidence']['path']).resolve(strict=True)
        assert E in evidence.parents and sha(evidence) == row['review_evidence']['sha256']
        assert read(evidence)['passed'] is True
        refs[str(evidence)] = sha(evidence)
    assert all(('/tests/test_gen54_' in name and name.endswith('.py'))
        or ('/tests/fixtures/gen54_' in name and name.endswith(('.json', '.json.gz')))
        for name in ADDED - NEW_PRODUCTION)
    assert len(source) == 1069 + len(ADDED) and set(source)-set(baseline) == ADDED
    assert {path for path in baseline if source[path] != baseline[path]} == PRODUCTION | set(approved)
    assert all(not Path(path).is_absolute() and '..' not in Path(path).parts for path in source)
    assert all(sha(candidate/path) == digest for path, digest in source.items())
    assert all(sha(W/path) == digest and sha(BASE.parent/path) == digest
        for path, digest in baseline.items())
    assert all(not (W/path).exists() for path in ADDED)
    assert all(focus.get(key) is True for key in
        ('passed', 'scope_complete', 'test_bodies_executed', 'source_unchanged'))
    assert focus['mode'] == 'focus' and focus['source_sha256'] == source
    assert focus['summary']['tests'] > 0
    assert all(focus['summary'][key] == 0 for key in ('failures', 'errors', 'skipped'))
    assert not focus['external_boundary_attempts'] and not focus['foreign_or_inconsistent_imports']
    assert review['passed'] is True and review['source_sha256'] == source
    assert ready['passed'] is True and ready['manual_mode'] is True and ready['task_active'] is False
    assert ready['deployment_processes_unchanged'] is True
    assert ready['selection']['generation'] == 2 and ready['selection']['mode'] == 'unknown'
    assert str(ready['selection']['stamp_ns']) == '1792374537080906391'
    assert ready['selection']['strategy'] == 'observe_return'
    assert ready['epoch_ns'] == 1792373681250409915 and ready['source_count'] == 1069
    assert 0 <= time.time_ns()-ready['recorded_ns'] < 120_000_000_000
    assert Path(ready['release']).resolve(strict=True) == RELEASE.resolve(strict=True)
    assert Path(ready['deployment']).resolve(strict=True) == DEPLOYMENT.resolve(strict=True)
    assert ready['release_sha256'] == RELEASE_SHA and ready['deployment_sha256'] == DEPLOYMENT_SHA
    assert ready['helper_sha256'] == sha(READINESS_HELPER)
    control = read(CONTROL)
    assert control['state'] == 'active' and type(control['revision']) is int
    assert control['revision'] == args.expected_control_revision == 56
    assert sha(CONTROL) == CONTROL_EXPECTED_SHA
    assert control == ready['control_snapshot'] and sha(CONTROL) == ready['control_sha256']
    refs.update({str(CONTROL): sha(CONTROL), str(BASE): BASE_SHA,
        str(RELEASE): RELEASE_SHA, str(DEPLOYMENT): DEPLOYMENT_SHA,
        str(READINESS_HELPER): sha(READINESS_HELPER)})
    assert all(sha(path) == digest for path, digest in refs.items())
    deployment = read(DEPLOYMENT)
    processes = dict(deployment['after'], **deployment['preserved_after'])
    assert len(deployment['after']) == 4 and len(deployment['preserved_after']) == 7
    assert ready['node_identities'] == processes and all(process_matches(row) for row in processes.values())
    output.mkdir()
    write(output/'plan.json', dict(source_sha256=source, baseline_sha256=baseline,
        evidence_sha256=refs, changed_paths=sorted(PRODUCTION | set(approved) | ADDED),
        control_snapshot=control, full_regression_passed=False, runtime_reloaded=False,
        motion_commands_sent=False, created_ns=time.time_ns()))
    for index, relative in enumerate(sorted(PRODUCTION | set(approved) | ADDED)):
        target = W/relative
        temporary = target.with_name(target.name+'.gen54.install.tmp')
        assert not temporary.exists() and all(sha(path) == digest for path, digest in refs.items())
        mode = target.stat().st_mode & 0o777 if target.exists() else 0o644
        with temporary.open('xb') as stream:
            stream.write((candidate/relative).read_bytes())
            stream.flush()
            os.fsync(stream.fileno())
        assert sha(temporary) == source[relative]
        os.chmod(temporary, mode)
        os.replace(temporary, target)
        write(output/('installed_%02d.json' % index), dict(path=relative,
            sha256=sha(target), recorded_ns=time.time_ns()))
    assert all(sha(W/path) == digest for path, digest in source.items())
    assert all(sha(path) == digest for path, digest in refs.items())
    result = dict(passed=True, source_count=len(source), source_sha256=source,
        changed_files={path: source[path] for path in sorted(PRODUCTION | set(approved) | ADDED)},
        original_baseline=str(BASE.parent), evidence_sha256=refs, control_snapshot=control,
        full_regression_passed=False, runtime_reloaded=False, motion_commands_sent=False,
        temperature_queried=False, recorded_ns=time.time_ns(), helper_sha256=sha(__file__))
    write(output/'result.json', result)
    print(json.dumps(dict(passed=True, result=str(output/'result.json'), sha256=sha(output/'result.json'))))


if __name__ == '__main__':
    main()
