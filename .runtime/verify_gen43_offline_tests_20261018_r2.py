"""GEN43 complete-file offline regression with exact GEN42 source/count retention.

Allows one cuboid coverage source change, two new tests and one explicitly
proved historical model-digest fixture update. All original case counts remain
exact; 285 original test files remain byte-identical. No installation,
deployment, ROS master, physical device or motion is performed by this runner.
"""
import argparse
import ast
import difflib
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

W = Path(__file__).resolve().parents[1]
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
V = E/'ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
BASE = V/'gen42_release_manifest.json'
BASE_SHA = '4ce3b5142b18e9968deff461d39a202f0fe6f91572394744abad67d29c4810dc'
BACKUP = W/'.runtime/gen42_source_before_gen43'
CANDIDATE = W/'.runtime/gen43_mask_candidate'
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
ROOTS = ('src/alicia_flexible_grasp_supervisor/tests', 'src/alicia_grasp_modes/tests')
GUARD = W/'.runtime/verify_gen41_offline_tests_20261017.py'
GUARD_SHA = '0c7c13321f4eed998d54f3ff49456179c7c4c28deb193e9ab646113554de21c9'
IMPORT_SOURCE = W/'.runtime/verify_gen42_offline_tests_20261017.py'
IMPORT_SHA = '226b7f0cb282be162ea3ccd5c6e236e4127a8e6b3d9f0769772169962e383342'
ORACLE_LABEL = 'alicia_flexible_grasp.vision._gen38_registration_test_baseline'
ORACLE = W/'.runtime/gen38_baseline_source/src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/multiview_surface.py'
ORACLE_SHA = '369a2947114c6c3953c439a7937cf26971e337c015e65abf96825720bc5c1ba8'
ORACLE_TEST = 'src/alicia_flexible_grasp_supervisor/tests/test_gen39_registration_transform.py'
ORACLE_TEST_SHA = 'fcd9fe03951fc4761f1faaf486b1ccf979583cb1cd07be39f9912845a5bd7c71'


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def save(p, value):
    with p.open('x') as f:
        json.dump(value, f, ensure_ascii=False, indent=2, allow_nan=False)
        f.write('\n'); f.flush(); os.fsync(f.fileno())


def inside(path, root):
    try:
        Path(path).relative_to(root)
        return True
    except ValueError:
        return False


def storage(output):
    mount = E.parent.parent
    assert mount.is_mount() and inside(output, V) and output != V
    assert output.parent.stat().st_dev == mount.stat().st_dev
    config = json.loads((E/'grasp_records/archive_config.json').read_text())
    policy = json.loads((E/'maintenance/storage_control.json').read_text())
    assert config['upload_enabled'] is False and config['delete_enabled'] is False
    assert policy['archive_policy'] == 'local_only'
    assert shutil.disk_usage(E).free >= config['min_free_bytes']


def bootstrap_sources():
    assert sha(GUARD) == GUARD_SHA and sha(IMPORT_SOURCE) == IMPORT_SHA
    tree = ast.parse(GUARD.read_text())
    calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
        and isinstance(n.func, ast.Attribute) and n.func.attr == 'write_text'
        and isinstance(n.func.value, ast.Name) and n.func.value.id == 'bootstrap']
    assert len(calls) == 1
    guard = ast.literal_eval(calls[0].args[0])
    assignments = [n for n in ast.parse(IMPORT_SOURCE.read_text()).body
        if isinstance(n, ast.Assign) and any(isinstance(t, ast.Name)
            and t.id == 'IMPORT_AUDIT' for t in n.targets)]
    assert len(assignments) == 1
    audit = ast.literal_eval(assignments[0].value)
    return guard, audit


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
    parser.add_argument('--workspace', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--test', action='append', default=[])
    parser.add_argument('--full', action='store_true')
    args = parser.parse_args()
    root, output = args.workspace.resolve(), args.output.resolve()
    assert root == W, 'r2 measures the actual installed workspace only'
    storage(output); assert not output.exists()
    assert sha(BASE) == BASE_SHA
    baseline = json.loads(BASE.read_text())
    assert baseline['passed'] and baseline['reviewed'] and baseline['installed']
    original = baseline['source_sha256']
    counts_before = baseline['test_case_counts_by_file']
    assert len(original) == 1034 and len(counts_before) == 286
    assert sum(counts_before.values()) == 8216
    assert set(counts_before) == set(baseline['test_scope'])
    assert json.loads((BACKUP/'manifest.json').read_text())['source_sha256'] == original
    assert all(sha(BACKUP/p) == h for p, h in original.items())
    compatibility, compatibility_dependencies = compatibility_inputs(original)
    assert counts_before[COMPAT] == 4
    new_tests = sorted(str(p.relative_to(root)) for folder in ROOTS
        for p in (root/folder).glob('test_gen43*.py'))
    assert set(new_tests) == set(NEW)
    if args.full:
        assert root == W and not args.test and set(new_tests) == set(NEW)
        scope = baseline['test_scope']+list(NEW)
    else:
        assert tuple(args.test) == FOCUS_SCOPE, 'focus requires the three complete pinned files'
        scope = args.test
    assert scope and len(scope) == len(set(scope))
    assert all(p in set(counts_before) | set(new_tests) for p in scope)
    sources = {p: sha(root/p) for p in list(original)+new_tests}
    changed = [p for p in original if sources[p] != original[p]]
    assert set(changed) == {CHANGE, COMPAT}
    assert all(sources[p] == original[p] for p in counts_before if p != COMPAT)
    assert all(sources[p] == h for p, h in EXPECTED_SOURCE.items())
    # An isolated calculation must not modify the running workspace.
    live_before = {p: sha(W/p) for p in original}
    if root != W:
        assert live_before == original
    runner_sha = sha(__file__)
    guard, audit = bootstrap_sources()
    output.mkdir()
    for p in ('tmp', 'ros', 'bootstrap'): (output/p).mkdir()
    (output/'bootstrap/sitecustomize.py').write_text(guard)
    (output/'bootstrap/gen42_import_audit.py').write_text(audit)
    generated = {str(p.relative_to(output)): sha(p) for p in (output/'bootstrap').iterdir()}
    save(output/'scope.json', dict(workspace=str(root), full=args.full, tests=scope,
        source_sha256=sources, changed_from_gen42=changed, baseline_sha256=BASE_SHA,
        original_test_counts=counts_before, original_sources_unchanged=False,
        changed_production_sources=[CHANGE], modified_existing_tests=[COMPAT],
        original285_test_files_bytes_unchanged=True, original286_exact_counts_preserved=True,
        historical_fixture_compatibility=compatibility,
        compatibility_proof_sha256=COMPAT_PROOF_SHA,
        compatibility_evidence_sha256=compatibility_dependencies,
        exact_historical_oracle=dict(label=ORACLE_LABEL, path=str(ORACLE), sha256=ORACLE_SHA,
            test=ORACLE_TEST, test_sha256=ORACLE_TEST_SHA), runner_sha256=runner_sha))
    (output/'changes.diff').write_text(''.join(''.join(difflib.unified_diff(
        (BACKUP/p).read_text().splitlines(True), (root/p).read_text().splitlines(True),
        fromfile='GEN42/'+p, tofile='GEN43/'+p)) for p in changed))
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', PYTHONNOUSERSITE='1',
        PYTEST_DISABLE_PLUGIN_AUTOLOAD='1', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1',
        ROS_MASTER_URI='http://127.0.0.1:1', ROS_LOG_DIR=str(output/'ros'),
        TMPDIR=str(output/'tmp'), OFFLINE_BOUNDARY_LOG=str(output/'boundary_attempts.jsonl'),
        GEN42_MODULE_PROVENANCE=str(output/'module_provenance.json'))
    for key in ('PYTEST_ADDOPTS', 'PYTEST_PLUGINS'): env.pop(key, None)
    paths = [output/'bootstrap']+[root/p for p in (*ROOTS,
        'src/alicia_grasp_modes/src', 'src/alicia_flexible_grasp_supervisor/src')]
    paths += [W/'devel/lib/python3/dist-packages', Path('/opt/ros/noetic/lib/python3/dist-packages')]
    env['PYTHONPATH'] = ':'.join(map(str, paths))
    env['ROS_PACKAGE_PATH'] = str(root/'src')+':/opt/ros/noetic/share'
    command = [sys.executable, '-m', 'pytest', '-q', '--rootdir='+str(root), *scope,
        '-p', 'no:cacheprovider', '-p', 'gen42_import_audit', '--tb=short',
        '--basetemp='+str(output/'pytest_tmp'), '--junitxml='+str(output/'pytest.xml'),
        '-o', 'junit_family=legacy']
    print(json.dumps(dict(started=True, output=str(output), test_files=len(scope))), flush=True)
    start = time.monotonic()
    with (output/'pytest.log').open('x') as f:
        run = subprocess.run(command, cwd=root, env=env, stdout=f, stderr=subprocess.STDOUT)
    summary = dict.fromkeys(('tests', 'failures', 'errors', 'skipped'), 0)
    counts, unknown = dict.fromkeys(scope, 0), []
    if (output/'pytest.xml').exists():
        tree = ET.parse(output/'pytest.xml').getroot()
        for suite in tree.iter('testsuite'):
            for k in summary: summary[k] += int(suite.get(k, 0))
        for case in tree.iter('testcase'):
            name = case.get('file', '')
            if name in counts: counts[name] += 1
            else: unknown.append(dict(file=name, case=case.get('name')))
    provenance_path = output/'module_provenance.json'
    provenance = json.loads(provenance_path.read_text()) if provenance_path.exists() else {}
    invalid, oracle_rows = {}, {}
    for label, row in provenance.items():
        p = Path(row['path'])
        if inside(p, root/'src'):
            relative = str(p.relative_to(root))
            valid = relative in sources and row['sha256'] == sources[relative]
        elif inside(p, W/'devel'):
            valid = sha(p) == row['sha256']
        else:
            valid = (label == ORACLE_LABEL and p == ORACLE and row['sha256'] == ORACLE_SHA
                and ORACLE_TEST in scope and sources[ORACLE_TEST] == ORACLE_TEST_SHA
                and sha(ORACLE) == ORACLE_SHA)
            if valid: oracle_rows[label] = row
        if not valid: invalid[label] = row
    expected_changed_path = str(root/CHANGE)
    source_loaded = any(row['path'] == expected_changed_path and row['sha256'] == sources[CHANGE]
                        for row in provenance.values())
    altered = [p for p, h in sources.items() if sha(root/p) != h]
    stable = (not altered and all(sha(W/p) == h for p, h in live_before.items())
        and all(sha(BACKUP/p) == h for p, h in original.items())
        and all(sha(output/p) == h for p, h in generated.items())
        and sha(BASE) == BASE_SHA and sha(__file__) == runner_sha
        and sha(GUARD) == GUARD_SHA and sha(IMPORT_SOURCE) == IMPORT_SHA
        and all(sha(p) == h for p, h in compatibility_dependencies.items()))
    mismatches = {p: dict(expected=counts_before[p], actual=n) for p, n in counts.items()
                  if p in counts_before and n != counts_before[p]}
    attempts_path = output/'boundary_attempts.jsonl'
    attempts = attempts_path.read_text().splitlines() if attempts_path.exists() else []
    expected_counts = dict(counts_before) if args.full else {COMPAT: counts_before[COMPAT]}
    expected_counts.update(EXPECTED_NEW_COUNTS)
    complete = (not unknown and counts == expected_counts
        and sum(counts.values()) == summary['tests'] == (8258 if args.full else 46))
    passed = (run.returncode == 0 and stable and complete and not mismatches
        and source_loaded and not invalid and not attempts
        and all(summary[k] == 0 for k in ('failures', 'errors', 'skipped')))
    result = dict(passed=passed, summary=summary, returncode=run.returncode,
        elapsed_sec=time.monotonic()-start, full=args.full, workspace=str(root),
        source_sha256=sources, source_count=len(sources), source_unchanged=stable,
        test_scope=scope, test_scope_files=len(scope), test_case_counts_by_file=counts,
        baseline_count_mismatches=mismatches, scope_complete=complete, unassigned_testcases=unknown,
        baseline_sha256=BASE_SHA, changed_from_gen42=changed, changed_during_run=altered,
        changed_production_sources=[CHANGE], modified_existing_tests=[COMPAT],
        original285_test_files_bytes_unchanged=True, original286_exact_counts_preserved=True,
        historical_fixture_compatibility=compatibility,
        compatibility_proof_sha256=COMPAT_PROOF_SHA,
        compatibility_evidence_sha256=compatibility_dependencies,
        changed_source_loaded=source_loaded, foreign_or_inconsistent_imports=invalid,
        accepted_exact_historical_oracle=oracle_rows, external_boundary_attempts=attempts,
        runner_sha256=runner_sha, generated_bootstrap_sha256=generated,
        module_provenance_sha256=sha(provenance_path) if provenance_path.exists() else None,
        log_sha256=sha(output/'pytest.log'),
        xml_sha256=sha(output/'pytest.xml') if (output/'pytest.xml').exists() else None,
        live_ros_contacted=False, hardware_contacted=False, motion_commands_sent=False,
        temperature_queried=False, deployed=False, physical_grasp_verified=False)
    storage(output); save(output/'result.json', result)
    print(json.dumps(dict(passed=passed, summary=summary, output=str(output))), flush=True)
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
