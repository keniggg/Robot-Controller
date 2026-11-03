"""GEN43 complete-file offline regression with exact GEN42 source/count retention.

Allows one cuboid coverage source change and two new tests. No installation,
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--test', action='append', default=[])
    parser.add_argument('--full', action='store_true')
    args = parser.parse_args()
    root, output = args.workspace.resolve(), args.output.resolve()
    assert root in (W, CANDIDATE)
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
    new_tests = sorted(str(p.relative_to(root)) for folder in ROOTS
        for p in (root/folder).glob('test_gen43*.py'))
    assert set(new_tests) <= set(NEW)
    if args.full:
        assert root == W and not args.test and set(new_tests) == set(NEW)
        scope = baseline['test_scope']+list(NEW)
    else:
        scope = args.test
    assert scope and len(scope) == len(set(scope))
    assert all(p in set(counts_before) | set(new_tests) for p in scope)
    sources = {p: sha(root/p) for p in list(original)+new_tests}
    changed = [p for p in original if sources[p] != original[p]]
    assert set(changed) <= {CHANGE}
    assert all(sources[p] == original[p] for p in counts_before)
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
        original_test_counts=counts_before, original_sources_unchanged=True,
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
        and sha(GUARD) == GUARD_SHA and sha(IMPORT_SOURCE) == IMPORT_SHA)
    mismatches = {p: dict(expected=counts_before[p], actual=n) for p, n in counts.items()
                  if p in counts_before and n != counts_before[p]}
    attempts_path = output/'boundary_attempts.jsonl'
    attempts = attempts_path.read_text().splitlines() if attempts_path.exists() else []
    complete = not unknown and all(counts.values()) and sum(counts.values()) == summary['tests']
    passed = (run.returncode == 0 and stable and complete and not mismatches
        and source_loaded and not invalid and not attempts
        and all(summary[k] == 0 for k in ('failures', 'errors', 'skipped')))
    result = dict(passed=passed, summary=summary, returncode=run.returncode,
        elapsed_sec=time.monotonic()-start, full=args.full, workspace=str(root),
        source_sha256=sources, source_count=len(sources), source_unchanged=stable,
        test_scope=scope, test_scope_files=len(scope), test_case_counts_by_file=counts,
        baseline_count_mismatches=mismatches, scope_complete=complete, unassigned_testcases=unknown,
        baseline_sha256=BASE_SHA, changed_from_gen42=changed, changed_during_run=altered,
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
