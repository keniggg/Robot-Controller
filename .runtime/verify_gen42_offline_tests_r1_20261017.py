"""Verify the actual GEN42 workspace offline; every run artifact stays on SanStand.

This helper never installs sources, seals a release, starts ROS, or moves hardware.
Full scope is the sealed GEN41 282-file scope plus the new GEN42 test file.
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
M = Path('/media/zhuyupei/SanStand')
E = M/'实验记录/Robot-Controller'
OUTPUT_ROOT = E/'ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
BASE = E/'ros_log/full_latest_gen36_restart_20261017T012037Z/verification/gen41_release_manifest.json'
BASE_SHA = 'b98c25da01b77582bff4567a791afed8b8c908fcfe391b3b84601276c9ed706d'
BACKUP = W/'.runtime/gen41_baseline_source'
BACKUP_MANIFEST_SHA = '61c2533a61070079fc58e18fade39eb4fa5ff673828ca1b3663d587b4c6994d2'
GUARD_SOURCE = W/'.runtime/verify_gen41_offline_tests_20261017.py'
GUARD_SOURCE_SHA = '0c7c13321f4eed998d54f3ff49456179c7c4c28deb193e9ab646113554de21c9'
EXTRA_DEPENDENCIES = {
    'tools/run_fresh_grasp6d_after_alignment.py':
        '8d750485b6166823486c2933579ef3faef0640a4a323c6b6c2554a42a7a0c45b',
}
ALLOWED_BASELINE_CHANGES = {
    'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py',
}
TEST_ROOTS = (
    'src/alicia_grasp_modes/tests',
    'src/alicia_flexible_grasp_supervisor/tests',
)
EXPECTED_GEN42_NEW_TEST = 'src/alicia_grasp_modes/tests/test_gen42_geometry_lock_handoff.py'


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def save(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())


def storage(output):
    assert M.is_mount(), 'SanStand is unavailable; no system-disk fallback'
    output.relative_to(OUTPUT_ROOT.resolve(strict=True))
    assert output != OUTPUT_ROOT.resolve()
    assert output.parent.stat().st_dev == M.stat().st_dev
    config = json.loads((E/'grasp_records/archive_config.json').read_text())
    policy = json.loads((E/'maintenance/storage_control.json').read_text())
    assert config['upload_enabled'] is False and config['delete_enabled'] is False
    assert policy['archive_policy'] == 'local_only'
    assert shutil.disk_usage(E).free >= config['min_free_bytes']


def relative_source(name):
    path = Path(name)
    return (path.as_posix() == name and not path.is_absolute()
            and '..' not in path.parts and (W/path).is_file())


def within(path, root):
    # The ROS Noetic workspace uses Python 3.8 (before Path.is_relative_to).
    try:
        Path(path).relative_to(root)
        return True
    except ValueError:
        return False


def frozen_python_guard():
    assert sha(GUARD_SOURCE) == GUARD_SOURCE_SHA
    tree = ast.parse(GUARD_SOURCE.read_text())
    writes = [node for node in ast.walk(tree) if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute) and node.func.attr == 'write_text'
        and isinstance(node.func.value, ast.Name) and node.func.value.id == 'bootstrap']
    assert len(writes) == 1
    source = ast.literal_eval(writes[0].args[0])
    assert 'sys.addaudithook(device_open)' in source
    return source


IMPORT_AUDIT = '''import hashlib,json,os,sys
from pathlib import Path
def pytest_sessionfinish(session,exitstatus):
    rows={}
    for name,module in list(sys.modules.items()):
        path=getattr(module,'__file__',None)
        if path and ('remote_grasp6d' in name or name.startswith('alicia_grasp_modes')
                     or name.startswith('alicia_flexible_grasp')):
            resolved=Path(path).resolve()
            rows[name]={'path':str(resolved),
                'sha256':hashlib.sha256(resolved.read_bytes()).hexdigest()}
    Path(os.environ['GEN42_MODULE_PROVENANCE']).write_text(json.dumps(rows,indent=2)+'\\n')
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--test', action='append', default=[],
                        help='Run complete focused test files; no implicit deselection')
    parser.add_argument('--full', action='store_true')
    args = parser.parse_args()
    output = args.output.resolve()
    storage(output)
    assert not output.exists()
    assert sha(BASE) == BASE_SHA
    assert sha(BACKUP/'manifest.json') == BACKUP_MANIFEST_SHA
    baseline = json.loads(BASE.read_text())
    backup = json.loads((BACKUP/'manifest.json').read_text())
    original = baseline['source_sha256']
    expected_counts = baseline['test_case_counts_by_file']
    assert baseline['passed'] is True and baseline['reviewed'] is True
    assert baseline['source_count'] == len(original) == 1032
    assert baseline['test_summary'] == dict(tests=8168, failures=0, errors=0, skipped=0)
    assert baseline['test_scope_files'] == len(baseline['test_scope']) == 282
    assert len(set(baseline['test_scope'])) == len(expected_counts) == 282
    assert set(baseline['test_scope']) == set(expected_counts)
    assert sum(expected_counts.values()) == 8168
    assert backup['release_sha256'] == BASE_SHA and backup['source_sha256'] == original
    assert all(sha(BACKUP/name) == digest for name, digest in original.items())
    assert all(sha(W/name) == digest for name, digest in EXTRA_DEPENDENCIES.items())
    new_tests = sorted(str(path.relative_to(W)) for root in TEST_ROOTS
                       for path in (W/root).glob('test_gen42*.py'))
    assert not set(new_tests) & set(original)
    if args.full:
        assert not args.test, '--full already owns the exact combined file scope'
        assert new_tests == [EXPECTED_GEN42_NEW_TEST], 'Unexpected GEN42 test-file scope'
        scope = baseline['test_scope'] + new_tests
    else:
        scope = args.test
    assert scope and len(scope) == len(set(scope))
    assert all(relative_source(name) and name in set(original) | set(new_tests)
               and '/tests/test_' in name for name in scope)
    sources = {name: sha(W/name) for name in original}
    sources.update(EXTRA_DEPENDENCIES)
    sources.update({name: sha(W/name) for name in new_tests})
    changed = sorted(name for name, digest in original.items() if sources[name] != digest)
    assert set(changed) <= ALLOWED_BASELINE_CHANGES, 'Unexpected GEN41 source/test mutation'
    runner_sha = sha(__file__)
    guard_text = frozen_python_guard()
    output.mkdir()
    for name in ('tmp', 'ros', 'bootstrap'):
        (output/name).mkdir()
    bootstrap = output/'bootstrap/sitecustomize.py'
    import_plugin = output/'bootstrap/gen42_import_audit.py'
    bootstrap.write_text(guard_text)
    import_plugin.write_text(IMPORT_AUDIT)
    generated = {str(path.relative_to(output)): sha(path) for path in (bootstrap, import_plugin)}
    save(output/'scope.json', dict(full=args.full, tests=scope,
        source_sha256=sources, source_count=len(sources), changed_from_gen41=changed,
        new_tests=new_tests, extra_dependencies_sha256=EXTRA_DEPENDENCIES,
        baseline=str(BASE), baseline_sha256=BASE_SHA,
        baseline_source_backup_manifest_sha256=BACKUP_MANIFEST_SHA,
        runner_sha256=runner_sha, guard_source_sha256=GUARD_SOURCE_SHA,
        generated_bootstrap_sha256=generated, original_file_test_counts=expected_counts,
        inherited_pytest_deselection_disabled=True))
    (output/'changes_from_gen41.diff').write_text(''.join(''.join(difflib.unified_diff(
        (BACKUP/name).read_text().splitlines(True), (W/name).read_text().splitlines(True),
        fromfile='GEN41/'+name, tofile='GEN42/'+name)) for name in changed))
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', PYTHONNOUSERSITE='1',
        PYTEST_DISABLE_PLUGIN_AUTOLOAD='1', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1',
        ROS_MASTER_URI='http://127.0.0.1:1', ROS_LOG_DIR=str(output/'ros'),
        TMPDIR=str(output/'tmp'), OFFLINE_BOUNDARY_LOG=str(output/'boundary_attempts.jsonl'),
        GEN42_MODULE_PROVENANCE=str(output/'module_provenance.json'))
    env.pop('PYTEST_ADDOPTS', None)
    env.pop('PYTEST_PLUGINS', None)
    env['PYTHONPATH'] = ':'.join(map(str, [output/'bootstrap'] + [W/name for name in (
        *TEST_ROOTS, 'src/alicia_grasp_modes/src',
        'src/alicia_flexible_grasp_supervisor/src', 'devel/lib/python3/dist-packages')]))
    env['PYTHONPATH'] += ':' + os.environ.get('PYTHONPATH', '')
    env['ROS_PACKAGE_PATH'] = str(W/'src') + ':/opt/ros/noetic/share'
    command = [sys.executable, '-m', 'pytest', '-q', '--rootdir='+str(W), *scope,
        '-p', 'no:cacheprovider', '-p', 'gen42_import_audit', '--tb=short',
        '--basetemp='+str(output/'pytest_tmp'), '--junitxml='+str(output/'pytest.xml'),
        '-o', 'junit_family=legacy']
    print(json.dumps(dict(started=True, output=str(output), full=args.full,
                         test_files=len(scope), source_count=len(sources))), flush=True)
    started = time.monotonic()
    with (output/'pytest.log').open('x') as stream:
        run = subprocess.run(command, cwd=W, env=env, stdout=stream, stderr=subprocess.STDOUT)
    summary = {name: 0 for name in ('tests', 'failures', 'errors', 'skipped')}
    counts = dict.fromkeys(scope, 0)
    unassigned = []
    if (output/'pytest.xml').exists():
        tree = ET.parse(output/'pytest.xml').getroot()
        for suite in tree.iter('testsuite'):
            for name in summary:
                summary[name] += int(suite.get(name, 0))
        for case in tree.iter('testcase'):
            name = case.get('file', '')
            if name in counts:
                counts[name] += 1
            else:
                unassigned.append(dict(file=name, name=case.get('name')))
    provenance_path = output/'module_provenance.json'
    provenance = json.loads(provenance_path.read_text()) if provenance_path.exists() else {}
    expected_base = str((W/next(iter(ALLOWED_BASELINE_CHANGES))).resolve())
    original_loaded = any(row['path'] == expected_base
        and row['sha256'] == sources[next(iter(ALLOWED_BASELINE_CHANGES))]
        for row in provenance.values())
    foreign_imports = {name: row for name, row in provenance.items()
        if not (within(row['path'], W/'src') or within(row['path'], W/'devel'))}
    altered = [name for name, digest in sources.items() if sha(W/name) != digest]
    backup_unchanged = all(sha(BACKUP/name) == digest for name, digest in original.items())
    generated_unchanged = all(sha(output/name) == digest for name, digest in generated.items())
    unchanged = (not altered and backup_unchanged and generated_unchanged
        and sha(BASE) == BASE_SHA and sha(BACKUP/'manifest.json') == BACKUP_MANIFEST_SHA
        and sha(__file__) == runner_sha and sha(GUARD_SOURCE) == GUARD_SOURCE_SHA)
    expected_in_scope = {name: value for name, value in expected_counts.items() if name in counts}
    count_mismatches = {name: dict(expected=value, actual=counts.get(name, 0))
        for name, value in expected_in_scope.items() if counts.get(name, 0) != value}
    retained = not count_mismatches and (not args.full or len(expected_in_scope) == 282)
    attempts = ((output/'boundary_attempts.jsonl').read_text().splitlines()
                if (output/'boundary_attempts.jsonl').exists() else [])
    complete = not unassigned and all(counts.values()) and sum(counts.values()) == summary['tests']
    passed = (run.returncode == 0 and unchanged and retained and complete
        and original_loaded and not foreign_imports and not attempts and summary['tests'] > 0
        and all(summary[name] == 0 for name in ('failures', 'errors', 'skipped')))
    result = dict(passed=passed, summary=summary, returncode=run.returncode,
        elapsed_sec=time.monotonic()-started, full=args.full, test_scope=scope,
        test_scope_files=len(scope), test_case_counts_by_file=counts, scope_complete=complete,
        baseline_file_test_counts_exact=retained, baseline_count_comparison='exact_per_file',
        baseline_scope_files_compared=len(expected_in_scope), baseline_count_mismatches=count_mismatches,
        unassigned_testcases=unassigned, source_sha256=sources, source_count=len(sources),
        source_unchanged=unchanged, changed_during_run=altered, changed_from_gen41=changed,
        baseline_backup_sources_unchanged=backup_unchanged, generated_bootstrap_unchanged=generated_unchanged,
        baseline=str(BASE), baseline_sha256=BASE_SHA,
        baseline_source_backup_manifest_sha256=BACKUP_MANIFEST_SHA,
        new_tests=new_tests, extra_dependencies_sha256=EXTRA_DEPENDENCIES,
        original_base_loaded_from_actual_workspace=original_loaded, foreign_imports=foreign_imports,
        module_provenance_sha256=sha(provenance_path) if provenance_path.exists() else None,
        external_boundary_attempts=attempts,
        external_guard_scope='Python socket network and CPython physical-device-open audit; native SDK factories require inert test fixtures',
        runner_sha256=runner_sha, guard_source_sha256=GUARD_SOURCE_SHA,
        generated_bootstrap_sha256=generated, command=command, log_sha256=sha(output/'pytest.log'),
        xml_sha256=sha(output/'pytest.xml') if (output/'pytest.xml').exists() else None,
        regression_scope='full_combined' if args.full else 'focused_complete_files',
        live_ros_contacted=False, motion_commands_sent=False, temperature_queried=False,
        sources_installed=False, deployed=False, physical_grasp_verified=False)
    storage(output)
    save(output/'result.json', result)
    print(json.dumps(dict(passed=passed, summary=summary, output=str(output))), flush=True)
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
