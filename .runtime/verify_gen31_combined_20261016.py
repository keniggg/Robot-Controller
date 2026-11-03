"""Offline regression for a reviewed, explicitly scoped GEN31 box uncertainty and observation planning repair."""
import argparse
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
E = M / '实验记录/Robot-Controller'
N = E / 'ros_log/full_latest_gen22_restart_20261015T003700Z'
BASE = N / 'verification/gen30_release_manifest.json'
BASE_SHA = 'c6554435fafe0589e73910e928439204654cd6e7edba993ad7ecd7df509f837f'
BACKUP = W / '.runtime/gen31_baseline_source'
BACKUP_SHA = '7d577e7fdbd2ecf29162fa9d6ad05494d6b654e6c2ffc71c04c9a2da8f1dcc78'
TEST_ROOTS = (W / 'src/alicia_grasp_modes/tests',
              W / 'src/alicia_flexible_grasp_supervisor/tests')


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def write(path, value):
    with Path(path).open('x') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())


def guard(output):
    assert M.is_mount() and not os.statvfs(M).f_flag & os.ST_RDONLY
    output.resolve().relative_to((N / 'verification').resolve())
    assert output.parent.stat().st_dev == M.stat().st_dev
    config = read(E / 'grasp_records/archive_config.json')
    assert config['upload_enabled'] is False and config['delete_enabled'] is False
    assert read(E / 'maintenance/storage_control.json')['archive_policy'] == 'local_only'
    assert shutil.disk_usage(E).free >= config['min_free_bytes']


def new_tests():
    return sorted(str(p.relative_to(W)) for root in TEST_ROOTS
                  for p in root.glob('test_gen31*.py'))


def checked_paths(values):
    assert isinstance(values, list) and len(values) == len(set(values))
    for name in values:
        path = Path(name)
        assert not path.is_absolute() and '..' not in path.parts
        assert path.parts[0] == 'src' and (W / path).is_file()
        (W / path).resolve().relative_to(W)
    return set(values)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scope', type=Path, required=True)
    parser.add_argument('--scope-sha256', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    guard(output)
    assert not output.exists()
    assert sha(BASE) == BASE_SHA and sha(BACKUP / 'manifest.json') == BACKUP_SHA
    baseline = read(BASE)
    assert baseline['passed'] and baseline['source_count'] == 976
    assert baseline['test_scope_files'] == 238 and baseline['test_summary']['tests'] == 7152
    originals = read(BACKUP / 'manifest.json')
    assert originals == baseline['source_sha256']
    assert all(sha(BACKUP / name) == digest for name, digest in originals.items())
    assert sha(args.scope) == args.scope_sha256
    plan = read(args.scope)
    assert plan['baseline_sha256'] == BASE_SHA
    production = checked_paths(plan['production'])
    changed_tests = checked_paths(plan['changed_tests'])
    resources = checked_paths(plan['new_resources'])
    assert production and not (production & changed_tests)
    assert not resources & set(originals)
    # A baseline-hashed helper can contain tests without having been explicitly
    # collected by GEN30. Every changed test file must now be collected too.
    assert changed_tests <= set(originals)
    assert all('/tests/' in name and Path(name).name.startswith('test_')
               for name in changed_tests)
    changed = {name for name, digest in originals.items() if sha(W / name) != digest}
    assert changed == production | changed_tests, ('Unexpected change scope', sorted(changed))
    added = new_tests()
    assert added and not set(added) & set(originals)
    assert sorted(plan['new_tests']) == added
    scope = list(dict.fromkeys(baseline['test_scope'] + sorted(changed_tests) + added))
    sources = dict(originals)
    for name in sorted(changed | resources | set(added)):
        sources[name] = sha(W / name)
    evidence = dict(plan['evidence_sha256'])
    evidence[str(args.scope.resolve())] = args.scope_sha256
    evidence[str(BACKUP / 'manifest.json')] = BACKUP_SHA
    for path, digest in evidence.items():
        assert sha(path) == digest
    runner_sha = sha(__file__)
    output.mkdir()
    (output / 'tmp').mkdir()
    (output / 'ros').mkdir()
    patch = dict(baseline=str(BASE), baseline_sha256=BASE_SHA,
        production=sorted(production), changed_tests=sorted(changed_tests),
        new_tests=added, new_resources=sorted(resources), purpose=plan['purpose'],
        evidence_sha256=evidence,
        files={name: dict(baseline_sha256=originals.get(name), sha256=sources[name])
               for name in sorted(changed | resources | set(added))})
    write(output / 'patch_manifest.json', patch)
    diff = ''.join(''.join(difflib.unified_diff(
        (BACKUP / name).read_text().splitlines(True), (W / name).read_text().splitlines(True),
        fromfile='GEN30/' + name, tofile='GEN31/' + name)) for name in sorted(changed))
    (output / 'production_and_fixture.diff').write_text(diff)
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', PYTHONNOUSERSITE='1',
        PYTEST_DISABLE_PLUGIN_AUTOLOAD='1', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1',
        ROS_MASTER_URI='http://127.0.0.1:1', TMPDIR=str(output / 'tmp'), ROS_LOG_DIR=str(output / 'ros'))
    env['PYTHONPATH'] = ':'.join(str(W / name) for name in (
        'src/alicia_flexible_grasp_supervisor/tests', 'src/alicia_grasp_modes/tests',
        'src/alicia_grasp_modes/src', 'src/alicia_flexible_grasp_supervisor/src',
        'devel/lib/python3/dist-packages')) + ':' + env.get('PYTHONPATH', '')
    env['ROS_PACKAGE_PATH'] = str(W / 'src') + ':' + env.get('ROS_PACKAGE_PATH', '')
    command = [sys.executable, '-m', 'pytest', '-q', *scope, '-p', 'no:cacheprovider',
        '--tb=short', '--basetemp=' + str(output / 'pytest_tmp'),
        '--junitxml=' + str(output / 'pytest.xml'), '-o', 'junit_family=legacy']
    print(json.dumps(dict(started=True, source_count=len(sources), test_files=len(scope), output=str(output))), flush=True)
    started = time.monotonic()
    with (output / 'pytest.log').open('x') as stream:
        run = subprocess.run(command, cwd=W, env=env, stdout=stream, stderr=subprocess.STDOUT)
    root = ET.parse(output / 'pytest.xml').getroot()
    summary = {key: sum(int(s.get(key, 0)) for s in root.iter('testsuite'))
               for key in ('tests', 'failures', 'errors', 'skipped')}
    counts, unassigned = dict.fromkeys(scope, 0), []
    for case in root.iter('testcase'):
        name = case.get('file', '')
        if name in counts:
            counts[name] += 1
        else:
            unassigned.append(dict(file=name, classname=case.get('classname'), name=case.get('name')))
    changed_during = [name for name, digest in sources.items() if sha(W / name) != digest]
    unchanged = not changed_during and sha(BASE) == BASE_SHA and sha(__file__) == runner_sha
    unchanged = unchanged and new_tests() == added and all(sha(path) == digest for path, digest in evidence.items())
    retained = all(counts[name] >= count for name, count in baseline['test_case_counts_by_file'].items())
    complete = not unassigned and retained and all(v > 0 for v in counts.values()) and sum(counts.values()) == summary['tests']
    passed = run.returncode == 0 and unchanged and complete and summary['tests'] > 7152 and all(summary[k] == 0 for k in ('failures', 'errors', 'skipped'))
    result = dict(passed=passed, summary=summary, returncode=run.returncode,
        elapsed_sec=time.monotonic() - started, source_sha256=sources, source_count=len(sources),
        source_unchanged=unchanged, changed_source_paths=changed_during,
        test_scope=scope, test_scope_files=len(scope), test_case_counts_by_file=counts,
        scope_complete=complete, baseline_file_test_counts_retained=retained,
        unassigned_testcases=unassigned, command=command, baseline=str(BASE), baseline_sha256=BASE_SHA,
        full_scope_baseline=str(BASE), full_scope_baseline_sha256=BASE_SHA,
        patch=str(output / 'patch_manifest.json'), patch_sha256=sha(output / 'patch_manifest.json'),
        scope_plan=str(args.scope.resolve()), scope_plan_sha256=args.scope_sha256,
        log_sha256=sha(output / 'pytest.log'), xml_sha256=sha(output / 'pytest.xml'), runner_sha256=runner_sha,
        regression_scope='full_combined', production_worktree_tested=True,
        live_ros_contacted=False, motion_commands_sent=False, temperature_queried=False)
    guard(output)
    write(output / 'result.json', result)
    print(json.dumps(dict(passed=passed, summary=summary, result=str(output / 'result.json'))), flush=True)
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
