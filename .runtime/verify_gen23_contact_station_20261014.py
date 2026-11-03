"""Run only the GEN23 contact handoff delta; preserve the sealed GEN22 baseline."""
import argparse
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
BASE = E / 'ros_log/full_latest_gen18_restart_20261014T012216Z/verification/gen22_release_manifest.json'
BASE_SHA256 = 'a99e400819346d3552f37eb2923fae870e6a3826d57451657a52ea25cc3c6da2'
PRODUCTION = ('src/alicia_grasp_modes/src/alicia_grasp_modes/contact_station_retirement.py',)
NEW_TESTS = ('src/alicia_grasp_modes/tests/test_gen23_contact_station_param_types.py',)
RELATED = (
    'src/alicia_grasp_modes/tests/test_contact_station_retirement.py',
    'src/alicia_grasp_modes/tests/test_frozen_station_retirement_history.py',
    'src/alicia_grasp_modes/tests/test_gen11_contact_remote_receipt.py',
    'src/alicia_grasp_modes/tests/test_gen11_contact_compute_admission_independent.py',
    'src/alicia_flexible_grasp_supervisor/tests/test_contact_pregrasp_task.py',
)
SCOPE = RELATED + NEW_TESTS
# Four unchanged related files contributed these cases to the sealed baseline.
# The supervisor file is in the baseline source map, but was not run there.
MINIMUM_CASES_BY_FILE = dict(zip(SCOPE, (74, 22, 44, 40, 1, 1)))
MINIMUM_DELTA_CASES = sum(MINIMUM_CASES_BY_FILE.values())


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def storage_guard(path):
    path = Path(path).resolve()
    assert M.is_mount() and not os.statvfs(M).f_flag & os.ST_RDONLY
    assert N / 'verification' in path.parents
    ancestor = path
    while not ancestor.exists():
        ancestor = ancestor.parent
    assert ancestor.stat().st_dev == M.stat().st_dev
    config = read(E / 'grasp_records/archive_config.json')
    assert config['upload_enabled'] is False and config['delete_enabled'] is False
    assert read(E / 'maintenance/storage_control.json')['archive_policy'] == 'local_only'
    return config


def save(path, value):
    storage_guard(path)
    with Path(path).open('x') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())


def baseline_release():
    assert sha(BASE) == BASE_SHA256
    baseline = read(BASE)
    assert all(baseline[k] is True for k in ('passed', 'reviewed', 'installed'))
    assert baseline['source_count'] == len(baseline['source_sha256']) == 941
    assert baseline['test_summary'] == dict(tests=6341, failures=0, errors=0, skipped=0)
    assert set(PRODUCTION + RELATED) <= set(baseline['source_sha256'])
    assert not set(NEW_TESTS) & set(baseline['source_sha256'])
    return baseline


def current_source_map(baseline):
    sources = dict(baseline['source_sha256'])
    for name, digest in sources.items():
        assert not Path(name).is_absolute() and '..' not in Path(name).parts
        if name not in PRODUCTION:
            assert sha(W / name) == digest, name
    for name in PRODUCTION + NEW_TESTS:
        sources[name] = sha(W / name)
    assert len(sources) == 942
    return sources


def xml_report(path):
    root = ET.parse(path).getroot()
    summary = {key: sum(int(row.get(key, 0)) for row in root.iter('testsuite'))
               for key in ('tests', 'failures', 'errors', 'skipped')}
    counts = dict.fromkeys(SCOPE, 0)
    unassigned = []
    for case in root.iter('testcase'):
        names = case.get('classname', '').split('.')
        owners = [name for name in SCOPE if Path(name).stem in names]
        if len(owners) == 1:
            counts[owners[0]] += 1
        else:
            unassigned.append(dict(classname=case.get('classname'), name=case.get('name')))
    return summary, counts, unassigned


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output', type=Path, required=True)
    a = ap.parse_args()
    out = a.output.resolve()
    config = storage_guard(out)
    assert out.parent.is_dir() and not out.exists()
    assert shutil.disk_usage(E).free >= config['min_free_bytes']
    baseline = baseline_release()
    sources = current_source_map(baseline)
    runner_sha256 = sha(__file__)
    out.mkdir()
    for name in ('tmp', 'ros'):
        (out / name).mkdir()
    patch_path = out / 'patch_manifest.json'
    save(patch_path, dict(baseline=str(BASE), baseline_sha256=BASE_SHA256,
        allowed_production=list(PRODUCTION), new_tests=list(NEW_TESTS),
        unchanged_baseline_source_count=940,
        files={name: dict(baseline_sha256=baseline['source_sha256'].get(name),
                         sha256=sources[name]) for name in PRODUCTION + NEW_TESTS}))
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', PYTHONNOUSERSITE='1',
        PYTEST_DISABLE_PLUGIN_AUTOLOAD='1', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1',
        ROS_MASTER_URI='http://127.0.0.1:1', TMPDIR=str(out / 'tmp'), ROS_LOG_DIR=str(out / 'ros'))
    env['PYTHONPATH'] = ':'.join(str(W / p) for p in (
        'src/alicia_flexible_grasp_supervisor/tests', 'src/alicia_grasp_modes/tests',
        'src/alicia_grasp_modes/src', 'src/alicia_flexible_grasp_supervisor/src',
        'devel/lib/python3/dist-packages')) + ':' + env.get('PYTHONPATH', '')
    env['ROS_PACKAGE_PATH'] = str(W / 'src') + ':' + env.get('ROS_PACKAGE_PATH', '')
    command = [sys.executable, '-m', 'pytest', '-q', *SCOPE, '-p', 'no:cacheprovider',
        '--tb=short', '--basetemp=' + str(out / 'pytest_tmp'), '--junitxml=' + str(out / 'pytest.xml')]
    print(json.dumps(dict(started=True, source_count=len(sources), test_files=len(SCOPE),
        regression_scope='delta_only', historical_baseline_tests=6341,
        minimum_delta_cases=MINIMUM_DELTA_CASES, output=str(out))), flush=True)
    started = time.monotonic()
    with (out / 'pytest.log').open('x') as log:
        completed = subprocess.run(command, cwd=W, env=env, stdout=log, stderr=subprocess.STDOUT)
    summary = dict(tests=0, failures=0, errors=0, skipped=0)
    counts, unassigned, xml_error = dict.fromkeys(SCOPE, 0), [], None
    try:
        summary, counts, unassigned = xml_report(out / 'pytest.xml')
    except (OSError, ET.ParseError, ValueError) as exc:
        xml_error = repr(exc)
    changes = []
    for name, digest in sources.items():
        try:
            same = sha(W / name) == digest
        except OSError:
            same = False
        if not same:
            changes.append(name)
    unchanged = not changes and sha(__file__) == runner_sha256 and sha(BASE) == BASE_SHA256
    scope_complete = (not unassigned and sum(counts.values()) == summary['tests']
        and all(counts[name] >= minimum for name, minimum in MINIMUM_CASES_BY_FILE.items()))
    passed = (completed.returncode == 0 and unchanged and xml_error is None and scope_complete
        and summary['tests'] >= MINIMUM_DELTA_CASES
        and all(summary[key] == 0 for key in ('failures', 'errors', 'skipped')))
    result = dict(passed=passed, summary=summary, returncode=completed.returncode,
        elapsed_sec=time.monotonic() - started, source_sha256=sources, source_count=len(sources),
        source_unchanged=unchanged, changed_source_paths=changes,
        test_scope=list(SCOPE), test_scope_files=len(SCOPE), regression_scope='delta_only',
        test_case_counts_by_file=counts, minimum_cases_by_file=MINIMUM_CASES_BY_FILE,
        minimum_delta_cases=MINIMUM_DELTA_CASES, scope_complete=scope_complete,
        unassigned_testcases=unassigned, xml_error=xml_error, command=command,
        baseline=str(BASE), baseline_sha256=BASE_SHA256,
        baseline_test_summary=baseline['test_summary'], baseline_test_scope=baseline['test_scope'],
        baseline_test_scope_files=baseline['test_scope_files'], baseline_tests_rerun=False,
        patch=str(patch_path), patch_sha256=sha(patch_path), log_sha256=sha(out / 'pytest.log'),
        xml_sha256=sha(out / 'pytest.xml') if (out / 'pytest.xml').is_file() else None,
        runner_sha256=runner_sha256, production_worktree_tested=True,
        live_ros_contacted=False, motion_commands_sent=False, temperature_queried=False)
    save(out / 'result.json', result)
    print(json.dumps(dict(passed=passed, delta_summary=summary,
        historical_baseline_summary=baseline['test_summary'], result=str(out / 'result.json'))), flush=True)
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
