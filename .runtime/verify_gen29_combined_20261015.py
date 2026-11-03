"""Offline full GEN28 regression plus scoped supplemental preview handoff repair."""
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
BASE = N / 'verification/gen28_release_manifest.json'
BASE_SHA = '2ae560cd671ae402366eaa41a2235905f51bd3b5cc6f43afd379df1f842bddfd'
BACKUP = W / '.runtime/gen29_baseline_source'
PRODUCTION = {
    'src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_task.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_remote.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_task.py',
    'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py',
}
NEW_RESOURCES = {'src/alicia_grasp_modes/src/alicia_grasp_modes/station_supplemental_admission.py'}
TEST_ROOT = W / 'src/alicia_grasp_modes/tests'
EVIDENCE = (
    'verification/gen28_combined_r1/result.json',
    'verification/gen28_final_independent_review/review.json',
    'gen28_deployment/result.json',
    'gen28_attempt_1_after_manual/result.json',
    'verification/gen29_recorded_handoff_diagnosis/result.json',
)



def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def new_tests():
    return sorted(str(path.relative_to(W)) for path in TEST_ROOT.glob('test_gen29*.py'))


def guard(output):
    assert M.is_mount() and not os.statvfs(M).f_flag & os.ST_RDONLY
    output.resolve().relative_to((N/'verification').resolve())
    assert output.parent.stat().st_dev == M.stat().st_dev
    config = read(E/'grasp_records/archive_config.json')
    assert config['upload_enabled'] is False and config['delete_enabled'] is False
    assert read(E/'maintenance/storage_control.json')['archive_policy'] == 'local_only'
    assert shutil.disk_usage(E).free >= config['min_free_bytes']


def write(path, value):
    with path.open('x') as file:
        json.dump(value, file, indent=2, ensure_ascii=False, allow_nan=False)
        file.write('\n')
        file.flush()
        os.fsync(file.fileno())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    output = args.output.resolve()
    guard(output)
    assert not output.exists()
    assert sha(BASE) == BASE_SHA
    baseline = read(BASE)
    assert baseline['passed'] and baseline['source_count'] == 965
    assert baseline['test_scope_files'] == 230 and baseline['test_summary']['tests'] == 7067
    originals = read(BACKUP/'manifest.json')
    assert PRODUCTION <= set(originals)
    for name, digest in originals.items():
        assert baseline['source_sha256'][name] == digest == sha(BACKUP/name)
    changed = sorted(name for name, digest in baseline['source_sha256'].items() if sha(W/name) != digest)
    assert set(changed) == PRODUCTION, ('Unexpected GEN29 change scope', changed)
    added = new_tests()
    resources = sorted(NEW_RESOURCES)
    assert not NEW_RESOURCES & set(baseline['source_sha256'])
    assert added and not set(added) & set(baseline['source_sha256'])
    scope = list(dict.fromkeys(baseline['test_scope'] + added))
    sources = dict(baseline['source_sha256'])
    for name in changed + added + resources:
        sources[name] = sha(W/name)
    evidence = {str(N/name): sha(N/name) for name in EVIDENCE}
    evidence[str(BACKUP/'manifest.json')] = sha(BACKUP/'manifest.json')
    runner_sha = sha(__file__)
    patch = dict(baseline=str(BASE), baseline_sha256=BASE_SHA,
        production=sorted(PRODUCTION), changed_tests=[], new_tests=added,
        new_resources=resources, evidence_sha256=evidence,
        files={name: dict(baseline_sha256=baseline['source_sha256'].get(name),
                         sha256=sources[name]) for name in changed+added+resources},
        purpose='Retain strictly bound supplemental observation computation across clock-only station pending, require new complete station measurement and original audit/admission before any motion.')
    output.mkdir()
    (output/'tmp').mkdir()
    (output/'ros').mkdir()
    write(output/'patch_manifest.json', patch)
    diff = ''.join(''.join(difflib.unified_diff(
        (BACKUP/name).read_text().splitlines(True), (W/name).read_text().splitlines(True),
        fromfile='GEN28/'+name, tofile='GEN29/'+name)) for name in changed)
    (output/'production_and_fixture.diff').write_text(diff)
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', PYTHONNOUSERSITE='1',
        PYTEST_DISABLE_PLUGIN_AUTOLOAD='1', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1',
        ROS_MASTER_URI='http://127.0.0.1:1', TMPDIR=str(output/'tmp'), ROS_LOG_DIR=str(output/'ros'))
    env['PYTHONPATH'] = ':'.join(str(W/name) for name in (
        'src/alicia_flexible_grasp_supervisor/tests', 'src/alicia_grasp_modes/tests',
        'src/alicia_grasp_modes/src', 'src/alicia_flexible_grasp_supervisor/src',
        'devel/lib/python3/dist-packages')) + ':' + env.get('PYTHONPATH', '')
    env['ROS_PACKAGE_PATH'] = str(W/'src') + ':' + env.get('ROS_PACKAGE_PATH', '')
    command = [sys.executable, '-m', 'pytest', '-q', *scope, '-p', 'no:cacheprovider',
        '--tb=short', '--basetemp='+str(output/'pytest_tmp'),
        '--junitxml='+str(output/'pytest.xml'), '-o', 'junit_family=legacy']
    print(json.dumps(dict(started=True, source_count=len(sources), test_files=len(scope), output=str(output))), flush=True)
    start = time.monotonic()
    with (output/'pytest.log').open('x') as log:
        run = subprocess.run(command, cwd=W, env=env, stdout=log, stderr=subprocess.STDOUT)
    root = ET.parse(output/'pytest.xml').getroot()
    summary = {key: sum(int(s.get(key, 0)) for s in root.iter('testsuite'))
               for key in ('tests', 'failures', 'errors', 'skipped')}
    counts = dict.fromkeys(scope, 0)
    unassigned = []
    for case in root.iter('testcase'):
        name = case.get('file', '')
        if name in counts:
            counts[name] += 1
        else:
            unassigned.append(dict(file=name, classname=case.get('classname'), name=case.get('name')))
    changed_during = [name for name, digest in sources.items() if sha(W/name) != digest]
    unchanged = not changed_during and sha(BASE) == BASE_SHA and sha(__file__) == runner_sha
    unchanged = unchanged and new_tests() == added and all(sha(path) == digest for path, digest in evidence.items())
    retained_counts = all(counts[name] >= count for name, count in baseline['test_case_counts_by_file'].items())
    complete = not unassigned and retained_counts and all(value > 0 for value in counts.values()) and sum(counts.values()) == summary['tests']
    passed = run.returncode == 0 and unchanged and complete and summary['tests'] > 7067 and all(summary[k] == 0 for k in ('failures', 'errors', 'skipped'))
    result = dict(passed=passed, summary=summary, returncode=run.returncode,
        elapsed_sec=time.monotonic()-start, source_sha256=sources, source_count=len(sources),
        source_unchanged=unchanged, changed_source_paths=changed_during,
        test_scope=scope, test_scope_files=len(scope), test_case_counts_by_file=counts,
        scope_complete=complete, baseline_file_test_counts_retained=retained_counts,
        unassigned_testcases=unassigned, command=command,
        baseline=str(BASE), baseline_sha256=BASE_SHA, full_scope_baseline=str(BASE),
        full_scope_baseline_sha256=BASE_SHA, patch=str(output/'patch_manifest.json'),
        patch_sha256=sha(output/'patch_manifest.json'), log_sha256=sha(output/'pytest.log'),
        xml_sha256=sha(output/'pytest.xml'), runner_sha256=runner_sha,
        regression_scope='full_combined', production_worktree_tested=True,
        live_ros_contacted=False, motion_commands_sent=False, temperature_queried=False)
    guard(output)
    write(output/'result.json', result)
    print(json.dumps(dict(passed=passed, summary=summary, result=str(output/'result.json'))), flush=True)
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
