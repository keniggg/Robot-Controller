"""Source-bound offline regression; all output stays on SanStand."""
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
E = M/'实验记录/Robot-Controller'
C = E/'ros_log/full_latest_gen29_restart_20261016T000728Z'
BASE = E/'ros_log/full_latest_gen22_restart_20261015T003700Z/verification/gen35_release_manifest.json'
BASE_SHA = '843e87ab303ed5e9ef264f19e4ea284511d1812d68010d1014df5da4be913001'
BACKUP = W/'.runtime/gen35_baseline_source'


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda: f.read(1024*1024), b''):
            h.update(chunk)
    return h.hexdigest()


def write(path, data):
    with Path(path).open('x') as f:
        json.dump(data, f, ensure_ascii=False, indent=2, allow_nan=False)
        f.write('\n')
        f.flush()
        os.fsync(f.fileno())


def guard(output):
    assert M.is_mount()
    output.resolve().relative_to((C/'verification').resolve())
    cfg = json.loads((E/'grasp_records/archive_config.json').read_text())
    storage = json.loads((E/'maintenance/storage_control.json').read_text())
    assert cfg['upload_enabled'] is False and cfg['delete_enabled'] is False
    assert storage['archive_policy'] == 'local_only'
    assert shutil.disk_usage(E).free >= cfg['min_free_bytes']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--test', action='append', default=[])
    parser.add_argument('--full', action='store_true')
    args = parser.parse_args()
    output = args.output.resolve()
    guard(output)
    assert not output.exists()
    assert sha(BASE) == BASE_SHA
    baseline = json.loads(BASE.read_text())
    original = baseline['source_sha256']
    assert baseline['source_count'] == 1001 and baseline['test_summary']['tests'] == 7570
    assert json.loads((BACKUP/'manifest.json').read_text())['source_sha256'] == original
    assert all(sha(BACKUP/name) == digest for name, digest in original.items())
    new_tests = sorted(str(p.relative_to(W)) for root in (
        W/'src/alicia_grasp_modes/tests', W/'src/alicia_flexible_grasp_supervisor/tests')
        for p in root.glob('test_gen36*.py'))
    scope = list(dict.fromkeys(baseline['test_scope']+new_tests)) if args.full else args.test
    assert scope and len(scope) == len(set(scope))
    assert all(name in set(original) | set(new_tests) and '/tests/test_' in name
               for name in scope)
    sources = {name: sha(W/name) for name in original}
    sources.update({name: sha(W/name) for name in scope})
    changed = sorted(name for name, digest in original.items() if sources[name] != digest)
    output.mkdir()
    for name in ('tmp', 'ros', 'bootstrap'):
        (output/name).mkdir()
    runner_sha = sha(__file__)
    write(output/'scope.json', dict(full=args.full, tests=scope, source_sha256=sources,
        changed_from_gen35=changed, baseline=str(BASE), baseline_sha256=BASE_SHA))
    (output/'changes_from_gen35.diff').write_text(''.join(''.join(difflib.unified_diff(
        (BACKUP/name).read_text().splitlines(True), (W/name).read_text().splitlines(True),
        fromfile='GEN35/'+name, tofile='GEN36/'+name)) for name in changed))
    # A BaseException cannot be swallowed by production's Exception recovery.
    # Install before imports in pytest and Python subprocesses, not just in a
    # test fixture. An attempted external call makes the whole run fail even
    # if a caller deliberately catches it.
    bootstrap = output/'bootstrap/sitecustomize.py'
    bootstrap.write_text('''import json, os, socket, sys, time
class OfflineExternalBoundaryViolation(BaseException):
    pass
def forbidden(*args, **kwargs):
    path = os.environ['OFFLINE_BOUNDARY_LOG']
    with open(path, 'a') as stream:
        stream.write(json.dumps(dict(at_ns=time.time_ns(), args=repr(args)[:600]))+'\\n')
    raise OfflineExternalBoundaryViolation('unsimulated external boundary')
socket.socket.connect = forbidden
socket.socket.connect_ex = forbidden
socket.socket.sendto = forbidden
socket.create_connection = forbidden
socket.getaddrinfo = forbidden
def device_open(event, args):
    if event != 'open' or not args or not isinstance(args[0], (str, bytes)):
        return
    path = os.path.realpath(os.fsdecode(args[0]))
    if not path.startswith('/dev/'):
        return
    virtual_files = ('/dev/null', '/dev/zero', '/dev/random', '/dev/urandom', '/dev/ptmx')
    virtual_directories = ('/dev/shm/', '/dev/fd/', '/dev/pts/')
    if path not in virtual_files and not path.startswith(virtual_directories):
        forbidden('physical_device_open', path)
sys.addaudithook(device_open)
''')
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', PYTHONNOUSERSITE='1',
        PYTEST_DISABLE_PLUGIN_AUTOLOAD='1', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1',
        ROS_MASTER_URI='http://127.0.0.1:1', TMPDIR=str(output/'tmp'),
        ROS_LOG_DIR=str(output/'ros'), OFFLINE_BOUNDARY_LOG=str(output/'boundary_attempts.jsonl'))
    env['PYTHONPATH'] = ':'.join([str(output/'bootstrap')]+[str(W/name) for name in (
        'src/alicia_flexible_grasp_supervisor/tests', 'src/alicia_grasp_modes/tests',
        'src/alicia_grasp_modes/src', 'src/alicia_flexible_grasp_supervisor/src',
        'devel/lib/python3/dist-packages')])+':'+env.get('PYTHONPATH', '')
    env['ROS_PACKAGE_PATH'] = str(W/'src')+':'+env.get('ROS_PACKAGE_PATH', '')
    command = [sys.executable, '-m', 'pytest', '-q', '--rootdir='+str(W),
        *scope, '-p', 'no:cacheprovider',
        '--tb=short', '--basetemp='+str(output/'pytest_tmp'),
        '--junitxml='+str(output/'pytest.xml'), '-o', 'junit_family=legacy']
    print(json.dumps(dict(started=True, output=str(output), full=args.full,
        test_files=len(scope), source_count=len(sources))), flush=True)
    started = time.monotonic()
    with (output/'pytest.log').open('x') as f:
        run = subprocess.run(command, cwd=W, env=env, stdout=f, stderr=subprocess.STDOUT)
    summary = {key: 0 for key in ('tests', 'failures', 'errors', 'skipped')}
    counts = dict.fromkeys(scope, 0)
    unassigned = []
    if (output/'pytest.xml').exists():
        root = ET.parse(output/'pytest.xml').getroot()
        for suite in root.iter('testsuite'):
            for key in summary:
                summary[key] += int(suite.get(key, 0))
        for case in root.iter('testcase'):
            name = case.get('file', '')
            if name in counts:
                counts[name] += 1
            else:
                unassigned.append(dict(file=name, name=case.get('name')))
    altered = [name for name, digest in sources.items() if sha(W/name) != digest]
    unchanged = not altered and sha(BASE) == BASE_SHA and sha(__file__) == runner_sha
    retained = (not args.full or all(counts.get(name, 0) >= count
        for name, count in baseline['test_case_counts_by_file'].items()))
    boundary_path = output/'boundary_attempts.jsonl'
    attempts = boundary_path.read_text().splitlines() if boundary_path.exists() else []
    complete = not unassigned and all(counts.values()) and sum(counts.values()) == summary['tests']
    passed = run.returncode == 0 and unchanged and retained and complete and not attempts
    passed = passed and summary['tests'] > 0 and all(summary[k] == 0 for k in ('failures', 'errors', 'skipped'))
    result = dict(passed=passed, summary=summary, returncode=run.returncode,
        elapsed_sec=time.monotonic()-started, source_sha256=sources, source_count=len(sources),
        source_unchanged=unchanged, changed_during_run=altered, test_scope=scope,
        test_scope_files=len(scope), test_case_counts_by_file=counts, scope_complete=complete,
        baseline_file_test_counts_retained=retained, unassigned_testcases=unassigned,
        regression_scope='full_combined' if args.full else 'focused',
        baseline=str(BASE), baseline_sha256=BASE_SHA, runner_sha256=runner_sha,
        bootstrap_sha256=sha(bootstrap), external_boundary_attempts=attempts,
        external_guard_scope='Python socket network calls and CPython physical-device open audit; C SDK factories require inert test fixtures',
        command=command, log_sha256=sha(output/'pytest.log'),
        xml_sha256=sha(output/'pytest.xml') if (output/'pytest.xml').exists() else None,
        live_ros_contacted=False, motion_commands_sent=False, temperature_queried=False,
        deployed=False, physical_grasp_verified=False)
    guard(output)
    write(output/'result.json', result)
    print(json.dumps(dict(passed=passed, summary=summary, result=str(output/'result.json'))), flush=True)
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
