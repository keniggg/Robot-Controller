"""Small, guarded GEN55 endpoint tests and actual-input replay; never deployment."""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import resource
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

W = Path(__file__).resolve().parents[1]
D = W/'.runtime/gen55_fk_isolated_r1'
V = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification')
ROOT = V/'gen55_held_pregrasp_reuse_r1'
RUNNER = W/'.runtime/verify_gen54_parallel_offline_20261018.py'
RUNNER_SHA = '2b9023c285ce3aadaaad37c0f734b96a567963dc384838ed3319c3858d24b5f5'
SOURCE = 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/held_pregrasp_selection.py'
TESTS = 'src/alicia_flexible_grasp_supervisor/tests/'
NEW = TESTS+'test_gen55_held_pregrasp_reuse.py'
GUARD_SHA = '5d00e04e91cc09ed375c6dcf7ce728eaee4b0c3cfa9190f4a0ab7c17f7f9e21e'
AUDIT_SHA = '87d31529b92cf02da30b2c9b19460a3d79917ae9d2d2208bbe5a6eee0f023ac7'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(path, value):
    with Path(path).open('x') as f:
        json.dump(value, f, indent=2, sort_keys=True, allow_nan=False)
        f.write('\n'); f.flush(); os.fsync(f.fileno())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('mode', choices=['new', 'legacy', 'replay_reverse'])
    parser.add_argument('--attempt', default='r1')
    args = parser.parse_args()
    assert args.attempt.isalnum()
    assert sha(RUNNER) == RUNNER_SHA
    spec = importlib.util.spec_from_file_location('gen54_guard_definitions', RUNNER)
    runner = importlib.util.module_from_spec(spec); spec.loader.exec_module(runner)
    serial = runner.load_serial()
    output = ROOT/(args.mode+'_'+args.attempt)
    serial.storage(output); output.mkdir()
    for name in ('tmp', 'ros', 'ros_home', 'bootstrap'):
        (output/name).mkdir()
    guard, audit = runner.bootstrap_sources(serial)
    (output/'bootstrap/sitecustomize.py').write_text(guard)
    (output/'bootstrap/gen42_import_audit.py').write_text(audit)
    assert sha(output/'bootstrap/sitecustomize.py') == GUARD_SHA
    assert sha(output/'bootstrap/gen42_import_audit.py') == AUDIT_SHA
    scope = [NEW]
    if args.mode == 'legacy':
        scope = [TESTS+name+'.py' for name in (
            'test_held_pregrasp_selection', 'test_held_pregrasp_diagnostics',
            'test_gen51_shared_held_guide_fallback', 'test_gen45_fresh_held_path_chain',
            'test_gen14_held_approach_precheck', 'test_contact_pregrasp_compute_guard')]
    baseline_path = V/'gen55_baseline_r1/result.json'
    assert sha(baseline_path) == 'c74a26f5bb9de1356141dc8665fa32d5261af3357d434a184ab930c38a55212f'
    baseline = json.loads(baseline_path.read_bytes())
    for path in scope:
        if path != NEW:
            assert sha(D/path) == baseline['source_sha256'][path]
    snapshot = {str(p):sha(p) for p in (D/'src').rglob('*.py') if p.is_file()}
    generated = {str(p):sha(p) for p in (W/'devel/lib/python3/dist-packages').rglob('*.py') if p.is_file()}
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', PYTHONNOUSERSITE='1',
        PYTEST_DISABLE_PLUGIN_AUTOLOAD='1', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1',
        ROS_MASTER_URI='http://127.0.0.1:1', ROS_LOG_DIR=str(output/'ros'),
        ROS_HOME=str(output/'ros_home'), TMPDIR=str(output/'tmp'),
        OFFLINE_BOUNDARY_LOG=str(output/'boundary_attempts.jsonl'),
        GEN42_MODULE_PROVENANCE=str(output/'module_provenance.json'),
        GEN55_REPLAY_ORDER='reverse' if args.mode == 'replay_reverse' else 'forward')
    for key in ('PYTEST_ADDOPTS', 'PYTEST_PLUGINS'): env.pop(key, None)
    paths = [output/'bootstrap']+[D/p for p in (*serial.ROOTS,
        'src/alicia_grasp_modes/src', 'src/alicia_flexible_grasp_supervisor/src')]
    paths += [W/'devel/lib/python3/dist-packages', Path('/opt/ros/noetic/lib/python3/dist-packages')]
    env['PYTHONPATH'] = ':'.join(map(str, paths))
    env['ROS_PACKAGE_PATH'] = str(D/'src')+':/opt/ros/noetic/share'
    command = [sys.executable, '-B', '-m', 'pytest', '-q', '--rootdir='+str(D), *scope,
        '-p', 'no:cacheprovider', '-p', 'gen42_import_audit', '--tb=short',
        '--basetemp='+str(output/'pytest_tmp'), '--junitxml='+str(output/'pytest.xml'),
        '-o', 'junit_family=legacy']
    if args.mode == 'replay_reverse':
        command += ['-k', 'test_actual_gen54_request_matches_original_all_numbers_and_checks']
    save(output/'launch.json', dict(command=command, helper_sha256=sha(__file__),
        scope=scope, tested_source_sha256=snapshot[str(D/SOURCE)],
        source_snapshot=snapshot, generated_snapshot=generated,
        baseline_receipt_sha256=sha(baseline_path), guard_sha256=GUARD_SHA,
        audit_sha256=AUDIT_SHA, test_source_sha256={p:sha(D/p) for p in scope},
        cpu_count=os.cpu_count(), affinity=sorted(os.sched_getaffinity(0)),
        loadavg=Path('/proc/loadavg').read_text(),
        memory={line.split(':')[0]:line.split(':')[1].strip() for line in
                Path('/proc/meminfo').read_text().splitlines()
                if line.startswith(('MemTotal:', 'MemAvailable:', 'SwapTotal:', 'SwapFree:'))}))
    before = resource.getrusage(resource.RUSAGE_CHILDREN); started = time.monotonic()
    with (output/'pytest.log').open('x') as log:
        child = subprocess.Popen(command, cwd=D, env=env, stdout=log, stderr=subprocess.STDOUT)
        returncode = child.wait()
    elapsed = time.monotonic()-started
    after = resource.getrusage(resource.RUSAGE_CHILDREN)
    xml = ET.parse(output/'pytest.xml').getroot()
    cases = list(xml.iter('testcase'))
    counts = dict(tests=len(cases), failures=sum(len(c.findall('failure')) for c in cases),
        errors=sum(len(c.findall('error')) for c in cases), skipped=sum(len(c.findall('skipped')) for c in cases))
    identities = [(c.get('classname'), c.get('name')) for c in cases]
    provenance = json.loads((output/'module_provenance.json').read_bytes())
    invalid = {}
    for name, row in provenance.items():
        path = row['path']
        expected = snapshot.get(path, generated.get(path))
        if expected != row['sha256'] or sha(path) != expected:
            invalid[name] = row
    loaded = any(row['path'] == str(D/SOURCE) and row['sha256'] == snapshot[str(D/SOURCE)]
                 for row in provenance.values())
    boundary_path = output/'boundary_attempts.jsonl'
    boundary = boundary_path.read_text().splitlines() if boundary_path.exists() else []
    source_stable = sha(D/SOURCE) == snapshot[str(D/SOURCE)]
    old_tests_unchanged = all(sha(D/p) == baseline['source_sha256'][p] for p in scope if p != NEW)
    live_source_unchanged = sha(W/SOURCE) == baseline['source_sha256'][SOURCE]
    replay_files = sorted((output/'pytest_tmp').glob('**/replay.json'))
    replays = [json.loads(p.read_bytes()) for p in replay_files]
    if replays:
        save(output/'replays.json', replays)
    passed = (returncode == 0 and counts['tests'] > 0 and not any(counts[k] for k in ('failures','errors','skipped'))
        and len(set(identities)) == len(identities) and not invalid and loaded and not boundary
        and source_stable and old_tests_unchanged and live_source_unchanged)
    result = dict(passed=passed, mode=args.mode, counts=counts, case_identities=identities,
        returncode=returncode, direct_child_reaped=True, elapsed_sec=elapsed,
        child_cpu_sec=(after.ru_utime-before.ru_utime)+(after.ru_stime-before.ru_stime),
        child_maxrss_kib=after.ru_maxrss, full_regression=False,
        workspace_root=str(D), final_integration_test=False,
        boundary_attempts=boundary, invalid_imports=invalid, candidate_source_imported=loaded,
        candidate_source_unchanged=source_stable, old_tests_unchanged=old_tests_unchanged,
        live_source_unchanged=live_source_unchanged, replay_count=len(replays),
        source_sha256=snapshot[str(D/SOURCE)], test_source_sha256={p:sha(D/p) for p in scope},
        files={str(p.relative_to(output)):sha(p) for p in output.rglob('*')
               if p.is_file() and 'pytest_tmp' not in p.parts},
        ROS_initialized=False, hardware_access=False, temperature_queried=False)
    save(output/'result.json', result)
    print(json.dumps({key:result[key] for key in ('passed','mode','counts','elapsed_sec','child_cpu_sec','boundary_attempts','invalid_imports','replay_count')}))
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
