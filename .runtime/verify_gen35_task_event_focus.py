"""Offline bounded task callback regression; logs only on SanStand."""
from pathlib import Path
import difflib
import hashlib
import json
import os
import subprocess
import time
import xml.etree.ElementTree as ET

W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
C = M / '实验记录/Robot-Controller/ros_log/full_latest_gen29_restart_20261016T000728Z'
OUT = C / 'verification/gen35_task_event_focus_r1'
PRODUCTION = ['src/alicia_grasp_modes/src/alicia_grasp_modes/' + name + '.py'
              for name in ('station_compute_wait_task', 'observe_return_task')]
PRODUCTION += ['src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py',
               'src/alicia_grasp_modes/scripts/mode_aware_grasp_task.py']
TESTS = ['src/alicia_grasp_modes/tests/' + name + '.py' for name in (
    'test_gen35_task_pending_handoff', 'test_gen35_task_event_identity', 'test_task_modes',
    'test_initial_preflight_task', 'test_station_compute_wait_task',
    'test_station_contact_admission', 'test_station_contact_current_wait',
    'test_station_contact_remote_current_wait', 'test_gen19_measurement_ack_clock',
    'test_observe_return_measurement', 'test_observation_measurement_task_boundaries',
    'test_measurement_timeout_reason', 'test_supplement_measurement_preparation',
    'test_station_requalification_task_finalize', 'test_station_requalification_three_owners')]

TESTS += ['src/alicia_flexible_grasp_supervisor/tests/test_grasp_task_node.py']

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def write(name, value):
    path = OUT / name
    with path.open('x') as stream:
        json.dump(value, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write('\n')

assert M.is_mount() and C.stat().st_dev == M.stat().st_dev
OUT.mkdir()
for name in ('tmp', 'ros'):
    (OUT / name).mkdir()
baseline = W / '.runtime/gen34_baseline_source'
originals = json.loads((baseline / 'manifest.json').read_text())['source_sha256']
paths = sorted(set(originals) | set(TESTS))
before = {name: sha(W / name) for name in paths}
write('source_before.json', before)
assert {name for name, old in originals.items() if before[name] != old} == set(PRODUCTION)
for name in PRODUCTION:
    assert sha(baseline / name) == originals[name]
    diff = ''.join(difflib.unified_diff((baseline / name).read_text().splitlines(True),
        (W / name).read_text().splitlines(True), fromfile='GEN33/' + name, tofile='GEN35/' + name))
    (OUT / (Path(name).stem + '.diff')).write_text(diff)
env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', PYTHONNOUSERSITE='1',
    PYTEST_DISABLE_PLUGIN_AUTOLOAD='1', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1',
    ROS_MASTER_URI='http://127.0.0.1:1', ROS_PACKAGE_PATH=str(W/'src')+':/opt/ros/noetic/share',
    TMPDIR=str(OUT/'tmp'), ROS_LOG_DIR=str(OUT/'ros'))
env['PYTHONPATH'] = ':'.join(str(W / p) for p in (
    'src/alicia_flexible_grasp_supervisor/tests', 'src/alicia_grasp_modes/tests',
    'src/alicia_grasp_modes/src', 'src/alicia_flexible_grasp_supervisor/src',
    'devel/lib/python3/dist-packages')) + ':/opt/ros/noetic/lib/python3/dist-packages'
cmd = ['/usr/bin/python3', '-m', 'pytest', *TESTS, '-q', '-p', 'no:cacheprovider',
    '--basetemp=' + str(OUT/'pytest_tmp'), '--junitxml=' + str(OUT/'pytest.xml')]
write('invocation.json', dict(command=cmd, cwd=str(W), environment={key: env[key] for key in (
    'PYTHONPATH', 'PYTHONNOUSERSITE', 'PYTHONDONTWRITEBYTECODE', 'PYTEST_DISABLE_PLUGIN_AUTOLOAD',
    'OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'ROS_MASTER_URI', 'ROS_PACKAGE_PATH', 'TMPDIR', 'ROS_LOG_DIR')}))
t = time.monotonic()
with (OUT/'pytest.log').open('x') as log:
    proc = subprocess.run(cmd, cwd=W, env=env, stdout=log, stderr=subprocess.STDOUT)
elapsed = time.monotonic() - t
after = {name: sha(W / name) for name in paths}
write('source_after.json', after)
xml = ET.parse(OUT/'pytest.xml').getroot()
suites = [xml] if xml.tag == 'testsuite' else list(xml.iter('testsuite'))
summary = {key: sum(int(s.get(key, 0)) for s in suites) for key in ('tests', 'failures', 'errors', 'skipped')}
passed = proc.returncode == 0 and before == after and not any(summary[k] for k in ('failures', 'errors', 'skipped'))
write('result.json', dict(passed=passed, exit_code=proc.returncode, wall_sec=elapsed,
    summary=summary, source_unchanged=before==after, source_count=len(paths),
    production_sha256={name: before[name] for name in PRODUCTION},
    test_sha256={name: before[name] for name in TESTS},
    evidence_sha256={name: sha(OUT/name) for name in ('source_before.json', 'source_after.json',
        'invocation.json', 'pytest.log', 'pytest.xml', 'station_compute_wait_task.diff', 'observe_return_task.diff', 'grasp_task_node.diff', 'mode_aware_grasp_task.diff')},
    script_sha256=sha(__file__), no_ros_or_hardware_calls=True))
print(json.dumps(dict(passed=passed, summary=summary, result=str(OUT/'result.json'), sha256=sha(OUT/'result.json'))))
raise SystemExit(0 if passed else 1)
