"""Offline GEN30 independent checks with all inherited sources pinned."""
from pathlib import Path
import hashlib
import json
import os
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

work = Path(__file__).resolve().parents[1]
mount = Path('/media/zhuyupei/SanStand')
root = mount / '实验记录/Robot-Controller/ros_log/full_latest_gen22_restart_20261015T003700Z/verification'
assert len(sys.argv) in (2, 3) and '/' not in sys.argv[1]
assert len(sys.argv) == 2 or sys.argv[2] == '--related'
out = root / sys.argv[1]
assert mount.is_mount() and not os.statvfs(mount).f_flag & os.ST_RDONLY
assert not out.exists()
tests = ['src/alicia_grasp_modes/tests/test_gen30_current_instance_independent.py']
if len(sys.argv) == 3:
    tests += ['src/alicia_grasp_modes/tests/' + name + '.py' for name in (
        'test_observation_current_support', 'test_observation_arrival_probable_foreground',
        'test_arrival_fragment_recovery_guards', 'test_arrival_matching_fragments',
        'test_arrival_matching_fragment_tracking', 'test_measured_mask_seeds',
        'test_side_seed_evidence_scale', 'test_qualified_station_appearance',
        'test_station_compute_wait_producer', 'test_rgb_instance')]
baseline = work / '.runtime/gen30_baseline_source/manifest.json'
paths = set(json.loads(baseline.read_text()))
paths.update(tests)
paths.update(('.runtime/gen30_baseline_source/manifest.json',
              '.runtime/run_gen30_current_instance_independent.py'))
for relative in ('src/alicia_grasp_modes/src/alicia_grasp_modes',
                 'src/alicia_grasp_modes/scripts'):
    paths.update(str(p.relative_to(work)) for p in (work / relative).rglob('*.py'))
def hashes():
    return {p: hashlib.sha256((work / p).read_bytes()).hexdigest() for p in sorted(paths)}
before = hashes()
out.mkdir()
for name in ('tmp', 'ros'):
    (out / name).mkdir()
env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', PYTHONNOUSERSITE='1',
    PYTEST_DISABLE_PLUGIN_AUTOLOAD='1', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1',
    ROS_MASTER_URI='http://127.0.0.1:1', TMPDIR=str(out / 'tmp'), ROS_LOG_DIR=str(out / 'ros'))
env['PYTHONPATH'] = ':'.join(str(work / p) for p in (
    'src/alicia_flexible_grasp_supervisor/tests', 'src/alicia_grasp_modes/tests',
    'src/alicia_grasp_modes/src', 'src/alicia_flexible_grasp_supervisor/src',
    'devel/lib/python3/dist-packages')) + ':' + env.get('PYTHONPATH', '')
env['ROS_PACKAGE_PATH'] = str(work / 'src') + ':' + env.get('ROS_PACKAGE_PATH', '')
cmd = [sys.executable, '-B', '-m', 'pytest', '-q', *tests, '-p', 'no:cacheprovider',
    '--tb=short', '--basetemp=' + str(out / 'pytest_tmp'), '--junitxml=' + str(out / 'pytest.xml')]
started = time.monotonic()
with (out / 'pytest.log').open('x') as stream:
    run = subprocess.run(cmd, cwd=work, env=env, stdout=stream, stderr=subprocess.STDOUT)
summary = {k: sum(int(row.get(k, 0)) for row in ET.parse(out / 'pytest.xml').getroot().iter('testsuite'))
    for k in ('tests', 'failures', 'errors', 'skipped')} if (out / 'pytest.xml').exists() else None
receipt = dict(returncode=run.returncode, summary=summary, source_sha256=before,
    source_unchanged=before == hashes(), test_files=tests, command=cmd,
    elapsed_sec=time.monotonic() - started, live_ros_contacted=False,
    motion_commands_sent=False, temperature_queried=False)
(out / 'receipt.json').write_text(json.dumps(receipt, indent=2) + '\n')
print(json.dumps({k:v for k,v in receipt.items() if k != 'source_sha256'}))
print((out / 'pytest.log').read_text())
sys.exit(run.returncode)
