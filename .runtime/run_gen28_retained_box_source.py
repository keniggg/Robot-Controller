"""Offline GEN28 source-binding tests; all test artifacts on SanStand."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

work = Path(__file__).resolve().parents[1]
mount = Path('/media/zhuyupei/SanStand')
root = mount / '实验记录/Robot-Controller/ros_log/full_latest_gen22_restart_20261015T003700Z/verification'
out = root / sys.argv[1]
assert mount.is_mount() and not os.statvfs(mount).f_flag & os.ST_RDONLY
assert not out.exists() and out.parent == root
scope = sys.argv[2:] or [
    'src/alicia_flexible_grasp_supervisor/tests/test_gen28_retained_box_source.py',
    'src/alicia_flexible_grasp_supervisor/tests/test_gen26_inferred_box_remote_pipeline.py',
    'src/alicia_flexible_grasp_supervisor/tests/test_gen26_registered_box_support_independent.py',
    'src/alicia_flexible_grasp_supervisor/tests/test_gen26_regular_box_inference_independent.py',
    'src/alicia_flexible_grasp_supervisor/tests/test_gen26_contact_provenance_independent.py',
    'src/alicia_flexible_grasp_supervisor/tests/test_snapshot_original_source.py']
files = sorted(set(scope + [
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/regular_box_inference.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/rgbd_snapshot.py',
    'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py',
    '.runtime/run_gen28_retained_box_source.py']))
def hashes():
    return {path: hashlib.sha256((work / path).read_bytes()).hexdigest() for path in files}
before = hashes()
out.mkdir()
for directory in ('tmp', 'ros'):
    (out / directory).mkdir()
env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', PYTHONNOUSERSITE='1',
    PYTEST_DISABLE_PLUGIN_AUTOLOAD='1', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1',
    ROS_MASTER_URI='http://127.0.0.1:1', TMPDIR=str(out / 'tmp'), ROS_LOG_DIR=str(out / 'ros'))
env['PYTHONPATH'] = ':'.join(str(work / path) for path in (
    'src/alicia_flexible_grasp_supervisor/tests', 'src/alicia_grasp_modes/tests',
    'src/alicia_grasp_modes/src', 'src/alicia_flexible_grasp_supervisor/src',
    'devel/lib/python3/dist-packages')) + ':' + env.get('PYTHONPATH', '')
env['ROS_PACKAGE_PATH'] = str(work / 'src') + ':' + env.get('ROS_PACKAGE_PATH', '')
cmd = [sys.executable, '-m', 'pytest', '-q', *scope, '-p', 'no:cacheprovider',
       '--tb=short', '--basetemp=' + str(out / 'pytest_tmp'), '--junitxml=' + str(out / 'pytest.xml')]
started = time.monotonic()
with (out / 'pytest.log').open('x') as stream:
    run = subprocess.run(cmd, cwd=work, env=env, stdout=stream, stderr=subprocess.STDOUT)
summary = {key: sum(int(row.get(key, 0)) for row in ET.parse(out / 'pytest.xml').getroot().iter('testsuite'))
           for key in ('tests', 'failures', 'errors', 'skipped')}
receipt = dict(returncode=run.returncode, summary=summary, source_sha256=before,
    source_unchanged=before == hashes(), elapsed_sec=time.monotonic() - started,
    command=cmd, live_ros_contacted=False, motion_commands_sent=False, temperature_queried=False)
(out / 'receipt.json').write_text(json.dumps(receipt, indent=2) + '\n')
print(json.dumps(receipt))
print((out / 'pytest.log').read_text())
sys.exit(run.returncode)
