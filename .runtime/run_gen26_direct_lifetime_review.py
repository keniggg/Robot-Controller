"""Offline direct lifetime regression; artifacts on SanStand."""
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
out.mkdir()
for directory in ('tmp', 'ros'):
    (out / directory).mkdir()
scope = sys.argv[2:] or ['src/alicia_flexible_grasp_supervisor/tests/test_gen26_direct_lifetime_downstream_independent.py']
files = scope + [
    'src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/direct_contact_authority.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/pregrasp_gateway.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/contact_pregrasp_gateway.py',
    'src/alicia_grasp_modes/scripts/mode_aware_grasp_task.py']
def hashes():
    return {p: hashlib.sha256((work/p).read_bytes()).hexdigest() for p in files}
before = hashes()
env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', PYTHONNOUSERSITE='1',
    PYTEST_DISABLE_PLUGIN_AUTOLOAD='1', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1',
    ROS_MASTER_URI='http://127.0.0.1:1', TMPDIR=str(out/'tmp'), ROS_LOG_DIR=str(out/'ros'))
env['PYTHONPATH'] = ':'.join(str(work/p) for p in (
    'src/alicia_flexible_grasp_supervisor/tests', 'src/alicia_grasp_modes/tests',
    'src/alicia_grasp_modes/src', 'src/alicia_flexible_grasp_supervisor/src',
    'devel/lib/python3/dist-packages')) + ':' + env.get('PYTHONPATH', '')
env['ROS_PACKAGE_PATH'] = str(work/'src') + ':' + env.get('ROS_PACKAGE_PATH', '')
cmd = [sys.executable, '-m', 'pytest', '-q', *scope, '-p', 'no:cacheprovider',
       '--tb=short', '--basetemp='+str(out/'pytest_tmp'), '--junitxml='+str(out/'pytest.xml')]
started = time.monotonic()
with (out/'pytest.log').open('x') as stream:
    run = subprocess.run(cmd, cwd=work, env=env, stdout=stream, stderr=subprocess.STDOUT)
summary = {k: sum(int(s.get(k, 0)) for s in ET.parse(out/'pytest.xml').getroot().iter('testsuite'))
           for k in ('tests', 'failures', 'errors', 'skipped')}
receipt = dict(returncode=run.returncode, summary=summary, source_sha256=before,
    source_unchanged=before == hashes(), elapsed_sec=time.monotonic()-started,
    command=cmd, live_ros_contacted=False, motion_commands_sent=False, temperature_queried=False)
(out/'receipt.json').write_text(json.dumps(receipt, indent=2)+'\n')
print(json.dumps(receipt))
print((out/'pytest.log').read_text())
sys.exit(run.returncode)
