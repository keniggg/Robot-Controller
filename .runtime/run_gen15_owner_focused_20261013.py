"""Run fixed GEN15 ownership/contact/closure regression without a ROS master."""
from pathlib import Path
import hashlib
import json
import os
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

W = Path(__file__).resolve().parent.parent
C = W / '.runtime/gen15_qualified_phase_timestamp_20261013'
N = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
O = N / 'verification' / sys.argv[1]
assert Path('/media/zhuyupei/SanStand').is_mount()
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
prior = N / 'verification/gen14_common_face_integrated_install/result.json'
actual = json.loads(prior.read_text())['source_sha256']
assert len(actual) == 904 and all(sha(W/p) == h for p, h in actual.items())
names = ['test_qualified_phase_timestamp_precision.py',
         'test_qualified_phase_epoch_independent.py',
         'test_near_field_exact_deadline.py', 'test_qualified_station_phase.py',
         'test_station_compute_wait_task.py', 'test_observed_contact_current_pose.py',
         'test_observe_return_task.py', 'test_station_compute_wait_integration.py',
         'test_record_sidecar_finalize.py']
scope = ['src/alicia_grasp_modes/tests/' + name for name in names]
paths = sorted(C.rglob('*.py')) + [Path(__file__)]
before = {str(p): sha(p) for p in paths}
env = os.environ.copy()
env.update(PYTHONNOUSERSITE='1', PYTHONDONTWRITEBYTECODE='1',
           OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1',
           ROS_MASTER_URI='http://127.0.0.1:1')
env['ROS_PACKAGE_PATH'] = str(C/'src') + ':' + env.get('ROS_PACKAGE_PATH', '')
env['PYTHONPATH'] = ':'.join([str(C/'src/alicia_grasp_modes/src'),
    str(C/'src/alicia_flexible_grasp_supervisor/src'),
    str(C/'src/alicia_flexible_grasp_supervisor/tests'), env.get('PYTHONPATH', '')])
O.mkdir(exist_ok=False)
pytest_args = ['-q', '-p', 'no:cacheprovider', *[str(C/p) for p in scope],
               '--junitxml='+str(O/'pytest.xml')]
bootstrap = '''import json, pathlib, sys, pytest
result = pytest.main(json.loads(sys.argv[1]))
import test_task_modes
from alicia_grasp_modes import observe_return_task
loaded = {'base':test_task_modes.base.__file__, 'task':observe_return_task.__file__}
pathlib.Path(sys.argv[3]).write_text(json.dumps(loaded, indent=2)+'\\n')
root = pathlib.Path(sys.argv[2]).resolve()
assert all(root in pathlib.Path(p).resolve().parents for p in loaded.values()), loaded
sys.exit(result)
'''
cmd = [sys.executable, '-c', bootstrap, json.dumps(pytest_args), str(C), str(O/'loaded_sources.json')]
start = time.monotonic()
with (O/'pytest.log').open('w') as log:
    completed = subprocess.run(cmd, cwd=W, env=env, stdout=log, stderr=subprocess.STDOUT)
root = ET.parse(O/'pytest.xml').getroot()
summary = {k: sum(int(x.attrib.get(k, 0)) for x in root.findall('testsuite'))
           for k in ('tests', 'failures', 'errors', 'skipped')}
after = {str(p): sha(p) for p in paths}
unchanged = all(sha(W/p) == h for p, h in actual.items())
result = dict(passed=completed.returncode == 0 and before == after and unchanged,
    summary=summary, returncode=completed.returncode, elapsed_sec=time.monotonic()-start,
    source_sha256=before, source_sha256_after=after, source_unchanged=before == after,
    actual_unchanged=unchanged, actual_source_count=len(actual), prior=str(prior),
    prior_sha256=sha(prior), pytest_xml_sha256=sha(O/'pytest.xml'),
    loaded_sources=json.loads((O/'loaded_sources.json').read_text()),
    test_scope=scope, command=cmd, candidate_only=True, no_ros_master=True,
    motion_commands_sent=False, temperature_queried=False,
    scope='Exact timestamp phase ownership, original qualified/contact/station checks, '
          'independent Unix epoch adversarial coverage and real filesystem recorder finalization; '
          'no physical grasp success claim.')
(O/'result.json').write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps({k: result[k] for k in ('passed', 'summary', 'source_unchanged', 'actual_unchanged', 'elapsed_sec')}), flush=True)
