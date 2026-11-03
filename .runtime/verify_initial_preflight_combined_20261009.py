"""Repeat the combined suite against frozen staged sources; evidence on SanStand."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

work = Path(__file__).resolve().parents[1]
stage = work / '.runtime/initial_precommit_staging_20261009'
root = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z')
assert Path('/media/zhuyupei/SanStand').is_mount()
previous = json.loads((root / 'verification/initial_preflight_combined/result.json').read_text())
output = root / 'verification/initial_preflight_combined_final'
output.mkdir(exist_ok=False)
paths = list(previous['source_sha256'])
def hashes():
    return {p: hashlib.sha256((stage / p).read_bytes()).hexdigest() for p in paths}
before = hashes()
env = os.environ.copy()
env.update(PYTHONDONTWRITEBYTECODE='1', PYTHONNOUSERSITE='1', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1',
           ROS_LOG_DIR=str(output / 'ros'))
env['PYTHONPATH'] = ':'.join([str(stage / 'src/alicia_grasp_modes/src'),
    str(stage / 'src/alicia_grasp_modes/tests'), str(work / 'src/alicia_flexible_grasp_supervisor/tests'),
    env.get('PYTHONPATH', '')])
command = ['/usr/bin/python3', '-m', 'pytest', '-q', '-p', 'no:cacheprovider']
command += [str(stage / 'src/alicia_grasp_modes/tests' / p) for p in previous['test_files']]
command += ['--junitxml', str(output / 'combined.xml')]
started = time.time_ns()
with (output / 'pytest.log').open('w') as log:
    completed = subprocess.run(command, cwd=work, env=env, stdout=log, stderr=subprocess.STDOUT)
cases = list(ET.parse(str(output / 'combined.xml')).getroot().iter('testcase'))
failed = [c.attrib for c in cases if c.find('failure') is not None or c.find('error') is not None]
result = dict(passed=completed.returncode == 0 and hashes() == before,
    returncode=completed.returncode, unique_tests=len({(c.get('classname'), c.get('name')) for c in cases}),
    failed=failed, source_sha256=before, source_unchanged=hashes()==before,
    command=command, test_files=previous['test_files'], started_ns=started, ended_ns=time.time_ns(),
    staging_only=True, physical_success_verified=False)
(output / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
print(json.dumps(dict(passed=result['passed'], tests=result['unique_tests'], failed=failed, report=str(output/'result.json'))), flush=True)
raise SystemExit(0 if result['passed'] else 1)
