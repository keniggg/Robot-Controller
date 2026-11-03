"""Prove a two-assignment fixture completion against unchanged installed code."""
from pathlib import Path
import ast
import hashlib
import json
import os
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

W = Path(__file__).resolve().parent.parent
C = W / '.runtime/gen15_gen11_fixture_completion_20261013'
N = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
O = N / 'verification' / sys.argv[1]
assert Path('/media/zhuyupei/SanStand').is_mount()
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
prior = N / 'verification/gen15_phase_timestamp_actual_integrated/result.json'
baseline = json.loads(prior.read_text())['source_sha256']
assert len(baseline) == 908 and all(sha(W/p) == h for p, h in baseline.items())
rel = 'src/alicia_grasp_modes/tests/test_gen11_contact_compute_admission_independent.py'
before_ast = ast.parse((W/rel).read_text())
after_ast = ast.parse((C/rel).read_text())
fixture = next(n for n in after_ast.body if isinstance(n, ast.FunctionDef) and n.name == 'task_for')
added = [n for n in fixture.body if isinstance(n, ast.Assign) and len(n.targets) == 1
         and isinstance(n.targets[0], ast.Attribute)
         and n.targets[0].attr in ('_near_field_phase_started_ns', '_near_field_phase_deadline_ns')]
assert len(added) == 2
fixture.body = [n for n in fixture.body if n not in added]
assert ast.dump(after_ast, include_attributes=False) == ast.dump(before_ast, include_attributes=False)
tests = [C/rel, C/'verification/test_fixture_cache_equivalence.py']
paths = tests + [C/'src/alicia_grasp_modes/tests/fixtures/gen11_contact_transport.json.gz', Path(__file__)]
hashes = {str(p): sha(p) for p in paths}
env = os.environ.copy()
env.update(PYTHONNOUSERSITE='1', PYTHONDONTWRITEBYTECODE='1',
           OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', ROS_MASTER_URI='http://127.0.0.1:1')
env['ROS_PACKAGE_PATH'] = str(W/'src')+':'+env.get('ROS_PACKAGE_PATH', '')
env['PYTHONPATH'] = ':'.join([str(C/'src/alicia_grasp_modes/tests'),
    str(W/'src/alicia_grasp_modes/tests'), str(W/'src/alicia_grasp_modes/src'),
    str(W/'src/alicia_flexible_grasp_supervisor/src'),
    str(W/'src/alicia_flexible_grasp_supervisor/tests'), env.get('PYTHONPATH', '')])
O.mkdir(exist_ok=False)
args = ['-q', '-p', 'no:cacheprovider', *map(str, tests), '--junitxml='+str(O/'pytest.xml')]
bootstrap = '''import json, pathlib, sys, pytest
status=pytest.main(json.loads(sys.argv[1]))
import test_gen11_contact_compute_admission_independent as tested
from alicia_grasp_modes import observe_return_task
paths={'test':tested.__file__,'base':tested.task_base.__file__,'task':observe_return_task.__file__}
pathlib.Path(sys.argv[4]).write_text(json.dumps(paths,indent=2)+'\\n')
assert pathlib.Path(paths['test']).resolve()==pathlib.Path(sys.argv[2]).resolve()
root=pathlib.Path(sys.argv[3]).resolve()
assert pathlib.Path(paths['base']).resolve()==root/'src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py'
assert pathlib.Path(paths['task']).resolve()==root/'src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_task.py'
sys.exit(status)
'''
command = [sys.executable, '-c', bootstrap, json.dumps(args), str(C/rel), str(W), str(O/'loaded_sources.json')]
start = time.monotonic()
with (O/'pytest.log').open('w') as log:
    run = subprocess.run(command, cwd=W, env=env, stdout=log, stderr=subprocess.STDOUT)
root = ET.parse(O/'pytest.xml').getroot()
cases = list(root.iter('testcase'))
old_cases = [x for x in ET.parse(prior.parent/'pytest.xml').getroot().iter('testcase')
             if 'test_gen11_contact_compute_admission_independent' in x.attrib['classname']]
retested = [x for x in cases if 'test_gen11_contact_compute_admission_independent' in x.attrib['classname']]
same_cases = sorted(x.attrib['name'] for x in old_cases) == sorted(x.attrib['name'] for x in retested)
summary = {k: sum(int(s.attrib.get(k, 0)) for s in root.findall('testsuite'))
           for k in ('tests', 'failures', 'errors', 'skipped')}
unchanged = all(sha(W/p) == h for p, h in baseline.items())
same_sources = all(sha(Path(p)) == h for p, h in hashes.items())
result = dict(passed=run.returncode == 0 and same_sources and unchanged and same_cases,
    summary=summary, returncode=run.returncode, elapsed_sec=time.monotonic()-start,
    source_sha256=hashes, source_unchanged=same_sources, actual_unchanged=unchanged,
    actual_source_count=len(baseline), prior=str(prior), prior_sha256=sha(prior),
    original_case_count=len(old_cases), unchanged_original_cases=same_cases,
    only_two_fixture_assignments_ast=True, original_test_and_fixture_logic_otherwise_identical=True,
    loaded_sources=json.loads((O/'loaded_sources.json').read_text()),
    pytest_xml_sha256=sha(O/'pytest.xml'), command=command,
    no_ros_master=True, motion_commands_sent=False, temperature_queried=False,
    scope='40 unchanged GEN11 cases plus actual installed setter cache equivalence and '
          'six missing/one-nanosecond owner-change rejects; no production edits.')
(O/'result.json').write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps({k: result[k] for k in ('passed', 'summary', 'actual_unchanged', 'source_unchanged', 'elapsed_sec')}), flush=True)
