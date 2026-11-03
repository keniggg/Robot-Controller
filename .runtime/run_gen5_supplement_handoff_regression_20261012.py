"""Verify the integrated handoff and existing contact chain without live ROS."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

work = Path(__file__).resolve().parents[1]
root = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_frozen_no_skin_20261012T021050Z')
assert Path('/media/zhuyupei/SanStand').is_mount()
out = root/'verification'/('gen5_handoff_integrated'+(sys.argv[1] if len(sys.argv)>1 else ''))
out.mkdir(exist_ok=False)
installed = json.loads((root/'verification/gen5_handoff_integration_install/install_receipt.json').read_text())
assert installed['passed'] is True
expected = installed['source_sha256']
def sources():
    return {p: hashlib.sha256((work/p).read_bytes()).hexdigest() for p in expected}
before = sources()
assert before == expected
old = json.loads((root/'verification/gen4_contact_compute_integration_r1/result.json').read_text())
scope = list(old['test_scope'])
mode = work/'src/alicia_grasp_modes/tests'
required = len(sys.argv)>2 and sys.argv[2]=='required'
if required:
    # Reproduce the reviewed component scopes together with the existing
    # contact suite. The separate broad comparison retains every old failure.
    for relative in ('gen5_handoff_task_stage/final_r6.xml','gen5_handoff_remote_stage/pytest_r3.xml'):
        for case in ET.parse(root/'verification'/relative).iter('testcase'):
            parts=case.attrib['classname'].split('.')
            index=next(i for i,p in enumerate(parts) if p.startswith('test_'))
            relative_path='/'.join(parts[:index+1])+'.py'
            if not (work/relative_path).is_file():
                matches=list((work/'src').glob('*/tests/'+parts[index]+'.py'))
                assert len(matches)==1,(relative_path,matches)
                relative_path=str(matches[0].relative_to(work))
            scope.append(relative_path)
    for filename in ('test_supplement_measurement_admission.py','test_station_planning_handoff.py',
                     'test_observation_measurement_unknown.py','test_station_compute_wait_producer.py',
                     'test_observation_arrival_station.py','test_contact_station_retirement.py'):
        scope.append(str((mode/filename).relative_to(work)))
else:
    for pattern in ('test_observ*.py', 'test_supplement*.py', 'test_station*.py',
                    'test_contact_station*.py', 'test_unknown_*.py', 'test_frozen*.py', 'test_arrival*.py'):
        scope.extend(str(p.relative_to(work)) for p in sorted(mode.glob(pattern)))
supervisor = work/'src/alicia_flexible_grasp_supervisor/tests'
scope.extend(str(p.relative_to(work)) for p in sorted(supervisor.glob('test_observe_return*.py')))
scope.extend(p for p in installed['staged_changed_sha256'] if '/tests/test_' in p)
scope = list(dict.fromkeys(scope))
env = os.environ.copy()
env.update(PYTHONNOUSERSITE='1', PYTHONDONTWRITEBYTECODE='1', OPENBLAS_NUM_THREADS='1',
    OMP_NUM_THREADS='1', ROS_MASTER_URI='http://127.0.0.1:1', TMPDIR='/tmp')
env['ROS_PACKAGE_PATH'] = str(work/'src')+':'+env.get('ROS_PACKAGE_PATH','')
env['PYTHONPATH'] = ':'.join(str(work/p) for p in (
    'src/alicia_flexible_grasp_supervisor/src', 'src/alicia_grasp_modes/src',
    'src/alicia_flexible_grasp_supervisor/tests', 'src/alicia_grasp_modes/tests'))+':'+env.get('PYTHONPATH','')
cmd = ['/usr/bin/python3', '-m', 'pytest', '-q', *scope, '-p', 'no:cacheprovider',
       '--junitxml='+str(out/'pytest.xml')]
start = time.monotonic()
with (out/'pytest.log').open('x') as log:
    process = subprocess.run(cmd, cwd=work, env=env, stdout=log, stderr=subprocess.STDOUT)
after = sources()
summary = {}
if (out/'pytest.xml').exists():
    suites = ET.parse(out/'pytest.xml').getroot().iter('testsuite')
    for suite in suites:
        for key in ('tests','failures','errors','skipped'):
            summary[key] = summary.get(key, 0)+int(suite.attrib.get(key,0))
receipt = dict(returncode=process.returncode, source_sha256=before, source_after_sha256=after,
    source_unchanged=before==after, test_scope=scope, scope_kind='reviewed_required' if required else 'broad',
    command=cmd, summary=summary,
    elapsed_sec=time.monotonic()-start, production_worktree_tested=True,
    motion_commands_sent=False, temperature_queried=False,
    passed=process.returncode==0 and before==after and summary.get('tests',0)>0
           and all(summary.get(k,0)==0 for k in ('failures','errors','skipped')))
(out/'result.json').write_text(json.dumps(receipt, indent=2)+'\n')
print(json.dumps(dict(passed=receipt['passed'], summary=summary,
    source_unchanged=before==after, result=str(out/'result.json'))), flush=True)
print('\n'.join((out/'pytest.log').read_text().splitlines()[-15:]), flush=True)
sys.exit(0 if receipt['passed'] else 1)
