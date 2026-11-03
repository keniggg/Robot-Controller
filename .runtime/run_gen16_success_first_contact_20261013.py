import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

W = Path(__file__).resolve().parent.parent
C = W/'.runtime/gen16_success_first_contact_20261013'
N = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
O = N/'verification'/sys.argv[1]
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
baseline = json.loads((C/'baseline_sha256.json').read_text())
assert all(sha(W/p) == h for p,h in baseline.items())
before = {str(p):sha(p) for p in sorted(C.rglob('*.py'))}
before[str(Path(__file__))] = sha(__file__)
t = C/'src/alicia_flexible_grasp_supervisor/tests'
selected = [str(t/'test_contact_success_first_selection.py')]
selected += [str(t/'test_remote_grasp6d_streaming.py')+'::'+name for name in (
    'test_contact_first_complete_candidate_stops_comparison_and_preserves_phase',
    'test_contact_rejected_result_does_not_start_incumbent_budget',
    'test_contact_comparison_cleans_private_deadline_after_error',
    'test_direct_near_field_uses_current_request_and_bounded_comparison_without_mujoco',
    'test_direct_near_field_strict_timeout_does_not_start_orientation_resolver',
    'test_direct_near_field_deadline_uses_phase_contract_not_snapshot')]
env = os.environ.copy()
env.update(PYTHONDONTWRITEBYTECODE='1',PYTHONNOUSERSITE='1',
    OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',ROS_MASTER_URI='http://127.0.0.1:1')
env['PYTHONPATH'] = ':'.join([str(t),str(W/'src/alicia_flexible_grasp_supervisor/tests'),
    str(W/'src/alicia_flexible_grasp_supervisor/src'),str(W/'devel/lib/python3/dist-packages'),
    env.get('PYTHONPATH','')])
env['ROS_PACKAGE_PATH'] = str(W/'src')+':'+env.get('ROS_PACKAGE_PATH','')
O.mkdir(exist_ok=False)
args=['-q','-p','no:cacheprovider',*selected,'--junitxml='+str(O/'pytest.xml')]
bootstrap='''import json, pathlib, sys, pytest
result=pytest.main(json.loads(sys.argv[1]))
import test_remote_grasp6d_streaming as suite
loaded=str(pathlib.Path(suite.remote_node.__file__).resolve())
pathlib.Path(sys.argv[3]).write_text(json.dumps({'remote':loaded},indent=2)+'\\n')
assert pathlib.Path(sys.argv[2]).resolve() in pathlib.Path(loaded).parents, loaded
sys.exit(result)
'''
cmd=[sys.executable,'-c',bootstrap,json.dumps(args),str(C),str(O/'loaded.json')]
started=time.monotonic()
with (O/'pytest.log').open('w') as f:
    child=subprocess.run(cmd,cwd=W,env=env,stdout=f,stderr=subprocess.STDOUT)
xml=ET.parse(O/'pytest.xml').getroot()
summary={k:sum(int(x.attrib.get(k,0)) for x in xml.findall('testsuite'))
         for k in ('tests','failures','errors','skipped')}
after={p:sha(p) for p in before}
actual_unchanged=all(sha(W/p)==h for p,h in baseline.items())
report=dict(passed=child.returncode==0 and before==after and actual_unchanged,
    returncode=child.returncode,summary=summary,elapsed_sec=time.monotonic()-started,
    candidate_source_sha256=before,after_sha256=after,actual_baseline_sha256=baseline,
    actual_unchanged=actual_unchanged,candidate_unchanged=before==after,
    loaded=(json.loads((O/'loaded.json').read_text()) if (O/'loaded.json').exists() else None),selected=selected,
    log_sha256=sha(O/'pytest.log'),xml_sha256=sha(O/'pytest.xml'),
    no_ros_master=True,no_physical_action=True,temperature_queried=False,
    scope='Real bounded selection loop and unchanged publication-path regressions; synthetic strict replies do not prove an actual grasp candidate exists.')
(O/'result.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(dict(result=str(O/'result.json'),sha256=sha(O/'result.json'),
    passed=report['passed'],summary=summary,elapsed_sec=report['elapsed_sec'])))
