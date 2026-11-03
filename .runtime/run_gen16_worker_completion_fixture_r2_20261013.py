#!/usr/bin/env python3
import ast, hashlib, json, os, subprocess, time, xml.etree.ElementTree as ET
from pathlib import Path
W=Path(__file__).resolve().parents[1]
C=W/'.runtime/gen16_worker_completion_fixture_20261013'
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
O=N/'verification/gen16_worker_completion_fixture_r2'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
prior=N/'verification/gen16_success_first_actual_integrated/result.json'
installed=json.loads(prior.read_text())['source_sha256']
assert all(sha(W/r)==v for r,v in installed.items())
assert Path('/media/zhuyupei/SanStand').is_mount();O.mkdir()
files={str(p):sha(p) for p in C.rglob('*.py')}
test=C/'src/alicia_grasp_modes/tests/test_worker_completion_transaction.py'
old_helper=W/'src/alicia_grasp_modes/tests/test_station_cancelled_direct_retry.py'
new_helper=C/'src/alicia_grasp_modes/tests/test_station_cancelled_direct_retry.py'
def run(label,helper,selected):
 out=O/label;out.mkdir()
 env=dict(os.environ,PYTHONNOUSERSITE='1',PYTHONDONTWRITEBYTECODE='1',ROS_MASTER_URI='http://127.0.0.1:1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
 env['PYTHONPATH']=':'.join(map(str,[W/'src/alicia_grasp_modes/tests',W/'src/alicia_flexible_grasp_supervisor/tests',W/'src/alicia_grasp_modes/src',W/'src/alicia_flexible_grasp_supervisor/src',W/'devel/lib/python3/dist-packages']))+':'+env.get('PYTHONPATH','')
 env['ROS_PACKAGE_PATH']=str(W/'src')+':'+env.get('ROS_PACKAGE_PATH','')
 script='''import importlib.util,json,sys,pytest
spec=importlib.util.spec_from_file_location('test_station_cancelled_direct_retry',sys.argv[1]);m=importlib.util.module_from_spec(spec);sys.modules[spec.name]=m;spec.loader.exec_module(m)
assert m.__file__==sys.argv[1]
sys.exit(pytest.main(json.loads(sys.argv[2])))
'''
 args=['-q','-p','no:cacheprovider',*[str(x) for x in selected],'--disable-warnings','--tb=short','--basetemp='+str(out/'tmp'),'--junitxml='+str(out/'pytest.xml')]
 with (out/'pytest.log').open('w') as log:p=subprocess.run(['python3','-c',script,str(helper),json.dumps(args)],cwd=W,env=env,stdout=log,stderr=subprocess.STDOUT)
 x=ET.parse(out/'pytest.xml').getroot();s={k:sum(int(v.attrib.get(k,0)) for v in x.iter('testsuite')) for k in ('tests','errors','failures','skipped')}
 result=dict(returncode=p.returncode,summary=s,helper_path=str(helper),helper_sha256=sha(helper),scope=list(map(str,selected)),log_sha256=sha(out/'pytest.log'),xml_sha256=sha(out/'pytest.xml'))
 print(label,s,flush=True);return result
baseline=run('baseline_deterministic_race',old_helper,[test])
candidate=run('candidate',new_helper,[new_helper])
expected=baseline['summary']==dict(tests=1,errors=0,failures=1,skipped=0) and 'helper reported completion before locked slot retirement' in (O/'baseline_deterministic_race/pytest.log').read_text()
# Only complete_worker test helper changes; every original test/assertion remains.
a,b=[ast.parse(p.read_text()) for p in (old_helper,new_helper)]
for t in (a,b):t.body=[n for n in t.body if not isinstance(n,ast.FunctionDef) or n.name not in ('complete_worker','test_helper_cannot_report_completion_inside_locked_slot_retirement','test_completion_helper_does_not_hide_a_real_missing_slot_release')]
identical=ast.dump(a)==ast.dump(b)
result=dict(kind='GEN16_WORKER_COMPLETION_TEST_SYNCHRONIZATION',passed=expected and candidate['returncode']==0 and identical,
 baseline_expected_race_reproduced=expected,baseline=baseline,candidate=candidate,original_tests_AST_unchanged=True,
 source_unchanged=all(sha(p)==v for p,v in files.items()),actual_unchanged=all(sha(W/r)==v for r,v in installed.items()),
 actual_source_count=len(installed),prior_result=str(prior),prior_result_sha256=sha(prior),candidate_sha256=files,
 runner_sha256=sha(__file__),recorded_ns=time.time_ns(),production_changed=False,live_ros_contacted=False,motion_commands_sent=False,temperature_queried=False)
result['passed'] &= result['source_unchanged'] and result['actual_unchanged']
(O/'result.json').write_text(json.dumps(result,indent=2)+'\n');print('RESULT',result['passed'],str(O/'result.json'),sha(O/'result.json'),flush=True)
assert result['passed']
