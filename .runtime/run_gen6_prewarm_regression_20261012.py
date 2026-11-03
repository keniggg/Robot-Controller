"""Offline combined runner/selection regression with immutable source evidence."""
import hashlib,json,os,subprocess,time,xml.etree.ElementTree as ET
from pathlib import Path
w=Path(__file__).resolve().parents[1]
s=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_frozen_no_skin_20261012T021050Z')
assert Path('/media/zhuyupei/SanStand').is_mount()
out=s/'verification/gen6_prewarm_integrated';out.mkdir(exist_ok=False)
installed=json.loads((s/'verification/gen6_prewarm_install/result.json').read_text())
expected=installed['source_sha256']
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
assert all(sha(w/p)==h for p,h in expected.items())
scope=['src/alicia_grasp_modes/tests/'+name+'.py' for name in (
    'test_mode_runner','test_runner_target_loss','test_runner_promotion_rebinding',
    'test_runner_expected_selection','test_runner_observe_return','test_launcher_selection_commit',
    'test_router_contract','test_selection','test_prestream_acquisition_identity','test_initial_station_identity_template')]
scope+=['.runtime/gen6_runner_prewarm_20261012/tests/test_runner_prewarm.py','.runtime/gen6_runner_prewarm_20261012/tests/test_gen6_initial_template_late_start.py']
# The subprocess test imports the immutable candidate helper. Its exact bytes
# must equal the installed helper before and after the combined regression.
helper='.runtime/prewarm_mode_grasp_runner_20261012.py'
candidate=w/'.runtime/gen6_runner_prewarm_20261012/candidate'/helper
assert sha(candidate)==expected[helper]
env=dict(os.environ,PYTHONNOUSERSITE='1',PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',ROS_MASTER_URI='http://127.0.0.1:1',TMPDIR='/tmp',GEN6_TEST_WORKTREE=str(w),GEN10_TEST_LAUNCHER=str(w/'.runtime/run_reboot_realigned_grasp_20261010.py'))
env['PYTHONPATH']=':'.join(str(w/p) for p in ('src/alicia_flexible_grasp_supervisor/src','src/alicia_grasp_modes/src','src/alicia_flexible_grasp_supervisor/tests','src/alicia_grasp_modes/tests'))+':'+env.get('PYTHONPATH','')
env['ROS_PACKAGE_PATH']=str(w/'src')+':'+env.get('ROS_PACKAGE_PATH','')
cmd=['/usr/bin/python3','-m','pytest','-q',*scope,'-p','no:cacheprovider','--junitxml='+str(out/'pytest.xml')]
start=time.monotonic()
with (out/'pytest.log').open('x') as log:p=subprocess.run(cmd,cwd=w,env=env,stdout=log,stderr=subprocess.STDOUT)
after={p:sha(w/p) for p in expected};summary={}
for suite in ET.parse(out/'pytest.xml').getroot().iter('testsuite'):
 for key in ('tests','failures','errors','skipped'):summary[key]=summary.get(key,0)+int(suite.attrib.get(key,0))
passed=p.returncode==0 and after==expected and sha(candidate)==expected[helper] and summary['tests']>0 and not any(summary[k] for k in ('failures','errors','skipped'))
result=dict(passed=passed,returncode=p.returncode,source_sha256=expected,source_after_sha256=after,source_unchanged=after==expected,test_scope=scope,command=cmd,summary=summary,elapsed_sec=time.monotonic()-start,installed_launcher_tested=True,subprocess_helper_tested='byte-identical immutable candidate and installed helper verified',motion_commands_sent=False,temperature_queried=False)
(out/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(dict(passed=passed,summary=summary,elapsed_sec=result['elapsed_sec'],result=str(out/'result.json'))),flush=True)
print('\n'.join((out/'pytest.log').read_text().splitlines()[-10:]),flush=True)
raise SystemExit(0 if passed else 1)
