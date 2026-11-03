#!/usr/bin/env python3
import hashlib,json,os,shutil,subprocess,sys,time,xml.etree.ElementTree as ET
from pathlib import Path
W=Path(__file__).resolve().parents[1]
C=W/'.runtime/gen17_remote_receipt_current_wait_r2_20261013'
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
O=N/'verification'/sys.argv[1]
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
baseline=json.loads((C/'baseline_sha256.json').read_text())
assert all(sha(W/r)==h for r,h in baseline.items())
assert Path('/media/zhuyupei/SanStand').is_mount()
O.mkdir()
before={str(p.relative_to(C)):sha(p) for p in C.rglob('*.py')}
testrel='src/alicia_grasp_modes/tests/test_station_contact_remote_current_wait.py'
production='src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_remote.py'
def run(label,root,scope):
 out=O/label;out.mkdir()
 env=dict(os.environ,PYTHONNOUSERSITE='1',PYTHONDONTWRITEBYTECODE='1',ROS_MASTER_URI='http://127.0.0.1:1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
 env['ROS_PACKAGE_PATH']=str(root/'src')+':'+str(W/'src')+':'+env.get('ROS_PACKAGE_PATH','')
 env['PYTHONPATH']=':'.join(str(root/r) for r in ('src/alicia_grasp_modes/src','src/alicia_flexible_grasp_supervisor/src','src/alicia_grasp_modes/tests','src/alicia_flexible_grasp_supervisor/tests'))+':'+str(W/'devel/lib/python3/dist-packages')+':'+env.get('PYTHONPATH','')
 args=['-q','-p','no:cacheprovider',*scope,'--disable-warnings','--tb=short','--basetemp='+str(out/'tmp'),'--junitxml='+str(out/'pytest.xml')]
 bootstrap='''import pathlib,json,pytest,sys
rc=pytest.main(json.loads(sys.argv[1]))
from alicia_grasp_modes import station_compute_wait_remote as implementation
actual=str(pathlib.Path(implementation.__file__).resolve())
pathlib.Path(sys.argv[3]).write_text(json.dumps({'remote':actual},indent=2)+'\\n')
assert actual==sys.argv[2],(actual,sys.argv[2])
sys.exit(rc)
'''
 start=time.monotonic()
 with (out/'pytest.log').open('w') as log:
  child=subprocess.run([sys.executable,'-c',bootstrap,json.dumps(args),str(root/production),str(out/'loaded.json')],cwd=W,env=env,stdout=log,stderr=subprocess.STDOUT)
 xml=ET.parse(out/'pytest.xml').getroot();counts={k:sum(int(s.attrib.get(k,0)) for s in xml.iter('testsuite')) for k in ('tests','failures','errors','skipped')}
 result=dict(returncode=child.returncode,summary=counts,elapsed_sec=time.monotonic()-start,command=args,log_sha256=sha(out/'pytest.log'),xml_sha256=sha(out/'pytest.xml'),loaded=json.loads((out/'loaded.json').read_text()))
 print(label,counts,flush=True);return result
bt=O/'baseline_test';bt.mkdir();shutil.copyfile(C/testrel,bt/Path(testrel).name)
control=run('baseline',W,[str(bt/Path(testrel).name)+'::test_receipt_context_crossing_releases_both_locks_then_uses_real_successor[geometry_ros]'])
scopes=[str(C/testrel),str(C/'src/alicia_grasp_modes/tests/test_gen11_contact_remote_receipt.py'),str(C/'src/alicia_grasp_modes/tests/test_station_compute_wait_remote.py'),str(C/'src/alicia_grasp_modes/tests/test_station_compute_work_handoff.py')]
generation_control=run('prior_remote_generation',W/'.runtime/gen17_remote_receipt_current_wait_20261013',[str(bt/Path(testrel).name)+'::test_original_preserve_identity_generation_change_cancels_receipt_wait'])
candidate=run('candidate',C,scopes)
result=dict(passed=control['returncode']==1 and control['summary']==dict(tests=1,failures=1,errors=0,skipped=0) and candidate['returncode']==0 and generation_control['returncode']==1 and generation_control['summary']==dict(tests=1,failures=1,errors=0,skipped=0),
 baseline=control,prior_remote_generation=generation_control,candidate=candidate,candidate_sha256=before,
 source_unchanged=all(sha(C/r)==h for r,h in before.items()),actual_unchanged=all(sha(W/r)==h for r,h in baseline.items()),actual_source_count=len(baseline),
 runner_sha256=sha(__file__),no_ros_master=True,no_motion_commands=True,physical_grasp_verified=False,
 limits=['Synthetic fixture pixels and plan geometry test timing/provenance, not actual grasp quality.','Real context/wait/quarantine/three-topic callbacks execute; context wrapper advances clock after real checks rather than returning success.'])
result['passed'] &=result['source_unchanged'] and result['actual_unchanged']
(O/'result.json').write_text(json.dumps(result,indent=2)+'\n')
print('RESULT',result['passed'],str(O/'result.json'),sha(O/'result.json'),flush=True)
assert result['passed']
