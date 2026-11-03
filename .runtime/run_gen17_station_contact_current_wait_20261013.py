#!/usr/bin/env python3
import hashlib,json,os,subprocess,time,xml.etree.ElementTree as ET
from pathlib import Path
W=Path(__file__).resolve().parents[1];C=W/'.runtime/gen17_station_contact_current_wait_20261013'
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
O=N/'verification'/os.environ.get('GEN17_WAIT_RUN','gen17_station_contact_current_wait_r1')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
release=N/'verification/gen16_success_first_release_manifest.json';sealed=json.loads(release.read_text())['source_sha256']
assert all(sha(W/r)==v for r,v in sealed.items());assert Path('/media/zhuyupei/SanStand').is_mount();O.mkdir()
files={str(p.relative_to(C)):sha(p) for p in C.rglob('*.py')}
SCOPE=['src/alicia_grasp_modes/tests/'+n for n in ('test_station_contact_current_wait.py','test_station_contact_admission.py','test_gen11_contact_compute_admission_independent.py','test_station_compute_wait_task.py')]
def run(label,root,scope):
 out=O/label;out.mkdir();env=dict(os.environ,PYTHONNOUSERSITE='1',PYTHONDONTWRITEBYTECODE='1',ROS_MASTER_URI='http://127.0.0.1:1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
 env['ROS_PACKAGE_PATH']=str(root/'src')+':'+str(W/'src')+':'+env.get('ROS_PACKAGE_PATH','')
 env['PYTHONPATH']=':'.join(str(root/r) for r in ('src/alicia_grasp_modes/src','src/alicia_flexible_grasp_supervisor/src','src/alicia_grasp_modes/tests','src/alicia_flexible_grasp_supervisor/tests'))+':'+str(W/'devel/lib/python3/dist-packages')+':'+env.get('PYTHONPATH','')
 args=['python3','-m','pytest','-q','-p','no:cacheprovider',*scope,'--disable-warnings','--tb=short','--basetemp='+str(out/'tmp'),'--junitxml='+str(out/'pytest.xml')]
 with (out/'pytest.log').open('w') as log:p=subprocess.run(args,cwd=W,env=env,stdout=log,stderr=subprocess.STDOUT)
 x=ET.parse(out/'pytest.xml').getroot();counts={k:sum(int(s.attrib.get(k,0)) for s in x.iter('testsuite')) for k in ('tests','errors','failures','skipped')}
 result=dict(returncode=p.returncode,summary=counts,command=args,log_sha256=sha(out/'pytest.log'),xml_sha256=sha(out/'pytest.xml'));print(label,result['summary'],flush=True);return result
focused=run('focused',C,[str(C/r) for r in SCOPE])
result=dict(passed=focused['returncode']==0,focused=focused,candidate_root=str(C),test_scope=SCOPE,candidate_sha256=files,source_unchanged=all(sha(C/r)==v for r,v in files.items()),actual_unchanged=all(sha(W/r)==v for r,v in sealed.items()),actual_source_count=len(sealed),release=str(release),release_sha256=sha(release),runner_sha256=sha(__file__),recorded_ns=time.time_ns(),live_ros_contacted=False,motion_commands_sent=False,physical_grasp_verified=False)
result['passed'] &=result['source_unchanged'] and result['actual_unchanged'];(O/'result.json').write_text(json.dumps(result,indent=2)+'\n');print('RESULT',result['passed'],str(O/'result.json'),sha(O/'result.json'),flush=True)
assert result['passed']
