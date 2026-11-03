"""Verify one fixed combined candidate scope without a ROS master."""
from pathlib import Path
import hashlib,json,os,subprocess,sys,time,xml.etree.ElementTree as ET
W=Path(__file__).resolve().parent.parent
C=W/'.runtime/gen14_arrival_repair_20261013'
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N=E/'ros_log/full_latest_contact_admission_20261013T011103Z'
assert Path('/media/zhuyupei/SanStand').is_mount()
O=N/'verification/gen14_measured_face_combined_memo_r1'
old=N/'verification/gen14_measured_face_author_r3/result.json'
prior=N/'verification/gen14_reporting_integrated_install/result.json'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
scope=sorted(set(json.loads(old.read_text())['test_scope'])|{'src/alicia_grasp_modes/tests/'+n for n in ('test_reference_texture_grouping.py','test_measured_face_precompute_independent.py','test_measurement_binding_memo.py','test_measurement_live_binding_independent.py')})
assert len(scope)==35
actual=json.loads(prior.read_text())['source_sha256']
assert all(sha(W/p)==h for p,h in actual.items())
paths=sorted(C.rglob('*.py'))+[Path(__file__)]
before={str(p):sha(p) for p in paths}
O.mkdir(exist_ok=False)
env=os.environ.copy();env.update(PYTHONNOUSERSITE='1',PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',ROS_MASTER_URI='http://127.0.0.1:1')
env['PYTHONPATH']=str(C/'src/alicia_grasp_modes/src')+':'+env.get('PYTHONPATH','')
cmd=[sys.executable,'-m','pytest','-q','-p','no:cacheprovider',*[str(C/p) for p in scope],'--junitxml='+str(O/'pytest.xml')]
start=time.monotonic()
with (O/'pytest.log').open('w') as log:r=subprocess.run(cmd,cwd=W,env=env,stdout=log,stderr=subprocess.STDOUT)
root=ET.parse(O/'pytest.xml').getroot();summary={k:sum(int(x.attrib.get(k,0)) for x in root.findall('testsuite')) for k in ('tests','failures','errors','skipped')}
after={str(p):sha(p) for p in paths};unchanged=all(sha(W/p)==h for p,h in actual.items())
result=dict(passed=r.returncode==0 and before==after and unchanged,summary=summary,returncode=r.returncode,elapsed_sec=time.monotonic()-start,source_sha256=before,source_sha256_after=after,source_unchanged=before==after,actual_unchanged=unchanged,actual_source_count=len(actual),prior_sha256=sha(prior),previous_scope_sha256=sha(old),pytest_xml_sha256=sha(O/'pytest.xml'),test_scope=scope,command=cmd,scope='Fixed 35-file common-face, train-only, dense grouping, source precompute, live binding, geometry/owner/deadline regression. No replay or physical success claim.',no_ros_master=True,motion_commands_sent=False,temperature_queried=False)
(O/'result.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:result[k] for k in ('passed','summary','source_unchanged','actual_unchanged','elapsed_sec')}),flush=True)
