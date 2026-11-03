"""Freeze staged code and retain offline-test evidence only on SanStand."""
import hashlib,json,os,subprocess,time
from pathlib import Path
import xml.etree.ElementTree as ET
work=Path(__file__).resolve().parents[1]
stage=work/'.runtime/contact_sdk_wait_staging_20261009'
pkg=stage/'src/alicia_flexible_grasp_supervisor'
out=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z/verification/contact_sdk_wait_final')
assert Path('/media/zhuyupei/SanStand').is_mount()
out.mkdir(exist_ok=False)
source_paths=[p for base in ('src','scripts') for p in (pkg/base).rglob('*.py')]
def hashes():return {str(p.relative_to(stage)):hashlib.sha256(p.read_bytes()).hexdigest() for p in source_paths}
before=hashes()
changed=['src/alicia_flexible_grasp_supervisor/'+p for p in [
 'src/alicia_flexible_grasp/robot/contact_sequence_precheck.py',
 'tests/test_contact_sample_collection.py','tests/test_contact_sequence_precheck.py']]
change_manifest={p:dict(staged_sha256=hashlib.sha256((stage/p).read_bytes()).hexdigest(),
 live_sha256=(hashlib.sha256((work/p).read_bytes()).hexdigest() if (work/p).exists() else None)) for p in changed}
tests=['test_contact_sample_collection.py','test_contact_sequence_precheck.py','test_stationary_following.py']
env=os.environ.copy();env.update(PYTHONDONTWRITEBYTECODE='1',PYTHONNOUSERSITE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',ROS_LOG_DIR=str(out/'ros'))
env['PYTHONPATH']=':'.join([str(pkg/'src'),str(pkg/'tests'),env.get('PYTHONPATH','')])
cmd=['/usr/bin/python3','-m','pytest','-q','-p','no:cacheprovider']+[str(pkg/'tests'/p) for p in tests]+['--junitxml',str(out/'tests.xml')]
started=time.time_ns()
with (out/'pytest.log').open('w') as f:p=subprocess.run(cmd,env=env,cwd=work,stdout=f,stderr=subprocess.STDOUT)
cases=list(ET.parse(str(out/'tests.xml')).getroot().iter('testcase'))
failed=[c.attrib for c in cases if c.find('failure') is not None or c.find('error') is not None]
result=dict(passed=p.returncode==0 and hashes()==before,returncode=p.returncode,staged=True,live_edited=False,
 source_unchanged=hashes()==before,tests=len(cases),test_nodeids=[c.get('classname')+'::'+c.get('name') for c in cases],failed=failed,
 command=cmd,pythonpath=env['PYTHONPATH'],source_sha256=before,changes=change_manifest,started_ns=started,ended_ns=time.time_ns(),
 boundaries='Offline actual message fixtures, original stationary_following_error, mocked ROS clocks/context; no live messages, no hardware commands; current GEN5 cause remains local gateway history not observed directly.')
(out/'result.json').write_text(json.dumps(result,indent=2,sort_keys=True)+'\n')
print(json.dumps({k:result[k] for k in ['passed','tests','failed','source_unchanged','changes']}))
raise SystemExit(p.returncode)
