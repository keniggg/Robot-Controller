"""Run independent candidate-only ownership tests, binding every input hash."""
from pathlib import Path
import hashlib, json, os, subprocess, sys, time, xml.etree.ElementTree as ET
W=Path(__file__).resolve().parent.parent
C=W/'.runtime/gen15_qualified_phase_timestamp_20261013'
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
O=N/'verification'/sys.argv[1]
assert Path('/media/zhuyupei/SanStand').is_mount()
sha=lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
baseline=json.loads((C/'baseline_sha256.json').read_text())
assert all(sha(W/p)==h for p,h in baseline.items())
names=['src/alicia_grasp_modes/tests/test_qualified_phase_epoch_independent.py']
paths=sorted(C.rglob('*.py'))+[Path(__file__)]
before={str(p):sha(p) for p in paths}
env=os.environ.copy()
env.update(PYTHONNOUSERSITE='1',PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',ROS_MASTER_URI='http://127.0.0.1:1')
env['ROS_PACKAGE_PATH']=str(C/'src')+':'+env.get('ROS_PACKAGE_PATH','')
env['PYTHONPATH']=':'.join([str(C/'src/alicia_grasp_modes/src'),str(C/'src/alicia_flexible_grasp_supervisor/src'),str(C/'src/alicia_flexible_grasp_supervisor/tests'),env.get('PYTHONPATH','')])
O.mkdir(exist_ok=False)
cmd=[sys.executable,'-m','pytest','-q','-p','no:cacheprovider',*[str(C/p) for p in names],'--junitxml='+str(O/'pytest.xml')]
start=time.monotonic()
with (O/'pytest.log').open('w') as log:
    r=subprocess.run(cmd,cwd=W,env=env,stdout=log,stderr=subprocess.STDOUT)
root=ET.parse(O/'pytest.xml').getroot()
summary={k:sum(int(x.attrib.get(k,0)) for x in root.findall('testsuite')) for k in ('tests','failures','errors','skipped')}
after={str(p):sha(p) for p in paths}
actual_unchanged=all(sha(W/p)==h for p,h in baseline.items())
result=dict(passed=r.returncode==0 and before==after and actual_unchanged,summary=summary,returncode=r.returncode,elapsed_sec=time.monotonic()-start,source_sha256=before,source_sha256_after=after,source_unchanged=before==after,actual_unchanged=actual_unchanged,actual_source_count=len(baseline),test_scope=names,command=cmd,pytest_xml_sha256=sha(O/'pytest.xml'),candidate_only=True,no_ros_master=True,motion_commands_sent=False,temperature_queried=False,scope='Independent translated Unix epoch fixtures; actual qualified task phase setter/reuse, strict owner and integer/monotonic deadlines, real observed-contact audit builder and task reader. No replay timing or physical success claim.')
(O/'result.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:result[k] for k in ('passed','summary','source_unchanged','actual_unchanged','elapsed_sec')}),flush=True)
