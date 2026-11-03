import json,os,sys,subprocess,hashlib,xml.etree.ElementTree as ET
from pathlib import Path
W=Path(__file__).resolve().parents[1];C=W/'.runtime/gen17_frozen_station_handoff_probe_20261013'
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
O=N/'verification'/sys.argv[1];O.mkdir()
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
actual=json.loads((N/'verification/gen16_success_first_release_manifest.json').read_text())['source_sha256']
assert all(sha(W/r)==h for r,h in actual.items())
files={str(p.relative_to(C)):sha(p) for p in C.rglob('*.py')}
env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONNOUSERSITE='1',ROS_MASTER_URI='http://127.0.0.1:1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
env['ROS_PACKAGE_PATH']=str(C/'src')+':'+str(W/'src')+':'+env.get('ROS_PACKAGE_PATH','')
env['PYTHONPATH']=':'.join(str(C/r) for r in ['src/alicia_grasp_modes/src','src/alicia_flexible_grasp_supervisor/src','src/alicia_grasp_modes/tests','src/alicia_flexible_grasp_supervisor/tests'])+':'+str(W/'devel/lib/python3/dist-packages')+':'+env.get('PYTHONPATH','')
args=[sys.executable,'-m','pytest','-q','-p','no:cacheprovider',str(C/'src/alicia_grasp_modes/tests/test_frozen_station_handoff_probe.py'),'--basetemp='+str(O/'tmp'),'--junitxml='+str(O/'pytest.xml'),'--tb=short']
with (O/'pytest.log').open('w') as f:r=subprocess.run(args,cwd=W,env=env,stdout=f,stderr=subprocess.STDOUT)
x=ET.parse(O/'pytest.xml').getroot();summary={k:sum(int(s.attrib.get(k,0)) for s in x.iter('testsuite')) for k in ['tests','failures','errors','skipped']}
result=dict(reproduced=r.returncode==0,summary=summary,source_sha256=files,source_unchanged=all(sha(C/r)==h for r,h in files.items()),actual_unchanged=all(sha(W/r)==h for r,h in actual.items()),actual_sources=len(actual),log_sha256=sha(O/'pytest.log'),xml_sha256=sha(O/'pytest.xml'),diagnoses=[json.loads(p.read_text()) for p in (O/'tmp').rglob('diagnosis.json')],command=args,runner_sha256=sha(__file__),no_ros_master=True,no_actions=True,synthetic_fixture=True)
(O/'result.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(dict(result=str(O/'result.json'),sha256=sha(O/'result.json'),summary=summary,reproduced=result['reproduced'])))
sys.exit(r.returncode)
