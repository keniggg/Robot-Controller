import hashlib,json,os,subprocess,time,xml.etree.ElementTree as ET
from pathlib import Path
W=Path(__file__).resolve().parents[1];C=W/'.runtime/gen17_station_contact_current_wait_20261013'
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z');O=N/'verification/gen17_join_clock_pair_probe_r1'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
assert Path('/media/zhuyupei/SanStand').is_mount();O.mkdir();rel=N/'verification/gen16_success_first_release_manifest.json';sealed=json.loads(rel.read_text())['source_sha256'];assert all(sha(W/r)==v for r,v in sealed.items())
manifest=C/'delivery_manifest.json';manifest_sha=sha(manifest);files=json.loads(manifest.read_text())['files'];assert all(sha(C/f['path'])==f['candidate_sha256'] for f in files)
env=dict(os.environ,PYTHONNOUSERSITE='1',PYTHONDONTWRITEBYTECODE='1',ROS_MASTER_URI='http://127.0.0.1:1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',GEN17_PAIR_OUTPUT=str(O))
env['ROS_PACKAGE_PATH']=str(C/'src')+':'+str(W/'src')+':'+env.get('ROS_PACKAGE_PATH','')
env['PYTHONPATH']=':'.join(str(C/r) for r in ('src/alicia_grasp_modes/src','src/alicia_flexible_grasp_supervisor/src','src/alicia_grasp_modes/tests','src/alicia_flexible_grasp_supervisor/tests'))+':'+str(W/'devel/lib/python3/dist-packages')+':'+env.get('PYTHONPATH','')
test=W/'.runtime/test_gen17_join_clock_pair_probe_20261013.py'
with (O/'pytest.log').open('w') as log:p=subprocess.run(['python3','-m','pytest','-q','-p','no:cacheprovider',str(test),'--disable-warnings','--tb=short','--basetemp='+str(O/'tmp'),'--junitxml='+str(O/'pytest.xml')],cwd=W,env=env,stdout=log,stderr=subprocess.STDOUT)
r=dict(returncode=p.returncode,candidate_manifest=str(manifest),candidate_manifest_sha256=manifest_sha,candidate_unchanged=sha(manifest)==manifest_sha and all(sha(C/f['path'])==f['candidate_sha256'] for f in files),actual_unchanged=all(sha(W/r)==v for r,v in sealed.items()),test_sha256=sha(test),runner_sha256=sha(__file__),log_sha256=sha(O/'pytest.log'),xml_sha256=sha(O/'pytest.xml'),outputs={str(x):sha(x) for x in O.glob('*.json')},no_ros_or_motion=True,recorded_ns=time.time_ns());(O/'result.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2));assert p.returncode==0
