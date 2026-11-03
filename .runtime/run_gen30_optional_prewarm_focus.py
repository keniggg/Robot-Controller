"""Offline bounded regression of optional prewarm and unchanged recovery."""
from pathlib import Path
import os,sys,json,subprocess,hashlib,time,xml.etree.ElementTree as ET
W=Path(__file__).resolve().parents[1];N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen22_restart_20261015T003700Z/verification')
assert Path('/media/zhuyupei/SanStand').is_mount()
out=N/'gen30_optional_prewarm_fix_r1';out.mkdir()
for n in ('tmp','ros'):(out/n).mkdir()
tests=['src/alicia_grasp_modes/tests/'+n+'.py' for n in ('test_measured_face_freeze_lock_independent','test_gen30_optional_topology_prewarm','test_gen30_source_sampling_topology','test_gen30_source_connected_owner','test_gen30_source_component_independent')]
paths=tests+['src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py','src/alicia_grasp_modes/src/alicia_grasp_modes/source_sampling_topology.py','src/alicia_grasp_modes/src/alicia_grasp_modes/source_connected_owner.py','.runtime/run_gen30_optional_prewarm_focus.py']
def hashes():return {p:hashlib.sha256((W/p).read_bytes()).hexdigest() for p in paths}
before=hashes()
env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONNOUSERSITE='1',PYTEST_DISABLE_PLUGIN_AUTOLOAD='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',ROS_MASTER_URI='http://127.0.0.1:1',TMPDIR=str(out/'tmp'),ROS_LOG_DIR=str(out/'ros'))
env['PYTHONPATH']=':'.join(str(W/p) for p in ('src/alicia_flexible_grasp_supervisor/tests','src/alicia_grasp_modes/tests','src/alicia_grasp_modes/src','src/alicia_flexible_grasp_supervisor/src','devel/lib/python3/dist-packages'))+':'+env.get('PYTHONPATH','')
env['ROS_PACKAGE_PATH']=str(W/'src')+':'+env.get('ROS_PACKAGE_PATH','')
cmd=[sys.executable,'-B','-m','pytest','-q',*tests,'-p','no:cacheprovider','--tb=short','--basetemp='+str(out/'pytest_tmp'),'--junitxml='+str(out/'pytest.xml')]
start=time.monotonic()
with (out/'pytest.log').open('x') as f:r=subprocess.run(cmd,cwd=W,env=env,stdout=f,stderr=subprocess.STDOUT)
xml=out/'pytest.xml';count={k:sum(int(row.get(k,0)) for row in ET.parse(xml).getroot().iter('testsuite')) for k in ('tests','failures','errors','skipped')}
result=dict(returncode=r.returncode,summary=count,elapsed_sec=time.monotonic()-start,source_sha256=before,source_unchanged=before==hashes(),command=cmd,no_ros=True,no_hardware=True)
(out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result));print((out/'pytest.log').read_text());sys.exit(r.returncode)
