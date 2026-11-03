from pathlib import Path
import os,sys,json,subprocess,hashlib,time,xml.etree.ElementTree as ET
W=Path(__file__).resolve().parents[1];N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen22_restart_20261015T003700Z/verification')
assert Path('/media/zhuyupei/SanStand').is_mount()
out=N/sys.argv[1];out.mkdir()
for n in ('tmp','ros'):(out/n).mkdir()
env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONNOUSERSITE='1',PYTEST_DISABLE_PLUGIN_AUTOLOAD='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',ROS_MASTER_URI='http://127.0.0.1:1',TMPDIR=str(out/'tmp'),ROS_LOG_DIR=str(out/'ros'))
env['PYTHONPATH']=':'.join(str(W/p) for p in ('src/alicia_flexible_grasp_supervisor/tests','src/alicia_grasp_modes/tests','src/alicia_grasp_modes/src','src/alicia_flexible_grasp_supervisor/src','devel/lib/python3/dist-packages'))+':'+env.get('PYTHONPATH','')
env['ROS_PACKAGE_PATH']=str(W/'src')+':'+env.get('ROS_PACKAGE_PATH','')
rows=[]
for which in sys.argv[2:]:
 cmd=[sys.executable,'-B',str(W/'.runtime/probe_gen30_prewarm_compat_case_r2.py'),which,str(out/which)]
 with (out/(which+'.log')).open('x') as f:r=subprocess.run(cmd,cwd=W,env=env,stdout=f,stderr=subprocess.STDOUT)
 xml=out/which/'pytest.xml'
 count={k:sum(int(r.get(k,0)) for r in ET.parse(xml).getroot().iter('testsuite')) for k in ('tests','failures','errors','skipped')} if xml.exists() else None
 rows.append(dict(which=which,returncode=r.returncode,summary=count));print(json.dumps(rows[-1]),flush=True)
paths=['src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py','.runtime/gen30_baseline_source/src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py','src/alicia_grasp_modes/tests/test_measured_face_freeze_lock_independent.py','.runtime/probe_gen30_prewarm_compat_case_r2.py','.runtime/run_gen30_prewarm_compat_r2.py']
result=dict(rows=rows,source_sha256={p:hashlib.sha256((W/p).read_bytes()).hexdigest() for p in paths},no_ros=True,no_hardware=True)
(out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
print(out)
