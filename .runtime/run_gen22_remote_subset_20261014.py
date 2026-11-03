import os,sys,json,hashlib,subprocess,time
from pathlib import Path
W=Path.cwd()
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen18_restart_20261014T012216Z/verification/gen22_qualified_rgbd_remote')
out=N/sys.argv[1]
out.mkdir()
for d in ('tmp','ros'): (out/d).mkdir()
files=['src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_remote.py','src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py','src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_remote.py','src/alicia_grasp_modes/tests/test_gen22_qualified_rgbd_remote.py']
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
versions={s:sha(s) for s in files}
env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONNOUSERSITE='1',PYTEST_DISABLE_PLUGIN_AUTOLOAD='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',ROS_MASTER_URI='http://127.0.0.1:1',TMPDIR=str(out/'tmp'),ROS_LOG_DIR=str(out/'ros'))
env['PYTHONPATH']=':'.join(str(W/p) for p in ('src/alicia_flexible_grasp_supervisor/tests','src/alicia_grasp_modes/tests','src/alicia_grasp_modes/src','src/alicia_flexible_grasp_supervisor/src','devel/lib/python3/dist-packages'))+':'+env.get('PYTHONPATH','')
env['ROS_PACKAGE_PATH']=str(W/'src')+':'+env.get('ROS_PACKAGE_PATH','')
tests=sys.argv[2:] or [files[-1]]
cmd=[sys.executable,'-m','pytest','-q',*tests,'-p','no:cacheprovider','--tb=short','--basetemp='+str(out/'pytest_tmp'),'--junitxml='+str(out/'pytest.xml')]
start=time.monotonic()
with (out/'pytest.log').open('w') as f: r=subprocess.run(cmd,env=env,stdout=f,stderr=subprocess.STDOUT)
receipt=dict(returncode=r.returncode,elapsed=time.monotonic()-start,source_sha256=versions,source_unchanged=all(sha(p)==v for p,v in versions.items()),log_sha256=sha(out/'pytest.log'),command=cmd)
(out/'receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
print((out/'pytest.log').read_text()[-14000:]);print(json.dumps(receipt))
sys.exit(r.returncode)
