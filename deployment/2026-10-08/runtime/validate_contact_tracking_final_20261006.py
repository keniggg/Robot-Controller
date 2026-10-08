from datetime import datetime, timezone
from pathlib import Path
import hashlib, json, os, subprocess, sys
w=Path(__file__).resolve().parents[1];m=Path('/media/zhuyupei/SanStand');assert m.is_mount()
e=m/'实验记录/Robot-Controller/ros_log';out=e/('contact_tracking_final_validation_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'));out.mkdir()
robot='src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/'
files=['src/alicia_flexible_grasp_supervisor/scripts/motion_gateway_node.py', 'src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py',
'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/contact_execution.py', 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/contact_height_margin.py',
*[robot+n+'.py' for n in ['pregrasp_gateway','pregrasp_compensation','contact_tracking_contract','contracted_trajectory','moveit_planner']],
 'tools/mujoco_digital_twin_server.py']
prefixes=('test_pregrasp','test_contact','test_observation','test_contracted','test_endpoint','test_stationary','test_gateway_tracking','test_mujoco')
tests=sorted(str(p.relative_to(w)) for p in (w/'src/alicia_flexible_grasp_supervisor/tests').glob('test_*.py') if p.name.startswith(prefixes))
tests += ['src/alicia_grasp_modes/tests/test_unknown_execution_speed.py', '.runtime/full_startup_candidate/test_full_startup_candidate.py']
hashes={f:hashlib.sha256((w/f).read_bytes()).hexdigest() for f in files+tests}
args=[sys.executable,'-m','pytest','-q','--junitxml='+str(out/'pytest.xml'),*tests]
env=dict(os.environ,PYTHONPATH=str(w/'src/alicia_flexible_grasp_supervisor/tests')+os.pathsep+os.environ.get('PYTHONPATH',''))
with (out/'pytest.log').open('w') as log:r=subprocess.run(args,cwd=w,env=env,stdout=log,stderr=subprocess.STDOUT)
unchanged=all(hashlib.sha256((w/f).read_bytes()).hexdigest()==h for f,h in hashes.items())
report=dict(tests_passed=r.returncode==0 and unchanged,reviewed=False,production_files=files,files=hashes,tests=tests,returncode=r.returncode,source_unchanged=unchanged,log=str(out/'pytest.log'))
(out/'validation.json').write_text(json.dumps(report,indent=2)+'\n');(w/'.runtime/current_contact_tracking_validation_20261006.json').write_text(json.dumps(dict(manifest=str(out/'validation.json')),indent=2)+'\n')
print(json.dumps(dict(output=str(out),returncode=r.returncode,source_unchanged=unchanged)),flush=True);print((out/'pytest.log').read_text()[-5500:]);sys.exit(r.returncode or (0 if unchanged else 2))
