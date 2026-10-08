from pathlib import Path
from datetime import datetime,timezone
import json,os,hashlib,subprocess,sys
w=Path(__file__).resolve().parents[1];m=Path('/media/zhuyupei/SanStand');assert m.is_mount()
out=m/'实验记录/Robot-Controller/ros_log'/('observation_identity_validation_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'));out.mkdir()
production=['src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_remote.py','src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py','src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_visual.py','src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py']
tests=sorted([str(p.relative_to(w)) for p in (w/'src/alicia_grasp_modes/tests').glob('test_*.py') if p.name.startswith(('test_observ','test_observe','test_unknown_execution','test_remote_modes','test_diagnostic_observation','test_runner_observe_return'))])
tests += ['src/alicia_flexible_grasp_supervisor/tests/'+n for n in ['test_observation_rejection_summary.py','test_observation_distance_search.py','test_observation_tilt_repair.py','test_remote_grasp6d_streaming.py','test_contact_height_audit_binding.py']]
assert all((w/f).is_file() for f in production+tests)
hashes={f:hashlib.sha256((w/f).read_bytes()).hexdigest() for f in production+tests}
env=dict(os.environ,PYTHONPATH=os.pathsep.join([str(w/'src/alicia_flexible_grasp_supervisor/tests'),str(w/'src/alicia_grasp_modes/tests'),os.environ.get('PYTHONPATH','')]))
with (out/'pytest.log').open('w') as log:r=subprocess.run([sys.executable,'-m','pytest','-q','--junitxml='+str(out/'pytest.xml'),*tests],cwd=w,env=env,stdout=log,stderr=subprocess.STDOUT)
unchanged=all(hashlib.sha256((w/f).read_bytes()).hexdigest()==h for f,h in hashes.items())
report=dict(tests_passed=r.returncode==0 and unchanged,reviewed=False,production_files=production,files=hashes,tests=tests,returncode=r.returncode,source_unchanged=unchanged,log=str(out/'pytest.log'))
(out/'validation.json').write_text(json.dumps(report,indent=2)+'\n');(w/'.runtime/current_observation_identity_validation_20261006.json').write_text(json.dumps(dict(manifest=str(out/'validation.json')),indent=2)+'\n')
print(json.dumps(dict(directory=str(out),returncode=r.returncode,unchanged=unchanged)),flush=True);print((out/'pytest.log').read_text()[-6000:],flush=True);sys.exit(r.returncode)
