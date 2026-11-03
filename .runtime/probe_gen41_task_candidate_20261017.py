"""Run task candidate imports in memory; production stays the GEN40 release."""
import hashlib, importlib, json, os
from pathlib import Path
import shutil, sys, time
W=Path(__file__).resolve().parents[1]
D=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen36_restart_20261017T012037Z')
O=Path(sys.argv[1]); O.relative_to(D/'verification')
assert Path('/media/zhuyupei/SanStand').is_mount() and not O.exists()
O.mkdir(); (O/'tmp').mkdir(); (O/'ros').mkdir(); (O/'sources').mkdir()
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
manifest=D/'verification/gen40_release_manifest.json'
assert sha(manifest)=='7d5e8042e8dc4c5735109b6e8d73dc0ab78436f720d51eed29f886f97a2f1c63'
base=json.loads(manifest.read_text())['source_sha256']
assert len(base)==1029 and all(sha(W/n)==h for n,h in base.items())
modules=('station_compute_reacquisition','station_contact_admission','station_compute_wait_task')
snap={}
for name in modules:
 src=W/'.runtime/gen41_candidate_source/src/alicia_grasp_modes/src/alicia_grasp_modes'/f'{name}.py'
 dst=O/'sources'/f'{name}.py';shutil.copyfile(src,dst);snap[name]=dict(source=str(src),sha256=sha(dst))
os.environ.update(TMPDIR=str(O/'tmp'),ROS_LOG_DIR=str(O/'ros'),OFFLINE_BOUNDARY_LOG=str(O/'boundary_attempts.jsonl'))
for name in modules:
 module=importlib.import_module('alicia_grasp_modes.'+name)
 exec(compile((O/'sources'/f'{name}.py').read_text(),str(O/'sources'/f'{name}.py'),'exec'),module.__dict__)
import pytest
scope=['src/alicia_grasp_modes/tests/test_gen41_task_availability_wait.py','src/alicia_grasp_modes/tests/test_station_compute_wait_task.py','src/alicia_grasp_modes/tests/test_arrival_feedback_gap_late_consumers.py']
started=time.monotonic()
code=pytest.main(['-q','--rootdir='+str(W),*scope,'-p','no:cacheprovider','--tb=short','--basetemp='+str(O/'pytest_tmp'),'--junitxml='+str(O/'pytest.xml'),'-o','junit_family=legacy'])
changed=[n for n,h in base.items() if sha(W/n)!=h]
boundary=(O/'boundary_attempts.jsonl').read_text().splitlines() if (O/'boundary_attempts.jsonl').exists() else []
result=dict(returncode=code,elapsed_sec=time.monotonic()-started,candidates=snap,source_count=1029,production_changed=changed,external_boundary_attempts=boundary,test_scope=scope,live_ros_contacted=False,motion_commands_sent=False,temperature_queried=False,applied=False,scope='candidate modules executed in existing Python module namespaces before task fixture import; original production files unchanged')
(O/'result.json').write_text(json.dumps(result,indent=2,sort_keys=True)+'\n')
print(json.dumps(result,sort_keys=True))
raise SystemExit(code or bool(changed) or bool(boundary))
