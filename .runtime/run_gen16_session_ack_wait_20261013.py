#!/usr/bin/env python3
"""External-only offline ACK evidence; refuses changed actual baseline."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET
W=Path(__file__).resolve().parents[1]
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
C=W/'.runtime/gen16_session_ack_wait_20261013'
OUT=N/'verification/gen16_session_ack_wait_r1'
SCOPE=['src/alicia_grasp_modes/tests/'+f for f in (
 'test_session_ack_contract_wait.py','test_observe_return_task.py',
 'test_observe_return_supplement_task.py','test_qualified_station_phase.py',
 'test_observe_return_measurement.py','test_station_compute_wait_task.py',
 'test_gen10_handoff_authority_independent.py',
 'test_qualified_phase_timestamp_precision.py','test_qualified_phase_epoch_independent.py')]
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def files(root):return {str(p.relative_to(root)):sha(p) for p in root.rglob('*.py') if '__pycache__' not in p.parts}
def run(name,root,scope,baseline=False):
 out=OUT/name;out.mkdir()
 env=dict(os.environ,PYTHONNOUSERSITE='1',ROS_MASTER_URI='http://127.0.0.1:1',
    GEN16_ACK_EXPECT_BASELINE=str(int(baseline)),GEN16_ACK_PROBE_OUTPUT=str(out))
 env['ROS_PACKAGE_PATH']=str(root/'src')+':'+env.get('ROS_PACKAGE_PATH','')
 env['PYTHONPATH']=':'.join(str(root/r) for r in ('src/alicia_grasp_modes/src',
  'src/alicia_flexible_grasp_supervisor/src','src/alicia_grasp_modes/tests',
  'src/alicia_flexible_grasp_supervisor/tests'))+':'+env.get('PYTHONPATH','')
 command=['python3','-m','pytest','-q',*[str(p) for p in scope],'--disable-warnings','--tb=short',
  '--junitxml='+str(out/'pytest.xml'),'--basetemp='+str(out/'tmp')]
 with (out/'pytest.log').open('w') as log:p=subprocess.run(command,cwd=W,env=env,stdout=log,stderr=subprocess.STDOUT)
 x=ET.parse(out/'pytest.xml').getroot()
 counts={k:sum(int(s.attrib.get(k,0)) for s in x.iter('testsuite')) for k in ('tests','errors','failures','skipped')}
 result=dict(name=name,returncode=p.returncode,summary=counts,command=command,
  log_sha256=sha(out/'pytest.log'),xml_sha256=sha(out/'pytest.xml'))
 (out/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(name,counts,flush=True)
 return result
assert Path('/media/zhuyupei/SanStand').is_mount()
OUT.mkdir()
release=N/'verification/gen15_phase_timestamp_release_manifest.json'
sealed=json.loads(release.read_text())['source_sha256']
assert all(sha(W/r)==v for r,v in sealed.items())
before=files(C)
proof=W/'.runtime/test_gen16_session_ack_r4_wire_20261013.py'
baseline=run('original_r4_boundary',W,[proof],True)
candidate=run('candidate_r4_boundary',C,[proof])
focused=run('candidate_focused',C,[C/r for r in SCOPE])
after=files(C)
result=dict(kind='GEN16_BOUNDED_SESSION_ACK_WAIT_AUTHOR_PROOF',passed=all(v['returncode']==0 for v in (baseline,candidate,focused)) and before==after,
 baseline=baseline,candidate=candidate,focused=focused,source_unchanged=before==after,
 actual_unchanged=all(sha(W/r)==v for r,v in sealed.items()),source_sha256=before,
 installed_source_count=len(sealed),release=str(release),release_sha256=sha(release),
 proof_sha256=sha(proof),runner_sha256=sha(__file__),recorded_ns=time.time_ns(),
 live_ros_contacted=False,motion_commands_sent=False,temperature_queried=False,physical_grasp_verified=False)
result['passed'] &= result['actual_unchanged']
(OUT/'result.json').write_text(json.dumps(result,indent=2)+'\n')
print('RESULT',result['passed'],str(OUT/'result.json'),sha(OUT/'result.json'),flush=True)
assert result['passed']
