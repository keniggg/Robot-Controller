from pathlib import Path
import sys, os, json, hashlib, time
import xml.etree.ElementTree as ET
w=Path(__file__).resolve().parents[1];stage=w/'.runtime/gen12_phase_entry_frame_owner_20261012'
out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=False)
sys.dont_write_bytecode=True
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:1', ROS_LOG_DIR=str(out/'ros'),PYTHONDONTWRITEBYTECODE='1')
sys.path[:0]=[str(stage/p) for p in ('src/alicia_grasp_modes/src','src/alicia_flexible_grasp_supervisor/src','src/alicia_grasp_modes/tests','src/alicia_flexible_grasp_supervisor/tests')]
os.environ['ROS_PACKAGE_PATH']=str(stage/'src')+':'+os.environ.get('ROS_PACKAGE_PATH','')
files=['test_qualified_station_phase.py','test_qualified_phase_caller_gates.py','test_arrival_phase_frame_owner.py','test_observation_arrival_station.py','test_station_compute_wait_producer.py','test_observe_return_measurement.py','test_unknown_stream.py']
paths=[stage/'src/alicia_grasp_modes/tests'/p for p in files]
prod=['src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_task.py','src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_producer.py','src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py','src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py']
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
old={p:sha(stage/p) for p in prod};actual={p:sha(w/p) for p in prod}
test_sha={str(p.relative_to(stage)):sha(p) for p in paths}
import pytest
with (out/'pytest.log').open('w') as f:
 stdout,stderr=sys.stdout,sys.stderr
 try:
  sys.stdout=sys.stderr=f
  rc=pytest.main(['-q','-p','no:cacheprovider','--basetemp='+str(out/'tmp'),'--junitxml='+str(out/'pytest.xml'),*map(str,paths)])
 finally:sys.stdout,sys.stderr=stdout,stderr
suites=list(ET.parse(out/'pytest.xml').getroot().iter('testsuite'));counts={k:sum(int(s.attrib.get(k,0)) for s in suites) for k in ('tests','errors','failures','skipped')}
result=dict(passed=rc==0,counts=counts,source_sha256=old,source_unchanged=old=={p:sha(stage/p) for p in prod},actual_unchanged=actual=={p:sha(w/p) for p in prod},tests=files,test_sha256=test_sha,tests_unchanged=test_sha=={str(p.relative_to(stage)):sha(p) for p in paths},pytest_xml_sha256=sha(out/'pytest.xml'),no_ros_actions=True)
(out/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result));sys.exit(rc)
