from pathlib import Path
import sys, os, json, hashlib, time
import xml.etree.ElementTree as ET
w=Path(__file__).resolve().parents[1];stage=w/'.runtime/gen13_station_fragment_identity_20261012'
out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=False)
sys.dont_write_bytecode=True
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:1', ROS_LOG_DIR=str(out/'ros'),PYTHONDONTWRITEBYTECODE='1')
sys.path[:0]=[str(stage/p) for p in ('src/alicia_grasp_modes/src','src/alicia_flexible_grasp_supervisor/src','src/alicia_grasp_modes/tests','src/alicia_flexible_grasp_supervisor/tests')]
os.environ['ROS_PACKAGE_PATH']=str(stage/'src')+':'+os.environ.get('ROS_PACKAGE_PATH','')
files=['test_qualified_station_appearance.py','test_observation_appearance_integration.py','test_observation_measurement_unknown.py','test_observation_arrival_station.py','test_arrival_fragment_group.py','test_arrival_fragment_support_guards.py','test_arrival_fragment_recovery_guards.py','test_arrival_matching_fragments.py','test_arrival_matching_fragment_tracking.py','test_station_compute_wait_producer.py','test_station_compute_wait_remote.py','test_station_compute_wait_task.py','test_station_compute_wait_integration.py','test_metric_ambiguity_hold.py','test_arrival_marker_revocation_race.py','test_observation_arrival_frame_completion.py','test_arrival_normal_completion_consumer.py']
files += ['test_qualified_station_identity_independent.py','test_station_fragment_typed_propagation.py','test_station_source_deadline_boundaries.py','test_arrival_fragment_control_flow.py','test_rgb_feedback_wait.py']
if len(sys.argv)>2: files=sys.argv[2:]
paths=[stage/'src/alicia_grasp_modes/tests'/p for p in files]
prod=['src/alicia_grasp_modes/src/alicia_grasp_modes/arrival_fragment_group.py','src/alicia_grasp_modes/src/alicia_grasp_modes/observation_appearance.py','src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_producer.py','src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py']
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
