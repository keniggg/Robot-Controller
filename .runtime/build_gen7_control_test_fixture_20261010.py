"""Derive a small exact-message test fixture from closed GEN7; no ROS I/O."""
import base64,hashlib,io,json
from pathlib import Path
import rosbag
WORK=Path(__file__).resolve().parents[1]
NEW=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z')
fullpath=NEW/'gen7_handoff_analysis/station_handoff_fixture/fixture.json'
f=json.loads(fullpath.read_text())
assert hashlib.sha256(fullpath.read_bytes()).hexdigest()=='ecf9b805c5e0f2b0c29e7af771b3ae74b98189d974ca363d336f3afce48622ce'
old=f['last_positive_status'];failure=f['first_loss_status'];contract=old['station_compute_wait']
events=[r for r in f['events'] if r['topic'] in ('/grasp/state','/grasp_mode/observe_return_session','/grasp_mode/observe_return_measurement_request','/grasp_mode/observe_return_measurement_ack','/alicia_d/accepted_joint_states','/alicia_d/control_reference_epoch')]
with rosbag.Bag(str(Path(f['record'])/'keyframes.bag')) as b:
 for topic,m,t in b.read_messages(topics=['/grasp/near_field_phase']):
  if t.to_nsec()>1791601379600000000:continue
  buf=io.BytesIO();m.serialize(buf);raw=buf.getvalue()
  events.append(dict(topic=topic,recorder_received_ns=t.to_nsec(),source_ns=m.header.stamp.to_nsec(),ros_type=m._type,ros_md5=m._md5sum,wire_sha256=hashlib.sha256(raw).hexdigest(),roswire_base64=base64.b64encode(raw).decode()))
events.sort(key=lambda r:r['recorder_received_ns'])
payload=dict(schema=1,source_fixture_sha256=hashlib.sha256(fullpath.read_bytes()).hexdigest(),generation=7,contract=contract,
 last_positive=f['last_positive'],first_loss_source_ns=failure['source_stamp_ns'],first_loss_ros_ns=failure['first_loss']['observed_stamp_ns'],
 arrival_frame_completion=failure['frame_timing']['arrival_frame_completion'],arrival_station_invalidated_reason=failure['frame_timing']['arrival_station_invalidated_reason'],events=events,
 limitations=['Exact state/phase/contract/q recorded messages; no second executing request exists in GEN7.',
 'Producer sensor lifecycle is exercised separately with synthetic RGBD; these recorded control payloads do not constitute a full real sensor replay.',
 'Recorder timestamps do not establish exact subscriber ingress or cross-topic callback ordering.'])
p=WORK/'.runtime/gen7_station_handoff_staging_20261010/src/alicia_grasp_modes/tests/fixtures/gen7_station_handoff_control.json'
p.write_text(json.dumps(payload,indent=2,sort_keys=True)+'\n')
print(json.dumps(dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),events=len(events),phase_events=sum(r['topic']=='/grasp/near_field_phase' for r in events))))
