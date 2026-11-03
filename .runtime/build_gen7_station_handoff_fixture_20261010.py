"""Freeze exact recorded messages at second-view admission; offline only."""
import base64,hashlib,io,json,time
from pathlib import Path
import rosbag
EXT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
REC=EXT/'grasp_records/20261010T030141.809371Z_7724ffb5910c4565bf751bd9c281954c'
NEW=EXT/'ros_log/full_latest_restart_20261010T002347Z'
OUT=NEW/'gen7_handoff_analysis/station_handoff_fixture'
assert Path('/media/zhuyupei/SanStand').is_mount()
assert json.loads((REC/'manifest.json').read_text())['recorder_closed']
OUT.mkdir(exist_ok=False)
observed=json.loads((NEW/'gen7_handoff_analysis/first_loss_observer_evidence.json').read_text())
sources={r['source_stamp_ns'] for r in observed['previous_ready_statuses']}
sources.update([1791601303774594068,1791601358449840307,1791601361780489206,1791601363778279781,1791601379317423820])
STOP=1791601379600000000
proto=['/grasp_mode/selection','/grasp_mode/initial_preflight_request','/grasp_mode/initial_preflight_ack','/grasp_mode/initial_preflight_attempt','/grasp_mode/observe_return_measurement_request','/grasp_mode/observe_return_measurement_ack','/grasp_mode/observe_return_session','/grasp_mode/observe_return_ack','/grasp/state','/alicia_d/control_reference_epoch']
topics=proto+['/perception/object','/perception/object_mask','/perception/unknown/detector_status','/grasp_6d/object_geometry','/grasp_6d/plan_enriched','/grasp_6d/preview_plan_enriched','/alicia_d/accepted_joint_states','/alicia_d/sdk_joint_states']
events=[];status={};objects={};names={};sessions=[];qualified=[];lastgeometry=None
with rosbag.Bag(str(REC/'keyframes.bag')) as b:
 for topic,m,t in b.read_messages(topics=topics):
  at=t.to_nsec()
  if at>STOP:continue
  source=m.header.stamp.to_nsec() if hasattr(m,'header') else None
  payload=None
  if m._type=='std_msgs/String':
   try:payload=json.loads(m.data)
   except ValueError:payload=m.data
  if topic=='/perception/unknown/detector_status':
   source=payload.get('source_stamp_ns')
   if source not in sources:continue
   status[source]=payload
  elif topic in ('/perception/object','/perception/object_mask'):
   if source not in sources:continue
   if topic=='/perception/object':objects[source]=dict(source_ns=source,received_ns=at,detected=m.detected,label=m.label,confidence=m.confidence,position_base=[getattr(m.pose_base.pose.position,k) for k in 'xyz'])
  elif topic in ('/alicia_d/accepted_joint_states','/alicia_d/sdk_joint_states'):
   if at<1791601378500000000:continue
  elif topic not in proto and topic!='/grasp_6d/object_geometry' and not m.valid:continue
  rawbuf=io.BytesIO();m.serialize(rawbuf);raw=rawbuf.getvalue()
  reread=type(m)();reread.deserialize(raw);check=io.BytesIO();reread.serialize(check)
  assert check.getvalue()==raw
  row=dict(topic=topic,recorder_received_ns=at,source_ns=source,ros_type=m._type,ros_md5=m._md5sum,wire_sha256=hashlib.sha256(raw).hexdigest(),roswire_base64=base64.b64encode(raw).decode())
  if payload is not None:row['payload']=payload
  if topic=='/grasp_mode/observe_return_session':sessions.append(payload)
  if topic=='/grasp_mode/observe_return_measurement_request' and payload.get('state')=='qualified':qualified.append(payload)
  if topic=='/grasp_6d/object_geometry' and m.valid:lastgeometry=dict(source_ns=source,received_ns=at,target_track_id=m.target_track_id)
  events.append(row);names[topic]=names.get(topic,0)+1
assert 1791601379054992675 in objects and objects[1791601379054992675]['detected']
assert status[1791601379054992675]['state']=='ready'
assert status[1791601379317423820]['frame_timing']['arrival_station_invalidated_reason']=='station_compute_wait_station_changed'
report=dict(schema=1,generation=7,record=str(REC),source_stamp_set=sorted(sources),last_positive=objects[1791601379054992675],last_positive_status=status[1791601379054992675],first_loss_status=status[1791601379317423820],sessions=sessions,qualified_requests=qualified,last_valid_remote_geometry=lastgeometry,selected_objects=objects,counts=names,events=events,wire_roundtrip_verified=len(events),limitations=['Exact ROS payloads and recorder stamps; recorder time is not subscriber callback ingress time.','Only selected RGBD identity sources retained; this fixture is not a complete RGB/depth sensor replay.','No ROS publisher, service or robot command was used.'])
p=OUT/'fixture.json';p.write_text(json.dumps(report,indent=2,sort_keys=True)+'\n')
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
(OUT/'manifest.json').write_text(json.dumps(dict(events=len(events),fixture_sha256=sha(p),source_record_manifest_sha256=sha(REC/'manifest.json'),builder_sha256=sha(Path(__file__))),indent=2)+'\n')
print(json.dumps(dict(path=str(p),sha256=sha(p),events=len(events),last_positive=report['last_positive'],last_geometry=lastgeometry,topics=names)))
