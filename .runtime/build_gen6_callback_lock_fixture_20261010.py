"""Extract original received ROS bytes and identity evidence; never replay to ROS."""
import base64,hashlib,io,json,time
from pathlib import Path
import rosbag

EXT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
REC=EXT/'grasp_records/20261010T024239.212466Z_bd212804e3d2427d8b64e164bf10cec0'
NEW=EXT/'ros_log/full_latest_restart_20261010T002347Z'
OUT=NEW/'gen6_handoff_analysis/arrival_lock_fixture'
OLD=1791600172893596172
STOP=1791600212981000000
assert Path('/media/zhuyupei/SanStand').is_mount()
assert json.loads((REC/'manifest.json').read_text())['recorder_closed']
OUT.mkdir(exist_ok=False)
def ns(m):return m.header.stamp.to_nsec()
def record(topic,m,t):
 s=io.BytesIO();m.serialize(s);raw=s.getvalue()
 result=dict(topic=topic,recorder_received_ns=t.to_nsec(),ros_type=m._type,ros_md5=m._md5sum,
  wire_sha256=hashlib.sha256(raw).hexdigest(),roswire_base64=base64.b64encode(raw).decode())
 if hasattr(m,'header'):result['source_ns']=ns(m)
 if m._type=='std_msgs/String':result['payload']=json.loads(m.data)
 return result
all_events=[];objects={};statuses={};masks={};geometries=[];plans=[];protocol=[];q=[]
topics=['/perception/object','/perception/object_mask','/perception/unknown/detector_status',
 '/grasp_6d/object_geometry','/grasp_6d/plan_enriched','/grasp_6d/preview_plan_enriched',
 '/grasp_mode/initial_preflight_request','/grasp_mode/initial_preflight_ack','/grasp_mode/initial_preflight_attempt',
 '/grasp_mode/observe_return_measurement_request','/grasp_mode/observe_return_measurement_ack',
 '/grasp_mode/observe_return_session','/grasp_mode/observe_return_ack','/grasp/state','/alicia_d/control_reference_epoch']
with rosbag.Bag(str(REC/'keyframes.bag')) as b:
 for topic,m,t in b.read_messages(topics=topics):
  if t.to_nsec()>STOP:continue
  event=(topic,m,t)
  if topic=='/perception/object':objects[ns(m)]=event
  elif topic=='/perception/object_mask':masks[ns(m)]=event
  elif topic=='/perception/unknown/detector_status':statuses[json.loads(m.data).get('source_stamp_ns')]=event
  elif topic=='/grasp_6d/object_geometry':geometries.append(event)
  elif topic in ('/grasp_6d/plan_enriched','/grasp_6d/preview_plan_enriched'):plans.append(event)
  else:protocol.append(event)
arm=next(json.loads(m.data) for topic,m,t in protocol if topic=='/grasp_mode/initial_preflight_request' and json.loads(m.data)['action']=='arm')
ack=next(json.loads(m.data) for topic,m,t in protocol if topic=='/grasp_mode/initial_preflight_ack' and json.loads(m.data)['action']=='arm')
commit=next(json.loads(m.data) for topic,m,t in protocol if topic=='/grasp_mode/initial_preflight_ack' and json.loads(m.data)['action']=='commit')
attempt=next(json.loads(m.data) for topic,m,t in protocol if topic=='/grasp_mode/initial_preflight_attempt')
qualified=next(json.loads(m.data) for topic,m,t in protocol if topic=='/grasp_mode/observe_return_measurement_request' and json.loads(m.data)['state']=='qualified')
binding=json.loads(arm['anchor_binding'])
old_status=json.loads(statuses[OLD][1].data);obj=objects[OLD][1]
selected_sources={binding['anchor_source_stamp_ns'],OLD,ack['reference_source_ns'],commit['reference_source_ns'],1791600162692783832,*qualified['source_stamps_ns']}
selected=list(protocol)+list(plans)+list(geometries)
for n in sorted(selected_sources):
 for series in (objects,masks,statuses):
  if n in series:selected.append(series[n])
selected.sort(key=lambda x:x[2].to_nsec())
records=[record(*v) for v in selected]
checks=dict(status_ready=old_status['state']=='ready',status_tracked=old_status['detail']=='tracked',
 target_locked=old_status['target_locked'] is True,target_not_lost=old_status['target_lost'] is False,
 status_exact_object_source=old_status['source_stamp_ns']==ns(obj),object_mask_exact_source=OLD in masks,
 mode=old_status['mode']==arm['selection']['mode'],generation=old_status['generation']==arm['selection']['generation'],
 selection_stamp=old_status['selection_stamp_ns']==arm['selection']['stamp_ns'],
 anchor_binding_fields={k:old_status.get(k)==v for k,v in binding.items()},
 initial_identity_template=old_status.get('initial_station_identity_template')==ack['reference_status'].get('initial_station_identity_template'),
 qualified_anchor_same=all(json.loads(statuses[n][1].data).get('anchor_source_stamp_ns')==binding['anchor_source_stamp_ns'] for n in qualified['source_stamps_ns'] if n in statuses))
assert all(v for k,v in checks.items() if k!='anchor_binding_fields') and all(checks['anchor_binding_fields'].values()),checks
frozen=next(m for topic,m,t in plans if m.valid and m.plan_id==arm['plan_id'])
assert frozen.target_track_id==arm['target_track_id']=='g1-t4'
remote_same=[record(*e) for e in geometries if ns(e[1])==OLD]
remote_prior=[e for e in geometries if ns(e[1])<=OLD and e[1].valid]
last_remote=max(remote_prior,key=lambda e:e[2].to_nsec())
assert last_remote[1].target_track_id==frozen.target_track_id
objvalues=dict(header=dict(stamp_ns=ns(obj),frame_id=obj.header.frame_id),detected=obj.detected,label=obj.label,
 confidence=obj.confidence,u=obj.u,v=obj.v,bbox=dict(x=obj.bbox_x,y=obj.bbox_y,width=obj.bbox_width,height=obj.bbox_height),depth_m=obj.depth_m,
 pose_base=dict(stamp_ns=obj.pose_base.header.stamp.to_nsec(),frame_id=obj.pose_base.header.frame_id,
  position=[getattr(obj.pose_base.pose.position,k) for k in 'xyz'],orientation=[getattr(obj.pose_base.pose.orientation,k) for k in 'xyzw']),
 pose_camera=dict(stamp_ns=obj.pose_camera.header.stamp.to_nsec(),frame_id=obj.pose_camera.header.frame_id,
  position=[getattr(obj.pose_camera.pose.position,k) for k in 'xyz'],orientation=[getattr(obj.pose_camera.pose.orientation,k) for k in 'xyzw']))
lines=(NEW/'contact_yaw_consumers_deployment/consumers.log').read_text().splitlines()
relevant=[line for line in lines if ('Grasp ignored stale object source age 38.377s' in line or '[1791600212.' in line and ('FAILED' in line or 'CAD envelope' in line))]
report=dict(schema=1,record=str(REC),bag=str(REC/'keyframes.bag'),created_ns=time.time_ns(),
 old_positive_source_ns=OLD,old_positive_received_ns=objects[OLD][2].to_nsec(),
 old_positive_age_at_recorder_sec=(objects[OLD][2].to_nsec()-OLD)/1e9,
 old_positive_object=objvalues,old_positive_status=old_status,
 association_checks=checks,token_arm=arm,token_arm_ack=ack,token_commit_ack=commit,
 frozen_plan=dict(plan_id=frozen.plan_id,target_track_id=frozen.target_track_id,source_ns=ns(frozen),diagnostic=frozen.diagnostic),
 same_source_remote_geometry=remote_same,
 last_valid_remote_geometry=dict(source_ns=ns(last_remote[1]),recorder_received_ns=last_remote[2].to_nsec(),target_track_id=last_remote[1].target_track_id),
 attempt=attempt,qualified=qualified,consumer_log_lines=relevant,
 events=records,source_stamp_set=sorted(selected_sources),
 limitations=[
  'Recorder receipt is not the task subscriber callback-entry time. The bag does not instrument acquisition/release of task locks.',
  'ObjectPose itself has no target_track_id. The exact-source READY status binds its original immutable producer anchor to the arm token; the token binds plan g1-t4. This does not manufacture a per-object remote track tag.',
  'No separate unknown-geometry topic was recorded; the exact-source detector status contains current geometry_metrics and initial_station_measurement. There was no /grasp_6d/object_geometry at the old-positive stamp; its preceding valid geometry belongs to frozen plan source.',
  'Messages retain original serialized bytes and original recorder timestamps; no source clock, qualification or physical command is synthesized.'
 ])
path=OUT/'fixture.json';path.write_text(json.dumps(report,indent=2,sort_keys=True,allow_nan=False)+'\n')
(out:=OUT/'manifest.json').write_text(json.dumps(dict(fixture_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
 events=len(records),source_record_manifest_sha256=hashlib.sha256((REC/'manifest.json').read_bytes()).hexdigest(),script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()),indent=2)+'\n')
print(json.dumps(dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),events=len(records),
 all_association_checks=True,same_source_remote_geometry_count=len(remote_same),old_received_ns=report['old_positive_received_ns'],old_age=report['old_positive_age_at_recorder_sec'])))
