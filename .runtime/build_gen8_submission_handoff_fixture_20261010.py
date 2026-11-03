"""Freeze original GEN8 post-arrival three-source joins and pending timeline."""
import base64,hashlib,io,json
from pathlib import Path
import numpy as np
import rosbag
EXT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
REC=EXT/'grasp_records/20261010T031939.390394Z_5136b6b5d69d49a3bd1ba57514758965'
NEW=EXT/'ros_log/full_latest_restart_20261010T002347Z';OUT=NEW/'gen8_handoff_analysis'
assert Path('/media/zhuyupei/SanStand').is_mount()
assert json.loads((REC/'manifest.json').read_text())['recorder_closed']
BEGIN=1791602438900000000;END=1791602529930000000
FAIL=1791602529913476467
objects={};masks={};statuses={};geometry=[];protocol=[];phases=[];events=[];states=[]
def wire(topic,m,t):
 b=io.BytesIO();m.serialize(b);raw=b.getvalue();back=type(m)();back.deserialize(raw);b2=io.BytesIO();back.serialize(b2);assert b2.getvalue()==raw
 d=dict(topic=topic,recorder_received_ns=t.to_nsec(),ros_type=m._type,ros_md5=m._md5sum,wire_sha256=hashlib.sha256(raw).hexdigest(),roswire_base64=base64.b64encode(raw).decode())
 if hasattr(m,'header'):d['source_ns']=m.header.stamp.to_nsec()
 if m._type=='std_msgs/String':d['payload']=json.loads(m.data)
 return d
topics=['/perception/object','/perception/object_mask','/perception/unknown/detector_status','/grasp_6d/object_geometry','/grasp_6d/plan_enriched','/grasp_6d/preview_plan_enriched','/grasp_mode/observe_return_measurement_request','/grasp_mode/observe_return_measurement_ack','/grasp_mode/observe_return_session','/grasp/near_field_phase','/grasp/state','/alicia_d/control_reference_epoch']
with rosbag.Bag(str(REC/'keyframes.bag')) as bag:
 for topic,m,t in bag.read_messages(topics=topics):
  at=t.to_nsec()
  if at>END:continue
  event=(topic,m,t)
  if topic=='/perception/unknown/detector_status':
   d=json.loads(m.data);s=d.get('source_stamp_ns')
   if s and at>=BEGIN:statuses[s]=(event,d)
  elif topic=='/perception/object':
   if at>=BEGIN:objects[m.header.stamp.to_nsec()]=event
  elif topic=='/perception/object_mask':
   if at>=BEGIN:masks[m.header.stamp.to_nsec()]=event
  elif topic=='/grasp_6d/object_geometry':
   if at>=BEGIN:geometry.append(event)
  else:
   protocol.append(event)
   if topic=='/grasp/near_field_phase':phases.append(event)
   if topic=='/grasp/state':states.append(event)
ready={s:v for s,v in statuses.items() if v[1].get('state')=='ready' and v[0][2].to_nsec()<FAIL}
join=[]
for s,(event,d) in sorted(ready.items()):
 times={k:v[s][2].to_nsec() for k,v in [('object',objects),('mask',masks)] if s in v};times['status']=event[2].to_nsec()
 join.append(dict(source_ns=s,received_ns=times,complete=set(times)=={'object','mask','status'},join_age_sec=(max(times.values())-s)/1e9,detected=objects[s][1].detected if s in objects else None,state=d['state'],detail=d['detail'],station_contract_source_ns=(d.get('station_compute_wait') or {}).get('source_ns')))
firstpending=next(e for e in geometry if e[1].failure_reason.startswith('TARGET_PROCESSING_PENDING'))
pendings=[]
for e in geometry:
 if e[1].failure_reason.startswith('TARGET_PROCESSING_PENDING'):
  d=json.loads(e[1].failure_reason.split(': ',1)[1]);c=d['station_compute_wait']
  pendings.append(dict(received_ns=e[2].to_nsec(),message_source_ns=e[1].header.stamp.to_nsec(),target_track_id=e[1].target_track_id,contract_source_ns=c['source_ns'],geometry_deadline_ns=c['geometry_deadline_ns'],identity_deadline_ns=c['identity_deadline_ns'],phase_deadline_ns=c['phase_deadline_ns']))
sources=set(sorted(ready)[:12]+sorted(ready)[-3:])
sources.update(p['contract_source_ns'] for p in pendings[:4])
chosen=list(protocol)+geometry[:5]+geometry[-3:]
for s in sorted(sources):
 for v in (objects,masks):
  if s in v:chosen.append(v[s])
 if s in statuses:chosen.append(statuses[s][0])
chosen.sort(key=lambda e:e[2].to_nsec());exact=[wire(*e) for e in chosen]
observer_files=[OUT/'station_compute_8_1791602335463383495.jsonl',OUT/'motion_boundary_8_1791602332953622902.jsonl']
rows=[json.loads(l) for p in observer_files for l in p.open()]
q=[r for r in rows if r['kind']=='accepted_q' and 1791602439311033248<=r['received_ros_ns']<FAIL]
qspan=np.ptp(np.array([r['q'] for r in q]),axis=0).tolist()
first_taskfail=next(r for r in rows if r['kind']=='task' and r['state']=='FAILED')
first_detectorloss=next(r for r in rows if r['kind']=='detector' and r.get('target_lost'))
joinages=[r['join_age_sec'] for r in join if r['complete']]
report=dict(generation=8,record=str(REC),all_ready_three_source_joins=join,join_summary=dict(ready=len(join),complete=sum(r['complete'] for r in join),detected=sum(r['detected'] is True for r in join),age_min=float(min(joinages)),age_max=float(max(joinages)),age_p90=float(np.quantile(joinages,.9))),
 pending_geometry_timeline=pendings,events=exact,exact_event_count=len(exact),first_task_failure=first_taskfail,subsequent_detector_loss=first_detectorloss,
 first_pending_geometry=wire(*firstpending),first_ready_statuses=[ready[s][1] for s in sorted(ready)[:4]],
 qualified_station_feedback=dict(samples=len(q),axis_range_rad=qspan,first=q[0],last=q[-1]),
 limitations=['Recorded source joins demonstrate what was published, not the precise remote callback entry/lock schedule.',
 'No second observing session, second executing request, CONTACT plan, or successful grasp was recorded.',
 'The producer station loss follows the first task failure; neither the failure nor source freshness may be attributed by message label alone.'])
p=OUT/'submission_handoff_fixture.json';p.write_text(json.dumps(report,indent=2,sort_keys=True)+'\n')
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
seal=dict(observers_closed=True,processes=[84919,84505],exit_codes=[0,0],files={str(p):sha(p) for p in observer_files},report=str(p),report_sha256=sha(p),no_hardware_commands=True)
(OUT/'observers_closed_seal.json').write_text(json.dumps(seal,indent=2,sort_keys=True)+'\n')
print(json.dumps(dict(path=str(p),sha256=sha(p),joins=report['join_summary'],events=len(exact),pending_geometry=len(pendings),station_q_samples=len(q),station_q_range=qspan,first_pending=pendings[0],first_task_failure_ns=first_taskfail['received_ros_ns'],detector_loss_ns=first_detectorloss['received_ros_ns'])))
