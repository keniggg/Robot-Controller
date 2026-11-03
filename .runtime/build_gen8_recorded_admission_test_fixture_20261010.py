import base64,hashlib,io,json
from pathlib import Path
import rosbag
WORK=Path(__file__).resolve().parents[1];NEW=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z')
full=NEW/'gen8_handoff_analysis/submission_handoff_fixture.json';f=json.loads(full.read_text())
assert hashlib.sha256(full.read_bytes()).hexdigest()=='513eae1df28b72128d563a3a6cc1898e0e087feafdd276eb3c0f94167cc41b5b'
sources={1791602440062703132,1791602440390553236,1791602440792152404,1791602529733577489}
events=[e for e in f['events'] if e['topic'] in ('/grasp_mode/observe_return_session','/grasp_mode/observe_return_measurement_request','/grasp/near_field_phase','/alicia_d/control_reference_epoch') or (e['topic']=='/perception/unknown/detector_status' and e['payload'].get('source_stamp_ns') in sources) or (e['topic'] in ('/perception/object','/perception/object_mask') and e.get('source_ns') in sources)]
def event(topic,m,t):
 b=io.BytesIO();m.serialize(b);raw=b.getvalue();return dict(topic=topic,recorder_received_ns=t.to_nsec(),ros_type=m._type,ros_md5=m._md5sum,source_ns=m.header.stamp.to_nsec() if hasattr(m,'header') else None,wire_sha256=hashlib.sha256(raw).hexdigest(),roswire_base64=base64.b64encode(raw).decode())
with rosbag.Bag(str(Path(f['record'])/'keyframes.bag')) as bag:
 for topic,m,t in bag.read_messages(topics=['/alicia_d/accepted_joint_states','/perception/object','/perception/object_mask','/perception/unknown/detector_status']):
  at=t.to_nsec()
  if topic=='/alicia_d/accepted_joint_states':
   if 1791602438700000000<=at<=1791602441300000000 or 1791602528700000000<=at<=1791602529930000000:events.append(event(topic,m,t))
  elif topic=='/perception/unknown/detector_status':
   d=json.loads(m.data)
   if d.get('source_stamp_ns')==1791602529733577489:
    e=event(topic,m,t);e['payload']=d;events.append(e)
  elif m.header.stamp.to_nsec()==1791602529733577489:events.append(event(topic,m,t))
unique={(e['topic'],e['recorder_received_ns'],e['wire_sha256']):e for e in events};events=sorted(unique.values(),key=lambda e:e['recorder_received_ns'])
r=dict(source_fixture_sha256=hashlib.sha256(full.read_bytes()).hexdigest(),events=events,sources=sorted(sources),context_source='Original prior qualified request/session/active phase seed g1-t4 context; omitted sensor history is not reconstructed.',limitations=['Original source stamps and message bytes remain unchanged.','Callback permutations and two-thread scheduling are explicit simulated orders, not claims about recorded callback ingress.','Final actual false publication coincides with original phase expiry; unexpired false fault isolation is separately synthetic.'])
out=WORK/'.runtime/gen8_station_compute_staging_20261010/src/alicia_grasp_modes/tests/fixtures/gen8_recorded_station_admission.json';out.write_text(json.dumps(r,indent=2,sort_keys=True)+'\n')
print(json.dumps(dict(path=str(out),sha256=hashlib.sha256(out.read_bytes()).hexdigest(),events=len(events),bytes=out.stat().st_size)))
