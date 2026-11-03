"""Original frozen remote callbacks on exact G45 status/ObjectPose/mask/SDK.

Recorded owner/session/phase and the queued computation are explicit initial
state providers. No producer, geometry computation or task motion is simulated
as completed. Exact messages are not restamped or rewritten. Mask pixels remain
only in memory and in their original sealed bag, never in replay output.
"""
from copy import deepcopy
import base64,hashlib,json,socket,sys,traceback
from pathlib import Path
from types import SimpleNamespace as NS
W=Path(__file__).resolve().parents[1];B=W/'.runtime/gen46_baseline_source'
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller');V=E/'ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
I=V/'gen45_station_owner_messages_r1';J=V/'gen46_g45_station_sdk_inputs_r1';G=E/'grasp_records/20261018T070056.706765Z_9d704cdd7a93474ea90deecc79af09ce';O=V/'gen46_g45_exact_callback_probe_r1'
sys.path[:0]=[str(B/'src/alicia_grasp_modes/tests'),str(B/'src/alicia_flexible_grasp_supervisor/tests'),str(B/'src/alicia_grasp_modes/src'),str(B/'src/alicia_flexible_grasp_supervisor/src'),str(W/'devel/lib/python3/dist-packages'),'/opt/ros/noetic/lib/python3/dist-packages']
attempts=[]
class OfflineSocket(socket.socket):
 def __new__(cls,*a,**k):attempts.append('socket');raise RuntimeError('exact callback probe forbids sockets')
socket.socket=OfflineSocket
import rosbag,roslib.message,rospy
from _pytest.monkeypatch import MonkeyPatch
from test_observation_measurement_remote import context
from test_station_compute_wait_remote import station
from test_station_compute_work_handoff import configure
from test_remote_modes import base,module
from test_unknown_pending_identity import deliver

def sha(path):
 h=hashlib.sha256()
 with Path(path).open('rb')as f:
  for b in iter(lambda:f.read(4*1024*1024),b''):h.update(b)
 return h.hexdigest()
def save(path,data):path.write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False)+'\n');assert json.loads(path.read_text())==data

def main():
 assert not O.exists()and Path('/media/zhuyupei/SanStand').is_mount();O.mkdir()
 m=json.loads((G/'manifest.json').read_text());assert m['recorder_closed']and m['state']=='complete';bagrow=next(x for x in m['files']if x['path']=='keyframes.bag');assert sha(G/'keyframes.bag')==bagrow['sha256']
 r=json.loads((I/'result.json').read_text());manifest=json.loads((B/'manifest.json').read_text());original_sources=manifest['source_sha256']
 for rel,want in original_sources.items():assert sha(B/rel)==want
 def rows(topic):
  d=r['files'][topic];assert sha(d['path'])==d['sha256'];return json.loads(Path(d['path']).read_text())['messages']
 def decode(row):
  wire=base64.b64decode(row['wire_base64']);assert hashlib.sha256(wire).hexdigest()==row['wire_sha256']
  cls=roslib.message.get_message_class(row['message_type']);msg=cls();msg.deserialize(wire);assert msg._md5sum==row['md5sum'];return msg
 statuses=[x for x in rows('/perception/unknown/detector_status')if(x['parsed_json']or{}).get('state')=='ready'and isinstance((x['parsed_json']or{}).get('station_compute_wait'),dict)and x['parsed_json']['station_compute_wait']['binding']['selection']['generation']==5]
 assert len(statuses)==33;sources={x['parsed_json']['source_stamp_ns']for x in statuses}
 objects={x['payload']['header']['stamp']['secs']*1000000000+x['payload']['header']['stamp']['nsecs']:(x,decode(x))for x in rows('/perception/object')if x['payload']['header']['stamp']['secs']*1000000000+x['payload']['header']['stamp']['nsecs']in sources}
 masks={}
 with rosbag.Bag(str(G/'keyframes.bag'),'r')as bag:
  for topic,msg,t in bag.read_messages(topics=['/perception/object_mask']):
   source=msg.header.stamp.to_nsec()
   if source in sources:masks[source]=(t.to_nsec(),msg)
 assert len(masks)==len(objects)==33
 sdk=json.loads((J/'accepted_joint_wires.json').read_text());phase=decode(next(x for x in rows('/grasp/near_field_phase')if x['payload']['active']is True));lease=deepcopy(rows('/grasp_mode/observe_return_measurement_request')[-1]['parsed_json']);session=deepcopy(rows('/grasp_mode/observe_return_session')[0]['parsed_json'])
 for k in ('stamp_ns','deadline_ns','origin_snapshot_stamp_ns','view_source_stamp_ns','return_verified_stamp_ns'):session[k]=int(session[k])
 plan=decode(next(x for x in rows('/grasp_6d/plan_enriched')if x['payload']['valid']is True));binding=statuses[0]['parsed_json']['station_compute_wait']['binding']
 events=[]
 for row in sdk:events.append((row['bag_timestamp_ns'],0,'sdk',row['source_stamp_ns'],decode(row)))
 for row in statuses:
  s=row['parsed_json']['source_stamp_ns'];events.extend([(row['received_ns'],1,'status',s,row['parsed_json']),(objects[s][0]['received_ns'],2,'object',s,objects[s][1]),(masks[s][0],3,'mask',s,masks[s][1])])
 # These are original observed clock instants, not claims about the unlogged
 # private state at those instants. The original poll determines its own pin.
 for ns in [1792306915443786382,1792306916088835000,1792306916352817296,1792306920504162788]:events.append((ns,4,'poll',0,None))
 events.sort(key=lambda x:(x[0],x[1]));trace=[];mp=MonkeyPatch();errors=[];n=None
 try:
  def forbidden(*a,**k):attempts.append('ROS_boundary');raise RuntimeError('offline callback probe boundary')
  for name in ('init_node','Publisher','Subscriber','Service','ServiceProxy','wait_for_service','set_param','delete_param'):mp.setattr(rospy,name,forbidden)
  c=context.__wrapped__(mp);c=station.__wrapped__(c,mp);n=c.node;configure(c)
  now=[1792306910000000000];at=lambda ns:rospy.Time(*divmod(ns,10**9));mp.setattr(rospy.Time,'now',staticmethod(lambda:at(now[0])))
  n._ros_source_clock_ns=lambda:now[0]
  import alicia_grasp_modes.station_compute_wait_remote as impl
  mp.setattr(impl.time,'monotonic',lambda:now[0]*1e-9)
  n._reset_station_compute_wait();n._initialize_unknown_pending_identity()
  n._mode_selection=deepcopy(binding['selection']);n._mode_ready=True;c.params[module.SELECTION_PARAM]=deepcopy(binding['selection'])
  n._stream_generation=7;n._target_identity_generation=1;n.target_instance_epoch=4
  n._observe_return_session=session;n._observe_return_prefix=deepcopy(plan);n._observe_return_origin_prefix=deepcopy(plan)
  n._observe_return_phase='observing_contact';n._observe_return_paused=False;n._observe_return_diagnostic_observation_only=False
  n._measurement_remote_request=lease;n._measurement_remote_epoch_ns=binding['control_epoch_ns'];n._measurement_remote_reference={'binding':binding['reference_binding']};n._measurement_remote_admission={}
  n._station_compute_phase=deepcopy(phase);n._station_compute_task_state='PLAN_PREGRASP';n.robot_execution_active=True;n.near_field_planning_active=True
  n._near_field_phase_started_sec=phase.header.stamp.to_sec();n._near_field_phase_deadline_sec=phase.deadline.to_sec()
  n.latest_object=n.latest_object_time=None;n.latest_joint_state=None;n.frames=base.SynchronizedRgbdBuffer(source_clock_ns=n._ros_source_clock_ns)
  n._station_compute_work={};n._station_compute_work_published=False;n._station_compute_publication_started=False
  def state(tag,**extra):
   ref=n._station_compute_reference;trace.append(dict(tag=tag,now_ns=now[0],active_ref=None if ref is None else ref['contract']['source_ns'],retired=n._station_compute_retired,pending=n._station_compute_pending,private_sources={str(k):v.get('kind')for k,v in n._station_private_pending_sources.items()},work_ids=list(n._station_compute_work),caches={k:sorted(getattr(n,k))for k in ('_station_compute_objects','_station_compute_masks','_station_compute_status')},**extra))
  retire=n._station_compute_retire
  def traced_retire(reason):state('retire_entry',reason=str(reason),stack=traceback.format_stack());return retire(reason)
  mp.setattr(n,'_station_compute_retire',traced_retire)
  reconcile=n._station_private_pending_reconcile
  def traced_reconcile():
   state('reconcile_entry')
   try:out=reconcile();state('reconcile_return',result=out);return out
   except BaseException:state('reconcile_exception',stack=traceback.format_exc());raise
  mp.setattr(n,'_station_private_pending_reconcile',traced_reconcile)
  private=n._station_private_pending_input
  def traced_private(kind,ns,value):
   state('private_input_entry',input_kind=kind,input_source=ns)
   out=private(kind,ns,value);state('private_input_return',input_kind=kind,input_source=ns,result=out);return out
  mp.setattr(n,'_station_private_pending_input',traced_private)
  for stamp,_,kind,source,value in events:
   if stamp<now[0]:continue
   if stamp>1792306920600000000:break
   now[0]=stamp;c.clock[0]=stamp*1e-9;state('event_entry',input_kind=kind,input_source=source)
   try:
    if kind=='sdk':n.latest_joint_state=deepcopy(value);n._station_compute_accepted_cb(deepcopy(value))
    elif kind=='poll':n._station_compute_poll()
    elif kind=='status':deliver(n,{'status':value},('status',))
    else:getattr(n,kind+'_cb')(deepcopy(value))
   except BaseException:state('event_exception',input_kind=kind,input_source=source,traceback=traceback.format_exc());raise
   state('event_return',input_kind=kind,input_source=source)
   if n._station_compute_retired:break
   if not n._station_compute_work and n._station_compute_reference is not None and n._station_compute_reference['contract']['source_ns']>=1792306911830121517:
    # Explicit outstanding-computation provider: no original image, prediction
    # result or publication is invented. Only original work-live shape is set.
    snapshot=NS(stamp_ns=1792306911830121517,sample_stamp_ns=(1792306911033774375,1792306911564159393,1792306911830121517))
    ticket=NS(request_id=15,generation=7,target_epoch=4,payload=(snapshot,None))
    n._stream_worker_ticket=ticket;n._station_compute_work[15]=dict(generation=7,target_epoch=4,binding=deepcopy(binding),snapshot=snapshot,source_stamps_ns=snapshot.sample_stamp_ns,snapshot_policy=n._station_compute_snapshot_policy())
    state('explicit_unpublished_work_provider_installed')
  state('replay_finished')
 except BaseException:errors.append(traceback.format_exc())
 finally:mp.undo()
 save(O/'trace.json',trace)
 loaded={}
 for name,mod in list(sys.modules.items()):
  p=getattr(mod,'__file__',None)
  if p and str(B)in str(p)and Path(p).is_file():loaded[name]=dict(path=str(Path(p).resolve()),sha256=sha(p))
 result=dict(completed=not errors,errors=errors,retired=None if n is None else n._station_compute_retired,final_ref=None if n is None or n._station_compute_reference is None else n._station_compute_reference['contract']['source_ns'],baseline_manifest_sha256=sha(B/'manifest.json'),bag_sha256=bagrow['sha256'],extraction_sha256=sha(I/'result.json'),sdk_extraction_sha256=sha(J/'result.json'),trace_sha256=sha(O/'trace.json'),module_provenance=loaded,external_boundary_attempts=attempts,original_messages_unchanged=True,initialization_provider='Recorded session/qualified lease/phase/selection/target identity prebound into original real-remote harness. Its ROS configuration provider comes from the existing offline fixture; no Task or geometry worker ran.',outstanding_work_provider='Explicit request15 metadata shape only, no computation result or publication authority.',mask_pixels_exported=False,temperature_read=False,hardware_contacted=False,ROS_initialized=False,production_or_tests_or_record_modified=False,helper_sha256=sha(__file__))
 assert not attempts and sha(G/'keyframes.bag')==bagrow['sha256'];save(O/'result.json',result);(O/'helper_source.py').write_bytes(Path(__file__).read_bytes());print(json.dumps(dict(result=str(O/'result.json'),sha256=sha(O/'result.json'),completed=result['completed'],errors=errors,retired=result['retired'],final_ref=result['final_ref']),ensure_ascii=False))
if __name__=='__main__':main()
