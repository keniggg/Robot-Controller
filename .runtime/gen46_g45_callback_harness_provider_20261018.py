"""Reusable OFFLINE test-harness provider; caller must install boundary guards.

No ROS graph/init, device, publication or bag access occurs here. It loads only
previously verified nonimage wire JSON. Caller provides original mask messages
in memory from its own single sealed-bag scan. Initialization is an explicit
state provider, not a reconstruction of original private memory.
"""
from copy import deepcopy
import base64,hashlib,json,time
from pathlib import Path
from types import SimpleNamespace as NS
import roslib.message
import rospy
from test_observation_measurement_remote import context
from test_station_compute_wait_remote import station
from test_station_compute_work_handoff import configure
from test_remote_modes import base,module
from test_unknown_pending_identity import deliver

W=Path(__file__).resolve().parents[1]
V=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification')
I=V/'gen45_station_owner_messages_r1';J=V/'gen46_g45_station_sdk_inputs_r1'
_REAL_MONOTONIC=time.monotonic
SOURCE3=(1792306911033774375,1792306911564159393,1792306911830121517)

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def decode(row):
 wire=base64.b64decode(row['wire_base64']);assert hashlib.sha256(wire).hexdigest()==row['wire_sha256']
 cls=roslib.message.get_message_class(row['message_type']);message=cls();message.deserialize(wire);assert message._md5sum==row['md5sum'];return message

def load_nonimage_inputs():
 r=json.loads((I/'result.json').read_text());assert r['passed']
 def rows(topic):
  info=r['files'][topic];assert sha(info['path'])==info['sha256'];return json.loads(Path(info['path']).read_text())['messages']
 statuses=[x for x in rows('/perception/unknown/detector_status')if(x['parsed_json']or{}).get('state')=='ready'and isinstance((x['parsed_json']or{}).get('station_compute_wait'),dict)and x['parsed_json']['station_compute_wait']['binding']['selection']['generation']==5]
 sources={x['parsed_json']['source_stamp_ns']for x in statuses};assert len(statuses)==33
 objects={x['payload']['header']['stamp']['secs']*1000000000+x['payload']['header']['stamp']['nsecs']:(x,decode(x))for x in rows('/perception/object')if x['payload']['header']['stamp']['secs']*1000000000+x['payload']['header']['stamp']['nsecs']in sources}
 sdk_rows=json.loads((J/'accepted_joint_wires.json').read_text());sdk=[(x,decode(x))for x in sdk_rows]
 phase=decode(next(x for x in rows('/grasp/near_field_phase')if x['payload']['active']is True));lease=deepcopy(rows('/grasp_mode/observe_return_measurement_request')[-1]['parsed_json']);session=deepcopy(rows('/grasp_mode/observe_return_session')[0]['parsed_json'])
 for k in ('stamp_ns','deadline_ns','origin_snapshot_stamp_ns','view_source_stamp_ns','return_verified_stamp_ns'):session[k]=int(session[k])
 plan=decode(next(x for x in rows('/grasp_6d/plan_enriched')if x['payload']['valid']is True));binding=deepcopy(statuses[0]['parsed_json']['station_compute_wait']['binding'])
 return NS(statuses=statuses,objects=objects,sdk=sdk,phase=phase,lease=lease,session=session,plan=plan,binding=binding,source3=SOURCE3,
  limits='Only original nonimage messages. No original private fusion/CAS state or actual MoveIt curve is claimed.')

def initialize_remote(monkeypatch,inputs,now_ns):
 """Return real ModeAware node, preserving REAL monotonic for worker probes.

 now_ns is caller's thread-safe historical ROS clock mapping. The existing
 fixture establishes non-hardware defaults; recorded metadata then binds owner.
 No fake private work is installed: caller must submit its real Snapshot/ticket.
 """
 c=context.__wrapped__(monkeypatch);c=station.__wrapped__(c,monkeypatch);n=c.node;configure(c)
 at=lambda ns:rospy.Time(*divmod(int(ns),10**9))
 monkeypatch.setattr(rospy.Time,'now',staticmethod(lambda:at(now_ns())))
 n._ros_source_clock_ns=now_ns;n._stream_source_clock=lambda:now_ns()*1e-9
 import alicia_grasp_modes.station_compute_wait_remote as impl
 # station fixture replaces time.monotonic with synthetic clock; restore it
 # explicitly so worker/callback deadline handling uses the caller's real clock.
 monkeypatch.setattr(impl.time,'monotonic',_REAL_MONOTONIC)
 n._reset_station_compute_wait();n._initialize_unknown_pending_identity()
 b=inputs.binding;n._mode_selection=deepcopy(b['selection']);n._mode_ready=True;c.params[module.SELECTION_PARAM]=deepcopy(b['selection'])
 n._stream_generation=7;n._target_identity_generation=1;n.target_instance_epoch=4
 # Preserve the original coordinator object/lock; supply only its recorded
 # generation/next request counters and the same historical ROS clock mapping.
 # There is no fabricated active/pending computation or result here.
 with n.inference_coordinator._lock:
  n.inference_coordinator._clock=n._stream_source_clock
  n.inference_coordinator._generation=7
  n.inference_coordinator._next_request_id=15
  n.inference_coordinator._target_epoch=4
 n._observe_return_session=deepcopy(inputs.session);n._observe_return_prefix=deepcopy(inputs.plan);n._observe_return_origin_prefix=deepcopy(inputs.plan)
 n._observe_return_phase='observing_contact';n._observe_return_paused=False;n._observe_return_diagnostic_observation_only=False
 n._measurement_remote_request=deepcopy(inputs.lease);n._measurement_remote_epoch_ns=b['control_epoch_ns'];n._measurement_remote_reference={'binding':b['reference_binding']};n._measurement_remote_admission={}
 n._station_compute_phase=deepcopy(inputs.phase);n._station_compute_task_state='PLAN_PREGRASP';n.robot_execution_active=True;n.near_field_planning_active=True
 n._near_field_phase_started_sec=inputs.phase.header.stamp.to_sec();n._near_field_phase_deadline_sec=inputs.phase.deadline.to_sec()
 n.latest_object=n.latest_object_time=None;n.latest_joint_state=None;n.frames=base.SynchronizedRgbdBuffer(source_clock_ns=now_ns)
 n._station_compute_work={};n._station_compute_work_published=False;n._station_compute_publication_started=False
 c.node=n;c.inputs=inputs;c.initialization_provider_limit='Existing real-remote offline fixture defaults; exact recorded owner/session/phase/selection then prebound. No producer requalification, historical private surface/CAS state or Task execution reconstructed.'
 return c

def callback_events(inputs,masks):
 """masks: source_ns -> (original bag timestamp_ns, original Image message)."""
 assert set(inputs.objects)==set(masks)=={x['parsed_json']['source_stamp_ns']for x in inputs.statuses}
 events=[]
 for row,msg in inputs.sdk:events.append((row['bag_timestamp_ns'],0,'sdk',row['source_stamp_ns'],msg))
 for row in inputs.statuses:
  s=row['parsed_json']['source_stamp_ns'];events.extend([(row['received_ns'],1,'status',s,row['parsed_json']),(inputs.objects[s][0]['received_ns'],2,'object',s,inputs.objects[s][1]),(masks[s][0],3,'mask',s,masks[s][1])])
 return sorted(events,key=lambda e:(e[0],e[1]))

def deliver_original_callback(node,kind,value):
 if kind=='sdk':node.latest_joint_state=deepcopy(value);return node._station_compute_accepted_cb(deepcopy(value))
 if kind=='status':return deliver(node,{'status':value},('status',))
 if kind in ('mask','object'):return getattr(node,kind+'_cb')(deepcopy(value))
 raise ValueError('Unsupported callback kind '+str(kind))
