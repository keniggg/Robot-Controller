"""Replay exact terminal/clock wires through original Task methods, no ROS I/O.

The registered owner is explicitly reconstructed from recorded owner metadata;
SDK samples are stationary providers at the recorded q, not historical samples.
This probe establishes control-flow causality, never historical callback order.
"""
import argparse,base64,hashlib,io,json,os,platform,struct,sys,time
from copy import deepcopy
from pathlib import Path

W=Path(__file__).resolve().parents[1]
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
V=E/'ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
INPUT=V/'gen49_g48_terminal_messages_r2/result.json'
BASE=W/'.runtime/gen49_baseline_source'

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 out=a.output.resolve();out.relative_to(V);assert not out.exists();out.mkdir();(out/'tmp').mkdir()
 os.environ.update(TMPDIR=str(out/'tmp'),ROS_HOME=str(out/'tmp'),ROS_LOG_DIR=str(out/'tmp'),ROS_MASTER_URI='http://127.0.0.1:1',ROS_PACKAGE_PATH=str(BASE/'src'),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1')
 sys.dont_write_bytecode=True;boundaries=[]
 def guard(event,args):
  if event.startswith('socket.') or event in ('subprocess.Popen','os.system','os.exec','os.posix_spawn','os.fork','os.forkpty'):
   boundaries.append(event);raise RuntimeError('offline boundary:'+event)
  if event=='open'and isinstance(args[0],(str,bytes,os.PathLike)):
   name=os.fsdecode(args[0]);flags=args[2]if len(args)>2 and isinstance(args[2],int)else 0;mode=args[1]if len(args)>1 and isinstance(args[1],str)else ''
   if (name.startswith('/dev/')and name not in ('/dev/null','/dev/urandom','/dev/random'))or'/hwmon/'in name or name.startswith('/sys/class/thermal'):
    boundaries.append(name);raise RuntimeError('device boundary')
   if name!='/dev/null'and(flags&(os.O_WRONLY|os.O_RDWR|os.O_CREAT|os.O_TRUNC|os.O_APPEND)or any(x in mode for x in 'wax+')):
    try:Path(name).resolve().relative_to(out)
    except ValueError:boundaries.append(name);raise RuntimeError('external write boundary')
 sys.addaudithook(guard)
 u=os.uname();platform.uname=lambda:platform.uname_result(*(tuple(u)+(u.machine,)));platform.architecture=lambda *a,**k:(str(8*struct.calcsize('P'))+'bit','ELF')
 sys.path[:0]=[str(BASE/'src/alicia_grasp_modes/src'),str(BASE/'src/alicia_flexible_grasp_supervisor/src'),str(BASE/'src/alicia_grasp_modes/tests'),str(BASE/'src/alicia_flexible_grasp_supervisor/tests'),str(W/'devel/lib/python3/dist-packages'),'/opt/ros/noetic/lib/python3/dist-packages']
 import pytest,rospy,roslib.message
 from test_station_compute_wait_task import station,configured,feedback,at_ns,base
 from alicia_grasp_modes import station_compute_wait_task as impl
 from alicia_flexible_grasp.robot.observation_view_gateway import _camera_configuration_contract
 from alicia_grasp_modes.rgb_observation_source import digest
 assert sha(BASE/'manifest.json')=='21f6c6a1b0b8072d78332a081af1a425c570c9e5dd1983b2f982960e3911e935'
 assert sha(INPUT)=='30641768cb887ede0ee402ef8434d0813bbc4fe7d84c653e4d1bed48bc114675'
 extract=json.loads(INPUT.read_text());evidence={str(INPUT):sha(INPUT),str(BASE/'manifest.json'):sha(BASE/'manifest.json')}
 def rows(topic):
  info=extract['files'][topic];q=Path(info['path']);assert sha(q)==info['sha256'];evidence[str(q)]=sha(q);return json.loads(q.read_text())['messages']
 def decode(row):
  wire=base64.b64decode(row['wire_base64']);assert hashlib.sha256(wire).hexdigest()==row['wire_sha256'];cls=roslib.message.get_message_class(row['message_type']);m=cls();m.deserialize(wire);return m
 previews=rows('/grasp_6d/preview_plan_enriched');terminal=next(x for x in previews if x['payload']['candidate_source']=='near_field_terminal');index=previews.index(terminal);clock=previews[index+1]
 assert terminal['wire_sha256']=='20302b2630344f2a57e40bb7354a147ac3b21f47b88b6b66140b4411a96feb43'
 geometry=rows('/grasp_6d/object_geometry');pending=[x for x in geometry if x['received_ns']<terminal['received_ns']and x['payload']['failure_reason'].startswith('TARGET_PROCESSING_PENDING:')][-1]
 contract=json.loads(pending['payload']['failure_reason'].partition(':')[2])['station_compute_wait'];binding=contract['binding']
 statuses=rows('/perception/unknown/detector_status');status=next(x['parsed_json']for x in statuses if(x['parsed_json']or{}).get('station_compute_wait')==contract)
 objects=rows('/perception/object');obj=next(decode(x)for x in objects if x['payload']['header']['stamp']['secs']*10**9+x['payload']['header']['stamp']['nsecs']==contract['source_ns']and x['payload']['detected'])
 session=deepcopy(rows('/grasp_mode/observe_return_session')[0]['parsed_json'])
 for key in ('stamp_ns','deadline_ns','origin_snapshot_stamp_ns','view_source_stamp_ns','return_verified_stamp_ns'):session[key]=int(session[key])
 lease=deepcopy(rows('/grasp_mode/observe_return_measurement_request')[-1]['parsed_json'])
 phase=decode(next(x for x in rows('/grasp/near_field_phase')if x['payload']['active']))
 plan=decode(next(x for x in rows('/grasp_6d/plan_enriched')if x['payload']['valid']and x['payload']['plan_id']==binding['view_plan_id']))
 camera_path=V.parent/'gen48_runtime/fresh_scene_r3/camera_configuration.json';camera=json.loads(camera_path.read_text());evidence[str(camera_path)]=sha(camera_path)
 assert digest(_camera_configuration_contract(camera))==binding['camera_configuration_sha256']
 from _pytest.monkeypatch import MonkeyPatch
 mp=MonkeyPatch()
 try:
  c=station.__wrapped__(configured.__wrapped__(mp),mp);n=c.node
  c.selection=deepcopy(binding['selection']);c.params.update({'/camera':camera,'/gui/joint_direct_mode':False});c.params['/grasp_mode/selection']=deepcopy(c.selection)
  import test_task_modes as task
  c.params[task.module.SELECTION_PARAM]=deepcopy(c.selection);n._mode_selection=deepcopy(c.selection);n._mode_bound_selection=deepcopy(c.selection);n._mode_ready=True
  n._bound_execution_plan=deepcopy(plan);n._bound_execution_plan_id=plan.plan_id;n._bound_execution_plan_digest=n._execution_plan_digest(plan);c.plan=plan
  n._observe_return_session=session;n._observation_measurement_request=lease;n._published_near_field_phase=phase;n._near_field_phase_id=phase.phase_id;n._near_field_phase_deadline_sec=phase.deadline.to_sec();n._following_epoch_ns=binding['control_epoch_ns']
  n._following_accepted_history.clear();n._following_sdk_history.clear();n._reset_station_compute_task()
  def measured(now):
   c.now[0]=now;c.mono[0]=10.+(now-terminal['received_ns'])/1e9
   m=base.JointState();m.header.frame_id='sdk_measured';m.header.stamp=at_ns(now);m.name=['Joint%d'%i for i in range(1,7)]+['right_finger'];m.position=list(binding['station_q_rad'])+[.05];n.accepted_joint_cb(m)
  measured(terminal['received_ns'])
  n._station_task_record=dict(contract=deepcopy(contract),pending=False,object=obj,mask=deepcopy(status['station_compute_mask']),geometry_mono=c.mono[0]+(contract['geometry_deadline_ns']-c.now[0])/1e9,identity_mono=c.mono[0]+(contract['identity_deadline_ns']-c.now[0])/1e9)
  n._station_contact_binding=deepcopy(binding)
  trace=[]
  def observe(label):
   failure=n._near_field_wait_failure(plan,{})
   result,candidate=n._copy_near_field_preview_candidate(plan,phase.header.stamp.to_nsec(),dict(near_field_strategy='single_snapshot_direct'))
   recovery=n._before_near_field_terminal_recovery(result,plan,{},None,None)
   trace.append(dict(label=label,now_ns=c.now[0],hook_failure=None if failure is None else dict(code=failure.code,reason=failure.reason),copy_result=dict(ok=result.ok,code=result.code,reason=result.reason),recovery=recovery,pending=n._station_task_record['pending'],retired=n._station_task_retired,preview_code=n.latest_grasp6d_preview_plan.diagnostic.partition(':')[0],preview_source=n.latest_grasp6d_preview_plan.candidate_source))
   assert failure is None and not result.ok and result.code==impl.PENDING and candidate is None and recovery=={'waiting':True}
  n.grasp6d_preview_plan_cb(decode(terminal));observe('exact terminal received while original current geometry is pending')
  measured(clock['received_ns']);n.grasp6d_preview_plan_cb(decode(clock));observe('next exact clock preview overwrites terminal')
  assert not boundaries
  result=dict(kind='GEN49_exact_G48_terminal_Task_baseline_control_flow_probe',passed=True,baseline_bug_reproduced=True,trace=trace,original_terminal_header_ns=decode(terminal).header.stamp.to_nsec(),terminal_received_ns=terminal['received_ns'],next_clock_received_ns=clock['received_ns'],terminal_visible_transport_ns=clock['received_ns']-terminal['received_ns'],phase_deadline_ns=phase.deadline.to_nsec(),registered_contract=contract,evidence_sha256=evidence,boundary_attempts=boundaries,providers=['Registered owner is reconstructed from recorded session/lease/phase/contract; admission before this interval is not replayed.','Fresh stationary SDK is a provider at recorded station_q_rad; no historical SDK delivery or subscriber lock ordering is claimed.'],ROS_initialized=False,hardware_contacted=False,temperature_read=False,production_tests_records_modified=False,helper_sha256=sha(__file__))
  q=out/'result.json';q.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');assert json.loads(q.read_text())==result
  print(json.dumps(dict(result=str(q),sha256=sha(q),trace=trace),ensure_ascii=False))
 finally:mp.undo()
if __name__=='__main__':main()
