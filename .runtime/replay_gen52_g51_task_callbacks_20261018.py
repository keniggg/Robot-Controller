"""Original GEN51 Task callbacks with exact final G51 wire and explicit initial owner.

This schedule uses recorder bundle completion, never claims original subscriber
processing order. No motion/geometry/MoveIt provider is called.
"""
from copy import deepcopy
import ast, base64, hashlib, io, json, os, platform, struct, sys, traceback
from pathlib import Path
from types import SimpleNamespace as NS
import extract_gen52_g51_station_wire_20261018 as p
import extract_gen50_retry_r30_rgb_20261018 as boundary
W=p.W; B=W/'.runtime/gen52_baseline_source'; I=p.OUT; O=I/'task_original_callback_replay_r5'

def main():
 assert p.E.parent.parent.is_mount() and not O.exists(); O.mkdir()
 os.environ.update(PYTHONDONTWRITEBYTECODE='1',ROS_MASTER_URI='http://127.0.0.1:1',ROS_PACKAGE_PATH=str(B/'src'),ROS_LOG_DIR=str(O),ROS_HOME=str(O),TMPDIR=str(O),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
 sys.dont_write_bytecode=True
 n=os.uname();platform.uname=lambda:platform.uname_result(*(tuple(n)+(n.machine,)));platform.architecture=lambda *a,**k:(str(8*struct.calcsize('P'))+'bit','ELF')
 sys.path[:0]=[str(B/'src/alicia_grasp_modes/tests'),str(B/'src/alicia_flexible_grasp_supervisor/tests'),str(B/'src/alicia_grasp_modes/src'),str(B/'src/alicia_flexible_grasp_supervisor/src'),str(W/'devel/lib/python3/dist-packages'),'/opt/ros/noetic/lib/python3/dist-packages']
 boundary.OUT=O;sys.addaudithook(boundary.audit)
 import rosbag, roslib.message, rospy
 from _pytest.monkeypatch import MonkeyPatch
 from test_station_compute_wait_task import station
 from test_task_modes import configured, module, base
 from alicia_grasp_modes import station_compute_wait_task as wait
 from alicia_grasp_modes.rgb_observation_source import digest
 from alicia_flexible_grasp.robot.observation_view_gateway import _camera_configuration_contract
 ex=json.loads((I/'extraction.json').read_text());cx=json.loads((I/'callback_context_r1/result.json').read_text())
 def read(spec):
  assert boundary.sha(spec['path'])==spec['sha256'];return json.loads(Path(spec['path']).read_text())['messages']
 def rows(topic):return read(ex['files'][topic])
 def decode(row):
  raw=base64.b64decode(row['wire_base64']);assert hashlib.sha256(raw).hexdigest()==row['wire_sha256']
  cls=roslib.message.get_message_class(row['message_type']);msg=cls();msg.deserialize(raw);assert msg._md5sum==row['md5sum'];return msg
 masks=[]
 # Read the original mask topic only. Pixels stay in memory; the sealed bag
 # and independently pinned full descriptors remain their recovery source.
 with rosbag.Bag(str(p.G/'keyframes.bag'),'r')as bag:
  for topic,msg,received in bag.read_messages(topics=['/perception/object_mask']):
   if 1792329988605104923<=msg.header.stamp.to_nsec()<=1792330055166587829:masks.append((received.to_nsec(),msg.header.stamp.to_nsec(),msg))
 statuses=rows('/perception/unknown/detector_status');sdk=read(cx['files']['/alicia_d/accepted_joint_states'])
 seed=1792329988605104923; end=1792330055166587829; start=seed
 seed_status=next(r['parsed_json']for r in statuses if (r['parsed_json']or{}).get('source_stamp_ns')==seed)
 binding=seed_status['station_compute_wait']['binding']
 session=deepcopy(rows('/grasp_mode/observe_return_session')[0]['parsed_json'])
 for k in ('stamp_ns','deadline_ns','origin_snapshot_stamp_ns','view_source_stamp_ns','return_verified_stamp_ns'):session[k]=int(session[k])
 lease=deepcopy(rows('/grasp_mode/observe_return_measurement_request')[-1]['parsed_json'])
 phase=decode(next(r for r in rows('/grasp/near_field_phase')if r['payload']['active']is True))
 plan=decode(next(r for r in rows('/grasp_6d/plan_enriched')if r['payload']['valid']is True and r['payload']['plan_id']==binding['view_plan_id']))
 cameras=read(cx['files']['/grasp_record/camera_runtime_metadata']);camera=next(r['parsed_json']['parameters']for r in cameras if isinstance((r['parsed_json']or{}).get('parameters'),dict))
 assert digest(_camera_configuration_contract(camera))==binding['camera_configuration_sha256']
 at=lambda ns:rospy.Time(*divmod(ns,10**9));events=[]
 for r in sdk:events.append((r['received_ns'],0,'sdk',decode(r).header.stamp.to_nsec(),decode(r)))
 for r in statuses:
  if r['received_ns']>=start and (r['parsed_json']or{}).get('source_stamp_ns',0)>=seed:events.append((r['received_ns'],1,'status',(r['parsed_json']or{}).get('source_stamp_ns',0),deepcopy(r['parsed_json'])))
 for r in rows('/perception/object'):
  msg=decode(r)
  if msg.header.stamp.to_nsec()>=seed:events.append((r['received_ns'],2,'object',msg.header.stamp.to_nsec(),msg))
 for recv,source,msg in masks:
  if source>=seed:events.append((recv,3,'mask',source,msg))
 for r in rows('/grasp_6d/object_geometry'):
  if start<=r['received_ns']<=end:events.append((r['received_ns'],4,'geometry',0,decode(r)))
 events.append((end,5,'tick',0,None));events.sort(key=lambda e:(e[0],e[1]))
 mp=MonkeyPatch();trace=[];exceptions=[];n=None;errors=[];now=[start]
 try:
  def forbidden(*a,**k):boundary.deny('ROS_live_boundary')
  for name in ('init_node','Publisher','Subscriber','Service','ServiceProxy','wait_for_service','set_param','delete_param'):mp.setattr(rospy,name,forbidden)
  configured_values=configured.__wrapped__(mp);c=station.__wrapped__(configured_values,mp);n=c.node
  mp.setattr(rospy.Time,'now',staticmethod(lambda:at(now[0])));mp.setattr(wait.time,'monotonic',lambda:now[0]/1e9)
  n._reset_station_compute_task();n.active=True;n.stage=base.GraspStages.PLAN_PREGRASP
  n._mode_ready=True;n._mode_selection=deepcopy(binding['selection']);n._mode_bound_selection=deepcopy(binding['selection']);c.params[module.SELECTION_PARAM]=deepcopy(binding['selection']);c.params['/camera']=camera;c.params['/gui/joint_direct_mode']=False
  n._observe_return_session=session;n._observation_measurement_request=lease
  n._published_near_field_phase=phase;n._near_field_active=True;n._near_field_phase_id=phase.phase_id;n._near_field_phase_deadline_sec=phase.deadline.to_sec()
  n._bound_execution_plan=plan;n._bound_execution_plan_id=plan.plan_id;n._bound_execution_plan_digest=n._execution_plan_digest(plan);n.latest_grasp6d_plan=deepcopy(plan);n.latest_target_geometry=deepcopy(plan.object_geometry);n._execution_authority_revoked=False
  n._following_epoch_ns=binding['control_epoch_ns'];n._following_accepted_history.clear();n._following_sdk_history.clear()
  for r in sdk:
   if r['received_ns']<start:n._following_accepted_history.append(decode(r))
  def state(tag,**kw):
   r=n._station_task_record
   geometry=getattr(n,'latest_target_geometry',None)
   trace.append(dict(tag=tag,now_ns=now[0],active_source_ns=None if r is None else r['contract']['source_ns'],pending=None if r is None else r['pending'],retired=n._station_task_retired,authority_revoked=n._execution_authority_revoked,geometry_valid=None if geometry is None else geometry.valid,geometry_source_ns=None if geometry is None else geometry.header.stamp.to_nsec(),geometry_failure=None if geometry is None else geometry.failure_reason,
      availability_pending_sources=sorted(n._station_task_availability_pending),cache_sources={k:sorted(getattr(n,k))for k in ('_station_task_statuses','_station_task_objects','_station_task_masks')},**kw))
  original_retire=n._station_task_retire
  def retire(reason):state('retire',reason=str(reason));return original_retire(reason)
  mp.setattr(n,'_station_task_retire',retire)
  context_source=Path(wait.__file__).read_text();tree=ast.parse(context_source)
  context_fn=next(x for x in ast.walk(tree)if isinstance(x,ast.FunctionDef)and x.name=='_station_task_context')
  owner_checks=next(x for x in context_fn.body if isinstance(x,ast.If)).test.values
  def tracer(frame,event,arg):
   if event=='exception'and frame.f_code.co_name=='_station_task_context'and str(arg[1])=='station_task_owner_changed':
    failed=[]
    for check in owner_checks:
     try:
      if eval(compile(ast.Expression(body=check),'<original owner predicate>','eval'),frame.f_globals,frame.f_locals):failed.append(ast.get_source_segment(context_source,check))
     except Exception as exc:failed.append(type(exc).__name__+': '+str(exc))
    exceptions.append(dict(now_ns=now[0],function='_station_task_context',reason='station_task_owner_changed',failed_original_disjuncts=failed))
   if event=='exception'and frame.f_code.co_name in ('_station_task_join','_station_task_check')and str(B)in frame.f_code.co_filename:
    typ,exc,tb=arg;contract=frame.f_locals.get('contract');record=frame.f_locals.get('record')
    chain=[]
    while tb is not None:chain.append(dict(function=tb.tb_frame.f_code.co_name,line=tb.tb_lineno));tb=tb.tb_next
    exceptions.append(dict(now_ns=now[0],function=frame.f_code.co_name,error_type=typ.__name__,reason=str(exc),offered_source_ns=contract.get('source_ns')if isinstance(contract,dict)else None,active_source_ns=(record or{}).get('contract',{}).get('source_ns'),chain=chain))
   return tracer
  sys.settrace(tracer)
  for stamp,priority,kind,source,value in events:
   if not start<=stamp<=end:continue
   now[0]=stamp;state('input_before',kind=kind,source_ns=source)
   if kind=='sdk':n.accepted_joint_cb(deepcopy(value))
   elif kind=='object':n.obj_cb(deepcopy(value))
   elif kind=='mask':n._station_compute_task_mask_cb(deepcopy(value))
   elif kind=='status':n._unknown_support_status_cb(NS(data=json.dumps(value)))
   elif kind=='geometry':n.target_geometry_cb(deepcopy(value))
   else:n._station_compute_task_tick()
   state('input_after',kind=kind,source_ns=source)
   if n._station_task_retired:break
 except BaseException:errors.append(traceback.format_exc())
 finally:sys.settrace(None);mp.undo()
 p.put(O/'trace.json',trace);p.put(O/'join_exceptions.json',exceptions)
 manifest=json.loads((B/'runner_manifest.json').read_text());p.put(O/'result.json',dict(completed=not errors,errors=errors,final_source=None if n is None or n._station_task_record is None else n._station_task_record['contract']['source_ns'],retired=None if n is None else n._station_task_retired,
    baseline_manifest_sha256=boundary.sha(B/'runner_manifest.json'),source_sha256=manifest['source_sha256'],
    trace_sha256=boundary.sha(O/'trace.json'),exceptions_sha256=boundary.sha(O/'join_exceptions.json'),external_boundary_attempts=boundary.BOUNDARY,
    initial_owner_provider='Original recorded selection/session/qualified lease/phase/plan/camera and accepted SDK history prebound into original Task fixture; no replay of earlier Task motion.',
    schedule_provider='Recorded bundle timestamps used as counterfactual subscriber processing schedule; no claim original Task callback ingress/order is recorded.',temperature_read=False,ROS_initialized=False,hardware_contacted=False,production_or_old_tests_modified=False,helper_sha256=boundary.sha(__file__)))
 print(json.dumps(dict(result=str(O/'result.json'),sha256=boundary.sha(O/'result.json'),completed=not errors,errors=errors,final_source=None if n is None or n._station_task_record is None else n._station_task_record['contract']['source_ns'],retired=None if n is None else n._station_task_retired,exceptions=len(exceptions)),ensure_ascii=False))

if __name__=='__main__':main()
