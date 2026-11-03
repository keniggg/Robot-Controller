#!/usr/bin/env python3
"""Offline actual producer callbacks/process with timed original pre-motion hold freeze.

Transport is replayed from saved receive clocks. Computation time always advances.
The original accepted pre-motion RGB reference is explicitly seeded; no arrival
success or station template is seeded. Default run retains original failure.
"""
import argparse,json,hashlib,importlib.util,threading,time,sys,inspect,functools,heapq
from copy import deepcopy
from dataclasses import fields,replace,is_dataclass,asdict
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock,patch
import cv2,numpy as np,rospy,tf2_py
from cv_bridge import CvBridge
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from alicia_flexible_grasp_supervisor.msg import NearFieldPlanningPhase,GraspState
from alicia_grasp_modes import tabletop as table
from alicia_grasp_modes.acquisition import InitialAcquisition
from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask

W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade');N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z');F=N/'verification/gen14_arrival_independent';PRE=1791860176945733070;ANCHOR=1791860167282787322

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def plain(x):
 if is_dataclass(x):return plain(asdict(x))
 if hasattr(x,'to_nsec'):return x.to_nsec()
 if hasattr(x,'__slots__'):return {k:plain(getattr(x,k))for k in x.__slots__}
 if isinstance(x,np.ndarray):return x.tolist()
 if isinstance(x,np.generic):return x.item()
 if isinstance(x,dict):return {k:plain(v)for k,v in x.items()}
 if isinstance(x,(tuple,list)):return [plain(v)for v in x]
 return x

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--candidate',default=str(W));p.add_argument('--expect-qualified',action='store_true');p.add_argument('--profile-functions',action='store_true');p.add_argument('--timer-phase',type=int,choices=(0,1,2),default=0);args=p.parse_args();O=Path(args.output);assert F in O.parents;O.mkdir(exist_ok=False);C=Path(args.candidate)
 paths=[C/'src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py']+[C/'src/alicia_grasp_modes/src/alicia_grasp_modes'/name for name in('observation_appearance.py','observation_arrival_confirmation.py','arrival_fragment_group.py','station_compute_wait_producer.py','observation_measured_face.py','arrival_dense_identity.py','arrival_train_only_proposal.py')];hashes={str(p):sha(p)for p in paths}
 spec=importlib.util.spec_from_file_location('actual_unknown_gen14',paths[0]);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
 f=json.loads((F/'fixture.json').read_text());full=json.loads((F/'full_arrival_frames/fixture.json').read_text());camera=f['camera'];cfg=f['unknown_parameters'];config=table.Config(**{k:v for k,v in cfg.items()if k in {x.name for x in fields(table.Config)}});K=[camera[k]for k in('fx','fy','cx','cy')];bridge=CvBridge();cv2.setNumThreads(1)
 executing=next(r for r in f['wire']if r['topic'].endswith('measurement_request')and r['message']['state']=='executing');requal=next(r for r in f['wire']if r['topic'].endswith('measurement_request')and r['message']['state']=='requalifying');selection=executing['message']['selection'];epoch=executing['message']['control_epoch_ns']
 sys.path.insert(0,str(W/'.runtime'))
 from gen14_remote_fusion_fixture_20261013 import load_recorded_parameters
 params,parameter_audit=load_recorded_parameters(camera,cfg,selection);deployment=json.loads((N/'full_restart_acceptance/result.json').read_text());assert hashlib.sha256(params['/robot_description'].encode()).hexdigest()==deployment['robot_description_sha256']
 def array(ns):return np.load(F/f'{ns}_source.npz')
 statuses=json.loads((F/'all_detector_statuses.json').read_text());pre_status=next(row['message']for row in statuses if row['message'].get('source_stamp_ns')==PRE and row['message'].get('state')=='ready');pre_timing=pre_status['frame_timing'];pre_hold_component=pre_timing['rgb_components']['hold.total'][0];pre_hold_start_ns=pre_timing['processing_start_ros_ns']+round(pre_hold_component['start_elapsed_sec']*1e9)
 for row in f['frames']:
  assert sha(Path(row['file']))==row['sha256'],row['file']
 for row in full['frames']:
  assert sha(Path(row['file']))==row['sha256'],row['file']
 d=array(ANCHOR);seg=table.segment_with_support_fallback(d['depth'],K,d['T_base_camera'],config);tracker=table.TargetTracker(config);anchor,_=tracker.choose(seg,d['depth'].shape);plane=seg.plane_base.copy();d=array(PRE);seg=table.segment_with_support_fallback(d['depth'],K,d['T_base_camera'],config,anchor,anchor,plane);matches=table._matching_candidates(seg.candidates,anchor,anchor,config);assert len(matches)==1;last=matches[0][1]
 original_plane=np.asarray(pre_status['reference_support_plane_base'],dtype=float);original_anchor_position=np.asarray(pre_status['anchor_position_base'],dtype=float);original_anchor_extent=np.asarray(pre_status['anchor_extent_m'],dtype=float);original_footprint=np.asarray(pre_status['reference_support_footprint_base'],dtype=float)
 historical_seed_residuals={key:float(np.max(np.abs(calculated-recorded)))for key,calculated,recorded in [('plane',plane,original_plane),('anchor_position',anchor.position_base,original_anchor_position),('anchor_extent',anchor.extent_m,original_anchor_extent),('anchor_footprint',anchor.support_footprint_base,original_footprint)]};assert all(value<1e-12 for value in historical_seed_residuals.values())
 anchor=replace(anchor,position_base=original_anchor_position,extent_m=original_anchor_extent,support_footprint_base=original_footprint,metrics=dict(anchor.metrics,support_plane_base=original_plane.tolist()));plane=original_plane.copy()
 now=[pre_hold_start_ns];running=[False];start=[now[0]];perf=[0];tf_index=[0];feed_images=[False];current_image=[-1];frames=full['frames'];samples=full['sdk'];dynamic=full['dynamic_tf'];image_pairs={};remote_box=[None];remote_input_events=[];remote_input_index=[0];remote_input_busy=[False];remote_input_journal=[];last_clock_return=[now[0]];clock_return_count=[0]
 def stamp(ns):return rospy.Time(ns//10**9,ns%10**9)
 def make_pair(data,info):
  rgb=bridge.cv2_to_imgmsg(data['rgb'],encoding='bgr8');depth=bridge.cv2_to_imgmsg(data['raw_depth'],encoding=info.get('depth_encoding','16UC1'))
  for msg,key in ((rgb,'rgb_header'),(depth,'depth_header')):
   h=info[key];msg.header.seq=h['seq'];msg.header.stamp=stamp(h['stamp']);msg.header.frame_id=h['frame_id']
  return rgb,depth
 preinfo=next(x for x in f['frames']if x['source_stamp_ns']==PRE);presrc=make_pair(d,preinfo)
 for row in frames:image_pairs[row['source_ns']]=make_pair(np.load(row['file']),row)
 for row in frames:
  rgb,depth=image_pairs[row['source_ns']];remote_input_events.extend([(row['rgb_received_ns'],'color',rgb),(row['depth_received_ns'],'depth',depth)])
 for row in samples:
  msg=JointState();msg.header.stamp=stamp(row['stamp_ns']);msg.name=row['names'];msg.position=row['positions'];remote_input_events.append((row['received_ns'],'joint',msg))
 remote_input_events.sort(key=lambda row:row[0])
 node=module.UnknownTabletopPerception.__new__(module.UnknownTabletopPerception);node.lock=threading.RLock();node.input_lock=threading.RLock();node.mode='unknown';node.generation=selection['generation'];node.selection_ns=selection['stamp_ns'];node.max_age=float(cfg.get('max_frame_age_sec',.8));node.tf_wait=float(cfg.get('tf_wait_sec',.4));node.minimum_quality=.5;node.base_frame='base_link';node.camera_frame='camera_link';node.config=config;node.rgb_instance_refinement=True;node.bridge=bridge;node.last_shape=(camera['height'],camera['width']);node.anchor_source_stamp_ns=ANCHOR;node.latest=None;node.last_consumed_ns=PRE;node.last_receive_wall=now[0]*1e-9;node.first_loss=None;node.last_good_source_stamp_ns=PRE;node.tracker=table.TargetTracker(config);node.tracker.anchor=anchor;node.tracker.last=last;node.tracker.plane_base=plane.copy();node.acquisition=InitialAcquisition(config);node._initialize_rgb_interest(cfg);node._rgb_interest_epoch=epoch;node.pubs={key:Mock()for key in('object','object_mask','object_detected','raw_object_detected','object_pose_camera','object_pose_base','detector_status','rgb_reference_status')};node._observation_measurement_ack_pub=Mock()
 buffer=tf2_py.BufferCore(rospy.Duration(600))
 def tfmsg(d):
  x=TransformStamped();x.header.seq=d['header']['seq'];x.header.stamp=stamp(d['header']['stamp']);x.header.frame_id=d['header']['frame_id'];x.child_frame_id=d['child_frame_id']
  for field in ('translation','rotation'):
   for key,value in d['transform'][field].items():setattr(getattr(x.transform,field),key,value)
  return x
 for row in full['static_tf']:buffer.set_transform_static(tfmsg(row['transform']),'original_static')
 def raw_clock():return start[0]+time.perf_counter_ns()-perf[0]if running[0]else now[0]
 def clock():
  at=raw_clock();node._rgb_interest_feedback.clear()
  for s in samples:
   if s['received_ns']<=at:
    q=dict(zip(s['names'],s['positions']));node._rgb_interest_feedback.append((s['stamp_ns'],np.array([q[k]for k in module.JOINT_NAMES])))
  while tf_index[0]<len(dynamic)and dynamic[tf_index[0]]['received_ns']<=at:buffer.set_transform(tfmsg(dynamic[tf_index[0]]['transform']),'original_dynamic');tf_index[0]+=1
  if feed_images[0]:
   indices=[i for i,r in enumerate(frames)if r['pair_available_ns']<=at]
   if indices and indices[-1]>current_image[0]:current_image[0]=indices[-1];node.latest=image_pairs[frames[current_image[0]]['source_ns']];node.last_receive_wall=frames[current_image[0]]['pair_available_ns']*1e-9
  if remote_box[0] is not None and not remote_input_busy[0]:
   remote_input_busy[0]=True
   try:
    while remote_input_index[0]<len(remote_input_events) and remote_input_events[remote_input_index[0]][0]<=at:
     received,kind,msg=remote_input_events[remote_input_index[0]];remote_input_index[0]+=1
     if kind!='joint' and not feed_images[0]:continue
     entered=raw_clock();getattr(remote_box[0],kind+'_cb')(msg)
     remote_input_journal.append(dict(kind=kind,original_received_ns=received,replay_callback_ns=entered,source_ns=msg.header.stamp.to_nsec(),completion_ns=raw_clock()))
   finally:remote_input_busy[0]=False
  result=raw_clock();assert result>=last_clock_return[0],('replay clock moved backward',last_clock_return[0],result)
  last_clock_return[0]=result;clock_return_count[0]+=1
  return result
 def lookup(target,source,when,timeout):
  limit=raw_clock()+int(timeout.to_sec()*1e9)
  while True:
   clock()
   try:return buffer.lookup_transform_core(target,source,when)
   except (tf2_py.LookupException,tf2_py.ConnectivityException,tf2_py.ExtrapolationException):
    if not running[0]or raw_clock()>=limit:raise
    time.sleep(.002)
 node.buffer=SimpleNamespace(lookup_transform=lookup)
 def begin(at):
  at=max(at,now[0]);now[0]=at;start[0]=at;perf[0]=time.perf_counter_ns();running[0]=True;clock()
 def end():clock();elapsed=time.perf_counter_ns()-perf[0];now[0]=start[0]+elapsed;running[0]=False;return elapsed*1e-9
 clock_defaults=[];original_monotonic=time.monotonic
 for name,loaded_module in list(sys.modules.items()):
  if name.startswith('alicia_grasp_modes.'):
   for function_name,function in vars(loaded_module).items():
    if inspect.isfunction(function) and function.__kwdefaults__ and function.__kwdefaults__.get('clock') is original_monotonic:
     function.__kwdefaults__=dict(function.__kwdefaults__,clock=lambda:clock()*1e-9);clock_defaults.append(name+'.'+function_name)
 function_profile=[]
 if args.profile_functions:
  targets={'match_observation_appearance':1,'corroborate_measured_face':2,'extract_measured_face':1,'_affine_proposal':2,'evaluate_appearance':2,'segment_with_support_fallback':None};wrappers={}
  def wrapped(original,mask_index):
   @functools.wraps(original)
   def invoke(*values,**kw):
    entry=raw_clock();started=time.perf_counter_ns();result=None;error=None;mask_digest=None
    if mask_index is not None and len(values)>mask_index and isinstance(values[mask_index],np.ndarray):mask_digest=hashlib.sha256(np.ascontiguousarray(values[mask_index]).tobytes()).hexdigest()
    try:
     result=original(*values,**kw);return result
    except Exception as exc:error=type(exc).__name__+': '+str(exc);raise
    finally:
     function_profile.append(dict(function=original.__module__+'.'+original.__name__,start_ns=entry,completion_ns=raw_clock(),wall_sec=(time.perf_counter_ns()-started)*1e-9,current_mask_sha256=mask_digest,error=error,result_code=result.get('code') if isinstance(result,dict) else getattr(result,'code',None)))
   return invoke
  for name,loaded_module in list(sys.modules.items()):
   if name.startswith('alicia_grasp_modes.') or loaded_module is module:
    for function_name,function in list(vars(loaded_module).items()):
     if inspect.isfunction(function) and function.__name__ in targets:
      if id(function) not in wrappers:wrappers[id(function)]=wrapped(function,targets[function.__name__])
      setattr(loaded_module,function_name,wrappers[id(function)])

 sys.path.insert(0,str(W/'.runtime'))
 from gen14_three_owner_remote_20261013 import build_remote,LocalPublisher
 from gen14_three_owner_task_fixture_20261013 import build_task
 task_script=C/'src/alicia_grasp_modes/scripts/mode_aware_grasp_task.py'
 task_spec=importlib.util.spec_from_file_location('actual_task_gen14',task_script);task_module=importlib.util.module_from_spec(task_spec);task_spec.loader.exec_module(task_module)
 period_ns=round(1e9/float(cfg.get('detect_hz',6.)));timer_offset_ns=round(args.timer_phase*period_ns/3)
 events=[];event_id=[0];journal=[];rows=[];task_box=[None];next_tick=[frames[0]['pair_available_ns']+timer_offset_ns];seen=set();task_finished=[False];phase_records=[];capture_running=[False];task_preamble=[False];preamble_started=[0];preamble_wall=[None]
 request_delay=requal['received_ns']-requal['message']['stamp_ns'];assert 0<=request_delay<20000000
 seed_manifest=json.loads((F/'three_owner_seed/manifest.json').read_text());offsets=seed_manifest['original_intertopic_receive_offsets_ns']
 def schedule(label,callback,delay_ns=0):
  event_id[0]+=1;heapq.heappush(events,(raw_clock()+delay_ns,event_id[0],label,callback))
 def dispatch():
  while events and events[0][0]<=raw_clock():
   due,_,label,callback=heapq.heappop(events);entry=raw_clock();started=time.perf_counter_ns()
   callback();journal.append(dict(kind=label,due_ns=due,start_ns=entry,completion_ns=raw_clock(),wall_sec=(time.perf_counter_ns()-started)*1e-9))
 def ack_delivery(message):
  saved=deepcopy(message);payload=json.loads(saved.data)
  if task_box[0] is not None:schedule('task_ack_'+payload['participant']+'_'+payload['state'],lambda:task_box[0]._observation_measurement_ack_cb(saved),request_delay)
  if payload['participant']=='unknown' and remote_box[0] is not None:schedule('remote_unknown_ack_'+payload['state'],lambda:remote_box[0]._measurement_remote_ack_cb(saved),request_delay)
 def process_tick():
  clock();pair=node.latest
  if pair is None or pair[0].header.stamp.to_nsec() in seen:return
  ns=pair[0].header.stamp.to_nsec();seen.add(ns);entry=raw_clock();started=time.perf_counter_ns();node.pubs={k:Mock()for k in node.pubs}
  node.process(None);completed=raw_clock();elapsed=(time.perf_counter_ns()-started)*1e-9
  published=node.pubs['detector_status'].publish;status=json.loads(published.call_args[0][0].data)if published.call_args else None;obs=(status or{}).get('observation_measurement',{})
  row=dict(source_ns=ns,processing_start_ns=entry,completion_ns=completed,wall_sec=elapsed,source_age_sec=(completed-ns)*1e-9,state=None if status is None else status['state'],detail=None if status is None else status['detail'],measurement_reason=obs.get('reason'),accepted_sources=obs.get('source_stamps_ns'),measurement_qualified=obs.get('qualified'),status=status,last_good_ns=node.last_good_source_stamp_ns,first_loss=node.first_loss)
  rows.append(row);print(json.dumps({k:v for k,v in row.items()if k not in('status','first_loss')}),flush=True)
  if status is not None:
   msg=String(data=json.dumps(status));schedule('remote_status',lambda:remote_box[0]._silhouette_source_status_cb(msg),offsets['pre_status'])
  objpub=node.pubs['object'].publish
  if objpub.call_args:
   obj=deepcopy(objpub.call_args[0][0]);schedule('remote_object',lambda:remote_box[0].object_cb(obj),offsets['pre_object']);schedule('task_object',lambda:task_box[0].obj_cb(obj),offsets['pre_object'])
  maskpub=node.pubs['object_mask'].publish
  if maskpub.call_args:
   mask=deepcopy(maskpub.call_args[0][0]);schedule('remote_mask',lambda:remote_box[0].mask_cb(mask),offsets['pre_mask'])
  next_tick[0]=max(completed,entry+round(1e9/float(cfg.get('detect_hz',6.))))
 def pump():
  clock();dispatch()
  if capture_running[0] and raw_clock()>=next_tick[0] and raw_clock()<=frames[-1]['pair_available_ns']:
   process_tick();dispatch()
 class DrivenCondition:
  def __init__(self):self.lock=threading.RLock();self.waits=[]
  def __enter__(self):self.lock.acquire();return self
  def __exit__(self,*args):self.lock.release()
  def notify_all(self):pass
  def wait(self,timeout=None):
   assert timeout is not None and timeout<=.02+1e-9
   entry=raw_clock();time.sleep(max(.02,float(timeout)));pump();self.waits.append(dict(start_ns=entry,completion_ns=raw_clock(),requested_sec=timeout,minimum_schedule_delay_sec=.02))
 condition=DrivenCondition()
 def publish_request(message):
  request=json.loads(message.data)
  if task_preamble[0]:
   elapsed=time.perf_counter_ns()-preamble_started[0];preamble_wall[0]=elapsed*1e-9;start[0]=full['arrival_ns'];perf[0]=preamble_started[0];running[0]=True;task_preamble[0]=False;clock()
   assert request==requal['message'],('task generated request differs from original arrival',request,requal['message'])
  journal.append(dict(kind='task_request_publish',state=request['state'],now_ns=raw_clock(),request=request))
  copied=deepcopy(message);schedule('producer_request_'+request['state'],lambda:node._observation_measurement_request_cb(copied),request_delay);schedule('remote_request_'+request['state'],lambda:remote_box[0]._measurement_remote_request_cb(copied),request_delay)
 def publish_state(message):
  copy=deepcopy(message);schedule('producer_task_state',lambda:node._station_compute_state_cb(copy),request_delay);schedule('remote_task_state',lambda:remote_box[0].grasp_state_cb(copy),request_delay)
 def publish_phase(message):
  copy=deepcopy(message);phase_records.append(plain(message));schedule('producer_task_phase',lambda:node._station_compute_phase_cb(copy),request_delay);schedule('remote_task_phase',lambda:remote_box[0].near_field_state_cb(copy),request_delay)
 error=None;hold_timing=None;remote_audit=None;task_audit=None;frozen_face=None
 with patch.object(module.rospy,'get_param',lambda key,default=None:params.get(key,default)),patch.object(module.rospy,'get_param_cached',lambda key,default=None:params.get(key,default)),patch.object(module.rospy,'get_master',lambda:SimpleNamespace(getParam=lambda key:(1,'sealed offline snapshot',params.get(key)))),patch.object(module.rospy.Time,'now',staticmethod(lambda:stamp(clock()))),patch.object(module.time,'monotonic',lambda:clock()*1e-9),patch.object(module.rospy,'logwarn_throttle',Mock()),patch.object(module.rospy,'logwarn',Mock()),patch.object(module.rospy,'loginfo',Mock()):
  try:
   initial_dirs=[F/'initial_surface_preceding'/str(ns) for ns in (1791860167282787322,1791860167481009244)]+[F/'initial_surface_fixture']
   remote,remote_module,seed,remote_audit=build_remote(C,node,raw_clock,executing['message'],F/'three_owner_seed',ack_delivery,parameters=params,lookup_transform=lookup,initial_surface_directories=initial_dirs);remote_box[0]=remote
   task,gcfg,task_audit=build_task(task_module,plan=seed['original_view_plan'],session=json.loads(seed['original_session'].data),executing_request=executing['message'],settle=requal['message']['settle'],publish_request=publish_request,publish_state=publish_state,publish_phase=publish_phase,publish_near_field=lambda message:None,condition=condition);task_box[0]=task
   node._observation_measurement_ack_pub=LocalPublisher(ack_delivery)
   begin(pre_hold_start_ns);hold=node._prepare_locked_rgb_hold_reference(presrc,SimpleNamespace(mask=d['recorded_mask']),camera,source_transform=d['T_base_camera']);hold_wall=end();assert hold is not None;node._locked_rgb_hold_reference=hold;hold_timing=dict(start_ns=start[0],completed_ns=now[0],wall_sec=hold_wall,source_age_sec=(now[0]-PRE)*1e-9);assert now[0]-PRE<800000000
   begin(executing['received_ns']);node._observation_measurement_request_cb(String(data=json.dumps(executing['message'])));remote._measurement_remote_request_cb(String(data=json.dumps(executing['message'])));dispatch();end()
   assert json.loads(node._observation_measurement_ack_pub.messages[-1].data)['accepted']
   assert json.loads(remote._measurement_remote_ack_pub.messages[-1].data)['accepted']
   face=node._observation_measurement['confirmation'].appearance_reference.measured_face
   if face is not None:frozen_face=dict(source_ns=face.source.source_ns,source_binding_sha256=face.source.binding_sha256,dense_reference_digest=face.reference.digest,audit=json.loads(face.audit_json))
   assert node._observation_measurement['confirmation'].source_stamps_ns==[]
   # One historical arrival-event construction uses its recorded event stamp;
   # all of its measured CPU is charged before that request can be delivered.
   now[0]=full['arrival_ns'];running[0]=False;feed_images[0]=True;capture_running[0]=True;task_preamble[0]=True;preamble_started[0]=time.perf_counter_ns()
   task._observation_measurement_finish(seed['original_view_plan'],gcfg);task_finished[0]=True;dispatch()
   while raw_clock()<=frames[-1]['pair_available_ns'] and node.first_loss is None:
    time.sleep(.002);pump()
    producer_ref=getattr(node,'_station_compute_registration',None);remote_ref=getattr(remote,'_station_compute_reference',None)
    if producer_ref is not None and remote_ref is not None and producer_ref['contract']==remote_ref['contract']:
     break
   end()
  except Exception as exc:
   error=type(exc).__name__+': '+str(exc)
   import traceback;traceback.print_exc()
   if running[0]:end()
 loaded={name:dict(path=str(Path(value.__file__).resolve()),sha256=sha(Path(value.__file__)))for name,value in sys.modules.items()if name.startswith('alicia_grasp_modes.')and getattr(value,'__file__',None)and Path(value.__file__).suffix=='.py'}
 allacks=[]
 for participant,owner in [('unknown',node),('remote',remote_box[0])]:
  if owner is None:continue
  pub=getattr(owner,'_observation_measurement_ack_pub',None)if participant=='unknown'else getattr(owner,'_measurement_remote_ack_pub',None)
  if hasattr(pub,'messages'):allacks.extend(json.loads(m.data)for m in pub.messages)
 unchanged=all(sha(Path(k))==v for k,v in hashes.items());qualified_receipt=getattr(task_box[0],'_qualified_station_receipt',None)if task_box[0]is not None else None
 remote=remote_box[0];surface,reference=(None,None)if remote is None else remote._active_multiview_surface();registration=None if remote is None else getattr(remote,'_latest_registration_evidence',None)
 producer_registration=getattr(node,'_station_compute_registration',None)
 fusion=dict(phase=None if remote is None else remote._observe_return_phase,paused=None if remote is None else remote._observe_return_paused,
  surface_source_stamps_ns=[] if surface is None else list(surface.view_stamps_ns),surface_point_count=0 if surface is None else len(surface.points_base),
  registration=plain(registration),information_gain=None if remote is None else deepcopy(remote._observe_return_step_gain),
  reached_context=None if remote is None else deepcopy(remote._observe_return_reached_contact_context),
  producer_registration_contract=None if producer_registration is None else deepcopy(producer_registration['contract']),
  remote_registration_contract=None if remote is None or remote._station_compute_reference is None else deepcopy(remote._station_compute_reference['contract']))
 report=dict(passed=bool(task_finished[0]and error is None and unchanged and qualified_receipt is not None and any(r['state']=='ready'and r['detail']=='tracked'for r in rows)),error=error,source_unchanged=unchanged,source_sha256=hashes,loaded_dependency_sha256=loaded,rows=rows,event_journal=journal,acks=allacks,task_finish_returned=task_finished[0],task_receipt=None if qualified_receipt is None else dict(request=qualified_receipt['request'],acks=qualified_receipt['acks']),actual_task_phases=phase_records,task_waits=condition.waits,task_fixture=task_audit,remote_fixture=remote_audit,premotion_preparation_timing=hold_timing,frozen_premotion_face=frozen_face,task_original_arrival_construction_wall_sec=preamble_wall[0],clock_defaults_bound_to_same_advancing_replay_clock=clock_defaults,function_profile=function_profile,raw_fixture_sha256=sha(F/'full_arrival_frames/fixture.json'),script_sha256=sha(Path(__file__)),helper_sha256={str(p):sha(p)for p in [W/'.runtime/gen14_three_owner_remote_20261013.py',W/'.runtime/gen14_three_owner_task_fixture_20261013.py']},clock_model='Original source/arrival/capture stamps. Inert historical node initialization is outside replay. Only initial task arrival-event construction uses its exact recorded event stamp, with every nanosecond of that CPU charged before publish. Thereafter one continuously advancing perf_counter clock covers producer/remote/task CPU, original 20ms task waits, TF/SDK receive gates and event delivery. No arrival frame/ACK qualification is seeded.',transport_model=dict(request_delay_ns=request_delay,source_intertopic_receive_offsets_ns=offsets,not_network_end_to_end_latency_measurement=True),camera_pairs_available=len(frames),no_ros_master=True,motion_commands_sent=False)
 report['historical_reference_exact_original_status_seed']=dict(source_ns=PRE,source_status_file=str(F/'all_detector_statuses.json'),sha256=sha(F/'all_detector_statuses.json'),calculated_roundoff_residuals=historical_seed_residuals,current_frame_geometry_unchanged=True)
 report['qualified_handoff_only_passed']=report['passed']
 report['fusion']=fusion
 fusion['producer_remote_exact_contract_match']=bool(fusion['producer_registration_contract'] is not None and fusion['producer_registration_contract']==fusion['remote_registration_contract'])
 report['passed']=bool(report['passed'] and surface is not None and len(surface.view_stamps_ns)>=2 and registration is not None and registration.result.ok and remote._observe_return_phase=='observing_contact' and not remote._observe_return_paused and fusion['producer_remote_exact_contract_match'] and all(r['source_age_sec']<.8 for r in rows if r['state']=='ready'))
 report['parameter_reconstruction']=parameter_audit
 report['remote_input_journal']=remote_input_journal
 report['timer_phase']=dict(predeclared_values=[0,1,2],selected=args.timer_phase,period_ns=period_ns,offset_ns=timer_offset_ns,selection_independent_of_frame_results=True)
 report['helper_sha256'][str(W/'.runtime/gen14_remote_fusion_fixture_20261013.py')]=sha(W/'.runtime/gen14_remote_fusion_fixture_20261013.py')
 report['remote_input_clock_model']='Original receive times are lower bounds. Due camera and SDK callbacks execute at the next advancing-clock observation; callback CPU is charged and no clock is rewound to their historical receipt. Exact raw images and frame stamps are unchanged.'
 report['clock_monotonicity']=dict(return_count=clock_return_count[0],every_clock_return_asserted_nondecreasing=True,last_return_ns=last_clock_return[0],reentrant_callbacks_return_current_time=True)
 report['scope']='Actual producer confirmation, actual remote source join, dual ACK, actual task qualified completion/phase, actual reached-view fusion registration/information gain, and producer continuation. No grasp candidate planning or physical execution is claimed.'
 (O/'result.json').write_text(json.dumps(plain(report),ensure_ascii=False,indent=2)+'\n');print('RESULT',O/'result.json');assert report['passed'],error or 'Three-owner real replay failed; retained result'
if __name__=='__main__':main()
