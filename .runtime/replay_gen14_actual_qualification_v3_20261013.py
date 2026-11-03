#!/usr/bin/env python3
"""Offline actual producer callbacks/process with timed original pre-motion hold freeze.

Transport is replayed from saved receive clocks. Computation time always advances.
The original accepted pre-motion RGB reference is explicitly seeded; no arrival
success or station template is seeded. Default run retains original failure.
"""
import argparse,json,hashlib,importlib.util,threading,time,sys,inspect,functools
from copy import deepcopy
from dataclasses import fields
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock,patch
import cv2,numpy as np,rospy,tf2_py
from cv_bridge import CvBridge
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from alicia_flexible_grasp_supervisor.msg import NearFieldPlanningPhase,GraspState
from alicia_grasp_modes import tabletop as table
from alicia_grasp_modes.acquisition import InitialAcquisition
from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask

W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade');N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z');F=N/'verification/gen14_arrival_independent';PRE=1791860176945733070;ANCHOR=1791860167282787322

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def plain(x):
 if isinstance(x,np.ndarray):return x.tolist()
 if isinstance(x,np.generic):return x.item()
 if isinstance(x,dict):return {k:plain(v)for k,v in x.items()}
 if isinstance(x,(tuple,list)):return [plain(v)for v in x]
 return x

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--candidate',default=str(W));p.add_argument('--expect-qualified',action='store_true');p.add_argument('--profile-functions',action='store_true');args=p.parse_args();O=Path(args.output);assert F in O.parents;O.mkdir(exist_ok=False);C=Path(args.candidate)
 paths=[C/'src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py']+[C/'src/alicia_grasp_modes/src/alicia_grasp_modes'/name for name in('observation_appearance.py','observation_arrival_confirmation.py','arrival_fragment_group.py','station_compute_wait_producer.py','observation_measured_face.py','arrival_dense_identity.py')];hashes={str(p):sha(p)for p in paths}
 spec=importlib.util.spec_from_file_location('actual_unknown_gen14',paths[0]);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
 f=json.loads((F/'fixture.json').read_text());full=json.loads((F/'full_arrival_frames/fixture.json').read_text());camera=f['camera'];cfg=f['unknown_parameters'];config=table.Config(**{k:v for k,v in cfg.items()if k in {x.name for x in fields(table.Config)}});K=[camera[k]for k in('fx','fy','cx','cy')];bridge=CvBridge();cv2.setNumThreads(1)
 executing=next(r for r in f['wire']if r['topic'].endswith('measurement_request')and r['message']['state']=='executing');requal=next(r for r in f['wire']if r['topic'].endswith('measurement_request')and r['message']['state']=='requalifying');selection=executing['message']['selection'];epoch=executing['message']['control_epoch_ns']
 context=json.loads((N/'readonly_full_frame_probe_002/context.json').read_text());params=deepcopy(context['parameters']);params.update({'/camera':camera,'/unknown_perception':cfg,'/grasp_mode/selection':selection,'/gui/joint_direct_mode':False});deployment=json.loads((N/'full_restart_acceptance/result.json').read_text());assert hashlib.sha256(params['/robot_description'].encode()).hexdigest()==deployment['robot_description_sha256']
 def array(ns):return np.load(F/f'{ns}_source.npz')
 statuses=json.loads((F/'all_detector_statuses.json').read_text());pre_status=next(row['message']for row in statuses if row['message'].get('source_stamp_ns')==PRE and row['message'].get('state')=='ready');pre_timing=pre_status['frame_timing'];pre_hold_component=pre_timing['rgb_components']['hold.total'][0];pre_hold_start_ns=pre_timing['processing_start_ros_ns']+round(pre_hold_component['start_elapsed_sec']*1e9)
 for row in f['frames']:
  assert sha(Path(row['file']))==row['sha256'],row['file']
 for row in full['frames']:
  assert sha(Path(row['file']))==row['sha256'],row['file']
 d=array(ANCHOR);seg=table.segment_with_support_fallback(d['depth'],K,d['T_base_camera'],config);tracker=table.TargetTracker(config);anchor,_=tracker.choose(seg,d['depth'].shape);plane=seg.plane_base.copy();d=array(PRE);seg=table.segment_with_support_fallback(d['depth'],K,d['T_base_camera'],config,anchor,anchor,plane);matches=table._matching_candidates(seg.candidates,anchor,anchor,config);assert len(matches)==1;last=matches[0][1]
 now=[pre_hold_start_ns];running=[False];start=[now[0]];perf=[0];tf_index=[0];feed_images=[False];current_image=[-1];frames=full['frames'];samples=full['sdk'];dynamic=full['dynamic_tf'];image_pairs={}
 def stamp(ns):return rospy.Time(ns//10**9,ns%10**9)
 def make_pair(data,info):
  rgb=bridge.cv2_to_imgmsg(data['rgb'],encoding='bgr8');depth=bridge.cv2_to_imgmsg(data['raw_depth'],encoding=info.get('depth_encoding','16UC1'))
  for msg,key in ((rgb,'rgb_header'),(depth,'depth_header')):
   h=info[key];msg.header.seq=h['seq'];msg.header.stamp=stamp(h['stamp']);msg.header.frame_id=h['frame_id']
  return rgb,depth
 preinfo=next(x for x in f['frames']if x['source_stamp_ns']==PRE);presrc=make_pair(d,preinfo)
 for row in frames:image_pairs[row['source_ns']]=make_pair(np.load(row['file']),row)
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
  return at
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
 calls=[];rows=[];handoff=None;fixed_template=None;post_qualified=[];frozen_face=None
 with patch.object(module.rospy,'get_param',lambda key,default=None:params.get(key,default)),patch.object(module.rospy,'get_param_cached',lambda key,default=None:params.get(key,default)),patch.object(module.rospy,'get_master',lambda:SimpleNamespace(getParam=lambda key:(1,'sealed offline snapshot',params.get(key)))),patch.object(module.rospy.Time,'now',staticmethod(lambda:stamp(clock()))),patch.object(module.time,'monotonic',lambda:clock()*1e-9),patch.object(module.rospy,'logwarn_throttle',Mock()),patch.object(module.rospy,'logwarn',Mock()),patch.object(module.rospy,'loginfo',Mock()):
  begin(pre_hold_start_ns);hold=node._prepare_locked_rgb_hold_reference(presrc,SimpleNamespace(mask=d['recorded_mask']),camera,source_transform=d['T_base_camera']);hold_wall=end();assert hold is not None;hold_timing=dict(original_recorded_hold_start_ns=pre_hold_start_ns,modeled_start_ns=start[0],completed_ns=now[0],wall_sec=hold_wall,source_age_sec=(now[0]-PRE)*1e-9,original_recorded_hold_wall_sec=pre_hold_component['wall_sec'],new_computation_charged=True,reference_geometry_and_pixels_seeded_from_original_accepted_source=True,source_transform_from_original_exact_source_fixture=True,source_transform_sha256=hashlib.sha256(np.ascontiguousarray(d['T_base_camera']).tobytes()).hexdigest());assert now[0]-PRE<800000000;assert hold['refined_mask_sha256']=='eaa8cd24002b0e3511b971db2c0fd531ab00fccee61f502edde8637051dbb5b9';node._locked_rgb_hold_reference=hold
  for request in (executing,requal):
   begin(request['received_ns']);node._observation_measurement_request_cb(String(data=json.dumps(request['message'])));elapsed=end();ack=json.loads(node._observation_measurement_ack_pub.publish.call_args[0][0].data);assert ack['accepted'],ack;calls.append(dict(request=request['message']['state'],started_ns=start[0],completed_ns=now[0],wall_sec=elapsed,ack=ack))
   if request['message']['state']=='executing':
    face=node._observation_measurement['confirmation'].appearance_reference.measured_face
    if face is not None:frozen_face=dict(source_ns=face.source.source_ns,source_binding_sha256=face.source.binding_sha256,dense_reference_digest=face.reference.digest,audit=json.loads(face.audit_json))
  assert node._observation_measurement['confirmation'].source_stamps_ns==[]
  feed_images[0]=True;period=round(1e9/float(cfg.get('detect_hz',5.)));tick=max(now[0],frames[0]['pair_available_ns']);seen=set()
  while tick<full['capture_deadline_ns']+400000000:
   begin(tick);source=node.latest
   if source is None or source[0].header.stamp.to_nsec()in seen:
    end();tick=max(now[0],tick+period);continue
   ns=source[0].header.stamp.to_nsec();seen.add(ns);node.pubs={key:Mock()for key in node.pubs};node.process(None);elapsed=end();pub=node.pubs['detector_status'].publish
   status=json.loads(pub.call_args[0][0].data)if pub.call_args else None;o=(status or{}).get('observation_measurement',{})
   row=dict(source_ns=ns,processing_start_ns=start[0],completion_ns=now[0],wall_sec=elapsed,source_age_sec=(now[0]-ns)*1e-9,state=None if status is None else status['state'],detail=None if status is None else status['detail'],measurement_reason=o.get('reason'),accepted_sources=o.get('source_stamps_ns'),measurement_qualified=o.get('qualified'),status=status,last_good_ns=node.last_good_source_stamp_ns,first_loss=node.first_loss)
   rows.append(row);print(json.dumps({k:v for k,v in row.items()if k not in('status','first_loss')}),flush=True)
   qualified_stamps=(node._observation_measurement or{}).get('qualified_stamps')
   if qualified_stamps and args.expect_qualified and handoff is None:
    assert len(qualified_stamps)==3 and len(set(qualified_stamps))==3 and all(x>full['arrival_ns']for x in qualified_stamps)
    assert now[0]<full['capture_deadline_ns'] and now[0]-qualified_stamps[-1]<800000000
    request=deepcopy(requal['message']);request.update(state='qualified',stamp_ns=now[0]+1,request_id=request['lease_id']+':qualified',source_stamps_ns=list(qualified_stamps))
    begin(now[0]+1);node._observation_measurement_request_cb(String(data=json.dumps(request)));callback_elapsed=end();ack=json.loads(node._observation_measurement_ack_pub.publish.call_args[0][0].data);assert ack['accepted'],ack
    marker=node._locked_rgb_hold_reference['observation_arrival_retry'];fixed_template=marker['station_appearance'];assert fixed_template.source_ns==qualified_stamps[-1]
    phase_ns=now[0]+1;phase=NearFieldPlanningPhase(active=True,phase_id=1,planning_stage='contact_replan',reference_target_track_id=request['target_track_id']);phase.header.stamp=stamp(phase_ns);phase.reference_source_stamp=stamp(qualified_stamps[-1]);phase.deadline=stamp(phase_ns+60000000000);begin(phase_ns);node._station_compute_phase_cb(phase);node._station_compute_state_cb(GraspState(state='PLAN_PREGRASP',active=True));phase_callback_elapsed=end()
    handoff=dict(qualified_sources=list(qualified_stamps),qualified_callback_request=request,ack=ack,callback_wall_sec=callback_elapsed,phase_ns=phase_ns,phase_deadline_ns=phase.deadline.to_nsec(),phase_callback_wall_sec=phase_callback_elapsed,template_source_ns=fixed_template.source_ns,template_rgb_sha256=fixed_template.rgb_sha256,template_mask_sha256=fixed_template.mask_sha256,phase_and_task_qualified_request_counterfactual=True,remote_participant_fusion_not_replayed=True)
   elif handoff is not None:
    current=(node._locked_rgb_hold_reference or{}).get('observation_arrival_retry',{}).get('station_appearance');post_qualified.append(dict(source_ns=ns,state=row['state'],detail=row['detail'],source_age_sec=row['source_age_sec'],last_good_ns=node.last_good_source_stamp_ns,first_loss=node.first_loss,template_identity_unchanged=current is fixed_template,template_source_ns=None if current is None else current.source_ns,registered=getattr(node,'_station_compute_registration',None)is not None))
   elif qualified_stamps:break
   if node.first_loss is not None:break
   tick=max(now[0],start[0]+period)
 loaded={name:dict(path=str(Path(value.__file__).resolve()),sha256=sha(Path(value.__file__)))for name,value in sys.modules.items()if name.startswith('alicia_grasp_modes.')and getattr(value,'__file__',None)and Path(value.__file__).suffix=='.py'}
 expected=(handoff is not None and len(handoff['qualified_sources'])==3 and len(post_qualified)>=1 and any(r['state']=='ready'for r in post_qualified)and all(r['first_loss']is None and r['template_identity_unchanged']for r in post_qualified))if args.expect_qualified else(not any(r['measurement_qualified']for r in rows)and sum(r['measurement_reason']=='arrival_fragment_group_original_rgb_identity_required'for r in rows)>=3 and all(r['last_good_ns']==PRE for r in rows))
 report=dict(passed=all(sha(Path(k))==v for k,v in hashes.items())and expected,expected_qualified=args.expect_qualified,qualified_handoff=handoff,post_qualified_process=post_qualified,loaded_dependency_sha256=loaded,scope='Producer qualification/process replay: real executing/requalifying callbacks and empty arrival confirmation; complete saved camera interval; any successful qualified callback/task phase is an explicit conditional counterfactual after three actually measured accepted frames, not seeded success',rows=rows,callbacks=calls,source_sha256=hashes,source_unchanged=all(sha(Path(k))==v for k,v in hashes.items()),premotion_preparation_timing=hold_timing,premotion_accepted_reference_seed=dict(source_ns=PRE,source_rgb_sha256=hold['source_rgb_sha256'],source_depth_sha256=hold['source_depth_sha256'],mask_sha256=hold['refined_mask_sha256'],actual_recorded_pixels_and_mask=True,not_arrival_qualification=True),qualified_state_seeded=False,source_restamped=False,clock_model='Original source and bag arrival times; PRE hold creation starts at recorded hold.total entry and its actual time is charged; subsequent callback begins max(original receive, prior completion), never backwards; arrival processing starts at first post-arrival recorded pair availability; 6Hz timer phase explicitly modeled because original processing-start for measurement branch was not recorded; actual computation and TF/SDK waits charged via perf_counter; latest available real pair replaces older pending pairs.',camera_pairs_available=len(frames),received_tf_and_sdk_time_gated=True,actual_cpu_clock_advances=True,raw_fixture=str(F/'full_arrival_frames/fixture.json'),raw_fixture_sha256=sha(F/'full_arrival_frames/fixture.json'),script_sha256=sha(Path(__file__)),no_ros_master=True,motion_commands_sent=False)
 report.update(clock_defaults_bound_to_same_advancing_replay_clock=clock_defaults,frozen_premotion_face=frozen_face,function_profile=function_profile,profile_overhead_charged=True if args.profile_functions else None)
 (O/'result.json').write_text(json.dumps(plain(report),ensure_ascii=False,indent=2)+'\n');print('RESULT',O/'result.json');assert report['passed'], 'Actual replay did not meet its declared expected outcome; see preserved result.json'
if __name__=='__main__':main()
