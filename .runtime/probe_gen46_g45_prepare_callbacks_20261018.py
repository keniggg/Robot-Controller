"""Bounded offline G45 real prepare/callback probe; no historical CAS claim.

Only the three requested raw RGB-D sources are decoded, with original masks,
SDK, TF and camera metadata. A measured first-source surface is an explicitly
counterfactual preappend provider. Stop after the original lifecycle check at
base line 9498; never accept a prediction, contact hardware or call WSL.
"""
import ast, base64, copy, hashlib, importlib.util, json, os, platform, signal
import socket, struct, sys, threading, time, traceback
from pathlib import Path
from types import SimpleNamespace as NS

W=Path(__file__).resolve().parents[1]; B=W/'.runtime/gen46_baseline_source'
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
V=E/'ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
G=E/'grasp_records/20261018T070056.706765Z_9d704cdd7a93474ea90deecc79af09ce'
O=Path(sys.argv[1]); SOURCE3=(1792306911033774375,1792306911564159393,1792306911830121517)
PROFILE_ONLY='--profile-tabletop' in sys.argv[2:]
REAL_MONO=time.monotonic; REAL_PERF=time.perf_counter; START=REAL_PERF()
attempts=[]; trace=[]; inputs={}; result={'kind':'bounded_G45_prepare_and_original_callbacks_counterfactual_probe',
 'pipeline_success':False,'historical_geometry_exact':False,'historical_callback_schedule_exact':False,
 'physical_grasp_verified':False,'ROS_initialized':False,'hardware_contacted':False,'temperature_read':False,
 'production_or_record_modified':False,'pixel_files_exported':False,'errors':[]}

class Boundary(BaseException):pass
class StageStop(BaseException):pass
class BudgetStop(BaseException):pass
def forbidden(*a,**k):attempts.append(repr(a)[:300]);raise Boundary('forbidden offline boundary')
def sh(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def put(name,value):
 p=O/name; data=(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False,default=str)+'\n').encode()
 p.write_bytes(data);assert p.read_bytes()==data;return sh(p)
def event(tag,**facts):trace.append(dict(tag=tag,elapsed_sec=REAL_PERF()-START,thread=threading.current_thread().name,**facts))
def alarm(*a):raise BudgetStop('55s total probe budget reached')

def main():
 assert Path('/media/zhuyupei/SanStand').is_mount() and V in O.parents and not O.exists()
 O.mkdir();os.environ['ROS_LOG_DIR']=str(O/'ros');os.environ['TMPDIR']=str(O/'tmp');(O/'tmp').mkdir()
 sys.dont_write_bytecode=True;signal.signal(signal.SIGALRM,alarm);signal.alarm(55)
 platform.architecture=lambda *a,**k:(str(struct.calcsize('P')*8)+'bit','ELF')
 platform._syscmd_uname=lambda *a,**k:'';socket.has_ipv6=False
 for name in ('connect','connect_ex','sendto','bind','listen'):setattr(socket.socket,name,forbidden)
 socket.create_connection=socket.getaddrinfo=forbidden
 def audit(ev,args):
  if ev in ('subprocess.Popen','os.system','os.posix_spawn','os.exec'):forbidden(ev)
  if ev=='open' and args and isinstance(args[0],(str,bytes,os.PathLike)):
   p=os.path.realpath(os.fsdecode(args[0]))
   if p.startswith('/dev/') and p not in ('/dev/null','/dev/zero','/dev/random','/dev/urandom') and not p.startswith(('/dev/shm/','/dev/fd/','/dev/pts/')):forbidden('device',p)
 sys.addaudithook(audit)
 sys.path[:0]=[str(B/'src/alicia_grasp_modes/tests'),str(B/'src/alicia_flexible_grasp_supervisor/tests'),str(B/'src/alicia_grasp_modes/src'),str(B/'src/alicia_flexible_grasp_supervisor/src'),str(W/'devel/lib/python3/dist-packages'),'/opt/ros/noetic/lib/python3/dist-packages',str(W/'.runtime')]
 import numpy as np, cv2, yaml, rospy, rosbag
 from _pytest.monkeypatch import MonkeyPatch
 mp=MonkeyPatch()
 for name in ('init_node','Publisher','Subscriber','Service','ServiceProxy','wait_for_service','set_param','delete_param'):mp.setattr(rospy,name,forbidden)
 import gen46_g45_callback_harness_provider_20261018 as provider
 from test_remote_modes import base,module
 from alicia_flexible_grasp.vision.rgbd_snapshot import RgbdSample, fuse_stable_samples
 from alicia_flexible_grasp.vision.multiview_surface import surface_view_from_observation,fused_surface_from_view
 from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
 import tf2_py
 cv2.setNumThreads(1)
 data=provider.load_nonimage_inputs();manifest=json.loads((B/'manifest.json').read_text())
 for rel,h in manifest['source_sha256'].items():assert sh(B/rel)==h
 result['baseline_manifest_sha256']=sh(B/'manifest.json');result['baseline_source_count']=len(manifest['source_sha256'])
 assert len(manifest['source_sha256'])==1044
 bagpath=G/'keyframes.bag';manifest_path=G/'manifest.json';record=json.loads(manifest_path.read_text())
 assert sh(manifest_path)=='1e41d1d431aa70a6de590117e2910e7c7b2fd37eef3f7a80fdf630c831395e0a'
 before=bagpath.stat(); file_identity=lambda st:(st.st_dev,st.st_ino,st.st_size,st.st_mtime_ns)
 result['bag_identity_before']=file_identity(before);result['bag_sha256_manifest_only']=next(x['sha256']for x in record['files']if x['path']=='keyframes.bag')
 masks={}; images={}; camera_rows=[]; error_rows=[]; core=tf2_py.BufferCore(rospy.Duration(600));tf_count=0
 wanted={x['parsed_json']['source_stamp_ns']for x in data.statuses}
 whitelist=['/perception/object_mask','/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw','/tf','/tf_static','/grasp_record/camera_runtime_metadata','/experiment/runtime_execution_error_snapshot']
 with rosbag.Bag(str(bagpath),'r') as bag:
  for topic,msg,stamp in bag.read_messages(topics=whitelist):
   if topic in ('/tf','/tf_static'):
    for transform in msg.transforms:
     (core.set_transform_static if topic=='/tf_static' else core.set_transform)(transform,'sealed_bag');tf_count+=1
   elif topic=='/grasp_record/camera_runtime_metadata':camera_rows.append((stamp.to_nsec(),json.loads(msg.data)))
   elif topic=='/experiment/runtime_execution_error_snapshot':error_rows.append((stamp.to_nsec(),json.loads(msg.data)))
   else:
    ns=msg.header.stamp.to_nsec()
    if topic=='/perception/object_mask' and ns in wanted:masks[ns]=(stamp.to_nsec(),msg)
    if ns in SOURCE3:images[(ns,topic)]=msg
 assert len(masks)==33 and all((s,t)in images for s in SOURCE3 for t in whitelist[1:3]),'missing exact request15 RGB-D'
 camera_entries=[(s,c)for s,c in camera_rows if s<SOURCE3[0]and c.get('status')=='available']
 assert camera_entries,'no available original camera parameter metadata before request15'
 cam_stamp,cam_evidence=camera_entries[-1];camera=cam_evidence['parameters'];K=NS(**{k:camera[k]for k in ('width','height','fx','fy','cx','cy','depth_scale')})
 result['camera_parameter_association']={'bag_timestamp_ns':cam_stamp,'sha256':hashlib.sha256(json.dumps(camera,sort_keys=True).encode()).hexdigest(),'image_synchronized':False,'read_interval':cam_evidence.get('parameter_read_interval')}
 class TF:
  def lookup_transform(self,target,source,stamp,*a):return core.lookup_transform_core(target,source,stamp)
 tf=TF(); tf.lookup_transform('base_link',images[(SOURCE3[-1],whitelist[1])].header.frame_id,rospy.Time(*divmod(SOURCE3[-1],10**9)))
 result['TF_messages_set']=tf_count
 cfg_path=B/'src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml';cfg=yaml.safe_load(cfg_path.read_text());arch=json.loads((G/'perception_execution_configuration.json').read_text())['parameters']
 remote=copy.deepcopy(cfg['grasp_6d']['remote']);remote['multiview']=arch['/grasp_6d/remote/multiview']
 urdf=B/'deployment/2026-10-08/runtime/candidate_models/expanded_metric_20260929/alicia_expanded_metric_candidate.urdf'
 handeye_path=W/'.runtime/candidate_models/expanded_metric_20260929/handeye_candidate_config.yaml';handeye=yaml.safe_load(handeye_path.read_text())['handeye']
 errors=[v for s,v in error_rows if s<=SOURCE3[-1]and v];assert errors,'missing historical runtime error snapshot'
 params={'/grasp':arch['/grasp'],'/grasp_6d/remote':remote,'/camera':camera,'/handeye':handeye,'/gripper':cfg['gripper'],'/robot_description':urdf.read_text(),'/perception':cfg.get('perception',{}),'/mujoco_digital_twin':cfg.get('mujoco_digital_twin',{}),'/grasp_6d/runtime_execution_error':errors[-1],module.SELECTION_PARAM:data.binding['selection'],'/gui/joint_direct_mode':False}
 reads=[]
 def get_param(name,default=None):
  reads.append(name)
  if name in params:return copy.deepcopy(params[name])
  for root in sorted(params,key=len,reverse=True):
   if name.startswith(root+'/'):
    value=params[root]
    try:
     for key in name[len(root)+1:].split('/'):value=value[key]
     return copy.deepcopy(value)
    except(KeyError,TypeError):return default
  return default
 clock={'ns':1792306910000000000,'anchor':None};lock=threading.Lock()
 def now_ns():
  with lock:return clock['ns'] if clock['anchor']is None else clock['ns']+int((REAL_MONO()-clock['anchor'])*1e9)
 c=provider.initialize_remote(mp,data,now_ns);n=c.node
 mp.setattr(rospy,'get_param',get_param);mp.setattr(rospy,'get_param_cached',get_param)
 # The existing fixture master provider reads c.params. Preserve that real
 # local boundary adapter while binding the exact historical owner below.
 c.params[module.SELECTION_PARAM]=copy.deepcopy(data.binding['selection']);c.params['/gui/joint_direct_mode']=False
 n._near_field_phase_id=data.binding['phase_id'];n._near_field_planning_stage='contact_replan'
 n.pose_estimator=NS(tf_buffer=tf,camera_frame='camera_link',base_frame='base_link',tf_timeout_sec=.2)
 n.tf_buffer=tf;n._camera_intrinsics=lambda:K;n.grasp_config=copy.deepcopy(arch['/grasp']);n._last_model_choice='unknown_tabletop'
 n.gate_audit_output_path=str(O/'unused_planning_audit.json');n.mujoco_audit_output_path=str(O/'unused_mujoco_audit.json')
 params['/grasp_6d/remote/gate_audit_output_path']=n.gate_audit_output_path;params['/mujoco_digital_twin/audit_output_path']=n.mujoco_audit_output_path
 allowed={'gripper_geometry','opening_fit_clearance_each_side_m','camera_depth_scale','tabletop_geometry_enabled','tabletop_geometry_config','hybrid_merge_config','adaptive_stage_limits','runtime_execution_error_max_age_sec','gripper_tool_jaw_axis','gripper_tool_finger_length_axis','soft_score_weights','target_absolute_sanity_distance_m','grasp_config','handeye_parent_frame','handeye_camera_frame','handeye_translation_xyz','handeye_rotation_xyzw','handeye_allow_static_fallback','_cached_tool_from_camera','_cached_tool_from_camera_stamp_ns','gripper_physical_open_width_m','candidate_min_downward_approach_cos','candidate_max_final_approach_lateral_m','planning_mask_min_iou','planning_mask_max_centroid_shift_px','planning_max_joint_delta_rad','mask_erosion_px','mask_internal_hole_max_area_px','depth_mad_scale','depth_mad_absolute_floor_m','geometry_support_bbox_expand_ratio','geometry_support_distance_threshold_m','geometry_voxel_size_m','geometry_outlier_neighbors','geometry_outlier_std_ratio','geometry_min_support_points','geometry_min_object_points','geometry_min_size_m','geometry_max_size_m','geometry_max_height_m'}
 source=B/'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py';tree=ast.parse(source.read_text());init=next(f for cls in tree.body if isinstance(cls,ast.ClassDef)and cls.name=='RemoteGrasp6DNode'for f in cls.body if isinstance(f,ast.FunctionDef)and f.name=='__init__')
 stage=next(f for cls in tree.body if isinstance(cls,ast.ClassDef)and cls.name=='RemoteGrasp6DNode'for f in cls.body if isinstance(f,ast.FunctionDef)and f.name=='_stage_runtime_params')
 allowed.update(a.attr for a in ast.walk(stage)if isinstance(a,ast.Attribute)and isinstance(a.value,ast.Name)and a.value.id=='self'and isinstance(a.ctx,ast.Load)and not a.attr.startswith('_'))
 env=dict(vars(base),self=n,remote_cfg=remote,gcfg=arch['/grasp'],grasp_cfg=arch['/grasp'],cam_cfg=camera,hcfg=handeye,gripper_cfg=cfg['gripper'],twin_cfg=cfg.get('mujoco_digital_twin',{}),gripper_geometry_cfg=remote['gripper_geometry'],default_soft_weights=base.SoftScoreWeights(),soft_weight_cfg=remote.get('soft_score_weights',{}),startup_continuous_config=base.load_continuous_runtime_config(remote,get_param=get_param))
 configured=[]
 for statement in init.body:
  if not isinstance(statement,ast.Assign):continue
  names=[v.attr for target in statement.targets for v in ast.walk(target)if isinstance(v,ast.Attribute)and isinstance(v.value,ast.Name)and v.value.id=='self']
  if names and all(v in allowed for v in names):exec(compile(ast.Module(body=[statement],type_ignores=[]),str(source),'exec'),env);configured.extend(names)
 n.mask_internal_hole_max_area_px=0 # exact unknown-mode reset policy, no hole fabrication
 n._refresh_runtime_params();n._station_compute_retired=False
 n._near_field_phase_id=data.binding['phase_id'];n._near_field_phase_started_sec=data.phase.header.stamp.to_sec();n._near_field_phase_deadline_sec=data.phase.deadline.to_sec()
 n._stream_worker_busy=False;n.last_submitted_stamp_ns=0;n._direct_near_field_submission_generation=None
 identity=TargetTrackIdentity(4,'g1-t4');samples=[];pixels=[]
 for s in SOURCE3:
  color=images[(s,whitelist[1])];depth=images[(s,whitelist[2])];mask=masks[s][1];obj=data.objects[s][1]
  rgb=n.bridge.imgmsg_to_cv2(color,desired_encoding='bgr8');raw=n.bridge.imgmsg_to_cv2(depth,desired_encoding='passthrough');m=n.bridge.imgmsg_to_cv2(mask,desired_encoding='mono8')
  row,js=max(((r,j)for r,j in data.sdk if j.header.stamp.to_nsec()<=s),key=lambda v:v[1].header.stamp.to_nsec());assert s-js.header.stamp.to_nsec()<200000000
  q=dict(zip(js.name,js.position));samples.append(RgbdSample(rgb,raw,m,(obj.bbox_x,obj.bbox_y,obj.bbox_width,obj.bbox_height),obj,s*1e-9,color.header.frame_id,np.array([q['Joint'+str(i)]for i in range(1,7)]),s,4,identity))
  pixels.append({'source_ns':s,'rgb_shape':rgb.shape,'depth_shape':raw.shape,'mask_shape':m.shape,'rgb_sha256':hashlib.sha256(np.ascontiguousarray(rgb).tobytes()).hexdigest(),'depth_sha256':hashlib.sha256(np.ascontiguousarray(raw).tobytes()).hexdigest(),'normalized_mask_sha256':hashlib.sha256(np.ascontiguousarray(m!=0,dtype=np.uint8).tobytes()).hexdigest(),'SDK_header_ns':js.header.stamp.to_nsec()})
 ds,dmin,dmax=n._snapshot_depth_config()
 def fuse(items):return fuse_stable_samples(items,True,n.planning_mask_min_iou,n.planning_mask_max_centroid_shift_px,n.planning_max_joint_delta_rad,n.mask_erosion_px,ds,dmin,dmax,n.depth_mad_scale,n.depth_mad_absolute_floor_m,internal_hole_max_area_px=0)
 snapshot=fuse(samples);assert snapshot.ok,(snapshot.failure_code,snapshot.failure_reason);assert snapshot.sample_stamp_ns==SOURCE3
 # Counterfactual seed uses real FIRST request source and original geometry
 # estimator, not fabricated points/a historical CAS receipt.
 seed_snapshot=fuse(samples[:1]);seed=copy.copy(n);seed.near_field_planning_active=False
 seed._near_field_fused_surface=seed._near_field_reference_view=None
 geom,_,transform=base.RemoteGrasp6DNode._prepare_snapshot_geometry(seed,seed_snapshot,rospy.Time(*divmod(SOURCE3[0],10**9)))
 assert geom.ok,(geom.failure_code,geom.failure_reason)
 observation=base.observation_from_snapshot(seed_snapshot,geom,transform,'base_link');view=surface_view_from_observation(observation)
 n._near_field_reference_view=view;n._near_field_fused_surface=fused_surface_from_view(view,voxel_size_m=n.geometry_voxel_size_m)
 n._near_field_surface_generation=7;n._near_field_surface_phase_id=n._near_field_phase_id
 n._latest_target_observation=observation;n._latest_raw_geometry_source=None;n._latest_registration_evidence=None;n._latest_box_registration_choice=None;n._latest_fused_scene_audit=None
 result['preappend_provider']='Counterfactual same-station first request15 raw sample through original estimator -> measured single-view surface; not historical post-observation private CAS.'
 result['parameter_provider']='GEN45 sealed startup YAML + archived /grasp/multiview + recorded camera/runtime_error; explicit fixture initialization; no full live remote dump; unused audit outputs redirected to probe dir.'
 result['source3_pixel_descriptors']=pixels;result['configured_original_assignments']=configured
 # Trace without replacing Condition or geometry lock. Wrappers call every
 # original gate and preserve all returns/exceptions.
 def state():
  ref=n._station_compute_reference;return dict(now_ns=now_ns(),ref=None if ref is None else ref['contract']['source_ns'],retired=n._station_compute_retired,pending=n._station_compute_pending,work_ids=list(n._station_compute_work))
 for name in ('_station_private_pending_input','_station_private_pending_reconcile','_station_compute_join_locked','_station_compute_context','_prepare_snapshot_geometry','_prepare_regular_box_snapshot','_generate_tabletop_candidates'):
  actual=getattr(n,name)
  def wrapped(*a,_f=actual,_name=name,**k):
   start=REAL_PERF();event('method_entry',method=_name,**state())
   try:
    if PROFILE_ONLY and _name=='_generate_tabletop_candidates':
     # Persist measured geometry/configuration only, never original image
     # arrays. Descriptor contains explicit type names; arrays remain binary.
     import cProfile,pstats,dataclasses
     arrays={}
     def freeze(v):
      if isinstance(v,np.ndarray):
       key='array_'+str(len(arrays));arrays[key]=v;return dict(npz_key=key,dtype=str(v.dtype),shape=list(v.shape))
      if dataclasses.is_dataclass(v):return dict(type=type(v).__module__+'.'+type(v).__name__,fields={f.name:freeze(getattr(v,f.name))for f in dataclasses.fields(v)})
      if hasattr(v,'items'):return {str(key):freeze(value)for key,value in v.items()}
      if isinstance(v,(tuple,list)):return [freeze(x)for x in v]
      if isinstance(v,np.generic):return v.item()
      if v is None or isinstance(v,(str,int,float,bool)):return v
      return dict(type=type(v).__module__+'.'+type(v).__name__,description='non-array diagnostic type')
     names=['_near_field_fused_surface','_near_field_reference_view','_latest_target_observation','_latest_registration_evidence','_latest_fused_scene_audit','_regular_box_snapshot','tabletop_geometry_config','gripper_geometry','adaptive_stage_limits','opening_fit_clearance_each_side_m','gripper_tool_jaw_axis','gripper_tool_finger_length_axis']
     frozen=dict(geometry=freeze(a[0]),node_inputs={key:freeze(getattr(n,key,None))for key in names},snapshot_quality=freeze(snapshot.quality),snapshot_source_stamps_ns=list(snapshot.sample_stamp_ns),parameters=freeze(params),counterfactual_preappend=result['preappend_provider'],clock_provider='Historical ROS clock frozen at pure-profile entry; real monotonic remains unchanged. No post-profile lifecycle acceptance is claimed.',function='original ModeAware _generate_tabletop_candidates -> frozen GEN45 base',source_sha256=sh(source))
     np.savez_compressed(O/'frozen_geometry_arrays.npz',**arrays);put('frozen_geometry_inputs.json',frozen)
     profiler=cProfile.Profile();out=profiler.runcall(_f,*a,**k);profiler.dump_stats(str(O/'tabletop.prof'))
     with (O/'profile_top.txt').open('w')as stream:pstats.Stats(profiler,stream=stream).strip_dirs().sort_stats('cumulative').print_stats(50)
     result['tabletop_profile']={'wall_sec':REAL_PERF()-start,'candidate_count':len(out[0]),'pstats_sha256':sh(O/'tabletop.prof'),'frozen_arrays_sha256':sh(O/'frozen_geometry_arrays.npz'),'frozen_inputs_sha256':sh(O/'frozen_geometry_inputs.json'),'new_offline_measurement_not_G45_historical':True}
     raise StageStop('PROFILE_TABLETOP_COMPLETED; no lifecycle9498 acceptance or pipeline success')
    return _f(*a,**k)
   finally:event('method_exit',method=_name,wall_sec=REAL_PERF()-start,**state())
  setattr(n,name,wrapped)
 original_check=n._require_stream_ticket_current
 def check(ticket):
  caller=sys._getframe(1);event('ticket_check_entry',caller=caller.f_code.co_name,line=caller.f_lineno,**state())
  if caller.f_code.co_name=='_prepare_and_predict'and caller.f_lineno==9498:result['entered_original_lifecycle9498']=True
  original_check(ticket)
  event('ticket_check_pass',caller=caller.f_code.co_name,line=caller.f_lineno,**state())
  if caller.f_code.co_name=='_prepare_and_predict'and caller.f_lineno==9498:raise StageStop('original lifecycle9498 passed; intentional offline stop')
 n._require_stream_ticket_current=check
 original_retire=n._station_compute_retire
 def retire(reason):event('retire_entry',reason=str(reason),**state());return original_retire(reason)
 n._station_compute_retire=retire
 events=provider.callback_events(data,masks);trigger=max(x[0]for x in events if x[3]==SOURCE3[-1]and x[2]in('status','object','mask'))+1000000
 result['worker_trigger_provider_ns']=trigger;result['worker_trigger_is_historical_entry']=False
 done=threading.Event();worker=None
 def work(ticket):
  event('worker_entry',**state())
  try:n._prepare_and_predict(ticket);result['unexpected_prepare_return']=True
  except StageStop as e:
   result['reached_original_lifecycle9498']=not PROFILE_ONLY;result['worker_stop']=str(e)
  except BaseException as e:result['worker_exception']={'type':type(e).__name__,'message':str(e),'traceback':traceback.format_exc()}
  finally:event('worker_exit',**state());done.set()
 with lock:clock['anchor']=REAL_MONO()
 for stamp,_,kind,s,value in events:
  if stamp<clock['ns']:continue
  if stamp>1792306920600000000:break
  target=(stamp-clock['ns'])*1e-9
  while REAL_MONO()-clock['anchor']<target:time.sleep(max(0.,min(.005,target-(REAL_MONO()-clock['anchor']))))
  event('callback_entry',kind=kind,source_ns=s,scheduled_ns=stamp,**state());provider.deliver_original_callback(n,kind,value);event('callback_return',kind=kind,source_ns=s,scheduled_ns=stamp,**state())
  if worker is None and stamp>=trigger:
   config=n._freeze_graspnet_input_config();assert n.submit_stream_snapshot(snapshot,graspnet_input_config=config,expected_generation=7,require_idle_worker=True),'original submit rejected'
   ticket=n._stream_worker_ticket;assert ticket is not None and n._station_compute_work_certificate(ticket),'missing original submitted work certificate'
   result['replay_ticket_request_id']=ticket.request_id;result['historical_request_id']=15
   with n._stream_condition:
    ticket,_=n.inference_coordinator.claim_for_preparation(ticket);n._stream_worker_ticket=None;n._stream_worker_busy=True
   if PROFILE_ONLY:
    with lock:
     clock['ns']+=int((REAL_MONO()-clock['anchor'])*1e9)
     clock['anchor']=None
   worker=threading.Thread(target=work,args=(ticket,),name='real-prepare-worker',daemon=True);worker.start()
   if PROFILE_ONLY:break
 assert worker is not None,'no worker was submitted'
 done.wait(max(0.,50-(REAL_PERF()-START)));result['worker_finished']=done.is_set();result['final_state']=state()
 result['callbacks_processed']=sum(x['tag']=='callback_return'for x in trace)
 result['bag_identity_after']=file_identity(bagpath.stat());assert result['bag_identity_after']==result['bag_identity_before']
 result['readiness_complete']=True;result['profile_only']=PROFILE_ONLY;result['elapsed_sec']=REAL_PERF()-START;result['boundary_attempts']=attempts
 result['module_provenance']={name:{'path':str(Path(m.__file__).resolve()),'sha256':sh(m.__file__)}for name,m in sys.modules.copy().items()if getattr(m,'__file__',None)and str(B)in str(m.__file__)and Path(m.__file__).is_file()}
 result['inputs']={str(p):sh(p)for p in [manifest_path,cfg_path,urdf,handeye_path,W/'.runtime/gen46_g45_callback_harness_provider_20261018.py']}
 # Do not restore global monkeypatches while a daemon is still calculating.
 if done.is_set():mp.undo()

if __name__=='__main__':
 try:main()
 except BaseException as e:result['errors'].append({'type':type(e).__name__,'message':str(e),'traceback':traceback.format_exc()})
 finally:
  signal.alarm(0);result['elapsed_sec']=REAL_PERF()-START;result['boundary_attempts']=attempts;result['helper_sha256']=sh(__file__)
  if O.exists():
   result['trace_sha256']=put('trace.json',trace);put('result.json',result)
   put('receipt.json',{'artifacts':{name:sh(O/name)for name in ['trace.json','result.json']},'pipeline_success':False,'source_modified':False})
   print(json.dumps({'result':str(O/'result.json'),'sha256':sh(O/'result.json'),'errors':result['errors'],'worker_exception':result.get('worker_exception'),'reached9498':result.get('reached_original_lifecycle9498')},ensure_ascii=False))
