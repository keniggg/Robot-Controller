"""Original pure proposal stage from qualified G47 box and measured surface.
Frozen GEN47 YAML plus archived G47 overrides; no complete remote parameter dump
is claimed. This does not recreate prepare/lifecycle acceptance or a robot plan.
"""
import ast,hashlib,json,os,sys,time,traceback,gzip,math,platform,struct
from pathlib import Path
from types import SimpleNamespace
from dataclasses import asdict
W=Path(__file__).resolve().parents[1];C=W/'.runtime/gen48_visibility_candidate';B=W/'.runtime/gen48_baseline_source'
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller');V=E/'ros_log/full_latest_gen41_restart_20261018T011222Z/verification';G=E/'grasp_records/20261018T085552.871836Z_7d7d31b2570b433b8ee05398edc47d73'
OUT=V/'gen48_g47_candidate_tabletop_r2';Q=V/'gen48_g47_candidate_qualification_r1';BOUNDARY=[]
def sha(path):
 h=hashlib.sha256()
 with Path(path).open('rb')as f:
  for block in iter(lambda:f.read(4*1024*1024),b''):h.update(block)
 return h.hexdigest()

def put(path,value):
 raw=(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n').encode()
 with path.open('xb')as f:f.write(raw)
 assert path.read_bytes()==raw
 return dict(path=str(path),sha256=sha(path))

def deny(kind,detail):
 BOUNDARY.append(dict(kind=kind,detail=str(detail)));raise RuntimeError('offline boundary: '+kind)

def audit_event(event,args):
 if event.startswith('socket.'):deny(event,args[:1])
 if event in ('subprocess.Popen','os.system','os.exec','os.posix_spawn','os.fork','os.forkpty'):deny(event,'process creation')
 if event=='open':
  path=args[0]
  if isinstance(path,(str,bytes,os.PathLike)):
   p=os.fsdecode(path)
   if p.startswith('/dev/') and p not in ('/dev/null','/dev/urandom','/dev/random'):deny('device_open',p)
   if p.startswith('/sys/class/thermal')or '/hwmon/'in p:deny('temperature_file',p)
   flags=args[2]if len(args)>2 and isinstance(args[2],int)else 0
   mode=args[1]if len(args)>1 and isinstance(args[1],str)else ''
   if p!='/dev/null' and (flags & (os.O_WRONLY|os.O_RDWR|os.O_CREAT|os.O_TRUNC|os.O_APPEND) or any(x in mode for x in 'wax+')):
    try:Path(p).resolve().relative_to(OUT.resolve())
    except ValueError:deny('write_outside_probe',p)

def plain(v):
 if hasattr(v,'items'):return {str(k):plain(x) for k,x in v.items()}
 if isinstance(v,(tuple,list)):return [plain(x) for x in v]
 if hasattr(v,'tolist'):return v.tolist()
 return v

def materialize_original(ctx):
 import importlib.util,threading,io
 from copy import deepcopy
 from dataclasses import replace
 from types import ModuleType
 import numpy as np
 result=ctx['result'];pin=ctx['pin'];report=ctx['report'];geometry=ctx['geometry'];planning=ctx['planning'];enclosure=ctx['enclosure'];q=ctx['q'];model=ctx['model'];archived=ctx['archived'];config=ctx['config'];remote_cfg=ctx['remote_cfg'];oldnode=ctx['node'];pins=ctx['pins']
 sys.path[:0]=[str(C/'src/alicia_grasp_modes/src'),str(W/'devel/lib/python3/dist-packages'),'/opt/ros/noetic/lib/python3/dist-packages']
 import rospy
 result['ROS_imported']=True
 for name in ('init_node','Publisher','Subscriber','Service','ServiceProxy','wait_for_service','set_param','delete_param','get_master'):setattr(rospy,name,lambda *a,_name=name,**k:deny('ROS_'+_name,'forbidden'))
 for name in ('loginfo','logwarn','logerr','logdebug','loginfo_throttle','logwarn_throttle','logerr_throttle','logdebug_throttle'):setattr(rospy,name,lambda *a,**k:None)
 stamp=report['snapshot_stamp_ns'];rospy.Time.now=staticmethod(lambda:rospy.Time(*divmod(stamp,10**9)))
 error_input=pin(V/'gen48_g47_request16_nonimage_inputs_r1/result.json','cae4514adb55870c5b5c7bdc76455a8b7c2989aa34efe129d616a136725ef18f')
 measured=error_input['nearest_sdk_at_or_before_snapshot'];error=error_input['latest_error_snapshot_received_at_or_before_source']['data'];assert measured['frame_id']=='sdk_measured' and 0<stamp-measured['header_stamp_ns']<500_000_000
 assert error['position_error_m']==report['remote_diagnostics']['tabletop_geometry']['adaptive_stage_profiles'][0]['execution_position_error_m']
 camera_meta=pin(V/'gen48_g47_exact_box_visibility_probe_r1/attempt_r6/camera_metadata.json','c35f528c163c618887604f84aa0fe36d12a816b41d5a17e8825022d1d13048bc')
 urdf=C/'deployment/2026-10-08/runtime/candidate_models/expanded_metric_20260929/alicia_expanded_metric_candidate.urdf';pin(urdf)
 params={'/grasp':archived['parameters']['/grasp'],'/grasp_6d/remote':remote_cfg,'/grasp_6d/runtime_execution_error':error,'/camera':camera_meta['intrinsics'],'/gripper':config['gripper'],'/robot_description':urdf.read_text(),'/perception':config.get('perception',{})};reads=[]
 def get_param(name,default=None):
  reads.append(name)
  if name in params:return deepcopy(params[name])
  for root in sorted(params,key=len,reverse=True):
   if name.startswith(root+'/'):
    v=params[root]
    try:
     for part in name[len(root)+1:].split('/'):v=v[part]
     return deepcopy(v)
    except(KeyError,TypeError):return deepcopy(default)
  raise ValueError('unprovided historical parameter '+name)
 rospy.get_param=rospy.get_param_cached=get_param
 # Load original source by its canonical ModeAware dependency name, avoiding
 # any rospkg fallback to installed/live workspace source.
 basepath=C/'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py';name='_alicia_original_remote_grasp6d';spec=importlib.util.spec_from_file_location(name,basepath);remote=importlib.util.module_from_spec(spec);sys.modules[name]=remote;spec.loader.exec_module(remote)
 modepath=C/'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py';spec=importlib.util.spec_from_file_location('gen48_diagnostic_ModeAware',modepath);mode=importlib.util.module_from_spec(spec);sys.modules[spec.name]=mode;spec.loader.exec_module(mode)
 node=mode.ModeAwareRemoteGrasp6DNode.__new__(mode.ModeAwareRemoteGrasp6DNode);node.__dict__.update(oldnode.__dict__)
 names={'adaptive_stage_limits','runtime_execution_error_max_age_sec','gripper_tool_jaw_axis','gripper_tool_finger_length_axis','soft_score_weights','gripper_physical_open_width_m','candidate_min_downward_approach_cos','candidate_max_final_approach_lateral_m','grasp_config'}
 env=dict(vars(remote),self=node,remote_cfg=remote_cfg,gcfg=params['/grasp'],grasp_cfg=params['/grasp'],cam_cfg=params['/camera'],gripper_cfg=config['gripper'],gripper_geometry_cfg=remote_cfg['gripper_geometry'],default_soft_weights=remote.SoftScoreWeights(),soft_weight_cfg=remote_cfg.get('soft_score_weights',{}))
 assignments=[]
 for statement in ctx['init'].body:
  if not isinstance(statement,ast.Assign):continue
  attrs=[z.attr for target in statement.targets for z in ast.walk(target) if isinstance(z,ast.Attribute) and isinstance(z.value,ast.Name) and z.value.id=='self']
  if attrs and all(a in names for a in attrs):exec(compile(ast.Module(body=[statement],type_ignores=[]),str(basepath),'exec'),env);assignments.append(dict(line=statement.lineno,fields=attrs))
 node.grasp_config=deepcopy(params['/grasp']);node._mode_selection=deepcopy(report['grasp_mode_selection']);assert_public_phase=report['observe_return']['phase'];assert assert_public_phase=='observation_contact';node._observe_return_phase='observing_contact';result['phase_provider']='Original observe_return_remote.py public observation_contact maps internal observing_contact, lines1568-1572.'
 cp=E/'ros_log/full_latest_gen41_restart_20261018T011222Z/gen47_runtime/compute_gateway_deployment_r1/consumers.log'
 with cp.open('rb') as f:prefix=f.read(179615)
 assert hashlib.sha256(prefix).hexdigest()=='cd464b48a741d78e46a2d8758d4997ae29212a688b8d26dbfc65192234477ba6'
 reqs=[json.loads(l.split('Observe-return measurement request: ',1)[1]) for l in prefix.decode().splitlines() if 'Observe-return measurement request: ' in l];node._measurement_remote_request=next(r for r in reversed(reqs) if r['state']=='qualified');assert node._measurement_remote_request['target_epoch']==report['target_epoch']
 from alicia_flexible_grasp.vision.rgbd_snapshot import FrozenRgbdSample,SnapshotResult
 from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity,observation_from_snapshot
 from alicia_flexible_grasp.vision.multiview_surface import FusedTargetSurface,SurfaceView,RegistrationResult,RegistrationConfig
 from alicia_flexible_grasp.vision.regular_box_inference import BoxInferenceResult
 from sensor_msgs.msg import JointState
 from types import MappingProxyType
 identity=TargetTrackIdentity(report['target_epoch'],report['multiview_surface']['target_track_id']);folder=V/'gen48_g47_exact_box_visibility_probe_r1/attempt_r6/inputs'/str(stamp);meta=pin(folder/'metadata.json');arrays={}
 for key in ('color_bgr','depth_raw','mask_uint8'):
  p=folder/(key+'.npy');pin(p,meta['files'][key]['file_sha256']);arrays[key]=np.load(p,allow_pickle=False)
 obj=meta['object_message'];bbox=tuple(obj[k] for k in ('bbox_x','bbox_y','bbox_width','bbox_height'));sample=FrozenRgbdSample(arrays['color_bgr'],arrays['depth_raw'],arrays['mask_uint8'],bbox,obj,report['snapshot_stamp_sec'],meta['files']['color_bgr']['frame_id'],None,stamp,identity.epoch,identity)
 # Only the two exact quality scalars consumed by pure stage generation are
 # present. This is not a rebuilt temporal-median/admission snapshot.
 diag=report['remote_diagnostics']['tabletop_geometry'];quality=SimpleNamespace(depth_mad_m=diag['depth_spatial_mad_m'],depth_repeatability_m=diag['depth_repeatability_m'])
 snapshot=SnapshotResult(True,'','',arrays['color_bgr'],arrays['depth_raw'],arrays['depth_raw'],arrays['mask_uint8'],bbox,obj,report['snapshot_stamp_sec'],sample.frame_id,quality,'instance_mask',stamp,identity.epoch,identity,tuple(report['pipeline_funnel']['snapshot_evidence']['source_stamp_ns']),sample,())
 receipts={}
 for key,ref in report['multiview_surface']['fused_scene_geometry']['surface_replay']['references'].items():
  p=Path(ref['store_root'])/ref['relative_path'];raw=pin(p,ref['gzip_sha256']);raw=gzip.decompress(raw);assert hashlib.sha256(raw).hexdigest()==ref['object_sha256'];receipts[key]=json.loads(raw)
 reference=receipts['reference'];ref=reference['reference'];node._stream_condition=threading.Condition(threading.RLock());node._geometry_state_lock=threading.RLock();node._stream_generation=reference['request_binding']['generation'];node._near_field_phase_id=reference['request_binding']['phase_id'];node.near_field_planning_active=True;node._target_identity_generation=1;node.target_instance_epoch=identity.epoch
 assert node._current_stream_target_identity()==identity
 node._near_field_fused_surface=FusedTargetSurface(identity,reference['points_base'],reference['view_indices'],reference['view_stamps_ns'],reference['view_sources']);node._near_field_reference_view=SurfaceView(identity,ref['stamp_ns'],ref['points_base'],ref['support_normal_base'],ref['support_offset_m'],ref['edge_clearance_px']);node._near_field_surface_generation=node._stream_generation;node._near_field_surface_phase_id=node._near_field_phase_id
 T=np.asarray(report['snapshot_transform']['T_base_optical']);node._latest_target_observation=observation_from_snapshot(snapshot,geometry,T);reg=report['multiview_surface']['latest_registration'];registration=RegistrationResult(reg['ok'],reg['code'],reg['snapshot_to_reference_transform'],reg['inlier_count'],reg['overlap_fraction'],reg['rmse_m'],reg['support_normal_angle_deg'],reg['support_plane_separation_m']);node._latest_registration_evidence=remote.RegistrationEvidence(identity,stamp,registration,reg['source_clipped']);node.multiview_registration_config=RegistrationConfig(**params['/grasp_6d/remote']['multiview'])
 joint=JointState();joint.header.stamp=rospy.Time(*divmod(measured['header_stamp_ns'],10**9));joint.header.frame_id=measured['frame_id'];joint.name=measured['names'];joint.position=measured['positions'];node.latest_joint_state=joint
 tool=node._inferred_contact_current_tool_audit();assert tool['available'],tool
 qualified=BoxInferenceResult(q['ok'],q['code'],model,MappingProxyType(q['audit']));node._regular_box_snapshot=dict(lifecycle=node._multiview_lifecycle_token(),observation=node._latest_target_observation,result=qualified,geometry=geometry,planning_geometry=planning,enclosure=enclosure,current_tool=tool)
 surface,_=node._snapshot_multiview_surface();assert np.array_equal(surface.points_base,np.asarray(receipts['snapshot']['points_base'])) and node._current_inferred_box() is model
 assert node._runtime_execution_position_error_m()==error['position_error_m']
 wall=time.perf_counter();cpu=time.thread_time();candidates,diagnostics=node._generate_tabletop_candidates(planning,snapshot=snapshot,contact_execution_phase=True,target_label='unknown_small');wall=time.perf_counter()-wall;cpu=time.thread_time()-cpu
 dr=put(OUT/'materialization_diagnostics.json',plain(diagnostics));cr=put(OUT/'materialized_candidates.json',[{k:plain(getattr(c,k)) for k in c.__dataclass_fields__} for c in candidates]);result.update(tabletop_materializer_replayed=True,materialized_candidate_count=len(candidates),materialization_diagnostics=dr,materialized_candidates=cr,materialization_wall_sec=wall,materialization_thread_cpu_sec=cpu,historical_sdk_seed=measured,historical_execution_error=error,original_current_tool_audit=tool,additional_original_configured_assignments=assignments,parameter_reads=reads,temporal_median_snapshot_recreated=False,observation_provider='Diagnostic TargetObservation from exact retained raw frame and original admitted fused GeometryEstimate; original runtime object/complete prepare lifecycle not reconstructed.',source_surface_exactly_restored=True)
 print(json.dumps(dict(stage='original_Mode_materialization_complete',candidate_count=len(candidates),failure_code=diagnostics.get('failure_code'),wall_sec=wall,thread_cpu_sec=cpu)),flush=True)
 prepared=SimpleNamespace(geometry=planning,snapshot=snapshot,stamp=rospy.Time(*divmod(stamp,10**9)),near_field=True,target_observation=node._latest_target_observation);gates=[]
 for candidate in candidates:
  row=dict(source_index=candidate.source_index,variant_index=candidate.variant_index)
  try:
   normalized=node._normalize_tabletop_candidate(prepared,candidate);row.update(normalization_ok=True,analytic_gate=plain(asdict(normalized.geometry_gate)),required_open_width_m=normalized.required_open_width_m)
  except remote.CandidateContractError as e:row.update(normalization_ok=False,code=e.code,reason=str(e))
  except BaseException as e:row.update(normalization_ok=False,unreconstructed_input_error=repr(e));gates.append(row);break
  gates.append(row)
 gr=put(OUT/'local_normalization_gates.json',gates);result.update(local_normalization_attempted=bool(candidates),normalization_pass_count=sum(r.get('normalization_ok',False) for r in gates),local_normalization_gates=gr,local_endpoint_gate_replayed=False,next_unreplayed_stage='Full prepare identity/acceptance, final registered contact/endpoint evaluators, IK/whole-arm path/simulation and live execution are not reconstructed.')
 return result

def main():
 assert E.parent.parent.is_mount() and not OUT.exists();OUT.mkdir();(OUT/'tmp').mkdir()
 os.environ.update(PYTHONDONTWRITEBYTECODE='1',TMPDIR=str(OUT/'tmp'),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1');sys.dont_write_bytecode=True;sys.addaudithook(audit_event)
 result=dict(kind='G47_candidate_original_Mode_tabletop_diagnostic_replay',completed=False,proposal_count=0,source_unchanged=False,full_prepare_replayed=False,tabletop_materializer_replayed=False,local_endpoint_gate_replayed=False,full_parameter_server_snapshot_available=False,configuration_basis='GEN47 frozen startup grasp_params.yaml plus archived G47 grasp/multiview overrides. No complete historical remote dump; no live/default provider is silently claimed exact.',ROS_imported=False,ROS_init_called=False,hardware_contacted=False,temperature_read=False,physical_grasp_verified=False,execution_authorized=False)
 inputs={}
 def pin(p,h=None):
  x=sha(p)
  if h:assert x==h,(str(p),x)
  inputs[str(p)]=x
  return json.loads(p.read_text()) if p.suffix=='.json' else p.read_bytes()
 try:
  native=os.uname();platform.uname=lambda:platform.uname_result(*(tuple(native)+(native.machine,)));platform.architecture=lambda *a,**k:(str(8*struct.calcsize('P'))+'bit','ELF')
  sys.path.append('/opt/ros/noetic/lib/python3/dist-packages')
  import numpy as np,yaml
  qr=pin(Q/'result.json','03b6c0ed6b37b2c6292aa96f18006877cef0529ac2693fde09df258f21579b1c');assert qr['qualified_box'] and qr['source_unchanged'] and not qr['boundary_attempts'];pins=qr['source_sha256'];assert all(sha(C/p)==h for p,h in pins.items())
  q=pin(Q/'qualification.json',qr['qualification']['sha256']);report=pin(G/'keyframes_audits/19674cf4fc7b10aa290adff95422e8911eaafd18a8ab55c242dd5ca1f959cda2.json','19674cf4fc7b10aa290adff95422e8911eaafd18a8ab55c242dd5ca1f959cda2');archived=pin(G/'perception_execution_configuration.json','ab8b528c42c217aed71f7ee1fff3ad8278912c667d00217346a7d9ff01138136')
  config_path=C/'src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml';config_raw=pin(config_path,'0655edccd1de224d0b95249165e2bfde46c47ac36ac2d38110fd61ea608fc30f');assert sha(B/'src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml')==inputs[str(config_path)];config=yaml.safe_load(config_raw);remote_cfg=config['grasp_6d']['remote'];remote_cfg['multiview']=archived['parameters']['/grasp_6d/remote/multiview']
  sys.path.insert(0,str(C/'src/alicia_flexible_grasp_supervisor/src'))
  from alicia_flexible_grasp.vision.regular_box_inference import RegularBoxInference,enclose_inferred_box_geometry
  from alicia_flexible_grasp.vision.object_geometry import GeometryEstimate
  from alicia_flexible_grasp.vision.multiview_surface import FusedTargetSurface
  from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
  from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry
  from alicia_flexible_grasp.grasp.tabletop_geometry_candidates import TabletopGeometryConfig,generate_tabletop_proposals
  from alicia_flexible_grasp.grasp.hybrid_grasp_candidates import MergeConfig
  from alicia_flexible_grasp.vision.remote_grasp6d_client import CandidateContractError
  source=C/'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py';text=source.read_text();tree=ast.parse(text);pin(source,pins[str(source.relative_to(C))])
  selected={'_continuous_config_error','_strict_config_number','_strict_config_integer','_strict_tabletop_approach_tilt_degrees','load_opening_fit_clearance_config','load_tabletop_geometry_config'};const={'TABLETOP_GEOMETRY_DEFAULTS','TABLETOP_GEOMETRY_MAX_CANDIDATES','CONTINUOUS_CONFIG_INVALID'}
  body=[n for n in tree.body if (isinstance(n,ast.FunctionDef) and n.name in selected) or (isinstance(n,ast.Assign) and any(isinstance(x,ast.Name) and x.id in const for x in n.targets))]
  env=dict(np=np,math=math,GripperGeometry=GripperGeometry,TabletopGeometryConfig=TabletopGeometryConfig,MergeConfig=MergeConfig,CandidateContractError=CandidateContractError);exec(compile(ast.Module(body=body,type_ignores=[]),str(source),'exec'),env)
  init=next(n for c in tree.body if isinstance(c,ast.ClassDef) and c.name=='RemoteGrasp6DNode' for n in c.body if isinstance(n,ast.FunctionDef) and n.name=='__init__');node=SimpleNamespace();env.update(self=node,gripper_geometry_cfg=remote_cfg['gripper_geometry'],remote_cfg=remote_cfg)
  names={'gripper_geometry','opening_fit_clearance_each_side_m','tabletop_geometry_enabled','tabletop_geometry_config','hybrid_merge_config'};assignments=[]
  for n in init.body:
   if not isinstance(n,ast.Assign):continue
   attrs=[z.attr for target in n.targets for z in ast.walk(target) if isinstance(z,ast.Attribute) and isinstance(z.value,ast.Name) and z.value.id=='self']
   if attrs and all(a in names for a in attrs):exec(compile(ast.Module(body=[n],type_ignores=[]),str(source),'exec'),env);assignments.append(dict(line=n.lineno,fields=attrs))
  assert all(hasattr(node,n) for n in names)
  g=report['replay_geometry'];geometry=GeometryEstimate(True,'','',g['obb_center_base_m'],g['R_base_obb'],g['obb_size_xyz_m'],g['support_normal_base'],g['support_offset_m'],g['support_inlier_ratio'],g['object_points_base_m'],g['source_mode']);model=RegularBoxInference.from_evidence_dict(q['model']);planning,enclosure=enclose_inferred_box_geometry(geometry,model)
  ref=report['multiview_surface']['fused_scene_geometry']['surface_replay']['references']['snapshot'];p=Path(ref['store_root'])/ref['relative_path'];raw=pin(p,ref['gzip_sha256']);decoded=gzip.decompress(raw);assert hashlib.sha256(decoded).hexdigest()==ref['object_sha256'];surface_data=json.loads(decoded);identity=TargetTrackIdentity(**surface_data['identity']);surface=FusedTargetSurface(identity,surface_data['points_base'],surface_data['view_indices'],surface_data['view_stamps_ns'],surface_data['view_sources'])
  before=time.perf_counter();cpu=time.thread_time();generation=generate_tabletop_proposals(object_points_base=planning.object_points_base,obb_center_base=planning.center_base,R_base_obb=planning.axes_base,obb_size_xyz_m=planning.size_xyz_m,support_point_base=-planning.support_offset_m*np.asarray(planning.support_normal_base),support_normal_base=planning.support_normal_base,config=node.tabletop_geometry_config,fused_surface=surface,finger_geometry=node.gripper_geometry,refine_obb_neighbor_angles=True,inferred_box=model);wall=time.perf_counter()-before;cpu=time.thread_time()-cpu
  pr=put(OUT/'proposals.json',dict(ok=generation.ok,failure_code=generation.failure_code,failure_reason=generation.failure_reason,sampled_angles_deg=plain(generation.sampled_angles_deg),surface_evidence_rejections=plain(generation.surface_evidence_rejections),proposals=[{k:plain(getattr(p,k)) for k in p.__dataclass_fields__} for p in generation.proposals]))
  result.update(proposals_completed=True,proposal_count=len(generation.proposals),generation_ok=generation.ok,proposals=pr)
  materialize_original(locals())
  assert all(sha(C/p)==h for p,h in pins.items()) and all(sha(Path(p))==h for p,h in inputs.items()) and not BOUNDARY
  result.update(completed=True,generation_ok=generation.ok,proposal_count=len(generation.proposals),failure_code=generation.failure_code,failure_reason=generation.failure_reason,opening_fit_clearance_each_side_m=node.opening_fit_clearance_each_side_m,physical_max_inner_gap_m=node.gripper_geometry.max_inner_gap_m,tabletop_geometry_config=plain(asdict(node.tabletop_geometry_config)),gripper_geometry=plain(asdict(node.gripper_geometry)),original_constructor_assignments=assignments,planning_enclosure=plain(enclosure),proposal_opening_summary=[dict(source_index=p.source_index,angle_deg=p.angle_deg,required_open_width_m=p.required_open_width_m,aperture_margin_m=p.aperture_margin_m) for p in generation.proposals],proposals=pr,wall_sec=wall,thread_cpu_sec=cpu,source_unchanged=True,source_sha256=pins,source_count=len(pins),input_evidence_sha256=inputs,boundary_attempts=BOUNDARY,helper_sha256=sha(__file__))
 except BaseException as e:result.update(error=repr(e),traceback=traceback.format_exc(),boundary_attempts=BOUNDARY,input_evidence_sha256=inputs,helper_sha256=sha(__file__))
 rr=put(OUT/'result.json',result);receipt=put(OUT/'receipt.json',dict(result=rr,boundary_attempts=BOUNDARY,source_unchanged=result['source_unchanged'],full_pipeline_success=False,physical_grasp_verified=False));print(json.dumps(dict(result=rr,receipt=receipt,proposal_count=result['proposal_count'],generation_ok=result.get('generation_ok'),failure_code=result.get('failure_code'),opening=result.get('proposal_opening_summary'),error=result.get('error')),ensure_ascii=False),flush=True);return 0 if result['completed'] else 1
if __name__=='__main__':raise SystemExit(main())
