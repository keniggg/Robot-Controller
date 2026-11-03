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
OUT=V/'gen48_g47_candidate_proposals_r2';Q=V/'gen48_g47_candidate_qualification_r1';BOUNDARY=[]
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

def main():
 assert E.parent.parent.is_mount() and not OUT.exists();OUT.mkdir();(OUT/'tmp').mkdir()
 os.environ.update(PYTHONDONTWRITEBYTECODE='1',TMPDIR=str(OUT/'tmp'),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1');sys.dont_write_bytecode=True;sys.addaudithook(audit_event)
 result=dict(kind='G47_candidate_original_pure_proposal_stage',completed=False,proposal_count=0,source_unchanged=False,full_prepare_replayed=False,tabletop_materializer_replayed=False,local_endpoint_gate_replayed=False,full_parameter_server_snapshot_available=False,configuration_basis='GEN47 frozen startup grasp_params.yaml plus archived G47 grasp/multiview overrides. No complete historical remote dump; no live/default provider is silently claimed exact.',ROS_imported=False,ROS_init_called=False,hardware_contacted=False,temperature_read=False,physical_grasp_verified=False,execution_authorized=False)
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
  assert all(sha(C/p)==h for p,h in pins.items()) and all(sha(Path(p))==h for p,h in inputs.items()) and not BOUNDARY
  result.update(completed=True,generation_ok=generation.ok,proposal_count=len(generation.proposals),failure_code=generation.failure_code,failure_reason=generation.failure_reason,opening_fit_clearance_each_side_m=node.opening_fit_clearance_each_side_m,physical_max_inner_gap_m=node.gripper_geometry.max_inner_gap_m,tabletop_geometry_config=plain(asdict(node.tabletop_geometry_config)),gripper_geometry=plain(asdict(node.gripper_geometry)),original_constructor_assignments=assignments,planning_enclosure=plain(enclosure),proposal_opening_summary=[dict(source_index=p.source_index,angle_deg=p.angle_deg,required_open_width_m=p.required_open_width_m,aperture_margin_m=p.aperture_margin_m) for p in generation.proposals],proposals=pr,wall_sec=wall,thread_cpu_sec=cpu,source_unchanged=True,source_sha256=pins,source_count=len(pins),input_evidence_sha256=inputs,boundary_attempts=BOUNDARY,helper_sha256=sha(__file__),next_unreplayed_stage='Original ModeAware tabletop adaptive/boundary materialization, candidate normalization and endpoint gates; no full prepare or authority reconstructed.')
 except BaseException as e:result.update(error=repr(e),traceback=traceback.format_exc(),boundary_attempts=BOUNDARY,input_evidence_sha256=inputs,helper_sha256=sha(__file__))
 rr=put(OUT/'result.json',result);receipt=put(OUT/'receipt.json',dict(result=rr,boundary_attempts=BOUNDARY,source_unchanged=result['source_unchanged'],full_pipeline_success=False,physical_grasp_verified=False));print(json.dumps(dict(result=rr,receipt=receipt,proposal_count=result['proposal_count'],generation_ok=result.get('generation_ok'),failure_code=result.get('failure_code'),opening=result.get('proposal_opening_summary'),error=result.get('error')),ensure_ascii=False),flush=True);return 0 if result['completed'] else 1
if __name__=='__main__':raise SystemExit(main())
