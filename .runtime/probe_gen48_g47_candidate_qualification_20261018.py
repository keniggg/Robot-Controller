"""Candidate qualification from the sealed G47 pixels and original admitted geometry.
No bag read, ROS import/init, SDK/TF capture, gate patch, provider or motion.
"""
import argparse,hashlib,json,os,sys,time,traceback,platform,struct
from pathlib import Path
from types import SimpleNamespace
W=Path(__file__).resolve().parents[1]
C=W/'.runtime/gen48_visibility_candidate'
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N=E/'ros_log/full_latest_gen41_restart_20261018T011222Z';V=N/'verification'
I=V/'gen48_g47_exact_box_visibility_probe_r1/attempt_r6'
G=E/'grasp_records/20261018T085552.871836Z_7d7d31b2570b433b8ee05398edc47d73'
AUDIT=G/'keyframes_audits/19674cf4fc7b10aa290adff95422e8911eaafd18a8ab55c242dd5ca1f959cda2.json'
STAMPS=(1792313803191381216,1792313803454153060,1792313803719039440)
CUBOID='src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/cuboid_rgb_interval.py'
OUT=None;BOUNDARY=[]
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

def main(args):
 global OUT
 OUT=args.output.resolve();OUT.relative_to(V)
 assert E.parent.parent.is_mount() and not OUT.exists()
 OUT.mkdir();(OUT/'tmp').mkdir();(OUT/'unused_ros').mkdir()
 os.environ.update(PYTHONDONTWRITEBYTECODE='1',ROS_MASTER_URI='http://127.0.0.1:1',ROS_LOG_DIR=str(OUT/'unused_ros'),ROS_HOME=str(OUT/'unused_ros'),TMPDIR=str(OUT/'tmp'),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
 sys.dont_write_bytecode=True;sys.addaudithook(audit_event)
 result=dict(kind='gen48_candidate_G47_exact_input_box_qualification',qualification_completed=False,qualified_box=False,full_regression_passed=False,runtime_deployed=False,physical_grasp_verified=False,ROS_imported=False,ROS_init_called=False,hardware_contacted=False,temperature_queried=False,motion_commands_sent=False,bag_read=False,temporal_median_depth_recreated=False,execution_snapshot_recreated=False,boundary_attempts=BOUNDARY)
 inputs={};source_map={}
 def pin(path,expected=None):
  actual=sha(path)
  if expected is not None:assert actual==expected,(str(path),actual)
  inputs[str(path)]=actual
  return json.loads(path.read_text()) if path.suffix=='.json' else actual
 try:
  baseline_path=W/'.runtime/gen48_baseline_source/manifest.json';base=pin(baseline_path,'36d16b9ad93a8a2866febb4140d9e165c59424ead0db64df904017a63013c28f')
  original=base['source_sha256'];assert len(original)==1052
  source_map={p:sha(C/p) for p in original}
  assert {p for p,h in source_map.items() if h!=original[p]}=={CUBOID}
  assert source_map[CUBOID]==args.candidate_cuboid_sha256
  root_result=pin(V/'gen48_g47_exact_box_visibility_probe_r1/result.json','272a4b842e6dde637f2d737b3a042fba615e2e3c80746722f8faab56647f48f1')
  assert root_result['source_pixels_exact'] and root_result['reproduced_original_failure']
  report=pin(AUDIT,'19674cf4fc7b10aa290adff95422e8911eaafd18a8ab55c242dd5ca1f959cda2')
  manifest=pin(G/'manifest.json','3350160fa69029890de9fd23c99b17aa5e3e74c6c47ff2adeccfb00fb854c7bc')
  assert manifest['state']=='complete' and manifest['result']=='failure' and manifest['recorder_closed']
  expected=report['remote_diagnostics']['tabletop_geometry']['regular_box_inference']
  assert tuple(report['pipeline_funnel']['snapshot_evidence']['source_stamp_ns'])==STAMPS
  camera=pin(I/'camera_metadata.json','c35f528c163c618887604f84aa0fe36d12a816b41d5a17e8825022d1d13048bc')
  assert camera['all_recorded_numeric_intrinsics_and_scales_consistent']
  import numpy as np
  native_uname=os.uname()
  platform.uname=lambda:platform.uname_result(*(tuple(native_uname)+(native_uname.machine,)))
  platform.architecture=lambda *a,**k:(str(8*struct.calcsize('P'))+'bit','ELF')
  sys.path.insert(0,str(C/'src/alicia_flexible_grasp_supervisor/src'))
  from alicia_flexible_grasp.vision import regular_box_inference as box
  from alicia_flexible_grasp.vision import cuboid_rgb_interval as cuboid
  from alicia_flexible_grasp.vision.object_geometry import GeometryEstimate
  from alicia_flexible_grasp.vision.rgbd_snapshot import FrozenRgbdSample,SnapshotResult
  from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
  records={};mask_hashes=[]
  for ns in STAMPS:
   folder=I/'inputs'/str(ns);metadata=pin(folder/'metadata.json')
   assert metadata['source_stamp_ns']==ns and metadata['exact_source'] and metadata['intrinsics']==camera['intrinsics']
   arrays={}
   for key in ('color_bgr','depth_raw','mask_uint8'):
    meta=metadata['files'][key];path=folder/(key+'.npy');pin(path,meta['file_sha256']);a=np.load(path,allow_pickle=False)
    assert list(a.shape)==meta['shape'] and str(a.dtype)==meta['dtype'] and box._array_hash(a)==meta['array_sha256']
    arrays[key]=a
   mask_hashes.append(box._array_hash((arrays['mask_uint8']>0).astype(np.uint8)))
   records[ns]=dict(arrays=arrays,metadata=metadata)
  ns=STAMPS[-1];last=records[ns];final=last['arrays'];metadata=last['metadata'];obj=metadata['object_message']
  for name,key in [('color','color_bgr'),('depth','depth_raw'),('mask','mask_uint8')]:assert box._array_hash(final[key])==expected['retained_pixel_sha256'][name+'_sha256']
  assert mask_hashes[-1]==expected['final_sample_binary_mask_sha256']
  identity=TargetTrackIdentity(report['target_epoch'],expected['source_target_identity']['track_id'])
  bbox=tuple(obj[k] for k in ('bbox_x','bbox_y','bbox_width','bbox_height'))
  sample=FrozenRgbdSample(final['color_bgr'],final['depth_raw'],final['mask_uint8'],bbox,obj,report['snapshot_stamp_sec'],metadata['files']['color_bgr']['frame_id'],None,ns,identity.epoch,identity)
  snapshot=SnapshotResult(True,'','',final['color_bgr'],final['depth_raw'],final['depth_raw'],final['mask_uint8'],bbox,obj,report['snapshot_stamp_sec'],sample.frame_id,None,'instance_mask',ns,identity.epoch,identity,STAMPS,sample,tuple(mask_hashes))
  assert box._retained_box_pixels(snapshot,identity,ns)[0] is sample
  g=report['replay_geometry'];geometry=GeometryEstimate(True,'','',g['obb_center_base_m'],g['R_base_obb'],g['obb_size_xyz_m'],g['support_normal_base'],g['support_offset_m'],g['support_inlier_ratio'],g['object_points_base_m'],g['source_mode'])
  assert len(geometry.object_points_base)==g['object_points_count']
  T=np.asarray(report['snapshot_transform']['T_base_optical']);intrinsics=SimpleNamespace(**camera['intrinsics'])
  put(OUT/'input_context.json',dict(source_stamps_ns=STAMPS,input_evidence_sha256=inputs,source_sha256=source_map,changed_production_path=CUBOID,camera_intrinsics=camera['intrinsics'],camera_info_present=camera['camera_info_crosscheck_present'],camera_parameters_not_image_synchronized=camera['parameters_are_not_image_synchronized'],geometry_provider='Exact GeometryEstimate fields and complete measured point list from original admitted G47 audit replay_geometry; not recomputed from RGBD.',geometry_point_count=len(geometry.object_points_base),snapshot_provider='Qualification-only carrier uses exact retained final raw frame plus original3 source stamps and mask hashes, not a recreated executable temporal-median snapshot.',T_base_optical=T.tolist()))
  print(json.dumps(dict(stage='exact_sources_bound',candidate_cuboid_sha256=source_map[CUBOID],geometry_point_count=len(geometry.object_points_base))),flush=True)
  wall=time.perf_counter();cpu=time.thread_time()
  qualification=box.qualify_regular_box_snapshot(snapshot,intrinsics,intrinsics.depth_scale,T,geometry,identity,ns)
  elapsed=time.perf_counter()-wall;threadcpu=time.thread_time()-cpu
  audit=cuboid._plain(dict(qualification.audit));model=qualification.model.to_evidence_dict() if qualification.model is not None else None
  if model is not None:box.validate_box_model_evidence(model)
  qr=put(OUT/'qualification.json',dict(ok=qualification.ok,code=qualification.code,audit=audit,model=model,wall_sec=elapsed,thread_cpu_sec=threadcpu))
  print(json.dumps(dict(stage='qualification_complete',ok=qualification.ok,code=qualification.code,reason=audit.get('reason'),cuboid_reason=audit.get('cuboid_qualification_reason'),wall_sec=elapsed,thread_cpu_sec=threadcpu)),flush=True)
  provenance={}
  for name,module in list(sys.modules.items()):
   path=getattr(module,'__file__',None)
   if not path:continue
   p=Path(path).resolve()
   try:rel=str(p.relative_to(C))
   except ValueError:
    if name.startswith('alicia_flexible_grasp'):raise AssertionError('foreign production import '+str(p))
    continue
   assert rel in source_map and sha(p)==source_map[rel],(name,str(p))
   provenance[name]=dict(path=str(p),sha256=source_map[rel])
  assert all(sha(C/p)==h for p,h in source_map.items())
  assert all(sha(Path(p))==h for p,h in inputs.items()) and not BOUNDARY
  result.update(qualification_completed=True,qualified_box=qualification.ok,qualification_code=qualification.code,qualification_reason=audit.get('reason'),cuboid_qualification_reason=audit.get('cuboid_qualification_reason'),qualification=qr,wall_sec=elapsed,thread_cpu_sec=threadcpu,source_sha256=source_map,source_count=len(source_map),source_unchanged=True,input_evidence_sha256=inputs,source_provenance=provenance,retained_pixel_sha256=audit.get('retained_pixel_sha256'),helper_sha256=sha(__file__),original_record_unchanged=True,downstream_tabletop_run=False,scope='Counterfactual candidate box qualification on exact G47 retained pixels and original admitted replay geometry only; no lifecycle/Task/robot authority or full pipeline claim.')
 except BaseException as error:
  result.update(error_type=type(error).__name__,error=str(error),traceback=traceback.format_exc(),source_sha256=source_map,input_evidence_sha256=inputs,source_unchanged=bool(source_map) and all(sha(C/p)==h for p,h in source_map.items()),helper_sha256=sha(__file__))
 rr=put(OUT/'result.json',result)
 seal=put(OUT/'receipt.json',dict(result=rr,helper_sha256=sha(__file__),boundary_attempts=BOUNDARY,qualification_completed=result['qualification_completed'],qualified_box=result['qualified_box'],source_unchanged=result.get('source_unchanged',False),source_count=len(source_map),full_regression_claimed=False,runtime_deployed=False))
 print(json.dumps(dict(result=rr,receipt=seal,qualified_box=result['qualified_box'],qualification_completed=result['qualification_completed'],error=result.get('error'))),flush=True)
 return 0 if result['qualification_completed'] and not BOUNDARY and result.get('source_unchanged') else 1

if __name__=='__main__':
 parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);parser.add_argument('--candidate-cuboid-sha256',required=True);args=parser.parse_args();raise SystemExit(main(args))
