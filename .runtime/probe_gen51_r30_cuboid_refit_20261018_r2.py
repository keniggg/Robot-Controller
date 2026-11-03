"""Bounded R30 qualification comparison; no ROS authority or execution carrier."""
import argparse, hashlib, importlib.util, io, json, os, platform, signal, struct, sys, time, traceback
from pathlib import Path
from types import SimpleNamespace

W=Path(__file__).resolve().parents[1]
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
V=E/'ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
ROOT=V/'gen51_retry_tabletop_review_r1'
G=E/'grasp_records/20261018T123624.045793Z_1db3d3bb4c554d7590c8c8003c414fb5'
AUDIT=G/'keyframes_audits/3fcf580665a8226efad87b55d1a693dbfd029a7a4641ad3208ee4679d8a7b39b.json'
INDEX=V/'gen50_retry_actual_handoff_r1/station_source_availability_r1/index.json'
REL='src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/cuboid_rgb_interval.py'
STAMPS=(1792327028528818845,1792327028792045116,1792327029323935508)
OUT=None; BOUNDARY=[]
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb')as f:
  for b in iter(lambda:f.read(4*1024*1024),b''):h.update(b)
 return h.hexdigest()
def put(p,v):
 raw=(json.dumps(v,ensure_ascii=False,indent=2,allow_nan=False)+'\n').encode()
 with p.open('xb')as f:f.write(raw)
 assert p.read_bytes()==raw
 return dict(path=str(p),sha256=sha(p))
def deny(kind,detail):
 BOUNDARY.append(dict(kind=kind,detail=str(detail)));raise RuntimeError('offline boundary '+kind)
def guard(event,args):
 if event.startswith('socket.'):deny(event,args[:1])
 if event in ('subprocess.Popen','os.system','os.exec','os.posix_spawn','os.fork','os.forkpty'):deny(event,'process creation')
 if event=='open' and isinstance(args[0],(str,bytes,os.PathLike)):
  p=os.fsdecode(args[0]);flags=args[2]if len(args)>2 and isinstance(args[2],int)else 0
  mode=args[1]if len(args)>1 and isinstance(args[1],str)else ''
  if p.startswith('/dev/')and p not in ('/dev/null','/dev/urandom','/dev/random'):deny('device',p)
  if p.startswith('/sys/class/thermal')or '/hwmon/'in p:deny('temperature',p)
  if p!='/dev/null'and(flags&(os.O_WRONLY|os.O_RDWR|os.O_CREAT|os.O_TRUNC|os.O_APPEND)or any(c in mode for c in 'wax+')):
   try:Path(p).resolve().relative_to(OUT)
   except ValueError:deny('write_outside_output',p)
def pin(p,h=None):
 actual=sha(p)
 if h:assert actual==h,(str(p),actual)
 return actual
def wire_sha(msg):
 stream=io.BytesIO();msg.serialize(stream);return hashlib.sha256(stream.getvalue()).hexdigest()
def setup(args):
 global OUT
 OUT=(ROOT/(args.attempt or args.mode)).resolve();OUT.relative_to(ROOT)
 assert E.parent.parent.is_mount()and not OUT.exists()
 OUT.mkdir();(OUT/'tmp').mkdir()
 os.environ.update(PYTHONDONTWRITEBYTECODE='1',ROS_MASTER_URI='http://127.0.0.1:1',ROS_HOME=str(OUT/'tmp'),ROS_LOG_DIR=str(OUT/'tmp'),TMPDIR=str(OUT/'tmp'),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
 sys.dont_write_bytecode=True
 native=os.uname();platform.uname=lambda:platform.uname_result(*(tuple(native)+(native.machine,)))
 platform.architecture=lambda *a,**k:(str(8*struct.calcsize('P'))+'bit','ELF')
 sys.addaudithook(guard)
 signal.signal(signal.SIGALRM,lambda *a:(_ for _ in ()).throw(TimeoutError('60 second attempt budget')))
 signal.alarm(55)
def metadata():
 manifest=json.loads((G/'manifest.json').read_bytes());assert manifest['state']=='complete'and manifest['result']=='failure'and manifest['recorder_closed']
 ref=next(r for r in manifest['files']if r['path']=='keyframes.bag');p=G/'keyframes.bag'
 before=pin(p,ref['sha256']);assert p.stat().st_size==ref['size_bytes']
 sys.path.insert(0,'/opt/ros/noetic/lib/python3/dist-packages')
 import rosbag,rospy
 for name in ('init_node','Publisher','Subscriber','Service','ServiceProxy','wait_for_service','get_param','set_param','delete_param'):
  setattr(rospy,name,lambda *a,_name=name,**k:deny('ROS_'+_name,'forbidden'))
 topics=['/grasp_record/camera_runtime_metadata','/supervisor/camera/color/camera_info']
 rows=[];info=[];keys=('width','height','fx','fy','cx','cy','depth_scale')
 with rosbag.Bag(str(p),'r')as bag:
  for topic,msg,received in bag.read_messages(topics=topics):
   if topic==topics[0]:
    v=json.loads(msg.data);params=v.get('parameters')
    if isinstance(params,dict)and all(k in params for k in keys):rows.append(dict(received_ns=received.to_nsec(),wire_sha256=wire_sha(msg),intrinsics={k:params[k]for k in keys}))
   else:info.append(dict(received_ns=received.to_nsec(),source_ns=msg.header.stamp.to_nsec(),width=msg.width,height=msg.height,K=list(msg.K),wire_sha256=wire_sha(msg)))
 assert rows and all(r['intrinsics']==rows[0]['intrinsics']for r in rows)
 cam=rows[0]['intrinsics'];expected=[cam['fx'],0.,cam['cx'],0.,cam['fy'],cam['cy'],0.,0.,1.]
 assert all(r['width']==cam['width']and r['height']==cam['height']and r['K']==expected for r in info)
 assert pin(p)==before and not BOUNDARY
 return dict(intrinsics=cam,runtime_metadata=rows,camera_info=info,parameters_not_exposure_synchronized=True,read_topics=topics,bag_sha256=before,bag_manifest_sha256=pin(G/'manifest.json'),boundary_attempts=BOUNDARY)
def qualification(mode):
 import numpy as np,cv2
 cv2.setNumThreads(1)
 release_path=V/'gen50_release_manifest.json';pin(release_path,'4151ad0e3d0eceb382a2cb006dafd9614a793618f74a75c2d1b5bc234c8630ca')
 release=json.loads(release_path.read_bytes());sources=release['source_sha256'];assert len(sources)==1058
 assert all(sha(W/p)==h for p,h in sources.items())
 assert sources[REL]=='f724adea2782b414cccac9154bbd8166ffd498e6f3d4742a68712aa62835ade5'
 sys.path.insert(0,str(W/'src/alicia_flexible_grasp_supervisor/src'))
 import alicia_flexible_grasp.vision
 candidate=W/'.runtime/gen51_cuboid_refit_candidate_r2'/REL
 if mode=='candidate':
  pin(candidate,'bd0d8371577b452d7beb6252994de0e94fd6149efa67e4bf051e5cefb119fa7d')
  name='alicia_flexible_grasp.vision.cuboid_rgb_interval';spec=importlib.util.spec_from_file_location(name,candidate)
  module=importlib.util.module_from_spec(spec);sys.modules[name]=module;spec.loader.exec_module(module);setattr(alicia_flexible_grasp.vision,'cuboid_rgb_interval',module)
 from alicia_flexible_grasp.vision import regular_box_inference as box,cuboid_rgb_interval as cuboid
 from alicia_flexible_grasp.vision.object_geometry import GeometryEstimate
 from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
 pin(AUDIT,'3fcf580665a8226efad87b55d1a693dbfd029a7a4641ad3208ee4679d8a7b39b');report=json.loads(AUDIT.read_bytes())
 assert tuple(report['pipeline_funnel']['snapshot_evidence']['source_stamp_ns'])==STAMPS and report['snapshot_stamp_ns']==STAMPS[-1]
 pin(INDEX,'a6f0abf94627bc07870d1deead1cec304415ac1a2dcefe553a2bdba857683e62');index=json.loads(INDEX.read_bytes())
 row=next(r for r in index['by_source']if r['source_stamp_ns']==STAMPS[-1]);arrays={};array_inputs={}
 for r in row['exact_messages']:
  a=r['array'];p=Path(a['path']);pin(p,a['file_sha256']);v=np.load(p,allow_pickle=False)
  assert str(v.dtype)==a['dtype']and list(v.shape)==a['shape']and box._array_hash(v)==a['array_sha256'];arrays[r['kind']]=v;array_inputs[str(p)]=dict(file_sha256=a['file_sha256'],array_sha256=a['array_sha256'])
 expected=report['remote_diagnostics']['tabletop_geometry']['regular_box_inference']
 for kind,key in (('color_bgr','color'),('depth_raw','depth'),('mask_original','mask')):assert box._array_hash(arrays[kind])==expected['retained_pixel_sha256'][key+'_sha256']
 camera_path=ROOT/'metadata/result.json';camresult=json.loads(camera_path.read_bytes());assert camresult['completed']and not camresult['boundary_attempts']
 cam=camresult['output']['intrinsics'];K=SimpleNamespace(**cam)
 g=report['replay_geometry'];geometry=GeometryEstimate(True,'','',g['obb_center_base_m'],g['R_base_obb'],g['obb_size_xyz_m'],g['support_normal_base'],g['support_offset_m'],g['support_inlier_ratio'],g['object_points_base_m'],g['source_mode'])
 identity=TargetTrackIdentity(report['target_epoch'],expected['source_target_identity']['track_id']);T=np.asarray(report['snapshot_transform']['T_base_optical'])
 wall=time.perf_counter();cpu=time.thread_time()
 q=box.qualify_regular_box_top(arrays['depth_raw'],arrays['mask_original'],K,K.depth_scale,T,geometry,identity,STAMPS[-1],'instance_mask',color_bgr=arrays['color_bgr'])
 elapsed=time.perf_counter()-wall;threadcpu=time.thread_time()-cpu;audit=cuboid._plain(dict(q.audit));model=q.model.to_evidence_dict()if q.model is not None else None
 if model is not None:box.validate_box_model_evidence(model)
 c=audit.get('cuboid_rgb_interval')or {};maximum=max(c['full_silhouette_rgb_distances_px'])if 'full_silhouette_rgb_distances_px'in c else None
 if mode=='baseline':
  assert not q.ok and q.code==expected['code']and audit['cuboid_qualification_reason']==expected['cuboid_qualification_reason']
 comparison=dict(original_maximum=max(expected['cuboid_rgb_interval']['full_silhouette_rgb_distances_px']),maximum_difference_px=maximum-max(expected['cuboid_rgb_interval']['full_silhouette_rgb_distances_px']),original_reason=expected['cuboid_qualification_reason'],reason_equal=audit.get('cuboid_qualification_reason')==expected['cuboid_qualification_reason'])
 provenance={}
 for name,module in list(sys.modules.items()):
  if not name.startswith('alicia_flexible_grasp'):continue
  file=getattr(module,'__file__',None)
  if not file:continue
  p=Path(file).resolve()
  if mode=='candidate'and p==candidate:provenance[name]=dict(path=str(p),sha256=sha(p));continue
  rel=str(p.relative_to(W));assert rel in sources and sha(p)==sources[rel];provenance[name]=dict(path=str(p),sha256=sources[rel])
 assert all(sha(W/p)==h for p,h in sources.items())and not BOUNDARY
 return dict(ok=q.ok,code=q.code,audit=audit,model=model,wall_sec=elapsed,thread_cpu_sec=threadcpu,maximum_cuboid_silhouette_distance_px=maximum,original_comparison=comparison,source_count=len(sources),source_sha256=sources,source_provenance=provenance,inputs_sha256={str(AUDIT):sha(AUDIT),str(INDEX):sha(INDEX),str(camera_path):sha(camera_path)},array_inputs=array_inputs,source_unchanged=True,original_geometry_points_count=len(geometry.object_points_base),original_source_ns=STAMPS[-1],public_api='qualify_regular_box_top',scope='Exact final retained RGBD/mask, original admitted fused geometry/TF/identity, recorded camera parameters. No reconstructed temporal snapshot, receipt, Task, execution or motion authority.',snapshot_source_binding_replayed=False)
def main(args):
 setup(args);result=dict(kind='gen51_r30_bounded_cuboid_refit_diagnostic',mode=args.mode,completed=False,physical_grasp_claimed=False,runtime_deployed=False,ROS_init_called=False,hardware_contacted=False,temperature_queried=False,boundary_attempts=BOUNDARY,helper_sha256=sha(__file__))
 try:result.update(output=metadata()if args.mode=='metadata'else qualification(args.mode),completed=True)
 except BaseException as e:result.update(error_type=type(e).__name__,error=str(e),traceback=traceback.format_exc())
 finally:signal.alarm(0)
 result['ROS_imported']='rospy'in sys.modules
 rr=put(OUT/'result.json',result);receipt=put(OUT/'receipt.json',dict(result=rr,helper_sha256=sha(__file__),completed=result['completed'],boundary_attempts=BOUNDARY,all_writes_confined=True))
 output=result.get('output',{});print(json.dumps(dict(result=rr,receipt=receipt,completed=result['completed'],ok=output.get('ok'),code=output.get('code'),maximum=output.get('maximum_cuboid_silhouette_distance_px'),wall_sec=output.get('wall_sec'),thread_cpu_sec=output.get('thread_cpu_sec'),error=result.get('error')),ensure_ascii=False),flush=True)
 return 0 if result['completed']and not BOUNDARY else 1
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--mode',choices=('metadata','baseline','candidate'),required=True);p.add_argument('--attempt');raise SystemExit(main(p.parse_args()))
