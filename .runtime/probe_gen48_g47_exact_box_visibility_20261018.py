#!/usr/bin/env python3
"""Exact closed G47 RGBD -> original box qualifier, with read-only visibility trace.

No gate is patched. The temporary snapshot is a qualification-only carrier of
original retained pixels and admitted replay geometry, not an execution snapshot.
"""
import hashlib,io,json,os,sys,time,traceback,platform,struct
from pathlib import Path
from types import SimpleNamespace

W=Path(__file__).resolve().parents[1]
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N=E/'ros_log/full_latest_gen41_restart_20261018T011222Z'
V=N/'verification'; OUT=V/'gen48_g47_exact_box_visibility_probe_r1'/'attempt_r6'
G=E/'grasp_records/20261018T085552.871836Z_7d7d31b2570b433b8ee05398edc47d73'
STAMPS=(1792313803191381216,1792313803454153060,1792313803719039440)
AUDIT=G/'keyframes_audits/19674cf4fc7b10aa290adff95422e8911eaafd18a8ab55c242dd5ca1f959cda2.json'
BOUNDARY=[]

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
def plain_message(value):
 if hasattr(value,'to_nsec'):return value.to_nsec()
 if hasattr(value,'__slots__'):return {k:plain_message(getattr(value,k))for k in value.__slots__}
 if isinstance(value,(list,tuple)):return [plain_message(x)for x in value]
 return value
def raw_sha(msg):
 stream=io.BytesIO();msg.serialize(stream);return hashlib.sha256(stream.getvalue()).hexdigest()
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

def main():
 assert E.parent.parent.is_mount() and not OUT.exists()
 OUT.mkdir();(OUT/'tmp').mkdir();(OUT/'ros').mkdir()
 os.environ.update(PYTHONDONTWRITEBYTECODE='1',ROS_MASTER_URI='http://127.0.0.1:1',ROS_LOG_DIR=str(OUT/'ros'),ROS_HOME=str(OUT/'ros'),TMPDIR=str(OUT/'tmp'),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
 sys.dont_write_bytecode=True;sys.addaudithook(audit_event)
 import numpy as np
 # Cryptodome otherwise shells out to file solely to discover pointer width.
 native_uname=os.uname()
 platform.uname=lambda:platform.uname_result(*(tuple(native_uname)+(native_uname.machine,)))
 platform.architecture=lambda *a,**k:(str(8*struct.calcsize('P'))+'bit','ELF')
 import rosbag
 import rospy
 for name in ('init_node','Publisher','Subscriber','Service','ServiceProxy','wait_for_service','get_param','set_param','delete_param'):
  setattr(rospy,name,lambda *a,_name=name,**k:deny('ROS_'+_name,'forbidden'))
 sys.path.insert(0,str(W/'src/alicia_flexible_grasp_supervisor/src'))
 from alicia_flexible_grasp.vision import regular_box_inference as box
 from alicia_flexible_grasp.vision import cuboid_rgb_interval as cuboid
 from alicia_flexible_grasp.vision.object_geometry import GeometryEstimate
 from alicia_flexible_grasp.vision.rgbd_snapshot import FrozenRgbdSample,SnapshotResult
 from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
 release_path=V/'gen47_release_manifest.json';release=json.loads(release_path.read_text());assert release['passed']and release['reviewed']and release['source_count']==1052
 source_map=release['source_sha256'];assert all(sha(W/rel)==h for rel,h in source_map.items())
 evidence={str(release_path):sha(release_path),str(AUDIT):sha(AUDIT),str(G/'manifest.json'):sha(G/'manifest.json'),str(Path(__file__)):sha(Path(__file__))}
 assert evidence[str(AUDIT)]=='19674cf4fc7b10aa290adff95422e8911eaafd18a8ab55c242dd5ca1f959cda2'
 report=json.loads(AUDIT.read_text());expected=report['remote_diagnostics']['tabletop_geometry']['regular_box_inference'];assert tuple(report['pipeline_funnel']['snapshot_evidence']['source_stamp_ns'])==STAMPS
 manifest=json.loads((G/'manifest.json').read_text());assert manifest['recorder_closed']and manifest['state']=='complete'and manifest['result']=='failure'
 bagref=next(row for row in manifest['files']if row['path']=='keyframes.bag');bag_path=G/'keyframes.bag'
 assert bag_path.stat().st_size==bagref['size_bytes']and sha(bag_path)==bagref['sha256'];evidence[str(bag_path)]=bagref['sha256']
 print(json.dumps(dict(stage='sealed_bag_verified',bag_bytes=bagref['size_bytes'],source_count=len(source_map))),flush=True)
 image_topics={'/supervisor/camera/color/image_raw':'color_bgr','/supervisor/camera/depth/image_raw':'depth_raw','/perception/object_mask':'mask_uint8'}
 topics=list(image_topics)+['/perception/object','/supervisor/camera/color/camera_info','/grasp_record/camera_runtime_metadata']
 records={ns:{}for ns in STAMPS}; camera_metadata=[]; camera_info=[]
 def decode(msg,key):
  enc=msg.encoding.lower();raw=bytes(msg.data)
  if enc in ('bgr8','rgb8'):dtype=np.dtype('u1');channels=3
  elif enc in ('mono8','8uc1'):dtype=np.dtype('u1');channels=1
  elif enc in ('16uc1','mono16'):dtype=np.dtype('>u2'if msg.is_bigendian else '<u2');channels=1
  elif enc=='32fc1':dtype=np.dtype('>f4'if msg.is_bigendian else '<f4');channels=1
  else:raise ValueError('unsupported image encoding '+enc)
  assert len(raw)==msg.step*msg.height
  row=np.frombuffer(raw,dtype=dtype).reshape(msg.height,msg.step//dtype.itemsize)
  arr=row[:,:msg.width*channels].reshape((msg.height,msg.width,channels)if channels>1 else(msg.height,msg.width)).copy()
  if enc=='rgb8':arr=arr[...,::-1].copy()
  arr=arr.astype(arr.dtype.newbyteorder('='),copy=False)
  return arr,dict(encoding=msg.encoding,wire_data_sha256=hashlib.sha256(raw).hexdigest(),shape=list(arr.shape),dtype=str(arr.dtype),array_sha256=box._array_hash(arr),step=msg.step,frame_id=msg.header.frame_id)
 with rosbag.Bag(str(bag_path),'r')as bag:
  for topic,msg,received in bag.read_messages(topics=topics):
   if topic=='/grasp_record/camera_runtime_metadata':
    q=json.loads(msg.data)
    if isinstance(q.get('parameters'),dict):camera_metadata.append(dict(received_ns=received.to_nsec(),wire_sha256=raw_sha(msg),value=q))
    continue
   if topic.endswith('camera_info'):
    camera_info.append(dict(received_ns=received.to_nsec(),source_stamp_ns=msg.header.stamp.to_nsec(),width=msg.width,height=msg.height,K=list(msg.K),frame_id=msg.header.frame_id,wire_sha256=raw_sha(msg)));continue
   ns=msg.header.stamp.to_nsec()
   if ns not in records:continue
   key=image_topics.get(topic,'object_message')
   if key=='object_message':value=plain_message(msg);entry=dict(topic=topic,received_ns=received.to_nsec(),source_stamp_ns=ns,wire_sha256=raw_sha(msg),value=value)
   else:
    arr,meta=decode(msg,key);value=arr;entry=dict(topic=topic,received_ns=received.to_nsec(),source_stamp_ns=ns,wire_sha256=raw_sha(msg),**meta)
   if key in records[ns]:
    prior=records[ns][key];assert prior['metadata']['wire_sha256']==entry['wire_sha256'];prior['metadata']['duplicate_count']=prior['metadata'].get('duplicate_count',0)+1
   else:records[ns][key]=dict(value=value,metadata=entry)
 assert all(set(row)==set(image_topics.values())|{'object_message'}for row in records.values()),{ns:list(row)for ns,row in records.items()}
 print(json.dumps(dict(stage='camera_evidence_counts',runtime_metadata=len(camera_metadata),camera_info=len(camera_info))),flush=True)
 put(OUT/'camera_evidence_raw.json',dict(runtime_metadata=camera_metadata,camera_info=camera_info))
 assert camera_metadata
 # Camera runtime metadata is recorded parameter evidence, not an image exposure stamp.
 # Require one exact numerical intrinsic/scale set through this closed attempt.
 camera_keys=('width','height','fx','fy','cx','cy','depth_scale')
 cams=[{key:entry['value']['parameters'][key]for key in camera_keys}for entry in camera_metadata]
 assert all(cam==cams[0]for cam in cams);cam=cams[0];intrinsics=SimpleNamespace(**cam)
 assert all(int(row['width'])==cam['width']and int(row['height'])==cam['height']and list(row['K'])==[cam['fx'],0.,cam['cx'],0.,cam['fy'],cam['cy'],0.,0.,1.]for row in camera_info)
 metadata_refs=[];arrays={};masks=[]
 for ns,row in records.items():
  folder=OUT/'inputs'/str(ns);folder.mkdir(parents=True)
  for key in image_topics.values():
   p=folder/(key+'.npy');np.save(p,row[key]['value'],allow_pickle=False);assert box._array_hash(np.load(p,allow_pickle=False))==row[key]['metadata']['array_sha256'];row[key]['metadata']['file']=str(p);row[key]['metadata']['file_sha256']=sha(p)
  obj=row['object_message']['value'];assert obj['detected']is True
  metadata_refs.append(put(folder/'metadata.json',dict(source_stamp_ns=ns,exact_source=True,files={k:v['metadata']for k,v in row.items()},object_message=obj,intrinsics=cam)))
  masks.append(box._array_hash((row['mask_uint8']['value']>0).astype(np.uint8)))
  arrays[ns]={key:row[key]['value']for key in image_topics.values()}
 final=arrays[STAMPS[-1]]
 assert all(expected['retained_pixel_sha256'][key+'_sha256']==box._array_hash(final[array])for key,array in [('color','color_bgr'),('depth','depth_raw'),('mask','mask_uint8')])
 assert masks[-1]==expected['final_sample_binary_mask_sha256']
 import cv2
 png=OUT/'request16_final_original_color.png'
 assert cv2.imwrite(str(png),final['color_bgr'])
 assert np.array_equal(cv2.imread(str(png),cv2.IMREAD_COLOR),final['color_bgr'])
 png_ref=put(OUT/'request16_final_original_color_metadata.json',dict(source_stamp_ns=STAMPS[-1],original_decoded_bgr_array_sha256=box._array_hash(final['color_bgr']),png_path=str(png),png_sha256=sha(png),decoded_pixels_exactly_equal=True,resampled=False,overlay_added=False))
 meta_ref=put(OUT/'camera_metadata.json',dict(intrinsics=cam,parameter_evidence=camera_metadata,camera_info=camera_info,parameters_are_not_image_synchronized=True,all_recorded_numeric_intrinsics_and_scales_consistent=True,camera_info_crosscheck_present=bool(camera_info)))
 identity=TargetTrackIdentity(report['target_epoch'],expected['source_target_identity']['track_id']);ns=STAMPS[-1]
 object_msg=records[ns]['object_message']['value'];bbox=tuple(object_msg[k]for k in ('bbox_x','bbox_y','bbox_width','bbox_height'))
 sample=FrozenRgbdSample(final['color_bgr'],final['depth_raw'],final['mask_uint8'],bbox,object_msg,report['snapshot_stamp_sec'],records[ns]['color_bgr']['metadata']['frame_id'],None,ns,identity.epoch,identity)
 snapshot=SnapshotResult(True,'','',final['color_bgr'],final['depth_raw'],final['depth_raw'],final['mask_uint8'],bbox,object_msg,report['snapshot_stamp_sec'],sample.frame_id,None,'instance_mask',ns,identity.epoch,identity,STAMPS,sample,tuple(masks))
 assert box._retained_box_pixels(snapshot,identity,ns)[0]is sample
 g=report['replay_geometry'];geometry=GeometryEstimate(True,'','',g['obb_center_base_m'],g['R_base_obb'],g['obb_size_xyz_m'],g['support_normal_base'],g['support_offset_m'],g['support_inlier_ratio'],g['object_points_base_m'],g['source_mode'])
 assert len(geometry.object_points_base)==g['object_points_count']
 T=np.asarray(report['snapshot_transform']['T_base_optical']);trace_rows=[]
 code=cuboid._assignment_stability.__code__
 def trace(frame,event,arg):
  if frame.f_code is code and event in ('call','exception','return'):
   names=('A','lo','hi','h','d','T','axis','yaw','hu','mid','size','camera','j','low','high','uncertainty','states')
   row=dict(event=event,locals={key:cuboid._plain(frame.f_locals[key])for key in names if key in frame.f_locals})
   parent=frame.f_back
   if parent is not None and parent.f_code.co_name=='infer_cuboid_rgb_interval':
    row['infer_locals']={key:cuboid._plain(parent.f_locals[key])for key in ('groups','seed_groups','result','nominal_A','nominal_lo','nominal_hi','top_height','hu','n','support_offset','T','K','origin','tn','slab','pixel_size','edge','yaw','axis','model')if key in parent.f_locals}
   if event=='exception':row.update(exception_type=arg[0].__name__,exception_reason=str(arg[1]))
   if event=='return':row['returned']=cuboid._plain(arg)
   trace_rows.append(row)
  return trace
 print(json.dumps(dict(stage='exact_three_sources_bound',source_stamp_ns=STAMPS,intrinsics=cam)),flush=True)
 previous=sys.gettrace();started=time.monotonic()
 try:
  sys.settrace(trace)
  qualification=box.qualify_regular_box_snapshot(snapshot,intrinsics,intrinsics.depth_scale,T,geometry,identity,ns)
 finally:sys.settrace(previous)
 elapsed=time.monotonic()-started
 actual=cuboid._plain(dict(qualification.audit))
 assert qualification.ok is False and qualification.code==expected['code']
 assert actual['cuboid_qualification_reason']==expected['cuboid_qualification_reason']=='common cuboid family has unresolved top/support silhouette visibility'
 assert actual['retained_pixel_sha256']==expected['retained_pixel_sha256']
 put(OUT/'assignment_trace_before_exact_comparison.json',dict(events=trace_rows,original_gate_modified=False))
 put(OUT/'qualification_before_exact_comparison.json',dict(ok=qualification.ok,code=qualification.code,reason=qualification.audit.get('reason',''),audit=actual,expected_audit=expected,wall_sec=elapsed))
 actual_c=actual['cuboid_rgb_interval'];expected_c=expected['cuboid_rgb_interval'];assert actual_c==expected_c,(set(actual_c),set(expected_c))
 errors=[row for row in trace_rows if row['event']=='exception'];assert len(errors)==1 and errors[0]['exception_reason']==expected['cuboid_qualification_reason']
 state=errors[0]['locals'];half=np.asarray(state['size'])/2.;camera=np.asarray(state['camera']);axis=np.asarray(state['axis']);j=state['j'];lo=state['low'];hi=state['high'];u=state['uncertainty']
 visibility=dict(failed_axis_index=j,nominal_camera_xy_m=camera.tolist(),half_size_xy_m=half.tolist(),axis_budget_xy_m=axis.tolist(),yaw_budget_rad=state['yaw'],yaw_budget_deg=state['yaw']*180/np.pi,height_budget_m=state['hu'],camera_rotated_interval_m=[lo,hi],rotated_axis_budget_m=u,outside_positive_margin_m=lo-half[j]-u,outside_negative_margin_m=-half[j]-u-hi,inside_low_margin_m=lo+half[j]-u,inside_high_margin_m=half[j]-u-hi,nominal_camera_distance_to_positive_face_m=float(camera[j]-half[j]),nominal_camera_distance_to_negative_face_m=float(camera[j]+half[j]),interpretation='Original independent enclosure predicate fails. This trace does not establish whether the exact coupled raster-strip family crosses visibility; root may analyze archived groups/result without changing this original failed result.')
 trace_ref=put(OUT/'assignment_stability_trace.json',dict(events=trace_rows,original_gate_modified=False,visibility=visibility))
 qual_ref=put(OUT/'qualification_replay.json',dict(ok=qualification.ok,code=qualification.code,reason=qualification.audit.get('reason',''),audit=actual,original_audit_cuboid_exactly_equal=True,wall_sec=elapsed))
 assert not BOUNDARY and all(sha(W/rel)==h for rel,h in source_map.items())and sha(bag_path)==bagref['sha256']
 result=dict(kind='gen48_exact_g47_box_visibility_original_qualification_probe',passed=True,passed_meaning='Exact source hashes and original failure reproduced; no qualified box or motion authority',original_qualification_ok=False,original_qualification_code=qualification.code,original_cuboid_failure=actual['cuboid_qualification_reason'],original_cuboid_audit_exactly_equal=True,source_stamp_ns=STAMPS,source_count=1052,source_unchanged=True,bag_manifest_verified=True,bag_sha256=bagref['sha256'],array_metadata=metadata_refs,original_color_png=png_ref,camera_metadata=meta_ref,qualification=qual_ref,visibility_trace=trace_ref,visibility=visibility,source_sha256=source_map,input_evidence_sha256=evidence,read_topics=topics,boundary_attempts=BOUNDARY,ROS_init_called=False,hardware_contacted=False,temperature_queried=False,gate_or_model_or_calibration_changed=False,execution_snapshot_recreated=False,temporal_median_depth_recreated=False,physical_grasp_verified=False)
 rr=put(OUT/'result.json',result);ss=put(OUT/'seal.json',dict(result=rr,qualification=qual_ref,trace=trace_ref,helper_sha256=sha(__file__),input_evidence_sha256=evidence))
 print(json.dumps(dict(result=rr,seal=ss,visibility=visibility),ensure_ascii=False),flush=True)

if __name__=='__main__':
 try:main()
 except BaseException as error:
  if OUT.is_dir()and not(OUT/'result.json').exists():
   put(OUT/'partial_failure.json',dict(kind='gen48_probe_partial_not_qualification',error_type=type(error).__name__,error=str(error),traceback=traceback.format_exc(),boundary_attempts=BOUNDARY,hardware_contacted=False,source_changed=False))
  raise
