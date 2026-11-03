#!/usr/bin/env python3
"""Exact closed G48 R22 RGBD -> unchanged GEN48 box qualifier and unknown-cell ray trace.

No gate is patched. The temporary snapshot is a qualification-only carrier of
original retained pixels and admitted replay geometry, not an execution snapshot.
"""
import hashlib,io,json,os,sys,time,traceback,platform,struct
from pathlib import Path
from types import SimpleNamespace

W=Path(__file__).resolve().parents[1]
SOURCE_ROOT=W/'.runtime/gen49_baseline_source'
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N=E/'ros_log/full_latest_gen41_restart_20261018T011222Z'
V=N/'verification'; OUT=V/'gen49_g48_exact_box_unknown_probe_r1'/'attempt_r1'
G=E/'grasp_records/20261018T095014.238124Z_ea3e1758e151468c9c945b7e94a07b1d'
STAMPS=(1792317082624930858,1792317083490865230,1792317083889718770)
AUDIT=V/'gen48_actual_handoff_monitor_r1/request22_atomic_audit_e7886527.json'
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
 OUT.mkdir(parents=True);(OUT/'tmp').mkdir();(OUT/'ros').mkdir()
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
 baseline=SOURCE_ROOT/'manifest.json'
 assert sha(baseline)=='21f6c6a1b0b8072d78332a081af1a425c570c9e5dd1983b2f982960e3911e935'
 sys.path.insert(0,str(SOURCE_ROOT/'src/alicia_flexible_grasp_supervisor/src'))
 from alicia_flexible_grasp.vision import regular_box_inference as box
 from alicia_flexible_grasp.vision import cuboid_rgb_interval as cuboid
 from alicia_flexible_grasp.vision.object_geometry import GeometryEstimate
 from alicia_flexible_grasp.vision.rgbd_snapshot import FrozenRgbdSample,SnapshotResult
 from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
 release_path=V/'gen48_release_manifest.json';release=json.loads(release_path.read_text());assert release['passed']and release['reviewed']and release['source_count']==1053
 source_map=release['source_sha256'];assert all(sha(SOURCE_ROOT/rel)==h for rel,h in source_map.items())
 evidence={str(release_path):sha(release_path),str(AUDIT):sha(AUDIT),str(G/'manifest.json'):sha(G/'manifest.json'),str(Path(__file__)):sha(Path(__file__))}
 assert evidence[str(AUDIT)]=='e788652700db6369e882e4bf93d35191fff4d04fdeaa4d4785677359e59a8438'
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
 png=OUT/'request22_final_original_color.png'
 assert cv2.imwrite(str(png),final['color_bgr'])
 assert np.array_equal(cv2.imread(str(png),cv2.IMREAD_COLOR),final['color_bgr'])
 png_ref=put(OUT/'request22_final_original_color_metadata.json',dict(source_stamp_ns=STAMPS[-1],original_decoded_bgr_array_sha256=box._array_hash(final['color_bgr']),png_path=str(png),png_sha256=sha(png),decoded_pixels_exactly_equal=True,resampled=False,overlay_added=False))
 meta_ref=put(OUT/'camera_metadata.json',dict(intrinsics=cam,parameter_evidence=camera_metadata,camera_info=camera_info,parameters_are_not_image_synchronized=True,all_recorded_numeric_intrinsics_and_scales_consistent=True,camera_info_crosscheck_present=bool(camera_info)))
 identity=TargetTrackIdentity(report['target_epoch'],expected['source_target_identity']['track_id']);ns=STAMPS[-1]
 object_msg=records[ns]['object_message']['value'];bbox=tuple(object_msg[k]for k in ('bbox_x','bbox_y','bbox_width','bbox_height'))
 sample=FrozenRgbdSample(final['color_bgr'],final['depth_raw'],final['mask_uint8'],bbox,object_msg,report['snapshot_stamp_sec'],records[ns]['color_bgr']['metadata']['frame_id'],None,ns,identity.epoch,identity)
 snapshot=SnapshotResult(True,'','',final['color_bgr'],final['depth_raw'],final['depth_raw'],final['mask_uint8'],bbox,object_msg,report['snapshot_stamp_sec'],sample.frame_id,None,'instance_mask',ns,identity.epoch,identity,STAMPS,sample,tuple(masks))
 assert box._retained_box_pixels(snapshot,identity,ns)[0]is sample
 g=report['replay_geometry'];geometry=GeometryEstimate(True,'','',g['obb_center_base_m'],g['R_base_obb'],g['obb_size_xyz_m'],g['support_normal_base'],g['support_offset_m'],g['support_inlier_ratio'],g['object_points_base_m'],g['source_mode'])
 assert len(geometry.object_points_base)==g['object_points_count']
 T=np.asarray(report['snapshot_transform']['T_base_optical']); trace_rows=[]
 code=cuboid._unknown_mask_rays.__code__
 argument_names=('pixels_yx','T','K','A','lo','hi','h','d','edge','yaw','hu')
 def trace(frame,event,arg):
  if frame.f_code is code and event=='return':
   row=dict(arguments={k:cuboid._plain(frame.f_locals[k])for k in argument_names},returned=cuboid._plain(arg),internal={k:cuboid._plain(frame.f_locals[k])for k in ('half','camera','center','size','mid')})
   parent=frame.f_back
   if parent is not None and parent.f_code.co_name=='infer_cuboid_rgb_interval':
    row['infer_locals']={k:cuboid._plain(parent.f_locals[k])for k in ('groups','seed_groups','result','nominal_A','nominal_lo','nominal_hi','top_height','hu','origin','tn','slab','pixel_size','edge','yaw','axis','visibility_family','model')if k in parent.f_locals}
   trace_rows.append(row)
  return trace
 print(json.dumps(dict(stage='exact_three_sources_bound',source_stamp_ns=STAMPS,original_color=str(png)),ensure_ascii=False),flush=True)
 previous=sys.gettrace();started=time.monotonic()
 try:
  sys.settrace(trace)
  qualification=box.qualify_regular_box_snapshot(snapshot,intrinsics,intrinsics.depth_scale,T,geometry,identity,ns)
 finally:sys.settrace(previous)
 elapsed=time.monotonic()-started
 actual=cuboid._plain(dict(qualification.audit))
 qual_ref=put(OUT/'qualification_replay.json',dict(ok=qualification.ok,code=qualification.code,audit=actual,expected_audit=expected,wall_sec=elapsed))
 assert qualification.ok is False and qualification.code==expected['code']
 assert actual['cuboid_qualification_reason']==expected['cuboid_qualification_reason']=='unknown mask band lies outside complete cuboid uncertainty family'
 assert actual['retained_pixel_sha256']==expected['retained_pixel_sha256'] and len(trace_rows)==1
 current=actual['cuboid_rgb_interval'];original=expected['cuboid_rgb_interval']; assert current['unknown_mask_pixel_count']==original['unknown_mask_pixel_count']==3994
 assert current['unexplained_unknown_mask_pixel_count']==original['unexplained_unknown_mask_pixel_count']==11
 assert current['unknown_mask_pixels_yx']==original['unknown_mask_pixels_yx'] and current['unknown_mask_pixels_sha256']==original['unknown_mask_pixels_sha256']
 captured=trace_rows[0];args=captured['arguments'];pixels=np.asarray(args['pixels_yx'],int);xy=pixels[:,::-1].astype(float)
 A=np.asarray(args['A']);T=np.asarray(args['T']);K=np.asarray(args['K']);half=np.asarray(captured['internal']['half']);camera=np.asarray(captured['internal']['camera']);center=np.asarray(captured['internal']['center'])
 shifts=((0.,0.),(-.5,-.5),(-.5,.5),(.5,-.5),(.5,.5));ray_records=[];flags=[]
 from itertools import product
 vertices=center+np.asarray(list(product((-1.,1.),repeat=3)))*half@A.T
 projected=cuboid._project(vertices,T,K);indices=cv2.convexHull(projected.astype(np.float32),returnPoints=False).reshape(-1);hull=projected[indices]
 def boundary_distance(point):
  start=hull;end=np.roll(hull,-1,axis=0);delta=end-start;v=point-start;t=np.clip(np.sum(v*delta,axis=1)/np.sum(delta*delta,axis=1),0.,1.)
  distance=float(np.sqrt(np.min(np.sum((v-t[:,None]*delta)**2,axis=1))));cross=delta[:,0]*v[:,1]-delta[:,1]*v[:,0];inside=bool(np.all(cross>=-1e-12)or np.all(cross<=1e-12))
  return distance if inside else -distance
 for shift in shifts:
  rays=cuboid._rays(xy+shift,T,K)@A
  with np.errstate(divide='ignore',invalid='ignore'):t0,t1=(-half-camera)/rays,(half-camera)/rays
  near=np.maximum(np.minimum(t0,t1).max(1),0.);far=np.maximum(t0,t1).min(1);hit=np.isfinite(near)&(far>=near);flags.append(hit)
  ray_records.append(dict(shift_xy=shift,compatible=hit,near=near,far=far,rays_local=rays))
 flags=np.asarray(flags).T;and_hits=np.all(flags,axis=1);assert np.array_equal(and_hits,np.asarray(captured['returned'],bool));failed=np.where(~and_hits)[0];assert len(failed)==11
 rejected=[]
 for idx in failed:
  y,x=map(int,pixels[idx]);neighborhoods=[]
  for source_ns,source in arrays.items():
   y0,y1=max(0,y-4),min(source['depth_raw'].shape[0],y+5);x0,x1=max(0,x-4),min(source['depth_raw'].shape[1],x+5);crop=source['color_bgr'][y0:y1,x0:x1]
   cp=OUT/'inputs'/str(source_ns)/('rejected_y%d_x%d_original_crop.png'%(y,x));assert cv2.imwrite(str(cp),crop)
   neighborhoods.append(dict(source_stamp_ns=source_ns,coordinate_yx=[y,x],same_image_coordinate_not_cross_frame_pixel_registration=True,original_bgr=source['color_bgr'][y,x].tolist(),original_depth_raw=int(source['depth_raw'][y,x]),original_mask_raw=int(source['mask_uint8'][y,x]),window_yx=[y0,y1,x0,x1],depth_raw=source['depth_raw'][y0:y1,x0:x1].tolist(),mask_raw=source['mask_uint8'][y0:y1,x0:x1].tolist(),bgr=source['color_bgr'][y0:y1,x0:x1].tolist(),color_crop_png=str(cp),color_crop_sha256=sha(cp)))
  details=[]
  for j,row in enumerate(ray_records):
   point=xy[idx]+shifts[j];details.append(dict(shift_xy=shifts[j],pixel_xy=point,compatible=bool(flags[idx,j]),near_optical_depth_m=float(row['near'][idx]),far_optical_depth_m=float(row['far'][idx]),near_minus_far_optical_depth_m=float(row['near'][idx]-row['far'][idx]),signed_projected_enclosure_boundary_distance_px=boundary_distance(point),ray_direction_in_enclosure_axes=row['rays_local'][idx]))
  rejected.append(dict(unknown_array_index=int(idx),source_stamp_ns=ns,pixel_yx=[y,x],original_depth_raw=int(final['depth_raw'][y,x]),original_mask_raw=int(final['mask_uint8'][y,x]),original_bgr=final['color_bgr'][y,x].tolist(),center_compatible=bool(flags[idx,0]),all_four_cell_corners_compatible=bool(np.all(flags[idx,1:])),all_five_compatible=bool(and_hits[idx]),rays=details,original_neighborhoods=neighborhoods))
 ray_ref=put(OUT/'unknown_pixel_ray_details.json',cuboid._plain(dict(source_stamp_ns=ns,unknown_count=len(pixels),rejected_count=len(failed),center_rejected_count=int(np.count_nonzero(~flags[:,0])),center_hits_but_some_corner_misses_count=int(np.count_nonzero(flags[:,0]&~and_hits)),any_five_ray_hits_but_not_all_count=int(np.count_nonzero(np.any(flags,axis=1)&~and_hits)),all_five_miss_count=int(np.count_nonzero(~np.any(flags,axis=1))),rejected_pixels=rejected,projected_enclosure_hull_xy=hull,axis_half_extents_m=half,enclosure_center_base_m=center,distance_semantics='Signed Euclidean image distance to projection of the exact enlarged OBB used by _unknown_mask_rays; positive inside. near-minus-far is a slab optical-depth parameter gap, not metric perpendicular point distance.',unknown_depth_cause='Direct original final-frame uint16 depth values are zero at the rejected mask pixels; the record does not establish the sensor/segmentation physical cause. Other-source neighborhoods are original values at the same image coordinates, not registered pixel identities.')))
 trace_ref=put(OUT/'unknown_ray_trace.json',dict(events=trace_rows,gate_modified=False))
 for source_ns,source in arrays.items():
  p=OUT/'inputs'/str(source_ns)/'original_color.png';assert cv2.imwrite(str(p),source['color_bgr']);assert np.array_equal(cv2.imread(str(p)),source['color_bgr'])
 print(json.dumps(dict(stage='reproduced_unknown_gate',unknown_count=3994,rejected_count=11,center_rejected_count=int(np.count_nonzero(~flags[:,0])),corner_only_rejected_count=int(np.count_nonzero(flags[:,0]&~and_hits)),pixels=pixels[failed].tolist(),ray_details=ray_ref,original_color=str(png)),ensure_ascii=False),flush=True)
 differences=[]
 def compare(x,y,path=''):
  if isinstance(x,dict)and isinstance(y,dict):
   if set(x)!=set(y):differences.append(dict(path=path,kind='keys',actual_keys=sorted(x),expected_keys=sorted(y)))
   for k in x.keys()&y.keys():compare(x[k],y[k],path+'/'+k)
  elif isinstance(x,list)and isinstance(y,list):
   if len(x)!=len(y):differences.append(dict(path=path,kind='length',actual=len(x),expected=len(y)))
   for i,(a,b)in enumerate(zip(x,y)):compare(a,b,path+'/'+str(i))
  elif type(x)in(int,float)and type(y)in(int,float)and x!=y:differences.append(dict(path=path,kind='numeric',actual=x,expected=y,absolute_difference=abs(x-y)))
  elif x!=y:differences.append(dict(path=path,kind='value',actual=x,expected=y))
 compare(actual,expected)
 diff_ref=put(OUT/'audit_comparison.json',dict(bit_exact=not differences,differences=differences,comparison_tolerance_policy='No production threshold changed. Differences are reported rather than hidden; replay conclusion requires identical original code/reason/source hashes/counts and full unknown pixel identity.'))
 assert all(row['kind']=='numeric'and row['absolute_difference']<=1e-9 for row in differences),differences[:5]
 assert not BOUNDARY and all(sha(SOURCE_ROOT/rel)==h for rel,h in source_map.items())and sha(bag_path)==bagref['sha256']
 evidence[str(baseline)]=sha(baseline)
 result=dict(kind='gen49_exact_G48_request22_original_unknown_pixel_probe',passed=True,passed_meaning='Exact original source pixels and unchanged GEN48 qualification failure reproduced; this grants no model or motion acceptance',request_id=22,selection_generation=8,remote_generation=7,target_epoch=4,original_qualification_ok=False,original_qualification_code=qualification.code,original_cuboid_failure=actual['cuboid_qualification_reason'],original_audit_bit_exact=not differences,audit_comparison=diff_ref,source_stamp_ns=STAMPS,source_count=1053,source_root=str(SOURCE_ROOT),source_unchanged=True,source_sha256=source_map,bag_manifest_verified=True,bag_sha256=bagref['sha256'],array_metadata=metadata_refs,original_color_png=png_ref,camera_metadata=meta_ref,camera_parameters_are_not_exposure_synchronized=True,qualification=qual_ref,unknown_ray_trace=trace_ref,unknown_pixel_details=ray_ref,unknown_pixel_count=3994,rejected_count=11,center_rejected_count=int(np.count_nonzero(~flags[:,0])),corner_only_rejected_count=int(np.count_nonzero(flags[:,0]&~and_hits)),rejected_pixels_yx=pixels[failed].tolist(),input_evidence_sha256=evidence,read_topics=topics,boundary_attempts=BOUNDARY,ROS_init_called=False,hardware_contacted=False,temperature_queried=False,gate_or_model_or_calibration_changed=False,execution_snapshot_recreated=False,temporal_median_depth_recreated=False,physical_grasp_verified=False)
 rr=put(OUT/'result.json',result);seal=put(OUT/'seal.json',dict(result=rr,helper_sha256=sha(__file__),input_evidence_sha256=evidence,trace=trace_ref,qualification=qual_ref,unknown_pixels=ray_ref))
 print(json.dumps(dict(result=rr,seal=seal),ensure_ascii=False),flush=True)

if __name__=='__main__':
 try:main()
 except BaseException as error:
  if OUT.is_dir()and not(OUT/'result.json').exists():
   put(OUT/'partial_failure.json',dict(kind='gen49_probe_partial_not_completed',error_type=type(error).__name__,error=str(error),traceback=traceback.format_exc(),boundary_attempts=BOUNDARY,hardware_contacted=False,source_changed=False))
  raise
